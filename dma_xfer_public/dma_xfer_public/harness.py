#!/usr/bin/env python3
"""
dma_xfer_public - harness.py
Black-box eval loop for the DMA controller. Mirrors the committee eval
(sim adapter + eval_secrets): decodes the 15-dim action, injects hidden
parameters into local_sim, samples the coverage curve every `interval` cycles.

Action layout (15 dims):
  [ch_sel, conf_wr, conf_field, d0, d1, d2, d3, start, pad0..pad6]
  - ch_sel (0..3)      : channel to configure/start
  - conf_wr (bool)     : perform a register write this cycle (start must be 0)
  - conf_field (0..2)  : 0=saddr 1=daddr 2={burst,dmode,len}
  - d0..d3 (bytes)     : 32-bit register data (little endian)
  - start (bool)       : rising edge starts the selected channel
"""

import json
import os
from typing import Dict, List, Optional, Tuple

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def _word4(byte_list) -> int:
    a = [int(x) & 0xFF for x in byte_list]
    return (a[3] << 24) | (a[2] << 16) | (a[1] << 8) | a[0]


def default_secrets() -> Dict:
    return {
        "arb_prio": [0, 1, 2, 3],
        "done_ok_modes": [20, 3, 21, 7],  # public placeholder (not used at eval)
        "xfer_hold_n": 16,
    }


def load_secrets(path: Optional[str] = None) -> Dict:
    if path is None:
        cands = [
            os.path.join(HERE, "..", "..", "committee_eval_private", "eval_secrets_dma.json"),
            os.path.join(HERE, "..", "..", "..", "发布包_v1.4", "committee_private",
                         "committee_eval_private", "eval_secrets_dma.json"),
        ]
        for c in cands:
            if os.path.exists(c):
                path = c
                break
    if path and os.path.exists(path):
        return json.load(open(path, encoding="utf-8"))
    return default_secrets()


class DmaXferHarness:
    DIMS = 15

    def __init__(self, secrets: Optional[Dict] = None, backend: str = "verilator"):
        import importlib.util
        ls = os.path.join(HERE, "local_sim.py")
        spec = importlib.util.spec_from_file_location("dma_ls", ls)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        self._ls = mod
        self.secrets = secrets or load_secrets()
        # backend: "verilator" (default, real RTL simulation data) or "local"
        self.backend = backend
        self._dut = self._make_dut()
        self._cs = self._mk_cov()
        self._prev_start_mask = 0

    def _make_dut(self):
        if self.backend == "local":
            return self._ls.DmaXferPublic(
                arb_prio=self.secrets.get("arb_prio"),
                done_ok_modes=self.secrets.get("done_ok_modes"),
                xfer_hold_n=self.secrets.get("xfer_hold_n"),
            )
        return self._mk_verilator_dut()

    def _mk_verilator_dut(self):
        import importlib.util
        vd = os.path.join(HERE, "verilator_harness", "verilator_dut.py")
        spec = importlib.util.spec_from_file_location("dma_vd", vd)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.VerilatorDut(
            arb_prio=self.secrets.get("arb_prio"),
            done_ok_modes=self.secrets.get("done_ok_modes"),
            xfer_hold_n=self.secrets.get("xfer_hold_n"),
        )

    def _mk_cov(self):
        import importlib.util
        cs = os.path.join(HERE, "coverage_simulator.py")
        spec = importlib.util.spec_from_file_location("dma_cs", cs)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        self._cs_mod = mod
        return mod.CoverageSimulator(os.path.join(HERE, "dut", "coverage_meta.json"))

    def reset(self) -> np.ndarray:
        old = getattr(self, "_dut", None)
        self._dut = self._make_dut()
        if old is not None and hasattr(old, "close"):
            old.close()
        self._cs.reset()
        self._prev_start_mask = 0
        return self._state()

    def _state(self) -> np.ndarray:
        return np.asarray(self._cs.get_bin_vector(), dtype=np.float32)

    @property
    def total_bins(self):
        return self._cs.total_bins

    @property
    def coverage(self):
        return self._cs.coverage

    def step(self, stimulus: np.ndarray):
        action = np.asarray(stimulus, dtype=np.float32).reshape(-1)
        a = [int(x) for x in action]
        ch = a[0] & 0xFF
        ch %= 4
        conf_wr = a[1]
        field = a[2] % 3
        data = _word4(a[3:7])
        start_bit = a[7]

        # per-channel strict start-edge detection (bitmask)
        cur_mask = 0
        if start_bit:
            cur_mask |= (1 << ch)
        new_rises = cur_mask & ~self._prev_start_mask
        self._prev_start_mask = cur_mask

        # Configuration write (only when start deasserted this cycle)
        if conf_wr and not start_bit:
            self._dut.apply_config(ch, field, data)

        # Start (rising edge per channel)
        if new_rises:
            self._dut.tick_start(new_rises)
        else:
            self._dut.tick_start(0)

        self._dut.step()
        self._cs.step(self._dut.read_signals())
        state = self._state()
        reward = float(np.sum(state)) / self.total_bins
        info = {"total_coverage": self._cs.coverage * 100.0}
        return state, reward, False, info


def _save_curve(curve, max_steps, interval):
    """Append this run's coverage curve to coverage_curves.json (one file per problem).

    Each call to run_episode (one simulation) records an entry so all runs are
    preserved for later AUC / convergence analysis.
    """
    import time
    path = os.path.join(HERE, "coverage_curves.json")
    record = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "max_steps": max_steps,
        "interval": interval,
        "curve": [[int(s), float(c)] for s, c in curve],
    }
    data = []
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            data = []
    data.append(record)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)


def run_episode(agent, secrets=None, max_steps=20000, interval=1000,
                backend="verilator"):
    h = DmaXferHarness(secrets, backend=backend)
    state = h.reset()
    curve = []
    for step in range(max_steps):
        action = np.asarray(agent.predict(state, step, max_steps), dtype=np.float32).reshape(-1)
        if action.size < h.DIMS:
            action = np.concatenate([action, np.zeros(h.DIMS - action.size, dtype=np.float32)])
        state, _r, _d, _info = h.step(action)
        if step % interval == 0:
            curve.append((step, h.coverage))
    curve.append((max_steps - 1, h.coverage))
    _save_curve(curve, max_steps, interval)
    return curve


if __name__ == "__main__":
    import sys
    sys.path.insert(0, HERE)

    class RandAgent:
        def __init__(self, seed=7):
            self.rng = np.random.RandomState(seed)
        def predict(self, coverage_state, step, max_steps):
            return self.rng.uniform(0, 256, DmaXferHarness.DIMS).astype(np.float32)

    curve = run_episode(RandAgent(), max_steps=6000, interval=1000)
    print("random curve:", curve)
