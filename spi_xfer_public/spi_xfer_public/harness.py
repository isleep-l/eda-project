#!/usr/bin/env python3
"""
spi_xfer_public - harness.py
Black-box eval loop for the SPI / SSP master serial transfer controller.
Mirrors the committee eval (sim adapter + eval_secrets): decodes the 12-dim
action into register writes / serial stimulus, injects hidden parameters
(SPEC_DFS_MIN, SPEC_HOLD_SS) into local_sim, samples the coverage curve
every `interval` cycles.

Action layout (12 dims, matches dut_spec.md §6):
  [reg_we, reg_addr, reg_wdata, reg_re, rxd, ss_in_n, rst_n, pad0..pad4]
  - reg_we     (bool)         : register write strobe
  - reg_addr   (0..15)        : 4-bit word address (see dut_spec.md reg map)
  - reg_wdata  (32-bit)       : register write data (DR write = TX FIFO push)
  - reg_re     (bool)         : register read strobe (addr=DR pops the RX FIFO)
  - rxd        (bool)         : serial receive data input
  - ss_in_n    (bool, 1=high) : slave-select input (SPI: must be 1 during
                                transfer, SSP: must be 0 during transfer)
  - rst_n      (bool, 1=run)  : async active-low reset
  - pad0..pad4                : reserved (ignored)
"""

import json
import os
from typing import Dict, List, Optional, Tuple

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def default_secrets() -> Dict:
    return {
        "spec_dfs_min": 4,   # minimum effective frame size (clamp)
        "spec_hold_ss": 4,   # HOLD_SS dwell duration, half clock periods
    }


def load_secrets(path: Optional[str] = None) -> Dict:
    if path is None:
        cands = [
            os.path.join(HERE, "..", "..", "committee_eval_private", "eval_secrets_spi_xfer.json"),
            os.path.join(HERE, "..", "..", "..", "发布包_v1.4", "committee_private",
                         "committee_eval_private", "eval_secrets_spi_xfer.json"),
        ]
        for c in cands:
            if os.path.exists(c):
                path = c
                break
    if path and os.path.exists(path):
        return json.load(open(path, encoding="utf-8"))
    return default_secrets()


class SpiXferHarness:
    DIMS = 12

    def __init__(self, secrets: Optional[Dict] = None, backend: str = "verilator"):
        import importlib.util
        ls = os.path.join(HERE, "local_sim.py")
        spec = importlib.util.spec_from_file_location("spi_xfer_ls", ls)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        self._ls = mod
        self.secrets = secrets or load_secrets()
        # backend: "verilator" (default, real RTL simulation data) or "local"
        self.backend = backend
        self._dut = self._make_dut()
        self._cs = self._mk_cov()

    def _make_dut(self):
        if self.backend == "local":
            return self._ls.SpiXferPublic(
                spec_dfs_min=self.secrets.get("spec_dfs_min"),
                spec_hold_ss=self.secrets.get("spec_hold_ss"),
            )
        return self._mk_verilator_dut()

    def _mk_verilator_dut(self):
        import importlib.util
        vd = os.path.join(HERE, "verilator_harness", "verilator_dut.py")
        spec = importlib.util.spec_from_file_location("spi_xfer_vd", vd)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.VerilatorDut(
            spec_dfs_min=self.secrets.get("spec_dfs_min"),
            spec_hold_ss=self.secrets.get("spec_hold_ss"),
        )

    def _mk_cov(self):
        import importlib.util
        cs = os.path.join(HERE, "coverage_simulator.py")
        spec = importlib.util.spec_from_file_location("spi_xfer_cs", cs)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        self._cs_mod = mod
        # spec_dfs_min drives the dfs_effective_min hidden-clamp bins
        return mod.CoverageSimulator(os.path.join(HERE, "dut", "coverage_meta.json"),
                                     spec_dfs_min=self.secrets.get("spec_dfs_min"))

    def reset(self) -> np.ndarray:
        old = getattr(self, "_dut", None)
        self._dut = self._make_dut()
        if old is not None and hasattr(old, "close"):
            old.close()
        self._cs.reset()
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

        reg_we   = a[0] & 1
        reg_addr = a[1] & 0xF
        wdata    = int(a[2]) & 0xFFFFFFFF
        reg_re   = a[3] & 1
        rxd      = a[4] & 1
        ss_in_n  = a[5] & 1
        rst_n    = a[6] & 1

        self._dut.step({
            "reg_we": reg_we,
            "reg_addr": reg_addr,
            "reg_wdata": wdata,
            "reg_re": reg_re,
            "rxd": rxd,
            "ss_in_n": ss_in_n,
            "rst_n": rst_n,
        })
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
    h = SpiXferHarness(secrets, backend=backend)
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
        """Fair-random baseline: reset once at the start (rst_n=0 for a few
        cycles), then hold rst_n=1 for the whole episode so the DUT actually
        runs; all other dims uniform(0, 256)."""

        RST_CYCLES = 64
        RST_IDX = 6  # index of rst_n in the 12-dim action

        def __init__(self, seed=7):
            self.rng = np.random.RandomState(seed)
        def predict(self, coverage_state, step, max_steps):
            a = self.rng.uniform(0, 256, SpiXferHarness.DIMS).astype(np.float32)
            a[self.RST_IDX] = 0.0 if step < self.RST_CYCLES else 1.0
            return a

    curve = run_episode(RandAgent(), max_steps=6000, interval=1000)
    print("fair-random curve:", curve)
