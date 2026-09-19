#!/usr/bin/env python3
"""
dma_xfer_public - Verilator harness runner
Builds (if needed) and drives the Verilator-compiled dma_top, returning a
per-cycle signal dictionary compatible with coverage_simulator.py and
local_sim.py read_signals().

Action format (dicts, mirroring harness.py's 15-dim decode):
  ch_sel      (0..3)      : channel to configure / start
  conf_wr     (bool)      : perform a register write this cycle
  conf_field  (0..2)      : 0=saddr 1=daddr 2={burst,dmode,len}
  conf_data   (32-bit)    : register write data (see dut_spec.md field map)
  start       (bool)      : rising edge starts the selected channel
  rst_n       (bool, 1=run)
Fields not present default to 0 / 1 (rst_n=1) / re=0.
"""

import os
import subprocess
import sys
from typing import Dict, List, Optional

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OBJ_DIR = os.path.join(SCRIPT_DIR, "obj_dir")
EXE = os.path.join(OBJ_DIR, "Vdma_top")
BAKE_MARKER = os.path.join(OBJ_DIR, "baked_params.txt")  # records baked -G values

# Output columns produced by sim_main.cpp (header order)
COV_KEYS = [
    "cov_ch0_state", "cov_ch1_state", "cov_ch2_state", "cov_ch3_state",
    "cov_arb_winner", "cov_arb_valid", "cov_arb_conflict", "cov_active_ch",
    "cov_dir", "cov_burst", "cov_saddr", "cov_daddr", "cov_len", "cov_hold",
    "cov_done_ok_valid", "cov_done_class",
    "cov_seq1", "cov_seq2", "cov_seq3", "cov_seq4", "cov_seq5", "cov_seq6",
    "cov_seq7", "cov_seq8", "cov_seq9", "cov_seq10", "cov_seq11", "cov_seq12",
]

# Map the raw cov_* observation pixels emitted by sim_main.cpp to the derived
# signal schema that coverage_simulator.py / local_sim.read_signals() expect.
def _translate(sig: Dict) -> Dict:
    return {
        "ch0_state":  sig.get("cov_ch0_state", 0),
        "ch1_state":  sig.get("cov_ch1_state", 0),
        "ch2_state":  sig.get("cov_ch2_state", 0),
        "ch3_state":  sig.get("cov_ch3_state", 0),
        "arb_winner": sig.get("cov_arb_winner", 0),
        "arb_valid":  sig.get("cov_arb_valid", 0),
        "active_ch":  sig.get("cov_active_ch", 0),
        "dir_dir":    (sig.get("cov_dir", 0) >> 3) & 0x1,
        "cfg_burst":  sig.get("cov_burst", 0),
        "cfg_saddr":  sig.get("cov_saddr", 0),
        "cfg_daddr":  sig.get("cov_daddr", 0),
        "cfg_len":    sig.get("cov_len", 0),
        "arb_conflict": sig.get("cov_arb_conflict", 0),
        "hold":       sig.get("cov_hold", 0),
        "done_ok_valid": sig.get("cov_done_ok_valid", 0),
        "done_ok":    sig.get("cov_done_class", 0),
        "seq1":  sig.get("cov_seq1", 0),
        "seq2":  sig.get("cov_seq2", 0),
        "seq3":  sig.get("cov_seq3", 0),
        "seq4":  sig.get("cov_seq4", 0),
        "seq5":  sig.get("cov_seq5", 0),
        "seq6":  sig.get("cov_seq6", 0),
        "seq7":  sig.get("cov_seq7", 0),
        "seq8":  sig.get("cov_seq8", 0),
        "seq9":  sig.get("cov_seq9", 0),
        "seq10": sig.get("cov_seq10", 0),
        "seq11": sig.get("cov_seq11", 0),
        "seq12": sig.get("cov_seq12", 0),
    }



class VerilatorHarness:
    """Drives the Verilator-compiled dma_top via a text protocol.

    ARB_PRIO / DONE_OK_MODES / XFER_HOLD_N are baked into the model at BUILD
    time (Verilator >=5.020 does not expose top-level parameters as
    runtime-assignable members). If the requested params differ from the
    currently-baked model, a rebuild is triggered with those values via
    build.sh's environment variables. The defaults passed here are the public
    placeholders (ascending priority, inert done_ok, hold=16), matching
    local_sim.py's out-of-box semantics.
    """

    def __init__(self, arb_prio: int = 0x30, done_ok_modes: int = 0,
                 xfer_hold_n: int = 16):
        self.arb_prio = arb_prio & 0xFF
        self.done_ok_modes = done_ok_modes & 0xFFFF
        self.xfer_hold_n = xfer_hold_n & 0xFFFF
        self._ensure_built()

    def _ensure_built(self):
        baked = self._read_baked_params()
        needs_build = (not os.path.exists(EXE)
                       or baked != (self.arb_prio, self.done_ok_modes,
                                    self.xfer_hold_n))
        if needs_build:
            self._build()

    @staticmethod
    def _read_baked_params():
        try:
            with open(BAKE_MARKER, "r", encoding="utf-8") as f:
                line = f.read().strip()
            parts = [int(p, 0) for p in line.split()]
            if len(parts) == 3:
                return (parts[0], parts[1], parts[2])
        except Exception:
            pass
        return None

    def _build(self):
        build_sh = os.path.join(SCRIPT_DIR, "build.sh")
        env = dict(os.environ)
        env["ARB_PRIO"] = str(self.arb_prio)
        env["DONE_OK_MODES"] = str(self.done_ok_modes)
        env["XFER_HOLD_N"] = str(self.xfer_hold_n)
        print(f"Building Verilator model "
              f"(ARB_PRIO={self.arb_prio} DONE_OK_MODES={self.done_ok_modes} "
              f"XFER_HOLD_N={self.xfer_hold_n})...")
        r = subprocess.run(["bash", build_sh], capture_output=True, text=True,
                           env=env)
        if r.returncode != 0:
            raise RuntimeError(f"Verilator build failed:\n{r.stdout}\n{r.stderr}")
        if not os.path.exists(EXE):
            raise RuntimeError("Build reported success but executable not found")
        with open(BAKE_MARKER, "w", encoding="utf-8") as f:
            f.write(f"{self.arb_prio} {self.done_ok_modes} {self.xfer_hold_n}\n")

    def run_actions(self, actions: List[Dict]) -> List[Dict]:
        """Run a sequence of actions (dicts) and return per-cycle signals."""
        lines = []
        for a in actions:
            ch     = int(a.get("ch_sel", 0)) & 3
            wr     = int(a.get("conf_wr", 0)) & 1
            field  = int(a.get("conf_field", 0)) & 3
            data   = int(a.get("conf_data", 0)) & 0xFFFFFFFF
            start  = int(a.get("start", 0)) & 1
            rst    = int(a.get("rst_n", 1)) & 1
            d0 = (data >> 0)  & 0xFF
            d1 = (data >> 8)  & 0xFF
            d2 = (data >> 16) & 0xFF
            d3 = (data >> 24) & 0xFF
            lines.append(f"{ch} {wr} {field} {d0} {d1} {d2} {d3} {start} {rst}")

        p = subprocess.run(
            [EXE],
            input="\n".join(lines) + "\n",
            capture_output=True, text=True)
        if p.returncode != 0:
            raise RuntimeError(f"sim_main failed:\n{p.stderr}")

        out_lines = p.stdout.strip().split("\n")
        if not out_lines:
            return []
        header = [c.strip() for c in out_lines[0].split(",")]

        results = []
        for line in out_lines[1:]:
            vals = [int(v.strip()) for v in line.split(",")]
            row = {header[i]: vals[i] for i in range(min(len(header), len(vals)))}
            results.append(row)
        return results


def run_verilator_simulation(actions: List[Dict],
                             arb_prio: int = 0x30, done_ok_modes: int = 0,
                             xfer_hold_n: int = 16) -> Dict:
    """Run a full simulation and return per-cycle signals."""
    harness = VerilatorHarness(arb_prio, done_ok_modes, xfer_hold_n)
    signals = harness.run_actions(actions)
    return {
        "signals": signals,
        "total_cycles": len(signals),
    }


def run_and_save_curve(actions: List[Dict], interval: int = 1000,
                       arb_prio: int = 0x30, done_ok_modes: int = 0,
                       xfer_hold_n: int = 16,
                       out_file: Optional[str] = None) -> List[List]:
    """Run a real Verilator simulation, sample coverage every `interval`
    cycles, and append the curve to out_file (JSON).

    Mirrors harness.run_episode's curve format: list of [step, coverage] where
    coverage is fraction of bins hit in [0, 1]. A point is taken every
    `interval` cycles plus one final point at the last cycle. Each simulation
    is recorded (with timestamp) so all real-sim runs are preserved.
    """
    import time
    import json as _json

    sim_dir = os.path.join(SCRIPT_DIR, "..")
    sys.path.insert(0, sim_dir)
    from coverage_simulator import CoverageSimulator  # noqa: E402

    signals = run_verilator_simulation(actions, arb_prio, done_ok_modes,
                                       xfer_hold_n)["signals"]
    total = len(signals)
    if total == 0:
        return []

    cs = CoverageSimulator(os.path.join(sim_dir, "dut", "coverage_meta.json"))
    curve: List[List] = []
    for cyc, sig in enumerate(signals):
        cs.step(_translate(sig))
        if cyc % interval == 0:
            curve.append([int(cyc), float(cs.coverage)])
    curve.append([int(total - 1), float(cs.coverage)])

    if out_file is None:
        out_file = os.path.join(SCRIPT_DIR, "coverage_curves_verilator.json")
    record = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "simulator": "verilator",
        "arb_prio": arb_prio,
        "done_ok_modes": done_ok_modes,
        "xfer_hold_n": xfer_hold_n,
        "max_steps": total,
        "interval": interval,
        "curve": curve,
    }
    data = []
    if os.path.exists(out_file):
        try:
            with open(out_file, "r", encoding="utf-8") as f:
                data = _json.load(f)
        except Exception:
            data = []
    data.append(record)
    with open(out_file, "w", encoding="utf-8") as f:
        _json.dump(data, f, indent=2)

    return curve


if __name__ == "__main__":
    # Self-test: minimal transfer on channel 0 (len=2, dmode correct placeholder)
    # NOTE: with the public inert DONE_OK_MODES placeholder (0), done_ok stays
    # inert; this smoke test just verifies the transfer engine runs.
    A = [
        {"rst_n": 0},
        {"rst_n": 1},
        {"ch_sel": 0, "conf_wr": 1, "conf_field": 0, "conf_data": 0x1000},  # saddr
        {"ch_sel": 0, "conf_wr": 1, "conf_field": 1, "conf_data": 0x2000},  # daddr
        # field 2: {burst[23:21], dmode[20:16], len[15:0]}; dmode=0, len=4
        {"ch_sel": 0, "conf_wr": 1, "conf_field": 2, "conf_data": 0x0000_0004},
        {"ch_sel": 0, "start": 1},
    ]
    A += [{}] * 200

    res = run_verilator_simulation(A)
    active = [s for s in res["signals"] if s.get("cov_ch0_state") == 1]
    print(f"total cycles: {res['total_cycles']}, ch0 active cycles: {len(active)}")
    if active:
        print("OK: transfer ran in Verilator model")
