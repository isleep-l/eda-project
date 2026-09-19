#!/usr/bin/env python3
"""
spi_xfer_public - Verilator harness runner
Builds (if needed) and drives the Verilator-compiled DUT, returning a
per-cycle signal dictionary compatible with coverage_simulator.py and
local_sim.py read_signals().
"""

import os
import sys
import subprocess
from typing import Dict, List, Optional

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OBJ_DIR = os.path.join(SCRIPT_DIR, "obj_dir")
EXE = os.path.join(OBJ_DIR, "Vspi_xfer_top")

# Output columns produced by sim_main.cpp (header order)
COV_KEYS = [
    "cov_fsm_state_id", "cov_fsm_state", "cov_sclk_re", "cov_sclk_fe",
    "cov_sclk_out", "cov_sclk_mask", "cov_bit_cnt", "cov_frame_cnt",
    "cov_x_done", "cov_last_frame", "cov_tx_shift_en", "cov_rx_shift_en",
    "cov_load_tx_shift", "cov_load_rx_buf", "cov_ss_active",
    "cov_xfer_complete", "cov_txd", "cov_dfs", "cov_eff_dfs", "cov_baudr",
    "cov_baud2", "cov_ndf", "cov_tmod", "cov_frf", "cov_scph", "cov_scpol",
    "cov_srl_test", "cov_ss_tgl_en", "cov_ser", "cov_tx_level",
    "cov_rx_level", "cov_tx_empty", "cov_tx_full", "cov_rx_empty",
    "cov_rx_full", "cov_hold_ss_cnt", "cov_rx_data", "cov_ss_in_n",
    "cov_start_xfer", "cov_s0", "cov_s1",
    "cov_s2", "reg_rdata",
]


class VerilatorHarness:
    """Drives the Verilator-compiled spi_xfer_top via a text protocol."""

    def __init__(self, spec_dfs_min: int = 4, spec_hold_ss: int = 4):
        self.spec_dfs_min = spec_dfs_min
        self.spec_hold_ss = spec_hold_ss
        self._ensure_built()

    def _ensure_built(self):
        if not os.path.exists(EXE):
            build_sh = os.path.join(SCRIPT_DIR, "build.sh")
            print("Building Verilator model (first run)...")
            r = subprocess.run(["bash", build_sh], capture_output=True, text=True)
            if r.returncode != 0:
                raise RuntimeError(f"Verilator build failed:\n{r.stdout}\n{r.stderr}")
            if not os.path.exists(EXE):
                raise RuntimeError("Build reported success but executable not found")

    def run_actions(self, actions: List[Dict]) -> List[Dict]:
        """Run a sequence of actions (dicts) and return per-cycle signals."""
        lines = []
        for a in actions:
            we   = int(a.get("reg_we", 0)) & 1
            addr = int(a.get("reg_addr", 0)) & 0xF
            wd   = int(a.get("reg_wdata", 0)) & 0xFFFFFFFF
            re   = int(a.get("reg_re", 0)) & 1
            rxd  = int(a.get("rxd", 0)) & 1
            ss   = int(a.get("ss_in_n", 1)) & 1
            rst  = int(a.get("rst_n", 1)) & 1
            lines.append(f"{we} {addr} {wd} {re} {rxd} {ss} {rst}")

        p = subprocess.run(
            [EXE, str(self.spec_dfs_min), str(self.spec_hold_ss)],
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
            # derived protocol class (mirrors local_sim)
            frf = row.get("cov_frf", 0)
            scph = row.get("cov_scph", 0)
            row["protocol"] = 1 if (frf == 0 and scph == 1) else (2 if frf == 1 else 0)
            results.append(row)
        return results


def run_verilator_simulation(actions: List[Dict],
                             spec_dfs_min: int = 4, spec_hold_ss: int = 4) -> Dict:
    """Run a full simulation and return per-cycle signals."""
    harness = VerilatorHarness(spec_dfs_min, spec_hold_ss)
    signals = harness.run_actions(actions)
    return {
        "signals": signals,
        "total_cycles": len(signals),
    }


def run_and_save_curve(actions: List[Dict], interval: int = 1000,
                       spec_dfs_min: int = 4, spec_hold_ss: int = 4,
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

    # Import the same coverage simulator the Python path uses so the real RTL
    # coverage is computed by the identical bin rules.
    sim_dir = os.path.join(SCRIPT_DIR, "..")
    sys.path.insert(0, sim_dir)
    from coverage_simulator import CoverageSimulator  # noqa: E402

    signals = run_verilator_simulation(actions, spec_dfs_min, spec_hold_ss)["signals"]
    total = len(signals)
    if total == 0:
        return []

    cs = CoverageSimulator(
        os.path.join(sim_dir, "dut", "coverage_meta.json"),
        spec_dfs_min=spec_dfs_min,
    )
    curve: List[List] = []
    for cyc, sig in enumerate(signals):
        cs.step(sig)
        if cyc % interval == 0:
            curve.append([int(cyc), float(cs.coverage)])
    curve.append([int(total - 1), float(cs.coverage)])

    if out_file is None:
        out_file = os.path.join(SCRIPT_DIR, "coverage_curves_verilator.json")
    record = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "simulator": "verilator",
        "spec_dfs_min": spec_dfs_min,
        "spec_hold_ss": spec_hold_ss,
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
    # Self-test: a minimal SPI0 transfer sequence
    A = [{"rst_n": 0}, {"rst_n": 1},
         {"reg_we": 1, "reg_addr": 0x0, "reg_wdata": 0x0000_0807},  # dfs=7 -> 8-bit frame
         {"reg_we": 1, "reg_addr": 0x3, "reg_wdata": 0x1},
         {"reg_we": 1, "reg_addr": 0x4, "reg_wdata": 0x8},
         {"reg_we": 1, "reg_addr": 0x5, "reg_wdata": 0x0},
         {"reg_we": 1, "reg_addr": 0x9, "reg_wdata": 0xFF},
         {"reg_we": 1, "reg_addr": 0x2, "reg_wdata": 0x1}]
    A += [{}] * 200

    res = run_verilator_simulation(A)
    done = [s for s in res["signals"] if s.get("cov_xfer_complete")]
    print(f"total cycles: {res['total_cycles']}, "
          f"transfer-complete pulses: {len(done)}")
    if done:
        print("OK: transfer completed in Verilator model")
