#!/usr/bin/env python3
"""
spi_xfer_public - VerilatorDut
Persistent-subprocess drop-in for local_sim.SpiXferPublic, driven by the
Verilator-compiled RTL (real RTL data in, real coverage out).

harness.py keeps its public interface (reset()/step()/state/reward) and the
same CoverageSimulator; switching the DUT backend from local_sim to the real
RTL is controlled by the harness backend switch (default "verilator", i.e.
the coverage feedback comes from real RTL simulation data by default).

Protocol (see verilator_harness/sim_main.cpp): one stdin action line per clock
cycle, one stdout row per cycle. The row uses the same cov_* schema that
local_sim.read_signals() returns, plus the derived "protocol" field.
"""

import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import run_verilator as rv  # noqa: E402


class VerilatorDut:
    """Drop-in replacement for local_sim.SpiXferPublic backed by real RTL."""

    def __init__(self, spec_dfs_min: int = 4, spec_hold_ss: int = 4):
        self.spec_dfs_min = int(spec_dfs_min or 4)
        self.spec_hold_ss = int(spec_hold_ss or 4)
        self._last = None
        self._p = None
        self._header = []
        self._spawn()

    def _spawn(self):
        # Build the Verilator model on demand (params are runtime argv here,
        # so one build serves every param value).
        rv.VerilatorHarness(self.spec_dfs_min, self.spec_hold_ss)
        self._p = subprocess.Popen(
            [rv.EXE, str(self.spec_dfs_min), str(self.spec_hold_ss)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            text=True, bufsize=1)
        line = self._p.stdout.readline()
        self._header = [c.strip() for c in line.strip().split(",")]

    @staticmethod
    def _line(a) -> str:
        we  = int(a.get("reg_we", 0)) & 1
        adr = int(a.get("reg_addr", 0)) & 0xF
        wd  = int(a.get("reg_wdata", 0)) & 0xFFFFFFFF
        re  = int(a.get("reg_re", 0)) & 1
        rxd = int(a.get("rxd", 0)) & 1
        ss  = int(a.get("ss_in_n", 1)) & 1
        rst = int(a.get("rst_n", 1)) & 1
        return f"{we} {adr} {wd} {re} {rxd} {ss} {rst}"

    def _row(self) -> dict:
        out = self._p.stdout.readline()
        if not out.strip():
            raise RuntimeError("VerilatorDut: sim_main exited unexpectedly")
        vals = [int(v.strip()) for v in out.strip().split(",")]
        row = {self._header[i]: vals[i]
               for i in range(min(len(self._header), len(vals)))}
        # derived protocol class (mirrors local_sim)
        frf = row.get("cov_frf", 0)
        scph = row.get("cov_scph", 0)
        row["protocol"] = 1 if (frf == 0 and scph == 1) else (2 if frf == 1 else 0)
        return row

    def step(self, action: dict):
        line = self._line(action)
        self._p.stdin.write(line + "\n")
        self._p.stdin.flush()
        self._last = self._row()
        return self

    def read_signals(self) -> dict:
        return self._last

    def close(self):
        if getattr(self, "_p", None):
            try:
                self._p.stdin.close()
                self._p.wait(timeout=5)
            except Exception:
                try:
                    self._p.kill()
                except Exception:
                    pass
            self._p = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
