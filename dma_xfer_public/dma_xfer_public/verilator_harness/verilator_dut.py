#!/usr/bin/env python3
"""
dma_xfer_public - VerilatorDut
Persistent-subprocess drop-in for local_sim.DmaXferPublic, driven by the
Verilator-compiled RTL (real RTL data in, real coverage out).

harness.py keeps its public interface (apply_config/tick_start/step/
read_signals) and the same CoverageSimulator; switching the DUT backend from
local_sim to the real RTL is controlled by the harness backend switch (default
"verilator", i.e. the coverage feedback comes from real RTL simulation data by
default).

Hidden params ARB_PRIO / DONE_OK_MODES / XFER_HOLD_N are baked into the model
at BUILD time (Verilator >=5.020 cannot assign top-level parameters at runtime),
so a param change triggers a rebuild via run_verilator.VerilatorHarness (which
remembers the baked values in baked_params.txt).

Protocol (see verilator_harness/sim_main.cpp): one stdin action line per clock
cycle:  ch_sel conf_wr conf_field d0 d1 d2 d3 start rst_n. One stdout row per
cycle; rows are translated via run_verilator._translate() to the ch0_state /
seq* / arb_* schema that local_sim.read_signals() returns.
"""

import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import run_verilator as rv  # noqa: E402


class VerilatorDut:
    """Drop-in replacement for local_sim.DmaXferPublic backed by real RTL."""

    def __init__(self, arb_prio=None, done_ok_modes=None, xfer_hold_n=16):
        self.arb_prio = self._enc_arb(arb_prio)
        self.done_ok_modes = self._enc_dom(done_ok_modes)
        self.xfer_hold_n = int(xfer_hold_n or 16)
        self._conf = None      # pending {ch, field, data} config write
        self._start = None     # pending start edge (channel index)
        self._last = None
        self._p = None
        self._header = []
        self._spawn()

    # -- hidden-param encoding (lists -> baked ints) -----------------------
    @staticmethod
    def _enc_arb(prio) -> int:
        """2 bits per channel: [ord0, ord1, ord2, ord3] -> ARB_PRIO."""
        if prio is None:
            return 0x30  # public placeholder (ascending)
        return sum((int(p) & 3) << (2 * i) for i, p in enumerate(prio))

    @staticmethod
    def _enc_dom(modes) -> int:
        """4 bits per channel: {m0..m3} -> DONE_OK_MODES."""
        if modes is None:
            return 0  # public placeholder (done_ok inert)
        return sum((int(m) & 0xF) << (4 * i) for i, m in enumerate(modes))

    def _spawn(self):
        # Build (or reuse) the Verilator model with these baked params.
        rv.VerilatorHarness(self.arb_prio, self.done_ok_modes, self.xfer_hold_n)
        self._p = subprocess.Popen(
            [rv.EXE],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            text=True, bufsize=1)
        line = self._p.stdout.readline()
        self._header = [c.strip() for c in line.strip().split(",")]

    def apply_config(self, ch: int, field: int, data: int):
        """Record a register config write (field 0/1/2, same as local_sim)."""
        self._conf = (int(ch) & 3, int(field) % 3, int(data) & 0xFFFFFFFF)

    def tick_start(self, start_mask: int):
        """Record a start edge. harness passes a single-bit rise mask whose bit
        index is the channel; a zero mask clears any pending start (but must
        NOT clear a pending config write, which may be set this same cycle)."""
        if start_mask:
            self._start = (int(start_mask).bit_length() - 1) & 3
        else:
            self._start = None

    def step(self):
        # Emit one protocol line for this cycle: config write, else start edge,
        # else idle. rst_n is always 1 (harness has no reset input).
        if self._conf is not None:
            ch, field, data = self._conf
            d0 = data & 0xFF
            d1 = (data >> 8) & 0xFF
            d2 = (data >> 16) & 0xFF
            d3 = (data >> 24) & 0xFF
            line = f"{ch} 1 {field} {d0} {d1} {d2} {d3} 0 1"
            self._conf = None
        elif self._start is not None:
            ch = self._start
            line = f"{ch} 0 0 0 0 0 0 1 1"
            self._start = None
        else:
            line = "0 0 0 0 0 0 0 0 1"
        self._p.stdin.write(line + "\n")
        self._p.stdin.flush()
        self._last = rv._translate(self._row())
        return self

    def _row(self) -> dict:
        out = self._p.stdout.readline()
        if not out.strip():
            raise RuntimeError("VerilatorDut: sim_main exited unexpectedly")
        vals = [int(v.strip()) for v in out.strip().split(",")]
        return {self._header[i]: vals[i]
                for i in range(min(len(self._header), len(vals)))}

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
