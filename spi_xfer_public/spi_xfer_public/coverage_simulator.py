#!/usr/bin/env python3
"""
spi_xfer_public - Coverage Simulator
Compatible with both local_sim and Verilator signal dictionaries.
Reads dut/coverage_meta.json and updates bin-hit counts per cycle.
"""

import json
import os
from typing import Dict, List, Optional

import numpy as np


class CoverageSimulator:
    """Coverage simulator compatible with local_sim and Verilator."""

    def __init__(self, meta_path: Optional[str] = None,
                 spec_dfs_min: int = 4):
        if meta_path is None:
            meta_path = os.path.join(os.path.dirname(__file__), "dut", "coverage_meta.json")

        with open(meta_path, "r") as f:
            self.meta = json.load(f)

        self.spec_dfs_min = spec_dfs_min
        self.coverpoints = self.meta["coverpoints"]
        self.bin_counts = {}
        self._total_bins = 0
        self.hit_bins = 0

        # Initialize bin counts
        for cp in self.coverpoints:
            cp_name = cp["name"]
            self.bin_counts[cp_name] = {}
            for bin_def in cp.get("bins", []):
                bin_name = bin_def["name"]
                self.bin_counts[cp_name][bin_name] = 0
                self._total_bins += 1

    def reset(self) -> np.ndarray:
        """Reset all bin counts and return the zeroed coverage vector.

        Attachment-C compatible: returns np.ndarray shape=(total_bins,),
        dtype float32.
        """
        for cp_name in self.bin_counts:
            for bin_name in self.bin_counts[cp_name]:
                self.bin_counts[cp_name][bin_name] = 0
        self.hit_bins = 0
        return np.zeros(self._total_bins, dtype=np.float32)

    def step(self, signals: Dict) -> np.ndarray:
        """Process one cycle of signals and update coverage.

        Attachment-C compatible: returns the current coverage vector as
        np.ndarray shape=(total_bins,), dtype float32.
        """
        for cp in self.coverpoints:
            cp_name = cp["name"]
            if cp["type"] == "basic":
                self._process_basic(cp, signals)
            elif cp["type"] == "boundary":
                self._process_boundary(cp, signals)
            elif cp["type"] == "condition":
                self._process_condition(cp, signals)
            elif cp["type"] == "sequential":
                self._process_sequential(cp, signals)
            elif cp["type"] == "cross":
                self._process_cross(cp, signals)
        return self._vector()

    @staticmethod
    def _active(signals: Dict) -> bool:
        """True while the transfer FSM is in an active state (not IDLE/SLEEP).

        Used to gate configuration-derived bins on a *real* in-progress
        transfer, so that merely writing registers without transmitting no
        longer earns coverage.
        """
        st = signals.get("cov_fsm_state_id", -1)
        return st not in (-1, 0, 10)

    def _process_basic(self, cp: Dict, signals: Dict):
        cp_name = cp["name"]
        active = self._active(signals)
        for bin_def in cp.get("bins", []):
            value = bin_def["value"]
            hit = False
            if cp_name == "fsm_state":
                hit = (signals.get("cov_fsm_state_id") == value)
            elif cp_name == "protocol_mode":
                hit = active and (signals.get("protocol") == value)
            elif cp_name == "tmod":
                hit = active and (signals.get("cov_tmod") == value)
            elif cp_name == "ss_sel":
                hit = active and (signals.get("cov_ser") == value)
            if hit:
                self._hit(cp_name, bin_def["name"])

    def _process_boundary(self, cp: Dict, signals: Dict):
        cp_name = cp["name"]
        active = self._active(signals)
        for bin_def in cp.get("bins", []):
            value = bin_def["value"]
            hit = False
            if cp_name == "dfs_boundary":
                hit = active and (signals.get("cov_eff_dfs") == value)
            elif cp_name == "baud_div_boundary":
                hit = active and (signals.get("cov_baudr") == value)
            elif cp_name == "frame_cnt_boundary":
                hit = active and (signals.get("cov_frame_cnt") == value)
            elif cp_name == "rx_data_boundary":
                # only meaningful when the RX FIFO actually holds received data
                hit = (signals.get("cov_rx_level", 0) > 0 and
                       signals.get("cov_rx_data") == value)
            elif cp_name == "tx_fifo_level_boundary":
                hit = active and (signals.get("cov_tx_level") == value)
            elif cp_name == "rx_fifo_level_boundary":
                hit = active and (signals.get("cov_rx_level") == value)
            elif cp_name == "hold_ss_cnt_boundary":
                # hold counter only meaningful inside the HOLD_SS state
                hit = (signals.get("cov_fsm_state_id") == 8 and
                       signals.get("cov_hold_ss_cnt") == value)
            if hit:
                self._hit(cp_name, bin_def["name"])

    def _process_condition(self, cp: Dict, signals: Dict):
        cp_name = cp["name"]
        active = self._active(signals)
        for bin_def in cp.get("bins", []):
            value = bin_def["value"]
            hit = False
            if cp_name == "baud2_path":
                hit = active and (signals.get("cov_baud2") == value)
            elif cp_name == "x_done":
                hit = active and (signals.get("cov_x_done") == value)
            elif cp_name == "last_frame":
                hit = active and (signals.get("cov_last_frame") == value)
            elif cp_name == "shift_ctl":
                # derived: (tx,rx): (0,0)->0 idle, (1,0)->1 tx, (0,1)->2 rx
                tx = signals.get("cov_tx_shift_en", 0)
                rx = signals.get("cov_rx_shift_en", 0)
                cur = 0
                if tx and not rx:
                    cur = 1
                elif rx and not tx:
                    cur = 2
                hit = active and (cur == value)
            elif cp_name == "txd_value":
                hit = active and (signals.get("cov_txd") == value)
            elif cp_name == "sclk_edges":
                # derived: (re,fe): (0,0)->0 none, (1,0)->1 rising, (0,1)->2 falling
                re = signals.get("cov_sclk_re", 0)
                fe = signals.get("cov_sclk_fe", 0)
                cur = 0
                if re and not fe:
                    cur = 1
                elif fe and not re:
                    cur = 2
                hit = active and (cur == value)
            elif cp_name == "srl_test":
                hit = active and (signals.get("cov_srl_test") == value)
            elif cp_name == "ss_toggle":
                hit = active and (signals.get("cov_ss_tgl_en") == value)
            elif cp_name == "dfs_effective_min":
                # hidden minimum clamp exercised: effective frame size stuck at
                # the hidden minimum while the raw DFS field decodes below it
                hit = active and (signals.get("cov_eff_dfs") == self.spec_dfs_min and
                                  signals.get("cov_dfs", 31) < self.spec_dfs_min - 1)
            elif cp_name == "dfs_effective_min_frame":
                # a frame actually completed under the clamped config:
                # sub-minimum DFS field, effective size stuck at the hidden
                # minimum, and the RX buffer was really loaded for a frame.
                hit = (signals.get("cov_eff_dfs") == self.spec_dfs_min and
                       signals.get("cov_dfs", 31) < self.spec_dfs_min - 1 and
                       signals.get("cov_load_rx_buf") == 1)
            if hit:
                self._hit(cp_name, bin_def["name"])

    def _process_sequential(self, cp: Dict, signals: Dict):
        cp_name = cp["name"]
        active = self._active(signals)
        if cp_name == "hold_last_frame":
            for bin_def in cp.get("bins", []):
                # HOLD_SS dwell counter at its max, on a frame that is flagged
                # as the last frame of a *multi-frame* transfer (frame counter
                # >= 1). Single-frame transfers never reach the "last frame of
                # several" condition.
                hit = (signals.get("cov_fsm_state_id") == 8 and
                       signals.get("cov_hold_ss_cnt") == 3 and
                       signals.get("cov_last_frame") == 1 and
                       signals.get("cov_frame_cnt", 0) >= 1)
                if hit:
                    self._hit(cp_name, bin_def["name"])
        elif cp_name == "seq_a":
            ab = signals.get("cov_s0", 0)
            for bin_def in cp.get("bins", []):
                b = bin_def["name"]
                hit = False
                if b == "seq_a_0":
                    hit = ab and active
                elif b == "seq_a_1":
                    hit = ab and active and signals.get("cov_frame_cnt", 0) >= 1
                elif b == "seq_a_2":
                    hit = (ab and signals.get("cov_fsm_state_id") == 8 and
                           signals.get("cov_hold_ss_cnt") == 3 and
                           signals.get("cov_last_frame") == 1 and
                           signals.get("cov_frame_cnt", 0) >= 1)
                elif b in ("seq_a_3", "seq_a_4", "seq_a_5"):
                    proto = {"seq_a_3": 0, "seq_a_4": 1, "seq_a_5": 2}[b]
                    hit = ab and signals.get("protocol") == proto
                if hit:
                    self._hit(cp_name, b)
        elif cp_name == "seq_b":
            # TX FIFO drained mid-transfer, then refilled before completion
            hit = (signals.get("cov_s1", 0) and active and
                   not signals.get("cov_tx_empty", 1) and
                   signals.get("cov_frame_cnt", 0) >= 1)
            if hit:
                self._hit(cp_name, "seq_b_0")
        elif cp_name == "seq_c":
            wa = signals.get("cov_s2", 0)
            for bin_def in cp.get("bins", []):
                b = bin_def["name"]
                hit = False
                if b == "seq_c_0":
                    hit = wa and active
                elif b == "seq_c_1":
                    hit = wa and active and signals.get("cov_frame_cnt", 0) >= 1
                if hit:
                    self._hit(cp_name, b)

    def _process_cross(self, cp: Dict, signals: Dict):
        cp_name = cp["name"]
        active = self._active(signals)
        for bin_def in cp.get("bins", []):
            cross = bin_def.get("cross", [])
            if len(cross) != 2:
                continue
            v0, v1 = cross
            hit = False
            if not active:
                hit = False
            elif cp_name == "protocol_x_tmod":
                hit = (signals.get("protocol") == v0 and
                       signals.get("cov_tmod") == v1)
            elif cp_name == "protocol_x_dfs_boundary":
                hit = (signals.get("protocol") == v0 and
                       signals.get("cov_eff_dfs") == v1)
            elif cp_name == "protocol_x_baud2_path":
                hit = (signals.get("protocol") == v0 and
                       signals.get("cov_baud2") == v1)
            elif cp_name == "protocol_x_hold_ss_cnt_boundary":
                hit = (signals.get("protocol") == v0 and
                       signals.get("cov_fsm_state_id") == 8 and
                       signals.get("cov_hold_ss_cnt") == v1)
            elif cp_name == "protocol_x_last_frame":
                hit = (signals.get("protocol") == v0 and
                       signals.get("cov_last_frame") == v1)
            if hit:
                self._hit(cp_name, bin_def["name"])

    def _hit(self, cp_name: str, bin_name: str):
        """Record a bin hit."""
        if self.bin_counts[cp_name][bin_name] == 0:
            self.hit_bins += 1
        self.bin_counts[cp_name][bin_name] += 1

    def get_coverage(self) -> Dict:
        return {
            "total_bins": self._total_bins,
            "hit_bins": self.hit_bins,
            "coverage_pct": (self.hit_bins / self._total_bins * 100) if self._total_bins > 0 else 0,
            "bin_counts": self.bin_counts,
        }

    def get_bin_vector(self) -> List[int]:
        """Coverage as a flat binary vector (1=hit, 0=not hit)."""
        return list(self._vector())

    def get_state(self) -> np.ndarray:
        """Current coverage vector (Attachment-C C.2 compatible).

        Same shape/dtype as reset()/step(): np.ndarray shape=(total_bins,),
        dtype float32, 1.0 = bin hit at least once, 0.0 = never hit.
        """
        return self._vector()

    def _vector(self) -> np.ndarray:
        """Current coverage as a flat binary vector, dtype float32."""
        vector = np.zeros(self._total_bins, dtype=np.float32)
        i = 0
        for cp in self.coverpoints:
            cp_name = cp["name"]
            for bin_def in cp.get("bins", []):
                bin_name = bin_def["name"]
                if self.bin_counts[cp_name][bin_name] > 0:
                    vector[i] = 1.0
                i += 1
        return vector

    # -- Attachment-C compatible properties ---------------------------------
    @property
    def total_bins(self) -> int:
        """Total number of coverage bins."""
        return self._total_bins

    @property
    def covered_bins(self) -> int:
        """Number of distinct bins hit so far."""
        return self.hit_bins

    @property
    def coverage(self) -> float:
        """Fraction of bins covered, in [0, 1]."""
        return (self.hit_bins / self._total_bins) if self._total_bins > 0 else 0.0


if __name__ == "__main__":
    sim = CoverageSimulator()
    print(f"Total bins: {sim.total_bins}")

    # Smoke test against local_sim
    import sys
    sys.path.insert(0, os.path.dirname(__file__))
    from local_sim import SpiXferPublic, A_CTRLR0, A_SER, A_BAUDR, A_TXFTLR, A_DR, A_SSIENR

    dut = SpiXferPublic()

    # SPI0, 8-bit frame (dfs field=7), loopback, single frame
    dut.write_reg(A_CTRLR0, 0x0000_0807)   # dfs=7 -> 8-bit frame, srl_test=1
    dut.write_reg(A_SER, 0x1)
    dut.write_reg(A_BAUDR, 8)
    dut.write_reg(A_TXFTLR, 0)
    dut.write_reg(A_DR, 0x0000_00FF)
    dut.write_reg(A_SSIENR, 1)

    for _ in range(400):
        dut.step({})
        sim.step(dut.read_signals())

    cov = sim.get_coverage()
    print(f"Coverage: {cov['hit_bins']}/{cov['total_bins']} = {cov['coverage_pct']:.1f}%")
