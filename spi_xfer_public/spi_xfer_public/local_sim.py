#!/usr/bin/env python3
"""
spi_xfer_public - Black-box simulator (local_sim.py)
Python implementation of the SPI / SSP master serial transfer controller for
local development.  Cycle-accurate mirror of dut/spi_xfer_top.v.

Hidden parameters (SPEC_DFS_MIN, SPEC_HOLD_SS) are injected externally at
evaluation time; the local defaults match the evaluation configuration.

Action space (13-dim, dict form):
    action = {"reg_we", "reg_addr", "reg_wdata", "reg_re", "rxd",
              "ss_in_n", "rst_n", "pad0".."pad4"}
  - reg_we   : register write strobe
  - reg_addr : 4-bit word address (see register map)
  - reg_wdata: 32-bit write data
  - reg_re   : register read strobe (addr=DR pops the RX FIFO)
  - rxd      : serial receive data input
  - ss_in_n  : slave-select input (SPI: must be 1 during transfer,
               SSP: must be 0 during transfer)
  - rst_n    : async active-low reset (1 = run, 0 = reset)
"""

from typing import Dict, List, Optional

# ---------------------------------------------------------------------------
# Register map (word offsets)
# ---------------------------------------------------------------------------
A_CTRLR0 = 0x0
A_CTRLR1 = 0x1
A_SSIENR = 0x2
A_SER    = 0x3
A_BAUDR  = 0x4
A_TXFTLR = 0x5
A_TXFLR  = 0x6
A_RXFLR  = 0x7
A_SR     = 0x8
A_DR     = 0x9

# ---------------------------------------------------------------------------
# FSM state encoding
# ---------------------------------------------------------------------------
S_IDLE      = 0
S_ASSERT_SS = 1
S_POP_TX    = 2
S_SHIFT_BIT = 3
S_PUSH_RX   = 4
S_TOGGLE_SS = 5
S_HOLD_MASK = 6
S_MASK_SCLK = 7
S_HOLD_SS   = 8
S_CLEAR_SS  = 9
S_SLEEP     = 10

STATE_NAMES = {
    S_IDLE: "IDLE", S_ASSERT_SS: "ASSERT_SS", S_POP_TX: "POP_TX",
    S_SHIFT_BIT: "SHIFT_BIT", S_PUSH_RX: "PUSH_RX", S_TOGGLE_SS: "TOGGLE_SS",
    S_HOLD_MASK: "HOLD_MASK", S_MASK_SCLK: "MASK_SCLK", S_HOLD_SS: "HOLD_SS",
    S_CLEAR_SS: "CLEAR_SS", S_SLEEP: "SLEEP",
}

ACTION_KEYS = ["reg_we", "reg_addr", "reg_wdata", "reg_re", "rxd",
               "ss_in_n", "rst_n", "pad0", "pad1", "pad2", "pad3", "pad4"]


class SpiXferPublic:
    """Black-box simulator for spi_xfer_public."""

    def __init__(self, spec_dfs_min: Optional[int] = None,
                 spec_hold_ss: Optional[int] = None):
        # Hidden configuration (evaluation-injected; local default = eval value)
        self.spec_dfs_min = spec_dfs_min if spec_dfs_min is not None else 4
        self.spec_hold_ss = spec_hold_ss if spec_hold_ss is not None else 4
        self.reset()

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------
    def reset(self):
        # Register file
        self.ctrlr0 = 0
        self.ctrlr1 = 0
        self.ssienr = 0
        self.ser    = 0
        self.baudr  = 0
        self.txftlr = 0
        self.start_xfer = False

        # FIFOs
        self.tx_mem = [0] * 8
        self.rx_mem = [0] * 8
        self.tx_wr_ptr = 0
        self.tx_rd_ptr = 0
        self.rx_wr_ptr = 0
        self.rx_rd_ptr = 0
        self.tx_cnt = 0
        self.rx_cnt = 0

        # sclk generator
        self.ssi_cnt = 0
        self.sclk_out = 1          # ~scpol with scpol=0
        self.sclk_re_ir = False
        self.sclk_fe_ir = False

        # master FSM
        self.c_state = S_IDLE
        self.bit_cnt = 0
        self.frame_cnt = 0
        self.last_frame = False
        self.x_done_1d = False
        self.tx_load_en = True
        self.abort_ir = False
        self.hold_cnt = 0
        self.fsm_sleep_ir = False
        self.fsm_busy_ir = False
        self.ssi_oe_n_ir = True
        self.abort_seen = False
        self.refill_seen = False
        self.write_active = False

        # shift controller
        self.tx_shift_reg = 0
        self.tx_buffer = 0
        self.txd = 0
        self.rx_shift_reg = 0
        self.rx_buffer = 0
        self.ss_n = 0xF
        self.tx_pop_ir = False
        self.rx_push_ir = False

        # top-level
        self.prev_fsm_state = 0

        # current-cycle combinational outputs
        self.reg_rdata = 0
        self.cov_xfer_complete = False

        # inputs (last step)
        self._in = {"reg_we": 0, "reg_addr": 0, "reg_wdata": 0,
                    "reg_re": 0, "rxd": 0, "ss_in_n": 1, "rst_n": 1}

    def inject_hidden_params(self, spec_dfs_min: int, spec_hold_ss: int):
        """Inject hidden parameters (called by the evaluation platform)."""
        self.spec_dfs_min = spec_dfs_min
        self.spec_hold_ss = spec_hold_ss

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _zero_pad(val: int, dfs: int) -> int:
        if dfs >= 32:
            return val & 0xFFFFFFFF
        return val & ((1 << dfs) - 1)

    @staticmethod
    def _bit(v: int, i: int) -> int:
        return (v >> i) & 1

    # ------------------------------------------------------------------
    # Step: one clock cycle
    # ------------------------------------------------------------------
    def step(self, action: Optional[Dict] = None):
        # ---- sample inputs -------------------------------------------------
        a = self._in
        if action is not None:
            a = {k: action.get(k, 0) for k in ACTION_KEYS}
            a["ss_in_n"] = action.get("ss_in_n", 1)
            a["rst_n"]   = action.get("rst_n", 1)
            self._in = a
        reg_we    = int(a["reg_we"]) & 1
        reg_addr  = int(a["reg_addr"]) & 0xF
        reg_wdata = int(a["reg_wdata"]) & 0xFFFFFFFF
        reg_re    = int(a["reg_re"]) & 1
        rxd       = int(a["rxd"]) & 1
        ss_in_n   = int(a["ss_in_n"]) & 1
        rst_n     = int(a["rst_n"]) & 1

        if not rst_n:
            self.reset()
            return

        # ---- config decode (combinational) --------------------------------
        dfs      = self.ctrlr0 & 0x1F
        frf      = (self.ctrlr0 >> 5) & 0x3
        scph     = (self.ctrlr0 >> 7) & 1
        scpol    = (self.ctrlr0 >> 8) & 1
        tmod     = (self.ctrlr0 >> 9) & 0x3
        srl_test = (self.ctrlr0 >> 11) & 1
        ss_tgl   = (self.ctrlr0 >> 12) & 1
        ndf      = self.ctrlr1 & 0x1FFFF
        dfs_eff  = max(dfs + 1, self.spec_dfs_min)
        baud2    = (self.baudr == 2)

        spi0 = (frf == 0) and (scph == 0)
        spi1 = (frf == 0) and (scph == 1)
        ssp  = (frf == 1)

        sclk_re_int = self.sclk_fe_ir if spi0 else self.sclk_re_ir
        sclk_fe_int = self.sclk_re_ir if spi0 else self.sclk_fe_ir

        # ---- register file combinational ----------------------------------
        tx_push = reg_we and (reg_addr == A_DR)
        rx_pop  = reg_re and (reg_addr == A_DR)

        if reg_addr == A_CTRLR0: reg_rdata = self.ctrlr0
        elif reg_addr == A_CTRLR1: reg_rdata = self.ctrlr1
        elif reg_addr == A_SSIENR: reg_rdata = self.ssienr
        elif reg_addr == A_SER:    reg_rdata = self.ser
        elif reg_addr == A_BAUDR:  reg_rdata = self.baudr
        elif reg_addr == A_TXFTLR: reg_rdata = self.txftlr
        elif reg_addr == A_TXFLR:  reg_rdata = self.tx_cnt
        elif reg_addr == A_RXFLR:  reg_rdata = self.rx_cnt
        elif reg_addr == A_SR:     reg_rdata = ((self.rx_cnt == 8) << 4) | \
                                                ((self.rx_cnt != 0) << 3) | \
                                                ((self.tx_cnt == 0) << 2) | \
                                                ((self.tx_cnt != 8) << 1) | \
                                                self.fsm_busy_ir
        elif reg_addr == A_DR:     reg_rdata = self.rx_mem[self.rx_rd_ptr]
        else:                      reg_rdata = 0
        self.reg_rdata = reg_rdata

        slv_sel        = (self.ser != 0)
        tx_empty_early = (not tx_push) and self.tx_pop_ir and (self.tx_cnt == 1)
        tx_fifo_ready  = (self.tx_cnt > self.txftlr) if tmod != 2 else (self.tx_cnt != 0)
        start_xfer_nxt = bool(slv_sel and (not tx_empty_early) and tx_fifo_ready)

        # ---- FIFO combinational -------------------------------------------
        tx_full  = (self.tx_cnt == 8)
        tx_empty = (self.tx_cnt == 0)
        rx_full  = (self.rx_cnt == 8)
        rx_empty = (self.rx_cnt == 0)
        tx_wr_ok = tx_push and (not tx_full)
        tx_rd_ok = self.tx_pop_ir and (not tx_empty)
        rx_wr_ok = self.rx_push_ir and (not rx_full)
        rx_rd_ok = rx_pop and (not rx_empty)
        tx_pop_data = self.tx_mem[self.tx_rd_ptr]

        # ---- sclk generator combinational --------------------------------
        r_det = ((self.baudr >> 1) - 1) & 0xFFFF
        f_det = (self.baudr - 1) & 0xFFFF

        # ---- master FSM combinational -------------------------------------
        tx_fifo_filled = 1 if tmod == 2 else (1 if self.tx_cnt > self.txftlr else 0)
        f_done    = (self.bit_cnt == dfs_eff)
        sspf_done = (self.bit_cnt == (dfs_eff - 1))
        if ((tmod == 3) and (frf == 0)) or (tmod == 2):
            x_done = (self.frame_cnt == ndf)
        else:
            x_done = self.last_frame

        # next-state logic
        cs = self.c_state
        if cs == S_IDLE:
            if (self.start_xfer and sclk_re_int and tx_fifo_filled and (not tx_empty) and
                    (((not ssp) and ss_in_n) or (ssp and (not ss_in_n)))):
                n_state = S_ASSERT_SS
            else:
                n_state = S_IDLE
        elif cs == S_ASSERT_SS:
            n_state = S_POP_TX if (sclk_fe_int or baud2) else S_ASSERT_SS
        elif cs == S_POP_TX:
            n_state = S_SHIFT_BIT if (sclk_re_int or baud2) else S_POP_TX
        elif cs == S_SHIFT_BIT:
            if f_done and (not ssp):
                n_state = S_PUSH_RX
            elif ssp and f_done and (not x_done):
                n_state = S_TOGGLE_SS
            elif ssp and f_done and self.x_done_1d:
                n_state = S_PUSH_RX
            else:
                n_state = S_SHIFT_BIT
        elif cs == S_PUSH_RX:
            if sclk_fe_int or baud2:
                if spi0 and (not baud2):
                    n_state = S_HOLD_MASK
                elif (spi1 and x_done) or (spi0 and baud2 and (x_done or ss_tgl)) or ssp:
                    n_state = S_MASK_SCLK
                elif spi0 and baud2 and (not x_done) and (not ss_tgl):
                    n_state = S_SHIFT_BIT
                else:
                    n_state = S_PUSH_RX
            elif sclk_re_int or baud2:
                if (not x_done) and spi1:
                    n_state = S_SHIFT_BIT
                else:
                    n_state = S_PUSH_RX
            else:
                n_state = S_PUSH_RX
        elif cs == S_TOGGLE_SS:
            n_state = S_SHIFT_BIT if (sclk_re_int or baud2) else S_TOGGLE_SS
        elif cs == S_HOLD_MASK:
            if sclk_re_int or baud2:
                if ss_tgl or ssp or spi1:
                    n_state = S_MASK_SCLK
                elif x_done or self.abort_ir:
                    n_state = S_MASK_SCLK
                else:
                    n_state = S_SHIFT_BIT
            else:
                n_state = S_HOLD_MASK
        elif cs == S_MASK_SCLK:
            if (sclk_fe_int or baud2) and spi0 and (not x_done) and (not self.abort_ir):
                n_state = S_TOGGLE_SS
            elif sclk_re_int and (ssp or (spi0 and (x_done or self.abort_ir))):
                n_state = S_CLEAR_SS
            elif (sclk_re_int or baud2) and (spi1 or ssp):
                n_state = S_HOLD_SS
            else:
                n_state = S_MASK_SCLK
        elif cs == S_HOLD_SS:
            if (sclk_fe_int or baud2) and (self.hold_cnt == (self.spec_hold_ss - 1)):
                n_state = S_CLEAR_SS
            else:
                n_state = S_HOLD_SS
        elif cs == S_CLEAR_SS:
            n_state = S_IDLE if (sclk_re_int or baud2) else S_CLEAR_SS
        elif cs == S_SLEEP:
            # mirror fl_ssi: leaving SLEEP only possible once SSI re-enabled
            n_state = S_IDLE if self.ssienr else S_SLEEP
        else:
            n_state = S_IDLE

        # state-override: !ssienr -> SLEEP; ss_in_n polarity violation -> IDLE
        if not self.ssienr:
            c_state_nxt = S_SLEEP
        elif (ssp and ss_in_n) or ((not ssp) and (not ss_in_n)):
            c_state_nxt = S_IDLE
        else:
            c_state_nxt = n_state

        # bit counter
        inc_bit_cnt = (cs == S_SHIFT_BIT) and sclk_re_int
        clr_bit_cnt = ((cs == S_TOGGLE_SS) and sclk_re_int and (ssp or ss_tgl)) or \
                      ((cs == S_HOLD_MASK) and sclk_re_int and (not ssp) and (not ss_tgl)) or \
                      ((cs == S_PUSH_RX) and sclk_re_int) or \
                      ((cs == S_IDLE) and sclk_re_int)
        if clr_bit_cnt:
            bit_cnt_nxt = 0
        elif inc_bit_cnt:
            bit_cnt_nxt = (self.bit_cnt + 1) & 0x3F
        else:
            bit_cnt_nxt = self.bit_cnt

        # frame counter
        inc_frame_cnt = (cs == S_SHIFT_BIT) and (self.bit_cnt == (dfs_eff - 2)) and sclk_re_int
        clr_frame_cnt = (cs == S_IDLE) or ((cs == S_CLEAR_SS) and (n_state != S_CLEAR_SS))
        if clr_frame_cnt:
            frame_cnt_nxt = 0
        elif inc_frame_cnt:
            frame_cnt_nxt = (self.frame_cnt + 1) & 0x1FFFF
        else:
            frame_cnt_nxt = self.frame_cnt

        # last_frame
        if tx_empty and (sclk_fe_int or baud2) and (cs != S_MASK_SCLK):
            last_frame_nxt = True
        elif (not tx_empty) and (cs in (S_SHIFT_BIT, S_IDLE)):
            last_frame_nxt = False
        else:
            last_frame_nxt = self.last_frame

        # tx_load_en
        if cs == S_POP_TX:
            if sclk_re_int or baud2:
                tx_load_en_nxt = False if tmod == 2 else True
            else:
                tx_load_en_nxt = self.tx_load_en
        elif cs == S_IDLE:
            tx_load_en_nxt = True
        else:
            tx_load_en_nxt = self.tx_load_en

        # abort
        if (cs == S_HOLD_MASK) and x_done:
            abort_nxt = True
        elif (cs == S_IDLE) or (cs == S_CLEAR_SS):
            abort_nxt = False
        else:
            abort_nxt = self.abort_ir

        # hold counter
        if cs != S_HOLD_SS:
            hold_nxt = 0
        elif sclk_fe_int or baud2:
            hold_nxt = 0 if (self.hold_cnt == (self.spec_hold_ss - 1)) else (self.hold_cnt + 1)
        else:
            hold_nxt = self.hold_cnt

        # sticky coverage flags (mirror spi_mstfsm)
        abort_now = bool(self.ssienr and ((ssp and ss_in_n) or ((not ssp) and (not ss_in_n))) and
                         self.c_state not in (S_IDLE, S_SLEEP))
        if abort_now:
            abort_seen_nxt = True
        elif n_state == S_CLEAR_SS:
            abort_seen_nxt = False
        else:
            abort_seen_nxt = self.abort_seen

        busy_now = (n_state != S_IDLE) and (n_state != S_SLEEP)
        refill_now = bool(busy_now and tx_empty and self.frame_cnt >= 1)
        if refill_now:
            refill_seen_nxt = True
        elif n_state == S_CLEAR_SS:
            refill_seen_nxt = False
        else:
            refill_seen_nxt = self.refill_seen

        # sclk mask
        sclk_mask = (n_state == S_SHIFT_BIT) or (n_state == S_PUSH_RX) or \
                    (n_state == S_HOLD_MASK) or \
                    ((n_state == S_POP_TX) and (spi1 or ssp)) or \
                    ((n_state == S_TOGGLE_SS) and ssp)

        # TX / RX shift enables
        tx_shift_en = (cs == S_SHIFT_BIT) and sclk_re_int and \
                      (((tmod >> 1) & 1) == 0 or tmod == 3)
        rx_shift_en = ((cs == S_SHIFT_BIT) and sclk_fe_int) or \
                      ((cs == S_PUSH_RX) and sclk_fe_int) or \
                      ((cs == S_TOGGLE_SS) and sclk_fe_int)

        # shift register load
        load_tx_shift = (self.tx_load_en and (not tx_empty)) and (
            ((cs == S_POP_TX) and sclk_re_int) or
            ((cs == S_PUSH_RX) and sclk_re_int and (not x_done) and (sclk_mask if ssp else True)) or
            ((cs == S_TOGGLE_SS) and sclk_re_int and ssp) or
            ((cs == S_HOLD_MASK) and sclk_re_int and (not x_done)))
        load_tx_buffer = (not self.tx_load_en) and (
            ((cs == S_POP_TX) and sclk_re_int) or
            ((cs == S_PUSH_RX) and sclk_re_int and (not x_done)) or
            ((cs == S_TOGGLE_SS) and sclk_re_int and ssp) or
            ((cs == S_HOLD_MASK) and sclk_re_int and (not x_done)))
        rx_load_gate = (tmod == 0) or (tmod == 2) or (tmod == 3)
        load_rx_buf = rx_load_gate and (
            ((cs == S_PUSH_RX) and sclk_re_int) or
            ((cs == S_TOGGLE_SS) and (sclk_re_int or baud2) and ssp) or
            ((cs == S_HOLD_MASK) and (sclk_re_int or baud2)) or
            ((cs == S_MASK_SCLK) and (not spi0) and sclk_re_int))

        # slave select enable
        if spi0 or spi1:
            fsm_slv_sel = (n_state != S_IDLE) and (n_state != S_CLEAR_SS) and \
                          (n_state != S_SLEEP) and (n_state != S_TOGGLE_SS)
        elif ssp:
            fsm_slv_sel = (n_state == S_ASSERT_SS) or (n_state == S_POP_TX) or \
                          (n_state == S_TOGGLE_SS) or \
                          ((n_state == S_SHIFT_BIT) and sspf_done and (not x_done) and sclk_re_int)
        else:
            fsm_slv_sel = False

        # output enable
        if spi0 or spi1:
            ssi_oe_n_iw = (n_state == S_IDLE) or (n_state == S_CLEAR_SS) or (n_state == S_SLEEP)
        elif ssp:
            ssi_oe_n_iw = (n_state == S_IDLE) or (n_state == S_ASSERT_SS) or \
                          (n_state == S_CLEAR_SS) or (n_state == S_SLEEP)
        else:
            ssi_oe_n_iw = True

        # ---- shift controller combinational --------------------------------
        rxd_mst_int = self.txd if srl_test else rxd
        old_txd = self.txd

        # ---- registered updates --------------------------------------------
        # register file
        ctrlr0_nxt, ctrlr1_nxt = self.ctrlr0, self.ctrlr1
        ssienr_nxt, ser_nxt = self.ssienr, self.ser
        baudr_nxt, txftlr_nxt = self.baudr, self.txftlr
        if reg_we:
            if reg_addr == A_SSIENR:
                ssienr_nxt = reg_wdata & 1
            elif not self.ssienr:
                if reg_addr == A_CTRLR0: ctrlr0_nxt = reg_wdata
                elif reg_addr == A_CTRLR1: ctrlr1_nxt = reg_wdata & 0x1FFFF
                elif reg_addr == A_SER: ser_nxt = reg_wdata & 0xF
                elif reg_addr == A_BAUDR: baudr_nxt = reg_wdata & 0xFFFF
                elif reg_addr == A_TXFTLR: txftlr_nxt = reg_wdata & 0xF

        # TX FIFO
        tx_wr_ptr_nxt, tx_rd_ptr_nxt, tx_cnt_nxt = self.tx_wr_ptr, self.tx_rd_ptr, self.tx_cnt
        if tx_wr_ok:
            self.tx_mem[self.tx_wr_ptr] = reg_wdata
            tx_wr_ptr_nxt = (self.tx_wr_ptr + 1) & 0x7
        if tx_rd_ok:
            tx_rd_ptr_nxt = (self.tx_rd_ptr + 1) & 0x7
        if tx_wr_ok and not tx_rd_ok:
            tx_cnt_nxt = self.tx_cnt + 1
        elif tx_rd_ok and not tx_wr_ok:
            tx_cnt_nxt = self.tx_cnt - 1

        # RX FIFO
        rx_wr_ptr_nxt, rx_rd_ptr_nxt, rx_cnt_nxt = self.rx_wr_ptr, self.rx_rd_ptr, self.rx_cnt
        if rx_wr_ok:
            self.rx_mem[self.rx_wr_ptr] = self.rx_buffer
            rx_wr_ptr_nxt = (self.rx_wr_ptr + 1) & 0x7
        if rx_rd_ok:
            rx_rd_ptr_nxt = (self.rx_rd_ptr + 1) & 0x7
        if rx_wr_ok and not rx_rd_ok:
            rx_cnt_nxt = self.rx_cnt + 1
        elif rx_rd_ok and not rx_wr_ok:
            rx_cnt_nxt = self.rx_cnt - 1

        # sclk generator
        if not self.ssienr:
            ssi_cnt_nxt = 0
        elif self.ssi_cnt == ((self.baudr - 1) & 0xFFFF):
            ssi_cnt_nxt = 0
        else:
            ssi_cnt_nxt = (self.ssi_cnt + 1) & 0xFFFF
        if not self.ssienr:
            sclk_out_nxt = 0 if scpol else 1        # ~scpol
        elif self.baudr == 0 or not sclk_mask or self.ssi_cnt == 0:
            sclk_out_nxt = 0 if scpol else 1        # ~scpol
        elif self.ssi_cnt == (self.baudr >> 1):
            sclk_out_nxt = scpol
        else:
            sclk_out_nxt = self.sclk_out
        if not self.ssienr:
            sclk_re_nxt = False
            sclk_fe_nxt = False
        else:
            sclk_re_nxt = (self.ssi_cnt == r_det)
            sclk_fe_nxt = (self.ssi_cnt == f_det)

        # master FSM
        fsm_sleep_nxt = (n_state == S_SLEEP)
        fsm_busy_nxt  = (n_state != S_IDLE) and (n_state != S_SLEEP)

        # shift controller
        tx_shift_reg_nxt, tx_buffer_nxt, txd_nxt = self.tx_shift_reg, self.tx_buffer, self.txd
        if load_tx_shift:
            tx_shift_reg_nxt = tx_pop_data
            tx_buffer_nxt = tx_pop_data
            txd_nxt = self._bit(tx_pop_data, dfs_eff - 1)
        elif load_tx_buffer:
            tx_shift_reg_nxt = self.tx_buffer
            txd_nxt = self._bit(self.tx_buffer, dfs_eff - 1)
        elif tx_shift_en:
            txd_nxt = self._bit(self.tx_shift_reg, dfs_eff - 1)
            if dfs_eff >= 32:
                tx_shift_reg_nxt = ((self.tx_shift_reg << 1) & 0xFFFFFFFF)
            else:
                mask = (1 << dfs_eff) - 1
                low = self.tx_shift_reg & mask
                tx_shift_reg_nxt = (self.tx_shift_reg & ~mask & 0xFFFFFFFF) | ((low << 1) & mask)

        if rx_shift_en:
            self.rx_shift_reg = ((self.rx_shift_reg << 1) & 0xFFFFFFFF) | rxd_mst_int
        if load_rx_buf:
            self.rx_buffer = self._zero_pad(self.rx_shift_reg, dfs_eff)

        if frf == 1:  # SSP: active high
            ss_n_nxt = self.ser & (0xF if fsm_slv_sel else 0)
        else:         # SPI: active low
            ss_n_nxt = (~(self.ser & (0xF if fsm_slv_sel else 0))) & 0xF

        tx_pop_ir_nxt  = load_tx_shift
        rx_push_ir_nxt = load_rx_buf if self.ssienr else False

        # top-level
        prev_fsm_state_nxt = self.c_state

        # sticky write-active flag (mirror spi_xfer_top): a DR write while the
        # FSM was busy; cleared on transfer completion.
        xfer_complete_now = (self.c_state == S_IDLE) and \
                            (self.prev_fsm_state != S_IDLE) and (self.prev_fsm_state != S_SLEEP)
        if xfer_complete_now:
            write_active_nxt = False
        elif reg_we and (reg_addr == A_DR) and self.fsm_busy_ir:
            write_active_nxt = True
        else:
            write_active_nxt = self.write_active

        # ---- commit ---------------------------------------------------------
        self.ctrlr0, self.ctrlr1 = ctrlr0_nxt, ctrlr1_nxt
        self.ssienr, self.ser    = ssienr_nxt, ser_nxt
        self.baudr, self.txftlr  = baudr_nxt, txftlr_nxt
        self.start_xfer = start_xfer_nxt

        self.tx_wr_ptr, self.tx_rd_ptr = tx_wr_ptr_nxt, tx_rd_ptr_nxt
        self.tx_cnt = tx_cnt_nxt
        self.rx_wr_ptr, self.rx_rd_ptr = rx_wr_ptr_nxt, rx_rd_ptr_nxt
        self.rx_cnt = rx_cnt_nxt

        self.ssi_cnt = ssi_cnt_nxt
        self.sclk_out = sclk_out_nxt
        self.sclk_re_ir, self.sclk_fe_ir = sclk_re_nxt, sclk_fe_nxt

        self.c_state = c_state_nxt
        self.bit_cnt = bit_cnt_nxt
        self.frame_cnt = frame_cnt_nxt
        self.last_frame = last_frame_nxt
        self.x_done_1d = x_done
        self.tx_load_en = tx_load_en_nxt
        self.abort_ir = abort_nxt
        self.hold_cnt = hold_nxt
        self.fsm_sleep_ir = fsm_sleep_nxt
        self.fsm_busy_ir = fsm_busy_nxt
        self.ssi_oe_n_ir = ssi_oe_n_iw
        self.abort_seen = abort_seen_nxt
        self.refill_seen = refill_seen_nxt
        self.write_active = write_active_nxt

        self.tx_shift_reg, self.tx_buffer, self.txd = tx_shift_reg_nxt, tx_buffer_nxt, txd_nxt
        self.ss_n = ss_n_nxt
        self.tx_pop_ir = tx_pop_ir_nxt
        self.rx_push_ir = rx_push_ir_nxt

        self.prev_fsm_state = prev_fsm_state_nxt
        self.cov_xfer_complete = (c_state_nxt == S_IDLE) and (self.prev_fsm_state != S_IDLE) and \
                                 (self.prev_fsm_state != S_SLEEP)

    # ------------------------------------------------------------------
    # Coverage observation signals
    # ------------------------------------------------------------------
    def read_signals(self) -> Dict:
        tx_full  = (self.tx_cnt == 8)
        tx_empty = (self.tx_cnt == 0)
        rx_full  = (self.rx_cnt == 8)
        rx_empty = (self.rx_cnt == 0)
        scph = (self.ctrlr0 >> 7) & 1
        if ((self.ctrlr0 >> 5) & 0x3) == 0:
            protocol = 1 if scph else 0
        else:
            protocol = 2
        return {
            "cov_fsm_state_id": self.c_state,
            "cov_fsm_state":    (1 << self.c_state),
            "protocol":         protocol,
            "cov_sclk_re":      1 if self.sclk_re_ir else 0,
            "cov_sclk_fe":      1 if self.sclk_fe_ir else 0,
            "cov_sclk_out":     self.sclk_out,
            "cov_sclk_mask":    1 if self._sclk_mask_now() else 0,
            "cov_bit_cnt":      self.bit_cnt,
            "cov_frame_cnt":    self.frame_cnt,
            "cov_x_done":       1 if self._x_done_now() else 0,
            "cov_last_frame":   1 if self.last_frame else 0,
            "cov_tx_shift_en":  1 if self._tx_shift_en_now() else 0,
            "cov_rx_shift_en":  1 if self._rx_shift_en_now() else 0,
            "cov_load_tx_shift":1 if self._load_tx_shift_now() else 0,
            "cov_load_rx_buf":  1 if self._load_rx_buf_now() else 0,
            "cov_ss_active":    1 if (self.ss_n != 0xF) else 0,
            "cov_xfer_complete":1 if self.cov_xfer_complete else 0,
            "cov_txd":          self.txd,
            "cov_dfs":          self.ctrlr0 & 0x1F,
            "cov_eff_dfs":      max((self.ctrlr0 & 0x1F) + 1, self.spec_dfs_min),
            "cov_baudr":        self.baudr,
            "cov_baud2":        1 if (self.baudr == 2) else 0,
            "cov_ndf":          self.ctrlr1 & 0x1FFFF,
            "cov_tmod":         (self.ctrlr0 >> 9) & 0x3,
            "cov_frf":          (self.ctrlr0 >> 5) & 0x3,
            "cov_scph":         (self.ctrlr0 >> 7) & 1,
            "cov_scpol":        (self.ctrlr0 >> 8) & 1,
            "cov_srl_test":     (self.ctrlr0 >> 11) & 1,
            "cov_ss_tgl_en":    (self.ctrlr0 >> 12) & 1,
            "cov_ser":          self.ser,
            "cov_tx_level":     self.tx_cnt,
            "cov_rx_level":     self.rx_cnt,
            "cov_tx_empty":     1 if tx_empty else 0,
            "cov_tx_full":      1 if tx_full else 0,
            "cov_rx_empty":     1 if rx_empty else 0,
            "cov_rx_full":      1 if rx_full else 0,
            "cov_hold_ss_cnt":  self.hold_cnt,
            "cov_s0":          1 if self.abort_seen else 0,
            "cov_s1":          1 if self.refill_seen else 0,
            "cov_s2":          1 if self.write_active else 0,
            "cov_rx_data":      self.rx_mem[self.rx_rd_ptr],
            "cov_ss_in_n":      1 if self._in["ss_in_n"] else 0,
            "cov_start_xfer":   1 if self.start_xfer else 0,
            "reg_rdata":        self._read_mux_now(),
        }

    # -- combinational mirror helpers used by read_signals ------------------
    def _read_mux_now(self):
        """Post-cycle register read mux (mirrors spi_regfile always_comb).
        Evaluated on the *post-update* registered state with the address of the
        cycle just executed, matching the DUT's reg_rdata sampling point."""
        ra = self._in["reg_addr"]
        if ra == A_CTRLR0:
            return self.ctrlr0
        if ra == A_CTRLR1:
            return self.ctrlr1
        if ra == A_SSIENR:
            return self.ssienr
        if ra == A_SER:
            return self.ser
        if ra == A_BAUDR:
            return self.baudr
        if ra == A_TXFTLR:
            return self.txftlr
        if ra == A_TXFLR:
            return self.tx_cnt
        if ra == A_RXFLR:
            return self.rx_cnt
        if ra == A_SR:
            return ((self.rx_cnt == 8) << 4) | \
                   ((self.rx_cnt != 0) << 3) | \
                   ((self.tx_cnt == 0) << 2) | \
                   ((self.tx_cnt != 8) << 1) | \
                   self.fsm_busy_ir
        if ra == A_DR:
            return self.rx_mem[self.rx_rd_ptr]
        return 0

    # -- combinational mirror helpers used by read_signals ------------------
    def _sclk_mask_now(self):
        # recompute n_state from current registered state (same as step)
        return self._n_state_now() in (S_SHIFT_BIT, S_PUSH_RX, S_HOLD_MASK) or \
               (self._n_state_now() == S_POP_TX and ((self.ctrlr0 >> 5) & 3) == 0 and ((self.ctrlr0 >> 7) & 1)) or \
               (self._n_state_now() == S_POP_TX and ((self.ctrlr0 >> 5) & 3) == 1) or \
               (self._n_state_now() == S_TOGGLE_SS and ((self.ctrlr0 >> 5) & 3) == 1)

    def _x_done_now(self):
        tmod = (self.ctrlr0 >> 9) & 3
        frf = (self.ctrlr0 >> 5) & 3
        if ((tmod == 3) and (frf == 0)) or (tmod == 2):
            return self.frame_cnt == (self.ctrlr1 & 0x1FFFF)
        return self.last_frame

    def _tx_shift_en_now(self):
        scph = (self.ctrlr0 >> 7) & 1
        frf = (self.ctrlr0 >> 5) & 3
        sclk_re_int = self.sclk_fe_ir if (frf == 0 and scph == 0) else self.sclk_re_ir
        tmod = (self.ctrlr0 >> 9) & 3
        return (self.c_state == S_SHIFT_BIT) and sclk_re_int and ((((tmod >> 1) & 1) == 0) or tmod == 3)

    def _rx_shift_en_now(self):
        scph = (self.ctrlr0 >> 7) & 1
        frf = (self.ctrlr0 >> 5) & 3
        sclk_fe_int = self.sclk_re_ir if (frf == 0 and scph == 0) else self.sclk_fe_ir
        return ((self.c_state == S_SHIFT_BIT) and sclk_fe_int) or \
               ((self.c_state == S_PUSH_RX) and sclk_fe_int) or \
               ((self.c_state == S_TOGGLE_SS) and sclk_fe_int)

    def _load_tx_shift_now(self):
        # approximate for observation only (gated by tx_load_en/tx_empty)
        return self._load_tx_shift_core_now()

    def _load_tx_shift_core_now(self):
        scph = (self.ctrlr0 >> 7) & 1
        frf = (self.ctrlr0 >> 5) & 3
        sclk_re_int = self.sclk_fe_ir if (frf == 0 and scph == 0) else self.sclk_re_ir
        tmod = (self.ctrlr0 >> 9) & 3
        ssp = (frf == 1)
        tx_empty = (self.tx_cnt == 0)
        if self.tx_load_en and not tx_empty:
            sm = self._sclk_mask_now()
            return ((self.c_state == S_POP_TX) and sclk_re_int) or \
                   ((self.c_state == S_PUSH_RX) and sclk_re_int and (not self._x_done_now()) and (sm if ssp else True)) or \
                   ((self.c_state == S_TOGGLE_SS) and sclk_re_int and ssp) or \
                   ((self.c_state == S_HOLD_MASK) and sclk_re_int and (not self._x_done_now()))
        return False

    def _load_rx_buf_now(self):
        scph = (self.ctrlr0 >> 7) & 1
        frf = (self.ctrlr0 >> 5) & 3
        sclk_re_int = self.sclk_fe_ir if (frf == 0 and scph == 0) else self.sclk_re_ir
        sclk_fe_int = self.sclk_re_ir if (frf == 0 and scph == 0) else self.sclk_fe_ir
        spi0 = (frf == 0) and (scph == 0)
        ssp = (frf == 1)
        baud2 = (self.baudr == 2)
        tmod = (self.ctrlr0 >> 9) & 3
        gate = (tmod == 0) or (tmod == 2) or (tmod == 3)
        return gate and (
            ((self.c_state == S_PUSH_RX) and sclk_re_int) or
            ((self.c_state == S_TOGGLE_SS) and (sclk_re_int or baud2) and ssp) or
            ((self.c_state == S_HOLD_MASK) and (sclk_re_int or baud2)) or
            ((self.c_state == S_MASK_SCLK) and (not spi0) and sclk_re_int))

    def _n_state_now(self):
        """Recompute n_state from current registered state (no inputs change)."""
        # This mirrors the step() next-state logic; inputs are self._in
        return self._n_state_from(self._in["ss_in_n"] if "ss_in_n" in self._in else 1)

    def _n_state_from(self, ss_in_n):
        cs = self.c_state
        dfs_eff = max((self.ctrlr0 & 0x1F) + 1, self.spec_dfs_min)
        frf = (self.ctrlr0 >> 5) & 3
        scph = (self.ctrlr0 >> 7) & 1
        tmod = (self.ctrlr0 >> 9) & 3
        ss_tgl = (self.ctrlr0 >> 12) & 1
        ndf = self.ctrlr1 & 0x1FFFF
        baud2 = (self.baudr == 2)
        spi0 = (frf == 0) and (scph == 0)
        spi1 = (frf == 0) and (scph == 1)
        ssp = (frf == 1)
        sclk_re_int = self.sclk_fe_ir if spi0 else self.sclk_re_ir
        sclk_fe_int = self.sclk_re_ir if spi0 else self.sclk_fe_ir
        tx_empty = (self.tx_cnt == 0)
        tx_fifo_filled = 1 if tmod == 2 else (1 if self.tx_cnt > self.txftlr else 0)
        f_done = (self.bit_cnt == dfs_eff)
        if ((tmod == 3) and (frf == 0)) or (tmod == 2):
            x_done = (self.frame_cnt == ndf)
        else:
            x_done = self.last_frame
        if cs == S_IDLE:
            if (self.start_xfer and sclk_re_int and tx_fifo_filled and (not tx_empty) and
                    (((not ssp) and ss_in_n) or (ssp and (not ss_in_n)))):
                return S_ASSERT_SS
            return S_IDLE
        elif cs == S_ASSERT_SS:
            return S_POP_TX if (sclk_fe_int or baud2) else S_ASSERT_SS
        elif cs == S_POP_TX:
            return S_SHIFT_BIT if (sclk_re_int or baud2) else S_POP_TX
        elif cs == S_SHIFT_BIT:
            if f_done and (not ssp):
                return S_PUSH_RX
            elif ssp and f_done and (not x_done):
                return S_TOGGLE_SS
            elif ssp and f_done and self.x_done_1d:
                return S_PUSH_RX
            return S_SHIFT_BIT
        elif cs == S_PUSH_RX:
            if sclk_fe_int or baud2:
                if spi0 and (not baud2):
                    return S_HOLD_MASK
                elif (spi1 and x_done) or (spi0 and baud2 and (x_done or ss_tgl)) or ssp:
                    return S_MASK_SCLK
                elif spi0 and baud2 and (not x_done) and (not ss_tgl):
                    return S_SHIFT_BIT
                return S_PUSH_RX
            elif sclk_re_int or baud2:
                if (not x_done) and spi1:
                    return S_SHIFT_BIT
                return S_PUSH_RX
            return S_PUSH_RX
        elif cs == S_TOGGLE_SS:
            return S_SHIFT_BIT if (sclk_re_int or baud2) else S_TOGGLE_SS
        elif cs == S_HOLD_MASK:
            if sclk_re_int or baud2:
                if ss_tgl or ssp or spi1:
                    return S_MASK_SCLK
                elif x_done or self.abort_ir:
                    return S_MASK_SCLK
                return S_SHIFT_BIT
            return S_HOLD_MASK
        elif cs == S_MASK_SCLK:
            if (sclk_fe_int or baud2) and spi0 and (not x_done) and (not self.abort_ir):
                return S_TOGGLE_SS
            elif sclk_re_int and (ssp or (spi0 and (x_done or self.abort_ir))):
                return S_CLEAR_SS
            elif (sclk_re_int or baud2) and (spi1 or ssp):
                return S_HOLD_SS
            return S_MASK_SCLK
        elif cs == S_HOLD_SS:
            if (sclk_fe_int or baud2) and (self.hold_cnt == (self.spec_hold_ss - 1)):
                return S_CLEAR_SS
            return S_HOLD_SS
        elif cs == S_CLEAR_SS:
            return S_IDLE if (sclk_re_int or baud2) else S_CLEAR_SS
        elif cs == S_SLEEP:
            return S_IDLE if self.ssienr else S_SLEEP
        return S_IDLE

    # ------------------------------------------------------------------
    # Convenience helpers for test scripts
    # ------------------------------------------------------------------
    def write_reg(self, addr: int, data: int):
        """Perform a single-cycle register write."""
        self.step({"reg_we": 1, "reg_addr": addr, "reg_wdata": data})

    def read_reg(self, addr: int) -> int:
        """Perform a single-cycle register read."""
        self.step({"reg_re": 1, "reg_addr": addr})
        return self.reg_rdata

    def idle_step(self, n: int = 1):
        """Advance N cycles with no bus activity (ss_in_n high)."""
        for _ in range(n):
            self.step({})


def action_to_list(a: Dict) -> List[int]:
    """Convert dict action to the flat 13-dim action vector."""
    return [int(a.get(k, 0)) for k in ACTION_KEYS]


def action_to_dict(v: List[int]) -> Dict:
    """Convert flat 13-dim action vector to dict form."""
    return {k: int(v[i]) for i, k in enumerate(ACTION_KEYS)}


if __name__ == "__main__":
    sim = SpiXferPublic()

    # Configure SPI0, dfs=8 (8-bit frame; DFS field is frame-size-minus-1)
    sim.write_reg(A_CTRLR0, 0x0000_0007)      # dfs=7 -> 8-bit frame, frf=0, scph=0, tmod=0
    sim.write_reg(A_SER, 0x1)
    sim.write_reg(A_BAUDR, 8)
    sim.write_reg(A_TXFTLR, 0)
    sim.write_reg(A_DR, 0x0000_00AB)          # push TX data
    sim.write_reg(A_SSIENR, 1)                # enable SSI

    for i in range(300):
        sim.step({})
        sig = sim.read_signals()
        if sig["cov_xfer_complete"]:
            print(f"[{i}] transfer complete; state={sig['cov_fsm_state_id']} "
                  f"frames={sig['cov_frame_cnt']} rx={sig['cov_rx_data']:#x}")
            break
    else:
        print("no transfer complete in 300 cycles; "
              f"state={sim.c_state} ({STATE_NAMES[sim.c_state]})")
