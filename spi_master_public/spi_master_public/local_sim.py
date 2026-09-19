#!/usr/bin/env python3
"""
spi_master_public - Cycle-accurate Python model of the SPI master DUT.

Mirrors spi_m_top / spi_m_regfile / spi_m_intctl / spi_m_fifo /
spi_m_sclkgen / spi_m_mstfsm / spi_m_shift exactly (synchronous, posedge clk,
async reset).  `step(action)` applies one action (one clock cycle) and updates
the internal state; `read_signals()` returns the post-cycle coverage signal
dictionary compatible with coverage_simulator.py and run_verilator.py.

Timing model (matches the RTL nonblocking-assignment semantics):
  * phase 1: combinational nets are computed from the *pre-edge* registered
    state (regfile decode, FIFO status, interrupt status, sclkgen edge
    comparison, and the full FSM combinational cone).
  * phase 2: every register updates from pre-edge values (a software write,
    a tx_push, a sclk edge, ... are all sampled at the same posedge).
  * phase 3: combinational nets are refreshed from the post-edge state so that
    read_signals() reports the coverage observables of the completed cycle.

Hidden configuration defaults match the evaluation platform:
  SPEC_CFS_MIN     = 8   (minimum Microwire control frame size)
  SPEC_HOLD_SS     = 4   (HOLD_SS hold duration, half clock periods)
  SPEC_TXFTLR_DFLT = 1   (TXFTLR reset value)
"""

# --------------------------------------------------------------------------
# Register addresses (6-bit word address)
# --------------------------------------------------------------------------
A_CTRLR0   = 0x00
A_CTRLR1   = 0x01
A_SSIENR   = 0x02
A_MWCR     = 0x03
A_SER      = 0x04
A_BAUDR    = 0x05
A_TXFTLR   = 0x06
A_RXFTLR   = 0x07
A_TXFLR    = 0x08
A_RXFLR    = 0x09
A_SR       = 0x0A
A_IMR      = 0x0B
A_ISR      = 0x0C
A_RISR     = 0x0D
A_TXOICR   = 0x0E
A_RXOICR   = 0x0F
A_RXUICR   = 0x10
A_MSTICR   = 0x11
A_ICR      = 0x12
A_IDR      = 0x16
A_VIDR     = 0x17
A_DR       = 0x18          # DR word range 0x18..0x3B (24..59)

# --------------------------------------------------------------------------
# FSM state ids (cov_fsm_state_id encoding)
# --------------------------------------------------------------------------
S_IDLE         = 0
S_ASSERT_SS    = 1
S_POP_TX       = 2
S_SHIFT_CTRL   = 3
S_SHIFT_BIT    = 4
S_PUSH_RX      = 5
S_TOGGLE_SS    = 6
S_HOLD_MASK    = 7
S_MASK_SCLK    = 8
S_HOLD_SS      = 9
S_CLEAR_SS     = 10
S_SLEEP        = 11
S_MWPOP        = 12
S_WAIT_READY   = 13
S_CLEAR_READY  = 14

IDR_VAL  = 0x53315F31
VIDR_VAL = 0x01020101


class SpiMasterPublic:
    """Cycle-accurate model of spi_master_public."""

    def __init__(self, spec_cfs_min=8, spec_hold_ss=4, spec_txftlr_dflt=1):
        self.spec_cfs_min = spec_cfs_min & 0xF
        self.spec_hold_ss = spec_hold_ss & 0xF
        self.spec_txftlr_dflt = spec_txftlr_dflt & 0xF
        # FIFO memory is NOT cleared by reset in the RTL (spi_m_fifo.v resets
        # only the rd/wr pointers); initialise once to 0 (Verilator power-on
        # state) and preserve contents across resets exactly like the DUT.
        self.tx_mem = [0] * 8
        self.rx_mem = [0] * 8
        self._reset_all()
        # power-on state matches Verilator (all flops start at 0); reset
        # (which the first cycle of every test performs) re-establishes the
        # register reset values via _reset_all()
        self.sclk_active = 0
        self.sclk_out    = 0

    # -- public action interface ------------------------------------------
    def step(self, action: dict):
        """Apply one action (one clock cycle)."""
        # latch inputs
        self.we       = int(action.get("reg_we", 0)) & 1
        self.addr     = int(action.get("reg_addr", 0)) & 0x3F
        self.wdata    = int(action.get("reg_wdata", 0)) & 0xFFFFFFFF
        self.re       = int(action.get("reg_re", 0)) & 1
        self.rxd_in   = int(action.get("rxd", 0)) & 1
        self.ss_in_n  = int(action.get("ss_in_n", 1)) & 1
        self.rst_n    = int(action.get("rst_n", 1)) & 1

        if self.rst_n == 0:
            self._reset_all()
        else:
            self._tick()

    def read_signals(self) -> dict:
        """Return the post-cycle coverage signal dictionary."""
        s = {}
        s["cov_fsm_state_id"]   = self._state_id(self.c_state)
        s["cov_fsm_state"]      = 1 << self.c_state
        s["cov_sclk_re"]        = self.sclk_re
        s["cov_sclk_fe"]        = self.sclk_fe
        s["cov_sclk_out"]       = self.sclk_out
        s["cov_sclk_mask"]      = self.sclk_mask
        s["cov_bit_cnt"]        = self.bit_cnt
        s["cov_ctrl_cnt"]       = self.ctrl_cnt
        s["cov_frame_cnt"]      = self.frame_cnt
        s["cov_f_done"]         = self.f_done
        s["cov_c_done"]         = self.c_done
        s["cov_x_done"]         = self.x_done
        s["cov_last_frame"]     = self.last_frame
        s["cov_tx_shift_en"]    = self.tx_shift_en
        s["cov_rx_shift_en"]    = self.rx_shift_en
        s["cov_load_tx_shift"]  = self.load_tx_shift
        s["cov_load_tx_buffer"] = self.load_tx_buffer
        s["cov_load_rx_buf"]    = self.load_rx_buf
        s["cov_s3"]             = self.load_start_bit
        s["cov_txd"]            = self.txd
        s["cov_ss_in_n"]        = self.ss_in_n
        s["cov_ss_active"]      = (self.ss_n != 0) if self.frf == 1 \
            else (((~self.ss_n) & 0xF) != 0)
        s["cov_dfs"]            = self.dfs
        s["cov_eff_dfs"]        = self.eff_dfs
        s["cov_cfs"]            = self.cfs_raw
        s["cov_eff_cfs"]        = self.eff_cfs
        s["cov_baudr"]          = self.baudr
        s["cov_baud2"]          = self.baud2
        s["cov_ndf"]            = self.ndf
        s["cov_tmod"]           = self.tmod
        s["cov_frf"]            = self.frf
        s["cov_scph"]           = self.scph
        s["cov_scpol"]          = self.scpol
        s["cov_srl_test"]       = self.srl_test
        s["cov_ss_tgl_en"]      = self.ss_tgl_en
        s["cov_slv_oe"]         = self.slv_oe
        s["cov_ser"]            = self.ser
        s["cov_tx_level"]       = self.tx_level
        s["cov_rx_level"]       = self.rx_level
        s["cov_tx_full"]        = self.tx_full
        s["cov_tx_empty"]       = self.tx_empty
        s["cov_rx_full"]        = self.rx_full
        s["cov_rx_empty"]       = self.rx_empty
        s["cov_tx_overflow"]    = self.tx_overflow
        s["cov_rx_overflow"]    = self.rx_overflow
        s["cov_rx_underflow"]   = self.rx_underflow
        s["cov_risr"]           = self.risr
        s["cov_isr"]            = self.isr
        s["cov_imr"]            = self.imr
        s["cov_ssi_txe_intr"]   = 0 if (self.isr & 1) else 1
        s["cov_ssi_mst_intr"]   = 0 if (self.isr & 0x20) else 1
        s["cov_s1"]             = self.mst_contention
        s["cov_multi_mst_sync"] = self.multi_mst_sync
        s["cov_s2"]             = self.dcol
        s["cov_hold_ss_cnt"]    = self.hold_ss_cnt
        s["cov_start_xfer"]     = self.start_xfer
        s["cov_fsm_busy"]       = self.fsm_busy
        s["cov_fsm_sleep"]      = self.fsm_sleep
        s["cov_ssi_busy"]       = self.fsm_busy_sr
        s["cov_ssi_sleep"]      = self.ssi_sleep
        s["cov_rx_push"]        = self.rx_push
        s["cov_tx_pop"]         = self.tx_pop
        s["cov_rx_data"]        = self.rx_pop_data
        s["cov_mwcr"]           = self.mwcr
        s["cov_spi0_control"]   = self.spi0_control
        s["cov_spi1_control"]   = self.spi1_control
        s["cov_txftlr"]         = self.txftlr
        s["cov_s0"]             = self.abort_seen
        # derived protocol class
        s["protocol"] = self.protocol
        # register read data (combinational read mux)
        s["reg_rdata"] = self._read_mux()
        return s

    # -- reset -------------------------------------------------------------
    def _reset_all(self):
        # sclkgen reset drives sclk_out <= !sclk_active using the *pre-reset*
        # sclk_active (at power-on Verilator flops start at 0 -> sclk_out=1 on
        # the first reset edge, 0 on later resets).
        pre_reset_sclk_active = getattr(self, 'sclk_active', 0)
        # regfile
        self.ctrlr0       = 0x0808
        self.ctrlr1       = 0
        self.ssienr       = 0
        self.mwcr         = 0
        self.ser          = 0
        self.imr          = 0x3F
        self.txftlr       = self.spec_txftlr_dflt & 0xF
        self.rxftlr       = 0
        self.baudr        = 0
        self.baud2        = 0
        self.dcol         = 0
        self.multi_mst_edge = 0
        self.ssi_sleep    = 1
        self.tx_push      = 0
        self.tx_push_data = 0
        self.fsm_busy_sr  = 0
        self.sclk_active  = 1
        self.txflr        = 0
        self.rxflr        = 0
        self.start_xfer   = 0
        # fifo pointers only (memory contents preserved across reset, RTL)
        self.tx_rd = 0
        self.tx_wr = 0
        self.rx_rd = 0
        self.rx_wr = 0
        # intctl
        self.irisr_tx_empty   = 0
        self.irisr_txo        = 0
        self.irisr_rxu        = 0
        self.irisr_rxo        = 0
        self.irisr_rx_full    = 0
        self.irisr_mst        = 0
        self.mst_contention   = 0
        # sclkgen
        self.ssi_cnt    = 0
        self.sclk_out   = 1 - pre_reset_sclk_active
        self.sclk_re    = 0
        self.sclk_fe    = 0
        # shift
        self.ss_n              = 0xF
        self.tx_pop            = 0
        self.rx_push           = 0
        self.tx_shift_reg      = 0
        self.tx_buffer         = 0
        self.rx_shift_reg      = 0
        self.rx_buffer         = 0
        self.txd               = 0
        self.load_start_bit_ir = 0
        # fsm
        self.c_state      = S_IDLE
        self.bit_cnt      = 0
        self.ctrl_cnt     = 0
        self.frame_cnt    = 0
        self.last_frame   = 0
        self.x_done_1d    = 1
        self.tx_load_en   = 1
        self.c_done_ir    = 0
        self.f_done_ir    = 0
        self.abort_ir     = 0
        self.abort_seen   = 0
        self.spi0_control = 0
        self.spi1_control = 0
        self.rxd_sync1    = 0
        self.rxd_sync2    = 0
        self.hold_ss_cnt  = 0
        self.fsm_sleep    = 0
        self.fsm_busy     = 0
        self.ssi_oe_n     = 1
        self.fsm_multi_mst = 0
        # latched inputs
        self.we = 0
        self.addr = 0
        self.wdata = 0
        self.re = 0
        self.rxd_in = 0
        self.ss_in_n = 1
        self.rst_n = 1
        # combinational snapshot
        self.sclk_mask = 0
        self.tx_dfs    = 0
        self._refresh_combo()
        self._fsm_combo()

    # -- derived config ----------------------------------------------------
    def _refresh_combo(self):
        """Compute all combinational nets from current registered state."""
        self.dfs       = self.ctrlr0 & 0x1F
        self.eff_dfs   = self.dfs
        self.frf       = (self.ctrlr0 >> 6) & 0x3
        self.scph      = (self.ctrlr0 >> 8) & 0x1
        self.scpol     = (self.ctrlr0 >> 9) & 0x1
        self.tmod      = (self.ctrlr0 >> 10) & 0x3
        self.slv_oe    = (self.ctrlr0 >> 12) & 0x1
        self.srl_test  = (self.ctrlr0 >> 13) & 0x1
        self.ss_tgl_en = (self.ctrlr0 >> 14) & 0x1
        self.cfs_raw   = (self.ctrlr0 >> 16) & 0xF
        self.eff_cfs   = max(self.cfs_raw, self.spec_cfs_min)
        self.ndf       = self.ctrlr1 + 1

        self.ssp   = (self.frf == 1)
        self.spi0  = (self.frf == 0 and self.scph == 0)
        self.spi1  = (self.frf == 0 and self.scph == 1)
        self.mwire = (self.frf == 2)

        if self.spi0:
            self.protocol = 0
        elif self.spi1:
            self.protocol = 1
        elif self.ssp:
            self.protocol = 2
        else:
            self.protocol = 3

        # fifo combinational
        self.tx_full  = ((self.tx_wr & 7) == (self.tx_rd & 7)) and \
                        ((self.tx_wr >> 3) != (self.tx_rd >> 3))
        self.tx_empty = (self.tx_wr == self.tx_rd)
        self.rx_full  = ((self.rx_wr & 7) == (self.rx_rd & 7)) and \
                        ((self.rx_wr >> 3) != (self.rx_rd >> 3))
        self.rx_empty = (self.rx_wr == self.rx_rd)
        self.tx_level = (self.tx_wr - self.tx_rd) & 0xF
        self.rx_level = (self.rx_wr - self.rx_rd) & 0xF
        self.rx_pop   = 1 if (self._is_dr(self.addr) and self.re and
                              self.ssienr) else 0
        self.rx_overflow  = 1 if (self.rx_push and self.rx_full and
                                  not self.rx_pop) else 0
        self.rx_underflow = 1 if (self.rx_pop and self.rx_empty) else 0
        self.tx_pop_data  = self.tx_mem[self.tx_rd & 7]
        self.rx_pop_data  = self.rx_mem[self.rx_rd & 7]

        # intctl combinational
        self.risr = (self.irisr_mst << 5) | (self.irisr_rx_full << 4) | \
                    (self.irisr_rxo << 3) | (self.irisr_rxu << 2) | \
                    (self.irisr_txo << 1) | self.irisr_tx_empty
        self.isr  = self.imr & self.risr
        self.multi_mst_sync = self.fsm_multi_mst ^ self.multi_mst_edge

        self.tx_overflow = self._calc_tx_overflow()

    def _calc_tx_overflow(self):
        dr_we     = 1 if (self.we and self._is_dr(self.addr)) else 0
        tx_push_c = 1 if (dr_we and self.ssienr) else 0
        tx_wc2    = self.tx_level & 7
        return 1 if (dr_we and (self.tx_full or
                   ((tx_push_c or self.tx_push) and tx_wc2 == 7) or
                   (tx_push_c and self.tx_push and tx_wc2 == 6))) else 0

    def _is_dr(self, addr):
        return 0x18 <= addr <= 0x3B

    # -- FSM combinational cone ---------------------------------------------
    def _fsm_combo(self):
        """Compute the master-FSM combinational outputs from the current
        (pre-edge) registered state.  Re-run in phase 3 to refresh them for
        read_signals()."""
        c = self.c_state

        # edge muxing: SPI0 / Microwire data is shifted on the opposite edge
        sclk_re_int = self.sclk_fe if (self.spi0 or self.mwire) else self.sclk_re
        sclk_fe_int = self.sclk_re if (self.spi0 or self.mwire) else self.sclk_fe
        self.sclk_re_int = sclk_re_int
        self.sclk_fe_int = sclk_fe_int

        # -- data frame done
        if self.mwire and (self.baud2 or (self.mwcr & 3) == 3):
            f_done = 1 if (self.bit_cnt == self.dfs + 1) else 0
        elif self.mwire and (self.mwcr & 3) == 2:
            f_done = self.f_done_ir
        else:
            f_done = 1 if (self.bit_cnt == self.dfs) else 0
        self.f_done = f_done
        sspf_done = 1 if (self.bit_cnt == (self.dfs - 1)) else 0

        # -- control frame done
        c_done = 1 if ((self.mwire and self.ctrl_cnt == self.eff_cfs) or
                       self.c_done_ir) else 0
        self.c_done = c_done

        # -- last_frame (next registered value)
        if self.tx_empty and (sclk_fe_int or self.baud2) and c != S_MASK_SCLK:
            last_frame_n = 1
        elif (not self.tx_empty) and c in (S_SHIFT_CTRL, S_SHIFT_BIT, S_IDLE):
            last_frame_n = 0
        else:
            last_frame_n = self.last_frame

        # -- x_done
        early_last_frame = self.last_frame
        if ((self.tmod == 3 and self.frf == 0) or
                (self.tmod == 2 and not self.mwire) or
                (self.mwire and (self.mwcr & 1) and not (self.mwcr & 2))):
            x_done = 1 if (self.frame_cnt == self.ndf) else 0
        else:
            x_done = early_last_frame
        self.x_done = x_done

        # -- f_done_ir / c_done_ir (next registered values)
        f_done_ir_n = 1 if (self.bit_cnt == self.dfs + 1) else 0
        if self.ctrl_cnt == self.eff_cfs and c == S_SHIFT_CTRL:
            c_done_ir_n = 1
        elif c != S_SHIFT_CTRL:
            c_done_ir_n = 0
        else:
            c_done_ir_n = self.c_done_ir

        # -- counter controls
        inc_frame_cnt = 1 if (c == S_SHIFT_BIT and
                              self.bit_cnt == self.dfs - 2 and
                              (self.tmod != 3 or
                               (self.tmod == 3 and
                                ((self.scph == 0 and self.spi0_control == 0) or
                                 (self.scph == 1 and self.spi1_control == 0)))) and
                              sclk_re_int) else 0
        inc_bit_cnt = 1 if ((c == S_MWPOP and sclk_re_int) or
                            (c == S_PUSH_RX and self.baud2 and self.mwire) or
                            (c == S_SHIFT_BIT and sclk_re_int)) else 0
        clr_bit_cnt = 1 if ((c == S_TOGGLE_SS and sclk_re_int and
                             (self.ssp or self.mwire or self.ss_tgl_en)) or
                            (c == S_HOLD_MASK and sclk_re_int and
                             (not self.ssp) and (not self.mwire) and
                             (not self.ss_tgl_en)) or
                            (c == S_PUSH_RX and sclk_re_int) or
                            (c == S_IDLE and sclk_re_int) or
                            (c == S_SHIFT_CTRL and sclk_fe_int) or
                            (c == S_MWPOP and (self.mwcr & 1) and
                             (sclk_fe_int or self.baud2)) or
                            (c == S_TOGGLE_SS and (sclk_fe_int or self.baud2) and
                             self.mwire)) else 0
        inc_ctrl_cnt = 1 if (c == S_SHIFT_CTRL and sclk_re_int) else 0
        clr_ctrl_cnt = 1 if ((c == S_SHIFT_BIT and sclk_re_int) or
                             (c == S_PUSH_RX and sclk_re_int) or
                             (c == S_IDLE and sclk_re_int) or
                             (c == S_TOGGLE_SS and sclk_fe_int)) else 0

        # -- next state
        n_state = self._next_state(c, self.tx_empty, sclk_re_int, sclk_fe_int,
                                   f_done, c_done, x_done, sspf_done)
        self.n_state = n_state

        # -- abort
        if c == S_HOLD_MASK and x_done:
            abort_n = 1
        elif c in (S_IDLE, S_CLEAR_SS):
            abort_n = 0
        else:
            abort_n = self.abort_ir

        # -- tx_load_en
        if c == S_POP_TX:
            if sclk_re_int or self.baud2:
                tx_load_en_n = 0 if ((not self.mwire) and self.tmod == 2) else 1
            else:
                tx_load_en_n = self.tx_load_en
        elif c == S_IDLE:
            tx_load_en_n = 1
        else:
            tx_load_en_n = self.tx_load_en

        # -- tx_dfs (data or control frame size for the txd mux)
        self.tx_dfs = self.eff_cfs if n_state == S_SHIFT_CTRL else self.dfs

        # -- start_xfer (regfile combinational)
        tx_empty_early = 1 if ((not self.tx_push) and self.tx_pop and
                               self.tx_level == 1) else 0
        slv_sel = 1 if (self.ser != 0) else 0
        if self.tmod != 2:
            tx_fifo_ready = 1 if (self.tx_level > (self.txftlr & 0x7)) else 0
        else:
            tx_fifo_ready = 0 if self.tx_empty else 1
        start_xfer_n = 1 if (slv_sel and not tx_empty_early and
                             not self.mst_contention and tx_fifo_ready) else 0

        # -- FSM outputs (combinational)
        tx_shift_en = 1 if ((c == S_SHIFT_BIT and sclk_re_int and
                             (self.mwire or (self.tmod >> 1) == 0 or
                              (self.tmod == 3 and
                               (self.spi0_control or self.spi1_control)))) or
                            (c == S_SHIFT_CTRL and sclk_re_int)) else 0
        if self.mwire and (self.mwcr & 2) and not self.srl_test:
            rx_shift_en = 0
        else:
            rx_shift_en = 1 if ((c == S_SHIFT_CTRL and
                                 (sclk_fe_int or self.baud2) and (self.mwcr & 2)) or
                                (c == S_SHIFT_BIT and sclk_fe_int) or
                                (c == S_PUSH_RX and sclk_fe_int) or
                                (c == S_TOGGLE_SS and sclk_fe_int and
                                 (not self.mwire))) else 0

        if self.tx_load_en and not self.tx_empty:
            load_tx_shift = 1 if ((c == S_POP_TX and sclk_re_int) or
                                  (c == S_PUSH_RX and sclk_re_int and not x_done) or
                                  (c == S_TOGGLE_SS and sclk_re_int and self.ssp) or
                                  (c == S_MWPOP and (sclk_re_int or self.baud2)) or
                                  (c == S_HOLD_MASK and sclk_re_int and
                                   (not self.mwire) and (not x_done) and
                                   (not abort_n))) else 0
        else:
            load_tx_shift = 0

        if not self.tx_load_en:
            load_tx_buffer = 1 if ((c == S_POP_TX and sclk_re_int) or
                                   (c == S_PUSH_RX and sclk_re_int and not x_done) or
                                   (c == S_TOGGLE_SS and sclk_re_int and self.ssp) or
                                   (c == S_HOLD_MASK and sclk_re_int and
                                    (not self.mwire) and (not x_done))) else 0
        else:
            load_tx_buffer = 0

        load_rx_cond = (self.tmod == 2 or self.tmod == 0 or self.mwire or
                        (self.tmod == 3 and
                         ((self.scph == 1 and self.spi1_control == 0) or
                          (self.scph == 0 and self.spi0_control == 0))))
        if load_rx_cond:
            load_rx_buf = 1 if ((c == S_MWPOP and (sclk_re_int or self.baud2) and
                                 self.srl_test) or
                                (c == S_PUSH_RX and sclk_re_int) or
                                (c == S_TOGGLE_SS and (sclk_re_int or self.baud2) and
                                 (self.ssp or (self.mwire and self.srl_test))) or
                                (c == S_HOLD_MASK and (sclk_re_int or self.baud2) and
                                 ((not self.mwire) or
                                  (self.mwire and self.srl_test))) or
                                (c == S_MASK_SCLK and (not self.spi0) and
                                 sclk_re_int and (not self.mwire))) else 0
        else:
            load_rx_buf = 0

        # slave select
        if self.spi0 or self.spi1:
            fsm_slv_sel = 1 if n_state not in (S_IDLE, S_CLEAR_SS, S_SLEEP,
                                               S_TOGGLE_SS) else 0
        elif self.ssp:
            fsm_slv_sel = 1 if (n_state in (S_ASSERT_SS, S_POP_TX, S_TOGGLE_SS) or
                                (n_state == S_SHIFT_BIT and sspf_done and
                                 not x_done and sclk_re_int)) else 0
        else:
            fsm_slv_sel = 1 if n_state not in (S_IDLE, S_CLEAR_SS, S_SLEEP) else 0

        # output enable
        if self.spi0 or self.spi1:
            ssi_oe_n_iw = 1 if n_state in (S_IDLE, S_CLEAR_SS, S_SLEEP) else 0
        elif self.ssp:
            ssi_oe_n_iw = 1 if n_state in (S_IDLE, S_ASSERT_SS, S_CLEAR_SS,
                                           S_SLEEP) else 0
        else:
            ssi_oe_n_iw = 1 if (n_state in (S_IDLE, S_SLEEP) or
                                (n_state == S_SHIFT_BIT and not (self.mwcr & 2)) or
                                ((not self.baud2) and sclk_fe_int and
                                 n_state == S_TOGGLE_SS) or
                                (self.ssi_oe_n and n_state == S_TOGGLE_SS) or
                                (n_state == S_MASK_SCLK and not (self.mwcr & 2)) or
                                (n_state == S_HOLD_MASK and not (self.mwcr & 2)) or
                                n_state == S_CLEAR_SS or n_state == S_PUSH_RX) else 0

        # clock mask
        sclk_mask = 1 if (n_state == S_SHIFT_BIT or n_state == S_PUSH_RX or
                          (n_state == S_MWPOP and c != S_WAIT_READY) or
                          (c == S_MWPOP and n_state == S_HOLD_MASK) or
                          n_state == S_HOLD_MASK or n_state == S_SHIFT_CTRL or
                          (n_state == S_POP_TX and (self.spi1 or self.ssp)) or
                          (n_state == S_CLEAR_READY and self.tx_empty) or
                          (n_state == S_TOGGLE_SS and (self.ssp or self.mwire))) else 0

        load_start_bit = 1 if (c == S_WAIT_READY and sclk_fe_int and
                               (self.tx_empty or (self.mwcr & 1))) else 0

        # multi-master
        fsm_multi_mst_iw = 1 if (n_state not in (S_IDLE, S_SLEEP) and
                                 ((self.ss_in_n == 1 and self.ssp) or
                                  (self.ss_in_n == 0 and not self.ssp))) else 0

        fsm_sleep_n = 1 if n_state == S_SLEEP else 0
        fsm_busy_n  = 1 if (n_state != S_IDLE and n_state != S_SLEEP) else 0

        # -- store combinational snapshot (used by phase 2 commit and
        #    read_signals after the phase-3 refresh)
        self.sclk_mask      = sclk_mask
        self.tx_shift_en    = tx_shift_en
        self.rx_shift_en    = rx_shift_en
        self.load_tx_shift  = load_tx_shift
        self.load_tx_buffer = load_tx_buffer
        self.load_rx_buf    = load_rx_buf
        self.load_start_bit = load_start_bit
        self.fsm_slv_sel    = fsm_slv_sel
        self.ssi_oe_n_iw    = ssi_oe_n_iw
        self.fsm_multi_mst_iw = fsm_multi_mst_iw
        self.fsm_sleep_n    = fsm_sleep_n
        self.fsm_busy_n     = fsm_busy_n
        self.start_xfer_n   = start_xfer_n
        self.last_frame_n   = last_frame_n
        self.f_done_ir_n    = f_done_ir_n
        self.c_done_ir_n    = c_done_ir_n
        self.abort_n        = abort_n
        self.tx_load_en_n   = tx_load_en_n
        self.inc_frame_cnt  = inc_frame_cnt
        self.inc_bit_cnt    = inc_bit_cnt
        self.clr_bit_cnt    = clr_bit_cnt
        self.inc_ctrl_cnt   = inc_ctrl_cnt
        self.clr_ctrl_cnt   = clr_ctrl_cnt

    # -- one clock cycle ----------------------------------------------------
    def _tick(self):
        """Synchronous update: phase 1 (pre-edge combinational), phase 2
        (register commit from pre-edge values), phase 3 (post-edge refresh)."""

        # ==================================================================
        # Phase 1: combinational nets from the pre-edge registered state.
        # ==================================================================
        self._refresh_combo()
        self._fsm_combo()

        # regfile decode wires (current-cycle address/data decode)
        ctrlr0_we = (self.addr == A_CTRLR0) and self.we
        ctrlr1_we = (self.addr == A_CTRLR1) and self.we
        ssienr_we = (self.addr == A_SSIENR) and self.we
        mwcr_we   = (self.addr == A_MWCR)   and self.we
        ser_we    = (self.addr == A_SER)    and self.we
        baudr_we  = (self.addr == A_BAUDR)  and self.we
        txftlr_we = (self.addr == A_TXFTLR) and self.we
        rxftlr_we = (self.addr == A_RXFTLR) and self.we
        dr_we     = 1 if (self._is_dr(self.addr) and self.we) else 0
        imr_we    = (self.addr == A_IMR)    and self.we
        rdec_dr   = self._is_dr(self.addr)
        rd_en     = self.re

        set0_imr5 = 1 if (imr_we and not (self.wdata & 0x20)) else 0
        set1_imr5 = 1 if (imr_we and (self.wdata & 0x20)) else 0

        rdec_icr_en    = (self.addr == A_ICR)
        rdec_txoicr_en = (self.addr == A_TXOICR)
        rdec_rxoicr_en = (self.addr == A_RXOICR)
        rdec_rxuicr_en = (self.addr == A_RXUICR)
        rdec_msticr_en = (self.addr == A_MSTICR)

        # pre-edge registered-state captures
        pre_ssienr        = self.ssienr
        pre_baudr         = self.baudr
        pre_sclk_active   = self.sclk_active
        pre_frf           = self.frf
        pre_ser           = self.ser
        pre_dfs           = self.dfs
        pre_scpol         = self.scpol
        pre_srl_test      = self.srl_test
        pre_imr           = self.imr
        pre_mwcr          = self.mwcr
        pre_tx_push       = self.tx_push
        pre_tx_push_data  = self.tx_push_data
        pre_tx_pop        = self.tx_pop
        pre_rx_push       = self.rx_push
        pre_tx_pop_data   = self.tx_pop_data
        pre_ssi_cnt       = self.ssi_cnt
        pre_sclk_out      = self.sclk_out
        pre_sclk_mask     = self.sclk_mask
        pre_multi_mst_sync = self.multi_mst_sync
        pre_fsm_multi_mst  = self.fsm_multi_mst
        pre_fsm_busy       = self.fsm_busy
        pre_fsm_sleep      = self.fsm_sleep
        pre_irisr_mst      = self.irisr_mst
        pre_rxd_sync1      = self.rxd_sync1
        pre_tx_empty       = self.tx_empty
        pre_tx_full        = self.tx_full
        pre_rx_full        = self.rx_full
        pre_rx_empty       = self.rx_empty
        pre_tx_level       = self.tx_level
        pre_rx_level       = self.rx_level
        pre_rx_pop         = self.rx_pop
        pre_tx_overflow    = self.tx_overflow
        pre_rx_overflow    = self.rx_overflow
        pre_rx_underflow   = self.rx_underflow
        pre_txd            = self.txd
        pre_rx_buffer      = self.rx_buffer
        pre_fsm_slv_sel    = self.fsm_slv_sel
        pre_load_tx_shift  = self.load_tx_shift
        pre_load_start_bit = self.load_start_bit
        pre_tx_shift_en    = self.tx_shift_en
        pre_load_tx_buffer = self.load_tx_buffer
        pre_rx_shift_en    = self.rx_shift_en
        pre_load_rx_buf    = self.load_rx_buf
        pre_tx_shift_reg   = self.tx_shift_reg
        pre_tx_buffer      = self.tx_buffer
        pre_rx_shift_reg   = self.rx_shift_reg
        pre_load_start_bit_ir = self.load_start_bit_ir
        pre_ssp            = self.ssp
        pre_tx_dfs         = self.tx_dfs
        pre_baud2          = self.baud2
        pre_hold_ss_cnt    = self.hold_ss_cnt

        # tx_push / rx_pop are gated by the *pre-edge* ssienr
        tx_push_c = 1 if (dr_we and pre_ssienr) else 0
        rx_pop_c  = 1 if (rdec_dr and rd_en and pre_ssienr) else 0

        # fifo soft reset
        fifo_rst = 1 if ((pre_ssienr and not ssienr_we) or
                         (pre_ssienr and ssienr_we and (self.wdata & 1))) else 0

        # ==================================================================
        # Phase 2: compute all next-state values from pre-edge captures.
        # ==================================================================
        # ---- FSM ----
        if pre_ssienr == 0:
            c_state_n = S_SLEEP
        elif (pre_ssp and self.ss_in_n == 1) or \
                ((not pre_ssp) and self.ss_in_n == 0):
            c_state_n = S_IDLE
        else:
            c_state_n = self.n_state

        if self.clr_bit_cnt and self.inc_bit_cnt and self.mwire:
            bit_cnt_n = 1
        elif self.clr_bit_cnt:
            bit_cnt_n = 0
        elif self.inc_bit_cnt:
            bit_cnt_n = (self.bit_cnt + 1) & 0x3F
        else:
            bit_cnt_n = self.bit_cnt

        if self.clr_ctrl_cnt:
            ctrl_cnt_n = 0
        elif self.inc_ctrl_cnt:
            ctrl_cnt_n = (self.ctrl_cnt + 1) & 0xF
        else:
            ctrl_cnt_n = self.ctrl_cnt

        if (self.c_state == S_CLEAR_SS and self.n_state != S_CLEAR_SS) or \
                self.c_state == S_IDLE:
            frame_cnt_n = 0
        elif self.inc_frame_cnt:
            frame_cnt_n = (self.frame_cnt + 1) & 0x1FFFF
        else:
            frame_cnt_n = self.frame_cnt

        last_frame_n  = self.last_frame_n
        x_done_1d_n   = self.x_done
        tx_load_en_n  = self.tx_load_en_n
        c_done_ir_n   = self.c_done_ir_n
        f_done_ir_n   = self.f_done_ir_n
        abort_n       = self.abort_n

        # sticky abort-seen flag (mirrors RTL wrong_pol / P_CLEAR_SS)
        if ((pre_ssp and self.ss_in_n == 1) or
                ((not pre_ssp) and self.ss_in_n == 0)) and \
                self.c_state not in (S_IDLE, S_SLEEP):
            abort_seen_n = 1
        elif self.c_state == S_CLEAR_SS:
            abort_seen_n = 0
        else:
            abort_seen_n = self.abort_seen

        rxd_sync1_n   = self.rxd_in
        rxd_sync2_n   = pre_rxd_sync1

        if self.c_state == S_HOLD_SS:
            if self.sclk_fe_int or pre_baud2:
                hold_ss_cnt_n = (pre_hold_ss_cnt + 1) & 0xF
            else:
                hold_ss_cnt_n = pre_hold_ss_cnt
        else:
            hold_ss_cnt_n = 0

        fsm_sleep_n = self.fsm_sleep_n
        fsm_busy_n  = self.fsm_busy_n
        ssi_oe_n_n  = self.ssi_oe_n_iw
        if self.fsm_multi_mst_iw:
            fsm_multi_mst_n = 1 - self.fsm_multi_mst
        else:
            fsm_multi_mst_n = self.fsm_multi_mst

        # spi0 / spi1 control flags
        if not pre_tx_empty:
            spi1_control_n = 1
        elif self.clr_bit_cnt:
            spi1_control_n = 0
        else:
            spi1_control_n = self.spi1_control
        if not pre_tx_empty:
            spi0_control_n = 1
        elif (self.c_state in (S_PUSH_RX, S_HOLD_MASK)) and self.sclk_re_int:
            spi0_control_n = 0
        else:
            spi0_control_n = self.spi0_control

        # ---- shift ----
        if pre_frf == 1:
            ss_n_n = pre_ser & (0xF if pre_fsm_slv_sel else 0)
        else:
            ss_n_n = (~(pre_ser & (0xF if pre_fsm_slv_sel else 0))) & 0xF

        tx_pop_n  = pre_load_tx_shift
        rx_push_n = 1 if (pre_ssienr and pre_load_rx_buf) else 0

        if pre_load_tx_shift:
            tx_sr_n  = pre_tx_pop_data
            tx_buf_n = pre_tx_pop_data
        elif pre_load_start_bit:
            tx_sr_n  = 0xFFFFFFFF
            tx_buf_n = pre_tx_buffer
        elif pre_tx_shift_en:
            # rotate within the dfs+1-bit window (RTL case(dfs))
            window   = (1 << (pre_dfs + 1)) - 1
            w        = pre_tx_shift_reg & window
            rot      = ((w << 1) | (w >> pre_dfs)) & window
            tx_sr_n  = (pre_tx_shift_reg & ~window) | rot
            tx_buf_n = pre_tx_buffer
        elif pre_load_tx_buffer:
            tx_sr_n  = pre_tx_buffer
            tx_buf_n = pre_tx_buffer
        else:
            tx_sr_n  = pre_tx_shift_reg
            tx_buf_n = pre_tx_buffer

        # txd (registered; reads the OLD shift register / buffer)
        if pre_load_start_bit_ir:
            txd_n = (pre_tx_shift_reg >> 15) & 1
        elif pre_load_tx_shift:
            txd_n = (pre_tx_pop_data >> pre_tx_dfs) & 1
        elif pre_load_tx_buffer:
            txd_n = (pre_tx_buffer >> pre_dfs) & 1
        elif pre_tx_shift_en:
            if 1 <= pre_tx_dfs <= 30:
                txd_n = (pre_tx_shift_reg >> (pre_tx_dfs - 1)) & 1
            else:
                txd_n = (pre_tx_shift_reg >> 30) & 1
        else:
            txd_n = pre_txd

        lsb_ir_n = pre_load_start_bit

        # rx shifter / buffer
        rxd_mst = pre_txd if pre_srl_test else self.rxd_in
        if pre_ssienr and pre_rx_shift_en:
            rx_sr_n = ((pre_rx_shift_reg & 0x7FFFFFFF) << 1) | (rxd_mst & 1)
        else:
            rx_sr_n = pre_rx_shift_reg
        if pre_ssienr and pre_load_rx_buf:
            rx_buf_n = self._zero_pad(pre_rx_shift_reg, pre_dfs)
        else:
            rx_buf_n = pre_rx_buffer

        # ---- sclkgen ----
        if pre_ssienr == 0:
            ssi_cnt_n = 0
        elif pre_ssi_cnt == ((pre_baudr - 1) & 0xFFFF):
            ssi_cnt_n = 0
        else:
            ssi_cnt_n = (pre_ssi_cnt + 1) & 0xFFFF

        if pre_ssienr == 0:
            sclk_out_n = 1 - pre_sclk_active
        elif pre_baudr == 0 or pre_sclk_mask == 0 or pre_ssi_cnt == 0:
            sclk_out_n = 1 - pre_sclk_active
        elif (pre_ssi_cnt & 0x7FFF) == ((pre_baudr >> 1) & 0x7FFF):
            sclk_out_n = pre_sclk_active
        else:
            sclk_out_n = pre_sclk_out

        if pre_ssienr == 0:
            sclk_re_n = 0
            sclk_fe_n = 0
        else:
            sclk_re_n = 1 if (pre_ssi_cnt ==
                              ((((pre_baudr >> 1) & 0x7FFF) - 1) & 0x7FFF)) else 0
            sclk_fe_n = 1 if (pre_ssi_cnt == ((pre_baudr - 1) & 0xFFFF)) else 0

        # ---- regfile ----
        ctrlr0_n = self.ctrlr0
        if ctrlr0_we and pre_ssienr == 0:
            wd = self.wdata
            if (wd >> 6) & 3 != 3:
                ctrlr0_n = (ctrlr0_n & ~(3 << 6)) | (((wd >> 6) & 3) << 6)
            if (wd & 0x1F) >= 3:
                ctrlr0_n = (ctrlr0_n & ~0x1F) | (wd & 0x1F)
            ctrlr0_n = (ctrlr0_n & ~(0x7F << 8)) | (((wd >> 8) & 0x7F) << 8)
            ctrlr0_n = (ctrlr0_n & ~(0xF << 16)) | (((wd >> 16) & 0xF) << 16)

        ctrlr1_n = self.ctrlr1
        if ctrlr1_we and pre_ssienr == 0:
            ctrlr1_n = self.wdata & 0xFFFF

        ssienr_n = self.ssienr
        if ssienr_we:
            ssienr_n = self.wdata & 1

        mwcr_n = self.mwcr
        if mwcr_we and pre_ssienr == 0:
            mwcr_n = self.wdata & 0x7

        ser_n = self.ser
        if ser_we:
            ser_n = self.wdata & 0xF

        baudr_n = self.baudr
        if baudr_we and pre_ssienr == 0:
            b = self.wdata & 0xFFFF
            baudr_n = (((b >> 1) & 0x7F) << 1) | (((b >> 8) & 0xFF) << 8)

        baud2_n = 1 if pre_baudr == 2 else 0

        txftlr_n = self.txftlr
        if txftlr_we and (self.wdata & 0xF) <= 7:
            txftlr_n = self.wdata & 0xF

        rxftlr_n = self.rxftlr
        if rxftlr_we and (self.wdata & 0xF) <= 7:
            rxftlr_n = self.wdata & 0xF

        imr_n = self.imr
        if imr_we:
            imr_n = self.wdata & 0x3F

        tx_push_n      = tx_push_c
        tx_push_data_n = self.wdata

        dcol_n = self.dcol
        if pre_multi_mst_sync:
            dcol_n = 1
        elif self.addr == A_SR and self.re:
            dcol_n = 0

        multi_mst_edge_n = pre_fsm_multi_mst
        fsm_busy_sr_n    = pre_fsm_busy
        ssi_sleep_n      = 1 if (pre_fsm_sleep and not pre_ssienr) else 0
        sclk_active_n    = 0 if (pre_scpol == 1 and pre_frf == 0) else 1
        start_xfer_n     = self.start_xfer_n

        if pre_ssienr == 0:
            txflr_n = 0
            rxflr_n = 0
        else:
            txflr_n = pre_tx_level
            rxflr_n = pre_rx_level

        # ---- intctl ----
        irisr_tx_empty_n = 0 if pre_ssienr == 0 else (1 if pre_tx_empty else 0)

        irisr_txo_n = self.irisr_txo
        if pre_ssienr == 0:
            irisr_txo_n = 0
        elif pre_tx_overflow:
            irisr_txo_n = 1
        elif (rdec_icr_en or rdec_txoicr_en) and rd_en:
            irisr_txo_n = 0

        irisr_rxu_n = self.irisr_rxu
        if pre_ssienr == 0:
            irisr_rxu_n = 0
        elif pre_rx_underflow:
            irisr_rxu_n = 1
        elif (rdec_icr_en or rdec_rxuicr_en) and rd_en:
            irisr_rxu_n = 0

        irisr_rxo_n = self.irisr_rxo
        if pre_ssienr == 0:
            irisr_rxo_n = 0
        elif pre_rx_overflow:
            irisr_rxo_n = 1
        elif (rdec_icr_en or rdec_rxoicr_en) and rd_en:
            irisr_rxo_n = 0

        irisr_rx_full_n = 0 if pre_ssienr == 0 else (1 if pre_rx_full else 0)

        irisr_mst_n = self.irisr_mst
        if pre_ssienr == 0:
            irisr_mst_n = 0
        elif pre_multi_mst_sync:
            irisr_mst_n = 1
        elif (rdec_icr_en or rdec_msticr_en) and rd_en:
            irisr_mst_n = 0

        mst_contention_n = self.mst_contention
        if pre_ssienr == 0:
            mst_contention_n = 0
        elif (pre_multi_mst_sync or pre_irisr_mst) and \
                (pre_imr & 0x20 or set1_imr5) and not set0_imr5:
            mst_contention_n = 1
        elif ((rdec_icr_en or rdec_msticr_en) and rd_en) or \
                (not pre_irisr_mst) or set0_imr5:
            mst_contention_n = 0

        # ==================================================================
        # Commit all registers (posedge clk).
        # ==================================================================
        # FSM
        self.c_state      = c_state_n
        self.bit_cnt      = bit_cnt_n
        self.ctrl_cnt     = ctrl_cnt_n
        self.frame_cnt    = frame_cnt_n
        self.last_frame   = last_frame_n
        self.x_done_1d    = x_done_1d_n
        self.tx_load_en   = tx_load_en_n
        self.c_done_ir    = c_done_ir_n
        self.f_done_ir    = f_done_ir_n
        self.abort_ir     = abort_n
        self.abort_seen   = abort_seen_n
        self.rxd_sync1    = rxd_sync1_n
        self.rxd_sync2    = rxd_sync2_n
        self.hold_ss_cnt  = hold_ss_cnt_n
        self.fsm_sleep    = fsm_sleep_n
        self.fsm_busy     = fsm_busy_n
        self.ssi_oe_n     = ssi_oe_n_n
        self.fsm_multi_mst = fsm_multi_mst_n
        self.spi0_control = spi0_control_n
        self.spi1_control = spi1_control_n

        # shift
        self.ss_n              = ss_n_n
        self.tx_pop            = tx_pop_n
        self.rx_push           = rx_push_n
        self.tx_shift_reg      = tx_sr_n
        self.tx_buffer         = tx_buf_n
        self.txd               = txd_n
        self.load_start_bit_ir = lsb_ir_n
        self.rx_shift_reg      = rx_sr_n
        self.rx_buffer         = rx_buf_n

        # sclkgen
        self.ssi_cnt  = ssi_cnt_n
        self.sclk_out = sclk_out_n
        self.sclk_re  = sclk_re_n
        self.sclk_fe  = sclk_fe_n

        # regfile
        self.ctrlr0       = ctrlr0_n
        self.ctrlr1       = ctrlr1_n
        self.ssienr       = ssienr_n
        self.mwcr         = mwcr_n
        self.ser          = ser_n
        self.baudr        = baudr_n
        self.baud2        = baud2_n
        self.txftlr       = txftlr_n
        self.rxftlr       = rxftlr_n
        self.imr          = imr_n
        self.tx_push      = tx_push_n
        self.tx_push_data = tx_push_data_n
        self.dcol         = dcol_n
        self.multi_mst_edge = multi_mst_edge_n
        self.fsm_busy_sr  = fsm_busy_sr_n
        self.ssi_sleep    = ssi_sleep_n
        self.sclk_active  = sclk_active_n
        self.start_xfer   = start_xfer_n
        self.txflr        = txflr_n
        self.rxflr        = rxflr_n

        # intctl
        self.irisr_tx_empty = irisr_tx_empty_n
        self.irisr_txo      = irisr_txo_n
        self.irisr_rxu      = irisr_rxu_n
        self.irisr_rxo      = irisr_rxo_n
        self.irisr_rx_full  = irisr_rx_full_n
        self.irisr_mst      = irisr_mst_n
        self.mst_contention = mst_contention_n

        # fifo (soft reset or normal operation, all pre-edge qualifiers)
        if fifo_rst == 0:
            self.tx_rd = 0
            self.tx_wr = 0
            self.rx_rd = 0
            self.rx_wr = 0
        else:
            if pre_tx_push and not pre_tx_full:
                self.tx_mem[self.tx_wr & 7] = pre_tx_push_data
                self.tx_wr = (self.tx_wr + 1) & 0xF
            if pre_tx_pop and not pre_tx_empty:
                self.tx_rd = (self.tx_rd + 1) & 0xF
            if pre_rx_push and not pre_rx_full:
                self.rx_mem[self.rx_wr & 7] = pre_rx_buffer
                self.rx_wr = (self.rx_wr + 1) & 0xF
            if rx_pop_c and not pre_rx_empty:
                self.rx_rd = (self.rx_rd + 1) & 0xF

        # ==================================================================
        # Phase 3: post-edge combinational refresh for read_signals().
        # ==================================================================
        self._refresh_combo()
        self._fsm_combo()

    # -- FSM next-state -----------------------------------------------------
    def _next_state(self, c, tx_empty, sclk_re_int, sclk_fe_int,
                    f_done, c_done, x_done, sspf_done):
        baud2 = self.baud2
        mwcr = self.mwcr
        if c == S_IDLE:
            if (self.start_xfer and sclk_re_int and
                    (self.tmod == 2 or
                     (self.tx_level > (self.txftlr & 0x7))) and
                    (not tx_empty) and
                    ((self.ssp and self.ss_in_n == 0) or
                     ((not self.ssp) and self.ss_in_n == 1))):
                return S_ASSERT_SS
            return S_IDLE
        elif c == S_ASSERT_SS:
            return S_POP_TX if (sclk_fe_int or baud2) else S_ASSERT_SS
        elif c == S_POP_TX:
            if sclk_re_int or baud2:
                return S_SHIFT_CTRL if self.mwire else S_SHIFT_BIT
            return S_POP_TX
        elif c == S_MWPOP:
            if baud2 and tx_empty:
                return S_HOLD_MASK
            if (sclk_re_int or baud2) and (not f_done or (mwcr & 1)):
                return S_SHIFT_BIT
            if (sclk_re_int or baud2) and f_done:
                return S_SHIFT_CTRL
            return S_MWPOP
        elif c == S_SHIFT_CTRL:
            if c_done and (not (mwcr & 2)) and (sclk_re_int or baud2):
                return S_TOGGLE_SS
            if c_done and (mwcr & 2) and (not tx_empty) and (sclk_fe_int or baud2):
                return S_MWPOP
            if c_done and (mwcr & 2) and tx_empty and (sclk_fe_int or baud2):
                return S_HOLD_MASK
            return S_SHIFT_CTRL
        elif c == S_SHIFT_BIT:
            if (f_done and (not self.ssp) and (not self.mwire)) or \
               (f_done and self.mwire and (not (mwcr & 2))) or \
               (f_done and self.x_done_1d and (not self.mwire) and self.ssp) or \
               (f_done and x_done and (not self.mwire) and (not self.ssp)):
                return S_PUSH_RX
            if f_done and self.mwire and (mwcr & 2) and (not (mwcr & 4)) and \
                    (not x_done):
                return S_MWPOP
            if f_done and self.mwire and (mwcr & 2) and (mwcr & 4) and \
                    (not x_done):
                return S_HOLD_MASK
            if x_done and self.mwire and (mwcr & 2) and f_done:
                return S_HOLD_MASK
            if self.ssp and f_done and (not x_done):
                return S_TOGGLE_SS
            return S_SHIFT_BIT
        elif c == S_PUSH_RX:
            if (sclk_fe_int or baud2) and self.spi0 and (not baud2):
                return S_HOLD_MASK
            if (sclk_fe_int or baud2) and \
                    ((self.spi1 and x_done) or
                     (self.spi0 and baud2 and (x_done or self.ss_tgl_en)) or
                     self.ssp):
                return S_MASK_SCLK
            if (sclk_re_int or baud2) and self.mwire and x_done:
                return S_MASK_SCLK
            if (sclk_re_int or baud2) and (not x_done) and self.spi1:
                return S_SHIFT_BIT
            if (sclk_re_int or baud2) and (not x_done) and self.mwire and \
                    (not (mwcr & 1)):
                return S_SHIFT_CTRL
            if ((sclk_re_int or baud2) and (not x_done) and self.mwire and
                    (mwcr & 1)) or \
               ((sclk_fe_int or baud2) and self.spi0 and baud2 and
                    (not x_done) and (not self.ss_tgl_en)):
                return S_SHIFT_BIT
            return S_PUSH_RX
        elif c == S_TOGGLE_SS:
            if (self.mwire and baud2 and sclk_fe_int) or \
               (self.mwire and (not baud2) and sclk_re_int) or \
               ((not self.mwire) and (sclk_re_int or baud2)):
                return S_SHIFT_BIT
            return S_TOGGLE_SS
        elif c == S_HOLD_MASK:
            if sclk_re_int or baud2:
                if self.ss_tgl_en:
                    return S_MASK_SCLK
                if self.ssp or self.mwire or self.spi1:
                    return S_MASK_SCLK
                if self.spi0 and (x_done or self.abort_ir):
                    return S_MASK_SCLK
                return S_SHIFT_BIT
            return S_HOLD_MASK
        elif c == S_MASK_SCLK:
            if (sclk_fe_int or baud2) and self.spi0 and (not x_done) and \
                    (not self.abort_ir):
                return S_TOGGLE_SS
            if (sclk_fe_int or baud2) and self.mwire:
                return S_CLEAR_SS
            if sclk_re_int and (self.ssp or
                                (self.spi0 and (x_done or self.abort_ir))):
                return S_CLEAR_SS
            if (sclk_re_int or baud2) and (self.spi1 or self.ssp):
                return S_HOLD_SS
            return S_MASK_SCLK
        elif c == S_HOLD_SS:
            if sclk_fe_int or baud2:
                return S_CLEAR_SS if (self.hold_ss_cnt >=
                                      (self.spec_hold_ss - 1)) else S_HOLD_SS
            return S_HOLD_SS
        elif c == S_CLEAR_SS:
            if self.mwire and (mwcr & 4) and (sclk_re_int or baud2):
                return S_WAIT_READY
            if sclk_re_int or baud2:
                return S_IDLE
            return S_CLEAR_SS
        elif c == S_WAIT_READY:
            if self.rxd_sync2 == 1 and (not (mwcr & 1)) and (not tx_empty) and \
                    sclk_fe_int:
                return S_MWPOP
            if self.rxd_sync2 == 1 and ((mwcr & 1) or tx_empty) and sclk_re_int:
                return S_CLEAR_READY
            return S_WAIT_READY
        elif c == S_CLEAR_READY:
            if ((sclk_fe_int and baud2) or ((not baud2) and sclk_re_int)) and \
                    (not tx_empty) and (mwcr & 1):
                return S_MWPOP
            if sclk_re_int and not ((mwcr & 1) and (not tx_empty)):
                return S_IDLE
            return S_CLEAR_READY
        elif c == S_SLEEP:
            # RTL default case: n_state = ssi_en ? P_IDLE : P_SLEEP
            return S_IDLE if self.ssienr else S_SLEEP
        else:
            return S_IDLE

    # -- helpers -------------------------------------------------------------
    def _zero_pad(self, reg, dfs):
        """RX buffer zero-padding: keep low dfs+1 bits, zero-extend."""
        if dfs >= 31:
            return reg & 0xFFFFFFFF
        mask = (1 << (dfs + 1)) - 1
        return reg & mask

    def _state_id(self, c):
        return c

    def _read_mux(self):
        a = self.addr
        if a == A_CTRLR0:
            return ((self.ctrlr0 >> 8) << 8) | ((self.ctrlr0 >> 6 & 3) << 6) | \
                   (self.ctrlr0 & 0x1F)
        if a == A_CTRLR1:
            return self.ctrlr1 & 0xFFFF
        if a == A_SSIENR:
            return self.ssienr
        if a == A_MWCR:
            return self.mwcr
        if a == A_SER:
            return self.ser
        if a == A_BAUDR:
            return self.baudr
        if a == A_TXFTLR:
            return self.txftlr
        if a == A_RXFTLR:
            return self.rxftlr
        if a == A_TXFLR:
            return self.txflr
        if a == A_RXFLR:
            return self.rxflr
        if a == A_SR:
            return (self.dcol << 6) | (self.rx_full << 4) | \
                   ((1 - self.rx_empty) << 3) | (self.tx_empty << 2) | \
                   ((1 - self.tx_full) << 1) | self.fsm_busy
        if a == A_IMR:
            return self.imr
        if a == A_ISR:
            return self.isr
        if a == A_RISR:
            return self.risr
        if a == A_TXOICR:
            return (self.isr >> 1) & 1
        if a == A_RXOICR:
            return (self.isr >> 3) & 1
        if a == A_RXUICR:
            return (self.isr >> 2) & 1
        if a == A_MSTICR:
            return (self.isr >> 5) & 1
        if a == A_ICR:
            return 1 if ((self.isr & 0x38) or (self.isr & 0x4) or
                         (self.isr & 0x2)) else 0
        if a == A_IDR:
            return IDR_VAL
        if a == A_VIDR:
            return VIDR_VAL
        if self._is_dr(a):
            return self.rx_pop_data
        return 0

    # -- convenience wrappers for self-tests ---------------------------------
    def write_reg(self, addr, val):
        self.step({"reg_we": 1, "reg_addr": addr, "reg_wdata": val})

    def read_reg(self, addr):
        self.step({"reg_re": 1, "reg_addr": addr})
        return self._read_mux()


def _smoke():
    dut = SpiMasterPublic()
    # SPI0, dfs=8 (field 7), tmod=0, loopback (srl_test=1 -> bit 13), single frame
    dut.write_reg(A_CTRLR0, 0x0000_2007)
    dut.write_reg(A_SER, 0x1)
    dut.write_reg(A_BAUDR, 8)
    dut.write_reg(A_TXFTLR, 0)
    # tx_push is gated by ssienr: enable before pushing data
    dut.write_reg(A_SSIENR, 1)
    dut.write_reg(A_DR, 0x0000_00FF)
    seen = set()
    for _ in range(500):
        dut.step({})
        sig = dut.read_signals()
        seen.add(sig["cov_fsm_state_id"])
    print(f"FSM states seen: {sorted(seen)}")
    print(f"tx_level={dut.tx_level} rx_level={dut.rx_level} "
          f"rx_data={dut.rx_pop_data:08X}")
    return seen


if __name__ == "__main__":
    _smoke()
