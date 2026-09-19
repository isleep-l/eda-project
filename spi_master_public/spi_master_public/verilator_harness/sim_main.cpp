// sim_main.cpp - Verilator testbench for spi_m_top
//
// Text interface (for run_verilator.py / cross_check.py):
//   argv[1] = SPEC_CFS_MIN      (default 8)
//   argv[2] = SPEC_HOLD_SS      (default 4)
//   argv[3] = SPEC_TXFTLR_DFLT  (default 1)
//
//   stdin : one action per line:  reg_we reg_addr reg_wdata reg_re rxd ss_in_n rst_n
//   stdout: header line (signal names, comma separated) followed by one
//           comma-separated line of signal values per simulated clock cycle.
//
// The first line of stdout is a header; subsequent lines are per-cycle values.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include "Vspi_m_top.h"
#include "verilated.h"

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    int cfs_min = (argc > 1) ? atoi(argv[1]) : 8;
    int hold_ss = (argc > 2) ? atoi(argv[2]) : 4;
    int txftlr_dflt = (argc > 3) ? atoi(argv[3]) : 1;

    Vspi_m_top* dut = new Vspi_m_top;

    // Hidden configuration (evaluation-injected constants)
    dut->SPEC_CFS_MIN     = cfs_min & 0xF;
    dut->SPEC_HOLD_SS     = hold_ss & 0xF;
    dut->SPEC_TXFTLR_DFLT = txftlr_dflt & 0xF;

    // Initialize all inputs
    dut->clk = 0;
    dut->rst_n = 1;
    dut->reg_we = 0;
    dut->reg_addr = 0;
    dut->reg_wdata = 0;
    dut->reg_re = 0;
    dut->rxd = 0;
    dut->ss_in_n = 1;

    printf("cov_fsm_state_id,cov_fsm_state,cov_sclk_re,cov_sclk_fe,"
           "cov_sclk_out,cov_sclk_mask,cov_bit_cnt,cov_ctrl_cnt,"
           "cov_frame_cnt,cov_f_done,cov_c_done,cov_x_done,cov_last_frame,"
           "cov_tx_shift_en,cov_rx_shift_en,cov_load_tx_shift,"
           "cov_load_tx_buffer,cov_load_rx_buf,cov_s3,cov_txd,"
           "cov_ss_in_n,cov_ss_active,cov_dfs,cov_eff_dfs,cov_cfs,cov_eff_cfs,"
           "cov_baudr,cov_baud2,cov_ndf,cov_tmod,cov_frf,cov_scph,cov_scpol,"
           "cov_srl_test,cov_ss_tgl_en,cov_slv_oe,cov_ser,cov_tx_level,"
           "cov_rx_level,cov_tx_full,cov_tx_empty,cov_rx_full,cov_rx_empty,"
           "cov_tx_overflow,cov_rx_overflow,cov_rx_underflow,cov_risr,"
           "cov_isr,cov_imr,cov_ssi_txe_intr,cov_ssi_mst_intr,"
           "cov_s1,cov_multi_mst_sync,cov_s2,cov_hold_ss_cnt,"
           "cov_start_xfer,cov_fsm_busy,cov_fsm_sleep,cov_ssi_busy,"
           "cov_ssi_sleep,cov_rx_push,cov_tx_pop,cov_rx_data,cov_mwcr,"
           "cov_spi0_control,cov_spi1_control,cov_txftlr,reg_rdata\n");
    fflush(stdout);  // flush header now: stdout is fully buffered on a pipe,
                     // otherwise the Python harness blocks forever on readline()

    long long we, addr, wdata, re, rxd, ss_in, rst;
    while (true) {
        if (scanf("%lld %lld %lld %lld %lld %lld %lld",
                  &we, &addr, &wdata, &re, &rxd, &ss_in, &rst) != 7) {
            break;
        }
        // Drive one full clock cycle per action: inputs settle at clk=0,
        // the posedge clocks them into the registers, then return to clk=0.
        dut->clk       = 0;
        dut->rst_n     = (rst & 1);
        dut->reg_we    = (we & 1);
        dut->reg_addr  = (addr & 0x3F);
        dut->reg_wdata = (uint32_t)wdata;
        dut->reg_re    = (re & 1);
        dut->rxd       = (rxd & 1);
        dut->ss_in_n   = (ss_in & 1);
        dut->eval();
        dut->clk = 1;
        dut->eval();
        dut->clk = 0;
        dut->eval();

        printf("%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,"
               "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,"
               "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,"
               "%u,%u,%u,%u,%u,%u,%u,%u\n",
            (uint32_t)dut->cov_fsm_state_id, (uint32_t)dut->cov_fsm_state,
            (uint32_t)dut->cov_sclk_re, (uint32_t)dut->cov_sclk_fe,
            (uint32_t)dut->cov_sclk_out, (uint32_t)dut->cov_sclk_mask,
            (uint32_t)dut->cov_bit_cnt, (uint32_t)dut->cov_ctrl_cnt,
            (uint32_t)dut->cov_frame_cnt, (uint32_t)dut->cov_f_done,
            (uint32_t)dut->cov_c_done, (uint32_t)dut->cov_x_done,
            (uint32_t)dut->cov_last_frame,
            (uint32_t)dut->cov_tx_shift_en, (uint32_t)dut->cov_rx_shift_en,
            (uint32_t)dut->cov_load_tx_shift,
            (uint32_t)dut->cov_load_tx_buffer, (uint32_t)dut->cov_load_rx_buf,
            (uint32_t)dut->cov_s3, (uint32_t)dut->cov_txd,
            (uint32_t)dut->cov_ss_in_n, (uint32_t)dut->cov_ss_active,
            (uint32_t)dut->cov_dfs, (uint32_t)dut->cov_eff_dfs,
            (uint32_t)dut->cov_cfs, (uint32_t)dut->cov_eff_cfs,
            (uint32_t)dut->cov_baudr, (uint32_t)dut->cov_baud2,
            (uint32_t)dut->cov_ndf, (uint32_t)dut->cov_tmod,
            (uint32_t)dut->cov_frf, (uint32_t)dut->cov_scph,
            (uint32_t)dut->cov_scpol, (uint32_t)dut->cov_srl_test,
            (uint32_t)dut->cov_ss_tgl_en, (uint32_t)dut->cov_slv_oe,
            (uint32_t)dut->cov_ser, (uint32_t)dut->cov_tx_level,
            (uint32_t)dut->cov_rx_level, (uint32_t)dut->cov_tx_full,
            (uint32_t)dut->cov_tx_empty, (uint32_t)dut->cov_rx_full,
            (uint32_t)dut->cov_rx_empty, (uint32_t)dut->cov_tx_overflow,
            (uint32_t)dut->cov_rx_overflow, (uint32_t)dut->cov_rx_underflow,
            (uint32_t)dut->cov_risr, (uint32_t)dut->cov_isr,
            (uint32_t)dut->cov_imr, (uint32_t)dut->cov_ssi_txe_intr,
            (uint32_t)dut->cov_ssi_mst_intr, (uint32_t)dut->cov_s1,
            (uint32_t)dut->cov_multi_mst_sync, (uint32_t)dut->cov_s2,
            (uint32_t)dut->cov_hold_ss_cnt, (uint32_t)dut->cov_start_xfer,
            (uint32_t)dut->cov_fsm_busy, (uint32_t)dut->cov_fsm_sleep,
            (uint32_t)dut->cov_ssi_busy, (uint32_t)dut->cov_ssi_sleep,
            (uint32_t)dut->cov_rx_push, (uint32_t)dut->cov_tx_pop,
            (uint32_t)dut->cov_rx_data, (uint32_t)dut->cov_mwcr,
            (uint32_t)dut->cov_spi0_control, (uint32_t)dut->cov_spi1_control,
            (uint32_t)dut->cov_txftlr, (uint32_t)dut->reg_rdata);
        fflush(stdout);
    }

    dut->final();
    delete dut;
    return 0;
}
