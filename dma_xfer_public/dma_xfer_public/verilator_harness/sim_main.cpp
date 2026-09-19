// sim_main.cpp - Verilator testbench for dma_top
//
// Text interface (for run_verilator.py):
//   stdin : one action per line:
//           ch_sel conf_wr conf_field d0 d1 d2 d3 start rst_n
//     - ch_sel    (0..3)     : channel being configured / started
//     - conf_wr   (bool)     : perform a register write this cycle
//     - conf_field(0..2)     : 0=saddr 1=daddr 2={burst,dmode,len}
//     - d0..d3    (bytes)    : 32-bit register data (little endian)
//     - start     (bool)     : if 1, raise the `start` bit of ch_sel (the RTL
//                              performs per-channel rising-edge detection)
//     - rst_n     (bool, 1=run)
//   stdout: header line (signal names, comma separated) followed by one
//           comma-separated line of signal values per simulated clock cycle.
//
// The hidden parameters (ARB_PRIO / DONE_OK_MODES / XFER_HOLD_N) are baked
// into the model at build time by build.sh (-G...). Verilator >=5.020 does not
// expose top-level module parameters as runtime-assignable --public members,
// so they are NOT set here.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include "Vdma_top.h"
#include "verilated.h"

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    Vdma_top* dut = new Vdma_top;

    // Initialize all inputs
    dut->clk = 0;
    dut->rst_n = 1;
    dut->conf_ch = 0;
    dut->conf_wr = 0;
    dut->conf_field = 0;
    dut->conf_data = 0;
    dut->start = 0;

    printf("cov_ch0_state,cov_ch1_state,cov_ch2_state,cov_ch3_state,"
           "cov_arb_winner,cov_arb_valid,cov_arb_conflict,cov_active_ch,"
           "cov_dir,cov_burst,cov_saddr,cov_daddr,cov_len,cov_hold,"
           "cov_done_ok_valid,cov_done_class,"
           "cov_seq1,cov_seq2,cov_seq3,cov_seq4,cov_seq5,cov_seq6,"
           "cov_seq7,cov_seq8,cov_seq9,cov_seq10,cov_seq11,cov_seq12\n");
    fflush(stdout);  // flush header now: stdout is fully buffered on a pipe,
                     // otherwise the Python harness blocks forever on readline()

    long long ch, wr, field, d0, d1, d2, d3, start, rst;
    while (true) {
        if (scanf("%lld %lld %lld %lld %lld %lld %lld %lld %lld",
                  &ch, &wr, &field, &d0, &d1, &d2, &d3, &start, &rst) != 9) {
            break;
        }
        // Drive one full clock cycle per action. Inputs settle at clk=0,
        // the posedge clocks them into the registers, then return to clk=0.
        uint32_t conf_data = ((uint32_t)(d0 & 0xFF)) |
                             ((uint32_t)(d1 & 0xFF) << 8) |
                             ((uint32_t)(d2 & 0xFF) << 16) |
                             ((uint32_t)(d3 & 0xFF) << 24);
        int chsel = (int)(ch & 3);
        dut->clk        = 0;
        dut->rst_n      = (rst & 1);
        dut->conf_ch    = chsel;
        dut->conf_wr    = (wr & 1);
        dut->conf_field = (field & 3);
        dut->conf_data  = conf_data;
        dut->start      = (start & 1) ? (1 << chsel) : 0;
        dut->eval();
        dut->clk = 1;
        dut->eval();
        dut->clk = 0;
        dut->eval();

        printf("%u,%u,%u,%u,%u,%u,%u,%u,"
               "%u,%u,%u,%u,%u,%u,%u,%u,"
               "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
            (uint32_t)dut->cov_ch0_state, (uint32_t)dut->cov_ch1_state,
            (uint32_t)dut->cov_ch2_state, (uint32_t)dut->cov_ch3_state,
            (uint32_t)dut->cov_arb_winner, (uint32_t)dut->cov_arb_valid,
            (uint32_t)dut->cov_arb_conflict, (uint32_t)dut->cov_active_ch,
            (uint32_t)dut->cov_dir, (uint32_t)dut->cov_burst,
            (uint32_t)dut->cov_saddr, (uint32_t)dut->cov_daddr,
            (uint32_t)dut->cov_len, (uint32_t)dut->cov_hold,
            (uint32_t)dut->cov_done_ok_valid, (uint32_t)dut->cov_done_class,
            (uint32_t)dut->cov_seq1, (uint32_t)dut->cov_seq2,
            (uint32_t)dut->cov_seq3, (uint32_t)dut->cov_seq4,
            (uint32_t)dut->cov_seq5, (uint32_t)dut->cov_seq6,
            (uint32_t)dut->cov_seq7, (uint32_t)dut->cov_seq8,
            (uint32_t)dut->cov_seq9, (uint32_t)dut->cov_seq10,
            (uint32_t)dut->cov_seq11, (uint32_t)dut->cov_seq12);
        fflush(stdout);
    }

    dut->final();
    delete dut;
    return 0;
}
