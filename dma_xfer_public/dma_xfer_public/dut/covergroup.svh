// dma_xfer_public - covergroup.svh
// SystemVerilog functional coverage definition (evaluation, VCS).
// Mirrors coverage_meta.json (development CoverageSimulator) 1:1.
// Topics: basic / boundary / condition / temporal (sequential window) / cross
//         / temporal_sequences (SVA cover-property equivalents, 12 bins).
// License: MIT

module dma_xfer_covergroup #(
    parameter ADDR_W = 32,
    parameter LEN_W  = 16
) (
    input logic             clk,
    input logic             rst_n,

    // coverage observation bus
    input logic [1:0] ch0_state,
    input logic [1:0] ch1_state,
    input logic [1:0] ch2_state,
    input logic [1:0] ch3_state,
    input logic [1:0] arb_winner,
    input logic       arb_valid,
    input logic [1:0] active_ch,
    input logic [3:0] dir_dir_signal,   // bit3 = direction (mem/io)
    input logic [2:0] cfg_burst,
    input logic [ADDR_W-1:0] cfg_saddr,
    input logic [ADDR_W-1:0] cfg_daddr,
    input logic [LEN_W-1:0]  cfg_len,
    input logic       arb_conflict,
    input logic       hold,
    input logic       done_ok_valid,
    input logic [3:0] done_ok,

    // temporal sequence cover hits (from assert_cov / dma_top cov_seq*)
    input logic cov_seq1, cov_seq2, cov_seq3, cov_seq4, cov_seq5, cov_seq6,
                cov_seq7, cov_seq8, cov_seq9, cov_seq10, cov_seq11, cov_seq12
);

    logic dir_dir;
    assign dir_dir = dir_dir_signal[3];

    // =========================================================
    // basic_functional
    // =========================================================
    covergroup basic_functional @(posedge clk);
        ch0_state_cp: coverpoint ch0_state {
            bins idle0   = {2'd0};
            bins active0 = {2'd1};
            bins done0   = {2'd2};
        }
        ch1_state_cp: coverpoint ch1_state {
            bins idle1   = {2'd0};
            bins active1 = {2'd1};
            bins done1   = {2'd2};
        }
        ch2_state_cp: coverpoint ch2_state {
            bins idle2   = {2'd0};
            bins active2 = {2'd1};
            bins done2   = {2'd2};
        }
        ch3_state_cp: coverpoint ch3_state {
            bins idle3   = {2'd0};
            bins active3 = {2'd1};
            bins done3   = {2'd2};
        }
        arb_winner_cp: coverpoint arb_winner iff (arb_valid) {
            bins win0 = {2'd0};
            bins win1 = {2'd1};
            bins win2 = {2'd2};
            bins win3 = {2'd3};
        }
        dir_dir_cp: coverpoint dir_dir {
            bins mem_mem = {1'b0};
            bins mem_io  = {1'b1};
        }
        cfg_burst_cp: coverpoint cfg_burst {
            bins burst0 = {3'd0};
            bins burst1 = {3'd1};
            bins burst2 = {3'd2};
            bins burst3 = {3'd3};
            bins burst4 = {3'd4};
            bins burst5 = {3'd5};
            bins burst6 = {3'd6};
            bins burst7 = {3'd7};
        }
        active_ch_cp: coverpoint active_ch {
            bins ch0_act = {2'd0};
            bins ch1_act = {2'd1};
            bins ch2_act = {2'd2};
            bins ch3_act = {2'd3};
        }
    endgroup

    // =========================================================
    // condition_combinational
    // =========================================================
    covergroup condition_combinational @(posedge clk);
        arb_conflict_cp: coverpoint arb_conflict {
            bins no_conflict = {1'b0};
            bins conflict    = {1'b1};
        }
        hold_cp: coverpoint hold {
            bins hold_off = {1'b0};
            bins hold_on  = {1'b1};
        }
    endgroup

    // =========================================================
    // data_boundary
    // =========================================================
    covergroup data_boundary @(posedge clk);
        saddr_cp: coverpoint cfg_saddr {
            bins saddr_zero     = {32'd0};
            bins saddr_one      = {32'd1};
            bins saddr_upper_m1 = {32'hFFFFFFFE};
            bins saddr_allones  = {32'hFFFFFFFF};
        }
        daddr_cp: coverpoint cfg_daddr {
            bins daddr_zero     = {32'd0};
            bins daddr_one      = {32'd1};
            bins daddr_upper_m1 = {32'hFFFFFFFE};
            bins daddr_allones  = {32'hFFFFFFFF};
        }
        len_cp: coverpoint cfg_len {
            bins len_zero     = {16'd0};
            bins len_one      = {16'd1};
            bins len_upper_m1 = {16'hFFFE};
            bins len_allones  = {16'hFFFF};
        }
    endgroup

    // =========================================================
    // temporal_window (correct-mode done-hold)
    // =========================================================
    covergroup temporal_window @(posedge clk);
        done_ok_cp: coverpoint done_ok iff (done_ok_valid) {
            bins done_ok_0 = {4'd0};
            bins done_ok_1 = {4'd1};
            bins done_ok_2 = {4'd2};
            bins done_ok_3 = {4'd3};
        }
    endgroup

    // =========================================================
    // cross_functional
    // =========================================================
    covergroup cross_functional;
        // arb_winner x hold
        arb_hold_cp: cross bf_inst.arb_winner_cp, cc_inst.hold_cp {
            bins win0_hold   = binsof(bf_inst.arb_winner_cp.win0) && binsof(cc_inst.hold_cp.hold_on);
            bins win0_nohold = binsof(bf_inst.arb_winner_cp.win0) && binsof(cc_inst.hold_cp.hold_off);
            bins win1_hold   = binsof(bf_inst.arb_winner_cp.win1) && binsof(cc_inst.hold_cp.hold_on);
            bins win1_nohold = binsof(bf_inst.arb_winner_cp.win1) && binsof(cc_inst.hold_cp.hold_off);
            bins win2_hold   = binsof(bf_inst.arb_winner_cp.win2) && binsof(cc_inst.hold_cp.hold_on);
            bins win2_nohold = binsof(bf_inst.arb_winner_cp.win2) && binsof(cc_inst.hold_cp.hold_off);
            bins win3_hold   = binsof(bf_inst.arb_winner_cp.win3) && binsof(cc_inst.hold_cp.hold_on);
            bins win3_nohold = binsof(bf_inst.arb_winner_cp.win3) && binsof(cc_inst.hold_cp.hold_off);
        }

        // done_ok x active_ch
        done_ch_cp: cross tw_inst.done_ok_cp, bf_inst.active_ch_cp {
            bins done_ch0 = binsof(tw_inst.done_ok_cp.done_ok_0) && binsof(bf_inst.active_ch_cp.ch0_act);
            bins done_ch1 = binsof(tw_inst.done_ok_cp.done_ok_1) && binsof(bf_inst.active_ch_cp.ch1_act);
            bins done_ch2 = binsof(tw_inst.done_ok_cp.done_ok_2) && binsof(bf_inst.active_ch_cp.ch2_act);
            bins done_ch3 = binsof(tw_inst.done_ok_cp.done_ok_3) && binsof(bf_inst.active_ch_cp.ch3_act);
        }

        // len x ch0_state (reachable combos)
        len_state_cp: cross db_inst.len_cp, bf_inst.ch0_state_cp {
            bins len1_done    = binsof(db_inst.len_cp.len_one) && binsof(bf_inst.ch0_state_cp.done0);
            bins len_hi_idle  = binsof(db_inst.len_cp.len_allones) && binsof(bf_inst.ch0_state_cp.idle0);
        }

        // saddr x direction
        addr_dir_cp: cross db_inst.saddr_cp, bf_inst.dir_dir_cp {
            bins saddr0_mem    = binsof(db_inst.saddr_cp.saddr_zero) && binsof(bf_inst.dir_dir_cp.mem_mem);
            bins saddr0_io     = binsof(db_inst.saddr_cp.saddr_zero) && binsof(bf_inst.dir_dir_cp.mem_io);
            bins saddr1_mem    = binsof(db_inst.saddr_cp.saddr_one) && binsof(bf_inst.dir_dir_cp.mem_mem);
            bins saddr1_io     = binsof(db_inst.saddr_cp.saddr_one) && binsof(bf_inst.dir_dir_cp.mem_io);
            bins saddr_hi_mem  = binsof(db_inst.saddr_cp.saddr_allones) && binsof(bf_inst.dir_dir_cp.mem_mem);
            bins saddr_hi_io   = binsof(db_inst.saddr_cp.saddr_allones) && binsof(bf_inst.dir_dir_cp.mem_io);
        }

        // burst x hold
        burst_hold_cp: cross bf_inst.cfg_burst_cp, cc_inst.hold_cp {
            bins burst0_nohold = binsof(bf_inst.cfg_burst_cp.burst0) && binsof(cc_inst.hold_cp.hold_off);
            bins burst0_hold   = binsof(bf_inst.cfg_burst_cp.burst0) && binsof(cc_inst.hold_cp.hold_on);
            bins burst7_nohold = binsof(bf_inst.cfg_burst_cp.burst7) && binsof(cc_inst.hold_cp.hold_off);
            bins burst7_hold   = binsof(bf_inst.cfg_burst_cp.burst7) && binsof(cc_inst.hold_cp.hold_on);
            bins burst1_nohold = binsof(bf_inst.cfg_burst_cp.burst1) && binsof(cc_inst.hold_cp.hold_off);
            bins burst1_hold   = binsof(bf_inst.cfg_burst_cp.burst1) && binsof(cc_inst.hold_cp.hold_on);
        }

        // direction x done class (classes 0/1 mem, 2/3 io)
        dir_done_cp: cross bf_inst.dir_dir_cp, tw_inst.done_ok_cp {
            bins mem_done0 = binsof(bf_inst.dir_dir_cp.mem_mem) && binsof(tw_inst.done_ok_cp.done_ok_0);
            bins mem_done1 = binsof(bf_inst.dir_dir_cp.mem_mem) && binsof(tw_inst.done_ok_cp.done_ok_1);
            bins io_done2  = binsof(bf_inst.dir_dir_cp.mem_io) && binsof(tw_inst.done_ok_cp.done_ok_2);
            bins io_done3  = binsof(bf_inst.dir_dir_cp.mem_io) && binsof(tw_inst.done_ok_cp.done_ok_3);
        }
    endgroup

    // =========================================================
    // temporal_sequences (SVA cover-property equivalents)
    // =========================================================
    covergroup temporal_sequences @(posedge clk);
        seq_full_handshake: coverpoint cov_seq1   { bins hit = {1}; }
        seq_multi_beat:     coverpoint cov_seq2   { bins hit = {1}; }
        seq_ok_uninterrupt: coverpoint cov_seq3   { bins hit = {1}; }
        seq_hold_persist:   coverpoint cov_seq4   { bins hit = {1}; }
        seq_hold_then_ok:   coverpoint cov_seq5   { bins hit = {1}; }
        seq_arb_resolved:   coverpoint cov_seq6   { bins hit = {1}; }
        seq_arb_cont:       coverpoint cov_seq7   { bins hit = {1}; }
        seq_done_observed:  coverpoint cov_seq8   { bins hit = {1}; }
        seq_idle_stable:    coverpoint cov_seq9   { bins hit = {1}; }
        seq_quick_done:     coverpoint cov_seq10  { bins hit = {1}; }
        seq_chan_switch:    coverpoint cov_seq11  { bins hit = {1}; }
        seq_done_consistent:coverpoint cov_seq12  { bins hit = {1}; }
    endgroup

    basic_functional bf_inst = new;
    condition_combinational cc_inst = new;
    data_boundary db_inst = new;
    temporal_window tw_inst = new;
    cross_functional cf_inst = new;
    temporal_sequences ts_inst = new;

endmodule
