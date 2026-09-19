// spi_xfer_public - SystemVerilog covergroup definition (VCS / Synopsys)
// Mirrors dut/coverage_meta.json bin structure. Instantiated on the coverage
// observation bus exported by spi_xfer_top.
module spi_xfer_covergroup (
    input logic clk,
    input logic rst_n,

    // Coverage observation bus (from spi_xfer_top)
    input logic [10:0] cov_fsm_state,
    input logic [3:0]  cov_fsm_state_id,
    input logic        cov_sclk_re,
    input logic        cov_sclk_fe,
    input logic        cov_sclk_out,
    input logic        cov_sclk_mask,
    input logic [5:0]  cov_bit_cnt,
    input logic [16:0] cov_frame_cnt,
    input logic        cov_x_done,
    input logic        cov_last_frame,
    input logic        cov_tx_shift_en,
    input logic        cov_rx_shift_en,
    input logic        cov_load_tx_shift,
    input logic        cov_load_rx_buf,
    input logic        cov_ss_active,
    input logic        cov_xfer_complete,
    input logic        cov_txd,
    input logic [4:0]  cov_dfs,
    input logic [5:0]  cov_eff_dfs,
    input logic [15:0] cov_baudr,
    input logic        cov_baud2,
    input logic [16:0] cov_ndf,
    input logic [1:0]  cov_tmod,
    input logic [1:0]  cov_frf,
    input logic        cov_scph,
    input logic        cov_scpol,
    input logic        cov_srl_test,
    input logic        cov_ss_tgl_en,
    input logic [3:0]  cov_ser,
    input logic [3:0]  cov_tx_level,
    input logic [3:0]  cov_rx_level,
    input logic        cov_tx_empty,
    input logic        cov_tx_full,
    input logic        cov_rx_empty,
    input logic        cov_rx_full,
    input logic [3:0]  cov_hold_ss_cnt,
    input logic [31:0] cov_rx_data,
    input logic        cov_ss_in_n,
    input logic        cov_start_xfer,
    input logic        cov_s0,
    input logic        cov_s1,
    input logic        cov_s2
);

    // Derived protocol class: 0=SPI0 (frf=0,scph=0), 1=SPI1 (frf=0,scph=1),
    //                          2=SSP (frf=1)
    logic [1:0] protocol;
    always_comb begin
        if (cov_frf == 2'd0) protocol = {1'b0, cov_scph};
        else                 protocol = 2'd2;
    end

    // ============================================================
    // Covergroup: fsm_protocol
    // ============================================================
    covergroup fsm_protocol @(posedge clk);
        cp_fsm_state: coverpoint cov_fsm_state_id {
            bins idle      = {4'd0};
            bins assert_ss = {4'd1};
            bins pop_tx    = {4'd2};
            bins shift_bit = {4'd3};
            bins push_rx   = {4'd4};
            bins toggle_ss = {4'd5};
            bins hold_mask = {4'd6};
            bins mask_sclk = {4'd7};
            bins hold_ss   = {4'd8};
            bins clear_ss  = {4'd9};
            bins sleep     = {4'd10};
        }
        cp_protocol: coverpoint protocol iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) {
            bins spi0 = {2'd0};
            bins spi1 = {2'd1};
            bins ssp  = {2'd2};
        }
        cp_tmod: coverpoint cov_tmod iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) {
            bins tmod_0 = {2'd0};
            bins tmod_1 = {2'd1};
            bins tmod_2 = {2'd2};
            bins tmod_3 = {2'd3};
        }
        cp_baud2: coverpoint cov_baud2 iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) {
            bins normal_div = {1'b0};
            bins baud2      = {1'b1};
        }
        cp_last_frame: coverpoint cov_last_frame iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) {
            bins more_frames = {1'b0};
            bins last_frame  = {1'b1};
        }
        cp_ss_toggle: coverpoint cov_ss_tgl_en iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) {
            bins ss_hold_cont = {1'b0};
            bins ss_toggle     = {1'b1};
        }
        cp_hold_ss: coverpoint cov_hold_ss_cnt iff (cov_fsm_state_id == 4'd8) {
            bins hold_0 = {4'd0};
            bins hold_1 = {4'd1};
            bins hold_2 = {4'd2};
            bins hold_3 = {4'd3};
        }
        cp_dfs_effective_min: coverpoint (cov_dfs < 5'd3 && cov_eff_dfs == 6'd4) iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) {
            bins min_hit = {1'b1};
        }
        cp_dfs_effective_min_frame: coverpoint (cov_eff_dfs == 6'd4 && cov_dfs < 5'd3 && cov_load_rx_buf) {
            bins min_frame_hit = {1'b1};
        }
    endgroup

    // ============================================================
    // Covergroup: data_boundary
    // ============================================================
    covergroup data_boundary @(posedge clk);
        cp_dfs: coverpoint cov_eff_dfs iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) {
            bins dfs_min  = {6'd4};
            bins dfs_8    = {6'd8};
            bins dfs_16   = {6'd16};
            bins dfs_24   = {6'd24};
            bins dfs_31   = {6'd31};
            bins dfs_max  = {6'd32};
        }
        cp_baudr: coverpoint cov_baudr iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) {
            bins baudr_1 = {16'd1};
            bins baudr_2 = {16'd2};
            bins baudr_3 = {16'd3};
            bins baudr_8 = {16'd8};
        }
        cp_frame_cnt: coverpoint cov_frame_cnt iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) {
            bins frame_0 = {17'd0};
            bins frame_1 = {17'd1};
            bins frame_2 = {17'd2};
            bins frame_3 = {17'd3};
            bins frame_7 = {17'd7};
        }
        cp_hold_last_frame: coverpoint (cov_fsm_state_id == 4'd8 && cov_hold_ss_cnt == 4'd3 &&
                                      cov_last_frame && cov_frame_cnt >= 17'd1) {
            bins last_frame_hit = {1'b1};
        }
        cp_rx_data: coverpoint cov_rx_data iff (cov_rx_level != 4'd0) {
            bins rx_all_zero   = {32'h00000000};
            bins rx_0x55       = {32'h00000055};
            bins rx_0xAA       = {32'h000000AA};
            bins rx_all_ones8  = {32'h000000FF};
            bins rx_all_ones32 = {32'hFFFFFFFF};
        }
        cp_tx_level: coverpoint cov_tx_level iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) {
            bins tx_empty = {4'd0};
            bins tx_half  = {4'd4};
            bins tx_full  = {4'd8};
        }
        cp_rx_level: coverpoint cov_rx_level iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) {
            bins rx_empty = {4'd0};
            bins rx_half  = {4'd4};
            bins rx_full  = {4'd8};
        }
    endgroup

    // ============================================================
    // Covergroup: seq_a
    // ============================================================
    covergroup seq_a @(posedge clk);
        cp_sa_0: coverpoint (cov_s0 &&
                             (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10)) {
            bins seq_a_0 = {1'b1};
        }
        cp_sa_1: coverpoint (cov_s0 &&
                             (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) &&
                             cov_frame_cnt >= 17'd1) {
            bins seq_a_1 = {1'b1};
        }
        cp_sa_2: coverpoint (cov_s0 && cov_fsm_state_id == 4'd8 &&
                             cov_hold_ss_cnt == 4'd3 && cov_last_frame &&
                             cov_frame_cnt >= 17'd1) {
            bins seq_a_2 = {1'b1};
        }
        cp_sa_3: coverpoint {protocol, cov_s0} {
            bins seq_a_3 = {3'b001};
            bins seq_a_4 = {3'b011};
            bins seq_a_5 = {3'b101};
        }
    endgroup

    // ============================================================
    // Covergroup: seq_b
    // ============================================================
    covergroup seq_b @(posedge clk);
        cp_sb_0: coverpoint (cov_s1 &&
                             (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) &&
                             (!cov_tx_empty) && cov_frame_cnt >= 17'd1) {
            bins seq_b_0 = {1'b1};
        }
    endgroup

    // ============================================================
    // Covergroup: seq_c
    // ============================================================
    covergroup seq_c @(posedge clk);
        cp_sc_0: coverpoint (cov_s2 &&
                             (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10)) {
            bins seq_c_0 = {1'b1};
        }
        cp_sc_1: coverpoint (cov_s2 &&
                             (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) &&
                             cov_frame_cnt >= 17'd1) {
            bins seq_c_1 = {1'b1};
        }
    endgroup

    // fsm_protocol / data_boundary instances must be declared
    // before cross_functional, which crosses their coverpoints by instance name.
    // NOTE: VCS T-2022.06 does not collect data from bare declarations
    // ("cg cg_inst;"); explicit construction ("= new;") is required.
    fsm_protocol     fsm_protocol_inst = new;
    data_boundary    data_boundary_inst = new;
    seq_a            seq_a_inst = new;
    seq_b            seq_b_inst = new;
    seq_c            seq_c_inst = new;

    // ============================================================
    // Covergroup: cross_functional
    // ============================================================
    // Crosses reference coverpoints of other covergroups (fsm_protocol /
    // data_boundary), so per IEEE 1800 §19.6 this covergroup must NOT carry
    // its own sampling event; sampling is driven by the referenced groups.
    covergroup cross_functional;
        protocol_x_tmod: cross fsm_protocol_inst.cp_protocol, fsm_protocol_inst.cp_tmod
            iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) {
            bins spi0_tmod2 = binsof(fsm_protocol_inst.cp_protocol.spi0) &&
                              binsof(fsm_protocol_inst.cp_tmod.tmod_2);
            bins spi0_tmod3 = binsof(fsm_protocol_inst.cp_protocol.spi0) &&
                              binsof(fsm_protocol_inst.cp_tmod.tmod_3);
            bins spi1_tmod2 = binsof(fsm_protocol_inst.cp_protocol.spi1) &&
                              binsof(fsm_protocol_inst.cp_tmod.tmod_2);
            bins spi1_tmod3 = binsof(fsm_protocol_inst.cp_protocol.spi1) &&
                              binsof(fsm_protocol_inst.cp_tmod.tmod_3);
            bins ssp_tmod2  = binsof(fsm_protocol_inst.cp_protocol.ssp) &&
                              binsof(fsm_protocol_inst.cp_tmod.tmod_2);
            bins ssp_tmod3  = binsof(fsm_protocol_inst.cp_protocol.ssp) &&
                              binsof(fsm_protocol_inst.cp_tmod.tmod_3);
        }
        protocol_x_dfs_boundary: cross fsm_protocol_inst.cp_protocol, data_boundary_inst.cp_dfs
            iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) {
            bins spi0_dfs8  = binsof(fsm_protocol_inst.cp_protocol.spi0) &&
                              binsof(data_boundary_inst.cp_dfs.dfs_8);
            bins spi0_dfs32 = binsof(fsm_protocol_inst.cp_protocol.spi0) &&
                              binsof(data_boundary_inst.cp_dfs.dfs_max);
            bins spi1_dfs8  = binsof(fsm_protocol_inst.cp_protocol.spi1) &&
                              binsof(data_boundary_inst.cp_dfs.dfs_8);
            bins spi1_dfs32 = binsof(fsm_protocol_inst.cp_protocol.spi1) &&
                              binsof(data_boundary_inst.cp_dfs.dfs_max);
            bins ssp_dfs8   = binsof(fsm_protocol_inst.cp_protocol.ssp) &&
                              binsof(data_boundary_inst.cp_dfs.dfs_8);
            bins ssp_dfs16  = binsof(fsm_protocol_inst.cp_protocol.ssp) &&
                              binsof(data_boundary_inst.cp_dfs.dfs_16);
        }
        protocol_x_baud2_path: cross fsm_protocol_inst.cp_protocol, fsm_protocol_inst.cp_baud2
            iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) {
            bins spi0_baud2 = binsof(fsm_protocol_inst.cp_protocol.spi0) &&
                              binsof(fsm_protocol_inst.cp_baud2.baud2);
            bins spi1_baud2 = binsof(fsm_protocol_inst.cp_protocol.spi1) &&
                              binsof(fsm_protocol_inst.cp_baud2.baud2);
            bins ssp_baud2  = binsof(fsm_protocol_inst.cp_protocol.ssp) &&
                              binsof(fsm_protocol_inst.cp_baud2.baud2);
        }
        protocol_x_hold_ss_cnt_boundary: cross fsm_protocol_inst.cp_protocol, fsm_protocol_inst.cp_hold_ss
            iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) {
            bins spi1_hold3 = binsof(fsm_protocol_inst.cp_protocol.spi1) &&
                              binsof(fsm_protocol_inst.cp_hold_ss.hold_3);
            bins ssp_hold3  = binsof(fsm_protocol_inst.cp_protocol.ssp) &&
                              binsof(fsm_protocol_inst.cp_hold_ss.hold_3);
        }
        protocol_x_last_frame: cross fsm_protocol_inst.cp_protocol, fsm_protocol_inst.cp_last_frame
            iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd10) {
            bins spi0_last = binsof(fsm_protocol_inst.cp_protocol.spi0) &&
                             binsof(fsm_protocol_inst.cp_last_frame.last_frame);
            bins spi1_last = binsof(fsm_protocol_inst.cp_protocol.spi1) &&
                             binsof(fsm_protocol_inst.cp_last_frame.last_frame);
            bins ssp_last  = binsof(fsm_protocol_inst.cp_protocol.ssp) &&
                             binsof(fsm_protocol_inst.cp_last_frame.last_frame);
        }
    endgroup

    // Covergroup instantiation (IEEE 1800: no parens). The cross_functional
    // instance follows its definition. Explicit "= new;" required for VCS
    // T-2022.06 data collection (see note above).
    cross_functional cross_functional_inst = new;

endmodule
