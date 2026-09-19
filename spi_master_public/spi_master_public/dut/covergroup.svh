// spi_master_public - SystemVerilog covergroup definition (VCS / Synopsys)
// Mirrors dut/coverage_meta.json bin structure. Instantiated on the coverage
// observation bus exported by spi_m_top.
//
// The hidden configuration values are parameterized so the evaluation
// platform can inject the same constants used at DUT compile time.
//
// All configuration/FSM-behaviour coverpoints are gated on a real in-progress
// transfer (cov_fsm_state_id outside {IDLE=0, SLEEP=11}) so that merely
// writing registers without transmitting earns no coverage.
module spi_m_covergroup #(
    parameter [3:0] SPEC_CFS_MIN     = 4'd8,
    parameter [3:0] SPEC_HOLD_SS     = 4'd4,
    parameter [3:0] SPEC_TXFTLR_DFLT = 4'd1
) (
    input logic clk,
    input logic rst_n,

    // Coverage observation bus (from spi_m_top)
    input logic [3:0]  cov_fsm_state_id,
    input logic [14:0] cov_fsm_state,
    input logic        cov_sclk_re,
    input logic        cov_sclk_fe,
    input logic        cov_sclk_out,
    input logic        cov_sclk_mask,
    input logic [5:0]  cov_bit_cnt,
    input logic [3:0]  cov_ctrl_cnt,
    input logic [16:0] cov_frame_cnt,
    input logic        cov_f_done,
    input logic        cov_c_done,
    input logic        cov_x_done,
    input logic        cov_last_frame,
    input logic        cov_tx_shift_en,
    input logic        cov_rx_shift_en,
    input logic        cov_load_tx_shift,
    input logic        cov_s3,
    input logic        cov_txd,
    input logic        cov_ss_active,
    input logic [4:0]  cov_dfs,
    input logic [4:0]  cov_eff_dfs,
    input logic [3:0]  cov_cfs,
    input logic [3:0]  cov_eff_cfs,
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
    input logic [3:0]  cov_hold_ss_cnt,
    input logic [31:0] cov_rx_data,
    input logic        cov_ss_in_n,
    input logic [5:0]  cov_risr,
    input logic        cov_s1,
    input logic        cov_s2,
    input logic        cov_ssi_busy,
    input logic [2:0]  cov_mwcr,
    input logic [3:0]  cov_txftlr,
    input logic        cov_s0
);

    // Derived protocol class: 0=SPI0 (frf=0,scph=0), 1=SPI1 (frf=0,scph=1),
    //                          2=SSP (frf=1),         3=Microwire (frf=2)
    logic [1:0] protocol;
    always_comb begin
        case (cov_frf)
            2'd0: protocol = {1'b0, cov_scph};
            2'd1: protocol = 2'd2;
            default: protocol = 2'd3;
        endcase
    end

    // ============================================================
    // Covergroup: fsm_protocol
    // ============================================================
    covergroup fsm_protocol @(posedge clk);
        cp_fsm_state: coverpoint cov_fsm_state_id {
            bins idle         = {4'd0};
            bins assert_ss    = {4'd1};
            bins pop_tx       = {4'd2};
            bins shift_ctrl   = {4'd3};
            bins shift_bit    = {4'd4};
            bins push_rx      = {4'd5};
            bins toggle_ss    = {4'd6};
            bins hold_mask    = {4'd7};
            bins mask_sclk    = {4'd8};
            bins hold_ss      = {4'd9};
            bins clear_ss     = {4'd10};
            bins sleep        = {4'd11};
            bins mwpop        = {4'd12};
            bins wait_ready   = {4'd13};
            bins clear_ready  = {4'd14};
        }
        cp_protocol: coverpoint protocol iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins spi0  = {2'd0};
            bins spi1  = {2'd1};
            bins ssp   = {2'd2};
            bins mwire = {2'd3};
        }
        cp_tmod: coverpoint cov_tmod iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins tmod_0 = {2'd0};
            bins tmod_1 = {2'd1};
            bins tmod_2 = {2'd2};
            bins tmod_3 = {2'd3};
        }
        cp_baud2: coverpoint cov_baud2 iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins normal_div = {1'b0};
            bins baud2      = {1'b1};
        }
        cp_sclk_edges: coverpoint {cov_sclk_re, cov_sclk_fe} iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins no_edge = {2'b00};
            bins rising  = {2'b10};
            bins falling = {2'b01};
        }
        cp_x_done: coverpoint cov_x_done iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins x_not_done = {1'b0};
            bins x_done     = {1'b1};
        }
        cp_c_done: coverpoint cov_c_done iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins c_not_done = {1'b0};
            bins c_done     = {1'b1};
        }
        cp_last_frame: coverpoint cov_last_frame iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins more_frames = {1'b0};
            bins last_frame  = {1'b1};
        }
        cp_shift_ctl: coverpoint {cov_tx_shift_en, cov_rx_shift_en} iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins shift_idle = {2'b00};
            bins tx_shift   = {2'b10};
            bins rx_shift   = {2'b01};
        }
        cp_srl_test: coverpoint cov_srl_test iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins normal_io = {1'b0};
            bins loopback  = {1'b1};
        }
        cp_ssi_busy: coverpoint cov_ssi_busy {
            bins idle     = {1'b0};
            bins transfer = {1'b1};
        }
        cp_cfs_effective_min: coverpoint (cov_cfs < SPEC_CFS_MIN && cov_eff_cfs == SPEC_CFS_MIN) iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins min_hit = {1'b1};
        }
        cp_cfs_effective_min_frame: coverpoint (cov_eff_cfs == SPEC_CFS_MIN && cov_cfs < SPEC_CFS_MIN &&
                                        cov_c_done) iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins min_frame_hit = {1'b1};
        }
        cp_cond_c: coverpoint cov_s3 iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins cond_c_0 = {1'b0};
            bins cond_c_1 = {1'b1};
        }
        cp_hold_ss: coverpoint cov_hold_ss_cnt iff (cov_fsm_state_id == 4'd9) {
            bins hold_0 = {4'd0};
            bins hold_1 = {4'd1};
            bins hold_2 = {4'd2};
            bins hold_3 = {4'd3};
        }
    endgroup

    // ============================================================
    // Covergroup: data_boundary
    // ============================================================
    covergroup data_boundary @(posedge clk);
        cp_dfs: coverpoint cov_eff_dfs iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins dfs_3  = {5'd3};
            bins dfs_4  = {5'd4};
            bins dfs_8  = {5'd8};
            bins dfs_16 = {5'd16};
            bins dfs_31 = {5'd31};
        }
        cp_cfs: coverpoint cov_eff_cfs iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins cfs_8  = {4'd8};
            bins cfs_12 = {4'd12};
            bins cfs_15 = {4'd15};
        }
        cp_baudr: coverpoint cov_baudr iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins baudr_2  = {16'd2};
            bins baudr_8  = {16'd8};
            bins baudr_16 = {16'd16};
        }
        cp_frame_cnt: coverpoint cov_frame_cnt iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins frame_0 = {17'd0};
            bins frame_1 = {17'd1};
            bins frame_2 = {17'd2};
        }
        cp_bit_cnt: coverpoint cov_bit_cnt iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins bit_0 = {6'd0};
            bins bit_8 = {6'd8};
        }
        cp_ctrl_cnt: coverpoint cov_ctrl_cnt iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins ctrl_0 = {4'd0};
            bins ctrl_8 = {4'd8};
        }
        cp_ndf: coverpoint cov_ndf iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins ndf_1  = {17'd1};
            bins ndf_4  = {17'd4};
            bins ndf_16 = {17'd16};
        }
        cp_hold_last_frame: coverpoint (cov_fsm_state_id == 4'd9 && cov_hold_ss_cnt == 4'd3 &&
                                      cov_last_frame && cov_frame_cnt >= 17'd1) {
            bins last_frame_hit = {1'b1};
        }
        // abort-then-recover: sticky cov_s0 is set the cycle a wrong-polarity
        // ss_in_n forces the FSM back to idle mid-transfer and cleared only
        // when a later transfer completes (x_done).  Each bin requires the
        // full multi-cycle sequence: start a transfer, abort it, then run a
        // fresh transfer to completion.
        cp_seq_a_0: coverpoint (cov_s0 &&
                                      (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11)) {
            bins seq_a_0 = {1'b1};
        }
        cp_seq_a_1: coverpoint (cov_s0 &&
                                      (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) &&
                                      cov_frame_cnt >= 17'd1) {
            bins seq_a_1 = {1'b1};
        }
        cp_seq_a_2: coverpoint (cov_s0 && cov_fsm_state_id == 4'd9 &&
                                           cov_hold_ss_cnt == 4'd3 && cov_last_frame &&
                                           cov_frame_cnt >= 17'd1) {
            bins seq_a_2 = {1'b1};
        }
        cp_txftlr: coverpoint cov_txftlr iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins txftlr_0   = {4'd0};
            bins txftlr_rst = {SPEC_TXFTLR_DFLT};
        }
        cp_rx_data: coverpoint cov_rx_data iff (cov_rx_level != 4'd0) {
            bins rx_all_zero   = {32'h00000000};
            bins rx_0x55       = {32'h00000055};
            bins rx_0xAA       = {32'h000000AA};
            bins rx_all_ones8  = {32'h000000FF};
            bins rx_all_ones32 = {32'hFFFFFFFF};
        }
        cp_ser: coverpoint cov_ser iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins slave_0 = {4'd1};
            bins slave_1 = {4'd2};
            bins slave_2 = {4'd4};
            bins slave_3 = {4'd8};
        }
        cp_tx_level: coverpoint cov_tx_level iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins tx_empty = {4'd0};
            bins tx_half  = {4'd4};
            bins tx_full  = {4'd8};
        }
        cp_rx_level: coverpoint cov_rx_level iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins rx_empty = {4'd0};
            bins rx_half  = {4'd4};
            bins rx_full  = {4'd8};
        }
        cp_risr: coverpoint cov_risr {
            bins tx_empty      = {6'd1};
            bins tx_overflow   = {6'd2};
            bins rx_underflow  = {6'd4};
            bins rx_overflow   = {6'd8};
            bins rx_full       = {6'd16};
            bins mst_collision = {6'd32};
        }
        cp_mwcr: coverpoint cov_mwcr iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins mwcr_0    = {3'd0};
            bins mwcr_ctrl = {3'd1};
            bins mwcr_full = {3'd3};
        }
    endgroup

    // ============================================================
    // Covergroup: cond_a / cond_b
    // ============================================================
    covergroup cond_a @(posedge clk);
        cp_ca_0: coverpoint cov_s1 {
            bins cond_a_0 = {1'b0};
            bins cond_a_1 = {1'b1};
        }
    endgroup

    covergroup cond_b @(posedge clk);
        cp_cb_0: coverpoint cov_s2 {
            bins cond_b_0 = {1'b0};
            bins cond_b_1 = {1'b1};
        }
    endgroup

    // fsm_protocol / data_boundary / cond_a / cond_b instances must be
    // declared before cross_functional, which crosses their coverpoints by
    // instance name.
    // NOTE: VCS T-2022.06 does not collect data from bare declarations
    // ("cg cg_inst;"); explicit construction ("= new;") is required.
    fsm_protocol     fsm_protocol_inst = new;
    data_boundary    data_boundary_inst = new;
    cond_a           cond_a_inst = new;
    cond_b           cond_b_inst = new;

    // ============================================================
    // Covergroup: cross_functional
    // ============================================================
    // Crosses below reference coverpoints of *other* covergroups
    // (fsm_protocol / data_boundary / cond_a / cond_b), so per IEEE 1800 §19.6 this
    // covergroup must NOT carry its own sampling event; sampling is driven by
    // the referenced covergroups' events.
    covergroup cross_functional;
        protocol_x_tmod: cross fsm_protocol_inst.cp_protocol, fsm_protocol_inst.cp_tmod
            iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins spi0_tmod2 = binsof(fsm_protocol_inst.cp_protocol.spi0) &&
                              binsof(fsm_protocol_inst.cp_tmod.tmod_2);
            bins spi0_tmod3 = binsof(fsm_protocol_inst.cp_protocol.spi0) &&
                              binsof(fsm_protocol_inst.cp_tmod.tmod_3);
            bins ssp_tmod2  = binsof(fsm_protocol_inst.cp_protocol.ssp) &&
                              binsof(fsm_protocol_inst.cp_tmod.tmod_2);
            bins ssp_tmod3  = binsof(fsm_protocol_inst.cp_protocol.ssp) &&
                              binsof(fsm_protocol_inst.cp_tmod.tmod_3);
        }
        protocol_x_mwcr_boundary: cross fsm_protocol_inst.cp_protocol, data_boundary_inst.cp_mwcr
            iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins mwire_cont = binsof(fsm_protocol_inst.cp_protocol.mwire) &&
                              binsof(data_boundary_inst.cp_mwcr.mwcr_0);
            bins mwire_ctrl = binsof(fsm_protocol_inst.cp_protocol.mwire) &&
                              binsof(data_boundary_inst.cp_mwcr.mwcr_ctrl);
            bins mwire_full = binsof(fsm_protocol_inst.cp_protocol.mwire) &&
                              binsof(data_boundary_inst.cp_mwcr.mwcr_full);
        }
        protocol_x_baud2_path: cross fsm_protocol_inst.cp_protocol, fsm_protocol_inst.cp_baud2
            iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins spi0_baud2  = binsof(fsm_protocol_inst.cp_protocol.spi0) &&
                               binsof(fsm_protocol_inst.cp_baud2.baud2);
            bins spi1_baud2  = binsof(fsm_protocol_inst.cp_protocol.spi1) &&
                               binsof(fsm_protocol_inst.cp_baud2.baud2);
            bins mwire_baud2 = binsof(fsm_protocol_inst.cp_protocol.mwire) &&
                               binsof(fsm_protocol_inst.cp_baud2.baud2);
        }
        protocol_x_eff_dfs_boundary: cross fsm_protocol_inst.cp_protocol, data_boundary_inst.cp_dfs
            iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins spi0_dfs8  = binsof(fsm_protocol_inst.cp_protocol.spi0) &&
                              binsof(data_boundary_inst.cp_dfs.dfs_8);
            bins ssp_dfs8   = binsof(fsm_protocol_inst.cp_protocol.ssp) &&
                              binsof(data_boundary_inst.cp_dfs.dfs_8);
            bins mwire_dfs8 = binsof(fsm_protocol_inst.cp_protocol.mwire) &&
                              binsof(data_boundary_inst.cp_dfs.dfs_8);
        }
        protocol_x_hold_ss_cnt_boundary: cross fsm_protocol_inst.cp_protocol, fsm_protocol_inst.cp_hold_ss
            iff (cov_fsm_state_id != 4'd0 && cov_fsm_state_id != 4'd11) {
            bins spi1_hold3 = binsof(fsm_protocol_inst.cp_protocol.spi1) &&
                              binsof(fsm_protocol_inst.cp_hold_ss.hold_3);
            bins ssp_hold3  = binsof(fsm_protocol_inst.cp_protocol.ssp) &&
                              binsof(fsm_protocol_inst.cp_hold_ss.hold_3);
        }
        cross_a: cross cond_a_inst.cp_ca_0, fsm_protocol_inst.cp_ssi_busy {
            bins cross_a_0 = binsof(cond_a_inst.cp_ca_0.cond_a_1) &&
                             binsof(fsm_protocol_inst.cp_ssi_busy.transfer);
        }
    endgroup

    // Covergroup instantiation (IEEE 1800: no parens). The cross_functional
    // instance follows its definition. Explicit "= new;" required for VCS
    // T-2022.06 data collection (see note above).
    cross_functional cross_functional_inst = new;

endmodule
