// dma_xfer_public - assert_cov.svh
// SystemVerilog Assertions (SVA): cover property equivalents of the 12 temporal
// sequence coverpoints in coverage_meta.json. These document the intended
// temporal sequences in SVA form (VCS-native) and mirror exactly the seq_hit
// behavior implemented in local_sim.py's temporal monitor and dma_top.v's
// cov_seq* outputs.
//
// The sequences are strong multi-cycle temporal constraints: hitting them
// requires constructing real, uninterrupted transfer handshakes — much harder
// than single-cycle value coverage.
// License: MIT
`default_nettype none
module assert_cov #(
    parameter N_CH = 4
) (
    input wire              clk,
    input wire              rst_n,

    // monitoring inputs (same bus dma_top exposes on its cov_* output)
    input wire [1:0]        cov_ch0_state,
    input wire [1:0]        cov_ch1_state,
    input wire [1:0]        cov_ch2_state,
    input wire [1:0]        cov_ch3_state,
    input wire [3:0]        cov_state,   // packed {ch3,ch2,ch1,ch0} raw for sequences
    input wire              cov_hold,
    input wire              cov_done_ok_valid,
    input wire [3:0]        cov_done_class,
    input wire [1:0]        cov_active_ch,
    input wire              cov_arb_valid,
    input wire [1:0]        cov_arb_winner,
    input wire              cov_arb_conflict,

    // program-side events (raw pins, not through cov_*)
    input wire [1:0]        conf_ch,
    input wire              conf_wr,
    input wire              cfg_write_ok,       // config write actually accepted
    input wire [N_CH-1:0]   start_rise          // per-channel start rising edge
);

    wire [1:0] s0 = cov_ch0_state, s1 = cov_ch1_state, s2 = cov_ch2_state, s3 = cov_ch3_state;
    localparam IDLE = 2'd0, ACTIVE = 2'd1, DONE = 2'd2;

    wire any_active = (s0==ACTIVE) || (s1==ACTIVE) || (s2==ACTIVE) || (s3==ACTIVE);
    wire any_done   = (s0==DONE) || (s1==DONE) || (s2==DONE) || (s3==DONE);
    // any_done below requires the channel also completed (got released). For the
    // cover property we approximate "a channel reached DONE" with any_done.

    // -------------------------------------------------------------
    // Q1: cfg -> start -> active -> done  (complete handshake)
    // -------------------------------------------------------------
    property p_seq1;
        @(posedge clk) disable iff (!rst_n)
            cfg_write_ok ##1 start_rise ##1 any_active ##1 any_done;
    endproperty
    cover property (p_seq1);

    // -------------------------------------------------------------
    // Q2: same channel ACTIVE >=3 consecutive cycles
    // -------------------------------------------------------------
    property p_seq2(c);
        @(posedge clk) disable iff (!rst_n)
            (c == ACTIVE[*3]);
    endproperty
    cover property (p_seq2(s0));
    cover property (p_seq2(s1));
    cover property (p_seq2(s2));
    cover property (p_seq2(s3));

    // -------------------------------------------------------------
    // Q3: clean correct transfer (start -> done_ok, no config interrupt)
    // -------------------------------------------------------------
    property p_seq3;
        @(posedge clk) disable iff (!rst_n)
            (!cfg_write_ok) throughout (start_rise ##[1:$] cov_done_ok_valid);
    endproperty
    cover property (p_seq3);

    // -------------------------------------------------------------
    // Q4: hold >=4 consecutive cycles (done-hold window persists)
    // -------------------------------------------------------------
    sequence s_hold4;
        cov_hold[*4];
    endsequence
    cover property (@(posedge clk) disable iff (!rst_n) s_hold4);

    // -------------------------------------------------------------
    // Q5: hold deasserts, then done_ok next cycle
    // -------------------------------------------------------------
    property p_seq5;
        @(posedge clk) disable iff (!rst_n)
            cov_hold ##1 !cov_hold ##1 cov_done_ok_valid;
    endproperty
    cover property (p_seq5);

    // -------------------------------------------------------------
    // Q6: arbitration conflict resolved within 2 cycles
    // -------------------------------------------------------------
    property p_seq6;
        @(posedge clk) disable iff (!rst_n)
            cov_arb_conflict ##[1:2] cov_arb_valid;
    endproperty
    cover property (p_seq6);

    // -------------------------------------------------------------
    // Q7: arbitration active >=2 consecutive cycles
    // -------------------------------------------------------------
    sequence s_arb2;
        cov_arb_valid[*2];
    endsequence
    cover property (@(posedge clk) disable iff (!rst_n) s_arb2);

    // -------------------------------------------------------------
    // Q8: a channel completes (DONE observed)
    // -------------------------------------------------------------
    cover property (@(posedge clk) disable iff (!rst_n) any_done);

    // -------------------------------------------------------------
    // Q9: global idle >=3 cycles (no request, no hold)
    // -------------------------------------------------------------
    sequence s_idle3;
        (s0==IDLE && s1==IDLE && s2==IDLE && s3==IDLE && !cov_hold)[*3];
    endsequence
    cover property (@(posedge clk) disable iff (!rst_n) s_idle3);

    // -------------------------------------------------------------
    // Q10: ACTIVE -> DONE within 2 cycles and no hold
    // -------------------------------------------------------------
    property p_seq10(c);
        @(posedge clk) disable iff (!rst_n)
            (c == ACTIVE && !cov_hold) ##[1:2] (c == DONE && !cov_hold);
    endproperty
    cover property (p_seq10(s0));
    cover property (p_seq10(s1));
    cover property (p_seq10(s2));
    cover property (p_seq10(s3));

    // -------------------------------------------------------------
    // Q11: two different channels granted in consecutive cycles
    // -------------------------------------------------------------
    property p_seq11;
        @(posedge clk) disable iff (!rst_n)
            cov_arb_valid ##1 cov_arb_valid && (cov_arb_winner != $past(cov_arb_winner, 1) );
    endproperty
    cover property (p_seq11);

    // -------------------------------------------------------------
    // Q12: done_ok class consistent with active channel
    // -------------------------------------------------------------
    property p_seq12;
        @(posedge clk) disable iff (!rst_n)
            cov_done_ok_valid && (cov_active_ch == cov_done_class[1:0]);
    endproperty
    cover property (p_seq12);

endmodule
`default_nettype wire
