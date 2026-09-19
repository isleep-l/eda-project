// dma_xfer_public - dma_top.v (top module)
//
// Multi-channel DMA transfer controller. Single-clock RTL mirroring local_sim.py.
//
// Hidden parameters (replaced at evaluation via module parameters; the public
// placeholder values simulate but use no "correct" modes so done_ok is inert):
//   ARB_PRIO        : channel priority permutation (index 0 = highest priority)
//   DONE_OK_MODES   : {m3,m2,m1,m0} four 4-bit dir_mode values. A transfer
//                     whose dir_mode equals one of these is "correct" and on
//                     completion opens the done-hold window; the matching mode
//                     maps to done_class 0/1/2/3 respectively.
//   XFER_HOLD_N     : done-hold window length (cycles)
//
// Interface / actions are decoded by the harness from the 15-dim action vector;
// this module offers raw configuration pins.
//   conf_ch / conf_wr / conf_field / conf_data : register write.
//     field 0 = saddr, field 1 = daddr, field 2 = {burst[23:21], dmode[20:16], len[15:0]}
//   start[3:0] : per-channel rising-edge start.
//
// FSM per channel:
//   IDLE -> (start) -> ACTIVE -> (last beat) -> DONE -> (one cycle) -> IDLE
//   A zero-length config never enters ACTIVE; start with len==0 -> DONE.
//
// Coverage observation signals exposed on cov_* outputs (verilator public).
// License: MIT

`define IDLE    2'd0
`define ACTIVE  2'd1
`define DONE    2'd2

module dma_top #(
    parameter ADDR_W       = 32,
    parameter LEN_W        = 16,
    parameter N_CH         = 4,
    parameter [15:0] DONE_OK_MODES = 16'h0000,  // hidden {m3,m2,m1,m0}
    parameter [15:0] XFER_HOLD_N   = 16'd16,    // hidden
    parameter [7:0]  ARB_PRIO      = 8'h10 | 8'h20 | 8'h30  // hidden (ascending)
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire [1:0]       conf_ch,
    input  wire             conf_wr,
    input  wire [1:0]       conf_field,
    input  wire [31:0]      conf_data,
    input  wire [N_CH-1:0]  start,

    // Coverage observation bus (verilator public)
    output wire [1:0]       cov_ch0_state,
    output wire [1:0]       cov_ch1_state,
    output wire [1:0]       cov_ch2_state,
    output wire [1:0]       cov_ch3_state,
    output reg  [1:0]       cov_arb_winner,
    output reg              cov_arb_valid,
    output reg              cov_arb_conflict,
    output reg  [1:0]       cov_active_ch,
    output reg  [3:0]       cov_dir,
    output reg  [2:0]       cov_burst,
    output reg  [ADDR_W-1:0] cov_saddr,
    output reg  [ADDR_W-1:0] cov_daddr,
    output reg  [LEN_W-1:0]  cov_len,
    output reg              cov_hold,
    output reg              cov_done_ok_valid,
    output reg  [3:0]       cov_done_class,

    // Temporal sequence cover hits (verilator public) — SVA cover-property
    // equivalents. A 1 means the sequence fired for the current cycle.
    output reg              cov_seq1,
    output reg              cov_seq2,
    output reg              cov_seq3,
    output reg              cov_seq4,
    output reg              cov_seq5,
    output reg              cov_seq6,
    output reg              cov_seq7,
    output reg              cov_seq8,
    output reg              cov_seq9,
    output reg              cov_seq10,
    output reg              cov_seq11,
    output reg              cov_seq12
);

    // per-channel registers
    reg [ADDR_W-1:0] saddr[0:N_CH-1];
    reg [ADDR_W-1:0] daddr[0:N_CH-1];
    reg [LEN_W-1:0]  len  [0:N_CH-1];
    reg [3:0]        dmode[0:N_CH-1];
    reg [2:0]        burst[0:N_CH-1];
    reg [1:0]        fsm  [0:N_CH-1];
    reg [LEN_W-1:0]  remain[0:N_CH-1];
    reg              got  [0:N_CH-1];
    reg              hevt [0:N_CH-1];   // one-cycle completion event
    reg [N_CH-1:0]   start_d;

    integer i, bi;

    // --------------------------------------------------------------
    // Register config + start-latch + DONE->IDLE auto-return
    // --------------------------------------------------------------
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            for (bi = 0; bi < N_CH; bi = bi + 1) begin
                saddr[bi] <= {ADDR_W{1'b0}};
                daddr[bi] <= {ADDR_W{1'b0}};
                len[bi]   <= {LEN_W{1'b0}};
                dmode[bi] <= 4'd0;
                burst[bi] <= 3'd0;
                fsm[bi]   <= `IDLE;
                remain[bi]<= {LEN_W{1'b0}};
                got[bi]   <= 1'b0;
                hevt[bi]  <= 1'b0;
            end
            start_d <= {N_CH{1'b0}};
        end else begin
            start_d <= start;

            // DONE -> IDLE after one sampled DONE cycle (channels reusable)
            for (bi = 0; bi < N_CH; bi = bi + 1)
                if (fsm[bi] == `DONE && !got[bi])
                    fsm[bi] <= `IDLE;

            // config write (blocked while a request is latched, transfer active,
            // or the global hold window is engaged)
            if (conf_wr && fsm[conf_ch] == `IDLE && !got[conf_ch] && !cov_hold) begin
                i = conf_ch;
                case (conf_field)
                    2'd0: saddr[i] <= conf_data[ADDR_W-1:0];
                    2'd1: daddr[i] <= conf_data[ADDR_W-1:0];
                    2'd2: begin
                        len[i]   <= conf_data[LEN_W-1:0];
                        remain[i]<= conf_data[LEN_W-1:0];
                        dmode[i] <= conf_data[20:16];
                        burst[i] <= conf_data[23:21];
                    end
                    default: ;
                endcase
            end

            // start: rising edge latches request
            for (bi = 0; bi < N_CH; bi = bi + 1)
                if (start[bi] && !start_d[bi] && !got[bi]) begin
                    got[bi]   <= 1'b1;
                    remain[bi]<= len[bi];
                    fsm[bi]   <= (len[bi] == {LEN_W{1'b0}}) ? `DONE : `ACTIVE;
                end
        end

    // --------------------------------------------------------------
    // Arbitration (grant highest-priority requesting channel)
    // --------------------------------------------------------------
    wire [N_CH-1:0] req;
    assign req[0] = got[0] && (fsm[0] == `ACTIVE);
    assign req[1] = got[1] && (fsm[1] == `ACTIVE);
    assign req[2] = got[2] && (fsm[2] == `ACTIVE);
    assign req[3] = got[3] && (fsm[3] == `ACTIVE);
    wire [N_CH-1:0] req_nh = req & ~{4{cov_hold}};
    wire arb_valid = (|req_nh);

    reg [1:0] w_ord0, w_ord1, w_ord2, w_ord3;
    wire [1:0] ord0 = ARB_PRIO[1:0];
    wire [1:0] ord1 = ARB_PRIO[3:2];
    wire [1:0] ord2 = ARB_PRIO[5:4];
    wire [1:0] ord3 = ARB_PRIO[7:6];
    always @(*) begin
        // consumed to avoid unused-warning
        w_ord0 = ord0; w_ord1 = ord1; w_ord2 = ord2; w_ord3 = ord3;
    end

    // --------------------------------------------------------------
    // Arbitrator instance
    // --------------------------------------------------------------
    wire [N_CH-1:0] g;
    wire [1:0]      w;
    arbiter #(.N_CH(N_CH), .PRIO_BITS(8), .ARB_PRIO(ARB_PRIO))
      u_arb ( .clk(clk), .rst_n(rst_n),
              .req(req_nh), .grant(g), .winner(w), .grant_valid() );

    // --------------------------------------------------------------
    // Beat consumption + completion detection (mirrors local_sim)
    // --------------------------------------------------------------
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            for (bi = 0; bi < N_CH; bi = bi + 1) begin
                fsm[bi]  <= `IDLE;
                remain[bi] <= {LEN_W{1'b0}};
                got[bi]  <= 1'b0;
                hevt[bi] <= 1'b0;
            end
        end else if (!cov_hold) begin
            for (bi = 0; bi < N_CH; bi = bi + 1)
                if (got[bi] && g[bi] && fsm[bi] == `ACTIVE) begin
                    if (remain[bi] == 1) begin
                        fsm[bi]   <= `DONE;
                        got[bi]   <= 1'b0;
                        remain[bi]<= {LEN_W{1'b0}};
                        hevt[bi]  <= 1'b1;
                    end else
                        remain[bi] <= remain[bi] - 1;
                end
        end

    // --------------------------------------------------------------
    // Done-hold window + xfer_done_ok pulse
    // --------------------------------------------------------------
    reg [1:0]   hld_state;
    reg [15:0]  hld_cnt;
    reg [3:0]   hld_class;
    wire [3:0]  any_hevt = {hevt[3], hevt[2], hevt[1], hevt[0]};
    wire [3:0]  comp_dmode = hevt[0] ? dmode[0] :
                             hevt[1] ? dmode[1] :
                             hevt[2] ? dmode[2] : dmode[3];
    wire comp_correct =
        (comp_dmode == DONE_OK_MODES[3:0]) ||
        (comp_dmode == DONE_OK_MODES[7:4]) ||
        (comp_dmode == DONE_OK_MODES[11:8]) ||
        (comp_dmode == DONE_OK_MODES[15:12]);

    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            hld_state <= `IDLE;
            hld_cnt   <= 16'd0;
            hld_class <= 4'd0;
            cov_hold  <= 1'b0;
            cov_done_ok_valid <= 1'b0;
            cov_done_class    <= 4'd0;
        end else begin
            cov_done_ok_valid <= 1'b0;
            case (hld_state)
                `IDLE: begin
                    if (|any_hevt) begin
                        if      (comp_dmode == DONE_OK_MODES[3:0])   hld_class <= 4'd0;
                        else if (comp_dmode == DONE_OK_MODES[7:4])   hld_class <= 4'd1;
                        else if (comp_dmode == DONE_OK_MODES[11:8])  hld_class <= 4'd2;
                        else if (comp_dmode == DONE_OK_MODES[15:12]) hld_class <= 4'd3;
                        if (comp_correct) begin
                            hld_state <= `ACTIVE;
                            hld_cnt   <= XFER_HOLD_N;
                            cov_hold  <= 1'b1;
                        end
                    end
                end
                `ACTIVE: begin
                    if (hld_cnt == 1) begin
                        cov_done_ok_valid <= 1'b1;
                        cov_done_class    <= hld_class;
                        hld_state <= `DONE;
                        cov_hold  <= 1'b0;
                    end else
                        hld_cnt <= hld_cnt - 1;
                end
                `DONE: hld_state <= `IDLE;
                default: hld_state <= `IDLE;
            endcase
        end

    // --------------------------------------------------------------
    // Coverage observation (cookie = last config write or last grant)
    // --------------------------------------------------------------
    wire arb_conflict_w = ( (req[0] + req[1] + req[2] + req[3]) > 1 );
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            cov_arb_winner <= 2'd0;
            cov_arb_valid  <= 1'b0;
            cov_arb_conflict <= 1'b0;
            cov_active_ch  <= 2'd0;
            cov_dir <= 4'd0; cov_burst <= 3'd0;
            cov_saddr <= {ADDR_W{1'b0}}; cov_daddr <= {ADDR_W{1'b0}}; cov_len <= {LEN_W{1'b0}};
        end else if (arb_valid && !cov_hold) begin
            cov_arb_winner  <= w;
            cov_arb_valid   <= 1'b1;
            cov_arb_conflict<= arb_conflict_w;
            cov_active_ch   <= w;
            cov_dir         <= dmode[w];
            cov_burst       <= burst[w];
            cov_saddr       <= saddr[w];
            cov_daddr       <= daddr[w];
            cov_len         <= len[w];
        end else if (conf_wr && fsm[conf_ch] == `IDLE && !got[conf_ch] && !cov_hold) begin
            cov_active_ch  <= conf_ch;
            cov_dir        <= dmode[conf_ch];
            cov_burst      <= burst[conf_ch];
            cov_saddr      <= saddr[conf_ch];
            cov_daddr      <= daddr[conf_ch];
            cov_len        <= len[conf_ch];
            cov_arb_valid  <= 1'b0;
            cov_arb_conflict<= 1'b0;
        end else begin
            cov_arb_valid  <= 1'b0;
            cov_arb_conflict<= 1'b0;
        end

    assign cov_ch0_state = fsm[0];
    assign cov_ch1_state = fsm[1];
    assign cov_ch2_state = fsm[2];
    assign cov_ch3_state = fsm[3];

    // ============================================================
    // Temporal sequence monitor (SVA cover-property equivalents)
    // Sequential flags are advanced on each clock edge and the cov_seq*
    // pulse reflects the sequence firing for the current cycle.
    // ============================================================
    wire cfg_write_ok = conf_wr && fsm[conf_ch] == `IDLE && !got[conf_ch] && !cov_hold;
    reg  [N_CH-1:0] start_d2;
    wire [N_CH-1:0] start_rise2 = start & ~start_d2;

    // Q1 handshake stage machine
    reg        s1_cfg;
    reg        s1_start;
    reg        s1_active;
    // Q3 clean window
    reg        s3_started;
    reg        s3_cfgs;
    // Q5 prev hold
    reg        hold_prev;
    // Q6 conflict recent counter
    reg  [2:0] conf_cnt;
    // Q7 arb continuous counter
    reg  [4:0] arb_cnt;
    // Q9 idle continuous counter
    reg  [4:0] idle_cnt;
    // Q10 active->done per channel
    reg  [3:0] a2d [0:N_CH-1];
    // Q2 per-channel ACTIVE continuity
    reg  [3:0] acont [0:N_CH-1];
    // Q4 hold continuity
    reg  [4:0] hold_cnt_internal;
    // Q11 prev winner
    reg  [1:0] pw;
    reg        pw_valid;

    wire [N_CH-1:0] any_active_v = {fsm[3]==`ACTIVE, fsm[2]==`ACTIVE,
                                    fsm[1]==`ACTIVE, fsm[0]==`ACTIVE};
    wire [N_CH-1:0] any_done_v   = {fsm[3]==`DONE && !got[3],
                                    fsm[2]==`DONE && !got[2],
                                    fsm[1]==`DONE && !got[1],
                                    fsm[0]==`DONE && !got[0]};
    wire           got_any = |{got[3], got[2], got[1], got[0]};
    wire           a_any_start = |start_rise2;
    wire           a_any_active = |any_active_v;
    wire           a_any_done   = |any_done_v;

    integer qi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start_d2    <= {N_CH{1'b0}};
            s1_cfg <= 1'b0; s1_start <= 1'b0; s1_active <= 1'b0;
            s3_started <= 1'b0; s3_cfgs <= 1'b0;
            hold_prev <= 1'b0;
            conf_cnt <= 3'd0;
            arb_cnt  <= 5'd0;
            idle_cnt <= 5'd0;
            pw <= 2'd0; pw_valid <= 1'b0;
            cov_seq1<='b0; cov_seq2<='b0; cov_seq3<='b0; cov_seq4<='b0;
            cov_seq5<='b0; cov_seq6<='b0; cov_seq7<='b0; cov_seq8<='b0;
            cov_seq9<='b0; cov_seq10<='b0; cov_seq11<='b0; cov_seq12<='b0;
        end else begin
            start_d2 <= start;
            cov_seq1<='b0; cov_seq2<='b0; cov_seq3<='b0; cov_seq4<='b0;
            cov_seq5<='b0; cov_seq6<='b0; cov_seq7<='b0; cov_seq8<='b0;
            cov_seq9<='b0; cov_seq10<='b0; cov_seq11<='b0; cov_seq12<='b0;

            // ---- Q1: cfg -> start -> active -> done ----
            if (cfg_write_ok) begin
                s1_cfg <= 1'b1; s1_start <= 1'b0; s1_active <= 1'b0;
            end else if (s1_cfg && !s1_start && a_any_start) begin
                s1_start <= 1'b1;
            end else if (s1_cfg && s1_start && !s1_active && a_any_active) begin
                s1_active <= 1'b1;
            end else if (s1_cfg && s1_start && s1_active && a_any_done) begin
                cov_seq1 <= 1'b1;
                s1_cfg <= 1'b0;
            end

            // ---- Q2: same channel ACTIVE >=3 ----
            for (qi = 0; qi < N_CH; qi = qi + 1) begin
                if (fsm[qi] == `ACTIVE) begin
                    if (acont[qi] >= 2) cov_seq2 <= 1'b1;
                    acont[qi] <= acont[qi] + 1;
                end else
                    acont[qi] <= 4'd0;
            end

            // ---- Q3: clean correct transfer ----
            if (cfg_write_ok) begin
                s3_started <= 1'b0; s3_cfgs <= 1'b0;
            end
            if (a_any_start) begin
                s3_started <= 1'b1; s3_cfgs <= 1'b0;
            end
            if (s3_started && cfg_write_ok) s3_cfgs <= 1'b1;
            if (s3_started && cov_done_ok_valid) begin
                if (!s3_cfgs) cov_seq3 <= 1'b1;
                s3_started <= 1'b0;
            end

            // ---- Q4: hold >=4 ----
            if (cov_hold) begin
                if (hold_cnt_internal >= 3) cov_seq4 <= 1'b1;  // 0-indexed 4th cycle
                hold_cnt_internal <= hold_cnt_internal + 1;
            end else
                hold_cnt_internal <= 5'd0;

            // ---- Q5: hold 1->0 then done_ok ----
            if (cov_done_ok_valid && hold_prev) cov_seq5 <= 1'b1;
            hold_prev <= cov_hold;

            // ---- Q6: conflict resolved ----
            if (arb_conflict_w) conf_cnt <= 3'd2;
            if (conf_cnt != 0) begin
                if (arb_valid) cov_seq6 <= 1'b1;
                conf_cnt <= conf_cnt - 1;
            end

            // ---- Q7: arb >=2 ----
            if (arb_valid) begin
                if (arb_cnt >= 1) cov_seq7 <= 1'b1;
                arb_cnt <= arb_cnt + 1;
            end else
                arb_cnt <= 5'd0;

            // ---- Q8: DONE observed ----
            if (a_any_done) cov_seq8 <= 1'b1;

            // ---- Q9: idle >=3 ----
            if (fsm[0]==`IDLE && fsm[1]==`IDLE && fsm[2]==`IDLE && fsm[3]==`IDLE
                && !got_any && !cov_hold) begin
                idle_cnt <= idle_cnt + 1;
                if (idle_cnt >= 2) cov_seq9 <= 1'b1;
            end else
                idle_cnt <= 5'd0;

            // ---- Q10: ACTIVE->DONE <=2 ----
            for (qi = 0; qi < N_CH; qi = qi + 1) begin
                if (fsm[qi] == `ACTIVE && !cov_hold) begin
                    a2d[qi] <= a2d[qi] + 1;
                end else if (fsm[qi] == `DONE && !got[qi] && !cov_hold) begin
                    if (a2d[qi] >= 1 && a2d[qi] <= 2) cov_seq10 <= 1'b1;
                    a2d[qi] <= 4'd0;
                end else
                    a2d[qi] <= 4'd0;
            end

            // ---- Q11: two channels consecutive ----
            if (arb_valid && !cov_hold) begin
                if (pw_valid && pw != w) cov_seq11 <= 1'b1;
                pw <= w;
                pw_valid <= 1'b1;
            end else
                pw_valid <= 1'b0;

            // ---- Q12: done_ok class ~ active channel ----
            if (cov_done_ok_valid) begin
                if (cov_active_ch == cov_done_class[1:0]) cov_seq12 <= 1'b1;
            end
        end
    end

endmodule
`undef IDLE
`undef ACTIVE
`undef DONE
