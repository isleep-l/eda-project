// File    : spi_mstfsm.v
// Abstract: spi_xfer_public SPI/SSP master transfer FSM.
//           Clean re-implementation of the fl_ssi master FSM for the
//           SPI0 / SPI1 / SSP frame formats (the Microwire control
//           frame handshake is out of scope for this controller).
//
// States
//   IDLE      : waiting for a transfer to start
//   ASSERT_SS : assert slave select, wait for first sclk edge
//   POP_TX    : pop/load TX data into the shift register
//   SHIFT_BIT : shift one data frame out/in
//   PUSH_RX   : capture RX data, load next TX frame
//   TOGGLE_SS : de-assert slave select between frames (SSP / SSTE)
//   HOLD_MASK : idle slot on the serial bus (SPI0)
//   MASK_SCLK : sclk idle, prepare for next frame / completion
//   HOLD_SS   : trailing chip-select hold (SPEC_HOLD_SS half periods)
//   CLEAR_SS  : de-assert slave select, finish transfer
//   SLEEP     : SSI disabled
module spi_mstfsm
  (
   input  wire        clk,
   input  wire        rst_n,
   input  wire        ssi_en,          // SSI enable
   input  wire        start_xfer,      // start transfer request
   input  wire        tx_empty,        // TX FIFO empty
   input  wire [3:0]  tx_fifo_level,   // TX FIFO level (0..8)
   input  wire [3:0]  txfthr,          // TX FIFO threshold
   input  wire        sclk_re,         // sclk rising edge pulse
   input  wire        sclk_fe,         // sclk falling edge pulse
   input  wire [1:0]  frf,             // frame format: 0=SPI, 1=SSP
   input  wire        scph,            // SPI clock phase
   input  wire [1:0]  tmod,            // transfer mode
   input  wire [5:0]  dfs,             // effective data frame size (clamped)
   input  wire [16:0] ndf,             // number of data frames
   input  wire        ss_tgl_en,       // slave select toggle enable
   input  wire        ss_in_n,         // slave select input
   input  wire        baud2,           // baud divisor == 2
   input  wire [3:0]  spec_hold_ss,    // hidden: HOLD_SS state duration

   output reg  [3:0]  c_state,         // registered current state
   output wire        fsm_busy,        // busy status
   output wire        fsm_sleep,       // sleep status
   output wire        sclk_mask,       // serial clock mask
   output wire        fsm_slv_sel,     // slave select enable for shift
   output wire        load_tx_shift,   // load TX shift register
   output wire        load_tx_buffer,  // reload TX shift register from buffer
   output wire        load_rx_buf,     // load RX buffer
   output wire        tx_shift_en,     // TX shift enable
   output wire        rx_shift_en,     // RX shift enable
   output wire [5:0]  tx_dfs,          // frame size for TX shifter
   output wire        ssi_oe_n,        // output enable
   // -- coverage observation
   output wire [10:0] cov_fsm_state,   // one-hot state
   output wire [5:0]  cov_bit_cnt,
   output wire [16:0] cov_frame_cnt,
   output wire        cov_x_done,
   output wire        cov_last_frame,
   output wire        cov_tx_shift_en,
   output wire        cov_rx_shift_en,
   output wire        cov_load_tx_shift,
   output wire        cov_load_rx_buf,
   output wire [3:0]  cov_hold_ss_cnt,
   output wire        cov_s0,           // sticky coverage status flag
   output wire        cov_s1            // sticky coverage status flag
   );

  // -- state encoding
  localparam S_IDLE       = 4'd0;
  localparam S_ASSERT_SS  = 4'd1;
  localparam S_POP_TX     = 4'd2;
  localparam S_SHIFT_BIT  = 4'd3;
  localparam S_PUSH_RX    = 4'd4;
  localparam S_TOGGLE_SS  = 4'd5;
  localparam S_HOLD_MASK  = 4'd6;
  localparam S_MASK_SCLK  = 4'd7;
  localparam S_HOLD_SS    = 4'd8;
  localparam S_CLEAR_SS   = 4'd9;
  localparam S_SLEEP      = 4'd10;

  // -- protocol decode
  wire spi0 = (frf == 2'b00) && (scph == 1'b0);
  wire spi1 = (frf == 2'b00) && (scph == 1'b1);
  wire ssp  = (frf == 2'b01);

  // -- internal sclk edges (SPI0 samples on the opposite edge)
  wire sclk_re_int = spi0 ? sclk_fe : sclk_re;
  wire sclk_fe_int = spi0 ? sclk_re : sclk_fe;

  // -- local registers and wires
  reg  [5:0]  bit_cnt;
  reg  [16:0] frame_cnt;
  reg         last_frame;
  reg         x_done_1d;
  reg         tx_load_en;
  reg         abort_ir;
  reg  [3:0]  hold_cnt;
  reg  [3:0]  n_state;

  wire [5:0]  dfs_eff  = dfs;
  wire        f_done   = (bit_cnt == dfs_eff);
  wire        sspf_done= (bit_cnt == (dfs_eff - 6'd1));
  wire        tx_fifo_filled = (tmod == 2'b10) ? 1'b1 : (tx_fifo_level > txfthr);
  wire        x_done   = ((tmod == 2'b11) && (frf == 2'b00) || (tmod == 2'b10)) ?
                         (frame_cnt == ndf) : last_frame;
  wire        inc_bit_cnt    = (c_state == S_SHIFT_BIT) && sclk_re_int;
  wire        clr_bit_cnt    = ((c_state == S_TOGGLE_SS) && sclk_re_int && (ssp || ss_tgl_en)) ||
                               ((c_state == S_HOLD_MASK) && sclk_re_int && (!ssp) && (!ss_tgl_en)) ||
                               ((c_state == S_PUSH_RX)   && sclk_re_int) ||
                               ((c_state == S_IDLE)      && sclk_re_int);
  wire        inc_frame_cnt  = (c_state == S_SHIFT_BIT) && (bit_cnt == (dfs_eff - 6'd2)) && sclk_re_int;
  wire        clr_frame_cnt  = (c_state == S_IDLE) ||
                               ((c_state == S_CLEAR_SS) && (n_state != S_CLEAR_SS));

  // -- bit counter
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bit_cnt <= 5'd0;
    end else begin
      if (clr_bit_cnt) begin
        bit_cnt <= 6'd0;
      end else if (inc_bit_cnt) begin
        bit_cnt <= bit_cnt + 6'd1;
      end
    end
  end

  // -- frame counter
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      frame_cnt <= 17'd0;
    end else begin
      if (clr_frame_cnt) begin
        frame_cnt <= 17'd0;
      end else if (inc_frame_cnt) begin
        frame_cnt <= frame_cnt + 17'd1;
      end
    end
  end

  // -- last frame decode
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      last_frame <= 1'b0;
    end else begin
      if (tx_empty && (sclk_fe_int || baud2) && (c_state != S_MASK_SCLK)) begin
        last_frame <= 1'b1;
      end else if ((!tx_empty) && ((c_state == S_SHIFT_BIT) || (c_state == S_IDLE))) begin
        last_frame <= 1'b0;
      end
    end
  end

  // -- delayed transfer-done
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      x_done_1d <= 1'b0;
    end else begin
      x_done_1d <= x_done;
    end
  end

  // -- TX load enable
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_load_en <= 1'b1;
    end else begin
      case (c_state)
        S_POP_TX : begin
          if (sclk_re_int || baud2) begin
            tx_load_en <= (tmod == 2'b10) ? 1'b0 : 1'b1;
          end
        end
        S_IDLE : tx_load_en <= 1'b1;
        default: tx_load_en <= tx_load_en;
      endcase
    end
  end

  // -- abort transfer (SPI0: data written into empty TX FIFO during HOLD_MASK)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      abort_ir <= 1'b0;
    end else begin
      if ((c_state == S_HOLD_MASK) && x_done) begin
        abort_ir <= 1'b1;
      end else if ((c_state == S_IDLE) || (c_state == S_CLEAR_SS)) begin
        abort_ir <= 1'b0;
      end
    end
  end

  // -- sticky coverage flags
  //  abort_seen : a wrong ss_in_n polarity forced the FSM back to IDLE
  //               mid-transfer (same condition that overrides n_state in the
  //               state register).  Set dominates clear so the flag survives
  //               an abort that happens in the same cycle n_state would leave
  //               the transfer; cleared only while exiting a transfer.
  //  refill_seen: TX FIFO drained empty while a transfer is in progress and
  //               at least one frame has already been emitted; cleared only
  //               while exiting a transfer.
  wire abort_now  = ssi_en && ((ssp && ss_in_n) || ((!ssp) && (!ss_in_n))) &&
                    (c_state != S_IDLE) && (c_state != S_SLEEP);
  wire busy_now   = (n_state != S_IDLE) && (n_state != S_SLEEP);
  wire refill_now = busy_now && tx_empty && (frame_cnt >= 17'd1);

  reg abort_seen_r;
  reg refill_seen_r;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      abort_seen_r <= 1'b0;
    end else if (abort_now) begin
      abort_seen_r <= 1'b1;
    end else if (n_state == S_CLEAR_SS) begin
      abort_seen_r <= 1'b0;
    end
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      refill_seen_r <= 1'b0;
    end else if (refill_now) begin
      refill_seen_r <= 1'b1;
    end else if (n_state == S_CLEAR_SS) begin
      refill_seen_r <= 1'b0;
    end
  end
  assign cov_s0 = abort_seen_r;
  assign cov_s1 = refill_seen_r;

  // -- HOLD_SS duration counter
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      hold_cnt <= 4'd0;
    end else begin
      if (c_state != S_HOLD_SS) begin
        hold_cnt <= 4'd0;
      end else if (sclk_fe_int || baud2) begin
        hold_cnt <= (hold_cnt == (spec_hold_ss - 4'd1)) ? 4'd0 : (hold_cnt + 4'd1);
      end
    end
  end

  // -- next state logic
  always_comb begin
    case (c_state)
      S_IDLE : begin
        if (start_xfer && sclk_re_int && tx_fifo_filled && (!tx_empty) &&
            (((!ssp) && ss_in_n) || (ssp && (!ss_in_n)))) begin
          n_state = S_ASSERT_SS;
        end else begin
          n_state = S_IDLE;
        end
      end
      S_ASSERT_SS : begin
        n_state = (sclk_fe_int || baud2) ? S_POP_TX : S_ASSERT_SS;
      end
      S_POP_TX : begin
        n_state = (sclk_re_int || baud2) ? S_SHIFT_BIT : S_POP_TX;
      end
      S_SHIFT_BIT : begin
        if (f_done && (!ssp)) begin
          n_state = S_PUSH_RX;
        end else if (ssp && f_done && (!x_done)) begin
          n_state = S_TOGGLE_SS;
        end else if (ssp && f_done && x_done_1d) begin
          n_state = S_PUSH_RX;
        end else begin
          n_state = S_SHIFT_BIT;
        end
      end
      S_PUSH_RX : begin
        if (sclk_fe_int || baud2) begin
          if (spi0 && (!baud2)) begin
            n_state = S_HOLD_MASK;
          end else if ((spi1 && x_done) || (spi0 && baud2 && (x_done || ss_tgl_en)) || ssp) begin
            n_state = S_MASK_SCLK;
          end else if (spi0 && baud2 && (!x_done) && (!ss_tgl_en)) begin
            n_state = S_SHIFT_BIT;
          end else begin
            n_state = S_PUSH_RX;
          end
        end else if (sclk_re_int || baud2) begin
          if ((!x_done) && spi1) begin
            n_state = S_SHIFT_BIT;
          end else begin
            n_state = S_PUSH_RX;
          end
        end else begin
          n_state = S_PUSH_RX;
        end
      end
      S_TOGGLE_SS : begin
        n_state = (sclk_re_int || baud2) ? S_SHIFT_BIT : S_TOGGLE_SS;
      end
      S_HOLD_MASK : begin
        if (sclk_re_int || baud2) begin
          if (ss_tgl_en || ssp || spi1) begin
            n_state = S_MASK_SCLK;
          end else if (x_done || abort_ir) begin
            n_state = S_MASK_SCLK;
          end else begin
            n_state = S_SHIFT_BIT;
          end
        end else begin
          n_state = S_HOLD_MASK;
        end
      end
      S_MASK_SCLK : begin
        if ((sclk_fe_int || baud2) && spi0 && (!x_done) && (!abort_ir)) begin
          n_state = S_TOGGLE_SS;
        end else if (sclk_re_int && (ssp || (spi0 && (x_done || abort_ir)))) begin
          n_state = S_CLEAR_SS;
        end else if ((sclk_re_int || baud2) && (spi1 || ssp)) begin
          n_state = S_HOLD_SS;
        end else begin
          n_state = S_MASK_SCLK;
        end
      end
      S_HOLD_SS : begin
        if ((sclk_fe_int || baud2) && (hold_cnt == (spec_hold_ss - 4'd1))) begin
          n_state = S_CLEAR_SS;
        end else begin
          n_state = S_HOLD_SS;
        end
      end
      S_CLEAR_SS : begin
        n_state = (sclk_re_int || baud2) ? S_IDLE : S_CLEAR_SS;
      end
      S_SLEEP : begin
        // mirror fl_ssi: leaving SLEEP is only possible once SSI re-enabled
        n_state = ssi_en ? S_IDLE : S_SLEEP;
      end
      default : n_state = S_IDLE;
    endcase
  end

  // -- sequential state assignment
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      c_state <= S_IDLE;
    end else begin
      if (!ssi_en) begin
        c_state <= S_SLEEP;
      end else if ((ssp && ss_in_n) || ((!ssp) && (!ss_in_n))) begin
        c_state <= S_IDLE;
      end else begin
        c_state <= n_state;
      end
    end
  end

  // -- sleep / busy
  reg fsm_sleep_ir;
  reg fsm_busy_ir;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      fsm_sleep_ir <= 1'b0;
      fsm_busy_ir  <= 1'b0;
    end else begin
      fsm_sleep_ir <= (n_state == S_SLEEP);
      fsm_busy_ir  <= (n_state != S_IDLE) && (n_state != S_SLEEP);
    end
  end
  assign fsm_sleep = fsm_sleep_ir;
  assign fsm_busy  = fsm_busy_ir;

  // -- serial clock mask
  assign sclk_mask =
         (n_state == S_SHIFT_BIT) ||
         (n_state == S_PUSH_RX)   ||
         (n_state == S_HOLD_MASK) ||
         ((n_state == S_POP_TX) && (spi1 || ssp)) ||
         ((n_state == S_TOGGLE_SS) && ssp);

  // -- TX shift enable
  assign tx_shift_en =
         (c_state == S_SHIFT_BIT) && sclk_re_int &&
         ((tmod[1] == 1'b0) || (tmod == 2'b11));

  // -- RX shift enable
  assign rx_shift_en =
         ((c_state == S_SHIFT_BIT) && sclk_fe_int) ||
         ((c_state == S_PUSH_RX)   && sclk_fe_int) ||
         ((c_state == S_TOGGLE_SS) && sclk_fe_int);

  // -- TX shift register load
  assign load_tx_shift =
         (tx_load_en && (!tx_empty)) ?
         ((c_state == S_POP_TX)   && sclk_re_int) ||
         ((c_state == S_PUSH_RX)  && sclk_re_int && (!x_done) && (ssp ? sclk_mask : 1'b1)) ||
         ((c_state == S_TOGGLE_SS)&& sclk_re_int && ssp) ||
         ((c_state == S_HOLD_MASK)&& sclk_re_int && (!x_done)) : 1'b0;

  // -- TX buffer reload (retransmit)
  assign load_tx_buffer =
         (!tx_load_en) ?
         ((c_state == S_POP_TX)   && sclk_re_int) ||
         ((c_state == S_PUSH_RX)  && sclk_re_int && (!x_done)) ||
         ((c_state == S_TOGGLE_SS)&& sclk_re_int && ssp) ||
         ((c_state == S_HOLD_MASK)&& sclk_re_int && (!x_done)) : 1'b0;

  // -- RX buffer load
  wire rx_load_gate = (tmod == 2'b00) || (tmod == 2'b10) || (tmod == 2'b11);
  assign load_rx_buf =
         rx_load_gate ?
         ((c_state == S_PUSH_RX)    && sclk_re_int) ||
         ((c_state == S_TOGGLE_SS)  && (sclk_re_int || baud2) && ssp) ||
         ((c_state == S_HOLD_MASK)  && (sclk_re_int || baud2)) ||
         ((c_state == S_MASK_SCLK)  && (!spi0) && sclk_re_int) : 1'b0;

  // -- slave select
  assign fsm_slv_sel =
         (spi0 || spi1) ?
         (n_state != S_IDLE) && (n_state != S_CLEAR_SS) && (n_state != S_SLEEP) &&
         (n_state != S_TOGGLE_SS) :
         (ssp) ?
         (n_state == S_ASSERT_SS) || (n_state == S_POP_TX) || (n_state == S_TOGGLE_SS) ||
         ((n_state == S_SHIFT_BIT) && sspf_done && (!x_done) && sclk_re_int) :
         1'b0;

  // -- output enable
  wire ssi_oe_n_iw =
         (spi0 || spi1) ?
         (n_state == S_IDLE) || (n_state == S_CLEAR_SS) || (n_state == S_SLEEP) :
         (ssp) ?
         (n_state == S_IDLE) || (n_state == S_ASSERT_SS) || (n_state == S_CLEAR_SS) ||
         (n_state == S_SLEEP) :
         1'b1;
  reg ssi_oe_n_ir;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ssi_oe_n_ir <= 1'b1;
    end else begin
      ssi_oe_n_ir <= ssi_oe_n_iw;
    end
  end
  assign ssi_oe_n = ssi_oe_n_ir;

  assign tx_dfs = dfs_eff;

  // -- coverage observation
  assign cov_fsm_state    = (11'd1 << c_state);
  assign cov_bit_cnt      = bit_cnt;
  assign cov_frame_cnt    = frame_cnt;
  assign cov_x_done       = x_done;
  assign cov_last_frame   = last_frame;
  assign cov_tx_shift_en  = tx_shift_en;
  assign cov_rx_shift_en  = rx_shift_en;
  assign cov_load_tx_shift= load_tx_shift;
  assign cov_load_rx_buf  = load_rx_buf;
  assign cov_hold_ss_cnt  = hold_cnt;

endmodule // spi_mstfsm
