// File    : spi_m_mstfsm.v
// Abstract: spi_master_public - Master transfer FSM.
//           15-state controller implementing SPI0 / SPI1 / SSP /
//           Microwire master transfers, including the Microwire
//           control-frame handshake (MWPOP / WAIT_READY / CLEAR_READY)
//           and the slave-select hold counter (SPEC_HOLD_SS).
module spi_m_mstfsm
  (
   input         clk,
   input         rst_n,
   input         ssi_en,        // internal SSI enable
   input         start_xfer,
   input         tx_empty,
   input         sclk_re,
   input         sclk_fe,
   input  [2:0]  mwcr,
   input  [1:0]  frf,
   input         scph,
   input         ss_tgl_en,
   input  [1:0]  tmod,
   input  [4:0]  dfs,
   input  [3:0]  cfs,           // effective control frame size
   input  [16:0] ndf,
   input         srl_test,
   input         baud2,
   input         ss_in_n,
   input         rxd,
   input  [3:0]  tx_fifo_level,
   input  [3:0]  txfthr,
   input  [3:0]  spec_hold_ss,

   output reg    fsm_busy,
   output        fsm_slv_sel,
   output        sclk_mask,
   output reg    ssi_oe_n,
   output reg    fsm_sleep,
   output        load_tx_shift,
   output        ssp_load_tx_shift,
   output        load_tx_buffer,
   output        load_rx_buf,
   output        tx_shift_en,
   output [4:0]  tx_dfs,
   output        rx_shift_en,
   output reg    fsm_multi_mst,
   output        load_start_bit,

   // -- coverage
   output [3:0]  cov_fsm_state_id,
   output [14:0] cov_fsm_state,
   output [5:0]  cov_bit_cnt,
   output [3:0]  cov_ctrl_cnt,
   output [16:0] cov_frame_cnt,
   output        cov_f_done,
   output        cov_c_done,
   output        cov_x_done,
   output        cov_last_frame,
   output        cov_tx_shift_en,
   output        cov_rx_shift_en,
   output        cov_load_tx_shift,
   output        cov_load_tx_buffer,
   output        cov_load_rx_buf,
   output        cov_s3,
   output [3:0]  cov_hold_ss_cnt,
   output        cov_spi0_control,
   output        cov_spi1_control,
   output        cov_fsm_busy,
   output        cov_fsm_sleep,
   output        cov_start_bit_ctl,
   output        cov_s0
   );

  // -- state encoding (one-hot)
  localparam [14:0] P_IDLE        = 15'b000_0000_0000_0001;
  localparam [14:0] P_ASSERT_SS   = 15'b000_0000_0000_0010;
  localparam [14:0] P_POP_TX      = 15'b000_0000_0000_0100;
  localparam [14:0] P_SHIFT_CTRL  = 15'b000_0000_0000_1000;
  localparam [14:0] P_SHIFT_BIT   = 15'b000_0000_0001_0000;
  localparam [14:0] P_PUSH_RX     = 15'b000_0000_0010_0000;
  localparam [14:0] P_TOGGLE_SS   = 15'b000_0000_0100_0000;
  localparam [14:0] P_HOLD_MASK   = 15'b000_0000_1000_0000;
  localparam [14:0] P_MASK_SCLK   = 15'b000_0001_0000_0000;
  localparam [14:0] P_HOLD_SS     = 15'b000_0010_0000_0000;
  localparam [14:0] P_CLEAR_SS    = 15'b000_0100_0000_0000;
  localparam [14:0] P_SLEEP       = 15'b000_1000_0000_0000;
  localparam [14:0] P_MWPOP       = 15'b001_0000_0000_0000;
  localparam [14:0] P_WAIT_READY  = 15'b010_0000_0000_0000;
  localparam [14:0] P_CLEAR_READY = 15'b100_0000_0000_0000;

  reg  [14:0] c_state, n_state;
  reg  [5:0]  bit_cnt;
  reg  [3:0]  ctrl_cnt;
  reg  [16:0] frame_cnt;
  reg         last_frame;
  reg         x_done_1d;
  reg         tx_load_en;
  reg         c_done_ir;
  reg         f_done_ir;
  reg         abort_ir;
  reg         abort_seen;
  reg         spi0_control;
  reg         spi1_control;
  reg         rxd_sync_1, rxd_sync_2;
  reg  [3:0]  hold_ss_cnt;

  wire        ssp;
  wire        spi0;
  wire        spi1;
  wire        mwire;
  wire        sclk_re_int;
  wire        sclk_fe_int;
  wire        tx_fifo_filled;
  wire        f_done;
  wire        sspf_done;
  wire        c_done;
  wire        x_done;
  wire        early_last_frame;
  wire        inc_bit_cnt;
  wire        clr_bit_cnt;
  wire        inc_ctrl_cnt;
  wire        clr_ctrl_cnt;
  wire        inc_frame_cnt;
  wire        clr_frame_cnt;
  wire        sclk_mask_int;
  wire        fsm_sleep_iw;
  wire        fsm_busy_iw;
  wire        fsm_multi_mst_iw;
  wire        ssi_oe_n_iw;
  wire        rxd_sync;

  assign ssp   = (frf == 2'b01);
  assign spi0  = (frf == 2'b00 && scph == 1'b0);
  assign spi1  = (frf == 2'b00 && scph == 1'b1);
  assign mwire = (frf == 2'b10);

  // -- edge muxing: SPI0 / Microwire data is sampled on the opposite edge
  assign sclk_re_int = (spi0 || mwire) ? sclk_fe : sclk_re;
  assign sclk_fe_int = (spi0 || mwire) ? sclk_re : sclk_fe;

  // -- bit counter
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      bit_cnt <= 6'b0;
    end else if (clr_bit_cnt & inc_bit_cnt & mwire) begin
      bit_cnt <= 6'b000001;
    end else if (clr_bit_cnt) begin
      bit_cnt <= 6'b0;
    end else if (inc_bit_cnt) begin
      bit_cnt <= bit_cnt + 6'd1;
    end
  end

  // -- control bit counter
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      ctrl_cnt <= 4'b0;
    end else if (clr_ctrl_cnt) begin
      ctrl_cnt <= 4'b0;
    end else if (inc_ctrl_cnt) begin
      ctrl_cnt <= ctrl_cnt + 4'd1;
    end
  end

  // -- data frame done
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      f_done_ir <= 1'b0;
    end else begin
      f_done_ir <= (bit_cnt == {1'b0, dfs} + 6'd1);
    end
  end
  assign f_done = (frf == 2'b10 && (baud2 || mwcr[1:0] == 2'b11)) ? (bit_cnt == {1'b0, dfs} + 6'd1) :
                  (frf == 2'b10 && mwcr[1:0] == 2'b10) ? f_done_ir :
                  (bit_cnt == {1'b0, dfs});
  assign sspf_done = (bit_cnt == {1'b0, dfs} - 6'd1);

  // -- control frame done
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      c_done_ir <= 1'b0;
    end else if (ctrl_cnt[3:0] == cfs && c_state == P_SHIFT_CTRL) begin
      c_done_ir <= 1'b1;
    end else if (c_state != P_SHIFT_CTRL) begin
      c_done_ir <= 1'b0;
    end
  end
  assign c_done = (mwire && ctrl_cnt[3:0] == cfs) || c_done_ir;

  // -- frame counter
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      frame_cnt <= 17'b0;
    end else if (clr_frame_cnt) begin
      frame_cnt <= 17'b0;
    end else if (inc_frame_cnt) begin
      frame_cnt <= frame_cnt + 17'd1;
    end
  end

  // -- SPI control flags
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      spi0_control <= 1'b0;
      spi1_control <= 1'b0;
    end else begin
      if (tx_empty == 1'b0) begin
        spi1_control <= 1'b1;
      end else if (clr_bit_cnt) begin
        spi1_control <= 1'b0;
      end
      if (tx_empty == 1'b0) begin
        spi0_control <= 1'b1;
      end else if ((c_state == P_PUSH_RX || c_state == P_HOLD_MASK) && sclk_re_int) begin
        spi0_control <= 1'b0;
      end
    end
  end

  // -- last frame
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      last_frame <= 1'b0;
    end else if (tx_empty && (sclk_fe_int || baud2) && c_state != P_MASK_SCLK) begin
      last_frame <= 1'b1;
    end else if ((~tx_empty) && (c_state == P_SHIFT_CTRL ||
                                 c_state == P_SHIFT_BIT ||
                                 c_state == P_IDLE)) begin
      last_frame <= 1'b0;
    end
  end
  assign early_last_frame = last_frame;

  assign x_done = ((tmod == 2'b11 && frf == 2'b00) ||
                   (tmod == 2'b10 && (~mwire)) ||
                   (mwire && mwcr[0] && (~mwcr[1]))) ?
                  (frame_cnt == ndf) : early_last_frame;

  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      x_done_1d <= 1'b1;
    end else begin
      x_done_1d <= x_done;
    end
  end

  // -- TX load enable
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      tx_load_en <= 1'b1;
    end else begin
      case (c_state)
        P_POP_TX: begin
          if (sclk_re_int || baud2) begin
            if ((~mwire) && tmod == 2'b10) begin
              tx_load_en <= 1'b0;
            end else begin
              tx_load_en <= 1'b1;
            end
          end
        end
        P_IDLE: tx_load_en <= 1'b1;
        default: tx_load_en <= tx_load_en;
      endcase
    end
  end

  assign tx_dfs = (n_state == P_SHIFT_CTRL) ? {1'b0, cfs} : dfs;

  // -- state sequential
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      c_state <= P_IDLE;
    end else if (ssi_en == 1'b0) begin
      c_state <= P_SLEEP;
    end else if ((ssp && ss_in_n == 1'b1) || ((~ssp) && ss_in_n == 1'b0)) begin
      c_state <= P_IDLE;
    end else begin
      c_state <= n_state;
    end
  end

  // -- abort transfer
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      abort_ir <= 1'b0;
    end else if (c_state == P_HOLD_MASK && x_done == 1'b1) begin
      abort_ir <= 1'b1;
    end else if (c_state == P_IDLE || c_state == P_CLEAR_SS) begin
      abort_ir <= 1'b0;
    end
  end

  // -- sticky "abort seen" flag: set the cycle a wrong-polarity ss_in_n
  //    forces the FSM back to idle mid-transfer, cleared only when a later
  //    transfer runs all the way out through P_CLEAR_SS (so the recovery's
  //    HOLD_SS dwell still observes it).  Sampled via cov_s0.
  wire        wrong_pol = ((ssp && ss_in_n == 1'b1) ||
                           ((~ssp) && ss_in_n == 1'b0));
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      abort_seen <= 1'b0;
    end else if (wrong_pol && (c_state != P_IDLE) && (c_state != P_SLEEP)) begin
      abort_seen <= 1'b1;
    end else if (c_state == P_CLEAR_SS) begin
      abort_seen <= 1'b0;
    end
  end

  assign tx_fifo_filled = (tmod == 2'b10) ? 1'b1 :
                          (tx_fifo_level > {1'b0, txfthr[2:0]});

  // -- next state logic
  always @(*) begin
    case (c_state)
      P_IDLE: begin
        if (start_xfer && sclk_re_int && tx_fifo_filled && (~tx_empty) &&
            ((ssp && ss_in_n == 1'b0) || ((~ssp) && ss_in_n == 1'b1)))
          n_state = P_ASSERT_SS;
        else
          n_state = P_IDLE;
      end
      P_ASSERT_SS:
        n_state = (sclk_fe_int || baud2) ? P_POP_TX : P_ASSERT_SS;
      P_POP_TX:
        if (sclk_re_int || baud2)
          n_state = mwire ? P_SHIFT_CTRL : P_SHIFT_BIT;
        else
          n_state = P_POP_TX;
      P_MWPOP:
        if (baud2 & tx_empty)
          n_state = P_HOLD_MASK;
        else if ((sclk_re_int || baud2) && (!f_done || mwcr[0]))
          n_state = P_SHIFT_BIT;
        else if ((sclk_re_int || baud2) && f_done)
          n_state = P_SHIFT_CTRL;
        else
          n_state = P_MWPOP;
      P_SHIFT_CTRL:
        if (c_done && (~mwcr[1]) && (sclk_re_int || baud2))
          n_state = P_TOGGLE_SS;
        else if (c_done && mwcr[1] && (~tx_empty) && (sclk_fe_int || baud2))
          n_state = P_MWPOP;
        else if (c_done && mwcr[1] && tx_empty && (sclk_fe_int || baud2))
          n_state = P_HOLD_MASK;
        else
          n_state = P_SHIFT_CTRL;
      P_SHIFT_BIT:
        if ((f_done && (~ssp) && (~mwire)) ||
            (f_done && mwire && (~mwcr[1])) ||
            (f_done && x_done_1d && (~mwire) && ssp) ||
            (f_done && x_done && (~mwire) && (~ssp)))
          n_state = P_PUSH_RX;
        else if (f_done && mwire && mwcr[1] && (~mwcr[2]) && (~x_done))
          n_state = P_MWPOP;
        else if (f_done && mwire && mwcr[1] && mwcr[2] && (~x_done))
          n_state = P_HOLD_MASK;
        else if (x_done && mwire && mwcr[1] && f_done)
          n_state = P_HOLD_MASK;
        else if (ssp && f_done && (~x_done))
          n_state = P_TOGGLE_SS;
        else
          n_state = P_SHIFT_BIT;
      P_PUSH_RX:
        if ((sclk_fe_int || baud2) && (spi0 && (~baud2)))
          n_state = P_HOLD_MASK;
        else if ((sclk_fe_int || baud2) && ((spi1 && x_done) ||
                 (spi0 && baud2 && (x_done || ss_tgl_en)) || ssp))
          n_state = P_MASK_SCLK;
        else if ((sclk_re_int || baud2) && (mwire && x_done))
          n_state = P_MASK_SCLK;
        else if ((sclk_re_int || baud2) && (~x_done && spi1))
          n_state = P_SHIFT_BIT;
        else if ((sclk_re_int || baud2) && (~x_done) && mwire && (~mwcr[0]))
          n_state = P_SHIFT_CTRL;
        else if (((sclk_re_int || baud2) && (~x_done) && mwire && mwcr[0]) ||
                 ((sclk_fe_int || baud2) && spi0 && baud2 && (~x_done) && (~ss_tgl_en)))
          n_state = P_SHIFT_BIT;
        else
          n_state = P_PUSH_RX;
      P_TOGGLE_SS:
        if ((mwire && baud2 && sclk_fe_int) ||
            (mwire && (~baud2) && sclk_re_int) ||
            ((~mwire) && (sclk_re_int || baud2)))
          n_state = P_SHIFT_BIT;
        else
          n_state = P_TOGGLE_SS;
      P_HOLD_MASK:
        if (sclk_re_int || baud2) begin
          if (ss_tgl_en)
            n_state = P_MASK_SCLK;
          else if (ssp || mwire || spi1)
            n_state = P_MASK_SCLK;
          else begin
            if (spi0 && (x_done || abort_ir))
              n_state = P_MASK_SCLK;
            else
              n_state = P_SHIFT_BIT;
          end
        end else begin
          n_state = P_HOLD_MASK;
        end
      P_MASK_SCLK:
        if ((sclk_fe_int || baud2) && spi0 && (~x_done) && (~abort_ir))
          n_state = P_TOGGLE_SS;
        else if ((sclk_fe_int || baud2) && mwire)
          n_state = P_CLEAR_SS;
        else if (sclk_re_int && (ssp || (spi0 && (x_done || abort_ir))))
          n_state = P_CLEAR_SS;
        else if ((sclk_re_int || baud2) && (spi1 || ssp))
          n_state = P_HOLD_SS;
        else
          n_state = P_MASK_SCLK;
      P_HOLD_SS:
        if (sclk_fe_int || baud2) begin
          n_state = (hold_ss_cnt >= (spec_hold_ss - 4'd1)) ? P_CLEAR_SS : P_HOLD_SS;
        end else begin
          n_state = P_HOLD_SS;
        end
      P_CLEAR_SS:
        if (mwire && mwcr[2] && (sclk_re_int || baud2))
          n_state = P_WAIT_READY;
        else if (sclk_re_int || baud2)
          n_state = P_IDLE;
        else
          n_state = P_CLEAR_SS;
      P_WAIT_READY:
        if (rxd_sync == 1'b1 && (~mwcr[0]) && (~tx_empty) && sclk_fe_int)
          n_state = P_MWPOP;
        else if (rxd_sync == 1'b1 && (mwcr[0] || tx_empty) && sclk_re_int)
          n_state = P_CLEAR_READY;
        else
          n_state = P_WAIT_READY;
      P_CLEAR_READY:
        if (((sclk_fe_int && baud2) || ((~baud2) && sclk_re_int)) &&
            (~tx_empty) && mwcr[0])
          n_state = P_MWPOP;
        else if (sclk_re_int && ~(mwcr[0] && (~tx_empty)))
          n_state = P_IDLE;
        else
          n_state = P_CLEAR_READY;
      default:
        n_state = ssi_en ? P_IDLE : P_SLEEP;
    endcase
  end

  // -- HOLD_SS counter
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      hold_ss_cnt <= 4'b0;
    end else if (c_state == P_HOLD_SS) begin
      if (sclk_fe_int || baud2) begin
        hold_ss_cnt <= hold_ss_cnt + 4'd1;
      end
    end else begin
      hold_ss_cnt <= 4'b0;
    end
  end

  // -- sleep / busy
  assign fsm_sleep_iw = (n_state == P_SLEEP);
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      fsm_sleep <= 1'b0;
    end else begin
      fsm_sleep <= fsm_sleep_iw;
    end
  end

  assign fsm_busy_iw = (n_state != P_IDLE) && (n_state != P_SLEEP);
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      fsm_busy <= 1'b0;
    end else begin
      fsm_busy <= fsm_busy_iw;
    end
  end

  // -- shift enables
  assign tx_shift_en =
      (c_state == P_SHIFT_BIT && sclk_re_int &&
       (mwire || tmod[1] == 1'b0 || (tmod == 2'b11 && (spi0_control || spi1_control)))) ||
      (c_state == P_SHIFT_CTRL && sclk_re_int);

  assign rx_shift_en =
      (mwire && mwcr[1] && (~srl_test)) ? 1'b0 :
      (c_state == P_SHIFT_CTRL && (sclk_fe_int || baud2) && mwcr[1]) ||
      (c_state == P_SHIFT_BIT && sclk_fe_int) ||
      (c_state == P_PUSH_RX && sclk_fe_int) ||
      (c_state == P_TOGGLE_SS && sclk_fe_int && (~mwire));

  // -- TX load enable
  assign load_tx_shift =
      (tx_load_en == 1'b1 && tx_empty == 1'b0) ?
      (c_state == P_POP_TX && sclk_re_int) ||
      (c_state == P_PUSH_RX && sclk_re_int && (~x_done)) ||
      (c_state == P_TOGGLE_SS && sclk_re_int && ssp) ||
      (c_state == P_MWPOP && (sclk_re_int || baud2)) ||
      (c_state == P_HOLD_MASK && sclk_re_int && (~mwire) && (~x_done) && (~abort_ir))
      : 1'b0;

  // SSP baud2 early pop (synchronous clocks: disabled)
  assign ssp_load_tx_shift = 1'b0;

  assign load_tx_buffer =
      (tx_load_en == 1'b0) ?
      (c_state == P_POP_TX && sclk_re_int) ||
      (c_state == P_PUSH_RX && sclk_re_int && (~x_done)) ||
      (c_state == P_TOGGLE_SS && sclk_re_int && ssp) ||
      (c_state == P_HOLD_MASK && sclk_re_int && (~mwire) && (~x_done))
      : 1'b0;

  assign load_rx_buf =
      (tmod == 2'b10 || tmod == 2'b00 || mwire ||
       (tmod == 2'b11 && ((scph == 1'b1 && spi1_control == 1'b0) ||
                          (scph == 1'b0 && spi0_control == 1'b0)))) ?
      (c_state == P_MWPOP && (sclk_re_int || baud2) && srl_test) ||
      (c_state == P_PUSH_RX && sclk_re_int) ||
      (c_state == P_TOGGLE_SS && (sclk_re_int || baud2) && (ssp || (mwire && srl_test))) ||
      (c_state == P_HOLD_MASK && (sclk_re_int || baud2) &&
       ((~mwire) || (mwire && srl_test))) ||
      (c_state == P_MASK_SCLK && (~spi0) && sclk_re_int && (~mwire))
      : 1'b0;

  // -- slave select
  assign fsm_slv_sel =
      (spi0 || spi1) ?
      (n_state != P_IDLE && n_state != P_CLEAR_SS &&
       n_state != P_SLEEP && n_state != P_TOGGLE_SS) :
      (ssp == 1'b1) ?
      (n_state == P_ASSERT_SS || n_state == P_POP_TX ||
       n_state == P_TOGGLE_SS ||
       (n_state == P_SHIFT_BIT && sspf_done == 1'b1 &&
        x_done == 1'b0 && sclk_re_int)) :
      (n_state != P_IDLE && n_state != P_CLEAR_SS && n_state != P_SLEEP);

  // -- output enable
  assign ssi_oe_n_iw =
      (spi0 || spi1) ?
      (n_state == P_IDLE || n_state == P_CLEAR_SS || n_state == P_SLEEP) :
      (ssp == 1'b1) ?
      (n_state == P_IDLE || n_state == P_ASSERT_SS ||
       n_state == P_CLEAR_SS || n_state == P_SLEEP) :
      (n_state == P_IDLE || n_state == P_SLEEP ||
       (n_state == P_SHIFT_BIT && (~mwcr[1])) ||
       ((~baud2) && sclk_fe_int && n_state == P_TOGGLE_SS) ||
       (ssi_oe_n && n_state == P_TOGGLE_SS) ||
       (n_state == P_MASK_SCLK && (~mwcr[1])) ||
       (n_state == P_HOLD_MASK && (~mwcr[1])) ||
       n_state == P_CLEAR_SS || n_state == P_PUSH_RX);

  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      ssi_oe_n <= 1'b1;
    end else begin
      ssi_oe_n <= ssi_oe_n_iw;
    end
  end

  // -- multi-master detection (toggle)
  assign fsm_multi_mst_iw =
      (n_state != P_IDLE && n_state != P_SLEEP) &&
      ((ss_in_n == 1'b1 && ssp == 1'b1) || (ss_in_n == 1'b0 && ssp == 1'b0));
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      fsm_multi_mst <= 1'b0;
    end else if (fsm_multi_mst_iw) begin
      fsm_multi_mst <= ~fsm_multi_mst;
    end
  end

  // -- clock mask
  assign sclk_mask_int =
      (n_state == P_SHIFT_BIT) ||
      (n_state == P_PUSH_RX) ||
      (n_state == P_MWPOP && c_state != P_WAIT_READY) ||
      (c_state == P_MWPOP && n_state == P_HOLD_MASK) ||
      (n_state == P_HOLD_MASK) ||
      (n_state == P_SHIFT_CTRL) ||
      (n_state == P_POP_TX && (spi1 || ssp)) ||
      (n_state == P_CLEAR_READY && tx_empty) ||
      (n_state == P_TOGGLE_SS && (ssp || mwire));
  assign sclk_mask = sclk_mask_int;

  // -- counter controls
  assign inc_frame_cnt =
      (c_state == P_SHIFT_BIT && bit_cnt == {1'b0, dfs} - 6'd2) &&
      ((tmod != 2'b11) || (tmod == 2'b11 &&
       ((scph == 1'b0 && spi0_control == 1'b0) ||
        (scph == 1'b1 && spi1_control == 1'b0)))) &&
      sclk_re_int;

  assign clr_frame_cnt =
      (c_state == P_CLEAR_SS && n_state != P_CLEAR_SS) ||
      (c_state == P_IDLE);

  assign inc_bit_cnt =
      (c_state == P_MWPOP && sclk_re_int) ||
      (c_state == P_PUSH_RX && baud2 && mwire) ||
      (c_state == P_SHIFT_BIT && sclk_re_int);

  assign clr_bit_cnt =
      (c_state == P_TOGGLE_SS && sclk_re_int && (ssp || mwire || ss_tgl_en)) ||
      (c_state == P_HOLD_MASK && sclk_re_int && (~ssp) && (~mwire) && (~ss_tgl_en)) ||
      (c_state == P_PUSH_RX && sclk_re_int) ||
      (c_state == P_IDLE && sclk_re_int) ||
      (c_state == P_SHIFT_CTRL && sclk_fe_int) ||
      (c_state == P_MWPOP && mwcr[0] && (sclk_fe_int || baud2)) ||
      (c_state == P_TOGGLE_SS && (sclk_fe_int || baud2) && mwire);

  assign inc_ctrl_cnt = (c_state == P_SHIFT_CTRL && sclk_re_int);

  assign clr_ctrl_cnt =
      (c_state == P_SHIFT_BIT && sclk_re_int) ||
      (c_state == P_PUSH_RX && sclk_re_int) ||
      (c_state == P_IDLE && sclk_re_int) ||
      (c_state == P_TOGGLE_SS && sclk_fe_int);

  assign load_start_bit = (c_state == P_WAIT_READY && sclk_fe_int && (tx_empty || mwcr[0]));

  // -- rxd synchronizer (2-stage)
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      rxd_sync_1 <= 1'b0;
      rxd_sync_2 <= 1'b0;
    end else begin
      rxd_sync_1 <= rxd;
      rxd_sync_2 <= rxd_sync_1;
    end
  end
  assign rxd_sync = rxd_sync_2;

  // -- coverage outputs
  assign cov_fsm_state    = c_state;
  assign cov_fsm_state_id = (c_state == P_ASSERT_SS)  ? 4'd1 :
                            (c_state == P_POP_TX)     ? 4'd2 :
                            (c_state == P_SHIFT_CTRL) ? 4'd3 :
                            (c_state == P_SHIFT_BIT)  ? 4'd4 :
                            (c_state == P_PUSH_RX)    ? 4'd5 :
                            (c_state == P_TOGGLE_SS)  ? 4'd6 :
                            (c_state == P_HOLD_MASK)  ? 4'd7 :
                            (c_state == P_MASK_SCLK)  ? 4'd8 :
                            (c_state == P_HOLD_SS)    ? 4'd9 :
                            (c_state == P_CLEAR_SS)   ? 4'd10 :
                            (c_state == P_SLEEP)      ? 4'd11 :
                            (c_state == P_MWPOP)      ? 4'd12 :
                            (c_state == P_WAIT_READY) ? 4'd13 :
                            (c_state == P_CLEAR_READY)? 4'd14 : 4'd0;
  assign cov_bit_cnt       = bit_cnt;
  assign cov_ctrl_cnt      = ctrl_cnt;
  assign cov_frame_cnt     = frame_cnt;
  assign cov_f_done        = f_done;
  assign cov_c_done        = c_done;
  assign cov_x_done        = x_done;
  assign cov_last_frame    = last_frame;
  assign cov_tx_shift_en   = tx_shift_en;
  assign cov_rx_shift_en   = rx_shift_en;
  assign cov_load_tx_shift = load_tx_shift;
  assign cov_load_tx_buffer= load_tx_buffer;
  assign cov_load_rx_buf   = load_rx_buf;
  assign cov_s3            = load_start_bit;
  assign cov_hold_ss_cnt   = hold_ss_cnt;
  assign cov_spi0_control  = spi0_control;
  assign cov_spi1_control  = spi1_control;
  assign cov_fsm_busy      = fsm_busy;
  assign cov_fsm_sleep     = fsm_sleep;
  assign cov_start_bit_ctl = load_start_bit;
  assign cov_s0            = abort_seen;

endmodule // spi_m_mstfsm
