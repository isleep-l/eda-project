// File    : spi_xfer_top.v
// Abstract: spi_xfer_public - SPI master serial transfer controller (top).
//           SPI0 / SPI1 / SSP master with TX/RX FIFOs, baud-rate
//           divider, 11-state transfer FSM and shift registers.
//
//           The DUT is programmed over a simple word-addressed register
//           bus.  A transfer starts automatically once SSIENR=1, at
//           least one slave is enabled (SER!=0) and the TX FIFO holds
//           more than TXFTLR words.
module spi_xfer_top
  #(
   parameter DFS_W          = 5,   // data frame size width (4..32)
   parameter NDF_W          = 17,  // number of data frames
   parameter BAUDR_W        = 16,  // baud divisor width
   parameter NUM_SLAVES     = 4,   // slave select lines
   parameter TX_DEPTH       = 8,   // TX FIFO depth
   parameter RX_DEPTH       = 8    // RX FIFO depth
   )
  (
   // -- clock / reset
   input  wire        clk,
   input  wire        rst_n,

   // -- register bus (word-addressed)
   input  wire        reg_we,
   input  wire [3:0]  reg_addr,
   input  wire [31:0] reg_wdata,
   input  wire        reg_re,
   output wire [31:0] reg_rdata,

   // -- serial interface
   input  wire        rxd,        // receive data
   input  wire        ss_in_n,    // slave select input
   output wire        sclk_out,   // serial clock
   output wire        txd,        // transmit data
   output wire [NUM_SLAVES-1:0] ss_n,
   output wire        ssi_busy,

   // -- hidden configuration (injected by the evaluation platform)
   input  wire [3:0]  SPEC_DFS_MIN,   // hidden: minimum clamped frame size
   input  wire [3:0]  SPEC_HOLD_SS,   // hidden: HOLD_SS duration (half periods)

   // -- coverage observation signals
   output wire [10:0] cov_fsm_state /* verilator public */,
   output wire [3:0]  cov_fsm_state_id /* verilator public */,
   output wire        cov_sclk_re /* verilator public */,
   output wire        cov_sclk_fe /* verilator public */,
   output wire        cov_sclk_out /* verilator public */,
   output wire        cov_sclk_mask /* verilator public */,
   output wire [5:0]  cov_bit_cnt /* verilator public */,
   output wire [16:0] cov_frame_cnt /* verilator public */,
   output wire        cov_x_done /* verilator public */,
   output wire        cov_last_frame /* verilator public */,
   output wire        cov_tx_shift_en /* verilator public */,
   output wire        cov_rx_shift_en /* verilator public */,
   output wire        cov_load_tx_shift /* verilator public */,
   output wire        cov_load_rx_buf /* verilator public */,
   output wire        cov_ss_active /* verilator public */,
   output wire        cov_xfer_complete /* verilator public */,
   output wire        cov_txd /* verilator public */,
   output wire [4:0]  cov_dfs /* verilator public */,
   output wire [5:0]  cov_eff_dfs /* verilator public */,
   output wire [15:0] cov_baudr /* verilator public */,
   output wire        cov_baud2 /* verilator public */,
   output wire [16:0] cov_ndf /* verilator public */,
   output wire [1:0]  cov_tmod /* verilator public */,
   output wire [1:0]  cov_frf /* verilator public */,
   output wire        cov_scph /* verilator public */,
   output wire        cov_scpol /* verilator public */,
   output wire        cov_srl_test /* verilator public */,
   output wire        cov_ss_tgl_en /* verilator public */,
   output wire [3:0]  cov_ser /* verilator public */,
   output wire [3:0]  cov_tx_level /* verilator public */,
   output wire [3:0]  cov_rx_level /* verilator public */,
   output wire        cov_tx_empty /* verilator public */,
   output wire        cov_tx_full /* verilator public */,
   output wire        cov_rx_empty /* verilator public */,
   output wire        cov_rx_full /* verilator public */,
   output wire [3:0]  cov_hold_ss_cnt /* verilator public */,
   output wire        cov_s0 /* verilator public */,
   output wire        cov_s1 /* verilator public */,
   output wire        cov_s2 /* verilator public */,
   output wire [31:0] cov_rx_data /* verilator public */,
   output wire        cov_ss_in_n /* verilator public */,
   output wire        cov_start_xfer
   );

  // -- internal nets
  wire        sclk_re;
  wire        sclk_fe;
  wire        sclk_mask;
  wire        fsm_slv_sel;
  wire        load_tx_shift;
  wire        load_tx_buffer;
  wire        load_rx_buf;
  wire        tx_shift_en;
  wire        rx_shift_en;
  wire [5:0]  tx_dfs;
  wire        ssi_oe_n;
  wire        fsm_busy;
  wire        fsm_sleep;
  wire        tx_pop;
  wire        rx_push;
  wire        tx_push;
  wire        rx_pop;
  wire        tx_full;
  wire        tx_empty;
  wire        rx_full;
  wire        rx_empty;
  wire [3:0]  tx_level;
  wire [3:0]  rx_level;
  wire [31:0] tx_pop_data;
  wire [31:0] rx_pop_data;
  wire [31:0] rx_push_data;
  wire [3:0]  fsm_c_state;
  wire        start_xfer;
  wire        baud2;

  // -- register file
  wire [31:0] ctrlr0;
  wire [31:0] ctrlr1;
  wire        ssienr;
  wire [3:0]  ser;
  wire [15:0] baudr;
  wire [3:0]  txftlr;

  // -- decoded config
  wire [4:0]  dfs    = ctrlr0[4:0];
  wire [1:0]  frf    = ctrlr0[6:5];
  wire        scph   = ctrlr0[7];
  wire        scpol  = ctrlr0[8];
  wire [1:0]  tmod   = ctrlr0[10:9];
  wire        srl_test = ctrlr0[11];
  wire        ss_tgl_en = ctrlr0[12];
  wire [16:0] ndf    = ctrlr1[16:0];

  // -- effective frame size (DFS is encoded frame-size-minus-1) with hidden
  //    minimum frame-size clamp (SPEC_DFS_MIN is a frame size, so its raw
  //    value is compared against the decoded frame size dfs+1)
  wire [5:0]  dfs_dec = {1'b0, dfs} + 6'd1;
  wire [5:0]  dfs_eff = (dfs_dec < {2'b00, SPEC_DFS_MIN}) ?
                        {2'b00, SPEC_DFS_MIN} : dfs_dec;

  assign baud2 = (baudr == 16'd2);

  // -- register file
  spi_regfile u_regfile (
    .clk        (clk),
    .rst_n      (rst_n),
    .reg_we     (reg_we),
    .reg_addr   (reg_addr),
    .reg_wdata  (reg_wdata),
    .reg_re     (reg_re),
    .reg_rdata  (reg_rdata),
    .ctrlr0     (ctrlr0),
    .ctrlr1     (ctrlr1),
    .ssienr     (ssienr),
    .ser        (ser),
    .baudr      (baudr),
    .txftlr     (txftlr),
    .fsm_busy   (fsm_busy),
    .tx_full    (tx_full),
    .tx_empty   (tx_empty),
    .tx_level   (tx_level),
    .rx_full    (rx_full),
    .rx_empty   (rx_empty),
    .rx_level   (rx_level),
    .rx_pop_data(rx_pop_data),
    .tx_pop     (tx_pop),
    .tx_push    (tx_push),
    .rx_pop     (rx_pop),
    .start_xfer (start_xfer)
  );

  // -- TX / RX FIFO
  spi_fifo u_fifo (
    .clk         (clk),
    .rst_n       (rst_n),
    .tx_push     (tx_push),
    .tx_push_data(reg_wdata),
    .tx_pop      (tx_pop),
    .rx_push     (rx_push),
    .rx_push_data(rx_push_data),
    .rx_pop      (rx_pop),
    .tx_full     (tx_full),
    .tx_empty    (tx_empty),
    .rx_full     (rx_full),
    .rx_empty    (rx_empty),
    .tx_level    (tx_level),
    .rx_level    (rx_level),
    .tx_pop_data (tx_pop_data),
    .rx_pop_data (rx_pop_data)
  );

  // -- serial clock generator
  spi_sclkgen u_sclkgen (
    .clk        (clk),
    .rst_n      (rst_n),
    .ssi_en     (ssienr),
    .baudr      (baudr),
    .sclk_mask  (sclk_mask),
    .sclk_active(scpol),
    .sclk_out   (sclk_out),
    .sclk_re    (sclk_re),
    .sclk_fe    (sclk_fe),
    .ssi_cnt_out()
  );

  // -- master transfer FSM
  spi_mstfsm u_mstfsm (
    .clk           (clk),
    .rst_n         (rst_n),
    .ssi_en        (ssienr),
    .start_xfer    (start_xfer),
    .tx_empty      (tx_empty),
    .tx_fifo_level (tx_level),
    .txfthr        (txftlr),
    .sclk_re       (sclk_re),
    .sclk_fe       (sclk_fe),
    .frf           (frf),
    .scph          (scph),
    .tmod          (tmod),
    .dfs           (dfs_eff),
    .ndf           (ndf),
    .ss_tgl_en     (ss_tgl_en),
    .ss_in_n       (ss_in_n),
    .baud2         (baud2),
    .spec_hold_ss  (SPEC_HOLD_SS),
    .c_state       (fsm_c_state),
    .fsm_busy      (fsm_busy),
    .fsm_sleep     (fsm_sleep),
    .sclk_mask     (sclk_mask),
    .fsm_slv_sel   (fsm_slv_sel),
    .load_tx_shift (load_tx_shift),
    .load_tx_buffer(load_tx_buffer),
    .load_rx_buf   (load_rx_buf),
    .tx_shift_en   (tx_shift_en),
    .rx_shift_en   (rx_shift_en),
    .tx_dfs        (tx_dfs),
    .ssi_oe_n      (ssi_oe_n),
    .cov_fsm_state (cov_fsm_state),
    .cov_bit_cnt   (cov_bit_cnt),
    .cov_frame_cnt (cov_frame_cnt),
    .cov_x_done    (cov_x_done),
    .cov_last_frame(cov_last_frame),
    .cov_tx_shift_en(cov_tx_shift_en),
    .cov_rx_shift_en(cov_rx_shift_en),
    .cov_load_tx_shift(cov_load_tx_shift),
    .cov_load_rx_buf(cov_load_rx_buf),
    .cov_hold_ss_cnt(cov_hold_ss_cnt),
    .cov_s0        (cov_s0),
    .cov_s1        (cov_s1)
  );

  // -- shift controller
  spi_shift u_shift (
    .clk           (clk),
    .rst_n         (rst_n),
    .tx_shift_en   (tx_shift_en),
    .load_tx_shift (load_tx_shift),
    .load_tx_buffer(load_tx_buffer),
    .rx_shift_en   (rx_shift_en),
    .load_rx_buf   (load_rx_buf),
    .tx_pop_data   (tx_pop_data),
    .rxd           (rxd),
    .dfs           (dfs_eff),
    .fsm_slv_sel   (fsm_slv_sel),
    .ser           (ser),
    .frf           (frf),
    .srl_test      (srl_test),
    .ssi_en        (ssienr),
    .txd           (txd),
    .tx_pop        (tx_pop),
    .rx_push       (rx_push),
    .ss_n          (ss_n),
    .rx_push_data  (rx_push_data),
    .cov_txd       (cov_txd),
    .cov_rx_buffer ()
  );

  assign ssi_busy = fsm_busy;

  // -- transfer completion pulse
  reg [3:0] prev_fsm_state;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      prev_fsm_state <= 4'd0;
    end else begin
      prev_fsm_state <= fsm_c_state;
    end
  end
  assign cov_xfer_complete = (fsm_c_state == 4'd0) && (prev_fsm_state != 4'd0) && (prev_fsm_state != 4'd10);

  // -- sticky coverage flag: a DR write (0x9) landed while a transfer was in
  //    progress.  Cleared when the transfer completes.
  reg write_active_r;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      write_active_r <= 1'b0;
    end else if (cov_xfer_complete) begin
      write_active_r <= 1'b0;
    end else if (reg_we && (reg_addr == 4'h9) && fsm_busy) begin
      write_active_r <= 1'b1;
    end
  end
  assign cov_s2 = write_active_r;

  // -- coverage observation
  assign cov_fsm_state_id = fsm_c_state;
  assign cov_sclk_re      = sclk_re;
  assign cov_sclk_fe      = sclk_fe;
  assign cov_sclk_out     = sclk_out;
  assign cov_sclk_mask    = sclk_mask;
  assign cov_ss_active    = |(~ss_n);
  assign cov_dfs          = dfs;
  assign cov_eff_dfs      = dfs_eff;
  assign cov_baudr        = baudr;
  assign cov_baud2        = baud2;
  assign cov_ndf          = ndf;
  assign cov_tmod         = tmod;
  assign cov_frf          = frf;
  assign cov_scph         = scph;
  assign cov_scpol        = scpol;
  assign cov_srl_test     = srl_test;
  assign cov_ss_tgl_en    = ss_tgl_en;
  assign cov_ser          = ser;
  assign cov_tx_level     = tx_level;
  assign cov_rx_level     = rx_level;
  assign cov_tx_empty     = tx_empty;
  assign cov_tx_full      = tx_full;
  assign cov_rx_empty     = rx_empty;
  assign cov_rx_full      = rx_full;
  assign cov_rx_data      = rx_pop_data;
  assign cov_ss_in_n      = ss_in_n;
  assign cov_start_xfer   = start_xfer;

endmodule // spi_xfer_top
