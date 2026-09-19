// File    : spi_m_top.v
// Abstract: spi_master_public - SPI / SSP / Microwire master (top).
//           Full-featured master: 15-state FSM with Microwire control
//           frames and handshake, 6 interrupts, TX/RX FIFOs, baud
//           divider and hidden configuration (SPEC_CFS_MIN /
//           SPEC_HOLD_SS / SPEC_TXFTLR_DFLT).
module spi_m_top
  (
   input  wire        clk,
   input  wire        rst_n,

   // -- register bus (word-addressed)
   input  wire        reg_we,
   input  wire [5:0]  reg_addr,
   input  wire [31:0] reg_wdata,
   input  wire        reg_re,
   output wire [31:0] reg_rdata,

   // -- serial interface
   input  wire        rxd,
   input  wire        ss_in_n,
   output wire        sclk_out,
   output wire        txd,
   output wire [3:0]  ss_n,
   output wire        ssi_busy,

   // -- interrupt outputs (active low)
   output wire        ssi_txe_intr,
   output wire        ssi_txo_intr,
   output wire        ssi_rxu_intr,
   output wire        ssi_rxf_intr,
   output wire        ssi_rxo_intr,
   output wire        ssi_mst_intr,

   // -- hidden configuration (injected by the evaluation platform)
   input  wire [3:0]  SPEC_CFS_MIN,     // hidden: minimum control frame size
   input  wire [3:0]  SPEC_HOLD_SS,     // hidden: HOLD_SS duration (half periods)
   input  wire [3:0]  SPEC_TXFTLR_DFLT, // hidden: TXFTLR reset value

   // -- coverage observation signals
   output wire [3:0]  cov_fsm_state_id /* verilator public */,
   output wire [14:0] cov_fsm_state /* verilator public */,
   output wire        cov_sclk_re /* verilator public */,
   output wire        cov_sclk_fe /* verilator public */,
   output wire        cov_sclk_out /* verilator public */,
   output wire        cov_sclk_mask /* verilator public */,
   output wire [5:0]  cov_bit_cnt /* verilator public */,
   output wire [3:0]  cov_ctrl_cnt /* verilator public */,
   output wire [16:0] cov_frame_cnt /* verilator public */,
   output wire        cov_f_done /* verilator public */,
   output wire        cov_c_done /* verilator public */,
   output wire        cov_x_done /* verilator public */,
   output wire        cov_last_frame /* verilator public */,
   output wire        cov_tx_shift_en /* verilator public */,
   output wire        cov_rx_shift_en /* verilator public */,
   output wire        cov_load_tx_shift /* verilator public */,
   output wire        cov_load_tx_buffer /* verilator public */,
   output wire        cov_load_rx_buf /* verilator public */,
   output wire        cov_s3 /* verilator public */,
   output wire        cov_txd /* verilator public */,
   output wire        cov_ss_in_n /* verilator public */,
   output wire        cov_ss_active /* verilator public */,
   output wire [4:0]  cov_dfs /* verilator public */,
   output wire [4:0]  cov_eff_dfs /* verilator public */,
   output wire [3:0]  cov_cfs /* verilator public */,
   output wire [3:0]  cov_eff_cfs /* verilator public */,
   output wire [15:0] cov_baudr /* verilator public */,
   output wire        cov_baud2 /* verilator public */,
   output wire [16:0] cov_ndf /* verilator public */,
   output wire [1:0]  cov_tmod /* verilator public */,
   output wire [1:0]  cov_frf /* verilator public */,
   output wire        cov_scph /* verilator public */,
   output wire        cov_scpol /* verilator public */,
   output wire        cov_srl_test /* verilator public */,
   output wire        cov_ss_tgl_en /* verilator public */,
   output wire        cov_slv_oe /* verilator public */,
   output wire [3:0]  cov_ser /* verilator public */,
   output wire [3:0]  cov_tx_level /* verilator public */,
   output wire [3:0]  cov_rx_level /* verilator public */,
   output wire        cov_tx_full /* verilator public */,
   output wire        cov_tx_empty /* verilator public */,
   output wire        cov_rx_full /* verilator public */,
   output wire        cov_rx_empty /* verilator public */,
   output wire        cov_tx_overflow /* verilator public */,
   output wire        cov_rx_overflow /* verilator public */,
   output wire        cov_rx_underflow /* verilator public */,
   output wire [5:0]  cov_risr /* verilator public */,
   output wire [5:0]  cov_isr /* verilator public */,
   output wire [5:0]  cov_imr /* verilator public */,
   output wire        cov_ssi_txe_intr /* verilator public */,
   output wire        cov_ssi_mst_intr /* verilator public */,
   output wire        cov_s1 /* verilator public */,
   output wire        cov_multi_mst_sync /* verilator public */,
   output wire        cov_s2 /* verilator public */,
   output wire [3:0]  cov_hold_ss_cnt /* verilator public */,
   output wire        cov_start_xfer /* verilator public */,
   output wire        cov_fsm_busy /* verilator public */,
   output wire        cov_fsm_sleep /* verilator public */,
   output wire        cov_ssi_busy /* verilator public */,
   output wire        cov_ssi_sleep /* verilator public */,
   output wire        cov_rx_push /* verilator public */,
   output wire        cov_tx_pop /* verilator public */,
   output wire [31:0] cov_rx_data /* verilator public */,
   output wire [2:0]  cov_mwcr /* verilator public */,
   output wire        cov_spi0_control /* verilator public */,
   output wire        cov_spi1_control /* verilator public */,
   output wire [3:0]  cov_txftlr /* verilator public */,
   output wire        cov_s0 /* verilator public */
   );

  // -- internal nets
  wire        sclk_re, sclk_fe, sclk_mask;
  wire        fsm_slv_sel, ssi_oe_n, fsm_busy, fsm_sleep, fsm_multi_mst;
  wire        load_tx_shift, load_tx_buffer, load_rx_buf;
  wire        tx_shift_en, rx_shift_en, load_start_bit, ssp_load_tx_shift;
  wire [4:0]  tx_dfs;
  wire        start_xfer, baud2, multi_mst_sync, mst_contention;
  wire        tx_push, rx_pop, tx_pop, rx_push, tx_overflow;
  wire        cov_tx_push;   // regfile internal observation (not exposed)
  wire        tx_full, tx_empty, rx_full, rx_empty;
  wire [3:0]  tx_level, rx_level, tx_wc, rx_wc, txftlr, rxftlr;
  wire [31:0] tx_push_data, tx_pop_data, rx_pop_data, rx_push_data;
  wire [3:0]  eff_cfs;
  wire [19:0] ctrlr0;
  wire [3:0]  ser;
  wire [16:0] ndf;
  wire        ssienr;
  wire [2:0]  mwcr;
  wire [5:0]  imr, isr, risr;
  wire        fifo_rst_n, ssi_sleep;
  wire        cov_scpol_active;   // driven by regfile sclk_active
  wire        set0_imr5, set1_imr5;
  wire        rdec_icr_en, rdec_txoicr_en, rdec_rxoicr_en, rdec_rxuicr_en, rdec_msticr_en;

  // -- decoded configuration
  wire [4:0]  dfs    = ctrlr0[4:0];
  wire [1:0]  frf    = ctrlr0[7:6];
  wire        scph   = ctrlr0[8];
  wire        scpol  = ctrlr0[9];
  wire [1:0]  tmod   = ctrlr0[11:10];
  wire        slv_oe = ctrlr0[12];
  wire        srl_test = ctrlr0[13];
  wire        ss_tgl_en = ctrlr0[14];

  assign eff_cfs = (ctrlr0[19:16] < SPEC_CFS_MIN) ? SPEC_CFS_MIN : ctrlr0[19:16];

  // -- FIFO word-count observers (used by the regfile for TXFLR/RXFLR and
  //    the start_xfer threshold).  Connected to the FIFO level outputs.
  assign tx_wc = tx_level;
  assign rx_wc = rx_level;

  // -- register file
  spi_m_regfile u_regfile (
    .clk              (clk),
    .resetn           (rst_n),
    .reg_we           (reg_we),
    .reg_addr         (reg_addr),
    .reg_wdata        (reg_wdata),
    .reg_re           (reg_re),
    .reg_rdata        (reg_rdata),
    .fsm_busy         (fsm_busy),
    .fsm_sleep        (fsm_sleep),
    .fsm_multi_mst    (fsm_multi_mst),
    .mst_contention   (mst_contention),
    .multi_mst_sync   (multi_mst_sync),
    .start_xfer       (start_xfer),
    .baud2            (baud2),
    .baudr            (cov_baudr),
    .sclk_active      (cov_scpol_active),
    .ctrlr0           (ctrlr0),
    .ser              (ser),
    .ndf              (ndf),
    .set0_imr5        (set0_imr5),
    .set1_imr5        (set1_imr5),
    .tx_push_data     (tx_push_data),
    .tx_push          (tx_push),
    .rx_pop           (rx_pop),
    .tx_pop           (tx_pop),
    .tx_fifo_wc       (tx_wc),
    .rx_fifo_wc       (rx_wc),
    .fifo_rst_n       (fifo_rst_n),
    .tx_full          (tx_full),
    .tx_empty         (tx_empty),
    .rx_full          (rx_full),
    .rx_empty         (rx_empty),
    .ssi_sleep        (ssi_sleep),
    .ssienr           (ssienr),
    .mwcr             (mwcr),
    .txftlr           (txftlr),
    .rxftlr           (rxftlr),
    .isr              (isr),
    .risr             (risr),
    .rdec_icr_en      (rdec_icr_en),
    .rdec_txoicr_en   (rdec_txoicr_en),
    .rdec_rxoicr_en   (rdec_rxoicr_en),
    .rdec_rxuicr_en   (rdec_rxuicr_en),
    .rdec_msticr_en   (rdec_msticr_en),
    .tx_overflow      (tx_overflow),
    .imr              (imr),
    .rx_pop_data      (rx_pop_data),
    .spec_cfs_min     (SPEC_CFS_MIN),
    .spec_txftlr_dflt (SPEC_TXFTLR_DFLT),
    .cov_dfs          (cov_dfs),
    .cov_eff_dfs      (cov_eff_dfs),
    .cov_cfs          (cov_cfs),
    .cov_eff_cfs      (cov_eff_cfs),
    .cov_frf          (cov_frf),
    .cov_scph         (cov_scph),
    .cov_scpol        (cov_scpol),
    .cov_tmod         (cov_tmod),
    .cov_srl_test     (cov_srl_test),
    .cov_ss_tgl_en    (cov_ss_tgl_en),
    .cov_slv_oe       (cov_slv_oe),
    .cov_ser          (cov_ser),
    .cov_baud2        (cov_baud2),
    .cov_ndf          (cov_ndf),
    .cov_s2           (cov_s2),
    .cov_ssi_busy     (cov_ssi_busy),
    .cov_ssi_sleep    (cov_ssi_sleep),
    .cov_multi_mst_sync(cov_multi_mst_sync),
    .cov_s1           (cov_s1),
    .cov_start_xfer   (cov_start_xfer),
    .cov_tx_push      (cov_tx_push),
    .cov_tx_pop       (cov_tx_pop),
    .cov_txftlr       (cov_txftlr)
  );

  // -- interrupt controller
  spi_m_intctl u_intctl (
    .clk              (clk),
    .resetn           (rst_n),
    .imr              (imr),
    .rdec_rxoicr_en   (rdec_rxoicr_en),
    .rdec_txoicr_en   (rdec_txoicr_en),
    .rdec_rxuicr_en   (rdec_rxuicr_en),
    .rdec_msticr_en   (rdec_msticr_en),
    .rdec_icr_en      (rdec_icr_en),
    .rd_en            (reg_re),
    .tx_empty         (tx_empty),
    .tx_overflow      (tx_overflow),
    .rx_underflow     (cov_rx_underflow),
    .rx_full          (rx_full),
    .rx_overflow      (cov_rx_overflow),
    .multi_mst_sync   (multi_mst_sync),
    .set0_imr5        (set0_imr5),
    .set1_imr5        (set1_imr5),
    .ssi_en           (ssienr),
    .ssi_txe_intr     (ssi_txe_intr),
    .ssi_txo_intr     (ssi_txo_intr),
    .ssi_rxu_intr     (ssi_rxu_intr),
    .ssi_rxf_intr     (ssi_rxf_intr),
    .ssi_rxo_intr     (ssi_rxo_intr),
    .ssi_mst_intr     (ssi_mst_intr),
    .mst_contention   (mst_contention),
    .risr             (risr),
    .isr              (isr)
  );

  // -- FIFOs
  spi_m_fifo u_fifo (
    .clk           (clk),
    .rst_n         (rst_n),
    .fifo_rst_n    (fifo_rst_n),
    .tx_push       (tx_push),
    .tx_push_data  (tx_push_data),
    .tx_pop        (tx_pop),
    .rx_push       (rx_push),
    .rx_push_data  (rx_push_data),
    .rx_pop        (rx_pop),
    .tx_full       (tx_full),
    .tx_empty      (tx_empty),
    .rx_full       (rx_full),
    .rx_empty      (rx_empty),
    .rx_overflow   (cov_rx_overflow),
    .rx_underflow  (cov_rx_underflow),
    .tx_level      (tx_level),
    .rx_level      (rx_level),
    .tx_pop_data   (tx_pop_data),
    .rx_pop_data   (rx_pop_data)
  );

  // -- serial clock generator
  spi_m_sclkgen u_sclkgen (
    .clk          (clk),
    .rst_n        (rst_n),
    .baudr        (cov_baudr),
    .sclk_mask    (sclk_mask),
    .sclk_active  (cov_scpol_active),
    .ssi_en       (ssienr),
    .sclk_out     (sclk_out),
    .sclk_re      (sclk_re),
    .sclk_fe      (sclk_fe)
  );

  // -- master transfer FSM
  spi_m_mstfsm u_mstfsm (
    .clk           (clk),
    .rst_n         (rst_n),
    .ssi_en        (ssienr),
    .start_xfer    (start_xfer),
    .tx_empty      (tx_empty),
    .sclk_re       (sclk_re),
    .sclk_fe       (sclk_fe),
    .mwcr          (mwcr),
    .frf           (frf),
    .scph          (scph),
    .ss_tgl_en     (ss_tgl_en),
    .tmod          (tmod),
    .dfs           (dfs),
    .cfs           (eff_cfs),
    .ndf           (ndf),
    .srl_test      (srl_test),
    .baud2         (baud2),
    .ss_in_n       (ss_in_n),
    .rxd           (rxd),
    .tx_fifo_level (tx_level),
    .txfthr        (txftlr),
    .spec_hold_ss  (SPEC_HOLD_SS),
    .fsm_busy      (fsm_busy),
    .fsm_slv_sel   (fsm_slv_sel),
    .sclk_mask     (sclk_mask),
    .ssi_oe_n      (ssi_oe_n),
    .fsm_sleep     (fsm_sleep),
    .load_tx_shift (load_tx_shift),
    .ssp_load_tx_shift(ssp_load_tx_shift),
    .load_tx_buffer(load_tx_buffer),
    .load_rx_buf   (load_rx_buf),
    .tx_shift_en   (tx_shift_en),
    .tx_dfs        (tx_dfs),
    .rx_shift_en   (rx_shift_en),
    .fsm_multi_mst (fsm_multi_mst),
    .load_start_bit(load_start_bit),
    .cov_fsm_state_id (cov_fsm_state_id),
    .cov_fsm_state (cov_fsm_state),
    .cov_bit_cnt   (cov_bit_cnt),
    .cov_ctrl_cnt  (cov_ctrl_cnt),
    .cov_frame_cnt (cov_frame_cnt),
    .cov_f_done    (cov_f_done),
    .cov_c_done    (cov_c_done),
    .cov_x_done    (cov_x_done),
    .cov_last_frame(cov_last_frame),
    .cov_tx_shift_en(cov_tx_shift_en),
    .cov_rx_shift_en(cov_rx_shift_en),
    .cov_load_tx_shift(cov_load_tx_shift),
    .cov_load_tx_buffer(cov_load_tx_buffer),
    .cov_load_rx_buf(cov_load_rx_buf),
    .cov_s3         (cov_s3),
    .cov_hold_ss_cnt(cov_hold_ss_cnt),
    .cov_spi0_control(cov_spi0_control),
    .cov_spi1_control(cov_spi1_control),
    .cov_fsm_busy   (cov_fsm_busy),
    .cov_fsm_sleep  (cov_fsm_sleep),
    .cov_start_bit_ctl(),
    .cov_s0         (cov_s0)
  );

  // -- shift controller
  spi_m_shift u_shift (
    .clk            (clk),
    .rst_n          (rst_n),
    .tx_shift_en    (tx_shift_en),
    .load_tx_buffer (load_tx_buffer),
    .load_start_bit (load_start_bit),
    .rx_shift_en    (rx_shift_en),
    .load_rx_buf    (load_rx_buf),
    .load_tx_shift  (load_tx_shift),
    .tx_pop_data    (tx_pop_data),
    .rxd            (rxd),
    .dfs            (dfs),
    .tx_dfs         (tx_dfs),
    .fsm_slv_sel    (fsm_slv_sel),
    .ser            (ser),
    .frf            (frf),
    .srl_test       (srl_test),
    .ssi_en         (ssienr),
    .ss_n           (ss_n),
    .txd            (txd),
    .rx_push_data   (rx_push_data),
    .rx_push        (rx_push),
    .tx_pop         (tx_pop)
  );

  assign ssi_busy = cov_ssi_busy;

  // -- coverage wiring
  assign cov_sclk_re      = sclk_re;
  assign cov_sclk_fe      = sclk_fe;
  assign cov_sclk_out     = sclk_out;
  assign cov_sclk_mask    = sclk_mask;
  assign cov_txd          = txd;
  assign cov_ss_in_n      = ss_in_n;
  assign cov_ss_active    = (frf == 2'b01) ? (|ss_n) : (|(~ss_n));
  assign cov_tx_level     = tx_level;
  assign cov_rx_level     = rx_level;
  assign cov_tx_full      = tx_full;
  assign cov_tx_empty     = tx_empty;
  assign cov_rx_full      = rx_full;
  assign cov_rx_empty     = rx_empty;
  assign cov_tx_overflow  = tx_overflow;
  assign cov_risr         = risr;
  assign cov_isr          = isr;
  assign cov_imr          = imr;
  assign cov_ssi_txe_intr = ssi_txe_intr;
  assign cov_ssi_mst_intr = ssi_mst_intr;
  assign cov_rx_push      = rx_push;
  assign cov_tx_pop       = tx_pop;
  assign cov_rx_data      = rx_pop_data;
  assign cov_mwcr         = mwcr;

endmodule // spi_m_top
