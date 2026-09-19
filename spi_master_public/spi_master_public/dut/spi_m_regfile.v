// File    : spi_m_regfile.v
// Abstract: spi_master_public - Register file.
//           Software-visible registers CTRLR0..DR plus the start_xfer
//           generator, FIFO status and the hidden configuration:
//             SPEC_CFS_MIN      - minimum Microwire control frame size
//             SPEC_TXFTLR_DFLT  - TXFTLR reset value
module spi_m_regfile
  (
   input         clk,
   input         resetn,
   input         reg_we,
   input  [5:0]  reg_addr,
   input  [31:0] reg_wdata,
   input         reg_re,
   output reg [31:0] reg_rdata,

   input         fsm_busy,
   input         fsm_sleep,
   input         fsm_multi_mst,
   input         mst_contention,
   output        multi_mst_sync,
   output reg    start_xfer,
   output reg    baud2,
   output reg [15:0] baudr,
   output reg    sclk_active,
   output [19:0] ctrlr0,
   output [3:0]  ser,
   output [16:0] ndf,
   output        set0_imr5,
   output        set1_imr5,
   output reg [31:0] tx_push_data,
   output reg        tx_push,
   output        rx_pop,
   input         tx_pop,
   input  [3:0]  tx_fifo_wc,
   input  [3:0]  rx_fifo_wc,
   output        fifo_rst_n,
   input         tx_full,
   input         tx_empty,
   input         rx_full,
   input         rx_empty,
   output        ssi_sleep,
   output reg    ssienr,
   output [2:0]  mwcr,
   output reg [3:0] txftlr,
   output reg [3:0] rxftlr,
   input  [5:0]  isr,
   input  [5:0]  risr,
   output        rdec_icr_en,
   output        rdec_txoicr_en,
   output        rdec_rxoicr_en,
   output        rdec_rxuicr_en,
   output        rdec_msticr_en,
   output        tx_overflow,
   output [5:0]  imr,
   input  [31:0] rx_pop_data,
   input  [3:0]  spec_cfs_min,
   input  [3:0]  spec_txftlr_dflt,

   // -- coverage
   output [4:0]  cov_dfs,
   output [4:0]  cov_eff_dfs,
   output [3:0]  cov_cfs,
   output [3:0]  cov_eff_cfs,
   output [1:0]  cov_frf,
   output        cov_scph,
   output        cov_scpol,
   output [1:0]  cov_tmod,
   output        cov_srl_test,
   output        cov_ss_tgl_en,
   output        cov_slv_oe,
   output [3:0]  cov_ser,
   output        cov_baud2,
   output [16:0] cov_ndf,
   output        cov_s2,
   output        cov_ssi_busy,
   output        cov_ssi_sleep,
   output        cov_multi_mst_sync,
   output        cov_s1,
   output        cov_start_xfer,
   output        cov_tx_push,
   output        cov_tx_pop,
   output        cov_txftlr
   );

  // -- register offsets
  localparam [5:0] A_CTRLR0   = 6'd0;
  localparam [5:0] A_CTRLR1   = 6'd1;
  localparam [5:0] A_SSIENR   = 6'd2;
  localparam [5:0] A_MWCR     = 6'd3;
  localparam [5:0] A_SER      = 6'd4;
  localparam [5:0] A_BAUDR    = 6'd5;
  localparam [5:0] A_TXFTLR   = 6'd6;
  localparam [5:0] A_RXFTLR   = 6'd7;
  localparam [5:0] A_TXFLR    = 6'd8;
  localparam [5:0] A_RXFLR    = 6'd9;
  localparam [5:0] A_SR       = 6'd10;
  localparam [5:0] A_IMR      = 6'd11;
  localparam [5:0] A_ISR      = 6'd12;
  localparam [5:0] A_RISR     = 6'd13;
  localparam [5:0] A_TXOICR   = 6'd14;
  localparam [5:0] A_RXOICR   = 6'd15;
  localparam [5:0] A_RXUICR   = 6'd16;
  localparam [5:0] A_MSTICR   = 6'd17;
  localparam [5:0] A_ICR      = 6'd18;
  localparam [5:0] A_IDR      = 6'd22;
  localparam [5:0] A_VIDR     = 6'd23;
  localparam [5:0] A_DR_LO    = 6'd24;
  localparam [5:0] A_DR_HI    = 6'd59;

  localparam [31:0] IDR_VAL  = 32'h53315F31;   // "S1_1"
  localparam [31:0] VIDR_VAL = 32'h0102_0101;  // v1.2.1.1

  wire        dr_en_w  = (reg_addr >= A_DR_LO) && (reg_addr <= A_DR_HI);
  wire        rdec_dr_en = (reg_addr >= A_DR_LO) && (reg_addr <= A_DR_HI);
  wire        tx_push_c;
  wire        tx_empty_early;
  wire        slv_sel;
  wire        tx_fifo_ready;

  wire ctrlr0_we = (reg_addr == A_CTRLR0) && reg_we;
  wire ctrlr1_we = (reg_addr == A_CTRLR1) && reg_we;
  wire ssienr_we = (reg_addr == A_SSIENR) && reg_we;
  wire mwcr_we   = (reg_addr == A_MWCR)   && reg_we;
  wire ser_we    = (reg_addr == A_SER)    && reg_we;
  wire baudr_we  = (reg_addr == A_BAUDR)  && reg_we;
  wire txftlr_we = (reg_addr == A_TXFTLR) && reg_we;
  wire rxftlr_we = (reg_addr == A_RXFTLR) && reg_we;
  wire dr_we     = dr_en_w && reg_we;
  wire imr_we    = (reg_addr == A_IMR)    && reg_we;

  reg [19:0] ctrlr0_ir;
  reg [15:0] ctrlr1;
  reg [2:0]  mwcr_ir;
  reg [3:0]  ser_ir;
  reg [5:0]  imr_ir;
  reg [3:0]  txflr, rxflr;
  reg        dcol;
  reg        fsm_busy_sr;
  reg        multi_mst_edge;
  reg        ssi_sleep_ir;

  assign ctrlr0 = ctrlr0_ir;
  assign ser    = ser_ir;
  assign imr    = imr_ir;
  assign mwcr   = mwcr_ir;

  assign cov_dfs        = ctrlr0_ir[4:0];
  assign cov_frf        = ctrlr0_ir[7:6];
  assign cov_scph       = ctrlr0_ir[8];
  assign cov_scpol      = ctrlr0_ir[9];
  assign cov_tmod       = ctrlr0_ir[11:10];
  assign cov_slv_oe     = ctrlr0_ir[12];
  assign cov_srl_test   = ctrlr0_ir[13];
  assign cov_ss_tgl_en  = ctrlr0_ir[14];
  assign cov_cfs        = ctrlr0_ir[19:16];

  // hidden minimum clamp on the Microwire control frame size
  assign cov_eff_cfs = (ctrlr0_ir[19:16] < spec_cfs_min) ? spec_cfs_min : ctrlr0_ir[19:16];
  assign cov_eff_dfs = ctrlr0_ir[4:0];

  // CTRLR0 - guarded by ssienr, FRF=11 and DFS<3 rejected
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      ctrlr0_ir <= 20'h0_0808;   // dfs=8, spi0
    end else if (ctrlr0_we && ssienr == 1'b0) begin
      if (reg_wdata[7:6] != 2'b11) ctrlr0_ir[7:6] <= reg_wdata[7:6];
      if (reg_wdata[4:0] >= 5'd3)  ctrlr0_ir[4:0] <= reg_wdata[4:0];
      ctrlr0_ir[14:8]  <= reg_wdata[14:8];
      ctrlr0_ir[19:16] <= reg_wdata[19:16];
    end
  end

  // CTRLR1 - number of data frames = ctrlr1 + 1
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      ctrlr1 <= 16'b0;
    end else if (ctrlr1_we && ssienr == 1'b0) begin
      ctrlr1 <= reg_wdata[15:0];
    end
  end
  assign ndf = {1'b0, ctrlr1} + 17'd1;

  // SSIENR
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      ssienr <= 1'b0;
    end else if (ssienr_we) begin
      ssienr <= reg_wdata[0];
    end
  end

  // MWCR
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      mwcr_ir <= 3'b0;
    end else if (mwcr_we && ssienr == 1'b0) begin
      mwcr_ir <= reg_wdata[2:0];
    end
  end

  // SER
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      ser_ir <= 4'b0;
    end else if (ser_we) begin
      ser_ir <= reg_wdata[3:0];
    end
  end

  // BAUDR - LSB forced to zero
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      baudr <= 16'b0;
    end else if (baudr_we && ssienr == 1'b0) begin
      baudr[7:0]  <= {reg_wdata[7:1], 1'b0};
      baudr[15:8] <= reg_wdata[15:8];
    end
  end

  // baud2 (registered)
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      baud2 <= 1'b0;
    end else begin
      baud2 <= (baudr == 16'd2);
    end
  end

  // TXFTLR - reset value from hidden config
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      txftlr <= spec_txftlr_dflt;
    end else if (txftlr_we) begin
      if (reg_wdata[3:0] <= 4'd7) begin
        txftlr <= reg_wdata[3:0];
      end
    end
  end

  // RXFTLR
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      rxftlr <= 4'b0;
    end else if (rxftlr_we) begin
      if (reg_wdata[3:0] <= 4'd7) begin
        rxftlr <= reg_wdata[3:0];
      end
    end
  end

  // IMR - all interrupts enabled by default
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      imr_ir <= 6'b111111;
    end else if (imr_we) begin
      imr_ir <= reg_wdata[5:0];
    end
  end
  assign set0_imr5 = imr_we & (~reg_wdata[5]);
  assign set1_imr5 = imr_we & reg_wdata[5];

  // DR push (registered, gated by ssienr)
  assign tx_push_c = dr_we & ssienr;
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      tx_push      <= 1'b0;
      tx_push_data <= 32'b0;
    end else begin
      tx_push      <= tx_push_c;
      tx_push_data <= reg_wdata;
    end
  end

  // DR pop (gated by ssienr)
  assign rx_pop = rdec_dr_en && reg_re && ssienr;

  // TX FIFO overflow
  assign tx_overflow = dr_we & (tx_full |
      ((tx_push_c | tx_push) & (tx_fifo_wc[2:0] == 3'b111)) |
      (tx_push_c & tx_push & (tx_fifo_wc[2:0] == 3'b110)));

  // FIFO soft reset when SSI disabled or SSIENR re-written while enabled
  assign fifo_rst_n = ((~ssienr_we) && ssienr) ||
                      (reg_wdata[0] && ssienr && ssienr_we);

  // sclk active level
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      sclk_active <= 1'b1;
    end else begin
      sclk_active <= (ctrlr0_ir[9] == 1'b1 && ctrlr0_ir[7:6] == 2'b00) ? 1'b0 : 1'b1;
    end
  end

  // TXFLR / RXFLR - cleared when SSI disabled
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      txflr <= 4'b0;
      rxflr <= 4'b0;
    end else if (ssienr == 1'b0) begin
      txflr <= 4'b0;
      rxflr <= 4'b0;
    end else begin
      txflr <= tx_fifo_wc;
      rxflr <= rx_fifo_wc;
    end
  end

  // DCOL - data collision status
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      dcol <= 1'b0;
    end else if (multi_mst_sync == 1'b1) begin
      dcol <= 1'b1;
    end else if ((reg_addr == A_SR) && reg_re) begin
      dcol <= 1'b0;
    end
  end

  // multi-master edge detect
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      multi_mst_edge <= 1'b0;
    end else begin
      multi_mst_edge <= fsm_multi_mst;
    end
  end
  assign multi_mst_sync = fsm_multi_mst ^ multi_mst_edge;

  // ssi_busy (registered)
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      fsm_busy_sr <= 1'b0;
    end else begin
      fsm_busy_sr <= fsm_busy;
    end
  end

  // ssi_sleep
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      ssi_sleep_ir <= 1'b1;
    end else begin
      ssi_sleep_ir <= fsm_sleep & (~ssienr);
    end
  end
  assign ssi_sleep = ssi_sleep_ir;

  // start_xfer
  assign tx_empty_early = (~tx_push) & tx_pop & (tx_fifo_wc == 4'd1);
  assign slv_sel = (ser_ir != 4'b0);
  assign tx_fifo_ready = (ctrlr0_ir[11:10] != 2'b10) ?
                         (tx_fifo_wc > {1'b0, txftlr[2:0]}) : (~tx_empty);

  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      start_xfer <= 1'b0;
    end else begin
      start_xfer <= slv_sel & (~tx_empty_early) & (~mst_contention) & tx_fifo_ready;
    end
  end

  // read mux
  always @(*) begin
    reg_rdata = 32'b0;
    case (reg_addr)
      A_CTRLR0: reg_rdata = {8'b0, ctrlr0_ir[19:8], ctrlr0_ir[7:6], 1'b0, ctrlr0_ir[4:0]};
      A_CTRLR1: reg_rdata[15:0] = ctrlr1;
      A_SSIENR: reg_rdata[0] = ssienr;
      A_MWCR:   reg_rdata[2:0] = mwcr_ir;
      A_SER:    reg_rdata[3:0] = ser_ir;
      A_BAUDR:  reg_rdata[15:0] = baudr;
      A_TXFTLR: reg_rdata[3:0] = txftlr;
      A_RXFTLR: reg_rdata[3:0] = rxftlr;
      A_TXFLR:  reg_rdata[3:0] = txflr;
      A_RXFLR:  reg_rdata[3:0] = rxflr;
      A_SR:     reg_rdata = {24'b0, dcol, 1'b0, rx_full, ~rx_empty, tx_empty, ~tx_full, fsm_busy};
      A_IMR:    reg_rdata[5:0] = imr_ir;
      A_ISR:    reg_rdata[5:0] = isr;
      A_RISR:   reg_rdata[5:0] = risr;
      A_TXOICR: reg_rdata[0] = isr[1];
      A_RXOICR: reg_rdata[0] = isr[3];
      A_RXUICR: reg_rdata[0] = isr[2];
      A_MSTICR: reg_rdata[0] = isr[5];
      A_ICR:    reg_rdata[0] = (|isr[5:3]) | isr[2] | isr[1];
      A_IDR:    reg_rdata = IDR_VAL;
      A_VIDR:   reg_rdata = VIDR_VAL;
      default:
        if (rdec_dr_en) reg_rdata = rx_pop_data;
        else            reg_rdata = 32'b0;
    endcase
  end

  assign rdec_txoicr_en = (reg_addr == A_TXOICR);
  assign rdec_rxoicr_en = (reg_addr == A_RXOICR);
  assign rdec_rxuicr_en = (reg_addr == A_RXUICR);
  assign rdec_msticr_en = (reg_addr == A_MSTICR);
  assign rdec_icr_en    = (reg_addr == A_ICR);

  assign cov_ser          = ser_ir;
  assign cov_baud2        = baud2;
  assign cov_ndf          = ndf;
  assign cov_s2            = dcol;
  assign cov_ssi_busy     = fsm_busy_sr;
  assign cov_ssi_sleep    = ssi_sleep_ir;
  assign cov_multi_mst_sync = multi_mst_sync;
  assign cov_s1            = mst_contention;
  assign cov_start_xfer   = start_xfer;
  assign cov_tx_push      = tx_push;
  assign cov_tx_pop       = tx_pop;
  assign cov_txftlr       = txftlr;

endmodule // spi_m_regfile
