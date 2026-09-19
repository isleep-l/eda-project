// File    : spi_regfile.v
// Abstract: spi_xfer_public register file.
//           Simple word-addressed register bus. Config registers
//           (CTRLR0, CTRLR1, SER, BAUDR, TXFTLR) accept writes only
//           while SSIENR == 0; SSIENR and DR are always accessible.
//
// Register map (word offset)
//   0x0 CTRLR0  [4:0]=DFS [6:5]=FRF [7]=SCPH [8]=SCPOL
//               [10:9]=TMOD [11]=SRL_TEST [12]=SSTE
//   0x1 CTRLR1  [16:0]=NDF
//   0x2 SSIENR  [0]=SSI_EN
//   0x3 SER     [3:0]=slave enable
//   0x4 BAUDR   [15:0]=baud divisor
//   0x5 TXFTLR  [3:0]=TX FIFO threshold
//   0x6 TXFLR   (RO) [3:0]=TX FIFO level
//   0x7 RXFLR   (RO) [3:0]=RX FIFO level
//   0x8 SR      (RO) [0]=busy [1]=TXF_NOT_FULL [2]=TXE
//                     [3]=RXF_NOT_EMPTY [4]=RXF_FULL
//   0x9 DR      (RW) write=TX FIFO push, read=RX FIFO pop
module spi_regfile
  (
   input  wire        clk,
   input  wire        rst_n,
   input  wire        reg_we,           // register write enable
   input  wire [3:0]  reg_addr,         // register word offset
   input  wire [31:0] reg_wdata,
   input  wire        reg_re,           // register read enable
   output reg  [31:0] reg_rdata,

   // -- register outputs
   output reg  [31:0] ctrlr0,
   output reg  [31:0] ctrlr1,
   output reg         ssienr,
   output reg  [3:0]  ser,
   output reg  [15:0] baudr,
   output reg  [3:0]  txftlr,

   // -- status / FIFO interface
   input  wire        fsm_busy,
   input  wire        tx_full,
   input  wire        tx_empty,
   input  wire [3:0]  tx_level,
   input  wire        rx_full,
   input  wire        rx_empty,
   input  wire [3:0]  rx_level,
   input  wire [31:0] rx_pop_data,
   input  wire        tx_pop,           // TX FIFO pop pulse from shift
   output wire        tx_push,          // TX FIFO push pulse
   output wire        rx_pop,           // RX FIFO pop pulse
   output reg         start_xfer        // start transfer request
   );

  localparam A_CTRLR0 = 4'h0;
  localparam A_CTRLR1 = 4'h1;
  localparam A_SSIENR = 4'h2;
  localparam A_SER    = 4'h3;
  localparam A_BAUDR  = 4'h4;
  localparam A_TXFTLR = 4'h5;
  localparam A_TXFLR  = 4'h6;
  localparam A_RXFLR  = 4'h7;
  localparam A_SR     = 4'h8;
  localparam A_DR     = 4'h9;

  // -- TX FIFO push / RX FIFO pop (DR access)
  assign tx_push = reg_we  && (reg_addr == A_DR);
  assign rx_pop  = reg_re  && (reg_addr == A_DR);

  // -- register writes
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ctrlr0 <= 32'd0;
      ctrlr1 <= 32'd0;
      ssienr <= 1'b0;
      ser    <= 4'd0;
      baudr  <= 16'd0;
      txftlr <= 4'd0;
    end else if (reg_we) begin
      if (reg_addr == A_SSIENR) begin
        ssienr <= reg_wdata[0];
      end else if (!ssienr) begin
        case (reg_addr)
          A_CTRLR0 : ctrlr0 <= reg_wdata;
          A_CTRLR1 : ctrlr1 <= {15'd0, reg_wdata[16:0]};
          A_SER    : ser    <= reg_wdata[3:0];
          A_BAUDR  : baudr  <= reg_wdata[15:0];
          A_TXFTLR : txftlr <= reg_wdata[3:0];
          default  : ;
        endcase
      end
    end
  end

  // -- register reads
  always_comb begin
    case (reg_addr)
      A_CTRLR0 : reg_rdata = ctrlr0;
      A_CTRLR1 : reg_rdata = ctrlr1;
      A_SSIENR : reg_rdata = {31'd0, ssienr};
      A_SER    : reg_rdata = {28'd0, ser};
      A_BAUDR  : reg_rdata = {16'd0, baudr};
      A_TXFTLR : reg_rdata = {28'd0, txftlr};
      A_TXFLR  : reg_rdata = {28'd0, tx_level};
      A_RXFLR  : reg_rdata = {28'd0, rx_level};
      A_SR     : reg_rdata = {27'd0, rx_full, (~rx_empty), tx_empty, (~tx_full), fsm_busy};
      A_DR     : reg_rdata = rx_pop_data;
      default  : reg_rdata = 32'd0;
    endcase
  end

  // -- start transfer generation (registered, mirrors fl_ssi)
  wire slv_sel        = (ser != 4'd0);
  wire tx_empty_early = (~tx_push) & tx_pop & (tx_level == 4'd1);
  wire tx_fifo_ready  = (ctrlr0[10:9] != 2'b10) ? (tx_level > {1'b0, txftlr}) : (~tx_empty);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      start_xfer <= 1'b0;
    end else begin
      start_xfer <= slv_sel && (!tx_empty_early) && tx_fifo_ready;
    end
  end

endmodule // spi_regfile
