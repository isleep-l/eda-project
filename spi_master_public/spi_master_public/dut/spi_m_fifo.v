// File    : spi_m_fifo.v
// Abstract: spi_master_public - TX / RX FIFOs (8 x 32-bit each).
//           Provides level/full/empty status and overflow/underflow
//           detection for the register file and interrupt controller.
module spi_m_fifo
  (
   input         clk,
   input         rst_n,      // async reset
   input         fifo_rst_n, // sync soft reset (ssi disabled)
   input         tx_push,
   input  [31:0] tx_push_data,
   input         tx_pop,
   input         rx_push,
   input  [31:0] rx_push_data,
   input         rx_pop,
   output        tx_full,
   output        tx_empty,
   output        rx_full,
   output        rx_empty,
   output        rx_overflow,   // rx_push while full
   output        rx_underflow,  // rx_pop while empty
   output [3:0]  tx_level,
   output [3:0]  rx_level,
   output [31:0] tx_pop_data,
   output [31:0] rx_pop_data
   );

  localparam [3:0] DEPTH = 4'd8;

  reg [31:0] tx_mem[0:7];
  reg [31:0] rx_mem[0:7];
  reg [3:0]  tx_rd, tx_wr;   // 4-bit pointers (bit 3 = wrap flag)
  reg [3:0]  rx_rd, rx_wr;

  wire tx_full_w  = ((tx_wr[2:0] == tx_rd[2:0]) && (tx_wr[3] != tx_rd[3]));
  wire tx_empty_w = (tx_wr == tx_rd);
  wire rx_full_w  = ((rx_wr[2:0] == rx_rd[2:0]) && (rx_wr[3] != rx_rd[3]));
  wire rx_empty_w = (rx_wr == rx_rd);

  assign tx_full  = tx_full_w;
  assign tx_empty = tx_empty_w;
  assign rx_full  = rx_full_w;
  assign rx_empty = rx_empty_w;

  assign tx_level = tx_wr - tx_rd;   // 4-bit difference
  assign rx_level = rx_wr - rx_rd;

  assign rx_overflow  = rx_push & rx_full_w & (~rx_pop);
  assign rx_underflow = rx_pop & rx_empty_w;

  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      tx_wr <= 4'b0;
      tx_rd <= 4'b0;
      rx_wr <= 4'b0;
      rx_rd <= 4'b0;
    end else if (fifo_rst_n == 1'b0) begin
      tx_wr <= 4'b0;
      tx_rd <= 4'b0;
      rx_wr <= 4'b0;
      rx_rd <= 4'b0;
    end else begin
      // TX FIFO
      if (tx_push & (~tx_full_w)) begin
        tx_mem[tx_wr[2:0]] <= tx_push_data;
        tx_wr <= tx_wr + 4'd1;
      end
      if (tx_pop & (~tx_empty_w)) begin
        tx_rd <= tx_rd + 4'd1;
      end
      // RX FIFO
      if (rx_push & (~rx_full_w)) begin
        rx_mem[rx_wr[2:0]] <= rx_push_data;
        rx_wr <= rx_wr + 4'd1;
      end
      if (rx_pop & (~rx_empty_w)) begin
        rx_rd <= rx_rd + 4'd1;
      end
    end
  end

  assign tx_pop_data = tx_mem[tx_rd[2:0]];
  assign rx_pop_data = rx_mem[rx_rd[2:0]];

endmodule // spi_m_fifo
