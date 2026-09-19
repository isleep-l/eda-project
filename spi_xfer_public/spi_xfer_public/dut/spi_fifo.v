// File    : spi_fifo.v
// Abstract: spi_xfer_public TX and RX FIFOs (8 x 32-bit, circular).
module spi_fifo
  (
   input  wire        clk,
   input  wire        rst_n,
   input  wire        tx_push,          // TX FIFO push
   input  wire [31:0] tx_push_data,
   input  wire        tx_pop,           // TX FIFO pop
   input  wire        rx_push,          // RX FIFO push
   input  wire [31:0] rx_push_data,
   input  wire        rx_pop,           // RX FIFO pop
   output wire        tx_full,
   output wire        tx_empty,
   output wire        rx_full,
   output wire        rx_empty,
   output wire [3:0]  tx_level,         // TX FIFO level 0..8
   output wire [3:0]  rx_level,         // RX FIFO level 0..8
   output wire [31:0] tx_pop_data,      // TX FIFO output data
   output wire [31:0] rx_pop_data       // RX FIFO output data
   );

  localparam AW = 3;                    // depth 8
  localparam DEPTH = 8;

  reg [31:0] tx_mem [0:DEPTH-1];
  reg [31:0] rx_mem [0:DEPTH-1];
  reg [AW-1:0] tx_wr_ptr;
  reg [AW-1:0] tx_rd_ptr;
  reg [AW-1:0] rx_wr_ptr;
  reg [AW-1:0] rx_rd_ptr;
  reg [3:0]    tx_cnt;
  reg [3:0]    rx_cnt;

  wire tx_wr_ok = tx_push && !tx_full;
  wire tx_rd_ok = tx_pop  && !tx_empty;
  wire rx_wr_ok = rx_push && !rx_full;
  wire rx_rd_ok = rx_pop  && !rx_empty;

  // -- TX FIFO control
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_wr_ptr <= 3'd0;
      tx_rd_ptr <= 3'd0;
      tx_cnt    <= 4'd0;
    end else begin
      if (tx_wr_ok) begin
        tx_mem[tx_wr_ptr] <= tx_push_data;
        tx_wr_ptr <= tx_wr_ptr + 3'd1;
      end
      if (tx_rd_ok) begin
        tx_rd_ptr <= tx_rd_ptr + 3'd1;
      end
      case ({tx_wr_ok, tx_rd_ok})
        2'b10 : tx_cnt <= tx_cnt + 4'd1;
        2'b01 : tx_cnt <= tx_cnt - 4'd1;
        default: tx_cnt <= tx_cnt;
      endcase
    end
  end

  // -- RX FIFO control
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_wr_ptr <= 3'd0;
      rx_rd_ptr <= 3'd0;
      rx_cnt    <= 4'd0;
    end else begin
      if (rx_wr_ok) begin
        rx_mem[rx_wr_ptr] <= rx_push_data;
        rx_wr_ptr <= rx_wr_ptr + 3'd1;
      end
      if (rx_rd_ok) begin
        rx_rd_ptr <= rx_rd_ptr + 3'd1;
      end
      case ({rx_wr_ok, rx_rd_ok})
        2'b10 : rx_cnt <= rx_cnt + 4'd1;
        2'b01 : rx_cnt <= rx_cnt - 4'd1;
        default: rx_cnt <= rx_cnt;
      endcase
    end
  end

  assign tx_full   = (tx_cnt == 4'd8);
  assign tx_empty  = (tx_cnt == 4'd0);
  assign rx_full   = (rx_cnt == 4'd8);
  assign rx_empty  = (rx_cnt == 4'd0);
  assign tx_level  = tx_cnt;
  assign rx_level  = rx_cnt;

  assign tx_pop_data = tx_mem[tx_rd_ptr];
  assign rx_pop_data = rx_mem[rx_rd_ptr];

endmodule // spi_fifo
