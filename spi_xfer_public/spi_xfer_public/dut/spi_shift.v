// File    : spi_shift.v
// Abstract: spi_xfer_public shift controller.
//           - TX shift register + TX buffer (MSB-first serial out)
//           - RX shift register + RX buffer (MSB-first serial in)
//           - loopback mux (srl_test)
//           - slave select outputs (active-low SPI / active-high SSP)
//           - TX FIFO pop and RX FIFO push pulses
module spi_shift
  (
   input  wire        clk,
   input  wire        rst_n,
   input  wire        tx_shift_en,     // TX shift enable
   input  wire        load_tx_shift,   // load TX shift register
   input  wire        load_tx_buffer,  // reload TX shift register from buffer
   input  wire        rx_shift_en,     // RX shift enable
   input  wire        load_rx_buf,     // load RX buffer
   input  wire [31:0] tx_pop_data,     // TX FIFO output data
   input  wire        rxd,             // receive data from serial bus
   input  wire [5:0]  dfs,             // effective data frame size
   input  wire        fsm_slv_sel,     // slave select enable from FSM
   input  wire [3:0]  ser,             // slave enable register
   input  wire [1:0]  frf,             // frame format: 0=SPI, 1=SSP
   input  wire        srl_test,        // loopback mode
   input  wire        ssi_en,          // SSI enable
   output reg         txd,             // transmit data out
   output wire        tx_pop,          // TX FIFO pop pulse
   output wire        rx_push,         // RX FIFO push pulse
   output reg  [3:0]  ss_n,            // slave select outputs
   output wire [31:0] rx_push_data,    // RX FIFO push data
   // -- coverage observation
   output wire        cov_txd,
   output wire [31:0] cov_rx_buffer
   );

  // -- local registers
  reg  [31:0] tx_shift_reg;
  reg  [31:0] tx_buffer;
  reg  [31:0] rx_shift_reg;
  reg  [31:0] rx_buffer;
  reg         tx_pop_ir;
  reg         rx_push_ir;

  // -- loopback mux
  wire rxd_mst_int = srl_test ? txd : rxd;

  // -- TX shift register
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_shift_reg <= 32'd0;
      tx_buffer    <= 32'd0;
      txd          <= 1'b0;
    end else begin
      if (load_tx_shift) begin
        tx_shift_reg <= tx_pop_data;
        tx_buffer    <= tx_pop_data;
        case (dfs)
          6'd4 : txd <= tx_pop_data[3];
          6'd5 : txd <= tx_pop_data[4];
          6'd6 : txd <= tx_pop_data[5];
          6'd7 : txd <= tx_pop_data[6];
          6'd8 : txd <= tx_pop_data[7];
          6'd9 : txd <= tx_pop_data[8];
          6'd10: txd <= tx_pop_data[9];
          6'd11: txd <= tx_pop_data[10];
          6'd12: txd <= tx_pop_data[11];
          6'd13: txd <= tx_pop_data[12];
          6'd14: txd <= tx_pop_data[13];
          6'd15: txd <= tx_pop_data[14];
          6'd16: txd <= tx_pop_data[15];
          6'd17: txd <= tx_pop_data[16];
          6'd18: txd <= tx_pop_data[17];
          6'd19: txd <= tx_pop_data[18];
          6'd20: txd <= tx_pop_data[19];
          6'd21: txd <= tx_pop_data[20];
          6'd22: txd <= tx_pop_data[21];
          6'd23: txd <= tx_pop_data[22];
          6'd24: txd <= tx_pop_data[23];
          6'd25: txd <= tx_pop_data[24];
          6'd26: txd <= tx_pop_data[25];
          6'd27: txd <= tx_pop_data[26];
          6'd28: txd <= tx_pop_data[27];
          6'd29: txd <= tx_pop_data[28];
          6'd30: txd <= tx_pop_data[29];
          6'd31: txd <= tx_pop_data[30];
          default : txd <= tx_pop_data[31];
        endcase
      end else if (load_tx_buffer) begin
        tx_shift_reg <= tx_buffer;
        case (dfs)
          6'd4 : txd <= tx_buffer[3];
          6'd5 : txd <= tx_buffer[4];
          6'd6 : txd <= tx_buffer[5];
          6'd7 : txd <= tx_buffer[6];
          6'd8 : txd <= tx_buffer[7];
          6'd9 : txd <= tx_buffer[8];
          6'd10: txd <= tx_buffer[9];
          6'd11: txd <= tx_buffer[10];
          6'd12: txd <= tx_buffer[11];
          6'd13: txd <= tx_buffer[12];
          6'd14: txd <= tx_buffer[13];
          6'd15: txd <= tx_buffer[14];
          6'd16: txd <= tx_buffer[15];
          6'd17: txd <= tx_buffer[16];
          6'd18: txd <= tx_buffer[17];
          6'd19: txd <= tx_buffer[18];
          6'd20: txd <= tx_buffer[19];
          6'd21: txd <= tx_buffer[20];
          6'd22: txd <= tx_buffer[21];
          6'd23: txd <= tx_buffer[22];
          6'd24: txd <= tx_buffer[23];
          6'd25: txd <= tx_buffer[24];
          6'd26: txd <= tx_buffer[25];
          6'd27: txd <= tx_buffer[26];
          6'd28: txd <= tx_buffer[27];
          6'd29: txd <= tx_buffer[28];
          6'd30: txd <= tx_buffer[29];
          6'd31: txd <= tx_buffer[30];
          default : txd <= tx_buffer[31];
        endcase
      end else if (tx_shift_en) begin
        // MSB-first: shift frame left, txd samples current MSB
        txd <= tx_shift_reg[dfs-1];
        case (dfs)
          6'd4 : tx_shift_reg[3:0]  <= {tx_shift_reg[2:0], 1'b0};
          6'd5 : tx_shift_reg[4:0]  <= {tx_shift_reg[3:0], 1'b0};
          6'd6 : tx_shift_reg[5:0]  <= {tx_shift_reg[4:0], 1'b0};
          6'd7 : tx_shift_reg[6:0]  <= {tx_shift_reg[5:0], 1'b0};
          6'd8 : tx_shift_reg[7:0]  <= {tx_shift_reg[6:0], 1'b0};
          6'd9 : tx_shift_reg[8:0]  <= {tx_shift_reg[7:0], 1'b0};
          6'd10: tx_shift_reg[9:0]  <= {tx_shift_reg[8:0], 1'b0};
          6'd11: tx_shift_reg[10:0] <= {tx_shift_reg[9:0], 1'b0};
          6'd12: tx_shift_reg[11:0] <= {tx_shift_reg[10:0], 1'b0};
          6'd13: tx_shift_reg[12:0] <= {tx_shift_reg[11:0], 1'b0};
          6'd14: tx_shift_reg[13:0] <= {tx_shift_reg[12:0], 1'b0};
          6'd15: tx_shift_reg[14:0] <= {tx_shift_reg[13:0], 1'b0};
          6'd16: tx_shift_reg[15:0] <= {tx_shift_reg[14:0], 1'b0};
          6'd17: tx_shift_reg[16:0] <= {tx_shift_reg[15:0], 1'b0};
          6'd18: tx_shift_reg[17:0] <= {tx_shift_reg[16:0], 1'b0};
          6'd19: tx_shift_reg[18:0] <= {tx_shift_reg[17:0], 1'b0};
          6'd20: tx_shift_reg[19:0] <= {tx_shift_reg[18:0], 1'b0};
          6'd21: tx_shift_reg[20:0] <= {tx_shift_reg[19:0], 1'b0};
          6'd22: tx_shift_reg[21:0] <= {tx_shift_reg[20:0], 1'b0};
          6'd23: tx_shift_reg[22:0] <= {tx_shift_reg[21:0], 1'b0};
          6'd24: tx_shift_reg[23:0] <= {tx_shift_reg[22:0], 1'b0};
          6'd25: tx_shift_reg[24:0] <= {tx_shift_reg[23:0], 1'b0};
          6'd26: tx_shift_reg[25:0] <= {tx_shift_reg[24:0], 1'b0};
          6'd27: tx_shift_reg[26:0] <= {tx_shift_reg[25:0], 1'b0};
          6'd28: tx_shift_reg[27:0] <= {tx_shift_reg[26:0], 1'b0};
          6'd29: tx_shift_reg[28:0] <= {tx_shift_reg[27:0], 1'b0};
          6'd30: tx_shift_reg[29:0] <= {tx_shift_reg[28:0], 1'b0};
          6'd31: tx_shift_reg[30:0] <= {tx_shift_reg[29:0], 1'b0};
          default : tx_shift_reg[31:0] <= {tx_shift_reg[30:0], 1'b0};
        endcase
      end
    end
  end

  // -- RX shift register
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_shift_reg <= 32'd0;
    end else begin
      if (rx_shift_en) begin
        rx_shift_reg <= {rx_shift_reg[30:0], rxd_mst_int};
      end
    end
  end

  // -- RX buffer (zero-pad upper unused bits)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_buffer <= 32'd0;
    end else begin
      if (load_rx_buf) begin
        case (dfs)
          6'd4 : rx_buffer <= {28'd0, rx_shift_reg[3:0]};
          6'd5 : rx_buffer <= {27'd0, rx_shift_reg[4:0]};
          6'd6 : rx_buffer <= {26'd0, rx_shift_reg[5:0]};
          6'd7 : rx_buffer <= {26'd0, rx_shift_reg[6:0]};
          6'd8 : rx_buffer <= {24'd0, rx_shift_reg[7:0]};
          6'd9 : rx_buffer <= {23'd0, rx_shift_reg[8:0]};
          6'd10: rx_buffer <= {22'd0, rx_shift_reg[9:0]};
          6'd11: rx_buffer <= {21'd0, rx_shift_reg[10:0]};
          6'd12: rx_buffer <= {20'd0, rx_shift_reg[11:0]};
          6'd13: rx_buffer <= {19'd0, rx_shift_reg[12:0]};
          6'd14: rx_buffer <= {18'd0, rx_shift_reg[13:0]};
          6'd15: rx_buffer <= {17'd0, rx_shift_reg[14:0]};
          6'd16: rx_buffer <= {16'd0, rx_shift_reg[15:0]};
          6'd17: rx_buffer <= {16'd0, rx_shift_reg[16:0]};
          6'd18: rx_buffer <= {14'd0, rx_shift_reg[17:0]};
          6'd19: rx_buffer <= {13'd0, rx_shift_reg[18:0]};
          6'd20: rx_buffer <= {12'd0, rx_shift_reg[19:0]};
          6'd21: rx_buffer <= {11'd0, rx_shift_reg[20:0]};
          6'd22: rx_buffer <= {10'd0, rx_shift_reg[21:0]};
          6'd23: rx_buffer <= {9'd0,  rx_shift_reg[22:0]};
          6'd24: rx_buffer <= {8'd0,  rx_shift_reg[23:0]};
          6'd25: rx_buffer <= {7'd0,  rx_shift_reg[24:0]};
          6'd26: rx_buffer <= {6'd0,  rx_shift_reg[25:0]};
          6'd27: rx_buffer <= {6'd0,  rx_shift_reg[26:0]};
          6'd28: rx_buffer <= {4'd0,  rx_shift_reg[27:0]};
          6'd29: rx_buffer <= {3'd0,  rx_shift_reg[28:0]};
          6'd30: rx_buffer <= {2'd0,  rx_shift_reg[29:0]};
          6'd31: rx_buffer <= {1'd0,  rx_shift_reg[30:0]};
          default : rx_buffer <= rx_shift_reg;
        endcase
      end
    end
  end

  // -- slave select outputs
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ss_n <= 4'b1111;
    end else begin
      if (frf == 2'b01) begin
        // SSP: active high
        ss_n <= ser & {4{fsm_slv_sel}};
      end else begin
        // SPI / others: active low
        ss_n <= ~(ser & {4{fsm_slv_sel}});
      end
    end
  end

  // -- TX pop / RX push pulses
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_pop_ir  <= 1'b0;
      rx_push_ir <= 1'b0;
    end else begin
      tx_pop_ir  <= load_tx_shift;
      rx_push_ir <= ssi_en ? load_rx_buf : 1'b0;
    end
  end

  assign tx_pop       = tx_pop_ir;
  assign rx_push      = rx_push_ir;
  assign rx_push_data = rx_buffer;

  assign cov_txd       = txd;
  assign cov_rx_buffer = rx_buffer;

endmodule // spi_shift
