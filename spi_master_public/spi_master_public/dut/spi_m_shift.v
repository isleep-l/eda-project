// File    : spi_m_shift.v
// Abstract: spi_master_public - Shift controller.
//           Combines the master TX shifter, TX buffer, RX shifter,
//           RX buffer, slave-select outputs and the FIFO push/pop
//           interface for SPI / SSP / Microwire protocols.
module spi_m_shift
  (
   input         clk,
   input         rst_n,
   input         tx_shift_en,
   input         load_tx_buffer,
   input         load_start_bit,   // microwire start bit
   input         rx_shift_en,
   input         load_rx_buf,
   input         load_tx_shift,
   input  [31:0] tx_pop_data,
   input         rxd,
   input  [4:0]  dfs,               // data frame size
   input  [4:0]  tx_dfs,            // data or control frame size
   input         fsm_slv_sel,
   input  [3:0]  ser,
   input  [1:0]  frf,
   input         srl_test,          // loopback
   input         ssi_en,
   output reg [3:0]  ss_n,
   output reg        txd,
   output [31:0] rx_push_data,
   output        rx_push,
   output reg    tx_pop
   );

  reg         rx_push_reg;
  reg  [31:0] tx_shift_reg;
  reg  [31:0] tx_buffer;
  reg  [31:0] rx_shift_reg;
  reg  [31:0] rx_buffer;
  reg         load_start_bit_ir;

  wire        rxd_mst_int;
  wire        ssp_data_valid_iw = 1'b0;
  wire        i_rx_shift_en = (ssi_en == 1'b0) ? 1'b0 : rx_shift_en;
  wire        i_load_rx_buf = (ssi_en == 1'b0) ? 1'b0 : load_rx_buf;

  // -- slave select outputs
  //  SSP: active high.  SPI / Microwire: active low.
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      ss_n <= 4'b1111;
    end else begin
      if (frf == 2'b01) begin
        ss_n <= ser & {4{fsm_slv_sel}};
      end else begin
        ss_n <= ~(ser & {4{fsm_slv_sel}});
      end
    end
  end

  // -- loopback mode
  assign rxd_mst_int = (srl_test == 1'b1) ? txd : rxd;

  // -- TX FIFO pop (registered)
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      tx_pop <= 1'b0;
    end else begin
      tx_pop <= (load_tx_shift & (~ssp_data_valid_iw));
    end
  end

  // -- RX FIFO push (registered)
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      rx_push_reg <= 1'b0;
    end else begin
      rx_push_reg <= i_load_rx_buf;
    end
  end
  assign rx_push = rx_push_reg;

  // -- TX shift register / TX buffer
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      tx_shift_reg <= 32'b0;
      tx_buffer    <= 32'b0;
    end else begin
      if (load_tx_shift == 1'b1) begin
        tx_shift_reg <= tx_pop_data;
        tx_buffer    <= tx_pop_data;
      end else if (load_start_bit == 1'b1) begin
        // microwire start bit (1 for master)
        tx_shift_reg <= 32'hFFFFFFFF;
      end else if (tx_shift_en == 1'b1) begin
        case (dfs)
          5'd1 : tx_shift_reg[1:0]  <= {tx_shift_reg[0],   tx_shift_reg[1]};
          5'd2 : tx_shift_reg[2:0]  <= {tx_shift_reg[1:0],  tx_shift_reg[2]};
          5'd3 : tx_shift_reg[3:0]  <= {tx_shift_reg[2:0],  tx_shift_reg[3]};
          5'd4 : tx_shift_reg[4:0]  <= {tx_shift_reg[3:0],  tx_shift_reg[4]};
          5'd5 : tx_shift_reg[5:0]  <= {tx_shift_reg[4:0],  tx_shift_reg[5]};
          5'd6 : tx_shift_reg[6:0]  <= {tx_shift_reg[5:0],  tx_shift_reg[6]};
          5'd7 : tx_shift_reg[7:0]  <= {tx_shift_reg[6:0],  tx_shift_reg[7]};
          5'd8 : tx_shift_reg[8:0]  <= {tx_shift_reg[7:0],  tx_shift_reg[8]};
          5'd9 : tx_shift_reg[9:0]  <= {tx_shift_reg[8:0],  tx_shift_reg[9]};
          5'd10: tx_shift_reg[10:0] <= {tx_shift_reg[9:0],  tx_shift_reg[10]};
          5'd11: tx_shift_reg[11:0] <= {tx_shift_reg[10:0], tx_shift_reg[11]};
          5'd12: tx_shift_reg[12:0] <= {tx_shift_reg[11:0], tx_shift_reg[12]};
          5'd13: tx_shift_reg[13:0] <= {tx_shift_reg[12:0], tx_shift_reg[13]};
          5'd14: tx_shift_reg[14:0] <= {tx_shift_reg[13:0], tx_shift_reg[14]};
          5'd15: tx_shift_reg[15:0] <= {tx_shift_reg[14:0], tx_shift_reg[15]};
          5'd16: tx_shift_reg[16:0] <= {tx_shift_reg[15:0], tx_shift_reg[16]};
          5'd17: tx_shift_reg[17:0] <= {tx_shift_reg[16:0], tx_shift_reg[17]};
          5'd18: tx_shift_reg[18:0] <= {tx_shift_reg[17:0], tx_shift_reg[18]};
          5'd19: tx_shift_reg[19:0] <= {tx_shift_reg[18:0], tx_shift_reg[19]};
          5'd20: tx_shift_reg[20:0] <= {tx_shift_reg[19:0], tx_shift_reg[20]};
          5'd21: tx_shift_reg[21:0] <= {tx_shift_reg[20:0], tx_shift_reg[21]};
          5'd22: tx_shift_reg[22:0] <= {tx_shift_reg[21:0], tx_shift_reg[22]};
          5'd23: tx_shift_reg[23:0] <= {tx_shift_reg[22:0], tx_shift_reg[23]};
          5'd24: tx_shift_reg[24:0] <= {tx_shift_reg[23:0], tx_shift_reg[24]};
          5'd25: tx_shift_reg[25:0] <= {tx_shift_reg[24:0], tx_shift_reg[25]};
          5'd26: tx_shift_reg[26:0] <= {tx_shift_reg[25:0], tx_shift_reg[26]};
          5'd27: tx_shift_reg[27:0] <= {tx_shift_reg[26:0], tx_shift_reg[27]};
          5'd28: tx_shift_reg[28:0] <= {tx_shift_reg[27:0], tx_shift_reg[28]};
          5'd29: tx_shift_reg[29:0] <= {tx_shift_reg[28:0], tx_shift_reg[29]};
          5'd30: tx_shift_reg[30:0] <= {tx_shift_reg[29:0], tx_shift_reg[30]};
          default: tx_shift_reg[31:0] <= {tx_shift_reg[30:0], tx_shift_reg[31]};
        endcase
      end else if (load_tx_buffer == 1'b1) begin
        tx_shift_reg <= tx_buffer;
      end
    end
  end

  // -- txd output
  //  registered; takes the bit of the shift register selected by dfs.
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      txd <= 1'b0;
    end else begin
      if (load_start_bit_ir == 1'b1) begin
        txd <= tx_shift_reg[15];
      end else if (load_tx_shift == 1'b1) begin
        case (tx_dfs)
          5'd0 : txd <= tx_pop_data[0];
          5'd1 : txd <= tx_pop_data[1];
          5'd2 : txd <= tx_pop_data[2];
          5'd3 : txd <= tx_pop_data[3];
          5'd4 : txd <= tx_pop_data[4];
          5'd5 : txd <= tx_pop_data[5];
          5'd6 : txd <= tx_pop_data[6];
          5'd7 : txd <= tx_pop_data[7];
          5'd8 : txd <= tx_pop_data[8];
          5'd9 : txd <= tx_pop_data[9];
          5'd10: txd <= tx_pop_data[10];
          5'd11: txd <= tx_pop_data[11];
          5'd12: txd <= tx_pop_data[12];
          5'd13: txd <= tx_pop_data[13];
          5'd14: txd <= tx_pop_data[14];
          5'd15: txd <= tx_pop_data[15];
          5'd16: txd <= tx_pop_data[16];
          5'd17: txd <= tx_pop_data[17];
          5'd18: txd <= tx_pop_data[18];
          5'd19: txd <= tx_pop_data[19];
          5'd20: txd <= tx_pop_data[20];
          5'd21: txd <= tx_pop_data[21];
          5'd22: txd <= tx_pop_data[22];
          5'd23: txd <= tx_pop_data[23];
          5'd24: txd <= tx_pop_data[24];
          5'd25: txd <= tx_pop_data[25];
          5'd26: txd <= tx_pop_data[26];
          5'd27: txd <= tx_pop_data[27];
          5'd28: txd <= tx_pop_data[28];
          5'd29: txd <= tx_pop_data[29];
          5'd30: txd <= tx_pop_data[30];
          default: txd <= tx_pop_data[31];
        endcase
      end else if (load_tx_buffer == 1'b1) begin
        case (dfs)
          5'd3 : txd <= tx_buffer[3];
          5'd4 : txd <= tx_buffer[4];
          5'd5 : txd <= tx_buffer[5];
          5'd6 : txd <= tx_buffer[6];
          5'd7 : txd <= tx_buffer[7];
          5'd8 : txd <= tx_buffer[8];
          5'd9 : txd <= tx_buffer[9];
          5'd10: txd <= tx_buffer[10];
          5'd11: txd <= tx_buffer[11];
          5'd12: txd <= tx_buffer[12];
          5'd13: txd <= tx_buffer[13];
          5'd14: txd <= tx_buffer[14];
          5'd15: txd <= tx_buffer[15];
          5'd16: txd <= tx_buffer[16];
          5'd17: txd <= tx_buffer[17];
          5'd18: txd <= tx_buffer[18];
          5'd19: txd <= tx_buffer[19];
          5'd20: txd <= tx_buffer[20];
          5'd21: txd <= tx_buffer[21];
          5'd22: txd <= tx_buffer[22];
          5'd23: txd <= tx_buffer[23];
          5'd24: txd <= tx_buffer[24];
          5'd25: txd <= tx_buffer[25];
          5'd26: txd <= tx_buffer[26];
          5'd27: txd <= tx_buffer[27];
          5'd28: txd <= tx_buffer[28];
          5'd29: txd <= tx_buffer[29];
          5'd30: txd <= tx_buffer[30];
          default: txd <= tx_buffer[31];
        endcase
      end else if (tx_shift_en == 1'b1) begin
        case (tx_dfs)
          5'd1 : txd <= tx_shift_reg[0];
          5'd2 : txd <= tx_shift_reg[1];
          5'd3 : txd <= tx_shift_reg[2];
          5'd4 : txd <= tx_shift_reg[3];
          5'd5 : txd <= tx_shift_reg[4];
          5'd6 : txd <= tx_shift_reg[5];
          5'd7 : txd <= tx_shift_reg[6];
          5'd8 : txd <= tx_shift_reg[7];
          5'd9 : txd <= tx_shift_reg[8];
          5'd10: txd <= tx_shift_reg[9];
          5'd11: txd <= tx_shift_reg[10];
          5'd12: txd <= tx_shift_reg[11];
          5'd13: txd <= tx_shift_reg[12];
          5'd14: txd <= tx_shift_reg[13];
          5'd15: txd <= tx_shift_reg[14];
          5'd16: txd <= tx_shift_reg[15];
          5'd17: txd <= tx_shift_reg[16];
          5'd18: txd <= tx_shift_reg[17];
          5'd19: txd <= tx_shift_reg[18];
          5'd20: txd <= tx_shift_reg[19];
          5'd21: txd <= tx_shift_reg[20];
          5'd22: txd <= tx_shift_reg[21];
          5'd23: txd <= tx_shift_reg[22];
          5'd24: txd <= tx_shift_reg[23];
          5'd25: txd <= tx_shift_reg[24];
          5'd26: txd <= tx_shift_reg[25];
          5'd27: txd <= tx_shift_reg[26];
          5'd28: txd <= tx_shift_reg[27];
          5'd29: txd <= tx_shift_reg[28];
          5'd30: txd <= tx_shift_reg[29];
          default: txd <= tx_shift_reg[30];
        endcase
      end
    end
  end

  // -- start bit delay register
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      load_start_bit_ir <= 1'b0;
    end else begin
      load_start_bit_ir <= load_start_bit;
    end
  end

  // -- RX shift register
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      rx_shift_reg <= 32'b0;
    end else if (i_rx_shift_en == 1'b1) begin
      rx_shift_reg <= {rx_shift_reg[30:0], rxd_mst_int};
    end
  end

  // -- RX buffer (zero padded)
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      rx_buffer <= 32'b0;
    end else if (i_load_rx_buf == 1'b1) begin
      case (dfs)
        5'd0 : rx_buffer <= {{31{1'b0}}, rx_shift_reg[0]};
        5'd1 : rx_buffer <= {{30{1'b0}}, rx_shift_reg[1:0]};
        5'd2 : rx_buffer <= {{29{1'b0}}, rx_shift_reg[2:0]};
        5'd3 : rx_buffer <= {{28{1'b0}}, rx_shift_reg[3:0]};
        5'd4 : rx_buffer <= {{27{1'b0}}, rx_shift_reg[4:0]};
        5'd5 : rx_buffer <= {{26{1'b0}}, rx_shift_reg[5:0]};
        5'd6 : rx_buffer <= {{25{1'b0}}, rx_shift_reg[6:0]};
        5'd7 : rx_buffer <= {{24{1'b0}}, rx_shift_reg[7:0]};
        5'd8 : rx_buffer <= {{23{1'b0}}, rx_shift_reg[8:0]};
        5'd9 : rx_buffer <= {{22{1'b0}}, rx_shift_reg[9:0]};
        5'd10: rx_buffer <= {{21{1'b0}}, rx_shift_reg[10:0]};
        5'd11: rx_buffer <= {{20{1'b0}}, rx_shift_reg[11:0]};
        5'd12: rx_buffer <= {{19{1'b0}}, rx_shift_reg[12:0]};
        5'd13: rx_buffer <= {{18{1'b0}}, rx_shift_reg[13:0]};
        5'd14: rx_buffer <= {{17{1'b0}}, rx_shift_reg[14:0]};
        5'd15: rx_buffer <= {{16{1'b0}}, rx_shift_reg[15:0]};
        5'd16: rx_buffer <= {{15{1'b0}}, rx_shift_reg[16:0]};
        5'd17: rx_buffer <= {{14{1'b0}}, rx_shift_reg[17:0]};
        5'd18: rx_buffer <= {{13{1'b0}}, rx_shift_reg[18:0]};
        5'd19: rx_buffer <= {{12{1'b0}}, rx_shift_reg[19:0]};
        5'd20: rx_buffer <= {{11{1'b0}}, rx_shift_reg[20:0]};
        5'd21: rx_buffer <= {{10{1'b0}}, rx_shift_reg[21:0]};
        5'd22: rx_buffer <= {{9{1'b0}}, rx_shift_reg[22:0]};
        5'd23: rx_buffer <= {{8{1'b0}}, rx_shift_reg[23:0]};
        5'd24: rx_buffer <= {{7{1'b0}}, rx_shift_reg[24:0]};
        5'd25: rx_buffer <= {{6{1'b0}}, rx_shift_reg[25:0]};
        5'd26: rx_buffer <= {{5{1'b0}}, rx_shift_reg[26:0]};
        5'd27: rx_buffer <= {{4{1'b0}}, rx_shift_reg[27:0]};
        5'd28: rx_buffer <= {{3{1'b0}}, rx_shift_reg[28:0]};
        5'd29: rx_buffer <= {{2{1'b0}}, rx_shift_reg[29:0]};
        5'd30: rx_buffer <= {{1{1'b0}}, rx_shift_reg[30:0]};
        default: rx_buffer <= rx_shift_reg[31:0];
      endcase
    end
  end

  assign rx_push_data = rx_buffer;

endmodule // spi_m_shift
