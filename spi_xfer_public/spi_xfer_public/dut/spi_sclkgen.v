// File    : spi_sclkgen.v
// Abstract: spi_xfer_public serial clock generator (master mode).
//           Divides ssi_clk by BAUDR to produce sclk_out and the
//           sclk rising/falling edge pulses consumed by the master FSM.
//
//           - ssi_cnt counts 0..BAUDR-1 and wraps.
//           - sclk_out follows the SCPOL idle polarity when the
//             generator is idle (sclk_mask==0), a transfer is not
//             running, or BAUDR==0 (divider disabled).
//           - sclk_out is driven to the active level when
//             ssi_cnt == BAUDR/2 and to the idle level at counter wrap.
//           - sclk_re pulses when ssi_cnt == BAUDR/2 - 1
//           - sclk_fe pulses when ssi_cnt == BAUDR - 1
module spi_sclkgen
  (
   input  wire        clk,        // ssi_clk
   input  wire        rst_n,      // async reset, active low
   input  wire        ssi_en,     // SSI enable
   input  wire [15:0] baudr,      // baud rate divisor
   input  wire        sclk_mask,  // serial clock mask from FSM
   input  wire        sclk_active,// SCPOL - active level of sclk_out
   output reg         sclk_out,   // serial clock output
   output wire        sclk_re,    // sclk rising edge pulse
   output wire        sclk_fe,    // sclk falling edge pulse
   output wire [15:0] ssi_cnt_out // divider counter (observation)
   );

  reg  [15:0] ssi_cnt;
  reg         sclk_re_ir;
  reg         sclk_fe_ir;

  wire [15:0] r_det = (baudr >> 1) - 16'd1;   // rising edge detect level
  wire [15:0] f_det = baudr - 16'd1;          // falling edge detect level

  // -- ssi_clk divider counter
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ssi_cnt <= 16'd0;
    end else begin
      if (!ssi_en) begin
        ssi_cnt <= 16'd0;
      end else if (ssi_cnt == (baudr - 16'd1)) begin
        ssi_cnt <= 16'd0;
      end else begin
        ssi_cnt <= ssi_cnt + 16'd1;
      end
    end
  end

  // -- sclk_out generation
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sclk_out <= ~sclk_active;
    end else begin
      if (!ssi_en) begin
        sclk_out <= ~sclk_active;
      end else if (baudr == 16'd0 || sclk_mask == 1'b0 || ssi_cnt == 16'd0) begin
        sclk_out <= ~sclk_active;
      end else if (ssi_cnt == (baudr >> 1)) begin
        sclk_out <= sclk_active;
      end
    end
  end

  // -- sclk edge detection
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sclk_re_ir <= 1'b0;
      sclk_fe_ir <= 1'b0;
    end else begin
      if (!ssi_en) begin
        sclk_re_ir <= 1'b0;
        sclk_fe_ir <= 1'b0;
      end else begin
        sclk_re_ir <= (ssi_cnt == r_det);
        sclk_fe_ir <= (ssi_cnt == f_det);
      end
    end
  end

  assign sclk_re      = sclk_re_ir;
  assign sclk_fe      = sclk_fe_ir;
  assign ssi_cnt_out  = ssi_cnt;

endmodule // spi_sclkgen
