// File    : spi_m_sclkgen.v
// Abstract: spi_master_public - Serial clock generator.
//           Generates sclk_out and rising/falling edge pulses for the
//           SPI / SSP / Microwire master.
//
//           sclk period = baudr ssi_clk cycles.  The counter wraps at
//           baudr-1.  sclk_out toggles active at the half-period point
//           (ssi_cnt[14:0]==baudr[15:1]).  Idle level is !sclk_active.
module spi_m_sclkgen
  (
   input        clk,
   input        rst_n,
   input [15:0] baudr,
   input        sclk_mask,   // FSM clock mask
   input        sclk_active, // active level (registered)
   input        ssi_en,      // SSI enable
   output reg   sclk_out,
   output       sclk_re,
   output       sclk_fe
   );

  wire        sclk_active_sync;
  wire [14:0] r_det;
  wire [15:0] f_det;
  reg  [15:0] ssi_cnt;
  reg         sclk_re_ir;
  reg         sclk_fe_ir;

  // -- ssi_clk counter
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      ssi_cnt <= 16'b0;
    end else begin
      if (ssi_en == 1'b0) begin
        ssi_cnt <= 16'b0;
      end else if (ssi_cnt == (baudr - 16'd1)) begin
        ssi_cnt <= 16'b0;
      end else begin
        ssi_cnt <= ssi_cnt + 16'd1;
      end
    end
  end

  // -- sclk_out
  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      sclk_out <= !sclk_active_sync;
    end else begin
      if (ssi_en == 1'b0) begin
        sclk_out <= !sclk_active_sync;
      end else if (baudr == 16'b0 || sclk_mask == 1'b0 || ssi_cnt == 16'b0) begin
        sclk_out <= !sclk_active_sync;
      end else if (ssi_cnt[14:0] == baudr[15:1]) begin
        sclk_out <= sclk_active_sync;
      end
    end
  end

  // -- edge detect levels
  assign r_det = baudr[15:1] - 15'd1;
  assign f_det = baudr - 16'd1;

  always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
      sclk_re_ir <= 1'b0;
      sclk_fe_ir <= 1'b0;
    end else begin
      if (ssi_en == 1'b0) begin
        sclk_re_ir <= 1'b0;
        sclk_fe_ir <= 1'b0;
      end else begin
        sclk_re_ir <= (ssi_cnt == {1'b0, r_det});
        sclk_fe_ir <= (ssi_cnt == f_det);
      end
    end
  end

  assign sclk_re = sclk_re_ir;
  assign sclk_fe = sclk_fe_ir;
  assign sclk_active_sync = sclk_active;

endmodule // spi_m_sclkgen
