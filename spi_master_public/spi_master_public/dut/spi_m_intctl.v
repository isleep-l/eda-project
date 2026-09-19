// File    : spi_m_intctl.v
// Abstract: spi_master_public - Interrupt control.
//           6 interrupt sources (mst collision, rx full, rx overflow,
//           rx underflow, tx overflow, tx empty) with raw / masked
//           status registers and read-to-clear semantics.
//
//   RISR[5] = multi-master collision   (clear: read ICR / MSTICR)
//   RISR[4] = RX FIFO full             (hardware clear)
//   RISR[3] = RX FIFO overflow         (clear: read ICR / RXOICR)
//   RISR[2] = RX FIFO underflow        (clear: read ICR / RXUICR)
//   RISR[1] = TX FIFO overflow         (clear: read ICR / TXOICR)
//   RISR[0] = TX FIFO empty            (hardware clear)
//
//   ISR = IMR & RISR ; interrupt outputs are active LOW.
module spi_m_intctl
  (
   input         clk,
   input         resetn,
   input  [5:0]  imr,
   input         rdec_rxoicr_en,
   input         rdec_txoicr_en,
   input         rdec_rxuicr_en,
   input         rdec_msticr_en,
   input         rdec_icr_en,
   input         rd_en,
   input         tx_empty,
   input         tx_overflow,
   input         rx_underflow,
   input         rx_full,
   input         rx_overflow,
   input         multi_mst_sync,
   input         set0_imr5,    // AHB write clears (masks) contention irq
   input         set1_imr5,    // AHB write sets (unmasks) contention irq
   input         ssi_en,
   output        ssi_txe_intr,
   output        ssi_txo_intr,
   output        ssi_rxu_intr,
   output        ssi_rxf_intr,
   output        ssi_rxo_intr,
   output        ssi_mst_intr,
   output reg    mst_contention,
   output [5:0]  risr,
   output reg [5:0] isr
   );

  reg irisr_rx_full;
  reg irisr_rx_fifo_overflow;
  reg irisr_rx_fifo_underflow;
  reg irisr_tx_fifo_overflow;
  reg irisr_tx_empty;
  reg irisr_mst_collision;

  // -- TX empty interrupt (hardware controlled)
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      irisr_tx_empty <= 1'b0;
    end else if (ssi_en == 1'b0) begin
      irisr_tx_empty <= 1'b0;
    end else begin
      irisr_tx_empty <= tx_empty;
    end
  end

  // -- TX overflow interrupt
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      irisr_tx_fifo_overflow <= 1'b0;
    end else if (ssi_en == 1'b0) begin
      irisr_tx_fifo_overflow <= 1'b0;
    end else if (tx_overflow == 1'b1) begin
      irisr_tx_fifo_overflow <= 1'b1;
    end else if ((rdec_icr_en | rdec_txoicr_en) & rd_en) begin
      irisr_tx_fifo_overflow <= 1'b0;
    end
  end

  // -- RX underflow interrupt
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      irisr_rx_fifo_underflow <= 1'b0;
    end else if (ssi_en == 1'b0) begin
      irisr_rx_fifo_underflow <= 1'b0;
    end else if (rx_underflow == 1'b1) begin
      irisr_rx_fifo_underflow <= 1'b1;
    end else if ((rdec_icr_en | rdec_rxuicr_en) & rd_en) begin
      irisr_rx_fifo_underflow <= 1'b0;
    end
  end

  // -- RX overflow interrupt
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      irisr_rx_fifo_overflow <= 1'b0;
    end else if (ssi_en == 1'b0) begin
      irisr_rx_fifo_overflow <= 1'b0;
    end else if (rx_overflow == 1'b1) begin
      irisr_rx_fifo_overflow <= 1'b1;
    end else if ((rdec_icr_en | rdec_rxoicr_en) & rd_en) begin
      irisr_rx_fifo_overflow <= 1'b0;
    end
  end

  // -- RX full interrupt (hardware controlled)
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      irisr_rx_full <= 1'b0;
    end else if (ssi_en == 1'b0) begin
      irisr_rx_full <= 1'b0;
    end else begin
      irisr_rx_full <= rx_full;
    end
  end

  // -- Master collision interrupt
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      irisr_mst_collision <= 1'b0;
    end else if (ssi_en == 1'b0) begin
      irisr_mst_collision <= 1'b0;
    end else if (multi_mst_sync == 1'b1) begin
      irisr_mst_collision <= 1'b1;
    end else if ((rdec_icr_en | rdec_msticr_en) & rd_en) begin
      irisr_mst_collision <= 1'b0;
    end
  end

  wire [5:0] irisr;
  assign irisr[5] = irisr_mst_collision;
  assign irisr[4] = irisr_rx_full;
  assign irisr[3] = irisr_rx_fifo_overflow;
  assign irisr[2] = irisr_rx_fifo_underflow;
  assign irisr[1] = irisr_tx_fifo_overflow;
  assign irisr[0] = irisr_tx_empty;

  assign risr = irisr;

  // -- masked interrupt status
  always @(*) begin
    isr = imr & risr;
  end

  // -- interrupt outputs (active low)
  assign ssi_txe_intr = !isr[0];
  assign ssi_txo_intr = !isr[1];
  assign ssi_rxu_intr = !isr[2];
  assign ssi_rxo_intr = !isr[3];
  assign ssi_rxf_intr = !isr[4];
  assign ssi_mst_intr = !isr[5];

  // -- master contention (stop transmit on collision)
  always @(posedge clk or negedge resetn) begin
    if (resetn == 1'b0) begin
      mst_contention <= 1'b0;
    end else if (ssi_en == 1'b0) begin
      mst_contention <= 1'b0;
    end else if ((multi_mst_sync | irisr_mst_collision) &
                 (imr[5] | set1_imr5) & (~set0_imr5)) begin
      mst_contention <= 1'b1;
    end else if (((rdec_icr_en | rdec_msticr_en) & rd_en) ||
                 (irisr_mst_collision == 1'b0) || set0_imr5) begin
      mst_contention <= 1'b0;
    end
  end

endmodule // spi_m_intctl
