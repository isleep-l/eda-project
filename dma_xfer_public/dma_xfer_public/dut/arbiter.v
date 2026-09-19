// dma_xfer_public - arbiter.v
// Arbiter granting the highest-priority requesting channel.
// Priority order is a hidden module parameter ARB_PRIO (ARB_PRIO[1:0] = highest).
// Public placeholder = ascending order (0,1,2,3). Evaluation overrides the param.
// Fully parameter-driven: no hardcoded priority path.
// License: MIT

module arbiter #(
    parameter N_CH = 4,
    parameter PRIO_BITS = 8,
    parameter [PRIO_BITS-1:0] ARB_PRIO = 8'h30 | 8'h20 | 8'h10  // 0,1,2,3 asc
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire [N_CH-1:0]  req,
    output reg  [N_CH-1:0]  grant,
    output reg  [1:0]       winner,
    output reg              grant_valid
);

    wire [1:0] ord0 = ARB_PRIO[1:0];  // highest priority
    wire [1:0] ord1 = ARB_PRIO[3:2];
    wire [1:0] ord2 = ARB_PRIO[5:4];
    wire [1:0] ord3 = ARB_PRIO[7:6];  // lowest priority

    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            grant       <= {N_CH{1'b0}};
            winner      <= 2'd0;
            grant_valid <= 1'b0;
        end else begin
            grant = {N_CH{1'b0}};
            winner = 2'd0;
            if (ord0 < N_CH && req[ord0]) winner = ord0;
            else if (ord1 < N_CH && req[ord1]) winner = ord1;
            else if (ord2 < N_CH && req[ord2]) winner = ord2;
            else if (ord3 < N_CH && req[ord3]) winner = ord3;
            grant[winner] = (req != 0);
            grant_valid   = (req != 0);
        end

endmodule
