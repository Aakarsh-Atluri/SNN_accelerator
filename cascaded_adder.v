module cascaded_adder #(
    parameter N      = 4096,   // number of neurons / weights
    parameter W      = 16,     // weight / bias bit-width
    parameter OUT    = 32,     // accumulator / result bit-width
    parameter ADDR_W = 12      // 2^12=4096
)(
    input  wire                  clk,
    input  wire                  rst,
    input  wire                  start,
    output reg                   busy,
    output reg                   valid,

    input  wire                  spike_in,
    input  wire                  spike_valid,

    output reg  [ADDR_W-1:0]     weight_addr,
    input  wire signed [W-1:0]   weight_data,
    input  wire signed [W-1:0]   bias,

    output reg  signed [OUT-1:0] result
);

// ---------------------------------------------------------------------------
// Internal registers
// ---------------------------------------------------------------------------
reg signed [OUT-1:0] acc;
reg [ADDR_W:0]       issue_count;    // one extra bit to safely reach N
reg [ADDR_W:0]       consume_count;

// Two-stage spike pipeline – needed because BRAM has 1-cycle read latency.
// Stage 1: registered alongside address presentation
// Stage 2: aligned with arriving weight_data → used in accumulation
reg spike_pipe_s1;
reg spike_pipe_s2;
reg spike_pipe_v1;  // valid flags
reg spike_pipe_v2;

// ---------------------------------------------------------------------------
// Combinational: sign-extend weight and bias; compute next accumulator value
// ---------------------------------------------------------------------------
reg signed [OUT-1:0] weight_ext;
reg signed [OUT-1:0] bias_ext;
reg signed [OUT-1:0] next_acc;

always @(*) begin
    weight_ext = {{(OUT-W){weight_data[W-1]}}, weight_data};
    bias_ext   = {{(OUT-W){bias[W-1]}},        bias};
    next_acc   = acc;
    if (spike_pipe_v2 && spike_pipe_s2)
        next_acc = acc + weight_ext;
end

// ---------------------------------------------------------------------------
// Address truncation (avoids non-constant part-select)
// ---------------------------------------------------------------------------
wire [ADDR_W-1:0] issue_addr;
assign issue_addr = issue_count[ADDR_W-1:0];

// ---------------------------------------------------------------------------
// Main sequential logic
// ---------------------------------------------------------------------------
    always @(negedge clk or negedge rst) begin
    if (rst) begin
        acc           <= {OUT{1'b0}};
        result        <= {OUT{1'b0}};
        issue_count   <= {(ADDR_W+1){1'b0}};
        consume_count <= {(ADDR_W+1){1'b0}};
        weight_addr   <= {ADDR_W{1'b0}};
        spike_pipe_s1 <= 1'b0;
        spike_pipe_s2 <= 1'b0;
        spike_pipe_v1 <= 1'b0;
        spike_pipe_v2 <= 1'b0;
        busy          <= 1'b0;
        valid         <= 1'b0;
    end else begin
        valid <= 1'b0;   // single-cycle pulse

        // ------------------------------------------------------------------
        // START
        // ------------------------------------------------------------------
        if (start && !busy) begin
            acc           <= {OUT{1'b0}};
            issue_count   <= {(ADDR_W+1){1'b0}};
            consume_count <= {(ADDR_W+1){1'b0}};
            weight_addr   <= {ADDR_W{1'b0}};
            spike_pipe_s1 <= 1'b0;
            spike_pipe_s2 <= 1'b0;
            spike_pipe_v1 <= 1'b0;
            spike_pipe_v2 <= 1'b0;
            busy          <= 1'b1;
        end

        // ------------------------------------------------------------------
        // RUNNING
        // ------------------------------------------------------------------
        else if (busy) begin

            // ---------------------------------------------------------------
            // STAGE 3 – accumulate (weight_data now aligned with spike_pipe_s2)
            // ---------------------------------------------------------------
            spike_pipe_v2 <= spike_pipe_v1;
            spike_pipe_s2 <= spike_pipe_s1;

            if (spike_pipe_v2) begin
                acc           <= next_acc;
                consume_count <= consume_count + 1'b1;

                if (consume_count == N - 1) begin
                    result <= next_acc + bias_ext;
                    valid  <= 1'b1;
                    busy   <= 1'b0;
                end
            end

            // ---------------------------------------------------------------
            // STAGE 1 – present address to BRAM; latch spike into stage-1 pipe
            // ---------------------------------------------------------------
            if (spike_valid && (issue_count < N)) begin
                weight_addr   <= issue_addr;   // BRAM will respond next cycle
                spike_pipe_s1 <= spike_in;
                spike_pipe_v1 <= 1'b1;
                issue_count   <= issue_count + 1'b1;
            end else begin
                spike_pipe_v1 <= 1'b0;
            end

        end // busy
    end // !rst
end

endmodule
