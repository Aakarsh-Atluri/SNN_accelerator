`timescale 1ns / 1ps
module cascaded_adder #(
    parameter N   = 4096,  // Number of input neurons
    parameter W   = 16,    // Weight/bias bit-width (signed)
    parameter OUT = 28     // Accumulator output width (must hold N * 2^W)
)(
    input  wire                  clk,
    input  wire                  rst_n,       // Active-low synchronous reset
    input  wire                  start,       // Pulse to begin a new inference
    // Serial spike input from spike_unpacker
    input  wire                  spike_in,    // Current spike bit (0 or 1)
    input  wire                  spike_valid, // spike_in is valid this cycle
    // Weight read port (connects to weight BRAM)
    output reg  [12:0]           weight_addr, // Address to read current weight
    input  wire signed [W-1:0]   weight_data, // Weight returned from BRAM
    // Bias (can be stored in a small register file or hardcoded per neuron)
    input  wire signed [W-1:0]   bias,
    // Outputs
    output reg  signed [OUT-1:0] result,
    output reg                   valid        // Pulses high for 1 cycle when result ready
);

    reg signed [OUT-1:0] acc;
    reg [12:0]           counter;   // counts 0..N-1
    reg                  running;

    // Sign-extend weight to OUT bits for accumulation
    wire signed [OUT-1:0] weight_extended = {{(OUT-W){weight_data[W-1]}}, weight_data};

    always @(posedge clk) begin
        if (!rst_n) begin
            acc         <= 0;
            result      <= 0;
            counter     <= 0;
            weight_addr <= 0;
            running     <= 0;
            valid       <= 0;
        end else begin
            valid <= 0;  // Default: not valid

            // Start a new inference pass
            if (start && !running) begin
                acc         <= 0;
                counter     <= 0;
                weight_addr <= 0;
                running     <= 1;
            end

            if (running && spike_valid) begin
                if (counter < N) begin
                    // Accumulate: add weight only when spike=1
                    if (spike_in)
                        acc <= acc + weight_extended;
                    counter     <= counter + 1;
                    weight_addr <= weight_addr + 1;
                end else begin
                    // All spikes processed: add bias and output result
                    result  <= acc + {{(OUT-W){bias[W-1]}}, bias};
                    valid   <= 1;
                    running <= 0;
                end
            end
        end
    end

endmodule
