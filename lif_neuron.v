`timescale 1ns / 1ps

module lif_neuron #(
    parameter THRESHOLD  = 16'sd1000,  // Firing threshold (signed)
    parameter LEAK_SHIFT = 3,          // Leakage = state >> LEAK_SHIFT (divide by 8)
    parameter REF_PERIOD = 3'd5        // Refractory cycles after firing
)(
    input  wire        clk,
    input  wire        rst_n,          // Active-low synchronous reset
    // Input from cascaded_adder
    input  wire signed [15:0] current_in,   // Weighted spike sum
    input  wire               current_valid, // current_in is valid this cycle
    // Output
    output reg         spike_out        // 1 = neuron fired this cycle
);

    reg signed [15:0] membrane;        // Membrane potential
    reg [2:0]         ref_counter;     // Refractory countdown

    always @(posedge clk) begin
        if (!rst_n) begin
            membrane    <= 16'sd0;
            ref_counter <= 3'd0;
            spike_out   <= 1'b0;
        end else begin
            spike_out <= 1'b0;  // Default: no spike

            if (ref_counter > 0) begin
                // In refractory period: hold membrane at zero, count down
                ref_counter <= ref_counter - 1;
                membrane    <= 16'sd0;
            end else if (current_valid) begin
                // Leaky integration: V(t+1) = V(t) - V(t)/tau + I(t)
                // Leakage implemented as arithmetic right-shift (divide by 2^LEAK_SHIFT)
                if (membrane + current_in - (membrane >>> LEAK_SHIFT) >= THRESHOLD) begin
                    // Threshold crossed: fire and reset
                    spike_out   <= 1'b1;
                    membrane    <= 16'sd0;
                    ref_counter <= REF_PERIOD;
                end else begin
                    membrane <= membrane - (membrane >>> LEAK_SHIFT) + current_in;
                end
            end
        end
    end

endmodule
