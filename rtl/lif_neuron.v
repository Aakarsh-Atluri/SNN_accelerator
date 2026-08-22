module lif_neuron #(
    parameter WIDTH      = 32,
    parameter THRESHOLD  = 100,
    parameter RESET_VAL  = 0,
    parameter LEAK_SHIFT = 3
)(
    input  wire clk,
    input  wire rst,

    // Input current
    input  wire signed [WIDTH-1:0] current_in,
    input  wire current_valid,

    // Outputs
    output reg spike_out,
    output reg spike_valid,

    // Debug visibility
    output reg signed [WIDTH-1:0] membrane_out
);

    // Internal Registers

    reg signed [WIDTH-1:0] membrane;
    reg signed [WIDTH-1:0] leak_value;
    reg signed [WIDTH-1:0] next_membrane;

    integer cycle;


    //using Combinational Logic for next state, fixed timing errors

    always @(*) begin

        // arithmetic shift preserves sign
        leak_value = membrane >>> LEAK_SHIFT;

        // default
        next_membrane = membrane - leak_value;

        // add input current
        if (current_valid)
            next_membrane = next_membrane + current_in;
    end

    // ============================================================
    // Sequential Logic
    // ============================================================

    always @(posedge clk or posedge rst) begin

        if (rst) begin

            membrane     <= RESET_VAL;
            membrane_out <= RESET_VAL;

            spike_out    <= 0;
            spike_valid  <= 0;

            cycle        <= 0;

        end else begin

            cycle <= cycle + 1;

            // default
            spike_valid <= 0;
            spike_out   <= 0;

            // Update only when "current_valid" arrives

            if (current_valid) begin

                $display(
                    "[LIF %0d] INPUT | membrane=%0d | leak=%0d | current=%0d | next_membrane=%0d",
                    cycle,
                    membrane,
                    leak_value,
                    current_in,
                    next_membrane
                );

                // Spike condition

                if (next_membrane >= THRESHOLD) begin

                    membrane <= RESET_VAL;

                    membrane_out <= RESET_VAL;

                    spike_out   <= 1;
                    spike_valid <= 1;

                    $display(
                        "[LIF %0d] SPIKE | threshold=%0d exceeded | membrane_reset=%0d",
                        cycle,
                        THRESHOLD,
                        RESET_VAL
                    );
                end


                // Normal membrane update

                else begin

                    membrane <= next_membrane;

                    membrane_out <= next_membrane;

                    spike_valid <= 1;

                    $display(
                        "[LIF %0d] UPDATE | new_membrane=%0d",
                        cycle,
                        next_membrane
                    );
                end
            end
        end
    end

endmodule
