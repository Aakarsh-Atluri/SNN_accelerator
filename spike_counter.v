`timescale 1ns / 1ps
module spike_counter #(
    parameter T_WINDOW = 25    // Number of timesteps in the rate-coding window
)(
    input  wire       clk,
    input  wire       rst_n,    // Active-low synchronous reset
    input  wire       start,    // Pulse to begin a new classification window
    input  wire       spike_in, // LIF spike output, valid each timestep
    input  wire       spike_valid, // spike_in is valid this cycle
    output reg [5:0]  count_1,  // Number of spike=1 timesteps seen
    output reg [5:0]  count_0,  // Number of spike=0 timesteps seen
    output reg        result,   // 1 = majority spikes, 0 = majority silence
    output reg        done      // Pulses 1 cycle when result is ready
);

    reg [4:0] timestamp;   // Current timestep index within window

    localparam IDLE  = 1'b0;
    localparam COUNT = 1'b1;
    reg state;

    always @(posedge clk) begin
        if (!rst_n) begin
            timestamp <= 0;
            count_1   <= 0;
            count_0   <= 0;
            result    <= 0;
            done      <= 0;
            state     <= IDLE;
        end else begin
            done <= 0;  // Default: not done

            case (state)
                IDLE: begin
                    if (start) begin
                        count_1   <= 0;
                        count_0   <= 0;
                        timestamp <= 0;
                        state     <= COUNT;
                    end
                end

                COUNT: begin
                    if (spike_valid) begin
                        if (timestamp < T_WINDOW - 1) begin
                            // Accumulate spike statistics
                            if (spike_in) count_1 <= count_1 + 1;
                            else          count_0 <= count_0 + 1;
                            timestamp <= timestamp + 1;
                        end else begin
                            // Final timestep: latch last sample and decide
                            if (spike_in) count_1 <= count_1 + 1;
                            else          count_0 <= count_0 + 1;
                            result <= (count_1 + spike_in > count_0 + ~spike_in)
                                      ? 1'b1 : 1'b0;
                            done  <= 1'b1;
                            state <= IDLE;
                        end
                    end
                end
            endcase
        end
    end

endmodule
