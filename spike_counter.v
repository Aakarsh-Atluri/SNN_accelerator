`timescale 1ns / 1ps
module spike_counter #(
    parameter T_WINDOW = 25    // Number of timesteps in the rate-coding window
)(
    input  wire        clk,
    input  wire        rst_n,    // Active-low synchronous reset
    input  wire        start,    // Pulse to begin a new classification window
    input  wire        spike_in, // LIF spike output, valid each timestep
    input  wire        spike_valid, // spike_in is valid this cycle
    output reg [5:0]   count_1,  // Number of spike=1 timesteps seen
    output reg [5:0]   count_0,  // Number of spike=0 timesteps seen
    output reg         result,   // 1 = majority spikes, 0 = majority silence
    output reg         done      // Pulses 1 cycle when result is ready
);

    // Sized tracking register (5 bits can hold 0 to 31; 25 fits safely)
    reg [4:0] timestamp;   

    // Safely cast the 32-bit parameter calculation into a sized 5-bit localparam
    localparam [4:0] MAX_TIMESTAMP = T_WINDOW[4:0] - 5'd1;

    localparam IDLE  = 1'b0;
    localparam COUNT = 1'b1;
    reg state;

    always @(posedge clk) begin
        if (!rst_n) begin
            timestamp <= 5'd0;
            count_1   <= 6'd0;
            count_0   <= 6'd0;
            result    <= 1'b0;
            done      <= 1'b0;
            state     <= IDLE;
        end else begin
            done <= 1'b0;  // Default pulse clear

            case (state)
                IDLE: begin
                    if (start) begin
                        count_1   <= 6'd0;
                        count_0   <= 6'd0;
                        timestamp <= 5'd0;
                        state     <= COUNT;
                    end
                end

                COUNT: begin
                    if (spike_valid) begin
                        if (timestamp < MAX_TIMESTAMP) begin
                            // Accumulate spike statistics with sized increments
                            if (spike_in) count_1 <= count_1 + 6'd1;
                            else          count_0 <= count_0 + 6'd1;
                            timestamp <= timestamp + 5'd1;
                        end else begin
                            // Final timestep: latch last sample and decide
                            if (spike_in) count_1 <= count_1 + 6'd1;
                            else          count_0 <= count_0 + 6'd1;
                            
                            // Lookahead comparison matching non-blocking behavior using sized 6-bit literals
                            result <= (count_1 + (spike_in ? 6'd1 : 6'd0) > count_0 + (spike_in ? 6'd0 : 6'd1))
                                      ? 1'b1 : 1'b0;
                                      
                            done  <= 1'b1;
                            state <= IDLE;
                        end
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule
