`timescale 1ns / 1ps

module master_fsm #(
    parameter N        = 4096,
    parameter T_WINDOW = 25
)(
    input  wire clk,
    input  wire rst_n,

    // From fifo_uart_controller
    input  wire fifo_empty,
    input  wire weights_loaded,

    // From/to spike_unpacker
    input  wire spike_valid_in,   // spike_unpacker says a spike is ready
    input  wire spike_bit,        // the spike value

    // To cascaded_adder
    output reg  adder_start,

    // From cascaded_adder
    input  wire adder_valid,

    // To/from lif_neuron
    output reg  lif_current_valid, // Gate: present result to LIF this cycle
    input  wire lif_spike_out,     // LIF fired?

    // To spike_counter
    output reg  sc_start,          // Begin a new rate-window
    output reg  sc_spike_valid,    // Pass spike_valid to spike_counter
    input  wire sc_done,           // spike_counter finished
    input  wire sc_result,         // 0 or 1 classification

    // Result to UART TX
    output reg [7:0] tx_data,
    output reg       tx_send,      // Pulse: load tx_data and transmit

    // Status
    output reg inference_done,     // Pulses when a result is ready
    output reg class_out           // 0 = no collision, 1 = collision
);

    // -------------------------------------------------------------------------
    // State encoding
    // -------------------------------------------------------------------------
    localparam S_IDLE         = 4'd0;
    localparam S_WAIT_WEIGHTS = 4'd1;
    localparam S_WAIT_SPIKES  = 4'd2;
    localparam S_STREAM       = 4'd3;
    localparam S_WAIT_MAC     = 4'd4;
    localparam S_LIF_STEP     = 4'd5;
    localparam S_CHECK_WIN    = 4'd6;
    localparam S_DECIDE       = 4'd7;
    localparam S_SEND_RESULT  = 4'd8;

    reg [3:0] state;

    // -------------------------------------------------------------------------
    // Counters
    // -------------------------------------------------------------------------
    reg [12:0] spike_count;     // Spikes delivered to adder in current timestep
    reg [4:0]  timestep_count;  // Timesteps completed in current window
    reg [5:0]  lif_spike_accum; // LIF spikes accumulated across T_WINDOW

    // -------------------------------------------------------------------------
    // State register
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state            <= S_IDLE;
            adder_start      <= 0;
            lif_current_valid<= 0;
            sc_start         <= 0;
            sc_spike_valid   <= 0;
            tx_send          <= 0;
            inference_done   <= 0;
            class_out        <= 0;
            tx_data          <= 0;
            spike_count      <= 0;
            timestep_count   <= 0;
            lif_spike_accum  <= 0;
        end else begin
            // Pulse signals: clear every cycle by default
            adder_start       <= 0;
            lif_current_valid <= 0;
            sc_start          <= 0;
            sc_spike_valid    <= 0;
            tx_send           <= 0;
            inference_done    <= 0;

            case (state)
                // ---- Wait until weights are in BRAM -------------------------
                S_IDLE: begin
                    if (!weights_loaded)
                        state <= S_WAIT_WEIGHTS;
                    else begin
                        timestep_count  <= 0;
                        lif_spike_accum <= 0;
                        sc_start        <= 1;  // Initialise spike_counter window
                        state           <= S_WAIT_SPIKES;
                    end
                end

                S_WAIT_WEIGHTS: begin
                    if (weights_loaded) begin
                        timestep_count  <= 0;
                        lif_spike_accum <= 0;
                        sc_start        <= 1;
                        state           <= S_WAIT_SPIKES;
                    end
                end

                // ---- Wait for spike FIFO to have data -----------------------
                S_WAIT_SPIKES: begin
                    spike_count <= 0;
                    if (!fifo_empty) begin
                        adder_start <= 1;   // Prime the cascaded_adder
                        state       <= S_STREAM;
                    end
                end

                // ---- Stream N spikes to cascaded_adder ----------------------
                S_STREAM: begin
                    if (spike_valid_in) begin
                        spike_count <= spike_count + 1;
                        if (spike_count == N - 1)
                            state <= S_WAIT_MAC;
                    end
                end

                // ---- Wait for MAC result ------------------------------------
                S_WAIT_MAC: begin
                    if (adder_valid) begin
                        lif_current_valid <= 1;  // Present result to LIF
                        state             <= S_LIF_STEP;
                    end
                end

                // ---- LIF processes the MAC result for one timestep ----------
                S_LIF_STEP: begin
                    // LIF output is registered: spike_out is valid next cycle
                    sc_spike_valid  <= 1;          // Tell spike_counter to sample
                    if (lif_spike_out)
                        lif_spike_accum <= lif_spike_accum + 1;
                    timestep_count <= timestep_count + 1;
                    state          <= S_CHECK_WIN;
                end

                // ---- Decide whether to do another timestep ------------------
                S_CHECK_WIN: begin
                    if (timestep_count < T_WINDOW)
                        state <= S_WAIT_SPIKES;   // More timesteps: get next chunk
                    else
                        state <= S_DECIDE;
                end

                // ---- Rate-decode and produce classification ------------------
                S_DECIDE: begin
                    class_out      <= (lif_spike_accum > (T_WINDOW >> 1))
                                      ? 1'b1 : 1'b0;
                    inference_done <= 1;
                    tx_data        <= (lif_spike_accum > (T_WINDOW >> 1))
                                      ? 8'h01 : 8'h00;
                    state          <= S_SEND_RESULT;
                end

                // ---- Send result byte to PC via UART ------------------------
                S_SEND_RESULT: begin
                    tx_send        <= 1;
                    // Reset for next frame
                    timestep_count  <= 0;
                    lif_spike_accum <= 0;
                    sc_start        <= 1;
                    state           <= S_WAIT_SPIKES;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
