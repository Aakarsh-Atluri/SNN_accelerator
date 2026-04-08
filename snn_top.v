`timescale 1ns / 1ps
// =============================================================================
// Module: snn_top
// Description: Top-level integration of the SNN inference engine.
//              Instantiates all submodules and connects them.
//
// Target: Xilinx Artix-7 (xc7a35t or larger)
// Clock:  100 MHz system clock (add clocking wizard if using oscillator)
//
// Pin Connections (example for Basys3 / Nexys A7):
//   clk   → W5  (100 MHz oscillator)
//   rst_n → T1  (BTN_CENTER, active LOW via inversion in XDC)
//   rx    → B18 (USB-UART RX from FTDI chip)
//   tx    → A18 (USB-UART TX to   FTDI chip)
//   result_led → LED[0]
// =============================================================================
module snn_top (
    input  wire clk,       // 100 MHz
    input  wire rst_n,     // Active-low reset (button)
    input  wire rx,        // UART receive
    output wire tx,        // UART transmit
    output wire result_led, // LED: 1 = collision detected
    output wire done_led    // LED: inference complete
);

    // -------------------------------------------------------------------------
    // Parameter declarations
    // -------------------------------------------------------------------------
    localparam N         = 4096;
    localparam W         = 16;
    localparam OUT       = 28;
    localparam T_WINDOW  = 25;
    localparam CLK_FREQ  = 100_000_000;
    localparam BAUD_RATE = 115200;

    // =========================================================================
    // Internal wire declarations
    // =========================================================================

    // FIFO ↔ spike_unpacker
    wire       fifo_rd_en;
    wire [7:0] fifo_dout;
    wire       fifo_empty;

    // fifo_uart_controller → weight_bram
    wire [11:0] bram_addr_w;
    wire [15:0] bram_din_w;
    wire        bram_wr_en_w;

    // master_fsm → weight_bram (read)
    wire [12:0] adder_weight_addr;  // from cascaded_adder
    wire [15:0] bram_dout;          // to cascaded_adder

    // fifo_uart_controller → master_fsm
    wire weights_loaded;

    // spike_unpacker → master_fsm / cascaded_adder
    wire spike_valid;
    wire spike_bit;

    // master_fsm → cascaded_adder
    wire adder_start;

    // cascaded_adder → master_fsm / lif_neuron
    wire adder_valid;
    wire signed [OUT-1:0] adder_result;

    // master_fsm → lif_neuron
    wire lif_current_valid;

    // lif_neuron → master_fsm
    wire lif_spike_out;

    // master_fsm → spike_counter
    wire sc_start;
    wire sc_spike_valid;

    // spike_counter → master_fsm
    wire sc_done;
    wire sc_result;

    // master_fsm → UART TX
    wire [7:0] tx_data;
    wire       tx_send;

    // master_fsm status
    wire inference_done;
    wire class_out;

    // =========================================================================
    // Module instantiations
    // =========================================================================

    // ---- 1. FIFO + UART Controller ------------------------------------------
    fifo_uart_controller #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (BAUD_RATE),
        .FIFO_DEPTH(1024)
    ) u_fifo_uart (
        .clk           (clk),
        .rst_n         (rst_n),
        .rx            (rx),
        .tx            (tx),          // Weight-loading & ACK TX handled here
        .fifo_rd_en    (fifo_rd_en),
        .fifo_dout     (fifo_dout),
        .fifo_empty    (fifo_empty),
        .bram_addr     (bram_addr_w),
        .bram_din      (bram_din_w),
        .bram_wr_en    (bram_wr_en_w),
        .weights_loaded(weights_loaded)
    );

    // ---- 2. Weight BRAM (8KB, inferred as Block RAM) -------------------------
    weight_bram #(
        .DEPTH(4096),
        .WIDTH(16),
        .ABITS(12)
    ) u_weight_bram (
        // Write port (from FIFO controller during weight loading)
        .clk_a  (clk),
        .we_a   (bram_wr_en_w),
        .addr_a (bram_addr_w),
        .din_a  (bram_din_w),
        // Read port (from cascaded_adder during inference)
        .clk_b  (clk),
        .addr_b (adder_weight_addr[11:0]),
        .dout_b (bram_dout)
    );

    // ---- 3. Spike Unpacker ---------------------------------------------------
    spike_unpacker u_unpacker (
        .clk        (clk),
        .rst_n      (rst_n),
        .fifo_empty (fifo_empty),
        .fifo_dout  (fifo_dout),
        .fifo_rd_en (fifo_rd_en),
        .spike_out  (spike_bit),
        .spike_valid(spike_valid)
    );

    // ---- 4. Cascaded Adder (MAC) --------------------------------------------
    // Note: bias = 0 for simplicity; replace with a register file if
    //       you have per-neuron biases loaded separately.
    cascaded_adder #(
        .N  (N),
        .W  (W),
        .OUT(OUT)
    ) u_mac (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (adder_start),
        .spike_in    (spike_bit),
        .spike_valid (spike_valid),
        .weight_addr (adder_weight_addr),
        .weight_data (bram_dout),
        .bias        (16'sd0),        // Tie to 0 or add bias BRAM
        .result      (adder_result),
        .valid       (adder_valid)
    );

    // ---- 5. LIF Neuron -------------------------------------------------------
    lif_neuron #(
        .THRESHOLD (16'sd1000),
        .LEAK_SHIFT(3),
        .REF_PERIOD(3'd5)
    ) u_lif (
        .clk           (clk),
        .rst_n         (rst_n),
        .current_in    (adder_result[15:0]),   // Take lower 16 bits of 28-bit result
        .current_valid (lif_current_valid),
        .spike_out     (lif_spike_out)
    );

    // ---- 6. Spike Counter (Rate Decoder) -------------------------------------
    spike_counter #(
        .T_WINDOW(T_WINDOW)
    ) u_sc (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (sc_start),
        .spike_in    (lif_spike_out),
        .spike_valid (sc_spike_valid),
        .count_1     (),               // Unused outputs left open
        .count_0     (),
        .result      (sc_result),
        .done        (sc_done)
    );

    // ---- 7. Master FSM -------------------------------------------------------
    master_fsm #(
        .N       (N),
        .T_WINDOW(T_WINDOW)
    ) u_fsm (
        .clk              (clk),
        .rst_n            (rst_n),
        .fifo_empty       (fifo_empty),
        .weights_loaded   (weights_loaded),
        .spike_valid_in   (spike_valid),
        .spike_bit        (spike_bit),
        .adder_start      (adder_start),
        .adder_valid      (adder_valid),
        .lif_current_valid(lif_current_valid),
        .lif_spike_out    (lif_spike_out),
        .sc_start         (sc_start),
        .sc_spike_valid   (sc_spike_valid),
        .sc_done          (sc_done),
        .sc_result        (sc_result),
        .tx_data          (tx_data),
        .tx_send          (tx_send),
        .inference_done   (inference_done),
        .class_out        (class_out)
    );

    // ---- 8. Status LEDs ------------------------------------------------------
    assign result_led = class_out;
    assign done_led   = inference_done;

endmodule
