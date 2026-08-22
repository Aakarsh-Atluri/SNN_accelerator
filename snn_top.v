`timescale 1ns / 1ps
// =============================================================================
// Module: snn_top
// Description: Top-level integration of the SNN inference engine accelerator.
//              Instantiates all submodules and connects them.
//
// Target Board: Digilent Arty A7-100T (xc7a100tcsg324-1)
// Clock:        100 MHz onboard oscillator (Pin E3)
//
// Pin Connections for Digilent Arty A7-100T:
//   clk        → Pin E3  (100 MHz oscillator, IOSTANDARD LVCMOS33)
//   rst_n      → Pin C2  (RESET pushbutton / CPU_RESETN, active-LOW)
//   rx         → Pin A9  (UART_TXD_IN / USB-UART RX from FT2232)
//   tx         → Pin D10 (UART_RXD_OUT / USB-UART TX to FT2232)
//   result_led → Pin H5  (LED 0, green: 1 = collision detected)
//   done_led   → Pin J5  (LED 1, green: 1 = inference completed)
// =============================================================================
module snn_top (
    input  wire clk,        // 100 MHz system clock
    input  wire rst_n,      // Active-low reset (Arty A7 RESET button / CPU_RESETN)
    input  wire rx,         // UART receive (FPGA input from PC)
    output wire tx,         // UART transmit (FPGA output to PC)
    output wire result_led, // LED 0: 1 = collision detected
    output wire done_led    // LED 1: 1 = inference complete
);

    // -------------------------------------------------------------------------
    // Parameter declarations
    // -------------------------------------------------------------------------
    localparam N         = 4096;
    localparam W         = 16;
    localparam OUT       = 32;
    localparam T_WINDOW  = 25;
    localparam CLK_FREQ  = 100_000_000;
    localparam BAUD_RATE = 115200;

    // -------------------------------------------------------------------------
    // Synchronize active-low reset to clock domain
    // -------------------------------------------------------------------------
    reg rst_n_sync0;
    reg rst_n_sync1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rst_n_sync0 <= 1'b0;
            rst_n_sync1 <= 1'b0;
        end else begin
            rst_n_sync0 <= 1'b1;
            rst_n_sync1 <= rst_n_sync0;
        end
    end

    wire sys_rst_n = rst_n_sync1;
    wire sys_rst   = ~sys_rst_n;

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
    wire [11:0] adder_weight_addr;  // from cascaded_adder
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
    wire lif_spike_valid;

    // master_fsm → spike_counter
    wire sc_start;
    wire sc_spike_valid;

    // spike_counter → master_fsm
    wire sc_done;
    wire sc_result;

    // master_fsm → UART TX (via fifo_uart_controller)
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
        .rst_n         (sys_rst_n),
        .rx            (rx),
        .tx            (tx),
        .fifo_rd_en    (fifo_rd_en),
        .fifo_dout     (fifo_dout),
        .fifo_empty    (fifo_empty),
        .bram_addr     (bram_addr_w),
        .bram_din      (bram_din_w),
        .bram_wr_en    (bram_wr_en_w),
        .tx_send_fsm   (tx_send),
        .tx_data_fsm   (tx_data),
        .tx_busy       (),
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
        .addr_b (adder_weight_addr),
        .dout_b (bram_dout)
    );

    // ---- 3. Spike Unpacker ---------------------------------------------------
    spike_unpacker u_unpacker (
        .clk        (clk),
        .rst_n      (sys_rst_n),
        .fifo_empty (fifo_empty),
        .fifo_dout  (fifo_dout),
        .fifo_rd_en (fifo_rd_en),
        .spike_out  (spike_bit),
        .spike_valid(spike_valid)
    );

    // ---- 4. Cascaded Adder (MAC) --------------------------------------------
    cascaded_adder #(
        .N     (N),
        .W     (W),
        .OUT   (OUT),
        .ADDR_W(12)
    ) u_mac (
        .clk         (clk),
        .rst         (sys_rst),
        .start       (adder_start),
        .busy        (),
        .spike_in    (spike_bit),
        .spike_valid (spike_valid),
        .weight_addr (adder_weight_addr),
        .weight_data (bram_dout),
        .bias        (16'sd0),        // Zero bias
        .result      (adder_result),
        .valid       (adder_valid)
    );

    // ---- 5. LIF Neuron -------------------------------------------------------
    lif_neuron #(
        .WIDTH     (32),
        .THRESHOLD (32'sd1000),
        .LEAK_SHIFT(3),
        .RESET_VAL (0)
    ) u_lif (
        .clk           (clk),
        .rst           (sys_rst),
        .current_in    (adder_result[31:0]),
        .current_valid (lif_current_valid),
        .spike_out     (lif_spike_out),
        .spike_valid   (lif_spike_valid),
        .membrane_out  ()
    );

    // ---- 6. Spike Counter (Rate Decoder) -------------------------------------
    spike_counter #(
        .T_WINDOW(T_WINDOW)
    ) u_sc (
        .clk         (clk),
        .rst_n       (sys_rst_n),
        .start       (sc_start),
        .spike_in    (lif_spike_out),
        .spike_valid (sc_spike_valid),
        .count_1     (),
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
        .rst_n            (sys_rst_n),
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

    // ---- 8. Status LEDs with Latch for Clear Visibility ----------------------
    reg done_latched;
    always @(posedge clk) begin
        if (!sys_rst_n) begin
            done_latched <= 1'b0;
        end else if (inference_done) begin
            done_latched <= 1'b1;
        end
    end

    assign result_led = class_out;
    assign done_led   = done_latched;

endmodule
