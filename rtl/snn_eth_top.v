`timescale 1ns / 1ps
// =============================================================================
// Module: snn_eth_top
// Description: Top-level integration of the SNN inference accelerator with
//              100 Mbps MII Raw Ethernet Communication.
//
// Target Board: Digilent Arty A7-100T (xc7a100tcsg324-1)
// Clock:        100 MHz onboard oscillator (Pin E3)
// PHY:          TI DP83848J (MII Interface)
//
// Status LEDs:
//   led[0] -> Heartbeat (blinks continuously)
//   led[1] -> RX Activity (flashes when Ethernet frames arrive)
//   led[2] -> TX Activity (flashes when Ethernet frames transmitted)
//   led[3] -> PHY Ready (ON when eth_rstn de-asserted)
// =============================================================================

module snn_eth_top #(
    parameter [47:0] FPGA_MAC = 48'h00183E04C552
)(
    // 100 MHz system clock from onboard oscillator
    input  wire        CLK100MHZ,
    input  wire        btn0,        // Active high reset button (btn[0])

    // Status LEDs
    output wire [3:0]  led,         // LED0: heartbeat, LED1: RX, LED2: TX, LED3: PHY ready

    // MII Ethernet Interface (TI DP83848J PHY)
    output wire        eth_ref_clk, // 25 MHz PHY reference clock
    output wire        eth_rstn,    // PHY active-low reset
    input  wire        eth_rx_clk,  // RX clock from PHY (25 MHz for 100Mbps)
    input  wire        eth_rx_dv,   // RX data valid
    input  wire [3:0]  eth_rxd,     // RX data [3:0]
    input  wire        eth_rxerr,   // RX error
    input  wire        eth_tx_clk,  // TX clock from PHY (25 MHz for 100Mbps)
    output wire        eth_tx_en,   // TX enable
    output wire [3:0]  eth_txd      // TX data [3:0]
);

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    localparam N         = 4096;
    localparam W         = 16;
    localparam OUT       = 32;
    localparam T_WINDOW  = 25;

    // -------------------------------------------------------------------------
    // Synchronize active-high reset button to clock domain (btn0 -> active-low)
    // -------------------------------------------------------------------------
    reg rst_n_sync0 = 1'b0;
    reg rst_n_sync1 = 1'b0;

    always @(posedge CLK100MHZ or posedge btn0) begin
        if (btn0) begin
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
    // Internal Wire Declarations
    // =========================================================================

    // FIFO <-> spike_unpacker
    wire       fifo_rd_en;
    wire [7:0] fifo_dout;
    wire       fifo_empty;

    // fifo_eth_controller -> weight_bram
    wire [11:0] bram_addr_w;
    wire [15:0] bram_din_w;
    wire        bram_wr_en_w;

    // master_fsm -> weight_bram (read)
    wire [11:0] adder_weight_addr;  // from cascaded_adder
    wire [15:0] bram_dout;          // to cascaded_adder

    // fifo_eth_controller -> master_fsm
    wire weights_loaded;

    // spike_unpacker -> master_fsm / cascaded_adder
    wire spike_valid;
    wire spike_bit;

    // master_fsm -> cascaded_adder
    wire adder_start;

    // cascaded_adder -> master_fsm / lif_neuron
    wire adder_valid;
    wire signed [OUT-1:0] adder_result;

    // master_fsm -> lif_neuron
    wire lif_current_valid;

    // lif_neuron -> master_fsm
    wire lif_spike_out;
    wire lif_spike_valid;

    // master_fsm -> spike_counter
    wire sc_start;
    wire sc_spike_valid;

    // spike_counter -> master_fsm
    wire sc_done;
    wire sc_result;

    // master_fsm -> Ethernet TX
    wire [7:0] tx_data;
    wire       tx_send;

    // master_fsm status
    wire inference_done;
    wire class_out;

    // =========================================================================
    // Module Instantiations
    // =========================================================================

    // ---- 1. FIFO + Ethernet Controller --------------------------------------
    fifo_eth_controller #(
        .FPGA_MAC   (FPGA_MAC),
        .SNN_ETH_TYP(16'h88B5),
        .FIFO_DEPTH (1024)
    ) u_fifo_eth (
        .clk           (CLK100MHZ),
        .rst_n         (sys_rst_n),
        .eth_ref_clk   (eth_ref_clk),
        .eth_rstn      (eth_rstn),
        .eth_rx_clk    (eth_rx_clk),
        .eth_rx_dv     (eth_rx_dv),
        .eth_rxd       (eth_rxd),
        .eth_rxerr     (eth_rxerr),
        .eth_tx_clk    (eth_tx_clk),
        .eth_tx_en     (eth_tx_en),
        .eth_txd       (eth_txd),
        .led_heartbeat (led[0]),
        .led_rx        (led[1]),
        .led_tx        (led[2]),
        .led_phy_ready (led[3]),
        .fifo_rd_en    (fifo_rd_en),
        .fifo_dout     (fifo_dout),
        .fifo_empty    (fifo_empty),
        .bram_addr     (bram_addr_w),
        .bram_din      (bram_din_w),
        .bram_wr_en    (bram_wr_en_w),
        .tx_send_fsm   (tx_send),
        .tx_data_fsm   (tx_data),
        .weights_loaded(weights_loaded)
    );

    // ---- 2. Weight BRAM (8KB) -----------------------------------------------
    weight_bram #(
        .DEPTH(4096),
        .WIDTH(16),
        .ABITS(12)
    ) u_weight_bram (
        // Write port (from Ethernet controller during weight loading)
        .clk_a  (CLK100MHZ),
        .we_a   (bram_wr_en_w),
        .addr_a (bram_addr_w),
        .din_a  (bram_din_w),
        // Read port (from cascaded_adder during inference)
        .clk_b  (CLK100MHZ),
        .addr_b (adder_weight_addr),
        .dout_b (bram_dout)
    );

    // ---- 3. Spike Unpacker --------------------------------------------------
    spike_unpacker u_unpacker (
        .clk        (CLK100MHZ),
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
        .clk         (CLK100MHZ),
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

    // ---- 5. LIF Neuron ------------------------------------------------------
    lif_neuron #(
        .WIDTH     (32),
        .THRESHOLD (32'sd1000),
        .LEAK_SHIFT(3),
        .RESET_VAL (0)
    ) u_lif (
        .clk           (CLK100MHZ),
        .rst           (sys_rst),
        .current_in    (adder_result[31:0]),
        .current_valid (lif_current_valid),
        .spike_out     (lif_spike_out),
        .spike_valid   (lif_spike_valid),
        .membrane_out  ()
    );

    // ---- 6. Spike Counter (Rate Decoder) ------------------------------------
    spike_counter #(
        .T_WINDOW(T_WINDOW)
    ) u_sc (
        .clk         (CLK100MHZ),
        .rst_n       (sys_rst_n),
        .start       (sc_start),
        .spike_in    (lif_spike_out),
        .spike_valid (sc_spike_valid),
        .count_1     (),
        .count_0     (),
        .result      (sc_result),
        .done        (sc_done)
    );

    // ---- 7. Master FSM ------------------------------------------------------
    master_fsm #(
        .N       (N),
        .T_WINDOW(T_WINDOW)
    ) u_fsm (
        .clk              (CLK100MHZ),
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

endmodule
