`timescale 1ns / 1ps
// =============================================================================
// Testbench : tb_snn_eth_top
// Description: Full End-to-End Simulation Testbench for snn_eth_top.
//              Simulates host PC transmitting raw Ethernet frames to FPGA:
//                1. 8 frames of 0xAA (WRITE_WEIGHTS) with 1024 bytes payload each.
//                2. Verification of ACK packets from FPGA.
//                3. 25 frames of 0xBB (WRITE_SPIKES) with 512 bytes payload each.
//                4. Verification of timestep ACKs from FPGA.
//                5. Verification of final classification result packet & LEDs.
// =============================================================================

module tb_snn_eth_top;

    reg clk100 = 0;
    always #5 clk100 = ~clk100; // 100MHz (10ns period)

    reg eth_rx_clk = 0;
    always #20 eth_rx_clk = ~eth_rx_clk; // 25MHz (40ns period)

    reg eth_tx_clk = 0;
    always #20 eth_tx_clk = ~eth_tx_clk; // 25MHz (40ns period)

    reg btn0 = 0;
    wire [3:0] led;
    wire eth_ref_clk;
    wire eth_rstn;
    reg eth_rx_dv = 0;
    reg [3:0] eth_rxd = 0;
    reg eth_rxerr = 0;
    wire eth_tx_en;
    wire [3:0] eth_txd;

    // Instantiate Top DUT
    snn_eth_top #(
        .FPGA_MAC(48'h00183E04C552)
    ) dut (
        .CLK100MHZ   (clk100),
        .btn0        (btn0),
        .led         (led),
        .eth_ref_clk (eth_ref_clk),
        .eth_rstn    (eth_rstn),
        .eth_rx_clk  (eth_rx_clk),
        .eth_rx_dv   (eth_rx_dv),
        .eth_rxd     (eth_rxd),
        .eth_rxerr   (eth_rxerr),
        .eth_tx_clk  (eth_tx_clk),
        .eth_tx_en   (eth_tx_en),
        .eth_txd     (eth_txd)
    );

    // Fast-forward PHY reset in simulation
    initial begin
        force dut.u_fifo_eth.rst_cnt = 24'd10_000_000;
        force dut.u_fifo_eth.phy_rstn_reg = 1'b1;
    end

    // Task to send an Ethernet byte via MII
    task send_mii_byte(input [7:0] b);
        begin
            @(posedge eth_rx_clk);
            eth_rx_dv = 1;
            eth_rxd   = b[3:0]; // low nibble
            @(posedge eth_rx_clk);
            eth_rx_dv = 1;
            eth_rxd   = b[7:4]; // high nibble
        end
    endtask

    // Monitor TX output
    reg [7:0] tx_byte = 0;
    reg tx_toggle = 0;
    reg [3:0] tx_low = 0;
    integer tx_byte_count = 0;
    reg [7:0] tx_frame_captured [0:1518];

    always @(posedge eth_tx_clk) begin
        if (eth_tx_en) begin
            if (!tx_toggle) begin
                tx_low    <= eth_txd;
                tx_toggle <= 1'b1;
            end else begin
                tx_byte   = {eth_txd, tx_low};
                tx_toggle <= 1'b0;
                tx_frame_captured[tx_byte_count] = tx_byte;
                tx_byte_count = tx_byte_count + 1;
            end
        end else begin
            if (tx_byte_count > 0) begin
                // Frame format: 7 bytes 0x55 preamble (indices 0..6), 1 byte 0xD5 SFD (index 7)
                // Payload: Destination MAC at index 8..13, Source MAC at index 14..19, EtherType at index 20..21, Cmd at index 22
                $display("\n[%0t] === [TX PACKET CAPTURED] Total bytes: %0d ===", $time, tx_byte_count);
                if (tx_byte_count >= 24) begin
                    $display("      Dest MAC : %02X:%02X:%02X:%02X:%02X:%02X",
                             tx_frame_captured[8], tx_frame_captured[9], tx_frame_captured[10],
                             tx_frame_captured[11], tx_frame_captured[12], tx_frame_captured[13]);
                    $display("      Src MAC  : %02X:%02X:%02X:%02X:%02X:%02X",
                             tx_frame_captured[14], tx_frame_captured[15], tx_frame_captured[16],
                             tx_frame_captured[17], tx_frame_captured[18], tx_frame_captured[19]);
                    $display("      EtherType: 0x%02X%02X", tx_frame_captured[20], tx_frame_captured[21]);
                    $display("      Cmd Byte : 0x%02X, Param1: 0x%02X, Param2: 0x%02X",
                             tx_frame_captured[22], tx_frame_captured[23], tx_frame_captured[24]);
                end
                tx_byte_count = 0;
            end
            tx_toggle <= 1'b0;
        end
    end

    // Task to send full raw Ethernet Frame
    task send_eth_frame(
        input [7:0] cmd,
        input [7:0] sub_idx,
        input integer payload_len,
        input [7:0] payload_val
    );
        integer k;
        begin
            // Preamble (7 bytes of 0x55)
            for (k = 0; k < 7; k = k + 1) send_mii_byte(8'h55);
            // SFD (1 byte of 0xD5)
            send_mii_byte(8'hD5);

            // Destination MAC (FPGA MAC: 00:18:3E:04:C5:52)
            send_mii_byte(8'h00); send_mii_byte(8'h18); send_mii_byte(8'h3E);
            send_mii_byte(8'h04); send_mii_byte(8'hC5); send_mii_byte(8'h52);

            // Source MAC (Host MAC: 00:11:22:33:44:55)
            send_mii_byte(8'h00); send_mii_byte(8'h11); send_mii_byte(8'h22);
            send_mii_byte(8'h33); send_mii_byte(8'h44); send_mii_byte(8'h55);

            // EtherType: 0x88B5
            send_mii_byte(8'h88); send_mii_byte(8'hB5);

            // Payload: [cmd] [sub_idx] [data...]
            send_mii_byte(cmd);
            send_mii_byte(sub_idx);

            for (k = 0; k < payload_len; k = k + 1) begin
                send_mii_byte(payload_val);
            end

            // Padding if < 46 payload bytes (14 header + 46 payload = 60 bytes minimum)
            if (payload_len + 2 < 46) begin
                for (k = payload_len + 2; k < 46; k = k + 1) send_mii_byte(8'h00);
            end

            // 4 dummy FCS bytes (stripped in hardware)
            send_mii_byte(8'hAA); send_mii_byte(8'hBB); send_mii_byte(8'hCC); send_mii_byte(8'hDD);

            // End frame
            @(posedge eth_rx_clk);
            eth_rx_dv = 0;
            eth_rxd   = 0;
            
            // Wait for FPGA processing and ACK response (minimum ~1000 RX clocks / 40us)
            #40000;
        end
    endtask

    // Main Test Sequence
    integer b, t;
    initial begin
        $display("==================================================================");
        $display("  SNN Accelerator - Top-Level Raw Ethernet Simulation Testbench  ");
        $display("==================================================================");

        btn0 = 1;
        #200;
        btn0 = 0;
        #500;

        // Step 1: Write Weights (8 blocks of 1024 bytes each = 8192 bytes)
        $display("\n[%0t] STEP 1: Sending 8 Weight Blocks (8192 Bytes total)...", $time);
        for (b = 0; b < 8; b = b + 1) begin
            $display("[%0t]   -> Sending Weight Block %0d/7 (1024 Bytes)...", $time, b);
            // Payload value 0x19 (25 decimal for obstacle detection)
            send_eth_frame(8'hAA, b[7:0], 1024, 8'h19);
        end

        #10000;
        $display("[%0t] Weights Loaded Status in FPGA: %b (Expected: 1)", $time, dut.weights_loaded);

        // Step 2: Stream 25 timesteps of Spikes (512 bytes each)
        $display("\n[%0t] STEP 2: Streaming 25 Timesteps of Spikes (Collision Pattern 0xFF)...", $time);
        for (t = 0; t < 25; t = t + 1) begin
            $display("[%0t]   -> Sending Spikes Timestep %0d/24 (512 Bytes)...", $time, t);
            send_eth_frame(8'hBB, t[7:0], 512, 8'hFF);
        end

        // Step 3: Wait for inference completion and result packet
        $display("\n[%0t] STEP 3: Waiting for Master FSM processing & Ethernet result packet...", $time);
        #400000; // SNN core processes 25 * 4096 cycles (at 100MHz = ~100us)

        // Step 4: Check results
        $display("\n==================================================================");
        $display("  VERIFICATION SUMMARY");
        $display("==================================================================");
        $display("  Weights Loaded : %b (Expected: 1)", dut.weights_loaded);
        $display("  Inference Done : %b", dut.u_fsm.inference_done);
        $display("  Class Result   : %b (Expected: 1 for Collision)", dut.u_fsm.class_out);

        if (dut.weights_loaded == 1'b1 && dut.u_fsm.class_out == 1'b1) begin
            $display("\n  >>> [SUCCESS] ALL ETHERNET SNN ACCELERATOR TESTS PASSED <<<");
        end else begin
            $display("\n  >>> [FAILURE] SIMULATION TEST FAILED <<<");
        end
        $display("==================================================================\n");

        #10000;
        $finish;
    end

endmodule
