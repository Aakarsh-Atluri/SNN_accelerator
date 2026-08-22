`timescale 1ns / 1ps
// =============================================================================
// Testbench : tb_snn_top
// Description: Full End-to-End Testbench for snn_top with UART communication.
//              Simulates the host PC communicating with the FPGA accelerator:
//                1. Sends 0xAA header + 8192 weight bytes over UART RX.
//                2. Receives 0x01 ACK on UART TX once weights are loaded.
//                3. Sends 0xBB command over UART RX.
//                4. Receives 0x01 ACK on UART TX.
//                5. Sends 25 timesteps of 512 bytes spikes (12800 bytes).
//                6. Receives ACKs per timestep.
//                7. Receives final classification result byte (0x01) on UART TX.
//                8. Verifies result_led and done_led status.
// =============================================================================

module tb_snn_top;

    reg clk;
    reg rst_n;
    reg rx;
    wire tx;
    wire result_led;
    wire done_led;

    // 100 MHz clock
    initial clk = 0;
    always #5 clk = ~clk;

    // Instantiate Top Module
    snn_top u_top (
        .clk        (clk),
        .rst_n      (rst_n),
        .rx         (rx),
        .tx         (tx),
        .result_led (result_led),
        .done_led   (done_led)
    );

    // -------------------------------------------------------------------------
    // UART Helper Tasks (Baud rate: 115200 -> bit period = 8680 ns)
    // -------------------------------------------------------------------------
    localparam BIT_PERIOD = 8680; // 8680 ns = 868 clock cycles at 100 MHz

    task uart_send_byte(input [7:0] data);
        integer b;
        begin
            // Start bit
            rx = 1'b0;
            #(BIT_PERIOD);

            // 8 Data bits (LSB first)
            for (b = 0; b < 8; b = b + 1) begin
                rx = data[b];
                #(BIT_PERIOD);
            end

            // Stop bit
            rx = 1'b1;
            #(BIT_PERIOD);
            #(BIT_PERIOD / 2);
        end
    endtask

    task uart_recv_byte(output [7:0] data);
        integer b;
        begin
            // Wait for start bit falling edge
            @(negedge tx);
            #(BIT_PERIOD / 2); // Sample at midpoint of start bit

            // Wait for first data bit midpoint
            #(BIT_PERIOD);

            // Sample 8 data bits
            for (b = 0; b < 8; b = b + 1) begin
                data[b] = tx;
                #(BIT_PERIOD);
            end

            // Stop bit
            #(BIT_PERIOD / 2);
        end
    endtask

    // -------------------------------------------------------------------------
    // Test Sequence
    // -------------------------------------------------------------------------
    reg [7:0] rx_ack;
    reg [7:0] rx_result;
    integer i, t;

    initial begin
        $display("==========================================================");
        $display("  SNN Accelerator - Top-Level UART Integration Testbench  ");
        $display("==========================================================");

        rst_n = 0;
        rx    = 1;
        #1000;
        rst_n = 1;
        #2000;

        // 1. Send 0xAA Header (Write Weights)
        $display("\n[%0t] Step 1: Sending 0xAA (Write Weights command)...", $time);
        uart_send_byte(8'hAA);

        // Send 8192 bytes of weights (4096 weights = 25 for obstacle, -2 otherwise)
        $display("[%0t] Sending 8192 weight bytes over UART...", $time);
        fork
            begin
                for (i = 0; i < 4096; i = i + 1) begin
                    // 16-bit weight: 25 = 0x0019
                    uart_send_byte(8'h00); // MSB
                    uart_send_byte(8'h19); // LSB
                end
            end
            begin
                // FPGA sends 0x01 ACK automatically when 8192nd byte is written
                uart_recv_byte(rx_ack);
            end
        join
        $display("[%0t] Weight transmission complete. Received Weight ACK: 0x%02h", $time, rx_ack);

        #10000;

        // 2. Send 0xBB (Start Spikes)
        $display("\n[%0t] Step 2: Sending 0xBB (Start Spikes command)...", $time);
        fork
            uart_send_byte(8'hBB);
            uart_recv_byte(rx_ack);
        join
        $display("[%0t] FPGA responded with ACK: 0x%02h (Expected: 0x01)", $time, rx_ack);

        // 3. Stream 25 timesteps of 512 bytes spikes (Collision pattern)
        $display("\n[%0t] Step 3: Streaming 25 timesteps of spikes...", $time);
        for (t = 0; t < 25; t = t + 1) begin
            fork
                begin
                    for (i = 0; i < 512; i = i + 1) begin
                        uart_send_byte(8'hFF);
                    end
                end
                begin
                    uart_recv_byte(rx_ack);
                end
            join
            $display("  Timestep %0d/25 ACK received: 0x%02h", t + 1, rx_ack);
        end

        // 4. Wait for Final Classification Result Byte from UART TX
        $display("\n[%0t] Step 4: Awaiting Final Classification Result on UART TX...", $time);
        uart_recv_byte(rx_result);
        $display("[%0t] Received Classification Result Byte: 0x%02h", $time, rx_result);

        #50000;

        // 5. Verify Outputs
        $display("\n==========================================================");
        $display("  VERIFICATION SUMMARY");
        $display("==========================================================");
        $display("  Result Byte : 0x%02h (Expected: 0x01)", rx_result);
        $display("  Result LED  : %b    (Expected: 1)", result_led);
        $display("  Done LED    : %b    (Expected: 1)", done_led);

        if (rx_result == 8'h01 && result_led == 1'b1 && done_led == 1'b1) begin
            $display("\n  >>> [SUCCESS] ALL TOP-LEVEL INTEGRATION CHECKS PASSED <<<");
        end else begin
            $display("\n  >>> [FAILURE] TOP-LEVEL INTEGRATION FAILED <<<");
        end
        $display("==========================================================\n");

        #10000;
        $finish;
    end

endmodule
