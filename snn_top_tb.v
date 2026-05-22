`timescale 1ns / 1ps

`ifndef SYNTHESIS

module snn_top_tb;

    // UUT Inputs
    reg clk;
    reg rst_n;
    reg rx;

    // UUT Outputs
    wire tx;
    wire result_led;
    wire done_led;

    // Instantiate the Top-Level Module
    snn_top uut (
        .clk(clk),
        .rst_n(rst_n),
        .rx(rx),
        .tx(tx),
        .result_led(result_led),
        .done_led(done_led)
    );

    // ---------------------------------------------------------
    // Clock Generation (100 MHz)
    // ---------------------------------------------------------
    initial begin
        clk = 0;
    end

    always #5 clk = ~clk;

    // ---------------------------------------------------------
    // UART Simulation Parameters
    // ---------------------------------------------------------
    localparam BAUD_RATE = 115200;
    localparam BIT_PERIOD = 1000000000 / BAUD_RATE; 

    task send_uart_byte;
        input [7:0] data;
        begin
            rx = 0; // Start bit
            #(BIT_PERIOD);
            
            rx = data[0]; #(BIT_PERIOD);
            rx = data[1]; #(BIT_PERIOD);
            rx = data[2]; #(BIT_PERIOD);
            rx = data[3]; #(BIT_PERIOD);
            rx = data[4]; #(BIT_PERIOD);
            rx = data[5]; #(BIT_PERIOD);
            rx = data[6]; #(BIT_PERIOD);
            rx = data[7]; #(BIT_PERIOD);
            
            rx = 1; // Stop bit
            #(BIT_PERIOD);
        end
    endtask

    task wait_for_ack;
        begin
            wait(tx == 0); 
            #(BIT_PERIOD * 10); 
            #(BIT_PERIOD * 2);  
        end
    endtask

    // ---------------------------------------------------------
    // Main Test Stimulus
    // ---------------------------------------------------------
    integer t_idx;
    integer w_idx;

    initial begin
        $timeformat(-9, 0, " ns", 12);

        rst_n = 0;
        rx = 1; 
        #100;
        rst_n = 1;
        #100;

        $display("\n[%0t] --- Starting SNN Simulation (ZERO INPUT TEST) ---", $time);

        // 1. Load Weights (still required so FSM advances)
        $display("[%0t] Sending Weight Load Command (0xAA)...", $time);
        send_uart_byte(8'hAA);

        w_idx = 0;
        $display("[%0t] Loading 4096 weights via UART...", $time);
        
        repeat (4096) begin
            send_uart_byte(8'h00); // MSB
            send_uart_byte(8'h01); // LSB
            w_idx = w_idx + 1;
            
            if (w_idx % 1024 == 0) begin
                $display("[%0t]   -> Successfully loaded %0d / 4096 weights", $time, w_idx);
            end
        end
        $display("[%0t] All weights loaded into BRAM.", $time);

        #50000;

        // 2. Stream Spikes
        $display("\n[%0t] Initializing Spike Stream Mode (0xBB)...", $time);
        send_uart_byte(8'hBB);
        
        wait_for_ack(); 
        $display("[%0t] ACK Received! Starting inference window...", $time);

        t_idx = 0;
        // MUST loop 25 times to satisfy T_WINDOW=25 in snn_top.v
        repeat (25) begin
            $display("\n[%0t] --- Streaming ZERO spikes for Timestep %0d / 25 ---", $time, t_idx + 1);
            
            // Send 0x00 (all 0s) instead of 0xFF
            repeat (512) begin
                send_uart_byte(8'h00);
            end
            
            $display("[%0t]   -> 512 bytes of ZERO sent. Waiting for MAC and LIF...", $time);
            
            wait_for_ack();
            $display("[%0t]   -> Chunk ACK received.", $time);
            
            t_idx = t_idx + 1;
        end

        // 3. Await Results
        $display("\n[%0t] Inference streaming finished. Waiting for final classification...", $time);
        $display("\n=============================================");
        $display("[%0t] INFERENCE COMPLETE!", $time);
        $display("[%0t] Result LED (Collision): %b (Expected: 0)", $time, result_led);
        
        if (result_led == 1'b0) begin
            $display("[%0t] TEST PASSED: Zero input correctly yielded 0 output.", $time);
        end else begin
            $display("[%0t] TEST FAILED: Output should have been 0.", $time);
        end
        $display("=============================================\n");

        #100000;
        $finish;
    end

  
endmodule
`endif