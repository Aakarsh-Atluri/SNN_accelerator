//simple testbench to check if there are any timing/cycle errors


`timescale 1ns/1ps
module tb_lif_neuron;

    parameter WIDTH      = 32;
    parameter THRESHOLD  = 10;
    parameter RESET_VAL  = 0;
    parameter LEAK_SHIFT = 1;

    reg clk;
    reg rst;

    reg signed [WIDTH-1:0] current_in;
    reg current_valid;

    wire spike_out;
    wire spike_valid;

    wire signed [WIDTH-1:0] membrane_out;

    // ============================================================
    // DUT
    // ============================================================

    lif_neuron #(
        .WIDTH(WIDTH),
        .THRESHOLD(THRESHOLD),
        .RESET_VAL(RESET_VAL),
        .LEAK_SHIFT(LEAK_SHIFT)
    ) dut (
        .clk(clk),
        .rst(rst),

        .current_in(current_in),
        .current_valid(current_valid),

        .spike_out(spike_out),
        .spike_valid(spike_valid),

        .membrane_out(membrane_out)
    );

    // ============================================================
    // Clock
    // ============================================================

    always #5 clk = ~clk;

    // ============================================================
    // Debug Monitor
    // ============================================================

    always @(posedge clk) begin

        $display(
            "[TB] time=%0t | current=%0d | valid=%0d | membrane=%0d | spike=%0d | spike_valid=%0d",
            $time,
            current_in,
            current_valid,
            membrane_out,
            spike_out,
            spike_valid
        );
    end

    // ============================================================
    // Send Current Task
    // ============================================================

    task send_current;
        input signed [WIDTH-1:0] val;
        begin

            @(posedge clk);

            current_in    <= val;
            current_valid <= 1;

            @(posedge clk);

            current_valid <= 0;
            current_in    <= 0;
        end
    endtask

    // ============================================================
    // Test
    // ============================================================

    initial begin

        clk = 0;
        rst = 1;

        current_in = 0;
        current_valid = 0;

        // --------------------------------------------------------
        // Reset
        // --------------------------------------------------------

        #20;
        rst = 0;

        #20;

        $display("\n======================================");
        $display("TEST 1 : Small accumulation");
        $display("======================================");

        send_current(3);
        send_current(3);

        #40;

        $display("\n======================================");
        $display("TEST 2 : Trigger spike");
        $display("======================================");

        // should exceed threshold

        send_current(8);

        #40;

        $display("\n======================================");
        $display("TEST 3 : Negative current");
        $display("======================================");

        send_current(-4);

        #40;

        $display("\n======================================");
        $display("SIMULATION COMPLETE");
        $display("======================================");

        #20;
        $finish;
    end

endmodule
