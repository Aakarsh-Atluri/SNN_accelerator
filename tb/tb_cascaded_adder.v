`timescale 1ns/1ps

module tb_cascaded_adder;

    parameter N      = 4;
    parameter W      = 16;
    parameter OUT    = 32;
    parameter ADDR_W = 2;

    reg clk;
    reg rst;

    reg start;

    reg spike_in;
    reg spike_valid;

    wire [ADDR_W-1:0] weight_addr;
    reg signed [W-1:0] weight_data;

    reg signed [W-1:0] bias;

    wire signed [OUT-1:0] result;

    wire busy;
    wire valid;

    integer expected;

    // ============================================================
    // DUT
    // ============================================================

    cascaded_adder #(
        .N     (N),
        .W     (W),
        .OUT   (OUT),
        .ADDR_W(ADDR_W)
    ) dut (
        .clk        (clk),
        .rst        (rst),
        .start      (start),
        .busy       (busy),
        .valid      (valid),
        .spike_in   (spike_in),
        .spike_valid(spike_valid),
        .weight_addr(weight_addr),
        .weight_data(weight_data),
        .bias       (bias),
        .result     (result)
    );

    // ============================================================
    // Clock
    // ============================================================

    always #5 clk = ~clk;

    // ============================================================
    // BRAM MODEL (Synchronous 1-cycle latency matching weight_bram)
    // ============================================================

    reg signed [W-1:0] mem [0:N-1];

    always @(posedge clk) begin
        weight_data <= mem[weight_addr];
    end

    // ============================================================
    // Debug
    // ============================================================

    always @(posedge clk) begin
        $display(
            "[TB] time=%0t | addr=%0d | weight=%0d | spike=%0d | valid=%0d | result=%0d",
            $time,
            weight_addr,
            weight_data,
            spike_in,
            spike_valid,
            result
        );
    end

    // ============================================================
    // Test
    // ============================================================

    initial begin

        clk = 0;
        rst = 1;

        start = 0;

        spike_in = 0;
        spike_valid = 0;

        bias = 0;

        expected = 8;

        // weights = [1,2,3,4]
        mem[0] = 1;
        mem[1] = 2;
        mem[2] = 3;
        mem[3] = 4;
        weight_data = 0;

        // reset
        #20;
        rst = 0;

        #20;

        // ========================================================
        // START
        // ========================================================

        @(posedge clk);
        start <= 1;

        @(posedge clk);
        start <= 0;

        // ========================================================
        // Continuous spike stream
        // spikes = [1,0,1,1]
        // expected = 1 + 0 + 3 + 4 = 8
        // ========================================================

        @(posedge clk);

        spike_valid <= 1;

        spike_in <= 1;
        @(posedge clk);

        spike_in <= 0;
        @(posedge clk);

        spike_in <= 1;
        @(posedge clk);

        spike_in <= 1;
        @(posedge clk);

        spike_valid <= 0;
        spike_in    <= 0;

        // ========================================================
        // WAIT FOR RESULT
        // ========================================================

        wait(valid);

        #10;

        $display("\n======================================");
        $display("EXPECTED = %0d", expected);
        $display("RESULT   = %0d", result);

        if (result == expected)
            $display("TEST PASSED");
        else
            $display("TEST FAILED");

        $display("======================================\n");

        #20;
        $finish;
    end

endmodule
