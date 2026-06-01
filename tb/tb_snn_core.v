`timescale 1ns / 1ps
// =============================================================================
// Testbench : tb_snn_core
// Description: Comprehensive self-checking testbench for the SNN accelerator
//              inference pipeline. Bypasses UART to directly load weights and
//              spike data, verifying MAC, LIF, and classification outputs.
//
// Simulator:   Vivado Behavioral Simulation (xsim)
//
// Parameters (overridden for fast simulation):
//   N          = 8    (8 input neurons instead of 4096)
//   T_WINDOW   = 4    (4 timesteps instead of 25)
//   THRESHOLD  = 25
//   LEAK_SHIFT = 3
//
// Test Scenario:
//   Weights = {10, 20, -5, 15, 30, -10, 25, 5}  (sum = 90)
//   Timesteps 0-2: all spikes = 1 -> MAC = 90, LIF fires (90 >= 25)
//   Timestep  3:   all spikes = 0 -> MAC = 0,  LIF silent
//   lif_spike_accum = 3 > SPIKE_THRESHOLD(2) -> class_out = 1
// =============================================================================

module tb_snn_core;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam N          = 8;
    localparam W          = 16;
    localparam OUT        = 32;
    localparam ADDR_W     = 3;       // log2(N)
    localparam T_WINDOW   = 4;
    localparam THRESHOLD  = 25;
    localparam LEAK_SHIFT = 3;
    localparam RESET_VAL  = 0;

    // FSM state encoding (mirrors master_fsm localparams)
    localparam [3:0] S_WAIT_SPIKES = 4'd2;

    // =========================================================================
    // Clock and Reset
    // =========================================================================
    reg clk;
    reg rst_n;

    initial clk = 0;
    always #5 clk = ~clk;          // 100 MHz, 10 ns period

    // =========================================================================
    // Weight Data (signed 16-bit)
    // =========================================================================
    reg signed [W-1:0] weight_values [0:N-1];

    initial begin
        weight_values[0] =  16'sd10;
        weight_values[1] =  16'sd20;
        weight_values[2] = -16'sd5;
        weight_values[3] =  16'sd15;
        weight_values[4] =  16'sd30;
        weight_values[5] = -16'sd10;
        weight_values[6] =  16'sd25;
        weight_values[7] =  16'sd5;
        // Sum when all spikes = 1: 10+20-5+15+30-10+25+5 = 90
    end

    // =========================================================================
    // Spike Data (1 byte per timestep for N=8)
    // =========================================================================
    reg [7:0] spike_bytes [0:T_WINDOW-1];

    initial begin
        spike_bytes[0] = 8'hFF;    // All spikes -> MAC=90 -> LIF spike
        spike_bytes[1] = 8'hFF;    // All spikes -> MAC=90 -> LIF spike
        spike_bytes[2] = 8'hFF;    // All spikes -> MAC=90 -> LIF spike
        spike_bytes[3] = 8'h00;    // No spikes  -> MAC=0  -> LIF silent
    end

    // =========================================================================
    // BRAM Write Port Signals (weight loading from testbench)
    // =========================================================================
    reg                bram_we_a;
    reg [ADDR_W-1:0]   bram_addr_a;
    reg [W-1:0]        bram_din_a;

    // =========================================================================
    // Testbench FIFO Model (replaces fifo_uart_controller's internal FIFO)
    // =========================================================================
    reg  [7:0]  fifo_mem [0:15];
    reg  [4:0]  wr_ptr, rd_ptr;
    wire        fifo_empty;
    wire [7:0]  fifo_dout;
    wire        fifo_rd_en;            // driven by spike_unpacker

    reg         fifo_wr_en;
    reg  [7:0]  fifo_din;

    assign fifo_empty = (wr_ptr == rd_ptr);
    assign fifo_dout  = fifo_mem[rd_ptr[3:0]];

    always @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= 5'd0;
            rd_ptr <= 5'd0;
        end else begin
            if (fifo_wr_en) begin
                fifo_mem[wr_ptr[3:0]] <= fifo_din;
                wr_ptr <= wr_ptr + 5'd1;
            end
            if (fifo_rd_en && !fifo_empty)
                rd_ptr <= rd_ptr + 5'd1;
        end
    end

    // =========================================================================
    // weights_loaded control signal
    // =========================================================================
    reg weights_loaded;

    // =========================================================================
    // Interconnect Wires
    // =========================================================================

    // Spike unpacker -> cascaded_adder / master_fsm
    wire        spike_valid;
    wire        spike_bit;

    // Cascaded adder <-> weight BRAM
    wire [ADDR_W-1:0]      adder_weight_addr;
    wire signed [W-1:0]    bram_dout;

    // Cascaded adder outputs
    wire                   adder_busy;
    wire                   adder_valid;
    wire signed [OUT-1:0]  adder_result;

    // Master FSM -> cascaded adder
    wire adder_start;

    // Master FSM -> LIF neuron
    wire lif_current_valid;

    // LIF neuron outputs
    wire        lif_spike_out;
    wire        lif_spike_valid;
    wire signed [OUT-1:0] lif_membrane_out;

    // Master FSM -> spike counter
    wire sc_start;
    wire sc_spike_valid;

    // Spike counter outputs
    wire [5:0]  sc_count_1, sc_count_0;
    wire        sc_result;
    wire        sc_done;

    // Master FSM outputs
    wire [7:0]  tx_data;
    wire        tx_send;
    wire        inference_done;
    wire        class_out;

    // =========================================================================
    // DUT Module Instantiations
    // =========================================================================

    // ---- Weight BRAM --------------------------------------------------------
    weight_bram #(
        .DEPTH (N),
        .WIDTH (W),
        .ABITS (ADDR_W)
    ) u_weight_bram (
        .clk_a  (clk),
        .we_a   (bram_we_a),
        .addr_a (bram_addr_a),
        .din_a  (bram_din_a),
        .clk_b  (clk),
        .addr_b (adder_weight_addr),
        .dout_b (bram_dout)
    );

    // ---- Spike Unpacker -----------------------------------------------------
    spike_unpacker u_unpacker (
        .clk        (clk),
        .rst_n      (rst_n),
        .fifo_empty (fifo_empty),
        .fifo_dout  (fifo_dout),
        .fifo_rd_en (fifo_rd_en),
        .spike_out  (spike_bit),
        .spike_valid(spike_valid)
    );

    // ---- Cascaded Adder (MAC) -----------------------------------------------
    cascaded_adder #(
        .N      (N),
        .W      (W),
        .OUT    (OUT),
        .ADDR_W (ADDR_W)
    ) u_mac (
        .clk         (clk),
        .rst         (~rst_n),             // active-high async reset
        .start       (adder_start),
        .busy        (adder_busy),
        .valid       (adder_valid),
        .spike_in    (spike_bit),
        .spike_valid (spike_valid),
        .weight_addr (adder_weight_addr),
        .weight_data (bram_dout),
        .bias        ({W{1'b0}}),          // zero bias
        .result      (adder_result)
    );

    // ---- LIF Neuron ---------------------------------------------------------
    lif_neuron #(
        .WIDTH      (OUT),
        .THRESHOLD  (THRESHOLD),
        .RESET_VAL  (RESET_VAL),
        .LEAK_SHIFT (LEAK_SHIFT)
    ) u_lif (
        .clk           (clk),
        .rst           (~rst_n),           // active-high async reset
        .current_in    (adder_result),
        .current_valid (lif_current_valid),
        .spike_out     (lif_spike_out),
        .spike_valid   (lif_spike_valid),
        .membrane_out  (lif_membrane_out)
    );

    // ---- Spike Counter (Rate Decoder) ---------------------------------------
    spike_counter #(
        .T_WINDOW (T_WINDOW)
    ) u_sc (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (sc_start),
        .spike_in    (lif_spike_out),
        .spike_valid (sc_spike_valid),
        .count_1     (sc_count_1),
        .count_0     (sc_count_0),
        .result      (sc_result),
        .done        (sc_done)
    );

    // ---- Master FSM ---------------------------------------------------------
    master_fsm #(
        .N        (N),
        .T_WINDOW (T_WINDOW)
    ) u_fsm (
        .clk               (clk),
        .rst_n             (rst_n),
        .fifo_empty        (fifo_empty),
        .weights_loaded    (weights_loaded),
        .spike_valid_in    (spike_valid),
        .spike_bit         (spike_bit),
        .adder_start       (adder_start),
        .adder_valid       (adder_valid),
        .lif_current_valid (lif_current_valid),
        .lif_spike_out     (lif_spike_out),
        .sc_start          (sc_start),
        .sc_spike_valid    (sc_spike_valid),
        .sc_done           (sc_done),
        .sc_result         (sc_result),
        .tx_data           (tx_data),
        .tx_send           (tx_send),
        .inference_done    (inference_done),
        .class_out         (class_out)
    );

    // =========================================================================
    // FSM State Monitor (hierarchical reference for synchronization)
    // =========================================================================
    wire [3:0] fsm_state = u_fsm.state;

    // =========================================================================
    // Spike Feeder
    //   Loads one byte into the FIFO per timestep, synchronized with FSM.
    //   Only fires when:
    //     - weights are loaded
    //     - FIFO is empty (previous data consumed)
    //     - FSM is in WAIT_SPIKES (ready for next timestep)
    //     - Not already writing (prevents double-write due to FIFO latency)
    //     - More timesteps remain
    // =========================================================================
    reg [2:0] ts_idx;

    always @(posedge clk) begin
        if (!rst_n) begin
            ts_idx     <= 3'd0;
            fifo_wr_en <= 1'b0;
            fifo_din   <= 8'd0;
        end else if (weights_loaded && fifo_empty && !fifo_wr_en &&
                     (fsm_state == S_WAIT_SPIKES) &&
                     (ts_idx < T_WINDOW)) begin
            fifo_wr_en <= 1'b1;
            fifo_din   <= spike_bytes[ts_idx];
            ts_idx     <= ts_idx + 3'd1;
            $display("[%0t] FIFO <- spike_byte[%0d] = 0x%02h",
                     $time, ts_idx, spike_bytes[ts_idx]);
        end else begin
            fifo_wr_en <= 1'b0;
        end
    end

    // =========================================================================
    // Result Capture — latch each MAC result for post-run verification
    // =========================================================================
    reg signed [OUT-1:0] mac_results [0:T_WINDOW-1];
    reg [2:0] mac_idx;

    always @(posedge clk) begin
        if (!rst_n) begin
            mac_idx <= 3'd0;
        end else if (adder_valid && mac_idx < T_WINDOW) begin
            mac_results[mac_idx] <= adder_result;
            mac_idx <= mac_idx + 3'd1;
        end
    end

    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    integer i;
    integer pass_count;
    integer fail_count;
    integer timeout_cnt;

    initial begin

        // --- Initialization ---
        rst_n          = 0;
        bram_we_a      = 0;
        bram_addr_a    = 0;
        bram_din_a     = 0;
        weights_loaded = 0;
        fifo_wr_en     = 0;
        fifo_din       = 0;
        pass_count     = 0;
        fail_count     = 0;

        // Initialize memory arrays to avoid X state in waveforms
        for (i = 0; i < 16; i = i + 1) begin
            fifo_mem[i] = 8'd0;
        end
        for (i = 0; i < T_WINDOW; i = i + 1) begin
            mac_results[i] = {OUT{1'b0}};
        end

        $display("");
        $display("==========================================================");
        $display("  SNN Accelerator - Core Pipeline Testbench");
        $display("==========================================================");
        $display("  N=%0d  W=%0d  OUT=%0d  T_WINDOW=%0d", N, W, OUT, T_WINDOW);
        $display("  THRESHOLD=%0d  LEAK_SHIFT=%0d  RESET_VAL=%0d",
                 THRESHOLD, LEAK_SHIFT, RESET_VAL);
        $display("==========================================================");

        // --- Reset phase ---
        repeat (10) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        // =================================================================
        // Phase 1: Load weights into BRAM via write port
        //   Signals are changed at negedge to avoid race conditions with
        //   the BRAM's posedge-triggered always block.
        // =================================================================
        $display("\n[%0t] Phase 1: Loading %0d weights into BRAM...", $time, N);

        for (i = 0; i < N; i = i + 1) begin
            @(negedge clk);
            bram_addr_a = i[ADDR_W-1:0];
            bram_din_a  = weight_values[i];
            bram_we_a   = 1;
            $display("  w[%0d] = %0d", i, $signed(weight_values[i]));
        end
        @(negedge clk);
        bram_we_a = 0;

        repeat (2) @(posedge clk);
        $display("[%0t] Weight loading complete.", $time);

        // =================================================================
        // Phase 2: Assert weights_loaded -> FSM begins
        //   FSM transitions: IDLE -> WAIT_WEIGHTS -> WAIT_SPIKES
        // =================================================================
        @(negedge clk);
        weights_loaded = 1;
        $display("\n[%0t] Phase 2: weights_loaded asserted. FSM running.", $time);

        // =================================================================
        // Phase 3: Wait for inference to complete (with timeout watchdog)
        //   The spike feeder (always block above) automatically loads
        //   one byte per timestep when the FSM reaches WAIT_SPIKES.
        // =================================================================
        $display("\n[%0t] Phase 3: Inference in progress (%0d timesteps)...\n",
                 $time, T_WINDOW);

        timeout_cnt = 0;
        while (!inference_done && timeout_cnt < 5000) begin
            @(posedge clk);
            timeout_cnt = timeout_cnt + 1;
        end

        if (timeout_cnt >= 5000) begin
            $display("");
            $display("[ERROR] Timeout after %0d cycles!", timeout_cnt);
            $display("  FSM state      = %0d", fsm_state);
            $display("  Timestep index = %0d", ts_idx);
            $display("  FIFO empty     = %b", fifo_empty);
            $display("  Adder busy     = %b", adder_busy);
            $finish;
        end

        // Allow class_out and tx_data to settle (they persist, but be safe)
        repeat (2) @(posedge clk);

        // =================================================================
        // Phase 4: Verify Results
        // =================================================================
        $display("");
        $display("==========================================================");
        $display("  VERIFICATION RESULTS");
        $display("==========================================================");

        // --- Check 1: Inference completed ---
        $display("");
        $display("  Check 1: Inference completed");
        $display("    Completed in %0d clock cycles", timeout_cnt);
        pass_count = pass_count + 1;

        // --- Check 2: MAC result for timestep 0 (all spikes) ---
        $display("");
        $display("  Check 2: MAC result (timestep 0, all spikes = 1)");
        $display("    Expected: 90");
        $display("    Got:      %0d", $signed(mac_results[0]));
        if (mac_results[0] == 32'sd90) begin
            $display("    >> PASS");
            pass_count = pass_count + 1;
        end else begin
            $display("    >> FAIL");
            fail_count = fail_count + 1;
        end

        // --- Check 3: MAC result for timestep 3 (no spikes) ---
        $display("");
        $display("  Check 3: MAC result (timestep 3, all spikes = 0)");
        $display("    Expected: 0");
        $display("    Got:      %0d", $signed(mac_results[3]));
        if (mac_results[3] == 32'sd0) begin
            $display("    >> PASS");
            pass_count = pass_count + 1;
        end else begin
            $display("    >> FAIL");
            fail_count = fail_count + 1;
        end

        // --- Check 4: Classification output ---
        $display("");
        $display("  Check 4: Classification output (class_out)");
        $display("    Expected: 1 (collision detected)");
        $display("    Got:      %0d", class_out);
        if (class_out == 1'b1) begin
            $display("    >> PASS");
            pass_count = pass_count + 1;
        end else begin
            $display("    >> FAIL");
            fail_count = fail_count + 1;
        end

        // --- Check 5: TX data byte ---
        $display("");
        $display("  Check 5: UART TX data byte");
        $display("    Expected: 0x01");
        $display("    Got:      0x%02h", tx_data);
        if (tx_data == 8'h01) begin
            $display("    >> PASS");
            pass_count = pass_count + 1;
        end else begin
            $display("    >> FAIL");
            fail_count = fail_count + 1;
        end

        // --- Summary ---
        $display("");
        $display("==========================================================");
        if (fail_count == 0)
            $display("  ALL %0d CHECKS PASSED", pass_count);
        else
            $display("  %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("==========================================================");
        $display("");

        #100;
        $finish;
    end

    // =========================================================================
    // Live Signal Monitor
    // =========================================================================
    always @(posedge clk) begin
        if (rst_n && weights_loaded) begin
            if (adder_start)
                $display("[%0t] >> Timestep %0d: ADDER start",
                         $time, u_fsm.timestep_count);
            if (adder_valid)
                $display("[%0t]    MAC result = %0d",
                         $time, $signed(adder_result));
            if (lif_current_valid)
                $display("[%0t]    LIF input  = %0d",
                         $time, $signed(adder_result));
            if (lif_spike_valid)
                $display("[%0t]    LIF output : spike=%b  membrane=%0d",
                         $time, lif_spike_out, $signed(lif_membrane_out));
            if (inference_done)
                $display("[%0t] ## INFERENCE DONE -> class=%b  tx=0x%02h",
                         $time, class_out, tx_data);
        end
    end

endmodule
