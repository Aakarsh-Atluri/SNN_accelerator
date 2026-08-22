`timescale 1ns / 1ps
// =============================================================================
// Module: weight_bram
// Description: Dual-port Block RAM for storing synaptic weights.
//              Port A: Write-only (used by fifo_uart_controller during loading)
//              Port B: Read-only  (used by cascaded_adder during inference)
//
//              Depth = N = 4096 entries, each 16 bits wide.
//              Total size = 4096 * 16 = 65536 bits = 8 KB — fits in Artix-7 BRAM.
//
// Note: Infer as Block RAM by keeping address and data registered.
//       Quartus/Vivado will automatically map this to BRAM primitives.
// =============================================================================
module weight_bram #(
    parameter DEPTH = 4096,   // Number of weight entries (one per neuron)
    parameter WIDTH = 16,     // Bits per weight (signed)
    parameter ABITS = 12      // Address bits (2^12 = 4096)
)(
    // Port A: Write port (weight loading via UART)
    input  wire              clk_a,
    input  wire              we_a,
    input  wire [ABITS-1:0]  addr_a,
    input  wire [WIDTH-1:0]  din_a,

    // Port B: Read port (inference by cascaded_adder)
    input  wire              clk_b,
    input  wire [ABITS-1:0]  addr_b,
    output reg  [WIDTH-1:0]  dout_b
);

    // Inferred BRAM storage
    reg [WIDTH-1:0] mem [0:DEPTH-1];

    // Port A: synchronous write
    always @(posedge clk_a) begin
        if (we_a)
            mem[addr_a] <= din_a;
    end

    // Port B: synchronous read (1-cycle read latency — matches cascaded_adder design)
    always @(posedge clk_b) begin
        dout_b <= mem[addr_b];
    end

    // Optional: pre-load with zeros (synthesis will ignore this for BRAM)
    integer i;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1)
            mem[i] = 0;
    end

endmodule
