`timescale 1ns / 1ps
module fifo_uart_controller #(
    parameter CLK_FREQ  = 100_000_000, // System clock (Hz)
    parameter BAUD_RATE = 115200,
    parameter FIFO_DEPTH = 1024        // Spike FIFO depth in bytes
)(
    input  wire clk,
    input  wire rst_n,            // Active-low synchronous reset

    // Physical UART pins
    input  wire rx,
    output wire tx,

    // Interface to spike_unpacker
    input  wire       fifo_rd_en,
    output wire [7:0] fifo_dout,
    output wire       fifo_empty,

    // Interface to weight_bram (write port)
    output reg  [11:0] bram_addr,
    output reg  [15:0] bram_din,   // Two bytes assembled into one 16-bit word
    output reg         bram_wr_en,

    // Status
    output reg  weights_loaded     // Stays high after all weights received
);

    // -------------------------------------------------------------------------
    // 1. BAUD RATE GENERATOR
    // -------------------------------------------------------------------------
    localparam BAUD_DIV = CLK_FREQ / BAUD_RATE;  // ~868 for 100MHz/115200

    reg [15:0] baud_cnt;
    wire baud_tick = (baud_cnt == BAUD_DIV - 1);

    always @(posedge clk) begin
        if (!rst_n || baud_tick) baud_cnt <= 0;
        else                     baud_cnt <= baud_cnt + 1;
    end

    // -------------------------------------------------------------------------
    // 2. UART RX - 8N1 (no parity, 1 stop bit)
    // -------------------------------------------------------------------------
    reg [3:0] rx_state;
    reg [7:0] rx_shift;
    reg       rx_valid;
    reg [7:0] rx_data;
    // Sample at mid-bit: use half-baud counter for start-bit alignment
    reg [15:0] rx_baud_cnt;
    wire rx_mid_tick = (rx_baud_cnt == (BAUD_DIV/2) - 1);
    wire rx_full_tick = (rx_baud_cnt == BAUD_DIV - 1);

    always @(posedge clk) begin
        if (!rst_n) begin
            rx_state    <= 0;
            rx_valid    <= 0;
            rx_baud_cnt <= 0;
        end else begin
            rx_valid <= 0;

            case (rx_state)
                0: begin // IDLE: watch for start bit
                    if (rx == 0) begin
                        rx_state    <= 1;
                        rx_baud_cnt <= 0;  // Start counting from falling edge
                    end
                end

                1: begin // Wait for mid-point of start bit
                    if (rx_mid_tick) begin
                        rx_baud_cnt <= 0;
                        rx_state    <= 2;  // Begin sampling data bits
                    end else rx_baud_cnt <= rx_baud_cnt + 1;
                end

                2,3,4,5,6,7,8,9: begin // 8 data bits (states 2-9)
                    if (rx_full_tick) begin
                        rx_shift    <= {rx, rx_shift[7:1]};  // LSB first
                        rx_baud_cnt <= 0;
                        rx_state    <= rx_state + 1;
                    end else rx_baud_cnt <= rx_baud_cnt + 1;
                end

                10: begin // Stop bit
                    if (rx_full_tick) begin
                        rx_data  <= rx_shift;
                        rx_valid <= 1;
                        rx_state <= 0;
                    end else rx_baud_cnt <= rx_baud_cnt + 1;
                end

                default: rx_state <= 0;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // 3. UART TX - sends ACK byte
    // -------------------------------------------------------------------------
    reg [9:0]  tx_shift;   // {stop, data[7:0], start}
    reg [3:0]  tx_state;
    reg        tx_active;
    reg        tx_start_pulse;

    assign tx = tx_active ? tx_shift[0] : 1'b1;  // Idle high

    always @(posedge clk) begin
        if (!rst_n) begin
            tx_state  <= 0;
            tx_active <= 0;
        end else begin
            if (tx_start_pulse && !tx_active) begin
                tx_shift  <= {1'b1, 8'h01, 1'b0};  // Stop + ACK(0x01) + Start
                tx_active <= 1;
                tx_state  <= 0;
            end else if (tx_active && baud_tick) begin
                tx_shift <= {1'b1, tx_shift[9:1]};  // Shift out LSB first
                if (tx_state == 9) tx_active <= 0;
                else               tx_state  <= tx_state + 1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 4. SPIKE FIFO (synchronous, power-of-2 depth)
    // -------------------------------------------------------------------------
    localparam FIFO_BITS = 10;  // log2(1024)

    reg [7:0]          fifo_mem [0:FIFO_DEPTH-1];
    reg [FIFO_BITS:0]  wr_ptr;   // One extra bit to distinguish full vs empty
    reg [FIFO_BITS:0]  rd_ptr;
    wire [FIFO_BITS-1:0] wr_addr = wr_ptr[FIFO_BITS-1:0];
    wire [FIFO_BITS-1:0] rd_addr = rd_ptr[FIFO_BITS-1:0];

    wire fifo_full  = (wr_ptr[FIFO_BITS] != rd_ptr[FIFO_BITS]) &&
                      (wr_ptr[FIFO_BITS-1:0] == rd_ptr[FIFO_BITS-1:0]);
    assign fifo_empty = (wr_ptr == rd_ptr);
    assign fifo_dout  = fifo_mem[rd_addr];

    reg fifo_wr_en_int;
    reg [7:0] fifo_din_int;

    always @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
        end else begin
            if (fifo_wr_en_int && !fifo_full)
                fifo_mem[wr_addr] <= fifo_din_int;
            if (fifo_wr_en_int && !fifo_full) wr_ptr <= wr_ptr + 1;
            if (fifo_rd_en && !fifo_empty)    rd_ptr <= rd_ptr + 1;
        end
    end

    // -------------------------------------------------------------------------
    // 5. WEIGHT BYTE ASSEMBLER
    //    Weights are 16-bit; PC sends them as 2 bytes (MSB first).
    //    We assemble pairs of bytes before writing to BRAM.
    // -------------------------------------------------------------------------
    reg [7:0]  weight_hi;          // High byte latch
    reg        weight_hi_valid;    // Waiting for low byte

    // Total weight bytes = N * (W/8) = 4096 * 2 = 8192
    localparam WEIGHT_BYTES = 8192;
    reg [13:0] weight_byte_cnt;   // Counts up to 8192

    // -------------------------------------------------------------------------
    // 6. CONTROLLER FSM
    // -------------------------------------------------------------------------
    localparam IDLE_ST      = 3'd0;
    localparam RCV_WEIGHTS  = 3'd1;
    localparam SEND_ACK     = 3'd2;
    localparam RCV_SPIKES   = 3'd3;

    reg [2:0]  ctrl_state;
    reg [9:0]  spike_byte_cnt;  // Counts bytes in current 512-byte chunk

    always @(posedge clk) begin
        if (!rst_n) begin
            ctrl_state      <= IDLE_ST;
            fifo_wr_en_int  <= 0;
            bram_wr_en      <= 0;
            bram_addr       <= 0;
            bram_din        <= 0;
            tx_start_pulse  <= 0;
            spike_byte_cnt  <= 0;
            weight_byte_cnt <= 0;
            weight_hi_valid <= 0;
            weight_hi       <= 0;
            weights_loaded  <= 0;
        end else begin
            // Defaults (pulsed signals cleared each cycle)
            fifo_wr_en_int <= 0;
            bram_wr_en     <= 0;
            tx_start_pulse <= 0;

            case (ctrl_state)
                IDLE_ST: begin
                    if (rx_valid) begin
                        if      (rx_data == 8'hAA) ctrl_state <= RCV_WEIGHTS;
                        else if (rx_data == 8'hBB) ctrl_state <= SEND_ACK;
                    end
                end

                // ---- Weight loading ----------------------------------------
                RCV_WEIGHTS: begin
                    if (rx_valid && weight_byte_cnt < WEIGHT_BYTES) begin
                        if (!weight_hi_valid) begin
                            // First byte of a weight word (MSB)
                            weight_hi       <= rx_data;
                            weight_hi_valid <= 1;
                        end else begin
                            // Second byte (LSB): assemble word and write BRAM
                            bram_din        <= {weight_hi, rx_data};
                            bram_wr_en      <= 1;
                            bram_addr       <= bram_addr + 1;
                            weight_hi_valid <= 0;
                            weight_byte_cnt <= weight_byte_cnt + 2;
                        end
                    end

                    if (weight_byte_cnt >= WEIGHT_BYTES) begin
                        weights_loaded <= 1;
                        ctrl_state     <= IDLE_ST;
                    end
                end

                // ---- ACK: tell PC to send next spike chunk -----------------
                SEND_ACK: begin
                    if (!tx_active) begin
                        tx_start_pulse <= 1;
                        ctrl_state     <= RCV_SPIKES;
                        spike_byte_cnt <= 0;
                    end
                end

                // ---- Spike reception (512 bytes = 4096 spikes per timestep) -
                RCV_SPIKES: begin
                    if (rx_valid) begin
                        fifo_din_int   <= rx_data;
                        fifo_wr_en_int <= 1;
                        spike_byte_cnt <= spike_byte_cnt + 1;

                        if (spike_byte_cnt == 10'd511) begin
                            // Chunk complete; send ACK and wait for next chunk
                            ctrl_state <= SEND_ACK;
                        end
                    end
                end

                default: ctrl_state <= IDLE_ST;
            endcase
        end
    end

endmodule
