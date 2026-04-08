`timescale 1ns / 1ps

module spike_unpacker (
    input  wire       clk,
    input  wire       rst_n,        // Active-low synchronous reset

    // Interface to FIFO (from fifo_uart_controller)
    input  wire       fifo_empty,
    input  wire [7:0] fifo_dout,
    output reg        fifo_rd_en,

    // Interface to cascaded_adder
    output reg        spike_out,    // Individual spike bit
    output reg        spike_valid   // High for 1 cycle per valid spike
);

    reg [2:0] bit_counter;
    reg [7:0] current_byte;

    localparam IDLE       = 2'd0;
    localparam READ_FIFO  = 2'd1;
    localparam SHIFT_BITS = 2'd2;

    reg [1:0] state;

    always @(posedge clk) begin
        if (!rst_n) begin
            state       <= IDLE;
            fifo_rd_en  <= 0;
            spike_valid <= 0;
            spike_out   <= 0;
            bit_counter <= 0;
        end else begin
            fifo_rd_en  <= 0;
            spike_valid <= 0;

            case (state)
                IDLE: begin
                    if (!fifo_empty) begin
                        fifo_rd_en <= 1;   // Request byte from FIFO
                        state      <= READ_FIFO;
                    end
                end

                READ_FIFO: begin
                    // FIFO output is registered: data appears the cycle after rd_en
                    current_byte <= fifo_dout;
                    bit_counter  <= 0;     // Start from LSB (bit 0)
                    state        <= SHIFT_BITS;
                end

                SHIFT_BITS: begin
                    spike_out   <= current_byte[bit_counter];  // Output LSB-first
                    spike_valid <= 1;

                    if (bit_counter == 3'd7) begin
                        state <= IDLE;     // All 8 bits done, fetch next byte
                    end else begin
                        bit_counter <= bit_counter + 1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule