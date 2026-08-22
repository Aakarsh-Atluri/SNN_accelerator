`timescale 1ns / 1ps
// =============================================================================
// Module: fifo_uart_controller
// Description: UART RX/TX controller with Spike FIFO and BRAM Weight Loader.
//              Handles:
//                1. Receiving 8192 bytes of weights (0xAA header) into BRAM,
//                   then acknowledging with 0x01 ACK and returning to IDLE.
//                2. Receiving 0xBB command, acknowledging with 0x01 ACK, and
//                   streaming 25 timesteps of 512 bytes into FIFO with per-block ACKs.
//                3. Transmitting classification result byte from Master FSM.
//                4. Automatically returning to IDLE after 25 timesteps.
// =============================================================================
module fifo_uart_controller #(
    parameter CLK_FREQ   = 100_000_000, // System clock (Hz)
    parameter BAUD_RATE  = 115200,
    parameter FIFO_DEPTH = 1024         // Spike FIFO depth in bytes
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

    // Interface from Master FSM for UART TX
    input  wire       tx_send_fsm,
    input  wire [7:0] tx_data_fsm,
    output wire       tx_busy,

    // Status
    output reg         weights_loaded  // Stays high after all weights received
);

    // -------------------------------------------------------------------------
    // 1. BAUD RATE PARAMETERS
    // -------------------------------------------------------------------------
    localparam [15:0] BAUD_DIV  = CLK_FREQ / BAUD_RATE;        // 100MHz / 115200 = 868
    localparam [15:0] BAUD_HALF = (CLK_FREQ / BAUD_RATE) / 2;  // 434

    // -------------------------------------------------------------------------
    // 2. UART RX - 8N1 with 2-Stage Input Synchronizer
    // -------------------------------------------------------------------------
    reg rx_sync0;
    reg rx_sync1;

    always @(posedge clk) begin
        if (!rst_n) begin
            rx_sync0 <= 1'b1;
            rx_sync1 <= 1'b1;
        end else begin
            rx_sync0 <= rx;
            rx_sync1 <= rx_sync0;
        end
    end

    reg [3:0]  rx_state;
    reg [7:0]  rx_shift;
    reg        rx_valid;
    reg [7:0]  rx_data;
    reg [15:0] rx_baud_cnt;

    wire rx_mid_tick  = (rx_baud_cnt == (BAUD_HALF - 16'd1));
    wire rx_full_tick = (rx_baud_cnt == (BAUD_DIV - 16'd1));

    always @(posedge clk) begin
        if (!rst_n) begin
            rx_state    <= 4'd0;
            rx_valid    <= 1'b0;
            rx_data     <= 8'd0;
            rx_shift    <= 8'd0;
            rx_baud_cnt <= 16'd0;
        end else begin
            rx_valid <= 1'b0;

            case (rx_state)
                4'd0: begin // IDLE: watch for start bit falling edge
                    if (rx_sync1 == 1'b0) begin
                        rx_state    <= 4'd1;
                        rx_baud_cnt <= 16'd0;
                    end
                end

                4'd1: begin // Wait for mid-point of start bit
                    if (rx_mid_tick) begin
                        if (rx_sync1 == 1'b0) begin // Valid start bit confirmed
                            rx_baud_cnt <= 16'd0;
                            rx_state    <= 4'd2;
                        end else begin
                            rx_state    <= 4'd0; // False start glitch
                        end
                    end else begin
                        rx_baud_cnt <= rx_baud_cnt + 16'd1;
                    end
                end

                4'd2, 4'd3, 4'd4, 4'd5, 4'd6, 4'd7, 4'd8, 4'd9: begin // 8 data bits (LSB first)
                    if (rx_full_tick) begin
                        rx_shift    <= {rx_sync1, rx_shift[7:1]};
                        rx_baud_cnt <= 16'd0;
                        rx_state    <= rx_state + 4'd1;
                    end else begin
                        rx_baud_cnt <= rx_baud_cnt + 16'd1;
                    end
                end

                4'd10: begin // Stop bit
                    if (rx_full_tick) begin
                        if (rx_sync1 == 1'b1) begin // Valid stop bit
                            rx_data  <= rx_shift;
                            rx_valid <= 1'b1;
                        end
                        rx_state <= 4'd0;
                    end else begin
                        rx_baud_cnt <= rx_baud_cnt + 16'd1;
                    end
                end

                default: rx_state <= 4'd0;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // 3. UART TX ENGINE (Handles both ACK bytes and FSM results)
    // -------------------------------------------------------------------------
    reg [9:0]  tx_shift;   // {stop(1), data[7:0], start(0)}
    reg [3:0]  tx_state;
    reg        tx_active;
    reg [15:0] tx_baud_cnt;

    reg        tx_start_req;
    reg [7:0]  tx_byte_in;

    assign tx      = tx_active ? tx_shift[0] : 1'b1;
    assign tx_busy = tx_active || tx_start_req;

    always @(posedge clk) begin
        if (!rst_n) begin
            tx_state    <= 4'd0;
            tx_active   <= 1'b0;
            tx_shift    <= 10'h3FF;
            tx_baud_cnt <= 16'd0;
        end else begin
            if (tx_start_req && !tx_active) begin
                tx_shift    <= {1'b1, tx_byte_in, 1'b0};  // Stop(1) + Data(8) + Start(0)
                tx_active   <= 1'b1;
                tx_state    <= 4'd0;
                tx_baud_cnt <= 16'd0;                     // Reset baud counter for full start bit
            end else if (tx_active) begin
                if (tx_baud_cnt == (BAUD_DIV - 16'd1)) begin
                    tx_baud_cnt <= 16'd0;
                    tx_shift    <= {1'b1, tx_shift[9:1]};  // Shift out LSB first
                    if (tx_state == 4'd9) begin
                        tx_active <= 1'b0;
                    end else begin
                        tx_state  <= tx_state + 4'd1;
                    end
                end else begin
                    tx_baud_cnt <= tx_baud_cnt + 16'd1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // 4. SPIKE FIFO (Distributed RAM / LUTRAM)
    // -------------------------------------------------------------------------
    localparam FIFO_BITS = 10;  // log2(1024)

    reg [7:0]            fifo_mem [0:FIFO_DEPTH-1];
    reg [FIFO_BITS:0]    wr_ptr;   // 11 bits
    reg [FIFO_BITS:0]    rd_ptr;   // 11 bits
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
            wr_ptr <= 11'd0;
            rd_ptr <= 11'd0;
        end else begin
            if (fifo_wr_en_int && !fifo_full)
                fifo_mem[wr_addr] <= fifo_din_int;
            if (fifo_wr_en_int && !fifo_full) wr_ptr <= wr_ptr + 11'd1;
            if (fifo_rd_en && !fifo_empty)    rd_ptr <= rd_ptr + 11'd1;
        end
    end

    // -------------------------------------------------------------------------
    // 5. WEIGHT BYTE ASSEMBLER & CONTROLLER FSM
    // -------------------------------------------------------------------------
    reg [7:0]  weight_hi;
    reg        weight_hi_valid;

    localparam [13:0] WEIGHT_BYTES = 14'd8192; // 4096 weights * 2 bytes
    reg [13:0] weight_byte_cnt;

    localparam [2:0] IDLE_ST              = 3'd0;
    localparam [2:0] RCV_WEIGHTS          = 3'd1;
    localparam [2:0] SEND_ACK_THEN_IDLE   = 3'd2;
    localparam [2:0] SEND_ACK_THEN_SPIKES = 3'd3;
    localparam [2:0] RCV_SPIKES           = 3'd4;

    reg [2:0]  ctrl_state;
    reg [9:0]  spike_byte_cnt;
    reg [4:0]  timestep_cnt;

    // Buffer for FSM result TX if TX is currently busy
    reg        fsm_tx_pending;
    reg [7:0]  fsm_tx_data_latched;

    always @(posedge clk) begin
        if (!rst_n) begin
            ctrl_state          <= IDLE_ST;
            fifo_wr_en_int      <= 1'b0;
            fifo_din_int        <= 8'd0;
            bram_wr_en          <= 1'b0;
            bram_addr           <= 12'd0;
            bram_din            <= 16'd0;
            spike_byte_cnt      <= 10'd0;
            timestep_cnt        <= 5'd0;
            weight_byte_cnt     <= 14'd0;
            weight_hi_valid     <= 1'b0;
            weight_hi           <= 8'd0;
            weights_loaded      <= 1'b0;
            tx_start_req        <= 1'b0;
            tx_byte_in          <= 8'd0;
            fsm_tx_pending      <= 1'b0;
            fsm_tx_data_latched <= 8'd0;
        end else begin
            fifo_wr_en_int <= 1'b0;
            bram_wr_en     <= 1'b0;
            tx_start_req   <= 1'b0;

            // Latch FSM TX request if it arrives
            if (tx_send_fsm) begin
                fsm_tx_pending      <= 1'b1;
                fsm_tx_data_latched <= tx_data_fsm;
            end

            case (ctrl_state)
                IDLE_ST: begin
                    // Priority 1: Service pending FSM result TX
                    if (fsm_tx_pending && !tx_active && !tx_start_req) begin
                        tx_byte_in     <= fsm_tx_data_latched;
                        tx_start_req   <= 1'b1;
                        fsm_tx_pending <= 1'b0;
                    end

                    // Priority 2: UART RX commands
                    if (rx_valid) begin
                        if (rx_data == 8'hAA) begin
                            ctrl_state      <= RCV_WEIGHTS;
                            weight_byte_cnt <= 14'd0;
                            weight_hi_valid <= 1'b0;
                            bram_addr       <= 12'd0;
                            weights_loaded  <= 1'b0;
                        end else if (rx_data == 8'hBB) begin
                            timestep_cnt    <= 5'd0;
                            ctrl_state      <= SEND_ACK_THEN_SPIKES;
                        end
                    end
                end

                RCV_WEIGHTS: begin
                    if (rx_valid && (weight_byte_cnt < WEIGHT_BYTES)) begin
                        if (!weight_hi_valid) begin
                            weight_hi       <= rx_data;
                            weight_hi_valid <= 1'b1;
                        end else begin
                            bram_din        <= {weight_hi, rx_data};
                            bram_wr_en      <= 1'b1;
                            bram_addr       <= bram_addr + 12'd1;
                            weight_hi_valid <= 1'b0;
                            weight_byte_cnt <= weight_byte_cnt + 14'd2;

                            if (weight_byte_cnt + 14'd2 == WEIGHT_BYTES) begin
                                weights_loaded <= 1'b1;
                                ctrl_state     <= SEND_ACK_THEN_IDLE; // ACK weights and return to IDLE
                            end
                        end
                    end
                end

                SEND_ACK_THEN_IDLE: begin
                    if (!tx_active && !tx_start_req) begin
                        tx_byte_in   <= 8'h01; // SNN_RESP_ACK
                        tx_start_req <= 1'b1;
                        ctrl_state   <= IDLE_ST;
                    end
                end

                SEND_ACK_THEN_SPIKES: begin
                    if (!tx_active && !tx_start_req) begin
                        tx_byte_in     <= 8'h01; // SNN_RESP_ACK
                        tx_start_req   <= 1'b1;
                        ctrl_state     <= RCV_SPIKES;
                        spike_byte_cnt <= 10'd0;
                    end
                end

                RCV_SPIKES: begin
                    // If FSM finished and wants to send classification result while in RCV_SPIKES
                    if (fsm_tx_pending && !tx_active && !tx_start_req) begin
                        tx_byte_in     <= fsm_tx_data_latched;
                        tx_start_req   <= 1'b1;
                        fsm_tx_pending <= 1'b0;
                    end

                    if (rx_valid) begin
                        fifo_din_int   <= rx_data;
                        fifo_wr_en_int <= 1'b1;
                        spike_byte_cnt <= spike_byte_cnt + 10'd1;

                        if (spike_byte_cnt == 10'd511) begin
                            if (timestep_cnt == 5'd24) begin
                                timestep_cnt <= 5'd0;
                                ctrl_state   <= SEND_ACK_THEN_IDLE; // Final timestep ACK & return to IDLE
                            end else begin
                                timestep_cnt <= timestep_cnt + 5'd1;
                                ctrl_state   <= SEND_ACK_THEN_SPIKES;
                            end
                        end
                    end
                end

                default: ctrl_state <= IDLE_ST;
            endcase
        end
    end

endmodule
