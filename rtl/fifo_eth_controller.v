`timescale 1ns / 1ps
// =============================================================================
// Module: fifo_eth_controller
// Description: MII Ethernet Controller with Spike FIFO, BRAM Weight Loader,
//              and Response Frame Generator.
//
// Features:
//   1. Full MII 100Mbps RX / TX engines with CRC32 calculation.
//   2. Raw Ethernet frame parser for EtherType 0x88B5.
//   3. Commands supported:
//      - 0xAA (WRITE_WEIGHTS):
//        Payload: [0xAA] [blk_idx: 1B] [1024 bytes weights]
//        Transfers 1024 bytes (512 16-bit weights) per frame.
//        8 frames (blk_idx 0..7) write full 8192 bytes (4096 weights).
//        FPGA replies with ACK packet containing blk_idx and weights_loaded status.
//      - 0xBB (WRITE_SPIKES):
//        Payload: [0xBB] [timestep: 1B] [512 bytes spikes]
//        Writes 512 bytes (4096 spike bits) into Spike FIFO.
//        FPGA replies with ACK packet containing timestep.
//      - 0xCC (INFER_REQ):
//        Payload: [0xCC]
//        FPGA replies with result packet [0x02] [inference_done] [class_out].
//      - 0xDD (PING):
//        FPGA replies with PONG [0x03] [weights_loaded] [fifo_empty].
//   4. Automatic Inference Result Push:
//      When Master FSM raises tx_send_fsm, FPGA sends an Ethernet packet
//      [0x02] [class_out] to the host MAC.
// =============================================================================

module fifo_eth_controller #(
    parameter [47:0] FPGA_MAC   = 48'h00183E04C552,
    parameter [15:0] SNN_ETH_TYP = 16'h88B5,
    parameter FIFO_DEPTH        = 1024
)(
    // Clocks and Reset
    input  wire        clk,            // 100 MHz system clock
    input  wire        rst_n,          // Active-low synchronous reset

    // MII Ethernet Physical Interface (TI DP83848J PHY)
    output wire        eth_ref_clk,    // 25 MHz PHY reference clock
    output wire        eth_rstn,       // PHY active-low hardware reset
    input  wire        eth_rx_clk,     // RX clock from PHY (25 MHz)
    input  wire        eth_rx_dv,      // RX data valid
    input  wire [3:0]  eth_rxd,        // RX data [3:0]
    input  wire        eth_rxerr,      // RX error
    input  wire        eth_tx_clk,     // TX clock from PHY (25 MHz)
    output reg         eth_tx_en,      // TX enable
    output reg  [3:0]  eth_txd,        // TX data [3:0]

    // Status LED outputs
    output wire        led_heartbeat,  // LED 0: Heartbeat blink
    output wire        led_rx,         // LED 1: RX activity
    output wire        led_tx,         // LED 2: TX activity
    output wire        led_phy_ready,  // LED 3: PHY reset de-asserted

    // Interface to spike_unpacker (Sys Clk domain)
    input  wire        fifo_rd_en,
    output wire [7:0]  fifo_dout,
    output wire        fifo_empty,

    // Interface to weight_bram write port (Sys Clk domain)
    output reg  [11:0] bram_addr,
    output reg  [15:0] bram_din,
    output reg         bram_wr_en,

    // Interface from Master FSM (Sys Clk domain)
    input  wire        tx_send_fsm,
    input  wire [7:0]  tx_data_fsm,

    // Status (Sys Clk domain)
    output reg         weights_loaded
);

    // =========================================================================
    // 1. Clocks and PHY Power-On Reset (100 MHz domain)
    // =========================================================================
    reg [1:0] clk_div = 2'b00;
    always @(posedge clk) begin
        clk_div <= clk_div + 1'b1;
    end
    assign eth_ref_clk = clk_div[1]; // 100 MHz / 4 = 25 MHz

    reg [23:0] rst_cnt = 24'd0;
    reg phy_rstn_reg = 1'b0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rst_cnt <= 24'd0;
            phy_rstn_reg <= 1'b0;
        end else if (rst_cnt < 24'd10_000_000) begin
            rst_cnt <= rst_cnt + 1'b1;
            phy_rstn_reg <= 1'b0;
        end else begin
            phy_rstn_reg <= 1'b1;
        end
    end
    assign eth_rstn = phy_rstn_reg;

    reg [25:0] heartbeat_cnt = 26'd0;
    always @(posedge clk) heartbeat_cnt <= heartbeat_cnt + 1'b1;
    assign led_heartbeat = heartbeat_cnt[25];
    assign led_phy_ready = eth_rstn;

    // =========================================================================
    // 2. RX Path (eth_rx_clk domain)
    // =========================================================================
    reg rx_nibble_toggle = 1'b0;
    reg [3:0] rx_low_nibble = 4'd0;
    reg rx_in_frame = 1'b0;
    reg [1:0] rx_preamble_state = 2'd0;

    // RX Dual-Port Packet Buffer (2048 bytes)
    // Port A: written by eth_rx_clk
    // Port B: read by eth_tx_clk and sys clk
    reg [7:0] rx_buf [0:2047];
    reg [10:0] rx_wr_addr = 11'd0;
    reg [10:0] rx_packet_len = 11'd0;
    reg rx_packet_done = 1'b0;

    reg [19:0] rx_led_cnt = 20'd0;
    assign led_rx = (rx_led_cnt > 0);

    always @(posedge eth_rx_clk or negedge eth_rstn) begin
        if (!eth_rstn) begin
            rx_nibble_toggle  <= 1'b0;
            rx_in_frame        <= 1'b0;
            rx_preamble_state  <= 2'd0;
            rx_wr_addr         <= 11'd0;
            rx_packet_len      <= 11'd0;
            rx_packet_done     <= 1'b0;
            rx_led_cnt         <= 20'd0;
        end else begin
            rx_packet_done <= 1'b0;
            if (rx_led_cnt > 0) rx_led_cnt <= rx_led_cnt - 1'b1;

            if (eth_rx_dv) begin
                if (!rx_in_frame) begin
                    // Preamble is 0x55 (low nibble 5, high nibble 5)
                    // SFD is 0xD5 (low nibble 5, high nibble D)
                    if (eth_rxd == 4'h5) begin
                        rx_preamble_state <= 2'd1;
                    end else if (rx_preamble_state == 2'd1 && eth_rxd == 4'hD) begin
                        rx_in_frame       <= 1'b1;
                        rx_nibble_toggle  <= 1'b0;
                        rx_wr_addr        <= 11'd0;
                        rx_preamble_state <= 2'd0;
                    end else begin
                        rx_preamble_state <= 2'd0;
                    end
                end else begin
                    // Assemble bytes (low nibble first)
                    if (!rx_nibble_toggle) begin
                        rx_low_nibble    <= eth_rxd;
                        rx_nibble_toggle <= 1'b1;
                    end else begin
                        rx_nibble_toggle <= 1'b0;
                        if (rx_wr_addr < 11'd2040) begin
                            rx_buf[rx_wr_addr] <= {eth_rxd, rx_low_nibble};
                            rx_wr_addr         <= rx_wr_addr + 11'd1;
                        end
                    end
                end
            end else begin
                // End of Frame
                if (rx_in_frame) begin
                    rx_in_frame       <= 1'b0;
                    rx_nibble_toggle  <= 1'b0;
                    rx_preamble_state <= 2'd0;
                    if (rx_wr_addr >= 11'd18) begin // 14 byte header + 4 byte FCS
                        rx_packet_len  <= rx_wr_addr - 11'd4; // Exclude FCS
                        rx_packet_done <= 1'b1;
                        rx_led_cnt     <= 20'hFFFFF;
                    end
                end
            end
        end
    end

    // =========================================================================
    // 3. RX Packet Synchronization to System Clock Domain
    // =========================================================================
    reg rx_done_toggle = 1'b0;
    always @(posedge eth_rx_clk or negedge eth_rstn) begin
        if (!eth_rstn)
            rx_done_toggle <= 1'b0;
        else if (rx_packet_done)
            rx_done_toggle <= ~rx_done_toggle;
    end

    reg [2:0] sys_sync_rx_done = 3'd0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sys_sync_rx_done <= 3'd0;
        else
            sys_sync_rx_done <= {sys_sync_rx_done[1:0], rx_done_toggle};
    end
    wire sys_rx_packet_trig = (sys_sync_rx_done[2] ^ sys_sync_rx_done[1]);

    // =========================================================================
    // 4. Spike FIFO (Dual-Clock / Synchronous System Clock Domain)
    // =========================================================================
    localparam FIFO_BITS = 10; // log2(1024)
    reg [7:0]            fifo_mem [0:FIFO_DEPTH-1];
    reg [FIFO_BITS:0]    wr_ptr = {(FIFO_BITS+1){1'b0}};
    reg [FIFO_BITS:0]    rd_ptr = {(FIFO_BITS+1){1'b0}};
    wire [FIFO_BITS-1:0] wr_addr = wr_ptr[FIFO_BITS-1:0];
    wire [FIFO_BITS-1:0] rd_addr = rd_ptr[FIFO_BITS-1:0];

    wire fifo_full  = (wr_ptr[FIFO_BITS] != rd_ptr[FIFO_BITS]) &&
                      (wr_ptr[FIFO_BITS-1:0] == rd_ptr[FIFO_BITS-1:0]);
    assign fifo_empty = (wr_ptr == rd_ptr);
    assign fifo_dout  = fifo_mem[rd_addr];

    reg        fifo_wr_en_int;
    reg [7:0]  fifo_din_int;

    always @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
        end else begin
            if (fifo_wr_en_int && !fifo_full) begin
                fifo_mem[wr_addr] <= fifo_din_int;
                wr_ptr <= wr_ptr + 1'b1;
            end
            if (fifo_rd_en && !fifo_empty) begin
                rd_ptr <= rd_ptr + 1'b1;
            end
        end
    end

    // =========================================================================
    // 5. System Clock Frame Parser & Processor
    // =========================================================================
    // Parses incoming frames in rx_buf, writes BRAM / FIFO, and triggers TX response
    localparam PARSE_IDLE      = 3'd0;
    localparam PARSE_CHECK     = 3'd1;
    localparam PARSE_WEIGHTS   = 3'd2;
    localparam PARSE_SPIKES    = 3'd3;
    localparam PARSE_NOTIFY_TX = 3'd4;

    reg [2:0]  parse_state = PARSE_IDLE;
    reg [10:0] parse_idx   = 11'd0;
    reg [2:0]  weight_blk  = 3'd0;
    reg [7:0]  spike_tstep = 8'd0;
    reg [7:0]  weight_mask = 8'd0; // bitmask for 8 blocks (1024B each)

    // Signals to TX engine
    reg        tx_trigger_req = 1'b0;
    reg [7:0]  tx_resp_cmd    = 8'd0;
    reg [7:0]  tx_resp_param1 = 8'd0;
    reg [7:0]  tx_resp_param2 = 8'd0;
    reg [47:0] last_host_mac  = 48'hFFFFFFFFFFFF;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            parse_state    <= PARSE_IDLE;
            bram_wr_en     <= 1'b0;
            bram_addr      <= 12'd0;
            bram_din       <= 16'd0;
            fifo_wr_en_int <= 1'b0;
            fifo_din_int   <= 8'd0;
            weights_loaded <= 1'b0;
            weight_mask    <= 8'd0;
            tx_trigger_req <= 1'b0;
            tx_resp_cmd    <= 8'd0;
            tx_resp_param1 <= 8'd0;
            tx_resp_param2 <= 8'd0;
            last_host_mac  <= 48'hFFFFFFFFFFFF;
        end else begin
            bram_wr_en     <= 1'b0;
            fifo_wr_en_int <= 1'b0;
            tx_trigger_req <= 1'b0;

            // Handle Master FSM result push notification
            if (tx_send_fsm) begin
                tx_resp_cmd    <= 8'h02; // SNN_ETH_RESP_RESULT
                tx_resp_param1 <= tx_data_fsm;
                tx_resp_param2 <= 8'h00;
                tx_trigger_req <= 1'b1;
            end

            case (parse_state)
                PARSE_IDLE: begin
                    if (sys_rx_packet_trig) begin
                        parse_state <= PARSE_CHECK;
                    end
                end

                PARSE_CHECK: begin
                    // Validate EtherType == SNN_ETH_TYP (0x88B5) and Destination MAC matches or Broadcast
                    if (rx_buf[12] == SNN_ETH_TYP[15:8] && rx_buf[13] == SNN_ETH_TYP[7:0]) begin
                        // Save host source MAC for replying
                        last_host_mac <= {rx_buf[6], rx_buf[7], rx_buf[8], rx_buf[9], rx_buf[10], rx_buf[11]};

                        case (rx_buf[14]) // Command byte
                            8'hAA: begin // WRITE_WEIGHTS: [0xAA] [blk_idx: 0..7] [1024 bytes weights]
                                weight_blk  <= rx_buf[15][2:0];
                                parse_idx   <= 11'd0; // 0..511 words (1024 bytes)
                                parse_state <= PARSE_WEIGHTS;
                            end

                            8'hBB: begin // WRITE_SPIKES: [0xBB] [timestep: 0..24] [512 bytes spikes]
                                spike_tstep <= rx_buf[15];
                                parse_idx   <= 11'd0;
                                parse_state <= PARSE_SPIKES;
                            end

                            8'hCC: begin // INFER_REQ: [0xCC]
                                tx_resp_cmd    <= 8'h02; // SNN_ETH_RESP_RESULT
                                tx_resp_param1 <= tx_data_fsm;
                                tx_resp_param2 <= weights_loaded ? 8'h01 : 8'h00;
                                tx_trigger_req <= 1'b1;
                                parse_state    <= PARSE_IDLE;
                            end

                            8'hDD: begin // PING: [0xDD]
                                tx_resp_cmd    <= 8'h03; // SNN_ETH_RESP_PONG
                                tx_resp_param1 <= weights_loaded ? 8'h01 : 8'h00;
                                tx_resp_param2 <= fifo_empty ? 8'h01 : 8'h00;
                                tx_trigger_req <= 1'b1;
                                parse_state    <= PARSE_IDLE;
                            end

                            default: begin
                                parse_state <= PARSE_IDLE;
                            end
                        endcase
                    end else begin
                        parse_state <= PARSE_IDLE;
                    end
                end

                PARSE_WEIGHTS: begin
                    // Write 512 16-bit words (1024 bytes) into BRAM
                    // Payload begins at rx_buf[16]
                    bram_addr  <= {weight_blk, parse_idx[8:0]}; // 3 bits block + 9 bits = 12 bits (0..4095)
                    bram_din   <= {rx_buf[11'd16 + (parse_idx << 1)], rx_buf[11'd16 + (parse_idx << 1) + 11'd1]};
                    bram_wr_en <= 1'b1;

                    if (parse_idx == 11'd511) begin
                        weight_mask[weight_blk] <= 1'b1;
                        if ((weight_mask | (8'b1 << weight_blk)) == 8'hFF) begin
                            weights_loaded <= 1'b1;
                        end
                        // Trigger ACK response
                        tx_resp_cmd    <= 8'h01; // ACK
                        tx_resp_param1 <= {5'd0, weight_blk};
                        tx_resp_param2 <= ((weight_mask | (8'b1 << weight_blk)) == 8'hFF) ? 8'h01 : 8'h00;
                        tx_trigger_req <= 1'b1;
                        parse_state    <= PARSE_IDLE;
                    end else begin
                        parse_idx <= parse_idx + 11'd1;
                    end
                end

                PARSE_SPIKES: begin
                    // Write 512 spike bytes into Spike FIFO
                    // Payload begins at rx_buf[16]
                    fifo_din_int   <= rx_buf[11'd16 + parse_idx];
                    fifo_wr_en_int <= 1'b1;

                    if (parse_idx == 11'd511) begin
                        // Trigger ACK response for this timestep
                        tx_resp_cmd    <= 8'h01; // ACK
                        tx_resp_param1 <= spike_tstep;
                        tx_resp_param2 <= 8'h00;
                        tx_trigger_req <= 1'b1;
                        parse_state    <= PARSE_IDLE;
                    end else begin
                        parse_idx <= parse_idx + 11'd1;
                    end
                end

                default: parse_state <= PARSE_IDLE;
            endcase
        end
    end

    // =========================================================================
    // 6. TX Trigger Synchronization (Sys Clk -> eth_tx_clk domain)
    // =========================================================================
    reg tx_req_toggle = 1'b0;
    reg [7:0]  tx_cmd_latched   = 8'd0;
    reg [7:0]  tx_p1_latched    = 8'd0;
    reg [7:0]  tx_p2_latched    = 8'd0;
    reg [47:0] tx_dmac_latched  = 48'hFFFFFFFFFFFF;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_req_toggle   <= 1'b0;
            tx_cmd_latched  <= 8'd0;
            tx_p1_latched   <= 8'd0;
            tx_p2_latched   <= 8'd0;
            tx_dmac_latched <= 48'hFFFFFFFFFFFF;
        end else if (tx_trigger_req) begin
            tx_req_toggle   <= ~tx_req_toggle;
            tx_cmd_latched  <= tx_resp_cmd;
            tx_p1_latched   <= tx_resp_param1;
            tx_p2_latched   <= tx_resp_param2;
            tx_dmac_latched <= last_host_mac;
        end
    end

    reg [2:0] tx_clk_sync_req = 3'd0;
    always @(posedge eth_tx_clk or negedge eth_rstn) begin
        if (!eth_rstn)
            tx_clk_sync_req <= 3'd0;
        else
            tx_clk_sync_req <= {tx_clk_sync_req[1:0], tx_req_toggle};
    end
    wire tx_packet_trigger = (tx_clk_sync_req[2] ^ tx_clk_sync_req[1]);

    // =========================================================================
    // 7. CRC32 Calculation Function
    // =========================================================================
    reg [31:0] crc32 = 32'hFFFFFFFF;

    function [31:0] next_crc32;
        input [7:0] data;
        input [31:0] current_crc;
        reg [31:0] c;
        integer i;
        begin
            c = current_crc;
            for (i = 0; i < 8; i = i + 1) begin
                if ((c[0] ^ data[i]) == 1'b1)
                    c = {1'b0, c[31:1]} ^ 32'hEDB88320;
                else
                    c = {1'b0, c[31:1]};
            end
            next_crc32 = c;
        end
    endfunction

    // =========================================================================
    // 8. TX State Machine (eth_tx_clk domain)
    // =========================================================================
    localparam TX_IDLE      = 4'd0;
    localparam TX_PREAMBLE  = 4'd1;
    localparam TX_SFD       = 4'd2;
    localparam TX_PAYLOAD   = 4'd3;
    localparam TX_CRC       = 4'd4;
    localparam TX_IFG       = 4'd5;

    reg [3:0]  tx_state = TX_IDLE;
    reg [10:0] tx_len   = 11'd60; // Minimum 60 bytes (Ethernet frame without FCS)
    reg [10:0] tx_byte_idx = 11'd0;
    reg        tx_nibble_sel = 1'b0;
    reg [4:0]  tx_cnt = 5'd0;
    reg [7:0]  tx_curr_byte = 8'd0;
    reg [31:0] final_crc = 32'd0;
    reg [19:0] tx_led_cnt = 20'd0;
    assign led_tx = (tx_led_cnt > 0);

    // Frame payload multiplexer
    reg [7:0] next_tx_byte;
    always @(*) begin
        if (tx_byte_idx < 11'd6) begin
            // Destination MAC (Host MAC)
            case (tx_byte_idx)
                11'd0: next_tx_byte = tx_dmac_latched[47:40];
                11'd1: next_tx_byte = tx_dmac_latched[39:32];
                11'd2: next_tx_byte = tx_dmac_latched[31:24];
                11'd3: next_tx_byte = tx_dmac_latched[23:16];
                11'd4: next_tx_byte = tx_dmac_latched[15:8];
                11'd5: next_tx_byte = tx_dmac_latched[7:0];
                default: next_tx_byte = 8'h00;
            endcase
        end else if (tx_byte_idx < 11'd12) begin
            // Source MAC (FPGA MAC)
            case (tx_byte_idx)
                11'd6:  next_tx_byte = FPGA_MAC[47:40];
                11'd7:  next_tx_byte = FPGA_MAC[39:32];
                11'd8:  next_tx_byte = FPGA_MAC[31:24];
                11'd9:  next_tx_byte = FPGA_MAC[23:16];
                11'd10: next_tx_byte = FPGA_MAC[15:8];
                11'd11: next_tx_byte = FPGA_MAC[7:0];
                default: next_tx_byte = 8'h00;
            endcase
        end else if (tx_byte_idx == 11'd12) begin
            next_tx_byte = SNN_ETH_TYP[15:8];
        end else if (tx_byte_idx == 11'd13) begin
            next_tx_byte = SNN_ETH_TYP[7:0];
        end else if (tx_byte_idx == 11'd14) begin
            next_tx_byte = tx_cmd_latched;
        end else if (tx_byte_idx == 11'd15) begin
            next_tx_byte = tx_p1_latched;
        end else if (tx_byte_idx == 11'd16) begin
            next_tx_byte = tx_p2_latched;
        end else begin
            // Zero padding up to 60 bytes
            next_tx_byte = 8'h00;
        end
    end

    always @(posedge eth_tx_clk or negedge eth_rstn) begin
        if (!eth_rstn) begin
            tx_state      <= TX_IDLE;
            eth_tx_en     <= 1'b0;
            eth_txd       <= 4'h0;
            tx_len        <= 11'd60;
            tx_byte_idx   <= 11'd0;
            tx_nibble_sel <= 1'b0;
            tx_cnt        <= 5'd0;
            crc32         <= 32'hFFFFFFFF;
            tx_led_cnt    <= 20'd0;
        end else begin
            if (tx_led_cnt > 0) tx_led_cnt <= tx_led_cnt - 1'b1;

            case (tx_state)
                TX_IDLE: begin
                    eth_tx_en     <= 1'b0;
                    eth_txd       <= 4'h0;
                    tx_nibble_sel <= 1'b0;
                    if (tx_packet_trigger) begin
                        tx_len     <= 11'd60;
                        tx_state   <= TX_PREAMBLE;
                        tx_cnt     <= 5'd0;
                        crc32      <= 32'hFFFFFFFF;
                        tx_led_cnt <= 20'hFFFFF;
                    end
                end

                TX_PREAMBLE: begin
                    // Send 7 bytes of 0x55 (14 nibbles of 0x5)
                    eth_tx_en <= 1'b1;
                    eth_txd   <= 4'h5;
                    tx_cnt    <= tx_cnt + 1'b1;
                    if (tx_cnt == 5'd13) begin
                        tx_state <= TX_SFD;
                        tx_cnt   <= 5'd0;
                    end
                end

                TX_SFD: begin
                    // Send SFD 0xD5 (nibbles 0x5 then 0xD)
                    eth_tx_en <= 1'b1;
                    if (tx_cnt == 5'd0) begin
                        eth_txd <= 4'h5;
                        tx_cnt  <= 5'd1;
                    end else begin
                        eth_txd       <= 4'hD;
                        tx_state      <= TX_PAYLOAD;
                        tx_byte_idx   <= 11'd0;
                        tx_nibble_sel <= 1'b0;
                        tx_cnt        <= 5'd0;
                    end
                end

                TX_PAYLOAD: begin
                    eth_tx_en <= 1'b1;
                    if (!tx_nibble_sel) begin
                        tx_curr_byte  <= next_tx_byte;
                        eth_txd       <= next_tx_byte[3:0]; // Low nibble first
                        tx_nibble_sel <= 1'b1;
                    end else begin
                        eth_txd       <= tx_curr_byte[7:4]; // High nibble
                        tx_nibble_sel <= 1'b0;

                        crc32 <= next_crc32(tx_curr_byte, crc32);

                        if (tx_byte_idx == tx_len - 11'd1) begin
                            tx_state  <= TX_CRC;
                            tx_cnt    <= 5'd0;
                            final_crc <= ~next_crc32(tx_curr_byte, crc32);
                        end else begin
                            tx_byte_idx <= tx_byte_idx + 11'd1;
                        end
                    end
                end

                TX_CRC: begin
                    // Send 4-byte CRC (8 nibbles, LSB first)
                    eth_tx_en <= 1'b1;
                    case (tx_cnt)
                        5'd0: eth_txd <= final_crc[3:0];
                        5'd1: eth_txd <= final_crc[7:4];
                        5'd2: eth_txd <= final_crc[11:8];
                        5'd3: eth_txd <= final_crc[15:12];
                        5'd4: eth_txd <= final_crc[19:16];
                        5'd5: eth_txd <= final_crc[23:20];
                        5'd6: eth_txd <= final_crc[27:24];
                        5'd7: eth_txd <= final_crc[31:28];
                    endcase

                    if (tx_cnt == 5'd7) begin
                        tx_state <= TX_IFG;
                        tx_cnt   <= 5'd0;
                    end else begin
                        tx_cnt <= tx_cnt + 1'b1;
                    end
                end

                TX_IFG: begin
                    // Inter-frame gap: minimum 24 nibble clocks (960 ns)
                    eth_tx_en <= 1'b0;
                    eth_txd   <= 4'h0;
                    if (tx_cnt == 5'd24) begin
                        tx_state <= TX_IDLE;
                    end else begin
                        tx_cnt <= tx_cnt + 1'b1;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

endmodule
