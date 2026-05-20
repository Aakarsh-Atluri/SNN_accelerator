module cascaded_adder #(
    parameter N   = 4096,
    parameter W   = 16,
    parameter OUT = 32
)(
    input  wire clk,
    input  wire rst,

    input  wire start,

    output reg busy,
    output reg valid,

    input  wire spike_in,
    input  wire spike_valid,

    output reg [$clog2(N)-1:0] weight_addr,
    input  wire signed [W-1:0] weight_data,

    input  wire signed [W-1:0] bias,

    output reg signed [OUT-1:0] result
);

    // Internal Registers

    reg signed [OUT-1:0] acc;

    reg [$clog2(N):0] issue_count;
    reg [$clog2(N):0] consume_count;

    reg spike_pipe;
    reg spike_pipe_valid;

    reg signed [OUT-1:0] weight_ext;
    reg signed [OUT-1:0] bias_ext;

    reg signed [OUT-1:0] next_acc;


    // Sign Extension : trained weights can be positive or negative, we need to preserve its sign in 32bit out.

    always @(*) begin

        weight_ext = {{(OUT-W){weight_data[W-1]}}, weight_data};
        bias_ext   = {{(OUT-W){bias[W-1]}}, bias};

        next_acc = acc;

        if (spike_pipe_valid && spike_pipe)
            next_acc = acc + weight_ext;
    end

    // ============================================================
    // Main Logic
    // ============================================================

    always @(posedge clk or posedge rst) begin

        if (rst) begin

            acc              <= 0;
            result           <= 0;

            issue_count      <= 0;
            consume_count    <= 0;

            weight_addr      <= 0;

            spike_pipe       <= 0;
            spike_pipe_valid <= 0;

            busy             <= 0;
            valid            <= 0;

        end else begin

            valid <= 0;

            // ====================================================
            // START
            // ====================================================

            if (start && !busy) begin                     //fixed the intial timing error.

                acc              <= 0;

                issue_count      <= 0;
                consume_count    <= 0;

                weight_addr      <= 0;

                spike_pipe       <= 0;
                spike_pipe_valid <= 0;

                busy             <= 1;
            end

            // RUNNING
            
            else if (busy) begin

                // ------------------------------------------------
                // STAGE 2
                // Consume pipeline
                // ------------------------------------------------

                if (spike_pipe_valid) begin

                    acc <= next_acc;

                    consume_count <= consume_count + 1;

                    // FINAL OUTPUT
                    if (consume_count == N-1) begin

                        result <= next_acc + bias_ext;

                        valid <= 1;
                        busy  <= 0;
                    end
                end

                // ------------------------------------------------
                // STAGE 1
                // Load next pipeline values
                // ------------------------------------------------

                if (spike_valid && issue_count < N) begin           

                    weight_addr <= issue_count[$clog2(N)-1:0];

                    spike_pipe       <= spike_in;
                    spike_pipe_valid <= 1;

                    issue_count <= issue_count + 1;
                end

                else begin
                    spike_pipe_valid <= 0;
                end
            end
        end
    end

endmodule
