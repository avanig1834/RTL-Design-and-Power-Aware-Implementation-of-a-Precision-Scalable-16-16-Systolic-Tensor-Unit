`timescale 1ns / 1ps

// =====================================================================
// Tensor Unit 16x16
// Top level of precision-scalable systolic tensor unit
//
// tu_fsm          : Sequencing, mode register, load/acc/clear
// pe_mode_decode  : Mode decoding and broadcast
// skew registers  : Row i of A delayed by i cycles
//                   Column j of B delayed by j cycles
// PE array        : Output-stationary systolic array
// C output        : Captured when cap is asserted
//
// Schedule        : A[i][k] and B[k][j] meet at PE(i,j)
//                   at step t = i+j+k
//
// Latency         : start -> done = 3N+1 cycles (+ bubbles)
// =====================================================================

module tensor_unit_16x16 #(
    parameter N  = 16,
    parameter DW = 16,
    parameter AW = 36
)(
    input  wire              clk,
    input  wire              reset,
    input  wire              start,

    input  wire              mode_1,
    input  wire              mode_0,

    input  wire              input_valid,

    input  wire [N*DW-1:0]   a_col,
    input  wire [N*DW-1:0]   b_row,

    output wire              busy,
    output wire              done,

    output reg  [N*N*AW-1:0] c_out
);

    localparam D = 2*N - 1;

    // ================================================================
    // 1. FSM + MODE DECODING
    // ================================================================

    wire [1:0] mode_q;

    wire m4, m8, m16, e1;
    wire rst_n, step, feed, cap;

    wire [3:0] en, sgn;

    wire [D-1:0] load_d;
    wire [D-1:0] acc_valid_d;
    wire [D-1:0] clr_d;

    tu_fsm #(
        .N(N)
    ) u_fsm (
        .clk(clk),
        .reset(reset),
        .start(start),

        .mode_1(mode_1),
        .mode_0(mode_0),
        .input_valid(input_valid),

        .busy(busy),
        .done(done),

        .mode_q(mode_q),

        .m4(),
        .m8(),
        .m16(),
        .e1(),

        .en(),
        .sgn(),

        .rst_n(rst_n),
        .step(step),
        .feed(feed),

        .load_d(load_d),
        .acc_valid_d(acc_valid_d),
        .clr_d(clr_d),

        .cap(cap)
    );

    pe_mode_decode u_dec (
        .mode_q(mode_q),

        .m4(m4),
        .m8(m8),
        .m16(m16),
        .e1(e1),

        .en(en),
        .sgn(sgn)
    );


    // ================================================================
    // 2. INPUT SKEW REGISTERS
    // ================================================================

    wire [N*DW-1:0] a_edge;
    wire [N*DW-1:0] b_edge;

    wire st0 = step;
    wire st1 = step & e1;
    wire st2 = step & m16;

    genvar i, j, s;

    generate

        for (i = 0; i < N; i = i + 1) begin : g_skew

            wire [DW-1:0] a_new;
            wire [DW-1:0] b_new;

            assign a_new =
                feed ? a_col[i*DW +: DW] : {DW{1'b0}};

            assign b_new =
                feed ? b_row[i*DW +: DW] : {DW{1'b0}};


            // --------------------------------------------------------
            // Row/column zero: No skew registers required
            // --------------------------------------------------------

            if (i == 0) begin : g_direct

                assign a_edge[i*DW +: DW] = a_new;
                assign b_edge[i*DW +: DW] = b_new;

            end


            // --------------------------------------------------------
            // Rows/columns 1 to N-1: Insert i stages of skew
            // --------------------------------------------------------

            else begin : g_chain

                reg [DW-1:0] a_sk [0:i-1];
                reg [DW-1:0] b_sk [0:i-1];

                for (s = 0; s < i; s = s + 1) begin : g_st

                    wire [DW-1:0] a_d;
                    wire [DW-1:0] b_d;

                    if (s == 0) begin : g_first

                        assign a_d = a_new;
                        assign b_d = b_new;

                    end

                    else begin : g_next

                        assign a_d = a_sk[s-1];
                        assign b_d = b_sk[s-1];

                    end


                    // ------------------------------------------------
                    // Mode-dependent skew register updates
                    // ------------------------------------------------

                    always @(posedge clk or negedge rst_n) begin

                        if (!rst_n) begin

                            a_sk[s] <= {DW{1'b0}};
                            b_sk[s] <= {DW{1'b0}};

                        end

                        else begin

                            if (st0) begin

                                a_sk[s][3:0] <= a_d[3:0];
                                b_sk[s][3:0] <= b_d[3:0];

                            end

                            if (st1) begin

                                a_sk[s][7:4] <= a_d[7:4];
                                b_sk[s][7:4] <= b_d[7:4];

                            end

                            if (st2) begin

                                a_sk[s][15:8] <= a_d[15:8];
                                b_sk[s][15:8] <= b_d[15:8];

                            end

                        end

                    end

                end


                // ----------------------------------------------------
                // IMPORTANT FIX:
                // Connect the final skew register to PE boundary
                // inside the i > 0 generate branch.
                // ----------------------------------------------------

                assign a_edge[i*DW +: DW] = a_sk[i-1];
                assign b_edge[i*DW +: DW] = b_sk[i-1];

            end

        end

    endgenerate


    // ================================================================
    // 3. 16x16 PROCESSING ELEMENT ARRAY
    // ================================================================

    wire [N*N*DW-1:0] pe_a_flat;
    wire [N*N*DW-1:0] pe_b_flat;

    wire [N*N*AW-1:0] pe_acc_flat;
    wire [N*N*AW-1:0] pe_c_flat;


    generate

        for (i = 0; i < N; i = i + 1) begin : g_row

            for (j = 0; j < N; j = j + 1) begin : g_col

                localparam P = i*N + j;

                wire [DW-1:0] a_src;
                wire [DW-1:0] b_src;


                // ----------------------------------------------------
                // A travels from LEFT to RIGHT
                // ----------------------------------------------------

                if (j == 0) begin : g_a_edge

                    assign a_src = a_edge[i*DW +: DW];

                end

                else begin : g_a_pass

                    assign a_src =
                        pe_a_flat[(P-1)*DW +: DW];

                end


                // ----------------------------------------------------
                // B travels from TOP to BOTTOM
                // ----------------------------------------------------

                if (i == 0) begin : g_b_edge

                    assign b_src = b_edge[j*DW +: DW];

                end

                else begin : g_b_pass

                    assign b_src =
                        pe_b_flat[(P-N)*DW +: DW];

                end


                // ----------------------------------------------------
                // Processing Element
                // ----------------------------------------------------

                pe u_pe (

                    .clk(clk),
                    .rst_n(rst_n),

                    .m4(m4),
                    .m8(m8),
                    .m16(m16),

                    .e1(e1),
                    .en(en),
                    .sgn(sgn),

                    .load(load_d[i+j]),
                    .acc_valid(acc_valid_d[i+j]),
                    .clr(clr_d[i+j]),

                    .a_in(a_src),
                    .b_in(b_src),

                    .a_out(pe_a_flat[P*DW +: DW]),
                    .b_out(pe_b_flat[P*DW +: DW]),

                    .acc(pe_acc_flat[P*AW +: AW]),
                    .c_out(pe_c_flat[P*AW +: AW]),

                    .c_q(),
                    .ovf()

                );

            end

        end

    endgenerate


    // ================================================================
    // 4. FINAL RESULT REGISTER
    // ================================================================

    always @(posedge clk) begin

        if (reset)

            c_out <= {(N*N*AW){1'b0}};

        else if (cap)

            c_out <= pe_c_flat;

    end


    // ================================================================
    // 5. DEBUG / MODE PROBE
    // ================================================================

    wire [1:0] eff_mode = mode_q;

endmodule
