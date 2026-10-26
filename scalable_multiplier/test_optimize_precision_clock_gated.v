`timescale 1ns / 1ps
// ===========================================================================
// Precision-scalable signed Baugh-Wooley multiplier, v2 (purely combinational)
//   mode: 2'b00 = INT4, 2'b01 = INT8, 2'b10 = INT16, 2'b11 = RESERVED
//
// REPLACES test_optimize.v -- do not compile both (duplicate module names).
//
// What changed versus v1 (all of it removes logic, none adds any):
//   1. One layer of operand isolation instead of three. v1 masked the nibbles
//      at the top, masked them again inside every block, and gated the block
//      output again. With zero operands the block output is already zero.
//   2. No per-block 32-bit sign extension (16 x 24-bit replicated sign + mux).
//      A block value is   V = q - 128*sx   (sx = block involves a signed
//      nibble), so all 16 terms are plain 8-bit unsigned numbers and the
//      sign handling collapses into ONE mode-dependent constant "corr".
//   3. The final XOR on bit 7 of every block disappears: q[7] = carry out.
//   4. Control decode is direct (no >= / == comparators on a 'top' value).
//   5. Block signedness is forced to 0 for inactive blocks, so an inactive
//      block is exactly 0 and its contribution is a constant.
//
// MASK_INPUTS = 1 : upper nibbles are ANDed with their enable here
//                   (use when the input registers may hold stale data).
// MASK_INPUTS = 0 : use ONLY if upstream logic guarantees inactive nibbles
//                   are zero on every relevant cycle. The supplied wrapper
//                   keeps MASK_INPUTS=1 because gated registers retain stale data.
// ===========================================================================

// ---------------- Basic cells ----------------
module bw_ha (input a, b, output s, c);
    assign s = a ^ b;
    assign c = a & b;
endmodule

module bw_fa (input a, b, ci, output s, co);
    assign s  = a ^ b ^ ci;
    assign co = (a & b) | (b & ci) | (a & ci);
endmodule

// ---------------- Sign-selectable 4x4 block (no enable, no output gate) ----------------
// sa/sb = 1 : that operand is signed, 0 : unsigned.
// Output q is an OFFSET-BINARY result:   true value V = q - 128*(sa|sb)
//   sa|sb = 0 : V = q            (unsigned x unsigned, 0..225)
//   sa|sb = 1 : V = q - 128      (8-bit two's complement, bit 7 of V = ~q[7])
// With a == 0 or b == 0 and sa = sb = 0, every node is 0 and q = 0.
module bw_4x4_sel (
    input  wire [3:0] a,
    input  wire [3:0] b,
    input  wire       sa,
    input  wire       sb,
    output wire [7:0] q
);
    wire k3 = sa ^ sb;      // correction at weight 2^3
    wire k4 = sa & sb;      // correction at weight 2^4

    wire pp00 = a[0] & b[0];
    wire pp10 = a[1] & b[0];
    wire pp20 = a[2] & b[0];
    wire pp30 = (a[3] & b[0]) ^ sa;

    wire pp01 = a[0] & b[1];
    wire pp11 = a[1] & b[1];
    wire pp21 = a[2] & b[1];
    wire pp31 = (a[3] & b[1]) ^ sa;

    wire pp02 = a[0] & b[2];
    wire pp12 = a[1] & b[2];
    wire pp22 = a[2] & b[2];
    wire pp32 = (a[3] & b[2]) ^ sa;

    wire pp03 = (a[0] & b[3]) ^ sb;
    wire pp13 = (a[1] & b[3]) ^ sb;
    wire pp23 = (a[2] & b[3]) ^ sb;
    wire pp33 = (a[3] & b[3]) ^ k3;

    wire c1_1, s1_2, c1_2, s1_3, c1_3, s1_4, c1_4;
    bw_ha u_h11 (.a(pp10), .b(pp01),            .s(q[1]), .c(c1_1));
    bw_fa u_f12 (.a(pp20), .b(pp11), .ci(c1_1), .s(s1_2), .co(c1_2));
    bw_fa u_f13 (.a(pp30), .b(pp21), .ci(c1_2), .s(s1_3), .co(c1_3));
    bw_fa u_f14 (.a(k4),   .b(pp31), .ci(c1_3), .s(s1_4), .co(c1_4));

    wire c2_2, s2_3, c2_3, s2_4, c2_4, s2_5, c2_5;
    bw_ha u_h22 (.a(s1_2), .b(pp02),            .s(q[2]), .c(c2_2));
    bw_fa u_f23 (.a(s1_3), .b(pp12), .ci(c2_2), .s(s2_3), .co(c2_3));
    bw_fa u_f24 (.a(s1_4), .b(pp22), .ci(c2_3), .s(s2_4), .co(c2_4));
    bw_fa u_f25 (.a(c1_4), .b(pp32), .ci(c2_4), .s(s2_5), .co(c2_5));

    wire c3_3, c3_4, c3_5, c3_6;
    bw_fa u_f33 (.a(s2_3), .b(pp03), .ci(k3),   .s(q[3]), .co(c3_3));
    bw_fa u_f34 (.a(s2_4), .b(pp13), .ci(c3_3), .s(q[4]), .co(c3_4));
    bw_fa u_f35 (.a(s2_5), .b(pp23), .ci(c3_4), .s(q[5]), .co(c3_5));
    bw_fa u_f36 (.a(c2_5), .b(pp33), .ci(c3_5), .s(q[6]), .co(c3_6));

    assign q[0] = pp00;
    assign q[7] = c3_6;     // the k7 XOR of v1 is folded into the parent's constant
endmodule

// ---------------- Top ----------------
module bw_mult16_seg #(
    parameter MASK_INPUTS = 1
)(
    input  wire [15:0]       a,
    input  wire [15:0]       b,
    input  wire [1:0]        mode,
    output wire signed [31:0] p
);
    // Explicit legal-mode decode. 2'b11 is RESERVED and disables all blocks.
    // Nibble 2 is never the signed nibble in this decomposition.
    wire mode4  = (mode == 2'b00);
    wire mode8  = (mode == 2'b01);
    wire mode16 = (mode == 2'b10);
    wire mode_valid = mode4 | mode8 | mode16;

    wire [3:0] en  = {mode16, mode16, (mode8 | mode16), mode_valid};
    wire [3:0] sgn = {mode16, 1'b0, mode8, mode4};

    // Single layer of operand masking (optional).
    wire [3:0] an [0:3];
    wire [3:0] bn [0:3];
    genvar i, j;
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_op
            if (MASK_INPUTS != 0 && i != 0) begin : g_mask
                assign an[i] = a[4*i +: 4] & {4{en[i]}};
                assign bn[i] = b[4*i +: 4] & {4{en[i]}};
            end else begin : g_pass
                assign an[i] = a[4*i +: 4];
                assign bn[i] = b[4*i +: 4];
            end
        end
    endgenerate

    // 16 blocks; an inactive block gets sa = sb = 0 so it is exactly zero.
    wire [511:0] terms;
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_row
            for (j = 0; j < 4; j = j + 1) begin : g_col
                wire [7:0] q;
                bw_4x4_sel u_blk (
                    .a (an[i]),
                    .b (bn[j]),
                    .sa(sgn[i] & en[j]),
                    .sb(sgn[j] & en[i]),
                    .q (q)
                );
                assign terms[32*(4*i+j) +: 32] = {24'd0, q} << (4*(i+j));
            end
        end
    endgenerate

    // corr = -128 * sum over signed-involving blocks of 16^(i+j)
    //   INT4 : only block(0,0)                         -> -0x80
    //   INT8 : blocks (0,1),(1,0),(1,1)                 -> -0x9000
    //   INT16: blocks (3,j),(i,3)                       -> -0x91100000
    wire [31:0] corr = mode[1] ? 32'h6EF00000 :
                       mode[0] ? 32'hFFFF7000 : 32'hFFFFFF80;

    reg [31:0] acc;
    integer k;
    always @* begin
        acc = corr;
        for (k = 0; k < 16; k = k + 1)
            acc = acc + terms[32*k +: 32];
    end
    // Reserved mode must not initiate arithmetic or expose an old result.
    assign p = mode_valid ? acc : 32'sd0;
endmodule


// ===========================================================================
// Precision-grouped operand registers with non-latch clock-enable generation
//
// Group 0: A/B nibble 0 [3:0]       -> ungated clock (INT4/8/16)
// Group 1: A/B nibble 1 [7:4]       -> gated clock (INT8/16)
// Group 2: A/B nibbles 2 and 3      -> shared gated clock (INT16 only)
//
// IMPORTANT:
// - This is an RTL implementation using negedge enable flops and AND gates,
//   not a substitute for a characterized library ICG cell in an ASIC flow.
// - Keep MASK_INPUTS=1: gated upper registers retain stale values when frozen.
// - Interface contract: assert start while !busy with input_valid=0. The first
//   input_valid beat must be in a later cycle, after mode_q has been captured.
// - The controller must reject start when mode_in==2'b11 and must not assert
//   input_valid for that rejected transaction. mode_q is held at its last legal
//   value for a reserved-mode start, as requested.
// - mode must remain fixed from accepted start through completion (done).
// ===========================================================================
module precision_operand_regs (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire        busy,
    input  wire        done,
    input  wire        input_valid,
    input  wire [1:0]  mode_in,
    input  wire [15:0] a_in,
    input  wire [15:0] b_in,
    output reg  [1:0]  mode_q,
    output reg  [15:0] a_q,
    output reg  [15:0] b_q
);
    wire legal_mode_in = (mode_in != 2'b11);

    // Capture mode only on an accepted, legal start. Hold throughout busy.
    // Reset to INT16 for deterministic startup; reset alone is NOT relied upon
    // to zero inactive nibbles after later mode changes.
    always @(posedge clk or posedge rst) begin
        if (rst)
            mode_q <= 2'b10;
        else if (start && !busy && legal_mode_in)
            mode_q <= mode_in;
    end

    wire mode8_or16 = (mode_q == 2'b01) || (mode_q == 2'b10);
    wire mode16_only = (mode_q == 2'b10);

    reg en_nibble1;
    reg en_upper;

    // Enable flops change only while clk is low, preventing mid-high-phase
    // changes to these AND-generated clocks. input_valid gaps also stop clocks.
    always @(negedge clk or posedge rst) begin
        if (rst) begin
            en_nibble1 <= 1'b0;
            en_upper   <= 1'b0;
        end else begin
            en_nibble1 <= input_valid && !((mode_q == 2'b11)) && mode8_or16;
            en_upper   <= input_valid && !((mode_q == 2'b11)) && mode16_only;
        end
    end

    wire clk_nibble1 = clk & en_nibble1;
    wire clk_upper   = clk & en_upper;

    // Nibble 0: ungated clock, but register updates only on valid input.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            a_q[3:0] <= 4'b0;
            b_q[3:0] <= 4'b0;
        end else if (input_valid && mode_q != 2'b11) begin
            a_q[3:0] <= a_in[3:0];
            b_q[3:0] <= b_in[3:0];
        end
    end

    // Nibble 1: A/B share one precision-group clock.
    always @(posedge clk_nibble1 or posedge rst) begin
        if (rst) begin
            a_q[7:4] <= 4'b0;
            b_q[7:4] <= 4'b0;
        end else begin
            a_q[7:4] <= a_in[7:4];
            b_q[7:4] <= b_in[7:4];
        end
    end

    // Nibbles 2 and 3: A/B share a second precision-group clock.
    always @(posedge clk_upper or posedge rst) begin
        if (rst) begin
            a_q[15:8] <= 8'b0;
            b_q[15:8] <= 8'b0;
        end else begin
            a_q[15:8] <= a_in[15:8];
            b_q[15:8] <= b_in[15:8];
        end
    end
endmodule

// Integrated wrapper: grouped operand clock gating + retained operand isolation.
module bw_mult16_precision_gated (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire        busy,
    input  wire        done,
    input  wire        input_valid,
    input  wire [1:0]  mode_in,
    input  wire [15:0] a_in,
    input  wire [15:0] b_in,
    output wire [1:0]  mode_q,
    output wire [15:0] a_q,
    output wire [15:0] b_q,
    output wire signed [31:0] product
);
    precision_operand_regs u_operand_regs (
        .clk(clk), .rst(rst), .start(start), .busy(busy), .done(done),
        .input_valid(input_valid), .mode_in(mode_in),
        .a_in(a_in), .b_in(b_in), .mode_q(mode_q), .a_q(a_q), .b_q(b_q)
    );

    // Keep masks enabled because upper register slices retain stale data when
    // their clocks are gated. The multiplier itself disables blocks per mode.
    bw_mult16_seg #(.MASK_INPUTS(1)) u_multiplier (
        .a(a_q), .b(b_q), .mode(mode_q), .p(product)
    );
endmodule
