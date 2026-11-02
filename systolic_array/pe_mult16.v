`timescale 1ns / 1ps
// 4x4 Baugh-Wooley block with signed/unsigned select (multiplier building block)
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
    pe_ha u_h11 (.a(pp10), .b(pp01),            .s(q[1]), .c(c1_1));
    pe_fa u_f12 (.a(pp20), .b(pp11), .ci(c1_1), .s(s1_2), .co(c1_2));
    pe_fa u_f13 (.a(pp30), .b(pp21), .ci(c1_2), .s(s1_3), .co(c1_3));
    pe_fa u_f14 (.a(k4),   .b(pp31), .ci(c1_3), .s(s1_4), .co(c1_4));

    wire c2_2, s2_3, c2_3, s2_4, c2_4, s2_5, c2_5;
    pe_ha u_h22 (.a(s1_2), .b(pp02),            .s(q[2]), .c(c2_2));
    pe_fa u_f23 (.a(s1_3), .b(pp12), .ci(c2_2), .s(s2_3), .co(c2_3));
    pe_fa u_f24 (.a(s1_4), .b(pp22), .ci(c2_3), .s(s2_4), .co(c2_4));
    pe_fa u_f25 (.a(c1_4), .b(pp32), .ci(c2_4), .s(s2_5), .co(c2_5));

    wire c3_3, c3_4, c3_5, c3_6;
    pe_fa u_f33 (.a(s2_3), .b(pp03), .ci(k3),   .s(q[3]), .co(c3_3));
    pe_fa u_f34 (.a(s2_4), .b(pp13), .ci(c3_3), .s(q[4]), .co(c3_4));
    pe_fa u_f35 (.a(s2_5), .b(pp23), .ci(c3_4), .s(q[5]), .co(c3_5));
    pe_fa u_f36 (.a(c2_5), .b(pp33), .ci(c3_5), .s(q[6]), .co(c3_6));

    assign q[0] = pp00;
    assign q[7] = c3_6;     // the k7 XOR of v1 is folded into the parent's constant
endmodule

`timescale 1ns / 1ps
// 16x16 precision-scalable multiplier, combinational
module pe_mult16 #(
    parameter MASK_INPUTS = 1
)(
    input  wire [15:0]        a,
    input  wire [15:0]        b,
    input  wire               m8,
    input  wire               m16,
    input  wire [3:0]         en,
    input  wire [3:0]         sgn,
    output wire signed [31:0] prod
);
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

    wire [31:0] corr = m16 ? 32'h6EF00000 :
                       m8  ? 32'hFFFF7000 : 32'hFFFFFF80;

    reg [31:0] acc;
    integer k;
    always @* begin
        acc = corr;
        for (k = 0; k < 16; k = k + 1)
            acc = acc + terms[32*k +: 32];
    end
    assign prod = acc;
endmodule
