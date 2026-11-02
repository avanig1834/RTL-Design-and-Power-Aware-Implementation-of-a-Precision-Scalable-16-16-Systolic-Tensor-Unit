`timescale 1ns / 1ps
// 2:1 mux (carry-select building block)
module acs_mux2 (input d0, d1, sel, output y);
    assign y = (d0 & ~sel) | (d1 & sel);
endmodule

// Ripple-carry adder
module acs_rca #(parameter W = 8) (
    input  [W-1:0] a, b,
    input          ci,
    output [W-1:0] s,
    output         co
);
    wire [W:0] c;
    assign c[0] = ci;
    genvar i;
    generate
        for (i = 0; i < W; i = i + 1) begin : g_fa
            pe_fa u_fa (.a(a[i]), .b(b[i]), .ci(c[i]), .s(s[i]), .co(c[i+1]));
        end
    endgenerate
    assign co = c[W];
endmodule

// Carry-select adder
module acs_csel #(parameter W = 8) (
    input  [W-1:0] a, b,
    input          ci,
    output [W-1:0] s,
    output         co
);
    wire [W-1:0] s_0, s_1;
    wire         co_0, co_1;
    acs_rca #(W) u_rca0 (.a(a), .b(b), .ci(1'b0), .s(s_0), .co(co_0));
    acs_rca #(W) u_rca1 (.a(a), .b(b), .ci(1'b1), .s(s_1), .co(co_1));
    genvar i;
    generate
        for (i = 0; i < W; i = i + 1) begin : g_mux
            acs_mux2 u_mux (.d0(s_0[i]), .d1(s_1[i]), .sel(ci), .y(s[i]));
        end
    endgenerate
    acs_mux2 u_mux_co (.d0(co_0), .d1(co_1), .sel(ci), .y(co));
endmodule


// Incrementer (rounding path)
module acs_inc #(parameter W = 8) (
    input  [W-1:0] a,
    input          ci,
    output [W-1:0] s,
    output         co
);
    wire [W:0] c;
    assign c[0] = ci;
    genvar i;
    generate
        for (i = 0; i < W; i = i + 1) begin : g_ha
            pe_ha u_ha (.a(a[i]), .b(c[i]), .s(s[i]), .c(c[i+1]));
        end
    endgenerate
    assign co = c[W];
endmodule

// Segmented accumulator (12/8/16-bit slices)
module pe_acc #(
    parameter TIE_EVEN = 0
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        m4,
    input  wire        m8,
    input  wire        m16,
    input  wire        e1,
    input  wire        clr,
    input  wire        valid,
    input  wire [31:0] prod,
    output wire [35:0] acc,
    output wire signed [35:0] c_out,
    output wire [31:0] c_q,
    output wire        ovf
);
    wire e2 = m16;

    wire sx0 = m4  & prod[7];
    wire sx1 = m8  & prod[15];
    wire sx2 = e2  & prod[31];

    wire g1 = valid & e1;
    wire g2 = valid & e2;
    wire [35:0] ext = { {4{sx2 & valid}},
                        prod[31:20] & {12{g2}},
                        prod[19:12] & {8{g1}},
                        prod[11:0]  & {12{valid}} };

    reg  [11:0] r0;
    reg  [7:0]  r1;
    reg  [15:0] r2;

    wire [11:0] x0 = r0 & {12{~clr}};
    wire [7:0]  x1 = r1 & {8{~clr}};
    wire [15:0] x2 = r2 & {16{~clr}};

    wire [11:0] s0; wire co0;
    wire [7:0]  s1; wire co1;
    wire [15:0] s2; wire co2_unused;

    acs_rca #(12) u_s0 (.a(x0), .b(ext[11:0]), .ci(1'b0), .s(s0), .co(co0));
    wire c1 = co0 & e1;
    wire c2 = co1 & e2;
    acs_csel #(8)  u_s1 (.a(x1), .b(ext[19:12]), .ci(c1), .s(s1), .co(co1));
    acs_csel #(16) u_s2 (.a(x2), .b(ext[35:20]), .ci(c2), .s(s2), .co(co2_unused));

    wire ld0 = valid;
    wire ld1 = valid & e1;
    wire ld2 = valid & e2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r0 <= 12'd0;  r1 <= 8'd0;  r2 <= 16'd0;
        end else begin
            if (ld0) r0 <= s0;
            if (ld1) r1 <= s1;
            if (ld2) r2 <= s2;
        end
    end

    assign acc = {r2, r1, r0};

    // output isolation: c_out / c_q valid only while valid = 0
    wire oe = ~valid;
    wire [11:0] o0 = r0 & {12{oe}};
    wire [7:0]  o1 = r1 & {8{oe & e1}};
    wire [15:0] o2 = r2 & {16{oe & e2}};

    wire s4 = m4 & o0[11];
    wire s8 = (m8 & o1[7]) | s4;
    assign c_out = { o2 | {16{s8}}, o1 | {8{s4}}, o0 };

    wire sticky = o0[2] | o0[1] | o0[0] | (TIE_EVEN ? o0[4] : 1'b0);
    wire rnd    = o0[3] & sticky;

    wire [7:0]  q0; wire qco0;
    wire [7:0]  q1; wire qco1;
    wire [15:0] q2; wire qco2_unused;
    acs_inc #(8)  u_q0 (.a(o0[11:4]), .ci(rnd),  .s(q0),  .co(qco0));
    wire qc1 = qco0 & e1;
    acs_inc #(8)  u_q1 (.a(o1),       .ci(qc1), .s(q1),  .co(qco1));
    wire qc2 = qco1 & e2;
    acs_inc #(16) u_q2 (.a(o2),       .ci(qc2), .s(q2),  .co(qco2_unused));
    assign c_q = {q2, q1, q0};

`ifdef SYNTHESIS
    assign ovf = 1'b0;
`else
    reg ovf_r;
    wire xs = (m4 & x0[11]) | (m8 & x1[7]) | (e2 & x2[15]);
    wire ps = sx0 | sx1 | sx2;
    wire ss = (m4 & s0[11]) | (m8 & s1[7]) | (e2 & s2[15]);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)           ovf_r <= 1'b0;
        else if (valid & clr) ovf_r <= 1'b0;
        else if (valid)       ovf_r <= ovf_r | (~(xs ^ ps) & (ss ^ xs));
    end
    assign ovf = ovf_r;
`endif
endmodule
