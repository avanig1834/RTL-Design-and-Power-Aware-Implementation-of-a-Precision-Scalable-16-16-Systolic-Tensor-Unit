`timescale 1ns / 1ps
// ----------------------------------------------------------------------------------
// tu_acc_str_csel : STRUCTURAL carry-select version of tu_acc_opt (same ports, same behaviour)
// ----------------------------------------------------------------------------------
// Adders are built explicitly from full/half-adder cells - no '+' operator:
//   * S0 (12b, all modes) : ripple-carry adder
//   * S1 (8b, INT8/16)    : carry-select (two RCAs + mux)
//   * S2 (16b, INT16)     : carry-select (two RCAs + mux)
// Faster than tu_acc_str_v, but both RCAs toggle -> ~2x S1/S2 adder power
// Carries S0->S1, S1->S2 killed with AND gates (precision gating)
// round-off            : half-adder incrementer chain, also carry-killed
//
// Power gating:
//   * slice registers load only when enabled (ld_k) -> Genus ICG per slice
//   * operand isolation: product into the adders is ANDed with valid (and
//     e1/e2 for S1/S2), so adders do not toggle while idle
//   * output isolation: c_out / c_q logic is driven from r & ~valid, so the
//     sign-extension and incrementers do not toggle during accumulation.
//     => c_out / c_q are VALID ONLY WHILE valid = 0. acc is always live.
//
// Only the slice registers use an always block (flip-flops).
// prod comes from the V2 multiplier (multipler.sv); full product sign-extended
// to 32 bits. S0 uses prod[11:0] as is; S1/S2 inputs are AND-gated by mode.
// mode: 00 = INT4, 01 = INT8, 10 = INT16, 11 = reserved
// ----------------------------------------------------------------------------------

// ------------------------ leaf cells ------------------------
module acs_ha (input a, b, output s, c);
    assign s = a ^ b;
    assign c = a & b;
endmodule

module acs_fa (input a, b, ci, output s, co);
    wire t = a ^ b;
    assign s = t ^ ci;
    assign co = (a & b) | (t & ci);
endmodule

module acs_mux2 (input in0, d1, sel, output y);
    assign y = (d0 & ~sel) | (d1 & sel);
endmodule

// ------------------- W-bit ripple-carry adder -------------------
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
            acs_fa u_fa (.a(a[i]), .b(b[i]), .ci(c[i]), .s(s[i]), .co(c[i+1]));
        end
    endgenerate
    assign co = c[W];
endmodule

// ------------------- W-bit carry-select adder block -------------------
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

// ------------------- W-bit incrementer (half-adder chain) -------------------
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
            acs_ha u_ha (.a(a[i]), .b(c[i]), .s(s[i]), .c(c[i+1]));
        end
    endgenerate
    assign co = c[W];
endmodule

// ------------------------ top ------------------------
module tu_acc_str_csel #(
    parameter TIE_EVEN = 0
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  mode,
    input  wire        clr,
    input  wire        valid,
    input  wire [31:0] prod, // V2 product, sign-extended to 32 bits
    output wire [35:0] acc,
    output wire signed [35:0] c_out,
    output wire [31:0] c_q,
    output wire        ovf
);

    // mode decode
    wire m4  = ~(mode[1] | mode[0]);
    wire m8  = ~mode[1] & mode[0];
    wire e0  = ~(mode[1] & mode[0]);
    wire e1  = mode[1] ^ mode[0];
    wire e2  = mode[1] & ~mode[0];

    // product to 2N+4 bits (V2 product is already sign extended)
    wire sx0 = m4 & prod[7];          // ovf monitor only
    wire sx1 = m8 & prod[15];         // ovf monitor only
    wire sx2 = e2 & prod[31];

    // operand isolation: valid folded into the slice masks
    wire g1 = valid & e1;
    wire g2 = valid & e2;
    wire [35:0] ext = { {4{sx2 & valid}},
                        prod[31:20] & {12{g2}},  // S2: INT16 only
                        prod[19:12] & {8{g1}},   // S1: INT8 / INT16
                        prod[11:0]  & {12{valid}} }; // S0: all modes

    // slice registers
    reg  [11:0] r0;
    reg  [7:0]  r1;
    reg  [15:0] r2;

    wire [11:0] x0 = r0 & {12{~clr}};
    wire [7:0]  x1 = r1 & {8{~clr}};
    wire [15:0] x2 = r2 & {16{~clr}};

    // segmented adders
    wire [11:0] s0; wire co0;
    wire [7:0]  s1; wire co1;
    wire [15:0] s2; wire co2_unused;

    acs_rca #(12) u_s0 (.a(x0), .b(ext[11:0]), .ci(1'b0), .s(s0), .co(co0));
    wire c1 = co0 & e1;               // carry kill INT4
    wire c2 = co1 & e2;               // carry kill INT4/INT8
    acs_csel #(8)  u_s1 (.a(x1), .b(ext[19:12]), .ci(c1), .s(s1), .co(co1));
    acs_csel #(16) u_s2 (.a(x2), .b(ext[35:20]), .ci(c2), .s(s2), .co(co2_unused));

    // load enables (precision gating of the registers -> ICG in Genus)
    wire v   = valid & e0;
    wire ld0 = v;
    wire ld1 = v & (e1 | clr);
    wire ld2 = v & (e2 | clr);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r0 <= 12'd0;
            r1 <= 8'd0;
            r2 <= 16'd0;
        end else begin
            if (ld0) r0 <= s0;
            if (ld1) r1 <= s1;
            if (ld2) r2 <= s2;
        end
    end

    assign acc = {r2, r1, r0};

    // exact result, sign-extended
    // output isolation: c_out / c_q valid only while valid = 0
    wire oe = ~valid;
    wire [11:0] o0 = r0 & {12{oe}};
    wire [7:0]  o1 = r1 & {8{oe}};
    wire [15:0] o2 = r2 & {16{oe}};

    wire s4 = m4 & o0[11];
    wire s8 = (m8 & o1[7]) | s4;
    assign c_out = { o2 | {16{s8}}, o1 | {8{s4}}, o0 };

    // round-off 2N+4 -> 2N (OR of 3 LSBs, AND with 4th LSB)
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

    // overflow monitor (verification only)
`ifdef SYNTHESIS
    assign ovf = 1'b0;
`else
    reg ovf_r;
    wire xs = (m4 & x0[11]) | (m8 & x1[7]) | (e2 & x2[15]);
    wire ps = sx0 | sx1 | sx2;
    wire ss = (m4 & s0[11]) | (m8 & s1[7]) | (e2 & s2[15]);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          ovf_r <= 1'b0;
        else if (valid & clr) ovf_r <= 1'b0;
        else if (v)          ovf_r <= ovf_r | (~(xs ^ ps) & (ss ^ xs));
    end
    assign ovf = ovf_r;
`endif

endmodule