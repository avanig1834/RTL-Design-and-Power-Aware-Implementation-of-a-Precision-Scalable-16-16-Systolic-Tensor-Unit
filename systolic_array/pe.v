`timescale 1ns / 1ps
// ===========================================================================
// pe : one processing element = DATAPATH ONLY (no FSM, no mode register).
//   All control comes from the global controller and is shared by every PE.
//   Only a_in / b_in and the result outputs are per-PE.
//
//   load       : capture a_in/b_in into the input registers this cycle
//   acc_valid  : accumulate prod into the accumulator this cycle
//   clr        : with acc_valid, replace (not add to) the old accumulator value
//   m4/m8/m16/e1/en/sgn : mode sub-signals, decoded once at the top level
//
//   Flow (no pipeline registers): a_q,b_q -> pe_mult16 -> pe_acc -> acc regs
// ===========================================================================
module pe #(
    parameter TIE_EVEN = 0
)(
    input  wire        clk,
    input  wire        rst_n,
    // global control (broadcast)
    input  wire        m4,
    input  wire        m8,
    input  wire        m16,
    input  wire        e1,
    input  wire [3:0]  en,
    input  wire [3:0]  sgn,
    input  wire        load,
    input  wire        acc_valid,
    input  wire        clr,
    // per-PE data
    input  wire [15:0] a_in,
    input  wire [15:0] b_in,
    // operand pass-through to the right / lower neighbour (systolic array)
    output wire [15:0] a_out,
    output wire [15:0] b_out,
    // per-PE result
    output wire [35:0] acc,
    output wire signed [35:0] c_out,
    output wire [31:0] c_q,
    output wire        ovf
);
    wire signed [31:0] prod;       // multiplier output = accumulator input
    wire [15:0] a_q;
    wire [15:0] b_q;
    assign a_out = a_q;            // the input registers double as the systolic
    assign b_out = b_q;            // pass-through registers (1 cycle per hop)

    pe_operand_regs u_regs (
        .clk(clk), .rst_n(rst_n), .iv(load), .e1(e1), .m16(m16),
        .a_in(a_in), .b_in(b_in), .a_q(a_q), .b_q(b_q));
        
    pe_mult16 #(.MASK_INPUTS(1)) u_mult (
        .a(a_q), .b(b_q), .m8(m8), .m16(m16), .en(en), .sgn(sgn),
        .prod(prod));

    pe_acc #(.TIE_EVEN(TIE_EVEN)) u_acc (
        .clk(clk), .rst_n(rst_n), .m4(m4), .m8(m8), .m16(m16), .e1(e1),
        .clr(clr), .valid(acc_valid), .prod(prod),
        .acc(acc), .c_out(c_out), .c_q(c_q), .ovf(ovf));
endmodule