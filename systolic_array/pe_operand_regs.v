`timescale 1ns / 1ps
// 16-bit A/B input registers with grouped clock gating (inferred ICG style).
// Plain enable flops: Genus (lp_insert_clock_gating) maps each group's enable
// onto a library ICG cell. Port list is unchanged, so pe.v needs no edit.
// Mode (e1, m16) must be static while operations are in flight.
module pe_operand_regs (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        iv,
    input  wire        e1,       // nibble 1 group active  (INT8/16)
    input  wire        m16,      // nibbles 2,3 group active (INT16)
    input  wire [15:0] a_in,
    input  wire [15:0] b_in,
    output reg  [15:0] a_q,
    output reg  [15:0] b_q
);
    wire en_n1 = iv & e1;        // nibble 1 group enable
    wire en_up = iv & m16;       // upper byte group enable

    // group 0 : nibble 0, always used
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_q[3:0] <= 4'b0;  b_q[3:0] <= 4'b0;
        end else if (iv) begin
            a_q[3:0] <= a_in[3:0];  b_q[3:0] <= b_in[3:0];
        end
    end
    // group 1 : nibble 1
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_q[7:4] <= 4'b0;  b_q[7:4] <= 4'b0;
        end else if (en_n1) begin
            a_q[7:4] <= a_in[7:4];  b_q[7:4] <= b_in[7:4];
        end
    end
    // group 2 : nibbles 2,3
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_q[15:8] <= 8'b0;  b_q[15:8] <= 8'b0;
        end else if (en_up) begin
            a_q[15:8] <= a_in[15:8];  b_q[15:8] <= b_in[15:8];
        end
    end
endmodule