`timescale 1ns / 1ps
// Mode decode: done once, in the global controller
module pe_mode_decode (
    input  wire [1:0] mode_q,
    output wire       m4,        // INT4
    output wire       m8,        // INT8
    output wire       m16,       // INT16
    output wire       e1,        // INT8 | INT16 : nibble1 / acc slice S1 active
    output wire [3:0] en,        // multiplier nibble enables
    output wire [3:0] sgn        // multiplier signed-nibble flags
);
    assign m4  = (mode_q == 2'b00);
    assign m8  = (mode_q == 2'b01);
    assign m16 = (mode_q == 2'b10);
    assign e1  = m8 | m16;
    assign en  = {m16, m16, e1, 1'b1};
    assign sgn = {m16, 1'b0, m8, m4};
endmodule
