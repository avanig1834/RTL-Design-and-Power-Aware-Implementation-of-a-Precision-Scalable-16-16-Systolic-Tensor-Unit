`timescale 1ns / 1ps
// Full adder (shared by multiplier and accumulator)
module pe_fa (input a, b, ci, output s, co);
    wire t = a ^ b;
    assign s  = t ^ ci;
    assign co = (a & b) | (t & ci);
endmodule

// Half adder (shared by multiplier and accumulator)
module pe_ha (input a, b, output s, c);
    assign s = a ^ b;
    assign c = a & b;
endmodule
