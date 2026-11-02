`timescale 1ns / 1ps
// ---------------------------------------------------------------------------
// tu_fsm : controller for the 16x16 precision-scalable systolic tensor unit
//          (optimised: no counters, busy/done are state bits)
//
//   reset       : synchronous, active-high. PEs get rst_n = ~reset.
//   mode        : {mode_1,mode_0} 00 INT4, 01 INT8, 10 INT16, 11 RESERVED.
//   start       : sampled only while busy = 0; mode latched at start, held
//                 until done; start with mode 11 ignored            (G4/G5)
//   input_valid : beat k = column k of A + row k of B; 0 in FEED = bubble
//                 (whole array stalls for that cycle)
//   done        : one-cycle pulse while busy; busy drops next edge
//   latency     : start sample -> done sample = 3N+1 + #bubbles     (G6/G7)
//
// Schedule t = i+j+k : PE(i,j) sits on diagonal d = i+j
//   load      = load_d[i+j]      (steps d   .. d+N-1)
//   acc_valid = acc_valid_d[i+j] (steps d+1 .. d+N)
//   clr       = clr_d[i+j]       (step  d+1, first MAC of the job)
//
// tok[d] = 1 : diagonal d loaded an operand at the previous step. The N beats
// form one contiguous train of 1s that moves one diagonal per step (bubbles
// stall it), so no counters are needed:
//   head of train (beat 0)   : tok[d] & ~tok[d+1]  -> clr
//   FEED ends                : head at N-2 while beat N-1 is being loaded
//   DRAIN ends               : tail at D-1 (last MAC of PE(N-1,N-1))
// tok[D] is one extra flop past the last diagonal, used only for the head test.
//
// c_out: the PE accumulators hold the result after done (no acc_valid) and
// their c_out is isolated while acc_valid = 0, so the array top can drive
// c_out straight from the PEs without a capture register. 'cap' marks the
// cycle before done if a capture register is still wanted.
// ---------------------------------------------------------------------------
module tu_fsm #(
    parameter N = 16                   // N >= 2
) (
    input  wire           clk,
    input  wire           reset,
    input  wire           start,
    input  wire           mode_1,
    input  wire           mode_0,
    input  wire           input_valid,
    output wire           busy,
    output wire           done,
    output reg  [1:0]     mode_q,      // reserved 2'b11 is never latched
    output wire           m4,
    output wire           m8,
    output wire           m16,
    output wire           e1,          // nibble 1 active (INT8 | INT16)
    output wire [3:0]     en,          // multiplier block-column enables
    output wire [3:0]     sgn,         // multiplier sign selects
    output wire           rst_n,       // PE reset (active-low)
    output wire           step,        // array advances at this edge
    output wire           feed,        // a_col / b_row carry a real beat
    output wire [2*N-2:0] load_d,
    output wire [2*N-2:0] acc_valid_d,
    output wire [2*N-2:0] clr_d,
    output wire           cap
);
    localparam D = 2*N - 1;

    // state = {busy, done, ph1, ph0} : busy and done come straight from flops
    localparam [3:0] S_IDLE  = 4'b0000,
                     S_FEED  = 4'b1000,
                     S_DRAIN = 4'b1001,
                     S_FIN   = 4'b1010,
                     S_DONE  = 4'b1100;

    reg [3:0] state;
    reg [D:0] tok;

    wire idle        = ~state[3];
    wire legal_start = idle & start & ~(mode_1 & mode_0);

    assign busy = state[3];
    assign done = state[2];
    assign feed = (state == S_FEED);
    assign step = (state == S_DRAIN) | (feed & input_valid);
    assign cap  = (state == S_FIN);

    wire [D:0] tok_nxt = {tok[D-1:0], feed};

    assign load_d      = {D{step}} & tok_nxt[D-1:0];
    assign acc_valid_d = {D{step}} & tok[D-1:0];
    assign clr_d       = {D{step}} & tok[D-1:0] & ~tok[D:1];

    wire feed_last  = tok[N-2] & ~tok[N-1];        // beat N-1 enters this step
    wire drain_last = tok[D-1] & ~tok[D-2];        // last MAC this step

    always @(posedge clk) begin
        if (reset | legal_start)
            tok <= {(D+1){1'b0}};
        else if (step)
            tok <= tok_nxt;
    end

    always @(posedge clk) begin
        if (reset)
            mode_q <= 2'b00;
        else if (legal_start)
            mode_q <= {mode_1, mode_0};
    end

    always @(posedge clk) begin
        if (reset)
            state <= S_IDLE;
        else case (state)
            S_IDLE  : if (legal_start)               state <= S_FEED;
            S_FEED  : if (input_valid & feed_last)   state <= S_DRAIN;
            S_DRAIN : if (drain_last)                state <= S_FIN;
            S_FIN   :                                state <= S_DONE;
            default :                                state <= S_IDLE;  // S_DONE + illegal
        endcase
    end

    // shared mode decode (one copy for the whole array)
    assign m4    = (mode_q == 2'b00);
    assign m8    = (mode_q == 2'b01);
    assign m16   = (mode_q == 2'b10);
    assign e1    = m8 | m16;
    assign en    = {m16, m16, e1, (m4 | e1)};
    assign sgn   = {m16, 1'b0, m8, m4};
    assign rst_n = ~reset;
endmodule
