`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// bw_mult16_precision_baseline_power_tb.v : power-stimulus testbench
// DUT: bw_mult16_precision_gated (baseline RTL without operand isolation/clock gating)
//
// One simulation run = one scenario = one VCD file (select with +SCEN=n).
// INT4/INT8/INT16 runs each perform 21 directed corner-case transactions
// plus CYCLES randomized transactions.
//
//   +SCEN=0  INT4  only (mode 00)            -> power_int4.vcd
//   +SCEN=1  INT8  only (mode 01)            -> power_int8.vcd
//   +SCEN=2  INT16 only (mode 10)            -> power_int16.vcd
//   +SCEN=3  idle   (mode_in 11 = reserved, start rejected, input_valid=0,
//                    a_in/b_in toggling)     -> power_idle.vcd
//   +SCEN=4  mixed  (bursts of 8..71 cycles: 30% INT4, 30% INT8, 20% INT16,
//                    20% idle)               -> power_mixed.vcd
//
// Optional plus-args:
//   +CYCLES=n     clock cycles in the dumped window         (default 2000)
//   +PERIOD_NS=x  clock period in ns - MUST match the Genus clock constraint
//                                                            (default 10.0)
//   +JUNK=1       random bits on the unused upper operand bits in INT4/INT8
//                 (default 0: operands are sign-extended, like real narrow data)
//   +FULLCMP=1    compare all 32 product bits (default 0: compare only the
//                 bits that carry the result: 8 bits INT4, 16 bits INT8)
//   +SEED=n       random seed                                (default fixed)
//
// Protocol (matches the DUT interface contract):
//   * A "start" cycle: start=1, busy=0, input_valid=0, mode_in=<mode>.
//     mode_q is captured at that posedge. mode_in=2'b11 is rejected by the DUT
//     (mode_q keeps its last legal value) and no input_valid beats follow.
//   * Then input_valid=1 beats, one operand pair per clock, busy=1, with
//     done=1 on the last beat. Mode stays fixed from start through done.
//   * Inputs are driven 1 ns AFTER the posedge, so the negedge enable flops of
//     the gated clocks (en_nibble1 / en_upper) see stable values - no race.
//   * The multiplier is combinational from the registered operands, so the
//     product is checked 1 ns after the capturing posedge.
//
// Flow per run: 3 reset cycles, 1 INT16 preload (start + 1 beat) so every
// register holds defined data (no X in the VCD), then $dumpvars on the DUT.
// The VCD therefore contains only the measured workload. Products are checked
// against an independent signed '*' model; idle cycles check that the product
// is held (no register or mode_q change).
//-----------------------------------------------------------------------------
module bw_mult16_precision_gated_power_tb;

    reg                clk;
    reg                rst;
    reg                start;
    reg                busy;
    reg                done;
    reg                input_valid;
    reg  [1:0]         mode_in;
    reg  [15:0]        a_in, b_in;
    wire [1:0]         mode_q;
    wire [15:0]        a_q, b_q;
    wire signed [31:0] product;

    bw_mult16_precision_gated dut (
        .clk(clk), .rst(rst), .start(start), .busy(busy), .done(done),
        .input_valid(input_valid), .mode_in(mode_in),
        .a_in(a_in), .b_in(b_in),
        .mode_q(mode_q), .a_q(a_q), .b_q(b_q), .product(product)
    );

    //-------------------------------------------------------------------------
    // Plus-args and clock
    //-------------------------------------------------------------------------
    integer scen, cycles, junk, seed, fullcmp;
    real    period, half;
    reg     clk_en;

    initial begin
        clk = 0; clk_en = 0;
        rst = 1; start = 0; busy = 0; done = 0; input_valid = 0;
        mode_in = 2'b11; a_in = 16'h0; b_in = 16'h0;
        if (!$value$plusargs("SCEN=%d",      scen))   scen   = 2;
        if (!$value$plusargs("CYCLES=%d",    cycles)) cycles = 2000;
        if (!$value$plusargs("JUNK=%d",      junk))   junk   = 0;
        if (!$value$plusargs("FULLCMP=%d",   fullcmp)) fullcmp = 0;
        if (!$value$plusargs("SEED=%d",      seed))   seed   = 32'h0B00_5EED;
        if (!$value$plusargs("PERIOD_NS=%f", period)) period = 10.0;
        half = period / 2.0;
        clk_en = 1;
    end

    always begin
        wait (clk_en);
        #(half) clk = ~clk;
    end

    //-------------------------------------------------------------------------
    // Counters / checking
    //-------------------------------------------------------------------------
    integer err, nops;
    integer ge0, ge1, ge2, nmode0, nmode1, nmode2, nmode3;

    // Baseline has no gated clocks. Count common-clock operand-register updates.
    always @(posedge clk) if (input_valid && (mode_q != 2'b11) && !rst) ge0 = ge0 + 1;

    reg signed [15:0] sa, sb;
    reg signed [31:0] pr;
    reg [31:0] e, mk;
    reg [1:0]  cur_mode;          // mode of the transaction in progress
    reg [31:0] prev_prod;         // last checked product (for idle hold check)

    // Start cycle: capture mode_q (or get rejected for 2'b11).
    task do_start(input [1:0] m, input [15:0] a, input [15:0] b);
        begin
            start = 1; busy = 0; done = 0; input_valid = 0; rst = 0;
            mode_in = m; a_in = a; b_in = b;
            @(posedge clk); #1;
            start = 0;
            if (m != 2'b11) cur_mode = m;
            nops = nops + 1;
        end
    endtask

    // One complete multiplication transaction (start + one valid beat).
    task run_pair(input [1:0] m, input [15:0] a, input [15:0] b);
        begin
            do_start(m, a, b);
            do_beat(a, b, 1'b1);
        end
    endtask

    // Directed corner cases for the selected signed precision.
    // Values are encoded at the active width and sign-extended to 16 bits.
    task run_directed(input [1:0] m);
        reg [15:0] zero_v, one_v, negone_v, maxpos_v, maxneg_v;
        reg [15:0] pos_v, neg_v, pos2_v, neg2_v;
        begin
            case (m)
                2'b00: begin // signed INT4: -8..+7
                    zero_v=16'h0000; one_v=16'h0001; negone_v=16'hFFFF;
                    maxpos_v=16'h0007; maxneg_v=16'hFFF8;
                    pos_v=16'h0003; neg_v=16'hFFFD;
                    pos2_v=16'h0006; neg2_v=16'hFFFA;
                end
                2'b01: begin // signed INT8: -128..+127
                    zero_v=16'h0000; one_v=16'h0001; negone_v=16'hFFFF;
                    maxpos_v=16'h007F; maxneg_v=16'hFF80;
                    pos_v=16'h0035; neg_v=16'hFFCB;
                    pos2_v=16'h007E; neg2_v=16'hFF81;
                end
                default: begin // signed INT16: -32768..+32767
                    zero_v=16'h0000; one_v=16'h0001; negone_v=16'hFFFF;
                    maxpos_v=16'h7FFF; maxneg_v=16'h8000;
                    pos_v=16'h1234; neg_v=16'hEDCC;
                    pos2_v=16'h7FFE; neg2_v=16'h8001;
                end
            endcase

            // Zero cases; identity (multiply by +1); +/-1.
            run_pair(m, zero_v, zero_v);
            run_pair(m, zero_v, maxpos_v);
            run_pair(m, maxneg_v, zero_v);
            run_pair(m, one_v, one_v);
            run_pair(m, one_v, maxpos_v);
            run_pair(m, maxpos_v, one_v);
            run_pair(m, negone_v, one_v);
            run_pair(m, one_v, negone_v);

            // Positive x positive, negative x negative, and mixed signs.
            run_pair(m, pos_v, pos2_v);
            run_pair(m, neg_v, neg2_v);
            run_pair(m, pos_v, neg_v);
            run_pair(m, neg_v, pos_v);
            run_pair(m, pos2_v, neg2_v);

            // Signed limits and near-limit values.
            run_pair(m, maxpos_v, maxpos_v);
            run_pair(m, maxneg_v, maxneg_v);
            run_pair(m, maxpos_v, maxneg_v);
            run_pair(m, maxneg_v, maxpos_v);
            run_pair(m, maxpos_v, negone_v);
            run_pair(m, maxneg_v, negone_v);
            run_pair(m, pos2_v, maxpos_v);
            run_pair(m, neg2_v, maxneg_v);
        end
    endtask

    // One input_valid beat: operands captured at the next posedge, product
    // checked right after it.
    task do_beat(input [15:0] a, input [15:0] b, input last);
        begin
            start = 0; busy = 1; done = last; input_valid = 1; rst = 0;
            mode_in = cur_mode; a_in = a; b_in = b;
            e = 32'd0; mk = 32'hFFFF_FFFF;
            case (cur_mode)
                2'b00: begin sa = {{12{a[3]}}, a[3:0]}; sb = {{12{b[3]}}, b[3:0]}; if (!fullcmp) mk = 32'hFF;   end
                2'b01: begin sa = {{8{a[7]}},  a[7:0]}; sb = {{8{b[7]}},  b[7:0]}; if (!fullcmp) mk = 32'hFFFF; end
                default: begin sa = a; sb = b; end
            endcase
            pr = sa * sb; e = pr;
            @(posedge clk); #1;
            if ((product & mk) !== (e & mk)) begin
                err = err + 1;
                if (err <= 10) $display("FAIL t=%0t mode=%b A=%h B=%h got=%h exp=%h",
                                        $time, cur_mode, a, b, product, e);
            end
            prev_prod = product;
            nops = nops + 1;
            if (last) begin busy = 0; done = 0; input_valid = 0; end
        end
    endtask

    // Idle cycle: reserved mode_in, no valid data, bus toggling. Nothing may
    // change inside the DUT, so the product must be held.
    task do_idle(input [15:0] a, input [15:0] b);
        begin
            start = 0; busy = 0; done = 0; input_valid = 0; rst = 0;
            mode_in = 2'b11; a_in = a; b_in = b;
            @(posedge clk); #1;
            if (product !== prev_prod) begin
                err = err + 1;
                if (err <= 10) $display("FAIL t=%0t idle: product changed got=%h held=%h",
                                        $time, product, prev_prod);
            end
            nops = nops + 1;
        end
    endtask

    // operand generator: realistic narrow data is sign-extended onto the bus
    reg [15:0] ra, rb, ga, gb;
    task gen(input [1:0] m);
        begin
            ra = $random(seed); rb = $random(seed);
            case (m)
                2'b00: if (junk) begin ga = ra; gb = rb; end
                       else begin ga = {{12{ra[3]}}, ra[3:0]}; gb = {{12{rb[3]}}, rb[3:0]}; end
                2'b01: if (junk) begin ga = ra; gb = rb; end
                       else begin ga = {{8{ra[7]}},  ra[7:0]}; gb = {{8{rb[7]}},  rb[7:0]}; end
                default: begin ga = ra; gb = rb; end    // INT16 and idle-bus toggling
            endcase
        end
    endtask

    integer n, burst, pick;
    reg [1:0] bm;

    initial begin
        err = 0; nops = 0; ge0 = 0; ge1 = 0; ge2 = 0;
        nmode0 = 0; nmode1 = 0; nmode2 = 0; nmode3 = 0;
        cur_mode = 2'b10; prev_prod = 32'd0;
        wait (clk_en);
        @(posedge clk); #1;

        // reset (3 cycles, async reset in DUT), then one INT16 preload
        // (start + 1 beat) so no register is X
        rst = 1;
        repeat (3) begin @(posedge clk); #1; end
        rst = 0;
        gen(2'b10);
        do_start(2'b10, ga, gb);
        gen(2'b10);
        do_beat(ga, gb, 1'b1);

        // ---- start the VCD for this scenario (DUT hierarchy only) ----------
        case (scen)
            0: $dumpfile("power_int4.vcd");
            1: $dumpfile("power_int8.vcd");
            2: $dumpfile("power_int16.vcd");
            3: $dumpfile("power_idle.vcd");
            default: $dumpfile("power_mixed.vcd");
        endcase
        $dumpvars(0, dut);
        ge0 = 0; ge1 = 0; ge2 = 0;

        case (scen)
            0: begin
                // Directed corner cases first, then random INT4 workload.
                run_directed(2'b00);
                gen(2'b00); do_start(2'b00, ga, gb);
                for (n = 0; n < cycles; n = n + 1) begin
                    gen(2'b00); do_beat(ga, gb, (n == cycles-1)); nmode0 = nmode0 + 1;
                end
            end
            1: begin
                // Directed corner cases first, then random INT8 workload.
                run_directed(2'b01);
                gen(2'b01); do_start(2'b01, ga, gb);
                for (n = 0; n < cycles; n = n + 1) begin
                    gen(2'b01); do_beat(ga, gb, (n == cycles-1)); nmode1 = nmode1 + 1;
                end
            end
            2: begin
                // Directed corner cases first, then random INT16 workload.
                run_directed(2'b10);
                gen(2'b10); do_start(2'b10, ga, gb);
                for (n = 0; n < cycles; n = n + 1) begin
                    gen(2'b10); do_beat(ga, gb, (n == cycles-1)); nmode2 = nmode2 + 1;
                end
            end
            3: begin
                for (n = 0; n < cycles; n = n + 1) begin
                    gen(2'b11); do_idle(ga, gb); nmode3 = nmode3 + 1;
                end
            end
            default: begin
                n = 0;
                while (n < cycles) begin
                    pick  = {$random(seed)} % 10;
                    burst = 8 + ({$random(seed)} % 64);
                    if      (pick < 3) bm = 2'b00;
                    else if (pick < 6) bm = 2'b01;
                    else if (pick < 8) bm = 2'b10;
                    else               bm = 2'b11;
                    if (bm != 2'b11) begin gen(bm); do_start(bm, ga, gb); end
                    else             begin gen(bm); do_start(bm, ga, gb); end // rejected start
                    while (burst > 0 && n < cycles) begin
                        gen(bm);
                        if (bm == 2'b11) do_idle(ga, gb);
                        else             do_beat(ga, gb, (burst == 1) || (n == cycles-1));
                        case (bm) 2'b00: nmode0 = nmode0 + 1; 2'b01: nmode1 = nmode1 + 1;
                                  2'b10: nmode2 = nmode2 + 1; default: nmode3 = nmode3 + 1; endcase
                        burst = burst - 1; n = n + 1;
                    end
                end
            end
        endcase

        @(negedge clk);
        $display("--------------------------------------------------------------");
        $display("Scenario %0d | window %0d cycles | period %0.2f ns | junk=%0d", scen, cycles, period, junk);
        $display("Cycles by mode  INT4=%0d INT8=%0d INT16=%0d idle(11)=%0d", nmode0, nmode1, nmode2, nmode3);
        $display("Baseline common-clock operand-register update events = %0d", ge0);
        $display("Functional errors = %0d  ->  %s", err, (err == 0) ? "OK" : "FAIL");
        $display("--------------------------------------------------------------");
        $finish;
    end

endmodule