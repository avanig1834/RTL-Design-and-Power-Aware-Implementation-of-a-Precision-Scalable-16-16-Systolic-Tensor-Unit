//==============================================================================
// tu_tb_all.sv : COMPLETE class-based SystemVerilog testbench (single file)
//                16x16 Precision-Scalable Systolic Tensor Unit
//
// Order : 1) interface tu_if   2) package tu_pkg (txn, coverage, generator,
//         driver, monitor, scoreboard, toggle monitor, schedule checker,
//         env, test)  3) tb_top
//
// Compile (Xcelium) :
//   xrun -sv -access +rwc -timescale 1ns/1ps -coverage all -top tb_top \
//        tu_tb_all.sv <your_dut>.sv +TEST=all       (+TEST=T01..T34, +VCD,
//        +LAT_BASE=<n>, +SCHED_OFS=<n>, +SCHED_STRICT, +NO_SCHED)
//==============================================================================
`timescale 1ns/1ps

//==============================================================================
// tu_if.sv : Interface + clocking blocks + protocol SVA for the
//            16x16 Precision-Scalable Systolic Tensor Unit
//
// DUT interface contract assumed by this testbench (adapt if your RTL differs):
//   * Synchronous, active-high reset.
//   * mode = {mode_1,mode_0} : 00 INT4, 01 INT8, 10 INT16, 11 RESERVED.
//   * 'start' is sampled only when busy==0. Mode is latched at start and kept
//     until done. A start with mode 2'b11 must be ignored (no busy, no done).
//   * After an accepted start, busy goes high on the next edge. The TB then
//     sends 16 beats with input_valid=1 (bubbles allowed while busy):
//        beat k : a_col[i*DW +: DW] = A[i][k]  (column k of A)
//                 b_row[j*DW +: DW] = B[k][j]  (row    k of B)
//     Only the low 4/8/16 bits of every operand are meaningful (two's
//     complement, sign extended internally according to the latched mode).
//   * done is a single-cycle pulse issued while busy; busy drops on the
//     following edge.  c_out[(i*N+j)*AW +: AW] = C[i][j] sign-extended to 36b,
//     valid from done until the next accepted start.
//   * Latency (start sample -> done sample) = LAT_BASE + #bubble cycles,
//     LAT_BASE defaults to 3N+1 : t=i+j+k gives s = 0..3N-2 (3N-1 array steps)
//     + 1 cycle to register c_out + 1 cycle for done. Override: +LAT_BASE=<n>
//==============================================================================

interface tu_if #(parameter int N = 16, parameter int DW = 16, parameter int AW = 36)
                 (input logic clk);

  // ---------------- DUT ports ----------------
  logic                 reset;
  logic                 start;
  logic                 mode_1;
  logic                 mode_0;
  logic                 input_valid;
  logic [N*DW-1:0]      a_col;
  logic [N*DW-1:0]      b_row;
  logic                 busy;
  logic                 done;
  logic [N*N*AW-1:0]    c_out;

  // ---------------- white-box probes (connected in tb_top) -------------------
  logic [1:0]           eff_mode;   // mode latched inside the DUT        (G4)
  logic [N*N*DW-1:0]    pe_a;       // all PE A-operand registers         (G8)
  logic [N*N*DW-1:0]    pe_b;       // all PE B-operand registers         (G8)
  logic [N*N*AW-1:0]    pe_acc;     // all PE accumulators                (G8)

  // ---------------- TB housekeeping ----------------
  bit                   sva_en     = 0;
  int unsigned          sva_errors = 0;

  logic [1:0] mode_pins;
  assign mode_pins = {mode_1, mode_0};

  // ---------------- clocking blocks ----------------
  clocking drv_cb @(posedge clk);
    default input #1step output #1;
    output reset, start, mode_1, mode_0, input_valid, a_col, b_row;
    input  busy, done;
  endclocking

  clocking mon_cb @(posedge clk);
    default input #1step;
    input reset, start, mode_1, mode_0, input_valid, a_col, b_row;
    input busy, done, c_out;
    input eff_mode, pe_a, pe_b, pe_acc;
  endclocking

  modport DRV (clocking drv_cb, input clk);
  modport MON (clocking mon_cb, input clk);
  modport DUT (input clk, reset, start, mode_1, mode_0, input_valid, a_col, b_row,
               output busy, done, c_out);

  // ===========================================================================
  //  Protocol assertions (G4, G5, G6)
  // ===========================================================================
  default clocking sva_cb @(posedge clk); endclocking

  `define TU_SVA_FAIL(msg) begin sva_errors++; $error("[SVA] %s @%0t", msg, $time); end

  // no X on control outputs once out of reset
  a_no_x: assert property (disable iff (!sva_en) !reset |-> !$isunknown({busy, done}))
    else `TU_SVA_FAIL("busy/done unknown")

  // reset clears busy/done on the next edge
  a_reset: assert property (disable iff (!sva_en) reset |=> (!busy && !done))
    else `TU_SVA_FAIL("busy/done not cleared by reset")

  // a legal start while idle raises busy
  a_start_busy: assert property (disable iff (!sva_en || reset)
                  (start && !busy && mode_pins != 2'b11) |=> busy)
    else `TU_SVA_FAIL("legal start did not raise busy")

  // G5 : reserved mode never starts arithmetic
  a_rsvd_no_start: assert property (disable iff (!sva_en || reset)
                  (start && !busy && mode_pins == 2'b11) |=> (!busy && !done))
    else `TU_SVA_FAIL("reserved mode 2'b11 started an operation")

  // done is a single-cycle pulse, only while busy, and busy drops after it
  a_done_pulse:   assert property (disable iff (!sva_en || reset) done |=> !done)
    else `TU_SVA_FAIL("done longer than one cycle")
  a_done_in_busy: assert property (disable iff (!sva_en || reset) done |-> busy)
    else `TU_SVA_FAIL("done asserted while not busy")
  a_busy_drop:    assert property (disable iff (!sva_en || reset) done |=> !busy)
    else `TU_SVA_FAIL("busy did not drop after done")
  a_busy_hold:    assert property (disable iff (!sva_en || reset) (busy && !done) |=> busy)
    else `TU_SVA_FAIL("busy dropped without done")

  // TB-side protocol rule : input_valid only while busy
  a_valid_only_busy: assert property (disable iff (!sva_en || reset) input_valid |-> busy)
    else `TU_SVA_FAIL("input_valid driven while not busy")

  // G4 : internal mode is stable for the whole operation
  a_mode_stable: assert property (disable iff (!sva_en || reset)
                  (busy && $past(busy) && !$past(reset)) |-> (eff_mode == $past(eff_mode)))
    else `TU_SVA_FAIL("latched mode changed while busy")

  // result is held stable while idle (also covers reserved-mode start, R5)
  a_result_stable: assert property (disable iff (!sva_en || reset)
                  (!busy && $past(!busy) && !$past(reset)) |-> $stable(c_out))
    else `TU_SVA_FAIL("c_out changed while idle")

  // busy may only rise because of a legal start sampled while idle and out of reset
  //   (covers start held during reset, T33, and start held across done, T32)
  a_no_spont_busy: assert property (disable iff (!sva_en)
                  (!busy && !(start && !reset && mode_pins != 2'b11)) |=> !busy)
    else `TU_SVA_FAIL("busy rose without a legal start")

  // ---------------- protocol cover points ----------------
  c_start_done : cover property (disable iff (!sva_en || reset) (start && !busy) ##[1:$] done);
  c_back2back  : cover property (disable iff (!sva_en || reset) done ##1 (start && !busy));
  c_rsvd_start : cover property (disable iff (!sva_en || reset) start && !busy && mode_pins == 2'b11);
  c_start_busy : cover property (disable iff (!sva_en || reset) start && busy);
  c_bubble     : cover property (disable iff (!sva_en || reset) busy && !input_valid ##1 input_valid);
  c_reset_busy : cover property (disable iff (!sva_en) busy && reset);
  c_start_rst  : cover property (disable iff (!sva_en) start && reset ##1 !reset && start ##1 busy);

  `undef TU_SVA_FAIL
endinterface

//==============================================================================
// tu_pkg.sv : class-based SystemVerilog testbench package
//             16x16 Precision-Scalable Systolic Tensor Unit
//
//   tu_txn        - transaction object (stimulus + observed result)
//   tu_coverage   - functional coverage (cp_mode, cp_test_kind, signs, ...)
//   tu_generator  - directed/random test sequences T01..T34
//   tu_driver     - drives the DUT pins through the virtual interface
//   tu_monitor    - passively reconstructs every operation from the pins
//   tu_scoreboard - golden model, overflow flag, latency formula, checks
//   tu_toggle_mon - precision-gating evidence (per-slice toggle counts)
//   tu_sched_chk  - cycle-by-cycle systolic schedule checker (t = i+j+k)
//   tu_env        - builds and connects everything
//   tu_test       - test selection (+TEST=<Txx|all>) and final report
//==============================================================================

package tu_pkg;

  parameter int N  = 16;   // array size
  parameter int DW = 16;   // operand pin width
  parameter int AW = 36;   // c_out width per element
  parameter int NUM_TESTS = 34;  // T01..T34

  // latency formula: start-sample -> done-sample = LAT_BASE + #bubbles
  // (override with +LAT_BASE=<n> to match a different RTL pipeline)
  int LAT_BASE = 3*N + 1; // 3N-1 systolic steps (s=0..3N-2) + c_out register + done

  typedef enum bit [1:0] {INT4 = 2'b00, INT8 = 2'b01, INT16 = 2'b10, RSVD = 2'b11} mode_e;

  typedef enum {K_ZERO, K_IDENTITY, K_POSITIVE, K_NEGATIVE, K_MIXED,
                K_MAXPOS, K_MAXNEG, K_MAXMIX, K_ACC_EXT, K_SPARSE,
                K_RANDOM, K_DIRTY,
                // corner-case kinds (T27..T34) - keep after K_DIRTY (T12 loops up to K_DIRTY)
                K_BOUNDARY, K_MINUS1, K_MIN_X_M1, K_ADV_UPPER, K_B_IDENTITY, K_PERM,
                K_SAMEDATA} kind_e;

  typedef enum {ACT_OP, ACT_RESET, ACT_IDLE} act_e;

  typedef enum {EXP_RESULT, EXP_ABORT, EXP_IGNORED} exp_e;

  typedef enum {EV_START, EV_DONE, EV_B2B, EV_RSVD, EV_START_BUSY,
                EV_BUBBLE, EV_RESET_BUSY, EV_RESET_IDLE, EV_WIGGLE,
                EV_START_HELD, EV_START_IN_RESET, EV_MODE_GLITCH} ev_e;

  //==============================================================================
  // tu_txn.sv : transaction object
  //   * One object = one "action" for the driver (operation, reset, idle) and
  //     one observed item from the monitor (result / ignored / aborted).
  //==============================================================================
  class tu_txn;

    // --------------------------- stimulus fields ------------------------------
    rand mode_e        mode;          // requested mode (may be RSVD)
    rand kind_e        kind;          // data pattern
    rand int unsigned  bubble_pct;    // % chance of an input_valid bubble per cycle
    rand bit           dirty_upper;   // put garbage above the active width

    constraint c_mode_legal  { soft mode != RSVD; }
    constraint c_bubble      { soft bubble_pct == 0; bubble_pct <= 90; }
    constraint c_dirty       { soft dirty_upper == 0; }

    act_e              action      = ACT_OP;
    int                test_id     = 0;
    int                n_cycles    = 2;     // ACT_RESET / ACT_IDLE length
    int                gap         = 1;     // idle cycles before start
    bit                back2back   = 0;     // start in the very first legal cycle
    bit                mode_wiggle = 0;     // toggle mode pins while busy  (G4)
    bit                start_busy  = 0;     // pulse start while busy      (R6)
    int                reset_at    = -1;    // assert reset after this many busy cycles
    bit                start_hold  = 0;     // keep start high (other mode) for the whole op
    int                pre_reset   = 0;     // reset cycles right before start (start rises on reset release)
    bit                start_in_rst= 0;     // with pre_reset : start already high during reset
    bit                mode_glitch = 0;     // other mode before start, RSVD pins right after start
    int                same_grp    = -1;    // same A/B replayed in several modes (cross-mode check)

    bit signed [DW-1:0] A[N][N];            // operands as driven on the 16-bit pins
    bit signed [DW-1:0] B[N][N];

    // --------------------------- observed / expected --------------------------
    exp_e              expect_kind = EXP_RESULT;
    exp_e              obs_kind    = EXP_RESULT;
    logic signed [AW-1:0] C[N][N];          // DUT result (sign extended to 36b)
    int                latency     = 0;     // start-sample -> done-sample cycles
    int                bubbles     = 0;     // busy && !input_valid before last beat
    int                beats       = 0;     // input beats observed
    bit                c_unchanged = 1;     // for reserved-mode starts
    bit                wiggled     = 0;     // mode pins changed while busy
    bit                mode_ok     = 1;     // DUT latched mode == start mode for whole op
    static int         id_cnt      = 0;
    int                uid;

    function new();
      uid = id_cnt++;
    endfunction

    // -------------------------- helpers ----------------------------------------
    static function int width(mode_e m);
      case (m) INT4: return 4; INT8: return 8; default: return 16; endcase
    endfunction

    static function int acc_width(mode_e m);
      case (m) INT4: return 12; INT8: return 20; default: return 36; endcase
    endfunction

    static function longint max_pos(mode_e m); return (longint'(1) << (width(m)-1)) - 1; endfunction
    static function longint min_neg(mode_e m); return -(longint'(1) << (width(m)-1));    endfunction

    // interpret the low 'width' bits of a 16-bit pin value as two's complement
    static function longint sext(bit [DW-1:0] v, mode_e m);
      int w = width(m);
      longint r = v & ((longint'(1) << w) - 1);
      if (r[w-1]) r = r - (longint'(1) << w);
      return r;
    endfunction

    // pack an in-range value onto the 16-bit pin, optionally with dirty upper bits
    function bit [DW-1:0] to_pin(longint val);
      int w = width(mode);
      bit [DW-1:0] p = val[DW-1:0];
      if (dirty_upper && w < DW) begin
        bit [DW-1:0] g = $urandom();
        bit [DW-1:0] lo_mask = (1 << w) - 1;
        p = (p & lo_mask) | (g & ~lo_mask);
      end
      return p;
    endfunction

    function longint rnd(longint lo, longint hi);
      longint span = hi - lo + 1;
      return lo + longint'({$urandom(), $urandom()} % span);
    endfunction

    function longint gen_val(kind_e k, int i, int j, bit is_a);
      longint mx = max_pos(mode), mn = min_neg(mode);
      case (k)
        K_ZERO     : return 0;
        K_IDENTITY : return is_a ? ((i == j) ? 1 : 0) : rnd(mn, mx);
        K_POSITIVE : return rnd(1, mx);
        K_NEGATIVE : return rnd(mn, -1);
        K_MIXED    : return ((i + j) % 2) ? rnd(1, mx) : rnd(mn, -1);
        K_MAXPOS   : return mx;
        K_MAXNEG   : return mn;
        K_MAXMIX   : return is_a ? mx : mn;            // max-pos x max-neg
        K_ACC_EXT  : return ((i + j) % 2) ? mx : mn;   // checkerboard of extremes
        K_SPARSE   : return ($urandom_range(3) == 0) ? rnd(mn, mx) : 0;
        K_BOUNDARY,
        K_ADV_UPPER: begin                             // {min, min+1, -1, 0, 1, max-1, max}
                       longint bv[7] = '{mn, mn + 1, -1, 0, 1, mx - 1, mx};
                       return bv[$urandom_range(6)];
                     end
        K_MINUS1   : return -1;                        // (-1) x (-1) everywhere
        K_MIN_X_M1 : return is_a ? mn : -1;            // min x (-1) = +2^(w-1) : not representable in w bits
        K_B_IDENTITY: return is_a ? rnd(mn, mx) : ((i == j) ? 1 : 0);              // C must equal A
        K_PERM     : return is_a ? rnd(mn, mx) : ((i + j == N - 1) ? 1 : 0);       // C = A with columns reversed
        K_SAMEDATA : return rnd(-8, 7);                // INT4 range, replayed in every mode
        default    : return rnd(mn, mx);               // K_RANDOM / K_DIRTY
      endcase
    endfunction

    // fill A and B according to kind
    function void build();
      if (mode == RSVD) begin
        foreach (A[i, j]) begin A[i][j] = $urandom(); B[i][j] = $urandom(); end
        return;
      end
      if (kind == K_DIRTY) dirty_upper = 1;
      foreach (A[i, j]) begin
        A[i][j] = to_pin(gen_val(kind, i, j, 1));
        B[i][j] = to_pin(gen_val(kind, i, j, 0));
        if (kind == K_ADV_UPPER) begin
          A[i][j] = adv_pin(A[i][j]);
          B[i][j] = adv_pin(B[i][j]);
        end
      end
    endfunction

    // adversarial upper bits : fill everything above the active width with the
    // INVERSE of the true sign (INT4 : +7 -> 16'hFFF7, -8 -> 16'h0008).
    // A DUT that sign-extends from bit 15 instead of bit w-1 gets every value wrong.
    function bit [DW-1:0] adv_pin(bit [DW-1:0] p);
      int w = width(mode);
      bit [DW-1:0] lo_mask;
      if (w >= DW) return p;
      lo_mask = (1 << w) - 1;
      return (p & lo_mask) | (p[w-1] ? '0 : ~lo_mask);
    endfunction

    function void post_randomize();
      build();
    endfunction

    // golden model : exact C = A x B with mode-dependent sign extension
    function void golden(output longint G[N][N], output bit ovf);
      longint lim_hi = (longint'(1) << (acc_width(mode)-1)) - 1;
      longint lim_lo = -(longint'(1) << (acc_width(mode)-1));
      ovf = 0;
      for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) begin
          G[i][j] = 0;
          for (int k = 0; k < N; k++)
            G[i][j] += sext(A[i][k], mode) * sext(B[k][j], mode);
          if (G[i][j] > lim_hi || G[i][j] < lim_lo) ovf = 1;
        end
    endfunction

    function tu_txn copy();
      copy = new();
      copy.mode = mode;  copy.kind = kind;  copy.bubble_pct = bubble_pct;
      copy.dirty_upper = dirty_upper;  copy.action = action;  copy.test_id = test_id;
      copy.n_cycles = n_cycles;  copy.gap = gap;  copy.back2back = back2back;
      copy.mode_wiggle = mode_wiggle;  copy.start_busy = start_busy;
      copy.reset_at = reset_at;  copy.expect_kind = expect_kind;
      copy.start_hold = start_hold;  copy.pre_reset = pre_reset;
      copy.start_in_rst = start_in_rst;  copy.mode_glitch = mode_glitch;
      copy.same_grp = same_grp;
      copy.A = A;  copy.B = B;
    endfunction

    function string sprint();
      return $sformatf("[T%02d #%0d] act=%s mode=%s kind=%s bub=%0d%% dirty=%0b b2b=%0b wig=%0b sbusy=%0b rst@%0d hold=%0b prerst=%0d/%0b glitch=%0b grp=%0d",
                       test_id, uid, action.name(), mode.name(), kind.name(), bubble_pct,
                       dirty_upper, back2back, mode_wiggle, start_busy, reset_at,
                       start_hold, pre_reset, start_in_rst, mode_glitch, same_grp);
    endfunction
  endclass

  //==============================================================================
  // tu_coverage.sv : functional coverage model
  //==============================================================================
  class tu_coverage;

    // R1 : cp_mode, cp_test_kind      R5 : cp_mode[RSVD]
    covergroup cg_op with function sample(mode_e m, kind_e k);
      option.per_instance = 1;
      cp_mode      : coverpoint m { bins INT4 = {INT4}; bins INT8 = {INT8};
                                    bins INT16 = {INT16}; bins RSVD = {RSVD}; }
      cp_test_kind : coverpoint k;
      cx_mode_kind : cross cp_mode, cp_test_kind { ignore_bins rsvd = binsof(cp_mode.RSVD); }
    endgroup

    // R2 : cp_a_sign x cp_b_sign (per product term A[i][k]*B[k][j])
    covergroup cg_sign with function sample(mode_e m, int sa, int sb);
      option.per_instance = 1;
      cp_sign_mode : coverpoint m { bins INT4 = {INT4}; bins INT8 = {INT8}; bins INT16 = {INT16}; }
      cp_a_sign    : coverpoint sa { bins neg = {-1}; bins zero = {0}; bins pos = {1}; }
      cp_b_sign    : coverpoint sb { bins neg = {-1}; bins zero = {0}; bins pos = {1}; }
      cx_sign      : cross cp_a_sign, cp_b_sign;
      cx_sign_mode : cross cp_sign_mode, cp_a_sign, cp_b_sign;
    endgroup

    // R3 : cp_acc_extreme
    covergroup cg_acc with function sample(mode_e m, int ext);
      option.per_instance = 1;
      cp_acc_mode    : coverpoint m { bins INT4 = {INT4}; bins INT8 = {INT8}; bins INT16 = {INT16}; }
      cp_acc_extreme : coverpoint ext { bins normal = {0}; bins max_pos = {1}; bins max_neg = {2}; }
      cx_acc         : cross cp_acc_mode, cp_acc_extreme;
    endgroup

    // R4 : cx_prev_mode x cur_mode
    covergroup cg_switch with function sample(mode_e p, mode_e c);
      option.per_instance = 1;
      cp_prev_mode : coverpoint p { bins INT4 = {INT4}; bins INT8 = {INT8}; bins INT16 = {INT16}; }
      cp_cur_mode  : coverpoint c { bins INT4 = {INT4}; bins INT8 = {INT8}; bins INT16 = {INT16}; }
      cx_prev_mode_cur_mode : cross cp_prev_mode, cp_cur_mode;
    endgroup

    // R6 : cp_ctrl_event
    covergroup cg_ctrl with function sample(ev_e e);
      option.per_instance = 1;
      cp_ctrl_event : coverpoint e;
    endgroup

    // R7 : cp_latency (extra cycles above LAT_BASE)
    covergroup cg_lat with function sample(mode_e m, int extra);
      option.per_instance = 1;
      cp_lat_mode : coverpoint m { bins INT4 = {INT4}; bins INT8 = {INT8}; bins INT16 = {INT16}; }
      cp_latency  : coverpoint extra { bins exact_base = {0}; bins few_bubbles = {[1:8]};
                                       bins many_bubbles = {[9:64]}; bins huge = {[65:$]}; }
      cx_lat_base : cross cp_lat_mode, cp_latency { ignore_bins nb = !binsof(cp_latency.exact_base); }
    endgroup

    function new();
      cg_op     = new();
      cg_sign   = new();
      cg_acc    = new();
      cg_switch = new();
      cg_ctrl   = new();
      cg_lat    = new();
    endfunction

    function void ctrl(ev_e e);
      cg_ctrl.sample(e);
    endfunction

    function void report();
      $display("---------------------------------------------------------------");
      $display(" FUNCTIONAL COVERAGE");
      $display("   cg_op     (cp_mode, cp_test_kind)        : %6.2f %%", cg_op.get_inst_coverage());
      $display("   cg_sign   (cp_a_sign x cp_b_sign)        : %6.2f %%", cg_sign.get_inst_coverage());
      $display("   cg_acc    (cp_acc_extreme)               : %6.2f %%", cg_acc.get_inst_coverage());
      $display("   cg_switch (cx_prev_mode x cur_mode)      : %6.2f %%", cg_switch.get_inst_coverage());
      $display("   cg_ctrl   (cp_ctrl_event)                : %6.2f %%", cg_ctrl.get_inst_coverage());
      $display("   cg_lat    (cp_latency)                   : %6.2f %%", cg_lat.get_inst_coverage());
      $display("---------------------------------------------------------------");
    endfunction
  endclass

  //==============================================================================
  // tu_generator.sv : produces the transaction sequence of every test T01..T34
  //==============================================================================
  class tu_generator;

    mailbox #(tu_txn) mbx;      // generator -> driver
    event             drv_done; // driver finished the current item
    int               tid;
    int               n_sent;
    int               grp_id;   // same-data group counter (T30)

    mode_e LM[3] = '{INT4, INT8, INT16};

    function new(mailbox #(tu_txn) mbx, event drv_done);
      this.mbx      = mbx;
      this.drv_done = drv_done;
    endfunction

    // ------------------------------------------------------------------ helpers
    task send(tu_txn t);
      t.test_id = tid;
      mbx.put(t);
      n_sent++;
      @(drv_done);
    endtask

    function tu_txn mk(mode_e m, kind_e k, int bub = 0);
      tu_txn t = new();
      if (!t.randomize() with { mode == m; kind == k; bubble_pct == bub; })
        $fatal(1, "[GEN] randomize failed");
      return t;
    endfunction

    task op(mode_e m, kind_e k, int bub = 0, int gap = 1, bit b2b = 0,
            bit wiggle = 0, bit sbusy = 0, int rst_at = -1);
      tu_txn t = mk(m, k, bub);
      t.gap         = gap;
      t.back2back   = b2b;
      t.mode_wiggle = wiggle;
      t.start_busy  = sbusy;
      t.reset_at    = rst_at;
      send(t);
    endtask

    // replay the operands of 'src' in mode m (no re-randomisation)
    task replay(tu_txn src, mode_e m, int grp, bit b2b = 0);
      tu_txn t = mk(m, src.kind);
      t.A = src.A;  t.B = src.B;
      t.same_grp  = grp;
      t.back2back = b2b;
      t.gap       = b2b ? 0 : 1;
      send(t);
    endtask

    // INT8-range data, sign-extended to 16 bits, replayed in INT8 and INT16
    task replay8(int grp);
      tu_txn t = mk(INT8, K_RANDOM);
      foreach (t.A[i, j]) begin
        t.A[i][j] = DW'(tu_txn::sext(t.A[i][j], INT8));
        t.B[i][j] = DW'(tu_txn::sext(t.B[i][j], INT8));
      end
      replay(t, INT8, grp);
      replay(t, INT16, grp, 1);
    endtask

    task ctl(mode_e m, kind_e k, bit hold = 0, int prerst = 0, bit st_rst = 0,
             bit glitch = 0, int gap = 1, bit b2b = 0);
      tu_txn t = mk(m, k);
      t.start_hold   = hold;
      t.pre_reset    = prerst;
      t.start_in_rst = st_rst;
      t.mode_glitch  = glitch;
      t.gap          = gap;
      t.back2back    = b2b;
      send(t);
    endtask

    task rsvd(int gap = 1, bit b2b = 0);
      op(RSVD, K_RANDOM, 0, gap, b2b);
    endtask

    task rst(int n = 3);
      tu_txn t = new();
      t.action   = ACT_RESET;
      t.n_cycles = n;
      send(t);
    endtask

    task idle(int n = 2);
      tu_txn t = new();
      t.action   = ACT_IDLE;
      t.n_cycles = n;
      send(t);
    endtask

    function kind_e rkind();
      kind_e k;
      void'(std::randomize(k) with { k inside {K_RANDOM, K_MIXED, K_POSITIVE, K_NEGATIVE, K_SPARSE}; });
      return k;
    endfunction

    // ------------------------------------------------------------- test bodies
    task run(string name);
      tid = name.substr(1, 2).atoi();
      idle(2);
      case (name)
        // ---------------- R1/R2 : data patterns (G1, G2) ----------------------
        "T01": foreach (LM[i]) op(LM[i], K_ZERO);
        "T02": foreach (LM[i]) op(LM[i], K_IDENTITY);
        "T03": repeat (3) op(INT4, K_RANDOM);
        "T04": foreach (LM[i]) op(LM[i], K_POSITIVE);
        "T05": foreach (LM[i]) op(LM[i], K_NEGATIVE);
        "T06": foreach (LM[i]) op(LM[i], K_MIXED);
        "T07": foreach (LM[i]) op(LM[i], K_MAXPOS);
        "T08": foreach (LM[i]) begin op(LM[i], K_MAXNEG); op(LM[i], K_MAXMIX); end
        "T09": repeat (3) op(INT8, K_RANDOM);
        "T10": repeat (3) op(INT16, K_RANDOM);
        "T11": foreach (LM[i]) begin op(LM[i], K_DIRTY); op(LM[i], K_SPARSE); end
        "T12": foreach (LM[i]) for (int k = 0; k <= int'(K_DIRTY); k++) op(LM[i], kind_e'(k));
        // ---------------- R3 : accumulator extremes (G3) ----------------------
        "T13": foreach (LM[i]) begin
                 op(LM[i], K_ACC_EXT); op(LM[i], K_MAXPOS);
                 op(LM[i], K_MAXNEG);  op(LM[i], K_MAXMIX);
               end
        // ---------------- R4 : mode switching (G4) ----------------------------
        "T14": repeat (2) begin op(INT4, K_RANDOM); op(INT8, K_RANDOM); op(INT16, K_RANDOM); end
        "T15": repeat (2) begin
                 op(INT16, K_MIXED, 0, 0, 1); op(INT8, K_MIXED, 0, 0, 1); op(INT4, K_MIXED, 0, 0, 1);
               end
        "T16": foreach (LM[p]) foreach (LM[c]) begin
                 op(LM[p], rkind(), 0, 0, 1);
                 op(LM[c], rkind(), 0, 0, 1, 1);          // mode pins wiggle while busy
               end
        // ---------------- R5 : reserved mode (G5) -----------------------------
        "T17": begin
                 op(INT8, K_RANDOM);
                 rsvd();                                   // result must stay unchanged
                 rsvd(3);
                 op(INT4, K_MIXED);
                 rsvd(0, 1);                               // reserved right after done
                 op(INT16, K_RANDOM, 0, 0, 1);
                 rst(3);
                 rsvd();                                   // reserved after reset
                 op(INT8, K_NEGATIVE);
               end
        // ---------------- R6 : control protocol (G6) --------------------------
        "T18": begin                                       // reset while busy
                 int pts[5] = '{0, 5, 15, 30, 44};
                 foreach (pts[p]) begin
                   op(LM[p % 3], K_RANDOM, 0, 1, 0, 0, 0, pts[p]);
                   op(LM[(p + 1) % 3], K_RANDOM);          // recovery
                 end
               end
        "T19": foreach (LM[i]) op(LM[i], K_RANDOM, 0, 1, 0, 0, 1);   // start while busy
        "T20": begin                                       // back-to-back
                 op(INT4, K_RANDOM);
                 repeat (6) op(LM[$urandom_range(2)], K_RANDOM, 0, 0, 1);
               end
        "T21": foreach (LM[i]) begin                       // input_valid bubbles
                 op(LM[i], K_RANDOM, 20);
                 op(LM[i], K_MIXED, 50);
                 op(LM[i], K_RANDOM, 90);
               end
        "T22": begin                                       // misc reset scenarios
                 rst(3);                                   // reset while idle
                 op(INT8, K_RANDOM);
                 rst(10);                                  // long reset
                 op(INT16, K_MIXED, 0, 1, 0, 0, 0, 40);    // reset in drain phase
                 op(INT4, K_RANDOM);
                 rst(1);                                   // reset right after done
                 op(INT16, K_RANDOM);
               end
        // ---------------- R7 : schedule & latency (G7) ------------------------
        "T23": foreach (LM[i]) begin
                 op(LM[i], K_RANDOM, 0);
                 op(LM[i], K_MAXNEG, 0, 0, 1);
                 op(LM[i], K_RANDOM, 25);
               end
        // ---------------- precision gating evidence (G8) ----------------------
        "T24", "T25", "T26": begin
                 mode_e m = LM[tid - 24];
                 op(m, K_MIXED); op(m, K_NEGATIVE); op(m, K_DIRTY);
                 op(m, K_MAXNEG); op(m, K_ACC_EXT); op(m, K_RANDOM, 30);
               end
        // ======================= corner cases (gap closure) ====================
        // T27 : adversarial upper bits - inverse-sign garbage above the active
        //       width (INT4 +7 -> 0xFFF7, -8 -> 0x0008). Catches sign extension
        //       from the wrong bit / ungated upper bits.            (G2, G8, section 5)
        "T27": foreach (LM[i]) repeat (2) op(LM[i], K_ADV_UPPER);
        // T28 : boundary set {min,min+1,-1,0,1,max-1,max}, (-1)x(-1), min x (-1)
        //       (+2^(w-1) is not representable in w bits -> must not wrap) (G2, G3)
        "T28": foreach (LM[i]) begin
                 op(LM[i], K_BOUNDARY); op(LM[i], K_BOUNDARY, 30);
                 op(LM[i], K_MINUS1);   op(LM[i], K_MIN_X_M1);
               end
        // T29 : B = I (C must equal A) and anti-diagonal permutation  (G1, G7)
        "T29": foreach (LM[i]) begin op(LM[i], K_B_IDENTITY); op(LM[i], K_PERM); end
        // T30 : section 14 - identical operands in every mode give identical C
        "T30": begin
                 repeat (2) begin
                   tu_txn s = mk(INT4, K_SAMEDATA);       // INT4-range, sign-extended to 16b
                   foreach (s.A[i, j]) begin
                     s.A[i][j] = DW'(tu_txn::sext(s.A[i][j], INT4));
                     s.B[i][j] = DW'(tu_txn::sext(s.B[i][j], INT4));
                   end
                   replay(s, INT4, grp_id);  replay(s, INT8, grp_id, 1);  replay(s, INT16, grp_id, 1);
                   grp_id++;
                   replay(s, INT16, grp_id); replay(s, INT4, grp_id, 1);  replay(s, INT8, grp_id);
                   grp_id++;
                 end
                 replay8(grp_id++);
               end
        // T31 : accumulator / result clear between ops - extreme op then
        //       back-to-back zero and identity (no residue)          (G3, G6)
        "T31": foreach (LM[i]) begin
                 op(LM[i], K_MAXNEG);
                 op(LM[i], K_ZERO, 0, 0, 1);
                 op(LM[i], K_ACC_EXT, 0, 0, 1);
                 op(LM[i], K_IDENTITY, 0, 0, 1);
                 op(LM[(i + 1) % 3], K_ZERO, 0, 0, 1);   // clear across a mode change
               end
        // T32 : start held high for the whole op with hopping (also RSVD) mode
        //       pins -> exactly one op in the start mode, then idle   (G4, G6)
        "T32": foreach (LM[i]) begin
                 ctl(LM[i], K_RANDOM, 1);
                 ctl(LM[(i + 2) % 3], K_MIXED, 1, 0, 0, 0, 0, 1);   // held start, back-to-back
               end
        // T33 : start vs reset - start high during reset is ignored, start in the
        //       first cycle after release is accepted                  (G6)
        "T33": foreach (LM[i]) begin
                 ctl(LM[i], K_RANDOM, 0, 3, 1);          // start already high during reset
                 ctl(LM[i], K_MIXED, 0, 1, 1);           // 1-cycle reset with start
                 ctl(LM[i], K_RANDOM, 0, 2, 0);          // start right at reset release
               end
        // T34 : mode pins change the cycle before and right after start; the
        //       latched mode must be the one sampled with start      (G4)
        "T34": foreach (LM[i]) begin
                 ctl(LM[i], K_RANDOM, 0, 0, 0, 1);
                 ctl(LM[i], K_ADV_UPPER, 0, 0, 0, 1);
               end
        default: $error("[GEN] unknown test %s", name);
      endcase
      idle(4);
    endtask
  endclass

  //==============================================================================
  // tu_driver.sv : converts transactions into pin activity (virtual interface)
  //==============================================================================
  class tu_driver;

    virtual tu_if.DRV  vif;
    mailbox #(tu_txn)  gen2drv;   // from generator
    mailbox #(tu_txn)  drv2sb;    // expected items to scoreboard
    event              drv_done;
    bit                last_done; // previous op ended with 'done' sampled this cycle
    int                timeouts;

    function new(virtual tu_if.DRV vif, mailbox #(tu_txn) gen2drv,
                 mailbox #(tu_txn) drv2sb, event drv_done);
      this.vif      = vif;
      this.gen2drv  = gen2drv;
      this.drv2sb   = drv2sb;
      this.drv_done = drv_done;
    endfunction

    task run();
      tu_txn t;
      forever begin
        gen2drv.get(t);
        case (t.action)
          ACT_RESET: do_reset(t.n_cycles);
          ACT_IDLE : repeat (t.n_cycles) @(vif.drv_cb);
          ACT_OP   : do_op(t);
        endcase
        if (t.action != ACT_OP) last_done = 0;
        ->drv_done;
      end
    endtask

    task set_mode(logic [1:0] m);
      vif.drv_cb.mode_1 <= m[1];
      vif.drv_cb.mode_0 <= m[0];
    endtask

    task do_reset(int n);
      vif.drv_cb.reset       <= 1;
      vif.drv_cb.start       <= 0;
      vif.drv_cb.input_valid <= 0;
      repeat (n) @(vif.drv_cb);
      vif.drv_cb.reset       <= 0;
      @(vif.drv_cb);
    endtask

    task do_op(tu_txn t);
      int k = 0, cyc = 0, bub_run = 0, limit;
      tu_txn exp;

      // ---------------- wait for an idle DUT ----------------
      if (!(t.back2back && last_done)) begin
        repeat (t.gap) @(vif.drv_cb);
        for (int w = 0; vif.drv_cb.busy; w++) begin
          if (w > LAT_BASE + 16 * 9 + 50) begin   // DUT stuck busy -> recover
            timeouts++;
            $error("[DRV] DUT stuck busy before %s - applying recovery reset", t.sprint());
            do_reset(2);
            break;
          end
          @(vif.drv_cb);
        end
      end
      last_done = 0;

      // ---------------- expected item for the scoreboard ----------------
      exp = t.copy();
      exp.expect_kind = (t.mode == RSVD)  ? EXP_IGNORED :
                        (t.reset_at >= 0) ? EXP_ABORT   : EXP_RESULT;
      drv2sb.put(exp);

      // ---------------- optional reset right before start (T33) ----------------
      //  start_in_rst=1 : start (and a legal mode) already high while reset is high
      //                   -> must NOT start anything; a_reset/a_no_spont_busy check it
      //  either way     : start is high in the first cycle after reset release
      //                   -> must be accepted (EXP_RESULT)
      if (t.pre_reset > 0) begin
        vif.drv_cb.reset       <= 1;
        vif.drv_cb.input_valid <= 0;
        vif.drv_cb.start       <= t.start_in_rst;
        set_mode(t.mode);
        repeat (t.pre_reset) @(vif.drv_cb);
        vif.drv_cb.reset       <= 0;
      end

      // ---------------- mode glitch around start (T34) ----------------
      //  pins show another legal mode the cycle before start and RSVD right after
      if (t.mode_glitch && t.pre_reset == 0) begin
        set_mode(mode_e'((int'(t.mode) + 1) % 3));
        @(vif.drv_cb);
      end

      // ---------------- start pulse (mode sampled here) ----------------
      set_mode(t.mode);
      vif.drv_cb.start <= 1;
      @(vif.drv_cb);
      vif.drv_cb.start <= t.start_hold;
      if (t.mode_glitch) set_mode(RSVD);

      if (t.mode == RSVD) begin
        @(vif.drv_cb);
        return;
      end

      // ---------------- stream 16 beats, then wait for done ----------------
      limit = LAT_BASE + 16 * 9 + 50;
      forever begin
        if (t.reset_at >= 0 && cyc == t.reset_at) begin
          vif.drv_cb.input_valid <= 0;
          vif.drv_cb.start       <= 0;
          set_mode(t.mode);
          do_reset(2);
          return;
        end

        if (k < N) begin
          if (t.bubble_pct > 0 && bub_run < 8 && $urandom_range(99) < t.bubble_pct) begin
            vif.drv_cb.input_valid <= 0;
            bub_run++;
          end else begin
            logic [N*DW-1:0] col, row;
            for (int i = 0; i < N; i++) begin
              col[i*DW +: DW] = t.A[i][k];   // column k of A
              row[i*DW +: DW] = t.B[k][i];   // row    k of B
            end
            vif.drv_cb.a_col       <= col;
            vif.drv_cb.b_row       <= row;
            vif.drv_cb.input_valid <= 1;
            k++;
            bub_run = 0;
          end
        end else begin
          vif.drv_cb.input_valid <= 0;
        end

        // G4 : mode pins must be ignored while busy
        if (t.mode_wiggle)
          set_mode(2'($urandom()));

        // T32 : start held high for the whole op, mode pins hopping (incl. RSVD)
        if (t.start_hold) begin
          vif.drv_cb.start <= 1;
          set_mode((cyc % 3 == 2) ? RSVD : mode_e'((int'(t.mode) + 1 + cyc % 2) % 3));
        end

        // R6 : a start while busy must be ignored
        if (t.start_busy) begin
          if (cyc == 7 || cyc == 20 || cyc == 33) begin
            vif.drv_cb.start <= 1;
            set_mode(mode_e'((int'(t.mode) + 1) % 3));
          end else begin
            vif.drv_cb.start <= 0;
            set_mode(t.mode);
          end
        end

        @(vif.drv_cb);
        cyc++;
        if (vif.drv_cb.done) break;
        if (cyc > limit) begin
          timeouts++;
          $error("[DRV] timeout waiting for done : %s", t.sprint());
          break;
        end
      end

      vif.drv_cb.input_valid <= 0;
      vif.drv_cb.start       <= 0;
      set_mode(t.mode);
      last_done = vif.drv_cb.done;
    endtask
  endclass

  //==============================================================================
  // tu_monitor.sv : passive monitor - rebuilds every operation from the pins
  //==============================================================================
  class tu_monitor;

    virtual tu_if.MON  vif;
    mailbox #(tu_txn)  mon2sb;
    tu_coverage        cov;

    int spurious_done, lost_busy;

    function new(virtual tu_if.MON vif, mailbox #(tu_txn) mon2sb, tu_coverage cov);
      this.vif    = vif;
      this.mon2sb = mon2sb;
      this.cov    = cov;
    endfunction

    task run();
      tu_txn               tr, rs;          // active op / pending reserved start
      bit                  active, rs_pend, prev_done, prev_rst, bub_cov, wig_cov, hold_cov;
      int                  hold_run;
      int                  rs_cyc;
      logic [N*N*AW-1:0]   rs_snap;

      forever begin
        logic               rst, st, iv, bz, dn;
        logic [1:0]         mp, em;
        logic [N*DW-1:0]    ac, br;
        logic [N*N*AW-1:0]  co;

        @(vif.mon_cb);
        rst = vif.mon_cb.reset;   st = vif.mon_cb.start;   iv = vif.mon_cb.input_valid;
        bz  = vif.mon_cb.busy;    dn = vif.mon_cb.done;    co = vif.mon_cb.c_out;
        mp  = {vif.mon_cb.mode_1, vif.mon_cb.mode_0};      em = vif.mon_cb.eff_mode;
        ac  = vif.mon_cb.a_col;   br = vif.mon_cb.b_row;

        // ---------------- reserved-start bookkeeping (G5) ----------------
        if (rs_pend) begin
          bit fin = rst || (st && !bz) || (rs_cyc >= 4);
          if (!rst && (bz || dn) && !(st && !bz)) rs.obs_kind = EXP_RESULT;  // it started!
          if (fin) begin
            rs.c_unchanged = (co === rs_snap);
            mon2sb.put(rs);
            rs_pend = 0;
          end
          rs_cyc++;
        end

        // ---------------- reset ----------------
        if (rst) begin
          if (st) cov.ctrl(EV_START_IN_RESET);     // T33 : must not start anything
          if (active) begin
            tr.obs_kind = EXP_ABORT;
            mon2sb.put(tr);
            active = 0;
            cov.ctrl(EV_RESET_BUSY);
          end else if (!prev_rst) cov.ctrl(EV_RESET_IDLE);
          prev_rst  = 1;
          prev_done = 0;
          continue;
        end
        prev_rst = 0;

        // ---------------- accepted start ----------------
        if (st && !bz) begin
          if (active) begin                 // busy dropped without done
            lost_busy++;
            tr.obs_kind = EXP_ABORT;
            mon2sb.put(tr);
            active = 0;
          end
          if (mode_e'(mp) == RSVD) begin
            rs = new();
            rs.mode     = RSVD;
            rs.obs_kind = EXP_IGNORED;
            rs_snap     = co;
            rs_pend     = 1;
            rs_cyc      = 0;
            cov.ctrl(EV_RSVD);
            cov.cg_op.sample(RSVD, K_RANDOM);
          end else begin
            tr = new();
            tr.mode     = mode_e'(mp);
            tr.obs_kind = EXP_RESULT;
            active   = 1;
            bub_cov  = 0;
            wig_cov  = 0;
            hold_cov = 0;
            hold_run = 0;
            cov.ctrl(EV_START);
            if (prev_done) cov.ctrl(EV_B2B);
          end
          prev_done = dn;
          continue;
        end

        if (st && bz) cov.ctrl(EV_START_BUSY);

        if (active) begin
          // T34 : pins already moved to RSVD in the first busy cycle
          if (tr.latency == 0 && mode_e'(mp) == RSVD) cov.ctrl(EV_MODE_GLITCH);
          // T32 : start held high while busy for more than N cycles
          hold_run = (st && bz) ? hold_run + 1 : 0;
          if (hold_run > N && !hold_cov) begin cov.ctrl(EV_START_HELD); hold_cov = 1; end
          tr.latency++;
          if (mp != tr.mode) begin
            tr.wiggled = 1;
            if (!wig_cov) begin cov.ctrl(EV_WIGGLE); wig_cov = 1; end
          end
          if (bz && em !== tr.mode) tr.mode_ok = 0;

          if (iv) begin
            if (tr.beats < N)
              for (int i = 0; i < N; i++) begin
                tr.A[i][tr.beats] = ac[i*DW +: DW];
                tr.B[tr.beats][i] = br[i*DW +: DW];
              end
            tr.beats++;
          end else if (bz && tr.beats < N) begin
            tr.bubbles++;
            if (!bub_cov) begin cov.ctrl(EV_BUBBLE); bub_cov = 1; end
          end

          if (dn) begin
            for (int i = 0; i < N; i++)
              for (int j = 0; j < N; j++)
                tr.C[i][j] = co[(i*N + j)*AW +: AW];
            mon2sb.put(tr);
            active = 0;
            cov.ctrl(EV_DONE);
          end
        end else if (dn) begin
          spurious_done++;
          $error("[MON] done observed with no active operation @%0t", $time);
        end

        prev_done = dn;
      end
    endtask
  endclass

  //==============================================================================
  // tu_scoreboard.sv : self-checking scoreboard (G1, G2, G3, G4, G5, G6, G7, G9)
  //   * expected items arrive from the driver (what was requested)
  //   * observed items arrive from the monitor (what the pins showed)
  //   * both streams are in order, so they are paired one-to-one
  //==============================================================================
  class tu_scoreboard;

    mailbox #(tu_txn) drv2sb, mon2sb;
    tu_coverage       cov;
    int               lat_base;

    int     n_items, n_pass, n_fail;
    int     n_result, n_ignored, n_abort;
    int     err_data, err_lat, err_kind, err_mode, err_ovf, err_beats, err_rsvd, err_sext, err_xmode;
    typedef longint mat_t[N][N];
    mat_t   grp_c[int];                 // same-data groups : result of the first run
    mode_e  grp_m[int];
    int     test_pass[int], test_fail[int];
    mode_e  prev_mode;
    bit     have_prev;

    function new(mailbox #(tu_txn) drv2sb, mailbox #(tu_txn) mon2sb, tu_coverage cov, int lat_base);
      this.drv2sb   = drv2sb;
      this.mon2sb   = mon2sb;
      this.cov      = cov;
      this.lat_base = lat_base;
    endfunction

    function int sgn(longint v);
      return (v > 0) ? 1 : (v < 0) ? -1 : 0;
    endfunction

    task run();
      tu_txn e, o;
      forever begin
        drv2sb.get(e);
        mon2sb.get(o);
        check(e, o);
      end
    endtask

    function void check(tu_txn e, tu_txn o);
      int  errs = 0;
      string tag = $sformatf("[SB][T%02d #%0d %s %s]", e.test_id, e.uid, e.mode.name(), e.kind.name());
      n_items++;

      // ------------------------- kind of outcome --------------------------------
      if (o.obs_kind != e.expect_kind) begin
        $error("%s expected outcome %s but observed %s", tag, e.expect_kind.name(), o.obs_kind.name());
        err_kind++; errs++;
      end

      case (e.expect_kind)
        // ===================================================== reserved (G5)
        EXP_IGNORED: begin
          n_ignored++;
          if (!o.c_unchanged) begin
            $error("%s reserved-mode start modified c_out", tag);
            err_rsvd++; errs++;
          end
        end

        // ===================================================== reset abort (G6)
        EXP_ABORT: begin
          n_abort++;
          have_prev = 0;
        end

        // ===================================================== full result
        EXP_RESULT: if (o.obs_kind == EXP_RESULT) begin
          longint G[N][N];
          bit     ovf;
          int     bad = 0, ext = 0;
          longint mx = tu_txn::max_pos(e.mode), mn = tu_txn::min_neg(e.mode);
          int     aw = tu_txn::acc_width(e.mode);
          n_result++;

          // G4 : mode used by the DUT must be the mode requested at start
          if (o.mode != e.mode) begin
            $error("%s monitor saw start mode %s", tag, o.mode.name());
            err_mode++; errs++;
          end
          if (!o.mode_ok) begin
            $error("%s internal mode changed during the operation", tag);
            err_mode++; errs++;
          end

          // stimulus integrity : 16 beats, same operands as requested
          if (o.beats != N) begin
            $error("%s observed %0d input beats (expected %0d)", tag, o.beats, N);
            err_beats++; errs++;
          end
          else if (o.A != e.A || o.B != e.B) begin
            $error("%s operands seen on the pins differ from the generated ones", tag);
            err_beats++; errs++;
          end

          // G3 : golden model must fit in the documented accumulator width
          e.golden(G, ovf);
          if (ovf) begin
            $error("%s golden result exceeds %0d-bit accumulator", tag, aw);
            err_ovf++; errs++;
          end

          // G1/G2/G3 : exact compare (36-bit sign-extended output)
          foreach (G[i, j]) begin
            longint got = longint'(o.C[i][j]);
            if ($isunknown(o.C[i][j]) || got != G[i][j]) begin
              if (bad < 5)
                $error("%s C[%0d][%0d] DUT=%0d (0x%09h) GOLDEN=%0d", tag, i, j, got, o.C[i][j], G[i][j]);
              bad++;
            end
            if (G[i][j] == N * mn * mn)      ext = 1;
            else if (G[i][j] == N * mn * mx && ext == 0) ext = 2;
          end
          if (bad) begin
            $error("%s %0d / %0d result elements mismatched", tag, bad, N*N);
            err_data++; errs++;
          end

          // G2/G3 : c_out bits [35:aw] must be a pure sign extension of bit aw-1
          if (aw < AW) begin
            int sbad = 0;
            foreach (o.C[i, j]) begin
              logic [AW-1:0] v  = o.C[i][j];
              longint        hi = longint'(o.C[i][j]) >>> (aw - 1);   // bits [35:aw-1]
              if (!$isunknown(v) && hi != 0 && hi != -1) begin
                if (sbad < 3)
                  $error("%s C[%0d][%0d]=0x%09h is not sign-extended from bit %0d", tag, i, j, v, aw-1);
                sbad++;
              end
            end
            if (sbad) begin err_sext++; errs++; end
          end

          // section 14 : identical operands give identical results in every mode
          if (e.same_grp >= 0) begin
            if (!grp_c.exists(e.same_grp)) begin
              mat_t r;
              foreach (r[i, j]) r[i][j] = longint'(o.C[i][j]);
              grp_c[e.same_grp] = r;
              grp_m[e.same_grp] = e.mode;
            end else begin
              int xbad = 0;
              foreach (o.C[i, j])
                if (longint'(o.C[i][j]) != grp_c[e.same_grp][i][j]) xbad++;
              if (xbad) begin
                $error("%s same operands: %0d elements differ from the %s run (group %0d)",
                       tag, xbad, grp_m[e.same_grp].name(), e.same_grp);
                err_xmode++; errs++;
              end
            end
          end

          // G7 : deterministic latency = LAT_BASE + bubble cycles
          if (o.latency != lat_base + o.bubbles) begin
            $error("%s latency %0d != LAT_BASE(%0d) + bubbles(%0d)", tag, o.latency, lat_base, o.bubbles);
            err_lat++; errs++;
          end

          // ------------------------- coverage -----------------------------------
          cov.cg_op.sample(e.mode, e.kind);
          cov.cg_acc.sample(e.mode, ext);
          cov.cg_lat.sample(e.mode, o.latency - lat_base);
          if (have_prev) cov.cg_switch.sample(prev_mode, e.mode);
          prev_mode = e.mode;
          have_prev = 1;
          for (int i = 0; i < N; i++)
            for (int k = 0; k < N; k++) begin
              int sa = sgn(tu_txn::sext(e.A[i][k], e.mode));
              for (int j = 0; j < N; j++)   // every product term A[i][k]*B[k][j]
                cov.cg_sign.sample(e.mode, sa, sgn(tu_txn::sext(e.B[k][j], e.mode)));
            end
        end else n_result++;
      endcase

      if (errs) begin
        n_fail++;
        test_fail[e.test_id] = test_fail.exists(e.test_id) ? test_fail[e.test_id] + 1 : 1;
      end else begin
        n_pass++;
        test_pass[e.test_id] = test_pass.exists(e.test_id) ? test_pass[e.test_id] + 1 : 1;
        $display("%t %s PASS  (%s, lat=%0d, bubbles=%0d)", $time, tag, e.expect_kind.name(),
                 o.latency, o.bubbles);
      end
    endfunction

    function int errors();
      return n_fail;
    endfunction

    function void report();
      $display("---------------------------------------------------------------");
      $display(" SCOREBOARD : %0d items, %0d PASS, %0d FAIL", n_items, n_pass, n_fail);
      $display("   results=%0d ignored(RSVD)=%0d aborted(reset)=%0d", n_result, n_ignored, n_abort);
      $display("   err: data=%0d latency=%0d outcome=%0d mode=%0d ovf=%0d beats=%0d rsvd=%0d sext=%0d xmode=%0d",
               err_data, err_lat, err_kind, err_mode, err_ovf, err_beats, err_rsvd, err_sext, err_xmode);
      for (int t = 1; t <= NUM_TESTS; t++)
        if (test_pass.exists(t) || test_fail.exists(t))
          $display("   T%02d : %3d pass %3d fail", t,
                   test_pass.exists(t) ? test_pass[t] : 0, test_fail.exists(t) ? test_fail[t] : 0);
      $display("---------------------------------------------------------------");
    endfunction
  endclass

  //==============================================================================
  // tu_toggle_mon.sv : precision-gating evidence (G8, T24-T26)
  //   Counts bit toggles of every PE operand / accumulator register, split into
  //   the "active" low slice and the "gated" upper slice of the current mode.
  //   In INT4 / INT8 the upper slices must not toggle at all - whatever the
  //   operand sign or the garbage on the upper pin bits.
  //==============================================================================
  class tu_toggle_mon;

    virtual tu_if.MON vif;

    // [mode][0 = low slice, 1 = upper slice]
    longint opd_tog[3][2];
    longint acc_tog[3][2];
    longint busy_cyc[3];

    function new(virtual tu_if.MON vif);
      this.vif = vif;
    endfunction

    static function int popc(logic [AW-1:0] v);
      int c = 0;
      for (int b = 0; b < AW; b++) c += (v[b] === 1'b1);
      return c;
    endfunction

    task run();
      logic [N*N*DW-1:0] pa_q, pb_q;
      logic [N*N*AW-1:0] acc_q;
      logic              bz_q, rst_q;
      logic [1:0]        em_q;

      forever begin
        logic [N*N*DW-1:0] pa, pb;
        logic [N*N*AW-1:0] acc;
        logic              bz, rst;
        logic [1:0]        em;

        @(vif.mon_cb);
        pa  = vif.mon_cb.pe_a;   pb = vif.mon_cb.pe_b;   acc = vif.mon_cb.pe_acc;
        bz  = vif.mon_cb.busy;   rst = vif.mon_cb.reset; em  = vif.mon_cb.eff_mode;

        // count only inside a running op (same mode in both samples, so the
        // one-off clear when an operation starts is not counted)
        if (bz && bz_q && !rst && !rst_q && em == em_q && em != 2'b11) begin
          int m  = em;
          int w  = tu_txn::width(mode_e'(em));
          int aw = tu_txn::acc_width(mode_e'(em));
          logic [DW-1:0] lm = (w  >= DW) ? '1 : ((DW'(1) << w) - 1);
          logic [AW-1:0] am = (aw >= AW) ? '1 : ((AW'(1) << aw) - 1);
          busy_cyc[m]++;
          for (int p = 0; p < N*N; p++) begin
            logic [DW-1:0] da = pa[p*DW +: DW] ^ pa_q[p*DW +: DW];
            logic [DW-1:0] db = pb[p*DW +: DW] ^ pb_q[p*DW +: DW];
            logic [AW-1:0] dc = acc[p*AW +: AW] ^ acc_q[p*AW +: AW];
            opd_tog[m][0] += popc(AW'(da & lm))  + popc(AW'(db & lm));
            opd_tog[m][1] += popc(AW'(da & ~lm)) + popc(AW'(db & ~lm));
            acc_tog[m][0] += popc(dc & am);
            acc_tog[m][1] += popc(dc & ~am);
          end
        end

        pa_q = pa;  pb_q = pb;  acc_q = acc;  bz_q = bz;  rst_q = rst;  em_q = em;
      end
    endtask

    // returns number of gating violations (upper-slice toggles in INT4 / INT8)
    function int errors();
      return (opd_tog[0][1] != 0) + (acc_tog[0][1] != 0) +
             (opd_tog[1][1] != 0) + (acc_tog[1][1] != 0);
    endfunction

    function void report();
      string nm[3] = '{"INT4 ", "INT8 ", "INT16"};
      $display("---------------------------------------------------------------");
      $display(" PRECISION-GATING TOGGLE REPORT (PE operand + accumulator regs)");
      $display("   mode  | busy cyc |  opd low tog | opd UPPER tog |  acc low tog | acc UPPER tog");
      for (int m = 0; m < 3; m++)
        $display("   %s | %8d | %12d | %13d | %12d | %13d %s", nm[m], busy_cyc[m],
                 opd_tog[m][0], opd_tog[m][1], acc_tog[m][0], acc_tog[m][1],
                 (m < 2 && (opd_tog[m][1] || acc_tog[m][1])) ? "<-- GATING FAIL" :
                 (m < 2 && busy_cyc[m] > 0) ? "(gated OK)" : "");
      $display("---------------------------------------------------------------");
    endfunction
  endclass

  //==============================================================================
  // tu_sched_chk.sv : cycle-by-cycle systolic schedule checker (G7, t = i+j+k)
  //   Uses the white-box PE operand probes (pe_a / pe_b).
  //   Array "step" s = 0,1,2,... : one step per accepted beat (input_valid while
  //   busy) and, once all N beats were given, one step per cycle until 3N-1 steps.
  //   After step s the PE at row i / column j must hold
  //        pe_a[i][j] = A[i][k]   and   pe_b[i][j] = B[k][j]   with k = s - i - j
  //   (+SCHED_STRICT : also zero when k is outside 0..N-1), i.e. A[i][k] and B[k][j] meet in PE(i,j)
  //   exactly at t = i + j + k.  Only the active low 4/8/16 bits are compared so
  //   the check is independent of how the DUT stores the gated upper bits.
  //   +SCHED_OFS=<n> : extra pipeline steps of the real RTL before PE(0,0) (default 0)
  //   +NO_SCHED      : disable the checker
  //==============================================================================
  class tu_sched_chk;

    virtual tu_if.MON vif;
    int  errors_cnt, steps_chk, ops_chk;
    int  ofs = 0;
    bit  en  = 1;
    bit  strict = 0;

    function new(virtual tu_if.MON vif);
      this.vif = vif;
      void'($value$plusargs("SCHED_OFS=%d", ofs));
      if ($test$plusargs("NO_SCHED")) en = 0;
      if ($test$plusargs("SCHED_STRICT")) strict = 1;
    endfunction

    task run();
      bit                 active, pend;
      int                 s, beats, w;
      mode_e              m;
      logic [DW-1:0]      A[N][N], B[N][N];
      if (!en) return;
      forever begin
        logic             rst, st, iv, bz, dn;
        logic [1:0]       mp;
        logic [N*DW-1:0]  ac, br;
        logic [N*N*DW-1:0] pa, pb;

        @(vif.mon_cb);
        rst = vif.mon_cb.reset;  st = vif.mon_cb.start;  iv = vif.mon_cb.input_valid;
        bz  = vif.mon_cb.busy;   dn = vif.mon_cb.done;
        mp  = {vif.mon_cb.mode_1, vif.mon_cb.mode_0};
        ac  = vif.mon_cb.a_col;  br = vif.mon_cb.b_row;
        pa  = vif.mon_cb.pe_a;   pb = vif.mon_cb.pe_b;

        if (rst) begin active = 0; pend = 0; continue; end

        // ---- result of the step taken at the previous edge ----
        if (active && pend) begin
          check_step(s - ofs, m, w, A, B, pa, pb);
          pend = 0;
        end

        if (st && !bz) begin
          active = (mode_e'(mp) != RSVD);
          m      = mode_e'(mp);
          w      = tu_txn::width(m);
          s      = -1;
          beats  = 0;
          if (active) ops_chk++;
          continue;
        end

        if (!active) continue;
        if (dn || !bz) begin active = 0; continue; end

        // ---- does the array advance at this edge? ----
        if (iv && beats < N) begin
          for (int i = 0; i < N; i++) begin
            A[i][beats] = ac[i*DW +: DW];
            B[beats][i] = br[i*DW +: DW];
          end
          beats++;
          s++;
          pend = 1;
        end else if (beats == N && s < 3*N - 2 + ofs) begin
          s++;
          pend = 1;
        end
      end
    endtask

    function void check_step(int s, mode_e m, int w, logic [DW-1:0] A[N][N],
                             logic [DW-1:0] B[N][N], logic [N*N*DW-1:0] pa,
                             logic [N*N*DW-1:0] pb);
      logic [DW-1:0] msk = (w >= DW) ? '1 : DW'((1 << w) - 1);
      int bad = 0;
      if (s < 0) return;
      steps_chk++;
      for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) begin
          int            k  = s - i - j;
          logic [DW-1:0] ea = (k >= 0 && k < N) ? A[i][k] : '0;
          logic [DW-1:0] eb = (k >= 0 && k < N) ? B[k][j] : '0;
          logic [DW-1:0] ga = pa[(i*N+j)*DW +: DW];
          logic [DW-1:0] gb = pb[(i*N+j)*DW +: DW];
          if (!strict && (k < 0 || k >= N)) continue;
          if ((ga & msk) !== (ea & msk) || (gb & msk) !== (eb & msk)) begin
            if (errors_cnt + bad < 10)
              $error("[SCHED] %s step %0d PE(%0d,%0d): a=0x%h b=0x%h expected A[%0d][%0d]=0x%h B[%0d][%0d]=0x%h @%0t",
                     m.name(), s, i, j, ga & msk, gb & msk, i, k, ea & msk, k, j, eb & msk, $time);
            bad++;
          end
        end
      if (bad) errors_cnt++;
    endfunction

    function int errors();
      return errors_cnt;
    endfunction

    function void report();
      $display(" SCHEDULE CHECK (t=i+j+k) : %s  ops=%0d steps=%0d failing steps=%0d",
               en ? "ON" : "OFF", ops_chk, steps_chk, errors_cnt);
    endfunction
  endclass

  //==============================================================================
  // tu_env.sv : environment - builds and connects all TB components
  //
  //   generator --gen2drv--> driver --(pins)--> DUT --(pins)--> monitor
  //                            |                                   |
  //                            +------drv2sb--> scoreboard <--mon2sb+
  //                                                 |
  //                                             coverage        toggle monitor
  //                                                             schedule checker
  //==============================================================================
  class tu_env;

    mailbox #(tu_txn) gen2drv, drv2sb, mon2sb;
    event             drv_done;

    tu_coverage       cov;
    tu_generator      gen;
    tu_driver         drv;
    tu_monitor        mon;
    tu_scoreboard     sb;
    tu_toggle_mon     tog;
    tu_sched_chk      sch;

    virtual tu_if     vif;

    function new(virtual tu_if vif, int lat_base);
      this.vif = vif;
      gen2drv  = new();
      drv2sb   = new();
      mon2sb   = new();
      cov      = new();
      gen      = new(gen2drv, drv_done);
      drv      = new(vif.DRV, gen2drv, drv2sb, drv_done);
      mon      = new(vif.MON, mon2sb, cov);
      sb       = new(drv2sb, mon2sb, cov, lat_base);
      tog      = new(vif.MON);
      sch      = new(vif.MON);
    endfunction

    task start_bg();
      fork
        drv.run();
        mon.run();
        sb.run();
        tog.run();
        sch.run();
      join_none
    endtask

    // wait until every expected item has been checked
    task drain();
      int guard = 0;
      while ((drv2sb.num() != 0 || mon2sb.num() != 0) && guard < 1000) begin
        @(vif.mon_cb);
        guard++;
      end
      repeat (5) @(vif.mon_cb);
    endtask

    function int errors();
      return sb.errors() + vif.sva_errors + drv.timeouts + mon.spurious_done +
             mon.lost_busy + tog.errors() + sch.errors() + drv2sb.num() + mon2sb.num();
    endfunction

    function void report();
      sb.report();
      tog.report();
      sch.report();
      cov.report();
      $display(" SVA errors=%0d  driver timeouts=%0d  spurious done=%0d  lost busy=%0d  unmatched exp=%0d obs=%0d",
               vif.sva_errors, drv.timeouts, mon.spurious_done, mon.lost_busy, drv2sb.num(), mon2sb.num());
    endfunction
  endclass

  //==============================================================================
  // tu_test.sv : test layer - selects which of T01..T34 to run
  //   +TEST=T05        run a single test
  //   +TEST=all        run T01..T34 back to back (default)
  //   +LAT_BASE=<n>    override expected base latency (default 3N+1)
  //   +SCHED_OFS=<n>   extra RTL pipeline steps for the schedule checker
  //   +NO_SCHED        disable the t=i+j+k schedule checker
  //   +SEED handled by the simulator (e.g. xrun -svseed random)
  //==============================================================================
  class tu_test;

    tu_env        env;
    virtual tu_if vif;
    string        test_name = "all";
    int           lat_base  = LAT_BASE;

    function new(virtual tu_if vif);
      this.vif = vif;
      void'($value$plusargs("TEST=%s", test_name));
      void'($value$plusargs("LAT_BASE=%d", lat_base));
      env = new(vif, lat_base);
    endfunction

    task run();
      $display("===============================================================");
      $display(" TENSOR UNIT 16x16 TB : TEST=%s  LAT_BASE=%0d", test_name, lat_base);
      $display("===============================================================");

      env.start_bg();
      env.gen.tid = 0;
      env.gen.rst(4);                 // initial reset
      vif.sva_en = 1;

      if (test_name == "all" || test_name == "ALL") begin
        for (int t = 1; t <= NUM_TESTS; t++) begin
          string n = $sformatf("T%02d", t);
          $display("%t ---------------- running %s ----------------", $time, n);
          env.gen.run(n);
        end
      end else
        env.gen.run(test_name);

      env.drain();
      vif.sva_en = 0;

      $display("===============================================================");
      env.report();
      if (env.errors() == 0 && env.sb.n_items > 0)
        $display(" ***  TEST %s PASSED  ***", test_name);
      else
        $display(" ***  TEST %s FAILED  (%0d error(s))  ***", test_name, env.errors());
      $display("===============================================================");
    endtask
  endclass


endpackage

//==============================================================================
// tb_top.sv : top module - clock, interface, DUT, white-box probes, test
//==============================================================================

module tb_top;
  import tu_pkg::*;

  logic clk = 0;
  always #5 clk = ~clk;                         // 100 MHz

  tu_if #(.N(N), .DW(DW), .AW(AW)) vif (clk);

  // ---------------- DUT ----------------
  tensor_unit_16x16 #(.N(N), .DW(DW), .AW(AW)) dut (
    .clk         (clk),
    .reset       (vif.reset),
    .start       (vif.start),
    .mode_1      (vif.mode_1),
    .mode_0      (vif.mode_0),
    .input_valid (vif.input_valid),
    .a_col       (vif.a_col),
    .b_row       (vif.b_row),
    .busy        (vif.busy),
    .done        (vif.done),
    .c_out       (vif.c_out)
  );

  // ---------------- white-box probes (rename to match your RTL) ----------------
  assign vif.eff_mode = dut.eff_mode;
  assign vif.pe_a     = dut.pe_a_flat;
  assign vif.pe_b     = dut.pe_b_flat;
  assign vif.pe_acc   = dut.pe_acc_flat;

  // ---------------- default pin values ----------------
  initial begin
    vif.reset       = 1;
    vif.start       = 0;
    vif.mode_1      = 0;
    vif.mode_0      = 0;
    vif.input_valid = 0;
    vif.a_col       = '0;
    vif.b_row       = '0;
  end

  // ---------------- activity capture for precision-gating evidence ----------
  initial begin
    if ($test$plusargs("VCD")) begin
      $dumpfile("tu_activity.vcd");
      $dumpvars(0, tb_top.dut);
    end
  end

  // ---------------- run the test ----------------
  tu_test t;
  initial begin
    t = new(vif);
    t.run();
    $finish;
  end

  initial begin
    #50ms;
    $error("[TOP] global watchdog expired");
    t.env.report();
    $display(" ***  TEST %s FAILED  (watchdog)  ***", t.test_name);
    $finish;
  end
endmodule
