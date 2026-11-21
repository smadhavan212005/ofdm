// Self-checking testbench for systolic_mm (+ clock-gate), driven by model/mm_gen_vectors.m
//
// GAP_MODE 0 : dense streaming, tests separated by 1 idle cycle
//          1 : random bubbles inside the operand stream, random idle gaps (0..39 cycles) between tests
// (unused lanes outside the M x N tile are driven with random garbage in every mode)
//          2 : mode 1 + stray in_valid/in_last while idle, a spurious `start` pulse in the middle of a run,
//              and back-to-back starts straight from DONE (no idle cycle)
// Reports clock-edge counts at the PE clock pins (gated vs the ungated reference) as a clock-power proxy.
// All stimulus is driven on the falling edge and results sampled on the falling edge: no races.
`timescale 1ns/1ps
module tb_systolic_mm;
    parameter ROWS     = 4;
    parameter COLS     = 4;
    parameter DW       = 8;
    parameter AW       = 32;
    parameter CLK_GATE = 1;
    parameter NTESTS   = 64;
    parameter GAP_MODE = 1;

    localparam MW = $clog2(ROWS + 1);
    localparam NW = $clog2(COLS + 1);

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg                  start = 0, in_valid = 0, in_last = 0, test_en = 0;
    reg  [MW-1:0]        m_len = 1;
    reg  [NW-1:0]        n_len = 1;
    reg  [ROWS*DW-1:0]   a_in = 0;
    reg  [COLS*DW-1:0]   b_in = 0;
    wire                 in_ready, busy, done, c_valid;
    wire [ROWS*COLS*AW-1:0] c_out;
    wire [ROWS*COLS-1:0] dbg_pe_clk;
    wire                 dbg_arr_clk;

    systolic_mm #(.ROWS(ROWS), .COLS(COLS), .DW(DW), .AW(AW), .CLK_GATE(CLK_GATE)) dut (
        .clk(clk), .rst_n(rst_n), .test_en(test_en),
        .start(start), .m_len(m_len), .n_len(n_len),
        .in_valid(in_valid), .in_last(in_last), .in_ready(in_ready), .a_in(a_in), .b_in(b_in),
        .busy(busy), .done(done), .c_valid(c_valid), .c_out(c_out),
        .dbg_pe_clk(dbg_pe_clk), .dbg_arr_clk(dbg_arr_clk));

    // ------------------------------------------------ clock-edge counters (power proxy)
    integer clk_edges = 0, arr_edges = 0, pe_edges = 0, done_pulses = 0, busy_cycles = 0;
    always @(posedge clk) begin
        if (rst_n) begin clk_edges = clk_edges + 1; if (busy) busy_cycles = busy_cycles + 1; if (done) done_pulses = done_pulses + 1; end
    end
    always @(posedge dbg_arr_clk) if (rst_n) arr_edges = arr_edges + 1;
    genvar gi;
    generate for (gi = 0; gi < ROWS*COLS; gi = gi + 1) begin : g_cnt
        always @(posedge dbg_pe_clk[gi]) if (rst_n) pe_edges = pe_edges + 1;
    end endgenerate

    // ------------------------------------------------ stimulus / checking
    integer ftest, fexp, fout;
    integer seed = 20260521;
    integer t, k, r, c, M, N, K, nread, val, ev, errors = 0, cells = 0, gap, tests_run = 0;
    reg signed [AW-1:0] got;
    integer got_i;
    reg [ROWS*DW-1:0] av;
    reg [COLS*DW-1:0] bv;
    reg skip_idle;

    initial begin
        ftest = $fopen("tests.txt", "r");
        fexp  = $fopen("expected.txt", "r");
        fout  = $fopen("rtl_c.txt", "w");
        repeat (4) @(negedge clk);
        rst_n = 1;
        repeat (3) @(negedge clk);

        skip_idle = 0;
        for (t = 0; t < NTESTS; t = t + 1) begin
            nread = $fscanf(ftest, "%d %d %d", M, N, K);

            // ---- idle period between tests (clocks should be gated off here)
            if (!skip_idle) begin
                gap = (GAP_MODE == 0) ? 1 : (($random(seed) & 63) % 40);
                repeat (gap) begin
                    if (GAP_MODE == 2) begin                         // stray traffic while idle must be ignored
                        in_valid = $random(seed); in_last = $random(seed);
                        a_in = {ROWS{8'h5A}}; b_in = {COLS{8'hA5}};
                    end
                    @(negedge clk);
                end
                in_valid = 0; in_last = 0;
            end

            // ---- start pulse
            start = 1; m_len = M; n_len = N;
            @(negedge clk);
            start = 0;
            skip_idle = 0;

            // ---- stream K k-slices
            for (k = 0; k < K; k = k + 1) begin
                if (GAP_MODE >= 1) begin
                    gap = ($random(seed) & 3);                       // 0..3 bubbles
                    repeat (gap) begin in_valid = 0; in_last = 0; @(negedge clk); end
                end
                av = 0; bv = 0;
                for (r = 0; r < M; r = r + 1) begin nread = $fscanf(ftest, "%d", val); av[r*DW +: DW] = val; end
                for (c = 0; c < N; c = c + 1) begin nread = $fscanf(ftest, "%d", val); bv[c*DW +: DW] = val; end
                // lanes outside the M x N tile are "don't care": drive random garbage there (adversarial)
                for (r = M; r < ROWS; r = r + 1) av[r*DW +: DW] = $random(seed);
                for (c = N; c < COLS; c = c + 1) bv[c*DW +: DW] = $random(seed);
                a_in = av; b_in = bv; in_valid = 1; in_last = (k == K - 1);
                if (GAP_MODE == 2 && k == 0 && K > 2) begin start = 1; m_len = 1; n_len = 1; end   // spurious start mid-run
                @(negedge clk);
                start = 0;
            end
            in_valid = 0; in_last = 0;
            a_in = {ROWS{8'h77}}; b_in = {COLS{8'h77}};              // garbage after the last beat

            // ---- wait for the result
            ev = 0;
            while (!c_valid) begin @(negedge clk); ev = ev + 1; if (ev > 10000) begin $display("TIMEOUT waiting for c_valid, test %0d", t); $finish; end end

            // ---- compare against expected (row-major M x N), cells outside the tile must be zero
            for (r = 0; r < ROWS; r = r + 1)
                for (c = 0; c < COLS; c = c + 1) begin
                    got = c_out[(r*COLS + c)*AW +: AW];
                    got_i = got;
                    if (r < M && c < N) begin
                        nread = $fscanf(fexp, "%d", val);
                        $fwrite(fout, "%0d ", got_i);
                        cells = cells + 1;
                        if (got_i !== val) begin
                            errors = errors + 1;
                            if (errors <= 10) $display("MISMATCH test %0d (M=%0d N=%0d K=%0d) C[%0d][%0d]: rtl=%0d exp=%0d", t, M, N, K, r, c, got_i, val);
                        end
                    end else if (got_i !== 0) begin
                        errors = errors + 1;
                        if (errors <= 10) $display("NONZERO outside tile, test %0d C[%0d][%0d] = %0d", t, r, c, got_i);
                    end
                end
            $fwrite(fout, "\n");
            tests_run = tests_run + 1;
            if (busy) begin errors = errors + 1; $display("busy high in DONE, test %0d", t); end

            // ---- optional back-to-back restart (no idle cycle in DONE)
            skip_idle = (GAP_MODE == 2) && (($random(seed) & 3) == 0);
            if (!skip_idle) @(negedge clk);
        end

        repeat (20) @(negedge clk);
        $fclose(fout);

        $display("-----------------------------------------------------------");
        $display("ROWS=%0d COLS=%0d DW=%0d AW=%0d CLK_GATE=%0d GAP_MODE=%0d tests=%0d", ROWS, COLS, DW, AW, CLK_GATE, GAP_MODE, tests_run);
        $display("cells checked = %0d, errors = %0d, done pulses = %0d (expected %0d)", cells, errors, done_pulses, tests_run);
        $display("clock activity: total cycles = %0d, busy cycles = %0d (%0d%%)", clk_edges, busy_cycles, (100*busy_cycles)/clk_edges);
        $display("  PE clock edges   : %0d   ungated reference (cycles x PEs) = %0d   -> %0d%% fewer", pe_edges, clk_edges*ROWS*COLS,
                 100 - (100*pe_edges)/(clk_edges*ROWS*COLS));
        $display("  array clock edges: %0d   ungated reference = %0d   -> %0d%% fewer", arr_edges, clk_edges, 100 - (100*arr_edges)/clk_edges);
        if (errors == 0 && done_pulses == tests_run && tests_run == NTESTS) $display("TB PASS");
        else                                                                $display("TB FAIL");
        $finish;
    end

    initial begin #500000000; $display("GLOBAL TIMEOUT"); $display("TB FAIL"); $finish; end
endmodule
