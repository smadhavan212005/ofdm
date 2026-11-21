// ----------------------------------------------------------------------------
// systolic_mm : parameterized INT8 output-stationary systolic matrix-multiply unit
//
//   Computes one tile   C (M x N) = A (M x K) * B (K x N)
//     M <= ROWS, N <= COLS (run-time tile size), K arbitrary (streamed).
//   Operands signed DW bits (INT8), accumulators signed AW bits (wrap on overflow).
//
//   Dataflow (output stationary): PE(r,c) owns C[r][c]. A flows left->right along row r,
//   B flows top->bottom along column c, one hop per clock. Inputs are skewed inside the
//   block (row r delayed r clocks, column c delayed c clocks), so the caller simply
//   presents one "k-slice" per clock:
//        a_in  = { A[ROWS-1][k] ... A[1][k], A[0][k] }     (A column k)
//        b_in  = { B[k][COLS-1] ... B[k][1], B[k][0] }     (B row k)
//   Row r / column c of a PE array outside the tile (r>=M or c>=N) are ignored.
//
//   Protocol
//     1. pulse `start` (with m_len, n_len, both 1..ROWS/COLS) while idle or done
//        -> accumulators clear, block goes busy
//     2. stream K beats with in_valid=1 (bubbles allowed); mark the last with in_last=1
//     3. wait for `done` (pulse) / c_valid (level): c_out holds C row-major
//        c_out[(r*COLS+c)*AW +: AW]  (zero outside the M x N tile), stable until next start
//
//   Clock gating (CLK_GATE=1)
//     * array-level : the input-skew registers are clocked only while busy
//     * PE-level    : every PE has its own ICG, enabled only while busy AND inside the
//                     M x N tile, so unused rows/columns and idle periods cost no clock
//                     toggles. CLK_GATE=0 uses plain clock enables (identical function).
//     dbg_pe_clk / dbg_arr_clk expose the gated clocks so a testbench can count edges
//     (clock-power proxy). Leave them unconnected in a real design.
//
//   Latency from the last input beat to done: about M + N - 1 clocks.
// ----------------------------------------------------------------------------
`timescale 1ns/1ps
module systolic_mm #(
    parameter ROWS     = 4,
    parameter COLS     = 4,
    parameter DW       = 8,
    parameter AW       = 32,
    parameter CLK_GATE = 1,
    localparam MW      = $clog2(ROWS + 1),
    localparam NW      = $clog2(COLS + 1)
)(
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      test_en,          // forces all clock gates on (DFT)
    // control
    input  wire                      start,
    input  wire [MW-1:0]             m_len,
    input  wire [NW-1:0]             n_len,
    // operand stream
    input  wire                      in_valid,
    input  wire                      in_last,
    output wire                      in_ready,
    input  wire [ROWS*DW-1:0]        a_in,
    input  wire [COLS*DW-1:0]        b_in,
    // status / result
    output wire                      busy,
    output wire                      done,
    output wire                      c_valid,
    output wire [ROWS*COLS*AW-1:0]   c_out,
    // clock-activity probes
    output wire [ROWS*COLS-1:0]      dbg_pe_clk,
    output wire                      dbg_arr_clk
);
    localparam DCW = $clog2(ROWS + COLS);

    // ------------------------------------------------------------------ control FSM
    localparam [1:0] IDLE = 2'd0, RUN = 2'd1, DRAIN = 2'd2, DONE = 2'd3;
    reg [1:0]     state;
    reg [MW-1:0]  m_q;
    reg [NW-1:0]  n_q;
    reg [DCW-1:0] dcnt;
    reg           done_r;
    initial begin state = IDLE; m_q = 1; n_q = 1; dcnt = 0; done_r = 0; end

    function [MW-1:0] clampm; input [MW-1:0] v;
        clampm = (v == 0) ? 1 : (v > ROWS) ? ROWS : v;
    endfunction
    function [NW-1:0] clampn; input [NW-1:0] v;
        clampn = (v == 0) ? 1 : (v > COLS) ? COLS : v;
    endfunction

    wire idle_or_done = (state == IDLE) || (state == DONE);
    wire start_ok     = start && idle_or_done;
    wire busy_w       = (state == RUN) || (state == DRAIN);
    wire [MW-1:0] m_sel = start_ok ? clampm(m_len) : m_q;   // tile size in force this cycle
    wire [NW-1:0] n_sel = start_ok ? clampn(n_len) : n_q;

    always @(posedge clk) begin
        done_r <= 1'b0;
        if (!rst_n) begin
            state <= IDLE; dcnt <= 0;
        end else begin
            case (state)
                IDLE, DONE: if (start) begin
                    state <= RUN;
                    m_q   <= clampm(m_len);
                    n_q   <= clampn(n_len);
                end
                RUN: if (in_valid && in_last) begin
                    state <= DRAIN;
                    dcnt  <= m_q + n_q - 2;              // clocks for the last k-slice to reach PE(M-1,N-1)
                end
                DRAIN: if (dcnt == 0) begin
                    state  <= DONE;
                    done_r <= 1'b1;
                end else begin
                    dcnt <= dcnt - 1'b1;
                end
                default: state <= IDLE;
            endcase
        end
    end

    assign in_ready = (state == RUN);
    assign busy     = busy_w;
    assign done     = done_r;
    assign c_valid  = (state == DONE);
    wire   v0       = in_valid && in_ready;

    // ------------------------------------------------------------------ array-level clock
    wire arr_en = busy_w | start_ok;
    wire arr_clk;
    wire arr_ce = (CLK_GATE != 0) ? 1'b1 : arr_en;
    generate
        if (CLK_GATE != 0) begin : g_arr_icg
            clk_gate u_icg (.clk(clk), .en(arr_en), .test_en(test_en), .gclk(arr_clk));
        end else begin : g_arr_plain
            assign arr_clk = clk;
        end
    endgenerate
    assign dbg_arr_clk = arr_clk;

    // ------------------------------------------------------------------ input skew
    wire signed [DW-1:0] a_row [0:ROWS-1];
    wire                 v_row [0:ROWS-1];
    wire signed [DW-1:0] b_col [0:COLS-1];

    genvar r, c;
    generate
        for (r = 0; r < ROWS; r = r + 1) begin : g_askew
            if (r == 0) begin : g_none
                assign a_row[0] = a_in[DW-1:0];
                assign v_row[0] = v0;
            end else begin : g_dly                       // r delay registers for A and its valid token
                reg signed [DW-1:0] sa [0:r-1];
                reg                 sv [0:r-1];
                integer i;
                initial for (i = 0; i < r; i = i + 1) begin sa[i] = 0; sv[i] = 0; end
                always @(posedge arr_clk) if (arr_ce) begin
                    sa[0] <= a_in[r*DW +: DW];
                    sv[0] <= start_ok ? 1'b0 : v0;       // start flushes stale valid tokens
                    for (i = 1; i < r; i = i + 1) begin
                        sa[i] <= sa[i-1];
                        sv[i] <= start_ok ? 1'b0 : sv[i-1];
                    end
                end
                assign a_row[r] = sa[r-1];
                assign v_row[r] = sv[r-1];
            end
        end
        for (c = 0; c < COLS; c = c + 1) begin : g_bskew
            if (c == 0) begin : g_none
                assign b_col[0] = b_in[DW-1:0];
            end else begin : g_dly                       // c delay registers for B
                reg signed [DW-1:0] sb [0:c-1];
                integer j;
                initial for (j = 0; j < c; j = j + 1) sb[j] = 0;
                always @(posedge arr_clk) if (arr_ce) begin
                    sb[0] <= b_in[c*DW +: DW];
                    for (j = 1; j < c; j = j + 1) sb[j] <= sb[j-1];
                end
                assign b_col[c] = sb[c-1];
            end
        end
    endgenerate

    // ------------------------------------------------------------------ PE grid
    wire signed [DW-1:0] a_w   [0:ROWS*(COLS+1)-1];      // A into PE(r,c) at r*(COLS+1)+c
    wire                 v_w   [0:ROWS*(COLS+1)-1];
    wire signed [DW-1:0] b_w   [0:(ROWS+1)*COLS-1];      // B into PE(r,c) at r*COLS+c
    wire signed [AW-1:0] acc_w [0:ROWS*COLS-1];

    generate
        for (r = 0; r < ROWS; r = r + 1) begin : g_r
            assign a_w[r*(COLS+1)] = a_row[r];
            assign v_w[r*(COLS+1)] = v_row[r];
        end
        for (c = 0; c < COLS; c = c + 1) begin : g_c
            assign b_w[c] = b_col[c];
        end

        for (r = 0; r < ROWS; r = r + 1) begin : g_row
            for (c = 0; c < COLS; c = c + 1) begin : g_col
                wire in_tile = (r < m_sel) && (c < n_sel);
                wire pe_en   = (busy_w | start_ok) & in_tile;
                wire pclk;
                if (CLK_GATE != 0) begin : g_icg
                    clk_gate u_icg (.clk(clk), .en(pe_en), .test_en(test_en), .gclk(pclk));
                end else begin : g_plain
                    assign pclk = clk;
                end
                assign dbg_pe_clk[r*COLS + c] = pclk;

                mac_pe #(.DW(DW), .AW(AW)) u_pe (
                    .clk  (pclk),
                    .ce   ((CLK_GATE != 0) ? 1'b1 : pe_en),
                    .clr  (start_ok),
                    .v_in (v_w[r*(COLS+1) + c]),
                    .a_in (a_w[r*(COLS+1) + c]),
                    .b_in (b_w[r*COLS + c]),
                    .v_out(v_w[r*(COLS+1) + c + 1]),
                    .a_out(a_w[r*(COLS+1) + c + 1]),
                    .b_out(b_w[(r+1)*COLS + c]),
                    .acc  (acc_w[r*COLS + c])
                );

                assign c_out[(r*COLS + c)*AW +: AW] =
                       ((r < m_q) && (c < n_q)) ? acc_w[r*COLS + c] : {AW{1'b0}};
            end
        end
    endgenerate
endmodule
