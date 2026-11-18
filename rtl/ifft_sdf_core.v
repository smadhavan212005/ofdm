// ----------------------------------------------------------------------------
// ifft_sdf_core : N-point pipelined radix-2 SDF IFFT, streaming, valid/ready
//
//   in  : natural-order samples, 1 complex sample per accepted beat, N beats = 1 symbol
//   out : BIT-REVERSED order (position j carries time sample bitrev(j)), N beats / symbol
//         (the downstream cp_insert_reorder stage restores natural order)
//
//   out[n] = 1/N * sum_k X[k] exp(+j 2 pi k n / N)      when every stage has SC=1
//
// Throughput : 1 sample/clock while the sink keeps up.
// Latency    : K = N + 2*log2(N) - 2 ticks from the first input to the first output.
//
// Stalling / flushing
//   * The whole datapath advances on a single `en` tick, so back-pressure is a
//     simple global stall.
//   * An SDF pipeline holds the tail of a symbol in its delay lines until further
//     ticks push it out. If the input goes idle on a symbol boundary for more than
//     FLUSH_IDLE cycles, the core injects K zero ticks (a "flush") and then
//     re-aligns its slot counter. Continuous streams never flush. s_ready is low
//     while flushing.
//   * Input gaps in the middle of a symbol just freeze the pipeline.
// ----------------------------------------------------------------------------
`timescale 1ns/1ps
module ifft_sdf_core #(
    parameter N          = 64,
    parameter DW         = 16,
    parameter TW         = 16,
    parameter SCALE_MASK = -1,        // bit s = 1: stage s scales by 1/2 (all ones = exact 1/N)
    parameter FLUSH_IDLE = 8,         // idle cycles (at symbol boundary) before auto-flush
    parameter TW_FILE    = "twiddle.mem"
)(
    input  wire                 clk,
    input  wire                 rst_n,
    // input stream
    input  wire                 s_valid,
    output wire                 s_ready,
    input  wire signed [DW-1:0] s_re,
    input  wire signed [DW-1:0] s_im,
    // output stream (bit-reversed order)
    output reg                  m_valid,
    input  wire                 m_ready,
    output wire signed [DW-1:0] m_re,
    output wire signed [DW-1:0] m_im
);
    localparam LOGN = $clog2(N);
    localparam S    = LOGN;
    localparam K    = N + 2*S - 2;            // input->output latency in ticks (= flush length)
    localparam KW   = $clog2(K + 1);
    localparam IW   = $clog2(FLUSH_IDLE + 2);

    // ------------------------------------------------------------------ controller
    reg [LOGN-1:0] cnt;                        // slot index of the sample entering stage 0
    reg            dirty;                      // pipeline holds un-emitted samples
    reg            flushing;
    reg [KW-1:0]   fcnt;
    reg [IW-1:0]   idle;
    reg [K-1:0]    tag;                        // "real sample" tag, travels with latency K-1

    wire can_adv    = ~m_valid | m_ready;
    wire run_tick   = ~flushing & s_valid & can_adv;
    wire flush_tick =  flushing & can_adv;
    wire en         = run_tick | flush_tick;
    assign s_ready  = ~flushing & can_adv;

    wire signed [DW-1:0] din_re = flushing ? {DW{1'b0}} : s_re;
    wire signed [DW-1:0] din_im = flushing ? {DW{1'b0}} : s_im;

    initial begin cnt = 0; dirty = 0; flushing = 0; fcnt = 0; idle = 0; tag = 0; m_valid = 0; end

    always @(posedge clk) begin
        if (!rst_n) begin
            cnt <= 0; dirty <= 0; flushing <= 0; fcnt <= 0; idle <= 0; tag <= 0; m_valid <= 0;
        end else begin
            // ---- tick: advance slot counter, tag pipe and output-valid flag ----
            if (en) begin
                cnt     <= cnt + 1'b1;
                tag     <= {tag[K-2:0], run_tick};
                m_valid <= tag[K-1];
            end else if (m_ready) begin
                m_valid <= 1'b0;               // consumed without a tick
            end

            // ---- flush trigger: idle at a symbol boundary with data still in flight ----
            if (run_tick) begin
                dirty <= 1'b1;
                idle  <= 0;
            end else if (!flushing) begin
                if (dirty && cnt == 0 && !s_valid) begin
                    if (idle >= FLUSH_IDLE) begin
                        flushing <= 1'b1;
                        fcnt     <= 0;
                        idle     <= 0;
                    end else begin
                        idle <= idle + 1'b1;
                    end
                end else begin
                    idle <= 0;
                end
            end

            // ---- flush progress ----
            if (flush_tick) begin
                fcnt <= fcnt + 1'b1;
                if (fcnt == K - 1) begin
                    flushing <= 1'b0;
                    dirty    <= 1'b0;
                    cnt      <= 0;             // re-align slot counter for the next symbol
                end
            end
        end
    end

    // ------------------------------------------------------------------ stage chain
    wire signed [DW-1:0] c_re  [0:S];
    wire signed [DW-1:0] c_im  [0:S];
    wire [LOGN-1:0]      c_cnt [0:S];
    assign c_re[0]  = din_re;
    assign c_im[0]  = din_im;
    assign c_cnt[0] = cnt;

    genvar s;
    generate
        for (s = 0; s < S; s = s + 1) begin : g_stage
            ifft_sdf_stage #(
                .N      (N),
                .LOGN   (LOGN),
                .DW     (DW),
                .TW     (TW),
                .D      (N >> (s + 1)),
                .OFF    (N - (N >> s)),
                .SC     ((SCALE_MASK >> s) & 1),
                .TW_FILE(TW_FILE)
            ) u_stage (
                .clk    (clk),
                .en     (en),
                .in_re  (c_re[s]),
                .in_im  (c_im[s]),
                .in_cnt (c_cnt[s]),
                .out_re (c_re[s+1]),
                .out_im (c_im[s+1]),
                .out_cnt(c_cnt[s+1])
            );
        end
    endgenerate

    assign m_re = c_re[S];
    assign m_im = c_im[S];
endmodule
