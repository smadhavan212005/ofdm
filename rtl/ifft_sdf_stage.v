// ----------------------------------------------------------------------------
// ifft_sdf_stage : one radix-2 DIF single-path-delay-feedback (SDF) stage
//
//   * Feedback FIFO of depth D holds either raw inputs (first half of a 2D block)
//     or butterfly differences (second half).
//   * Control follows the data: every stage receives `in_cnt`, the time-slot index
//     of its input sample, and derives its phase from (in_cnt - OFF) where OFF is
//     the summed delay of the preceding stages.
//   * Pipelining: 2 register stages per butterfly stage
//         R1: FIFO/butterfly mux + twiddle fetch   (feedback loop = one adder only)
//         R2: complex multiply (x twiddle) + round + saturate
//     The twiddle multiplier sits OUTSIDE the feedback loop, so it is freely pipelined.
//   * Scaling: SC=1 divides the stage output by 2 (round half up). Output always saturates.
//   * Twiddle W = exp(+j*2*pi*k/N) (IFFT), Q2.(TW-2), loaded from TW_FILE.
//     D==2 stage needs only {1, +j}  -> no multiplier;  D==1 stage needs only 1.
//
// All registers advance only when `en` is high (global stall = AXI back-pressure).
// ----------------------------------------------------------------------------
`timescale 1ns/1ps
module ifft_sdf_stage #(
    parameter N       = 64,
    parameter LOGN    = 6,          // log2(N)
    parameter DW      = 16,
    parameter TW      = 16,
    parameter D       = 32,         // feedback depth of this stage = N / 2^(stage+1)
    parameter OFF     = 0,          // sum of D of all previous stages
    parameter SC      = 1,          // 1: scale by 1/2
    parameter TW_FILE = "twiddle.mem"
)(
    input  wire                  clk,
    input  wire                  en,
    input  wire signed [DW-1:0]  in_re,
    input  wire signed [DW-1:0]  in_im,
    input  wire [LOGN-1:0]       in_cnt,
    output reg  signed [DW-1:0]  out_re,
    output reg  signed [DW-1:0]  out_im,
    output reg  [LOGN-1:0]       out_cnt
);
    localparam LOGD = (D > 1) ? $clog2(D) : 0;
    localparam SH   = TW - 2 + SC;                       // total right shift after multiply
    localparam signed [DW-1:0] SMAX = {1'b0, {(DW-1){1'b1}}};
    localparam signed [DW-1:0] SMIN = {1'b1, {(DW-1){1'b0}}};

    // ---------------- control derived from the carried slot counter ----------------
    wire [LOGN-1:0] p     = in_cnt - OFF;                // stream position seen by this stage
    wire            phase = p[LOGD];                     // 0: fill FIFO / emit diffs, 1: butterfly
    wire [LOGN-1:0] nidx  = p & (D - 1);                 // pair index within the half block

    // ---------------- feedback FIFO (shift register, depth D, width DW+1) ----------
    reg signed [DW:0] f_re [0:D-1];
    reg signed [DW:0] f_im [0:D-1];
    wire signed [DW:0] fo_re = f_re[D-1];
    wire signed [DW:0] fo_im = f_im[D-1];
    wire signed [DW:0] x_re  = in_re;                    // sign-extend
    wire signed [DW:0] x_im  = in_im;

    wire signed [DW:0] sum_re = fo_re + x_re;
    wire signed [DW:0] sum_im = fo_im + x_im;
    wire signed [DW:0] dif_re = fo_re - x_re;
    wire signed [DW:0] dif_im = fo_im - x_im;

    wire signed [DW:0] fin_re = phase ? dif_re : x_re;   // what goes into the FIFO
    wire signed [DW:0] fin_im = phase ? dif_im : x_im;
    wire signed [DW:0] z_re   = phase ? sum_re : fo_re;  // what goes to the multiplier
    wire signed [DW:0] z_im   = phase ? sum_im : fo_im;

    integer i;
    initial for (i = 0; i < D; i = i + 1) begin f_re[i] = 0; f_im[i] = 0; end
    always @(posedge clk) if (en) begin
        for (i = D - 1; i > 0; i = i - 1) begin
            f_re[i] <= f_re[i-1];
            f_im[i] <= f_im[i-1];
        end
        f_re[0] <= fin_re;
        f_im[0] <= fin_im;
    end

    // ---------------- R1 registers ----------------
    reg signed [DW:0]  z_re_r, z_im_r;
    reg [LOGN-1:0]     cnt_r;
    initial begin z_re_r = 0; z_im_r = 0; cnt_r = 0; out_re = 0; out_im = 0; out_cnt = 0; end

    always @(posedge clk) if (en) begin
        z_re_r <= z_re;
        z_im_r <= z_im;
        cnt_r  <= in_cnt;
    end

    generate
    if (D > 2) begin : g_mult
        // ----- generic stage: full complex multiplier -----
        reg [2*TW-1:0] rom [0:N/2-1];
        initial $readmemh(TW_FILE, rom);
        localparam SHIFT_K = LOGN - 1 - LOGD;
        localparam signed [TW-1:0] ONE = 1 << (TW - 2);
        wire [LOGN-1:0]        kidx = nidx << SHIFT_K;
        wire [2*TW-1:0]        rw   = rom[kidx[LOGN-2:0]];   // kidx < N/2 always
        wire signed [TW-1:0]   w_c  = phase ? ONE : rw[2*TW-1:TW];
        wire signed [TW-1:0]   w_s  = phase ? {TW{1'b0}} : rw[TW-1:0];

        reg signed [TW-1:0] wc_r, ws_r;
        initial begin wc_r = 0; ws_r = 0; end
        always @(posedge clk) if (en) begin
            wc_r <= w_c;
            ws_r <= w_s;
        end

        localparam MW = DW + TW + 2;
        localparam signed [MW-1:0] RND = 1 << (SH - 1);
        wire signed [MW-1:0] pr = z_re_r * wc_r - z_im_r * ws_r;
        wire signed [MW-1:0] pi = z_re_r * ws_r + z_im_r * wc_r;
        wire signed [MW-1:0] rr = (pr + RND) >>> SH;
        wire signed [MW-1:0] ri = (pi + RND) >>> SH;

        always @(posedge clk) if (en) begin
            out_re  <= (rr > SMAX) ? SMAX : (rr < SMIN) ? SMIN : rr[DW-1:0];
            out_im  <= (ri > SMAX) ? SMAX : (ri < SMIN) ? SMIN : ri[DW-1:0];
            out_cnt <= cnt_r;
        end
    end else begin : g_triv
        // ----- D==2: twiddle in {1,+j};  D==1: twiddle == 1 -----
        reg jflag_r;                                     // multiply by +j
        initial jflag_r = 1'b0;
        if (D == 2) begin : g_j
            always @(posedge clk) if (en) jflag_r <= ~phase & nidx[0];
        end

        localparam MW = DW + 2;
        wire signed [MW-1:0] zr = z_re_r;
        wire signed [MW-1:0] zi = z_im_r;
        wire signed [MW-1:0] yr = jflag_r ? -zi : zr;     // j*(zr + j zi) = -zi + j zr
        wire signed [MW-1:0] yi = jflag_r ?  zr : zi;
        wire signed [MW-1:0] rr = (yr + SC) >>> SC;       // round half up, matches (v + 2^(SH-1)) >> SH
        wire signed [MW-1:0] ri = (yi + SC) >>> SC;

        always @(posedge clk) if (en) begin
            out_re  <= (rr > SMAX) ? SMAX : (rr < SMIN) ? SMIN : rr[DW-1:0];
            out_im  <= (ri > SMAX) ? SMAX : (ri < SMIN) ? SMIN : ri[DW-1:0];
            out_cnt <= cnt_r;
        end
    end
    endgenerate
endmodule
