// ----------------------------------------------------------------------------
// ofdm_ifft_tx : [QAM mapper] -> pipelined SDF IFFT -> bit-reverse + cyclic prefix
//                AXI-Stream in, AXI-Stream out
//
//   BPS = 0 : no mapper. s_axis_tdata = {im[DW-1:0], re[DW-1:0]}  (signed, two's complement)
//   BPS = 2/4/6 : mapper enabled. s_axis_tdata[BPS-1:0] = one QAM symbol per subcarrier,
//                 layout [I bits | Q bits], Gray coded.
//
//   One input symbol = N consecutive accepted beats (subcarrier 0 first).
//   s_axis_tlast is not required (symbol boundaries are counted) and is ignored.
//   One output symbol = CP + N beats: cyclic prefix first, tlast on the final beat.
//   m_axis_tdata = {im, re}.
//
//   Overall gain: out = 1/N * IFFT-sum when SCALE_MASK is all ones (== MATLAB ifft()).
// ----------------------------------------------------------------------------
`timescale 1ns/1ps
module ofdm_ifft_tx #(
    parameter N          = 64,
    parameter DW         = 16,
    parameter TW         = 16,
    parameter CP         = 16,
    parameter BPS        = 0,
    parameter QAM_PEAK   = (1 << (DW - 2)),
    parameter SCALE_MASK = -1,
    parameter FLUSH_IDLE = 8,
    parameter TW_FILE    = "twiddle.mem",
    localparam S_W       = (BPS == 0) ? 2*DW : BPS
)(
    input  wire                 clk,
    input  wire                 rst_n,
    // AXI-Stream slave
    input  wire [S_W-1:0]       s_axis_tdata,
    input  wire                 s_axis_tvalid,
    output wire                 s_axis_tready,
    input  wire                 s_axis_tlast,     // unused
    // AXI-Stream master
    output wire [2*DW-1:0]      m_axis_tdata,
    output wire                 m_axis_tvalid,
    input  wire                 m_axis_tready,
    output wire                 m_axis_tlast
);
    wire signed [DW-1:0] in_re, in_im;

    generate
        if (BPS == 0) begin : g_nomap
            assign in_re = s_axis_tdata[DW-1:0];
            assign in_im = s_axis_tdata[2*DW-1:DW];
        end else begin : g_map
            qam_mapper #(.BPS(BPS), .DW(DW), .PEAK(QAM_PEAK)) u_map (
                .bits(s_axis_tdata[BPS-1:0]), .re(in_re), .im(in_im));
        end
    endgenerate

    wire                 c_valid, c_ready;
    wire signed [DW-1:0] c_re, c_im;

    ifft_sdf_core #(
        .N(N), .DW(DW), .TW(TW), .SCALE_MASK(SCALE_MASK),
        .FLUSH_IDLE(FLUSH_IDLE), .TW_FILE(TW_FILE)
    ) u_ifft (
        .clk(clk), .rst_n(rst_n),
        .s_valid(s_axis_tvalid), .s_ready(s_axis_tready), .s_re(in_re), .s_im(in_im),
        .m_valid(c_valid), .m_ready(c_ready), .m_re(c_re), .m_im(c_im)
    );

    cp_insert_reorder #(.N(N), .DW(DW), .CP(CP)) u_cp (
        .clk(clk), .rst_n(rst_n),
        .i_valid(c_valid), .i_ready(c_ready), .i_data({c_im, c_re}),
        .m_axis_tdata(m_axis_tdata), .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready), .m_axis_tlast(m_axis_tlast)
    );
endmodule
