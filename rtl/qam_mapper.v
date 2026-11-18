// ----------------------------------------------------------------------------
// qam_mapper : combinational Gray-coded square QAM mapper (QPSK / 16-QAM / 64-QAM)
//
//   bits = [ I bits (upper BPS/2) | Q bits (lower BPS/2) ]
//   per axis (m = BPS/2): Gray -> binary, level = 2*bin - (2^m - 1)   (802.11 style)
//   output = level * UNIT,  UNIT = floor(PEAK / (2^m - 1)),  PEAK = outermost level
//   Bit-accurate with model/qam_map.m
// ----------------------------------------------------------------------------
`timescale 1ns/1ps
module qam_mapper #(
    parameter BPS  = 4,               // 2, 4 or 6
    parameter DW   = 16,
    parameter PEAK = 16384
)(
    input  wire [BPS-1:0]        bits,
    output wire signed [DW-1:0]  re,
    output wire signed [DW-1:0]  im
);
    localparam M      = BPS / 2;
    localparam LEVELS = 1 << M;
    localparam UNIT   = PEAK / (LEVELS - 1);

    function signed [DW-1:0] axis;
        input [M-1:0] g;
        integer bin, k, lv;
        begin
            bin = g;
            for (k = 1; k < M; k = k + 1) bin = bin ^ (g >> k);   // Gray -> binary
            lv   = 2 * bin - (LEVELS - 1);
            axis = lv * UNIT;
        end
    endfunction

    assign re = axis(bits[BPS-1:M]);
    assign im = axis(bits[M-1:0]);
endmodule
