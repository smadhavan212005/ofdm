function s = qam_map(bits, BPS, DW, PEAK)
% QAM_MAP  Gray-coded square QAM mapper, bit-accurate with rtl/qam_mapper.v
%
%   bits : integer array, each element is a BPS-bit symbol (0 .. 2^BPS-1)
%   BPS  : bits per symbol: 2 (QPSK), 4 (16-QAM), 6 (64-QAM)
%   PEAK : integer amplitude of the OUTERMOST constellation level per axis
%
% Bit layout of one symbol:  [ I bits (upper BPS/2) | Q bits (lower BPS/2) ]
% Per axis (m = BPS/2 bits): Gray code -> binary -> level = 2*bin - (2^m - 1)
%   (802.11 style:  m=2:  00->-3  01->-1  11->+1  10->+3)
% Output = level * UNIT, UNIT = floor(PEAK / (2^m - 1))
    m    = BPS/2;
    unit = floor(PEAK / (2^m - 1));
    ib   = floor(bits / 2^m);
    qb   = mod(bits, 2^m);
    s    = axis_level(ib, m) * unit + 1j * (axis_level(qb, m) * unit);
end

function lv = axis_level(g, m)
    b = g;
    for k = 1:m-1
        b = bitxor(b, floor(g / 2^k));
    end
    lv = 2*b - (2^m - 1);
end
