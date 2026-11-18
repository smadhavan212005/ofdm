function [wc, ws] = twiddle_table(N, TW)
% TWIDDLE_TABLE  Integer twiddles W_N^k = exp(+j*2*pi*k/N), k = 0..N/2-1, in Q2.(TW-2).
%   wc = round(cos * 2^(TW-2)),  ws = round(sin * 2^(TW-2))   (IFFT => positive exponent)
    k  = 0:(N/2 - 1);
    wc = round(cos(2*pi*k/N) * 2^(TW-2));
    ws = round(sin(2*pi*k/N) * 2^(TW-2));
end
