function [y_nat, y_raw] = ifft_sdf_model(x, N, DW, TW, SC)
% IFFT_SDF_MODEL  Bit-accurate model of the radix-2 DIF SDF IFFT core (rtl/ifft_sdf_core.v)
%
%   [y_nat, y_raw] = ifft_sdf_model(x, N, DW, TW, SC)
%
%   x   : length-N complex vector of INTEGERS (re/im are signed DW-bit values)
%   N   : transform size (power of two)
%   DW  : data width (bits) of every stage input/output, per component
%   TW  : twiddle width (bits). Twiddles are Q2.(TW-2): 1.0 == 2^(TW-2)
%   SC  : 1xlog2(N) vector, SC(s+1)=1 -> stage s divides by 2 (default: all ones,
%         which makes the core an exact "1/N" IFFT, i.e. equal to MATLAB ifft())
%
%   y_nat : output in natural order    (this is what the CP stage emits)
%   y_raw : output in bit-reversed order (order the core streams samples out)
%
% Arithmetic per butterfly (identical to RTL):
%   sum = a + b           -> round_shift(sum * ONE)
%   dif = a - b           -> round_shift(dif * W)          W = exp(+j*2*pi*k/N)
%   round_shift(v) = saturate( floor( (v + 2^(sh-1)) / 2^sh ) ),  sh = TW-2+SC(s)
%   (round half up, saturate to signed DW bits)
%
% Works in MATLAB and GNU Octave (no toolboxes).

    S = log2(N);
    if abs(S - round(S)) > 0 || N < 2
        error('N must be a power of two >= 2');
    end
    if nargin < 5 || isempty(SC), SC = ones(1, S); end
    x = x(:).';
    assert(numel(x) == N, 'x must have N elements');
    assert(all(real(x) == round(real(x))) && all(imag(x) == round(imag(x))), 'x must be integer-valued');

    [wc, ws] = twiddle_table(N, TW);          % integer tables, length N/2
    ONE  = 2^(TW-2);
    smax = 2^(DW-1) - 1;
    smin = -2^(DW-1);

    a = x;
    for s = 0:S-1
        D  = N / 2^(s+1);
        sh = TW - 2 + SC(s+1);
        R  = 2^(sh-1);
        for blk = 0:(N/(2*D) - 1)
            for n = 0:D-1
                i = blk*2*D + n + 1;
                j = i + D;
                p = a(i);  q = a(j);
                sm = p + q;
                df = p - q;
                k  = n * (N / (2*D));
                % sum path: W = 1.0 -> (c,s) = (ONE,0)
                a(i) = cmul_rnd(sm, ONE, 0, R, 2^sh, smin, smax);
                % difference path: W = exp(+j*2*pi*k/N)
                a(j) = cmul_rnd(df, wc(k+1), ws(k+1), R, 2^sh, smin, smax);
            end
        end
    end

    y_raw = a;
    idx   = bitrev_idx(N);                     % idx(j+1) = bitrev(j)
    y_nat = zeros(1, N);
    y_nat(idx + 1) = y_raw;                    % stream position j holds time sample bitrev(j)
end

function r = cmul_rnd(z, c, s, R, P, smin, smax)
% (zr + j zi)*(c + j s), then floor((v+R)/P), then saturate. All integer math (exact in doubles).
    zr = real(z);  zi = imag(z);
    pr = zr*c - zi*s;
    pi_ = zr*s + zi*c;
    rr = floor((pr + R) / P);
    ri = floor((pi_ + R) / P);
    rr = min(max(rr, smin), smax);
    ri = min(max(ri, smin), smax);
    r  = rr + 1j*ri;
end
