function gen_vectors(outdir, N, DW, TW, CP, BPS, NSYM, seed, scale_mask, qam_peak)
% GEN_VECTORS  Generate stimulus + golden output for the OFDM IFFT/CP RTL testbench.
%
%   gen_vectors(outdir, N, DW, TW, CP, BPS, NSYM, seed [, scale_mask [, qam_peak]])
%
%   BPS = 0 : RTL has no mapper, input words are {im,re} complex integers
%   BPS = 2/4/6 : input words are BPS-bit QAM symbols (QPSK/16QAM/64QAM)
%   scale_mask : bit s = 1 -> stage s divides by 2 (default all ones => exact 1/N IFFT)
%   qam_peak   : outermost-level amplitude per axis (default 2^(DW-2))
%
% Files written to outdir:
%   twiddle.mem   N/2 lines of {cos,sin} hex (Q2.(TW-2)) for $readmemh in the RTL
%   stim.hex      NSYM*N input words
%   expected.hex  NSYM*(N+CP) golden output words {im,re}  (cyclic prefix included)
%   ideal.txt     floating-point ideal output (re im per line), same length/order
%   cfg.txt       key=value (read by compare_results.m)

    if nargin < 9 || isempty(scale_mask), scale_mask = 2^log2(N) - 1; end
    if nargin < 10 || isempty(qam_peak),  qam_peak = 2^(DW-2); end
    S = log2(N);
    assert(CP >= 0 && CP <= N, 'CP must be in [0, N]');
    if ~exist(outdir, 'dir'), mkdir(outdir); end

    if exist('OCTAVE_VERSION', 'builtin') ~= 0
        rand('twister', seed); %#ok<RAND>
    else
        rng(seed);
    end

    SC = zeros(1, S);
    for s = 0:S-1, SC(s+1) = mod(floor(scale_mask / 2^s), 2); end
    nscaled = sum(SC);

    smax = 2^(DW-1) - 1;  smin = -2^(DW-1);
    stim_words = zeros(NSYM*N, 1);       % uint-valued, packed input words
    expected   = zeros(NSYM*(N+CP), 1);
    ideal      = zeros(NSYM*(N+CP), 1);

    for m = 1:NSYM
        if BPS == 0
            % complex-integer input. Mix of: random, full-scale (stresses saturation), impulse, tone
            switch mod(m-1, 4)
                case 0   % uniform random, quarter scale
                    amp = 2^(DW-2);
                    re = randi([-amp, amp-1], 1, N);  im = randi([-amp, amp-1], 1, N);
                case 1   % full-scale random sign extremes
                    re = smax*(2*randi([0 1], 1, N) - 1) - (rand(1, N) < 0.2);
                    im = smax*(2*randi([0 1], 1, N) - 1) - (rand(1, N) < 0.2);
                    re = max(min(re, smax), smin);  im = max(min(im, smax), smin);
                case 2   % impulse + DC
                    re = zeros(1, N);  im = zeros(1, N);  re(1+mod(m, N)) = smax;  im(1) = smin;
                otherwise % single tone, mid scale
                    k0 = 1 + mod(3*m, N);
                    re = zeros(1, N);  im = zeros(1, N);  re(k0) = 2^(DW-2);  im(k0) = -2^(DW-3);
            end
            X = re + 1j*im;
            words = mod(im, 2^DW) * 2^DW + mod(re, 2^DW);   % {im,re}
        else
            bits  = randi([0, 2^BPS - 1], 1, N);
            X     = qam_map(bits, BPS, DW, qam_peak);
            words = bits;
        end
        stim_words((m-1)*N + (1:N)) = words(:);

        [y, ~] = ifft_sdf_model(X, N, DW, TW, SC);
        yc = [y(N-CP+1:N), y];                              % cyclic prefix = last CP samples
        w  = mod(imag(yc), 2^DW) * 2^DW + mod(real(yc), 2^DW);
        expected((m-1)*(N+CP) + (1:N+CP)) = w(:);

        yi = ifft(X) * N / 2^nscaled;                       % float reference on the same integers
        yic = [yi(N-CP+1:N), yi];
        ideal((m-1)*(N+CP) + (1:N+CP)) = yic(:);
    end

    % ---- twiddle ROM ----
    [wc, ws] = twiddle_table(N, TW);
    fid = fopen(fullfile(outdir, 'twiddle.mem'), 'w');
    for k = 1:N/2
        v = mod(wc(k), 2^TW) * 2^TW + mod(ws(k), 2^TW);
        h = hex64(v);
        fprintf(fid, '%s\n', h(end-ceil(2*TW/4)+1:end));   % exactly the digits a 2*TW-bit word needs
    end
    fclose(fid);

    write_hex(fullfile(outdir, 'stim.hex'),     stim_words);
    write_hex(fullfile(outdir, 'expected.hex'), expected);

    fid = fopen(fullfile(outdir, 'ideal.txt'), 'w');
    for i = 1:numel(ideal)
        fprintf(fid, '%.6f %.6f\n', real(ideal(i)), imag(ideal(i)));
    end
    fclose(fid);

    fid = fopen(fullfile(outdir, 'cfg.txt'), 'w');
    fprintf(fid, 'N=%d\nDW=%d\nTW=%d\nCP=%d\nBPS=%d\nNSYM=%d\nSCALE_MASK=%d\n', N, DW, TW, CP, BPS, NSYM, scale_mask);
    fclose(fid);
    fprintf('gen_vectors: N=%d DW=%d TW=%d CP=%d BPS=%d NSYM=%d scale_mask=%d -> %s\n', N, DW, TW, CP, BPS, NSYM, scale_mask, outdir);
end

function write_hex(fname, v)
    fid = fopen(fname, 'w');
    for i = 1:numel(v), fprintf(fid, '%s\n', hex64(v(i))); end
    fclose(fid);
end

function s = hex64(v)
% exact integer -> 16 hex digits (v < 2^53)
    hi = floor(v / 2^32);  lo = v - hi*2^32;
    s  = sprintf('%08X%08X', hi, lo);
end
