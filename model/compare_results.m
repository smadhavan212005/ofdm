function ok = compare_results(simdir, tol_lsb)
% COMPARE_RESULTS  Compare RTL output dump against the bit-accurate model and the float ideal.
%
%   ok = compare_results(simdir [, tol_lsb])
%
%   rtl_out.hex  (written by the testbench)  vs  expected.hex (fixed-point golden model)
%   Pass criterion: max |RTL - golden| <= tol_lsb on every I and Q sample (default 0 = bit-exact)
%   Also reports the fixed-point implementation error vs ideal floating-point IFFT (SNR, max LSB).
    if nargin < 2, tol_lsb = 0; end
    cfg = read_cfg(fullfile(simdir, 'cfg.txt'));
    DW = cfg.DW;

    rtl = read_words(fullfile(simdir, 'rtl_out.hex'), DW);
    gold = read_words(fullfile(simdir, 'expected.hex'), DW);
    ide = load(fullfile(simdir, 'ideal.txt'));
    ide = ide(:,1) + 1j*ide(:,2);

    n = numel(gold);
    if numel(rtl) ~= n
        fprintf('FAIL: RTL produced %d samples, expected %d\n', numel(rtl), n);
        ok = false; return;
    end
    e1 = rtl - gold;
    maxe = max(max(abs(real(e1))), max(abs(imag(e1))));
    e2 = rtl - ide;
    sig = sum(abs(ide).^2);  nse = sum(abs(e2).^2);
    snr = 10*log10(sig / max(nse, eps));
    fprintf('samples compared      : %d  (%d symbols x %d)\n', n, cfg.NSYM, cfg.N + cfg.CP);
    fprintf('RTL vs golden (fixed) : max |err| = %g LSB   (tolerance %g)\n', maxe, tol_lsb);
    fprintf('RTL vs ideal  (float) : max |err| = %.2f LSB, SNR = %.1f dB\n', ...
            max(max(abs(real(e2))), max(abs(imag(e2)))), snr);
    % per-symbol float-vs-RTL error (symbols that clip inside a stage show up here)
    L = cfg.N + cfg.CP;
    for m = 1:cfg.NSYM
        r = (m-1)*L + (1:L);
        em = rtl(r) - ide(r);
        fprintf('  sym %2d: max |err vs float| = %8.2f LSB   SNR = %6.1f dB\n', m-1, ...
                max(max(abs(real(em))), max(abs(imag(em)))), 10*log10(sum(abs(ide(r)).^2)/max(sum(abs(em).^2),eps)));
    end
    ok = (maxe <= tol_lsb);
    if ok, fprintf('COMPARE PASS\n'); else, fprintf('COMPARE FAIL\n'); end
end

function c = read_cfg(f)
    fid = fopen(f, 'r');  c = struct();
    while true
        l = fgetl(fid);  if ~ischar(l), break; end
        kv = strsplit(strtrim(l), '=');
        if numel(kv) == 2, c.(kv{1}) = str2double(kv{2}); end
    end
    fclose(fid);
end

function z = read_words(f, DW)
    fid = fopen(f, 'r');  z = [];
    while true
        l = fgetl(fid);  if ~ischar(l), break; end
        l = strtrim(l);  if isempty(l), continue; end
        l = regexprep(l, '^0x', '');
        v = hex2dec_wide(l);
        re = mod(v, 2^DW);          im = mod(floor(v / 2^DW), 2^DW);
        if re >= 2^(DW-1), re = re - 2^DW; end
        if im >= 2^(DW-1), im = im - 2^DW; end
        z(end+1, 1) = re + 1j*im; %#ok<AGROW>
    end
    fclose(fid);
end

function v = hex2dec_wide(s)
    v = 0;
    for i = 1:numel(s)
        v = v*16 + hex2dec(s(i));
    end
end
