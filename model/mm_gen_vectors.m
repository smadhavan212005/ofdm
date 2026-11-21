function mm_gen_vectors(outdir, ROWS, COLS, AW, NT, KMAX, seed)
% MM_GEN_VECTORS  Test vectors for the systolic MAC array testbench.
%   Files in outdir:
%     tests.txt     per test:  "M N K"  then K lines, each = M values of A(:,k) followed by N values of B(k,:)
%     expected.txt  per test:  M*N integers, C row-major (golden, wrapped to AW bits)
%     tiles.txt     one line per test: "M N K"  (for the comparison script)
%   Test mix: full tiles, random partial tiles, K=1, K=KMAX, extreme values (-128*-128 accumulation,
%   which overflows small AW), zeros, and an all +127 case.
    if ~exist(outdir, 'dir'), mkdir(outdir); end
    if exist('OCTAVE_VERSION', 'builtin') ~= 0, rand('twister', seed); else, rng(seed); end

    ft = fopen(fullfile(outdir, 'tests.txt'), 'w');
    fe = fopen(fullfile(outdir, 'expected.txt'), 'w');
    fm = fopen(fullfile(outdir, 'tiles.txt'), 'w');
    for t = 1:NT
        switch mod(t-1, 8)
            case 0, M = ROWS; N = COLS; K = randi(KMAX);                 kind = 'rand';
            case 1, M = randi(ROWS); N = randi(COLS); K = randi(KMAX);   kind = 'rand';
            case 2, M = randi(ROWS); N = randi(COLS); K = 1;             kind = 'rand';
            case 3, M = ROWS; N = COLS; K = KMAX;                        kind = 'min';   % all -128: max-magnitude products
            case 4, M = randi(ROWS); N = randi(COLS); K = randi(KMAX);   kind = 'zero';
            case 5, M = ROWS; N = COLS; K = KMAX;                        kind = 'max';   % all +127
            case 6, M = 1; N = 1; K = randi(KMAX);                       kind = 'rand';
            otherwise, M = randi(ROWS); N = randi(COLS); K = randi(KMAX); kind = 'edge'; % mix of extremes
        end
        switch kind
            case 'rand', A = randi([-128 127], M, K); B = randi([-128 127], K, N);
            case 'min',  A = -128*ones(M, K);         B = -128*ones(K, N);
            case 'max',  A = 127*ones(M, K);          B = 127*ones(K, N);
            case 'zero', A = zeros(M, K);             B = randi([-128 127], K, N);
            otherwise
                ex = [-128 -127 -1 0 1 126 127];
                A = reshape(ex(randi(numel(ex), 1, M*K)), M, K);   B = reshape(ex(randi(numel(ex), 1, K*N)), K, N);
        end
        C = mm_model(A, B, AW);
        fprintf(ft, '%d %d %d\n', M, N, K);
        for k = 1:K
            fprintf(ft, '%d ', A(:, k)); fprintf(ft, '%d ', B(k, :)); fprintf(ft, '\n');
        end
        fprintf(fe, '%d ', C.'); fprintf(fe, '\n');       % C.' => row-major
        fprintf(fm, '%d %d %d\n', M, N, K);
    end
    fclose(ft); fclose(fe); fclose(fm);
    fprintf('mm_gen_vectors: ROWS=%d COLS=%d AW=%d NT=%d KMAX=%d -> %s\n', ROWS, COLS, AW, NT, KMAX, outdir);
end
