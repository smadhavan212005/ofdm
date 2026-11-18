function idx = bitrev_idx(N)
% BITREV_IDX  idx(j+1) = bit-reversal of j over log2(N) bits (j = 0..N-1). Toolbox-free.
    L   = round(log2(N));
    idx = zeros(1, N);
    for j = 0:N-1
        r = 0;  v = j;
        for b = 1:L
            r = r*2 + mod(v, 2);
            v = floor(v/2);
        end
        idx(j+1) = r;
    end
end
