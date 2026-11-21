function ok = mm_compare(simdir)
% MM_COMPARE  Compare the RTL result dump (rtl_c.txt) with the golden expected.txt, test by test.
    tiles = load(fullfile(simdir, 'tiles.txt'));
    got   = fileread(fullfile(simdir, 'rtl_c.txt'));   exp_ = fileread(fullfile(simdir, 'expected.txt'));
    g = sscanf(got, '%d');  e = sscanf(exp_, '%d');
    nt = size(tiles, 1);
    if numel(g) ~= numel(e)
        fprintf('FAIL: RTL dumped %d values, expected %d\n', numel(g), numel(e)); ok = false; return;
    end
    bad = find(g ~= e);
    fprintf('tests: %d, values compared: %d, mismatches: %d\n', nt, numel(e), numel(bad));
    ok = isempty(bad);
    if ok, fprintf('MM COMPARE PASS\n'); else, fprintf('MM COMPARE FAIL (first bad index %d: rtl=%d exp=%d)\n', bad(1), g(bad(1)), e(bad(1))); end
end
