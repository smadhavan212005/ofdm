#!/usr/bin/env bash
# Regression for the systolic MAC array. args: NAME ROWS COLS AW CLK_GATE NTESTS KMAX GAP_MODE [SEED]
cd "$(dirname "$0")"
declare -a CASES=(
  # ---- the default 4x4, gated vs plain clock-enable, three traffic modes
  "mm4x4_g1_m0   4 4 32 1 80 16 0"
  "mm4x4_g1_m1   4 4 32 1 80 16 1"
  "mm4x4_g1_m2   4 4 32 1 80 16 2"
  "mm4x4_g0_m1   4 4 32 0 80 16 1"
  "mm4x4_g0_m2   4 4 32 0 80 16 2"
  # ---- other array shapes
  "mm1x1_g1_m2   1 1 32 1 48 20 2"
  "mm2x3_g1_m2   2 3 32 1 80 16 2"
  "mm3x2_g0_m2   3 2 32 0 80 16 2"
  "mm1x4_g1_m1   1 4 32 1 48 16 1"
  "mm4x1_g1_m1   4 1 32 1 48 16 1"
  "mm8x8_g1_m2   8 8 32 1 80 24 2"
  "mm8x8_g0_m1   8 8 32 0 48 24 1"
  "mm5x7_g1_m2   5 7 32 1 80 16 2"
  # ---- accumulator width: overflow must wrap exactly like the model
  "mm4x4_aw20    4 4 20 1 80 40 2 3"
  "mm4x4_aw16    4 4 16 1 80 40 2 4"
  "mm3x3_aw18_g0 3 3 18 0 80 40 1 5"
  # ---- long K (deep accumulation)
  "mm4x4_k200    4 4 32 1 24 200 1 6"
)
pass=0; fail=0
for c in "${CASES[@]}"; do
  set -- $c
  if ./run_mm.sh "$@" > /tmp/regmm_$1.log 2>&1; then
    printf "PASS  %-14s %s | %s\n" "$1" "$(grep -m1 'cells checked' /tmp/regmm_$1.log)" "$(grep -m1 'PE clock edges' /tmp/regmm_$1.log | sed 's/ \+/ /g; s/ungated reference (cycles x PEs) = [0-9]* *//')"
    pass=$((pass+1))
  else
    printf "FAIL  %-14s (see /tmp/regmm_$1.log)\n" "$1"; fail=$((fail+1))
  fi
done
echo "------------------------------------------------------------------"
echo "mm regression: $pass passed, $fail failed"
[ $fail -eq 0 ]
