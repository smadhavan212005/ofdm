#!/usr/bin/env bash
# Regression: golden model (Octave/MATLAB) vs RTL (Icarus), bit-exact (TOL=0) unless noted.
#   args to run_sim.sh: NAME N DW TW CP BPS NSYM STALL_MODE [SCALE_MASK] [FLUSH_IDLE] [TOL]
cd "$(dirname "$0")"
declare -a CASES=(
  # ---- N=64 baseline, no mapper, three traffic modes
  "n64_m0        64 16 16 16 0 12 0"
  "n64_m1        64 16 16 16 0 12 1"
  "n64_m2        64 16 16 16 0 12 2"
  # ---- QAM mapper in front
  "n64_qpsk_m1   64 16 16 16 2 8 1"
  "n64_16qam_m2  64 16 16 16 4 8 2"
  "n64_64qam_m0  64 16 16 16 6 8 0"
  # ---- other sizes
  "n256_m0       256 16 16 64 0 6 0"
  "n256_64qam_m2 256 16 16 32 6 6 2"
  "n1024_m1      1024 16 16 128 0 3 1"
  "n16_m2        16 16 16 4 0 12 2"
  "n8_m2         8 16 16 2 0 12 2"
  "n4_m2         4 16 16 1 0 12 2"
  "n2_m2         2 16 16 1 0 12 2"
  # ---- CP corner cases
  "n64_cp0       64 16 16 0 0 6 1"
  "n64_cpN       64 16 16 64 0 6 1"
  # ---- widths / scaling schedules / flush timing
  "n64_dw12      64 12 14 16 0 8 1"
  "n64_dw20      64 20 18 16 0 8 1"
  "n64_noscale2  64 16 16 16 0 8 1 15"
  "n64_fi0       64 16 16 16 0 8 2 63 0"
  "n64_fi200     64 16 16 16 0 8 2 63 200"
)
pass=0; fail=0
for c in "${CASES[@]}"; do
  set -- $c
  if ./run_sim.sh "$@" > /tmp/reg_$1.log 2>&1; then
    printf "PASS  %-14s %s\n" "$1" "$(grep -m1 'outputs:' /tmp/reg_$1.log)"; pass=$((pass+1))
  else
    printf "FAIL  %-14s (see /tmp/reg_$1.log)\n" "$1"; fail=$((fail+1))
  fi
done
echo "------------------------------------------------------------------"
echo "regression: $pass passed, $fail failed"
[ $fail -eq 0 ]
