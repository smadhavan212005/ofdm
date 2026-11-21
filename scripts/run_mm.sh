#!/usr/bin/env bash
# run_mm.sh NAME ROWS COLS AW CLK_GATE NTESTS KMAX GAP_MODE [SEED]
#   Golden vectors (MATLAB/Octave) -> Icarus simulation of rtl/accel/systolic_mm.v -> MATLAB/Octave comparison.
set -euo pipefail
NAME=$1; ROWS=$2; COLS=$3; AW=$4; CG=$5; NT=$6; KMAX=$7; GAP=$8; SEED=${9:-1}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
SIM=$ROOT/sim/$NAME
mkdir -p "$SIM"; cd "$SIM"
if [ "${MATLAB:-octave}" = "matlab" ]; then RUN() { matlab -batch "$1"; }
else RUN() { octave --no-gui -q --eval "$1" 2>&1 | grep -v -e DISPLAY -e 'disabling GUI'; }; fi

RUN "addpath('$ROOT/model'); mm_gen_vectors('$SIM',$ROWS,$COLS,$AW,$NT,$KMAX,$SEED);"
iverilog -g2012 -Wall -o sim.vvp \
  -P tb_systolic_mm.ROWS=$ROWS -P tb_systolic_mm.COLS=$COLS -P tb_systolic_mm.AW=$AW \
  -P tb_systolic_mm.CLK_GATE=$CG -P tb_systolic_mm.NTESTS=$NT -P tb_systolic_mm.GAP_MODE=$GAP \
  $ROOT/rtl/accel/clk_gate.v $ROOT/rtl/accel/mac_pe.v $ROOT/rtl/accel/systolic_mm.v $ROOT/tb/tb_systolic_mm.v 2>&1 | grep -v 'timescale' || true
vvp -n sim.vvp | tee sim.log
grep -q "TB PASS" sim.log
RUN "addpath('$ROOT/model'); ok = mm_compare('$SIM'); if ~ok, exit(1); end"
