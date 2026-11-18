#!/usr/bin/env bash
# run_sim.sh NAME N DW TW CP BPS NSYM STALL_MODE [SCALE_MASK] [FLUSH_IDLE] [TOL]
#   Generates vectors with the MATLAB/Octave golden model, simulates with Icarus Verilog,
#   then runs the MATLAB/Octave comparison.  Set MATLAB=matlab to use MATLAB instead of Octave.
set -euo pipefail
NAME=$1; N=$2; DW=$3; TW=$4; CP=$5; BPS=$6; NSYM=$7; MODE=$8
SCALE_MASK=${9:-$(( (1 << $(awk -v n=$N 'BEGIN{print log(n)/log(2)}' | cut -d. -f1) ) - 1 ))}
FLUSH_IDLE=${10:-8}
TOL=${11:-0}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
SIM=$ROOT/sim/$NAME
mkdir -p "$SIM"
cd "$SIM"

if [ "${MATLAB:-octave}" = "matlab" ]; then
  RUN() { matlab -batch "$1"; }
else
  RUN() { octave --no-gui -q --eval "$1" 2>&1 | grep -v -e DISPLAY -e 'disabling GUI'; }
fi

RUN "addpath('$ROOT/model'); gen_vectors('$SIM',$N,$DW,$TW,$CP,$BPS,$NSYM,1,$SCALE_MASK);"

iverilog -g2012 -Wall -o sim.vvp \
  -P tb_ofdm_ifft_tx.N=$N -P tb_ofdm_ifft_tx.DW=$DW -P tb_ofdm_ifft_tx.TW=$TW \
  -P tb_ofdm_ifft_tx.CP=$CP -P tb_ofdm_ifft_tx.BPS=$BPS -P tb_ofdm_ifft_tx.NSYM=$NSYM \
  -P tb_ofdm_ifft_tx.STALL_MODE=$MODE -P tb_ofdm_ifft_tx.SCALE_MASK=$SCALE_MASK \
  -P tb_ofdm_ifft_tx.FLUSH_IDLE=$FLUSH_IDLE -P tb_ofdm_ifft_tx.TOL_LSB=$TOL \
  $ROOT/rtl/qam_mapper.v $ROOT/rtl/ifft_sdf_stage.v $ROOT/rtl/ifft_sdf_core.v \
  $ROOT/rtl/cp_insert_reorder.v $ROOT/rtl/ofdm_ifft_tx.v $ROOT/tb/tb_ofdm_ifft_tx.v 2>&1 | grep -v 'timescale' || true

vvp -n sim.vvp | tee sim.log
grep -q "TB PASS" sim.log

RUN "addpath('$ROOT/model'); ok = compare_results('$SIM',$TOL); if ~ok, exit(1); end"
