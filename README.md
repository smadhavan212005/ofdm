# OFDM IFFT + Cyclic-Prefix transmitter (RTL + MATLAB golden model)

```
 AXI-S in ─► [qam_mapper] ─► ifft_sdf_core ─► cp_insert_reorder ─► AXI-S out
  (bits or     (optional,     radix-2 DIF SDF    bit-reverse + CP,
  {im,re})     BPS=2/4/6)     N-point, pipelined ping-pong buffer
```

| file | what |
|---|---|
| `rtl/ofdm_ifft_tx.v` | top level, AXI-Stream in/out |
| `rtl/ifft_sdf_core.v` | stream controller (stall, flush) + stage chain |
| `rtl/ifft_sdf_stage.v` | one SDF stage: feedback FIFO, butterfly, twiddle multiplier |
| `rtl/cp_insert_reorder.v` | bit-reversal reorder + cyclic prefix, ping-pong RAM |
| `rtl/qam_mapper.v` | Gray QPSK/16-QAM/64-QAM |
| `model/ifft_sdf_model.m` | bit-accurate fixed-point model (MATLAB or Octave, no toolboxes) |
| `model/gen_vectors.m`, `compare_results.m` | stimulus/expected generation, RTL-vs-model-vs-float report |
| `tb/tb_ofdm_ifft_tx.v` | self-checking TB (3 traffic modes) |
| `scripts/run_sim.sh`, `regress.sh` | one config / 20-config regression |

## Interfaces
* **Input** `s_axis_*`: `BPS=0` → `tdata = {im, re}` (signed, DW bits each). `BPS=2/4/6` → `tdata[BPS-1:0]` = one QAM symbol, layout `[I bits | Q bits]`. One symbol = N accepted beats, subcarrier 0 first. `s_axis_tlast` is ignored (symbols are counted).
* **Output** `m_axis_*`: `tdata = {im, re}`; each symbol = CP + N beats, cyclic prefix first, `tlast` on the last beat.
* **Scaling**: with the default `SCALE_MASK` (every stage ÷2) the core is exactly `1/N · Σ X[k]e^{+j2πkn/N}`, i.e. MATLAB `ifft()`. Clear mask bits to keep gain in the last stages (more SNR, less headroom).

## Parameters (`ofdm_ifft_tx`)
`N` (power of 2, ≥2) · `DW` data width · `TW` twiddle width (Q2.(TW-2)) · `CP` (0..N) · `BPS` (0 = no mapper) · `QAM_PEAK` · `SCALE_MASK` · `FLUSH_IDLE` · `TW_FILE`.
Radix-4 is not implemented: radix-2 SDF, `log2(N)` stages.

## Architecture notes
* **Control follows data.** Each stage gets the time-slot index of its sample (`in_cnt`) and computes its phase from `in_cnt - OFF` (OFF = Σ delays of earlier stages). No per-stage counters, and back-pressure is one global enable.
* **Two registers per stage.** R1: FIFO/butterfly mux. R2: complex multiply, round, saturate. The twiddle multiplier is outside the feedback loop, so the loop is a single adder. Last two stages need no multiplier (twiddles {1} and {1, +j}).
* **Latency** core: `K = N + 2·log2(N) − 2` clocks. The CP stage adds one symbol of buffering (it must hold the whole symbol to prepend the last CP samples).
* **Throughput**: 1 sample/clk into the core; output is `N/(N+CP)` of the clock rate on the input side because the CP adds samples. The core is back-pressured by the ping-pong buffer.
* **Flush.** An SDF pipeline leaves the tail of a symbol in its delay lines. If input idles on a symbol boundary for more than `FLUSH_IDLE` cycles the core injects K zero ticks and re-aligns. Back-to-back symbols never flush; mid-symbol gaps just freeze the pipeline. While flushing, `s_ready=0`, so a symbol arriving right after a flush starts waits up to K cycles.
* **Rounding/saturation** (model and RTL identical): round-half-up, saturate to DW bits at every stage output. A full-scale input can clip inside a stage (a ½-scaled butterfly with a 45° twiddle reaches √2·full-scale). Keep headroom: the mapper default peak is `2^(DW-2)`.

## Run
```
./scripts/run_sim.sh n64 64 16 16 16 4 8 2     # N DW TW CP BPS NSYM STALL_MODE  (16-QAM, random stalls + idle gaps)
./scripts/regress.sh                           # 20 configs, bit-exact
MATLAB=matlab ./scripts/run_sim.sh ...         # use MATLAB instead of Octave
```
Needs `iverilog` (≥ 11, `-g2012`) and `octave` or `matlab`. STALL_MODE: 0 continuous, 1 random gaps/back-pressure, 2 adds long idle gaps (flush) and long sink stalls (both banks full).

## Verification status
* Pass criterion: RTL == golden model, **bit-exact** (`TOL=0`; `-P ...TOL_LSB=n` for a tolerance). Also reports RTL vs floating-point `ifft` (SNR, max LSB).
* Regression: N = 2…1024, DW 12/16/20, CP = 0/N/others, QPSK/16/64-QAM, partial scaling masks, FLUSH_IDLE 0/8/200, all traffic modes → 20/20 pass.
* Mutation checks: disabling the flush or changing the rounding constant makes the TB fail.
* Typical SNR vs float (N=64, DW=16, quarter-scale input): 53–65 dB; the full-scale stress symbol clips by design.
* Not done: MATLAB itself was not available here (model run under Octave 8.4; only base functions are used). Not synthesised for a real FPGA/ASIC target: no timing or area numbers. Yosys elaborates it; the FIFOs/RAM map to SRL/BRAM only in an FPGA flow.

## Accelerator: parameterized INT8 systolic matrix-multiply unit with clock gating

Files: `rtl/accel/{clk_gate,mac_pe,systolic_mm}.v`, `model/mm_{model,gen_vectors,compare}.m`, `tb/tb_systolic_mm.v`, `scripts/{run_mm,regress_mm}.sh`.

**Architecture.** Output-stationary ROWS x COLS array. PE(r,c) owns C[r][c]. A streams left-to-right, B top-to-bottom, with internal input skew (row r delayed r clocks, column c delayed c clocks). K is streamed; a valid token travels with A. Signed INT8 operands, AW-bit wrap-around accumulators (AW >= 2*DW). Run-time tile size `m_len <= ROWS`, `n_len <= COLS`. FSM: IDLE -> RUN -> DRAIN -> DONE; `start` is accepted from IDLE or DONE (back-to-back tiles).

**Interface.** `start, m_len, n_len`, streaming `in_valid/in_last/in_ready` with `a_in[ROWS*DW]`, `b_in[COLS*DW]`; `busy, done, c_valid`, `c_out[ROWS*COLS*AW]` (row-major, zero outside the M x N tile).

**Clock gating.** `CLK_GATE=1`: one latch-based ICG at array level plus one per PE (enabled only for PEs inside the active tile while busy); `test_en` forces clocks on for scan. `CLK_GATE=0`: plain clock-enable baseline for comparison.

**Verification.** MATLAB golden model (`C = A*B` wrapped to AW bits). Testbench covers full/partial tiles, K=1, all -128, all +127, zeros, edge values, 1x1, random bubbles, stray inputs while idle, a spurious start mid-run, back-to-back starts, and random garbage on unused lanes. `scripts/regress_mm.sh`: 17 configurations (4x4, 1x1, 2x3, 3x2, 1x4, 4x1, 5x7, 8x8, AW=16/18/20, K up to 200), all bit-exact PASS. Verilator lint is clean apart from the intentional ICG latches. Yosys elaboration of 4x4 gives 16 multipliers and 17 latches (16 PE + 1 array).

**Clock-activity result.** PE clock edges are 37-56% below the ungated reference on the gated runs (0% with `CLK_GATE=0`, as expected). This depends on tile-size mix and idle gaps.

**Mutation checks.** Mutants that were detected: PE enable dropping during DRAIN, PE accumulating without a valid token, B-skew one register short, a start accepted while busy, and the skew-valid flush removed (found only after unused lanes were driven with random data). Two mutants survive: the redundant `v_out` flush in `mac_pe`, and clearing PEs by the old tile size instead of the new one. Both appear to be equivalent mutants (tokens fully drain before DONE; PEs outside the previous tile are already zero). I did not formally prove this.

**Honest limits.**
- The ICG is a behavioral latch model. It was not run through synthesis, STA or CTS.
- Clock-edge counts are an activity proxy, not measured power.
- No Fmax or area numbers.
- Accumulators wrap; there is no saturation.
- On FPGA, use BUFGCE or plain clock enables instead of the latch ICG.
