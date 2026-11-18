**GitGub Repository: https://github.com/smadhavan212005/ofdm/**

# SDF-OFDM-Tx: Pipelined OFDM Transmitter Core (IFFT + Cyclic Prefix)

A parameterizable, fully pipelined **OFDM transmitter back-end** written in Verilog/SystemVerilog and verified against a **bit-accurate MATLAB golden model**.

```
 AXI-Stream in ──► [QAM mapper] ──► SDF IFFT core ──► bit-reverse + CP insert ──► AXI-Stream out
   (bits or I/Q)     (optional)      radix-2 DIF,        ping-pong buffer,          (CP + N samples
                                     N points,           N+CP beats / symbol         per symbol, tlast)
                                     1 sample / clock
```

Contents

1. [Overview and features](#1-overview-and-features)
2. [Repository layout](#2-repository-layout)
3. [Quick start](#3-quick-start)
4. [Interfaces and data formats](#4-interfaces-and-data-formats)
5. [Parameters](#5-parameters)
6. [Theory and math](#6-theory-and-math)
7. [Architecture in detail](#7-architecture-in-detail)
8. [Fixed-point design and calculations](#8-fixed-point-design-and-calculations)
9. [Timing, latency, throughput and resources](#9-timing-latency-throughput-and-resources)
10. [Verification workflow](#10-verification-workflow)
11. [Results](#11-results)
12. [Design decisions and trade-offs](#12-design-decisions-and-trade-offs)
13. [Limitations and future work](#13-limitations-and-future-work)
14. [Tool requirements](#14-tool-requirements)

---

## 1. Overview and features

An OFDM transmitter turns a block of frequency-domain symbols (one complex value per subcarrier) into a time-domain waveform: **IFFT**, then prepend a **cyclic prefix (CP)** that copies the end of the symbol to the front. The CP absorbs multipath delay spread and turns linear convolution with the channel into circular convolution, which makes one-tap equalization possible at the receiver.

This project implements that chain in hardware:

| Feature | Detail |
|---|---|
| Transform | N-point IFFT, N a power of two (verified 2 to 1024), radix-2 decimation-in-frequency |
| Architecture | Single-path delay feedback (SDF), one complex sample per clock, log2(N) stages |
| Pipelining | 2 register stages per butterfly stage; twiddle multiplier outside the feedback loop |
| Cyclic prefix | Any length 0..N, inserted by a ping-pong buffer that also undoes the bit-reversed output order |
| Interfaces | AXI-Stream in and out with full back-pressure (`tvalid`/`tready`/`tlast`) |
| Mapper (optional) | Gray-coded QPSK, 16-QAM, 64-QAM in front of the IFFT |
| Fixed point | Parameterizable data width `DW` and twiddle width `TW`; per-stage scaling mask; round-half-up; saturation |
| Verification | Bit-accurate MATLAB model; self-checking testbench; 20-configuration regression; RTL equals model with 0 LSB error |

Application example: IEEE 802.11a/g uses N=64 with a 16-sample CP (25 %) at 20 MS/s. This core handles those parameters directly (`N=64, CP=16, BPS=2/4/6`) and would only need a clock of about 20 MHz or more to sustain that rate. (Pilot insertion, null carriers and windowing are not part of this project; see Section 13.)

---

## 2. Repository layout

```
.
├── README.md
├── rtl/
│   ├── ofdm_ifft_tx.v        Top level: mapper + IFFT core + CP stage, AXI-Stream
│   ├── ifft_sdf_core.v       Stream controller (stall, flush, latency tags) + stage chain
│   ├── ifft_sdf_stage.v      One SDF stage: feedback FIFO, butterfly, twiddle multiply, round/saturate
│   ├── cp_insert_reorder.v   Bit-reversal reorder + cyclic prefix, ping-pong buffer
│   └── qam_mapper.v          Combinational Gray QAM mapper
├── model/                    MATLAB / Octave golden model (no toolboxes needed)
│   ├── ifft_sdf_model.m      Bit-accurate fixed-point IFFT model
│   ├── twiddle_table.m       Integer twiddle factors
│   ├── bitrev_idx.m          Bit-reversal index (toolbox-free)
│   ├── qam_map.m             Bit-accurate QAM mapper
│   ├── gen_vectors.m         Creates stimulus, expected output, twiddle ROM, ideal float output
│   └── compare_results.m     RTL vs golden vs floating-point report
├── tb/
│   └── tb_ofdm_ifft_tx.v     Self-checking testbench (3 traffic modes)
├── scripts/
│   ├── run_sim.sh            One configuration end to end
│   └── regress.sh            20-configuration regression
└── sim/                      Generated files (created by the scripts)
```

(The RTL module is named `ofdm_ifft_tx`; rename the top module and folder to match your project name if you wish.)

---

## 3. Quick start

```bash
# One configuration: N=64, DW=16, TW=16, CP=16, 16-QAM (BPS=4), 8 symbols, traffic mode 2
./scripts/run_sim.sh my_run 64 16 16 16 4 8 2

# Full regression (20 configurations, bit-exact comparison)
./scripts/regress.sh

# Use MATLAB instead of Octave
MATLAB=matlab ./scripts/run_sim.sh my_run 64 16 16 16 4 8 2
```

`run_sim.sh` arguments:

```
NAME N DW TW CP BPS NSYM STALL_MODE [SCALE_MASK] [FLUSH_IDLE] [TOL_LSB]
```

| Arg | Meaning |
|---|---|
| NAME | Output folder under `sim/` |
| N, DW, TW, CP | Transform size, data width, twiddle width, cyclic prefix length |
| BPS | 0 = no mapper (complex input); 2 / 4 / 6 = QPSK / 16-QAM / 64-QAM |
| NSYM | Number of OFDM symbols to simulate |
| STALL_MODE | 0 continuous; 1 random gaps and back-pressure; 2 adds long idle gaps (flush) and long sink stalls |
| SCALE_MASK | Bit s = 1 means stage s divides by 2 (default: all ones) |
| FLUSH_IDLE | Idle cycles at a symbol boundary before auto-flush (default 8) |
| TOL_LSB | Allowed per-component error vs the golden model (default 0 = bit-exact) |

A passing run prints `TB PASS` from the testbench and `COMPARE PASS` from MATLAB/Octave.

---

## 4. Interfaces and data formats

### 4.1 Top-level ports (`ofdm_ifft_tx`)

| Port | Dir | Width | Description |
|---|---|---|---|
| `clk`, `rst_n` | in | 1 | Clock, synchronous active-low reset |
| `s_axis_tdata` | in | `BPS==0 ? 2*DW : BPS` | Input beat (see below) |
| `s_axis_tvalid` / `s_axis_tready` | in / out | 1 | AXI-Stream handshake |
| `s_axis_tlast` | in | 1 | Ignored: symbols are counted internally (N beats = one symbol) |
| `m_axis_tdata` | out | `2*DW` | `{im[DW-1:0], re[DW-1:0]}`, signed two's complement |
| `m_axis_tvalid` / `m_axis_tready` | out / in | 1 | AXI-Stream handshake |
| `m_axis_tlast` | out | 1 | High on the last of the N+CP beats of each symbol |

### 4.2 Input word

* `BPS = 0`: `tdata = {im, re}`, each `DW` bits signed. One beat per subcarrier, subcarrier 0 first, N beats per symbol.
* `BPS = 2/4/6`: `tdata[BPS-1:0]` holds one QAM symbol, layout `[ I bits (upper BPS/2) | Q bits (lower BPS/2) ]`.

### 4.3 Output symbol

```
beat index :  0 ... CP-1 | CP ... CP+N-1
content    :  y[N-CP..N-1] (cyclic prefix) | y[0..N-1]   (time-domain samples, natural order)
```

Total `N + CP` beats with `tlast` on the last.

### 4.4 Core-internal valid/ready (`ifft_sdf_core`)

Input in natural order; output in **bit-reversed order** (stream position j carries time sample `bitrev(j)`). The CP stage reorders it.

---

## 5. Parameters

| Parameter | Default | Range / meaning |
|---|---|---|
| `N` | 64 | Power of two, at least 2 (verified 2 to 1024) |
| `DW` | 16 | Data width per I/Q component, between stages and at the output |
| `TW` | 16 | Twiddle width. Format Q2.(TW-2), so 1.0 equals 2^(TW-2). TW at least 3 |
| `CP` | 16 | Cyclic prefix length, 0..N |
| `BPS` | 0 | 0 = no mapper; 2, 4 or 6 |
| `QAM_PEAK` | 2^(DW-2) | Outermost constellation level per axis (mapper only) |
| `SCALE_MASK` | -1 (all ones) | Bit s = 1: stage s divides by 2 |
| `FLUSH_IDLE` | 8 | Idle cycles at a symbol boundary before the auto-flush starts |
| `TW_FILE` | `"twiddle.mem"` | Hex ROM file generated by `gen_vectors.m` |

Practical limits: the MATLAB model packs I/Q words into a double, so `2*DW <= 52` (DW up to 26). The RTL uses SystemVerilog-2012 `localparam` in the module header (Icarus needs `-g2012`).

---

## 6. Theory and math

### 6.1 The transform implemented

```
y[n] = (1/N) * sum_{k=0}^{N-1} X[k] * exp(+j*2*pi*k*n / N),     n = 0..N-1
```

This is the same definition as MATLAB `ifft()`. With every stage scaled by 1/2, the 1/N factor is built in (2^log2(N) = N).

### 6.2 Radix-2 decimation-in-frequency (DIF) decomposition

Split the input index into first and second halves:

```
y[2m]   = (1/N) * sum_{k=0}^{N/2-1} ( X[k] + X[k+N/2] )               * exp(+j*2*pi*k*m/(N/2))
y[2m+1] = (1/N) * sum_{k=0}^{N/2-1} ( X[k] - X[k+N/2] ) * W_N^k        * exp(+j*2*pi*k*m/(N/2))

with  W_N^k = exp(+j*2*pi*k/N)
```

So one stage forms, for every pair (a, b) separated by D = N/2:

```
sum  = a + b                      -> even-indexed outputs
diff = (a - b) * W                -> odd-indexed outputs
```

and the same procedure repeats on the two half-size problems. After log2(N) stages the result is in **bit-reversed order**: the sample at stream position j is time sample `bitrev(j)` (bit reversal over log2(N) bits).

### 6.3 Stage parameters

For stage `s = 0 .. S-1`, with `S = log2(N)`:

```
D_s   = N / 2^(s+1)                 feedback delay (number of pairs) of the stage
block = 2 * D_s                     block length the stage works on
OFF_s = sum_{i<s} D_i = N - N/2^s   accumulated delay of earlier stages
W     = W_N^k,  k = n * N / (2*D_s),  n = 0 .. D_s-1   twiddle of pair n
```

Example, N = 8 (S = 3): D = 4, 2, 1 and OFF = 0, 4, 6. The total feedback storage is `sum D_s = N - 1` complex words, which is why SDF is memory-efficient.

Special stages: the last stage (D = 1) has only `W = 1`; the second-to-last (D = 2) has `W in {1, +j}`. These two stages need no multiplier (a swap and a negate for the +j).

### 6.4 SDF operation of one stage

Each stage holds a FIFO of depth D. In every block of 2D input samples:

```
Phase 0 (first D samples):  store input x into FIFO; emit the FIFO output (diff from the previous block) * W
Phase 1 (next D samples)  : a = FIFO output (the earlier sample), b = input
                            emit (a + b);  store (a - b) into FIFO
```

The sums leave while the differences wait in the FIFO and leave during phase 0 of the next block, so the output is one continuous stream `[sums ... diffs ...]`, shifted in time by D relative to the input. That shift is exactly the `OFF` used to align the next stage (Section 7.2).

---

## 7. Architecture in detail

### 7.1 Block diagram

```
            en (global tick)                         en
 s_axis ─┬─► [mapper] ─► din ─► STAGE 0 ─► STAGE 1 ─ ... ─► STAGE S-1 ─► m_re/m_im
         │               cnt ─►  D=N/2     D=N/4            D=1           (bit-reversed)
         │    ┌──────────────────────────────────────────────────────────┐
         └───►│ controller: slot counter `cnt`, tag shift register,      │
              │ stall (can_adv), idle detect, flush FSM                  │
              └──────────────────────────────────────────────────────────┘
                                                   │ m_valid/m_ready
                                                   ▼
                              cp_insert_reorder (ping-pong, bit-reverse write, CP read) ─► m_axis
```

### 7.2 "Control follows data": the slot counter

Instead of a separate counter per stage, the controller keeps one free-running slot counter `cnt` (0..N-1) and attaches it to every sample entering stage 0. Each stage passes the counter along through the **same registers as the data**, so each stage always sees the slot index belonging to the sample at its input.

A stage derives its control from that carried value:

```
p     = (cnt - OFF_s) mod N       position in this stage's own stream
phase = bit log2(D) of p           0: store / emit diffs, 1: butterfly
n     = p mod D                    pair index (twiddle index)
k     = n * N/(2D)                 index into the shared twiddle ROM
```

Why `OFF` works: stage s produces its first useful output D_s slots after its input begins, so stage s+1's stream is stage s's stream delayed by D_s; the subtraction undoes that shift so that every stage sees its first real block starting at p = 0.

### 7.3 One stage: datapath and the two pipeline registers

```
 in ─►(+/-)─► mux ─► R1 ─► complex multiply ─► round ─► saturate ─► R2 ─► out
        ▲      │      (z, w, cnt)   by W (or 1)
        └─ FIFO ◄─ diff / x          (FIFO depth D, width DW+1)
```

* **FIFO** (depth D, width DW+1 per component): holds raw inputs (DW bits) or differences (DW+1 bits). It stores the **unrounded** difference, so no precision is lost in the feedback path.
* **Butterfly**: `sum = fifo_out + x`, `diff = fifo_out - x`, each DW+1 bits (no overflow possible).
* **Mux**: in phase 1 the multiplier input is `sum` with weight 1.0; in phase 0 it is `fifo_out` with weight `W[n]`. So **one** complex multiplier handles both paths.
* **R1 register**: holds `z` (DW+1), the twiddle `(c, s)`, and `cnt`. The feedback loop contains only the add/subtract and the mux.
* **Multiplier, round, saturate (R2)**: because the multiplier is outside the feedback loop it can be pipelined freely without breaking the recurrence.

Per generic stage: 4 real multipliers (a 3-multiplier form is possible), 4 real adders in the butterfly, 2 adders in the complex product. For N = 64 (S = 6) there are 4 generic stages = 16 real multipliers; for N = 256, 6 stages = 24.

### 7.4 Stream controller (`ifft_sdf_core`)

All state (FIFOs, R1/R2 registers, slot counter, tag shift register) advances only on a single enable `en`:

```
can_adv    = ~m_valid | m_ready          output register free or being consumed
run_tick   = ~flushing & s_valid & can_adv
flush_tick =  flushing & can_adv
en         = run_tick | flush_tick
s_ready    = ~flushing & can_adv
```

Back-pressure is therefore a simple global stall: freezing every register preserves all data. A sample that is consumed while `en` is low just clears `m_valid`.

**Latency tag.** A 1-bit "real sample" tag is shifted through a K-bit shift register on every tick; the output `m_valid` is the tag that has travelled the same distance as the data. It marks which output beats are real (not pipeline-fill garbage, not flush zeros).

### 7.5 Why a flush is needed, and how it works

An SDF pipeline always keeps the last ~N samples of a symbol inside its delay lines; they only come out when more samples are clocked in. In a continuous OFDM stream the next symbol pushes them out, but after the last symbol (or an idle gap) they would be stuck.

Flush procedure:

1. The input is idle at a symbol boundary (`cnt == 0`, data in flight) for more than `FLUSH_IDLE` cycles.
2. The controller feeds **K zero ticks** (`s_ready = 0` meanwhile), pushing the last real samples to the output.
3. The slot counter is reset to 0 for the next symbol. Leftover data in the delay lines is irrelevant: it only affects pipeline-fill positions whose output is tagged invalid.

Input gaps in the **middle** of a symbol never flush; the pipeline just freezes. Back-to-back symbols never flush.

### 7.6 Cyclic prefix stage (`cp_insert_reorder`)

Ping-pong memory: 2 banks x N words x `2*DW` bits.

* **Write side**: the core delivers a symbol in bit-reversed order. Sample number j (counter `wcnt`) is written to address `{bank, bitrev(wcnt)}`, so after N writes the bank holds the symbol in natural order. When the bank is full, the writer toggles to the other bank.
* **Read side**: reads addresses `N-CP .. N-1` first (the cyclic prefix), then `0 .. N-1`:
  ```
  raddr = (ridx < CP) ? (N - CP + ridx) : (ridx - CP),     ridx = 0 .. N+CP-1
  ```
  The memory read register **is** the AXI-Stream output register, so the read loop is one stage deep. `tlast` is asserted at `ridx = N+CP-1`.
* **Bank-full flags**: each bank has a writer toggle and a reader toggle; `full = wtog XOR rtog`. Each toggle has a single driver, so there is no multi-driver hazard.
* **Back-pressure**: `i_ready = ~full[wbank]`. The core stalls only when both banks are full.

The CP forces a full-symbol buffer because the last CP samples of the symbol (produced last) must be sent first.

### 7.7 QAM mapper (`qam_mapper`)

Gray-coded, 802.11 style, per axis with `m = BPS/2` bits:

```
bin   = gray2bin(g) = g ^ (g>>1) ^ (g>>2) ...
level = 2*bin - (2^m - 1)               -> odd integers in [-(2^m-1), +(2^m-1)]
out   = level * UNIT,   UNIT = floor(PEAK / (2^m - 1))
```

16-QAM per axis: `00 -> -3`, `01 -> -1`, `11 -> +1`, `10 -> +3`. With the default `PEAK = 2^(DW-2) = 16384` (DW = 16):

| Mod | m | UNIT | Output levels per axis |
|---|---|---|---|
| QPSK | 1 | 16384 | +/-16384 |
| 16-QAM | 2 | 5461 | +/-5461, +/-16383 |
| 64-QAM | 3 | 2340 | +/-2340, 7020, 11700, 16380 |

The constellation is scaled by peak amplitude, not normalized to unit average power.

---

## 8. Fixed-point design and calculations

### 8.1 Number formats

| Signal | Format |
|---|---|
| Input / stage output / final output | signed `DW` bits per component (default 16) |
| FIFO / butterfly outputs | signed `DW+1` bits (extra bit for the sum/difference) |
| Twiddle | signed `TW` bits, Q2.(TW-2): `value = round(cos or sin * 2^(TW-2))`, so 1.0 = 2^(TW-2) = 16384 for TW = 16 (exactly representable) |
| Multiplier accumulator | signed `DW + TW + 2` bits (34 bits for 16/16) |

### 8.2 Per-stage arithmetic (identical in RTL and model)

```
z       = sum or diff (DW+1 bits)           w = 1.0 (sum path) or W (diff path)
pr      = z_re*w_c - z_im*w_s               pi = z_re*w_s + z_im*w_c        (exact integers)
SH      = TW - 2 + SC                       SC = 1 if the stage is scaled by 1/2
out     = saturate_DW( floor( (p + 2^(SH-1)) / 2^SH ) )                     (round half up)
```

* The `TW-2` part of the shift removes the twiddle scale factor; `SC` is the ½ scaling.
* Sum path (w = 1.0) reduces to `floor((z + 1)/2)` when scaled, and to `z` when not.
* For the +j stage: `j*(zr + j zi) = -zi + j zr`, then the same rounding.
* Saturation limits are `[-2^(DW-1), 2^(DW-1) - 1]`.

### 8.3 Overall gain and output amplitude

With all stages scaled, the total gain is `(1/2)^S = 1/N`, so the core output equals the exact IFFT. For uncorrelated inputs the output RMS is

```
rms(y) = rms(X) / sqrt(N)
```

(For N = 64 the output is 8x smaller in RMS than the input constellation.) If this loses too many bits, clear `SCALE_MASK` bits in the last stages to keep gain there, at the cost of headroom (saturation then protects against overflow).

### 8.4 Overflow and headroom

* Sum path with scaling: `(a+b)/2` always fits in DW bits, so it cannot overflow.
* Difference path: `|(a-b)/2 * W|` per component can reach about `sqrt(2) * full-scale` (a-b up to 2x full scale, W at 45 degrees), so it **can clip** for adversarial full-scale inputs. The mapper default of `2^(DW-2)` leaves 1 bit of headroom.
* Final output: bounded by `(1/N) * sum |X|`, which is at most `sqrt(2) * max|X|` in the worst case (all subcarriers phase-aligned); real OFDM symbols are far below this.

### 8.5 Quantization noise estimate

Each stage rounds its output (error about uniform, variance 1/12 LSB^2 per component). A butterfly output is `(a +/- b)/2`, so noise already present at the inputs is attenuated by a factor 1/2 in power per stage. The noise injected at stage i therefore reaches the output attenuated by (1/2)^(S-1-i):

```
sigma_q^2 (per component) ~ (1/12) * sum_{j=0}^{S-1} (1/2)^j  ~  1/6  LSB^2
complex noise power       ~  2 * 1/6 = 1/3  LSB^2          (independent of N)
signal power              =  E|X|^2 / N                     (shrinks with N)
```

Example (N = 64, DW = 16, uniform random input in +/-2^14): `E|X|^2 = 2 * (2^15)^2 / 12 = 1.79e8`; divided by 64 this is `2.8e6`; SNR ~ `2.8e6 / (1/3)` ~ **69 dB**. Measured is about 65 dB; the difference comes from the finite twiddle precision and the small positive bias of round-half-up. Because the signal power falls as 1/N while noise stays constant, the SNR drops about 3 dB per doubling of N (Section 11).

### 8.6 Twiddle precision

Rounding error per twiddle component is at most `0.5 / 2^(TW-2)` = about 3e-5 for TW = 16. Twiddle error is a second-order noise contribution compared to the data rounding at DW = 16.

---

## 9. Timing, latency, throughput and resources

### 9.1 Latency (in clock ticks, no stalls)

```
Core:     K = (N - 1) + (2*S - 1) = N + 2*log2(N) - 2        first input -> first core output
          (N-1 from the feedback delays, 2*S from two registers per stage; the final register is the output)
Flush:    K zero ticks (equal to the latency, so the last real sample reaches the output)
CP stage: waits for the whole symbol (N samples), +2 cycles bank flag and read register
First output beat ~ K + N + 2
```

| N | K (core) | First output beat (approx.) |
|---|---|---|
| 64 | 74 | 140 |
| 256 | 270 | 528 |
| 1024 | 1042 | 2068 |

(The 64-point testbench measures first output at cycle 146 from time 0, which includes about 6 reset cycles; this matches 140.)

### 9.2 Throughput

* The core accepts one complex sample per clock (`s_ready` high) while the sink keeps up.
* Each symbol leaves the output as `N + CP` beats, so with a continuous sink the output is always busy and the **input** is accepted at an average of `N / (N + CP)` samples per clock (0.8 for N = 64, CP = 16). The core is stalled for CP cycles per symbol through the ping-pong buffer.
* Measured (N = 64, CP = 16, 4 symbols, no stalls): 320 beats in 320 consecutive cycles, i.e. 100 % output utilization.
* Useful-data efficiency at the output: `N/(N+CP)` (80 % in this configuration); the CP overhead is inherent to OFDM.

Sustained output rate: if the clock is `f_clk`, samples leave at `f_clk` per second, each OFDM symbol takes `(N + CP)/f_clk` seconds. For the 802.11a-style numbers (80 samples per 4 us) a 20 MHz clock is the minimum; a higher clock simply leaves idle gaps.

### 9.3 Storage

| Item | Size (N = 64, DW = 16) | General formula |
|---|---|---|
| Feedback FIFOs | 63 words x 2 x 17 bits = 2142 bits | `(N-1) * 2 * (DW+1)` |
| Ping-pong CP buffer | 2 x 64 x 32 = 4096 bits | `2 * N * 2*DW` |
| Twiddle ROM (shared) | 32 x 32 bits | `(N/2) * 2*TW` |
| Tag shift register | 74 bits | `K` |

### 9.4 Resource and timing status

The design has been elaborated with Yosys and linted with Verilator (width warnings only), but it has **not** been taken through an FPGA/ASIC place-and-route flow, so **no Fmax, LUT, DSP or BRAM figures are claimed**. Expected structure on an FPGA: `(S-2)` complex multipliers mapped to DSP blocks, FIFOs mapped to shift-register LUTs or block RAM, ping-pong buffer in block RAM.

---

## 10. Verification workflow

### 10.1 End-to-end flow

```
  gen_vectors.m  ──► stim.hex       input words
  (MATLAB/Octave)    expected.hex   golden output words (CP included), from the bit-accurate model
                     twiddle.mem    twiddle ROM for the RTL
                     ideal.txt      floating-point reference output
                     cfg.txt        N, DW, TW, CP, BPS, NSYM, SCALE_MASK
        │
        ▼
  iverilog / vvp (tb_ofdm_ifft_tx.v)
        - drives stim.hex through the AXI-Stream input (with gaps in modes 1, 2)
        - applies sink back-pressure (modes 1, 2)
        - compares every output beat and tlast with expected.hex
        - writes rtl_out.hex
        │
        ▼
  compare_results.m  ──► RTL vs golden (fixed point): max error in LSB, pass if <= TOL
                         RTL vs ideal (floating point): max error, SNR, per symbol
```

### 10.2 File formats

* `stim.hex`, `expected.hex`, `rtl_out.hex`: one 16-hex-digit word per line. Output words are `{im, re}` with `re` in the low `DW` bits (two's complement). BPS > 0 stimulus words are the raw QAM bits.
* `twiddle.mem`: one line per `k = 0 .. N/2-1`, `{cos, sin}` with `cos` in the upper `TW` bits.

### 10.3 Stimulus content (BPS = 0)

Symbols cycle through four types: uniform random at quarter scale; full-scale random sign extremes (deliberately stresses saturation); an impulse plus a DC-like pair; a single mid-scale tone. With the mapper enabled, random bits are mapped through the Gray QAM mapper.

### 10.4 Testbench traffic modes

| Mode | Source | Sink |
|---|---|---|
| 0 | Always valid | Always ready (maximum throughput) |
| 1 | Random gaps, including mid-symbol | Random `tready` (about 62 % ready) |
| 2 | Mode 1 plus long idle gaps between symbols (forces auto-flush) | Mode 1 plus occasional long stalls (forces both ping-pong banks full) |

The checker flags value mismatches, `tlast` errors, missing outputs and extra outputs, and a watchdog catches hangs. After the last expected output it waits to make sure nothing extra appears.

### 10.5 Regression matrix (`scripts/regress.sh`)

| Group | Configurations |
|---|---|
| Baseline | N = 64, no mapper, modes 0 / 1 / 2 |
| Mapper | QPSK (mode 1), 16-QAM (mode 2), 64-QAM (mode 0) |
| Sizes | N = 256 (CP 64, and 64-QAM CP 32), 1024, 16, 8, 4, 2 |
| CP corners | CP = 0, CP = N |
| Widths | DW/TW = 12/14, 20/18 |
| Scaling | `SCALE_MASK` = 15 (last two stages unscaled) |
| Flush timing | `FLUSH_IDLE` = 0 and 200 in mode 2 |

Pass criterion: **max |RTL - golden| = 0 LSB** (bit exact), correct `tlast`, exactly NSYM x (N+CP) output beats.

### 10.6 Does the testbench really detect faults? (mutation checks)

* Flush disabled (`FLUSH_IDLE = 100000`) in mode 2: only 480 of 640 beats emerge, `TB FAIL`.
* Rounding constant in the multiplier stage set to 0: mismatches by one LSB appear immediately, `TB FAIL`.

---

## 11. Results

All regression cases pass: **20 / 20, RTL equals the golden model exactly (0 LSB)**, including random stalls, idle gaps with flush, and full ping-pong buffers.

SNR of the fixed-point model against floating-point `ifft` (DW = TW = 16, uniform random input in +/-2^14):

| N | Max error (LSB) | SNR (dB) |
|---|---|---|
| 2 | 0.50 | 86.2 |
| 4 | 0.75 | 76.7 |
| 8 | 1.38 | 78.4 |
| 64 | 1.84 | 65.0 |
| 256 | 2.64 | 59.4 |

(SNR falls roughly 3 dB per doubling of N because the output signal power scales as 1/N while the rounding noise stays constant.)

Per-symbol result of the N = 64 run with four different stimulus types (RTL vs floating point):

| Symbol type | Max error (LSB) | SNR (dB) |
|---|---|---|
| Random, quarter scale | 1.91 | 65.3 |
| Full-scale random (stress) | 1544 | 19.6 (clips inside a stage; RTL still equals the model) |
| Impulse | 0.93 | 63.0 |
| Single tone | 0.78 | 53.1 |

Throughput check: 4 symbols x (64 + 16) = 320 beats delivered in 320 consecutive clock cycles.

---

## 12. Design decisions and trade-offs

* **SDF vs other FFT architectures**: SDF needs only N-1 words of storage and log2(N) butterflies with a single data path, and streams one sample per clock, ideal for an always-on transmitter. The price is a latency of about N and one multiplier per stage (reduced by the trivial last stages).
* **Twiddle multiplier outside the feedback loop**: storing the *unmultiplied* difference keeps the feedback path to a single adder and lets the multiplier be pipelined without changing the FIFO depth. It also avoids rounding the stored values.
* **One multiplier for both paths**: weighting the sum path by 1.0 makes the sum and difference paths share the same datapath, with identical rounding in RTL and model.
* **Carried slot counter instead of per-stage counters**: fewer counters, easy to verify, no misalignment between stages, and stalls are trivially handled by the single enable.
* **Per-stage 1/2 scaling**: prevents growth (no overflow in the sum path) and gives an exact `ifft()`; the mask allows more gain where headroom permits.
* **Round half up + saturation everywhere**: cheap to implement (add a constant, shift) and trivially bit-accurate in MATLAB (`floor((v + R)/P)`).
* **Bit reversal merged into the CP buffer**: the CP stage already needs a full-symbol buffer, so writing to `bitrev(address)` adds no extra memory.
* **Automatic flush**: makes the core usable with sporadic or finite symbol bursts, not only infinite streams, with a programmable idle threshold.
* **Global enable**: simple and robust back-pressure, but it fans out to every register; in a high-frequency design this net may need buffering or replication.
* **`initial` blocks** zero the datapath arrays for simulation and FPGA; control logic has a synchronous reset. An ASIC flow would need explicit resets on the datapath registers instead.

---

## 13. Limitations and future work

* Radix-2 only; a radix-2^2 or radix-4 SDF would halve the number of non-trivial multipliers.
* `N` is fixed at synthesis time (not runtime-configurable).
* No pilot insertion, null/guard subcarriers, DC handling, windowing, scrambling or PAPR reduction; the input is assumed to be the complete subcarrier vector in order.
* Constellations are scaled by peak value, not normalized to unit average power.
* Latency of about N + K samples is inherent in SDF plus the CP buffer.
* Large `D` FIFOs are plain shift registers in RTL; an explicit circular-buffer RAM version would suit very large N.
* `s_axis_tlast` is ignored; adding a resync on `tlast` would make the core robust against framing errors.
* Not yet run through FPGA synthesis and place-and-route, so there are no Fmax/area numbers; MATLAB itself was not available during development (model run in GNU Octave 8.4, base functions only).

Natural next steps: synthesize for a target FPGA and report Fmax, LUT/DSP/BRAM; add a `tuser` start-of-symbol flag; add a radix-2^2 variant; extend with pilot/guard-band insertion and a matching receiver FFT.

---

## 14. Tool requirements

* **Icarus Verilog** 11 or newer (tested with 12.0), `-g2012`
* **MATLAB** (R2019a or newer for `matlab -batch`) (tested with R2019a); no toolboxes are required
* Developed in Linux Environment
* Bash for the helper scripts
