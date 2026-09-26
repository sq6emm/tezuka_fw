# Performance on the LibreSDR (2x Cortex-A9, xc7z020)

Measured on 2026-09-26 on the board itself, trxd `role = "trx"`, 1296 MHz,
USB, one web client connected (scope + audio), nothing else switched on.
"% of one core" comes from trxd's own engine profile (logged once a minute,
`engine profile (% of one core): ...`); "total" is `/proc/stat` busy time over
both cores.

## Where the CPU went, and what was done

| Stage (engine thread) | before | after | what changed |
|---|---|---|---|
| DDC 384 -> 48 kHz | 13-14 % | 11 % | - |
| receive DSP (SSB demod + AGC) | 57 % | 10 % | narrow modes (SSB/CW/data) demodulate at 12 kHz: the 331-tap complex passband FIR runs at a quarter of the rate |
| 48 -> 12 kHz audio decimation | 1 % | 0 % | the 12 kHz demod output *is* the 12 kHz audio |
| web scope | 9-10 % | 6 % | - |
| engine load (wall time) | 82-85 % | 28 % | |
| live CW decoder thread | 33 % | 0 % outside CW mode | the timing decoder refits a 6 s window 10 times a second; it now runs in CW mode only (as on the IC-705) |
| **total, both cores** | **~95 %** | **40 %** | |

Two more things that were cheap to fix and expensive to leave:

- The AD9361 `in_voltage0_hardwaregain` attribute costs **74 ms per read**
  (SPI round trip in the driver). It is read once a second on its own thread
  (`rx-gain`), never on the sample path.
- DeepCW streaming live (NEURAL in the CW box) needs more than both cores even
  with a 6 s window (`Worker::with_window`, added to sdroxide-deepcw): it
  falls behind and copies fragments. TIMING is the default; NEURAL stays
  available for weak signals when a late copy is acceptable.
- The CW skimmer (a second DeepCW) was removed.

## Remaining budget

About 60 % of the two cores is free with the web UI open. The biggest
remaining consumers are the DDC (11 %), the scope (6 %), the live CW decoder
in CW mode (~33 %), and Q65/PI4 when switched on (each a burst every
60 s).

## What could move into the FPGA

The Libre bitstream (`simple` project, Vivado 2023.1) uses 27 % of the LUTs,
26 % of the BRAM (37 of 140 tiles), 44 % of the DSP48 (96 of 220) and meets
timing with 0.63 ns to spare. It already does the heavy fixed-rate work:
the x8 FIR decimator/interpolator (3.072 MS/s <-> 384 kS/s) and Maia's wide
spectrometer (every span above 300 kHz).

| Candidate | CPU saved | FPGA cost | Verdict |
|---|---|---|---|
| Channel DDC (NCO + decimation 384 k -> 48 k) | ~11 % of one core | NCO + two CIC/FIR stages, a second DMA stream (the 384 k stream must stay for TCI IQ and the scope) | Worth it only if CPU becomes short again; about a week with simulation and two board builds. |
| Narrow scope (spans <= 300 kHz) | ~6 % | the ARM FFTs a decimated stream Maia does not see | Not worth it. |
| SSB demod filter | ~5 % now | a fixed FIR with loadable taps | Not worth it after the 12 kHz change. |
| Timing CW decoder | 33 % in CW mode | it is not DSP: a search over a speed/threshold grid | Better fixed in software (see below). |
| **DeepCW (neural)** | enables NEURAL live | see below | Feasible, large project. |

### DeepCW in the FPGA

The model is a Conformer-CTC with about 3.6 M parameters, reading a 65-bin
log spectrogram at 66.7 frames/s. One 6 s window is about 0.4-1.4 GMAC
(parameters x frames after subsampling); one decode a second is more than
the two A9 cores' realistic NEON throughput, which is what was measured.

The fabric left over has 124 DSP48s: about 18 GMAC/s at 150 MHz in int8.
Weights quantised to int8 are 3.6 MB, too much for BRAM (about 450 kB free)
but trivial to stream from DDR once per window through an HP port. So the
shape that fits is a **matrix-multiply engine** (systolic int8 array + weight
and activation DMA) with the ARM keeping the small non-matmul operations
(layer norm, softmax, activations, the depthwise convolutions).

What that takes, in order:

1. Quantise the ONNX model to int8 and check on PC recordings (the SR6LB
   beacon, the loopback test) that it still copies.
2. The GEMM engine in HDL with cocotb tests (maia-hdl already has the
   infrastructure), timed at 150 MHz in the `simple` project.
3. A UIO driver and an rten operator (or a small hand-written executor for
   this one graph) that sends the matmuls to the engine.
4. Measure: the goal is one decode a second of a 6 s window at < 20 % CPU.

That is weeks of work and several bitstream builds, not a night. It was not
started without agreeing on it first.

### Cheaper first

- **Timing decoder**: `CwDecoder` re-analyses its 6 s window every 100 ms
  (`HOP_S` in sdroxide-dsp `cw.rs`). Every 250 ms would cut it to ~40 % of
  the cost for a quarter second more latency; skipping analysis while the
  signal is a steady carrier or silence would cut most of the rest.
- **DeepCW in software**: decode every 2 s instead of 1 s, and only while
  there is keying at the pitch (the timing front end already knows).
