# The web scope: sources, LO placement, sweep (2026-10-06)

One planner decides where the picture comes from: `src/trxd/src/scopeplan.rs`,
pure functions with unit tests. It looks only at the view (centre, span) and
the radio (LO, rates, Maia present, may the LO move). The modulation never
enters; modes only constrain where the LO may go (`retune` in `trx.rs`:
the receive channel inside the stream, the TX frequency, DATV's offset
beside the signal).

| Source | Covers | Used for |
|---|---|---|
| Channel | 48 kS/s around the receive frequency | spans up to +/-10 kHz the channel covers |
| Stream | the FPGA's x8 decimated 384 kS/s, LO +/-192 kHz | spans up to +/-150 kHz the stream covers |
| Maia | 4096-bin spectrometer at the ADC rate, LO +/-0.45 x the rate: +/-11 MHz on the LibreSDR trx image (24.576 MS/s), +/-1.38 MHz on the others (3.072 MS/s) | anything one look can show: wider views, and narrower ones the stream misses (DATV's LO beside the signal) |
| Sweep | the LO stepped across, Maia's rows stitched | views wider than one look: only on the 3.072 MS/s images (+/-2.5, +/-5, +/-10 MHz) |

Rules:

- A source is used only where it covers the whole view. The view's LO
  window (`lo_window`) is fed to `retune`; where the LO cannot go there
  (DATV, transmitting), the next source that covers it takes over. No
  empty columns.
- Maia looks open the AD936x's analog RX filter over the view (in 100 kHz
  steps, up to 2.76 MHz). Its default, 1 MHz, hid every station more than
  500 kHz from the LO; narrower views close it again (DATV's own needs
  still apply).
- The DC spike: views up to +/-50 kHz keep the LO outside them
  (`dc_keepout`); wider ones blank it.
- Sweep: 10 looks of 2.76 MHz for +/-12.5 MHz, Maia at 60 rows/s meanwhile
  (one row after the LO settles is taken per look): about 1.5 lines a
  second. While it runs there is no audio, no decoding and no S-meter, and
  the TCI audio is silent; keying ends it (the LO comes back first, then
  one Maia look while sending). Not through a transverter, not in DATV.
  A click on the swept view tunes there and drops to +/-1.25 MHz.

Measured on Libre 2, 100 MHz WFM: +/-125 kHz stream 15 rows/s, +/-1.25 MHz
Maia 15 rows/s with the filter at 2.7 MHz, +/-12.5 MHz sweep 1.5 lines/s
over 87.5-112.5 MHz; DATV mode at 33-500 kS/s (LO 54-347 kHz below the
signal) with no empty columns.

## The wide LibreSDR trx image (2026-10-06)

The LibreSDR's trx image runs the AD9361 at 24.576 MS/s and decimates x64
(two ADI x8 FIR stages) to the same 384 kS/s stream; transmit interpolates
x64 the same way (maia-hdl projects/simple/rate64.tcl: the DAC-side stage
pulls a pre-stage through a small FIFO, a saturating x4 restores the 12 dB
the second stage's FIR scaling costs). Maia's spectrometer therefore sees
+/-11 MHz in one look: the whole FM band live at 15 rows/s, with audio. The
channel DDC decimates 512x (24.576 MS/s -> 48 kHz). trxd takes the rate from
the image's rate file (`/lib/firmware/fpga-trx.rate`: "24576000 64"); the
DATV images stay at 3.072 MS/s and x8. FPGA: 138 of 220 DSPs (the two new
stages: 42), WNS +0.5 ns.

## Zoom and the analog filter on the wide image (2026-10-06 later)

- Maia's core has a second spectrometer input on the wide image (maia_hdl
  config.spectrometer_zoom, register 0x20 bit 16): the first x8 decimation
  stage's output, 3.072 MS/s, 750 Hz bins. The planner takes it for the
  views between the stream (+/-150 kHz, 94 Hz bins) and the full band
  (+/-2.5 MHz and wider, 6 kHz bins): no "big squares" when zooming in.
  Switching inputs is one register bit, no AD9361 recalibration.
- The analog RX filter follows the view in steps on the wide image: 1 MHz
  for the channel/stream views (listening), 6 MHz up to +/-2.5 MHz, all
  of Maia's view above. Kept open at 22 MHz, the whole FM band took the
  AD9361 gain from 73 to 61 dB and RDS decoded on 0 of 6 stations (6 of 6
  at 1 MHz). In the +/-5M and +/-10M views RDS is weaker for that reason.
- Maia's rows are rotated -545 bins by the core (maia.rs ROTATION): trxd
  rotates them back, the zoom rows too.
