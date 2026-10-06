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
| Maia | 4096-bin spectrometer at the ADC rate, LO +/-1.38 MHz (0.45 x 3.072 MS/s) | anything one look can show: wider views, and narrower ones the stream misses (DATV's LO beside the signal) |
| Sweep | the LO stepped across, Maia's rows stitched | views wider than one look (+/-2.5, +/-5, +/-12.5 MHz) |

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
