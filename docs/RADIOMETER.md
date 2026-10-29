# Radiometer: radio astronomy on the FPGA spectrometer

trxd `src/radiometer.rs`; the engine hooks in `src/trx.rs` (`ra_*`); the
RADIOMETER card in `web/index.html`. Added 2026-10-29 after a design
review; nothing below has been checked against the sky yet (see
"Bench first").

## What is measured

The Maia spectrometer integrates 4096-bin power spectra of the whole
converter band in the FPGA (`src/maia.rs`: 1..1023 FFTs per row, rows at
15 a second). Every full-rate row goes into the radiometer, weighted by
its integration time, into the accumulator of the phase the operator
marked. One sample (`integration_s`, 1 s by default) is the band's power:
the mean of the band's unmasked bins times the band's bin count, so a
masked or flagged bin does not lower it.

Kelvin come only from the hot/cold pair, per bin and for the band:

    g    = (P_hot - P_cold) / (T_hot - T_cold)      counts per kelvin
    T_rx = P_cold / g - T_cold                       everything behind the feed
    T(P) = P / g - T_rx                              antenna temperature

so OFF reads the sky, ON the sky plus the object, their difference the
object (`t_a`), and `T_sys = T_rx + T_off`. The dBm calibration, the
gain tables and the transverter offsets play no part. With the effective
aperture `A_e` (dish area times efficiency) the flux density is
`S = 2 k T_a / A_e` (1 Jy = 1e-26 W/m^2/Hz).

The radiometer equation gives the rms of `t_a` as
`T_sys * sqrt(1 / (B t_on) + 1 / (B t_off))` (`sigma_exp`); the card also
shows the scatter the OFF samples actually have (`sigma_meas`) and the
Allan deviation of the latest run of same-phase samples. Where the Allan
deviation stops falling with tau, gain drift has taken over: that tau is
the longest useful integration between reference marks.

| Board | Window per look | 1 MHz band, 1 s, T_sys 150 K |
|---|---|---|
| Pluto / R2 / Libre DATV images, 3.072 MS/s | +/-1.38 MHz, 750 Hz bins | 0.15 K |
| Libre wide trx image, 24.576 MS/s | +/-11 MHz, 6 kHz bins | 0.15 K (20 MHz band: 0.03 K) |

## Through a transverter

The hot load sits at the transverter's input (the feed), so its gain, its
LO and their drift are inside `g` and `T_rx`. The band is given on the
air; the engine tunes the AD936x to the IF through `Transverter::to_if`
and mirrors the band's offsets for an inverting one. A transverter without
an image filter responds to both sidebands: the hot/cold scale is then a
double-sideband one, a source in one sideband reads half its `t_a`. At
10 GHz the targets are continuum (Sun, Moon, Cas A, Tau A): LO stability
does not matter. The hydrogen line (1420.405 MHz) is a direct AD936x band;
the reference correction from the 1PPS keeps it within a bin.

## The engine while observing (`src/trx.rs`)

* `ra_start` puts the LO beside the band (`band + dc_exclude + 20 kHz`
  below its low edge when that fits the +/-0.45 x adc window, else on its
  centre with the DC bins masked), pins a manual gain (the current one, or
  the one given), opens the analog RX filter over the band, and ends a
  sweep. `ra_stop` puts the gain mode and the filter back and retunes.
* While `ra` is set: `retune` only follows the dial inside the stream
  (`apply_offsets`), `scope_lo_window` and `dc_keepout` give nothing, no
  sweep and no zoom source, `tx_check` refuses, `rxgain` and port switches
  are refused, DATV commands are refused, the filter planner stays out.
* `ra_intake` drains every Maia row each block (the scope and the meter
  took only the newest before: `maia_newest`) and stamps rows with UTC
  from `unix_now` and the row's `Instant`.
* Rows read at a gain other than the pinned one (the readback thread's
  value, 0.5 dB tolerance) are dropped and counted.

## Interference

* Row flag: the band power against the median of the last 31 unflagged
  rows, in units of one row's radiometer sigma (`1 / sqrt(B t_row)`);
  `rfi_row_sigma` (8) over it and the row is left out. The median restarts
  at every phase mark (the power may step there).
* Bin flag at each sample: each band bin against the baseline interpolated
  from 64-bin block medians, in units of its own sigma (`1 / sqrt(FFTs)`);
  `rfi_bin_sigma` (6) over it and the bin is left out of that sample. A
  bin flagged in more than 20 % of the samples (after 10) is masked for the
  session. A wide spectral line can trip this: set `rfi_bin_sigma = 0` for
  line work (the spectra export keeps every bin anyway).
* Always masked: bin 0 (the fastlock flag), DC +/- `dc_exclude_hz`
  (50 kHz: the LO leak and the converter's 1/f), bins outside the band.

## Web commands

`ra_start {center_hz, width_hz, gain_db, integration_s, t_hot, t_cold,
a_eff_m2, dc_exclude_hz, rfi_row_sigma, rfi_bin_sigma}`, `ra_stop`,
`ra_phase {phase: idle|hot|cold|on|off}`, `ra_clear {phase?}`,
`ra_csv {what: series|spectra}` (the file comes back to that page),
`ra_get` (the last 900 samples for the chart). The board sends `{"type":
"ra", "r": Results}` once a second to every page; the state message
carries `ra_ok` (the bitstream has the spectrometer) and `ra` (the band on
the air, the LO, the gain, the phase).

## Bench first (no sky needed)

1. Gain stability: a 50 ohm load on RX, START with a pinned gain, IDLE
   for two hours; the Allan deviation shows the knee between white noise
   and drift against the AD936x temperature in the CSV. Sets the default
   cadence of reference marks.
2. The kelvin arithmetic: the load at room temperature (HOT, T_hot =
   its temperature) and in iced water (COLD, 273 K); with the board's own
   T_rx of about 300 K at 1296 MHz the Y factor is 0.3 dB, the band sigma
   at 1 MHz over 10 s is 0.001 dB: a 1 K check without a noise source.
3. Linearity over the hot/cold/Sun range with the HP 8642B stepped on top
   of the noise (the Siglent only with permission).
4. A carrier from the generator on and off: the row and bin flags, the
   sticky mask, the mirror bin (the quadrature image, 30-40 dB down, may
   need its own flag: not done).
5. The first sky test: a Sun drift scan at 10 GHz (beam width from the
   transit, `t_a` against the expected ~500 K for a 1 m dish).

## Not in this version

Frequency switching for lines (the LO toggled, spectra divided), a noise
diode on a GPIO as a switched reference, drift-scan baseline fitting, the
velocity axis (needs pointing: trxd has none), brightness temperature with
the beam filling, logging to flash. The raw accumulators are in the spectra
export so reductions can be redone on a PC.
