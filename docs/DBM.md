# Receive level in dBm

The S-meter, the dBm readout and the scope scale show the level at the
antenna socket in dBm, the same in every mode, filter, bandwidth and AGC
state, once the board is calibrated (tools/sqtrx-cal).

## Measurement (src/trxd/src/power.rs)

Not the demodulator: its filters, decimation and AGC differ per mode. The
level meter reads

* the **48 kS/s channel** every mode shares (the stream mixed to the dial
  and decimated by one fixed filter): bands up to about +-20 kHz around the
  dial (CW, SSB, AM, FM filters);
* the **384 kS/s stream** after the FPGA decimator: bands up to about
  +-160 kHz around the LO;
* a **Maia spectrometer row** (the whole 3 MHz the AD936x delivers):
  anything wider (DATV channels).

Each is an averaged Blackman-Harris FFT (2048 points, 8 averages: about
3 readings a second on the channel) normalised by Parseval: the sum of the
bins is the mean |z|^2, so a complex tone of amplitude A reads 20 log10 A
dBFS wherever it falls between bins, and noise of variance s^2 reads
s^2/fs per Hz in any bandwidth. From it:

* **band power**: the bins of the measured band (the receive filter, or a
  DATV signal's channel), edge bins by the share inside. A carrier reads
  its full power in any filter that holds it (its window lobe is +-94 Hz
  on the channel: a carrier closer than that to a filter edge reads up to a
  few dB low, as it would through a real filter);
* **noise density**: the median of the bins around the band (band, window
  skirts and DC kept out), corrected to the mean; dBm/Hz;
* **clipping**: any stream sample at 0.9 of full scale or above since the
  last reading (the reading is then too low: reduce the gain);
* **mostly noise**: the band holds less than twice the noise in it.

## Calibration (src/trxd/src/calib.rs)

    dBm = dBFS - G - E(G, f) + K(f) + c (T - T0)

* dBFS: band power at the channel's scale (stream readings + `stream_db`,
  spectrometer readings + `maia_db`);
* G: the AD936x RX gain as the chip reports it (manual, or wherever its AGC
  is: read back once a second);
* E(G, f): the chip's error against its report, from the gain table of the
  band holding f (else the nearest), interpolated over G; 0 at the
  reference gain (40 dB);
* K(f): measured at the reference gain at every frequency of the plan,
  interpolated linearly over the AD936x frequency (the IF through a
  transverter, whose own gain is the table's `xvtr` entry);
* c, T0: optional temperature coefficient against the AD936x temperature
  at the measurement (0 unless set).

One table per RX socket pair (`<state_dir>/calib-rx1.json`,
`calib-rx2.json`, on jffs2); the pair in use applies.

Quality, shown in SET and in the meter:

| | |
|---|---|
| calibrated | inside the measured frequencies (within max(20 MHz, 5 %) of a point, or between two) |
| extrapolated (`dBm*`) | outside them: the nearest point's K (above 3.2 GHz with the Siglent: the highest points), or a transverter without an offset |
| legacy (`dBm~`) | no table: the band's old S-meter points (settings.json), if it had any |
| none (`dBm~`) | no table, no points: the board type's curve (below) |

### Without a table: the board type's curve

`calib::board_curve()` picks a curve by the device tree model (ADALM-Pluto,
LibreSDR, PlutoSky R2; anything else takes the LibreSDR's): K = 12 dB plus a correction by frequency, less a part by AGC
gain. The AD936x front end loses gain below a few hundred MHz, mostly in
its top gain steps, which the reported gain does not show; each board's
own front end adds its part (the LibreSDR reads 3..6 dB lower than the
Pluto at every frequency with the same S+N/N). Measured with an HP 8642B
into RX1, -30..-120 dBm at 50.15, 70.2, 144.3, 435, 1296 and 2100 MHz
(2026-10-08); between the points by log frequency and gain, held outside.
After it, the Pluto read within +/-0.4 dB from -40 to -110 dBm at all six
(before: up to 11.4 dB low on 6 m). `trxd --sim` keeps the constant.

| dB added at the top gain step | 50 | 70 | 144 | 435 | 1296 | 2100 MHz |
|---|---|---|---|---|---|---|
| ADALM-Pluto | 11.3 | 7.8 | 3.9 | 2.2 | 0.8 | 0.7 |
| LibreSDR | 14.1 | 10.5 | 7.2 | 6.2 | 5.5 | 6.7 |
| PlutoSky R2 | 7.9 | 4.1 | 0.8 | -0.5 | -1.2 | -0.2 |

### Front-end gain per band

SET > RX FRONT-END GAIN: dB in front of the socket per band (or
transverter), an LNA positive, cable or filter negative (`settings.json`
`fe_gain`, on jffs2). It comes off the conversion whatever gives it (table,
old points or curve), so the meter and the scope labels show the level at
the LNA's input.

### Old points

The old per-band S-meter points (SET > S-METER CALIBRATION, removed) still
load and apply only where there is no table at all; a table, once
uploaded, replaces them for that socket pair.

S-units: IARU Region 1, S9 = -73 dBm below 30 MHz, -93 dBm above, 6 dB per
S unit.

The scope's level labels are dBm of a carrier (its FFT reads a tone at its
power, like the meter) with the offset at the dial's frequency, when the
socket pair has a table; dBFS otherwise.

## Web API (calibration tool)

| Command | Reply |
|---|---|
| `{"cmd":"meter_raw"}`, optionally `"lo_hz","hi_hz"` (absolute) | `meter_raw`: dial, LO, AD936x frequency, gain readback, temperature, clip, and per source (`chan`, `stream`, `maia`) `dbfs`, `peak_dbfs`, `noise_dbfs_hz`, `seq` (a new average: +1); runs the stream meter for 10 s |
| `{"cmd":"calib_get"}` | `calib`: socket pair in use, per pair its table and band status |
| `{"cmd":"calib_set","port":1,"table":{...}}` | `calib_ack` (validated: ranges, sizes), then `calib` |
| `{"cmd":"calib_clear","port":1}` | `calib_ack`, then `calib` |

Plus the existing `freq`, `mode`, `rxgain` (manual dB / slow), `port`.

## Procedure

tools/sqtrx-cal/README.md. In short: Siglent tracking generator through a
30-40 dB pad, path measured into the Siglent first, then into RX1 and RX2;
K at 40 dB gain on every band (edges and middle) and a general grid up to
3.2 GHz; gain sweeps 0-70 dB at one frequency per band with the level
stepped to stay 25 dB over the noise and under clipping. The HP generator
covers to 2.1 GHz with levels to -150 dBm (its own attenuator; cable loss
given or measured with the Siglent).
