# Frequency reference: OCXO vs chrony vs GPS PPS

The AD936x derives every LO and the sample clock from one 40 MHz oscillator.
Whatever that oscillator does, the signal does, multiplied up to RF:
1 ppb is 1.3 Hz at 1296 MHz and 10.4 Hz at 10368 MHz.

## What each option can and cannot fix

Two separate things matter:

* **Accuracy**: the long-term average frequency, i.e. where a beacon lands on the dial.
* **Stability**: how much the frequency moves over 1 s to 100 s. This is what
  decides whether a Q65/PI4 symbol or a CW note stays in its bin.

Software correction (`xo_correction`) moves the *average*. It cannot remove
wander faster than its own averaging window, and it cannot touch phase noise.
Only a better oscillator improves stability.

| Setup | Accuracy (long term) | Stability (1-100 s) | @1296 MHz | @10368 MHz |
|---|---|---|---|---|
| Board TCXO, free running | about ±1-2 ppm, plus drift as the board warms (typical TCXO) | TCXO class; tens of ppb when the temperature moves (TX heating) | ±1.3-2.6 kHz | ±10-20 kHz |
| TCXO + **chrony over internet NTP** (`reference.mode = "chrony"`) | 5-50 ppb after a 30 min fit (time noise 0.1-1 ms; see below) | unchanged (TCXO) | ±6-65 Hz, plus thermal wander | ±50-500 Hz |
| TCXO + **chrony over LAN NTP / GPS-fed chrony** | 1-5 ppb | unchanged (TCXO) | ±1-6 Hz, plus thermal wander | ±10-50 Hz |
| TCXO + **GPS 1PPS into the FPGA refmeter** (`"pps"`) | about 1-5 ppb, updated every 64 s, so it follows thermal drift | TCXO class below 64 s | ±1-6 Hz | ±10-50 Hz |
| **OSC5A2B02 OCXO as external 10 MHz** (R2: ADF4001 lock; Libre: vctcxo_lock) | ±10 ppb (spec) and ages with time | OCXO class: about 1e-11 to 1e-10, flicker floor near τ = 500 s | ±13 Hz | ±104 Hz |
| **OCXO + GPS 1PPS refmeter** (both at once) | ≤ 1 ppb | OCXO class | < 1.3 Hz | < 10 Hz |
| Libre VCTCXO + PPS, hardware PI loop (`vctcxo_lock`) | about 1 ppb, the oscillator itself steered | VCTCXO class | ~1 Hz | ~10 Hz |

Sources and confidence:

* OSC5A2B02 (CTI surplus OCXO): ±10 ppb stability, 5 V at 600 mA during warm-up and
  250 mA once stable, EFC −1..+2 ppm
  ([NTMS / AA5C](https://www.ntms.org/files/Mar2022/AA5C_LowCost_10MHz_Reference-1.pdf),
  [EasyEDA part page](https://easyeda.com/components/CTI-OSC5A2B02-10Mhz-OCXO_ea1ce85e0ab7468eb8857e8e97ff70f8)).
  The Allan-deviation shape, "noisier than an Oscilloquartz, still under the 1 ppb
  QO-100 needs, flicker floor near τ = 500 s", is from
  [PA1EJO's measurements](https://pa1ejo.wordpress.com/category/electronics/).
  I found no aging figure, so plan on recalibrating it.
* TCXO and NTP numbers are typical values for the component class, not measured on these boards.
* The chrony rows follow from the fit in `refclock.rs`. The error of a frequency
  estimate from timestamps is about `σ_t / (T · √(n/12))`, with σ_t the clock's time noise,
  T the window and n the sample count. The unit test
  `time_fit_error_scales_with_clock_noise` exercises exactly this.

## Answers

1. **Can chrony adjust the reference?** Not directly. chrony only steers
   the Linux system clock. trxd closes the loop itself: the FPGA `refmeter`
   counts the 40 MHz reference against chrony-disciplined time (or a PPS) and
   writes the result to the AD936x `xo_correction`. That makes the LO and
   sample rate correct *on average*.
2. **OCXO vs chrony/NTP.** They fix different things. The OCXO gives
   short-term stability and ±10 ppb out of the box. Internet NTP gives only
   slow average accuracy, typically *worse* than the OCXO's ±10 ppb, and does
   nothing for stability. Over NTP the OCXO beats chrony disciplining on both
   counts. NTP is useful with an OCXO only for tracking its aging over days
   to weeks (a day-long fit with 1 ms of noise is about 10 ppb; a week, about 2 ppb).
3. **Best setup.** Use an OCXO for stability plus GPS 1PPS for accuracy. On PlutoSky R2,
   connect the OCXO's 10 MHz to the REF input (the ADF4001 locks the VCTCXO to it)
   and the GPS PPS to **EXT_IO0 (JP5, 3.3 V)**. The refmeter then measures the
   OCXO-locked clock against GPS and trims `xo_correction`: a software GPSDO.
   On Libre, feed 10 MHz or PPS and let `S22gpsdo` steer the VCTCXO through
   `vctcxo_lock` in hardware.

## Without GPS (the default firmware)

The firmware ships without gpsd: time comes from NTP (chrony), and
`reference.mode = "auto"` then uses the chrony-disciplined clock. Recommended:

* **OSC5A2B02 (or any OCXO) as the external 10 MHz**: this sets stability and
  gives ±10 ppb out of the box (±13 Hz at 1296 MHz).
* Keep `auto` / `chrony`: over a 30 min window it will only nudge the
  correction by the OCXO's residual error. It stays within `min_step_hz` most of
  the time, so it rarely retunes, and it tracks the OCXO's aging over weeks.
  With a LAN NTP server instead of the public pool the estimate gets several
  times better.
* A 1PPS can still be added later on EXT_IO0 (R2) without a firmware change
  (the refmeter and `/dev/pps0` are there). For chrony to use it, add a
  `refclock PPS` line (see `/etc/chrony.conf`).

## Configuration (`[reference]` in trxd.toml)

```toml
[reference]
mode = "auto"          # auto | pps | chrony | 10mhz | off
pps_window_s = 64      # PPS averaging per estimate
chrony_window_s = 1800 # snapshot fit window without PPS
min_step_hz = 0.2      # rewrite xo_correction only past this (Hz at 40 MHz = 5 ppb)
```

`auto` uses PPS when the refmeter sees pulses, otherwise chrony time once the
kernel reports the clock synchronised. On Libre the hardware loop belongs to
the board's `S22gpsdo` (`gpsdo_boot.sh`: reference choice, calibrated DAC
centre); trxd only reports it there, and corrects in software only with
`mode = "chrony"`. On R2, `S22refclk` still picks the ADF4001 source. Status goes to the log (`reference`) every 5 minutes:
source, measured frequency, error in ppb, applied `xo_correction`, and the
Libre lock bit.

Corrections retune the synthesizers for a few milliseconds. The beacon
transmitter accepts them only between seconds 56 and 58.5 of the minute,
which is always carrier.

## Hardware notes

* PlutoSky R2 EXT_IO0 is FPGA pin Y18, bank 33, VCCO 3.3 V. A GPS module's
  3.3 V PPS connects directly. For a 5 V PPS, add a divider.
* The same PPS is routed to EMIO GPIO 52 (Linux gpio 106). The device tree
  adds a `pps-gpio` node, so `/dev/pps0` exists and chrony can use it as a
  refclock alongside gpsd's NMEA.
