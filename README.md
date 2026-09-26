# tezuka_fw_simple

A stripped fork of [tezuka_fw](https://github.com/F5OEO/tezuka_fw) for
**PlutoSky R2** and **LibreSDR** (Zynq-7020 + AD936x). It turns one of these
boards into:

* a **headless remote transceiver** (SQTRV): SSB, CW and data modes over
  **TCI** and **Hamlib rigctld** (FT8 is left to WSJT-X on the PC), with
  on-board Q65/PI4 decoders and a live CW decoder, published to **MQTT**, and
  low-rate **DATV** (DVB-S2 from and to the browser's camera; docs/DATV.md); or
* an **IARU-R1 MGM beacon transmitter**: PI4 or Q65-60x plus CW
  identification plus carrier, GPS/NTP-timed; or
* a **beacon receiver**: PI4, Q65 and CW decodes, plus a carrier
  frequency/SNR measurement every minute, published to MQTT.

Frequency range is that of the AD936x: about 47 MHz to 6 GHz. HF needs a transverter.

## What changed vs tezuka_fw

* FPGA: only the AD936x interface, DMA, an x8 FIR decimator/interpolator, and
  a reference-oscillator meter. Maia-SDR, DVB-S2, the sweeper and the CS8/CS12
  packers are gone: LUTs dropped from 43% to 13% on PlutoSky R2.
* Software: one daemon, `trxd`, replaces the Dashboard, Maia, DATV, SatDump,
  GNU Radio and the rest.
* Runs from QSPI flash with A/B slots and automatic rollback; updates over SSH alone.
* A built-in HTTPS web UI laid out like an IC-9700.

## Install (QSPI flash, no SD card)

The boards run from their 32 MB QSPI flash with **two firmware slots** and
automatic rollback ([docs/FLASH.md](docs/FLASH.md)). Everything below works
over Ethernet and SSH alone.

A board already running tezuka from flash:

```sh
tools/fw-push.sh --migrate -p analog <board-ip> build/plutoskyr2.zip
```

This first conversion rewrites the start of the flash once, and that write is not
protected by rollback, so keep the power stable for about half a minute.

After that, every update goes into the slot that is not running:

```sh
tools/fw-push.sh -p analog <board-ip> build/plutoskyr2.zip
```

The new slot must confirm itself within 4 minutes (network up, trxd running).
If it hasn't after three boots, for any reason including a kernel panic,
a watchdog reset or a power cycle after an early hang, U-Boot switches back to the previous slot by itself. On the board,
`fw-update --status` shows both slots and `fw-update --rollback` switches back
by hand.

(The build zip still contains `sdimg/` for SD-card boot, same A/B idea.)

## Web UI

Open `https://<board>/`. The certificate is self-signed, so accept the
browser warning once; HTTPS is what lets the browser use your microphone. The
password is `web.password` in trxd.toml, or the one generated on first
start in `/mnt/jffs2/trxd-web/password`.

The layout follows an IC-9700 front panel:
- **LCD:** a large VFO display (scroll or drag a digit to tune it), VFO A/B, split, the mode/filter/AGC tags, and an S-meter that switches to Po on TX.
- **Scope:** spectrum over waterfall, with click, drag and wheel tuning and automatic reference level. Spans from ±2.5 kHz to ±1.25 MHz; the wide ones come straight from Maia's FPGA spectrometer over the full ADC bandwidth.
- **Touch keys:** band, mode, FIL1-3, AGC, tuning step, RF/AF/drive/TX-attenuator, and 8 memories.
- **Transmit:** PTT (hold, Space, or Shift-click to latch) with your microphone, TUNE, a CW keyboard with macros, and the Q65/PI4 decode list (click to tune).

Audio is 12 kHz µ-law over the same WebSocket, 96 kbit/s each way.

**Icom RC-28:** plug it into the PC running the browser, then choose it once
under the RC-28 chip. The browser remembers it, and later visits reconnect
without asking. This uses WebHID, so it needs Chrome, Edge or Opera, over HTTPS.
- **Knob:** tunes by the current step, faster when spun quickly.
- **TRANSMIT:** hold for PTT, or tap to latch.
- **F-1 / F-2:** assignable (default: tuning step and VFO A/B). Their LEDs show
  the state; LINK lights while the radio link is up.
- Click MIC once per visit before transmitting with it. A HID button is not a
  "user gesture" for the browser's microphone permission.

## Configure

Edit `trxd.toml`. Either:

* on the USB drive the board exposes: edit, then eject. The file is checked, stored, and trxd
  restarts; `trxd-status.txt` on the drive says whether it was accepted; or
* over SSH: edit `/mnt/jffs2/trxd.toml`, then run `/etc/init.d/S80trxd restart`.

The template with every option is `package/trxd/trxd.toml`.

```toml
role = "trx"              # trx | beacon-tx | beacon-rx
callsign = "SQ6EMM"
locator = "JO81CE"
[trx]
freq_hz = 144174000
decoders = ["q65"]       # q65 | pi4
```

## Use

| Service | Where |
|---|---|
| TCI (WSJT-X, JTDX, MSHV, sdroxide, ...) | `ws://<board>:40001` |
| Hamlib | `rigctl -m 2 -r <board>:4532` (and WSJT-X "Hamlib NET rigctl") |
| MQTT broker | `<board>:1883`, WebSocket `:9001` |

MQTT topics, with `<p>` = `trxd/<hostname>`:

| Topic | Content |
|---|---|
| `<p>/online` | `true` / `false` (retained, last will) |
| `<p>/state` | frequency, mode, PTT, gains, clients, time sync (retained) |
| `<p>/decode/q65-60d`, `.../pi4`, `.../cw` | one JSON per decode: `utc`, `freq_hz`, `audio_hz`, `dt`, `snr_db`, `message`, `call` |
| `<p>/carrier` | beacon-rx: measured carrier `freq_hz`, `offset_hz`, `snr_db` each minute |
| `<p>/reference` | reference oscillator: source, measured error in ppb, `xo_correction` |
| `<p>/cmd/<name>` | commands: `freq`, `mode`, `ptt`, `tune`, `cw` (text), `cw_wpm`, `drive`, `txatt`, `rxgain`, `filter` |

```sh
mosquitto_sub -h <board> -t 'trxd/+/decode/#' -v
mosquitto_pub -h <board> -t trxd/plutoskyr2/cmd/freq -m 144050000
mosquitto_pub -h <board> -t trxd/plutoskyr2/cmd/cw -m "CQ CQ DE SQ6EMM K"
```

## Frequency reference

Put a GPS 1PPS on **EXT_IO0** (PlutoSky R2, 3.3 V) and/or a 10 MHz OCXO on
the reference input. The FPGA measures the 40 MHz reference against the PPS
and trxd trims the AD936x to it. See [docs/REFERENCE.md](docs/REFERENCE.md)
for how an OSC5A2B02 OCXO, chrony/NTP and GPS compare.

## Build

```sh
./getbuildroot.sh
./build.sh -j16 plutoskyr2      # or libre, or all
cd src/trxd && cargo test       # the daemon on a PC; `cargo run -- --sim` runs it on a simulated radio
```

Needs rustup, since `src/trxd/rust-toolchain.toml` pins Rust 1.98. The
bitstreams come from the `simple` project in the maia-sdr fork (see
CLAUDE.md). The design is described in [docs/PLAN.md](docs/PLAN.md).

## Licence

GPL-3.0 as tezuka_fw. `trxd` links sdroxide's DeepCW port and model
(AGPL-3.0), so the trxd binary is AGPL-3.0: anyone you offer its TCI or
rigctld service to over a network must be offered its source.
