# tezuka_fw_simple

**SQTRX** by SQ6EMM/Claude (based on
[tezuka_fw](https://github.com/F5OEO/tezuka_fw)/[maia-sdr](https://github.com/F5OEO/maia-sdr)
by F5OEO).

A stripped fork of [tezuka_fw](https://github.com/F5OEO/tezuka_fw) for
**PlutoSky R2** and **LibreSDR** (Zynq-7020 + AD936x). It turns one of these
boards into:

* a **headless remote transceiver** (SQTRX): SSB, CW and data modes over
  **TCI** and **Hamlib rigctld** (FT8 is left to WSJT-X on the PC), with
  on-board Q65/PI4 decoders and a live CW decoder (web UI and TCI), and
  low-rate **DATV** (DVB-S2 from and to the browser's camera; docs/DATV.md),
  FreeDV **RADE V2** voice in the browser (docs/RADE.md), and receive-only
  bands (airband AM, marine and PMR/LPD FM, broadcast wide FM with RDS); or
* an **IARU-R1 MGM beacon transmitter**: PI4 or Q65-60x plus CW
  identification plus carrier, GPS/NTP-timed; or
* a **beacon receiver**: PI4, Q65 and CW decodes, plus a carrier
  frequency/SNR measurement every minute, written to the log.

Frequency range is that of the AD936x: about 47 MHz to 6 GHz. HF needs a transverter.

## Firmware flavours

Five images from one tree (docs/FLAVOURS.md):

| | LibreSDR | PlutoSky R2 | ADALM-Pluto |
|---|---|---|---|
| **BASIC**: narrowband SQTRX (SSB/CW/data, Q65/PI4, CW decoders, TCI/rigctld, dBm meter, beacons) | `libre-basic` | `plutoskyr2-basic` | `pluto-basic` |
| **BASIC+**: BASIC plus DATV (DVB-S2/T2 transmit and receive in the FPGA) | `libre-plus` | `plutoskyr2-plus` | not available (xc7z010 too small) |

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
| Web UI | `https://<board>/` |

There is no MQTT (removed 2026-10-01: it was an unauthenticated control
path). Decodes, beacon-rx carrier reports and the reference state go to the
log (`logread`, or `/var/log/messages`); in the trx role decodes also go to
the web UI and to TCI.

## DATV quality

Libre transmitting on 1255 MHz, as a Siglent SVA1032X analyser with a small
antenna decodes it. DVB-S2 QPSK 1/2 at 250 kS/s (EVM 0.86 %, MER 41 dB) and
8PSK 3/4 at 250 kS/s (EVM 1.19 %, MER 38.5 dB):

<p>
<img src=docs/images/siglent/dvbs2-qpsk-12-250k.png width=49%>
<img src=docs/images/siglent/dvbs2-8psk-34-250k.png width=49%>
</p>

DVB-T2 1.7 MHz QPSK 1/2, spectrum only (the analyser has no OFDM
demodulator):

<img src=docs/images/siglent/dvbt2-qpsk-12-1m7.png width=49%>

All rates and modes, with their figures: [docs/DATV-QUALITY.md](docs/DATV-QUALITY.md).

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

## Credits and software used

Firmware and FPGA:
- [tezuka_fw](https://github.com/F5OEO/tezuka_fw) by F5OEO (Evariste
  Courjaud): the Buildroot tree this is forked from (fork:
  [sq6emm/tezuka_fw](https://github.com/sq6emm/tezuka_fw), branch `simple`).
- [maia-sdr](https://github.com/maia-sdr/maia-sdr) by Daniel Estevez
  (EA4GPZ), in F5OEO's fork [F5OEO/maia-sdr](https://github.com/F5OEO/maia-sdr):
  the FPGA project (Amaranth HDL), spectrometer, DDC and DMA; the `simple`
  and `datv-ddc` work is in [sq6emm/maia-sdr](https://github.com/sq6emm/maia-sdr).
- [Analog Devices HDL](https://github.com/analogdevicesinc/hdl) (AD9361
  interface, DMA), [ADI Linux](https://github.com/analogdevicesinc/linux) and
  [ADI u-boot-xlnx](https://github.com/analogdevicesinc/u-boot-xlnx).
- [Buildroot](https://buildroot.org/) with a
  [Bootlin toolchain](https://toolchains.bootlin.com/); on the board:
  [BusyBox](https://busybox.net/), [Dropbear](https://matt.ucc.asn.au/dropbear/dropbear.html),
  [chrony](https://chrony-project.org/),
  [pps-tools](https://github.com/redlab-i/pps-tools),
  [Avahi](https://avahi.org/), [libgpiod](https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git/),
  [WireGuard tools](https://www.wireguard.com/), [mtd-utils](https://git.infradead.org/mtd-utils.git),
  [iproute2](https://git.kernel.org/pub/scm/network/iproute2/iproute2.git),
  [nano](https://www.nano-editor.org/).
- AMD/Xilinx Vivado 2023.1 builds the bitstreams.

trxd (Rust):
- [sdroxide](https://github.com/dividebysandwich/sdroxide) (fork with the
  crates trxd pins: [sq6emm/sdroxide](https://github.com/sq6emm/sdroxide)):
  DSP chain, TCI and rigctld servers, the DeepCW port.
- [DeepCW](https://github.com/e04/deepcw-engine): the neural CW decoder and
  its trained model, run with [rten](https://github.com/robertknight/rten).
- [mfsk-core](https://github.com/jl1nie/mfsk-core): Q65 (the mode by K1JT,
  [WSJT-X](https://wsjt.sourceforge.io/)).
- PI4: [the OZ2M / DJ5HG specification](https://rudius.net/oz2m/ngnb/pi4_.htm),
  implemented in trxd.
- DVB-S2 and DVB-T2 tables and the T2 modulator's structure:
  [GNU Radio gr-dtv](https://github.com/gnuradio/gnuradio/tree/main/gr-dtv);
  DVB-S2 checked against [leansdr / leandvb](https://github.com/pabr/leansdr).
  The DVB-T2 profile follows
  [Portsdown 4](https://github.com/BritishAmateurTelevisionClub/portsdown4)
  and the [BATC](https://batc.org.uk/) receivers.
- Crates: [RustFFT](https://github.com/ejmahler/RustFFT),
  [num-complex](https://github.com/rust-num/num-complex),
  [rustls](https://github.com/rustls/rustls), [rcgen](https://github.com/rustls/rcgen),
  [tungstenite](https://github.com/snapview/tungstenite-rs),
  [crossbeam](https://github.com/crossbeam-rs/crossbeam),
  [rayon](https://github.com/rayon-rs/rayon), [serde](https://serde.rs/),
  [toml](https://github.com/toml-rs/toml), [tracing](https://github.com/tokio-rs/tracing),
  [lzma-rs](https://github.com/gendx/lzma-rs),
  [RustCrypto sha2](https://github.com/RustCrypto/hashes),
  [getrandom](https://github.com/rust-random/getrandom),
  [libc](https://github.com/rust-lang/libc); the full list with versions is
  `src/trxd/Cargo.lock`.
- The web UI has no third-party code; the browser supplies H.264/Opus
  encoding ([WebCodecs](https://www.w3.org/TR/webcodecs/)) and the RC-28
  link ([WebHID](https://wicg.github.io/webhid/)).
- DATV audio: [FFmpeg](https://ffmpeg.org/)'s libavcodec (LGPL-2.1+), a
  minimal static build with the Opus decoder and the AAC encoder only
  (`package/ffmpeg-aac`), turns the browser's Opus into AAC-LC on the board.

## Licence

GPL-3.0 as tezuka_fw. `trxd` links sdroxide's DeepCW port and model
(AGPL-3.0), so the trxd binary is AGPL-3.0: anyone you offer its TCI or
rigctld service to over a network must be offered its source.
