# tezuka_fw_simple — plan

A stripped fork of tezuka_fw that turns a Zynq-7020/AD936x board into a
headless, remotely controlled narrowband transceiver (SSB / CW / FT8 / Q65)
and, selected by config, into an IARU-R1 MGM beacon transmitter or beacon
receiver/decoder (results to the log; MQTT was removed 2026-10-01).

Boards: `plutoskyr2`, `libre` (both xc7z020). Everything else from tezuka_fw is
removed.

## What is removed (vs tezuka_fw)

| Area | Removed |
|---|---|
| Boards | pluto, plutoplus, e200, e310, fishball*, nano, signalsdrpro, pciesdr7010, opensdrlab7010mini |
| FPGA | Maia-SDR IP (spectrometer, recorder, DDC), sweeper, cs12/cs8 packers, DVB-S2 encoder, RRC, source/dest/fir switches, uartlite, iqburst |
| Packages | DATV, maia-httpd/kmod/wasm, rust-wasm, wasm-pack, futuresdr, satdump, gnuradio (sdonly), gr-*, srt, libgse, pluto_stream, nng, iio_ws, classifier, kalibrate, fl2k, soapyplutosdr, rxtools, sdr_usb_gadget, mqtt_pubd, hamlib, civetwebws, tezuka_tools(_mini) |
| Rootfs | Dashboard, IQEngine, Maia overlay, DATV overlay, api_controller.sh, sweep/LNB/classifier/bandplan scripts, NFS |

Kept: ADI kernel + u-boot, libiio/iiod (network backend, handy for
debugging), dropbear, avahi, gpsd + chrony (beacon timing),
wireguard, SD-boot image flow, `board-fpga`.

## FPGA (`maia-sdr` fork, `maia-hdl/projects/simple`)

```
AD9361 LVDS -> axi_ad9361 -> adc_fifo -> [x8 FIR decimator, bypassable] -> [libre: XO NCO] -> cpack -> ADC DMA -> DDR
DDR -> DAC DMA -> upack -> [x8 FIR interpolator, bypassable] -> [libre: XO NCO] -> dac_fifo -> axi_ad9361
```

* x8 decimator / interpolator are ADI's `ad_add_{de,inter}polation_filter`
  (same as ADI's own Pluto design). Enabled by `up_adc_gpio_out[0]` /
  `up_dac_gpio_out[0]` (axi_ad9361 regs `0x790200BC` / `0x790240BC`).
  With the AD9361 at 1.536 MS/s the ARM sees 192 kS/s IQ.
* plutoskyr2: ADF4001 10 MHz reference init + auto-detect (`system_top.v`), RGMII-over-EMIO.
* libre: `vctcxo_lock` (10 MHz / PPS VCTCXO discipline) + `iq_xo_corrector` NCO.

## Software: `trxd` (Rust, `src/trxd`)

One daemon, role selected in `/mnt/jffs2/trxd.toml` (default copy on the USB
mass-storage `config.txt` style path is `/etc/trxd.toml`):

* `role = "trx"` — remote transceiver
  * IIO: AD9361 LO / gains / rate via sysfs, RX/TX streaming via libiio local backend.
  * DSP: 192 kS/s IQ -> NCO + decimate -> 48 kS/s -> SSB/CW/DIG demod, AGC;
    TX: 48 kS/s audio -> SSB (Weaver/phasing) or CW keyer -> interpolate to 192 kS/s.
  * **TCI server** (WebSocket, port 40001): CAT + RX audio + TX audio + IQ.
  * **rigctld server** (TCP 4532, Hamlib `rigctl -m 2` compatible).
  * **Decoders** (on-board, results to the web UI and TCI `spot:`):
    FT8 (mfsk-core), Q65-60A..E (mfsk-core), CW skimmer (DeepCW Conformer-CTC, rten).
* `role = "beacon-tx"` — GPS/NTP-timed MGM beacon:
  even minute digital mode (PI4 / Q65-60x) + carrier, odd minute CW + carrier
  (same cycle as MGMBeacon.ino / BeaconModes).
* `role = "beacon-rx"` — fixed frequency, records each minute, decodes
  PI4 / Q65 / CW, measures carrier SNR + frequency offset, writes both to the log.

## Constraints / honest limits

* AD936x tunes 70 MHz..6 GHz (tezuka extends to ~47 MHz). HF needs a transverter.
* Cortex-A9 @ 766 MHz: DeepCW inference is the heaviest part — a handful of
  concurrent CW channels, not a 100-signal skimmer. FT8/Q65 decoding once per
  period is fine.
* Beacon timing needs a GPS (gpsd + PPS) or NTP; PlutoSkyR2/Libre take a
  10 MHz reference for frequency accuracy.

## FPGA offload (phase 7, `trx_core` IP)

Everything per-sample moves to the fabric; everything branchy stays on ARM.

| Block | Where | Notes |
|---|---|---|
| RX DDC: NCO + CIC + comp. FIR, 3.072 MS/s -> 48 kS/s | FPGA | ARM sees 48 kS/s (64x less than raw) |
| TX DUC: 48 kS/s -> 3.072 MS/s + NCO | FPGA | |
| Beacon sequencer: phase-continuous FSK/CW NCO, tones in BRAM, start on PPS edge | FPGA | sample-exact, GPS-locked symbol timing; ARM arms it once a minute |
| Skimmer channelizer | FPGA (optional) | only if CPU runs short |
| SSB/CW demod, AGC | ARM | a few % of a core |
| FT8 / Q65 / PI4 decoding | ARM | search + LDPC/Fano, once per period |
| DeepCW Conformer inference | ARM | no inference IP for Zynq-7000; this bounds CW slots |

trxd detects the core (ID register) and falls back to the software path of
phases 3-5, which stays the reference implementation.

## Firmware updates over the network

SD card layout: `BOOT.bin` (FSBL + U-Boot), `uEnv.txt`, and two slots `a/`, `b/`
(kernel, ramdisk, dtb, bitstream, VERSION, SHA256SUMS). U-Boot's `slot_select`
(in `uboot-env.txt`, re-imported from `uEnv.txt` each boot) picks the slot from
the QSPI env and counts unconfirmed boots; `fw-update` writes the inactive slot
and `S99fwconfirm` confirms it once network + trxd are up. Three unconfirmed
boots roll back. Host side: `tools/fw-push.sh <ip> build/<board>.zip`.

## Phases

1. Strip firmware tree, two defconfigs. **Done.**
2. Minimal FPGA project (`simple`), bitstreams for both boards. **Done**: timing met;
   PlutoSky R2 LUTs 43% -> 13%.
3. trxd core: IIO, DSP, rigctld, TCI, simulated radio (MQTT removed 2026-10-01). **Done** (host-tested).
4. Decoders: FT8, Q65-60A..E, PI4, DeepCW skimmer. **Done** (round-trip tests; the
   skimmer needed VHF+ CW segments in sdroxide, see CLAUDE.md).
5. Beacon TX / RX roles. **Done** (TX->RX loopback tests for Q65, PI4, CW, carrier).
6. Reference disciplining: FPGA `refmeter` (1PPS / chrony), `xo_correction`. **Done**.
7. Buildroot package, init scripts, A/B network updates. **Done**; image build in progress.
8. On-hardware bring-up. **Not done** (no board access yet). First things to check:
   IIO buffer write() on the DAC, FPGA decimator bit, TX spectrum, PPS on EXT_IO0.
9. FPGA offload (table above): DDC/DUC to 48 kS/s, PPS-armed beacon sequencer.
   **Next**, once 8 confirms the software path.
