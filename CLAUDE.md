# CLAUDE.md

## Communication
- Always respond in English.

## Project overview

**tezuka_fw_simple** is a stripped fork of tezuka_fw (a Buildroot
`BR2_EXTERNAL` tree for Zynq-7000/AD936x SDRs), turning a PlutoSky R2 or
LibreSDR into a headless remote transceiver (TCI + rigctld, SSB/CW/data, Q65/PI4,
CW decoders) or an IARU-R1 MGM beacon transmitter/receiver. See `docs/PLAN.md` for architecture, `docs/REFERENCE.md` for the
frequency-reference design and `docs/DATV.md` for DVB-S2 video (trxd
`src/dvbs2/`: TX, RX, TS mux/demux; web UI DATV panel) and `docs/DATV-FPGA.md`
for its FPGA front end (Maia DDC + ring DMA, maia-sdr branch `datv-ddc`); `docs/DATV-OTA.md` has every DATV mode over the air on every band.
`docs/RADE.md`: RADE V2 digital voice, run in the browser (rade_c built to
WASM in `src/rade-web/`, the module in the `model` flash partition).

Boards: `plutoskyr2`, `libre` (both xc7z020). `boards.json` is the single
source of truth for `build.sh` and CI.

## Layout

| Path | Purpose |
|---|---|
| `configs/<board>_defconfig` | Buildroot defconfigs |
| `board/tezuka/common/` | Shared kernel/u-boot config, overlays (`overlay_base`, `overlay_tezuka`), image scripts |
| `board/tezuka/<board>/` | DTS, u-boot DTS, `bitstream/simple-<mode>.xsa` + `boot-mode`, board overlay |
| `package/board-fpga` | Extracts `system_top.bit` from the boot mode's `bitstream/simple-<mode>.xsa`; the other modes go to `/lib/firmware` |
| `package/trxd` | Buildroot package + init script + default config for trxd |
| `src/trxd` | The daemon (Rust) |
| `src/rade-web` | RADE V2 for the page: WASM wrapper, Docker build, Node round-trip test |
| `docs/` | Design notes |

## Build

```bash
./getbuildroot.sh            # once; or hardlink an existing buildroot/ tree
./build.sh -j16 plutoskyr2   # or libre, or all
```

Output: `output/<board>/images/`, `build/<board>.zip`: `flash/` (A/B QSPI:
`firmware.itb`, `model.bin`, `uboot-ab.env`, `SHA256SUMS`; the build fails if
the FIT exceeds a 12.25 MB slot) and `sdimg/` (SD boot, same A/B idea).
Flash layout, update and rollback: docs/FLASH.md. Install/update over SSH:
`tools/fw-push.sh [--migrate] <ip> build/<board>.zip` -> `/usr/sbin/fw-update`.

trxd is built by rustup's cargo pinned by `src/trxd/rust-toolchain.toml`
(Buildroot 2026.02's Rust 1.88 is too old for rten/kstring). The package
passes `--locked`, so `src/trxd/Cargo.lock` must be committed and current.

### trxd on a PC

```bash
cd src/trxd
cargo test                       # 40+ tests incl. Q65/PI4/CW round trips
cargo run --release -- --sim     # simulated radio; TCI :40001, rigctld :4532
```

`--sim` loops TX back into RX and adds a keyed CW test signal. The web UI
(`src/trxd/web/index.html`, compiled in; server `src/trxd/src/web.rs`) is
reachable at https://127.0.0.1:<web.https_port>/ when running the sim - set
`[web] https_port = 8443, http_port = 0, password = "...", state_dir = <tmp>`.
Headless-Chrome check of the UI: drive Chrome over CDP (`--remote-debugging-port`,
`--ignore-certificate-errors`, `--use-fake-device-for-media-stream`).

Cross build as
in `package/trxd/trxd.mk` (target `armv7-unknown-linux-gnueabihf`, linker =
the board output's `host/bin/arm-linux-gcc`).

### sdroxide dependency

trxd reuses sdroxide crates (DSP, TCI server, rigctld server)
pinned by git rev in `src/trxd/Cargo.toml`. A `[patch]` section currently
points all of them at the local worktree `../sdroxide-vhf` (branch
`vhf-cw-segments`, uncommitted): VHF+ CW segments (the skimmer never spotted
above 30 MHz without them). Remove it once that branch is pushed and bump
the rev.

## FPGA

Bitstreams come from the `simple` project in the maia-sdr fork
(`/home/dawszy/git/my/maia-sdr`, branch `simple`,
`maia-hdl/projects/simple`): AD9361 + DMA + x8 FIR decimator/interpolator,
`refmeter` (reference counter vs 1PPS / software snapshots, 0x43C10000), and
Maia's spectrometer only (`maia_scope.tcl`, 0x7C460000, ring at
0x16000000 - the web UI's wide scope, read by `src/trxd/src/maia.rs`), and
on Libre `vctcxo_lock` + `iq_xo_corrector`.

```bash
cd /home/dawszy/git/my/maia-sdr/maia-hdl/projects/simple
source /home/dawszy/git/my/maia-sdr/sourceme.local
FPGA_MODE=trx PROJECT_NAME=plutoskyr2 make all   # installs board/tezuka/plutoskyr2/bitstream/simple-trx.xsa here (docs/FPGA-MODES.md)
```

Pass `PROJECT_NAME` in the environment, not as a make argument (ADI's
makefile turns command-line variables into a sub-directory). Each build starts
with `rm -rf *.log *.runs ...` in the project directory, so keep build logs
elsewhere, and build boards one after another.

Register map / control bits trxd relies on:
- `0x790200BC` / `0x790240BC` bit 0: RX decimator / TX interpolator enable.
- `0x43C10000` refmeter (see `refmeter.v` header).
- PlutoSky R2 EXT_IO0 (Y18, bank 33 = 3.3 V) = GPS 1PPS -> refmeter and EMIO
  52 (gpio 106, `pps-gpio` -> `/dev/pps0`).

## Important constraints

- Never read, grep, or glob inside `buildroot/` (third-party tree).
- U-Boot boot scripts (`board/tezuka/common/uboot-env.txt`: ab_count,
  slot_select, qspi_slot_select, qspiboot) can be exercised in U-Boot sandbox
  (build `sandbox_defconfig` from output/<board>/build/uboot-custom, import the
  file with `host load hostfs` + `env import -t`). Do that after any change:
  a broken script strands a remote board at the U-Boot prompt.
- Do not modify the board scripts `S22refclk` (R2 ADF4001 source) or
  `S22gpsdo`/`gpsdo_boot.sh` (Libre vctcxo_lock acquisition) from trxd; trxd
  only reports on Libre's hardware loop.
- Verify before calling a task done: `cargo test` for trxd; for images, a
  complete `make` and a sane image size; for DTS changes, `dtc -I dtb -O dts`.

## Git
- Do not mention Claude as author or co-author in commit messages.
- Stage only files you changed. Commit/push only when asked.
- ASCII, no embedded double-quotes.
