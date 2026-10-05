# Running from QSPI flash, with A/B slots

The boards boot from their 32 MB QSPI flash. No SD card is needed. The
firmware keeps two complete copies ("slots") so that an update over SSH can
never leave a remote board without a working system.

## Layout

| Offset | Size | Label | Contents |
|---|---|---|---|
| `0x0000000` | 1 MB | `qspi-fsbl-uboot` | FSBL + U-Boot. Never written by updates. |
| `0x0100000` | 128 KB | `qspi-uboot-env` | U-Boot environment: slot choice, boot counter, the A/B boot scripts. |
| `0x0120000` | 896 KB | `qspi-nvmfs` | jffs2: `/mnt/jffs2` (trxd.toml, web certificate and password, keys). |
| `0x0200000` | 12.25 MB | `linux-a` | Slot A: one FIT image (kernel lzma, device tree, bitstream gzip, rootfs xz). |
| `0x0E40000` | 12.25 MB | `linux-b` | Slot B: same. |
| `0x1A80000` | 5.5 MB | `model` | The web page's RADE V2 module (docs/RADE.md), shared by both slots; `fw-update` writes it when it changed. It held the DeepCW model until 2026-10-04. |

Slot A sits where tezuka's single image used to start. A board whose U-Boot
environment has no A/B scripts therefore still boots slot A.

### Size budget (per slot)

| Part | Size |
|---|---|
| Kernel, `Image.lzma` | ~4.6 MB |
| Bitstream (LibreSDR: the `trx` mode's), gzip | ~0.7 MB |
| Rootfs, xz with the ARM filter (trxd ~2.6 MB with the web UI and the CW-RS network; the `datv` bitstream 1.1 MB) | ~6.5 MB |
| **FIT total** | **must be ≤ 12.25 MB**; the build fails if not. LibreSDR 2026-09-29: 11.85 MB (990 KB free) |

This fits because of three cuts:
- **DeepCW model:** moved to its own partition and bf16-rounded, 13.4 MB → 5.3 MB compressed. The accuracy matrix is unchanged.
- **Kernel:** shipped as lzma instead of zImage, saving 1.6 MB and avoiding tezuka's zImage corruption bug #450.
- **Rootfs:** trimmed of Wi-Fi drivers and firmware, DRM, the Fortran/OpenMP/C++ runtimes, CA certificates, libiio/iiod and the gpsd client tools.

2026-09-29, 730 KB more without dropping anything (FPGA mode bitstreams and
the larger CW-RS network had left 265 KB):
- the rootfs compressed again with xz's ARM branch filter
  (`postimage-qspi.sh`; the kernel's XZ decoder has it): 250 KB;
- trxd with fat LTO and one codegen unit: 110 KB (and a little faster);
- the CW-RS network as RSQ3 (`trxd --pack-rsnn`: 16-bit weights, 32-bit
  biases, the fixed-point path's own values, so the outputs are the same
  bit for bit): 386 KB;
- the boot bitstream once only: fpga-mode takes it out of the running
  slot's FIT (docs/FPGA-MODES.md).
Left if more is needed: NFS client (about 130 KB), USB host network and HID
drivers (about 140 KB): both optional features today.

## Updating: `tools/fw-push.sh`

```sh
tools/fw-push.sh -p analog <board-ip> build/plutoskyr2.zip
```

On the board, `fw-update` does the following:

1. Checks the build's own `flash/SHA256SUMS`, the slot size, and that the
   device tree inside is for this board model.
2. Writes `firmware.itb` into the slot that is **not** running, reads it back
   and compares byte for byte.
4. In one write of the U-Boot environment: installs the A/B boot scripts
   shipped with the build (`uboot-ab.env`), sets `slot=<new>`,
   `upgrade_available=1`, `bootcount=0`.
5. Reboots.

U-Boot counts every boot of an unconfirmed slot. `iminfo` verifies every
image hash in the slot before booting. A failure resets, which counts, and so
does a kernel panic (`panic=10`). The boot script starts the Zynq system
watchdog before it reads the slot (`wdt_start`, about 160-190 s), so a hang
while loading, in `bootm` or in the kernel before `S15watchdog` takes the
watchdog over also resets the board and counts. (U-Boot itself has no
watchdog support compiled in, and it is never rewritten, so the watchdog is
started from the environment's script, with `mw`; a hang in U-Boot before the
script runs is not covered.) An interrupted autoboot does not start it, so
work at the U-Boot prompt or a manual `run dfu_sf` is not reset.
`S99fwconfirm` confirms the slot once the uplink has an address, dropbear
runs and the same trxd process has stayed up for 20 s, which clears
`upgrade_available`. The uplink is not `usb0` (it always has its static
192.168.2.1, even with Ethernet broken): each healthy boot records in
`/mnt/jffs2/fw-uplinks` which other interfaces had an address, and a new slot
must bring up one of them. An empty record means a USB-only board, where
`usb0` counts. The fourth unconfirmed boot switches back to the previous slot
automatically and sets `rolled_back=1`.

A **confirmed** slot that fails to load (a flash read error, a damaged
image) no longer drops to USB DFU: U-Boot boots the other slot, for that boot
only, with `UBOOT_SLOT=<other> UBOOT_FALLBACK=1` on the kernel command line.
Nothing is saved: `slot=` still names the failed slot, `fw-update --status`
and the console say so, and the next `fw-update` writes over the failed slot.
DFU is entered only when both slots fail (the watchdog then resets the board
after about three minutes and it tries again).

### The U-Boot environment has one copy

A power cut while the environment is written (`fw-update`, `fw_setenv`, or
U-Boot counting the boots of a new slot) leaves it with a bad CRC. U-Boot
then uses its built-in defaults, which boot slot A, and the A/B scripts, the
boot counter and the network settings are gone.

A redundant environment (`CONFIG_SYS_REDUNDAND_ENVIRONMENT`, two sectors with
a flags byte) would avoid that, but it is a U-Boot build option, and U-Boot is
never rewritten by updates: boards in the field keep the single-copy U-Boot
they have. The rootfs's `fw_env.config` must match the U-Boot on the board;
a two-copy `fw_env.config` against a single-copy U-Boot makes every Linux
write unreadable for U-Boot (bad CRC, defaults, slot A). So it is not used.

Instead `S20uenv` (after jffs2 is mounted, before the network starts) keeps
a copy: on every boot of a confirmed slot it saves the environment to
`/mnt/jffs2/uboot-env.bak` when it changed, and on a bad CRC it writes that
copy back, with `slot=` the slot running now, nothing pending, and
`env_restored=1` (logged in `fw-update.log`, shown by `fw-update --status`).
The network settings are then back before `S40network` reads them. Tested
with the real `fw_printenv`/`fw_setenv` against a file-backed environment:
good, unchanged, pending (no backup taken), damaged (restored), erased with
no copy.

On the board:

```sh
fw-update --status      # slots, versions, next boot, rollback flag
fw-update --rollback    # go back to the other slot by hand
```

## First install on a board running tezuka from flash

```sh
tools/fw-push.sh --migrate -p analog <board-ip> build/plutoskyr2.zip
```

tezuka keeps one ~30 MB image across the whole region, so there is no free
place to stage the new one. The conversion therefore writes slot A over the old
image, verifies it, then writes the model and the environment. **This one write
is not protected by rollback**: if power fails during those ~30 s, the board
needs USB DFU, JTAG or an SD card to recover. U-Boot itself is never touched,
so USB DFU recovery keeps working. Every later update goes to the other slot
and is protected.

The U-Boot boot scripts were run in U-Boot's sandbox build for every state
(fresh, pending, counting, rollback, failed load; 2026-10-01 also: confirmed
slot failing with the other slot good, both failing, the kernel command line
of a fallback boot, and the watchdog register values). The `fw-update` flash
writes were tested against simulated MTD devices for both layouts. Neither
has run on a real board yet.

## Images and flavours

Five images (docs/FLAVOURS.md), each built by `./build.sh`:

| Image | Build | Zip | Buildroot output |
|---|---|---|---|
| LibreSDR BASIC+ | `./build.sh -j16 libre-plus` (or `libre`) | `build/libre-plus.zip` (also `build/libre.zip`) | `output/libre` |
| LibreSDR BASIC | `./build.sh -j16 libre-basic` | `build/libre-basic.zip` | `output/libre-basic` |
| PlutoSky R2 BASIC+ | `./build.sh -j16 plutoskyr2-plus` (or `plutoskyr2`) | `build/plutoskyr2-plus.zip` (also `build/plutoskyr2.zip`) | `output/plutoskyr2` |
| PlutoSky R2 BASIC | `./build.sh -j16 plutoskyr2-basic` | `build/plutoskyr2-basic.zip` | `output/plutoskyr2-basic` |
| ADALM-Pluto BASIC | `./build.sh -j16 pluto-basic` (or `pluto`) | `build/pluto-basic.zip` | `output/pluto-basic` |

`./build.sh all` builds all five (about 6 GB of output each, under /data).
A flavour is a fragment appended to the board's defconfig
(`configs/flavour/basic.config`, `plus.config`): BASIC installs only the
`trx` bitstream (`BR2_PACKAGE_BOARD_FPGA_MODES="trx"`), BASIC+ every mode
the board has (`trx`, `datv`). The flavour is written to `/etc/fw-flavour`
and into the image's `VERSION` (e.g. `v0.3.21-60-g1234 basic`) and
`flash/FLAVOUR`. On a BASIC image trxd reports `features.datv = false`:
the web UI hides DATV and trxd refuses DATV commands.

**Flashing** is the same for all five, one board at a time:

```sh
tools/fw-push.sh <board-ip> build/libre-basic.zip
```

`fw-update` checks that the image's device tree is for the running board
(the LibreSDR, PlutoSky R2 and ADALM-Pluto models differ, so an image for
another board is refused). **Changing flavour** on the same board is a
normal update, with rollback like any other: `fw-update` logs
`flavour changes: plus -> basic (settings are kept)`. Nothing in
`/mnt/jffs2` (trxd.toml, web settings, calibration, keys) is touched.

### ADALM-Pluto

* Same 32 MB layout as the other boards: two 12.25 MB slots, the 5.5 MB
  model partition, rollback included. The FIT has the three Pluto device
  trees (Rev A, B, C); U-Boot's `adi_hwref` picks the configuration for
  the board revision, as with tezuka.
* Network: USB only. The board appears as a USB network adapter
  (192.168.2.1), or use a USB-Ethernet adapter on the OTG port.
  `S99fwconfirm` takes any interface with an address as the uplink there
  (`/etc/fw-usb-only` in the Pluto overlay).
* First install on a Pluto running tezuka: as for the other boards,
  `tools/fw-push.sh --migrate -p analog 192.168.2.1 build/pluto-basic.zip`
  (slot A replaces tezuka's single image; that one write has no rollback).
* The AD9363 is set up as an AD9364 in the device tree (70 MHz - 6 GHz).
  The U-Boot environment's `attr_name` / `attr_val` (the usual Pluto way)
  still override it, and `mode` (1r1t / 2r2t) is applied as by tezuka.
* No refmeter in its bitstream: reference disciplining is off (trxd logs
  why) and there is no PPS input.
* **Recovery** through the Pluto's USB DFU (U-Boot's `dfu_sf`, as with
  tezuka): U-Boot enters it by itself when neither slot boots, and from a
  running board `device_reboot sf` reboots into it. (The button does not:
  the common environment skips the DFU button, as tezuka did.) Then, from
  a PC, `dfu-util -a firmware.dfu -D flash/pluto.dfu` writes the FIT at
  0x200000 (slot A), and `dfu-util -a boot.dfu -D flash/boot.dfu` the
  bootloader (only if that is broken). If the environment still names slot
  B and B fails, U-Boot boots slot A for that boot by itself (the boot
  script's fallback); `fw-update` then writes the next update over B.

## Recovery if everything fails

U-Boot is never written, so these always remain:

* USB DFU: `dfu_sf` in U-Boot, as with tezuka (`boot.dfu`, `pluto.dfu`).
* JTAG: `flash/jtag/` in the build zip.
* An SD card with `sdimg/` on it, if the board's boot switch allows SD.

## Fixed: a warm reboot could hang (LibreSDR, Winbond W25Q256)

Until 2026-09-28 a LibreSDR sometimes did not come back from a warm reboot
(twice after fw-update), and a power cycle always cured it. The QSPI is a
32 MiB Winbond W25Q256; the Zynq QSPI driver reaches its upper 16 MiB with
3-byte addresses and the chip's extended address register (the bank), and
resets the bank to 0 at shutdown so that the BootROM, which reads the lower
16 MiB, finds the boot image. The ADI kernel read the bank register only
for ST/Micron, Macronix and PMC; the upstream tezuka patch (0004) added
CFI_MFR_WINBOND, which is 0xda, but the chip reports the JEDEC ID 0xef. So
the bank read failed at probe ("failed to read ear reg"), the driver took
the bank for 0 whatever U-Boot had left (it reads slot B above 16 MiB),
skipped the switches to 0 and the reset at shutdown, and the BootROM could
come up reading the upper half. Patch 0010-spi-nor-winbond-ear.patch
matches 0xef and treats an unreadable bank as unknown. Checked on Libre 1
with the UART console: no warning, 10 warm reboots in a row and a flash
into slot A with its reboot, all fine.
