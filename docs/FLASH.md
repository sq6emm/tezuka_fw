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
| `0x1A80000` | 5.5 MB | `model` | DeepCW model (bf16-rounded, xz), shared by both slots. |

Slot A sits where tezuka's single image used to start. A board whose U-Boot
environment has no A/B scripts therefore still boots slot A.

### Size budget (per slot)

| Part | Size |
|---|---|
| Kernel, `Image.lzma` | ~4.6 MB |
| Bitstream, gzip | ~0.5 MB |
| Rootfs, xz (incl. trxd ~2 MB and web UI) | ~6 MB |
| **FIT total** | **must be ≤ 12.25 MB**; the build fails if not |

This fits because of three cuts:
- **DeepCW model:** moved to its own partition and bf16-rounded, 13.4 MB → 5.3 MB compressed. The accuracy matrix is unchanged.
- **Kernel:** shipped as lzma instead of zImage, saving 1.6 MB and avoiding tezuka's zImage corruption bug #450.
- **Rootfs:** trimmed of Wi-Fi drivers and firmware, DRM, the Fortran/OpenMP/C++ runtimes, CA certificates, libiio/iiod and the gpsd client tools.

## Updating: `tools/fw-push.sh`

```sh
tools/fw-push.sh -p analog <board-ip> build/plutoskyr2.zip
```

On the board, `fw-update` does the following:

1. Checks the build's own `flash/SHA256SUMS`, the slot size, and that the
   device tree inside is for this board model.
2. Writes `firmware.itb` into the slot that is **not** running, reads it back
   and compares byte for byte.
3. Writes `model.bin` only if the model changed, also verified.
4. In one write of the U-Boot environment: installs the A/B boot scripts
   shipped with the build (`uboot-ab.env`), sets `slot=<new>`,
   `upgrade_available=1`, `bootcount=0`.
5. Reboots.

U-Boot counts every boot of an unconfirmed slot. `iminfo` verifies every
image hash in the slot before booting. A failure resets, which counts, and so
does a kernel panic (`panic=10`). A lockup after init has started is caught by
the watchdog (`S15watchdog`). U-Boot has no watchdog, though, so a hang in
U-Boot or early in the kernel stays hung until the power is cycled. Each power
cycle counts as a boot. `S99fwconfirm` confirms
the slot once the network is up, dropbear listens and trxd runs, which clears
`upgrade_available`. The fourth unconfirmed boot switches back to the previous
slot automatically and sets `rolled_back=1`.

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
(fresh, pending, counting, rollback, failed load). The `fw-update` flash
writes were tested against simulated MTD devices for both layouts. Neither
has run on a real board yet.

## Recovery if everything fails

U-Boot is never written, so these always remain:

* USB DFU: `dfu_sf` in U-Boot, as with tezuka (`boot.dfu`, `pluto.dfu`).
* JTAG: `flash/jtag/` in the build zip.
* An SD card with `sdimg/` on it, if the board's boot switch allows SD.
