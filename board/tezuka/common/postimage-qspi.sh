#!/bin/sh
set -e

COMMON_DIR="$(dirname "$0")"
BIN_DIR="$1"
# Args from BR2_ROOTFS_POST_IMAGE_SCRIPT_ARG in board config file
BOARD_DIR="$2"
dfu_suffix="$HOST_DIR/bin/dfu-suffix"

DEVICE_VID=0x0456
DEVICE_PID=0xb673

# Buildroot's host-bootgen (xilinx_v2025.2) may be broken.
# Test it, fall back to system bootgen if needed.
. "$(dirname "$0")/find-bootgen.sh"

# ── Flash update (.frm / .dfu) ────────────────────────────────────────────────
# boot.img: FSBL + U-Boot only (no bitstream — FPGA loaded via pluto.itb FIT image)
# pluto.itb: FIT image bundling kernel + rootfs + bitstream + DTB
QSPIDIR="$BIN_DIR/flash"
SDIMGDIR="$BIN_DIR/sdimg"
JTAGDIR="$QSPIDIR/jtag"

# The rootfs again with xz's ARM branch filter (the kernel's XZ decoder has
# it: CONFIG_XZ_DEC_ARM): about 250 KB less flash than Buildroot's plain xz
# (docs/FLASH.md). The SD image's uramdisk takes the same file.
if [ -f "$BIN_DIR/rootfs.cpio" ]; then
	xz --check=crc32 --arm --lzma2=preset=9e -c "$BIN_DIR/rootfs.cpio" > "$BIN_DIR/rootfs.cpio.xz.tmp"
	mv "$BIN_DIR/rootfs.cpio.xz.tmp" "$BIN_DIR/rootfs.cpio.xz"
fi

echo "generating FIT image (pluto.itb)"
cp "$BOARD_DIR/plutomaia.its" "$BIN_DIR/plutomaia.its"
gzip -9 -n -c "$BIN_DIR/system_top.bit" > "$BIN_DIR/system_top.bit.gz"   # fpga@1 in the FIT
(cd "$BIN_DIR" && mkimage -f plutomaia.its pluto.itb)
cp "$BIN_DIR/pluto.itb" "$QSPIDIR/firmware.itb"   # the bare FIT, for the A/B slots

echo "generating pluto.frm"
md5sum "$BIN_DIR/pluto.itb" | cut -d ' ' -f 1 > "$BIN_DIR/pluto.md5"
cat "$BIN_DIR/pluto.itb" "$BIN_DIR/pluto.md5" > "$BIN_DIR/pluto.frm"

echo "generating pluto.dfu"
"$dfu_suffix" -a "$BIN_DIR/pluto.itb" -v "$DEVICE_VID" -p "$DEVICE_PID"
mv "$BIN_DIR/pluto.itb" "$BIN_DIR/pluto.dfu"

echo "generating boot.img"
echo "img : {[bootloader] $BIN_DIR/fsbl.elf $BIN_DIR/u-boot.elf}" > "$BIN_DIR/boot.bif"
"$BOOTGEN" -image "$BIN_DIR/boot.bif" -w -o i "$BIN_DIR/boot.img"

echo "generating boot.frm"
cat "$BIN_DIR/boot.img" "$BIN_DIR/uboot-env.bin" "$COMMON_DIR/target_mtd_info.key" | \
	tee "$BIN_DIR/boot.frm" | md5sum | cut -d ' ' -f1 | tee -a "$BIN_DIR/boot.frm"

echo "generating boot.dfu"
cp "$BIN_DIR/boot.img" "$BIN_DIR/boot.bin.tmp"
"$dfu_suffix" -a "$BIN_DIR/boot.bin.tmp" -v "$DEVICE_VID" -p "$DEVICE_PID"
mv "$BIN_DIR/boot.bin.tmp" "$BIN_DIR/boot.dfu"

echo "generating uboot-env.dfu"
cp "$BIN_DIR/uboot-env.bin" "$BIN_DIR/uboot-env.bin.tmp"
"$dfu_suffix" -a "$BIN_DIR/uboot-env.bin.tmp" -v "$DEVICE_VID" -p "$DEVICE_PID"
mv "$BIN_DIR/uboot-env.bin.tmp" "$BIN_DIR/uboot-env.dfu"

cp "$BIN_DIR/boot.dfu" "$BIN_DIR/boot.frm" "$BIN_DIR/pluto.dfu" "$BIN_DIR/pluto.frm" $QSPIDIR

# JTAG - FIXME arm-linux is fixed path
"$HOST_DIR/bin/arm-linux-strip" "$BIN_DIR/u-boot.elf"
cp "$BIN_DIR/u-boot.elf" "$JTAGDIR"
cp "$BOARD_DIR/bitstream/fsbl.elf" "$JTAGDIR"
cp "$BR2_EXTERNAL/tools/jtag-recovery/xilinx-tcl.cfg" "$JTAGDIR"
cp "$BR2_EXTERNAL/tools/jtag-recovery/boot_fsbl_uboot.sh" "$JTAGDIR"
cp "$BR2_EXTERNAL/tools/jtag-recovery/boot_fsbl_uboot.bat" "$JTAGDIR"
cp "$BR2_EXTERNAL/tools/jtag-recovery/tezuka.cfg" "$JTAGDIR"
# ── A/B flash slots (docs/FLASH.md) ──────────────────────────────────────────
# firmware.itb goes into QSPI partition linux-a or linux-b (12.25 MB each),
# model.bin into the shared `model` partition (5.5 MB). /usr/sbin/fw-update
# writes them; SHA256SUMS lets it verify before and after writing.
SLOT_MAX=$((0xC40000))
MODEL_MAX=$((0x580000))
FIT_BYTES=$(wc -c < "$QSPIDIR/firmware.itb")
if [ "$FIT_BYTES" -gt "$SLOT_MAX" ]; then
    echo "ERROR: firmware.itb is $FIT_BYTES bytes, larger than a flash slot ($SLOT_MAX)." >&2
    echo "       Trim the rootfs or kernel (docs/FLASH.md has the budget)." >&2
    exit 1
fi
echo "firmware.itb: $FIT_BYTES bytes, $(( (SLOT_MAX - FIT_BYTES) / 1024 )) KiB free in a slot"
MODEL="$BR2_EXTERNAL/package/trxd/deepcw-model.bin"
if [ -f "$MODEL" ]; then
    [ "$(wc -c < "$MODEL")" -le "$MODEL_MAX" ] || { echo "ERROR: model.bin larger than its partition" >&2; exit 1; }
    cp "$MODEL" "$QSPIDIR/model.bin"
fi
"$COMMON_DIR/uboot-ab-env.sh" "$BIN_DIR/uboot-env.txt" > "$QSPIDIR/uboot-ab.env"
[ "$(wc -l < "$QSPIDIR/uboot-ab.env")" -eq 10 ] || { echo "ERROR: uboot-ab.env incomplete" >&2; exit 1; }
(cd "$COMMON_DIR" && git describe --abbrev=4 --always --tags --dirty 2>/dev/null || echo unknown) > "$QSPIDIR/VERSION"
(cd "$QSPIDIR" && sha256sum firmware.itb $( [ -f model.bin ] && echo model.bin ) uboot-ab.env VERSION > SHA256SUMS)
