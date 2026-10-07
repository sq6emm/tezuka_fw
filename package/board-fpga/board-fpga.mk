################################################################################
#
# board-fpga -- extract FPGA bitstream for the target board
#
# FPGA modes (docs/FPGA-MODES.md): the board's bitstream/ folder has a
# boot-mode file naming a mode; U-Boot's bitstream (the FIT's) is
# <project>-<mode>.xsa and every other <project>-<mode>.xsa goes into the
# rootfs as /lib/firmware/fpga-<mode>.bin for fpga-mode (which takes the
# boot one out of the running slot's FIT when it needs it: flash space);
# the boot mode's name goes to /etc/fpga-boot-mode.
#
################################################################################

BOARD_FPGA_VERSION = 1.0
BOARD_FPGA_SITE = $(BR2_EXTERNAL_PLUTOSDR_PATH)/board/tezuka/$(call qstrip,$(BR2_PACKAGE_BOARD_FPGA_BOARD))/bitstream
BOARD_FPGA_SITE_METHOD = local
BOARD_FPGA_INSTALL_IMAGES = YES
BOARD_FPGA_INSTALL_TARGET = YES
BOARD_FPGA_PROJECT = $(call qstrip,$(BR2_PACKAGE_BOARD_FPGA_PROJECT))
# The flavour's modes ("" = all the board has) and name (docs/FLAVOURS.md).
BOARD_FPGA_MODES = $(call qstrip,$(BR2_PACKAGE_BOARD_FPGA_MODES))
BOARD_FPGA_FLAVOUR = $(call qstrip,$(BR2_PACKAGE_BOARD_FPGA_FLAVOUR))

# The local site's rsync copies but never deletes: a bitstream or rate file
# gone from the tree (a mode dropped) stayed in $(@D) and went into the
# rootfs again (2026-10-07: a stale fpga-datv.bin made the PlutoSky R2 FIT
# 12 KB too big for its slot). Drop what the tree no longer has.
define BOARD_FPGA_DROP_STALE
	for f in $(@D)/*.xsa $(@D)/*.rate; do \
		[ -e "$$f" ] || continue; \
		[ -e "$(BOARD_FPGA_SITE)/$${f##*/}" ] || { echo "board-fpga: dropping stale $${f##*/}"; rm -f "$$f"; }; \
	done
endef
BOARD_FPGA_POST_RSYNC_HOOKS += BOARD_FPGA_DROP_STALE

define BOARD_FPGA_INSTALL_IMAGES_CMDS
	mode=$$(cat $(@D)/boot-mode) || { echo "board-fpga: no boot-mode file beside the bitstreams" >&2; exit 1; }; \
	$(UNZIP) -p $(@D)/$(BOARD_FPGA_PROJECT)-$$mode.xsa system_top.bit > $(BINARIES_DIR)/system_top.bit
endef

# A mode whose image runs the AD936x at another converter rate than 3.072
# MS/s (x8) has a rate file beside it, "<rate> <decimation> [<DATV input>]"
# (the wide trx images: "24576000 64"; the wide DATV images, whose DDC and T2
# resampler take the first decimation stage: "24576000 64 3072000"); trxd
# reads /lib/firmware/fpga-<mode>.rate.
define BOARD_FPGA_INSTALL_TARGET_CMDS
	rm -f $(TARGET_DIR)/lib/firmware/fpga-*.bin $(TARGET_DIR)/lib/firmware/fpga-*.rate $(TARGET_DIR)/etc/fpga-boot-mode $(TARGET_DIR)/etc/fw-flavour
	$(if $(BOARD_FPGA_FLAVOUR),echo $(BOARD_FPGA_FLAVOUR) > $(TARGET_DIR)/etc/fw-flavour)
	mode=$$(cat $(@D)/boot-mode) || exit 1; \
	mkdir -p $(TARGET_DIR)/lib/firmware; \
	echo $$mode > $(TARGET_DIR)/etc/fpga-boot-mode; \
	for x in $(@D)/$(BOARD_FPGA_PROJECT)-*.xsa; do \
		[ -e "$$x" ] || continue; \
		m=$${x##*/$(BOARD_FPGA_PROJECT)-}; m=$${m%.xsa}; \
		[ "$$m" = "$$mode" ] && continue; \
		case " $(BOARD_FPGA_MODES) " in "  ") ;; *" $$m "*) ;; *) continue ;; esac; \
		$(UNZIP) -p $$x system_top.bit > $(@D)/mode.bit; \
		python3 $(BR2_EXTERNAL_PLUTOSDR_PATH)/board/tezuka/common/bit2bin.py \
			$(@D)/mode.bit $(TARGET_DIR)/lib/firmware/fpga-$$m.bin || exit 1; \
	done; \
	for r in $(@D)/$(BOARD_FPGA_PROJECT)-*.rate; do \
		[ -e "$$r" ] || continue; \
		m=$${r##*/$(BOARD_FPGA_PROJECT)-}; m=$${m%.rate}; \
		cp $$r $(TARGET_DIR)/lib/firmware/fpga-$$m.rate; \
	done
endef

$(eval $(generic-package))
