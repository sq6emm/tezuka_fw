################################################################################
#
# board-fpga -- extract FPGA bitstream for the target board
#
# FPGA modes (docs/FPGA-MODES.md): when the board's bitstream/ folder has a
# boot-mode file naming a mode other than "all", U-Boot's bitstream (the
# FIT's) is <project>-<mode>.xsa and every other <project>-<mode>.xsa goes
# into the rootfs as /lib/firmware/fpga-<mode>.bin for fpga-mode (which
# takes the boot one out of the running slot's FIT when it needs it: flash
# space); the boot mode's name goes to /etc/fpga-boot-mode. Without it: <project>.xsa, as
# always.
#
################################################################################

BOARD_FPGA_VERSION = 1.0
BOARD_FPGA_SOURCE = $(call qstrip,$(BR2_PACKAGE_BOARD_FPGA_PROJECT)).xsa
BOARD_FPGA_SITE = $(BR2_EXTERNAL_PLUTOSDR_PATH)/board/tezuka/$(call qstrip,$(BR2_PACKAGE_BOARD_FPGA_BOARD))/bitstream
BOARD_FPGA_SITE_METHOD = local
BOARD_FPGA_INSTALL_IMAGES = YES
BOARD_FPGA_INSTALL_TARGET = YES
BOARD_FPGA_PROJECT = $(call qstrip,$(BR2_PACKAGE_BOARD_FPGA_PROJECT))

define BOARD_FPGA_INSTALL_IMAGES_CMDS
	mode=$$(cat $(@D)/boot-mode 2>/dev/null || echo all); \
	if [ "$$mode" = all ]; then xsa=$(@D)/$(BOARD_FPGA_SOURCE); \
	else xsa=$(@D)/$(BOARD_FPGA_PROJECT)-$$mode.xsa; fi; \
	$(UNZIP) -p $$xsa system_top.bit > $(BINARIES_DIR)/system_top.bit
endef

define BOARD_FPGA_INSTALL_TARGET_CMDS
	rm -f $(TARGET_DIR)/lib/firmware/fpga-*.bin $(TARGET_DIR)/etc/fpga-boot-mode
	mode=$$(cat $(@D)/boot-mode 2>/dev/null || echo all); \
	if [ "$$mode" != all ]; then \
		mkdir -p $(TARGET_DIR)/lib/firmware; \
		echo $$mode > $(TARGET_DIR)/etc/fpga-boot-mode; \
		for x in $(@D)/$(BOARD_FPGA_PROJECT)-*.xsa; do \
			[ -e "$$x" ] || continue; \
			m=$${x##*/$(BOARD_FPGA_PROJECT)-}; m=$${m%.xsa}; \
			[ "$$m" = "$$mode" ] && continue; \
			$(UNZIP) -p $$x system_top.bit > $(@D)/mode.bit; \
			python3 $(BR2_EXTERNAL_PLUTOSDR_PATH)/board/tezuka/common/bit2bin.py \
				$(@D)/mode.bit $(TARGET_DIR)/lib/firmware/fpga-$$m.bin || exit 1; \
		done; \
	fi
endef

$(eval $(generic-package))
