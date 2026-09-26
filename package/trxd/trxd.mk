################################################################################
#
# trxd
#
################################################################################

TRXD_VERSION = 0.1.0
TRXD_SITE = $(BR2_EXTERNAL_PLUTOSDR_PATH)/src/trxd
TRXD_SITE_METHOD = local
TRXD_LICENSE = AGPL-3.0-only
TRXD_LICENSE_FILES = Cargo.toml

TRXD_RUST_TARGET = armv7-unknown-linux-gnueabihf

# rustup's cargo, not Buildroot's host-rustc: see Config.in. Crates come from
# Cargo.lock (--locked); the download needs network access, as maia-httpd's
# did. The Cortex-A9 has NEON; the armv7 target does not assume it.
TRXD_CARGO_ENV = \
	PATH="$(HOME)/.cargo/bin:$(PATH)" \
	RUSTUP_TOOLCHAIN= \
	CARGO_TARGET_DIR="$(@D)/target" \
	CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABIHF_LINKER="$(TARGET_CC)" \
	CC_armv7_unknown_linux_gnueabihf="$(TARGET_CC)" \
	CXX_armv7_unknown_linux_gnueabihf="$(TARGET_CXX)" \
	AR_armv7_unknown_linux_gnueabihf="$(TARGET_AR)" \
	RUSTFLAGS="-C target-cpu=cortex-a9 -C target-feature=+neon"

# Built from the tree itself (src/trxd), not Buildroot's rsynced copy, so
# that relative paths in Cargo.toml (the temporary sdroxide [patch]) and the
# pinned rust-toolchain.toml resolve; the artefacts still land in $(@D).
define TRXD_BUILD_CMDS
	cd $(TRXD_SITE) && $(TRXD_CARGO_ENV) cargo build --release --locked --target $(TRXD_RUST_TARGET)
endef

define TRXD_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/target/$(TRXD_RUST_TARGET)/release/trxd $(TARGET_DIR)/usr/bin/trxd
	$(INSTALL) -D -m 0644 $(TRXD_PKGDIR)/trxd.toml $(TARGET_DIR)/etc/trxd.toml
	$(INSTALL) -D -m 0755 $(TRXD_PKGDIR)/S80trxd $(TARGET_DIR)/etc/init.d/S80trxd
endef

$(eval $(generic-package))
