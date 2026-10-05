################################################################################
#
# ffmpeg-aac: static libavcodec/libavutil, Opus decoder + AAC encoder only
#
################################################################################

FFMPEG_AAC_VERSION = 7.1.1
FFMPEG_AAC_SOURCE = ffmpeg-$(FFMPEG_AAC_VERSION).tar.xz
FFMPEG_AAC_SITE = https://ffmpeg.org/releases
FFMPEG_AAC_LICENSE = LGPL-2.1+
FFMPEG_AAC_LICENSE_FILES = COPYING.LGPLv2.1
FFMPEG_AAC_INSTALL_STAGING = YES
FFMPEG_AAC_INSTALL_TARGET = NO

# Everything off, then the two codecs trxd uses (src/trxd/src/dvbs2/aacx.c);
# libswresample stays: ffmpeg's Opus decoder needs it.
define FFMPEG_AAC_CONFIGURE_CMDS
	cd $(@D) && ./configure \
		--prefix=/usr \
		--enable-cross-compile \
		--cross-prefix="$(TARGET_CROSS)" \
		--cc="$(TARGET_CC)" \
		--ar="$(TARGET_AR)" \
		--nm="$(TARGET_NM)" \
		--ranlib="$(TARGET_RANLIB)" \
		--arch=arm \
		--cpu=cortex-a9 \
		--target-os=linux \
		--extra-cflags="$(TARGET_CFLAGS)" \
		--enable-static \
		--disable-shared \
		--enable-pic \
		--disable-autodetect \
		--disable-everything \
		--disable-programs \
		--disable-doc \
		--disable-avdevice \
		--disable-avformat \
		--disable-avfilter \
		--disable-swscale \
		--disable-network \
		--disable-debug \
		--enable-encoder=aac \
		--enable-decoder=opus
endef

define FFMPEG_AAC_BUILD_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D)
endef

define FFMPEG_AAC_INSTALL_STAGING_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D) DESTDIR=$(STAGING_DIR) install
endef

$(eval $(generic-package))
