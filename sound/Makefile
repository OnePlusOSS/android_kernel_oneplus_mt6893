# SPDX-License-Identifier: GPL-2.0
# Makefile for the Linux sound card driver
#

obj-$(CONFIG_SOUND) += soundcore.o
obj-$(CONFIG_DMASOUND) += oss/dmasound/
obj-$(CONFIG_SND) += core/ i2c/ drivers/ isa/ pci/ ppc/ arm/ sh/ synth/ usb/ \
	firewire/ sparc/ spi/ parisc/ pcmcia/ mips/ soc/ atmel/ hda/ x86/ xen/
obj-$(CONFIG_SND_AOA) += aoa/

# This one must be compilable even if sound is configured out
obj-$(CONFIG_AC97_BUS) += ac97_bus.o
obj-$(CONFIG_AC97_BUS_NEW) += ac97/

ifeq ($(CONFIG_SND),y)
  obj-y += last.o
endif

soundcore-objs  := sound_core.o

# TODO:remove
ifndef OPLUS_BUG_COMPATIBILITY
subdir-ccflags-y += -DOPLUS_BUG_COMPATIBILITY
endif

ifndef OPLUS_BUG_STABILITY
subdir-ccflags-y += -DOPLUS_BUG_STABILITY
endif

ifndef OPLUS_BUG_DEBUG
subdir-ccflags-y += -DOPLUS_BUG_DEBUG
endif

ifndef OPLUS_FEATURE_SPEAKER_MUTE
subdir-ccflags-y += -DOPLUS_FEATURE_SPEAKER_MUTE
endif

ifndef OPLUS_ARCH_EXTENDS
subdir-ccflags-y += -DOPLUS_ARCH_EXTENDS
endif
