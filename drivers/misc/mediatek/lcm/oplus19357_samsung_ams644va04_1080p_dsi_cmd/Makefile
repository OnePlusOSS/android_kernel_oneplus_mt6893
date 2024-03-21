
# Makefile for misc devices that really don't fit anywhere else.
#
#include $(srctree)/drivers/misc/mediatek/Makefile.custom
subdir-ccflags-y += -Werror
obj-y += oplus19357_samsung_ams644va04_1080p_dsi_cmd.o

ccflags-$(CONFIG_MTK_LCM) += -I$(srctree)/drivers/misc/mediatek/lcm/inc

#ifdef OPLUS_FEATURE_RAMLESS_AOD
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/v3/ \
             -I$(srctree)/drivers/misc/mediatek/cmdq/v3/$(MTK_PLATFORM)
#endif /* OPLUS_FEATURE_RAMLESS_AOD */

