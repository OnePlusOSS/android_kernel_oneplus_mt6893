subdir-ccflags-y += -Werror

obj-y += oplus_display_private_api.o \
         oplus_display_panel_power.o \
         oplus_display_common.o \
         oplus_display_dc.o \
         oplus_display_onscreenfingerprint.o

obj-y += display_panel/

obj-y += oplus_display_mtk_debug.o

ccflags-y += -I$(srctree)/drivers/misc/mediatek/dramc/$(MTK_PLATFORM)	\
             -I$(srctree)/drivers/misc/mediatek/smi/			\
             -I$(srctree)/drivers/misc/mediatek/mmp/			\
             -I$(srctree)/drivers/misc/mediatek/mmdvfs/			\
             -I$(srctree)/drivers/iommu/				\
             -I$(srctree)/drivers/devfreq/				\
             -I$(srctree)/drivers/gpu/drm/				\
             -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/mach/				\
             -I$(srctree)/drivers/misc/mediatek/include/mt-plat \
             -I$(srctree)/drivers/misc/mediatek/leds/ \
             -I$(srctree)/drivers/misc/mediatek/leds/$(MTK_PLATFORM) \
             -I$(srctree)/drivers/misc/mediatek/cmdq/mailbox/ \
             -I$(srctree)/include/drm/ \
             -I$(srctree)/drivers/gpu/drm/mediatek/

ccflags-$(CONFIG_MTK_CMDQ_MBOX_EXT) += -I$(srctree)/drivers/misc/mediatek/cmdq/mailbox

subdir-ccflags-y += -I$(srctree)/drivers/staging/android/mtk_ion/ \
		    -I$(srctree)/drivers/staging/android/mtk_ion/mtk
