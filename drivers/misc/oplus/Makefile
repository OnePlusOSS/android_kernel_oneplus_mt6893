#ifdef OPLUS_ARCH_EXTEND
obj-y += oplus_display_private_api.o
obj-y += oplus_display_onscreenfingerprint.o
obj-y += oplus_display_alwaysondisplay.o
obj-y += oplus_display_common.o
obj-y += oplus_display_dc.o
obj-y += display_panel/
ccflags-y += -I$(srctree)/include/linux/
ccflags-y += -I$(srctree)/include/uapi/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include/
ifeq ($(CONFIG_MACH_MT6785), y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/videox/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/dispsys/
else
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/mt6779/videox/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/mt6779/dispsys/
endif
ccflags-y += -I$(srctree)/drivers/misc/oplus/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mmp/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/lcm/inc/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/ext_disp \
	     -I$(srctree)/drivers/misc/mediatek/ext_disp/v2 \
	     -I$(srctree)/drivers/misc/mediatek/ext_disp/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include/		\
             -I$(srctree)/drivers/misc/mediatek/video/common/		\
             -I$(srctree)/drivers/misc/mediatek/video/common/rdma20/	\
             -I$(srctree)/drivers/misc/mediatek/video/common/wdma20/	\
             -I$(srctree)/drivers/misc/mediatek/video/common/layering_rule_base/v1.2/ \
             -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/dispsys/	\
             -I$(srctree)/drivers/misc/mediatek/sync/			\
             -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include/mach/ \
             -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM) \
             -I$(srctree)/drivers/misc/mediatek/base/power/include	\
             -I$(srctree)/drivers/misc/mediatek/base/power/include/spm_v2 \
             -I$(srctree)/drivers/misc/mediatek/smi/			\
             -I$(srctree)/drivers/misc/mediatek/dramc/$(MTK_PLATFORM)	\
             -I$(srctree)/drivers/misc/mediatek/gpu/ged/include/	\
	     -I$(srctree)/drivers/devfreq/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/v3/ \
		    -I$(srctree)/drivers/misc/mediatek/cmdq/v3/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include/ \
             -I$(srctree)/drivers/misc/mediatek/video/common/ \
             -I$(srctree)/drivers/misc/mediatek/video/common/rdma20/ \
             -I$(srctree)/drivers/misc/mediatek/video/common/wdma20/ \
             -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/videox/ \
             -I$(srctree)/drivers/misc/mediatek/base/power/include/ \
             -I$(srctree)/drivers/misc/mediatek/smi/ \
             -I$(srctree)/drivers/misc/mediatek/gpu/ged/include/ \
             -I$(srctree)/drivers/iommu/ \
             -I$(srctree)/drivers/misc/mediatek/smi/variant/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/
ifeq ($(CONFIG_MTK_ION),y)
subdir-ccflags-y += -I$(srctree)/drivers/staging/android/mtk_ion/ \
             -I$(srctree)/drivers/staging/android/mtk_ion/mtk/
else
subdir-ccflags-y += -I$(srctree)/drivers/staging/android/aosp_ion/
endif
#endif/*OPLUS_ARCH_EXTEND*/
