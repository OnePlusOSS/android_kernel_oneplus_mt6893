#include $(srctree)/drivers/misc/mediatek/Makefile.custom
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#for enable KASAN, use GCC complile tool, add flag -Wno-misleading-indentation
ifeq ($(CONFIG_KASAN),y)
subdir-ccflags-y += -Wno-format
endif
#endif /*OPLUS_FEATURE_CAMERA_COMMON*/
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/mmdvfs

obj-y += s5kgm1spmipiraw_Sensor.o
