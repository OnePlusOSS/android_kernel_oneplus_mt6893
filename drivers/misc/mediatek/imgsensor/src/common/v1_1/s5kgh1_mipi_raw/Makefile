#include $(srctree)/drivers/misc/mediatek/Makefile.custom
#ifdef OPLUS_FEATURE_CAMERA_COMMON
ifeq ($(CONFIG_KASAN),y)
subdir-ccflags-y += -Wno-format
endif
#endif /*OPLUS_FEATURE_CAMERA_COMMON*/
obj-y += s5kgh1mipiraw_Sensor.o

