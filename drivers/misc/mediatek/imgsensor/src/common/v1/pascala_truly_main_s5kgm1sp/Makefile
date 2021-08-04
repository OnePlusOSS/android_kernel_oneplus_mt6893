#include $(srctree)/drivers/misc/mediatek/Makefile.custom
#for enable KASAN, use GCC complile tool, add flag -Wno-misleading-indentation
ifeq ($(CONFIG_KASAN),y)
subdir-ccflags-y += -Wno-format
endif
obj-y += s5kgm1spmipiraw_Sensor.o