
# Makefile for misc devices that really don't fit anywhere else.
#
#include $(srctree)/drivers/misc/mediatek/Makefile.custom
subdir-ccflags-y += -Werror
obj-y += Bl_lm3697.o

LCM_DEFINES := $(shell echo $(CONFIG_CUSTOM_KERNEL_LCM) | tr a-z A-Z)
DEFINES += $(foreach LCM,$(LCM_DEFINES),$(LCM))
ccflags-$(CONFIG_MTK_LCM) += $(addprefix -D, $(DEFINES))
ccflags-$(CONFIG_MTK_LCM) += -I$(srctree)/drivers/misc/mediatek/lcm/inc
