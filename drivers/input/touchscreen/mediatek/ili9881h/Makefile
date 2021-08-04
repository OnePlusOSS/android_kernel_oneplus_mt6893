# Version: 1.0
# Date : 2018/5/14
#
# -------------------------Revision History:----------------------------------
#  <author>	 <data> 	<version >			<desc>
#
#
#*****************************************************************************/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ili9881h/
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/base
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/alsps/inc/

obj-$(CONFIG_TOUCHSCREEN_ILI9881H) += ilitek_main.o \
                                      ilitek_spi.o \
									  ilitek_plat_mtk.o \
									  ilitek_ic.o \
									  ilitek_touch.o \
									  ilitek_mp.o \
									  ilitek_fw.o \
									  ilitek_node.o	\
									  oplus_node.o