#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#


MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/pmic/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/pmic/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/power/supply/mediatek/misc

ifeq ($(CONFIG_OPLUS_CHARGER_MTK6785),y)
	obj-y += mtk_battery.o mtk_power_misc.o mtk_gauge_coulomb_service.o mtk_gauge_class.o mtk_battery_recovery.o mtk_battery_core.o
else ifeq ($(CONFIG_OPLUS_CHARGER_MTK6769),y)
subdir-ccflags-y += -Werror -I$(srctree)/drivers/power/supply/mediatek/misc
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc/
	obj-y += oplus_mtk_battery.o mtk_power_misc.o mtk_gauge_coulomb_service.o mtk_gauge_class.o mtk_battery_recovery.o mtk_battery_core.o
else ifeq ($(CONFIG_OPLUS_CHARGER_MTK6768),y)
subdir-ccflags-y += -Werror -I$(srctree)/drivers/power/supply/mediatek/misc
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc/
        obj-y += oplus_mtk_battery.o mtk_power_misc.o mtk_gauge_coulomb_service.o mtk_gauge_class.o mtk_battery_recovery.o mtk_battery_core.o
else ifeq ($(CONFIG_OPLUS_CHARGER_MTK6769R),y)
subdir-ccflags-y += -Werror -I$(srctree)/drivers/power/supply/mediatek/misc
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc/
        obj-y += oplus_mtk_battery.o mtk_power_misc.o mtk_gauge_coulomb_service.o mtk_gauge_class.o mtk_battery_recovery.o mtk_battery_core.o
else ifeq ($(CONFIG_MTK_GAUGE_VERSION),30)
ifeq ($(CONFIG_OPLUS_CHARGER_MTK6771),y)
	obj-y += oplus_mtk_battery.o mtk_power_misc.o mtk_gauge_coulomb_service.o mtk_gauge_class.o mtk_battery_recovery.o mtk_battery_core.o
else
	obj-y += mtk_battery.o mtk_power_misc.o mtk_gauge_coulomb_service.o mtk_gauge_class.o mtk_battery_recovery.o mtk_battery_core.o
endif
endif
