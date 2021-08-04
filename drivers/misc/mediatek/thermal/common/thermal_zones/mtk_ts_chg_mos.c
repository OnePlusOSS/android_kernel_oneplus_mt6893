// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
 /**************************************************************************
 * Copyright (c) 2020-2030 OPLUS Mobile Comm Corp.,Ltd. All Rights Reserved.
 * File                   : mtk_ts_flashlight.cpp
 * Description     : file to set flashlight ntc as a mtk thermal zone
 * -------------Rivision History-----------------------------------------
 * <version>      <date>            <author>                                      <Modify.desc>
 * OPLUS Mobile Comm Proprietary and Confidential.
 *************************************************************************/

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <mt-plat/aee.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/uaccess.h>
#include "mt-plat/mtk_thermal_monitor.h"
#include "mach/mtk_thermal.h"
#include "mtk_thermal_timer.h"
#include <linux/uidgid.h>
#include <linux/slab.h>
#include "tzbatt_initcfg.h"
#include <linux/power_supply.h>
extern int mt6360_get_chg_thermal_temperature(int *temp);


/* ************************************ */
static kuid_t uid = KUIDT_INIT(0);
static kgid_t gid = KGIDT_INIT(1000);
static DEFINE_SEMAPHORE(sem_mutex);

static unsigned int interval;	/* seconds, 0 : no auto polling */
static int trip_temp[10] = { 120000, 110000, 100000, 90000, 80000,
				70000, 65000, 60000, 55000, 50000 };

/* static unsigned int cl_dev_dis_charge_state = 0; */
static struct thermal_zone_device *thz_dev;
/* static struct thermal_cooling_device *cl_dev_dis_charge; */
static int kernelmode;
static int g_THERMAL_TRIP[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static int num_trip = 0;
static char g_bind0[20] = { 0 };
static char g_bind1[20] = { 0 };
static char g_bind2[20] = { 0 };
static char g_bind3[20] = { 0 };
static char g_bind4[20] = { 0 };
static char g_bind5[20] = { 0 };
static char g_bind6[20] = { 0 };
static char g_bind7[20] = { 0 };
static char g_bind8[20] = { 0 };
static char g_bind9[20] = { 0 };

struct NTC_TEMPERATURE {
	int NTC_Temp;
	int TemperatureR;
};

#define TEMPERATURE_TBL_SIZE 121
#define mtk_chg_mos_TEMP_CRIT 100
static struct NTC_TEMPERATURE CHG_MOS_Temperature_Table[] = {
	{-20,1790339},
	{-19,1789683},
	{-18,1788989},
	{-17,1788254},
	{-16,1787477},
	{-15,1786655},
	{-14,1785786},
	{-13,1784869},
	{-12,1783901},
	{-11,1782880},
	{-10,1781803},
	{-9, 1780668},
	{-8, 1779473},
	{-7, 1778214},
	{-6, 1776889},
	{-5, 1775496},
	{-4,  1774031},
	{-3, 1772492},
	{-2, 1770875},
	{-1, 1769178},
	{0,  1767398},
	{1,  1765530},
	{2,  1763573},
	{3,  1761522},
	{4,  1759375},
	{5,  1757127},
	{6,  1754775},
	{7,  1752317},
	{8,  1749747},
	{9,  1747062},
	{10, 1744260},
	{11, 1741335},
	{12, 1738284},
	{13, 1735103},
	{14, 1731789},
	{15, 1728336},
	{16, 1724743},
	{17, 1721004},
	{18, 1717116},
	{19, 1713074},
	{20, 1708876},
	{21, 1704516},
	{22, 1699992},
	{23, 1695299},
	{24, 1690434},
	{25, 1685393},
	{26, 1680172},
	{27, 1674768},
	{28, 1669177},
	{29, 1663397},
	{30, 1657423},
	{31, 1651253},
	{32, 1644884},
	{33, 1638313},
	{34, 1631537},
	{35, 1624554},
	{36, 1617362},
	{37, 1609959},
	{38, 1602342},
	{39, 1594510},
	{40, 1586462},
	{41, 1578196},
	{42, 1569712},
	{43, 1561009},
	{44, 1552086},
	{45, 1542944},
	{46, 1533583},
	{47, 1524002},
	{48, 1514204},
	{49, 1504189},
	{50, 1493958},
	{51, 1483514},
	{52, 1472858},
	{53, 1461993},
	{54, 1450922},
	{55, 1439647},
	{56, 1428173},
	{57, 1416502},
	{58, 1404640},
	{59, 1392590},
	{60, 1380357},
	{61, 1367946},
	{62, 1355363},
	{63, 1342613},
	{64, 1329701},
	{65, 1316635},
	{66, 1303421},
	{67, 1290065},
	{68, 1276574},
	{69, 1262956},
	{70, 1249217},
	{71, 1235365},
	{72, 1221409},
	{73, 1207355},
	{74, 1193211},
	{75, 1178987},
	{76, 1164689},
	{77, 1150327},
	{78, 1135908},
	{79, 1121441},
	{80, 1106933},
	{81, 1092395},
	{82, 1077833},
	{83, 1063256},
	{84, 1048672},
	{85, 1034090},
	{86, 1019518},
	{87, 1004962},
	{88, 990432},
	{89, 975935},
	{90, 961477},
	{91, 947068},
	{92, 932713},
	{93, 918421},
	{94, 904197},
	{95, 890048},
	{96, 875981},
	{97, 862002},
	{98, 848116},
	{99, 834328},
	{100,820646},
};

static int size = 120;

static int mtk_chg_mos_get_temp(struct thermal_zone_device *thermal, int *t)
{
	int i, real_temp, temp_uv;
	int next_volt, now_volt;
	int now_temp_idx;

	int ret = mt6360_get_chg_thermal_temperature(&temp_uv);

	pr_err("[%s] T_chg get_temp_status %d, temp_uv, %d\n", __func__, ret, temp_uv);

	real_temp = CHG_MOS_Temperature_Table[size].NTC_Temp * 1000;

	for (i = size - 1; i > 0; i--) {
		if (temp_uv > CHG_MOS_Temperature_Table[i].TemperatureR) {
			//real_temp = CHG_MOS_Temperature_Table[i].NTC_Temp * 1000;
		} else {
			i = (i + 1) <= 120 ? (i + 1) : 120 ;
			now_temp_idx = (i - 1) > 0 ? (i - 1) : 0;
			next_volt = CHG_MOS_Temperature_Table[i].TemperatureR;
			now_volt = CHG_MOS_Temperature_Table[now_temp_idx].TemperatureR;
			real_temp = (now_volt - temp_uv) * 1000/(now_volt - next_volt) + CHG_MOS_Temperature_Table[now_temp_idx].NTC_Temp * 1000;
			break;
		}
	}

	*t = real_temp;

	pr_err("[%s] T_chg_mos real_temp, %d\n", __func__, real_temp);

	thermal->polling_delay = 0;

	return 0;
}

static int mtk_chg_mos_bind(struct thermal_zone_device *thermal,
			     struct thermal_cooling_device *cdev)
{
	return 0;
}
static int mtk_chg_mos_unbind(struct thermal_zone_device *thermal,
			       struct thermal_cooling_device *cdev)
{
	return 0;
}

static int mtk_chg_mos_get_mode(
struct thermal_zone_device *thermal, enum thermal_device_mode *mode)
{
	*mode = (kernelmode) ? THERMAL_DEVICE_ENABLED : THERMAL_DEVICE_DISABLED;
	return 0;
}

static int mtk_chg_mos_set_mode(
struct thermal_zone_device *thermal, enum thermal_device_mode mode)
{
	kernelmode = mode;
	return 0;
}

static int mtk_chg_mos_get_trip_type(
struct thermal_zone_device *thermal, int trip, enum thermal_trip_type *type)
{
	*type = g_THERMAL_TRIP[trip];
	return 0;
}

static int mtk_chg_mos_get_trip_temp(
struct thermal_zone_device *thermal, int trip, int *temp)
{
	*temp = trip_temp[trip];
	return 0;
}

static int mtk_chg_mos_get_crit_temp(struct thermal_zone_device *thermal,
				      int *temperature)
{
	*temperature = mtk_chg_mos_TEMP_CRIT;
	return 0;
}

/* bind callback functions to thermalzone */
static struct thermal_zone_device_ops mtk_chg_mos_dev_ops = {
	.bind = mtk_chg_mos_bind,
	.unbind = mtk_chg_mos_unbind,
	.get_temp = mtk_chg_mos_get_temp,
	.get_mode = mtk_chg_mos_get_mode,
	.set_mode = mtk_chg_mos_set_mode,
	.get_trip_type = mtk_chg_mos_get_trip_type,
	.get_trip_temp = mtk_chg_mos_get_trip_temp,
	.get_crit_temp = mtk_chg_mos_get_crit_temp,
};


static int mtk_chg_mos_read(struct seq_file *m, void *v)
{

	seq_printf(m,
		"[mtk_chg_mos_read] trip_0_temp=%d,trip_1_temp=%d,trip_2_temp=%d,trip_3_temp=%d,\n",
		trip_temp[0], trip_temp[1], trip_temp[2], trip_temp[3]);

	seq_printf(m,
		"trip_4_temp=%d,trip_5_temp=%d,trip_6_temp=%d,trip_7_temp=%d,trip_8_temp=%d,trip_9_temp=%d,\n",
		trip_temp[4], trip_temp[5], trip_temp[6],
		trip_temp[7], trip_temp[8], trip_temp[9]);

	seq_printf(m,
		"g_THERMAL_TRIP_0=%d,g_THERMAL_TRIP_1=%d,g_THERMAL_TRIP_2=%d,g_THERMAL_TRIP_3=%d,",
		g_THERMAL_TRIP[0], g_THERMAL_TRIP[1],
		g_THERMAL_TRIP[2], g_THERMAL_TRIP[3]);

	seq_printf(m,
		"g_THERMAL_TRIP_4=%d,g_THERMAL_TRIP_5=%d,g_THERMAL_TRIP_6=%d,g_THERMAL_TRIP_7=%d,",
		g_THERMAL_TRIP[4], g_THERMAL_TRIP[5],
		g_THERMAL_TRIP[6], g_THERMAL_TRIP[7]);

	seq_printf(m,
		"g_THERMAL_TRIP_8=%d,g_THERMAL_TRIP_9=%d,\n",
		g_THERMAL_TRIP[8], g_THERMAL_TRIP[9]);

	seq_printf(m,
		"cooldev0=%s,cooldev1=%s,cooldev2=%s,cooldev3=%s,cooldev4=%s,\n",
		g_bind0, g_bind1, g_bind2, g_bind3, g_bind4);

	seq_printf(m,
		"cooldev5=%s,cooldev6=%s,cooldev7=%s,cooldev8=%s,cooldev9=%s,time_ms=%d\n",
		g_bind5, g_bind6, g_bind7, g_bind8, g_bind9, interval * 1000);

	return 0;
}


static int mtk_chg_mos_register_thermal(void);
static void mtk_chg_mos_unregister_thermal(void);

static ssize_t mtk_chg_mos_write(
struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	pr_debug("Power/chg_mos_Thermal: write, write, write!!!");
	pr_debug("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
	pr_debug("*****************************************");
	pr_debug("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
	return -EINVAL;
}

static int mtk_chg_mos_register_thermal(void)
{
	pr_err("[%s]\n", __func__);

	/* trips : trip 0~1 */
	thz_dev = mtk_thermal_zone_device_register("mtk_chg_mos", num_trip,
						NULL, &mtk_chg_mos_dev_ops,
						0, 0, 0, interval * 1000);

	return 0;
}

static void mtk_chg_mos_unregister_thermal(void)
{
	pr_err("[%s]\n", __func__);

	if (thz_dev) {
		mtk_thermal_zone_device_unregister(thz_dev);
		thz_dev = NULL;
	}
}

static int mtk_chg_mos_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_chg_mos_read, NULL);
}

static const struct file_operations mtkts_chg_mos_fops = {
	.owner = THIS_MODULE,
	.open = mtk_chg_mos_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = mtk_chg_mos_write,
	.release = single_release,
};

static int __init mtk_chg_mos_init(void)
{
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *mtk_chg_mos_dir = NULL;

	pr_err("[%s]\n", __func__);


	mtk_chg_mos_dir = mtk_thermal_get_proc_drv_therm_dir_entry();
	if (!mtk_chg_mos_dir) {
		pr_err("%s mkdir /proc/driver/thermal failed\n",
								__func__);
	} else {
		entry = proc_create("tzchg_mos", 0664, mtk_chg_mos_dir,
							&mtkts_chg_mos_fops);
		if (entry)
			proc_set_user(entry, uid, gid);
	}
	mtk_chg_mos_register_thermal();
	return 0;
}

static void __exit mtk_chg_mos_exit(void)
{
	pr_err("[%s]\n", __func__);
	mtk_chg_mos_unregister_thermal();
}
module_init(mtk_chg_mos_init);
module_exit(mtk_chg_mos_exit);

