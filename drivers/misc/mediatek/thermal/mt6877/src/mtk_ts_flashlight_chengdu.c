// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

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

extern int mt6360_get_flashlight_temperature(int *temp);

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
static int mtk_flashlight_debug_log;
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

/* static int flashlight_write_flag=0; */

#define MTK_FLASHLIGHT_TEMP_CRIT 120000	/* 120.000 degree Celsius */

#define mtk_flashlight_dprintk(fmt, args...)   \
do {                                    \
	if (mtk_flashlight_debug_log) {                \
		pr_debug("[Thermal/TZ/flashlight]" fmt, ##args); \
	}                                   \
} while (0)

#define mtk_flashlight_printk(fmt, args...)   \
pr_info("[Thermal/TZ/flashlight]" fmt, ##args)

struct NTC_TEMPERATURE {
	int ntc_temp;
	int temperature_r;
};

#define TEMPERATURE_TBL_SIZE 166

static struct NTC_TEMPERATURE FLASHLIGHT_Temperature_Table[] = {
	{-40, 1795971},
	{-39, 1795675},
	{-38, 1795360},
	{-37, 1795024},
	{-36, 1794667},
	{-35, 1794286},
	{-34, 1793882},
	{-33, 1793450},
	{-32, 1792993},
	{-31, 1792509},
	{-30, 1791993},
	{-29, 1791445},
	{-28, 1790868},
	{-27, 1790249},
	{-26, 1789601},
	{-25, 1788909},
	{-24, 1788181},
	{-23, 1787404},
	{-22, 1786587},
	{-21, 1785726},
	{-20, 1784810},
	{-19, 1783842},
	{-18, 1782824},
	{-17, 1781746},
	{-16, 1780610},
	{-15, 1779412},
	{-14, 1778150},
	{-13, 1776819},
	{-12, 1775420},
	{-11, 1773943},
	{-10, 1772393},
	{-9,  1770760},
	{-8,  1769040},
	{-7,  1767237},
	{-6,  1765345},
	{-5,  1763355},
	{-4,  1761265},
	{-3,  1759072},
	{-2,  1756772},
	{-1,  1754361},
	{0,   1751846},
	{1,   1749196},
	{2,   1746429},
	{3,   1743538},
	{4,   1740516},
	{5,   1737348},
	{6,   1734042},
	{7,   1730582},
	{8,   1726978},
	{9,   1723241},
	{10,  1719319},
	{11,  1715214},
	{12,  1710979},
	{13,  1706542},
	{14,  1701907},
	{15,  1697084},
	{16,  1692151},
	{17,  1686935},
	{18,  1681579},
	{19,  1675948},
	{20,  1670130},
	{21,  1664151},
	{22,  1657932},
	{23,  1651485},
	{24,  1644694},
	{25,  1637838},
	{26,  1630827},
	{27,  1623581},
	{28,  1616120},
	{29,  1608409},
	{30,  1600466},
	{31,  1592292},
	{32,  1583862},
	{33,  1575169},
	{34,  1566264},
	{35,  1557085},
	{36,  1547687},
	{37,  1538029},
	{38,  1528097},
	{39,  1517913},
	{40,  1507507},
	{41,  1496817},
	{42,  1485919},
	{43,  1474738},
	{44,  1463363},
	{45,  1451703},
	{46,  1439856},
	{47,  1427715},
	{48,  1415385},
	{49,  1402737},
	{50,  1389977},
	{51,  1376968},
	{52,  1363742},
	{53,  1350225},
	{54,  1336680},
	{55,  1322800},
	{56,  1308734},
	{57,  1294524},
	{58,  1280219},
	{59,  1265717},
	{60,  1250885},
	{61,  1236090},
	{62,  1221222},
	{63,  1206137},
	{64,  1190863},
	{65,  1175434},
	{66,  1160114},
	{67,  1144501},
	{68,  1129109},
	{69,  1113239},
	{70,  1097698},
	{71,  1082010},
	{72,  1066205},
	{73,  1050312},
	{74,  1034368},
	{75,  1018750},
	{76,  1002834},
	{77,  986992},
	{78,  971271},
	{79,  955326},
	{80,  939579},
	{81,  924088},
	{82,  908470},
	{83,  893016},
	{84,  877585},
	{85,  862256},
	{86,  847064},
	{87,  831946},
	{88,  816931},
	{89,  802051},
	{90,  787285},
	{91,  772661},
	{92,  758152},
	{93,  743848},
	{94,  729655},
	{95,  715663},
	{96,  701769},
	{97,  688065},
	{98,  674578},
	{99,  661264},
	{100, 648074},
	{101, 635102},
	{102, 622298},
	{103, 609681},
	{104, 597274},
	{105, 585015},
	{106, 572922},
	{107, 561098},
	{108, 549479},
	{109, 537993},
	{110, 526745},
	{111, 515653},
	{112, 504775},
	{113, 494100},
	{114, 483604},
	{115, 473310},
	{116, 463211},
	{117, 453299},
	{118, 443577},
	{119, 434056},
	{120, 424705},
	{121, 415555},
	{122, 406574},
	{123, 397781},
	{124, 389176},
	{125, 380744}
};

static int pre_temp_idx = 70;


static int mtk_flashlight_get_temp(struct thermal_zone_device *thermal, int *t)
{
	int real_temp, uv_l, uv_r;
	int temp_uv = 1636364;

	int ret = mt6360_get_flashlight_temperature(&temp_uv);

	mtk_flashlight_printk("[%s] T_flashlight get_temp_status %d, temp_uv, %d\n", __func__, ret, temp_uv);

	if (pre_temp_idx > TEMPERATURE_TBL_SIZE - 1) {
		pre_temp_idx = TEMPERATURE_TBL_SIZE - 1;
	} else if (pre_temp_idx < 0) {
		pre_temp_idx = 0;
	}

	if (temp_uv < FLASHLIGHT_Temperature_Table[pre_temp_idx].temperature_r) {
		for (; pre_temp_idx < TEMPERATURE_TBL_SIZE - 1; pre_temp_idx++) {
			if (temp_uv <= FLASHLIGHT_Temperature_Table[pre_temp_idx].temperature_r
					&& temp_uv > FLASHLIGHT_Temperature_Table[pre_temp_idx+1].temperature_r)
				break;
		}
	} else if (temp_uv > FLASHLIGHT_Temperature_Table[pre_temp_idx].temperature_r) {
		for (; pre_temp_idx > 0; pre_temp_idx--) {
			if (temp_uv <= FLASHLIGHT_Temperature_Table[pre_temp_idx].temperature_r
					&& temp_uv > FLASHLIGHT_Temperature_Table[pre_temp_idx+1].temperature_r)
				break;
		}
	}
	if ((temp_uv >= FLASHLIGHT_Temperature_Table[0].temperature_r) && pre_temp_idx <= 0) {
		real_temp = FLASHLIGHT_Temperature_Table[0].ntc_temp * 1000;
	} else if (temp_uv <= FLASHLIGHT_Temperature_Table[TEMPERATURE_TBL_SIZE-1].temperature_r
				&& pre_temp_idx >= TEMPERATURE_TBL_SIZE-1) {
		real_temp = FLASHLIGHT_Temperature_Table[TEMPERATURE_TBL_SIZE-1].ntc_temp * 1000;
	} else {
		uv_l = FLASHLIGHT_Temperature_Table[pre_temp_idx].temperature_r;
		uv_r = FLASHLIGHT_Temperature_Table[pre_temp_idx+1].temperature_r;
		real_temp = FLASHLIGHT_Temperature_Table[pre_temp_idx].ntc_temp * 1000 +
			 ((temp_uv - uv_l) * 1000) / (uv_r - uv_l);
	}

	*t = real_temp;

	mtk_flashlight_printk("[%s] T_flashlight real_temp, %d\n", __func__, real_temp);

	thermal->polling_delay = 0;  /* no-auto polling */

	return 0;
}

static int mtk_flashlight_bind(struct thermal_zone_device *thermal,
			     struct thermal_cooling_device *cdev)
{
	return 0;
}

static int mtk_flashlight_unbind(struct thermal_zone_device *thermal,
			       struct thermal_cooling_device *cdev)
{
	return 0;
}

static int mtk_flashlight_get_mode(
struct thermal_zone_device *thermal, enum thermal_device_mode *mode)
{
	*mode = (kernelmode) ? THERMAL_DEVICE_ENABLED : THERMAL_DEVICE_DISABLED;
	return 0;
}

static int mtk_flashlight_set_mode(
struct thermal_zone_device *thermal, enum thermal_device_mode mode)
{
	kernelmode = mode;
	return 0;
}

static int mtk_flashlight_get_trip_type(
struct thermal_zone_device *thermal, int trip, enum thermal_trip_type *type)
{
	*type = g_THERMAL_TRIP[trip];
	return 0;
}

static int mtk_flashlight_get_trip_temp(
struct thermal_zone_device *thermal, int trip, int *temp)
{
	*temp = trip_temp[trip];
	return 0;
}

static int mtk_flashlight_get_crit_temp(struct thermal_zone_device *thermal,
				      int *temperature)
{
	*temperature = MTK_FLASHLIGHT_TEMP_CRIT;
	return 0;
}

/* bind callback functions to thermalzone */
static struct thermal_zone_device_ops mtk_flashlight_dev_ops = {
	.bind = mtk_flashlight_bind,
	.unbind = mtk_flashlight_unbind,
	.get_temp = mtk_flashlight_get_temp,
	.get_mode = mtk_flashlight_get_mode,
	.set_mode = mtk_flashlight_set_mode,
	.get_trip_type = mtk_flashlight_get_trip_type,
	.get_trip_temp = mtk_flashlight_get_trip_temp,
	.get_crit_temp = mtk_flashlight_get_crit_temp,
};



static int mtk_flashlight_read(struct seq_file *m, void *v)
/* static int mtk_flashlight_read(
 * char *buf, char **start, off_t off, int count, int *eof, void *data)
 */
{
	seq_printf(m,
		"[mtk_flashlight_read] trip_0_temp=%d,trip_1_temp=%d,trip_2_temp=%d,trip_3_temp=%d,\n",
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

static int mtk_flashlight_register_thermal(void);
static void mtk_flashlight_unregister_thermal(void);

static ssize_t mtk_flashlight_write(
struct file *file, const char __user *buffer, size_t count, loff_t *data)
/* static ssize_t mtk_flashlight_write(
 * struct file *file, const char *buffer, int count, void *data)
 */
{
	pr_debug("Power/flashlight_Thermal: write, write, write!!!");
	pr_debug("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
	pr_debug("*****************************************");
	pr_debug("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
	return -EINVAL;
}


static int mtk_flashlight_register_thermal(void)
{
	mtk_flashlight_dprintk("[%s]\n", __func__);

	/* trips : trip 0~1 */
	thz_dev = mtk_thermal_zone_device_register("flashlight_tmp", num_trip,
						NULL, &mtk_flashlight_dev_ops,
						0, 0, 0, interval * 1000);

	return 0;
}

static void mtk_flashlight_unregister_thermal(void)
{
	mtk_flashlight_dprintk("[%s]\n", __func__);

	if (thz_dev) {
		mtk_thermal_zone_device_unregister(thz_dev);
		thz_dev = NULL;
	}
}

static int mtk_flashlight_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_flashlight_read, NULL);
}

static const struct file_operations mtk_flashlight_fops = {
	.owner = THIS_MODULE,
	.open = mtk_flashlight_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = mtk_flashlight_write,
	.release = single_release,
};

static int mtk_flashlight_probe(struct platform_device *pdev)
{
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *mtk_flashlight_dir = NULL;

	mtk_flashlight_dprintk("[%s]\n", __func__);

	if (!pdev->dev.of_node) {
		mtk_flashlight_dprintk("[%s]Only DT based supported\n",
		__func__);
		return -ENODEV;
	}

	mtk_flashlight_dir = mtk_thermal_get_proc_drv_therm_dir_entry();
	if (!mtk_flashlight_dir) {
		mtk_flashlight_dprintk(
			"[%s]: mkdir /proc/driver/thermal failed\n", __func__);
	} else {
		entry = proc_create("tzflashlight", 0664, mtk_flashlight_dir,
				&mtk_flashlight_fops);
		if (entry)
			proc_set_user(entry, uid, gid);
	}
	mtk_flashlight_register_thermal();
	return 0;
}

#ifdef CONFIG_OF
const struct of_device_id mt_thermistor_of_match_TS_chengdu[2] = {
	{.compatible = "mediatek,mtboard-thermistor_TS_chengdu", },
	{},
};
#endif

#define THERMAL_THERMISTOR_NAME    "mtboard-thermistor_TS_chengdu"
static struct platform_driver mtk_thermal_flashlight_driver = {
	.remove = NULL,
	.shutdown = NULL,
	.probe = mtk_flashlight_probe,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		.name = THERMAL_THERMISTOR_NAME,
#ifdef CONFIG_OF
		.of_match_table = mt_thermistor_of_match_TS_chengdu,
#endif
	},
};

static int __init mtk_flashlight_init(void)
{
	int err ;
	err = platform_driver_register(&mtk_thermal_flashlight_driver);
	if (err) {
		mtk_flashlight_dprintk("thermal driver callback register failed.\n");
		return err;
	}
	return 0;
}

static void __exit mtk_flashlight_exit(void)
{
	mtk_flashlight_dprintk("[%s]\n", __func__);
	mtk_flashlight_unregister_thermal();
}
module_init(mtk_flashlight_init);
module_exit(mtk_flashlight_exit);
