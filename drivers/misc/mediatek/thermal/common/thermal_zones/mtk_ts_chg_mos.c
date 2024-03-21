// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
 /**************************************************************************
 * Copyright (c) 2020-2030 OPLUS Mobile Comm Corp.,Ltd. All Rights Reserved.
 * OPLUS_FEATURE_CHG_BASIC
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
#include <soc/oplus/system/oplus_project.h>
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
	int ntc_temp;
	int temperaturer;
};

#define TEMPERATURE_TBL_SIZE 121
#define MTK_CHG_MOS_TEMP_CRIT 100
static struct NTC_TEMPERATURE CHG_MOS_Temperature_Table_2073x[] = {
	{-20, 1741010},
	{-19, 1737351},
	{-18, 1733501},
	{-17, 1729452},
	{-16, 1725198},
	{-15, 1720729},
	{-14, 1716039},
	{-13, 1711120},
	{-12, 1705964},
	{-11, 1700562},
	{-10, 1694908},
	{-9,  1688984},
	{-8, 1682791},
	{-7, 1676321},
	{-6, 1669568},
	{-5, 1662523},
	{-4, 1655189},
	{-3, 1647550},
	{-2, 1639598},
	{-1, 1631328},
	{0,  1622732},
	{1,  1613795},
	{2,  1604523},
	{3,  1594908},
	{4,  1584948},
	{5,  1574638},
	{6,  1563975},
	{7,  1552955},
	{8,  1541578},
	{9,  1529841},
	{10, 1517743},
	{11, 1505284},
	{12, 1492466},
	{13, 1479290},
	{14, 1465758},
	{15, 1451873},
	{16, 1437633},
	{17, 1423051},
	{18, 1408132},
	{19, 1392883},
	{20, 1377312},
	{21, 1361430},
	{22, 1345243},
	{23, 1328762},
	{24, 1311998},
	{25, 1294964},
	{26, 1277671},
	{27, 1260133},
	{28, 1242365},
	{29, 1224383},
	{30, 1206199},
	{31, 1187836},
	{32, 1169306},
	{33, 1150626},
	{34, 1131815},
	{35, 1112890},
	{36, 1093869},
	{37, 1074770},
	{38, 1055611},
	{39, 1036411},
	{40, 1017187},
	{41, 997973},
	{42, 978774},
	{43, 959608},
	{44, 940490},
	{45, 921440},
	{46, 902453},
	{47, 883562},
	{48, 864784},
	{49, 846134},
	{50, 827628},
	{51, 809301},
	{52, 791145},
	{53, 773173},
	{54, 755397},
	{55, 737826},
	{56, 720471},
	{57, 703342},
	{58, 686449},
	{59, 669798},
	{60, 653396},
	{61, 637253},
	{62, 621370},
	{63, 605756},
	{64, 590414},
	{65, 575349},
	{66, 560595},
	{67, 546124},
	{68, 531936},
	{69, 518033},
	{70, 504416},
	{71, 491055},
	{72, 477981},
	{73, 465191},
	{74, 452686},
	{75, 440467},
	{76, 428560},
	{77, 416931},
	{78, 405582},
	{79, 394504},
	{80, 383698},
	{81, 373159},
	{82, 362886},
	{83, 352869},
	{84, 343112},
	{85, 333607},
	{86, 324348},
	{87, 315335},
	{88, 306557},
	{89, 298019},
	{90, 289708},
	{91, 281650},
	{92, 273816},
	{93, 266193},
	{94, 258782},
	{95, 251578},
	{96, 244548},
	{97, 237712},
	{98, 231071},
	{99, 224617},
	{100, 218345},
};

static struct NTC_TEMPERATURE CHG_MOS_Temperature_Table[] = {
	{-20, 1790339},
	{-19, 1789683},
	{-18, 1788989},
	{-17, 1788254},
	{-16, 1787477},
	{-15, 1786655},
	{-14, 1785786},
	{-13, 1784869},
	{-12, 1783901},
	{-11, 1782880},
	{-10, 1781803},
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
	{100, 820646},
};

static int size = 120;


static struct NTC_TEMPERATURE CHG_MOS_Temperature_Table_21061[] = {
	{-40, 1795775},
	{-39, 1795468},
	{-38, 1795141},
	{-37, 1794793},
	{-36, 1794423},
	{-35, 1794029},
	{-34, 1793610},
	{-33, 1793166},
	{-32, 1792694},
	{-31, 1792194},
	{-30, 1791662},
	{-29, 1791097},
	{-28, 1790501},
	{-27, 1789870},
	{-26, 1789195},
	{-25, 1788491},
	{-24, 1787738},
	{-23, 1786947},
	{-22, 1786111},
	{-21, 1785221},
	{-20, 1784279},
	{-19, 1783302},
	{-18, 1782248},
	{-17, 1781151},
	{-16, 1779991},
	{-15, 1778768},
	{-14, 1777480},
	{-13, 1776124},
	{-12, 1774694},
	{-11, 1773194},
	{-10, 1771613},
	{-9, 1769949},
	{-8, 1768209},
	{-7, 1766373},
	{-6, 1764447},
	{-5, 1762429},
	{-4, 1760308},
	{-3, 1758090},
	{-2, 1755763},
	{-1, 1753319},
	{0, 1750765},
	{1, 1748096},
	{2, 1745305},
	{3, 1742381},
	{4, 1739332},
	{5, 1736124},
	{6, 1732785},
	{7, 1729328},
	{8, 1725681},
	{9, 1721908},
	{10, 1717958},
	{11, 1713834},
	{12, 1709547},
	{13, 1705113},
	{14, 1700497},
	{15, 1695652},
	{16, 1690643},
	{17, 1685496},
	{18, 1680079},
	{19, 1674476},
	{20, 1668613},
	{21, 1662595},
	{22, 1656459},
	{23, 1650000},
	{24, 1643205},
	{25, 1636363},
	{26, 1629222},
	{27, 1621852},
	{28, 1614241},
	{29, 1606389},
	{30, 1598274},
	{31, 1589940},
	{32, 1581341},
	{33, 1572497},
	{34, 1563375},
	{35, 1554031},
	{36, 1544390},
	{37, 1534552},
	{38, 1524391},
	{39, 1514013},
	{40, 1503361},
	{41, 1492465},
	{42, 1481359},
	{43, 1469906},
	{44, 1458249},
	{45, 1446365},
	{46, 1434220},
	{47, 1421848},
	{48, 1409205},
	{49, 1396412},
	{50, 1383236},
	{51, 1369995},
	{52, 1356431},
	{53, 1342682},
	{54, 1328795},
	{55, 1314693},
	{56, 1300416},
	{57, 1285861},
	{58, 1271210},
	{59, 1256357},
	{60, 1241340},
	{61, 1226203},
	{62, 1210994},
	{63, 1195567},
	{64, 1180165},
	{65, 1164406},
	{66, 1148769},
	{67, 1133086},
	{68, 1117147},
	{69, 1101242},
	{70, 1085146},
	{71, 1069183},
	{72, 1053112},
	{73, 1037288},
	{74, 1021116},
	{75, 1004946},
	{76, 988823},
	{77, 972794},
	{78, 956908},
	{79, 941221},
	{80, 925364},
	{81, 909351},
	{82, 893609},
	{83, 877963},
	{84, 862402},
	{85, 846963},
	{86, 831581},
	{87, 816339},
	{88, 801220},
	{89, 786258},
	{90, 771428},
	{91, 756763},
	{92, 742172},
	{93, 727805},
	{94, 713568},
	{95, 699553},
	{96, 685655},
	{97, 671968},
	{98, 658447},
	{99, 645117},
	{100, 631927},
	{101, 618975},
	{102, 606207},
	{103, 593646},
	{104, 581230},
	{105, 569062},
	{106, 557077},
	{107, 545294},
	{108, 533731},
	{109, 522316},
	{110, 511137},
	{111, 500154},
	{112, 489374},
	{113, 478792},
	{114, 468402},
	{115, 458218},
	{116, 448232},
	{117, 438436},
	{118, 428843},
	{119, 419441},
	{120, 410221},
	{121, 401202},
	{122, 392353},
	{123, 383703},
	{124, 375227},
	{125, 366936},
};


static void mtk_chg_mos_ntc_switch(void)
{
	int i;
	for (i = size - 1; i >= 0; i--)
		CHG_MOS_Temperature_Table[i].temperaturer =
				CHG_MOS_Temperature_Table_2073x[i].temperaturer;
}


static int mtk_chg_mos_get_temp(struct thermal_zone_device *thermal, int *t)
{
	int i, real_temp, temp_uv;
	int next_volt, now_volt;
	int now_temp_idx;
	int voocphy_size = 0;

	int ret = mt6360_get_chg_thermal_temperature(&temp_uv);

	pr_err("[%s] T_chg get_temp_status %d, temp_uv, %d \n", __func__, ret, temp_uv);

	if (get_project() == 21061) {
		voocphy_size = (sizeof(CHG_MOS_Temperature_Table_21061) / sizeof(CHG_MOS_Temperature_Table_21061[0]));

		for (i = voocphy_size- 1; i >= 0; i--) {
			if (CHG_MOS_Temperature_Table_21061[i].temperaturer >= temp_uv)
				break;
			else if (i == 0)
				break;
		}

		if (i < voocphy_size- 1 && i > 0) {
			now_volt = CHG_MOS_Temperature_Table_21061[i].temperaturer;
			next_volt = CHG_MOS_Temperature_Table_21061[i+1].temperaturer;
			real_temp = (now_volt - temp_uv) * 1000/(now_volt - next_volt) + CHG_MOS_Temperature_Table_21061[i].ntc_temp * 1000;
		} else if (i <= 0) {
			real_temp = CHG_MOS_Temperature_Table_21061[0].ntc_temp;
		} else {
			real_temp = CHG_MOS_Temperature_Table_21061[voocphy_size- 1].ntc_temp;
		}

		pr_err("[%s] i= %d, real_temp, %d voocphy_size =%d  temp_uv =%d\n", __func__, i, real_temp, voocphy_size, temp_uv);
	} else {
		real_temp = CHG_MOS_Temperature_Table[size].ntc_temp * 1000;

		for (i = size - 1; i > 0; i--) {
			if (temp_uv > CHG_MOS_Temperature_Table[i].temperaturer) {
			} else {
				i = (i + 1) <= 120 ? (i + 1) : 120;
				now_temp_idx = i - 1;
				next_volt = CHG_MOS_Temperature_Table[i].temperaturer;
				now_volt = CHG_MOS_Temperature_Table[now_temp_idx].temperaturer;
				real_temp = (now_volt - temp_uv) * 1000/(now_volt - next_volt) + CHG_MOS_Temperature_Table[now_temp_idx].ntc_temp * 1000;
				break;
			}
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
	*temperature = MTK_CHG_MOS_TEMP_CRIT;
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
	if (get_project() >= 20730 && get_project() <= 20733)
		mtk_chg_mos_ntc_switch();

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

