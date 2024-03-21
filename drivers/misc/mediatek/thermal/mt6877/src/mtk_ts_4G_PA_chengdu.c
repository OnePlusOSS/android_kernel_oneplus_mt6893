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
#include <tmp_bts.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
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
static int mtk_PA_4G_debug_log = 1;
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

/* static int PA_4G_write_flag=0; */
struct iio_channel *thermistor_ch1_1;
static int g_adc_channel1;

static int polling_trip_temp1 = 40000;
static int polling_trip_temp2 = 20000;
static int polling_factor1 = 5000;
static int polling_factor2 = 10000;

#define PA_4G_RAP_ADC_CHANNEL			(1)
#define PA_4G_RAP_PULL_UP_VOLTAGE		1800
#define PA_4G_RAP_PULL_UP_R			100000
#define PA_4G_TAP_OVER_CRITICAL_LOW	4397119


#define MTK_PA_4G_TEMP_CRIT 120000	/* 120.000 degree Celsius */

#define mtkts_PA_4G_dprintk(fmt, args...)   \
do {									\
	if (mtk_PA_4G_debug_log) {				\
		pr_debug("[Thermal/TZ/PA_4G]" fmt, ##args); \
	}								   \
} while (0)

#define mtkts_PA_4G_printk(fmt, args...)   \
pr_err("[Thermal/TZ/PA_4G]" fmt, ##args)

struct PA_4G_TEMPERATURE {
	__s32 PA_4G_Temp;
	__s32 TemperatureR;
};

static int g_rap_pull_up_r = PA_4G_RAP_PULL_UP_R;
static int g_tap_over_critical_low = PA_4G_TAP_OVER_CRITICAL_LOW;
static int g_rap_pull_up_voltage = PA_4G_RAP_PULL_UP_VOLTAGE;
static int g_rap_adc_channel = PA_4G_RAP_ADC_CHANNEL;

static int g_PA_4G_temperature_r;
/* struct PA_4G_TempERATURE PA_4G_Temperature_Table[] = {0}; */
#define TEMPERATURE_TBL_SIZE 121

static struct PA_4G_TEMPERATURE PA_4G_Temperature_Table[] = {
	{-40, 4397119},
	{-39, 4092874},
	{-38, 3811717},
	{-37, 3551749},
	{-36, 3311236},
	{-35, 3088599},
	{-34, 2882396},
	{-33, 2691310},
	{-32, 2514137},
	{-31, 2349778},
	{-30, 2197225},
	{-29, 2055558},
	{-28, 1923932},
	{-27, 1801573},
	{-26, 1687773},
	{-25, 1581881},
	{-24, 1483100},
	{-23, 1391113},
	{-22, 1305413},
	{-21, 1225531},
	{-20, 1151037},
	{-19, 1081535},
	{-18, 1016661},
	{-17, 956080},
	{-16, 899481},
	{-15, 846579},
	{-14, 797111},
	{-13, 750834},
	{-12, 707524},
	{-11, 666972},
	{-10, 628988},
	{-9,  593342},
	{-8,  559931},
	{-7,  528602},
	{-6,  499212},
	{-5,  471632},
	{-4,  445772},
	{-3,  421480},
	{-2,  398652},
	{-1,  377193},
	{0,   357012},
	{1,   338006},
	{2,   320122},
	{3,   303287},
	{4,   287434},
	{5,   272500},
	{6,   258426},
	{7,   245160},
	{8,   232649},
	{9,   220847},
	{10,  209710},
	{11,  199196},
	{12,  189268},
	{13,  179890},
	{14,  171028},
	{15,  162651},
	{16,  154726},
	{17,  147232},
	{18,  140142},
	{19,  133432},
	{20,  127080},
	{21,  121066},
	{22,  115368},
	{23,  109970},
	{24,  104852},
	{25,  100000},
	{26,  95398},
	{27,  91032},
	{28,  86889},
	{29,  82956},
	{30,  79221},
	{31,  75675},
	{32,  72306},
	{33,  69104},
	{34,  66061},
	{35,  63167},
	{36,  60218},
	{37,  57797},
	{38,  55306},
	{39,  52934},
	{40,  50677},
	{41,  48528},
	{42,  46482},
	{43,  44533},
	{44,  42675},
	{45,  40904},
	{46,  39213},
	{47,  37601},
	{48,  36063},
	{49,  34595},
	{50,  33195},
	{51,  31859},
	{52,  30584},
	{53,  29366},
	{54,  28203},
	{55,  27091},
	{56,  26028},
	{57,  25013},
	{58,  24042},
	{59,  23113},
	{60,  22224},
	{61,  21374},
	{62,  20561},
	{63,  19782},
	{64,  19036},
	{65,  18323},
	{66,  17640},
	{67,  16986},
	{68,  16360},
	{69,  15760},
	{70,  15184},
	{71,  14631},
	{72,  14101},
	{73,  13592},
	{74,  13104},
	{75,  12635},
	{76,  12187},
	{77,  11757},
	{78,  11344},
	{79,  10947},
	{80,  10566},
	{81,  10200},
	{82,  9848},
	{83,  9510},
	{84,  9185},
	{85,  8872},
	{86,  8572},
	{87,  8283},
	{88,  8006},
	{89,  7738},
	{90,  7481},
	{91,  7234},
	{92,  6997},
	{93,  6769},
	{94,  6548},
	{95,  6337},
	{96,  6137},
	{97,  5934},
	{98,  5744},
	{99,  5561},
	{100, 5384},
	{101, 5214},
	{102, 5051},
	{103, 4893},
	{104, 4741},
	{105, 4594},
	{106, 4453},
	{107, 4316},
	{108, 4184},
	{109, 4057},
	{110, 3934},
	{111, 3816},
	{112, 3701},
	{113, 3591},
	{114, 3484},
	{115, 3380},
	{116, 3281},
	{117, 3185},
	{118, 3093},
	{119, 3003},
	{120, 2916},
	{121, 2832},
	{122, 2751},
	{123, 2672},
	{124, 2596},
	{125, 2522}
};

/* convert register to temperature  */
static __s16 mtkts_PA_4G_thermistor_conver_temp(__s32 Res)
{
	int i = 0;
	int asize = 0;
	__s32 RES1 = 0, RES2 = 0;
	__s32 TAP_Value = -2000, TMP1 = 0, TMP2 = 0;

	asize = (sizeof(PA_4G_Temperature_Table) / sizeof(struct PA_4G_TEMPERATURE));

	if (Res >= PA_4G_Temperature_Table[0].TemperatureR) {
		TAP_Value = -400;	/* min */
	} else if (Res <= PA_4G_Temperature_Table[asize - 1].TemperatureR) {
		TAP_Value = 1250;	/* max */
	} else {
		RES1 = PA_4G_Temperature_Table[0].TemperatureR;
		TMP1 = PA_4G_Temperature_Table[0].PA_4G_Temp;
		/* mtkts_PA_4G_dprintk("%d : RES1 = %d,TMP1 = %d\n",__LINE__,
		 * RES1,TMP1);
		 */

		for (i = 0; i < asize; i++) {
			if (Res >= PA_4G_Temperature_Table[i].TemperatureR) {
				RES2 = PA_4G_Temperature_Table[i].TemperatureR;
				TMP2 = PA_4G_Temperature_Table[i].PA_4G_Temp;
				/* mtkts_PA_4G_dprintk("%d :i=%d, RES2 = %d,
				 * TMP2 = %d\n",__LINE__,i,RES2,TMP2);
				 */
				break;
			}
			RES1 = PA_4G_Temperature_Table[i].TemperatureR;
			TMP1 = PA_4G_Temperature_Table[i].PA_4G_Temp;
			/* mtkts_PA_4G_dprintk("%d :i=%d, RES1 = %d,
			 * TMP1 = %d\n",__LINE__,i,RES1,TMP1);
			 */
		}

		TAP_Value = (((Res - RES2) * TMP1) + ((RES1 - Res) * TMP2)) * 10
								/ (RES1 - RES2);
	}

	return TAP_Value;
}

/* convert ADC_AP_temp_volt to register */
/*Volt to Temp formula same with 6589*/
static __s16 mtk_ts_PA_4G_volt_to_temp(__u32 dwVolt)
{
	__s32 TRes;
	__u64 dwVCriAP = 0;
	__u64 dwVCriAP2 = 0;
	__s32 PA_4G_TMP = -100;

	/* SW workaround-----------------------------------------------------
	 * dwVCriAP = (TAP_OVER_CRITICAL_LOW * 1800) /
	 * (TAP_OVER_CRITICAL_LOW + 39000);
	 * dwVCriAP = (TAP_OVER_CRITICAL_LOW * RAP_PULL_UP_VOLT) /
	 * (TAP_OVER_CRITICAL_LOW + RAP_PULL_UP_R);
	 */

	dwVCriAP = ((__u64)g_tap_over_critical_low *
		(__u64)g_rap_pull_up_voltage);
	dwVCriAP2 = (g_tap_over_critical_low + g_rap_pull_up_r);
	do_div(dwVCriAP, dwVCriAP2);


	if (dwVolt > ((__u32)dwVCriAP)) {
		TRes = g_tap_over_critical_low;
	} else {
		/* TRes = (39000*dwVolt) / (1800-dwVolt);
		 * TRes = (RAP_PULL_UP_R*dwVolt) / (RAP_PULL_UP_VOLT-dwVolt);
		 */
		TRes = (g_rap_pull_up_r * dwVolt)
				/ (g_rap_pull_up_voltage - dwVolt);
	}
	/* ------------------------------------------------------------------ */

	g_PA_4G_temperature_r = TRes;

	/* convert register to temperature */
	PA_4G_TMP = mtkts_PA_4G_thermistor_conver_temp(TRes);

	return PA_4G_TMP;
}

static int get_hw_PA_4G_temp(void)
{
	int val = 0;
	int ret = 0, output;

	if (!thermistor_ch1_1) {
		mtkts_PA_4G_dprintk("thermistor_ch1_1 NULL\n");
		return -1;
	}
	ret = iio_read_channel_processed(thermistor_ch1_1, &val);
	mtkts_PA_4G_dprintk("%s val=%d\n", __func__, val);

	if (ret < 0) {
		mtkts_PA_4G_dprintk("IIO channel read failed %d\n", ret);
		return ret;
	}
	/* ret = ret*1800/4096; 82's ADC power */
	mtkts_PA_4G_dprintk("APtery output mV = %d\n", val);
	output = mtk_ts_PA_4G_volt_to_temp(val);
	mtkts_PA_4G_dprintk("PA_4G output temperature = %d\n", output);
	return output;
}

static DEFINE_MUTEX(PA_4G_lock);
/*int ts_PA_4G_at_boot_time = 0;*/
int mtkts_PA_4G_get_hw_temp(void)
{
	int t_ret = 0;

	mutex_lock(&PA_4G_lock);

	/* get HW AP temp (TSAP) */
	/* cat /sys/class/power_supply/AP/AP_temp */
	t_ret = get_hw_PA_4G_temp();
	t_ret = t_ret * 100;

	mutex_unlock(&PA_4G_lock);

	if (t_ret > 40000)	/* abnormal high temp */
		mtkts_PA_4G_dprintk("T_PA_4G=%d\n", t_ret);

	mtkts_PA_4G_dprintk("[%s] T_PA_4G, %d\n", __func__,
									t_ret);
	return t_ret;
}

static int mtk_PA_4G_get_temp(struct thermal_zone_device *thermal, int *t)
{
	*t = mtkts_PA_4G_get_hw_temp();

	if ((int)*t > 52000)
		mtkts_PA_4G_dprintk("T=%d\n", (int)*t);

	if ((int)*t >= polling_trip_temp1)
		thermal->polling_delay = interval * 1000;
	else if ((int)*t < polling_trip_temp2)
		thermal->polling_delay = interval * polling_factor2;
	else
		thermal->polling_delay = interval * polling_factor1;

	return 0;
}

static int mtk_PA_4G_bind(struct thermal_zone_device *thermal,
				 struct thermal_cooling_device *cdev)
{
	/*int table_val = 0;

	if (!strcmp(cdev->type, g_bind0)) {
		table_val = 0;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind1)) {
		table_val = 1;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind2)) {
		table_val = 2;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind3)) {
		table_val = 3;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind4)) {
		table_val = 4;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind5)) {
		table_val = 5;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind6)) {
		table_val = 6;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind7)) {
		table_val = 7;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind8)) {
		table_val = 8;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind9)) {
		table_val = 9;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else {
		return 0;
	}

	if (mtk_thermal_zone_bind_cooling_device(thermal, table_val, cdev)) {
		mtkts_PA_4G_dprintk(
			"[%s] error binding cooling dev\n", __func__);
		return -EINVAL;
	}

	mtkts_PA_4G_dprintk("[%s] binding OK, %d\n", __func__, table_val);*/
	return 0;
}

static int mtk_PA_4G_unbind(struct thermal_zone_device *thermal,
				   struct thermal_cooling_device *cdev)
{
	/*int table_val = 0;

	if (!strcmp(cdev->type, g_bind0)) {
		table_val = 0;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind1)) {
		table_val = 1;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind2)) {
		table_val = 2;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind3)) {
		table_val = 3;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind4)) {
		table_val = 4;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind5)) {
		table_val = 5;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind6)) {
		table_val = 6;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind7)) {
		table_val = 7;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind8)) {
		table_val = 8;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind9)) {
		table_val = 9;
		mtkts_PA_4G_dprintk("[%s] %s\n", __func__, cdev->type);
	} else
		return 0;

	if (thermal_zone_unbind_cooling_device(thermal, table_val, cdev)) {
		mtkts_PA_4G_dprintk(
				"[%s] error unbinding cooling dev\n", __func__);

		return -EINVAL;
	}

	mtkts_PA_4G_dprintk("[%s] unbinding OK\n", __func__);*/
	return 0;
}

static int mtk_PA_4G_get_mode(
struct thermal_zone_device *thermal, enum thermal_device_mode *mode)
{
	*mode = (kernelmode) ? THERMAL_DEVICE_ENABLED : THERMAL_DEVICE_DISABLED;
	return 0;
}

static int mtk_PA_4G_set_mode(
struct thermal_zone_device *thermal, enum thermal_device_mode mode)
{
	kernelmode = mode;
	return 0;
}

static int mtk_PA_4G_get_trip_type(
struct thermal_zone_device *thermal, int trip, enum thermal_trip_type *type)
{
	*type = g_THERMAL_TRIP[trip];
	return 0;
}

static int mtk_PA_4G_get_trip_temp(
struct thermal_zone_device *thermal, int trip, int *temp)
{
	*temp = trip_temp[trip];
	return 0;
}

static int mtk_PA_4G_get_crit_temp(struct thermal_zone_device *thermal,
					  int *temperature)
{
	*temperature = MTK_PA_4G_TEMP_CRIT;
	return 0;
}

/* bind callback functions to thermalzone */
static struct thermal_zone_device_ops mtk_PA_4G_dev_ops = {
	.bind = mtk_PA_4G_bind,
	.unbind = mtk_PA_4G_unbind,
	.get_temp = mtk_PA_4G_get_temp,
	.get_mode = mtk_PA_4G_get_mode,
	.set_mode = mtk_PA_4G_set_mode,
	.get_trip_type = mtk_PA_4G_get_trip_type,
	.get_trip_temp = mtk_PA_4G_get_trip_temp,
	.get_crit_temp = mtk_PA_4G_get_crit_temp,
};

static int mtkts_PA_4G_param_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", g_rap_pull_up_r);
	seq_printf(m, "%d\n", g_rap_pull_up_voltage);
	seq_printf(m, "%d\n", g_tap_over_critical_low);
	seq_printf(m, "%d\n", g_adc_channel1);

	return 0;
}


static ssize_t mtkts_PA_4G_param_write(
struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	int len = 0;
	struct param_data {
		char desc[512];
		char pull_R[10], pull_V[10];
		char overcrilow[16];
		char NTC_TABLE[10];
		unsigned int valR, valV, over_cri_low, ntc_table;
	};
	/* external pin: 0/1/12/13/14/15, can't use pin:2/3/4/5/6/7/8/9/10/11,
	 * choose "adc_channel=11" to check if there is any param input
	 */
	unsigned int adc_channel = 11;

	struct param_data *ptr_param_data = kmalloc(
					sizeof(*ptr_param_data), GFP_KERNEL);

	if (ptr_param_data == NULL)
		return -ENOMEM;

	len = (count < (sizeof(ptr_param_data->desc) - 1)) ?
				count : (sizeof(ptr_param_data->desc) - 1);

	if (copy_from_user(ptr_param_data->desc, buffer, len)) {
		kfree(ptr_param_data);
		return 0;
	}
	ptr_param_data->desc[len] = '\0';

	mtkts_PA_4G_dprintk("[%s]\n", __func__);

	if (sscanf
	    (ptr_param_data->desc, "%9s %d %9s %d %15s %d %9s %d %d",
			ptr_param_data->pull_R, &ptr_param_data->valR,
			ptr_param_data->pull_V, &ptr_param_data->valV,
			ptr_param_data->overcrilow,
			&ptr_param_data->over_cri_low,
			ptr_param_data->NTC_TABLE,
			&ptr_param_data->ntc_table, &adc_channel) >= 8) {
		if (!strcmp(ptr_param_data->pull_R, "PUP_R")) {
			g_rap_pull_up_r = ptr_param_data->valR;
			mtkts_PA_4G_dprintk("g_rap_pull_up_r=%d\n",
							g_rap_pull_up_r);
		} else {
			kfree(ptr_param_data);
			mtkts_PA_4G_dprintk(
					"[mtkts_PA_4G_write] bad PUP_R argument\n");
			return -EINVAL;
		}

		if (!strcmp(ptr_param_data->pull_V, "PUP_VOLT")) {
			g_rap_pull_up_voltage = ptr_param_data->valV;
			mtkts_PA_4G_dprintk("g_Rat_pull_up_voltage=%d\n",
							g_rap_pull_up_voltage);
		} else {
			kfree(ptr_param_data);
			mtkts_PA_4G_dprintk(
					"[mtkts_PA_4G_write] bad PUP_VOLT argument\n");
			return -EINVAL;
		}

		if (!strcmp(ptr_param_data->overcrilow, "OVER_CRITICAL_L")) {
			g_tap_over_critical_low = ptr_param_data->over_cri_low;
			mtkts_PA_4G_dprintk("g_tap_over_critical_low=%d\n",
					      g_tap_over_critical_low);
		} else {
			kfree(ptr_param_data);
			mtkts_PA_4G_dprintk(
					"[mtkts_PA_4G_write] bad OVERCRIT_L argument\n");
			return -EINVAL;
		}

		if (!strcmp(ptr_param_data->NTC_TABLE, "NTC_TABLE")) {
			mtkts_PA_4G_dprintk("ntc_table=%d\n",
							ptr_param_data->ntc_table);
		} else {
			kfree(ptr_param_data);
			mtkts_PA_4G_dprintk(
					"[mtkts_PA_4G_write] bad NTC_TABLE argument\n");
			return -EINVAL;
		}

		/* external pin: 0/1/12/13/14/15,
		 * can't use pin:2/3/4/5/6/7/8/9/10/11,
		 * choose "adc_channel=11" to check if there is any param input
		 */
		if ((adc_channel >= 2) && (adc_channel <= 11))
			/* check unsupport pin value, if unsupport,
			 * set channel = 1 as default setting.
			 */
			g_rap_adc_channel = AUX_IN1_NTC;
		else {
			g_rap_adc_channel = adc_channel;
		}
		mtkts_PA_4G_dprintk("adc_channel=%d\n", adc_channel);
		mtkts_PA_4G_dprintk("g_rap_adc_channel=%d\n",
						g_rap_adc_channel);

		kfree(ptr_param_data);
		return count;
	}

	mtkts_PA_4G_dprintk("[mtkts_PA_4G_write] bad argument\n");
	kfree(ptr_param_data);
	return -EINVAL;
}

static int mtkts_PA_4G_param_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtkts_PA_4G_param_read, NULL);
}

static int mtk_PA_4G_read(struct seq_file *m, void *v)
{
	seq_printf(m,
		"[mtk_PA_4G_read] trip_0_temp=%d,trip_1_temp=%d,trip_2_temp=%d,trip_3_temp=%d,\n",
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

/* static ssize_t mtk_PA_4G_write(
 * struct file *file, const char *buffer, int count, void *data)
 */
static ssize_t mtk_PA_4G_write(
struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	pr_debug("Power/PA_4G_Thermal: write, write, write!!!");
	pr_debug("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
	pr_debug("*****************************************");
	pr_debug("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
	return -EINVAL;
}

static int mtk_PA_4G_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_PA_4G_read, NULL);
}

static const struct file_operations mtkts_PA_4G_fops = {
	.owner = THIS_MODULE,
	.open = mtk_PA_4G_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = mtk_PA_4G_write,
	.release = single_release,
};

static const struct file_operations mtkts_PA_4G_param_fops = {
	.owner = THIS_MODULE,
	.open = mtkts_PA_4G_param_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = mtkts_PA_4G_param_write,
	.release = single_release,
};

static int mtk_PA_4G_register_thermal(void)
{
	mtkts_PA_4G_dprintk("[%s]\n", __func__);

	/* trips : trip 0~1 */
	thz_dev = mtk_thermal_zone_device_register("PA_4G_tmp", num_trip,
						NULL, &mtk_PA_4G_dev_ops,
						0, 0, 0, interval * 1000);

	return 0;
}

static void mtk_PA_4G_unregister_thermal(void)
{
	mtkts_PA_4G_dprintk("[%s]\n", __func__);

	if (thz_dev) {
		mtk_thermal_zone_device_unregister(thz_dev);
		thz_dev = NULL;
	}
}

static int mtkts_PA_4G_probe(struct platform_device *pdev)
{
	int err = 0;
	int ret = 0;
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *mtkts_PA_4G_dir = NULL;

	mtkts_PA_4G_dprintk("[%s]\n", __func__);

	if (!pdev->dev.of_node) {
		mtkts_PA_4G_dprintk("[%s]Only DT based supported\n",
		__func__);
		return -ENODEV;
	}

	thermistor_ch1_1 = devm_kzalloc(&pdev->dev, sizeof(*thermistor_ch1_1),
		GFP_KERNEL);
	if (!thermistor_ch1_1)
		return -ENOMEM;

	thermistor_ch1_1 = iio_channel_get(&pdev->dev, "thermistor-ch1");
	ret = IS_ERR(thermistor_ch1_1);
	if (ret) {
		mtkts_PA_4G_dprintk("[%s] fail to get auxadc iio ch1: %d\n",
			__func__, ret);
		thermistor_ch1_1 = NULL;
		return ret;
	}

	g_adc_channel1 = thermistor_ch1_1->channel->channel;
	mtkts_PA_4G_dprintk("[%s]get auxadc iio ch: %d\n", __func__,
		thermistor_ch1_1->channel->channel);

	mtkts_PA_4G_dir = mtk_thermal_get_proc_drv_therm_dir_entry();
	if (!mtkts_PA_4G_dir) {
		mtkts_PA_4G_dprintk(
			"[%s]: mkdir /proc/driver/thermal failed\n", __func__);
	} else {
		entry = proc_create("tzPA_4G", 0664, mtkts_PA_4G_dir,
				&mtkts_PA_4G_fops);
		if (entry)
			proc_set_user(entry, uid, gid);

		entry = proc_create("tzPA_4G_param", 0664, mtkts_PA_4G_dir,
				&mtkts_PA_4G_param_fops);
		if (entry)
			proc_set_user(entry, uid, gid);
	}
	mtk_PA_4G_register_thermal();

	return err;
}

#ifdef CONFIG_OF
const struct of_device_id mt_thermistor_of_match_4GPA[2] = {
	{.compatible = "mediatek,mtboard-thermistor2_chengdu", },
	{},
};
#endif

#define THERMAL_THERMISTOR_NAME    "mtboard-thermistor2_chengdu"
static struct platform_driver mtk_thermal_PA_4G_driver = {
	.remove = NULL,
	.shutdown = NULL,
	.probe = mtkts_PA_4G_probe,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		.name = THERMAL_THERMISTOR_NAME,
#ifdef CONFIG_OF
		.of_match_table = mt_thermistor_of_match_4GPA,
#endif
	},
};

static int __init mtk_PA_4G_init(void)
{
	int err = 0;

	err = platform_driver_register(&mtk_thermal_PA_4G_driver);
	if (err) {
		mtkts_PA_4G_dprintk("thermal driver callback register failed.\n");
		return err;
	}

	return 0;
}

static void __exit mtk_PA_4G_exit(void)
{
	mtkts_PA_4G_dprintk("[%s]\n", __func__);
	mtk_PA_4G_unregister_thermal();
}
module_init(mtk_PA_4G_init);
module_exit(mtk_PA_4G_exit);
