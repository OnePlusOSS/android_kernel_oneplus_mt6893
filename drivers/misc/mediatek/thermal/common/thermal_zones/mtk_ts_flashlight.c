// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
 /**************************************************************************
 * Copyright (c) 2020-2030 OPLUS Mobile Comm Corp.,Ltd. All Rights Reserved.
 * OPLUS_FEATURE_CAMERA_COMMON
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
struct iio_channel *thermistor_ch3;
static int g_adc_channel3;

static int polling_trip_temp1 = 40000;
static int polling_trip_temp2 = 20000;
static int polling_factor1 = 5000;
static int polling_factor2 = 10000;

#define MTK_FLASHLIGHT_TEMP_CRIT 120000	/* 120.000 degree Celsius */

#define mtkts_flashlight_dprintk(fmt, args...)   \
do {									\
	if (mtk_flashlight_debug_log) {				\
		pr_debug("[Thermal/TZ/flashlight]" fmt, ##args); \
	}								   \
} while (0)

#define mtkts_flashlight_printk(fmt, args...)   \
pr_err("[Thermal/TZ/flashlight]" fmt, ##args)

struct FLASHLIGHT_TEMPERATURE {
	__s32 FLASHLIGHT_Temp;
	__s32 TemperatureR;
};

static int g_rap_pull_up_r = FLASHLIGHT_RAP_PULL_UP_R;
static int g_tap_over_critical_low = FLASHLIGHT_TAP_OVER_CRITICAL_LOW;
static int g_rap_pull_up_voltage = FLASHLIGHT_RAP_PULL_UP_VOLTAGE;
static int g_rap_adc_channel = FLASHLIGHT_RAP_ADC_CHANNEL;

static int g_flashlight_temperature_r;
/* struct FLASHLIGHT_TempERATURE FLASHLIGHT_Temperature_Table[] = {0}; */
#define TEMPERATURE_TBL_SIZE 121

static struct FLASHLIGHT_TEMPERATURE FLASHLIGHT_Temperature_Table[] = {
	{-20, 1260250},
	{-19, 1179691},
	{-18, 1104854},
	{-17, 1035294},
	{-16, 970604},
	{-15, 910411},
	{-14, 854373},
	{-13, 802176},
	{-12, 753532},
	{-11, 708176},
	{-10, 665863},
	{-9,  626371},
	{-8,  589493},
	{-7,  555039},
	{-6,  522834},
	{-5,  492718},
	{-4,  464541},
	{-3,  438167},
	{-2,  413468},
	{-1,  390328},
	{0,   368638},
	{1,   348299},
	{2,   329218},
	{3,   311309},
	{4,   294493},
	{5,   278697},
	{6,   263852},
	{7,   249895},
	{8,   236769},
	{9,   224417},
	{10,  212791},
	{11,  201842},
	{12,  191528},
	{13,  181807},
	{14,  172643},
	{15,  163999},
	{16,  155843},
	{17,  148145},
	{18,  140876},
	{19,  134011},
	{20,  127523},
	{21,  121390},
	{22,  115591},
	{23,  110105},
	{24,  104914},
	{25,  100000},
	{26,  95346},
	{27,  90938},
	{28,  86762},
	{29,  82802},
	{30,  79048},
	{31,  75487},
	{32,  72108},
	{33,  68901},
	{34,  65857},
	{35,  62965},
	{36,  60218},
	{37,  57607},
	{38,  55125},
	{39,  52765},
	{40,  50520},
	{41,  48384},
	{42,  46350},
	{43,  44415},
	{44,  42572},
	{45,  40816},
	{46,  39143},
	{47,  37548},
	{48,  36027},
	{49,  34577},
	{50,  33194},
	{51,  31874},
	{52,  30615},
	{53,  29412},
	{54,  28263},
	{55,  27166},
	{56,  26118},
	{57,  25116},
	{58,  24159},
	{59,  23243},
	{60,  22367},
	{61,  21529},
	{62,  20728},
	{63,  19960},
	{64,  19226},
	{65,  18522},
	{66,  17848},
	{67,  17203},
	{68,  16584},
	{69,  15991},
	{70,  15422},
	{71,  14877},
	{72,  14354},
	{73,  13853},
	{74,  13371},
	{75,  12909},
	{76,  12466},
	{77,  12040},
	{78,  11631},
	{79,  11238},
	{80,  10860},
	{81,  10497},
	{82,  10149},
	{83,  9813},
	{84,  9491},
	{85,  9181},
	{86,  8882},
	{87,  8595},
	{88,  8319},
	{89,  8053},
	{90,  7797},
	{91,  7550},
	{92,  7312},
	{93,  7084},
	{94,  6863},
	{95,  6651},
	{96,  6446},
	{97,  6249},
	{98,  6058},
	{99,  5875},
	{100, 5698}
};

/* convert register to temperature  */
static __s16 mtkts_flashlight_thermistor_conver_temp(__s32 Res)
{
	int i = 0;
	int asize = 0;
	__s32 RES1 = 0, RES2 = 0;
	__s32 TAP_Value = -2000, TMP1 = 0, TMP2 = 0;

	asize = (sizeof(FLASHLIGHT_Temperature_Table) / sizeof(struct FLASHLIGHT_TEMPERATURE));

	if (Res >= FLASHLIGHT_Temperature_Table[0].TemperatureR) {
		TAP_Value = -400;	/* min */
	} else if (Res <= FLASHLIGHT_Temperature_Table[asize - 1].TemperatureR) {
		TAP_Value = 1250;	/* max */
	} else {
		RES1 = FLASHLIGHT_Temperature_Table[0].TemperatureR;
		TMP1 = FLASHLIGHT_Temperature_Table[0].FLASHLIGHT_Temp;
		/* mtkts_flashlight_dprintk("%d : RES1 = %d,TMP1 = %d\n",__LINE__,
		 * RES1,TMP1);
		 */

		for (i = 0; i < asize; i++) {
			if (Res >= FLASHLIGHT_Temperature_Table[i].TemperatureR) {
				RES2 = FLASHLIGHT_Temperature_Table[i].TemperatureR;
				TMP2 = FLASHLIGHT_Temperature_Table[i].FLASHLIGHT_Temp;
				/* mtkts_flashlight_dprintk("%d :i=%d, RES2 = %d,
				 * TMP2 = %d\n",__LINE__,i,RES2,TMP2);
				 */
				break;
			}
			RES1 = FLASHLIGHT_Temperature_Table[i].TemperatureR;
			TMP1 = FLASHLIGHT_Temperature_Table[i].FLASHLIGHT_Temp;
			/* mtkts_flashlight_dprintk("%d :i=%d, RES1 = %d,
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
static __s16 mtk_ts_flashlight_volt_to_temp(__u32 dwVolt)
{
	__s32 TRes;
	__u64 dwVCriAP = 0;
	__u64 dwVCriAP2 = 0;
	__s32 flashlight_TMP = -100;

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

	g_flashlight_temperature_r = TRes;

	/* convert register to temperature */
	flashlight_TMP = mtkts_flashlight_thermistor_conver_temp(TRes);

	return flashlight_TMP;
}

static int get_hw_flashlight_temp(void)
{
	int val = 0;
	int ret = 0, output;

	ret = iio_read_channel_processed(thermistor_ch3, &val);
	mtkts_flashlight_dprintk("%s val=%d\n", __func__, val);

	if (ret < 0) {
		mtkts_flashlight_dprintk("IIO channel read failed %d\n", ret);
		return ret;
	}
	/* ret = ret*1800/4096; 82's ADC power */
	mtkts_flashlight_dprintk("APtery output mV = %d\n", val);
	output = mtk_ts_flashlight_volt_to_temp(val);
	mtkts_flashlight_dprintk("flashlight output temperature = %d\n", output);
	return output;
}

static DEFINE_MUTEX(flashlight_lock);
/*int ts_flashlight_at_boot_time = 0;*/
int mtkts_flashlight_get_hw_temp(void)
{
	int t_ret = 0;

	mutex_lock(&flashlight_lock);

	/* get HW AP temp (TSAP) */
	/* cat /sys/class/power_supply/AP/AP_temp */
	t_ret = get_hw_flashlight_temp();
	t_ret = t_ret * 100;

	mutex_unlock(&flashlight_lock);

	if (t_ret > 40000)	/* abnormal high temp */
		mtkts_flashlight_dprintk("T_flashlight=%d\n", t_ret);

	mtkts_flashlight_dprintk("[%s] T_flashlight, %d\n", __func__,
									t_ret);
	return t_ret;
}

static int mtk_flashlight_get_temp(struct thermal_zone_device *thermal, int *t)
{
	*t = mtkts_flashlight_get_hw_temp();

	if ((int)*t > 52000)
		mtkts_flashlight_dprintk("T=%d\n", (int)*t);

	if ((int)*t >= polling_trip_temp1)
		thermal->polling_delay = interval * 1000;
	else if ((int)*t < polling_trip_temp2)
		thermal->polling_delay = interval * polling_factor2;
	else
		thermal->polling_delay = interval * polling_factor1;

	return 0;
}

static int mtk_flashlight_bind(struct thermal_zone_device *thermal,
				 struct thermal_cooling_device *cdev)
{
	/*int table_val = 0;

	if (!strcmp(cdev->type, g_bind0)) {
		table_val = 0;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind1)) {
		table_val = 1;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind2)) {
		table_val = 2;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind3)) {
		table_val = 3;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind4)) {
		table_val = 4;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind5)) {
		table_val = 5;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind6)) {
		table_val = 6;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind7)) {
		table_val = 7;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind8)) {
		table_val = 8;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind9)) {
		table_val = 9;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else {
		return 0;
	}

	if (mtk_thermal_zone_bind_cooling_device(thermal, table_val, cdev)) {
		mtkts_flashlight_dprintk(
			"[%s] error binding cooling dev\n", __func__);
		return -EINVAL;
	}

	mtkts_flashlight_dprintk("[%s] binding OK, %d\n", __func__, table_val);*/
	return 0;
}

static int mtk_flashlight_unbind(struct thermal_zone_device *thermal,
				   struct thermal_cooling_device *cdev)
{
	/*int table_val = 0;

	if (!strcmp(cdev->type, g_bind0)) {
		table_val = 0;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind1)) {
		table_val = 1;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind2)) {
		table_val = 2;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind3)) {
		table_val = 3;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind4)) {
		table_val = 4;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind5)) {
		table_val = 5;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind6)) {
		table_val = 6;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind7)) {
		table_val = 7;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind8)) {
		table_val = 8;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind9)) {
		table_val = 9;
		mtkts_flashlight_dprintk("[%s] %s\n", __func__, cdev->type);
	} else
		return 0;

	if (thermal_zone_unbind_cooling_device(thermal, table_val, cdev)) {
		mtkts_flashlight_dprintk(
				"[%s] error unbinding cooling dev\n", __func__);

		return -EINVAL;
	}

	mtkts_flashlight_dprintk("[%s] unbinding OK\n", __func__);*/
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

static int mtkts_flashlight_param_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", g_rap_pull_up_r);
	seq_printf(m, "%d\n", g_rap_pull_up_voltage);
	seq_printf(m, "%d\n", g_tap_over_critical_low);
	seq_printf(m, "%d\n", g_adc_channel3);

	return 0;
}


static ssize_t mtkts_flashlight_param_write(
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

	mtkts_flashlight_dprintk("[%s]\n", __func__);

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
			mtkts_flashlight_dprintk("g_rap_pull_up_r=%d\n",
							g_rap_pull_up_r);
		} else {
			kfree(ptr_param_data);
			mtkts_flashlight_dprintk(
					"[mtkts_flashlight_write] bad PUP_R argument\n");
			return -EINVAL;
		}

		if (!strcmp(ptr_param_data->pull_V, "PUP_VOLT")) {
			g_rap_pull_up_voltage = ptr_param_data->valV;
			mtkts_flashlight_dprintk("g_Rat_pull_up_voltage=%d\n",
							g_rap_pull_up_voltage);
		} else {
			kfree(ptr_param_data);
			mtkts_flashlight_dprintk(
					"[mtkts_flashlight_write] bad PUP_VOLT argument\n");
			return -EINVAL;
		}

		if (!strcmp(ptr_param_data->overcrilow, "OVER_CRITICAL_L")) {
			g_tap_over_critical_low = ptr_param_data->over_cri_low;
			mtkts_flashlight_dprintk("g_tap_over_critical_low=%d\n",
					      g_tap_over_critical_low);
		} else {
			kfree(ptr_param_data);
			mtkts_flashlight_dprintk(
					"[mtkts_flashlight_write] bad OVERCRIT_L argument\n");
			return -EINVAL;
		}

		if (!strcmp(ptr_param_data->NTC_TABLE, "NTC_TABLE")) {
			mtkts_flashlight_dprintk("ntc_table=%d\n",
							ptr_param_data->ntc_table);
		} else {
			kfree(ptr_param_data);
			mtkts_flashlight_dprintk(
					"[mtkts_flashlight_write] bad NTC_TABLE argument\n");
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
		mtkts_flashlight_dprintk("adc_channel=%d\n", adc_channel);
		mtkts_flashlight_dprintk("g_rap_adc_channel=%d\n",
						g_rap_adc_channel);

		kfree(ptr_param_data);
		return count;
	}

	mtkts_flashlight_dprintk("[mtkts_flashlight_write] bad argument\n");
	kfree(ptr_param_data);
	return -EINVAL;
}

static int mtkts_flashlight_param_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtkts_flashlight_param_read, NULL);
}

static int mtk_flashlight_read(struct seq_file *m, void *v)
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

/* static ssize_t mtk_flashlight_write(
 * struct file *file, const char *buffer, int count, void *data)
 */
static ssize_t mtk_flashlight_write(
struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	pr_debug("Power/flashlight_Thermal: write, write, write!!!");
	pr_debug("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
	pr_debug("*****************************************");
	pr_debug("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
	return -EINVAL;
}

static int mtk_flashlight_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_flashlight_read, NULL);
}

static const struct file_operations mtkts_flashlight_fops = {
	.owner = THIS_MODULE,
	.open = mtk_flashlight_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = mtk_flashlight_write,
	.release = single_release,
};

static const struct file_operations mtkts_flashlight_param_fops = {
	.owner = THIS_MODULE,
	.open = mtkts_flashlight_param_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = mtkts_flashlight_param_write,
	.release = single_release,
};

static int mtk_flashlight_register_thermal(void)
{
	mtkts_flashlight_dprintk("[%s]\n", __func__);

	/* trips : trip 0~1 */
	thz_dev = mtk_thermal_zone_device_register("mtkflashlight", num_trip,
						NULL, &mtk_flashlight_dev_ops,
						0, 0, 0, interval * 1000);

	return 0;
}

static void mtk_flashlight_unregister_thermal(void)
{
	mtkts_flashlight_dprintk("[%s]\n", __func__);

	if (thz_dev) {
		mtk_thermal_zone_device_unregister(thz_dev);
		thz_dev = NULL;
	}
}

static int mtkts_flashlight_probe(struct platform_device *pdev)
{
	int err = 0;
	int ret = 0;
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *mtkts_flashlight_dir = NULL;

	mtkts_flashlight_dprintk("[%s]\n", __func__);

	if (!pdev->dev.of_node) {
		mtkts_flashlight_dprintk("[%s]Only DT based supported\n",
		__func__);
		return -ENODEV;
	}

	thermistor_ch3 = devm_kzalloc(&pdev->dev, sizeof(*thermistor_ch3),
		GFP_KERNEL);
	if (!thermistor_ch3)
		return -ENOMEM;

	thermistor_ch3 = iio_channel_get(&pdev->dev, "thermistor-ch3");
	ret = IS_ERR(thermistor_ch3);
	if (ret) {
		mtkts_flashlight_dprintk("[%s] fail to get auxadc iio ch0: %d\n",
			__func__, ret);
		return ret;
	}

	g_adc_channel3 = thermistor_ch3->channel->channel;
	mtkts_flashlight_dprintk("[%s]get auxadc iio ch: %d\n", __func__,
		thermistor_ch3->channel->channel);

	mtkts_flashlight_dir = mtk_thermal_get_proc_drv_therm_dir_entry();
	if (!mtkts_flashlight_dir) {
		mtkts_flashlight_dprintk(
			"[%s]: mkdir /proc/driver/thermal failed\n", __func__);
	} else {
		entry = proc_create("tzflashlight", 0664, mtkts_flashlight_dir,
				&mtkts_flashlight_fops);
		if (entry)
			proc_set_user(entry, uid, gid);

		entry = proc_create("tzflashlight_param", 0664, mtkts_flashlight_dir,
				&mtkts_flashlight_param_fops);
		if (entry)
			proc_set_user(entry, uid, gid);
	}

	return err;
}

#ifdef CONFIG_OF
const struct of_device_id mt_thermistor_of_match4[2] = {
	{.compatible = "mediatek,mtboard-thermistor4", },
	{},
};
#endif

#define THERMAL_THERMISTOR_NAME    "mtboard-thermistor4"
static struct platform_driver mtk_thermal_flashlight_driver = {
	.remove = NULL,
	.shutdown = NULL,
	.probe = mtkts_flashlight_probe,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		.name = THERMAL_THERMISTOR_NAME,
#ifdef CONFIG_OF
		.of_match_table = mt_thermistor_of_match4,
#endif
	},
};

static int __init mtk_flashlight_init(void)
{
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *mtk_flashlight_dir = NULL;
	int err = 0;

	mtkts_flashlight_dprintk("[%s]\n", __func__);

	err = platform_driver_register(&mtk_thermal_flashlight_driver);
	if (err) {
		mtkts_flashlight_dprintk("thermal driver callback register failed.\n");
		return err;
	}
	/*err = mtk_flashlight_register_cooler();
	if (err)
		return err;*/

	mtk_flashlight_dir = mtk_thermal_get_proc_drv_therm_dir_entry();
	if (!mtk_flashlight_dir) {
		mtkts_flashlight_dprintk("%s mkdir /proc/driver/thermal failed\n",
								__func__);
	} else {
		entry = proc_create("tzflashlight", 0664, mtk_flashlight_dir,
							&mtkts_flashlight_fops);
		if (entry)
			proc_set_user(entry, uid, gid);
	}
	mtk_flashlight_register_thermal();
	return 0;
}

static void __exit mtk_flashlight_exit(void)
{
	mtkts_flashlight_dprintk("[%s]\n", __func__);
	mtk_flashlight_unregister_thermal();
}
module_init(mtk_flashlight_init);
module_exit(mtk_flashlight_exit);
