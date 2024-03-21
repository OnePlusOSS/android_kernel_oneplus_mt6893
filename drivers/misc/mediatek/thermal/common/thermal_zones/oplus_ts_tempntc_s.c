// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
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
#include <linux/seq_file.h>
#include "mt-plat/mtk_thermal_monitor.h"
#include "mach/mtk_thermal.h"
#include <linux/uidgid.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/iio/consumer.h>
#include <linux/kthread.h>	/* For Kthread_run */
#include <linux/of_gpio.h>
#include "oplus_tempntc.h"


struct temp_data {
	struct platform_device *pdev;
	int btb_con_volt;
	int usb_con_volt;
	int pa1_ntc_volt;
	int pa2_ntc_volt;
	struct task_struct      *oplus_tempntc_kthread;
	struct iio_channel      *iio_channel_btb_con;
	struct iio_channel      *iio_channel_usb_con;
	struct power_supply *ac_psy;
	int ac_status;
	int tempntc_pin;
	struct pinctrl *pinctrl;
	struct pinctrl_state *ntcctrl_high;
	struct pinctrl_state *ntcctrl_low;
	struct delayed_work init_work;
	int delay_time;
	int len_array;
	int *con_volt_pa;
	int *con_volt_btb;
	int *con_temp;
	bool is_kthread_get_adc;
};
static struct temp_data *pinfo;

int con_volt_21061[] = {
	1759,
	1756,
	1753,
	1749,
	1746,
	1742,
	1738,
	1734,
	1730,
	1725,
	1720,
	1715,
	1709,
	1704,
	1697,
	1691,
	1684,
	1677,
	1670,
	1662,
	1654,
	1646,
	1637,
	1628,
	1618,
	1608,
	1598,
	1587,
	1575,
	1564,
	1551,
	1539,
	1526,
	1512,
	1498,
	1484,
	1469,
	1454,
	1438,
	1422,
	1405,
	1388,
	1371,
	1353,
	1335,
	1316,
	1297,
	1278,
	1258,
	1238,
	1218,
	1198,
	1177,
	1156,
	1136,
	1114,
	1093,
	1072,
	1050,
	1029,
	1007,
	986,
	964,
	943,
	921,
	900,
	879,
	858,
	837,
	816,
	796,
	775,
	755,
	736,
	716,
	697,
	678,
	659,
	641,
	623,
	605,
	588,
	571,
	555,
	538,
	522,
	507,
	492,
	477,
	463,
	449,
	435,
	422,
	409,
	396,
	384,
	372,
	360,
	349,
	338,
	327,
	317,
	307,
	297,
	288,
	279,
	270,
	261,
	253,
	245,
	237,
	230,
	222,
	215,
	209,
	202,
	196,
	189,
	183,
	178,
	172,
	167,
	162,
	156,
	152,
	147,
	142,
	138,
	134,
	130,
	126,
	122,
	118,
	114,
	111,
	108,
	104,
	101,
	98,
	95,
	92,
	90,
	87,
	84,
	82,
	80,
	77,
	75,
	73,
	71,
	69,
	67,
	65,
	63,
	61,
	59,
	58,
	56,
	55,
	53,
	52,
	50,
	49,
	47,
	46,
	45,
};

int con_volt_21015[] = {
	1719,
	1714,
	1708,
	1701,
	1695,
	1688,
	1680,
	1673,
	1664,
	1656,
	1647,
	1637,
	1627,
	1617,
	1606,
	1595,
	1583,
	1571,
	1558,
	1544,
	1530,
	1516,
	1501,
	1486,
	1470,
	1453,
	1436,
	1419,
	1401,
	1382,
	1363,
	1344,
	1324,
	1304,
	1283,
	1262,
	1241,
	1219,
	1197,
	1175,
	1152,
	1129,
	1106,
	1083,
	1060,
	1037,
	1014,
	990,
	967,
	944,
	921,
	898,
	875,
	852,
	829,
	807,
	785,
	763,
	741,
	720,
	699,
	679,
	659,
	639,
	619,
	600,
	581,
	563,
	545,
	528,
	511,
	494,
	478,
	462,
	447,
	432,
	418,
	404,
	390,
	377,
	364,
	351,
	339,
	328,
	317,
	306,
	295,
	285,
	275,
	265,
	256,
	247,
	239,
	230,
	222,
	215,
	207,
	200,
	193,
	186,
	180,
	174,
	168,
	162,
	156,
	151,
	146,
	141,
	136,
	131,
	127,
	123,
	119,
	115,
	111,
	107,
	103,
	100,
	97,
	94,
	90,
	87,
	85,
	82,
	79,
	77,
	74,
	72,
	69,
	67,
	65,
	63,
	61,
	59,
	57,
	55,
	54,
	52,
	50,
	49,
	47,
	46,
	45,
	43,
	42,
	41,
	39,
	38,
	37,
	36,
	35,
	34,
	33,
	32,
	31,
	30,
	29,
	29,
	28,
	27,
	26,
	25,
	25,
	24,
	23,
	23,
};


int con_temp_21061[] = {
	-40,
	-39,
	-38,
	-37,
	-36,
	-35,
	-34,
	-33,
	-32,
	-31,
	-30,
	-29,
	-28,
	-27,
	-26,
	-25,
	-24,
	-23,
	-22,
	-21,
	-20,
	-19,
	-18,
	-17,
	-16,
	-15,
	-14,
	-13,
	-12,
	-11,
	-10,
	-9,
	-8,
	-7,
	-6,
	-5,
	-4,
	-3,
	-2,
	-1,
	0,
	1,
	2,
	3,
	4,
	5,
	6,
	7,
	8,
	9,
	10,
	11,
	12,
	13,
	14,
	15,
	16,
	17,
	18,
	19,
	20,
	21,
	22,
	23,
	24,
	25,
	26,
	27,
	28,
	29,
	30,
	31,
	32,
	33,
	34,
	35,
	36,
	37,
	38,
	39,
	40,
	41,
	42,
	43,
	44,
	45,
	46,
	47,
	48,
	49,
	50,
	51,
	52,
	53,
	54,
	55,
	56,
	57,
	58,
	59,
	60,
	61,
	62,
	63,
	64,
	65,
	66,
	67,
	68,
	69,
	70,
	71,
	72,
	73,
	74,
	75,
	76,
	77,
	78,
	79,
	80,
	81,
	82,
	83,
	84,
	85,
	86,
	87,
	88,
	89,
	90,
	91,
	92,
	93,
	94,
	95,
	96,
	97,
	98,
	99,
	100,
	101,
	102,
	103,
	104,
	105,
	106,
	107,
	108,
	109,
	110,
	111,
	112,
	113,
	114,
	115,
	116,
	117,
	118,
	119,
	120,
	121,
	122,
	123,
	124,
	125,
};

int convert_volt_to_precise_temp(int volt)
{
	int volt1, volt2, temp1, temp2;
	int temp, i;

	if (volt <= 0) {
		volt = 2457;
	}

	if (volt >= pinfo->con_volt_pa[0]) {
		temp = pinfo->con_temp[0] * 1000;
	} else if (volt <=  pinfo->con_volt_pa[pinfo->len_array- 1]) {
		temp = pinfo->con_temp[pinfo->len_array- 1] * 1000;
	} else {
		volt1 = pinfo->con_volt_pa[0];
		temp1 =  pinfo->con_temp[0];

		for (i = 0; i < pinfo->len_array; i++) {
			if (volt >= pinfo->con_volt_pa[i]) {
				volt2 = pinfo->con_volt_pa[i];
				temp2 = pinfo->con_temp[i];
				break;
			}
			volt1 = pinfo->con_volt_pa[i];
			temp1 =  pinfo->con_temp[i];
		}
		/* now volt1 > volt > volt2 */
		temp = mult_frac((((volt - volt2) * temp1) + ((volt1 - volt) * temp2)), 1000, (volt1 - volt2));
	}
	return temp;
}

int convert_volt_to_temp(int volt)
{
	int i;
	int temp_volt = 0;
	int result_temp = 0;

	temp_volt = volt;
	if (temp_volt <= 0) {
		temp_volt = 2457;
	}

	for (i = pinfo->len_array- 1; i >= 0; i--) {
		if (pinfo->con_volt_btb[i] >= temp_volt)
			break;
		else if (i == 0)
			break;
	}

	result_temp = pinfo->con_temp[i];
	return result_temp;
}

int oplus_chg_get_battery_btb_temp_cal(void)
{
	int ret = 0;

	if (pinfo == NULL) {
		return 25;
	}

	ret = convert_volt_to_temp(pinfo->btb_con_volt);
	pr_err("%s ret = %d, btb_con_volt = %d\n", __func__, ret, pinfo->btb_con_volt);
	return ret;
}
EXPORT_SYMBOL(oplus_chg_get_battery_btb_temp_cal);


int oplus_chg_get_usb_btb_temp_cal(void)
{
	int ret = 0;

	if (pinfo == NULL) {
		return 25;
	}

	ret = convert_volt_to_temp(pinfo->usb_con_volt);
	pr_err("%s ret = %d,usb_con_volt = %d\n", __func__, ret, pinfo->usb_con_volt);
	return ret;
}
EXPORT_SYMBOL(oplus_chg_get_usb_btb_temp_cal);

void  oplus_chg_adc_switch_ctrl(void)
{
	if(!pinfo->ac_status) {
		printk("%s ac_status =%d\n", __func__, pinfo->ac_status);
		pinfo->delay_time = 800;
		schedule_delayed_work(&pinfo->init_work, 0);
	}
	return;
}
EXPORT_SYMBOL(oplus_chg_adc_switch_ctrl);



int oplus_get_pa1_con_temp(void)
{
	int ret = 0;

	if (pinfo == NULL) {
		return 25;
	 }
	ret = convert_volt_to_precise_temp(pinfo->pa1_ntc_volt);
	pr_err("%s ret = %d, pa1_ntc_volt = %d\n", __func__, ret, pinfo->pa1_ntc_volt);

	return ret;
}
EXPORT_SYMBOL(oplus_get_pa1_con_temp);


int oplus_get_pa2_con_temp(void)
{
	int ret = 0;

	if (pinfo == NULL) {
		return 25;
	}

	ret = convert_volt_to_precise_temp(pinfo->pa2_ntc_volt);
	pr_err("%s ret = %d, pa2_ntc_volt = %d\n", __func__, ret, pinfo->pa2_ntc_volt);

	return ret;
}
EXPORT_SYMBOL(oplus_get_pa2_con_temp);



bool is_kthread_get_adc(void)
{
	if (!pinfo)
		return 0;
	return pinfo->is_kthread_get_adc;
}

int oplus_tempntc_get_volt(struct temp_data *info)
{
	int ret;
	int ntcctrl_gpio_value = 0;
	static int last_ntcctrl_gpio = -1;

	int iio_chan2_volt;
	int iio_chan1_volt;

	if ((!info->iio_channel_btb_con) || (!info->iio_channel_usb_con) || (!info)) {
		pr_err("%s conntinue\n", __func__);
		return 0;
	}

	ntcctrl_gpio_value = gpio_get_value(info->tempntc_pin);

	if (last_ntcctrl_gpio != ntcctrl_gpio_value) {
		last_ntcctrl_gpio = ntcctrl_gpio_value;
		printk("%s ntcctrl_gpio_value =%d  last_ntcctrl_gpio =%d\n", __func__, ntcctrl_gpio_value, last_ntcctrl_gpio);
	}

	if (ntcctrl_gpio_value == 0) {
		ret = iio_read_channel_processed(info->iio_channel_usb_con, &iio_chan1_volt);
		if (ret < 0) {
			pr_err("%s pa1_ntc_volt read error!\n");
		} else {
			info->pa1_ntc_volt = iio_chan1_volt;
		}

		ret = iio_read_channel_processed(info->iio_channel_btb_con, &iio_chan2_volt);
		if (ret < 0) {
			pr_err("%s pa2_ntc_volt read error!\n");
		} else {
			info->pa2_ntc_volt = iio_chan2_volt;
		}

		pinctrl_select_state(info->pinctrl, info->ntcctrl_high);
		msleep(100);
		ret = gpio_get_value(info->tempntc_pin);

		ret = iio_read_channel_processed(info->iio_channel_usb_con, &iio_chan1_volt);
		if (ret < 0) {
			pr_err("%s usb_con_volt read error!\n");
		} else {
			info->usb_con_volt = iio_chan1_volt;
		}

		ret = iio_read_channel_processed(info->iio_channel_btb_con, &iio_chan2_volt);
		if (ret < 0) {
			pr_err("%s btb_con_volt read error!\n");
		} else {
			info->btb_con_volt = iio_chan2_volt;
		}

	} else {
		ret = iio_read_channel_processed(info->iio_channel_usb_con, &iio_chan1_volt);
		if (ret < 0) {
			pr_err("%s usb_con_volt read error!\n");
		} else {
			info->usb_con_volt = iio_chan1_volt;
		}

		ret = iio_read_channel_processed(info->iio_channel_btb_con, &iio_chan2_volt);
		if (ret < 0) {
			pr_err("%s btb_con_volt read error!\n");
		} else {
			info->btb_con_volt = iio_chan2_volt;
		}

		pinctrl_select_state(info->pinctrl, info->ntcctrl_low);
		msleep(100);
		ret = gpio_get_value(info->tempntc_pin);

		ret = iio_read_channel_processed(info->iio_channel_usb_con, &iio_chan1_volt);
		if (ret < 0) {
			pr_err("%s pa1_ntc_volt read error!\n");
		} else {
			info->pa1_ntc_volt = iio_chan1_volt;
		}

		ret = iio_read_channel_processed(info->iio_channel_btb_con, &iio_chan2_volt);
		if (ret < 0) {
			pr_err("%s pa2_ntc_volt read error!\n");
		} else {
			info->pa2_ntc_volt = iio_chan2_volt;
		}
	}

	info->is_kthread_get_adc = true;
	pr_err("%s is_kthread_get_adc = %d,ntcctrl_gpio_value = %d, pa1_ntc_volt = %d,pa2_ntc_volt = %d,usb_con_volt = %d,btb_con_volt = %d\n",
		__func__, info->is_kthread_get_adc, ntcctrl_gpio_value, info->pa1_ntc_volt, info->pa2_ntc_volt, info->usb_con_volt, info->btb_con_volt);

	return 0;
}

static void oplus_tempntc_det_work(struct work_struct *work)
{
	union power_supply_propval propval;
	int ret = 0;

	if (!pinfo->ac_psy)
		pinfo->ac_psy = power_supply_get_by_name("ac");

	pinfo->delay_time = 5000;

	if(pinfo->ac_psy) {
		ret = power_supply_get_property(pinfo->ac_psy,
						POWER_SUPPLY_PROP_ONLINE,
						&propval);
		pinfo->ac_status = propval.intval;
		if(pinfo->ac_status)
			pinfo->delay_time = 800;
	}

	oplus_tempntc_get_volt(pinfo);

	schedule_delayed_work(&pinfo->init_work, msecs_to_jiffies(pinfo->delay_time));
}


static int oplus_ntcctrl_gpio_init(struct temp_data *info, struct device *dev)
{
	if (!info) {
		printk("info  null !\n");
		return -EINVAL;
	}

	info->pinctrl = devm_pinctrl_get(dev);

	if (IS_ERR_OR_NULL(info->pinctrl)) {
		printk("get temp ntc princtrl fail\n");
		return -EINVAL;
	}

	info->ntcctrl_high = pinctrl_lookup_state(info->pinctrl, "ntcctrl_high");
	if (IS_ERR_OR_NULL(info->ntcctrl_high)) {
		printk("get ntcctrl_high fail\n");
		return -EINVAL;
	}

	info->ntcctrl_low = pinctrl_lookup_state(info->pinctrl, "ntcctrl_low");
	if (IS_ERR_OR_NULL(info->ntcctrl_low)) {
		printk("get ntcctrl_low fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(info->pinctrl, info->ntcctrl_low);
	return 0;
}

static int oplus_tempntc_parse_dt(struct temp_data *info,
				struct device *dev)
{
	struct device_node *np = dev->of_node;

	info->iio_channel_btb_con = iio_channel_get(dev, "auxadc1-usb_btb_temp");
	if (IS_ERR(info->iio_channel_btb_con)) {
		pr_err("battery ID CHANNEL ERR \n");
	}

	info->iio_channel_usb_con = iio_channel_get(dev, "auxadc2-battery_btb_temp");
	if (IS_ERR(info->iio_channel_usb_con)) {
		pr_err("Flash_PA CHANNEL ERR \n");
	}

	info->tempntc_pin = of_get_named_gpio(np, "qcom,ntcctrl-gpio", 0);
	if(info->tempntc_pin < 0) {
		pr_err("[%s]: tempntc_pin < 0 !!! \r\n", __func__);
	}

	if (gpio_request(info->tempntc_pin, "TEMPNTC_GPIO") < 0)
		pr_err("[%s]: tempntc_pin gpio_request fail !!! \r\n", __func__);

	return 0;
}

static int oplus_tempntc_data_init(struct temp_data *info)
{
	info->btb_con_volt = -1;
	info->usb_con_volt = -1;
	info->pa1_ntc_volt = -1;
	info->pa2_ntc_volt = -1;
	info->ac_status = 0;
	info->len_array = ARRAY_SIZE(con_temp_21061);
	info->con_volt_pa = con_volt_21061;
	if (oplus_voocphy_get_bidirect_cp_support())
		info->con_volt_btb = con_volt_21015;
	else
		info->con_volt_btb = con_volt_21061;
	info->con_temp = con_temp_21061;
	if (!info->ac_psy) {
		info->ac_psy = power_supply_get_by_name("ac");
	}

	return 0;
}

static int oplus_tempntc_pdrv_probe(struct platform_device *pdev)
{
	struct temp_data *info;

	pr_err("%s: starts\n", __func__);
	info = devm_kzalloc(&pdev->dev, sizeof(struct temp_data), GFP_KERNEL);
	if (!info) {
		pr_err(" kzalloc() failed\n");
		return -ENOMEM;
	}

	pinfo = info;
	platform_set_drvdata(pdev, info);
	info->pdev = pdev;

	oplus_tempntc_data_init(info);
	oplus_tempntc_parse_dt(info, &pdev->dev);
	oplus_ntcctrl_gpio_init(info, &pdev->dev);

	INIT_DELAYED_WORK(&info->init_work, oplus_tempntc_det_work);
	schedule_delayed_work(&info->init_work, 0);
	return 0;
}

static int oplus_tempntc_pdrv_remove(struct platform_device *pdev)
{
	power_supply_put(pinfo->ac_psy);
	return 0;
}

static void oplus_tempntc_pdrv_shutdown(struct platform_device *dev)
{
}

static const struct of_device_id oplus_tempntc_of_match[] = {
	{.compatible = "oplus,ntc_ctrl", },
	{},
};
MODULE_DEVICE_TABLE(of, oplus_tempntc_of_match)

static int __maybe_unused tempntc_suspend(struct device *dev)
{
	cancel_delayed_work_sync(&pinfo->init_work);
	return 0;
}

static int __maybe_unused tempntc_resume(struct device *dev)
{
	/* Schedule timer to check current status */
	schedule_delayed_work(&pinfo->init_work,
			msecs_to_jiffies(pinfo->delay_time));
	return 0;
}

static SIMPLE_DEV_PM_OPS(tempntc_pm_ops, tempntc_suspend, tempntc_resume);

static struct platform_driver oplus_tempntc_driver = {
	.probe = oplus_tempntc_pdrv_probe,
	.remove = oplus_tempntc_pdrv_remove,
	.shutdown = oplus_tempntc_pdrv_shutdown,
	.driver = {
		.name = "oplus_tempntc",
		.owner = THIS_MODULE,
		.of_match_table = oplus_tempntc_of_match,
		.pm	= &tempntc_pm_ops,
	},
};

static int __init oplus_tempntc_init(void)
{
	int ret;
	ret = platform_driver_register(&oplus_tempntc_driver);
	if (ret) {
		pr_err("%s fail to tempntc device\n", __func__);
		return ret;
	}
	return ret;
}

late_initcall(oplus_tempntc_init);

static void __exit oplus_tempntc_exit(void)
{
	platform_driver_unregister(&oplus_tempntc_driver);
}
module_exit(oplus_tempntc_exit);

MODULE_AUTHOR("shuai.sun <sunshuai.@oplus.com>");
MODULE_DESCRIPTION("oplus Device Driver");
MODULE_LICENSE("GPL");
