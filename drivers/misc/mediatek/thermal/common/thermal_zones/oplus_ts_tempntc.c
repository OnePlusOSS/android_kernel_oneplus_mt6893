// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
#define pr_fmt(fmt)	"[OPLUS_TEMPNTC] %s: " fmt, __func__

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
#include <soc/oplus/system/oplus_project.h>
#include <linux/power_supply.h>
#include <linux/iio/consumer.h>
#include <linux/kthread.h>	/* For Kthread_run */
#include <linux/of_gpio.h>

#define CHGNTC_ENABLE_LEVEL		     	(1)
#define CHGNTC_DISABLE_LEVEL			(0)
#define  BOARD_TEMP_GPIO_NUM   154
#define OPLUS_DALEY_MS 300
enum {
	BATT_ID_NTC = 0,
	FLASH_NTC,
	CHARGER_NTC,
	PA_NTC,
	BB_NTC,
	BAT_CON_NTC,
	NTC_MAX_INDEX
};

enum {
	NTC_NO_SWITCH = 0,
	NTC_SWITCH_1,
	NTC_SWITCH_2,
	NTC_SWITCH_MAX
};

enum {
	SWITCH_LOW = 0,
	SWITCH_HIGH,
	SWITCH_LEVEL_MAX
};

enum {
	CHANNEL_0 = 0,
	CHANNEL_1,
	CHANNEL_2,
	CHANNEL_3,
	CHANNEL_4
};

struct temp_data {
	struct platform_device *pdev;
	int batt_id_volt;
	int flash_ntc_volt;
	int charger_ntc_volt;
	int pa_ntc_volt;
	int bb_ntc_volt;
	int bat_con_ntc_volt;
	struct task_struct      *oplus_tempntc_kthread;
	struct iio_channel      *iio_channel_btb;
	struct iio_channel      *iio_channel_ftp;
	struct iio_channel      *iio_channel_btc;
	struct iio_channel      *iio_channel_bb;
	struct iio_channel      *iio_channel_pa;
	struct iio_channel      *iio_channel_flash;
	struct iio_channel      *iio_channel_batid;
	struct iio_channel      *iio_channel_charger;
	struct iio_channel      *iio_channel_0;
	struct iio_channel      *iio_channel_1;
	struct iio_channel      *iio_channel_2;
	struct iio_channel      *iio_channel_3;
	struct iio_channel      *iio_channel_4;
	struct power_supply *ac_psy;
	struct pinctrl *pinctrl;
	struct pinctrl_state *ntc_switch1_ctrl_high;
	struct pinctrl_state *ntc_switch1_ctrl_low;
	struct pinctrl_state *ntc_switch2_ctrl_high;
	struct pinctrl_state *ntc_switch2_ctrl_low;
	struct delayed_work init_work;
	struct completion temp_det_start;
	struct mutex det_lock;
	bool is_kthread_get_adc;
	int ntcswitch1_pin;
	int ntcswitch2_pin;
	int ntcswitchgpio_number;
	bool disable_ntc_switch;
	bool ntc_general_method;
	unsigned int ntc_map[NTC_MAX_INDEX][3];
	int ntc_volt_general[NTC_MAX_INDEX];
};
static struct temp_data *pinfo;
extern void oplus_gpio_switch_lock(void);
extern void oplus_gpio_switch_unlock(void);
extern void oplus_gpio_value_switch(unsigned int pin, unsigned int val);

static bool ntc_present[NTC_MAX_INDEX];
int get_flash_ntc_volt(void)
{
    if (!pinfo)
        return -1;
    return pinfo->flash_ntc_volt;
}

int get_batt_id_ntc_volt(void)
{
    if (!pinfo)
        return -1;
    return pinfo->batt_id_volt;
}

int get_charger_ntc_volt(void)
{
	if (!pinfo)
		return -1;
	return pinfo->charger_ntc_volt;
}

int get_pa_ntc_volt(void)
{
	if (!pinfo)
		return -1;
	return pinfo->pa_ntc_volt;
}

int get_bb_ntc_volt(void)
{
	if (!pinfo)
		return -1;
	return pinfo->bb_ntc_volt;
}

int get_bat_con_ntc_volt(void)
{
	if (!pinfo)
		return -1;
	return pinfo->bat_con_ntc_volt;
}

bool is_kthread_get_adc(void)
{
    if (!pinfo)
        return 0;
    return pinfo->is_kthread_get_adc;
}

void oplus_tempntc_read_ntcswitch1_high(struct temp_data *info) {
	int ret;
        int iio_chan1_volt;
        int iio_chan3_volt;

	ret = iio_read_channel_processed(info->iio_channel_ftp, &iio_chan1_volt);
	if (ret < 0) {
		pr_err("PA_NTC read error!\n");
	} else {
		info->pa_ntc_volt = iio_chan1_volt;
	}

	ret = iio_read_channel_processed(info->iio_channel_btc, &iio_chan3_volt);
	if (ret < 0) {
		pr_err("CHARGE_NTC read error!\n");
	} else {
		info->charger_ntc_volt = iio_chan3_volt;
	}
}

void oplus_tempntc_read_ntcswitch1_low(struct temp_data *info) {
	int ret;
	int iio_chan1_volt;
	int iio_chan3_volt;

	ret = iio_read_channel_processed(info->iio_channel_ftp, &iio_chan1_volt);
	if (ret < 0) {
		pr_err("FLASH_NTC read error!\n");
	} else {
		info->flash_ntc_volt = iio_chan1_volt;
	}

	ret = iio_read_channel_processed(info->iio_channel_btc, &iio_chan3_volt);
	if (ret < 0) {
		pr_err("BAT_ID read error!\n");
	} else {
		info->batt_id_volt = iio_chan3_volt;
	}
}

void oplus_tempntc_onegpioswitch_read_ntcswitch1_high(struct temp_data *info) {
	int ret;
        int iio_chan1_volt;
        int iio_chan3_volt;

	ret = iio_read_channel_processed(info->iio_channel_ftp, &iio_chan1_volt);
	if (ret < 0) {
		pr_err("PA_NTC read error!\n");
	} else {
		info->bb_ntc_volt = iio_chan1_volt;
	}

	ret = iio_read_channel_processed(info->iio_channel_btc, &iio_chan3_volt);
	if (ret < 0) {
		pr_err("CHARGE_NTC read error!\n");
	} else {
		info->pa_ntc_volt = iio_chan3_volt;
	}
}

void oplus_tempntc_onegpioswitch_read_ntcswitch1_low(struct temp_data *info) {
	int ret;
	int iio_chan1_volt;
	int iio_chan3_volt;

	ret = iio_read_channel_processed(info->iio_channel_ftp, &iio_chan1_volt);
	if (ret < 0) {
		pr_err("FLASH_NTC read error!\n");
	} else {
		info->bat_con_ntc_volt = iio_chan1_volt;
	}

	ret = iio_read_channel_processed(info->iio_channel_btc, &iio_chan3_volt);
	if (ret < 0) {
		pr_err("BAT_ID read error!\n");
	} else {
		info->charger_ntc_volt = iio_chan3_volt;
	}
}


void oplus_tempntc_read_ntcswitch2_high(struct temp_data *info) {
	int ret;
        int iio_chan0_volt;

	ret = iio_read_channel_processed(info->iio_channel_btb, &iio_chan0_volt);
	if (ret < 0) {
		pr_err("BAT_CON_NTC read error!\n");
	} else {
		info->bat_con_ntc_volt = iio_chan0_volt;
	}

}

void oplus_tempntc_read_ntcswitch2_low(struct temp_data *info) {
	int ret;
	int iio_chan0_volt;

	ret = iio_read_channel_processed(info->iio_channel_btb, &iio_chan0_volt);
	if (ret < 0) {
		pr_err("BB_NTC read error!\n");
	} else {
		info->bb_ntc_volt = iio_chan0_volt;
	}
}

int oplus_tempntc_read_ntc_general(int ntc)
{
	struct iio_channel *iio_channel_temp = NULL;
	int iio_channel_volt = 0;
	int ret = 0;
	unsigned int channel_num = 0;

	if (!pinfo) {
		pr_err("pinfo NULL\n");
		return -EINVAL;
	}

	channel_num = pinfo->ntc_map[ntc][0];
	pr_info("Read ntc : %d channel num %d\n", ntc, channel_num);

	switch (channel_num) {
	case CHANNEL_0:
		iio_channel_temp = pinfo->iio_channel_0;
		break;
	case CHANNEL_1:
		iio_channel_temp = pinfo->iio_channel_1;
		break;
	case CHANNEL_2:
		iio_channel_temp = pinfo->iio_channel_2;
		break;
	case CHANNEL_3:
		iio_channel_temp = pinfo->iio_channel_3;
		break;
	case CHANNEL_4:
		iio_channel_temp = pinfo->iio_channel_4;
		break;
	default:
		iio_channel_temp = NULL;
		 break;
	}

	if (!iio_channel_temp) {
		pr_err("channel NULL\n");
		return -EINVAL;
	}

	ret = iio_read_channel_processed(iio_channel_temp, &iio_channel_volt);
	if (ret < 0) {
		pr_err("read error!\n");
		return -EINVAL;
	}
	pr_info("ntc = %d, iio_channel_volt = %d\n", ntc, iio_channel_volt);
	return iio_channel_volt;
}

void oplus_tempntc_get_volt_general(unsigned int switch_num, int switch_state)
{
	int ntcswitch_gpio_value = 0;
	struct pinctrl_state *ntc_switch_ctrl_temp = NULL;
	int i = 0;

	if (!pinfo) {
		pr_err("pinfo NULL,return\n");
		return;
	}

	if (switch_num == NTC_SWITCH_1 && switch_state == SWITCH_LOW) {
		ntc_switch_ctrl_temp = pinfo->ntc_switch1_ctrl_low;
		ntcswitch_gpio_value = gpio_get_value(pinfo->ntcswitch1_pin);
	} else if (switch_num == NTC_SWITCH_1 && switch_state == SWITCH_HIGH) {
		ntc_switch_ctrl_temp = pinfo->ntc_switch1_ctrl_high;
		ntcswitch_gpio_value = gpio_get_value(pinfo->ntcswitch1_pin);
	} else if (switch_num == NTC_SWITCH_2 && switch_state == SWITCH_LOW) {
		ntc_switch_ctrl_temp = pinfo->ntc_switch2_ctrl_low;
		ntcswitch_gpio_value = gpio_get_value(pinfo->ntcswitch2_pin);
	} else if (switch_num == NTC_SWITCH_2 && switch_state == SWITCH_HIGH) {
		ntc_switch_ctrl_temp = pinfo->ntc_switch2_ctrl_high;
		ntcswitch_gpio_value = gpio_get_value(pinfo->ntcswitch2_pin);
	} else {
		ntc_switch_ctrl_temp = NULL;
		ntcswitch_gpio_value = 0;
	}

	if ((ntc_switch_ctrl_temp != NULL) && (ntcswitch_gpio_value != switch_state)) {
		pinctrl_select_state(pinfo->pinctrl, ntc_switch_ctrl_temp);
		msleep(OPLUS_DALEY_MS);
	}

	for (i = 0; i < NTC_MAX_INDEX; ++i) {
		if (!ntc_present[i]) {
			pinfo->ntc_volt_general[i] = -1;
		} else if (pinfo->ntc_map[i][1] == switch_num &&
				pinfo->ntc_map[i][2] == switch_state) {
			pinfo->ntc_volt_general[i] = oplus_tempntc_read_ntc_general(i);
			pr_info("ntc = %d, channel : %d, switch : %d, state :%d, volt = %d",
					i, pinfo->ntc_map[i][0], pinfo->ntc_map[i][1], pinfo->ntc_map[i][2],
					pinfo->ntc_volt_general[i]);
		}
	}
}

int oplus_tempntc_get_volt(struct temp_data *info)
{
	int ret;
	int ntcswitch1_gpio_value = 0;
	int ntcswitch2_gpio_value = 0;
	int iio_chan_bb;
	int iio_chan_pa;
	int iio_chan_flash;
	int iio_chan_batid;
	int iio_chan_charger;

	if (!info) {
		pr_err("info NULL\n");
		return 0;
	}

	if (pinfo->ntc_general_method) {
		oplus_tempntc_get_volt_general(NTC_NO_SWITCH, SWITCH_LOW);  /* Read NTCs without switch ctrl*/
		if (info->ntcswitchgpio_number) {
			ntcswitch1_gpio_value = gpio_get_value(info->ntcswitch1_pin);
			if (ntcswitch1_gpio_value == SWITCH_LOW) {
				oplus_tempntc_get_volt_general(NTC_SWITCH_1, SWITCH_LOW);  /* Read NTCs controlled by sw1 low*/
				oplus_tempntc_get_volt_general(NTC_SWITCH_1, SWITCH_HIGH);  /* Read NTCs controlled by sw1 high*/
			} else {
				oplus_tempntc_get_volt_general(NTC_SWITCH_1, SWITCH_HIGH);  /* Read NTCs controlled by sw1 low*/
				oplus_tempntc_get_volt_general(NTC_SWITCH_1, SWITCH_LOW);  /* Read NTCs controlled by sw1 high*/
			}
		}
		if (info->ntcswitchgpio_number > 1) {
			ntcswitch2_gpio_value = gpio_get_value(info->ntcswitch2_pin);
			if (ntcswitch2_gpio_value == SWITCH_LOW) {
				oplus_tempntc_get_volt_general(NTC_SWITCH_2, SWITCH_LOW);  /* Read NTCs controlled by sw2 low*/
				oplus_tempntc_get_volt_general(NTC_SWITCH_2, SWITCH_HIGH);  /* Read NTCs controlled by sw2 high*/
			} else {
				oplus_tempntc_get_volt_general(NTC_SWITCH_2, SWITCH_HIGH);  /* Read NTCs controlled by sw2 low*/
				oplus_tempntc_get_volt_general(NTC_SWITCH_2, SWITCH_LOW);  /* Read NTCs controlled by sw2 high*/
			}
		}
		info->batt_id_volt	= info->ntc_volt_general[BATT_ID_NTC];
		info->flash_ntc_volt	= info->ntc_volt_general[FLASH_NTC];
		info->charger_ntc_volt	= info->ntc_volt_general[CHARGER_NTC];
		info->pa_ntc_volt	= info->ntc_volt_general[PA_NTC];
		info->bb_ntc_volt	= info->ntc_volt_general[BB_NTC];
		info->bat_con_ntc_volt	= info->ntc_volt_general[BAT_CON_NTC];

	} else if (pinfo->disable_ntc_switch) {
		pr_err("Get tempntc This project will use ntc switch  \n");
		if ((!info) || (!info->iio_channel_bb) || (!info->iio_channel_pa)|| (!info->iio_channel_batid)
			|| (!info->iio_channel_flash) ||(!info->iio_channel_charger)){
			pr_err("conntinue\n");
			return 0;
		}

		ret = iio_read_channel_processed(info->iio_channel_bb, &iio_chan_bb);
		if (ret < 0) {
			pr_err("BB_NTC read error!\n");
		}
		else {
			info->bb_ntc_volt = iio_chan_bb;
		}
		ret = iio_read_channel_processed(info->iio_channel_pa, &iio_chan_pa);
		if (ret < 0) {
			pr_err("PA_NTC read error!\n");
		}
		else {
			info->pa_ntc_volt = iio_chan_pa;
		}
		ret = iio_read_channel_processed(info->iio_channel_flash, &iio_chan_flash);
		if (ret < 0) {
			pr_err("FLASH_NTC read error!\n");
		} else {
			info->flash_ntc_volt = iio_chan_flash;
		}
		ret = iio_read_channel_processed(info->iio_channel_batid, &iio_chan_batid);
		if (ret < 0) {
			pr_err("BATTERY_ID_NTC read error!\n");
		}
		else {
			info->batt_id_volt = iio_chan_batid;
		}
		ret = iio_read_channel_processed(info->iio_channel_charger, &iio_chan_charger);
		if (ret < 0) {
			pr_err("CHARGER_NTC read error!\n");
			}
		else {
			info->charger_ntc_volt = iio_chan_charger;
		}
	} else if (info->ntcswitchgpio_number == 1) {
		pr_err("Get tempntc This project  use one ntc switch gpio \n");
		if ((!info) || (!info->iio_channel_ftp)
			|| (!info->iio_channel_btc)) {
			pr_err("conntinue\n");
			return 0;
		}

		ntcswitch1_gpio_value = gpio_get_value(info->ntcswitch1_pin);
		pr_err("ntcswitch1_gpio_value = %d .\n",
			ntcswitch1_gpio_value);

		if (ntcswitch1_gpio_value == 0) {
			oplus_tempntc_onegpioswitch_read_ntcswitch1_low(info);
			/* -------------------------------------------------- */
			pinctrl_select_state(info->pinctrl, info->ntc_switch1_ctrl_high);
			msleep(OPLUS_DALEY_MS);

			ret = gpio_get_value(info->ntcswitch1_pin);
			if (ret < 0) {
				pr_err("ntcswitch1_gpio_value = %d\n", ret);
			}
			oplus_tempntc_onegpioswitch_read_ntcswitch1_high(info);
		} else {
			oplus_tempntc_onegpioswitch_read_ntcswitch1_high(info);
			/* -------------------------------------------------- */
			pinctrl_select_state(info->pinctrl, info->ntc_switch1_ctrl_low);
			msleep(OPLUS_DALEY_MS);

			ret = gpio_get_value(info->ntcswitch1_pin);
			if (ret < 0) {
				pr_err("ntcswitch1_gpio_value = %d\n", ret);
			}
			oplus_tempntc_onegpioswitch_read_ntcswitch1_low(info);
		}
	} else {
		pr_err("Get tempntc This project will  use > 1 ntc switch gpio \n");
		if ((!info->iio_channel_btb) || (!info->iio_channel_ftp)
			|| (!info->iio_channel_btc) || (!info)) {
			pr_err("conntinue\n");
			return 0;
		}

		ntcswitch1_gpio_value = gpio_get_value(info->ntcswitch1_pin);
		ntcswitch2_gpio_value = gpio_get_value(info->ntcswitch2_pin);
		pr_err("ntcswitch1_gpio_value = %d , ntcswitch2_gpio_value = %d.\n",
			ntcswitch1_gpio_value, ntcswitch2_gpio_value);

		if (ntcswitch1_gpio_value == 0) {
			oplus_tempntc_read_ntcswitch1_low(info);
			/* -------------------------------------------------- */
			pinctrl_select_state(info->pinctrl, info->ntc_switch1_ctrl_high);
			msleep(OPLUS_DALEY_MS);

			ret = gpio_get_value(info->ntcswitch1_pin);
			if (ret < 0) {
				pr_err("ntcswitch1_gpio_value = %d\n", ret);
			}
			oplus_tempntc_read_ntcswitch1_high(info);
		} else {
			oplus_tempntc_read_ntcswitch1_high(info);
			/* -------------------------------------------------- */
			pinctrl_select_state(info->pinctrl, info->ntc_switch1_ctrl_low);
			msleep(OPLUS_DALEY_MS);

			ret = gpio_get_value(info->ntcswitch1_pin);
			if (ret < 0) {
				pr_err("ntcswitch1_gpio_value = %d\n", ret);
			}
			oplus_tempntc_read_ntcswitch1_low(info);
		}

		if (ntcswitch2_gpio_value == 0) {
			oplus_tempntc_read_ntcswitch2_low(info);
			/* -------------------------------------------------- */
			pinctrl_select_state(info->pinctrl, info->ntc_switch2_ctrl_high);
			msleep(OPLUS_DALEY_MS);

			ret = gpio_get_value(info->ntcswitch2_pin);
			if (ret < 0) {
				pr_err("ntcswitch2_gpio_value = %d\n", ret);
			}
			oplus_tempntc_read_ntcswitch2_high(info);
		} else {
			oplus_tempntc_read_ntcswitch2_high(info);
			/* -------------------------------------------------- */
			pinctrl_select_state(info->pinctrl, info->ntc_switch2_ctrl_low);
			msleep(OPLUS_DALEY_MS);

			ret = gpio_get_value(info->ntcswitch2_pin);
			if (ret < 0) {
				pr_err("ntcswitch2_gpio_value = %d\n", ret);
			}
			oplus_tempntc_read_ntcswitch2_low(info);
		}
	}

	pinfo->is_kthread_get_adc = true;
	pr_err("BAT_CON_NTC[%d], PA_NTC[%d], CHARGE_NTC[%d], BB_NTC[%d], FLASH_NTC[%d], BAT_ID[%d].\n",
		info->bat_con_ntc_volt, info->pa_ntc_volt, info->charger_ntc_volt,
		info->bb_ntc_volt, info->flash_ntc_volt, info->batt_id_volt);

	return 0;
}

static void oplus_tempntc_det_work(struct work_struct *work)
{
	mutex_lock(&pinfo->det_lock);
	oplus_tempntc_get_volt(pinfo);
	mutex_unlock(&pinfo->det_lock);
	schedule_delayed_work(&pinfo->init_work, msecs_to_jiffies(5000));
}

static int oplus_tempntc_parse_dt(struct temp_data *info,
				struct device *dev)
{
	struct device_node *np = dev->of_node;
	int rc;

	info->iio_channel_btb = iio_channel_get(dev, "auxadc0-bb_or_bat_con_v");
	if (IS_ERR(info->iio_channel_btb)) {
		pr_err("BAT_CON_NTC BB_NTC ERR \n");
	}

	info->iio_channel_ftp = iio_channel_get(dev, "auxadc1-flash_or_pa_v");
	if (IS_ERR(info->iio_channel_ftp)){
		pr_err("Flash PA CHANNEL ERR \n");
	}

	info->iio_channel_btc = iio_channel_get(dev, "auxadc3-bat_id_or_charge_v");
	if (IS_ERR(info->iio_channel_btc)){
		pr_err("BAT_ID CHARGE_NTC CHANNEL ERR \n");
	}

	info->iio_channel_bb = iio_channel_get(dev, "auxadc0-bb_v");
	if (IS_ERR(info->iio_channel_bb)){
		pr_err("BB NTC CHANNEL ERR \n");
	}

	info->iio_channel_pa = iio_channel_get(dev, "auxadc1-pa_v");
	if (IS_ERR(info->iio_channel_pa)){
		pr_err("PA NTC CHANNEL ERR \n");
	}

	info->iio_channel_flash = iio_channel_get(dev, "auxadc2-flash_v");
	if (IS_ERR(info->iio_channel_flash)){
		pr_err("FLASH NTC CHANNEL ERR \n");
	}

	info->iio_channel_batid = iio_channel_get(dev, "auxadc3-bat_id_v");
	if (IS_ERR(info->iio_channel_batid)){
		pr_err("BATTERY ID NTC CHANNEL ERR \n");
	}

	info->iio_channel_charger = iio_channel_get(dev, "auxadc4-charger_v");
	if (IS_ERR(info->iio_channel_charger)){
		pr_err("CHARGER NTC CHANNEL ERR \n");
	}

	info->disable_ntc_switch = of_property_read_bool(np, "disable_ntc_switch");

	if (of_property_read_u32(np, "ntc_switch_gpio_number", &info->ntcswitchgpio_number) < 0) {
		info->ntcswitchgpio_number = 2;/*default switch gpio number is 2,miami prj is 1*/
		pr_err("ntc_switch_gpio_number  !!! \r\n");
	}

	if (!info->disable_ntc_switch) {
		pr_info("This project will use ntc switch  \n");
		info->ntcswitch1_pin = of_get_named_gpio(np, "ntc_switch1_gpio", 0);

		if (info->ntcswitch1_pin < 0)
			pr_err("ntc_switch1_gpio < 0 !!! \r\n");

		if (gpio_request(info->ntcswitch1_pin, "NTC_SWITCH1_GPIO") < 0)
			pr_err("ntc_switch1_gpio gpio_request fail !!! \r\n");

		info->ntcswitch2_pin = of_get_named_gpio(np, "ntc_switch2_gpio", 0);
		if (info->ntcswitch2_pin < 0)
			pr_err("ntc_switch2_gpio < 0 !!! \r\n");

		if (gpio_request(info->ntcswitch2_pin, "NTC_SWITCH2_GPIO") < 0)
			pr_err("ntc_switch2_gpio gpio_request fail !!! \r\n");
	}

	info->ntc_general_method = of_property_read_bool(np, "oplus,ntc_general_method");
	if (info->ntc_general_method) {
		info->iio_channel_0 = iio_channel_get(dev, "auxadc0");
		if (IS_ERR(info->iio_channel_0))
			pr_err("CHANNEL 0 ERR \n");

		info->iio_channel_1 = iio_channel_get(dev, "auxadc1");
		if (IS_ERR(info->iio_channel_1))
			pr_err("CHANNEL 1 ERR \n");

		info->iio_channel_2 = iio_channel_get(dev, "auxadc2");
		if (IS_ERR(info->iio_channel_2))
			pr_err("CHANNEL 2 ERR \n");

		info->iio_channel_3 = iio_channel_get(dev, "auxadc3");
		if (IS_ERR(info->iio_channel_3))
			pr_err("CHANNEL 3 ERR \n");

		info->iio_channel_4 = iio_channel_get(dev, "auxadc4");
		if (IS_ERR(info->iio_channel_4))
			pr_err("CHANNEL 4 ERR \n");

		rc = of_property_read_u32_array(np, "oplus,batt_id_ntc",
				info->ntc_map[BATT_ID_NTC], 3);
		if (rc < 0)
			pr_err("no oplus,batt_id_ntc map\n");
		else
			ntc_present[BATT_ID_NTC] = true;

		rc = of_property_read_u32_array(np, "oplus,flash_ntc",
				info->ntc_map[FLASH_NTC], 3);
		if (rc < 0)
			pr_err("no oplus,flash_ntc map\n");
		else
			ntc_present[FLASH_NTC] = true;

		rc = of_property_read_u32_array(np, "oplus,charger_ntc",
				info->ntc_map[CHARGER_NTC], 3);
		if (rc < 0)
			pr_err("no oplus,charger_ntc map\n");
		else
			ntc_present[CHARGER_NTC] = true;

		rc = of_property_read_u32_array(np, "oplus,pa_ntc",
				info->ntc_map[PA_NTC], 3);
		if (rc < 0)
			pr_err("no oplus,pa_ntc map\n");
		else
			ntc_present[PA_NTC] = true;

		rc = of_property_read_u32_array(np, "oplus,bb_ntc",
				info->ntc_map[BB_NTC], 3);
		if (rc < 0)
			pr_err("no oplus,bb_ntc map\n");
		else
			ntc_present[BB_NTC] = true;

		rc = of_property_read_u32_array(np, "oplus,bat_con_ntc",
				info->ntc_map[BAT_CON_NTC], 3);
		if (rc < 0)
			pr_err("no oplus,bat_con_ntc map\n");
		else
			ntc_present[BAT_CON_NTC] = true;
	}

	return 0;
}

static int oplus_tempntc_data_init(struct temp_data *info)
{
	info->flash_ntc_volt = -1;
	info->batt_id_volt = -1;
	info->pa_ntc_volt = -1;
	info->charger_ntc_volt = -1;
	info->bb_ntc_volt = -1;
	info->bat_con_ntc_volt = -1;
	mutex_init(&info->det_lock);

	return 0;
}

static int oplus_ntcctrl_gpio_init(struct temp_data *info,struct device *dev)
{
	if (!info) {
		pr_err("info  null !\n");
		return -EINVAL;
	}

	info->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(info->pinctrl)) {
		pr_err("get temp ntc princtrl fail\n");
		return -EINVAL;
	}

	if (info->ntcswitchgpio_number == 1) {
		info->ntc_switch1_ctrl_high = pinctrl_lookup_state(info->pinctrl, "ntc_switch1_ctrl_high");
		if (IS_ERR_OR_NULL(info->ntc_switch1_ctrl_high)) {
			pr_err("get ntc_switch1_ctrl_high fail\n");
			return -EINVAL;
		}

		info->ntc_switch1_ctrl_low = pinctrl_lookup_state(info->pinctrl, "ntc_switch1_ctrl_low");
		if (IS_ERR_OR_NULL(info->ntc_switch1_ctrl_low)) {
			pr_err("get ntc_switch1_ctrl_low fail\n");
			return -EINVAL;
		}

		pinctrl_select_state(info->pinctrl, info->ntc_switch1_ctrl_low);
	} else {
		info->ntc_switch1_ctrl_high = pinctrl_lookup_state(info->pinctrl, "ntc_switch1_ctrl_high");
		if (IS_ERR_OR_NULL(info->ntc_switch1_ctrl_high)) {
			pr_err("get ntc_switch1_ctrl_high fail\n");
			return -EINVAL;
		}

		info->ntc_switch1_ctrl_low = pinctrl_lookup_state(info->pinctrl, "ntc_switch1_ctrl_low");
		if (IS_ERR_OR_NULL(info->ntc_switch1_ctrl_low)) {
			pr_err("get ntc_switch1_ctrl_low fail\n");
			return -EINVAL;
		}

		info->ntc_switch2_ctrl_high = pinctrl_lookup_state(info->pinctrl, "ntc_switch2_ctrl_high");
		if (IS_ERR_OR_NULL(info->ntc_switch2_ctrl_high)) {
			pr_err("get ntc_switch2_ctrl_high fail\n");
			return -EINVAL;
		}

		info->ntc_switch2_ctrl_low = pinctrl_lookup_state(info->pinctrl, "ntc_switch2_ctrl_low");
		if (IS_ERR_OR_NULL(info->ntc_switch2_ctrl_low)) {
			pr_err("get ntc_switch2_ctrl_low fail\n");
			return -EINVAL;
		}

		pinctrl_select_state(info->pinctrl, info->ntc_switch2_ctrl_low);
		msleep(100);
		pinctrl_select_state(info->pinctrl, info->ntc_switch1_ctrl_low);
	}
	return 0;

}


static int oplus_tempntc_pdrv_probe(struct platform_device *pdev)
{
	struct temp_data *info;

	pr_err("starts\n");
	info = devm_kzalloc(&pdev->dev,sizeof(struct temp_data), GFP_KERNEL);
	if (!info) {
		pr_err(" kzalloc() failed\n");
		return -ENOMEM;
	}

	pinfo = info;
	platform_set_drvdata(pdev, info);
	info->pdev = pdev;

	oplus_tempntc_data_init(info);
	oplus_tempntc_parse_dt(info, &pdev->dev);

	if (!pinfo->disable_ntc_switch){
		pr_err("Probe This project will use ntc switch  \n");
		oplus_ntcctrl_gpio_init(info, &pdev->dev);
	}

	INIT_DELAYED_WORK(&info->init_work, oplus_tempntc_det_work);
	schedule_delayed_work(&info->init_work, msecs_to_jiffies(5000));

	return 0;
}

static int oplus_tempntc_pdrv_remove(struct platform_device *pdev)
{
	return 0;
}

static void oplus_tempntc_pdrv_shutdown(struct platform_device *dev)
{

}

static const struct of_device_id oplus_tempntc_of_match[] = {
	{.compatible = "oplus-tempntc",},
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
			msecs_to_jiffies(5000));
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
		.pm = &tempntc_pm_ops,
	},
};

static int __init oplus_tempntc_init(void)
{
    int ret;
	ret = platform_driver_register(&oplus_tempntc_driver);
    if (ret) {
		pr_err("%s fail to tempntc device\n");
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

MODULE_AUTHOR("wy.chuang <wy.chuang@mediatek.com>");
MODULE_DESCRIPTION("MTK Gauge Device Driver");
MODULE_LICENSE("GPL");
