// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>

#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/rtc.h>
#include <soc/oplus/device_info.h>

#include "oplus_charger.h"
#include "oplus_wrap.h"
#include "oplus_gauge.h"
#include "oplus_adapter.h"
#include "oplus_wireless.h"

static struct oplus_wpc_chip *g_wpc_chip = NULL;

void oplus_wpc_set_otg_en_val(int value)
{
	return;
}

int oplus_wpc_get_otg_en_val(struct oplus_chg_chip *chip)
{
	return 0;
}

void oplus_wpc_set_vbat_en_val(int value)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return;
	}
	if (g_wpc_chip->wpc_ops && g_wpc_chip->wpc_ops->wpc_set_vbat_en) {
		g_wpc_chip->wpc_ops->wpc_set_vbat_en(value);
		return;
	} else {
		return;
	}
}

void oplus_wpc_set_booster_en_val(int value)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return;
	}
	if (g_wpc_chip->wpc_ops && g_wpc_chip->wpc_ops->wpc_set_booster_en) {
		g_wpc_chip->wpc_ops->wpc_set_booster_en(value);
		return;
	} else {
		return;
	}
}


void oplus_wpc_set_ext1_wired_otg_en_val(int value)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return;
	}
	if (g_wpc_chip->wpc_ops && g_wpc_chip->wpc_ops->wpc_set_ext1_wired_otg_en) {
		g_wpc_chip->wpc_ops->wpc_set_ext1_wired_otg_en(value);
		return;
	} else {
		return;
	}
}

void oplus_wpc_set_ext2_wireless_otg_en_val(int value)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return;
	}
	if (g_wpc_chip->wpc_ops && g_wpc_chip->wpc_ops->wpc_set_ext2_wireless_otg_en) {
		g_wpc_chip->wpc_ops->wpc_set_ext2_wireless_otg_en(value);
		return;
	} else {
		return;
	}
}

void oplus_wpc_set_rtx_function_prepare(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return;
	}
	if (g_wpc_chip->wpc_ops && g_wpc_chip->wpc_ops->wpc_set_rtx_function_prepare) {
		g_wpc_chip->wpc_ops->wpc_set_rtx_function_prepare();
		return;
	} else {
		return;
	}
}

void oplus_wpc_set_rtx_function(bool enable)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return;
	}
	if (g_wpc_chip->wpc_ops && g_wpc_chip->wpc_ops->wpc_set_rtx_function) {
		g_wpc_chip->wpc_ops->wpc_set_rtx_function(enable);
		return;
	} else {
		return;
	}
}

void oplus_wpc_dis_wireless_chg(int value)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return;
	}
	if (g_wpc_chip->wpc_ops && g_wpc_chip->wpc_ops->wpc_dis_wireless_chg) {
		g_wpc_chip->wpc_ops->wpc_dis_wireless_chg(value);
		return;
	} else {
		return;
	}
}

int oplus_wpc_get_idt_en_val(void)
{
	return 0;
}

bool oplus_wpc_get_wired_otg_online(void)
{
	return false;
}

bool oplus_wpc_get_wired_chg_present(void)
{
	return false;
}

void oplus_wpc_dcin_irq_enable(bool enable)
{
	return;
}



bool oplus_wpc_get_wireless_charge_start(void)
{
	if (!g_wpc_chip) {
		//chg_err("g_wpc_chip null, return\n");
		return false;
	}
	if (g_wpc_chip->wpc_ops && g_wpc_chip->wpc_ops->wpc_get_wireless_charge_start) {
		return g_wpc_chip->wpc_ops->wpc_get_wireless_charge_start();
	} else {
		return false;
	}
}

bool oplus_wpc_get_normal_charging(void)
{
	if (!g_wpc_chip) {
		//chg_err("g_wpc_chip null, return\n");
		return false;
	}
	if (g_wpc_chip->wpc_ops && g_wpc_chip->wpc_ops->wpc_get_normal_charging) {
		return g_wpc_chip->wpc_ops->wpc_get_normal_charging();
	} else {
		return false;
	}
}

bool oplus_wpc_get_fast_charging(void)
{
	if (!g_wpc_chip) {
		//chg_err("g_wpc_chip null, return\n");
		return false;
	}
	if (g_wpc_chip->wpc_ops && g_wpc_chip->wpc_ops->wpc_get_fast_charging) {
		return g_wpc_chip->wpc_ops->wpc_get_fast_charging();
	} else {
		return false;
	}
}

bool oplus_wpc_get_otg_charging(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return false;
	}
	if (g_wpc_chip->wpc_ops && g_wpc_chip->wpc_ops->wpc_get_otg_charging) {
		return g_wpc_chip->wpc_ops->wpc_get_otg_charging();
	} else {
		return false;
	}
}



bool oplus_wpc_get_ffc_charging(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return false;
	}
	if (g_wpc_chip->wpc_ops && g_wpc_chip->wpc_ops->wpc_get_ffc_charging) {
		return g_wpc_chip->wpc_ops->wpc_get_ffc_charging();
	} else {
		return false;
	}
}

bool oplus_wpc_check_chip_is_null(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return true;
	}
	return false;
}

bool oplus_wpc_get_fw_updating(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return false;
	}
	if (g_wpc_chip->wpc_ops && g_wpc_chip->wpc_ops->wpc_get_fw_updating) {
		return g_wpc_chip->wpc_ops->wpc_get_fw_updating();
	} else {
		return false;
	}
}

int oplus_wpc_get_adapter_type(void)
{
	if (!g_wpc_chip) {
		//chg_err("g_wpc_chip null, return\n");
		return -EINVAL;
	}
	if (g_wpc_chip->wpc_ops && g_wpc_chip->wpc_ops->wpc_get_adapter_type) {
		return g_wpc_chip->wpc_ops->wpc_get_adapter_type();
	} else {
		return 0;
	}
}

int oplus_wpc_set_tx_start(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return -EINVAL;
	}
	if (g_wpc_chip->wpc_ops && g_wpc_chip->wpc_ops->wpc_set_tx_start) {
		return g_wpc_chip->wpc_ops->wpc_set_tx_start();
	} else {
		return 0;
	}
}

void oplus_wpc_dis_tx_power(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return;
	}
	if (g_wpc_chip->wpc_ops->wpc_dis_tx_power) {
		g_wpc_chip->wpc_ops->wpc_dis_tx_power();
		return;
	} else {
		return;
	}
}

void oplus_wpc_print_log(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return;
	}
	if (g_wpc_chip->wpc_ops && g_wpc_chip->wpc_ops->wpc_print_log) {
		g_wpc_chip->wpc_ops->wpc_print_log();
		return;
	} else {
		return;
	}
}

void oplus_wpc_set_wrx_en_value(int value)
{
	struct oplus_wpc_chip *chip = g_wpc_chip;
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_wpc_chip not ready!\n", __func__);
		return;
	}
	if (chip->wpc_gpios.wrx_en_gpio <= 0) {
		chg_err("wrx_en_gpio not exist, return\n");
		return;
	}
	if (IS_ERR_OR_NULL(chip->wpc_gpios.pinctrl)
		|| IS_ERR_OR_NULL(chip->wpc_gpios.wrx_en_active)
		|| IS_ERR_OR_NULL(chip->wpc_gpios.wrx_en_sleep)) {
		chg_err("pinctrl null, return\n");
		return;
	}
	if (value == 1) {
		gpio_direction_output(chip->wpc_gpios.wrx_en_gpio, 1);
		pinctrl_select_state(chip->wpc_gpios.pinctrl,
			chip->wpc_gpios.wrx_en_active);
	} else {
		pinctrl_select_state(chip->wpc_gpios.pinctrl,
			chip->wpc_gpios.wrx_en_sleep);
	}
	chg_err("set value:%d, gpio_val:%d\n",
		value, gpio_get_value(chip->wpc_gpios.wrx_en_gpio));
}


int oplus_wpc_get_wrx_en_val(void)
{
	struct oplus_wpc_chip *chip = g_wpc_chip;
	if (!chip) {
		chg_err("oplus_wpc_chip not ready!\n", __func__);
		return 0;
	}
	if (chip->wpc_gpios.wrx_en_gpio <= 0) {
		chg_err("wrx_en_gpio not exist, return\n");
		return 0;
	}
	if (IS_ERR_OR_NULL(chip->wpc_gpios.pinctrl)
		|| IS_ERR_OR_NULL(chip->wpc_gpios.wrx_en_active)
		|| IS_ERR_OR_NULL(chip->wpc_gpios.wrx_en_sleep)) {
		chg_err("pinctrl null, return\n");
		return 0;
	}
	return gpio_get_value(chip->wpc_gpios.wrx_en_gpio);
}

int oplus_wpc_get_wrx_otg_en_val(void)
{
	struct oplus_wpc_chip *chip = g_wpc_chip;
	if (!chip) {
		chg_err("oplus_wpc_chip not ready!\n", __func__);
		return 0;
	}
	if (chip->wpc_gpios.wrx_otg_en_gpio <= 0) {
		chg_err("wrx_otg_en_gpio not exist, return\n");
		return 0;
	}
	if (IS_ERR_OR_NULL(chip->wpc_gpios.pinctrl)
		|| IS_ERR_OR_NULL(chip->wpc_gpios.wrx_otg_en_active)
		|| IS_ERR_OR_NULL(chip->wpc_gpios.wrx_otg_en_sleep)) {
		chg_err("pinctrl null, return\n");
		return 0;
	}
	return gpio_get_value(chip->wpc_gpios.wrx_otg_en_gpio);
}

void oplus_wpc_set_wrx_otg_en_value(int value)
{
	struct oplus_wpc_chip *chip = g_wpc_chip;
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_wpc_chip not ready!\n", __func__);
		return;
	}
	if (chip->wpc_gpios.wrx_otg_en_gpio <= 0) {
		chg_err("wrx_otg_en_gpio not exist, return\n");
		return;
	}
	if (IS_ERR_OR_NULL(chip->wpc_gpios.pinctrl)
		|| IS_ERR_OR_NULL(chip->wpc_gpios.wrx_otg_en_active)
		|| IS_ERR_OR_NULL(chip->wpc_gpios.wrx_otg_en_sleep)) {
		chg_err("pinctrl null, return\n");
		return;
	}
	if (value == 1) {
		gpio_direction_output(chip->wpc_gpios.wrx_otg_en_gpio, 1);
		pinctrl_select_state(chip->wpc_gpios.pinctrl,
			chip->wpc_gpios.wrx_otg_en_active);
	} else {
		pinctrl_select_state(chip->wpc_gpios.pinctrl,
			chip->wpc_gpios.wrx_otg_en_sleep);
	}
	chg_err("set value:%d, gpio_val:%d\n",
		value, gpio_get_value(chip->wpc_gpios.wrx_otg_en_gpio));
}

int oplus_wpc_get_real_type(void)
{
	struct oplus_wpc_chip *chip = g_wpc_chip;
	int real_type = POWER_SUPPLY_TYPE_UNKNOWN;

	if (!chip || !chip->wpc_ops || !chip->wpc_ops->wpc_get_real_type) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_wpc_chip not ready!\n", __func__);
		return real_type;
        }

	real_type = chip->wpc_ops->wpc_get_real_type();

	return real_type;
}

static enum power_supply_property oplus_wpc_wireless_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int oplus_wpc_wireless_get_prop(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct oplus_wpc_chip *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		if (chip && chip->wpc_ops
			&& chip->wpc_ops->wpc_get_online_status) {
			val->intval = chip->wpc_ops->wpc_get_online_status();
		} else {
			val->intval = 0;
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (chip && chip->wpc_ops
                        && chip->wpc_ops->wpc_get_voltage_now) {
                        val->intval = chip->wpc_ops->wpc_get_voltage_now();
                } else {
                        val->intval = 0;
                }
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (chip && chip->wpc_ops
                        && chip->wpc_ops->wpc_get_current_now) {
                        val->intval = chip->wpc_ops->wpc_get_current_now();
                } else {
                        val->intval = 0;
                }
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	if (rc < 0) {
		pr_debug("Couldn't get prop %d rc = %d\n", psp, rc);
		return -ENODATA;
	}
	return 0;
}

static int oplus_wpc_wireless_set_prop(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		break;
	default:
		chg_err("set prop %d is not supported\n", psp);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int oplus_wpc_wireless_prop_is_writeable(struct power_supply *psy,
					    enum power_supply_property psp)
{
	int rc;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}

	return rc;
}


static const struct power_supply_desc oplus_wpc_wireless_psy_desc = {
	.name = "wireless",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = oplus_wpc_wireless_props,
	.num_properties = ARRAY_SIZE(oplus_wpc_wireless_props),
	.get_property = oplus_wpc_wireless_get_prop,
	.set_property = oplus_wpc_wireless_set_prop,
	.property_is_writeable = oplus_wpc_wireless_prop_is_writeable,
};

static int oplus_wpc_init_wireless_psy(struct oplus_wpc_chip *chip)
{
	struct power_supply_config wireless_cfg = {};

	wireless_cfg.drv_data = chip;
	wireless_cfg.of_node = chip->dev->of_node;
	chip->wireless_psy = devm_power_supply_register(
		chip->dev, &oplus_wpc_wireless_psy_desc, &wireless_cfg);
	if (IS_ERR(chip->wireless_psy)) {
		chg_err("Couldn't register wireless power supply\n");
		return PTR_ERR(chip->wireless_psy);
	}

	return 0;
}


void oplus_wpc_init(struct oplus_wpc_chip *chip)
{
	g_wpc_chip = chip;

	oplus_wpc_init_wireless_psy(chip);
}
