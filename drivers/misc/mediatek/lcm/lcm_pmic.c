/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/kernel.h>

#ifdef OPLUS_BUG_STABILITY
#ifdef CONFIG_SET_LCD_BIAS_ODM_HQ
typedef enum {
	FIRST_VSP_AFTER_VSN = 0,
	FIRST_VSN_AFTER_VSP = 1
} LCD_BIAS_POWER_ON_SEQUENCE;
#endif //CONFIG_SET_LCD_BIAS_ODM_HQ
#endif /* OPLUS_BUG_STABILITY */

/*#ifdef OPLUS_BUG_STABILITY*/
#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV) || defined(CONFIG_SET_LCD_BIAS_ODM_HQ)
/*#else*/
/*#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)*/
/*#endif*/
static struct regulator *disp_bias_pos;
static struct regulator *disp_bias_neg;
static int regulator_inited;

int display_bias_regulator_init(void)
{
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(disp_bias_pos);
		pr_info("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		pr_info("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}
EXPORT_SYMBOL(display_bias_regulator_init);

int disp_late_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_info("enable regulator disp_bias_pos fail, ret = %d\n",
		ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_info("enable regulator disp_bias_neg fail, ret = %d\n",
		ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(disp_late_bias_enable);

int display_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 5400000, 5400000);
	if (ret < 0)
		pr_info("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 5400000, 5400000);
	if (ret < 0)
		pr_info("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

#if 0
	/* get voltage */
	ret = mtk_regulator_get_voltage(&disp_bias_pos);
	if (ret < 0)
		pr_info("get voltage disp_bias_pos fail\n");
	pr_debug("pos voltage = %d\n", ret);

	ret = mtk_regulator_get_voltage(&disp_bias_neg);
	if (ret < 0)
		pr_info("get voltage disp_bias_neg fail\n");
	pr_debug("neg voltage = %d\n", ret);
#endif
	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_info("enable regulator disp_bias_pos fail, ret = %d\n",
			ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_info("enable regulator disp_bias_neg fail, ret = %d\n",
			ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_enable);

int display_bias_disable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_info("disable regulator disp_bias_neg fail, ret = %d\n",
			ret);
	retval |= ret;

	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_info("disable regulator disp_bias_pos fail, ret = %d\n",
			ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_disable);

#else
int display_bias_regulator_init(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_regulator_init);

int display_bias_enable(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_enable);

int disp_late_bias_enable(void)
{
	return 0;
}
EXPORT_SYMBOL(disp_late_bias_enable);

int display_bias_disable(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_disable);
#endif

#ifdef OPLUS_BUG_STABILITY
#ifdef CONFIG_SET_LCD_BIAS_ODM_HQ
void pmi_lcd_bias_set_vspn_vol(unsigned int value)
{
	int ret = 0;
	unsigned int level;

	display_bias_regulator_init();

	level = value * 1000;

	ret = regulator_set_voltage(disp_bias_pos, level, level);
	if (ret < 0)
		pr_info("set voltage disp_bias_pos fail, ret = %d\n", ret);

	ret = regulator_set_voltage(disp_bias_neg, level, level);
	if (ret < 0)
		pr_info("set voltage disp_bias_neg fail, ret = %d\n", ret);
}
EXPORT_SYMBOL(pmi_lcd_bias_set_vspn_vol);

void pmi_lcd_bias_set_vspn_en(unsigned int en, unsigned int seq)
{
	int retval = 0;

	display_bias_regulator_init();

    if (en) {			/* enable regulator */
		if (seq == FIRST_VSP_AFTER_VSN) {
            retval |= regulator_enable(disp_bias_pos);
	        //mdelay(5);
			retval |= regulator_enable(disp_bias_neg);
		} else if (seq == FIRST_VSN_AFTER_VSP) {
		    retval |= regulator_enable(disp_bias_neg);
			//mdelay(5);
			retval |= regulator_enable(disp_bias_pos);

		}
		if (retval < 0)
		pr_info("enable regulator disp_bias fail, retval = %d\n", retval);
    } else {			/* disable regulator */
        if (seq == FIRST_VSP_AFTER_VSN) {
			retval |= regulator_disable(disp_bias_pos);
			//mdelay(5);
			retval |= regulator_disable(disp_bias_neg);
        } else if (seq == FIRST_VSN_AFTER_VSP) {
			retval |= regulator_disable(disp_bias_neg);
			//mdelay(5);
			retval |= regulator_disable(disp_bias_pos);
        }
		if (retval < 0)
		pr_info("disable regulator disp_bias fail, retval = %d\n", retval);
    }
}
EXPORT_SYMBOL(pmi_lcd_bias_set_vspn_en);


int pmi_lcd_bias_vsp_is_enabled(void)
{
	return regulator_is_enabled(disp_bias_pos);
}
EXPORT_SYMBOL(pmi_lcd_bias_vsp_is_enabled);


int pmi_lcd_bias_vsn_is_enabled(void)
{
	return regulator_is_enabled(disp_bias_neg);
}
EXPORT_SYMBOL(pmi_lcd_bias_vsn_is_enabled);
#endif //CONFIG_SET_LCD_BIAS_ODM_HQ
#endif /* OPLUS_BUG_STABILITY */

