/* Copyright (c) 2019 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <soc/oplus/system/oplus_project.h>
#include <soc/oplus/device_info.h>


//#include <soc/oplus/device_info.h>
//#include <soc/oplus/oplus_project.h>
//#include <soc/oplus/boot_mode.h>
#include <linux/regmap.h>
#define FAN53870_PRODUCT_ID_REG  0x00
#define FAN53870_LDO_ENABLE_REG  0x03
#define FAN53870_LDO1_VOUT_REG   0x04
#define FAN53870_LDO2_VOUT_REG   0x05
#define FAN53870_LDO3_VOUT_REG   0x06
#define FAN53870_LDO4_VOUT_REG   0x07
#define FAN53870_LDO5_VOUT_REG   0x08
#define FAN53870_LDO6_VOUT_REG   0x09
#define FAN53870_LDO7_VOUT_REG   0x0A
#define FAN53870_PRODUCT_ID_REG  0x00
#define FAN53870_LDO_ENABLE_REG  0x03
#define FAN53870_LDO1_VOUT_REG   0x04
#define FAN53870_LDO12_SEQ_REG   0x0B
#define FAN53870_LDO34_SEQ_REG   0x0C
#define FAN53870_LDO56_SEQ_REG   0x0D
#define FAN53870_LDO7_SEQ_REG    0x0E
#define FAN53870_LDO_NUM_MAX     8
#define FAN53870_LDO2_VOUT_BASE  800

struct fan53870_platform_data {
    unsigned int ldo1_min_vol;
    unsigned int ldo1_max_vol;
    unsigned int ldo1_step_vol;
};

/*
 * struct TPS65132_pw_chip
 * @dev: Parent device structure
 * @regmap: Used for I2C register access
 * @pdata: TPS65132 platform data
 */
struct fan53870_pw_chip {
    struct device *dev;
    struct fan53870_platform_data *pdata;
    struct regmap *regmap;
    int en_gpio;
};
static struct fan53870_pw_chip *fan53870_pchip;
int is_fan53870_1p1 = -1;
int ldo5_power_on = 0;
int ldo7_power_on = 0;
void enable_fan53870_gpio(int pwr_status)
{
    if(fan53870_pchip->en_gpio)
        gpio_set_value(fan53870_pchip->en_gpio, pwr_status);
}
EXPORT_SYMBOL(enable_fan53870_gpio);

static int is_fan53870_always_enable_on_probe(void)
{
	if (is_project(0x212A1) || is_project(21851) || is_project(21876)){
		return 1;
	}
	return 0;
}

static int fan53870_mask_write_reg(struct regmap *regmap, uint reg, uint mask, uint value)
{
    int rc;

    pr_err("Writing 0x%02x to 0x%04x with mask 0x%02x\n", value, reg, mask);
    rc = regmap_update_bits(regmap, reg, mask, value);
    if (rc < 0)
        pr_err("failed to write 0x%02x to 0x%04x with mask 0x%02x\n",
            value, reg, mask);

    return rc;
}
/*export functions*/
int fan53870_ldo1_regmap_read(void)
{
    int ret = 0;
    unsigned int reg_value;

    ret = regmap_read(fan53870_pchip->regmap, FAN53870_LDO2_VOUT_REG, &reg_value);
    if (ret == 0) {
        pr_err("read ldo1 reg 05 value = %d\n", reg_value);
        reg_value = 8000 + 8000*reg_value; /*(uv : reg_value = (800 + (reg_value - 99)*8)*1000;*/
        return reg_value;
    }
    pr_err("error read ldo1 reg 05 value ret = %d, reg_value = %d\n", ret, reg_value);

    return ret;
}
EXPORT_SYMBOL(fan53870_ldo1_regmap_read);

int fan53870_ldo1_set_voltage(unsigned int set_uV)
{
    int ret;
    unsigned int reg_value = 0x0;

    reg_value = (set_uV/8000 - 1);

    if (reg_value < 0x63 || reg_value > 0xA4) {
        pr_err("[lcd] reg_value error 0x%x!\n", reg_value);
        ret = -1;
        goto out;
    }

    pr_err("[LCD]fan53870_ldo1_set_voltage: voltage = %d , reg_value = 0x%x!\n", set_uV,reg_value);
    ret = regmap_write(fan53870_pchip->regmap, FAN53870_LDO2_VOUT_REG, reg_value);
    pr_err("[lcd]Writing 0x%02x to 0x%02x \n", reg_value, FAN53870_LDO2_VOUT_REG);
    if (ret < 0)
        goto out;

    ret = fan53870_mask_write_reg(fan53870_pchip->regmap, FAN53870_LDO12_SEQ_REG, 0x38, 0x00);
    if (ret < 0)
        goto out;

    ret = fan53870_mask_write_reg(fan53870_pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x02, 0x02);
    if (ret < 0)
        goto out;

    return 0;
out:
    pr_err("[lcd]fan53870_ldo1_set_voltagefailed!\n");
    return ret;
}
EXPORT_SYMBOL(fan53870_ldo1_set_voltage);

int fan53870_ldo1_disable(void)
{
    int ret;
    struct fan53870_pw_chip *pchip = fan53870_pchip;
    ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x02, 0x00);
    pr_err("fan53870_ldo1_disable!\n");
    return ret;
}
EXPORT_SYMBOL(fan53870_ldo1_disable);

int fan53870_ldo1_20601_set_voltage(int set_uV)
{
    int ret;
    unsigned int reg_value = 0x8E; /*1.144V*/
    struct fan53870_pw_chip *pchip = fan53870_pchip;

    pr_err("fan53870_ldo1_set_voltage: voltage = %d!\n", set_uV);
    ret = regmap_write(pchip->regmap, FAN53870_LDO1_VOUT_REG, reg_value);
    pr_info("Writing 0x%02x to 0x%02x \n", reg_value, FAN53870_LDO1_VOUT_REG);
    if (ret < 0)
        goto out;

    ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO12_SEQ_REG, 0x07, 0x00);
    if (ret < 0)
        goto out;

    ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x01, 0x01);
    if (ret < 0)
        goto out;

    return 0;
out:
    pr_err("fan53870_ldo1_20601_set_voltagefailed!\n");
    return ret;
}
EXPORT_SYMBOL(fan53870_ldo1_20601_set_voltage);

int fan53870_ldo1_20601_disable(void)
{
    int ret;
    struct fan53870_pw_chip *pchip = fan53870_pchip;
    ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x01, 0x00);
    pr_err("fan53870_ldo1_20601_disable!\n");
    return ret;
}
EXPORT_SYMBOL(fan53870_ldo1_20601_disable);


int fan53870_ldo1_20817_set_voltage(int set_uV)
{
    int ret;
    unsigned int reg_value = 0xBB; /*1.144V*/
    struct fan53870_pw_chip *pchip = fan53870_pchip;
	unsigned int readback;

    pr_err("fan53870_ldo1_20817_set_voltage: voltage = %d!\n", set_uV);
    ret = regmap_write(pchip->regmap, FAN53870_LDO1_VOUT_REG, reg_value);
    pr_info("Writing 0x%02x to 0x%02x \n", reg_value, FAN53870_LDO1_VOUT_REG);
    if (ret < 0)
        goto out;

    ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO12_SEQ_REG, 0x07, 0x00);
    if (ret < 0)
        goto out;

    ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x01, 0x01);
    if (ret < 0)
	{goto out;}

	ret = regmap_read(pchip->regmap, 0x04, &readback);
	if (ret < 0)
        pr_err("regmap_read failed!\n");
    pr_err("%s : ret:%d read 0x04:%d probe done\n", __func__, ret, readback);
	ret = regmap_read(pchip->regmap, 0x0B, &readback);
	if (ret < 0)
        pr_err("regmap_read failed!\n");
    pr_err("%s : ret:%d read 0x0B:%d probe done\n", __func__, ret, readback);
	ret = regmap_read(pchip->regmap, 0x03, &readback);
	if (ret < 0)
        pr_err("regmap_read failed!\n");
    pr_err("%s : ret:%d read 0x03:%d probe done\n", __func__, ret, readback);

    return 0;

out:
    pr_err("fan53870_ldo3_20817_set_voltagefailed!\n");
    return ret;
}
EXPORT_SYMBOL(fan53870_ldo1_20817_set_voltage);

int fan53870_ldo1_20817_disable(void)
{
    int ret;
    struct fan53870_pw_chip *pchip = fan53870_pchip;
    ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x01, 0x00);
    pr_err("fan53870_ldo3_20817_disable!\n");
    return ret;
}
EXPORT_SYMBOL(fan53870_ldo1_20817_disable);
int fan53870_ldo5_20817_set_voltage(int set_uV)
{
    int ret;
    unsigned int reg_value = 0x10; /*1.144V*/
    struct fan53870_pw_chip *pchip = fan53870_pchip;

    pr_err("fan53870_ldo5_20817_set_voltage: voltage = %d!\n", set_uV);
    ret = regmap_write(pchip->regmap, FAN53870_LDO5_VOUT_REG, reg_value);
    pr_info("Writing 0x%02x to 0x%02x \n", reg_value, FAN53870_LDO5_VOUT_REG);
    if (ret < 0)
        goto out;

    ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO56_SEQ_REG, 0x07, 0x00);
    if (ret < 0)
        goto out;

    ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x10, 0x10);
    if (ret < 0)
	{goto out;}

    return 0;

out:
    pr_err("fan53870_ldo3_20817_set_voltagefailed!\n");
    return ret;
}
EXPORT_SYMBOL(fan53870_ldo5_20817_set_voltage);
int fan53870_ldo5_20817_disable(void)
{
    int ret;
    struct fan53870_pw_chip *pchip = fan53870_pchip;
    ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x10, 0x00);
    pr_err("fan53870_ldo3_20817_disable!\n");
    return ret;
}
EXPORT_SYMBOL(fan53870_ldo5_20817_disable);
#if !defined(CONFIG_MTK_PMIC_CHIP_MT6358)
int fan53870_ldo3_20615_set_voltage(int set_uV)
{
    int ret;
    unsigned int reg_value = 0x11; /*1.144V*/
    struct fan53870_pw_chip *pchip = fan53870_pchip;

    if (!pchip || !pchip->regmap) {
        pr_err("pchip or pchip->regmap is NULL\n");
        ret = -1;
        goto out;
    }

    pr_err("fan53870_ldo3_set_voltage: voltage = %d!\n", set_uV);
    ret = regmap_write(pchip->regmap, FAN53870_LDO3_VOUT_REG, reg_value);
    pr_info("Writing 0x%02x to 0x%02x \n", reg_value, FAN53870_LDO3_VOUT_REG);
    if (ret < 0)
        goto out;

    ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO34_SEQ_REG, 0x07, 0x00);
    if (ret < 0)
        goto out;

    ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x04, 0x04);
    if (ret < 0)
        goto out;

    return 0;

out:
    pr_err("fan53870_ldo3_20615_set_voltagefailed!\n");
    return ret;
}
EXPORT_SYMBOL(fan53870_ldo3_20615_set_voltage);

int fan53870_ldo3_20615_disable(void)
{
    int ret;
    struct fan53870_pw_chip *pchip = fan53870_pchip;
    ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x04, 0x00);
    pr_err("fan53870_ldo3_20615_disable!\n");
    return ret;
}
EXPORT_SYMBOL(fan53870_ldo3_20615_disable);
#endif
int fan53870_cam_ldo_set_voltage(unsigned int LDO_NUM, int set_mv)
{
    int ret;
    unsigned int reg_value = 0; /*2.804V*/
    struct fan53870_pw_chip *pchip = fan53870_pchip;
    pr_err("fan53870_cam_ldo_set_voltage LDO_NUM:%d set_mv:%d\n", LDO_NUM, set_mv);
    switch(LDO_NUM){
        case 1:
            if (set_mv == 1050) {
                reg_value = 0x83;    /*1.056V*/
            } else if (set_mv == 1100) {
                reg_value = 0x89;    /*1.104V*/
            } else if (set_mv == 1200) {
                reg_value = 0x95;    /*1.200V*/
            } else {
                reg_value = 0x80;    /*1.032V*/
            }

            ret = regmap_write(pchip->regmap, FAN53870_LDO1_VOUT_REG, reg_value);
            pr_err("Writing 0x%02x to 0x%02x \n", reg_value, FAN53870_LDO1_VOUT_REG);
            if (ret < 0)
                goto out;

            ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO12_SEQ_REG, 0x07, 0x00);
            if (ret < 0)
                goto out;

            ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x01, 0x01);
            if (ret < 0)
                goto out;
        break;
        case 2:
		#if !defined(CONFIG_MTK_PMIC_CHIP_MT6358)
            if (set_mv == 1100) {
                reg_value = 0x89;
            } else if(set_mv == 800){
                reg_value =0x69;    /*0.848V*/
			} else if (set_mv == 1050) {
                reg_value = 0x83;    /*1.056V*/
            } else if (set_mv == 1250) {
                reg_value = 0x9B;
            } else if (set_mv == 1400) {
                reg_value = 0xAE;
            } else if (set_mv <= 800 || set_mv > 1504) {
                reg_value =0x63; /*800mv*/
            } else {
                reg_value = (set_mv - FAN53870_LDO2_VOUT_BASE)/8 + 0x63;
            }
		#else
            reg_value =0x95;    /*1.200V*/
		#endif
            ret = regmap_write(pchip->regmap, FAN53870_LDO2_VOUT_REG, reg_value);
            pr_err("Writing 0x%02x to 0x%02x \n", reg_value, FAN53870_LDO2_VOUT_REG);
            if (ret < 0)
                goto out;

            ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO12_SEQ_REG, 0x38, 0x00);
            if (ret < 0)
                goto out;

            ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x02, 0x02);
            if (ret < 0)
                goto out;
        break;
        case 3:
            if(set_mv == 2900) {
                reg_value =0xBF;/*2.900V*/
            } else if(set_mv == 2800) {
                reg_value =0xB3;/*2.804V*/
            } else {
                reg_value=0x10; /*1.5v*/
            }
            ret = regmap_write(pchip->regmap, FAN53870_LDO3_VOUT_REG, reg_value);
            pr_err("Writing 0x%02x to 0x%02x \n", reg_value, FAN53870_LDO3_VOUT_REG);
            if (ret < 0)
                goto out;

            ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO34_SEQ_REG, 0x07, 0x00);
            if (ret < 0)
                goto out;

            ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x04, 0x04);
            if (ret < 0)
                goto out;
        break;
        case 4:
            if (set_mv == 2800) {
                reg_value =0xB3;    /*2.804V*/
            } else if (set_mv == 2700) {
                reg_value =0xA6;    /*2.7V*/
            }else if (set_mv == 2900) {
                reg_value =0xBF;    /*2.9V*/
            } 
			else if (set_mv == 1800) {
                reg_value =0x36;    /*1.8V*/
            } else {
                reg_value =0x10; /*1.5V*/
            }
            ret = regmap_write(pchip->regmap, FAN53870_LDO4_VOUT_REG, reg_value);
            pr_err("Writing 0x%02x to 0x%02x \n", reg_value, FAN53870_LDO4_VOUT_REG);
            if (ret < 0)
                goto out;

            ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO34_SEQ_REG, 0x38, 0x00);
            if (ret < 0)
                goto out;

            ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x08, 0x08);
            if (ret < 0)
                goto out;
        break;
        case 5:
            if(set_mv==1800){
                reg_value = 0x36; /*1.804V*/
#if !defined(CONFIG_MTK_PMIC_CHIP_MT6358)
            } else if(set_mv == 2700){
                reg_value = 0xA6; /*2.7V*/
            } else if(set_mv == 2800){
                reg_value = 0xB3; /*2.8V*/
            } else if (set_mv == 2900) {
                reg_value = 0xBF;
            } else if(set_mv == 3300) {
                reg_value = 0xf1;    /*3.3V*/
#endif
            } else if (set_mv >= 3000 ) {
				if (is_project(21015) || is_project(21217)) {
					pr_err("reg_value = 0xf1");
					reg_value = 0xf1; /*3.300V*/
				}
            } else {
                reg_value = 0x10; /*1.5V*/
            }
            ret = regmap_write(pchip->regmap, FAN53870_LDO5_VOUT_REG, reg_value);
            pr_err("Writing 0x%02x to 0x%02x \n", reg_value, FAN53870_LDO5_VOUT_REG);
            if (ret < 0)
                goto out;

            ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO56_SEQ_REG, 0x07, 0x00);
            if (ret < 0)
                goto out;

            ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x10, 0x10);
            if (ret < 0)
                goto out;
        break;
        case 6:
            if (set_mv == 2800) {
                reg_value =0xB3; /*2.804V*/
#if !defined(CONFIG_MTK_PMIC_CHIP_MT6358)
            } else if (set_mv == 2900) {
                reg_value = 0xBF;
#endif
            } else if (set_mv == 1800) {
                reg_value =0x36; /*1.804V*/
#if !defined(CONFIG_MTK_PMIC_CHIP_MT6358)
            } else if (set_mv == 2850) {
                reg_value = 0xB9;
#endif
            } else {
                reg_value =0x10; /*1.5V*/
            }
            ret = regmap_write(pchip->regmap, FAN53870_LDO6_VOUT_REG, reg_value);
            pr_err("Writing 0x%02x to 0x%02x \n", reg_value, FAN53870_LDO6_VOUT_REG);
            if (ret < 0)
                goto out;

            ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO56_SEQ_REG, 0x38, 0x00);
            if (ret < 0)
                goto out;

            ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x20, 0x20);
            if (ret < 0)
                goto out;
        break;
        case 7:
            if (set_mv == 2800) {
                reg_value =0xB3; /*2.804V*/
            } else if (set_mv == 2900) {
                reg_value =0xBF; /*2.9V*/
#if !defined(CONFIG_MTK_PMIC_CHIP_MT6358)
            } else if (set_mv == 2850) {
                reg_value = 0xB9;
#endif
            } else if (set_mv == 3300) {
                reg_value = 0xF1; /*3.3V*/
            } else {
                reg_value =0x10; /*1.5V*/
            }
            ret = regmap_write(pchip->regmap, FAN53870_LDO7_VOUT_REG, reg_value);
            pr_err("Writing 0x%02x to 0x%02x \n", reg_value, FAN53870_LDO7_VOUT_REG);
            if (ret < 0)
                goto out;

            ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO7_SEQ_REG, 0x07, 0x00);

            if (ret < 0)
                goto out;
                        //bit6 enable ldo7
            ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x40, 0x40);
            if (ret < 0)
                goto out;
        break;
        default:
            printk("error ldo number\n");
            ret = -1;
            goto out;
        break;
    }
    return 0;
out:
    pr_err("fan53870_cam_ldo_set_voltage error!\n");
    return ret;
}
EXPORT_SYMBOL(fan53870_cam_ldo_set_voltage);

int fan53870_cam_ldo_disable(unsigned int LDO_NUM)
{
    int ret;
    struct fan53870_pw_chip *pchip = fan53870_pchip;
    pr_info("fan53870_cam_ldo_disable LDO-%d\n", LDO_NUM);
    if (LDO_NUM > 0 && LDO_NUM < FAN53870_LDO_NUM_MAX) {
        LDO_NUM = 1 << (LDO_NUM - 1);
        ret = fan53870_mask_write_reg(pchip->regmap, FAN53870_LDO_ENABLE_REG, LDO_NUM, 0x00);
        pr_info("fan53870_cam_ldo_disable channel:0x%x!\n", LDO_NUM);
    } else {
        ret = -1;
        pr_err("fan53870_cam_ldo_disable invalid LDO_NUM\n");
    }

    return ret;
}
EXPORT_SYMBOL(fan53870_cam_ldo_disable);

int is_fan53870_pmic(void)
{
    pr_err("[hjz_debug]probe pmic, is_fan53870_1p1: %d \n", is_fan53870_1p1);
    return is_fan53870_1p1;
}
EXPORT_SYMBOL(is_fan53870_pmic);

static int fan53870_dt(struct device *dev, struct fan53870_platform_data *pdata)
{
    int rc = 0;
    struct device_node *np = dev->of_node;
    int reset_gpio = 0;

    rc = of_property_read_u32(np, "ldo1_min_vol", &pdata->ldo1_min_vol);
    if (rc < 0) {
        pr_err("%s: failed to get regulator base rc=%d\n", "ldo1_min_vol", rc);
        return rc;
    }
    rc = of_property_read_u32(np, "ldo1_max_vol", &pdata->ldo1_max_vol);
    if (rc < 0) {
        pr_err("%s: failed to get regulator base rc=%d\n", "ldo1_max_vol", rc);
        return rc;
    }
    rc = of_property_read_u32(np, "ldo1_step_vol", &pdata->ldo1_step_vol);
    if (rc < 0) {
        pr_err("%s: failed to get regulator base rc=%d\n", "ldo1_step_vol", rc);
        return rc;
    }
    pr_err("%s: ldo1_min_vol=%d, ldo1_max_vol=%d, ldo1_step_vol=%d\n", __func__,
           pdata->ldo1_min_vol, pdata->ldo1_max_vol, pdata->ldo1_step_vol);
    rc = of_property_read_u32(np, "fan53870,ldo5-always-on", &ldo5_power_on);
    if (rc < 0) {
        pr_err("failed to request fan53870,ldo5-always-on rc=%d\n", rc);
        ldo5_power_on = 0;
    }
    rc = of_property_read_u32(np, "fan53870,ldo7-always-on", &ldo7_power_on);
    if (rc < 0) {
        pr_err("failed to request fan53870,ldo7-always-on rc=%d\n", rc);
        ldo7_power_on = 0;
    }
    /*get reset resource*/
    reset_gpio = of_get_named_gpio(np, "fan53870,gpio_rst", 0);
    if (!gpio_is_valid(reset_gpio)) {
        pr_err("RESET GPIO is invalid.\n");
        return 0;
    }
    rc = gpio_request(reset_gpio, "fan53870_reset");
    if (rc) {
        pr_err("Failed to request RESET GPIO. rc = %d\n", rc);
        return 0;
    }
    gpio_direction_output(reset_gpio, 1);

    return 0;
}

static struct regmap_config fan53870_regmap = {
    .reg_bits = 8,
    .val_bits = 8,
};

static int fan53870_pmic_probe(struct i2c_client *client,
               const struct i2c_device_id *id)
{
    struct fan53870_platform_data *pdata = client->dev.platform_data;
    struct fan53870_pw_chip *pchip;
#if defined(CONFIG_MACH_MT6877)
    struct device *dev;
    struct device_node *np;
#else
    struct device *dev;
    struct device_node *np;
#endif
    unsigned int product_id;
    int ret = 0;
    int access_time = 3;

    pr_err("%s Enter\n", __func__);

    if (client->dev.of_node) {
        pdata = devm_kzalloc(&client->dev,
            sizeof(struct fan53870_platform_data), GFP_KERNEL);
        if (!pdata) {
            dev_err(&client->dev, "Failed to allocate memory\n");
            return -ENOMEM;
        }

        ret = fan53870_dt(&client->dev, pdata);
        if (ret)
            return ret;
    } else
        pdata = client->dev.platform_data;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "fail : i2c functionality check...\n");
        return -EOPNOTSUPP;
    }

    if (pdata == NULL) {
        dev_err(&client->dev, "fail : no platform data.\n");
        return -ENODATA;
    }

    pchip = devm_kzalloc(&client->dev, sizeof(struct fan53870_pw_chip),
                 GFP_KERNEL);
    if (!pchip) {
        dev_err(&client->dev, "no memory to alloc struct fan53870_pw_chip\n");
        return -ENOMEM;
    }
    fan53870_pchip = pchip;
    pchip->pdata = pdata;
    pchip->dev = &client->dev;
#if defined(CONFIG_MACH_MT6877)
    dev = &client->dev;
    np = dev->of_node;
    pchip->en_gpio = of_get_named_gpio(np, "en-gpios", 0);
    if (pchip->en_gpio < 0) {
        pr_err("fan53870_pchip->en_gpio not specified\n");
    }
    if (!gpio_is_valid(pchip->en_gpio) ) {
        pr_err("fan53870_pchip en gpio\n");
        return -EINVAL;
    }
#else
    if (is_project(21127) || is_project(21305)) {
        dev = &client->dev;
        np = dev->of_node;
        pchip->en_gpio = of_get_named_gpio(np, "en-gpios", 0);
        if (pchip->en_gpio < 0) {
            pr_err("fan53870_pchip->en_gpio not specified\n");
        }
        if (!gpio_is_valid(pchip->en_gpio) ) {
            pr_err("fan53870_pchip en gpio\n");
            return -EINVAL;
        }
    }
#endif
    pchip->regmap = devm_regmap_init_i2c(client, &fan53870_regmap);
    if (IS_ERR(pchip->regmap)) {
        ret = PTR_ERR(pchip->regmap);
        dev_err(&client->dev, "fail : allocate register map: %d\n",
            ret);
        goto error_enable;
    }

    i2c_set_clientdata(client, pchip);
    enable_fan53870_gpio(1);
    ret = regmap_read(pchip->regmap, FAN53870_PRODUCT_ID_REG, &product_id);
    while (ret == -1 && --access_time) {
        mdelay(2);
        ret = regmap_read(pchip->regmap, FAN53870_PRODUCT_ID_REG, &product_id);
    }
    if(ret == 0){
        is_fan53870_1p1 = (product_id == 0x01) ? 1 : 0;
    }else{
        is_fan53870_1p1 = -1;
        pr_err("%s : ret:%d regmap_read error fan53870 access failed!\n", __func__, ret);
    }
    if (ldo5_power_on == 1) { //for fingerprint 3.3v
    pr_info("%s set fan53870 ldo5 alway on 3.3v\n", __func__);
    fan53870_cam_ldo_set_voltage(5, 3300);
    }
    if (ldo7_power_on == 1) { //for fingerprint 3.3v
    pr_info("%s set fan53870 ldo7 alway on 3.3v\n", __func__);
    fan53870_cam_ldo_set_voltage(7, 3300);
    }
    pr_err("%s : ret:%d product_id:%d probe done\n", __func__, ret, product_id);
    if (!is_fan53870_always_enable_on_probe()) {
        enable_fan53870_gpio(0);
    }
    return 0;
error_enable:
    devm_kfree(&client->dev, pchip->pdata);
    devm_kfree(&client->dev, pchip);
    pr_err("%s : probe failed\n", __func__);
    return ret;
}

static int fan53870_pmic_remove(struct i2c_client *client)
{
    struct fan53870_pw_chip *pchip = i2c_get_clientdata(client);
    int ret = 0;

    if(!pchip || !pchip->regmap || !pchip->pdata) {
        dev_err(&client->dev, "%s, have null pointer!\n", __func__);
        return -1;
    }

    dev_info(pchip->dev, "%s :  failed\n", __func__);

    ret = regmap_write(pchip->regmap, FAN53870_LDO_ENABLE_REG, 0x00);
    if (ret < 0)
        dev_err(pchip->dev, "i2c failed to access register\n");

    return 0;
}

static const struct i2c_device_id fan53870_pmic_ids[] = {
    { "fan53870", 0 },
    { }
};

static struct i2c_driver fan53870_i2c_driver = {
    .probe = fan53870_pmic_probe,
    .remove = fan53870_pmic_remove,
    .driver = {
        .name = "fan53870",
        .owner = THIS_MODULE,
    },
    .id_table = fan53870_pmic_ids,
};

module_i2c_driver(fan53870_i2c_driver);

MODULE_DESCRIPTION("FAN53870 Power Driver");
MODULE_AUTHOR("xxx");
MODULE_LICENSE("GPL");

