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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"
/* device tree should be defined in flashlight-dt.h */
#ifndef ZHAOYUN_DTNAME
#define ZHAOYUN_DTNAME "mediatek,flashlights_zhaoyun"
#endif
#ifndef ZHAOYUN_DTNAME_I2C
#define ZHAOYUN_DTNAME_I2C   "mediatek,strobe_main"
#endif

#define ZHAOYUN_NAME "flashlights-zhaoyun"

/* define registers */
#define ZHAOYUN_REG_ENABLE           (0x01)
#define ZHAOYUN_MASK_ENABLE_LED1     (0x01)
#define ZHAOYUN_MASK_ENABLE_LED2     (0x02)
#define ZHAOYUN_DISABLE              (0x00)
#define ZHAOYUN_TORCH_MODE           (0x08)
#define ZHAOYUN_FLASH_MODE           (0x0C)
#define ZHAOYUN_ENABLE_LED1          (0x01)
#define ZHAOYUN_ENABLE_LED1_TORCH    (0x09)
#define ZHAOYUN_ENABLE_LED1_FLASH    (0x0D)
#define ZHAOYUN_ENABLE_LED2          (0x02)
#define ZHAOYUN_ENABLE_LED2_TORCH    (0x0A)
#define ZHAOYUN_ENABLE_LED2_FLASH    (0x0E)

#define ZHAOYUNLITE_ENABLE_LED1_TORCH    (0x0B)
#define ZHAOYUNLITE_ENABLE_LED1_FLASH    (0x0F)
#define ZHAOYUN_REG_TORCH_LEVEL_LED1 (0x05)
#define ZHAOYUN_REG_FLASH_LEVEL_LED1 (0x03)
#define ZHAOYUN_REG_TORCH_LEVEL_LED2 (0x06)
#define ZHAOYUN_REG_FLASH_LEVEL_LED2 (0x04)

#define ZHAOYUN_REG_TIMING_CONF      (0x08)
#define ZHAOYUN_TORCH_RAMP_TIME      (0x10)
#define ZHAOYUN_FLASH_TIMEOUT        (0x0F)

#define ZHAOYUN_AW36515_SOFT_RESET_ENABLE (0x80)
#define ZHAOYUN_AW36515_REG_BOOST_CONFIG (0x07)


/* define channel, level */
#define ZHAOYUN_CHANNEL_NUM          2
#define ZHAOYUN_CHANNEL_CH1          0
#define ZHAOYUN_CHANNEL_CH2          1
/* define level */
#define ZHAOYUN_LEVEL_NUM 26
#define ZHAOYUN_LEVEL_TORCH 7

#define ZHAOYUN_HW_TIMEOUT 400 /* ms */

/* define mutex and work queue */
static DEFINE_MUTEX(zhaoyun_mutex);
static struct work_struct zhaoyun_work_ch1;
static struct work_struct zhaoyun_work_ch2;

/* define pinctrl */
#define ZHAOYUN_PINCTRL_PIN_HWEN 0
#define ZHAOYUN_PINCTRL_PIN_TEMP 1
#define ZHAOYUN_PINCTRL_PINSTATE_LOW 0
#define ZHAOYUN_PINCTRL_PINSTATE_HIGH 1
#define ZHAOYUN_PINCTRL_STATE_HWEN_HIGH "zhaoyun_hwen_high"
#define ZHAOYUN_PINCTRL_STATE_HWEN_LOW  "zhaoyun_hwen_low"
#define ZHAOYUN_PINCTRL_STATE_TEMP_HIGH "zhaoyun_temp_high"
#define ZHAOYUN_PINCTRL_STATE_TEMP_LOW  "zhaoyun_temp_low"

/* define device id */
#define USE_AW3643_IC	0x0001
#define USE_SY7806_IC	0x0011
#define USE_OCP81373_IC 0x0111
#define USE_AW36515_IC  0x1111
#define USE_AW36518_IC  0x1110
#define USE_KTD2687_IC  0x1001
#define USE_NOT_PRO	0x11111

extern struct flashlight_operations sy7806_ops;
extern struct i2c_client *sy7806_i2c_client;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t zhaoyun_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t zhaoyun_set_reg(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t zhaoyun_set_hwen(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t zhaoyun_get_hwen(struct device* cd, struct device_attribute *attr, char* buf);

static DEVICE_ATTR(reg, 0660, zhaoyun_get_reg,  zhaoyun_set_reg);
static DEVICE_ATTR(hwen, 0660, zhaoyun_get_hwen,  zhaoyun_set_hwen);

struct i2c_client *zhaoyun_flashlight_client;

static struct pinctrl *zhaoyun_pinctrl;
static struct pinctrl_state *zhaoyun_hwen_high;
static struct pinctrl_state *zhaoyun_hwen_low;
static struct pinctrl_state *zhaoyun_temp_high;
static struct pinctrl_state *zhaoyun_temp_low;

/* define usage count */
static int use_count;
static int is_aw36518 = 0;

/* define i2c */
static struct i2c_client *zhaoyun_i2c_client;

/* platform data */
struct zhaoyun_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* zhaoyun chip data */
struct zhaoyun_chip_data {
	struct i2c_client *client;
	struct zhaoyun_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};

enum FLASHLIGHT_DEVICE {
	AW3643_SM = 0x12,
	SY7806_SM = 0x1c,
	OCP81373_SM = 0x3a,
	AW36515_SM = 0x02,
	AW36518_SM = 0x1a,
	KTD2687_SM = 0x01,
};

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int zhaoyun_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

       pr_err("zhaoyun_pinctrl_init start\n");
	/* get pinctrl */
	zhaoyun_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(zhaoyun_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(zhaoyun_pinctrl);
	}
	pr_err("devm_pinctrl_get finish %p\n", zhaoyun_pinctrl);

	/* Flashlight HWEN pin initialization */
	zhaoyun_hwen_high = pinctrl_lookup_state(zhaoyun_pinctrl, ZHAOYUN_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(zhaoyun_hwen_high)) {
		pr_err("Failed to init (%s)\n", ZHAOYUN_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(zhaoyun_hwen_high);
	}
	pr_err("pinctrl_lookup_state zhaoyun_hwen_high finish %p\n", zhaoyun_hwen_high);
	zhaoyun_hwen_low = pinctrl_lookup_state(zhaoyun_pinctrl, ZHAOYUN_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(zhaoyun_hwen_low)) {
		pr_err("Failed to init (%s)\n", ZHAOYUN_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(zhaoyun_hwen_low);
	}
	pr_err("pinctrl_lookup_state zhaoyun_hwen_low finish\n");
	zhaoyun_temp_high = pinctrl_lookup_state(
		zhaoyun_pinctrl, ZHAOYUN_PINCTRL_STATE_TEMP_HIGH);
	if (IS_ERR(zhaoyun_temp_high)) {
		printk("Failed to init (%s)\n", ZHAOYUN_PINCTRL_STATE_TEMP_HIGH);
		ret = PTR_ERR(zhaoyun_temp_high);
	}
	pr_err("pinctrl_lookup_state zhaoyun_temp_high finish\n");
	zhaoyun_temp_low = pinctrl_lookup_state(
		zhaoyun_pinctrl, ZHAOYUN_PINCTRL_STATE_TEMP_LOW);
	if (IS_ERR(zhaoyun_temp_low)) {
		printk("Failed to init (%s)\n", ZHAOYUN_PINCTRL_STATE_TEMP_LOW);
		ret = PTR_ERR(zhaoyun_temp_low);
	}
	pr_err("pinctrl_lookup_state zhaoyun_temp_low finish\n");
	return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int zhaoyun_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct zhaoyun_chip_data *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}

/* i2c wrapper function */
static int zhaoyun_read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	struct zhaoyun_chip_data *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (val < 0)
		pr_err("failed read at 0x%02x\n", reg);

	return val;
}

static int zhaoyun_pinctrl_set(int pin, int state)
{
	int ret = 0;
	unsigned char reg, val;

	if (IS_ERR(zhaoyun_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case ZHAOYUN_PINCTRL_PIN_HWEN:
		if (state == ZHAOYUN_PINCTRL_PINSTATE_LOW && !IS_ERR(zhaoyun_hwen_low)){
			reg = ZHAOYUN_REG_ENABLE;
			val = 0x00;
			zhaoyun_write_reg(zhaoyun_i2c_client, reg, val);
			//pinctrl_select_state(zhaoyun_pinctrl, zhaoyun_hwen_low);//rm to keep HWEN high
		}
		else if (state == ZHAOYUN_PINCTRL_PINSTATE_HIGH && !IS_ERR(zhaoyun_hwen_high))
			pinctrl_select_state(zhaoyun_pinctrl, zhaoyun_hwen_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	case ZHAOYUN_PINCTRL_PIN_TEMP:
		if (state == ZHAOYUN_PINCTRL_PINSTATE_LOW && !IS_ERR(zhaoyun_temp_low))
			ret = pinctrl_select_state(zhaoyun_pinctrl, zhaoyun_temp_low);
		else if (state == ZHAOYUN_PINCTRL_PINSTATE_HIGH && !IS_ERR(zhaoyun_temp_high))
			ret = pinctrl_select_state(zhaoyun_pinctrl, zhaoyun_temp_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_info("pin(%d) state(%d)\n", pin, state);

	return ret;
}
/******************************************************************************
 * zhaoyun operations
 *****************************************************************************/

static const int *zhaoyun_current;
static const unsigned char *zhaoyun_torch_level;
static const unsigned char *zhaoyun_flash_level;

static const int aw3643_current[ZHAOYUN_LEVEL_NUM] = {
	22,  46,  70,  93,  116, 140, 163, 198, 245, 304,
	351, 398, 445, 503,  550, 597, 656, 703, 750, 796,
	855, 902, 949, 996, 1054, 1101
};
static const unsigned char aw3643_torch_level[ZHAOYUN_LEVEL_NUM] = {
	0x06, 0x0F, 0x17, 0x1F, 0x27, 0x2F, 0x37, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned char aw3643_flash_level[ZHAOYUN_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x50, 0x54, 0x59, 0x5D
};

static const int sy7806_current[ZHAOYUN_LEVEL_NUM] = {
	24,  46,  70,  93, 116, 141, 164, 199, 246, 304,
	351, 398, 445, 504, 551, 598, 656, 703, 750, 797,
	856, 903, 949, 996, 1055, 1102
};

static const unsigned char sy7806_torch_level[ZHAOYUN_LEVEL_NUM] = {
	0x08, 0x10, 0x18, 0x21, 0x29, 0x31, 0x3A, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned char sy7806_flash_level[ZHAOYUN_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x50, 0x54, 0x59, 0x5D
};

static const int ocp81373_current[ZHAOYUN_LEVEL_NUM] = {
	24,  46,  70,  93, 116, 141, 164, 199, 246, 304,
	351, 398, 445, 504, 551, 598, 656, 703, 750, 797,
	856, 903, 949, 996, 1055, 1102
};

static const unsigned char ocp81373_torch_level[ZHAOYUN_LEVEL_NUM] = {
	0x08, 0x10, 0x18, 0x21, 0x29, 0x31, 0x3A, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned char ocp81373_flash_level[ZHAOYUN_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x50, 0x54, 0x59, 0x5D
};

static const int AW36515_current[ZHAOYUN_LEVEL_NUM] = {
	24,  46,  70,  93, 116, 141, 164, 199, 246, 304,
	351, 398, 445, 504, 551, 598, 656, 703, 750, 797,
	856, 903, 949, 996, 1055, 1102
};

static const int AW36518_current[ZHAOYUN_LEVEL_NUM] = {
	24,  46,  70,  93, 116, 141, 164, 199, 246, 304,
	351, 398, 445, 504, 551, 598, 656, 703, 750, 797,
	856, 903, 949, 996, 1055, 1102
};

static const unsigned char AW36515_torch_level[ZHAOYUN_LEVEL_NUM] = {
	0x0C, 0x17, 0x23, 0x31, 0x3B, 0x47, 0x53, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned char AW36515_flash_level[ZHAOYUN_LEVEL_NUM] = {
	0x03, 0x05, 0x08, 0x0C, 0x0E, 0x12, 0x14, 0x19, 0x1F, 0x26,
	0x2C, 0x32, 0x38, 0x40, 0x46, 0x4C, 0x53, 0x59, 0x5F, 0x65,
	0x6D, 0x73, 0x79, 0x7F, 0x82, 0x86
};

static const unsigned char AW36518_torch_level[ZHAOYUN_LEVEL_NUM] = {
	0x0F, 0x1E, 0x2E, 0x3D, 0x4C, 0x5D, 0x6C, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned char AW36518_flash_level[ZHAOYUN_LEVEL_NUM] = {
	0x04, 0x08, 0x0B, 0x10, 0x13, 0x17, 0x1B, 0x21, 0x29, 0x33,
	0x3B, 0x43, 0x4B, 0x55, 0x5D, 0x65, 0x6F, 0x77, 0x7F, 0x87,
	0x91, 0x99, 0xA1, 0xA9, 0xB3, 0xBB
};

static const int KTD2687_current[ZHAOYUN_LEVEL_NUM] = {
	24,  46,  70,  93, 116, 141, 164, 199, 246, 304,
	351, 398, 445, 504, 551, 598, 656, 703, 750, 797,
	856, 903, 949, 996, 1055, 1102
};

static const unsigned char KTD2687_torch_level[ZHAOYUN_LEVEL_NUM] = {
	0x0F, 0x1E, 0x2F, 0x42, 0x4A, 0x5F, 0x6F, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned char KTD2687_flash_level[ZHAOYUN_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x08, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x4F, 0x51, 0x53, 0x54
};


static volatile unsigned char zhaoyun_reg_enable;
static volatile int zhaoyun_level_ch1 = -1;
static volatile int zhaoyun_level_ch2 = -1;

static int zhaoyun_is_torch(int level)
{
	if (level >= ZHAOYUN_LEVEL_TORCH)
		return -1;

	return 0;
}

static int zhaoyun_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= ZHAOYUN_LEVEL_NUM)
		level = ZHAOYUN_LEVEL_NUM - 1;

	return level;
}

/* flashlight enable function */
static int zhaoyun_enable_ch1(void)
{
	unsigned char reg, val;

	reg = ZHAOYUN_REG_ENABLE;
	if (!zhaoyun_is_torch(zhaoyun_level_ch1)) {
		/* torch mode */
		if (is_aw36518 == 1){
			zhaoyun_reg_enable |= ZHAOYUNLITE_ENABLE_LED1_TORCH;
		} else {
			zhaoyun_reg_enable |= ZHAOYUN_ENABLE_LED1_TORCH;
		}
	} else {
		/* flash mode */
		if (is_aw36518 == 1){
			zhaoyun_reg_enable |= ZHAOYUNLITE_ENABLE_LED1_FLASH;
		} else {
			zhaoyun_reg_enable |= ZHAOYUN_ENABLE_LED1_FLASH;
		}
	}
	val = zhaoyun_reg_enable;

	return zhaoyun_write_reg(zhaoyun_i2c_client, reg, val);
}

static int zhaoyun_enable_ch2(void)
{
	unsigned char reg, val;

	reg = ZHAOYUN_REG_ENABLE;
	if (!zhaoyun_is_torch(zhaoyun_level_ch2)) {
		/* torch mode */
		zhaoyun_reg_enable |= ZHAOYUN_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		zhaoyun_reg_enable |= ZHAOYUN_ENABLE_LED2_FLASH;
	}
	val = zhaoyun_reg_enable;

	return zhaoyun_write_reg(zhaoyun_i2c_client, reg, val);
}

int zhaoyun_temperature_enable(bool On)
{
	if(On) {
		zhaoyun_pinctrl_set(ZHAOYUN_PINCTRL_PIN_TEMP, ZHAOYUN_PINCTRL_PINSTATE_HIGH);
	} else {
		zhaoyun_pinctrl_set(ZHAOYUN_PINCTRL_PIN_TEMP, ZHAOYUN_PINCTRL_PINSTATE_LOW);
	}

	return 0;
}


static int zhaoyun_enable(int channel)
{
	if (channel == ZHAOYUN_CHANNEL_CH1)
		zhaoyun_enable_ch1();
	else if (channel == ZHAOYUN_CHANNEL_CH2)
		zhaoyun_enable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}
	return 0;
}

/* flashlight disable function */
static int zhaoyun_disable_ch1(void)
{
	unsigned char reg, val;

	reg = ZHAOYUN_REG_ENABLE;
	if (zhaoyun_reg_enable & ZHAOYUN_MASK_ENABLE_LED2) {
		/* if LED 2 is enable, disable LED 1 */
		zhaoyun_reg_enable &= (~ZHAOYUN_ENABLE_LED1);
	} else {
		/* if LED 2 is enable, disable LED 1 and clear mode */
		zhaoyun_reg_enable &= (~ZHAOYUN_ENABLE_LED1_FLASH);
	}
	val = zhaoyun_reg_enable;
	return zhaoyun_write_reg(zhaoyun_i2c_client, reg, val);
}

static int zhaoyun_disable_ch2(void)
{
	unsigned char reg, val;

	reg = ZHAOYUN_REG_ENABLE;
	if (zhaoyun_reg_enable & ZHAOYUN_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		zhaoyun_reg_enable &= (~ZHAOYUN_ENABLE_LED2);
	} else {
		/* if LED 1 is enable, disable LED 2 and clear mode */
		zhaoyun_reg_enable &= (~ZHAOYUN_ENABLE_LED2_FLASH);
	}
	val = zhaoyun_reg_enable;

	return zhaoyun_write_reg(zhaoyun_i2c_client, reg, val);
}

static int zhaoyun_disable(int channel)
{
	if (channel == ZHAOYUN_CHANNEL_CH1) {
		zhaoyun_disable_ch1();
		pr_info("ZHAOYUN_CHANNEL_CH1\n");
	} else if (channel == ZHAOYUN_CHANNEL_CH2) {
		zhaoyun_disable_ch2();
		pr_info("ZHAOYUN_CHANNEL_CH2\n");
	} else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* set flashlight level */
static int zhaoyun_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val;

	level = zhaoyun_verify_level(level);

	/* set torch brightness level */
	reg = ZHAOYUN_REG_TORCH_LEVEL_LED1;
	val = zhaoyun_torch_level[level];
	ret = zhaoyun_write_reg(zhaoyun_i2c_client, reg, val);

	zhaoyun_level_ch1 = level;

	/* set flash brightness level */
	reg = ZHAOYUN_REG_FLASH_LEVEL_LED1;
	val = zhaoyun_flash_level[level];
	ret = zhaoyun_write_reg(zhaoyun_i2c_client, reg, val);

	return ret;
}

int zhaoyun_set_level_ch2(int level)
{
	int ret;
	unsigned char reg, val;

	level = zhaoyun_verify_level(level);

	/* set torch brightness level */
	reg = ZHAOYUN_REG_TORCH_LEVEL_LED2;
	val = zhaoyun_torch_level[level];
	ret = zhaoyun_write_reg(zhaoyun_i2c_client, reg, val);

	zhaoyun_level_ch2 = level;

	/* set flash brightness level */
	reg = ZHAOYUN_REG_FLASH_LEVEL_LED2;
	val = zhaoyun_flash_level[level];
	ret = zhaoyun_write_reg(zhaoyun_i2c_client, reg, val);

	return ret;
}

static int zhaoyun_set_level(int channel, int level)
{
	if (channel == ZHAOYUN_CHANNEL_CH1)
		zhaoyun_set_level_ch1(level);
	else if (channel == ZHAOYUN_CHANNEL_CH2)
		zhaoyun_set_level_ch2(level);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}
/* flashlight init */
int zhaoyun_init(void)
{
	int ret;
	unsigned char reg, val, reg_val;
	int chip_id;

	zhaoyun_pinctrl_set(ZHAOYUN_PINCTRL_PIN_HWEN, ZHAOYUN_PINCTRL_PINSTATE_HIGH);
	chip_id = zhaoyun_read_reg(zhaoyun_i2c_client, 0x0c);
	msleep(2);
	pr_info("flashlight chip id: reg:0x0c, chip_id 0x%x",chip_id);
	if ( chip_id == AW36515_SM ) {
		reg_val = zhaoyun_read_reg(zhaoyun_i2c_client, ZHAOYUN_AW36515_REG_BOOST_CONFIG);
		reg_val |= ZHAOYUN_AW36515_SOFT_RESET_ENABLE;
		pr_info("flashlight chip id: reg:0x0c, data:0x%x;boost confgiuration: reg:0x07, reg_val: 0x%x", chip_id, reg_val);
		ret = zhaoyun_write_reg(zhaoyun_i2c_client, ZHAOYUN_AW36515_REG_BOOST_CONFIG, reg_val);
		msleep(2);
	}
	/* clear enable register */
	reg = ZHAOYUN_REG_ENABLE;
	val = ZHAOYUN_DISABLE;
	ret = zhaoyun_write_reg(zhaoyun_i2c_client, reg, val);

	zhaoyun_reg_enable = val;

	/* set torch current ramp time and flash timeout */
	reg = ZHAOYUN_REG_TIMING_CONF;
	val = ZHAOYUN_TORCH_RAMP_TIME | ZHAOYUN_FLASH_TIMEOUT;
	ret = zhaoyun_write_reg(zhaoyun_i2c_client, reg, val);

	return ret;
}

/* flashlight uninit */
int zhaoyun_uninit(void)
{
	zhaoyun_disable(ZHAOYUN_CHANNEL_CH1);
	zhaoyun_disable(ZHAOYUN_CHANNEL_CH2);
	zhaoyun_pinctrl_set(ZHAOYUN_PINCTRL_PIN_HWEN, ZHAOYUN_PINCTRL_PINSTATE_LOW);

	return 0;
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer zhaoyun_timer_ch1;
static struct hrtimer zhaoyun_timer_ch2;
static unsigned int zhaoyun_timeout_ms[ZHAOYUN_CHANNEL_NUM];

static void zhaoyun_work_disable_ch1(struct work_struct *data)
{
	pr_info("ht work queue callback\n");
	zhaoyun_disable_ch1();
}

static void zhaoyun_work_disable_ch2(struct work_struct *data)
{
	pr_info("lt work queue callback\n");
	zhaoyun_disable_ch2();
}

static enum hrtimer_restart zhaoyun_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&zhaoyun_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart zhaoyun_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&zhaoyun_work_ch2);
	return HRTIMER_NORESTART;
}

int zhaoyun_timer_start(int channel, ktime_t ktime)
{
	if (channel == ZHAOYUN_CHANNEL_CH1)
		hrtimer_start(&zhaoyun_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == ZHAOYUN_CHANNEL_CH2)
		hrtimer_start(&zhaoyun_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

int zhaoyun_timer_cancel(int channel)
{
	if (channel == ZHAOYUN_CHANNEL_CH1)
		hrtimer_cancel(&zhaoyun_timer_ch1);
	else if (channel == ZHAOYUN_CHANNEL_CH2)
		hrtimer_cancel(&zhaoyun_timer_ch2);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int zhaoyun_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= ZHAOYUN_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}
	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_info("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		zhaoyun_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		zhaoyun_pinctrl_set(ZHAOYUN_PINCTRL_PIN_HWEN, ZHAOYUN_PINCTRL_PINSTATE_HIGH);
		zhaoyun_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		zhaoyun_pinctrl_set(ZHAOYUN_PINCTRL_PIN_HWEN, ZHAOYUN_PINCTRL_PINSTATE_HIGH);
		if (fl_arg->arg == 1) {
			if (zhaoyun_timeout_ms[channel]) {
				ktime = ktime_set(zhaoyun_timeout_ms[channel] / 1000,
						(zhaoyun_timeout_ms[channel] % 1000) * 1000000);
				zhaoyun_timer_start(channel, ktime);
			}
			zhaoyun_enable(channel);
		} else {
			zhaoyun_disable(channel);
			zhaoyun_timer_cancel(channel);
			zhaoyun_pinctrl_set(ZHAOYUN_PINCTRL_PIN_HWEN, ZHAOYUN_PINCTRL_PINSTATE_LOW);
		}
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_info("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = ZHAOYUN_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_info("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = ZHAOYUN_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = zhaoyun_verify_level(fl_arg->arg);
		pr_info("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = zhaoyun_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_info("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = ZHAOYUN_HW_TIMEOUT;
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int zhaoyun_open(void)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int zhaoyun_release(void)
{
	/* uninit chip and clear usage count */
/*
	mutex_lock(&zhaoyun_mutex);
	use_count--;
	if (!use_count)
		zhaoyun_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&zhaoyun_mutex);

	pr_info("Release: %d\n", use_count);
*/
	return 0;
}

static int zhaoyun_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&zhaoyun_mutex);
	if (set) {
		if (!use_count)
			ret = zhaoyun_init();
		use_count++;
		pr_info("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = zhaoyun_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_info("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&zhaoyun_mutex);

	return ret;
}

static ssize_t zhaoyun_strobe_store(struct flashlight_arg arg)
{
	zhaoyun_set_driver(1);
	zhaoyun_set_level(arg.channel, arg.level);
	zhaoyun_timeout_ms[arg.channel] = 0;
	zhaoyun_enable(arg.channel);
	msleep(arg.dur);
	zhaoyun_disable(arg.channel);
	//zhaoyun_release(NULL);
	zhaoyun_set_driver(0);
	return 0;
}

static struct flashlight_operations zhaoyun_ops = {
	zhaoyun_open,
	zhaoyun_release,
	zhaoyun_ioctl,
	zhaoyun_strobe_store,
	zhaoyun_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int zhaoyun_chip_init(struct zhaoyun_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * zhaoyun_init();
	 */

	return 0;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ZHAOYUN Debug file
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t zhaoyun_get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char reg_val;
	unsigned char i;
	ssize_t len = 0;
	for(i=0;i<0x0E;i++)
	{
		reg_val = zhaoyun_read_reg(zhaoyun_i2c_client,i);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg%2X = 0x%2X \n, ", i,reg_val);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\r\n");
	return len;
}

static ssize_t zhaoyun_set_reg(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned int databuf[2];
	if(2 == sscanf(buf,"%x %x",&databuf[0], &databuf[1]))
	{
		//i2c_write_reg(databuf[0],databuf[1]);
		zhaoyun_write_reg(zhaoyun_i2c_client,databuf[0],databuf[1]);
	}
	return len;
}

static ssize_t zhaoyun_get_hwen(struct device* cd,struct device_attribute *attr, char* buf)
{
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "//zhaoyun_hwen_on(void)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 1 > hwen\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "//zhaoyun_hwen_off(void)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 0 > hwen\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");

	return len;
}

static ssize_t zhaoyun_set_hwen(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned char databuf[16];

	sscanf(buf,"%c",&databuf[0]);
#if 1
	if(databuf[0] == 0) {			// OFF
		//zhaoyun_hwen_low();
	} else {				// ON
		//zhaoyun_hwen_high();
	}
#endif
	return len;
}

static int zhaoyun_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_reg);
	err = device_create_file(dev, &dev_attr_hwen);

	return err;
}

static int zhaoyun_parse_dt(struct device *dev,
		struct zhaoyun_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				ZHAOYUN_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int zhaoyun_chip_id(void)
{
	int chip_id;
	int reg00_id = -1;
	zhaoyun_pinctrl_set(ZHAOYUN_PINCTRL_PIN_HWEN, ZHAOYUN_PINCTRL_PINSTATE_HIGH);
	msleep(1);
	chip_id = zhaoyun_read_reg(zhaoyun_i2c_client, 0x0c);
	pr_info("flashlight chip id: reg:0x0c, data:0x%x", chip_id);
	if (chip_id == AW36515_SM) {
		reg00_id = zhaoyun_read_reg(zhaoyun_i2c_client, 0x00);
		pr_info("flashlight chip id: reg:0x00, data:0x%x", reg00_id);
		if (reg00_id == 0x30) {
			chip_id = AW36515_SM;
			pr_info("flashlight reg00_id = 0x%x, set chip_id to AW36515_SM", reg00_id);
		} else {
			chip_id = KTD2687_SM;
			pr_info("flashlight reg00_id = 0x%x, set chip_id to KTD2687_SM", reg00_id);
		}
	}
	zhaoyun_pinctrl_set(ZHAOYUN_PINCTRL_PIN_HWEN, ZHAOYUN_PINCTRL_PINSTATE_LOW);
	if (chip_id == AW3643_SM) {
		pr_info(" the device's flashlight driver IC is AW3643\n");
		return USE_AW3643_IC;
	} else if (chip_id == SY7806_SM){
		pr_info(" the device's flashlight driver IC is SY7806\n");
		return USE_SY7806_IC;
	} else if (chip_id == OCP81373_SM){
		pr_info(" the device's flashlight driver IC is OCP81373\n");
		return USE_OCP81373_IC;
	} else if (chip_id == AW36515_SM){
		pr_info(" the device's flashlight driver IC is AW36515\n");
		return USE_AW36515_IC;
	} else if (chip_id == AW36518_SM){
		pr_info(" the device's flashlight driver IC is AW36518\n");
		is_aw36518 = 1;
		return USE_AW36518_IC;
	} else if (chip_id == KTD2687_SM){
		pr_info(" the device's flashlight driver IC is KTD2687\n");
		return USE_KTD2687_IC;
	} else {
		pr_err(" the device's flashlight driver IC is not used in our project!\n");
		return USE_NOT_PRO;
	}
}

static int zhaoyun_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct zhaoyun_chip_data *chip;
	struct zhaoyun_platform_data *pdata = client->dev.platform_data;
	int err;
	int i;
	int chip_id;

	pr_info("zhaoyun_i2c_probe Probe start.\n");

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct zhaoyun_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pr_err("Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct zhaoyun_platform_data), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_free;
		}
		client->dev.platform_data = pdata;
		err = zhaoyun_parse_dt(&client->dev, pdata);
		if (err)
			goto err_free;
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	zhaoyun_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&zhaoyun_work_ch1, zhaoyun_work_disable_ch1);
	INIT_WORK(&zhaoyun_work_ch2, zhaoyun_work_disable_ch2);

	/* init timer */
	hrtimer_init(&zhaoyun_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	zhaoyun_timer_ch1.function = zhaoyun_timer_func_ch1;
	hrtimer_init(&zhaoyun_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	zhaoyun_timer_ch2.function = zhaoyun_timer_func_ch2;
	zhaoyun_timeout_ms[ZHAOYUN_CHANNEL_CH1] = 100;
	zhaoyun_timeout_ms[ZHAOYUN_CHANNEL_CH2] = 100;

	/* init chip hw */
	zhaoyun_chip_init(chip);
	chip_id = zhaoyun_chip_id();
	if(chip_id == USE_AW3643_IC){
		zhaoyun_current = aw3643_current;
		zhaoyun_torch_level = aw3643_torch_level;
		zhaoyun_flash_level = aw3643_flash_level;
	} else if (chip_id == USE_SY7806_IC){
		zhaoyun_current = sy7806_current;
		zhaoyun_torch_level = sy7806_torch_level;
		zhaoyun_flash_level = sy7806_flash_level;
	} else if (chip_id == USE_OCP81373_IC){
		zhaoyun_current = ocp81373_current;
		zhaoyun_torch_level = ocp81373_torch_level;
		zhaoyun_flash_level = ocp81373_flash_level;
	} else if (chip_id == USE_AW36515_IC){
		zhaoyun_current = AW36515_current;
		zhaoyun_torch_level = AW36515_torch_level;
		zhaoyun_flash_level = AW36515_flash_level;
	} else if (chip_id == USE_AW36518_IC){
		zhaoyun_current = AW36518_current;
		zhaoyun_torch_level = AW36518_torch_level;
		zhaoyun_flash_level = AW36518_flash_level;
	} else if (chip_id == USE_KTD2687_IC){
		zhaoyun_current = KTD2687_current;
		zhaoyun_torch_level = KTD2687_torch_level;
		zhaoyun_flash_level = KTD2687_flash_level;
	} else if (chip_id == USE_NOT_PRO){
		zhaoyun_current = aw3643_current;
		zhaoyun_torch_level = aw3643_torch_level;
		zhaoyun_flash_level = aw3643_flash_level;
	}

	/* register flashlight operations */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&zhaoyun_ops)) {
                pr_err("Failed to register flashlight device.\n");
				err = -EFAULT;
				goto err_free;
			}
	} else {
		if (flashlight_dev_register(ZHAOYUN_NAME, &zhaoyun_ops)) {
			pr_err("Failed to register flashlight device.\n");
			err = -EFAULT;
			goto err_free;
		}
	}

    zhaoyun_create_sysfs(client);

	pr_info("Probe done.\n");

	return 0;

err_free:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int zhaoyun_i2c_remove(struct i2c_client *client)
{
	struct zhaoyun_platform_data *pdata = dev_get_platdata(&client->dev);
	struct zhaoyun_chip_data *chip = i2c_get_clientdata(client);
	int i;

	pr_info("Remove start.\n");

	client->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(ZHAOYUN_NAME);
	/* flush work queue */
	flush_work(&zhaoyun_work_ch1);
	flush_work(&zhaoyun_work_ch2);

	/* unregister flashlight operations */
	flashlight_dev_unregister(ZHAOYUN_NAME);

	/* free resource */
	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);

	pr_info("Remove done.\n");

	return 0;
}

static const struct i2c_device_id zhaoyun_i2c_id[] = {
	{ZHAOYUN_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id zhaoyun_i2c_of_match[] = {
	{.compatible = ZHAOYUN_DTNAME_I2C},
	{},
};
MODULE_DEVICE_TABLE(of, zhaoyun_i2c_of_match);
#endif

static struct i2c_driver zhaoyun_i2c_driver = {
	.driver = {
		   .name = ZHAOYUN_NAME,
#ifdef CONFIG_OF
		   .of_match_table = zhaoyun_i2c_of_match,
#endif
		   },
	.probe = zhaoyun_i2c_probe,
	.remove = zhaoyun_i2c_remove,
	.id_table = zhaoyun_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int zhaoyun_probe(struct platform_device *dev)
{
	pr_info("Probe start %s.\n", ZHAOYUN_DTNAME_I2C);

	/* init pinctrl */
	if (zhaoyun_pinctrl_init(dev)) {
		pr_err("Failed to init pinctrl.\n");
		return -1;
	}

	if (i2c_add_driver(&zhaoyun_i2c_driver)) {
		pr_err("Failed to add i2c driver.\n");
		return -1;
	}

	pr_info("Probe done.\n");

	return 0;
}

static int zhaoyun_remove(struct platform_device *dev)
{
	pr_info("Remove start.\n");

	i2c_del_driver(&zhaoyun_i2c_driver);

	pr_info("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id zhaoyun_of_match[] = {
	{.compatible = ZHAOYUN_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, zhaoyun_of_match);
#else
static struct platform_device zhaoyun_platform_device[] = {
	{
		.name = ZHAOYUN_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, zhaoyun_platform_device);
#endif

static struct platform_driver zhaoyun_platform_driver = {
	.probe = zhaoyun_probe,
	.remove = zhaoyun_remove,
	.driver = {
		.name = ZHAOYUN_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = zhaoyun_of_match,
#endif
	},
};

static int __init flashlight_zhaoyun_init(void)
{
	int ret;

	pr_info("flashlight_zhaoyun-Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&zhaoyun_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&zhaoyun_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_info("flashlight_zhaoyun Init done.\n");

	return 0;
}

static void __exit flashlight_zhaoyun_exit(void)
{
	pr_info("flashlight_zhaoyun-Exit start.\n");

	platform_driver_unregister(&zhaoyun_platform_driver);

	pr_info("flashlight_zhaoyun Exit done.\n");
}


module_init(flashlight_zhaoyun_init);
module_exit(flashlight_zhaoyun_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joseph <zhangzetao@awinic.com.cn>");
MODULE_DESCRIPTION("AW Flashlight ZHAOYUN Driver");

