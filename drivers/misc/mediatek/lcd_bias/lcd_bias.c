/********************************************
 ** Copyright (C) 2018 OPLUS Mobile Comm Corp. Ltd.
 ** ODM_HQ_EDIT
 ** File: lcd_bias.c
 ** Description: Source file for LCD bias
 **          To Control LCD bias voltage
 ** Version:1.1
 ** Date : 2019/09/17
 ** Author: Liyan@ODM_HQ.Multimedia.LCD
 ** ---------------- Revision History: --------------------------
 ** <version>    <date>          < author >              <desc>
 **  1.0           2018/10/03   Liyan@ODM_HQ   Source file for LCD bias
 **  1.1           2019/09/17   Liyan@ODM_HQ   Optimize power on sequence
 ********************************************/

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/timer.h>
#include <string.h>
#else
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#endif

#ifndef CONFIG_EXT_LCD_BIAS_ODM_HQ
#include "../lcm/inc/lcm_pmic.h"
#endif

#include "lcd_bias.h"

/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/
#ifdef BUILD_LK
static struct mt_i2c_t lcd_bias_i2c;
#else
static struct i2c_client *lcd_bias_i2c_client;
static struct pinctrl *lcd_bias_pctrl; /* static pinctrl instance */

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int lcd_bias_dts_probe(struct platform_device *pdev);
static int lcd_bias_dts_remove(struct platform_device *pdev);
static int lcd_bias_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lcd_bias_i2c_remove(struct i2c_client *client);
#endif

/*****************************************************************************
 * Extern Area
 *****************************************************************************/
#ifdef CONFIG_EXT_LCD_BIAS_ODM_HQ
static void ext_lcd_bias_write_byte(unsigned char addr, unsigned char value)
{
	int ret = 0;
	unsigned char write_data[2] = {0};

	write_data[0] = addr;
	write_data[1] = value;

#ifdef BUILD_LK
	lcd_bias_i2c.id = LCD_BIAS_I2C_BUSNUM;
	lcd_bias_i2c.addr = LCD_BIAS_I2C_ADDR;
	lcd_bias_i2c.mode = LCD_BIAS_ST_MODE;
	lcd_bias_i2c.speed = LCD_BIAS_MAX_ST_MODE_SPEED;
	ret = i2c_write(&lcd_bias_i2c, write_data, 2);
#else
	if (NULL == lcd_bias_i2c_client) {
		LCD_BIAS_PRINT("[LCD][BIAS] lcd_bias_i2c_client is null!!\n");
		return;
	}
	ret = i2c_master_send(lcd_bias_i2c_client, write_data, 2);
#endif

	if (ret < 0)
		LCD_BIAS_PRINT("[LCD][BIAS] i2c write data fail!! ret=%d\n", ret);
	else
		LCD_BIAS_PRINT("[LCD][BIAS] i2c write data success!!\n");
}

static void ext_lcd_bias_set_vspn(unsigned int en, unsigned int seq, unsigned int value)
{
	unsigned char level;

	level = (value - 4000) / 100;  /* eg.  5.0V= 4.0V + Hex 0x0A (Bin 0 1010) * 100mV */

#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);

	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);

	if (seq == FIRST_VSP_AFTER_VSN) {
		if (en) {
			mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
			mdelay(1);
			ext_lcd_bias_write_byte(LCD_BIAS_VPOS_ADDR, level);
			ext_lcd_bias_write_byte(LCD_BIAS_VNEG_ADDR, level);
			ext_lcd_bias_write_byte(LCD_BIAS_APPS_MTP_ADDR, 128);
			mdelay(5);
			mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
		} else {
			mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
			mdelay(5);
			mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
		}
	} else if (seq == FIRST_VSN_AFTER_VSP) {
		if (en) {
			mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
			mdelay(1);
			ext_lcd_bias_write_byte(LCD_BIAS_VNEG_ADDR, level);
			ext_lcd_bias_write_byte(LCD_BIAS_VPOS_ADDR, level);
			mdelay(5);
			mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
		} else {
			mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
			mdelay(5);
			mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
		}
	}

#else
	if (seq == FIRST_VSP_AFTER_VSN) {
		if (en) {
			lcd_bias_gpio_select_state(LCD_BIAS_GPIO_STATE_ENP1);
			mdelay(1);
			ext_lcd_bias_write_byte(LCD_BIAS_VPOS_ADDR, level);
			ext_lcd_bias_write_byte(LCD_BIAS_VNEG_ADDR, level);
			mdelay(1);
			lcd_bias_gpio_select_state(LCD_BIAS_GPIO_STATE_ENN1);
		} else {
			lcd_bias_gpio_select_state(LCD_BIAS_GPIO_STATE_ENP0);
			mdelay(5);
			lcd_bias_gpio_select_state(LCD_BIAS_GPIO_STATE_ENN0);
		}
	} else if (seq == FIRST_VSN_AFTER_VSP) {
		if (en) {
			lcd_bias_gpio_select_state(LCD_BIAS_GPIO_STATE_ENN1);
			mdelay(1);
			ext_lcd_bias_write_byte(LCD_BIAS_VNEG_ADDR, level);
			ext_lcd_bias_write_byte(LCD_BIAS_VPOS_ADDR, level);
			mdelay(1);
			lcd_bias_gpio_select_state(LCD_BIAS_GPIO_STATE_ENP1);
		} else {
			lcd_bias_gpio_select_state(LCD_BIAS_GPIO_STATE_ENN0);
			mdelay(5);
			lcd_bias_gpio_select_state(LCD_BIAS_GPIO_STATE_ENP0);
		}
	}

#endif
}

#else

static void pmi_lcd_bias_set_vspn(unsigned int en, unsigned int seq, unsigned int value)
{
	pr_err("do nothing!!\n");
}
#endif

void lcd_bias_set_vspn(unsigned int en, unsigned int seq, unsigned int value)
{
	if ((value <= 4000) || (value > 6200)) {
		LCD_BIAS_PRINT("[LCD][BIAS] unreasonable voltage value:%d\n", value);
		return;
	}

#ifdef CONFIG_EXT_LCD_BIAS_ODM_HQ
	ext_lcd_bias_set_vspn(en, seq, value);
#else
	pmi_lcd_bias_set_vspn(en, seq, value);
#endif
}

#ifndef BUILD_LK
/*****************************************************************************
 * Data Structure
 *****************************************************************************/
static const char *lcd_bias_state_name[LCD_BIAS_GPIO_STATE_MAX] = {
	"lcd_bias_gpio_enp0",
	"lcd_bias_gpio_enp1",
	"lcd_bias_gpio_enn0",
	"lcd_bias_gpio_enn1"
};/* DTS state mapping name */

static const struct of_device_id gpio_of_match[] = {
	{ .compatible = "mediatek,gpio_lcd_bias", },
	{},
};

static const struct of_device_id i2c_of_match[] = {
	{ .compatible = "mediatek,i2c_lcd_bias", },
	{},
};


static const struct i2c_device_id lcd_bias_i2c_id[] = {
	{"Lcd_Bias_I2C", 0},
	{},
};

static struct platform_driver lcd_bias_platform_driver = {
	.probe = lcd_bias_dts_probe,
	.remove = lcd_bias_dts_remove,
	.driver = {
		.name = "Lcd_Bias_DTS",
		.of_match_table = gpio_of_match,
	},
};

static struct i2c_driver lcd_bias_i2c_driver = {
	/************************************************************
Attention:
Althouh i2c_bus do not use .id_table to match, but it must be defined,
otherwise the probe function will not be executed!
	 ************************************************************/
	.id_table = lcd_bias_i2c_id,
	.probe = lcd_bias_i2c_probe,
	.remove = lcd_bias_i2c_remove,
	.driver = {
		.name = "Lcd_Bias_I2C",
		.of_match_table = i2c_of_match,
	},
};

/*****************************************************************************
 * Function
 *****************************************************************************/
static long lcd_bias_set_state(const char *name)
{
	int ret = 0;
	struct pinctrl_state *pState = 0;

	BUG_ON(!lcd_bias_pctrl);

	pState = pinctrl_lookup_state(lcd_bias_pctrl, name);
	if (IS_ERR(pState)) {
		pr_err("set state '%s' failed\n", name);
		ret = PTR_ERR(pState);
		goto exit;
	}

	/* select state! */
	pinctrl_select_state(lcd_bias_pctrl, pState);

exit:
	return ret; /* Good! */
}

void lcd_bias_gpio_select_state(LCD_BIAS_GPIO_STATE s)
{
	BUG_ON(!((unsigned int)(s) < (unsigned int)(LCD_BIAS_GPIO_STATE_MAX)));
	lcd_bias_set_state(lcd_bias_state_name[s]);
}

static long lcd_bias_dts_init(struct platform_device *pdev)
{
	int ret = 0;
	struct pinctrl *pctrl;
	LCD_BIAS_PRINT("[LCD][BIAS]%s:enter\n", __func__);
	/* retrieve */
	pctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pctrl)) {
		dev_err(&pdev->dev, "Cannot find disp pinctrl!");
		ret = PTR_ERR(pctrl);
		goto exit;
	}

	lcd_bias_pctrl = pctrl;
	LCD_BIAS_PRINT("[LCD][BIAS]%s:exit\n", __func__);

exit:
	return ret;
}

static int lcd_bias_dts_probe(struct platform_device *pdev)
{
	int ret = 0;
	LCD_BIAS_PRINT("%s:enter\n", __func__);
	ret = lcd_bias_dts_init(pdev);
	if (ret) {
		LCD_BIAS_PRINT("[LCD][BIAS] lcd_bias_dts_probe failed\n");
		return ret;
	}

	LCD_BIAS_PRINT("[LCD][BIAS] lcd_bias_dts_probe success\n");

	return 0;
}

static int lcd_bias_dts_remove(struct platform_device *pdev)
{
	platform_driver_unregister(&lcd_bias_platform_driver);

	return 0;
}

static int lcd_bias_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	if (NULL == client) {
		LCD_BIAS_PRINT("[LCD][BIAS] i2c_client is NULL\n");
		return -1;
	}

	lcd_bias_i2c_client = client;
	LCD_BIAS_PRINT("[LCD][BIAS] lcd_bias_i2c_probe success addr = 0x%x\n", client->addr);

	return 0;
}

static int lcd_bias_i2c_remove(struct i2c_client *client)
{
	lcd_bias_i2c_client = NULL;
	i2c_unregister_device(client);

	return 0;
}

static int __init lcd_bias_init(void)
{
	LCD_BIAS_PRINT("%s:enter\n", __func__);

	if (i2c_add_driver(&lcd_bias_i2c_driver)) {
		LCD_BIAS_PRINT("[LCD][BIAS] Failed to register lcd_bias_i2c_driver!\n");
		return -1;
	}

	if (platform_driver_register(&lcd_bias_platform_driver)) {
		LCD_BIAS_PRINT("[LCD][BIAS] Failed to register lcd_bias_platform_driver!\n");
		i2c_del_driver(&lcd_bias_i2c_driver);
		return -1;
	}

	return 0;
}

static void __exit lcd_bias_exit(void)
{
	platform_driver_unregister(&lcd_bias_platform_driver);
	i2c_del_driver(&lcd_bias_i2c_driver);
}

module_init(lcd_bias_init);
module_exit(lcd_bias_exit);

MODULE_AUTHOR("Oly Peng <penghoubing@huaqin.com>");
MODULE_DESCRIPTION("MTK LCD BIAS I2C Driver");
MODULE_LICENSE("GPL");
#endif
