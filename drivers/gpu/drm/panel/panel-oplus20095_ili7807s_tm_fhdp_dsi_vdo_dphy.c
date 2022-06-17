/*
 * Copyright (c) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/gpio/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <linux/of_graph.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <soc/oplus/device_info.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_panel_ext.h"
#include "../mediatek/mtk_log.h"
#include "../mediatek/mtk_drm_graphics_base.h"
#endif
/* enable this to check panel self -bist pattern */
/* #define PANEL_BIST_PATTERN */
/****************TPS65132***********/
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
//#include "lcm_i2c.h"
#include <mt-plat/mtk_boot_common.h>
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif

#define AVDD_REG 0x00
#define AVDD_REG 0x01
#define MAX_NORMAL_BRIGHTNESS   3067

/* i2c control start */
#define LCM_I2C_ID_NAME "I2C_LCD_BIAS"
static struct i2c_client *_lcm_i2c_client;
/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int _lcm_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
static int _lcm_i2c_remove(struct i2c_client *client);
extern int tp_gesture_enable_flag(void);
extern void lcd_queue_load_tp_fw(void);
extern unsigned int __attribute((weak)) is_project(int project) { return 0; }
extern void tp_ftm_extra(void);
extern unsigned long esd_flag;
static int esd_brightness;
extern unsigned long oplus_max_normal_brightness;
/*****************************************************************************
 * Data Structure
 *****************************************************************************/
struct _lcm_i2c_dev {
	struct i2c_client *client;

};

static const struct of_device_id _lcm_i2c_of_match[] = {
	{ .compatible = "mediatek,I2C_LCD_BIAS", },
	{},
};

static const struct i2c_device_id _lcm_i2c_id[] = {
	{LCM_I2C_ID_NAME, 0},
	{}
};

static struct i2c_driver _lcm_i2c_driver = {
	.id_table = _lcm_i2c_id,
	.probe = _lcm_i2c_probe,
	.remove = _lcm_i2c_remove,
	/* .detect               = _lcm_i2c_detect, */
	.driver = {
		   .owner = THIS_MODULE,
		   .name = LCM_I2C_ID_NAME,
		   .of_match_table = _lcm_i2c_of_match,
		   },

};

/*****************************************************************************
 * Function
 *****************************************************************************/


static int _lcm_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	pr_debug("[LCM][I2C] NT: info==>name=%s addr=0x%x\n",
		client->name, client->addr);
	_lcm_i2c_client = client;
	return 0;
}


static int _lcm_i2c_remove(struct i2c_client *client)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	_lcm_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}


static int _lcm_i2c_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = _lcm_i2c_client;
	char write_data[2] = { 0 };

	if (client == NULL) {
		pr_debug("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		pr_info("[LCM][ERROR] _lcm_i2c write data fail !!\n");

	return ret;
}

/*
 * module load/unload record keeping
 */
static int __init _lcm_i2c_init(void)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	i2c_add_driver(&_lcm_i2c_driver);
	pr_debug("[LCM][I2C] %s success\n", __func__);
	return 0;
}

static void __exit _lcm_i2c_exit(void)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	i2c_del_driver(&_lcm_i2c_driver);
}

module_init(_lcm_i2c_init);
module_exit(_lcm_i2c_exit);
/***********************************/

struct tianma {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos;
	struct gpio_desc *bias_neg;
	struct gpio_desc *bias_en;
	bool prepared;
	bool enabled;

	int error;
	int info;
	bool is_normal_mode;
};

#define tm_dcs_write_seq(ctx, seq...)                                     \
	({                                                                     \
		const u8 d[] = {seq};                                          \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		tm_dcs_write(ctx, d, ARRAY_SIZE(d));                      \
	})

#define tm_dcs_write_seq_static(ctx, seq...)                              \
	({                                                                     \
		static const u8 d[] = {seq};                                   \
		tm_dcs_write(ctx, d, ARRAY_SIZE(d));                      \
	})

static inline struct tianma *panel_to_tm(struct drm_panel *panel)
{
	return container_of(panel, struct tianma, panel);
}

#ifdef PANEL_SUPPORT_READBACK
static int tm_dcs_read(struct tianma *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret, cmd);
		pr_notice("error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void tm_panel_get_data(struct tianma *ctx)
{
	u8 buffer[3] = {0};
	static int ret;
	pr_info("%s+\n", __func__);

	if (ret == 0) {
		ret = tm_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("%s  0x%08x\n", __func__,buffer[0] | (buffer[1] << 8));
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static void tm_dcs_write(struct tianma *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static void tm_panel_init(struct tianma *ctx)
{
	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x06);
	tm_dcs_write_seq_static(ctx,0x3E,0xE2);
	tm_dcs_write_seq_static(ctx,0x08,0x20);
	tm_dcs_write_seq_static(ctx,0x11,0x70);
	tm_dcs_write_seq_static(ctx,0x10,0x0D);

	tm_dcs_write_seq_static(ctx,0x48,0x0F); // 1bit ESD check
	tm_dcs_write_seq_static(ctx,0x4D,0x80); // 1bit ESD check
	tm_dcs_write_seq_static(ctx,0x4E,0x40); // 1bit ESD check
	tm_dcs_write_seq_static(ctx,0xC7,0x05); // 1bit ESD check

	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x05);
	tm_dcs_write_seq_static(ctx,0x7E,0x07);

	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x03);
	tm_dcs_write_seq_static(ctx,0x83,0x20);
	tm_dcs_write_seq_static(ctx,0x84,0x00); //CABC 12bit

	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x03);
	tm_dcs_write_seq_static(ctx,0x86,0x47);
	tm_dcs_write_seq_static(ctx,0x87,0x56);
	tm_dcs_write_seq_static(ctx,0xAF,0x18);

	tm_dcs_write_seq_static(ctx,0x8C,0xE2);
	tm_dcs_write_seq_static(ctx,0x8D,0xE3);
	tm_dcs_write_seq_static(ctx,0x8E,0xE5);
	tm_dcs_write_seq_static(ctx,0x8F,0xE6);
	tm_dcs_write_seq_static(ctx,0x90,0xE9);
	tm_dcs_write_seq_static(ctx,0x91,0xEB);
	tm_dcs_write_seq_static(ctx,0x92,0xEE);
	tm_dcs_write_seq_static(ctx,0x93,0xF0);
	tm_dcs_write_seq_static(ctx,0x94,0xF6);
	tm_dcs_write_seq_static(ctx,0x95,0xFA);

	tm_dcs_write_seq_static(ctx,0x96,0xBF);
	tm_dcs_write_seq_static(ctx,0x97,0xC4);
	tm_dcs_write_seq_static(ctx,0x98,0xC7);
	tm_dcs_write_seq_static(ctx,0x99,0xC9);
	tm_dcs_write_seq_static(ctx,0x9A,0xCF);
	tm_dcs_write_seq_static(ctx,0x9B,0xD4);
	tm_dcs_write_seq_static(ctx,0x9C,0xDA);
	tm_dcs_write_seq_static(ctx,0x9D,0xE2);
	tm_dcs_write_seq_static(ctx,0x9E,0xEB);
	tm_dcs_write_seq_static(ctx,0x9F,0xFA);

	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x00);  //Page0
	tm_dcs_write_seq_static(ctx,0x51,0x00,0x00);
	tm_dcs_write_seq_static(ctx,0x53,0x24);
	tm_dcs_write_seq_static(ctx,0x55,0x02);  //11.5%

	tm_dcs_write_seq_static(ctx,0x35,0x00);
	tm_dcs_write_seq_static(ctx,0x11);
	msleep(80);
	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x01);
	tm_dcs_write_seq_static(ctx,0x41,0x41);
	tm_dcs_write_seq_static(ctx,0x59,0x41);
	tm_dcs_write_seq_static(ctx,0x72,0x41);
	tm_dcs_write_seq_static(ctx,0x8A,0x41);
	tm_dcs_write_seq_static(ctx,0xD1,0x22);
	tm_dcs_write_seq_static(ctx,0xD5,0x26);
	tm_dcs_write_seq_static(ctx,0xD8,0x62);
	tm_dcs_write_seq_static(ctx,0xDC,0x02);
	tm_dcs_write_seq_static(ctx,0xE2,0x01);
	tm_dcs_write_seq_static(ctx,0xE5,0x0B);
	tm_dcs_write_seq_static(ctx,0xE7,0x74);
	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x05);
	tm_dcs_write_seq_static(ctx,0x7A,0x3D);
	tm_dcs_write_seq_static(ctx,0x56,0xFF);
	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x06);
	tm_dcs_write_seq_static(ctx,0x96,0x10);
	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x11);
	tm_dcs_write_seq_static(ctx,0x00,0x00);
	tm_dcs_write_seq_static(ctx,0x01,0x0A);
	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x02);
	tm_dcs_write_seq_static(ctx,0x40,0x0C);
	tm_dcs_write_seq_static(ctx,0x41,0x00);
	tm_dcs_write_seq_static(ctx,0x42,0x0B);
	tm_dcs_write_seq_static(ctx,0x43,0x31);
	tm_dcs_write_seq_static(ctx,0x36,0x00);
	tm_dcs_write_seq_static(ctx,0x75,0x00);
	tm_dcs_write_seq_static(ctx,0x46,0x42);
	tm_dcs_write_seq_static(ctx,0x47,0x03);
	tm_dcs_write_seq_static(ctx,0x01,0x55);
	tm_dcs_write_seq_static(ctx,0x6B,0x11);
	tm_dcs_write_seq_static(ctx,0x82,0x32);
	tm_dcs_write_seq_static(ctx,0x1B,0x00);
	tm_dcs_write_seq_static(ctx,0x06,0x6C);
	tm_dcs_write_seq_static(ctx,0x0E,0x18);
	tm_dcs_write_seq_static(ctx,0x0F,0x20);
	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x12);
	tm_dcs_write_seq_static(ctx,0x10,0x02);
	tm_dcs_write_seq_static(ctx,0x11,0x00);
	tm_dcs_write_seq_static(ctx,0x12,0x0A);
	tm_dcs_write_seq_static(ctx,0x13,0x1B);
	tm_dcs_write_seq_static(ctx,0x16,0x0B);
	tm_dcs_write_seq_static(ctx,0x17,0x00);
	tm_dcs_write_seq_static(ctx,0x1A,0x13);
	tm_dcs_write_seq_static(ctx,0x1B,0x22);
	tm_dcs_write_seq_static(ctx,0xC0,0x48);
	tm_dcs_write_seq_static(ctx,0xC2,0x18);
	tm_dcs_write_seq_static(ctx,0xC3,0x20);
	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x0B);
	tm_dcs_write_seq_static(ctx,0x94,0x88);
	tm_dcs_write_seq_static(ctx,0x95,0x20);
	tm_dcs_write_seq_static(ctx,0x96,0x06);
	tm_dcs_write_seq_static(ctx,0x97,0x06);
	tm_dcs_write_seq_static(ctx,0x98,0xCB);
	tm_dcs_write_seq_static(ctx,0x99,0xCB);
	tm_dcs_write_seq_static(ctx,0x9A,0x06);
	tm_dcs_write_seq_static(ctx,0x9B,0xC8);
	tm_dcs_write_seq_static(ctx,0x9C,0x05);
	tm_dcs_write_seq_static(ctx,0x9D,0x05);
	tm_dcs_write_seq_static(ctx,0x9E,0xA9);
	tm_dcs_write_seq_static(ctx,0x9F,0xA9);
	tm_dcs_write_seq_static(ctx,0xC0,0x85);
	tm_dcs_write_seq_static(ctx,0xC1,0x6B);
	tm_dcs_write_seq_static(ctx,0xC2,0x04);
	tm_dcs_write_seq_static(ctx,0xC3,0x04);
	tm_dcs_write_seq_static(ctx,0xC4,0x87);
	tm_dcs_write_seq_static(ctx,0xC5,0x87);
	tm_dcs_write_seq_static(ctx,0xD2,0x04);
	tm_dcs_write_seq_static(ctx,0xD3,0x85);
	tm_dcs_write_seq_static(ctx,0xD4,0x03);
	tm_dcs_write_seq_static(ctx,0xD5,0x03);
	tm_dcs_write_seq_static(ctx,0xD6,0x71);
	tm_dcs_write_seq_static(ctx,0xD7,0x71);
	tm_dcs_write_seq_static(ctx,0xAB,0xE0);
	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x0C);
	tm_dcs_write_seq_static(ctx,0x00,0x3F);
	tm_dcs_write_seq_static(ctx,0x01,0xCE);
	tm_dcs_write_seq_static(ctx,0x02,0x3F);
	tm_dcs_write_seq_static(ctx,0x03,0xD1);
	tm_dcs_write_seq_static(ctx,0x04,0x3F);
	tm_dcs_write_seq_static(ctx,0x05,0xD1);
	tm_dcs_write_seq_static(ctx,0x06,0x3F);
	tm_dcs_write_seq_static(ctx,0x07,0xE4);
	tm_dcs_write_seq_static(ctx,0x08,0x3F);
	tm_dcs_write_seq_static(ctx,0x09,0xE8);
	tm_dcs_write_seq_static(ctx,0x0A,0x3F);
	tm_dcs_write_seq_static(ctx,0x0B,0xD4);
	tm_dcs_write_seq_static(ctx,0x0C,0x3F);
	tm_dcs_write_seq_static(ctx,0x0D,0xD2);
	tm_dcs_write_seq_static(ctx,0x0E,0x3F);
	tm_dcs_write_seq_static(ctx,0x0F,0xDE);
	tm_dcs_write_seq_static(ctx,0x10,0x3F);
	tm_dcs_write_seq_static(ctx,0x11,0xE3);
	tm_dcs_write_seq_static(ctx,0x12,0x3F);
	tm_dcs_write_seq_static(ctx,0x13,0xCC);
	tm_dcs_write_seq_static(ctx,0x14,0x3F);
	tm_dcs_write_seq_static(ctx,0x15,0xCD);
	tm_dcs_write_seq_static(ctx,0x16,0x3F);
	tm_dcs_write_seq_static(ctx,0x17,0xCF);
	tm_dcs_write_seq_static(ctx,0x18,0x3F);
	tm_dcs_write_seq_static(ctx,0x19,0xD5);
	tm_dcs_write_seq_static(ctx,0x1A,0x3F);
	tm_dcs_write_seq_static(ctx,0x1B,0xE6);
	tm_dcs_write_seq_static(ctx,0x1C,0x3F);
	tm_dcs_write_seq_static(ctx,0x1D,0xCA);
	tm_dcs_write_seq_static(ctx,0x1E,0x3F);
	tm_dcs_write_seq_static(ctx,0x1F,0xD8);
	tm_dcs_write_seq_static(ctx,0x20,0x3F);
	tm_dcs_write_seq_static(ctx,0x21,0xCB);
	tm_dcs_write_seq_static(ctx,0x22,0x3F);
	tm_dcs_write_seq_static(ctx,0x23,0xDF);
	tm_dcs_write_seq_static(ctx,0x24,0x3F);
	tm_dcs_write_seq_static(ctx,0x25,0xE7);
	tm_dcs_write_seq_static(ctx,0x26,0x3F);
	tm_dcs_write_seq_static(ctx,0x27,0xE5);
	tm_dcs_write_seq_static(ctx,0x28,0x3F);
	tm_dcs_write_seq_static(ctx,0x29,0xC8);
	tm_dcs_write_seq_static(ctx,0x2A,0x3F);
	tm_dcs_write_seq_static(ctx,0x2B,0xE1);
	tm_dcs_write_seq_static(ctx,0x2C,0x3F);
	tm_dcs_write_seq_static(ctx,0x2D,0xE2);
	tm_dcs_write_seq_static(ctx,0x2E,0x3F);
	tm_dcs_write_seq_static(ctx,0x2F,0xD7);
	tm_dcs_write_seq_static(ctx,0x30,0x3F);
	tm_dcs_write_seq_static(ctx,0x31,0xDA);
	tm_dcs_write_seq_static(ctx,0x32,0x3F);
	tm_dcs_write_seq_static(ctx,0x33,0xC9);
	tm_dcs_write_seq_static(ctx,0x34,0x3F);
	tm_dcs_write_seq_static(ctx,0x35,0xDC);
	tm_dcs_write_seq_static(ctx,0x36,0x3F);
	tm_dcs_write_seq_static(ctx,0x37,0xD0);
	tm_dcs_write_seq_static(ctx,0x38,0x3F);
	tm_dcs_write_seq_static(ctx,0x39,0xE0);
	tm_dcs_write_seq_static(ctx,0x3A,0x3F);
	tm_dcs_write_seq_static(ctx,0x3B,0xD6);
	tm_dcs_write_seq_static(ctx,0x3C,0x3F);
	tm_dcs_write_seq_static(ctx,0x3D,0xD3);
	tm_dcs_write_seq_static(ctx,0x3E,0x3F);
	tm_dcs_write_seq_static(ctx,0x3F,0xDD);
	tm_dcs_write_seq_static(ctx,0x80,0x3F);
	tm_dcs_write_seq_static(ctx,0x81,0xDA);
	tm_dcs_write_seq_static(ctx,0x82,0x3F);
	tm_dcs_write_seq_static(ctx,0x83,0xE6);
	tm_dcs_write_seq_static(ctx,0x84,0x3F);
	tm_dcs_write_seq_static(ctx,0x85,0xE6);
	tm_dcs_write_seq_static(ctx,0x86,0x3F);
	tm_dcs_write_seq_static(ctx,0x87,0xCF);
	tm_dcs_write_seq_static(ctx,0x88,0x3F);
	tm_dcs_write_seq_static(ctx,0x89,0xCE);
	tm_dcs_write_seq_static(ctx,0x8A,0x3F);
	tm_dcs_write_seq_static(ctx,0x8B,0xCC);
	tm_dcs_write_seq_static(ctx,0x8C,0x3F);
	tm_dcs_write_seq_static(ctx,0x8D,0xCC);
	tm_dcs_write_seq_static(ctx,0x8E,0x3F);
	tm_dcs_write_seq_static(ctx,0x8F,0xDC);
	tm_dcs_write_seq_static(ctx,0x90,0x3F);
	tm_dcs_write_seq_static(ctx,0x91,0xDE);
	tm_dcs_write_seq_static(ctx,0x92,0x3F);
	tm_dcs_write_seq_static(ctx,0x93,0xD4);
	tm_dcs_write_seq_static(ctx,0x94,0x3F);
	tm_dcs_write_seq_static(ctx,0x95,0xD9);
	tm_dcs_write_seq_static(ctx,0x96,0x3F);
	tm_dcs_write_seq_static(ctx,0x97,0xDD);
	tm_dcs_write_seq_static(ctx,0x98,0x3F);
	tm_dcs_write_seq_static(ctx,0x99,0xDF);
	tm_dcs_write_seq_static(ctx,0x9A,0x3F);
	tm_dcs_write_seq_static(ctx,0x9B,0xD5);
	tm_dcs_write_seq_static(ctx,0x9C,0x3F);
	tm_dcs_write_seq_static(ctx,0x9D,0xE7);
	tm_dcs_write_seq_static(ctx,0x9E,0x3F);
	tm_dcs_write_seq_static(ctx,0x9F,0xD1);
	tm_dcs_write_seq_static(ctx,0xA0,0x3F);
	tm_dcs_write_seq_static(ctx,0xA1,0xD6);
	tm_dcs_write_seq_static(ctx,0xA2,0x3F);
	tm_dcs_write_seq_static(ctx,0xA3,0xD3);
	tm_dcs_write_seq_static(ctx,0xA4,0x3F);
	tm_dcs_write_seq_static(ctx,0xA5,0xE4);
	tm_dcs_write_seq_static(ctx,0xA6,0x3F);
	tm_dcs_write_seq_static(ctx,0xA7,0xDB);
	tm_dcs_write_seq_static(ctx,0xA8,0x3F);
	tm_dcs_write_seq_static(ctx,0xA9,0xE8);
	tm_dcs_write_seq_static(ctx,0xAA,0x3F);
	tm_dcs_write_seq_static(ctx,0xAB,0xE2);
	tm_dcs_write_seq_static(ctx,0xAC,0x3F);
	tm_dcs_write_seq_static(ctx,0xAD,0xD2);
	tm_dcs_write_seq_static(ctx,0xAE,0x3F);
	tm_dcs_write_seq_static(ctx,0xAF,0xE3);
	tm_dcs_write_seq_static(ctx,0xB0,0x3F);
	tm_dcs_write_seq_static(ctx,0xB1,0xCB);
	tm_dcs_write_seq_static(ctx,0xB2,0x3F);
	tm_dcs_write_seq_static(ctx,0xB3,0xD0);
	tm_dcs_write_seq_static(ctx,0xB4,0x3F);
	tm_dcs_write_seq_static(ctx,0xB5,0xE5);
	tm_dcs_write_seq_static(ctx,0xB6,0x3F);
	tm_dcs_write_seq_static(ctx,0xB7,0xCD);
	tm_dcs_write_seq_static(ctx,0xB8,0x3F);
	tm_dcs_write_seq_static(ctx,0xB9,0xD8);
	tm_dcs_write_seq_static(ctx,0xBA,0x3F);
	tm_dcs_write_seq_static(ctx,0xBB,0xC8);
	tm_dcs_write_seq_static(ctx,0xBC,0x3F);
	tm_dcs_write_seq_static(ctx,0xBD,0xC9);
	tm_dcs_write_seq_static(ctx,0xBE,0x3F);
	tm_dcs_write_seq_static(ctx,0xBF,0xCA);
	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x0E);
	tm_dcs_write_seq_static(ctx,0x00,0xA3);
	tm_dcs_write_seq_static(ctx,0x02,0x0F);
	tm_dcs_write_seq_static(ctx,0x04,0x02);
	tm_dcs_write_seq_static(ctx,0xF0,0x02);
	/*************table 1 for 90Hz**************/
	tm_dcs_write_seq_static(ctx,0x20,0x07);
	tm_dcs_write_seq_static(ctx,0x25,0x0A);
	tm_dcs_write_seq_static(ctx,0x26,0xCD);
	tm_dcs_write_seq_static(ctx,0x27,0x10);
	tm_dcs_write_seq_static(ctx,0x29,0x2C);
	tm_dcs_write_seq_static(ctx,0x2D,0x7C);
	/*************table 3 for 60hz**************/
	tm_dcs_write_seq_static(ctx,0x40,0x11);
	tm_dcs_write_seq_static(ctx,0x47,0x00);
	tm_dcs_write_seq_static(ctx,0x49,0xC8);
	tm_dcs_write_seq_static(ctx,0x45,0x0A);
	tm_dcs_write_seq_static(ctx,0x46,0xE1);
	tm_dcs_write_seq_static(ctx,0x4D,0xBA);
	tm_dcs_write_seq_static(ctx,0xC0,0x01);
	tm_dcs_write_seq_static(ctx,0xC6,0x5D);
	tm_dcs_write_seq_static(ctx,0xC7,0x5D);
	tm_dcs_write_seq_static(ctx,0xC8,0x5D);
	tm_dcs_write_seq_static(ctx,0xC9,0x5D);
	tm_dcs_write_seq_static(ctx,0x21,0x85);
	tm_dcs_write_seq_static(ctx,0x23,0x85);
	tm_dcs_write_seq_static(ctx,0x2B,0x1F);
	tm_dcs_write_seq_static(ctx,0x05,0x20);
	tm_dcs_write_seq_static(ctx,0xB0,0x21);
	tm_dcs_write_seq_static(ctx,0x41,0x44);
	tm_dcs_write_seq_static(ctx,0x4B,0x1F);
	/************table 1 for 90Hz*******************/
	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x1E);
	tm_dcs_write_seq_static(ctx,0x00,0x80);
	tm_dcs_write_seq_static(ctx,0x06,0x3E);
	tm_dcs_write_seq_static(ctx,0x07,0x3E);
	tm_dcs_write_seq_static(ctx,0x08,0x3E);
	tm_dcs_write_seq_static(ctx,0x09,0x3E);
	tm_dcs_write_seq_static(ctx,0xA4,0x1D);
	tm_dcs_write_seq_static(ctx,0xA5,0x41);
	tm_dcs_write_seq_static(ctx,0xA6,0x31);
	tm_dcs_write_seq_static(ctx,0xA7,0x25);
	tm_dcs_write_seq_static(ctx,0xC8,0x21);
	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x00);
	tm_dcs_write_seq_static(ctx,0x29);
	msleep(20);
	tm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x0F);
}

static int tm_disable(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tm(panel);

	if (!ctx->enabled)
		return 0;
	pr_info("%s\n", __func__);
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	usleep_range(20 * 1000, 20 * 1000);
	ctx->enabled = false;

	return 0;
}

static int tm_unprepare(struct drm_panel *panel)
{

	struct tianma *ctx = panel_to_tm(panel);
	pr_info("%s hx\n", __func__);

	if (!ctx->prepared)
		return 0;
	if(!ctx->is_normal_mode)
		tp_ftm_extra();
	usleep_range(10 * 1000, 10 * 1000);
	tm_dcs_write_seq_static(ctx, 0xFF,0x78,0x07,0x00);
	usleep_range(5 * 1000, 5 * 1000);
	tm_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	usleep_range(35 * 1000, 35 * 1000);
	tm_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(80 * 1000, 80 * 1000);

    pr_info("%s: tp_gesture_enable_flag = %d \n", __func__, tp_gesture_enable_flag());
    if ((0 == tp_gesture_enable_flag())||(esd_flag == 1)) {
        pr_info("%s: going to cut off power \n", __func__);

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(2 * 1000, 2 * 1000);

        ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
                "bias", 1, GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->bias_neg, 0);
        devm_gpiod_put(ctx->dev, ctx->bias_neg);

	usleep_range(2 * 1000, 2 * 1000);

        ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
                "bias", 0, GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->bias_pos, 0);
        devm_gpiod_put(ctx->dev, ctx->bias_pos);
        usleep_range(2 * 1000, 2 * 1000);
    }

	ctx->error = 0;
	ctx->prepared = false;

	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tm(panel);
	int ret;

	pr_info("ili %s+\n", __func__);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(5000);
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	_lcm_i2c_write_bytes(0x0, 0xf);
	_lcm_i2c_write_bytes(0x1, 0xf);
    usleep_range(2*1000,2*1000);
	pr_info("ili %s-\n", __func__);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	return 0;
}

static int tm_prepare(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tm(panel);
	int ret;

	pr_info("%s ili+\n", __func__);
	if (ctx->prepared)
		return 0;

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(30);

	lcd_queue_load_tp_fw();

	tm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		tm_unprepare(panel);

	ctx->prepared = true;

#ifdef PANEL_SUPPORT_READBACK
	tm_panel_get_data(ctx);
#endif

	pr_info("%s-\n", __func__);
	return ret;
}

static int tm_enable(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

static const struct drm_display_mode default_mode = {
	.clock = 328022,
	.hdisplay = 1080,
	.hsync_start = 1080 + 160,//HFP
	.hsync_end = 1080 + 160 + 90,//HSA
	.htotal = 1080 + 160 + 90 + 158,//HBP
	.vdisplay = 2400,
	.vsync_start = 2400 + 1240,//VFP
	.vsync_end = 2400 + 1240 + 10,//VSA
	.vtotal = 2400 + 1240 + 10 + 15,//VBP
	.vrefresh = 60,
};

static const struct drm_display_mode performance_mode = {
	.clock = 328197,
	.hdisplay = 1080,
	.hsync_start = 1080 + 160,//HFP
	.hsync_end = 1080 + 160 + 90,//HSA
	.htotal = 1080 + 160 + 90 + 158,//HBP
	.vdisplay = 2400,
	.vsync_start = 2400 + 20,//VFP
	.vsync_end = 2400 + 20 + 10,//VSA
	.vtotal = 2400 + 20 + 10 + 15,//VBP
	.vrefresh = 90,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.pll_clk = 553,
	.vfp_low_power = 2480,//45hz
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9C,
	},
	.phy_timcon = {
		.clk_trail = 13,
	},
	//.oplus_panel_index = 20041,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2400,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 43,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 3255,
		.initial_offset = 6144,
		.final_offset = 7072,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		},
	.data_rate = 1107,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 90,
	},
	.vendor = "ILI7807S_TM",
	.manufacture = "ili_tm_4096",
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 553,
	.vfp_low_power = 1240,//60hz
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9C,
	},
	//.oplus_panel_index = 20041,
	.phy_timcon = {
		.clk_trail = 13,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2400,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 43,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 3255,
		.initial_offset = 6144,
		.final_offset = 7072,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		},
	.data_rate = 1107,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 90,
	},
	.vendor = "ILI7807S_TM",
	.manufacture = "ili_tm_4096",
};

static int tm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	static int backlight_gamma = 0;

	char bl_tb0[] = {0x51, 0x07, 0xff};
	char bl_tb1[] = {0xFF,0x78,0x07,0x00};
	char bl_tb2[] = {0xFF,0x78,0x07,0x0F};

	char bl_tb3[] = {0xFF,0x78,0x07,0x08};
	char bl_tb4[] = {0xE0,0x00,0x00,0x1B,0x45,0x00,0x87,0xB4,0xDA,0x15,0x11,0x3D,0x7D,0x25,0xAD,0xF5,0x2F,0x2A,0x66,0xA4,0xCC,0x3E,0xFE,0x24,0x4C,0x3F,0x66,0x88,0xB9,0x0F,0xD7,0xD9};
	char bl_tb5[] = {0xE1,0x00,0x00,0x1B,0x45,0x00,0x87,0xB4,0xDA,0x15,0x11,0x3D,0x7D,0x25,0xAD,0xF5,0x2F,0x2A,0x66,0xA4,0xCC,0x3E,0xFE,0x24,0x4C,0x3F,0x66,0x88,0xB9,0x0F,0xD7,0xD9};

	char bl_tb6[] = {0xE0,0x15,0x9C,0x9E,0xA2,0x15,0xAA,0xB3,0xBB,0x15,0xCA,0xD7,0xF1,0x2A,0x09,0x36,0x60,0x2A,0x8F,0xC6,0xEC,0x3F,0x1B,0x3D,0x64,0x3F,0x7B,0x9A,0xC5,0x0F,0xD8,0xD9};
	char bl_tb7[] = {0xE1,0x15,0x9C,0x9E,0xA2,0x15,0xAA,0xB3,0xBB,0x15,0xCA,0xD7,0xF1,0x2A,0x09,0x36,0x60,0x2A,0x8F,0xC6,0xEC,0x3F,0x1B,0x3D,0x64,0x3F,0x7B,0x9A,0xC5,0x0F,0xD8,0xD9};

	pr_err("%s backlight = %d\n", __func__, level);
	if (level > 4095)
		level = 4095;

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0) {
		level = 2047;
	}

	if(level < 14 && level > 0) {
		backlight_gamma = 1;
		cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
		cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
		cb(dsi, handle, bl_tb7, ARRAY_SIZE(bl_tb7));
	}else if(level > 13 && backlight_gamma == 1) {
		backlight_gamma = 0;
		cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
		cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
		cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
	}
	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;
	esd_brightness = level;
	if (!cb)
		return -1;

	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	return 0;
}

static int oplus_esd_backlight_check(void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0x51, 0x07, 0xff};

	pr_err("%s esd_backlight = %d\n", __func__, esd_brightness);
	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
	if (!cb)
		return -1;
	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 1;
}

static struct drm_display_mode *get_mode_by_id(struct drm_panel *panel,
   unsigned int mode)
{
   struct drm_display_mode *m;
   unsigned int i = 0;

   list_for_each_entry(m, &panel->connector->modes, head) {
          if (i == mode)
                  return m;
          i++;
  }
  return NULL;
}
static int mtk_panel_ext_param_set(struct drm_panel *panel, unsigned int mode)
{
  struct mtk_panel_ext *ext = find_panel_ext(panel);
  int ret = 0;
  struct drm_display_mode *m = get_mode_by_id(panel, mode);
  pr_info("%s: mode is %d, refresh is %d\n", __func__, mode,m->vrefresh);

  if (m->vrefresh == 60)
          ext->params = &ext_params;
  else if (m->vrefresh == 90)
          ext->params = &ext_params_90hz;
  //else if (m->vrefresh == 120)
          //ext->params = &ext_params_120hz;
  else
          ret = 1;

  return ret;
}

static int mtk_panel_ext_param_get(struct mtk_panel_params *ext_para,
			 unsigned int mode)
{
	int ret = 0;
	pr_err("%s ,%d\n", __func__,mode);
	if (mode == 0)
		ext_para = &ext_params;
	else if (mode == 1)
		ext_para = &ext_params_90hz;
	else
		ret = 1;

	return ret;

}

static void cabc_switch(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int cabc_mode)
{
	char bl_tb1[] = {0x55, 0x00};
	char bl_tb0[] = {0xFF,0x78,0x07,0x00};
	char bl_tb2[] = {0xFF,0x78,0x07,0x0F};

	pr_err("%s cabc = %d\n", __func__, cabc_mode);
	if(cabc_mode == 2)
		return;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0)); //page 0
	if (cabc_mode == 1) {
		bl_tb1[1] = 2;
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	}else if(cabc_mode == 3){
		bl_tb1[1] = 3;
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	}else if(cabc_mode == 0){
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	}
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2)); //page F
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct tianma *ctx = panel_to_tm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int panel_info_get(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tm(panel);

	return ctx->info;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = tm_setbacklight_cmdq,
	.esd_backlight_check = oplus_esd_backlight_check,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	//.info_get = panel_info_get,
	//.mode_switch = mode_switch,
	.cabc_switch = cabc_switch,
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	/**
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *           become ready and start receiving video data
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *          display the first valid frame after starting to receive
	 *          video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *           turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *             to power itself down completely
	 */
	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static int jdi_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *mode2;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

	mode2 = drm_mode_duplicate(panel->drm, &performance_mode);
	if (!mode2) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			performance_mode.hdisplay,
			performance_mode.vdisplay,
			performance_mode.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode2);

	panel->connector->display_info.width_mm = 68;
	panel->connector->display_info.height_mm = 150;

	return 1;
}

static const struct drm_panel_funcs jdi_drm_funcs = {
	.disable = tm_disable,
	.unprepare = tm_unprepare,
	.prepare = tm_prepare,
	.enable = tm_enable,
	.get_modes = jdi_get_modes,
};

static int jdi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct tianma *ctx;
	struct device_node *backlight;
	int ret;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			pr_info("device_node name:%s\n", remote_node->name);
                   }
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm.\n", __func__);
		return 0;
	}
	pr_info("hx %s+\n", __func__);
	ctx = devm_kzalloc(dev, sizeof(struct tianma), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->info = 2004101;
	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
			 |MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "cannot get reset-gpios %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);
	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(dev, "cannot get bias-gpios 0 %ld\n",
			PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(dev, "cannot get bias-gpios 1 %ld\n",
		PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	devm_gpiod_put(dev, ctx->bias_neg);

	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &jdi_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0){
		return ret;
	}

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0){
		return ret;
	}
#endif

        ctx->is_normal_mode = true;
        if( META_BOOT == get_boot_mode() || FACTORY_BOOT == get_boot_mode() )
            ctx->is_normal_mode = false;
        pr_info("%s: is_normal_mode = %d \n", __func__, ctx->is_normal_mode);

	register_device_proc("lcd", "ILI7807S_TM", "ili_tm_4096");
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	pr_info("%s-\n", __func__);

	return ret;
}

static int jdi_remove(struct mipi_dsi_device *dsi)
{
	struct tianma *ctx = mipi_dsi_get_drvdata(dsi);
	//NVT H -> L
	pr_info(" %s will reset pin to L\n", __func__);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	//end

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id jdi_of_match[] = {
	{
		.compatible = "oplus20095,tianma,ili7807s,dphy,vdo",
	},
	{} };

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
	.probe = jdi_probe,
	.remove = jdi_remove,
	.driver = {
			.name = "panel-oplus20095_ili7807s_fhdp_dsi_vdo_dphy",
			.owner = THIS_MODULE,
			.of_match_table = jdi_of_match,
		},
};

module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("Elon Hsu <elon.hsu@mediatek.com>");
MODULE_DESCRIPTION("jdi r66451 CMD AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
