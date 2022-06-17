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
#include <soc/oplus/oplus_project.h>
#include <linux/gpio/consumer.h>
#include <linux/of_graph.h>
#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

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

/* #ifdef OPLUS_BUG_STABILITY */
#include <mt-plat/mtk_boot_common.h>
/* #endif */ /* OPLUS_BUG_STABILITY */
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/oplus19131_data_hw_roundedpattern.h"
#endif

#define AVDD_REG 0x00
#define AVDD_REG 0x01
#define MAX_NORMAL_BRIGHTNESS   2047

/* i2c control start */
#define LCM_I2C_ID_NAME "I2C_LCD_BIAS"
static struct i2c_client *_lcm_i2c_client;
static unsigned int is_export_project = 0;
extern unsigned long oplus_max_normal_brightness;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int _lcm_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
static int _lcm_i2c_remove(struct i2c_client *client);

/* #ifdef OPLUS_BUG_STABILITY */
extern int __attribute__((weak)) tp_gesture_enable_flag(void) {return 0;};
extern void __attribute__((weak)) lcd_queue_load_tp_fw(void) {return;};
void __attribute__((weak)) switch_spi7cs_state(bool normal) {return;}
/* #endif */ /* OPLUS_BUG_STABILITY */
static int esd_brightness;
/* extern unsigned long esd_flag; */
static unsigned long esd_flag = 0;
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

struct jdi {
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

    bool is_normal_mode;
};

#define jdi_dcs_write_seq(ctx, seq...)                                     \
	({                                                                     \
		const u8 d[] = {seq};                                          \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		jdi_dcs_write(ctx, d, ARRAY_SIZE(d));                      \
	})

#define jdi_dcs_write_seq_static(ctx, seq...)                              \
	({                                                                     \
		static const u8 d[] = {seq};                                   \
		jdi_dcs_write(ctx, d, ARRAY_SIZE(d));                      \
	})

static inline struct jdi *panel_to_jdi(struct drm_panel *panel)
{
	return container_of(panel, struct jdi, panel);
}

#ifdef PANEL_SUPPORT_READBACK
static int jdi_dcs_read(struct jdi *ctx, u8 cmd, void *data, size_t len)
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

static void jdi_panel_get_data(struct jdi *ctx)
{
	u8 buffer[3] = {0};
	static int ret;
	pr_info("%s+\n", __func__);

	if (ret == 0) {
		ret = jdi_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("%s  0x%08x\n", __func__,buffer[0] | (buffer[1] << 8));
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static void jdi_dcs_write(struct jdi *ctx, const void *data, size_t len)
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

static void jdi_vdd_tp_start(struct jdi *ctx)
{
	jdi_dcs_write_seq_static(ctx, 0xFF,0xC0);
	jdi_dcs_write_seq_static(ctx, 0X4B,0x1F);
}

static void jdi_vdd_tp_end(struct jdi *ctx)
{
	jdi_dcs_write_seq_static(ctx, 0xFF,0xC0);
	jdi_dcs_write_seq_static(ctx, 0X4B,0x0E);
}

static void jdi_panel_init(struct jdi *ctx)
{
	int version;
	version = get_PCB_Version();
	pr_info("%s+ PCB_version:%d\n", __func__,version);
	//add for cabc
	jdi_dcs_write_seq_static(ctx, 0xFF,0x23);
	jdi_dcs_write_seq_static(ctx, 0XFB,0x01);

	jdi_dcs_write_seq_static(ctx, 0x05,0x22);
	jdi_dcs_write_seq_static(ctx, 0x06,0x01);

	jdi_dcs_write_seq_static(ctx, 0x08,0x07);
	jdi_dcs_write_seq_static(ctx, 0x09,0x04);
	jdi_dcs_write_seq_static(ctx, 0x10,0x82);
	jdi_dcs_write_seq_static(ctx, 0x11,0x01);
	jdi_dcs_write_seq_static(ctx, 0x12,0x95);
	jdi_dcs_write_seq_static(ctx, 0x15,0x68);
	jdi_dcs_write_seq_static(ctx, 0x16,0x0B);
	//CABC_PWM_UI
	jdi_dcs_write_seq_static(ctx,0x30,0xFF);
	jdi_dcs_write_seq_static(ctx,0x31,0xFD);
	jdi_dcs_write_seq_static(ctx,0x32,0xFA);
	jdi_dcs_write_seq_static(ctx,0x33,0xF7);
	jdi_dcs_write_seq_static(ctx,0x34,0xF4);
	jdi_dcs_write_seq_static(ctx,0x35,0xF0);
	jdi_dcs_write_seq_static(ctx,0x36,0xED);
	jdi_dcs_write_seq_static(ctx,0x37,0xEC);
	jdi_dcs_write_seq_static(ctx,0x38,0xEB);
	jdi_dcs_write_seq_static(ctx,0x39,0xEA);
	jdi_dcs_write_seq_static(ctx,0x3A,0xE9);
	jdi_dcs_write_seq_static(ctx,0x3B,0xE8);
	jdi_dcs_write_seq_static(ctx,0x3D,0xE7);
	jdi_dcs_write_seq_static(ctx,0x3F,0xE6);
	jdi_dcs_write_seq_static(ctx,0x40,0xE5);
	jdi_dcs_write_seq_static(ctx,0x41,0xE4);
	//CABC_PWM_STILL
	jdi_dcs_write_seq_static(ctx,0x45,0xFF);
	jdi_dcs_write_seq_static(ctx,0x46,0xFA);
	jdi_dcs_write_seq_static(ctx,0x47,0xF2);
	jdi_dcs_write_seq_static(ctx,0x48,0xE8);
	jdi_dcs_write_seq_static(ctx,0x49,0xE4);
	jdi_dcs_write_seq_static(ctx,0x4A,0xDC);
	jdi_dcs_write_seq_static(ctx,0x4B,0xD7);
	jdi_dcs_write_seq_static(ctx,0x4C,0xD5);
	jdi_dcs_write_seq_static(ctx,0x4D,0xD3);
	jdi_dcs_write_seq_static(ctx,0x4E,0xD2);
	jdi_dcs_write_seq_static(ctx,0x4F,0xD0);
	jdi_dcs_write_seq_static(ctx,0x50,0xCE);
	jdi_dcs_write_seq_static(ctx,0x51,0xCD);
	jdi_dcs_write_seq_static(ctx,0x52,0xCB);
	jdi_dcs_write_seq_static(ctx,0x53,0xC6);
	jdi_dcs_write_seq_static(ctx,0x54,0xC3);

	jdi_dcs_write_seq_static(ctx, 0xA0,0x11);

	jdi_dcs_write_seq_static(ctx, 0xFF,0xF0);
	jdi_dcs_write_seq_static(ctx, 0xFB,0x01);
	jdi_dcs_write_seq_static(ctx, 0xD2,0x52);
//cabc end
      //set 120hz
	jdi_dcs_write_seq_static(ctx, 0xFF, 0x25);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x18, 0x22);

	jdi_dcs_write_seq_static(ctx, 0xFF, 0x10);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0xB0, 0x01); //Enable MIPI CRC
	//DSC ON && set PPS
	jdi_dcs_write_seq_static(ctx, 0xC0, 0x03);
	jdi_dcs_write_seq_static(ctx, 0xC1, 0x89, 0x28, 0x00, 0x08, 0x00, 0xAA,
				0x02, 0x0E, 0x00, 0x2B, 0x00, 0x07, 0x0D, 0xB7,
				0x0C, 0xB7);
	jdi_dcs_write_seq_static(ctx, 0xC2, 0x1B, 0xA0);
	if( version < 5 ){
		jdi_dcs_write_seq_static(ctx, 0xFF, 0x20);
		jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
		jdi_dcs_write_seq_static(ctx, 0x01, 0x66);
		jdi_dcs_write_seq_static(ctx, 0x06, 0x40);
		jdi_dcs_write_seq_static(ctx, 0x07, 0x38);
		jdi_dcs_write_seq_static(ctx, 0x69, 0x91);
		//jdi_dcs_write_seq_static(ctx, 0x89, 0x17);
		jdi_dcs_write_seq_static(ctx, 0x95, 0xD1);
		jdi_dcs_write_seq_static(ctx, 0x96, 0xD1);
		jdi_dcs_write_seq_static(ctx, 0xF2, 0x64);
		jdi_dcs_write_seq_static(ctx, 0xF4, 0x64);
		jdi_dcs_write_seq_static(ctx, 0xF6, 0x64);
		jdi_dcs_write_seq_static(ctx, 0xF8, 0x64);

		jdi_dcs_write_seq_static(ctx, 0xFF, 0x24);
		jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
		jdi_dcs_write_seq_static(ctx, 0x01, 0x0F);
		jdi_dcs_write_seq_static(ctx, 0x03, 0x0C);
		jdi_dcs_write_seq_static(ctx, 0x05, 0x1D);
		jdi_dcs_write_seq_static(ctx, 0x08, 0x2F);
		jdi_dcs_write_seq_static(ctx, 0x09, 0x2E);
		jdi_dcs_write_seq_static(ctx, 0x0A, 0x2D);
		jdi_dcs_write_seq_static(ctx, 0x0B, 0x2C);
		jdi_dcs_write_seq_static(ctx, 0x11, 0x17);
		jdi_dcs_write_seq_static(ctx, 0x12, 0x13);
		jdi_dcs_write_seq_static(ctx, 0x13, 0x15);
		jdi_dcs_write_seq_static(ctx, 0x15, 0x14);
		jdi_dcs_write_seq_static(ctx, 0x16, 0x16);
		jdi_dcs_write_seq_static(ctx, 0x17, 0x18);
		jdi_dcs_write_seq_static(ctx, 0x1B, 0x01);
		jdi_dcs_write_seq_static(ctx, 0x1D, 0x1D);
		jdi_dcs_write_seq_static(ctx, 0x20, 0x2F);
		jdi_dcs_write_seq_static(ctx, 0x21, 0x2E);
		jdi_dcs_write_seq_static(ctx, 0x22, 0x2D);
		jdi_dcs_write_seq_static(ctx, 0x23, 0x2C);
		jdi_dcs_write_seq_static(ctx, 0x29, 0x17);
		jdi_dcs_write_seq_static(ctx, 0x2A, 0x13);
		jdi_dcs_write_seq_static(ctx, 0x2B, 0x15);
		jdi_dcs_write_seq_static(ctx, 0x2F, 0x14);
		jdi_dcs_write_seq_static(ctx, 0x30, 0x16);
		jdi_dcs_write_seq_static(ctx, 0x31, 0x18);
		jdi_dcs_write_seq_static(ctx, 0x32, 0x04);
		jdi_dcs_write_seq_static(ctx, 0x34, 0x10);
		jdi_dcs_write_seq_static(ctx, 0x35, 0x1F);
		jdi_dcs_write_seq_static(ctx, 0x36, 0x1F);
		jdi_dcs_write_seq_static(ctx, 0x37, 0x20);
		jdi_dcs_write_seq_static(ctx, 0x4D, 0x1B);
		jdi_dcs_write_seq_static(ctx, 0x4E, 0x4B);
		jdi_dcs_write_seq_static(ctx, 0x4F, 0x4B);
		jdi_dcs_write_seq_static(ctx, 0x53, 0x4B);
		jdi_dcs_write_seq_static(ctx, 0x71, 0x30);
		jdi_dcs_write_seq_static(ctx, 0x79, 0x11);
		jdi_dcs_write_seq_static(ctx, 0x7A, 0x82);
		jdi_dcs_write_seq_static(ctx, 0x7B, 0x96);
		jdi_dcs_write_seq_static(ctx, 0x7D, 0x04);
		jdi_dcs_write_seq_static(ctx, 0x80, 0x04);
		jdi_dcs_write_seq_static(ctx, 0x81, 0x04);
		jdi_dcs_write_seq_static(ctx, 0x82, 0x13);
		jdi_dcs_write_seq_static(ctx, 0x84, 0x31);
		jdi_dcs_write_seq_static(ctx, 0x85, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x86, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x87, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x90, 0x13);
		jdi_dcs_write_seq_static(ctx, 0x92, 0x31);
		jdi_dcs_write_seq_static(ctx, 0x93, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x94, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x95, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x9C, 0xF4);
		jdi_dcs_write_seq_static(ctx, 0x9D, 0x01);
		jdi_dcs_write_seq_static(ctx, 0xA0, 0x16);
		jdi_dcs_write_seq_static(ctx, 0xA2, 0x16);
		jdi_dcs_write_seq_static(ctx, 0xA3, 0x02);
		jdi_dcs_write_seq_static(ctx, 0xA4, 0x04);
		jdi_dcs_write_seq_static(ctx, 0xA5, 0x04);
		jdi_dcs_write_seq_static(ctx, 0xC9, 0x00);
		jdi_dcs_write_seq_static(ctx, 0xD9, 0x80);
		jdi_dcs_write_seq_static(ctx, 0xE9, 0x02);

		jdi_dcs_write_seq_static(ctx, 0xFF, 0x25);
		jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
		jdi_dcs_write_seq_static(ctx, 0x19, 0xE4);
		jdi_dcs_write_seq_static(ctx, 0x21, 0x40);
		jdi_dcs_write_seq_static(ctx, 0x66, 0xD8);
		jdi_dcs_write_seq_static(ctx, 0x68, 0x50);
		jdi_dcs_write_seq_static(ctx, 0x69, 0x10);
		jdi_dcs_write_seq_static(ctx, 0x6B, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x6D, 0x0D);
		jdi_dcs_write_seq_static(ctx, 0x6E, 0x48);
		jdi_dcs_write_seq_static(ctx, 0x72, 0x41);
		jdi_dcs_write_seq_static(ctx, 0x73, 0x4A);
		jdi_dcs_write_seq_static(ctx, 0x74, 0xD0);
		//jdi_dcs_write_seq_static(ctx, 0x76, 0x83);
		jdi_dcs_write_seq_static(ctx, 0x77, 0x62);
		jdi_dcs_write_seq_static(ctx, 0x79, 0x81);//add
		jdi_dcs_write_seq_static(ctx, 0x7D, 0x03);
		jdi_dcs_write_seq_static(ctx, 0x7E, 0x15);
		jdi_dcs_write_seq_static(ctx, 0x7F, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x84, 0x4D);
		jdi_dcs_write_seq_static(ctx, 0xCF, 0x80);
		jdi_dcs_write_seq_static(ctx, 0xD6, 0x80);
		jdi_dcs_write_seq_static(ctx, 0xD7, 0x80);
		jdi_dcs_write_seq_static(ctx, 0xEF, 0x20);
		jdi_dcs_write_seq_static(ctx, 0xF0, 0x84);

		jdi_dcs_write_seq_static(ctx, 0xFF, 0x26);
		jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
		jdi_dcs_write_seq_static(ctx, 0x80, 0x01);
		jdi_dcs_write_seq_static(ctx, 0x81, 0x16);
		jdi_dcs_write_seq_static(ctx, 0x83, 0x03);
		jdi_dcs_write_seq_static(ctx, 0x84, 0x03);
		jdi_dcs_write_seq_static(ctx, 0x85, 0x01);
		jdi_dcs_write_seq_static(ctx, 0x86, 0x03);
		jdi_dcs_write_seq_static(ctx, 0x87, 0x01);
		jdi_dcs_write_seq_static(ctx, 0x8A, 0x1A);
		jdi_dcs_write_seq_static(ctx, 0x8B, 0x11);
		jdi_dcs_write_seq_static(ctx, 0x8C, 0x24);
		jdi_dcs_write_seq_static(ctx, 0x8E, 0x42);
		jdi_dcs_write_seq_static(ctx, 0x8F, 0x11);
		jdi_dcs_write_seq_static(ctx, 0x90, 0x11);
		jdi_dcs_write_seq_static(ctx, 0x91, 0x11);
		jdi_dcs_write_seq_static(ctx, 0x9A, 0x81);
		jdi_dcs_write_seq_static(ctx, 0x9B, 0x03);
		jdi_dcs_write_seq_static(ctx, 0x9C, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x9D, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x9E, 0x00);

		jdi_dcs_write_seq_static(ctx, 0xFF, 0x27);
		jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
		jdi_dcs_write_seq_static(ctx, 0x01, 0x60);
		jdi_dcs_write_seq_static(ctx, 0x20, 0x81);
		jdi_dcs_write_seq_static(ctx, 0x21, 0xEA);
		jdi_dcs_write_seq_static(ctx, 0x25, 0x82);
		jdi_dcs_write_seq_static(ctx, 0x26, 0x1F);
		jdi_dcs_write_seq_static(ctx, 0x6E, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x6F, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x70, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x71, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x72, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x75, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x76, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x77, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x7D, 0x09);
		jdi_dcs_write_seq_static(ctx, 0x7E, 0x5F);
		jdi_dcs_write_seq_static(ctx, 0x80, 0x23);
		jdi_dcs_write_seq_static(ctx, 0x82, 0x09);
		jdi_dcs_write_seq_static(ctx, 0x83, 0x5F);
		jdi_dcs_write_seq_static(ctx, 0x88, 0x01);
		jdi_dcs_write_seq_static(ctx, 0x89, 0x10);
		jdi_dcs_write_seq_static(ctx, 0xA5, 0x10);
		jdi_dcs_write_seq_static(ctx, 0xA6, 0x23);
		jdi_dcs_write_seq_static(ctx, 0xA7, 0x01);
		jdi_dcs_write_seq_static(ctx, 0xB6, 0x40);
		jdi_dcs_write_seq_static(ctx, 0xE3, 0x02);
		jdi_dcs_write_seq_static(ctx, 0xE4, 0xE0);
		jdi_dcs_write_seq_static(ctx, 0xE5, 0x01);//add
		jdi_dcs_write_seq_static(ctx, 0xE6, 0x70);//add
		jdi_dcs_write_seq_static(ctx, 0xE9, 0x03);
		jdi_dcs_write_seq_static(ctx, 0xEA, 0x2F);
		jdi_dcs_write_seq_static(ctx, 0xEB, 0x01);//add
		jdi_dcs_write_seq_static(ctx, 0xEC, 0x98);//add

		jdi_dcs_write_seq_static(ctx, 0xFF, 0x2A);
		jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
		jdi_dcs_write_seq_static(ctx, 0x00, 0x91);
		jdi_dcs_write_seq_static(ctx, 0x03, 0x20);
		jdi_dcs_write_seq_static(ctx, 0x07, 0x52);// modify
		jdi_dcs_write_seq_static(ctx, 0x0A, 0x60);
		jdi_dcs_write_seq_static(ctx, 0x0C, 0x06);
		jdi_dcs_write_seq_static(ctx, 0x0D, 0x40);
		jdi_dcs_write_seq_static(ctx, 0x0E, 0x02);
		jdi_dcs_write_seq_static(ctx, 0x0F, 0x01);
		jdi_dcs_write_seq_static(ctx, 0x11, 0x58);
		jdi_dcs_write_seq_static(ctx, 0x15, 0x0E);
		jdi_dcs_write_seq_static(ctx, 0x16, 0x79);
		jdi_dcs_write_seq_static(ctx, 0x19, 0x0D);
		jdi_dcs_write_seq_static(ctx, 0x1A, 0xF2);
		jdi_dcs_write_seq_static(ctx, 0x1B, 0x14);
		jdi_dcs_write_seq_static(ctx, 0x1D, 0x36);
		jdi_dcs_write_seq_static(ctx, 0x1E, 0x55);
		jdi_dcs_write_seq_static(ctx, 0x1F, 0x55);
		jdi_dcs_write_seq_static(ctx, 0x20, 0x55);
		//jdi_dcs_write_seq_static(ctx, 0x27, 0x80);
		jdi_dcs_write_seq_static(ctx, 0x28, 0x0A);
		jdi_dcs_write_seq_static(ctx, 0x29, 0x0B);
		jdi_dcs_write_seq_static(ctx, 0x2A, 0x4B);//modify
		jdi_dcs_write_seq_static(ctx, 0x2B, 0x05);//modify
		jdi_dcs_write_seq_static(ctx, 0x2D, 0x08);//add
		jdi_dcs_write_seq_static(ctx, 0x2F, 0x01);
		jdi_dcs_write_seq_static(ctx, 0x30, 0x47);
		jdi_dcs_write_seq_static(ctx, 0x31, 0x23);//modify
		jdi_dcs_write_seq_static(ctx, 0x33, 0x25);//modify
		jdi_dcs_write_seq_static(ctx, 0x34, 0xFF);
		jdi_dcs_write_seq_static(ctx, 0x35, 0x2C);//modify
		jdi_dcs_write_seq_static(ctx, 0x36, 0x75);//modify
		jdi_dcs_write_seq_static(ctx, 0x37, 0xFB);//modify
		jdi_dcs_write_seq_static(ctx, 0x38, 0x2E);//MODIFY
		jdi_dcs_write_seq_static(ctx, 0x39, 0x73);//MODIFY
		jdi_dcs_write_seq_static(ctx, 0x3A, 0x47);
		//jdi_dcs_write_seq_static(ctx, 0x44, 0x4C);
		//jdi_dcs_write_seq_static(ctx, 0x45, 0x09);
		jdi_dcs_write_seq_static(ctx, 0x46, 0x40);
		jdi_dcs_write_seq_static(ctx, 0x47, 0x02);
		//jdi_dcs_write_seq_static(ctx, 0x48, 0x00);
		jdi_dcs_write_seq_static(ctx, 0x4A, 0xF0);
		jdi_dcs_write_seq_static(ctx, 0x4E, 0x0E);
		jdi_dcs_write_seq_static(ctx, 0x4F, 0x8B);//MODIFY
		jdi_dcs_write_seq_static(ctx, 0x52, 0x0E);//MODIFY
		jdi_dcs_write_seq_static(ctx, 0x53, 0x04);//MODIFY
		jdi_dcs_write_seq_static(ctx, 0x54, 0x14);
		jdi_dcs_write_seq_static(ctx, 0x56, 0x36);
		jdi_dcs_write_seq_static(ctx, 0x57, 0x80);//MODIFY
		jdi_dcs_write_seq_static(ctx, 0x58, 0x80);//MODIFY
		jdi_dcs_write_seq_static(ctx, 0x59, 0x80);//MODIFY
		jdi_dcs_write_seq_static(ctx, 0x60, 0x80);
		jdi_dcs_write_seq_static(ctx, 0x61, 0x0A);
		jdi_dcs_write_seq_static(ctx, 0x62, 0x03);
		jdi_dcs_write_seq_static(ctx, 0x63, 0xED);//MODIFY
		jdi_dcs_write_seq_static(ctx, 0x65, 0x05);//ADD
		jdi_dcs_write_seq_static(ctx, 0x66, 0x01);
		jdi_dcs_write_seq_static(ctx, 0x67, 0x04);
		jdi_dcs_write_seq_static(ctx, 0x68, 0x4D);//MODIFY
		jdi_dcs_write_seq_static(ctx, 0x6A, 0x0A);//ADD
		jdi_dcs_write_seq_static(ctx, 0x6B, 0xC9);//MODIFY
		jdi_dcs_write_seq_static(ctx, 0x6C, 0x1F);//MODIFY
		jdi_dcs_write_seq_static(ctx, 0x6D, 0xE3);//MODIFY
		jdi_dcs_write_seq_static(ctx, 0x6E, 0xC6);//MODIFY
		jdi_dcs_write_seq_static(ctx, 0x6F, 0x20);//MODIFY
		jdi_dcs_write_seq_static(ctx, 0x70, 0xE2);//MODIFY
		jdi_dcs_write_seq_static(ctx, 0x71, 0x04);
		jdi_dcs_write_seq_static(ctx, 0x7A, 0X04);//ADD
		jdi_dcs_write_seq_static(ctx, 0X7B, 0X04);//ADD
		jdi_dcs_write_seq_static(ctx, 0X7C, 0x01);//ADD
		jdi_dcs_write_seq_static(ctx, 0X7D, 0x01);//ADD
		jdi_dcs_write_seq_static(ctx, 0X7F, 0XE0);//ADD
		jdi_dcs_write_seq_static(ctx, 0X83, 0X0E);//ADD
		jdi_dcs_write_seq_static(ctx, 0X84, 0X8B);//ADD
		jdi_dcs_write_seq_static(ctx, 0X87, 0X0E);//ADD
		jdi_dcs_write_seq_static(ctx, 0X88, 0X04);//ADD
		jdi_dcs_write_seq_static(ctx, 0X89, 0X14);//ADD
		jdi_dcs_write_seq_static(ctx, 0X8B, 0X36);//ADD
		jdi_dcs_write_seq_static(ctx, 0X8C, 0X40);//ADD
		jdi_dcs_write_seq_static(ctx, 0X8D, 0X40);//ADD
		jdi_dcs_write_seq_static(ctx, 0X8E, 0X40);//ADD
		jdi_dcs_write_seq_static(ctx, 0X95, 0X80);//ADD
		jdi_dcs_write_seq_static(ctx, 0X96, 0X0A);//ADD
		jdi_dcs_write_seq_static(ctx, 0X97, 0X12);//ADD
		jdi_dcs_write_seq_static(ctx, 0X98, 0X92);//ADD
		jdi_dcs_write_seq_static(ctx, 0X9A, 0X0A);//ADD
		jdi_dcs_write_seq_static(ctx, 0X9B, 0X02);//ADD
		jdi_dcs_write_seq_static(ctx, 0X9C, 0X49);//ADD
		jdi_dcs_write_seq_static(ctx, 0X9D, 0X98);//ADD
		jdi_dcs_write_seq_static(ctx, 0X9F, 0X5F);//ADD
		jdi_dcs_write_seq_static(ctx, 0XA0, 0XFF);//ADD
		jdi_dcs_write_seq_static(ctx, 0XA2, 0X3A);//ADD
		jdi_dcs_write_seq_static(ctx, 0XA3, 0XD9);//ADD
		jdi_dcs_write_seq_static(ctx, 0XA4, 0XFA);//ADD
		jdi_dcs_write_seq_static(ctx, 0XA5, 0X3C);//ADD
		jdi_dcs_write_seq_static(ctx, 0XA6, 0XD7);//ADD
		jdi_dcs_write_seq_static(ctx, 0XA7, 0X49);//ADD

		jdi_dcs_write_seq_static(ctx, 0xFF, 0x2C);
		jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
		jdi_dcs_write_seq_static(ctx, 0x00, 0x02);
		jdi_dcs_write_seq_static(ctx, 0x01, 0x02);
		jdi_dcs_write_seq_static(ctx, 0x02, 0x02);
		jdi_dcs_write_seq_static(ctx, 0x03, 0x16);
		jdi_dcs_write_seq_static(ctx, 0x04, 0x16);
		jdi_dcs_write_seq_static(ctx, 0x05, 0x16);
		jdi_dcs_write_seq_static(ctx, 0x0D, 0x1F);
		jdi_dcs_write_seq_static(ctx, 0x0E, 0x1F);
		jdi_dcs_write_seq_static(ctx, 0x16, 0x1B);
		jdi_dcs_write_seq_static(ctx, 0x17, 0x4B);
		jdi_dcs_write_seq_static(ctx, 0x18, 0x4B);
		jdi_dcs_write_seq_static(ctx, 0x19, 0x4B);
		jdi_dcs_write_seq_static(ctx, 0x2A, 0x03);
		//jdi_dcs_write_seq_static(ctx, 0x3B, 0x01);
		jdi_dcs_write_seq_static(ctx, 0x4D, 0x16);
		jdi_dcs_write_seq_static(ctx, 0x4E, 0x03);
		jdi_dcs_write_seq_static(ctx, 0X53, 0X02);//ADD
		jdi_dcs_write_seq_static(ctx, 0X54, 0X02);//ADD
		jdi_dcs_write_seq_static(ctx, 0X55, 0X02);//ADD
		jdi_dcs_write_seq_static(ctx, 0X56, 0X0F);//ADD
		jdi_dcs_write_seq_static(ctx, 0X58, 0X0F);//ADD
		jdi_dcs_write_seq_static(ctx, 0X59, 0X0F);//ADD
		jdi_dcs_write_seq_static(ctx, 0X61, 0X1F);//ADD
		jdi_dcs_write_seq_static(ctx, 0X62, 0X1F);//ADD
		jdi_dcs_write_seq_static(ctx, 0X6A, 0X15);//ADD
		jdi_dcs_write_seq_static(ctx, 0X6B, 0X37);//ADD
		jdi_dcs_write_seq_static(ctx, 0X6C, 0X37);//ADD
		jdi_dcs_write_seq_static(ctx, 0X6D, 0X37);//ADD
		jdi_dcs_write_seq_static(ctx, 0X7E, 0X03);//ADD
		jdi_dcs_write_seq_static(ctx, 0X9D, 0X10);//ADD
		jdi_dcs_write_seq_static(ctx, 0X9E, 0X03);//ADD
	}
	// CCMON
	jdi_dcs_write_seq_static(ctx, 0xFF, 0xF0);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x5A, 0x00);

//11bit
	jdi_dcs_write_seq_static(ctx, 0xFF,0x23);
	msleep(1);
	jdi_dcs_write_seq_static(ctx, 0xFB,0x01);
	jdi_dcs_write_seq_static(ctx, 0x00,0x68);
	jdi_dcs_write_seq_static(ctx, 0x07,0x00);
	jdi_dcs_write_seq_static(ctx, 0x08,0x01);
	jdi_dcs_write_seq_static(ctx, 0x09,0xC7);

        jdi_dcs_write_seq_static(ctx, 0xFF,0x2A);
	msleep(1);
	jdi_dcs_write_seq_static(ctx, 0xFB,0x01);
	jdi_dcs_write_seq_static(ctx, 0x7B,0x40);//add 2020/03/19
	//CCMOFF
	//CCMRUN
	jdi_dcs_write_seq_static(ctx, 0xFF,0XC0);
	jdi_dcs_write_seq_static(ctx, 0xFB,0x01);
	jdi_dcs_write_seq_static(ctx, 0x9C,0x11); // VESA
	jdi_dcs_write_seq_static(ctx, 0x9D,0x11);// VESA

	jdi_dcs_write_seq_static(ctx, 0xFF,0XE0);
	jdi_dcs_write_seq_static(ctx, 0xFB,0X01);
	jdi_dcs_write_seq_static(ctx, 0x35,0X82);//VESA

	jdi_dcs_write_seq_static(ctx, 0xFF,0XD0);
	jdi_dcs_write_seq_static(ctx, 0xFB,0x01);
	jdi_dcs_write_seq_static(ctx, 0x53,0x22);//C-PHY
	jdi_dcs_write_seq_static(ctx, 0x54,0x02);//C-PHY

	jdi_dcs_write_seq_static(ctx, 0xFF,0x10);
	jdi_dcs_write_seq_static(ctx, 0xFB,0x01);
	jdi_dcs_write_seq_static(ctx, 0x53,0x24);
	jdi_dcs_write_seq_static(ctx, 0x55,0x01);
    //open TE
	jdi_dcs_write_seq_static(ctx, 0x35,0x00);
	//end by zsq
	jdi_dcs_write_seq_static(ctx, 0x11);

	msleep(80);
	/* Display On*/
	jdi_dcs_write_seq_static(ctx, 0x29);
	msleep(10);
	pr_info("%s-\n", __func__);
}

static void jdi_export_panel_init(struct jdi *ctx)
{
	//add for cabc
	jdi_dcs_write_seq_static(ctx, 0xFF,0x23);
	jdi_dcs_write_seq_static(ctx, 0XFB,0x01);

	jdi_dcs_write_seq_static(ctx, 0x05,0x22);
	jdi_dcs_write_seq_static(ctx, 0x06,0x01);

	jdi_dcs_write_seq_static(ctx, 0x08,0x07);
	jdi_dcs_write_seq_static(ctx, 0x09,0x04);
	jdi_dcs_write_seq_static(ctx, 0x10,0x82);
	jdi_dcs_write_seq_static(ctx, 0x11,0x01);
	jdi_dcs_write_seq_static(ctx, 0x12,0x95);
	jdi_dcs_write_seq_static(ctx, 0x15,0x68);
	jdi_dcs_write_seq_static(ctx, 0x16,0x0B);
	//CABC_PWM_UI
	jdi_dcs_write_seq_static(ctx,0x30,0xFF);
	jdi_dcs_write_seq_static(ctx,0x31,0xFD);
	jdi_dcs_write_seq_static(ctx,0x32,0xFA);
	jdi_dcs_write_seq_static(ctx,0x33,0xF7);
	jdi_dcs_write_seq_static(ctx,0x34,0xF4);
	jdi_dcs_write_seq_static(ctx,0x35,0xF0);
	jdi_dcs_write_seq_static(ctx,0x36,0xED);
	jdi_dcs_write_seq_static(ctx,0x37,0xEC);
	jdi_dcs_write_seq_static(ctx,0x38,0xEB);
	jdi_dcs_write_seq_static(ctx,0x39,0xEA);
	jdi_dcs_write_seq_static(ctx,0x3A,0xE9);
	jdi_dcs_write_seq_static(ctx,0x3B,0xE8);
	jdi_dcs_write_seq_static(ctx,0x3D,0xE7);
	jdi_dcs_write_seq_static(ctx,0x3F,0xE6);
	jdi_dcs_write_seq_static(ctx,0x40,0xE5);
	jdi_dcs_write_seq_static(ctx,0x41,0xE4);
	//CABC_PWM_STILL
	jdi_dcs_write_seq_static(ctx,0x45,0xFF);
	jdi_dcs_write_seq_static(ctx,0x46,0xFA);
	jdi_dcs_write_seq_static(ctx,0x47,0xF2);
	jdi_dcs_write_seq_static(ctx,0x48,0xE8);
	jdi_dcs_write_seq_static(ctx,0x49,0xE4);
	jdi_dcs_write_seq_static(ctx,0x4A,0xDC);
	jdi_dcs_write_seq_static(ctx,0x4B,0xD7);
	jdi_dcs_write_seq_static(ctx,0x4C,0xD5);
	jdi_dcs_write_seq_static(ctx,0x4D,0xD3);
	jdi_dcs_write_seq_static(ctx,0x4E,0xD2);
	jdi_dcs_write_seq_static(ctx,0x4F,0xD0);
	jdi_dcs_write_seq_static(ctx,0x50,0xCE);
	jdi_dcs_write_seq_static(ctx,0x51,0xCD);
	jdi_dcs_write_seq_static(ctx,0x52,0xCB);
	jdi_dcs_write_seq_static(ctx,0x53,0xC6);
	jdi_dcs_write_seq_static(ctx,0x54,0xC3);

	jdi_dcs_write_seq_static(ctx, 0xA0,0x11);

	jdi_dcs_write_seq_static(ctx, 0xFF,0xF0);
	jdi_dcs_write_seq_static(ctx, 0xFB,0x01);
	jdi_dcs_write_seq_static(ctx, 0xD2,0x52);
//cabc end
	  //set 120hz
	jdi_dcs_write_seq_static(ctx, 0xFF, 0x25);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x18, 0x22);

	jdi_dcs_write_seq_static(ctx, 0xFF, 0x10);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0xB0, 0x01); //Enable MIPI CRC
	//DSC ON && set PPS
	jdi_dcs_write_seq_static(ctx, 0xC0, 0x03);
	jdi_dcs_write_seq_static(ctx, 0xC1, 0x89, 0x28, 0x00, 0x08, 0x00, 0xAA,
		0x02, 0x0E, 0x00, 0x2B, 0x00, 0x07, 0x0D, 0xB7,
		0x0C, 0xB7);
	jdi_dcs_write_seq_static(ctx, 0xC2, 0x1B, 0xA0);
/////////////////OSC 108.4M/////////////////////////////
	jdi_dcs_write_seq_static(ctx, 0xFF, 0x20);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x18, 0x66);
	jdi_dcs_write_seq_static(ctx, 0x1B, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x2F, 0x83);
	jdi_dcs_write_seq_static(ctx, 0xF2, 0x65);
	jdi_dcs_write_seq_static(ctx, 0xF3, 0x64);
	jdi_dcs_write_seq_static(ctx, 0xF4, 0x65);
	jdi_dcs_write_seq_static(ctx, 0xF5, 0x64);
	jdi_dcs_write_seq_static(ctx, 0xF6, 0x65);
	jdi_dcs_write_seq_static(ctx, 0xF7, 0x64);
	jdi_dcs_write_seq_static(ctx, 0xF8, 0x65);
	jdi_dcs_write_seq_static(ctx, 0xF9, 0x64);
	jdi_dcs_write_seq_static(ctx, 0xFF, 0x24);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0xC6, 0xC0);
	jdi_dcs_write_seq_static(ctx, 0xFF, 0x25);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x79, 0x7F);
	jdi_dcs_write_seq_static(ctx, 0xFF, 0x26);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x88, 0x04);
	jdi_dcs_write_seq_static(ctx, 0xFF, 0x27);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x21, 0xE7);
	jdi_dcs_write_seq_static(ctx, 0xE4, 0xDA);
	jdi_dcs_write_seq_static(ctx, 0xE6, 0x6D);
	jdi_dcs_write_seq_static(ctx, 0xFF, 0x2A);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x16, 0x9B);
	jdi_dcs_write_seq_static(ctx, 0x19, 0x0E);
	jdi_dcs_write_seq_static(ctx, 0x1A, 0x18);
	jdi_dcs_write_seq_static(ctx, 0x1E, 0x54);
	jdi_dcs_write_seq_static(ctx, 0x1F, 0x54);
	jdi_dcs_write_seq_static(ctx, 0x20, 0x54);
	jdi_dcs_write_seq_static(ctx, 0x28, 0xFD);
	jdi_dcs_write_seq_static(ctx, 0x2A, 0x50);
	jdi_dcs_write_seq_static(ctx, 0x2B, 0x03);
	jdi_dcs_write_seq_static(ctx, 0x31, 0x42);
	jdi_dcs_write_seq_static(ctx, 0x37, 0xFA);
	jdi_dcs_write_seq_static(ctx, 0x38, 0x2F);
	jdi_dcs_write_seq_static(ctx, 0x39, 0x72);
	jdi_dcs_write_seq_static(ctx, 0x4F, 0x9B);
	jdi_dcs_write_seq_static(ctx, 0x53, 0x18);
	jdi_dcs_write_seq_static(ctx, 0x57, 0x7E);
	jdi_dcs_write_seq_static(ctx, 0x58, 0x7E);
	jdi_dcs_write_seq_static(ctx, 0x59, 0x7E);
	jdi_dcs_write_seq_static(ctx, 0x61, 0xC8);
	jdi_dcs_write_seq_static(ctx, 0x63, 0xEE);
	jdi_dcs_write_seq_static(ctx, 0x64, 0x03);
	jdi_dcs_write_seq_static(ctx, 0x68, 0x87);
	jdi_dcs_write_seq_static(ctx, 0x6B, 0xCA);
	jdi_dcs_write_seq_static(ctx, 0x6C, 0x1D);
	jdi_dcs_write_seq_static(ctx, 0x6D, 0xE5);
	jdi_dcs_write_seq_static(ctx, 0x6E, 0xC7);
	jdi_dcs_write_seq_static(ctx, 0x6F, 0x1E);
	jdi_dcs_write_seq_static(ctx, 0x70, 0xE4);
	jdi_dcs_write_seq_static(ctx, 0x84, 0x9B);
	jdi_dcs_write_seq_static(ctx, 0x88, 0x18);
	jdi_dcs_write_seq_static(ctx, 0x8C, 0x3F);
	jdi_dcs_write_seq_static(ctx, 0x8D, 0x3F);
	jdi_dcs_write_seq_static(ctx, 0x8E, 0x3F);
	jdi_dcs_write_seq_static(ctx, 0x96, 0xFD);
	jdi_dcs_write_seq_static(ctx, 0x98, 0x98);
	jdi_dcs_write_seq_static(ctx, 0x9D, 0x20);
	jdi_dcs_write_seq_static(ctx, 0x9F, 0x64);
	jdi_dcs_write_seq_static(ctx, 0xA2, 0x3B);
	jdi_dcs_write_seq_static(ctx, 0xA4, 0xF9);
	jdi_dcs_write_seq_static(ctx, 0xA5, 0x3E);
	jdi_dcs_write_seq_static(ctx, 0xA6, 0xD6);
	jdi_dcs_write_seq_static(ctx, 0xFF, 0x2C);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x4F, 0x2E);
	jdi_dcs_write_seq_static(ctx, 0x9F, 0x01);
	jdi_dcs_write_seq_static(ctx, 0xFF, 0xE0);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x5A, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xFF, 0xD0);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x53, 0x22);
	jdi_dcs_write_seq_static(ctx, 0x54, 0x02);
	jdi_dcs_write_seq_static(ctx, 0xFF, 0xC0);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x9C, 0x11);
	jdi_dcs_write_seq_static(ctx, 0x9D, 0x11);
/////////////////////////////////////////////
	// CCMON
	jdi_dcs_write_seq_static(ctx, 0xFF, 0xF0);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x5A, 0x00);

//11bit
	jdi_dcs_write_seq_static(ctx, 0xFF,0x23);
	msleep(1);
	jdi_dcs_write_seq_static(ctx, 0xFB,0x01);
	jdi_dcs_write_seq_static(ctx, 0x00,0x68);
	jdi_dcs_write_seq_static(ctx, 0x07,0x00);
	jdi_dcs_write_seq_static(ctx, 0x08,0x01);
	jdi_dcs_write_seq_static(ctx, 0x09,0xC7);

		jdi_dcs_write_seq_static(ctx, 0xFF,0x2A);
	msleep(1);
	jdi_dcs_write_seq_static(ctx, 0xFB,0x01);
	jdi_dcs_write_seq_static(ctx, 0x7B,0x40);//add 2020/03/19
	//CCMOFF
	//CCMRUN
	jdi_dcs_write_seq_static(ctx, 0xFF,0XE0);
	jdi_dcs_write_seq_static(ctx, 0xFB,0X01);
	jdi_dcs_write_seq_static(ctx, 0x35,0X82);//VESA

	jdi_dcs_write_seq_static(ctx, 0xFF,0x10);
	jdi_dcs_write_seq_static(ctx, 0xFB,0x01);
	jdi_dcs_write_seq_static(ctx, 0x53,0x24);
	jdi_dcs_write_seq_static(ctx, 0x55,0x01);
	//open TE
	jdi_dcs_write_seq_static(ctx, 0x35,0x00);
	//end by zsq
	jdi_dcs_write_seq_static(ctx, 0x11);

	msleep(80);
	/* Display On*/
	jdi_dcs_write_seq_static(ctx, 0x29);
	msleep(10);
	pr_info("%s_420-\n", __func__);
}

static int jdi_disable(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	if (!ctx->enabled)
		return 0;
	pr_info("%s\n", __func__);
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int jdi_unprepare(struct drm_panel *panel)
{

	struct jdi *ctx = panel_to_jdi(panel);
	pr_info("%s\n", __func__);

	if (!ctx->prepared)
		return 0;

	jdi_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(60 * 1000, 60 * 1000);

/* #ifdef OPLUS_BUG_STABILITY */
    pr_info("%s: tp_gesture_enable_flag = %d \n", __func__, tp_gesture_enable_flag());
    if ((0 == tp_gesture_enable_flag())||(esd_flag == 1)) {
/* #endif */ /* OPLUS_BUG_STABILITY */
        ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->reset_gpio, 0);
        devm_gpiod_put(ctx->dev, ctx->reset_gpio);

        pr_info("%s: going to cut off power \n", __func__);
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
        //add for ldo
        ctx->bias_en = devm_gpiod_get(ctx->dev, "ldo", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->bias_en, 0);
        devm_gpiod_put(ctx->dev, ctx->bias_en);
/* #ifdef OPLUS_BUG_STABILITY */
        if( ctx->is_normal_mode ){
            switch_spi7cs_state(false);
        }
    }
/* #endif */ /* OPLUS_BUG_STABILITY */

	ctx->error = 0;
	ctx->prepared = false;

	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	int ret;

	pr_info("%s+\n", __func__);

	//add for ldo
	ctx->bias_en = devm_gpiod_get(ctx->dev, "ldo", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_en, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_en);
	udelay(2000);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(2000);
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	_lcm_i2c_write_bytes(0x0, 0xf);
	_lcm_i2c_write_bytes(0x1, 0xf);
	pr_info("%s-\n", __func__);
	return 0;
}
static int lcm_panel_poweroff(struct drm_panel *panel)
{
	return 0;
}

static int jdi_prepare(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	int ret;

	pr_info("%s+\n", __func__);
	if (ctx->prepared)
		return 0;


 /*	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	msleep(2);
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	_lcm_i2c_write_bytes(0x0, 0xf);
	_lcm_i2c_write_bytes(0x1, 0xf);
*/
	usleep_range(10 * 1000, 10 * 1000);
	//NVT H -> L -> H -> L -> H
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	usleep_range(5 * 1000, 5 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5 * 1000, 5 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(2 * 1000, 2 * 1000);
	jdi_vdd_tp_start(ctx);
	usleep_range(10 * 1000, 10 * 1000);
//end
	jdi_vdd_tp_end(ctx);
	if (is_export_project)
		jdi_export_panel_init(ctx);
	else
		jdi_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		jdi_unprepare(panel);

	ctx->prepared = true;

#ifdef PANEL_SUPPORT_READBACK
	jdi_panel_get_data(ctx);
#endif
/* #ifdef OPLUS_BUG_STABILITY */
	if(ctx->is_normal_mode) {
		switch_spi7cs_state(true);
		lcd_queue_load_tp_fw();
	}
/* #endif */ /* OPLUS_BUG_STABILITY */
	pr_info("%s-\n", __func__);
	return ret;
}

static int jdi_enable(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

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
	.clock = 478570,
	.hdisplay = 1080,
	.hsync_start = 1080 + 464,//HFP
	.hsync_end = 1080 + 464 + 20,//HSA
	.htotal = 1080 + 464 + 20 + 48,//HBP
	.vdisplay = 2400,
	.vsync_start = 2400 + 2460,//VFP
	.vsync_end = 2400 + 2460 + 10,//VSA
	.vtotal = 2400 + 2460 + 10 + 10,//VBP
	.vrefresh = 60,
};

static const struct drm_display_mode performance_mode = {
	.clock = 478618,
	.hdisplay = 1080,
	.hsync_start = 1080 + 464,//HFP
	.hsync_end = 1080 + 464 + 20,//HSA
	.htotal = 1080 + 464 + 20 + 48,//HBP
	.vdisplay = 2400,
	.vsync_start = 2400 + 846,//VFP
	.vsync_end = 2400 + 846 + 10,//VSA
	.vtotal = 2400 + 846 + 10 + 10,//VBP
	.vrefresh = 90,
};

static const struct drm_display_mode performance_mode1 = {
	.clock = 478570,
	.hdisplay = 1080,
	.hsync_start = 1080 + 464,//HFP
	.hsync_end = 1080 + 464 + 20,//HSA
	.htotal = 1080 + 464 + 20 + 48,//HBP
	.vdisplay = 2400,
	.vsync_start = 2400 + 54,//VFP
	.vsync_end = 2400 + 54 + 10,//VSA
	.vtotal = 2400 + 54 + 10 + 10,//VBP
	.vrefresh = 120,
};


#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.pll_clk = 464,
	//.vfp_low_power = 4178,//45hz
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9C,
	},
	.is_cphy = 1,
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
	.data_rate = 928,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 120,
	},
	.dyn = {
		.switch_en = 1,
		.pll_clk = 428,
		.hfp = 396,
		.vfp = 2460,
		.data_rate = 856,
	},
	.vendor = "NT36672C_JDI",
	.manufacture = "nt2048",
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 464,
	.vfp_low_power = 2460,//60hz
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {

	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9C,
	},
	.is_cphy = 1,
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
	.data_rate = 928,
	.dyn_fps = {
		.switch_en = 1,
		.switch_en = 1, .vact_timing_fps = 120,
	},
	.dyn = {
		.switch_en = 1,
		.pll_clk = 428,
		.hfp = 396,
		.vfp = 846,
		.data_rate = 856,
	},
	.vendor = "NT36672C_JDI",
	.manufacture = "nt2048",
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static struct mtk_panel_params ext_params_120hz = {
	.pll_clk = 464,
	.vfp_low_power = 2460,//idle 60hz
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9C,
	},
	.is_cphy = 1,
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
	.data_rate = 928,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 120,
	},
	.dyn = {
		.switch_en = 1,
		.pll_clk = 428,
		.vfp_lp_dyn = 2460,
		.hfp = 396,
		.vfp = 54,
		.data_rate = 856,
	},
	.vendor = "NT36672C_JDI",
	.manufacture = "nt2048",
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static int jdi_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0x07, 0xff, 0x00};
	if (level > 2047)
		level = 2047;
	pr_err("%s backlight = %d\n", __func__, level);
	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;
	esd_brightness = level;
	if (!cb)
		return -1;

	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	return 0;
}

static int oplus_esd_backlight_check(void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0x51, 0x07, 0xff, 0x00};

	pr_err("%s esd_backlight = %d\n", __func__, esd_brightness);
	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
	if (!cb)
		return -1;
	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 1;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	pr_err("%s ,%d\n", __func__,mode);
	if (mode == 0)
		ext->params = &ext_params;
	else if (mode == 2)
		ext->params = &ext_params_90hz;
	else if (mode == 1)
		ext->params = &ext_params_120hz;
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
	else if (mode == 2)
		ext_para = &ext_params_90hz;
	else if (mode == 1)
		ext_para = &ext_params_120hz;
	else
		ret = 1;

	return ret;

}

static void mode_switch_60_to_90(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct jdi *ctx = panel_to_jdi(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		/* display off */
		jdi_dcs_write_seq_static(ctx, 0x28);
		jdi_dcs_write_seq_static(ctx, 0x10);
		usleep_range(200 * 1000, 230 * 1000);
	} else if (stage == AFTER_DSI_POWERON) {
		/* set PLL to 285M */
		jdi_dcs_write_seq_static(ctx, 0xB0, 0x00);
		jdi_dcs_write_seq_static(ctx, 0xB6, 0x59, 0x00, 0x06, 0x23,
			0x8A, 0x13, 0x1A, 0x05, 0x04, 0xFA, 0x05, 0x20);

		/* switch to 90hz */
		jdi_dcs_write_seq_static(ctx, 0xB0, 0x04);
		jdi_dcs_write_seq_static(ctx, 0xF1, 0x2A);
		jdi_dcs_write_seq_static(ctx, 0xC1, 0x0C);
		jdi_dcs_write_seq_static(ctx, 0xC2, 0x09, 0x24, 0x0E,
			0x00, 0x00, 0x0E);
		jdi_dcs_write_seq_static(ctx, 0xC1, 0x94, 0x42, 0x00,
			0x16, 0x05);
		jdi_dcs_write_seq_static(ctx, 0xCF, 0x64, 0x0B, 0x00,
			0x5E, 0x00, 0xC3, 0x02, 0x4B, 0x0B, 0x77, 0x0B,
			0x8B, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
			0x02, 0x02, 0x03, 0x03, 0x03, 0x00, 0x9B, 0x00,
			0xB9, 0x00, 0xB9, 0x00, 0xB9, 0x00, 0xB9, 0x00,
			0xB9, 0x00, 0xB9, 0x03, 0x98, 0x03, 0x98, 0x03,
			0x98, 0x03, 0x98, 0x03, 0x98, 0x00, 0x9B, 0x00,
			0xB9, 0x00, 0xB9, 0x00, 0xB9, 0x00, 0xB9, 0x00,
			0xB9, 0x00, 0xB9, 0x03, 0x98, 0x03, 0x98, 0x03,
			0x98, 0x03, 0x98, 0x03, 0x98, 0x01, 0x42, 0x01,
			0x42, 0x01, 0x42, 0x01, 0x42, 0x01, 0x42, 0x01,
			0x42, 0x01, 0x42, 0x01, 0x42, 0x01, 0x42, 0x01,
			0x42, 0x01, 0x42, 0x01, 0x42, 0x1C, 0x1C, 0x1C,
			0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C,
			0x1C, 0x00, 0x90, 0x01, 0x9A, 0x01, 0x9A, 0x03,
			0x33, 0x03, 0x33, 0x09, 0xA4, 0x09, 0xA4, 0x09,
			0xA4, 0x09, 0xA4, 0x09, 0xA4, 0x09, 0xA4, 0x0F,
			0x88, 0x19);
		jdi_dcs_write_seq_static(ctx, 0xC4, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x02, 0x00, 0x00, 0x00, 0x37, 0x00, 0x01);
		jdi_dcs_write_seq_static(ctx, 0xD7, 0x00, 0x69, 0x34,
			0x00, 0xa0, 0x0a, 0x00, 0x00, 0x3c);
		jdi_dcs_write_seq_static(ctx, 0xD8, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00,
			0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x05,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x11, 0x00, 0x34, 0x00, 0x00, 0x00, 0x18,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
		jdi_dcs_write_seq_static(ctx, 0xDF, 0x50, 0x42, 0x58,
			0x81, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x6B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE6, 0x0E, 0xFF, 0xD4, 0x0E, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x0F, 0x00, 0x53, 0x18, 0xE5,
			0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
		jdi_dcs_write_seq_static(ctx, 0xB0, 0x04);
		jdi_dcs_write_seq_static(ctx, 0xBB, 0x59, 0xC8, 0xC8,
			0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0x4A,
			0x48, 0x46, 0x44, 0x42, 0x40, 0x3E, 0x3C, 0x3A,
			0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0x04, 0x00, 0x02, 0x02, 0x00, 0x04,
			0x69, 0x5A, 0x00, 0x0B, 0x76, 0x0F, 0xFF, 0x0F,
			0xFF, 0x0F, 0xFF, 0x14, 0x81, 0xF4);
		jdi_dcs_write_seq_static(ctx, 0xE8, 0x00, 0x02);
		jdi_dcs_write_seq_static(ctx, 0xE4, 0x00, 0x35);
		jdi_dcs_write_seq_static(ctx, 0xB0, 0x84);
		jdi_dcs_write_seq_static(ctx, 0xE4, 0x33, 0xB4, 0x00,
			0x00, 0x00, 0x58, 0x04, 0x04, 0x9A);
		jdi_dcs_write_seq_static(ctx, 0xD4, 0x93, 0x93, 0x60,
			0x1E, 0xE1, 0x02, 0x08, 0x00, 0x00, 0x02, 0x22,
			0x02, 0xEC, 0x03, 0x83, 0x04, 0x00, 0x04, 0x00,
			0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x01, 0x00,
			0x01, 0x00);

		/* display on */
		jdi_dcs_write_seq_static(ctx, 0x29);
		usleep_range(120 * 1000, 150 * 1000);
		jdi_dcs_write_seq_static(ctx, 0x11);
		usleep_range(200 * 1000, 230 * 1000);
	}
}

static void mode_switch_90_to_60(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct jdi *ctx = panel_to_jdi(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		/* display off */
		jdi_dcs_write_seq_static(ctx, 0x28);
		jdi_dcs_write_seq_static(ctx, 0x10);
		usleep_range(200 * 1000, 230 * 1000);
	} else if (stage == AFTER_DSI_POWERON) {
		/* set PLL to 190M */
		jdi_dcs_write_seq_static(ctx, 0xB0, 0x00);
		jdi_dcs_write_seq_static(ctx, 0xB6, 0x51, 0x00, 0x06, 0x23,
			0x8A, 0x13, 0x1A, 0x05, 0x04, 0xFA, 0x05, 0x20);

		/* switch to 60hz */
		jdi_dcs_write_seq_static(ctx, 0xB0, 0x04);
		jdi_dcs_write_seq_static(ctx, 0xF1, 0x2A);
		jdi_dcs_write_seq_static(ctx, 0xC1, 0x0C);
		jdi_dcs_write_seq_static(ctx, 0xC2, 0x09, 0x24, 0x0E,
			0x00, 0x00, 0x0E);
		jdi_dcs_write_seq_static(ctx, 0xC1, 0x94, 0x42, 0x00,
			0x16, 0x05);
		jdi_dcs_write_seq_static(ctx, 0xCF, 0x64, 0x0B, 0x00,
			0x5E, 0x00, 0xC3, 0x02, 0x4B, 0x0B, 0x77, 0x0B,
			0x8B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x9B, 0x00,
			0xB9, 0x00, 0xB9, 0x00, 0xB9, 0x00, 0xB9, 0x00,
			0xB9, 0x00, 0xB9, 0x03, 0x98, 0x03, 0x98, 0x03,
			0x98, 0x03, 0x98, 0x03, 0x98, 0x00, 0x9B, 0x00,
			0xB9, 0x00, 0xB9, 0x00, 0xB9, 0x00, 0xB9, 0x00,
			0xB9, 0x00, 0xB9, 0x03, 0x98, 0x03, 0x98, 0x03,
			0x98, 0x03, 0x98, 0x03, 0x98, 0x01, 0x42, 0x01,
			0x42, 0x01, 0x42, 0x01, 0x42, 0x01, 0x42, 0x01,
			0x42, 0x01, 0x42, 0x01, 0x42, 0x01, 0x42, 0x01,
			0x42, 0x01, 0x42, 0x01, 0x42, 0x1C, 0x1C, 0x1C,
			0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C,
			0x1C, 0x00, 0x90, 0x01, 0x9A, 0x01, 0x9A, 0x03,
			0x33, 0x03, 0x33, 0x09, 0xA4, 0x09, 0xA4, 0x09,
			0xA4, 0x09, 0xA4, 0x09, 0xA4, 0x09, 0xA4, 0x0F,
			0x88, 0x19);
		jdi_dcs_write_seq_static(ctx, 0xC4, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x02, 0x00, 0x00, 0x00, 0x50, 0x00, 0x01);
		jdi_dcs_write_seq_static(ctx, 0xD7, 0x00, 0x69, 0x34,
			0x00, 0xa0, 0x0a, 0x00, 0x00, 0x5a);
		jdi_dcs_write_seq_static(ctx, 0xD8, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a, 0x00,
			0x5a, 0x00, 0x5a, 0x00, 0x5a, 0x00, 0x5a, 0x05,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x0A, 0x50, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x1A, 0x00, 0x4E, 0x00, 0x00, 0x00, 0x24,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
		jdi_dcs_write_seq_static(ctx, 0xDF, 0x50, 0x42, 0x58,
			0x81, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x6B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x10, 0x00, 0xFF, 0xD4, 0x0E, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x0F, 0x00, 0x53, 0x18, 0x0F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
		jdi_dcs_write_seq_static(ctx, 0xB0, 0x04);
		jdi_dcs_write_seq_static(ctx, 0xBB, 0x59, 0xC8, 0xC8,
			0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8, 0x4A,
			0x48, 0x46, 0x44, 0x42, 0x40, 0x3E, 0x3C, 0x3A,
			0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0x04, 0x00, 0x01, 0x01, 0x00, 0x04,
			0x69, 0x5A, 0x00, 0x0B, 0x76, 0x0F, 0xFF, 0x0F,
			0xFF, 0x0F, 0xFF, 0x14, 0x81, 0xF4);
		jdi_dcs_write_seq_static(ctx, 0xE8, 0x00, 0x02);
		jdi_dcs_write_seq_static(ctx, 0xE4, 0x00, 0x35);
		jdi_dcs_write_seq_static(ctx, 0xB0, 0x84);
		jdi_dcs_write_seq_static(ctx, 0xE4, 0x33, 0xB4, 0x00,
			0x00, 0x00, 0x58, 0x04, 0x00, 0x00);
		jdi_dcs_write_seq_static(ctx, 0xD4, 0x93, 0x93, 0x60,
			0x1E, 0xE1, 0x02, 0x08, 0x00, 0x00, 0x02, 0x22,
			0x02, 0xEC, 0x03, 0x83, 0x04, 0x00, 0x04, 0x00,
			0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x01, 0x00,
			0x01, 0x00);

		/* display on */
		jdi_dcs_write_seq_static(ctx, 0x29);
		usleep_range(120 * 1000, 150 * 1000);
		jdi_dcs_write_seq_static(ctx, 0x11);
		usleep_range(200 * 1000, 230 * 1000);
	}
}

static int mode_switch(struct drm_panel *panel, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;

	if (cur_mode == 0 && dst_mode == 1) { /* 60 switch to 90 */
		mode_switch_60_to_90(panel, stage);
	} else if (cur_mode == 1 && dst_mode == 0) { /* 90 switch to 60 */
		mode_switch_90_to_60(panel, stage);
	} else
		ret = 1;

	return ret;
}

static void cabc_switch(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int cabc_mode)
{
	char bl_tb0[] = {0x55, 0x00};
	char bl_tb1[] = {0xFF, 0x10};
	char bl_tb2[] = {0xFB, 0x01};
	char bl_tb3[] = {0x53, 0x2C};
	char bl_tb4[] = {0x03, 0xC0};
	char bl_tb5[] = {0x03, 0x00};
	char bl_tb6[] = {0xFF, 0x23};
	if(cabc_mode == 2)
		return;
	if(cabc_mode == 3)
		cabc_mode = 2;

	pr_err("%s cabc = %d\n", __func__, cabc_mode);
	bl_tb0[1] = (u8)cabc_mode;

	cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));//FF  23
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));//FB  01
	if(cabc_mode == 0)
		cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));//03  C0
	else
		cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));//03  00
	usleep_range(5 * 1000, 5 * 1000);
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));//FF 10
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));//FB 01
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));//FB 01
	if(cabc_mode != 0){
		cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));//55 0X
	}
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct jdi *ctx = panel_to_jdi(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = jdi_setbacklight_cmdq,
	/* .esd_backlight_check = oplus_esd_backlight_check, */
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.mode_switch = mode_switch,
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
	struct drm_display_mode *mode3;

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

	mode3 = drm_mode_duplicate(panel->drm, &performance_mode1);
	if (!mode3) {
		dev_info(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			 performance_mode1.hdisplay, performance_mode1.vdisplay,
			 performance_mode1.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode3);
	mode3->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode3);

	panel->connector->display_info.width_mm = 69;
	panel->connector->display_info.height_mm = 152;

	return 1;
}

static const struct drm_panel_funcs jdi_drm_funcs = {
	.disable = jdi_disable,
	.unprepare = jdi_unprepare,
	.prepare = jdi_prepare,
	.enable = jdi_enable,
	.get_modes = jdi_get_modes,
};

static int jdi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct jdi *ctx;
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
	pr_info("%s+\n", __func__);
	ctx = devm_kzalloc(dev, sizeof(struct jdi), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 3;
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
	register_device_proc("lcd", "NT36672C_JDI", "nt2048");
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
/* #ifdef OPLUS_BUG_STABILITY */
    ctx->is_normal_mode = true;
    if( META_BOOT == get_boot_mode() || FACTORY_BOOT == get_boot_mode() )
        ctx->is_normal_mode = false;
    pr_info("%s: is_normal_mode = %d \n", __func__, ctx->is_normal_mode);
/* #endif */ /* OPLUS_BUG_STABILITY */

	if (get_project() == 19420)
		is_export_project = 1;
	pr_info("%s-\n", __func__);

	return ret;
}

static int jdi_remove(struct mipi_dsi_device *dsi)
{
	struct jdi *ctx = mipi_dsi_get_drvdata(dsi);
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
		.compatible = "jdi,nt36672c,cphy,vdo",
	},
	{} };

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
	.probe = jdi_probe,
	.remove = jdi_remove,
	.driver = {

			.name = "panel-jdi-nt36672c-cphy-vdo",
			.owner = THIS_MODULE,
			.of_match_table = jdi_of_match,
		},
};

module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("Elon Hsu <elon.hsu@mediatek.com>");
MODULE_DESCRIPTION("jdi r66451 CMD AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
