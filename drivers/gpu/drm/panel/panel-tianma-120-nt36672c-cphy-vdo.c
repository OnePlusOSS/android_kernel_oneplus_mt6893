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

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <soc/oplus/device_info.h>
#include <linux/of_graph.h>
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
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/oplus19131_data_hw_roundedpattern.h"
#endif

/* #ifdef OPLUS_BUG_STABILITY */
#include <mt-plat/mtk_boot_common.h>
/* #endif */ /* OPLUS_BUG_STABILITY */

#define AVDD_REG 0x00
#define AVDD_REG 0x01
#define MAX_NORMAL_BRIGHTNESS   2047

/* i2c control start */
#define LCM_I2C_ID_NAME "I2C_LCD_BIAS"
static struct i2c_client *_lcm_i2c_client;
extern unsigned int get_project(void);
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

    bool is_normal_mode;
};

#define tianma_dcs_write_seq(ctx, seq...)                                     \
	({                                                                     \
		const u8 d[] = {seq};                                          \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		tianma_dcs_write(ctx, d, ARRAY_SIZE(d));                      \
	})

#define tianma_dcs_write_seq_static(ctx, seq...)                              \
	({                                                                     \
		static const u8 d[] = {seq};                                   \
		tianma_dcs_write(ctx, d, ARRAY_SIZE(d));                      \
	})

static inline struct tianma *panel_to_tianma(struct drm_panel *panel)
{
	return container_of(panel, struct tianma, panel);
}

#ifdef PANEL_SUPPORT_READBACK
static int tianma_dcs_read(struct tianma *ctx, u8 cmd, void *data, size_t len)
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

static void tianma_panel_get_data(struct tianma *ctx)
{
	u8 buffer[3] = {0};
	static int ret;
	pr_info("%s+\n", __func__);

	if (ret == 0) {
		ret = tianma_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("%s  0x%08x\n", __func__,buffer[0] | (buffer[1] << 8));
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static void tianma_dcs_write(struct tianma *ctx, const void *data, size_t len)
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
		pr_notice("error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static void tianma_vdd_tp_start(struct tianma *ctx)
{
	tianma_dcs_write_seq_static(ctx, 0xFF,0xC0);
	tianma_dcs_write_seq_static(ctx, 0X4B,0x1F);
}

static void tianma_vdd_tp_end(struct tianma *ctx)
{
	tianma_dcs_write_seq_static(ctx, 0xFF,0xC0);
	tianma_dcs_write_seq_static(ctx, 0X4B,0x0E);
}

static void tianma_panel_init(struct tianma *ctx)
{
/* #ifdef OPLUS_BUG_STABILITY */
//add for cabc
	tianma_dcs_write_seq_static(ctx, 0xFF,0x23);
	tianma_dcs_write_seq_static(ctx, 0XFB,0x01);

	tianma_dcs_write_seq_static(ctx, 0x05,0x22);
	tianma_dcs_write_seq_static(ctx, 0x06,0x01);
	tianma_dcs_write_seq_static(ctx, 0x08,0x07);
	tianma_dcs_write_seq_static(ctx, 0x09,0x04);
	tianma_dcs_write_seq_static(ctx, 0x10,0x82);
	tianma_dcs_write_seq_static(ctx, 0x11,0x01);
	tianma_dcs_write_seq_static(ctx, 0x12,0x95);
	tianma_dcs_write_seq_static(ctx, 0x15,0x68);
	tianma_dcs_write_seq_static(ctx, 0x16,0x0B);
	//CABC_PWM_UI
	tianma_dcs_write_seq_static(ctx,0x30,0xFF);
	tianma_dcs_write_seq_static(ctx,0x31,0xFD);
	tianma_dcs_write_seq_static(ctx,0x32,0xFA);
	tianma_dcs_write_seq_static(ctx,0x33,0xF7);
	tianma_dcs_write_seq_static(ctx,0x34,0xF4);
	tianma_dcs_write_seq_static(ctx,0x35,0xF0);
	tianma_dcs_write_seq_static(ctx,0x36,0xED);
	tianma_dcs_write_seq_static(ctx,0x37,0xEC);
	tianma_dcs_write_seq_static(ctx,0x38,0xEB);
	tianma_dcs_write_seq_static(ctx,0x39,0xEA);
	tianma_dcs_write_seq_static(ctx,0x3A,0xE9);
	tianma_dcs_write_seq_static(ctx,0x3B,0xE8);
	tianma_dcs_write_seq_static(ctx,0x3D,0xE7);
	tianma_dcs_write_seq_static(ctx,0x3F,0xE6);
	tianma_dcs_write_seq_static(ctx,0x40,0xE5);
	tianma_dcs_write_seq_static(ctx,0x41,0xE4);
	//CABC_PWM_STILL
	tianma_dcs_write_seq_static(ctx,0x45,0xFF);
	tianma_dcs_write_seq_static(ctx,0x46,0xFA);
	tianma_dcs_write_seq_static(ctx,0x47,0xF2);
	tianma_dcs_write_seq_static(ctx,0x48,0xE8);
	tianma_dcs_write_seq_static(ctx,0x49,0xE4);
	tianma_dcs_write_seq_static(ctx,0x4A,0xDC);
	tianma_dcs_write_seq_static(ctx,0x4B,0xD7);
	tianma_dcs_write_seq_static(ctx,0x4C,0xD5);
	tianma_dcs_write_seq_static(ctx,0x4D,0xD3);
	tianma_dcs_write_seq_static(ctx,0x4E,0xD2);
	tianma_dcs_write_seq_static(ctx,0x4F,0xD0);
	tianma_dcs_write_seq_static(ctx,0x50,0xCE);
	tianma_dcs_write_seq_static(ctx,0x51,0xCD);
	tianma_dcs_write_seq_static(ctx,0x52,0xCB);
	tianma_dcs_write_seq_static(ctx,0x53,0xC6);
	tianma_dcs_write_seq_static(ctx,0x54,0xC3);

	tianma_dcs_write_seq_static(ctx, 0xA0,0x11);

	tianma_dcs_write_seq_static(ctx, 0xFF,0xF0);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0xD2,0x52);
/* #endif */ /* OPLUS_BUG_STABILITY */

	tianma_dcs_write_seq_static(ctx, 0xFF,0x25);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0x18,0x22);

	tianma_dcs_write_seq_static(ctx, 0xFF, 0x10);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	//DSC ON && set PPS
	tianma_dcs_write_seq_static(ctx, 0xC0, 0x03);
	tianma_dcs_write_seq_static(ctx, 0xC1, 0x89, 0x28, 0x00, 0x08, 0x00, 0xAA,
				0x02, 0x0E, 0x00, 0x2B, 0x00, 0x07, 0x0D, 0xB7,
				0x0C, 0xB7);
	tianma_dcs_write_seq_static(ctx, 0xC2, 0x1B, 0xA0);

	tianma_dcs_write_seq_static(ctx, 0xFF,0xC0);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0x9C,0x11);
	tianma_dcs_write_seq_static(ctx, 0x9D,0x11);

	tianma_dcs_write_seq_static(ctx, 0xFF,0xD0);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0x53,0x22);
	tianma_dcs_write_seq_static(ctx, 0x54,0x02);

	tianma_dcs_write_seq_static(ctx, 0xFF,0xE0);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0x35,0x82);
	
	tianma_dcs_write_seq_static(ctx, 0xFF,0xF0);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0xA0,0x08);

	tianma_dcs_write_seq_static(ctx, 0xFF,0x10);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0x3B,0x03,0X14,0X36,0X04,0X04);
	tianma_dcs_write_seq_static(ctx, 0xB0,0x00);
	tianma_dcs_write_seq_static(ctx, 0xC0,0x03);
//add update
	tianma_dcs_write_seq_static(ctx, 0XFF, 0X24);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X10, 0X8B);
	tianma_dcs_write_seq_static(ctx, 0X11, 0X8C);
	tianma_dcs_write_seq_static(ctx, 0X28, 0X8B);
	tianma_dcs_write_seq_static(ctx, 0X29, 0X8C);
	tianma_dcs_write_seq_static(ctx, 0X33, 0X03);
	tianma_dcs_write_seq_static(ctx, 0X35, 0X02);
	tianma_dcs_write_seq_static(ctx, 0X36, 0X27);
	tianma_dcs_write_seq_static(ctx, 0X4E, 0X51);
	tianma_dcs_write_seq_static(ctx, 0X4F, 0X51);
	tianma_dcs_write_seq_static(ctx, 0X53, 0X51);

	tianma_dcs_write_seq_static(ctx, 0XFF, 0X25);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X66, 0X40);
	tianma_dcs_write_seq_static(ctx, 0X67, 0X29);
	tianma_dcs_write_seq_static(ctx, 0XF0, 0X05);

	tianma_dcs_write_seq_static(ctx, 0XFF, 0X26);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X01, 0XF8);
	tianma_dcs_write_seq_static(ctx, 0X04, 0XF8);
	tianma_dcs_write_seq_static(ctx, 0X05, 0X08);
	tianma_dcs_write_seq_static(ctx, 0X06, 0X16);
	tianma_dcs_write_seq_static(ctx, 0X08, 0X16);
	tianma_dcs_write_seq_static(ctx, 0X9A, 0X81);
	tianma_dcs_write_seq_static(ctx, 0X9B, 0X03);

	tianma_dcs_write_seq_static(ctx, 0XFF, 0X27);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X88, 0X03);

	tianma_dcs_write_seq_static(ctx, 0XFF, 0X2A);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X16, 0X86);
	tianma_dcs_write_seq_static(ctx, 0X1A, 0XFF);
	tianma_dcs_write_seq_static(ctx, 0X4F, 0X86);
	tianma_dcs_write_seq_static(ctx, 0X53, 0XFF);
	tianma_dcs_write_seq_static(ctx, 0X84, 0X86);
	tianma_dcs_write_seq_static(ctx, 0X88, 0XFF);

	tianma_dcs_write_seq_static(ctx, 0XFF, 0X2C);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X30, 0XF8);
	tianma_dcs_write_seq_static(ctx, 0X33, 0XF8);
	tianma_dcs_write_seq_static(ctx, 0X35, 0X21);
	tianma_dcs_write_seq_static(ctx, 0X37, 0X21);
	tianma_dcs_write_seq_static(ctx, 0X61, 0X02);
	tianma_dcs_write_seq_static(ctx, 0X62, 0X27);
	tianma_dcs_write_seq_static(ctx, 0X6B, 0X3B);
	tianma_dcs_write_seq_static(ctx, 0X6C, 0X3B);
	tianma_dcs_write_seq_static(ctx, 0X6D, 0X3B);
	tianma_dcs_write_seq_static(ctx, 0X82, 0XF8);
	tianma_dcs_write_seq_static(ctx, 0X85, 0XF8);
	tianma_dcs_write_seq_static(ctx, 0X87, 0X11);
	tianma_dcs_write_seq_static(ctx, 0X89, 0X11);
//end
//11bit
	tianma_dcs_write_seq_static(ctx, 0xFF,0x23);
	msleep(1);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0x00,0x68);
	tianma_dcs_write_seq_static(ctx, 0x07,0x00);
	tianma_dcs_write_seq_static(ctx, 0x08,0x01);
	tianma_dcs_write_seq_static(ctx, 0x09,0xC7);

	tianma_dcs_write_seq_static(ctx, 0xFF,0x10);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0x53,0x24);
	tianma_dcs_write_seq_static(ctx, 0x55,0x01);
    //open TE
	tianma_dcs_write_seq_static(ctx, 0x35,0x00);

        tianma_dcs_write_seq_static(ctx, 0xFF,0xF0);
        tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
        tianma_dcs_write_seq_static(ctx, 0x5A,0x00);
        tianma_dcs_write_seq_static(ctx, 0xFF,0x10);
        tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
// endif

	tianma_dcs_write_seq_static(ctx, 0x11);
	msleep(80);
	//Display On
	tianma_dcs_write_seq_static(ctx, 0x29);
	msleep(10);
	pr_info("%s-\n", __func__);
}

static void tianma_export_panel_init(struct tianma *ctx)
{
/* #ifdef OPLUS_BUG_STABILITY */
//add for cabc
	tianma_dcs_write_seq_static(ctx, 0xFF,0x23);
	tianma_dcs_write_seq_static(ctx, 0XFB,0x01);

	tianma_dcs_write_seq_static(ctx, 0x05,0x22);
	tianma_dcs_write_seq_static(ctx, 0x06,0x01);
	tianma_dcs_write_seq_static(ctx, 0x08,0x07);
	tianma_dcs_write_seq_static(ctx, 0x09,0x04);
	tianma_dcs_write_seq_static(ctx, 0x10,0x82);
	tianma_dcs_write_seq_static(ctx, 0x11,0x01);
	tianma_dcs_write_seq_static(ctx, 0x12,0x95);
	tianma_dcs_write_seq_static(ctx, 0x15,0x68);
	tianma_dcs_write_seq_static(ctx, 0x16,0x0B);
	//CABC_PWM_UI
	tianma_dcs_write_seq_static(ctx,0x30,0xFF);
	tianma_dcs_write_seq_static(ctx,0x31,0xFD);
	tianma_dcs_write_seq_static(ctx,0x32,0xFA);
	tianma_dcs_write_seq_static(ctx,0x33,0xF7);
	tianma_dcs_write_seq_static(ctx,0x34,0xF4);
	tianma_dcs_write_seq_static(ctx,0x35,0xF0);
	tianma_dcs_write_seq_static(ctx,0x36,0xED);
	tianma_dcs_write_seq_static(ctx,0x37,0xEC);
	tianma_dcs_write_seq_static(ctx,0x38,0xEB);
	tianma_dcs_write_seq_static(ctx,0x39,0xEA);
	tianma_dcs_write_seq_static(ctx,0x3A,0xE9);
	tianma_dcs_write_seq_static(ctx,0x3B,0xE8);
	tianma_dcs_write_seq_static(ctx,0x3D,0xE7);
	tianma_dcs_write_seq_static(ctx,0x3F,0xE6);
	tianma_dcs_write_seq_static(ctx,0x40,0xE5);
	tianma_dcs_write_seq_static(ctx,0x41,0xE4);
	//CABC_PWM_STILL
	tianma_dcs_write_seq_static(ctx,0x45,0xFF);
	tianma_dcs_write_seq_static(ctx,0x46,0xFA);
	tianma_dcs_write_seq_static(ctx,0x47,0xF2);
	tianma_dcs_write_seq_static(ctx,0x48,0xE8);
	tianma_dcs_write_seq_static(ctx,0x49,0xE4);
	tianma_dcs_write_seq_static(ctx,0x4A,0xDC);
	tianma_dcs_write_seq_static(ctx,0x4B,0xD7);
	tianma_dcs_write_seq_static(ctx,0x4C,0xD5);
	tianma_dcs_write_seq_static(ctx,0x4D,0xD3);
	tianma_dcs_write_seq_static(ctx,0x4E,0xD2);
	tianma_dcs_write_seq_static(ctx,0x4F,0xD0);
	tianma_dcs_write_seq_static(ctx,0x50,0xCE);
	tianma_dcs_write_seq_static(ctx,0x51,0xCD);
	tianma_dcs_write_seq_static(ctx,0x52,0xCB);
	tianma_dcs_write_seq_static(ctx,0x53,0xC6);
	tianma_dcs_write_seq_static(ctx,0x54,0xC3);

	tianma_dcs_write_seq_static(ctx, 0xA0,0x11);

	tianma_dcs_write_seq_static(ctx, 0xFF,0xF0);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0xD2,0x52);
/* #endif */ /* OPLUS_BUG_STABILITY */

	tianma_dcs_write_seq_static(ctx, 0xFF,0x25);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0x18,0x22);

	tianma_dcs_write_seq_static(ctx, 0xFF, 0x10);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	//DSC ON && set PPS
	tianma_dcs_write_seq_static(ctx, 0xC0, 0x03);
	tianma_dcs_write_seq_static(ctx, 0xC1, 0x89, 0x28, 0x00, 0x08, 0x00, 0xAA,
				0x02, 0x0E, 0x00, 0x2B, 0x00, 0x07, 0x0D, 0xB7,
				0x0C, 0xB7);
	tianma_dcs_write_seq_static(ctx, 0xC2, 0x1B, 0xA0);

	tianma_dcs_write_seq_static(ctx, 0xFF,0xC0);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0x9C,0x11);
	tianma_dcs_write_seq_static(ctx, 0x9D,0x11);

	tianma_dcs_write_seq_static(ctx, 0xFF,0xD0);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0x53,0x22);
	tianma_dcs_write_seq_static(ctx, 0x54,0x02);

	tianma_dcs_write_seq_static(ctx, 0xFF,0xE0);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0x35,0x82);

	tianma_dcs_write_seq_static(ctx, 0xFF,0xF0);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0xA0,0x08);

	tianma_dcs_write_seq_static(ctx, 0xFF,0x10);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0x3B,0x03,0X14,0X36,0X04,0X04);
	tianma_dcs_write_seq_static(ctx, 0xB0,0x00);
	tianma_dcs_write_seq_static(ctx, 0xC0,0x03);
//add update
	tianma_dcs_write_seq_static(ctx, 0XFF, 0X24);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X10, 0X8B);
	tianma_dcs_write_seq_static(ctx, 0X11, 0X8C);
	tianma_dcs_write_seq_static(ctx, 0X28, 0X8B);
	tianma_dcs_write_seq_static(ctx, 0X29, 0X8C);
	tianma_dcs_write_seq_static(ctx, 0X33, 0X03);
	tianma_dcs_write_seq_static(ctx, 0X35, 0X02);
	tianma_dcs_write_seq_static(ctx, 0X36, 0X27);
	tianma_dcs_write_seq_static(ctx, 0X4E, 0X51);
	tianma_dcs_write_seq_static(ctx, 0X4F, 0X51);
	tianma_dcs_write_seq_static(ctx, 0X53, 0X51);

	tianma_dcs_write_seq_static(ctx, 0XFF, 0X25);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X66, 0X40);
	tianma_dcs_write_seq_static(ctx, 0X67, 0X29);
	tianma_dcs_write_seq_static(ctx, 0XF0, 0X05);

	tianma_dcs_write_seq_static(ctx, 0XFF, 0X26);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X01, 0XF8);
	tianma_dcs_write_seq_static(ctx, 0X04, 0XF8);
	tianma_dcs_write_seq_static(ctx, 0X05, 0X08);
	tianma_dcs_write_seq_static(ctx, 0X06, 0X16);
	tianma_dcs_write_seq_static(ctx, 0X08, 0X16);
	tianma_dcs_write_seq_static(ctx, 0X9A, 0X81);
	tianma_dcs_write_seq_static(ctx, 0X9B, 0X03);

	tianma_dcs_write_seq_static(ctx, 0XFF, 0X27);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X88, 0X03);

	tianma_dcs_write_seq_static(ctx, 0XFF, 0X2A);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X16, 0X86);
	tianma_dcs_write_seq_static(ctx, 0X1A, 0XFF);
	tianma_dcs_write_seq_static(ctx, 0X4F, 0X86);
	tianma_dcs_write_seq_static(ctx, 0X53, 0XFF);
	tianma_dcs_write_seq_static(ctx, 0X84, 0X86);
	tianma_dcs_write_seq_static(ctx, 0X88, 0XFF);

	tianma_dcs_write_seq_static(ctx, 0XFF, 0X2C);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X30, 0XF8);
	tianma_dcs_write_seq_static(ctx, 0X33, 0XF8);
	tianma_dcs_write_seq_static(ctx, 0X35, 0X21);
	tianma_dcs_write_seq_static(ctx, 0X37, 0X21);
	tianma_dcs_write_seq_static(ctx, 0X61, 0X02);
	tianma_dcs_write_seq_static(ctx, 0X62, 0X27);
	tianma_dcs_write_seq_static(ctx, 0X6B, 0X3B);
	tianma_dcs_write_seq_static(ctx, 0X6C, 0X3B);
	tianma_dcs_write_seq_static(ctx, 0X6D, 0X3B);
	tianma_dcs_write_seq_static(ctx, 0X82, 0XF8);
	tianma_dcs_write_seq_static(ctx, 0X85, 0XF8);
	tianma_dcs_write_seq_static(ctx, 0X87, 0X11);
	tianma_dcs_write_seq_static(ctx, 0X89, 0X11);
//end
//----------------108.4-------------------
	tianma_dcs_write_seq_static(ctx, 0XFF, 0X20);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X18, 0X66);
	tianma_dcs_write_seq_static(ctx, 0X2F, 0X83);
	tianma_dcs_write_seq_static(ctx, 0XF2, 0X65);
	tianma_dcs_write_seq_static(ctx, 0XF3, 0X64);
	tianma_dcs_write_seq_static(ctx, 0XF4, 0X65);
	tianma_dcs_write_seq_static(ctx, 0XF5, 0X64);
	tianma_dcs_write_seq_static(ctx, 0XF6, 0X65);
	tianma_dcs_write_seq_static(ctx, 0XF7, 0X64);
	tianma_dcs_write_seq_static(ctx, 0XF8, 0X65);
	tianma_dcs_write_seq_static(ctx, 0XF9, 0X64);

	tianma_dcs_write_seq_static(ctx, 0XFF, 0X26);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X06, 0X15);
	tianma_dcs_write_seq_static(ctx, 0X08, 0X15);
	tianma_dcs_write_seq_static(ctx, 0X88, 0X01);

	tianma_dcs_write_seq_static(ctx, 0XFF, 0X27);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X21, 0XE7);
	tianma_dcs_write_seq_static(ctx, 0XE4, 0XDA);
	tianma_dcs_write_seq_static(ctx, 0XE6, 0X6D);

	tianma_dcs_write_seq_static(ctx, 0XFF, 0X2A);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X07, 0X4C);
	tianma_dcs_write_seq_static(ctx, 0X19, 0X0F);
	tianma_dcs_write_seq_static(ctx, 0X1A, 0X03);
	tianma_dcs_write_seq_static(ctx, 0X1E, 0X53);
	tianma_dcs_write_seq_static(ctx, 0X1F, 0X53);
	tianma_dcs_write_seq_static(ctx, 0X20, 0X53);
	tianma_dcs_write_seq_static(ctx, 0X2F, 0X02);
	tianma_dcs_write_seq_static(ctx, 0X52, 0X0F);
	tianma_dcs_write_seq_static(ctx, 0X53, 0X03);
	tianma_dcs_write_seq_static(ctx, 0X57, 0X7D);
	tianma_dcs_write_seq_static(ctx, 0X58, 0X7D);
	tianma_dcs_write_seq_static(ctx, 0X59, 0X7D);
	tianma_dcs_write_seq_static(ctx, 0X7B, 0X40);
	tianma_dcs_write_seq_static(ctx, 0X87, 0X0F);
	tianma_dcs_write_seq_static(ctx, 0X88, 0X03);
	tianma_dcs_write_seq_static(ctx, 0X8C, 0X3E);
	tianma_dcs_write_seq_static(ctx, 0X8D, 0X43);
	tianma_dcs_write_seq_static(ctx, 0X8E, 0X3E);

	tianma_dcs_write_seq_static(ctx, 0XFF, 0X2C);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X35, 0X20);
	tianma_dcs_write_seq_static(ctx, 0X37, 0X20);
	tianma_dcs_write_seq_static(ctx, 0X4F, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X87, 0X10);
	tianma_dcs_write_seq_static(ctx, 0X89, 0X10);
	tianma_dcs_write_seq_static(ctx, 0X9F, 0X01);

	tianma_dcs_write_seq_static(ctx, 0XFF, 0XE0);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X35, 0X82);

	tianma_dcs_write_seq_static(ctx, 0XFF, 0XC0);
	tianma_dcs_write_seq_static(ctx, 0XFB, 0X01);
	tianma_dcs_write_seq_static(ctx, 0X9C, 0X11);
	tianma_dcs_write_seq_static(ctx, 0X9D, 0X11);

//11bit
	tianma_dcs_write_seq_static(ctx, 0xFF,0x23);
	msleep(1);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0x00,0x68);
	tianma_dcs_write_seq_static(ctx, 0x07,0x00);
	tianma_dcs_write_seq_static(ctx, 0x08,0x01);
	tianma_dcs_write_seq_static(ctx, 0x09,0xC7);

	tianma_dcs_write_seq_static(ctx, 0xFF,0x10);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0x53,0x24);
	tianma_dcs_write_seq_static(ctx, 0x55,0x01);
    //open TE
	tianma_dcs_write_seq_static(ctx, 0x35,0x00);

        tianma_dcs_write_seq_static(ctx, 0xFF,0xF0);
        tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
        tianma_dcs_write_seq_static(ctx, 0x5A,0x00);
        tianma_dcs_write_seq_static(ctx, 0xFF,0x10);
        tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
// endif

	tianma_dcs_write_seq_static(ctx, 0x11);
	msleep(80);
	//Display On
	tianma_dcs_write_seq_static(ctx, 0x29);
	msleep(10);
	pr_info("%s-\n", __func__);
}

static int tianma_disable(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tianma(panel);

	if (!ctx->enabled)
		return 0;
	pr_info("%s,120\n", __func__);
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int tianma_unprepare(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tianma(panel);
	pr_info("%s,120\n", __func__);

	if (!ctx->prepared)
		return 0;

	tianma_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	tianma_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
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
	struct tianma *ctx = panel_to_tianma(panel);
	int ret;

	pr_info("%s+,120\n", __func__);

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

static int tianma_prepare(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tianma(panel);
	int ret;

	pr_info("%s,120+\n", __func__);
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
	tianma_vdd_tp_start(ctx);
	usleep_range(10 * 1000, 10 * 1000);
//end
	tianma_vdd_tp_end(ctx);
	if (is_export_project)
		tianma_export_panel_init(ctx);
	else
		tianma_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		tianma_unprepare(panel);

	ctx->prepared = true;

#ifdef PANEL_SUPPORT_READBACK
	tianma_panel_get_data(ctx);
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

static int tianma_enable(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tianma(panel);

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
	.vendor = "NT36672C_TM",
	.manufacture = "tm2048",
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
	.vendor = "NT36672C_TM",
	.manufacture = "tm2048",
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
	.vendor = "NT36672C_TM",
	.manufacture = "tm2048",
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static int tianma_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0x07, 0xff, 0x00};
	if (level > 2047)
		level = 2047;
	pr_err("%s,120 backlight = %d\n", __func__, level);
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
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));//53 2C
	if(cabc_mode != 0){
		cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));//55 0X
	}
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct tianma *ctx = panel_to_tianma(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = tianma_setbacklight_cmdq,
	/* .esd_backlight_check = oplus_esd_backlight_check, */
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
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

static int tianma_get_modes(struct drm_panel *panel)
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

static const struct drm_panel_funcs tianma_drm_funcs = {
	.disable = tianma_disable,
	.unprepare = tianma_unprepare,
	.prepare = tianma_prepare,
	.enable = tianma_enable,
	.get_modes = tianma_get_modes,
};

static int tianma_probe(struct mipi_dsi_device *dsi)
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
	pr_info("%s,120+\n", __func__);
	ctx = devm_kzalloc(dev, sizeof(struct tianma), GFP_KERNEL);
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

		if (!ctx->backlight){
			return -EPROBE_DEFER;
		}
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
	ctx->panel.funcs = &tianma_drm_funcs;

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
	register_device_proc("lcd", "NT36672C_TM", "tm2048");
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

static int tianma_remove(struct mipi_dsi_device *dsi)
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

static const struct of_device_id tianma_of_match[] = {
	{
		.compatible = "tianma,120,nt36672c,cphy,vdo",
	},
	{} };

MODULE_DEVICE_TABLE(of, tianma_of_match);

static struct mipi_dsi_driver tianma_driver = {
	.probe = tianma_probe,
	.remove = tianma_remove,
	.driver = {

			.name = "panel-tianma-120-nt36672c-cphy-vdo",
			.owner = THIS_MODULE,
			.of_match_table = tianma_of_match,
		},
};

module_mipi_dsi_driver(tianma_driver);

MODULE_AUTHOR("Elon Hsu <elon.hsu@mediatek.com>");
MODULE_DESCRIPTION("tianma r66451 CMD AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
