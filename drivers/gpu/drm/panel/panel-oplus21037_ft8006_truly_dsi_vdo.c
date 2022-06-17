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
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

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

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif

#include <mt-plat/mtk_boot_common.h>
static char bl_tb0[] = { 0x51, 0xff };
/*****************************************************************************
 * Function Prototype
 *****************************************************************************/

extern int __attribute__((weak)) tp_gesture_enable_flag(void) {return 0;};
extern void __attribute__((weak)) lcd_queue_load_tp_fw(void) {return;};
extern void __attribute__((weak)) tp_gpio_current_leakage_handler(bool normal) {return;};
extern bool __attribute__((weak)) tp_boot_mode_normal() {return true;};
extern unsigned long esd_flag;
static int esd_brightness = 1023;
extern unsigned long oplus_max_normal_brightness;
extern void disp_aal_set_dre_en(int enable);
extern int _20015_lcm_i2c_write_bytes(unsigned char addr, unsigned char value);
static int cabc_lastlevel = 0;
static int last_brightness = 0;
static void cabc_switch(void *dsi, dcs_write_gce cb,
                void *handle, unsigned int cabc_mode);
static int backlight_gamma = 0;
extern unsigned int g_shutdown_flag;

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;

//	struct gpio_desc *bias_en;
	bool prepared;
	bool enabled;

	int error;

    bool is_normal_mode;
};

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
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

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;

	if (ret == 0) {
		ret = lcm_dcs_read(ctx,  0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
static struct regulator *disp_bias_pos;
static struct regulator *disp_bias_neg;


static int lcm_panel_bias_regulator_init(void)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(disp_bias_pos);
		pr_err("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		pr_err("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}

static int lcm_panel_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_err("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_err("enable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}

static int lcm_panel_bias_disable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_bias_regulator_init();

	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_err("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_err("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}
#endif

static void lcm_panel_init(struct lcm *ctx)
{
	pr_info("SYQ ft8006 truly %s+\n", __func__);

//-------------  Display Initial Code Setting  -------------------------
//Stop reload
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A);

//SPI Not LoadFinish
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x24);
	lcm_dcs_write_seq_static(ctx, 0x90,0x5A);

	//lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x03);
	//lcm_dcs_write_seq_static(ctx, 0x80,0x25,0x01);
//----------------------LCD initial code start----------------------//
//Blank 8
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x08);
	lcm_dcs_write_seq_static(ctx, 0x80,0xC8,0x2C,0x01);
//Blank 9
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x09);
	lcm_dcs_write_seq_static(ctx, 0x80,0x5A,0x51,0xB5,0x2A,0x6C,0xE5,0x4A,0x01,0x40,0x62,0x0F,0x82,0x20,0x08,0xF0,0xB7);
	lcm_dcs_write_seq_static(ctx, 0x90,0x00,0x24,0x42,0x0A,0xE3,0x91,0xA4,0xF0,0xAE,0xB9,0x66,0x20,0x19,0xA1,0x26,0x00);
	lcm_dcs_write_seq_static(ctx, 0xA0,0x51,0x55,0x55,0x00,0xA0,0x4D,0x06,0x11,0x0D,0x60,0x00,0xFF,0xFF,0x03,0xA5,0xE6);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x08,0x3A,0x12,0x64,0x0B,0x00,0x00,0x11,0x00,0x60,0x00,0xFF,0xFF,0x03,0xFF,0x34);
	lcm_dcs_write_seq_static(ctx, 0xC0,0x0C,0xFF,0x18,0x9F,0x0F,0x00,0x08,0x00);
//Blank 11
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x0B);
	lcm_dcs_write_seq_static(ctx, 0x80,0x00,0x00,0x20,0x44,0x08,0x00,0x60,0x47,0x00,0x00,0x10,0x22,0x04,0x00,0xB0,0x23);
	lcm_dcs_write_seq_static(ctx, 0x90,0x15,0x00);
//Blank 12
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x0C);
	lcm_dcs_write_seq_static(ctx, 0x80,0xBA,0x68,0x68,0x01,0x32,0x74,0xD0,0x07,0x00,0x60,0x15,0x00,0x50,0x15,0x56,0x51);
	lcm_dcs_write_seq_static(ctx, 0x90,0x15,0x55,0x61,0x15,0x00,0x60,0x15,0x00,0x50,0x15,0x56,0x51,0x15,0x55,0x61,0x95);
	lcm_dcs_write_seq_static(ctx, 0xA0,0xAB,0x18,0x00,0x05,0x00,0x05,0x00,0x05,0x00,0x49,0x29,0x84,0x52,0x01,0x09,0x00);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x00,0x00);
//Blank 13
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x0D);
	lcm_dcs_write_seq_static(ctx, 0x80,0xF0,0xB1,0x71,0xEF,0x4B,0xC0,0x80);
//Blank 14
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x0E);
	lcm_dcs_write_seq_static(ctx, 0x80,0xFF,0x01,0x55,0x55,0x23,0x88,0x88,0x1C);
//Blank 15
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x0F);
	lcm_dcs_write_seq_static(ctx, 0x80,0xBD,0x07,0x70,0xC0,0x12,0x08,0x64,0x08,0x52,0x51,0x58,0x49,0x03,0x52,0x4C,0x4C);
	lcm_dcs_write_seq_static(ctx, 0x90,0x68,0x68,0x68,0x4C,0x4C,0x7C,0x14,0x00,0x20,0x06,0xC2,0x00,0x04,0x06,0x0C,0x00);
	lcm_dcs_write_seq_static(ctx, 0xA0,0x00,0x92,0x00,0x00);
//Blank 16
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x10);
	lcm_dcs_write_seq_static(ctx, 0x80,0x00,0x00,0x03,0xE7,0x1F,0x17,0x10,0x48,0x80,0xAA,0xD0,0x18,0x30,0x88,0x41,0x8A);
	lcm_dcs_write_seq_static(ctx, 0x90,0x39,0x28,0xA9,0xC5,0x9A,0x7B,0xF0,0x07,0x7E,0xE0,0x07,0x7E,0x20,0x10,0x00);
//Blank 17
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x11);
	lcm_dcs_write_seq_static(ctx, 0x80,0x46,0x77,0x03,0x40,0xCA,0xF3,0xFF,0x83,0x30,0x08,0xC4,0x06,0xA1,0xD8,0x24,0x18);
	lcm_dcs_write_seq_static(ctx, 0x90,0x30,0xC6,0x66,0xC1,0x80,0x31,0x15,0xCB,0xE5,0xD2,0x68,0x6C,0x36,0x1D,0x04,0xC8);
	lcm_dcs_write_seq_static(ctx, 0xA0,0xB0,0xD9,0x88,0x60,0xB0,0x81,0x40,0x1A,0x1B,0x48,0x63,0x03,0xB9,0x00,0x1C,0x80);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x50,0x30,0x00,0xE0,0xE1,0x01,0x00,0x28,0x0E,0x06,0x43,0x55,0x55,0x55,0x55,0x55);
	lcm_dcs_write_seq_static(ctx, 0xC0,0x95,0x88,0x88,0x88,0x88,0x88,0xC8,0x08,0x86,0xC6,0xE3,0x81,0x00,0x20,0x00,0x21);
	lcm_dcs_write_seq_static(ctx, 0xD0,0x42,0x88,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x31,0x04,0x41,0x06,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0xE0,0x00,0x92,0x04,0x00,0x92,0x04,0x00,0x00,0x00,0x00,0x92,0x04,0x00,0x85,0x11,0x0C);
	lcm_dcs_write_seq_static(ctx, 0xF0,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x5E,0x4A,0x01,0x78,0x00,0x08,0x00,0x00);
//Blank 18
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x12);
	lcm_dcs_write_seq_static(ctx, 0x80,0x00,0x00,0x00,0x00,0x00,0x02,0x03,0x00,0x00,0x00,0x00,0x02,0x03,0x01,0x41,0x37);
	lcm_dcs_write_seq_static(ctx, 0x90,0xF1,0xE7,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2D,0x23,0x05);
	lcm_dcs_write_seq_static(ctx, 0xA0,0xFB,0x08,0x2D,0x23,0x05,0xFB,0x0C);
//Blank 19
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x13);
	lcm_dcs_write_seq_static(ctx, 0x80,0xFD,0x0F,0x00,0x0C,0x00,0x00,0x00,0x00,0x01,0x08,0x01,0x1C,0x44,0x0C,0xCE,0xE7);
	lcm_dcs_write_seq_static(ctx, 0x90,0x62,0x0E,0x24,0x98,0xAC,0x21,0x01,0x00,0xD0,0x93,0x24,0x49,0x06,0x20);
//Blank 20
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x14);
	lcm_dcs_write_seq_static(ctx, 0x80,0x01,0x02,0x41,0x36,0xE9,0xEF,0xF7,0xFB,0xFD,0x7E,0x01,0x00,0x00,0x90,0xC5,0x87);
	lcm_dcs_write_seq_static(ctx, 0x90,0xC7,0x3C,0x20,0x36,0x3C,0xE6,0x01,0x71,0xE1,0x31,0x0F,0x88,0x09,0x8F,0x79,0xC0);
	lcm_dcs_write_seq_static(ctx, 0xA0,0x36,0x98,0xCD,0x04,0x36,0xC2,0x6C,0x26,0x00,0xF8,0x82,0xB5,0xE2,0xF7,0x47,0x00);
	lcm_dcs_write_seq_static(ctx, 0xB0,0xD8,0x89,0xDF,0x1F,0x01,0x00,0x00,0x00,0x00,0x1C,0xB2,0x21,0x2B,0x00,0x40,0xA1);
	lcm_dcs_write_seq_static(ctx, 0xC0,0x50,0x78,0x07,0x85,0xF4,0x80,0x51,0x1C,0x6F,0xA0,0x90,0x1E,0x30,0x8A,0xE3,0x05);
	lcm_dcs_write_seq_static(ctx, 0xD0,0x14,0xD2,0x03,0x46,0x71,0xBC,0x82,0x42,0x7A,0xC0,0x28,0x8E,0x77,0x51,0x48,0x0F);
	lcm_dcs_write_seq_static(ctx, 0xE0,0x18,0xC5,0xF1,0x26,0x0A,0xE9,0x01,0xA3,0x38,0xDE,0x43,0x21,0x3D,0x60,0x14,0xC7);
	lcm_dcs_write_seq_static(ctx, 0xF0,0x5B,0x28,0xA4,0x07,0x8C,0xE2,0x18,0x01,0xBF,0xDF,0x08,0x00,0x00,0x00,0x00,0x00);
//Blank 21
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x15);
	lcm_dcs_write_seq_static(ctx, 0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x52,0xD9,0xE0,0xF7);
	lcm_dcs_write_seq_static(ctx, 0x90,0xA3,0x00,0x25,0x5B,0xC5,0xA8,0xC5,0x03,0x00,0x00,0x5C,0x31,0x6A,0xF1,0x41,0x8C);
	lcm_dcs_write_seq_static(ctx, 0xA0,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0x00,0xF0,0x00,0x14,0xCC,0x34,0x41);
	lcm_dcs_write_seq_static(ctx, 0xC0,0x13,0xA2,0x54,0x15,0x41,0x18,0x06,0x00,0x08,0xA2,0x6D,0xDB,0xB6,0x6D,0xDB,0xB6);
	lcm_dcs_write_seq_static(ctx, 0xD0,0x6D,0xDB,0xB6,0x6D,0xDB,0xB6,0x6D,0xDB,0xB6,0x6D,0xDB,0xB6,0x6D,0x5B,0xCD,0x34);
	lcm_dcs_write_seq_static(ctx, 0xE0,0x41,0x53,0xB2,0x54,0x55,0x51,0x1C,0x47,0x10,0x0C,0xA3,0x6D,0xDB,0xB6,0x6D,0xDB);
	lcm_dcs_write_seq_static(ctx, 0xF0,0xB6,0x6D,0xDB,0xB6,0x6D,0xDB,0xB6,0x6D,0xDB,0xB6,0x6D,0xDB,0xB6,0x6D,0x5B,0x80);
//Blank 22
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x16);
	lcm_dcs_write_seq_static(ctx, 0x80,0x82,0x82,0x82,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0x90,0x00,0x00,0x00,0x00,0xF0,0x20);
//Blank 24
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x18);
	lcm_dcs_write_seq_static(ctx, 0x80,0xB0,0x31,0x97,0x98,0xA4,0xF1,0xB1,0x65,0x08,0x08,0x4A,0x4B,0x2B,0xCB,0xAB,0x35);
	lcm_dcs_write_seq_static(ctx, 0x90,0x47,0xFD,0x5C,0x7C,0x09,0x88,0x5A,0xAA,0x39,0x69,0x69,0x76,0x07,0xCC,0x1B,0xFB);
	lcm_dcs_write_seq_static(ctx, 0xA0,0x88,0xB8,0xC7,0x67,0xF6,0xF5,0x35,0xC7,0xA7,0x4A,0xAA,0x0A,0x04,0xFF,0x00,0x80);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x80,0x00,0x04,0x20,0x00,0x01,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x04,0x00);
//Blank 25
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x19);
	lcm_dcs_write_seq_static(ctx, 0x80,0xC0,0xAF,0xA3,0x9B,0x92,0x8D,0x8A,0x86,0x84,0x83,0x82,0x80,0x00,0x80,0xF3,0xBF);
	lcm_dcs_write_seq_static(ctx, 0x90,0xF0,0x5F,0xFF,0xEF,0xCF,0x8F,0x0F,0xFF,0xAF,0xB5,0x71,0x0E,0x6C,0x4A,0x69,0x08);
	lcm_dcs_write_seq_static(ctx, 0xA0,0x00,0x00,0x28,0x00);
//Blank 26
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x1A);
	lcm_dcs_write_seq_static(ctx, 0x80,0x00,0x04,0x08,0x0C,0x00,0x10,0x14,0x18,0x1C,0x00,0x20,0x28,0x30,0x38,0x00,0x40);
	lcm_dcs_write_seq_static(ctx, 0x90,0x48,0x50,0x58,0x00,0x60,0x68,0x70,0x78,0x00,0x80,0x88,0x90,0x98,0x00,0xA0,0xA8);
	lcm_dcs_write_seq_static(ctx, 0xA0,0xB0,0xB8,0x00,0xC0,0xC8,0xD0,0xD8,0x00,0xE0,0xE8,0xF0,0xF8,0x00,0xFC,0xFE,0xFF);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x00,0x00,0x04,0x08,0x0C,0x00,0x10,0x14,0x18,0x1C,0x00,0x20,0x28,0x30,0x38,0x00);
	lcm_dcs_write_seq_static(ctx, 0xC0,0x40,0x48,0x50,0x58,0x00,0x60,0x68,0x70,0x78,0x00,0x80,0x88,0x90,0x98,0x00,0xA0);
	lcm_dcs_write_seq_static(ctx, 0xD0,0xA8,0xB0,0xB8,0x00,0xC0,0xC8,0xD0,0xD8,0x00,0xE0,0xE8,0xF0,0xF8,0x00,0xFC,0xFE);
	lcm_dcs_write_seq_static(ctx, 0xE0,0xFF,0x00,0x00,0x04,0x08,0x0C,0x00,0x10,0x14,0x18,0x1C,0x00,0x20,0x28,0x30,0x38);
	lcm_dcs_write_seq_static(ctx, 0xF0,0x00,0x40,0x48,0x50,0x58,0x00,0x60,0x68,0x70,0x78,0x00,0x80,0x88,0x90,0x98,0x00);
//Blank 27
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x1B);
	lcm_dcs_write_seq_static(ctx, 0x80,0xA0,0xA8,0xB0,0xB8,0x00,0xC0,0xC8,0xD0,0xD8,0x00,0xE0,0xE8,0xF0,0xF8,0x00,0xFC);
	lcm_dcs_write_seq_static(ctx, 0x90,0xFE,0xFF,0x00,0x00);
//Blank 32
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x20);
	lcm_dcs_write_seq_static(ctx, 0x80,0x81,0x00,0x00,0x00,0x00,0x00,0x00);
//Blank 34
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x22);
	lcm_dcs_write_seq_static(ctx, 0x80,0x2D,0xD3,0x00,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x9F,0x00);
//Blank 35
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x23);
	lcm_dcs_write_seq_static(ctx, 0x80,0x01,0x05,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0x90,0xFF,0x0F,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0xFF,0x07,0x35);
//Blank 36
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x24);
	lcm_dcs_write_seq_static(ctx, 0x80,0x00,0x03,0x00,0xFF,0xFF,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF2,0xE6);
	lcm_dcs_write_seq_static(ctx, 0x90,0x5A,0x5A,0x5A,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);

//----------------------LCD initial code End----------------------//

//SPI FINISH
	lcm_dcs_write_seq_static(ctx, 0x42,0x24);
	lcm_dcs_write_seq_static(ctx, 0x90,0x00);
//Blank select 2F
	lcm_dcs_write_seq_static(ctx, 0x41,0x5A,0x2F);
	lcm_dcs_write_seq_static(ctx, 0x19,0x00);

	//lcm_dcs_write_seq_static(ctx, 0x90,0x0F,0x0A);//CE
	//lcm_dcs_write_seq_static(ctx, 0x94,0x01);//contrast
//INT CANCEL
	lcm_dcs_write_seq_static(ctx, 0x4C,0x03);

//CMD1 LEDPWM set
	lcm_dcs_write_seq_static(ctx, 0x51,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0x53,0x24);
	lcm_dcs_write_seq_static(ctx, 0x55,0x01);
	lcm_dcs_write_seq_static(ctx, 0x35, 0x00);

	lcm_dcs_write_seq_static(ctx, 0x11, 0x00);
	msleep(120);
	lcm_dcs_write_seq_static(ctx, 0x29, 0x00);

	pr_info("%s-\n", __func__);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->prepared)
		return 0;
	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(20);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(150);

	ctx->error = 0;
	ctx->prepared = false;

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_disable();
#else
	pr_info("%s: tp_gesture_enable_flag = %d \n", __func__, tp_gesture_enable_flag());
	if ((0 == tp_gesture_enable_flag()) || (esd_flag == 1) || (g_shutdown_flag == 1)) {
		pr_info("%s:  \n", __func__);
		/*ctx->reset_gpio =
			devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->reset_gpio)) {
			dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
				__func__, PTR_ERR(ctx->reset_gpio));
			return PTR_ERR(ctx->reset_gpio);
		}
		gpiod_set_value(ctx->reset_gpio, 0);
		devm_gpiod_put(ctx->dev, ctx->reset_gpio);
		usleep_range(2 * 1000, 2 * 1000);
		tp_gpio_current_leakage_handler(false);
		usleep_range(2 * 1000, 2 * 1000);*/
		ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
			"bias", 1, GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->bias_neg)) {
			dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
				__func__, PTR_ERR(ctx->bias_neg));
			return PTR_ERR(ctx->bias_neg);
		}
		gpiod_set_value(ctx->bias_neg, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_neg);

		udelay(1000);

		ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
			"bias", 0, GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->bias_pos)) {
			dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
				__func__, PTR_ERR(ctx->bias_pos));
			return PTR_ERR(ctx->bias_pos);
		}
		gpiod_set_value(ctx->bias_pos, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_pos);
	}
#endif
	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("innolux %s+\n", __func__);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(3000);
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	_20015_lcm_i2c_write_bytes(0x0, 0x14);
	msleep(1);
	_20015_lcm_i2c_write_bytes(0x1, 0x14);

	pr_info("%s-\n", __func__);
//  	lcd_queue_load_tp_fw();
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	usleep_range(5000, 5100);
	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s\n", __func__);
	if (ctx->prepared)
		return 0;

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_enable();
#else
	pr_info("%s-\n", __func__);
#endif
	tp_gpio_current_leakage_handler(false);
	usleep_range(5 * 1000, 5 * 1000);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	usleep_range(2 * 1000, 2 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(6 * 1000, 6 * 1000);
	if (tp_boot_mode_normal()) {
		tp_gpio_current_leakage_handler(true);
	}
	usleep_range(3 * 1000, 3 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(5 * 1000, 5 * 1000);
	if (tp_boot_mode_normal()) {
		lcd_queue_load_tp_fw();
	}
	usleep_range(8 * 1000, 8 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(15 * 1000, 15 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(35 * 1000, 35 * 1000);
	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_rst(panel);
#endif
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

#define VAC (1600)
#define HAC (720)

static struct drm_display_mode default_mode = {
	.clock = 85997,
	.hdisplay = HAC,
	.hsync_start = HAC + 155,//HFP
	.hsync_end = HAC + 155 + 16,//HSA
	.htotal = HAC + 155 + 16 + 150,//HBP1289
	.vdisplay = VAC,
	.vsync_start = VAC + 135,//VFP
	.vsync_end = VAC + 135 + 8,//VSA
	.vtotal = VAC + 135 + 8 + 106,//VBP4948
	.vrefresh = 60,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	//.pll_clk = 553,
	//.vfp_low_power = 2528,//45hz
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9c, .mask_list[0] = 0x9c,
	},
	.data_rate = 733,
	.dyn = {
		.pll_clk = 362,
		.switch_en = 1,
		.data_rate = 724,
		.hbp = 139,
	},
	.oplus_teot_ns_multiplier = 90,
	.vendor = "21037_ft8006_truly",
	.manufacture = "FT8006",
};

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0xFF, 0xFF};
	pr_err("%s backlight = %d\n", __func__, level);
	if (level > 4095)
		level = 4095;
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0) {
		level = 2047;
	}
	bl_tb0[1] = level >> 4;
	bl_tb0[2] = level & 0x0F;
	esd_brightness = level;
	if (!cb)
		return -1;
        if (last_brightness == 0)
                msleep(15);

	pr_err("%s SYQ bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	last_brightness = level;
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

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	pr_err("%s ,%d\n", __func__,mode);
	if (mode == 0)
		ext->params = &ext_params;
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
	else
		ret = 1;

	return ret;

}

static void cabc_switch(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int cabc_mode)
{
	char bl_tb0[] = {0x55, 0x00};
	char bl_tb3[] = {0x53, 0x2C};
	pr_err("%s cabc = %d\n", __func__, cabc_mode);
	if(cabc_mode == 3)
		cabc_mode = 2;
	bl_tb0[1] = (u8)cabc_mode;
    msleep(5);
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));//FB 01
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));//55 0X
	cabc_lastlevel = cabc_mode;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.esd_backlight_recovery = oplus_esd_backlight_check,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
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

	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static int lcm_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;

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

	panel->connector->display_info.width_mm = 68;
	panel->connector->display_info.height_mm = 151;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
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
	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO
			 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(dev, "%s: cannot get bias-pos 0 %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(dev, "%s: cannot get bias-neg 1 %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	devm_gpiod_put(dev, ctx->bias_neg);

	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	oplus_max_normal_brightness = 3276;
	register_device_proc("lcd","21037_ft8006_truly","FT8006");
	pr_info("%s-\n", __func__);

	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}
static const struct of_device_id jdi_of_match[] = {
	{
		.compatible = "oplus21037_ft8006_truly_vdo",
	},
	{} };

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
			.name = "oplus21037_ft8006_truly_vdo_lcm_drv",
			.owner = THIS_MODULE,
			.of_match_table = jdi_of_match,
		},
};

module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("Shanguofan");
MODULE_DESCRIPTION("ft8006 truly VDO LCD Panel Driver");
MODULE_LICENSE("GPL v2");
