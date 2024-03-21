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
#define pr_fmt(fmt) "dsi_cmd: %s: " fmt, __func__

#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/backlight.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <soc/oplus/device_info.h>
#include <linux/timer.h>
#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_drm_graphics_base.h"
#include "../mediatek/mtk_log.h"
#include "../mediatek/mtk_panel_ext.h"
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif
extern unsigned long esd_flag;
#include <mt6370_pmu_bled.h>
#if (defined(OPLUS_BUG_STABILITY) && defined(CONFIG_MFD_MT6370_PMU))
	extern unsigned int pmic_custom_flag;
#endif
#define BLED_REAPTIME_NORMAL1 9
#define BLED_REAPTIME_NORMAL2 10
#define BLED_REAPTIME_SUSPEND 0
#define BLED_EN_SET 1
#define BLED_EN_CLN 0
extern void lcd_queue_load_tp_fw(void);
extern int tp_gesture_enable_flag(void);
// #ifdef OPLUS_BUG_COMPATIBILITY
extern void disp_aal_set_dre_en(int enable);
// #endif
// #ifdef OPLUS_BUG_COMPATIBILITY
static int cabc_lastlevel = 0;
static int last_brightness = 0;
//#endif
// #ifdef OPLUS_BUG_COMPATIBILITY
extern int shutdown_flag;
//#endif

static struct timer_list dyn_timer;

static int blmap_table[] = {
// #ifdef OPLUS_BUG_COMPATIBILITY
	27,2,
	14,8,
	16,8,
	16,8,
	16,8,
// #endif
	16,14,
	16,15,
	22,1,
	19,9,
	19,9,
	19,9,
	19,9,
	27,22,
	29,30,
	32,44,
	35,59,
	38,74,
	38,74,
	38,74,
	38,74,
	48,138,
	48,138,
	48,138,
	54,182,
	54,182,
	54,182,
	48,132,
	69,313,
	77,384,
	70,318,
	77,384,
	80,414,
	90,516,
	90,516,
	106,690,
	103,657,
	167,1396,
	194,1715,
	209,1898,
	238,2260,
	284,2849,
	310,3191,
	346,3675,
	326,3400,
	342,3625,
	352,3769,
	255,2342,
	192,1395
};
struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;

	bool prepared;
	bool enabled;

	int error;
};

#define lcm_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = {seq};                                          \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define lcm_dcs_write_seq_static(ctx, seq...)                                  \
	({                                                                     \
		static const u8 d[] = {seq};                                   \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
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
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
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
	static int bias_enable_flag = 0;
	lcm_panel_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 5550000, 5550000);
	if (ret < 0)
		pr_err("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 5550000, 5550000);
	if (ret < 0)
		pr_err("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	if( bias_enable_flag && regulator_is_enabled(disp_bias_pos)&&regulator_is_enabled(disp_bias_neg))
	{
		pr_err("bias regulator already enabled !\n");
		return retval;
	}
	bias_enable_flag = 1;

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
	pr_debug("bias regulator disable !\n");
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
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}
//#ifdef OPLUS_BUG_COMPATIBILITY
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(1 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(10 * 1000);
//#endif
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

    lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x00);     //PAGE0
    lcm_dcs_write_seq_static(ctx,0x51,0x0F, 0xFE);
    lcm_dcs_write_seq_static(ctx,0x53,0x24);  //pwm enable
    lcm_dcs_write_seq_static(ctx,0x55,0x01);
// #endif
//#ifdef OPLUS_BUG_COMPATIBILITY
	lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x06); // pwm freq 28k
	lcm_dcs_write_seq_static(ctx,0x08,0x20);  //pwm osc 116
//#endif
//#ifdef OPLUS_BUG_COMPATIBILITY
    lcm_dcs_write_seq_static(ctx,0xC7,0x05);
    lcm_dcs_write_seq_static(ctx,0x48,0x09);
// #endif
    lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x03);
    lcm_dcs_write_seq_static(ctx,0x83,0x30);  //11bit
//#ifdef OPLUS_BUG_COMPATIBILITY
    lcm_dcs_write_seq_static(ctx,0x84,0x01);  //28K
// #endif
//#ifdef OPLUS_BUG_COMPATIBILITY
    lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x03);
    lcm_dcs_write_seq_static(ctx,0x88,0xCC);
    lcm_dcs_write_seq_static(ctx,0x89,0xE5);
    lcm_dcs_write_seq_static(ctx,0x8A,0xED);
    lcm_dcs_write_seq_static(ctx,0x8B,0xEF);
    lcm_dcs_write_seq_static(ctx,0xB3,0xE5);
    lcm_dcs_write_seq_static(ctx,0xAC,0xFA);
    lcm_dcs_write_seq_static(ctx,0xAD,0xE0);
    lcm_dcs_write_seq_static(ctx,0x8C,0x9B);
    lcm_dcs_write_seq_static(ctx,0x8D,0xA9);
    lcm_dcs_write_seq_static(ctx,0x8E,0xAD);
    lcm_dcs_write_seq_static(ctx,0x8F,0xAF);
    lcm_dcs_write_seq_static(ctx,0x90,0xB2);
    lcm_dcs_write_seq_static(ctx,0x91,0xB9);
    lcm_dcs_write_seq_static(ctx,0x92,0xBA);
    lcm_dcs_write_seq_static(ctx,0x93,0xBD);
    lcm_dcs_write_seq_static(ctx,0x94,0xCA);
    lcm_dcs_write_seq_static(ctx,0x95,0xD5);
    lcm_dcs_write_seq_static(ctx,0xB4,0xD9);
    lcm_dcs_write_seq_static(ctx,0xAE,0xD3);
    lcm_dcs_write_seq_static(ctx,0x96,0x70);
    lcm_dcs_write_seq_static(ctx,0x97,0x89);
    lcm_dcs_write_seq_static(ctx,0x98,0x99);
    lcm_dcs_write_seq_static(ctx,0x99,0x9F);
    lcm_dcs_write_seq_static(ctx,0x9A,0xA3);
    lcm_dcs_write_seq_static(ctx,0x9B,0xA6);
    lcm_dcs_write_seq_static(ctx,0x9C,0xA9);
    lcm_dcs_write_seq_static(ctx,0x9D,0xAD);
    lcm_dcs_write_seq_static(ctx,0x9E,0xC2);
    lcm_dcs_write_seq_static(ctx,0x9F,0xCC);
    lcm_dcs_write_seq_static(ctx,0xB5,0xD0);
    lcm_dcs_write_seq_static(ctx,0xA0,0xC0);
    lcm_dcs_write_seq_static(ctx,0xA1,0x7E);
    lcm_dcs_write_seq_static(ctx,0xA2,0x7E);
    lcm_dcs_write_seq_static(ctx,0xA7,0xE5);
    lcm_dcs_write_seq_static(ctx,0xA8,0xE5);
    lcm_dcs_write_seq_static(ctx,0xB6,0x00);
    lcm_dcs_write_seq_static(ctx,0xB7,0xD0);
    lcm_dcs_write_seq_static(ctx,0xB8,0xD0);
    lcm_dcs_write_seq_static(ctx,0xB1,0x66);
    lcm_dcs_write_seq_static(ctx,0xB2,0x66);
    lcm_dcs_write_seq_static(ctx,0x86,0x6C);
    lcm_dcs_write_seq_static(ctx,0x87,0x6C);
//#endif  //CABC END
    lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x00);		//PAGE0
    lcm_dcs_write_seq_static(ctx,0x11,0x00);
//#ifdef OPLUS_BUG_COMPATIBILITY
    msleep(80);
// #endif
    lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x0C);  //noise modify 531
    lcm_dcs_write_seq_static(ctx,0x00,0x3F);
    lcm_dcs_write_seq_static(ctx,0x01,0xD3);
    lcm_dcs_write_seq_static(ctx,0x02,0x3F);
    lcm_dcs_write_seq_static(ctx,0x03,0xCF);
    lcm_dcs_write_seq_static(ctx,0x04,0x3F);
    lcm_dcs_write_seq_static(ctx,0x05,0xD6);
    lcm_dcs_write_seq_static(ctx,0x06,0x3F);
    lcm_dcs_write_seq_static(ctx,0x07,0xD1);
    lcm_dcs_write_seq_static(ctx,0x08,0x3F);
    lcm_dcs_write_seq_static(ctx,0x09,0xC9);
    lcm_dcs_write_seq_static(ctx,0x0A,0x3F);
    lcm_dcs_write_seq_static(ctx,0x0B,0xDF);
    lcm_dcs_write_seq_static(ctx,0x0C,0x3F);
    lcm_dcs_write_seq_static(ctx,0x0D,0xDD);
    lcm_dcs_write_seq_static(ctx,0x0E,0x3F);
    lcm_dcs_write_seq_static(ctx,0x0F,0xD9);
    lcm_dcs_write_seq_static(ctx,0x10,0x3F);
    lcm_dcs_write_seq_static(ctx,0x11,0xD8);
    lcm_dcs_write_seq_static(ctx,0x12,0x3F);
    lcm_dcs_write_seq_static(ctx,0x13,0xE4);
    lcm_dcs_write_seq_static(ctx,0x14,0x3F);
    lcm_dcs_write_seq_static(ctx,0x15,0xCD);
    lcm_dcs_write_seq_static(ctx,0x16,0x3F);
    lcm_dcs_write_seq_static(ctx,0x17,0xD4);
    lcm_dcs_write_seq_static(ctx,0x18,0x3F);
    lcm_dcs_write_seq_static(ctx,0x19,0xE1);
    lcm_dcs_write_seq_static(ctx,0x1A,0x3F);
    lcm_dcs_write_seq_static(ctx,0x1B,0xD0);
    lcm_dcs_write_seq_static(ctx,0x1C,0x3F);
    lcm_dcs_write_seq_static(ctx,0x1D,0xD5);
    lcm_dcs_write_seq_static(ctx,0x1E,0x3F);
    lcm_dcs_write_seq_static(ctx,0x1F,0xE0);
    lcm_dcs_write_seq_static(ctx,0x20,0x3F);
    lcm_dcs_write_seq_static(ctx,0x21,0xDC);
    lcm_dcs_write_seq_static(ctx,0x22,0x3F);
    lcm_dcs_write_seq_static(ctx,0x23,0xE3);
    lcm_dcs_write_seq_static(ctx,0x24,0x3F);
    lcm_dcs_write_seq_static(ctx,0x25,0xCB);
    lcm_dcs_write_seq_static(ctx,0x26,0x3F);
    lcm_dcs_write_seq_static(ctx,0x27,0xDA);
    lcm_dcs_write_seq_static(ctx,0x28,0x3F);
    lcm_dcs_write_seq_static(ctx,0x29,0xDE);
    lcm_dcs_write_seq_static(ctx,0x2A,0x3F);
    lcm_dcs_write_seq_static(ctx,0x2B,0xC8);
    lcm_dcs_write_seq_static(ctx,0x2C,0x3F);
    lcm_dcs_write_seq_static(ctx,0x2D,0xE2);
    lcm_dcs_write_seq_static(ctx,0x2E,0x3F);
    lcm_dcs_write_seq_static(ctx,0x2F,0xCA);
    lcm_dcs_write_seq_static(ctx,0x30,0x3F);
    lcm_dcs_write_seq_static(ctx,0x31,0xDB);
    lcm_dcs_write_seq_static(ctx,0x32,0x3F);
    lcm_dcs_write_seq_static(ctx,0x33,0xD7);
    lcm_dcs_write_seq_static(ctx,0x34,0x3F);
    lcm_dcs_write_seq_static(ctx,0x35,0xCC);
    lcm_dcs_write_seq_static(ctx,0x36,0x3F);
    lcm_dcs_write_seq_static(ctx,0x37,0xCE);
    lcm_dcs_write_seq_static(ctx,0x38,0x3F);
    lcm_dcs_write_seq_static(ctx,0x39,0xD2);
    lcm_dcs_write_seq_static(ctx,0x80,0x2E);
    lcm_dcs_write_seq_static(ctx,0x81,0xA8);
    lcm_dcs_write_seq_static(ctx,0x82,0x2C);
    lcm_dcs_write_seq_static(ctx,0x83,0x96);
    lcm_dcs_write_seq_static(ctx,0x84,0x2E);
    lcm_dcs_write_seq_static(ctx,0x85,0xA6);
    lcm_dcs_write_seq_static(ctx,0x86,0x2E);
    lcm_dcs_write_seq_static(ctx,0x87,0xAC);
    lcm_dcs_write_seq_static(ctx,0x88,0x2D);
    lcm_dcs_write_seq_static(ctx,0x89,0x9D);
    lcm_dcs_write_seq_static(ctx,0x8A,0x2D);
    lcm_dcs_write_seq_static(ctx,0x8B,0x9B);
    lcm_dcs_write_seq_static(ctx,0x8C,0x2E);
    lcm_dcs_write_seq_static(ctx,0x8D,0xB0);
    lcm_dcs_write_seq_static(ctx,0x8E,0x2C);
    lcm_dcs_write_seq_static(ctx,0x8F,0x98);
    lcm_dcs_write_seq_static(ctx,0x90,0x2E);
    lcm_dcs_write_seq_static(ctx,0x91,0xAF);
    lcm_dcs_write_seq_static(ctx,0x92,0x2D);
    lcm_dcs_write_seq_static(ctx,0x93,0xA0);
    lcm_dcs_write_seq_static(ctx,0x94,0x2E);
    lcm_dcs_write_seq_static(ctx,0x95,0xA3);
    lcm_dcs_write_seq_static(ctx,0x96,0x2E);
    lcm_dcs_write_seq_static(ctx,0x97,0xAD);
    lcm_dcs_write_seq_static(ctx,0x98,0x2E);
    lcm_dcs_write_seq_static(ctx,0x99,0xAE);
    lcm_dcs_write_seq_static(ctx,0x9A,0x2C);
    lcm_dcs_write_seq_static(ctx,0x9B,0x97);
    lcm_dcs_write_seq_static(ctx,0x9C,0x2E);
    lcm_dcs_write_seq_static(ctx,0x9D,0xAB);
    lcm_dcs_write_seq_static(ctx,0x9E,0x2E);
    lcm_dcs_write_seq_static(ctx,0x9F,0xA9);
    lcm_dcs_write_seq_static(ctx,0xA0,0x2E);
    lcm_dcs_write_seq_static(ctx,0xA1,0xA7);
    lcm_dcs_write_seq_static(ctx,0xA2,0x2E);
    lcm_dcs_write_seq_static(ctx,0xA3,0xA5);
    lcm_dcs_write_seq_static(ctx,0xA4,0x2D);
    lcm_dcs_write_seq_static(ctx,0xA5,0x9E);
    lcm_dcs_write_seq_static(ctx,0xA6,0x2E);
    lcm_dcs_write_seq_static(ctx,0xA7,0xAA);
    lcm_dcs_write_seq_static(ctx,0xA8,0x2E);
    lcm_dcs_write_seq_static(ctx,0xA9,0xB2);
    lcm_dcs_write_seq_static(ctx,0xAA,0x2D);
    lcm_dcs_write_seq_static(ctx,0xAB,0x9C);
    lcm_dcs_write_seq_static(ctx,0xAC,0x2E);
    lcm_dcs_write_seq_static(ctx,0xAD,0xA4);
    lcm_dcs_write_seq_static(ctx,0xAE,0x2D);
    lcm_dcs_write_seq_static(ctx,0xAF,0x9F);
    lcm_dcs_write_seq_static(ctx,0xB0,0x2C);
    lcm_dcs_write_seq_static(ctx,0xB1,0x99);
    lcm_dcs_write_seq_static(ctx,0xB2,0x2E);
    lcm_dcs_write_seq_static(ctx,0xB3,0xA1);
    lcm_dcs_write_seq_static(ctx,0xB4,0x2E);
    lcm_dcs_write_seq_static(ctx,0xB5,0xB1);
    lcm_dcs_write_seq_static(ctx,0xB6,0x2E);
    lcm_dcs_write_seq_static(ctx,0xB7,0xA2);
    lcm_dcs_write_seq_static(ctx,0xB8,0x2D);
    lcm_dcs_write_seq_static(ctx,0xB9,0x9A);
//#ifdef OPLUS_BUG_COMPATIBILITY
    lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x01);
    lcm_dcs_write_seq_static(ctx,0xE6,0x22);
// #endif
    lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x02);
    lcm_dcs_write_seq_static(ctx,0x1B,0x00);  //120hz
//#ifdef OPLUS_BUG_COMPATIBILITY
    lcm_dcs_write_seq_static(ctx,0x01,0x55);  //timeout
    lcm_dcs_write_seq_static(ctx,0x02,0x09);   //timeout 256us
// #endif
    lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x07);
    lcm_dcs_write_seq_static(ctx,0x29,0xCF); //dsc on

    lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x00);		//PAGE0
    lcm_dcs_write_seq_static(ctx,0x29,0x00);
    msleep(20);
    lcm_dcs_write_seq_static(ctx,0x35, 0x00);
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
//#ifdef OPLUS_BUG_COMPATIBILITY
	udelay(1 * 1000);
// #endif
//ifdef OPLUS_BUG_COMPATIBILITY
// lihao MULTIMEDIA.DISPLAY.LCD 2021/07/28 add Frame date-diaplay off delay
	msleep(5);
//#endif
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x78,0x07,0x00);
	lcm_dcs_write_seq_static(ctx, 0x28, 0x00);
	msleep(20);
	lcm_dcs_write_seq_static(ctx, 0x10, 0x00);
//#ifdef OPLUS_BUG_COMPATIBILITY
	msleep(120);
//#endif
	ctx->error = 0;
	ctx->prepared = false;

	pr_info("%s: tp_gesture_enable_flag = %d \n", __func__, tp_gesture_enable_flag());
// #ifdef OPLUS_BUG_COMPATIBILITY
	if ((0 == tp_gesture_enable_flag()) || /*(1 == shutdown_flag) ||*/ (esd_flag == 1)) {
// #endif
//#ifdef OPLUS_BUG_COMPATIBILITY
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	mdelay(3);
// #endif
	lcm_panel_bias_disable();
	}
// #endif
	return 0;
}

// #ifdef OPLUS_BUG_COMPATIBILITY
static void set_cabc_mode(struct lcm *ctx, int cabc_mode)
{
	char cabc_tb0[] = {0x55, cabc_mode&0xFF};
	char cabc_tb1[] = {0x53, 0x24};

	if (cabc_mode != 0) {
		pr_info("%s: set cabc mode %d\n", __func__, cabc_mode);
		lcm_dcs_write(ctx, cabc_tb0, ARRAY_SIZE(cabc_tb0));
		lcm_dcs_write(ctx, cabc_tb1, ARRAY_SIZE(cabc_tb1));
	}
}
// #endif
static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s\n", __func__);
	if (ctx->prepared)
		return 0;
//#ifdef OPLUS_BUG_COMPATIBILITY
	lcm_panel_bias_enable();
// #endif
    lcd_queue_load_tp_fw();
	lcm_panel_init(ctx);
// #ifdef OPLUS_BUG_COMPATIBILITY
	mt6370_pmu_reg_a0(BLED_EN_SET);
// #endif
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
// #ifdef OPLUS_BUG_COMPATIBILITY
	set_cabc_mode(ctx, cabc_lastlevel);
// #endif

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
// #ifdef OPLUS_BUG_COMPATIBILITY
#define HFP (86)
#define HSA (4)
#define HBP (86)
#define VSA (4)
#define VBP (35)
#define VAC (2412)
#define HAC (1080)
#define VFP_60hz (2540)
#define VFP_90hz (875)
#define VFP_120hz (40)
#define VFP_50hz (3540)
#define VFP_48hz (3790)
#define VFP_45hz (4200)
#define VFP_30hz (7545)
#define DYN_PLL_CLK (562)
#define DYN_DATA_RATE (1124)
#define HFP_DYN (94)
// #endif
#if defined(CONFIG_MTK_PANEL_EXT)
static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	unsigned char data[3] = {0x00, 0x00, 0x00};
	unsigned char id[3] = {0x40, 0x00, 0x00};
	ssize_t ret;

	ret = mipi_dsi_dcs_read(dsi, 0x4, data, 3);
	if (ret < 0) {
		pr_err("%s error\n", __func__);
		return 0;
	}

	pr_info("ATA read data %x %x %x\n", data[0], data[1], data[2]);

	if (data[0] == id[0] &&
			data[1] == id[1] &&
			data[2] == id[2])
		return 1;

	pr_info("ATA expect read data is %x %x %x\n",
			id[0], id[1], id[2]);

	return 0;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	char bl_tb0[] = {0x51, 0x0F, 0xFF};
// #ifdef OPLUS_BUG_COMPATIBILITY
	char bl_tb1[] = {0x55, 0x00};
	char bl_tb3[] = {0x53, 0x24};
//#endif
// #ifdef OPLUS_BUG_COMPATIBILITY
	if (level < 265) {
		mt6370_pmu_reg_a3(BLED_REAPTIME_NORMAL2);
		}else {
		mt6370_pmu_reg_a3(BLED_REAPTIME_NORMAL1);
		}
// #endif
//#ifdef OPLUS_BUG_COMPATIBILITY
	if (level > 2047){
		level = 2047;
// #ifdef OPLUS_BUG_COMPATIBILITY
	}else if(level > 0 && level < 5){
// #endif
		level = 2;
	}
// #ifdef OPLUS_BUG_COMPATIBILITY
    if (last_brightness == 0) {
        mdelay(43);
   }
   last_brightness = level;
//#endif

    bl_tb0[1] = (level&0x7ff)>>7;/*get hight 4bit*/
    bl_tb0[2] = (level<<1)&0xfe;/*get low 7bit*/

	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
// #ifdef OPLUS_BUG_COMPATIBILITY
	if ((last_brightness == 0) && (cabc_lastlevel != 0)) {
		bl_tb1[1] = cabc_lastlevel;
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
		cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	}
	last_brightness = level;
// #endif
// #ifdef OPLUS_BUG_COMPATIBILITY
	if (level ==0){
		mt6370_pmu_reg_a3(BLED_REAPTIME_SUSPEND);
		mt6370_pmu_reg_a0(BLED_EN_CLN);
	}
// #endif
	return 0;
}
// #ifdef OPLUS_BUG_COMPATIBILITY
static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
                void *handle)
{
	char bl_tb0[] = {0x51, 0x07, 0xff};
	bl_tb0[1] = (esd_brightness&0x7ff)>>7;/*get hight 4bit*/
	bl_tb0[2] = (esd_brightness<<1)&0xfe;/*get low 7bit*/
	if (!cb)
		return -1;
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 1;
}
// #endif
static void cabc_switch(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int cabc_mode)
{
	char bl_tb0[] = {0x55, 0x00};
	char bl_tb1[] = {0xFF, 0x78,0x07,0x00};
	char bl_tb3[] = {0x53, 0x2C};

	printk("%s cabc = %d\n", __func__, cabc_mode);
	bl_tb0[1] = (u8)cabc_mode;
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
// #ifdef OPLUS_BUG_COMPATIBILITY
	cabc_lastlevel = cabc_mode;
// #endif
}
static int lcm_get_virtual_heigh(void)
{
	return VAC;
}

static int lcm_get_virtual_width(void)
{
	return HAC;
}

static struct drm_display_mode performance_mode_120 = {
// #ifdef OPLUS_BUG_COMPATIBILITY
	.clock = 375444,
// #endif
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP_120hz,
	.vsync_end = VAC + VFP_120hz + VSA,
	.vtotal = VAC + VFP_120hz + VSA + VBP,
	.vrefresh = 120,
};

static struct drm_display_mode default_mode_30 = {
// #ifdef OPLUS_BUG_COMPATIBILITY
	.clock = 376649,
// #endif
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP_30hz,
	.vsync_end = VAC + VFP_30hz + VSA,
	.vtotal = VAC + VFP_30hz + VSA + VBP,
	.vrefresh = 30,
};

static struct drm_display_mode default_mode_45 = {
// #ifdef OPLUS_BUG_COMPATIBILITY
	.clock = 375915,
// #endif
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP_45hz,
	.vsync_end = VAC + VFP_45hz + VSA,
	.vtotal = VAC + VFP_45hz + VSA + VBP,
	.vrefresh = 45,
};

static struct drm_display_mode default_mode_48 = {
// #ifdef OPLUS_BUG_COMPATIBILITY
	.clock = 376258,
// #endif
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP_48hz,
	.vsync_end = VAC + VFP_48hz + VSA,
	.vtotal = VAC + VFP_48hz + VSA + VBP,
	.vrefresh = 48,
};

static struct drm_display_mode default_mode_50 = {
// #ifdef OPLUS_BUG_COMPATIBILITY
	.clock = 376235,
// #endif
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP_50hz,
	.vsync_end = VAC + VFP_50hz + VSA,
	.vtotal = VAC + VFP_50hz + VSA + VBP,
	.vrefresh = 50,
};

static struct drm_display_mode default_mode_60 = {
// #ifdef OPLUS_BUG_COMPATIBILITY
	.clock = 376122,
// #endif
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP_60hz,
	.vsync_end = VAC + VFP_60hz + VSA,
	.vtotal = VAC + VFP_60hz + VSA + VBP,
	.vrefresh = 60,
};

static struct drm_display_mode performance_mode_90 = {
// #ifdef OPLUS_BUG_COMPATIBILITY
	.clock = 375971,
// #endif
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP_90hz,
	.vsync_end = VAC + VFP_90hz + VSA,
	.vtotal = VAC + VFP_90hz + VSA + VBP,
	.vrefresh = 90,
};

static struct mtk_panel_params ext_params_120hz = {
	.pll_clk = 553,
// #ifdef OPLUS_BUG_COMPATIBILITY
	.phy_timcon = {
	    .hs_trail = 14,
	},
//#endif
//	.vfp_low_power = VFP_120hz,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
		.mask_list[0] = 0x9c,
	},
//#ifdef OPLUS_BUG_COMPATIBILITY
	.lcm_esd_check_table[1] = {
		.cmd = 0x0D,
		.count = 1,
		.para_list[0] = 0x00,

	},
// #end
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
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
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
	.data_rate = 1106,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 120,
	},
	.dyn = {
		.switch_en = 0,
		.pll_clk = DYN_PLL_CLK,
		.data_rate = DYN_DATA_RATE,
		.vsa = VSA,
		.vbp = VBP,
		.vfp = VFP_120hz,
		//.vfp_lp_dyn = ,
		.hsa = HSA,
		.hbp = HBP,
		.hfp = HFP_DYN,
	},
	.oplus_display_global_dre = 1,
	.brightness_max = 2047,
	.brightness_min = 2,
	.blmap = blmap_table,
	.blmap_size = sizeof(blmap_table)/sizeof(blmap_table[0]),
};

static struct mtk_panel_params ext_params_30hz = {
	.pll_clk = 553,
// #ifdef OPLUS_BUG_COMPATIBILITY
	.phy_timcon = {
	    .hs_trail = 14,
	},
//#endif
//	.vfp_low_power = VFP_30hz,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
		.mask_list[0] = 0x9c,
	},
//#ifdef OPLUS_BUG_COMPATIBILITY
	.lcm_esd_check_table[1] = {
		.cmd = 0x0D,
		.count = 1,
		.para_list[0] = 0x00,

	},
// #endif
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
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
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
	.data_rate = 1106,
	.dyn_fps = {
// #ifdef OPLUS_BUG_COMPATIBILITY
		.switch_en = 1, .vact_timing_fps = 120,
// #endif
	},
	.dyn = {
		.switch_en = 0,
		.pll_clk = DYN_PLL_CLK,
		.data_rate = DYN_DATA_RATE,
		.vsa = VSA,
		.vbp = VBP,
		.vfp = VFP_30hz,
		//.vfp_lp_dyn = ,
		.hsa = HSA,
		.hbp = HBP,
		.hfp = HFP_DYN,
	},
	.oplus_display_global_dre = 1,
	.brightness_max = 2047,
	.brightness_min = 2,
	.blmap = blmap_table,
	.blmap_size = sizeof(blmap_table)/sizeof(blmap_table[0]),
};

static struct mtk_panel_params ext_params_45hz = {
	.pll_clk = 553,
// #ifdef OPLUS_BUG_COMPATIBILITY
	.phy_timcon = {
	    .hs_trail = 14,
	},
//#endif
//	.vfp_low_power = VFP_120hz,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
		.mask_list[0] = 0x9c,
	},
//#ifdef OPLUS_BUG_COMPATIBILITY
	.lcm_esd_check_table[1] = {
		.cmd = 0x0D,
		.count = 1,
		.para_list[0] = 0x00,
	},
// #endif
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
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
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
	.data_rate = 1106,
	.dyn_fps = {
// #ifdef OPLUS_BUG_COMPATIBILITY
		.switch_en = 1, .vact_timing_fps = 120,
// #endif
	},
	.dyn = {
		.switch_en = 0,
		.pll_clk = DYN_PLL_CLK,
		.data_rate = DYN_DATA_RATE,
		.vsa = VSA,
		.vbp = VBP,
		.vfp = VFP_45hz,
		//.vfp_lp_dyn = ,
		.hsa = HSA,
		.hbp = HBP,
		.hfp = HFP_DYN,
	},
	.oplus_display_global_dre = 1,
	.brightness_max = 2047,
	.brightness_min = 2,
	.blmap = blmap_table,
	.blmap_size = sizeof(blmap_table)/sizeof(blmap_table[0]),
};

static struct mtk_panel_params ext_params_48hz = {
	.pll_clk = 553,
// #ifdef OPLUS_BUG_COMPATIBILITY
	.phy_timcon = {
	    .hs_trail = 14,
	},
//#endif
//	.vfp_low_power = VFP_120hz,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
		.mask_list[0] = 0x9c,
	},
//#ifdef OPLUS_BUG_COMPATIBILITY
	.lcm_esd_check_table[1] = {
		.cmd = 0x0D,
		.count = 1,
		.para_list[0] = 0x00,
	},
// #endif
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
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
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
	.data_rate = 1106,
	.dyn_fps = {
// #ifdef OPLUS_BUG_COMPATIBILITY
		.switch_en = 1, .vact_timing_fps = 120,
// #endif
	},
	.dyn = {
		.switch_en = 0,
		.pll_clk = DYN_PLL_CLK,
		.data_rate = DYN_DATA_RATE,
		.vsa = VSA,
		.vbp = VBP,
		.vfp = VFP_48hz,
		//.vfp_lp_dyn = ,
		.hsa = HSA,
		.hbp = HBP,
		.hfp = HFP_DYN,
	},
	.oplus_display_global_dre = 1,
	.brightness_max = 2047,
	.brightness_min = 2,
	.blmap = blmap_table,
	.blmap_size = sizeof(blmap_table)/sizeof(blmap_table[0]),
};

static struct mtk_panel_params ext_params_50hz = {
	.pll_clk = 553,
// #ifdef OPLUS_BUG_COMPATIBILITY
	.phy_timcon = {
	    .hs_trail = 14,
	},
//#endif
//	.vfp_low_power = VFP_120hz,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
		.mask_list[0] = 0x9c,
	},
//#ifdef OPLUS_BUG_COMPATIBILITY
	.lcm_esd_check_table[1] = {
		.cmd = 0x0D,
		.count = 1,
		.para_list[0] = 0x00,
	},
// #endif
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
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
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
	.data_rate = 1106,
	.dyn_fps = {
// #ifdef OPLUS_BUG_COMPATIBILITY
		.switch_en = 1, .vact_timing_fps = 120,
// #endif
	},
	.dyn = {
		.switch_en = 0,
		.pll_clk = DYN_PLL_CLK,
		.data_rate = DYN_DATA_RATE,
		.vsa = VSA,
		.vbp = VBP,
		.vfp = VFP_50hz,
		//.vfp_lp_dyn = ,
		.hsa = HSA,
		.hbp = HBP,
		.hfp = HFP_DYN,
	},
	.oplus_display_global_dre = 1,
	.brightness_max = 2047,
	.brightness_min = 2,
	.blmap = blmap_table,
	.blmap_size = sizeof(blmap_table)/sizeof(blmap_table[0]),
};

static struct mtk_panel_params ext_params_60hz = {
	.pll_clk = 553,
// #ifdef OPLUS_BUG_COMPATIBILITY
	.phy_timcon = {
	    .hs_trail = 14,
	},
//#endif
//	.vfp_low_power = VFP_60hz,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
		.mask_list[0] = 0x9c,
	},
//#ifdef OPLUS_BUG_COMPATIBILITY
	.lcm_esd_check_table[1] = {
		.cmd = 0x0D,
		.count = 1,
		.para_list[0] = 0x00,
	},
// #endif
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
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
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
	.data_rate = 1106,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 120,
	},
	.dyn = {
		.switch_en = 0,
		.pll_clk = DYN_PLL_CLK,
		.data_rate = DYN_DATA_RATE,
		.vsa = VSA,
		.vbp = VBP,
		.vfp = VFP_60hz,
		//.vfp_lp_dyn = ,
		.hsa = HSA,
		.hbp = HBP,
		.hfp = HFP_DYN,
	},
	.oplus_display_global_dre = 1,
	.brightness_max = 2047,
	.brightness_min = 2,
	.blmap = blmap_table,
	.blmap_size = sizeof(blmap_table)/sizeof(blmap_table[0]),
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 553,
// #ifdef OPLUS_BUG_COMPATIBILITY
	.phy_timcon = {
	    .hs_trail = 14,
	},
//#endif
//	.vfp_low_power = VFP_90hz,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
		.mask_list[0] = 0x9c,
	},
//#ifdef OPLUS_BUG_COMPATIBILITY
	.lcm_esd_check_table[1] = {
		.cmd = 0x0D,
		.count = 1,
		.para_list[0] = 0x00,
	},
// #endif
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
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
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
	.data_rate = 1106,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 120,
	},
	.dyn = {
		.switch_en = 0,
		.pll_clk = DYN_PLL_CLK,
		.data_rate = DYN_DATA_RATE,
		.vsa = VSA,
		.vbp = VBP,
		.vfp = VFP_90hz,
		//.vfp_lp_dyn = ,
		.hsa = HSA,
		.hbp = HBP,
		.hfp = HFP_DYN,
	},
	.oplus_display_global_dre = 1,
	.brightness_max = 2047,
	.brightness_min = 2,
	.blmap = blmap_table,
	.blmap_size = sizeof(blmap_table)/sizeof(blmap_table[0]),
};

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	pr_err("%s ,%d\n", __func__,mode);
	if (mode == 0)
		ext->params = &ext_params_120hz;
	else if (mode == 1)
// #ifdef OPLUS_BUG_COMPATIBILITY
		ext->params = &ext_params_90hz;
	else if (mode == 2)
		ext->params = &ext_params_60hz;
	else if (mode == 3)
		ext->params = &ext_params_50hz;
	else if (mode == 4)
		ext->params = &ext_params_48hz;
	else if (mode == 5)
		ext->params = &ext_params_45hz;
	else if (mode == 6)
		ext->params = &ext_params_30hz;
//#endif
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
		ext_para = &ext_params_120hz;
	else if (mode == 1)
// #ifdef OPLUS_BUG_COMPATIBILITY
		ext_para = &ext_params_90hz;
	else if (mode == 2)
		ext_para = &ext_params_60hz;
	else if (mode == 3)
		ext_para = &ext_params_50hz;
	else if (mode == 4)
		ext_para = &ext_params_48hz;
	else if (mode == 5)
		ext_para = &ext_params_45hz;
	else if (mode == 6)
		ext_para = &ext_params_30hz;
// #endif
	else
		ret = 1;

	return ret;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.ata_check = panel_ata_check,
	.get_virtual_heigh = lcm_get_virtual_heigh,
	.get_virtual_width = lcm_get_virtual_width,
	.cabc_switch = cabc_switch,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
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
	struct drm_display_mode *mode2;
	struct drm_display_mode *mode3;
	struct drm_display_mode *mode4;
	struct drm_display_mode *mode5;
	struct drm_display_mode *mode6;
	struct drm_display_mode *mode7;

	mode = drm_mode_duplicate(panel->drm, &performance_mode_120);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			performance_mode_120.hdisplay, performance_mode_120.vdisplay,
			performance_mode_120.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

	mode2 = drm_mode_duplicate(panel->drm, &default_mode_30);
	if (!mode2) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode_30.hdisplay,
			default_mode_30.vdisplay,
			default_mode_30.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode2);

	mode3 = drm_mode_duplicate(panel->drm, &default_mode_45);
	if (!mode3) {
		dev_info(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			 default_mode_45.hdisplay, default_mode_45.vdisplay,
			 default_mode_45.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode3);
	mode3->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode3);

	mode4 = drm_mode_duplicate(panel->drm, &default_mode_48);
	if (!mode4) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode_48.hdisplay, default_mode_48.vdisplay,
			default_mode_48.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode4);
	mode4->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode4);


	mode5 = drm_mode_duplicate(panel->drm, &default_mode_50);
	if (!mode5) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode_50.hdisplay, default_mode_50.vdisplay,
			default_mode_50.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode5);
	mode5->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode5);


	mode6 = drm_mode_duplicate(panel->drm, &default_mode_60);
	if (!mode6) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode_60.hdisplay, default_mode_60.vdisplay,
			default_mode_60.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode6);
	mode6->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode6);


	mode7 = drm_mode_duplicate(panel->drm, &performance_mode_90);
	if (!mode7) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			performance_mode_90.hdisplay, performance_mode_90.vdisplay,
			performance_mode_90.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode7);
	mode7->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode7);

	panel->connector->display_info.width_mm = 68;
	panel->connector->display_info.height_mm = 152;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static void dyn_timer_fn(struct timer_list *arg)
{
	pr_info("\n");
	ext_params_120hz.dyn.switch_en = 1;
	ext_params_90hz.dyn.switch_en = 1;
	ext_params_60hz.dyn.switch_en = 1;
	ext_params_50hz.dyn.switch_en = 1;
	ext_params_48hz.dyn.switch_en = 1;
	ext_params_45hz.dyn.switch_en = 1;
	ext_params_30hz.dyn.switch_en = 1;
}

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
#if (defined(OPLUS_BUG_STABILITY) && defined(CONFIG_MFD_MT6370_PMU))
	pmic_custom_flag = 1;
#endif
	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

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
#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_enable();
#else
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
#endif

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
	ret = mtk_panel_ext_create(dev, &ext_params_120hz, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
// #ifdef OPLUS_BUG_COMPATIBILITY
	disp_aal_set_dre_en(1);
// #endif
	register_device_proc("lcd","ili7807s_boe_boe","BOE");
	timer_setup(&dyn_timer, dyn_timer_fn, 0);
	mod_timer(&dyn_timer, jiffies + (60*HZ));

	pr_info("%s-\n", __func__);

	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);

	del_timer_sync(&dyn_timer);
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "boe,boe,ili7807s,vdo", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-boe-boe-ili7807s-vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Tai-Hua Tseng <tai-hua.tseng@mediatek.com>");
MODULE_DESCRIPTION("boe boe ili7807s VDO LCD Panel Driver");
MODULE_LICENSE("GPL v2");

