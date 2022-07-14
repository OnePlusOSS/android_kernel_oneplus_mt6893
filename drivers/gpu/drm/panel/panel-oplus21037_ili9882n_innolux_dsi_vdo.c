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
#include <linux/gpio.h>

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
	pr_info("SYQ innolux %s+\n", __func__);

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98,0x82,0x02);
	lcm_dcs_write_seq_static(ctx, 0x01, 0x15);
        lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x03);
        lcm_dcs_write_seq_static(ctx, 0x83, 0x20);
        lcm_dcs_write_seq_static(ctx, 0x84, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x03);
	lcm_dcs_write_seq_static(ctx, 0x86, 0x6C);
        lcm_dcs_write_seq_static(ctx, 0x88, 0xE1);
        lcm_dcs_write_seq_static(ctx, 0x89, 0xE8);
        lcm_dcs_write_seq_static(ctx, 0x8A, 0xF0);
        lcm_dcs_write_seq_static(ctx, 0x8B, 0xF7);
        lcm_dcs_write_seq_static(ctx, 0x8C, 0xBF);
        lcm_dcs_write_seq_static(ctx, 0x8D, 0xC5);
        lcm_dcs_write_seq_static(ctx, 0x8E, 0xC8);
        lcm_dcs_write_seq_static(ctx, 0x8F, 0xCE);
        lcm_dcs_write_seq_static(ctx, 0x90, 0xD1);
        lcm_dcs_write_seq_static(ctx, 0x91, 0xD6);
        lcm_dcs_write_seq_static(ctx, 0x92, 0xDC);
        lcm_dcs_write_seq_static(ctx, 0x93, 0xE3);
        lcm_dcs_write_seq_static(ctx, 0x94, 0xED);
        lcm_dcs_write_seq_static(ctx, 0x95, 0xFA);
	lcm_dcs_write_seq_static(ctx, 0x95, 0xFA);
	lcm_dcs_write_seq_static(ctx, 0xAE, 0xDA);
	lcm_dcs_write_seq_static(ctx, 0x96, 0x7A);
	lcm_dcs_write_seq_static(ctx, 0x97, 0x89);
	lcm_dcs_write_seq_static(ctx, 0x98, 0x99);
	lcm_dcs_write_seq_static(ctx, 0x99, 0xA5);
	lcm_dcs_write_seq_static(ctx, 0x9A, 0xAE);
	lcm_dcs_write_seq_static(ctx, 0x9B, 0xAF);
	lcm_dcs_write_seq_static(ctx, 0x9C, 0xB0);
	lcm_dcs_write_seq_static(ctx, 0x9D, 0xB3);
	lcm_dcs_write_seq_static(ctx, 0x9E, 0xB7);
	lcm_dcs_write_seq_static(ctx, 0x9F, 0xCC);
	lcm_dcs_write_seq_static(ctx, 0xB5, 0xB0);
	lcm_dcs_write_seq_static(ctx, 0xAF, 0x18);
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x08);
	lcm_dcs_write_seq_static(ctx, 0xE0, 0x00, 0x09, 0x3B);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x62, 0x94, 0x50);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0xC1, 0xE6, 0x14);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x38, 0x55, 0x76);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0xA7, 0xD3, 0xFE);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0xAA, 0x2B, 0x62);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x87, 0xB4, 0xFE);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0xDD, 0x0F, 0x54);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x81, 0x03, 0x84);
	lcm_dcs_write_seq_static(ctx, 0xE1, 0x00, 0x09, 0x3B);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x62, 0x94, 0x50);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0xC1, 0xE6, 0x14);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x38, 0x55, 0x76);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0xA7, 0xD3, 0xFE);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0xAA, 0x2B, 0x62);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x87, 0xB4, 0xFE);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0xDD, 0x0F, 0x54);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x81, 0x03, 0xCC);
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x06);
	lcm_dcs_write_seq_static(ctx, 0x06, 0xA4);
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x53, 0x24);
	lcm_dcs_write_seq_static(ctx, 0x55, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x35, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x11, 0x00);
	msleep(60);
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x29, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x0F);
	msleep(20);
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
	int tp_irq = 0;
	pr_info("%s\n", __func__);

	if (!ctx->prepared)
		return 0;
	msleep(5);
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(20);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(80);

	ctx->error = 0;
	ctx->prepared = false;

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_disable();
#else
	if(g_shutdown_flag == 1) {
		ctx->reset_gpio =
			devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->reset_gpio)) {
			dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
				__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
		gpiod_set_value(ctx->reset_gpio, 0);
		devm_gpiod_put(ctx->dev, ctx->reset_gpio);
		udelay(2000);
	}
	pr_info("%s: tp_gesture_enable_flag = %d \n", __func__, tp_gesture_enable_flag());
	if ((0 == tp_gesture_enable_flag()) || (esd_flag == 1) || (g_shutdown_flag == 1)) {
		pr_info("%s: going to cut off power \n", __func__);
		if(esd_flag != 1) {
			tp_irq = gpio_to_irq(316);
			pr_info("tp_irq:%d\n", tp_irq);
			disable_irq_nosync(tp_irq);
		}
		if(g_shutdown_flag == 1) {
			tp_gpio_current_leakage_handler(false);
			udelay(2000);
		}
		ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
			"bias", 1, GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->bias_neg)) {
			dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
				__func__, PTR_ERR(ctx->bias_neg));
			return PTR_ERR(ctx->bias_neg);
		}
		gpiod_set_value(ctx->bias_neg, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_neg);

		udelay(2000);

		ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
			"bias", 0, GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->bias_pos)) {
			dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
				__func__, PTR_ERR(ctx->bias_pos));
			return PTR_ERR(ctx->bias_pos);
		}
	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
/*        usleep_range(2 * 1000, 2 * 1000);
        ctx->bias_en = devm_gpiod_get(ctx->dev, "ldo", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->bias_en, 0);
        devm_gpiod_put(ctx->dev, ctx->bias_en);*/
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
/*	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	usleep_range(5 * 1000, 5 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5 * 1000, 5 * 1000);*/
/*	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(2000);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	_lcm_i2c_write_bytes(0x0, 0xf);
	_lcm_i2c_write_bytes(0x1, 0xf);
    msleep(10);*/
#endif

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5 * 1000, 5 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(10 * 1000, 10 * 1000);
	if (tp_boot_mode_normal()) {
		tp_gpio_current_leakage_handler(true);
		lcd_queue_load_tp_fw();
	}
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
	.hsync_start = HAC + 26,//HFP
	.hsync_end = HAC + 26 + 8,//HSA
	.htotal = HAC + 26 + 8 + 17,//HBP1289
	.vdisplay = VAC,
	.vsync_start = VAC + 243,//VFP
	.vsync_end = VAC + 243 + 2,//VSA
	.vtotal = VAC + 243 + 2 + 16,//VBP4948
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
		.hbp = 12,
	},
	.oplus_teot_ns_multiplier = 90,
	.vendor = "21037_ili9882n_innolux",
	.manufacture = "INNOLUX",
};

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0xFF, 0xFF};
	char bl_tb1[] = {0xFF, 0x98, 0x82, 0x00};
	char bl_tb2[] = {0xFF,0x98,0x82,0x08};
	char bl_tb3[] = {0xE0,0x55,0x3F,0x48,0x50,0x5F,0x55,0x6E,0x7D,0x92,0xA5,0xA5,0xC9,0xEA,0x0C,0x30,0xAA,0x59,0x8E,0xAE,0xD9,0xFE,0xFB,0x2A,0x63,0x80,0x03,0x84};
	char bl_tb4[] = {0xE1,0x55,0x3F,0x48,0x50,0x5E,0x55,0x6D,0x7D,0x92,0xA5,0xA5,0xC9,0xEA,0x0C,0x30,0xAA,0x59,0x8E,0xAE,0xD9,0xFE,0xFB,0x2A,0x63,0x80,0x03,0xCC};
	char bl_tb5[] = {0xE0,0x00,0x09,0x3B,0x62,0x94,0x50,0xC1,0xE6,0x14,0x38,0x55,0x76,0xA7,0xD3,0xFE,0xAA,0x2B,0x62,0x87,0xB4,0xFE,0xDD,0x0F,0x54,0x81,0x03,0x84};
	char bl_tb6[] = {0xE1,0x00,0x09,0x3B,0x62,0x94,0x50,0xC1,0xE6,0x14,0x38,0x55,0x76,0xA7,0xD3,0xFE,0xAA,0x2B,0x62,0x87,0xB4,0xFE,0xDD,0x0F,0x54,0x81,0x03,0xCC};
	char bl_tb7[] = {0xFF, 0x98, 0x82, 0x0F};
	pr_err("%s backlight = %d\n", __func__, level);
	if (level > 4095)
                level = 4095;
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0) {
		level = 2047;
	}
	if(level < 14 && level > 0){
		backlight_gamma = 1;
		cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
		cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
		cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
	}else if(level > 13 && backlight_gamma == 1){
		backlight_gamma = 0;
		cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
		cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
		cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
	}
	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;
	esd_brightness = level;
	if (!cb)
		return -1;
        if (last_brightness == 0)
                msleep(15);

	pr_err("%s SYQ bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	//return 0;
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	cb(dsi, handle, bl_tb7, ARRAY_SIZE(bl_tb7));
	last_brightness = level;
	return 0;
}

static int oplus_esd_backlight_check(void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0x51, 0x07, 0xff};
	char bl_tb1[] = {0xFF, 0x98, 0x82, 0x00};
	char bl_tb2[] = {0xFF, 0x98, 0x82, 0x0F};

	pr_err("%s esd_backlight = %d\n", __func__, esd_brightness);
	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
	if (!cb)
		return -1;
	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
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
	char bl_tb1[] = {0xFF, 0x98, 0x82, 0x00};
	char bl_tb3[] = {0x53, 0x2C};
	char bl_tb4[] = {0xFF, 0x98, 0x82, 0x0F};
	pr_err("%s cabc = %d\n", __func__, cabc_mode);
	if(cabc_mode == 3)
		cabc_mode = 2;
	bl_tb0[1] = (u8)cabc_mode;
    msleep(5);
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));//FB 01
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));//55 0X
	cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
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
	dsi->lanes = 3;
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
	register_device_proc("lcd","21037_ili9882n_innolux","INNOLUX");
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
		.compatible = "oplus21037_ili9882n_innolux_vdo",
	},
	{} };

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
			.name = "oplus21037_ili9882n_innolux_vdo_lcm_drv",
			.owner = THIS_MODULE,
			.of_match_table = jdi_of_match,
		},
};

module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("Shanguofan");
MODULE_DESCRIPTION("ili9882n innolux VDO LCD Panel Driver");
MODULE_LICENSE("GPL v2");
