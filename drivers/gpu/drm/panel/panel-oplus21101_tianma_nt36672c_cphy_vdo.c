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
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif

/* #ifdef OPLUS_BUG_STABILITY */
#include <mt-plat/mtk_boot_common.h>
/* #endif */ /* OPLUS_BUG_STABILITY */


/*****************************************************************************
 * Function Prototype
 *****************************************************************************/

extern unsigned long esd_flag;
extern unsigned int g_shutdown_flag;
static int esd_brightness = 1023;
static int last_brightness = 0;
extern unsigned long oplus_max_normal_brightness;
extern void disp_aal_set_dre_en(int enable);
/* #ifdef OPLUS_BUG_STABILITY */
extern int __attribute__((weak)) tp_gesture_enable_flag(void) {return 0;};
extern void __attribute__((weak)) lcd_queue_load_tp_fw(void) {return;};
void __attribute__((weak)) switch_spi7cs_state(bool normal) {return;};
extern int _20015_lcm_i2c_write_bytes(unsigned char addr, unsigned char value);
/* #endif */ /* OPLUS_BUG_STABILITY */

/*****************************************************************************
 * Data Structure
 *****************************************************************************/
static int cabc_lastlevel = 0;

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

static void tianma_panel_init(struct tianma *ctx)
{
/* #ifdef OPLUS_BUG_STABILITY */
//add for cabc
	tianma_dcs_write_seq_static(ctx, 0xFF, 0x25);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x18, 0x21);
	tianma_dcs_write_seq_static(ctx, 0xFF, 0xC0);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x9C, 0x11);
	tianma_dcs_write_seq_static(ctx, 0x9D, 0x11);
	tianma_dcs_write_seq_static(ctx, 0xFF, 0xD0);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x53, 0x22);
	tianma_dcs_write_seq_static(ctx, 0x54, 0x02);
	tianma_dcs_write_seq_static(ctx, 0xFF, 0xE0);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
    	tianma_dcs_write_seq_static(ctx, 0x35, 0x82);
    	tianma_dcs_write_seq_static(ctx, 0xFF, 0xF0);
    	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x1C, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x33, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x5A, 0x00);
	tianma_dcs_write_seq_static(ctx, 0xD2, 0x52);
	tianma_dcs_write_seq_static(ctx, 0xFF, 0x10);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x3B, 0x03,0x14,0x36,0x04,0x04);
	tianma_dcs_write_seq_static(ctx, 0xB0, 0x00);
	tianma_dcs_write_seq_static(ctx, 0xC0, 0x00);
	tianma_dcs_write_seq_static(ctx, 0x51, 0x0, 0x0);
	tianma_dcs_write_seq_static(ctx, 0x53, 0x24);

        tianma_dcs_write_seq_static(ctx, 0xFF,0xF0);
        tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
        tianma_dcs_write_seq_static(ctx, 0x5A,0x00);
        tianma_dcs_write_seq_static(ctx, 0xFF,0x10);
        tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
// endif

//CABC
	tianma_dcs_write_seq_static(ctx, 0xFF,0x23);
	tianma_dcs_write_seq_static(ctx, 0xFB,0x01);
	tianma_dcs_write_seq_static(ctx, 0x00,0x80);
	tianma_dcs_write_seq_static(ctx, 0x05,0x22);
	tianma_dcs_write_seq_static(ctx, 0x06,0x01);
	tianma_dcs_write_seq_static(ctx, 0x07,0x00);
	tianma_dcs_write_seq_static(ctx, 0x08,0x01);
	tianma_dcs_write_seq_static(ctx, 0x09,0x00);
	tianma_dcs_write_seq_static(ctx, 0x10,0x82);
	tianma_dcs_write_seq_static(ctx, 0x11,0x01);
	tianma_dcs_write_seq_static(ctx, 0x12,0x95);
	tianma_dcs_write_seq_static(ctx, 0x15,0x68);
	tianma_dcs_write_seq_static(ctx, 0x16,0x0B);
//CABC_PWM_UI
	tianma_dcs_write_seq_static(ctx, 0x30,0xFF);
	tianma_dcs_write_seq_static(ctx, 0x31,0xFD);
	tianma_dcs_write_seq_static(ctx, 0x32,0xFA);
	tianma_dcs_write_seq_static(ctx, 0x33,0xF7);
	tianma_dcs_write_seq_static(ctx, 0x34,0xF4);
	tianma_dcs_write_seq_static(ctx, 0x35,0xF0);
	tianma_dcs_write_seq_static(ctx, 0x36,0xED);
	tianma_dcs_write_seq_static(ctx, 0x37,0xEC);
	tianma_dcs_write_seq_static(ctx, 0x38,0xEB);
	tianma_dcs_write_seq_static(ctx, 0x39,0xEA);
	tianma_dcs_write_seq_static(ctx, 0x3A,0xE9);
	tianma_dcs_write_seq_static(ctx, 0x3B,0xE8);
	tianma_dcs_write_seq_static(ctx, 0x3D,0xE7);
	tianma_dcs_write_seq_static(ctx, 0x3F,0xE6);
	tianma_dcs_write_seq_static(ctx, 0x40,0xE5);
	tianma_dcs_write_seq_static(ctx, 0x41,0xE4);
//CABC_PWM_STILL
	tianma_dcs_write_seq_static(ctx, 0x45,0xFF);
	tianma_dcs_write_seq_static(ctx, 0x46,0xFA);
	tianma_dcs_write_seq_static(ctx, 0x47,0xF2);
	tianma_dcs_write_seq_static(ctx, 0x48,0xE8);
	tianma_dcs_write_seq_static(ctx, 0x49,0xE4);
	tianma_dcs_write_seq_static(ctx, 0x4A,0xDC);
	tianma_dcs_write_seq_static(ctx, 0x4B,0xD7);
	tianma_dcs_write_seq_static(ctx, 0x4C,0xD5);
	tianma_dcs_write_seq_static(ctx, 0x4D,0xD3);
	tianma_dcs_write_seq_static(ctx, 0x4E,0xD2);
	tianma_dcs_write_seq_static(ctx, 0x4F,0xD0);
	tianma_dcs_write_seq_static(ctx, 0x50,0xCE);
	tianma_dcs_write_seq_static(ctx, 0x51,0xCD);
	tianma_dcs_write_seq_static(ctx, 0x52,0xCB);
	tianma_dcs_write_seq_static(ctx, 0x53,0xC6);
	tianma_dcs_write_seq_static(ctx, 0x54,0xC3);

	tianma_dcs_write_seq_static(ctx, 0xFF, 0x10);
        tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
        tianma_dcs_write_seq_static(ctx, 0x53, 0x24);
	tianma_dcs_write_seq_static(ctx, 0x55,0x01);

    	/*TE*/
	tianma_dcs_write_seq_static(ctx, 0x35,0x00);

	tianma_dcs_write_seq_static(ctx, 0x11);
	msleep(100);
	//Display On
	tianma_dcs_write_seq_static(ctx, 0x29);
	msleep(40);
	pr_info("%s-\n", __func__);
}

static int tianma_disable(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tianma(panel);

	if (!ctx->enabled)
		return 0;
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	usleep_range(20 * 1000, 20 * 1000 + 100);
	ctx->enabled = false;

	return 0;
}

static int tianma_unprepare(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tianma(panel);

	if (!ctx->prepared)
		return 0;

	tianma_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	tianma_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(60 * 1000, 60 * 1000 + 100);
	pr_info("%s: tp_gesture_enable_flag = %d g_shutdown_flag = %d \n", __func__,
			tp_gesture_enable_flag(), g_shutdown_flag);
	if (1 == g_shutdown_flag) {
		ctx->reset_gpio = devm_gpiod_get(ctx->dev,
				"reset", GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->reset_gpio)) {
			dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
				__func__, PTR_ERR(ctx->reset_gpio));
			return PTR_ERR(ctx->reset_gpio);
		}
		gpiod_set_value(ctx->reset_gpio, 0);
		devm_gpiod_put(ctx->dev, ctx->reset_gpio);

		usleep_range(2 * 1000, 2 * 1000 + 100);
	}
	if (0 == tp_gesture_enable_flag() || (esd_flag == 1) || (1 == g_shutdown_flag)) {
		ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
				"bias", 1, GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->bias_neg)) {
			dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
				__func__, PTR_ERR(ctx->bias_neg));
			return PTR_ERR(ctx->bias_neg);
		}
		gpiod_set_value(ctx->bias_neg, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_neg);

		usleep_range(2 * 1000, 2 * 1000 + 100);

		ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
				"bias", 0, GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->bias_pos)) {
			dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
				__func__, PTR_ERR(ctx->bias_pos));
			return PTR_ERR(ctx->bias_pos);
		}
		gpiod_set_value(ctx->bias_pos, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_pos);
		usleep_range(140 * 1000, 140 * 1000 + 100);
		if(ctx->is_normal_mode) {
			switch_spi7cs_state(false);
		}
	}

	ctx->error = 0;
	ctx->prepared = false;

	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tianma(panel);
	int ret;

	pr_info("%s+\n", __func__);

	//add for ldo
	ctx->bias_en = devm_gpiod_get(ctx->dev, "ldo", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_en, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_en);
	usleep_range(20 * 1000, 20 * 1000 + 100);
	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	usleep_range(20 * 1000, 20 * 1000 + 100);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	_20015_lcm_i2c_write_bytes(0x0, 0xf);
	_20015_lcm_i2c_write_bytes(0x1, 0xf);
	msleep(2);
	pr_info("%s-\n", __func__);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tianma(panel);
	int ret;

        pr_info("%s+\n", __func__);

	if (ctx->prepared)
                return 0;
	if (0 == tp_gesture_enable_flag() || (esd_flag == 1)) {
		ctx->bias_en = devm_gpiod_get(ctx->dev, "ldo", GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_en, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_en);
		usleep_range(30 * 1000, 30 * 1000 + 100);
	}

	return 0;
}

static int tianma_prepare(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tianma(panel);
	int ret;

	pr_info("%s+\n", __func__);
	if (ctx->prepared)
		return 0;

	usleep_range(2 * 1000, 2 * 1000);
	//NVT H -> L -> H -> L -> H
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);

	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(10 * 1000, 10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10 * 1000, 10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(10 * 1000, 10 * 1000);
	if(ctx->is_normal_mode) {
		switch_spi7cs_state(true);
	}
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(10 * 1000, 10 * 1000);
	if(ctx->is_normal_mode) {
		lcd_queue_load_tp_fw();
	}

	tianma_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		tianma_unprepare(panel);

	ctx->prepared = true;

#ifdef PANEL_SUPPORT_READBACK
	tianma_panel_get_data(ctx);
#endif

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
	.clock = 313152,
	.hdisplay = 1080,
	.hsync_start = 1080 + 276,
	.hsync_end = 1080 + 276 + 22,
	.htotal = 1080 + 276 + 22 + 22,
	.vdisplay = 2408,
	.vsync_start = 2408 + 1300,
	.vsync_end = 2408 + 1300 + 10,
	.vtotal = 2408 + 1300 + 10 + 10,
	.vrefresh = 60,
};

static const struct drm_display_mode performance_mode = {
	.clock = 312732,
	.hdisplay = 1080,
	.hsync_start = 1080 + 276,
	.hsync_end = 1080 + 276 + 22,
	.htotal = 1080 + 276 + 22 + 22,
	.vdisplay = 2408,
	.vsync_start = 2408 + 54,
	.vsync_end = 2408 + 54 + 10,
	.vtotal = 2408 + 54 + 10 + 10,
	.vrefresh = 90,
};

static const struct drm_display_mode performance_mode_30hz = {
	.clock = 313152,
	.hdisplay = 1080,
	.hsync_start = 1080 + 276,
	.hsync_end = 1080 + 276 + 22,
	.htotal = 1080 + 276 + 22 + 22,
	.vdisplay = 2408,
	.vsync_start = 2408 + 5028,
	.vsync_end = 2408 + 5028 + 10,
	.vtotal = 2408 + 5028 + 10 + 10,
	.vrefresh = 30,
};

static const struct drm_display_mode performance_mode_45hz = {
	.clock = 313110,
	.hdisplay = 1080,
	.hsync_start = 1080 + 276,
	.hsync_end = 1080 + 276 + 22,
	.htotal = 1080 + 276 + 22 + 22,
	.vdisplay = 2408,
	.vsync_start = 2408 + 2542,
	.vsync_end = 2408 + 2542 + 10,
	.vtotal = 2408 + 2542 + 10 + 10,
	.vrefresh = 45,
};

static const struct drm_display_mode performance_mode_48hz = {
	.clock = 313152,
	.hdisplay = 1080,
	.hsync_start = 1080 + 276,
	.hsync_end = 1080 + 276 + 22,
	.htotal = 1080 + 276 + 22 + 22,
	.vdisplay = 2408,
	.vsync_start = 2408 + 2232,
	.vsync_end = 2408 + 2232 + 10,
	.vtotal = 2408 + 2232 + 10 + 10,
	.vrefresh = 48,
};

static const struct drm_display_mode performance_mode_50hz = {
	.clock = 313040,
	.hdisplay = 1080,
	.hsync_start = 1080 + 276,
	.hsync_end = 1080 + 276 + 22,
	.htotal = 1080 + 276 + 22 + 22,
	.vdisplay = 2408,
	.vsync_start = 2408 + 2044,
	.vsync_end = 2408 + 2044 + 10,
	.vtotal = 2408 + 2044 + 10 + 10,
	.vrefresh = 50,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.pll_clk = 548,
	.vfp_low_power = 2538,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.is_cphy = 1,
	.lcm_esd_check_table[0] = {
	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9c,
	},
	.is_cphy = 1,
	.data_rate = 1096,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
	},
	.dyn = {
		.switch_en = 1,
		.pll_clk = 553,
		.hfp = 290,
		.vfp = 1300,
	},
	.phy_timcon = {
		.hs_prpr = 0x0B,
	},
	.vendor = "NT36672C_TM_Alice",
	.manufacture = "Alice_nt_tm3276",
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 548,
	.vfp_low_power = 1300,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.is_cphy = 1,
	.lcm_esd_check_table[0] = {
	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9c,
	},
	.is_cphy = 1,
	.data_rate = 1096,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
	},
	.dyn = {
		.switch_en = 1,
		.pll_clk = 553,
		.hfp = 290,
		.vfp = 54,
	},
	.phy_timcon = {
		.hs_prpr = 0x0B,
	},
        .vendor = "NT36672C_TM_Alice",
        .manufacture = "Alice_nt_tm3276",
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static struct mtk_panel_params ext_params_30hz = {
	.pll_clk = 548,
	.vfp_low_power = 5028,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.is_cphy = 1,
	.lcm_esd_check_table[0] = {
	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9c,
	},
	.is_cphy = 1,
	.data_rate = 1096,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
	},
	.dyn = {
		.switch_en = 1,
		.pll_clk = 553,
		.hfp = 290,
		.vfp = 5028,
	},
	.phy_timcon = {
		.hs_prpr = 0x0B,
	},
        .vendor = "NT36672C_TM_Alice",
        .manufacture = "Alice_nt_tm3276",
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static struct mtk_panel_params ext_params_45hz = {
	.pll_clk = 548,
	.vfp_low_power = 5028,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.is_cphy = 1,
	.lcm_esd_check_table[0] = {
	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9c,
	},
	.is_cphy = 1,
	.data_rate = 1096,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
	},
	.dyn = {
		.switch_en = 1,
		.pll_clk = 553,
		.hfp = 290,
		.vfp = 2542,
	},
	.phy_timcon = {
		.hs_prpr = 0x0B,
	},
        .vendor = "NT36672C_TM_Alice",
        .manufacture = "Alice_nt_tm3276",
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static struct mtk_panel_params ext_params_48hz = {
	.pll_clk = 548,
	.vfp_low_power = 2542,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.is_cphy = 1,
	.lcm_esd_check_table[0] = {
	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9c,
	},
	.is_cphy = 1,
	.data_rate = 1096,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
	},
	.dyn = {
		.switch_en = 1,
		.pll_clk = 553,
		.hfp = 290,
		.vfp = 2232,
	},
	.phy_timcon = {
		.hs_prpr = 0x0B,
	},
        .vendor = "NT36672C_TM_Alice",
        .manufacture = "Alice_nt_tm3276",
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static struct mtk_panel_params ext_params_50hz = {
	.pll_clk = 548,
	.vfp_low_power = 2232,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.is_cphy = 1,
	.lcm_esd_check_table[0] = {
	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9c,
	},
	.is_cphy = 1,
	.data_rate = 1096,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
	},
	.dyn = {
		.switch_en = 1,
		.pll_clk = 553,
		.hfp = 290,
		.vfp = 2044,
	},
	.phy_timcon = {
		.hs_prpr = 0x0B,
	},
        .vendor = "NT36672C_TM_Alice",
        .manufacture = "Alice_nt_tm3276",
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
	char bl_tb1[] = {0x55, 0x00};
	char bl_tb3[] = {0x53, 0x24};

	if (level > 4095)
                level = 4095;
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0) {
		level = 2047;
	}

	if(level == 0) {
		cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	}

	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;
	esd_brightness = level;
	if (last_brightness == 0) {
		usleep_range(15 * 1000, 15 * 1000 + 100);
	}
	if (!cb)
		return -1;

	pr_err("%s level=%d, bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, level, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	if ((last_brightness == 0) && (cabc_lastlevel != 0)) {
		bl_tb1[1] = cabc_lastlevel;
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
		cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	}
	last_brightness = level;

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

	if (ext && mode == 0)
		ext->params = &ext_params;
	else if (ext && mode == 1)
		ext->params = &ext_params_90hz;
	else if (ext && mode == 2)
		ext->params = &ext_params_30hz;
	else if (ext && mode == 3)
		ext->params = &ext_params_45hz;
	else if (ext && mode == 4)
		ext->params = &ext_params_48hz;
	else if (ext && mode == 5)
		ext->params = &ext_params_50hz;
	else
		ret = 1;

	return ret;
}

static int mtk_panel_ext_param_get(struct mtk_panel_params *ext_para,
			 unsigned int mode)
{
	int ret = 0;

	if (mode == 0)
		ext_para = &ext_params;
	else if (mode == 1)
		ext_para = &ext_params_90hz;
	else if (mode == 2)
		ext_para = &ext_params_30hz;
	else if (mode == 3)
		ext_para = &ext_params_45hz;
	else if (mode == 4)
		ext_para = &ext_params_48hz;
	else if (mode == 5)
		ext_para = &ext_params_50hz;
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

	pr_err("%s cabc = %d\n", __func__, cabc_mode);
	if(cabc_mode == 3)
                cabc_mode = 2;
	bl_tb0[1] = (u8)cabc_mode;
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	cabc_lastlevel = cabc_mode;
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
	.esd_backlight_recovery = oplus_esd_backlight_check,
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
	struct drm_display_mode *mode4;
	struct drm_display_mode *mode5;
	struct drm_display_mode *mode6;

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

	mode3 = drm_mode_duplicate(panel->drm, &performance_mode_30hz);
	if (!mode3) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			performance_mode_30hz.hdisplay,
			performance_mode_30hz.vdisplay,
			performance_mode_30hz.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode3);
	mode3->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode3);

	mode4 = drm_mode_duplicate(panel->drm, &performance_mode_45hz);
	if (!mode4) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			performance_mode_45hz.hdisplay,
			performance_mode_45hz.vdisplay,
			performance_mode_45hz.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode4);
	mode4->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode4);

	mode5 = drm_mode_duplicate(panel->drm, &performance_mode_48hz);
	if (!mode5) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			performance_mode_48hz.hdisplay,
			performance_mode_48hz.vdisplay,
			performance_mode_48hz.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode5);
	mode5->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode5);

	mode6 = drm_mode_duplicate(panel->drm, &performance_mode_50hz);
	if (!mode6) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			performance_mode_50hz.hdisplay,
			performance_mode_50hz.vdisplay,
			performance_mode_50hz.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode6);
	mode6->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode6);

	panel->connector->display_info.width_mm = 67;
	panel->connector->display_info.height_mm = 153;

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
	pr_info("%s+\n", __func__);
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

        oplus_max_normal_brightness = 3276;
	disp_aal_set_dre_en(1);
	register_device_proc("lcd", "NT36672C_TM_Alice", "Alice_nt_tm3276");
/* #ifdef OPLUS_BUG_STABILITY */
    ctx->is_normal_mode = true;
    if( META_BOOT == get_boot_mode() || FACTORY_BOOT == get_boot_mode() )
        ctx->is_normal_mode = false;
    pr_info("%s: is_normal_mode = %d \n", __func__, ctx->is_normal_mode);
/* #endif */ /* OPLUS_BUG_STABILITY */

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
		.compatible = "oplus21101,tianma,nt36672c,cphy,vdo",
	},
	{} };

MODULE_DEVICE_TABLE(of, tianma_of_match);

static struct mipi_dsi_driver tianma_driver = {
	.probe = tianma_probe,
	.remove = tianma_remove,
	.driver = {

			.name = "oplus21101_nt36672c_tm_cphy_dsi_vdo_lcm_drv",
			.owner = THIS_MODULE,
			.of_match_table = tianma_of_match,
		},
};

module_mipi_dsi_driver(tianma_driver);

MODULE_AUTHOR("Elon Hsu <elon.hsu@mediatek.com>");
MODULE_DESCRIPTION("tianma r66451 CMD AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
