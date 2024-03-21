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
static int esd_brightness = 1023;
static int last_brightness = 0;
extern unsigned long oplus_max_normal_brightness;
extern void disp_aal_set_dre_en(int enable);
/* #ifdef OPLUS_BUG_STABILITY */
extern int __attribute__((weak)) tp_gesture_enable_flag(void) {return 0;};
extern void __attribute__((weak)) lcd_queue_load_tp_fw(void) {return;};
void __attribute__((weak)) switch_spi7cs_state(bool normal) {return;}
extern int _20015_lcm_i2c_write_bytes(unsigned char addr, unsigned char value);
/* #endif */ /* OPLUS_BUG_STABILITY */

/*****************************************************************************
 * Data Structure
 *****************************************************************************/

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

    	/*CABC begin*/
    	tianma_dcs_write_seq_static(ctx, 0xFF, 0x23);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x00, 0x80);
	tianma_dcs_write_seq_static(ctx, 0x01, 0x80);
	tianma_dcs_write_seq_static(ctx, 0x11, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x12, 0x95);
	tianma_dcs_write_seq_static(ctx, 0x15, 0x68);
	tianma_dcs_write_seq_static(ctx, 0x16, 0x0B);
	//Max gain
	tianma_dcs_write_seq_static(ctx, 0x0A, 0xFF);
        tianma_dcs_write_seq_static(ctx, 0x0B, 0xFF);
        tianma_dcs_write_seq_static(ctx, 0x0C, 0xFF);
        tianma_dcs_write_seq_static(ctx, 0x0D, 0x3F);
	//Image_Compensation
	tianma_dcs_write_seq_static(ctx, 0x19, 0x09);
        tianma_dcs_write_seq_static(ctx, 0x1A, 0x09);
        tianma_dcs_write_seq_static(ctx, 0x1B, 0x09);
        tianma_dcs_write_seq_static(ctx, 0x1C, 0x09);
        tianma_dcs_write_seq_static(ctx, 0x1D, 0x09);
        tianma_dcs_write_seq_static(ctx, 0x1E, 0x09);
        tianma_dcs_write_seq_static(ctx, 0x1F, 0x0A);
        tianma_dcs_write_seq_static(ctx, 0x20, 0x0B);
        tianma_dcs_write_seq_static(ctx, 0x21, 0x0C);
        tianma_dcs_write_seq_static(ctx, 0x22, 0x0E);
        tianma_dcs_write_seq_static(ctx, 0x23, 0x0F);
        tianma_dcs_write_seq_static(ctx, 0x24, 0x10);
        tianma_dcs_write_seq_static(ctx, 0x25, 0x11);
        tianma_dcs_write_seq_static(ctx, 0x26, 0x12);
        tianma_dcs_write_seq_static(ctx, 0x27, 0x12);
        tianma_dcs_write_seq_static(ctx, 0x28, 0x12);
        tianma_dcs_write_seq_static(ctx, 0x29, 0x0D);
        tianma_dcs_write_seq_static(ctx, 0x2A, 0x22);
        tianma_dcs_write_seq_static(ctx, 0x2B, 0x2B);
	//PWM_5500h_0x01
	tianma_dcs_write_seq_static(ctx, 0x30, 0xFC);
	tianma_dcs_write_seq_static(ctx, 0x31, 0xFA);
    	tianma_dcs_write_seq_static(ctx, 0x32, 0xED);
    	tianma_dcs_write_seq_static(ctx, 0x33, 0xEC);
    	tianma_dcs_write_seq_static(ctx, 0x34, 0xEB);
    	tianma_dcs_write_seq_static(ctx, 0x35, 0xEA);
    	tianma_dcs_write_seq_static(ctx, 0x36, 0xE9);
	tianma_dcs_write_seq_static(ctx, 0x37, 0xE8);
    	tianma_dcs_write_seq_static(ctx, 0x38, 0xE6);
    	tianma_dcs_write_seq_static(ctx, 0x39, 0xE5);
    	tianma_dcs_write_seq_static(ctx, 0x3A, 0xE4);
    	tianma_dcs_write_seq_static(ctx, 0x3B, 0xE3);
    	tianma_dcs_write_seq_static(ctx, 0x3D, 0xE2);
    	tianma_dcs_write_seq_static(ctx, 0x3F, 0xE0);
    	tianma_dcs_write_seq_static(ctx, 0x40, 0xDE);
    	tianma_dcs_write_seq_static(ctx, 0x41, 0xDB);
	//PWM_5500h_02
	tianma_dcs_write_seq_static(ctx, 0x45, 0xE8);
	tianma_dcs_write_seq_static(ctx, 0x46, 0xDD);
    	tianma_dcs_write_seq_static(ctx, 0x47, 0xD4);
    	tianma_dcs_write_seq_static(ctx, 0x48, 0xD2);
    	tianma_dcs_write_seq_static(ctx, 0x49, 0xCF);
    	tianma_dcs_write_seq_static(ctx, 0x4A, 0xCA);
   	tianma_dcs_write_seq_static(ctx, 0x4B, 0xC6);
    	tianma_dcs_write_seq_static(ctx, 0x4C, 0xC0);
    	tianma_dcs_write_seq_static(ctx, 0x4D, 0xBD);
    	tianma_dcs_write_seq_static(ctx, 0x4E, 0xBA);
    	tianma_dcs_write_seq_static(ctx, 0x4F, 0xB8);
    	tianma_dcs_write_seq_static(ctx, 0x50, 0xB7);
    	tianma_dcs_write_seq_static(ctx, 0x51, 0xB5);
    	tianma_dcs_write_seq_static(ctx, 0x52, 0xB3);
    	tianma_dcs_write_seq_static(ctx, 0x53, 0xB0);
    	tianma_dcs_write_seq_static(ctx, 0x54, 0xAD);
	//PWM_5500h_03
	tianma_dcs_write_seq_static(ctx, 0x58, 0xE4);
	tianma_dcs_write_seq_static(ctx, 0x59, 0xD8);
    	tianma_dcs_write_seq_static(ctx, 0x5A, 0xCE);
    	tianma_dcs_write_seq_static(ctx, 0x5B, 0xCA);
    	tianma_dcs_write_seq_static(ctx, 0x5C, 0xC6);
    	tianma_dcs_write_seq_static(ctx, 0x5D, 0xC2);
    	tianma_dcs_write_seq_static(ctx, 0x5E, 0xBC);
    	tianma_dcs_write_seq_static(ctx, 0x5F, 0xB5);
    	tianma_dcs_write_seq_static(ctx, 0x60, 0xAF);
    	tianma_dcs_write_seq_static(ctx, 0x61, 0xA9);
    	tianma_dcs_write_seq_static(ctx, 0x62, 0xA4);
    	tianma_dcs_write_seq_static(ctx, 0x63, 0x9F);
    	tianma_dcs_write_seq_static(ctx, 0x64, 0x9B);
    	tianma_dcs_write_seq_static(ctx, 0x65, 0x98);
    	tianma_dcs_write_seq_static(ctx, 0x66, 0x94);
    	tianma_dcs_write_seq_static(ctx, 0x67, 0x90);
    	/*CABC end*/

	tianma_dcs_write_seq_static(ctx, 0xFF, 0x23);
        tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
        tianma_dcs_write_seq_static(ctx, 0x05, 0x36);
        tianma_dcs_write_seq_static(ctx, 0x06, 0x02);

	tianma_dcs_write_seq_static(ctx, 0xFF, 0x10);
        tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
        tianma_dcs_write_seq_static(ctx, 0x53, 0x24);
	tianma_dcs_write_seq_static(ctx, 0x55,0x01);
	tianma_dcs_write_seq_static(ctx, 0x68, 0x00, 0x00);

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

	usleep_range(20 * 1000, 20 * 1000);
	ctx->enabled = false;

	return 0;
}

static int tianma_unprepare(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tianma(panel);
	pr_info("%s\n", __func__);

	if (!ctx->prepared)
		return 0;

	tianma_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	tianma_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(60 * 1000, 60 * 1000);
/*
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
*/

/* #ifdef OPLUS_BUG_STABILITY */
    pr_info("%s: tp_gesture_enable_flag = %d \n", __func__, tp_gesture_enable_flag());
    if (0 == tp_gesture_enable_flag() || (esd_flag == 1)) {
/* #endif */ /* OPLUS_BUG_STABILITY */
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
	usleep_range(140 * 1000, 140 * 1000);
        //ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
        //gpiod_set_value(ctx->reset_gpio, 0);
        //devm_gpiod_put(ctx->dev, ctx->reset_gpio);
        //usleep_range(2 * 1000, 2 * 1000);

	if( ctx->is_normal_mode ){
		switch_spi7cs_state(false);
	}
	//add for ldo
/*	ctx->bias_en = devm_gpiod_get(ctx->dev, "ldo", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_en, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_en);
	usleep_range(20 * 1000, 20 * 1000);*/
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
	_20015_lcm_i2c_write_bytes(0x0, 0xf);
	_20015_lcm_i2c_write_bytes(0x1, 0xf);
	msleep(2);
/* #ifdef OPLUS_BUG_STABILITY */
//	lcd_queue_load_tp_fw();
/* #endif */ /* OPLUS_BUG_STABILITY */
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
		usleep_range(30 * 1000, 30 * 1000);
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
//	tianma_vdd_tp_start(ctx);
	usleep_range(10 * 1000, 10 * 1000);
//end
//	tianma_vdd_tp_end(ctx);
/* #ifdef OPLUS_BUG_STABILITY */
    if(ctx->is_normal_mode) {
        lcd_queue_load_tp_fw();
    }
/* #endif */ /* OPLUS_BUG_STABILITY */
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
	.clock = 300591,
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
	.clock = 300591,
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
		.vfp = 1296,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.oplus_display_global_dre = 1,
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 548,
	.vfp_low_power = 1296,
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
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.oplus_display_global_dre = 1,
};

static int map_exp[4096] = {0};

static void init_global_exp_backlight(void)
{
        int lut_index[41] = {0, 4, 99, 144, 187, 227, 264, 300, 334, 366, 397, 427, 456, 484, 511, 537, 563, 587, 611, 635, 658, 680,
                                                702, 723, 744, 764, 784, 804, 823, 842, 861, 879, 897, 915, 933, 950, 967, 984, 1000, 1016, 1023};
        int lut_value1[41] = {4, 4, 6, 14, 24, 37, 52, 69, 87, 107, 128, 150, 173, 197, 222, 248, 275, 302, 330, 358, 387, 416, 446,
                                                477, 507, 539, 570, 602, 634, 667, 700, 733, 767, 801, 835, 869, 903, 938, 973, 1008, 1023};
        int index_start = 0, index_end = 0;
        int value1_start = 0, value1_end = 0;
        int i,j;
        int index_len = sizeof(lut_index) / sizeof(int);
        int value_len = sizeof(lut_value1) / sizeof(int);
        if (index_len == value_len) {
                for (i = 0; i < index_len - 1; i++) {
                        index_start = lut_index[i] * oplus_max_normal_brightness / 1023;
                        index_end = lut_index[i+1] * oplus_max_normal_brightness / 1023;
                        value1_start = lut_value1[i] * oplus_max_normal_brightness / 1023;
                        value1_end = lut_value1[i+1] * oplus_max_normal_brightness / 1023;
                        for (j = index_start; j <= index_end; j++) {
                                map_exp[j] = value1_start + (value1_end - value1_start) * (j - index_start) / (index_end - index_start);
                        }
                }
        }
}

static int tianma_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0x07, 0xff, 0x00};

	if (level > 4095)
		level = 4095;
	pr_err("%s backlight = -%d\n", __func__, level);
	if ((level > 0) && (level < oplus_max_normal_brightness)) {
		level = map_exp[level];
	}
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0) {
                level = 2047;
        }
	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;
	esd_brightness = level;
	if (last_brightness == 0) {
                msleep(15);
	}
	if (!cb)
		return -1;

	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
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

	if (mode == 0)
		ext->params = &ext_params;
	else if (mode == 1)
		ext->params = &ext_params_90hz;
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
	char bl_tb3[] = {0xFF, 0x23}; //cmd page3
        char bl_tb4[] = {0x03, 0x0};
	char bl_tb5[] = {0x03, 0xC0};
	char bl_tb6[] = {0x53, 0x2C};
	char bl_tb7[] = {0x01, 0x84};

	pr_err("%s cabc = %d\n", __func__, cabc_mode);
	bl_tb0[1] = (u8)cabc_mode;
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
	if (0 == cabc_mode) {
		cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
		cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
                cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
		cb(dsi, handle, bl_tb7, ARRAY_SIZE(bl_tb7));
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	}else {
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
		cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
		cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

		cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
                cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
		cb(dsi, handle, bl_tb7, ARRAY_SIZE(bl_tb7));

		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
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

        oplus_max_normal_brightness = 3330;
        init_global_exp_backlight();
	disp_aal_set_dre_en(1);
	register_device_proc("lcd", "NT36672C_TM_ALICE_R", "tianma2048");
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
		.compatible = "oplus21639,tianma,nt36672c,cphy,vdo",
	},
	{} };

MODULE_DEVICE_TABLE(of, tianma_of_match);

static struct mipi_dsi_driver tianma_driver = {
	.probe = tianma_probe,
	.remove = tianma_remove,
	.driver = {

			.name = "oplus21639_nt36672c_tm_cphy_dsi_vdo_lcm_drv",
			.owner = THIS_MODULE,
			.of_match_table = tianma_of_match,
		},
};

module_mipi_dsi_driver(tianma_driver);

MODULE_AUTHOR("Elon Hsu <elon.hsu@mediatek.com>");
MODULE_DESCRIPTION("tianma r66451 CMD AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
