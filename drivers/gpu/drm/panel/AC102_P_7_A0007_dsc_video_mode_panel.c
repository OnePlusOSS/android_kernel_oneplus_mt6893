// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <soc/oplus/device_info.h>
#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_panel_ext.h"
#include "../mediatek/mtk_log.h"
#include "../mediatek/mtk_drm_graphics_base.h"
#include "include/AC102_P_7_A0007_dsc_video_mode_panel.h"
#endif

#if defined(CONFIG_RT4831A_I2C)
#include "../../../misc/mediatek/gate_ic/gate_i2c.h"
#endif
#include "../oplus/oplus_display_mtk_debug.h"
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/AC102_P_7_A0007_data_hw_roundedpattern.h"
#endif
#define MAX_NORMAL_BRIGHTNESS    (3276)
extern unsigned long esd_flag;
extern unsigned int g_shutdown_flag;
extern unsigned int oplus_display_brightness;
extern unsigned long oplus_max_normal_brightness;
extern unsigned int oplus_lcm_display_on;
extern void __attribute((weak)) lcd_queue_load_tp_fw(void) { return; };
extern int __attribute__((weak)) tp_gesture_enable_flag(void) {return 0;};
extern void __attribute__((weak)) tp_gpio_current_leakage_handler(bool normal) {return;};
extern bool __attribute__((weak)) tp_boot_mode_normal() {return true;};

/* enable this to check panel self -bist pattern */
/* #define PANEL_BIST_PATTERN */

/* option function to read data from some panel address */
/* #define PANEL_SUPPORT_READBACK */

struct tianma {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pm_enable_gpio;
	struct gpio_desc *bias_ldo,*bias_pos, *bias_neg;
	bool prepared;
	bool enabled;

	int error;
};

static int current_fps = 60;
static int bl_gamma = 0;
static short cabc_status = 0;
static int esd_brightness;
static bool dimming_is_on = false;
static short frame_count = 0;

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
		DDPMSG("[error]error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void tianma_panel_get_data(struct tianma *ctx)
{
	u8 buffer[3] = {0};
	static int ret;
	if (ret == 0) {
		ret = tianma_dcs_read(ctx, 0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
static struct LCD_setting_table lcm_suspend_setting[] = {
	{0xFF, 2, {0xFF, 0x10} },
	{0x28, 2, {0x28, 0x00} },

};

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
		DDPMSG("[error]error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

#define HFP_SUPPORT 0

static void tianma_init_set_cabc(struct tianma *ctx, int cabc_mode)
{
	DISP_INFO("[lcm] init set cabc_mode %d", cabc_mode);
	tianma_dcs_write_seq_static(ctx, 0xFF, 0x10);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	if (cabc_mode == 0) {
		tianma_dcs_write_seq_static(ctx, 0x55, 0x00);
	} else if (cabc_mode == 1) {
		tianma_dcs_write_seq_static(ctx, 0x55, 0x01);
	} else if (cabc_mode == 2) {
		tianma_dcs_write_seq_static(ctx, 0x55, 0x02);
	} else if (cabc_mode == 3) {
		tianma_dcs_write_seq_static(ctx, 0x55, 0x02);
	} else {
		tianma_dcs_write_seq_static(ctx, 0x55, 0x00);
	}
}

static void tianma_panel_init(struct tianma *ctx)
{
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		DDPMSG("[error]%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}
	usleep_range(10 * 1000, 15 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(5 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(5 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(5 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	usleep_range(10 * 1000, 10 * 1000);
	/* Add for loading tp fw when screen ON*/
	lcd_queue_load_tp_fw();

	//CABC
	tianma_dcs_write_seq_static(ctx, 0xFF, 0x23);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x00, 0x80);
	tianma_dcs_write_seq_static(ctx, 0x05, 0x22);
	tianma_dcs_write_seq_static(ctx, 0x06, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x07, 0x00);
	tianma_dcs_write_seq_static(ctx, 0x08, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x09, 0x00);
	tianma_dcs_write_seq_static(ctx, 0x10, 0x82);
	tianma_dcs_write_seq_static(ctx, 0x11, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x12, 0x95);
	tianma_dcs_write_seq_static(ctx, 0x15, 0x68);
	tianma_dcs_write_seq_static(ctx, 0x16, 0x0B);
	//CABC_PWM_UI
	tianma_dcs_write_seq_static(ctx, 0x30, 0xFF);
	tianma_dcs_write_seq_static(ctx, 0x31, 0xFD);
	tianma_dcs_write_seq_static(ctx, 0x32, 0xFA);
	tianma_dcs_write_seq_static(ctx, 0x33, 0xF7);
	tianma_dcs_write_seq_static(ctx, 0x34, 0xF4);
	tianma_dcs_write_seq_static(ctx, 0x35, 0xF0);
	tianma_dcs_write_seq_static(ctx, 0x36, 0xED);
	tianma_dcs_write_seq_static(ctx, 0x36, 0xED);
	tianma_dcs_write_seq_static(ctx, 0x37, 0xEC);
	tianma_dcs_write_seq_static(ctx, 0x38, 0xEB);
	tianma_dcs_write_seq_static(ctx, 0x39, 0xEA);
	tianma_dcs_write_seq_static(ctx, 0x3A, 0xE9);
	tianma_dcs_write_seq_static(ctx, 0x3B, 0xE8);
	tianma_dcs_write_seq_static(ctx, 0x3D, 0xE7);
	tianma_dcs_write_seq_static(ctx, 0x3F, 0xE6);
	tianma_dcs_write_seq_static(ctx, 0x40, 0xE5);
	tianma_dcs_write_seq_static(ctx, 0x41, 0xE4);
	//CABC_PWM_STILL
	tianma_dcs_write_seq_static(ctx, 0x45, 0xFF);
	tianma_dcs_write_seq_static(ctx, 0x46, 0xFA);
	tianma_dcs_write_seq_static(ctx, 0x47, 0xF2);
	tianma_dcs_write_seq_static(ctx, 0x48, 0xE8);
	tianma_dcs_write_seq_static(ctx, 0x49, 0xE4);
	tianma_dcs_write_seq_static(ctx, 0x4A, 0xDC);
	tianma_dcs_write_seq_static(ctx, 0x4B, 0xD7);
	tianma_dcs_write_seq_static(ctx, 0x4C, 0xD5);
	tianma_dcs_write_seq_static(ctx, 0x4D, 0xD3);
	tianma_dcs_write_seq_static(ctx, 0x4E, 0xD2);
	tianma_dcs_write_seq_static(ctx, 0x4F, 0xD0);
	tianma_dcs_write_seq_static(ctx, 0x50, 0xCE);
	tianma_dcs_write_seq_static(ctx, 0x51, 0xCD);
	tianma_dcs_write_seq_static(ctx, 0x52, 0xCB);
	tianma_dcs_write_seq_static(ctx, 0x53, 0xC6);
	tianma_dcs_write_seq_static(ctx, 0x54, 0xC3);
	//CABC_PWM_MOV
	tianma_dcs_write_seq_static(ctx, 0x58, 0xFF);
	tianma_dcs_write_seq_static(ctx, 0x59, 0xF6);
	tianma_dcs_write_seq_static(ctx, 0x5A, 0xED);
	tianma_dcs_write_seq_static(ctx, 0x5B, 0xE5);
	tianma_dcs_write_seq_static(ctx, 0x5C, 0xE0);
	tianma_dcs_write_seq_static(ctx, 0x5D, 0xD9);
	tianma_dcs_write_seq_static(ctx, 0x5E, 0xD4);
	tianma_dcs_write_seq_static(ctx, 0x5F, 0xD0);
	tianma_dcs_write_seq_static(ctx, 0x60, 0xCB);
	tianma_dcs_write_seq_static(ctx, 0x61, 0xC7);
	tianma_dcs_write_seq_static(ctx, 0x62, 0xC4);
	tianma_dcs_write_seq_static(ctx, 0x63, 0xC0);
	tianma_dcs_write_seq_static(ctx, 0x64, 0xBB);
	tianma_dcs_write_seq_static(ctx, 0x65, 0xB8);
	tianma_dcs_write_seq_static(ctx, 0x66, 0xB4);
	tianma_dcs_write_seq_static(ctx, 0x67, 0xB0);
	//INIT
	tianma_dcs_write_seq_static(ctx, 0xFF, 0x25);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x18, 0x22);
	tianma_dcs_write_seq_static(ctx, 0xFF, 0xE0);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x35, 0x82);
	tianma_dcs_write_seq_static(ctx, 0xFF, 0xF0);
	tianma_dcs_write_seq_static(ctx, 0xFF, 0xF0);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x1C, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x33, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x5A, 0x00);
	tianma_dcs_write_seq_static(ctx, 0x9C, 0x17);
	tianma_dcs_write_seq_static(ctx, 0xFF, 0xD0);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x53, 0x22);
	tianma_dcs_write_seq_static(ctx, 0x54, 0x02);
	tianma_dcs_write_seq_static(ctx, 0xFF, 0xC0);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x9C, 0x11);
	tianma_dcs_write_seq_static(ctx, 0x9D, 0x11);
	tianma_dcs_write_seq_static(ctx, 0xFF, 0x25);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	tianma_dcs_write_seq_static(ctx, 0xD7, 0x82);
	tianma_dcs_write_seq_static(ctx, 0xDA, 0x03);
	tianma_dcs_write_seq_static(ctx, 0xDD, 0x02);
	tianma_dcs_write_seq_static(ctx, 0xE0, 0x03);
	tianma_dcs_write_seq_static(ctx, 0xFF, 0x10);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x3B, 0x03, 0x14, 0x36, 0x04, 0x04);
	tianma_dcs_write_seq_static(ctx, 0xB0, 0x00);
	tianma_dcs_write_seq_static(ctx, 0xC0, 0x03);
	tianma_dcs_write_seq_static(ctx, 0xC1, 0x89, 0x28, 0x00, 0x08, 0x00, 0xAA, 0x02, 0x0E, 0x00, 0x2b, 0x00, 0x07, 0x0d, 0xb7, 0x0c, 0xb7);
	usleep_range(3 * 1000, 3 * 1000);
	tianma_dcs_write_seq_static(ctx, 0xC2, 0x1B, 0xA0);
	tianma_dcs_write_seq_static(ctx, 0xFF, 0x10);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x35, 0x00);
	tianma_dcs_write_seq_static(ctx, 0x53, 0x24);
	tianma_dcs_write_seq_static(ctx, 0x53, 0x24);
	tianma_dcs_write_seq_static(ctx, 0x11);
	usleep_range(120 * 1000, 121 * 1000);
	tianma_dcs_write_seq_static(ctx, 0x29);
	usleep_range(20 * 1000, 21 * 1000);
	tianma_init_set_cabc(ctx, cabc_status);
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

	ctx->enabled = false;

	return 0;
}

static int tianma_unprepare(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tianma(panel);
	if (!ctx->prepared)
		return 0;

	usleep_range(5 * 1000, 5 * 1000);
	tianma_dcs_write_seq_static(ctx, 0xFF, 0x10);
	tianma_dcs_write_seq_static(ctx, 0xFB, 0x01);
	tianma_dcs_write_seq_static(ctx, 0x28);
	usleep_range(60 * 1000, 60 * 1000);
	tianma_dcs_write_seq_static(ctx, 0x10);
	usleep_range(60 * 1000, 60 * 1000);

	DISP_INFO("[TP] tp_gesture_enable_flag = %d, g_shutdown_flag = %d, esd_flag = %d \n", tp_gesture_enable_flag(), g_shutdown_flag, esd_flag);
	if (1 == g_shutdown_flag) {
		ctx->reset_gpio =
                        devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
                if (IS_ERR(ctx->reset_gpio)) {
                        DDPMSG("[error]%s: cannot get reset_gpio %ld\n",
                                __func__, PTR_ERR(ctx->reset_gpio));
                        return PTR_ERR(ctx->reset_gpio);
                }
                gpiod_set_value(ctx->reset_gpio, 0);
                devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	}

	if (0 == tp_gesture_enable_flag() || (esd_flag == 1) || (1 == g_shutdown_flag)) {
		ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
			"bias", 2, GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->bias_pos)) {
			DDPMSG("[error]%s: cannot get bias_pos %ld\n",
				__func__, PTR_ERR(ctx->bias_pos));
			return PTR_ERR(ctx->bias_pos);
		}
		gpiod_set_value(ctx->bias_pos, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_pos);

		udelay(1000);

		ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
			"bias", 1, GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->bias_neg)) {
			DDPMSG("[error]%s: cannot get bias_neg %ld\n",
				__func__, PTR_ERR(ctx->bias_neg));
			return PTR_ERR(ctx->bias_neg);
		}
		gpiod_set_value(ctx->bias_neg, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_neg);
	}
	oplus_lcm_display_on = 0;
	ctx->error = 0;
	ctx->prepared = false;
	return 0;
}

static int tianma_prepare(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tianma(panel);
	int ret;
	if (ctx->prepared)
		return 0;

	ctx->bias_ldo = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_LOW);
	if (IS_ERR(ctx->bias_ldo)) {
		DDPMSG("[error]%s: cannot get bias_ldo %ld\n",
			__func__, PTR_ERR(ctx->bias_ldo));
		return PTR_ERR(ctx->bias_ldo);
	}
	gpiod_set_value(ctx->bias_ldo, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_ldo);

	udelay(2000);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		DDPMSG("[error]%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	udelay(1000);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 2, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		DDPMSG("[error]%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	if (tp_boot_mode_normal()) {
		tp_gpio_current_leakage_handler(true);
	}

	tianma_panel_init(ctx);
	oplus_lcm_display_on = 1;
	ret = ctx->error;
	if (ret < 0)
		tianma_unprepare(panel);

	ctx->prepared = true;
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_rst(panel);
#endif

#ifdef PANEL_SUPPORT_READBACK
	tianma_panel_get_data(ctx);
#endif
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
	.clock = ((FRAME_WIDTH + HFP + HSA + HBP) * (FRAME_HEIGHT + MODE_0_VFP + VSA + VBP) * MODE_0_FPS) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_0_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_0_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_0_VFP + VSA + VBP,
	.vrefresh = MODE_0_FPS,
};

static const struct drm_display_mode performance_mode_1 = {
	.clock = ((FRAME_WIDTH + HFP + HSA + HBP) * (FRAME_HEIGHT + MODE_1_VFP + VSA + VBP) * MODE_1_FPS) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_1_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_1_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_1_VFP + VSA + VBP,
	.vrefresh = MODE_1_FPS,
};

static const struct drm_display_mode performance_mode_2 = {
	.clock = ((FRAME_WIDTH + HFP + HSA + HBP) * (FRAME_HEIGHT + MODE_2_VFP + VSA + VBP) * MODE_2_FPS) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_2_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_2_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_2_VFP + VSA + VBP,
	.vrefresh = MODE_2_FPS,
};

static const struct drm_display_mode performance_mode_3 = {
	.clock = ((FRAME_WIDTH + HFP + HSA + HBP) * (FRAME_HEIGHT + MODE_3_VFP + VSA + VBP) * MODE_3_FPS) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_3_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_3_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_3_VFP + VSA + VBP,
	.vrefresh = MODE_3_FPS,
};

static const struct drm_display_mode performance_mode_4 = {
	.clock = ((FRAME_WIDTH + HFP + HSA + HBP) * (FRAME_HEIGHT + MODE_4_VFP + VSA + VBP) * MODE_4_FPS) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_4_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_4_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_4_VFP + VSA + VBP,
	.vrefresh = MODE_4_FPS,
};

#if defined(CONFIG_MTK_PANEL_EXT)

static struct mtk_panel_params ext_params = {//60hz
	.vfp_low_power = 4120,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB,
		.count = 2,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
	},
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	.dsc_params = {
		.bdg_dsc_enable = 1,
		.ver                   =  DSC_VER,
		.slice_mode            =  DSC_SLICE_MODE,
		.rgb_swap              =  DSC_RGB_SWAP,
		.dsc_cfg               =  DSC_DSC_CFG,
		.rct_on                =  DSC_RCT_ON,
		.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
		.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
		.bp_enable             =  DSC_BP_ENABLE,
		.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
		.pic_height            =  FRAME_HEIGHT,
		.pic_width             =  FRAME_WIDTH,
		.slice_height          =  DSC_SLICE_HEIGHT,
		.slice_width           =  DSC_SLICE_WIDTH,
		.chunk_size            =  DSC_CHUNK_SIZE,
		.xmit_delay            =  DSC_XMIT_DELAY,
		.dec_delay             =  DSC_DEC_DELAY,
		.scale_value           =  DSC_SCALE_VALUE,
		.increment_interval    =  DSC_INCREMENT_INTERVAL,
		.decrement_interval    =  DSC_DECREMENT_INTERVAL,
		.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
		.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
		.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
		.initial_offset        =  DSC_INITIAL_OFFSET,
		.final_offset          =  DSC_FINAL_OFFSET,
		.flatness_minqp        =  DSC_FLATNESS_MINQP,
		.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
		.rc_model_size         =  DSC_RC_MODEL_SIZE,
		.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
		.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
		.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
		.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
		.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,
	},
	.hstx_cklp_en = 1,
	.data_rate = DATA_RATE,
	.pll_clk = MIPI_CLK,
	.phy_timcon = {
		.hs_trail = 9,
		.clk_trail = 10,
	},
	.bdg_ssc_disable = 1,
	.ssc_disable = 1,
	.dyn = {
		.switch_en = 1,
		.pll_clk = HOPPING_MIPI_CLK,
		.data_rate = HOPPING_DATA_RATE,
		.hbp = HOPPING_HBP,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static struct mtk_panel_params ext_params_mode_1 = {//90hz
	.vfp_low_power = 2480,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB,
		.count = 2,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
	},
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	.dsc_params = {
		.bdg_dsc_enable = 1,
		.ver                   =  DSC_VER,
		.slice_mode            =  DSC_SLICE_MODE,
		.rgb_swap              =  DSC_RGB_SWAP,
		.dsc_cfg               =  DSC_DSC_CFG,
		.rct_on                =  DSC_RCT_ON,
		.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
		.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
		.bp_enable             =  DSC_BP_ENABLE,
		.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
		.pic_height            =  FRAME_HEIGHT,
		.pic_width             =  FRAME_WIDTH,
		.slice_height          =  DSC_SLICE_HEIGHT,
		.slice_width           =  DSC_SLICE_WIDTH,
		.chunk_size            =  DSC_CHUNK_SIZE,
		.xmit_delay            =  DSC_XMIT_DELAY,
		.dec_delay             =  DSC_DEC_DELAY,
		.scale_value           =  DSC_SCALE_VALUE,
		.increment_interval    =  DSC_INCREMENT_INTERVAL,
		.decrement_interval    =  DSC_DECREMENT_INTERVAL,
		.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
		.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
		.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
		.initial_offset        =  DSC_INITIAL_OFFSET,
		.final_offset          =  DSC_FINAL_OFFSET,
		.flatness_minqp        =  DSC_FLATNESS_MINQP,
		.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
		.rc_model_size         =  DSC_RC_MODEL_SIZE,
		.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
		.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
		.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
		.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
		.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,
	},
	.hstx_cklp_en = 0,
	.data_rate = DATA_RATE,
	.pll_clk = MIPI_CLK,
        .phy_timcon = {
		.hs_trail = 9,
		.clk_trail = 10,
        },
	.bdg_ssc_disable = 1,
	.ssc_disable = 1,
	.dyn = {
		.switch_en = 1,
		.pll_clk = HOPPING_MIPI_CLK,
		.data_rate = HOPPING_DATA_RATE,
		.hbp = HOPPING_HBP,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static struct mtk_panel_params ext_params_mode_2 = {//50hz
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB,
		.count = 2,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
	},
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	.dsc_params = {
		.bdg_dsc_enable = 1,
		.ver                   =  DSC_VER,
		.slice_mode            =  DSC_SLICE_MODE,
		.rgb_swap              =  DSC_RGB_SWAP,
		.dsc_cfg               =  DSC_DSC_CFG,
		.rct_on                =  DSC_RCT_ON,
		.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
		.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
		.bp_enable             =  DSC_BP_ENABLE,
		.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
		.pic_height            =  FRAME_HEIGHT,
		.pic_width             =  FRAME_WIDTH,
		.slice_height          =  DSC_SLICE_HEIGHT,
		.slice_width           =  DSC_SLICE_WIDTH,
		.chunk_size            =  DSC_CHUNK_SIZE,
		.xmit_delay            =  DSC_XMIT_DELAY,
		.dec_delay             =  DSC_DEC_DELAY,
		.scale_value           =  DSC_SCALE_VALUE,
		.increment_interval    =  DSC_INCREMENT_INTERVAL,
		.decrement_interval    =  DSC_DECREMENT_INTERVAL,
		.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
		.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
		.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
		.initial_offset        =  DSC_INITIAL_OFFSET,
		.final_offset          =  DSC_FINAL_OFFSET,
		.flatness_minqp        =  DSC_FLATNESS_MINQP,
		.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
		.rc_model_size         =  DSC_RC_MODEL_SIZE,
		.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
		.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
		.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
		.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
		.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,
	},
	.hstx_cklp_en = 0,
	.data_rate = DATA_RATE,
	.pll_clk = MIPI_CLK,
        .phy_timcon = {
		.hs_trail = 9,
		.clk_trail = 10,
        },
	.bdg_ssc_disable = 1,
	.ssc_disable = 1,
	.dyn = {
		.switch_en = 1,
		.pll_clk = HOPPING_MIPI_CLK,
		.data_rate = HOPPING_DATA_RATE,
		.hbp = HOPPING_HBP,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static struct mtk_panel_params ext_params_mode_3 = {//48hz
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB,
		.count = 2,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
	},
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	.dsc_params = {
		.bdg_dsc_enable = 1,
		.ver                   =  DSC_VER,
		.slice_mode            =  DSC_SLICE_MODE,
		.rgb_swap              =  DSC_RGB_SWAP,
		.dsc_cfg               =  DSC_DSC_CFG,
		.rct_on                =  DSC_RCT_ON,
		.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
		.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
		.bp_enable             =  DSC_BP_ENABLE,
		.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
		.pic_height            =  FRAME_HEIGHT,
		.pic_width             =  FRAME_WIDTH,
		.slice_height          =  DSC_SLICE_HEIGHT,
		.slice_width           =  DSC_SLICE_WIDTH,
		.chunk_size            =  DSC_CHUNK_SIZE,
		.xmit_delay            =  DSC_XMIT_DELAY,
		.dec_delay             =  DSC_DEC_DELAY,
		.scale_value           =  DSC_SCALE_VALUE,
		.increment_interval    =  DSC_INCREMENT_INTERVAL,
		.decrement_interval    =  DSC_DECREMENT_INTERVAL,
		.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
		.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
		.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
		.initial_offset        =  DSC_INITIAL_OFFSET,
		.final_offset          =  DSC_FINAL_OFFSET,
		.flatness_minqp        =  DSC_FLATNESS_MINQP,
		.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
		.rc_model_size         =  DSC_RC_MODEL_SIZE,
		.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
		.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
		.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
		.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
		.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,
	},
	.hstx_cklp_en = 0,
	.data_rate = DATA_RATE,
	.pll_clk = MIPI_CLK,
        .phy_timcon = {
		.hs_trail = 9,
		.clk_trail = 10,
        },
	.bdg_ssc_disable = 1,
	.ssc_disable = 1,
	.dyn = {
		.switch_en = 1,
		.pll_clk = HOPPING_MIPI_CLK,
		.data_rate = HOPPING_DATA_RATE,
		.hbp = HOPPING_HBP,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static struct mtk_panel_params ext_params_mode_4 = {//30hz
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB,
		.count = 2,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
	},
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	.dsc_params = {
		.bdg_dsc_enable = 1,
		.ver                   =  DSC_VER,
		.slice_mode            =  DSC_SLICE_MODE,
		.rgb_swap              =  DSC_RGB_SWAP,
		.dsc_cfg               =  DSC_DSC_CFG,
		.rct_on                =  DSC_RCT_ON,
		.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
		.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
		.bp_enable             =  DSC_BP_ENABLE,
		.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
		.pic_height            =  FRAME_HEIGHT,
		.pic_width             =  FRAME_WIDTH,
		.slice_height          =  DSC_SLICE_HEIGHT,
		.slice_width           =  DSC_SLICE_WIDTH,
		.chunk_size            =  DSC_CHUNK_SIZE,
		.xmit_delay            =  DSC_XMIT_DELAY,
		.dec_delay             =  DSC_DEC_DELAY,
		.scale_value           =  DSC_SCALE_VALUE,
		.increment_interval    =  DSC_INCREMENT_INTERVAL,
		.decrement_interval    =  DSC_DECREMENT_INTERVAL,
		.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
		.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
		.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
		.initial_offset        =  DSC_INITIAL_OFFSET,
		.final_offset          =  DSC_FINAL_OFFSET,
		.flatness_minqp        =  DSC_FLATNESS_MINQP,
		.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
		.rc_model_size         =  DSC_RC_MODEL_SIZE,
		.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
		.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
		.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
		.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
		.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,
	},
	.hstx_cklp_en = 0,
	.data_rate = DATA_RATE,
	.pll_clk = MIPI_CLK,
        .phy_timcon = {
		.hs_trail = 9,
		.clk_trail = 10,
        },
	.bdg_ssc_disable = 1,
	.ssc_disable = 1,
	.dyn = {
		.switch_en = 1,
		.pll_clk = HOPPING_MIPI_CLK,
		.data_rate = HOPPING_DATA_RATE,
		.hbp = HOPPING_HBP,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static void tianma_gamma_enter(void *dsi, dcs_write_gce cb, void *handle)
{
	char bl_tb1[] = {0xFF, 0x20};
	char bl_tb2[] = {0xFB, 0x01};
	char bl_tb3[] = {0x95, 0x09};
	char bl_tb4[] = {0x96, 0x09};
	char bl_tb5[] = {0xFF, 0x20};
	char bl_tb6[] = {0xFB, 0x01};
	char bl_tb7[] = {0xB0, 0x00, 0x00, 0x00, 0x04, 0x00, 0x0D, 0x00, 0x15, 0x00, 0x1C, 0x00, 0x24, 0x00, 0x2B, 0x00, 0x33};
	char bl_tb8[] = {0xB1, 0x00, 0x39, 0x00, 0x54, 0x00, 0x6B, 0x00, 0x98, 0x00, 0xBF, 0x01, 0x09, 0x01, 0x4E, 0x01, 0x51};
	char bl_tb9[] = {0xB2, 0x01, 0x9B, 0x01, 0xFA, 0x02, 0x36, 0x02, 0x84, 0x02, 0xB9, 0x02, 0xF8, 0x03, 0x0D, 0x03, 0x22};
	char bl_tb10[] = {0xB3, 0x03, 0x39, 0x03, 0x52, 0x03, 0x6F, 0x03, 0x92, 0x03, 0xB6, 0x03, 0xBC, 0x00, 0x00};
	char bl_tb11[] = {0xB4, 0x00, 0x00, 0x00, 0x04, 0x00, 0x0D, 0x00, 0x15, 0x00, 0x1C, 0x00, 0x24, 0x00, 0x2B, 0x00, 0x33};
	char bl_tb12[] = {0xB5, 0x00, 0x39, 0x00, 0x54, 0x00, 0x6B, 0x00, 0x98, 0x00, 0xBF, 0x01, 0x09, 0x01, 0x4E, 0x01, 0x51};
	char bl_tb13[] = {0xB6, 0x01, 0x9B, 0x01, 0xFA, 0x02, 0x36, 0x02, 0x84, 0x02, 0xB9, 0x02, 0xF8, 0x03, 0x0D, 0x03, 0x22};
	char bl_tb14[] = {0xB7, 0x03, 0x39, 0x03, 0x52, 0x03, 0x6F, 0x03, 0x92, 0x03, 0xB6, 0x03, 0xBC, 0x00, 0x00};
	char bl_tb15[] = {0xB8, 0x00, 0x00, 0x00, 0x04, 0x00, 0x0D, 0x00, 0x15, 0x00, 0x1C, 0x00, 0x24, 0x00, 0x2B, 0x00, 0x33};
	char bl_tb16[] = {0xB9, 0x00, 0x39, 0x00, 0x54, 0x00, 0x6B, 0x00, 0x98, 0x00, 0xBF, 0x01, 0x09, 0x01, 0x4E, 0x01, 0x51};
	char bl_tb17[] = {0xBA, 0x01, 0x9B, 0x01, 0xFA, 0x02, 0x36, 0x02, 0x84, 0x02, 0xB9, 0x02, 0xF8, 0x03, 0x0D, 0x03, 0x22};
	char bl_tb18[] = {0xBB, 0x03, 0x39, 0x03, 0x52, 0x03, 0x6F, 0x03, 0x92, 0x03, 0xB6, 0x03, 0xBC, 0x00, 0x00};
	char bl_tb19[] = {0xFF, 0x21};
	char bl_tb20[] = {0xFB, 0x01};
	char bl_tb21[] = {0xB0, 0x00, 0x00, 0x00, 0x04, 0x00, 0x0D, 0x00, 0x15, 0x00, 0x1C, 0x00, 0x24, 0x00, 0x2B, 0x00, 0x33};
	char bl_tb22[] = {0xB1, 0x00, 0x39, 0x00, 0x54, 0x00, 0x6B, 0x00, 0x98, 0x00, 0xBF, 0x01, 0x09, 0x01, 0x4E, 0x01, 0x51};
	char bl_tb23[] = {0xB2, 0x01, 0x9B, 0x01, 0xFA, 0x02, 0x36, 0x02, 0x84, 0x02, 0xB9, 0x02, 0xF8, 0x03, 0x0D, 0x03, 0x22};
	char bl_tb24[] = {0xB3, 0x03, 0x39, 0x03, 0x52, 0x03, 0x6F, 0x03, 0x92, 0x03, 0xB6, 0x03, 0xBC, 0x00, 0x00};
	char bl_tb25[] = {0xB4, 0x00, 0x00, 0x00, 0x04, 0x00, 0x0D, 0x00, 0x15, 0x00, 0x1C, 0x00, 0x24, 0x00, 0x2B, 0x00, 0x33};
	char bl_tb26[] = {0xB5, 0x00, 0x39, 0x00, 0x54, 0x00, 0x6B, 0x00, 0x98, 0x00, 0xBF, 0x01, 0x09, 0x01, 0x4E, 0x01, 0x51};
	char bl_tb27[] = {0xB6, 0x01, 0x9B, 0x01, 0xFA, 0x02, 0x36, 0x02, 0x84, 0x02, 0xB9, 0x02, 0xF8, 0x03, 0x0D, 0x03, 0x22};
	char bl_tb28[] = {0xB7, 0x03, 0x39, 0x03, 0x52, 0x03, 0x6F, 0x03, 0x92, 0x03, 0xB6, 0x03, 0xBC, 0x00, 0x00};
	char bl_tb29[] = {0xB8, 0x00, 0x00, 0x00, 0x04, 0x00, 0x0D, 0x00, 0x15, 0x00, 0x1C, 0x00, 0x24, 0x00, 0x2B, 0x00, 0x33};
	char bl_tb30[] = {0xB9, 0x00, 0x39, 0x00, 0x54, 0x00, 0x6B, 0x00, 0x98, 0x00, 0xBF, 0x01, 0x09, 0x01, 0x4E, 0x01, 0x51};
	char bl_tb31[] = {0xBA, 0x01, 0x9B, 0x01, 0xFA, 0x02, 0x36, 0x02, 0x84, 0x02, 0xB9, 0x02, 0xF8, 0x03, 0x0D, 0x03, 0x22};
	char bl_tb32[] = {0xBB, 0x03, 0x39, 0x03, 0x52, 0x03, 0x6F, 0x03, 0x92, 0x03, 0xB6, 0x03, 0xBC, 0x00, 0x00};
	char bl_tb33[] = {0xFF, 0x10};
	char bl_tb34[] = {0xFB, 0x01};

	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
	cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
	cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
	cb(dsi, handle, bl_tb7, ARRAY_SIZE(bl_tb7));
	cb(dsi, handle, bl_tb8, ARRAY_SIZE(bl_tb8));
	cb(dsi, handle, bl_tb9, ARRAY_SIZE(bl_tb9));
	cb(dsi, handle, bl_tb10, ARRAY_SIZE(bl_tb10));
	cb(dsi, handle, bl_tb11, ARRAY_SIZE(bl_tb11));
	cb(dsi, handle, bl_tb12, ARRAY_SIZE(bl_tb12));
	cb(dsi, handle, bl_tb13, ARRAY_SIZE(bl_tb13));
	cb(dsi, handle, bl_tb14, ARRAY_SIZE(bl_tb14));
	cb(dsi, handle, bl_tb15, ARRAY_SIZE(bl_tb15));
	cb(dsi, handle, bl_tb16, ARRAY_SIZE(bl_tb16));
	cb(dsi, handle, bl_tb17, ARRAY_SIZE(bl_tb17));
	cb(dsi, handle, bl_tb18, ARRAY_SIZE(bl_tb18));
	cb(dsi, handle, bl_tb19, ARRAY_SIZE(bl_tb19));
	cb(dsi, handle, bl_tb20, ARRAY_SIZE(bl_tb20));
	cb(dsi, handle, bl_tb21, ARRAY_SIZE(bl_tb21));
	cb(dsi, handle, bl_tb22, ARRAY_SIZE(bl_tb22));
	cb(dsi, handle, bl_tb23, ARRAY_SIZE(bl_tb23));
	cb(dsi, handle, bl_tb24, ARRAY_SIZE(bl_tb24));
	cb(dsi, handle, bl_tb25, ARRAY_SIZE(bl_tb25));
	cb(dsi, handle, bl_tb26, ARRAY_SIZE(bl_tb26));
	cb(dsi, handle, bl_tb27, ARRAY_SIZE(bl_tb27));
	cb(dsi, handle, bl_tb28, ARRAY_SIZE(bl_tb28));
	cb(dsi, handle, bl_tb29, ARRAY_SIZE(bl_tb29));
	cb(dsi, handle, bl_tb30, ARRAY_SIZE(bl_tb30));
	cb(dsi, handle, bl_tb31, ARRAY_SIZE(bl_tb31));
	cb(dsi, handle, bl_tb32, ARRAY_SIZE(bl_tb32));
	cb(dsi, handle, bl_tb33, ARRAY_SIZE(bl_tb33));
	cb(dsi, handle, bl_tb34, ARRAY_SIZE(bl_tb34));
}
static void tianma_gamma_exit(void *dsi, dcs_write_gce cb, void *handle)
{
	char bl_tb1[] = {0xFF, 0x20};
	char bl_tb2[] = {0xFB, 0x01};
	char bl_tb3[] = {0x95, 0xD1};
	char bl_tb4[] = {0x96, 0xD1};
	char bl_tb5[] = {0xFF, 0x20};
	char bl_tb6[] = {0xFB, 0x01};
	char bl_tb7[] = {0xB0, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x47, 0x00, 0x6A, 0x00, 0x86, 0x00, 0x9E, 0x00, 0xB4, 0x00, 0xC7};
	char bl_tb8[] = {0xB1, 0x00, 0xD8, 0x01, 0x12, 0x01, 0x3A, 0x01, 0x7E, 0x01, 0xAC, 0x01, 0xF8, 0x02, 0x30, 0x02, 0x32};
	char bl_tb9[] = {0xB2, 0x02, 0x68, 0x02, 0xA5, 0x02, 0xCD, 0x02, 0xFE, 0x03, 0x21, 0x03, 0x48, 0x03, 0x57, 0x03, 0x65};
	char bl_tb10[] = {0xB3, 0x03, 0x76, 0x03, 0x89, 0x03, 0x9E, 0x03, 0xAF, 0x03, 0xD1, 0x03, 0xD8, 0x00, 0x00};
	char bl_tb11[] = {0xB4, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x47, 0x00, 0x6A, 0x00, 0x86, 0x00, 0x9E, 0x00, 0xB4, 0x00, 0xC7};
	char bl_tb12[] = {0xB5, 0x00, 0xD8, 0x01, 0x12, 0x01, 0x3A, 0x01, 0x7E, 0x01, 0xAC, 0x01, 0xF8, 0x02, 0x30, 0x02, 0x32};
	char bl_tb13[] = {0xB6, 0x02, 0x68, 0x02, 0xA5, 0x02, 0xCD, 0x02, 0xFE, 0x03, 0x21, 0x03, 0x48, 0x03, 0x57, 0x03, 0x65};
	char bl_tb14[] = {0xB7, 0x03, 0x76, 0x03, 0x89, 0x03, 0x9E, 0x03, 0xAF, 0x03, 0xD1, 0x03, 0xD8, 0x00, 0x00};
	char bl_tb15[] = {0xB8, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x47, 0x00, 0x6A, 0x00, 0x86, 0x00, 0x9E, 0x00, 0xB4, 0x00, 0xC7};
	char bl_tb16[] = {0xB9, 0x00, 0xD8, 0x01, 0x12, 0x01, 0x3A, 0x01, 0x7E, 0x01, 0xAC, 0x01, 0xF8, 0x02, 0x30, 0x02, 0x32};
	char bl_tb17[] = {0xBA, 0x02, 0x68, 0x02, 0xA5, 0x02, 0xCD, 0x02, 0xFE, 0x03, 0x21, 0x03, 0x48, 0x03, 0x57, 0x03, 0x65};
	char bl_tb18[] = {0xBB, 0x03, 0x76, 0x03, 0x89, 0x03, 0x9E, 0x03, 0xAF, 0x03, 0xD1, 0x03, 0xD8, 0x00, 0x00};
	char bl_tb19[] = {0xFF, 0x21};
	char bl_tb20[] = {0xFB, 0x01};
	char bl_tb21[] = {0xB0, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x47, 0x00, 0x6A, 0x00, 0x86, 0x00, 0x9E, 0x00, 0xB4, 0x00, 0xC7};
	char bl_tb22[] = {0xB1, 0x00, 0xD8, 0x01, 0x12, 0x01, 0x3A, 0x01, 0x7E, 0x01, 0xAC, 0x01, 0xF8, 0x02, 0x30, 0x02, 0x32};
	char bl_tb23[] = {0xB2, 0x02, 0x68, 0x02, 0xA5, 0x02, 0xCD, 0x02, 0xFE, 0x03, 0x21, 0x03, 0x48, 0x03, 0x57, 0x03, 0x65};
	char bl_tb24[] = {0xB3, 0x03, 0x76, 0x03, 0x89, 0x03, 0x9E, 0x03, 0xAF, 0x03, 0xD1, 0x03, 0xD8, 0x00, 0x00};
	char bl_tb25[] = {0xB4, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x47, 0x00, 0x6A, 0x00, 0x86, 0x00, 0x9E, 0x00, 0xB4, 0x00, 0xC7};
	char bl_tb26[] = {0xB5, 0x00, 0xD8, 0x01, 0x12, 0x01, 0x3A, 0x01, 0x7E, 0x01, 0xAC, 0x01, 0xF8, 0x02, 0x30, 0x02, 0x32};
	char bl_tb27[] = {0xB6, 0x02, 0x68, 0x02, 0xA5, 0x02, 0xCD, 0x02, 0xFE, 0x03, 0x21, 0x03, 0x48, 0x03, 0x57, 0x03, 0x65};
	char bl_tb28[] = {0xB7, 0x03, 0x76, 0x03, 0x89, 0x03, 0x9E, 0x03, 0xAF, 0x03, 0xD1, 0x03, 0xD8, 0x00, 0x00};
	char bl_tb29[] = {0xB8, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x47, 0x00, 0x6A, 0x00, 0x86, 0x00, 0x9E, 0x00, 0xB4, 0x00, 0xC7};
	char bl_tb30[] = {0xB9, 0x00, 0xD8, 0x01, 0x12, 0x01, 0x3A, 0x01, 0x7E, 0x01, 0xAC, 0x01, 0xF8, 0x02, 0x30, 0x02, 0x32};
	char bl_tb31[] = {0xBA, 0x02, 0x68, 0x02, 0xA5, 0x02, 0xCD, 0x02, 0xFE, 0x03, 0x21, 0x03, 0x48, 0x03, 0x57, 0x03, 0x65};
	char bl_tb32[] = {0xBB, 0x03, 0x76, 0x03, 0x89, 0x03, 0x9E, 0x03, 0xAF, 0x03, 0xD1, 0x03, 0xD8, 0x00, 0x00};
	char bl_tb33[] = {0xFF, 0x10};
	char bl_tb34[] = {0xFB, 0x01};

	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
	cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
	cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
	cb(dsi, handle, bl_tb7, ARRAY_SIZE(bl_tb7));
	cb(dsi, handle, bl_tb8, ARRAY_SIZE(bl_tb8));
	cb(dsi, handle, bl_tb9, ARRAY_SIZE(bl_tb9));
	cb(dsi, handle, bl_tb10, ARRAY_SIZE(bl_tb10));
	cb(dsi, handle, bl_tb11, ARRAY_SIZE(bl_tb11));
	cb(dsi, handle, bl_tb12, ARRAY_SIZE(bl_tb12));
	cb(dsi, handle, bl_tb13, ARRAY_SIZE(bl_tb13));
	cb(dsi, handle, bl_tb14, ARRAY_SIZE(bl_tb14));
	cb(dsi, handle, bl_tb15, ARRAY_SIZE(bl_tb15));
	cb(dsi, handle, bl_tb16, ARRAY_SIZE(bl_tb16));
	cb(dsi, handle, bl_tb17, ARRAY_SIZE(bl_tb17));
	cb(dsi, handle, bl_tb18, ARRAY_SIZE(bl_tb18));
	cb(dsi, handle, bl_tb19, ARRAY_SIZE(bl_tb19));
	cb(dsi, handle, bl_tb20, ARRAY_SIZE(bl_tb20));
	cb(dsi, handle, bl_tb21, ARRAY_SIZE(bl_tb21));
	cb(dsi, handle, bl_tb22, ARRAY_SIZE(bl_tb22));
	cb(dsi, handle, bl_tb23, ARRAY_SIZE(bl_tb23));
	cb(dsi, handle, bl_tb24, ARRAY_SIZE(bl_tb24));
	cb(dsi, handle, bl_tb25, ARRAY_SIZE(bl_tb25));
	cb(dsi, handle, bl_tb26, ARRAY_SIZE(bl_tb26));
	cb(dsi, handle, bl_tb27, ARRAY_SIZE(bl_tb27));
	cb(dsi, handle, bl_tb28, ARRAY_SIZE(bl_tb28));
	cb(dsi, handle, bl_tb29, ARRAY_SIZE(bl_tb29));
	cb(dsi, handle, bl_tb30, ARRAY_SIZE(bl_tb30));
	cb(dsi, handle, bl_tb31, ARRAY_SIZE(bl_tb31));
	cb(dsi, handle, bl_tb32, ARRAY_SIZE(bl_tb32));
	cb(dsi, handle, bl_tb33, ARRAY_SIZE(bl_tb33));
	cb(dsi, handle, bl_tb34, ARRAY_SIZE(bl_tb34));
}

static int tianma_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0x0f, 0xff};
	char bl_tb1[] = {0x53, 0x24};

	if (level > 4095)
		level = 4095;
	DISP_INFO("set backlight level = %d\n", level);
	bl_tb0[1] = (level & 0xff00) >> 8;
	bl_tb0[2] = level & 0xff;
	if (!cb)
		return -1;

	if (level > 0 && level < 14 && bl_gamma == 0) {
		tianma_gamma_enter(dsi, cb, handle);
		bl_gamma = 1;
	} else if (level > 13 && bl_gamma == 1) {
		tianma_gamma_exit(dsi, cb, handle);
		bl_gamma = 0;
	} else if (level == 0) {
		bl_gamma = 0;
	}

	if (dimming_is_on) {
		if ((frame_count > 6) || (level == 0)) {
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			dimming_is_on = false;
		} else {
			frame_count++;
		}
	}

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	oplus_display_brightness = level;
	esd_brightness = level;

	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0x51, 0x03, 0xff};
	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
	if (!cb)
		return -1;
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 1;
}

static int oplus_esd_recovery_pullup_gpio(struct drm_panel *panel, int status)
{
	struct tianma *ctx = panel_to_tianma(panel);
	if (status == 1) {
		ctx->bias_ldo = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_LOW);
		if (IS_ERR(ctx->bias_ldo)) {
			DDPMSG("[error]%s: cannot get bias_ldo %ld\n",
				__func__, PTR_ERR(ctx->bias_ldo));
			return PTR_ERR(ctx->bias_ldo);
		}
		gpiod_set_value(ctx->bias_ldo, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_ldo);
		usleep_range(10 * 1000, 11 * 1000);
	} else if (status == 0) {
		ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->reset_gpio)) {
			DDPMSG("[error]%s: cannot get reset_gpio %ld\n",
				__func__, PTR_ERR(ctx->reset_gpio));
			return PTR_ERR(ctx->reset_gpio);
		}
		gpiod_set_value(ctx->reset_gpio, 0);
		devm_gpiod_put(ctx->dev, ctx->reset_gpio);

		tp_gpio_current_leakage_handler(false);

		usleep_range(1000, 1000);

		ctx->bias_ldo = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->bias_ldo)) {
			DDPMSG("[error]%s: cannot get bias_ldo %ld\n",
				__func__, PTR_ERR(ctx->bias_ldo));
			return PTR_ERR(ctx->bias_ldo);
		}
		gpiod_set_value(ctx->bias_ldo, 1);
		devm_gpiod_put(ctx->dev, ctx->bias_ldo);
		usleep_range(1000 * 1000, 1000 * 1000);
	}
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

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(panel, mode);
	if (m->vrefresh == MODE_0_FPS) {
		ext->params = &ext_params;
	} else if (m->vrefresh == MODE_1_FPS) {
		ext->params = &ext_params_mode_1;
	} else if (m->vrefresh == MODE_2_FPS) {
		ext->params = &ext_params_mode_2;
	} else if (m->vrefresh == MODE_3_FPS) {
		ext->params = &ext_params_mode_3;
	} else if (m->vrefresh == MODE_4_FPS) {
		ext->params = &ext_params_mode_4;
	} else {
		ret = 1;
	}
	if (!ret) {
		current_fps = m->vrefresh;
	}

	return ret;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct tianma *ctx = panel_to_tianma(panel);
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		DDPMSG("[error]%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
#ifdef BDG_PORTING_DBG
	struct tianma *ctx = panel_to_tianma(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	unsigned char data[3] = {0x00, 0x00, 0x00};
	unsigned char id[3] = {0x6e, 0x48, 0x00};
	ssize_t ret;
	ret = mipi_dsi_dcs_read(dsi, 0x4, data, 3);
	if (ret < 0) {
		DDPMSG("[error]%s error\n", __func__);
		return 0;
	}

	DDPINFO("ATA read data %x %x %x\n", data[0], data[1], data[2]);

	if (data[0] == id[0])
		return 1;

	DDPINFO("ATA expect read data is %x %x %x\n",
			id[0], id[1], id[2]);
#endif
	return 1;
}

static void tianma_cabc_mode_switch(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int cabc_mode)
{
	unsigned char cabc_cmd_0[] = {0x53, 0x2c};
	unsigned char cabc_cmd_1[] = {0xFF, 0x10};
	unsigned char cabc_cmd_2[] = {0xFB, 0x01};
	unsigned char cabc_cmd_3[] = {0x55, 0x00};
	cabc_status = cabc_mode;

	switch(cabc_mode) {
		case 0://CLOSE
			cabc_cmd_3[1] = 0x00;
			break;
		case 1://UI
			cabc_cmd_3[1] = 0x01;
			break;
		case 2://STILL
			cabc_cmd_3[1] = 0x02;
			break;
		case 3://MOV--->STILL
			cabc_cmd_3[1] = 0x02;
			break;
		default:
			cabc_cmd_3[1] = 0x00;
			DDPMSG("[lcd_info]%s: cabc_mode=%d is not support, close cabc !\n", __func__, cabc_mode);
			break;
	}

	cb(dsi, handle, cabc_cmd_1, ARRAY_SIZE(cabc_cmd_1));
	cb(dsi, handle, cabc_cmd_2, ARRAY_SIZE(cabc_cmd_2));
	if (!dimming_is_on) {
		cb(dsi, handle, cabc_cmd_0, ARRAY_SIZE(cabc_cmd_0));
		dimming_is_on = true;
		frame_count = 0;
	}
	cb(dsi, handle, cabc_cmd_3, ARRAY_SIZE(cabc_cmd_3));
	DISP_INFO("cabc_mode switch to %d, set cabc_para=%#x\n", cabc_mode, cabc_cmd_3[1]);
}

static struct mtk_panel_funcs ext_funcs = {
	.set_backlight_cmdq = tianma_setbacklight_cmdq,
	.reset = panel_ext_reset,
	.ext_param_set = mtk_panel_ext_param_set,
	.ata_check = panel_ata_check,
	.cabc_switch = tianma_cabc_mode_switch,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.esd_recovery_pullup_gpio = oplus_esd_recovery_pullup_gpio,
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
	struct drm_display_mode *mode_1;
	struct drm_display_mode *mode_2;
	struct drm_display_mode *mode_3;
	struct drm_display_mode *mode_4;
	pr_info("[%s]\n", __func__);
	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		DDPMSG("[error]failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

	mode_1 = drm_mode_duplicate(panel->drm, &performance_mode_1);
	if (!mode_1) {
		DDPMSG("[error]failed to add mode %ux%ux@%u\n",
			performance_mode_1.hdisplay,
			performance_mode_1.vdisplay,
			performance_mode_1.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode_1);
	mode_1->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode_1);

	mode_2 = drm_mode_duplicate(panel->drm, &performance_mode_2);
	if (!mode_2) {
		DDPMSG("[error]failed to add mode %ux%ux@%u\n",
			performance_mode_2.hdisplay,
			performance_mode_2.vdisplay,
			performance_mode_2.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode_2);
	mode_2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode_2);

	mode_3 = drm_mode_duplicate(panel->drm, &performance_mode_3);
	if (!mode_3) {
		DDPMSG("[error]failed to add mode %ux%ux@%u\n",
			performance_mode_3.hdisplay,
			performance_mode_3.vdisplay,
			performance_mode_3.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode_3);
	mode_3->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode_3);

	mode_4 = drm_mode_duplicate(panel->drm, &performance_mode_4);
	if (!mode_4) {
		DDPMSG("[error]failed to add mode %ux%ux@%u\n",
			performance_mode_4.hdisplay,
			performance_mode_4.vdisplay,
			performance_mode_4.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode_4);
	mode_4->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode_4);

	panel->connector->display_info.width_mm = 70;
	panel->connector->display_info.height_mm = 155;

	return 2;
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
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("tianma_nt36672e_vdo_120hz_vfp_6382 %s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info(" tianma_nt36672e_vdo_120hz_vfp_6382 isn't current lcm\n");
		return -ENODEV;
	}
	ctx = devm_kzalloc(dev, sizeof(struct tianma), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		DDPMSG("[error]cannot get reset-gpios %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	ctx->bias_ldo = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_LOW);
	if (IS_ERR(ctx->bias_ldo)) {
		DDPMSG("[error]%s: cannot get bias_ldo %ld\n",
			__func__, PTR_ERR(ctx->bias_ldo));
		return PTR_ERR(ctx->bias_ldo);
	}
	gpiod_set_value(ctx->bias_ldo, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_ldo);
	pr_info("%s gpio 136 set low\n", __func__);

#ifndef CONFIG_RT4831A_I2C
	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		DDPMSG("[error]%s: cannot get bias-pos 1 %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 2, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		DDPMSG("[error]%s: cannot get bias-neg 2 %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	devm_gpiod_put(dev, ctx->bias_neg);
#endif

	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &tianma_drm_funcs;

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
	register_device_proc("lcd", "AC102", "P_7");
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	pr_info("%s+\n", __func__);

	return ret;
}

static int tianma_remove(struct mipi_dsi_device *dsi)
{
	struct tianma *ctx = mipi_dsi_get_drvdata(dsi);
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id tianma_of_match[] = {
	{ .compatible = "AC102_P_7_A0007_dsc_video_mode_panel", },
	{ }
};

MODULE_DEVICE_TABLE(of, tianma_of_match);

static struct mipi_dsi_driver tianma_driver = {
	.probe = tianma_probe,
	.remove = tianma_remove,
	.driver = {
		.name = "AC102_P_7_A0007_dsc_video_mode_panel",
		.owner = THIS_MODULE,
		.of_match_table = tianma_of_match,
	},
};

module_mipi_dsi_driver(tianma_driver);

MODULE_AUTHOR("Jingjing Liu <jingjing.liu@mediatek.com>");
MODULE_DESCRIPTION("tianma nt36672E vdo 120hz 6382 Panel Driver");
MODULE_LICENSE("GPL v2");

