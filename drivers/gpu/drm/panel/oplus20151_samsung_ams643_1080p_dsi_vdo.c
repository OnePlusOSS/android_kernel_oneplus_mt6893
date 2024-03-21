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

#include <mt-plat/mtk_boot_common.h>

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/oplus20151_AMS643XY04_data_hw_roundedpattern.h"
#endif

static u32 flag_hbm = 0;
static u32 aod_mode = 0;
/* #ifdef OPLUS_BUG_STABILITY */
#define RAMLESS_AOD_PAYLOAD_SIZE	100
#define OPLUS_DC_BACKLIGHT_THRESHOLD 1100
static int esd_brightness;

static int oplus_dc_alpha = 0.5;
extern int oplus_dc_enable_real;
extern int oplus_dc_threshold;
//static int oplus_dc_enable = 0;
static unsigned int safe_mode = 2;
extern unsigned long oplus_display_brightness;
extern unsigned long oplus_max_normal_brightness;
extern char send_cmd[RAMLESS_AOD_PAYLOAD_SIZE];
//extern unsigned int aod_light_mode;
/* #endif */ /* OPLUS_BUG_STABILITY */

#define LCM_DSI_CMD_MODE 0

#define REGFLAG_CMD       0xFFFA
#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD

#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   2047
struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_vci;

	bool prepared;
	bool enabled;

	int error;

	bool hbm_en;
	bool hbm_wait;
};

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

struct ba {
	u32 brightness;
	u32 alpha;
};

static struct ba brightness_seed_alpha_lut_dc[] = {
	{0, 1020},
	{1, 1020},
	{165, 1001},
	{195, 993},
	{235, 980},
	{305, 960},
	{400, 920},
	{500, 880},
	{580, 840},
	{591, 800},
	{660, 720},
	{720, 640},
	{780, 560},
	{825, 480},
	{865, 400},
	{900, 320},
	{963, 240},
	{1000, 160},
	{1050, 80},
	{1100, 0},
};

static struct LCM_setting_table lcm_aod_to_normal[] = {
	/* sleep out */
	{REGFLAG_CMD,1,{0x11}},
	{REGFLAG_DELAY,20,{}},

	/* Display off */
	//{REGFLAG_CMD,1,{0x28}},
	/* MIPI Video cmd*/
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2,{0xF2,0x0F}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},

	/* AOD Mode off Setting */
	{REGFLAG_CMD,2,{0x53,0x20}},

	/* ELVSS Fix setting */
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0xB0,0x1E}},
	{REGFLAG_CMD,8,{0xB3,0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},

	/* DBV 2.2 Curve Setting */
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0xB0, 0x3C}},
	{REGFLAG_CMD,2,{0xB5, 0x00}},
	{REGFLAG_CMD,2,{0xB0, 0x15}},
	{REGFLAG_CMD,18,{0xB5, 0x00, 0xB4, 0x10, 0xE1, 0x74, 0x1B, 0xE1, 0xFC, 0x26, 0x22, 0xB8, 0x36, 0x43, 0xB8, 0x40, 0x64, 0x4C}},
	{REGFLAG_CMD,2,{0xB0, 0x2E}},
	{REGFLAG_CMD,12,{0xB3, 0x44, 0xC0, 0xB4, 0x10, 0x31, 0x74, 0x1F, 0xC2, 0xB8, 0x3B, 0x80}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},

	/* Dimming Setting 11bit */
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0xB0, 0x01}},
	{REGFLAG_CMD,2,{0xB3, 0x7F}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},

	/*seed setting*/
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x80, 0x92}},
	{REGFLAG_CMD,2,{0xB1, 0x00}},
	{REGFLAG_CMD,3,{0xB0, 0x2B,0xB1}},
	{REGFLAG_CMD,22,{0xB1, 0xE8, 0x00, 0x04, 0x02, 0xFF, 0x02, 0x00, 0x00, 0xFF, 0x14, 0xFF, 0xDE, 0xFD, 0x00, 0xF0, 0xFA, 0xEB, 0x00, 0xFF, 0xFF, 0xFF}},
	{REGFLAG_CMD,3,{0xB0, 0x55, 0xB1}},
	{REGFLAG_CMD,2,{0xB1, 0x80}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},

	{REGFLAG_DELAY,100,{}},

	/* Display on */
	{REGFLAG_CMD,1,{0x29}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_seed_setting[] = {
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x80, 0x92}},
	{REGFLAG_CMD,2,{0xB1, 0x00}},
	{REGFLAG_CMD,3,{0xB0, 0x2B,0xB1}},
	{REGFLAG_CMD,22,{0xB1, 0xE8, 0x00, 0x04, 0x02, 0xFF, 0x02, 0x00, 0x00, 0xFF, 0x14, 0xFF, 0xDE, 0xFD, 0x00, 0xF0, 0xFA, 0xEB, 0x00, 0xFF, 0xFF, 0xFF}},
	{REGFLAG_CMD,3,{0xB0, 0x55, 0xB1}},
	{REGFLAG_CMD,2,{0xB1, 0x80}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_high_mode[] = {
	/* aod 50nit*/
	{REGFLAG_CMD,3, {0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2, {0x53,0x22}},
	{REGFLAG_CMD,3, {0xF0,0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_low_mode[] = {
	/* aod 10nit*/
	{REGFLAG_CMD,3, {0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2, {0x53,0x23}},
	{REGFLAG_CMD,3, {0xF0,0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setbrightness_normal[] = {
	{REGFLAG_CMD,3, {0x51,0x00,0x00}},
	{REGFLAG_CMD,3, {0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2, {0x53,0x20}},
	{REGFLAG_CMD,2, {0xB7,0x02}},
	{REGFLAG_CMD,3, {0xF0,0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setbrightness_hbm[] = {
	{REGFLAG_CMD,3, {0x51,0x00,0x00}},
	{REGFLAG_CMD,3, {0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2, {0x53,0xE8}},
	{REGFLAG_CMD,2, {0xB7,0x02}},
	{REGFLAG_CMD,3, {0xF0,0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_finger_HBM_on_setting[] = {
	{REGFLAG_CMD,3, {0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,3, {0x51,0x05,0xFF}},
	{REGFLAG_CMD,2, {0x53,0xE0}},
	{REGFLAG_CMD,2, {0xB7,0x00}},
	{REGFLAG_CMD,3, {0xF0,0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_normal_HBM_on_setting[] = {
	{REGFLAG_CMD,3, {0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,3, {0x51,0x03,0xFF}},
	{REGFLAG_CMD,2, {0x53,0xE0}},
	{REGFLAG_CMD,2, {0xB7,0x02}},
	{REGFLAG_CMD,3, {0xF0,0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
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

static int interpolate(int x, int xa, int xb, int ya, int yb)
{
	int bf, factor, plus;
	int sub = 0;

	bf = 2 * (yb - ya) * (x - xa) / (xb - xa);
	factor = bf / 2;
	plus = bf % 2;
	if ((xa - xb) && (yb - ya))
		sub = 2 * (x - xa) * (x - xb) / (yb - ya) / (xa - xb);

	return ya + factor + plus + sub;
}

static int oplus_seed_bright_to_alpha(int brightness)
{
	int level = ARRAY_SIZE(brightness_seed_alpha_lut_dc);
	int i = 0;
	int alpha;

	for (i = 0; i < ARRAY_SIZE(brightness_seed_alpha_lut_dc); i++) {
		if (brightness_seed_alpha_lut_dc[i].brightness >= brightness)
			break;
	}

	if (i == 0)
		alpha = brightness_seed_alpha_lut_dc[0].alpha;
	else if (i == level)
		alpha = brightness_seed_alpha_lut_dc[level - 1].alpha;
	else
		alpha = interpolate(brightness,
			brightness_seed_alpha_lut_dc[i-1].brightness,
			brightness_seed_alpha_lut_dc[i].brightness,
			brightness_seed_alpha_lut_dc[i-1].alpha,
			brightness_seed_alpha_lut_dc[i].alpha);

	return alpha;
}

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

static void lcm_panel_init(struct lcm *ctx)
{
/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for lcm %s\n", __func__);
/* #endif */ /* OPLUS_BUG_STABILITY */

	/*sleep out*/
	lcm_dcs_write_seq_static(ctx, 0x11);
	msleep(20);
	/* TE sync on*/
	lcm_dcs_write_seq_static(ctx, 0x35, 0x00);

	/* Fail Safe Setting*/
	lcm_dcs_write_seq_static(ctx, 0xFC, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x03);
	lcm_dcs_write_seq_static(ctx, 0xED, 0x40,0xFF,0x08,0x87,0xA4,0x4A,0x73,0xE2,0x9F,0x00);
	lcm_dcs_write_seq_static(ctx, 0xFC, 0xA5, 0xA5);

	/* ELVSS Dim Setting */
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x05);
	lcm_dcs_write_seq_static(ctx, 0xB3, 0x07);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);

	/* ELVSS Fix Setting */
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x1E);
	lcm_dcs_write_seq_static(ctx, 0xB3,0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);

	/* ACL OFF */
	lcm_dcs_write_seq_static(ctx, 0x55, 0x00);

	/* FD Setting */
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xCD, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);

	/* PCD setting off*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xEA, 0x48);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);

	/* DBV 2.2 Curve Setting */
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x3C);
	lcm_dcs_write_seq_static(ctx, 0xB5, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x15);
	lcm_dcs_write_seq_static(ctx, 0xB5, 0x00, 0xB4, 0x10, 0xE1, 0x74, 0x1B, 0xE1, 0xFC, 0x26, 0x22, 0xB8, 0x36, 0x43, 0xB8, 0x40, 0x64, 0x4C);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x2E);
	lcm_dcs_write_seq_static(ctx, 0xB3, 0x44, 0xC0, 0xB4, 0x10, 0x31, 0x74, 0x1F, 0xC2, 0xB8, 0x3B, 0x80);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);

	/* Dimming Setting 11bit */
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xB3, 0x7F);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	lcm_dcs_write_seq_static(ctx, 0x53, 0x20);

	/* Seed CRC mode enable*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0x80, 0x92);
	lcm_dcs_write_seq_static(ctx, 0xB1, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x2B, 0xB1);
	lcm_dcs_write_seq_static(ctx, 0xB1, 0xE8, 0x00, 0x04, 0x02, 0xFF, 0x02, 0x00, 0x00, 0xFF, 0x14, 0xFF, 0xDE, 0xFD, 0x00, 0xF0, 0xFA, 0xEB, 0x00, 0xFF, 0xFF, 0xFF);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x55, 0xB1);
	lcm_dcs_write_seq_static(ctx, 0xB1, 0x80);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	msleep(100);

	/* Display On*/
	lcm_dcs_write_seq_static(ctx, 0x29);
	pr_err("debug_end for lcm %s\n", __func__);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for lcm %s\n", __func__);
/* #endif */ /* OPLUS_BUG_STABILITY */
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
/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for lcm %s\n", __func__);
/* #endif */ /* OPLUS_BUG_STABILITY */

	if (!ctx->prepared)
		return 0;

	ctx->error = 0;
	ctx->prepared = false;
	ctx->hbm_en = false;
	return 0;
}

extern int power_mode; 

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for lcm %s,power_mode:%d\n", __func__,power_mode);
/* #endif */ /* OPLUS_BUG_STABILITY */
	if (ctx->prepared)
		return 0;
	if(power_mode == 2){
		lcm_panel_init(ctx);
		aod_mode = 0;
	}

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif
	pr_err("%s - lcm_panel_init,resume status\n", __func__);
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for lcm %s\n", __func__);
/* #endif */ /* OPLUS_BUG_STABILITY */

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
	.clock = 170435,
	.hdisplay = 1080,
	.hsync_start = 1080 + 52,
	.hsync_end = 1080 + 52 + 14,
	.htotal = 1080 + 52 + 14 + 22,
	.vdisplay = 2400,
	.vsync_start = 2400 + 29,
	.vsync_end = 2400 + 29 + 2,
	.vtotal = 2400 + 29 + 2 + 9,
	.vrefresh = 60,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static int enter_aod(void *handle, int enter)
{
	/* enter & exit AOD cmd */
	return 0;
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

static unsigned long panel_doze_get_mode_flags(struct drm_panel *panel, int doze_en)
{
	unsigned long mode_flags;

	//DDPINFO("%s doze_en:%d\n", __func__, doze_en);
	if (doze_en) {
		mode_flags = MIPI_DSI_MODE_LPM
		       | MIPI_DSI_MODE_EOT_PACKET
		       | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	} else {
		mode_flags = MIPI_DSI_MODE_VIDEO
		       | MIPI_DSI_MODE_VIDEO_BURST
		       | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
		       | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	}

	/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for %s, mode flags =%d, doze_en = %d\n", __func__,mode_flags,doze_en);
	/* #endif */ /* OPLUS_BUG_STABILITY */
	return mode_flags;
}

/*
static int panel_doze_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	DDPINFO("%s\n", __func__);

	//lcm_normal_to_aod(ctx);
	lcm_panel_init_cmd(ctx);
	return 0;
}
*/

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	//struct lcm *ctx = panel_to_lcm(panel);
	unsigned int i=0;
/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for lcm %s\n", __func__);
/* #endif */ /* OPLUS_BUG_STABILITY */

	/* Switch back to VDO mode */
	for (i = 0; i < (sizeof(lcm_aod_to_normal) / sizeof(struct LCM_setting_table)); i++) {
		unsigned cmd;
		cmd = lcm_aod_to_normal[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY:
					msleep(lcm_aod_to_normal[i].count);
				break;

			case REGFLAG_UDELAY:
				udelay(lcm_aod_to_normal[i].count);
				break;

			case REGFLAG_END_OF_TABLE:
				break;

			default:
				cb(dsi, handle, lcm_aod_to_normal[i].para_list, lcm_aod_to_normal[i].count);
		}
	}
	aod_mode = 0;
	return 0;
}

static struct LCM_setting_table lcm_normal_to_aod_sam[] = {

	/* Internal VDO Packet generation enable*/
	{REGFLAG_CMD, 3, {0xF0, 0x5A,0x5A}},
	{REGFLAG_CMD, 3, {0xFC, 0x5A,0x5A}},
	{REGFLAG_CMD, 3, {0xB0, 0x14,0xFE}},
	{REGFLAG_CMD, 2, {0xFE, 0x12}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5,0xA5}},
	{REGFLAG_CMD, 3, {0xFC, 0xA5,0xA5}},

	/*Sleep out*/
	{REGFLAG_CMD, 1, {0x11}},
	{REGFLAG_DELAY, 10, {}},

	/* MIPI Mode cmd */
	{REGFLAG_CMD,3,{0xF0, 0x5A,0x5A}},
	{REGFLAG_CMD,2,{0xF2, 0x03}},
	{REGFLAG_CMD,3,{0xF0, 0xA5,0xA5}},

	/* TE vsync ON */
	{REGFLAG_CMD,2,{0x35,0x00}},
	{REGFLAG_DELAY,10,{}},

	/* AOD Setting */
	/*{REGFLAG_CMD,45,{0x81,0x38,0x1B,0x43,0x2B,0xC4,0x1A,
					 0x17,0x44,0x43,0x85,0x28,
					 0x1B,0x82,0x82,0x58,0xED,
					 0x00,0x00,0x00,0x00,0x00,
					 0x00,0x00,0x00,0x00,0x00,
					 0x00,0x00,0x00,0x00,0x00,
					 0x00,0x00,0x00,0x00,0x00,
					 0x00,0xFF,0xFF,0xFF,0x00,
					 0x00,0x00,0x00}},*/

	/* PCD setting off */
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2,{0xEA,0x48}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},

	/* AOD AMP ON */
	{REGFLAG_CMD,3,{0xFC,0x5A,0x5A}},
	{REGFLAG_CMD,3,{0xB0,0x06,0xFD}},
	{REGFLAG_CMD,2,{0xFD,0x85}},
	{REGFLAG_CMD,3,{0xFC,0xA5,0xA5}},

	/* AOD Mode On Setting */
	{REGFLAG_CMD,2,{0x53,0x22}},

	/* Internal VDO Packet generation enable*/
	{REGFLAG_CMD,3,{0xF0, 0x5A,0x5A}},
	{REGFLAG_CMD,3,{0xFC, 0x5A,0x5A}},
	{REGFLAG_CMD,3,{0xB0, 0x14,0xFE}},
	{REGFLAG_CMD,3,{0xFE, 0x10}},
	{REGFLAG_CMD,3,{0xF0, 0xA5,0xA5}},
	{REGFLAG_CMD,3,{0xFC, 0xA5,0xA5}},

	/*AOD IP Setting*/
	{REGFLAG_CMD,3,{0xF0, 0x5A,0x5A}},
	{REGFLAG_CMD,3,{0xB0, 0x03,0xC2}},
	{REGFLAG_CMD,3,{0xC2, 0x04}},
	{REGFLAG_CMD,3,{0xF0, 0xA5,0xA5}},

	/*seed setting*/
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x80, 0x92}},
	{REGFLAG_CMD,2,{0xB1, 0x00}},
	{REGFLAG_CMD,3,{0xB0, 0x2B,0xB1}},
	{REGFLAG_CMD,22,{0xB1, 0xE8, 0x00, 0x04, 0x02, 0xFF, 0x02, 0x00, 0x00, 0xFF, 0x14, 0xFF, 0xDE, 0xFD, 0x00, 0xF0, 0xFA, 0xEB, 0x00, 0xFF, 0xFF, 0xFF}},
	{REGFLAG_CMD,3,{0xB0, 0x55, 0xB1}},
	{REGFLAG_CMD,2,{0xB1, 0x80}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},

	{REGFLAG_DELAY,100,{}},

	#if 1
	/* Image Data Write for AOD Mode */
	/* Display on */
	{REGFLAG_CMD,1,{0x29}},

	/* MIPI Video cmd*/
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2,{0xF2,0x0F}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
	#endif

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i=0;
/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for lcm %s\n", __func__);
/* #endif */ /* OPLUS_BUG_STABILITY */

	for (i = 0; i < (sizeof(lcm_normal_to_aod_sam) / sizeof(struct LCM_setting_table)); i++) {
		unsigned cmd;
		cmd = lcm_normal_to_aod_sam[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY:
				msleep(lcm_normal_to_aod_sam[i].count);
				break;

			case REGFLAG_UDELAY:
				udelay(lcm_normal_to_aod_sam[i].count);
				break;

			case REGFLAG_END_OF_TABLE:
				break;

			default:
				cb(dsi, handle, lcm_normal_to_aod_sam[i].para_list, lcm_normal_to_aod_sam[i].count);
		}
	}
	flag_hbm = 0;
	aod_mode = 1;
	return 0;
}

static int panel_doze_enable_start(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;
/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for lcm %s\n", __func__);
/* #endif */ /* OPLUS_BUG_STABILITY */

	/* #ifdef OPLUS_BUG_STABILITY */
	struct lcm *ctx = panel_to_lcm(panel);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(2 * 1000, 2 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	/* #endif */

	msleep(5);
	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);
	cmd = 0x10;
	cb(dsi, handle, &cmd, 1);
	msleep(120);
	return 0;
}

static int panel_doze_enable_end(void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;
	int send_buf[3];
/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for lcm %s\n", __func__);
/* #endif */ /* OPLUS_BUG_STABILITY */

	cmd = 0x29;
	cb(dsi, handle, &cmd, 1);
	send_buf[0] = 0xF0;
	send_buf[1] = 0x5A;
	send_buf[2] = 0x5A;
	cb(dsi, handle, send_buf, 3);
	send_buf[0] = 0xF2;
	send_buf[1] = 0x0F;
	cb(dsi, handle, send_buf, 2);
	send_buf[0] = 0xF0;
	send_buf[1] = 0xA5;
	send_buf[2] = 0xA5;
	cb(dsi, handle, send_buf, 3);

	return 0;
}

static int panel_doze_area_set(void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;

/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for lcm %s\n", __func__);
/* #endif */ /* OPLUS_BUG_STABILITY */

#if 0
	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);
	msleep(12);
#endif
	cb(dsi, handle, send_cmd, ARRAY_SIZE(send_cmd));

	//memset(send_cmd, 0, RAMLESS_AOD_PAYLOAD_SIZE);
	return 0;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for lcm %s\n", __func__);
/* #endif */ /* OPLUS_BUG_STABILITY */
	if (level == 0) {
		for (i = 0; i < sizeof(lcm_aod_high_mode)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_aod_high_mode[i].para_list, lcm_aod_high_mode[i].count);
		}
	} else {
		for (i = 0; i < sizeof(lcm_aod_low_mode)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_aod_low_mode[i].para_list, lcm_aod_low_mode[i].count);
		}
	}
	printk("[soso] %s : %d !\n",__func__, level);

	//memset(send_cmd, 0, RAMLESS_AOD_PAYLOAD_SIZE);
	return 0;
}

static int panel_doze_post_disp_on(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;

/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for lcm %s\n", __func__);
/* #endif */ /* OPLUS_BUG_STABILITY */

	cmd = 0x29;
	cb(dsi, handle, &cmd, 1);
	//msleep(2);

	return 0;
}

static int panel_doze_post_disp_off(void *dsi, dcs_write_gce cb, void *handle)
{

	int cmd = 0;

/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for lcm %s\n", __func__);
/* #endif */ /* OPLUS_BUG_STABILITY */

	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);

	return 0;
}

static int oplus_lcm_dc_backlight(void *dsi, dcs_write_gce cb,void *handle,unsigned int level,int hbm_en)
{
	int i, k;
	struct LCM_setting_table *seed_table;
	int seed_alpha = oplus_seed_bright_to_alpha(level);
	if (!oplus_dc_enable_real || hbm_en || level >= OPLUS_DC_BACKLIGHT_THRESHOLD ||
	    level < 4) {
		goto dc_disable;
	}

	if (oplus_dc_alpha == seed_alpha)
		goto dc_enable;

	seed_table = kmemdup(lcm_seed_setting, sizeof(lcm_seed_setting), GFP_KERNEL);
	if (!seed_table)
		goto dc_disable;

	for (i = 0; i < sizeof(lcm_seed_setting)/sizeof(lcm_seed_setting[0]); i++) {
		if ((seed_table[i].para_list[0] == 0xB1) && (seed_table[i].count == 22)){
			for (k = 1; k < seed_table[i].count; k++) {
				seed_table[i].para_list[k]= seed_table[i].para_list[k] * (1020 - seed_alpha) / 1020;
			}
		}
	}

	for (i = 0; i < sizeof(lcm_seed_setting)/sizeof(lcm_seed_setting[0]); i++){
		cb(dsi, handle, seed_table[i].para_list, seed_table[i].count);
	}

	kfree(seed_table);
	if (!oplus_dc_alpha)
		pr_err("Enter DC");

	oplus_dc_alpha = seed_alpha;

dc_enable:
	return OPLUS_DC_BACKLIGHT_THRESHOLD;

dc_disable:
	if (oplus_dc_alpha) {
		for (i = 0; i < sizeof(lcm_seed_setting)/sizeof(lcm_seed_setting[0]); i++){
			cb(dsi, handle, lcm_seed_setting[i].para_list, lcm_seed_setting[i].count);
		}
		pr_err("exit DC");
	}

	oplus_dc_alpha = 0;
	return level;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	char bl_tb0[] = {0x51, 0x07, 0xFF};
	char bl_tb1[] = {0x53, 0x20};
	if (!cb)
		return -1;

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0) {
		mapped_level = 2047;
	} else {
		mapped_level = oplus_lcm_dc_backlight(dsi,cb,handle, level, 0);
	}
	esd_brightness = mapped_level;
	bl_tb0[1] = mapped_level >> 8;
	bl_tb0[2] = mapped_level & 0xFF;

	/*add for global hbm*/
/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for display panel backlight value,func:=%s,level :=%d, mapped_level := %d,aod_mode:= %d\n", __func__, level, mapped_level, aod_mode);
/* #endif */ /* OPLUS_BUG_STABILITY */
	if(aod_mode == 1 && level != 0) {
		return 0;
	}

	if(mapped_level <= 2047){
	    if(flag_hbm == 1){
			bl_tb1[1] = 0x20;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			flag_hbm = 0;
		}
	    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	} else if(mapped_level > 2047 && mapped_level <= 4095){
	    if(flag_hbm == 0){
			bl_tb1[1] = 0xE0;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			flag_hbm = 1;
			}
	    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	}
	return 0;

}

static int lcm_set_dc_backlight(void *dsi, dcs_write_gce cb,
           void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0x07, 0xFF};

	pr_err("lcm_set_dc_backlight ,backlight = %d\n", oplus_display_brightness);
	bl_tb0[1] = oplus_display_brightness >> 8;
	bl_tb0[2] = oplus_display_brightness & 0xFF;

	oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 1);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}

static int oplus_esd_backlight_check(void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0x51, 0x03, 0xff};

	pr_err("%s esd_backlight = %d\n", __func__, esd_brightness);
	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
	if (!cb)
		return -1;
	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 1;
}

static int lcm_set_safe_mode(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	char safe_lock[] = {0xFC, 0x5A, 0x5A};
	char safe_unlock[] = {0xFC, 0xA5, 0xA5};
	char safe_on[] = {0xED, 0x00,0x01,0x00,0x40,0xFF,0x08,0x87,0xA4,0x4A,0x73,0xE2,0x9F,0x00};
	char safe_off[] = {0xED, 0x00,0x01,0x00,0x40,0x04,0x08,0x87,0x84,0x4A,0x73,0xE2,0x1F,0x00};

	if (!cb)
		return -1;

	pr_err("debug for safe_mode= %u, level=%u\n", safe_mode, level);
	if(safe_mode != level) {
		if(level == 1) {
			cb(dsi, handle, safe_lock, ARRAY_SIZE(safe_lock));
			cb(dsi, handle, safe_on, ARRAY_SIZE(safe_on));
			cb(dsi, handle, safe_unlock, ARRAY_SIZE(safe_unlock));
		} else if (level == 0) {
			cb(dsi, handle, safe_lock, ARRAY_SIZE(safe_lock));
			cb(dsi, handle, safe_off, ARRAY_SIZE(safe_off));
			cb(dsi, handle, safe_unlock, ARRAY_SIZE(safe_unlock));
		}
		safe_mode = level;
	}

	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for lcm %s\n", __func__);
/* #endif */ /* OPLUS_BUG_STABILITY */

	if (ctx->prepared)
		return 0;

	ctx->bias_vci = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_vci, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_vci);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10 * 1000, 10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(1 * 1000, 1 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(5 * 1000, 5 * 1000);
	pr_err("debug for lcm_end %s\n", __func__);
	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

/* #ifdef OPLUS_BUG_STABILITY */
	pr_err("debug for lcm %s\n", __func__);
/* #endif */ /* OPLUS_BUG_STABILITY */

	if (ctx->prepared)
		return 0;

        ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->reset_gpio, 0);
        devm_gpiod_put(ctx->dev, ctx->reset_gpio);
        msleep(10);
        pr_info("%s: going to cut off power \n", __func__);
        ctx->bias_vci = devm_gpiod_get_index(ctx->dev,
                "bias", 0, GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->bias_vci, 0);
        devm_gpiod_put(ctx->dev, ctx->bias_vci);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	return 0;
}

static int lcm_panel_disp_off(void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;

	DDPINFO("%s\n", __func__);

	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);
	usleep_range(10*1000,11*1000);

	cmd = 0x10;
	cb(dsi, handle, &cmd, 1);
	usleep_range(120*1000,121*1000);

	return 0;
}


static void lcm_setbrightness(void *dsi,
			      dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int BL_MSB = 0;
	unsigned int BL_LSB = 0;
	unsigned int hbm_brightness = 0;
	int i = 0;

	printk("[soso] %s level is %d\n", __func__, level);

	if (level > BRIGHTNESS_HALF) {
		hbm_brightness = level - 2047;
		BL_LSB = hbm_brightness >> 8;
		BL_MSB = hbm_brightness & 0xFF;

		lcm_setbrightness_hbm[0].para_list[1] = BL_LSB;
		lcm_setbrightness_hbm[0].para_list[2] = BL_MSB;

		for (i = 0; i < sizeof(lcm_setbrightness_hbm)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_setbrightness_hbm[i].para_list, lcm_setbrightness_hbm[i].count);
		}
	} else {
		BL_LSB = level >> 8;
		BL_MSB = level & 0xFF;

		lcm_setbrightness_normal[0].para_list[1] = BL_LSB;
		lcm_setbrightness_normal[0].para_list[2] = BL_MSB;

		for (i = 0; i < sizeof(lcm_setbrightness_normal)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_setbrightness_normal[i].para_list, lcm_setbrightness_normal[i].count);
		}
	}
}

static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	int i = 0;
	int level = 0;
	if (!cb)
		return -1;

	pr_err("oplus_display_brightness= %ld, hbm_mode=%u\n", oplus_display_brightness, hbm_mode);

	if(hbm_mode == 1) {
		//oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 1);
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (hbm_mode == 0) {
		/* level = oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 0); */
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);
		printk("[soso] %s : %d ! backlight %d !\n",__func__, hbm_mode, level);
	}

	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
			      dcs_write_gce cb, void *handle, bool en)
{
	//char hbm_tb[] = {0x53, 0xe0};
	struct lcm *ctx = panel_to_lcm(panel);
	int i = 0;
	int level = 0;
	if (!cb)
		return -1;
	if (ctx->hbm_en == en)
		goto done;

	pr_err("debug for oplus_display_brightness= %ld, en=%u\n", oplus_display_brightness, en);

	if(en == 1) {
		//oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 1);
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (en == 0) {
		level = oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 0);
		lcm_setbrightness(dsi, cb, handle, level);
		printk("[soso] %s : %d ! backlight %d !\n",__func__, en, level);
		if (level <= BRIGHTNESS_HALF)
			flag_hbm = 0;
		else
			flag_hbm = 1;
	}
	ctx->hbm_en = en;
	ctx->hbm_wait = true;
done:
	return 0;
}

static void panel_hbm_get_state(struct drm_panel *panel, bool *state)
{
	struct lcm *ctx = panel_to_lcm(panel);

	*state = ctx->hbm_en;
}

static void panel_hbm_set_state(struct drm_panel *panel, bool state)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->hbm_en = state;
}

static void panel_hbm_get_wait_state(struct drm_panel *panel, bool *wait)
{
	struct lcm *ctx = panel_to_lcm(panel);

	*wait = ctx->hbm_wait;
}

static bool panel_hbm_set_wait_state(struct drm_panel *panel, bool wait)
{
	struct lcm *ctx = panel_to_lcm(panel);
	bool old = ctx->hbm_wait;

	ctx->hbm_wait = wait;
	return old;
}

static struct mtk_panel_params ext_params = {
	.pll_clk = 553,
	.data_rate = 1107,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	//.lcm_esd_check_table[0] = {
	//	.cmd = 0x0a,
	//	.count = 1,
	//	.para_list[0] = 0x9F,
	//},
	.oplus_lpx_ns_multiplier = 0x4B,
	.phy_timcon = {
		.clk_trail = 13,
	},
	.dyn = {
		.switch_en = 1,
		.data_rate = 1124,
		.hbp = 41,
	},
#ifndef OPLUS_BUG_STABILITY
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
#endif
	.hbm_en_time = 2,
	.hbm_dis_time = 1,
	.vendor = "AMS643XY04_MTK05",
	.manufacture = "samsung4096",
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.oplus_panel_cv_switch = 1,
	.oplus_dc_then_hbm_on = 1,
};

static struct mtk_panel_funcs ext_funcs = {
//	.aod = enter_aod,
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.doze_enable = panel_doze_enable,
	.doze_enable_start = panel_doze_enable_start,
	/* .doze_enable_end = panel_doze_enable_end, */
	.doze_disable = panel_doze_disable,
	//.doze_area = panel_doze_area,
	.doze_get_mode_flags = panel_doze_get_mode_flags,
	/* .doze_post_disp_on = panel_doze_post_disp_on, */
	/* .doze_post_disp_off = panel_doze_post_disp_off, */
	.set_hbm = lcm_set_hbm,
	#if FALSE
	.set_safe_mode = lcm_set_safe_mode,
	#endif
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.esd_backlight_recovery = oplus_esd_backlight_check,
	#if 1
	.panel_disp_off = lcm_panel_disp_off,
	#endif
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.hbm_get_state = panel_hbm_get_state,
	.hbm_set_state = panel_hbm_set_state,
	.hbm_get_wait_state = panel_hbm_get_wait_state,
	.hbm_set_wait_state = panel_hbm_set_wait_state,
	.doze_area_set = panel_doze_area_set,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.set_dc_backlight = lcm_set_dc_backlight,
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

	panel->connector->display_info.width_mm = 67;
	panel->connector->display_info.height_mm = 149;

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

	oplus_dc_threshold = OPLUS_DC_BACKLIGHT_THRESHOLD;
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

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;

#if (LCM_DSI_CMD_MODE)
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;
#else
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST
			 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

#endif
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
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	ctx->hbm_en = false;

	register_device_proc("lcd", "AMS643XY04_MTK05", "samsung4096");
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
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

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "20151,samsung,dphy,vdo", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "oplus20151_samsung_ams643_1080p_dsi_vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);
