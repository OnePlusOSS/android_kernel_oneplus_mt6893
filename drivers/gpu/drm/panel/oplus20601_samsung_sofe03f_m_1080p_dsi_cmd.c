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

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/oplus20171_AMB655XL08_data_hw_roundedpattern.h"
#include "../mediatek/mtk_corner_pattern/oplus20171_AMB655XL08_data_hw_roundedpattern_l.h"
#include "../mediatek/mtk_corner_pattern/oplus20171_AMB655XL08_data_hw_roundedpattern_r.h"
#endif

#ifdef CONFIG_MACH_MT6885
#include <soc/oplus/system/oplus_project.h>
//extern int is_check_camera_status(void);
int lcm_status = 1;
#endif

extern u32 flag_writ;
extern u32 flag_hbm;
static u32 aod_mode = 0;
static int esd_brightness;
extern int is_fan53870_pmic(void);
extern int fan53870_ldo1_20601_set_voltage(int set_uV);
extern int fan53870_ldo1_20601_disable(void);
#define RAMLESS_AOD_PAYLOAD_SIZE	100
#define OPLUS_DC_BACKLIGHT_THRESHOLD 260
extern int oplus_dc_alpha;
extern int oplus_dc_enable_real;
extern int oplus_dc_enable;
extern int exit_dc_flag;
extern unsigned long oplus_display_brightness;
extern char send_cmd[RAMLESS_AOD_PAYLOAD_SIZE];
extern void disp_aal_set_dre_en(int enable);

#define LCM_DSI_CMD_MODE 1

#define REGFLAG_CMD       0xFFFA
#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD

#define BRIGHTNESS_MAX    2047
#define BRIGHTNESS_HALF   1023

struct ba {
        u32 brightness;
        u32 alpha;
};

static struct ba brightness_seed_alpha_lut_dc[] = {
        {0, 0xff},
        {1, 0xfc},
        {2, 0xfb},
        {3, 0xfa},
        {4, 0xf9},
        {5, 0xf8},
        {6, 0xf7},
        {8, 0xf6},
        {10, 0xf4},
        {15, 0xf0},
        {20, 0xea},
        {30, 0xe0},
        {45, 0xd0},
        {70, 0xbc},
        {100, 0x98},
        {120, 0x80},
        {140, 0x70},
        {160, 0x58},
        {180, 0x48},
        {200, 0x30},
        {220, 0x20},
        {240, 0x10},
        {260, 0x00},
};

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

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_gpio;

	bool prepared;
	bool enabled;

	int error;

	bool hbm_en;
	bool hbm_wait;
	bool cv_state;
};

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[128];
};

static struct LCM_setting_table lcm_aod_to_normal[] = {
	    /*display off*/
        //{REGFLAG_CMD, 1, {0x28}},
        //{REGFLAG_DELAY,17,{}},

        {REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
        {REGFLAG_CMD,2,{0xB0,0x0B}},
        {REGFLAG_CMD,3,{0xD8,0x09,0x70}},
        {REGFLAG_CMD,2,{0x53,0x20}},
        {REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
        /*seed Setting*/
        {REGFLAG_CMD,2,{0x81, 0x92}},
        {REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
        {REGFLAG_CMD,2,{0xB0, 0x2B}},
        {REGFLAG_CMD,22,{0xB1,0xC6,0x05,0x00,0x09,0xD1,0x00,0x0C,0x00,0xC4,
                          0x17,0xEF,0xD3,0xEB,0x05,0xD2,0xE2,0xEA,0x00,
                          0xFC,0xFF,0xFF}},
        {REGFLAG_CMD,2,{0xB1, 0x00}},
        {REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},

        //{REGFLAG_DELAY,17,{}},
        {REGFLAG_CMD,1,{0x29}},
        {REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_normal_to_aod_sam[] = {
	/* DSC Setting */
	{REGFLAG_CMD, 129, {0x9E, 0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x09, 0x60, 0x04, 0x38,\
					 0x00, 0x1E, 0x02, 0x1C, 0x02, 0x1C, 0x02, 0x00, 0x02, 0x0E,\
					 0x00, 0x20, 0x02, 0xE3, 0x00, 0x07, 0x00, 0x0C, 0x03, 0x50,\
					 0x03, 0x64, 0x18, 0x00, 0x10, 0xF0, 0x03, 0x0C, 0x20, 0x00,\
					 0x06, 0x0B, 0x0B, 0x33, 0x0E, 0x1C, 0x2A, 0x38, 0x46, 0x54,\
					 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B, 0x7D, 0x7E, 0x01, 0x02,\
					 0x01, 0x00, 0x09, 0x40, 0x09, 0xBE, 0x19, 0xFC, 0x19, 0xFA,\
					 0x19, 0xF8, 0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A, 0xF6,\
					 0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4, 0x00, 0x00,\
					 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
					 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
					 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
					 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{REGFLAG_CMD,2,{0x9D,0x01}},

	{REGFLAG_CMD,1,{0x11}},
	{REGFLAG_DELAY,15,{}},

	 /* VLIN current limit */
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0xB0, 0x04}},
	{REGFLAG_CMD, 6, {0xD5, 0x24, 0x9E, 0x9E, 0x00, 0x20}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},

	/* OSC Setting */
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 3, {0xFC, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0xB0, 0x16}},
	{REGFLAG_CMD, 2, {0xD1, 0x22}},
	{REGFLAG_CMD, 2, {0xD6, 0x11}},
	{REGFLAG_CMD, 3, {0xFC, 0xA5, 0xA5}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},

	/* MIPI CLK Setting 415M*/
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 3, {0xFC, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0xB0, 0x01}},
	{REGFLAG_CMD, 4, {0xE4, 0xDC, 0x9B, 0xE7}},
	{REGFLAG_CMD, 15, {0xE9, 0x11, 0x75, 0xDC, 0x9B, 0xE7, 0x96, 0x2F,\
                             0x58, 0x96, 0x2F, 0x58, 0x00, 0x32, 0x32}},
	{REGFLAG_CMD, 3, {0xFC, 0xA5, 0xA5}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},

	/* TE vsync ON */
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0x35, 0x00}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},

	/* CASET/PASET Setting */
	{REGFLAG_CMD, 5, {0x2A, 0x00,0x00,0x04,0x37}},
	{REGFLAG_CMD, 5, {0x2B, 0x00,0x00,0x09,0x5F}},

	/* ERR_FG Setting */
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0xB0, 0x02}},
	{REGFLAG_CMD, 5, {0xEC, 0x00, 0xC2, 0xC2, 0x42}},
	{REGFLAG_CMD, 2, {0xB0, 0x0D}},
	{REGFLAG_CMD, 2, {0xEC, 0x19}},
	{REGFLAG_CMD, 2, {0xB0, 0x06}},
	{REGFLAG_CMD, 2, {0xE4, 0xD0}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},

	/* fast discharge Setting */
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0xD5, 0x8D}},
	{REGFLAG_CMD, 2, {0xB0, 0x0A}},
	{REGFLAG_CMD, 2, {0xD5, 0x05}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},

	/* ELVSS Dim Setting */
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0xB0, 0x06}},
	{REGFLAG_CMD, 2, {0xB7, 0x01}},
	{REGFLAG_CMD, 2, {0xB0, 0x05}},
	{REGFLAG_CMD, 2, {0xB7, 0x13}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},

	/* Dimming Delay control */
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0xB0, 0x01}},
	{REGFLAG_CMD, 2, {0xB7, 0x4C}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},

	/* ACL off */
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0x55, 0x00}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},

	/* 60hz Transition */
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0x60, 0x00}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},

	/* Backlight Dimming Setting */
	/*{REGFLAG_CMD, 3, {0x51, 0x07, 0xFF}},*/
	{REGFLAG_DELAY,65,{}},
	{REGFLAG_CMD, 2, {0x53, 0x28}},

	/* Display off*/
	{REGFLAG_CMD, 3, {0x9F, 0x5A,0x5A}},
	{REGFLAG_CMD, 1, {0x28}},
	{REGFLAG_CMD, 3, {0x9F, 0xA5,0xA5}},
	{REGFLAG_DELAY,20,{}},

	/*AOD On*/
	{REGFLAG_CMD,3,{0xF0, 0x5A,0x5A}},
	{REGFLAG_CMD,2,{0x60, 0X00}},
	{REGFLAG_CMD,2,{0xB0, 0x0B}},
	{REGFLAG_CMD,3,{0xD8, 0x00,0x00}},
	{REGFLAG_CMD,2,{0x53, 0x22}},
	{REGFLAG_CMD,2,{0xE3, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0xA5,0xA5}},

	/*seed Setting*/
	{REGFLAG_CMD,2,{0x81, 0x92}},
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0xB0, 0x2B}},
	{REGFLAG_CMD,22,{0xB1,0xC6,0x05,0x00,0x09,0xD1,0x00,0x0C,0x00,0xC4,
                          0x17,0xEF,0xD3,0xEB,0x05,0xD2,0xE2,0xEA,0x00,
                          0xFC,0xFF,0xFF}},
	{REGFLAG_CMD,2,{0xB1, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},

	{REGFLAG_DELAY,20,{}},
	/* Image Data Write for AOD Mode */
	/* Display on */
	//{REGFLAG_CMD,1,{0x29}},
	//{REGFLAG_CMD,1,{0x13}},
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

static struct LCM_setting_table lcm_seed_setting[] = {
        {REGFLAG_CMD,2,{0x81, 0x92}},
        {REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
        {REGFLAG_CMD,2,{0xB0, 0x2B}},
        {REGFLAG_CMD,22,{0xB1,0xC6,0x05,0x00,0x09,0xD1,0x00,0x0C,0x00,0xC4,
                          0x17,0xEF,0xD3,0xEB,0x05,0xD2,0xE2,0xEA,0x00,
                          0xFC,0xFF,0xFF}},
        {REGFLAG_CMD,2,{0xB1, 0x00}},
        {REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
        {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setbrightness_normal[] = {
        {REGFLAG_CMD,3, {0x51,0x00,0x00}},
        {REGFLAG_CMD,3, {0xF0, 0x5A, 0x5A}},
        {REGFLAG_CMD,2, {0x53,0x20}},
        {REGFLAG_CMD,3, {0xF0, 0xA5, 0xA5}},
        {REGFLAG_CMD,3, {0xF0, 0x5A, 0x5A}},
        {REGFLAG_CMD,2, {0xB0,0x01}},
        {REGFLAG_CMD,2, {0xB7,0x4C}},
        {REGFLAG_CMD,3, {0xF0, 0xA5, 0xA5}},
        {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setbrightness_hbm[] = {
        {REGFLAG_CMD,3, {0x51,0x00,0x00}},
        {REGFLAG_CMD,3, {0xF0, 0x5A, 0x5A}},
        {REGFLAG_CMD,2, {0x53,0xE8}},
        {REGFLAG_CMD,3, {0xF0, 0xA5, 0xA5}},
        {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_finger_HBM_on_setting[] = {
        {REGFLAG_CMD,3, {0xF0, 0x5A, 0x5A}},
        {REGFLAG_CMD,2, {0xB0,0x01}},
        {REGFLAG_CMD,2, {0xB7,0x4C}},
        {REGFLAG_CMD,3, {0xF0, 0xA5, 0xA5}},
        {REGFLAG_CMD,3, {0xF0, 0x5A, 0x5A}},
        {REGFLAG_CMD,3, {0x51,0x0B,0xF4}},
        {REGFLAG_CMD,2, {0x53,0xE0}},
        {REGFLAG_CMD,3, {0xF0, 0xA5, 0xA5}},
        {REGFLAG_CMD,3, {0xF0, 0x5A, 0x5A}},
        {REGFLAG_CMD,3, {0xB0,0x01}},
        {REGFLAG_CMD,2, {0xB7,0x44}},
        {REGFLAG_CMD,3, {0xF0, 0xA5, 0xA5}},
        {REGFLAG_END_OF_TABLE, 0x00, {}}
};

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128, "DCS sequence too big for stack");\
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

static struct regulator *disp_ldo3;

static int lcm_panel_ldo3_regulator_init(struct device *dev)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;
    pr_err("get lcm_panel_ldo3_regulator_init\n");

	/* please only get regulator once in a driver */
	disp_ldo3 = regulator_get(dev, "VMC");
	if (IS_ERR(disp_ldo3)) { /* handle return value */
		ret = PTR_ERR(disp_ldo3);
		pr_err("get ldo3 fail, error: %d\n", ret);
		return ret;
	}
	regulator_inited = 1;
	return ret; /* must be 0 */

}

static int lcm_panel_ldo3_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_ldo3_regulator_init(dev);

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_ldo3, 3000000, 3000000);
	if (ret < 0)
		pr_err("set voltage disp_ldo3 fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(disp_ldo3);
	if (ret < 0)
		pr_err("enable regulator disp_ldo3 fail, ret = %d\n", ret);
	retval |= ret;
    pr_err("get lcm_panel_ldo3_enable\n");

	return retval;
}

static int lcm_panel_ldo3_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_ldo3_regulator_init(dev);

	ret = regulator_disable(disp_ldo3);
	if (ret < 0)
		pr_err("disable regulator disp_ldo3 fail, ret = %d\n", ret);
	retval |= ret;
    pr_err("disable regulator disp_ldo3\n");

	return retval;
}

#ifdef CONFIG_MACH_MT6885
int is_check_lcm_status(void)
{
	pr_err("For checking lcm_status: %d \n", lcm_status);
	return lcm_status;
}
EXPORT_SYMBOL(is_check_lcm_status);
#endif

static void lcm_panel_init(struct lcm *ctx)
{
	pr_err("debug for lcm ZJB%s\n", __func__);
	/* DSC Setting */
	lcm_dcs_write_seq_static(ctx,0x9E, 0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x09, 0x60, 0x04, 0x38,\
								 0x00, 0x1E, 0x02, 0x1C, 0x02, 0x1C, 0x02, 0x00, 0x02, 0x0E,\
								 0x00, 0x20, 0x02, 0xE3, 0x00, 0x07, 0x00, 0x0C, 0x03, 0x50,\
								 0x03, 0x64, 0x18, 0x00, 0x10, 0xF0, 0x03, 0x0C, 0x20, 0x00,\
								 0x06, 0x0B, 0x0B, 0x33, 0x0E, 0x1C, 0x2A, 0x38, 0x46, 0x54,\
								 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B, 0x7D, 0x7E, 0x01, 0x02,\
								 0x01, 0x00, 0x09, 0x40, 0x09, 0xBE, 0x19, 0xFC, 0x19, 0xFA,\
								 0x19, 0xF8, 0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A, 0xF6,\
								 0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4, 0x00, 0x00,\
								 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
								 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
								 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
								 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx,0x9D,0x01);
	/* Display On Setting */
	lcm_dcs_write_seq_static(ctx, 0x11);
	msleep(15);
	 /* VLIN current limit */
	lcm_dcs_write_seq_static(ctx,0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x04);
	lcm_dcs_write_seq_static(ctx,0xD5, 0x24, 0x9E, 0x9E, 0x00, 0x20);
	lcm_dcs_write_seq_static(ctx,0xF0, 0xA5, 0xA5);
	/* OSC Setting */
	lcm_dcs_write_seq_static(ctx,0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx,0xFC, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x16);
	lcm_dcs_write_seq_static(ctx,0xD1, 0x22);
	lcm_dcs_write_seq_static(ctx,0xD6, 0x11);
	lcm_dcs_write_seq_static(ctx,0xFC, 0xA5, 0xA5);
	lcm_dcs_write_seq_static(ctx,0xF0, 0xA5, 0xA5);
	/* MIPI CLK Setting 415M*/
	lcm_dcs_write_seq_static(ctx,0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx,0xFC, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x01);
	lcm_dcs_write_seq_static(ctx,0xE4, 0xDC, 0x9B, 0xE7);
	lcm_dcs_write_seq_static(ctx,0xE9, 0x11, 0x75, 0xDC, 0x9B, 0xE7, 0x96, 0x2F,\
                                    0x58, 0x96, 0x2F, 0x58, 0x00, 0x32, 0x32);
	lcm_dcs_write_seq_static(ctx,0xFC, 0xA5, 0xA5);
	lcm_dcs_write_seq_static(ctx,0xF0, 0xA5, 0xA5);
	/* TE vsync ON */
	lcm_dcs_write_seq_static(ctx,0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx,0x35, 0x00);
	lcm_dcs_write_seq_static(ctx,0xF0, 0xA5, 0xA5);
	/* Tsp hsync ON */
	/*
	lcm_dcs_write_seq_static(ctx,0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx,0xDF, 0x83, 0x00, 0x10);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x01);
	lcm_dcs_write_seq_static(ctx,0xE6, 0x01);
	lcm_dcs_write_seq_static(ctx,0xF0, 0xA5, 0xA5);
	*/
	/* CASET/PASET Setting */
	lcm_dcs_write_seq_static(ctx,0x2A, 0x00,0x00,0x04,0x37);
	lcm_dcs_write_seq_static(ctx,0x2B, 0x00,0x00,0x09,0x5F);
	/* ERR_FG Setting */
	lcm_dcs_write_seq_static(ctx,0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x02);
	lcm_dcs_write_seq_static(ctx,0xEC, 0x00, 0xC2, 0xC2, 0x42);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x0D);
	lcm_dcs_write_seq_static(ctx,0xEC, 0x19);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x06);
	lcm_dcs_write_seq_static(ctx,0xE4, 0xD0);
	lcm_dcs_write_seq_static(ctx,0xF0, 0xA5, 0xA5);
	/* FD Setting */
	lcm_dcs_write_seq_static(ctx,0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx,0xD5, 0x8D);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x0A);
	lcm_dcs_write_seq_static(ctx,0xD5, 0x05);
	lcm_dcs_write_seq_static(ctx,0xF0, 0xA5, 0xA5);
	/* ELVSS Dim Setting */
	lcm_dcs_write_seq_static(ctx,0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x06);
	lcm_dcs_write_seq_static(ctx,0xB7, 0x01);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x05);
	lcm_dcs_write_seq_static(ctx,0xB7, 0x13);
	lcm_dcs_write_seq_static(ctx,0xF0, 0xA5, 0xA5);
	/* Dimming Delay control */
	lcm_dcs_write_seq_static(ctx,0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x01);
	lcm_dcs_write_seq_static(ctx,0xB7, 0x4C);
	lcm_dcs_write_seq_static(ctx,0xF0, 0xA5, 0xA5);
	/* ACL off */
	lcm_dcs_write_seq_static(ctx,0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx,0x55, 0x00);
	lcm_dcs_write_seq_static(ctx,0xF0, 0xA5, 0xA5);
	/* 60hz Transition */
	lcm_dcs_write_seq_static(ctx,0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx,0x60, 0x00);
	lcm_dcs_write_seq_static(ctx,0xF0, 0xA5, 0xA5);
	/* seed setting */
	lcm_dcs_write_seq_static(ctx,0x81, 0x92);
	lcm_dcs_write_seq_static(ctx,0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x2B);
	lcm_dcs_write_seq_static(ctx,0xB1, 0xC6, 0x05, 0x00, 0x09, 0xD1, 0x00, 0x0C, 0x00, 0xC4, 0x17, 0xEF, 0xD3, 0xEB, 0x05, 0xD2, 0xE2, 0xEA, 0x00, 0xFC, 0xFF, 0xFF);
	lcm_dcs_write_seq_static(ctx,0xB1, 0x00);
	lcm_dcs_write_seq_static(ctx,0xF0, 0xA5, 0xA5);
	/* Backlight Dimming Setting */
	/*lcm_dcs_write_seq_static(ctx,0x51, 0x07, 0xFF);*/
	msleep(65);
	lcm_dcs_write_seq_static(ctx,0x53, 0x28);
	/* Display On*/
	lcm_dcs_write_seq_static(ctx,0x9F, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx,0x29);
	lcm_dcs_write_seq_static(ctx,0x9F, 0xA5, 0xA5);

}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_err("debug for lcm %s\n", __func__);
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

	pr_err("debug for lcm %s\n", __func__);

	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(10);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(120);

	ctx->error = 0;
	ctx->prepared = false;
#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_disable();
#endif
#if 0
	ctx->bias_gpio =
	devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_gpio, 0);
	msleep(15);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);
#endif
	ctx->hbm_en = false;
	return 0;
}

extern int power_mode;
static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;
        pr_err("debug for lcm %s\n", __func__);

#if 0
	ctx->bias_gpio =
	devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);
#endif
	
	if(power_mode == 2)
	{
		DDPINFO("%s + lcm_panel_init,resume status\n", __func__);
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

	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	pr_err("debug for lcm %s\n", __func__);

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
	.clock = 163530,
	.hdisplay = 1080,
	.hsync_start = 1080 + 40,
	.hsync_end = 1080 + 40 + 10,
	.htotal = 1080 + 40 + 10 + 20,
	.vdisplay = 2400,
	.vsync_start = 2400 + 20,
	.vsync_end = 2400 + 20 + 2,
	.vtotal = 2400 + 20 + 2 + 8,
	.vrefresh = 60,

};

static const struct drm_display_mode performance_mode = {
	.clock = 327060,
	.hdisplay = 1080,
	.hsync_start = 1080 + 40,
	.hsync_end = 1080 + 40 + 10,
	.htotal = 1080 + 40 + 10 + 20,
	.vdisplay = 2400,
	.vsync_start = 2400 + 20,
	.vsync_end = 2400 + 20 + 2,
	.vtotal = 2400 + 20 + 2 + 8,
	.vrefresh = 120,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xA2, .count = 1, .para_list[0] = 0x11, .mask_list[0] = 0x11,
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
		.slice_height = 30,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 739,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 848,
		.slice_bpg_offset = 868,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
	},
	.data_rate = 830,
	.hbm_en_time = 2,
	.hbm_dis_time = 1,
	.oplus_display_global_dre = 1,
    .vendor = "AMS644VA04_MTK04",
    .manufacture = "samsung1024",
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
	.corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
	.corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
	.corner_pattern_tp_size_r = sizeof(top_rc_pattern_r),
	.corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 60,
	},
};

static struct mtk_panel_params ext_params_120hz = {
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
        .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
        .cmd = 0xA2, .count = 1, .para_list[0] = 0x11, .mask_list[0] = 0x11,
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
       .slice_height = 30,
       .slice_width = 540,
       .chunk_size = 540,
       .xmit_delay = 512,
       .dec_delay = 526,
       .scale_value = 32,
       .increment_interval = 739,
       .decrement_interval = 7,
       .line_bpg_offset = 12,
       .nfl_bpg_offset = 848,
       .slice_bpg_offset = 868,
       .initial_offset = 6144,
       .final_offset = 4336,
       .flatness_minqp = 3,
       .flatness_maxqp = 12,
       .rc_model_size = 8192,
       .rc_edge_factor = 6,
       .rc_quant_incr_limit0 = 11,
       .rc_quant_incr_limit1 = 11,
       .rc_tgt_offset_hi = 3,
       .rc_tgt_offset_lo = 3,
       },
    .data_rate = 830,
    .hbm_en_time = 2,
	.hbm_dis_time = 1,
	.oplus_display_global_dre = 1,
    .vendor = "AMS644VA04_MTK04",
    .manufacture = "samsung1024",
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
	.corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
	.corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
	.corner_pattern_tp_size_r = sizeof(top_rc_pattern_r),
	.corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
    .dyn_fps = {
        .switch_en = 1, .vact_timing_fps = 120,
    },

};
static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	if (mode == 0)
		ext->params = &ext_params;
	else if (mode == 1)
		ext->params = &ext_params_120hz;
	else
		ret = 1;

	return ret;
}

static void mode_switch_60_to_120(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
        struct lcm *ctx = panel_to_lcm(panel);

        if (stage == BEFORE_DSI_POWERDOWN) {
                /* display off */
        } else if (stage == AFTER_DSI_POWERON) {
                /* display on switch to 120hz */
        lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
        lcm_dcs_write_seq_static(ctx, 0x60, 0x10);
        lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
    }
}

static void mode_switch_120_to_60(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
        struct lcm *ctx = panel_to_lcm(panel);

        if (stage == BEFORE_DSI_POWERDOWN) {
                /* display off */
        } else if (stage == AFTER_DSI_POWERON) {
                /* display on switch to 60hz */
                lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
                lcm_dcs_write_seq_static(ctx, 0x60, 0x00);
                lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
        }
}

static int mode_switch(struct drm_panel *panel, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;

	if (cur_mode == 0 && dst_mode == 1) { /* 60 switch to 90 */
		mode_switch_60_to_120(panel, stage);
	} else if (cur_mode == 1 && dst_mode == 0) { /* 90 switch to 60 */
		mode_switch_120_to_60(panel, stage);
	} else
		ret = 1;

	return ret;
}

static int enter_aod(void *handle, int enter)
{
	/* enter & exit AOD cmd */
	return 0;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
#if 0
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
#endif
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

	pr_err("debug for %s, mode flags =%d, doze_en = %d\n", __func__,mode_flags,doze_en);
	return mode_flags;
}
/*static int panel_doze_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	DDPINFO("%s\n", __func__);

	//lcm_normal_to_aod(ctx);
	lcm_panel_init_cmd(ctx);
	return 0;
}*/

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	//struct lcm *ctx = panel_to_lcm(panel);
	unsigned int i=0;
	pr_err("debug for lcm %s\n", __func__);

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

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i=0;
	pr_err("debug for lcm %s\n", __func__);

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

static int panel_doze_disp_on(void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;
	//int send_buf[3];
	pr_err("lcm: %s\n", __func__);

	cmd = 0x29;
	cb(dsi, handle, &cmd, 1);

	return 0;
}

static int panel_doze_enable_start(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;
	pr_err("debug for lcm %s\n", __func__);

	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);
	cmd = 0x10;
	cb(dsi, handle, &cmd, 1);

	return 0;
}

static int panel_doze_enable_end(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;
	int send_buf[3];
	pr_err("debug for lcm %s\n", __func__);

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
#if 0
static int panel_doze_area_set(void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;

	pr_err("debug for lcm %s\n", __func__);

#if 0
	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);
	msleep(12);
#endif
	cb(dsi, handle, send_cmd, ARRAY_SIZE(send_cmd));

	//memset(send_cmd, 0, RAMLESS_AOD_PAYLOAD_SIZE);
	return 0;
}
#endif
static int panel_doze_post_disp_on(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{

	int cmd = 0;

	pr_err("debug for lcm %s\n", __func__);

	cmd = 0x29;
	cb(dsi, handle, &cmd, 1);

	return 0;
}


static int panel_doze_post_disp_off(void *dsi, dcs_write_gce cb, void *handle)
{

	int cmd = 0;

	pr_err("debug for lcm %s\n", __func__);

	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);

	return 0;
}

static int map_exp[4096] = {0};

void init_global_exp_backlight(void)
{
	int lut_index[41] = {0, 4, 99, 144, 187, 227, 264, 300, 334, 366, 397, 427, 456, 484, 511, 537, 563, 587, 611, 635, 658, 680,
						702, 723, 744, 764, 784, 804, 823, 842, 861, 879, 897, 915, 933, 950, 967, 984, 1000, 1016, 1023};
	int lut_value1[41] = {0, 4, 6, 14, 24, 37, 52, 69, 87, 107, 128, 150, 173, 197, 222, 248, 275, 302, 330, 358, 387, 416, 446, 
						477, 507, 539, 570, 602, 634, 667, 700, 733, 767, 801, 835, 869, 903, 938, 973, 1008, 1023};
	int index_start = 0, index_end = 0;
	int value1_start = 0, value1_end = 0;
	int i,j;
	int index_len = sizeof(lut_index) / sizeof(int);
	int value_len = sizeof(lut_value1) / sizeof(int);
	int res =0, sub = 0, mod = 0;
	if (index_len == value_len) {
		for (i = 0; i < index_len - 1; i++) {
			index_start = lut_index[i];
			index_end = lut_index[i+1];
			value1_start = lut_value1[i];
			value1_end = lut_value1[i+1];
			for (j = index_start; j <= index_end; j++) {
				//map_exp[j] = value1_start + (value1_end - value1_start) * (j - index_start) / (index_end - index_start);
				res = 2 * (value1_end - value1_start) * (j - index_start) / (index_end - index_start);
				sub = res / 2;
				mod = res % 2;
				map_exp[j] = value1_start + sub + mod;
			}
		}
	}
}
static int oplus_seed_bright_to_alpha(int brightness)
{
        int level = ARRAY_SIZE(brightness_seed_alpha_lut_dc);
        int i = 0;
        int alpha;

        for (i = 0; i < ARRAY_SIZE(brightness_seed_alpha_lut_dc); i++){
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
				seed_table[i].para_list[k]= seed_table[i].para_list[k] * (255 - seed_alpha) / 255;
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
	if (oplus_dc_alpha && level != 0) {
	//	for (i = 0; i < sizeof(lcm_seed_setting)/sizeof(lcm_seed_setting[0]); i++){
	//		cb(dsi, handle, lcm_seed_setting[i].para_list, lcm_seed_setting[i].count);
	//	}
		exit_dc_flag = 1;
		pr_err("exit DC");
	}

	oplus_dc_alpha = 0;
	return level;
}

static int oplus_lcm_dc_post_exitd(void *dsi, dcs_write_gce cb,void *handle)
{
	int i;

	pr_err("debug for lcm %s,oplus_dc_enable = %d\n", __func__,oplus_dc_enable);
	for (i = 0; i < sizeof(lcm_seed_setting)/sizeof(lcm_seed_setting[0]); i++){
		cb(dsi, handle, lcm_seed_setting[i].para_list, lcm_seed_setting[i].count);
	}
	return 0;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	unsigned int mapped_level_exp = 0;
	char bl_tb0[] = {0x51, 0x03, 0xFF};
	char bl_tb1[] = {0x53, 0x20};
	if (!cb)
		return -1;
	if ((level < BRIGHTNESS_HALF) && (level > 0)) {
		mapped_level_exp = map_exp[level];
	} else {
		mapped_level_exp = level;
	}
	mapped_level = oplus_lcm_dc_backlight(dsi,cb,handle, mapped_level_exp, 0);
	if (mapped_level_exp == 1) {
		pr_err("enter aod!!!\n");
		panel_doze_disp_on(dsi,cb,handle);
		return 0;
	} else if (mapped_level_exp == 1023) {
		mapped_level = 2047;
	} else if (level == 2047) {
		mapped_level = 4095;
	} else {
		mapped_level = mapped_level * 2;
	}
	esd_brightness = mapped_level;
	bl_tb0[1] = mapped_level >> 8;
	bl_tb0[2] = mapped_level & 0xFF;

	pr_err("samsung lcm: func:=%s,flag_writ=%d, level=%d, exp_level=%d, mapped_level=%d, aod_mode= %d\n", __func__, flag_writ, level, mapped_level_exp, mapped_level, aod_mode);
	if(aod_mode == 1 && level != 0) {
		return 0;
	}

	/*add for global hbm*/
	if(mapped_level_exp == 1 || mapped_level == 1){
		pr_info("enter aod mode, ignore set backlight to 1\n");
	} else if(mapped_level_exp <= 1023){
	    if(flag_writ == 3||flag_writ == 0){
			bl_tb1[1] = 0x20;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			flag_hbm = 0;
		}
	    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	} else if (mapped_level_exp >= 1024 && mapped_level_exp <= 2047){
	    if(flag_writ == 0||flag_writ == 3){
			bl_tb1[1] = 0xE0;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			flag_hbm = 1;
			}
	    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	}

	return 0;

}


static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0x51, 0x03, 0xff};

	//pr_err("%s esd_backlight = %d\n", __func__, esd_brightness);
	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
	if (!cb)
		return -1;
	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 1;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_err("debug for lcm %s\n", __func__);

	if (ctx->prepared)
		return 0;
	lcm_panel_ldo3_enable(ctx->dev);
	msleep(2);

	#if 0
	if (is_project(20601) || is_project(20660) || is_project(20602)) {
		lcm_status = 1;
		if (is_check_camera_status()) {
			pr_err("lcm_panel enable: the gpiod_set_value is 1, camera_status%d\n", is_check_camera_status());
		} else {
			ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
			gpiod_set_value(ctx->bias_gpio, 1);
			devm_gpiod_put(ctx->dev, ctx->bias_gpio);
			pr_err("lcm_panel enable: the gpiod_set_value is 1, camera_status%d\n", is_check_camera_status());
		}
	} else {
		ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_gpio, 1);
		devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	}
	#else
	ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	#endif

	msleep(5);
	fan53870_ldo1_20601_set_voltage(1100000);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(2);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(10);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_err("debug for samsung lcm %s  ctx->prepared %d \n", __func__,ctx->prepared);

	if (ctx->prepared)
		return 0;

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(10);
    fan53870_ldo1_20601_disable();
    msleep(5);

	#if 0
	if (is_project(20601) || is_project(20660) || is_project(20602)) {
		lcm_status = 0;
		if (is_check_camera_status()) {
			pr_err("lcm_panel disable: the gpiod_set_value is 1, camera_status%d\n", is_check_camera_status());
		} else {
			ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
			gpiod_set_value(ctx->bias_gpio, 0);
			devm_gpiod_put(ctx->dev, ctx->bias_gpio);
			pr_err("lcm_panel disable: the gpiod_set_value is 0, camera_status%d\n", is_check_camera_status());
		}
	} else {
		ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_gpio, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	}
	#else
	//ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	//gpiod_set_value(ctx->bias_gpio, 0);
	//devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	#endif

	msleep(2);
	lcm_panel_ldo3_disable(ctx->dev);
	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	msleep(110);
	return 0;
}

static int lcm_panel_disp_off(void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;

	pr_err("samsung lcm: %s\n", __func__);

	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);
	msleep(10);

	cmd = 0x10;
	cb(dsi, handle, &cmd, 1);
	msleep(120);

	return 0;
}


static void lcm_setbrightness(void *dsi,
			      dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int BL_MSB = 0;
	unsigned int BL_LSB = 0;
	unsigned int hbm_brightness = 0;
	int i = 0;

	pr_err("samsung lcm: %s level is %d\n", __func__, level);

	if (level > BRIGHTNESS_HALF) {
		level = level * 2;
		hbm_brightness = level;
		BL_LSB = hbm_brightness >> 8;
		BL_MSB = hbm_brightness & 0xFF;

		lcm_setbrightness_hbm[0].para_list[1] = BL_LSB;
		lcm_setbrightness_hbm[0].para_list[2] = BL_MSB;

		for (i = 0; i < sizeof(lcm_setbrightness_hbm)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_setbrightness_hbm[i].para_list, lcm_setbrightness_hbm[i].count);
		}
	} else {
		level = level * 2;
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
	int oplus_display_brightness_exp = 0;
	if (!cb)
		return -1;

	if ((oplus_display_brightness < BRIGHTNESS_HALF) && (oplus_display_brightness > 0)) {
                oplus_display_brightness_exp = map_exp[oplus_display_brightness];
        } else {
                oplus_display_brightness_exp = oplus_display_brightness;
        }
	pr_err("oplus_display_brightness= %ld, oplus_display_brightness_exp = %d, hbm_mode=%u\n", oplus_display_brightness, oplus_display_brightness_exp, hbm_mode);

	if(hbm_mode == 1) {
		oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness_exp, 1);
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (hbm_mode == 0) {
		level = oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness_exp, 0);
		lcm_setbrightness(dsi, cb, handle, level);
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
	int oplus_display_brightness_exp = 0;
	if (!cb)
		return -1;
	if (ctx->hbm_en == en)
		goto done;

        if ((oplus_display_brightness < BRIGHTNESS_HALF) && (oplus_display_brightness > 0)) {
                oplus_display_brightness_exp = map_exp[oplus_display_brightness];
        } else {
                oplus_display_brightness_exp = oplus_display_brightness;
        }

	pr_err("samsung lcm: debug for oplus_display_brightness= %ld, oplus_display_brightness_exp = %d, en=%u\n", oplus_display_brightness, oplus_display_brightness_exp, en);
	if(en == 1) {
		if (oplus_display_brightness == 0) {
			lcm_setbrightness(dsi, cb, handle, 0);
			goto done;
		}
		oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness_exp, 1);
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (en == 0) {
		level = oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness_exp, 0);
		lcm_setbrightness(dsi, cb, handle, level);
		printk("[soso] %s : %d ! backlight %d !\n",__func__, en, level);
		flag_hbm = 0;
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
#if 0
static bool panel_no_video_cmd_switch_state(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	ctx->cv_state = true;

    return ctx->cv_state;
}
#endif
static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.doze_enable = panel_doze_enable,
	.doze_disable = panel_doze_disable,
	.doze_post_disp_on = panel_doze_post_disp_on,
	.set_hbm = lcm_set_hbm,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.hbm_get_state = panel_hbm_get_state,
	.hbm_set_state = panel_hbm_set_state,
	.hbm_get_wait_state = panel_hbm_get_wait_state,
	.hbm_set_wait_state = panel_hbm_set_wait_state,
	//.panel_no_cv_switch = panel_no_video_cmd_switch_state,
	.ext_param_set = mtk_panel_ext_param_set,
	.mode_switch = mode_switch,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.lcm_dc_post_exitd = oplus_lcm_dc_post_exitd,
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
	pr_err("debug for lcm_get_modes= %s, en=%u\n", __func__, mode);
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

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;

	dsi_node = of_get_parent(dev->of_node);
	pr_info("[soso]samsung probe\n");
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			pr_info("device_node name:%s\n", remote_node->name);
        }
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm,dev->of_node is %s\n", __func__,dev->of_node);
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
	ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	msleep(2);
	lcm_panel_ldo3_enable(ctx->dev);
	msleep(5);
    fan53870_ldo1_20601_set_voltage(1100000);
	msleep(5);
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
	pr_info("%s samsung lcm+\n", __func__);
	init_global_exp_backlight();
	disp_aal_set_dre_en(1);
	register_device_proc("lcd", "AMS644VA04_MTK04", "samsung1024");
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
	{ .compatible = "oplus20601_samsung_sofe03f_m_1080p_dsi_cmd", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "oplus20601_samsung_sofe03f_m_1080p_dsi_cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);
