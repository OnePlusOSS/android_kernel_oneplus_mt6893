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
#include <linux/of_graph.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_panel_ext.h"
#include "../mediatek/mtk_log.h"
#include "../mediatek/mtk_drm_graphics_base.h"
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/oplus20171_BOE_NT37701_data_hw_roundedpattern.h"
#endif

extern int is_fan53870_pmic(void);
extern int pmic_ldo_2_set_voltage_uv(unsigned int set_uV);
extern int pmic_ldo_2_set_disable(void);
extern void __attribute__((weak)) lcdinfo_notify(unsigned long val, void *v) { return; };

static int esd_brightness;
static bool aod_state = false;
extern unsigned long oplus_display_brightness;
extern unsigned long oplus_max_normal_brightness;
//static int aod_finger_unlock_flag = 0;
/*#ifdef OPLUS_FEATURE_TP_BASIC*/
__attribute__((weak)) void lcd_tp_refresh_switch(int fps)
{
    return;
}
/*#endif OPLUS_FEATURE_TP_BASIC*/

#define LCM_DSI_CMD_MODE 1
#define TRUE 1
#define FALSE 0

#define REGFLAG_CMD       0xFFFA
#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD

#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   3515
#define SILKY_MAX_NORMAL_BRIGHTNESS   8191
#define LCM_BRIGHTNESS_TYPE 2

/*#ifdef OPLUS_BUG_STABILITY*/
#include "../oplus/oplus_display_panel_power.h"
extern int oplus_export_drm_panel(struct drm_panel *panel_node);
extern int fan53870_ldo1_regmap_read(void);
extern int dsi_panel_parse_panel_power_cfg(struct panel_voltage_bak *panel_vol);
static PANEL_VOLTAGE_BAK panel_vol_bak[PANEL_VOLTAGE_ID_MAX] = {
						{0, 1800000, 1800000, 1800000, "vddi"},
						{1, 1104000, 1200000, 1232000, "vddr"},
						{2, 0, 1, 2, ""}};
/*#endif*/ /*OPLUS_BUG_STABILITY*/

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_gpio;
	struct gpio_desc *ldo_gpio;

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
	{REGFLAG_CMD,6,{0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_CMD,2,{0x6F,0x0D}},
	{REGFLAG_CMD,2,{0xB5,0x4F}},
	{REGFLAG_CMD,2,{0x65,0x00}},
	{REGFLAG_CMD,1,{0x38}},
	{REGFLAG_CMD,1,{0x2C}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_seed_setting[] = {
	{REGFLAG_CMD,6,{0xF0, 0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_CMD,2,{0xB2,0x80}},

	{REGFLAG_CMD,37,{0xB3,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0D,0xBB,0x0D,0xBB,0x0F,0xFF}},

	{REGFLAG_CMD,2,{0x6F,0x2C}},
	{REGFLAG_CMD,39,{0xB3,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30}},
	{REGFLAG_CMD,39,{0xB4,0x00,0x48,0x00,0x48,0x00,0x48,0x00,0x48,0x00,0x48,0x00,0x48,0x00,0x48,0x00,0x48,0x00,0x48,0x00,0x48,0x00,0x48,0x00,0x48,0x00,0x48,0x00,0x48,0x00,0x48,0x00,0x48,0x00,0x48,0x00,0x48,0x00,0x48}},

	{REGFLAG_CMD,2,{0x6F,0x2C}},
	{REGFLAG_CMD,21,{0xB2,0x00,0x00,0x00,0x17,0x00,0x2E,0x00,0x45,0x00,0x5D,0x00,0x74,0x00,0x8B,0x00,0xA3,0x00,0xBA,0x00,0xD1}},
	{REGFLAG_CMD,2,{0x6F,0xC0}},
	{REGFLAG_CMD,21,{0xB2,0x00,0xE9,0x01,0x8D,0x02,0x2E,0x02,0xCE,0x03,0x6F,0x04,0x10,0x04,0xB1,0x05,0x51,0x05,0xF2,0x06,0x93}},
	{REGFLAG_CMD,2,{0x6F,0xD4}},
	{REGFLAG_CMD,21,{0xB2,0x07,0x34,0x07,0xD4,0x08,0x75,0x09,0x16,0x09,0xB7,0x0A,0x58,0x0A,0xF8,0x0B,0x99,0x0C,0x3A,0x0C,0xDB}},
	{REGFLAG_CMD,2,{0x6F,0xE8}},
	{REGFLAG_CMD,11,{0xB2,0x0D,0x7B,0x0E,0x1C,0x0E,0xBD,0x0F,0x5E,0x0F,0xFF}},
	{REGFLAG_CMD,2,{0x6F,0x52}},
	{REGFLAG_CMD,21,{0xB3,0x00,0x14,0x00,0x28,0x00,0x3C,0x00,0x50,0x00,0x64,0x00,0x78,0x00,0x8C,0x00,0xA0,0x00,0xB4,0x00,0xC8}},
	{REGFLAG_CMD,2,{0x6F,0x66}},
	{REGFLAG_CMD,21,{0xB3,0x01,0x55,0x01,0xDF,0x02,0x69,0x02,0xF3,0x03,0x7D,0x04,0x07,0x04,0x91,0x05,0x1B,0x05,0xA5,0x06,0x2F}},
	{REGFLAG_CMD,2,{0x6F,0x7A}},
	{REGFLAG_CMD,21,{0xB3,0x06,0xB9,0x07,0x43,0x07,0xCD,0x08,0x57,0x08,0xE1,0x09,0x6B,0x09,0xF5,0x0A,0x7F,0x0B,0x09,0x0B,0x93}},
	{REGFLAG_CMD,2,{0x6F,0x8E}},
	{REGFLAG_CMD,9,{0xB3,0x0C,0x1D,0x0C,0xA7,0x0D,0x31,0x0D,0xBB}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_seed_exit[] = {
	{REGFLAG_CMD,6,{0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_CMD,16,{0xB2,0x11,0x00,0x20,0x20,0x00,0x08,0x08,0x00,0x00,0x40,0x30,0x00,0x00,0x00,0x00}},
	{REGFLAG_CMD,2,{0x6F,0x0F}},
	{REGFLAG_CMD,12,{0xB2,0x50,0x20,0x25,0x7B,0x25,0x7B,0x2F,0xFF,0x00,0x00,0x00}},

	{REGFLAG_CMD,37,{0xB3,0x00,0x09,0x00,0x41,0x00,0x42,0x00,0xA4,0x00,0xA5,0x01,0x15,0x01,0x16,0x02,0x0B,0x02,0x0C,0x03,0x48,0x03,0x49,0x05,0x7B,0x05,0x7C,0x09,0x9B,0x09,0x9C,0x0D,0xBB,0x0D,0xBB,0x0F,0xFF}},
	{REGFLAG_CMD,2,{0x6F,0x2C}},
	{REGFLAG_CMD,39,{0xB3,0x09,0x54,0x09,0x54,0x08,0xF6,0x08,0xF2,0x08,0x50,0x08,0x4C,0x07,0x92,0x07,0x8E,0x05,0xF6,0x05,0xE4,0x03,0xE0,0x03,0xF0,0x00,0x30,0x00,0x50,0x00,0x30,0x00,0x50,0x00,0x30,0x00,0x30,0x00,0x30}},
	{REGFLAG_CMD,2,{0x6F,0x24}},
	{REGFLAG_CMD,9,{0xB3,0x00,0x03,0x07,0x5F,0x07,0x5F,0x0F,0xFE}},
	{REGFLAG_CMD,39,{0xB4,0x0D,0xFE,0x0D,0xFE,0x0D,0x71,0x0D,0x6B,0x0C,0x78,0x0C,0x72,0x0B,0x5B,0x0B,0x55,0x08,0xF1,0x08,0xD6,0x05,0xD0,0x05,0xE8,0x00,0x48,0x00,0x78,0x00,0x48,0x00,0x78,0x00,0x48,0x00,0x48,0x00,0x48}},
	{REGFLAG_CMD,2,{0x6F,0x26}},
	{REGFLAG_CMD,11,{0xB4,0x04,0xA2,0x04,0xA2,0x04,0xA2,0x04,0xA2,0x04,0xA2}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_high_mode[] = {
	/* aod 50nit*/
	{REGFLAG_CMD,6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_CMD,2, {0x6F,0x26}},
	{REGFLAG_CMD,11,{0xB4,0x04,0xA2,0x04,0xA2,0x04,0xA2,0x04,0xA2,0x04,0xA2}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_low_mode[] = {
	/* aod 10nit*/
	{REGFLAG_CMD,6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_CMD,2, {0x6F,0x26}},
	{REGFLAG_CMD,11, {0xB4,0x08,0x8A,0x08,0x8A,0x08,0x8A,0x08,0x8A,0x08,0x8A}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setbrightness_normal[] = {
	{REGFLAG_CMD,3, {0x51,0x00,0x00}},
	{REGFLAG_CMD,6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_CMD,2, {0xB2,0x11}},
};

static struct LCM_setting_table lcm_finger_HBM_on_setting[] = {
	{REGFLAG_CMD,3, {0x51,0x0F,0xFF}},
	{REGFLAG_CMD,6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_CMD,2, {0xB2,0x01}},
};

static struct LCM_setting_table lcm_finger_HBM_DC_ADD[] = {
	{REGFLAG_CMD,6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_CMD,2, {0xB2,0x80}},
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

static void lcm_panel_init(struct lcm *ctx)
{
    pr_err("debug for boe lcm 3_ban%s\n", __func__);
    /* Display On Setting */
    lcm_dcs_write_seq_static(ctx, 0xFF,0xAA,0x55,0xA5,0x81);
    lcm_dcs_write_seq_static(ctx,0x6F,0x0F);    /* OSC=101.6Mhz */
    lcm_dcs_write_seq_static(ctx,0xFD,0x01,0x46);

    //OSC Strong
    lcm_dcs_write_seq_static(ctx,0x6F,0x04);
    lcm_dcs_write_seq_static(ctx,0xFD,0x5F);
    lcm_dcs_write_seq_static(ctx,0x6F,0x1A);
    lcm_dcs_write_seq_static(ctx,0xFD,0x5F);

    lcm_dcs_write_seq_static(ctx,0xFF,0xAA,0x55,0xA5,0x81);
    lcm_dcs_write_seq_static(ctx,0x6F,0x21);
    lcm_dcs_write_seq_static(ctx,0xFA,0x02,0x02);
    lcm_dcs_write_seq_static(ctx,0x6F,0x14);
    lcm_dcs_write_seq_static(ctx,0xF3,0x00);

    lcm_dcs_write_seq_static(ctx,0xF0,0x55,0xAA,0x52,0x08,0x01);
    lcm_dcs_write_seq_static(ctx, 0xC3,0x8D,0x01,0x96,0xD0,0x22,0x02,0x00);
    /* slove ELVDD drop*/
    lcm_dcs_write_seq_static(ctx,0x6F,0x01);
    lcm_dcs_write_seq_static(ctx,0xD2,0x01);
    lcm_dcs_write_seq_static(ctx,0x6F,0x03);
    lcm_dcs_write_seq_static(ctx,0xD2,0x33);
    lcm_dcs_write_seq_static(ctx,0x6F,0x06);
    lcm_dcs_write_seq_static(ctx,0xD2,0x00);
    lcm_dcs_write_seq_static(ctx,0x6F,0x0F);
    lcm_dcs_write_seq_static(ctx,0xD2,0x00);
    lcm_dcs_write_seq_static(ctx,0xC7,0x01);

    lcm_dcs_write_seq_static(ctx,0x35);
    lcm_dcs_write_seq_static(ctx,0x53,0x20);
    lcm_dcs_write_seq_static(ctx,0x51,0x00,0x00,0x0F,0xFE);
    lcm_dcs_write_seq_static(ctx,0x2A,0x00,0x00,0x04,0x37);
    lcm_dcs_write_seq_static(ctx,0x2B,0x00,0x00,0x09,0x5F);
    /*VESA 8bit3x slice 20 for BOE*/
    lcm_dcs_write_seq_static(ctx,0x91,0x89,0x28,0x00,0x14,0xC2,0x00,0x03,0x1C,0x02,0x8C,0x00,0x0F,0x05,0x0E,0x02,0x8B,0x10,0xF0);
    /*PPS Table1*/
    lcm_dcs_write_seq_static(ctx,0x03,0x01);
    lcm_dcs_write_seq_static(ctx,0x90,0x01);
    lcm_dcs_write_seq_static(ctx,0x2C);
    /* ELVSS swire offset */
    lcm_dcs_write_seq_static(ctx,0x82,0xAE);
    /* Normal Display */
    lcm_dcs_write_seq_static(ctx,0x2F,0x01);//01 60 02 90
    lcm_dcs_write_seq_static(ctx,0xF0,0x55,0xAA,0x52,0x08,0x01);
    lcm_dcs_write_seq_static(ctx,0xCD,0x05,0x81);
    lcm_dcs_write_seq_static(ctx,0x6F,0x10);
    lcm_dcs_write_seq_static(ctx,0xD8,0x14);
	lcm_dcs_write_seq_static(ctx,0x6F,0x12);
	lcm_dcs_write_seq_static(ctx,0xBC,0x01);
    lcm_dcs_write_seq_static(ctx,0x11);
    msleep(64);
    lcm_dcs_write_seq_static(ctx,0x29);
	//msleep(20);

}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_err("debug for lcm %s, ctx->enabled=%d\n", __func__, ctx->enabled);

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

	pr_err("debug for lcm %s, ctx->prepared=%d\n", __func__, ctx->prepared);

	if (!ctx->prepared)
		return 0;

	if (aod_state) {
		lcm_dcs_write_seq_static(ctx,0xF0,0x55,0xAA,0x52,0x08,0x01);
		lcm_dcs_write_seq_static(ctx, 0xCE, 0x0C);
	}

	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(10);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(180);

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
	aod_state = false;

#if 0
	ctx->bias_gpio =
	devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);
#endif
	msleep(10);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(5);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(5);
	gpiod_set_value(ctx->reset_gpio, 1);
	//msleep(10);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(15);

	if(power_mode == 2)
	{
		DDPINFO("%s + lcm_panel_init, boe resume status\n", __func__);
		lcm_panel_init(ctx);
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
	pr_err("debug for lcm %s, ctx->enabled=%d\n", __func__, ctx->enabled);

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
	.clock = 170434,
	.hdisplay = 1080,
	.hsync_start = 1080 + 52,
	.hsync_end = 1080 + 52 + 14,
	.htotal = 1080 + 52 + 14 + 22,
	.vdisplay = 2400,
	.vsync_start = 2400 + 21,
	.vsync_end = 2400 + 21 + 2,
	.vtotal = 2400 + 21 + 2 + 9,
	.vrefresh = 60,
};

static const struct drm_display_mode performance_mode = {
	.clock = 255651,
	.hdisplay = 1080,
	.hsync_start = 1080 + 52,
	.hsync_end = 1080 + 52 + 14,
	.htotal = 1080 + 52 + 14 + 22,
	.vdisplay = 2400,
	.vsync_start = 2400 + 21,
	.vsync_end = 2400 + 21 + 2,
	.vtotal = 2400 + 21 + 2 + 9,
	.vrefresh = 90,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.pll_clk = 209,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xFA, .count = 1, .para_list[0] = 0x00, .mask_list[0] = 0xFF,
	},
	/*
	.lcm_esd_check_table[2] = {
		.cmd = 0xEA, .count = 1, .para_list[0] = 0x00, .mask_list[0] = 0x80,
	},
	*/
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
        .enable = 1,
        .ver = 17,
        .slice_mode = 0,
        .rgb_swap = 0,
        .dsc_cfg = 34,
        .rct_on = 1,
        .bit_per_channel = 8,
        .dsc_line_buf_depth = 9,
        .bp_enable = 1,
        .bit_per_pixel = 128,
        .pic_height = 2400,
        .pic_width = 1080,
        .slice_height = 20,
        .slice_width = 1080,
        .chunk_size = 1080,
        .xmit_delay = 512,
        .dec_delay = 796,
        .scale_value = 32,
        .increment_interval = 652,
        .decrement_interval = 15,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 1294,
        .slice_bpg_offset = 651,
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
	.data_rate = 418,
	.hbm_en_time = 2,
	.hbm_dis_time = 1,
	.oplus_hbm_on_sync_with_flush = 1,
	.oplus_hbm_off_sync_with_flush = 1,
	.oplus_wait_te_num = 3,
    .dyn_fps = {
        .switch_en = 0, .vact_timing_fps = 60,
    },
    .vendor = "21015_NT37800",
    .manufacture = "boe_nt37800_2048",
    .oplus_serial_para0 = 0xD8,
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 300,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
        .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
        .cmd = 0xFA, .count = 1, .para_list[0] = 0x00, .mask_list[0] = 0xFF,
	},
	/*
	.lcm_esd_check_table[2] = {
        .cmd = 0xEA, .count = 1, .para_list[0] = 0x00, .mask_list[0] = 0x80,
	},
	*/
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
    .dsc_params = {
        .enable = 1,
        .ver = 17,
        .slice_mode = 0,
        .rgb_swap = 0,
        .dsc_cfg = 34,
        .rct_on = 1,
        .bit_per_channel = 8,
        .dsc_line_buf_depth = 9,
        .bp_enable = 1,
        .bit_per_pixel = 128,
        .pic_height = 2400,
        .pic_width = 1080,
        .slice_height = 20,
        .slice_width = 1080,
        .chunk_size = 1080,
        .xmit_delay = 512,
        .dec_delay = 796,
        .scale_value = 32,
        .increment_interval = 652,
        .decrement_interval = 15,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 1294,
        .slice_bpg_offset = 651,
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
    .hbm_en_time = 2,
	.hbm_dis_time = 1,
	.oplus_hbm_on_sync_with_flush = 1,
	.oplus_hbm_off_sync_with_flush = 1,
	.oplus_wait_te_num = 3,
	.data_rate = 600,
    .dyn_fps = {
        .switch_en = 0, .vact_timing_fps = 90,
    },
    .vendor = "21015_NT37800",
    .manufacture = "boe_nt37800_2048",
    .oplus_serial_para0 = 0xD8,
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

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

	if (!m) {
		pr_err("%s, get mode failed\n", __func__);
		return 1;
	}

	if (!ext) {
		pr_err("%s, find_panel_ext failed\n", __func__);
		return 1;
	}
	if (mode == 0)
		ext->params = &ext_params;
	else if (mode == 1)
		ext->params = &ext_params_90hz;

	if (m->vrefresh == 60)
		ext->params = &ext_params;
	else if (m->vrefresh == 90)
		ext->params = &ext_params_90hz;
	else
		ret = 1;

	return ret;
}

static void mode_switch_60_to_90(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		/* display on switch to 90hz */
		lcm_dcs_write_seq_static(ctx, 0xF0,0x55,0xAA,0x52,0x08,0x01);
		//lcm_dcs_write_seq_static(ctx, 0xC3,0x00);
		lcm_dcs_write_seq_static(ctx, 0x2C);
	} else if (stage == AFTER_DSI_POWERON) {
		lcm_dcs_write_seq_static(ctx, 0xF0,0x55,0xAA,0x52,0x08,0x01);
		lcm_dcs_write_seq_static(ctx, 0x2F, 0x02);
		usleep_range(20*1000, 21*1000);
		lcm_dcs_write_seq_static(ctx, 0xC3,0x92,0x01,0x88,0xD0,0x22,0x02,0x00);
		lcm_dcs_write_seq_static(ctx, 0x2C);
	//} else if (stage == AFTER_CHANGE_DATARATE) {
		//usleep_range(20*1000, 21*1000);
		//lcm_dcs_write_seq_static(ctx, 0xF0,0x55,0xAA,0x52,0x08,0x01);
		//lcm_dcs_write_seq_static(ctx, 0xC3,0x9A,0x01,0x9F,0xD0,0x20,0x81,0x00);
    }
}

static void mode_switch_90_to_60(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		/* display on switch to 60hz */
		lcm_dcs_write_seq_static(ctx, 0xF0,0x55,0xAA,0x52,0x08,0x01);
		//lcm_dcs_write_seq_static(ctx, 0xC3,0x00);
		lcm_dcs_write_seq_static(ctx, 0x2C);
		lcm_dcs_write_seq_static(ctx, 0x2F, 0x01); /* 60 FPS */
		usleep_range(26*1000, 27*1000);
	} else if (stage == AFTER_DSI_POWERON) {
		lcm_dcs_write_seq_static(ctx, 0xF0,0x55,0xAA,0x52,0x08,0x01);
		lcm_dcs_write_seq_static(ctx, 0xC3,0x8D,0x01,0x96,0xD0,0x22,0x02,0x00);
		lcm_dcs_write_seq_static(ctx, 0x2C);
	//} else if (stage == AFTER_CHANGE_DATARATE) {
		//usleep_range(20*1000, 21*1000);
		//lcm_dcs_write_seq_static(ctx, 0xF0,0x55,0xAA,0x52,0x08,0x01);
		//lcm_dcs_write_seq_static(ctx, 0xC3,0x8D,0x01,0x9C,0xD0,0x20,0x81,0x00);
    }
}

static int mode_switch(struct drm_panel *panel, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(panel, dst_mode);

	if (!m) {
		pr_err("%s, get mode failed\n", __func__);
		return 1;
	}

	if (cur_mode == dst_mode)
		return ret;
/*#ifdef OPLUS_FEATURE_TP_BASIC*/
	if(stage == BEFORE_DSI_POWERDOWN) {
		pr_info("[TP] ready charge to %d hz", m->vrefresh);
		lcd_tp_refresh_switch(m->vrefresh);
	}
/*#endif OPLUS_FEATURE_TP_BASIC*/
	pr_info("mode_switch m->vrefresh is %d ", m->vrefresh);
	if (m->vrefresh == 60) { /*switch to 60 */
		mode_switch_90_to_60(panel, stage);
	} else if (m->vrefresh == 90) { /*switch to 90 */
		mode_switch_60_to_90(panel, stage);
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

extern bool oplus_fp_notify_down_delay;
static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	//struct lcm *ctx = panel_to_lcm(panel);
	unsigned int i=0;
	//pr_err("debug for lcm %s, oplus_fp_notify_down_delay=%d\n", __func__, oplus_fp_notify_down_delay);

	/*if (oplus_fp_notify_down_delay)
		aod_finger_unlock_flag = 1;*/
	usleep_range(35*1000, 35*1000 + 100);
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

	/*if (aod_finger_unlock_flag == 1) {
		struct lcm *ctx = panel_to_lcm(panel);
		DDPINFO("finger unlock in aod\n");
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
		ctx->hbm_en = true;
		ctx->hbm_wait = false;
		aod_finger_unlock_flag = 0;
	}*/

	aod_state = false;

	return 0;
}

static struct LCM_setting_table lcm_normal_to_aod_sam[] = {
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x01}},
    {REGFLAG_CMD, 19, {0xD2,0x00,0x01,0x22,0x33,0x25,0x00,0x00,0x44,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x01}},
    {REGFLAG_CMD, 2, {0xC3,0x00}},
    {REGFLAG_CMD, 2, {0xC7,0x01}},

    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x81}},
    {REGFLAG_CMD, 2, {0x6F,0x0F}},
    {REGFLAG_CMD, 3, {0xFD,0x01,0x46}},

    //OSC Strong
    {REGFLAG_CMD, 2, {0x6F,0x04}},
    {REGFLAG_CMD, 2, {0xFD,0x5F}},
    {REGFLAG_CMD, 2, {0x6F,0x1A}},
    {REGFLAG_CMD, 2, {0xFD,0x5F}},

    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x81}},
    {REGFLAG_CMD, 2, {0x6F,0x21}},
    {REGFLAG_CMD, 3, {0xFA,0x02,0x02}},
    {REGFLAG_CMD, 2, {0x6F,0x14}},
    {REGFLAG_CMD, 2, {0xF3,0x00}},

    {REGFLAG_CMD, 1, {0x35}},
    {REGFLAG_CMD, 2, {0x53,0x20}},

    {REGFLAG_CMD, 2, {0x6F,0x02}},
    {REGFLAG_CMD, 3, {0x51,0x0F,0xFE}},

    {REGFLAG_CMD, 5, {0x2A,0x00,0x00,0x04,0x37}},
    {REGFLAG_CMD, 5, {0x2B,0x00,0x00,0x09,0x5F}},

    {REGFLAG_CMD, 19, {0x91,0x89,0x28,0x00,0x14,0xC2,0x00,0x03,0x1C,0x02,0x8C,0x00,0x0F,0x05,0x0E,0x02,0x8B,0x10,0xF0}},
    {REGFLAG_CMD, 2, {0x03,0x01}},
    {REGFLAG_CMD, 2, {0x90,0x01}},
    {REGFLAG_CMD, 1, {0x2C}},
    {REGFLAG_CMD, 2, {0x82,0xAE}},
    {REGFLAG_CMD, 2, {0x2F,0x01}},
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x01}},
    {REGFLAG_CMD, 3, {0xCD,0x05,0x81}},
    {REGFLAG_CMD, 2, {0x6F,0x10}},
    {REGFLAG_CMD, 2, {0xD8,0x14}},
    {REGFLAG_CMD, 1, {0x11}},
    {REGFLAG_DELAY,120,{}},
    {REGFLAG_CMD, 1, {0x29}},
    //{REGFLAG_DELAY,20,{}},

    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
    {REGFLAG_CMD, 2, {0xC0,0x46}},
    {REGFLAG_CMD, 2, {0x6F,0x01}},
    {REGFLAG_CMD, 2, {0xC0,0xB3}},
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x2E}},
    {REGFLAG_CMD, 2, {0xFB,0xD1}},
    {REGFLAG_CMD, 2, {0x6F,0x61}},
    {REGFLAG_CMD, 2, {0xF3,0x80}},

    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x01}},
    {REGFLAG_CMD, 2, {0x6F,0x02}},
    {REGFLAG_CMD, 2, {0xD2,0x24}},

    {REGFLAG_CMD, 2, {0x6F,0x06}},
    {REGFLAG_CMD, 2, {0xD2,0x06}},

    //{REGFLAG_CMD, 1, {0x28}},
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
    {REGFLAG_CMD, 2, {0x6F,0x0D}},
    {REGFLAG_CMD, 2, {0xB5,0x50}},
    {REGFLAG_CMD, 2, {0x2F,0x01}},
    {REGFLAG_CMD, 1, {0x39}},
    {REGFLAG_CMD, 1, {0x2C}},
    {REGFLAG_DELAY,20,{}},
    {REGFLAG_CMD, 2, {0x65,0x01}},
    {REGFLAG_CMD, 1, {0x2C}},
    //{REGFLAG_DELAY,20,{}},

    /* Display on */
    //{REGFLAG_CMD, 1, {0x29}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i=0;
	pr_err("debug for lcm %s\n", __func__);
	aod_state = true;

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

	return 0;
}

static int panel_doze_enable_start(void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;
	pr_err("debug for lcm %s\n", __func__);

	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);
	cmd = 0x10;
	cb(dsi, handle, &cmd, 1);

	return 0;
}

static int panel_doze_enable_end(void *dsi, dcs_write_gce cb, void *handle)
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
static int panel_doze_post_disp_on(void *dsi, dcs_write_gce cb, void *handle)
{

	int cmd = 0;

	pr_err("debug for boe lcm %s\n", __func__);

	cmd = 0x29;
	cb(dsi, handle, &cmd, 1);
	//msleep(2);

	return 0;
}


static int panel_doze_post_disp_off(void *dsi, dcs_write_gce cb, void *handle)
{

	int cmd = 0;

	pr_err("debug for boe lcm %s\n", __func__);

	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);

	return 0;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	unsigned int notify_level = 0;
	char bl_tb0[] = {0x51, 0x03, 0xFF};
	//char bl_tb1[] = {0x53, 0x20};
	if (!cb)
		return -1;

	//mapped_level = oplus_lcm_dc_backlight(dsi,cb,handle, level, 0);
	mapped_level = level;
	bl_tb0[1] = mapped_level >> 8;
	bl_tb0[2] = mapped_level & 0xFF;

	esd_brightness = mapped_level;

	pr_err("boe lcm: debug for display panel backlight value,func:=%s,level :=%d, mapped_level := %d\n", __func__, level, mapped_level);
	if (mapped_level > 1) {
		notify_level = mapped_level * 2047 / MAX_NORMAL_BRIGHTNESS;
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &notify_level);
	}

	if (level == 1 || mapped_level == 1) {
		pr_info("enter aod mode, ignore set backlight to 1\n");
	} else {
		/*add for global hbm*/
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

	if (ctx->prepared)
		return 0;

	pr_err("debug for boe lcm %s\n", __func__);

	ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	usleep_range(5000, 5100);
	ctx->ldo_gpio = devm_gpiod_get(ctx->dev, "ldo", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->ldo_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->ldo_gpio);
	usleep_range(10 * 1000, 11 * 1000);
	lcm_panel_ldo3_enable(ctx->dev);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	usleep_range(5000, 5100);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_err("debug for boe lcm %s  ctx->prepared %d \n", __func__,ctx->prepared);

	usleep_range(5000, 5100);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(5);
	lcm_panel_ldo3_disable(ctx->dev);
	msleep(5);
	ctx->ldo_gpio = devm_gpiod_get(ctx->dev, "ldo", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->ldo_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->ldo_gpio);
	msleep(10);
	//delete for not reset fan53870 which influence camera
	/*ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_gpio, 0);
	msleep(5);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);*/

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	//msleep(110);
	return 0;
}

static int lcm_panel_disp_off(void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;

	pr_err("boe lcm: %s\n", __func__);

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
	int i = 0;

	pr_err("boe lcm: %s level is %d\n", __func__, level);

	BL_LSB = level >> 8;
	BL_MSB = level & 0xFF;

	lcm_setbrightness_normal[0].para_list[1] = BL_LSB;
	lcm_setbrightness_normal[0].para_list[2] = BL_MSB;

	for (i = 0; i < sizeof(lcm_setbrightness_normal)/sizeof(struct LCM_setting_table); i++){
		cb(dsi, handle, lcm_setbrightness_normal[i].para_list, lcm_setbrightness_normal[i].count);
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
		//level = oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 0);
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness); //level
		printk("[soso] %s : %d ! backlight %d !\n",__func__, hbm_mode, oplus_display_brightness);
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

	pr_err("boe lcm: debug for oplus_display_brightness= %ld, en=%u\n", oplus_display_brightness, en);

	if(en == 1) {
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (en == 0) {
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);
		printk("[soso] %s : %d ! backlight %d !\n",__func__, en, oplus_display_brightness);
	}
	lcdinfo_notify(1, &en);
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

static int lcm_get_aod_state()
{
	return aod_state;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

	pr_err("debug for lcm %s\n", __func__);
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

static char esd_check_pre_cmd0[] = {0xFF, 0xAA, 0x55, 0xA5, 0x81};
static char esd_check_pre_cmd1[] = {0x6F, 0x0F};

static int lcm_esd_check_precondition(void *dsi, dcs_write_gce cb, void *handle)
{
       cb(dsi, handle, esd_check_pre_cmd0, ARRAY_SIZE(esd_check_pre_cmd0));
       cb(dsi, handle, esd_check_pre_cmd1, ARRAY_SIZE(esd_check_pre_cmd1));
       return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.doze_enable = panel_doze_enable,
	.doze_disable = panel_doze_disable,
	//.doze_post_disp_on = panel_doze_post_disp_on,
	.set_hbm = lcm_set_hbm,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	//.panel_disp_off = lcm_panel_disp_off,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.hbm_get_state = panel_hbm_get_state,
	.hbm_set_state = panel_hbm_set_state,
	.hbm_get_wait_state = panel_hbm_get_wait_state,
	.hbm_set_wait_state = panel_hbm_set_wait_state,
	//.panel_no_cv_switch = panel_no_video_cmd_switch_state,
	.ext_param_set = mtk_panel_ext_param_set,
	.mode_switch = mode_switch,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.esd_check_precondition = lcm_esd_check_precondition,
	//.oplus_get_aod_state = lcm_get_aod_state,
	.set_aod_light_mode = panel_set_aod_light_mode,
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

	panel->connector->display_info.width_mm = 69;
	panel->connector->display_info.height_mm = 151;

	return 1;
}


/*#ifdef OPLUS_BUG_STABILITY*/
static int lcm_set_power(unsigned int voltage_id, unsigned int voltage_value)
{
	int ret = 0;
	pr_err("%s voltage_id = %u, voltage_value = %u\n",
		__func__, voltage_id, voltage_value);

	switch(voltage_id) {
	/*0 represent the vddi*/
	case 0: {
			printk("%s vddi not need to update\n", __func__);
			ret = -1;
		}
		break;
	/*case 1 represent vddr*/
	case 1: {
			ret = pmic_ldo_2_set_voltage_uv(voltage_value);
			if (ret < 0) {
				printk("%s error ret = %d", __func__, ret);
			}
		}
		break;
	/*case 2 represent vgh*/
	case 2:
		break;
	default:
		printk("%s error voltage id %d", __func__, voltage_id);
		break;
	}

	return ret;
}

/*voltage_id represent the PANEL_VOLTAGE_ENUM
  return the voltage_value of the voltage_id*/
static int lcm_update_power(uint32_t voltage_id)
{
	int ret = 0;
	pr_err("%s voltage_id = %d\n", __func__, voltage_id);

	switch(voltage_id) {
	/*0 represent the vddi*/
	case 0: {
			printk("%s vddi not need to update\n", __func__);
			ret = -1;
		}
		break;
	/*case 1 represent vddr*/
	case 1: {
			ret = fan53870_ldo1_regmap_read();
			if (ret < 0) {
				printk("%s error ret = %d", __func__, ret);
			}
		}
		break;
	/*case 2 represent vgh*/
	case 2:
		break;
	default:
		printk("%s error voltage id %d", __func__, voltage_id);
		break;
	}
	return ret;
}
/*#endif*/ /*OPLUS_BUG_STABILITY*/


static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
/*#ifdef OPLUS_BUG_STABILITY*/
	.oplus_set_power =  lcm_set_power,
	.oplus_update_power_value = lcm_update_power,
/*#endif*/ /*OPLUS_BUG_STABILITY*/
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;

	pr_info("[soso]boe probe\n");
	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			 remote_node = of_graph_get_remote_port_parent(endpoint);
			 pr_info("device_node name:%s\n", remote_node->name);
                   }
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm. of_node is %s\n", __func__, dev->of_node->name);
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
	ctx->ldo_gpio = devm_gpiod_get(ctx->dev, "ldo", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->ldo_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->ldo_gpio);
	msleep(2);
	lcm_panel_ldo3_enable(ctx->dev);
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

	/*#ifdef OPLUS_BUG_STABILITY*/
	oplus_export_drm_panel(&ctx->panel);
	dsi_panel_parse_panel_power_cfg(panel_vol_bak);
	/*#endif*/ /*OPLUS_BUG_STABILITY*/

	register_device_proc("lcd", "21015_NT37800", "boe_nt37800_2048");
	ctx->hbm_en = false;
	oplus_max_normal_brightness = SILKY_MAX_NORMAL_BRIGHTNESS;
	pr_info("%s zjb+\n", __func__);

	//register_device_proc("lcd", "AMS644VA04_MTK04", "samsung1024");
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
	{ .compatible = "oplus21015_boe_nt37701_1080p_dsi_cmd", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "oplus21015_boe_nt37701_1080p_dsi_cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);
