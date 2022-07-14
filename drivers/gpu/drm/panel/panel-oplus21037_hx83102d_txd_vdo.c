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
extern int tp_gesture_enable_flag(void);
extern unsigned int is_project(int project);
/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int esd_brightness = 1023;
extern void lcd_queue_load_tp_fw(void);
extern unsigned long esd_flag;
#ifdef OPLUS_BUG_STABILITY
static int cabc_lastlevel = 0;
static int last_brightness = 0;
static void cabc_switch(void *dsi, dcs_write_gce cb,
                void *handle, unsigned int cabc_mode);
#endif
static int backlight_gamma = 1;
extern unsigned long oplus_max_normal_brightness;
extern void disp_aal_set_dre_en(int enable);
extern int _20015_lcm_i2c_write_bytes(unsigned char addr, unsigned char value);
/***********************************/

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
	pr_info("SYQ %s+\n", __func__);
	lcm_dcs_write_seq_static(ctx, 0xB9 ,0x83 ,0x10 ,0x2D);
	lcm_dcs_write_seq_static(ctx, 0xC0 ,0x30 ,0x30 ,0x00 ,0x00 ,0x19 ,0x42 ,0x00 ,0x08 ,0x00 ,0x1A ,0x1B);
	lcm_dcs_write_seq_static(ctx, 0xB1 ,0x22 ,0x00 ,0x35 ,0x33 ,0x22 ,0x34 ,0x2F ,0x57 ,0x0D ,0x0D ,0x0D);
	lcm_dcs_write_seq_static(ctx, 0xB2 ,0x00 ,0x00 ,0x06 ,0x40 ,0x00 ,0x0E ,0xEE ,0x34 ,0x00 ,0x00 ,0x00 ,0x01 ,0x85 ,0xE0 ,0x57);
	lcm_dcs_write_seq_static(ctx, 0xB4 ,0x58 ,0x4F ,0x58 ,0x4F ,0x58 ,0x52 ,0x58 ,0x4F ,0x05 ,0xFF ,0x03 ,0x03 ,0x00 ,0xFF);
	lcm_dcs_write_seq_static(ctx, 0xCC ,0x02);
	lcm_dcs_write_seq_static(ctx, 0xD3 ,0x06 ,0x02 ,0x01 ,0x01 ,0x00 ,0x08 ,0x00 ,0x47 ,0x00 ,0x43 ,0x33 ,0x0B ,0x0B ,0x00 ,0x00 ,0x32 ,0x10 ,0x07 ,0x00 ,0x07 ,0x54 ,0x15 ,0xA8 ,0x05 ,0xA8 ,0x32 ,0x10 ,0x08 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x16 ,0x00 ,0x00 ,0x00 ,0x16);
	lcm_dcs_write_seq_static(ctx, 0xD5 ,0x18 ,0x18 ,0x18 ,0x18 ,0x39 ,0x39 ,0x18 ,0x18 ,0x20 ,0x21 ,0x22 ,0x23 ,0x38 ,0x38 ,0x38 ,0x38 ,0x04 ,0x05 ,0x06 ,0x07 ,0x00 ,0x01 ,0x02 ,0x03 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18);
	lcm_dcs_write_seq_static(ctx, 0xD6 ,0x18 ,0x18 ,0x38 ,0x38 ,0x39 ,0x39 ,0x18 ,0x18 ,0x23 ,0x22 ,0x21 ,0x20 ,0x18 ,0x18 ,0x38 ,0x38 ,0x03 ,0x02 ,0x01 ,0x00 ,0x07 ,0x06 ,0x05 ,0x04 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18);
	lcm_dcs_write_seq_static(ctx, 0xE7 ,0xFF ,0x06 ,0x01);
	lcm_dcs_write_seq_static(ctx, 0xBD ,0x01);
	lcm_dcs_write_seq_static(ctx, 0xE7 ,0x01);
	lcm_dcs_write_seq_static(ctx, 0xBD ,0x02);
	lcm_dcs_write_seq_static(ctx, 0xD8 ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xF0 ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xF0);
	lcm_dcs_write_seq_static(ctx, 0xBD ,0x00);
	lcm_dcs_write_seq_static(ctx, 0xE0 ,0x07 ,0x03 ,0x08 ,0x0D ,0x11 ,0x16 ,0x29 ,0x2F ,0x35 ,0x32 ,0x4D ,0x56 ,0x5F ,0x72 ,0x75 ,0x82 ,0x8E ,0xA3 ,0xA5 ,0x53 ,0x5B ,0x68 ,0x7F ,0x00 ,0x03 ,0x08 ,0x0D ,0x11 ,0x16 ,0x29 ,0x2F ,0x35 ,0x32 ,0x4D ,0x56 ,0x5F ,0x72 ,0x75 ,0x82 ,0x8E ,0xA3 ,0xA5 ,0x53 ,0x5B ,0x68 ,0x7F);
	lcm_dcs_write_seq_static(ctx, 0xBA ,0x71 ,0x23 ,0xA8 ,0x93 ,0xB2 ,0xC0 ,0xC0 ,0x01 ,0x10 ,0x00 ,0x00 ,0x00 ,0x0D ,0x3D ,0x82 ,0x77 ,0x04 ,0x01 ,0x00);
	lcm_dcs_write_seq_static(ctx, 0xBF ,0xFC ,0x00 ,0x04 ,0x9C ,0xF6 ,0x00 ,0x51);
	lcm_dcs_write_seq_static(ctx, 0xCB ,0x00 ,0x13 ,0x00 ,0x02 ,0x47);
	lcm_dcs_write_seq_static(ctx, 0xBD ,0x01);
	lcm_dcs_write_seq_static(ctx, 0xCB ,0x01);
	lcm_dcs_write_seq_static(ctx, 0xBD ,0x02);
	lcm_dcs_write_seq_static(ctx, 0xB4 ,0x42 ,0x00 ,0x33 ,0x00 ,0x33 ,0x88 ,0xB3 ,0x00);
	lcm_dcs_write_seq_static(ctx, 0xB1 ,0x7F ,0x03 ,0xF5);
	lcm_dcs_write_seq_static(ctx, 0xBD ,0x00);
	lcm_dcs_write_seq_static(ctx, 0xC9 ,0x00 ,0x08 ,0x80);
	
	lcm_dcs_write_seq_static(ctx, 0x11, 0x00);
	msleep(85);
	lcm_dcs_write_seq_static(ctx, 0x51, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x53, 0x24);
	lcm_dcs_write_seq_static(ctx, 0x35, 0x00);
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
	msleep(80);

	ctx->error = 0;
	ctx->prepared = false;

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_disable();
#else
	pr_info("%s: tp_gesture_enable_flag = %d \n", __func__, tp_gesture_enable_flag());
	if ((0 == tp_gesture_enable_flag()) || (esd_flag == 1)) {
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
	*/

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

	pr_info("%s+\n", __func__);

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
	
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
        usleep_range(5 * 1000, 5 * 1000);
        gpiod_set_value(ctx->reset_gpio, 0);
        usleep_range(5 * 1000, 5 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
        usleep_range(10 * 1000, 10 * 1000);
        devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	pr_info("%s-\n", __func__);
	#ifdef OPLUS_BUG_STABILITY
//  	lcd_queue_load_tp_fw();
  	#endif/* OPLUS_BUG_STABILITY */
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

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5 * 1000, 5 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(10 * 1000, 10 * 1000);
	lcd_queue_load_tp_fw();
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
	.clock = 86881,
	.hdisplay = HAC,
	.hsync_start = HAC + 22,//HFP
	.hsync_end = HAC + 22 + 21,//HSA
	.htotal = HAC + 22 + 21 + 21,//HBP
	.vdisplay = VAC,
	.vsync_start = VAC + 240,//VFP
	.vsync_end = VAC + 240 + 2,//VSA
	.vtotal = VAC + 240 + 2 + 16,//VBP
	.vrefresh = 60,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	//.pll_clk = 553,
	//.vfp_low_power = 2528,//45hz
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9c,
	},
	.data_rate = 730,
	.dyn = {
		.pll_clk = 362,
		.switch_en = 1,
		.data_rate = 724,
		.hbp = 33,
	},
};

static int map_exp[4096] = {0};

static void init_global_exp_backlight(void)
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

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0xFF, 0xFF};

	pr_err("%s backlight = %d\n", __func__, level);
	if (level > 4095)
                level = 4095;
	if ((level > 0) && (level < oplus_max_normal_brightness)) {
		level = map_exp[level];
	}
	pr_err ("%s after change backlight = %d\n", __func__, level);
	if ((level > 0) && (level < 5))
                level = 5;
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0) {
		level = 2047;
	}
        bl_tb0[1] = level >> 8;
        bl_tb0[2] = level & 0xFF;
	esd_brightness = level;
	if (!cb)
		return -1;

	pr_err("%s SYQ bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	//return 0;
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
#ifdef OPLUS_BUG_STABILITY
/*	if ((last_brightness == 0) && (cabc_lastlevel != 0)) {
		bl_tb1[1] = cabc_lastlevel;
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
		cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	}
*/
	last_brightness = level;
#endif
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
	bl_tb0[1] = (u8)cabc_mode;
    msleep(5);
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));//FB 01
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));//55 0X
#ifdef OPLUS_BUG_STABILITY
	cabc_lastlevel = cabc_mode;
#endif
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
	init_global_exp_backlight();
	if (is_project(20651) || is_project(20652))
		disp_aal_set_dre_en(1);
	if (is_project(20651) || is_project(20652)) {
		memcpy(ext_params.vendor, "innolux_nt_carr", strlen("innolux_nt_carr") > 31?31:strlen("innolux_nt_carr"));
		memcpy(ext_params.manufacture, "nt2048", strlen("nt2048") > 31?31:strlen("nt2048"));
		register_device_proc("lcd","innolux_nt_carr","nt2048");
	} else {
		memcpy(ext_params.vendor, "innolux_nt", strlen("innolux_nt") > 31?31:strlen("innolux_nt"));
		memcpy(ext_params.manufacture, "nt2048", strlen("nt2048") > 31?31:strlen("nt2048"));
		register_device_proc("lcd","innolux_nt","nt2048");
	}
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
		.compatible = "oplus21037_hx83102d_txd_vdo",
	},
	{} };

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
			.name = "oplus21037_hx83102d_txd_vdo_lcm_drv",
			.owner = THIS_MODULE,
			.of_match_table = jdi_of_match,
		},
};

module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("Elon Hsu <elon.hsu@mediatek.com>");
MODULE_DESCRIPTION("jdi r66451 CMD AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
