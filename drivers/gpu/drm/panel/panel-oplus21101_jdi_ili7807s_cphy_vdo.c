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
#include <linux/fb.h>
#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

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
/*#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/oplus21041_mtk_data_hw_roundedpattern.h"
#endif*/

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/

extern unsigned long esd_flag;
/* extern int shutdown_flag; */
static int esd_brightness = 1023;
extern unsigned long oplus_max_normal_brightness;
extern void disp_aal_set_dre_en(int enable);
extern int __attribute__((weak)) tp_gesture_enable_flag(void) {return 0;};
extern void __attribute__((weak)) lcd_queue_load_tp_fw(void) {return;};
extern int _20015_lcm_i2c_write_bytes(unsigned char addr, unsigned char value);
extern void __attribute__((weak)) tp_gpio_current_leakage_handler(bool normal) {return;};
extern bool __attribute__((weak)) tp_boot_mode_normal() {return true;};
static int backlight_gamma = 0;
/*****************************************************************************
 * Data Structure
 *****************************************************************************/

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos;
	struct gpio_desc *bias_neg;
	struct gpio_desc *bias_en;
	struct gpio_desc *iscphy_gpio;
	bool prepared;
	bool enabled;

	int error;

    bool is_normal_mode;
};

#define lcm_dcs_write_seq(ctx, seq...)                                     \
	({                                                                     \
		const u8 d[] = {seq};                                          \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                      \
	})

#define lcm_dcs_write_seq_static(ctx, seq...)                              \
	({                                                                     \
		static const u8 d[] = {seq};                                   \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                      \
	})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
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
		pr_notice("error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;
	pr_info("%s+\n", __func__);

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("%s  0x%08x\n", __func__,buffer[0] | (buffer[1] << 8));
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

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
		pr_notice("error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static void lcm_panel_init(struct lcm *ctx)
{
	lcm_dcs_write_seq_static(ctx,0xFF,0x10);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x3B,0x03,0x14,0x36,0x04,0x04);
	lcm_dcs_write_seq_static(ctx,0xB0,0x00);
	lcm_dcs_write_seq_static(ctx,0xC0,0x00);  //no dsc=00,vdsc=03

	lcm_dcs_write_seq_static(ctx,0xC1,0x89,0x28,0x00,0x0C,0x02,0x00,0x02,0x0E,0x01,0x1F,0x00,0x07,0x08,0xBB,0x08,0x7A);
	lcm_dcs_write_seq_static(ctx,0xC2,0x10,0xF0);

	lcm_dcs_write_seq_static(ctx,0xFF,0x25);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x18,0x22);  //0x22: DP60hz,0x21: DP90hz,0x20: DP120hz

	lcm_dcs_write_seq_static(ctx,0xFF,0x23);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x00,0x80);
	lcm_dcs_write_seq_static(ctx,0x07,0x00);
	lcm_dcs_write_seq_static(ctx,0x08,0x01);
	lcm_dcs_write_seq_static(ctx,0x09,0x00);

	lcm_dcs_write_seq_static(ctx,0xFF,0xE0);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x35,0x82);

	lcm_dcs_write_seq_static(ctx,0xFF,0xF0);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x1C,0x01);
	lcm_dcs_write_seq_static(ctx,0x33,0x01);
	lcm_dcs_write_seq_static(ctx,0x5A,0x00);

	lcm_dcs_write_seq_static(ctx,0xFF,0xD0);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x53,0x22);
	lcm_dcs_write_seq_static(ctx,0x54,0x02);
	lcm_dcs_write_seq_static(ctx,0x1C,0x88);
	lcm_dcs_write_seq_static(ctx,0x1D,0x08);

	lcm_dcs_write_seq_static(ctx,0xFF,0xC0);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x9C,0x11);
	lcm_dcs_write_seq_static(ctx,0x9D,0x11);

	lcm_dcs_write_seq_static(ctx,0xFF,0x10);

	lcm_dcs_write_seq_static(ctx,0x51,0x03,0xFF);
	lcm_dcs_write_seq_static(ctx,0x53,0x24);
	lcm_dcs_write_seq_static(ctx,0x55,0x00);
	lcm_dcs_write_seq_static(ctx,0x11,0x00);
	msleep(120);
	lcm_dcs_write_seq_static(ctx,0x29,0x00);
	msleep(50);

	pr_info("alice - ili7807 %s\n", __func__);
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

	usleep_range(20 * 1000, 20 * 1000);
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

	usleep_range(5 * 1000, 5 * 1000);
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x78,0x07,0x00);
	usleep_range(10 * 1000, 10 * 1000);
	lcm_dcs_write_seq_static(ctx, 0x28);
	usleep_range(20 * 1000, 20 * 1000);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(80 * 1000, 80 * 1000);
/*
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
*/

	pr_info("%s: tp_gesture_enable_flag = %d \n", __func__, tp_gesture_enable_flag());
	if (0 == tp_gesture_enable_flag() || (esd_flag == 1) /*|| (shutdown_flag == 1)*/) {
		pr_info("%s: going to cut off power \n", __func__);
		ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->reset_gpio, 0);
		devm_gpiod_put(ctx->dev, ctx->reset_gpio);
		if(esd_flag != 1) {
			tp_irq = gpio_to_irq(316);
			pr_info("tp_irq:%d\n", tp_irq);
			disable_irq_nosync(tp_irq);
		}
		tp_gpio_current_leakage_handler(false);
		usleep_range(2 * 1000, 2 * 1000);
		ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
			"bias", 1, GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_neg, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_neg);

		usleep_range(2 * 1000, 2 * 1000);

		ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
			"bias", 0, GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_pos, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_pos);
		usleep_range(2 * 1000, 2 * 1000);
    }

	ctx->error = 0;
	ctx->prepared = false;

	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("zk-20211210-20.59-%s+\n", __func__);

	//add for ldo
	ctx->bias_en = devm_gpiod_get(ctx->dev, "ldo", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_en, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_en);
	udelay(2000);
	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(5000);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	_20015_lcm_i2c_write_bytes(0x0, 0xf);
	_20015_lcm_i2c_write_bytes(0x1, 0xf);
	pr_info("%s-\n", __func__);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

        pr_info("%s+\n", __func__);

	if (ctx->prepared)
                return 0;
	if (0 == tp_gesture_enable_flag() || (esd_flag == 1) /*|| (shutdown_flag == 1)*/) {
		ctx->bias_en = devm_gpiod_get(ctx->dev, "ldo", GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_en, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_en);
		usleep_range(30 * 1000, 30 * 1000);
	}

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
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
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10 * 1000, 10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(10 * 1000, 10 * 1000);

	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
//	lcm_vdd_tp_start(ctx);
	usleep_range(10 * 1000, 10 * 1000);
//end
//	lcm_vdd_tp_end(ctx);/* #ifdef OPLUS_BUG_STABILITY */
	if (tp_boot_mode_normal()) {
		tp_gpio_current_leakage_handler(true);
		lcd_queue_load_tp_fw();
	}
	lcm_panel_init(ctx);
	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif
	pr_info("%s-\n", __func__);
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

static const struct drm_display_mode default_mode = {
	.clock = 256146,
	.hdisplay = 1080,
	.hsync_start = 1080 + 614,
	.hsync_end = 1080 + 614 + 20,
	.htotal = 1080 + 614 + 20 + 20,
	.vdisplay = 2388,
	.vsync_start = 2388 + 54,
	.vsync_end = 2388 + 54 + 8,
	.vtotal = 2388 + 54 + 8 + 12,
	.vrefresh = 60,
};

/*static const struct drm_display_mode performance_mode = {
	.clock = 300591,
	.hdisplay = 1080,
	.hsync_start = 1080 + 26,
	.hsync_end = 1080 + 26 + 4,
	.htotal = 1080 + 26 + 4 + 65,
	.vdisplay = 2400,
	.vsync_start = 2400 + 16,
	.vsync_end = 2400 + 16 + 4,
	.vtotal = 2400 + 16 + 4 + 20,
	.vrefresh = 90,
};*/

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.pll_clk = 450,
	.vfp_low_power = 2500,
	/* .mipi_cphy = 1, */
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.wait_sof_before_dec_vfp = 1,
	.lcm_esd_check_table[0] = {
	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9c,
	},
	.is_cphy = 1,
	.data_rate = 900,
	.dyn_fps = {
		.switch_en = 0,//zk
		.vact_timing_fps = 90,
	},
	.dyn = {
		.switch_en = 0,//zk
		.pll_clk = 553,
		.data_rate = 1107,
		.vfp = 1295,
	},
	.phy_timcon = {
		.hs_prpr = 0x0B,
	},
	.vendor = "ILI7807s_JDI_OdinA",
	.manufacture = "OdinA_ili_jdi3276",
/*#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 0,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif*/
};

/*static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 548,
	.vfp_low_power = 1260,
	// .mipi_cphy = 1,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.wait_sof_before_dec_vfp = 1,
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
		.data_rate = 1107,
		.vfp = 38,
	},
	.phy_timcon = {
		.hs_prpr = 0x0B,
	}
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 0,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};*/

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
	char bl_tb0[] = {0x51, 0x07, 0xff, 0x00};
	/*char bl_tb1[] = {0xFF, 0x78,0x07,0x00};
	char bl_tb2[] = {0xFF, 0x78,0x07,0x08};
	char bl_tb3[] = {0xE0, 0x15,0x9F,0x9E,0xA3,0x15,0xAB,0xB3,0xBB,0x15,0xCC,0xDB,0xF5,0x2A,0x0D,0x38,0x61,0x2A,0x8F,0xC8,0xED,0x3F,0x1F,0x41,0x63,0x3F,0x7B,0x99,0xC2,0x0F,0xD8,0xD9};
	char bl_tb4[] = {0xE1, 0x15,0x9F,0x9E,0xA3,0x15,0xAB,0xB3,0xBB,0x15,0xCC,0xDB,0xF5,0x2A,0x0D,0x38,0x61,0x2A,0x8F,0xC8,0xED,0x3F,0x1F,0x41,0x63,0x3F,0x7B,0x99,0xC2,0x0F,0xD8,0xD9};
	char bl_tb5[] = {0xE0, 0x00,0x00,0x1B,0x46,0x00,0x82,0xAD,0xD0,0x15,0x05,0x2F,0x6F,0x25,0x9C,0xE3,0x1D,0x2A,0x56,0x96,0xBE,0x3E,0xF3,0x1A,0x45,0x3F,0x5E,0x84,0xB5,0x0F,0xD6,0xD9};
	char bl_tb6[] = {0xE1, 0x00,0x00,0x1B,0x46,0x00,0x82,0xAD,0xD0,0x15,0x05,0x2F,0x6F,0x25,0x9C,0xE3,0x1D,0x2A,0x56,0x96,0xBE,0x3E,0xF3,0x1A,0x45,0x3F,0x5E,0x84,0xB5,0x0F,0xD6,0xD9};*/

	if (level > 4095)
		level = 4095;
	pr_err("zk-%s backlight = %d\n", __func__, level);
//dump_stack();
	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;
	esd_brightness = level;
	if (!cb)
		return -1;

	/*if(level < 14 && level > 0){
		backlight_gamma = 1;
		cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
		cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
		cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
	}else if(level > 13 && backlight_gamma == 1){
		backlight_gamma = 0;
		cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
		cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
		cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
	}*/

	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	//cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
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

	if (ext && mode == 0)
		ext->params = &ext_params;
	//else if (mode == 1)
	//	ext->params = &ext_params_90hz;
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
	//else if (mode == 1)
	//	ext_para = &ext_params_90hz;
	else
		ret = 1;

	return ret;

}

static void cabc_switch(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int cabc_mode)
{
	char bl_tb0[] = {0x55, 0x00};
	char bl_tb1[] = {0xFF, 0x78,0x07,0x00};
	char bl_tb3[] = {0x53, 0x2C};

	pr_err("%s cabc = %d\n", __func__, cabc_mode);
	if(cabc_mode == 3)
		cabc_mode = 2;

	bl_tb0[1] = (u8)cabc_mode;
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
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

static int lcm_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;
	//struct drm_display_mode *mode2;

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

	/*mode2 = drm_mode_duplicate(panel->drm, &performance_mode);
	if (!mode2) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			performance_mode.hdisplay,
			performance_mode.vdisplay,
			performance_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode2);*/

	panel->connector->display_info.width_mm = 69;
	panel->connector->display_info.height_mm = 150;

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

/*zk begin add for c-ppy*/
	ctx->iscphy_gpio = devm_gpiod_get(dev, "iscphy", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->iscphy_gpio)) {
		dev_err(dev, "cannot get iscphy-gpios %ld\n",
			PTR_ERR(ctx->iscphy_gpio));
		return PTR_ERR(ctx->iscphy_gpio);
	}
	devm_gpiod_put(dev, ctx->iscphy_gpio);
/*end add for c-ppy*/

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
	ctx->panel.funcs = &lcm_drm_funcs;

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
        init_global_exp_backlight();
	disp_aal_set_dre_en(1);
	register_device_proc("lcd", "ILI7807s_JDI_OdinA", "OdinA_ili_jdi3276");
	pr_info("%s-\n", __func__);
	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
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

static const struct of_device_id lcm_of_match[] = {
	{
		.compatible = "oplus21101,jdi,ili7807s,cphy,vdo",
	},
	{} };

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {

			.name = "oplus21101_jdi_ili7807s_cphy_vdo",
			.owner = THIS_MODULE,
			.of_match_table = lcm_of_match,
		},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Elon Hsu <elon.hsu@mediatek.com>");
MODULE_DESCRIPTION("lcm r66451 CMD AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
