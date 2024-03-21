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
#include "include/AC102_P_3_A0010_fhdp_dsi_vdo.h"
#endif

/* add for dips_drv log  */
#include "../oplus/oplus_display_mtk_debug.h"
#if defined(CONFIG_RT4831A_I2C)
#include "../../../misc/mediatek/gate_ic/gate_i2c.h"
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/AC102_P_3_A0010_data_hw_roundedpattern.h"
#endif
#define MAX_NORMAL_BRIGHTNESS    (3276)
extern unsigned int oplus_display_brightness;
extern unsigned long oplus_max_normal_brightness;
extern unsigned long esd_flag;
extern unsigned int g_shutdown_flag;

extern void __attribute((weak)) lcd_queue_load_tp_fw(void) { return; };
extern int __attribute__((weak)) tp_gesture_enable_flag(void) {return 0;};
extern int __attribute((weak)) tp_control_reset_gpio(bool enable) { return 0; };
extern int _20015_lcm_i2c_write_bytes(unsigned char addr, unsigned char value);
extern int _20015_lcm_i2c_read_bytes(unsigned char addr, char *value);
/* enable this to check panel self -bist pattern */
/* #define PANEL_BIST_PATTERN */

/* option function to read data from some panel address */
/* #define PANEL_SUPPORT_READBACK */

struct lcm {
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

static unsigned char g_GammaFlag = 0;
static int current_fps = 60;
static int cabc_status = 0;
static int esd_brightness;

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
		DISP_ERR("[error]error %d reading dcs seq:(%#x)\n", ret, cmd);
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
static struct LCD_setting_table lcm_suspend_setting[] = {
	{0xFF, 2, {0xFF, 0x10} },
	{0x28, 2, {0x28, 0x00} },

};

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
		DISP_ERR("[error]error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

#define HFP_SUPPORT 0

static void lcm_init_set_cabc(struct lcm *ctx, int cabc_mode)
{
	DISP_INFO("[lcm] init set cabc_mode %d\n", cabc_mode);
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x78, 0x07, 0x00);
	if (cabc_mode == 0) {
		lcm_dcs_write_seq_static(ctx, 0x55, 0x00);
	} else if (cabc_mode == 1) {
		lcm_dcs_write_seq_static(ctx, 0x55, 0x01);
	} else if (cabc_mode == 2) {
		lcm_dcs_write_seq_static(ctx, 0x55, 0x02);
	} else if (cabc_mode == 3) {
		lcm_dcs_write_seq_static(ctx, 0x55, 0x02);
	} else {
		lcm_dcs_write_seq_static(ctx, 0x55, 0x00);
	}
}

static void lcm_panel_init(struct lcm *ctx)
{
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	DISP_INFO("begin!\n");
	if (IS_ERR(ctx->reset_gpio)) {
		DISP_ERR("[error]: cannot get reset_gpio %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return;
	}
	udelay(2 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(5 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(5 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	usleep_range(10 * 1000, 10 * 1000);
	/* Add for loading tp fw when screen ON*/
	lcd_queue_load_tp_fw();

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x78, 0x07, 0x06);
	lcm_dcs_write_seq_static(ctx, 0x3E, 0xE2);//OFF 11 reload
	/*1bit esdcheck*/
	lcm_dcs_write_seq_static(ctx, 0x48, 0x0F);
	lcm_dcs_write_seq_static(ctx, 0x4D, 0xFF);
	lcm_dcs_write_seq_static(ctx, 0x4E, 0xFF);
	lcm_dcs_write_seq_static(ctx, 0x4F, 0x23);
	lcm_dcs_write_seq_static(ctx, 0xC7, 0x05);

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x78, 0x07, 0x02);
	lcm_dcs_write_seq_static(ctx, 0x1B, 0x00);//90 /180
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x78, 0x07, 0x07);
	lcm_dcs_write_seq_static(ctx, 0x29, 0xCF);//ON dsc
	lcm_dcs_write_seq_static(ctx, 0x82, 0x20);

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x78, 0x07, 0x17);//dsc params
	lcm_dcs_write_seq_static(ctx, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x89, 0x30);
	lcm_dcs_write_seq_static(ctx, 0xfe, 0x80, 0x09, 0x60, 0x04, 0x38, 0x00, 0x0c, 0x02, 0x1c, 0x02);
	lcm_dcs_write_seq_static(ctx, 0xfe, 0x1c, 0x00, 0xaa, 0x02, 0x0e, 0x00, 0x20, 0x00, 0x43, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xfe, 0x07, 0x00, 0x0c, 0x08, 0xbb, 0x08, 0x7a, 0x18, 0x00, 0x1b);
	lcm_dcs_write_seq_static(ctx, 0xfe, 0xa0, 0x03, 0x0c, 0x20, 0x00, 0x06, 0x0b, 0x0b, 0x33, 0x0e);
	lcm_dcs_write_seq_static(ctx, 0xfe, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79);
	lcm_dcs_write_seq_static(ctx, 0xfe, 0x7b, 0x7d, 0x7e, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40, 0x09);
	lcm_dcs_write_seq_static(ctx, 0xfe, 0xbe, 0x19, 0xfc, 0x19, 0xfa, 0x19, 0xf8, 0x1a, 0x38, 0x1a);
	lcm_dcs_write_seq_static(ctx, 0xfe, 0x78, 0x1a, 0xb6, 0x2a, 0xf6, 0x2b, 0x34, 0x2b, 0x74, 0x3b);
	lcm_dcs_write_seq_static(ctx, 0xfe, 0x74, 0x6b, 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xfe, 0x00, 0x00, 0x00);

	/* improve TP noise */
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x78, 0x07, 0x05);
	lcm_dcs_write_seq_static(ctx, 0x69, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x72, 0x6A);
	lcm_dcs_write_seq_static(ctx, 0x74, 0x42);

	/* CABC start */
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x78, 0x07, 0x06);
	lcm_dcs_write_seq_static(ctx, 0x08, 0x20);
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x78, 0x07, 0x03);
	lcm_dcs_write_seq_static(ctx, 0x83, 0x20);
	lcm_dcs_write_seq_static(ctx, 0x84, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x86, 0x47);
	lcm_dcs_write_seq_static(ctx, 0x87, 0x56);
	lcm_dcs_write_seq_static(ctx, 0xAF, 0x18);
	lcm_dcs_write_seq_static(ctx, 0x8C, 0xE2);
	lcm_dcs_write_seq_static(ctx, 0x8D, 0xE3);
	lcm_dcs_write_seq_static(ctx, 0x8E, 0xE5);
	lcm_dcs_write_seq_static(ctx, 0x8F, 0xE6);
	lcm_dcs_write_seq_static(ctx, 0x90, 0xE9);
	lcm_dcs_write_seq_static(ctx, 0x91, 0xEB);
	lcm_dcs_write_seq_static(ctx, 0x92, 0xEE);
	lcm_dcs_write_seq_static(ctx, 0x93, 0xF0);
	lcm_dcs_write_seq_static(ctx, 0x94, 0xF6);
	lcm_dcs_write_seq_static(ctx, 0x95, 0xFA);

	lcm_dcs_write_seq_static(ctx, 0x96, 0xBF);
	lcm_dcs_write_seq_static(ctx, 0x97, 0xC4);
	lcm_dcs_write_seq_static(ctx, 0x98, 0xC7);
	lcm_dcs_write_seq_static(ctx, 0x99, 0xC9);
	lcm_dcs_write_seq_static(ctx, 0x9A, 0xCF);
	lcm_dcs_write_seq_static(ctx, 0x9B, 0xD4);
	lcm_dcs_write_seq_static(ctx, 0x9C, 0xDA);
	lcm_dcs_write_seq_static(ctx, 0x9D, 0xE2);
	lcm_dcs_write_seq_static(ctx, 0x9E, 0xEB);
	lcm_dcs_write_seq_static(ctx, 0x9F, 0xFA);
	/* CABC end */

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x78, 0x07, 0x03);
	lcm_dcs_write_seq_static(ctx, 0x83, 0x20);//15khz
	lcm_dcs_write_seq_static(ctx, 0x84, 0x00);//12bit

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x78, 0x07, 0x06);
	lcm_dcs_write_seq_static(ctx, 0x08, 0x20);//15khz

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x78, 0x07, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x53, 0x24);
	lcm_dcs_write_seq_static(ctx, 0x35, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x55, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x55, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x11, 0x00);
	usleep_range(61 * 1000, 62 * 1000);
	lcm_dcs_write_seq_static(ctx, 0x29, 0x00);
	usleep_range(4 * 1000, 5 * 1000);
	lcm_init_set_cabc(ctx, cabc_status);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	DISP_INFO("begin!\n");
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
	DISP_INFO("begin!\n");

	usleep_range(5 * 1000, 5 * 1000);
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x78, 0x07, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x28);
	usleep_range(20 * 1000, 21 * 1000);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(80 * 1000, 81 * 1000);

	DISP_INFO(" tp_gesture_enable_flag = %d  g_shutdown_flag = %d, esd_flag = %d\n", tp_gesture_enable_flag(), g_shutdown_flag, esd_flag);
	if (1 == g_shutdown_flag) {
		ctx->reset_gpio =
			devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->reset_gpio)) {
			DISP_ERR("[error]: cannot get reset_gpio %ld\n",
				PTR_ERR(ctx->reset_gpio));
			return PTR_ERR(ctx->reset_gpio);
		}
		gpiod_set_value(ctx->reset_gpio, 0);
		devm_gpiod_put(ctx->dev, ctx->reset_gpio);
		udelay(3000);
		tp_control_reset_gpio(false);
		udelay(2000);

	}
	if (0 == tp_gesture_enable_flag() || (esd_flag == 1) || (1 == g_shutdown_flag)) {
		ctx->bias_ldo = devm_gpiod_get_index(ctx->dev,
			"bias", 0, GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->bias_ldo)) {
			DISP_ERR("[error]: cannot get bias_ldo %ld\n",
				PTR_ERR(ctx->bias_ldo));
			return PTR_ERR(ctx->bias_ldo);
		}
		gpiod_set_value(ctx->bias_ldo, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_ldo);

		udelay(2000);

		ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
			"bias", 2, GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->bias_pos)) {
			DISP_ERR("[error]: cannot get bias_pos %ld\n",
				PTR_ERR(ctx->bias_pos));
			return PTR_ERR(ctx->bias_pos);
		}
		gpiod_set_value(ctx->bias_pos, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_pos);

		udelay(3000);

		ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
			"bias", 1, GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->bias_neg)) {
			DISP_ERR("[error]: cannot get bias_neg %ld\n",
				PTR_ERR(ctx->bias_neg));
			return PTR_ERR(ctx->bias_neg);
		}
		gpiod_set_value(ctx->bias_neg, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_neg);
	}

	ctx->error = 0;
	ctx->prepared = false;
	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;
	unsigned char reg_value = 0;
	DISP_INFO("begin!\n");
	if (ctx->prepared)
		return 0;

	ctx->bias_ldo = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_LOW);
	if (IS_ERR(ctx->bias_ldo)) {
		DISP_ERR("[error]: cannot get bias_ldo %ld\n",
			PTR_ERR(ctx->bias_ldo));
		return PTR_ERR(ctx->bias_ldo);
	}
	gpiod_set_value(ctx->bias_ldo, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_ldo);

	udelay(2000);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		DISP_ERR("[error]: cannot get bias_neg %ld\n",
			PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	udelay(3000);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 2, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		DISP_ERR("[error]: cannot get bias_pos %ld\n",
			PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
	ret = _20015_lcm_i2c_read_bytes(0x03, &reg_value);
	if (ret < 0) {
		DISP_DEBUG("fail to lcm gate driver IC read 0x03\n");
	}else {
		DISP_DEBUG("_20015_lcm_i2c_read_bytes 0x03 = 0x%x   ret = %d\n", reg_value, ret);
	}
	usleep_range(1000, 1100);
	ret = _20015_lcm_i2c_write_bytes(0x00, 0x0f); /*set VSP voltage etc. 4.0+ (parameter 2)* 0.10*/
	if (ret < 0) {
		DISP_DEBUG("fail to lcm gate driver IC write 0x00\n");
	}else {
		DISP_DEBUG("_20015_lcm_i2c_write_bytes 0x00 return value = %d\n", ret);
	}
	usleep_range(1000, 1100);
	ret = _20015_lcm_i2c_write_bytes(0x01, 0x0f); /*set VSN voltage etc. 4.0+ (parameter 2)* 0.10*/
	if (ret < 0) {
		DISP_DEBUG("fail to lcm gate driver IC write 0x01\n");
	}else {
		DISP_DEBUG("_20015_lcm_i2c_write_bytes 0x01 return value = %d\n", ret);
	}
	usleep_range(1000, 1100);
	ret = _20015_lcm_i2c_write_bytes(0x03, 0x43); /*Applications configure register,default:0x43*/
	if (ret < 0) {
		DISP_DEBUG("fail to lcm gate driver IC write 0x03\n");
	}else {
		DISP_DEBUG("_20015_lcm_i2c_write_bytes 0x03 return value = %d\n", ret);
	}

	udelay(2000);
	tp_control_reset_gpio(true);

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
	DISP_INFO("begin!\n");
	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

static void lcm_gamma_enter (void *dsi, dcs_write_gce cb, void *handle)
{
    char bl_tb0[] = {0xFF, 0x78, 0x07, 0x00};
    char bl_tb1[] = {0xFF, 0x78, 0x07, 0x08};
    char bl_tb2[] = {0xE0, 0x15, 0xB5, 0xB7, 0xBA, 0x15, 0xC1, 0xC8, 0xCF, 0x15, 0xDC, 0xE8, 0xFF, 0x2A, 0x15, 0x3F};
    char bl_tb3[] = {0xfe, 0x67, 0x2A, 0x95, 0xCD, 0xF1, 0x3F, 0x21, 0x42, 0x66, 0x3F, 0x7F, 0x9D, 0xC6, 0x0F, 0xD6, 0xD7};
    char bl_tb4[] = {0xE1, 0x15, 0xB5, 0xB7, 0xBA, 0x15, 0xC1, 0xC8, 0xCF, 0x15, 0xDC, 0xE8, 0xFF, 0x2A, 0x15, 0x3F};
    char bl_tb5[] = {0xfe, 0x67, 0x2A, 0x95, 0xCD, 0xF1, 0x3F, 0x21, 0x42, 0x66, 0x3F, 0x7F, 0x9D, 0xC6, 0x0F, 0xD6, 0xD7};

    cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
    cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
    cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
    cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
}

static void lcm_gamma_exit (void *dsi, dcs_write_gce cb, void *handle)
{
    char bl_tb0[] = {0xFF, 0x78, 0x07, 0x00};
    char bl_tb1[] = {0xFF, 0x78, 0x07, 0x08};
    char bl_tb2[] = {0xE0, 0x00, 0x00, 0x1C, 0x4A, 0x00, 0x87, 0xB4, 0xD7, 0x15, 0x0E, 0x38, 0x79, 0x25, 0xA7, 0xF3};
    char bl_tb3[] = {0xfe, 0x29, 0x2A, 0x60, 0x9D, 0xC3, 0x3E, 0xF4, 0x13, 0x44, 0x3F, 0x59, 0x7C, 0xAE, 0x0F, 0xCE, 0xD7};
    char bl_tb4[] = {0xE1, 0x00, 0x00, 0x1C, 0x4A, 0x00, 0x87, 0xB4, 0xD7, 0x15, 0x0E, 0x38, 0x79, 0x25, 0xA7, 0xF3};
    char bl_tb5[] = {0xfe, 0x29, 0x2A, 0x60, 0x9D, 0xC3, 0x3E, 0xF4, 0x13, 0x44, 0x3F, 0x59, 0x7C, 0xAE, 0x0F, 0xCE, 0xD7};

    cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
    cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
    cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
    cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
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
	.pll_clk = MIPI_CLK,
        .phy_timcon = {
		.hs_trail = 9,
		.clk_trail = 10,
        },
	.data_rate = DATA_RATE,
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
	.pll_clk = MIPI_CLK,
        .phy_timcon = {
		.hs_trail = 9,
		.clk_trail = 10,
        },
	.data_rate = DATA_RATE,
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

static char bl_tb0[] = {0x51, 0x03, 0xff};
static char bl_tb2[] = {0xFF, 0x78, 0x07, 0x00};
//static char bl_tb3[] = {0x53, 0x24};

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
    char APbl_tb1[] = {0xFF, 0x78, 0x07, 0x06};
    char APbl_tb2[] = {0x3C, 0x08};
    char APbl_tb3[] = {0xFF, 0x78, 0x07, 0x00};

    if (level > 4095)
        level = 4095;
    DISP_DEBUG(" backlight1 =  %d\n", level);

    if(level < 14 && level > 0 && g_GammaFlag == 0){
        g_GammaFlag = 1;
        DISP_DEBUG(" backlight < 14 enter gamma!\n");
        lcm_gamma_enter(dsi, cb, handle);
    }else if(level > 13 && g_GammaFlag == 1){
        g_GammaFlag = 0;
        DISP_DEBUG(" backlight > 13 exit gamma!\n");
        lcm_gamma_exit(dsi, cb, handle);
    }else if(level == 0){
        g_GammaFlag = 0;
    }

    bl_tb0[1] = (level & 0xff00) >> 8;
    bl_tb0[2] = level & 0xff;
    if (!cb)
        return -1;
    DISP_DEBUG(" backlight2 =%d,bl0=0x%x,bl1=0x%x\n", level,bl_tb0[1],bl_tb0[2]);
    cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
    cb(dsi, handle, APbl_tb1, ARRAY_SIZE(APbl_tb1));
    cb(dsi, handle, APbl_tb2, ARRAY_SIZE(APbl_tb2));
    cb(dsi, handle, APbl_tb3, ARRAY_SIZE(APbl_tb3));

    oplus_display_brightness = level;
    esd_brightness = level;
    return 0;
}

static struct drm_display_mode *get_mode_by_id(struct drm_panel *panel,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;
	DISP_INFO("begin!\n");
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
	struct lcm *ctx = panel_to_lcm(panel);
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		DISP_ERR("[error]: cannot get reset_gpio %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
#ifdef BDG_PORTING_DBG
	struct lcm *ctx = panel_to_lcm(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	unsigned char data[3] = {0x00, 0x00, 0x00};
	unsigned char id[3] = {0x6e, 0x48, 0x00};
	ssize_t ret;
	ret = mipi_dsi_dcs_read(dsi, 0x4, data, 3);
	if (ret < 0) {
		DISP_ERR("[error]%s error\n", __func__);
		return 0;
	}

	DISP_INFO("ATA read data %x %x %x\n", data[0], data[1], data[2]);

	if (data[0] == id[0])
		return 1;

	DISP_INFO("ATA expect read data is %x %x %x\n",
			id[0], id[1], id[2]);
#endif
	return 1;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0x51, 0x03, 0xff};
	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
	if (!cb)
		return -1;
	DISP_INFO(" bl_tb0[1]=%x, bl_tb0[2]=%x\n", bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 1;
}

static void lcm_cabc_mode_switch(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int cabc_mode)
{
	DISP_INFO("begin!\n");
	unsigned char cabc_cmd_1[] = {0xFF, 0x78,0x07,0x00};
	unsigned char cabc_cmd_2[] = {0x55, 0x00};
	cabc_status = cabc_mode;

	switch(cabc_mode) {
		case 0://CLOSE
			cabc_cmd_2[1] = 0x00;
			break;
		case 1://UI
			cabc_cmd_2[1] = 0x01;
			break;
		case 2://STILL
			cabc_cmd_2[1] = 0x02;
			break;
		case 3://MOV--->STILL
			cabc_cmd_2[1] = 0x02;
			break;
		default:
			cabc_cmd_2[1] = 0x00;
			DISP_ERR("[lcd_info]: cabc_mode=%d is not support, close cabc !\n", cabc_mode);
			break;
	}
	cb(dsi, handle, cabc_cmd_1, ARRAY_SIZE(cabc_cmd_1));
	cb(dsi, handle, cabc_cmd_2, ARRAY_SIZE(cabc_cmd_2));
	DISP_INFO("[lcd_info]:cabc mode_%d, set cabc_para=%#x\n", cabc_mode, cabc_cmd_2[1]);
}

static struct mtk_panel_funcs ext_funcs = {
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.reset = panel_ext_reset,
	.ext_param_set = mtk_panel_ext_param_set,
	.ata_check = panel_ata_check,
	.cabc_switch = lcm_cabc_mode_switch,
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
	struct drm_display_mode *mode_1;
	struct drm_display_mode *mode_2;
	struct drm_display_mode *mode_3;
	struct drm_display_mode *mode_4;
	DISP_INFO("begin!\n");
	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		DISP_ERR("[error]failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

	mode_1 = drm_mode_duplicate(panel->drm, &performance_mode_1);
	if (!mode_1) {
		DISP_ERR("[error]failed to add mode %ux%ux@%u\n",
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
	DISP_INFO("begin!\n");
	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				DISP_ERR("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			DISP_ERR("device_node name %s\n", remote_node->name);
		}
	}

	if (remote_node != dev->of_node) {
		DISP_ERR(" lcm_ili7807s_vdo_120hz_vfp_6382 isn't current lcm\n");
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
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
		DISP_ERR("[error]cannot get reset-gpios %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	ctx->bias_ldo = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_LOW);
	if (IS_ERR(ctx->bias_ldo)) {
		DISP_ERR("[error]: cannot get bias_ldo %ld\n",
			PTR_ERR(ctx->bias_ldo));
		return PTR_ERR(ctx->bias_ldo);
	}
	gpiod_set_value(ctx->bias_ldo, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_ldo);
	DISP_INFO(" gpio 136 set low\n");

#ifndef CONFIG_RT4831A_I2C
	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		DISP_ERR("[error]: cannot get bias-pos 1 %ld\n",
			PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 2, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		DISP_ERR("[error]: cannot get bias-neg 2 %ld\n",
			PTR_ERR(ctx->bias_neg));
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
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	register_device_proc("lcd", "AC102", "P_3");
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	DISP_INFO("Successful\n");
	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
	DISP_INFO("begin!\n");
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "AC102_P_3_A0010_fhdp_dsi_vdo", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "AC102_P_3_A0010_fhdp_dsi_vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Jingjing Liu <jingjing.liu@mediatek.com>");
MODULE_DESCRIPTION("boe ili7807s vdo 90hz 6382 Panel Driver");
MODULE_LICENSE("GPL v2");

