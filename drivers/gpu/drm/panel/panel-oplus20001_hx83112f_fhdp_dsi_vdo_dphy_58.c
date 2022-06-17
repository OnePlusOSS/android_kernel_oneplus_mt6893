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
/* enable this to check panel self -bist pattern */
/* #define PANEL_BIST_PATTERN */
/****************TPS65132***********/
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
//#include "lcm_i2c.h"

#ifndef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif

#define AVDD_REG 0x00
#define AVDD_REG 0x01
#define MAX_NORMAL_BRIGHTNESS   3562
/* i2c control start */
#define LCM_I2C_ID_NAME "I2C_LCD_BIAS"
static struct i2c_client *_lcm_i2c_client;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int _lcm_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
static int _lcm_i2c_remove(struct i2c_client *client);

#ifdef OPLUS_BUG_STABILITY
extern int __attribute__((weak)) tp_gesture_enable_flag(void) { return 0; };
extern void __attribute__((weak)) lcd_queue_load_tp_fw(void) { return; };
#endif /* OPLUS_BUG_STABILITY */
extern unsigned long esd_flag;
extern unsigned long oplus_max_normal_brightness;
static int esd_brightness;
/*****************************************************************************
 * Data Structure
 *****************************************************************************/
struct _lcm_i2c_dev {
	struct i2c_client *client;

};

static const struct of_device_id _lcm_i2c_of_match[] = {
	{ .compatible = "mediatek,I2C_LCD_BIAS", },
	{},
};

static const struct i2c_device_id _lcm_i2c_id[] = {
	{LCM_I2C_ID_NAME, 0},
	{}
};

static struct i2c_driver _lcm_i2c_driver = {
	.id_table = _lcm_i2c_id,
	.probe = _lcm_i2c_probe,
	.remove = _lcm_i2c_remove,
	/* .detect               = _lcm_i2c_detect, */
	.driver = {
		   .owner = THIS_MODULE,
		   .name = LCM_I2C_ID_NAME,
		   .of_match_table = _lcm_i2c_of_match,
		   },

};

/*****************************************************************************
 * Function
 *****************************************************************************/


static int _lcm_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	pr_debug("[LCM][I2C] NT: info==>name=%s addr=0x%x\n",
		client->name, client->addr);
	_lcm_i2c_client = client;
	return 0;
}


static int _lcm_i2c_remove(struct i2c_client *client)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	_lcm_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}


static int _lcm_i2c_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = _lcm_i2c_client;
	char write_data[2] = { 0 };

	if (client == NULL) {
		pr_debug("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		pr_info("[LCM][ERROR] _lcm_i2c write data fail !!\n");

	return ret;
}

/*
 * module load/unload record keeping
 */
static int __init _lcm_i2c_init(void)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	i2c_add_driver(&_lcm_i2c_driver);
	pr_debug("[LCM][I2C] %s success\n", __func__);
	return 0;
}

static void __exit _lcm_i2c_exit(void)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	i2c_del_driver(&_lcm_i2c_driver);
}

module_init(_lcm_i2c_init);
module_exit(_lcm_i2c_exit);
/***********************************/

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

#define tm_dcs_write_seq(ctx, seq...)                                     \
	({                                                                     \
		const u8 d[] = {seq};                                          \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		tm_dcs_write(ctx, d, ARRAY_SIZE(d));                      \
	})

#define tm_dcs_write_seq_static(ctx, seq...)                              \
	({                                                                     \
		static const u8 d[] = {seq};                                   \
		tm_dcs_write(ctx, d, ARRAY_SIZE(d));                      \
	})

static inline struct tianma *panel_to_tm(struct drm_panel *panel)
{
	return container_of(panel, struct tianma, panel);
}

#ifdef PANEL_SUPPORT_READBACK
static int tm_dcs_read(struct tianma *ctx, u8 cmd, void *data, size_t len)
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

static void tm_panel_get_data(struct tianma *ctx)
{
	u8 buffer[3] = {0};
	static int ret;
	pr_info("%s+\n", __func__);

	if (ret == 0) {
		ret = tm_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("%s  0x%08x\n", __func__,buffer[0] | (buffer[1] << 8));
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static void tm_dcs_write(struct tianma *ctx, const void *data, size_t len)
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

static void tm_panel_init(struct tianma *ctx)
{
	tm_dcs_write_seq_static(ctx, 0xB9, 0x83,0x11,0x2F);
	tm_dcs_write_seq_static(ctx, 0xCE, 0x00,0x10,0xE0);
	tm_dcs_write_seq_static(ctx, 0xBD, 0x03);
	tm_dcs_write_seq_static(ctx, 0xE4, 0x8F,0x80,0xAA,0xA5,0xAB,0xB0,0xC8,0xDE,0xFF,0xFF,0xFF,0x03,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x55,0x55,0x01,0x00,0x24,0x28,0x32,0x47,0x50,0x8D,0xB6,0xFC,0xFF,0xFF,0x03);
	tm_dcs_write_seq_static(ctx, 0xBD, 0x00);
	tm_dcs_write_seq_static(ctx, 0x35, 0x00);
	tm_dcs_write_seq_static(ctx, 0x11);
	msleep(60);
	tm_dcs_write_seq_static(ctx, 0x29);
	msleep(20);
	tm_dcs_write_seq_static(ctx, 0xBD, 0x02);
	tm_dcs_write_seq_static(ctx, 0xBF, 0xC0, 0x40);
	tm_dcs_write_seq_static(ctx, 0xBD, 0x00);
	tm_dcs_write_seq_static(ctx, 0x55, 0x01);
	msleep(10);
	tm_dcs_write_seq_static(ctx, 0x53, 0x24);
	pr_info("%s hx-\n", __func__);
}

static int tm_disable(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tm(panel);

	if (!ctx->enabled)
		return 0;
	pr_info("%s\n", __func__);
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	usleep_range(20 * 1000, 20 * 1000);
	ctx->enabled = false;

	return 0;
}

static int tm_unprepare(struct drm_panel *panel)
{

	struct tianma *ctx = panel_to_tm(panel);
	pr_info("%s hx\n", __func__);

	if (!ctx->prepared)
		return 0;

	tm_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	usleep_range(35 * 1000, 35 * 1000);
	tm_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(80 * 1000, 80 * 1000);

	/*ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);*/

#ifdef OPLUS_BUG_STABILITY
    pr_info("%s: tp_gesture_enable_flag = %d \n", __func__, tp_gesture_enable_flag());
    if ((0 == tp_gesture_enable_flag())||(esd_flag == 1)) {
#endif /* OPLUS_BUG_STABILITY */
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
        usleep_range(2 * 1000, 2 * 1000);
#ifdef OPLUS_BUG_STABILITY
    }
#endif /* OPLUS_BUG_STABILITY */


	ctx->error = 0;
	ctx->prepared = false;

	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tm(panel);
	int ret;

	pr_info("%s+\n", __func__);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(2000);
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	_lcm_i2c_write_bytes(0x0, 0xf);
	_lcm_i2c_write_bytes(0x1, 0xf);

	pr_info("%s-\n", __func__);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	return 0;
}

static int tm_prepare(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tm(panel);
	int ret;

	pr_info("%s hx+\n", __func__);
	if (ctx->prepared)
		return 0;

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(30);

#ifdef OPLUS_BUG_STABILITY
	lcd_queue_load_tp_fw();
#endif /* OPLUS_BUG_STABILITY */

	tm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		tm_unprepare(panel);

	ctx->prepared = true;

#ifdef PANEL_SUPPORT_READBACK
	tm_panel_get_data(ctx);
#endif

	pr_info("%s-\n", __func__);
	return ret;
}

static int tm_enable(struct drm_panel *panel)
{
	struct tianma *ctx = panel_to_tm(panel);

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
	.clock = 328022,
	.hdisplay = 1080,
	.hsync_start = 1080 + 160,//HFP
	.hsync_end = 1080 + 160 + 90,//HSA
	.htotal = 1080 + 160 + 90 + 140,//HBP
	.vdisplay = 2400,
	.vsync_start = 2400 + 1330,//VFP
	.vsync_end = 2400 + 1330 + 15,//VSA
	.vtotal = 2400 + 1330 + 15 + 15,//VBP
	.vrefresh = 60,
};

static const struct drm_display_mode performance_mode = {
	.clock = 328197,
	.hdisplay = 1080,
	.hsync_start = 1080 + 160,//HFP
	.hsync_end = 1080 + 160 + 90,//HSA
	.htotal = 1080 + 160 + 90 + 140,//HBP
	.vdisplay = 2400,
	.vsync_start = 2400 + 78,//VFP
	.vsync_end = 2400 + 78 + 15,//VSA
	.vtotal = 2400 + 78 + 15 + 15,//VBP
	.vrefresh = 90,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.pll_clk = 553,
	.vfp_low_power = 2580,//45hz
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
	.cmd = 0x0A, .count = 1, .para_list[0] = 0x1D, .mask_list[0] = 0x1D,
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
		.slice_height = 8,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 43,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 3255,
		.initial_offset = 6144,
		.final_offset = 7072,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		},
	.data_rate = 1107,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 90,
	},
	.tp_lcd_suspend = 1,
	.vendor = "HX83112F_TM",
	.manufacture = "hx_tm_4096",
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 553,
	.vfp_low_power = 1330,//60hz
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
	.cmd = 0x0A, .count = 1, .para_list[0] = 0x1D, .mask_list[0] = 0x1D,
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
		.slice_height = 8,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 43,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 3255,
		.initial_offset = 6144,
		.final_offset = 7072,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		},
	.data_rate = 1107,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 90,
	},
	.tp_lcd_suspend = 1,
	.vendor = "HX83112F_TM",
	.manufacture = "hx_tm_4096",
};

static int tm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0x07, 0xff};
	pr_err("%s backlight = %d\n", __func__, level);
	if (level > 4095)
		level = 4095;

	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;
	esd_brightness = level;
	if (!cb)
		return -1;

	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
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
static int mtk_panel_ext_param_set(struct drm_panel *panel, unsigned int mode)
{
  struct mtk_panel_ext *ext = find_panel_ext(panel);
  int ret = 0;
  struct drm_display_mode *m = get_mode_by_id(panel, mode);
  pr_info("%s: mode is %d, refresh is %d\n", __func__, mode,m->vrefresh);

  if (m->vrefresh == 60)
          ext->params = &ext_params;
  else if (m->vrefresh == 90)
          ext->params = &ext_params_90hz;
  //else if (m->vrefresh == 120)
          //ext->params = &ext_params_120hz;
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
	else if (mode == 1)
		ext_para = &ext_params_90hz;
	else
		ret = 1;

	return ret;

}

static void cabc_switch(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int cabc_mode)
{
	char bl_tb1[] = {0x55, 0x00};

	pr_err("%s cabc = %d\n", __func__, cabc_mode);
	if(cabc_mode == 2)
		return;

	if (cabc_mode == 1 || cabc_mode == 3 || cabc_mode == 0) {
		bl_tb1[1] = cabc_mode;
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	}
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct tianma *ctx = panel_to_tm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = tm_setbacklight_cmdq,
	.esd_backlight_check = oplus_esd_backlight_check,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
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

static int jdi_get_modes(struct drm_panel *panel)
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

	panel->connector->display_info.width_mm = 68;
	panel->connector->display_info.height_mm = 150;

	return 1;
}

static const struct drm_panel_funcs jdi_drm_funcs = {
	.disable = tm_disable,
	.unprepare = tm_unprepare,
	.prepare = tm_prepare,
	.enable = tm_enable,
	.get_modes = jdi_get_modes,
};

static int jdi_probe(struct mipi_dsi_device *dsi)
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
	pr_info("hx %s+\n", __func__);
	ctx = devm_kzalloc(dev, sizeof(struct tianma), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
			 |MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

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
	ctx->panel.funcs = &jdi_drm_funcs;

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
#ifndef OPLUS_BUG_STABILITY
        ctx->is_normal_mode = true;
        if( META_BOOT == get_boot_mode() || FACTORY_BOOT == get_boot_mode() )
            ctx->is_normal_mode = false;
        pr_info("%s: is_normal_mode = %d \n", __func__, ctx->is_normal_mode);
#endif /* OPLUS_BUG_STABILITY */

	register_device_proc("lcd", "HX83112F_TM", "hx_tm_4096");
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	pr_info("%s-\n", __func__);

	return ret;
}

static int jdi_remove(struct mipi_dsi_device *dsi)
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

static const struct of_device_id jdi_of_match[] = {
	{
		.compatible = "tianma,hx83112f,dphy,vdo,58",
	},
	{} };

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
	.probe = jdi_probe,
	.remove = jdi_remove,
	.driver = {
			.name = "hx83112f_fhdp_dsi_vdo_dphy_tianma_58_lcm_drv",
			.owner = THIS_MODULE,
			.of_match_table = jdi_of_match,
		},
};

module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("Elon Hsu <elon.hsu@mediatek.com>");
MODULE_DESCRIPTION("jdi r66451 CMD AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
