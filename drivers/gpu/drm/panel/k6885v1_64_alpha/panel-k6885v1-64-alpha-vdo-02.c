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

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_panel_ext.h"
#include "../mediatek/mtk_log.h"
#include "../mediatek/mtk_drm_graphics_base.h"
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif

#define HSA 14
#define HBP 22
#define HFP 48

#define VSA 2
#define VBP 9
#define VFP 21

#define LCM_DSI_CMD_MODE 0
#define REGFLAG_CMD       0xFFFA
#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
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
};

static char bl_tb0[] = {0x51, 0x03, 0xFF};

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
	DDPINFO("%s +\n", __func__);
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(15);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(1);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(10);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

#if (LCM_DSI_CMD_MODE)
	DDPINFO("%s\n", __func__);

	lcm_dcs_write_seq_static(ctx, 0x28);
    msleep(10);
	lcm_dcs_write_seq_static(ctx, 0x10);
    msleep(150);
    lcm_dcs_write_seq_static(ctx, 0x11);
    msleep(15);

    /* MIPI Mode cmd */
    lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A,0x5A);
    msleep(2);
    lcm_dcs_write_seq_static(ctx, 0xF2, 0x03);   
    msleep(2);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5,0xA5);
    msleep(2);

    /* TE vsync ON */
    lcm_dcs_write_seq_static(ctx, 0x35);
    msleep(12);
    
    /* AOD Setting */
    lcm_dcs_write_seq_static(ctx, 0x81,0x3F,0x0F,0x06,0x09,0xF2,0x6B,

					 0x15,0xE4,0x29,0xD3,0x15,

					 0x0F,0x06,0x32,0x93,0xD3,

					 0x15,0xE4,0x3E,0x24,0x19,

					 0x00,0x00,0x00,0x00,0x00,

					 0x1B,0x82,0x79,0xD8,0x65,

			    	 0x60,0x01,0x10,0x00,0x00,

			    	 0x00,0xFF,0xFF,0xFF,0x00,

			    	 0x00,0xFF,0x00);
    msleep(2);    

    /* AOD AMP ON */
    lcm_dcs_write_seq_static(ctx, 0xFC,0x5A,0x5A);
    msleep(2);
    lcm_dcs_write_seq_static(ctx, 0xB0,0x05,0xFD);
    msleep(2);
    lcm_dcs_write_seq_static(ctx, 0xFD,0xEF);
    msleep(2);
    lcm_dcs_write_seq_static(ctx, 0xFC,0xA5,0xA5);
    msleep(2);    

    
    /* AOD Mode On Setting */
    lcm_dcs_write_seq_static(ctx, 0x53,0x22);
    msleep(110);    
   
    /* Image Data Write for AOD Mode */
    /* Display on */
    lcm_dcs_write_seq_static(ctx, 0x29);
    msleep(2);     
  
    /* MIPI Video cmd*/
    lcm_dcs_write_seq_static(ctx, 0xF0,0x5A,0x5A);
    msleep(2);     
    lcm_dcs_write_seq_static(ctx, 0xF2,0x0F);
    msleep(2);    
    lcm_dcs_write_seq_static(ctx, 0xF0,0xA5,0xA5);
    msleep(2); 
	
#else

	lcm_dcs_write_seq_static(ctx, 0x9F, 0xA5, 0xA5);
	lcm_dcs_write_seq_static(ctx, 0x11);
	msleep(20);
	lcm_dcs_write_seq_static(ctx, 0x9F, 0x5A, 0x5A);

	lcm_dcs_write_seq_static(ctx, 0x9F, 0xA5, 0xA5);
	lcm_dcs_write_seq_static(ctx, 0x35, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x9F, 0x5A, 0x5A);

	lcm_dcs_write_seq_static(ctx, 0xFC, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xED, 0x00, 0x01, 0x00, 0x40, 0x04, 0x08, 0xA8, 0x84, 0x4A, 0x73, 0x02, 0x0A);
	lcm_dcs_write_seq_static(ctx, 0xFC, 0xA5, 0xA5);

	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x05);
	lcm_dcs_write_seq_static(ctx, 0xB3, 0x87);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	lcm_dcs_write_seq_static(ctx, 0x53, 0x20);
	lcm_dcs_write_seq_static(ctx, 0x55, 0x00);
	msleep(100);
	lcm_dcs_write_seq_static(ctx, 0x9F, 0xA5, 0xA5);
	lcm_dcs_write_seq_static(ctx, 0x29);

#endif

}

#define sleep_ms 2
static void lcm_panel_init_cmd(struct lcm *ctx)
{
	DDPINFO("%s +\n", __func__);
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(2);
	gpiod_set_value(ctx->reset_gpio, 1);
#if 0
	msleep(1);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(10);
#endif
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	lcm_dcs_write_seq_static(ctx, 0x28);
    msleep(sleep_ms);
	/*Sleep in*/
	lcm_dcs_write_seq_static(ctx, 0x10);
    msleep(sleep_ms);

	/* Internal VDO Packet generation enable*/
    lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A,0x5A);
    msleep(sleep_ms);
	lcm_dcs_write_seq_static(ctx, 0xFC, 0x5A,0x5A);
    msleep(sleep_ms);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x14,0xFE);
    msleep(sleep_ms);
    lcm_dcs_write_seq_static(ctx, 0xFE, 0x12);   
   	msleep(sleep_ms);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5,0xA5);
    msleep(sleep_ms);
	lcm_dcs_write_seq_static(ctx, 0xFC, 0xA5,0xA5);
    msleep(sleep_ms);

	/*Sleep out*/
    lcm_dcs_write_seq_static(ctx, 0x11);
    msleep(sleep_ms);

    /* MIPI Mode cmd */
    lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A,0x5A);
    msleep(sleep_ms);
    lcm_dcs_write_seq_static(ctx, 0xF2, 0x03);   
   	msleep(sleep_ms);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5,0xA5);
    msleep(sleep_ms);

    /* TE vsync ON */
    lcm_dcs_write_seq_static(ctx, 0x35);
    msleep(sleep_ms);
    
    /* AOD Setting */
    lcm_dcs_write_seq_static(ctx, 0x81,0x3F,0x0F,0x06,0x09,0xF2,0x6B,
					 0x15,0xE4,0x29,0xD3,0x15,
					 0x0F,0x06,0x32,0x93,0xD3,
					 0x15,0xE4,0x3E,0x24,0x19,
					 0x00,0x00,0x00,0x00,0x00,
					 0x1B,0x82,0x79,0xD8,0x65,
			    	 0x60,0x01,0x10,0x00,0x00,
			    	 0x00,0xFF,0xFF,0xFF,0x00,
			    	 0x00,0xFF,0x00);
    msleep(sleep_ms);    

    /* AOD AMP ON */
    lcm_dcs_write_seq_static(ctx, 0xFC,0x5A,0x5A);
    msleep(sleep_ms);
    lcm_dcs_write_seq_static(ctx, 0xB0,0x05,0xFD);
    msleep(sleep_ms);
    lcm_dcs_write_seq_static(ctx, 0xFD,0xEF);
    msleep(sleep_ms);
    lcm_dcs_write_seq_static(ctx, 0xFC,0xA5,0xA5);
    msleep(sleep_ms);    

    
    /* AOD Mode On Setting */
    lcm_dcs_write_seq_static(ctx, 0x53,0x22);
    msleep(sleep_ms);   

	/* Internal VDO Packet generation enable*/
    lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A,0x5A);
    msleep(sleep_ms);
	lcm_dcs_write_seq_static(ctx, 0xFC, 0x5A,0x5A);
    msleep(sleep_ms);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x14,0xFE);
    msleep(sleep_ms);
    lcm_dcs_write_seq_static(ctx, 0xFE, 0x10);   
   	msleep(sleep_ms);
    lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5,0xA5);
    msleep(sleep_ms);
	lcm_dcs_write_seq_static(ctx, 0xFC, 0xA5,0xA5);
    msleep(sleep_ms);
   
    /* Image Data Write for AOD Mode */
    /* Display on */
    lcm_dcs_write_seq_static(ctx, 0x29);
    msleep(sleep_ms);     
  
    /* MIPI Video cmd*/
    lcm_dcs_write_seq_static(ctx, 0xF0,0x5A,0x5A);
   	msleep(sleep_ms);     
    lcm_dcs_write_seq_static(ctx, 0xF2,0x0F);
    msleep(sleep_ms);    
    lcm_dcs_write_seq_static(ctx, 0xF0,0xA5,0xA5);
    msleep(sleep_ms); 

	DDPINFO("%s +\n", __func__);

}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	
	DDPINFO("%s\n", __func__);
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
	
	DDPINFO("%s\n", __func__);
	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(10);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(150);

	ctx->error = 0;
	ctx->prepared = false;
#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_disable();
#endif
	ctx->bias_gpio =
	devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_gpio, 0);
	msleep(15);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);

	ctx->hbm_en = false;

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;
	
	DDPINFO("%s\n", __func__);
	pr_info("%s\n", __func__);
	if (ctx->prepared)
		return 0;

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_enable();
#endif
	ctx->bias_gpio =
	devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	lcm_panel_init(ctx);

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
	
	DDPINFO("%s\n", __func__);
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
	.clock = 179222,
	.hdisplay = 1080,
	.hsync_start = 1080 + HFP,
	.hsync_end = 1080 + HFP + HSA,
	.htotal = 1080 + HFP + HSA + HBP,
	.vdisplay = 2400,
	.vsync_start = 2400 + VFP,
	.vsync_end = 2400 + VFP + VSA,
	.vtotal = 2400 + VFP + VSA + VBP,
	.vrefresh = 60,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int lcm_setbacklight_cmdq(void *dsi,
		dcs_write_gce cb, void *handle, unsigned int level)
{
	bl_tb0[1] = level * 4 >> 8;
	bl_tb0[2] = level * 4 & 0xFF;

	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
			      dcs_write_gce cb, void *handle, bool en)
{
	char hbm_tb[] = {0x53, 0xe0};
	struct lcm *ctx = panel_to_lcm(panel);

	if (!cb)
		return -1;

	if (ctx->hbm_en == en)
		goto done;

	if (en)
		hbm_tb[1] = 0xe0;
	else
		hbm_tb[1] = 0x20;

	cb(dsi, handle, hbm_tb, ARRAY_SIZE(hbm_tb));

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
	.data_rate = 1098,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x00,
		.count = 1,
		.para_list[0] = 0x24,
	},
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.hbm_en_time = 2,
	.hbm_dis_time = 1,
};

static unsigned long panel_doze_get_mode_flags(struct drm_panel *panel, int doze_en)
{
	unsigned long mode_flags;

	DDPINFO("%s doze_en:%d\n", __func__, doze_en);
	if (doze_en) {
		mode_flags = MIPI_DSI_MODE_LPM
		       | MIPI_DSI_MODE_EOT_PACKET
		       | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	} else {
		mode_flags = MIPI_DSI_MODE_VIDEO
		       | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
		       | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
		       | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	}
	return mode_flags;
}


static int panel_doze_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	DDPINFO("%s\n", __func__);

	/* Switch to CMD mode */
	//lcm_normal_to_aod(ctx);
	lcm_panel_init_cmd(ctx);
}

static void lcm_aod_to_normal(struct lcm *ctx)
{
	/* Display off */
	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(33);

	/* AOD AMP Off */		    	 
	lcm_dcs_write_seq_static(ctx, 0xFC,0x5A,0x5A);
	 msleep(2);
    lcm_dcs_write_seq_static(ctx, 0xB0,0x05,0xFD);
	 msleep(2);
	lcm_dcs_write_seq_static(ctx, 0xFD,0x6F);
	 msleep(2);
	lcm_dcs_write_seq_static(ctx, 0xFC,0xA5,0xA5);
	 msleep(2);
  
    /* AOD Mode off Setting */
	lcm_dcs_write_seq_static(ctx, 0x53,0xE0);
	 msleep(2);
	/* Display on */
	lcm_dcs_write_seq_static(ctx, 0x29);
	 msleep(2);

};

static int panel_doze_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	DDPINFO("%s\n", __func__);

	/* Switch back to VDO mode */
	lcm_aod_to_normal(ctx);
	//lcm_panel_init(ctx);
	//lcm_display_on(ctx);
}

static int panel_doze_area(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	DDPINFO("%s\n", __func__);
	//lcm_noramal_to_aod(ctx);
	lcm_panel_init_cmd(ctx);
#if 0
	lcm_dcs_write_seq_static(ctx, 0x81,0x3F,0x0F,0x06,0x09,0xF2,0x6B,
					 0x15,0xE4,0x29,0xD3,0x15,
					 0x0F,0x06,0x32,0x93,0xD3,
					 0x15,0xE4,0x3E,0x24,0x19,
					 0x00,0x00,0x00,0x00,0x00,
					 0x1B,0x82,0x79,0xD8,0x65,
			    	 0x60,0x01,0x10,0x00,0x00,
			    	 0x00,0xFF,0xFF,0xFF,0x00,
			    	 0x00,0xFF,0x00);
    msleep(2);    
#endif
	return 0;
}
static int panel_doze_area_set(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	DDPINFO("%s\n", __func__);
	//lcm_noramal_to_aod(ctx);
	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(2);
#if 1
	lcm_dcs_write_seq_static(ctx, 0x81,0x3F,0x0F,0x06,0x09,0xF2,0x6B,
					 0x15,0xE4,0x29,0xD3,0x15,
					 0x0F,0x06,0x32,0x93,0xD3,
					 0x15,0xE4,0x3E,0x24,0x19,
					 0x00,0x00,0x00,0x00,0x00,
					 0x1B,0x82,0x79,0xD8,0x65,
			    	 0x60,0x01,0x10,0x00,0x00,
			    	 0x00,0xFF,0xFF,0xFF,0x00,
			    	 0x00,0xFF,0x00);
    msleep(2);    
#endif
	lcm_dcs_write_seq_static(ctx, 0x29);
	msleep(2);

	return 0;
}

static int panel_doze_post_disp_on(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	DDPINFO("%s\n", __func__);
#if 0
	//lcm_noramal_to_aod(ctx);
	//lcm_panel_init_cmd(ctx);
	//lcm_display_on(ctx);
	lcm_dcs_write_seq_static(ctx, 0x29);
    msleep(2);    
#endif
	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.hbm_get_state = panel_hbm_get_state,
	.hbm_get_wait_state = panel_hbm_get_wait_state,
	.hbm_set_wait_state = panel_hbm_set_wait_state,
	.doze_disable = panel_doze_disable,
	.doze_area = panel_doze_area,
	.doze_area_set = panel_doze_area_set,
	.doze_post_disp_on = panel_doze_post_disp_on,
	.doze_get_mode_flags = panel_doze_get_mode_flags,
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

	panel->connector->display_info.width_mm = 71;
	panel->connector->display_info.height_mm = 153;

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
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
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
	{ .compatible = "s68fc01,lcm", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "s68fc01,lcm",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);
