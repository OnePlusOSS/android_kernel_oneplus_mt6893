// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <linux/of_graph.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <soc/oplus/device_info.h>


#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_panel_ext.h"
#include "../mediatek/mtk_drm_graphics_base.h"
#endif

#include "../mediatek/mtk_corner_pattern/oplus22612_roma_data_hw_roundedpattern.h"

#define REGFLAG_CMD       0xFFFA
#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
/* add for ofp */
#include "../oplus/oplus_display_onscreenfingerprint.h"
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
#include <mt-plat/mtk_boot_common.h>


#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   3515
#define DC_BAKLIGHT_THERSHOLD   2313
//extern int oplus_dc_alpha;
//extern int oplus_dc_enable_real;
#define LCM_BRIGHTNESS_TYPE 2
static unsigned int esd_brightness = 1023;
static u32 flag_hbm = 0;
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_display_backlight;
static bool aod_state = false;
extern bool aod_flag;
extern void disp_aal_set_dre_en(int enable);
extern unsigned int oplus_enhance_mipi_strength;
//extern bool g_is_silky_panel;
extern bool is_forbid_enter_ulps_aod_state;
//extern bool pq_trigger;
//extern bool enter_dc_flag;
static bool panel_init = false;
static bool aod_after_panel_init = false;

struct LCM_setting_table {
  	unsigned int cmd;
 	unsigned char count;
 	unsigned char para_list[128];
};

static struct LCM_setting_table lcm_setbrightness_normal[] = {
	{REGFLAG_CMD,3, {0x51, 0x00, 0x00}},
};

static struct LCM_setting_table lcm_finger_HBM_on_setting[] = {
        {REGFLAG_CMD,3, {0x51, 0x0f, 0xff}},
};


static struct LCM_setting_table lcm_aod_to_normal[] = {
	{REGFLAG_CMD,4, {0xFF, 0x78, 0x38, 0x00}},
	{REGFLAG_CMD,1, {0x38}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_high_mode[] = {
	{REGFLAG_CMD,4, {0xFF, 0x78, 0x38, 0x00}},
	{REGFLAG_CMD,1, {0x39}},
	{REGFLAG_CMD,4, {0xFF, 0x78, 0x38, 0x0C}},
	{REGFLAG_CMD,2, {0xBE, 0x01}},
	{REGFLAG_CMD,4, {0xFF, 0x78, 0x38, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_low_mode[] = {
        {REGFLAG_CMD,4, {0xFF, 0x78, 0x38, 0x00}},
        {REGFLAG_CMD,1, {0x39}},
        {REGFLAG_CMD,4, {0xFF, 0x78, 0x38, 0x0C}},
        {REGFLAG_CMD,2, {0xBE, 0x00}},
	{REGFLAG_CMD,4, {0xFF, 0x78, 0x38, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

struct lcm_pmic_info {
	struct regulator *reg_vufs18;
	struct regulator *reg_vmch3p0;
};

struct jdi {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;
	struct gpio_desc *bias_gpio;
	struct gpio_desc *vddr1p5_enable_gpio;
	struct gpio_desc *vddr_aod_enable_gpio;
	struct gpio_desc *te_switch_gpio,*te_out_gpio;
	struct gpio_desc *pw_1p8_gpio, *pw_reset_gpio;
	bool prepared;
	bool enabled;

	int error;
};

#define jdi_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = { seq };                                        \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128,                          \
				 "DCS sequence too big for stack");            \
		jdi_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define jdi_dcs_write_seq_static(ctx, seq...)                                  \
	({                                                                     \
		static const u8 d[] = { seq };                                 \
		jdi_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

static inline struct jdi *panel_to_jdi(struct drm_panel *panel)
{
	return container_of(panel, struct jdi, panel);
}

#ifdef PANEL_SUPPORT_READBACK
static int jdi_dcs_read(struct jdi *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void jdi_panel_get_data(struct jdi *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	pr_info("%s+\n", __func__);

	if (ret == 0) {
		ret = jdi_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("%s  0x%08x\n", __func__, buffer[0] | (buffer[1] << 8));
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static void jdi_dcs_write(struct jdi *ctx, const void *data, size_t len)
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
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}


static struct regulator *disp_ldo3;

static int lcm_panel_ldo3_regulator_init(struct device *dev)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;
    pr_err("get lcm_panel_ldo3_regulator_init\n");

	/* please only get regulator once in a driver */
	//disp_ldo3 = regulator_get(dev, "VMC");
	disp_ldo3 = regulator_get(dev, "ldo3");
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


static void jdi_panel_init(struct jdi *ctx)
{
	//APL Code
        jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x0B);
        jdi_dcs_write_seq_static(ctx,0x17,0xF3);
        jdi_dcs_write_seq_static(ctx,0x18,0xF3);
        jdi_dcs_write_seq_static(ctx,0x19,0xF3);
        jdi_dcs_write_seq_static(ctx,0x1A,0xF3);
        jdi_dcs_write_seq_static(ctx,0x1B,0xF3);
        jdi_dcs_write_seq_static(ctx,0x1C,0xF3);
        jdi_dcs_write_seq_static(ctx,0x1D,0xF3);
        jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x0A);
        jdi_dcs_write_seq_static(ctx,0x37,0x4F);
        jdi_dcs_write_seq_static(ctx,0x38,0x70);
        jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x00);
        jdi_dcs_write_seq_static(ctx,0x55,0x03);

	jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x08);
	jdi_dcs_write_seq_static(ctx,0x45,0x4C);
	jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x02);
	jdi_dcs_write_seq_static(ctx,0x38,0x13);   //60Hz
	jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x07);
	jdi_dcs_write_seq_static(ctx,0x29,0x01);
	jdi_dcs_write_seq_static(ctx,0x20,0x00,0x00,0x00,0x00,0x00,0x11,0x00,0x00,0xab,0x30,0x80,0x09,0x6c,0x04,0x38,0x00,0x0c,0x02,0x1c,0x02,0x1c,0x02,0x00,0x02,0x0e,0x00,0x20,0x01,0x1f,0x00,0x07,0x00,0x0c,0x08,0xbb,0x08,0x7a,0x18,0x00,0x10,0xf0,0x07,0x10,0x20,0x00,0x06,0x0f,0x0f,0x33,0x0e,0x1c,0x2a,0x38,0x46,0x54,0x62,0x69,0x70,0x77,0x79,0x7b,0x7d,0x7e,0x02,0x02,0x22,0x00,0x2a,0x40,0x2a,0xbe,0x3a,0xfc,0x3a,0xfa,0x3a,0xf8,0x3b,0x38,0x3b,0x78,0x3b,0xb6,0x4b,0xb6,0x4b,0xf4,0x4b,0xf4,0x6c,0x34,0x84,0x74,0x00,0x00,0x00,0x00,0x00,0x00);
	jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x00);
	jdi_dcs_write_seq_static(ctx,0x95,0x10);
	jdi_dcs_write_seq_static(ctx,0x51,0x00,0x00);
	jdi_dcs_write_seq_static(ctx,0x53,0x20);
	jdi_dcs_write_seq_static(ctx,0x35,0x00);
	jdi_dcs_write_seq_static(ctx,0x11,0x00);
	msleep(120);
	jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x08);
	jdi_dcs_write_seq_static(ctx,0x57,0x25);
	jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x06);
	jdi_dcs_write_seq_static(ctx,0xC6,0x01);
	jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x17);
	jdi_dcs_write_seq_static(ctx,0x20,0x00);
	jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x00);
	jdi_dcs_write_seq_static(ctx,0x29,0x00);

	panel_init = true;
	pr_info("SYQ %s-\n", __func__);
}

static int jdi_disable(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}
	ctx->enabled = false;

	return 0;
}

static int jdi_unprepare(struct drm_panel *panel)
{

	struct jdi *ctx = panel_to_jdi(panel);

	if (!ctx->prepared)
		return 0;

	pr_err("[lh]%s+\n", __func__);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	msleep(15);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	msleep(125);

    /* keep vcore off */
	//lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_SUSPEND_PWR_CTRL, MT_LPM_SMC_ACT_SET, PW_REG_SPM_VCORE_REQ, 0x0);
	pr_info("%s, call lpm_smc_spm_dbg keep vcore off for display off!\n", __func__);

	ctx->error = 0;
	ctx->prepared = false;
	pr_err("[lh]%s-\n", __func__);

	return 0;
}

static int jdi_prepare(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_err("[lh]%s+\n", __func__);

	msleep(10);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(5);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(5);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(15);
	//msleep(10);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	jdi_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		jdi_unprepare(panel);

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	jdi_panel_get_data(ctx);
#endif

	pr_info("%s-\n", __func__);
	return ret;
}

static int jdi_enable(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

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
	.clock = 166800,
	.hdisplay = 1080,
	.hsync_start = 1080 + 9,//HFP
	.hsync_end = 1080 + 9 + 2,//HSA
	.htotal = 1080 + 9 + 2 + 21,//HBP
	.vdisplay = 2412,
	.vsync_start = 2412 + 52,//VFP
	.vsync_end = 2412 + 52 + 14,//VSA
	.vtotal = 2412 + 52+ 14 + 22,//VBP
};

static const struct drm_display_mode performance_mode_90hz = {
	.clock = 250200,
	.hdisplay = 1080,
	.hsync_start = 1080 + 9,//HFP
	.hsync_end = 1080 + 9 + 2,//HSA
	.htotal = 1080 + 9 + 2 + 21,//HBP
	.vdisplay = 2412,
	.vsync_start = 2412 + 52,//VFP
	.vsync_end = 2412 + 52 + 14,//VSA
	.vtotal = 2412 + 52+ 14 + 22,//VBP
};

static const struct drm_display_mode performance_mode_120hz = {
	.clock = 333600,
	.hdisplay = 1080,
	.hsync_start = 1080 + 9,//HFP
	.hsync_end = 1080 + 9 + 2,//HSA
	.htotal = 1080 + 9 + 2 + 21,//HBP
	.vdisplay = 2412,
	.vsync_start = 2412 + 52,//VFP
	.vsync_end = 2412 + 52 + 14,//VSA
	.vtotal = 2412 + 52+ 14 + 22,//VBP
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.pll_clk = 209,
	.phy_timcon = {
	    .clk_trail = 5,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_check_multi = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0xDC,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        .round_corner_en = 1,
        .corner_pattern_height = ROUND_CORNER_H_TOP,
        .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
        .corner_pattern_tp_size = sizeof(top_rc_pattern),
        .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
        .vendor = "tian_ma_t7_plus_roma",
        .manufacture = "tian_ma_t7_plus",
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.dsc_params = {
        .enable = 1,
        .ver = 17,
        .slice_mode = 1,
        .rgb_swap = 0,
        .dsc_cfg = 40,
        .rct_on = 1,
        .bit_per_channel = 10,
        .dsc_line_buf_depth = 11,
        .bp_enable = 1,
        .bit_per_pixel = 128,
        .pic_height = 2412,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 512,
        .dec_delay = 526,
        .scale_value = 32,
        .increment_interval = 287,
        .decrement_interval = 7,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2170,
        .initial_offset = 6144,
        .final_offset = 4336,
        .flatness_minqp = 7,
        .flatness_maxqp = 16,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 15,
        .rc_quant_incr_limit1 = 15,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
		},
	.data_rate = 418,
        .color_vivid_status = true,
        .color_srgb_status = true,
        .color_softiris_status = true,
        .color_dual_panel_status = false,
        .color_dual_brightness_status = true,
#ifdef CONFIG_OPLUS_OFP_V2
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 0,
	.oplus_ofp_hbm_off_delay = 17,
#endif
	.oplus_osc_hoping_fps_switch = true,
	//.oplus_wait_te = 0,
	.oplus_uiready_before_time = 17,
	.oplus_display_global_dre = 1,
	.backlight_dsiable_threhold = 3000,
	.oplus_custom_hdr_color_tmp = true,
        .oplus_custom_hdr_red = 989,
        .oplus_custom_hdr_green = 1024,
        .oplus_custom_hdr_blue = 810,
	.oplus_panel_use_rgb_gain = true,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 60,
	},
        .cmd_null_pkt_en = 1,
        .cmd_null_pkt_len = 105,
	.oplus_display_global_dre = 1,
	.oplus_ofp_mipi_switch_waite_frame = 1,
	.oplus_ofp_mipi_switch_config = 1,
	.oplus_bit_per_channel = 10,
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 417,
	.phy_timcon = {
            .clk_trail = 12,
        },
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_check_multi = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0xDC,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.vendor = "tian_ma_t7_plus_roma",
        .manufacture = "tian_ma_t7_plus",
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.dsc_params = {
        .enable = 1,
        .ver = 17,
        .slice_mode = 1,
        .rgb_swap = 0,
        .dsc_cfg = 40,
        .rct_on = 1,
        .bit_per_channel = 10,
        .dsc_line_buf_depth = 11,
        .bp_enable = 1,
        .bit_per_pixel = 128,
        .pic_height = 2412,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 512,
        .dec_delay = 526,
        .scale_value = 32,
        .increment_interval = 287,
        .decrement_interval = 7,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2170,
        .initial_offset = 6144,
        .final_offset = 4336,
        .flatness_minqp = 7,
        .flatness_maxqp = 16,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 15,
        .rc_quant_incr_limit1 = 15,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
		},
	.data_rate = 834,
        .color_vivid_status = true,
        .color_srgb_status = true,
        .color_softiris_status = true,
        .color_dual_panel_status = false,
        .color_dual_brightness_status = true,
#ifdef CONFIG_OPLUS_OFP_V2
	.oplus_ofp_need_keep_apart_backlight = true,
        .oplus_ofp_hbm_on_delay = 0,
        .oplus_ofp_pre_hbm_off_delay = 0,
        .oplus_ofp_hbm_off_delay = 12,
#endif
	.oplus_osc_hoping_fps_switch = true,
	//.oplus_wait_te = 0,
	.oplus_uiready_before_time = 12,
	.oplus_display_global_dre = 1,
	.backlight_dsiable_threhold = 3000,
	.oplus_custom_hdr_color_tmp = true,
        .oplus_custom_hdr_red = 989,
        .oplus_custom_hdr_green = 1024,
        .oplus_custom_hdr_blue = 810,
	.oplus_panel_use_rgb_gain = true,
        .dyn_fps = {
                .switch_en = 1, .vact_timing_fps = 90,
        },
        .cmd_null_pkt_en = 1,
        .cmd_null_pkt_len = 105,
	.oplus_display_global_dre = 1,
	.oplus_ofp_mipi_switch_waite_frame = 1,
	.oplus_ofp_mipi_switch_config = 1,
	.oplus_bit_per_channel = 10,
};

static struct mtk_panel_params ext_params_120hz = {
	.pll_clk = 417,
	.phy_timcon = {
            .clk_trail = 12,
        },
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_check_multi = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0xDC,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        .round_corner_en = 1,
        .corner_pattern_height = ROUND_CORNER_H_TOP,
        .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
        .corner_pattern_tp_size = sizeof(top_rc_pattern),
        .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.vendor = "tian_ma_t7_plus_roma",
        .manufacture = "tian_ma_t7_plus",
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.dsc_params = {
        .enable = 1,
        .ver = 17,
        .slice_mode = 1,
        .rgb_swap = 0,
        .dsc_cfg = 40,
        .rct_on = 1,
        .bit_per_channel = 10,
        .dsc_line_buf_depth = 11,
        .bp_enable = 1,
        .bit_per_pixel = 128,
        .pic_height = 2412,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 512,
        .dec_delay = 526,
        .scale_value = 32,
        .increment_interval = 287,
        .decrement_interval = 7,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2170,
        .initial_offset = 6144,
        .final_offset = 4336,
        .flatness_minqp = 7,
        .flatness_maxqp = 16,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 15,
        .rc_quant_incr_limit1 = 15,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
		},
	.data_rate = 834,
        .color_vivid_status = true,
        .color_srgb_status = true,
        .color_softiris_status = true,
        .color_dual_panel_status = false,
        .color_dual_brightness_status = true,
#ifdef CONFIG_OPLUS_OFP_V2
	.oplus_ofp_need_keep_apart_backlight = true,
        .oplus_ofp_hbm_on_delay = 0,
        .oplus_ofp_pre_hbm_off_delay = 2,
        .oplus_ofp_hbm_off_delay = 0,
#endif
	.oplus_osc_hoping_fps_switch = true,
	//.oplus_wait_te = 1,
	.oplus_uiready_before_time = 9,
	.oplus_display_global_dre = 1,
	.backlight_dsiable_threhold = 3000,
	.oplus_custom_hdr_color_tmp = true,
        .oplus_custom_hdr_red = 989,
        .oplus_custom_hdr_green = 1024,
        .oplus_custom_hdr_blue = 810,
	.oplus_panel_use_rgb_gain = true,
        .dyn_fps = {
                .switch_en = 1, .vact_timing_fps = 120,
        },
        .cmd_null_pkt_en = 1,
        .cmd_null_pkt_len = 105,
	.oplus_display_global_dre = 1,
	.oplus_ofp_mipi_switch_waite_frame = 1,
	.oplus_ofp_mipi_switch_config = 1,
	.oplus_bit_per_channel = 10,
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

#if defined(CONFIG_PXLW_IRIS)
static void iris_panel_commandq(void *dsi, dcs_write_gce cb,
			void *handle, char *src, int s_len)
{
	char dest[512] = {0,};
	int len = 0;
	len = iris_conver_one_panel_cmd(dest, src, s_len);
	if (len > 0)
		cb(dsi, handle, dest, len);
}
#endif

static int jdi_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	char bl_tb0[] = {0x51, 0x07, 0xFF};
//	char bl_tb1[] = {0xB2, 0x01};
//	char bl_tb2[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00};
	char bl_aod1[] = {0xFF,0x78,0x38,0x00};
	char bl_aod2[] = {0x13};

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0)
		level = 3515;
	//level = oplus_lcm_dc_backlight(dsi,cb,handle, level, 0);
	if (level > 1 && level <8)
        level = 8;
	if (level > 4095)
		level = 4095;
	pr_info("%s backlight = -%d\n", __func__, level);
	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;

	if (!cb)
		return -1;

	panel_init = false;
	if (level ==1) {
		cb(dsi, handle, bl_aod1, ARRAY_SIZE(bl_aod1));
		cb(dsi, handle, bl_aod2, ARRAY_SIZE(bl_aod2));
		cb(dsi, handle, bl_aod1, ARRAY_SIZE(bl_aod1));
		aod_after_panel_init = false;
		pr_err("enter aod!!!\n");
		return 0;
	}

//	if (level >2047)
//		bl_tb1[1] = 0x01;
//	else
//		bl_tb1[1] = 0x11;
	esd_brightness = level;

	if (aod_after_panel_init == true) {
		cb(dsi, handle, bl_aod1, ARRAY_SIZE(bl_aod1));
                cb(dsi, handle, bl_aod2, ARRAY_SIZE(bl_aod2));
                cb(dsi, handle, bl_aod1, ARRAY_SIZE(bl_aod1));
                aod_after_panel_init = false;
	}
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

//	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
//	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	oplus_display_brightness = level;

	return 0;
}

static void lcm_setbrightness(void *dsi,
			      dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int BL_MSB = 0;
	unsigned int BL_LSB = 0;
	int i = 0;

	printk("[lh] %s level is %d\n", __func__, level);
	        if (level > 1 && level < 8)
                	level = 8;
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

	if (!cb)
		return -1;

	pr_err("[lh]oplus_display_brightness= %ld, hbm_mode=%u\n", oplus_display_brightness, hbm_mode);

	if(hbm_mode == 1) {
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (hbm_mode == 0) {
		//level = oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 0);
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);  //level
		printk("[lh] %s : %d ! backlight %d !\n",__func__, hbm_mode, oplus_display_brightness);
	}

	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
			      dcs_write_gce cb, void *handle, bool en)
{
	//char hbm_tb[] = {0x53, 0xe0};
        char bl_aod1[] = {0xFF,0x78,0x38,0x00};
        char bl_aod2[] = {0x13};
	int i = 0;
	if (!cb)
		return -1;

	pr_err("debug for oplus_display_brightness= %ld, en=%u\n", oplus_display_brightness, en);

	panel_init = false;
	if (aod_after_panel_init == true) {
		cb(dsi, handle, bl_aod1, ARRAY_SIZE(bl_aod1));
                cb(dsi, handle, bl_aod2, ARRAY_SIZE(bl_aod2));
                cb(dsi, handle, bl_aod1, ARRAY_SIZE(bl_aod1));
                aod_after_panel_init = false;
	}
	if(en == 1) {
		//oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 1);
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (en == 0) {
		//level = oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 0);
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);
		printk("[soso] %s : %d ! backlight %d !\n",__func__, en, oplus_display_brightness);
		if (oplus_display_brightness <= BRIGHTNESS_HALF)
			flag_hbm = 0;
		else
			flag_hbm = 1;
	}
	//lcdinfo_notify(1, &en);
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
static int backlight_recovery_after_prepare(void *dsi, dcs_write_gce cb,
        void *handle)
{
/*
        char bl_tb0[] = {0x51, 0x03, 0xff};

        //pr_err("%s esd_backlight = %d\n", __func__, esd_brightness);
        bl_tb0[1] = oplus_display_backlight * 3515 / 8191 >> 8;
        bl_tb0[2] = oplus_display_backlight * 3515 / 8191 & 0xFF;
        if (!cb)
                return -1;
        pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
        cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
*/
        return 1;
}
static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct jdi *ctx = panel_to_jdi(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}
#if 0
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
#endif

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	//struct lcm *ctx = panel_to_lcm(panel);
	unsigned int i=0;
	pr_err("debug for lcm %s\n", __func__);

	/*if (oplus_fp_notify_down_delay)
		aod_finger_unlock_flag = 1;*/

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
       /* keep vcore on */
       //lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_SUSPEND_PWR_CTRL, MT_LPM_SMC_ACT_SET, PW_REG_SPM_VCORE_REQ, 0x0);
       pr_info("%s, call lpm_smc_spm_dbg keep vcore off for exit AOD!\n", __func__);

	aod_state = false;
	aod_flag = false;
	return 0;
}

static struct LCM_setting_table lcm_normal_to_aod_sam_panel_init[] = {
/*
   {REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x08}},
   {REGFLAG_CMD, 2, {0x45,0x4C}},
   {REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x07}},
   {REGFLAG_CMD, 2, {0x29,0x01}},
   {REGFLAG_CMD, 100, {0x20,0x00,0x00,0x00,0x00,0x00,0x11,0x00,0x00,0xab,0x30,0x80,0x09,0x6c,0x04,0x38,0x00,0x0c,0x02,0x1c,0x02,0x1c,0x02,0x00,0x02,0x0e,0x00,0x20,0x01,0x1f,0x00,0x07,0x00,0x0c,0x08,0xbb,0x08,0x7a,0x18,0x00,0x10,0xf0,0x07,0x10,0x20,0x00,0x06,0x0f,0x0f,0x33,0x0e,0x1c,0x2a,0x38,0x46,0x54,0x62,0x69,0x70,0x77,0x79,0x7b,0x7d,0x7e,0x02,0x02,0x22,0x00,0x2a,0x40,0x2a,0xbe,0x3a,0xfc,0x3a,0xfa,0x3a,0xf8,0x3b,0x38,0x3b,0x78,0x3b,0xb6,0x4b,0xf6,0x4c,0x34,0x4c,0x74,0x5c,0x74,0x8c,0xf4,0x00,0x00,0x00,0x00,0x00,0x00}},
   {REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x00}},
   {REGFLAG_CMD, 2, {0x96,0x21}},
   {REGFLAG_CMD, 2, {0x97,0x8A}},
   {REGFLAG_CMD, 2, {0x98,0x0C}},
   {REGFLAG_CMD, 2, {0x95,0x10}},

    {REGFLAG_CMD, 1, {0x35}},
    {REGFLAG_CMD, 2, {0x53,0x20}},

    {REGFLAG_CMD, 1, {0x11}},
    {REGFLAG_DELAY,120,{}},
    {REGFLAG_CMD, 1, {0x29}},*/
    //{REGFLAG_DELAY,20,{}},

    //{REGFLAG_CMD, 1, {0x28}},
    {REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x00}},
    {REGFLAG_CMD, 1, {0x39}},
    {REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x0C}},
    {REGFLAG_CMD, 2, {0xBE,0x01}},
    {REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x00}},
    {REGFLAG_CMD, 1, {0x22}},
    {REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x00}},
    //{REGFLAG_DELAY,20,{}},

    /* Display on */
    //{REGFLAG_CMD, 1, {0x29}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_normal_to_aod_sam[] = {
/*
   {REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x08}},
   {REGFLAG_CMD, 2, {0x45,0x4C}},
   {REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x07}},
   {REGFLAG_CMD, 2, {0x29,0x01}},
   {REGFLAG_CMD, 100, {0x20,0x00,0x00,0x00,0x00,0x00,0x11,0x00,0x00,0xab,0x30,0x80,0x09,0x6c,0x04,0x38,0x00,0x0c,0x02,0x1c,0x02,0x1c,0x02,0x00,0x02,0x0e,0x00,0x20,0x01,0x1f,0x00,0x07,0x00,0x0c,0x08,0xbb,0x08,0x7a,0x18,0x00,0x10,0xf0,0x07,0x10,0x20,0x00,0x06,0x0f,0x0f,0x33,0x0e,0x1c,0x2a,0x38,0x46,0x54,0x62,0x69,0x70,0x77,0x79,0x7b,0x7d,0x7e,0x02,0x02,0x22,0x00,0x2a,0x40,0x2a,0xbe,0x3a,0xfc,0x3a,0xfa,0x3a,0xf8,0x3b,0x38,0x3b,0x78,0x3b,0xb6,0x4b,0xf6,0x4c,0x34,0x4c,0x74,0x5c,0x74,0x8c,0xf4,0x00,0x00,0x00,0x00,0x00,0x00}},
   {REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x00}},
   {REGFLAG_CMD, 2, {0x96,0x21}},
   {REGFLAG_CMD, 2, {0x97,0x8A}},
   {REGFLAG_CMD, 2, {0x98,0x0C}},
   {REGFLAG_CMD, 2, {0x95,0x10}},

    {REGFLAG_CMD, 1, {0x35}},
    {REGFLAG_CMD, 2, {0x53,0x20}},

    {REGFLAG_CMD, 1, {0x11}},
    {REGFLAG_DELAY,120,{}},
    {REGFLAG_CMD, 1, {0x29}},*/
    //{REGFLAG_DELAY,20,{}},

    //{REGFLAG_CMD, 1, {0x28}},
    {REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x00}},
    {REGFLAG_CMD, 1, {0x39}},
    {REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x0C}},
    {REGFLAG_CMD, 2, {0xBE,0x01}},
    {REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x00}},
    //{REGFLAG_DELAY,20,{}},

    /* Display on */
    //{REGFLAG_CMD, 1, {0x29}},

        {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
        unsigned int i=0;
        pr_err("debug for lcm %s, panel_init = %d\n", __func__, panel_init);
        aod_state = true;
        aod_flag = true;

        if (panel_init == true) {
                for (i = 0; i < (sizeof(lcm_normal_to_aod_sam_panel_init) / sizeof(struct LCM_setting_table)); i++) {
                        unsigned cmd;
                        cmd = lcm_normal_to_aod_sam_panel_init[i].cmd;

                        switch (cmd) {

                                case REGFLAG_DELAY:
                                        msleep(lcm_normal_to_aod_sam_panel_init[i].count);
                                        break;

                                case REGFLAG_UDELAY:
                                        udelay(lcm_normal_to_aod_sam_panel_init[i].count);
                                        break;

                                case REGFLAG_END_OF_TABLE:
                                        break;

                                default:
                                        cb(dsi, handle, lcm_normal_to_aod_sam_panel_init[i].para_list, lcm_normal_to_aod_sam_panel_init[i].count);
                        }
                }
                panel_init = false;
		aod_after_panel_init = true;
        } else {
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
        }

       /* keep vcore off */
       //lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_SUSPEND_PWR_CTRL, MT_LPM_SMC_ACT_SET, PW_REG_SPM_VCORE_REQ, 0x1);
       pr_info("%s, call lpm_smc_spm_dbg keep vcore on for enter AOD!\n", __func__);

	return 0;
}

#if 0
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
#endif

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

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	int ret;


	if (ctx->prepared)
		return 0;

	pr_err("[lh]debug for samsung_amb670 lcm %s\n", __func__);
	/*
	ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	msleep(5);
	fan53870_ldo1_set_voltage(1120000);
	msleep(10);
	lcm_panel_ldo3_enable(ctx->dev);
	*/
	//enable te
/*	ctx->te_switch_gpio = devm_gpiod_get(ctx->dev, "te_switch", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->te_switch_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->te_switch_gpio);
	usleep_range(5000, 5100);*/

	//usleep_range(30000, 30100);  //beishusheng modify,2022-06-25
	lcm_panel_ldo3_enable(ctx->dev);
	msleep(5);
	ctx->vddr_aod_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-aod-en", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr_aod_enable_gpio, 1);
	usleep_range(1000, 1100);
	ctx->vddr1p5_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr1p5_enable_gpio);

	ret = ctx->error;
	if (ret < 0)
		jdi_unprepare(panel);

	usleep_range(5000, 5100);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	int ret;


	if (ctx->prepared)
		return 0;

	pr_err("[lh]debug for ili7838a lcm %s  ctx->prepared %d \n", __func__,ctx->prepared);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
    msleep(5);
	//enable 1.5V
	ctx->vddr1p5_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vddr1p5_enable_gpio);
	usleep_range(5000, 5100);
	ctx->vddr_aod_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-aod-en", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->vddr_aod_enable_gpio, 0);
        usleep_range(5000, 5100);
    lcm_panel_ldo3_disable(ctx->dev);
    /*
    //beishusheng modify, 2022-06-10
	lcm_panel_vmch_ldo_disable(ctx->dev);
	usleep_range(1000, 1100);
	//set vddi 3.0v
	lcm_panel_vufs_ldo_disable(ctx->dev);
    */
	ret = ctx->error;
	if (ret < 0)
		jdi_unprepare(panel);

	return 0;
}

/*
struct drm_display_mode *get_mode_by_id_hfp(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}
*/

/*
static int mtk_panel_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id_hfp(connector, mode);

	if (drm_mode_vrefresh(m) == 60)
		ext->params = &ext_params;
	else if (drm_mode_vrefresh(m) == 90)
		ext->params = &ext_params_90hz;
	else if (drm_mode_vrefresh(m) == 120)
		ext->params = &ext_params_120hz;
	else
		ret = 1;

	return ret;
}
*/


static int mtk_panel_ext_param_set(struct drm_panel *panel,
             unsigned int mode)
{
    struct mtk_panel_ext *ext = find_panel_ext(panel);
    int ret = 0;
    if (ext == NULL)
        return 1;

    if (mode == 0)
        ext->params = &ext_params;
    else if (mode == 2)
        ext->params = &ext_params_90hz;
    else if (mode == 1)
        ext->params = &ext_params_120hz;
    else
        ret = 1;

    return ret;
}


static void mode_switch_to_120(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	pr_info("%s\n", __func__);

        jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x02);
	jdi_dcs_write_seq_static(ctx,0x38,0x11);
	jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x00);
}

static void mode_switch_to_90(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	pr_info("%s\n", __func__);

	jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x02);
        jdi_dcs_write_seq_static(ctx,0x38,0x12);
        jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x00);
}

static void mode_switch_to_60(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	pr_info("%s\n", __func__);

	jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x02);
        jdi_dcs_write_seq_static(ctx,0x38,0x13);
        jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x00);

}

static int mode_switch(struct drm_panel *panel, unsigned int cur_mode,
    unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
    int ret = 0;

    if (dst_mode == 0) {
        mode_switch_to_60(panel);
    } else if (dst_mode == 2) {
        mode_switch_to_90(panel);
    } else if (dst_mode == 1) {
        mode_switch_to_120(panel);
    }else
        ret = 1;

    return ret;
}


static int oplus_panel_osc_change(void *dsi, dcs_write_gce cb, void *handle, bool en)
{
	char bl_tb0[] = {0xFF, 0x78, 0x38, 0x0F};
	char bl_tb1[] = {0x6E, 0x12};
	char bl_tb2[] = {0x6E, 0x13};
	char bl_tb3[] = {0xFF, 0x78, 0x38, 0x00};

	printk("%s, %d\n", __func__, en);

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	if (en) {
		cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	} else {
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	}
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));

	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = jdi_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	.mode_switch = mode_switch,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.backlight_recovery_after_prepare = backlight_recovery_after_prepare,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.set_hbm = lcm_set_hbm,
	.doze_enable = panel_doze_enable,
	.doze_disable = panel_doze_disable,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.lcm_osc_change = oplus_panel_osc_change,
};
#endif

static int jdi_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *mode2;
	struct drm_display_mode *mode3;

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

    mode2 = drm_mode_duplicate(panel->drm, &performance_mode_90hz);
    if (!mode2) {
            dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
                    performance_mode_90hz.hdisplay, performance_mode_90hz.vdisplay,
                    performance_mode_90hz.vrefresh);
            return -ENOMEM;
    }

	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode2);

    mode3 = drm_mode_duplicate(panel->drm, &performance_mode_120hz);
    if (!mode3) {
            dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
                    performance_mode_120hz.hdisplay, performance_mode_120hz.vdisplay,
                    performance_mode_120hz.vrefresh);
            return -ENOMEM;
    }

	drm_mode_set_name(mode3);
	mode3->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode3);
	
	panel->connector->display_info.width_mm = 70;
	panel->connector->display_info.height_mm = 155;

	return 1;
}

static const struct drm_panel_funcs jdi_drm_funcs = {
	.disable = jdi_disable,
	.unprepare = jdi_unprepare,
	.prepare = jdi_prepare,
	.enable = jdi_enable,
	.get_modes = jdi_get_modes,
};

static bool is_probe_finish = false;
static int jdi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct jdi *ctx;
	struct device_node *backlight;
	int ret;
	pr_info("[tianma] ili7838a %s+\n", __func__);

	if (is_probe_finish == true)
		return 0;
	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);

		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct jdi), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
/*
	g_pmic = kzalloc(sizeof(struct lcm_pmic_info), GFP_KERNEL);
	if (!g_pmic) {
		pr_err("[lh]fail to alloc lcm_pmic_info (ENOMEM)\n");
		return -ENOMEM;
	}*/
	mipi_dsi_set_drvdata(dsi, ctx);
	pr_err("[lh]%s %d\n",__func__,__LINE__);
	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
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
		dev_info(dev, "cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);
/*
	ctx->te_switch_gpio = devm_gpiod_get(dev, "te_switch", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->te_switch_gpio)) {
		dev_err(dev, "%s: cannot get te_switch_gpio %ld\n",
			__func__, PTR_ERR(ctx->te_switch_gpio));
		return PTR_ERR(ctx->te_switch_gpio);
	}
	gpiod_set_value(ctx->te_switch_gpio, 1);
	devm_gpiod_put(dev, ctx->te_switch_gpio);


	ctx->te_out_gpio = devm_gpiod_get(dev, "te_out", GPIOD_IN);
	if (IS_ERR(ctx->te_out_gpio)) {
		dev_err(dev, "%s: cannot get te_out_gpio %ld\n",
			__func__, PTR_ERR(ctx->te_out_gpio));
		return PTR_ERR(ctx->te_out_gpio);
	}

	devm_gpiod_put(dev, ctx->te_out_gpio);
*/
	//lcm_panel_vufs_ldo_enable(ctx->dev);
	lcm_panel_ldo3_enable(ctx->dev);
	usleep_range(5000, 5100);
	ctx->vddr_aod_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-aod-en", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->vddr_aod_enable_gpio, 1);
        usleep_range(1000, 1100);
	ctx->vddr1p5_enable_gpio = devm_gpiod_get(dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p5_enable_gpio)) {
		dev_err(dev, "%s: cannot get vddr1p5_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p5_enable_gpio));
		return PTR_ERR(ctx->vddr1p5_enable_gpio);
	}
	devm_gpiod_put(dev, ctx->vddr1p5_enable_gpio);

	usleep_range(5000, 5100);

        //lcm_panel_vmch_ldo_enable(ctx->dev);

	ctx->prepared = true;
	ctx->enabled = true;
	//drm_panel_init(&ctx->panel, dev, &jdi_drm_funcs, DRM_MODE_CONNECTOR_DSI);
    drm_panel_init(&ctx->panel);
    ctx->panel.dev = dev;
    ctx->panel.funcs = &jdi_drm_funcs;

	drm_panel_add(&ctx->panel);

	is_probe_finish = true;
	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;

#endif
	register_device_proc("lcd", "tian_ma_t7_plus_roma", "tian_ma_t7_plus");
	disp_aal_set_dre_en(1);
	oplus_enhance_mipi_strength = 1;
	//g_is_silky_panel = true;
	is_forbid_enter_ulps_aod_state = true;
	pr_info("%s- lcm,ili7838a,vdo,60hz\n", __func__);

	return ret;
}

static int jdi_remove(struct mipi_dsi_device *dsi)
{
	struct jdi *ctx = mipi_dsi_get_drvdata(dsi);
//#if defined(CONFIG_MTK_PANEL_EXT)
//	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
//#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
//#if defined(CONFIG_MTK_PANEL_EXT)
//	mtk_panel_detach(ext_ctx);
//	mtk_panel_remove(ext_ctx);
//#endif

	return 0;
}

static const struct of_device_id jdi_of_match[] = {
	{
	    .compatible = "oplus22612,tianma,ili7838a,t7_plus,cmd",
	},
	{}
};

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
	.probe = jdi_probe,
	.remove = jdi_remove,
	.driver = {
		.name = "oplus22612_tianma_ili7838a_t7_plus_fhd_dsi_cmd_lcm_drv",
		.owner = THIS_MODULE,
		.of_match_table = jdi_of_match,
	},
};

module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("shaohua deng <shaohua.deng@mediatek.com>");
MODULE_DESCRIPTION("JDI NT36672E VDO 60HZ AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
