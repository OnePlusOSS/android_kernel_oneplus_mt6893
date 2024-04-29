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
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
/* add for ofp */
#include "../oplus/oplus_display_onscreenfingerprint.h"
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
#include <mt-plat/mtk_boot_common.h>
//#include "../oplus/oplus_drm_disp_panel.h"
#include "../../../misc/mediatek/lpm/inc/mtk_lpm_module.h"
#include "../../../misc/mediatek/lpm/modules/include/mt6877/mt6877_pwr_ctrl.h"

#include "../mediatek/mtk_corner_pattern/ac138_data_hw_roundpattern.h"
/* add for dips_drv log  */
#include "../oplus/oplus_display_mtk_debug.h"

#define REGFLAG_CMD       0xFFFA
#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define PHYSICAL_WIDTH (69690)
#define PHYSICAL_HEIGHT (155330)

#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   3515
#define LCM_BRIGHTNESS_TYPE 2
static unsigned int esd_brightness = 1023;
extern unsigned long oplus_display_brightness;
extern unsigned long oplus_max_normal_brightness;
extern unsigned int oplus_lcm_display_on;
static bool aod_state = false;

extern void lcdinfo_notify(unsigned long val, void *v);
extern int pmic_ldo_set_voltage_mv(unsigned int ldo_num, int set_mv);
extern int pmic_ldo_set_disable(unsigned int ldo_num);

//extern unsigned int oplus_enhance_mipi_strength;
struct LCM_setting_table {
  	unsigned int cmd;
 	unsigned char count;
 	unsigned char para_list[128];
};


static int get_mode_enum(struct drm_display_mode *m)
{
	int ret = 0;
	int m_vrefresh = 0;

	if (m == NULL) {
		pr_info("get_mode_enum drm_display_mode *m is null ,default 60fps\n");
		ret = 60;
		return ret;
	}

		m_vrefresh = drm_mode_vrefresh(m);
		DISP_INFO("m_vrefresh=%d\n", m_vrefresh);
		if (m_vrefresh == 60) {
			ret = 60;
		} else if (m_vrefresh == 90) {
			ret = 90;
		} else if (m_vrefresh == 120) {
			ret = 120;
		} else {
			ret = 60;
		}
	return ret;
}

static struct LCM_setting_table init_setting_60hz[] = {
	/* last 1 bit esd check-burn code */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x06}},
	{REGFLAG_CMD, 2, {0xC6,0x01}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x08}},
        {REGFLAG_CMD, 2, {0xD2,0x05}},
	/* open CRC_V line esd */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x02}},
	{REGFLAG_CMD, 2, {0xF8,0x01}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x20}},
	{REGFLAG_CMD, 2, {0xB3,0x50}},
	{REGFLAG_CMD, 2, {0xB5,0x03}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x06}},
	{REGFLAG_CMD, 2, {0xD2,0x01}},
	{REGFLAG_CMD, 2, {0xA1,0x00}},
	/* SY7561 SWIRE Pules > 500us */
	{REGFLAG_CMD, 4, {0xFF, 0x08, 0x38, 0x1F}},
	{REGFLAG_CMD, 6, {0x89, 0x11, 0x01, 0x00, 0xF0, 0x44}},
	/* Flash reload EN */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x21}},
	{REGFLAG_CMD, 2, {0xA4,0x18}},
	/* Demura data select */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x19}},
	{REGFLAG_CMD, 2, {0xC5,0x00}},
	/* Demura Setting */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x30}},
	{REGFLAG_CMD, 2, {0x87,0x04}},
	{REGFLAG_CMD, 2, {0xAE,0x01}},
	{REGFLAG_CMD, 2, {0x80,0x01}},
	/* OTP dont reload */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x08}},
	{REGFLAG_CMD, 2, {0xC8,0x62}},
	/* SPR on */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x14}},
	{REGFLAG_CMD, 2, {0x80,0x01}},
	/* Round off */
	{REGFLAG_CMD, 4, {0xFF,0x08, 0x38, 0x17}},
	{REGFLAG_CMD, 2, {0xA0,0x3D}},
	{REGFLAG_CMD, 4, {0xFF,0x08, 0x38, 0x13}},
	{REGFLAG_CMD, 2, {0xF9,0x00}},
	/* IR Drop off */
	{REGFLAG_CMD, 4, {0xFF,0x08, 0x38, 0x00}},
	{REGFLAG_CMD, 2, {0x95,0x00}},
	/* DSC SETTING 10Bit 3.75 v1.1 */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x07}},
	{REGFLAG_CMD, 2, {0x8A,0x01}},
	{REGFLAG_CMD, 3, {0x8B,0x11,0xE0}},
	{REGFLAG_CMD, 100, {0x81,0x00,0x00,0x00,0x00,0x00,0x11,0x00,0x00,0xab,
                        0x30,0x80,0x09,0x6c,0x04,0x38,0x00,0x0c,0x02,0x1c,
                        0x02,0x1c,0x02,0x00,0x02,0x0e,0x00,0x20,0x01,0x1f,
                        0x00,0x07,0x00,0x0c,0x08,0xbb,0x08,0x7a,0x18,0x00,
                        0x10,0xf0,0x07,0x10,0x20,0x00,0x06,0x0f,0x0f,0x33,
                        0x0e,0x1c,0x2a,0x38,0x46,0x54,0x62,0x69,0x70,0x77,
                        0x79,0x7b,0x7d,0x7e,0x02,0x02,0x22,0x00,0x2a,0x40,
                        0x2a,0xbe,0x3a,0xfc,0x3a,0xfa,0x3a,0xf8,0x3b,0x38,
                        0x3b,0x78,0x3b,0xb6,0x4b,0xb6,0x4b,0xf4,0x4b,0xf4,
                        0x6c,0x34,0x84,0x74,0x00,0x00,0x00,0x00,0x00,0x00}},
	/* wait 3ms for dsc to complete */
	{REGFLAG_DELAY, 3, {}},
	/* Frequence Transition 60Hz */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x2D}},
	{REGFLAG_CMD, 2, {0x81,0x05}},
	/* TE on */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x00}},
	{REGFLAG_CMD, 2, {0x35,0x00}},
	/* Dimmig off */
	{REGFLAG_CMD, 2, {0x53,0x20}},
	{REGFLAG_CMD, 3, {0x51,0x00,0x00}},
	/* SleepOut+DELAY+DisplayOn */
	{REGFLAG_CMD, 1, {0x11}},
	{REGFLAG_DELAY, 120, {}},
	/* TE lock */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x02}},
	{REGFLAG_CMD, 2, {0xA7,0x16}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x00}},
	{REGFLAG_CMD, 1, {0x29}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x20}},
};

static struct LCM_setting_table init_setting_90hz[] = {
	/* last 1 bit esd check-burn code */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x06}},
	{REGFLAG_CMD, 2, {0xC6,0x01}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x08}},
        {REGFLAG_CMD, 2, {0xD2,0x05}},
	/* open CRC_V line esd */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x02}},
	{REGFLAG_CMD, 2, {0xF8,0x01}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x20}},
	{REGFLAG_CMD, 2, {0xB3,0x50}},
	{REGFLAG_CMD, 2, {0xB5,0x03}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x06}},
	{REGFLAG_CMD, 2, {0xD2,0x01}},
	{REGFLAG_CMD, 2, {0xA1,0x00}},
	/* SY7561 SWIRE Pules > 500us */
	{REGFLAG_CMD, 4, {0xFF, 0x08, 0x38, 0x1F}},
	{REGFLAG_CMD, 6, {0x89, 0x11, 0x01, 0x00, 0xF0, 0x44}},
	/* Flash reload EN */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x21}},
	{REGFLAG_CMD, 2, {0xA4,0x18}},
	/* Demura data select */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x19}},
	{REGFLAG_CMD, 2, {0xC5,0x00}},
	/* Demura Setting */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x30}},
	{REGFLAG_CMD, 2, {0x87,0x04}},
	{REGFLAG_CMD, 2, {0xAE,0x01}},
	{REGFLAG_CMD, 2, {0x80,0x01}},
	/* OTP dont reload */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x08}},
	{REGFLAG_CMD, 2, {0xC8,0x62}},
	/* SPR on */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x14}},
	{REGFLAG_CMD, 2, {0x80,0x01}},
	/* Round off */
	{REGFLAG_CMD, 4, {0xFF,0x08, 0x38, 0x17}},
	{REGFLAG_CMD, 2, {0xA0,0x3D}},
	{REGFLAG_CMD, 4, {0xFF,0x08, 0x38, 0x13}},
	{REGFLAG_CMD, 2, {0xF9,0x00}},
	/* IR Drop off */
	{REGFLAG_CMD, 4, {0xFF,0x08, 0x38, 0x00}},
	{REGFLAG_CMD, 2, {0x95,0x00}},
	/* DSC SETTING 10Bit 3.75 v1.1 */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x07}},
	{REGFLAG_CMD, 2, {0x8A,0x01}},
	{REGFLAG_CMD, 3, {0x8B,0x11,0xE0}},
	{REGFLAG_CMD, 100, {0x81,0x00,0x00,0x00,0x00,0x00,0x11,0x00,0x00,0xab,
                        0x30,0x80,0x09,0x6c,0x04,0x38,0x00,0x0c,0x02,0x1c,
                        0x02,0x1c,0x02,0x00,0x02,0x0e,0x00,0x20,0x01,0x1f,
                        0x00,0x07,0x00,0x0c,0x08,0xbb,0x08,0x7a,0x18,0x00,
                        0x10,0xf0,0x07,0x10,0x20,0x00,0x06,0x0f,0x0f,0x33,
                        0x0e,0x1c,0x2a,0x38,0x46,0x54,0x62,0x69,0x70,0x77,
                        0x79,0x7b,0x7d,0x7e,0x02,0x02,0x22,0x00,0x2a,0x40,
                        0x2a,0xbe,0x3a,0xfc,0x3a,0xfa,0x3a,0xf8,0x3b,0x38,
                        0x3b,0x78,0x3b,0xb6,0x4b,0xb6,0x4b,0xf4,0x4b,0xf4,
                        0x6c,0x34,0x84,0x74,0x00,0x00,0x00,0x00,0x00,0x00}},
	/* wait 3ms for dsc to complete */
	{REGFLAG_DELAY, 3, {}},
	/* Frequence Transition 90Hz */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x2D}},
	{REGFLAG_CMD, 2, {0x81,0x04}},
	/* TE on */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x00}},
	{REGFLAG_CMD, 2, {0x35,0x00}},
	/* Dimmig off */
	{REGFLAG_CMD, 2, {0x53,0x20}},
	{REGFLAG_CMD, 3, {0x51,0x00,0x00}},
	/* SleepOut+DELAY+DisplayOn */
	{REGFLAG_CMD, 1, {0x11}},
	{REGFLAG_DELAY, 120, {}},
	/* TE lock */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x02}},
	{REGFLAG_CMD, 2, {0xA7,0x16}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x00}},
	{REGFLAG_CMD, 1, {0x29}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x20}},
};

static struct LCM_setting_table init_setting_120hz[] = {
	/* last 1 bit esd check-burn code */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x06}},
	{REGFLAG_CMD, 2, {0xC6,0x01}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x08}},
        {REGFLAG_CMD, 2, {0xD2,0x05}},
	/* open CRC_V line esd */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x02}},
	{REGFLAG_CMD, 2, {0xF8,0x01}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x20}},
	{REGFLAG_CMD, 2, {0xB3,0x50}},
	{REGFLAG_CMD, 2, {0xB5,0x03}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x06}},
	{REGFLAG_CMD, 2, {0xD2,0x01}},
	{REGFLAG_CMD, 2, {0xA1,0x00}},
	/* SY7561 SWIRE Pules > 500us */
	{REGFLAG_CMD, 4, {0xFF, 0x08, 0x38, 0x1F}},
	{REGFLAG_CMD, 6, {0x89, 0x11, 0x01, 0x00, 0xF0, 0x44}},
	/* Flash reload EN */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x21}},
	{REGFLAG_CMD, 2, {0xA4,0x18}},
	/* Demura data select */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x19}},
	{REGFLAG_CMD, 2, {0xC5,0x00}},
	/* Demura Setting */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x30}},
	{REGFLAG_CMD, 2, {0x87,0x04}},
	{REGFLAG_CMD, 2, {0xAE,0x01}},
	{REGFLAG_CMD, 2, {0x80,0x01}},
	/* OTP dont reload */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x08}},
	{REGFLAG_CMD, 2, {0xC8,0x62}},
	/* SPR on */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x14}},
	{REGFLAG_CMD, 2, {0x80,0x01}},
	/* Round off */
	{REGFLAG_CMD, 4, {0xFF,0x08, 0x38, 0x17}},
	{REGFLAG_CMD, 2, {0xA0,0x3D}},
	{REGFLAG_CMD, 4, {0xFF,0x08, 0x38, 0x13}},
	{REGFLAG_CMD, 2, {0xF9,0x00}},
	/* IR Drop off */
	{REGFLAG_CMD, 4, {0xFF,0x08, 0x38, 0x00}},
	{REGFLAG_CMD, 2, {0x95,0x00}},
	/* DSC SETTING 10Bit 3.75 v1.1 */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x07}},
	{REGFLAG_CMD, 2, {0x8A,0x01}},
	{REGFLAG_CMD, 3, {0x8B,0x11,0xE0}},
	{REGFLAG_CMD, 100, {0x81,0x00,0x00,0x00,0x00,0x00,0x11,0x00,0x00,0xab,
                        0x30,0x80,0x09,0x6c,0x04,0x38,0x00,0x0c,0x02,0x1c,
                        0x02,0x1c,0x02,0x00,0x02,0x0e,0x00,0x20,0x01,0x1f,
                        0x00,0x07,0x00,0x0c,0x08,0xbb,0x08,0x7a,0x18,0x00,
                        0x10,0xf0,0x07,0x10,0x20,0x00,0x06,0x0f,0x0f,0x33,
                        0x0e,0x1c,0x2a,0x38,0x46,0x54,0x62,0x69,0x70,0x77,
                        0x79,0x7b,0x7d,0x7e,0x02,0x02,0x22,0x00,0x2a,0x40,
                        0x2a,0xbe,0x3a,0xfc,0x3a,0xfa,0x3a,0xf8,0x3b,0x38,
                        0x3b,0x78,0x3b,0xb6,0x4b,0xb6,0x4b,0xf4,0x4b,0xf4,
                        0x6c,0x34,0x84,0x74,0x00,0x00,0x00,0x00,0x00,0x00}},
	/* wait 3ms for dsc to complete */
	{REGFLAG_DELAY, 3, {}},
	/* Frequence Transition 120Hz */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x2D}},
	{REGFLAG_CMD, 2, {0x81,0x00}},
	/* TE on */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x00}},
	{REGFLAG_CMD, 2, {0x35,0x00}},
	/* Dimmig off */
	{REGFLAG_CMD, 2, {0x53,0x20}},
	{REGFLAG_CMD, 3, {0x51,0x00,0x00}},
	/* SleepOut+DELAY+DisplayOn */
	{REGFLAG_CMD, 1, {0x11}},
	{REGFLAG_DELAY, 120, {}},
	/* TE lock */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x02}},
	{REGFLAG_CMD, 2, {0xA7,0x16}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x00}},
	{REGFLAG_CMD, 1, {0x29}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x20}},
};

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;
	struct gpio_desc *bias_gpio;
	struct gpio_desc *vddr1p2_enable_gpio;
	struct gpio_desc *vddr_aod_enable_gpio;
	struct gpio_desc *vci_enable_gpio;
	struct drm_display_mode *m;
	struct gpio_desc *te_switch_gpio,*te_out_gpio;
	bool prepared;
	bool enabled;
	int error;
};

#define lcm_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = { seq };                                        \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128,                          \
				 "DCS sequence too big for stack");            \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define lcm_dcs_write_seq_static(ctx, seq...)                                  \
	({                                                                     \
		static const u8 d[] = { seq };                                 \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
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
		DISP_ERR("error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	DISP_INFO("%s+\n", __func__);

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		DISP_INFO("0x%08x\n",buffer[0] | (buffer[1] << 8));
		DISP_INFO("return %d data(0x%08x) to dsi engine\n",
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
		DISP_ERR("error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static void push_table(struct lcm *ctx, struct LCM_setting_table *table, unsigned int count)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			usleep_range(table[i].count*1000, table[i].count*1000 + 100);
			break;
		case REGFLAG_UDELAY:
			usleep_range(table[i].count, table[i].count + 100);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			lcm_dcs_write(ctx, table[i].para_list, table[i].count);
			break;
		}
	}
}

static void lcm_panel_init(struct lcm *ctx)
{
	int mode_id = -1;
	struct drm_display_mode *m = ctx->m;
	mode_id = get_mode_enum(m);
	DISP_INFO("%s +\n", __func__);
	switch (mode_id) {
		DISP_DEBUG("%s:last_fps_mode=%d\n", __func__, mode_id);
		case 120:
			push_table(ctx, init_setting_120hz, sizeof(init_setting_120hz) / sizeof(struct LCM_setting_table));
			break;
		case 90:
			push_table(ctx, init_setting_90hz, sizeof(init_setting_90hz) / sizeof(struct LCM_setting_table));
			break;
		case 60:
			push_table(ctx, init_setting_60hz, sizeof(init_setting_60hz) / sizeof(struct LCM_setting_table));
			break;
		default:
			push_table(ctx, init_setting_60hz, sizeof(init_setting_60hz) / sizeof(struct LCM_setting_table));
			break;
	}

	DISP_INFO("%s -\n", __func__);
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

static int lcm_unprepare(struct drm_panel *panel)
{

	struct lcm *ctx = panel_to_lcm(panel);
	DISP_DEBUG("%s:prepared=%d\n", __func__, ctx->prepared);

	if (!ctx->prepared)
		return 0;

	oplus_lcm_display_on = 0;
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	/* Wait 0ms, actual 15ms */
	usleep_range(15000, 15100);
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	/* Wait > 100ms, actual 125ms */
	usleep_range(125000, 125100);
	/* keep vcore off */
	mtk_lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_SUSPEND_PWR_CTRL, MT_LPM_SMC_ACT_SET, PW_REG_SPM_VCORE_REQ, 0x0);
	DISP_DEBUG(" call lpm_smc_spm_dbg keep vcore off for display off!\n");

	ctx->error = 0;
	ctx->prepared = false;
	DISP_INFO("%s:success\n", __func__);

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	DISP_DEBUG("%s:prepared=%d\n", __func__, ctx->prepared);
	if (ctx->prepared)
		return 0;

	/* Wait > 10ms, actual 10ms */
	usleep_range(10000, 10100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio, 0);
	/* Wait 30us, actual 5ms */
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	/* Wait > 20ms, actual 20ms */
	usleep_range(20000, 20100);
	lcm_panel_init(ctx);
	oplus_lcm_display_on = 1;

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	DISP_INFO("%s:success\n", __func__);
	return ret;
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
	.pll_clk = 300,
	.phy_timcon = {
	    .hs_trail = 6,
	    .clk_trail = 7,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_two_para_compare = 1,
	/* 0x0A:pre-set 0x08, normal 0x9C, AOD 0xDC */
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,  .mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
        .cmd = 0xB0, .count = 1, .para_list[0] = 0x0F, .mask_list[0] = 0x05, .revert_flag = 1,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,


#endif
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.vendor = "A0004",
	.manufacture = "P_3",
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
	.panel_bpp = 10,
	.data_rate = 600,
	.oplus_serial_para0 = 0x81,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
        .oplus_ofp_aod_off_insert_black = 2,
        .oplus_ofp_aod_off_black_frame_total_time = 67,
        .oplus_ofp_need_keep_apart_backlight = true,
        .oplus_ofp_hbm_on_delay = 0,
        .oplus_ofp_pre_hbm_off_delay = 0,
        .oplus_ofp_hbm_off_delay = 0,
        .dyn_fps = {
                .switch_en = 1, .vact_timing_fps = 60,
        },
		.skip_unnecessary_switch = true,
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 300,
	.phy_timcon = {
            .hs_trail = 6,
            .clk_trail = 7,
        },
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_two_para_compare = 1,
	/* 0x0A:pre-set 0x08, normal 0x9C, AOD 0xDC */
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,  .mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
        .cmd = 0xB0, .count = 1, .para_list[0] = 0x0F, .mask_list[0] = 0x05, .revert_flag = 1,
	},

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.vendor = "A0004",
	.manufacture = "P_3",
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
	.panel_bpp = 10,
	.data_rate = 600,
	.oplus_serial_para0 = 0x81,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
        .oplus_ofp_aod_off_insert_black = 2,
        .oplus_ofp_aod_off_black_frame_total_time = 56,
        .oplus_ofp_need_keep_apart_backlight = true,
        .oplus_ofp_hbm_on_delay = 0,
        .oplus_ofp_pre_hbm_off_delay = 0,
        .oplus_ofp_hbm_off_delay = 0,
        .dyn_fps = {
                .switch_en = 1, .vact_timing_fps = 90,
        },
		.skip_unnecessary_switch = true,
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
};

static struct mtk_panel_params ext_params_120hz = {
	.pll_clk = 553.5,
	.phy_timcon = {
            .hs_trail = 10,
            .clk_trail = 11,
        },
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_two_para_compare = 1,
	/* 0x0A:pre-set 0x08, normal 0x9C, AOD 0xDC */
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,  .mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
        .cmd = 0xB0, .count = 1, .para_list[0] = 0x0F, .mask_list[0] = 0x05, .revert_flag = 1,
    },

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.cmd_null_pkt_en = 0,
	.cmd_null_pkt_len = 105,
	.vendor = "A0004",
	.manufacture = "P_3",
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
	.panel_bpp = 10,
	.data_rate = 1107,
	.oplus_serial_para0 = 0x81,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
        .oplus_ofp_aod_off_insert_black = 2,
        .oplus_ofp_aod_off_black_frame_total_time = 50,
        .oplus_ofp_need_keep_apart_backlight = true,
        .oplus_ofp_hbm_on_delay = 0,
        .oplus_ofp_pre_hbm_off_delay = 2,
        .oplus_ofp_hbm_off_delay = 0,
	.dyn_fps = {
                .switch_en = 1, .vact_timing_fps = 120,
        },
	.skip_unnecessary_switch = true,
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}


static struct LCM_setting_table bl_level[] = {
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x00}},
    {REGFLAG_CMD, 3, {0x51,0x07,0xFF}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x20}},
};

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	unsigned int mapped_level = 0;
	/* 07FF = 2047 */
	int i = 0;
	if (!cb)
		return -1;

	if (level == 1) {
		DISP_INFO("enter aod!!!\n");
		return 0;
	}
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0)
		level = 2047;

	bl_level[1].para_list[1] = level >> 8;
	bl_level[1].para_list[2] = level & 0xFF;
	mapped_level = level;
	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

    for (i = 0; i < sizeof(bl_level)/sizeof(struct LCM_setting_table); i++) {
            cb(dsi, handle, bl_level[i].para_list, bl_level[i].count);
    }

	esd_brightness = level;
	oplus_display_brightness = level;
	DISP_INFO("backlight=%d,paralist[1]=0x%x,paralist[2]=0x%x\n", level, bl_level[1].para_list[1], bl_level[1].para_list[2]);
	return 0;
}

static void lcm_setbrightness(void *dsi,
			      dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int BL_MSB = 0;
	unsigned int BL_LSB = 0;
	int i = 0;

	DISP_DEBUG("level is %d\n",level);
	BL_LSB = level >> 8;
	BL_MSB = level & 0xFF;

	bl_level[1].para_list[1] = BL_LSB;
	bl_level[1].para_list[2] = BL_MSB;

    for (i = 0; i < sizeof(bl_level)/sizeof(struct LCM_setting_table); i++) {
            cb(dsi, handle, bl_level[i].para_list, bl_level[i].count);
    }
	DISP_INFO("backlight=%d,paralist[1]=0x%x,paralist[2]=0x%x\n", level, bl_level[1].para_list[1], bl_level[1].para_list[2]);
}

static struct LCM_setting_table lcm_finger_HBM_on_setting[] = {
    {REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x00}},
    {REGFLAG_CMD, 3, {0x51,0x0F,0xFF}},
    {REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x20}},

};

static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	int i = 0;

	if (!cb)
		return -1;

	DISP_INFO("oplus_display_brightness= %ld, hbm_mode=%u\n", oplus_display_brightness, hbm_mode);

	if(hbm_mode == 1) {
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (hbm_mode == 0) {
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);  //level
		DISP_DEBUG(" %d ! backlight %d !\n",hbm_mode, oplus_display_brightness);
	}

	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
			      dcs_write_gce cb, void *handle, bool en)
{
	int i = 0;
	int level = 0;
	if (!cb)
		return -1;

	DISP_INFO("oplus_display_brightness= %ld, en=%u\n", oplus_display_brightness, en);

	if(en == 1) {
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (en == 0) {
		level = oplus_display_brightness;
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);
		DISP_DEBUG(": %d ! backlight %d !\n", en, oplus_display_brightness);
	}
	lcdinfo_notify(1, &en);
	return 0;
}


static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
	void *handle)
{
	/* 03FF = 1023 */
	char bl_tb0[] = {0x51, 0x03, 0xff};

	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
 	if (!cb)
		return -1;
	DISP_INFO(" bl_tb0[1]=%x, bl_tb0[2]=%x\n", bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

 	return 1;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		DISP_ERR("%s:cannot get reset-gpios %ld\n", __func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	return 0;
}

static struct LCM_setting_table lcm_aod_to_normal[] = {
	/* AOD Mode OFF */
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x00}},
	{REGFLAG_CMD, 1, {0x38}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x20}},
	{REGFLAG_CMD, 3, {0x51, 0x00, 0x00}},
};

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i=0;

	/* Switch back to VDO mode */
	for (i = 0; i < (sizeof(lcm_aod_to_normal) / sizeof(struct LCM_setting_table)); i++) {
		unsigned cmd;
		cmd = lcm_aod_to_normal[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY:
				// if (handle == NULL) {
					usleep_range(lcm_aod_to_normal[i].count * 1000, lcm_aod_to_normal[i].count * 1000 + 100);
				// } else {
					// cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(lcm_aod_to_normal[i].count * 1000), CMDQ_GPR_R14);
				// }
				break;

			case REGFLAG_UDELAY:
				// if (handle == NULL) {
					usleep_range(lcm_aod_to_normal[i].count, lcm_aod_to_normal[i].count + 100);
				// } else {
					// cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(lcm_aod_to_normal[i].count), CMDQ_GPR_R14);
				// }
				break;

			case REGFLAG_END_OF_TABLE:
				break;

			default:
				cb(dsi, handle, lcm_aod_to_normal[i].para_list, lcm_aod_to_normal[i].count);
		}
	}


	aod_state = false;
	/* keep vcore off */
	mtk_lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_SUSPEND_PWR_CTRL, MT_LPM_SMC_ACT_SET, PW_REG_SPM_VCORE_REQ, 0x0);
	lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	DISP_INFO(" call lpm_smc_spm_dbg keep vcore off for exit AOD!\n");
	DISP_INFO("%s:success\n", __func__);

	return 0;
}

static struct LCM_setting_table lcm_normal_to_aod[] = {
	 /* AOD Switch */
	 {REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x00}},
	 {REGFLAG_CMD, 5, {0x51,0x00,0x00,0x0D,0xBB}},
	 /* AOD on */
	 {REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x05}},
	 {REGFLAG_CMD, 2, {0x80,0x19}},
	 {REGFLAG_CMD, 10, {0xD0,0xFF,0xAF,0x56,0x3D,0x2D,0x2D,0x2D,0x2D,0xFF}},
	 {REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x00}},
	 {REGFLAG_CMD, 1, {0x39}},
	 /* Delay */
	 {REGFLAG_DELAY, 30, {}},
	 /* page 20 */
	 {REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x20}},
};

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i=0;

	aod_state = true;

	for (i = 0; i < (sizeof(lcm_normal_to_aod) / sizeof(struct LCM_setting_table)); i++) {
		unsigned cmd;
		cmd = lcm_normal_to_aod[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY:
				msleep(lcm_normal_to_aod[i].count);
				break;

			case REGFLAG_UDELAY:
				udelay(lcm_normal_to_aod[i].count);
				break;

			case REGFLAG_END_OF_TABLE:
				break;

			default:
				cb(dsi, handle, lcm_normal_to_aod[i].para_list, lcm_normal_to_aod[i].count);
		}
	}
	/* keep vcore on */
	mtk_lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_SUSPEND_PWR_CTRL, MT_LPM_SMC_ACT_SET, PW_REG_SPM_VCORE_REQ, 0x1);
	DISP_INFO("call lpm_smc_spm_dbg keep vcore on for enter AOD then Succ!\n");
	DISP_INFO("%s:success\n", __func__);

	return 0;
}

static struct LCM_setting_table lcm_aod_high_mode[] = {
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x00}},
	{REGFLAG_CMD, 5, {0x51,0x00,0x00,0x0D,0xBB}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x20}},
};

static struct LCM_setting_table lcm_aod_low_mode[] = {
	//10nit
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x00}},
	{REGFLAG_CMD, 5, {0x51,0x00,0x00,0x0A,0xe0}},
	{REGFLAG_CMD, 4, {0xFF,0x08,0x38,0x20}},
};

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

	DISP_DEBUG("+ %s\n");
	if (level == 0) {
		for (i = 0; i < sizeof(lcm_aod_high_mode)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_aod_high_mode[i].para_list, lcm_aod_high_mode[i].count);
		}
	} else {
		for (i = 0; i < sizeof(lcm_aod_low_mode)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_aod_low_mode[i].para_list, lcm_aod_low_mode[i].count);
		}
	}
	DISP_DEBUG("%s:success %d !\n", __func__, level);

	return 0;
}

static struct regulator *mt6315_6_vbuck4;
static int vddr6_buck4_regulator_init(struct device *dev)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;

    pr_err("get vddr6_buck4_regulator_init\n");

	/* please only get regulator once in a driver */
	mt6315_6_vbuck4 = devm_regulator_get(dev, "6_vbuck4");
	if (IS_ERR_OR_NULL(mt6315_6_vbuck4)) { /* handle return value */
		ret = PTR_ERR(mt6315_6_vbuck4);
		pr_err("get vddr6_buck4_optional fail, error: %d\n", ret);
		//return ret;
	}
	regulator_inited = 1;
	return ret; /* must be 0 */
}
static int vddr6_buck4_regulator_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	vddr6_buck4_regulator_init(dev);

	/* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(mt6315_6_vbuck4)) {
		ret = regulator_set_voltage(mt6315_6_vbuck4, 1190000, 1193750);
		if (ret < 0)
			DISP_ERR("set voltage mt6315_6_vbuck4 fail, ret = %d\n", ret);
		retval |= ret;
	}

	/* enable regulator */
	if (!IS_ERR_OR_NULL(mt6315_6_vbuck4)) {
		ret = regulator_enable(mt6315_6_vbuck4);
		if (ret < 0)
			DISP_ERR("enable regulator mt6315_6_vbuck4 fail, ret = %d\n", ret);
		retval |= ret;
	}
	DISP_INFO("get vddr6_buck4_regulator_enable\n");
	return retval;
}
static int vddr6_buck4_regulator_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	vddr6_buck4_regulator_init(dev);

	if (!IS_ERR_OR_NULL(mt6315_6_vbuck4)) {
		ret = regulator_disable(mt6315_6_vbuck4);
		if (ret < 0)
			DISP_ERR("disable regulator mt6315_6_vbuck4 fail, ret = %d\n", ret);
		retval |= ret;
	}
	return retval;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	DISP_DEBUG("p_3 a0004 lcm ctx->prepared %d\n", ctx->prepared);

	if (ctx->prepared)
		return 0;
	/* VDDI pmic always enable */
	/* Wait > 0, actual 2ms */
	usleep_range(2000, 2100);
	/* VDDR LDO vBuck enable */
	vddr6_buck4_regulator_enable(ctx->dev);
	/* Wait > 0, actual 2ms */
	usleep_range(2000, 2100);
	/* VCI LDO enable */
	pmic_ldo_set_voltage_mv(3,3000);
	/* Wait > 10ms, actual 15ms */
	usleep_range(5000, 5100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	DISP_INFO("%s:success\n", __func__);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	DISP_DEBUG("p_3 a0004 lcm ctx->prepared %d \n",ctx->prepared);

	if (ctx->prepared)
		return 0;

	/* RST GPIO disable */
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	/* Wait > 0, actual 5ms */
	usleep_range(5000, 5100);
	/* VCI LDO disable */
	pmic_ldo_set_disable(3);
	/* Wait > 0, actual 5ms */
	usleep_range(5000, 5100);
	/* VDDR LDO vBuck disable */
	vddr6_buck4_regulator_disable(ctx->dev);
	/* Wait > 0, actual 15ms */
	usleep_range(15000, 15100);
	/* VDDI pmic always enable */
	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	/* fast resume/suspend 70ms interval */
	usleep_range(70000, 70100);
	DISP_INFO("%s:success\n", __func__);
	return 0;
}

static struct drm_display_mode *get_mode_by_id(struct drm_panel *panel, unsigned int mode)
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
	pr_err("%s ,%d\n", __func__, mode);
	if (drm_mode_vrefresh(m) == 60) {
		ext->params = &ext_params;
		DISP_DEBUG("set 60 params success\n");
	} else if (drm_mode_vrefresh(m) == 90) {
		ext->params = &ext_params_90hz;
		DISP_DEBUG("set 90 params success\n");
	} else if (drm_mode_vrefresh(m) == 120) {
		ext->params = &ext_params_120hz;
		DISP_DEBUG("set 120 params success\n");
	} else
		ret = 1;

	return ret;
}

static unsigned int last_fps_mode = 60;
static void mode_switch_to_120(struct drm_panel *panel, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == AFTER_DSI_POWERON) {
		lcm_dcs_write_seq_static(ctx, 0xFF, 0x08, 0x38, 0x2D);
		lcm_dcs_write_seq_static(ctx, 0x81, 0x00);
		lcm_dcs_write_seq_static(ctx, 0xFF,0x08,0x38,0x20);
		last_fps_mode = 120;
		DISP_DEBUG("set 120 success\n");
	}
}

static void mode_switch_to_90(struct drm_panel *panel, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		lcm_dcs_write_seq_static(ctx, 0xFF, 0x08, 0x38, 0x2D);
		lcm_dcs_write_seq_static(ctx, 0x81, 0x04);
		lcm_dcs_write_seq_static(ctx, 0xFF,0x08,0x38,0x20);
		if (last_fps_mode == 120) {
			usleep_range(8300, 8400);
		}
		last_fps_mode = 90;
		DISP_DEBUG("set 90 success\n");
	}
}

static void mode_switch_to_60(struct drm_panel *panel, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		lcm_dcs_write_seq_static(ctx, 0xFF, 0x08, 0x38, 0x2D);
		lcm_dcs_write_seq_static(ctx, 0x81, 0x05);
		lcm_dcs_write_seq_static(ctx, 0xFF,0x08,0x38,0x20);
		if (last_fps_mode == 120) {
			usleep_range(8300, 8400);
		}
		last_fps_mode = 60;
		DISP_DEBUG("set 60 success\n");
	}
}

static int mode_switch(struct drm_panel *panel, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(panel, dst_mode);
	struct lcm *ctx = panel_to_lcm(panel);
	DISP_INFO("cur_mode = %d dst_mode %d\n", cur_mode, dst_mode);

	if (drm_mode_vrefresh(m) == 60) {
		/* mode switch to 60 */
		mode_switch_to_60(panel, stage);
	} else if (drm_mode_vrefresh(m) == 90) {
		/* mode switch to 90 */
		mode_switch_to_90(panel, stage);
	} else if (drm_mode_vrefresh(m) == 120) {
		/* mode switch to 120 */
		mode_switch_to_120(panel, stage);
	} else
		ret = 1;
	ctx->m = m;

	return ret;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	.mode_switch = mode_switch,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.set_hbm = lcm_set_hbm,
	.doze_enable = panel_doze_enable,
	.doze_disable = panel_doze_disable,
	.set_aod_light_mode = panel_set_aod_light_mode,
};
#endif

static int lcm_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *mode2;
	struct drm_display_mode *mode3;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		DISP_ERR("failed to add mode %ux%ux@%u\n",
			 default_mode.hdisplay, default_mode.vdisplay,
			 default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);


	mode2 = drm_mode_duplicate(panel->drm, &performance_mode_90hz);
	if (!mode2) {
		DISP_ERR("failed to add mode %ux%ux@%u\n",
			 performance_mode_90hz.hdisplay, performance_mode_90hz.vdisplay,
			 performance_mode_90hz.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode2);

	mode3 = drm_mode_duplicate(panel->drm, &performance_mode_120hz);
	if (!mode3) {
		DISP_ERR("failed to add mode %ux%ux@%u\n",
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
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;

	DISP_INFO("p_3 A0004 +\n");

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);

		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				DISP_ERR("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			DISP_ERR("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		DISP_ERR("skip probe due to not current lcm\n");
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	mipi_dsi_set_drvdata(dsi, ctx);
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
		DISP_ERR("cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	usleep_range(5000, 5100);
	vddr6_buck4_regulator_enable(ctx->dev);

	usleep_range(5000, 5100);
	pmic_ldo_set_voltage_mv(3,3000);

	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;

#endif
	register_device_proc("lcd", "A0004", "P_3");
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;

	DISP_INFO("success!\n");

	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
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

static const struct of_device_id lcm_of_match[] = {
	{
	    .compatible = "ac138,p_3,a0004,cmd",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "ac138_p_3_a0004_cmd_panel",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

static int __init lcm_drv_init(void)
{
	int ret = 0;

	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	ret = mipi_dsi_driver_register(&lcm_driver);
	if (ret < 0)
		pr_notice("%s, Failed to register lcm driver: %d\n",
			__func__, ret);

	mtk_panel_unlock();
	pr_notice("%s- ret:%d\n", __func__, ret);
	return 0;
}

static void __exit lcm_drv_exit(void)
{
	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	mipi_dsi_driver_unregister(&lcm_driver);
	mtk_panel_unlock();
	pr_notice("%s-\n", __func__);
}
module_init(lcm_drv_init);
module_exit(lcm_drv_exit);

MODULE_AUTHOR("oplus");
MODULE_DESCRIPTION("ac138,p_3,a0004,OLED Driver");
MODULE_LICENSE("GPL v2");
