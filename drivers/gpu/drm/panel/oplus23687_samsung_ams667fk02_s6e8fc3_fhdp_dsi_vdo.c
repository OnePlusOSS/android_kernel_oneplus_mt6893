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
#include <mt-plat/mtk_boot_common.h>
#include "../mediatek/mtk_panel_ext.h"
#include "../mediatek/mtk_drm_graphics_base.h"
#include "../oplus/oplus_display_onscreenfingerprint.h"

#include "../mediatek/mtk_corner_pattern/oplus23687_data_hw_roundedpattern.h"

#define REGFLAG_CMD             0xFFFA
#define REGFLAG_DELAY           0xFFFC
#define REGFLAG_UDELAY          0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD

#define LCM_DSI_CMD_MODE        0
#define BRIGHTNESS_MAX	        4095
#define BRIGHTNESS_HALF         2047
#define MAX_NORMAL_BRIGHTNESS   2047
#define LCM_BRIGHTNESS_TYPE     2
#define OPLUS_DC_BACKLIGHT_THRESHOLD 1200

static unsigned int esd_brightness = 1023;
static u32 flag_hbm = 0;
extern unsigned int oplus_display_brightness;
extern unsigned long oplus_max_normal_brightness;
static bool aod_state = false;
extern bool aod_flag;
extern void disp_aal_set_dre_en(int enable);
static bool panel_init = false;
static bool aod_after_panel_init = false;
static bool is_probe_finish = false;
extern int oplus_dc_alpha;
extern int oplus_dc_enable_real;
extern int oplus_dc_enable;
extern int exit_dc_flag;
extern int oplus_dc_flag;
extern unsigned long seed_mode;
extern void lcdinfo_notify(unsigned long val, void *v);
static int current_fps = 60;
static unsigned int osc_mipi_hopping_status = 0;
extern unsigned int oplus_enhance_mipi_strength;

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *vddr1v35_enable_gpio;
	struct gpio_desc *vci_3v_enable_gpio;
	bool prepared;
	bool enabled;
	bool hbm_en;
	bool hbm_wait;
	int error;
};

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[128];
};

struct ba {
	u32 brightness;
	u32 alpha;
};

static struct ba brightness_seed_alpha_lut_dc[] = {
	{0,2000},
	{1,1999},
	{12,1960},
	{66,1950},
	{86,1945},
	{138,1936},
	{150,1928},
	{169,1920},
	{195,1912},
	{220,1904},
	{225,1897},
	{240,1889},
	{246,1881},
	{264,1873},
	{267,1865},
	{285,1857},
	{290,1847},
	{315,1832},
	{344,1818},
	{351,1800},
	{365,1787},
	{379,1771},
	{407,1748},
	{426,1716},
	{448,1686},
	{459,1661},
	{474,1631},
	{488,1631},
	{505,1599},
	{523,1567},
	{537,1528},
	{550,1505},
	{575,1458},
	{600,1410},
	{625,1348},
	{650,1285},
	{675,1222},
	{700,1151},
	{725,1151},
	{750,1080},
	{800,930},
	{900,800},
	{1000,650},
	{1100,380},
	{1200,0},
};

static struct LCM_setting_table lcm_setbrightness_normal[] = {
	{REGFLAG_CMD,2, {0x53, 0x28}},
	{REGFLAG_CMD,3, {0x51, 0x00, 0x00}},
};

static struct LCM_setting_table lcm_setbrightness_hbm[] = {
	{REGFLAG_CMD,3, {0x51, 0x00, 0x00}},
	{REGFLAG_CMD,2, {0x53, 0xE8}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_finger_HBM_on_setting[] = {
	/*HBM ON*/
	{REGFLAG_CMD,3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2, {0x53, 0xE8}},
	{REGFLAG_CMD,3, {0x51, 0x0C, 0x00}},
	{REGFLAG_CMD,3, {0xF0, 0xA5, 0xA5}},
};

static struct LCM_setting_table lcm_normal_HBM_on_setting[] = {
	{REGFLAG_CMD,2, {0x53, 0xE8}},
	{REGFLAG_CMD,3, {0x51, 0x0F, 0xFF}},
};

static struct LCM_setting_table lcm_aod_to_normal[] = {
	// {REGFLAG_CMD, 4, {0xFF, 0x78, 0x38, 0x00}},
	// {REGFLAG_CMD, 1, {0x38}},
	// {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_high_mode[] = {
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0x53, 0x28}},
	{REGFLAG_CMD, 3, {0x51, 0x03, 0xFF}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	// {REGFLAG_CMD,2,{0x60,0X01 }},
	// {REGFLAG_CMD,4,{0xB0,0x01, 0x17, 0xB2 }},
	// {REGFLAG_CMD,2,{0xB2,0x00 }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0x03, 0xF4 }},
	// {REGFLAG_CMD,2,{0xF4,0x4A }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0x1B, 0xF4 }},
	// {REGFLAG_CMD,2,{0xF4,0x9A }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0x1D, 0xF4 }},
	// {REGFLAG_CMD,2,{0xF4,0x1F }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0x11, 0xB2 }},
	// {REGFLAG_CMD,2,{0xB2,0x16 }},
	// {REGFLAG_CMD,2,{0xB7,0x02 }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0x86, 0xB1 }},
	// {REGFLAG_CMD,2,{0xB1,0x02 }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0x88, 0xB1 }},
	// {REGFLAG_CMD,2,{0xB1,0x27 }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0x18, 0xB6 }},
	// {REGFLAG_CMD,2,{0xB6,0x82 }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0x2D, 0xB6 }},
	// {REGFLAG_CMD,2,{0xB6,0x0E }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0xBC, 0xB5 }},
	// {REGFLAG_CMD,2,{0xB5,0x26 }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0x91, 0xC3 }},
	// {REGFLAG_CMD,4,{0xC3,0x0B, 0x00, 0x06 }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0xA3, 0xC3 }},
	// {REGFLAG_CMD,9,{0xC3,0x16, 0x14, 0x00, 0x15, 0x04, 0x0B, 0x11, 0x11 }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0xB7, 0xC3 }},
	// {REGFLAG_CMD,2,{0xC3,0x02 }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0xBE, 0xC3 }},
	// {REGFLAG_CMD,2,{0xC3,0x71 }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0xEE, 0xC3 }},
	// {REGFLAG_CMD,2,{0xC3,0x04 }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0xF0, 0xC3 }},
	// {REGFLAG_CMD,2,{0xC3,0x04 }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0xF3, 0xC3 }},
	// {REGFLAG_CMD,3,{0xC3,0x1D, 0x1D }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0x2F, 0xF6 }},
	// {REGFLAG_CMD,3,{0xF6,0x26, 0x28 }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0x32, 0xF6 }},
	// {REGFLAG_CMD,3,{0xF6,0x23, 0x10 }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0x39, 0xF6 }},
	// {REGFLAG_CMD,2,{0xF6,0x01 }},
	// {REGFLAG_CMD,4,{0xB0,0x01, 0x61, 0xB2 }},
	// {REGFLAG_CMD,2,{0xB2,0x00 }},
	// {REGFLAG_CMD,4,{0xB0,0x00, 0x03, 0xCD }},
	// {REGFLAG_CMD,2,{0xCD,0x14 }},
	// {REGFLAG_CMD,2,{0xF7,0x0B }},
	// {REGFLAG_CMD,3,{0xF0,0xA5, 0xA5 }},
	// {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_low_mode[] = {
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0x53, 0x28}},
	{REGFLAG_CMD, 3, {0x51, 0x01, 0xFF}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

// static struct LCM_setting_table lcm_normal_to_aod_sam_panel_init[] = {
// 	{REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x00}},
// 	{REGFLAG_CMD, 1, {0x39}},
// 	{REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x0C}},
// 	{REGFLAG_CMD, 2, {0xBE,0x01}},
// 	{REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x00}},
// 	{REGFLAG_CMD, 1, {0x22}},
// 	{REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x00}},
// 	//{REGFLAG_DELAY,20,{}},

// 	/* Display on */
// 	//{REGFLAG_CMD, 1, {0x29}},

// 	{REGFLAG_END_OF_TABLE, 0x00, {}}
// };

static struct LCM_setting_table lcm_normal_to_aod_sam[] = {
	{REGFLAG_CMD, 1, {0x11}},
	{REGFLAG_DELAY, 20, {}},

	{REGFLAG_CMD, 2, {0x35, 0x00}},

	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0x60, 0x21}},
	{REGFLAG_CMD, 3, {0xF7, 0x0B}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},

	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0x53, 0x28}},
	{REGFLAG_CMD, 3, {0x51, 0x03, 0xFF}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},
	{REGFLAG_DELAY, 120,{}},
	{REGFLAG_CMD, 1, {0x29}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	//  {REGFLAG_CMD,2,{0x60,0x01 }}, 	//Frequency Select
	//  {REGFLAG_CMD,4,{0xB0,0x01, 0x17, 0xB2 }},
	//  {REGFLAG_CMD,2,{0xB2,0x01 }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0x03, 0xF4 }},
	//  {REGFLAG_CMD,2,{0xF4,0x6C }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0x1B, 0xF4 }},
	//  {REGFLAG_CMD,2,{0xF4,0x83 }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0x1D, 0xF4 }},
	//  {REGFLAG_CMD,2,{0xF4,0x12 }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0x11, 0xB2 }},
	//  {REGFLAG_CMD,2,{0xB2,0x1F }},
	//  {REGFLAG_CMD,2,{0xB7,0x01 }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0x86, 0xB1 }},
	//  {REGFLAG_CMD,2,{0xB1,0x01 }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0x88, 0xB1 }},
	//  {REGFLAG_CMD,2,{0xB1,0x26 }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0x18, 0xB6 }},
	//  {REGFLAG_CMD,2,{0xB6,0x81 }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0x2D, 0xB6 }},
	//  {REGFLAG_CMD,2,{0xB6,0x01 }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0xBC, 0xB5 }},
	//  {REGFLAG_CMD,2,{0xB5,0x25 }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0x91, 0xC3 }},
	//  {REGFLAG_CMD,4,{0xC3,0x0F, 0x00, 0x02 }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0xA3, 0xC3 }},
	//  {REGFLAG_CMD,9,{0xC3,0x70, 0x28, 0x00, 0x29, 0x04, 0x3F, 0x47,0x3F }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0XB7, 0xC3 }},
	//  {REGFLAG_CMD,2,{0xC3,0x80 }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0xBE, 0xC3 }},
	//  {REGFLAG_CMD,2,{0xC3,0x11 }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0xEE, 0xC3 }},
	//  {REGFLAG_CMD,2,{0xC3,0x44 }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0xF0, 0xC3 }},
	//  {REGFLAG_CMD,2,{0xC3,0xFF }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0xF3, 0xC3 }},
	//  {REGFLAG_CMD,3,{0xC3,0x0A, 0x0A }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0x2F, 0xF6 }},
	//  {REGFLAG_CMD,3,{0xF6,0x88, 0x8A }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0x32, 0xF6 }},
	//  {REGFLAG_CMD,3,{0xF6,0x88, 0x45 }},
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0x39, 0xF6 }},
	//  {REGFLAG_CMD,2,{0xF6,0x41 }},
	//  {REGFLAG_CMD,4,{0xB0,0x01, 0x61, 0xB2 }},
	//  {REGFLAG_CMD,4,{0xB2,0x01, 0x00, 0x14 }},	//30Hz High Mode(50nit)
	//  {REGFLAG_CMD,4,{0xB0,0x00, 0x03, 0xCD }},
	//  {REGFLAG_CMD,2,{0xCD,0x34 }},
	//  {REGFLAG_CMD,2,{0xF7,0x0B }},
	//  {REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},
	//  {REGFLAG_DELAY, 17,{}},
	//  {REGFLAG_CMD, 1, {0x29}},
	//  {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_seed_setting[] = {
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x86}},
	{REGFLAG_CMD,2,{0x62, 0x00}},
	{REGFLAG_CMD,3,{0xB0, 0x2B, 0x62}},
	{REGFLAG_CMD,22,{0x62,0xDA,0x00,0x00,0x0C,0xD7,0x03,0x09,0x05,0xC4,0x16,0xFA,0xE2,0xF7,0x00,0xE4,0xE9,0xE6,0x03,0xFF,0xFF,0xFF}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};

static struct LCM_setting_table lcm_seed_mode0[] = {
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x21, 0x00}},
	{REGFLAG_CMD,2,{0xF8, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x86}},
	{REGFLAG_CMD,2,{0x62, 0x00}},
	{REGFLAG_CMD,3,{0xB0, 0x2B, 0x62}},
	{REGFLAG_CMD,22,{0x62,0xDA,0x00,0x00,0x0C,0xD7,0x03,0x09,0x05,0xC4,0x16,0xFA,0xE2,0xF7,0x00,0xE4,0xE9,0xE6,0x03,0xFF,0xFF,0xFF}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x01, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};

static struct LCM_setting_table lcm_seed_mode1[] = {
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x21, 0x00}},
	{REGFLAG_CMD,2,{0xF8, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x06}},
	{REGFLAG_CMD,2,{0x62, 0x00}},
	{REGFLAG_CMD,3,{0xB0, 0x01, 0x62}},
	{REGFLAG_CMD,22,{0x62,0xB0,0x02,0x06,0x3F,0xD3,0x15,0x05,0x09,0xB0,0x48,0xF2,0xDC,0xC4,0x07,0xC8,0xE9,0xE9,0x1D,0xFD,0xF5,0xE1}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x01, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};

static struct LCM_setting_table lcm_seed_mode2[] = {
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x21, 0x00}},
	{REGFLAG_CMD,2,{0xF8, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x86}},
	{REGFLAG_CMD,2,{0x62, 0x00}},
	{REGFLAG_CMD,3,{0xB0, 0x2B, 0x62}},
	{REGFLAG_CMD,22,{0x62,0xD8,0x00,0x04,0x00,0xFF,0x02,0x00,0x00,0xFF,0x18,0xFF,0xE4,0xFB,0x00,0xF0,0xF6,0xEA,0x01,0xFF,0xFF,0xFF}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x01, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};

static struct LCM_setting_table lcm_seed_mode3[] = {
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x21, 0x00}},
	{REGFLAG_CMD,2,{0xF8, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x86}},
	{REGFLAG_CMD,2,{0x62, 0x00}},
	{REGFLAG_CMD,3,{0xB0, 0x2B, 0x62}},
	{REGFLAG_CMD,22,{0x62,0xDA,0x00,0x00,0x08,0xDB,0x05,0x0B,0x01,0xC6,0x16,0xFA,0xE2,0xF7,0x00,0xE4,0xE9,0xE6,0x03,0xFD,0xF5,0xE1}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x01, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};

#define FRAME_WIDTH             (1080)
#define FRAME_HEIGHT            (2400)
#define HFP                     (60)
#define HBP                     (60)
#define HSA                     (12)
#define VFP_60HZ                (2448)
#define VFP_120HZ               (16)
#define VBP                     (14)
#define VSA                     (2)

static const struct drm_display_mode default_mode = {
	.clock = 353710, // ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP + VBP + VSA) * 60) / 1000
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + VFP_60HZ,
	.vsync_end = FRAME_HEIGHT + VFP_60HZ + VSA,
	.vtotal = FRAME_HEIGHT + VFP_60HZ + VSA + VBP,
	.vrefresh = 60,
};

static const struct drm_display_mode performance_mode_120hz = {
	.clock = 353710, // ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP + VBP + VSA) * 120) / 1000
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + VFP_120HZ,
	.vsync_end = FRAME_HEIGHT + VFP_120HZ + VSA,
	.vtotal = FRAME_HEIGHT + VFP_120HZ + VSA + VBP,
	.vrefresh = 120,
};

static struct mtk_panel_params ext_params = {
	.pll_clk = 499,
	.data_rate = 998,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		// .send_mode = 1,
		// .send_cmd_need_delay = 1,
		.dfps_cmd_table[0] = {0, 3 , {0xF0, 0x5A, 0x5A}},
		.dfps_cmd_table[1] = {0, 2 , {0x60, 0x21}},
		.dfps_cmd_table[2] = {0, 2 , {0xF7, 0x0B}},
		.dfps_cmd_table[3] = {0, 3 , {0xF0, 0xA5, 0xA5}},
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9D, .mask_list[0] = 0x9D,
	},
	// .lcm_esd_check_table[1] = {
	// 	.cmd = 0x05, .count = 1, .para_list[0] = 0x00,
	// },


	#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
		.round_corner_en = 1,
		.corner_pattern_height = ROUND_CORNER_H_TOP,
		.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
		.corner_pattern_tp_size = sizeof(top_rc_pattern),
		.corner_pattern_lt_addr = (void *)top_rc_pattern,
	#endif

	.oplus_display_global_dre = 1,
	.oplus_custom_hdr_color_tmp = true,
	.oplus_custom_hdr_red = 950,
	.oplus_custom_hdr_green = 1024,
	.oplus_custom_hdr_blue = 800,
	.oplus_panel_use_rgb_gain = true,

	.vendor = "AMS643AG01_23687",
	.manufacture = "samsung2048_23687",
#ifdef CONFIG_OPLUS_OFP_V2
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.oplus_uiready_before_time = 17,
#endif

	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2400,
		.pic_width = 1080,
		.slice_height = 40,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 989,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 631,
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
};

static struct mtk_panel_params ext_params_120hz = {
	.pll_clk = 499,
	.data_rate = 998,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		// .send_mode = 1,
		// .send_cmd_need_delay = 1,
		.dfps_cmd_table[0] = {0, 3 , {0xF0, 0x5A, 0x5A}},
		.dfps_cmd_table[1] = {0, 2 , {0x60, 0x01}},
		.dfps_cmd_table[2] = {0, 2 , {0xF7, 0x0B}},
		.dfps_cmd_table[3] = {0, 3 , {0xF0, 0xA5, 0xA5}},
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9D, .mask_list[0] = 0x9D,
	},
	// .lcm_esd_check_table[1] = {
	// 	.cmd = 0x05, .count = 1, .para_list[0] = 0x00,
	// },

	#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
		.round_corner_en = 1,
		.corner_pattern_height = ROUND_CORNER_H_TOP,
		.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
		.corner_pattern_tp_size = sizeof(top_rc_pattern),
		.corner_pattern_lt_addr = (void *)top_rc_pattern,
	#endif

	.oplus_display_global_dre = 1,
	.oplus_custom_hdr_color_tmp = true,
	.oplus_custom_hdr_red = 950,
	.oplus_custom_hdr_green = 1024,
	.oplus_custom_hdr_blue = 800,
	.oplus_panel_use_rgb_gain = true,

	.vendor = "AMS643AG01_23687",
	.manufacture = "samsung2048_23687",
#ifdef CONFIG_OPLUS_OFP_V2
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.oplus_uiready_before_time = 17,
#endif

	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2400,
		.pic_width = 1080,
		.slice_height = 40,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 989,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 631,
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
};

#define lcm_dcs_write_seq(ctx, seq...) \
	({ \
		const u8 d[] = { seq }; \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128, \
				 "DCS sequence too big for stack"); \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d)); \
	})

#define lcm_dcs_write_seq_static(ctx, seq...) \
	({ \
		static const u8 d[] = { seq }; \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d)); \
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
		dev_info(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	pr_info("%s+\n", __func__);

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("%s  0x%08x\n", __func__, buffer[0] | (buffer[1] << 8));
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
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static int panel_osc_freq_change(void *dsi, dcs_write_gce cb, void *handle, bool en)
{
	char osc_tb0[] = {0xF0, 0x5A, 0x5A};
	char osc_tb1[] = {0xFC, 0x5A, 0x5A};
	char osc_tb2[] = {0xDF, 0x09, 0x30, 0x95, 0x4E, 0x29, 0x4E, 0X29};	// OSC=96.3MHz@MIPI Speed=0.998Gbps
	char osc_tb3[] = {0xF0, 0xA5, 0xA5};
	char osc_tb4[] = {0xFC, 0xA5, 0xA5};

	pr_info("debug for %s, %d\n", __func__, en);

	if (en == 0) {            // OSC=96.3MHz@MIPI Speed=0.998Gbps
		osc_tb2[4] = 0x4E;
		osc_tb2[5] = 0x29;
		osc_tb2[6] = 0x4E;
		osc_tb2[7] = 0X29;
	} else if (en == 1) {     // OSC=95.33MHz@MIPI Speed=0.998Gbps
		osc_tb2[4] = 0x4D;
		osc_tb2[5] = 0x5F;
		osc_tb2[6] = 0x4D;
		osc_tb2[7] = 0x5F;
	}
	cb(dsi, handle, osc_tb0, ARRAY_SIZE(osc_tb0));
	cb(dsi, handle, osc_tb1, ARRAY_SIZE(osc_tb1));
	cb(dsi, handle, osc_tb2, ARRAY_SIZE(osc_tb2));
	cb(dsi, handle, osc_tb3, ARRAY_SIZE(osc_tb3));
	cb(dsi, handle, osc_tb4, ARRAY_SIZE(osc_tb4));

	osc_mipi_hopping_status = en;

	return 0;
}

static void lcm_panel_init(struct lcm *ctx)
{
	pr_info("debug for %s+\n", __func__);
	lcm_dcs_write_seq_static(ctx, 0x11, 0x00);
	usleep_range(20*1000, 21*1000);

	// DSC PPS Setting
	lcm_dcs_write_seq_static(ctx, 0x07, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x9D, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x9E, 0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x09, 0x60, 0x04,
								  0x38, 0x00, 0x28, 0x02, 0x1C, 0x02, 0x1C, 0x02, 0x00, 0x02,
								  0x0E, 0x00, 0x20, 0x03, 0xDD, 0x00, 0x07, 0x00, 0x0C, 0x02,
								  0x77, 0x02, 0x8B, 0x18, 0x00, 0x10, 0xF0, 0x03, 0x0C, 0x20,
								  0x00, 0x06, 0x0B, 0x0B, 0x33, 0x0E, 0x1C, 0x2A, 0x38, 0x46,
								  0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B, 0x7D, 0x7E, 0x01,
								  0x02, 0x01, 0x00, 0x09, 0x40, 0x09, 0xBE, 0x19, 0xFC, 0x19,
								  0xFA, 0x19, 0xF8, 0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A,
								  0xF6, 0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4, 0x00);

	lcm_dcs_write_seq_static(ctx, 0x35, 0x00);

	// Frequency Change Setting
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	pr_info("debug for %s, fps:%d\n", __func__, current_fps);
	if (current_fps == 60)
		lcm_dcs_write_seq_static(ctx, 0x60, 0x21);	//60hz
	else if (current_fps == 120)
		lcm_dcs_write_seq_static(ctx, 0x60, 0x01);	//120hz
	else
		lcm_dcs_write_seq_static(ctx, 0x60, 0x21);	//60hz
	lcm_dcs_write_seq_static(ctx, 0xF7, 0x0B);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);

	// Flat Mode Control
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x89, 0xB1);
	lcm_dcs_write_seq_static(ctx, 0xB1, 0x2F);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x03, 0xB8);
	lcm_dcs_write_seq_static(ctx, 0xB8, 0xDF);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x06, 0xB8);
	lcm_dcs_write_seq_static(ctx, 0xB8, 0xDF);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0xF8, 0xB5);
	lcm_dcs_write_seq_static(ctx, 0xB5, 0xE2, 0x0E, 0x0E, 0x0E, 0x92, 0x28, 0x76, 0x80, 0x00,
								  0x00, 0x00, 0x00, 0xFF, 0x90, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);

	// FFC Setting
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xFC, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x14, 0xDF);
	lcm_dcs_write_seq_static(ctx, 0xDF, 0x01);
	if (osc_mipi_hopping_status == 0) {
		lcm_dcs_write_seq_static(ctx, 0xDF, 0x09, 0x30, 0x95, 0x4E, 0x29, 0x4E, 0X29);    // OSC=96.3MHz@MIPI Speed=0.998Gbps
	} else if (osc_mipi_hopping_status == 1) {
		lcm_dcs_write_seq_static(ctx, 0xDF, 0x09, 0x30, 0x95, 0x4D, 0x5F, 0x4D, 0X5F);    // OSC=95.33MHz@MIPI Speed=0.998Gbps
	}
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	lcm_dcs_write_seq_static(ctx, 0xFC, 0xA5, 0xA5);

	// Brightness Control Dimming Setting
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0x53, 0x28);
	lcm_dcs_write_seq_static(ctx, 0x51, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);

	usleep_range(100*1000, 101*1000);
	lcm_dcs_write_seq_static(ctx, 0x29, 0x00);

	panel_init = true;

	pr_info("debug for %s-\n", __func__);
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

	pr_info("debug for %s+\n", __func__);

	lcm_dcs_write_seq_static(ctx, 0x28);
	usleep_range(10000, 11000);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(150*1000, 151*1000);

	ctx->error = 0;
	ctx->prepared = false;
	ctx->hbm_en = false;

	pr_info("debug for %s-\n", __func__);

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_info("debug for %s+\n", __func__);

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

	pr_info("debug for %s-\n", __func__);

	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	pr_info("debug for %s+\n", __func__);

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	pr_info("debug for %s-\n", __func__);

	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_info("debug for lcm %s+\n", __func__);

	ctx->vddr1v35_enable_gpio = devm_gpiod_get(ctx->dev,
		"vddr-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1v35_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr1v35_enable_gpio);
	usleep_range(1000, 1100);

	ctx->vci_3v_enable_gpio = devm_gpiod_get(ctx->dev,
		"vci-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vci_3v_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vci_3v_enable_gpio);
	usleep_range(5000, 5100);

	ctx->reset_gpio = devm_gpiod_get(ctx->dev,
		"reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(5000, 5100);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	usleep_range(5000, 5100);

	pr_info("debug for lcm %s-\n", __func__);

	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_info("debug for lcm %s+\n", __func__);

	ctx->reset_gpio = devm_gpiod_get(ctx->dev,
		"reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(5000, 5100);

	ctx->vci_3v_enable_gpio = devm_gpiod_get(ctx->dev,
		"vci-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vci_3v_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vci_3v_enable_gpio);
	usleep_range(1000, 1100);

	ctx->vddr1v35_enable_gpio = devm_gpiod_get(ctx->dev,
		"vddr-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1v35_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vddr1v35_enable_gpio);
	usleep_range(1000, 1100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	pr_info("debug for lcm %s-\n", __func__);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

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

static int oplus_lcm_dc_backlight(void *dsi, dcs_write_gce cb,void *handle,unsigned int level,int hbm_en)
{
	int i, k;
	struct LCM_setting_table *seed_table = NULL;
	int seed_alpha = oplus_seed_bright_to_alpha(level);
	static int last_level = OPLUS_DC_BACKLIGHT_THRESHOLD;
	static int last_seed_alpha = 0;
	if (!oplus_dc_enable_real || hbm_en || level >= OPLUS_DC_BACKLIGHT_THRESHOLD ||
		level < 4) {
		goto dc_disable;
	}

	if (((level < last_level) && (seed_alpha < last_seed_alpha)) ||
		((level > last_level) && (seed_alpha > last_seed_alpha))){
		seed_alpha = last_seed_alpha;
	}
	last_level = level;
	last_seed_alpha = seed_alpha;

	if (oplus_dc_alpha == seed_alpha)
		goto dc_enable;

	if (seed_mode == 102)
		seed_table = kmemdup(lcm_seed_mode0, sizeof(lcm_seed_mode0), GFP_KERNEL);
	else if (seed_mode == 101)
		seed_table = kmemdup(lcm_seed_mode1, sizeof(lcm_seed_mode1), GFP_KERNEL);
	else if (seed_mode == 100)
		seed_table = kmemdup(lcm_seed_mode2, sizeof(lcm_seed_mode2), GFP_KERNEL);
	else if (seed_mode == 103)
		seed_table = kmemdup(lcm_seed_mode3, sizeof(lcm_seed_mode3), GFP_KERNEL);

	if (!seed_table)
		goto dc_disable;

	pr_info("debug for lcm %s seed_alpha = %d\n", __func__, seed_alpha);

	for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(lcm_seed_mode0[0]); i++) {
		if ((seed_table[i].para_list[0] == 0x62) && (seed_table[i].count == 22)){
			for (k = 1; k < seed_table[i].count; k++) {
				seed_table[i].para_list[k]= seed_table[i].para_list[k] * (2000 - seed_alpha) / 2000;
			}
		}
	}

	for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(lcm_seed_mode0[0]); i++){
		cb(dsi, handle, seed_table[i].para_list, seed_table[i].count);
	}

	kfree(seed_table);
	if (!oplus_dc_alpha)
		pr_err("debug for lcm Enter DC");
	oplus_dc_alpha = seed_alpha;

dc_enable:
	return OPLUS_DC_BACKLIGHT_THRESHOLD;

dc_disable:
	pr_info("debug for lcm oplus_lcm_dc_backlight oplus_dc_flag = %d,oplus_dc_alpha = %d\n",oplus_dc_flag,oplus_dc_alpha);
	if (oplus_dc_alpha && level != 0 ) {
		if(oplus_dc_flag == 1) {
			if (seed_mode == 102){ //DCI-P3
				for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(struct LCM_setting_table); i++){
					cb(dsi, handle, lcm_seed_mode0[i].para_list, lcm_seed_mode0[i].count);
				}
			} else if (seed_mode == 101){ //sRGB
				for (i = 0; i < sizeof(lcm_seed_mode1)/sizeof(struct LCM_setting_table); i++){
					cb(dsi, handle, lcm_seed_mode1[i].para_list, lcm_seed_mode1[i].count);
				}
			} else if (seed_mode == 100){
				for (i = 0; i < sizeof(lcm_seed_mode2)/sizeof(struct LCM_setting_table); i++){
					cb(dsi, handle, lcm_seed_mode2[i].para_list, lcm_seed_mode2[i].count);
				}
			} else if (seed_mode == 103){
				for (i = 0; i < sizeof(lcm_seed_mode3)/sizeof(struct LCM_setting_table); i++){
					cb(dsi, handle, lcm_seed_mode3[i].para_list, lcm_seed_mode3[i].count);
				}
			}
			oplus_dc_flag = 0;
		}else {
			exit_dc_flag = 1;
		}

		pr_err("debug for lcm exit DC");
	}

	oplus_dc_alpha = 0;

	return level;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	char bl_tb0[] = {0x51, 0x00, 0x00};
	char bl_tb1[] = {0x53, 0x20};
	char post_backlight_on1[] = {0x29};

	if (!cb)
		return -1;

	pr_info("debug for %s+, level = %d\n", __func__, level);

	if ((get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) && (level > 1))
		level = 1023;

	// mapped_level = oplus_lcm_dc_backlight(dsi,cb,handle, level, 0);
	esd_brightness = level;
	mapped_level = level;

	bl_tb0[1] = mapped_level >> 8;
	bl_tb0[2] = mapped_level & 0xFF;

	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

	if (level == 1 || mapped_level == 1) {
		pr_info("enter aod mode, ignore set backlight to 1\n");
		if (aod_state == 1) {
			cb(dsi, handle, post_backlight_on1, ARRAY_SIZE(post_backlight_on1));
		}
	} else if (level <= BRIGHTNESS_HALF) {
		if (flag_hbm == 1) {
			bl_tb1[1] = 0x28;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			flag_hbm = 0;
		}
		cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	} else if (level > BRIGHTNESS_HALF && level <= BRIGHTNESS_MAX) {
		if (flag_hbm == 0) {
			bl_tb1[1] = 0xE8;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			flag_hbm = 1;
		}
		cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	}

	return 0;
}

static void lcm_setbrightness(void *dsi,
				  dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int BL_MSB = 0;
	unsigned int BL_LSB = 0;
	unsigned int hbm_brightness = 0;
	int i = 0;

	printk("debug for %s level is %d\n", __func__, level);

	if (level > BRIGHTNESS_HALF) {
		hbm_brightness = level - BRIGHTNESS_HALF;
		BL_LSB = hbm_brightness >> 8;
		BL_MSB = hbm_brightness & 0xFF;

		lcm_setbrightness_hbm[0].para_list[1] = BL_LSB;
		lcm_setbrightness_hbm[0].para_list[2] = BL_MSB;

		for (i = 0; i < sizeof(lcm_setbrightness_hbm)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, lcm_setbrightness_hbm[i].para_list, lcm_setbrightness_hbm[i].count);
		}
	} else {
		BL_LSB = level >> 8;
		BL_MSB = level & 0xFF;

		lcm_setbrightness_normal[1].para_list[1] = BL_LSB;
		lcm_setbrightness_normal[1].para_list[2] = BL_MSB;

		for (i = 0; i < sizeof(lcm_setbrightness_normal)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, lcm_setbrightness_normal[i].para_list, lcm_setbrightness_normal[i].count);
		}
	}
}

static int oplus_send_cmd_before_dsi_read(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_err("oplus_send_cmd_before_dsi_read");
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);

	return 0;
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
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (hbm_mode == 0) {
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);
		printk("debug for %s : %d ! backlight %d !\n",__func__, hbm_mode, oplus_display_brightness);
	}

	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
				  dcs_write_gce cb, void *handle, bool en)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int i = 0;
	if (!cb)
		return -1;

	pr_err("debug for oplus_display_brightness= %ld, en=%u\n", oplus_display_brightness, en);

	if(en == 1) {
		//oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 1);
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (en == 0) {
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);
		if (oplus_display_brightness <= BRIGHTNESS_HALF)
			flag_hbm = 0;
		else
			flag_hbm = 1;
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
		printk("[soso] %s : %d !  backlight %d !\n",__func__, en, oplus_display_brightness);
	}
	lcdinfo_notify(1, &en);
	ctx->hbm_en = en;
	ctx->hbm_wait = true;
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

static int lcm_esd_backlight_recovery(void *dsi, dcs_write_gce cb, void *handle)
{
	char bl_tb0[] = {0x51, 0x03, 0xFF};

	if (!cb)
		return -1;

	pr_err("debug for %s+, esd_brightness = %d\n", __func__, esd_brightness);

	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	pr_info("debug for %s-\n", __func__);

	return 1;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

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
	aod_state = false;
	return 0;
}

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

static int panel_doze_enable_start(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;

	pr_err("debug for lcm %s\n", __func__);
	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);
	//cmd = 0x10;
	//cb(dsi, handle, &cmd, 1);
	usleep_range(16*1000, 16*1000 + 100); //dalay 1 frame
	return 0;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

	pr_err("debug for lcm %s+\n", __func__);

	if (level == 0) {
		for (i = 0; i < sizeof(lcm_aod_high_mode)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, lcm_aod_high_mode[i].para_list, lcm_aod_high_mode[i].count);
		}
	} else {
		for (i = 0; i < sizeof(lcm_aod_low_mode)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, lcm_aod_low_mode[i].para_list, lcm_aod_low_mode[i].count);
		}
	}
	pr_err("debug for lcm %s- level = %d !\n",__func__, level);

	return 0;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	if (ext == NULL)
		return 1;

	if (mode == 0) {
		ext->params = &ext_params;
		current_fps = 60;
	} else if (mode == 1) {
		ext->params = &ext_params_120hz;
		current_fps = 120;
	} else
		ret = 1;

	return ret;
}

static void mode_switch_to_120(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_err("debug for lcm %s\n", __func__);

	lcm_dcs_write_seq_static(ctx,0xF0,0x5A,0x5A);
	lcm_dcs_write_seq_static(ctx,0x60,0x01);
	lcm_dcs_write_seq_static(ctx,0xF7,0x0B);
	lcm_dcs_write_seq_static(ctx,0xF0,0xA5,0xA5);
}

static void mode_switch_to_60(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_err("debug for lcm %s\n", __func__);

	lcm_dcs_write_seq_static(ctx,0xF0,0x5A,0x5A);
	lcm_dcs_write_seq_static(ctx,0x60,0x21);
	lcm_dcs_write_seq_static(ctx,0xF7,0x0B);
	lcm_dcs_write_seq_static(ctx,0xF0,0xA5,0xA5);
}

static int mode_switch(struct drm_panel *panel, unsigned int cur_mode,
	unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;

	if (dst_mode == 0) {
		mode_switch_to_60(panel);
	} else if (dst_mode == 1) {
		mode_switch_to_120(panel);
	}else
		ret = 1;

	return ret;
}

static int lcm_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *mode2;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
			dev_err(panel->drm->dev, "lcm failed to add mode %ux%ux@%u\n",
					default_mode.hdisplay, default_mode.vdisplay,
					default_mode.vrefresh);
			return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

	mode2 = drm_mode_duplicate(panel->drm, &performance_mode_120hz);
	if (!mode2) {
			dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
					performance_mode_120hz.hdisplay, performance_mode_120hz.vdisplay,
					performance_mode_120hz.vrefresh);
			return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode2);

	panel->connector->display_info.width_mm = 69;
	panel->connector->display_info.height_mm = 155;

	return 1;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	// .mode_switch = mode_switch,
	.esd_backlight_recovery = lcm_esd_backlight_recovery,
	.send_cmd_before_dsi_read = oplus_send_cmd_before_dsi_read,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.set_hbm = lcm_set_hbm,
	.doze_enable = panel_doze_enable,
	.doze_disable = panel_doze_disable,
	.doze_enable_start = panel_doze_enable_start,
	// .set_aod_light_mode = panel_set_aod_light_mode,
	.lcm_osc_change = panel_osc_freq_change,
	#ifndef CONFIG_OPLUS_OFP_V2
	.hbm_get_state = panel_hbm_get_state,
	.hbm_set_state = panel_hbm_set_state,
	.hbm_get_wait_state = panel_hbm_get_wait_state,
	.hbm_set_wait_state = panel_hbm_set_wait_state,
	#endif
};

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

	pr_err("debug for lcm %s+\n", __func__);

	if (is_probe_finish == true)
		return 0;

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);

		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_err("debug for %s(), No panel connected, skip lcm probe\n");
				return -ENODEV;
			}
			pr_info("debug for %s(), device node name = %s\n", remote_node->name);
		}
	}

	if (remote_node != dev->of_node) {
		pr_err("debug for %s(), skip lcm probe due to not current lcm\n", __func__);
		return -ENODEV;
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

	ctx->vddr1v35_enable_gpio = devm_gpiod_get(ctx->dev,
		"vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1v35_enable_gpio)) {
		dev_err(dev, "lcm cannot get vddr1v35_enable_gpio %ld\n",
			PTR_ERR(ctx->vddr1v35_enable_gpio));
		return PTR_ERR(ctx->vddr1v35_enable_gpio);
	}
	gpiod_set_value(ctx->vddr1v35_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr1v35_enable_gpio);
	usleep_range(1000, 1100);

	ctx->vci_3v_enable_gpio = devm_gpiod_get(ctx->dev,
		"vci-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci_3v_enable_gpio)) {
		dev_err(dev, "lcm cannot get vci_3v_enable_gpio %ld\n",
			PTR_ERR(ctx->vci_3v_enable_gpio));
		return PTR_ERR(ctx->vci_3v_enable_gpio);
	}
	gpiod_set_value(ctx->vci_3v_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vci_3v_enable_gpio);
	usleep_range(1000, 1100);

	ctx->reset_gpio = devm_gpiod_get(ctx->dev,
		"reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "cannot get reset-gpios %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;
	drm_panel_add(&ctx->panel);

	is_probe_finish = true;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;

	register_device_proc("lcd", "AMS643AG01_23687", "samsung2048_23687");
	disp_aal_set_dre_en(1);
	oplus_enhance_mipi_strength = 1;
	ctx->hbm_en = false;
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;

	pr_err("debug for lcm %s-\n", __func__);

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
	{
		.compatible = "oplus23687_samsung_ams667fk02_s6e8fc3_fhdp_dsi_vdo",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "oplus23687_samsung_ams667fk02_s6e8fc3_fhdp_dsi_vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);
