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
#include <soc/oplus/system/oplus_project.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_panel_ext.h"
#include "../mediatek/mtk_log.h"
#include "../mediatek/mtk_drm_graphics_base.h"
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/oplus20615_mtk_data_hw_roundedpattern.h"
#include "../mediatek/mtk_corner_pattern/oplus20615_mtk_data_hw_roundedpattern_l.h"
#include "../mediatek/mtk_corner_pattern/oplus20615_mtk_data_hw_roundedpattern_r.h"
#endif

static u32 flag_hbm;
//EXPORT_SYMBOL(flag_hbm);
static int esd_brightness = 1023;
extern int is_fan53870_pmic(void);
extern int fan53870_ldo3_20615_set_voltage(int set_uV);
extern int fan53870_ldo3_20615_disable(void);

extern int fan53870_ldo1_20817_set_voltage(int set_uV);
extern int fan53870_ldo1_20817_disable(void);
extern int fan53870_ldo5_20817_set_voltage(int set_uV);
extern int fan53870_ldo5_20817_disable(void);
extern int pmic_ldo_set_voltage_mv(unsigned int ldo_num, int set_mv);
extern int pmic_ldo_set_disable(unsigned int ldo_num);
extern void lcdinfo_notify(unsigned long val, void *v);


static int esd_brightness;
static bool aod_state = false;
static int aod_finger_unlock_flag = 0;
extern unsigned long oplus_display_brightness;
extern unsigned long oplus_max_normal_brightness;
extern unsigned long seed_mode;
extern unsigned int aod_light_mode;

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
#define MAX_NORMAL_BRIGHTNESS   2047
#define LCM_BRIGHTNESS_TYPE 2
/*#ifdef OPLUS_BUG_STABILITY*/
#define SILKY_MAX_NORMAL_BRIGHTNESS   8191
#include "../oplus/oplus_display_panel_power.h"
extern int oplus_export_drm_panel(struct drm_panel *panel_node);
extern int fan53870_ldo1_regmap_read(void);
extern int dsi_panel_parse_panel_power_cfg(struct panel_voltage_bak *panel_vol);
static PANEL_VOLTAGE_BAK panel_vol_bak[PANEL_VOLTAGE_ID_MAX] = {
						{0, 1800000, 1800000, 1800000, "vddi"},
						{1, 1104000, 1200000, 1232000, "vddr"},
						{2, 0, 1, 2, ""}};
/*#endif*/ /*OPLUS_BUG_STABILITY*/

struct ba {
	u32 brightness;
	u32 alpha;
};
static struct ba brightness_seed_alpha_lut_dc[] = {
	{12, 12},
	{30, 30},
	{70, 70},
	{90, 90},
	{105, 105},
	{142, 142},
	{225, 225},
	{300, 300},
	{525, 525},
	{650, 650},
	{800, 800},
	{970, 970},
	{1030, 1030},
	{1170, 1170},
	{1680, 1680},
	{2300, 2300},
	{2700, 2700},
	{3515, 3515},
};

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
static struct lcm *local_lcm_ctx = NULL;

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[128];
};

static struct LCM_setting_table lcm_aod_to_normal[] = {
	/*Display Off*/
	/*{REGFLAG_CMD,3,{0x9F,0x5A,0x5A}},
	{REGFLAG_CMD,1,{0x28}},
	{REGFLAG_CMD,3,{0x9F,0xA5,0xA5}},
	{REGFLAG_DELAY,20,{}},*/

	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2,{0x91,0x02}},
	{REGFLAG_CMD,2,{0x53,0x20}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
	/*seed Setting*/
	/*{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2,{0x5D,0x46}},
	{REGFLAG_CMD,2,{0x62,0x00}},
	{REGFLAG_CMD,3,{0xB0,0x16,0x62}},
	{REGFLAG_CMD,22,{0x62, 0xE0, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},*/

	//{REGFLAG_CMD,1,{0x29}},
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
	{REGFLAG_CMD,2,{0xC2,0x14}},
	{REGFLAG_CMD,2,{0x9D,0x01}},

	{REGFLAG_CMD,1,{0x11}},
	{REGFLAG_DELAY,10,{}},
	/* dcdc sm3010 power ctrl */
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,3,{0xB0,0x01,0x63}},
	{REGFLAG_CMD,2,{0x63,0x13}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
	{REGFLAG_DELAY,125,{}},


	/* TE vsync ON */
	{REGFLAG_CMD, 2, {0x35, 0x00}},

	/* CASET/PASET Setting */
	{REGFLAG_CMD, 5, {0x2A, 0x00,0x00,0x04,0x37}},
	{REGFLAG_CMD, 5, {0x2B, 0x00,0x00,0x09,0x5F}},

	/*FQ CON Setting */
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,3,{0xB0,0x27,0xF2}},
	{REGFLAG_CMD,2,{0xF2,0x00}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},

	/*seed seeting*/
/*	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2,{0x5D,0x46}},
	{REGFLAG_CMD,2,{0x62,0x00}},
	{REGFLAG_CMD,3,{0xB0,0x16,0x62}},
	{REGFLAG_CMD,22,{0x62, 0xE0, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
*/
	/*Frequency Change*/
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,3,{0x60,0x00,0x00}},
	{REGFLAG_CMD,2,{0xF7,0x0F}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},

	/* Backlight Dimming Setting */
	{REGFLAG_CMD, 2, {0x55, 0x00}},

	/* acl setting */
	{REGFLAG_CMD, 2, {0x53, 0x20}},

	/* Display On*/
	//{REGFLAG_CMD, 1, {0x29}},

	/* Display off*/
	{REGFLAG_CMD, 1, {0x28}},
	{REGFLAG_DELAY,17,{}},

	/* hbm elvss update */
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,3,{0xB0, 0xB4, 0x63}},
	{REGFLAG_CMD,12,{0x63, 0xD1, 0x0D, 0x7D, 0xDE, 0x8E, 0x53, 0xE8, 0xEF, 0x29, 0xF9, 0x4F}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
	/* aod ctrl setting */
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,3,{0xBB, 0x3D, 0x0C}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
	/*AOD On*/
	{REGFLAG_CMD,3,{0xF0, 0x5A,0x5A}},
	{REGFLAG_CMD,2,{0x91, 0X01}},
	{REGFLAG_CMD,2,{0x53, 0x24}},
	{REGFLAG_CMD,2,{0xBB, 0x3D}},//0x1D
	{REGFLAG_CMD,3,{0xF0, 0xA5,0xA5}},

	/*seed Setting*/
	/*{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2,{0x5D,0x46}},
	{REGFLAG_CMD,2,{0x62,0x00}},
	{REGFLAG_CMD,3,{0xB0,0x16,0x62}},
	{REGFLAG_CMD,22,{0x62, 0xE0, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},*/
	/* Image Data Write for AOD Mode */
	/* Display on */
	//{REGFLAG_CMD,1,{0x29}},
	//{REGFLAG_CMD,1,{0x13}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_normal_to_aod_low_light_sam[] = {
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
	{REGFLAG_CMD,2,{0xC2,0x14}},
	{REGFLAG_CMD,2,{0x9D,0x01}},

	{REGFLAG_CMD,1,{0x11}},
	{REGFLAG_DELAY,10,{}},
	/* dcdc sm3010 power ctrl */
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,3,{0xB0,0x01,0x63}},
	{REGFLAG_CMD,2,{0x63,0x13}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
	{REGFLAG_DELAY,125,{}},


	/* TE vsync ON */
	{REGFLAG_CMD, 2, {0x35, 0x00}},

	/* CASET/PASET Setting */
	{REGFLAG_CMD, 5, {0x2A, 0x00,0x00,0x04,0x37}},
	{REGFLAG_CMD, 5, {0x2B, 0x00,0x00,0x09,0x5F}},

	/*FQ CON Setting */
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,3,{0xB0,0x27,0xF2}},
	{REGFLAG_CMD,2,{0xF2,0x00}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},

	/*seed seeting*/
/*	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2,{0x5D,0x46}},
	{REGFLAG_CMD,2,{0x62,0x00}},
	{REGFLAG_CMD,3,{0xB0,0x16,0x62}},
	{REGFLAG_CMD,22,{0x62, 0xE0, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
*/
	/*Frequency Change*/
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,3,{0x60,0x00,0x00}},
	{REGFLAG_CMD,2,{0xF7,0x0F}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},

	/* Backlight Dimming Setting */
	{REGFLAG_CMD, 2, {0x55, 0x00}},

	/* acl setting */
	{REGFLAG_CMD, 2, {0x53, 0x20}},

	/* Display On*/
	//{REGFLAG_CMD, 1, {0x29}},

	/* Display off*/
	{REGFLAG_CMD, 1, {0x28}},
	{REGFLAG_DELAY,17,{}},

	/* hbm elvss update */
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,3,{0xB0, 0xB4, 0x63}},
	{REGFLAG_CMD,12,{0x63, 0xD1, 0x0D, 0x7D, 0xDE, 0x8E, 0x53, 0xE8, 0xEF, 0x29, 0xF9, 0x4F}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
	/* aod ctrl setting */
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,3,{0xBB, 0x3D, 0x0C}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
	/*AOD On*/
	{REGFLAG_CMD,3,{0xF0, 0x5A,0x5A}},
	{REGFLAG_CMD,2,{0x91, 0X01}},
	{REGFLAG_CMD,2,{0x53, 0x25}},
	{REGFLAG_CMD,2,{0xBB, 0x3D}},//0x1D
	{REGFLAG_CMD,3,{0xF0, 0xA5,0xA5}},

	/*seed Setting*/
	/*{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2,{0x5D,0x46}},
	{REGFLAG_CMD,2,{0x62,0x00}},
	{REGFLAG_CMD,3,{0xB0,0x16,0x62}},
	{REGFLAG_CMD,22,{0x62, 0xE0, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},*/
	/* Image Data Write for AOD Mode */
	/* Display on */
	//{REGFLAG_CMD,1,{0x29}},
	//{REGFLAG_CMD,1,{0x13}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_high_mode[] = {
	/* aod 50nit*/
	{REGFLAG_CMD,3, {0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2, {0x53,0x24}},
	{REGFLAG_CMD,3, {0xF0,0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_low_mode[] = {
	/* aod 10nit*/
	{REGFLAG_CMD,3, {0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2, {0x53,0x25}},
	{REGFLAG_CMD,3, {0xF0,0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_seed_mode0[] = {
	//sRGB
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x06}},
	{REGFLAG_CMD,2,{0x62, 0x00}},
	{REGFLAG_CMD,3,{0xB0, 0x01, 0x62}},
	{REGFLAG_CMD,22,{0x62,0xB8,0x00,0x06,0x3A,0xD7,0x17,0x07,0x05,0xD2,0x48,0xF2,0xDC,0xC4,0x07,0xC8,0xE9,0xE5,0x1C,0xFF,0xFF,0xFF}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};

static struct LCM_setting_table lcm_seed_mode1[] = {
	//DCI-P3
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x86}},
	{REGFLAG_CMD,2,{0x62, 0x00}},
	{REGFLAG_CMD,3,{0xB0, 0x2B, 0x62}},
	{REGFLAG_CMD,22,{0x62,0xD8,0x00,0x04,0x00,0xFF,0x02,0x00,0x00,0xFF,0x18,0xFF,0xE4,0xFB,0x00,0xF0,0xF6,0xEA,0x01,0xFF,0xFF,0xFF}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};

static struct LCM_setting_table lcm_seed_setting[] = {
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x46}},
	{REGFLAG_CMD,2,{0x62, 0x00}},
	{REGFLAG_CMD,3,{0xB0, 0x16,0x62}},
	{REGFLAG_CMD,22,{0x62, 0xE0, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};

static struct LCM_setting_table lcm_setbrightness_normal[] = {
	{REGFLAG_CMD,2, {0x53,0x20}},
	{REGFLAG_CMD,3, {0x51,0x00,0x00}},
	//{REGFLAG_CMD,3, {0xF0, 0x5A, 0x5A}},
	//{REGFLAG_CMD,2, {0xB0,0x01}},
	//{REGFLAG_CMD,2, {0xB7,0x4C}},
	//{REGFLAG_CMD,3, {0xF0, 0xA5, 0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setbrightness_hbm[] = {
	{REGFLAG_CMD,2, {0x53,0xE8}},
	{REGFLAG_CMD,3, {0x51,0x00,0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_finger_HBM_on_setting[] = {
	/*HBM ON*/
	{REGFLAG_CMD,3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2, {0x53,0xE0}},
	{REGFLAG_CMD,3, {0x51,0x0F,0xFF}},
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
	pr_err("debug for lcm %s, seed_mode: %d\n", __func__, seed_mode);
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
	lcm_dcs_write_seq_static(ctx, 0xC2, 0x14);
	lcm_dcs_write_seq_static(ctx, 0x9D, 0x01);
    /* Sleep Out(11h) */
	lcm_dcs_write_seq_static(ctx,0x11);
	msleep(10);
	/* dcdc sm3010 power ctrl */
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x01, 0x63);
	lcm_dcs_write_seq_static(ctx, 0x63, 0x13);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	msleep(85);
	/* Seed Setting */
	if (seed_mode == 101) { //sRGB
		lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
		lcm_dcs_write_seq_static(ctx, 0x5D, 0x06);
		lcm_dcs_write_seq_static(ctx, 0x62, 0x00);
		lcm_dcs_write_seq_static(ctx, 0xB0, 0x01, 0x62);
		lcm_dcs_write_seq_static(ctx, 0x62, 0xB8,0x00,0x06,0x3A,0xD7,0x17,0x07,0x05,0xD2,0x48,0xF2,0xDC,0xC4,0x07,0xC8,0xE9,0xE5,0x1C,0xFF,0xFF,0xFF);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	}else { //DCI-P3
		lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
		lcm_dcs_write_seq_static(ctx, 0x5D, 0x86);
		lcm_dcs_write_seq_static(ctx, 0x62, 0x00);
		lcm_dcs_write_seq_static(ctx, 0xB0, 0x2B, 0x62);
		lcm_dcs_write_seq_static(ctx, 0x62, 0xD8,0x00,0x04,0x00,0xFF,0x02,0x00,0x00,0xFF,0x18,0xFF,0xE4,0xFB,0x00,0xF0,0xF6,0xEA,0x01,0xFF,0xFF,0xFF);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	}
	msleep(20);
    /* TE Vsync On */
	lcm_dcs_write_seq_static(ctx, 0x35, 0x00);
    /* CASET/PASET Setting */
	lcm_dcs_write_seq_static(ctx, 0x2A, 0x00, 0x00, 0x04, 0x37);
	lcm_dcs_write_seq_static(ctx, 0x2B, 0x00, 0x00, 0x09, 0x5F);
	/*FQ CON Setting */
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x27, 0xF2);
	lcm_dcs_write_seq_static(ctx, 0xF2, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*Frequency Change*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0x60, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xF7, 0x0F);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*Loading effect compensation Control */
	/*lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x04, 0x8F);
	lcm_dcs_write_seq_static(ctx, 0x8F, 0x25);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);*/
	/* Diming setting */
	lcm_dcs_write_seq_static(ctx, 0x53, 0x20);
	/* acl setting */
	lcm_dcs_write_seq_static(ctx, 0x55, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x29);
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

	if (ctx->prepared) {
		pr_err("%s failed!!\n", __func__);
		return 0;
	}
	pr_err("debug for lcm %s\n", __func__);
	aod_state = false;

#if 0
	ctx->bias_gpio =
	devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);
#endif

	if(power_mode == 2)
	{
		pr_info("%s + lcm_panel_init,resume status\n", __func__);
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
	.clock = 251505,
	.hdisplay = 1080,
	.hsync_start = 1080 + 40,
	.hsync_end = 1080 + 40 + 10,
	.htotal = 1080 + 40 + 10 + 20,
	.vdisplay = 2400,
	.vsync_start = 2400 + 20,
	.vsync_end = 2400 + 20 + 2,
	.vtotal = 2400 + 20 + 2 + 8,
	.vrefresh = 90,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_two_para_compare = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9F, .mask_list[0] = 0x9D,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x03, .count = 1, .para_list[0] = 0x01, .mask_list[0] = 0x01,
	},
        .lcm_esd_check_table[2] = {
                .cmd = 0x05, .count = 1, .para_list[0] = 0x00, .mask_list[0] = 0x00,
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
	.data_rate = 600,
#ifdef CONFIG_OPLUS_OFP_V2
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 16,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 9,
#else
	.hbm_en_time = 1,
	.hbm_dis_time = 0,
	.oplus_need_hbm_wait = 0,
	.oplus_hbm_on_sync_with_flush = 0,
	.oplus_hbm_off_sync_with_flush = 0,
	.oplus_wait_te_num = 3,
	.oplus_samsung_panel =0,
	//before hbm need wait
	.oplus_need_before_hbm_wait= 1,
	//delay TE num
	.before_hbm_en_time = 2,
	.before_hbm_dis_time = 0,
	//delay time: us
	.before_hbm_en_delay_time =11000,
#endif
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
	.dyn = {
        .switch_en = 1,
    },
	.vendor = "AMS643YE05_SAMSUNG",
	.manufacture = "Samsung2048",
};

static struct mtk_panel_params ext_params_90hz = {
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_two_para_compare = 1,
	.lcm_esd_check_table[0] = {
        .cmd = 0x0A, .count = 1, .para_list[0] = 0x9F, .mask_list[0] = 0x9D,
	},
	.lcm_esd_check_table[1] = {
        .cmd = 0x03, .count = 1, .para_list[0] = 0x01, .mask_list[0] = 0x01,
	},
        .lcm_esd_check_table[2] = {
                .cmd = 0x05, .count = 1, .para_list[0] = 0x00, .mask_list[0] = 0x00,
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
	.data_rate = 600,
#ifdef CONFIG_OPLUS_OFP_V2
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
#else
	.hbm_en_time = 1,
	.hbm_dis_time = 0,
	.oplus_need_hbm_wait = 0,
	.oplus_hbm_on_sync_with_flush = 0,
	.oplus_hbm_off_sync_with_flush = 0,
	.oplus_wait_te_num = 3,
	.oplus_samsung_panel =0,
	//before hbm need wait
	.oplus_need_before_hbm_wait=1,
	//delay TE num
	.before_hbm_en_time = 2,
	.before_hbm_dis_time = 0,
	//delay time: us
	.before_hbm_en_delay_time =8000,
#endif
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
        .switch_en = 1, .vact_timing_fps = 90,
	},
	.dyn = {
        .switch_en = 1,
	},
	.vendor = "AMS643YE05_SAMSUNG",
	.manufacture = "Samsung2048",
};
static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	if (ext && mode == 0)
		ext->params = &ext_params;
	else if (ext && mode == 1)
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
	/* display off */
	} else if (stage == AFTER_DSI_POWERON) {
	/* display on switch to 90hz */
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0x60, 0x08 ,0x00);
	lcm_dcs_write_seq_static(ctx, 0xF7, 0x0F);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	}
}

static void mode_switch_90_to_60(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		/* display off */
	} else if (stage == AFTER_DSI_POWERON) {
		/* display on switch to 60hz */
		lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
		lcm_dcs_write_seq_static(ctx, 0x60, 0x00 ,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF7, 0x0F);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	usleep_range(26*1000, 27*1000);
	}
}

static int panel_set_seed(void *dsi, dcs_write_gce cb, void *handle, unsigned int seed_mode)
{
	int i = 0;

	pr_err("debug for lcm %s, seed_mode=%d\n", __func__, seed_mode);

	if (seed_mode == 101) {
		for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_seed_mode0[i].para_list, lcm_seed_mode0[i].count);
		}
	} else if (seed_mode == 102){
		for (i = 0; i < sizeof(lcm_seed_mode1)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_seed_mode1[i].para_list, lcm_seed_mode1[i].count);
		}
	}
	return 0;
}

static int mode_switch(struct drm_panel *panel, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;

	if (cur_mode == 0 && dst_mode == 1) { /* 60 switch to 90 */
		mode_switch_60_to_90(panel, stage);
	} else if (cur_mode == 1 && dst_mode == 0) { /* 90 switch to 60 */
		mode_switch_90_to_60(panel, stage);
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

extern bool oplus_fp_notify_down_delay;
static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	//struct lcm *ctx = panel_to_lcm(panel);
	unsigned int i=0;

	pr_err("debug for lcm %s, oplus_fp_notify_down_delay=%d\n", __func__, oplus_fp_notify_down_delay);

	if (oplus_fp_notify_down_delay)
		aod_finger_unlock_flag = 1;

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
				//judge last backlight hbm or not
				if (flag_hbm == 1) {
					if (lcm_aod_to_normal[i].para_list[0] == 0x53) {
						//send temp 0x53 order, in order not to change original lcm_aod_to_normal
						unsigned char tmp1[] = {0x53, 0xE0};
						cb(dsi, handle, tmp1, ARRAY_SIZE(tmp1));
						continue;
					}
				}
				cb(dsi, handle, lcm_aod_to_normal[i].para_list, lcm_aod_to_normal[i].count);
		}
	}
	panel_set_seed(dsi, cb, handle, seed_mode);
	usleep_range(20*1000, 21*1000);
	aod_state = false;
doze_disable_end:
	return 0;
}

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i=0;

	pr_err("debug for lcm %s\n", __func__);
	aod_state = true;

	/* normal aod light mode */
	if (aod_light_mode == 0) {
		pr_info("debug for %s normal aod light mode\n", __func__);
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
	} else {
		pr_info("debug for %s low aod light mode\n", __func__);
		for (i = 0; i < (sizeof(lcm_normal_to_aod_low_light_sam) / sizeof(struct LCM_setting_table)); i++) {
			unsigned cmd;
			cmd = lcm_normal_to_aod_low_light_sam[i].cmd;

			switch (cmd) {

				case REGFLAG_DELAY:
					msleep(lcm_normal_to_aod_low_light_sam[i].count);
					break;

				case REGFLAG_UDELAY:
					udelay(lcm_normal_to_aod_low_light_sam[i].count);
					break;

				case REGFLAG_END_OF_TABLE:
					break;

				default:
					cb(dsi, handle, lcm_normal_to_aod_low_light_sam[i].para_list, lcm_normal_to_aod_low_light_sam[i].count);
			}
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
	msleep(16);

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

	pr_err("debug for lcm %s\n", __func__);

	char post_backlight_on0[] = {0xF0,0x5A,0x5A};
	char post_backlight_on1[] = {0x29};
	char post_backlight_on2[] = {0xF0, 0xA5,0xA5};

	cb(dsi, handle, post_backlight_on0, ARRAY_SIZE(post_backlight_on0));
	cb(dsi, handle, post_backlight_on1, ARRAY_SIZE(post_backlight_on1));
	cb(dsi, handle, post_backlight_on2, ARRAY_SIZE(post_backlight_on2));

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

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	unsigned int notify_level = 0;
	char bl_tb0[] = {0x51, 0x07, 0xFF};
	char bl_tb1[] = {0x53, 0x20};

	char post_backlight_on0[] = {0xF0, 0x5A,0x5A};
	char post_backlight_on1[] = {0x29};
	char post_backlight_on2[] = {0xF0, 0xA5,0xA5};
	char HBM_ELVSS0[] = {0xF0, 0x5A,0x5A};
	char HBM_ELVSS1[] = {0xB0, 0xB4, 0x63};
	char HBM_ELVSS2[] = {0x63, 0xD1, 0x0D, 0x7D, 0xDE, 0x8E, 0x53, 0xEB, 0xEF, 0x29, 0xF9, 0x4F};
	char HBM_ELVSS3[] = {0xF0, 0xA5, 0xA5};

	if (!cb)
		return -1;
	if (level > 2047 && level <= 3327)
		level = 3327;
	mapped_level = level;
	if((mapped_level > 0)&& (mapped_level < 4))
		mapped_level = 4;
	bl_tb0[1] = mapped_level >> 8;
	bl_tb0[2] = mapped_level & 0xFF;

	esd_brightness = mapped_level;

	pr_err("debug for display panel backlight value,func:=%s,level :=%d, mapped_level := %d\n", __func__, level, mapped_level);
	if (mapped_level > 1) {
		notify_level = mapped_level * 2047 / MAX_NORMAL_BRIGHTNESS;
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &notify_level);
	}


	if(level == 1 || mapped_level == 1){
		pr_info("enter aod mode, ignore set backlight to 1\n");
		if (aod_state == 1) {
			cb(dsi, handle, post_backlight_on0, ARRAY_SIZE(post_backlight_on0));
			cb(dsi, handle, post_backlight_on1, ARRAY_SIZE(post_backlight_on1));
			cb(dsi, handle, post_backlight_on2, ARRAY_SIZE(post_backlight_on2));
		}
	} else if (level <= BRIGHTNESS_HALF){
	    if(flag_hbm == 1){
			bl_tb1[1] = 0x20;

			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			flag_hbm = 0;
		}
	    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	} else if(mapped_level > 2047 && mapped_level <= 4095){
	    if(flag_hbm == 0){
			bl_tb1[1] = 0xE0;
			cb(dsi, handle, HBM_ELVSS0, ARRAY_SIZE(HBM_ELVSS0));
			cb(dsi, handle, HBM_ELVSS1, ARRAY_SIZE(HBM_ELVSS1));
			cb(dsi, handle, HBM_ELVSS2, ARRAY_SIZE(HBM_ELVSS2));
			cb(dsi, handle, HBM_ELVSS3, ARRAY_SIZE(HBM_ELVSS3));
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			flag_hbm = 1;
			}
        pr_info("debug for bl_tb0[0]=0x%x,bl_tb0[1]=0x%x,bl_tb0[2]=0x%x\n", bl_tb0[0],bl_tb0[1],bl_tb0[2]);
	    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	}
	return 0;
}

static int oplus_panel_osc_change(void *dsi, dcs_write_gce cb, void *handle, bool en)
{
	char bl_tb0[] = {0xF0, 0x5A, 0x5A};
	char bl_tb1[] = {0xFC, 0x5A, 0x5A};
	char bl_tb2[] = {0xB0, 0x66, 0xC5};
	char bl_tb3[] = {0xC5, 0x00, 0x8C};
	char bl_tb4[] = {0xB0, 0x2A, 0xC5};
	char bl_tb5[] = {0xC5, 0x0D, 0x10, 0x80, 0x45};
	char bl_tb6[] = {0xB0, 0x3E, 0xC5};
	char bl_tb7[] = {0xC5, 0x8E, 0x82};
	char bl_tb8[] = {0xF0, 0xA5, 0xA5};
	char bl_tb9[] = {0xFC, 0xA5, 0xA5};

	pr_err("%s, %d\n", __func__, en);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
	cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
	cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));

	if (en) {
		bl_tb7[1] = 0x90;
		bl_tb7[2] = 0x37;
	} else {
		bl_tb7[1] = 0x8E;//0x8E
        bl_tb7[2] = 0x82;//0x82
	}
	cb(dsi, handle, bl_tb7, ARRAY_SIZE(bl_tb7));
	cb(dsi, handle, bl_tb8, ARRAY_SIZE(bl_tb8));
	cb(dsi, handle, bl_tb9, ARRAY_SIZE(bl_tb9));
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

	pr_err("debug for lcm %s, ctx->prepared=%d\n", __func__, ctx->prepared);

	if (ctx->prepared)
		return 0;
	lcm_panel_ldo3_enable(ctx->dev);
	//msleep(2);

	//ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	//gpiod_set_value(ctx->bias_gpio, 1);
	//devm_gpiod_put(ctx->dev, ctx->bias_gpio);

	msleep(5);
	if (3 == get_PCB_Version()) {
		ret = pmic_ldo_set_voltage_mv(1, 1504);
		if (ret < 0)
			pr_err("debug for pmic_oled ldo1 set 1500 fail\n");
	} else {
		ret = pmic_ldo_set_voltage_mv(5, 1504);
		if (ret < 0)
			pr_err("debug for pmic_oled ldo5 set 1500 fail\n");
	}
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



	if (ctx->prepared)
		return 0;

	pr_err("debug for samsung lcm %s  ctx->prepared %d \n", __func__,ctx->prepared);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(10);
    if (3 == get_PCB_Version()) {
		pmic_ldo_set_disable(1);
	} else {
		pmic_ldo_set_disable(5);
	}
    msleep(5);
	//delete for not reset fan53870 which influence camera
	//ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	//gpiod_set_value(ctx->bias_gpio, 0);
	//devm_gpiod_put(ctx->dev, ctx->bias_gpio);

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

	printk("[soso] %s level is %d\n", __func__, level);

	if (level > BRIGHTNESS_HALF) {
		hbm_brightness = level - 2047;
		BL_LSB = hbm_brightness >> 8;
		BL_MSB = hbm_brightness & 0xFF;

		lcm_setbrightness_hbm[1].para_list[1] = BL_LSB;
		lcm_setbrightness_hbm[1].para_list[2] = BL_MSB;

		for (i = 0; i < sizeof(lcm_setbrightness_hbm)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_setbrightness_hbm[i].para_list, lcm_setbrightness_hbm[i].count);
		}
	} else {
		BL_LSB = level >> 8;
		BL_MSB = level & 0xFF;

		lcm_setbrightness_normal[1].para_list[1] = BL_LSB;
		lcm_setbrightness_normal[1].para_list[2] = BL_MSB;

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
	if (!cb)
		return -1;

	pr_err("oplus_display_brightness= %ld, hbm_mode=%u\n", oplus_display_brightness, hbm_mode);
	if((oplus_display_brightness >= 0)&& (oplus_display_brightness < 4))
		oplus_display_brightness = 4;

	if(hbm_mode == 1) {
		//oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 1);
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (hbm_mode == 0) {
		//level = oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 0);
		if((oplus_display_brightness > 0)&& (oplus_display_brightness < 4))
			oplus_display_brightness = 4;
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);  //level
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

	if (!cb)
		return -1;
	if (ctx->hbm_en == en)
		goto done;

	pr_err("debug for oplus_display_brightness= %ld, en=%u\n", oplus_display_brightness, en);

	if(en == 1) {
		//oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 1);
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (en == 0) {
		if((oplus_display_brightness > 0)&& (oplus_display_brightness < 4))
			oplus_display_brightness = 4;
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
	if (aod_finger_unlock_flag) {
		ctx->hbm_wait = false;
		aod_finger_unlock_flag = 0;
	}
	else
		ctx->hbm_wait = true;
done:
	return 0;
}

static int panel_sn_set(struct drm_panel *panel)
{
    //char hbm_tb[] = {0x53, 0xe0};
    struct lcm *ctx = panel_to_lcm(panel);
    if (!ctx)
	    return -1;
    lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
    pr_err("debug for panel_sn_set 0xF0 0x5A 0x5A\n");

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
static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.doze_enable = panel_doze_enable,
	.doze_disable = panel_doze_disable,
	.doze_post_disp_on = panel_doze_post_disp_on,
	.set_hbm = lcm_set_hbm,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	//.panel_disp_off = lcm_panel_disp_off,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	#ifndef CONFIG_OPLUS_OFP_V2
	.hbm_get_state = panel_hbm_get_state,
	.hbm_set_state = panel_hbm_set_state,
	.hbm_get_wait_state = panel_hbm_get_wait_state,
	.hbm_set_wait_state = panel_hbm_set_wait_state,
	#endif
	//.panel_no_cv_switch = panel_no_video_cmd_switch_state,
	.ext_param_set = mtk_panel_ext_param_set,
	.mode_switch = mode_switch,
	.sn_set = panel_sn_set,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.lcm_osc_change = oplus_panel_osc_change,
	.set_seed = panel_set_seed,
	//.esd_backlight_recovery = oplus_esd_backlight_recovery,
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
	local_lcm_ctx = ctx;
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
    if (3 == get_PCB_Version()) {
		ret = pmic_ldo_set_voltage_mv(1, 1504);
		if (ret < 0)
			pr_err("debug for pmic_oled ldo1 set 1500 fail\n");
	} else {
		ret = pmic_ldo_set_voltage_mv(5, 1504);
		if (ret < 0)
			pr_err("debug for pmic_oled ldo5 set 1500 fail\n");
	}
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
    /*#ifdef OPLUS_BUG_STABILITY*/
	oplus_max_normal_brightness = SILKY_MAX_NORMAL_BRIGHTNESS;
    /*#endif OPLUS_BUG_STABILITY*/
	pr_info("%s samsung lcm+\n", __func__);
	register_device_proc("lcd", "AMS643YE05_SAMSUNG", "Samsung2048");
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
	{ .compatible = "oplus20615_samsung_ams643ye05_1080p_dsi_cmd", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "oplus20615_samsung_ams643ye05_1080p_dsi_cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);
