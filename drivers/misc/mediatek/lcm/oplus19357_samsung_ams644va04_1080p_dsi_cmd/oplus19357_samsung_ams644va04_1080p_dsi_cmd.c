/******************************************************************
** Copyright (C), 2004-2017, OPLUS Mobile Comm Corp., Ltd.
** OPLUS_BUG_STABILITY
** File: - oplus_samsung_ams628nw_1080p_dsi_cmd.c
** Description: Source file for lcd drvier.
** lcd driver including parameter and power control.
** Version: 1.0
** Date : 2017/12/27
**
** ------------------------------- Revision History:---------------
** liping 2017/12/27 1.0 build this module
*******************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#include <mt-plat/mtk_boot_common.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#include <platform/boot_mode.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
/*#include <mach/mt_pm_ldo.h>*/
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif
#include <linux/slab.h>
#include <linux/string.h>
#include <soc/oplus/device_info.h>

#include "ddp_hal.h"

/*#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "data_hw_roundedpattern.h"
#endif*/

/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
#include <cmdq_helper_ext.h>
/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */

#define DEBUG_INTERFACE

#define SYSFS_FOLDER "dsi_access"
#define BUFFER_LENGTH 128

/* #ifndef OPLUS_FEATURE_RAMLESS_AOD */
/* #define USE_DSI_SET_CMDQ_V4 0 */
/* #else */
#define USE_DSI_SET_CMDQ_V4 1
/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */

#define TRUE 1
#define FALSE 0

enum read_write {
	CMD_READ = 0,
	CMD_WRITE = 1,
};

#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif

static struct LCM_UTIL_FUNCS *lcm_util = NULL;

#define SET_RESET_PIN(v) (lcm_util->set_reset_pin((v)))
#define MDELAY(n) (lcm_util->mdelay(n))
#define UDELAY(n) (lcm_util->udelay(n))

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util->dsi_set_cmdq_V2(cmd, count, ppara, force_update)

#if (USE_DSI_SET_CMDQ_V4)
#define dsi_set_cmdq_V4(para_tbl, size, hs) \
			lcm_util->dsi_set_cmdq_V4(para_tbl, size, hs)
#endif

#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util->dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util->dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util->dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						lcm_util->dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)			lcm_util->dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define dsi_set_cmdq_V22(cmdq,cmd, count, ppara, force_update)	lcm_util->dsi_set_cmdq_V22(cmdq,cmd, count, ppara, force_update)

#define LCM_DSI_CMD_MODE 0
#define FRAME_WIDTH (1080)
#define FRAME_HEIGHT (2400)

#define PHYSICAL_WIDTH (67)
#define PHYSICAL_HEIGHT (149)
#define PHYSICAL_WIDTH_UM (67180)
#define PHYSICAL_HEIGHT_UM (149280)

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define REGFLAG_DELAY      0xA0
#define REGFLAG_UDELAY     0xA1

#define REGFLAG_END_OF_TABLE   0xA2
#define REGFLAG_RESET_LOW  0xA3
#define REGFLAG_RESET_HIGH  0xA4

#define BRIGHTNESS_MAX    2047
#define BRIGHTNESS_HALF   1023
#define BRIGHTNESS_MIN    2
#define BRIGHTNESS_AOD    1
#define BRIGHTNESS_OFF    0
#define BRIGHTNESS_SHIFT  256

struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

/* #ifndef OPLUS_BUG_STABILITY */
/*struct dsi_debug {
	bool long_rpkt;
	unsigned char length;
	unsigned char rlength;
	unsigned char buffer[BUFFER_LENGTH];
	unsigned char read_buffer[BUFFER_LENGTH];
	unsigned char command_len;
	unsigned char *command_buf;
	unsigned char cmds[64];
};

static struct dsi_debug debug;
static struct dsi_debug debug_read;*/
/* #endif */ /* OPLUS_BUG_STABILITY */

extern unsigned int esd_recovery_backlight_level;
extern unsigned int islcmconnected;
unsigned int rlta_enable = 1;
//extern int primary_display_is_video_mode(void);

extern unsigned long oplus_display_brightness;
static unsigned int hbm_mode_backlight_level = 2;
static bool aod_in = false;
static bool aod_state = false;
static bool aod_out = true;

static bool hbm_en;
static bool hbm_wait;

static unsigned int aod_light_brightness_mode = 0;
extern unsigned int __attribute__((weak)) get_boot_mode(void) {return 0;};

/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
extern s32 cmdqRecFlush(struct cmdqRecStruct *handle);
extern s32 cmdqRecReset(struct cmdqRecStruct *handle);
/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */

extern enum mtkfb_power_mode primary_display_get_power_mode_nolock(void);
enum mtkfb_power_mode {
	FB_SUSPEND = 0,
	FB_RESUME,
	DOZE_SUSPEND,
	DOZE,
	MTKFB_POWER_MODE_UNKNOWN,
};

#define SEED_PARAMETER {0xE0,0x00,0x06,0x10,0xFF,0x00,0x00,0x00,0xFF,0x2A,0xFF,0xE2,0xFF,0x00,0xEE,0xFF,0xF1,0x00,0xFF,0xFF,0xFF}

static struct LCM_setting_table lcm_initialization_cmd_setting[] = {
	{0x11, 0, {}},
	{REGFLAG_DELAY, 20, {}},
	/* TE vsync ON */
	{0x35, 1, {0x00}},
	/* FAIL SAFE ON */
	{0xFC, 2, {0x5A,0x5A}},
	{0xB0, 1, {0x03}},
	{0xED, 10, {0x40,0xFF,0x08,0x87,0xA4,0x4A,0x73,0xE2,0x9F,0x00}},
	{0xFC, 2, {0xA5,0xA5}},
	/* ELVSS Dim Setting */
	{0xF0, 2, {0x5A,0x5A}},
	{0xB0, 1, {0x05}},
	{0xB3, 1, {0x07}},
	{0xF0, 2, {0xA5,0xA5}},
	/* ACL off */
	{0x55, 1, {0x00}},
	/* Backlight Dimming Setting */
	{0x53, 1, {0x20}},
	{0xF0, 2, {0x5A,0x5A}},
	{0xF2, 1, {0x0F}},
	{0xF0, 2, {0xA5,0xA5}},
	{REGFLAG_DELAY, 5, {}},
	/* Fast Discharge Set */
	{0xF0, 2, {0x5A,0x5A}},
	{0xB0, 1, {0x01}},
	{0xCD, 1, {0x01}},
	{0xF0, 2, {0xA5,0xA5}},
	/* PCD OFF Set */
	{0xF0, 2, {0x5A,0x5A}},
	{0xEA, 1, {0x48}},
	{0xF0, 2, {0xA5,0xA5}},
	/* Seed CRC mode enable */
	{0xF0, 2, {0x5A,0x5A}},
	{0x80, 1, {0x92}},
	{0xB1, 1, {0x00}},
	{0xB0, 2, {0x2B,0xB1}},
	{0xB1, 21, SEED_PARAMETER},
	{0xB0, 2, {0x55,0xB1}},
	{0xB1, 1, {0x80}},
	{0xF0, 2, {0xA5,0xA5}},
	{REGFLAG_DELAY, 110, {}},
	/* Display On*/
	{0x29, 0, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	{0x28, 0, {}},
	{REGFLAG_DELAY, 10, {}},
	{0x10, 0, {}},
	{REGFLAG_DELAY, 150, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if 1
static struct LCM_setting_table lcm_aod_in_setting[] = {
	{0x28, 0, {}},
	{REGFLAG_DELAY, 10, {}},
	{0x10, 0, {}},
	{REGFLAG_DELAY, 150, {}},

	/* Internal VDO Packet generation enable*/
	{0xF0, 2, {0x5A,0x5A}},
	{0xFC, 2, {0x5A,0x5A}},
	{0xB0, 2, {0x14,0xFE}},
	{0xFE, 1, {0x12}},
	{0xF0, 2, {0xA5,0xA5}},
	{0xFC, 2, {0xA5,0xA5}},

	/*Sleep out*/
	{0x11, 0, {}},
	{REGFLAG_DELAY, 20, {}},

	/* MIPI Mode cmd */
	{0xF0, 2, {0x5A,0x5A}},
	{0xF2, 1, {0x03}},
	{0xF0, 2, {0xA5,0xA5}},

	/* TE vsync ON */
	{0x35, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},

	/* AOD Setting */
	{0x81, 44, {0x38,0x1B,0x43,0x2B,0xC4,0x1A,
				0x17,0x44,0x43,0x85,0x28,
				0x1B,0x82,0x82,0x58,0xED,
				0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,
				0x00,0xFF,0xFF,0xFF,0x00,
				0x00,0x00,0x00}},

				/* PCD OFF Set */
	{0xF0, 2, {0x5A,0x5A}},
	{0xEA, 1, {0x48}},
	{0xF0, 2, {0xA5,0xA5}},

	/* AOD AMP ON */
	{0xFC, 2, {0x5A,0x5A}},
	{0xB0, 2, {0x06,0xFD}},
	{0xFD, 1, {0x85}},
	{0xFC, 2, {0xA5,0xA5}},

	/* AOD Mode On Setting */
	{0x53, 1, {0x22}},

	/* Internal VDO Packet generation enable*/
	{0xF0, 2, {0x5A,0x5A}},
	{0xFC, 2, {0x5A,0x5A}},
	{0xB0, 2, {0x14,0xFE}},
	{0xFE, 1, {0x10}},
	{0xF0, 2, {0xA5,0xA5}},
	{0xFC, 2, {0xA5,0xA5}},

	/*AOD IP Setting*/
	{0xF0, 2, {0x5A,0x5A}},
	{0xB0, 2, {0x03,0xC2}},
	{0xC2, 1, {0x04}},
	{0xF0, 2, {0xA5,0xA5}},

	/* Seed CRC mode enable */
	{0xF0, 2, {0x5A,0x5A}},
	{0x80, 1, {0x92}},
	{0xB1, 1, {0x00}},
	{0xB0, 2, {0x2B,0xB1}},
	{0xB1, 21, SEED_PARAMETER},
	{0xB0, 2, {0x55,0xB1}},
	{0xB1, 1, {0x80}},
	{0xF0, 2, {0xA5,0xA5}},

	{REGFLAG_DELAY,100,{}},

	#if 1
	/* Image Data Write for AOD Mode */
	/* Display On*/
	{0x29, 0, {}},

	/* MIPI Video cmd*/
	{0xF0, 2, {0x5A,0x5A}},
	{0xF2, 1, {0x0F}},
	{0xF0, 2, {0xA5,0xA5}},
	#endif
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif
#if (USE_DSI_SET_CMDQ_V4)
static struct LCM_setting_table_V3 lcm_aod_in_setting_v3[] = {
	{REGFLAG_ESCAPE_ID, 0x28, 0, {}},
	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
	{REGFLAG_ESCAPE_ID, 0x10, 0, {}},
	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 122, {}},

	/* Internal VDO Packet generation enable*/
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, 0xFC, 2, {0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, 0xB0, 2, {0x14,0xFE}},
	{REGFLAG_ESCAPE_ID, 0xFE, 1, {0x12}},
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0xA5,0xA5}},
	{REGFLAG_ESCAPE_ID, 0xFC, 2, {0xA5,0xA5}},

	/*Sleep out*/
	{REGFLAG_ESCAPE_ID, 0x11, 0, {}},
	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 20, {}},

	/* MIPI Mode cmd */
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, 0xF2, 1, {0x03}},
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0xA5,0xA5}},

	/* TE vsync ON */
	{REGFLAG_ESCAPE_ID, 0x35, 1, {0x00}},
	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},

	/* AOD Setting */
	/*
	{REGFLAG_ESCAPE_ID, 0x81, 44, {0x38,0x1B,0x43,0x1C,0x83,0x08,
				0x15,0xF4,0x33,0x64,0x62,
				0x19,0x33,0x80,0xA9,0x08,
				0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,
				0x00,0xAC,0x00,0x00,0x00,
				0x00,0xFF,0xFF,0xFF,0x00,
				0x00,0x00,0x00}},
				*/

	/* PCD OFF Set */
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, 0xEA, 1, {0x48}},
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0xA5,0xA5}},

	/* AOD AMP ON */
	{REGFLAG_ESCAPE_ID, 0xFC, 2, {0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, 0xB0, 2, {0x06,0xFD}},
	{REGFLAG_ESCAPE_ID, 0xFD, 1, {0x85}},
	{REGFLAG_ESCAPE_ID, 0xFC, 2, {0xA5,0xA5}},

	/* AOD Mode On Setting */
	{REGFLAG_ESCAPE_ID, 0x53, 1, {0x22}},

	/* Internal VDO Packet generation enable*/
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, 0xFC, 2, {0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, 0xB0, 2, {0x14,0xFE}},
	{REGFLAG_ESCAPE_ID, 0xFE, 1, {0x10}},
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0xA5,0xA5}},
	{REGFLAG_ESCAPE_ID, 0xFC, 2, {0xA5,0xA5}},

	/*AOD IP Setting*/
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, 0xB0, 2, {0x03,0xC2}},
	{REGFLAG_ESCAPE_ID, 0xC2, 1, {0x04}},
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0xA5,0xA5}},

	/* Seed CRC mode enable */
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, 0x80, 1, {0x92}},
	{REGFLAG_ESCAPE_ID, 0xB1, 1, {0x00}},
	{REGFLAG_ESCAPE_ID, 0xB0, 2, {0x2B,0xB1}},
	{REGFLAG_ESCAPE_ID, 0xB1, 21, SEED_PARAMETER},
	{REGFLAG_ESCAPE_ID, 0xB0, 2, {0x55,0xB1}},
	{REGFLAG_ESCAPE_ID, 0xB1, 1, {0x80}},
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0xA5,0xA5}},

	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3,100,{}},

	#if 1
	/* Image Data Write for AOD Mode */
	/* Display On*/
	{REGFLAG_ESCAPE_ID, 0x29, 0, {}},

	/* MIPI Video cmd*/
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, 0xF2, 1, {0x0F}},
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0xA5,0xA5}}
	#endif

	//{REGFLAG_ESCAPE_ID, REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct LCM_setting_table lcm_seed_setting[] = {
	{0xF0, 2, {0x5A,0x5A}},
	{0x80, 1, {0x92}},
	{0xB1, 1, {0x00}},
	{0xB0, 2, {0x2B,0xB1}},
	{0xB1, 21, SEED_PARAMETER},
	{0xB0, 2, {0x55,0xB1}},
	{0xB1, 1, {0x80}},
	{0xF0, 2, {0xA5,0xA5}},
};

struct ba {
	u32 brightness;
	u32 alpha;
};

static struct ba brightness_seed_alpha_lut_dc[] = {
	{0, 0xff},
	{1, 0xfc},
	{2, 0xfb},
	{3, 0xfa},
	{4, 0xf9},
	{5, 0xf8},
	{6, 0xf7},
	{8, 0xf6},
	{10, 0xf4},
	{15, 0xf0},
	{20, 0xea},
	{30, 0xe0},
	{45, 0xd0},
	{70, 0xbc},
	{100, 0x98},
	{120, 0x80},
	{140, 0x70},
	{160, 0x58},
	{180, 0x48},
	{200, 0x30},
	{220, 0x20},
	{240, 0x10},
	{260, 0x00},
};

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

	for (i = 0; i < ARRAY_SIZE(brightness_seed_alpha_lut_dc); i++){
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


#if (USE_DSI_SET_CMDQ_V4)
static struct LCM_setting_table_V3 lcm_aod_display_on_setting_v3[] = {
#if 1
	/* Image Data Write for AOD Mode */
	/* Display On*/
	{REGFLAG_ESCAPE_ID, 0x29, 0, {}},

	/* MIPI Video cmd*/
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, 0xF2, 1, {0x0F}},
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0xA5,0xA5}},

	//{REGFLAG_END_OF_TABLE, 0x00, {}}
#else
	/* Display On*/
	{REGFLAG_ESCAPE_ID, 0x29, 0, {}}
#endif
};

#else

static struct LCM_setting_table lcm_aod_display_on_setting[] = {
	/* Image Data Write for AOD Mode */
	/* Display On*/
	{0x29, 0, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#endif

static struct LCM_setting_table lcm_aod_from_display_on[] = {
	/* AOD MODE ON Setting*/
	{0xF0, 2, {0x5A,0x5A}},
	{0x53, 1, {0x22}},
	{0xF0, 2, {0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_out_setting[] = {
	/* Display off */
	//{0x28, 0, {}},
	/* MIPI Video cmd*/
	{0xF0, 2, {0x5A,0x5A}},
	{0xF2, 1, {0x0F}},
	{0xF0, 2, {0xA5,0xA5}},

	//{REGFLAG_DELAY, 33, {}},
#if 0
	/* AOD AMP Off */
	{0xFC, 2, {0x5A,0x5A}},
	{0xB0, 2, {0x05,0xFD}},
	{0xFD, 1, {0x6F}},
	{0xFC, 2, {0xA5,0xA5}},
#endif
	/* AOD Mode off Setting */
	{0x53, 1, {0x20}},

	/*seed setting*/
	{0xF0, 2, {0x5A,0x5A}},
	{0x80, 1, {0x92}},
	{0xB1, 1, {0x00}},
	{0xB0, 2, {0x2B,0xB1}},
	{0xB1, 21,{0xE0,0x00,0x06,0x10,0xFF,0x00,0x00,0x00,0xFF,0x2A,0xFF,0xE2,0xFF,0x00,0xEE,0xFF,0xF1,0x00,0xFF,0xFF,0xFF}},
	{0xB0, 2, {0x55,0xB1}},
	{0xB1, 1, {0x80}},
	{0xF0, 2, {0xA5,0xA5}},

	/* Display on */
	{0x29, 0, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if (USE_DSI_SET_CMDQ_V4)
static struct LCM_setting_table_V3 lcm_aod_out_setting_v3[] = {
	/* Display off */
	//{REGFLAG_ESCAPE_ID, 0x28, 0, {}},
	/* MIPI Video cmd*/
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, 0xF2, 1, {0x0F}},
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0xA5,0xA5}},

	//{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 33, {}},
#if 0
	/* AOD AMP Off */
	{REGFLAG_ESCAPE_ID, 0xFC, 2, {0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, 0xB0, 2, {0x05,0xFD}},
	{REGFLAG_ESCAPE_ID, 0xFD, 1, {0x6F}},
	{REGFLAG_ESCAPE_ID, 0xFC, 2, {0xA5,0xA5}},
#endif
	/* AOD Mode off Setting */
	{REGFLAG_ESCAPE_ID, 0x53, 1, {0x20}},

	/*seed setting*/
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, 0x80, 1, {0x92}},
	{REGFLAG_ESCAPE_ID, 0xB1, 1, {0x00}},
	{REGFLAG_ESCAPE_ID, 0xB0, 2, {0x2B,0xB1}},
	{REGFLAG_ESCAPE_ID, 0xB1, 21,{0xE0,0x00,0x06,0x10,0xFF,0x00,0x00,0x00,0xFF,0x2A,0xFF,0xE2,0xFF,0x00,0xEE,0xFF,0xF1,0x00,0xFF,0xFF,0xFF}},
	{REGFLAG_ESCAPE_ID, 0xB0, 2, {0x55,0xB1}},
	{REGFLAG_ESCAPE_ID, 0xB1, 1, {0x80}},
	{REGFLAG_ESCAPE_ID, 0xF0, 2, {0xA5,0xA5}},

	/* Display on */
	{REGFLAG_ESCAPE_ID, 0x29, 0, {}}

	//{REGFLAG_ESCAPE_ID, REGFLAG_END_OF_TABLE, 0x00, {}}
};

#endif

static struct LCM_setting_table lcm_aod_out_from_hbm_setting[] = {
	/* Backlight Dimming Setting */
	{0xF0, 2, {0x5A,0x5A}},
	{0xB0, 1, {0x05}},
	{0xB3, 1, {0x07}},
	{0xF0, 2, {0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_setbrightness_normal[] = {
	{0x51, 2, {0x00,0x00}},
	{0xF0, 2, {0x5A,0x5A}},
	{0x53, 1, {0x20}},
	{0xB7, 1, {0x02}},
	{0xF0, 2, {0xA5,0xA5}}
};

static struct LCM_setting_table lcm_setbrightness_hbm[] = {
	{0x51, 2, {0x00,0x00}},
	{0xF0, 2, {0x5A,0x5A}},
	{0x53, 1, {0xE8}},
	{0xB7, 1, {0x02}},
	{0xF0, 2, {0xA5,0xA5}}
};

static struct LCM_setting_table lcm_normal_HBM_on_setting[] = {
	{0xF0, 2, {0x5A,0x5A}},
	{0x51, 2, {0x03,0xFF}},
	{0x53, 1, {0xE0}},
	{0xB7, 1, {0x02}},
	{0xF0, 2, {0xA5,0xA5}},
};

static struct LCM_setting_table lcm_finger_HBM_on_setting[] = {
	{0xF0, 2, {0x5A,0x5A}},
	{0x51, 2, {0x02,0xFF}},
	{0x53, 1, {0xE0}},
	{0xB7, 1, {0x00}},
	{0xF0, 2, {0xA5,0xA5}},
};

static struct LCM_setting_table lcm_aod_high_mode[] = {
	/* aod 50nit*/
	{0xF0, 2, {0x5A,0x5A}},
	{0x53, 1, {0x22}},
	{0xF0, 2, {0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_low_mode[] = {
	/* aod 10nit*/
	{0xF0, 2, {0x5A,0x5A}},
	{0x53, 1, {0x23}},
	{0xF0, 2, {0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_safe_on_mode[] = {
	/* FAIL SAFE Setting */
	{0xFC, 2, {0x5A,0x5A}},
	{0xB0, 1, {0x03}},
	{0xED, 10, {0x40,0xFF,0x08,0x87,0xA4,0x4A,0x73,0xE2,0x9F,0x00}},
	{0xFC, 2, {0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_safe_off_mode[] = {
	/* FAIL SAFE Setting */
	{0xFC, 2, {0x5A,0x5A}},
	{0xB0, 1, {0x03}},
	{0xED, 10, {0x40,0x04,0x08,0x87,0x84,0x4A,0x73,0xE2,0x1F,0x00}},
	{0xFC, 2, {0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
static struct LCM_setting_table lcm_aod_to_normal_setting[] = {
	/* Backlight setting */
	{0x51, 2, {0x00, 0x00}},
	{0xF0, 2, {0x5A, 0x5A}},
	{0x53, 1, {0x20}},
	{0xB7, 1, {0x02}},
	{0xF0, 2, {0xA5, 0xA5}},

	/* Display off */
	/* {0x28, 0, {}}, */
	/* MIPI Video cmd*/
	{0xF0, 2, {0x5A, 0x5A}},
	{0xF2, 1, {0x0F}},
	{0xF0, 2, {0xA5, 0xA5}},

	/* {REGFLAG_DELAY, 33, {}}, */
#if FALSE
	/* AOD AMP Off */
	{0xFC, 2, {0x5A, 0x5A}},
	{0xB0, 2, {0x05, 0xFD}},
	{0xFD, 1, {0x6F}},
	{0xFC, 2, {0xA5, 0xA5}},
#endif
	/* AOD Mode off Setting */
	{0x53, 1, {0x20}},

	/*seed setting*/
	{0xF0, 2, {0x5A, 0x5A}},
	{0x80, 1, {0x92}},
	{0xB1, 1, {0x00}},
	{0xB0, 2, {0x2B, 0xB1}},
	{0xB1, 21, {0xE0, 0x00, 0x06, 0x10, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x2A, 0xFF, 0xE2, 0xFF, 0x00, 0xEE, 0xFF, 0xF1, 0x00, 0xFF, 0xFF, 0xFF}},
	{0xB0, 2, {0x55, 0xB1}},
	{0xB1, 1, {0x80}},
	{0xF0, 2, {0xA5, 0xA5}},

	/* Display on */
	{0x29, 0, {}},
	/* {0x51, 2, {0x01, 0xFF}}, */

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_normal_to_aod_setting[] = {
	{0x28, 0, {}},
	{REGFLAG_DELAY, 10, {}},
	{0x10, 0, {}},
	{REGFLAG_DELAY, 150, {}},

	/* Internal VDO Packet generation enable*/
	{0xF0, 2, {0x5A, 0x5A}},
	{0xFC, 2, {0x5A, 0x5A}},
	{0xB0, 2, {0x14, 0xFE}},
	{0xFE, 1, {0x12}},
	{0xF0, 2, {0xA5, 0xA5}},
	{0xFC, 2, {0xA5, 0xA5}},

	/*Sleep out*/
	{0x11, 0, {}},
	{REGFLAG_DELAY, 20, {}},

	/* MIPI Mode cmd */
	{0xF0, 2, {0x5A, 0x5A}},
	{0xF2, 1, {0x03}},
	{0xF0, 2, {0xA5, 0xA5}},

	/* TE vsync ON */
	{0x35, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},
#if FALSE
	/* AOD Setting */
	{0x81, 44, {0x38, 0x1B, 0x43, 0x2B, 0xC4, 0x1A,
				0x17, 0x44, 0x43, 0x85, 0x28,
				0x1B, 0x82, 0x82, 0x58, 0xED,
				0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0xFF, 0xFF, 0xFF, 0x00,
				0x00, 0x00, 0x00}},
#endif
	/* PCD OFF Set */
	{0xF0, 2, {0x5A, 0x5A}},
	{0xEA, 1, {0x48}},
	{0xF0, 2, {0xA5, 0xA5}},

	/* AOD AMP ON */
	{0xFC, 2, {0x5A, 0x5A}},
	{0xB0, 2, {0x06, 0xFD}},
	{0xFD, 1, {0x85}},
	{0xFC, 2, {0xA5, 0xA5}},

	/* AOD Mode On Setting */
	{0x53, 1, {0x22}},

	/* Internal VDO Packet generation enable*/
	{0xF0, 2, {0x5A, 0x5A}},
	{0xFC, 2, {0x5A, 0x5A}},
	{0xB0, 2, {0x14, 0xFE}},
	{0xFE, 1, {0x10}},
	{0xF0, 2, {0xA5, 0xA5}},
	{0xFC, 2, {0xA5, 0xA5}},

	/*AOD IP Setting*/
	{0xF0, 2, {0x5A, 0x5A}},
	{0xB0, 2, {0x03, 0xC2}},
	{0xC2, 1, {0x04}},
	{0xF0, 2, {0xA5, 0xA5}},

	/* Seed CRC mode enable */
	{0xF0, 2, {0x5A, 0x5A}},
	{0x80, 1, {0x92}},
	{0xB1, 1, {0x00}},
	{0xB0, 2, {0x2B, 0xB1}},
	{0xB1, 21, {0xE0, 0x00, 0x06, 0x10, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x2A, 0xFF, 0xE2, 0xFF, 0x00, 0xEE, 0xFF, 0xF1, 0x00, 0xFF, 0xFF, 0xFF}},
	{0xB0, 2, {0x55, 0xB1}},
	{0xB1, 1, {0x80}},
	{0xF0, 2, {0xA5, 0xA5}},

	{REGFLAG_DELAY, 100, {}},

	#if 1
	/* Image Data Write for AOD Mode */
	/* Display On*/
	{0x29, 0, {}},

	/* MIPI Video cmd*/
	{0xF0, 2, {0x5A, 0x5A}},
	{0xF2, 1, {0x0F}},
	{0xF0, 2, {0xA5, 0xA5}},
	#endif
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table_on_ramless_aod(void *cmdq, struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY:
			cmdqRecFlush(cmdq);
			cmdqRecReset(cmdq);
			if (table[i].count <= 10) {
				MDELAY(table[i].count);
			} else {
				MDELAY(table[i].count);
			}
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, 1);
		}
	}
}
/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY :
			if (table[i].count <= 10) {
				MDELAY(table[i].count);
			} else {
				MDELAY(table[i].count);
			}
			break;
		case REGFLAG_UDELAY :
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE :
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void push_table22(void *handle,struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY :
			if (table[i].count <= 10) {
				MDELAY(table[i].count);
			} else {
				MDELAY(table[i].count);
			}
		break;

		case REGFLAG_UDELAY :
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE :
			break;

		default:
			dsi_set_cmdq_V22(handle, cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	if (lcm_util == NULL) {
		lcm_util = kmalloc(sizeof(struct LCM_UTIL_FUNCS),GFP_KERNEL);
	}
	memcpy(lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
	int boot_mode = 0;

	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	//params->physical_width = PHYSICAL_WIDTH;
	//params->physical_height = PHYSICAL_HEIGHT;
	params->physical_width_um = PHYSICAL_WIDTH_UM;
	params->physical_height_um = PHYSICAL_HEIGHT_UM;

	params->dsi.mode   = BURST_VDO_MODE;

	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active                = 2;
	params->dsi.vertical_backporch          = 9;
	params->dsi.vertical_frontporch         = 21;
	params->dsi.vertical_active_line                = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active      = 16;
	params->dsi.horizontal_backporch        = 22;
	params->dsi.horizontal_frontporch       = 54;
	params->dsi.horizontal_active_pixel         = FRAME_WIDTH;

	//mipi clk 553.5MHz
	params->dsi.PLL_CLOCK = 553;
	params->dsi.data_rate = 1107;

	//mipi dynamic 562 MHz
	params->dsi.dynamic_switch_mipi = 1;
	params->dsi.data_rate_dyn = 1117;
	params->dsi.horizontal_backporch_dyn = 33;

	params->dsi.ssc_disable = 1;
	params->dsi.CLK_TRAIL = 10;
	params->dsi.HS_PRPR = 9;
	params->dsi.CLK_HS_PRPR = 9;

	/* clk continuous video mode */
	params->dsi.cont_clock = 0;

	params->dsi.clk_lp_per_line_enable = 0;
	if (get_boot_mode() == META_BOOT) {
		boot_mode++;
		LCD_DEBUG("META_BOOT\n");
	}
	if (get_boot_mode() == ADVMETA_BOOT) {
		boot_mode++;
		LCD_DEBUG("ADVMETA_BOOT\n");
	}
	if (get_boot_mode() == ATE_FACTORY_BOOT) {
		boot_mode++;
		LCD_DEBUG("ATE_FACTORY_BOOT\n");
	}
	if (get_boot_mode() == FACTORY_BOOT) {
		boot_mode++;
		LCD_DEBUG("FACTORY_BOOT\n");
	}
#ifndef BUILD_LK
	if (boot_mode == 0) {
		LCD_DEBUG("neither META_BOOT or FACTORY_BOOT\n");
		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 0;
		//params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
		//params->dsi.lcm_esd_check_table[0].count = 1;
		//params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
	}
#endif

/*#ifndef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->corner_pattern_height = ROUND_CORNER_H_TOP;
	params->corner_pattern_height_bot = ROUND_CORNER_H_BOT;
	params->corner_pattern_tp_size = sizeof(top_rc_pattern);
	params->corner_pattern_lt_addr = (void *)top_rc_pattern;
#endif*/


	params->hbm_en_time = 1;
	params->hbm_dis_time = 0;


#ifndef BUILD_LK
	memcpy(params->vendor, "S644VA04_MTK04", strlen("S644VA04_MTK04") > 31?31:strlen("S644VA04_MTK04"));
	memcpy(params->manufacture, "samsung1024 cmd mode", strlen("samsung1024 cmd mode") > 31?31:strlen("samsung1024 cmd mode"));
	register_device_proc("lcd", "S644VA04_MTK04", "samsung1024 cmd mode");
#endif
}

static void poweron_before_ulps(void)
{
	//lcd_vpoc_setting(1);
	//MDELAY(2);
	lcd_vci_setting(1);
	MDELAY(6);
	lcd_rst_setting(1);
	MDELAY(2);
	lcd_rst_setting(0);
	MDELAY(2);
	lcd_rst_setting(1);
	MDELAY(2);
	LCD_DEBUG("[soso] poweron_before_ulps\n");
}

static void lcm_init_power(void)
{
	LCD_DEBUG("[soso] lcm_init_power\n");
}

static void lcm_suspend_power(void)
{
	aod_in = false;
	aod_state = false;
	aod_out = true;
	LCD_DEBUG("[soso] lcm_suspend_power\n");
}

static void poweroff_after_ulps(void)
{
	lcd_rst_setting(0);
	MDELAY(12);
	lcd_vci_setting(0);
	//MDELAY(2);
	//lcd_vpoc_setting(0);
	LCD_DEBUG("[soso] poweroff_after_ulps\n");
}

static void lcm_aod(int enter)
{
	if (enter == 1) {
		aod_in = true;
		aod_out = false;
		/*
		if (aod_light_brightness_mode == 0) {
			lcm_aod_in_setting_v3[24].para_list[0] = 0x22;
		} else {
			lcm_aod_in_setting_v3[24].para_list[0] = 0x23;
		}
		*/
		lcd_rst_setting(0);
		MDELAY(2);
		lcd_rst_setting(1);
		MDELAY(6);
#if (USE_DSI_SET_CMDQ_V4)
		printk("[soso] lcm_aod_in_setting_v3 =  %d \n",ARRAY_SIZE(lcm_aod_in_setting_v3));
		dsi_set_cmdq_V4(lcm_aod_in_setting_v3, ARRAY_SIZE(lcm_aod_in_setting_v3), 0);
#else
		push_table(lcm_aod_in_setting, sizeof(lcm_aod_in_setting) / sizeof(struct LCM_setting_table), 1);
#endif
	}
	else{
		//aod_in = false;
		//aod_out = true;
#if (USE_DSI_SET_CMDQ_V4)
		printk("[soso] lcm_aod_out_setting_v3 =  %d \n",ARRAY_SIZE(lcm_aod_out_setting_v3));
		dsi_set_cmdq_V4(lcm_aod_out_setting_v3, ARRAY_SIZE(lcm_aod_out_setting_v3), 0);
#else
		push_table(lcm_aod_out_setting, sizeof(lcm_aod_out_setting) / sizeof(struct LCM_setting_table), 1);
#endif
	}
	printk("[soso] lcm aod %d \n",enter);
}

static void lcm_aod_display_on(void *handle)
{
if (0)
{
#if (USE_DSI_SET_CMDQ_V4)
	dsi_set_cmdq_V4(lcm_aod_display_on_setting_v3, ARRAY_SIZE(lcm_aod_display_on_setting_v3), 0);
#else
	push_table22(handle,lcm_aod_display_on_setting, sizeof(lcm_aod_display_on_setting) / sizeof(struct LCM_setting_table), 1);
#endif
}
	LCD_DEBUG("[soso] lcm_aod_display_on \n");
}

static void lcm_aod_out(void *handle)
{

	if (0) {
	if (hbm_en) {
		push_table22(handle,lcm_aod_out_from_hbm_setting, sizeof(lcm_aod_out_from_hbm_setting) / sizeof(struct LCM_setting_table), 1);
		LCD_DEBUG("[soso] aod out from hbm\n");
	} else {
		push_table22(handle,lcm_aod_out_setting, sizeof(lcm_aod_out_setting) / sizeof(struct LCM_setting_table), 1);
		LCD_DEBUG("[soso] aod out\n");
	}
	}
	LCD_DEBUG("[soso] lcm_aod_out already init at cv switch \n");

}

static void disp_lcm_aod_from_display_on(void)
{
	aod_state = true;
	aod_out = false;
	LCD_DEBUG("[soso] aod from display on\n");
}

static void lcm_resume_power(void)
{
	lcm_init_power();
	LCD_DEBUG("[soso] lcm_resume_power \n");
}


static void lcm_init(void)
{
	MDELAY(5);
	push_table(lcm_initialization_cmd_setting, sizeof(lcm_initialization_cmd_setting) / sizeof(struct LCM_setting_table), 1);
	hbm_en = false;
	LCD_DEBUG("[soso]lcm_initialization_setting\n");
}

static void lcm_suspend(void)
{
	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
	hbm_en = false;
	LCD_DEBUG("[soso] lcm_suspend \n");
}

static void lcm_resume(void)
{
	lcm_init();
	LCD_DEBUG("[soso] lcm_resume\n");
}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB<<8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16)|(y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}

#define OPLUS_DC_BACKLIGHT_THRESHOLD 260

extern int oplus_dc_enable_real;
extern int oplus_dc_alpha;
static int oplus_lcm_dc_backlight(void *handle, unsigned int level, int hbm_en)
{
	int i, k;
	struct LCM_setting_table *seed_table;
	int seed_alpha = 0;

	/*
	if (!primary_display_is_video_mode()) {
		return level;
	}
	*/

	if (oplus_dc_enable_real) {
		if ((level == 3) || (level == 2)) {
			level = 4;
		}
	}

	seed_alpha = oplus_seed_bright_to_alpha(level);

	if (!oplus_dc_enable_real || hbm_en || level >= OPLUS_DC_BACKLIGHT_THRESHOLD ||
		level < BRIGHTNESS_MIN) {
		goto dc_disable;
	}

	if (oplus_dc_alpha == seed_alpha)
		goto dc_enable;

	seed_table = kmemdup(lcm_seed_setting, sizeof(lcm_seed_setting), GFP_KERNEL);
	if (!seed_table)
		goto dc_disable;

	for (i = 0; i < sizeof(lcm_seed_setting)/sizeof(lcm_seed_setting[0]); i++) {
		if (seed_table[i].count != 21 && seed_table[i].cmd != 0xB1)
			continue;
		if (i == 4) {
			for (k = 0; k < seed_table[i].count; k++)
				seed_table[i].para_list[k] = seed_table[i].para_list[k] * (255 - seed_alpha) / 255;
		}
	}

	if (handle)
		push_table22(handle,seed_table,
			sizeof(lcm_seed_setting)/sizeof(lcm_seed_setting[0]), 1);
	else
		push_table(seed_table,
			sizeof(lcm_seed_setting)/sizeof(lcm_seed_setting[0]), 1);

	kfree(seed_table);
	if (!oplus_dc_alpha)
		pr_err("Enter DC");

	oplus_dc_alpha = seed_alpha;

dc_enable:
	return OPLUS_DC_BACKLIGHT_THRESHOLD;

dc_disable:
	if (oplus_dc_alpha) {
		if (handle)
			push_table22(handle,lcm_seed_setting,
				sizeof(lcm_seed_setting)/sizeof(lcm_seed_setting[0]), 1);
		else
			push_table(lcm_seed_setting,
				sizeof(lcm_seed_setting)/sizeof(lcm_seed_setting[0]), 1);
		pr_err("exit DC");
	}

	oplus_dc_alpha = 0;
	return level;
}

static void lcm_setbrightness(void *handle, unsigned int level)
{
	unsigned int BL_MSB = 0;
	unsigned int BL_LSB = 0;
	unsigned int hbm_brightness = 0;

	printk("[soso] %s level is %d\n", __func__, level);

	/*
	if (!primary_display_is_video_mode()) {
		return;
	}
	*/

	if (level > BRIGHTNESS_HALF) {
		hbm_brightness = level;
		BL_LSB = hbm_brightness / BRIGHTNESS_SHIFT;
		BL_MSB = hbm_brightness % BRIGHTNESS_SHIFT;

		lcm_setbrightness_hbm[0].para_list[0] = BL_LSB;
		lcm_setbrightness_hbm[0].para_list[1] = BL_MSB;

		push_table22(handle,lcm_setbrightness_hbm,
			sizeof(lcm_setbrightness_hbm)/sizeof(lcm_setbrightness_hbm[0]), 1);
	} else {
		BL_LSB = level / BRIGHTNESS_SHIFT;
		BL_MSB = level % BRIGHTNESS_SHIFT;

		lcm_setbrightness_normal[0].para_list[0] = BL_LSB;
		lcm_setbrightness_normal[0].para_list[1] = BL_MSB;

		push_table22(handle,lcm_setbrightness_normal,
			sizeof(lcm_setbrightness_normal)/sizeof(lcm_setbrightness_normal[0]), 1);
	}
}

static void lcm_aod_to_normal_backlight_mapping(void *handle, unsigned int level)
{
	unsigned int bl_msb = 0;
	unsigned int bl_lsb = 0;
	unsigned int mapped_level = 0;

	/*
	if (!primary_display_is_video_mode()) {
		return;
	}
	*/

	mapped_level = oplus_lcm_dc_backlight(handle, level, hbm_en);
	printk("[soso] %s level:%d, mapped_level:%d\n", __func__, level, mapped_level);

	if (level > BRIGHTNESS_HALF) {
		bl_lsb = mapped_level / BRIGHTNESS_SHIFT;
		bl_msb = mapped_level % BRIGHTNESS_SHIFT;

		lcm_aod_to_normal_setting[0].para_list[0] = bl_lsb;
		lcm_aod_to_normal_setting[0].para_list[1] = bl_msb;
		lcm_aod_to_normal_setting[2].para_list[0] = lcm_setbrightness_hbm[2].para_list[0];

	} else {
		bl_lsb = mapped_level / BRIGHTNESS_SHIFT;
		bl_msb = mapped_level % BRIGHTNESS_SHIFT;

		lcm_aod_to_normal_setting[0].para_list[0] = bl_lsb;
		lcm_aod_to_normal_setting[0].para_list[1] = bl_msb;
		lcm_aod_to_normal_setting[2].para_list[0] = lcm_setbrightness_normal[2].para_list[0];
	}
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;

	if (!islcmconnected) {
		return;
	}

	if (aod_in && !aod_state) {
		aod_in = false;
		aod_state = true;
		lcm_aod_display_on(handle);
	} else {
		if (level >= BRIGHTNESS_MAX) {
			mapped_level = BRIGHTNESS_MAX;
		} else if (level < BRIGHTNESS_OFF) {
			mapped_level = BRIGHTNESS_OFF;
		} else {
			mapped_level = level;
		}

		if (level == BRIGHTNESS_AOD)
			return;

		if (mapped_level != BRIGHTNESS_OFF)
			esd_recovery_backlight_level = mapped_level;

		if (!aod_out && mapped_level >= BRIGHTNESS_OFF && aod_state) {
			aod_in = false;
			aod_out = true;
			aod_state = false;
			lcm_aod_out(handle);
			if (primary_display_get_power_mode_nolock() == DOZE) {
				hbm_mode_backlight_level = mapped_level;
				printk("[soso] %s, aod out from doze, skip backlight setting!", __func__);
				return;
			}
		}

		hbm_mode_backlight_level = mapped_level;
		mapped_level = oplus_lcm_dc_backlight(handle, mapped_level, hbm_en);

		if (!hbm_en) {
			lcm_setbrightness(handle,mapped_level);
		}
	}
}

static void lcm_set_hbm_mode(void *handle,unsigned int hbm_level)
{
	//hbm_en is 670nit,no need response hbm node set
	if (hbm_en) {
		return;
	}

	//hbm_level 1 for no dimming layer need finger HBM 670nit.other for normal HBM 600nit.
	if (hbm_level == 1) {
		push_table22(handle,lcm_finger_HBM_on_setting,
			 sizeof(lcm_finger_HBM_on_setting)/sizeof(lcm_finger_HBM_on_setting[0]),1);
		oplus_lcm_dc_backlight(handle, hbm_mode_backlight_level, 1);
	} else if ((hbm_level == 0) || (hbm_level == 2)) {
		int level = oplus_lcm_dc_backlight(handle, hbm_mode_backlight_level, 0);
		lcm_setbrightness(handle,level);
	} else {
		oplus_lcm_dc_backlight(handle, hbm_mode_backlight_level, 1);
		push_table22(handle,lcm_normal_HBM_on_setting,
			 sizeof(lcm_normal_HBM_on_setting)/sizeof(lcm_normal_HBM_on_setting[0]),1);
	}
	printk("[soso] %s : %d !\n",__func__, hbm_level);
}

static bool lcm_get_hbm_state(void)
{
	return hbm_en;
}

static bool lcm_get_hbm_wait(void)
{
	return hbm_wait;
}

static bool lcm_set_hbm_wait_ramless(bool wait, void *qhandle)
{
	bool old = hbm_wait;

	hbm_wait = wait;

	/*
	 * update backlight after enter fingerprint hbm mode
	 */
	if (!wait && hbm_en)
		oplus_lcm_dc_backlight(qhandle, 0, 1);

	return old;
}

static bool lcm_set_hbm_cmdq(bool en, void *qhandle)
{
	bool old = hbm_en;

	if (hbm_en == en)
		goto done;

	//only for dimming layer need finger HBM 670 nit.
	if (en) {
		push_table22(qhandle,lcm_finger_HBM_on_setting,
			sizeof(lcm_finger_HBM_on_setting)/sizeof(lcm_finger_HBM_on_setting[0]),1);
		printk("[soso] %s : %d !\n",__func__, en);
	} else {
		if (!aod_state) {
			int level = oplus_lcm_dc_backlight(qhandle, hbm_mode_backlight_level, 0);
			lcm_setbrightness(qhandle,level);
			printk("[soso] %s : %d ! backlight %d !\n",__func__, en, level);
		} else {
				if (aod_light_brightness_mode == 0) {
					lcm_aod_from_display_on[1].para_list[0] = 0x22;
				} else {
					lcm_aod_from_display_on[1].para_list[0] = 0x23;
				}
				if (qhandle == NULL) {
					push_table(lcm_aod_from_display_on, sizeof(lcm_aod_from_display_on) / sizeof(struct LCM_setting_table), 1);
					printk("[soso] %s : %d ! wfp back aod from doze_suspend!\n",__func__, en);
				} else {
					push_table22(qhandle,lcm_aod_from_display_on, sizeof(lcm_aod_from_display_on) / sizeof(struct LCM_setting_table), 1);
					printk("[soso] %s : %d ! wfp back aod!\n",__func__, en);
				}
		}
	}

	hbm_en = en;
	lcm_set_hbm_wait_ramless(true, qhandle);

done:
	return old;
}

/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
/* mode 0: video->cmd, mode 1: cmd->video */
static void lcm_set_aod_cv_mode(void *qhandle, unsigned int mode)
{
	if (mode == 0) {
		push_table_on_ramless_aod(qhandle, lcm_normal_to_aod_setting,
			sizeof(lcm_normal_to_aod_setting)/sizeof(lcm_normal_to_aod_setting[0]), 1);
		printk("[soso] %s : %d !\n",__func__, mode);
	} else {
		lcm_aod_to_normal_backlight_mapping(qhandle, oplus_display_brightness);
		push_table_on_ramless_aod(qhandle, lcm_aod_to_normal_setting,
			sizeof(lcm_aod_to_normal_setting)/sizeof(lcm_aod_to_normal_setting[0]), 1);
		printk("[soso] %s : %d !\n", __func__, mode);
	}
}
/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */

static void lcm_set_aod_brightness(void *qhandle, unsigned int mode)
{
	if (mode == 0) {
		push_table22(qhandle, lcm_aod_high_mode,
			sizeof(lcm_aod_high_mode)/sizeof(lcm_aod_high_mode[0]), 1);
		printk("[soso] %s : %d !\n", __func__, mode);
	} else {
		push_table22(qhandle, lcm_aod_low_mode,
			sizeof(lcm_aod_low_mode)/sizeof(lcm_aod_low_mode[0]), 1);
		printk("[soso] %s : %d !\n", __func__, mode);
	}
	aod_light_brightness_mode = mode;
}

void lcm_set_safe_mode(void *qhandle,unsigned int mode)
{
	if (mode == 0) {
		push_table22(qhandle,lcm_safe_off_mode,
			sizeof(lcm_safe_off_mode)/sizeof(lcm_safe_off_mode[0]),1);
		printk("[soso] %s : %d !\n",__func__, mode);
	} else {
		push_table22(qhandle,lcm_safe_on_mode,
			sizeof(lcm_safe_on_mode)/sizeof(lcm_safe_on_mode[0]),1);
		printk("[soso] %s : %d !\n",__func__, mode);
	}
}

/* #ifndef OPLUS_BUG_STABILITY */
/*int parse_input(void *handle,const char *buf)
{
	int retval = 0;
	int index = 0;
	int cmdindex = 0;
	int ret = 0;
	char *input = (char *)buf;
	char *token = NULL;
	unsigned long value = 0;

	printk("[soso][DISP] parse_input %s\n", buf);

	input[strlen(input)] = '\0';

	while (input != NULL && index < BUFFER_LENGTH) {
		token = strsep(&input, " ");
		retval = kstrtoul(token, 16, &value);
		if (retval < 0) {
			printk("[soso] %s: Failed to convert from string (%s) to hex number\n", __func__, token);
			continue;
		}
		debug.buffer[index] = (unsigned char)value;
		index++;
	}

	if (index > 1) {
		debug.length = index - 1;
	}

	if (debug.length <= 0) {
		return 0;
	}

	while (cmdindex < debug.length) {
		debug.cmds[cmdindex] = debug.buffer[cmdindex+1];
		cmdindex++;
	}
	printk("[soso] %s debug.buffer[0] is 0x%x debug.length = %d \n",
		__func__, debug.buffer[0],debug.length);

	while (ret < debug.length) {
		printk("[soso] debug.cmds is 0x%x\n",  debug.cmds[ret]);
		ret++;
	}
	dsi_set_cmdq_V22(handle, debug.buffer[0], debug.length, debug.cmds, 1);

	return 1;
}

extern unsigned char read_buffer[128];
extern int reg_rlengh;

int parse_reg_output(void *handle,const char *buf)
{
	int retval = 0;
	int index = 0;
	int ret = 0;
	char *input = (char *)buf;
	char *token = NULL;
	unsigned long value = 0;

	input[strlen(input)] = '\0';

	while (input != NULL && index < BUFFER_LENGTH) {
		token = strsep(&input, " ");
		retval = kstrtoul(token, 16, &value);
		if (retval < 0) {
			printk("[soso] %s: Failed to convert from string (%s) to hex number\n",
				__func__, token);
			continue;
		}
		debug_read.buffer[index] = (unsigned char)value;
		index++;
	}

	if (index > 1) {
		debug_read.length = debug_read.buffer[1];
	}

	if (debug_read.length <= 0) {
		return 0;
	}

	reg_rlengh = debug_read.length;

	printk("[soso] %s debug.buffer[0] is 0x%x debug.length = %d \n",
		__func__, debug_read.buffer[0],debug_read.length);

	retval = read_reg_v2(debug_read.buffer[0],
				debug_read.read_buffer, debug_read.length);

	if (retval<0) {
		printk("[soso] %s error can not read the reg 0x%x \n",
			__func__,debug_read.buffer[0]);
		return -1;
	}

	while (ret < debug_read.length) {
		printk("[soso] reg cmd 0x%x read_buffer is 0x%x \n",
				debug_read.buffer[0], debug_read.read_buffer[ret]);
		read_buffer[ret] = debug_read.read_buffer[ret];
		ret++;
	}
	return 1;
}*/
/* #endif */ /* OPLUS_BUG_STABILITY */

static void lcm_doze_enable(void *handle)
{
	pr_err("%s %d\n", __func__, __LINE__);
	push_table22(handle, lcm_aod_in_setting, ARRAY_SIZE(lcm_aod_in_setting), 1);
}
static void lcm_doze_dizable(void *handle)
{
	pr_err("%s %d\n", __func__, __LINE__);
	push_table22(handle, lcm_aod_out_setting, ARRAY_SIZE(lcm_aod_out_setting), 1);
}

static struct LCM_setting_table lcm_aod_area[] = {
	/* AOD Setting */
	{0x81, 44, {0x3F,0x0F,0x06,0x09,0xF2,0x6B,
				0x15,0xE4,0x29,0xD3,0x15,
				0x0F,0x06,0x32,0x93,0xD3,
				0x15,0xE4,0x3E,0x24,0x19,
				0x00,0x00,0x00,0x00,0x00,
				0x1B,0x82,0x79,0xD8,0x65,
				0x60,0x01,0x10,0x00,0x00,
				0x00,0xFF,0xFF,0xFF,0x00,
				0x00,0xFF,0x00,}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if (USE_DSI_SET_CMDQ_V4)
static struct LCM_setting_table_V3 lcm_aod_area_v3[] = {
	/* AOD Setting */
	{REGFLAG_ESCAPE_ID, 0x81, 44, {0x3F,0x0F,0x06,0x09,0xF2,0x6B,
				0x15,0xE4,0x29,0xD3,0x15,
				0x0F,0x06,0x32,0x93,0xD3,
				0x15,0xE4,0x3E,0x24,0x19,
				0x00,0x00,0x00,0x00,0x00,
				0x1B,0x82,0x79,0xD8,0x65,
				0x60,0x01,0x10,0x00,0x00,
				0x00,0xFF,0xFF,0xFF,0x00,
				0x00,0xFF,0x00,}}

	//{REGFLAG_ESCAPE_ID,REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static void lcm_set_aod_area(void *handle, unsigned char *area)
{
	unsigned int i = 0;

	if (!islcmconnected) {
		return;
	}

	printk("[soso] %s aod area update!\n", __func__);

#if (USE_DSI_SET_CMDQ_V4)
	for(i = 0; i<lcm_aod_area_v3[0].count; i++) {

		lcm_aod_area_v3[0].para_list[i] = area[i];
		//pr_err("%s lcm_aod_area[0].para_list[%d] = 0x%x\n", __func__, i, lcm_aod_area_v3[0].para_list[i]);
	}

	dsi_set_cmdq_V4(lcm_aod_area_v3, ARRAY_SIZE(lcm_aod_area_v3), 0);
#else
	for(i = 0; i<lcm_aod_area[0].count; i++) {

		lcm_aod_area[0].para_list[i] = area[i];
		//pr_err("%s lcm_aod_area[0].para_list[%d] = 0x%x\n", __func__, i, lcm_aod_area[0].para_list[i]);
	}

	push_table(lcm_aod_area, sizeof(lcm_aod_area) / sizeof(struct LCM_setting_table), 1);
#endif
}

 void lcm_set_aod_area_test(void *handle)
{
	if (!islcmconnected) {
		return;
	}

		push_table22(handle,lcm_aod_area,
			sizeof(lcm_aod_area)/sizeof(lcm_aod_area[0]), 1);
}

struct LCM_DRIVER oplus19357_samsung_ams644va04_1080p_dsi_cmd_lcm_drv=
{
	.name = "oplus19357_samsung_ams644va04_1080p_dsi_cmd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.update = lcm_update,
	.poweron_before_ulps = poweron_before_ulps,
	.poweroff_after_ulps = poweroff_after_ulps,
	.get_hbm_state = lcm_get_hbm_state,
	.set_hbm_cmdq = lcm_set_hbm_cmdq,
	.get_hbm_wait = lcm_get_hbm_wait,
	.set_hbm_wait_ramless = lcm_set_hbm_wait_ramless,
	.set_hbm_mode_cmdq = lcm_set_hbm_mode,
	.aod = lcm_aod,
	.disp_lcm_aod_from_display_on = disp_lcm_aod_from_display_on,
	.set_aod_brightness = lcm_set_aod_brightness,
	.set_safe_mode = lcm_set_safe_mode,
	/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
	.set_aod_area_cmdq = lcm_set_aod_area,
	.doze_enable = lcm_doze_enable,
	.doze_disable = lcm_doze_dizable,
	.set_aod_cv_mode = lcm_set_aod_cv_mode,
	/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */
};
