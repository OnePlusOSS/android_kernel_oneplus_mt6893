/********************************************
 ** Copyright (C) 2019 OPLUS Mobile Comm Corp. Ltd.
 ** ODM_HQ_EDIT
 ** File: nt36672c_fhdp_dsi_vdo_boe_t0_zal1878.c
 ** Description: Source file for LCD driver
 **          To Control LCD driver
 ** Version:1.0
 ** Date : 2019/10/14
 ** Author: Liyan@ODM_HQ.Multimedia.LCD
 ** ---------------- Revision History: --------------------------
 ** <version>    <date>          < author >              <desc>
 **  1.0           2019/10/14   Liyan@ODM_HQ   Source file for LCD driver
 ********************************************/

#define LOG_TAG "LCM_NT36672C_TIANMA_SALA"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#include <mt-plat/mtk_boot_common.h>
#endif
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "data_hw_roundedpattern.h"
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
#include "disp_dts_gpio.h"
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define NT36672C_LCM_ID_SALA (0x05)


/*static const unsigned int BL_MIN_LEVEL = 20;*/
#ifndef BUILD_LK
typedef struct LCM_UTIL_FUNCS LCM_UTIL_FUNCS;
typedef struct LCM_PARAMS LCM_PARAMS;
typedef struct LCM_DRIVER LCM_DRIVER;
#endif
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define SET_LCM_VSP_PIN(v)  (lcm_util.set_gpio_lcd_enp_bias((v)))
#define SET_LCM_VSN_PIN(v)  (lcm_util.set_gpio_lcd_enn_bias((v)))
#define MDELAY(n)        (lcm_util.mdelay(n))
#define UDELAY(n)        (lcm_util.udelay(n))


#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifdef CONFIG_SET_LCD_BIAS_ODM_HQ
#define SET_LCD_BIAS_EN(en, seq, value)                           lcm_util.set_lcd_bias_en(en, seq, value)
#endif

#ifndef BUILD_LK
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>
#endif

#define MTK_GPIO_DESC_BASE 301
#define GPIO_LCD_VSP_EN (MTK_GPIO_DESC_BASE + 23)
#define GPIO_LCD_VSN_EN (MTK_GPIO_DESC_BASE + 202)

/* static unsigned char lcd_id_pins_value = 0xFF; */
/*static const unsigned char LCD_MODULE_ID = 0x01;
extern int gesture_flag;wuxuewen temp delete
extern void lcd_queue_load_tp_fw(void);wuxuewen temp delete
extern nvt_tp;  wuxuewen temp delete
*/
/* zyw: FIX ME */
/*extern bool is_sala_a(void);*/
extern bool __attribute__((weak)) is_sala_a(void) {return 0;}

bool is_sala_three_camera(void)
{
	if ((get_Operator_Version() == 90) || (get_Operator_Version() == 92))
		return true;
	else
		return false;
}

#define LCM_DSI_CMD_MODE    0
#define FRAME_WIDTH        (1080)
#define FRAME_HEIGHT    (2400)
#define LCM_DENSITY        (320)

#define LCM_PHYSICAL_WIDTH        (67716)
#define LCM_PHYSICAL_HEIGHT        (150480)

#define REGFLAG_DELAY        0xFFFC
#define REGFLAG_UDELAY    0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW    0xFFFE
#define REGFLAG_RESET_HIGH    0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static int blmap_table_v1[] = {
	54, 23,
	22, 33,
	25, 31,
	29, 28,
	29, 28,
	29, 28,
	32, 24,
	35, 17,
	38, 9,
	35, 18,
	38, 9,
	38, 9,
	41, 3,
	41, 4,
	41, 2,
	48, 33,
	48, 32,
	54, 65,
	54, 64,
	61, 106,
	58, 87,
	64, 128,
	67, 150,
	67, 148,
	77, 222,
	77, 222,
	77, 223,
	87, 308,
	87, 308,
	96, 392,
	90, 335,
	106, 495,
	106, 496,
	116, 600,
	119, 627,
	125, 696,
	129, 743,
	135, 815,
	151, 1011,
	141, 888,
	154, 1053,
	174, 1315,
	170, 1263,
	183, 1437,
	193, 1579,
	203, 1723,
	196, 1622,
	222, 2016,
	232, 2172,
	232, 2172,
	245, 2377,
	274, 2850,
	264, 2680,
	303, 3342,
	300, 3292,
	312, 3505,
	351, 4200,
	341, 4013,
	361, 4385,
	383, 4800,
	400, 5128,
	416, 5440,
	470, 6510,
	280, 2678
};

static int blmap_table_v2[] = {
	0, 142,
	0, 142,
	0, 142,
	0, 142,
	0, 142,
	0, 142,
	0, 142,
	0, 142,
	0, 142,
	0, 142,
	0, 142,
	38, 9,
	41, 3,
	41, 4,
	41, 2,
	48, 33,
	48, 32,
	54, 65,
	54, 64,
	61, 106,
	58, 87,
	64, 128,
	67, 150,
	67, 148,
	77, 222,
	77, 222,
	77, 223,
	87, 308,
	87, 308,
	96, 392,
	90, 335,
	106, 495,
	106, 496,
	116, 600,
	119, 627,
	125, 696,
	129, 743,
	135, 815,
	151, 1011,
	141, 888,
	154, 1053,
	174, 1315,
	170, 1263,
	183, 1437,
	193, 1579,
	203, 1723,
	196, 1622,
	222, 2016,
	232, 2172,
	232, 2172,
	245, 2377,
	274, 2850,
	264, 2680,
	303, 3342,
	300, 3292,
	312, 3505,
	351, 4200,
	341, 4013,
	361, 4385,
	383, 4800,
	400, 5128,
	416, 5440,
	470, 6510,
	280, 2678
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 10, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 60, {} },
};

static struct LCM_setting_table init_setting_vdo[] = {
	{0xFF, 1, {0x23}},/* 12bit PWM */
	{0xFB, 1, {0x01}},/* 12bit PWM */
	{0x00, 1, {0x80}},/* 12bit PWM */
	{0x07, 1, {0x00}},
	{0x08, 1, {0x01}},
	{0x09, 1, {0x00}},
	{0x05, 1, {0x2d}},/*dimming speed*/
	{0x0A, 1, {0x00}},/*cabc begin*/
	{0x0B, 1, {0x00}},
	{0x0C, 1, {0x00}},
	{0x0D, 1, {0x00}},
	{0x11, 1, {0x01}},
	{0x12, 1, {0x95}},
	{0x15, 1, {0x68}},
	{0x16, 1, {0x0B}},
	{0x30, 1, {0xFA}},
	{0x31, 1, {0xF6}},
	{0x32, 1, {0xF4}},
	{0x33, 1, {0xF4}},
	{0x34, 1, {0xEF}},
	{0x35, 1, {0xED}},
	{0x36, 1, {0xED}},
	{0x37, 1, {0xEC}},
	{0x38, 1, {0xEC}},
	{0x39, 1, {0xEC}},
	{0x3A, 1, {0xDF}},
	{0x3B, 1, {0xDC}},
	{0x3D, 1, {0xDC}},
	{0x3F, 1, {0xDC}},
	{0x40, 1, {0xDA}},
	{0x41, 1, {0xD2}},
	{0x45, 1, {0xE2}},
	{0x46, 1, {0xDC}},
	{0x47, 1, {0xCD}},
	{0x48, 1, {0xC5}},
	{0x49, 1, {0xBB}},
	{0x4A, 1, {0xB3}},
	{0x4B, 1, {0xB1}},
	{0x4C, 1, {0xB0}},
	{0x4D, 1, {0xAB}},
	{0x4E, 1, {0xAA}},
	{0x4F, 1, {0xA7}},
	{0x50, 1, {0xA1}},
	{0x51, 1, {0x9B}},
	{0x52, 1, {0x97}},
	{0x53, 1, {0x92}},
	{0x54, 1, {0x89}},
	{0x58, 1, {0xD8}},
	{0x59, 1, {0xD2}},
	{0x5A, 1, {0xCA}},
	{0x5B, 1, {0xC2}},
	{0x5C, 1, {0xBA}},
	{0x5D, 1, {0xAA}},
	{0x5E, 1, {0xAA}},
	{0x5F, 1, {0xAA}},
	{0x60, 1, {0xAA}},
	{0x61, 1, {0xA9}},
	{0x62, 1, {0xA0}},
	{0x63, 1, {0x98}},
	{0x64, 1, {0x90}},
	{0x65, 1, {0x8A}},
	{0x66, 1, {0x84}},
	{0x67, 1, {0x7A}},
	{0x6F, 1, {0x00}},
	{0xA0, 1, {0x01}},
	{0xFF, 1, {0xF0}},
	{0xFB, 1, {0x01}},
	{0xD2, 1, {0x52}},/*cabc end*/

	{0xFF, 1, {0x24}},
	{0xFB, 1, {0x01}},
	{0xE3, 1, {0x03}},

	{0xFF, 1, {0x25}},
	{0xFB, 1, {0x01}},
	{0x21, 1, {0x40}},
	{0xFF, 1, {0x20}},
	{0xFB, 1, {0x01}},
	{0x31, 1, {0x56}},

	{0xFF, 1, {0x2A}},
	{0xFB, 1, {0x01}},
	{0x28, 1, {0xE6}},
	{0x29, 1, {0x0C}},
	{0x2A, 1, {0x1C}},
	{0x2D, 1, {0x06}},
	{0x2F, 1, {0x02}},
	{0x30, 1, {0x4A}},
	{0x33, 1, {0x1D}},
	{0x34, 1, {0xE8}},
	{0x35, 1, {0x32}},
	{0x36, 1, {0x07}},
	{0x37, 1, {0xE3}},
	{0x38, 1, {0x36}},
	{0x39, 1, {0x03}},
	{0x3A, 1, {0x4A}},

	{0XFF, 1, {0XF0}},
	{0XFB, 1, {0X01}},
	{0X5A, 1, {0X00}},
	{0XFF, 1, {0XD0}},
	{0XFB, 1, {0X01}},
	{0X53, 1, {0X22}},
	{0X54, 1, {0X02}},

	{0xFF, 1, {0x25}},
	{0xFB, 1, {0x01}},
	{0x18, 1, {0x20}},
	{0xFF, 1, {0xF0}},
	{0xFB, 1, {0x01}},
	{0xA0, 1, {0x08}},
	{0xFF, 1, {0x10}},
	{0xFB, 1, {0x01}},
	{0x3B, 5, {0X03, 0X14, 0X36, 0X04, 0X04}},
	{0xB0, 1, {0x01}},
	{0xC0, 1, {0X00}},

	{0x35, 1, {0x00}}, /* TE*/
	{0x55, 1, {0x01}},
	{0x68, 2, {0x04, 0x01}},
	{0x53, 1, {0x24}},
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 10, {} },
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 70, {} },
	/* bist mode */
	/*0XFF, 1, {0X20}},*/
	/*0XFB, 1, {0X01}},*/
	/*0X86, 1, {0X03}},*/
};

static struct LCM_setting_table bl_level[] = {
	{0x51, 3, {0x0F, 0xFF, 0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
static struct LCM_setting_table bl_level_dimming_exit[] = {
	{0x53, 1, {0x24}},
	{0x51, 3, {0x0F, 0xFF, 0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
static struct LCM_setting_table lcm_cabc_enter_setting_ui[] = {
	{0x53, 1, {0x2C}},
	{0x55, 1, {0x01}},
	/*    {0x53, 1, {0x24}},*/
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
static struct LCM_setting_table lcm_cabc_enter_setting_still[] = {
	{0x53, 1, {0x2C}},
	{0x55, 1, {0x02}},
	/*    {0x53, 1, {0x24}},*/
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
static struct LCM_setting_table lcm_cabc_enter_setting_moving[] = {
	{0x53, 1, {0x2C}},
	{0x55, 1, {0x03}},
	/*    {0x53, 1, {0x24}},*/
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
static struct LCM_setting_table lcm_cabc_exit_setting[] = {
	{0x53, 1, {0x2C}},
	{0x55, 1, {0x00}},
	/*    {0x53, 1, {0x24}},*/
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
		unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V22(cmdq, cmd,
					table[i].count,
					table[i].para_list,
					force_update);
		}
	}
}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
static void lcm_dfps_int(struct LCM_DSI_PARAMS *dsi)
{
	struct dfps_info *dfps_params = dsi->dfps_params;

	dsi->dfps_enable = 1;
	dsi->dfps_default_fps = 6000;/*real fps * 100, to support float*/
	dsi->dfps_def_vact_tim_fps = 9000;/*real vact timing fps * 100*/

	/*traversing array must less than DFPS_LEVELS*/
	/*DPFS_LEVEL0*/
	dfps_params[0].level = DFPS_LEVEL0;
	dfps_params[0].fps = 6000;/*real fps * 100, to support float*/
	dfps_params[0].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/*if mipi clock solution*/
	/*dfps_params[0].PLL_CLOCK = xx;*/
	/*dfps_params[0].data_rate = xx; */
	/*if HFP solution*/
	/*dfps_params[0].horizontal_frontporch = xx;*/
	dfps_params[0].vertical_frontporch = 1291;
	dfps_params[0].vertical_frontporch_for_low_power = 2500;

	/*if need mipi hopping params add here*/
	dfps_params[0].dynamic_switch_mipi = 1;
	dfps_params[0].PLL_CLOCK_dyn = 550;
	dfps_params[0].horizontal_frontporch_dyn = 288;
	dfps_params[0].vertical_frontporch_dyn = 1291;
	dfps_params[0].vertical_frontporch_for_low_power_dyn = 2500;

	/*DPFS_LEVEL1*/
	dfps_params[1].level = DFPS_LEVEL1;
	dfps_params[1].fps = 9000;/*real fps * 100, to support float*/
	dfps_params[1].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/*if mipi clock solution*/
	/*dfps_params[1].PLL_CLOCK = xx;*/
	/*dfps_params[1].data_rate = xx; */
	/*if HFP solution*/
	/*dfps_params[1].horizontal_frontporch = xx;*/
	dfps_params[1].vertical_frontporch = 54;
	dfps_params[1].vertical_frontporch_for_low_power = 2500;

	/*if need mipi hopping params add here*/
	dfps_params[1].dynamic_switch_mipi = 1;
	dfps_params[1].PLL_CLOCK_dyn = 550;
	dfps_params[1].horizontal_frontporch_dyn = 288;
	dfps_params[1].vertical_frontporch_dyn = 54;
	dfps_params[1].vertical_frontporch_for_low_power_dyn = 2500;

	/*DPFS_LEVEL2*/
	dfps_params[2].level = DFPS_LEVEL2;
	dfps_params[2].fps = 4500;/*real fps * 100, to support float*/
	dfps_params[2].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/*if mipi clock solution*/
	/*dfps_params[1].PLL_CLOCK = xx;*/
	/*dfps_params[1].data_rate = xx; */
	/*if HFP solution*/
	/*dfps_params[1].horizontal_frontporch = xx;*/
	dfps_params[2].vertical_frontporch = 2500;
	dfps_params[2].vertical_frontporch_for_low_power = 2500;

	/*if need mipi hopping params add here*/
	dfps_params[2].dynamic_switch_mipi = 1;
	dfps_params[2].PLL_CLOCK_dyn = 550;
	dfps_params[2].horizontal_frontporch_dyn = 288;
	dfps_params[2].vertical_frontporch_dyn = 2500;
	dfps_params[2].vertical_frontporch_for_low_power_dyn = 2500;

	dsi->dfps_num = 3;
}
#endif

static void lcm_get_params(LCM_PARAMS *params)
{
	int boot_mode = 0;
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
	params->density = LCM_DENSITY;

	params->dsi.IsCphy = 1;
#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
	/*lcm_dsi_mode = CMD_MODE;*/
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;/*SYNC_PULSE_VDO_MODE;*/
	params->dsi.switch_mode = CMD_MODE;
	/*lcm_dsi_mode = SYNC_PULSE_VDO_MODE;*/
#endif
	/*LCM_LOGI("lcm_get_params lcm_dsi_mode %d\n", lcm_dsi_mode);*/
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_THREE_LANE;/*LCM_THREE_LANE;*/
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 10;
	params->dsi.vertical_backporch = 10;
	params->dsi.vertical_frontporch = 1291;
	params->dsi.vertical_frontporch_for_low_power = 2500;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 20;
	params->dsi.horizontal_backporch = 22;
	params->dsi.horizontal_frontporch = 256;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 1;
#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	/* this value must be in MTK suggested table */
	params->dsi.PLL_CLOCK = 538;
#else
	/* this value must be in MTK suggested table */
	params->dsi.PLL_CLOCK = 538;
#endif
	/*params->dsi.PLL_CK_CMD = 220;*/
	/*params->dsi.PLL_CK_VDO = 255;*/
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif
	/* clk continuous video mode */
	params->dsi.cont_clock = 0;

	params->dsi.clk_lp_per_line_enable = 0;
	if (get_boot_mode() == META_BOOT) {
		boot_mode++;
		LCM_LOGI("META_BOOT\n");
	}
	if (get_boot_mode() == ADVMETA_BOOT) {
		boot_mode++;
		LCM_LOGI("ADVMETA_BOOT\n");
	}
	if (get_boot_mode() == ATE_FACTORY_BOOT) {
		boot_mode++;
		LCM_LOGI("ATE_FACTORY_BOOT\n");
	}
	if (get_boot_mode() == FACTORY_BOOT)     {
		boot_mode++;
		LCM_LOGI("FACTORY_BOOT\n");
	}
	if (boot_mode == 0) {
		LCM_LOGI("neither META_BOOT or FACTORY_BOOT\n");
		params->dsi.esd_check_enable = 0;
		params->dsi.customization_esd_check_enable = 0;
		params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
		params->dsi.lcm_esd_check_table[0].count = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
	}
	if((!is_sala_a()) && (!is_sala_three_camera())) {
		params->blmap = blmap_table_v2;
		params->blmap_size = sizeof(blmap_table_v2) / sizeof(blmap_table_v2[0]);
	} else {
		params->blmap = blmap_table_v1;
		params->blmap_size = sizeof(blmap_table_v1) / sizeof(blmap_table_v1[0]);
	}
	params->brightness_max = 4095;
	params->brightness_min = 10;
	params->dsi.HS_ZERO = 48;

	/*if need mipi hopping params add here*/
	params->dsi.dynamic_switch_mipi = 1;
	params->dsi.PLL_CLOCK_dyn = 550;
	params->dsi.horizontal_frontporch_dyn = 288;

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	/****DynFPS start****/
	lcm_dfps_int(&(params->dsi));
	/****DynFPS end****/
#endif

	register_device_proc("lcd", "nt36672c_tianma", "tianma");
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->corner_pattern_height = ROUND_CORNER_H_TOP;
	params->corner_pattern_height_bot = ROUND_CORNER_H_BOT;
	params->corner_pattern_tp_size = sizeof(top_rc_pattern);
	params->corner_pattern_lt_addr = (void *)top_rc_pattern;
#endif
}

static void lcm_init_power(void)
{
	LCM_LOGI("%s: enter\n", __func__);
	if ((!gpio_get_value(GPIO_LCD_VSP_EN)) && (!gpio_get_value(GPIO_LCD_VSN_EN))) { /*when vsp and vsn is not enable*/
		LCM_LOGI("%s: set lcd bias on\n", __func__);
		MDELAY(1);
		SET_LCM_VSP_PIN(1);
		MDELAY(3);
		SET_LCM_VSN_PIN(1);
		MDELAY(10);
	}
	LCM_LOGI("%s: exit\n", __func__);
}
/*#ifdef OPLUS_FEATURE_TP_BASIC*/
extern void lcd_queue_load_tp_fw(void);
extern int tp_gesture_enable_flag(void);
static void lcm_suspend_power(void)
{
	LCM_LOGI("%s: enter\n", __func__);
	pr_debug("%s: tp_gesture_enable_flag = %d \n", __func__, tp_gesture_enable_flag());
	if (0 == tp_gesture_enable_flag()) {
		/*#endif OPLUS_FEATURE_TP_BASIC*/
		LCM_LOGI("%s: set lcd bias off\n", __func__);
		MDELAY(8);
		SET_LCM_VSN_PIN(0);
		MDELAY(1);
		SET_LCM_VSP_PIN(0);
		MDELAY(10);
	}
	LCM_LOGI("%s: exit\n", __func__);
}

static void lcm_resume_power(void)
{
	LCM_LOGI("%s: enter\n", __func__);
	lcm_init_power();
	LCM_LOGI("%s: exit\n", __func__);
}

static void lcm_init(void)
{
	int size;
	LCM_LOGI("%s: enter\n", __func__);

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(3);
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);
	lcd_queue_load_tp_fw();
	size = sizeof(init_setting_vdo) /
		sizeof(struct LCM_setting_table);
	push_table(NULL, init_setting_vdo, size, 1);

	LCM_LOGI("%s: exit\n", __func__);
}

static void lcm_suspend(void)
{
	LCM_LOGI("%s: enter\n", __func__);
	push_table(NULL, lcm_suspend_setting,
			sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),
			1);
	MDELAY(10);

	LCM_LOGI("%s: exit\n", __func__);
}

static void lcm_resume(void)
{
	LCM_LOGI("%s: enter\n", __func__);
	lcm_init();
	LCM_LOGI("%s: exit\n", __func__);
}

#ifdef BUILD_LK
static unsigned int lcm_compare_id(void)
{
}
#endif

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	if ((!is_sala_a()) && (!is_sala_three_camera())) {
		LCM_LOGI("%s,nt36672c sala backlight: level = %d\n", __func__, level);
		if (level == 0) {
			bl_level_dimming_exit[1].para_list[0] = (level >> 8) & 0x0F;
			bl_level_dimming_exit[1].para_list[1] = level & 0xFF;
			push_table(handle,
					bl_level_dimming_exit,
					sizeof(bl_level_dimming_exit) / sizeof(struct LCM_setting_table),
					1);
		} else {
			if (level > 4095) {
				level = 4095;
			} else if (level > 0 && level < 142) {
				level = 142;
			}
			bl_level[0].para_list[0] = (level >> 8) & 0x0F;
			bl_level[0].para_list[1] = level & 0xFF;
			push_table(handle,
					bl_level,
					sizeof(bl_level) / sizeof(struct LCM_setting_table),
					1);
		}
	} else {
		LCM_LOGI("%s,nt sala_A/3 backlight: level = %d\n", __func__, level);
		if (level == 0) {
			bl_level_dimming_exit[1].para_list[0] = (level >> 8) & 0x0F;
			bl_level_dimming_exit[1].para_list[1] = level & 0xFF;
			push_table(handle,
					bl_level_dimming_exit,
					sizeof(bl_level_dimming_exit) / sizeof(struct LCM_setting_table),
					1);
		} else {
			if (level > 4095) {
				level = 4095;
			} else if (level > 0 && level < 24) {
				level = 24;
			}
			bl_level[0].para_list[0] = (level >> 8) & 0x0F;
			bl_level[0].para_list[1] = level & 0xFF;
			push_table(handle,
					bl_level,
					sizeof(bl_level) / sizeof(struct LCM_setting_table),
					1);
		}
	}
}

static void lcm_set_cabc_mode_cmdq(void *handle, unsigned int level)
{
	LCM_LOGI("%s [lcd] cabc_mode is %d \n", __func__, level);

	if (level == 1) {
		push_table(handle, lcm_cabc_enter_setting_ui, sizeof(lcm_cabc_enter_setting_ui) / sizeof(struct LCM_setting_table), 1);
	} else if (level == 2) {
		push_table(handle, lcm_cabc_enter_setting_still, sizeof(lcm_cabc_enter_setting_still) / sizeof(struct LCM_setting_table), 1);
	} else if (level == 3) {
		push_table(handle, lcm_cabc_enter_setting_moving, sizeof(lcm_cabc_enter_setting_moving) / sizeof(struct LCM_setting_table), 1);
	} else {
		push_table(handle, lcm_cabc_exit_setting, sizeof(lcm_cabc_exit_setting) / sizeof(struct LCM_setting_table), 1);
	}
}
LCM_DRIVER nt36672c_fhdp_dsi_vdo_tianma_zal1852_lcm_drv = {
	.name = "nt36672c_tianma_sala",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
#ifdef BUILD_LK
	.compare_id = lcm_compare_id,
#endif
	.init_power = lcm_init_power,
	.suspend_power = lcm_suspend_power,
	.resume_power = lcm_resume_power,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.set_cabc_mode_cmdq = lcm_set_cabc_mode_cmdq,
};
