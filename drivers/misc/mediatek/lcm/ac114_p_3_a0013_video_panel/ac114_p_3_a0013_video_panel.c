/***********************************************************
** Copyright (C), 2008-2016, OPLUS Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** File: - nt36525b_hlt_even_boe_hdp_dsi_vdo_lcm.c
** Description: source file for lcm nt36525b+hlt in kernel stage
**
** Version: 1.0
** Date : 2019/9/25
**
** ------------------------------- Revision History: -------------------------------
**<author><data><version ><desc>
**  lianghao       2019/9/25     1.0     source file for lcm nt36525b+hlt in kernel stage
**
****************************************************************/
#define pr_fmt(fmt) "dsi_cmd: %s: " fmt, __func__

#define LOG_TAG "LCM"
#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"
#include "../mt65xx_lcm_list.h"
#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include "disp_dts_gpio.h"
#endif
/*#include <linux/update_tpfw_notifier.h>*/
#include "disp_cust.h"
/*#include <soc/oplus/device_info.h>*/
static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define SET_LCM_VDD18_PIN(v)    (lcm_util.set_gpio_lcm_vddio_ctl((v)))
#define SET_LCM_VSP_PIN(v)  (lcm_util.set_gpio_lcd_enp_bias((v)))
#define SET_LCM_VSN_PIN(v)  (lcm_util.set_gpio_lcd_enn_bias((v)))
#define MDELAY(n)       (lcm_util.mdelay(n))
#define UDELAY(n)   (lcm_util.udelay(n))

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
#endif

#define LCM_DSI_CMD_MODE        0
#define FRAME_WIDTH             (720)
#define FRAME_HEIGHT            (1612)
#define LCM_PHYSICAL_WIDTH      (67932)
#define LCM_PHYSICAL_HEIGHT     (152092)
#define REGFLAG_DELAY   0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW   0xFFFE
#define REGFLAG_RESET_HIGH  0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif
/*extern unsigned int esd_recovery_backlight_level;*/
extern int gesture_flag;
extern int tp_gesture_enable_flag(void);
extern int tp_control_reset_gpio(bool enable);
extern void lcd_queue_load_tp_fw(void);
extern unsigned int oplus_display_brightness;
/*extern void ili_resume_by_ddi(void);*/
/*extern void core_config_sleep_ctrl(bool out);*/

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[200];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} }
};
static int last_powerflag = 0;
static int esd_flag = 0;
#if 1
static int blmap_table[] = {
                36, 8,
                16, 11,
                17, 12,
                19, 13,
                19, 15,
                20, 14,
                22, 14,
                22, 14,
                24, 10,
                24, 8 ,
                26, 4 ,
                27, 0 ,
                29, 9 ,
                29, 9 ,
                30, 14,
                33, 25,
                34, 30,
                36, 44,
                37, 49,
                40, 65,
                40, 69,
                43, 88,
                46, 109,
                47, 112,
                50, 135,
                53, 161,
                53, 163,
                60, 220,
                60, 223,
                64, 257,
                63, 255,
                71, 334,
                71, 331,
                75, 375,
                80, 422,
                84, 473,
                89, 529,
                88, 518,
                99, 653,
                98, 640,
                103, 707,
                117, 878,
                115, 862,
                122, 947,
                128, 1039,
                135, 1138,
                132, 1102,
                149, 1355,
                157, 1478,
                166, 1611,
                163, 1563,
                183, 1900,
                180, 1844,
                203, 2232,
                199, 2169,
                209, 2344,
                236, 2821,
                232, 2742,
                243, 2958,
                255, 3188,
                268, 3433,
                282, 3705,
                317, 4400,
                176, 1555};
#endif

/*
static struct LCM_setting_table init_setting_cmd[] = {
	{ 0xFF, 0x03, {0x98, 0x07, 0x00} },
};
*/

static struct LCM_setting_table bl_level[] = {
	/* { 0xFF, 0x03, {0x98, 0x81, 0x00} }, */
	{0x51, 2, {0x00, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table bl_level_1[] = {
	/* { 0xFF, 0x03, {0x98, 0x81, 0x00} }, */
	{REGFLAG_DELAY, 25, {}},
	{0x51, 2, {0x00, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table init_setting_vdo[] = {
		{0xFF, 03, {0x98, 0x83, 0x06}},
		{0x06, 01, {0xA4}},
		{0x3E, 01, {0xE2}},
		{0xFF, 03, {0x98, 0x83, 0x03}},
		{0x83, 01, {0x20}},
		{0x84, 01, {0x00}},
		{0xFF, 03, {0x98, 0x83, 0x03}},
		{0x86, 01, {0x6C}},
		{0x88, 01, {0xE1}},
		{0x89, 01, {0xe8}},
		{0x8A, 01, {0xF0}},
		{0x8B, 01, {0xF7}},
		{0x8C, 01, {0xBF}},
		{0x8D, 01, {0xC5}},
		{0x8E, 01, {0xC8}},
		{0x8F, 01, {0xCE}},
		{0x90, 01, {0xD1}},
		{0x91, 01, {0xD6}},
		{0x92, 01, {0xDC}},
		{0x93, 01, {0xE3}},
		{0x94, 01, {0xED}},
		{0x95, 01, {0xFA}},
		{0xAF, 01, {0x18}},

		{0xFF, 03, {0x98, 0x83, 0x07}},
		{0x00, 01, {0x00}},
		{0x01, 01, {0x00}},

		{0xFF, 03, {0x98, 0x83, 0x00}},
		{0x53, 01, {0x24}},
		{0x35, 01, {0x00}},
		{0x11, 01, {0x00}},
		{REGFLAG_DELAY, 80, {}},
		{0x29, 01, {0x00}},
		{REGFLAG_DELAY, 20, {}},
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
    unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;
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
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void push_table_cust(void *cmdq, struct LCM_setting_table_V3*table,
	unsigned int count, bool hs)
{
	set_lcm_nolock(table, count, hs);
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
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
	dfps_params[0].vertical_frontporch = 1360;
	dfps_params[0].vertical_frontporch_for_low_power = 0;

	/*DPFS_LEVEL1*/
	dfps_params[1].level = DFPS_LEVEL1;
	dfps_params[1].fps = 9000;/*real fps * 100, to support float*/
	dfps_params[1].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/*if mipi clock solution*/
	/*dfps_params[1].PLL_CLOCK = xx;*/
	/*dfps_params[1].data_rate = xx; */
	/*if HFP solution*/
	/*dfps_params[1].horizontal_frontporch = xx;*/
	dfps_params[1].vertical_frontporch = 370;
	dfps_params[1].vertical_frontporch_for_low_power = 0;

	dsi->dfps_num = 2;
}
#endif

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));
	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
	lcm_dsi_mode = CMD_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
	lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
#endif
	pr_debug("lcm_get_params lcm_dsi_mode %d\n", lcm_dsi_mode);
	params->dsi.switch_mode_enable = 0;
	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 20;
	params->dsi.vertical_frontporch = 370;
	/*params->dsi.vertical_frontporch_for_low_power = 540;*/
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.horizontal_sync_active = 8;
	params->dsi.horizontal_backporch = 10;
	params->dsi.horizontal_frontporch = 20;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 1;
	params->dsi.dynamic_switch_mipi = 1;
	params->dsi.horizontal_backporch_dyn = 20;
	params->dsi.data_rate_dyn = 912;
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 360;	/* this value must be in MTK suggested table */
#else
	params->dsi.data_rate = 899;	/* this value must be in MTK suggested table */
#endif

	params->blmap = blmap_table;
	params->blmap_size = sizeof(blmap_table)/sizeof(blmap_table[0]);
	params->brightness_max = 4096;
	params->brightness_min = 13;

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        params->round_corner_en = 1;
        params->full_content = 1;
        params->corner_pattern_width = 720;
        params->corner_pattern_height = 106;
        params->corner_pattern_height_bot = 106;
#endif

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
        /****DynFPS start****/
        lcm_dfps_int(&(params->dsi));
        /****DynFPS end****/
#endif

	register_device_proc("lcd", "A0013", "P_3");
}


static void lcm_init_power(void)
{
	pr_debug("lcm_init_power\n");
	SET_LCM_VSP_PIN(1);
	MDELAY(3);
	SET_LCM_VSN_PIN(1);
	MDELAY(15);
}

static void lcm_set_esd_flag(int num)
{
	pr_debug("lcm_set_esd_flag num = %d\n" , num);
	esd_flag = num;
}

static void lcm_suspend_power(void)
{
	pr_debug("lcm_suspend_power\n");
	pr_debug("esd_flag num == %d, tp_gesture_enable_flag == %d\n", esd_flag, tp_gesture_enable_flag());
	if ((0 == tp_gesture_enable_flag()) || esd_flag == 1) {
		SET_LCM_VSN_PIN(0);
		MDELAY(2);
		SET_LCM_VSP_PIN(0);
		MDELAY(70);
	}
}

static void lcm_shudown_power(void)
{
	pr_debug("a0013 lcm_shudown_power\n");
	tp_control_reset_gpio(false);
	MDELAY(12);
	SET_RESET_PIN(0);
	MDELAY(2);
	SET_LCM_VSN_PIN(0);
	MDELAY(2);
	SET_LCM_VSP_PIN(0);
	MDELAY(2);
}

static void lcm_resume_power(void)
{
	pr_debug("lcm_resume_power\n");
	SET_LCM_VSP_PIN(1);
	MDELAY(3);
	SET_LCM_VSN_PIN(1);
	/*base voltage = 4.0 each step = 100mV; 4.0+20 * 0.1 = 6.0v;*/
	/* #if defined(MTK_LCM_DEVICE_TREE_SUPPORT_PASCAL_E) */
	if (display_bias_setting(0x14)) {
		pr_err("fatal error: lcd gate ic setting failed \n");
	}
	/* #endif */
	MDELAY(10);
}

static void lcm_init(void)
{
	pr_debug("lcm_init\n");
	SET_RESET_PIN(0);
	MDELAY(3);
	SET_RESET_PIN(1);
	MDELAY(15);
	lcd_queue_load_tp_fw();
	MDELAY(10);
	push_table(NULL, init_setting_vdo, sizeof(init_setting_vdo) / sizeof(struct LCM_setting_table), 1);
	pr_debug("p_3a0013_lcm_mode = vdo mode :%d\n", lcm_dsi_mode);
}

static void lcm_suspend(void)
{
	pr_debug("lcm_suspend\n");
	MDELAY(5);
	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	last_powerflag = 0;
}

static void lcm_resume(void)
{
	pr_debug("lcm_resume\n");
	last_powerflag = 1;
	lcm_init();
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	if (level > 4095)
		level = 4095;

	if (1 == last_powerflag) {
			bl_level_1[1].para_list[0] = 0x000F&(level >> 8);
			bl_level_1[1].para_list[1] = 0x00FF&(level);
			pr_debug("[ HW  backlight a0013]level_1 = %d para_list[0] = %x, \
			para_list[1]=%x\n", level, bl_level_1[1].para_list[0], bl_level_1[1].para_list[1]);
			push_table(handle, bl_level_1, sizeof(bl_level_1) / sizeof(struct LCM_setting_table), 1);
			last_powerflag = 0;
	} else {
			bl_level[0].para_list[0] = 0x000F&(level >> 8);
			bl_level[0].para_list[1] = 0x00FF&(level);
			pr_debug("[ HW  backlight a0013]level = %d para_list[0] = %x, \
			para_list[1]=%x\n", level, bl_level[0].para_list[0], bl_level[0].para_list[1]);
			push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	}
	oplus_display_brightness = level;
}

static struct LCM_setting_table set_cabc_off[] = {
	{0xFF, 3, {0x98, 0x83, 0x00}},
	{0x55, 1, {0x00}},
};
static struct LCM_setting_table set_cabc_1[] = {
	{0xFF, 3, {0x98, 0x83, 0x00}},
	{0x55, 1, {0x01}},
};
static struct LCM_setting_table set_cabc_2[] = {
	{0xFF, 3, {0x98, 0x83, 0x00}},
	{0x55, 1, {0x02}},
};

static void lcm_set_cabc_cmdq(void *handle, unsigned int level) {
	pr_debug("[lcm] cabc set level %d\n", level);
	if (level == 3) {
		level = 2;
		pr_debug("[lcm] cabc set level_2 %d\n", level);
	}
	if (level == 0) {
		push_table(handle, set_cabc_off, sizeof(set_cabc_off) / sizeof(struct LCM_setting_table), 1);
	} else if (level == 1) {
		push_table(handle, set_cabc_1, sizeof(set_cabc_1) / sizeof(struct LCM_setting_table), 1);
	} else if (level == 2) {
		push_table(handle, set_cabc_2, sizeof(set_cabc_2) / sizeof(struct LCM_setting_table), 1);
	} else {
		pr_debug("[lcm]  level %d is not support\n", level);
	}
}

static unsigned int lcm_esd_recover(void)
{
	unsigned int level = oplus_display_brightness;
#ifndef BUILD_LK
	lcm_resume_power();
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);

	push_table(NULL, init_setting_vdo, sizeof(init_setting_vdo) / sizeof(struct LCM_setting_table), 1);
	pr_debug("a0013_p_3_lcm_mode = vdo mode esd recovery :%d----\n", lcm_dsi_mode);
	pr_debug("lcm_esd_recovery\n");
	bl_level_1[1].para_list[0] = 0x000F&(level >> 8);
	bl_level_1[1].para_list[1] = 0x00FF&(level);
	pr_debug("[ lcm_esd_recovery backlight a0013]level_1 = %d para_list[0] = %x, \
		para_list[1]=%x\n", level, bl_level_1[1].para_list[0], bl_level_1[1].para_list[1]);
	push_table(NULL, bl_level_1, sizeof(bl_level_1) / sizeof(struct LCM_setting_table), 1);
	return FALSE;
#else
	return FALSE;
#endif
}

static struct LCM_setting_table_V3 set_gamma_enter[] = {
		{0x29, 0xFF, 0x03, {0x98, 0x83, 0x08}},
		{0x29, 0xE0, 0x1D, {0x55, 0x95, 0x97, 0x9D, 0xA5, 0x55, 0xB3, 0xC2, 0xCF, 0xE2, 0xA9, \
		0xF5, 0x18, 0x39, 0x5A, 0xAA, 0x7D, 0xA5, 0xD6, 0xF5, 0xFF, 0x1A, 0x39, 0x61, 0x8F, \
		0x3F, 0xB3, 0xC6, 0xDA}},
		{0x29, 0xE1, 0x1D, {0x55, 0x95, 0x97, 0x9D, 0xA5, 0x55, 0xB3, 0xC2, 0xCF, 0xE2, 0xA9, \
		0xF5, 0x18, 0x39, 0x5A, 0xAA, 0x7D, 0xA5, 0xD6, 0xF5, 0xFF, 0x1A, 0x39, 0x61, 0x8F, \
		0x3F, 0xB3, 0xC6, 0xDA}},
		{0x29, 0xFF, 0x03, {0x98, 0x83, 0x00}},
};

static struct LCM_setting_table_V3 set_gamma_exit[] = {
		{0x29, 0xFF, 0x03, {0x98, 0x83, 0x08}},
		{0x29, 0xE0, 0x1D, {0x00, 0x00, 0x16, 0x4B, 0x80, 0x50, 0xC2, 0xFA, 0x25, 0x58, 0x95, \
		0x81, 0xC3, 0xF6, 0x25, 0xAA, 0x50, 0x7C, 0xB8, 0xDA, 0xFF, 0x01, 0x23, 0x4A, 0x7C, \
		0x3F, 0xA6, 0xC6, 0xDA}},
		{0x29, 0xE1, 0x1D, {0x00, 0x00, 0x16, 0x4B, 0x80, 0x50, 0xC2, 0xFA, 0x25, 0x58, 0x95, \
		0x81, 0xC3, 0xF6, 0x25, 0xAA, 0x50, 0x7C, 0xB8, 0xDA, 0xFF, 0x01, 0x23, 0x4A, 0x7C, \
		0x3F, 0xA6, 0xC6, 0xDA}},
		{0x29, 0xFF, 0x03, {0x98, 0x83, 0x00}},
};

static void lcm_set_gamma_mode_cmdq(void *handle, unsigned int level)
{
	pr_debug("%s [lcd] gamma_mode is %d \n", __func__, level);

	if (1 == level) {
		push_table_cust(handle, set_gamma_enter, sizeof(set_gamma_enter) / sizeof(struct LCM_setting_table_V3), 0);
	} else {
		push_table_cust(handle, set_gamma_exit, sizeof(set_gamma_exit) / sizeof(struct LCM_setting_table_V3), 0);
	}
}

struct LCM_DRIVER ac114_p_3_a0013_video_panel_lcm_drv = {
	.name = "ac114_p_3_a0013_video_panel",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.shutdown_power = lcm_shudown_power,
	.esd_recover = lcm_esd_recover,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.set_cabc_mode_cmdq = lcm_set_cabc_cmdq,
	.set_gamma_mode_cmdq = lcm_set_gamma_mode_cmdq,
	.set_esd_flag = lcm_set_esd_flag,
};

