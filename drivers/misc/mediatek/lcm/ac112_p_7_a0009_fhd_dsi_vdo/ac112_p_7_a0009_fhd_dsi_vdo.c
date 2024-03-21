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
#define FRAME_WIDTH             (1080)
#define FRAME_HEIGHT            (2400)
#define LCM_PHYSICAL_WIDTH      (67930)
#define LCM_PHYSICAL_HEIGHT     (150960)
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
extern void tp_gpio_current_leakage_handler(bool normal);
extern int shut_down_flag;
/*extern void ili_resume_by_ddi(void);*/
/*extern void core_config_sleep_ctrl(bool out);*/

struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[200];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} }
};
unsigned int g_bl_flag = 0;
static unsigned int bl_old_level = 0;
static int cabc_status = 0;

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
static struct LCM_setting_table init_setting_vdo[] = {
	{0xFF, 1, {0x25}},
	{0xFB, 1, {0x01}},
	{0x18, 1, {0x21}},
	{0x21, 1, {0xc0}},
	{0xFF, 1, {0xE0}},
	{0xFB, 1, {0x01}},
	{0x35, 1, {0x82}},

	{0xFF, 1, {0xF0}},
	{0xFB, 1, {0x01}},
	{0x1C, 1, {0x01}},
	{0x33, 1, {0x01}},
	{0x5A, 1, {0x00}},
	{0x9C, 1, {0x17}},
	{0x9F, 1, {0x19}},

	{0xFF, 1, {0xD0}},
	{0xFB, 1, {0x01}},
	{0x53, 1, {0x22}},
	{0x54, 1, {0x02}},

	{0xFF, 1, {0xC0}},
	{0xFB, 1, {0x01}},
	{0x9C, 1, {0x11}},
	{0x9D, 1, {0x11}},
/* CABC*/
	{0xFF, 1, {0x23}},
	{0xFB, 1, {0x01}},
	{0x00, 1, {0x80}},
	{0x05, 1, {0x22}},
	{0x06, 1, {0x01}},
	{0x07, 1, {0x00}},
	{0x08, 1, {0x01}},
	{0x09, 1, {0x00}},
	{0x10, 1, {0x82}},
	{0x11, 1, {0x01}},
	{0x12, 1, {0x95}},
	{0x15, 1, {0x68}},
	{0x16, 1, {0x0B}},
/* CABC_PWM_UI*/
	{0x30, 1, {0xFF}},
	{0x31, 1, {0xFD}},
	{0x32, 1, {0xFA}},
	{0x33, 1, {0xF7}},
	{0x34, 1, {0xF4}},
	{0x35, 1, {0xF0}},
	{0x36, 1, {0xED}},
	{0x37, 1, {0xEC}},
	{0x38, 1, {0xEB}},
	{0x39, 1, {0xEA}},
	{0x3A, 1, {0xE9}},
	{0x3B, 1, {0xE8}},
	{0x3D, 1, {0xE7}},
	{0x3F, 1, {0xE6}},
	{0x40, 1, {0xE5}},
	{0x41, 1, {0xE4}},
/* CABC_PWM_STILL*/
	{0x45, 1, {0xFF}},
	{0x46, 1, {0xFA}},
	{0x47, 1, {0xF2}},
	{0x48, 1, {0xE8}},
	{0x49, 1, {0xE4}},
	{0x4A, 1, {0xDC}},
	{0x4B, 1, {0xD7}},
	{0x4C, 1, {0xD5}},
	{0x4D, 1, {0xD3}},
	{0x4E, 1, {0xD2}},
	{0x4F, 1, {0xD0}},
	{0x50, 1, {0xCE}},
	{0x51, 1, {0xCD}},
	{0x52, 1, {0xCB}},
	{0x53, 1, {0xC6}},
	{0x54, 1, {0xC3}},

	{0xFF, 1, {0x10}},
	{0xFB, 1, {0x01}},
	{0x3B, 5, {0x03, 0x14, 0x36, 0x04, 0x04}},
	{0xB0, 1, {0x00}},
	{0xC0, 1, {0x00}},
	{0x51, 2, {0x00, 0x00}},
	{0x53, 1, {0x24}},
	{0x35, 1, {0x00}},
	{0x11, 1, {0x00}},
	{REGFLAG_UDELAY, 120000, {} },
	{0x29, 1, {0x00}},
};

static struct LCM_setting_table set_cabc_ui[] = {
	{0xFF, 0x01, {0x10}},
	{0xFB, 0x01, {0x01}},
	{0x53, 0x01, {0x2C}},
	{0x55, 0x01, {0x01}}
};
static struct LCM_setting_table set_cabc_still[] = {
	{0xFF, 0x01, {0x10}},
	{0xFB, 0x01, {0x01}},
	{0x53, 0x01, {0x2C}},
	{0x55, 0x01, {0x02}}
};
static struct LCM_setting_table set_cabc_off[] = {
	{0xFF, 0x01, {0x10}},
	{0xFB, 0x01, {0x01}},
	{0x53, 0x01, {0x2C}},
	{0x55, 0x01, {0x00}}
};

static struct LCM_setting_table set_dimming_off[] = {
	{0xFF, 0x01, {0x10}},
	{0xFB, 0x01, {0x01}},
	{0x53, 0x01, {0x24}}
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
	params->dsi.vertical_sync_active = 10;
	params->dsi.vertical_backporch = 10;
	params->dsi.vertical_frontporch = 54;
	/*params->dsi.vertical_frontporch_for_low_power = 540;*/
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.horizontal_sync_active = 18;
	params->dsi.horizontal_backporch = 30;
	params->dsi.horizontal_frontporch = 30;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 1;
	/* params->dsi.HS_TRAIL = 6; */
	params->dsi.CLK_HS_PRPR = 8;

	params->dsi.dynamic_switch_mipi = 1;
	params->dsi.horizontal_sync_active_dyn = 4;
	params->dsi.horizontal_backporch_dyn = 12;
	params->dsi.data_rate_dyn = 1086;
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 360;	/* this value must be in MTK suggested table */
#else
	params->dsi.data_rate = 1107;	/* this value must be in MTK suggested table */
#endif

	params->blmap = blmap_table;
	params->blmap_size = sizeof(blmap_table)/sizeof(blmap_table[0]);
	params->brightness_max = 4096;
	params->brightness_min = 4;

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->full_content = 1;
	params->corner_pattern_width = 1080;
	params->corner_pattern_height = 150;
	params->corner_pattern_height_bot = 150;
#endif

	register_device_proc("lcd", "A0009", "P_7");
}


static void lcm_init_power(void)
{
	pr_info("lcm_init_power\n");
	MDELAY(1);
	SET_LCM_VDD18_PIN(0);  /* ON 0; OFF 1 */
	MDELAY(3);
	SET_LCM_VSP_PIN(1);
	MDELAY(3);
	SET_LCM_VSN_PIN(1);
	MDELAY(15);
}

static void lcm_suspend_power(void)
{
	pr_info("lcm_suspend_power, shut_down_flag:%d\n", shut_down_flag);
	if (!tp_gesture_enable_flag() && !shut_down_flag) {
		printk("lcm_tp_suspend_power_on\n");
		SET_LCM_VSN_PIN(0);
		MDELAY(2);
		SET_LCM_VSP_PIN(0);
		MDELAY(3);
	}
	MDELAY(26);
}

static void lcm_suspend_power_1p8(void)
{
	if (!tp_gesture_enable_flag()) {
		pr_info("lcm_suspend_power_1p8\n");
		SET_RESET_PIN(0);
		MDELAY(2);
		tp_gpio_current_leakage_handler(false);
		MDELAY(30);
		SET_LCM_VDD18_PIN(1);   /* ON 0; OFF 1 */
		MDELAY(1000);
	}
}

static void lcm_shutdown_power(void)
{
	pr_info("lcm_shutdown_power\n");
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
	SET_LCM_VDD18_PIN(0);   /* ON 0; OFF 1 */
	MDELAY(2);
	SET_RESET_PIN(1);
	MDELAY(14);
	pr_info("lcm_resume_power\n");
	SET_LCM_VSP_PIN(1);
	MDELAY(3);
	SET_LCM_VSN_PIN(1);
	/*base voltage = 4.0 each step = 100mV; 4.0+20 * 0.1 = 6.0v;*/
	/* #if defined(MTK_LCM_DEVICE_TREE_SUPPORT_PASCAL_E) */
	if (display_bias_setting(0xf)) {
		pr_err("fatal error: lcd gate ic setting failed \n");
	}
	/* #endif */
	MDELAY(7);
}

static void lcm_init_set_cabc(int cabc_mode) {
	pr_info("[lcm] init set cabc_mode %d\n", cabc_mode);
	if (cabc_mode == 0) {
		push_table(NULL, set_cabc_off, sizeof(set_cabc_off) / sizeof(struct LCM_setting_table), 1);
	} else if (cabc_mode == 1) {
		push_table(NULL, set_cabc_ui, sizeof(set_cabc_ui) / sizeof(struct LCM_setting_table), 1);
	} else if (cabc_mode == 2) {
		push_table(NULL, set_cabc_still, sizeof(set_cabc_still) / sizeof(struct LCM_setting_table), 1);
	} else {
		pr_info("[lcm]  cabc_mode %d is not support\n", cabc_mode);
	}
}

extern void lcd_queue_load_tp_fw(void);
static void lcm_init(void)
{
	pr_info("lcm_init\n");
	SET_RESET_PIN(1);
	MDELAY(3);
	SET_RESET_PIN(0);
	MDELAY(3);
	SET_RESET_PIN(1);
	MDELAY(15);
	tp_gpio_current_leakage_handler(true);
	MDELAY(5);
	lcd_queue_load_tp_fw();
	MDELAY(5);
	push_table(NULL, init_setting_vdo, sizeof(init_setting_vdo) / sizeof(struct LCM_setting_table), 1);
	lcm_init_set_cabc(cabc_status);
	pr_err("lcm_mode = vdo mode :%d\n", lcm_dsi_mode);
}

static void lcm_suspend(void)
{
	pr_info("lcm_suspend\n");
	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
	pr_info("lcm_resume\n");
	lcm_init();
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	if (level > 4095)
		level = 4095;

	if (0 == level) {
		bl_level[0].para_list[0] = 0;
		bl_level[0].para_list[1] = 0;
		pr_info("[ HW backlight ]level = %d para_list[0] = %x, \
		para_list[1]=%x\n", level, bl_level[0].para_list[0], bl_level[0].para_list[1]);
		push_table(handle, set_dimming_off, sizeof(set_dimming_off) / sizeof(struct LCM_setting_table), 1);
		push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
		pr_info("close backlight level==%u ", level);
	} else {
		bl_level[0].para_list[0] = 0x000F&(level >> 8);
		bl_level[0].para_list[1] = 0x00FF&(level);
		pr_info("[ HW  backlight ]level = %d para_list[0] = %x, \
		para_list[1]=%x\n", level, bl_level[0].para_list[0], bl_level[0].para_list[1]);
		push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	}
	bl_old_level = level;
}

static void lcm_set_cabc_cmdq(void *handle, unsigned int level) {
	pr_info("[lcm] cabc set level %d\n", level);

	if (level == 3) {
		level = 2;
		pr_info("[lcm] cabc set level_2 %d\n", level);
	}

	if (level == 0) {
		push_table(handle, set_cabc_off, sizeof(set_cabc_off) / sizeof(struct LCM_setting_table), 1);
	} else if (level == 1) {
		push_table(handle, set_cabc_ui, sizeof(set_cabc_ui) / sizeof(struct LCM_setting_table), 1);
	} else if (level == 2) {
		push_table(handle, set_cabc_still, sizeof(set_cabc_still) / sizeof(struct LCM_setting_table), 1);
	} else {
		pr_info("[lcm]  level %d is not support\n", level);
	}
	cabc_status = level;
}
static void lcm_get_cabc_status(int *status) {
	pr_debug("[lcm] cabc get to %d\n", cabc_status);
	*status = cabc_status;
}

static unsigned int lcm_esd_recover(void)
{
#ifndef BUILD_LK
	lcm_resume_power();
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);

	push_table(NULL, init_setting_vdo, sizeof(init_setting_vdo) / sizeof(struct LCM_setting_table), 1);
	pr_debug("lcm_mode = vdo mode esd recovery :%d----\n", lcm_dsi_mode);
	tp_gpio_current_leakage_handler(true);
	MDELAY(5);
	lcd_queue_load_tp_fw();
	MDELAY(10);
	pr_info("lcm_esd_recovery\n");
	push_table(NULL, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	lcm_init_set_cabc(cabc_status);
	/*push_table_cust(NULL, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table_V3), 0);*/
	return FALSE;
#else
	return FALSE;
#endif
}

static struct LCM_setting_table_V3 set_gamma_enter[] = {
		{0x23, 0xFF, 0x01, {0x20}},
		{0x23, 0xFB, 0x01, {0x01}},
		{0x23, 0x95, 0x01, {0x09}},
		{0x23, 0x96, 0x01, {0x09}},
		{0x23, 0xFF, 0x01, {0x20}},
		{0x23, 0xFB, 0x01, {0x01}},
		{0x29, 0xB0, 0x10, {0x00, 0x00, 0x00, 0x04, 0x00, 0x0D, 0x00, 0x15, 0x00, 0x1C, 0x00, \
		0x24, 0x00, 0x2B, 0x00, 0x33}},
		{0x29, 0xB1, 0x10, {0x00, 0x39, 0x00, 0x54, 0x00, 0x6B, 0x00, 0x98, 0x00, 0xBF, 0x01, \
		0x09, 0x01, 0x4E, 0x01, 0x51}},
		{0x29, 0xB2, 0x10, {0x01, 0x9B, 0x01, 0xFA, 0x02, 0x36, 0x02, 0x84, 0x02, 0xB9, 0x02, \
		0xF8, 0x03, 0x0D, 0x03, 0x22}},
		{0x29, 0xB3, 0x0E, {0x03, 0x39, 0x03, 0x52, 0x03, 0x6F, 0x03, 0x92, 0x03, 0xB6, 0x03, \
		0xBC, 0x00, 0x00}},
		{0x29, 0xB4, 0x10, {0x00, 0x00, 0x00, 0x04, 0x00, 0x0D, 0x00, 0x15, 0x00, 0x1C, 0x00, \
		0x24, 0x00, 0x2B, 0x00, 0x33}},
		{0x29, 0xB5, 0x10, {0x00, 0x39, 0x00, 0x54, 0x00, 0x6B, 0x00, 0x98, 0x00, 0xBF, 0x01, \
		0x09, 0x01, 0x4E, 0x01, 0x51}},
		{0x29, 0xB6, 0x10, {0x01, 0x9B, 0x01, 0xFA, 0x02, 0x36, 0x02, 0x84, 0x02, 0xB9, 0x02, \
		0xF8, 0x03, 0x0D, 0x03, 0x22}},
		{0x29, 0xB7, 0x0E, {0x03, 0x39, 0x03, 0x52, 0x03, 0x6F, 0x03, 0x92, 0x03, 0xB6, 0x03, \
		0xBC, 0x00, 0x00}},
		{0x29, 0xB8, 0x10, {0x00, 0x00, 0x00, 0x04, 0x00, 0x0D, 0x00, 0x15, 0x00, 0x1C, 0x00, \
		0x24, 0x00, 0x2B, 0x00, 0x33}},
		{0x29, 0xB9, 0x10, {0x00, 0x39, 0x00, 0x54, 0x00, 0x6B, 0x00, 0x98, 0x00, 0xBF, 0x01, \
		0x09, 0x01, 0x4E, 0x01, 0x51}},
		{0x29, 0xBA, 0x10, {0x01, 0x9B, 0x01, 0xFA, 0x02, 0x36, 0x02, 0x84, 0x02, 0xB9, 0x02, \
		0xF8, 0x03, 0x0D, 0x03, 0x22}},
		{0x29, 0xBB, 0x0E, {0x03, 0x39, 0x03, 0x52, 0x03, 0x6F, 0x03, 0x92, 0x03, 0xB6, 0x03, \
		0xBC, 0x00, 0x00}},
		{0x23, 0xFF, 0x01, {0x21}},
		{0x23, 0xFB, 0x01, {0x01}},
		{0x29, 0xB0, 0x10, {0x00, 0x00, 0x00, 0x04, 0x00, 0x0D, 0x00, 0x15, 0x00, 0x1C, 0x00, \
		0x24, 0x00, 0x2B, 0x00, 0x33}},
		{0x29, 0xB1, 0x10, {0x00, 0x39, 0x00, 0x54, 0x00, 0x6B, 0x00, 0x98, 0x00, 0xBF, 0x01, \
		0x09, 0x01, 0x4E, 0x01, 0x51}},
		{0x29, 0xB2, 0x10, {0x01, 0x9B, 0x01, 0xFA, 0x02, 0x36, 0x02, 0x84, 0x02, 0xB9, 0x02, \
		0xF8, 0x03, 0x0D, 0x03, 0x22}},
		{0x29, 0xB3, 0x0E, {0x03, 0x39, 0x03, 0x52, 0x03, 0x6F, 0x03, 0x92, 0x03, 0xB6, 0x03, \
		0xBC, 0x00, 0x00}},
		{0x29, 0xB4, 0x10, {0x00, 0x00, 0x00, 0x04, 0x00, 0x0D, 0x00, 0x15, 0x00, 0x1C, 0x00, \
		0x24, 0x00, 0x2B, 0x00, 0x33}},
		{0x29, 0xB5, 0x10, {0x00, 0x39, 0x00, 0x54, 0x00, 0x6B, 0x00, 0x98, 0x00, 0xBF, 0x01, \
		0x09, 0x01, 0x4E, 0x01, 0x51}},
		{0x29, 0xB6, 0x10, {0x01, 0x9B, 0x01, 0xFA, 0x02, 0x36, 0x02, 0x84, 0x02, 0xB9, 0x02, \
		0xF8, 0x03, 0x0D, 0x03, 0x22}},
		{0x29, 0xB7, 0x0E, {0x03, 0x39, 0x03, 0x52, 0x03, 0x6F, 0x03, 0x92, 0x03, 0xB6, 0x03, \
		0xBC, 0x00, 0x00}},
		{0x29, 0xB8, 0x10, {0x00, 0x00, 0x00, 0x04, 0x00, 0x0D, 0x00, 0x15, 0x00, 0x1C, 0x00, \
		0x24, 0x00, 0x2B, 0x00, 0x33}},
		{0x29, 0xB9, 0x10, {0x00, 0x39, 0x00, 0x54, 0x00, 0x6B, 0x00, 0x98, 0x00, 0xBF, 0x01, \
		0x09, 0x01, 0x4E, 0x01, 0x51}},
		{0x29, 0xBA, 0x10, {0x01, 0x9B, 0x01, 0xFA, 0x02, 0x36, 0x02, 0x84, 0x02, 0xB9, 0x02, \
		0xF8, 0x03, 0x0D, 0x03, 0x22}},
		{0x29, 0xBB, 0x0E, {0x03, 0x39, 0x03, 0x52, 0x03, 0x6F, 0x03, 0x92, 0x03, 0xB6, 0x03, \
		0xBC, 0x00, 0x00}},
		{0x23, 0xFF, 0x01, {0x10}},
		{0x23, 0xFB, 0x01, {0x01}},
};

static struct LCM_setting_table_V3 set_gamma_exit[] = {
		{0x23, 0xFF, 0x01, {0x20}},
		{0x23, 0xFB, 0x01, {0x01}},
		{0x23, 0x95, 0x01, {0xD1}},
		{0x23, 0x96, 0x01, {0xD1}},
		{0x23, 0xFF, 0x01, {0x20}},
		{0x23, 0xFB, 0x01, {0x01}},
		{0x29, 0xB0, 0x10, {0x00, 0x00, 0x00, 0x1C, 0x00, 0x47, 0x00, 0x6A, 0x00, 0x86, 0x00, \
		0x9E, 0x00, 0xB4, 0x00, 0xC7}},
		{0x29, 0xB1, 0x10, {0x00, 0xD8, 0x01, 0x12, 0x01, 0x3A, 0x01, 0x7E, 0x01, 0xAC, 0x01, \
		0xF8, 0x02, 0x30, 0x02, 0x32}},
		{0x29, 0xB2, 0x10, {0x02, 0x68, 0x02, 0xA5, 0x02, 0xCD, 0x02, 0xFE, 0x03, 0x21, 0x03, \
		0x48, 0x03, 0x57, 0x03, 0x65}},
		{0x29, 0xB3, 0x0E, {0x03, 0x76, 0x03, 0x89, 0x03, 0x9E, 0x03, 0xAF, 0x03, 0xD1, 0x03, \
		0xD8, 0x00, 0x00}},
		{0x29, 0xB4, 0x10, {0x00, 0x00, 0x00, 0x1C, 0x00, 0x47, 0x00, 0x6A, 0x00, 0x86, 0x00, \
		0x9E, 0x00, 0xB4, 0x00, 0xC7}},
		{0x29, 0xB5, 0x10, {0x00, 0xD8, 0x01, 0x12, 0x01, 0x3A, 0x01, 0x7E, 0x01, 0xAC, 0x01, \
		0xF8, 0x02, 0x30, 0x02, 0x32}},
		{0x29, 0xB6, 0x10, {0x02, 0x68, 0x02, 0xA5, 0x02, 0xCD, 0x02, 0xFE, 0x03, 0x21, 0x03, \
		0x48, 0x03, 0x57, 0x03, 0x65}},
		{0x29, 0xB7, 0x0E, {0x03, 0x76, 0x03, 0x89, 0x03, 0x9E, 0x03, 0xAF, 0x03, 0xD1, 0x03, \
		0xD8, 0x00, 0x00}},
		{0x29, 0xB8, 0x10, {0x00, 0x00, 0x00, 0x1C, 0x00, 0x47, 0x00, 0x6A, 0x00, 0x86, 0x00, \
		0x9E, 0x00, 0xB4, 0x00, 0xC7}},
		{0x29, 0xB9, 0x10, {0x00, 0xD8, 0x01, 0x12, 0x01, 0x3A, 0x01, 0x7E, 0x01, 0xAC, 0x01, \
		0xF8, 0x02, 0x30, 0x02, 0x32}},
		{0x29, 0xBA, 0x10, {0x02, 0x68, 0x02, 0xA5, 0x02, 0xCD, 0x02, 0xFE, 0x03, 0x21, 0x03, \
		0x48, 0x03, 0x57, 0x03, 0x65}},
		{0x29, 0xBB, 0x0E, {0x03, 0x76, 0x03, 0x89, 0x03, 0x9E, 0x03, 0xAF, 0x03, 0xD1, 0x03, \
		0xD8, 0x00, 0x00}},
		{0x23, 0xFF, 0x01, {0x21}},
		{0x23, 0xFB, 0x01, {0x01}},
		{0x29, 0xB0, 0x10, {0x00, 0x00, 0x00, 0x1C, 0x00, 0x47, 0x00, 0x6A, 0x00, 0x86, 0x00, \
		0x9E, 0x00, 0xB4, 0x00, 0xC7}},
		{0x29, 0xB1, 0x10, {0x00, 0xD8, 0x01, 0x12, 0x01, 0x3A, 0x01, 0x7E, 0x01, 0xAC, 0x01, \
		0xF8, 0x02, 0x30, 0x02, 0x32}},
		{0x29, 0xB2, 0x10, {0x02, 0x68, 0x02, 0xA5, 0x02, 0xCD, 0x02, 0xFE, 0x03, 0x21, 0x03, \
		0x48, 0x03, 0x57, 0x03, 0x65}},
		{0x29, 0xB3, 0x0E, {0x03, 0x76, 0x03, 0x89, 0x03, 0x9E, 0x03, 0xAF, 0x03, 0xD1, 0x03, \
		0xD8, 0x00, 0x00}},
		{0x29, 0xB4, 0x10, {0x00, 0x00, 0x00, 0x1C, 0x00, 0x47, 0x00, 0x6A, 0x00, 0x86, 0x00, \
		0x9E, 0x00, 0xB4, 0x00, 0xC7}},
		{0x29, 0xB5, 0x10, {0x00, 0xD8, 0x01, 0x12, 0x01, 0x3A, 0x01, 0x7E, 0x01, 0xAC, 0x01, \
		0xF8, 0x02, 0x30, 0x02, 0x32}},
		{0x29, 0xB6, 0x10, {0x02, 0x68, 0x02, 0xA5, 0x02, 0xCD, 0x02, 0xFE, 0x03, 0x21, 0x03, \
		0x48, 0x03, 0x57, 0x03, 0x65}},
		{0x29, 0xB7, 0x0E, {0x03, 0x76, 0x03, 0x89, 0x03, 0x9E, 0x03, 0xAF, 0x03, 0xD1, 0x03, \
		0xD8, 0x00, 0x00}},
		{0x29, 0xB8, 0x10, {0x00, 0x00, 0x00, 0x1C, 0x00, 0x47, 0x00, 0x6A, 0x00, 0x86, 0x00, \
		0x9E, 0x00, 0xB4, 0x00, 0xC7}},
		{0x29, 0xB9, 0x10, {0x00, 0xD8, 0x01, 0x12, 0x01, 0x3A, 0x01, 0x7E, 0x01, 0xAC, 0x01, \
		0xF8, 0x02, 0x30, 0x02, 0x32}},
		{0x29, 0xBA, 0x10, {0x02, 0x68, 0x02, 0xA5, 0x02, 0xCD, 0x02, 0xFE, 0x03, 0x21, 0x03, \
		0x48, 0x03, 0x57, 0x03, 0x65}},
		{0x29, 0xBB, 0x0E, {0x03, 0x76, 0x03, 0x89, 0x03, 0x9E, 0x03, 0xAF, 0x03, 0xD1, 0x03, \
		0xD8, 0x00, 0x00}},
		{0x23, 0xFF, 0x01, {0x10}},
		{0x23, 0xFB, 0x01, {0x01}},
};

static void lcm_set_gamma_mode_cmdq(void *handle, unsigned int level)
{
	pr_err("%s [lcd] gamma_mode is %d \n", __func__, level);

	if (1 == level) {
		push_table_cust(handle, set_gamma_enter, sizeof(set_gamma_enter) / sizeof(struct LCM_setting_table_V3), 0);
	} else {
		push_table_cust(handle, set_gamma_exit, sizeof(set_gamma_exit) / sizeof(struct LCM_setting_table_V3), 0);
	}
}

struct LCM_DRIVER ac112_p_7_a0009_fhd_dsi_vdo_lcm_drv = {
	.name = "ac112_p_7_a0009_fhd_dsi_vdo_lcm_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.shutdown_power = lcm_shutdown_power,
	.esd_recover = lcm_esd_recover,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.set_gamma_mode_cmdq = lcm_set_gamma_mode_cmdq,
	.set_cabc_mode_cmdq = lcm_set_cabc_cmdq,
	.get_cabc_status = lcm_get_cabc_status,
	.suspend_power_1p8 = lcm_suspend_power_1p8,
};

