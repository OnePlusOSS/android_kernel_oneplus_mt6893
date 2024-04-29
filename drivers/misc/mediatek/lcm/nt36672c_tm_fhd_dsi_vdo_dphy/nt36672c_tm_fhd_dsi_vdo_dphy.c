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

#include <linux/fb.h>
#include <linux/notifier.h>
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

#define SET_RESET_PIN(v)        (lcm_util.set_reset_pin((v)))
#define SET_LCM_VDD18_PIN(v)    (lcm_util.set_gpio_lcm_vddio_ctl((v)))
#define SET_LCM_VSP_PIN(v)      (lcm_util.set_gpio_lcd_enp_bias((v)))
#define SET_LCM_VSN_PIN(v)      (lcm_util.set_gpio_lcd_enn_bias((v)))
#define MDELAY(n)               (lcm_util.mdelay(n))
#define UDELAY(n)               (lcm_util.udelay(n))

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
#ifdef CONFIG_MTK_MT6382_BDG
#define dsi_set_cmdq_backlight(cmdq, cmd, count, para_list, force_update) \
lcm_util.dsi_mt6382_send_cmd(cmdq, cmd, count, para_list, force_update)
#define dsi_set_cmdq_cabc(cmdq, cmd, count, para_list, force_update) \
lcm_util.dsi_mt6382_send_cabc_cmd(cmdq, cmd, count, para_list, force_update)
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
#endif

#define LCM_DSI_CMD_MODE        0
#define FRAME_WIDTH             (1080)
#define FRAME_HEIGHT            (2400)
#define LCM_PHYSICAL_WIDTH      (70000)
#define LCM_PHYSICAL_HEIGHT     (156000)
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
extern int shut_down_flag;
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
static unsigned int tm_bl_level = 0;
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
	176, 1555
};
#endif

static struct LCM_setting_table init_setting_cmd[] = {
	/* { 0xFF, 0x03, {0x98, 0x81, 0x03} }, */
	{ 0xFF, 0x03, {0x98, 0x07, 0x00} },
};

static struct LCM_setting_table bl_level[] = {
	/* { 0xFF, 0x03, {0x98, 0x81, 0x00} }, */
	{0x51, 2, {0x00, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static unsigned char bl_level_mt6382[2] = {
	0x00, 0x00
};

#ifdef CONFIG_MTK_MT6382_BDG
static unsigned char dimming_level[2] = {
	0x02, 0x01
};
#endif

#define LCM_NT36672C 1
#if LCM_NT36672C
static struct LCM_setting_table init_setting_vdo[] = {
	{0xFF, 0x01, {0x10}},
	{0xFB, 0x01, {0x01}},
	{0x3B, 0x05, {0x03, 0x14, 0x36, 0x04, 0x04}},
	{0xB0, 0x01, {0x00}},
	{0xC2, 0x02, {0x1B, 0xA0}},
	{0xE9, 0x01, {0x01}},
	{0xFF, 0x01, {0x20}},
	{0xFB, 0x01, {0x01}},
	{0x01, 0x01, {0x66}},
	{0x06, 0x01, {0x50}},
	{0x07, 0x01, {0x28}},
	{0x17, 0x01, {0x55}},
	{0x18, 0x01, {0x66}},
	{0x1B, 0x01, {0x01}},
	{0x2F, 0x01, {0x83}},
	{0x5C, 0x01, {0x90}},
	{0x5E, 0x01, {0xAA}},
	{0x69, 0x01, {0xD0}},
	{0x6D, 0x01, {0x55}},
	{0x89, 0x01, {0x0D}},
	{0x8A, 0x01, {0x0D}},
	{0x95, 0x01, {0xD1}},
	{0x96, 0x01, {0xD1}},
	{0xF2, 0x01, {0x66}},
	{0xF3, 0x01, {0x54}},
	{0xF4, 0x01, {0x66}},
	{0xF5, 0x01, {0x54}},
	{0xF6, 0x01, {0x66}},
	{0xF7, 0x01, {0x54}},
	{0xF8, 0x01, {0x66}},
	{0xF9, 0x01, {0x54}},
	/* cabc begin */
	{0xFF, 0x01, {0x23}},
	{0xFB, 0x01, {0x01}},
	{0x00, 0x01, {0x80}},
	{0x05, 0x01, {0x22}},
	{0x06, 0x01, {0x01}},
	{0x07, 0x01, {0x00}},
	{0x08, 0x01, {0x01}},
	{0x09, 0x01, {0x00}},
	{0x10, 0x01, {0x82}},
	{0x11, 0x01, {0x01}},
	{0x12, 0x01, {0x95}},
	{0x15, 0x01, {0x68}},
	{0x16, 0x01, {0x0B}},
	{0x2B, 0x01, {0x9E}},
	{0x30, 0x01, {0xFB}},
	{0x31, 0x01, {0xF6}},
	{0x32, 0x01, {0xF5}},
	{0x33, 0x01, {0xEF}},
	{0x34, 0x01, {0xEC}},
	{0x35, 0x01, {0xEA}},
	{0x36, 0x01, {0xE9}},
	{REGFLAG_DELAY, 1, {}},
	{0x36, 0x01, {0xE9}},
	{0x37, 0x01, {0xE8}},
	{0x38, 0x01, {0xE6}},
	{0x39, 0x01, {0xE3}},
	{0x3A, 0x01, {0xDE}},
	{0x3B, 0x01, {0xDE}},
	{0x3D, 0x01, {0xDA}},
	{0x3F, 0x01, {0xD9}},
	{0x40, 0x01, {0xD8}},
	{0x41, 0x01, {0xD7}},
	{0x45, 0x01, {0xF1}},
	{0x46, 0x01, {0xED}},
	{0x47, 0x01, {0xEC}},
	{0x48, 0x01, {0xE7}},
	{0x49, 0x01, {0xE4}},
	{0x4A, 0x01, {0xDC}},
	{0x4B, 0x01, {0xD7}},
	{0x4C, 0x01, {0xD5}},
	{0x4D, 0x01, {0xCE}},
	{0x4E, 0x01, {0xCD}},
	{0x4F, 0x01, {0xC1}},
	{0x50, 0x01, {0xBF}},
	{0x51, 0x01, {0xBC}},
	{0x52, 0x01, {0xBB}},
	{0x53, 0x01, {0xBA}},
	{0x54, 0x01, {0xB9}},
	{0x58, 0x01, {0xDA}},
	{0x59, 0x01, {0xD5}},
	{0x5A, 0x01, {0xD3}},
	{0x5B, 0x01, {0xC9}},
	{0x5C, 0x01, {0xC3}},
	{0x5D, 0x01, {0xBA}},
	{0x5E, 0x01, {0xB6}},
	{0x5F, 0x01, {0xB5}},
	{0x60, 0x01, {0xAE}},
	{0x61, 0x01, {0xAB}},
	{0x62, 0x01, {0xAA}},
	{0x63, 0x01, {0xA8}},
	{0x64, 0x01, {0xA7}},
	{0x65, 0x01, {0xA5}},
	{0x66, 0x01, {0xA3}},
	{0x67, 0x01, {0xA1}},
	/* cabc end */
	{0xFF, 0x01, {0x24}},
	{0xFB, 0x01, {0x01}},
	{0x03, 0x01, {0x13}},
	{0x04, 0x01, {0x15}},
	{0x05, 0x01, {0x17}},
	{0x09, 0x01, {0x26}},
	{0x0A, 0x01, {0x27}},
	{0x0B, 0x01, {0x28}},
	{0x0C, 0x01, {0x24}},
	{0x0D, 0x01, {0x01}},
	{0x0E, 0x01, {0x2F}},
	{0x0F, 0x01, {0x2D}},
	{0x10, 0x01, {0x2E}},
	{0x11, 0x01, {0x2C}},
	{0x12, 0x01, {0x8B}},
	{0x13, 0x01, {0x8C}},
	{0x16, 0x01, {0x0F}},
	{0x17, 0x01, {0x22}},
	{0x1B, 0x01, {0x13}},
	{0x1C, 0x01, {0x15}},
	{0x1D, 0x01, {0x17}},
	{0x21, 0x01, {0x26}},
	{0x22, 0x01, {0x27}},
	{0x23, 0x01, {0x28}},
	{0x24, 0x01, {0x24}},
	{0x25, 0x01, {0x01}},
	{0x26, 0x01, {0x2F}},
	{0x27, 0x01, {0x2D}},
	{0x28, 0x01, {0x2E}},
	{0x29, 0x01, {0x2C}},
	{0x2A, 0x01, {0x8B}},
	{0x2B, 0x01, {0x8C}},
	{0x30, 0x01, {0x0F}},
	{0x31, 0x01, {0x22}},
	{0x32, 0x01, {0x00}},
	{0x33, 0x01, {0x03}},
	{0x35, 0x01, {0x01}},
	{0x36, 0x01, {0x01}},
	{0x4E, 0x01, {0x32}},
	{0x4F, 0x01, {0x32}},
	{0x53, 0x01, {0x32}},
	{0x71, 0x01, {0x28}},
	{0x77, 0x01, {0x80}},
	{0x79, 0x01, {0x03}},
	{0x7A, 0x01, {0x02}},
	{0x7B, 0x01, {0x8E}},
	{0x7D, 0x01, {0x02}},
	{0x80, 0x01, {0x02}},
	{0x81, 0x01, {0x02}},
	{0x82, 0x01, {0x13}},
	{0x84, 0x01, {0x31}},
	{0x85, 0x01, {0x13}},
	{0x86, 0x01, {0x22}},
	{0x87, 0x01, {0x31}},
	{0x90, 0x01, {0x13}},
	{0x92, 0x01, {0x31}},
	{0x93, 0x01, {0x13}},
	{0x94, 0x01, {0x22}},
	{0x95, 0x01, {0x31}},
	{0x9C, 0x01, {0xF4}},
	{0x9D, 0x01, {0x01}},
	{0xA0, 0x01, {0x0E}},
	{0xA2, 0x01, {0x0E}},
	{0xA3, 0x01, {0x02}},
	{0xA4, 0x01, {0x02}},
	{0xA5, 0x01, {0x02}},
	{0xC4, 0x01, {0x40}},
	{0xC6, 0x01, {0xC0}},
	{0xC9, 0x01, {0x00}},
	{0xD9, 0x01, {0x80}},
	{0xE9, 0x01, {0x02}},
	{0xFF, 0x01, {0x25}},
	{0xFB, 0x01, {0x01}},
	{0x0F, 0x01, {0x13}},
	{0x18, 0x01, {0x22}},
	{0x19, 0x01, {0xE4}},
	{0x20, 0x01, {0x00}},
	{0x21, 0x01, {0x40}},
	{0x66, 0x01, {0x40}},
	{0x67, 0x01, {0x29}},
	{0x68, 0x01, {0x50}},
	{0x69, 0x01, {0x20}},
	{0x6B, 0x01, {0x00}},
	{0x71, 0x01, {0x6D}},
	{0x77, 0x01, {0x60}},
	{0x78, 0x01, {0xA5}},
	{0x7D, 0x01, {0x40}},
	{0x7E, 0x01, {0x2D}},
	{0x84, 0x01, {0x70}},
	{0xC0, 0x01, {0x4D}},
	{0xC1, 0x01, {0xA9}},
	{0xC2, 0x01, {0xD2}},
	{0xC4, 0x01, {0x11}},
	{0xD6, 0x01, {0x80}},
	{0xD7, 0x01, {0x82}},
	{0xDA, 0x01, {0x02}},
	{0xDD, 0x01, {0x02}},
	{0xE0, 0x01, {0x02}},
	{0xF0, 0x01, {0x00}},
	{0xF1, 0x01, {0x04}},
	{0xFF, 0x01, {0x26}},
	{0xFB, 0x01, {0x01}},
	{0x00, 0x01, {0x10}},
	{0x01, 0x01, {0xED}},
	{0x03, 0x01, {0x00}},
	{0x04, 0x01, {0xED}},
	{0x05, 0x01, {0x08}},
	{0x06, 0x01, {0x0E}},
	{0x08, 0x01, {0x0E}},
	{0x14, 0x01, {0x06}},
	{0x15, 0x01, {0x01}},
	{0x74, 0x01, {0xAF}},
	{0x81, 0x01, {0x0D}},
	{0x83, 0x01, {0x02}},
	{0x85, 0x01, {0x01}},
	{0x86, 0x01, {0x04}},
	{0x87, 0x01, {0x01}},
	{0x88, 0x01, {0x00}},
	{0x8A, 0x01, {0x1A}},
	{0x8B, 0x01, {0x11}},
	{0x8C, 0x01, {0x24}},
	{0x8E, 0x01, {0x42}},
	{0x8F, 0x01, {0x11}},
	{0x90, 0x01, {0x11}},
	{0x91, 0x01, {0x11}},
	{0x9A, 0x01, {0x80}},
	{0x9B, 0x01, {0x04}},
	{0x9C, 0x01, {0x00}},
	{0x9D, 0x01, {0x00}},
	{0x9E, 0x01, {0x00}},
	{0xFF, 0x01, {0x27}},
	{0xFB, 0x01, {0x01}},
	{0x01, 0x01, {0x60}},
	{0x20, 0x01, {0x81}},
	{0x21, 0x01, {0x6D}},
	{0x25, 0x01, {0x81}},
	{0x26, 0x01, {0xAB}},
	{0x6E, 0x01, {0x23}},
	{0x6F, 0x01, {0x01}},
	{0x70, 0x01, {0x00}},
	{0x71, 0x01, {0x00}},
	{0x72, 0x01, {0x00}},
	{0x73, 0x01, {0x21}},
	{0x74, 0x01, {0x03}},
	{0x75, 0x01, {0x00}},
	{0x76, 0x01, {0x00}},
	{0x77, 0x01, {0x00}},
	{0x7D, 0x01, {0x09}},
	{0x7E, 0x01, {0x63}},
	{0x80, 0x01, {0x23}},
	{0x82, 0x01, {0x09}},
	{0x83, 0x01, {0x63}},
	{0x88, 0x01, {0x03}},
	{0x89, 0x01, {0x01}},
	{0xE3, 0x01, {0x02}},
	{0xE4, 0x01, {0xDA}},
	{0xE5, 0x01, {0x01}},
	{0xE6, 0x01, {0xE7}},
	{0xE9, 0x01, {0x03}},
	{0xEA, 0x01, {0x56}},
	{0xEB, 0x01, {0x02}},
	{0xEC, 0x01, {0x39}},
	{0xFF, 0x01, {0x2A}},
	{0xFB, 0x01, {0x01}},
	{0x00, 0x01, {0x91}},
	{0x03, 0x01, {0x20}},
	{0x04, 0x01, {0x78}},
	{0x07, 0x01, {0x50}},
	{0x0A, 0x01, {0x70}},
	{0x0C, 0x01, {0x09}},
	{0x0D, 0x01, {0x40}},
	{0x0E, 0x01, {0x02}},
	{0x0F, 0x01, {0x00}},
	{0x11, 0x01, {0xF0}},
	{0x15, 0x01, {0x0E}},
	{0x16, 0x01, {0xCF}},
	{0x19, 0x01, {0x0E}},
	{0x1A, 0x01, {0xA3}},
	{0x1B, 0x01, {0x14}},
	{0x1D, 0x01, {0x36}},
	{0x1E, 0x01, {0x37}},
	{0x1F, 0x01, {0x37}},
	{0x20, 0x01, {0x37}},
	{0x28, 0x01, {0xDF}},
	{0x29, 0x01, {0x1F}},
	{0x2A, 0x01, {0xFF}},
	{0x2D, 0x01, {0x05}},
	{0x2F, 0x01, {0x06}},
	{0x30, 0x01, {0x1E}},
	{0x31, 0x01, {0x8C}},
	{0x33, 0x01, {0x1E}},
	{0x34, 0x01, {0xE1}},
	{0x35, 0x01, {0x44}},
	{0x36, 0x01, {0xCC}},
	{0x37, 0x01, {0xDA}},
	{0x38, 0x01, {0x4A}},
	{0x39, 0x01, {0xC6}},
	{0x3A, 0x01, {0x1E}},
	{0x45, 0x01, {0x0E}},
	{0x46, 0x01, {0x40}},
	{0x47, 0x01, {0x03}},
	{0x4A, 0x01, {0xA0}},
	{0x4E, 0x01, {0x0E}},
	{0x4F, 0x01, {0xCF}},
	{0x52, 0x01, {0x0E}},
	{0x53, 0x01, {0xA3}},
	{0x54, 0x01, {0x14}},
	{0x56, 0x01, {0x36}},
	{0x57, 0x01, {0x76}},
	{0x58, 0x01, {0x76}},
	{0x59, 0x01, {0x76}},
	{0x60, 0x01, {0x80}},
	{0x61, 0x01, {0x8E}},
	{0x62, 0x01, {0x0B}},
	{0x63, 0x01, {0xA1}},
	{0x65, 0x01, {0x03}},
	{0x66, 0x01, {0x03}},
	{0x67, 0x01, {0x0F}},
	{0x68, 0x01, {0x68}},
	{0x6A, 0x01, {0xCE}},
	{0x6B, 0x01, {0x90}},
	{0x6C, 0x01, {0x20}},
	{0x6D, 0x01, {0xA6}},
	{0x6E, 0x01, {0x8D}},
	{0x6F, 0x01, {0x22}},
	{0x70, 0x01, {0xA4}},
	{0x71, 0x01, {0x0F}},
	{0x79, 0x01, {0x4C}},
	{0x7A, 0x01, {0x09}},
	{0x7B, 0x01, {0x40}},
	{0x7F, 0x01, {0xF0}},
	{0x83, 0x01, {0x0E}},
	{0x84, 0x01, {0xCF}},
	{0x87, 0x01, {0x0E}},
	{0x88, 0x01, {0xA3}},
	{0x89, 0x01, {0x14}},
	{0x8B, 0x01, {0x36}},
	{0x8C, 0x01, {0x4E}},
	{0x8D, 0x01, {0x4E}},
	{0x8E, 0x01, {0x4E}},
	{0x95, 0x01, {0x80}},
	{0x96, 0x01, {0xDF}},
	{0x97, 0x01, {0x17}},
	{0x98, 0x01, {0xA2}},
	{0x9A, 0x01, {0x05}},
	{0x9B, 0x01, {0x04}},
	{0x9C, 0x01, {0x14}},
	{0x9D, 0x01, {0xA5}},
	{0x9F, 0x01, {0xD1}},
	{0xA0, 0x01, {0xE1}},
	{0xA2, 0x01, {0x30}},
	{0xA3, 0x01, {0xFD}},
	{0xA4, 0x01, {0xDC}},
	{0xA5, 0x01, {0x34}},
	{0xA6, 0x01, {0xF9}},
	{0xA7, 0x01, {0x14}},
	{0xFF, 0x01, {0x2C}},
	{0xFB, 0x01, {0x01}},
	{0x00, 0x01, {0x02}},
	{0x01, 0x01, {0x02}},
	{0x02, 0x01, {0x02}},
	{0x03, 0x01, {0x21}},
	{0x04, 0x01, {0x21}},
	{0x05, 0x01, {0x21}},
	{0x0D, 0x01, {0x01}},
	{0x0E, 0x01, {0x01}},
	{0x17, 0x01, {0x73}},
	{0x18, 0x01, {0x73}},
	{0x19, 0x01, {0x73}},
	{0x2D, 0x01, {0xAF}},
	{0x2F, 0x01, {0x10}},
	{0x30, 0x01, {0xED}},
	{0x32, 0x01, {0x00}},
	{0x33, 0x01, {0xED}},
	{0x35, 0x01, {0x1E}},
	{0x37, 0x01, {0x1E}},
	{0x4D, 0x01, {0x21}},
	{0x4E, 0x01, {0x02}},
	{0x4F, 0x01, {0x03}},
	{0x53, 0x01, {0x02}},
	{0x54, 0x01, {0x02}},
	{0x55, 0x01, {0x02}},
	{0x56, 0x01, {0x13}},
	{0x58, 0x01, {0x13}},
	{0x59, 0x01, {0x13}},
	{0x61, 0x01, {0x01}},
	{0x62, 0x01, {0x01}},
	{0x6B, 0x01, {0x46}},
	{0x6C, 0x01, {0x46}},
	{0x6D, 0x01, {0x46}},
	{0x80, 0x01, {0xAF}},
	{0x81, 0x01, {0x10}},
	{0x82, 0x01, {0xED}},
	{0x84, 0x01, {0x00}},
	{0x85, 0x01, {0xED}},
	{0x87, 0x01, {0x14}},
	{0x89, 0x01, {0x14}},
	{0x9D, 0x01, {0x13}},
	{0x9E, 0x01, {0x02}},
	{0x9F, 0x01, {0x05}},
	{0xFF, 0x01, {0xE0}},
	{0xFB, 0x01, {0x01}},
	{0x35, 0x01, {0x82}},
	{0x9F, 0x01, {0x00}}, /* 2022/12/16 nine stripes */
	{0x9C, 0x01, {0x00}}, /* 2022/12/16 nine stripes */
	{0xFF, 0x01, {0xF0}},
	{0xFB, 0x01, {0x01}},
	{0x1C, 0x01, {0x01}},
	{0x33, 0x01, {0x01}},
	{0x5A, 0x01, {0x00}},
	{0x9C, 0x01, {0x17}},
	{0x9F, 0x01, {0x19}},
	{0xFF, 0x01, {0xD0}},
	{0xFB, 0x01, {0x01}},
	{0x53, 0x01, {0x22}},
	{0x54, 0x01, {0x02}},
	{0xFF, 0x01, {0xC0}},
	{0xFB, 0x01, {0x01}},
	{0x9C, 0x01, {0x11}},
	{0x9D, 0x01, {0x11}},
	{0xFF, 0x01, {0x2B}},
	{0xFB, 0x01, {0x01}},
	{0xB7, 0x01, {0x2A}},
	{0xB8, 0x01, {0x0C}},
	{0xC0, 0x01, {0x01}},
	{0xFF, 0x01, {0x10}},
	{0xFB, 0x01, {0x01}},
	{0xC1, 0x10, {0x89, 0x28, 0x00, 0x08, 0x00, 0xAA, 0x02, 0x0E, 0x00, 0x2B, 0x00, 0x07, 0x0D, 0xB7, 0x0C, 0xB7}},
	{REGFLAG_DELAY, 5, {}},
	{0xC2, 0x02, {0x1B, 0xA0}},
	{REGFLAG_DELAY, 5, {}},
	/* {0x35, 0x01, {0x00}}, */
	{0x68, 0x02, {0x02, 0x01}},
	{0x51, 0x02, {0x00, 0x00}},
	{0x53, 0x01, {0x2C}},
	{0x55, 0x01, {0x01}},
	{0x11, 0x01, {0x00}},
	{REGFLAG_DELAY, 120, {} },
	{0x29, 0x01, {0x00}},
	{REGFLAG_DELAY, 20, {} },
};
#endif

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

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

#if LCM_NT36672C
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
static void lcm_dfps_int(struct LCM_DSI_PARAMS *dsi)
{
	dsi->dfps_enable = 1;
	dsi->dfps_default_fps = 9000;/*real fps * 100, to support float*/
	dsi->dfps_def_vact_tim_fps = 9000;/*real vact timing fps * 100*/

	/* DPFS_LEVEL0 */
	dsi->dfps_params[0].level = DFPS_LEVEL0;
	dsi->dfps_params[0].fps = 6000;/*real fps * 100, to support float*/
	dsi->dfps_params[0].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/* if mipi clock solution */
	/* dfps_params[0].PLL_CLOCK = 500; */
	/* dfps_params[0].data_rate = xx; */
	/* if vfp solution */
	dsi->dfps_params[0].vertical_frontporch = 1291;

	/* DPFS_LEVEL1 */
	dsi->dfps_params[1].level = DFPS_LEVEL1;
	dsi->dfps_params[1].fps = 9000;/*real fps * 100, to support float*/
	dsi->dfps_params[1].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/* if mipi clock solution */
	/* dfps_params[1].PLL_CLOCK = 500 */
	/* dfps_params[1].data_rate = xx; */
	dsi->dfps_params[1].vertical_frontporch = 54;

	/* DPFS_LEVEL2 */
	dsi->dfps_params[2].level = DFPS_LEVEL2;
	dsi->dfps_params[2].fps = 4500;/*real fps * 100, to support float*/
	dsi->dfps_params[2].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/* if mipi clock solution */
	/* dfps_params[2].PLL_CLOCK = 500 */
	/* dfps_params[2].data_rate = xx; */
	dsi->dfps_params[2].vertical_frontporch = 2528;

	dsi->dfps_num = 3;
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
	/*params->dsi.vertical_sync_active = 4;*/
	/*params->dsi.vertical_backporch = 40;*/
	/*params->dsi.vertical_frontporch = 1280;*/
	/*params->dsi.vertical_frontporch_for_low_power = 2500;45HZ*/
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	/*params->dsi.horizontal_sync_active = 4;*/
	/*params->dsi.horizontal_backporch = 22;*/
	/*params->dsi.horizontal_frontporch = 22;*/
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.bdg_ssc_disable = 1;
	/*params->dsi.CLK_HS_PRPR = 7;*/

	params->dsi.dynamic_switch_mipi = 1;
	/*params->dsi.horizontal_sync_active_dyn = 4;*/
	/*params->dsi.horizontal_backporch_dyn = 30;*/
	params->dsi.data_rate_dyn = 769;
#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 360;	/* this value must be in MTK suggested table */
#else
	params->dsi.PLL_CLOCK = 380; /*501;*/
	params->dsi.data_rate = 760; /*1002;*/	/* this value must be in MTK suggested table */
#endif

#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
	params->blmap = blmap_table;
	params->blmap_size = sizeof(blmap_table)/sizeof(blmap_table[0]);
	params->brightness_max = 4096;
	params->brightness_min = 4;
	params->dsi.vertical_sync_active = 10;
	params->dsi.vertical_backporch = 10;
	params->dsi.vertical_frontporch = 54;
	params->dsi.horizontal_sync_active = 22;
	params->dsi.horizontal_backporch = 22;
	params->dsi.horizontal_frontporch = 164;
	params->dsi.ssc_disable = 1;
	params->dsi.dsc_enable = 0;
	params->dsi.bdg_dsc_enable = 1;
#ifdef CONFIG_MTK_MT6382_BDG
	params->dsi.dsc_params.ver = 17;
	params->dsi.dsc_params.slice_mode = 1;
	params->dsi.dsc_params.rgb_swap = 0;
	params->dsi.dsc_params.dsc_cfg = 34;
	params->dsi.dsc_params.rct_on = 1;
	params->dsi.dsc_params.bit_per_channel = 8;
	params->dsi.dsc_params.dsc_line_buf_depth = 9;
	params->dsi.dsc_params.bp_enable = 1;
	params->dsi.dsc_params.bit_per_pixel = 128;
	params->dsi.dsc_params.pic_height = 2400;
	params->dsi.dsc_params.pic_width = 1080;
	params->dsi.dsc_params.slice_height = 8;
	params->dsi.dsc_params.slice_width = 540;
	params->dsi.dsc_params.chunk_size = 540;
	params->dsi.dsc_params.xmit_delay = 170;
	params->dsi.dsc_params.dec_delay = 526;
	params->dsi.dsc_params.scale_value = 32;
	params->dsi.dsc_params.increment_interval = 43;
	params->dsi.dsc_params.decrement_interval = 7;
	params->dsi.dsc_params.line_bpg_offset = 12;
	params->dsi.dsc_params.nfl_bpg_offset = 3511;
	params->dsi.dsc_params.slice_bpg_offset = 3255;
	params->dsi.dsc_params.initial_offset = 6144;
	params->dsi.dsc_params.final_offset = 7072;
	params->dsi.dsc_params.flatness_minqp = 3;
	params->dsi.dsc_params.flatness_maxqp = 12;
	params->dsi.dsc_params.rc_model_size = 8192;
	params->dsi.dsc_params.rc_edge_factor = 6;
	params->dsi.dsc_params.rc_quant_incr_limit0 = 11;
	params->dsi.dsc_params.rc_quant_incr_limit1 = 11;
	params->dsi.dsc_params.rc_tgt_offset_hi = 3;
	params->dsi.dsc_params.rc_tgt_offset_lo = 3;
#endif
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	/* with dsc */
	/****DynFPS start****/
	lcm_dfps_int(&(params->dsi));
#endif
	/****DynFPS end****/
	/* register_device_proc("lcd", "nt36525b", "hlt"); */
	register_device_proc("lcd", "nt36672c", "cost");
}
#else
static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));
	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
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
	params->dsi.LANE_NUM = LCM_THREE_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	/*params->dsi.vertical_sync_active = 2;*/
	/*params->dsi.vertical_backporch = 254;*/
	/*params->dsi.vertical_frontporch = 10;*/
	/*params->dsi.vertical_frontporch_for_low_power = 540;*/
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	/*params->dsi.horizontal_sync_active = 4;*/
	/*params->dsi.horizontal_backporch = 18;*/
	params->dsi.horizontal_frontporch = 20;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	/*params->dsi.ssc_disable = 1;*/
	/*params->dsi.HS_TRAIL = 6;*/
	/*params->dsi.HS_PRPR = 5;*/
	params->dsi.CLK_HS_PRPR = 7;
	/*jump pll_clk*/
	/*params->dsi.horizontal_sync_active_ext = 4; */
	/*params->dsi.horizontal_backporch_ext = 30; */
	params->dsi.dynamic_switch_mipi = 1;
	params->dsi.horizontal_sync_active_dyn = 4;
	params->dsi.horizontal_backporch_dyn = 30;
	params->dsi.data_rate_dyn = 720;
#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 380;  /* this value must be in MTK suggested table */
#else
	params->dsi.data_rate = 760;  /* this value must be in MTK suggested table */
#endif
	/* params->dsi.PLL_CK_CMD = 360; */
	/* params->dsi.PLL_CK_VDO = 360; */
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif
	/*params->dsi.clk_lp_per_line_enable = 0;*/
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 0;
	params->full_content = 1;
	params->corner_pattern_width = 720;
	params->corner_pattern_height = 75;
	params->corner_pattern_height_bot = 75;
#endif
	params->blmap = blmap_table;
	params->blmap_size = sizeof(blmap_table)/sizeof(blmap_table[0]);
	params->brightness_max = 4096;
	params->brightness_min = 4;
	/* register_device_proc("lcd", "nt36525b", "hlt"); */
}
#endif

static void lcm_init_power(void)
{
	/*pr_debug("lcm_init_power\n");*/
	pr_debug("lcm_init_power\n");
	MDELAY(1);
	SET_LCM_VSP_PIN(1);
	MDELAY(3);
	SET_LCM_VSN_PIN(1);
}

static void lcm_suspend_power(void)
{
	pr_err("lcm_suspend_power\n");
	if (!tp_gesture_enable_flag()) {
		printk("lcm_tp_suspend_power_on\n");
		SET_LCM_VSN_PIN(0);
		MDELAY(6);
		SET_LCM_VSP_PIN(0);
	}
}

static void lcm_shudown_power(void)
{
	printk("nt36672c_lcm_mode lcm_shudown_power\n");
	MDELAY(12);
	SET_RESET_PIN(0);
	MDELAY(2);
	SET_LCM_VSN_PIN(0);
	MDELAY(6);
	SET_LCM_VSP_PIN(0);
}

static void lcm_resume_power(void)
{
	pr_err("lcm_resume_power\n");
	SET_LCM_VSP_PIN(1);
	MDELAY(3);
	SET_LCM_VSN_PIN(1);
	/*base voltage = 4.0 each step = 100mV; 4.0+20 * 0.1 = 6.0v;*/
	/* #if defined(MTK_LCM_DEVICE_TREE_SUPPORT_PASCAL_E) */
	if (display_bias_setting(0xf)) {
		pr_err("fatal error: lcd gate ic setting failed \n");
	}
	/* #endif */
	MDELAY(10);
}

extern void lcd_queue_load_tp_fw(void);
static void lcm_init(void)
{
	pr_err("lcm_init\n");
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(15);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(15);
	if (lcm_dsi_mode == CMD_MODE) {
		push_table(NULL, init_setting_cmd, sizeof(init_setting_cmd) / sizeof(struct LCM_setting_table), 1);
		pr_err("nt36672c_lcm_mode_lcm_mode = cmd mode :%d----\n", lcm_dsi_mode);
	} else {
		push_table(NULL, init_setting_vdo, sizeof(init_setting_vdo) / sizeof(struct LCM_setting_table), 1);
		pr_err("nt36672c_lcm_mode_lcm_mode = vdo mode :%d\n", lcm_dsi_mode);
	}
	lcd_queue_load_tp_fw();
	MDELAY(10);
}

static void lcm_suspend(void)
{
	struct fb_event event;
	int blank_mode = FB_BLANK_POWERDOWN;
	pr_err("lcm_suspend\n");

	pr_err("SHUTDOWN = %d\n", shut_down_flag);
	if (shut_down_flag == 1) {
		pr_err("SHUTDOWN ENTER!\n");
		event.info  = NULL;
		event.data = &blank_mode;
		fb_notifier_call_chain(FB_EARLY_EVENT_BLANK, &event);
	}

	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
	pr_err("lcm_resume\n");
	lcm_init();
}
/*
static struct LCM_setting_table lcm_cabc_enter_setting[] = {
	{0x53, 1, {0x2c}},
	{0x55, 1, {0x01}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_cabc_exit_setting[] = {
	{0x53, 1, {0x2c}},
	{0x55, 1, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/
#ifdef CONFIG_MTK_MT6382_BDG
static unsigned char set_cabc_off[2] = {
	0xff, 0x00
};
static unsigned char set_cabc_ui[2] = {
	0x2C, 0x01
};
static unsigned char set_cabc_still[2] = {
	0x2C, 0x02
};
static unsigned char set_cabc_move[2] = {
	0x2C, 0x03
};

static int cabc_status;
static void lcm_set_cabc_cmdq(void *handle, unsigned int level)
{
	pr_debug("[lcm] cabc set level %d\n", level);
	if (level == 0) {
		dsi_set_cmdq_cabc(handle, 0x55, 1, &set_cabc_off[1], 1);
	} else if (level == 1) {
		dsi_set_cmdq_cabc(handle, 0x55, 1, &set_cabc_ui[1], 1);
	} else if (level == 2) {
		dsi_set_cmdq_cabc(handle, 0x55, 1, &set_cabc_still[1], 1);
	} else if (level == 3) {
		dsi_set_cmdq_cabc(handle, 0x55, 1, &set_cabc_move[1], 1);
	} else {
		pr_info("[lcm]  level %d is not support\n", level);
	}
	cabc_status = level;
	/*dump_stack();*/
}
static void lcm_get_cabc_status(int *status)
{
	pr_debug("[lcm] cabc get to %d\n", cabc_status);
	*status = cabc_status;
}
#endif

/*
static void push_table_cust(void *cmdq, struct LCM_setting_table_V3*table, unsigned int count, bool hs)
{
    set_lcm(table, count, hs);
}

static struct LCM_setting_table_V3 bl_level[] = {
    {0x39, 0x51, 2, {0x04, 0x00} }
};
*/

static void mt6382_lcm_setbacklight_cmdq(void *handle, unsigned int *lcm_cmd, unsigned int *lcm_count, unsigned int *level_value)
{
	tm_bl_level = *level_value;
	/*tm_bl_level = oplus_private_set_backlight(tm_bl_level);*/

#ifdef CONFIG_MTK_MT6382_BDG
	if (tm_bl_level < 100 && tm_bl_level > 1) {
		dimming_level[0] = 0x05;
		dsi_set_cmdq_backlight(handle, 0x68,  2, dimming_level, 1);
	} else if (tm_bl_level < 200 && tm_bl_level > 99) {
		dimming_level[0] = 0x04;
		dsi_set_cmdq_backlight(handle, 0x68,  2, dimming_level, 1);
	} else if (tm_bl_level < 600 && tm_bl_level > 199) {
		dimming_level[0] = 0x03;
		dsi_set_cmdq_backlight(handle, 0x68,  2, dimming_level, 1);
	} else {
		dimming_level[0] = 0x02;
		dsi_set_cmdq_backlight(handle, 0x68,  2, dimming_level, 1);
	}
#endif

	if (0 == tm_bl_level) {
		/* push_table(handle, lcm_dimming_off_setting, sizeof(lcm_dimming_off_setting) / sizeof(struct LCM_setting_table), 1); */
		bl_level_mt6382[0] = 0;
		bl_level_mt6382[1] = 0;
		pr_err("[ HWlevelmt6382_ backlight nt36672c]level = %d para_list[0] = %x, \
		para_list[1]=%x\n", tm_bl_level, bl_level_mt6382[0], bl_level_mt6382[1]);
#ifdef CONFIG_MTK_MT6382_BDG
		dsi_set_cmdq_backlight(handle, 0x51,  2, bl_level_mt6382, 1);
#endif
	} else {
		bl_level_mt6382[0] = 0x000F&(tm_bl_level >> 8);
		bl_level_mt6382[1] = 0x00FF&(tm_bl_level >> 0);
		pr_err("[ HW check mt6382_ backlight nt36672c]level = %d para_list[0] = %x, \
		para_list[1]=%x\n", tm_bl_level, bl_level_mt6382[0], bl_level_mt6382[1]);
#ifdef CONFIG_MTK_MT6382_BDG
		dsi_set_cmdq_backlight(handle, 0x51,  2, bl_level_mt6382, 1);
#endif
		/*push_table_cust(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table_V3), 0);*/
	}
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
	if (lcm_dsi_mode == CMD_MODE) {
		push_table(NULL, init_setting_cmd, sizeof(init_setting_cmd) / sizeof(struct LCM_setting_table), 1);
		pr_debug("nt36672c_lcm_mode = cmd mode esd recovery :%d----\n", lcm_dsi_mode);
	} else {
		push_table(NULL, init_setting_vdo, sizeof(init_setting_vdo) / sizeof(struct LCM_setting_table), 1);
		pr_debug("nt36672c_lcm_mode = vdo mode esd recovery :%d----\n", lcm_dsi_mode);
	}
	lcd_queue_load_tp_fw();
	MDELAY(10);
	pr_debug("lcm_esd_recovery\n");
	push_table(NULL, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	/*push_table_cust(NULL, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table_V3), 0);*/
	return FALSE;
#else
	return FALSE;
#endif
}

struct LCM_DRIVER nt36672c_tm_fhd_dsi_vdo_dphy_lcm_drv = {
	.name = "nt36672c_tm_fhd_dsi_vdo_dphy_lcm_drv",
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
	.set_lcm_cmd = mt6382_lcm_setbacklight_cmdq,
#ifdef CONFIG_MTK_MT6382_BDG
	/*.set_cabc_cmdq = lcm_set_cabc_cmdq,*/
	.set_cabc_mode_cmdq = lcm_set_cabc_cmdq,
	.get_cabc_status = lcm_get_cabc_status,
#endif
};
