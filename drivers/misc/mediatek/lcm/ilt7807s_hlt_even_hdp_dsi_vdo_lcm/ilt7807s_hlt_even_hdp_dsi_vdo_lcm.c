/***********************************************************
** Copyright (C), 2008-2016, OPLUS Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** File: - ilt7807s_hlt_hdp_dsi_vdo_lcm.c
** Description: source file for lcm ilt7807s_hlt in kernel stage
**
** Version: 1.0
** Date : 2020/1/6
**
** ------------------------------- Revision History: -------------------------------
**  	<author>		<data> 	   <version >	       <desc>
**  lianghao       2020/1/6     1.0     source file for lcm ilt7807s_hlt in kernel stage
**
****************************************************************/
#define pr_fmt(fmt) "dsi_cmd: %s: " fmt, __func__

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

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

//#ifdef ODM_WT_EDIT
//#include <linux/update_tpfw_notifier.h>
//#endif
//#include "disp_cust.h"
#include <soc/oplus/device_info.h>
static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define SET_LCM_VDD18_PIN(v)	(lcm_util.set_gpio_lcm_vddio_ctl((v)))
#define SET_LCM_VSP_PIN(v)	(lcm_util.set_gpio_lcd_enp_bias((v)))
#define SET_LCM_VSN_PIN(v)	(lcm_util.set_gpio_lcd_enn_bias((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

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
#include <linux/kernel.h>
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

#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH										(720)
#define FRAME_HEIGHT										(1600)

#define LCM_PHYSICAL_WIDTH									(67930)
#define LCM_PHYSICAL_HEIGHT									(150960)

#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE		0xFFFD
#define REGFLAG_RESET_LOW		0xFFFE
#define REGFLAG_RESET_HIGH		0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif
//extern unsigned int esd_recovery_backlight_level;
extern int gesture_flag;
extern int tp_gesture_enable_flag(void);
//extern void ili_resume_by_ddi(void);
//extern void core_config_sleep_ctrl(bool out);
struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} }
};

static int blmap_table[] = {
					36, 5,
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

static struct LCM_setting_table init_setting_cmd[] = {
	{ 0xFF, 0x03, {0x98, 0x82, 0x03}},
};

static struct LCM_setting_table init_setting_vdo[] = {
		{0xFF, 03, {0x78, 0x07, 0x00}}, /*Page0*/
		{0x11, 01, {0x00}},
		{REGFLAG_DELAY, 110, {}},       //modify for timing between 60~120ms

		{0xFF, 03, {0x78, 0x07, 0x01}}, 	/*page1*/
		{0x00, 01, {0x42}},
		{0x01, 01, {0x51}},
		{0x02, 01, {0x00}},	  /*FTI_A_RISE*/
		{0x03, 01, {0x9F}},	  /*FTI_A_FALL*/
		{0x04, 01, {0x00}},
		{0x05, 01, {0x00}},
		{0x06, 01, {0x00}},
		{0x07, 01, {0x00}},
		{0x08, 01, {0x89}},	  /*CLK RISE          */
		{0x09, 01, {0x0A}},	  /*DUMMY CKV CLK FALL*/
		{0x0a, 01, {0x30}},
		{0x0b, 01, {0x00}},
		{0x0C, 01, {0x04}},   /*CLW_A1_RISE*/
		{0x0E, 01, {0x04}},   /*CLW_A1_FALL*/ 

		{0xFF, 03, {0x78, 0x07, 0x11}}, 	/*page11*/
		{0x00, 01, {0x04}},  /*CKV CLW Front*/
		{0x01, 01, {0x04}},  /*CKV CLW back */
		{0x18, 01, {0x5A}},  /*reftp_rise_LH*/
		{0x19, 01, {0x00}},	 /*reftp_fall_LH*/
		{0x1A, 01, {0x5A}},	 /*reftp_rise_LV*/
		{0x1B, 01, {0x6A}},	 /*reftp_fall_LV*/
		{0x1C, 01, {0x64}},	 /*GOFF_Fall    */
		{0x1D, 01, {0x00}},	 /*GOFF_Rise    */

		{0xFF, 03, {0x78, 0x07, 0x0E}},
		{0x02, 01, {0x0D}},

		{0xFF, 03, {0x78, 0x07, 0x01}},
		{0x31, 01, {0x07}},	  /*GOUTR01                */
		{0x32, 01, {0x07}},	  /*GOUTR02                */
		{0x33, 01, {0x07}},	  /*GOUTR03                */
		{0x34, 01, {0x07}},	  /*GOUTR04                */
		{0x35, 01, {0x07}},	  /*GOUTR05                */
		{0x36, 01, {0x07}},	  /*GOUTR06                */
		{0x37, 01, {0x07}},	  /*GOUTR07                */
		{0x38, 01, {0x40}},	  /*GOUTR08 CTSW_VCOM      */
		{0x39, 01, {0x40}},	  /*GOUTR09 CTSW_VCOM      */
		{0x3a, 01, {0x02}},	  /*GOUTR10 CTSW           */
		{0x3b, 01, {0x24}},	  /*GOUTR11 EN_TOUCH       */
		{0x3c, 01, {0x00}},	  /*GOUTR12 VGL_G          */
		{0x3d, 01, {0x00}},	  /*GOUTR13 VGL_G          */
		{0x3e, 01, {0x30}},	  /*GOUTR14 MUXB           */
		{0x3f, 01, {0x2F}},	  /*GOUTR15 MUXG           */
		{0x40, 01, {0x2E}},	  /*GOUTR16 MUXR           */
		{0x41, 01, {0x10}},	  /*GOUTR17 CKBL           */
		{0x42, 01, {0x12}},	  /*GOUTR18 CKL            */
		{0x43, 01, {0x2D}},	  /*GOUTR19 RESET          */
		{0x44, 01, {0x41}},	  /*GOUTR20 VGH_G          */
		{0x45, 01, {0x41}},	  /*GOUTR21 VGH_G          */
		{0x46, 01, {0x01}},	  /*GOUTR22 CN             */
		{0x47, 01, {0x24}},	  /*GOUTR23 CNB            */
		{0x48, 01, {0x08}},	  /*GOUTR24 STVL           */
		{0x49, 01, {0x07}},	  /*GOUTL01                */
		{0x4a, 01, {0x07}},	  /*GOUTL02                */
		{0x4b, 01, {0x07}},	  /*GOUTL03                */
		{0x4c, 01, {0x07}},	  /*GOUTL04                */
		{0x4d, 01, {0x07}},	  /*GOUTL05                */
		{0x4e, 01, {0x07}},	  /*GOUTL06                */
		{0x4f, 01, {0x07}},	  /*GOUTL07                */
		{0x50, 01, {0x40}},	  /*GOUTL08 CTSW_VCOM      */
		{0x51, 01, {0x40}},	  /*GOUTL09 CTSW_VCOM      */
		{0x52, 01, {0x02}},	  /*GOUTL10 CTSW           */
		{0x53, 01, {0x24}},	  /*GOUTL11 EN_TOUCH       */
		{0x54, 01, {0x00}},	  /*GOUTL12 VGL_G          */
		{0x55, 01, {0x00}},	  /*GOUTL13 VGL_G          */
		{0x56, 01, {0x30}},	  /*GOUTL14 MUXB           */
		{0x57, 01, {0x2F}},	  /*GOUTL15 MUXG           */
		{0x58, 01, {0x2E}},	  /*GOUTL16 MUXR           */
		{0x59, 01, {0x11}},	  /*GOUTL17 CKBR           */
		{0x5a, 01, {0x13}},	  /*GOUTL18 CKR            */
		{0x5b, 01, {0x2D}},	  /*GOUTL19 RESET          */
		{0x5c, 01, {0x41}},	  /*GOUTL20 VGH_G          */
		{0x5d, 01, {0x41}},	  /*GOUTL21 VGH_G          */
		{0x5e, 01, {0x01}},	  /*GOUTL22 CN             */
		{0x5f, 01, {0x24}},	  /*GOUTL23 CNB            */
		{0x60, 01, {0x09}},	  /*GOUTL24 STVR           */
		{0x61, 01, {0x07}},	  /*GOUTR01                */
		{0x62, 01, {0x07}},	  /*GOUTR02                */
		{0x63, 01, {0x07}},	  /*GOUTR03                */
		{0x64, 01, {0x07}},	  /*GOUTR04                */
		{0x65, 01, {0x07}},	  /*GOUTR05                */
		{0x66, 01, {0x07}},	  /*GOUTR06                */
		{0x67, 01, {0x07}},	  /*GOUTR07                */
		{0x68, 01, {0x40}},	  /*GOUTR08 CTSW_VCOM      */
		{0x69, 01, {0x40}},	  /*GOUTR09 CTSW_VCOM VCSW1*/
		{0x6a, 01, {0x02}},	  /*GOUTR10 CTSW           */
		{0x6b, 01, {0x24}},	  /*GOUTR11 EN_TOUCH       */
		{0x6c, 01, {0x01}},	  /*GOUTR12 VGL_G          */
		{0x6d, 01, {0x01}},	  /*GOUTR13 VGL_G          */
		{0x6e, 01, {0x30}},	  /*GOUTR14 MUXB           */
		{0x6f, 01, {0x2F}},	  /*GOUTR15 MUXG           */
		{0x70, 01, {0x2E}},	  /*GOUTR16 MUXR           */
		{0x71, 01, {0x13}},	  /*GOUTR17 CKBL           */
		{0x72, 01, {0x11}},	  /*GOUTR18 CKL            */
		{0x73, 01, {0x2D}},	  /*GOUTR19 RESET          */
		{0x74, 01, {0x41}},	  /*GOUTR20 VGH_G          */
		{0x75, 01, {0x41}},	  /*GOUTR21 VGH_G VCSW2    */
		{0x76, 01, {0x24}},	  /*GOUTR22 CN             */
		{0x77, 01, {0x00}},	  /*GOUTR23 CNB            */
		{0x78, 01, {0x09}},	  /*GOUTR24 STVL           */
		{0x79, 01, {0x07}},	  /*GOUTL01                */
		{0x7a, 01, {0x07}},	  /*GOUTL02                */
		{0x7b, 01, {0x07}},	  /*GOUTL03                */
		{0x7c, 01, {0x07}},	  /*GOUTL04                */
		{0x7d, 01, {0x07}},	  /*GOUTL05                */
		{0x7e, 01, {0x07}},	  /*GOUTL06                */
		{0x7f, 01, {0x07}},	  /*GOUTL07                */
		{0x80, 01, {0x40}},	  /*GOUTL08 CTSW_VCOM      */
		{0x81, 01, {0x40}},	  /*GOUTL09 CTSW_VCOM      */
		{0x82, 01, {0x02}},	  /*GOUTL10 CTSW           */
		{0x83, 01, {0x24}},	  /*GOUTL11 EN_TOUCH       */
		{0x84, 01, {0x01}},	  /*GOUTL12 VGL_G          */
		{0x85, 01, {0x01}},	  /*GOUTL13 VGL_G          */
		{0x86, 01, {0x30}},	  /*GOUTL14 MUXB           */
		{0x87, 01, {0x2F}},	  /*GOUTL15 MUXG           */
		{0x88, 01, {0x2E}},	  /*GOUTL16 MUXR           */
		{0x89, 01, {0x12}},	  /*GOUTL17 CKBR           */
		{0x8a, 01, {0x10}},	  /*GOUTL18 CKR            */
		{0x8b, 01, {0x2D}},	  /*GOUTL19 RESET          */
		{0x8c, 01, {0x41}},	  /*GOUTL20 VGH_G          */
		{0x8d, 01, {0x41}},	  /*GOUTL21 VGH_G          */
		{0x8e, 01, {0x24}},	  /*GOUTL22 CN             */
		{0x8f, 01, {0x00}},	  /*GOUTL23 CNB            */
		{0x90, 01, {0x08}},	  /*GOUTL24 STVR           */
		{0x92, 01, {0x10}},	  /*CNB                    */
		{0xA2, 01, {0x4C}},	  /*stch2_rise             */
		{0xA3, 01, {0x49}},	  /*stch2_Fall             */
		{0xA7, 01, {0x10}},
		{0xAE, 01, {0x00}},
		{0xB0, 01, {0x20}},	  /*CNB*/
		{0xB4, 01, {0x05}},
		{0xC0, 01, {0x0C}},
		{0xC6, 01, {0x64}},	 /*GOFF_Fall*/
		{0xC7, 01, {0x00}},	 /*GOFF_Rise*/
		{0xD1, 01, {0x02}},
		{0xD2, 01, {0x10}},
		{0xD4, 01, {0x14}},
		{0xD5, 01, {0x01}},	/*GOFF_ON*/
		{0xD7, 01, {0x80}},
		{0xD8, 01, {0x40}},
		{0xD9, 01, {0x04}},	/* GOFF ON SEQ */
		{0xDF, 01, {0x4A}},
		{0xDE, 01, {0x03}},	/*VCSW1 OFF SEQ*/
		{0xDD, 01, {0x08}},	/* GOFF OFF SEQ*/
		//{0xE0, 01, {0x40}},	/*GOFF_Off   FAE--Chang20210129    */
		{0xE1, 01, {0x28}},	/*VGL_G_Off    */
		{0xE2, 01, {0x06}},	/*VCSW1_Off    */
		{0xE3, 01, {0x31}},
		{0xE4, 01, {0x02}},
		{0xE5, 01, {0x6D}},  /*CNB EN_TOUCH ABO      */
		{0xE6, 01, {0x12}},	 /*On_1_frame Off_2_frame*/
		{0xE7, 01, {0x0C}},
		{0xED, 01, {0x56}},

		{0xFF, 03, {0x78, 0x07, 0x02}},
		{0x01, 01, {0xD5}},		/*time out GIP toggle*/
		{0x47, 01, {0x00}},	    /*CKH connect        */
		{0x4F, 01, {0x01}},	    /*Dummy CKH connect  */
		{0x6B, 01, {0x11}},

		{0xFF, 03, {0x78, 0x07, 0x03}},
                {0x81, 01, {0x14}},
                {0x82, 01, {0x15}},
                {0x83, 01, {0x20}},          /*PWM 12bit*/
                {0x84, 01, {0x00}},          /*38.7K*/
                {0x88, 01, {0xCC}},          /*hei 80%*/
                {0x89, 01, {0xE5}},          /*hui 89.84%*/
                {0x8A, 01, {0xED}},          /*shuiguo 92.97%*/
                {0x8B, 01, {0xEF}},
                {0xB3, 01, {0xE5}},          /*zhuomian 89.84%*/
                {0xAC, 01, {0xFA}},          /*bai 98%*/
                {0xAD, 01, {0xE0}},          /*bai 87.89%*/
                {0x8C, 01, {0x9B}},          /*hei 60.94%*/
                {0x8D, 01, {0xA9}},
                {0x8E, 01, {0xAD}},
                {0x8F, 01, {0xAF}},
                {0x90, 01, {0xB2}},          /*hui 69.92%*/
                {0x91, 01, {0xBC}},
                {0x92, 01, {0xBD}},
                {0x93, 01, {0xBA}},          /*shuiguo 73%*/
                {0x94, 01, {0xCA}},
                {0x95, 01, {0xD5}},
                {0xB4, 01, {0xD9}},          /*zhuomian 71%->85.2%*/
                {0xAE, 01, {0xD3}},          /*bai 82.81%*/
                {0x96, 01, {0x70}},          /*hei 44.14%*/
                {0x97, 01, {0x89}},
                {0x98, 01, {0x99}},
                {0x99, 01, {0x9F}},
                {0x9A, 01, {0xA3}},          /*hui 64%*/
                {0x9B, 01, {0xA6}},
                {0x9C, 01, {0xA9}},
                {0x9D, 01, {0xAD}},          /*shuiguo 67.97%*/
                {0x9E, 01, {0xC2}},
                {0x9F, 01, {0xCC}},
                {0xB5, 01, {0xD0}},          /*zhuomian 64.84%->81%*/
                {0xA0, 01, {0xC0}},
                {0xA1, 01, {0x7E}},         /*threshold*/
                {0xA2, 01, {0x7E}},         /*threshold*/
                {0xA7, 01, {0xE5}},
                {0xA8, 01, {0xE5}},
                {0xB6, 01, {0x00}},
                {0xB7, 01, {0xD0}},
                {0xB8, 01, {0xD0}},
                {0xB1, 01, {0x66}},
                {0xB2, 01, {0x66}},
                {0x86, 01, {0x6C}},
                {0x87, 01, {0x6C}},

		{0xFF, 03, {0x78, 0x07, 0x04}},
		{0xBD, 01, {0x01}},

		{0xFF, 03, {0x78, 0x07, 0x05}},
        /*{0x1C, 01, {0x85}},*/ /*VCOM=-0.1V 8E*/
		{0x72, 01, {0x56}},		/*VGH=9V       */
		{0x74, 01, {0x56}},		/*VGL= -9V     */
		{0x76, 01, {0x51}},		/*VGHO=8V      */
		{0x7A, 01, {0x51}},		/*VGL0= -8V    */
		{0x7B, 01, {0x88}},		/*GVDDP=5.2V   */
		{0x7C, 01, {0x79}},		/*GVDDN: 0x88=-5.2V 0x79=-4.9V */
		{0xAE, 01, {0x29}},			/*pwr_d2a_cp_vgh_en    */
		{0xB1, 01, {0x39}},			/*pwr_d2a_cp_vgl_en    */
		{0x46, 01, {0x58}},			/*pwr_tcon_vgho_en     */
		{0x47, 01, {0x78}},			/*pwr_tcon_vglo_en     */
		{0xB5, 01, {0x58}},			/*PWR_D2A_HVREG_VGHO_EN*/
		{0xB7, 01, {0x78}},			/*PWR_D2A_HVREG_VGLO_EN*/
		{0xC9, 01, {0x90}},			/*Power off            */
		{0x56, 01, {0xFF}},			/*0924                 */

		{0xFF, 03, {0x78, 0x07, 0x06}},
		{0x08, 01, {0x20}},          /*PWM 0x20--28kHz*/
		//{0x3E, 01, {0xE2}}, 		/*11 dont reload otp*/
		{0xC0, 01, {0x40}},		/*Res=720*1600 Y       */
		{0xC1, 01, {0x16}},		/*Res=720*1600 Y       */
		{0xC2, 01, {0xFA}},		/*Res=720*1600 X       */
		{0xC3, 01, {0x06}},		/*ss_reg               */
		{0x96, 01, {0x50}},		/*save power mipi bais */
		{0xDD, 01, {0x17}},		/*3LANE 1207           */
		{0xD6, 01, {0x55}},
		{0xCD, 01, {0x66}},     //0x66 output VSYNC
		{0xB4, 01, {0xDC}},
		{0xB5, 01, {0x24}},

		/*Gamma Register*/
		{0xFF, 03, {0x78, 0x07, 0x08}},
                {0xE0, 31, {0x00, 0x00, 0x1B, 0x45, 0x00, 0x84, 0xB1, 0xD5, 0x15, 0x0D, 0x38, 0x7A, 0x25,
                            0xAA, 0xF3, 0x2F, 0x2A, 0x68, 0xA8, 0xD1, 0x3F, 0x04, 0x28, 0x52, 0x3F, 0x6F,
                            0x91, 0xBD, 0x0F, 0xD8, 0xD8}},
                {0xE1, 31, {0x00, 0x00, 0x1B, 0x45, 0x00, 0x84, 0xB1, 0xD5, 0x15, 0x0D, 0x38, 0x7A, 0x25,
                            0xAA, 0xF3, 0x2F, 0x2A, 0x68, 0xA8, 0xD1, 0x3F, 0x04, 0x28, 0x52, 0x3F, 0x6F,
                            0x91, 0xBD, 0x0F, 0xD8, 0xD8}},

		{0xFF, 03, {0x78, 0x07, 0x0B}},
		{0xC0, 01, {0x88}},
		{0xC1, 01, {0x1F}},
		{0xC2, 01, {0x06}},
		{0xC3, 01, {0x06}},
		{0xC4, 01, {0xCB}},
		{0xC5, 01, {0xCB}},
		{0xD2, 01, {0x45}},
		{0xD3, 01, {0x0A}},
		{0xD4, 01, {0x04}},
		{0xD5, 01, {0x04}},
		{0xD6, 01, {0x7E}},
		{0xD7, 01, {0x7E}},
		{0xAB, 01, {0xE0}},

		{0xFF, 03, {0x78, 0x07, 0x0C}}, 	/*TP Modulation */
		{0x00, 01, {0x27}},
		{0x01, 01, {0x9D}},
		{0x02, 01, {0x27}},
		{0x03, 01, {0x9E}},
		{0x04, 01, {0x27}},
		{0x05, 01, {0xA5}},
		{0x06, 01, {0x27}},
		{0x07, 01, {0xA5}},
		{0x08, 01, {0x27}},
		{0x09, 01, {0xA3}},
		{0x0A, 01, {0x27}},
		{0x0B, 01, {0xA0}},
		{0x0C, 01, {0x27}},
		{0x0D, 01, {0xA1}},
		{0x0E, 01, {0x27}},
		{0x0F, 01, {0xA4}},
		{0x10, 01, {0x27}},
		{0x11, 01, {0xA7}},
		{0x12, 01, {0x27}},
		{0x13, 01, {0XA6}},
		{0x14, 01, {0x27}},
		{0x15, 01, {0xA2}},

		{0xFF, 03, {0x78, 0x07, 0x0E}},
		{0x00, 01, {0xA3}},		/*LH mode                                  */
		{0x04, 01, {0x00}},		/*TSHD_1VP off & TSVD free run on (00 OFF) */
		{0x20, 01, {0x07}},		/*5 unit                                   */
		{0x27, 01, {0x00}},		/*Unit line=320                            */
		{0x29, 01, {0xC8}},		/*Unit line=320                            */
		{0x25, 01, {0x0B}},		/*TP2_unit0=170us                          */
		{0x26, 01, {0x0C}},		/*TP2_unit0=170us                          */
		{0x2D, 01, {0x21}},		/*RTN 6.125 usec                           */
		{0x30, 01, {0x04}},		/*RTN 6.125 usec                           */
		{0x21, 01, {0x24}},		/*TSVD1 rising position shift  24          */
		{0x23, 01, {0x44}},		/*TSVD2 rising position shift  44          */
		{0xB0, 01, {0x21}},		/*TP1 unit                                 */
		{0xC0, 01, {0x12}},		/*TP3 unit                                 */
		{0x05, 01, {0x20}},		/*PageC TP modulation ON (20_ON / 24_OFF)  */
		{0x2B, 01, {0x0A}},		/*PageC TP modulation step                 */

		{0xFF, 03, {0x78, 0x07, 0x12}},
		{0x10, 01, {0x10}},		/*t8_de 1207 */
		{0x12, 01, {0x08}},		/*t9_de 1207 */
		{0x13, 01, {0x4F}},		/*t7_de 1207 */
		{0x16, 01, {0x08}},		/*SOURCE SDT */
		{0x3A, 01, {0x05}},		/*PCT2       */
		{0xC0, 01, {0xA1}},		/*BIST RTNA  */
		{0xC2, 01, {0x12}},		/*BIST VBP	1*/
		{0xC3, 01, {0x20}},		/*BIST VFP	3*/

		{0xFF, 03, {0x78, 0x07, 0x1E}},
		{0xAB, 01, {0x23}},		/*TSVD2 position*/
		{0xA2, 01, {0x00}},		/*TSHD_LV_FALLIN*/
		{0xAD, 01, {0x02}},		/*FOR  90Hz     */
		{0x00, 01, {0x91}},		/*90Hz TP1-1    */
		{0x08, 01, {0x91}},		/*90Hz TP3-2    */
		{0x09, 01, {0x91}},		/*90Hz TP3-1    */
		{0x0A, 01, {0x10}},		/*TP3 t8_de     */
		{0x0C, 01, {0x08}},		/*TP3 t9_de     */
		{0x0D, 01, {0x4F}},		/*TP3 t7_de     */

		{0xFF, 03, {0x78, 0x07, 0x00}},
		{0x68, 02, {0x04,0x00}},
		{0x51, 02, {0x00,0x00}},
		{0x53, 01, {0x24}},
		{0x29, 01, {0x00}},
		{REGFLAG_DELAY, 20, {}},
		{0x35, 01, {0x00}},
		{REGFLAG_DELAY, 20, {}},

};

static struct LCM_setting_table bl_level[] = {
	{0x51, 2, {0x00,0xFF} }
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

/*static void push_table_cust(void *cmdq, struct LCM_setting_table_V3*table,
	unsigned int count, bool hs)
{
	set_lcm(table, count, hs);
}*/

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
	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 16;
	params->dsi.vertical_frontporch = 32;
	//params->dsi.vertical_frontporch_for_low_power = 540;
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.horizontal_sync_active = 12;
        params->dsi.horizontal_backporch = 64;
        params->dsi.horizontal_frontporch = 80;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 1;
	//params->dsi.HS_TRAIL = 6;
	//params->dsi.HS_PRPR = 5;
	params->dsi.CLK_HS_PRPR = 7;
	// jump pll_clk
	params->dsi.dynamic_switch_mipi = 1;
	params->dsi.horizontal_sync_active_dyn = 12;
	params->dsi.horizontal_backporch_dyn = 46;
	params->dsi.data_rate_dyn = 720;

#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 360;	/* this value must be in MTK suggested table */
#else
	params->dsi.data_rate = 735;	/* this value must be in MTK suggested table */
#endif
	//params->dsi.PLL_CK_CMD = 360;
	//params->dsi.PLL_CK_VDO = 360;
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif
	//params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->full_content = 1;
	params->corner_pattern_width = 720;
	params->corner_pattern_height = 75;
	params->corner_pattern_height_bot = 75;
#endif

	params->blmap = blmap_table;
	params->blmap_size = sizeof(blmap_table)/sizeof(blmap_table[0]);
	params->brightness_max = 4095;
	params->brightness_min = 6;

	register_device_proc("lcd", "ili7807s", "hlt_ilitek");

}

static void lcm_init_power(void)
{
	pr_debug("lcm_init_power\n");
	MDELAY(1);
	SET_LCM_VSP_PIN(1);
	MDELAY(3);
	SET_LCM_VSN_PIN(1);
}

static void lcm_suspend_power(void)
{
	pr_debug("lcm_suspend_power\n");
	if(!tp_gesture_enable_flag()) {
		printk("lcm_tp_suspend_power_on\n");
		SET_LCM_VSN_PIN(0);
		MDELAY(2);
		SET_LCM_VSP_PIN(0);
	}
}

#ifdef OPLUS_BUG_STABILITY
static void lcm_shudown_power(void)
{
    printk("samir lcm_shudown_power\n");
    SET_RESET_PIN(0);
    MDELAY(2);
    SET_LCM_VSN_PIN(0);
    MDELAY(2);
    SET_LCM_VSP_PIN(0);
}
#endif

static void lcm_resume_power(void)
{
	pr_debug("lcm_resume_power\n");
	SET_LCM_VSP_PIN(1);
	MDELAY(3);
	SET_LCM_VSN_PIN(1);
	//base voltage = 4.0 each step = 100mV; 4.0+20 * 0.1 = 6.0v;
	if ( display_bias_setting(0x14) )
		pr_err("fatal error: lcd gate ic setting failed \n");
	MDELAY(5);
}

extern void lcd_queue_load_tp_fw(void);
static void lcm_init(void)
{
	pr_err("lcm_init\n");
	SET_RESET_PIN(0);
	MDELAY(2);
	SET_RESET_PIN(1);

	MDELAY(5);
	lcd_queue_load_tp_fw();
	MDELAY(10);

	if (lcm_dsi_mode == CMD_MODE) {
		push_table(NULL, init_setting_cmd, sizeof(init_setting_cmd) / sizeof(struct LCM_setting_table), 1);
		pr_debug("ilt7807s_hlt_lcm_mode = cmd mode :%d----\n", lcm_dsi_mode);
	} else {
		push_table(NULL, init_setting_vdo, sizeof(init_setting_vdo) / sizeof(struct LCM_setting_table), 1);
		pr_err("ilt7807s_hlt_lcm_mode = vdo mode :%d\n", lcm_dsi_mode);
	}
}

static void lcm_suspend(void)
{
	pr_err("lcm_suspend\n");

	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);


}

static void lcm_resume(void)
{
	pr_err("lcm_resume\n");
	lcm_init();
}

#define BL_X_MIN  2
#define BL_X_MID	2047
#define BL_X_MAX	4095

#define BL_Y_MIN  8
#define BL_Y_MID  3343
#define BL_Y_MAX  4095

#define VALUE_MASK 1000000
#define OPLUS_BRIGHT_TO_BL(out, v, BL_MIN, BL_MAX, BRIGHT_MIN, BRIGHT_MAX) do { \
	out = (((int)BL_MAX - (int)BL_MIN)*v + \
			((int)BRIGHT_MAX*(int)BL_MIN -(int)BRIGHT_MIN*(int)BL_MAX)) \
	/((int)BRIGHT_MAX - (int)BRIGHT_MIN); \
} while (0)

#define BL_LEVEL_MIN    8
static unsigned int oplus_private_set_backlight(unsigned int level)
{
	unsigned int value_a = 796;
	unsigned int value_b = BL_Y_MIN*1000000;
	unsigned int level_temp;

	if(level > 0) {
		if(level < 2048) {
			level_temp = (value_a * level * level  + value_b) / VALUE_MASK;
			pr_err("check brightness level= %d, level_temp == %u \n", level, level_temp);
			if (level_temp < BL_LEVEL_MIN) {
				level_temp = BL_LEVEL_MIN;
			}
		} else if ((level < 4096) && (level >= 2048)) {
			OPLUS_BRIGHT_TO_BL(level_temp, level, BL_Y_MID, BL_Y_MAX, BL_X_MID, BL_X_MAX);
			pr_err("check brightness level_temp == %u \n", level_temp);
		}
	} else if (0 == level) {
		level_temp = level;
	} else {
		pr_err("check brightness level fail level > 4095, level==%u ", level);
		level_temp = level;
	}

	return level_temp;
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	level = oplus_private_set_backlight(level);

	bl_level[0].para_list[0] = 0x000F&(level >> 8);
	bl_level[0].para_list[1] = 0x00FF&(level << 0);
	//MDELAY(5);
	pr_err("[ HW check backlight ilt7807s+hlt]level=%d,para_list[0]=%x,para_list[1]=%x\n",level,bl_level[0].para_list[0],bl_level[0].para_list[1]);
	push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	//push_table_cust(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table_V3), 0);
	//dump_stack();

}

static struct LCM_setting_table set_cabc_off[] = {
	{0x55, 0x01, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table set_cabc_ui[] = {
	{0x53, 0x01, {0x2C}},
	{0x55, 0x01, {0x01}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table set_cabc_still[] = {
	{0x53, 0x01, {0x2C}},
	{0x55, 0x01, {0x02}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table set_cabc_move[] = {
	{0x53, 0x01, {0x2C}},
	{0x55, 0x01, {0x03}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static int cabc_status;
static void lcm_set_cabc_cmdq(void *handle, unsigned int level){
	pr_err("[lcm] cabc set level %d\n", level);
	if (level==0){
		push_table(handle, set_cabc_off, sizeof(set_cabc_off) / sizeof(struct LCM_setting_table), 1);
	}else if (level == 1){
		push_table(handle, set_cabc_ui, sizeof(set_cabc_ui) / sizeof(struct LCM_setting_table), 1);
	}else if(level==2){
		push_table(handle, set_cabc_still, sizeof(set_cabc_still) / sizeof(struct LCM_setting_table), 1);
	}else if(level==3){
		push_table(handle, set_cabc_move, sizeof(set_cabc_move) / sizeof(struct LCM_setting_table), 1);
	}else{
		pr_info("[lcm]  level %d is not support\n", level);
	}
	cabc_status = level;
}

static void lcm_get_cabc_status(int *status){
	pr_info("[lcm] cabc get to %d\n", cabc_status);
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

	if (lcm_dsi_mode == CMD_MODE) {
		push_table(NULL, init_setting_cmd, sizeof(init_setting_cmd) / sizeof(struct LCM_setting_table), 1);
		pr_debug("ilt7807s_hlt_lcm_mode = cmd mode esd recovery :%d----\n", lcm_dsi_mode);
	} else {
		push_table(NULL, init_setting_vdo, sizeof(init_setting_vdo) / sizeof(struct LCM_setting_table), 1);
		pr_debug("ilt7807s_hlt_lcm_mode = vdo mode esd recovery :%d----\n", lcm_dsi_mode);
	}
	pr_debug("lcm_esd_recovery\n");
	push_table(NULL, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	//push_table_cust(NULL, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table_V3), 0);
	return FALSE;
#else
	return FALSE;
#endif
}

struct LCM_DRIVER ilt7807s_hlt_even_hdp_dsi_vdo_lcm_drv = {
	.name = "ilt7807s_hlt_even_hdp_dsi_vdo_lcm",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
#ifdef OPLUS_BUG_STABILITY
	.shutdown_power = lcm_shudown_power,
#endif
	.esd_recover = lcm_esd_recover,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,

	.set_cabc_mode_cmdq = lcm_set_cabc_cmdq,
	.get_cabc_status = lcm_get_cabc_status,
};

