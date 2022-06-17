/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_panel.h
** Description : oplus display panel char dev  /dev/oplus_panel
** Version : 1.0
** Date : 2020/06/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Sheng       2020/06/13        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DISPLAY_PANEL_H_
#define _OPLUS_DISPLAY_PANEL_H_

#include <linux/fs.h>
#include <linux/printk.h>
#include <linux/device.h>
#include <asm/ioctl.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cdev.h>

#define OPLUS_PANEL_NAME "oplus_display"
#define OPLUS_PANEL_CLASS_NAME "oplus_display_class"

#define OPLUS_PANEL_IOCTL_BASE			'o'

#define PANEL_IO(nr)			_IO(OPLUS_PANEL_IOCTL_BASE, nr)
#define PANEL_IOR(nr, type)		_IOR(OPLUS_PANEL_IOCTL_BASE, nr, type)
#define PANEL_IOW(nr, type)		_IOW(OPLUS_PANEL_IOCTL_BASE, nr, type)
#define PANEL_IOWR(nr, type)		_IOWR(OPLUS_PANEL_IOCTL_BASE, nr, type)

#define PANEL_IOCTL_NR(n)       _IOC_NR(n)
#define PANEL_IOCTL_SIZE(n)		_IOC_SIZE(n)

#define PANEL_IOCTL_DEF(ioctl, _func) \
	[PANEL_IOCTL_NR(ioctl)] = {		\
		.cmd = ioctl,			\
		.func = _func,			\
		.name = #ioctl,			\
	}

typedef int oplus_panel_feature(void *data);

#define PANEL_REG_MAX_LENS 28
#define PANEL_TX_MAX_BUF 112

static dev_t dev_num = 0;
static struct class *panel_class;
static struct device *panel_dev;
static int panel_ref = 0;
static struct cdev panel_cdev;

struct panel_ioctl_desc {
	unsigned int cmd;
	oplus_panel_feature *func;
	const char *name;
};

struct panel_vol_set{
	uint32_t panel_id;
	uint32_t panel_vol;
};

struct panel_vol_get{
	uint32_t panel_id;
	uint32_t panel_min;
	uint32_t panel_cur;
	uint32_t panel_max;
};

struct panel_id
{
	uint32_t DA;
	uint32_t DB;
	uint32_t DC;
};

struct panel_info{
	char version[32];
	char manufacture[32];
};

struct panel_serial_number
{
	char serial_number[40];
};

struct display_timing_info {
	uint32_t h_active;
	uint32_t v_active;
	uint32_t refresh_rate;
	uint32_t clk_rate_hz_h32;  /* the high 32bit of clk_rate_hz */
	uint32_t clk_rate_hz_l32;  /* the low 32bit of clk_rate_hz */
};

struct panel_reg_get {
	uint32_t reg_rw[PANEL_REG_MAX_LENS];
	uint32_t lens; /*reg_rw lens, lens represent for u32 to user space*/
};

struct panel_reg_rw {
	uint32_t rw_flags; /*1 for read, 0 for write*/
	uint32_t cmd;
	uint32_t lens;     /*lens represent for u8 to kernel space*/
	uint32_t value[PANEL_REG_MAX_LENS]; /*for read, value is empty, just user get function for read the value*/
};

struct kernel_loglevel {
	unsigned int enable;
	unsigned int log_level;
};

#define RAMLESS_AOD_AREA_NUM 6
struct panel_aod_area {
	uint32_t x;
	uint32_t y;
	uint32_t w;
	uint32_t h;
	uint32_t color;
	uint32_t bitdepth;
	uint32_t mono;
	uint32_t gray;
};

struct panel_aod_area_para {
	struct panel_aod_area aod_area[RAMLESS_AOD_AREA_NUM];
	uint32_t size;
};

/*oplus ioctl case start*/
#define PANEL_COMMOND_BASE 0x00
#define PANEL_COMMOND_MAX  0x5D

#define PANEL_IOCTL_SET_POWER				  PANEL_IOW(0x01, struct panel_vol_set)
#define PANEL_IOCTL_GET_POWER				  PANEL_IOWR(0x02, struct panel_vol_get)
#define PANEL_IOCTL_SET_SEED				  PANEL_IOW(0x03, unsigned int)
#define PANEL_IOCTL_GET_SEED				  PANEL_IOWR(0x04, unsigned int)
#define PANEL_IOCTL_GET_PANELID			      PANEL_IOWR(0x05, struct panel_id)
#define PANEL_IOCTL_SET_FFL				      PANEL_IOW(0x06, unsigned int)
#define PANEL_IOCTL_GET_FFL				      PANEL_IOWR(0x07, unsigned int)
#define PANEL_IOCTL_SET_AOD				      PANEL_IOW(0x08, unsigned int)
#define PANEL_IOCTL_GET_AOD				      PANEL_IOWR(0x09, unsigned int)
#define PANEL_IOCTL_SET_MAX_BRIGHTNESS		  PANEL_IOW(0x0A, unsigned int)
#define PANEL_IOCTL_GET_MAX_BRIGHTNESS		  PANEL_IOWR(0x0B, unsigned int)
#define PANEL_IOCTL_GET_PANELINFO			  PANEL_IOWR(0x0C, struct panel_info)
#define PANEL_IOCTL_GET_CCD				      PANEL_IOWR(0x0D, unsigned int)
#define PANEL_IOCTL_GET_SERIAL_NUMBER		  PANEL_IOWR(0x0E, struct panel_serial_number)
#define PANEL_IOCTL_SET_HBM				      PANEL_IOW(0x0F, unsigned int)
#define PANEL_IOCTL_GET_HBM				      PANEL_IOWR(0x10, unsigned int)
#define PANEL_IOCTL_SET_DIM_ALPHA			  PANEL_IOW(0x11, unsigned int)
#define PANEL_IOCTL_GET_DIM_ALPHA			  PANEL_IOWR(0x12, unsigned int)
#define PANEL_IOCTL_SET_DIM_DC_ALPHA		  PANEL_IOW(0x13, unsigned int)
#define PANEL_IOCTL_GET_DIM_DC_ALPHA		  PANEL_IOWR(0x14, unsigned int)
#define PANEL_IOCTL_SET_AUDIO_READY  		  PANEL_IOW(0x15, unsigned int)
#define PANEL_IOCTL_GET_DISPLAY_TIMING_INFO   PANEL_IOWR(0x16, struct display_timing_info)
#define PANEL_IOCTL_GET_PANEL_DSC 			  PANEL_IOWR(0x17, unsigned int)
#define PANEL_IOCTL_SET_POWER_STATUS		  PANEL_IOW(0x18, unsigned int)
#define PANEL_IOCTL_GET_POWER_STATUS		  PANEL_IOWR(0x19, unsigned int)
#define PANEL_IOCTL_SET_REGULATOR_CONTROL     PANEL_IOW(0x1A, unsigned int)
#define PANEL_IOCTL_SET_CLOSEBL_FLAG		  PANEL_IOW(0x1B, unsigned int)
#define PANEL_IOCTL_GET_CLOSEBL_FLAG		  PANEL_IOWR(0x1C, unsigned int)
#define PANEL_IOCTL_SET_PANEL_REG   		  PANEL_IOW(0x1D, struct panel_reg_rw)
#define PANEL_IOCTL_GET_PANEL_REG   		  PANEL_IOWR(0x1E, struct panel_reg_get)
#define PANEL_IOCTL_SET_DIMLAYER_HBM		  PANEL_IOW(0x1F, unsigned int)
#define PANEL_IOCTL_GET_DIMLAYER_HBM		  PANEL_IOWR(0x20, unsigned int)
#define PANEL_IOCTL_SET_DIMLAYER_BL_EN		  PANEL_IOW(0x21, unsigned int)
#define PANEL_IOCTL_GET_DIMLAYER_BL_EN		  PANEL_IOWR(0x22, unsigned int)
#define PANEL_IOCTL_SET_PANEL_BLANK           PANEL_IOW(0x23, unsigned int)
#define PANEL_IOCTL_SET_SPR		              PANEL_IOW(0x24, unsigned int)
#define PANEL_IOCTL_GET_SPR		              PANEL_IOWR(0x25, unsigned int)
#define PANEL_IOCTL_GET_ROUNDCORNER	          PANEL_IOWR(0x26, unsigned int)
#define PANEL_IOCTL_SET_DYNAMIC_OSC_CLOCK	  PANEL_IOW(0x27, unsigned int)
#define PANEL_IOCTL_GET_DYNAMIC_OSC_CLOCK	  PANEL_IOWR(0x28, unsigned int)
#define PANEL_IOCTL_SET_FP_PRESS              PANEL_IOW(0x29, unsigned int)
#define PANEL_IOCTL_SET_OPLUS_BRIGHTNESS      PANEL_IOW(0x2A, unsigned int)
#define PANEL_IOCTL_GET_OPLUS_BRIGHTNESS      PANEL_IOWR(0x2B, unsigned int)
#define PANEL_IOCTL_SET_LCM_CABC	          PANEL_IOW(0x2C, unsigned int)
#define PANEL_IOCTL_GET_LCM_CABC		      PANEL_IOWR(0x2D, unsigned int)
#define PANEL_IOCTL_SET_AOD_AREA	          PANEL_IOW(0x2E, struct panel_aod_area_para)
#define PANEL_IOCTL_GET_OPLUS_MAXBRIGHTNESS   PANEL_IOWR(0x2F, unsigned int)
#define PANEL_IOCTL_SET_ESD                   PANEL_IOW(0x2F, unsigned int)
#define PANEL_IOCTL_GET_ESD                   PANEL_IOW(0x30, unsigned int)
#define PANEL_IOCTL_SET_MTK_LOG_LEVEL         PANEL_IOW(0x31, struct kernel_loglevel)
#define PANEL_IOCTL_SET_CABC_STATUS			  PANEL_IOW(0x59, unsigned int)
#define PANEL_IOCTL_GET_CABC_STATUS			  PANEL_IOWR(0x5A, unsigned int)
#define PANEL_IOCTL_SET_DRE_STATUS			  PANEL_IOW(0x5B, unsigned int)
#define PANEL_IOCTL_GET_DRE_STATUS			  PANEL_IOWR(0x5C, unsigned int)

/*oplus ioctl case end*/

#endif /*_OPLUS_DISPLAY_PANEL_H_*/
