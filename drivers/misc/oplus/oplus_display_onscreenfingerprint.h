/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** OPLUS_BUG_STABILITY
** File : oplus_display_onscreenfingerprint.h
** Description : oplus_display_onscreenfingerprint. implement
** Version : 1.0
** Date : 2020/05/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Zhang.JianBin2020/05/13        1.0          Modify for MT6779_R
******************************************************************/
#ifndef _OPLUS_DISPLAY_ONSCREENFINGERPRINT_H_
#define _OPLUS_DISPLAY_ONSCREENFINGERPRINT_H_

#include <linux/err.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/leds.h>
#include "primary_display.h"
#include "ddp_hal.h"
#include "ddp_manager.h"
#include <linux/types.h>
#include "disp_session.h"
#include "disp_lcm.h"
#include "disp_helper.h"
#include "oplus_display_private_api.h"
#include <linux/fb.h>
extern int mtk_disp_lcm_set_hbm(bool en, struct disp_lcm_handle *plcm, void *qhandle);
extern void fingerprint_send_notify(struct fb_info *fbi, uint8_t fingerprint_op_mode);
int oplus_display_panel_set_hbm(void *buf);
int oplus_display_panel_get_hbm(void *buf);
int oplus_display_panel_set_finger_print(void *buf);
extern void oplus_cmdq_handle_clear_dirty(struct cmdqRecStruct *cmdq_handle);
extern void oplus_cmdq_flush_config_handle_mira(void *handle, int blocking);
extern void hbm_notify_init(void);
extern void  hbm_notify(void);
extern int primary_display_set_hbm_wait_ramless(bool en);
extern void fpd_notify(void);
extern int ramless_dc_wait;
#define RAMLESS_AOD_VSYNC_WAIT_COUNT 3
#endif /*_OPLUS_DISPLAY_ONSCREENFINGERPRINT_H_*/

