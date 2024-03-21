/***************************************************************
** Copyright (C),  2018,  OPLUS Mobile Comm Corp.,  Ltd
** OPLUS_BUG_STABILITY
** File : oplus_display_private_api.h
** Description : oplus display private api implement
** Version : 1.0
** Date : 2018/03/20
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Hu.Jie          2018/03/20        1.0           Build this moudle
**   Guo.Ling        2018/10/11        1.1           Modify for SDM660
**   Guo.Ling        2018/11/27        1.2           Modify for mt6779
**   Lin.Hao         2019/10/31        1.3           Modify for MT6779_Q
**   Zhang.JianBin2020/03/30        1.4           Modify for MT6779_R
******************************************************************/
#ifndef _OPLUS_DISPLAY_PRIVATE_API_H_
#define _OPLUS_DISPLAY_PRIVATE_API_H_

#include <linux/err.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/err.h>
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
#include "display_panel/oplus_display_panel.h"



extern atomic_t fpd_task_task_wakeup;
extern unsigned long fpd_hbm_time;
extern unsigned long fpd_send_uiready_time;
extern bool flag_lcd_off;
extern bool ds_rec_fpd;
extern bool doze_rec_fpd;
extern bool oplus_fp_notify_down_delay;
extern bool oplus_display_fppress_support;
/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
extern bool oplus_display_aod_ramless_support;
extern bool oplus_display_aod_support;
extern unsigned char aod_area_cmd[];
/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */

extern int primary_display_aod_backlight(int level);
extern int primary_display_setbacklight_nolock(unsigned int level);
extern void _primary_path_switch_dst_lock(void);
extern void _primary_path_switch_dst_unlock(void);
extern void _primary_path_lock(const char *caller);
extern void _primary_path_unlock(const char *caller);
extern bool primary_display_get_fp_hbm_state(void);
extern int panel_serial_number_read(char cmd, uint64_t *buf, int num);
extern int primary_display_read_serial(char addr, uint64_t *buf, int lenth);
extern int _is_lcm_inited(struct disp_lcm_handle *plcm);
extern int disp_lcm_aod(struct disp_lcm_handle *plcm, int enter);
extern int disp_lcm_aod_from_display_on(struct disp_lcm_handle *plcm);
extern int disp_lcm_set_aod_mode(struct disp_lcm_handle *plcm, void *handle, unsigned int mode);
extern int oplus_disp_lcm_set_hbm(struct disp_lcm_handle *plcm, void *handle, unsigned int hbm_level);
extern int primary_display_set_hbm_mode(unsigned int level);

extern enum  lcm_power_state primary_display_get_lcm_power_state_nolock(void);
extern enum DISP_POWER_STATE oplus_primary_set_state(enum DISP_POWER_STATE new_state);
extern enum lcm_power_state primary_display_set_lcm_power_state_nolock(enum lcm_power_state new_state);
extern enum mtkfb_power_mode primary_display_get_power_mode_nolock(void);

extern unsigned int __attribute__((weak)) DSI_dcs_read_lcm_reg_v3_wrapper_DSI0(unsigned char cmd,
		unsigned char *buffer, unsigned char buffer_size) { return 0; };
extern unsigned int __attribute__((weak)) DSI_dcs_read_lcm_reg_v4_wrapper_DSI0(unsigned char cmd,
		unsigned char *buffer, unsigned char buffer_size) { return 0; };

extern void oplus_cmdq_flush_config_handle_mira(void *handle, int blocking);
extern void oplus_cmdq_handle_clear_dirty(struct cmdqRecStruct *cmdq_handle);
extern void oplus_delayed_trigger_kick_set(int params);

extern int _read_serial_by_cmdq(char cmd, uint64_t *buf, int num);
extern int primary_display_set_aod_mode_nolock(unsigned int mode);
extern void oplus_display_aod_backlight(void);
extern int primary_display_set_safe_mode(unsigned int level);

#endif /* _OPLUS_DISPLAY_PRIVATE_API_H_ */

