/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_common.c
** Description : oplus common feature
** Version : 1.0
** Date : 2020/07/1
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  JianBin.Zhang   2020/07/01        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DISPLAY_COMMON_H_
#define _OPLUS_DISPLAY_COMMON_H_

#include <oplus_display_private_api.h>

int oplus_display_panel_get_max_brightness(void *buf);
int oplus_display_panel_get_serial_number(void *buf);
int oplus_display_panel_set_closebl_flag(void *buf);
int oplus_display_panel_get_closebl_flag(void *buf);
int oplus_display_get_brightness(void *buf);
int oplus_display_set_brightness(void *buf);
int oplus_display_panel_set_cabc(void *buf);
int oplus_display_panel_get_cabc(void *buf);
int oplus_display_panel_set_esd(void *buf);
int oplus_display_panel_get_esd(void *buf);
int oplus_display_set_aod_area(void *buf);
unsigned int oplus_get_ssc_config_data(void);
#endif /*_OPLUS_DISPLAY_COMMON_H_*/
