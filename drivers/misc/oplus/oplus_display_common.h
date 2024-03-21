/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** OPLUS_BUG_STABILITY
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
int primary_display_read_serial(char cmd, uint64_t *buf, int num);
int _read_serial_by_cmdq(char cmd, uint64_t *buf, int num);
int oplus_display_panel_get_serial_number(void *buf);
int panel_serial_number_read(char cmd, uint64_t *buf, int num);
int oplus_display_panel_set_closebl_flag(void *buf);
int oplus_display_panel_get_closebl_flag(void *buf);
int oplus_display_get_brightness(void *buf);
int oplus_display_set_brightness(void *buf);
int oplus_display_panel_set_cabc(void *buf);
int oplus_display_panel_get_cabc(void *buf);
#endif /*_OPLUS_DISPLAY_COMMON_H_*/
