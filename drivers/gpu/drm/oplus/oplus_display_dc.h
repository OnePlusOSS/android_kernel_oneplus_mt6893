/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_dc.c
** Description : oplus dc feature
** Version : 1.0
** Date : 2020/07/1
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  JianBin.Zhang   2020/07/01        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DISPLAY_DC_H_
#define _OPLUS_DISPLAY_DC_H_
#include <linux/err.h>
#include <linux/of.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>

int oplus_display_panel_set_dc_alpha(void *buf);
int oplus_display_panel_get_dc_alpha(void *buf);
int oplus_display_panel_get_dimlayer_enable(void *buf);
int oplus_display_panel_set_dimlayer_enable(void *buf);
int oplus_display_panel_set_dim_alpha(void *buf);
int oplus_display_panel_get_dim_alpha(void *buf);
int oplus_display_panel_set_dim_dc_alpha(void *buf);
int oplus_display_panel_get_dim_dc_alpha(void *buf);
#endif /*_OPLUS_DISPLAY_DC_H_*/
