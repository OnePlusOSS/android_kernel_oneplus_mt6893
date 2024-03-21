/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_panel_power.h
** Description : oplus display panel power control
** Version : 1.0
** Date : 2020/06/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Sheng       2020/06/13        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DISPLAY_PANEL_POWER_H_
#define _OPLUS_DISPLAY_PANEL_POWER_H_

#include <linux/err.h>

enum PANEL_VOLTAGE_ENUM {
	PANEL_VOLTAGE_ID_VDDI = 0,
	PANEL_VOLTAGE_ID_VDDR,
	PANEL_VOLTAGE_ID_VG_BASE,
	PANEL_VOLTAGE_ID_MAX,
};

#define PANEL_VOLTAGE_VALUE_COUNT 4

typedef struct panel_voltage_bak {
	u32 voltage_id;
	u32 voltage_min;
	u32 voltage_current;
	u32 voltage_max;
	char pwr_name[20];
}PANEL_VOLTAGE_BAK;

int oplus_panel_set_vg_base(unsigned int panel_vol_value);

#endif /*_OPLUS_DISPLAY_PANEL_POWER_H_*/
