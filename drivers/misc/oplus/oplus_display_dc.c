/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** OPLUS_BUG_STABILITY
** File : oplus_display_dc.c
** Description : oplus dc feature
** Version : 1.0
** Date : 2020/07/1
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  JianBin.Zhang   2020/07/01        1.0           Build this moudle
******************************************************************/
#include "oplus_display_dc.h"
#include "disp_drv_log.h"

int oplus_dc_alpha = 0;
int oplus_dc_enable = 0;
extern int oplus_panel_alpha;
extern int oplus_underbrightness_alpha;
extern bool primary_display_get_fp_hbm_state(void);
extern int __attribute__((weak)) oplus_get_panel_brightness_to_alpha(void) { return 0; };

int oplus_display_panel_set_dc_alpha(void *buf)
{
	unsigned int *set_dc_alpha = buf;
	oplus_dc_alpha = (*set_dc_alpha);
	pr_err("func is %s, value is %d", __func__, (*set_dc_alpha));
	return 0;
}

int oplus_display_panel_get_dc_alpha(void *buf)
{
	unsigned int *get_dc_alpha = buf;
	(*get_dc_alpha) = oplus_dc_alpha;
	pr_err("func is %s, value is %d", __func__, oplus_dc_alpha);
	return 0;
}

int oplus_display_panel_get_dimlayer_enable(void *buf)
{
	unsigned int *dc_enable = buf;
	(*dc_enable) = oplus_dc_enable;
	pr_err("func is %s, value is %d", __func__, oplus_dc_enable);
	return 0;
}

int oplus_display_panel_set_dimlayer_enable(void *buf)
{
	unsigned int *dc_enable = buf;
	oplus_dc_enable = (*dc_enable);
	pr_err("func is %s, value is %d", __func__, (*dc_enable));
	return 0;
}

int oplus_display_panel_get_dim_alpha(void *buf)
{
	unsigned int *dim_alpha = buf;

	if (!primary_display_get_fp_hbm_state()) {
		(*dim_alpha) = 0;
		return 0;
	}

	oplus_underbrightness_alpha = oplus_get_panel_brightness_to_alpha();
	(*dim_alpha) = oplus_underbrightness_alpha;

	return 0;
}

int oplus_display_panel_set_dim_alpha(void *buf)
{
	unsigned int *dim_alpha = buf;

	oplus_panel_alpha = (*dim_alpha);

	return 0;
}

int oplus_display_panel_get_dim_dc_alpha(void *buf)
{
	unsigned int *dim_dc_alpha = buf;

	(*dim_dc_alpha) = oplus_dc_alpha;

	return 0;
}

int oplus_display_panel_set_dim_dc_alpha(void *buf)
{
	unsigned int *dim_dc_alpha = buf;

	oplus_dc_alpha = (*dim_dc_alpha);

	return 0;
}
