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
#include "oplus_display_dc.h"

extern int oplus_panel_alpha;
extern int oplus_dc_alpha;
extern int oplus_underbrightness_alpha;
extern int oplus_dc_enable;
extern bool oplus_mtk_drm_get_hbm_state(void);
extern int oplus_get_panel_brightness_to_alpha(void);

int oplus_display_panel_get_dim_alpha(void *buf)
{
	unsigned int *dim_alpha = buf;

	if (!oplus_mtk_drm_get_hbm_state()) {
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

int oplus_display_panel_get_dimlayer_enable(void *buf)
{
	unsigned int *dimlayer_enable = buf;

	(*dimlayer_enable) = oplus_dc_enable;

	return 0;
}

int oplus_display_panel_set_dimlayer_enable(void *buf)
{
	unsigned int *dimlayer_enable = buf;

	pr_info("oplus_display_panel_set_dimlayer_enable %d\n", *dimlayer_enable);
	oplus_dc_enable = (*dimlayer_enable);

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

	pr_info("oplus_display_panel_set_dim_dc_alpha %d\n", *dim_dc_alpha);
	oplus_dc_alpha = (*dim_dc_alpha);

	return 0;
}


