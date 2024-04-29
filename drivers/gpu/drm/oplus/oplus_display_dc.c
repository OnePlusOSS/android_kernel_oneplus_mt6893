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
#ifdef CONFIG_OPLUS_OFP_V2
/* add for ofp */
#include "oplus_display_onscreenfingerprint.h"
int oplus_dc_flag = 0;
#endif

extern int oplus_panel_alpha;
extern int oplus_dc_alpha;
extern int oplus_underbrightness_alpha;
extern int oplus_dc_enable;
extern bool oplus_mtk_drm_get_hbm_state(void);
extern int oplus_get_panel_brightness_to_alpha(void);

int oplus_display_panel_get_dim_alpha(void *buf)
{
	unsigned int *dim_alpha = buf;

#ifdef CONFIG_OPLUS_OFP_V2
	if (!oplus_ofp_get_hbm_state()) {
#else
	if (!oplus_mtk_drm_get_hbm_state()) {
#endif
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
#ifdef CONFIG_OPLUS_OFP_V2
	struct drm_crtc *crtc = NULL;
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct drm_device *ddev = get_drm_device();
	int last_oplus_dc_flag = 0;
	if (!ddev) {
		printk(KERN_ERR "find ddev fail\n");
		return 0;
	}

	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (IS_ERR_OR_NULL(crtc)) {
		printk(KERN_ERR "find crtc fail\n");
		return 0;
	}

	mtk_crtc = to_mtk_crtc(crtc);
	if (mtk_crtc && (mtk_crtc->panel_ext->params->oplus_dc_exit_flag)) {
		last_oplus_dc_flag = (*dimlayer_enable);
		if (0 == last_oplus_dc_flag)
			oplus_dc_flag = 1;
		if (mtk_crtc && (!mtk_crtc->panel_ext->params->oplus_dc_moss_flag)) {
			if (1 == last_oplus_dc_flag)
				oplus_dc_flag = 2;
		}
		printk(" set_dimlayer_enable oplus_dc_flag = %d \n ", oplus_dc_flag);
	}
#endif
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


