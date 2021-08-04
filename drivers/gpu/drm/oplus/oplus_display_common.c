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
***************************************************************/
#include <oplus_display_common.h>
#include "display_panel/oplus_display_panel.h"

#define PANEL_SERIAL_NUM_REG 0xA1
#define PANEL_REG_READ_LEN   10
#define BOE_PANEL_SERIAL_NUM_REG 0xA3
#define PANEL_SERIAL_NUM_REG_TIANMA 0xD6

extern unsigned int hbm_mode;
extern unsigned long cabc_mode;
extern unsigned long cabc_true_mode;
extern unsigned long cabc_sun_flag;
extern unsigned long cabc_back_flag;
extern void disp_aal_set_dre_en(int enable);
extern unsigned long silence_mode;
extern unsigned long oplus_display_brightness;
extern int oplus_max_brightness;
extern unsigned long oplus_max_normal_brightness;
extern uint64_t serial_number;
extern unsigned long esd_mode;
extern unsigned long seed_mode;

extern struct drm_device* get_drm_device(void);
extern int mtk_drm_setbacklight(struct drm_crtc *crtc, unsigned int level);
extern int oplus_mtk_drm_sethbm(struct drm_crtc *crtc, unsigned int hbm_mode);
extern int panel_serial_number_read(char cmd, int num);
extern int oplus_mtk_drm_setcabc(struct drm_crtc *crtc, unsigned int hbm_mode);
extern int oplus_mtk_drm_setseed(struct drm_crtc *crtc, unsigned int seed_mode);

enum {
	CABC_LEVEL_0,
	CABC_LEVEL_1,
	CABC_LEVEL_2 = 3,
	CABC_EXIT_SPECIAL = 8,
	CABC_ENTER_SPECIAL = 9,
};

int oplus_display_set_brightness(void *buf)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *set_brightness = buf;
	unsigned int oplus_set_brightness = (*set_brightness);

	printk("%s %d\n", __func__, oplus_set_brightness);

	if (oplus_set_brightness > oplus_max_brightness || oplus_set_brightness < OPLUS_MIN_BRIGHTNESS) {
		printk(KERN_ERR "%s, brightness:%d out of scope\n", __func__, oplus_set_brightness);
		return -1;
	}

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}
	mtk_drm_setbacklight(crtc, oplus_set_brightness);

	return 0;
}

int oplus_display_get_brightness(void *buf)
{
	unsigned int *brightness = buf;

	(*brightness) = oplus_display_brightness;

	return 0;
}

int oplus_display_panel_get_max_brightness(void *buf)
{
	unsigned int *brightness = buf;

	(*brightness) = oplus_max_normal_brightness;

	return 0;
}

int oplus_display_panel_get_hbm(void *buf)
{
	unsigned int *hbm = buf;

	(*hbm) = hbm_mode;

	return 0;
}

int oplus_display_panel_set_hbm(void *buf)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *tmp = buf;

	printk("%s, %d to be %d\n", __func__, hbm_mode, *tmp);

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}

	oplus_mtk_drm_sethbm(crtc, *tmp);
	hbm_mode = (*tmp);

	if ((*tmp) == 1) {
		usleep_range(30000, 31000);
	}

	return 0;
}

int oplus_display_panel_get_serial_number(void *buf)
{
	struct panel_serial_number *p_snumber = buf;
	int ret = 0;

	printk("%s read serial number 0x%x\n", __func__, serial_number);
	ret = scnprintf(p_snumber->serial_number, PAGE_SIZE, "Get panel serial number: %llx\n", serial_number);
	return ret;
}

int oplus_display_panel_get_cabc(void *buf)
{
	unsigned int *c_mode = buf;

	printk("%s CABC_mode=%ld\n", __func__, cabc_true_mode);
	*c_mode = (unsigned int)cabc_true_mode;

	return 0;
}

int oplus_display_panel_set_cabc(void *buf)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *c_mode = buf;
	cabc_mode = (unsigned long)(*c_mode);

	cabc_true_mode = cabc_mode;
	printk("%s,cabc mode is %d, cabc_back_flag is %d\n", __func__, cabc_mode, cabc_back_flag);
	if (cabc_mode < 4) {
		cabc_back_flag = cabc_mode;
	}

	if (cabc_mode == CABC_ENTER_SPECIAL) {
		cabc_sun_flag = 1;
		cabc_true_mode = 0;
	} else if (cabc_mode == CABC_EXIT_SPECIAL) {
		cabc_sun_flag = 0;
		cabc_true_mode = cabc_back_flag;
	} else if (cabc_sun_flag == 1) {
		if (cabc_back_flag == CABC_LEVEL_0) {
			disp_aal_set_dre_en(1);
			printk("%s sun enable dre\n", __func__);
		} else {
			disp_aal_set_dre_en(0);
			printk("%s sun disable dre\n", __func__);
		}
		return 0;
	}

	printk("%s,cabc mode is %d\n", __func__, cabc_true_mode);

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}
	if (cabc_true_mode == CABC_LEVEL_0 && cabc_back_flag == CABC_LEVEL_0) {
		disp_aal_set_dre_en(1);
		printk("%s enable dre\n", __func__);
	} else {
		disp_aal_set_dre_en(0);
		printk("%s disable dre\n", __func__);
	}
	oplus_mtk_drm_setcabc(crtc, cabc_true_mode);
	if (cabc_true_mode != cabc_back_flag) {
		cabc_true_mode = cabc_back_flag;
	}

	return 0;
}

int oplus_display_panel_get_closebl_flag(void *buf)
{
	unsigned int *closebl_flag = buf;

	printk("%s silence_mode=%ld\n", __func__, silence_mode);
	(*closebl_flag) = silence_mode;

	return 0;
}

int oplus_display_panel_set_closebl_flag(void *buf)
{
	unsigned int *closebl_flag = buf;

	msleep(1000);
	silence_mode = (*closebl_flag);
	printk("%s silence_mode=%ld\n", __func__, silence_mode);

	return 0;
}

int oplus_display_panel_get_esd(void *buf)
{
	unsigned int *p_esd = buf;

	printk("%s esd=%ld\n", __func__, esd_mode);
	(*p_esd) = esd_mode;

	return 0;
}

int oplus_display_panel_set_esd(void *buf)
{
	unsigned int *p_esd = buf;

	esd_mode = (*p_esd);
	printk("%s,esd mode is %d\n", __func__, esd_mode);

	return 0;
}

int oplus_display_panel_get_vendor(void *buf)
{
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct panel_info *p_info = buf;
	struct drm_device *ddev = get_drm_device();

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}

	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc || !mtk_crtc->panel_ext || !mtk_crtc->panel_ext->params) {
		pr_err("falied to get lcd proc info\n");
		return -EINVAL;
	}

	memcpy(p_info->version, mtk_crtc->panel_ext->params->vendor,
               sizeof(mtk_crtc->panel_ext->params->vendor) > 31?31:(sizeof(mtk_crtc->panel_ext->params->vendor)+1));
	memcpy(p_info->manufacture, mtk_crtc->panel_ext->params->manufacture,
               sizeof(mtk_crtc->panel_ext->params->vendor) > 31?31:(sizeof(mtk_crtc->panel_ext->params->vendor)+1));

	return 0;
}

int oplus_display_panel_get_seed(void *buf)
{
	unsigned int *seed = buf;

	printk("%s seed_mode=%ld\n", __func__, seed_mode);
	(*seed) = (unsigned int)seed_mode;

	return 0;
}

int oplus_display_panel_set_seed(void *buf)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *seed_mode_tmp = buf;

	printk("%s, %d to be %d\n", __func__, seed_mode, *seed_mode_tmp);

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}
	oplus_mtk_drm_setseed(crtc, *seed_mode_tmp);
	seed_mode = (*seed_mode_tmp);

	return 0;
}
