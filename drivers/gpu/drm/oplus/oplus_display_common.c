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
#ifdef CONFIG_OPLUS_OFP_V2
/* add for ofp */
#include "oplus_display_onscreenfingerprint.h"
#endif

#define PANEL_SERIAL_NUM_REG 0xA1
#define PANEL_REG_READ_LEN   10
#define BOE_PANEL_SERIAL_NUM_REG 0xA3
#define PANEL_SERIAL_NUM_REG_TIANMA 0xD6
#define POWER_MODE_OFF 0
#define SILKY_MAX_NORMAL_BRIGHTNESS 8191
extern unsigned int hbm_mode;
extern unsigned long cabc_mode;
extern unsigned long cabc_true_mode;
extern unsigned long cabc_sun_flag;
extern unsigned long cabc_back_flag;
extern void disp_aal_set_dre_en(int enable);
extern unsigned long silence_mode;
extern int oplus_max_brightness;
extern unsigned long oplus_display_brightness;
extern unsigned long oplus_max_normal_brightness;
extern uint64_t serial_number;
extern unsigned long esd_mode;
extern unsigned long seed_mode;
extern unsigned long aod_light_mode;
extern bool oplus_fp_notify_down_delay;
extern bool oplus_fp_notify_up_delay;
extern int power_mode;
extern unsigned int m_da;
extern unsigned int m_db;
extern unsigned int m_dc;

char oplus_rx_reg[PANEL_TX_MAX_BUF] = {0x0};
char oplus_rx_len = 0;

enum {
	REG_WRITE = 0,
	REG_READ,
	REG_X,
};

extern struct drm_device* get_drm_device(void);
extern int mtk_drm_setbacklight(struct drm_crtc *crtc, unsigned int level);
extern int oplus_mtk_drm_sethbm(struct drm_crtc *crtc, unsigned int hbm_mode);
extern int panel_serial_number_read(char cmd, int num);
extern int oplus_mtk_drm_setcabc(struct drm_crtc *crtc, unsigned int hbm_mode);
extern int oplus_mtk_drm_setseed(struct drm_crtc *crtc, unsigned int seed_mode);
extern int mtkfb_set_aod_backlight_level(unsigned int level);
extern bool oplus_mtk_drm_get_hbm_state(void);
extern int dsi_display_read_panel_reg(char cmd, char *data, size_t len);
extern void ddic_dsi_send_cmd(unsigned int cmd_num, char val[20]);

enum {
	CABC_LEVEL_0,
	CABC_LEVEL_1,
	CABC_LEVEL_2 = 3,
	CABC_EXIT_SPECIAL = 8,
	CABC_ENTER_SPECIAL = 9,
	GLOBAL_DRE_OPEN = 10,
	GLOBAL_DRE_CLOSE = 11,
};

int oplus_display_set_brightness(void *buf)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *set_brightness = buf;
	unsigned int oplus_set_brightness = (*set_brightness);

	printk("%s %d\n", __func__, oplus_set_brightness);

	if (oplus_set_brightness > oplus_max_brightness) {
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
	if (m_new_pq_persist_property[DISP_PQ_CCORR_SILKY_BRIGHTNESS]) {
		(*brightness) = SILKY_MAX_NORMAL_BRIGHTNESS;
	} else {
		(*brightness) = oplus_max_normal_brightness;
	}

	return 0;
}

#ifndef CONFIG_OPLUS_OFP_V2
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
#endif

int oplus_display_panel_get_panel_bpp(void *buf)
{
	unsigned int *panel_bpp = buf;
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
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

	(*panel_bpp) = mtk_crtc->panel_ext->params->panel_bpp;
	printk("%s panel_bpp : %d\n", __func__, *panel_bpp);

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

int oplus_display_get_cabc_status(void *buf)
{
	unsigned int *c_mode = buf;

	printk("%s CABC_mode=%ld\n", __func__, cabc_true_mode);
	*c_mode = (unsigned int)cabc_true_mode;

	return 0;
}

int oplus_display_set_cabc_status(void *buf)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	struct mtk_drm_crtc *mtk_crtc;
	unsigned int *c_mode = buf;
	cabc_mode = (unsigned long)(*c_mode);

	cabc_true_mode = cabc_mode;
	if (!ddev) {
		printk("set cabc status ddev fail\n");
		return 0;
	}
	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (IS_ERR_OR_NULL(crtc)) {
		printk("find crtc fail\n");
		return -1;
	}
	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc || !mtk_crtc->panel_ext || !mtk_crtc->panel_ext->params) {
		pr_err("falied to get lcd proc info\n");
		return -EINVAL;
	}
	printk("%s,cabc mode is %d, cabc_back_flag is %d,oplus_display_global_dre = %d\n", __func__,
		cabc_mode, cabc_back_flag, mtk_crtc->panel_ext->params->oplus_display_global_dre);
	if (cabc_mode < 4) {
		cabc_back_flag = cabc_mode;
	}

	if (mtk_crtc->panel_ext->params->oplus_display_global_dre) {
		if (cabc_mode == GLOBAL_DRE_OPEN) {
			disp_aal_set_dre_en(1);
			printk("%s adb cmd enable dre\n", __func__);
			return 0;
		} else if (cabc_mode == GLOBAL_DRE_CLOSE) {
			disp_aal_set_dre_en(0);
			printk("%s adb cmd disable dre\n", __func__);
			return 0;
		}
	}

	if (cabc_mode == CABC_ENTER_SPECIAL) {
		cabc_sun_flag = 1;
		cabc_true_mode = 0;
	} else if (cabc_mode == CABC_EXIT_SPECIAL) {
		cabc_sun_flag = 0;
		cabc_true_mode = cabc_back_flag;
	} else if (cabc_sun_flag == 1) {
		if (cabc_back_flag == CABC_LEVEL_0 || mtk_crtc->panel_ext->params->oplus_display_global_dre) {
			if (mtk_crtc->panel_ext->params->backlight_dsiable_threhold) {
				if (oplus_display_brightness < mtk_crtc->panel_ext->params->backlight_dsiable_threhold) {
					disp_aal_set_dre_en(0);
					printk("%s sun disable dre\n", __func__);
				} else {
					disp_aal_set_dre_en(1);
					printk("%s sun enable dre\n", __func__);
				}
			} else {
				disp_aal_set_dre_en(1);
				printk("%s sun enable dre\n", __func__);
			}
		} else {
			disp_aal_set_dre_en(0);
			printk("%s sun disable dre\n", __func__);
		}
		return 0;
	}

	printk("%s,cabc mode is %d\n", __func__, cabc_true_mode);

	if ((cabc_true_mode == CABC_LEVEL_0 && cabc_back_flag == CABC_LEVEL_0) || mtk_crtc->panel_ext->params->oplus_display_global_dre) {
		if (mtk_crtc->panel_ext->params->backlight_dsiable_threhold) {
			if (oplus_display_brightness < mtk_crtc->panel_ext->params->backlight_dsiable_threhold) {
				disp_aal_set_dre_en(0);
				printk("%s disable dre\n", __func__);
			} else {
				disp_aal_set_dre_en(1);
				printk("%s enable dre\n", __func__);
			}
		} else {
			disp_aal_set_dre_en(1);
			printk("%s sun enable dre\n", __func__);
		}
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

int oplus_big_endian_copy(void *dest, void *src, int count)
{
	int index = 0, knum = 0, rc = 0;
	uint32_t *u_dest = (uint32_t*) dest;
	char *u_src = (char*) src;

	if (dest == NULL || src == NULL) {
		printk("%s null pointer\n", __func__);
		return -EINVAL;
	}

	if (dest == src) {
		return rc;
	}

	while (count > 0) {
		u_dest[index] = ((u_src[knum] << 24) | (u_src[knum+1] << 16) | (u_src[knum+2] << 8) | u_src[knum+3]);
		index += 1;
		knum += 4;
		count = count - 1;
	}

	return rc;
}

int oplus_display_panel_get_reg(void *data)
{
	struct panel_reg_get *panel_reg = data;
	uint32_t u32_bytes = sizeof(uint32_t)/sizeof(char);

	u32_bytes = oplus_rx_len%u32_bytes ? (oplus_rx_len/u32_bytes + 1) : oplus_rx_len/u32_bytes;
	oplus_big_endian_copy(panel_reg->reg_rw, oplus_rx_reg, u32_bytes);
	panel_reg->lens = oplus_rx_len;

	return 0;
}

int oplus_display_panel_set_reg(void *data)
{
	char reg[PANEL_TX_MAX_BUF] = {0x0};
	char payload[PANEL_TX_MAX_BUF] = {0x0};
	u32 value = 0;
	int len = 0, index = 0;
	struct panel_reg_rw *reg_rw = data;

	if (reg_rw->lens > PANEL_REG_MAX_LENS) {
		pr_err("error: wrong input reg len\n");
		return -EINVAL;
	}

	/*begin read*/
	if (reg_rw->rw_flags == REG_READ) {
		value = reg_rw->cmd;
		len = reg_rw->lens;

		dsi_display_read_panel_reg(value, reg, len);

		for (index = 0; index < len; index++) {
			printk("%x ", reg[index]);
		}

		memcpy(oplus_rx_reg, reg, PANEL_TX_MAX_BUF);
		oplus_rx_len = len;

		return 0;
	}
	/*end read*/

	/*begin write*/
	if (reg_rw->rw_flags == REG_WRITE) {
		memcpy(payload, reg_rw->value, reg_rw->lens);
		reg[0] = reg_rw->cmd;
		len = reg_rw->lens;
		for (index=0; index < len; index++) {
			reg[index + 1] = payload[index];
		}
		ddic_dsi_send_cmd(len, reg);

		return 0;
	}
	/*end write*/
	printk("%s error: please check the args!\n", __func__);
	return -1;
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

int oplus_display_panel_get_id(void *buf)
{
	struct panel_id *panel_rid = buf;

	pr_err("%s: 0xDA= 0x%x, 0xDB=0x%x, 0xDC=0x%x\n", __func__, m_da, m_db, m_dc);

	panel_rid->DA = (uint32_t)m_da;
	panel_rid->DB = (uint32_t)m_db;
	panel_rid->DC = (uint32_t)m_dc;

	return 0;
}

#ifndef CONFIG_OPLUS_OFP_V2
int oplus_panel_get_aod_light_mode(void *buf)
{
	unsigned int *aod_lm = buf;

	printk("%s aod_light_mode=%ld\n", __func__, aod_light_mode);
	(*aod_lm) = (unsigned int)aod_light_mode;

	return 0;
}

int oplus_panel_set_aod_light_mode(void *buf)
{
	unsigned int *aod_lm = buf;

	printk("%s, %d to be %d\n", __func__, aod_light_mode, *aod_lm);
#if defined(CONFIG_MACH_MT6853)
	if (power_mode == POWER_MODE_OFF) {
		printk(KERN_INFO "oplus_set_aod_light_mode = %d return on power_mode:%d\n", *aod_lm, power_mode);
		return 0;
	}
#endif

	if (oplus_mtk_drm_get_hbm_state()) {
		printk(KERN_INFO "oplus_set_aod_light_mode = %d return on hbm\n", *aod_lm);
		return 0;
	}

	if (*aod_lm != aod_light_mode) {
		printk(KERN_INFO " set aod backlight to %s nit\n", (*aod_lm == 0)?"50":"10");
		mtkfb_set_aod_backlight_level(*aod_lm);
		aod_light_mode = (*aod_lm);
	}

	return 0;
}

int oplus_display_panel_notify_fp_press(void *buf)
{
	unsigned int fingerprint_op_mode = 0x0;

	fingerprint_op_mode = *(unsigned int*)buf;
	printk(KERN_ERR "%s receive uiready %d\n", __func__, fingerprint_op_mode);

	if (fingerprint_op_mode == 1) {
		oplus_fp_notify_down_delay = true;
	} else {
		oplus_fp_notify_up_delay = true;
	}

	return 0;
}
#endif

extern unsigned char aod_area_set_flag;
extern struct aod_area oplus_aod_area[RAMLESS_AOD_AREA_NUM];
extern char send_cmd[RAMLESS_AOD_PAYLOAD_SIZE];
int oplus_display_set_aod_area(void *buf)
{
	struct panel_aod_area_para *para = (struct panel_aod_area_para *)buf;
	int i, cnt = 0;
	char payload[RAMLESS_AOD_PAYLOAD_SIZE];

	memset(oplus_aod_area, 0, sizeof(struct aod_area) * RAMLESS_AOD_AREA_NUM);

	if (para->size > RAMLESS_AOD_AREA_NUM) {
		pr_err("aod area size is invalid, size=%d\n", para->size);
		return -1;
	}

	pr_info("%s %d\n", __func__, __LINE__);
	for (i = 0; i < para->size; i++) {
		struct aod_area *area = &oplus_aod_area[cnt];

		area->x = para->aod_area[i].x;
		area->y = para->aod_area[i].y;
		area->w = para->aod_area[i].w;
		area->h = para->aod_area[i].h;
		area->color = para->aod_area[i].color;
		area->bitdepth = para->aod_area[i].bitdepth;
		area->mono = para->aod_area[i].mono;
		area->gray = para->aod_area[i].gray;
		pr_info("%s %d rect[%dx%d-%dx%d]-%d-%d-%d-%x\n", __func__, __LINE__,
			area->x, area->y, area->w, area->h,
			area->color, area->bitdepth, area->mono, area->gray);
		area->enable = true;
		cnt++;
	}

	memset(payload, 0, RAMLESS_AOD_PAYLOAD_SIZE);
	memset(send_cmd, 0, RAMLESS_AOD_PAYLOAD_SIZE);

	for (i = 0; i < RAMLESS_AOD_AREA_NUM; i++) {
		struct aod_area *area = &oplus_aod_area[i];

		payload[0] |= (!!area->enable) << (RAMLESS_AOD_AREA_NUM - i - 1);
		if (area->enable) {
			int h_start = area->x;
			int h_block = area->w / 100;
			int v_start = area->y;
			int v_end = area->y + area->h;
			int off = i * 5;

			/* Rect Setting */
			payload[1 + off] = h_start >> 4;
			payload[2 + off] = ((h_start & 0xf) << 4) | (h_block & 0xf);
			payload[3 + off] = v_start >> 4;
			payload[4 + off] = ((v_start & 0xf) << 4) | ((v_end >> 8) & 0xf);
			payload[5 + off] = v_end & 0xff;

			/* Mono Setting */
			#define SET_MONO_SEL(index, shift) \
				if (i == index) \
					payload[31] |= area->mono << shift;

			SET_MONO_SEL(0, 6);
			SET_MONO_SEL(1, 5);
			SET_MONO_SEL(2, 4);
			SET_MONO_SEL(3, 2);
			SET_MONO_SEL(4, 1);
			SET_MONO_SEL(5, 0);
			#undef SET_MONO_SEL

			/* Depth Setting */
			if (i < 4)
				payload[32] |= (area->bitdepth & 0x3) << ((3 - i) * 2);
			else if (i == 4)
				payload[33] |= (area->bitdepth & 0x3) << 6;
			else if (i == 5)
				payload[33] |= (area->bitdepth & 0x3) << 4;
			/* Color Setting */
			#define SET_COLOR_SEL(index, reg, shift) \
				if (i == index) \
					payload[reg] |= (area->color & 0x7) << shift;
			SET_COLOR_SEL(0, 34, 4);
			SET_COLOR_SEL(1, 34, 0);
			SET_COLOR_SEL(2, 35, 4);
			SET_COLOR_SEL(3, 35, 0);
			SET_COLOR_SEL(4, 36, 4);
			SET_COLOR_SEL(5, 36, 0);
			#undef SET_COLOR_SEL
			/* Area Gray Setting */
			payload[37 + i] = area->gray & 0xff;
		}
	}
	payload[43] = 0x00;
	send_cmd[0] = 0x81;
	for (i = 0; i < 44; i++) {
		pr_info("payload[%d] = 0x%x- send_cmd[%d] = 0x%x-", i, payload[i], i, send_cmd[i]);
		send_cmd[i+1] = payload[i];
	}
	aod_area_set_flag = 1;
	return 0;
}

unsigned int oplus_get_ssc_config_data(void)
{
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int deltal = 2; /* Delta1 is SSC range, default is 0%~-5% */
	unsigned int deltal_div = 1;
	unsigned int pdeltal = 0;
	unsigned int data_rate;
	unsigned int pcw_ratio = 0;

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		pr_err("%s find crtc fail\n", __func__);
		return 0;
	}

	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc || !mtk_crtc->panel_ext || !mtk_crtc->panel_ext->params) {
		pr_err("%s falied to get panel parmas info\n", __func__);
		return 0;
	}

	if (mtk_crtc->panel_ext->params->ssc_enable != 1) {
		pr_debug("%s no ssc config\n", __func__);
		return 0;
	}

	data_rate = mtk_crtc->panel_ext->params->data_rate;

	if (data_rate > 2500) {
		pr_info("mipitx Data Rate exceed limitation(%d)\n", data_rate);
		return 0;
	} else if (data_rate >= 2000) { /* 2G ~ 2.5G */
		pcw_ratio = 1;
	} else if (data_rate >= 1000) { /* 1G ~ 2G */
		pcw_ratio = 2;
	} else if (data_rate >= 500) { /* 500M ~ 1G */
		pcw_ratio = 4;
	} else if (data_rate > 250) { /* 250M ~ 500M */
		pcw_ratio = 8;
	} else if (data_rate >= 125) { /* 125M ~ 250M */
		pcw_ratio = 16;
	} else {
		pr_info("dataRate is too low(%d)\n", data_rate);
		return 0;
	}

	/* SSC config */
	deltal = (mtk_crtc->panel_ext->params->ssc_range == 0) ?
		deltal : mtk_crtc->panel_ext->params->ssc_range;
	deltal_div = (mtk_crtc->panel_ext->params->ssc_range_div == 0) ?
		deltal_div : mtk_crtc->panel_ext->params->ssc_range_div;
	pdeltal = ((deltal * (data_rate / 2) * pcw_ratio *
			262144)/deltal_div + 281664) / 563329;

	pr_info("%s, deltal:%d, delta1_div:%d, data_rate:%d, pcw_ratio:%d, pdeltal:%x\n",
			__func__, deltal, deltal_div, data_rate, pcw_ratio, pdeltal);

	return (pdeltal << 16 | pdeltal);
}
