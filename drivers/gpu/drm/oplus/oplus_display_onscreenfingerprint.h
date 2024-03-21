/***************************************************************
** Copyright (C),  2021,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_onscreenfingerprint.h
** Description : oplus_display_onscreenfingerprint header
** Version : 1.0
** Date : 2021/12/10
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
** Zhongliuhe      2021/12/10        1.0          Modify for OFP
******************************************************************/
#ifndef _OPLUS_DISPLAY_ONSCREENFINGERPRINT_H_
#define _OPLUS_DISPLAY_ONSCREENFINGERPRINT_H_

/* please just only include linux common head file to keep me pure */
#include <linux/err.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <drm/drm_crtc.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/leds.h>
#include <linux/types.h>

enum oplus_ofp_log_level {
	OPLUS_OFP_LOG_LEVEL_ERR = 0,
	OPLUS_OFP_LOG_LEVEL_WARN = 1,
	OPLUS_OFP_LOG_LEVEL_INFO = 2,
	OPLUS_OFP_LOG_LEVEL_DEBUG = 3,
};

enum oplus_ofp_irq_type {
	OPLUS_OFP_TE_RDY = 0,
	OPLUS_OFP_FRAME_DONE = 1,
};

enum oplus_ofp_pressed_icon_status {
	OPLUS_OFP_PRESSED_ICON_OFF_FRAME_DONE = 0,		/* pressed icon has not been flush to DDIC ram */
	OPLUS_OFP_PRESSED_ICON_OFF = 1,					/* pressed icon has not been displayed in panel */
	OPLUS_OFP_PRESSED_ICON_ON_FRAME_DONE = 2,		/* pressed icon has been flush to DDIC ram */
	OPLUS_OFP_PRESSED_ICON_ON = 3,					/* pressed icon has been displayed in panel */
};

typedef struct fp_underscreen_info {
	uint8_t touch_state;
	uint8_t area_rate;
	uint16_t x;
	uint16_t y;
}fp_underscreen_info_t;

/* remember to initialize params */
struct oplus_ofp_params {
	bool ofp_support;								/* whether ofp is support or not */
	bool aod_unlocking;								/* whether is aod unlocking or not */
	bool aod_state;									/* whether panel is aod state or not */
	bool touch_state;									/* whether is touch state or not */
	int doze_active;								/* DOZE_ACTIVE property value */
	unsigned int aod_light_mode;					/* 0:50nit, 1:10nit */
	unsigned int aod_off_hbm_on_delay;				/* do some frame delay to keep apart aod off cmd and hbm on cmd */
	ktime_t aod_off_cmd_timestamp;					/* record aod off cmd timestamp for aod off hbm on delay judgment */
	int hbm_enable;									/* HBM_ENABLE property value */
	bool hbm_state;									/* whether panel is hbm state or not */
	unsigned int hbm_mode;							/* value of a node used for fingerprint calibration */
	bool fp_press;									/* whether pressed icon layer is ready or not */
	int pressed_icon_status;						/* indicate that pressed icon has been displayed in panel or not */
	int notifier_chain_value;						/* MTK_ONSCREENFINGERPRINT_EVENT notifier chain value */
	bool timing_switching;							/* indicates whether timing is switching in this frame or not */
	struct hrtimer timer;							/* add for uiready notifier call chain */
	struct workqueue_struct *aod_off_set_wq;		/* send aod off cmd workqueue */
	struct work_struct aod_off_set_work;			/* use to send aod off cmd to speed up aod unlocking */
};

/* log level config */
extern int oplus_ofp_log_level;
extern bool g_mobile_log;
extern bool g_detail_log;

/* just use for ofp */
#define OFP_ERR(fmt, arg...)	\
	do {	\
		if (oplus_ofp_log_level >= OPLUS_OFP_LOG_LEVEL_ERR)	\
			pr_err("[OFP][ERR][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define OFP_WARN(fmt, arg...)	\
	do {	\
		if (oplus_ofp_log_level >= OPLUS_OFP_LOG_LEVEL_WARN)	\
			pr_warn("[OFP][WARN][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define OFP_INFO(fmt, arg...)	\
	do {	\
		if ((oplus_ofp_log_level >= OPLUS_OFP_LOG_LEVEL_INFO) || (g_mobile_log))	\
			pr_info("[OFP][INFO][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define OFP_DEBUG(fmt, arg...)	\
	do {	\
		if ((oplus_ofp_log_level >= OPLUS_OFP_LOG_LEVEL_DEBUG) || (g_detail_log))	\
			pr_info("[OFP][DEBUG][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

extern int g_commit_pid;
extern struct drm_device *get_drm_device(void);
extern int mtkfb_set_aod_backlight_level(unsigned int level);
extern void mtk_drm_crtc_wk_lock(struct drm_crtc *crtc, bool get, const char *func, int line);

#ifndef CONFIG_OPLUS_OFP_V2
int oplus_display_panel_set_finger_print(void *buf);
int notify_display_fpd(bool mode);
#endif
/* -------------------- oplus_ofp_params -------------------- */
inline bool oplus_ofp_is_support(void);
int oplus_ofp_init(void *mtk_drm_private);
int oplus_ofp_get_aod_unlocking_state(void);
int oplus_ofp_get_touch_state(void);
int oplus_ofp_get_aod_state(void);
int oplus_ofp_set_aod_state(bool aod_state);
int oplus_ofp_get_hbm_state(void);
int oplus_ofp_set_hbm_state(bool hbm_state);
int oplus_ofp_property_update(int prop_id, unsigned int prop_val);

/* -------------------- fod -------------------- */
int oplus_ofp_hbm_handle(void *drm_crtc, void *mtk_crtc_state, void *cmdq_pkt);
int oplus_ofp_pressed_icon_status_update(int irq_type);
enum hrtimer_restart oplus_ofp_notify_uiready_timer_handler(struct hrtimer *timer);
int oplus_ofp_notify_uiready(void *mtk_drm_crtc);
bool oplus_ofp_backlight_filter(int bl_level);

/* -------------------- aod -------------------- */
int oplus_ofp_aod_off_status_handle(void *mtk_drm_crtc);
int oplus_ofp_doze_status_handle(bool doze_enable, void *drm_crtc, void *mtk_panel_ext, void *drm_panel, void *mtk_dsi, void *dcs_write_gce);
int oplus_ofp_set_aod_light_mode_after_doze_enable(void *mtk_panel_ext, void *mtk_dsi, void *dcs_write_gce);
void oplus_ofp_aod_off_set_work_handler(struct work_struct *work_item);
int oplus_ofp_touchpanel_event_notifier_call(struct notifier_block *nb, unsigned long action, void *data);
int oplus_ofp_aod_off_hbm_on_delay_check(void *mtk_drm_crtc);

/* -------------------- node -------------------- */
/* fod part */
int oplus_ofp_notify_fp_press(void *buf);
ssize_t oplus_ofp_notify_fp_press_attr(struct kobject *obj, struct kobj_attribute *attr,
	const char *buf, size_t count);
int oplus_ofp_get_hbm(void *buf);
int oplus_ofp_set_hbm(void *buf);
ssize_t oplus_ofp_get_hbm_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
ssize_t oplus_ofp_set_hbm_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);

/* aod part */
int oplus_ofp_set_aod_light_mode(void *buf);
int oplus_ofp_get_aod_light_mode(void *buf);
ssize_t oplus_ofp_set_aod_light_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_ofp_get_aod_light_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);

#endif /*_OPLUS_DISPLAY_ONSCREENFINGERPRINT_H_*/

