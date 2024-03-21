/***************************************************************
** Copyright (C),  2021,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_onscreenfingerprint.c
** Description : oplus_display_onscreenfingerprint implement
** Version : 1.0
** Date : 2021/12/10
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
** Zhongliuhe      2021/12/10        1.0          Modify for OFP
******************************************************************/

#include "oplus_display_onscreenfingerprint.h"
#include "mtk_drm_drv.h"
#include "mtk_drm_helper.h"
#include "mtk_drm_trace.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_mmp.h"
#include "mtk_log.h"
#include "mtk_dump.h"

#ifndef CONFIG_OPLUS_OFP_V2
extern bool oplus_fp_notify_down_delay;
extern bool oplus_fp_notify_up_delay;
bool oplus_doze_fpd_nodelay;

int oplus_display_panel_set_finger_print(void *buf)
{
	unsigned int *fingerprint_op_mode = buf;

	if ((*fingerprint_op_mode) == 1) {
		oplus_fp_notify_down_delay = true;
	} else {
		oplus_fp_notify_up_delay = true;
	}

	printk(KERN_ERR "%s receive uiready %d\n", __func__, (*fingerprint_op_mode));
	return 0;
}

int notify_display_fpd(bool mode) {
	pr_debug("lcm mode = %d\n", mode);
	oplus_doze_fpd_nodelay = mode;
	return 0;
}
#else
/* -------------------- parameters -------------------- */
int g_commit_pid = 0;
/* log level config */
int oplus_ofp_log_level = OPLUS_OFP_LOG_LEVEL_INFO;
EXPORT_SYMBOL(oplus_ofp_log_level);
/* ofp global structure */
static struct oplus_ofp_params g_oplus_ofp_params = {0};
/* extern para */
extern unsigned int oplus_display_brightness;

/* -------------------- oplus_ofp_params -------------------- */
static struct oplus_ofp_params *oplus_ofp_get_params(void)
{
	return &g_oplus_ofp_params;
}

inline bool oplus_ofp_is_support(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return 0;
	}

	return p_oplus_ofp_params->ofp_support;
}

int oplus_ofp_init(void *mtk_drm_private)
{
	struct mtk_drm_private *priv = mtk_drm_private;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!priv || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	p_oplus_ofp_params->ofp_support = mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_HBM);
	OFP_INFO("ofp support:%d\n", p_oplus_ofp_params->ofp_support);

	if (oplus_ofp_is_support()) {
		/* add for uiready notifier call chain */
		hrtimer_init(&p_oplus_ofp_params->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		p_oplus_ofp_params->timer.function = oplus_ofp_notify_uiready_timer_handler;

		/* add workqueue to send aod off cmd */
		p_oplus_ofp_params->aod_off_set_wq = create_singlethread_workqueue("aod_off_set");
		INIT_WORK(&p_oplus_ofp_params->aod_off_set_work, oplus_ofp_aod_off_set_work_handler);
	}

	return 0;
}

/* aod unlocking value update */
int oplus_ofp_aod_unlocking_update(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (p_oplus_ofp_params->fp_press || p_oplus_ofp_params->doze_active != 0) {
		/* press icon layer is ready in aod mode */
		p_oplus_ofp_params->aod_unlocking = true;
		OFP_INFO("oplus_ofp_aod_unlocking: %d\n", p_oplus_ofp_params->aod_unlocking);
		mtk_drm_trace_c("%d|oplus_ofp_aod_unlocking|%d", g_commit_pid, p_oplus_ofp_params->aod_unlocking);
	}

	return 0;
}

int oplus_ofp_get_aod_unlocking_state(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return 0;
	}

	return p_oplus_ofp_params->aod_unlocking;
}

int oplus_ofp_get_touch_state(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return 0;
	}

	return p_oplus_ofp_params->touch_state;
}

int oplus_ofp_get_aod_state(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return 0;
	}

	return p_oplus_ofp_params->aod_state;
}

int oplus_ofp_set_aod_state(bool aod_state)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	p_oplus_ofp_params->aod_state = aod_state;
	OFP_INFO("oplus_ofp_aod_state: %d\n", p_oplus_ofp_params->aod_state);
	mtk_drm_trace_c("%d|oplus_ofp_aod_state|%d", g_commit_pid, p_oplus_ofp_params->aod_state);

	return 0;
}

int oplus_ofp_get_hbm_state(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return 0;
	}

	return p_oplus_ofp_params->hbm_state;
}

int oplus_ofp_set_hbm_state(bool hbm_state)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	p_oplus_ofp_params->hbm_state = hbm_state;
	OFP_INFO("oplus_ofp_hbm_state: %d\n", hbm_state);
	mtk_drm_trace_c("%d|oplus_ofp_hbm_state|%d", g_commit_pid, hbm_state);

	return 0;
}

/* update doze_active and hbm_enable property value */
int oplus_ofp_property_update(int prop_id, unsigned int prop_val)
{
	static unsigned int last_disp_mode_idx = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	switch (prop_id) {
	case CRTC_PROP_DOZE_ACTIVE:
		if (prop_val != p_oplus_ofp_params->doze_active) {
			OFP_INFO("DOZE_ACTIVE:%d\n", prop_val);
		}
		p_oplus_ofp_params->doze_active = prop_val;
		mtk_drm_trace_c("%d|oplus_ofp_doze_active|%d", g_commit_pid, p_oplus_ofp_params->doze_active);
		break;

	case CRTC_PROP_HBM_ENABLE:
		if (prop_val != p_oplus_ofp_params->hbm_enable) {
			OFP_INFO("HBM_ENABLE:%d\n", prop_val);
		}
		p_oplus_ofp_params->hbm_enable = prop_val;
		mtk_drm_trace_c("%d|oplus_ofp_hbm_enable|%d", g_commit_pid, p_oplus_ofp_params->hbm_enable);
		break;
	case CRTC_PROP_DISP_MODE_IDX:
		if (prop_val != last_disp_mode_idx) {
			p_oplus_ofp_params->timing_switching = true;
			OFP_DEBUG("DISP_MODE_IDX:%u\n", prop_val);
		} else {
			p_oplus_ofp_params->timing_switching = false;
		}
		OFP_DEBUG("p_oplus_ofp_params->timing_switching:%d\n", p_oplus_ofp_params->timing_switching);
		mtk_drm_trace_c("%d|oplus_ofp_timing_switching|%d", g_commit_pid, p_oplus_ofp_params->timing_switching);
		last_disp_mode_idx = prop_val;
		break;

	default:
		break;
	}

	return 0;
}


/* -------------------- fod -------------------- */
/* wait te and delay while using cmdq */
static int oplus_ofp_cmdq_pkt_wait(struct mtk_drm_crtc *mtk_crtc, struct cmdq_pkt *cmdq_handle, int te_count, int delay_us)
{
	int wait_te_count = te_count;

	if ((wait_te_count <= 0) && (delay_us <= 0)) {
		return 0;
	}

	if (!mtk_crtc) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	mtk_drm_trace_begin("oplus_ofp_cmdq_pkt_wait");

	if (cmdq_handle == NULL) {
		cmdq_handle = cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
	}

	OFP_INFO("wait %d te and delay %dus\n", wait_te_count, delay_us);

	if (wait_te_count > 0) {
		cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);

		while (wait_te_count > 0) {
			OFP_DEBUG("start to wait EVENT_TE, remain %d te count\n", wait_te_count);
			cmdq_pkt_wfe(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
			OFP_DEBUG("complete the EVENT_TE waiting\n");
			wait_te_count--;
		}
	}

	if (delay_us > 0) {
		OFP_DEBUG("start to sleep %d us", delay_us);
		cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(delay_us), CMDQ_GPR_R12);
	}

	mtk_drm_trace_end();

	return 0;
}

static int oplus_ofp_hbm_wait_handle(struct drm_crtc *crtc, struct cmdq_pkt *cmdq_handle, bool before_hbm, bool hbm_en)
{
	unsigned int refresh_rate = 0;
	unsigned int us_per_frame = 0;
	unsigned int te_count = 0;
	unsigned int delay_us = 0;
	int ret = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();
	struct mtk_panel_params *panel_ext = mtk_drm_get_lcm_ext_params(crtc);
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle2;

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!panel_ext | !mtk_crtc) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (p_oplus_ofp_params->aod_unlocking && hbm_en && !p_oplus_ofp_params->aod_off_hbm_on_delay
			&& !panel_ext->oplus_ofp_need_to_sync_data_in_aod_unlocking) {
		OFP_DEBUG("no need to delay in aod unlocking\n");
		return 0;
	}
	mtk_drm_trace_begin("oplus_ofp_hbm_wait_handle");

	refresh_rate = panel_ext->dyn_fps.vact_timing_fps;
	us_per_frame = 1000000/refresh_rate;

	if (before_hbm) {
		if (hbm_en) {
			if (p_oplus_ofp_params->aod_unlocking == true) {
				if (panel_ext->oplus_ofp_aod_off_insert_black > 0) {
					bool need_aod_off_hbm_on_delay = (bool)p_oplus_ofp_params->aod_off_hbm_on_delay;

					if (panel_ext->oplus_ofp_aod_off_black_frame_total_time > 0) {
						if (ktime_sub(ktime_get(), p_oplus_ofp_params->aod_off_cmd_timestamp) >=
							ms_to_ktime(panel_ext->oplus_ofp_aod_off_black_frame_total_time)) {
							/* as te_rdy irq will be disabled in idle mode, so the aod_off_hbm_on_delay
							may not be accurate. then add oplus_ofp_aod_off_black_frame_total_time
							to check whether the timestamp of hbm cmd is in the black frames or not */
							OFP_DEBUG("no need to do some delay because it's not in the black frames\n");
							need_aod_off_hbm_on_delay = false;
						}
					}

					if (need_aod_off_hbm_on_delay) {
						/* do some frame delay to keep apart aod off cmd and hbm on cmd */
						te_count = p_oplus_ofp_params->aod_off_hbm_on_delay;
						delay_us = (us_per_frame >> 1) + 500;
						p_oplus_ofp_params->aod_off_hbm_on_delay = 0;
						OFP_INFO("wait %d te and %dus to keep apart aod off cmd and hbm on cmd\n", te_count, delay_us);
					} else if (panel_ext->oplus_ofp_need_to_sync_data_in_aod_unlocking) {
						if (p_oplus_ofp_params->timing_switching) {
							/*
							 when timing switch cmds and hbm on cmds are sent at the same frame,
							 ddic has not changed to the latest frame rate, but delay has been calculated
							 according to the latest frame rate which would cause flicker issue,
							 so add one te wait to fix it
							*/
							te_count = 2;
						} else {
							/* wait 1 te ,then send hbm on cmds in the second half of the frame */
							te_count = 1;
						}
						delay_us = (us_per_frame >> 1) + 500;
					}
				} else if (panel_ext->oplus_ofp_need_to_sync_data_in_aod_unlocking) {
					/* wait 1 te ,then send hbm on cmds in the second half of the frame */
					te_count = 1;
					delay_us = (us_per_frame >> 1) + 500;
				}
			} else {
				/* backlight will affect hbm on/off time in some panel, need to keep apart the 51 cmd for stable hbm time */
				if (panel_ext->oplus_ofp_need_keep_apart_backlight) {
					/* flush the blocking frame , otherwise the dim Layer would be delay to next frame so that flicker happen */
					mtk_drm_trace_begin("cmdq_handle2");
					cmdq_handle2 = cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
					cmdq_pkt_wait_no_clear(cmdq_handle2, mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
					/* delay one frame */
					cmdq_pkt_sleep(cmdq_handle2, CMDQ_US_TO_TICK(us_per_frame), CMDQ_GPR_R12);
					cmdq_pkt_flush(cmdq_handle2);
					cmdq_pkt_destroy(cmdq_handle2);
					mtk_drm_trace_end();

					/* send hbm on cmd in next frame */
					te_count = 1;
					delay_us = (us_per_frame >> 1) + 500;
				} else {
					te_count = 1;
					delay_us = (us_per_frame >> 1) + 500;
				}
			}
		} else {
			if (panel_ext->oplus_ofp_pre_hbm_off_delay) {
				te_count = 0;
				/* the delay time bfore hbm off */
				delay_us = panel_ext->oplus_ofp_pre_hbm_off_delay * 1000;

				/* flush the blocking frame , otherwise the dim Layer would be delay to next frame so that flicker happen */
				mtk_drm_trace_begin("cmdq_handle2");
				cmdq_handle2 = cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
				cmdq_pkt_wait_no_clear(cmdq_handle2, mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
				/* delay some time to wait for data */
				cmdq_pkt_sleep(cmdq_handle2, CMDQ_US_TO_TICK(delay_us), CMDQ_GPR_R12);
				cmdq_pkt_flush(cmdq_handle2);
				cmdq_pkt_destroy(cmdq_handle2);
				mtk_drm_trace_end();
			}
		}
	} else {
		if (hbm_en) {
			if (p_oplus_ofp_params->aod_unlocking == true) {
				OFP_DEBUG("no need to delay after hbm on cmds have been sent in aod unlocking\n");
			} else if (panel_ext->oplus_ofp_hbm_on_delay) {
				te_count = 0;
				/* the time when hbm on need */
				delay_us = panel_ext->oplus_ofp_hbm_on_delay * 1000;
			}
		} else {
			if (panel_ext->oplus_ofp_hbm_off_delay) {
				te_count = 0;
				/* the time when hbm off need */
				delay_us = panel_ext->oplus_ofp_hbm_off_delay * 1000;
			}
		}
	}

	OFP_INFO("before_hbm = %d, te_count = %d, delay_us = %d\n", before_hbm, te_count, delay_us);
	ret = oplus_ofp_cmdq_pkt_wait(mtk_crtc, cmdq_handle, te_count, delay_us);
	if (ret) {
		OFP_ERR("oplus_ofp_cmdq_pkt_wait failed\n");
	}

	mtk_drm_trace_end();

	return ret;
}

static int oplus_ofp_set_panel_hbm(struct drm_crtc *crtc, bool hbm_en)
{
	bool doze_en = false;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	struct cmdq_pkt *cmdq_handle;

	if (!p_oplus_ofp_params || !crtc || !mtk_crtc) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (oplus_ofp_get_hbm_state() == hbm_en) {
		OFP_DEBUG("already in hbm state %d\n", hbm_en);
		return 0;
	}

	if (!(comp && comp->funcs && comp->funcs->io_cmd)) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!(mtk_crtc->enabled)) {
		OFP_ERR("skip, slept\n");
		return -EINVAL;
	}

	mtk_drm_trace_begin("oplus_ofp_set_panel_hbm");

	mtk_drm_trace_begin("mtk_drm_send_lcm_cmd_prepare");
	OFP_INFO("prepare to send hbm cmd\n");
	mtk_drm_send_lcm_cmd_prepare(crtc, &cmdq_handle);
	mtk_drm_trace_end();

	if (oplus_ofp_get_aod_state()) {
		OFP_INFO("send aod off cmd before hbm on because panel is still in aod mode\n");
		oplus_ofp_aod_off_status_handle(mtk_crtc);

		mtk_drm_trace_begin("DSI_SET_DOZE");
		comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_DOZE, &doze_en);
		OFP_INFO("DSI_SET_DOZE %d\n", doze_en);
		mtk_drm_trace_end();
	}

	/* delay before hbm cmd */
	oplus_ofp_hbm_wait_handle(crtc, cmdq_handle, true, hbm_en);

	/* send hbm cmd */
	mtk_drm_trace_begin("DSI_HBM_SET");
	comp->funcs->io_cmd(comp, cmdq_handle, DSI_HBM_SET, &hbm_en);
	OFP_INFO("DSI_HBM_SET %d\n", hbm_en);
	mtk_drm_trace_end();

	/* delay after hbm cmd */
	oplus_ofp_hbm_wait_handle(crtc, cmdq_handle, false, hbm_en);

	mtk_drm_trace_begin("mtk_drm_send_lcm_cmd_flush");
	mtk_drm_send_lcm_cmd_flush(crtc, &cmdq_handle, true);
	oplus_ofp_set_hbm_state(hbm_en);
	mtk_drm_trace_end();

	if (hbm_en == false && p_oplus_ofp_params->aod_unlocking == true) {
		p_oplus_ofp_params->aod_unlocking = false;
		OFP_INFO("oplus_ofp_aod_unlocking: %d\n", p_oplus_ofp_params->aod_unlocking);
		mtk_drm_trace_c("%d|oplus_ofp_aod_unlocking|%d", g_commit_pid, p_oplus_ofp_params->aod_unlocking);
	}

	OFP_INFO("hbm cmd is flushed\n");
	mtk_drm_trace_end();

	return 0;
}

int oplus_ofp_hbm_handle(void *drm_crtc, void *mtk_crtc_state, void *cmdq_pkt)
{
	int hbm_en = 0;
	struct drm_crtc *crtc = drm_crtc;
	struct mtk_crtc_state *state = mtk_crtc_state;
	struct cmdq_pkt *cmdq_handle = cmdq_pkt;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!crtc || !state || !cmdq_handle || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (p_oplus_ofp_params->hbm_mode) {
		OFP_DEBUG("already in hbm mode %u\n", p_oplus_ofp_params->hbm_mode);
		return -EFAULT;
	}

	mtk_drm_trace_begin("oplus_ofp_hbm_handle");

	hbm_en = state->prop_val[CRTC_PROP_HBM_ENABLE];
	OFP_DEBUG("hbm_en = %d\n", hbm_en);

	if (mtk_crtc_is_frame_trigger_mode(crtc)) {
		if ((!state->prop_val[CRTC_PROP_DOZE_ACTIVE] && hbm_en > 0 && oplus_display_brightness != 0)
			|| (state->prop_val[CRTC_PROP_DOZE_ACTIVE] && hbm_en > 1 && oplus_display_brightness != 0)) {
			OFP_DEBUG("set hbm on\n");
			oplus_ofp_set_panel_hbm(crtc, true);

			/*bypass pq when enter hbm */
			mtk_atomic_hbm_bypass_pq(crtc, cmdq_handle, 1);
			OFP_DEBUG("bypass pq in hbm mode\n");
		} else if (hbm_en == 0 || p_oplus_ofp_params->fp_press == false
			|| oplus_display_brightness == 0) {
			OFP_DEBUG("set hbm off\n");
			oplus_ofp_set_panel_hbm(crtc, false);

			/*no need to bypass pq when exit hbm */
			mtk_atomic_hbm_bypass_pq(crtc, cmdq_handle, 0);
			OFP_DEBUG("no need to bypass pq in normal mode\n");
		}
	}

	mtk_drm_trace_end();

	return 0;
}

/* update pressed icon status */
int oplus_ofp_pressed_icon_status_update(int irq_type)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();
	static int last_hbm_enable = 0;

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	mtk_drm_trace_begin("oplus_ofp_pressed_icon_status_update");
	if (irq_type == OPLUS_OFP_FRAME_DONE) {
		if (((last_hbm_enable & 0x2) == 0) && ((p_oplus_ofp_params->hbm_enable & 0x2) > 0)) {
			/* pressed icon has been flush to DDIC ram */
			p_oplus_ofp_params->pressed_icon_status = OPLUS_OFP_PRESSED_ICON_ON_FRAME_DONE;
			OFP_INFO("pressed icon status: OPLUS_OFP_PRESSED_ICON_ON_FRAME_DONE\n");
		} else if (((last_hbm_enable & 0x2) > 0) && ((p_oplus_ofp_params->hbm_enable & 0x2) == 0)) {
			/* pressed icon has not been flush to DDIC ram */
			p_oplus_ofp_params->pressed_icon_status = OPLUS_OFP_PRESSED_ICON_OFF_FRAME_DONE;
			OFP_INFO("pressed icon status: OPLUS_OFP_PRESSED_ICON_OFF_FRAME_DONE\n");
		}
		last_hbm_enable = p_oplus_ofp_params->hbm_enable;
	} else if (irq_type == OPLUS_OFP_TE_RDY) {
		if (p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_ON_FRAME_DONE) {
			/* pressed icon has been displayed in panel */
			p_oplus_ofp_params->pressed_icon_status = OPLUS_OFP_PRESSED_ICON_ON;
			OFP_INFO("pressed icon status: OPLUS_OFP_PRESSED_ICON_ON\n");
		} else if (p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_OFF_FRAME_DONE) {
			/* pressed icon has not been displayed in panel */
			p_oplus_ofp_params->pressed_icon_status = OPLUS_OFP_PRESSED_ICON_OFF;
			OFP_INFO("pressed icon status: OPLUS_OFP_PRESSED_ICON_OFF\n");
		}
	}
	mtk_drm_trace_c("%d|oplus_ofp_pressed_icon_status|%d", g_commit_pid, p_oplus_ofp_params->pressed_icon_status);
	mtk_drm_trace_end();

	return 0;
}

/* timer */
enum hrtimer_restart oplus_ofp_notify_uiready_timer_handler(struct hrtimer *timer)
{
	struct fb_event event;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	event.info  = NULL;
	event.data = &p_oplus_ofp_params->notifier_chain_value;

	mtk_drm_trace_begin("oplus_ofp_notify_uiready_timer_handler");
	/* notifer fingerprint that pressed icon ui is ready */
	OFP_INFO("send uiready: %d\n", p_oplus_ofp_params->notifier_chain_value);
	fb_notifier_call_chain(MTK_ONSCREENFINGERPRINT_EVENT, &event);
	mtk_drm_trace_c("%d|oplus_ofp_notifier_chain_value|%d", g_commit_pid, p_oplus_ofp_params->notifier_chain_value);
	mtk_drm_trace_end();

	return HRTIMER_NORESTART;
}

/* notify uiready */
int oplus_ofp_notify_uiready(void *mtk_drm_crtc)
{
	static int last_notifier_chain_value = 0;
	static int last_hbm_state = false;
	static ktime_t hbm_cmd_timestamp = 0;
	ktime_t delta_time = 0;
	unsigned int refresh_rate = 0;
	unsigned int delay_ms = 0;
	struct fb_event event;
	struct mtk_drm_crtc *mtk_crtc = mtk_drm_crtc;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!mtk_crtc || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (p_oplus_ofp_params->aod_unlocking == true) {
		if (last_hbm_state == false && (oplus_ofp_get_hbm_state() == true)) {
			/* hbm cmd is sent to ddic */
			hbm_cmd_timestamp = ktime_get();
			OFP_INFO("hbm_cmd_timestamp:%lu\n", ktime_to_ms(hbm_cmd_timestamp));
		}

		if ((p_oplus_ofp_params->fp_press == true) && (p_oplus_ofp_params->hbm_enable & 0x2) > 0
			&& (oplus_ofp_get_hbm_state() == true) && (p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_ON)) {
			/* pressed icon has been displayed in panel but hbm may not take effect */
			p_oplus_ofp_params->notifier_chain_value = 1;
		} else if (p_oplus_ofp_params->fp_press == false && (p_oplus_ofp_params->hbm_enable & 0x2) == 0) {
			/*  finger is not pressed down */
			p_oplus_ofp_params->notifier_chain_value = 0;
		}
	} else {
		if ((p_oplus_ofp_params->fp_press == true) && (p_oplus_ofp_params->hbm_enable & 0x2) > 0
			&& (oplus_ofp_get_hbm_state() == true) && (p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_ON)) {
			/* pressed icon has been displayed in panel and hbm is always in effect */
			p_oplus_ofp_params->notifier_chain_value = 1;
		} else if (p_oplus_ofp_params->fp_press == false && (p_oplus_ofp_params->hbm_enable & 0x2) == 0
					&& p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_OFF) {
			/* hbm is on but pressed icon has not been displayed */
			p_oplus_ofp_params->notifier_chain_value = 0;
		}
	}
	last_hbm_state = oplus_ofp_get_hbm_state();

	if (last_notifier_chain_value != p_oplus_ofp_params->notifier_chain_value) {
		mtk_drm_trace_begin("oplus_ofp_notify_uiready");
		if (p_oplus_ofp_params->aod_unlocking == true && p_oplus_ofp_params->notifier_chain_value == 1) {
			/* check whether hbm is taking effect or not */
			if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params) {
				refresh_rate = mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps;
				if (mtk_crtc->panel_ext->params->oplus_uiready_before_time)
					delay_ms = mtk_crtc->panel_ext->params->oplus_uiready_before_time + 1000/refresh_rate;
				else
					delay_ms = mtk_crtc->panel_ext->params->oplus_ofp_hbm_on_delay + 1000/refresh_rate;
			} else {
				delay_ms = 17;
			}

			delta_time = ktime_sub(ktime_get(), hbm_cmd_timestamp);
			if (ktime_sub(ms_to_ktime(delay_ms), delta_time) > 0) {
				/* hbm is not taking effect, set a timer to wait and then send uiready */
				hrtimer_start(&p_oplus_ofp_params->timer, ktime_sub(ms_to_ktime(delay_ms), delta_time), HRTIMER_MODE_REL);
				OFP_INFO("delay %luus to notify\n", ktime_to_us(ktime_sub(ms_to_ktime(delay_ms), delta_time)));
			} else {
				/* hbm is taking effect, so send uiready immediately */
				event.info  = NULL;
				event.data = &p_oplus_ofp_params->notifier_chain_value;
				OFP_INFO("send uiready: %d\n", p_oplus_ofp_params->notifier_chain_value);
				fb_notifier_call_chain(MTK_ONSCREENFINGERPRINT_EVENT, &event);
				mtk_drm_trace_c("%d|oplus_ofp_notifier_chain_value|%d", g_commit_pid, p_oplus_ofp_params->notifier_chain_value);
			}
		} else {
			/* send uiready immediately */
			event.info  = NULL;
			event.data = &p_oplus_ofp_params->notifier_chain_value;
			OFP_INFO("send uiready: %d\n", p_oplus_ofp_params->notifier_chain_value);
			fb_notifier_call_chain(MTK_ONSCREENFINGERPRINT_EVENT, &event);
			mtk_drm_trace_c("%d|oplus_ofp_notifier_chain_value|%d", g_commit_pid, p_oplus_ofp_params->notifier_chain_value);
		}
		last_notifier_chain_value = p_oplus_ofp_params->notifier_chain_value;
		mtk_drm_trace_end();
	}

	return 0;
}

/* need filter backlight in hbm state and aod unlocking process */
bool oplus_ofp_backlight_filter(int bl_level)
{
	bool need_filter_backlight = false;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	mtk_drm_trace_begin("oplus_ofp_backlight_filter");

	if (oplus_ofp_get_hbm_state()) {
		if (bl_level == 0) {
			oplus_ofp_set_hbm_state(false);
			OFP_DEBUG("backlight is 0, set hbm state to false\n");
			if (p_oplus_ofp_params->aod_unlocking == true) {
				p_oplus_ofp_params->aod_unlocking = false;
				OFP_INFO("oplus_ofp_aod_unlocking: %d\n", p_oplus_ofp_params->aod_unlocking);
				mtk_drm_trace_c("%d|oplus_ofp_aod_unlocking|%d", g_commit_pid, p_oplus_ofp_params->aod_unlocking);
			}
			need_filter_backlight = false;
			p_oplus_ofp_params->touch_state = false;
		} else {
			OFP_INFO("hbm state is true, filter backlight setting\n");
			need_filter_backlight = true;
		}
	} else if (p_oplus_ofp_params->aod_unlocking && p_oplus_ofp_params->fp_press) {
		OFP_INFO("aod unlocking is true, filter backlight setting\n");
		need_filter_backlight = true;
	}

	mtk_drm_trace_end();

	OFP_DEBUG("end\n");

	return need_filter_backlight;
}

/* -------------------- aod -------------------- */
/* aod off status handle */
int oplus_ofp_aod_off_status_handle(void *mtk_drm_crtc)
{
	struct mtk_drm_crtc *mtk_crtc = mtk_drm_crtc;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!mtk_crtc || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	mtk_drm_trace_begin("oplus_ofp_aod_off_status_handle");
	/* doze disable status handle */
	OFP_INFO("aod off status handle\n");

	if (oplus_ofp_is_support()) {
		/* update aod unlocking value */
		oplus_ofp_aod_unlocking_update();
	}

	oplus_ofp_set_aod_state(false);

	/* aod off cmd is sent to ddic */
	p_oplus_ofp_params->aod_off_cmd_timestamp = ktime_get();
	OFP_DEBUG("aod_off_cmd_timestamp:%lu\n", ktime_to_ms(p_oplus_ofp_params->aod_off_cmd_timestamp));

	mtk_drm_trace_end();

	return 0;
}

/* aod status handle */
int oplus_ofp_doze_status_handle(bool doze_enable, void *drm_crtc, void *mtk_panel_ext, void *drm_panel, void *mtk_dsi, void *dcs_write_gce)
{
	struct drm_crtc *crtc = drm_crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!crtc || !mtk_crtc || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	mtk_drm_trace_begin("oplus_ofp_doze_status_handle");

	if (doze_enable) {
		OFP_INFO("doze status handle before doze enable\n");
		/* before doze enable */
		if (oplus_ofp_is_support()) {
			/* hbm mode -> normal mode -> aod mode */
			if (oplus_ofp_get_hbm_state() == true) {
				if (mtk_crtc_is_frame_trigger_mode(crtc)) {
					struct mtk_panel_ext *ext = mtk_panel_ext;

					mtk_drm_trace_begin("oplus_ofp_hbm_off_before_doze_enable");
					if (ext && ext->funcs && ext->funcs->hbm_set_cmdq) {
						OFP_INFO("hbm off before doze enable\n");
						ext->funcs->hbm_set_cmdq(drm_panel, mtk_dsi, dcs_write_gce, NULL, false);
					}
					oplus_ofp_set_hbm_state(false);
					mtk_drm_trace_end();
				}
			}

			/* reset aod unlocking flag when fingerprint unlocking failed */
			if (p_oplus_ofp_params->aod_unlocking == true) {
				p_oplus_ofp_params->aod_unlocking = false;
				OFP_INFO("oplus_ofp_aod_unlocking: %d\n", p_oplus_ofp_params->aod_unlocking);
				mtk_drm_trace_c("%d|oplus_ofp_aod_unlocking|%d", g_commit_pid, p_oplus_ofp_params->aod_unlocking);
			}
		}
		oplus_ofp_set_aod_state(true);
	} else {
		oplus_ofp_aod_off_status_handle(mtk_crtc);
	}

	mtk_drm_trace_end();

	return 0;
}

int oplus_ofp_set_aod_light_mode_after_doze_enable(void *mtk_panel_ext, void *mtk_dsi, void *dcs_write_gce)
{
	struct mtk_panel_ext *ext = mtk_panel_ext;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!ext || !mtk_dsi || !dcs_write_gce || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (p_oplus_ofp_params->aod_light_mode) {
		if (ext->funcs && ext->funcs->set_aod_light_mode) {
			ext->funcs->set_aod_light_mode(mtk_dsi, dcs_write_gce, NULL, p_oplus_ofp_params->aod_light_mode);
			OFP_INFO("set_aod_light_mode:%u\n", p_oplus_ofp_params->aod_light_mode);
		}
	}

	OFP_DEBUG("end\n");

	return 0;
}

/* aod off cmd cmdq set */
#define OFP_MAX_WAIT_RESUME_CNT   20
int oplus_ofp_aod_off_set_cmdq(struct drm_crtc *crtc)
{
	int i, j, count = 0;
	bool doze_en = false;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp, *comp;
	struct cmdq_pkt *cmdq_handle;
	struct cmdq_client *client;
	struct mtk_crtc_state *crtc_state;
	struct mtk_drm_private *priv = NULL;

	if (!crtc || !mtk_crtc) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(crtc->state)) {
		DDPPR_ERR("%s crtc->state is invalid\n", __func__);
		return -EINVAL;
	}

	crtc_state = to_mtk_crtc_state(crtc->state);
	if (IS_ERR_OR_NULL(crtc_state)) {
		DDPPR_ERR("%s crtc_state is invalid\n", __func__);
		return -EINVAL;
	}

	mtk_drm_trace_begin("oplus_ofp_aod_off_set_cmdq");
	if (mtk_crtc->base.dev) {
		priv = mtk_crtc->base.dev->dev_private;
		for (count = 0; count < OFP_MAX_WAIT_RESUME_CNT; count++) {
			if (!priv->suspend_state) {
				DDPMSG("%s suspend_state is null, mtk_drm_sys resume\n", __func__);
				break;
			} else {
				mdelay(5);
				pr_err("%s mtk_drm_sys not resume, wait count: %d\n", __func__, count);
			}
		}
	} else {
		DDPPR_ERR("%s-%d invalid params\n", __func__, __LINE__);
		return -EINVAL;
	}

	if(count == OFP_MAX_WAIT_RESUME_CNT) {
		DDPPR_ERR("%s-%d wait drm resume timeout \n", __func__, __LINE__);
		return -EINVAL;
	}

	if (!crtc_state->prop_val[CRTC_PROP_DOZE_ACTIVE]) {
		OFP_INFO("not in doze mode\n");
	}

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	mtk_drm_crtc_wk_lock(crtc, 1, __func__, __LINE__);

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (unlikely(!output_comp)) {
		OFP_ERR("Invalid params\n");
		mtk_drm_crtc_wk_lock(crtc, 0, __func__, __LINE__);
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		mtk_drm_trace_end();
		return -ENODEV;
	}

	if (IS_ERR_OR_NULL(output_comp->mtk_crtc)) {
		DDPPR_ERR("%s output_comp->mtk_crtc is invalid\n", __func__);
		output_comp->mtk_crtc = mtk_crtc;
	}

	if (IS_ERR_OR_NULL(&output_comp->mtk_crtc->base)) {
		DDPPR_ERR("%s output_comp->mtk_crtc->base is invalid\n", __func__);
		mtk_drm_crtc_wk_lock(crtc, 0, __func__, __LINE__);
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		mtk_drm_trace_end();
		return -EINVAL;
	}

	client = mtk_crtc->gce_obj.client[CLIENT_CFG];
	if (!mtk_crtc->enabled) {
		/* 1. power on mtcmos */
		mtk_drm_trace_begin("power_on_mtcmos");
		mtk_drm_top_clk_prepare_enable(crtc->dev);

		if (mtk_crtc_with_trigger_loop(crtc))
			mtk_crtc_start_trig_loop(crtc);

		mtk_ddp_comp_io_cmd(output_comp, NULL, CONNECTOR_ENABLE, NULL);

		for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
			mtk_dump_analysis(comp);

		OFP_INFO("power on mtcmos\n");
		mtk_drm_trace_end();
	}

	/* send LCM CMD */
	mtk_drm_trace_begin("prepare_to_send_aod_off_cmd");
	OFP_INFO("prepare to send aod off cmd\n");
	mtk_drm_send_lcm_cmd_prepare(crtc, &cmdq_handle);
	mtk_drm_trace_end();

	mtk_drm_trace_begin("DSI_SET_DOZE");
	if (output_comp->funcs && output_comp->funcs->io_cmd)
		output_comp->funcs->io_cmd(output_comp,
			cmdq_handle, DSI_SET_DOZE, &doze_en);
	OFP_INFO("DSI_SET_DOZE %d\n", doze_en);
	mtk_drm_trace_end();

	mtk_drm_trace_begin("flush_aod_off_cmd");
	mtk_drm_send_lcm_cmd_flush(crtc, &cmdq_handle, 1);
	OFP_INFO("flush aod off cmd\n");
	mtk_drm_trace_end();

	oplus_ofp_aod_off_status_handle(mtk_crtc);

	if (!mtk_crtc->enabled) {
		mtk_drm_trace_begin("power_off_mtcmos");
		if (mtk_crtc_with_trigger_loop(crtc))
			mtk_crtc_stop_trig_loop(crtc);

		mtk_ddp_comp_io_cmd(output_comp, NULL, CONNECTOR_DISABLE, NULL);

		mtk_drm_top_clk_disable_unprepare(crtc->dev);
		OFP_INFO("power off mtcmos\n");
		mtk_drm_trace_end();
	}

	mtk_drm_crtc_wk_lock(crtc, 0, __func__, __LINE__);

	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	mtk_drm_trace_end();

	return 0;
}

int oplus_ofp_crtc_aod_off_set(void)
{
	int ret = 0;
	struct drm_crtc *crtc;
	struct drm_device *drm_dev = get_drm_device();

	OFP_DEBUG("start\n");

	if (IS_ERR_OR_NULL(drm_dev)) {
		OFP_ERR("invalid drm dev\n");
		return -EINVAL;
	}

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(drm_dev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (IS_ERR_OR_NULL(crtc)) {
		OFP_ERR("find crtc fail\n");
		return -EINVAL;
	}

	ret = oplus_ofp_aod_off_set_cmdq(crtc);

	OFP_DEBUG("end\n");

	return ret;
}

void oplus_ofp_aod_off_set_work_handler(struct work_struct *work_item)
{
	int ret = 0;

	mtk_drm_trace_begin("oplus_ofp_aod_off_set_work_handler");
	OFP_INFO("send aod off cmd to speed up aod unlocking\n");
	ret = oplus_ofp_crtc_aod_off_set();
	if (ret) {
		OFP_ERR("failed to send aod off cmd\n");
	}
	mtk_drm_trace_end();

	return;
}

int oplus_ofp_aod_off_set(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OFP_DEBUG("start\n");

	if (oplus_ofp_get_hbm_state()) {
		OFP_DEBUG("ignore aod off setting in hbm state\n");
		return 0;
	}

	mtk_drm_trace_begin("oplus_ofp_aod_off_set");
	if (oplus_ofp_get_aod_state() && p_oplus_ofp_params->doze_active != 0) {
		OFP_INFO("queue aod off set work\n");
		queue_work(p_oplus_ofp_params->aod_off_set_wq, &p_oplus_ofp_params->aod_off_set_work);
		oplus_ofp_set_aod_state(false);
	}
	mtk_drm_trace_end();

	OFP_DEBUG("end\n");

	return 0;
}
/*
 touchpanel notify touchdown event when fingerprint is pressed,
 then display driver send aod off cmd immediately and vsync change to 120hz,
 so that press icon layer can sent down faster
*/
int oplus_ofp_touchpanel_event_irq_call(struct fp_underscreen_info *tp_info)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!oplus_ofp_is_support()) {
		OFP_DEBUG("no need to send aod off cmd in doze mode to speed up fingerprint unlocking\n");
		return IRQ_HANDLED;
	}

	mtk_drm_trace_begin("oplus_ofp_touchpanel_event_irq_call");

	if (tp_info->touch_state) {
		OFP_INFO("tp touchdown\n");
		p_oplus_ofp_params->touch_state = true;
		/* send aod off cmd in doze mode to speed up fingerprint unlocking */
		oplus_ofp_aod_off_set();
	} else {
		OFP_INFO("tp touchup\n");
		p_oplus_ofp_params->touch_state = false;
	}

	mtk_drm_trace_end();

	return IRQ_HANDLED;
}
EXPORT_SYMBOL(oplus_ofp_touchpanel_event_irq_call);

/*
 as there have some black frames are inserted in aod off cmd flow which will affect hbm on cmd execution time,
 so check how many black frames have taken effect first,
 then calculate delay time to keep apart aod off cmd and hbm on cmd to make sure ui ready is accurate
*/
int oplus_ofp_aod_off_hbm_on_delay_check(void *mtk_drm_crtc)
{
	static bool last_aod_unlocking = false;
	static unsigned int te_rdy_irq_count = 0;
	unsigned int aod_off_insert_black = 0;
	struct mtk_drm_crtc *mtk_crtc = mtk_drm_crtc;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!mtk_crtc || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!oplus_ofp_is_support()) {
		OFP_DEBUG("no need to check aod off hbm on delay\n");
		return 0;
	}

	mtk_drm_trace_begin("oplus_ofp_aod_off_hbm_on_delay_check");

	if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params
			&& mtk_crtc->panel_ext->params->oplus_ofp_aod_off_insert_black > 0) {
		if (p_oplus_ofp_params->aod_unlocking == true) {
			if (last_aod_unlocking == false) {
				te_rdy_irq_count = 1;
			} else if (te_rdy_irq_count != 0 && te_rdy_irq_count < 10) {
				te_rdy_irq_count++;
			} else {
				/* 10 irq is enough */
				te_rdy_irq_count = 10;
			}
		} else {
			te_rdy_irq_count = 0;
			p_oplus_ofp_params->aod_off_hbm_on_delay = 0;
		}

		aod_off_insert_black = mtk_crtc->panel_ext->params->oplus_ofp_aod_off_insert_black;
		if (te_rdy_irq_count < aod_off_insert_black) {
			p_oplus_ofp_params->aod_off_hbm_on_delay = aod_off_insert_black - te_rdy_irq_count;
			OFP_DEBUG("aod_off_insert_black=%d,te_rdy_irq_count=%d,aod_off_hbm_on_delay=%d\n",
				aod_off_insert_black, te_rdy_irq_count, p_oplus_ofp_params->aod_off_hbm_on_delay);
		} else {
			p_oplus_ofp_params->aod_off_hbm_on_delay = 0;
		}

		last_aod_unlocking = p_oplus_ofp_params->aod_unlocking;
	}

	mtk_drm_trace_end();

	OFP_DEBUG("end\n");

	return 0;
}

/* -------------------- node -------------------- */
/* fod part */
/* notify fp press for hidl */
int oplus_ofp_notify_fp_press(void *buf)
{
	unsigned int *fp_press = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params || !fp_press) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if ((*fp_press) == 1) {
		/* finger is pressed down and pressed icon layer is ready */
		p_oplus_ofp_params->fp_press = true;
	} else {
		p_oplus_ofp_params->fp_press = false;
	}
	OFP_INFO("receive uiready %d\n", p_oplus_ofp_params->fp_press);
	mtk_drm_trace_c("%d|oplus_ofp_fp_press|%d", g_commit_pid, p_oplus_ofp_params->fp_press);


	if (oplus_ofp_is_support() && p_oplus_ofp_params->fp_press) {
		/* send aod off cmd in doze mode to speed up fingerprint unlocking */
		OFP_DEBUG("fp press is true\n");
		oplus_ofp_aod_off_set();
	}

	return 0;
}

/* notify fp press for sysfs */
ssize_t oplus_ofp_notify_fp_press_attr(struct kobject *obj, struct kobj_attribute *attr,
	const char *buf, size_t count)
{
	unsigned int fp_press = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	if (kstrtouint(buf, 0, &fp_press)) {
		OFP_ERR("kstrtouint error!\n");
		return count;
	}

	if (fp_press == 1) {
		/* finger is pressed down and pressed icon layer is ready */
		p_oplus_ofp_params->fp_press = true;
	} else {
		p_oplus_ofp_params->fp_press = false;
	}
	OFP_INFO("receive uiready %d\n", p_oplus_ofp_params->fp_press);
	mtk_drm_trace_c("%d|oplus_ofp_fp_press|%d", g_commit_pid, p_oplus_ofp_params->fp_press);


	if (oplus_ofp_is_support() && p_oplus_ofp_params->fp_press) {
		/* send aod off cmd in doze mode to speed up fingerprint unlocking */
		OFP_DEBUG("fp press is true\n");
		oplus_ofp_aod_off_set();
	}

	return count;
}

int oplus_ofp_drm_set_hbm(struct drm_crtc *crtc, unsigned int hbm_mode)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle;
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (!crtc || !mtk_crtc) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!(mtk_crtc->enabled)) {
		OFP_ERR("should not set hbm if mtk crtc is not enabled\n");
		return -EFAULT;
	}

	mutex_lock(&mtk_crtc->lock);

	mtk_drm_trace_begin("mtk_drm_send_lcm_cmd_prepare");
	OFP_INFO("prepare to send hbm cmd\n");
	mtk_drm_send_lcm_cmd_prepare(crtc, &cmdq_handle);
	mtk_drm_trace_end();

	/* set hbm */
	 if (comp && comp->funcs && comp->funcs->io_cmd) {
		 mtk_drm_trace_begin("LCM_HBM");
		comp->funcs->io_cmd(comp, cmdq_handle, LCM_HBM, &hbm_mode);
		OFP_INFO("LCM_HBM\n");
		mtk_drm_trace_end();
	}

	mtk_drm_trace_begin("mtk_drm_send_lcm_cmd_flush");
	mtk_drm_send_lcm_cmd_flush(crtc, &cmdq_handle, 0);
	OFP_INFO("mtk_drm_send_lcm_cmd_flush end\n");
	oplus_ofp_set_hbm_state(hbm_mode);
	mtk_drm_trace_end();

	 mutex_unlock(&mtk_crtc->lock);

	 return 0;
}

int oplus_ofp_get_hbm(void *buf)
{
	unsigned int *hbm_mode = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	(*hbm_mode) = p_oplus_ofp_params->hbm_mode;
	OFP_INFO("hbm_mode = %d\n", *hbm_mode);

	return 0;
}

int oplus_ofp_set_hbm(void *buf)
{
	unsigned int *hbm_mode = buf;
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!ddev || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		OFP_ERR("find crtc fail\n");
		return -EINVAL;
	}

	OFP_INFO("%d to %d\n", p_oplus_ofp_params->hbm_mode, *hbm_mode);
	oplus_ofp_drm_set_hbm(crtc, *hbm_mode);
	p_oplus_ofp_params->hbm_mode = (*hbm_mode);
	mtk_drm_trace_c("%d|oplus_ofp_hbm_mode|%d", g_commit_pid, p_oplus_ofp_params->hbm_mode);

	return 0;
}

ssize_t oplus_ofp_get_hbm_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OFP_INFO("hbm_mode = %d\n", p_oplus_ofp_params->hbm_mode);

	return sprintf(buf, "%d\n", p_oplus_ofp_params->hbm_mode);
}

ssize_t oplus_ofp_set_hbm_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int hbm_mode = 0;
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!ddev || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	if (kstrtouint(buf, 10, &hbm_mode)) {
		OFP_ERR("kstrtouint error!\n");
		return count;
	}

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		OFP_ERR("find crtc fail\n");
		return -count;
	}

	OFP_INFO("%d to %d\n", p_oplus_ofp_params->hbm_mode, hbm_mode);
	oplus_ofp_drm_set_hbm(crtc, hbm_mode);
	p_oplus_ofp_params->hbm_mode = hbm_mode;
	mtk_drm_trace_c("%d|oplus_ofp_hbm_mode|%d", g_commit_pid, p_oplus_ofp_params->hbm_mode);

	return count;
}



int oplus_ofp_set_aod_light_mode(void *buf)
{
	int rc = 0;
	unsigned int *aod_light_mode = buf;
	static unsigned int last_aod_light_mode = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	last_aod_light_mode = p_oplus_ofp_params->aod_light_mode;
	p_oplus_ofp_params->aod_light_mode = (*aod_light_mode);
	OFP_INFO("aod_light_mode:%u\n", p_oplus_ofp_params->aod_light_mode);
	mtk_drm_trace_c("%d|oplus_ofp_aod_light_mode|%d", g_commit_pid, p_oplus_ofp_params->aod_light_mode);

	if (!oplus_ofp_is_support()) {
		OFP_DEBUG("aod is not supported\n");
		return 0;
	} else if (!oplus_ofp_get_aod_state()) {
		OFP_ERR("not in aod mode, should not set aod_light_mode\n");
		return 0;
	} else if (oplus_ofp_get_hbm_state()) {
		OFP_INFO("ignore aod light mode setting in hbm state\n");
		return 0;
	}

	if (last_aod_light_mode != p_oplus_ofp_params->aod_light_mode) {
		mtkfb_set_aod_backlight_level(p_oplus_ofp_params->aod_light_mode);
	}

	OFP_DEBUG("end\n");

	return rc;
}

int oplus_ofp_get_aod_light_mode(void *buf)
{
	unsigned int *aod_light_mode = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	*aod_light_mode = p_oplus_ofp_params->aod_light_mode;
	OFP_INFO("aod_light_mode:%u\n", *aod_light_mode);

	OFP_DEBUG("end\n");

	return 0;
}

ssize_t oplus_ofp_set_aod_light_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int aod_light_mode = 0;
	static unsigned int last_aod_light_mode = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	sscanf(buf, "%u", &aod_light_mode);

	last_aod_light_mode = p_oplus_ofp_params->aod_light_mode;
	p_oplus_ofp_params->aod_light_mode = aod_light_mode;
	OFP_INFO("aod_light_mode:%u\n", p_oplus_ofp_params->aod_light_mode);
	mtk_drm_trace_c("%d|oplus_ofp_aod_light_mode|%d", g_commit_pid, p_oplus_ofp_params->aod_light_mode);

	if (!oplus_ofp_is_support()) {
		OFP_DEBUG("aod is not supported\n");
		return count;
	} else if (!oplus_ofp_get_aod_state()) {
		OFP_ERR("not in aod mode, should not set aod_light_mode\n");
		return count;
	} else if (oplus_ofp_get_hbm_state()) {
		OFP_INFO("ignore aod light mode setting in hbm state\n");
		return count;
	}

	if (last_aod_light_mode != p_oplus_ofp_params->aod_light_mode) {
		mtkfb_set_aod_backlight_level(p_oplus_ofp_params->aod_light_mode);
	}

	OFP_DEBUG("end\n");

	return count;
}

ssize_t oplus_ofp_get_aod_light_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OFP_INFO("aod_light_mode:%u\n", p_oplus_ofp_params->aod_light_mode);

	OFP_DEBUG("end\n");

	return sprintf(buf, "%u\n", p_oplus_ofp_params->aod_light_mode);
}

MODULE_AUTHOR("Liuhe Zhong");
MODULE_DESCRIPTION("OPLUS ofp device");
MODULE_LICENSE("GPL v2");
#endif
