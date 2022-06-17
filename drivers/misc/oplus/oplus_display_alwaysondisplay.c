/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_alwaysondisplay.h
** Description : oplus_display_alwaysondisplay. implement
** Version : 1.0
** Date : 2020/05/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Zhang.JianBin2020/05/13        1.0          Modify for MT6779_R
******************************************************************/
#include "oplus_display_private_api.h"
#include "disp_drv_log.h"
#include <linux/fb.h>
#include <linux/time.h>
#include <linux/timekeeping.h>
#include <linux/oplus_mm_kevent_fb.h>
#include <linux/delay.h>
#include <soc/oplus/oplus_project.h>
#include "ddp_dsi.h"
#include "fbconfig_kdebug.h"
#include "oplus_display_alwaysondisplay.h"
/*
 * we will create a sysfs which called /sys/kernel/oplus_display,
 * In that directory, oplus display private api can be called
 */
/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
extern int primary_display_def_dst_mode;
extern int primary_display_cur_dst_mode;
extern unsigned char aod_area_cmd[];
extern void set_is_dc(unsigned int is_dc);
/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */

 /*
 * modify for support aod state.
 */
extern unsigned int aod_light_mode;
extern bool oplus_display_aodlight_support;
extern bool primary_display_get_fp_hbm_state(void);
extern int primary_display_aod_backlight(int level);
int disp_lcm_aod_from_display_on(struct disp_lcm_handle *plcm);
int _set_aod_mode_by_cmdq(unsigned int mode);
int primary_display_set_aod_mode_nolock(unsigned int mode);
void oplus_display_aod_backlight(void);
extern bool oplus_flag_lcd_off;
extern void oplus_cmdq_flush_config_handle_mira(void *handle, int blocking);
extern void oplus_cmdq_handle_clear_dirty(struct cmdqRecStruct *cmdq_handle);
extern void oplus_delayed_trigger_kick_set(int params);
extern enum lcm_power_state primary_display_get_lcm_power_state_nolock(void );
extern enum lcm_power_state
primary_display_set_lcm_power_state_nolock(enum lcm_power_state new_state);
extern enum DISP_POWER_STATE oplus_primary_set_state(enum DISP_POWER_STATE new_state);
extern enum mtkfb_power_mode primary_display_get_power_mode_nolock(void );
extern void oplus_cmdq_build_trigger_loop(void);
extern void oplus_cmdq_reset_config_handle(void);

int disp_lcm_aod_from_display_on(struct disp_lcm_handle *plcm)
{
	 struct LCM_DRIVER *lcm_drv = NULL;

	 DISPMSG("[soso] %s \n", __func__);
	 if (_is_lcm_inited(plcm)) {
		 lcm_drv = plcm->drv;

		 if (lcm_drv->resume_power)
			 lcm_drv->resume_power();

		 if (lcm_drv->disp_lcm_aod_from_display_on) {
			 lcm_drv->disp_lcm_aod_from_display_on();
		 } else {
			 DISP_PR_ERR("FATAL ERROR, lcm_drv->aod is null\n");
			 return -1;
		 }

		 oplus_flag_lcd_off = false;

		 return 0;
	 }

	 DISP_PR_ERR("lcm_drv is null\n");
	 return -1;
}

int disp_lcm_set_aod_mode(struct disp_lcm_handle *plcm, void *handle, unsigned int mode)
{
	 struct LCM_DRIVER *lcm_drv = NULL;

	 DISPFUNC();
	 if (_is_lcm_inited(plcm)) {
		 lcm_drv = plcm->drv;
		 if (lcm_drv->set_aod_brightness) {
			 lcm_drv->set_aod_brightness(handle, mode);
		 } else {
			 DISP_PR_ERR("FATAL ERROR, lcm_drv->set_aod_brightness is null\n");
			 return -1;
		 }
		 return 0;
	 }
	 DISP_PR_ERR("lcm_drv is null\n");
	 return -1;
}

int _set_aod_mode_by_cmdq(unsigned int mode)
{
	int ret = 0;

	struct cmdqRecStruct *cmdq_handle_aod_mode = NULL;

	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_cmd,
		MMPROFILE_FLAG_PULSE, 1, 1);

	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP,&cmdq_handle_aod_mode);

	if(ret!=0)
	{
		DISPCHECK("fail to create primary cmdq handle for aod mode\n");
		return -1;
	}

	if (primary_display_is_video_mode()) {
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
			MMPROFILE_FLAG_PULSE, 1, 2);
		cmdqRecReset(cmdq_handle_aod_mode);
		ret = disp_lcm_set_aod_mode(pgc->plcm,cmdq_handle_aod_mode,mode);
		oplus_cmdq_flush_config_handle_mira(cmdq_handle_aod_mode, 1);
	} else {
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
			MMPROFILE_FLAG_PULSE, 1, 3);
		cmdqRecReset(cmdq_handle_aod_mode);
		cmdqRecWait(cmdq_handle_aod_mode, CMDQ_SYNC_TOKEN_CABC_EOF);
		oplus_cmdq_handle_clear_dirty(cmdq_handle_aod_mode);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_aod_mode);
		ret = disp_lcm_set_aod_mode(pgc->plcm,cmdq_handle_aod_mode,mode);
		cmdqRecSetEventToken(cmdq_handle_aod_mode, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		cmdqRecSetEventToken(cmdq_handle_aod_mode, CMDQ_SYNC_TOKEN_CABC_EOF);
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
			MMPROFILE_FLAG_PULSE, 1, 4);
		oplus_cmdq_flush_config_handle_mira(cmdq_handle_aod_mode, 1);
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
			MMPROFILE_FLAG_PULSE, 1, 6);
	}
	cmdqRecDestroy(cmdq_handle_aod_mode);
	cmdq_handle_aod_mode = NULL;
	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
		MMPROFILE_FLAG_PULSE, 1, 5);

	return ret;
}

int primary_display_set_aod_mode_nolock(unsigned int mode)
{
	int ret = 0;

	if (oplus_flag_lcd_off)
	{
		pr_err("lcd is off,don't allow to set aod\n");
		return 0;
	}

	DISPFUNC();

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL) {
		DISPMSG("%s skip due to stage %s\n", __func__, disp_helper_stage_spy());
		return 0;
	}

	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
			MMPROFILE_FLAG_START, 0, 0);
	if (pgc->state == DISP_SLEPT) {
		DISPCHECK("Sleep State set aod mode invald\n");
	} else {
		primary_display_idlemgr_kick((char *)__func__, 0);
		if (primary_display_cmdq_enabled()) {
			if (primary_display_is_video_mode()) {
				mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
					       MMPROFILE_FLAG_PULSE, 0, 7);
			} else {
				_set_aod_mode_by_cmdq(mode);
			}
			//atomic_set(&delayed_trigger_kick, 1);
			oplus_delayed_trigger_kick_set(1);
		}
	}
	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
		MMPROFILE_FLAG_END, 0, 0);
	return ret;
}
//#endif

void oplus_display_aod_backlight()
{
	int ret;
	/* blocking flush before stop trigger loop */
	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		int event_ret;
		mmprofile_log_ex(ddp_mmp_get_events()->primary_suspend,
				 MMPROFILE_FLAG_PULSE, 1, 2);
		event_ret = dpmgr_wait_event_timeout(pgc->dpmgr_handle,
				 DISP_PATH_EVENT_FRAME_DONE, HZ * 1);

		mmprofile_log_ex(ddp_mmp_get_events()->primary_suspend,
				 MMPROFILE_FLAG_PULSE, 2, 2);
		DISPCHECK("display path is busy now,wait frame done,event=%d\n",
				event_ret);
		if (event_ret <= 0) {
				DISP_PR_ERR("wait frame done in suspend timeout\n");
				mmprofile_log_ex(ddp_mmp_get_events()->primary_suspend,
					 MMPROFILE_FLAG_PULSE, 3, 2);
				primary_display_diagnose(__func__, __LINE__);
				ret = -1;
			}
	}
	DISPCHECK("%s stop trig loop\n", __func__);
	_cmdq_stop_trigger_loop();

	dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);

	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		mmprofile_log_ex(ddp_mmp_get_events()->primary_suspend,
			 MMPROFILE_FLAG_PULSE, 1, 4);
		DISP_PR_ERR("[POWER]stop display path failed, still busy\n");
		dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
		ret = -1;
		/*
		 * even path is busy(stop fail), we still need to
		 * continue power off other module/devices
		 */
		/* goto done; */
	}

	if (primary_display_get_lcm_power_state_nolock() != LCM_ON_LOW_POWER) {
		if (pgc->plcm->drv->aod)
			disp_lcm_aod(pgc->plcm, 1);
		primary_display_set_lcm_power_state_nolock(LCM_ON_LOW_POWER);
	}

	dpmgr_path_power_off(pgc->dpmgr_handle, CMDQ_DISABLE);
	pgc->lcm_refresh_rate = 60;
	/* pgc->state = DISP_SLEPT; */
	oplus_primary_set_state(DISP_SLEPT);
}

int oplus_panel_get_aod_light_mode(void *buf)
{
	unsigned int *aod_mode = buf;
	(*aod_mode) = aod_light_mode;
	printk(KERN_INFO "oplus_panel_get_aod_light_mode = %d\n",aod_light_mode);
	return 0;
}

int oplus_panel_set_aod_light_mode(void *buf)
{
	unsigned int *temp_save = buf;
	int ret = 0;

	pr_info("%s, oplus_display_aodlight_support = %d, aod_light_mode = %d\n", __func__, oplus_display_aodlight_support, *temp_save);
	if (oplus_display_aodlight_support) {

		if (primary_display_get_fp_hbm_state()) {
			printk(KERN_INFO "oplus_set_aod_light_mode = %d return on hbm\n",(*temp_save));
			return 0;
		}
		aod_light_mode = (*temp_save);
		ret = primary_display_aod_backlight(aod_light_mode);
		printk(KERN_INFO "oplus_panel_set_aod_light_mode = %d\n",(*temp_save));
	}

	return 0;
}

/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
int disp_lcm_set_aod_area(struct disp_lcm_handle *plcm, void *handle, unsigned char *area)
{
	struct LCM_DRIVER *lcm_drv = NULL;

	DISPFUNC();
	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->set_aod_area_cmdq) {
			lcm_drv->set_aod_area_cmdq(handle, area);
		} else {
			DISP_PR_ERR("FATAL ERROR, lcm_drv->set_aod_area_cmdq is null\n");
			return -1;
		}

		return 0;
	}
	DISP_PR_ERR("lcm_drv is null\n");
	return -1;
}

int disp_lcm_set_aod_cv_mode(struct disp_lcm_handle *plcm, void *handle, unsigned int mode)
{
	 struct LCM_DRIVER *lcm_drv = NULL;

	 DISPFUNC();
	 if (_is_lcm_inited(plcm)) {
		 lcm_drv = plcm->drv;
		 if (lcm_drv->set_aod_cv_mode) {
			 lcm_drv->set_aod_cv_mode(handle, mode);
		 } else {
			 DISP_PR_ERR("FATAL ERROR, lcm_drv->set_aod_cv_mode is null\n");
			 return -1;
		 }
		 return 0;
	 }
	 DISP_PR_ERR("lcm_drv is null\n");
	 return -1;
}

int disp_lcm_doze_enable(struct disp_lcm_handle *plcm, void *handle)
{
	struct LCM_DRIVER *lcm_drv = NULL;

	DISPMSG("%s\n", __func__);
	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->doze_enable) {
			lcm_drv->doze_enable(handle);
		} else {
			DISP_PR_ERR("FATAL ERROR, lcm_drv->doze_enable is null\n");
			return -1;
		}
		return 0;
	}

	DISP_PR_ERR("lcm_drv is null\n");
	return -1;
}

int disp_lcm_doze_disable(struct disp_lcm_handle *plcm, void *handle)
{
	struct LCM_DRIVER *lcm_drv = NULL;

	DISPMSG("%s\n", __func__);
	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->doze_disable) {
			lcm_drv->doze_disable(handle);
		} else {
			DISP_PR_ERR("FATAL ERROR, lcm_drv->doze_disable is null\n");
			return -1;
		}
		return 0;
	}

	DISP_PR_ERR("lcm_drv is null\n");
	return -1;
}

int _set_aod_area_by_cmdq(unsigned char *area)
{
	int ret = 0;
	struct cmdqRecStruct *cmdq_handle_area = NULL;
	DISPFUNC();
	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl, MMPROFILE_FLAG_PULSE, 1, 1);
	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle_area);
	DISPDBG("primary aod area, handle=%p\n", cmdq_handle_area);
	if (ret) {
		DISP_PR_ERR("fail to create primary cmdq handle for aod area\n");
		return -1;
	}

	if (!primary_display_is_video_mode()) {
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl, MMPROFILE_FLAG_PULSE, 1, 3);
		cmdqRecReset(cmdq_handle_area);
		cmdqRecWait(cmdq_handle_area, CMDQ_SYNC_TOKEN_CABC_EOF);
		oplus_cmdq_handle_clear_dirty(cmdq_handle_area);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_area);
		disp_lcm_set_aod_area(pgc->plcm, cmdq_handle_area, area);
		cmdqRecSetEventToken(cmdq_handle_area, CMDQ_SYNC_TOKEN_CABC_EOF);
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl, MMPROFILE_FLAG_PULSE, 1, 4);
		oplus_cmdq_flush_config_handle_mira(cmdq_handle_area, 1);
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl, MMPROFILE_FLAG_PULSE, 1, 6);
		DISPMSG("[BL]_set_aod_area_by_cmdq ret=%d\n", ret);
	}
	cmdqRecDestroy(cmdq_handle_area);
	cmdq_handle_area = NULL;
	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl, MMPROFILE_FLAG_PULSE, 1, 5);

	return ret;
}

int primary_display_set_aod_area(unsigned char *area, int use_cmdq)
{
	int ret = 0;

	DISPFUNC();
	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL) {
		DISPMSG("%s skip due to stage %s\n", __func__, disp_helper_stage_spy());
		return 0;
	}

	if (primary_display_get_power_mode_nolock() != DOZE) {
		DISPMSG("only doze mode set aod area!\n");
		/* return 0; */
	}

	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl, MMPROFILE_FLAG_START, 0, 0);

	/* _primary_path_switch_dst_lock(); */
	/* _primary_path_lock(__func__); */

	if (pgc->state == DISP_SLEPT) {
		DISP_PR_ERR("Sleep State set aod_area invald\n");
	} else {
		primary_display_idlemgr_kick(__func__, 0);
		if (primary_display_cmdq_enabled()) {
			if (primary_display_is_video_mode()) {
				mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
						 MMPROFILE_FLAG_PULSE, 0, 7);
			DISPMSG("video mode can't setaod area!\n");

			} else {
				if (use_cmdq) {
					_set_aod_area_by_cmdq(area);
				} else {
					disp_lcm_set_aod_area(pgc->plcm, NULL, area);
				}
			}
			/* atomic_set(&delayed_trigger_kick, 1); */
			oplus_delayed_trigger_kick_set(1);
		}
	}

	/* _primary_path_unlock(__func__); */
	/* _primary_path_switch_dst_unlock(); */

	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl, MMPROFILE_FLAG_END, 0, 0);
	return ret;
}

/* mode: 0, switch to cmd mode; 1, switch to vdo mode */
int primary_display_switch_aod_mode(int mode)
{
	enum DISP_STATUS ret = DISP_STATUS_ERROR;
	disp_path_handle disp_handle = NULL;
	struct disp_ddp_path_config *pconfig = NULL;
	int switch_mode = 0;
#if FALSE
	if (!disp_helper_get_option(DISP_OPT_CV_BYSUSPEND)) {
		return 0;
	}
#endif

	DISPFUNC();
	DISPCHECK("%s YYP mode = %d\n", __func__, mode);

	/* _primary_path_switch_dst_lock(); */
	/* disp_sw_mutex_lock(&(pgc->capture_lock)); */
	/* _primary_path_lock(__func__); */
	mmprofile_log_ex(ddp_mmp_get_events()->primary_display_switch_dst_mode,
			 MMPROFILE_FLAG_START, primary_display_cur_dst_mode,
			 mode);
	DISPCHECK("[C2V][cmd/video]aod cur_mode:%d, dst_mode:%d\n", primary_display_cur_dst_mode,
		mode);

	if (pgc->plcm->params->type != LCM_TYPE_DSI) {
		mmprofile_log_ex(
			ddp_mmp_get_events()->primary_display_switch_dst_mode,
			MMPROFILE_FLAG_PULSE, 5, pgc->plcm->params->type);
		DISPCHECK("[C2V]dst mode switch only support DSI IF\n");
		goto done;
	}
	if (pgc->state == DISP_SLEPT) {
		mmprofile_log_ex(
			ddp_mmp_get_events()->primary_display_switch_dst_mode,
			MMPROFILE_FLAG_PULSE, 6, pgc->state);
		DISPCHECK("%s: primary display path is already slept, skip\n",
			  __func__);
		goto done;
	}

#if FALSE
	if (mode == primary_display_cur_dst_mode) {
		mmprofile_log_ex(
			ddp_mmp_get_events()->primary_display_switch_dst_mode,
			MMPROFILE_FLAG_PULSE, 7, mode);
		DISPCHECK("%s: not need switch,cur_mode:%d, switch_mode:%d\n",
			  __func__, primary_display_cur_dst_mode, mode);
		goto done;
	}
#endif
	primary_display_idlemgr_kick(__func__, 0);

	/*
	 * When switch to VDO mode, go back to DL mode
	 * if display path changes to DC mode by SMART OVL
	 */
	if (disp_helper_get_option(DISP_OPT_SMART_OVL) && !primary_display_is_video_mode()) {
		/* switch to the mode before idle */
		do_primary_display_switch_mode(DISP_SESSION_DIRECT_LINK_MODE,
					primary_get_sess_id(), 0, NULL, 0);
		set_is_dc(0);
		DISPMSG("switch to the mode before idle\n");
	}

	mmprofile_log_ex(ddp_mmp_get_events()->primary_display_switch_dst_mode,
			 MMPROFILE_FLAG_PULSE, 4, 0);
	oplus_cmdq_reset_config_handle();

	/* 1.modify lcm mode - sw */
	mmprofile_log_ex(ddp_mmp_get_events()->primary_display_switch_dst_mode,
			 MMPROFILE_FLAG_PULSE, 4, 1);

	DISPCHECK("%s [cmd/video] primary_display_def_dst_mode = %d\n", __func__, primary_display_def_dst_mode);

	if (mode) {
		pgc->plcm->params->dsi.mode = primary_display_def_dst_mode;
		switch_mode = primary_display_def_dst_mode;
	} else {
		pgc->plcm->params->dsi.mode = CMD_MODE;
		switch_mode = CMD_MODE;
	}

	DISPCHECK("%s [cmd/video] mode = %d\n", __func__, primary_display_is_video_mode());

	dpmgr_path_set_video_mode(pgc->dpmgr_handle,
			  primary_display_is_video_mode());

	/* 2.Change PLL CLOCK parameter and build fps lcm command */
#if FALSE
	disp_lcm_adjust_fps(pgc->cmdq_handle_config, pgc->plcm,
			    pgc->lcm_refresh_rate);
#endif
	disp_handle = pgc->dpmgr_handle;
	pconfig = dpmgr_path_get_last_config(disp_handle);
	pconfig->dispif_config.dsi.PLL_CLOCK = pgc->plcm->params->dsi.PLL_CLOCK;
	pconfig->dispif_config.dsi.mode = pgc->plcm->params->dsi.mode;

	/*3.send aod to normal cmd*/
	if (primary_display_is_video_mode()) {
		/* disp_lcm_aod(pgc->plcm, 0); */
	} else {
		disp_lcm_aod(pgc->plcm, 1);
		disp_lcm_set_aod_area(pgc->plcm, NULL, aod_area_cmd);
	}

	/* 4.re-config RDMA golden setting */
	/* RDMA golden setting would change depend on VDO/CMD mode */
	dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_config,
		DDP_RDMA_GOLDEN_SETTING, pconfig);

	/* 5.Switch mode and change DSI clock */
	if (dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_config,
		DDP_SWITCH_AOD_MODE, (void *)&switch_mode) != 0) {
		mmprofile_log_ex(
			ddp_mmp_get_events()->primary_display_switch_dst_mode,
			MMPROFILE_FLAG_PULSE, 9, 0);
		ret = -1;
	}

	/* 6. rebuild trigger loop */
	mmprofile_log_ex(ddp_mmp_get_events()->primary_display_switch_dst_mode,
			 MMPROFILE_FLAG_PULSE, 4, 2);
	_cmdq_stop_trigger_loop();
	oplus_cmdq_build_trigger_loop();
	_cmdq_start_trigger_loop();
	oplus_cmdq_reset_config_handle();
	_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

	mmprofile_log_ex(ddp_mmp_get_events()->primary_display_switch_dst_mode,
			 MMPROFILE_FLAG_PULSE, 4, 3);
	primary_display_cur_dst_mode = mode;
	DISPCHECK("[cmd/video]primary_display_cur_dst_mode %d\n",
		primary_display_cur_dst_mode);
	if (primary_display_is_video_mode()) {
		dpmgr_map_event_to_irq(pgc->dpmgr_handle,
				       DISP_PATH_EVENT_IF_VSYNC,
				       DDP_IRQ_RDMA0_DONE);
	} else {
		dpmgr_map_event_to_irq(pgc->dpmgr_handle,
				       DISP_PATH_EVENT_IF_VSYNC,
				       DDP_IRQ_DSI0_EXT_TE);
	}

	ret = DISP_STATUS_OK;
done:
	primary_display_idlemgr_kick(__func__, 0);

	mmprofile_log_ex(ddp_mmp_get_events()->primary_display_switch_dst_mode,
			MMPROFILE_FLAG_END, primary_display_cur_dst_mode, mode);
	/* _primary_path_unlock(__func__); */
	/* disp_sw_mutex_unlock(&(pgc->capture_lock)); */
	/* _primary_path_switch_dst_unlock(); */
	return ret;
}
/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */
