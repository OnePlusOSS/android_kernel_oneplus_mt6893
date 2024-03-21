/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** OPLUS_BUG_STABILITY
** File : oplus_display_onscreenfingerprint.h
** Description : oplus_display_onscreenfingerprint. implement
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
#ifdef OPLUS_FEATURE_MM_FEEDBACK
//#include <soc/oplus/system/oplus_mm_kevent_fb.h>
#endif /* OPLUS_FEATURE_MM_FEEDBACK */
#include <linux/delay.h>
/* #include <soc/oplus/oplus_project.h> */
#include "ddp_dsi.h"
#include "fbconfig_kdebug.h"
#include "oplus_display_onscreenfingerprint.h"
/*
 * we will create a sysfs which called /sys/kernel/oplus_display,
 * In that directory, oplus display private api can be called
 */
int notify_display_fpd(bool mode);
void fpd_notify_check_trig(void);
void fpd_notify(void);
bool need_update_fpd_fence(struct disp_frame_cfg_t *cfg);

static struct task_struct *hbm_notify_task;
static wait_queue_head_t hbm_notify_task_wq;
static atomic_t hbm_task_task_wakeup = ATOMIC_INIT(0);
int ramless_dc_wait = 0;
int hbm_sof_flag = 0;
extern bool oplus_display_aod_ramless_support;

#define TWO_TE_TIME_MS 42
struct task_struct *fpd_notify_task;
wait_queue_head_t fpd_notify_task_wq;
atomic_t fpd_task_task_wakeup = ATOMIC_INIT(0);
unsigned long fpd_hbm_time = 0;
unsigned long fpd_send_uiready_time = 0;
unsigned long HBM_mode = 0;
unsigned long HBM_pre_mode = 0;
struct timespec hbm_time_on;
struct timespec hbm_time_off;
long hbm_on_start = 0;
extern bool oplus_fp_notify_up_delay;
extern bool oplus_fp_notify_down_delay;

//#ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT
static int fpd_notify_worker_kthread(void *data)
{
	 int ret = 0;
	 while (1) {
		 ret = wait_event_interruptible(fpd_notify_task_wq, atomic_read(&fpd_task_task_wakeup));
		 atomic_set(&fpd_task_task_wakeup, 0);

		 if (doze_rec_fpd || ds_rec_fpd) {
			 fpd_send_uiready_time = jiffies;
			 if (jiffies_to_msecs(fpd_send_uiready_time - fpd_hbm_time) <= TWO_TE_TIME_MS) {
				 //timeout is 250 jiffies,1s
				 dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, HZ);
				 pr_info("[fpdnotify] wait hbm te done\n");
			 }
		 }
		 fingerprint_send_notify(NULL, 1);
		 doze_rec_fpd = false;
		 ds_rec_fpd = false;

		 if (kthread_should_stop())
			 break;
	 }
	 return 0;
}

static void fpd_notify_init(void)
{
	/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
	if (oplus_display_aod_ramless_support) {
		if (!primary_display_is_video_mode()) {
			return;
		}
	}
	/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */
	 if (!fpd_notify_task) {
		 fpd_notify_task = kthread_create(fpd_notify_worker_kthread, NULL,"FPD_NOTIFY");
		 init_waitqueue_head(&fpd_notify_task_wq);
		 wake_up_process(fpd_notify_task);
	 }
	 pr_info("[fpdnotify] init\n");
}

void fpd_notify(void)
{
	/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
	if (oplus_display_aod_ramless_support) {
		if (!primary_display_is_video_mode()) {
			return;
		}
	}
	/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */
	 if (fpd_notify_task != NULL) {
		 atomic_set(&fpd_task_task_wakeup, 1);
		 wake_up_interruptible(&fpd_notify_task_wq);
		 pr_info("[fpdnotify] notify\n");
	 } else {
		 pr_info("[fpdnotify] notify is NULL\n");
	 }
}

 int notify_display_fpd(bool mode) {
	 if (oplus_display_fppress_support) {
		 if (mode) {
			 /* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
			 if (oplus_display_aod_ramless_support) {
				return 0;
			 }
			 /* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */
			 if (primary_display_get_power_mode_nolock() == DOZE_SUSPEND) {
				 ds_rec_fpd = true;
				 pr_info("[fpdnotify] ds rec fpd\n");
			 } else if (primary_display_get_power_mode_nolock() == DOZE) {
				 doze_rec_fpd = true;
				 pr_info("[fpdnotify] doze rec fpd\n");
				 _primary_path_switch_dst_lock();
				 _primary_path_lock(__func__);
				 primary_display_set_lcm_hbm(true);
				 _primary_path_unlock(__func__);
				 _primary_path_switch_dst_unlock();
				 fpd_hbm_time = jiffies;
			 }
		 } else {
			 doze_rec_fpd = false;
			 ds_rec_fpd = false;
			 pr_info("[fpdnotify] fp up reset flag\n");
		 }
	 }
	 return 0;
 }

 /*
 * add for fingerprint notify frigger
 */
 bool need_update_fpd_fence(struct disp_frame_cfg_t *cfg)
 {
	 bool ret = 0;

	 if (oplus_fp_notify_down_delay && ((cfg->hbm_en & 0x2) > 0)) {
		/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
		if (oplus_display_aod_ramless_support) {
			if (!primary_display_is_video_mode()) {
				pr_info("[fpdnotify] %s: ramless aod cmd returned\n", __func__);
				return ret;
			}
		}
		/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */
		 oplus_fp_notify_down_delay = false;
		 fpd_notify_init();
		 if (cfg->present_fence_idx != (unsigned int)-1) {
			 ret = 1;
		 }
	 }
	 return ret;
 }

 /* fpd_notify is called when target frame frame done */
 void fpd_notify_check_trig(void)
 {
	 static unsigned int last_fpd_fence;
	 unsigned int cur_fpd_fence;

	 if (fpd_notify_task == NULL)
		 return;

	/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
	if (oplus_display_aod_ramless_support) {
		if (!primary_display_is_video_mode()) {
			return;
		}
	}
	/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */

	if (oplus_display_aod_ramless_support) {
		cmdqBackupReadSlot(pgc->fpd_fence, 0, &cur_fpd_fence);
	} else {
		cmdqBackupReadSlot(pgc->fpd_fence, 1, &cur_fpd_fence);
	}

	 if (cur_fpd_fence > last_fpd_fence) {
		 pr_info("[fpdnotify] %s cur_fpd_fence:%u last_fpd_fence:%u\n",__func__,cur_fpd_fence,last_fpd_fence);
		 last_fpd_fence = cur_fpd_fence;
		if (oplus_display_aod_ramless_support) {
			hbm_sof_flag = 1;
		} else {
			fpd_notify();
		}
	 }
 }

int oplus_display_panel_set_hbm(void *buf)
{
	int ret;
	unsigned int *HBM = buf;

	HBM_pre_mode = HBM_mode;
	//ret = kstrtoul(buf, 10, &HBM_mode);
	HBM_mode = (*HBM);
	printk("%s HBM_mode=%ld\n", __func__, (*HBM));
	ret = primary_display_set_hbm_mode((unsigned int)HBM_mode);
	if (HBM_mode == 1) {
		mdelay(80);
		printk("%s delay done\n", __func__);
	}
	if (HBM_mode == 8) {
	 	get_monotonic_boottime(&hbm_time_on);
	 	hbm_on_start = hbm_time_on.tv_sec;
	}
	return ret;
}

int oplus_display_panel_get_hbm(void *buf)
{
	unsigned int *HBM = buf;
	printk("%s HBM_mode=%ld\n", __func__, HBM_mode);
	//sprintf(buf, "%ld\n", HBM_mode);
	(*HBM) = HBM_mode;
	return 0;
}

int oplus_display_panel_set_finger_print(void *buf)
{
	uint8_t fingerprint_op_mode = 0x0;
	uint32_t *finger_print = buf;
	if (oplus_display_fppress_support) {
		/* will ignoring event during panel off situation. */
		if (flag_lcd_off)
		{
			pr_err("%s panel in off state, ignoring event.\n", __func__);
			return 0;
		}
		fingerprint_op_mode = (uint8_t)(*finger_print);
		if (fingerprint_op_mode == 1) {
			oplus_fp_notify_down_delay = true;
		} else {
			oplus_fp_notify_up_delay = true;
			ds_rec_fpd = false;
			doze_rec_fpd = false;
		}
		pr_info("%s receive uiready %d\n", __func__,fingerprint_op_mode);
	}
	return 0;
}

static int disp_lcm_set_hbm_wait_ramless(bool wait, struct disp_lcm_handle *plcm, void *qhandle)
{
	if (!_is_lcm_inited(plcm)) {
		DISP_PR_ERR("lcm_drv is null\n");
		return -1;
	}

	if (!plcm->drv->set_hbm_wait_ramless) {
		DISP_PR_ERR("FATAL ERROR, lcm_drv->set_hbm_wait_ramless is null\n");
		return -1;
	}

	plcm->drv->set_hbm_wait_ramless(wait, qhandle);

	return 0;
}

int primary_display_set_hbm_wait_ramless(bool en)
{
	int ret = 0;
	struct cmdqRecStruct *qhandle_wait = NULL;

	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &qhandle_wait);
	if (ret) {
		DISPMSG("%s:failed to create cmdq handle\n", __func__);
		return -1;
	}

	if (!primary_display_is_video_mode()) {
		cmdqRecReset(qhandle_wait);
		cmdqRecWait(qhandle_wait, CMDQ_SYNC_TOKEN_CABC_EOF);
		oplus_cmdq_handle_clear_dirty(qhandle_wait);

		_cmdq_insert_wait_frame_done_token_mira(qhandle_wait);
		disp_lcm_set_hbm_wait_ramless(en, pgc->plcm, qhandle_wait);

		cmdqRecSetEventToken(qhandle_wait, CMDQ_SYNC_TOKEN_CABC_EOF);
		oplus_cmdq_flush_config_handle_mira(qhandle_wait, 1);
	} else {
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl, MMPROFILE_FLAG_PULSE, 1, 2);
		disp_lcm_set_hbm_wait_ramless(en, pgc->plcm, qhandle_wait);

		oplus_cmdq_flush_config_handle_mira(qhandle_wait, 1);
		DISPMSG("[BL]qhandle_hbm ret=%d\n", ret);
	}

	cmdqRecDestroy(qhandle_wait);
	qhandle_wait = NULL;

	return ret;
}

static int hbm_notify_worker_kthread(void *data)
{
	int ret = 0;
	while (1) {
		ret = wait_event_interruptible(hbm_notify_task_wq, atomic_read(&hbm_task_task_wakeup));
		atomic_set(&hbm_task_task_wakeup, 0);
		primary_display_set_hbm_wait_ramless(false);
		mdelay(5);
		if (kthread_should_stop()) {
			break;
		}
	}
	return 0;
}

void hbm_notify_init(void)
{
	if (!hbm_notify_task) {
		hbm_notify_task = kthread_create(hbm_notify_worker_kthread, NULL, "hbm_NOTIFY");
		init_waitqueue_head(&hbm_notify_task_wq);
		wake_up_process(hbm_notify_task);
		pr_info("[hbmnotify] init CREATE\n");
	}
	pr_info("[hbmnotify] init\n");
}

void hbm_notify(void)
{
	if (hbm_notify_task != NULL) {
		atomic_set(&hbm_task_task_wakeup, 1);
		wake_up_interruptible(&hbm_notify_task_wq);
		pr_info("[hbmnotify] notify\n");
	} else {
		pr_info("[hbmnotify] notify is NULL\n");
	}
}

int mtk_disp_lcm_set_hbm(bool en, struct disp_lcm_handle *plcm, void *qhandle)
{
	if (!_is_lcm_inited(plcm)) {
		DISP_PR_ERR("lcm_drv is null\n");
		return -1;
	}
	/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
	if (oplus_display_aod_ramless_support) {
		if (!disp_lcm_is_video_mode(plcm)) {
			DISPCHECK("%s disp is cmd Ramless set hbm [%d]\n", __func__, en);
		}
	}
	/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */

	if (!plcm->drv->set_hbm_cmdq) {
		DISP_PR_ERR("FATAL ERROR, lcm_drv->set_hbm_cmdq is null\n");
		return -1;
	}

	plcm->drv->set_hbm_cmdq(en, qhandle);

	return 0;
}
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
