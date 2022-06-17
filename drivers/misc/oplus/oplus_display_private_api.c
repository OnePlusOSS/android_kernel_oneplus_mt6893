/***************************************************************
** Copyright (C),  2018,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_private_api.h
** Description : oplus display private api implement
** Version : 1.0
** Date : 2018/03/20
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Hu.Jie          2018/03/20        1.0           Build this moudle
**   Guo.Ling        2018/10/11        1.1           Modify for SDM660
**   Guo.Ling        2018/11/27        1.2           Modify for mt6779
**   Lin.Hao         2019/11/01        1.3           Modify for MT6779_Q
******************************************************************/
#define pr_fmt(fmt) "oplus_api: %s: " fmt, __func__
#include "oplus_display_private_api.h"
#include "disp_drv_log.h"
#include <linux/oplus_mm_kevent_fb.h>
#include <linux/fb.h>
#include <linux/time.h>
#include <linux/timekeeping.h>
#include <linux/of_fdt.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <soc/oplus/oplus_project.h>
#include "primary_display.h"
#include "display_panel/oplus_display_panel.h"

/*
 * we will create a sysfs which called /sys/kernel/oplus_display,
 * In that directory, oplus display private api can be called
 */
#define LCM_ID_READ_LEN 1
#define LCM_REG_READ_LEN 16
#define PANEL_SERIAL_NUM_REG 0xA1
#define PANEL_SERIAL_NUM_REG_CONTINUE 0xA8
#define PANEL_REG_READ_LEN   16

bool oplus_display_panelid_support;
bool oplus_display_panelnum_support;
bool oplus_display_panelnum_continue_support;
#if 0
bool oplus_display_ffl_support;
bool oplus_display_sau_support;
#endif
unsigned long oplus_display_brightness = 0;
unsigned int oplus_set_brightness = 0;
unsigned int aod_light_mode = 0;
bool oplus_flag_lcd_off = false;
unsigned long oplus_silence_mode = 0;
unsigned int oplus_fp_silence_mode = 0;
int esd_status = 0;
unsigned long seed_mode = 0;
bool oplus_display_cabc_support; /* if cabc is supported(cmd sent without cmdq handle) */
bool oplus_display_cabc_cmdq_support; /* if cabc is supported(cmd sent with cmdq handle) */
bool oplus_display_mipi_before_init; /* if frame data must be sent before lcd init(ic:TM TD4330) */
bool oplus_display_lcm_id_check_support; /* whether lcm id is needed to be checked or not(only himax ic of 18311&18011 for now) */
bool oplus_display_aod_support; /* if aod feature is supported */
bool oplus_display_hbm_support;
bool oplus_display_fppress_support;
/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
bool oplus_display_aod_ramless_support; /* if ramless aod feature is supported */
bool oplus_display_aodlight_support; /* if aod light mode( high-light or low-light) option is supported */
/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */
#if 0
bool oplus_display_aod_legacy_support; /* if aod feature is supported(for old project like 17197 which aodlight mode and fppress is not supported on) */
#endif
#ifdef OPLUS_FEATURE_MULTIBITS_BL
bool oplus_display_tenbits_support;
bool oplus_display_elevenbits_support;
bool oplus_display_twelvebits_support;
#endif /* OPLUS_FEATURE_MULTIBITS_BL */
bool oplus_display_backlight_ic_support; /* if backlight ic like lm3697 or ktd3136 is supported */
bool oplus_display_bl_set_on_lcd_esd_recovery;
/* recover backlight after esd recovery (cmd sent without a cmdq handle) */
bool oplus_display_bl_set_cmdq_on_lcd_esd_recovery;
/* recover backlight after esd recovery (cmd sent with a cmdq handle) */
u32 oplus_display_esd_try_count = 0; /* customize esd consecutive recover max time until success (only 18531&18561&18161 for now)*/
#if 0
bool oplus_display_esd_check_dsi_rx_data1; /* if need to read DSI_RX_DATA1 reg on esd checking(only himax ic of 18311&18011 for now)*/
/* tp fw must be reloaded after lcd esd recovery if value is 1(18311&18011&18611) */
u32 oplus_display_tp_fw_reload_on_lcd_esd_recovery = 0;
/* need recover backlight after esd recovery if value is 1(cmd sent withous cmdq handle) */
u32 oplus_display_dyn_data_rate = 0;
u32 oplus_display_dyn_dsi_hbp = 0;
u32 oplus_display_lcd_height = 0; /* for projects which share the same lcd driver file but have different resolution height */

extern void ffl_set_enable(unsigned int enable);
#endif
extern int __attribute((weak)) primary_display_read_lcm_id(char cmd, uint32_t *buf, int num) { return 0;};
extern int __attribute((weak)) primary_display_read_serial(char addr, uint64_t *buf, int lenth) { return 1;};
extern int __attribute((weak)) panel_serial_number_read(char cmd, uint64_t *buf, int num) { return 0;};
extern int __attribute((weak)) primary_display_set_cabc_mode(unsigned int level) { return 0;};
extern int  __attribute((weak)) primary_display_set_hbm_mode(unsigned int level) {return 0;};
extern int __attribute((weak)) primary_display_setbacklight(unsigned int level) {return 0;};
extern void __attribute((weak)) _primary_path_switch_dst_lock(void) {return;};
extern void __attribute((weak)) _primary_path_switch_dst_unlock(void) {return;};
extern void __attribute((weak)) _primary_path_lock(const char *caller) {return;};
extern void __attribute((weak)) _primary_path_unlock(const char *caller) {return;};
extern int __attribute((weak)) primary_display_aod_backlight(int level) {return 0;};
extern bool __attribute((weak)) primary_display_get_fp_hbm_state(void) {return 0;};
extern unsigned int __attribute((weak)) delay_uiready = 0;
#if defined(CONFIG_MACH_MT6785)
extern enum DISP_HELPER_STAGE disp_helper_get_stage(void);
extern const char *disp_helper_stage_spy(void);
extern void oplus_delayed_trigger_kick_set(int params);
extern void oplus_cmdq_flush_config_handle_mira(void *handle, int blocking);
extern void oplus_cmdq_handle_clear_dirty(struct cmdqRecStruct *cmdq_handle);
extern int _is_lcm_inited(struct disp_lcm_handle *plcm);

static int disp_lcm_set_safe_mode(struct disp_lcm_handle *plcm, void *handle, unsigned int mode)
{
	DISPFUNC();
	if (_is_lcm_inited(plcm)) {
		if (plcm->drv->set_safe_mode) {
			plcm->drv->set_safe_mode(handle, mode);
		} else {
			DISP_PR_ERR("FATAL ERROR, lcm_drv->set_safe_mode is null\n");
			return -1;
		}
		return 0;
	}
	DISP_PR_ERR("lcm_drv is null\n");
	return -1;
}

/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
struct aod_area {
	bool enable;
	int x;
	int y;
	int w;
	int h;
	int color;
	int bitdepth;
	int mono;
	int gray;
};

#define RAMLESS_AOD_AREA_NUM		6
#define RAMLESS_AOD_PAYLOAD_SIZE	100
struct aod_area oplus_aod_area[RAMLESS_AOD_AREA_NUM];
unsigned char aod_area_cmd[RAMLESS_AOD_PAYLOAD_SIZE];
extern void print_hex_dump(const char *level, const char *prefix_str, int prefix_type, int rowsize, int groupsize, const void *buf, size_t len, bool ascii);

static ssize_t oplus_display_set_aod_area(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count) {
	char *bufp = (char *)buf;
	char *token;
	int i, cnt = 0;

	if (!oplus_display_aod_ramless_support) {
		pr_info("ramless true aod is disable\n");
		return count;
	}

	memset(oplus_aod_area, 0, sizeof(struct aod_area) * RAMLESS_AOD_AREA_NUM);

	pr_err(" %s \n", __func__);
	while ((token = strsep(&bufp, ":")) != NULL) {
		struct aod_area *area = &oplus_aod_area[cnt];
		if (!*token) {
			continue;
		}

		sscanf(token, "%d %d %d %d %d %d %d %d",
			&area->x, &area->y, &area->w, &area->h,
			&area->color, &area->bitdepth, &area->mono, &area->gray);
		pr_err("aod area: %s %d rect[%dx%d-%dx%d]-%d-%d-%d-%x\n", __func__, __LINE__,
			area->x, area->y, area->w, area->h,
			area->color, area->bitdepth, area->mono, area->gray);
		area->enable = true;
		cnt++;
	}


	memset(aod_area_cmd, 0, RAMLESS_AOD_PAYLOAD_SIZE);

	for (i = 0; i < RAMLESS_AOD_AREA_NUM; i++) {
		struct aod_area *area = &oplus_aod_area[i];

		aod_area_cmd[0] |= (!!area->enable) << (RAMLESS_AOD_AREA_NUM - i - 1);
		if (area->enable) {
			int h_start = area->x;
			int h_block = area->w / 100;
			int v_start = area->y;
			int v_end = area->y + area->h;
			int off = i * 5;

			/* Rect Setting */
			aod_area_cmd[1 + off] = h_start >> 4;
			aod_area_cmd[2 + off] = ((h_start & 0xf) << 4) | (h_block & 0xf);
			aod_area_cmd[3 + off] = v_start >> 4;
			aod_area_cmd[4 + off] = ((v_start & 0xf) << 4) | ((v_end >> 8) & 0xf);
			aod_area_cmd[5 + off] = v_end & 0xff;

			/* Mono Setting */
			#define SET_MONO_SEL(index, shift) \
				if (i == index) {\
					aod_area_cmd[31] |= area->mono << shift;\
				}

			SET_MONO_SEL(0, 6);
			SET_MONO_SEL(1, 5);
			SET_MONO_SEL(2, 4);
			SET_MONO_SEL(3, 2);
			SET_MONO_SEL(4, 1);
			SET_MONO_SEL(5, 0);
			#undef SET_MONO_SEL

			/* Depth Setting */
			if (i < 4) {
				aod_area_cmd[32] |= (area->bitdepth & 0x3) << ((3 - i) * 2);
			} else if (i == 4) {
				aod_area_cmd[33] |= (area->bitdepth & 0x3) << 6;
			} else if (i == 5) {
				aod_area_cmd[33] |= (area->bitdepth & 0x3) << 4;
			}
			/* Color Setting */
			#define SET_COLOR_SEL(index, reg, shift) \
				if (i == index) {\
					aod_area_cmd[reg] |= (area->color & 0x7) << shift;\
				}
			SET_COLOR_SEL(0, 34, 4);
			SET_COLOR_SEL(1, 34, 0);
			SET_COLOR_SEL(2, 35, 4);
			SET_COLOR_SEL(3, 35, 0);
			SET_COLOR_SEL(4, 36, 4);
			SET_COLOR_SEL(5, 36, 0);
			#undef SET_COLOR_SEL
			/* Area Gray Setting */
			aod_area_cmd[37 + i] = area->gray & 0xff;
		}
	}
	aod_area_cmd[43] = 0x00;

	/* rc = mipi_dsi_dcs_write(mipi_device, 0x81, payload, 43); */
	/* pr_err(" %s payload = %s\n", __func__, aod_area_cmd); */
	/*
	for(i=0;i<44;i++)
	{
		pr_err(" %s payload[%d] = 0x%x\n", __func__, i,aod_area_cmd[i]);
	}
	*/
	/*
	_primary_path_switch_dst_lock();
	_primary_path_lock(__func__);
	primary_display_set_aod_area(aod_area_cmd);
	_primary_path_unlock(__func__);
	_primary_path_switch_dst_unlock();
	print_hex_dump(KERN_INFO, "aod area setting raw data: ", DUMP_PREFIX_ADDRESS, 16, 1, aod_area_cmd, sizeof(aod_area_cmd), true);
	*/
	return count;
}
int oplus_display_panel_set_aod_area(void *buf)
{
	struct panel_aod_area_para *para = (struct panel_aod_area_para *)buf;
	int i, cnt = 0;

	if (!oplus_display_aod_ramless_support) {
		pr_info("ramless true aod is disable\n");
		return -1;
	}

	if (para->size > RAMLESS_AOD_AREA_NUM) {
		pr_err("aod area size is invalid, size=%d\n", para->size);
		return -1;
	}

	memset(oplus_aod_area, 0, sizeof(struct aod_area) * RAMLESS_AOD_AREA_NUM);

	pr_err(" %s \n", __func__);

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

	memset(aod_area_cmd, 0, RAMLESS_AOD_PAYLOAD_SIZE);

	for (i = 0; i < RAMLESS_AOD_AREA_NUM; i++) {
		struct aod_area *area = &oplus_aod_area[i];

		aod_area_cmd[0] |= (!!area->enable) << (RAMLESS_AOD_AREA_NUM - i - 1);
		if (area->enable) {
			int h_start = area->x;
			int h_block = area->w / 100;
			int v_start = area->y;
			int v_end = area->y + area->h;
			int off = i * 5;

			/* Rect Setting */
			aod_area_cmd[1 + off] = h_start >> 4;
			aod_area_cmd[2 + off] = ((h_start & 0xf) << 4) | (h_block & 0xf);
			aod_area_cmd[3 + off] = v_start >> 4;
			aod_area_cmd[4 + off] = ((v_start & 0xf) << 4) | ((v_end >> 8) & 0xf);
			aod_area_cmd[5 + off] = v_end & 0xff;

			/* Mono Setting */
			#define SET_MONO_SEL(index, shift) \
				if (i == index) {\
					aod_area_cmd[31] |= area->mono << shift;\
				}

			SET_MONO_SEL(0, 6);
			SET_MONO_SEL(1, 5);
			SET_MONO_SEL(2, 4);
			SET_MONO_SEL(3, 2);
			SET_MONO_SEL(4, 1);
			SET_MONO_SEL(5, 0);
			#undef SET_MONO_SEL

			/* Depth Setting */
			if (i < 4) {
				aod_area_cmd[32] |= (area->bitdepth & 0x3) << ((3 - i) * 2);
			} else if (i == 4) {
				aod_area_cmd[33] |= (area->bitdepth & 0x3) << 6;
			} else if (i == 5) {
				aod_area_cmd[33] |= (area->bitdepth & 0x3) << 4;
			}
			/* Color Setting */
			#define SET_COLOR_SEL(index, reg, shift) \
				if (i == index) {\
					aod_area_cmd[reg] |= (area->color & 0x7) << shift;\
				}
			SET_COLOR_SEL(0, 34, 4);
			SET_COLOR_SEL(1, 34, 0);
			SET_COLOR_SEL(2, 35, 4);
			SET_COLOR_SEL(3, 35, 0);
			SET_COLOR_SEL(4, 36, 4);
			SET_COLOR_SEL(5, 36, 0);
			#undef SET_COLOR_SEL
			/* Area Gray Setting */
			aod_area_cmd[37 + i] = area->gray & 0xff;
		}
	}
	aod_area_cmd[43] = 0x00;

	/* rc = mipi_dsi_dcs_write(mipi_device, 0x81, payload, 43); */
	/* pr_err(" %s payload = %s\n", __func__, aod_area_cmd); */
	/*
	for(i=0;i<44;i++)
	{
		pr_err(" %s payload[%d] = 0x%x\n", __func__, i,aod_area_cmd[i]);
	}
	*/
	/*
	_primary_path_switch_dst_lock();
	_primary_path_lock(__func__);
	primary_display_set_aod_area(aod_area_cmd);
	_primary_path_unlock(__func__);
	_primary_path_switch_dst_unlock();
	print_hex_dump(KERN_INFO, "aod area setting raw data: ", DUMP_PREFIX_ADDRESS, 16, 1, aod_area_cmd, sizeof(aod_area_cmd), true);
	*/
	return 0;
}
/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */

static int _set_safe_mode_by_cmdq(unsigned int level)
{
	int ret = 0;

	struct cmdqRecStruct *cmdq_handle_SAFE_mode = NULL;

	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_cmd,
		MMPROFILE_FLAG_PULSE, 1, 1);

	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle_SAFE_mode);

	if (ret != 0) {
		DISPCHECK("fail to create primary cmdq handle for SAFE mode\n");
		return -1;
	}

	if (primary_display_is_video_mode()) {
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
			MMPROFILE_FLAG_PULSE, 1, 2);
		cmdqRecReset(cmdq_handle_SAFE_mode);

	    ret = disp_lcm_set_safe_mode(pgc->plcm, cmdq_handle_SAFE_mode, level);\

		oplus_cmdq_flush_config_handle_mira(cmdq_handle_SAFE_mode, 1);
		DISPCHECK("[BL]_set_safe_mode_by_cmdq ret=%d\n", ret);
	} else {
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
			MMPROFILE_FLAG_PULSE, 1, 3);
		cmdqRecReset(cmdq_handle_SAFE_mode);
		cmdqRecWait(cmdq_handle_SAFE_mode, CMDQ_SYNC_TOKEN_CABC_EOF);
		oplus_cmdq_handle_clear_dirty(cmdq_handle_SAFE_mode);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_SAFE_mode);

	    ret = disp_lcm_set_safe_mode(pgc->plcm, cmdq_handle_SAFE_mode, level);

		cmdqRecSetEventToken(cmdq_handle_SAFE_mode, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		cmdqRecSetEventToken(cmdq_handle_SAFE_mode, CMDQ_SYNC_TOKEN_CABC_EOF);
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
			MMPROFILE_FLAG_PULSE, 1, 4);
		oplus_cmdq_flush_config_handle_mira(cmdq_handle_SAFE_mode, 1);
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
			MMPROFILE_FLAG_PULSE, 1, 6);
		DISPCHECK("[BL]_set_safe_mode_by_cmdq ret=%d\n", ret);
	}
	cmdqRecDestroy(cmdq_handle_SAFE_mode);
	cmdq_handle_SAFE_mode = NULL;
	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
		MMPROFILE_FLAG_PULSE, 1, 5);

	return ret;
}

int primary_display_set_safe_mode(unsigned int level)
{
	int ret = 0;

	if (!oplus_display_aod_ramless_support) {
		pr_err("panel is not ramless oled unsupported!!\n");
		return 0;
	}

	if (oplus_flag_lcd_off) {
		pr_err("lcd is off,don't allow to set hbm\n");
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
		DISP_PR_ERR("Sleep State set backlight invald\n");
	} else {
		primary_display_idlemgr_kick((char *)__func__, 0);
		if (primary_display_cmdq_enabled()) {
			if (primary_display_is_video_mode()) {
				mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
					       MMPROFILE_FLAG_PULSE, 0, 7);
				_set_safe_mode_by_cmdq(level);
			} else {
				_set_safe_mode_by_cmdq(level);
			}
			/* atomic_set(&delayed_trigger_kick, 1); */
			oplus_delayed_trigger_kick_set(1);
		}
	}
	mdelay(20);
	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
		MMPROFILE_FLAG_END, 0, 0);
	return ret;
}
#endif

unsigned int oplus_display_normal_max_brightness = 0;
int oplus_mtkfb_custom_data_init(struct platform_device *pdev)
{
	int of_ret = 0;

	if (!pdev) {
		pr_err("%s, pdev is null\n", __func__);
		return -1;
	}

	#ifdef OPLUS_FEATURE_MULTIBITS_BL
	oplus_display_tenbits_support = of_property_read_bool(pdev->dev.of_node, "oplus_display_tenbits_support");
	oplus_display_elevenbits_support = of_property_read_bool(pdev->dev.of_node, "oplus_display_elevenbits_support");
	oplus_display_twelvebits_support = of_property_read_bool(pdev->dev.of_node, "oplus_display_twelvebits_support");
	#endif /* OPLUS_FEATURE_MULTIBITS_BL */
	oplus_display_backlight_ic_support = of_property_read_bool(pdev->dev.of_node, "oplus_display_backlight_ic_support");
	oplus_display_cabc_support = of_property_read_bool(pdev->dev.of_node, "oplus_display_cabc_support");
	oplus_display_cabc_cmdq_support = of_property_read_bool(pdev->dev.of_node, "oplus_display_cabc_cmdq_support");
	oplus_display_bl_set_on_lcd_esd_recovery = of_property_read_bool(pdev->dev.of_node, "oplus_display_bl_set_on_lcd_esd_recovery");
	oplus_display_bl_set_cmdq_on_lcd_esd_recovery = of_property_read_bool(pdev->dev.of_node, "oplus_display_bl_set_cmdq_on_lcd_esd_recovery");
	oplus_display_mipi_before_init = of_property_read_bool(pdev->dev.of_node, "oplus_display_mipi_before_init");
	oplus_display_lcm_id_check_support = of_property_read_bool(pdev->dev.of_node, "oplus_display_lcm_id_check_support");
	oplus_display_aod_support = of_property_read_bool(pdev->dev.of_node, "oplus_display_aod_support");
	oplus_display_hbm_support = of_property_read_bool(pdev->dev.of_node, "oplus_display_hbm_support");
	oplus_display_fppress_support = of_property_read_bool(pdev->dev.of_node, "oplus_display_fppress_support");
	oplus_display_aod_ramless_support = of_property_read_bool(pdev->dev.of_node, "oplus_display_aod_ramless_support");
	oplus_display_aodlight_support = of_property_read_bool(pdev->dev.of_node, "oplus_display_aodlight_support");
	oplus_display_panelid_support = of_property_read_bool(pdev->dev.of_node, "oplus_display_panelid_support");
	oplus_display_panelnum_support = of_property_read_bool(pdev->dev.of_node, "oplus_display_panelnum_support");
	oplus_display_panelnum_continue_support = of_property_read_bool(pdev->dev.of_node, "oplus_display_panelnum_continue_support");

	of_ret = of_property_read_u32(pdev->dev.of_node, "oplus_display_esd_try_count", &oplus_display_esd_try_count);
	if (!of_ret)
		dev_err(&pdev->dev, "read property oplus_display_esd_try_count failed.");
	else
		DISPMSG("%s:oplus_display_esd_try_count=%u\n", __func__, oplus_display_esd_try_count);

	of_ret = of_property_read_u32(pdev->dev.of_node, "oplus_display_normal_max_brightness", &oplus_display_normal_max_brightness);
	if (!of_ret)
		dev_err(&pdev->dev, "read property oplus_display_normal_max_brightness failed.");
	else
		DISPMSG("%s:oplus_display_normal_max_brightness=%u\n", __func__, oplus_display_normal_max_brightness);

	return of_ret;
}

static ssize_t oplus_display_get_brightness(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	if (oplus_display_brightness > LED_FULL || oplus_display_brightness < LED_OFF) {
		oplus_display_brightness = LED_OFF;
	}
	//printk(KERN_INFO "oplus_display_get_brightness = %ld\n",oplus_display_brightness);
	return sprintf(buf, "%ld\n", oplus_display_brightness);
}

int oplus_display_panel_set_brightness(void *buf)
{
	unsigned int *oplus_brightness = buf;

	oplus_set_brightness = (*oplus_brightness);

	pr_info("brightness = %d\n", oplus_set_brightness);

	if (oplus_set_brightness > LED_FULL || oplus_set_brightness < LED_OFF) {
		return -1;
	}

	_primary_path_switch_dst_lock();
	_primary_path_lock(__func__);
	primary_display_setbacklight_nolock(oplus_set_brightness);
	_primary_path_unlock(__func__);
	_primary_path_switch_dst_unlock();

	return 0;
}

int oplus_display_panel_get_brightness(void *buf)
{
	unsigned int *oplus_brightness = buf;

	if (oplus_display_brightness > LED_FULL || oplus_display_brightness < LED_OFF) {
		oplus_display_brightness = LED_OFF;
	}

	*oplus_brightness = oplus_display_brightness;

	return 0;
}

static ssize_t oplus_display_set_brightness(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t num)
{
	int ret;

	ret = kstrtouint(buf, 10, &oplus_set_brightness);

	printk("%s %d\n", __func__, oplus_set_brightness);

	if (oplus_set_brightness > LED_FULL || oplus_set_brightness < LED_OFF) {
		return num;
	}

	_primary_path_switch_dst_lock();
	_primary_path_lock(__func__);
	primary_display_setbacklight(oplus_set_brightness);
	_primary_path_unlock(__func__);
	_primary_path_switch_dst_unlock();

	return num;
}

static ssize_t oplus_display_get_max_brightness(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int max_brightness = 0;

	if (!oplus_display_normal_max_brightness)
		max_brightness = LED_FULL;
	else
		max_brightness = oplus_display_normal_max_brightness;

	//printk(KERN_INFO "oplus_display_get_max_brightness = %d\n",LED_FULL);
	return sprintf(buf, "%u\n", max_brightness);
}

int oplus_display_panel_get_max_brightness(void *buf)
{
	uint32_t *max_brightness = buf;

	if (!max_brightness) {
		pr_info("invalid argument:NULL\n");
		return -1;
	}

	if (!oplus_display_normal_max_brightness)
		(*max_brightness) = LED_FULL;
	else
		(*max_brightness) = oplus_display_normal_max_brightness;

	pr_info("value is %d", *max_brightness);
	return 0;
}

static ssize_t oplus_get_aod_light_mode(struct device *dev,
		struct device_attribute *attr, char *buf) {

	printk(KERN_INFO "oplus_get_aod_light_mode = %d\n",aod_light_mode);

	return sprintf(buf, "%d\n", aod_light_mode);
}

static ssize_t oplus_set_aod_light_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count) {
	unsigned int temp_save = 0;
	int ret = 0;

	pr_info("%s, oplus_display_aodlight_support = %d\n", __func__, oplus_display_aodlight_support);

	if (oplus_display_aodlight_support) {
		ret = kstrtouint(buf, 10, &temp_save);

		if (primary_display_get_fp_hbm_state()) {
			printk(KERN_INFO "oplus_set_aod_light_mode = %d return on hbm\n",temp_save);
			return count;
		}
		aod_light_mode = temp_save;
		ret = primary_display_aod_backlight(aod_light_mode);
		printk(KERN_INFO "oplus_set_aod_light_mode = %d\n",temp_save);
	}

	return count;
}

int oplus_panel_set_aod_light_mode(void *buf)
{
	unsigned int *temp_save = buf;
	int ret = 0;

	pr_info("aodlight_support = %d\n", oplus_display_aodlight_support);

	if (oplus_display_aodlight_support) {
		if (primary_display_get_fp_hbm_state()) {
			pr_info("oplus_set_aod_light_mode = %d return on hbm\n", (*temp_save));
			return 0;
		}
		aod_light_mode = (*temp_save);
		ret = primary_display_aod_backlight(aod_light_mode);
		pr_info("oplus_panel_set_aod_light_mode = %d\n", (*temp_save));
	}

	return 0;
}

int oplus_panel_get_aod_light_mode(void *buf)
{
	unsigned int *aod_mode = buf;
	(*aod_mode) = aod_light_mode;
	pr_info("oplus_panel_get_aod_light_mode = %d\n", aod_light_mode);
	return 0;
}


int oplus_panel_alpha = 0;
int oplus_underbrightness_alpha = 0;
int alpha_save = 0;
struct ba {
	u32 brightness;
	u32 alpha;
};

struct ba brightness_alpha_lut[] = {
	{0, 0xff},
	{1, 0xee},
	{2, 0xe8},
	{3, 0xe6},
	{4, 0xe5},
	{6, 0xe4},
	{10, 0xe0},
	{20, 0xd5},
	{30, 0xce},
	{45, 0xc6},
	{70, 0xb7},
	{100, 0xad},
	{150, 0xa0},
	{227, 0x8a},
	{300, 0x80},
	{400, 0x6e},
	{500, 0x5b},
	{600, 0x50},
	{800, 0x38},
	{1023, 0x18},
};

static int interpolate(int x, int xa, int xb, int ya, int yb)
{
	int bf, factor, plus;
	int sub = 0;

	bf = 2 * (yb - ya) * (x - xa) / (xb - xa);
	factor = bf / 2;
	plus = bf % 2;
	if ((xa - xb) && (yb - ya))
		sub = 2 * (x - xa) * (x - xb) / (yb - ya) / (xa - xb);

	return ya + factor + plus + sub;
}

int bl_to_alpha(int brightness)
{
	int level = ARRAY_SIZE(brightness_alpha_lut);
	int i = 0;
	int alpha;

	for (i = 0; i < ARRAY_SIZE(brightness_alpha_lut); i++){
		if (brightness_alpha_lut[i].brightness >= brightness)
			break;
	}

	if (i == 0)
		alpha = brightness_alpha_lut[0].alpha;
	else if (i == level)
		alpha = brightness_alpha_lut[level - 1].alpha;
	else
		alpha = interpolate(brightness,
			brightness_alpha_lut[i-1].brightness,
			brightness_alpha_lut[i].brightness,
			brightness_alpha_lut[i-1].alpha,
			brightness_alpha_lut[i].alpha);
	return alpha;
}

int brightness_to_alpha(int brightness)
{
	int alpha;

	if (brightness <= 3)
		return alpha_save;

	alpha = bl_to_alpha(brightness);

	alpha_save = alpha;

	return alpha;
}

int oplus_get_panel_brightness_to_alpha(void)
{
	if (oplus_panel_alpha)
		return oplus_panel_alpha;

	return brightness_to_alpha(oplus_display_brightness);
}

static ssize_t oplus_display_get_dim_alpha(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	if (!primary_display_get_fp_hbm_state())
		return sprintf(buf, "%d\n", 0);

	oplus_underbrightness_alpha = oplus_get_panel_brightness_to_alpha();

	return sprintf(buf, "%d\n", oplus_underbrightness_alpha);
}

static ssize_t oplus_display_set_dim_alpha(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
	sscanf(buf, "%x", &oplus_panel_alpha);
	return count;
}

int oplus_display_panel_get_dim_alpha(void *buf)
{
	unsigned int *dim_alpha = buf;

	if (!primary_display_get_fp_hbm_state()) {
		(*dim_alpha) = 0;
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

int oplus_dc_alpha = 0;
int oplus_dc_enable = 0;

static ssize_t oplus_display_get_dc_enable(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", oplus_dc_enable);
}

static ssize_t oplus_display_set_dc_enable(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
	sscanf(buf, "%x", &oplus_dc_enable);
	return count;
}

int oplus_display_panel_get_dimlayer_enable(void *buf)
{
	unsigned int *dc_enable = buf;
	(*dc_enable) = oplus_dc_enable;
	pr_info("value is %d", oplus_dc_enable);
	return 0;
}

int oplus_display_panel_set_dimlayer_enable(void *buf)
{
	unsigned int *dc_enable = buf;
	oplus_dc_enable = (*dc_enable);
	pr_info("value is %d", (*dc_enable));
	return 0;
}

static ssize_t oplus_display_get_dim_dc_alpha(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", oplus_dc_alpha);
}

static ssize_t oplus_display_set_dim_dc_alpha(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
	sscanf(buf, "%x", &oplus_dc_alpha);
	return count;
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

unsigned long HBM_mode = 0;
unsigned long HBM_pre_mode = 0;
/*struct timespec hbm_time_on;
struct timespec hbm_time_off;
long hbm_on_start = 0;*/

static ssize_t LCM_HBM_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	printk("%s HBM_mode=%ld\n", __func__, HBM_mode);
	return sprintf(buf, "%ld\n", HBM_mode);
}

static ssize_t LCM_HBM_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t num)
{
	int ret;
	unsigned char payload[100] = "";
	printk("oplus_display_hbm_support = %d\n", oplus_display_hbm_support);
	if (oplus_display_hbm_support) {
		HBM_pre_mode = HBM_mode;
		ret = kstrtoul(buf, 10, &HBM_mode);
		printk("%s HBM_mode=%ld\n", __func__, HBM_mode);
		ret = primary_display_set_hbm_mode((unsigned int)HBM_mode);
		/*if (HBM_mode == 8) {
			get_monotonic_boottime(&hbm_time_on);
			hbm_on_start = hbm_time_on.tv_sec;
		} else if (HBM_pre_mode == 8 && HBM_mode != 8) {
			get_monotonic_boottime(&hbm_time_off);
			scnprintf(payload, sizeof(payload), "EventID@@%d$$hbm@@hbm state on time = %ld sec$$ReportLevel@@%d",
				OPLUS_MM_DIRVER_FB_EVENT_ID_HBM,(hbm_time_off.tv_sec - hbm_on_start),OPLUS_MM_DIRVER_FB_EVENT_REPORTLEVEL_LOW);
			upload_mm_kevent_fb_data(OPLUS_MM_DIRVER_FB_EVENT_MODULE_DISPLAY,payload);
		}*/
	}
	return num;
}

int oplus_display_panel_set_hbm(void *buf)
{
	int ret = -1;
	unsigned int *HBM = buf;

	if (oplus_display_hbm_support) {
		HBM_pre_mode = HBM_mode;
		HBM_mode = (*HBM);
		pr_info("HBM_mode=%ld\n", (*HBM));
		ret = primary_display_set_hbm_mode((unsigned int)HBM_mode);
/*
		if (HBM_mode == 1) {
			mdelay(80);
			printk("%s delay done\n", __func__);
		}
		if (HBM_mode == 8) {
			get_monotonic_boottime(&hbm_time_on);
			hbm_on_start = hbm_time_on.tv_sec;
		}
*/
	} else {
		pr_info("not to support set hbm\n");
	}
	return ret;
}

int oplus_display_panel_get_hbm(void *buf)
{
	unsigned int *HBM = buf;
	pr_info("HBM_mode=%ld\n", HBM_mode);
	(*HBM) = HBM_mode;
	return 0;
}


#if 0
unsigned int ffl_set_mode = 0;
unsigned int ffl_backlight_on = 0;
extern bool ffl_trigger_finish;

static ssize_t FFL_SET_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	printk("%s ffl_set_mode=%d\n", __func__, ffl_set_mode);
	return sprintf(buf, "%d\n", ffl_set_mode);
}

static ssize_t FFL_SET_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t num)
{
	int ret;
	printk("oplus_display_ffl_support = %d\n", oplus_display_ffl_support);
	if (oplus_display_ffl_support) {
		ret = kstrtouint(buf, 10, &ffl_set_mode);
		printk("%s ffl_set_mode=%d\n", __func__, ffl_set_mode);
		if (ffl_trigger_finish && (ffl_backlight_on == 1) && (ffl_set_mode == 1)) {
			ffl_set_enable(1);
		}
	}
	return num;
}
#endif

unsigned char lcm_id_addr = 0;
static uint32_t lcm_id_info = 0x0;

static ssize_t lcm_id_info_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 0;
	if ((oplus_display_panelid_support) && (lcm_id_addr != 0)) {
		if (oplus_display_aod_ramless_support) {
			ret = primary_display_read_lcm_id(lcm_id_addr, &lcm_id_info, LCM_REG_READ_LEN);
		} else {
			ret = primary_display_read_lcm_id(lcm_id_addr, &lcm_id_info, LCM_ID_READ_LEN);
		}
		ret = scnprintf(buf, PAGE_SIZE, "LCM ID[%x]: 0x%x 0x%x\n", lcm_id_addr, lcm_id_info, 0);
	} else {
		ret = scnprintf(buf, PAGE_SIZE, "LCM ID[00]: 0x00 0x00\n");
	}
	lcm_id_addr = 0;
	return ret;
}

static ssize_t lcm_id_info_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t num)
{
	int ret;
	ret = kstrtou8(buf, 0, &lcm_id_addr);
	printk("%s lcm_id_addr = 0x%x\n", __func__, lcm_id_addr);
	return num;
}

int oplus_display_panel_get_id(void *buf)
{
	int ret = 0;
	unsigned char id_addr;
	struct panel_id *p_id = buf;

	id_addr = (unsigned char)p_id->DA; /*for mtk DA represent the id_addr to read*/
	p_id->DB = 0;
	p_id->DC = 0;

	pr_info("\n");

	if ((oplus_display_panelid_support) && (id_addr != 0)) {
		if (oplus_display_aod_ramless_support) {
			ret = primary_display_read_lcm_id(id_addr, &p_id->DA, LCM_REG_READ_LEN);
		} else {
			ret = primary_display_read_lcm_id(id_addr, &p_id->DA, LCM_ID_READ_LEN);
		}
		ret = 0;
	} else {
		pr_err("read id not support\n");
		ret = -1;
	}

	return ret;
}


static uint64_t serial_number = 0x0;

int lcm_first_get_serial(void)
{
	int ret = 0;
	if (oplus_display_panelnum_support) {
		pr_err("lcm_first_get_serial\n");
		ret = panel_serial_number_read(PANEL_SERIAL_NUM_REG, &serial_number,
				PANEL_REG_READ_LEN);
	}

	return ret;
}

int lcm_first_get_serial_change(void)
{
	int ret = 0;

	pr_err("lcm_first_get_serial\n");
	ret = panel_serial_number_read(PANEL_SERIAL_NUM_REG, &serial_number,
			PANEL_REG_READ_LEN);

	return ret;
}

static ssize_t mdss_get_panel_serial_number(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 0;

	if (oplus_display_panelnum_support) {
		if (serial_number == 0) {
			ret = primary_display_read_serial(PANEL_SERIAL_NUM_REG, &serial_number,
				PANEL_REG_READ_LEN);
		}
		if (ret <= 0 && serial_number == 0)
			ret = scnprintf(buf, PAGE_SIZE, "Get serial number failed: %d\n",ret);
		else
			ret = scnprintf(buf, PAGE_SIZE, "Get panel serial number: %llx\n",serial_number);
	} else if (oplus_display_panelnum_continue_support) {
		if (serial_number == 0) {
			ret = primary_display_read_serial(PANEL_SERIAL_NUM_REG_CONTINUE, &serial_number,
				PANEL_REG_READ_LEN);
		}
		if (ret <= 0 && serial_number == 0)
			ret = scnprintf(buf, PAGE_SIZE, "Get serial number failed: %d\n",ret);
		else
			ret = scnprintf(buf, PAGE_SIZE, "Get panel serial number: %llx\n",serial_number);
	} else {
		ret = scnprintf(buf, PAGE_SIZE, "Unsupported panel!!\n");
	}
	return ret;
}

static ssize_t panel_serial_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{

    DISPMSG("[soso] Lcm read 0xA1 reg = 0x%llx\n", serial_number);

	return count;
}

static ssize_t silence_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	pr_info("oplus_silence_mode=%ld\n", oplus_silence_mode);
	return sprintf(buf, "%ld\n", oplus_silence_mode);
}

static ssize_t silence_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t num)
{
	int ret;
	ret = kstrtoul(buf, 10, &oplus_silence_mode);
	pr_info("oplus_silence_mode=%ld\n", oplus_silence_mode);
	return num;
}

int oplus_display_panel_get_closebl_flag(void *buf)
{
	unsigned int *s_mode = buf;

	pr_info("oplus_silence_mode=%ld\n", oplus_silence_mode);
	(*s_mode) = (unsigned int)oplus_silence_mode;

	return 0;
}

int oplus_display_panel_set_closebl_flag(void *buf)
{
	unsigned int *s_mode = buf;

	oplus_silence_mode = (*s_mode);
	pr_info("oplus_silence_mode=%ld\n", (*s_mode));

	return 0;
}

bool oplus_fp_notify_down_delay = false;
bool oplus_fp_notify_up_delay = false;
bool ds_rec_fpd;
bool doze_rec_fpd;
void fingerprint_send_notify(struct fb_info *fbi, uint8_t fingerprint_op_mode)
{
	struct fb_event event;

	event.info  = fbi;
	event.data = &fingerprint_op_mode;
	if(delay_uiready > 0) {
		msleep(delay_uiready * 16);
		pr_info("%s delay %d ms for uiready.\n", __func__, delay_uiready * 16);
	}
	fb_notifier_call_chain(MTK_ONSCREENFINGERPRINT_EVENT, &event);
	pr_info("%s send uiready : %d\n", __func__, fingerprint_op_mode);
}

int oplus_display_panel_set_finger_print(void *buf)
{
	uint8_t fingerprint_op_mode = 0x0;
	uint32_t *finger_print = buf;
	if (oplus_display_fppress_support) {
		/* will ignoring event during panel off situation. */
		if (oplus_flag_lcd_off)
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

static ssize_t fingerprint_notify_trigger(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t num)
{
	uint8_t fingerprint_op_mode = 0x0;

	if (oplus_display_fppress_support) {
		/* will ignoring event during panel off situation. */
		if (oplus_flag_lcd_off)
		{
			pr_err("%s panel in off state, ignoring event.\n", __func__);
			return num;
		}
		if (kstrtou8(buf, 0, &fingerprint_op_mode))
		{
			pr_err("%s kstrtouu8 buf error!\n", __func__);
			return num;
		}
		if (fingerprint_op_mode == 1) {
			oplus_fp_notify_down_delay = true;
		} else {
			oplus_fp_notify_up_delay = true;
			ds_rec_fpd = false;
			doze_rec_fpd = false;
		}
		pr_info("%s receive uiready %d\n", __func__,fingerprint_op_mode);
	}
	return num;
}

unsigned long CABC_mode = 2;

unsigned long cabc_old_mode = 1;
unsigned long cabc_true_mode = 1;
unsigned long cabc_sun_flag = 0;
unsigned long cabc_back_flag = 1;

enum{
    CABC_LEVEL_0,
    CABC_LEVEL_1,
    CABC_LEVEL_3 = 3,
    CABC_EXIT_SPECIAL = 8,
    CABC_ENTER_SPECIAL = 9,
};

#if defined(CONFIG_MACH_MT6768) || defined(CONFIG_MACH_MT6785)
bool backlight_high_light_dre_cabc = true;
#else
bool backlight_high_light_dre_cabc = false;
#endif
/*
* add dre only use for camera
*/
extern void disp_aal_set_dre_en(int enable);

static ssize_t LCM_CABC_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    if (backlight_high_light_dre_cabc) {
        pr_err("%s CABC_mode=%ld\n", __func__, cabc_true_mode);
        return sprintf(buf, "%ld\n", cabc_true_mode);
    } else {
        printk("%s CABC_mode=%ld\n", __func__, CABC_mode);
        return sprintf(buf, "%ld\n", CABC_mode);
    }
}

static ssize_t LCM_CABC_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t num)
{
    int ret = 0;
    if (backlight_high_light_dre_cabc) {
        ret = kstrtoul(buf, 10, &cabc_old_mode);
        cabc_true_mode = cabc_old_mode;
        if(cabc_old_mode < 4) {
            cabc_back_flag = cabc_old_mode;
        }
        pr_err("%s CABC_mode=%ld cabc_back_flag = %d\n", __func__, cabc_old_mode, cabc_back_flag);
        if(CABC_ENTER_SPECIAL == cabc_old_mode) {
            cabc_sun_flag = 1;
            cabc_true_mode = 0;
        } else if (CABC_EXIT_SPECIAL == cabc_old_mode) {
            cabc_sun_flag = 0;
            cabc_true_mode = cabc_back_flag;
        } else if (1 == cabc_sun_flag) {
            if(CABC_LEVEL_0 == cabc_back_flag) {
                disp_aal_set_dre_en(1);
                pr_err("%s enable dre1\n", __func__);
            } else {
                disp_aal_set_dre_en(1);
                pr_err("%s disable dre1\n", __func__);
            }
            return num;
        }

        if(cabc_true_mode == CABC_LEVEL_0 && cabc_back_flag == CABC_LEVEL_0) {
            disp_aal_set_dre_en(1);
            pr_err("%s enable dre2\n", __func__);
        } else {
            disp_aal_set_dre_en(1);
            pr_err("%s enable dre 2\n", __func__);
        }

        pr_err("%s  cabc_true_mode = %d\n", __func__,  cabc_true_mode);
        ret = primary_display_set_cabc_mode((unsigned int)cabc_true_mode);

        if(cabc_true_mode != cabc_back_flag) {
            cabc_true_mode = cabc_back_flag;
        }
    } else {
        ret = kstrtoul(buf, 10, &CABC_mode);
        if( CABC_mode > 3 ){
            CABC_mode = 3;
        }
        printk("%s CABC_mode=%ld\n", __func__, CABC_mode);

        if (CABC_mode == 0) {
            disp_aal_set_dre_en(1);
            printk("%s enable dre\n", __func__);
        } else {
            disp_aal_set_dre_en(0);
            printk("%s disable dre\n", __func__);
        }

        /*
        * modify for oled not need set cabc
        */
        if (oplus_display_cabc_support || oplus_display_cabc_cmdq_support) {
            ret = primary_display_set_cabc_mode((unsigned int)CABC_mode);
        }
    }

    return num;
}

int oplus_display_panel_get_cabc(void *buf)
{
	unsigned int *cabc_status = buf;

	if (backlight_high_light_dre_cabc) {
		pr_info("CABC_mode=%ld\n", cabc_true_mode);
		(*cabc_status) = cabc_true_mode;
	} else {
		pr_info("CABC_mode=%ld\n", CABC_mode);
		(*cabc_status) = CABC_mode;
	}

	return 0;
}

int oplus_display_panel_set_cabc(void *buf)
{
	int ret = -1;
	unsigned int *cabc_status = buf;

	if (backlight_high_light_dre_cabc) {
		cabc_old_mode = (*cabc_status);
		cabc_true_mode = cabc_old_mode;
		if(cabc_old_mode < 4) {
			cabc_back_flag = cabc_old_mode;
		}
		pr_err("CABC_mode=%ld cabc_back_flag = %d\n", cabc_old_mode, cabc_back_flag);
		if(CABC_ENTER_SPECIAL == cabc_old_mode) {
			cabc_sun_flag = 1;
			cabc_true_mode = 0;
		} else if (CABC_EXIT_SPECIAL == cabc_old_mode) {
			cabc_sun_flag = 0;
			cabc_true_mode = cabc_back_flag;
		} else if (1 == cabc_sun_flag) {
			if(CABC_LEVEL_0 == cabc_back_flag) {
				disp_aal_set_dre_en(1);
				pr_err("enable dre1\n");
			} else {
				disp_aal_set_dre_en(1);
				pr_err("disable dre1\n");
			}
			return 0;
		}

		if(cabc_true_mode == CABC_LEVEL_0 && cabc_back_flag == CABC_LEVEL_0) {
			disp_aal_set_dre_en(1);
			pr_err("enable dre2\n");
		} else {
			disp_aal_set_dre_en(1);
			pr_err("enable dre 2\n");
		}

		pr_err("cabc_true_mode = %d\n", cabc_true_mode);
		ret = primary_display_set_cabc_mode((unsigned int)cabc_true_mode);

		if(cabc_true_mode != cabc_back_flag) {
			cabc_true_mode = cabc_back_flag;
		}
	} else {
		CABC_mode = (*cabc_status);
		if(CABC_mode > 3) {
			CABC_mode = 3;
		}
		pr_info("CABC_mode=%ld\n", CABC_mode);

		if (CABC_mode == 0) {
			disp_aal_set_dre_en(1);
			pr_info("enable dre\n");
		} else {
			disp_aal_set_dre_en(0);
			pr_info("disable dre\n");
		}

		/*
		 * modify for oled not need set cabc
		 */
		if (oplus_display_cabc_support || oplus_display_cabc_cmdq_support) {
			ret = primary_display_set_cabc_mode((unsigned int)CABC_mode);
		}
	}
	pr_info("ret = %d\n", ret);

	return 0;
}

static ssize_t oplus_display_get_seed(struct device *dev,
		struct device_attribute *attr, char *buf)
{
        printk(KERN_INFO "oplus_display_get_seed = %d\n", seed_mode);

        return sprintf(buf, "%d\n", seed_mode);
}
static ssize_t oplus_display_set_seed(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;

	ret = kstrtoul(buf, 10, &seed_mode);
	printk("%s,esd mode is %d\n", __func__, seed_mode);
	return count;
}


static struct kobject *oplus_display_kobj;

static DEVICE_ATTR(oplus_brightness, S_IRUGO|S_IWUSR, oplus_display_get_brightness, oplus_display_set_brightness);
static DEVICE_ATTR(oplus_max_brightness, S_IRUGO|S_IWUSR, oplus_display_get_max_brightness, NULL);

static DEVICE_ATTR(aod_light_mode_set, S_IRUGO|S_IWUSR, oplus_get_aod_light_mode, oplus_set_aod_light_mode);
static DEVICE_ATTR(dim_alpha, S_IRUGO|S_IWUSR, oplus_display_get_dim_alpha, oplus_display_set_dim_alpha);
static DEVICE_ATTR(dimlayer_bl_en, S_IRUGO|S_IWUSR, oplus_display_get_dc_enable, oplus_display_set_dc_enable);
static DEVICE_ATTR(dim_dc_alpha, S_IRUGO|S_IWUSR, oplus_display_get_dim_dc_alpha, oplus_display_set_dim_dc_alpha);
/*static DEVICE_ATTR(ffl_set, S_IRUGO|S_IWUSR, FFL_SET_show, FFL_SET_store);*/
static DEVICE_ATTR(panel_id, S_IRUGO|S_IWUSR, lcm_id_info_show, lcm_id_info_store);
static DEVICE_ATTR(panel_serial_number, S_IRUGO|S_IWUSR, mdss_get_panel_serial_number, panel_serial_store);
static DEVICE_ATTR(hbm, S_IRUGO|S_IWUSR, LCM_HBM_show, LCM_HBM_store);
static DEVICE_ATTR(notify_fppress, S_IRUGO|S_IWUSR, NULL, fingerprint_notify_trigger);
static DEVICE_ATTR(sau_closebl_node, S_IRUGO|S_IWUSR, silence_show, silence_store);
static DEVICE_ATTR(LCM_CABC, S_IRUGO|S_IWUSR, LCM_CABC_show, LCM_CABC_store);
static DEVICE_ATTR(seed, S_IRUGO|S_IWUSR, oplus_display_get_seed, oplus_display_set_seed);
#if defined(CONFIG_MACH_MT6785)
/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
static DEVICE_ATTR(aod_area, S_IRUGO|S_IWUSR, NULL, oplus_display_set_aod_area);
/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */
#endif

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *oplus_display_attrs[] = {
	&dev_attr_oplus_brightness.attr,
	&dev_attr_oplus_max_brightness.attr,
	&dev_attr_aod_light_mode_set.attr,
	&dev_attr_dim_alpha.attr,
	&dev_attr_dimlayer_bl_en.attr,
	&dev_attr_dim_dc_alpha.attr,
	/*&dev_attr_ffl_set.attr,*/
	&dev_attr_panel_id.attr,
	&dev_attr_panel_serial_number.attr,
	&dev_attr_hbm.attr,
	&dev_attr_notify_fppress.attr,
	&dev_attr_sau_closebl_node.attr,
	&dev_attr_LCM_CABC.attr,
	&dev_attr_seed.attr,
#if defined(CONFIG_MACH_MT6785)
	/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
	&dev_attr_aod_area.attr,
	/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */
#endif
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group oplus_display_attr_group = {
	.attrs = oplus_display_attrs,
};

static int __init oplus_display_private_api_init(void)
{
	int retval;

	if (get_eng_version() == 1)
		oplus_dc_enable = 1;

	oplus_display_kobj = kobject_create_and_add("oplus_display", kernel_kobj);
	if (!oplus_display_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(oplus_display_kobj, &oplus_display_attr_group);
	if (retval)
		kobject_put(oplus_display_kobj);

	return retval;
}

static void __exit oplus_display_private_api_exit(void)
{
	kobject_put(oplus_display_kobj);
}

module_init(oplus_display_private_api_init);
module_exit(oplus_display_private_api_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Hujie");
