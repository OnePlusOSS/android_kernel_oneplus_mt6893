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
***************************************************************/
#include <oplus_display_common.h>
#include "display_panel/oplus_display_panel.h"
#include "disp_drv_log.h"
#include <linux/time.h>
#include <linux/delay.h>
#include "ddp_dsi.h"
#include <soc/oplus/system/oplus_project.h>

uint64_t serial_number = 0x0;
extern bool oplus_display_panelnum_support;
unsigned long silence_mode = 0;
unsigned int fp_silence_mode = 0;
extern bool oplus_display_local_dre_support;
extern unsigned long CABC_mode;
extern unsigned long cabc_true_mode;
extern unsigned long cabc_sun_flag;
extern unsigned long cabc_back_flag;

extern unsigned long oplus_display_brightness;
extern unsigned int oplus_set_brightness;
extern unsigned long CABC_mode;
extern bool oplus_display_mt6382_support;
extern void __attribute__((weak)) disp_aal_set_dre_en(int enable) { return; };
extern bool oplus_display_panelid_support;
extern unsigned int oplus_display_normal_max_brightness;
#if (defined(CONFIG_MACH_MT6765) || (defined(CONFIG_MACH_MT6768) && !defined(CONFIG_MTK_MT6382_BDG)))
extern int primary_display_set_cabc_mode(unsigned int level);
#endif
#ifdef CONFIG_MTK_MT6382_BDG
extern int primary_display_set_cabc_mode(unsigned int level);
#endif
#ifdef OPLUS_FEATURE_DISPLAY
/* add for limu project adapt to 2 backlight levels */
extern char *  mtkfb_find_lcm_driver(void);
#endif
#define PANEL_REG_READ_LEN   16
#define PANEL_SERIAL_NUM_REG 0xA8
#define LCM_ID_READ_LEN 1
#define LCM_REG_READ_LEN 16

/*******************************************************************************/
/*******************************************************************************/

#define dsi_set_cmdq(pdata, queue_size, force_update) \
		PM_lcm_utils_dsi0.dsi_set_cmdq(pdata, queue_size, force_update)


/*******************************************************************************/
/*******************************************************************************/

int oplus_display_panel_get_max_brightness(void *buf)
{
	uint32_t *max_brightness = buf;
#ifdef OPLUS_FEATURE_DISPLAY
/* add for limu project adapt to 2 backlight levels */
	if (!strcmp(mtkfb_find_lcm_driver(), "oplus22261_ili9883c_hlt_hdp_dsi_vdo_lcm") ||
		!strcmp(mtkfb_find_lcm_driver(), "oplus22261_td4160_truly_hdp_dsi_vdo_lcm") ||
		!strcmp(mtkfb_find_lcm_driver(), "oplus22261_ili9883c_yfhlt_hdp_dsi_vdo_lcm")) {
		struct LCM_PARAMS *lcm_param = disp_lcm_get_params(pgc->plcm);
		if (!lcm_param) {
			pr_err("%s, lcm_param is null\n", __func__);
			return -1;
		}
		oplus_display_normal_max_brightness = lcm_param->brightness_max;
	}
#endif
	if (!max_brightness) {
		printk("invalid argument:NULL\n");
		return -1;
	}

	if (!oplus_display_normal_max_brightness)
		(*max_brightness) = LED_FULL;
	else
		(*max_brightness) = oplus_display_normal_max_brightness;

	printk("func is %s, value is %d", __func__, *max_brightness);
	return 0;
}

int oplus_display_panel_get_serial_number(void *buf)
{
	int ret = 0 , i = 0;
	unsigned char read[30] = {0};
	unsigned base_index = 11;
	struct panel_serial_number* p_serial_number = buf;

	struct LCM_PARAMS *lcm_param = disp_lcm_get_params(pgc->plcm);

	if (!oplus_display_panelnum_support) {
		DISPMSG("%s panel num read not support\n", __func__);
		return ret;
	}

	if (!lcm_param) {
		pr_err("%s, lcm_param is null\n", __func__);
		return -1;
	}

	p_serial_number->date = 0;
	p_serial_number->precise_time = 0;

	if(lcm_param->PANEL_SERIAL_REG == 0)
		lcm_param->PANEL_SERIAL_REG = PANEL_SERIAL_NUM_REG;
	printk("lcm_param->PANEL_SERIAL_REG IS 0X:%x",lcm_param->PANEL_SERIAL_REG);

	if (lcm_param->lcd_serial_number) {
		return 0;
	}

	for (i = 0;i < 10; i++) {
		ret = DSI_dcs_read_lcm_reg_v3_wrapper_DSI0(lcm_param->PANEL_SERIAL_REG, read, 16);
		if (ret < 0) {
			ret = scnprintf(buf, PAGE_SIZE,
					"Get panel serial number failed, reason:%d", ret);
			msleep(20);
			continue;
		}

		/*  0xA1			   11th		12th	13th	14th	15th
		 *  HEX				0x32		0x0C	0x0B	0x29	0x37
		 *  Bit		   [D7:D4][D3:D0] [D5:D0] [D5:D0] [D5:D0] [D5:D0]
		 *  exp			  3	  2	   C	   B	   29	  37
		 *  Yyyy,mm,dd	  2014   2m	  12d	 11h	 41min   55sec
		 *  panel_rnum.data[24:21][20:16] [15:8]  [7:0]
		 *  panel_rnum:precise_time					  [31:24] [23:16] [reserved]
		*/
		p_serial_number->date =
			(((read[base_index] & 0xF0) << 20)
			|((read[base_index] & 0x0F) << 16)
			|((read[base_index+1] & 0x1F) << 8)
			|(read[base_index+2] & 0x1F));

		p_serial_number->precise_time =
			(((read[base_index+3] & 0x3F) << 24)
			|((read[base_index+4] & 0x3F) << 16)
			|(read[base_index+5] << 8)
			|(read[base_index+6]));

		if (!read[base_index]) {
			/*
			 * the panel we use always large than 2011, so
			 * force retry when year is 2011
			 */
			msleep(20);
			continue;
		}

		break;
	}
	return 0;
}

int primary_display_read_serial(char cmd, uint64_t *buf, int num)
{
	int ret = 0;

	DISPFUNC();
	if (flag_lcd_off) {
		pr_err("lcd is off, Not allowed to get panel's serial number\n");
		ret = 2;
		return ret;
	}

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL) {
		DISPMSG("%s skip due to stage %s\n", __func__, disp_helper_stage_spy());
		return 0;
	}
	DISPMSG("[DISP] primary_display_read_serial 0x%x\n", cmd);

	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
		MMPROFILE_FLAG_START, 0, 0);

	_primary_path_switch_dst_lock();
	_primary_path_lock(__func__);

	if (pgc->state == DISP_SLEPT) {
		DISP_PR_ERR("Sleep State set backlight invald\n");
	} else {
		primary_display_idlemgr_kick(__func__, 0);
		if (primary_display_cmdq_enabled()) {
			if (primary_display_is_video_mode()) {
				mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
						MMPROFILE_FLAG_PULSE, 0, 7);
				ret = _read_serial_by_cmdq(cmd, buf, num);
			} else {
				ret = _read_serial_by_cmdq(cmd, buf, num);
			}
			//atomic_set(&delayed_trigger_kick, 1);
			oplus_delayed_trigger_kick_set(1);
		}
	}
	_primary_path_unlock(__func__);
	_primary_path_switch_dst_unlock();

	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
			MMPROFILE_FLAG_END, 0, 0);
	return ret;
}

int _read_serial_by_cmdq(char cmd, uint64_t *buf, int num)
{
	int ret = 0;
	struct cmdqRecStruct *cmdq_handle_serial_mode = NULL;
	DISPMSG("[DISP] _read_serial_by_cmdq.\n");

	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_cmd, MMPROFILE_FLAG_PULSE, 1, 1);

	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle_serial_mode);
	if (ret!=0) {
		DISPCHECK("fail to create primary cmdq handle for read reg\n");
		return -1;
	}

	if (primary_display_is_video_mode()) {
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
					MMPROFILE_FLAG_PULSE, 1, 2);
		cmdqRecReset(cmdq_handle_serial_mode);

		ret = panel_serial_number_read(cmd, buf, num);
		//_cmdq_flush_config_handle_mira(cmdq_handle_serial_mode, 1);
		oplus_cmdq_flush_config_handle_mira(cmdq_handle_serial_mode, 1);
		DISPCHECK("[BL]_set_reg_by_cmdq ret=%d\n",ret);
	} else {
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
					MMPROFILE_FLAG_PULSE, 1, 3);
		cmdqRecReset(cmdq_handle_serial_mode);
		cmdqRecWait(cmdq_handle_serial_mode, CMDQ_SYNC_TOKEN_CABC_EOF);
		oplus_cmdq_handle_clear_dirty(cmdq_handle_serial_mode);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_serial_mode);

		ret = panel_serial_number_read(cmd, buf, num);

		cmdqRecSetEventToken(cmdq_handle_serial_mode, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		cmdqRecSetEventToken(cmdq_handle_serial_mode, CMDQ_SYNC_TOKEN_CABC_EOF);
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
							MMPROFILE_FLAG_PULSE, 1, 4);
		//_cmdq_flush_config_handle_mira(cmdq_handle_serial_mode, 1);
		oplus_cmdq_flush_config_handle_mira(cmdq_handle_serial_mode, 1);
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
							MMPROFILE_FLAG_PULSE, 1, 6);

		DISPCHECK("[BL]_set_reg_by_cmdq ret=%d\n",ret);
	}
	cmdqRecDestroy(cmdq_handle_serial_mode);
	cmdq_handle_serial_mode = NULL;
	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
		MMPROFILE_FLAG_PULSE, 1, 5);

	return ret;
}

atomic_t LCMREG_byCPU = ATOMIC_INIT(0);

typedef struct panel_serial_info
{
	int reg_index;
	uint64_t year;
	uint64_t month;
	uint64_t day;
	uint64_t hour;
	uint64_t minute;
	uint64_t second;
	uint64_t reserved[2];
} PANEL_SERIAL_INFO;

int panel_serial_number_read(char cmd, uint64_t *buf, int num)
{
	int array[4];
	int ret = 0;
	int count = 30;
	int i = 0;
	unsigned char read[16] = {'\0'};
	PANEL_SERIAL_INFO panel_serial_info;

	pr_err("lcd is status flag_lcd_off = %d\n", flag_lcd_off);
	#ifdef OPLUS_BUG_STABILITY
	if (!(oplus_display_aod_support || oplus_display_aod_ramless_support)) {
		pr_err("panel is not samsung unsupported!!\n");
		return 0;
	}
	#endif
	if (flag_lcd_off) {
		pr_err("panel_serial_number_read:lcd is off\n");
		return 0;
	}

	/* set max return packet size */
	/* array[0] = 0x00013700; */
	while (count > 0) {
		array[0] = 0x3700 + (num << 16);
		dsi_set_cmdq(array, 1, 1);

		atomic_set(&LCMREG_byCPU, 1);
#if defined(CONFIG_MACH_MT6765)
		ret = DSI_dcs_read_lcm_reg_v4_wrapper_DSI0(cmd,	read, num);
#else
		if (oplus_display_aod_ramless_support) {
			ret = DSI_dcs_read_lcm_reg_v4_wrapper_DSI0(cmd,	read, num);
		} else {
			ret = DSI_dcs_read_lcm_reg_v3_wrapper_DSI0(cmd,	read, num);
		}
#endif
		for(i = 0;i < 16;i++) {
			pr_err("[zjb]the %d times,the %d ,serial_number is  0x%x \n",count,i,read[i]);
		}
		count--;
		if (ret == 0) {
			*buf = 0;
			printk("%s [soso] error can not read the reg 0x%x \n",
				__func__, cmd);
			continue;
		} else {
			if (oplus_display_aod_ramless_support) {
				panel_serial_info.reg_index = 4;
				panel_serial_info.month		= read[panel_serial_info.reg_index]	& 0x0F;
				panel_serial_info.year		= ((read[panel_serial_info.reg_index + 1] & 0xE0) >> 5) + 7;
				panel_serial_info.day		= read[panel_serial_info.reg_index + 1] & 0x1F;
				panel_serial_info.hour		= read[panel_serial_info.reg_index + 2] & 0x17;
				panel_serial_info.minute	= read[panel_serial_info.reg_index + 3];
				panel_serial_info.second	= read[panel_serial_info.reg_index + 4];
			} else {
				panel_serial_info.reg_index = 11;
				panel_serial_info.year		= (read[panel_serial_info.reg_index] & 0xF0) >> 0x4;
				panel_serial_info.month		= read[panel_serial_info.reg_index]		& 0x0F;
				panel_serial_info.day		= read[panel_serial_info.reg_index + 1]	& 0x1F;
				panel_serial_info.hour		= read[panel_serial_info.reg_index + 2]	& 0x1F;
				panel_serial_info.minute	= read[panel_serial_info.reg_index + 3]	& 0x3F;
				panel_serial_info.second	= read[panel_serial_info.reg_index + 4]	& 0x3F;
			}

			*buf = (panel_serial_info.year		<< 56)\
				 + (panel_serial_info.month		<< 48)\
				 + (panel_serial_info.day		<< 40)\
				 + (panel_serial_info.hour		<< 32)\
				 + (panel_serial_info.minute	<< 24)\
				 + (panel_serial_info.second	<< 16);

				/*
				* add for lcd serial del unused params;
				*/
				//+ (panel_serial_info.reserved[0] << 8)+ (panel_serial_info.reserved[1]);
			if (panel_serial_info.year < 6) {
				continue;
			} else {
				printk("%s year:0x%llx, month:0x%llx, day:0x%llx, hour:0x%llx, minute:0x%llx, second:0x%llx!\n",
					__func__,
					panel_serial_info.year,
					panel_serial_info.month,
					panel_serial_info.day,
					panel_serial_info.hour,
					panel_serial_info.minute,
					panel_serial_info.second);
				break;
			}
		}
	}
	printk("%s Get panel serial number[0x%llx] successfully after try = %d!\n",
			__func__, *buf, (30 - count));
	atomic_set(&LCMREG_byCPU, 0);
	return ret;
}

/*
* add lcm id info read
*/
int lcm_id_info_read(char cmd, uint32_t *buf, int num)
{
	int array[4];
	int ret = 0;
	int count = 30;
	int i = 0;
	unsigned char read[16] = {'\0'};
	#ifdef OPLUS_BUG_STABILITY
	if (!(oplus_display_aod_support || oplus_display_aod_ramless_support)) {
		pr_err("panel is not samsung unsupported!!\n");
		return 0;
	}
	#endif
	if (flag_lcd_off) {
		pr_err("lcm_id_info_read:lcd is off\n");
		return 0;
	}

	while (count > 0) {
		array[0] = 0x00013700;
		dsi_set_cmdq(array, 1, 1);
		atomic_set(&LCMREG_byCPU, 1);
		if (oplus_display_aod_ramless_support) {
			ret = DSI_dcs_read_lcm_reg_v4_wrapper_DSI0(cmd,	read, num);
		} else {
			ret = DSI_dcs_read_lcm_reg_v3_wrapper_DSI0(cmd,	read, num);
		}

		count--;
		if (ret == 0) {
			continue;
		} else {
			break;
		}
		atomic_set(&LCMREG_byCPU, 0);
	}
	if (ret == 0) {
		*buf = 0;
		printk("%s [soso] error can not read the reg 0x%x\n", __func__, cmd);
	} else {
		*buf = read[0];
		printk("%s [0x%x] successfully after try = %d!\n", __func__, *buf, (30 - count));
	}
	if (oplus_display_aod_ramless_support) {
		for(i = 0; i < num; i++) {
			printk("%s [soso] read the reg value[%d] = 0x%x\n", __func__, i, read[i]);
		}
	}
	return ret;
}

int _read_lcm_id_by_cmdq(char cmd, uint32_t *buf, int num)
{
	int ret = 0;
	struct cmdqRecStruct *cmdq_handle_serial_mode = NULL;
	DISPMSG("[DISP] _read_lcm_id_by_cmdq.\n");

	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_cmd, MMPROFILE_FLAG_PULSE, 1, 1);

	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle_serial_mode);
	if (ret != 0) {
		DISPCHECK("fail to create primary cmdq handle for read reg\n");
		return -1;
	}

	if (primary_display_is_video_mode()) {
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
					MMPROFILE_FLAG_PULSE, 1, 2);
		cmdqRecReset(cmdq_handle_serial_mode);

		ret = lcm_id_info_read(cmd, buf, num);

		oplus_cmdq_flush_config_handle_mira(cmdq_handle_serial_mode, 1);
		DISPCHECK("[BL]_read_lcm_id_by_cmdq ret=%d\n", ret);
	} else {
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
					MMPROFILE_FLAG_PULSE, 1, 3);
		cmdqRecReset(cmdq_handle_serial_mode);
		cmdqRecWait(cmdq_handle_serial_mode, CMDQ_SYNC_TOKEN_CABC_EOF);
		oplus_cmdq_handle_clear_dirty(cmdq_handle_serial_mode);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_serial_mode);

		ret = lcm_id_info_read(cmd, buf, num);

		cmdqRecSetEventToken(cmdq_handle_serial_mode, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		cmdqRecSetEventToken(cmdq_handle_serial_mode, CMDQ_SYNC_TOKEN_CABC_EOF);
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
							MMPROFILE_FLAG_PULSE, 1, 4);
		oplus_cmdq_flush_config_handle_mira(cmdq_handle_serial_mode, 1);
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
							MMPROFILE_FLAG_PULSE, 1, 6);

		DISPCHECK("[BL]_read_lcm_id_by_cmdq ret=%d\n", ret);
	}
	cmdqRecDestroy(cmdq_handle_serial_mode);
	cmdq_handle_serial_mode = NULL;
	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
		MMPROFILE_FLAG_PULSE, 1, 5);

	return ret;
}

int primary_display_read_lcm_id(char cmd, uint32_t *buf, int num)
{
	int ret = 0;

	DISPFUNC();
	if (flag_lcd_off) {
		pr_err("lcd is off, Not allowed to get panel's serial number\n");
		ret = 2;
		return ret;
	}

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL) {
		DISPMSG("%s skip due to stage %s\n", __func__, disp_helper_stage_spy());
		return 0;
	}
	DISPMSG("[DISP] primary_display_read_lcm_id 0x%x\n", cmd);

	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
		MMPROFILE_FLAG_START, 0, 0);

	_primary_path_switch_dst_lock();
	_primary_path_lock(__func__);

	if (pgc->state == DISP_SLEPT) {
		DISP_PR_ERR("Sleep State set backlight invald\n");
	} else {
		primary_display_idlemgr_kick(__func__, 0);
		if (primary_display_cmdq_enabled()) {
			if (primary_display_is_video_mode()) {
				mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
						MMPROFILE_FLAG_PULSE, 0, 7);
				ret = _read_lcm_id_by_cmdq(cmd, buf, num);
			} else {
				ret = _read_lcm_id_by_cmdq(cmd, buf, num);
			}
			oplus_delayed_trigger_kick_set(1);
		}
	}
	_primary_path_unlock(__func__);
	_primary_path_switch_dst_unlock();

	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
			MMPROFILE_FLAG_END, 0, 0);
	return ret;
}

int oplus_display_panel_get_closebl_flag(void *buf)
{
	unsigned int *s_mode = buf;

	printk("%s silence_mode=%ld\n", __func__, silence_mode);
	(*s_mode) = (unsigned int)silence_mode;

	return 0;
}

int oplus_display_panel_set_closebl_flag(void *buf)
{
	unsigned int *s_mode = buf;

	silence_mode = (*s_mode);
	printk("%s silence_mode=%ld\n", __func__, silence_mode);

	return 0;
}

int oplus_display_set_brightness(void *buf)
{
	unsigned int *oplus_brightness = buf;

	oplus_set_brightness = (*oplus_brightness);
	printk("%s %d\n", __func__, oplus_set_brightness);

	if (oplus_set_brightness > LED_FULL) {
		return -1;
	}

	_primary_path_switch_dst_lock();
	_primary_path_lock(__func__);
	primary_display_setbacklight_nolock(oplus_set_brightness);
	_primary_path_unlock(__func__);
	_primary_path_switch_dst_unlock();

	return 0;
}

int oplus_display_get_brightness(void *buf)
{
	unsigned int *brightness = buf;

	(*brightness) = oplus_display_brightness;

	return 0;
}

enum{
	CABC_LEVEL_0,
	CABC_LEVEL_1,
	CABC_LEVEL_3  = 3,
	CABC_SWITCH_GAMMA = 7,
	CABC_EXIT_SPECIAL = 8,
	CABC_ENTER_SPECIAL = 9,
};
int oplus_display_panel_get_cabc(void *buf)
{
	unsigned int *cabc_status = buf;

	printk("%s CABC_mode=%ld\n", __func__, cabc_true_mode);
	(*cabc_status) = cabc_true_mode;

	return 0;
}

int oplus_display_panel_set_cabc(void *buf)
{
	unsigned int *cabc_status = buf;
#ifdef CONFIG_MACH_MT6768
	int prj_id = 0, ret = 0;
#endif

	CABC_mode = (*cabc_status);
	cabc_true_mode = CABC_mode;
	if (CABC_mode < 4)
		cabc_back_flag = CABC_mode;

	printk("%s CABC_mode=%ld, cabc_back_flag = %d\n", __func__, CABC_mode, cabc_back_flag);
#ifdef CONFIG_MTK_MT6382_BDG
	if (!oplus_display_mt6382_support) {
	} else {
	primary_display_set_cabc_mode((unsigned int)cabc_true_mode);
	}
#endif
	if (CABC_ENTER_SPECIAL == CABC_mode) {
		cabc_sun_flag = 1;
		cabc_true_mode = 0;
	} else if (CABC_EXIT_SPECIAL == CABC_mode) {
		cabc_sun_flag = 0;
		cabc_true_mode = cabc_back_flag;
	} else if (CABC_SWITCH_GAMMA == CABC_mode) {
		cabc_sun_flag = 1;
		cabc_true_mode = 0;
	} else if (1 == cabc_sun_flag) {
		if (CABC_LEVEL_0 == cabc_back_flag) {
			disp_aal_set_dre_en(1);
			pr_err("%s enable dre1\n", __func__);
		} else {
			if (oplus_display_local_dre_support) {
				disp_aal_set_dre_en(0);
				pr_err("%s disable dre1\n", __func__);
			}
		}
		return 0;
	}

	if (cabc_true_mode == CABC_LEVEL_0 && cabc_back_flag == CABC_LEVEL_0) {
		disp_aal_set_dre_en(1);
		pr_err("%s enable dre2\n", __func__);
	} else {
		if (oplus_display_local_dre_support) {
			disp_aal_set_dre_en(0);
			pr_err("%s disable dre2\n", __func__);
		}
	}

	pr_err("%s cabc_true_mode = %d\n", __func__,  cabc_true_mode);
#if (defined(CONFIG_MACH_MT6765) || (defined(CONFIG_MACH_MT6768) && !defined(CONFIG_MTK_MT6382_BDG)))
	printk("%s cabc primary_display_set_cabc_mode\n", __func__);
	primary_display_set_cabc_mode((unsigned int)cabc_true_mode);
#endif

#ifdef CONFIG_MACH_MT6768
	prj_id = get_project();
	if (prj_id == 22351 || prj_id == 22352 || prj_id == 22353 ||
		prj_id == 22361 || prj_id == 22362 || prj_id == 22363 || prj_id == 22364 ||
		prj_id == 22365 || prj_id == 22366 || prj_id == 22367 || prj_id == 22368) {
		printk("%s cabc primary_display_set_cabc_mode prj_id = %d\n", __func__, prj_id);
		ret = primary_display_set_cabc_mode((unsigned int)cabc_true_mode);
		if (ret) {
			printk("%s fail ret : %d\n", __func__, ret);
			return -1;
		}
	}
#endif

	if(cabc_true_mode != cabc_back_flag) {
		cabc_true_mode = cabc_back_flag;
	}

        return 0;
}

int oplus_display_panel_get_id(void *buf)
{
	int ret = 0;
	unsigned char id_addr;
	struct panel_id *p_id = buf;

	id_addr = (unsigned char)p_id->DA; /*for mtk DA represent the id_addr to read*/
	p_id->DB = 0;
	p_id->DC = 0;

	if ((oplus_display_panelid_support) && (id_addr != 0)) {
		if (oplus_display_aod_ramless_support) {
			ret = primary_display_read_lcm_id(id_addr, &p_id->DA, LCM_REG_READ_LEN);
		} else {
			ret = primary_display_read_lcm_id(id_addr, &p_id->DA, LCM_ID_READ_LEN);
		}
		ret = 0;
	} else {
		printk("error: read id not support\n");
		ret = -1;
	}

	return ret;
}

int oplus_display_panel_get_vendor(void *buf)
{
	struct panel_info *p_info = buf;
	struct LCM_PARAMS *lcm_param = disp_lcm_get_params(pgc->plcm);

	if (!lcm_param) {
		printk(KERN_ERR "get lcm_param fail\n");
		return -1;
	}

	memcpy(p_info->version, lcm_param->vendor,
               sizeof(lcm_param->vendor) > 31?31:(sizeof(lcm_param->vendor)+1));
	memcpy(p_info->manufacture, lcm_param->manufacture,
               sizeof(lcm_param->manufacture) > 31?31:(sizeof(lcm_param->manufacture)+1));

	return 0;
}
