/***************************************************************
** Copyright (C),  2018,  OPLUS Mobile Comm Corp.,  Ltd
** OPLUS_BUG_STABILITY
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
**   Zhang.JianBin 2020/3/30         1.4           Modify for MT6779_R
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
#include <soc/oplus/system/oplus_project.h>
/*
 * we will create a sysfs which called /sys/kernel/oplus_display,
 * In that directory, oplus display private api can be called
 */

#define LCM_ID_READ_LEN 1
#define LCM_REG_READ_LEN 16
#define PANEL_SERIAL_NUM_REG 0xA8
#define PANEL_REG_READ_LEN   16

unsigned long oplus_display_brightness = 0;
unsigned int oplus_set_brightness = 0;
extern unsigned int aod_light_mode;
bool oplus_flag_lcd_off = false;
#ifdef OPLUS_BUG_STABILITY
bool oplus_display_cabc_cmdq_support; /* if cabc is supported(cmd sent with cmdq handle) */
bool oplus_display_aod_support; /* if aod feature is supported */
#endif
bool oplus_display_hbm_support;
bool oplus_display_elevenbits_support;
bool oplus_display_tenbits_support;
bool oplus_display_fppress_support;
bool oplus_display_meta_idle_support;
bool oplus_display_panelnum_support;
bool oplus_display_aodlight_support;
int is_dvt_panel = 1;
bool oplus_display_panelid_support;
bool oplus_display_sau_support;
bool oplus_display_twelvebits_support;
bool oplus_display_local_dre_support = 0;
unsigned int oplus_display_normal_max_brightness;
extern bool oplus_display_mt6382_support;

/*bool oplus_display_ffl_support;*/
/*bool oplus_display_cabc_support;*/

/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
bool oplus_display_aod_ramless_support;
extern int __attribute__((weak)) primary_display_set_aod_area(char *area, int use_cmdq) { return 0; };
/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */

/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
bool oplus_display_aod_ramless_support;
extern unsigned int __attribute((weak)) delay_uiready = 0;
/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */
#if 0
extern void __attribute__((weak)) ffl_set_enable(unsigned int enable) { return; };
extern int __attribute__((weak)) primary_display_set_cabc_mode(unsigned int level) { return 0; };
#endif

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
extern int primary_display_read_lcm_id(char cmd, uint32_t *buf, int num);
int oplus_disp_lcm_set_hbm(struct disp_lcm_handle *plcm, void *handle, unsigned int hbm_level);
#if 0
int disp_lcm_aod_from_display_on(struct disp_lcm_handle *plcm);
int disp_lcm_set_aod_mode(struct disp_lcm_handle *plcm, void *handle, unsigned int mode);
#endif
#if 0
int notify_display_fpd(bool mode);
void fpd_notify_check_trig(void);
void fpd_notify(void);
bool need_update_fpd_fence(struct disp_frame_cfg_t *cfg);
#endif
int primary_display_read_serial(char cmd, uint64_t *buf, int num);
int _read_serial_by_cmdq(char cmd, uint64_t *buf, int num);

int _set_hbm_mode_by_cmdq(unsigned int level);
int primary_display_set_hbm_mode(unsigned int level);
int disp_lcm_poweron_before_ulps(struct disp_lcm_handle *plcm);
int disp_lcm_poweroff_after_ulps(struct disp_lcm_handle *plcm);

#define dsi_set_cmdq(pdata, queue_size, force_update) \
		PM_lcm_utils_dsi0.dsi_set_cmdq(pdata, queue_size, force_update)


void get_panel_state(void) {
	if (strstr(boot_command_line, "is_dvt_panel=1")) {
		is_dvt_panel = 1;
	} else {
		is_dvt_panel = 0;
	}
	pr_err("[LCD] func:%s, is_dvt_panel = %d \n", __func__, is_dvt_panel);
}

static ssize_t oplus_display_get_brightness(struct  kobject *kobj,
                                struct kobj_attribute *attr, char *buf)
{
	if (oplus_display_brightness > LED_FULL) {
		oplus_display_brightness = LED_OFF;
	}
	/* printk(KERN_INFO "%s = %lu\n", __func__, oplus_display_brightness); */
	return sprintf(buf, "%lu\n", oplus_display_brightness);
}

static ssize_t oplus_display_set_brightness(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t num)
{
	int ret;

	ret = kstrtouint(buf, 10, &oplus_set_brightness);

	printk("%s %d\n", __func__, oplus_set_brightness);

	if (oplus_set_brightness > LED_FULL) {
		return num;
	}

	_primary_path_switch_dst_lock();
	_primary_path_lock(__func__);
	primary_display_setbacklight_nolock(oplus_set_brightness);
	_primary_path_unlock(__func__);
	_primary_path_switch_dst_unlock();

	return num;
}

static ssize_t oplus_display_get_max_brightness(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	//printk(KERN_INFO "oplus_display_get_max_brightness = %d\n",LED_FULL);
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
	if (!oplus_display_normal_max_brightness)
		return sprintf(buf, "%d\n", LED_FULL);
	else
		return sprintf(buf, "%u\n", oplus_display_normal_max_brightness);
}

static ssize_t oplus_get_aod_light_mode(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf) {

	printk(KERN_INFO "oplus_get_aod_light_mode = %u\n", aod_light_mode);

	return sprintf(buf, "%u\n", aod_light_mode);
}

static ssize_t oplus_set_aod_light_mode(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t count) {
	unsigned int temp_save = 0;
	int ret = 0;

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

static ssize_t oplus_display_get_dim_alpha(struct kobject *kobj,
                                struct kobj_attribute *attr, char *buf)
{
	if (!primary_display_get_fp_hbm_state())
		return sprintf(buf, "%d\n", 0);

	oplus_underbrightness_alpha = oplus_get_panel_brightness_to_alpha();

	return sprintf(buf, "%d\n", oplus_underbrightness_alpha);
}

static ssize_t oplus_display_set_dim_alpha(struct kobject *kobj,
                               struct kobj_attribute *attr,
                               const char *buf, size_t count)
{
	sscanf(buf, "%x", &oplus_panel_alpha);
	return count;
}


extern int oplus_dc_alpha;
extern int oplus_dc_enable;
static ssize_t oplus_display_get_dc_enable(struct kobject *kobj,
                                struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", oplus_dc_enable);
}

static ssize_t oplus_display_set_dc_enable(struct kobject *kobj,
                               struct kobj_attribute *attr,
                               const char *buf, size_t count)
{
	sscanf(buf, "%x", &oplus_dc_enable);
	return count;
}

static ssize_t oplus_display_get_dim_dc_alpha(struct kobject *kobj,
                                struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", oplus_dc_alpha);
}

static ssize_t oplus_display_set_dim_dc_alpha(struct kobject *kobj,
                               struct kobj_attribute *attr,
                               const char *buf, size_t count)
{
	sscanf(buf, "%x", &oplus_dc_alpha);
	return count;
}

extern unsigned long HBM_mode;
extern unsigned long HBM_pre_mode;
extern struct timespec hbm_time_on;
extern struct timespec hbm_time_off;
extern long hbm_on_start;

/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
struct panel_aod_area oplus_aod_area[RAMLESS_AOD_AREA_NUM];
unsigned char aod_area_cmd[RAMLESS_AOD_PAYLOAD_SIZE];

/* aod area debug command(for 1080*2400):
echo 444 400 292 330 0 2 0 255 : \
	240 760 700 75 0 2 0 255 : \
	331 835 518 56 0 2 0 255 : \
	315 921 550 75 0 2 0 255 : \
	403 2065 304 254 0 3 0 255 > /sys/kernel/oplus_display/aod_area
*/
static ssize_t oplus_display_set_aod_area(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t count) {
	char *bufp = (char *)buf;
	char *token = NULL;
	int i, cnt = 0;
	static unsigned int area_enable = false;

	if (!oplus_display_aod_ramless_support) {
		pr_info("ramless true aod is disable\n");
		return count;
	}

	memset(oplus_aod_area, 0, sizeof(struct panel_aod_area) * RAMLESS_AOD_AREA_NUM);

	pr_err(" %s \n", __func__);
	token = strsep(&bufp, ":");
	while (token != NULL) {
		struct panel_aod_area *area = &oplus_aod_area[cnt];
		if (!*token) {
			continue;
		}

		sscanf(token, "%d %d %d %d %d %d %d %d",
			&area->x, &area->y, &area->w, &area->h,
			&area->color, &area->bitdepth, &area->mono, &area->gray);
		pr_err("aod area: %s %d rect[%dx%d-%dx%d]-%d-%d-%d-%x\n", __func__, __LINE__,
			area->x, area->y, area->w, area->h,
			area->color, area->bitdepth, area->mono, area->gray);
		area_enable = true;
		cnt++;
		token = strsep(&bufp, ":");
	}


	memset(aod_area_cmd, 0, RAMLESS_AOD_PAYLOAD_SIZE);

	for (i = 0; i < RAMLESS_AOD_AREA_NUM; i++) {
		struct panel_aod_area *area = &oplus_aod_area[i];

		aod_area_cmd[0] |= (!!area_enable) << (RAMLESS_AOD_AREA_NUM - i - 1);
		if (area_enable) {
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
	*/
	return count;
}

int oplus_display_panel_set_aod_area(void *buf)
{
	struct panel_aod_area_para *para = (struct panel_aod_area_para *)buf;
	int i, cnt = 0;
	static unsigned int area_enable = false;

	if (!oplus_display_aod_ramless_support) {
		pr_info("ramless true aod is disable\n");
		return -1;
	}

	if (para->size > RAMLESS_AOD_AREA_NUM) {
		pr_err("aod area size is invalid, size=%d\n", para->size);
		return -1;
	}

	memset(oplus_aod_area, 0, sizeof(struct panel_aod_area) * RAMLESS_AOD_AREA_NUM);

	pr_err(" %s \n", __func__);

	for (i = 0; i < para->size; i++) {
		struct panel_aod_area *area = &oplus_aod_area[cnt];

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
		area_enable = true;
		cnt++;
	}

	memset(aod_area_cmd, 0, RAMLESS_AOD_PAYLOAD_SIZE);

	for (i = 0; i < RAMLESS_AOD_AREA_NUM; i++) {
		struct panel_aod_area *area = &oplus_aod_area[i];

		aod_area_cmd[0] |= (!!area_enable) << (RAMLESS_AOD_AREA_NUM - i - 1);
		if (area_enable) {
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
	*/
	return 0;
}
/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */

static ssize_t LCM_HBM_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	printk("%s HBM_mode=%lu\n", __func__, HBM_mode);
	return sprintf(buf, "%lu\n", HBM_mode);
}

static ssize_t LCM_HBM_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t num)
{
	int ret;
	//unsigned char payload[100] = "";
	//printk("oplus_display_hbm_support = %d\n", oplus_display_hbm_support);
	//if (oplus_display_hbm_support) {
		HBM_pre_mode = HBM_mode;
		ret = kstrtoul(buf, 10, &HBM_mode);
		printk("%s HBM_mode=%ld\n", __func__, HBM_mode);
		ret = primary_display_set_hbm_mode((unsigned int)HBM_mode);
		if (HBM_mode == 1) {
			mdelay(80);
			printk("%s delay done\n", __func__);
		}
		if (HBM_mode == 8) {
			get_monotonic_boottime(&hbm_time_on);
			hbm_on_start = hbm_time_on.tv_sec;
		} 
		#if 0
		else if (HBM_pre_mode == 8 && HBM_mode != 8) {
			get_monotonic_boottime(&hbm_time_off);
			scnprintf(payload, sizeof(payload), "EventID@@%d$$hbm@@hbm state on time = %ld sec$$ReportLevel@@%d",
				OPLUS_MM_DIRVER_FB_EVENT_ID_HBM,(hbm_time_off.tv_sec - hbm_on_start),OPLUS_MM_DIRVER_FB_EVENT_REPORTLEVEL_LOW);
			upload_mm_kevent_fb_data(OPLUS_MM_DIRVER_FB_EVENT_MODULE_DISPLAY,payload);
		}
		#endif
	//}
	return num;
}

#if 0
unsigned int ffl_set_mode = 0;
unsigned int ffl_backlight_on = 0;
extern bool ffl_trigger_finish;

static ssize_t FFL_SET_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	printk("%s ffl_set_mode=%d\n", __func__, ffl_set_mode);
	return sprintf(buf, "%d\n", ffl_set_mode);
}

static ssize_t FFL_SET_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t num)
{
	int ret;
	unsigned char payload[32] = "";
	printk("oplus_display_ffl_support = %d\n", oplus_display_ffl_support);
	if (oplus_display_ffl_support) {
		ret = kstrtouint(buf, 10, &ffl_set_mode);
		printk("%s ffl_set_mode=%d\n", __func__, ffl_set_mode);
		if (ffl_trigger_finish && (ffl_backlight_on == 1) && (ffl_set_mode == 1)) {
			ffl_set_enable(1);
		}

		if (ffl_set_mode == 1) {
			ret = scnprintf(payload, sizeof(payload), "EventID@@%d$$fflset@@%d",OPLUS_MM_DIRVER_FB_EVENT_ID_FFLSET,ffl_set_mode);
			upload_mm_kevent_fb_data(OPLUS_MM_DIRVER_FB_EVENT_MODULE_DISPLAY,payload);
		}
	}
	return num;
}
#endif
unsigned char lcm_id_addr = 0;
static uint32_t lcm_id_info = 0x0;

static ssize_t lcm_id_info_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
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

static ssize_t lcm_id_info_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t num)
{
	int ret;
	ret = kstrtou8(buf, 0, &lcm_id_addr);
	printk("%s lcm_id_addr = 0x%x\n", __func__, lcm_id_addr);
	return num;
}

extern uint64_t serial_number;
int lcm_first_get_serial(void)
{
	int ret = 0;
	struct LCM_PARAMS *lcm_param = disp_lcm_get_params(pgc->plcm);

	if (!lcm_param) {
		pr_err("%s, lcm_param is null\n", __func__);
		return -1;
	}

	printk("lcm_param->PANEL_SERIAL_REG IS 0X:%x",lcm_param->PANEL_SERIAL_REG);

	if(lcm_param->PANEL_SERIAL_REG == 0)
		lcm_param->PANEL_SERIAL_REG = PANEL_SERIAL_NUM_REG;

	if (oplus_display_panelnum_support) {
		pr_err("lcm_first_get_serial\n");
		ret = panel_serial_number_read(lcm_param->PANEL_SERIAL_REG, &serial_number,
				PANEL_REG_READ_LEN);
	}
	return ret;
}

#ifdef OPLUS_BUG_STABILITY
int lcm_first_get_serial_change(void)
{
	int ret = 0;

	pr_err("lcm_first_get_serial\n");
	ret = panel_serial_number_read(PANEL_SERIAL_NUM_REG, &serial_number,
			PANEL_REG_READ_LEN);

	return ret;
}
#endif

static ssize_t mdss_get_panel_serial_number(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	struct LCM_PARAMS *lcm_param = disp_lcm_get_params(pgc->plcm);

	if (!lcm_param) {
		pr_err("%s, lcm_param is null\n", __func__);
		return -1;
	}

	printk("lcm_param->PANEL_SERIAL_REG IS 0X:%x",lcm_param->PANEL_SERIAL_REG);

	if(lcm_param->PANEL_SERIAL_REG == 0)
		lcm_param->PANEL_SERIAL_REG = PANEL_SERIAL_NUM_REG;

	if (oplus_display_panelnum_support) {
		if (serial_number == 0) {
			ret = primary_display_read_serial(lcm_param->PANEL_SERIAL_REG, &serial_number,
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

static ssize_t panel_serial_store(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{

    DISPMSG("[soso] Lcm read 0xA1 reg = 0x%llx\n", serial_number);

	return count;
}

extern unsigned long silence_mode;
extern unsigned int fp_silence_mode;

static ssize_t silence_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	printk("%s silence_mode=%lu\n", __func__, silence_mode);
	return sprintf(buf, "%lu\n", silence_mode);
}

static ssize_t silence_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t num)
{
	int ret;
	ret = kstrtoul(buf, 10, &silence_mode);
	printk("%s silence_mode=%ld\n", __func__, silence_mode);
	return num;
}

bool flag_lcd_off = false;
bool oplus_fp_notify_down_delay = false;
bool oplus_fp_notify_up_delay = false;
bool ds_rec_fpd;
bool doze_rec_fpd;

void fingerprint_send_notify(struct fb_info *fbi, uint8_t fingerprint_op_mode)
{
	struct fb_event event;

	event.info  = fbi;
	event.data = &fingerprint_op_mode;
	if (delay_uiready > 0) {
		msleep(delay_uiready * 16);
		pr_info("%s delay %d ms for uiready.\n", __func__, delay_uiready * 16);
	}
	fb_notifier_call_chain(MTK_ONSCREENFINGERPRINT_EVENT, &event);
	pr_info("%s send uiready : %d\n", __func__, fingerprint_op_mode);
}

static ssize_t fingerprint_notify_trigger(struct kobject *kobj,
                               struct kobj_attribute *attr,
                               const char *buf, size_t num)
{
	uint8_t fingerprint_op_mode = 0x0;

	if (oplus_display_fppress_support) {
		/* will ignoring event during panel off situation. */
		if (flag_lcd_off)
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

unsigned long CABC_mode = 1;
unsigned long cabc_true_mode = 0;
unsigned long cabc_sun_flag = 0;
unsigned long cabc_back_flag = 1;

enum{
	CABC_LEVEL_0,
	CABC_LEVEL_1,
	CABC_LEVEL_3  = 3,
	CABC_SWITCH_GAMMA = 7,
	CABC_EXIT_SPECIAL = 8,
	CABC_ENTER_SPECIAL = 9,
};


/*
* add dre only use for camera
*/
extern void __attribute__((weak)) disp_aal_set_dre_en(int enable) { return; };

static ssize_t LCM_CABC_show(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
	printk("%s CABC_mode=%lu\n", __func__, cabc_true_mode);
	return sprintf(buf, "%lu\n", cabc_true_mode);
}

static ssize_t LCM_CABC_store(struct kobject *kobj,
        struct kobj_attribute *attr, const char *buf, size_t num)
{
    int ret = 0;
#ifdef CONFIG_MACH_MT6768
	int prj_id = 0;
#endif
	ret = kstrtoul(buf, 10, &CABC_mode);
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
		return num;
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
	ret = primary_display_set_cabc_mode((unsigned int)cabc_true_mode);
#endif
#ifdef CONFIG_MACH_MT6768
	prj_id = get_project();
	if (prj_id == 22351 || prj_id == 22352 || prj_id == 22353 ||
	    prj_id == 22361 || prj_id == 22362 || prj_id == 22363 || prj_id == 22364 || prj_id == 22365 || prj_id == 22366 || prj_id == 22367) {
		printk("%s cabc primary_display_set_cabc_mode prj_id = %d\n", __func__, prj_id);
		ret = primary_display_set_cabc_mode((unsigned int)cabc_true_mode);
		if (ret) {
			printk("%s fail ret : %d\n", __func__, ret);
			return -1;
		}
	}
#endif
	if(cabc_true_mode != cabc_back_flag)
		cabc_true_mode = cabc_back_flag;

    return num;
}


/*
* add for samsung lcd hbm node
*/

int oplus_disp_lcm_set_hbm(struct disp_lcm_handle *plcm, void *handle, unsigned int hbm_level)
{
	struct LCM_DRIVER *lcm_drv = NULL;

	DISPFUNC();
	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->set_hbm_mode_cmdq) {
			lcm_drv->set_hbm_mode_cmdq(handle, hbm_level);
		} else {
			DISP_PR_ERR("FATAL ERROR, lcm_drv->disp_lcm_set_hbm is null\n");
			return -1;
		}
		return 0;
	}
	DISP_PR_ERR("lcm_drv is null\n");
	return -1;
}
//#endif

//#ifdef OPLUS_BUG_STABILITY
int _set_hbm_mode_by_cmdq(unsigned int level)
{
	int ret = 0;

	struct cmdqRecStruct *cmdq_handle_HBM_mode = NULL;

	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_cmd,
		MMPROFILE_FLAG_PULSE, 1, 1);

	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP,&cmdq_handle_HBM_mode);

	if(ret!=0)
	{
		DISPCHECK("fail to create primary cmdq handle for HBM mode\n");
		return -1;
	}

	if (primary_display_is_video_mode()) {
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
			MMPROFILE_FLAG_PULSE, 1, 2);
		cmdqRecReset(cmdq_handle_HBM_mode);
		ret = oplus_disp_lcm_set_hbm(pgc->plcm,cmdq_handle_HBM_mode,level);
		//_cmdq_flush_config_handle_mira(cmdq_handle_HBM_mode, 1);
		oplus_cmdq_flush_config_handle_mira(cmdq_handle_HBM_mode, 1);
		DISPCHECK("[BL]_set_HBM_mode_by_cmdq ret=%d\n",ret);
	} else {
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
			MMPROFILE_FLAG_PULSE, 1, 3);
		cmdqRecReset(cmdq_handle_HBM_mode);
		cmdqRecWait(cmdq_handle_HBM_mode, CMDQ_SYNC_TOKEN_CABC_EOF);
		oplus_cmdq_handle_clear_dirty(cmdq_handle_HBM_mode);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_HBM_mode);
		ret = oplus_disp_lcm_set_hbm(pgc->plcm,cmdq_handle_HBM_mode,level);
		cmdqRecSetEventToken(cmdq_handle_HBM_mode, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		cmdqRecSetEventToken(cmdq_handle_HBM_mode, CMDQ_SYNC_TOKEN_CABC_EOF);
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
			MMPROFILE_FLAG_PULSE, 1, 4);
		//_cmdq_flush_config_handle_mira(cmdq_handle_HBM_mode, 1);
		oplus_cmdq_flush_config_handle_mira(cmdq_handle_HBM_mode, 1);
		mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
			MMPROFILE_FLAG_PULSE, 1, 6);
		DISPCHECK("[BL]_set_HBM_mode_by_cmdq ret=%d\n",ret);
	}
	cmdqRecDestroy(cmdq_handle_HBM_mode);
	cmdq_handle_HBM_mode = NULL;
	mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
		MMPROFILE_FLAG_PULSE, 1, 5);

	return ret;
}

int primary_display_set_hbm_mode(unsigned int level)
{
	int ret = 0;
	if (flag_lcd_off)
	{
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
	_primary_path_switch_dst_lock();
	_primary_path_lock(__func__);
	if (pgc->state == DISP_SLEPT) {
		DISPCHECK("Sleep State set backlight invald\n");
	} else {
		primary_display_idlemgr_kick((char *)__func__, 0);
		if (primary_display_cmdq_enabled()) {
			if (primary_display_is_video_mode()) {
				mmprofile_log_ex(ddp_mmp_get_events()->primary_set_bl,
					       MMPROFILE_FLAG_PULSE, 0, 7);
				_set_hbm_mode_by_cmdq(level);
			} else {
				_set_hbm_mode_by_cmdq(level);
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
//#endif/*OPLUS_BUG_STABILITY*/
//#endif

/*
* add power seq api for ulps
*/
int disp_lcm_poweron_before_ulps(struct disp_lcm_handle *plcm)
{
	struct LCM_DRIVER *lcm_drv = NULL;

	DISPFUNC();
	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->poweron_before_ulps) {
			lcm_drv->poweron_before_ulps();
		} else {
			DISP_PR_ERR("FATAL ERROR, lcm_drv->poweron_before_ulps is null\n");
			return -1;
		}
		return 0;
	}
	DISP_PR_ERR("lcm_drv is null\n");
	return -1;
}

int disp_lcm_poweroff_after_ulps(struct disp_lcm_handle *plcm)
{
	struct LCM_DRIVER *lcm_drv = NULL;

	DISPFUNC();
	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->poweroff_after_ulps) {
			//if ((0 == tp_gesture_enable_flag()) || (1 == display_esd_recovery_lcm())) {
			//if (0 == tp_gesture_enable_flag()) {
				lcm_drv->poweroff_after_ulps();
			//}
		} else {
			DISP_PR_ERR("FATAL ERROR, lcm_drv->poweroff_after_ulps is null\n");
			return -1;
		}
		return 0;
	}
	DISP_PR_ERR("lcm_drv is null\n");
	return -1;
}

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

	if (flag_lcd_off) {
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

static struct kobject *oplus_display_kobj;

static struct kobj_attribute dev_attr_oplus_brightness = __ATTR(oplus_brightness, S_IRUGO|S_IWUSR, oplus_display_get_brightness, oplus_display_set_brightness);
static struct kobj_attribute dev_attr_oplus_max_brightness = __ATTR(oplus_max_brightness, S_IRUGO|S_IWUSR, oplus_display_get_max_brightness, NULL);
static struct kobj_attribute dev_attr_aod_light_mode_set = __ATTR(aod_light_mode_set, S_IRUGO|S_IWUSR, oplus_get_aod_light_mode, oplus_set_aod_light_mode);
static struct kobj_attribute dev_attr_dim_alpha = __ATTR(dim_alpha, S_IRUGO|S_IWUSR, oplus_display_get_dim_alpha, oplus_display_set_dim_alpha);
static struct kobj_attribute dev_attr_dimlayer_bl_en = __ATTR(dimlayer_bl_en, S_IRUGO|S_IWUSR, oplus_display_get_dc_enable, oplus_display_set_dc_enable);
static struct kobj_attribute dev_attr_dim_dc_alpha = __ATTR(dim_dc_alpha, S_IRUGO|S_IWUSR, oplus_display_get_dim_dc_alpha, oplus_display_set_dim_dc_alpha);
static struct kobj_attribute dev_attr_hbm = __ATTR(hbm, S_IRUGO|S_IWUSR, LCM_HBM_show, LCM_HBM_store);
//static struct kobj_attribute dev_attr_ffl_set = __ATTR(ffl_set, S_IRUGO|S_IWUSR, FFL_SET_show, FFL_SET_store);
static struct kobj_attribute dev_attr_panel_id = __ATTR(panel_id, S_IRUGO|S_IWUSR, lcm_id_info_show, lcm_id_info_store);
static struct kobj_attribute dev_attr_panel_serial_number = __ATTR(panel_serial_number, S_IRUGO|S_IWUSR, mdss_get_panel_serial_number, panel_serial_store);
static struct kobj_attribute dev_attr_sau_closebl_node = __ATTR(sau_closebl_node, S_IRUGO|S_IWUSR, silence_show, silence_store);
static struct kobj_attribute dev_attr_oplus_notify_fppress = __ATTR(oplus_notify_fppress, S_IRUGO|S_IWUSR, NULL, fingerprint_notify_trigger);
static struct kobj_attribute dev_attr_LCM_CABC = __ATTR(LCM_CABC, S_IRUGO|S_IWUSR, LCM_CABC_show, LCM_CABC_store);
/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
static struct kobj_attribute dev_attr_aod_area = __ATTR(aod_area, S_IRUGO|S_IWUSR, NULL, oplus_display_set_aod_area);
/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */

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
	&dev_attr_hbm.attr,
	//&dev_attr_ffl_set.attr,
	&dev_attr_panel_id.attr,
	&dev_attr_panel_serial_number.attr,
	&dev_attr_sau_closebl_node.attr,
	&dev_attr_oplus_notify_fppress.attr,
	&dev_attr_LCM_CABC.attr,
	/* #ifdef OPLUS_FEATURE_RAMLESS_AOD */
	&dev_attr_aod_area.attr,
	/* #endif */ /* OPLUS_FEATURE_RAMLESS_AOD */
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group oplus_display_attr_group = {
	.attrs = oplus_display_attrs,
};

static int __init oplus_display_private_api_init(void)
{
	int retval;

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
