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
**   Li.Ping         2019/11/18        1.3           Modify for mt6885
******************************************************************/
#include "oplus_display_private_api.h"
#include "mtk_disp_aal.h"
#include "oplus_display_panel_power.h"
#include <linux/notifier.h>
#include "mtk_debug.h"

/*
 * we will create a sysfs which called /sys/kernel/oplus_display,
 * In that directory, oplus display private api can be called
 */

#define PANEL_SERIAL_NUM_REG 0xA1
#define PANEL_SERIAL_NUM_REG_SUMSUNG 0xD8
#define PANEL_REG_READ_LEN   10
#define BOE_PANEL_SERIAL_NUM_REG 0xA3
#define PANEL_SERIAL_NUM_REG_TIANMA 0xD6

uint64_t serial_number = 0x0;
struct aod_area oplus_aod_area[RAMLESS_AOD_AREA_NUM];
char send_cmd[RAMLESS_AOD_PAYLOAD_SIZE];

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


extern struct drm_device* get_drm_device(void);
extern int mtk_drm_setbacklight(struct drm_crtc *crtc, unsigned int level);
extern int oplus_mtk_drm_sethbm(struct drm_crtc *crtc, unsigned int hbm_mode);
extern int oplus_mtk_drm_setseed(struct drm_crtc *crtc, unsigned int seed_mode);
extern PANEL_VOLTAGE_BAK panel_vol_bak[PANEL_VOLTAGE_ID_MAX];
extern struct drm_panel *p_node;

unsigned long esd_mode = 0;
EXPORT_SYMBOL(esd_mode);
unsigned int aod_light_mode = 0;
unsigned long seed_mode = 0;
extern int mtk_crtc_mipi_freq_switch(struct drm_crtc *crtc, unsigned int en, unsigned int userdata);
extern int mtk_crtc_osc_freq_switch(struct drm_crtc *crtc, unsigned int en, unsigned int userdata);
unsigned long osc_mode = 0;
unsigned long mipi_hopping_mode = 0;
unsigned long oplus_display_brightness = 0;
unsigned long oplus_max_normal_brightness = 0;
int oplus_max_brightness = OPLUS_MAX_BRIGHTNESS;

bool oplus_fp_notify_down_delay = false;
bool oplus_fp_notify_up_delay = false;
extern void fingerprint_send_notify(unsigned int fingerprint_op_mode);
extern int oplus_mtk_drm_setcabc(struct drm_crtc *crtc, unsigned int hbm_mode);
extern int mtkfb_set_aod_backlight_level(unsigned int level);

#ifdef OPLUS_BUG_STABILITY
static BLOCKING_NOTIFIER_HEAD(lcdinfo_notifiers);
int register_lcdinfo_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&lcdinfo_notifiers, nb);
}
int unregister_lcdinfo_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&lcdinfo_notifiers, nb);
}

EXPORT_SYMBOL(register_lcdinfo_notifier);
EXPORT_SYMBOL(unregister_lcdinfo_notifier);

void lcdinfo_notify(unsigned long val, void *v)
{
	blocking_notifier_call_chain(&lcdinfo_notifiers, val, v);
}
#endif

bool oplus_mtk_drm_get_hbm_state(void)
{
	bool hbm_en = false;
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_device *ddev = get_drm_device();
	struct mtk_dsi *dsi;
	struct mtk_ddp_comp *comp;
	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
		typeof(*crtc), head);
	if (!crtc) {
		DDPPR_ERR("get hbm find crtc fail\n");
		return 0;
	}
	mtk_crtc = to_mtk_crtc(crtc);
	comp = mtk_ddp_comp_request_output(mtk_crtc);

	dsi = container_of(comp, struct mtk_dsi, ddp_comp);
	if (mtk_crtc->panel_ext->funcs->hbm_get_state) {
		mtk_crtc->panel_ext->funcs->hbm_get_state(dsi->panel, &hbm_en);
	}
	return hbm_en;
}

static ssize_t oplus_display_get_brightness(struct kobject *kobj,
                                struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%ld\n", oplus_display_brightness);
}

static ssize_t oplus_display_set_brightness(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t num)
{
	int ret;
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int oplus_set_brightness = 0;

	ret = kstrtouint(buf, 10, &oplus_set_brightness);

	printk("%s %d\n", __func__, oplus_set_brightness);

	if (oplus_set_brightness > oplus_max_brightness || oplus_set_brightness < OPLUS_MIN_BRIGHTNESS) {
		printk(KERN_ERR "%s, brightness:%d out of scope\n", __func__, oplus_set_brightness);
		return num;
	}

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return 0;
	}
	mtk_drm_setbacklight(crtc, oplus_set_brightness);

	return num;
}

static ssize_t oplus_display_get_max_brightness(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", oplus_max_brightness);
}

static ssize_t oplus_display_get_maxbrightness(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	if (m_new_pq_persist_property[DISP_PQ_CCORR_SILKY_BRIGHTNESS])
		return sprintf(buf, "%u\n", oplus_max_brightness);
	else
		return sprintf(buf, "%u\n", oplus_max_normal_brightness);
}

unsigned int hbm_mode = 0;
static ssize_t oplus_display_get_hbm(struct kobject *kobj,
                                struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%ld\n", hbm_mode);
}

static ssize_t oplus_display_set_hbm(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t num)
{
	int ret;
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int tmp = 0;

	ret = kstrtouint(buf, 10, &tmp);

	printk("%s, %d to be %d\n", __func__, hbm_mode, tmp);

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return 0;
	}

	oplus_mtk_drm_sethbm(crtc, tmp);
	hbm_mode = tmp;

	if (tmp == 1)
		usleep_range(30000, 31000);
	return num;
}

static ssize_t oplus_display_set_panel_pwr(struct kobject *kobj,
                               struct kobj_attribute *attr,
                               const char *buf, size_t num)
{
	u32 panel_vol_value = 0, panel_vol_id = 0;
	int rc = 0;

	sscanf(buf, "%d %d", &panel_vol_id, &panel_vol_value);
	panel_vol_id = panel_vol_id & 0x0F;

	pr_err("debug for %s, buf = [%s], id = %d value = %d, num = %d\n",
		__func__, buf, panel_vol_id, panel_vol_value, num);

	if (panel_vol_id < 0 || panel_vol_id >= PANEL_VOLTAGE_ID_MAX) {
		return -EINVAL;
	}

	if (panel_vol_value < panel_vol_bak[panel_vol_id].voltage_min ||
		panel_vol_id > panel_vol_bak[panel_vol_id].voltage_max) {
		return -EINVAL;
	}

	if (panel_vol_id == PANEL_VOLTAGE_ID_VG_BASE) {
		pr_err("%s: set the VGH_L pwr = %d \n", __func__, panel_vol_value);
		rc = oplus_panel_set_vg_base(panel_vol_value);
		if (rc < 0) {
			return rc;
		}

		return num;
	}

	if (p_node && p_node->funcs->oplus_set_power) {
		rc = p_node->funcs->oplus_set_power(panel_vol_id, panel_vol_value);
		if (rc) {
			pr_err("Set voltage(%s) fail, rc=%d\n",
				 __func__, rc);
			return -EINVAL;
		}
	}

	return num;
}

static ssize_t oplus_display_get_panel_pwr(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf) {
	int ret = 0;
	u32 i = 0;

	for (i = 0; i < (PANEL_VOLTAGE_ID_MAX-1); i++) {
		if (p_node && p_node->funcs->oplus_update_power_value) {
			ret = p_node->funcs->oplus_update_power_value(panel_vol_bak[i].voltage_id);
		}

		if (ret < 0) {
			pr_err("%s : update_current_voltage error = %d\n", __func__, ret);
		}
		else {
			panel_vol_bak[i].voltage_current = ret;
		}
	}

	return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d\n",
		panel_vol_bak[0].voltage_id, panel_vol_bak[0].voltage_min,
		panel_vol_bak[0].voltage_current, panel_vol_bak[0].voltage_max,
		panel_vol_bak[1].voltage_id, panel_vol_bak[1].voltage_min,
		panel_vol_bak[1].voltage_current, panel_vol_bak[1].voltage_max,
		panel_vol_bak[2].voltage_id, panel_vol_bak[2].voltage_min,
		panel_vol_bak[2].voltage_current, panel_vol_bak[2].voltage_max);
}

static ssize_t fingerprint_notify_trigger(struct kobject *kobj,
                               struct kobj_attribute *attr,
                               const char *buf, size_t num)
{
	unsigned int fingerprint_op_mode = 0x0;

	if (kstrtouint(buf, 0, &fingerprint_op_mode)) {
		pr_err("%s kstrtouu8 buf error!\n", __func__);
		return num;
	}

	if (fingerprint_op_mode == 1) {
		oplus_fp_notify_down_delay = true;
	} else {
		oplus_fp_notify_up_delay = true;
	}

	printk(KERN_ERR "%s receive uiready %d\n", __func__, fingerprint_op_mode);
	return num;
}

/*
* add for lcd serial
*/
int panel_serial_number_read(struct drm_crtc *crtc, char cmd, int num)
{
	char para[20] = {0};
	char *panel_name;
	int count = 10;
	PANEL_SERIAL_INFO panel_serial_info;
	struct mtk_ddp_comp *comp;
	struct mtk_drm_crtc *mtk_crtc;

#ifdef NO_AOD_6873
	return 0;
#endif

	para[0] = cmd;
	para[1] = num;

	mtk_crtc = to_mtk_crtc(crtc);
	comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (!(mtk_crtc->enabled)) {
		DDPINFO("Sleep State set backlight stop --crtc not ebable\n");
		return 0;
	}

	if (!comp || !comp->funcs || !comp->funcs->io_cmd) {
		DDPINFO("cannot find output component\n");
		return 0;
	}
	if (mtk_crtc && mtk_crtc->panel_ext && mtk_crtc->panel_ext->params
		&& mtk_crtc->panel_ext->params->oplus_serial_para0) {
	    para[0] = mtk_crtc->panel_ext->params->oplus_serial_para0;
	} else {
	    para[0] = PANEL_SERIAL_NUM_REG;
	}

	if (mtk_crtc && mtk_crtc->panel_ext && mtk_crtc->panel_ext->params
		&& mtk_crtc->panel_ext->params->oplus_serial_para2) {
	    para[2] = mtk_crtc->panel_ext->params->oplus_serial_para2;
	} else {
	    para[2] = 0;
	}

	mtk_ddp_comp_io_cmd(comp, NULL, GET_PANEL_NAME, &panel_name);
	printk("panelname=%s\n", panel_name);
	if (!strcmp(panel_name, "oplus20131_boe_nt37800_1080p_dsi_cmd")
		|| !strcmp(panel_name, "oplus20131_tianma_nt37701_32_1080p_dsi_cmd")
		|| !strcmp(panel_name, "oplus20171_boe_nt37701_1080p_dsi_cmd")
		|| !strcmp(panel_name, "oplus20171_tianma_nt37701_42_1080p_dsi_cmd"))
		para[0] = BOE_PANEL_SERIAL_NUM_REG;
	if ((!strcmp(panel_name, "oplus20615_samsung_ams643ye01_1080p_dsi_cmd"))
		|| (!strcmp(panel_name, "oplus20615_samsung_ams643ye05_1080p_dsi_cmd"))
		|| (!strcmp(panel_name, "oplus_samsung_ams643ye04_1080p_dsi_cmd"))
		|| (!strcmp(panel_name, "oplus2169E_samsung_ams643ag01_1080p_dsi_cmd,lcm"))) {
		para[0] = PANEL_SERIAL_NUM_REG_SUMSUNG;
		comp->funcs->io_cmd(comp, NULL, PANEL_SN_SET, NULL);
	}
	if ((!strcmp(panel_name, "oplus20131_samsung_ana6705_1080p_dsi_cmd,lcm")) || (!strcmp(panel_name, "oplus20601_samsung_sofe03f_m_1080p_dsi_cmd")) || (!strcmp(panel_name, "oplus20171_samsung_ana6705_1080p_dsi_cmd,lcm")))
		para[2] = 11;
	while (count > 0) {
		comp->funcs->io_cmd(comp, NULL, DSI_READ, para);
		count--;
		if ((!strcmp(panel_name, "oplus20131_samsung_ana6705_1080p_dsi_cmd,lcm"))
			|| (!strcmp(panel_name, "oplus20601_samsung_sofe03f_m_1080p_dsi_cmd"))
			|| (!strcmp(panel_name, "oplus20171_samsung_ana6705_1080p_dsi_cmd,lcm"))
			|| (!strcmp(panel_name, "oplus2169E_samsung_ams643ag01_1080p_dsi_cmd,lcm"))
			|| (!strcmp(panel_name, "oplus21127_samsung_ams643ag01_1080p_dsi_cmd,lcm"))
			|| (!strcmp(panel_name, "oplus20181_samsung_ams643ye01_1080p_dsi_cmd,lcm"))
			|| (!strcmp(panel_name, "oplus21851_samsung_ams643ag01_1080p_dsi_cmd,lcm"))
			|| (!strcmp(panel_name, "oplus21081_samsung_ams643ag01_1080p_dsi_cmd,lcm"))) {
				panel_serial_info.reg_index = 0;
				panel_serial_info.month     = para[panel_serial_info.reg_index] & 0x0F;
				panel_serial_info.year      = (para[panel_serial_info.reg_index] & 0xF0) >> 4;
		} else if ((!strcmp(panel_name, "oplus20131_boe_nt37800_1080p_dsi_cmd"))
			|| (!strcmp(panel_name, "oplus20131_tianma_nt37701_32_1080p_dsi_cmd"))
			|| (!strcmp(panel_name, "oplus20171_boe_nt37701_1080p_dsi_cmd"))
			|| (!strcmp(panel_name, "oplus20171_tianma_nt37701_42_1080p_dsi_cmd"))
			|| (!strcmp(panel_name, "oplus21015_tianma_nt37701_42_1080p_dsi_cmd"))
			|| (!strcmp(panel_name, "oplus21015_boe_nt37701_1080p_dsi_cmd"))
			|| (!strcmp(panel_name, "oplus21015_boe_nt37701_2ftp_1080p_dsi_cmd"))) {
				panel_serial_info.reg_index = 0;
				panel_serial_info.month     = para[panel_serial_info.reg_index] & 0x0F;
				panel_serial_info.year      = ((para[panel_serial_info.reg_index] & 0xF0) >> 4) + 1;
		} else if ((!strcmp(panel_name, "oplus20615_samsung_ams643ye01_1080p_dsi_cmd"))
			|| (!strcmp(panel_name, "oplus20615_samsung_ams643ye05_1080p_dsi_cmd"))
			|| (!strcmp(panel_name, "oplus_samsung_ams643ye04_1080p_dsi_cmd"))) {
				panel_serial_info.reg_index = 7;
				panel_serial_info.month     = para[panel_serial_info.reg_index] & 0x0F;
				panel_serial_info.year      = ((para[panel_serial_info.reg_index] & 0xF0) >> 4);
		} else {
				panel_serial_info.reg_index = 4;
				panel_serial_info.month     = para[panel_serial_info.reg_index] & 0x0F;
				panel_serial_info.year      = ((para[panel_serial_info.reg_index + 1] & 0xE0) >> 5) + 7;
		}

		if ((!strcmp(panel_name, "oplus20181_samsung_ams643ye01_1080p_dsi_cmd,lcm"))
			|| (!strcmp(panel_name, "oplus21851_samsung_ams643ag01_1080p_dsi_cmd,lcm"))
			|| (!strcmp(panel_name, "oplus21081_samsung_ams643ag01_1080p_dsi_cmd,lcm"))) {
			panel_serial_info.day       = para[panel_serial_info.reg_index + 1] & 0x1F;
			panel_serial_info.hour      = para[panel_serial_info.reg_index + 2] & 0x1F;
			panel_serial_info.minute    = para[panel_serial_info.reg_index + 3] & 0x3F;
			panel_serial_info.second    = para[panel_serial_info.reg_index + 4] & 0x3F;
		} else {
			panel_serial_info.day       = para[panel_serial_info.reg_index + 1] & 0x1F;
			panel_serial_info.hour      = para[panel_serial_info.reg_index + 2] & 0x17;
			panel_serial_info.minute    = para[panel_serial_info.reg_index + 3];
			panel_serial_info.second    = para[panel_serial_info.reg_index + 4];
		}

		panel_serial_info.reserved[0] = 0;
		panel_serial_info.reserved[1] = 0;

		serial_number = (panel_serial_info.year     << 56)\
							+ (panel_serial_info.month      << 48)\
							+ (panel_serial_info.day        << 40)\
							+ (panel_serial_info.hour       << 32)\
							+ (panel_serial_info.minute << 24)\
							+ (panel_serial_info.second << 16)\
							+ (panel_serial_info.reserved[0] << 8)\
							+ (panel_serial_info.reserved[1]);
		if (panel_serial_info.year <= 7) {
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
	printk("%s Get panel serial number[0x%llx]\n", __func__, serial_number);
	return 1;
}

static ssize_t oplus_get_panel_serial_number(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
		typeof(*crtc), head);
	if (!crtc) {
		DDPPR_ERR("get_panel_serial_number find crtc fail\n");
		return 0;
	}
	if (serial_number == 0) {
		printk(KERN_ERR "dsi_cmd %s Failed. serial_number == 0,run panel_serial_number_read()\n", __func__);
		panel_serial_number_read(crtc, PANEL_SERIAL_NUM_REG, PANEL_REG_READ_LEN);
		printk(KERN_ERR "%s .after read, serial_number: %llx\n", __func__, serial_number);
	}
	return scnprintf(buf, PAGE_SIZE, "Get panel serial number: %llx\n", serial_number);
}

static ssize_t panel_serial_store(struct kobject *kobj,
                struct kobj_attribute *attr,
                const char *buf, size_t count)
{
        printk("[soso] Lcm read 0xA1 reg = 0x%llx\n", serial_number);
        return count;
}

int oplus_dc_threshold = 260;
int oplus_panel_alpha = 0;
int oplus_underbrightness_alpha = 0;
int alpha_save = 0;
struct ba {
	u32 brightness;
	u32 alpha;
};
struct ba brightness_seed_alpha_lut_dc[] = {
	{0, 0xff},
	{1, 0xfc},
	{2, 0xfb},
	{3, 0xfa},
	{4, 0xf9},
	{5, 0xf8},
	{6, 0xf7},
	{8, 0xf6},
	{10, 0xf4},
	{15, 0xf0},
	{20, 0xea},
	{30, 0xe0},
	{45, 0xd0},
	{70, 0xbc},
	{100, 0x98},
	{120, 0x80},
	{140, 0x70},
	{160, 0x58},
	{180, 0x48},
	{200, 0x30},
	{220, 0x20},
	{240, 0x10},
	{260, 0x00},
};
struct ba brightness_alpha_lut[] = {
	{0, 0xFF},
	{5, 0xEE},
	{7, 0xED},
	{10, 0xE7},
	{20, 0xE3},
	{35, 0xDC},
	{60, 0xD1},
	{90, 0xCE},
	{150, 0xC1},
	{280, 0xAA},
	{460, 0x95},
	{650, 0x7F},
	{850, 0x79},
	{1000, 0x6E},
	{1150, 0x62},
	{1300, 0x52},
	{1500, 0x4C},
	{1700, 0x42},
	{1900, 0x35},
	{2047, 0x24},
};

struct ba brightness_alpha_lut_BOE[] = {
	{0, 0xFF},
	{12, 0xEE},
	{20, 0xE8},
	{35, 0xE5},
	{65, 0xDA},
	{100, 0xD8},
	{150, 0xCD},
	{210, 0xC5},
	{320, 0xB9},
	{450, 0xA9},
	{630, 0xA0},
	{870, 0x94},
	{1150, 0x86},
	{1500, 0x7B},
	{1850, 0x6B},
	{2250, 0x66},
	{2650, 0x55},
	{3050, 0x47},
	{3400, 0x39},
	{3515, 0x24},
};

struct ba brightness_alpha_lut_index[] = {
	{0, 0xFF},
	{150, 0xEE},
	{190, 0xEB},
	{230, 0xE6},
	{270, 0xE1},
	{310, 0xDA},
	{350, 0xD7},
	{400, 0xD3},
	{450, 0xD0},
	{500, 0xCD},
	{600, 0xC3},
	{700, 0xB8},
	{900, 0xA3},
	{1100, 0x90},
	{1300, 0x7B},
	{1400, 0x70},
	{1500, 0x65},
	{1700, 0x4E},
	{1900, 0x38},
	{2047, 0x23},
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
	int alpha;
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;

	int i = 0;
	int level = 0;
	struct drm_device *ddev = get_drm_device();

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return 0;
	}

	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc || !mtk_crtc->panel_ext || !mtk_crtc->panel_ext->params) {
		pr_err("falied to get lcd proc info\n");
		return 0;
	}

	if(!strcmp(mtk_crtc->panel_ext->params->manufacture, "boe_nt37800_2048")) {
		level = ARRAY_SIZE(brightness_alpha_lut_BOE);
		for (i = 0; i < ARRAY_SIZE(brightness_alpha_lut_BOE); i++) {
			if (brightness_alpha_lut_BOE[i].brightness >= brightness)
				break;
		}

		if (i == 0)
			alpha = brightness_alpha_lut_BOE[0].alpha;
		else if (i == level)
			alpha = brightness_alpha_lut_BOE[level - 1].alpha;
		else
			alpha = interpolate(brightness,
				brightness_alpha_lut_BOE[i-1].brightness,
				brightness_alpha_lut_BOE[i].brightness,
				brightness_alpha_lut_BOE[i-1].alpha,
				brightness_alpha_lut_BOE[i].alpha);
	}
	else {
		level = ARRAY_SIZE(brightness_alpha_lut);
		for (i = 0; i < ARRAY_SIZE(brightness_alpha_lut); i++) {
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
	}

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

int oplus_seed_bright_to_alpha(int brightness)
{
	int level = ARRAY_SIZE(brightness_seed_alpha_lut_dc);
	int i = 0;
	int alpha;

	for (i = 0; i < ARRAY_SIZE(brightness_seed_alpha_lut_dc); i++) {
		if (brightness_seed_alpha_lut_dc[i].brightness >= brightness)
			break;
	}

	if (i == 0)
		alpha = brightness_seed_alpha_lut_dc[0].alpha;
	else if (i == level)
		alpha = brightness_seed_alpha_lut_dc[level - 1].alpha;
	else
		alpha = interpolate(brightness,
			brightness_seed_alpha_lut_dc[i-1].brightness,
			brightness_seed_alpha_lut_dc[i].brightness,
			brightness_seed_alpha_lut_dc[i-1].alpha,
			brightness_seed_alpha_lut_dc[i].alpha);

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
	if (!oplus_mtk_drm_get_hbm_state())
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

int oplus_dc_alpha = 0;
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

unsigned long silence_mode = 0;
static ssize_t silence_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	printk("%s silence_mode=%ld\n", __func__, silence_mode);
	return sprintf(buf, "%ld\n", silence_mode);
}

static ssize_t silence_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t num)
{
	int ret;
	msleep(1000);
	ret = kstrtoul(buf, 10, &silence_mode);
	printk("%s silence_mode=%ld\n", __func__, silence_mode);
	return num;
}

/*unsigned long CABC_mode = 2;
*
* add dre only use for camera
*
* extern void disp_aal_set_dre_en(int enable);*/

/*static ssize_t LCM_CABC_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    printk("%s CABC_mode=%ld\n", __func__, CABC_mode);
    return sprintf(buf, "%ld\n", CABC_mode);
}

static ssize_t LCM_CABC_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t num)
{
    int ret = 0;

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

    if (oplus_display_cabc_support) {
        ret = primary_display_set_cabc_mode((unsigned int)CABC_mode);
    }

    return num;
}*/

static ssize_t oplus_display_get_ESD(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
	printk("%s esd=%ld\n", __func__, esd_mode);
	return sprintf(buf, "%ld\n", esd_mode);
}

static ssize_t oplus_display_set_ESD(struct kobject *kobj,
        struct kobj_attribute *attr, const char *buf, size_t num)
{
	int ret = 0;

	ret = kstrtoul(buf, 10, &esd_mode);
	printk("%s,esd mode is %d\n", __func__, esd_mode);
	return num;
}

unsigned long cabc_mode = 1;
unsigned long cabc_true_mode = 1;
unsigned long cabc_sun_flag = 0;
unsigned long cabc_back_flag = 1;
extern void disp_aal_set_dre_en(int enable);

enum{
	CABC_LEVEL_0,
	CABC_LEVEL_1,
	CABC_LEVEL_2 = 3,
	CABC_EXIT_SPECIAL = 8,
	CABC_ENTER_SPECIAL = 9,
};

static ssize_t oplus_display_get_CABC(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
	printk("%s CABC_mode=%ld\n", __func__, cabc_true_mode);
	return sprintf(buf, "%ld\n", cabc_true_mode);
}

static ssize_t oplus_display_set_CABC(struct kobject *kobj,
        struct kobj_attribute *attr, const char *buf, size_t num)
{
	int ret = 0;
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();

	ret = kstrtoul(buf, 10, &cabc_mode);
	cabc_true_mode = cabc_mode;
	printk("%s,cabc mode is %d, cabc_back_flag is %d\n", __func__, cabc_mode, cabc_back_flag);
	if(cabc_mode < 4)
		cabc_back_flag = cabc_mode;

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
		return num;
	}

	printk("%s,cabc mode is %d\n", __func__, cabc_true_mode);

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return 0;
	}
	if (cabc_true_mode == CABC_LEVEL_0 && cabc_back_flag == CABC_LEVEL_0) {
		disp_aal_set_dre_en(1);
		printk("%s enable dre\n", __func__);
	} else {
		disp_aal_set_dre_en(0);
		printk("%s disable dre\n", __func__);
	}
	oplus_mtk_drm_setcabc(crtc, cabc_true_mode);
	if (cabc_true_mode != cabc_back_flag) cabc_true_mode = cabc_back_flag;

	return num;
}

unsigned char aod_area_set_flag = 0;
static ssize_t oplus_display_set_aod_area(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	char *bufp = (char *)buf;
	char *token;
	int i, cnt = 0;
	char payload[RAMLESS_AOD_PAYLOAD_SIZE];

	memset(oplus_aod_area, 0, sizeof(struct aod_area) * RAMLESS_AOD_AREA_NUM);

	pr_err("yzq: %s %d\n", __func__, __LINE__);
	while ((token = strsep(&bufp, ":")) != NULL) {
		struct aod_area *area = &oplus_aod_area[cnt];
		if (!*token)
			continue;

		sscanf(token, "%d %d %d %d %d %d %d %d",
			&area->x, &area->y, &area->w, &area->h,
			&area->color, &area->bitdepth, &area->mono, &area->gray);
		pr_err("yzq: %s %d rect[%dx%d-%dx%d]-%d-%d-%d-%x\n", __func__, __LINE__,
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
	for(i = 0; i< 44; i++){
		pr_err("payload[%d] = 0x%x- send_cmd[%d] = 0x%x-", i,payload[i],i,send_cmd[i]);
		send_cmd[i+1] = payload[i];
	}
	aod_area_set_flag = 1;
	return count;
}

static ssize_t oplus_get_aod_light_mode(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
        printk(KERN_INFO "oplus_get_aod_light_mode = %d\n", aod_light_mode);

        return sprintf(buf, "%d\n", aod_light_mode);
}

/*unsigned char aod_lightmode_set_flag = 0;*/
static ssize_t oplus_set_aod_light_mode(struct kobject *kobj,
                struct kobj_attribute *attr,
                const char *buf, size_t count) {
        unsigned int temp_save = 0;
        int ret = 0;

        ret = kstrtouint(buf, 10, &temp_save);
        if (oplus_mtk_drm_get_hbm_state()) {
                printk(KERN_INFO "oplus_set_aod_light_mode = %d return on hbm\n", temp_save);
                return count;
        }
        /*aod_light_mode = temp_save;
        aod_lightmode_set_flag = 1;*/
        printk(KERN_INFO "oplus_set_aod_light_mode = %d\n", temp_save);
        if (temp_save != aod_light_mode) {
                printk(KERN_INFO " set aod backlight to %s nit\n", (temp_save == 0)?"50":"10");
                mtkfb_set_aod_backlight_level(temp_save);
                aod_light_mode = temp_save;
        }

        return count;
}

static ssize_t oplus_display_get_seed(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
        printk(KERN_INFO "seed_mode = %d\n", seed_mode);

        return sprintf(buf, "%d\n", seed_mode);
}

/*unsigned char aod_lightmode_set_flag = 0;*/
static ssize_t oplus_display_set_seed(struct kobject *kobj,
                struct kobj_attribute *attr,
                const char *buf, size_t count) {
	struct drm_crtc *crtc;
	unsigned int temp_save = 0;
	int ret = 0;
	struct drm_device *ddev = get_drm_device();

	ret = kstrtouint(buf, 10, &temp_save);
	printk(KERN_INFO "seed = %d\n", temp_save);
	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return 0;
	}

	oplus_mtk_drm_setseed(crtc, temp_save);
	seed_mode = temp_save;

	return count;
}

static ssize_t oplus_display_get_osc(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
        printk(KERN_INFO "osc_mode = %d\n", osc_mode);

        return sprintf(buf, "%d\n", osc_mode);
}

static ssize_t oplus_display_set_osc(struct kobject *kobj,
                struct kobj_attribute *attr,
                const char *buf, size_t count) {
	struct drm_crtc *crtc;
	unsigned int temp_save = 0;
	int ret = 0;
	struct drm_device *ddev = get_drm_device();

	ret = kstrtouint(buf, 10, &temp_save);
	printk(KERN_INFO "osc mode = %d\n", temp_save);

	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return 0;
	}

	mtk_crtc_osc_freq_switch(crtc, temp_save, 0);
	osc_mode = temp_save;

	return count;
}

static ssize_t oplus_display_get_mipi_hopping(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
        printk(KERN_INFO "mipi_hopping_mode = %d\n", mipi_hopping_mode);

        return sprintf(buf, "%d\n", mipi_hopping_mode);
}

static ssize_t oplus_display_set_mipi_hopping(struct kobject *kobj,
                struct kobj_attribute *attr,
                const char *buf, size_t count) {
	struct drm_crtc *crtc;
	unsigned int temp_save = 0;
	int ret = 0;
	struct drm_device *ddev = get_drm_device();

	ret = kstrtouint(buf, 10, &temp_save);
	printk(KERN_INFO "mipi_hopping_mode = %d\n", temp_save);

	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return 0;
	}

	mtk_crtc_mipi_freq_switch(crtc, temp_save, 0);
	mipi_hopping_mode = temp_save;
	mtk_crtc_osc_freq_switch(crtc, mipi_hopping_mode, 0);
	return count;
}
static struct kobject *oplus_display_kobj;
static struct kobj_attribute dev_attr_oplus_brightness = __ATTR(oplus_brightness, S_IRUGO|S_IWUSR, oplus_display_get_brightness, oplus_display_set_brightness);
static struct kobj_attribute dev_attr_oplus_max_brightness = __ATTR(oplus_max_brightness, S_IRUGO|S_IWUSR, oplus_display_get_max_brightness, NULL);
static struct kobj_attribute dev_attr_max_brightness = __ATTR(max_brightness, S_IRUGO|S_IWUSR, oplus_display_get_maxbrightness, NULL);
static struct kobj_attribute dev_attr_hbm = __ATTR(hbm, S_IRUGO|S_IWUSR, oplus_display_get_hbm, oplus_display_set_hbm);
static struct kobj_attribute dev_attr_seed = __ATTR(seed, S_IRUGO|S_IWUSR, oplus_display_get_seed, oplus_display_set_seed);
static struct kobj_attribute dev_attr_osc = __ATTR(osc, S_IRUGO|S_IWUSR, oplus_display_get_osc, oplus_display_set_osc);
static struct kobj_attribute dev_attr_mipi_hopping = __ATTR(mipi_hopping, S_IRUGO|S_IWUSR, oplus_display_get_mipi_hopping, oplus_display_set_mipi_hopping);
static struct kobj_attribute dev_attr_panel_pwr = __ATTR(panel_pwr, S_IRUGO|S_IWUSR, oplus_display_get_panel_pwr, oplus_display_set_panel_pwr);
static struct kobj_attribute dev_attr_oplus_notify_fppress = __ATTR(oplus_notify_fppress, S_IRUGO|S_IWUSR, NULL, fingerprint_notify_trigger);
static struct kobj_attribute dev_attr_dim_alpha = __ATTR(dim_alpha, S_IRUGO|S_IWUSR, oplus_display_get_dim_alpha, oplus_display_set_dim_alpha);
static struct kobj_attribute dev_attr_dimlayer_bl_en = __ATTR(dimlayer_bl_en, S_IRUGO|S_IWUSR, oplus_display_get_dc_enable, oplus_display_set_dc_enable);
static struct kobj_attribute dev_attr_dim_dc_alpha = __ATTR(dim_dc_alpha, S_IRUGO|S_IWUSR, oplus_display_get_dim_dc_alpha, oplus_display_set_dim_dc_alpha);
static struct kobj_attribute dev_attr_panel_serial_number = __ATTR(panel_serial_number, S_IRUGO|S_IWUSR, oplus_get_panel_serial_number, panel_serial_store);
static struct kobj_attribute dev_attr_LCM_CABC = __ATTR(LCM_CABC, S_IRUGO|S_IWUSR, oplus_display_get_CABC, oplus_display_set_CABC);
static struct kobj_attribute dev_attr_esd = __ATTR(esd, S_IRUGO|S_IWUSR, oplus_display_get_ESD, oplus_display_set_ESD);
static struct kobj_attribute dev_attr_sau_closebl_node = __ATTR(sau_closebl_node, S_IRUGO|S_IWUSR, silence_show, silence_store);
static struct kobj_attribute dev_attr_aod_light_mode_set = __ATTR(aod_light_mode_set, S_IRUGO|S_IWUSR, oplus_get_aod_light_mode, oplus_set_aod_light_mode);
static struct kobj_attribute dev_attr_aod_area = __ATTR(aod_area, S_IRUGO|S_IWUSR, NULL, oplus_display_set_aod_area);

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *oplus_display_attrs[] = {
	&dev_attr_oplus_brightness.attr,
	&dev_attr_oplus_max_brightness.attr,
	&dev_attr_max_brightness.attr,
	&dev_attr_hbm.attr,
	&dev_attr_seed.attr,
	&dev_attr_osc.attr,
	&dev_attr_mipi_hopping.attr,
	&dev_attr_panel_pwr.attr,
	&dev_attr_oplus_notify_fppress.attr,
	&dev_attr_dim_alpha.attr,
	&dev_attr_dimlayer_bl_en.attr,
	&dev_attr_dim_dc_alpha.attr,
	&dev_attr_panel_serial_number.attr,
	&dev_attr_LCM_CABC.attr,
	&dev_attr_esd.attr,
	&dev_attr_sau_closebl_node.attr,
	&dev_attr_aod_light_mode_set.attr,
	&dev_attr_aod_area.attr,
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
