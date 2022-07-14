/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_panel_power.c
** Description : oplus display panel power control
** Version : 1.0
** Date : 2020/06/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Sheng       2020/06/13        1.0           Build this moudle
******************************************************************/
#include "oplus_display_panel_power.h"
#include <linux/printk.h>
#include <linux/string.h>
#include <drm/drm_panel.h>
#include "display_panel/oplus_display_panel.h"

PANEL_VOLTAGE_BAK panel_vol_bak[PANEL_VOLTAGE_ID_MAX] = {{0}, {0}, {2, 0, 1, 2, ""}};
u32 panel_need_recovery = 0;
struct drm_panel *p_node = NULL;

int oplus_export_drm_panel(struct drm_panel *panel_node)
{
	printk("%s panel_node = %p\n", __func__, panel_node);
	if (panel_node) {
		p_node = panel_node;
		return 0;
	} else {
		printk("%s error :panel node is null, check the lcm driver!\n", __func__);
		return -1;
	}
}
EXPORT_SYMBOL(oplus_export_drm_panel);

int oplus_panel_set_vg_base(unsigned int panel_vol_value)
{
	int ret = 0;

	if (panel_vol_value > panel_vol_bak[PANEL_VOLTAGE_ID_VG_BASE].voltage_max ||
		panel_vol_value < panel_vol_bak[PANEL_VOLTAGE_ID_VG_BASE].voltage_min) {
		printk("%s error: panel_vol exceeds the range\n", __func__);
		panel_need_recovery = 0;
		return -EINVAL;
	}

	if (panel_vol_value == panel_vol_bak[PANEL_VOLTAGE_ID_VG_BASE].voltage_current) {
		printk("%s panel_vol the same as before\n", __func__);
		panel_need_recovery = 0;
	} else {
		printk("%s set panel_vol = %d\n", __func__, panel_vol_value);
		panel_vol_bak[PANEL_VOLTAGE_ID_VG_BASE].voltage_current = panel_vol_value;
		panel_need_recovery = 1;
	}

	return ret;
}

int dsi_panel_parse_panel_power_cfg(struct panel_voltage_bak *panel_vol)
{
	int ret = 0;
	printk("%s test", __func__);
	if (panel_vol == NULL) {
		printk("%s error handle", __func__);
		return -1;
	}

	memcpy((void *)panel_vol_bak, panel_vol, sizeof(struct panel_voltage_bak)*PANEL_VOLTAGE_ID_MAX);

	return ret;
}
EXPORT_SYMBOL(dsi_panel_parse_panel_power_cfg);

int oplus_panel_need_recovery(unsigned int panel_vol_value)
{
	int ret = 0;

	if (panel_need_recovery == 1) {
		printk("%s \n", __func__);
		ret = 1;
	}

	return ret;
}
EXPORT_SYMBOL(oplus_panel_need_recovery);

int oplus_display_panel_set_pwr(void *buf)
{
	struct panel_vol_set *panel_vol = buf;
	u32 panel_vol_value = 0, panel_vol_id = 0;
	int rc = 0;

	panel_vol_id = ((panel_vol->panel_id & 0x0F)-1);
	panel_vol_value = panel_vol->panel_vol;

	pr_err("debug for %s, buf = [%s], id = %d value = %d\n",
		__func__, buf, panel_vol_id, panel_vol_value);

	if (panel_vol_id < 0 || panel_vol_id > PANEL_VOLTAGE_ID_MAX) {
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

		return 0;
	}

	if (p_node && p_node->funcs->oplus_set_power) {
		rc = p_node->funcs->oplus_set_power(panel_vol_id, panel_vol_value);
		if (rc) {
			pr_err("Set voltage(%s) fail, rc=%d\n",
				 __func__, rc);
			return -EINVAL;
		}

		return 0;
	}

	return -EINVAL;
}

int oplus_display_panel_get_pwr(void *buf)
{
	int ret = 0;
	u32 i = 0;

	struct panel_vol_get *panel_vol = buf;
	int pid = (panel_vol->panel_id - 1);
	pr_err("%s : [id] = %d\n", __func__, pid);

	panel_vol->panel_min = panel_vol_bak[pid].voltage_min;
	panel_vol->panel_max = panel_vol_bak[pid].voltage_max;
	panel_vol->panel_cur = panel_vol_bak[pid].voltage_current;

	if (pid >= 0 && pid < (PANEL_VOLTAGE_ID_MAX-1)) {
		if (p_node && p_node->funcs->oplus_update_power_value) {
			ret = p_node->funcs->oplus_update_power_value(panel_vol_bak[pid].voltage_id);
		}

		if (ret < 0) {
			pr_err("%s : update_current_voltage error = %d\n", __func__, ret);
		}
		else {
			panel_vol_bak[i].voltage_current = ret;
			panel_vol->panel_cur = panel_vol_bak[pid].voltage_current;
			ret = 0;
		}
	}

	return ret;
}


