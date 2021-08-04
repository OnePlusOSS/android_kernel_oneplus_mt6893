/***************************************************
 * File:touch.c
 * Copyright (c)  2008- 2030  oplus Mobile communication Corp.ltd.
 * Description:
 *             tp dev
 * Version:1.0:
 * Date created:2016/09/02
 * TAG: BSP.TP.Init
*/

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include "oplus_touchscreen/tp_devices.h"
#include "oplus_touchscreen/touchpanel_common.h"
#include <soc/oplus/system/oplus_project.h>
#include <soc/oplus/device_info.h>


#define MAX_LIMIT_DATA_LENGTH         100
extern char *saved_command_line;
int g_tp_dev_vendor = TP_UNKNOWN;
int g_tp_prj_id = 0;

/*if can not compile success, please update vendor/oplus_touchsreen*/
struct tp_dev_name tp_dev_names[] = {
	{TP_OFILM, "OFILM"},
	{TP_BIEL, "BIEL"},
	{TP_TRULY, "TRULY"},
	{TP_BOE, "BOE"},
	{TP_G2Y, "G2Y"},
	{TP_TPK, "TPK"},
	{TP_JDI, "JDI"},
	{TP_TIANMA, "TIANMA"},
	{TP_SAMSUNG, "SAMSUNG"},
	{TP_DSJM, "DSJM"},
	{TP_BOE_B8, "BOEB8"},
	{TP_INNOLUX, "INNOLUX"},
	{TP_HIMAX_DPT, "DPT"},
	{TP_AUO, "AUO"},
	{TP_DEPUTE, "DEPUTE"},
	{TP_HUAXING, "HUAXING"},
	{TP_HLT, "HLT"},
	{TP_DJN, "DJN"},
	{TP_UNKNOWN, "UNKNOWN"},
};

#define GET_TP_DEV_NAME(tp_type) ((tp_dev_names[tp_type].type == (tp_type))?tp_dev_names[tp_type].name:"UNMATCH")

#ifndef CONFIG_MTK_FB
void primary_display_esd_check_enable(int enable)
{
	return;
}
EXPORT_SYMBOL(primary_display_esd_check_enable);
#endif /*CONFIG_MTK_FB*/

bool __init tp_judge_ic_match(char *tp_ic_name)
{
	return true;
}

bool  tp_judge_ic_match_commandline(struct panel_info *panel_data)
{
	int prj_id = 0;
	int i = 0;
	bool ic_matched = false;
	prj_id = get_project();
	pr_err("[TP] get_project() = %d \n", prj_id);
	pr_err("[TP] boot_command_line = %s \n", saved_command_line);

	for (i = 0; i < panel_data->project_num; i++) {
		if (prj_id == panel_data->platform_support_project[i]) {
			g_tp_prj_id = panel_data->platform_support_project_dir[i];

			if (strstr(saved_command_line, panel_data->platform_support_commandline[i])
					|| strstr("default_commandline", panel_data->platform_support_commandline[i])) {
				pr_err("[TP] Driver match the project\n");
				ic_matched = true;
				return true;
			}
		}
	}

	if (!ic_matched) {
		pr_err("[TP] Driver does not match the project\n");
		pr_err("Lcd module not found\n");
		return false;
	}

	return true;
}

int tp_util_get_vendor(struct hw_resource *hw_res,
		       struct panel_info *panel_data)
{
	char *vendor;
	int prj_id = 0;
	prj_id = get_project();

	panel_data->test_limit_name = kzalloc(MAX_LIMIT_DATA_LENGTH, GFP_KERNEL);

	if (panel_data->test_limit_name == NULL) {
		pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
	}

	panel_data->extra = kzalloc(MAX_LIMIT_DATA_LENGTH, GFP_KERNEL);

	if (panel_data->extra == NULL) {
		pr_err("[TP]panel_data.extra kzalloc error\n");
	}


	if (panel_data->tp_type == TP_UNKNOWN) {
		pr_err("[TP]%s type is unknown\n", __func__);
		return 0;
	}
	vendor = GET_TP_DEV_NAME(panel_data->tp_type);

	if (prj_id == 20171 || prj_id == 20172 || prj_id == 20353) {
		memcpy(panel_data->manufacture_info.version, "0xaa0100000", 11);
	}

	if (prj_id == 20817 || prj_id == 20827 || prj_id == 20831) {
		memcpy(panel_data->manufacture_info.version, "0xaa4110000", 11);
	}


	if (strstr(saved_command_line, "20171_tianma_nt37701")) {
		hw_res->TX_NUM = 18;
		hw_res->RX_NUM = 40;
		vendor = "TIANMA";
		pr_info("[TP]hw_res->TX_NUM:%d,hw_res->RX_NUM:%d\n",
		    hw_res->TX_NUM,
		    hw_res->RX_NUM);
	}

	strcpy(panel_data->manufacture_info.manufacture, vendor);

	switch (prj_id) {
	case 20171:
	case 20172:
	case 20353:
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
			"tp/%d/FW_%s_%s.%s",
			g_tp_prj_id, panel_data->chip_name, vendor, !strcmp(vendor,
			"SAMSUNG") ? "bin" : "img");

		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
			"tp/%d/LIMIT_%s_%s.img",
			g_tp_prj_id, panel_data->chip_name, vendor);
		}
		panel_data->manufacture_info.fw_path = panel_data->fw_name;
		break;

	default:
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
			 "tp/%d/FW_%s_%s.img",
			 g_tp_prj_id, panel_data->chip_name, vendor);

		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
				 "tp/%d/LIMIT_%s_%s.img",
				 g_tp_prj_id, panel_data->chip_name, vendor);
		}

		if (panel_data->extra) {
			snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
				 "tp/%d/BOOT_FW_%s_%s.ihex",
				 prj_id, panel_data->chip_name, vendor);
		}

		panel_data->manufacture_info.fw_path = panel_data->fw_name;
		break;
	}

	pr_info("[TP]vendor:%s fw:%s limit:%s\n",
		vendor,
		panel_data->fw_name,
		panel_data->test_limit_name == NULL ? "NO Limit" : panel_data->test_limit_name);

	return 0;
}

