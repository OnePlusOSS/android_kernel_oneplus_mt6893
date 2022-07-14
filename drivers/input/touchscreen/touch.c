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
#include "touch.h"


#define MAX_LIMIT_DATA_LENGTH         100
extern char *saved_command_line;
int g_tp_dev_vendor = TP_UNKNOWN;
int g_tp_prj_id = 0;
struct hw_resource *g_hw_res;
static bool is_tp_type_got_in_match = false;    /*indicate whether the tp type is specified in the dts*/
int tp_type = 0;
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
	{TP_BOE_B3, "BOE"},
	{TP_CDOT, "CDOT"},
     {TP_INX, "INX"},
     {TP_INX, "TXD"},
     {TP_UNKNOWN, "UNKNOWN"},
};

typedef enum {
	TP_INDEX_NULL,
	himax_83112a,
	himax_83112f,
	ili9881_auo,
	ili9881_tm,
	nt36525b_boe,
    nt36525b_inx,
    ili9882n_cdot,
    ili9882n_hlt,
    ili9882n_inx,
    nt36525b_hlt,
    nt36672c,
    ili9881_inx,
    goodix_gt9886,
    focal_ft3518,
    td4330,
    himax_83112b,
    himax_83102d,
    ili7807s_tm,
    ili7807s_tianma,
    ili7807s_jdi,
    hx83102d_txd,
    ft8006s_truly,
    ili9882n_truly,
    ili7807s_boe,
    ili7807s_hlt,
	nt36672c_boe,
} TP_USED_INDEX;
TP_USED_INDEX tp_used_index  = TP_INDEX_NULL;

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
	pr_err("[TP] tp_ic_name = %s \n", tp_ic_name);
	switch(get_project()) {
	case 0x210A0:
		pr_info("[TP] case 210A0\n");
		if (strstr(tp_ic_name, "ilitek,ili7807s") && strstr(saved_command_line, "ili7807s_tm")) {
			pr_info("[TP] touch judge ic = ilitek,ili7807s,TIANMA\n");
			return true;
		}
		if (strstr(tp_ic_name, "ilitek,ili7807s") && strstr(saved_command_line, "ili7807s_jdi")) {
			pr_info("[TP] touch judge ic = ilitek,ili7807s,JDI\n");
			return true;
		}
		pr_info("tp judge ic match failed 210A0\n");
		return false;
	case 20131:
		if(strstr(saved_command_line, "oplus20131_tianma_nt37701_1080p_dsi_cmd")) {
			pr_info("tp judge ic match 20131 oplus20131_tianma_nt37701_1080p_dsi_cmd\n");
			return true;
		} else if (strstr(saved_command_line, "oplus20131_tianma_nt37701_32_1080p_dsi_cmd")) {
			pr_info("tp judge ic match 20131 oplus20131_tianma_nt37701_32_1080p_dsi_cmd\n");
			return true;
		} else if (strstr(saved_command_line, "oplus20131_boe_nt37800_1080p_dsi_cmd")) {
			pr_info("tp judge ic match 20131 oplus20131_boe_nt37800_1080p_dsi_cmd\n");
			return true;
		} else if (strstr(saved_command_line, "oplus20131_samsung_ana6705_1080p_dsi_cmd")) {
			pr_info("tp judge ic match 20131 oplus20131_samsung_ana6705_1080p_dsi_cmd\n");
			return true;
		}
		pr_info("tp judge ic match failed 20131\n");
		return false;
	case 20645:
		if(strstr(saved_command_line, "oplus20645_tianma_nt37701_1080p_dsi_cmd")) {
			pr_info("tp judge ic match 20645 oplus20645_tianma_nt37701_1080p_dsi_cmd\n");
			return true;
		} else if (strstr(saved_command_line, "oplus20645_tianma_nt37701_32_1080p_dsi_cmd")) {
			pr_info("tp judge ic match 20645 oplus20645_tianma_nt37701_32_1080p_dsi_cmd\n");
			return true;
		} else if (strstr(saved_command_line, "oplus20645_boe_nt37800_1080p_dsi_cmd")) {
			pr_info("tp judge ic match 20645 oplus20645_boe_nt37800_1080p_dsi_cmd\n");
			return true;
		} else if (strstr(saved_command_line, "oplus20645_samsung_ana6705_1080p_dsi_cmd")) {
			pr_info("tp judge ic match 20645 oplus20645_samsung_ana6705_1080p_dsi_cmd\n");
			return true;
		}
		pr_info("tp judge ic match failed oplus20645\n");
		return false;
	case 20613:
	case 20001:
	case 20002:
	case 20003:
	case 20200:
		pr_info("tp judge ic forward for 20001\n");
		if (strstr(tp_ic_name, "nf_nt36672c") && strstr(saved_command_line, "nt36672c")) {
			pr_info("tp judge ic nf_nt36672c 20001\n");
			return true;
		}
		if (strstr(tp_ic_name, "hx83112f_nf") && strstr(saved_command_line, "hx83112f")) {
			pr_info("tp judge ic hx83112f 20001\n");
			return true;
		}
		if (strstr(tp_ic_name, "ili7807s") && strstr(saved_command_line, "ili7807s")) {
			pr_info("tp judge ic ili7807s 20001\n");
			return true;
		}
		pr_info("tp judge ic match failed 20001\n");
		return false;
    	case 20611:
    	case 20610:
    	case 20680:
	case 20686:
		pr_info("tp judge ic forward for 20611\n");
		if (strstr(tp_ic_name, "nf_nt36672c") && strstr(saved_command_line, "nt36672c")) {
			return true;
		}
		if (strstr(tp_ic_name, "hx83112f_nf") && strstr(saved_command_line, "hx83112f")) {
			return true;
		}
		return false;
	case 20630:
	case 20631:
	case 20632:
	case 20633:
	case 20634:
	case 20635:
	case 0x206B4:
		pr_info("[TP] case 20630\n");
		is_tp_type_got_in_match = true;
		if (strstr(tp_ic_name, "Goodix-gt9886")&&strstr(saved_command_line, "hx83112f_fhdp_dsi_vdo_dphy_tianma_lcm_drv")) {
			pr_info("[TP] Goodix-gt9886\n");
			tp_type = 1;
            g_tp_dev_vendor = TP_SAMSUNG;
            return true;
        }
		if (strstr(tp_ic_name, "focaltech,fts")&&strstr(saved_command_line, "s68fc01")) {
			pr_info("[TP] focaltech 3518\n");
			tp_type = 0;
            g_tp_dev_vendor = TP_SAMSUNG;
            return true;
        }
		break;
	case 20608:
	case 20609:
	case 0x2060A:
	case 0x2060B:
	case 0x206F0:
	case 0x206FF:
	case 20796:
	case 20795:
	case 0x2070B:
	case 0x2070C:
	case 0x2070E:
	case 21625:
	case 0x216A0:
	case 0x216A1:
	case 0x216B5:
	case 0x216B6:
		pr_info("[TP] case 20609\n");
		if (strstr(tp_ic_name, "novatek,nf_nt36672c") && strstr(saved_command_line, "oplus20609_nt36672c_tm_cphy_dsi_vdo_lcm_drv")) {
			pr_info("[TP] touch judge ic = novatek,nf_nt36672c,TIANMA\n");
			return true;
		}
		if (strstr(tp_ic_name, "ilitek,ili7807s") && strstr(saved_command_line, "oplus20609_ili7807s_tm_cphy_dsi_vdo_lcm_drv")) {
			pr_info("[TP] touch judge ic = ilitek,ili9882n(ili7807s),TIANMA\n");
			return true;
		}
		return false;
	case 20637:
	case 20638:
	case 20639:
	case 0x206B7:
		pr_info("[TP] case 20638\n");
		is_tp_type_got_in_match = true;
		/*if (strstr(tp_ic_name, "focaltech,fts")&&strstr(saved_command_line, "oplus20638_s68fc01_lcm_drv")) {*/
			pr_info("[TP] focaltech 3518\n");
			tp_type = 0;
			g_tp_dev_vendor = TP_SAMSUNG;
			return true;
		/*}*/
		break;
	case 20095:
		pr_info("tp judge ic forward for 20095\n");
		if (strstr(tp_ic_name, "nf_nt36672c") && strstr(saved_command_line, "nt36672c")) {
			return true;
		}
		if (strstr(tp_ic_name, "ilitek,ili7807s") && strstr(saved_command_line, "ili7807s")) {
			return true;
		}
		return false;
	case 19131:
	case 19132:
	case 19420:
		pr_info("tp judge ic forward for 19131\n");
		if (strstr(tp_ic_name, "nf_nt36672c") && strstr(saved_command_line, "nt36672c_fhdp_dsi_vdo")) {
			return true;
		}
		if (strstr(tp_ic_name, "hx83112f_nf") && strstr(saved_command_line, "hx83112f_fhdp_dsi_vdo")) {
			return true;
		}
		return false;
	case 21101:
	case 21102:
	case 21235:
	case 21236:
	case 21831:
		if (strstr(tp_ic_name, "ilitek,ili7807s") && strstr(saved_command_line, "oplus21101_ili9883a")) {
			return true;
		}

		if (strstr(tp_ic_name, "td4160") && strstr(saved_command_line, "oplus21101_td4160")) {
			return true;
		}
		return false;
	case 21639:
	case 0x2163B:
	case 0x2163C:
	case 0x2163D:
	case 0x216CD:
	case 0x216CE:
		if (strstr(tp_ic_name, "ilitek,ili7807s") && strstr(saved_command_line, "oplus2163b_ili9883a")) {
			return true;
		}

		if (strstr(tp_ic_name, "td4160") && strstr(saved_command_line, "oplus2163b_td4160_inx_hd_vdo_lcm_drv")) {
			return true;
		}

		if (strstr(tp_ic_name, "novatek,nf_nt36672c") && strstr(saved_command_line, "oplus21639_nt36672c_tm_cphy_dsi_vdo_lcm_drv")) {
			return true;
		}

		if (strstr(tp_ic_name, "td4160") && strstr(saved_command_line, "oplus2163b_td4160_truly_hd_vdo_lcm_drv")) {
			return true;
		}
		return false;
	case 20015:
	case 20016:
	case 20651:
	case 20652:
	case 20653:
	case 20654:
	case 20108:
	case 20109:
	case 20307:
		pr_info("[TP] tp_judge_ic_match case 20015\n");
		if (strstr(tp_ic_name, "nf_nt36672c") && strstr(saved_command_line, "nt36672c")) {
			pr_info("tp judge ic match 20015 nf_nt36525b\n");
			return true;
		}
		if (strstr(tp_ic_name, "nf_nt36525b") && strstr(saved_command_line, "nt35625b_boe_hlt_b3_vdo_lcm_drv")) {
			pr_info("tp judge ic match 20015 nf_nt36525b\n");
			return true;
		}
		if (strstr(tp_ic_name, "ili7807s") && strstr(saved_command_line, "ili9882n")) {
			pr_info("tp judge ic match 20015 ili9882n\n");
			return true;
		}
		if (strstr(tp_ic_name, "hx83102") && strstr(saved_command_line, "hx83102_truly_vdo_hd_dsi_lcm_drv")) {
			pr_info("tp judge ic match 20015 hx83102\n");
			return true;
		}
		pr_info("tp judge ic match failed 20015\n");
		return false;
	case 21037:
		if(strstr(tp_ic_name, "nf_nt36525b") && strstr(saved_command_line, "oplus21037_nt35625b_innolux_dphy_dsi_vdo_lcm_drv")) {
			pr_info("tp judge ic match 21037 nf_nt36525b\n");
			return true;
		} else if (strstr(tp_ic_name, "hx83102d_nf") && strstr(saved_command_line, "oplus21037_hx83102d_txd_dphy_dsi_vdo_lcm_drv")) {
			pr_info("tp judge ic match 21037 hx83102d_nf\n");
			return true;
		} else if (strstr(tp_ic_name, "ili7807s") && strstr(saved_command_line, "oplus21037_ili9882n_innolux_dphy_dsi_vdo_lcm_drv")) {
			pr_info("tp judge ic match 21037 ili9882n\n");
			return true;
		} else if (strstr(tp_ic_name, "focaltech,fts") && strstr(saved_command_line, "oplus21037_ft8006_truly_dphy_dsi_vdo_lcm_drv")) {
			pr_info("tp judge ic match 21037 focaltech,fts\n");
			return true;
		}
		pr_info("tp judge ic match failed 21037\n");
		return false;
	case 21041:
		if(strstr(tp_ic_name, "nt36672c") && strstr(saved_command_line, "oplus21041_nt36672c_tm_cphy_dsi_vdo_lcm_drv")) {
			pr_info("tp judge ic match 21041 nt36672c\n");
			return true;
		} else if (strstr(tp_ic_name, "ili7807s") && strstr(saved_command_line, "oplus21041_ili7807s_tm_cphy_dsi_vdo_lcm_drv")) {
			pr_info("tp judge ic match 21041 ili7807s\n");
			return true;
		} else if (strstr(tp_ic_name, "ili7807s") && strstr(saved_command_line, "oplus21041_ili7807s_jdi_cphy_dsi_vdo_lcm_drv")) {
			pr_info("tp judge ic match 21041 ili9882n\n");
			return true;
		}
		pr_info("tp judge ic match failed 21041\n");
		return false;
	case 20730:
	case 20731:
	case 20732:
		pr_info("%s forward for 20730\n", __func__);
                if (strstr(tp_ic_name, "focaltech,fts") && strstr(saved_command_line, "oplus20730_samsung_ams643xy04_lcm_drv_1")) {
			return true;
		}

	        if (strstr(tp_ic_name, "Goodix-gt9886") && strstr(saved_command_line, "oplus20730_samsung_ams643xy04_lcm_drv_2")) {
		return true;
		}

		if (strstr(tp_ic_name, "Goodix-gt9886") && strstr(saved_command_line, "oplus20730_samsung_ams643xy04_lcm_drv_3")) {
		return true;
		}
                pr_err("[TP] ERROR! ic is not match driver\n");
		return false;
	case 20761:
	case 20762:
	case 20764:
	case 20767:
	case 20766:
	case 0x2167A:
	case 0x2167B:
	case 0x2167C:
	case 0x2167D:
	case 0x216AF:
	case 0x216B0:
	case 0x216B1:
		pr_info("tp judge ic for 2076x & 216xx\n");
		if (strstr(tp_ic_name, "ili9882n") && strstr(saved_command_line, "ilt9882n_truly_even_hdp_dsi_vdo_lcm")) {
			return true;
		}
		if (strstr(tp_ic_name, "ili9882n") && strstr(saved_command_line, "ilt7807s_hlt_even_hdp_dsi_vdo_lcm")) {
			return true;
		}
		if (strstr(tp_ic_name, "nf_nt36525b") && strstr(saved_command_line, "nt36525b_hlt")) {
			return true;
		}
		pr_err("[TP] ERROR! ic is not match driver\n");
		return false;
	case 21684:
	case 21685:
	case 21686:
	case 21687:
	case 21690:
	case 21691:
	case 21692:
		pr_info("[TP] case 21684\n");
		if (strstr(tp_ic_name, "ilitek,ili9882n") && strstr(saved_command_line, "ili7807s_fhdp_dsi_vdo_boe_boe_zal5603")) {
			pr_info("[TP] touch judge ic = ilitek,nf_ili7807s,BOE\n");
			return true;
		}
		if (strstr(tp_ic_name, "ilitek,ili9882n") && strstr(saved_command_line, "ili7807s_boe_60hz_fhdp_dsi_vdo_zal5603")) {
			pr_info("[TP] touch judge ic = ilitek,nf_ili7807s,HLT\n");
			return true;
		}
		break;
	case 20041:
	case 20042:
	case 20043:
		pr_info("tp judge ic forward for 20041\n");
		if (strstr(tp_ic_name, "novatek,nf_nt36672c") && strstr(saved_command_line, "nt36672c_fhdp_dsi_vdo_dphy_jdi_lcm_drv")) {
			return true;
		}
		if (strstr(tp_ic_name, "novatek,nf_nt36672c") && strstr(saved_command_line, "nt36672c_fhdp_dsi_vdo_dphy_boe_lcm_drv")) {
			return true;
		}
		if (strstr(tp_ic_name, "himax,hx83112f_nf") && strstr(saved_command_line, "hx83112f")) {
			return true;
		}
		if (strstr(tp_ic_name, "ilitek,ili7807s") && strstr(saved_command_line, "ili7807s")) {
			return true;
		}
		return false;
	default:
		break;
	}
	return true;
}

EXPORT_SYMBOL(tp_judge_ic_match);
bool  tp_judge_ic_match_commandline(struct panel_info *panel_data)
{
	int prj_id = 0;
	int i = 0;
	bool ic_matched = false;
	prj_id = get_project();

	pr_err("[TP] get_project() = %d \n", prj_id);
	pr_err("[TP] saved_command_line = %s \n", saved_command_line);

	for (i = 0; i < panel_data->project_num; i++) {
		if (prj_id == panel_data->platform_support_project[i]) {
			g_tp_prj_id = panel_data->platform_support_project_dir[i];
            pr_err("[TP] platform_support_commandline = %s \n", panel_data->platform_support_commandline[i]);
			if (strstr(saved_command_line, panel_data->platform_support_commandline[i])
					|| strstr("default_commandline", panel_data->platform_support_commandline[i])) {
				pr_err("[TP] Driver match the project\n");
				ic_matched = true;
			}
		}
	}

	if (!ic_matched) {
		pr_err("[TP] Driver does not match the project\n");
		pr_err("Lcd module not found\n");
		return false;
	}

	switch (prj_id) {
	case 20615:
	case 20662:
	case 20619:
	case 21609:
	case 21651:
		pr_info("[TP] case 20615/20662/20619/21609/21651\n");
		is_tp_type_got_in_match = true;
		pr_err("[TP] touch ic = focal_ft3518 \n");
		tp_used_index = focal_ft3518;
		g_tp_dev_vendor = TP_SAMSUNG;
		break;
	case 20761:
	case 20762:
	case 20764:
	case 20767:
	case 20766:
	case 0x2167A:
	case 0x2167B:
	case 0x2167C:
	case 0x2167D:
	case 0x216AF:
	case 0x216B0:
	case 0x216B1:
		is_tp_type_got_in_match = true;
		if (strstr(saved_command_line, "ilt9882n_truly_even_hdp_dsi_vdo_lcm")) {
			pr_err("[TP] touch ic = ilt9882n_truly_jdi \n");
			tp_used_index = ili9882n_truly;
			g_tp_dev_vendor = TP_TRULY;
		}
		if (strstr(saved_command_line, "ilt7807s_hlt_even_hdp_dsi_vdo_lcm")) {
			pr_err("[TP] touch ic = ilt7807S_hlt_jdi \n");
			tp_used_index = ili7807s_hlt;
			g_tp_dev_vendor = TP_HLT;
		}
		if (strstr(saved_command_line, "nt36525b_hlt_even_boe_hdp_dsi_vdo_lcm")) {
			g_tp_dev_vendor = TP_HLT;
			tp_used_index = nt36525b_hlt;
			pr_err("[TP] touch ic = nt36525b_hlt_b8\n");
		}
		break;
	case 20001:
	case 20002:
	case 20003:
	case 20200:
		pr_info("[TP] case 20001\n");
		is_tp_type_got_in_match = true;

		if (strstr(saved_command_line, "nt36672c")) {
			pr_err("[TP] touch ic = nt36672c_jdi \n");
			tp_used_index = nt36672c;
			g_tp_dev_vendor = TP_JDI;
		}

		if (strstr(saved_command_line, "hx83112f")) {
			pr_err("[TP] touch ic = hx83112f_tianma \n");
			tp_used_index = himax_83112f;
			g_tp_dev_vendor = TP_TIANMA;
		}
		if (strstr(saved_command_line, "ili7807s")) {
			tp_used_index = ili7807s_tm;
			g_tp_dev_vendor = TP_TIANMA;
		}
		break;
	case 20015:
	case 20016:
	case 20651:
	case 20652:
	case 20653:
	case 20654:
	case 20108:
	case 20109:
	case 20307:
		pr_info("[TP] case 20015\n");

		is_tp_type_got_in_match = true;

		if (strstr(saved_command_line, "nt36672c")) {
			pr_err("[TP] touch ic = nt36672c_jdi\n");
			tp_used_index = nt36672c;
			g_tp_dev_vendor = TP_JDI;
		}
		if (strstr(saved_command_line, "oplus20625_nt35625b_boe_hlt_b3_vdo_lcm_drv")) {
			pr_info("[TP] touch ic = novatek,nf_nt36525b\n");
			g_tp_dev_vendor = TP_INX;
			tp_used_index = nt36525b_boe;
		}
		if (strstr(saved_command_line, "oplus20015_ili9882n_truly_hd_vdo_lcm_drv")) {
			pr_info("[TP] touch ic = tchip,ilitek\n");
			g_tp_dev_vendor = TP_CDOT;
			tp_used_index = ili9882n_cdot;
		}
		if (strstr(saved_command_line, "oplus20015_ili9882n_boe_hd_vdo_lcm_drv")) {
			pr_info("[TP] touch ic = tchip,ilitek,HLT\n");
			g_tp_dev_vendor = TP_HLT;
			tp_used_index = ili9882n_hlt;
		}
		if (strstr(saved_command_line, "oplus20015_ili9882n_innolux_hd_vdo_lcm_drv")) {
			pr_info("[TP] touch ic = tchip,ilitek,INX\n");
			g_tp_dev_vendor = TP_INX;
			tp_used_index = ili9882n_inx;
		}
		if (strstr(saved_command_line, "oplus20015_hx83102_truly_vdo_hd_dsi_lcm_drv")) {
			pr_info("[TP] touch ic = himax,hx83102d_nf,TRULY\n");
			g_tp_dev_vendor = TP_TRULY;
			tp_used_index = himax_83102d;
		}
		break;
	case 20613:
	case 20611:
	case 20610:
	case 20680:
	case 20686:
		pr_info("[TP] case 20611\n");
		is_tp_type_got_in_match = true;
		if (strstr(saved_command_line, "nt36672c")) {
			pr_err("[TP] touch ic = nt36672c_jdi \n");
			tp_used_index = nt36672c;
			g_tp_dev_vendor = TP_JDI;
		}

		if (strstr(saved_command_line, "hx83112f")) {
			pr_err("[TP] touch ic = hx83112f_tianma \n");
			tp_used_index = himax_83112f;
			g_tp_dev_vendor = TP_TIANMA;
		}
        break;
	case 20095:
		pr_info("[TP] case 20095\n");
		is_tp_type_got_in_match = true;
		if (strstr(saved_command_line, "nt36672c")) {
			pr_err("[TP] touch ic = nt36672c \n");
			tp_used_index = nt36672c;
			g_tp_dev_vendor = TP_DSJM;
		}

		if (strstr(saved_command_line, "ili7807s")) {
			pr_err("[TP] touch ic = 7807s_tianma \n");
			tp_used_index = ili7807s_tm;
			g_tp_dev_vendor = TP_TIANMA;
		}
		break;
	case 20181:
	case 20355:
	case 20391:
	case 20392:
	case 21061:
	case 20827:
	case 20831:
	case 21081:
	case 0x212A1:
	case 21881:
	case 21882:
		pr_info("[TP] case %d,\n", prj_id);
		is_tp_type_got_in_match = true;
		pr_err("[TP] touch ic = focaltech3518 \n");
		tp_used_index = focal_ft3518;
		g_tp_dev_vendor = TP_SAMSUNG;
		break;

	case 21101:
	case 21102:
	case 21235:
	case 21236:
	case 21831:
                is_tp_type_got_in_match = true;

                if (strstr(saved_command_line, "oplus21101_ili9883a")) {
                        tp_used_index = ili9881_auo;
                        g_tp_dev_vendor = TP_BOE;
                }

                if (strstr(saved_command_line, "oplus21101_td4160")) {
                        pr_err("[TP] touch ic = td4160 \n");
                        tp_used_index = ili9881_auo;
                        g_tp_dev_vendor = TP_INX;
                }
                break;

	case 21639:
	case 0x2163B:
	case 0x2163C:
	case 0x2163D:
	case 0x216CD:
	case 0x216CE:
                is_tp_type_got_in_match = true;

                if (strstr(saved_command_line, "oplus2163b_ili9883a")) {
                        tp_used_index = ili9881_auo;
                        g_tp_dev_vendor = TP_BOE;
                }

                if (strstr(saved_command_line, "oplus2163b_td4160_inx_hd_vdo_lcm_drv")) {
                        pr_err("[TP] touch ic = td4160 \n");
                        tp_used_index = ili9881_auo;
                        g_tp_dev_vendor = TP_INX;
                }

                if (strstr(saved_command_line, "oplus21639_nt36672c_tm_cphy_dsi_vdo_lcm_drv")) {
                        pr_err("[TP] touch ic = nf_nt36672c \n");
                        tp_used_index = nt36672c;
                        g_tp_dev_vendor = TP_TIANMA;
                }

                if (strstr(saved_command_line, "oplus2163b_td4160_truly_hd_vdo_lcm_drv")) {
                        pr_err("[TP] touch ic = td4160 \n");
                        tp_used_index = ili9881_auo;
                        g_tp_dev_vendor = TP_TRULY;
                }
                break;

	case 19131:
	case 19132:
	case 19420:
		pr_info("[TP] case 19131\n");
		is_tp_type_got_in_match = true;

		if (strstr(saved_command_line, "nt36672c_fhdp_dsi_vdo_auo_cphy_90hz_tianma")) {
			pr_err("[TP] touch ic = nt36672c_tianma \n");
			tp_used_index = nt36672c;
			g_tp_dev_vendor = TP_TIANMA;
		}

		if (strstr(saved_command_line, "nt36672c_fhdp_dsi_vdo_auo_cphy_120hz_tianma")) {
			pr_err("[TP] touch ic = nt36672c_tianma \n");
			tp_used_index = nt36672c;
			g_tp_dev_vendor = TP_TIANMA;
		}

		if (strstr(saved_command_line, "nt36672c_fhdp_dsi_vdo_auo_cphy_90hz_jdi")) {
			pr_err("[TP] touch ic = nt36672c_jdi \n");
			tp_used_index = nt36672c;
			g_tp_dev_vendor = TP_JDI;
		}

		if (strstr(saved_command_line, "hx83112f_fhdp_dsi_vdo_auo_cphy_90hz_jdi")) {
			pr_err("[TP] touch ic = hx83112f_jdi \n");
			tp_used_index = himax_83112f;
			g_tp_dev_vendor = TP_JDI;
		}
		break;
	case 21037:
		pr_info("[TP] enter case thor-a 21037\n");
		is_tp_type_got_in_match = true;
		if (strstr(saved_command_line, "oplus21037_nt35625b_innolux")) {
			pr_info("[TP] touch ic = novatek,nf_nt36525b,inx\n");
			g_tp_dev_vendor = TP_INX;
			tp_used_index = nt36525b_inx;
		}
		if (strstr(saved_command_line, "oplus21037_hx83102d_txd")) {
			pr_info("[TP] touch ic = himax,hx83102d_nf,txd\n");
			g_tp_dev_vendor = TP_TXD;
			tp_used_index = hx83102d_txd;
		}
		if (strstr(saved_command_line, "oplus21037_ili9882n_innolux")) {
			pr_info("[TP] touch ic = ilitek,ili9882n inx\n");
			g_tp_dev_vendor = TP_INX;
			tp_used_index = ili9882n_inx;
		}
		if (strstr(saved_command_line, "oplus21037_ft8006_truly")) {
			pr_info("[TP] touch ic = focal,ft8006s truly\n");
			g_tp_dev_vendor = TP_TRULY;
			tp_used_index = ft8006s_truly;
		}
		break;
	case 21041:
	case 21042:
	case 21241:
	case 21242:
		pr_info("[TP] enter case odin-a 21041\n");
		is_tp_type_got_in_match = true;
		if (strstr(saved_command_line, "oplus21041_ili7807s_tm")) {
			pr_info("[TP] touch ic = novatek,ili9882n(ili7807s) tianma\n");
			g_tp_dev_vendor = TP_TIANMA;
			tp_used_index = ili7807s_tianma;
		}
		if (strstr(saved_command_line, "oplus21041_ili7807s_jdi")) {
			pr_info("[TP] touch ic = novatek,ili9882n(ili7807s) jdi\n");
			g_tp_dev_vendor = TP_JDI;
			tp_used_index = ili7807s_jdi;
		}
		break;
	case 0x210A0:
		pr_info("[TP] enter case 210A0\n");
		is_tp_type_got_in_match = true;
		if (strstr(saved_command_line, "ili7807s_tm")) {
			pr_info("[TP] touch ic = novatek,ili9882n(ili7807s) tianma\n");
			g_tp_dev_vendor = TP_TIANMA;
			tp_used_index = ili7807s_tianma;
		}
		if (strstr(saved_command_line, "ili7807s_jdi")) {
			pr_info("[TP] touch ic = novatek,ili9882n(ili7807s) jdi\n");
			g_tp_dev_vendor = TP_JDI;
			tp_used_index = ili7807s_jdi;
		}
		break;
	case 20730:
	case 20731:
	case 20732:
		pr_info("%s forward for 20730\n", __func__);
		is_tp_type_got_in_match = true;
		if (strstr(saved_command_line, "oplus20730_samsung_ams643xy04_lcm_drv_1")) {
			pr_err("[TP] touch ic = FT_3518 \n");
			tp_used_index = focal_ft3518;
			g_tp_dev_vendor = TP_SAMSUNG;
		}

		if (strstr(saved_command_line, "oplus20730_samsung_ams643xy04_lcm_drv_2")) {
			pr_err("[TP] touch ic = Goodix-gt9886 \n");
			tp_used_index =  goodix_gt9886;
			g_tp_dev_vendor = TP_SAMSUNG;
		}

		if (strstr(saved_command_line, "oplus20730_samsung_ams643xy04_lcm_drv_3")) {
			pr_err("[TP] touch ic = Goodix-gt9886 \n");
			tp_used_index =  goodix_gt9886;
			g_tp_dev_vendor = TP_SAMSUNG;
		}
		break;

	case 20608:
	case 20609:
	case 0x2060A:
	case 0x2060B:
	case 0x206F0:
	case 0x206FF:
	case 20796:
	case 20795:
	case 0x2070B:
	case 0x2070C:
	case 0x2070E:
	case 21625:
	case 0x216A0:
	case 0x216A1:
	case 0x216B5:
	case 0x216B6:
		pr_info("[TP] case 20609\n");
		is_tp_type_got_in_match = true;

		if (strstr(saved_command_line, "oplus20609_nt36672c_tm_cphy_dsi_vdo_lcm_drv")) {
			pr_info("[TP] touch ic = novatek,nf_nt6672c\n");
			g_tp_dev_vendor = TP_TIANMA;
			tp_used_index = nt36672c;
		}
		if (strstr(saved_command_line, "oplus20609_ili7807s_tm_cphy_dsi_vdo_lcm_drv")) {
			pr_info("[TP] touch ic = novatek,ili9882n(ili7807s)\n");
			g_tp_dev_vendor = TP_TIANMA;
			tp_used_index = ili9882n_inx;
		}
		break;
	case 21684:
	case 21685:
	case 21686:
	case 21687:
	case 21690:
	case 21691:
	case 21692:
		pr_info("[TP] case 21684\n");
		is_tp_type_got_in_match = true;

		if (strstr(saved_command_line, "ili7807s_fhdp_dsi_vdo_boe_boe_zal5603")) {
			pr_info("[TP] touch ic = ilitek,ili7807s\n");
			g_tp_dev_vendor = TP_BOE;
			tp_used_index = ili7807s_boe;
		}
		if (strstr(saved_command_line, "ili7807s_boe_60hz_fhdp_dsi_vdo_zal5603")) {
			pr_info("[TP] touch ic = ilitek,ili7807s\n");
			g_tp_dev_vendor = TP_HLT;
			tp_used_index = ili7807s_hlt;
		}
		break;
/* bringup add for sala touchscreen.*/
	case 20682:
		pr_info("[TP] case 20682\n");
		is_tp_type_got_in_match = true;
		tp_used_index = nt36672c;

		if (strstr(saved_command_line, "nt36672c_tianma")) {
			g_tp_dev_vendor = TP_TIANMA;

		} else if (strstr(saved_command_line, "nt36672c_jdi")) {
			g_tp_dev_vendor = TP_JDI;

		} else if (strstr(saved_command_line, "nt36672c_boe")) {
			g_tp_dev_vendor = TP_BOE;
		} else {
			g_tp_dev_vendor = TP_UNKNOWN;
		}

		pr_err("[TP] g_tp_dev_vendor: %s\n", tp_dev_names[g_tp_dev_vendor].name);
        break;
	/* bringup add for sala touchscreen.*/
	case 20041:
	case 20042:
	case 20043:
		pr_info("[TP] case 20041\n");
		is_tp_type_got_in_match = true;
		if (strstr(saved_command_line, "nt36672c_fhdp_dsi_vdo_dphy_jdi_lcm_drv")) {
			tp_used_index = nt36672c;
			g_tp_dev_vendor = TP_JDI;
		}
		if (strstr(saved_command_line, "nt36672c_fhdp_dsi_vdo_dphy_boe_lcm_drv")) {
			tp_used_index = nt36672c_boe;
			g_tp_dev_vendor = TP_BOE;
		}
		if (strstr(saved_command_line, "hx83112f")) {
			tp_used_index = himax_83112f;
			g_tp_dev_vendor = TP_TIANMA;
		}
		if (strstr(saved_command_line, "ili7807s")) {
			tp_used_index = ili7807s_tm;
			g_tp_dev_vendor = TP_TIANMA;
		}
		break;
	default:
		pr_info("other project, no need process here!\n");
		break;
	}

	pr_info("[TP]ic:%d, vendor:%d\n", tp_used_index, g_tp_dev_vendor);
	return true;
}
EXPORT_SYMBOL(tp_judge_ic_match_commandline);
int tp_util_get_vendor(struct hw_resource *hw_res,
		       struct panel_info *panel_data)
{
	char *vendor;
	int prj_id = 0;
	prj_id = get_project();
	g_hw_res = hw_res;

	panel_data->test_limit_name = kzalloc(MAX_LIMIT_DATA_LENGTH, GFP_KERNEL);

	if (panel_data->test_limit_name == NULL) {
		pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
	}

	panel_data->extra = kzalloc(MAX_LIMIT_DATA_LENGTH, GFP_KERNEL);

	if (panel_data->extra == NULL) {
		pr_err("[TP]panel_data.extra kzalloc error\n");
	}

	if (is_tp_type_got_in_match) {
		panel_data->tp_type = g_tp_dev_vendor;
	}
	if (panel_data->tp_type == TP_UNKNOWN) {
		pr_err("[TP]%s type is unknown\n", __func__);
		return 0;
	}
	vendor = GET_TP_DEV_NAME(panel_data->tp_type);

	if (prj_id == 19165 || prj_id == 19166) {
		memcpy(panel_data->manufacture_info.version, "0xBD3100000", 11);
	}

	if (prj_id == 20171 || prj_id == 20172 || prj_id == 20353) {
		memcpy(panel_data->manufacture_info.version, "0xaa0100000", 11);
	}

	if (prj_id == 20817 || prj_id == 20827 || prj_id == 20831 || prj_id == 21881 || prj_id == 21882) {
		memcpy(panel_data->manufacture_info.version, "0xaa4110000", 11);
	}

	if (prj_id == 20181 || prj_id == 20355) {
		memcpy(panel_data->manufacture_info.version, "0xbd3650000", 11);
	}

	if (prj_id == 0x212A1) {
		memcpy(panel_data->manufacture_info.version, "0xAA3080000", 11);
	}

	if (prj_id == 0x2169E || prj_id == 0x2169F || prj_id == 21711 || prj_id == 21712
		|| prj_id == 0x2162D || prj_id == 0x2162E || prj_id == 0x216C9 || prj_id == 0x216CA) {
        memcpy(panel_data->manufacture_info.version, "0xft3518s00", 11);
	}

	if (prj_id == 21061) {
		memcpy(panel_data->manufacture_info.version, "0xaa2420000", 11);
	}
	if (prj_id == 21081 || prj_id == 21127 || prj_id == 21305) {
		memcpy(panel_data->manufacture_info.version, "0xAA2660000", 11);
	}

	if (prj_id == 20075 || prj_id == 20076) {
	    memcpy(panel_data->manufacture_info.version, "0xRA5230000", 11);
	}

	if (prj_id == 21015 || prj_id == 21217) {
		memcpy(panel_data->manufacture_info.version, "0xaa2160000", 11);
		if (strstr(saved_command_line, "boe_nt37701_2ftp_1080p_dsi_cmd")) {
			pr_err("BOE panel is double layer");
			vendor = "BOE_TWO";
		}
	}
	if (20730 == prj_id || 20731 == prj_id || 20732 == prj_id) {
		if (focal_ft3518 == tp_used_index) {
		memcpy(panel_data->manufacture_info.version, "focalt_", 7);
		}
		if (goodix_gt9886 == tp_used_index) {
		memcpy(panel_data->manufacture_info.version, "goodix_", 7);
		}
	}
	if (strstr(saved_command_line, "20171_tianma_nt37701") || strstr(saved_command_line, "oplus21015_tianma")) {
		hw_res->TX_NUM = 18;
		hw_res->RX_NUM = 40;
		vendor = "TIANMA";
		pr_info("[TP]hw_res->TX_NUM:%d,hw_res->RX_NUM:%d\n",
		    hw_res->TX_NUM,
		    hw_res->RX_NUM);
	}
	if (prj_id == 20630 || prj_id == 20631 || prj_id == 20632 || prj_id == 20633 || prj_id == 20634 || prj_id == 20635 || prj_id == 0x206B4 ||
prj_id == 20637 || prj_id == 20638 || prj_id == 20639 || prj_id == 0x206B7) {
			panel_data->tp_type = TP_SAMSUNG;
			if(tp_type){
					memcpy(panel_data->manufacture_info.version, "goodix_", 7);
			}else{
					memcpy(panel_data->manufacture_info.version, "focaltech_", 10);
			}
	}
	if (prj_id == 20131 || prj_id == 20133 || prj_id == 20255 || prj_id == 20257
		|| prj_id == 20644 || prj_id == 20645 || prj_id == 20649 || prj_id == 0x2064A || prj_id == 0x2068D) {
		memcpy(panel_data->manufacture_info.version, "0xbd3520000", 11);
	}
	if (prj_id == 20615 || prj_id == 20662 || prj_id == 20619 || prj_id == 21609|| prj_id == 21651) {
		memcpy(panel_data->manufacture_info.version, "focalt_", 7);
	}
	if(strstr(saved_command_line, "oplus20131_tianma_nt37701_1080p_dsi_cmd")) {
		hw_res->TX_NUM = 18;
		hw_res->RX_NUM = 40;
		vendor = "TIANMA";
	} else if (strstr(saved_command_line, "oplus20131_tianma_nt37701_32_1080p_dsi_cmd")) {
	    vendor = "TIANMA";
	} else if (strstr(saved_command_line, "oplus20131_boe_nt37800_1080p_dsi_cmd")) {
	    vendor = "BOE";
	} else if (strstr(saved_command_line, "oplus20131_samsung_ana6705_1080p_dsi_cmd")) {
		vendor = "SAMSUNG";
	}

	if(strstr(saved_command_line, "oplus20645_tianma_nt37701_1080p_dsi_cmd")) {
		hw_res->TX_NUM = 18;
		hw_res->RX_NUM = 40;
		vendor = "TIANMA";
	} else if (strstr(saved_command_line, "oplus20645_tianma_nt37701_32_1080p_dsi_cmd")) {
	    vendor = "TIANMA";
	} else if (strstr(saved_command_line, "oplus20645_boe_nt37800_1080p_dsi_cmd")) {
	    vendor = "BOE";
	} else if (strstr(saved_command_line, "oplus20645_samsung_ana6705_1080p_dsi_cmd")) {
		vendor = "SAMSUNG";
	}
	if (prj_id == 20015 || prj_id == 20016 || prj_id == 20651 || prj_id == 20652 || prj_id == 20653 \
|| prj_id == 20654 || prj_id == 20108 || prj_id == 20109 || prj_id == 20307) {
		pr_info("[TP] tp_util_get_vendor 20015 rename vendor\n");
		if (strstr(saved_command_line, "nt36672c")) {
			vendor = "JDI";
		}
		if (strstr(saved_command_line, "oplus20625_nt35625b_boe_hlt_b3_vdo_lcm_drv")) {
			vendor = "INX";
		}
		if (strstr(saved_command_line, "oplus20015_ili9882n_truly_hd_vdo_lcm_drv")) {
			vendor = "CDOT";
		}
		if (strstr(saved_command_line, "oplus20015_ili9882n_boe_hd_vdo_lcm_drv")) {
			vendor = "HLT";
		}
		if (strstr(saved_command_line, "oplus20015_ili9882n_innolux_hd_vdo_lcm_drv")) {
			vendor = "INX";
		}
		if (strstr(saved_command_line, "oplus20015_hx83102_truly_vdo_hd_dsi_lcm_drv")) {
			vendor = "TRULY";
		}
		pr_info("[TP] we config this for FW name vendor:%s\n", vendor);
	}

	strcpy(panel_data->manufacture_info.manufacture, vendor);
	pr_err("[TP]tp_util_get_vendor line =%d\n", __LINE__);
	pr_err("[TP] enter case %d\n", prj_id);
	switch (prj_id) {
	case 20615:
	case 20662:
	case 20619:
	case 21609:
	case 21651:
		pr_info("[TP] enter case 20615/20662/20619/21609/21651\n");
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				"tp/20615/FW_%s_%s.img",
				"FT3518", vendor);
		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
				"tp/20615/LIMIT_%s_%s.img",
				"FT3518", vendor);
		}
/*
		if (panel_data->extra) {
			snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
				"tp/20615/BOOT_FW_%s_%s.ihex",
				"FT3518", vendor);
		}
*/
		panel_data->manufacture_info.fw_path = panel_data->fw_name;
		break;
	case 20761:
	case 20762:
	case 20764:
	case 20767:
	case 20766:
		if ((tp_used_index == ili9882n_truly) && (g_tp_dev_vendor == TP_TRULY)) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				 "tp/20761/FW_%s_%s.bin",
				 "NF_ILI9882N", vendor);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					 "tp/20761/LIMIT_%s_%s.ini",
					 "NF_ILI9882N", vendor);
			}

			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			pr_info("[TP]: firmware_headfile = FW_20762_ILI9882N_TRULY\n");
			memcpy(panel_data->manufacture_info.version, "XL_NSZ_9882_", 12);
			panel_data->firmware_headfile.firmware_data = FW_20762_ILI9882H_TRULY;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_20762_ILI9882H_TRULY);
		}
		if ((tp_used_index == ili7807s_hlt) && (g_tp_dev_vendor == TP_HLT)) {
			vendor = "HLT";
			strcpy(panel_data->manufacture_info.manufacture, vendor);
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				 "tp/20761/FW_%s_%s.bin",
				 "NF_ILI7807S", vendor);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					 "tp/20761/LIMIT_%s_%s.ini",
					 "NF_ILI7807S", vendor);
			}

			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			pr_info("[TP]: firmware_headfile = FW_20761_ILI7807S_HLT\n");
			memcpy(panel_data->manufacture_info.version, "HLT_B6_7807_", 12);
			panel_data->firmware_headfile.firmware_data = FW_20761_ILI7807S_HLT;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_20761_ILI7807S_HLT);
		}
		break;
	case 0x2167A:
	case 0x2167B:
	case 0x2167C:
	case 0x2167D:
	case 0x216AF:
	case 0x216B0:
	case 0x216B1:
		if ((tp_used_index == ili9882n_truly) && (g_tp_dev_vendor == TP_TRULY)) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				 "tp/2167A/FW_%s_%s.bin",
				 "NF_ILI9882N", vendor);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					 "tp/2167A/LIMIT_%s_%s.ini",
					 "NF_ILI9882N", vendor);
			}

			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			pr_info("[TP]: firmware_headfile = FW_20762_ILI9882N_TRULY\n");
			memcpy(panel_data->manufacture_info.version, "XL_NSZ_9882_", 12);
			panel_data->firmware_headfile.firmware_data = FW_20762_ILI9882H_TRULY;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_20762_ILI9882H_TRULY);
		}
		if ((tp_used_index == ili7807s_hlt) && (g_tp_dev_vendor == TP_HLT)) {
			vendor = "HLT";
			strcpy(panel_data->manufacture_info.manufacture, vendor);
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				 "tp/2167A/FW_%s_%s.bin",
				 "NF_ILI7807S", vendor);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					 "tp/2167A/LIMIT_%s_%s.ini",
					 "NF_ILI7807S", vendor);
			}

			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			pr_info("[TP]: firmware_headfile = FW_20761_ILI7807S_HLT\n");
			memcpy(panel_data->manufacture_info.version, "HLT_B6_7807_", 12);
			panel_data->firmware_headfile.firmware_data = FW_20761_ILI7807S_HLT;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_20761_ILI7807S_HLT);
		}
		if ((tp_used_index == nt36525b_hlt) && (g_tp_dev_vendor == TP_HLT)) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				"tp/2167A/FW_%s_%s.bin",
				"NF_NT36525B", "HLTB8");

			if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
				"tp/2167A/LIMIT_%s_%s.img",
				"NF_NT36525B", "HLTB8");
			}

			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			strcpy(panel_data->manufacture_info.manufacture, "HLT");
			memcpy(panel_data->manufacture_info.version, "HLT_B8_NT25_", 12);
			panel_data->firmware_headfile.firmware_data = FW_206AC_NF_NT36525B_HLT_B8;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_206AC_NF_NT36525B_HLT_B8);
		}
		break;
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

	case 20131:
	case 20133:
	case 20255:
	case 20257:
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
			 "tp/20131/FW_%s_%s.%s",
			  panel_data->chip_name, vendor, !strcmp(vendor,
					"SAMSUNG") ? "bin" : "img");

		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
				 "tp/20131/LIMIT_%s_%s.img",
				 panel_data->chip_name, vendor);
		}
		panel_data->manufacture_info.fw_path = panel_data->fw_name;
		break;
	case 20644:
	case 20645:
	case 20649:
	case 0x2064A:
	case 0x2068D:
	snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
			 "tp/20645/FW_%s_%s.%s",
			  panel_data->chip_name, vendor, !strcmp(vendor,
					"SAMSUNG") ? "bin" : "img");

		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
				 "tp/20645/LIMIT_%s_%s.img",
				 panel_data->chip_name, vendor);
		}
		panel_data->manufacture_info.fw_path = panel_data->fw_name;
		break;
	case 20181:
	case 20355:
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
			"tp/20181/FW_%s_%s.img",
			panel_data->chip_name, vendor);

		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
			"tp/20181/LIMIT_%s_%s.img",
			panel_data->chip_name, vendor);
		}
		panel_data->manufacture_info.fw_path = panel_data->fw_name;
		break;
	case 21081:
	case 21127:
	case 21305:
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
			"tp/%d/FW_%s_%s.img",
			g_tp_prj_id, panel_data->chip_name, vendor);

		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
			"tp/%d/LIMIT_%s_%s.img",
			g_tp_prj_id, panel_data->chip_name, vendor);
		}
		panel_data->manufacture_info.fw_path = panel_data->fw_name;
		break;
	case 0x212A1:
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
			"tp/%X/FW_%s_%s.img",
			g_tp_prj_id, panel_data->chip_name, vendor);

		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
			"tp/%X/LIMIT_%s_%s.img",
			g_tp_prj_id, panel_data->chip_name, vendor);
		}
		panel_data->manufacture_info.fw_path = panel_data->fw_name;
		break;
	case 0x2169E:
	case 0x2169F:
	case 21711:
	case 21712:
	case 0x2162D:
	case 0x2162E:
	case 0x216C9:
	case 0x216CA:
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
			"tp/2169E/FW_%s_%s.img",
			panel_data->chip_name, vendor);

		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
			"tp/2169E/LIMIT_%s_%s.img",
			panel_data->chip_name, vendor);
		}
		panel_data->manufacture_info.fw_path = panel_data->fw_name;
		break;
	case 20391:
	case 20392:
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				"tp/20391/FW_%s_%s.img",
				panel_data->chip_name, vendor);

		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
				"tp/20391/LIMIT_%s_%s.img",
				panel_data->chip_name, vendor);
		}
		panel_data->manufacture_info.fw_path = panel_data->fw_name;
		memcpy(panel_data->manufacture_info.version, "0xFA2720000", 11);
		break;

	case 20001:
	case 20002:
	case 20003:
	case 20200:
		pr_info("[TP] enter case OPLUS_20001\n");

		if (tp_used_index == nt36672c) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				 "tp/20001/FW_%s_%s.img",
				 "NF_NT36672C", vendor);
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					 "tp/20001/LIMIT_%s_%s.img",
					 "NF_NT36672C", vendor);
			}
			if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					 "tp/20001/BOOT_FW_%s_%s.ihex",
					 "NF_NT36672C", vendor);
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;

			if (tp_used_index == nt36672c) {
				pr_info("[TP]: firmware_headfile = FW_20001_NF_NT36672C_JDI_fae_jdi\n");
				memcpy(panel_data->manufacture_info.version, "0xFA219DN", 9);
				panel_data->firmware_headfile.firmware_data = FW_20001_NF_NT36672C_JDI;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_20001_NF_NT36672C_JDI);
			}
		}

		if (tp_used_index == himax_83112f) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				 "tp/20001/FW_%s_%s.img",
				 "NF_HX83112F", vendor);
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					 "tp/20001/LIMIT_%s_%s.img",
					 "NF_HX83112F", vendor);
			}
			if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					 "tp/20001/BOOT_FW_%s_%s.ihex",
					 "NF_HX83112F", vendor);
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			if (tp_used_index == himax_83112f) {
				pr_info("[TP]: firmware_headfile = FW_20001_NF_HX83112F_TIANMA\n");
				memcpy(panel_data->manufacture_info.version, "0xFA219TH", 9);
				panel_data->firmware_headfile.firmware_data = FW_20001_NF_HX83112F_TIANMA;
                panel_data->firmware_headfile.firmware_size = sizeof(FW_20001_NF_HX83112F_TIANMA);
			}
		}

		if (tp_used_index == ili7807s_tm) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				"tp/20001/FW_%s_%s.img",
				"NF_ILI7807S", vendor);
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					"tp/20001/LIMIT_%s_%s.img",
					"NF_ILI7807S", vendor);
			}
			if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					"tp/20001/BOOT_FW_%s_%s.ihex",
					"NF_ILI7807S", vendor);
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			if (tp_used_index == ili7807s_tm) {
				pr_info("[TP]: firmware_headfile = FW_20001_NF_ILI7807S_TIANMA\n");
				memcpy(panel_data->manufacture_info.version, "0xFA219TI", 9);
				panel_data->firmware_headfile.firmware_data = FW_20001_NF_ILI7807S_TIANMA;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_20001_NF_ILI7807S_TIANMA);
			}
		}

		break;

	case 20151:
	case 20301:
	case 20302:
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				"tp/20151/FW_%s_%s.img",
				"FT3518", "SAMSUNG");

		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
				"tp/20151/LIMIT_%s_%s.img",
				"FT3518", "SAMSUNG");
		}
		panel_data->manufacture_info.fw_path = panel_data->fw_name;
		memcpy(panel_data->manufacture_info.version, "0xFA2720000", 11);
		break;
	case 20015:
	case 20016:
	case 20651:
	case 20652:
	case 20653:
	case 20654:
	case 20108:
	case 20109:
	case 20307:
		pr_info("[TP] enter case 20015\n");
		if (tp_used_index == nt36525b_boe) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/20015/FW_%s_%s.bin",
					 "NF_NT36525B", vendor);
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/20015/LIMIT_%s_%s.img",
						 "NF_NT36525B", vendor);
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "0xAA006IN", 9);
			panel_data->firmware_headfile.firmware_data = FW_20015_NT36525B_INX;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_20015_NT36525B_INX);
		}
		if (tp_used_index == ili9882n_cdot) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/20015/FW_%s_%s.bin",
					 "NF_ILI9882N", vendor);
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/20015/LIMIT_%s_%s.ini",
						 "NF_ILI9882N", vendor);
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "ILI9882N_", 9);
			panel_data->firmware_headfile.firmware_data = FW_20015_ILI9882N_CDOT;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_20015_ILI9882N_CDOT);
		}
		if (tp_used_index == ili9882n_hlt) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/20015/FW_%s_%s.bin",
					 "NF_ILI9882N", vendor);
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/20015/LIMIT_%s_%s.ini",
						 "NF_ILI9882N", vendor);
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "0xAA006HI", 9);
			panel_data->firmware_headfile.firmware_data = FW_20015_ILI9882N_HLT;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_20015_ILI9882N_HLT);
		}
		if (tp_used_index == ili9882n_inx) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/20015/FW_%s_%s.bin",
					 "NF_ILI9882N", vendor);
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/20015/LIMIT_%s_%s.ini",
						 "NF_ILI9882N", vendor);
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "0xAA006II", 9);
			panel_data->firmware_headfile.firmware_data = FW_20015_ILI9882N_INX;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_20015_ILI9882N_INX);
		}
		if (tp_used_index == himax_83102d) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/20015/FW_%s_%s.bin",
					 "NF_HX83102D", vendor);
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/20015/LIMIT_%s_%s.ini",
						 "NF_HX83102D", vendor);
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "0xAA006TH000", 12);
			panel_data->firmware_headfile.firmware_data = FW_20015_NF_HX83102D_TRULY;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_20015_NF_HX83102D_TRULY);
		}
		break;
	case 20630:
	case 20631:
	case 20632:
	case 20633:
	case 20634:
	case 20635:
	case 0x206B4:
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
                 "tp/20630/FW_%s_%s.img",
                 panel_data->chip_name, vendor);

        if (panel_data->test_limit_name) {
            snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
                     "tp/20630/LIMIT_%s_%s.img",
                     panel_data->chip_name, vendor);
        }

        if (panel_data->extra) {
            snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
                     "tp/20630/BOOT_FW_%s_%s.ihex",
                     panel_data->chip_name, vendor);
        }
        panel_data->manufacture_info.fw_path = panel_data->fw_name;
        break;
	case 20637:
	case 20638:
	case 20639:
	case 0x206B7:
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				"tp/20637/FW_%s_%s.img",
				panel_data->chip_name, vendor);

		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					"tp/20637/LIMIT_%s_%s.img",
					panel_data->chip_name, vendor);
		}

		if (panel_data->extra) {
			snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					"tp/20637/BOOT_FW_%s_%s.ihex",
					panel_data->chip_name, vendor);
		}
		panel_data->manufacture_info.fw_path = panel_data->fw_name;
		break;
	case 20601:
	case 20660:
	case 20602:
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
			 "tp/20601/FW_%s_%s.img",
			  panel_data->chip_name, vendor);
		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
				 "tp/20601/LIMIT_%s_%s.img",
				 panel_data->chip_name, vendor);
		}
		panel_data->manufacture_info.fw_path = panel_data->fw_name;
		break;
		case 20613:
	case 20611:
	case 20610:
	case 20680:
	case 20686:
		pr_info("[TP] enter case OPLUS_20611\n");

		if (tp_used_index == nt36672c) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				 "tp/20611/FW_%s_%s.img",
				 "NF_NT36672C", "JDI");

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					 "tp/20611/LIMIT_%s_%s.img",
					 "NF_NT36672C", "JDI");
				}

			if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					 "tp/20611/BOOT_FW_%s_%s.ihex",
					 "NF_NT36672C", "JDI");
			}

			panel_data->manufacture_info.fw_path = panel_data->fw_name;

			if ((tp_used_index == nt36672c) && (g_tp_dev_vendor == TP_JDI)) {
				pr_info("[TP]: firmware_headfile = FW_20611_NF_NT36672C_JDI\n");
				memcpy(panel_data->manufacture_info.version, "0xFA356DN", 9);
				panel_data->firmware_headfile.firmware_data = FW_20611_NF_NT36672C_JDI;
                		panel_data->firmware_headfile.firmware_size = sizeof(FW_20611_NF_NT36672C_JDI);
			}
		}
		if (tp_used_index == himax_83112f) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
                     		"tp/20611/FW_%s_%s.img",
                     		"NF_HX83112F", vendor);

            		if (panel_data->test_limit_name) {
                	snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
                         		"tp/20611/LIMIT_%s_%s.img",
                         		"NF_HX83112F", vendor);
            		}

            		if (panel_data->extra) {
                	snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
                         	"tp/20611/BOOT_FW_%s_%s.ihex",
                         	"NF_HX83112F", vendor);
            		}
            	panel_data->manufacture_info.fw_path = panel_data->fw_name;
            		if (tp_used_index == himax_83112f) {
                	pr_info("[TP]: firmware_headfile = FW_20001_NF_HX83112F_TIANMA\n");
                	memcpy(panel_data->manufacture_info.version, "0xFA219TH", 9);
                	panel_data->firmware_headfile.firmware_data = FW_20001_NF_HX83112F_TIANMA;
                	panel_data->firmware_headfile.firmware_size = sizeof(FW_20001_NF_HX83112F_TIANMA);
            		}
        	}
			break;

	case 20095:
		pr_info("[TP] enter case 20095\n");

		if (tp_used_index == nt36672c) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				 "tp/20095/FW_%s_%s.img",
				 "NF_NT36672C", "DSJM");

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					 "tp/20095/LIMIT_%s_%s.img",
					 "NF_NT36672C", "DSJM");
			}

			if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					 "tp/20095/BOOT_FW_%s_%s.ihex",
					 "NF_NT36672C", "DSJM");
			}

			panel_data->manufacture_info.fw_path = panel_data->fw_name;

			if ((tp_used_index == nt36672c) && (g_tp_dev_vendor == TP_DSJM)) {
				pr_info("[TP]: firmware_headfile = FW_20611_NF_NT36672C_DSJM\n");
				memcpy(panel_data->manufacture_info.version, "0xRA356DN", 9);
				panel_data->firmware_headfile.firmware_data = FW_20611_NF_NT36672C_DSJM;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_20611_NF_NT36672C_DSJM);
			}
		}
		if (g_tp_dev_vendor == TP_TIANMA) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				"tp/20095/FW_%s_%s.img",
				"NF_7807S", "TIANMA");

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					"tp/20095/LIMIT_%s_%s.img",
					"NF_7807S", "TIANMA");
			}

			if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					"tp/20095/BOOT_FW_%s_%s.ihex",
					"NF_7807S", "TIANMA");
			}

			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			pr_info("[TP]: firmware_headfile = FW_20095_NF_7807S_TIANMA\n");
			memcpy(panel_data->manufacture_info.version, "0xRA356TI", 9);
			panel_data->firmware_headfile.firmware_data = FW_20095_NF_7807S_TIANMA;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_20095_NF_7807S_TIANMA);
		}
		break;

	case 19131:
	case 19132:
	case 19420:
		pr_info("[TP] enter case 19131\n");

		if (tp_used_index == nt36672c) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				 "tp/19131/FW_%s_%s.img",
				 "NF_NT36672C", vendor);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					 "tp/19131/LIMIT_%s_%s.img",
					 "NF_NT36672C", vendor);
			}

			if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					 "tp/19131/BOOT_FW_%s_%s.ihex",
					 "NF_NT36672C", vendor);
			}

			panel_data->manufacture_info.fw_path = panel_data->fw_name;

			if ((tp_used_index == nt36672c) && (g_tp_dev_vendor == TP_JDI)) {
				pr_info("[TP]: firmware_headfile = FW_19131_NF_NT36672C_JDI_fae_jdi\n");
				memcpy(panel_data->manufacture_info.version, "0xDD300JN200", 12);
				/*panel_data->firmware_headfile.firmware_data = FW_19131_NF_NT36672C_JDI;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_19131_NF_NT36672C_JDI);*/
				panel_data->firmware_headfile.firmware_data = FW_19131_NF_NT36672C_JDI_fae_jdi;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_19131_NF_NT36672C_JDI_fae_jdi);
			}

			if ((tp_used_index == nt36672c) && (g_tp_dev_vendor == TP_TIANMA)) {
				pr_info("[TP]: firmware_headfile = FW_19131_NF_NT36672C_TIANMA_fae_tianma\n");
				memcpy(panel_data->manufacture_info.version, "0xDD300TN000", 12);
				/*panel_data->firmware_headfile.firmware_data = FW_19131_NF_NT36672C_TIANMA_oplus;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_19131_NF_NT36672C_TIANMA_oplus);*/
				panel_data->firmware_headfile.firmware_data = FW_19131_NF_NT36672C_TIANMA_fae_tianma;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_19131_NF_NT36672C_TIANMA_fae_tianma);
			}
		}

		if (tp_used_index == himax_83112f) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				 "tp/19131/FW_%s_%s.img",
				 "NF_HX83112F", vendor);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					 "tp/19131/LIMIT_%s_%s.img",
					 "NF_HX83112F", vendor);
			}

			if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					 "tp/19131/BOOT_FW_%s_%s.ihex",
					 "NF_HX83112F", vendor);
			}

			panel_data->manufacture_info.fw_path = panel_data->fw_name;

			if (tp_used_index == himax_83112f) {
				pr_info("[TP]: firmware_headfile = FW_19131_NF_HX83112F_JDI\n");
				memcpy(panel_data->manufacture_info.version, "0xDD300JH000", 12);
				panel_data->firmware_headfile.firmware_data = FW_19131_NF_HX83112F_JDI;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_19131_NF_HX83112F_JDI);
			}
		}
		break;

	case 21101:
	case 21102:
	case 21235:
	case 21236:
	case 21831:
		if (strstr(saved_command_line, "oplus21101_ili9883a_boe_hd")) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					"tp/21101/FW_%s_%s.img",
					"NF_9883A", "BOE");

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						"tp/21101/LIMIT_%s_%s.img",
						"NF_9883A", "BOE");
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "0xAA270BI01", 11);
			panel_data->firmware_headfile.firmware_data = FW_21101_NF_7807S_BOE;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_21101_NF_7807S_BOE);
		}

		if (strstr(saved_command_line, "oplus21101_ili9883a_boe_bi_hd")) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					"tp/21101/FW_%s_%s_%s.img",
					"NF_9883A", "BOE", "THIN");

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						"tp/21101/LIMIT_%s_%s.img",
						"NF_9883A", "BOE");
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "0xAA270bI01", 11);
			panel_data->firmware_headfile.firmware_data = FW_21101_NF_7807S_BOE_THIN;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_21101_NF_7807S_BOE_THIN);
		}

		if (strstr(saved_command_line, "oplus21101_td4160")) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					"tp/21101/FW_%s_%s.img",
					"TD4160", "INX");

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						"tp/21101/LIMIT_%s_%s.img",
						"TD4160", "INX");
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "0xAA270TI01", 11);
			panel_data->firmware_headfile.firmware_data = FW_21101_TD4160_INX;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_21101_TD4160_INX);
		}
		break;

	case 21639:
	case 0x2163B:
	case 0x2163C:
	case 0x2163D:
	case 0x216CD:
	case 0x216CE:
		if (strstr(saved_command_line, "oplus2163b_ili9883a")) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					"tp/2163B/FW_%s_%s.img",
					"NF_9883A", "BOE");

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						"tp/2163B/LIMIT_%s_%s.img",
						"NF_9883A", "BOE");
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "0xAA270BI01", 11);
			panel_data->firmware_headfile.firmware_data = FW_2163B_NF_7807S_BOE;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_2163B_NF_7807S_BOE);
		}

		if (strstr(saved_command_line, "oplus2163b_td4160_inx_hd_vdo_lcm_drv")) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					"tp/2163B/FW_%s_%s.img",
					"TD4160", "INX");

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						"tp/2163B/LIMIT_%s_%s.img",
						"TD4160", "INX");
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "0xAA270TI01", 11);
			panel_data->firmware_headfile.firmware_data = FW_2163B_TD4160_INX;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_2163B_TD4160_INX);
		}

		if (strstr(saved_command_line, "oplus21639_nt36672c_tm_cphy_dsi_vdo_lcm_drv")) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					"tp/2163B/FW_%s_%s.bin",
					"NF_NT36672C", "TIANMA");

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						"tp/2163B/LIMIT_%s_%s.img",
						"NF_NT36672C", "TIANMA");
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "0xAB417TMNT01", 13);
			panel_data->firmware_headfile.firmware_data = FW_21639_NF_NT36672C_TIANMA;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_21639_NF_NT36672C_TIANMA);
		}

		if (strstr(saved_command_line, "oplus2163b_td4160_truly_hd_vdo_lcm_drv")) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					"tp/2163B/FW_%s_%s.img",
					"TD4160", "TRULY");

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						"tp/2163B/LIMIT_%s_%s.img",
						"TD4160", "TRULY");
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "0xAB417_TR_TD_01", 16);
			panel_data->firmware_headfile.firmware_data = FW_2163B_TD4160_TRULY;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_2163B_TD4160_TRULY);
		}
		break;

	case 21037:
		pr_info("[TP] enter case thor-a 21037\n");
		if (tp_used_index == nt36525b_inx) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/21037/FW_%s_%s.bin",
					 "NF_NT36525B", "INX");
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/21037/LIMIT_%s_%s.img",
						 "NF_NT36525B", "INX");
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "0xAA278IN", 9);
			panel_data->firmware_headfile.firmware_data = FW_21037_NT36525B_INX;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_21037_NT36525B_INX);
		}
		if (tp_used_index == hx83102d_txd) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/21037/FW_%s_%s.bin",
					 "NF_HX83102D", "TXD");
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/21037/LIMIT_%s_%s.img",
						 "NF_HX83102D", "TXD");
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "0xAA278TH000", 12);
			panel_data->firmware_headfile.firmware_data = FW_21037_HX83102D_TXD;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_21037_HX83102D_TXD);
		}
		if (tp_used_index == ili9882n_inx) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/21037/FW_%s_%s.img",
					 "NF_ILI9882N", "INX");
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/21037/LIMIT_%s_%s.img",
						 "NF_ILI9882N", "INX");
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "AA278II", 7);
			panel_data->firmware_headfile.firmware_data = FW_21037_ILI9882N_INX;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_21037_ILI9882N_INX);
		}
		if (tp_used_index == ft8006s_truly) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/21037/FW_%s_%s.img",
					 "NF_FT8006S", "TRULY");
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/21037/LIMIT_%s_%s.img",
						 "NF_FT8006S", "TRULY");
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "AA278TF", 7);
			panel_data->firmware_headfile.firmware_data = FW_21037_FT8006S_TRULY;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_21037_FT8006S_TRULY);
		}
		break;
	case 0x210A0:
		pr_info("[TP] enter case 210A0\n");
		if (tp_used_index == ili7807s_tianma) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					"tp/210A0/FW_%s_%s.img",
					"NF_ILI7807S", "TIANMA");
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						"tp/210A0/LIMIT_%s_%s.img",
						"NF_ILI7807S", "TIANMA");
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "AA231TI", 7);
			panel_data->firmware_headfile.firmware_data = FW_21041_ILI7807S_TIANMA;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_21041_ILI7807S_TIANMA);
		}
		if (tp_used_index == ili7807s_jdi) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					"tp/21041/FW_%s_%s.img",
					"NF_ILI7807S", "JDI");
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						"tp/21041/LIMIT_%s_%s.img",
						"NF_ILI7807S", "JDI");
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "AA231DI", 14);
			panel_data->firmware_headfile.firmware_data = FW_21041_ILI7807S_JDI;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_21041_ILI7807S_JDI);
		}
		break;
	case 21041:
	case 21042:
	case 21241:
	case 21242:
		pr_info("[TP] enter case odin-a 21041\n");
		if (tp_used_index == ili7807s_tianma) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/21041/FW_%s_%s.img",
					 "NF_ILI7807S", "TIANMA");
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/21041/LIMIT_%s_%s.img",
						 "NF_ILI7807S", "TIANMA");
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "AA231TI", 7);
			panel_data->firmware_headfile.firmware_data = FW_21041_ILI7807S_TIANMA;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_21041_ILI7807S_TIANMA);
		}
		if (tp_used_index == ili7807s_jdi) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/21041/FW_%s_%s.img",
					 "NF_ILI7807S", "JDI");
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/21041/LIMIT_%s_%s.img",
						 "NF_ILI7807S", "JDI");
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "AA231DI", 14);
			panel_data->firmware_headfile.firmware_data = FW_21041_ILI7807S_JDI;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_21041_ILI7807S_JDI);
		}
		break;
	case 20608:
	case 20609:
	case 0x2060A:
	case 0x2060B:
	case 0x206F0:
	case 0x206FF:
	case 20796:
	case 20795:
	case 0x2070B:
	case 0x2070C:
	case 0x2070E:
		pr_info("[TP] enter case 20609\n");
		if (tp_used_index == nt36672c) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/20609/FW_%s_%s.bin",
					 "NF_NT36672C", vendor);
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/20609/LIMIT_%s_%s.img",
						 "NF_NT36672C", vendor);
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "RA627_TM_NT_", 12);
			panel_data->firmware_headfile.firmware_data = FW_20609_NT36672C_TIANMA;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_20609_NT36672C_TIANMA);
		}
		if (tp_used_index == ili9882n_inx) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/20609/FW_%s_%s.bin",
					 "NF_ILI7807S", vendor);
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/20609/LIMIT_%s_%s.img",
						 "NF_ILI7807S", vendor);
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "RA627_TM_ILI_", 13);
			panel_data->firmware_headfile.firmware_data = FW_20609_ILI7807S_TIANMA;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_20609_ILI7807S_TIANMA);
		}
		break;
	case 21625:
	case 0x216A0:
	case 0x216A1:
	case 0x216B5:
	case 0x216B6:
		pr_info("[TP] enter case 21625\n");
		if (tp_used_index == nt36672c) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/21625/FW_%s_%s.bin",
					 "NF_NT36672C", vendor);
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/21625/LIMIT_%s_%s.img",
						 "NF_NT36672C", vendor);
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "AB151_TM_NT_", 12);
			panel_data->firmware_headfile.firmware_data = FW_20609_NT36672C_TIANMA;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_20609_NT36672C_TIANMA);
		}
		if (tp_used_index == ili9882n_inx) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/21625/FW_%s_%s.bin",
					 "NF_ILI7807S", vendor);
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/21625/LIMIT_%s_%s.img",
						 "NF_ILI7807S", vendor);
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "AB151_TM_ILI_", 13);
			panel_data->firmware_headfile.firmware_data = FW_20609_ILI7807S_TIANMA;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_20609_ILI7807S_TIANMA);
		}
		break;
	case 20730:
	case 20731:
	case 20732:

                pr_info("%s forward for 20730\n", __func__);
		if (tp_used_index == focal_ft3518) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				 "tp/20730/FW_%s_%s.img",
				 "FT3518", vendor);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					 "tp/20730/LIMIT_%s_%s.img",
					 "FT3518", vendor);
			}

			if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					 "tp/20730/BOOT_FW_%s_%s.ihex",
					 "FT3518", vendor);
			}
		}

		if (tp_used_index == goodix_gt9886) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				 "tp/20730/FW_%s_%s.img",
				"GT9886", vendor);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					 "tp/20730/LIMIT_%s_%s.img",
					 "GT9886", vendor);
			}

			if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					 "tp/20730/BOOT_FW_%s_%s.ihex",
					 "GT9886", vendor);
			}
                }
		panel_data->manufacture_info.fw_path = panel_data->fw_name;
		break;
	case 21684:
	case 21685:
	case 21686:
	case 21687:
	case 21690:
	case 21691:
	case 21692:
		pr_info("[TP] enter case 21684\n");
		if (tp_used_index == ili7807s_boe) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/21684/FW_%s_%s.bin",
					 "NF_ILI7807S", vendor);
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/21684/LIMIT_%s_%s.img",
						 "NF_ILI7807S", vendor);
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "AB089_BOE_ILI_", 14);
			panel_data->firmware_headfile.firmware_data = FW_21684_ILI7807S_BOE;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_21684_ILI7807S_BOE);
		}
		if (tp_used_index == ili7807s_hlt) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/21684/FW_%s_%s.bin",
					 "NF_ILI7807S", vendor);
			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/21684/LIMIT_%s_%s.img",
						 "NF_ILI7807S", vendor);
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			memcpy(panel_data->manufacture_info.version, "AB089_HLT_ILI_", 14);
			panel_data->firmware_headfile.firmware_data = FW_21684_ILI7807S_BOE;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_21684_ILI7807S_HLT);
		}
		break;
/*bringup add for sala touchscreen.*/
	case 20682:
		pr_info("[TP] enter case 20682\n");
		if (tp_used_index == nt36672c) {
			if (g_tp_dev_vendor == TP_TIANMA) {
				pr_info("[TP] 20682 tp vendor tianma\n");
				snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					"tp/20682/FW_%s_%s.bin",
					"NT36672C_NF", vendor);
				if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					"tp/20682/LIMIT_%s_%s.img",
					"NT36672C_NF", vendor);
				}
				if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					"tp/20682/BOOT_FW_%s_%s.ihex",
					"NT36672C_NF", vendor);
				}
				panel_data->manufacture_info.fw_path = panel_data->fw_name;
				memcpy(panel_data->manufacture_info.version, "NT72C_TM_", 9);
				panel_data->firmware_headfile.firmware_data = FW_20682_NT36672C_TIANMA;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_20682_NT36672C_TIANMA);
			} else if (g_tp_dev_vendor == TP_JDI) {
				pr_info("[TP] 20682 tp vendor jdi\n");
				snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					"tp/20682/FW_%s_%s.bin",
					"NT36672C_NF", vendor);

				if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					"tp/20682/LIMIT_%s_%s.img",
					"NT36672C_NF", vendor);
				}

				if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					"tp/20682/BOOT_FW_%s_%s.ihex",
					"NT36672C_NF", vendor);
				}
				panel_data->manufacture_info.fw_path = panel_data->fw_name;
				memcpy(panel_data->manufacture_info.version, "NT72C_JDI", 9);
				panel_data->firmware_headfile.firmware_data = FW_20682_NT36672C_JDI;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_20682_NT36672C_JDI);
			} else if (g_tp_dev_vendor == TP_BOE) {
				pr_info("[TP] 20682 tp vendor boe\n");
				snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					"tp/20682/FW_%s_%s.bin",
					"NT36672C_NF", vendor);

				if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					"tp/20682/LIMIT_%s_%s.img",
					"NT36672C_NF", vendor);
				}

				if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					"tp/20682/BOOT_FW_%s_%s.ihex",
					"NT36672C_NF", vendor);
				}
				panel_data->manufacture_info.fw_path = panel_data->fw_name;
				memcpy(panel_data->manufacture_info.version, "NT72C_BOE", 9);
				panel_data->firmware_headfile.firmware_data = FW_20682_NT36672C_BOE;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_20682_NT36672C_BOE);
			} else {
				pr_info("[TP] 20682 tp ic not found\n");
			}
		}
		break;

	/* bringup add for sala touchscreen.*/
	case 20041:
	case 20042:
	case 20043:
		pr_info("[TP] enter case 20041\n");
		if (tp_used_index == nt36672c) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				"tp/20041/FW_%s_%s.img",
				"NF_NT36672C", vendor);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
				"tp/20041/LIMIT_%s_%s.img",
				"NF_NT36672C", vendor);
			}
			if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
				"tp/20041/BOOT_FW_%s_%s.ihex",
				"NF_NT36672C", vendor);
			}

			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			if (tp_used_index == nt36672c) {
				pr_info("[TP]: firmware_headfile = FW_20001_NF_NT36672C_JDI\n");
				memcpy(panel_data->manufacture_info.version, "0xFA278DN", 9);
				panel_data->firmware_headfile.firmware_data = FW_20041_NF_NT36672C_JDI;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_20041_NF_NT36672C_JDI);
			}
		}

		if (tp_used_index == nt36672c_boe) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				"tp/20041/FW_%s_%s.img",
				"NF_NT36672C", vendor);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					"tp/20041/LIMIT_%s_%s.img",
			 		"NF_NT36672C", vendor);
			}
			if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					"tp/20041/BOOT_FW_%s_%s.ihex",
					"NF_NT36672C", vendor);
			}
			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			if (tp_used_index == nt36672c_boe) {
				pr_info("[TP]: firmware_headfile = FW_20001_NF_NT36672C_BOE\n");
				memcpy(panel_data->manufacture_info.version, "0xFA278BN", 9);
				panel_data->firmware_headfile.firmware_data = FW_20041_NF_NT36672C_BOE;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_20041_NF_NT36672C_BOE);
			}

			hw_res->TX_NUM = 18;
			hw_res->RX_NUM = 36;
			pr_info("[TP] BOE panel TX_NUM = %d, RX_NUM = %d \n", hw_res->TX_NUM, hw_res->RX_NUM);
		}

		if (tp_used_index == himax_83112f) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				"tp/20041/FW_%s_%s.img",
				"NF_HX83112F", vendor);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
				"tp/20041/LIMIT_%s_%s.img",
				"NF_HX83112F", vendor);
			}

			if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
				"tp/20041/BOOT_FW_%s_%s.ihex",
				"NF_HX83112F", vendor);
			}

			panel_data->manufacture_info.fw_path = panel_data->fw_name;

			if (tp_used_index == himax_83112f) {
				pr_info("[TP]: firmware_headfile = FW_20041_NF_HX83112F_TIANMA\n");
				memcpy(panel_data->manufacture_info.version, "0xFA278TH", 9);
				panel_data->firmware_headfile.firmware_data = FW_20041_NF_HX83112F_TIANMA;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_20041_NF_HX83112F_TIANMA);
			}
		}

		if (tp_used_index == ili7807s_tm) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
			"tp/20041/FW_%s_%s.img",
			"NF_ILI7807S", vendor);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					"tp/20041/LIMIT_%s_%s.img",
					"NF_ILI7807S", vendor);
			}

			if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					"tp/20041/BOOT_FW_%s_%s.ihex",
					"NF_ILI7807S", vendor);
			}

			panel_data->manufacture_info.fw_path = panel_data->fw_name;

			if (tp_used_index == ili7807s_tm) {
				pr_info("[TP]: firmware_headfile = FW_20041_NF_ILI7807S_TIANMA\n");
				memcpy(panel_data->manufacture_info.version, "0xFA278TI", 9);
				panel_data->firmware_headfile.firmware_data = FW_20041_NF_ILI7807S_TIANMA;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_20041_NF_ILI7807S_TIANMA);
			}
		}
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
EXPORT_SYMBOL(tp_util_get_vendor);
/**
 * Description:
 * pulldown spi7 cs to avoid current leakage
 * because of current sourcing from cs (pullup state) flowing into display module
 **/
void switch_spi7cs_state(bool normal)
{
    if(normal){
        if( !IS_ERR_OR_NULL(g_hw_res->pin_set_high) ) {
            pr_info("%s: going to set spi7 cs to spi mode .\n", __func__);
            pinctrl_select_state(g_hw_res->pinctrl, g_hw_res->pin_set_high);
        }else{
            pr_info("%s: cannot to set spi7 cs to spi mode .\n", __func__);
        }
    } else {
        if( !IS_ERR_OR_NULL(g_hw_res->pin_set_low) ) {
            pr_info("%s: going to set spi7 cs to pulldown .\n", __func__);
            pinctrl_select_state(g_hw_res->pinctrl, g_hw_res->pin_set_low);
        }else{
            pr_info("%s: cannot to set spi7 cs to pulldown .\n", __func__);
        }
    }
}
EXPORT_SYMBOL(switch_spi7cs_state);
