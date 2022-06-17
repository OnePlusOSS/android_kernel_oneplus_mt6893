/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include "mtk_charger_intf.h"

int mtk_hvdcp_v20_reset_ta_vchr(struct charger_manager *pinfo)
{
    int ret = 0;
    charger_dev_reset_hvdcp_ta_ex(pinfo->chg1_dev);
    pinfo->hvdcp.is_connect = 0;
    chr_info("%s: OK\n", __func__);
    return ret;
}

static int hvdcp_v20_check_leave_status(struct charger_manager *pinfo)
{
    int ret = 0;
    
    return ret;  
}

int mtk_hvdcp_v20_check_charger(struct charger_manager *pinfo)
{
    int ret = 0;
    bool enabled = false;
    
    charger_dev_hvdcp_can_enabled(pinfo->chg1_dev,&enabled);
    pinfo->hvdcp.is_enabled = enabled;
    chr_info("%s: %d\n",__func__,pinfo->hvdcp.is_enabled);
    return ret;  
}

int mtk_hvdcp_v20_algorithm(struct charger_manager *pinfo)
{
    int ret = 0;
    struct hvdcp_v20 *hvdcp = &pinfo->hvdcp;
    if(!hvdcp->is_enabled)
        goto out;  
    
    if(hvdcp_v20_check_leave_status(pinfo)){
     charger_dev_reset_hvdcp_ta_ex(pinfo->chg1_dev);
      hvdcp->is_connect = 0;
      goto out;
    }
    
    if(hvdcp->is_connect){
      chr_info("%s: is connected\n",__func__);
      goto out;
    }
    charger_dev_send_hvdcp_pattern_ex(pinfo->chg1_dev,1);
    hvdcp->is_connect = 1;
out:
     chr_info("%s:is_enabled:%d, is_connect:%d\n",__func__,hvdcp->is_enabled,hvdcp->is_connect);

    return ret;
}

bool mtk_hvdcp_v20_get_is_connect(struct charger_manager *pinfo)
{
    return pinfo->hvdcp.is_connect;
}

bool mtk_hvdcp_v20_get_is_enable(struct charger_manager *pinfo)
{
    return pinfo->hvdcp.is_enabled;
}

int mtk_hvdcp_v20_init(struct charger_manager *pinfo)
{
    int ret = 0;
    
    pinfo->hvdcp.is_enabled = 0;
    pinfo->hvdcp.is_connect = 0;
    
    return ret;
}