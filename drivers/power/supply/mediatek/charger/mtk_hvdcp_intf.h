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

struct hvdcp_v20 {
    bool is_enabled;
    bool is_connect;
};
extern int mtk_hvdcp_v20_check_charger(struct charger_manager *pinfo);
extern int mtk_hvdcp_v20_algorithm(struct charger_manager *pinfo);
extern bool mtk_hvdcp_v20_get_is_connect(struct charger_manager *pinfo);
extern bool mtk_hvdcp_v20_get_is_enable(struct charger_manager *pinfo);
extern int mtk_hvdcp_v20_init(struct charger_manager *pinfo);
extern int mtk_hvdcp_v20_reset_ta_vchr(struct charger_manager *pinfo);

