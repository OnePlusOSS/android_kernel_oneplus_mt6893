/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mt6785-afe-gpio.h  --  Mediatek 6785 afe gpio ctrl definition
 *
 * Copyright (c) 2017 MediaTek Inc.
 * Author: Eason Yen <eason.yen@mediatek.com>
 */

#ifndef _MT6785_AFE_GPIO_H_
#define _MT6785_AFE_GPIO_H_

struct mtk_base_afe;

int mt6785_afe_gpio_init(struct mtk_base_afe *afe);

int mt6785_afe_gpio_request(struct mtk_base_afe *afe, bool enable,
			    int dai, int uplink);

#ifdef OPLUS_BUG_STABILITY
int mt6785_afe_gpio_extamp_select(struct mtk_base_afe *afe, bool enable, int mode);
#endif /* OPLUS_BUG_STABILITY */
#endif
