/*
 * Copyright (C) 2017 MediaTek Inc.
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

#ifndef __FLASHLIGHT_CUSTOM_CTRL_H__
#define __FLASHLIGHT_CUSTOM_CTRL_H__

#include <linux/types.h>
#include <soc/oplus/system/oplus_project.h>

#include "richtek/rt-flashlight.h"
#include "../flashlight-core.h"

void Oplus2Channel_set_timeout_custom(unsigned int* cur_timeout, int channel, int timeout, int *ch1_timeout, int *ch2_timeout);
void Oplus2Channel_set_duty_custom(int cur_ch1_duty, int cur_ch2_duty, int channel, int duty, int *ch1_duty, int *ch2_duty);
void Oplus2Channel_set_onoff_custom(int channel, int param, int *ch1_status, int *ch2_status, int *need_op_ch_number); // need_op_ch_number: 0:ch1 1:ch2 3:ch1&ch2
int Oplusflashlight_set_strobe_timeout_custom(struct flashlight_device* flashlight_dev, int default_timeout);
int Oplusflashlight_get_scenairo_custom(int decouple);
int Oplus_get_current_duty_custom(const int *mt6360_current, int idx);
int Oplus_get_max_duty_torch_custom(int default_max_level);
int Oplus_get_max_duty_num_custom(int default_max_level);
int Oplus_get_hw_timeout_custom(int default_timeout);
void Oplus_get_torch_and_strobe_level_custom(int default_torch_level, int default_strobe_level, int idx, int *torch_level, int *strobe_level);

#endif
