/*
 * Copyright (C) 2018 MediaTek Inc.
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
#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

#define MAX_EEPROM_SIZE_16K 0x4000
#define MAX_EEPROM_SIZE_32K 0x8000

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	{OV64B_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX766_SENSOR_ID_20817, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{IMX615_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX355_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX615_SENSOR_ID_20817, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX355_SENSOR_ID_20817, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV02B10_SENSOR_ID, 0xA4, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX616_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX319_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
/*  ADD before this line */
	{0, 0, 0}       /*end of list */
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	return 0;
}


