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

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{OV13B10_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX355_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV02B10_SENSOR_ID, 0xA4, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX615_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV64B_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV64B_SENSOR_ID_212A1, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX355_SENSOR_ID_212A1, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX615_SENSOR_ID_212A1, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV16A10_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3P9SP_SENSOR_ID, 0xA8, Common_read_region},
	{OV48B_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV32A_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX471_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{GC02K0_SENSOR_ID, 0xA4, Common_read_region},
	{GC02M1B_SENSOR_ID1, 0xA2, Common_read_region},
	{HI846_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5KGM1ST_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5KGW3_SENSOR_ID_20631, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV32A1Q_SENSOR_ID_20631, 0xA8, Common_read_region},
	{HI846_SENSOR_ID_20631, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{GC02K_SENSOR_ID_20631, 0xA4, Common_read_region},
	{S5KGM1ST_SENSOR_ID_20633c, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX471_SENSOR_ID_20633c, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HI846_SENSOR_ID_20633c, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5KGM1ST_SENSOR_ID_ANNA, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX471_SENSOR_ID_ANNA, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HI846_SENSOR_ID_ANNA, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV02B10_SENSOR_ID_ANNA, 0xA4, Common_read_region},
	{S5KGW3_SENSOR_ID_ANNA, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX581_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{GC02M1_SENSOR_ID_DUFU, 0xA4, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5KGM1ST_SENSOR_ID_20611, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3P9SP_SENSOR_ID_20611, 0xA8, Common_read_region},
	{HI846_SENSOR_ID_20611, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{GC02K_SENSOR_ID_20611, 0xA4, Common_read_region},
	{ATHENSD_S5KGW3_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{ATHENSD_S5KGW3P1_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{ATHENSD_IMX471_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{ATHENSD_HI846_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{ATHENSD_GC02K_SENSOR_ID, 0xA4, Common_read_region},
#endif
	{IMX586_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K,
		BL24SA64_write_region},
	{IMX576_SENSOR_ID, 0xA2, Common_read_region},
	{IMX519_SENSOR_ID, 0xA0, Common_read_region},
	{IMX319_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3M5SX_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K,
		BL24SA64_write_region},
	{IMX686_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HI846_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5KGD1SP_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV16A10_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3P9SP_SENSOR_ID, 0xA8, Common_read_region},
	{S5K2T7SP_SENSOR_ID, 0xA4, Common_read_region},
	{IMX386_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2L7_SENSOR_ID, 0xA0, Common_read_region},
	{IMX398_SENSOR_ID, 0xA0, Common_read_region},
	{IMX350_SENSOR_ID, 0xA0, Common_read_region},
	{IMX386_MONO_SENSOR_ID, 0xA0, Common_read_region},
	{S5KJD1_SENSOR_ID, 0xB0, Common_read_region, DEFAULT_MAX_EEPROM_SIZE_8K,
		DW9763_write_region},
	{IMX499_SENSOR_ID, 0xA0, Common_read_region},
	{IMX481_SENSOR_ID, 0xA4, Common_read_region, DEFAULT_MAX_EEPROM_SIZE_8K,
		BL24SA64_write_region},
	{IMX766_SENSOR_ID_MOSS, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{IMX471_SENSOR_ID_MOSS, 0xA8, Common_read_region},
	{IMX355_SENSOR_ID_MOSS, 0xA2, Common_read_region},
	{GC02M1_SENSOR_ID_MOSS, 0xA4, Common_read_region},
	{S5KGM1ST_SENSOR_ID_MOSSA, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{GC02M1_SENSOR_ID_MOSSA, 0xA4, Common_read_region},

	{OV48B_SENSOR_ID_CHIVAS, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV32A_SENSOR_ID_CHIVAS, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HI846_SENSOR_ID_CHIVAS, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{GC02K0_SENSOR_ID_CHIVAS, 0xA4, Common_read_region},
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


