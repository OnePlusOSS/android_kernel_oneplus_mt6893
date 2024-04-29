/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

#define MAX_EEPROM_SIZE_16K 0x4000
#define MAX_EEPROM_SIZE_32K 0x8000
#define MAX_EEPROM_SIZE_8K 0x2000

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
	{OV64B_SENSOR_ID_21851, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX355_SENSOR_ID_21851, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX471_SENSOR_ID_21851, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
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

	{IMX766_SENSOR_ID22693, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{S5KHM6SP_SENSOR_ID22693, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3P9SP_SENSOR_ID22693, 0xA8, Common_read_region},
	{IMX355_SENSOR_ID22693, 0xA2, Common_read_region},
	{GC02M1_SENSOR_ID22693, 0xA4, Common_read_region},
	{S5KJN1_SENSOR_ID23707, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HI1634Q_SENSOR_ID23707, 0xA2, Common_read_region, MAX_EEPROM_SIZE_8K},

	{S5KHP3SP_SENSOR_ID22629, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{IMX615_SENSOR_ID22629, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX355_SENSOR_ID22629, 0xA2, Common_read_region},
	{GC02M1_SENSOR_ID22629, 0xA4, Common_read_region},

	{IMX882_SENSOR_ID23687, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX890_SENSOR_ID23689, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HI1634Q_SENSOR_ID23687, 0xA2, Common_read_region, MAX_EEPROM_SIZE_8K},
	{HI846W_SENSOR_ID23687, 0x42, hi846w_read_region, MAX_EEPROM_SIZE_8K},
	{GC02M1_SENSOR_ID23687, 0xA4, Common_read_region},
	{IMX882PD_SENSOR_ID23687, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},

	{OVA0B4_SENSOR_ID22633, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3P9SP_SENSOR_ID22633, 0xA8, Common_read_region},

	{OV48B_SENSOR_ID_CHIVAS, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV32A_SENSOR_ID_CHIVAS, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HI846_SENSOR_ID_CHIVAS, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{GC02K0_SENSOR_ID_CHIVAS, 0xA4, Common_read_region},

	{OV64B_SENSOR_ID22277, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV32C_SENSOR_ID22277, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX355_SENSOR_ID22277, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX709_SENSOR_ID22277, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},

	{IMX882_SENSOR_ID23231, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV32C_SENSOR_ID23231, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX355_SENSOR_ID23231, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX709_SENSOR_ID23231, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX882PD_SENSOR_ID23231, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},

	{OV64B_SENSOR_ID23031, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV08D_SENSOR_ID23031, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
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


