/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
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
	/*add for 20601 OTP  */
	{IMX686_SENSOR_ID_20601, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV32A1Q_SENSOR_ID_20601,0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HI846_SENSOR_ID_20601 , 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{GC2385_SENSOR_ID_20601, 0xA4, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV64B_SENSOR_ID_20645, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV32A1Q_SENSOR_ID_20645, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV02B10_SENSOR_ID_20645, 0xA4, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HI846_SENSOR_ID_20645, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV64B_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX766_SENSOR_ID_21015, 0xA2, Common_read_region, MAX_EEPROM_SIZE_32K},
	{IMX615_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX355_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV02B10_SENSOR_ID, 0xA4, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX319_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX616_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX319_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	/*add for 20615 OTP*/
	{IMX682_SENSOR_ID_20615, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX471_SENSOR_ID_20615, 0xA8, Common_read_region},
	{HI846_SENSOR_ID_20615, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV02B10_SENSOR_ID_20615, 0xA4, Common_read_region},
	/*add for 20619 OTP*/
	{OV64B_SENSOR_ID_20619, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX471_SENSOR_ID_20619, 0xA8, Common_read_region},
	{HI846_SENSOR_ID_20619, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV02B10_SENSOR_ID_20619, 0xA4, Common_read_region},
	/*add for 21651 OTP*/
	{OV64B_SENSOR_ID_21651, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX471_SENSOR_ID_21651, 0xA8, Common_read_region},
	{IMX355_SENSOR_ID_21651, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{GC02M1_SENSOR_ID_21651, 0xA4, Common_read_region},
	/*add for 21127 OTP*/
	{OV50A_SENSOR_ID_21127,  0xA2, Common_read_region, MAX_EEPROM_SIZE_32K},
	{IMX471_SENSOR_ID_21127, 0xA8, Common_read_region},
	{IMX709_SENSOR_ID_21127, 0xA6, Common_read_region},
	{GC02M1_SENSOR_ID_21127, 0xA4, Common_read_region},
	/*add for 21305 OTP*/
	{IMX766_SENSOR_ID_21305, 0xA2, Common_read_region, MAX_EEPROM_SIZE_32K},
	{IMX615_SENSOR_ID_21305, 0xA8, Common_read_region},
	{IMX709_SENSOR_ID_21305, 0xA6, Common_read_region},
	{IMX355_SENSOR_ID_21305, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{GC02M1_SENSOR_ID_21305, 0xA4, Common_read_region},
	/*add for 19165 */
	{IMX686_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HI846_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5KGD1SP_SENSOR_ID, 0xA8, Common_read_region},
	/*add for 20827 20831 OTP*/
	{IMX766_SENSOR_ID_20817, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{IMX615_SENSOR_ID_20817, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX355_SENSOR_ID_20817, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	/*add for 21881 21882 OTP*/
	{IMX766_SENSOR_ID_21881, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{IMX615_SENSOR_ID_21881, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX355_SENSOR_ID_21881, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	#else
	/*Below is commom sensor */
	{IMX586_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K,
		BL24SA64_write_region},
	{IMX576_SENSOR_ID, 0xA2, Common_read_region},
	{IMX519_SENSOR_ID, 0xA0, Common_read_region},
	{IMX319_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3M5SX_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K,
		BL24SA64_write_region},
	{IMX686_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HI846_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5KGD1SP_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K2T7SP_SENSOR_ID, 0xA4, Common_read_region},
	{IMX386_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2L7_SENSOR_ID, 0xA0, Common_read_region},
	{IMX398_SENSOR_ID, 0xA0, Common_read_region},
	{IMX350_SENSOR_ID, 0xA0, Common_read_region},
	{IMX386_MONO_SENSOR_ID, 0xA0, Common_read_region},
	{IMX499_SENSOR_ID, 0xA0, Common_read_region},
	{S5KJD1_SENSOR_ID, 0xB0, Common_read_region, DEFAULT_MAX_EEPROM_SIZE_8K,
		DW9763_write_region},
	{IMX481_SENSOR_ID, 0xA4, Common_read_region, DEFAULT_MAX_EEPROM_SIZE_8K,
		BL24SA64_write_region},
	{OV13B10LN_SENSOR_ID, 0xB0, Common_read_region},
#ifdef SUPPORT_S5K4H7
	{S5K4H7LN_SENSOR_ID, 0x20, s5k4h7_read_region},
#endif
	/*  ADD before this line */
	#endif
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


