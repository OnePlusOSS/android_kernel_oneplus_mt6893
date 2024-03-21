/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define MAX_EEPROM_SIZE_16K 0x4000
#endif

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	/*Below is commom sensor */
	{IMX519_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2T7SP_SENSOR_ID, 0xA4, Common_read_region},
	{IMX338_SENSOR_ID, 0xA0, Common_read_region},
	{S5K4E6_SENSOR_ID, 0xA8, Common_read_region},
	{IMX386_SENSOR_ID, 0xA0, Common_read_region},
	{S5K3M3_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2L7_SENSOR_ID, 0xA0, Common_read_region},
	{IMX398_SENSOR_ID, 0xA0, Common_read_region},
	{IMX350_SENSOR_ID, 0xA0, Common_read_region},
	{IMX318_SENSOR_ID, 0xA0, Common_read_region},
	{IMX386_MONO_SENSOR_ID, 0xA0, Common_read_region},
	{MIAMI_OV64B_SENSOR_ID, 0xA0, Common_read_region},
	{MIAMI_AAC2_OV64B_SENSOR_ID, 0xA0, Common_read_region},
	{MIAMI_S5KJN1_SENSOR_ID, 0xA0, Common_read_region},
	{MIAMI_SC800CS_SENSOR_ID, 0xA2, Common_read_region},
	{MIAMI_OV8856_SENSOR_ID, 0xA2, Common_read_region},
	{ALADDIN_OV50D40_SENSOR_ID, 0xA0, Common_read_region},
	{ALADDIN_OV08D10_SENSOR_ID, 0xA0, Common_read_region},
	{ATOM_OV50D40_SENSOR_ID, 0xA0, Common_read_region},
	{ATOM_IMX355_SENSOR_ID, 0xA0, Common_read_region},
	{ATOM_S5K5E9YX04_SENSOR_ID, 0xA8, Common_read_region},
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{EVEN_QTECH_MAIN_S5KGM1ST03_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{EVEN_QTECH_MAIN_OV13B10_SENSOR_ID, 0xA0, Common_read_region},
	{EVEN_HLT_DEPTH_GC02M1B_MIPI_SENSOR_ID, 0xA2, Common_read_region},
	{EVEN_SHENGTAI_MACRO_OV02B10_SENSOR_ID, 0xA4, Common_read_region},
	{EVEN_SHENGTAI_FRONT_OV8856_SENSOR_ID, 0xA0, Common_read_region},
	{EVEN_SHINETECH_MAIN_S5KJN103_SENSOR_ID, 0xA0, Common_read_region},
	{EVENC_SHENGTAI_FRONT_OV8856_SENSOR_ID, 0xA0, Common_read_region},
	{EVENC_SHENGTAI_MACRO_OV02B10_SENSOR_ID, 0xA4, Common_read_region},
#endif

	/*B+B. No Cal data for main2 OV8856*/
	{S5K2P7_SENSOR_ID, 0xA0, Common_read_region},
#ifdef SUPPORT_S5K4H7
	{S5K4H7_SENSOR_ID, 0xA0, zte_s5k4h7_read_region},
	{S5K4H7SUB_SENSOR_ID, 0xA0, zte_s5k4h7_sub_read_region},
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


