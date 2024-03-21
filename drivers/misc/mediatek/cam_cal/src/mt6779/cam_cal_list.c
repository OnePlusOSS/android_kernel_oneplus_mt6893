// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define IMX586_MAX_EEPROM_SIZE 0x3FFF
#else
#define IMX586_MAX_EEPROM_SIZE 0x24D0
#endif
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define MAX_EEPROM_SIZE_16K 0x4000
#define MAX_EEPROM_SIZE_0 (0x3000)
#endif
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include <soc/oplus/system/oplus_project.h>
#endif
struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	/*Below is commom sensor */
	{GM1ST_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_0},
	{IMX471_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_0},
	{OV32A_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_0},
	{HI846_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_0},
	{GC02M0_SENSOR_ID0, 0xA8, Common_read_region},
	{GC02M1_SENSOR_ID0, 0xA8, Common_read_region},
	{OV02B10_SENSOR_ID, 0xA4, Common_read_region, MAX_EEPROM_SIZE_0},
	#else
	{IMX519_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2T7SP_SENSOR_ID, 0xA4, Common_read_region},
	{IMX386_SENSOR_ID, 0xA0, Common_read_region},
	{IMX386_MONO_SENSOR_ID, 0xA0, Common_read_region},
	{IMX398_SENSOR_ID, 0xA0, Common_read_region},
	{IMX350_SENSOR_ID, 0xA0, Common_read_region},
	{IMX586_SENSOR_ID, 0xA0, Common_read_region, IMX586_MAX_EEPROM_SIZE},
	{IMX576_SENSOR_ID, 0xA2, Common_read_region},
	{S5K2L7_SENSOR_ID, 0xA0, Common_read_region},
	{IMX230_SENSOR_ID, 0xA0, Common_read_region},
	{IMX338_SENSOR_ID, 0xA0, Common_read_region},
	{IMX318_SENSOR_ID, 0xA0, Common_read_region},
	{IMX258_SENSOR_ID, 0xA0, Common_read_region},
	{S5K4E6_SENSOR_ID, 0xA8, Common_read_region},
	#endif
	/*  ADD before this line */
	{0, 0, 0}	/*end of list */
};

#ifdef OPLUS_FEATURE_CAMERA_COMMON
struct stCAM_CAL_LIST_STRUCT g_camCalList_p95[] = {
        {S5KGW1_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_0},
        {S5KGH1_SENSOR_ID, 0xA8, Common_read_region},
        {GC8054_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_0},
        {S5K3M5_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_0},
        {GC02M0F_SENSOR_ID, 0xA8, Common_read_region},
        {IMX586_SENSOR_ID, 0xA0, Common_read_region, IMX586_MAX_EEPROM_SIZE},
        /*  ADD before this line */
        {0, 0, 0}       /*end of list */
};
#endif

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if (is_project(19550) || is_project(19551) || is_project(19553) || is_project(19556) ||
	    is_project(19357) || is_project(19354) || is_project(19358) || is_project(19359)) {
		*ppCamcalList = &g_camCalList_p95[0];
	}
	#endif
	return 0;
}


