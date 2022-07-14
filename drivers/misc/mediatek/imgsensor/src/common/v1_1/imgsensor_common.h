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

#ifndef __IMGSENSOR_COMMON_H__
#define __IMGSENSOR_COMMON_H__

#include "kd_camera_feature.h"
#include "kd_imgsensor_define.h"

/******************************************************************************
 * Debug configuration
 ******************************************************************************/
#define PREFIX "[imgsensor]"
#define PLATFORM_POWER_SEQ_NAME "platform_power_seq"

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG(fmt, arg...)  pr_debug(PREFIX fmt, ##arg)
#define PK_PR_ERR(fmt, arg...)  pr_err(fmt, ##arg)
#define PK_INFO(fmt, arg...) pr_debug(PREFIX fmt, ##arg)
#else
#define PK_DBG(fmt, arg...)
#define PK_PR_ERR(fmt, arg...)  pr_err(fmt, ##arg)
#define PK_INFO(fmt, arg...) pr_debug(PREFIX fmt, ##arg)
#endif

#define IMGSENSOR_LEGACY_COMPAT

#define IMGSENSOR_TOSTRING(value)           #value
#define IMGSENSOR_STRINGIZE(stringizedName) IMGSENSOR_TOSTRING(stringizedName)

#if OPLUS_FEATURE_CAMERA_COMMON
#define MIPI_SWITCH
#endif

enum IMGSENSOR_ARCH {
	IMGSENSOR_ARCH_V1 = 0,
	IMGSENSOR_ARCH_V2,
	IMGSENSOR_ARCH_V3
};

enum IMGSENSOR_RETURN {
	IMGSENSOR_RETURN_SUCCESS = 0,
	IMGSENSOR_RETURN_ERROR   = -1,
};

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define CAMERA_MODULE_SN_LENGTH                (20)
#define AESYNC_DATA_LENGTH_TOTAL               (65)
#define DUALCAM_CALI_DATA_LENGTH               (1561)
#define DUALCAM_CALI_DATA_LENGTH_8ALIGN        (1568)
#define DUALCAM_CALI_DATA_LENGTH_QCOM_MAIN     (1561+128)
#define IMX586_STEREO_START_ADDR               (0x2600)
#define IMX586_STEREO_START_ADDR_WIDE          (0x2600)
#define IMX586_STEREO_START_ADDR_TELE          (0x2CB0)
#define IMX586_AESYNC_START_ADDR               (0x3360)
#define S5K3M5SX_STEREO_START_ADDR             (0x2600)
#define S5K3M5SX_AESYNC_START_ADDR             (0x2C60)
#define IMX319_STEREO_START_ADDR               (0x2600)
#define IMX319_AESYNC_START_ADDR               (0x2D00)
#define DUALCAM_CALI_DATA_LENGTH_TOTAL_TELE    (2450)
#define DUALCAM_CALI_DATA_LENGTH_TELE          (909)
#define CAMERA_MODULE_INFO_LENGTH              (8)
#define S5K3P9S_DUALCAM_CALI_PART2_LENGTH      (0x115)
#define OV48B_STEREO_START_ADDR          (0x2840) //
#define OV48B_STEREO_START_ADDR_WIDE     (0x2840)
#define OV48B_STEREO_START_ADDR_TELE     (0x2EA0)
#define OV48B_AESYNC_START_ADDR          (0x2820) //for later use
#define GC8054_STEREO_START_ADDR          (0x25D0)
#define GC8054_AESYNC_START_ADDR          (0x0C90) //for later use
#define S5K3P9SP_STEREO_START_ADDR        (0x0700) //
#define S5K3P9SP_STEREO_START_ADDR2        (0x1D00)
#define GC02M0B_STEREO_START_ADDR        (0x1500) //
#define S5K3P9S_DUALCAM_CALI_PART_LENGTH    (0x500)
#define S5KGM1ST_STEREO_START_ADDR          (0x2840) //
#define OV16A10_AESYNC_START_ADDR              (0x2E60)
#define OV16A10_STEREO_START_ADDR              (0x2F00) //
#define OV16A10_STEREO_START_ADDR_WIDE         (0x2F00)
#define OV16A10_STEREO_START_ADDR_TELE         (0x3520)
#define S5KGW3_AESYNC_START_ADDR               (0x2540)
#define S5KGW3_STEREO_START_ADDR               (0x1900) //
#define S5KGW3_STEREO_START_ADDR_WIDE          (0x1900)
#define S5KGW3_STEREO_START_ADDR_TELE          (0x1F20)
#define IMX471_STEREO_START_ADDR               (0x1900) //
#define GC02M1B_STEREO_START_ADDR              (0x0E28) //
extern char gOtpCheckdata[7][40];
#endif

#define LENGTH_FOR_SNPRINTF 256
#endif

