/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifndef __IMGSENSOR_HWCFG_mt6785_CTRL_H__
#define __IMGSENSOR_HWCFG_mt6785_CTRL_H__
#include "imgsensor_hwcfg_custom.h"


struct IMGSENSOR_SENSOR_LIST *oplus_gimgsensor_sensor_list = NULL;
struct IMGSENSOR_HW_CFG *oplus_imgsensor_custom_config = NULL;
struct IMGSENSOR_HW_POWER_SEQ *oplus_sensor_power_sequence = NULL;
struct CAMERA_DEVICE_INFO gImgEepromInfo;

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_21684[MAX_NUM_OF_SUPPORT_SENSOR] = {

#if defined(IMX682_MIPI_RAW)
{IMX682_SENSOR_ID, SENSOR_DRVNAME_IMX682_MIPI_RAW, IMX682_MIPI_RAW_SensorInit},
#endif
#if defined(S5KGM1SP_MIPI_RAW)
{S5KGM1SP_SENSOR_ID, SENSOR_DRVNAME_S5KGM1SP_MIPI_RAW, S5KGM1SP_MIPI_RAW_SensorInit},
#endif
#if defined(IMX471_MIPI_RAW1)
{IMX471_SENSOR_ID1, SENSOR_DRVNAME_IMX471_MIPI_RAW1, IMX471_MIPI_RAW1_SensorInit},
#endif
#if defined(OV8856_MIPI_RAW)
{OV8856_SENSOR_ID, SENSOR_DRVNAME_OV8856_MIPI_RAW, OV8856_MIPI_RAW_SensorInit},
#endif
#if defined(OV02B10_MIPI_RAW)
{OV02B10_SENSOR_ID, SENSOR_DRVNAME_OV02B10_MIPI_RAW, OV02B10_MIPI_RAW_SensorInit},
#endif
#if defined(OV02B1B_MIPI_MONO)
{OV02B1B_SENSOR_ID, SENSOR_DRVNAME_OV02B1B_MIPI_MONO, OV02B1B_MIPI_MONO_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_21690[MAX_NUM_OF_SUPPORT_SENSOR] = {

#if defined(OV64B40_MIPI_RAW_21690)
{OV64B40_SENSOR_ID_21690, SENSOR_DRVNAME_OV64B40_MIPI_RAW_21690, OV64B40_MIPI_RAW_21690_SensorInit},
#endif
#if defined(S5KJN1_MIPI_RAW_21690)
{S5KJN1_SENSOR_ID_21690, SENSOR_DRVNAME_S5KJN1_MIPI_RAW_21690, S5KJN1_MIPI_RAW_21690_SensorInit},
#endif
#if defined(HI1634Q_MIPI_RAW_21690)
{HI1634Q_SENSOR_ID_21690, SENSOR_DRVNAME_HI1634Q_MIPI_RAW_21690, HI1634Q_MIPI_RAW_21690_SensorInit},
#endif
#if defined(S5K3P9SP_MIPI_RAW_21690)
{S5K3P9SP_SENSOR_ID_21690, SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW_21690, S5K3P9SP_MIPI_RAW_21690_SensorInit},
#endif
#if defined(GC02M1HLT_MIPI_RAW_21690)
{GC02M1HLT_SENSOR_ID_21690, SENSOR_DRVNAME_GC02M1HLT_MIPI_RAW_21690, GC02M1HLT_MIPI_RAW_21690_SensorInit},
#endif
#if defined(GC02M1SHINE_MIPI_RAW_21690)
{GC02M1SHINE_SENSOR_ID_21690, SENSOR_DRVNAME_GC02M1SHINE_MIPI_RAW_21690, GC02M1SHINE_MIPI_RAW_21690_SensorInit},
#endif
#if defined(OV02B1B_MIPI_MONO_21690)
{OV02B1B_SENSOR_ID_21690, SENSOR_DRVNAME_OV02B1B_MIPI_MONO_21690, OV02B1B_MIPI_MONO_21690_SensorInit},
#endif
#if defined(GC02M1BSY_MIPI_RAW_21690)
{GC02M1BSY_SENSOR_ID_21690, SENSOR_DRVNAME_GC02M1BSY_MIPI_RAW_21690, GC02M1BSY_MIPI_RAW_21690_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_21684[] = {
	{
		IMGSENSOR_SENSOR_IDX_MAIN,
		IMGSENSOR_I2C_DEV_0,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},//IMGSENSOR_HW_ID_REGULATOR
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_SUB,
		IMGSENSOR_I2C_DEV_1,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_MAIN2,
		IMGSENSOR_I2C_DEV_1,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_SUB2,
		IMGSENSOR_I2C_DEV_0,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_MAIN3,
		IMGSENSOR_I2C_DEV_1,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
	{IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_21690[] = {
	{
		IMGSENSOR_SENSOR_IDX_MAIN,
		IMGSENSOR_I2C_DEV_0,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_SUB,
		IMGSENSOR_I2C_DEV_1,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_MAIN2,
		IMGSENSOR_I2C_DEV_1,
		{

			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
 			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_WL2864},
 			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
#ifdef MIPI_SWITCH
			{IMGSENSOR_HW_PIN_MIPI_SWITCH_EN, IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,
				IMGSENSOR_HW_ID_GPIO},
#endif
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_SUB2,
		IMGSENSOR_I2C_DEV_3,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_NONE},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_NONE},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_MAIN3,
		IMGSENSOR_I2C_DEV_1,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
	{IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_21684[] = {
#if defined(S5KGM1SP_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KGM1SP_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_Low, 1},
			{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 0},
			{AFVDD, Vol_2800, 2},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(IMX471_MIPI_RAW1)
	{
		SENSOR_DRVNAME_IMX471_MIPI_RAW1,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1100, 0},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 3}
		},
	},
#endif
#if defined(OV8856_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV8856_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{RST, Vol_High, 3}
		},
	},
#endif
#if defined(OV02B10_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV02B10_MIPI_RAW,
		{
			{PDN, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 6},
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_High, 10, Vol_High, 1},
		},
	},
#endif
#if defined(OV02B1B_MIPI_MONO)
	{
		SENSOR_DRVNAME_OV02B1B_MIPI_MONO,
		{
			{PDN, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 6},
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_High, 10, Vol_High, 1},
		},
	},
#endif
#if defined(GC02M1B_MIPI_RAW_MACRO)
    {
        SENSOR_DRVNAME_GC02M1B_MIPI_RAW_MACRO,
        {
            {PDN, Vol_Low, 1},
            {DVDD, Vol_1800, 0},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 1},
            {PDN, Vol_High, 2},
            {SensorMCLK, Vol_High, 1}
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_21690[] = {
#if defined(OV64B40_MIPI_RAW_21690)
	{
		SENSOR_DRVNAME_OV64B40_MIPI_RAW_21690,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1100, 1},
			{DOVDD, Vol_1800, 1},
			{AFVDD, Vol_2800, 2},
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(S5KJN1_MIPI_RAW_21690)
	{
		SENSOR_DRVNAME_S5KJN1_MIPI_RAW_21690,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1050, 1},
			{AVDD, Vol_2800, 5},
			{AFVDD, Vol_2800, 0},
			{RST, Vol_High, 2},
			{SensorMCLK, Vol_High, 2}
		},
	},
#endif
#if defined(HI1634Q_MIPI_RAW_21690)
	{
		SENSOR_DRVNAME_HI1634Q_MIPI_RAW_21690,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 1},
			{SensorMCLK, Vol_High, 5},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(S5K3P9SP_MIPI_RAW_21690)
	{
		SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW_21690,
		{
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_Low, 1},
			{DVDD, Vol_1100, 1},
			{AVDD, Vol_2800, 1},
			{DOVDD, Vol_1800, 0},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(GC02M1HLT_MIPI_RAW_21690)
	{
		SENSOR_DRVNAME_GC02M1HLT_MIPI_RAW_21690,
		{
			{RST, Vol_Low, 3},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{RST, Vol_High, 5},
			{SensorMCLK, Vol_High, 3},
		},
	},
#endif
#if defined(GC02M1SHINE_MIPI_RAW_21690)
	{
		SENSOR_DRVNAME_GC02M1SHINE_MIPI_RAW_21690,
		{
			{RST, Vol_Low, 3},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{RST, Vol_High, 5},
			{SensorMCLK, Vol_High, 3},
		},
	},
#endif
#if defined(OV02B1B_MIPI_MONO_21690)
	{
		SENSOR_DRVNAME_OV02B1B_MIPI_MONO_21690,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 6},
			{SensorMCLK, Vol_High, 5},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(GC02M1BSY_MIPI_RAW_21690)
	{
		SENSOR_DRVNAME_GC02M1BSY_MIPI_RAW_21690,
		{
			{RST, Vol_Low, 3},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{RST, Vol_High, 5},
			{SensorMCLK, Vol_High, 3},
		},
	},
#endif
    /* add new sensor before this line */
    {NULL,},
};
struct CAMERA_DEVICE_INFO gImgEepromInfo_21684 = {
    .i4SensorNum = 5,
    .pCamModuleInfo = {
        {S5KGM1SP_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r",  "s5kgm1sp"},
        {IMX471_SENSOR_ID1,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f", "imx471"},
        {OV8856_SENSOR_ID, 0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "ov8856"},
        {OV02B10_SENSOR_ID,  0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "ov02b10"},
        {OV02B1B_SENSOR_ID, 0xFF, {0xFF, 0xFF}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r3", "ov02b1b"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {OV48B_STEREO_START_ADDR,HI846_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

void oplus_imgsensor_hwcfg()
{
    if(is_project(21690) || is_project(21691) || is_project(21692)
        || is_project(21684) || is_project(21685) || is_project(21686)) {
        oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_21690;
        oplus_imgsensor_custom_config = imgsensor_custom_config_21690;
        oplus_sensor_power_sequence = sensor_power_sequence_21690;
        //gImgEepromInfo = gImgEepromInfo_21684;
    } else {
        oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_21684;
        oplus_imgsensor_custom_config = imgsensor_custom_config_21684;
        oplus_sensor_power_sequence = sensor_power_sequence_21684;
        //gImgEepromInfo = gImgEepromInfo_21684;
    }
}
#endif
