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
#ifdef SENSOR_PLATFORM_4G_20682
struct IMGSENSOR_HW_POWER_SEQ *oplus_platform_power_sequence = NULL;
#endif
struct CAMERA_DEVICE_INFO gImgEepromInfo;


struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20682[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(SALAA_QTECH_MAIN_S5KGM1SP)
{SALAA_QTECH_MAIN_S5KGM1SP_SENSOR_ID, SENSOR_DRVNAME_SALAA_QTECH_MAIN_S5KGM1SP, SALAA_QTECH_MAIN_S5KGM1SP_SensorInit},
#endif
#if defined(IMX682_MIPI_RAW)
{IMX682_SENSOR_ID, SENSOR_DRVNAME_IMX682_MIPI_RAW, IMX682_MIPI_RAW_SensorInit},
#endif
#if defined(S5KGM1SP_MIPI_RAW)
{S5KGM1SP_SENSOR_ID, SENSOR_DRVNAME_S5KGM1SP_MIPI_RAW, S5KGM1SP_MIPI_RAW_SensorInit},
#endif
#if defined(IMX471_MIPI_RAW1)
{IMX471_SENSOR_ID1, SENSOR_DRVNAME_IMX471_MIPI_RAW1, IMX471_MIPI_RAW1_SensorInit},
#endif
#if defined(SALA_WIDE_OV8856)
{SALA_WIDE_OV8856_SENSOR_ID,SENSOR_DRVNAME_SALA_WIDE_OV8856,SALA_WIDE_OV8856_SensorInit},
#endif
#if defined(OV8856_MIPI_RAW)
{OV8856_SENSOR_ID, SENSOR_DRVNAME_OV8856_MIPI_RAW, OV8856_MIPI_RAW_SensorInit},
#endif
#if defined(SALA_OV02B10_MIPI_RAW)
{SALA_OV02B10_SENSOR_ID, SENSOR_DRVNAME_SALA_OV02B10_MIPI_RAW, SALA_OV02B10_MIPI_RAW_SensorInit},
#endif
#if defined(OV02B1B_MIPI_MONO)
{OV02B1B_SENSOR_ID, SENSOR_DRVNAME_OV02B1B_MIPI_MONO, OV02B1B_MIPI_MONO_SensorInit},
#endif
#if defined(OV64B_MIPI_RAW)
{OV64B_SENSOR_ID, SENSOR_DRVNAME_OV64B_MIPI_RAW, OV64B_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO)
{GC02M1B_SENSOR_ID, SENSOR_DRVNAME_GC02M1B_MIPI_MONO, GC02M1B_MIPI_MONO_SensorInit},
#endif
#if defined(GC02M1B_MIPI_RAW_20601)
{GC02M1B_SENSOR_ID_20601, SENSOR_DRVNAME_GC02M1B_MIPI_RAW_20601, GC02M1B_MIPI_RAW_20601_SensorInit},
#endif
#if defined(S5KGM1ST_MIPI_RAW)
{S5KGM1ST_SENSOR_ID, SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW, S5KGM1ST_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M1B_MIPI_RAW_MACRO)
{GC02M1B_SENSOR_ID_MACRO, SENSOR_DRVNAME_GC02M1B_MIPI_RAW_MACRO, GC02M1B_MIPI_RAW_MACRO_SensorInit},
#endif

    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20730[MAX_NUM_OF_SUPPORT_SENSOR] = {

#if defined(OV64B_MIPI_RAW_20730)
{OV64B_SENSOR_ID_20730, SENSOR_DRVNAME_OV64B_MIPI_RAW_20730, OV64B_MIPI_RAW_20730_SensorInit},
#endif
#if defined(IMX471_MIPI_RAW2_20730)
{IMX471_SENSOR_ID2_20730, SENSOR_DRVNAME_IMX471_MIPI_RAW2_20730, IMX471_MIPI_RAW2_20730_SensorInit},
#endif
#if defined(OV8856_MIPI_RAW2_20730)
{OV8856_SENSOR_ID2_20730, SENSOR_DRVNAME_OV8856_MIPI_RAW2_20730, OV8856_MIPI_RAW2_20730_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO_20730)
{GC02M1B_SENSOR_ID_20730, SENSOR_DRVNAME_GC02M1B_MIPI_MONO_20730, GC02M1B_MIPI_MONO_20730_SensorInit},
#endif
#if defined(GC02M1_MIPI_RAW_20730)
{GC02M1_SENSOR_ID_20730, SENSOR_DRVNAME_GC02M1_MIPI_RAW_20730, GC02M1_MIPI_RAW_20730_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_19661[MAX_NUM_OF_SUPPORT_SENSOR] = {

#if defined(S5KGW1_MIPI_RAW)
{S5KGW1_SENSOR_ID, SENSOR_DRVNAME_S5KGW1_MIPI_RAW, S5KGW1_MIPI_RAW_SensorInit},
#endif
#if defined(S5K3P9SP_MIPI_RAW)
{S5K3P9SP_SENSOR_ID, SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW, S5K3P9SP_MIPI_RAW_SensorInit},
#endif
#if defined(GC2375H_MIPI_RAW_19661)
{GC2375H_SENSOR_ID_19661, SENSOR_DRVNAME_GC2375H_MIPI_RAW_19661, GC2375H_MIPI_RAW_19661_SensorInit},
#endif
#if defined(OV8856_MIPI_RAW)
{OV8856_SENSOR_ID, SENSOR_DRVNAME_OV8856_MIPI_RAW, OV8856_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M0_MIPI_MONO)
{GC02M0_SENSOR_ID, SENSOR_DRVNAME_GC02M0_MIPI_MONO, GC02M0_MIPI_MONO_SensorInit},
#endif
#if defined(S5KGM1SP_MIPI_RAW)
{S5KGM1SP_SENSOR_ID, SENSOR_DRVNAME_S5KGM1SP_MIPI_RAW, S5KGM1SP_MIPI_RAW_SensorInit},
#endif
#if defined(GC02K0_MIPI_RAW)
{GC02K0_SENSOR_ID, SENSOR_DRVNAME_GC02K0_MIPI_RAW, GC02K0_MIPI_RAW_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20682[] = {
{
		IMGSENSOR_SENSOR_IDX_MAIN,
		IMGSENSOR_I2C_DEV_0,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_AVDD2, IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_WL2864},
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
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
#ifdef MIPI_SWITCH
			{IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL, IMGSENSOR_HW_ID_GPIO},
#endif
			{IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_MAIN2,
		IMGSENSOR_I2C_DEV_1,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
#ifdef MIPI_SWITCH
			{IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL, IMGSENSOR_HW_ID_GPIO},
#endif
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_SUB2,
		IMGSENSOR_I2C_DEV_0,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_WL2864},
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
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
	{IMGSENSOR_SENSOR_IDX_NONE}
};


struct IMGSENSOR_HW_CFG imgsensor_custom_config_20730[] = {
{
		IMGSENSOR_SENSOR_IDX_MAIN,
		IMGSENSOR_I2C_DEV_0,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_AVDD2, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
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
#ifdef MIPI_SWITCH
			{IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL, IMGSENSOR_HW_ID_GPIO},
#endif
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
#ifdef MIPI_SWITCH
			{IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL, IMGSENSOR_HW_ID_GPIO},
#endif
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

struct IMGSENSOR_HW_CFG imgsensor_custom_config_19661[] = {
	{
		IMGSENSOR_SENSOR_IDX_MAIN,
		IMGSENSOR_I2C_DEV_0,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_WL2864},
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
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
#ifdef MIPI_SWITCH
			{IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL, IMGSENSOR_HW_ID_GPIO},
#endif
			{IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_MAIN2,
		IMGSENSOR_I2C_DEV_1,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
#ifdef MIPI_SWITCH
			{IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL, IMGSENSOR_HW_ID_GPIO},
#endif
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_SUB2,
		IMGSENSOR_I2C_DEV_0,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_WL2864},
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
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_WL2864},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_PDN,	 IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,	 IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},

	{IMGSENSOR_SENSOR_IDX_NONE}
};

#ifdef SENSOR_PLATFORM_4G_20682
struct IMGSENSOR_HW_POWER_SEQ platform_power_sequence_20682[] ={
	{
		PLATFORM_POWER_SEQ_NAME,
		{
			{
				IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,
				IMGSENSOR_HW_PIN_STATE_LEVEL_0,
				0,
				IMGSENSOR_HW_PIN_STATE_LEVEL_0,
				0
			},
		},
		IMGSENSOR_SENSOR_IDX_SUB,
	},
	{
		PLATFORM_POWER_SEQ_NAME,
		{
			{
				IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,
				IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
				0,
				IMGSENSOR_HW_PIN_STATE_LEVEL_0,
				0
			},
		},
		IMGSENSOR_SENSOR_IDX_MAIN2,
	},
};

struct IMGSENSOR_HW_POWER_SEQ platform_power_sequence_19661[] ={
	{
		PLATFORM_POWER_SEQ_NAME,
		{
			{
				IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,
				IMGSENSOR_HW_PIN_STATE_LEVEL_0,
				0,
				IMGSENSOR_HW_PIN_STATE_LEVEL_0,
				0
			},
		},
		IMGSENSOR_SENSOR_IDX_SUB,
	},
	{
		PLATFORM_POWER_SEQ_NAME,
		{
			{
				IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,
				IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
				0,
				IMGSENSOR_HW_PIN_STATE_LEVEL_0,
				0
			},
		},
		IMGSENSOR_SENSOR_IDX_MAIN2,
	},
};
#endif


struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20682[] = {
#if defined(SALAA_QTECH_MAIN_S5KGM1SP)
	{
		SENSOR_DRVNAME_SALAA_QTECH_MAIN_S5KGM1SP,
		{
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_Low, 1},
			{AVDD2, Vol_2800, 0},
			{DVDD, Vol_1050, 0},
			{DOVDD, Vol_1800, 0},
			{AFVDD, Vol_2800, 2},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(IMX682_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX682_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_1800, 0},
			{AVDD2, Vol_2900, 0},
			{DVDD, Vol_1100, 0},
			{DOVDD, Vol_1800, 0},
			{AFVDD, Vol_2800, 2},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(S5KGM1SP_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KGM1SP_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_Low, 1},
			{AVDD2, Vol_2800, 0},
			{DVDD, Vol_1050, 0},
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
#if defined(SALA_WIDE_OV8856)
	{
		SENSOR_DRVNAME_SALA_WIDE_OV8856,
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
#if defined(SALA_OV02B10_MIPI_RAW)
	{
		SENSOR_DRVNAME_SALA_OV02B10_MIPI_RAW,
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
#if defined(OV64B_MIPI_RAW)
    {
        SENSOR_DRVNAME_OV64B_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 1},
            {AVDD2, Vol_2800, 2},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 5},
            {AFVDD, Vol_2800, 2},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(GC02M1B_MIPI_MONO)
    {
        SENSOR_DRVNAME_GC02M1B_MIPI_MONO,
        {
            {PDN, Vol_Low, 1},
            //{DVDD, Vol_1200, 0},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0, Vol_Low, 8},
            {SensorMCLK, Vol_High, 1},
            {PDN, Vol_High, 2}
        },
    },
#endif
#if defined(GC02M1B_MIPI_RAW_20601)
    {
        SENSOR_DRVNAME_GC02M1B_MIPI_RAW_20601,
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
#if defined(S5KGM1ST_MIPI_RAW)
    {
        SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW,
        {
            {SensorMCLK, Vol_High, 0},
            {RST, Vol_Low, 1},
            {AVDD2, Vol_2800, 0},
            {DVDD, Vol_1050, 0},
            {DOVDD, Vol_1800, 0},
            {AFVDD, Vol_2800, 2},
            {RST, Vol_High, 5},
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
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_19661[] = {
#if defined(S5KGW1_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KGW1_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1050, 0},
			{DOVDD, Vol_1800, 0},
			{AFVDD, Vol_2800, 2},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(S5KGM1SP_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KGM1SP_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1050, 0},
			{DOVDD, Vol_1800, 0},
			{AFVDD, Vol_2800, 2},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(S5K3P9SP_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1050, 0},
			{DOVDD, Vol_1800, 3},
			{RST, Vol_High, 5},
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
#if defined(GC2375H_MIPI_RAW_19661)
	{
		SENSOR_DRVNAME_GC2375H_MIPI_RAW_19661,
		{
			{PDN, Vol_High, 2},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_Low, 10, Vol_High, 1},
		},
	},
#endif
#if defined(GC02M0_MIPI_MONO)
	{
		SENSOR_DRVNAME_GC02M0_MIPI_MONO,
		{
			{PDN, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 2},
			{PDN, Vol_High, 3},
		},
	},
#endif
#if defined(GC02K0_MIPI_RAW)
 	{
		SENSOR_DRVNAME_GC02K0_MIPI_RAW,
		{
			{PDN, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 2},
			{PDN, Vol_High, 3},
 		},
 	},
#endif
    /* add new sensor before this line */
    {NULL,},
};
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20730[] = {
#if defined(OV64B_MIPI_RAW_20730)
    {
        SENSOR_DRVNAME_OV64B_MIPI_RAW_20730,
        {
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 1},
            {AVDD, Vol_2800, 2},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 5},
            {AFVDD, Vol_2800, 2},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(IMX471_MIPI_RAW2_20730)
	{
		SENSOR_DRVNAME_IMX471_MIPI_RAW2_20730,
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
#if defined(OV8856_MIPI_RAW2_20730)
	{
		SENSOR_DRVNAME_OV8856_MIPI_RAW2_20730,
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
#if defined(GC02M1B_MIPI_MONO_20730)
    {
        SENSOR_DRVNAME_GC02M1B_MIPI_MONO_20730,
        {
            {PDN, Vol_Low, 1},
            //{DVDD, Vol_1200, 0},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0, Vol_Low, 8},
            {SensorMCLK, Vol_High, 1},
            {PDN, Vol_High, 2}
        },
    },
#endif
#if defined(GC02M1_MIPI_RAW_20730)
    {
        SENSOR_DRVNAME_GC02M1_MIPI_RAW_20730,
        {
            {PDN, Vol_Low, 1},
            //{DVDD, Vol_1800, 0},
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

struct CAMERA_DEVICE_INFO gImgEepromInfo_20682 = {
    .i4SensorNum = 5,
    .pCamModuleInfo = {
//        {IMX682_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r", "imx682"},
        {S5KGM1SP_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r",  "s5kgm1sp"},
        {SALAA_QTECH_MAIN_S5KGM1SP_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r", "salaaqtechmains5kgm1sp"},
        {IMX471_SENSOR_ID1,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f", "imx471"},
        {OV8856_SENSOR_ID, 0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "ov8856"},
        {SALA_OV02B10_SENSOR_ID,  0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "ov02b10"},
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

struct CAMERA_DEVICE_INFO gImgEepromInfo_20730 = {
    .i4SensorNum = 5,
    .pCamModuleInfo = {
        {OV64B_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r",  "ov64b"},
        {IMX471_SENSOR_ID2,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f", "imx471"},
        {OV8856_SENSOR_ID2_20730, 0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "ov8856"},
        {OV02B10_SENSOR_ID,  0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "ov02b10"},
        {OV02B1B_SENSOR_ID, 0xFF, {0xFF, 0xFF}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r3", "ov02b1b"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR_20730,OV8856_STEREO_START_ADDR_20730},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_19661 = {
    .i4SensorNum = 5,
    .pCamModuleInfo = {
        {S5KGW1_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r",  "s5kgw1"},
        {S5K3P9SP_SENSOR_ID,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f", "s5k3p9sp"},
        {OV8856_SENSOR_ID, 0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "ov8856"},
        {GC2375H_SENSOR_ID_19661,  0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc2375h"},
        {GC02M0_SENSOR_ID, 0xFF, {0xFF, 0xFF}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r3", "gc02m0"},
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
    if (is_project(20682) || is_project(20683)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20682;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20682;
         oplus_sensor_power_sequence = sensor_power_sequence_20682;
#ifdef SENSOR_PLATFORM_4G_20682
         oplus_platform_power_sequence = platform_power_sequence_20682;
#endif
         gImgEepromInfo = gImgEepromInfo_20682;
    }
    else if(is_project(19661)){
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_19661;
         oplus_imgsensor_custom_config = imgsensor_custom_config_19661;
         oplus_sensor_power_sequence = sensor_power_sequence_19661;
#ifdef SENSOR_PLATFORM_4G_20682
         oplus_platform_power_sequence = platform_power_sequence_19661;
#endif
         gImgEepromInfo = gImgEepromInfo_19661;
    }
	else if(is_project(20730) || is_project(20731) || is_project(20732))
	{
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20730;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20730;
         oplus_sensor_power_sequence = sensor_power_sequence_20730;
//#ifdef SENSOR_PLATFORM_4G_20730
         oplus_platform_power_sequence = platform_power_sequence_20682;
//#endif
         gImgEepromInfo = gImgEepromInfo_20730;
	}
	else {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20682;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20682;
         oplus_sensor_power_sequence = sensor_power_sequence_20682;
#ifdef SENSOR_PLATFORM_4G_20682
         oplus_platform_power_sequence = platform_power_sequence_20682;
#endif
         gImgEepromInfo = gImgEepromInfo_20682;
    }
}
#endif
