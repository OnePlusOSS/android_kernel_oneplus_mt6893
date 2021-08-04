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
#ifndef __IMGSENSOR_HWCFG_20075_CTRL_H__
#define __IMGSENSOR_HWCFG_20075_CTRL_H__
#include "imgsensor_hwcfg_custom.h"

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20075[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV48B_MIPI_RAW)
{OV48B_SENSOR_ID, SENSOR_DRVNAME_OV48B_MIPI_RAW, OV48B_MIPI_RAW_SensorInit},
#endif
#if defined(OV32A_MIPI_RAW)
{OV32A_SENSOR_ID, SENSOR_DRVNAME_OV32A_MIPI_RAW, OV32A_MIPI_RAW_SensorInit},
#endif
#if defined(HI846_MIPI_RAW)
{HI846_SENSOR_ID, SENSOR_DRVNAME_HI846_MIPI_RAW, HI846_MIPI_RAW_SensorInit},
#endif
#if defined(GC02K0_MIPI_RAW)
{GC02K0_SENSOR_ID, SENSOR_DRVNAME_GC02K0_MIPI_RAW, GC02K0_MIPI_RAW_SensorInit},
#endif

    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20075[] = {
    {
            IMGSENSOR_SENSOR_IDX_MAIN,
            IMGSENSOR_I2C_DEV_0,
            {
                    {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
                    {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
                    {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
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
                    {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
            },
    },
    {
            IMGSENSOR_SENSOR_IDX_MAIN2,
            IMGSENSOR_I2C_DEV_2,
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
            IMGSENSOR_I2C_DEV_3,
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
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20075[] = {
#if defined(OV48B_MIPI_RAW)
    {
        SENSOR_DRVNAME_OV48B_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 0},
            {DOVDD, Vol_1800, 0},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1200, 5},
            // {AFVDD, Vol_2800, 2},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(OV32A_MIPI_RAW)
    {
        SENSOR_DRVNAME_OV32A_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 5},
            {AVDD, Vol_2800, 1},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 1},
            {RST, Vol_High, 5}
        },
    },
#endif
#if defined(HI846_MIPI_RAW)
    {
        SENSOR_DRVNAME_HI846_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1200, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2}
        },
    },
#endif
#if defined(GC02K0_MIPI_RAW)
    {
        SENSOR_DRVNAME_GC02K0_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 5},
            {AVDD, Vol_2800, 5},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 1},
        },
    },
#endif

    /* add new sensor before this line */
    {NULL,},
};

#ifdef SENSOR_PLATFORM_5G_B
struct CAMERA_DEVICE_INFO gImgEepromInfo= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV48B_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x98,0x94}, "Cam_r0", "ov48b2q"},
        {OV32A_SENSOR_ID,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "ov32a1q"},
        {HI846_SENSOR_ID,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "hi846"},
        {GC02K0_SENSOR_ID, 0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02k0"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {OV48B_STEREO_START_ADDR, HI846_STEREO_START_ADDR },
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};
#endif

#endif
