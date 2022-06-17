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
#ifndef __IMGSENSOR_HWCFG_mt6873_CTRL_H__
#define __IMGSENSOR_HWCFG_mt6873_CTRL_H__
#include "imgsensor_hwcfg_custom.h"


struct IMGSENSOR_SENSOR_LIST *oplus_gimgsensor_sensor_list = NULL;
struct IMGSENSOR_HW_CFG *oplus_imgsensor_custom_config = NULL;
struct IMGSENSOR_HW_POWER_SEQ *oplus_sensor_power_sequence = NULL;
struct IMGSENSOR_HW_POWER_SEQ *oplus_platform_power_sequence = NULL;
struct CAMERA_DEVICE_INFO gImgEepromInfo;

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_19131[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV48B_MIPI_RAW)
{OV48B_SENSOR_ID, SENSOR_DRVNAME_OV48B_MIPI_RAW, OV48B_MIPI_RAW_SensorInit},
#endif
#if defined(S5K3P9SP_MIPI_RAW)
{S5K3P9SP_SENSOR_ID, SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW, S5K3P9SP_MIPI_RAW_SensorInit},
#endif
#if defined(GC8054_MIPI_RAW)
{GC8054_SENSOR_ID, SENSOR_DRVNAME_GC8054_MIPI_RAW, GC8054_MIPI_RAW_SensorInit},
#endif
#if defined(GC02K0B_MIPI_MONO)
{GC02K0_SENSOR_ID, SENSOR_DRVNAME_GC02K0B_MIPI_MONO, GC02K0_MIPI_MONO_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO1)
{GC02M1B_SENSOR_ID1, SENSOR_DRVNAME_GC02M1B_MIPI_MONO1, GC02M1B_MIPI_MONO1_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO)
{GC02M1B_SENSOR_ID, SENSOR_DRVNAME_GC02M1B_MIPI_MONO, GC02M1B_MIPI_MONO_SensorInit},
#endif
#if defined(HI846_MIPI_RAW)
{HI846_SENSOR_ID, SENSOR_DRVNAME_HI846_MIPI_RAW, HI846_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M0B_MIPI_MONO)
{GC02M0_SENSOR_ID, SENSOR_DRVNAME_GC02M0B_MIPI_MONO, GC02M0_MIPI_MONO_SensorInit},
#endif
#if defined(GC02M0B_MIPI_MONO1)
{GC02M0_SENSOR_ID1, SENSOR_DRVNAME_GC02M0B_MIPI_MONO1, GC02M0_MIPI_MONO1_SensorInit},
#endif
#if defined(GC02M0B_MIPI_MONO2)
{GC02M0_SENSOR_ID2, SENSOR_DRVNAME_GC02M0B_MIPI_MONO2, GC02M0_MIPI_MONO2_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_19131[] = {
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
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_MAIN3,
        IMGSENSOR_I2C_DEV_4,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
        //  {IMGSENSOR_HW_PIN_MIPI_SWITCH_EN, IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_SUB3,
        IMGSENSOR_I2C_DEV_5,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
        //  {IMGSENSOR_HW_PIN_MIPI_SWITCH_EN, IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ platform_power_sequence_19131[] = {
    {
        PLATFORM_POWER_SEQ_NAME,
        {
            /*
            {
                IMGSENSOR_HW_PIN_MIPI_SWITCH_EN,
                IMGSENSOR_HW_PIN_STATE_LEVEL_0,
                0,
                IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
                0
            },
            */
            {
                IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,
                IMGSENSOR_HW_PIN_STATE_LEVEL_0,
                0,
                IMGSENSOR_HW_PIN_STATE_LEVEL_0,
                0
            },
        },
        IMGSENSOR_SENSOR_IDX_SUB3,
    },
    {
        PLATFORM_POWER_SEQ_NAME,
        {
            /*
            {
                IMGSENSOR_HW_PIN_MIPI_SWITCH_EN,
                IMGSENSOR_HW_PIN_STATE_LEVEL_0,
                0,
                IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
                0
            },
            */
            {
                IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,
                IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
                0,
                IMGSENSOR_HW_PIN_STATE_LEVEL_0,
                0
            },
        },
        IMGSENSOR_SENSOR_IDX_MAIN3,
    },
    {NULL}
};
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_19131[] = {
#if defined(OV48B_MIPI_RAW)
        {
                SENSOR_DRVNAME_OV48B_MIPI_RAW,
                {
                        {RST, Vol_Low, 1},
                        {SensorMCLK, Vol_High, 0},
                        {DOVDD, Vol_1800, 0},
                        {AVDD, Vol_2800, 0},
                        {DVDD, Vol_1200, 5},
                        //{AFVDD, Vol_2800, 2},
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
                        {DVDD, Vol_1100, 1},
                        {AVDD, Vol_2800, 1},
                        {DOVDD, Vol_1800, 1},
                        //{AFVDD, Vol_2800, 5},
                        {RST, Vol_High, 2},
                },
        },
#endif
#if defined(GC8054_MIPI_RAW)
        {
                SENSOR_DRVNAME_GC8054_MIPI_RAW,
                {
                        {SensorMCLK, Vol_High, 0},
                        {RST, Vol_Low,  1},
                        {DOVDD, Vol_1800, 1},
                        {DVDD, Vol_1200, 1},
                        {AVDD, Vol_2800, 1},
                        {RST, Vol_High, 1},
                        //{AFVDD, Vol_Low, 5}
                },
        },
#endif
#if defined(HI846_MIPI_RAW)
                {
                        SENSOR_DRVNAME_HI846_MIPI_RAW,
                        {
                                {RST, Vol_Low, 1},
                                {DOVDD, Vol_1800, 1},
                                {AVDD, Vol_2800, 0},
                                {DVDD, Vol_1200, 1},
                                {SensorMCLK, Vol_High, 1},
                                {RST, Vol_High, 2}
                        },
                },
#endif
#if defined(GC02M0B_MIPI_MONO)
        {
                SENSOR_DRVNAME_GC02M0B_MIPI_MONO,
                {
                        {RST, Vol_Low, 1},
                        {DOVDD, Vol_1800, 0},
                        {DVDD, Vol_1200, 1},
                        {AVDD, Vol_2800, 0},
                        {SensorMCLK, Vol_High, 1},
                        {RST, Vol_High, 2}
                },
        },
#endif
#if defined(GC02M0B_MIPI_MONO1)
        {
                SENSOR_DRVNAME_GC02M0B_MIPI_MONO1,
                {
                        {RST, Vol_Low, 1},
                        //{DVDD, Vol_1200, 0},
                        {DOVDD, Vol_1800, 1},
                        {AVDD, Vol_2800, 0},
                        {SensorMCLK, Vol_High, 1},
                        {RST, Vol_High, 2}
                },
        },
#endif
#if defined(GC02M0B_MIPI_MONO2)
        {
                SENSOR_DRVNAME_GC02M0B_MIPI_MONO2,
                {
                        {RST, Vol_Low, 1},
                        //{DVDD, Vol_1200, 0},
                        {DOVDD, Vol_1800, 1},
                        {AVDD, Vol_2800, 0},
                        {SensorMCLK, Vol_High, 1},
                        {RST, Vol_High, 2}
                },
        },
#endif
#if defined(GC02K0B_MIPI_MONO)
        {
                SENSOR_DRVNAME_GC02K0B_MIPI_MONO,
                {
                        {RST, Vol_Low, 1},
                        //{DVDD, Vol_1200, 0},
                        {DOVDD, Vol_1800, 1},
                        {AVDD, Vol_2800, 0},
                        {SensorMCLK, Vol_High, 1},
                        {RST, Vol_High, 2}
                },
        },
#endif
#if defined(GC02M1B_MIPI_MONO)
        {
                SENSOR_DRVNAME_GC02M1B_MIPI_MONO,
                {
                        {RST, Vol_Low, 1},
                        //{DVDD, Vol_1200, 0},
                        {DOVDD, Vol_1800, 1},
                        {AVDD, Vol_2800, 0, Vol_Low, 8},
                        {SensorMCLK, Vol_High, 1},
                        {RST, Vol_High, 2}
                },
        },
#endif
#if defined(GC02M1B_MIPI_MONO1)
        {
                SENSOR_DRVNAME_GC02M1B_MIPI_MONO1,
                {
                        {RST, Vol_Low, 1},
                        //{DVDD, Vol_1200, 0},
                        {DOVDD, Vol_1800, 1},
                        {AVDD, Vol_2800, 0, Vol_Low, 8},
                        {SensorMCLK, Vol_High, 1},
                        {RST, Vol_High, 2}
                },
        },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_19131 = {
    .i4SensorNum = 6,
    .pCamModuleInfo = {
        {OV48B_SENSOR_ID,     0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x98,0x94}, "Cam_r0", "ov48b2q"},
        {S5K3P9SP_SENSOR_ID,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "s5k3p9sp"},
        {HI846_SENSOR_ID,   0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "hi846"},
        {GC02M0_SENSOR_ID,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f1", "gc02m0"},
        {GC02M0_SENSOR_ID1, 0xFF, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02m0"},
        {GC02M0_SENSOR_ID2, 0xFF, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r3", "gc02m0"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = IMGSENSOR_SENSOR_IDX_SUB2,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {OV48B_STEREO_START_ADDR, HI846_STEREO_START_ADDR_19131},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};


void oplus_imgsensor_hwcfg()
{
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_19131;
         oplus_imgsensor_custom_config = imgsensor_custom_config_19131;
         oplus_sensor_power_sequence = sensor_power_sequence_19131;
         oplus_platform_power_sequence = platform_power_sequence_19131;
         gImgEepromInfo = gImgEepromInfo_19131;
}
#endif
