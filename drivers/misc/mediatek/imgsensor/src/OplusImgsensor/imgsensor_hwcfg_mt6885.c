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
#ifndef __IMGSENSOR_HWCFG_20131_CTRL_H__
#define __IMGSENSOR_HWCFG_20131_CTRL_H__
#include "imgsensor_hwcfg_custom.h"

#define PCB_VERSION_EVB  (3)
#define PCB_VERSION_EVB2 (2)
#define PCB_VERSION_T0   (2)
#define PCB_VERSION_T1   (1)

struct IMGSENSOR_SENSOR_LIST *oplus_gimgsensor_sensor_list = NULL;
struct IMGSENSOR_HW_CFG *oplus_imgsensor_custom_config = NULL;
struct IMGSENSOR_HW_POWER_SEQ *oplus_sensor_power_sequence = NULL;
struct CAMERA_DEVICE_INFO gImgEepromInfo;

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20131[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV64B_MIPI_RAW)
{OV64B_SENSOR_ID, SENSOR_DRVNAME_OV64B_MIPI_RAW, OV64B_MIPI_RAW_SensorInit},
#endif
#if defined(IMX616_MIPI_RAW)
{IMX616_SENSOR_ID, SENSOR_DRVNAME_IMX616_MIPI_RAW, IMX616_MIPI_RAW_SensorInit},
#endif
#if defined(IMX615_MIPI_RAW)
{IMX615_SENSOR_ID, SENSOR_DRVNAME_IMX615_MIPI_RAW, IMX615_MIPI_RAW_SensorInit},
#endif
#if defined(IMX319_MIPI_RAW)
{IMX319_SENSOR_ID, SENSOR_DRVNAME_IMX319_MIPI_RAW, IMX319_MIPI_RAW_SensorInit},
#endif
#if defined(OV02B10_MIPI_RAW)
{OV02B10_SENSOR_ID, SENSOR_DRVNAME_OV02B10_MIPI_RAW, OV02B10_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO)
{GC02M1B_SENSOR_ID, SENSOR_DRVNAME_GC02M1B_MIPI_MONO, GC02M1B_MIPI_MONO_SensorInit},
#endif

    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_21015[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(IMX766_MIPI_RAW21015)
{IMX766_SENSOR_ID_21015, SENSOR_DRVNAME_IMX766_MIPI_RAW_21015, IMX766_MIPI_RAW_21015_SensorInit},
#endif
#if defined(IMX709_MIPI_RAW21015)
{IMX709_SENSOR_ID_21015, SENSOR_DRVNAME_IMX709_MIPI_RAW_21015, IMX709_MIPI_RAW_21015_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW21015)
{IMX355_SENSOR_ID_21015, SENSOR_DRVNAME_IMX355_MIPI_RAW_21015, IMX355_MIPI_RAW_21015_SensorInit},
#endif
#if defined(OV02B10_MIPI_RAW21015)
{OV02B10_SENSOR_ID_21015, SENSOR_DRVNAME_OV02B10_MIPI_RAW_21015, OV02B10_MIPI_RAW_21015_SensorInit},
#endif
#if defined(IMX709_MIPI_MONO21015)
{IMX709_MONO_SENSOR_ID_21015, SENSOR_DRVNAME_IMX709_MIPI_MONO_21015, IMX709_MIPI_MONO_21015_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20131[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_SUB2,
        IMGSENSOR_I2C_DEV_2,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20131[] = {
#if defined(OV64B_MIPI_RAW)
    {
        SENSOR_DRVNAME_OV64B_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 1},
            {AVDD, Vol_2800, 2},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 5},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(IMX616_MIPI_RAW)
    {
        SENSOR_DRVNAME_IMX616_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2900, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(IMX615_MIPI_RAW)
    {
        SENSOR_DRVNAME_IMX615_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2900, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(IMX319_MIPI_RAW)
    {
        SENSOR_DRVNAME_IMX319_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(OV02B10_MIPI_RAW)
    {
        SENSOR_DRVNAME_OV02B10_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 4, Vol_Low, 8},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 10}
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
    /* add new sensor before this line */
    {NULL,},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_20131 = {
    .i4SensorNum = 5,
    .pCamModuleInfo = {
        {OV64B_SENSOR_ID,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "ov64b2q"},
        {IMX616_SENSOR_ID,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx616"},
        {IMX319_SENSOR_ID,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "imx319"},
        {OV02B10_SENSOR_ID, 0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "ov02b10"},
        {GC02M1B_SENSOR_ID, 0xFF, {0xFF, 0xFF}, 0xFF, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r3", "gc02m1b"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR, IMX319_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};
struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20171[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV64B_MIPI_RAW)
{OV64B_SENSOR_ID, SENSOR_DRVNAME_OV64B_MIPI_RAW, OV64B_MIPI_RAW_SensorInit},
#endif
#if defined(IMX616_MIPI_RAW)
{IMX616_SENSOR_ID, SENSOR_DRVNAME_IMX616_MIPI_RAW, IMX616_MIPI_RAW_SensorInit},
#endif
#if defined(IMX615_MIPI_RAW)
{IMX615_SENSOR_ID, SENSOR_DRVNAME_IMX615_MIPI_RAW, IMX615_MIPI_RAW_SensorInit},
#endif
#if defined(IMX319_MIPI_RAW)
{IMX319_SENSOR_ID, SENSOR_DRVNAME_IMX319_MIPI_RAW, IMX319_MIPI_RAW_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW)
{IMX355_SENSOR_ID, SENSOR_DRVNAME_IMX355_MIPI_RAW, IMX355_MIPI_RAW_SensorInit},
#endif
#if defined(OV02B10_MIPI_RAW)
{OV02B10_SENSOR_ID, SENSOR_DRVNAME_OV02B10_MIPI_RAW, OV02B10_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO)
{GC02M1B_SENSOR_ID, SENSOR_DRVNAME_GC02M1B_MIPI_MONO, GC02M1B_MIPI_MONO_SensorInit},
#endif

    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20171[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_MAIN3,
        IMGSENSOR_I2C_DEV_4,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20171[] = {
#if defined(OV64B_MIPI_RAW)
    {
        SENSOR_DRVNAME_OV64B_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 1},
            {AVDD, Vol_2800, 2},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 5},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(IMX616_MIPI_RAW)
    {
        SENSOR_DRVNAME_IMX616_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2900, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(IMX615_MIPI_RAW)
    {
        SENSOR_DRVNAME_IMX615_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2900, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(IMX319_MIPI_RAW)
    {
        SENSOR_DRVNAME_IMX319_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(IMX355_MIPI_RAW)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1200, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(OV02B10_MIPI_RAW)
    {
        SENSOR_DRVNAME_OV02B10_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0, Vol_Low, 8},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 10}
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
    /* add new sensor before this line */
    {NULL,},
};

#ifdef SENSOR_PLATFORM_5G_H
struct CAMERA_DEVICE_INFO gImgEepromInfo_20171= {
    .i4SensorNum = 5,
    .pCamModuleInfo = {
        {OV64B_SENSOR_ID,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "ov64b2q"},
        {IMX615_SENSOR_ID,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx615"},
        /*{IMX319_SENSOR_ID,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "imx319"},*/
        {IMX355_SENSOR_ID,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "imx355"},
        {OV02B10_SENSOR_ID, 0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "ov02b10"},
        {GC02M1B_SENSOR_ID, 0xFF, {0xFF, 0xFF}, 0xFF, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r3", "gc02m1b"},
    },

    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = DUALCAM_CALI_DATA_LENGTH_TOTAL_QCOM,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR, IMX355_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
    .i4DistortionAddr = IMX355_DISTORTIONPARAMS_START_ADDR,
};
#endif
struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20615[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(IMX682_MIPI_RAW_20615)
{IMX682_SENSOR_ID_20615, SENSOR_DRVNAME_IMX682_MIPI_RAW_20615, IMX682_MIPI_RAW_20615_SensorInit},
#endif
#if defined(IMX471_MIPI_RAW_20615)
{IMX471_SENSOR_ID_20615, SENSOR_DRVNAME_IMX471_MIPI_RAW_20615, IMX471_MIPI_RAW_20615_SensorInit},
#endif
#if defined(HI846_MIPI_RAW_20615)
{HI846_SENSOR_ID_20615, SENSOR_DRVNAME_HI846_MIPI_RAW_20615, HI846_MIPI_RAW_20615_SensorInit},
#endif
#if defined(OV02B10_MIPI_RAW_20615)
{OV02B10_SENSOR_ID_20615, SENSOR_DRVNAME_OV02B10_MIPI_RAW_20615, OV02B10_MIPI_RAW_20615_SensorInit},
#endif

    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20615[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD_1,IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20615[] = {
#if defined(IMX682_MIPI_RAW_20615)
    {
        SENSOR_DRVNAME_IMX682_MIPI_RAW_20615,
        {
            {RST, Vol_Low, 1},
            {AVDD_1, Vol_1800, 0},
            {AVDD, Vol_2900, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 2},
            //{AFVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 3}
        },
    },
#endif
#if defined(IMX471_MIPI_RAW_20615)
    {
        SENSOR_DRVNAME_IMX471_MIPI_RAW_20615,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2}
        },
    },
#endif
#if defined(HI846_MIPI_RAW_20615)
      {
          SENSOR_DRVNAME_HI846_MIPI_RAW_20615,
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
#if defined(OV02B10_MIPI_RAW_20615)
    {
        SENSOR_DRVNAME_OV02B10_MIPI_RAW_20615,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0, Vol_Low, 8},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 10}
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_21651[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV64B_MIPI_RAW_21651)
{OV64B_SENSOR_ID_21651, SENSOR_DRVNAME_OV64B_MIPI_RAW_21651, OV64B_MIPI_RAW_21651_SensorInit},
#endif
#if defined(IMX471_MIPI_RAW_21651)
{IMX471_SENSOR_ID_21651, SENSOR_DRVNAME_IMX471_MIPI_RAW_21651, IMX471_MIPI_RAW_21651_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW_21651)
{IMX355_SENSOR_ID_21651, SENSOR_DRVNAME_IMX355_MIPI_RAW_21651, IMX355_MIPI_RAW_21651_SensorInit},
#endif
#if defined(GC02M1_MIPI_RAW_21651)
{GC02M1_SENSOR_ID_21651, SENSOR_DRVNAME_GC02M1_MIPI_RAW_21651, GC02M1_MIPI_RAW_21651_SensorInit},
#endif

    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_21651[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD_1,IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_21651[] = {
#if defined(OV64B_MIPI_RAW_21651)
    {
        SENSOR_DRVNAME_OV64B_MIPI_RAW_21651,
        {
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 1},
            {AVDD, Vol_2800, 2},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 5},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(IMX471_MIPI_RAW_21651)
    {
        SENSOR_DRVNAME_IMX471_MIPI_RAW_21651,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2}
        },
    },
#endif
#if defined(IMX355_MIPI_RAW_21651)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW_21651,
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
#if defined(GC02M1_MIPI_RAW_21651)
    {
        SENSOR_DRVNAME_GC02M1_MIPI_RAW_21651,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0, Vol_Low, 8},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 10}
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

#ifdef SENSOR_PLATFORM_5G_H
struct CAMERA_DEVICE_INFO gImgEepromInfo_20615= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {IMX682_SENSOR_ID_20615, 0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "imx682"},
        {IMX471_SENSOR_ID_20615, 0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx471"},
        {HI846_SENSOR_ID_20615,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "hi846"},
        {OV02B10_SENSOR_ID_20615,0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "ov02b10"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {IMX682_STEREO_START_ADDR_20615, HI846_STEREO_START_ADDR_20615},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_21651= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV64B_SENSOR_ID_21651, 0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "imx682"},
        {IMX471_SENSOR_ID_21651, 0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx471"},
        {IMX355_SENSOR_ID_21651,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "hi846"},
        {GC02M1_SENSOR_ID_21651,0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "ov02b10"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {IMX682_STEREO_START_ADDR_20615, HI846_STEREO_START_ADDR_20615},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};
#endif

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20619[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV64B_MIPI_RAW_20619)
{OV64B_SENSOR_ID_20619, SENSOR_DRVNAME_OV64B_MIPI_RAW_20619, OV64B_MIPI_RAW_20619_SensorInit},
#endif
#if defined(IMX471_MIPI_RAW_20619)
{IMX471_SENSOR_ID_20619, SENSOR_DRVNAME_IMX471_MIPI_RAW_20619, IMX471_MIPI_RAW_20619_SensorInit},
#endif
#if defined(HI846_MIPI_RAW_20619)
{HI846_SENSOR_ID_20619, SENSOR_DRVNAME_HI846_MIPI_RAW_20619, HI846_MIPI_RAW_20619_SensorInit},
#endif
#if defined(OV02B10_MIPI_RAW_20619)
{OV02B10_SENSOR_ID_20619, SENSOR_DRVNAME_OV02B10_MIPI_RAW_20619, OV02B10_MIPI_RAW_20619_SensorInit},
#endif

    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20619[] = {
#if defined(OV64B_MIPI_RAW_20619)
    {
        SENSOR_DRVNAME_OV64B_MIPI_RAW_20619,
        {
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 1},
            {AVDD, Vol_2800, 2},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 5},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(IMX471_MIPI_RAW_20619)
    {
        SENSOR_DRVNAME_IMX471_MIPI_RAW_20619,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2}
        },
    },
#endif
#if defined(HI846_MIPI_RAW_20619)
    {
        SENSOR_DRVNAME_HI846_MIPI_RAW_20619,
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
#if defined(OV02B10_MIPI_RAW_20619)
    {
        SENSOR_DRVNAME_OV02B10_MIPI_RAW_20619,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0, Vol_Low, 8},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 10}
        },
    },
#endif

    /* add new sensor before this line */
    {NULL,},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_20619= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV64B_SENSOR_ID_20619, 0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "ov64b"},
        {IMX471_SENSOR_ID_20619, 0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx471"},
        {HI846_SENSOR_ID_20619,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "hi846"},
        {OV02B10_SENSOR_ID_20619,0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "ov02b10"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR, HI846_STEREO_START_ADDR_20615},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_21061[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV64B_MIPI_RAW)
{OV64B_SENSOR_ID, SENSOR_DRVNAME_OV64B_MIPI_RAW, OV64B_MIPI_RAW_SensorInit},
#endif
#if defined(IMX471_MIPI_RAW)
{IMX471_SENSOR_ID, SENSOR_DRVNAME_IMX471_MIPI_RAW, IMX471_MIPI_RAW_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW)
{IMX355_SENSOR_ID, SENSOR_DRVNAME_IMX355_MIPI_RAW, IMX355_MIPI_RAW_SensorInit},
#endif
#if defined(OV02B10_MIPI_RAW)
{OV02B10_SENSOR_ID, SENSOR_DRVNAME_OV02B10_MIPI_RAW, OV02B10_MIPI_RAW_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_21061[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_21015[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD_1,IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD_1,IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_PDN,  IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_21061[] = {
#if defined(OV64B_MIPI_RAW)
    {
        SENSOR_DRVNAME_OV64B_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 1},
            {AVDD, Vol_2800, 2},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 5},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(IMX471_MIPI_RAW)
    {
        SENSOR_DRVNAME_IMX471_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2900, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2}
        },
    },
#endif
#if defined(IMX355_MIPI_RAW)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1200, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(OV02B10_MIPI_RAW)
    {
        SENSOR_DRVNAME_OV02B10_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 5, Vol_Low, 5},
            {RST, Vol_High, 5, Vol_Low, 5},
            {SensorMCLK, Vol_High, 5, Vol_Low, 5},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

#ifdef SENSOR_PLATFORM_5G_H
struct CAMERA_DEVICE_INFO gImgEepromInfo_21061= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV64B_SENSOR_ID,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "ov64b2q"},
        {IMX471_SENSOR_ID,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx471"},
        {IMX355_SENSOR_ID,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "imx355"},
        {OV02B10_SENSOR_ID, 0xA4, {0x00, 0x06}, 0xE0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "ov02b10"},
    },

    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = DUALCAM_CALI_DATA_LENGTH_TOTAL_QCOM,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR, IMX355_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
    .i4DistortionAddr = IMX355_DISTORTIONPARAMS_START_ADDR,
};
#endif

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20645[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV64B_MIPI_RAW_20645)
    {OV64B_SENSOR_ID_20645, SENSOR_DRVNAME_OV64B_MIPI_RAW_20645, OV64B_MIPI_RAW_20645_SensorInit},
#endif
#if defined(OV32A1Q_MIPI_RAW_20645)
    {OV32A1Q_SENSOR_ID_20645, SENSOR_DRVNAME_OV32A1Q_MIPI_RAW_20645, OV32A1Q_MIPI_RAW_20645_SensorInit},
#endif
#if defined(HI846_MIPI_RAW_20645)
    {HI846_SENSOR_ID_20645, SENSOR_DRVNAME_HI846_MIPI_RAW_20645, HI846_MIPI_RAW_20645_SensorInit},
#endif
#if defined(OV02B10_MIPI_RAW_20645)
    {OV02B10_SENSOR_ID_20645, SENSOR_DRVNAME_OV02B10_MIPI_RAW_20645, OV02B10_MIPI_RAW_20645_SensorInit},
#endif
    {0, {0}, NULL}, /* end of list */
};
struct IMGSENSOR_SENSOR_LIST
gimgsensor_sensor_list_21127[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV50A_MIPI_RAW_21127)
	{OV50A_SENSOR_ID_21127, SENSOR_DRVNAME_OV50A_MIPI_RAW_21127, OV50A_MIPI_RAW_21127_SensorInit},
#endif
//#if defined(IMX471_MIPI_RAW_21127)
//	{IMX471_SENSOR_ID_21127, SENSOR_DRVNAME_IMX471_MIPI_RAW_21127, IMX471_MIPI_RAW_21127_SensorInit},
//#endif
#if defined(IMX709_MIPI_RAW_21127)
	{IMX709_SENSOR_ID_21127, SENSOR_DRVNAME_IMX709_MIPI_RAW_21127, IMX709_MIPI_RAW_21127_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO_21127)
	{GC02M1B_SENSOR_ID_21127, SENSOR_DRVNAME_GC02M1B_MIPI_MONO_21127, GC02M1B_MIPI_MONO_21127_SensorInit},
#endif
#if defined(GC02M1_MIPI_RAW_21127)
	{GC02M1_SENSOR_ID_21127, SENSOR_DRVNAME_GC02M1_MIPI_RAW_21127, GC02M1_MIPI_RAW_21127_SensorInit},
#endif
    {0, {0}, NULL}, /* end of list */
};
struct IMGSENSOR_SENSOR_LIST
gimgsensor_sensor_list_21305[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(IMX766_MIPI_RAW_21305)
	{IMX766_SENSOR_ID_21305, SENSOR_DRVNAME_IMX766_MIPI_RAW_21305, IMX766_MIPI_RAW_21305_SensorInit},
#endif
//#if defined(IMX615_MIPI_RAW_21305)
//	{IMX615_SENSOR_ID_21305, SENSOR_DRVNAME_IMX615_MIPI_RAW_21305, IMX615_MIPI_RAW_21305_SensorInit},
//#endif
#if defined(IMX709_MIPI_RAW_21305)
	{IMX709_SENSOR_ID_21305, SENSOR_DRVNAME_IMX709_MIPI_RAW_21305, IMX709_MIPI_RAW_21305_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW_21305)
	{IMX355_SENSOR_ID_21305, SENSOR_DRVNAME_IMX355_MIPI_RAW_21305, IMX355_MIPI_RAW_21305_SensorInit},
#endif
#if defined(GC02M1_MIPI_RAW_21305)
	{GC02M1_SENSOR_ID_21305, SENSOR_DRVNAME_GC02M1_MIPI_RAW_21305, GC02M1_MIPI_RAW_21305_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_21127[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_AVDD_1, IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_21305[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD_1,IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_AVDD_1, IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20645[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_SUB2,
        IMGSENSOR_I2C_DEV_2,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};


struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20645[] = {
#if defined(OV64B_MIPI_RAW_20645)
    {
        SENSOR_DRVNAME_OV64B_MIPI_RAW_20645,
        {
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 1},
            {AVDD, Vol_2800, 2},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 5},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(OV32A1Q_MIPI_RAW_20645)
    {
        SENSOR_DRVNAME_OV32A1Q_MIPI_RAW_20645,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {RST, Vol_High, 2},
            {SensorMCLK, Vol_High, 1},
        },
    },
#endif
#if defined(HI846_MIPI_RAW_20645)
    {
        SENSOR_DRVNAME_HI846_MIPI_RAW_20645,
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
#if defined(OV02B10_MIPI_RAW_20645)
    {
        SENSOR_DRVNAME_OV02B10_MIPI_RAW_20645,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 4, Vol_Low, 8},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 10}
        },
    },
#endif

    /* add new sensor before this line */
    {NULL,},


};


struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_21015[] = {
#if defined(IMX766_MIPI_RAW21015)
    {
        SENSOR_DRVNAME_IMX766_MIPI_RAW_21015,
        {
            {RST, Vol_Low, 1},
            {AVDD_1, Vol_2800, 4},
            {AVDD, Vol_High, 2},
            {DVDD, Vol_High, 2},
            {DVDD_1, Vol_High, 4},
            {DOVDD, Vol_1800, 3},
            {SensorMCLK, Vol_High, 6},
            {AFVDD_21015, Vol_2800, 1},
            {RST, Vol_High, 3},
        },
    },
#endif
#if defined(IMX709_MIPI_RAW21015)
    {
        SENSOR_DRVNAME_IMX709_MIPI_RAW_21015,
        {
            {SensorMCLK, Vol_High, 1},
            {PDN, Vol_Low, 0},
            {RST, Vol_Low, 1},
            {AVDD, Vol_2900, 0},
            {DVDD, Vol_1000, 0},
            {DOVDD, Vol_1800, 0},
            {PDN, Vol_High, 1},
            {RST, Vol_High, 8},
        },
    },
#endif
#if defined(IMX355_MIPI_RAW21015)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW_21015,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1200, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(OV02B10_MIPI_RAW21015)
    {
        SENSOR_DRVNAME_OV02B10_MIPI_RAW_21015,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0, Vol_Low, 8},
            {SensorMCLK, Vol_High, 7},
            {RST, Vol_High, 10}
        },
    },
#endif

    {NULL,},
};
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_21127[] = {
#if defined(OV50A_MIPI_RAW_21127)
    {
        SENSOR_DRVNAME_OV50A_MIPI_RAW_21127,
        {
//            {PDN, Vol_High, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 1},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 1},
            {AFVDD, Vol_2800, 1},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(IMX471_MIPI_RAW_21127)
    {
        SENSOR_DRVNAME_IMX471_MIPI_RAW_21127,
        {
//            {PDN, Vol_High, 1},
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2}
        },
    },
#endif
#if defined(IMX709_MIPI_RAW_21127)
    {
        SENSOR_DRVNAME_IMX709_MIPI_RAW_21127,
        {
//            {PDN, Vol_Low, 1},      //FAN53870 Power supply
            {SensorMCLK, Vol_High, 1},
            {AVDD_1, Vol_Low, 0},   //PONV
            {RST, Vol_Low, 1},
            {AVDD, Vol_2900, 0},
            {DVDD, Vol_1000, 0},
            {DOVDD, Vol_1800, 0},
            {AVDD_1, Vol_High, 1},
            {RST, Vol_High, 8},
        },
    },
#endif
#if defined(GC02M1B_MIPI_MONO_21127)
    {
        SENSOR_DRVNAME_GC02M1B_MIPI_MONO_21127,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0, Vol_Low, 8},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2}
        },
    },
#endif
#if defined(GC02M1_MIPI_RAW_21127)
    {
        SENSOR_DRVNAME_GC02M1_MIPI_RAW_21127,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0, Vol_Low, 8},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 10}
        },
    },
#endif

    {NULL,},
};
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_21305[] = {
#if defined(IMX766_MIPI_RAW_21305)
    {
        SENSOR_DRVNAME_IMX766_MIPI_RAW_21305,
        {
//            {PDN, Vol_High, 1},
            {RST, Vol_Low, 1},
            {AVDD_1, Vol_1800, 4},
            {AVDD, Vol_2800, 2},
            {DVDD, Vol_1100, 2},
            {DOVDD, Vol_1800, 3},
            {SensorMCLK, Vol_High, 6},
            {AFVDD, Vol_2800, 1},
            {RST, Vol_High, 3},
        },
    },
#endif
#if defined(IMX615_MIPI_RAW_21305)
    {
        SENSOR_DRVNAME_IMX615_MIPI_RAW_21305,
        {
//            {PDN, Vol_High, 1},
            {RST, Vol_Low, 1},
            {AVDD, Vol_2900, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(IMX709_MIPI_RAW_21305)
    {
        SENSOR_DRVNAME_IMX709_MIPI_RAW_21305,
        {
//            {PDN, Vol_Low, 1},      //FAN53870 Power supply
            {SensorMCLK, Vol_High, 1},
            {AVDD_1, Vol_Low, 0},   //PONV
            {RST, Vol_Low, 1},
            {AVDD, Vol_2900, 0},
            {DVDD, Vol_1000, 0},
            {DOVDD, Vol_1800, 0},
            {AVDD_1, Vol_High, 1},
            {RST, Vol_High, 8},
        },
    },
#endif
#if defined(IMX355_MIPI_RAW_21305)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW_21305,
        {
//            {PDN, Vol_High, 1},
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_High, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2}
        },
    },
#endif
#if defined(GC02M1_MIPI_RAW_21305)
    {
        SENSOR_DRVNAME_GC02M1_MIPI_RAW_21305,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0, Vol_Low, 8},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 10}
        },
    },
#endif

    /* add new sensor before this line */
    {NULL,},
};
#ifdef SENSOR_PLATFORM_5G_H
struct CAMERA_DEVICE_INFO gImgEepromInfo_21127 = {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV50A_SENSOR_ID_21127,   0xA2, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "ov50a"},
//        {IMX471_SENSOR_ID_21127,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx471"},
        {IMX709_SENSOR_ID_21127,  0xA6, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx709"},
        {GC02M1B_SENSOR_ID_21127, 0xFF, {0xFF, 0xFF}, 0xFF, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02m1b"},
        {GC02M1_SENSOR_ID_21127,  0xA4, {0x00, 0x06}, 0xE0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r3", "gc02m1"},
    },

    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = DUALCAM_CALI_DATA_LENGTH_TOTAL_QCOM,
//    .i4MWStereoAddr = {OV50A_STEREO_START_ADDR, GC02M1B_STEREO_START_ADDR},
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
    .i4DistortionAddr = 0xFFFF,
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_21305 = {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {IMX766_SENSOR_ID_21305,  0xA2, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "imx766"},
//        {IMX615_SENSOR_ID_21305,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx615"},
        {IMX709_SENSOR_ID_21305,  0xA6, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx709"},
        {IMX355_SENSOR_ID_21305,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "imx355"},
        {GC02M1_SENSOR_ID_21305,  0xA4, {0x00, 0x06}, 0xE0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r3", "gc02m1"},
    },

    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = DUALCAM_CALI_DATA_LENGTH_TOTAL_QCOM,
    .i4MWStereoAddr = {IMX766_STEREO_START_ADDR, IMX355_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
    .i4DistortionAddr = IMX355_DISTORTIONPARAMS_START_ADDR,
};
#endif
struct CAMERA_DEVICE_INFO gImgEepromInfo_20645 = {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV64B_SENSOR_ID_20645,     0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "ov64b_20645"},
        {OV32A1Q_SENSOR_ID_20645,   0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "ov32a1q_20645"},
        {HI846_SENSOR_ID_20645,     0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "hi846_20645"},
        {OV02B10_SENSOR_ID_20645,   0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "ov02b10_20645"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR, HI846_STEREO_START_ADDR_20645},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20601[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(IMX686_MIPI_RAW_20601)
    {IMX686_SENSOR_ID_20601, SENSOR_DRVNAME_IMX686_MIPI_RAW_20601, IMX686_MIPI_RAW_20601_SensorInit},
#endif
#if defined(OV32A1Q_MIPI_RAW_20601)
    {OV32A1Q_SENSOR_ID_20601, SENSOR_DRVNAME_OV32A1Q_MIPI_RAW_20601, OV32A1Q_MIPI_RAW_20601_SensorInit},
#endif
#if defined(HI846_MIPI_RAW_20601)
    {HI846_SENSOR_ID_20601, SENSOR_DRVNAME_HI846_MIPI_RAW_20601, HI846_MIPI_RAW_20601_SensorInit},
#endif
#if defined(GC02M1B_MIPI_RAW_20601)
    {GC02M1B_SENSOR_ID_20601, SENSOR_DRVNAME_GC02M1B_MIPI_RAW_20601, GC02M1B_MIPI_RAW_20601_SensorInit},
#endif
#if defined(GC2385_MIPI_RAW_20601)
    {GC2385_SENSOR_ID_20601, SENSOR_DRVNAME_GC2385_MIPI_RAW_20601, GC2385_MIPI_RAW_20601_SensorInit},
#endif

    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

#define IMX766_REL_SENSOR_ID_21015 (IMX766_SENSOR_ID_21015 - SENSOR_ID_OFFSET_21015)
#define IMX709_REL_SENSOR_ID_21015 (IMX709_SENSOR_ID_21015 - SENSOR_ID_OFFSET_21015)
#define IMX355_REL_SENSOR_ID_21015 (IMX355_SENSOR_ID_21015 - SENSOR_ID_OFFSET_21015)
#define OV02B10_REL_SENSOR_ID_21015 (OV02B10_SENSOR_ID_21015 - SENSOR_ID_OFFSET_21015)
#define IMX709_REL_MONO_SENSOR_ID_21015 (IMX709_MONO_SENSOR_ID_21015 - 0xF000)
struct CAMERA_DEVICE_INFO gImgEepromInfo_21015 = {
    .i4SensorNum = 5,
    .pCamModuleInfo = {
        {IMX766_REL_SENSOR_ID_21015,   0xA2, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "imx766"},
        {IMX709_REL_SENSOR_ID_21015,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx709"},
        {IMX355_REL_SENSOR_ID_21015,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "imx355"},
        {OV02B10_REL_SENSOR_ID_21015, 0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "ov02b10"},
        {IMX709_REL_MONO_SENSOR_ID_21015,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f_mono", "imx709_mono"},
    },

    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,

    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = DUALCAM_CALI_DATA_LENGTH_TOTAL_QCOM,
    .i4MWStereoAddr = {IMX766_STEREO_START_ADDR, IMX355_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
    .i4DistortionAddr = IMX355_DISTORTIONPARAMS_START_ADDR,
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20601[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD_1,IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
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
    {
        IMGSENSOR_SENSOR_IDX_MAIN3,
        IMGSENSOR_I2C_DEV_4,
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

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20601[] = {
#if defined(IMX686_MIPI_RAW_20601)
    {
        SENSOR_DRVNAME_IMX686_MIPI_RAW_20601,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2900, 0},
            {AVDD_1, Vol_1800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 2},
            //{AFVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 3}
        },
    },
#endif
#if defined(HI846_MIPI_RAW_20601)
    {
        SENSOR_DRVNAME_HI846_MIPI_RAW_20601,
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
#if defined(GC02M1B_MIPI_RAW_20601)
    {
        SENSOR_DRVNAME_GC02M1B_MIPI_RAW_20601,
        {
            {RST, Vol_Low, 1},
            {DVDD, Vol_1800, 0},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 1},
            {RST, Vol_High, 2},
            {SensorMCLK, Vol_High, 1}
        },
    },
#endif
#if defined(GC2385_MIPI_RAW_20601)
    {
        SENSOR_DRVNAME_GC2385_MIPI_RAW_20601,
        {
            {RST, Vol_Low, 1},
            //{DVDD, Vol_1200, 0},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2}
        },
    },
#endif
#if defined(OV32A1Q_MIPI_RAW_20601)
    {
        SENSOR_DRVNAME_OV32A1Q_MIPI_RAW_20601,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {RST, Vol_High, 2},
            {SensorMCLK, Vol_High, 1},
        },
    },
#endif

    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_19165[MAX_NUM_OF_SUPPORT_SENSOR] = {
    #if defined(IMX686Q2R_MIPI_RAW)
    {IMX686Q2R_SENSOR_ID, SENSOR_DRVNAME_IMX686Q2R_MIPI_RAW, IMX686Q2R_MIPI_RAW_SensorInit},
    #endif
    #if defined(S5KGD1SPQ2R_MIPI_RAW)
    {S5KGD1SPQ2R_SENSOR_ID, SENSOR_DRVNAME_S5KGD1SPQ2R_MIPI_RAW, S5KGD1SPQ2R_MIPI_RAW_SensorInit},
    #endif
    #if defined(IMX616Q2R_MIPI_RAW)
    {IMX616Q2R_SENSOR_ID, SENSOR_DRVNAME_IMX616Q2R_MIPI_RAW, IMX616Q2R_MIPI_RAW_SensorInit},
    #endif
    #if defined(HI846Q2R_MIPI_RAW)
    {HI846Q2R_SENSOR_ID, SENSOR_DRVNAME_HI846Q2R_MIPI_RAW, HI846Q2R_MIPI_RAW_SensorInit},
    #endif
    #if defined(GC02M0B_MIPI_MONO)
    {GC02M0_SENSOR_ID, SENSOR_DRVNAME_GC02M0B_MIPI_MONO, GC02M0_MIPI_MONO_SensorInit},
    #endif
    #if defined(GC02M0B_MIPI_MONO1)
    {GC02M0_SENSOR_ID1, SENSOR_DRVNAME_GC02M0B_MIPI_MONO1, GC02M0_MIPI_MONO1_SensorInit},
    #endif
    /* ADD sensor driver before this line  */
    {0, {0}, NULL},
};


struct CAMERA_DEVICE_INFO gImgEepromInfo_19165 = {
    .i4SensorNum = 5,
    .pCamModuleInfo = {
        {IMX686Q2R_SENSOR_ID,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x98,0x94}, "Cam_r0", "imx686"},
        {S5KGD1SPQ2R_SENSOR_ID, 0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "s5kgd1sp"},
        {HI846Q2R_SENSOR_ID,    0xA0, {0x00, 0x06}, 0xB0, 1, {0x1A15,0x1A1B,0x1A19,0x1A17}, "Cam_r1", "hi846"},
        {GC02M0_SENSOR_ID,      0xFF, {0xFF, 0xFF}, 0xFF, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02m0"},
        {GC02M0_SENSOR_ID1,     0xFF, {0xFF, 0xFF}, 0xFF, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r3", "gc02m0"}
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {IMX686Q2R_STEREO_START_ADDR, HI846Q2R_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_19165[] = {
#if defined(IMX686Q2R_MIPI_RAW)
        {
                SENSOR_DRVNAME_IMX686Q2R_MIPI_RAW,
                {
                       {RST, Vol_Low, 1},
                       {AVDD, Vol_2900, 0},
                       {AVDD_1 ,Vol_1800, 0},
                       {DVDD, Vol_1100, 0},
                       {DOVDD,Vol_1800, 1},
                       {SensorMCLK, Vol_High, 1},
                       {RST, Vol_High, 3},
                },
        },
#endif
#if defined(S5KGD1SPQ2R_MIPI_RAW)
        {
                SENSOR_DRVNAME_S5KGD1SPQ2R_MIPI_RAW,
                {
                        {RST, Vol_Low, 1},
                        {AVDD, Vol_2800, 0},
                        {DVDD, Vol_1100, 0},
                        {DOVDD, Vol_1800, 1},
                        {SensorMCLK, Vol_High, 1},
                        {RST, Vol_High, 2}
                },
        },
#endif
#if defined(IMX616Q2R_MIPI_RAW)
      {
                SENSOR_DRVNAME_IMX616Q2R_MIPI_RAW,
                {
                        {RST, Vol_Low, 1},
                        {AVDD, Vol_2900,0},
                        {DVDD, Vol_1100, 0},
                        {DOVDD, Vol_1800, 1},
                        {SensorMCLK, Vol_High, 1},
                        {RST, Vol_High, 2},
                },
      },
#endif
#if defined(HI846Q2R_MIPI_RAW)
       {
                SENSOR_DRVNAME_HI846Q2R_MIPI_RAW,
                {
                        {RST, Vol_Low, 1},
                        {AVDD, Vol_2800, 0},
                        {DVDD, Vol_1200, 0},
                        {DOVDD, Vol_1800, 1},
                        {SensorMCLK, Vol_High, 1},
                        {RST, Vol_High, 2},
                },
       },
#endif
#if defined(GC02M0B_MIPI_MONO)
      {
                SENSOR_DRVNAME_GC02M0B_MIPI_MONO,
                {
                       {RST, Vol_Low, 1},
                       //{DVDD, Vol_1200, 0},
                       {DOVDD, Vol_1800, 1},
                       {AVDD, Vol_2800, 0},
                       {SensorMCLK, Vol_High, 1},
                       {RST, Vol_High, 2},
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
                       {RST, Vol_High, 2},
                },
       },
#endif

    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_19165[] = {
        {
                IMGSENSOR_SENSOR_IDX_MAIN,
                IMGSENSOR_I2C_DEV_0,
                {
                        {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
                        {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
                        #ifdef OPLUS_FEATURE_CAMERA_COMMON
                        {IMGSENSOR_HW_PIN_AVDD_1,  IMGSENSOR_HW_ID_GPIO},
                        #endif
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
                        {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
                },
        },

        {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20817[MAX_NUM_OF_SUPPORT_SENSOR] = {
/*20201224 add for 20817 sensor porting*/
#if defined(IMX766_MIPI_RAW20817)
	{IMX766_SENSOR_ID_20817, SENSOR_DRVNAME_IMX766_MIPI_RAW_20817, IMX766_MIPI_RAW20817_SensorInit},
#endif
#if defined(IMX615_MIPI_RAW20817)
	{IMX615_SENSOR_ID_20817, SENSOR_DRVNAME_IMX615_MIPI_RAW_20817, IMX615_MIPI_RAW20817_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW20817)
	{IMX355_SENSOR_ID_20817, SENSOR_DRVNAME_IMX355_MIPI_RAW_20817, IMX355_MIPI_RAW20817_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO20817)
	{GC02M1B_SENSOR_ID_20817, SENSOR_DRVNAME_GC02M1B_MIPI_MONO_20817, GC02M1B_MIPI_MONO20817_SensorInit},
#endif

    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20817[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD_1,IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_VOIS,  IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_SUB2,
        IMGSENSOR_I2C_DEV_3,
        {
            {IMGSENSOR_HW_PIN_UNDEF,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_UNDEF,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_UNDEF, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_UNDEF,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_UNDEF,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_MAIN3,
        IMGSENSOR_I2C_DEV_4,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20817[] = {
#if defined(IMX766_MIPI_RAW20817)
    {
        SENSOR_DRVNAME_IMX766_MIPI_RAW_20817,
        {
            {RST, Vol_Low, 1},
            {VOIS, Vol_High, 1},
            {AFVDD_20817, Vol_2800, 1},
            {AVDD, Vol_2800, 2},
            {AVDD_1, Vol_1800, 3},
            {DVDD, Vol_High, 4},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 6},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(IMX615_MIPI_RAW20817)
    {
        SENSOR_DRVNAME_IMX615_MIPI_RAW_20817,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2900, 2},
            {DVDD, Vol_High, 4},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(IMX355_MIPI_RAW20817)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW_20817,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2700, 2},
            {DVDD, Vol_1200, 3},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 3},
        },
    },
#endif
#if defined(GC02M1B_MIPI_MONO20817)
    {
        SENSOR_DRVNAME_GC02M1B_MIPI_MONO_20817,
        {
            {RST, Vol_Low, 1},
            //{DVDD, Vol_1200, 0},
            {DOVDD, Vol_1800, 1},
            //{AVDD, Vol_2800, 0, Vol_Low, 8},
            {AVDD, Vol_2800, 8},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 8},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20817_EVB2[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD_1,IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD,	 IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD_1,IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_VOIS,  IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_SUB2,
        IMGSENSOR_I2C_DEV_3,
        {
            {IMGSENSOR_HW_PIN_UNDEF,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_UNDEF,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_UNDEF, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_UNDEF,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_UNDEF,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_MAIN3,
        IMGSENSOR_I2C_DEV_4,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20817_EVB2[] = {
#if defined(IMX766_MIPI_RAW20817)
    {
        SENSOR_DRVNAME_IMX766_MIPI_RAW_20817,
        {
            {RST, Vol_Low, 1},
            {VOIS, Vol_High, 1},
            {AFVDD_20817, Vol_2800, 1},
            {AVDD_1, Vol_2800, 4},
            {AVDD, Vol_1800, 2},
            {DVDD, Vol_High, 2},
            {DVDD_1, Vol_High, 4},
            {DOVDD, Vol_1800, 3},
            {SensorMCLK, Vol_High, 6},
            {RST, Vol_High, 3},
        },
    },
#endif
#if defined(IMX615_MIPI_RAW20817)
    {
        SENSOR_DRVNAME_IMX615_MIPI_RAW_20817,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2900, 2},
            {DVDD, Vol_High, 4},
            {DOVDD, Vol_1800, 3},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(IMX355_MIPI_RAW20817)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW_20817,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 2},
            {DVDD, Vol_1200, 3},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 3},
        },
    },
#endif
#if defined(GC02M1B_MIPI_MONO20817)
    {
        SENSOR_DRVNAME_GC02M1B_MIPI_MONO_20817,
        {
            {RST, Vol_Low, 1},
            //{DVDD, Vol_1200, 0},
            {DOVDD, Vol_1800, 1},
            //{AVDD, Vol_2800, 0, Vol_Low, 8},
            {AVDD, Vol_2800, 8},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 8},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20817_EVT[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD_1,IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD_1,IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_VOIS  ,IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_VOIS_1,IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_SUB2,
        IMGSENSOR_I2C_DEV_3,
        {
            {IMGSENSOR_HW_PIN_UNDEF,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_UNDEF,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_UNDEF,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_UNDEF,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_UNDEF,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_MAIN3,
        IMGSENSOR_I2C_DEV_4,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20817_EVT[] = {
#if defined(IMX766_MIPI_RAW20817)
    {
        SENSOR_DRVNAME_IMX766_MIPI_RAW_20817,
        {
            {RST, Vol_Low, 1},
            {VOIS, Vol_High, 1},
            {VOIS_1, Vol_High, 1},
            {AFVDD_20817, Vol_2800, 1},
            {AVDD_1, Vol_2800, 4},
            {AVDD, Vol_1800, 2},
            {DVDD, Vol_High, 2},
            {DVDD_1, Vol_High, 4},
            {DOVDD, Vol_1800, 3},
            {SensorMCLK, Vol_High, 6},
            {RST, Vol_High, 3},
        },
    },
#endif
#if defined(IMX615_MIPI_RAW20817)
    {
        SENSOR_DRVNAME_IMX615_MIPI_RAW_20817,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2900, 2},
            {DVDD, Vol_High, 4},
            {DOVDD, Vol_1800, 3},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(IMX355_MIPI_RAW20817)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW_20817,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 2},
            {DVDD, Vol_1200, 3},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 3},
        },
    },
#endif
#if defined(GC02M1B_MIPI_MONO20817)
    {
        SENSOR_DRVNAME_GC02M1B_MIPI_MONO_20817,
        {
            {RST, Vol_Low, 1},
            //{DVDD, Vol_1200, 0},
            {DOVDD, Vol_1800, 1},
            //{AVDD, Vol_2800, 0, Vol_Low, 8},
            {AVDD, Vol_2800, 8},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 8},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

#ifdef SENSOR_PLATFORM_5G_H
struct CAMERA_DEVICE_INFO gImgEepromInfo_20817= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {IMX766_SENSOR_ID_20817,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0xFF,0x94}, "Cam_r0", "imx766"},
        {IMX615_SENSOR_ID_20817,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx615"},
        {IMX355_SENSOR_ID_20817,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "imx355"},
        {GC02M1B_SENSOR_ID_20817, 0xFF, {0xFF, 0xFF}, 0xFF, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02m1b"},
        {GC02M1B_SENSOR_ID_20817, 0xFF, {0xFF, 0xFF}, 0xFF, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02m1b"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {IMX766_STEREO_START_ADDR, IMX355_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
    .i4DistortionAddr = IMX355_DISTORTIONPARAMS_START_ADDR,
};
#endif

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_21881[MAX_NUM_OF_SUPPORT_SENSOR] = {
/*20201224 add for 21881 sensor porting*/
#if defined(IMX766_MIPI_RAW21881)
	{IMX766_SENSOR_ID_21881, SENSOR_DRVNAME_IMX766_MIPI_RAW_21881, IMX766_MIPI_RAW21881_SensorInit},
#endif
#if defined(IMX615_MIPI_RAW21881)
	{IMX615_SENSOR_ID_21881, SENSOR_DRVNAME_IMX615_MIPI_RAW_21881, IMX615_MIPI_RAW21881_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW21881)
	{IMX355_SENSOR_ID_21881, SENSOR_DRVNAME_IMX355_MIPI_RAW_21881, IMX355_MIPI_RAW21881_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO21881)
	{GC02M1B_SENSOR_ID_21881, SENSOR_DRVNAME_GC02M1B_MIPI_MONO_21881, GC02M1B_MIPI_MONO21881_SensorInit},
#endif

    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_21881[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD_1,IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD_1,IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_VOIS,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_VOIS_1,IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_SUB2,
        IMGSENSOR_I2C_DEV_3,
        {
            {IMGSENSOR_HW_PIN_UNDEF,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_UNDEF,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_UNDEF,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_UNDEF,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_UNDEF,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_MAIN3,
        IMGSENSOR_I2C_DEV_4,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_21881[] = {
#if defined(IMX766_MIPI_RAW21881)
    {
        SENSOR_DRVNAME_IMX766_MIPI_RAW_21881,
        {
            {RST, Vol_Low, 1},
            {VOIS, Vol_High, 1},
            {VOIS_1, Vol_High, 1},
            {AFVDD_21881, Vol_2800, 1},
            {AVDD_1, Vol_2800, 4},
            {AVDD, Vol_1800, 2},
            {DVDD, Vol_High, 2},
            {DVDD_1, Vol_High, 4},
            {DOVDD, Vol_1800, 10},
            {SensorMCLK, Vol_High, 6},
            {RST, Vol_High, 3},
        },
    },
#endif
#if defined(IMX615_MIPI_RAW21881)
    {
        SENSOR_DRVNAME_IMX615_MIPI_RAW_21881,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2900, 2},
            {DVDD, Vol_High, 4},
            {DOVDD, Vol_1800, 10},
            {SensorMCLK, Vol_High, 3},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(IMX355_MIPI_RAW21881)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW_21881,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 2},
            {DVDD, Vol_1200, 3},
            {DOVDD, Vol_1800, 10},
            {SensorMCLK, Vol_High, 6},
            {RST, Vol_High, 3},
        },
    },
#endif
#if defined(GC02M1B_MIPI_MONO21881)
    {
        SENSOR_DRVNAME_GC02M1B_MIPI_MONO_21881,
        {
            {RST, Vol_Low, 1},
            //{DVDD, Vol_1200, 0},
            {DOVDD, Vol_1800, 1},
            //{AVDD, Vol_2800, 0, Vol_Low, 8},
            {AVDD, Vol_2800, 8},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 8},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

#ifdef SENSOR_PLATFORM_5G_H
struct CAMERA_DEVICE_INFO gImgEepromInfo_21881= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {IMX766_SENSOR_ID_21881,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0xFF,0x94}, "Cam_r0", "imx766"},
        {IMX615_SENSOR_ID_21881,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx615"},
        {IMX355_SENSOR_ID_21881,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "imx355"},
        {GC02M1B_SENSOR_ID_21881, 0xFF, {0xFF, 0xFF}, 0xFF, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02m1b"},
        {GC02M1B_SENSOR_ID_21881, 0xFF, {0xFF, 0xFF}, 0xFF, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02m1b"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {IMX766_STEREO_START_ADDR, IMX355_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
    .i4DistortionAddr = IMX355_DISTORTIONPARAMS_START_ADDR,
};
#endif

void oplus_imgsensor_hwcfg()
{
    if (is_project(20131) || is_project(20133)
          || is_project(20255) || is_project(20257)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20131;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20131;
         oplus_sensor_power_sequence = sensor_power_sequence_20131;
         gImgEepromInfo = gImgEepromInfo_20131;
    } else if (is_project(20171) || is_project(20353)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20171;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20171;
         oplus_sensor_power_sequence = sensor_power_sequence_20171;
#ifdef SENSOR_PLATFORM_5G_H
         gImgEepromInfo = gImgEepromInfo_20171;
#endif
    } else if (is_project(21061)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_21061;
         oplus_imgsensor_custom_config = imgsensor_custom_config_21061;
         oplus_sensor_power_sequence = sensor_power_sequence_21061;
#ifdef SENSOR_PLATFORM_5G_H
         gImgEepromInfo = gImgEepromInfo_21061;
#endif
    } else if (is_project(20615) || is_project(21609) || is_project(20662)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20615;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20615;
         oplus_sensor_power_sequence = sensor_power_sequence_20615;
#ifdef SENSOR_PLATFORM_5G_H
         gImgEepromInfo = gImgEepromInfo_20615;
#endif
    } else if (is_project(20619)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20619;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20615;
         oplus_sensor_power_sequence = sensor_power_sequence_20619;
#ifdef SENSOR_PLATFORM_5G_H
         gImgEepromInfo = gImgEepromInfo_20619;
#endif
    } else if (is_project(21651)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_21651;
         oplus_imgsensor_custom_config = imgsensor_custom_config_21651;
         oplus_sensor_power_sequence = sensor_power_sequence_21651;
#ifdef SENSOR_PLATFORM_5G_H
         gImgEepromInfo = gImgEepromInfo_21651;
#endif
    }  else if (is_project(20645)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20645;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20645;
         oplus_sensor_power_sequence = sensor_power_sequence_20645;
         gImgEepromInfo = gImgEepromInfo_20645;
    }  else if (is_project(20601) || is_project(20602) || is_project(20660)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20601;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20601;
         oplus_sensor_power_sequence = sensor_power_sequence_20601;
         gImgEepromInfo = gImgEepromInfo_20131;
    }  else if (is_project(21015) || is_project(21217) || is_project(21218)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_21015;
         oplus_imgsensor_custom_config = imgsensor_custom_config_21015;
         oplus_sensor_power_sequence = sensor_power_sequence_21015;
         gImgEepromInfo = gImgEepromInfo_21015;
    }  else if (is_project(21127)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_21127;
         oplus_imgsensor_custom_config = imgsensor_custom_config_21127;
         oplus_sensor_power_sequence = sensor_power_sequence_21127;
#ifdef SENSOR_PLATFORM_5G_H
         gImgEepromInfo = gImgEepromInfo_21127;
#endif
    }  else if (is_project(21305)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_21305;
         oplus_imgsensor_custom_config = imgsensor_custom_config_21305;
         oplus_sensor_power_sequence = sensor_power_sequence_21305;
#ifdef SENSOR_PLATFORM_5G_H
         gImgEepromInfo = gImgEepromInfo_21305;
#endif
    } else if (is_project(19165)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_19165;
         oplus_imgsensor_custom_config = imgsensor_custom_config_19165;
         oplus_sensor_power_sequence = sensor_power_sequence_19165;
         gImgEepromInfo = gImgEepromInfo_19165;
    } else if (is_project(20817) || is_project(20827) || is_project(20831)) {
         int pcbVer = 0;
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20817;
         /*20210119 add for EVB2*/
         pcbVer = get_PCB_Version();
         pr_debug("oplus_imgsensor_hwcfg pcbVer: %d\n", pcbVer);
         if (PCB_VERSION_EVB == pcbVer) {
             /*20201224 add for 20817 sensor porting*/
             oplus_imgsensor_custom_config = imgsensor_custom_config_20817;
             oplus_sensor_power_sequence = sensor_power_sequence_20817;
             pr_debug("imgsensor_custom_config_20817 Selected\n");
         } else if(PCB_VERSION_EVB2 == pcbVer || PCB_VERSION_T0 == pcbVer
                 || PCB_VERSION_T1 == pcbVer) {
             oplus_imgsensor_custom_config = imgsensor_custom_config_20817_EVB2;
             oplus_sensor_power_sequence = sensor_power_sequence_20817_EVB2;
             pr_debug("imgsensor_custom_config_20817_EVB2 Selected\n");
         } else {
             /*20210311 add for 20817 sensor porting support EVT*/
             oplus_imgsensor_custom_config = imgsensor_custom_config_20817_EVT;
             oplus_sensor_power_sequence = sensor_power_sequence_20817_EVT;
             pr_debug("imgsensor_custom_config_20817_EVT Selected\n");
         }
#ifdef SENSOR_PLATFORM_5G_H
         gImgEepromInfo = gImgEepromInfo_20817;
#endif
    } else if (is_project(21881) || is_project(21882)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_21881;
         oplus_imgsensor_custom_config = imgsensor_custom_config_21881;
         oplus_sensor_power_sequence = sensor_power_sequence_21881;
         pr_debug("imgsensor_custom_config_21881 Selected\n");
#ifdef SENSOR_PLATFORM_5G_H
         gImgEepromInfo = gImgEepromInfo_21881;
#endif
    } else {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20131;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20131;
         oplus_sensor_power_sequence = sensor_power_sequence_20131;
         gImgEepromInfo = gImgEepromInfo_20131;
    }
}
#endif
