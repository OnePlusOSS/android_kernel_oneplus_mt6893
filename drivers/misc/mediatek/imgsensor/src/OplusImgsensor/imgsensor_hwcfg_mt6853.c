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
#ifndef __IMGSENSOR_HWCFG_20181_CTRL_H__
#define __IMGSENSOR_HWCFG_20181_CTRL_H__
#include "imgsensor_hwcfg_custom.h"

struct IMGSENSOR_SENSOR_LIST *oplus_gimgsensor_sensor_list = NULL;
struct IMGSENSOR_HW_CFG *oplus_imgsensor_custom_config = NULL;
struct IMGSENSOR_HW_POWER_SEQ *oplus_sensor_power_sequence = NULL;
struct IMGSENSOR_HW_POWER_SEQ *oplus_platform_power_sequence = NULL;
struct CAMERA_DEVICE_INFO gImgEepromInfo;

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20181[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV64B_MIPI_RAW)
{OV64B_SENSOR_ID, SENSOR_DRVNAME_OV64B_MIPI_RAW, OV64B_MIPI_RAW_SensorInit},
#endif

#if defined(IMX615_MIPI_RAW)
{IMX615_SENSOR_ID, SENSOR_DRVNAME_IMX615_MIPI_RAW, IMX615_MIPI_RAW_SensorInit},
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

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_23031[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV64B_MIPI_RAW23031)
{OV64B_SENSOR_ID23031, SENSOR_DRVNAME_OV64B_MIPI_RAW23031, OV64B_MIPI_RAW23031_SensorInit},
#endif
#if defined(OV08D_MIPI_RAW23031)
{OV08D_SENSOR_ID23031, SENSOR_DRVNAME_OV08D_MIPI_RAW23031, OV08D_MIPI_RAW23031_SensorInit},
#endif
#if defined(GC02M1_MIPI_RAW23031)
{GC02M1_SENSOR_ID23031, SENSOR_DRVNAME_GC02M1_MIPI_RAW23031, GC02M1_MIPI_RAW23031_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_22277[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV64B_MIPI_RAW22277)
{OV64B_SENSOR_ID22277, SENSOR_DRVNAME_OV64B_MIPI_RAW22277, OV64B_MIPI_RAW22277_SensorInit},
#endif
#if defined(OV32C_MIPI_RAW22277)
{OV32C_SENSOR_ID22277, SENSOR_DRVNAME_OV32C_MIPI_RAW22277, OV32C_MIPI_RAW22277_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW22277)
{IMX355_SENSOR_ID22277, SENSOR_DRVNAME_IMX355_MIPI_RAW22277, IMX355_MIPI_RAW22277_SensorInit},
#endif
#if defined(IMX709_MIPI_RAW22277)
{IMX709_SENSOR_ID22277, SENSOR_DRVNAME_IMX709_MIPI_RAW22277, IMX709_MIPI_RAW22277_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_23231[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(IMX882_MIPI_RAW23231)
{IMX882_SENSOR_ID23231, SENSOR_DRVNAME_IMX882_MIPI_RAW23231, IMX882_MIPI_RAW23231_SensorInit},
#endif
#if defined(OV32C_MIPI_RAW23231)
{OV32C_SENSOR_ID23231, SENSOR_DRVNAME_OV32C_MIPI_RAW23231, OV32C_MIPI_RAW23231_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW23231)
{IMX355_SENSOR_ID23231, SENSOR_DRVNAME_IMX355_MIPI_RAW23231, IMX355_MIPI_RAW23231_SensorInit},
#endif
#if defined(IMX709_MIPI_RAW23231)
{IMX709_SENSOR_ID23231, SENSOR_DRVNAME_IMX709_MIPI_RAW23231, IMX709_MIPI_RAW23231_SensorInit},
#endif
#if defined(IMX882PD_MIPI_RAW23231)
{IMX882PD_SENSOR_ID23231, SENSOR_DRVNAME_IMX882PD_MIPI_RAW23231, IMX882PD_MIPI_RAW23231_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_lijing[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(S5KJN103_MIPI_RAW_LIJING)
    {S5KJN103_SENSOR_ID_LIJING, SENSOR_DRVNAME_S5KJN103_MIPI_RAW_LIJING, S5KJN103_MIPI_RAW_LIJING_SensorInit},
#endif
#if defined(S5K3L6_MIPI_RAW_LIJING)
    {S5K3L6_SENSOR_ID_LIJING, SENSOR_DRVNAME_S5K3L6_MIPI_RAW_LIJING, S5K3L6_MIPI_RAW_LIJING_SensorInit},
#endif
#if defined(SC800CS_MIPI_RAW_LIJING)
    {SC800CS_SENSOR_ID_LIJING, SENSOR_DRVNAME_SC800CS_MIPI_RAW_LIJING, SC800CS_MIPI_RAW_LIJING_SensorInit},
#endif
#if defined(HI846_MIPI_RAW_LIJING)
    {HI846_SENSOR_ID_LIJING, SENSOR_DRVNAME_HI846_MIPI_RAW_LIJING, HI846_MIPI_RAW_LIJING_SensorInit},
#endif
#if defined(SC201CS_MIPI_RAW_LIJING)
    {SC201CS_SENSOR_ID_LIJING, SENSOR_DRVNAME_SC201CS_MIPI_RAW_LIJING, SC201CS_MIPI_RAW_LIJING_SensorInit},
#endif
#if defined(C2519_MIPI_RAW_LIJING)
    {C2519_SENSOR_ID_LIJING, SENSOR_DRVNAME_C2519_MIPI_RAW_LIJING, C2519_MIPI_RAW_LIJING_SensorInit},
#endif
    /* ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_23053[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(S5K3L6_MIPI_RAW_LIJINGA)
    {S5K3L6_SENSOR_ID_LIJINGA, SENSOR_DRVNAME_S5K3L6_MIPI_RAW_LIJINGA, S5K3L6_MIPI_RAW_LIJINGA_SensorInit},
#endif
#if defined(HI556_MIPI_RAW_LIJINGA)
    {HI556_SENSOR_ID_LIJINGA, SENSOR_DRVNAME_HI556_MIPI_RAW_LIJINGA, HI556_MIPI_RAW_LIJINGA_SensorInit},
#endif
#if defined(SC520CS_MIPI_RAW_LIJINGA)
    {SC520CS_SENSOR_ID_LIJINGA, SENSOR_DRVNAME_SC520CS_MIPI_RAW_LIJINGA, SC520CS_MIPI_RAW_LIJINGA_SensorInit},
#endif
    /* ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_23253[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(S5K3L6_MIPI_RAW_LIJINGA)
    {S5K3L6_SENSOR_ID_LIJINGA, SENSOR_DRVNAME_S5K3L6_MIPI_RAW_LIJINGA, S5K3L6_MIPI_RAW_LIJINGA_SensorInit},
#endif
#if defined(HI846_MIPI_RAW_LIJINGA)
    {HI846_SENSOR_ID_LIJINGA, SENSOR_DRVNAME_HI846_MIPI_RAW_LIJINGA, HI846_MIPI_RAW_LIJINGA_SensorInit},
#endif
#if defined(SC201CS_MIPI_RAW_LIJINGA)
    {SC201CS_SENSOR_ID_LIJINGA, SENSOR_DRVNAME_SC201CS_MIPI_RAW_LIJINGA, SC201CS_MIPI_RAW_LIJINGA_SensorInit},
#endif
    /* ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};
struct IMGSENSOR_HW_CFG imgsensor_custom_config_23053[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO}, //SCL2
            {IMGSENSOR_HW_PIN_DVDD_1,IMGSENSOR_HW_ID_GPIO}, //SDA2
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AFVDD,   IMGSENSOR_HW_ID_REGULATOR},
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
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_23253[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO}, //SCL2
            {IMGSENSOR_HW_PIN_DVDD_1,IMGSENSOR_HW_ID_GPIO}, //SDA2
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AFVDD,   IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},     //SCL8
            {IMGSENSOR_HW_PIN_AVDD_1,IMGSENSOR_HW_ID_GPIO},     //SDA8
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {IMGSENSOR_SENSOR_IDX_NONE}
};
struct IMGSENSOR_HW_CFG imgsensor_custom_config_lijing[] = {
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
            {IMGSENSOR_HW_PIN_AFVDD,   IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},//SCL4
            {IMGSENSOR_HW_PIN_DVDD_1,IMGSENSOR_HW_ID_GPIO},//SDA4
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
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},//SCL8
            {IMGSENSOR_HW_PIN_AVDD_1,IMGSENSOR_HW_ID_GPIO},     //SDA8
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD_1,IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_23053[] = {
#if defined(S5K3L6_MIPI_RAW_LIJINGA)
    {
        SENSOR_DRVNAME_S5K3L6_MIPI_RAW_LIJINGA,
        {
            {PDN, Vol_High, 1},
            {DVDD_1, Vol_High, 1},
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1050, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
            {AF_VDD, Vol_2800, 0},
        },
    },
#endif
#if defined(HI556_MIPI_RAW_LIJINGA)
    {
        SENSOR_DRVNAME_HI556_MIPI_RAW_LIJINGA,
        {
            {RST, Vol_Low, 0},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 1},
            {DVDD, Vol_1200, 2},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(SC520CS_MIPI_RAW_LIJINGA)
    {
        SENSOR_DRVNAME_SC520CS_MIPI_RAW_LIJINGA,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1200, 2},
            {AVDD, Vol_2800, 2},
            {RST, Vol_High, 2},
            {SensorMCLK, Vol_High, 5},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_23253[] = {
#if defined(S5K3L6_MIPI_RAW_LIJINGA)
    {
        SENSOR_DRVNAME_S5K3L6_MIPI_RAW_LIJINGA,
        {
            {PDN, Vol_High, 1},
            {DVDD_1, Vol_High, 1},
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1050, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
            {AF_VDD, Vol_2800, 0},
        },
    },
#endif
#if defined(HI846_MIPI_RAW_LIJINGA)
    {
        SENSOR_DRVNAME_HI846_MIPI_RAW_LIJINGA,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1200, 0},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(SC201CS_MIPI_RAW_LIJINGA)
    {
        SENSOR_DRVNAME_SC201CS_MIPI_RAW_LIJINGA,
        {
            {AVDD_1, Vol_High, 1},
            {DVDD, Vol_High, 1},
            {RST, Vol_Low, 0},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 2},
            {RST, Vol_High, 2},
            {RST, Vol_Low, 2},
            {RST, Vol_High, 5},
            {SensorMCLK, Vol_High, 1},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_lijing[] = {
#if defined(S5KJN103_MIPI_RAW_LIJING)
    {
        SENSOR_DRVNAME_S5KJN103_MIPI_RAW_LIJING,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 0},
            {DVDD, Vol_1100, 1},
            {AVDD, Vol_2800, 5},
            {AF_VDD, Vol_2800, 0},
            {RST, Vol_High, 2},
            {SensorMCLK, Vol_High, 10},
        },
    },
#endif
#if defined(S5K3L6_MIPI_RAW_LIJING)
    {
        SENSOR_DRVNAME_S5K3L6_MIPI_RAW_LIJING,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
            {AF_VDD, Vol_2800, 0},
        },
    },
#endif
#if defined(SC800CS_MIPI_RAW_LIJING)
    {
        SENSOR_DRVNAME_SC800CS_MIPI_RAW_LIJING,
        {
            {PDN, Vol_High, 1},
            {DVDD_1, Vol_High, 1},
            {RST, Vol_Low, 0},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1200, 2},
            {AVDD, Vol_High, 1},
            {RST, Vol_High, 1},
            {RST, Vol_Low, 1},
            {RST, Vol_High, 4},
            {SensorMCLK, Vol_High, 5},
        },
    },
#endif
#if defined(HI846_MIPI_RAW_LIJING)
    {
        SENSOR_DRVNAME_HI846_MIPI_RAW_LIJING,
        {
            {PDN, Vol_High, 1},
            {DVDD_1, Vol_High, 1},
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1200, 0},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(SC201CS_MIPI_RAW_LIJING)
    {
        SENSOR_DRVNAME_SC201CS_MIPI_RAW_LIJING,
        {
            {AVDD_1, Vol_High, 1},
            {DVDD, Vol_High, 1},
            {RST, Vol_Low, 0},
            {DOVDD, Vol_1800, 1},
            {DVDD_1, Vol_1200, 1},
            {PDN, Vol_High, 1},
            {AVDD, Vol_High, 2},
            {PDN, Vol_Low, 0},
            {RST, Vol_High, 2},
            {RST, Vol_Low, 2},
            {RST, Vol_High, 5},
            {SensorMCLK, Vol_High, 1},
        },
    },
#endif
#if defined(C2519_MIPI_RAW_LIJING)
    {
        SENSOR_DRVNAME_C2519_MIPI_RAW_LIJING,
        {
            {AVDD_1, Vol_High, 1},
            {DVDD, Vol_High, 1},
            {RST, Vol_Low, 0},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_High, 5},
            {RST, Vol_High, 5},
            {SensorMCLK, Vol_High, 1},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_lijing = {
    .i4SensorNum = 3,
    .pCamModuleInfo = {
        {S5KJN103_SENSOR_ID_LIJING,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5kjn103_mipi_raw_lijing"},
        {SC800CS_SENSOR_ID_LIJING, 0xA0, {0x00, 0x06}, 0x0834, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "sc800cs_mipi_raw_lijing"},
        {SC201CS_SENSOR_ID_LIJING,  0xFF, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "sc201cs_mipi_raw_lijing"},
    },
    .i4MWDataIdx = 0xFF,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_23053 = {
    .i4SensorNum = 2,
    .pCamModuleInfo = {
        {S5K3L6_SENSOR_ID_LIJINGA,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5k3l6_mipi_raw_lijinga"},
    },
    .i4MWDataIdx = 0xFF,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_23253 = {
    .i4SensorNum = 3,
    .pCamModuleInfo = {
        {S5K3L6_SENSOR_ID_LIJINGA,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5k3l6_mipi_raw_lijinga"},
        {SC201CS_SENSOR_ID_LIJINGA,  0xFF, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "sc201cs_mipi_raw_lijinga"},
    },
    .i4MWDataIdx = 0xFF,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_fanli[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(S5KJN103_MIPI_RAW_FANLI)
    {S5KJN103_SENSOR_ID_FANLI, SENSOR_DRVNAME_S5KJN103_MIPI_RAW_FANLI, S5KJN103_MIPI_RAW_FANLI_SensorInit},
#endif
#if defined(S5K3L6_MIPI_RAW_FANLI)
    {S5K3L6_SENSOR_ID_FANLI, SENSOR_DRVNAME_S5K3L6_MIPI_RAW_FANLI, S5K3L6_MIPI_RAW_FANLI_SensorInit},
#endif
#if defined(OV08D10_MIPI_RAW_FANLI)
    {OV08D10_SENSOR_ID_FANLI, SENSOR_DRVNAME_OV08D10_MIPI_RAW_FANLI, OV08D10_MIPI_RAW_FANLI_SensorInit},
#endif
#if defined(GC02M1_MIPI_RAW_FANLI)
    {GC02M1_SENSOR_ID_FANLI, SENSOR_DRVNAME_GC02M1_MIPI_RAW_FANLI, GC02M1_MIPI_RAW_FANLI_SensorInit},
#endif
#if defined(OV02B1B_MIPI_MONO_FANLI)
    {OV02B1B_SENSOR_ID_FANLI, SENSOR_DRVNAME_OV02B1B_MIPI_MONO_FANLI, OV02B1B_MIPI_MONO_FANLI_SensorInit},
#endif
    /* ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_fanli_T0[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            //{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AFVDD,   IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_fanli[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            //{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AFVDD,   IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_SUB,
        IMGSENSOR_I2C_DEV_1,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD_1,  IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_AVDD_1,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_AVDD_1,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_AVDD_1,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_fanli_T0[] = {
#if defined(S5KJN103_MIPI_RAW_FANLI)
    {
        SENSOR_DRVNAME_S5KJN103_MIPI_RAW_FANLI,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 0},
            {DVDD, Vol_1050, 1},
            {AVDD, Vol_2800, 5},
            {AF_VDD, Vol_2800, 0},
            {RST, Vol_High, 2},
            {SensorMCLK, Vol_High, 10},
        },
    },
#endif
#if defined(S5K3L6_MIPI_RAW_FANLI)
    {
        SENSOR_DRVNAME_S5K3L6_MIPI_RAW_FANLI,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1050, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
            {AF_VDD, Vol_2800, 0},
        },
    },
#endif
#if defined(OV08D10_MIPI_RAW_FANLI)
    {
        SENSOR_DRVNAME_OV08D10_MIPI_RAW_FANLI,
        {
            {RST, Vol_Low, 3},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 6},
            {DVDD, Vol_1200, 8},
            {RST, Vol_High, 3},
            {SensorMCLK, Vol_High, 8},
        },
    },
#endif
#if defined(GC02M1_MIPI_RAW_FANLI)
    {
        SENSOR_DRVNAME_GC02M1_MIPI_RAW_FANLI,
        {
            {RST, Vol_Low, 3},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 5},
            {RST, Vol_High, 5},
            {SensorMCLK, Vol_High, 3},
        },
    },
#endif
#if defined(OV02B1B_MIPI_MONO_FANLI)
    {
        SENSOR_DRVNAME_OV02B1B_MIPI_MONO_FANLI,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 10},
            {RST, Vol_High, 10},
            {SensorMCLK, Vol_High, 6},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_fanli[] = {
#if defined(S5KJN103_MIPI_RAW_FANLI)
    {
        SENSOR_DRVNAME_S5KJN103_MIPI_RAW_FANLI,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 0},
            {DVDD, Vol_1050, 1},
            {AVDD, Vol_2800, 5},
            {AF_VDD, Vol_2800, 0},
            {RST, Vol_High, 8},
            {SensorMCLK, Vol_High, 10},
        },
    },
#endif
#if defined(S5K3L6_MIPI_RAW_FANLI)
    {
        SENSOR_DRVNAME_S5K3L6_MIPI_RAW_FANLI,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1050, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
            {AF_VDD, Vol_2800, 0},
        },
    },
#endif
#if defined(OV08D10_MIPI_RAW_FANLI)
    {
        SENSOR_DRVNAME_OV08D10_MIPI_RAW_FANLI,
        {
            {RST, Vol_Low, 3},
            {DOVDD, Vol_1800, 1},
            {AVDD_1, Vol_2800, 6},
            {DVDD, Vol_1200, 8},
            {RST, Vol_High, 3},
            {SensorMCLK, Vol_High, 8},
        },
    },
#endif
#if defined(GC02M1_MIPI_RAW_FANLI)
    {
        SENSOR_DRVNAME_GC02M1_MIPI_RAW_FANLI,
        {
            {RST, Vol_Low, 3},
            {DOVDD, Vol_1800, 1},
            {AVDD_1, Vol_2800, 5},
            {RST, Vol_High, 5},
            {SensorMCLK, Vol_High, 3},
        },
    },
#endif
#if defined(OV02B1B_MIPI_MONO_FANLI)
    {
        SENSOR_DRVNAME_OV02B1B_MIPI_MONO_FANLI,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD_1, Vol_2800, 10},
            {RST, Vol_High, 10},
            {SensorMCLK, Vol_High, 0},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_22331 = {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {S5KJN103_SENSOR_ID_FANLI, 0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5kjn103_mipi_raw_fanli"},
        {OV08D10_SENSOR_ID_FANLI,  0xA0, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "ov08d10_mipi_raw_fanli"},
        {OV02B1B_SENSOR_ID_FANLI,  0xFF, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "ov02b1b_mipi_mono_fanli"},
        {GC02M1_SENSOR_ID_FANLI,   0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "GC02M1_mipi_raw_fanli"},
    },
    .i4MWDataIdx = 0xFF,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_22088 = {
    .i4SensorNum = 3,
    .pCamModuleInfo = {
        {S5K3L6_SENSOR_ID_FANLI,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "S5K3L6_mipi_raw_fanli"},
        {OV08D10_SENSOR_ID_FANLI,  0xA0, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "ov08d10_mipi_raw_fanli"},
        {OV02B1B_SENSOR_ID_FANLI,  0xFF, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "ov02b1b_mipi_mono_fanli"},
    },
    .i4MWDataIdx = 0xFF,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20151[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV48B_MIPI_RAW)
{OV48B_SENSOR_ID, SENSOR_DRVNAME_OV48B_MIPI_RAW, OV48B_MIPI_RAW_SensorInit},
#endif
#if defined(S5KGM1ST_MIPI_RAW)
{S5KGM1ST_SENSOR_ID, SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW, S5KGM1ST_MIPI_RAW_SensorInit},
#endif
#if defined(IMX471_MIPI_RAW)
{IMX471_SENSOR_ID, SENSOR_DRVNAME_IMX471_MIPI_RAW, IMX471_MIPI_RAW_SensorInit},
#endif
#if defined(HI846_MIPI_RAW)
{HI846_SENSOR_ID, SENSOR_DRVNAME_HI846_MIPI_RAW, HI846_MIPI_RAW_SensorInit},
#endif
#if defined(GC02K0_MIPI_RAW)
{GC02K0_SENSOR_ID, SENSOR_DRVNAME_GC02K0_MIPI_RAW, GC02K0_MIPI_RAW_SensorInit},
#endif
#if defined(OV02B10_MIPI_RAW)
{OV02B10_SENSOR_ID, SENSOR_DRVNAME_OV02B10_MIPI_RAW, OV02B10_MIPI_RAW_SensorInit},
#endif
#if defined(OV02A1B_MIPI_MONO)
{OV02A1B_SENSOR_ID, SENSOR_DRVNAME_OV02A1B_MIPI_MONO, OV02A1B_MIPI_MONO_SensorInit},
#endif
#if defined(OV02B1B_MIPI_MONO)
{OV02B1B_SENSOR_ID1, SENSOR_DRVNAME_OV02B1B_MIPI_MONO, OV02B1B_MIPI_MONO_SensorInit},
#endif

    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST gimgsensor_sensor_list_20630[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(S5KGW3_MIPI_RAW_ATHENSB)
{S5KGW3_SENSOR_ID_20631, SENSOR_DRVNAME_S5KGW3_MIPI_RAW_ATHENSB, S5KGW3_MIPI_RAW_SensorInit},
#endif
#if defined(OV32A1Q_MIPI_RAW_ATHENSB)
{OV32A1Q_SENSOR_ID_20631, SENSOR_DRVNAME_OV32A1Q_MIPI_RAW_ATHENSB, ATHENSB_OV32A1Q_MIPI_RAW_SensorInit},
#endif
#if defined(HI846_MIPI_RAW_ATHENSB)
{HI846_SENSOR_ID_20631, SENSOR_DRVNAME_HI846_MIPI_RAW_ATHENSB, ATHENSB_HI846_MIPI_RAW_SensorInit},
#endif
#if defined(GC02K_MIPI_RAW_ATHENSB)
{GC02K_SENSOR_ID_20631, SENSOR_DRVNAME_GC02K_MIPI_RAW_ATHENSB, ATHENSB_GC02K_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M1B_MIPI_RAW_ATHENSB)
{GC02M1B_SENSOR_ID_20631, SENSOR_DRVNAME_GC02M1B_MIPI_RAW_ATHENSB, ATHENSB_GC02M1B_MIPI_RAW_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */

};

struct IMGSENSOR_SENSOR_LIST gimgsensor_sensor_list_20633[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(S5KGM1ST_MIPI_RAW_ATHENSC)
    {S5KGM1ST_SENSOR_ID_20633c, SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_ATHENSC, ATHENSC_S5KGM1ST_MIPI_RAW_SensorInit},
#endif
#if defined(IMX471_MIPI_RAW_ATHENSC)
    {IMX471_SENSOR_ID_20633c, SENSOR_DRVNAME_IMX471_MIPI_RAW_ATHENSC, ATHENSC_IMX471_MIPI_RAW_SensorInit},
#endif
#if defined(HI846_MIPI_RAW_ATHENSC)
    {HI846_SENSOR_ID_20633c, SENSOR_DRVNAME_HI846_MIPI_RAW_ATHENSC, ATHENSC_HI846_MIPI_RAW_SensorInit},
#endif
#if defined(GC02K_MIPI_RAW_ATHENSB)
    {GC02K_SENSOR_ID_20631, SENSOR_DRVNAME_GC02K_MIPI_RAW_ATHENSB, ATHENSB_GC02K_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M1B_MIPI_RAW_ATHENSB)
    {GC02M1B_SENSOR_ID_20631, SENSOR_DRVNAME_GC02M1B_MIPI_RAW_ATHENSB, ATHENSB_GC02M1B_MIPI_RAW_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20391[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV64B_MIPI_RAW)
{OV64B_SENSOR_ID, SENSOR_DRVNAME_OV64B_MIPI_RAW, OV64B_MIPI_RAW_SensorInit},
#endif
#if defined(OV48B_MIPI_RAW)
{OV48B_SENSOR_ID, SENSOR_DRVNAME_OV48B_MIPI_RAW, OV48B_MIPI_RAW_SensorInit},
#endif
#if defined(OV32A_MIPI_RAW)
{OV32A_SENSOR_ID, SENSOR_DRVNAME_OV32A_MIPI_RAW, OV32A_MIPI_RAW_SensorInit},
#endif
#if defined(HI846_MIPI_RAW)
{HI846_SENSOR_ID, SENSOR_DRVNAME_HI846_MIPI_RAW, HI846_MIPI_RAW_SensorInit},
#endif
#if defined(OV02B10_MIPI_RAW)
{OV02B10_SENSOR_ID, SENSOR_DRVNAME_OV02B10_MIPI_RAW, OV02B10_MIPI_RAW_SensorInit},
#endif

    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20001[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV16A10_MIPI_RAW)
{OV16A10_SENSOR_ID, SENSOR_DRVNAME_OV16A10_MIPI_RAW, OV16A10_MIPI_RAW_SensorInit},
#endif
#if defined(S5K4H7_MIPI_RAW)
{S5K4H7_SENSOR_ID, SENSOR_DRVNAME_S5K4H7_MIPI_RAW, S5K4H7_MIPI_RAW_SensorInit},
#endif
#if defined(HI846_MIPI_RAW)
{HI846_SENSOR_ID, SENSOR_DRVNAME_HI846_MIPI_RAW, HI846_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO)
{GC02M1B_SENSOR_ID, SENSOR_DRVNAME_GC02M1B_MIPI_MONO, GC02M1B_MIPI_MONO_SensorInit},
#endif

    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_22693[MAX_NUM_OF_SUPPORT_SENSOR] = {
	#if defined(IMX766_MIPI_RAW22693)
	{IMX766_SENSOR_ID22693, SENSOR_DRVNAME_IMX766_MIPI_RAW22693, IMX766_MIPI_RAW22693_SensorInit},
	#endif
	#if defined(S5KHM6SP_MIPI_RAW22693)
	{S5KHM6SP_SENSOR_ID22693, SENSOR_DRVNAME_S5KHM6SP_MIPI_RAW22693, S5KHM6SP_MIPI_RAW22693_SensorInit},
	#endif
	#if defined(S5K3P9SP_MIPI_RAW22693)
	{S5K3P9SP_SENSOR_ID22693, SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW22693, S5K3P9SP_MIPI_RAW22693_SensorInit},
	#endif
	#if defined(IMX355_MIPI_RAW22693)
	{IMX355_SENSOR_ID22693, SENSOR_DRVNAME_IMX355_MIPI_RAW22693, IMX355_MIPI_RAW22693_SensorInit},
	#endif
	#if defined(GC02M1_MIPI_RAW22693)
	{GC02M1_SENSOR_ID22693, SENSOR_DRVNAME_GC02M1_MIPI_RAW22693, GC02M1_MIPI_RAW22693_SensorInit},
	#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_23707[MAX_NUM_OF_SUPPORT_SENSOR] = {
    #if defined(S5KJN1_MIPI_RAW23707)
    {S5KJN1_SENSOR_ID23707, SENSOR_DRVNAME_S5KJN1_MIPI_RAW23707, S5KJN1_MIPI_RAW23707_SensorInit},
    #endif
    #if defined(HI1634Q_MIPI_RAW23707)
    {HI1634Q_SENSOR_ID23707, SENSOR_DRVNAME_HI1634Q_MIPI_RAW23707, HI1634Q_MIPI_RAW23707_SensorInit},
    #endif
    #if defined(GC02M1_MIPI_RAW23707)
    {GC02M1_SENSOR_ID23707, SENSOR_DRVNAME_GC02M1_MIPI_RAW23707, GC02M1_MIPI_RAW23707_SensorInit},
    #endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_23687[MAX_NUM_OF_SUPPORT_SENSOR] = {
	#if defined(IMX882_MIPI_RAW23687)
	{IMX882_SENSOR_ID23687, SENSOR_DRVNAME_IMX882_MIPI_RAW23687, IMX882_MIPI_RAW23687_SensorInit},
	#endif
	#if defined(IMX890_MIPI_RAW23689)
	{IMX890_SENSOR_ID23689, SENSOR_DRVNAME_IMX890_MIPI_RAW23689, IMX890_MIPI_RAW23689_SensorInit},
	#endif
	#if defined(HI1634Q_MIPI_RAW23687)
	{HI1634Q_SENSOR_ID23687, SENSOR_DRVNAME_HI1634Q_MIPI_RAW23687, HI1634Q_MIPI_RAW23687_SensorInit},
	#endif
	#if defined(HI846W_MIPI_RAW23687)
	{HI846W_SENSOR_ID23687, SENSOR_DRVNAME_HI846W_MIPI_RAW23687, HI846W_MIPI_RAW23687_SensorInit},
	#endif
	#if defined(GC02M1_MIPI_RAW23687)
	{GC02M1_SENSOR_ID23687, SENSOR_DRVNAME_GC02M1_MIPI_RAW23687, GC02M1_MIPI_RAW23687_SensorInit},
	#endif
	#if defined(IMX882PD_MIPI_RAW23687)
	{IMX882PD_SENSOR_ID23687, SENSOR_DRVNAME_IMX882PD_MIPI_RAW23687, IMX882PD_MIPI_RAW23687_SensorInit},
	#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_22629[MAX_NUM_OF_SUPPORT_SENSOR] = {
	#if defined(S5KHP3SP_MIPI_RAW22629)
	{S5KHP3SP_SENSOR_ID22629, SENSOR_DRVNAME_S5KHP3SP_MIPI_RAW22629, S5KHP3SP_MIPI_RAW22629_SensorInit},
	#endif
    #if defined(IMX615_MIPI_RAW22629)
	{IMX615_SENSOR_ID22629, SENSOR_DRVNAME_IMX615_MIPI_RAW22629, IMX615_MIPI_RAW22629_SensorInit},
	#endif
	#if defined(IMX355_MIPI_RAW22629)
	{IMX355_SENSOR_ID22629, SENSOR_DRVNAME_IMX355_MIPI_RAW22629, IMX355_MIPI_RAW22629_SensorInit},
	#endif
	#if defined(GC02M1_MIPI_RAW22629)
	{GC02M1_SENSOR_ID22629, SENSOR_DRVNAME_GC02M1_MIPI_RAW22629, GC02M1_MIPI_RAW22629_SensorInit},
	#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_22633[MAX_NUM_OF_SUPPORT_SENSOR] = {
    #if defined(OVA0B4_MIPI_RAW22633)
    {OVA0B4_SENSOR_ID22633, SENSOR_DRVNAME_OVA0B4_MIPI_RAW22633, OVA0B4_MIPI_RAW22633_SensorInit},
    #endif
    #if defined(S5K3P9SP_MIPI_RAW22633)
    {S5K3P9SP_SENSOR_ID22633, SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW22633, S5K3P9SP_MIPI_RAW22633_SensorInit},
    #endif
    #if defined(SC202CS_MIPI_MONO22633)
    {SC202CS_SENSOR_ID22633, SENSOR_DRVNAME_SC202CS_MIPI_MONO22633, SC202CS_MIPI_MONO22633_SensorInit},
    #endif
    #if defined(OV02B1B_MIPI_MONO22633)
    {OV02B1B_SENSOR_ID22633, SENSOR_DRVNAME_OV02B1B_MIPI_MONO22633, OV02B1B_MIPI_MONO22633_SensorInit},
    #endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_moss[MAX_NUM_OF_SUPPORT_SENSOR] = {
	#if defined(IMX766_MOSSMIPI_RAW)
	{IMX766_SENSOR_ID_MOSS, SENSOR_DRVNAME_IMX766_MOSSMIPI_RAW, IMX766_MIPI_RAW_MOSS_SensorInit},
	#endif
	#if defined(IMX471_MOSSMIPI_RAW)
	{IMX471_SENSOR_ID_MOSS, SENSOR_DRVNAME_IMX471_MOSSMIPI_RAW, IMX471_MIPI_RAW_MOSS_SensorInit},
	#endif
	#if defined(IMX355_MOSSMIPI_RAW)
	{IMX355_SENSOR_ID_MOSS, SENSOR_DRVNAME_IMX355_MOSSMIPI_RAW, IMX355_MIPI_RAW_MOSS_SensorInit},
	#endif
	#if defined(GC02M1_MOSSMIPI_RAW)
	{GC02M1_SENSOR_ID_MOSS, SENSOR_DRVNAME_GC02M1_MOSSMIPI_RAW, GC02M1_MIPI_RAW_MOSS_SensorInit},
	#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_mossa[MAX_NUM_OF_SUPPORT_SENSOR] = {
	#if defined(S5KGM1ST_MOSSAMIPI_RAW)
	{S5KGM1ST_SENSOR_ID_MOSSA, SENSOR_DRVNAME_S5KGM1ST_MOSSAMIPI_RAW, S5KGM1ST_MIPI_RAW_MOSSA_SensorInit},
	#endif
	#if defined(IMX471_MOSSMIPI_RAW)
	{IMX471_SENSOR_ID_MOSS, SENSOR_DRVNAME_IMX471_MOSSMIPI_RAW, IMX471_MIPI_RAW_MOSS_SensorInit},
	#endif
	#if defined(IMX355_MOSSMIPI_RAW)
	{IMX355_SENSOR_ID_MOSS, SENSOR_DRVNAME_IMX355_MOSSMIPI_RAW, IMX355_MIPI_RAW_MOSS_SensorInit},
	#endif
	#if defined(GC02M1_MOSSAMIPI_RAW)
	{GC02M1_SENSOR_ID_MOSSA, SENSOR_DRVNAME_GC02M1_MOSSAMIPI_RAW, GC02M1_MIPI_RAW_MOSSA_SensorInit},
	#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20075[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV48B_CHIVASMIPI_RAW)
{OV48B_SENSOR_ID_CHIVAS, SENSOR_DRVNAME_OV48B_CHIVASMIPI_RAW, OV48B_MIPI_RAW_CHIVAS_SensorInit},
#endif
#if defined(OV32A_CHIVASMIPI_RAW)
{OV32A_SENSOR_ID_CHIVAS, SENSOR_DRVNAME_OV32A_CHIVASMIPI_RAW, OV32A_MIPI_RAW_CHIVAS_SensorInit},
#endif
#if defined(HI846_CHIVASMIPI_RAW)
{HI846_SENSOR_ID_CHIVAS, SENSOR_DRVNAME_HI846_CHIVASMIPI_RAW, HI846_MIPI_RAW_CHIVAS_SensorInit},
#endif
#if defined(GC02K0_CHIVASMIPI_RAW)
{GC02K0_SENSOR_ID_CHIVAS, SENSOR_DRVNAME_GC02K0_CHIVASMIPI_RAW, GC02K0_MIPI_RAW_CHIVAS_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_21081[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(IMX581_MIPI_RAW)
{IMX581_SENSOR_ID, SENSOR_DRVNAME_IMX581_MIPI_RAW, IMX581_MIPI_RAW_SensorInit},
#endif
#if defined(IMX471_MIPI_RAW)
{IMX471_SENSOR_ID, SENSOR_DRVNAME_IMX471_MIPI_RAW, IMX471_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO)
{GC02M1B_SENSOR_ID, SENSOR_DRVNAME_GC02M1B_MIPI_MONO, GC02M1B_MIPI_MONO_SensorInit},
#endif
#if defined(GC02M1_MIPI_RAW)
{GC02M1_SENSOR_ID_DUFU, SENSOR_DRVNAME_GC02M1_MIPI_RAW, GC02M1_MIPI_RAW_SensorInit},
#endif

    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20015[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV13B10_MIPI_RAW_CARR)
	{CARR_OV13B10_SENSOR_ID, SENSOR_DRVNAME_OV13B10_MIPI_RAW_CARR, CARR_OV13B10_MIPI_RAW_SensorInit},
#endif
#if defined(S5K3L6_MIPI_RAW_CARR)
	{CARR_S5K3L6_SENSOR_ID, SENSOR_DRVNAME_S5K3L6_MIPI_RAW_CARR, CARR_S5K3L6_MIPI_RAW_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW_CARR)
	{CARR_IMX355_SENSOR_ID, SENSOR_DRVNAME_IMX355_MIPI_RAW_CARR, CARR_IMX355_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO_CARR)
	{CARR_GC02M1B_SENSOR_ID, SENSOR_DRVNAME_GC02M1B_MIPI_MONO_CARR, CARR_GC02M1B_MIPI_MONO_SensorInit},
#endif
#if defined(OV02B10_MIPI_RAW_CARR)
	{CARR_OV02B10_SENSOR_ID, SENSOR_DRVNAME_OV02B10_MIPI_RAW_CARR, CARR_OV02B10_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M1_MIPI_RAW_CARR)
	{CARR_GC02M1_SENSOR_ID, SENSOR_DRVNAME_GC02M1_MIPI_RAW_CARR, CARR_GC02M1_MIPI_RAW_SensorInit},
#endif
    /* ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_21037[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV13B10_MIPI_RAW_CARR)
	{CARR_OV13B10_SENSOR_ID, SENSOR_DRVNAME_OV13B10_MIPI_RAW_CARR, CARR_OV13B10_MIPI_RAW_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW_CARR)
	{CARR_IMX355_SENSOR_ID, SENSOR_DRVNAME_IMX355_MIPI_RAW_CARR, CARR_IMX355_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO_CARR)
	{CARR_GC02M1B_SENSOR_ID, SENSOR_DRVNAME_GC02M1B_MIPI_MONO_CARR, CARR_GC02M1B_MIPI_MONO_SensorInit},
#endif
    /* ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_21041[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(S5KGM1ST_MIPI_RAW_ODINA)
	{S5KGM1ST_SENSOR_ID_ODINA, SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_ODINA, S5KGM1ST_MIPI_RAW_ODINA_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW_ODINA)
	{IMX355_SENSOR_ID_ODINA, SENSOR_DRVNAME_IMX355_MIPI_RAW_ODINA, IMX355_MIPI_RAW_ODINA_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO_ODINA)
	{GC02M1B_SENSOR_ID_ODINA, SENSOR_DRVNAME_GC02M1B_MIPI_MONO_ODINA, GC02M1B_MIPI_MONO_ODINA_SensorInit},
#endif
#if defined(OV02B10_MIPI_RAW_ODINA)
	{OV02B10_SENSOR_ID_ODINA, SENSOR_DRVNAME_OV02B10_MIPI_RAW_ODINA, OV02B10_MIPI_RAW_ODINA_SensorInit},
#endif
    /* ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_216A0[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(S5KGM1ST_MIPI_RAW_APOLLOF)
	{S5KGM1ST_SENSOR_ID_APOLLOF, SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_APOLLOF, S5KGM1ST_MIPI_RAW_APOLLOF_SensorInit},
#endif
#if defined(S5K3P9SP_MIPI_RAW_APOLLOF)
	{S5K3P9SP_SENSOR_ID_APOLLOF, SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW_APOLLOF, S5K3P9SP_MIPI_RAW_APOLLOF_SensorInit},
#endif
#if defined(OV02B10_MIPI_RAW_APOLLOF)
	{OV02B10_SENSOR_ID_APOLLOF, SENSOR_DRVNAME_OV02B10_MIPI_RAW_APOLLOF, OV02B10_MIPI_RAW_APOLLOF_SensorInit},
#endif
#if defined(OV02B1B_MIPI_MONO_APOLLOF)
	{OV02B1B_SENSOR_ID_APOLLOF, SENSOR_DRVNAME_OV02B1B_MIPI_MONO_APOLLOF, OV02B1B_MIPI_MONO_APOLLOF_SensorInit},
#endif
#if defined(OV64B_MIPI_RAW_APOLLOF)
	{OV64B_SENSOR_ID_APOLLOF, SENSOR_DRVNAME_OV64B_MIPI_RAW_APOLLOF, OV64B_MIPI_RAW_APOLLOF_SensorInit},
#endif
	 /* ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_22610[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV64B40_MIPI_RAW_HAWAII)
	{OV64B40_SENSOR_ID_HAWAII, SENSOR_DRVNAME_OV64B40_MIPI_RAW_HAWAII, OV64B40_MIPI_RAW_HAWAII_SensorInit},
#endif
#if defined(OV08D10_MIPI_RAW_HAWAII)
	{OV08D10_SENSOR_ID_HAWAII, SENSOR_DRVNAME_OV08D10_MIPI_RAW_HAWAII, OV08D10_MIPI_RAW_HAWAII_SensorInit},
#endif
#if defined(OV16A1Q_MIPI_RAW_HAWAII)
	{OV16A1Q_SENSOR_ID_HAWAII, SENSOR_DRVNAME_OV16A1Q_MIPI_RAW_HAWAII, OV16A1Q_MIPI_RAW_HAWAII_SensorInit},
#endif
#if defined(OV02B1B_MIPI_MONO_HAWAII)
	{OV02B1B_SENSOR_ID_HAWAII, SENSOR_DRVNAME_OV02B1B_MIPI_MONO_HAWAII, OV02B1B_MIPI_MONO_HAWAII_SensorInit},
#endif
	 /* ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_alicert[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(S5K3L6_MIPI_RAW_ALICERT)
    {S5K3L6_SENSOR_ID_ALICERT, SENSOR_DRVNAME_S5K3L6_MIPI_RAW_ALICERT, S5K3L6_MIPI_RAW_ALICERT_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW_ALICERT)
    {IMX355_SENSOR_ID_ALICERT, SENSOR_DRVNAME_IMX355_MIPI_RAW_ALICERT, IMX355_MIPI_RAW_ALICERT_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO_ALICERT)
    {GC02M1B_SENSOR_ID_ALICERT, SENSOR_DRVNAME_GC02M1B_MIPI_MONO_ALICERT, GC02M1B_MIPI_MONO_ALICERT_SensorInit},
#endif
#if defined(S5KGM1ST_MIPI_RAW_ALICER)
    {S5KGM1ST_SENSOR_ID_ALICER, SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_ALICER, S5KGM1ST_MIPI_RAW_ALICER_SensorInit},
#endif
    /* ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_aliceq[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(S5K3L6_MIPI_RAW_ALICERT)
    {S5K3L6_SENSOR_ID_ALICERT, SENSOR_DRVNAME_S5K3L6_MIPI_RAW_ALICERT, S5K3L6_MIPI_RAW_ALICERT_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW_ALICER)
    {IMX355_SENSOR_ID_ALICER, SENSOR_DRVNAME_IMX355_MIPI_RAW_ALICER, IMX355_MIPI_RAW_ALICER_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO_ALICERT)
    {GC02M1B_SENSOR_ID_ALICERT, SENSOR_DRVNAME_GC02M1B_MIPI_MONO_ALICERT, GC02M1B_MIPI_MONO_ALICERT_SensorInit},
#endif
#if defined(S5KGM1ST_MIPI_RAW_ALICER)
    {S5KGM1ST_SENSOR_ID_ALICER, SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_ALICER, S5KGM1ST_MIPI_RAW_ALICER_SensorInit},
#endif
    /* ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_alicer[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(S5KJN103_MIPI_RAW_ALICER)
    {S5KJN103_SENSOR_ID_ALICER, SENSOR_DRVNAME_S5KJN103_MIPI_RAW_ALICER, S5KJN103_MIPI_RAW_ALICER_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW_ALICER)
    {IMX355_SENSOR_ID_ALICER, SENSOR_DRVNAME_IMX355_MIPI_RAW_ALICER, IMX355_MIPI_RAW_ALICER_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO_ALICER)
    {GC02M1B_SENSOR_ID_ALICER, SENSOR_DRVNAME_GC02M1B_MIPI_MONO_ALICER, GC02M1B_MIPI_MONO_ALICER_SensorInit},
#endif
#if defined(S5KGM1ST_MIPI_RAW_ALICER)
    {S5KGM1ST_SENSOR_ID_ALICER, SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_ALICER, S5KGM1ST_MIPI_RAW_ALICER_SensorInit},
#endif
    /* ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};


struct IMGSENSOR_SENSOR_LIST gimgsensor_sensor_list_20611[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(S5KGM1ST_MIPI_RAW_20611)
    {S5KGM1ST_SENSOR_ID_20611, SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_20611, S5KGM1ST_MAIN_MIPI_RAW_20611_SensorInit},
#endif
//#if defined(OV16A10_MIPI_RAW_20611)
//    {OV16A10_SENSOR_ID_20611, SENSOR_DRVNAME_OV16A10_MIPI_RAW_20611, OV16A10_MAIN_MIPI_RAW_20611_SensorInit},
//#endif
#if defined(S5K3P9SP_MIPI_RAW_20611)
	{S5K3P9SP_SENSOR_ID_20611, SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW_20611, S5K3P9SP_MIPI_RAW_20611_SensorInit},
#endif
#if defined(HI846_MIPI_RAW_20611)
	{HI846_SENSOR_ID_20611, SENSOR_DRVNAME_HI846_MIPI_RAW_20611, HI846_MIPI_RAW_20611_SensorInit},
#endif
#if defined(GC02K_MIPI_RAW_20611)
	{GC02K_SENSOR_ID_20611, SENSOR_DRVNAME_GC02K_MIPI_RAW_20611, GC02K_MIPI_RAW_20611_SensorInit},
#endif
#if defined(OV02B1B_MIPI_RAW_20611)
	{OV02B1B_SENSOR_ID_20611, SENSOR_DRVNAME_OV02B1B_MIPI_RAW_20611, OV02B1B_MIPI_RAW_20611_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};
struct IMGSENSOR_SENSOR_LIST gimgsensor_sensor_list_athensd[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(S5KGW3_MIPI_RAW_ATHENSD)
    {ATHENSD_S5KGW3_SENSOR_ID, SENSOR_DRVNAME_S5KGW3_MIPI_RAW_ATHENSD, S5KGW3_MIPI_RAW_ATHENSD_SensorInit},
#endif
#if defined(S5KGW3P1_MIPI_RAW_ATHENSD)
    {ATHENSD_S5KGW3P1_SENSOR_ID, SENSOR_DRVNAME_S5KGW3P1_MIPI_RAW_ATHENSD, S5KGW3P1_MIPI_RAW_ATHENSD_SensorInit},
#endif
#if defined(IMX471_MIPI_RAW_ATHENSD)
    {ATHENSD_IMX471_SENSOR_ID, SENSOR_DRVNAME_IMX471_MIPI_RAW_ATHENSD, IMX471_MIPI_RAW_ATHENSD_SensorInit},
#endif
#if defined(HI846_MIPI_RAW_ATHENSD)
    {ATHENSD_HI846_SENSOR_ID, SENSOR_DRVNAME_HI846_MIPI_RAW_ATHENSD, HI846_MIPI_RAW_ATHENSD_SensorInit},
#endif
#if defined(GC02K_MIPI_RAW_ATHENSD)
    {ATHENSD_GC02K_SENSOR_ID, SENSOR_DRVNAME_GC02K_MIPI_RAW_ATHENSD, GC02K_MIPI_RAW_ATHENSD_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};
struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_blade[MAX_NUM_OF_SUPPORT_SENSOR] = {

    #if defined(HI1336_MIPI_RAW_BLADE)
    {HI1336_SENSOR_ID_BLADE, SENSOR_DRVNAME_HI1336_MIPI_RAW_BLADE, HI1336_MIPI_RAW_BLADE_SensorInit},
    #endif

    #if defined(HI556_MIPI_RAW_BLADE)
    {HI556_SENSOR_ID_BLADE, SENSOR_DRVNAME_HI556_MIPI_RAW_BLADE, HI556_MIPI_RAW_BLADE_SensorInit},
    #endif
    #if defined(SC201CS_MIPI_MONO_BLADE)
    {SC201CS_SENSOR_ID_BLADE, SENSOR_DRVNAME_SC201CS_MIPI_MONO_BLADE, SC201CS_MIPI_RAW_BLADE_SensorInit},
    #endif
    #if defined(C2599_MIPI_MONO_BLADE)
    {C2599_SENSOR_ID_BLADE, SENSOR_DRVNAME_C2599_MIPI_MONO_BLADE, C2599_MIPI_RAW_BLADE_SensorInit},
    #endif
    #if defined(BF20A1_MIPI_RAW_BLADE)
    {BF20A1_SENSOR_ID_BLADE, SENSOR_DRVNAME_BF20A1_MIPI_RAW_BLADE, BF20A1_MIPI_RAW_BLADE_SensorInit},
    #endif
    #if defined(GC030A_MIPI_RAW_BLADE)
    {GC030A_SENSOR_ID_BLADE, SENSOR_DRVNAME_GC030A_MIPI_RAW_BLADE, GC030A_MIPI_RAW_BLADE_SensorInit},
    #endif
    #if defined(HI5021SQT_MIPI_RAW_BLADE)
    {HI5021SQT_SENSOR_ID_BLADE, SENSOR_DRVNAME_HI5021SQT_MIPI_RAW_BLADE, HI5021SQT_MIPI_RAW_BLADE_SensorInit},
    #endif
    #if defined(S5KJN103_MIPI_RAW_BLADE)
    {S5KJN103_SENSOR_ID_BLADE, SENSOR_DRVNAME_S5KJN103_MIPI_RAW_BLADE,S5KJN103_MIPI_RAW_BLADE_SensorInit},
    #endif
    #if defined(SC800CS_MIPI_RAW_BLADE)
    {SC800CS_SENSOR_ID_BLADE, SENSOR_DRVNAME_SC800CS_MIPI_RAW_BLADE, SC800CS_MIPI_RAW_BLADE_SensorInit},
    #endif
    #if defined(SC201CS_MIPI_RAW_BLADE)
    {SC201CS1_SENSOR_ID_BLADE, SENSOR_DRVNAME_SC201CS_MIPI_RAW_BLADE, SC201CS1_MIPI_RAW_BLADE_SensorInit},
    #endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};
struct IMGSENSOR_HW_CFG imgsensor_custom_config_athensd[] = {
	{
		IMGSENSOR_SENSOR_IDX_MAIN,
		IMGSENSOR_I2C_DEV_0,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
			//{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			//{IMGSENSOR_HW_DVDD_SEL, IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
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
			//{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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
			//{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_SUB2,
		IMGSENSOR_I2C_DEV_3,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			//{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
			//{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
		},
	},
	{IMGSENSOR_SENSOR_IDX_NONE}
};
struct IMGSENSOR_HW_CFG imgsensor_custom_config_blade[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD_1, IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD_1, IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD_1, IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD_1, IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_athensd[] = {
#if defined(S5KGW3_MIPI_RAW_ATHENSD)
	{
		SENSOR_DRVNAME_S5KGW3_MIPI_RAW_ATHENSD,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1100, 1},
			//{DVDDSEL, Vol_Low, 0},// low to output 1.05v,high to output 1.1v, datasheet request 1.05v
			{AVDD, Vol_2800, 1},
			{AF_VDD, Vol_2800, 0},
			{RST, Vol_High, 2},
			{SensorMCLK, Vol_High, 5},
		},
	},
#endif
#if defined(S5KGW3P1_MIPI_RAW_ATHENSD)
	{
		SENSOR_DRVNAME_S5KGW3P1_MIPI_RAW_ATHENSD,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1100, 1},
			//{DVDDSEL, Vol_Low, 0},// low to output 1.05v,high to output 1.1v, datasheet request 1.05v
			{AVDD, Vol_2800, 1},
			{AF_VDD, Vol_2800, 0},
			{RST, Vol_High, 2},
			{SensorMCLK, Vol_High, 5},
		},
	},
#endif
#if defined(IMX471_MIPI_RAW_ATHENSD)
	{
		SENSOR_DRVNAME_IMX471_MIPI_RAW_ATHENSD,
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
#if defined(HI846_MIPI_RAW_ATHENSD)
		{
			SENSOR_DRVNAME_HI846_MIPI_RAW_ATHENSD,
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
#if defined(GC02K_MIPI_RAW_ATHENSD)
	{
		SENSOR_DRVNAME_GC02K_MIPI_RAW_ATHENSD,
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
	/* add new sensor before this line */
	{NULL,},
};
struct IMGSENSOR_HW_POWER_SEQ platform_power_sequence_athensd[] ={
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
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_blade[] = {

#if defined(HI1336_MIPI_RAW_BLADE)
    {
        SENSOR_DRVNAME_HI1336_MIPI_RAW_BLADE,
        {
                    {RST, Vol_Low, 1},
                    {DOVDD, Vol_1800, 2},
                    {AF_VDD, Vol_2800, 2},
                    {AVDD, Vol_2800, 2},
                    {DVDD, Vol_1100, 2},
                    {SensorMCLK, Vol_High, 2},
                    {RST, Vol_High, 1},
        },
    },
#endif

#if defined(HI556_MIPI_RAW_BLADE)
    {
        SENSOR_DRVNAME_HI556_MIPI_RAW_BLADE,
        {
            {PDN, Vol_High, 1},
            {DVDD_1, Vol_High, 5},
            {RST, Vol_Low, 0},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_High, 1},
            {DVDD, Vol_High, 2},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(BF20A1_MIPI_RAW_BLADE)
    {
        SENSOR_DRVNAME_BF20A1_MIPI_RAW_BLADE,
        {
            {PDN, Vol_High, 1},
            {DVDD_1, Vol_High, 5},
            {RST, Vol_Low, 0},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_High, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 1},
            {RST, Vol_Low, 1},
        },
    },
#endif
#if defined(GC030A_MIPI_RAW_BLADE)
    {
            SENSOR_DRVNAME_GC030A_MIPI_RAW_BLADE,
            {
            {RST, Vol_Low, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {AVDD, Vol_High, 1},
            {RST, Vol_High, 1},
            {RST, Vol_Low, 1},
            },
   },
#endif
#if defined(SC800CS_MIPI_RAW_BLADE)
    {
        SENSOR_DRVNAME_SC800CS_MIPI_RAW_BLADE,
        {
            {PDN, Vol_High, 1},
            {DVDD_1, Vol_High, 5},
            {RST, Vol_Low, 0},
            {DOVDD, Vol_2800, 1},
            {DVDD, Vol_High, 2},
            {AVDD, Vol_High, 1},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(SC201CS_MIPI_MONO_BLADE)
    {
        SENSOR_DRVNAME_SC201CS_MIPI_MONO_BLADE,
        {
            {RST, Vol_Low, 0},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_High, 1},
            {RST, Vol_High, 2},
            {RST, Vol_Low, 2},
            {RST, Vol_High, 2},
            {SensorMCLK, Vol_High, 5},
        },
    },
#endif
#if defined(C2599_MIPI_MONO_BLADE)
    {
        SENSOR_DRVNAME_C2599_MIPI_MONO_BLADE,
        {
            {PDN, Vol_High, 1},
            {DVDD_1, Vol_High, 5},
            {RST, Vol_Low, 0},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_High, 1},
            {RST, Vol_High, 5},
            {SensorMCLK, Vol_High, 1},
        },
    },
#endif
#if defined(HI5021SQT_MIPI_RAW_BLADE)
    {
        SENSOR_DRVNAME_HI5021SQT_MIPI_RAW_BLADE,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 2},
            {DVDD, Vol_High, 2},
            {DVDD_1, Vol_High, 2},
            {PDN, Vol_High, 2},
            {AVDD, Vol_High, 2},
            {RST, Vol_High, 2},
            {PDN, Vol_Low, 1},
            {AF_VDD, Vol_2800, 2},
            {SensorMCLK, Vol_High, 2},
        },
    },
#endif
#if defined(S5KJN103_MIPI_RAW_BLADE)
    {
        SENSOR_DRVNAME_S5KJN103_MIPI_RAW_BLADE,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 0},
            {DVDD, Vol_High, 1},
            {DVDD_1, Vol_High, 2},
            {PDN, Vol_High, 2},
            {AVDD, Vol_High, 5},
            {AF_VDD, Vol_2800, 0},
            {RST, Vol_High, 2},
            {PDN, Vol_Low, 1},
            {SensorMCLK, Vol_High, 10},
        },
    },
#endif
#if defined(S5KJN103_MIPI_RAW_BLADE)
    {
        SENSOR_DRVNAME_S5KJN103_MIPI_RAW_BLADE,
        {
                    {RST, Vol_Low, 1},
                    {DOVDD, Vol_1800, 0},
                    {DVDD, Vol_High, 1},
                    {DVDD_1, Vol_High, 2},
                    {PDN, Vol_High, 2},
                    {AVDD, Vol_High, 5},
                    {AF_VDD, Vol_2800, 0},
                    {RST, Vol_High, 2},
                    {PDN, Vol_Low, 1},
                    {SensorMCLK, Vol_High, 10},
        },
    },
#endif
#if defined(SC201CS_MIPI_RAW_BLADE)
    {
        SENSOR_DRVNAME_SC201CS_MIPI_RAW_BLADE,
        {
            {PDN, Vol_High, 1},
            {DVDD_1, Vol_High, 5},
            {RST, Vol_Low, 0},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_High, 1},
            {RST, Vol_High, 2},
            {RST, Vol_Low, 2},
            {RST, Vol_High, 2},
            {SensorMCLK, Vol_High, 5},
        },
    },
#endif

    /* add new sensor before this line */
    {NULL,},
};
struct CAMERA_DEVICE_INFO gImgEepromInfo_athensd = {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {ATHENSD_S5KGW3_SENSOR_ID,      0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5kgw3_mipi_raw_athensd"},
        {ATHENSD_S5KGW3P1_SENSOR_ID,    0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5kgw3p1_mipi_raw_athensd"},
        {ATHENSD_IMX471_SENSOR_ID,      0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx471_mipi_raw_athensd"},
        {ATHENSD_HI846_SENSOR_ID,       0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "hi846_mipi_raw_athensd"},
        {ATHENSD_GC02K_SENSOR_ID,       0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r3", "gc02k_mipi_raw_athensd"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {S5KGW3_STEREO_START_ADDR_ATHENSD, HI846_STEREO_START_ADDR_ATHENSD},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20181[] = {
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
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_23031[] = {
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
            {IMGSENSOR_HW_PIN_DVDD_1,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_22277[] = {
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
            {IMGSENSOR_HW_PIN_DVDD_1,  IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_23231[] = {
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
            {IMGSENSOR_HW_PIN_DVDD_1,  IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_SUB3,
        IMGSENSOR_I2C_DEV_5,
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
        IMGSENSOR_SENSOR_IDX_MAIN4,
        IMGSENSOR_I2C_DEV_6,
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

    {IMGSENSOR_SENSOR_IDX_NONE}
};


struct IMGSENSOR_HW_CFG imgsensor_custom_config_20151[] = {
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
                    {IMGSENSOR_HW_PIN_MIPI_SWITCH_EN, IMGSENSOR_HW_ID_GPIO},
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
                    {IMGSENSOR_HW_PIN_MIPI_SWITCH_EN, IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
            },
    },
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20630[] = {
    {
            IMGSENSOR_SENSOR_IDX_MAIN,
            IMGSENSOR_I2C_DEV_0,
            {
                    {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
                    {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
                    {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
                    {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
                    //{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
                    #ifdef OPLUS_FEATURE_CAMERA_COMMON
                    //{IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL, IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
                    #endif
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
                    {IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL, IMGSENSOR_HW_ID_GPIO},
                    //{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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
                    {IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL, IMGSENSOR_HW_ID_GPIO},
                    //{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
            },
    },
    {
            IMGSENSOR_SENSOR_IDX_SUB2,
            IMGSENSOR_I2C_DEV_3,
            {
                    {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
                    {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
                    {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
                    //{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
                    //{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
            },
    },
    {
            IMGSENSOR_SENSOR_IDX_MAIN3,
            IMGSENSOR_I2C_DEV_4,
            {
                    {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
                    {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
                    {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
                    //{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
                    //{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
            },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20633[] = {
    {
            IMGSENSOR_SENSOR_IDX_MAIN,
            IMGSENSOR_I2C_DEV_0,
            {
                    {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
                    {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
                    {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
                    {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
                    //{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
                    #ifdef OPLUS_FEATURE_CAMERA_COMMON
                    //{IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL, IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
                    #endif
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
                    {IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL, IMGSENSOR_HW_ID_GPIO},
                    //{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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
                    {IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL, IMGSENSOR_HW_ID_GPIO},
                    //{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
            },
    },
    {
            IMGSENSOR_SENSOR_IDX_SUB2,
            IMGSENSOR_I2C_DEV_3,
            {
                    {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
                    {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
                    {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
                    //{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
                    //{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
            },
    },
    {
            IMGSENSOR_SENSOR_IDX_MAIN3,
            IMGSENSOR_I2C_DEV_4,
            {
                    {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
                    {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
                    {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
                    //{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
                    //{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
            },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20001[] = {
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

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20391[] = {
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
                    {IMGSENSOR_HW_PIN_MIPI_SWITCH_EN, IMGSENSOR_HW_ID_GPIO},
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
                    {IMGSENSOR_HW_PIN_MIPI_SWITCH_EN, IMGSENSOR_HW_ID_GPIO},
                    {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
            },
    },
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_22629[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
            //{IMGSENSOR_HW_PIN_VOIS,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD_1, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD_1, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_VOIS,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_DVDD_1,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD_1,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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

struct IMGSENSOR_HW_CFG imgsensor_custom_config_22633[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
            // {IMGSENSOR_HW_PIN_VOIS,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD_1, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD_1, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_VOIS,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_DVDD_1,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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

struct IMGSENSOR_HW_CFG imgsensor_custom_config_22693[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
            //{IMGSENSOR_HW_PIN_VOIS,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD_1, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD_1, IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_DVDD_1,  IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD_1,  IMGSENSOR_HW_ID_REGULATOR},
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

struct IMGSENSOR_HW_CFG imgsensor_custom_config_23707[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD1, IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_VOIS,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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

struct IMGSENSOR_HW_CFG imgsensor_custom_config_23687[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_GPIO},
            //{IMGSENSOR_HW_PIN_VOIS,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD1, IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_VOIS,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_MAIN4,
        IMGSENSOR_I2C_DEV_4,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_GPIO},
            //{IMGSENSOR_HW_PIN_VOIS,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_VOIS,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_moss[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
            //{IMGSENSOR_HW_PIN_VOIS,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD_1,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD_1,   IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD_1,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
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
            {IMGSENSOR_HW_PIN_DVDD_1,  IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_mossa[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DVDD_1,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD_1,   IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_DVDD_1,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
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
            {IMGSENSOR_HW_PIN_DVDD_1,  IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}
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

struct IMGSENSOR_HW_CFG imgsensor_custom_config_21081[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD1,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
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


struct IMGSENSOR_HW_CFG imgsensor_custom_config_216A0[] = {
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
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_22610[] = {
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
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_SUB,
        IMGSENSOR_I2C_DEV_1,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AVDD_1,  IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AVDD_1,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_alicert[] = {
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
            {IMGSENSOR_HW_PIN_AFVDD,   IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_aliceq[] = {
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
            {IMGSENSOR_HW_PIN_AFVDD,   IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_alicer[] = {
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
            {IMGSENSOR_HW_PIN_AFVDD,   IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20611[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            //{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL, IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
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
            //{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
            //{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_SUB2,
        IMGSENSOR_I2C_DEV_3,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            //{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            //{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
        },
    },
    {
        IMGSENSOR_SENSOR_IDX_MAIN3,
        IMGSENSOR_I2C_DEV_4,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            //{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
            //{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE, IMGSENSOR_HW_ID_NONE},
        },
    },

    {IMGSENSOR_SENSOR_IDX_NONE}

};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20181[] = {
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
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_23031[] = {
#if defined(OV64B_MIPI_RAW23031)
    {
        SENSOR_DRVNAME_OV64B_MIPI_RAW23031,
        {
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 1},
            {AVDD, Vol_2800, 2},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 5},
            {AFVDD, Vol_2800, 2},
            {RST, Vol_High, 6},
        },
    },
#endif
#if defined(OV08D_MIPI_RAW23031)
    {
        SENSOR_DRVNAME_OV08D_MIPI_RAW23031,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 1},
            {DVDD, Vol_1100, 6},
            {RST, Vol_High, 1},
            {SensorMCLK, Vol_High, 8},
        },
    },
#endif
#if defined(GC02M1_MIPI_RAW23031)
    {
        SENSOR_DRVNAME_GC02M1_MIPI_RAW23031,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_22277[] = {
#if defined(OV64B_MIPI_RAW22277)
    {
        SENSOR_DRVNAME_OV64B_MIPI_RAW22277,
        {
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 1},
            {AVDD, Vol_2800, 2},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 5},
            {AFVDD, Vol_2800, 2},
            {RST, Vol_High, 6},
        },
    },
#endif
#if defined(OV32C_MIPI_RAW22277)
    {
        SENSOR_DRVNAME_OV32C_MIPI_RAW22277,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 1},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 6}
        },
    },
#endif
#if defined(IMX355_MIPI_RAW22277)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW22277,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 1},
            {DVDD, Vol_1050, 1},
            {DVDD_1, Vol_1200, 2},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(IMX709_MIPI_RAW22277)
    {
        SENSOR_DRVNAME_IMX709_MIPI_RAW22277,
        {
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_Low, 1},
            {AVDD, Vol_2900, 0},
            {DVDD, Vol_1000, 0},
            {DOVDD, Vol_1800, 0},
            {AFVDD, Vol_2800, 2},
            {RST, Vol_High, 8},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_23231[] = {
#if defined(IMX882_MIPI_RAW23231)
    {
        SENSOR_DRVNAME_IMX882_MIPI_RAW23231,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 2},
            {DVDD, Vol_1100, 5},
            {AFVDD, Vol_2800, 2},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 6},
        },
    },
#endif
#if defined(OV32C_MIPI_RAW23231)
    {
        SENSOR_DRVNAME_OV32C_MIPI_RAW23231,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 1},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 6}
        },
    },
#endif
#if defined(IMX355_MIPI_RAW23231)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW23231,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 1},
            {DVDD, Vol_1050, 1},
            {DVDD_1, Vol_1200, 2},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(IMX882PD_MIPI_RAW23231)
    {
        SENSOR_DRVNAME_IMX882PD_MIPI_RAW23231,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 2},
            {DVDD, Vol_1100, 5},
            {AFVDD, Vol_2800, 2},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 6},
        },
    },
#endif
#if defined(IMX709_MIPI_RAW23231)
    {
        SENSOR_DRVNAME_IMX709_MIPI_RAW23231,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2900, 1},
            {DVDD, Vol_800, 1},
            {AFVDD, Vol_2800, 2},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 8},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_on_sequence_23231[] = {
#if defined(IMX882_MIPI_RAW23231)
    {
        SENSOR_DRVNAME_IMX882_MIPI_RAW23231,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {DOVDD, Vol_Low, 4},
            {DOVDD, Vol_1800, 1},
            {DOVDD, Vol_Low, 4},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 2},
            {DVDD, Vol_1100, 5},
            {AFVDD, Vol_2800, 2},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 6},
        },
    },
#endif
#if defined(OV32C_MIPI_RAW23231)
    {
        SENSOR_DRVNAME_OV32C_MIPI_RAW23231,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 1},
            {DVDD, Vol_1100, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 6}
        },
    },
#endif
#if defined(IMX355_MIPI_RAW23231)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW23231,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {DOVDD, Vol_Low, 4},
            {DOVDD, Vol_1800, 1},
            {DOVDD, Vol_Low, 4},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 1},
            {DVDD, Vol_1050, 1},
            {DVDD_1, Vol_1200, 2},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(IMX882PD_MIPI_RAW23231)
    {
        SENSOR_DRVNAME_IMX882PD_MIPI_RAW23231,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2900, 1},
            {DVDD, Vol_800, 1},
            {AFVDD, Vol_2800, 2},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 8},
        },
    },
#endif
#if defined(IMX709_MIPI_RAW23231)
    {
        SENSOR_DRVNAME_IMX709_MIPI_RAW23231,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {DOVDD, Vol_Low, 4},
            {DOVDD, Vol_1800, 1},
            {DOVDD, Vol_Low, 4},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2900, 1},
            {DVDD, Vol_800, 1},
            {AFVDD, Vol_2800, 2},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 8},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20151[] = {
#if defined(OV48B_MIPI_RAW)
    {
        SENSOR_DRVNAME_OV48B_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 0},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1200, 5},
            // {AFVDD, Vol_2800, 2},
            {SensorMCLK, Vol_High, 0},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(S5KGM1ST_MIPI_RAW)
    {
        SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DOVDD, Vol_1800, 0},
            {DVDD, Vol_1100, 1},
            {RST, Vol_High, 1},
            {SensorMCLK, Vol_High, 2},
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
#if defined(IMX471_MIPI_RAW)
    {
        SENSOR_DRVNAME_IMX471_MIPI_RAW,
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
#if defined(OV02B10_MIPI_RAW)
    {
        SENSOR_DRVNAME_OV02B10_MIPI_RAW,
        {
            {IMGSENSOR_HW_PIN_MIPI_SWITCH_EN, Vol_Low, 10},
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 5},
            {AVDD, Vol_2800, 10},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(OV02A1B_MIPI_MONO)
    {
        SENSOR_DRVNAME_OV02A1B_MIPI_MONO,
        {
            {IMGSENSOR_HW_PIN_MIPI_SWITCH_EN, Vol_High, 10},
            {RST, Vol_High, 1, Vol_Low, 1},
            {DOVDD, Vol_1800, 0},
            {AVDD, Vol_2800, 0},
            {RST, Vol_Low, 9, Vol_High, 9},
            {SensorMCLK, Vol_High, 5},
        },
    },
#endif
#if defined(OV02B1B_MIPI_MONO)
    {
        SENSOR_DRVNAME_OV02B1B_MIPI_MONO,
        {
            {IMGSENSOR_HW_PIN_MIPI_SWITCH_EN, Vol_High, 10},
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 0},
            {AVDD, Vol_2800, 10},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 5},
        },
    },
#endif

    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ platform_power_sequence_20630[] ={
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
        IMGSENSOR_SENSOR_IDX_SUB,
    },
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
        IMGSENSOR_SENSOR_IDX_MAIN2,
    },
};

struct IMGSENSOR_HW_POWER_SEQ platform_power_sequence_20633[] ={
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
        IMGSENSOR_SENSOR_IDX_SUB,
    },
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
        IMGSENSOR_SENSOR_IDX_MAIN2,
    },
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20630[] = {
#if defined(S5KGW3_MIPI_RAW_ATHENSB)
        {
            SENSOR_DRVNAME_S5KGW3_MIPI_RAW_ATHENSB,
            {
                {RST, Vol_Low, 1},
                {DOVDD, Vol_1800, 1},
                {DVDD, Vol_1100, 1},
                {AVDD, Vol_2800, 1},
                {AF_VDD, Vol_2800, 0},
                {RST, Vol_High, 2},
                {SensorMCLK, Vol_High, 5},
            },
        },
#endif
#if defined(OV32A1Q_MIPI_RAW_ATHENSB)
        {
            SENSOR_DRVNAME_OV32A1Q_MIPI_RAW_ATHENSB,
            {
                {RST, Vol_Low, 1},
                {AVDD, Vol_2800, 0},
                {DVDD, Vol_1100, 0},
                {DOVDD, Vol_1800, 1},
                {MIPISEL, Vol_High, 3},
                {SensorMCLK, Vol_High, 1},
                {RST, Vol_High, 2},
            },
        },
#endif
#if defined(HI846_MIPI_RAW_ATHENSB)
        {
            SENSOR_DRVNAME_HI846_MIPI_RAW_ATHENSB,
            {
                {RST, Vol_Low, 1},
                {AVDD, Vol_2800, 0},
                {DVDD, Vol_1200, 0},
                {DOVDD, Vol_1800, 1},
                {MIPISEL, Vol_Low, 3},
                {SensorMCLK, Vol_High, 1},
                {RST, Vol_High, 2},
            },
        },
#endif
#if defined(GC02K_MIPI_RAW_ATHENSB)
        {
            SENSOR_DRVNAME_GC02K_MIPI_RAW_ATHENSB,
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
#if defined(GC02M1B_MIPI_RAW_ATHENSB)
        {
            SENSOR_DRVNAME_GC02M1B_MIPI_RAW_ATHENSB,
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

    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20633[] = {
#if defined(S5KGM1ST_MIPI_RAW_ATHENSC)
        {
            SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_ATHENSC,
            {
                {RST, Vol_Low, 1},
                {AVDD, Vol_2800, 0},
                {DVDD, Vol_1500, 0},
                {DOVDD, Vol_1800, 1},
                {AF_VDD, Vol_2800, 0},
                {RST, Vol_High, 1},
                {SensorMCLK, Vol_High, 1},

            },
        },
#endif
#if defined(IMX471_MIPI_RAW_ATHENSC)
        {
            SENSOR_DRVNAME_IMX471_MIPI_RAW_ATHENSC,
            {
                {RST, Vol_Low, 1},
                {AVDD, Vol_2800, 0},
                {DVDD, Vol_1100, 0},
                {DOVDD, Vol_1800, 1},
                {MIPISEL, Vol_High, 3},
                {SensorMCLK, Vol_High, 1},
                {RST, Vol_High, 2},
            },
        },
#endif
#if defined(HI846_MIPI_RAW_ATHENSC)
        {
            SENSOR_DRVNAME_HI846_MIPI_RAW_ATHENSC,
            {
                {RST, Vol_Low, 1},
                {AVDD, Vol_2800, 0},
                {DVDD, Vol_1200, 0},
                {DOVDD, Vol_1800, 1},
                {MIPISEL, Vol_Low, 3},
                {SensorMCLK, Vol_High, 1},
                {RST, Vol_High, 2},
            },
        },
#endif

#if defined(GC02K_MIPI_RAW_ATHENSB)
    {
        SENSOR_DRVNAME_GC02K_MIPI_RAW_ATHENSB,
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
#if defined(GC02M1B_MIPI_RAW_ATHENSB)
    {
        SENSOR_DRVNAME_GC02M1B_MIPI_RAW_ATHENSB,
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


    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20001[] = {
#if defined(OV16A10_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV16A10_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(S5K4H7_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K4H7_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1100, 1},
			{RST, Vol_High, 5},
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
#if defined(GC02M1B_MIPI_MONO)
		{
			SENSOR_DRVNAME_GC02M1B_MIPI_MONO,
			{
				{RST, Vol_Low, 1},
				{DOVDD, Vol_1800, 0},
				{AVDD, Vol_2800, 0},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
		},
#endif

    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20391[] = {
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
#if defined(OV48B_MIPI_RAW)
    {
        SENSOR_DRVNAME_OV48B_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 0},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1200, 5},
            // {AFVDD, Vol_2800, 2},
            {SensorMCLK, Vol_High, 0},
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
#if defined(OV02B10_MIPI_RAW)
    {
        SENSOR_DRVNAME_OV02B10_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 5},
            {AVDD, Vol_2800, 10},
            {RST, Vol_High, 2},
            {SensorMCLK, Vol_High, 1},
        },
    },
#endif

    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_23687[] = {
#if defined(IMX882_MIPI_RAW23687)
    {
        SENSOR_DRVNAME_IMX882_MIPI_RAW23687,
        {
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 1},
            {AVDD, Vol_2800, 2},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 5},
            {AFVDD, Vol_2800, 2},
            {RST, Vol_High, 6},
        },
    },
#endif
#if defined(IMX890_MIPI_RAW23689)
    {
        SENSOR_DRVNAME_IMX890_MIPI_RAW23689,
        {
            {VOIS , Vol_High, 2},
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 1},
            {AVDD, Vol_2800, 2},
            {AVDD1, Vol_1800, 2},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 5},
            {AFVDD, Vol_2800, 2},
            {RST, Vol_High, 6},
        },
    },
#endif
#if defined(HI1634Q_MIPI_RAW23687)
    {
        SENSOR_DRVNAME_HI1634Q_MIPI_RAW23687,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 1},
            {DVDD, Vol_1100, 1},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(HI846W_MIPI_RAW23687)
    {
        SENSOR_DRVNAME_HI846W_MIPI_RAW23687,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1200, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(GC02M1_MIPI_RAW23687)
    {
        SENSOR_DRVNAME_GC02M1_MIPI_RAW23687,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 5},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2}
        },
    },
#endif
#if defined(IMX882PD_MIPI_RAW23687)
    {
        SENSOR_DRVNAME_IMX882PD_MIPI_RAW23687,
        {
            {VOIS , Vol_High, 2},
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 1},
            {AVDD, Vol_2800, 2},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 5},
            {AFVDD, Vol_2800, 2},
            {RST, Vol_High, 6},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_22693= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {S5KHM6SP_SENSOR_ID22693,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5khm6sp"},
        {S5K3P9SP_SENSOR_ID22693,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "s5k3p9sp"},
        {IMX355_SENSOR_ID22693,     0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "imx355"},
        {GC02M1_SENSOR_ID22693, 0xA4, {0x00, 0x05}, 0xE0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02m1"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3230,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR, IMX355_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
    .i4DistortionAddr = IMX355_DISTORTIONPARAMS_START_ADDR,
};
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_22693[] = {
#if defined(IMX766_MIPI_RAW22693)
		{
			SENSOR_DRVNAME_IMX766_MIPI_RAW22693,
			{
				{RST, Vol_Low, 1},          //2     0
				{AF_VDD, Vol_2800, 1},       //11    10
				//{VOIS , Vol_High, 2},     //
				{AVDD_1, Vol_1800, 1},      //6     11
				{AVDD, Vol_2800, 1},        //3     11
				{DVDD_1, Vol_1100, 1},      //7     3
				{DOVDD, Vol_1800, 1},       //5     7
				{DVDD, Vol_1100, 1},        //4     11
				{SensorMCLK, Vol_High, 2},  //15    11
				{RST, Vol_High, 1},         //2     11
			},
		},
#endif
#if defined(S5KHM6SP_MIPI_RAW22693)
		{
			SENSOR_DRVNAME_S5KHM6SP_MIPI_RAW22693,
			{
				{RST, Vol_Low, 1},          //2     0
				{AFVDD, Vol_2800, 1},       //11    10
				//{VOIS , Vol_High, 2},     //
				//{AVDD_1, Vol_2800, 1},        //3     11
				//{DVDD_1, Vol_1200, 1},     //7     3
				{DOVDD, Vol_1800, 1},       //5     7
				{AVDD, Vol_1800, 1},      //6     11
				{DVDD, Vol_1100, 1},        //4     11
				{RST, Vol_High, 1},         //2     11
				{SensorMCLK, Vol_High, 20},  //15    11
			},
		},
#endif
#if defined(S5K3P9SP_MIPI_RAW22693)
		{
			SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW22693,
			{
				{DVDD_1, Vol_1100, 1},//1.05V   //7     11
				{RST, Vol_Low, 1},              //2     0
				{AVDD, Vol_2800, 1},            //3     11
				{DVDD, Vol_1100, 1},            //4     11
				{DOVDD, Vol_1800, 1},           //5     7
				{SensorMCLK, Vol_High, 1},      //15    11
				{RST, Vol_High, 2}              //2     11
			},
		},
#endif
#if defined(IMX355_MIPI_RAW22693)
		{
			SENSOR_DRVNAME_IMX355_MIPI_RAW22693,
			{
				{AVDD, Vol_2800, 1},
				{DVDD_1, Vol_High, 1},//1.2V
				{RST, Vol_Low, 1},
				{DVDD, Vol_1200, 1},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
		},
#endif
#if defined(GC02M1_MIPI_RAW22693)
    {
			SENSOR_DRVNAME_GC02M1_MIPI_RAW22693,
			{
				{RST, Vol_Low, 1},
				{DOVDD, Vol_1800, 1},
				{AVDD, Vol_2800, 5},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_23707[] = {
#if defined(S5KJN1_MIPI_RAW23707)
    {
        SENSOR_DRVNAME_S5KJN1_MIPI_RAW23707,
        {
            {PDN, Vol_High, 10},
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 5},
            {AVDD, Vol_2800, 2},
            {AFVDD, Vol_2800, 2},
            {RST, Vol_High, 2},
            {SensorMCLK, Vol_High, 10},
        },
    },
#endif
#if defined(HI1634Q_MIPI_RAW23707)
    {
        SENSOR_DRVNAME_HI1634Q_MIPI_RAW23707,
        {
            {PDN, Vol_High, 10},
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 1},
            {DVDD, Vol_1100, 1},
            {SensorMCLK, Vol_High, 5},
            {RST, Vol_High, 5},
        },
    },
#endif
#if defined(GC02M1_MIPI_RAW23707)
    {
        SENSOR_DRVNAME_GC02M1_MIPI_RAW23707,
        {
            {PDN, Vol_High, 10},
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 1},
            {RST, Vol_High, 3},
            {SensorMCLK, Vol_High, 1},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_22629[] = {
#if defined(S5KHP3SP_MIPI_RAW22629)
		{
			SENSOR_DRVNAME_S5KHP3SP_MIPI_RAW22629,
			{
				{PDN, Vol_High, 10},          //17     0
				{RST, Vol_Low, 10},          //2     0
				{AFVDD, Vol_2800, 1},       //11    10
				//{VOIS , Vol_High, 2},     //
				{AVDD, Vol_1800, 1},      //6     11
				{DVDD, Vol_1100, 3},        //4     11
				{DOVDD, Vol_1800, 3},       //5     7
				{RST, Vol_High, 10},         //2     11
				{SensorMCLK, Vol_High, 10},  //15    11
			},
		},
#endif
#if defined(IMX615_MIPI_RAW22629)
    {
        SENSOR_DRVNAME_IMX615_MIPI_RAW22629,
        {
            {PDN, Vol_High, 10},          //17     0
            {RST, Vol_Low, 1},
            {AVDD, Vol_2900, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined(IMX355_MIPI_RAW22629)
		{
			SENSOR_DRVNAME_IMX355_MIPI_RAW22629,
			{
				{PDN, Vol_High, 10},          //17     0
				{AVDD, Vol_2800, 1},
				{DVDD_1, Vol_High, 1},//1.2V
				{RST, Vol_Low, 1},
				{AVDD, Vol_2800, 1},
				{DVDD, Vol_1200, 1},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
		},
#endif
#if defined(GC02M1_MIPI_RAW22629)
    {
			SENSOR_DRVNAME_GC02M1_MIPI_RAW22629,
			{
				{RST, Vol_Low, 1},
				{AVDD, Vol_2800, 5},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_22629= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {S5KHP3SP_SENSOR_ID22629,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x94,0x94}, "Cam_r0", "s5khp3sp"},
        {IMX615_SENSOR_ID22629,     0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx615"},
        {IMX355_SENSOR_ID22629,     0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "imx355"},
        {GC02M1_SENSOR_ID22629,     0xA4, {0x00, 0x05}, 0xE0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02m1"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3230,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR, IMX355_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
    .i4DistortionAddr = IMX355_DISTORTIONPARAMS_START_ADDR,
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_22633= {
    .i4SensorNum = 2,
    .pCamModuleInfo = {
        {OVA0B4_SENSOR_ID22633,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x94,0x94}, "Cam_r0", "ova0b4"},
        {S5K3P9SP_SENSOR_ID22633,     0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "s5k3p9sp"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3230,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_moss[] = {
#if defined(IMX766_MOSSMIPI_RAW)
		{
			SENSOR_DRVNAME_IMX766_MOSSMIPI_RAW,
			{
				{RST, Vol_Low, 1},
				{AFVDD, Vol_2800, 1},
				//{VOIS , Vol_High, 2},
				{AVDD_1, Vol_High, 1},
				{AVDD, Vol_High, 1},
				{DVDD, Vol_High, 1},
				{DVDD_1, Vol_1200, 1},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 2},
				{RST, Vol_High, 1},
			},
		},
#endif
#if defined(IMX471_MOSSMIPI_RAW)
		{
			SENSOR_DRVNAME_IMX471_MOSSMIPI_RAW,
			{
				{DVDD_1, Vol_High, 1},//1.05V
				{RST, Vol_Low, 1},
				{AVDD, Vol_High, 1},
				{DVDD, Vol_High, 1},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
		},
#endif
#if defined(IMX355_MOSSMIPI_RAW)
		{
			SENSOR_DRVNAME_IMX355_MOSSMIPI_RAW,
			{
				{AVDD, Vol_Low, 1},
				{DVDD_1, Vol_High, 1},//1.2V
				{RST, Vol_Low, 1},
				{AVDD, Vol_High, 1},
				{DVDD, Vol_High, 1},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
		},
#endif
#if defined(GC02M1_MOSSMIPI_RAW)
    {
			SENSOR_DRVNAME_GC02M1_MOSSMIPI_RAW,
			{
				{RST, Vol_Low, 1},
				{DOVDD, Vol_1800, 1},
				{AVDD, Vol_High, 5},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_mossa[] = {
#if defined(S5KGM1ST_MOSSAMIPI_RAW)
    {
        SENSOR_DRVNAME_S5KGM1ST_MOSSAMIPI_RAW,
        {
				{RST, Vol_Low, 1},
				{AF_VDD, Vol_2800, 1},
				{AVDD, Vol_High, 1},
				{DVDD_1, Vol_1200, 1},
				{DVDD, Vol_High, 1},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 2},
				{RST, Vol_High, 1},
        },
    },
#endif
#if defined(IMX471_MOSSMIPI_RAW)
		{
			SENSOR_DRVNAME_IMX471_MOSSMIPI_RAW,
			{
				{DVDD_1, Vol_High, 1},//1.05V
				{RST, Vol_Low, 1},
				{AVDD, Vol_High, 1},
				{DVDD, Vol_High, 1},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
		},
#endif
#if defined(IMX355_MOSSMIPI_RAW)
		{
			SENSOR_DRVNAME_IMX355_MOSSMIPI_RAW,
			{
				{AVDD, Vol_Low, 1},
				{DVDD_1, Vol_High, 1},//1.2V
				{RST, Vol_Low, 1},
				{AVDD, Vol_High, 1},
				{DVDD, Vol_High, 1},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
		},
#endif
#if defined(GC02M1_MOSSAMIPI_RAW)
    {
			SENSOR_DRVNAME_GC02M1_MOSSAMIPI_RAW,
			{
				{RST, Vol_Low, 1},
				{DOVDD, Vol_1800, 1},
				{AVDD, Vol_High, 5},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_22633[] = {
#if defined(OVA0B4_MIPI_RAW22633)
    {
			SENSOR_DRVNAME_OVA0B4_MIPI_RAW22633,
			{
				{RST, Vol_Low, 1},          // 2     0
				{AFVDD, Vol_2800, 1},       // 11    10
				// {VOIS , Vol_High, 2},     //
				{AVDD, Vol_1800, 1},       // 6     11
				{DOVDD, Vol_1800, 1},        // 4     11
				{DVDD, Vol_1100, 1},      // 5     7
				{SensorMCLK, Vol_High, 2},  // 15    11
				{RST, Vol_High, 5},         // 2     11
			},
    },
#endif
#if defined(S5K3P9SP_MIPI_RAW22633)
		{
			SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW22633,
			{
				{DVDD_1, Vol_1100, 1}, // 1.05V   // 7     11
				{RST, Vol_Low, 1},              // 2     0
				{AVDD, Vol_2800, 1},            // 3     11
				{DVDD, Vol_1100, 1},            // 4     11
				{DOVDD, Vol_1800, 1},           // 5     7
				{SensorMCLK, Vol_High, 1},      // 15    11
				{RST, Vol_High, 2}              // 2     11
			},
		},
#endif
#if defined(SC202CS_MIPI_MONO22633)
    {
        SENSOR_DRVNAME_SC202CS_MIPI_MONO22633,
        {
            {PDN, Vol_Low, 1},//1
            {DOVDD, Vol_1800, 0},//5
            {AVDD, Vol_2800, 1},//3
            {PDN, Vol_High, 1},//1
            {PDN, Vol_Low, 1},//1
            {PDN, Vol_High, 4},//1
            {SensorMCLK, Vol_High, 4},//15
        },
    },
#endif
#if defined(OV02B1B_MIPI_MONO22633)
    {
        SENSOR_DRVNAME_OV02B1B_MIPI_MONO22633,
        {
            {PDN, Vol_Low, 1},
            //{DVDD, Vol_1200, 0},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 6},
            {PDN, Vol_High, 4},
            {SensorMCLK, Vol_High, 1},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20075[] = {
#if defined(OV48B_CHIVASMIPI_RAW)
    {
        SENSOR_DRVNAME_OV48B_CHIVASMIPI_RAW,
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
#if defined(OV32A_CHIVASMIPI_RAW)
    {
        SENSOR_DRVNAME_OV32A_CHIVASMIPI_RAW,
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
#if defined(HI846_CHIVASMIPI_RAW)
    {
        SENSOR_DRVNAME_HI846_CHIVASMIPI_RAW,
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
#if defined(GC02K0_CHIVASMIPI_RAW)
    {
        SENSOR_DRVNAME_GC02K0_CHIVASMIPI_RAW,
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

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_21101[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(S5KJN103_MIPI_RAW)
    {S5KJN103_SENSOR_ID, SENSOR_DRVNAME_S5KJN103_MIPI_RAW, S5KJN103_MIPI_RAW_SensorInit},
#endif
#if defined(S5KGM1ST_MIPI_RAW)
    {S5KGM1ST_SENSOR_ID, SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW, S5KGM1ST_MIPI_RAW_SensorInit},
#endif
#if defined(S5K3L6_MIPI_RAW)
    {S5K3L6_SENSOR_ID, SENSOR_DRVNAME_S5K3L6_MIPI_RAW, S5K3L6_MIPI_RAW_SensorInit},
#endif
#if defined(S5K3P9SP_MIPI_RAW)
     {S5K3P9SP_SENSOR_ID, SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW, S5K3P9SP_MIPI_RAW_SensorInit},
 #endif
#if defined(IMX355_MIPI_RAW)
    {IMX355_SENSOR_ID, SENSOR_DRVNAME_IMX355_MIPI_RAW, IMX355_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO)
    {GC02M1B_SENSOR_ID, SENSOR_DRVNAME_GC02M1B_MIPI_MONO, GC02M1B_MIPI_MONO_SensorInit},
#endif
    /* ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_21101[] = {
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
            {IMGSENSOR_HW_PIN_AFVDD,   IMGSENSOR_HW_ID_REGULATOR},
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
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
            {IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
        },
    },
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20015[] = {
	{
		IMGSENSOR_SENSOR_IDX_MAIN,
		IMGSENSOR_I2C_DEV_0,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
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
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
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
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_21041[] = {
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
			{IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
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
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
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
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_21101[] = {
#if defined(S5KJN103_MIPI_RAW)
    {
        SENSOR_DRVNAME_S5KJN103_MIPI_RAW,
        {
            {RST, Vol_Low, 0},
            {AF_VDD, Vol_2800, 5},
            {DVDD, Vol_1100, 2},
            {AVDD, Vol_2800, 0, 0, 2},
            {DOVDD, Vol_1800, 1},
            {RST, Vol_High, 2},
            {SensorMCLK, Vol_High, 10},
        },
    },
#endif
#if defined(S5KGM1ST_MIPI_RAW)
    {
        SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {RST, Vol_High, 2},
            {AF_VDD, Vol_2800, 0},
            {SensorMCLK, Vol_High, 1},
        },
    },
#endif
#if defined(S5K3L6_MIPI_RAW)
    {
        SENSOR_DRVNAME_S5K3L6_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {RST, Vol_High, 2},
            {AF_VDD, Vol_2800, 0},
            {SensorMCLK, Vol_High, 1},
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
            {DOVDD, Vol_1800, 0},
            {RST, Vol_High, 2},
          },
        },
#endif
#if defined(IMX355_MIPI_RAW)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1200, 1},
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
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0},
            {RST, Vol_High, 2},
            {SensorMCLK, Vol_High, 1, 0, 1},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};


struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_21081[] = {
#if defined(IMX581_MIPI_RAW)
    {
		SENSOR_DRVNAME_IMX581_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2900, 0},
			{AVDD1, Vol_1800, 0},
			{DVDD, Vol_1100, 1},
			{AF_VDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 3}
		},
	},
#endif
#if defined(IMX471_MIPI_RAW)
    {
        SENSOR_DRVNAME_IMX471_MIPI_RAW,
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
#if defined(GC02M1B_MIPI_MONO)
    {
        SENSOR_DRVNAME_GC02M1B_MIPI_MONO,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2}
        },
    },
#endif
#if defined(GC02M1_MIPI_RAW)
    {
        SENSOR_DRVNAME_GC02M1_MIPI_RAW,
        {
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20015[] = {
#if defined(OV13B10_MIPI_RAW_CARR)
	{
		SENSOR_DRVNAME_OV13B10_MIPI_RAW_CARR,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{RST, Vol_High, 5},
			{SensorMCLK, Vol_High, 1},
		},
	},
#endif
#if defined(S5K3L6_MIPI_RAW_CARR)
	{
		SENSOR_DRVNAME_S5K3L6_MIPI_RAW_CARR,
		{
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1100, 1},
			{RST, Vol_High, 6},
			{SensorMCLK, Vol_High, 5},
		},
	},
#endif
#if defined(IMX355_MIPI_RAW_CARR)
	{
		SENSOR_DRVNAME_IMX355_MIPI_RAW_CARR,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 2},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(GC02M1B_MIPI_MONO_CARR)
	{
		SENSOR_DRVNAME_GC02M1B_MIPI_MONO_CARR,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(OV02B10_MIPI_RAW_CARR)
	{
		SENSOR_DRVNAME_OV02B10_MIPI_RAW_CARR,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 9},
			{RST, Vol_High, 1}
		},
	},
#endif
#if defined(GC02M1_MIPI_RAW_CARR)
	{
		SENSOR_DRVNAME_GC02M1_MIPI_RAW_CARR,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_216A0[] = {
#if defined(S5KGM1ST_MIPI_RAW_APOLLOF)
    {
        SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_APOLLOF,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {RST, Vol_High, 2},
            {AF_VDD, Vol_2800, 0},
            {SensorMCLK, Vol_High, 1},
            // #ifdef VENDOR_EDIT
            // 	{MIPISEL, Vol_High, 3},
            // #endif
        },
    },
#endif

#if defined(S5K3P9SP_MIPI_RAW_APOLLOF)
        {
            SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW_APOLLOF,
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

#if defined(OV02B10_MIPI_RAW_APOLLOF)
    {
        SENSOR_DRVNAME_OV02B10_MIPI_RAW_APOLLOF,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 5},
            {SensorMCLK, Vol_High, 0},
            {RST, Vol_High, 10}
        },
    },
#endif

#if defined(OV02B1B_MIPI_MONO_APOLLOF)
    {
        SENSOR_DRVNAME_OV02B1B_MIPI_MONO_APOLLOF,
        {
            {RST, Vol_Low, 1},
            //{DVDD, Vol_1200, 0},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 5},
            {RST, Vol_High, 4},
              {SensorMCLK, Vol_High, 1}
        },
    },
#endif

#if defined(OV64B_MIPI_RAW_APOLLOF)
    {
        SENSOR_DRVNAME_OV64B_MIPI_RAW_APOLLOF,
        {
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 1},
            {AVDD, Vol_2800, 2},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 5},
            {RST, Vol_High, 5},
            {AF_VDD, Vol_2800, 0},
        },
    },
#endif

    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_22610[] = {

#if defined(OV64B40_MIPI_RAW_HAWAII)
    {
        SENSOR_DRVNAME_OV64B40_MIPI_RAW_HAWAII,
        {
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 1},
            {AVDD, Vol_2800, 2},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1100, 5},
            {RST, Vol_High, 5},
            {AF_VDD, Vol_2800, 10},
        },
    },
#endif

#if defined(OV08D10_MIPI_RAW_HAWAII)
    {
        SENSOR_DRVNAME_OV08D10_MIPI_RAW_HAWAII,
        {
            {RST, Vol_Low, 3},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 5},
            {AVDD_1, Vol_2800, 3},
            {DVDD, Vol_1200, 5},
            {RST, Vol_High, 3},
            {SensorMCLK, Vol_High, 8},
        },
    },
#endif

#if defined(OV16A1Q_MIPI_RAW_HAWAII)
	{
		SENSOR_DRVNAME_OV16A1Q_MIPI_RAW_HAWAII,
		{
			{RST, Vol_Low, 3},
			{AVDD, Vol_2800, 0},
			{AVDD_1, Vol_2800, 2},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 0},
			{RST, Vol_High, 6},
			{SensorMCLK, Vol_High, 3}
		},
	},
#endif

#if defined(OV02B1B_MIPI_MONO_HAWAII)
    {
        SENSOR_DRVNAME_OV02B1B_MIPI_MONO_HAWAII,
        {
            {RST, Vol_Low, 1},
            //{DVDD, Vol_1200, 0},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 6},
            {AVDD_1, Vol_2800, 6},
            {RST, Vol_High, 8},
            {SensorMCLK, Vol_High, 1},
        },
    },
#endif

    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_21041[] = {
#if defined(S5KGM1ST_MIPI_RAW_ODINA)
	{
		SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_ODINA,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1100, 0},
			{DOVDD, Vol_1800, 1},
			{RST, Vol_High, 2},
			{AF_VDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			// #ifdef VENDOR_EDIT
			// 	{MIPISEL, Vol_High, 3},
			// #endif
		},
	},
#endif
#if defined(IMX355_MIPI_RAW_ODINA)
	{
		SENSOR_DRVNAME_IMX355_MIPI_RAW_ODINA,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(GC02M1B_MIPI_MONO_ODINA)
	{
		SENSOR_DRVNAME_GC02M1B_MIPI_MONO_ODINA,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif

#if defined(OV02B10_MIPI_RAW_ODINA)
	{
		SENSOR_DRVNAME_OV02B10_MIPI_RAW_ODINA,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 5},
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_High, 10}
		},
	},
#endif
	/* add new sensor before this line */
	{NULL,},
};
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_alicert[] = {
#if defined(S5K3L6_MIPI_RAW_ALICERT)
    {
        SENSOR_DRVNAME_S5K3L6_MIPI_RAW_ALICERT,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
            {AF_VDD, Vol_2800, 0},
        },
    },
#endif
#if defined(IMX355_MIPI_RAW_ALICERT)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW_ALICERT,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1200, 1},
            {AVDD, Vol_2800, 0},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2}
        },
    },
#endif
#if defined(GC02M1B_MIPI_MONO_ALICERT)
    {
        SENSOR_DRVNAME_GC02M1B_MIPI_MONO_ALICERT,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0},
            {RST, Vol_High, 2},
            {SensorMCLK, Vol_High, 1, 0, 1},
        },
    },
#endif
#if defined(S5KGM1ST_MIPI_RAW_ALICER)
    {
        SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_ALICER,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {RST, Vol_High, 2},
            {AF_VDD, Vol_2800, 0},
            {SensorMCLK, Vol_High, 1},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_aliceq[] = {
#if defined(S5K3L6_MIPI_RAW_ALICERT)
    {
        SENSOR_DRVNAME_S5K3L6_MIPI_RAW_ALICERT,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2},
            {AF_VDD, Vol_2800, 0},
        },
    },
#endif
#if defined(IMX355_MIPI_RAW_ALICER)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW_ALICER,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1200, 1},
            {AVDD, Vol_2800, 0},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2}
        },
    },
#endif
#if defined(GC02M1B_MIPI_MONO_ALICERT)
    {
        SENSOR_DRVNAME_GC02M1B_MIPI_MONO_ALICERT,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0},
            {RST, Vol_High, 2},
            {SensorMCLK, Vol_High, 1, 0, 1},
        },
    },
#endif
#if defined(S5KGM1ST_MIPI_RAW_ALICER)
    {
        SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_ALICER,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {RST, Vol_High, 2},
            {AF_VDD, Vol_2800, 0},
            {SensorMCLK, Vol_High, 1},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_alicer[] = {
#if defined(S5KJN103_MIPI_RAW_ALICER)
    {
        SENSOR_DRVNAME_S5KJN103_MIPI_RAW_ALICER,
        {
            {RST, Vol_Low, 1},
            {AF_VDD, Vol_2800, 8},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {RST, Vol_High, 2},
            {AF_VDD, Vol_2800, 0},
            {SensorMCLK, Vol_High, 10},
        },
    },
#endif
#if defined(IMX355_MIPI_RAW_ALICER)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW_ALICER,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1200, 1},
            {AVDD, Vol_2800, 0},
            {SensorMCLK, Vol_High, 1},
            {RST, Vol_High, 2}
        },
    },
#endif
#if defined(GC02M1B_MIPI_MONO_ALICER)
    {
        SENSOR_DRVNAME_GC02M1B_MIPI_MONO_ALICER,
        {
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {AVDD, Vol_2800, 0},
            {RST, Vol_High, 2},
            {SensorMCLK, Vol_High, 1, 0, 1},
        },
    },
#endif
#if defined(S5KGM1ST_MIPI_RAW_ALICER)
    {
        SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_ALICER,
        {
            {RST, Vol_Low, 1},
            {AVDD, Vol_2800, 0},
            {DVDD, Vol_1100, 0},
            {DOVDD, Vol_1800, 1},
            {RST, Vol_High, 2},
            {AF_VDD, Vol_2800, 0},
            {SensorMCLK, Vol_High, 1},
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20611[] = {
#if defined(S5KGM1ST_MIPI_RAW_20611)
		{
			SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_20611,
			{
				{RST, Vol_Low, 1},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1200, 0},
				{DOVDD, Vol_1800, 1},
				{MIPISEL,Vol_High,3},
				{AF_VDD, Vol_2800, 0},
				{RST, Vol_High, 2},
				{SensorMCLK, Vol_High, 1},
			},
		},
#endif
//#if defined(OV16A10_MIPI_RAW_20611)
//		{
//			SENSOR_DRVNAME_OV16A10_MIPI_RAW_20611,
//			{
//				{RST, Vol_Low, 1},
//				{AVDD, Vol_2800, 0},
//				{DVDD, Vol_1500, 0},
//				{DOVDD, Vol_1800, 1},
//				{MIPISEL, Vol_High, 3},
//				{SensorMCLK, Vol_High, 1},
//				{RST, Vol_High, 2},
//			},
//		},
//#endif
#if defined(S5K3P9SP_MIPI_RAW_20611)
	{
		SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW_20611,
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
#if defined(HI846_MIPI_RAW_20611)
		{
			SENSOR_DRVNAME_HI846_MIPI_RAW_20611,
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
#if defined(GC02K_MIPI_RAW_20611)
	{
		SENSOR_DRVNAME_GC02K_MIPI_RAW_20611,
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
#if defined(OV02B1B_MIPI_RAW_20611)
	{
		SENSOR_DRVNAME_OV02B1B_MIPI_RAW_20611,
		{
			{RST, Vol_Low, 1},
			//{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 5},
			{RST, Vol_High, 4},
  			{SensorMCLK, Vol_High, 1}
		},
	},
#endif
	/* add new sensor before this line */
	{NULL,},
};

struct IMGSENSOR_HW_POWER_SEQ platform_power_sequence_20611[] ={
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

struct CAMERA_DEVICE_INFO gImgEepromInfo_20181= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV64B_SENSOR_ID,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "ov64b2q"},
        {IMX615_SENSOR_ID,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx615"},
        {IMX355_SENSOR_ID,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "imx355"},
        {OV02B10_SENSOR_ID, 0xA4, {0x00, 0x05}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "ov02b10"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3230,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR, IMX355_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_23031= {
    .i4SensorNum = 3,
    .pCamModuleInfo = {
        {OV64B_SENSOR_ID23031,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x90,0x94}, "Cam_r0", "ov64b"},
        {OV08D_SENSOR_ID23031,   0xA0, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "ov08d"},
        {GC02M1_SENSOR_ID23031,  0xFF, {0xFF, 0xFF}, 0xFF, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "gc02m1"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3230,
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_22277= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV64B_SENSOR_ID22277,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x90,0x94}, "Cam_r0", "ov64b"},
        {OV32C_SENSOR_ID22277,   0xA0, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "ov32c"},
        {IMX355_SENSOR_ID22277,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "imx355"},
        {IMX709_SENSOR_ID22277,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x90,0x94}, "Cam_r2", "imx709"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = IMGSENSOR_SENSOR_IDX_SUB2,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3230,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR_MW_22277, IMX355_STEREO_START_ADDR_22277},
    .i4MTStereoAddr = {OV64B_STEREO_START_ADDR_MT_22277, IMX709_STEREO_START_ADDR_22277},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_23231= {
    .i4SensorNum =  5,
    .pCamModuleInfo = {
        {IMX882_SENSOR_ID23231,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x90,0x94}, "Cam_r0", "imx882"},
        {OV32C_SENSOR_ID23231,   0xA0, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "ov32c"},
        {IMX355_SENSOR_ID23231,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "imx355"},
        {IMX709_SENSOR_ID23231,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x90,0x94}, "Cam_r2", "imx709"},
        {IMX882PD_SENSOR_ID23231,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x90,0x94}, "Cam_r3", "imx882pd"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = IMGSENSOR_SENSOR_IDX_SUB2,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3230,
    .i4MWStereoAddr = {IMX882_STEREO_START_ADDR_MW_23231, IMX355_STEREO_START_ADDR_23231},
    .i4MTStereoAddr = {IMX882_STEREO_START_ADDR_MT_23231, IMX709_STEREO_START_ADDR_23231},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
    .i4DistortionAddr = IMX355_DISTORTIONPARAMS_START_ADDR,
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_20151= {
    .i4SensorNum = 5,
    .pCamModuleInfo = {
        {OV48B_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x98,0x94}, "Cam_r0", "ov48b"},
        {IMX471_SENSOR_ID,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f", "imx471"},
        {HI846_SENSOR_ID,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "hi846"},
        {OV02B10_SENSOR_ID,  0xA4, {0x00, 0x06}, 0xE0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "ov02b10"},
        {OV02A1B_SENSOR_ID,  0xFF, {0xFF, 0xFF}, 0xFF, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r3", "ov02a1b"},
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

struct CAMERA_DEVICE_INFO gImgEepromInfo_20001= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV16A10_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x98,0x94}, "Cam_r0", "ov16a10"},  //weixiugai
        {S5K4H7_SENSOR_ID,  0x20, {0x00, 0x06}, 0x24, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "5sk4h7"},  //weixiugai
        {HI846_SENSOR_ID,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "hi846"},
        {GC02M1B_SENSOR_ID, 0xA2, {0xFF, 0xFF}, 0x0C, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02m1b"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {OV16A10_STEREO_START_ADDR, HI846_STEREO_START_ADDR_20001},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

 struct CAMERA_DEVICE_INFO gImgEepromInfo_23707= {
     .i4SensorNum = 3,
     .pCamModuleInfo = {
         {S5KJN1_SENSOR_ID23707,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5kjn1"},
         {HI1634Q_SENSOR_ID23707,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "hi1634q"},
         {GC02M1_SENSOR_ID23707, 0xA4, {0x00, 0x05}, 0xE0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "gc02m1"},
     },
     .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
     .i4MTDataIdx = 0xFF,
     .i4FrontDataIdx = 0xFF,
     .i4NormDataLen = 40,
     .i4MWDataLen = 3230,
     .i4MWStereoAddr = {0xFFFF, 0xFFFF},
     .i4MTStereoAddr = {0xFFFF, 0xFFFF},
     .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
     .i4DistortionAddr = IMX355_DISTORTIONPARAMS_START_ADDR,
 };

struct CAMERA_DEVICE_INFO gImgEepromInfo_20391= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV64B_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x98,0x94}, "Cam_r0", "ov64b2q"},
        {OV32A_SENSOR_ID,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f", "ov32a1q"},
        {HI846_SENSOR_ID,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "hi846"},
        {OV02B10_SENSOR_ID,  0xA4, {0x00, 0x06}, 0xE0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "ov02b10"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR, HI846_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_20075= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV48B_SENSOR_ID_CHIVAS,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x98,0x94}, "Cam_r0", "ov48b2q"},
        {OV32A_SENSOR_ID_CHIVAS,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "ov32a1q"},
        {HI846_SENSOR_ID_CHIVAS,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "hi846"},
        {GC02K0_SENSOR_ID_CHIVAS, 0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02k0"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {OV48B_STEREO_START_ADDR, HI846_STEREO_START_ADDR_19131},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_20630 = {
    .i4SensorNum = 5,
    .pCamModuleInfo = {
        {S5KGW3_SENSOR_ID_20631,              0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5kgw3_20630"},
        {OV32A1Q_SENSOR_ID_20631,     0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "athensb_ov32a1q_20630"},
        {HI846_SENSOR_ID_20631,       0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "athensb_hi846_20630"},
        {GC02M1B_SENSOR_ID_20631,     0xFF, {0xFF, 0xFF}, 0xFF, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "athensb_gc02m1b_20630"},
        {GC02K_SENSOR_ID_20631,       0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r3", "athensb_gc02k_20630"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {S5KGW3_STEREO_START_ADDR_20630, HI846_STEREO_START_ADDR_20630},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_20633 = {
    .i4SensorNum = 5,
    .pCamModuleInfo = {
        {S5KGM1ST_SENSOR_ID_20633c,    0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "athensc_s5kgm1st_20630"},
        {IMX471_SENSOR_ID_20633c,      0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "athensc_imx471_20630"},
        {HI846_SENSOR_ID_20633c,       0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "athensc_hi846_20630"},
        {GC02M1B_SENSOR_ID_20631,     0xFF, {0xFF, 0xFF}, 0xFF, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "athensb_gc02m1b_20630"},
        {GC02K_SENSOR_ID_20631,       0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r3", "athensb_gc02k_20630"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {S5KGM1ST_STEREO_START_ADDR_20633, HI846_STEREO_START_ADDR_20633},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_21101 = {
    .i4SensorNum = 3,
    .pCamModuleInfo = {
        {S5KGM1ST_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5kgm1st"},
        {IMX355_SENSOR_ID,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx355"},
        {GC02M1B_SENSOR_ID,  0xFF, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "gc02m1b"},
    },
    .i4MWDataIdx = 0xFF,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_21102 = {
    .i4SensorNum = 3,
    .pCamModuleInfo = {
        {S5K3L6_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5k3l6"},
        {IMX355_SENSOR_ID,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx355"},
        {GC02M1B_SENSOR_ID,  0xFF, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "gc02m1b"},
    },
    .i4MWDataIdx = 0xFF,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_22084 = {
    .i4SensorNum = 3,
    .pCamModuleInfo = {
        {S5K3L6_SENSOR_ID_LIJING,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5k3l6_mipi_raw_lijing"},
        {SC800CS_SENSOR_ID_LIJING, 0xA0, {0x00, 0x06}, 0x0834, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "sc800cs_mipi_raw_lijing"},
        {SC201CS_SENSOR_ID_LIJING,  0xFF, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "sc201cs_mipi_raw_lijing"},
    },
    .i4MWDataIdx = 0xFF,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_21831 = {
    .i4SensorNum = 3,
    .pCamModuleInfo = {
        {S5KGM1ST_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5kgm1st"},
        {S5K3P9SP_SENSOR_ID,  0xA8, {0x00, 0x06}, 0xE0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "s5k3p9sp"},
        {GC02M1B_SENSOR_ID,  0xFF, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "gc02m1b"},
    },
    .i4MWDataIdx = 0xFF,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_21081= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {IMX581_SENSOR_ID,      0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "imx581"},
        {IMX471_SENSOR_ID,      0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx471"},
        //{GC02M1B_SENSOR_ID,     0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "gc02m1b"},
        //{GC02M1_SENSOR_ID_DUFU, 0xA4, {0x00, 0x05}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02m"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3230,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR, IMX355_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
    .i4DistortionAddr = IMX355_DISTORTIONPARAMS_START_ADDR,
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_20015 = {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV48B_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "ov48b2q"},
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

struct CAMERA_DEVICE_INFO gImgEepromInfo_21041 = {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV48B_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "ov48b2q"},
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
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_alicert = {
    .i4SensorNum = 3,
    .pCamModuleInfo = {
        {S5K3L6_SENSOR_ID_ALICERT,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5k3l6"},
        {IMX355_SENSOR_ID_ALICERT,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx355"},
        {GC02M1B_SENSOR_ID_ALICERT,  0xFF, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "gc02m1b"},
    },
    .i4MWDataIdx = 0xFF,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_aliceq = {
    .i4SensorNum = 3,
    .pCamModuleInfo = {
        {S5K3L6_SENSOR_ID_ALICERT,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5k3l6"},
        {IMX355_SENSOR_ID_ALICER,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx355"},
        {GC02M1B_SENSOR_ID_ALICERT,  0xFF, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "gc02m1b"},
    },
    .i4MWDataIdx = 0xFF,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_alicer = {
    .i4SensorNum = 3,
    .pCamModuleInfo = {
        {S5KGM1ST_SENSOR_ID_ALICER,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5kgm1st"},
        {IMX355_SENSOR_ID_ALICER,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx355"},
        {GC02M1B_SENSOR_ID_ALICER,  0xFF, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "gc02m1b"},
    },
    .i4MWDataIdx = 0xFF,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_20611 = {
    .i4SensorNum = 5,
    .pCamModuleInfo = {
        {S5KGM1ST_SENSOR_ID_20611,    0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "s5kgm1st_mipi_raw_20611"},
        {S5K3P9SP_SENSOR_ID_20611,      0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "s5k3p9sp_mipi_raw_20611"},
        {HI846_SENSOR_ID_20611,       0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "hi846_mipi_raw_20611"},
        {GC02K_SENSOR_ID_20611,     0xFF, {0xFF, 0xFF}, 0xFF, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02k_mipi_raw_20611"},
        {OV02B1B_SENSOR_ID_20611,       0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r3", "ov02b1b_mipi_raw_20611"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {S5KGM1ST_STEREO_START_ADDR_20611, HI846_STEREO_START_ADDR_20611},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_blade = {
    .i4SensorNum = 3,
    .pCamModuleInfo = {
        {HI1336_SENSOR_ID_BLADE,  0xA0, {0x00, 0x06}, 0x50, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "hi1336"},
        {HI556_SENSOR_ID_BLADE,  0x40, {0x00, 0x02}, 0x76B, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "hi556"},
        {BF20A1_SENSOR_ID_BLADE,     0xFF, {0xFF, 0xFF}, 0xFF, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "bf20a1"},
    },
    .i4MWDataIdx = 0xFF,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_bladeA = {
    .i4SensorNum = 5,
    .pCamModuleInfo = {
        {HI5021SQT_SENSOR_ID_BLADE,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x94,0x94}, "Cam_r0", "hi5021sqt"},
        {S5KJN103_SENSOR_ID_BLADE,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_f", "s5kjn103_mipi_raw_blade"},
        {SC800CS_SENSOR_ID_BLADE,  0xA0, {0x00, 0x06}, 0x834, 0, {0x92,0x96,0x94,0x94}, "Cam_r1",  "sc800cs"},
        {SC201CS1_SENSOR_ID_BLADE,  0xA4, {0x00, 0x06}, 0x834, 0, {0x34,0x36,0x36,0x36}, "Cam_r2", "sc201cs1"},
        {BF20A1_SENSOR_ID_BLADE,     0xFF, {0xFF, 0xFF}, 0xFF, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r3", "bf20a1"},
    },
    .i4MWDataIdx = 0xFF,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_23687= {
    .i4SensorNum = 5,
    .pCamModuleInfo = {
        {IMX882_SENSOR_ID23687,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x94,0x94}, "Cam_r0", "imx882"},
        {HI1634Q_SENSOR_ID23687,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "hi1634q"},
        {HI846W_SENSOR_ID23687,   0xA0, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "hi846w"},
        {GC02M1_SENSOR_ID23687,   0xA0, {0x00, 0x05}, 0xE0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02m1"},
        {IMX882PD_SENSOR_ID23687, 0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x94,0x94}, "Cam_r3", "imx882pd"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3230,
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
    .i4DistortionAddr = 0xFFFF,
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_23689= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {IMX890_SENSOR_ID23689,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x94,0x94}, "Cam_r0", "imx890"},
        {HI1634Q_SENSOR_ID23687,  0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "hi1634q"},
        {HI846W_SENSOR_ID23687,   0xA0, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "hi846w"},
        {GC02M1_SENSOR_ID23687,   0xA0, {0x00, 0x05}, 0xE0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02m1"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3230,
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
    .i4DistortionAddr = 0xFFFF,
};
struct IMGSENSOR_SENSOR_LIST
 gimgsensor_sensor_list_20041[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV16A10_MIPI_RAW)
    {OV16A10_SENSOR_ID, SENSOR_DRVNAME_OV16A10_MIPI_RAW, OV16A10_MIPI_RAW_SensorInit},
#endif
#if defined(S5K4H7_MIPI_RAW)
    {S5K4H7_SENSOR_ID, SENSOR_DRVNAME_S5K4H7_MIPI_RAW, S5K4H7_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO1)
    {GC02M1B_SENSOR_ID1, SENSOR_DRVNAME_GC02M1B_MIPI_MONO1, GC02M1B_MIPI_MONO1_SensorInit},
#endif
#if defined(GC02K0_MIPI_RAW)
    {GC02K0_SENSOR_ID, SENSOR_DRVNAME_GC02K0_MIPI_RAW, GC02K0_MIPI_RAW_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW_LAFITE)
    {IMX355_SENSOR_ID_LAFITE, SENSOR_DRVNAME_IMX355_MIPI_RAW_LAFITE, IMX355_MIPI_RAW_LAFITE_SensorInit},
#endif
    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};


struct IMGSENSOR_HW_CFG imgsensor_custom_config_20041[] = {
	{
		IMGSENSOR_SENSOR_IDX_MAIN,
		IMGSENSOR_I2C_DEV_0,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
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
		IMGSENSOR_I2C_DEV_1,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
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
		IMGSENSOR_SENSOR_IDX_SUB3,
		IMGSENSOR_I2C_DEV_5,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_PDN,	 IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,	 IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_MAIN4,
		IMGSENSOR_I2C_DEV_6,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_PDN,	 IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,	 IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_SUB4,
		IMGSENSOR_I2C_DEV_7,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_PDN,	 IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,	 IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_MAIN5,
		IMGSENSOR_I2C_DEV_7,
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
		IMGSENSOR_SENSOR_IDX_SUB5,
		IMGSENSOR_I2C_DEV_7,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_PDN,	 IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,	 IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_MAIN6,
		IMGSENSOR_I2C_DEV_7,
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
		IMGSENSOR_SENSOR_IDX_SUB6,
		IMGSENSOR_I2C_DEV_7,
		{
			{IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_PDN,	 IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,	 IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},

    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20041[] = {
#if defined(OV16A10_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV16A10_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(S5K4H7_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K4H7_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1100, 1},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(GC02M1B_MIPI_MONO1)
	{
		SENSOR_DRVNAME_GC02M1B_MIPI_MONO1,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
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
#if defined(IMX355_MIPI_RAW_LAFITE)
	{
		SENSOR_DRVNAME_IMX355_MIPI_RAW_LAFITE,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1200, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_20041= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV16A10_SENSOR_ID, 0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x98,0x94}, "Cam_r0", "ov16a10"},  //weixiugai
        {S5K4H7_SENSOR_ID, 0x20, {0x00, 0x06}, 0x24, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "5sk4h7"},  //weixiugai
        {GC02M1B_SENSOR_ID1, 0xA2, {0xFF, 0xFF}, 0x0C, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "gc02m1b"},
        {GC02K0_SENSOR_ID, 0xA4, {0xFF, 0xFF}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02k0"},
        {IMX355_SENSOR_ID, 0xA8, {0xFF, 0xFF}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f", "imx355"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {OV16A10_STEREO_START_ADDR, GC02M1B_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_216A0 = {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV48B_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "ov48b2q"},
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

struct CAMERA_DEVICE_INFO gImgEepromInfo_22610 = {
    .i4SensorNum = 3,
    .pCamModuleInfo = {
        {OV64B40_SENSOR_ID_HAWAII,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x94,0x94}, "Cam_r0", "ov64b40"},
        {OV08D10_SENSOR_ID_HAWAII,  0xA0, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "ov08d10"}, 
        {OV02B1B_SENSOR_ID_HAWAII,  0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "ov02b1b"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR },
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_22705 = {
    .i4SensorNum = 3,
    .pCamModuleInfo = {
        {OV64B40_SENSOR_ID_HAWAII,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0x96,0x94,0x94}, "Cam_r0", "ov64b40"},
        {OV16A1Q_SENSOR_ID_HAWAII,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f1",  "ov16a1q"},
        {OV02B1B_SENSOR_ID_HAWAII,  0xA4, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "ov02b1b"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR },
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_20609[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(S5KGM1ST_MIPI_RAW_APOLLOB)
	{S5KGM1ST_SENSOR_ID_APOLLOB, SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_APOLLOB, S5KGM1ST_MIPI_RAW_APOLLOB_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW_APOLLOB)
	{IMX355_SENSOR_ID_APOLLOB, SENSOR_DRVNAME_IMX355_MIPI_RAW_APOLLOB, IMX355_MIPI_RAW_APOLLOB_SensorInit},
#endif
#if defined(GC02M1B_MIPI_MONO_APOLLOB)
	{GC02M1B_SENSOR_ID_APOLLOB, SENSOR_DRVNAME_GC02M1B_MIPI_MONO_APOLLOB, GC02M1B_MIPI_MONO_APOLLOB_SensorInit},
#endif
#if defined(OV02B10_MIPI_RAW_APOLLOB)
	{OV02B10_SENSOR_ID_APOLLOB, SENSOR_DRVNAME_OV02B10_MIPI_RAW_APOLLOB, OV02B10_MIPI_RAW_APOLLOB_SensorInit},
#endif
#if defined(S5K3P9SP_MIPI_RAW_APOLLOB)
	{S5K3P9SP_SENSOR_ID_APOLLOB, SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW_APOLLOB, S5K3P9SP_MIPI_RAW_APOLLOB_SensorInit},
#endif
#if defined(OV13B10_MIPI_RAW_APOLLOB)
	{OV13B10_SENSOR_ID_APOLLOB, SENSOR_DRVNAME_OV13B10_MIPI_RAW_APOLLOB, OV13B10_MIPI_RAW_APOLLOB_SensorInit},
#endif
#if defined(GC02M1_MIPI_RAW_APOLLOT)
	{GC02M1_SENSOR_ID_APOLLOT, SENSOR_DRVNAME_GC02M1_MIPI_RAW_APOLLOT, GC02M1_MIPI_RAW_APOLLOT_SensorInit},
#endif
    /* ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20609[] = {
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
			{IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
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
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
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
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_20796[] = {
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
			{IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
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
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
    {IMGSENSOR_SENSOR_IDX_NONE}
};
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20609[] = {
#if defined(S5KGM1ST_MIPI_RAW_APOLLOB)
	{
		SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_APOLLOB,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1100, 0},
			{DOVDD, Vol_1800, 1},
			{RST, Vol_High, 2},
			{AF_VDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			// #ifdef VENDOR_EDIT
			// 	{MIPISEL, Vol_High, 3},
			// #endif
		},
	},
#endif
#if defined(IMX355_MIPI_RAW_APOLLOB)
	{
		SENSOR_DRVNAME_IMX355_MIPI_RAW_APOLLOB,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(GC02M1B_MIPI_MONO_APOLLOB)
	{
		SENSOR_DRVNAME_GC02M1B_MIPI_MONO_APOLLOB,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif

#if defined(OV02B10_MIPI_RAW_APOLLOB)
	{
		SENSOR_DRVNAME_OV02B10_MIPI_RAW_APOLLOB,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 5},
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_High, 10}
		},
	},
#endif

#if defined(OV13B10_MIPI_RAW_APOLLOB)
		{
			SENSOR_DRVNAME_OV13B10_MIPI_RAW_APOLLOB,
			{
				{RST, Vol_Low, 1},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1200, 0},
				{DOVDD, Vol_1800, 1},
				{RST, Vol_High, 5},
				{SensorMCLK, Vol_High, 1},
			},
		},
#endif

#if defined(S5K3P9SP_MIPI_RAW_APOLLOB)
		{
			SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW_APOLLOB,
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

#if defined(GC02M1_MIPI_RAW_APOLLOT)
	{
		SENSOR_DRVNAME_GC02M1_MIPI_RAW_APOLLOT,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 5},
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_High, 10}
		},
	},
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_20609 = {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV48B_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "ov48b2q"},
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

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_212A1[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV64B_MIPI_RAW212A1)
{OV64B_SENSOR_ID_212A1, SENSOR_DRVNAME_OV64B_MIPI_RAW212A1, OV64B_MIPI_RAW_212A1_SensorInit},
#endif
#if defined(IMX615_MIPI_RAW212A1)
{IMX615_SENSOR_ID_212A1, SENSOR_DRVNAME_IMX615_MIPI_RAW212A1, IMX615_MIPI_RAW_212A1_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW212A1)
{IMX355_SENSOR_ID_212A1, SENSOR_DRVNAME_IMX355_MIPI_RAW212A1, IMX355_MIPI_RAW_212A1_SensorInit},
#endif
#if defined(GC02M1_MIPI_RAW212A1)
{GC02M1_SENSOR_ID_212A1, SENSOR_DRVNAME_GC02M1_MIPI_RAW212A1, GC02M1_MIPI_RAW_212A1_SensorInit},
#endif

    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_212A1[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
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
        IMGSENSOR_I2C_DEV_2,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
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

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_212A1[] = {
#if defined(OV64B_MIPI_RAW212A1)
    {
        SENSOR_DRVNAME_OV64B_MIPI_RAW212A1,
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
#if defined(IMX615_MIPI_RAW212A1)
    {
        SENSOR_DRVNAME_IMX615_MIPI_RAW212A1,
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
#if defined(IMX355_MIPI_RAW212A1)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW212A1,
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
#if defined(GC02M1_MIPI_RAW212A1)
    {
        SENSOR_DRVNAME_GC02M1_MIPI_RAW212A1,
        {
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_212A1= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV64B_SENSOR_ID_212A1,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "ov64b2q"},
        {IMX615_SENSOR_ID_212A1,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx615"},
        {IMX355_SENSOR_ID_212A1,     0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "imx355"},
        {GC02M1_SENSOR_ID_212A1, 0xA4, {0x00, 0x05}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02m1"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3230,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR, IMX355_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
    .i4DistortionAddr = IMX355_DISTORTIONPARAMS_START_ADDR,
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_21851[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV64B_MIPI_RAW21851)
{OV64B_SENSOR_ID_21851, SENSOR_DRVNAME_OV64B_MIPI_RAW21851, OV64B_MIPI_RAW_21851_SensorInit},
#endif
#if defined(IMX471_MIPI_RAW21851)
{IMX471_SENSOR_ID_21851, SENSOR_DRVNAME_IMX471_MIPI_RAW21851, IMX471_MIPI_RAW_21851_SensorInit},
#endif
#if defined(IMX355_MIPI_RAW21851)
{IMX355_SENSOR_ID_21851, SENSOR_DRVNAME_IMX355_MIPI_RAW21851, IMX355_MIPI_RAW_21851_SensorInit},
#endif
#if defined(GC02M1_MIPI_RAW21851)
{GC02M1_SENSOR_ID_21851, SENSOR_DRVNAME_GC02M1_MIPI_RAW21851, GC02M1_MIPI_RAW_21851_SensorInit},
#endif

    /*  ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_21851[] = {
    {
        IMGSENSOR_SENSOR_IDX_MAIN,
        IMGSENSOR_I2C_DEV_0,
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
        IMGSENSOR_I2C_DEV_2,
        {
            {IMGSENSOR_HW_PIN_MCLK,  IMGSENSOR_HW_ID_MCLK},
            {IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
            {IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_REGULATOR},
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

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_21851[] = {
#if defined(OV64B_MIPI_RAW21851)
    {
        SENSOR_DRVNAME_OV64B_MIPI_RAW21851,
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
#if defined(IMX471_MIPI_RAW21851)
    {
        SENSOR_DRVNAME_IMX471_MIPI_RAW21851,
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
#if defined(IMX355_MIPI_RAW21851)
    {
        SENSOR_DRVNAME_IMX355_MIPI_RAW21851,
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
#if defined(GC02M1_MIPI_RAW21851)
    {
        SENSOR_DRVNAME_GC02M1_MIPI_RAW21851,
        {
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
        },
    },
#endif
    /* add new sensor before this line */
    {NULL,},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_21851= {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV64B_SENSOR_ID_21851,   0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "ov64b2q"},
        {IMX471_SENSOR_ID_21851,  0xA8, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "imx471"},
        {IMX355_SENSOR_ID_21851,     0xA2, {0x00, 0x06}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r1", "imx355"},
        {GC02M1_SENSOR_ID_21851, 0xA4, {0x00, 0x05}, 0xB0, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_r2", "gc02m1"},
    },
    .i4MWDataIdx = IMGSENSOR_SENSOR_IDX_MAIN2,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3230,
    .i4MWStereoAddr = {OV64B_STEREO_START_ADDR, IMX355_STEREO_START_ADDR},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
    .i4DistortionAddr = IMX355_DISTORTIONPARAMS_START_ADDR,
};

struct IMGSENSOR_SENSOR_LIST
    gimgsensor_sensor_list_210A0[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(OV02B10_MIPI_RAW_APOLLOW)
	{OV02B10_SENSOR_ID_APOLLOW, SENSOR_DRVNAME_OV02B10_MIPI_RAW_APOLLOW, OV02B10_MIPI_RAW_APOLLOW_SensorInit},
#endif
#if defined(OV02B1B_MIPI_MONO_APOLLOW)
	{OV02B1B_SENSOR_ID_APOLLOW, SENSOR_DRVNAME_OV02B1B_MIPI_MONO_APOLLOW, OV02B1B_MIPI_MONO_APOLLOW_SensorInit},
#endif
#if defined(OV64B_MIPI_RAW_APOLLOW)
	{OV64B_SENSOR_ID_APOLLOW, SENSOR_DRVNAME_OV64B_MIPI_RAW_APOLLOW, OV64B_MIPI_RAW_APOLLOW_SensorInit},
#endif
#if defined(S5K3P9SP_MIPI_RAW_APOLLOW)
	{S5K3P9SP_SENSOR_ID_APOLLOW, SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW_APOLLOW, S5K3P9SP_MIPI_RAW_APOLLOW_SensorInit},
#endif

    /* ADD sensor driver before this line */
    {0, {0}, NULL}, /* end of list */
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config_210A0[] = {
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
			{IMGSENSOR_HW_PIN_AFVDD, IMGSENSOR_HW_ID_REGULATOR},
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
			{IMGSENSOR_HW_PIN_AVDD,  IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_DOVDD, IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_DVDD,  IMGSENSOR_HW_ID_REGULATOR},
			{IMGSENSOR_HW_PIN_PDN,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_RST,   IMGSENSOR_HW_ID_GPIO},
			{IMGSENSOR_HW_PIN_NONE,  IMGSENSOR_HW_ID_NONE},
		},
	},
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_210A0[] = {
#if defined(OV02B10_MIPI_RAW_APOLLOW)
	{
		SENSOR_DRVNAME_OV02B10_MIPI_RAW_APOLLOW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 5},
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_High, 10},
		},
	},
#endif

#if defined(OV02B1B_MIPI_MONO_APOLLOW)
	{
		SENSOR_DRVNAME_OV02B1B_MIPI_MONO_APOLLOW,
		{
			{RST, Vol_Low, 1},
			//{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 5},
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_High, 10},
		},
	},
#endif

#if defined(OV64B_MIPI_RAW_APOLLOW)
	{
		SENSOR_DRVNAME_OV64B_MIPI_RAW_APOLLOW,
		{
			{RST, Vol_Low, 1},
			{SensorMCLK, Vol_High, 1},
			{AVDD, Vol_2800, 2},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1100, 5},
			{RST, Vol_High, 5},
			{AF_VDD, Vol_2800, 0},
		},
	},
#endif

#if defined(S5K3P9SP_MIPI_RAW_APOLLOW)
		{
			SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW_APOLLOW,
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

    /* add new sensor before this line */
    {NULL,},
};

struct CAMERA_DEVICE_INFO gImgEepromInfo_210A0 = {
    .i4SensorNum = 4,
    .pCamModuleInfo = {
        {OV48B_SENSOR_ID,  0xA0, {0x00, 0x06}, 0xB0, 1, {0x92,0xFF,0xFF,0x94}, "Cam_r0", "ov48b2q"},
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
void oplus_imgsensor_hwcfg()
{
    if (is_project(22083) || is_project(22291) || is_project(22292)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_lijing;
         oplus_imgsensor_custom_config = imgsensor_custom_config_lijing;
         oplus_sensor_power_sequence = sensor_power_sequence_lijing;
         gImgEepromInfo = gImgEepromInfo_lijing;
    } else if (is_project(23053) || is_project(23054) || is_project(23055)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_23053;
         oplus_imgsensor_custom_config = imgsensor_custom_config_23053;
         oplus_sensor_power_sequence = sensor_power_sequence_23053;
         gImgEepromInfo = gImgEepromInfo_23053;
    } else if (is_project(23253)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_23253;
         oplus_imgsensor_custom_config = imgsensor_custom_config_23253;
         oplus_sensor_power_sequence = sensor_power_sequence_23253;
         gImgEepromInfo = gImgEepromInfo_23253;
    } else if (is_project(22084)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_lijing;
         oplus_imgsensor_custom_config = imgsensor_custom_config_lijing;
         oplus_sensor_power_sequence = sensor_power_sequence_lijing;
         gImgEepromInfo = gImgEepromInfo_22084;
    } else if (is_project(22088) || is_project(22332)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_fanli;
         if ((get_PCB_Version() > 2 && is_project(22088)) || (get_PCB_Version() > 4)) {
            oplus_imgsensor_custom_config = imgsensor_custom_config_fanli;
            oplus_sensor_power_sequence = sensor_power_sequence_fanli;
         } else {
            oplus_imgsensor_custom_config = imgsensor_custom_config_fanli_T0;
            oplus_sensor_power_sequence = sensor_power_sequence_fanli_T0;
         }
         gImgEepromInfo = gImgEepromInfo_22088;
    } else if (is_project(22087) || is_project(22331) || is_project(22333) || is_project(22334) || is_project(22869)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_fanli;
         if ((get_PCB_Version() > 2 && (is_project(22087) || is_project(22331))) || (get_PCB_Version() > 4)) {
            oplus_imgsensor_custom_config = imgsensor_custom_config_fanli;
            oplus_sensor_power_sequence = sensor_power_sequence_fanli;
         } else {
            oplus_imgsensor_custom_config = imgsensor_custom_config_fanli_T0;
            oplus_sensor_power_sequence = sensor_power_sequence_fanli_T0;
         }
         gImgEepromInfo = gImgEepromInfo_22331;
    } else if (is_project(20181) || is_project(20355)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20181;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20181;
         oplus_sensor_power_sequence = sensor_power_sequence_20181;
         gImgEepromInfo = gImgEepromInfo_20181;
    } else if (is_project(23031)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_23031;
         oplus_imgsensor_custom_config = imgsensor_custom_config_23031;
         oplus_sensor_power_sequence = sensor_power_sequence_23031;
         gImgEepromInfo = gImgEepromInfo_23031;
    } else if (is_project(22277)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_22277;
         oplus_imgsensor_custom_config = imgsensor_custom_config_22277;
         oplus_sensor_power_sequence = sensor_power_sequence_22277;
         gImgEepromInfo = gImgEepromInfo_22277;
    }else if (is_project(23231)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_23231;
         oplus_imgsensor_custom_config = imgsensor_custom_config_23231;
         oplus_sensor_power_sequence = sensor_power_sequence_23231;
         oplus_sensor_power_on_sequence = sensor_power_on_sequence_23231;
         gImgEepromInfo = gImgEepromInfo_23231;
    }    else if (is_project(20631)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20630;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20630;
         oplus_sensor_power_sequence = sensor_power_sequence_20630;
         oplus_platform_power_sequence = platform_power_sequence_20630;
         gImgEepromInfo = gImgEepromInfo_20630;
    } else if (is_project(20633)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20633;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20633;
         oplus_sensor_power_sequence = sensor_power_sequence_20633;
         oplus_platform_power_sequence = platform_power_sequence_20633;
         gImgEepromInfo = gImgEepromInfo_20633;
    } else if (is_project(20391) || is_project(20392)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20391;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20391;
         oplus_sensor_power_sequence = sensor_power_sequence_20391;
         gImgEepromInfo = gImgEepromInfo_20391;
    } else if (is_project(20609) || is_project(0x2060B) || is_project(0x2060A)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20609;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20609;
         oplus_sensor_power_sequence = sensor_power_sequence_20609;
         gImgEepromInfo = gImgEepromInfo_20609;
    } else if (is_project(0x206FF) || is_project(20796) || is_project(0x2070C) || is_project(20795) || is_project(21747) || is_project(21748)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20609;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20796;
         oplus_sensor_power_sequence = sensor_power_sequence_20609;
         gImgEepromInfo = gImgEepromInfo_20609;
    }  else if (is_project(20151) || is_project(20301) || is_project(20302)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20151;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20151;
         oplus_sensor_power_sequence = sensor_power_sequence_20151;
         gImgEepromInfo = gImgEepromInfo_20151;
    } else if (is_project(20001) || is_project(20002) || is_project(20003) || is_project(20200)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20001;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20001;
         oplus_sensor_power_sequence = sensor_power_sequence_20001;
         gImgEepromInfo = gImgEepromInfo_20001;
    } else if (is_project(22629) || is_project(22710) || is_project(22711)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_22629;
         oplus_imgsensor_custom_config = imgsensor_custom_config_22629;
         oplus_sensor_power_sequence = sensor_power_sequence_22629;
         gImgEepromInfo = gImgEepromInfo_22629;
    } else if (is_project(23613) || is_project(23686) || is_project(23687)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_23687;
         oplus_imgsensor_custom_config = imgsensor_custom_config_23687;
         oplus_sensor_power_sequence = sensor_power_sequence_23687;
         gImgEepromInfo = gImgEepromInfo_23687;
    } else if (is_project(23689)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_23687;
         oplus_imgsensor_custom_config = imgsensor_custom_config_23687;
         oplus_sensor_power_sequence = sensor_power_sequence_23687;
         gImgEepromInfo = gImgEepromInfo_23689;
    } else if (is_project(22633) || is_project(22712) || is_project(22713)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_22633;
         oplus_imgsensor_custom_config = imgsensor_custom_config_22633;
         oplus_sensor_power_sequence = sensor_power_sequence_22633;
         gImgEepromInfo = gImgEepromInfo_22633;
    } else if (is_project(22693) || is_project(22694) || is_project(22612) || is_project(0x226B1)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_22693;
         oplus_imgsensor_custom_config = imgsensor_custom_config_22693;
         oplus_sensor_power_sequence = sensor_power_sequence_22693;
         gImgEepromInfo = gImgEepromInfo_22693;
    } else if (is_project(0x2169E) || is_project(0x2169F) || is_project(0x216C9) || is_project(0x216CA)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_moss;
         oplus_imgsensor_custom_config = imgsensor_custom_config_moss;
         oplus_sensor_power_sequence = sensor_power_sequence_moss;
         //gImgEepromInfo = gImgEepromInfo_moss;
    }  else if (is_project(21711) || is_project(21712)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_mossa;
         oplus_imgsensor_custom_config = imgsensor_custom_config_mossa;
         oplus_sensor_power_sequence = sensor_power_sequence_mossa;
         //gImgEepromInfo = gImgEepromInfo_mossa;
    }  else if (is_project(20075) || is_project(20076)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20075;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20075;
         oplus_sensor_power_sequence = sensor_power_sequence_20075;
         gImgEepromInfo = gImgEepromInfo_20075;
    } else if (is_project(21101) || is_project(21102) || is_project(21235) || is_project(21236) || is_project(21831)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_21101;
         oplus_imgsensor_custom_config = imgsensor_custom_config_21101;
         oplus_sensor_power_sequence = sensor_power_sequence_21101;
         if (is_project(21102)) {
            gImgEepromInfo = gImgEepromInfo_21102;
         } else if (is_project(21831)) {
            gImgEepromInfo = gImgEepromInfo_21831;
         } else {
            gImgEepromInfo = gImgEepromInfo_21101;
         }
    } else if (is_project(0x2163B) || is_project(0x2163C)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_alicert;
         oplus_imgsensor_custom_config = imgsensor_custom_config_alicert;
         oplus_sensor_power_sequence = sensor_power_sequence_alicert;
         gImgEepromInfo = gImgEepromInfo_alicert;
    } else if (is_project(0x2163D)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_aliceq;
         oplus_imgsensor_custom_config = imgsensor_custom_config_aliceq;
         oplus_sensor_power_sequence = sensor_power_sequence_aliceq;
         gImgEepromInfo = gImgEepromInfo_aliceq;
    } else if (is_project(21639) || is_project(0x216CD) || is_project(0x216CE)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_alicer;
         oplus_imgsensor_custom_config = imgsensor_custom_config_alicer;
         oplus_sensor_power_sequence = sensor_power_sequence_alicer;
         gImgEepromInfo = gImgEepromInfo_alicer;
    } else if (is_project(21081)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_21081;
         oplus_imgsensor_custom_config = imgsensor_custom_config_21081;
         oplus_sensor_power_sequence = sensor_power_sequence_21081;
         gImgEepromInfo = gImgEepromInfo_21081;
    } else if (is_project(20015) || is_project(20016) || is_project(20108) || is_project(20109) || is_project(20307) || is_project(20013)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20015;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20015;
         oplus_sensor_power_sequence = sensor_power_sequence_20015;
         gImgEepromInfo = gImgEepromInfo_20015;
    } else if (is_project(21037)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_21037;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20015;
         oplus_sensor_power_sequence = sensor_power_sequence_20015;
         gImgEepromInfo = gImgEepromInfo_20015;
    } else if (is_project(21041) || is_project(21042)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_21041;
         oplus_imgsensor_custom_config = imgsensor_custom_config_21041;
         oplus_sensor_power_sequence = sensor_power_sequence_21041;
         gImgEepromInfo = gImgEepromInfo_21041;
    } else if (is_project(0x216A0)){
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_216A0;
         oplus_imgsensor_custom_config = imgsensor_custom_config_216A0;
         oplus_sensor_power_sequence = sensor_power_sequence_216A0;
         gImgEepromInfo = gImgEepromInfo_216A0;
    } else if (is_project(22610) || is_project(22705) || is_project(22706)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_22610;
         oplus_imgsensor_custom_config = imgsensor_custom_config_22610;
         oplus_sensor_power_sequence = sensor_power_sequence_22610;
         if (is_project(22610)) {
            gImgEepromInfo = gImgEepromInfo_22610;
         }
         else if (is_project(22705) || is_project(22706)) {
            gImgEepromInfo = gImgEepromInfo_22705;
         }
    } else if (is_project(20095) || is_project(20610) || is_project(20611) || is_project(20613) || is_project(20680) || is_project(20686)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20611;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20611;
         oplus_sensor_power_sequence = sensor_power_sequence_20611;
         oplus_platform_power_sequence = platform_power_sequence_20611;
         gImgEepromInfo = gImgEepromInfo_20611;
    } else if (is_project(20041) || is_project(20042) || is_project(20043)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20041;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20041;
         oplus_sensor_power_sequence = sensor_power_sequence_20041;
         gImgEepromInfo = gImgEepromInfo_20041;
     } else if (is_project(20639) || is_project(20638) || is_project(0x206B7)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_athensd;
         oplus_imgsensor_custom_config = imgsensor_custom_config_athensd;
         oplus_sensor_power_sequence = sensor_power_sequence_athensd;
         oplus_platform_power_sequence = platform_power_sequence_athensd;
         gImgEepromInfo = gImgEepromInfo_athensd;
     } else if (is_project(0x212A1)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_212A1;
         oplus_imgsensor_custom_config = imgsensor_custom_config_212A1;
         oplus_sensor_power_sequence = sensor_power_sequence_212A1;
         gImgEepromInfo = gImgEepromInfo_212A1;
     } else if (is_project(0x210A0)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_210A0;
         oplus_imgsensor_custom_config = imgsensor_custom_config_210A0;
         oplus_sensor_power_sequence = sensor_power_sequence_210A0;
         gImgEepromInfo = gImgEepromInfo_210A0;
    }  else if (is_project(22604) || is_project(22603) || is_project(0x2266B) || is_project(0x2266C) || is_project(22631) || is_project(22632)
        || is_project(0x2260A) || is_project(22609) || is_project(22669) || is_project(0x2260B) || is_project(0x2266A) || is_project(23602)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_blade;
         oplus_imgsensor_custom_config = imgsensor_custom_config_blade;
         oplus_sensor_power_sequence = sensor_power_sequence_blade;
         if (is_project(22603) || is_project(22604) || is_project(0x2260A) || is_project(22631) || is_project(22632) || is_project(23602)) {
            gImgEepromInfo = gImgEepromInfo_blade;
         } else if(is_project(22669) || is_project(0x2260B) || is_project(0x2266A) || is_project(22609)) {
            gImgEepromInfo = gImgEepromInfo_bladeA;
         }
    } else if (is_project(21851) || is_project(21876)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_21851;
         oplus_imgsensor_custom_config = imgsensor_custom_config_21851;
         oplus_sensor_power_sequence = sensor_power_sequence_21851;
         gImgEepromInfo = gImgEepromInfo_21851;
    } else if (is_project(23707) || is_project(23709)) {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_23707;
         oplus_imgsensor_custom_config = imgsensor_custom_config_23707;
         oplus_sensor_power_sequence = sensor_power_sequence_23707;
          gImgEepromInfo = gImgEepromInfo_23707;
    }  else {
         oplus_gimgsensor_sensor_list = gimgsensor_sensor_list_20181;
         oplus_imgsensor_custom_config = imgsensor_custom_config_20181;
         oplus_sensor_power_sequence = sensor_power_sequence_20181;
         gImgEepromInfo = gImgEepromInfo_20181;
    }
}

