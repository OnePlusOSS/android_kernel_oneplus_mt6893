/*
 * Copyright (C) 2015 MediaTek Inc.
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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     OV50Amipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 * Setting version:
 * ------------
 *   update full pd setting for OV50A
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#define PFX "OV50A_camera_sensor"

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/types.h>
#include "imgsensor_eeprom.h"
#include "ov50amipiraw_Sensor.h"
#include "ov50a_Sensor_setting.h"

static unsigned int _is_initFlag;

static kal_uint8 qsc_flag = 0;
static kal_uint8 otp_flag = OTP_QSC_NONE;
static uint8_t deviceInfo_register_value = 0;

#define LOG_INF(format, args...)    \
    pr_debug(PFX "[%s] " format, __func__, ##args)

#define LOG_ERR(format, args...)    \
    pr_err(PFX "[%s] " format, __func__, ##args)

#define I2C_BUFFER_LEN 765    /*trans# max is 255, each 3 bytes*/
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para);
inline void hold_group(kal_uint8 group);
inline void unhold_group(kal_uint8 group);

static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = OV50A_SENSOR_ID_21127,

    .checksum_value = 0x81c2c910,//test_Pattern_mode

    .pre = { //Res1_OV50A40_4096x3072_4C2SCG_10bit_30fps_AG64_PDVC1_4096x768_1872Msps_20211224
        .pclk = 75000000,
        .linelength = 600,
        .framelength = 4168,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 1247616000,
        .max_framerate = 300, /* 30fps */
    },
    .cap = { //Res1_OV50A40_4096x3072_4C2SCG_10bit_30fps_AG64_PDVC1_4096x768_1872Msps_20211224
        .pclk = 75000000,
        .linelength = 600,
        .framelength = 4168,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 1247616000,
        .max_framerate = 300, /* 30fps */
    },
    .normal_video = { //Res5_OV50A40_4096x2304_4C2SCG_10bit_30fps_AG64_PDVC1_4096x576_1872Msps_29211224
        .pclk = 75000000,
        .linelength = 450,
        .framelength = 5552,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 2304,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 1247616000,
        .max_framerate = 300, /* 30fps */
    },
    .hs_video = { //Res7_OV50A40_2048x1152_4C2SCG_10bit_120fps_AG64_PDVC1_4096x288_1872Msps_20211224
        .pclk = 75000000,
        .linelength = 450,
        .framelength = 1388,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2048,
        .grabwindow_height = 1152,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 1247616000,
        .max_framerate = 1200, /* 120fps */
    },
    .slim_video = { //Res1_OV50A40_4096x3072_4C2SCG_10bit_30fps_AG64_PDVC1_4096x768_1872Msps_20211224
        .pclk = 75000000,
        .linelength = 600,
        .framelength = 4168,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 1247616000,
        .max_framerate = 300, /* 30fps */
    },
    .custom1 = { //Res3_OV50A40_4096x3072_4C2SCG_10bit_24fps_AG64_PDVC1_4096x768_1872Msps_20211224
        .pclk = 75000000,
        .linelength = 600,
        .framelength = 5208,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 1247616000,
        .max_framerate = 240, /* 24fps */
    },
    .custom2 = { //Res4_OV50A40_4096x2304_4C2SCG_10bit_60fps_AG64_PDVC1_4096x576_1872Msps_29211224
        .pclk = 75000000,
        .linelength = 450,
        .framelength = 2776,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 2304,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 1247616000,
        .max_framerate = 600, /* 60fps */
    },
    .custom3 = { //Res0_OV50A40_8192x6144_10bit_30fps_AG16_PDVC1_4096x1536_3500Msps_20211224
        .pclk = 75000000,
        .linelength = 390,
        .framelength = 6412,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 8192,
        .grabwindow_height = 6144,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 2394000000,
        .max_framerate = 300, /* 30fps */
    },
    .custom4 = { //Res6_OV50A40_4096x2304_4C2SCG_STG2_10bit_30fps_AG64_PDVC2_4096x576_1872Msps_20211224
        .pclk = 75000000,
        .linelength = 450,
        .framelength = 5552, //2776*2
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 2304,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 1247616000,
        .max_framerate = 300, /* 30fps */
    },
    .custom5 = { //Res8_OV50A40_2048x1152_4C1SCG_10bit_240fps_AG64_1872Msps_20211224
        .pclk = 75000000,
        .linelength = 225,
        .framelength = 1388,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2048,
        .grabwindow_height = 1152,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 1247616000,
        .max_framerate = 2400, /* 240fps */
    },
    .custom6 = { //Res9_OV50A40_1280x720_4C1FastSpeed_10bit_480fps_AG16_1872Msps_20211224
        .pclk = 75000000,
        .linelength = 195,
        .framelength = 804,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 1247616000,
        .max_framerate = 4800, /* 480fps */
    },
    .custom7 = { //Res2_OV50A40_4096x3072_Cropping_10bit_30fps_AG64_PDVC1_2048x768_1872Msps_20211224
        .pclk = 75000000,
        .linelength = 600,
        .framelength = 4168,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 1247616000,
        .max_framerate = 300, /* 30fps */
    },
    .margin = 32,                    /* sensor framelength & shutter margin */
    .min_shutter = 16,                /* min shutter */
    .min_gain = 64, /*1x gain*/
    //.max_gain = 4096, /*64x gain*/
    .max_gain = 4080, /*63.75x gain*/
    .min_gain_iso = 100,
    .gain_step = 1,
    .gain_type = 1,/*to be modify,no gain table for sony*/
    .max_frame_length = 0xffffe9,     /* max framelength by sensor register's limitation */
    .ae_shut_delay_frame = 0,        //check
    .ae_sensor_gain_delay_frame = 0,//check
    .ae_ispGain_delay_frame = 2,
    .ihdr_support = 0,
    .ihdr_le_firstline = 0,
    .sensor_mode_num = 12,            //support sensor mode num

    .cap_delay_frame = 1,            //enter capture delay frame num
    .pre_delay_frame = 1,            //enter preview delay frame num
    .video_delay_frame = 2,            //enter video delay frame num
    .hs_video_delay_frame = 1,        //enter high speed video  delay frame num
    .slim_video_delay_frame = 1,    //enter slim video delay frame num
    .custom1_delay_frame = 1,        //enter custom1 delay frame num
    .custom2_delay_frame = 1,        //enter custom2 delay frame num
    .custom3_delay_frame = 1,        //enter custom3 delay frame num
    .custom4_delay_frame = 2,        //enter custom4 delay frame num
    .custom5_delay_frame = 1,        //enter custom5 delay frame num
	.custom6_delay_frame = 1,        //enter custom6 delay frame num
	.custom7_delay_frame = 1,        //enter custom7 delay frame num
    .frame_time_delay_frame = 2,

    .isp_driving_current = ISP_DRIVING_8MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_CPHY,
    .mipi_settle_delay_mode = 1,
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
    .mipi_lane_num = SENSOR_MIPI_3_LANE,//mipi lane num
    .i2c_addr_table = {0x20, 0xff},
    .i2c_speed = 1000,
};

static struct imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,
    .sensor_mode = IMGSENSOR_MODE_INIT,
    .shutter = 0x3D0,
    .gain = 0x100,
    .dummy_pixel = 0,
    .dummy_line = 0,
    .current_fps = 30,
    .autoflicker_en = KAL_FALSE,
    .test_pattern = KAL_FALSE,
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
    .ihdr_en = 0,
    .i2c_write_id = 0x20,
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[12] = {
    {8192, 6144, 000, 000, 8192, 6144, 4096, 3072,
    0000, 0000, 4096, 3072, 0, 0, 4096, 3072}, /* Preview */
    {8192, 6144, 000, 000, 8192, 6144, 4096, 3072,
    0000, 0000, 4096, 3072, 0, 0, 4096, 3072}, /* Capture */
    {8192, 6144, 000, 768, 8192, 4608, 4096, 2304,
    0000, 0000, 4096, 2304, 0, 0, 4096, 2304}, /* normal_video */
    {8192, 6144, 000, 768, 8192, 4608, 2048, 1152,
    0000, 0000, 2048, 1152, 0, 0, 2048, 1152}, /* hs_video */
    {8192, 6144, 000, 000, 8192, 6144, 4096, 3072,
    0000, 0000, 4096, 3072, 0, 0, 4096, 3072}, /* slim_video */
    {8192, 6144, 000, 000, 8192, 6144, 4096, 3072,
    0000, 0000, 4096, 3072, 0, 0, 4096, 3072}, /* custom1 */
    {8192, 6144, 000, 768, 8192, 4608, 4096, 2304,
    0000, 0000, 4096, 2304, 0, 0, 4096, 2304}, /* custom2 */
    {8192, 6144, 000, 000, 8192, 6144, 8192, 6144,
    0000, 0000, 8192, 6144, 0, 0, 8192, 6144}, /* custom3 */
    {8192, 6144, 000, 768, 8192, 4608, 4096, 2304,
    0000, 0000, 4096, 2304, 0, 0, 4096, 2304}, /* custom4 */
    {8192, 6144, 000, 768, 8192, 4608, 2048, 1152,
    0000, 0000, 2048, 1152, 0, 0, 2048, 1152}, /* custom5 */
    {8192, 6144, 000, 1632, 8192, 2880, 2048, 720,
     384, 0000, 1280, 720, 0, 0, 1280, 720},     /* custom6 */
    {8192, 6144, 000, 1536, 8192, 3072, 8192, 3072,
    2048, 0000, 4096, 3072, 0, 0, 4096, 3072}, /* custom7 */
};

//the index order of VC_STAGGER_NE/ME/SE in array identify the order they are read out in MIPI transfer
static struct SENSOR_VC_INFO2_STRUCT SENSOR_VC_INFO2[12] = {
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //preivew
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
            {VC_PDAF_STATS, 0x01, 0x30, 5120, 768},  //4096*10/8 = 5120
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //capture
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
            {VC_PDAF_STATS, 0x01, 0x30, 5120, 768},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //normal_video
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x900},
            {VC_PDAF_STATS, 0x02, 0x30, 5120, 576},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //hs_video
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x800, 0x480},
            {VC_PDAF_STATS, 0x01, 0x30, 2560, 288},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //slim_video
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
            {VC_PDAF_STATS, 0x01, 0x30, 5120, 768},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom1
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
            {VC_PDAF_STATS, 0x01, 0x30, 5120, 768},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom2
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x900},
            {VC_PDAF_STATS, 0x01, 0x30, 5120, 576},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom3     QRMSC 8192x6144_24FPS
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x2000, 0x1800},
            {VC_PDAF_STATS, 0x01, 0x30, 5120, 1536},
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom4
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x900},
            {VC_STAGGER_ME, 0x01, 0x2b, 0x1000, 0x900},
            {VC_PDAF_STATS, 0x02, 0x30, 5120, 576},
        },
        1
    },
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom5
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x800, 0x480},
//            {VC_PDAF_STATS, 0x00, 0x30, 2560, 288},
        },
        1
    },
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom6
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x500, 0x2d0},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom7
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xC00},
            {VC_PDAF_STATS, 0x01, 0x30, 5120, 768},
        },
        1
    }
};

static void get_vc_info_2(struct SENSOR_VC_INFO2_STRUCT *pvcinfo2, kal_uint32 scenario)
{
    switch (scenario) {
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[1],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[2],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[3],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_SLIM_VIDEO:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[4],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CUSTOM1:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[5],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CUSTOM2:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[6],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CUSTOM3:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[7],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CUSTOM4:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[8],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CUSTOM5:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[9],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CUSTOM6:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[10],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[11],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    default:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[0],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    }
}

/* #define RAW_TYPE_OVERRIDE     //it's PDO function define */
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX = 0,
    .i4PitchY = 0,
    .i4PairNum = 0,
    .i4SubBlkW = 0,
    .i4SubBlkH = 0,
    .i4PosL = {{0, 0} },
    .i4PosR = {{0, 0} },
    .i4BlockNumX = 0,
    .i4BlockNumY = 0,
    .i4LeFirst = 0,
    .i4Crop = {
        {0, 0}, {0, 0}, {0, 384}, {0, 192}, {0, 0},
        {0, 0}, {0, 384}, {0, 0}, {0, 384}, {0, 384}
    },  //{0, 1632}
    .iMirrorFlip = 0,
};

static kal_uint16 ov50a_table_write_cmos_sensor(
                    kal_uint16 *para, kal_uint32 len)
{
    char puSendCmd[I2C_BUFFER_LEN];
    kal_uint32 tosend, IDX;
    kal_uint16 addr = 0, addr_last = 0, data;

    tosend = 0;
    IDX = 0;
    while (len > IDX) {
        addr = para[IDX];
        {
            puSendCmd[tosend++] = (char)(addr >> 8);
            puSendCmd[tosend++] = (char)(addr & 0xFF);
            data = para[IDX + 1];
            puSendCmd[tosend++] = (char)(data & 0xFF);
            IDX += 2;
            addr_last = addr;
        }

        if ((I2C_BUFFER_LEN - tosend) < 3 ||
            len == IDX ||
            addr != addr_last) {
            iBurstWriteReg_multi(puSendCmd, tosend,
                imgsensor.i2c_write_id,
                3, imgsensor_info.i2c_speed);
            tosend = 0;
        }
    }
    return 0;
}

static kal_uint16 ov50a_burst_write_cmos_sensor(
                    kal_uint16 *para, kal_uint32 len)
{
    char puSendCmd[I2C_BUFFER_LEN];
    kal_uint32 tosend, IDX;
    kal_uint16 addr = 0, addr_last = 0, data;

    tosend = 0;
    IDX = 0;
    while (len > IDX) {
        addr = para[IDX];
        if(tosend==0)
        {
            puSendCmd[tosend++] = (char)(addr >> 8);
            puSendCmd[tosend++] = (char)(addr & 0xFF);
            data = para[IDX + 1];
            puSendCmd[tosend++] = (char)(data & 0xFF);
            IDX += 2;
            addr_last = addr;
        }
        else if(addr == addr_last+1)
        {
            data = para[IDX + 1];
            puSendCmd[tosend++] = (char)(data & 0xFF);
            addr_last = addr;
            IDX += 2;
        }

        if (( tosend>=I2C_BUFFER_LEN)||
            len == IDX ||
            addr != addr_last) {
            iBurstWriteReg_multi(puSendCmd, tosend,
                imgsensor.i2c_write_id,
                tosend, imgsensor_info.i2c_speed);
            tosend = 0;
        }
    }
    return 0;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte = 0;
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

#define EEPROM_I2C_ADDR     (0xA2)
#define EEPROM_QSC_ADDR     (0x1E30)
#define EEPROM_QSC_LENGTH   (3568)

#define SENSOR_QSC_ADDR1    (0x71f0)
#define SENSOR_QSC_LENGTH1  (32)
#define SENSOR_QSC_ADDR2    (0x7290)
#define SENSOR_QSC_LENGTH2  (3536)

static kal_uint8 qsc_buffer[EEPROM_QSC_LENGTH + 2] = {0x00};
static kal_uint16 OV50A_QSC_setting[EEPROM_QSC_LENGTH * 2] = {0x00};

static void read_cmos_eeprom_table(kal_uint16 addr, kal_uint8 *table, kal_uint32 size)
{
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

    iReadRegI2C(pusendcmd, 2, (u8 *)table, size, EEPROM_I2C_ADDR);
/*
	int i = 0;
    for (i = 0; i < size; i ++) {
       table[i] =  Eeprom_1ByteDataRead((addr+i), EEPROM_I2C_ADDR);
    }
*/
}

bool Checksum_EEPROM(kal_uint8 *buffer, kal_uint32 size, kal_uint8 read_checksum) {
	kal_uint32 i = 0;
	kal_uint32 sum = 0;
	for(i =0; i< size; i++) {
		sum += buffer[i];
	}
	sum %=255;
	if( (sum & 0xff) == read_checksum) {
		pr_info("checksum success  checksum = 0x%x", sum);
		return 1;
	} else {
		pr_info("checksum fail, read_checksum = 0x%x, calculation_checksum = 0x0x",
			read_checksum, sum);
		return 0;
	}
	return 0;
}

static void read_QSC_from_EEPROM()
{
	kal_uint16 idx = 0;
	kal_uint16 * QSC_setting = OV50A_QSC_setting;
//	kal_uint8 temp = 0;
	/*read otp data to distinguish module*/
	otp_flag = OTP_QSC_NONE;
	pr_info("OTP type: Custom Only");

	read_cmos_eeprom_table(EEPROM_QSC_ADDR, qsc_buffer, EEPROM_QSC_LENGTH + 2);

	if( qsc_buffer[EEPROM_QSC_LENGTH] != 0x01) {  //flag of qsc
		otp_flag = OTP_QSC_INVALED;
		pr_info("qsc err flag = %d", qsc_buffer[EEPROM_QSC_LENGTH]);
		return ;
	}

	if(Checksum_EEPROM(qsc_buffer, EEPROM_QSC_LENGTH, qsc_buffer[EEPROM_QSC_LENGTH + 1])) {
		otp_flag = OTP_QSC_CUSTOM;
	} else {
		otp_flag = OTP_QSC_INVALED;
		pr_info("qsc err readchecksum = %d", qsc_buffer[EEPROM_QSC_LENGTH]);
		return ;
	}

	for (idx = 0; idx < SENSOR_QSC_LENGTH1; idx++) {
		QSC_setting[2 * idx] = SENSOR_QSC_ADDR1 + idx;
		QSC_setting[2 * idx + 1] = qsc_buffer[idx];
	}

	for (idx = 0; idx < SENSOR_QSC_LENGTH2; idx++) {
		QSC_setting[2 * (idx + SENSOR_QSC_LENGTH1) ] =  SENSOR_QSC_ADDR2 + idx;
		QSC_setting[2 * (idx + SENSOR_QSC_LENGTH1) + 1] = qsc_buffer[SENSOR_QSC_LENGTH1 + idx];
	}
/*
	pr_info("[test] qsc_buffer[0x%x] =  0x%x",EEPROM_QSC_LENGTH-1 , qsc_buffer[EEPROM_QSC_LENGTH-1]);
	pr_info("[test] qsc_buffer[0x%x] =  0x%x",EEPROM_QSC_LENGTH , qsc_buffer[EEPROM_QSC_LENGTH]);
	pr_info("[test] qsc_buffer[0x%x] =  0x%x",EEPROM_QSC_LENGTH+1 , qsc_buffer[EEPROM_QSC_LENGTH+1]);

	temp = 0;
	read_cmos_eeprom_table(EEPROM_QSC_ADDR, &temp, 1);
	pr_info("[test] [%d] =  0x%x",EEPROM_QSC_ADDR , temp);

	temp = 0;
	read_cmos_eeprom_table(0x2c20, &temp, 1);
	pr_info("[test] [%d] =  0x%x",0x2c20 , temp);

	for (idx = 0; idx < EEPROM_QSC_LENGTH; idx++) {
		pr_info("[test] OV50A_QSC_setting[2 * %d]  0x%x,  0x%x",
		idx, QSC_setting[2 * idx], QSC_setting[2 * idx + 1]);
	}
*/
}

static void write_sensor_QSC(void)
{
	LOG_INF("write_sensor_QSC Start otp_flag = %d\n", otp_flag);
	if (otp_flag == OTP_QSC_CUSTOM) {
		ov50a_burst_write_cmos_sensor(OV50A_QSC_setting,
			sizeof(OV50A_QSC_setting) / sizeof(kal_uint16));
	}
	if (otp_flag == OTP_QSC_INVALED) {
		ov50a_burst_write_cmos_sensor(addr_data_pair_QPDC_ov50a,
			sizeof(addr_data_pair_QPDC_ov50a) / sizeof(kal_uint16));
	}
	LOG_INF("write_sensor_QSC End\n");
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pusendcmd[4] = {(char)(addr >> 8),
        (char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2CTiming(pusendcmd, 3, imgsensor.i2c_write_id,
        imgsensor_info.i2c_speed);
}

static void set_dummy(void)
{
	if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3 ||
		imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM7) {
		imgsensor.frame_length &= 0xFFFFFFFC;
	} else {
		imgsensor.frame_length &= 0xFFFFFFFE;
	}
	hold_group(0x00);
	write_cmos_sensor(0x380c, imgsensor.line_length >> 8);
	write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);
	if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM4) {
		write_cmos_sensor(0x380e, ((imgsensor.frame_length >> 1) >> 8) & 0xFF);
		write_cmos_sensor(0x380f, (imgsensor.frame_length >> 1) & 0xFE);
	} else {
		write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8) & 0xFF);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFE);
	}
	unhold_group(0x00);
	LOG_INF("line_length = %d, frame_length = %d", imgsensor.line_length, imgsensor.frame_length);
	LOG_INF("shutter = %d, secend_shutter = %d", imgsensor.shutter, imgsensor.secend_shutter);
	LOG_INF("gain = 0x%x, secend_gain = 0x%x", imgsensor.gain, imgsensor.secend_gain);
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
    kal_uint32 frame_length = imgsensor.frame_length;
	if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM4) {
		frame_length = imgsensor.pclk / (framerate * 10 * 2) / imgsensor.line_length;
	} else {
		frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	}

    spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ?
            frame_length : imgsensor.min_frame_length;
    imgsensor.dummy_line = imgsensor.frame_length -
        imgsensor.min_frame_length;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
    imgsensor.frame_length = imgsensor_info.max_frame_length;
    imgsensor.dummy_line = imgsensor.frame_length -
        imgsensor.min_frame_length;
    }
    if (min_framelength_en)
    imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);

}

static void set_max_framerate_video(UINT16 framerate, kal_bool min_framelength_en)
{
    set_max_framerate(framerate, min_framelength_en);
    set_dummy();
}

static void calc_shutter(kal_uint32 *shutter) {
	kal_uint16 realtime_fps = 0;
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (*shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = *shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	*shutter = (*shutter < imgsensor_info.min_shutter) ?
	imgsensor_info.min_shutter : *shutter;
	*shutter = (*shutter >
		(imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) :
		*shutter;

	if(imgsensor.autoflicker_en == KAL_TRUE){
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
			imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305) {
			realtime_fps = 296;
			set_max_framerate(realtime_fps, 0);
		}else if(realtime_fps >= 147 && realtime_fps <= 150) {
			realtime_fps = 146;
			set_max_framerate(realtime_fps, 0);
		}
	}

	if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3 ||
		imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM7) {
		*shutter &= 0xFFFFFFFC;
		imgsensor.frame_length &= 0xFFFFFFFC;
	} else {
		*shutter &= 0xFFFFFFFE;
		imgsensor.frame_length &= 0xFFFFFFFE;
	}
	LOG_INF("shutter =%d, framelength =%d, realtime_fps =%d\n",
			*shutter, imgsensor.frame_length, realtime_fps);
}

static void write_shutter_w_gph(kal_uint32 shutter, kal_bool gph)
{
	calc_shutter(&shutter);
	/*Warning : shutter must be even. Odd might happen Unexpected Results */
	if (_is_initFlag) {
		write_cmos_sensor(0x3840, (imgsensor.frame_length >> 16) & 0xFF);
		write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8) & 0xFF);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFE);
		write_cmos_sensor(0x3500, (shutter >> 16) & 0xFF);
		write_cmos_sensor(0x3501, (shutter >> 8) & 0xFF);
		write_cmos_sensor(0x3502, shutter & 0xFE);
		_is_initFlag = 0;
	} else {
		if (gph)
			hold_group(0x00);
		write_cmos_sensor(0x3840, (imgsensor.frame_length >> 16) & 0xFF);
		write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8) & 0xFF);
		write_cmos_sensor(0x380f, (imgsensor.frame_length) & 0xFE);
		write_cmos_sensor(0x3500, (shutter >> 16) & 0xFF);
		write_cmos_sensor(0x3501, (shutter >> 8) & 0xFF);
		write_cmos_sensor(0x3502, shutter & 0xFE);
		if (gph)
			unhold_group(0x00);
	}
}

static void set_shutter_w_gph(kal_uint32 shutter, kal_bool gph)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	write_shutter_w_gph(shutter, gph);
}

static void set_shutter(kal_uint32 shutter)  //should not be kal_uint16 -- can't reach long exp
{
	set_shutter_w_gph(shutter, KAL_TRUE);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 iReg = 0x0000;

	//platform 1xgain = 64, sensor driver 1*gain = 0x100
	iReg = gain*256/BASEGAIN;

	if(iReg < 0x100)    //sensor 1xGain
	{
		iReg = 0X100;
	}
	if(iReg > 0x3fc0)    //sensor 63.75xGain
	{
		iReg = 0x3fc0;
	}
	return iReg;        /* sensorGlobalGain */
}

static void calc_gain(kal_uint16 gain, kal_uint16 *reg_gain)
{
	kal_uint16 max_gain = imgsensor_info.max_gain;
	unsigned long flags;

	if (gain < imgsensor_info.min_gain || gain > max_gain) {
		LOG_INF("Error gain setting");

		if (gain < imgsensor_info.min_gain)
			gain = imgsensor_info.min_gain;
		else if (gain > max_gain)
			gain = max_gain;
	}

	if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3 ||
			imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM6) {
		if (gain > 1020) {
			LOG_INF("custom3 custom6 maxgain 15.9375x");
			gain = 1020;
		}
	}

	*reg_gain = gain2reg(gain);
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.gain = *reg_gain;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, *reg_gain);
}

static void set_gain_w_gph(kal_uint16 gain, kal_bool gph)
{
	kal_uint16 reg_gain = 0;
    calc_gain(gain, &reg_gain);
	if(gph) {
		hold_group(0x0a);
	}
	write_cmos_sensor(0x3508, (reg_gain >> 8));
	write_cmos_sensor(0x3509, (reg_gain & 0xff));
	if(gph) {
		unhold_group(0x0a);
	}
}
static void set_gain(kal_uint16 gain)
{
	return set_gain_w_gph(gain, KAL_FALSE);
}
/* ITD: Modify Dualcam By Jesse 190924 Start */
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 target_frame_length, kal_bool auto_extend_en)
{
	spin_lock(&imgsensor_drv_lock);
	if(target_frame_length > 1)
		imgsensor.dummy_line = target_frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + imgsensor.dummy_line;
	imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_shutter(shutter);
}


inline void hold_group(kal_uint8 group)
{
//	LOG_INF("[test] %s  group=0x%x", __func__, group & 0x0f);
	write_cmos_sensor(0x3208, group & 0x0f);
}
inline void unhold_group(kal_uint8 group)
{
//	LOG_INF("[test] %s  group=0x%x", __func__, group & 0x0f);
	write_cmos_sensor(0x3208, (group & 0x0f)|0x10);
	write_cmos_sensor(0x3208, (group & 0x0f)|0xa0);
}


static void sensor_init(void)
{
	write_cmos_sensor(0x0103, 0x01);//SW Reset, need delay
	mdelay(5);
	LOG_INF("sensor_init start\n");
	ov50a_table_write_cmos_sensor(
			addr_data_pair_init_ov50a,
			sizeof(addr_data_pair_init_ov50a) / sizeof(kal_uint16));
	_is_initFlag = 1;
	LOG_INF("sensor_init end\n");
}

static void preview_setting(void)
{
	int _length = 0;

	LOG_INF("preview_setting\n");
	_length = sizeof(addr_data_pair_preview_ov50a) / sizeof(kal_uint16);
	ov50a_table_write_cmos_sensor(
			addr_data_pair_preview_ov50a,
			_length);
	LOG_INF("preview_setting_end\n");
}


static void capture_setting(kal_uint16 currefps)
{
	int _length = 0;

	LOG_INF("capture_setting currefps = %d\n", currefps);
	_length = sizeof(addr_data_pair_preview_ov50a) / sizeof(kal_uint16);
	ov50a_table_write_cmos_sensor(
			addr_data_pair_capture_ov50a,
			_length);
}

static void normal_video_setting(kal_uint16 currefps)
{
	int _length = 0;

	LOG_INF("normal_video_setting\n");
	_length = sizeof(addr_data_pair_video_ov50a) / sizeof(kal_uint16);
	ov50a_table_write_cmos_sensor(
			addr_data_pair_video_ov50a,
			_length);
}

static void hs_video_setting(void)
{
	int _length = 0;

	LOG_INF("hs_video_setting\n");
	_length = sizeof(addr_data_pair_hs_video_ov50a) / sizeof(kal_uint16);
	ov50a_table_write_cmos_sensor(
			addr_data_pair_hs_video_ov50a,
			_length);
}

static void slim_video_setting(void)
{
	int _length = 0;

	LOG_INF("slim_video_setting\n");
	_length = sizeof(addr_data_pair_slim_video_ov50a) / sizeof(kal_uint16);
	ov50a_table_write_cmos_sensor(
			addr_data_pair_slim_video_ov50a,
			_length);
}

/* ITD: Modify Dualcam By Jesse 190924 Start */
static void custom1_setting(void)
{
	int _length = 0;

	LOG_INF("custom1_setting_start\n");
	_length = sizeof(addr_data_pair_custom1) / sizeof(kal_uint16);
	ov50a_table_write_cmos_sensor(
			addr_data_pair_custom1,
			_length);
	LOG_INF("custom1_setting_end\n");
}    /*    custom1_setting  */

static void custom2_setting(void)
{
	int _length = 0;

	LOG_INF("custom2_setting_start\n");
	_length = sizeof(addr_data_pair_custom2) / sizeof(kal_uint16);
	ov50a_table_write_cmos_sensor(
			addr_data_pair_custom2,
			_length);
	LOG_INF("custom2_setting_end\n");
}    /*    custom2_setting  */

static void custom3_setting(void)
{
	int _length = 0;

	LOG_INF("E\n");
	ov50a_burst_write_cmos_sensor(addr_data_pair_custom3,
			sizeof(addr_data_pair_custom3)/sizeof(kal_uint16));

	_length = sizeof(addr_data_pair_custom3) / sizeof(kal_uint16);
	ov50a_table_write_cmos_sensor(
			addr_data_pair_custom3,
			_length);
}    /*    custom3_setting  */

static void custom4_setting(void)
{
	int _length = 0;

	LOG_INF("custom4_setting_start\n");
	_length = sizeof(addr_data_pair_custom4) / sizeof(kal_uint16);
	ov50a_table_write_cmos_sensor(
			addr_data_pair_custom4,
			_length);

	LOG_INF("custom4_setting_end\n");
}    /*    custom4_setting  */

static void custom5_setting(void)
{
	int _length = 0;

	LOG_INF("E\n");
	_length = sizeof(addr_data_pair_custom5) / sizeof(kal_uint16);
	ov50a_table_write_cmos_sensor(
			addr_data_pair_custom5,
			_length);
}    /*    custom5_setting  */

static void custom6_setting(void)
{
	int _length = 0;

	LOG_INF("E\n");
	_length = sizeof(addr_data_pair_custom6) / sizeof(kal_uint16);
	ov50a_table_write_cmos_sensor(
			addr_data_pair_custom6,
			_length);
}    /*    custom6_setting  */

static void custom7_setting(void)
{
	int _length = 0;

	LOG_INF("E\n");
	_length = sizeof(addr_data_pair_custom7) / sizeof(kal_uint16);
	ov50a_table_write_cmos_sensor(
			addr_data_pair_custom7,
			_length);
}    /*    custom7_setting  */

static void calc_dual_gain(kal_uint16 lgain, kal_uint16 sgain, kal_uint16 *reg_lg, kal_uint16 *reg_sg)
{
	if (lgain < BASEGAIN || lgain > 64 * BASEGAIN) {
		LOG_INF("Error lgain setting");
		if (lgain < BASEGAIN)
			lgain = BASEGAIN;
		else if (lgain > 64 * BASEGAIN)
			lgain = 64 * BASEGAIN;
	}
	if (sgain < BASEGAIN || sgain > 64 * BASEGAIN) {
		LOG_INF("Error sgain setting");
		if (sgain < BASEGAIN)
			sgain = BASEGAIN;
		else if (sgain > 64 * BASEGAIN)
			sgain = 64 * BASEGAIN;
	}

	*reg_lg = gain2reg(lgain);
	*reg_sg = gain2reg(sgain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = *reg_lg;
	imgsensor.secend_gain = *reg_sg;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("lgain:%d, reg_lg:0x%x, sgain:%d, reg_sg:0x%x\n",
			lgain, *reg_lg, sgain, *reg_sg);
}
static void hdr_write_dual_gain(kal_uint16 lgain, kal_uint16 sgain)
{
	kal_uint16 reg_lg, reg_sg;

	calc_dual_gain(lgain, sgain, &reg_lg, &reg_sg);
	/* Long exposure */
	if (reg_lg) {
		write_cmos_sensor(0x3508, (reg_lg >> 8));
		write_cmos_sensor(0x3509, (reg_lg & 0xFF));
	}
	/* Muddle exposure */
	if (reg_sg) {
		write_cmos_sensor(0x3548, (reg_sg >> 8));
		write_cmos_sensor(0x3549, (reg_sg & 0xFF));
	}
}

static void calc_dual_shutter(kal_uint16 *le, kal_uint16 *se) {
	kal_uint16 realtime_fps = 0;
	kal_uint16 exposure_cnt = 2;
	kal_uint16 pre_se = read_cmos_sensor(0x3541) << 8  | read_cmos_sensor(0x3542);

	*le = (*le / exposure_cnt);
	*se = (*se / exposure_cnt);
	*le = (kal_uint16)max(imgsensor_info.min_shutter, (kal_uint32)*le);
	*se = (kal_uint16)max(imgsensor_info.min_shutter, (kal_uint32)*se);
	if (*le < *se) {
		*se = *le;
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = max((kal_uint32)((*le + pre_se + (exposure_cnt * imgsensor_info.margin)) * exposure_cnt), imgsensor.min_frame_length);
	imgsensor.frame_length = max((kal_uint32)((*le + *se + (exposure_cnt * imgsensor_info.margin)) * exposure_cnt), imgsensor.frame_length);
	imgsensor.frame_length = min(imgsensor.frame_length, imgsensor_info.max_frame_length);
	imgsensor.shutter = *le;
	imgsensor.secend_shutter = *se;
	spin_unlock(&imgsensor_drv_lock);

	if(imgsensor.autoflicker_en == KAL_TRUE){
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
			imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305) {
			realtime_fps = 296;
			set_max_framerate(realtime_fps, 0);
		}else if(realtime_fps >= 147 && realtime_fps <= 150) {
			realtime_fps = 146;
			set_max_framerate(realtime_fps, 0);
		}
	}
	LOG_INF("le =%d, se =%d, pre_se =%d, framelength =%d, realtime_fps =%d min_frame_length=%d\n",
			*le, *se, pre_se, imgsensor.frame_length, realtime_fps, imgsensor.min_frame_length);
}

static void hdr_write_dual_shutter(kal_uint16 le, kal_uint16 se)
{
	calc_dual_shutter(&le, &se);
	/*Warning : shutter must be even. Odd might happen Unexpected Results */
	hold_group(0x00);
	write_cmos_sensor(0x3840, (imgsensor.frame_length >> 1) >> 16 & 0xFF);
	write_cmos_sensor(0x380e, (imgsensor.frame_length >> 1) >> 8 & 0xFF);
	write_cmos_sensor(0x380f, (imgsensor.frame_length >> 1) & 0xFE);
	/* Long exposure */
	if (le != 0) {
		write_cmos_sensor(0x3500, 0x00);
		write_cmos_sensor(0x3501, (le >> 8) & 0xFF);
		write_cmos_sensor(0x3502, (le) & 0xFE);
	}
	/* Muddle exposure */
	if (se != 0) {
		write_cmos_sensor(0x3540, 0x00);
		write_cmos_sensor(0x3541, (se >> 8) & 0xFF);
		write_cmos_sensor(0x3542, (se) & 0xFE);
	}
	unhold_group(0x00);
}

static void seamless_switch_to_custom4(
    kal_uint32 shutter, kal_uint32 gain,
    kal_uint32 shutter_2ndframe, kal_uint32 gain_2ndframe)
{
    int _length = 0;
	kal_uint16 le = (kal_uint16)shutter;
	kal_uint16 se = (kal_uint16)shutter_2ndframe;
	kal_uint16 reg_lgain = 0;
	kal_uint16 reg_sgain = 0;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
	imgsensor.pclk = imgsensor_info.custom4.pclk;
	imgsensor.line_length = imgsensor_info.custom4.linelength;
	imgsensor.frame_length = imgsensor_info.custom4.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	calc_dual_shutter(&le, &se);
	calc_dual_gain((kal_uint16)gain, (kal_uint16)gain_2ndframe, &reg_lgain, &reg_sgain);
	if (le != 0) {
		pair_seamless_switch_custom4[2 * 21 + 1] = (le >> 8) & 0xFF;
		pair_seamless_switch_custom4[2 * 22 + 1] = le & 0xFE;
	}
	if (reg_lgain != 0) {
		pair_seamless_switch_custom4[2 * 23 + 1] = (reg_lgain >> 8) & 0xFF;
		pair_seamless_switch_custom4[2 * 24 + 1] = reg_lgain & 0xFF;
	}
	if (se != 0) {
		pair_seamless_switch_custom4[2 * 25 + 1] = (se >> 8) & 0xFF;
		pair_seamless_switch_custom4[2 * 26 + 1] = se & 0xFE;
	}
	if (reg_sgain != 0) {
		pair_seamless_switch_custom4[2 * 27 + 1] = (reg_sgain >> 8) & 0xFF;
		pair_seamless_switch_custom4[2 * 28 + 1] = reg_sgain & 0xFF;
	}
	pair_seamless_switch_custom4[2 * 29 + 1] = ((imgsensor.frame_length >> 1) >> 8) & 0xFF;
	pair_seamless_switch_custom4[2 * 30 + 1] = (imgsensor.frame_length >> 1) & 0xFE;

	LOG_INF("seamless switch to custom4 start\n");
	_length = sizeof(pair_seamless_switch_custom4) / sizeof(kal_uint16);
	ov50a_table_write_cmos_sensor(
			pair_seamless_switch_custom4,
			_length);

	LOG_INF("seamless switch to custom4 end\n");
}    /*    seamless_switch_to_custom4  */

static void seamless_switch_to_normal_video(kal_uint32 shutter, kal_uint32 gain)
{
    int _length = 0;
	kal_uint16 reg_gain = 0;;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = MSDK_SCENARIO_ID_VIDEO_PREVIEW;
	imgsensor.pclk = imgsensor_info.normal_video .pclk;
	imgsensor.line_length = imgsensor_info.normal_video .linelength;
	imgsensor.frame_length = imgsensor_info.normal_video .framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video .framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	calc_shutter(&shutter);
	calc_gain(gain, &reg_gain);
	if (shutter != 0)  {
		pair_seamless_switch_normal_video[21*2 +1] = (shutter >> 8) & 0xFF;
		pair_seamless_switch_normal_video[22*2 +1] = shutter & 0xFE;
	}
	if (reg_gain != 0) {
		pair_seamless_switch_normal_video[23 * 2 +1] = (reg_gain >> 8) & 0xFF;
		pair_seamless_switch_normal_video[24 * 2 +1] = reg_gain  & 0xFF;
	}
	pair_seamless_switch_normal_video[25 * 2 +1] = (imgsensor.frame_length >> 8) & 0xFF;
	pair_seamless_switch_normal_video[26 * 2 +1] = (imgsensor.frame_length) & 0xFE;

	LOG_INF("seamless switch to normal video start\n");
	_length = sizeof(pair_seamless_switch_normal_video) / sizeof(kal_uint16);
	ov50a_table_write_cmos_sensor(
			pair_seamless_switch_normal_video,
			_length);

	LOG_INF("seamless switch to normal video end\n");
}    /*    seamless_switch_to_normal_video   */

static void seamless_switch_to_custom7(kal_uint32 shutter, kal_uint32 gain)
{
    int _length = 0;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM7;
	imgsensor.pclk = imgsensor_info.custom7.pclk;
	imgsensor.line_length = imgsensor_info.custom7.linelength;
	imgsensor.frame_length = imgsensor_info.custom7.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom7.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	LOG_INF("seamless switch to custom7 start\n");
	hold_group(0x01);
	_length = sizeof(pair_seamless_switch_custom7) / sizeof(kal_uint16);
	ov50a_table_write_cmos_sensor(
			pair_seamless_switch_custom7,
			_length);
	set_shutter_w_gph(shutter, KAL_FALSE);
	set_gain_w_gph(gain, KAL_FALSE);
	unhold_group(0x01);
	LOG_INF("seamless switch to custom7 end\n");
}    /*    seamless_switch_to_custom7  */

static void seamless_switch_to_preview(kal_uint32 shutter, kal_uint32 gain)
{
    int _length = 0;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode =  MSDK_SCENARIO_ID_CAMERA_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	LOG_INF("seamless switch to preview start\n");
	hold_group(0x01);
	_length = sizeof(pair_seamless_switch_preview) / sizeof(kal_uint16);
	ov50a_table_write_cmos_sensor(
			pair_seamless_switch_preview,
			_length);
	set_shutter_w_gph(shutter, KAL_FALSE);
	set_gain_w_gph(gain, KAL_FALSE);
	unhold_group(0x01);
	LOG_INF("seamless switch to preview end\n");
}    /*    seamless_switch_to_preview*/

static kal_uint32 return_sensor_id(void)
{
    return ((read_cmos_sensor(0x300a) << 8) | (read_cmos_sensor(0x300b)));
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;

    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == OV50A_SENSOR_ID) {
				*sensor_id = imgsensor_info.sensor_id;
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);

				read_QSC_from_EEPROM();
				if(deviceInfo_register_value == 0x00){
					Eeprom_DataInit(IMGSENSOR_SENSOR_IDX_MAIN, imgsensor_info.sensor_id);
					deviceInfo_register_value = 0x01;
				}
				LOG_INF("RENM0_module_id=%d\n",imgsensor_info.module_id);
	//            Eeprom_DataInit(IMGSENSOR_SENSOR_IDX_MAIN, *sensor_id);
				return ERROR_NONE;
			}
			retry--;
		} while (retry > 0);
		i++;
		retry = 1;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        LOG_INF("get_imgsensor_id: 0x%x fail\n", *sensor_id);
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    return ERROR_NONE;
}

static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 1;
    kal_uint32 sensor_id = 0;

    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
		if (sensor_id ==OV50A_SENSOR_ID) {
			sensor_id = imgsensor_info.sensor_id;
			LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
			break;
		}
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id) {
        LOG_INF("Open sensor id: 0x%x fail\n", sensor_id);
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    sensor_init();

    if (!qsc_flag) {
        write_sensor_QSC();
        qsc_flag = 1;
    }

    spin_lock(&imgsensor_drv_lock);
    imgsensor.autoflicker_en = KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.shutter = 0x3D0;
    imgsensor.gain = 0x100;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}

static kal_uint32 close(void)
{
    qsc_flag = 0;
    return ERROR_NONE;
}   /*  close  */

static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.current_fps = imgsensor.current_fps;
    //imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}

static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    imgsensor.pclk = imgsensor_info.cap.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.cap.linelength;
    imgsensor.frame_length = imgsensor_info.cap.framelength;
    imgsensor.min_frame_length = imgsensor_info.cap.framelength;
    //imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    capture_setting(imgsensor.current_fps);
    return ERROR_NONE;
} /* capture() */

static kal_uint32 normal_video(
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    //imgsensor.current_fps = 300;
    //imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    normal_video_setting(imgsensor.current_fps);
    return ERROR_NONE;
}

static kal_uint32 hs_video(
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();
    return ERROR_NONE;
}

static kal_uint32 slim_video(
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    if (1) {
        preview_setting();
    } else {
        slim_video_setting();
    }
    return ERROR_NONE;
}

/* ITD: Modify Dualcam By Jesse 190924 Start */
static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom1_setting();
    return ERROR_NONE;
}   /*  Custom1   */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom2_setting();
    return ERROR_NONE;
}   /*  Custom2   */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom3_setting();
    return ERROR_NONE;
}   /*  Custom3*/


static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    imgsensor.pclk = imgsensor_info.custom4.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom4.linelength;
    imgsensor.frame_length = imgsensor_info.custom4.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom4_setting();
    return ERROR_NONE;
}   /*  Custom4    */

static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
    imgsensor.pclk = imgsensor_info.custom5.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom5.linelength;
    imgsensor.frame_length = imgsensor_info.custom5.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom5_setting();
    return ERROR_NONE;
}/*    Custom5 */

static kal_uint32 Custom6(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM6;
    imgsensor.pclk = imgsensor_info.custom6.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom6.linelength;
    imgsensor.frame_length = imgsensor_info.custom6.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom6.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom6_setting();
    return ERROR_NONE;
}/*    Custom6 */

static kal_uint32 Custom7(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM7;
    imgsensor.pclk = imgsensor_info.custom7.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom7.linelength;
    imgsensor.frame_length = imgsensor_info.custom7.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom7.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom7_setting();
    return ERROR_NONE;
}/*    Custom7 */

static kal_uint32 get_resolution(
        MSDK_SENSOR_RESOLUTION_INFO_STRUCT * sensor_resolution)
{
    sensor_resolution->SensorFullWidth =
        imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight =
        imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth =
        imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight =
        imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth =
        imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight =
        imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth =
        imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight =
        imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth =
        imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight =
        imgsensor_info.slim_video.grabwindow_height;

/* ITD: Modify Dualcam By Jesse 190924 Start */
    sensor_resolution->SensorCustom1Width =
        imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height =
        imgsensor_info.custom1.grabwindow_height;

    sensor_resolution->SensorCustom2Width =
        imgsensor_info.custom2.grabwindow_width;
    sensor_resolution->SensorCustom2Height =
        imgsensor_info.custom2.grabwindow_height;

    sensor_resolution->SensorCustom3Width =
        imgsensor_info.custom3.grabwindow_width;
    sensor_resolution->SensorCustom3Height =
        imgsensor_info.custom3.grabwindow_height;

    sensor_resolution->SensorCustom4Width =
        imgsensor_info.custom4.grabwindow_width;
    sensor_resolution->SensorCustom4Height =
        imgsensor_info.custom4.grabwindow_height;

    sensor_resolution->SensorCustom5Width =
        imgsensor_info.custom5.grabwindow_width;
    sensor_resolution->SensorCustom5Height =
        imgsensor_info.custom5.grabwindow_height;

    sensor_resolution->SensorCustom6Width =
        imgsensor_info.custom6.grabwindow_width;
    sensor_resolution->SensorCustom6Height =
        imgsensor_info.custom6.grabwindow_height;

    sensor_resolution->SensorCustom7Width =
        imgsensor_info.custom7.grabwindow_width;
    sensor_resolution->SensorCustom7Height =
        imgsensor_info.custom7.grabwindow_height;

    return ERROR_NONE;
}   /*  get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
              MSDK_SENSOR_INFO_STRUCT *sensor_info,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    if (scenario_id == 0)
    LOG_INF("scenario_id = %d\n", scenario_id);

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat =
        imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame =
        imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame =
        imgsensor_info.slim_video_delay_frame;

    sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
    sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
    sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
    sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
    sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;
	sensor_info->Custom6DelayFrame = imgsensor_info.custom6_delay_frame;
	sensor_info->Custom7DelayFrame = imgsensor_info.custom7_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
/* The frame of setting shutter default 0 for TG int */
    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
    /* The frame of setting sensor gain */
    sensor_info->AESensorGainDelayFrame =
        imgsensor_info.ae_sensor_gain_delay_frame;
    sensor_info->AEISPGainDelayFrame =
        imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
    sensor_info->PDAF_Support = PDAF_SUPPORT_CAMSV_QPD;

    sensor_info->HDR_Support = HDR_SUPPORT_STAGGER; /*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR*/
    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;   // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

    break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

    break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

        sensor_info->SensorGrabStartX =
            imgsensor_info.normal_video.startx;
        sensor_info->SensorGrabStartY =
            imgsensor_info.normal_video.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

    break;
    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
    break;
    case MSDK_SCENARIO_ID_SLIM_VIDEO:
        sensor_info->SensorGrabStartX =
            imgsensor_info.slim_video.startx;
        sensor_info->SensorGrabStartY =
            imgsensor_info.slim_video.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
    break;
    case MSDK_SCENARIO_ID_CUSTOM1:
        sensor_info->SensorGrabStartX =
            imgsensor_info.custom1.startx;
        sensor_info->SensorGrabStartY =
            imgsensor_info.custom1.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
    break;
    case MSDK_SCENARIO_ID_CUSTOM2:
        sensor_info->SensorGrabStartX =
            imgsensor_info.custom2.startx;
        sensor_info->SensorGrabStartY =
            imgsensor_info.custom2.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;
    break;
    case MSDK_SCENARIO_ID_CUSTOM3:
        sensor_info->SensorGrabStartX =
            imgsensor_info.custom3.startx;
        sensor_info->SensorGrabStartY =
            imgsensor_info.custom3.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;
    break;

    case MSDK_SCENARIO_ID_CUSTOM4:
        sensor_info->SensorGrabStartX =
            imgsensor_info.custom4.startx;
        sensor_info->SensorGrabStartY =
            imgsensor_info.custom4.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom4.mipi_data_lp2hs_settle_dc;
    break;
    case MSDK_SCENARIO_ID_CUSTOM5:
        sensor_info->SensorGrabStartX =
            imgsensor_info.custom5.startx;
        sensor_info->SensorGrabStartY =
            imgsensor_info.custom5.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom5.mipi_data_lp2hs_settle_dc;
    break;
    case MSDK_SCENARIO_ID_CUSTOM6:
        sensor_info->SensorGrabStartX =
            imgsensor_info.custom6.startx;
        sensor_info->SensorGrabStartY =
            imgsensor_info.custom6.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom6.mipi_data_lp2hs_settle_dc;
    break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        sensor_info->SensorGrabStartX =
            imgsensor_info.custom7.startx;
        sensor_info->SensorGrabStartY =
            imgsensor_info.custom7.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom7.mipi_data_lp2hs_settle_dc;
    break;
    default:
        sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
    break;
    }

    return ERROR_NONE;
}   /*  get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);

    switch (scenario_id) {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        preview(image_window, sensor_config_data);
    break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        capture(image_window, sensor_config_data);
    break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        normal_video(image_window, sensor_config_data);
    break;
    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        hs_video(image_window, sensor_config_data);
    break;
    case MSDK_SCENARIO_ID_SLIM_VIDEO:
        slim_video(image_window, sensor_config_data);
    break;
    case MSDK_SCENARIO_ID_CUSTOM1:
        Custom1(image_window, sensor_config_data);
    break;
    case MSDK_SCENARIO_ID_CUSTOM2:
        Custom2(image_window, sensor_config_data);
    break;
    case MSDK_SCENARIO_ID_CUSTOM3:
        Custom3(image_window, sensor_config_data);
    break;
    case MSDK_SCENARIO_ID_CUSTOM4:
        Custom4(image_window, sensor_config_data);
    break;
    case MSDK_SCENARIO_ID_CUSTOM5:
        Custom5(image_window, sensor_config_data);
    break;
    case MSDK_SCENARIO_ID_CUSTOM6:
        Custom6(image_window, sensor_config_data);
    break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        Custom7(image_window, sensor_config_data);
    break;
    default:
        LOG_INF("Error ScenarioId setting");
        preview(image_window, sensor_config_data);
    return ERROR_INVALID_SCENARIO_ID;
    }

    return ERROR_NONE;
}   /* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
    // SetVideoMode Function should fix framerate
    if (framerate == 0)
    // Dynamic frame rate
    return ERROR_NONE;

    spin_lock(&imgsensor_drv_lock);
    if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
    imgsensor.current_fps = 296;
    else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
    imgsensor.current_fps = 146;
    else
    imgsensor.current_fps = framerate;
    spin_unlock(&imgsensor_drv_lock);

    set_max_framerate_video(imgsensor.current_fps, 1);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable,
            UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d\n",
        enable, framerate);

    spin_lock(&imgsensor_drv_lock);
    if (enable) //enable auto flicker
    imgsensor.autoflicker_en = KAL_TRUE;
    else //Cancel Auto flick
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(
            enum MSDK_SCENARIO_ID_ENUM scenario_id,
            MUINT32 framerate)
{
	kal_uint32 frameHeight;
	kal_uint32 m_dummyline;
	kal_uint32 m_framelength;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	if (framerate == 0)
		return ERROR_NONE;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frameHeight = imgsensor_info.pre.pclk / framerate * 10 /
				imgsensor_info.pre.linelength;
			m_dummyline =
				(frameHeight > imgsensor_info.pre.framelength) ?
				(frameHeight - imgsensor_info.pre.framelength):0;
			m_framelength = imgsensor_info.pre.framelength + m_dummyline;
			m_framelength = round_up(m_framelength, 2);
			if(m_framelength > imgsensor.shutter + imgsensor_info.margin) {
				spin_lock(&imgsensor_drv_lock);
				imgsensor.frame_length = m_framelength;
				imgsensor.min_frame_length = imgsensor.frame_length;
				imgsensor.dummy_line = m_dummyline;
				spin_unlock(&imgsensor_drv_lock);
				set_dummy();
			} else {
				LOG_INF("new framelength(%d) <= shutter(%d) + margin(%d)",
					m_framelength, imgsensor.shutter, imgsensor_info.margin);
				LOG_INF("current framelength(%d)", imgsensor.frame_length);
			}
		break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			frameHeight = imgsensor_info.normal_video.pclk / framerate * 10 /
				imgsensor_info.normal_video.linelength;
			m_dummyline =
				(frameHeight > imgsensor_info.normal_video.framelength) ?
				(frameHeight - imgsensor_info.normal_video.framelength):0;
			m_framelength = imgsensor_info.normal_video.framelength + m_dummyline;
			m_framelength = round_up(m_framelength, 2);
			if(m_framelength > imgsensor.shutter + imgsensor_info.margin) {
				spin_lock(&imgsensor_drv_lock);
				imgsensor.frame_length = m_framelength;
				imgsensor.min_frame_length = imgsensor.frame_length;
				imgsensor.dummy_line = m_dummyline;
				spin_unlock(&imgsensor_drv_lock);
				set_dummy();
			} else {
				LOG_INF("new framelength(%d) <= shutter(%d) + margin(%d)",
					m_framelength, imgsensor.shutter, imgsensor_info.margin);
				LOG_INF("current framelength(%d)", imgsensor.frame_length);
			}
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			frameHeight = imgsensor_info.cap.pclk / framerate * 10 /
				imgsensor_info.cap.linelength;
			m_dummyline =
				(frameHeight > imgsensor_info.cap.framelength) ?
				(frameHeight - imgsensor_info.cap.framelength):0;
			m_framelength = imgsensor_info.cap.framelength + m_dummyline;
			m_framelength = round_up(m_framelength, 2);
			if(m_framelength > imgsensor.shutter + imgsensor_info.margin) {
				spin_lock(&imgsensor_drv_lock);
				imgsensor.frame_length = m_framelength;
				imgsensor.min_frame_length = imgsensor.frame_length;
				imgsensor.dummy_line = m_dummyline;
				spin_unlock(&imgsensor_drv_lock);
				set_dummy();
			} else {
				LOG_INF("new framelength(%d) <= shutter(%d) + margin(%d)",
					m_framelength, imgsensor.shutter, imgsensor_info.margin);
				LOG_INF("current framelength(%d)", imgsensor.frame_length);
			}
		break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frameHeight = imgsensor_info.hs_video.pclk / framerate * 10 /
				imgsensor_info.hs_video.linelength;
			m_dummyline =
				(frameHeight > imgsensor_info.hs_video.framelength) ?
				(frameHeight - imgsensor_info.hs_video.framelength):0;
			m_framelength = imgsensor_info.hs_video.framelength + m_dummyline;
			m_framelength = round_up(m_framelength, 2);
			if(m_framelength > imgsensor.shutter + imgsensor_info.margin) {
				spin_lock(&imgsensor_drv_lock);
				imgsensor.frame_length = m_framelength;
				imgsensor.min_frame_length = imgsensor.frame_length;
				imgsensor.dummy_line = m_dummyline;
				spin_unlock(&imgsensor_drv_lock);
				set_dummy();
			} else {
				LOG_INF("new framelength(%d) <= shutter(%d) + margin(%d)",
					m_framelength, imgsensor.shutter, imgsensor_info.margin);
				LOG_INF("current framelength(%d)", imgsensor.frame_length);
			}
		break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frameHeight = imgsensor_info.slim_video.pclk / framerate * 10 /
				imgsensor_info.slim_video.linelength;
			m_dummyline =
				(frameHeight > imgsensor_info.slim_video.framelength) ?
				(frameHeight - imgsensor_info.slim_video.framelength):0;
			m_framelength = imgsensor_info.slim_video.framelength + m_dummyline;
			m_framelength = round_up(m_framelength, 2);
			if(m_framelength > imgsensor.shutter + imgsensor_info.margin) {
				spin_lock(&imgsensor_drv_lock);
				imgsensor.frame_length = m_framelength;
				imgsensor.min_frame_length = imgsensor.frame_length;
				imgsensor.dummy_line = m_dummyline;
				spin_unlock(&imgsensor_drv_lock);
				set_dummy();
			} else {
				LOG_INF("new framelength(%d) <= shutter(%d) + margin(%d)",
					m_framelength, imgsensor.shutter, imgsensor_info.margin);
				LOG_INF("current framelength(%d)", imgsensor.frame_length);
			}
		break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			frameHeight = imgsensor_info.custom1.pclk / framerate * 10 /
				imgsensor_info.custom1.linelength;
			m_dummyline =
				(frameHeight > imgsensor_info.custom1.framelength) ?
				(frameHeight - imgsensor_info.custom1.framelength):0;
			m_framelength = imgsensor_info.custom1.framelength + m_dummyline;
			m_framelength = round_up(m_framelength, 2);
			if(m_framelength > imgsensor.shutter + imgsensor_info.margin) {
				spin_lock(&imgsensor_drv_lock);
				imgsensor.frame_length = m_framelength;
				imgsensor.min_frame_length = imgsensor.frame_length;
				imgsensor.dummy_line = m_dummyline;
				spin_unlock(&imgsensor_drv_lock);
				set_dummy();
			} else {
				LOG_INF("new framelength(%d) <= shutter(%d) + margin(%d)",
					m_framelength, imgsensor.shutter, imgsensor_info.margin);
				LOG_INF("current framelength(%d)", imgsensor.frame_length);
			}
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			frameHeight = imgsensor_info.custom2.pclk / framerate * 10 /
				imgsensor_info.custom2.linelength;
			m_dummyline =
				(frameHeight > imgsensor_info.custom2.framelength) ?
				(frameHeight - imgsensor_info.custom2.framelength):0;
			m_framelength = imgsensor_info.custom2.framelength + m_dummyline;
			m_framelength = round_up(m_framelength, 2);
			if(m_framelength > imgsensor.shutter + imgsensor_info.margin) {
				spin_lock(&imgsensor_drv_lock);
				imgsensor.frame_length = m_framelength;
				imgsensor.min_frame_length = imgsensor.frame_length;
				imgsensor.dummy_line = m_dummyline;
				spin_unlock(&imgsensor_drv_lock);
				set_dummy();
			} else {
				LOG_INF("new framelength(%d) <= shutter(%d) + margin(%d)",
					m_framelength, imgsensor.shutter, imgsensor_info.margin);
				LOG_INF("current framelength(%d)", imgsensor.frame_length);
			}
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			frameHeight = imgsensor_info.custom3.pclk / framerate * 10 /
				imgsensor_info.custom3.linelength;
			m_dummyline =
				(frameHeight > imgsensor_info.custom3.framelength) ?
				(frameHeight - imgsensor_info.custom3.framelength):0;
			m_framelength = imgsensor_info.custom3.framelength + m_dummyline;
			m_framelength = round_up(m_framelength, 4);
			if(m_framelength > imgsensor.shutter + imgsensor_info.margin) {
				spin_lock(&imgsensor_drv_lock);
				imgsensor.frame_length = m_framelength;
				imgsensor.min_frame_length = imgsensor.frame_length;
				imgsensor.dummy_line = m_dummyline;
				spin_unlock(&imgsensor_drv_lock);
				set_dummy();
			} else {
				LOG_INF("new framelength(%d) <= shutter(%d) + margin(%d)",
					m_framelength, imgsensor.shutter, imgsensor_info.margin);
				LOG_INF("current framelength(%d)", imgsensor.frame_length);
			}
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			frameHeight = imgsensor_info.custom4.pclk / framerate * 10 /
				imgsensor_info.custom4.linelength;
			m_dummyline =
				(frameHeight > imgsensor_info.custom4.framelength) ?
				(frameHeight - imgsensor_info.custom4.framelength):0;
			m_framelength = imgsensor_info.custom4.framelength + m_dummyline;
			m_framelength = round_up(m_framelength, 2);
			if(m_framelength > ((imgsensor.shutter + imgsensor.secend_shutter +
				(2 * imgsensor_info.margin)) * 2)) {
				spin_lock(&imgsensor_drv_lock);
				imgsensor.frame_length = m_framelength;
				imgsensor.min_frame_length = imgsensor.frame_length;
				imgsensor.dummy_line = m_dummyline;
				spin_unlock(&imgsensor_drv_lock);
				set_dummy();
			} else {
				LOG_INF("new framelength(%d) <= (shutter1(%d) + shutter2(%d) + 2 * margin(%d))*2",
					m_framelength, imgsensor.shutter, imgsensor.secend_shutter,imgsensor_info.margin);
				LOG_INF("current framelength(%d)", imgsensor.frame_length);
			}
		break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			frameHeight = imgsensor_info.custom5.pclk / framerate * 10 /
				imgsensor_info.custom5.linelength;
			m_dummyline =
				(frameHeight > imgsensor_info.custom5.framelength) ?
				(frameHeight - imgsensor_info.custom5.framelength):0;
			m_framelength = imgsensor_info.custom5.framelength + m_dummyline;
			m_framelength = round_up(m_framelength, 2);
			if(m_framelength > imgsensor.shutter + imgsensor_info.margin) {
				spin_lock(&imgsensor_drv_lock);
				imgsensor.frame_length = m_framelength;
				imgsensor.min_frame_length = imgsensor.frame_length;
				imgsensor.dummy_line = m_dummyline;
				spin_unlock(&imgsensor_drv_lock);
				set_dummy();
			} else {
				LOG_INF("new framelength(%d) <= shutter(%d) + margin(%d)",
					m_framelength, imgsensor.shutter, imgsensor_info.margin);
				LOG_INF("current framelength(%d)", imgsensor.frame_length);
			}
			break;
		case MSDK_SCENARIO_ID_CUSTOM6:
			frameHeight = imgsensor_info.custom6.pclk / framerate * 10 /
				imgsensor_info.custom6.linelength;
			m_dummyline =
				(frameHeight > imgsensor_info.custom6.framelength) ?
				(frameHeight - imgsensor_info.custom6.framelength):0;
			m_framelength = imgsensor_info.custom6.framelength + m_dummyline;
			m_framelength = round_up(m_framelength, 2);
			if(m_framelength > imgsensor.shutter + imgsensor_info.margin) {
				spin_lock(&imgsensor_drv_lock);
				imgsensor.frame_length = m_framelength;
				imgsensor.min_frame_length = imgsensor.frame_length;
				imgsensor.dummy_line = m_dummyline;
				spin_unlock(&imgsensor_drv_lock);
				set_dummy();
			} else {
				LOG_INF("new framelength(%d) <= shutter(%d) + margin(%d)",
					m_framelength, imgsensor.shutter, imgsensor_info.margin);
				LOG_INF("current framelength(%d)", imgsensor.frame_length);
			}
		break;
		case MSDK_SCENARIO_ID_CUSTOM7:
			frameHeight = imgsensor_info.custom7.pclk / framerate * 10 /
				imgsensor_info.custom7.linelength;
			m_dummyline =
				(frameHeight > imgsensor_info.custom7.framelength) ?
				(frameHeight - imgsensor_info.custom7.framelength):0;
			m_framelength = imgsensor_info.custom7.framelength + m_dummyline;
			m_framelength = round_up(m_framelength, 4);
			if(m_framelength > imgsensor.shutter + imgsensor_info.margin) {
				spin_lock(&imgsensor_drv_lock);
				imgsensor.frame_length = m_framelength;
				imgsensor.min_frame_length = imgsensor.frame_length;
				imgsensor.dummy_line = m_dummyline;
				spin_unlock(&imgsensor_drv_lock);
				set_dummy();
			} else {
				LOG_INF("new framelength(%d) <= shutter(%d) + margin(%d)",
					m_framelength, imgsensor.shutter, imgsensor_info.margin);
				LOG_INF("current framelength(%d)", imgsensor.frame_length);
			}
			break;
		default:  //coding with  preview scenario by default
		frameHeight = imgsensor_info.pre.pclk / framerate * 10 /
			imgsensor_info.pre.linelength;
		m_dummyline =
			(frameHeight > imgsensor_info.pre.framelength) ?
			(frameHeight - imgsensor_info.pre.framelength):0;
		m_framelength = imgsensor_info.pre.framelength + m_dummyline;
		m_framelength = round_up(m_framelength, 2);
		if(m_framelength > imgsensor.shutter + imgsensor_info.margin) {
			spin_lock(&imgsensor_drv_lock);
			imgsensor.frame_length = m_framelength;
			imgsensor.min_frame_length = imgsensor.frame_length;
			imgsensor.dummy_line = m_dummyline;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
		} else {
			LOG_INF("new framelength(%d) <= shutter(%d) + margin(%d)",
				m_framelength, imgsensor.shutter, imgsensor_info.margin);
			LOG_INF("current framelength(%d)", imgsensor.frame_length);
		}
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(
            enum MSDK_SCENARIO_ID_ENUM scenario_id,
            MUINT32 *framerate)
{
    if (scenario_id == 0)
    LOG_INF("[3058]scenario_id = %d\n", scenario_id);

    switch (scenario_id) {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        *framerate = imgsensor_info.pre.max_framerate;
    break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        *framerate = imgsensor_info.normal_video.max_framerate;
    break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        *framerate = imgsensor_info.cap.max_framerate;
    break;
    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        *framerate = imgsensor_info.hs_video.max_framerate;
    break;
    case MSDK_SCENARIO_ID_SLIM_VIDEO:
        *framerate = imgsensor_info.slim_video.max_framerate;
    break;
    case MSDK_SCENARIO_ID_CUSTOM1:
        *framerate = imgsensor_info.custom1.max_framerate;
    break;
    case MSDK_SCENARIO_ID_CUSTOM2:
        *framerate = imgsensor_info.custom2.max_framerate;
    break;
    case MSDK_SCENARIO_ID_CUSTOM3:
        *framerate = imgsensor_info.custom3.max_framerate;
    break;
    case MSDK_SCENARIO_ID_CUSTOM4:
        *framerate = imgsensor_info.custom4.max_framerate;
    break;
    case MSDK_SCENARIO_ID_CUSTOM5:
        *framerate = imgsensor_info.custom5.max_framerate;
    break;
    case MSDK_SCENARIO_ID_CUSTOM6:
        *framerate = imgsensor_info.custom6.max_framerate;
    break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        *framerate = imgsensor_info.custom7.max_framerate;
    break;
    default:
    break;
    }

    return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_uint8 mode)
{
    LOG_INF("%s mode:%d\n",__func__, mode);
    if (mode == 5) {
        write_cmos_sensor(0x3510, 0x00);//b
        write_cmos_sensor(0x401a, 0x00);//blk_lvl_l
        write_cmos_sensor(0x3550, 0x00);//b
        write_cmos_sensor(0x491a, 0x00);//blk_lvl_m
    } else {
        write_cmos_sensor(0x3510, 0x01);
        write_cmos_sensor(0x401a, 0x40);
        write_cmos_sensor(0x3550, 0x01);
        write_cmos_sensor(0x491a, 0x40);
    }

    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = mode;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

static kal_uint32 get_sensor_temperature(void)
{
    UINT32 temperature = 0;
    INT32 temperature_convert = 0;

    /*TEMP_SEN_CTL */
    write_cmos_sensor(0x4d12, 0x01);
    temperature = (read_cmos_sensor(0x4d13) << 8) |
        read_cmos_sensor(0x4d13);
    if (temperature < 0xc000)
        temperature_convert = temperature / 256;
    else
        temperature_convert = 192 - temperature / 256;

    if (temperature_convert > 192) {
        //LOG_INF("Temperature too high: %d\n",
                //temperature_convert);
        temperature_convert = 192;
    } else if (temperature_convert < -64) {
        //LOG_INF("Temperature too low: %d\n",
                //temperature_convert);
        temperature_convert = -64;
    }

    return 20;
    //return temperature_convert;
}

static kal_uint32 seamless_switch(enum MSDK_SCENARIO_ID_ENUM scenario_id,
    kal_uint32 shutter, kal_uint32 gain,
    kal_uint32 shutter_2ndframe, kal_uint32 gain_2ndframe)
{
	UINT32 delay_time = 0;
	if (scenario_id == MSDK_SCENARIO_ID_CAMERA_PREVIEW) {
		delay_time = 1000 / (imgsensor_info.custom7.pclk / imgsensor.frame_length / imgsensor_info.custom7.linelength);
	} else if (scenario_id == MSDK_SCENARIO_ID_CUSTOM7) {
		delay_time = 1000 / (imgsensor_info.pre.pclk / imgsensor.frame_length / imgsensor_info.pre.linelength);
	}
	mdelay(delay_time);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CUSTOM4:
			seamless_switch_to_custom4(shutter, gain, shutter_2ndframe, gain_2ndframe);
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			seamless_switch_to_normal_video(shutter, gain);
			break;
		case MSDK_SCENARIO_ID_CUSTOM7:
			seamless_switch_to_custom7(shutter, gain);
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			seamless_switch_to_preview(shutter, gain);
			break;
		default :
			break;
	}
	return 0;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor(0x0100, 0x01);
	} else {
		write_cmos_sensor(0x0100, 0x00);
		write_cmos_sensor(0x3208, 0x00);
		write_cmos_sensor(0x300b, 0x00);
		write_cmos_sensor(0x3208, 0x10);
	}
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
            UINT8 *feature_para, UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
    UINT16 *feature_data_16 = (UINT16 *) feature_para;
    UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
    UINT32 *feature_data_32 = (UINT32 *) feature_para;
    INT32 *feature_return_para_i32 = (INT32 *) feature_para;
    unsigned long long *feature_data = (unsigned long long *) feature_para;

    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    #if 1
    UINT32 *pAeCtrls = NULL;
    UINT32 *pScenarios = NULL;
    #endif
    struct SENSOR_VC_INFO2_STRUCT *pvcinfo2;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
        (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    struct SET_PD_BLOCK_INFO_T *PDAFinfo;
    switch (feature_id) {
        case SENSOR_FEATURE_SET_SENSOR_OTP:
        {
            enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
            LOG_INF("SENSOR_FEATURE_SET_SENSOR_OTP\n");
            ret = Eeprom_CallWriteService((ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(feature_para));
            if (ret == IMGSENSOR_RETURN_SUCCESS)
                return ERROR_NONE;
            else
                return ERROR_MSDK_IS_ACTIVATED;
        }
        break;
    #if 1
    case SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS:
        pScenarios = (MUINT32 *)((uintptr_t)(*(feature_data+1)));
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*pScenarios = MSDK_SCENARIO_ID_CUSTOM7;
				break;
			case MSDK_SCENARIO_ID_CUSTOM7:
				*pScenarios = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*pScenarios = MSDK_SCENARIO_ID_CUSTOM4;
				break;
			case MSDK_SCENARIO_ID_CUSTOM4:
				*pScenarios = MSDK_SCENARIO_ID_VIDEO_PREVIEW;
				break;
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			case MSDK_SCENARIO_ID_CUSTOM1:
			case MSDK_SCENARIO_ID_CUSTOM2:
			case MSDK_SCENARIO_ID_CUSTOM3:

			case MSDK_SCENARIO_ID_CUSTOM5:
			case MSDK_SCENARIO_ID_CUSTOM6:
			default:
				*pScenarios = 0xff;
				break;
		}
        LOG_INF("SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS %d %d\n", *feature_data, *pScenarios);
        break;
    case SENSOR_FEATURE_SEAMLESS_SWITCH:
        pAeCtrls = (MUINT32 *)((uintptr_t)(*(feature_data+1)));
        if (pAeCtrls)
            seamless_switch((*feature_data), pAeCtrls[SHUTTER_NE_FRM_1],
                pAeCtrls[GAIN_NE_FRM_1],  pAeCtrls[SHUTTER_SE_FRM_1], pAeCtrls[GAIN_SE_FRM_1]);
        else
            seamless_switch((*feature_data), 0, 0, 0, 0);
        break;
        #endif
    case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
        *(feature_data + 1) = imgsensor_info.min_gain;
        *(feature_data + 2) = imgsensor_info.max_gain;
        break;
    case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
        *(feature_data + 0) = imgsensor_info.min_gain_iso;
        *(feature_data + 1) = imgsensor_info.gain_step;
        *(feature_data + 2) = imgsensor_info.gain_type;
        break;
    case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
    *(feature_data + 1) = imgsensor_info.min_shutter;
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CUSTOM2:
        case MSDK_SCENARIO_ID_CUSTOM4:
        case MSDK_SCENARIO_ID_CUSTOM5:
        case MSDK_SCENARIO_ID_CUSTOM6:
            *(feature_data + 2) = 2;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM7:
        default:
            *(feature_data + 2) = 1;
            break;
        }
        break;
    case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
        *(MINT32 *)(signed long)(*(feature_data + 1)) = -2021000;
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.cap.pclk;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.normal_video.pclk;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.hs_video.pclk;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.slim_video.pclk;
            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom1.pclk;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom2.pclk;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom3.pclk;
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom4.pclk;
                break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom5.pclk;
                break;
        case MSDK_SCENARIO_ID_CUSTOM6:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom6.pclk;
                break;
        case MSDK_SCENARIO_ID_CUSTOM7:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom7.pclk;
                break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.pre.pclk;
            break;
        }
        break;
    case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.cap.framelength << 16)
                + imgsensor_info.cap.linelength;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.normal_video.framelength << 16)
                + imgsensor_info.normal_video.linelength;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.hs_video.framelength << 16)
                + imgsensor_info.hs_video.linelength;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.slim_video.framelength << 16)
                + imgsensor_info.slim_video.linelength;
            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.custom1.framelength << 16)
                + imgsensor_info.custom1.linelength;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.custom2.framelength << 16)
                + imgsensor_info.custom2.linelength;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.custom3.framelength << 16)
                + imgsensor_info.custom3.linelength;
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.custom4.framelength << 16)
                + imgsensor_info.custom4.linelength;
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.custom5.framelength << 16)
                + imgsensor_info.custom5.linelength;
            break;
		case MSDK_SCENARIO_ID_CUSTOM6:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.custom6.framelength << 16)
                + imgsensor_info.custom6.linelength;
            break;
		case MSDK_SCENARIO_ID_CUSTOM7:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.custom7.framelength << 16)
                + imgsensor_info.custom7.linelength;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.pre.framelength << 16)
                + imgsensor_info.pre.linelength;
            break;
        }
        break;
    case SENSOR_FEATURE_GET_PERIOD:
        *feature_return_para_16++ = imgsensor.line_length;
        *feature_return_para_16 = imgsensor.frame_length;
        *feature_para_len = 4;
    break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        *feature_return_para_32 = imgsensor.pclk;
        *feature_para_len = 4;
    break;
    case SENSOR_FEATURE_SET_ESHUTTER:
        set_shutter(*feature_data);
    break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
    break;
    case SENSOR_FEATURE_SET_GAIN:
        set_gain((UINT16) *feature_data);
    break;
    case SENSOR_FEATURE_SET_FLASHLIGHT:
    break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
    break;
    case SENSOR_FEATURE_SET_REGISTER:
        if(sensor_reg_data->RegAddr == 0xff )
            seamless_switch(sensor_reg_data->RegData, 1920, 369, 960, 369);
        else
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);

    break;
    case SENSOR_FEATURE_GET_STAGGER_TARGET_SCENARIO:
        if (*feature_data == MSDK_SCENARIO_ID_VIDEO_PREVIEW) {
            switch (*(feature_data + 1)) {
            case HDR_RAW_STAGGER_2EXP:
                *(feature_data + 2) = MSDK_SCENARIO_ID_CUSTOM4;//custom4 was the 2 exp mode for preview mode
                break;
            default:
                break;
            }
        }else if (*feature_data == MSDK_SCENARIO_ID_CUSTOM4) {
            switch (*(feature_data + 1)) {
            case HDR_NONE:
                *(feature_data + 2) = MSDK_SCENARIO_ID_VIDEO_PREVIEW;//normal_video mode for video preview mode
                break;
            default:
                break;
            }
        }
        break;
    case SENSOR_FEATURE_GET_STAGGER_MAX_EXP_TIME:
        if (*feature_data == MSDK_SCENARIO_ID_CUSTOM4) {
            switch (*(feature_data + 1)) {
            case VC_STAGGER_NE:
                *(feature_data + 2) = 2745;
                break;
            case VC_STAGGER_ME:
                *(feature_data + 2) = 2745;
                break;
            case VC_STAGGER_SE:
                *(feature_data + 2) = 2745;
                break;
            default:
                *(feature_data + 2) = 2745;
                break;
            }
        } else {
            *(feature_data + 2) = 0;
        }
        break;
    case SENSOR_FEATURE_SET_HDR_SHUTTER://for 2EXP
        LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
            (UINT16) *feature_data, (UINT16) *(feature_data + 1));
        // implement write shutter for NE/SE
        hdr_write_dual_shutter((UINT16)*feature_data,
            (UINT16)*(feature_data+1));
        break;
    case SENSOR_FEATURE_SET_DUAL_GAIN://for 2EXP
        LOG_INF("SENSOR_FEATURE_SET_DUAL_GAIN LGAIN=%d, SGAIN=%d\n",
            (UINT16) *feature_data, (UINT16) *(feature_data + 1));
        // implement write gain for NE/SE
        hdr_write_dual_gain((UINT16)*feature_data,
            (UINT16)*(feature_data+1));
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        sensor_reg_data->RegData =
            read_cmos_sensor(sensor_reg_data->RegAddr);
    break;
    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
        *feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
        *feature_para_len = 4;
    break;
    case SENSOR_FEATURE_SET_VIDEO_MODE:
        set_video_mode(*feature_data);
    break;
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
        get_imgsensor_id(feature_return_para_32);
    break;
    case SENSOR_FEATURE_CHECK_MODULE_ID:
        *feature_return_para_32 = imgsensor_info.module_id;
        break;
    case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
        set_auto_flicker_mode((BOOL)*feature_data_16,
            *(feature_data_16+1));
    break;
    case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
        set_max_framerate_by_scenario(
            (enum MSDK_SCENARIO_ID_ENUM)*feature_data,
            *(feature_data+1));
    break;
    case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
        get_default_framerate_by_scenario(
            (enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
            (MUINT32 *)(uintptr_t)(*(feature_data+1)));
    break;
    case SENSOR_FEATURE_SET_TEST_PATTERN:
        set_test_pattern_mode((UINT8)*feature_data);
    break;
    case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
        *feature_return_para_32 = imgsensor_info.checksum_value;
        *feature_para_len = 4;
    break;
    case SENSOR_FEATURE_SET_FRAMERATE:
        spin_lock(&imgsensor_drv_lock);
        imgsensor.current_fps = *feature_data_32;
        spin_unlock(&imgsensor_drv_lock);
        LOG_INF("current fps :%d\n", imgsensor.current_fps);
    break;
    case SENSOR_FEATURE_GET_CROP_INFO:
        LOG_INF("GET_CROP_INFO scenarioId:%d\n",
            *feature_data_32);

        wininfo = (struct  SENSOR_WINSIZE_INFO_STRUCT *)
            (uintptr_t)(*(feature_data+1));
        switch (*feature_data_32) {
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                memcpy((void *)wininfo,
                    (void *)&imgsensor_winsize_info[1],
                    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                memcpy((void *)wininfo,
                    (void *)&imgsensor_winsize_info[2],
                    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
            case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                memcpy((void *)wininfo,
                    (void *)&imgsensor_winsize_info[3],
                    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
            case MSDK_SCENARIO_ID_SLIM_VIDEO:
                memcpy((void *)wininfo,
                    (void *)&imgsensor_winsize_info[4],
                    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
            case MSDK_SCENARIO_ID_CUSTOM1:
                memcpy((void *)wininfo,
                    (void *)&imgsensor_winsize_info[5],
                    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
            case MSDK_SCENARIO_ID_CUSTOM2:
                memcpy((void *)wininfo,
                    (void *)&imgsensor_winsize_info[6],
                    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
            case MSDK_SCENARIO_ID_CUSTOM3:
                memcpy((void *)wininfo,
                    (void *)&imgsensor_winsize_info[7],
                    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
            case MSDK_SCENARIO_ID_CUSTOM4:
                memcpy((void *)wininfo,
                    (void *)&imgsensor_winsize_info[8],
                    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
            case MSDK_SCENARIO_ID_CUSTOM5:
                memcpy((void *)wininfo,
                    (void *)&imgsensor_winsize_info[9],
                    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
            case MSDK_SCENARIO_ID_CUSTOM6:
                memcpy((void *)wininfo,
                    (void *)&imgsensor_winsize_info[10],
                    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
            case MSDK_SCENARIO_ID_CUSTOM7:
                memcpy((void *)wininfo,
                    (void *)&imgsensor_winsize_info[11],
                    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            default:
                memcpy((void *)wininfo,
                    (void *)&imgsensor_winsize_info[0],
                    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        }
    break;
    case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
        LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
            (UINT16)*feature_data, (UINT16)*(feature_data+1),
            (UINT16)*(feature_data+2));
    break;
    case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.cap.mipi_pixel_rate;
				break;
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.normal_video.mipi_pixel_rate;
                break;
            case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.hs_video.mipi_pixel_rate;
                break;
            case MSDK_SCENARIO_ID_SLIM_VIDEO:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.slim_video.mipi_pixel_rate;
                break;
            case MSDK_SCENARIO_ID_CUSTOM1:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.custom1.mipi_pixel_rate;
                break;
            case MSDK_SCENARIO_ID_CUSTOM2:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.custom2.mipi_pixel_rate;
                break;
            case MSDK_SCENARIO_ID_CUSTOM3:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.custom3.mipi_pixel_rate;
                break;
            case MSDK_SCENARIO_ID_CUSTOM4:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.custom4.mipi_pixel_rate;
                break;
            case MSDK_SCENARIO_ID_CUSTOM5:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.custom5.mipi_pixel_rate;
                break;
            case MSDK_SCENARIO_ID_CUSTOM6:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.custom6.mipi_pixel_rate;
                break;
            case MSDK_SCENARIO_ID_CUSTOM7:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.custom7.mipi_pixel_rate;
                break;				
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            default:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.pre.mipi_pixel_rate;
                break;
            }
    break;
    case SENSOR_FEATURE_GET_BINNING_TYPE:
        switch (*(feature_data + 1)) {
            case MSDK_SCENARIO_ID_CUSTOM3:    //full size
            case MSDK_SCENARIO_ID_CUSTOM7:    //full size center crop
                *feature_return_para_32 = 1;
                break;
            case MSDK_SCENARIO_ID_CUSTOM5:
                *feature_return_para_32 = 1256;
                break;
            case MSDK_SCENARIO_ID_CUSTOM6:
                *feature_return_para_32 = 3080;
                break;
            default:
                *feature_return_para_32 = 1119; /*BINNING_AVERAGED*/
                break;
            }
        LOG_INF("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
            *feature_return_para_32);
        *feature_para_len = 4;

        break;
    case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
        /*
            HDR_NONE = 0,
            HDR_RAW = 1,
            HDR_CAMSV = 2,
            HDR_RAW_ZHDR = 9,
            HDR_MultiCAMSV = 10,
            HDR_RAW_STAGGER_2EXP = 0xB,
            HDR_RAW_STAGGER_MIN = HDR_RAW_STAGGER_2EXP,
            HDR_RAW_STAGGER_3EXP = 0xC,
            HDR_RAW_STAGGER_MAX = HDR_RAW_STAGGER_3EXP,
         */
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0xb;
			break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
        default:
            *(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
            break;
        }
        LOG_INF("SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY scenarioId:%llu, HDR:%llu\n",
            *feature_data, *(feature_data+1));
        break;
    case SENSOR_FEATURE_GET_VC_INFO2:
        LOG_INF("SENSOR_FEATURE_GET_VC_INFO2 %d\n", (UINT16) (*feature_data));
        pvcinfo2 = (struct SENSOR_VC_INFO2_STRUCT *) (uintptr_t) (*(feature_data + 1));
        get_vc_info_2(pvcinfo2, *feature_data_32);
        break;
    case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
        switch (*feature_data) {
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            case MSDK_SCENARIO_ID_SLIM_VIDEO:
            case MSDK_SCENARIO_ID_CUSTOM1:
            case MSDK_SCENARIO_ID_CUSTOM2:
            case MSDK_SCENARIO_ID_CUSTOM3:
            case MSDK_SCENARIO_ID_CUSTOM4:
			case MSDK_SCENARIO_ID_CUSTOM7:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
				break;
			case MSDK_SCENARIO_ID_CUSTOM5:
            case MSDK_SCENARIO_ID_CUSTOM6:
            default:
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                break;
        }
		LOG_INF(
		    "SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu=%d\n",
		    *feature_data,
		    *(MUINT32 *)(uintptr_t)(*(feature_data + 1)));

        break;
    case SENSOR_FEATURE_GET_PDAF_INFO:
        LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n", (UINT16) *feature_data);
        PDAFinfo = (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CUSTOM2:
        case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM4:
        case MSDK_SCENARIO_ID_CUSTOM5:
        case MSDK_SCENARIO_ID_CUSTOM6:
		case MSDK_SCENARIO_ID_CUSTOM7:
        default:
            memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info, sizeof(struct SET_PD_BLOCK_INFO_T));
            break;
        }
        break;
    case SENSOR_FEATURE_GET_PDAF_DATA:
        break;
    case SENSOR_FEATURE_SET_PDAF:
            LOG_INF("PDAF mode :%d\n", *feature_data_16);
            imgsensor.pdaf_mode = *feature_data_16;
        break;
    case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
        *feature_return_para_i32 = get_sensor_temperature();
        *feature_para_len = 4;
    break;
    case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
        LOG_INF("SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME\n");
        set_shutter_frame_length((UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16) *(feature_data + 2));
        break;
    case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
        streaming_control(KAL_FALSE);
        break;

    case SENSOR_FEATURE_SET_STREAMING_RESUME:
        if (*feature_data != 0)
            set_shutter(*feature_data);
        streaming_control(KAL_TRUE);
        break;
    case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
        /*
         * 1, if driver support new sw frame sync
         * set_shutter_frame_length() support third para auto_extend_en
         */
        *(feature_data + 1) = 1; /* margin info by scenario */
        *(feature_data + 2) = imgsensor_info.margin;
        break;
    default:
    break;
    }

    return ERROR_NONE;
}   /*  feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 OV50A_MIPI_RAW_21127_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc != NULL)
    *pfFunc =  &sensor_func;
    return ERROR_NONE;
}
