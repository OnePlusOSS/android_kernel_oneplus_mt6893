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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     IMX766mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx766mipiraw_Sensor.h"
#include "imx766_eeprom.h"
#include "imgsensor_eeprom.h"
#include "imx766_seamless_switch.h"

#define MAX_VC_INFO_CNT_766 12
#define SENSOR_GET_LINELENGTH_FOR_READOUT (0x1 << 0)

/***************Modify Following Strings for Debug**********************/
#define PFX "IMX766_camera_sensor"
/****************************   Modify end    **************************/
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)

#define I2C_BUFFER_LEN 765 /* trans# max is 255, each 3 bytes */

static kal_uint8 qsc_flag = 0;
static kal_uint8 otp_flag = 0;
static kal_uint16 imx766_QSC_setting[3072 * 2];
BYTE imx766_21305_LRC_data[352] = {0};
static uint8_t deviceInfo_register_value = 0;
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = IMX766_SENSOR_ID_21305,

    .checksum_value = 0x8ac2d94a,
    .pre = { //QBIN_HVBin_4096x3072_30FPS W-3
        .pclk = 1507200000,
        .linelength = 15616,
        .framelength = 3216,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 798170000,
        .max_framerate = 300, /* 30fps */
    },
    .cap = { //QBIN_HVBin_4096x3072_30FPS
        .pclk = 1488000000,
        .linelength = 15616,
        .framelength = 3176,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 787200000,
        .max_framerate = 300, /* 30fps */
    },
    .normal_video = { //QBIN(HVBIN)_4096x2304_30FPS  reg_Q-6
        .pclk = 2246400000,
        .linelength = 15616,
        .framelength = 4794,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 2304,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 938060000,
        .max_framerate = 300, /* 30fps */
    },
    .hs_video = { //QBIN(HVBIN) - V2H2_FHD_2048x1152_120FPS
        .pclk = 1300800000,
        .linelength = 8816,
        .framelength = 1228,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2048,
        .grabwindow_height = 1152,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 517030000,
        .max_framerate = 1200, /* 120fps */
    },
    .slim_video = { //QBIN_HVBin_4096x3072_30FPS
        .pclk = 1488000000,
        .linelength = 15616,
        .framelength = 3176,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 787200000,
        .max_framerate = 300, /* 30fps */
    },
    .custom1 = { //QBIN_HVBin_4096x3072_24FPS
        .pclk = 1502400000,
        .linelength = 15616,
        .framelength = 4008,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 586970000,
        .max_framerate = 240, /* 24fps */
    },
    .custom2 = { /* QBIN_HVBin_4096x2304_60FPS */
        .pclk = 2284800000,
        .linelength = 15616,
        .framelength = 2438,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 2304,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 894170000,
        .max_framerate = 600, /* 60fps */
    },
    .custom3 = { /* QRMSC 8192x6144_24FPS */
        .pclk = 1752000000,
        .linelength = 11552,
        .framelength = 6318,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 8192,
        .grabwindow_height = 6144,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 1738970000,
        .max_framerate = 240, /* 24fps */
    },
    .custom4 = { //QBIN(HVBIN)_2DOL_4096x2304_30FPS  reg_R-6
        .pclk = 2246400000,
        .linelength = 15616,
        .framelength = 4792,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 2304,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 938060000,
        .max_framerate = 300, /* 30fps */
    },
    .custom5 = { /* QBIN(HVBIN) - V2H2_FHD_2048x1152_240FPS */
        .pclk = 2716800000,
        .linelength = 8816,
        .framelength = 1284,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2048,
        .grabwindow_height = 1152,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 1110860000,
        .max_framerate = 2400, /* 240fps */
    },
    .custom6 = { /* QBIN - V2H2_1280x720_480FPS */
        .pclk = 2400000000,
        .linelength = 5568,
        .framelength = 896,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 842060000,
        .max_framerate = 4800, /* 480fps */
    },
    .custom7 = { /* QRMSC_4096x3072_30FPS V-4 */
        .pclk = 1507200000,
        .linelength = 11552,
        .framelength = 4348,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 798170000,
        .max_framerate = 300, /* 30fps */
    },

    .min_gain = 64, /*1x gain*/
    .max_gain = 4096, /*64x gain*/
    .min_gain_iso = 100,
    .margin = 48,        /* sensor framelength & shutter margin */
    .min_shutter = 8,    /* min shutter */
    .gain_step = 1,
    .gain_type = 0,
    .max_frame_length = 0xffff,
    .ae_shut_delay_frame = 0,
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,    /* isp gain delay frame for AE cycle */
    .ihdr_support = 0,    /* 1, support; 0,not support */
    .ihdr_le_firstline = 0,    /* 1,le first ; 0, se first */
    .temperature_support = 0, /* 1, support; 0,not support */
    .sensor_mode_num = 12,    /* support sensor mode num */
    .frame_time_delay_frame = 3,

    .cap_delay_frame = 2,    /* enter capture delay frame num */
    .pre_delay_frame = 2,    /* enter preview delay frame num */
    .video_delay_frame = 2,    /* enter video delay frame num */
    .hs_video_delay_frame = 2,
    .slim_video_delay_frame = 2,    /* enter slim video delay frame num */
    .custom1_delay_frame = 2,    /* enter custom1 delay frame num */
    .custom2_delay_frame = 2,    /* enter custom2 delay frame num */
    .custom3_delay_frame = 2,
    .custom4_delay_frame = 2,
    .custom5_delay_frame = 2,
    .custom6_delay_frame = 2,
    .isp_driving_current = ISP_DRIVING_6MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    /* .mipi_sensor_type = MIPI_OPHY_NCSI2, */
    /* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
    .mipi_sensor_type = MIPI_CPHY, /* 0,MIPI_OPHY_NCSI2; 1,MIPI_OPHY_CSI2 */
    .mipi_settle_delay_mode = 0,
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,
    .mclk = 24, /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
    /*.mipi_lane_num = SENSOR_MIPI_4_LANE,*/
    .mipi_lane_num = SENSOR_MIPI_3_LANE,
    .i2c_addr_table = {0x34, 0xff},
    /* record sensor support all write id addr,
     * only supprt 4 must end with 0xff
     */
    .i2c_speed = 1000, /* i2c read/write speed */
};
static struct imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,    /* mirrorflip information */
    .sensor_mode = IMGSENSOR_MODE_INIT,
    /* IMGSENSOR_MODE enum value,record current sensor mode,such as:
     * INIT, Preview, Capture, Video,High Speed Video, Slim Video
     */
    .shutter = 0x3D0,    /* current shutter */
    .gain = 0x100,        /* current gain */
    .dummy_pixel = 0,    /* current dummypixel */
    .dummy_line = 0,    /* current dummyline */
    .current_fps = 300,
    .autoflicker_en = KAL_FALSE,
    .test_pattern = KAL_FALSE,
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
    .ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
    .i2c_write_id = 0x34, /* record current sensor's i2c write id */
    .current_ae_effective_frame = 2,
    .extend_frame_length_en = KAL_FALSE,
    .fast_mode_on = KAL_FALSE,
};

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
        {0, 0}, {0, 0}, {0, 356}, {0, 356}, {0, 0},
        {0, 0}, {0, 356}, {0, 0}, {0, 356}, {0, 356}
    },  //{0, 1632}
    .iMirrorFlip = 0,
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
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //preivew
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
            {VC_PDAF_STATS_PIX_1, 0x00, 0x30, 0xa00, 0x300},
            {VC_PDAF_STATS_PIX_2, 0x00, 0x31, 0xa00, 0x300},
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //capture
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
            {VC_PDAF_STATS_PIX_1, 0x00, 0x30, 0xa00, 0x300},
            {VC_PDAF_STATS_PIX_2, 0x00, 0x31, 0xa00, 0x300},
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //normal_video  //QBIN(HVBIN)_4096x2304_30FPS
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x900},
            {VC_PDAF_STATS_PIX_1, 0x00, 0x30, 0xa00, 0x240},
            {VC_PDAF_STATS_PIX_2, 0x00, 0x31, 0xa00, 0x240},
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //hs_video    QBIN(VBIN) - V2H2_FHD_2048x1152_120FPS
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x800, 0x480},
            {VC_PDAF_STATS_PIX_1, 0x00, 0x30, 0x500, 0x120},
            {VC_PDAF_STATS_PIX_2, 0x00, 0x31, 0x500, 0x120},
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //slim_video  QBIN_VBin_4096x3072_30FPS
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
            {VC_PDAF_STATS_PIX_1, 0x00, 0x30, 0xa00, 0x300},
            {VC_PDAF_STATS_PIX_2, 0x00, 0x31, 0xa00, 0x300},
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom1    QBIN_VBin_4096x3072_24FPS
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
            {VC_PDAF_STATS_PIX_1, 0x00, 0x30, 0xa00, 0x300},
            {VC_PDAF_STATS_PIX_2, 0x00, 0x31, 0xa00, 0x300},
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom2     QBIN(VBIN)_4096x2304_60FPS
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x900},
            {VC_PDAF_STATS_PIX_1, 0x00, 0x30, 0xa00, 0x240},
            {VC_PDAF_STATS_PIX_2, 0x00, 0x31, 0xa00, 0x240},
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom3     QRMSC 8192x6144_24FPS
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x2000, 0x1800},
            {VC_PDAF_STATS_PIX_1, 0x00, 0x30, 0x1400, 0x600},
            {VC_PDAF_STATS_PIX_2, 0x00, 0x31, 0x1400, 0x600},
        },
        1
    },
    {
        0x04, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom4	QBIN(VBIN)_2DOL_4096x2304_30FPS
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x900},
            {VC_STAGGER_ME, 0x01, 0x2b, 0x1000, 0x900},
            {VC_PDAF_STATS_PIX_1, 0x00, 0x30, 0xa00, 0x240},
            {VC_PDAF_STATS_PIX_2, 0x00, 0x31, 0xa00, 0x240},
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom5    QBIN(VBIN) - V2H2_FHD_2048x1152_240FPS
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x800, 0x480},
            {VC_PDAF_STATS_PIX_1, 0x00, 0x30, 0x500, 0x120},
            {VC_PDAF_STATS_PIX_2, 0x00, 0x31, 0x500, 0x120},
        },
        1
    },
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom6    QBIN - V2H2_1280x720_480FPS
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x500, 0x2d0},
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom7
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xC00},
            {VC_PDAF_STATS_PIX_1, 0x00, 0x30, 0xa00, 0x300},
            {VC_PDAF_STATS_PIX_2, 0x00, 0x31, 0xa00, 0x300},
        },
        1
    }
};

static struct SEAMLESS_SYS_DELAY seamless_sys_delays[] = {
    { MSDK_SCENARIO_ID_CAMERA_PREVIEW, MSDK_SCENARIO_ID_CUSTOM7, 1 },
    { MSDK_SCENARIO_ID_CUSTOM7, MSDK_SCENARIO_ID_CAMERA_PREVIEW, 1 },
    { MSDK_SCENARIO_ID_VIDEO_PREVIEW, MSDK_SCENARIO_ID_CUSTOM4, 1 },
    { MSDK_SCENARIO_ID_CUSTOM4, MSDK_SCENARIO_ID_VIDEO_PREVIEW, 1 },
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
static kal_uint16 imx766_table_write_cmos_sensor(kal_uint16 *para,
                                                 kal_uint32 len)
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
        //iWriteRegI2C(puSendCmd, 3, imgsensor.i2c_write_id);
        //tosend = 0;
        /* Write when remain buffer size is less than 3 bytes
        * or reach end of data
        */
        if ((I2C_BUFFER_LEN - tosend) < 3
             || IDX == len || addr != addr_last) {
            iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id, 3,
                imgsensor_info.i2c_speed);
            tosend = 0;
        }
    }

    return 0;
}

static kal_uint16 imx766_burst_write_cmos_sensor(
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
        else if(addr == addr_last + 1)
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

static void read_module_data(void)
{
    kal_uint32 Dac_master = 0, Dac_mac = 0, Dac_inf = 0;
    kal_uint32 vcmID = 0, lensID = 0;

    Dac_mac = (Eeprom_1ByteDataRead(0x93, IMX766_EEPROM_SLAVE_ADDRESS) << 8) | Eeprom_1ByteDataRead(0x92, IMX766_EEPROM_SLAVE_ADDRESS);
    Dac_inf = (Eeprom_1ByteDataRead(0x95, IMX766_EEPROM_SLAVE_ADDRESS) << 8) | Eeprom_1ByteDataRead(0x94, IMX766_EEPROM_SLAVE_ADDRESS);
    vcmID = (Eeprom_1ByteDataRead(0xb, IMX766_EEPROM_SLAVE_ADDRESS) << 8) | Eeprom_1ByteDataRead(0xa, IMX766_EEPROM_SLAVE_ADDRESS);
    lensID = (Eeprom_1ByteDataRead(0x9, IMX766_EEPROM_SLAVE_ADDRESS) << 8) | Eeprom_1ByteDataRead(0x8, IMX766_EEPROM_SLAVE_ADDRESS);
    Dac_mac = Dac_mac / 4;
    Dac_inf = Dac_inf/ 4;
    Dac_master = (9*Dac_mac+40*Dac_inf)/49;
    pr_info("Dac_inf:%d Dac_Mac:%d Dac_master:%d\n", Dac_inf, Dac_mac, Dac_master);
    memcpy(&gImgEepromInfo.camNormdata[IMGSENSOR_SENSOR_IDX_MAIN2][28], &Dac_master, 4); //set AFCodeData to MAIN2.
    memcpy(&gImgEepromInfo.camNormdata[IMGSENSOR_SENSOR_IDX_MAIN][40], &vcmID, 4);
    memcpy(&gImgEepromInfo.camNormdata[IMGSENSOR_SENSOR_IDX_MAIN][44], &lensID, 4);
    memcpy(&gImgEepromInfo.camNormdata[IMGSENSOR_SENSOR_IDX_MAIN][48], &Dac_mac, 4);
    memcpy(&gImgEepromInfo.camNormdata[IMGSENSOR_SENSOR_IDX_MAIN][52], &Dac_inf, 4);
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte = 0;
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};

    iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 2, imgsensor.i2c_write_id);
    return ((get_byte<<8)&0xff00) | ((get_byte>>8)&0x00ff);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte = 0;
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

    iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

static enum IMGSENSOR_RETURN write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

    return iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static kal_uint32 get_exp_cnt_by_scenario(kal_uint32 scenario)
{
    kal_uint32 exp_cnt = 0, i = 0;
    struct SENSOR_VC_INFO2_STRUCT vcinfo2;

    get_vc_info_2(&vcinfo2, scenario);

    for (i = 0; i < MAX_VC_INFO_CNT_766; ++i) {
        if (vcinfo2.vc_info[i].VC_FEATURE > VC_STAGGER_MIN_NUM &&
            vcinfo2.vc_info[i].VC_FEATURE < VC_STAGGER_MAX_NUM) {
            exp_cnt++;
        }
    }

    LOG_INF("%s exp_cnt %d\n", __func__, exp_cnt);
    return max(exp_cnt, (kal_uint32)1);
}

static kal_uint16 read_cmos_eeprom_8(kal_uint16 addr)
{
    kal_uint16 get_byte = 0;
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

    iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, IMX766_EEPROM_SLAVE_ADDRESS);
    return get_byte;
}

static void read_cmos_eeprom_table(kal_uint16 addr, kal_uint8 *table, kal_uint32 size)
{
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

    iReadRegI2C(pusendcmd, 2, (u8 *)table, size, IMX766_EEPROM_SLAVE_ADDRESS);
}

static void read_sensor_Cali(void)
{
    kal_uint16 idx = 0, addr_qsc = 0xfaf, sensor_lrc = 0x7F00;
    kal_uint8 imx766_QSC_setting1[3072];
    /*read otp data to distinguish module*/
    otp_flag = OTP_QSC_NONE;

    pr_info("OTP type: Custom Only");
    otp_flag = OTP_QSC_CUSTOM;

    addr_qsc = 0x1E30;
    read_cmos_eeprom_table(addr_qsc, imx766_QSC_setting1, 3072);
    for (idx = 0; idx < 3072; idx++) {
        sensor_lrc = 0xC800 + idx;
        imx766_QSC_setting[2 * idx] = sensor_lrc;
        imx766_QSC_setting[2 * idx + 1] = imx766_QSC_setting1[idx];
    }
#if 0
    for (idx = 0; idx < 3072; idx++) {
        pr_info("alex qsc data imx766_QSC_setting[0x%x] = 0x%x",
            imx766_QSC_setting[2 * idx], imx766_QSC_setting[2 * idx + 1]);
    }
#endif
    pr_info("QSC Version: 0x%x", read_cmos_eeprom_8(0x2a32));
}

static void write_sensor_QSC(void)
{
    if ((otp_flag == OTP_QSC_CUSTOM) || (otp_flag == OTP_QSC_INTERNAL)) {
        //LOG_INF("writing ... sizeof(imx766_QSC_setting) %d sizeof(kal_uint16): %d   a/b: %d\n", sizeof(imx766_QSC_setting), sizeof(kal_uint16), (sizeof(imx766_QSC_setting) / sizeof(kal_uint16)));
        //imx766_table_write_cmos_sensor(imx766_QSC_setting,
        //    sizeof(imx766_QSC_setting) / sizeof(kal_uint16));
        imx766_burst_write_cmos_sensor(imx766_QSC_setting,
            sizeof(imx766_QSC_setting) / sizeof(kal_uint16));
    }
}

static void imx766_get_pdaf_reg_setting(MUINT32 regNum, kal_uint16 *regDa)
{
    int i, idx;

    for (i = 0; i < regNum; i++) {
        idx = 2 * i;
        regDa[idx + 1] = read_cmos_sensor_8(regDa[idx]);
        /*LOG_INF("%x %x", regDa[idx], regDa[idx+1]);*/
    }
}
static void imx766_set_pdaf_reg_setting(MUINT32 regNum, kal_uint16 *regDa)
{
    imx766_table_write_cmos_sensor(regDa, regNum*2);
}

static kal_uint32 get_cur_exp_cnt(){
    kal_uint32 exp_cnt = 1;

    if (0x1 == (read_cmos_sensor_8(0x33D0) & 0x1)) { // DOL_EN
        if (0x1 == (read_cmos_sensor_8(0x33D1) & 0x3)) { // DOL_MODE
            exp_cnt = 3;
        } else {
            exp_cnt = 2;
        }
    }

    return exp_cnt;
}

static enum IMGSENSOR_RETURN write_frame_len(kal_uint32 fll)
{
    // //write_frame_len should be called inside GRP_PARAM_HOLD (0x0104)
    // FRM_LENGTH_LINES must be multiple of 4
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint32 exp_cnt = get_cur_exp_cnt();
    imgsensor.frame_length = round_up(fll / exp_cnt, 4) * exp_cnt;

    LOG_INF("extend_frame_length_en %d\n", imgsensor.extend_frame_length_en);
    if (imgsensor.extend_frame_length_en == KAL_FALSE) {
        LOG_INF("fll %d exp_cnt %d\n", imgsensor.frame_length, exp_cnt);
        ret += write_cmos_sensor_8(0x0340, imgsensor.frame_length / exp_cnt >> 8);
        ret += write_cmos_sensor_8(0x0341, imgsensor.frame_length / exp_cnt & 0xFF);
    }

    LOG_INF("fast_mode_on %d\n", imgsensor.fast_mode_on);
    if (imgsensor.fast_mode_on == KAL_TRUE) {
        imgsensor.fast_mode_on = KAL_FALSE;
        ret += write_cmos_sensor_8(0x3010, 0x00);
    }
    return ret;
}

static void set_dummy(void)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint8 write_retry = 2;
    LOG_INF("dummyline = %d, dummypixels = %d\n",
        imgsensor.dummy_line, imgsensor.dummy_pixel);
    /* return;*/ /* for test */
    while(write_retry) {
        ret = write_frame_len(imgsensor.frame_length);
        if (ret == IMGSENSOR_RETURN_SUCCESS)
            break;
        write_retry--;
        mdelay(2);
    }
}/*    set_dummy  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
    kal_uint8 itemp;

    LOG_INF("image_mirror = %d\n", image_mirror);
    itemp = read_cmos_sensor_8(0x0101);
    itemp &= ~0x03;

    switch (image_mirror) {
    case IMAGE_NORMAL:
        write_cmos_sensor_8(0x0101, itemp);
    break;
    case IMAGE_V_MIRROR:
        write_cmos_sensor_8(0x0101, itemp | 0x02);
    break;
    case IMAGE_H_MIRROR:
        write_cmos_sensor_8(0x0101, itemp | 0x01);
    break;
    case IMAGE_HV_MIRROR:
        write_cmos_sensor_8(0x0101, itemp | 0x03);
    break;
    }
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
    /*kal_int16 dummy_line;*/
    kal_uint32 frame_length = imgsensor.frame_length;

    LOG_INF("framerate = %d, min framelength should enable %d\n", framerate,
        min_framelength_en);

    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
    spin_lock(&imgsensor_drv_lock);
    if (frame_length >= imgsensor.min_frame_length)
        imgsensor.frame_length = frame_length;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;

    imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if (min_framelength_en)
        imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);
    set_dummy();
}/*    set_max_framerate  */

#define MAX_CIT_LSHIFT 7
static void write_shutter(kal_uint32 shutter, kal_bool gph)
{
    kal_uint16 realtime_fps = 0;
    kal_uint16 l_shift = 1;

    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    if (shutter < imgsensor_info.min_shutter) {
        shutter = imgsensor_info.min_shutter;
    }

    if (gph) {
        write_cmos_sensor_8(0x0104, 0x01);
    }

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        LOG_INF("autoflicker enable, realtime_fps = %d\n", realtime_fps);
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
    }

    /* long expsoure */
    if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) {
        for (l_shift = 1; l_shift < MAX_CIT_LSHIFT; l_shift++) {
            if ((shutter >> l_shift) < (imgsensor_info.max_frame_length - imgsensor_info.margin))
                break;
        }
        if (l_shift > MAX_CIT_LSHIFT) {
            LOG_INF("Unable to set such a long exposure %d, set to max\n", shutter);
            l_shift = MAX_CIT_LSHIFT;
        }
        shutter = shutter >> l_shift;
        imgsensor.frame_length = shutter + imgsensor_info.margin;
        LOG_INF("enter long exposure mode, time is %d", l_shift);
        write_cmos_sensor_8(0x3128, read_cmos_sensor(0x3128) | (l_shift & 0x7));
        write_frame_len(imgsensor.frame_length);
        /* Frame exposure mode customization for LE*/
        imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
        imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
        imgsensor.current_ae_effective_frame = 2;
    } else {
        write_cmos_sensor_8(0x3128, read_cmos_sensor(0x3128) & 0xf8);
        write_frame_len(imgsensor.frame_length);
        imgsensor.current_ae_effective_frame = 2;
        //LOG_INF("set frame_length\n");
    }

    /* Update Shutter */
    write_cmos_sensor_8(0x0350, 0x01); /* Enable auto extend */
    write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
    write_cmos_sensor_8(0x0203, shutter  & 0xFF);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x00);
    }

    LOG_INF("shutter =%d, framelength =%d read shutter h: 0x%x l: 0x%x\n",
        shutter, imgsensor.frame_length, read_cmos_sensor_8(0x0202), read_cmos_sensor_8(0x0203));
}/*    write_shutter  */

/*************************************************************************
 * FUNCTION
 *    set_shutter_w_gph
 *
 * DESCRIPTION
 *    This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *    iShutter : exposured lines,gph
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter_w_gph(kal_uint32 shutter, kal_bool gph)
{
    unsigned long flags;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    write_shutter(shutter, gph);
}

static void set_shutter(kal_uint32 shutter)
{
    LOG_INF("Fine Integ Time = %d", (read_cmos_sensor_8(0x0200) << 8) | read_cmos_sensor_8(0x0201));
    set_shutter_w_gph(shutter, KAL_TRUE);
} /* set_shutter */

static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length,
                kal_bool auto_extend_en)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    kal_int32 dummy_line = 0;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
    /* LOG_INF("shutter =%d, frame_time =%d\n", shutter, frame_time); */

    /* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger
     * than frame exposure
     */
    /* AE doesn't update sensor gain at capture mode, thus extra exposure
     * lines must be updated here.
     */

    /* OV Recommend Solution */
    /* if shutter bigger than frame_length,
     * should extend frame length first
     */
    spin_lock(&imgsensor_drv_lock);
    /*Change frame time */
    if (frame_length > 1)
        dummy_line = frame_length - imgsensor.frame_length;
    imgsensor.frame_length = imgsensor.frame_length + dummy_line;

    /*  */
    //if (shutter > imgsensor.frame_length - imgsensor_info.margin)
    //    imgsensor.frame_length = shutter + imgsensor_info.margin;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);

    shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
    shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
                   ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    write_cmos_sensor_8(0x0104, 0x01);
    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
        else {
            /* Extend frame length */
            write_frame_len(imgsensor.frame_length);
        }
    } else {
        /* Extend frame length */
        write_frame_len(imgsensor.frame_length);
    }

    /* Update Shutter */
    if (auto_extend_en)
        write_cmos_sensor_8(0x0350, 0x01); /* Enable auto extend */
    else
        write_cmos_sensor_8(0x0350, 0x00); /* Disable auto extend */
    write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
    write_cmos_sensor_8(0x0203, shutter & 0xFF);
    write_cmos_sensor_8(0x0104, 0x00);

    LOG_INF("Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
        shutter, imgsensor.frame_length, frame_length, dummy_line, read_cmos_sensor_8(0x0350));
}/* set_shutter_frame_length */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint16 reg_gain = 0x0;

    reg_gain = 16384 - (16384*64)/gain;
    return (kal_uint16) reg_gain;
}
#if 0
static kal_uint8 diggain2reg_u(const kal_uint16 gain)
{
    kal_uint8 reg_gain = 0x0;

    reg_gain = gain / 1024 ;
    LOG_INF("diggain2reg_u reg_gain(0x%x) gain(%d)", reg_gain, gain);
    return (kal_uint8) reg_gain;
}

static kal_uint8 diggain2reg_l(const kal_uint16 gain)
{
    kal_uint8 reg_gain = 0x0;
    kal_uint8 idx = 0;
    kal_uint32 diff = 0;
    float times = 0.0;
    float gain_times[] = {
        1.00, 1.01, 1.02, 1.04, 1.05, 1.06, 1.07, 1.08, 1.10, 1.11,
        1.12, 1.14, 1.15, 1.16, 1.18, 1.19, 1.20, 1.21, 1.23, 1.25,
        1.26, 1.27, 1.29, 1.30, 1.32, 1.33, 1.35, 1.36, 1.38, 1.39,
        1.41, 1.43, 1.45, 1.46, 1.48, 1.50, 1.51, 1.53, 1.55, 1.57,
        1.59, 1.60, 1.62, 1.64, 1.66, 1.68, 1.70, 1.72, 1.74, 1.76,
        1.78, 1.80, 1.82, 1.84, 1.86, 1.88, 1.91, 1.93, 1.95, 1.97,
        2.00, 2.02, 2.04, 2.07, 2.09, 2.11, 2.14, 2.16, 2.19, 2,21,
        2.24, 2.27, 2.29, 2.32, 2.34, 2.37, 2.40, 2.43, 2.45, 2.48,
        2.51, 2.54, 2.57, 2.60, 2.63, 2.66, 2.69, 2.72, 2.75, 2.79,
        2.82, 2.85, 2.88, 2.92, 2.95, 2.98, 3.02, 3.05, 3.09, 3.13,
        3.16, 3.20, 3.23, 3.27, 3.31, 3.35, 3.39, 3.43, 3.47, 3.51,
        3.55, 3.59, 3.63, 3.67, 3.71, 3.76, 3.80, 3.85, 3.89, 3.93,
        3.98, 4.03
    };
    kal_uint8 reg_l[] = {
        0x00, 0x03, 0x06, 0x09, 0x0c, 0x0f, 0x12, 0x15, 0x19, 0x1c,
        0x1f, 0x23, 0x26, 0x29, 0x2d, 0x30, 0x34, 0x37, 0x3b, 0x3f,
        0x42, 0x46, 0x4a, 0x4e, 0x51, 0x55, 0x59, 0x5d, 0x61, 0x65,
        0x6a, 0x6e, 0x72, 0x76, 0x7b, 0x7f, 0x83, 0x88, 0x8c, 0x91,
        0x96, 0x9a, 0x9f, 0xa4, 0xa9, 0xae, 0xb3, 0xb8, 0xbd, 0xc2,
        0xc7, 0xcd, 0xd2, 0xd7, 0xdd, 0xe2, 0xe8, 0xed, 0xf3, 0xf9,
        0xff, 0x05, 0x0b, 0x11, 0x17, 0x1d, 0x23, 0x2a, 0x30, 0x37,
        0x3d, 0x44, 0x4A, 0x51, 0x58, 0x5f, 0x66, 0x6d, 0x74, 0x7c,
        0x83, 0x8a, 0x92, 0x9a, 0xa1, 0xa9, 0xb1, 0xb9, 0xc1, 0xc9,
        0xd2, 0xda, 0xe2, 0xeb, 0xf4, 0xfc, 0x05, 0x0e, 0x17, 0x20,
        0x2a, 0x33, 0x3c, 0x46, 0x50, 0x5a, 0x63, 0x6d, 0x78, 0x82,
        0x8c, 0x97, 0xa1, 0xac, 0xb7, 0xc2, 0xcd, 0xd9, 0xe4, 0xef,
        0xfb, 0x07
    };

    times = gain / 1024 ;
    for (idx = 0; idx < (sizeof(reg_l) / sizeof(kal_uint8)); idx++) {
        diff = ((kal_uint32)(times * 1000)) - ((kal_uint32)(gain_times[idx] * 1000));
        if (diff < 0) {
            if (idx == 0)
                reg_gain = reg_l[idx];
            else
                reg_gain = reg_l[idx - 1];
        }else
            reg_gain = reg_l[idx - 1];
    }
    LOG_INF("diggain2reg_l size(%d) idx(%d) reg_gain(0x%x) times(%f) gain(%d)",
        (sizeof(reg_l) / sizeof(kal_uint8)), idx, reg_gain, times, gain);
    return (kal_uint16) reg_gain;
}
#endif
/*************************************************************************
 * FUNCTION
 *    set_gain
 *
 * DESCRIPTION
 *    This function is to set global gain to sensor.
 *
 * PARAMETERS
 *    iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *    the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain_w_gph(kal_uint16 gain, kal_bool gph)
{
    kal_uint16 reg_gain;

    if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
        LOG_INF("Error gain setting");
        if (gain < imgsensor_info.min_gain)
            gain = imgsensor_info.min_gain;
        else
           gain = imgsensor_info.max_gain;
    }

    if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3 || imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM7) {
        if (gain > 1024)
            gain = 1024;
    }

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("gain = %d, reg_gain = 0x%x\n ", gain, reg_gain);

    if (gph) {
        write_cmos_sensor_8(0x0104, 0x01);
    }
    write_cmos_sensor_8(0x0204, (reg_gain>>8) & 0xFF);
    write_cmos_sensor_8(0x0205, reg_gain & 0xFF);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x00);
    }

    return gain;
} /* set_gain_w_gph */

static kal_uint16 set_gain(kal_uint16 gain)
{
    return set_gain_w_gph(gain, KAL_TRUE);
} /* set_gain */

static kal_uint32 imx766_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
    return ERROR_NONE;
}

static kal_uint16 imx766_feedback_awbgain[] = {
    0x0b90, 0x00,
    0x0b91, 0x01,
    0x0b92, 0x00,
    0x0b93, 0x01,
};
/*write AWB gain to sensor*/
static void feedback_awbgain(kal_uint32 r_gain, kal_uint32 b_gain)
{
    UINT32 r_gain_int = 0;
    UINT32 b_gain_int = 0;

    //LOG_INF("r_gain: %d, b_gain: %d\n", r_gain, b_gain);
    r_gain_int = r_gain / 512;
    b_gain_int = b_gain / 512;
    imx766_feedback_awbgain[1] = r_gain_int;
    imx766_feedback_awbgain[3] = (((r_gain*100) / 512) - (r_gain_int * 100)) * 2;
    imx766_feedback_awbgain[5] = b_gain_int;
    imx766_feedback_awbgain[7] = (((b_gain * 100) / 512) - (b_gain_int * 100)) * 2;
    imx766_table_write_cmos_sensor(imx766_feedback_awbgain,
        sizeof(imx766_feedback_awbgain)/sizeof(kal_uint16));
}

/*************************************************************************
 * FUNCTION
 *    night_mode
 *
 * DESCRIPTION
 *    This function night mode of sensor.
 *
 * PARAMETERS
 *    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 streaming_control(kal_bool enable)
{
    LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
    if (enable)
        write_cmos_sensor_8(0x0100, 0X01);
    else
        write_cmos_sensor_8(0x0100, 0x00);
    return ERROR_NONE;
}

static kal_uint16 imx766_init_setting[] = {
    0x0136,0x18,
    0x0137,0x00,
    0x33F0,0x0D,
    0x33F1,0x08,
    0x0111,0x03,
    0x0101,0x00,
    0x33D3,0x01,
    0x3892,0x01,
    0x4C14,0x00,
    0x4C15,0x07,
    0x4C16,0x00,
    0x4C17,0x1B,
    0x4C1A,0x00,
    0x4C1B,0x03,
    0x4C1C,0x00,
    0x4C1D,0x00,
    0x4C1E,0x00,
    0x4C1F,0x02,
    0x4C20,0x00,
    0x4C21,0x5F,
    0x4C26,0x00,
    0x4C27,0x43,
    0x4C28,0x00,
    0x4C29,0x09,
    0x4C2A,0x00,
    0x4C2B,0x4A,
    0x4C2C,0x00,
    0x4C2D,0x00,
    0x4C2E,0x00,
    0x4C2F,0x02,
    0x4C30,0x00,
    0x4C31,0xC6,
    0x4C3E,0x00,
    0x4C3F,0x55,
    0x4C52,0x00,
    0x4C53,0x97,
    0x4CB4,0x00,
    0x4CB5,0x55,
    0x4CC8,0x00,
    0x4CC9,0x97,
    0x4D04,0x00,
    0x4D05,0x4F,
    0x4D74,0x00,
    0x4D75,0x55,
    0x4F06,0x00,
    0x4F07,0x5F,
    0x4F48,0x00,
    0x4F49,0xC6,
    0x544A,0xFF,
    0x544B,0xFF,
    0x544E,0x01,
    0x544F,0xBD,
    0x5452,0xFF,
    0x5453,0xFF,
    0x5456,0x00,
    0x5457,0xA5,
    0x545A,0xFF,
    0x545B,0xFF,
    0x545E,0x00,
    0x545F,0xA5,
    0x5496,0x00,
    0x5497,0xA2,
    0x54F6,0x01,
    0x54F7,0x55,
    0x54F8,0x01,
    0x54F9,0x61,
    0x5670,0x00,
    0x5671,0x85,
    0x5672,0x01,
    0x5673,0x77,
    0x5674,0x01,
    0x5675,0x2F,
    0x5676,0x02,
    0x5677,0x55,
    0x5678,0x00,
    0x5679,0x85,
    0x567A,0x01,
    0x567B,0x77,
    0x567C,0x01,
    0x567D,0x2F,
    0x567E,0x02,
    0x567F,0x55,
    0x5680,0x00,
    0x5681,0x85,
    0x5682,0x01,
    0x5683,0x77,
    0x5684,0x01,
    0x5685,0x2F,
    0x5686,0x02,
    0x5687,0x55,
    0x5688,0x00,
    0x5689,0x85,
    0x568A,0x01,
    0x568B,0x77,
    0x568C,0x01,
    0x568D,0x2F,
    0x568E,0x02,
    0x568F,0x55,
    0x5690,0x01,
    0x5691,0x7A,
    0x5692,0x02,
    0x5693,0x6C,
    0x5694,0x01,
    0x5695,0x35,
    0x5696,0x02,
    0x5697,0x5B,
    0x5698,0x01,
    0x5699,0x7A,
    0x569A,0x02,
    0x569B,0x6C,
    0x569C,0x01,
    0x569D,0x35,
    0x569E,0x02,
    0x569F,0x5B,
    0x56A0,0x01,
    0x56A1,0x7A,
    0x56A2,0x02,
    0x56A3,0x6C,
    0x56A4,0x01,
    0x56A5,0x35,
    0x56A6,0x02,
    0x56A7,0x5B,
    0x56A8,0x01,
    0x56A9,0x80,
    0x56AA,0x02,
    0x56AB,0x72,
    0x56AC,0x01,
    0x56AD,0x2F,
    0x56AE,0x02,
    0x56AF,0x55,
    0x5902,0x0E,
    0x5A50,0x04,
    0x5A51,0x04,
    0x5A69,0x01,
    0x5C49,0x0D,
    0x5D60,0x08,
    0x5D61,0x08,
    0x5D62,0x08,
    0x5D63,0x08,
    0x5D64,0x08,
    0x5D67,0x08,
    0x5D6C,0x08,
    0x5D6E,0x08,
    0x5D71,0x08,
    0x5D8E,0x14,
    0x5D90,0x03,
    0x5D91,0x0A,
    0x5D92,0x1F,
    0x5D93,0x05,
    0x5D97,0x1F,
    0x5D9A,0x06,
    0x5D9C,0x1F,
    0x5DA1,0x1F,
    0x5DA6,0x1F,
    0x5DA8,0x1F,
    0x5DAB,0x1F,
    0x5DC0,0x06,
    0x5DC1,0x06,
    0x5DC2,0x07,
    0x5DC3,0x06,
    0x5DC4,0x07,
    0x5DC7,0x07,
    0x5DCC,0x07,
    0x5DCE,0x07,
    0x5DD1,0x07,
    0x5E3E,0x00,
    0x5E3F,0x00,
    0x5E41,0x00,
    0x5E48,0x00,
    0x5E49,0x00,
    0x5E4A,0x00,
    0x5E4C,0x00,
    0x5E4D,0x00,
    0x5E4E,0x00,
    0x6026,0x03,
    0x6028,0x03,
    0x602A,0x03,
    0x602C,0x03,
    0x602F,0x03,
    0x6036,0x03,
    0x6038,0x03,
    0x603A,0x03,
    0x603C,0x03,
    0x603F,0x03,
    0x6074,0x19,
    0x6076,0x19,
    0x6078,0x19,
    0x607A,0x19,
    0x607D,0x19,
    0x6084,0x32,
    0x6086,0x32,
    0x6088,0x32,
    0x608A,0x32,
    0x608D,0x32,
    0x60C2,0x4A,
    0x60C4,0x4A,
    0x60CB,0x4A,
    0x60D2,0x4A,
    0x60D4,0x4A,
    0x60DB,0x4A,
    0x62F9,0x14,
    0x6305,0x13,
    0x6307,0x13,
    0x630A,0x13,
    0x630D,0x0D,
    0x6317,0x0D,
    0x632F,0x2E,
    0x6333,0x2E,
    0x6339,0x2E,
    0x6343,0x2E,
    0x6347,0x2E,
    0x634D,0x2E,
    0x6352,0x00,
    0x6353,0x5F,
    0x6366,0x00,
    0x6367,0x5F,
    0x638F,0x95,
    0x6393,0x95,
    0x6399,0x95,
    0x63A3,0x95,
    0x63A7,0x95,
    0x63AD,0x95,
    0x63B2,0x00,
    0x63B3,0xC6,
    0x63C6,0x00,
    0x63C7,0xC6,
    0x8BDB,0x02,
    0x8BDE,0x02,
    0x8BE1,0x2D,
    0x8BE4,0x00,
    0x8BE5,0x00,
    0x8BE6,0x01,
    0x9002,0x14,
    0x9200,0xB5,
    0x9201,0x9E,
    0x9202,0xB5,
    0x9203,0x42,
    0x9204,0xB5,
    0x9205,0x43,
    0x9206,0xBD,
    0x9207,0x20,
    0x9208,0xBD,
    0x9209,0x22,
    0x920A,0xBD,
    0x920B,0x23,
    0xB5D7,0x10,
    0xBD24,0x00,
    0xBD25,0x00,
    0xBD26,0x00,
    0xBD27,0x00,
    0xBD28,0x00,
    0xBD29,0x00,
    0xBD2A,0x00,
    0xBD2B,0x00,
    0xBD2C,0x32,
    0xBD2D,0x70,
    0xBD2E,0x25,
    0xBD2F,0x30,
    0xBD30,0x3B,
    0xBD31,0xE0,
    0xBD32,0x69,
    0xBD33,0x40,
    0xBD34,0x25,
    0xBD35,0x90,
    0xBD36,0x58,
    0xBD37,0x00,
    0xBD38,0x00,
    0xBD39,0x00,
    0xBD3A,0x00,
    0xBD3B,0x00,
    0xBD3C,0x32,
    0xBD3D,0x70,
    0xBD3E,0x25,
    0xBD3F,0x90,
    0xBD40,0x58,
    0xBD41,0x00,
    0x793B,0x01,
    0xACC6,0x00,
    0xACF5,0x00,
    0x793B,0x00,
    0x1F04,0xB3,
    0x1F05,0x01,
    0x1F06,0x07,
    0x1F07,0x66,
    0x1F08,0x01,
    0x4D18,0x00,
    0x4D19,0x9D,
    0x4D88,0x00,
    0x4D89,0x97,
    0x5C57,0x0A,
    0x5D94,0x1F,
    0x5D9E,0x1F,
    0x5E50,0x23,
    0x5E51,0x20,
    0x5E52,0x07,
    0x5E53,0x20,
    0x5E54,0x07,
    0x5E55,0x27,
    0x5E56,0x0B,
    0x5E57,0x24,
    0x5E58,0x0B,
    0x5E60,0x24,
    0x5E61,0x24,
    0x5E62,0x1B,
    0x5E63,0x23,
    0x5E64,0x1B,
    0x5E65,0x28,
    0x5E66,0x22,
    0x5E67,0x28,
    0x5E68,0x23,
    0x5E70,0x25,
    0x5E71,0x24,
    0x5E72,0x20,
    0x5E73,0x24,
    0x5E74,0x20,
    0x5E75,0x28,
    0x5E76,0x27,
    0x5E77,0x29,
    0x5E78,0x24,
    0x5E80,0x25,
    0x5E81,0x25,
    0x5E82,0x24,
    0x5E83,0x25,
    0x5E84,0x23,
    0x5E85,0x2A,
    0x5E86,0x28,
    0x5E87,0x2A,
    0x5E88,0x28,
    0x5E90,0x24,
    0x5E91,0x24,
    0x5E92,0x28,
    0x5E93,0x29,
    0x5E97,0x25,
    0x5E98,0x25,
    0x5E99,0x2A,
    0x5E9A,0x2A,
    0x5E9E,0x3A,
    0x5E9F,0x3F,
    0x5EA0,0x17,
    0x5EA1,0x3F,
    0x5EA2,0x17,
    0x5EA3,0x32,
    0x5EA4,0x10,
    0x5EA5,0x33,
    0x5EA6,0x10,
    0x5EAE,0x3D,
    0x5EAF,0x48,
    0x5EB0,0x3B,
    0x5EB1,0x45,
    0x5EB2,0x37,
    0x5EB3,0x3A,
    0x5EB4,0x31,
    0x5EB5,0x3A,
    0x5EB6,0x31,
    0x5EBE,0x40,
    0x5EBF,0x48,
    0x5EC0,0x3F,
    0x5EC1,0x45,
    0x5EC2,0x3F,
    0x5EC3,0x3A,
    0x5EC4,0x32,
    0x5EC5,0x3A,
    0x5EC6,0x33,
    0x5ECE,0x4B,
    0x5ECF,0x4A,
    0x5ED0,0x48,
    0x5ED1,0x4C,
    0x5ED2,0x45,
    0x5ED3,0x3F,
    0x5ED4,0x3A,
    0x5ED5,0x3F,
    0x5ED6,0x3A,
    0x5EDE,0x48,
    0x5EDF,0x45,
    0x5EE0,0x3A,
    0x5EE1,0x3A,
    0x5EE5,0x4A,
    0x5EE6,0x4C,
    0x5EE7,0x3F,
    0x5EE8,0x3F,
    0x5EEC,0x06,
    0x5EED,0x06,
    0x5EEE,0x02,
    0x5EEF,0x06,
    0x5EF0,0x01,
    0x5EF1,0x09,
    0x5EF2,0x05,
    0x5EF3,0x06,
    0x5EF4,0x04,
    0x5EFC,0x07,
    0x5EFD,0x09,
    0x5EFE,0x05,
    0x5EFF,0x08,
    0x5F00,0x04,
    0x5F01,0x09,
    0x5F02,0x05,
    0x5F03,0x09,
    0x5F04,0x04,
    0x5F0C,0x08,
    0x5F0D,0x09,
    0x5F0E,0x06,
    0x5F0F,0x09,
    0x5F10,0x06,
    0x5F11,0x09,
    0x5F12,0x09,
    0x5F13,0x09,
    0x5F14,0x06,
    0x5F1C,0x09,
    0x5F1D,0x09,
    0x5F1E,0x09,
    0x5F1F,0x09,
    0x5F20,0x08,
    0x5F21,0x09,
    0x5F22,0x09,
    0x5F23,0x09,
    0x5F24,0x09,
    0x5F2C,0x09,
    0x5F2D,0x09,
    0x5F2E,0x09,
    0x5F2F,0x09,
    0x5F33,0x09,
    0x5F34,0x09,
    0x5F35,0x09,
    0x5F36,0x09,
    0x5F3A,0x01,
    0x5F3D,0x07,
    0x5F3F,0x01,
    0x5F4B,0x01,
    0x5F4D,0x04,
    0x5F4F,0x02,
    0x5F51,0x02,
    0x5F5A,0x02,
    0x5F5B,0x01,
    0x5F5D,0x03,
    0x5F5E,0x07,
    0x5F5F,0x01,
    0x5F60,0x01,
    0x5F61,0x01,
    0x5F6A,0x01,
    0x5F6C,0x01,
    0x5F6D,0x01,
    0x5F6E,0x04,
    0x5F70,0x02,
    0x5F72,0x02,
    0x5F7A,0x01,
    0x5F7B,0x03,
    0x5F7C,0x01,
    0x5F7D,0x01,
    0x5F82,0x01,
    0x60C6,0x4A,
    0x60C8,0x4A,
    0x60D6,0x4A,
    0x60D8,0x4A,
    0x62E4,0x33,
    0x62E9,0x33,
    0x62EE,0x1C,
    0x62EF,0x33,
    0x62F3,0x33,
    0x62F6,0x1C,
    0x33F2,0x01,
    0x1F04,0xA3,
    0x1F05,0x01,
    0x406E,0x00,
    0x406F,0x08,
    0x4D08,0x00,
    0x4D09,0x2C,
    0x4D0E,0x00,
    0x4D0F,0x64,
    0x4D18,0x00,
    0x4D19,0xB1,
    0x4D1E,0x00,
    0x4D1F,0xCB,
    0x4D3A,0x00,
    0x4D3B,0x91,
    0x4D40,0x00,
    0x4D41,0x64,
    0x4D4C,0x00,
    0x4D4D,0xE8,
    0x4D52,0x00,
    0x4D53,0xCB,
    0x4D78,0x00,
    0x4D79,0x2C,
    0x4D7E,0x00,
    0x4D7F,0x64,
    0x4D88,0x00,
    0x4D89,0xAB,
    0x4D8E,0x00,
    0x4D8F,0xCB,
    0x4DA6,0x00,
    0x4DA7,0xE7,
    0x4DAC,0x00,
    0x4DAD,0xCB,
    0x5B98,0x00,
    0x5C52,0x05,
    0x5C57,0x09,
    0x5D94,0x0A,
    0x5D9E,0x0A,
    0x5E50,0x22,
    0x5E51,0x22,
    0x5E52,0x07,
    0x5E53,0x20,
    0x5E54,0x06,
    0x5E55,0x23,
    0x5E56,0x0A,
    0x5E57,0x23,
    0x5E58,0x0A,
    0x5E60,0x25,
    0x5E61,0x29,
    0x5E62,0x1C,
    0x5E63,0x26,
    0x5E64,0x1C,
    0x5E65,0x2D,
    0x5E66,0x1E,
    0x5E67,0x2A,
    0x5E68,0x1E,
    0x5E70,0x26,
    0x5E71,0x26,
    0x5E72,0x22,
    0x5E73,0x23,
    0x5E74,0x20,
    0x5E75,0x28,
    0x5E76,0x23,
    0x5E77,0x28,
    0x5E78,0x23,
    0x5E80,0x28,
    0x5E81,0x28,
    0x5E82,0x29,
    0x5E83,0x27,
    0x5E84,0x26,
    0x5E85,0x2A,
    0x5E86,0x2D,
    0x5E87,0x2A,
    0x5E88,0x2A,
    0x5E90,0x26,
    0x5E91,0x23,
    0x5E92,0x28,
    0x5E93,0x28,
    0x5E97,0x2F,
    0x5E98,0x2E,
    0x5E99,0x32,
    0x5E9A,0x32,
    0x5E9E,0x50,
    0x5E9F,0x50,
    0x5EA0,0x1E,
    0x5EA1,0x50,
    0x5EA2,0x1D,
    0x5EA3,0x3E,
    0x5EA4,0x14,
    0x5EA5,0x3E,
    0x5EA6,0x14,
    0x5EAE,0x58,
    0x5EAF,0x5E,
    0x5EB0,0x4B,
    0x5EB1,0x5A,
    0x5EB2,0x4B,
    0x5EB3,0x4C,
    0x5EB4,0x3A,
    0x5EB5,0x4C,
    0x5EB6,0x38,
    0x5EBE,0x56,
    0x5EBF,0x57,
    0x5EC0,0x50,
    0x5EC1,0x55,
    0x5EC2,0x50,
    0x5EC3,0x46,
    0x5EC4,0x3E,
    0x5EC5,0x46,
    0x5EC6,0x3E,
    0x5ECE,0x5A,
    0x5ECF,0x5F,
    0x5ED0,0x5E,
    0x5ED1,0x5A,
    0x5ED2,0x5A,
    0x5ED3,0x50,
    0x5ED4,0x4C,
    0x5ED5,0x50,
    0x5ED6,0x4C,
    0x5EDE,0x57,
    0x5EDF,0x55,
    0x5EE0,0x46,
    0x5EE1,0x46,
    0x5EE5,0x73,
    0x5EE6,0x6E,
    0x5EE7,0x5F,
    0x5EE8,0x5A,
    0x5EEC,0x0A,
    0x5EED,0x0A,
    0x5EEE,0x0F,
    0x5EEF,0x0A,
    0x5EF0,0x0E,
    0x5EF1,0x08,
    0x5EF2,0x0C,
    0x5EF3,0x0C,
    0x5EF4,0x0F,
    0x5EFC,0x0A,
    0x5EFD,0x0A,
    0x5EFE,0x14,
    0x5EFF,0x0A,
    0x5F00,0x14,
    0x5F01,0x0A,
    0x5F02,0x14,
    0x5F03,0x0A,
    0x5F04,0x19,
    0x5F0C,0x0A,
    0x5F0D,0x0A,
    0x5F0E,0x0A,
    0x5F0F,0x05,
    0x5F10,0x0A,
    0x5F11,0x06,
    0x5F12,0x08,
    0x5F13,0x0A,
    0x5F14,0x0C,
    0x5F1C,0x0A,
    0x5F1D,0x0A,
    0x5F1E,0x0A,
    0x5F1F,0x0A,
    0x5F20,0x0A,
    0x5F21,0x0A,
    0x5F22,0x0A,
    0x5F23,0x0A,
    0x5F24,0x0A,
    0x5F2C,0x0A,
    0x5F2D,0x05,
    0x5F2E,0x06,
    0x5F2F,0x0A,
    0x5F33,0x0A,
    0x5F34,0x0A,
    0x5F35,0x0A,
    0x5F36,0x0A,
    0x5F3A,0x00,
    0x5F3D,0x02,
    0x5F3F,0x0A,
    0x5F4A,0x0A,
    0x5F4B,0x0A,
    0x5F4D,0x0F,
    0x5F4F,0x00,
    0x5F51,0x00,
    0x5F5A,0x00,
    0x5F5B,0x00,
    0x5F5D,0x0A,
    0x5F5E,0x02,
    0x5F5F,0x0A,
    0x5F60,0x0A,
    0x5F61,0x00,
    0x5F6A,0x00,
    0x5F6C,0x0A,
    0x5F6D,0x06,
    0x5F6E,0x0F,
    0x5F70,0x00,
    0x5F72,0x00,
    0x5F7A,0x00,
    0x5F7B,0x0A,
    0x5F7C,0x0A,
    0x5F7D,0x00,
    0x5F82,0x06,
    0x60C6,0x36,
    0x60C8,0x36,
    0x60D6,0x36,
    0x60D8,0x36,
    0x62DF,0x56,
    0x62E0,0x52,
    0x62E4,0x38,
    0x62E5,0x51,
    0x62E9,0x35,
    0x62EA,0x54,
    0x62EE,0x1D,
    0x62EF,0x38,
    0x62F3,0x33,
    0x62F6,0x26,
    0x6412,0x1E,
    0x6413,0x1E,
    0x6414,0x1E,
    0x6415,0x1E,
    0x6416,0x1E,
    0x6417,0x1E,
    0x6418,0x1E,
    0x641A,0x1E,
    0x641B,0x1E,
    0x641C,0x1E,
    0x641D,0x1E,
    0x641E,0x1E,
    0x641F,0x1E,
    0x6420,0x1E,
    0x6421,0x1E,
    0x6422,0x1E,
    0x6424,0x1E,
    0x6425,0x1E,
    0x6426,0x1E,
    0x6427,0x1E,
    0x6428,0x1E,
    0x6429,0x1E,
    0x642A,0x1E,
    0x642B,0x1E,
    0x642C,0x1E,
    0x642E,0x1E,
    0x642F,0x1E,
    0x6430,0x1E,
    0x6431,0x1E,
    0x6432,0x1E,
    0x6433,0x1E,
    0x6434,0x1E,
    0x6435,0x1E,
    0x6436,0x1E,
    0x6438,0x1E,
    0x6439,0x1E,
    0x643A,0x1E,
    0x643B,0x1E,
    0x643D,0x1E,
    0x643E,0x1E,
    0x643F,0x1E,
    0x6441,0x1E,
    0x33F2,0x02,
    0x1F08,0x00,
    0xA307,0x30,
    0xA309,0x30,
    0xA30B,0x30,
    0xA406,0x03,
    0xA407,0x48,
    0xA408,0x03,
    0xA409,0x48,
    0xA40A,0x03,
    0xA40B,0x48,

    0x86A9,0x4E,
};

static kal_uint16 imx766_capture_30_setting[] = {
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x02,
    0x0342,0x3D,
    0x0343,0x00,
    0x0340,0x0C,
    0x0341,0x68,
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x00,
    0x0347,0x00,
    0x0348,0x1F,
    0x0349,0xFF,
    0x034A,0x17,
    0x034B,0xFF,
    0x0900,0x01,
    0x0901,0x22,
    0x0902,0x08,
    0x3005,0x03,
    0x3120,0x04,
    0x3121,0x01,
    0x3200,0x41,
    0x3201,0x41,
    0x32D6,0x00,
    0x0408,0x00,
    0x0409,0x00,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x10,
    0x040D,0x00,
    0x040E,0x0C,
    0x040F,0x00,
    0x034C,0x10,
    0x034D,0x00,
    0x034E,0x0C,
    0x034F,0x00,
    0x0301,0x05,
    0x0303,0x04,
    0x0305,0x04,
    0x0306,0x01,
    0x0307,0x36,
    0x030B,0x02,
    0x030D,0x03,
    0x030E,0x01,
    0x030F,0x1F,
    0x30CB,0x00,
    0x30CC,0x10,
    0x30CD,0x00,
    0x30CE,0x03,
    0x30CF,0x00,
    0x319C,0x01,
    0x3800,0x01,
    0x3801,0x01,
    0x3802,0x02,
    0x3847,0x03,
    0x38B0,0x00,
    0x38B1,0x00,
    0x38B2,0x00,
    0x38B3,0x00,
    0x38C4,0x01,
    0x38C5,0x2C,
    0x4C3A,0x02,
    0x4C3B,0xD2,
    0x4C68,0x04,
    0x4C69,0x7E,
    0x4CF8,0x07,
    0x4CF9,0x9E,
    0x4DB8,0x08,
    0x4DB9,0x98,
    0x0202,0x0C,
    0x0203,0x38,
    0x0224,0x01,
    0x0225,0xF4,
    0x313A,0x01,
    0x313B,0xF4,
    0x3803,0x00,
    0x3804,0x17,
    0x3805,0xC0,
    0x0204,0x00,
    0x0205,0x00,
    0x020E,0x01,
    0x020F,0x00,
    0x0216,0x00,
    0x0217,0x00,
    0x0218,0x01,
    0x0219,0x00,
    0x313C,0x00,
    0x313D,0x00,
    0x313E,0x01,
    0x313F,0x00,
    0x0860,0x01,
    0x0861,0x2D,
    0x0862,0x01,
    0x0863,0x2D,
    0x30B4,0x01,
    0x3066,0x00,
    0x3067,0x30,
    0x3068,0x00,
    0x3069,0x31,
    0x33D0,0x00,
    0x33D1,0x00,
    0x33D4,0x01,
    0x33DC,0x0A,
    0x33DD,0x0A,
    0x33DE,0x0A,
    0x33DF,0x0A,
    0x3070,0x01,
    0x3077,0x01,
    0x3078,0x30,
    0x3079,0x01,
    0x307A,0x30,
    0x307B,0x01,
    0x3080,0x02,
    0x3087,0x02,
    0x3088,0x30,
    0x3089,0x02,
    0x308A,0x30,
    0x308B,0x02,
    0x3901,0x2B,
    0x3902,0x00,
    0x3903,0x12,
    0x3905,0x2B,
    0x3906,0x01,
    0x3907,0x12,
    0x3909,0x2B,
    0x390A,0x02,
    0x390B,0x12,
    0x3911,0x00,
};

static kal_uint16 imx766_preview_setting[] = {
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x02,
    0x0342,0x3D,
    0x0343,0x00,
    0x0340,0x0C,
    0x0341,0x90,
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x00,
    0x0347,0x00,
    0x0348,0x1F,
    0x0349,0xFF,
    0x034A,0x17,
    0x034B,0xFF,
    0x0900,0x01,
    0x0901,0x22,
    0x0902,0x08,
    0x3005,0x03,
    0x3120,0x04,
    0x3121,0x01,
    0x3200,0x41,
    0x3201,0x41,
    0x32D6,0x00,
    0x0408,0x00,
    0x0409,0x00,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x10,
    0x040D,0x00,
    0x040E,0x0C,
    0x040F,0x00,
    0x034C,0x10,
    0x034D,0x00,
    0x034E,0x0C,
    0x034F,0x00,
    0x0301,0x05,
    0x0303,0x04,
    0x0305,0x04,
    0x0306,0x01,
    0x0307,0x3A,
    0x030B,0x02,
    0x030D,0x03,
    0x030E,0x01,
    0x030F,0x23,
    0x30CB,0x00,
    0x30CC,0x10,
    0x30CD,0x00,
    0x30CE,0x03,
    0x30CF,0x00,
    0x319C,0x01,
    0x3800,0x01,
    0x3801,0x01,
    0x3802,0x02,
    0x3847,0x03,
    0x38B0,0x00,
    0x38B1,0x00,
    0x38B2,0x00,
    0x38B3,0x00,
    0x38C4,0x01,
    0x38C5,0x2C,
    0x4C3A,0x02,
    0x4C3B,0xD2,
    0x4C68,0x04,
    0x4C69,0x7E,
    0x4CF8,0x07,
    0x4CF9,0x9E,
    0x4DB8,0x08,
    0x4DB9,0x98,
    0x0202,0x0C,
    0x0203,0x60,
    0x0224,0x01,
    0x0225,0xF4,
    0x313A,0x01,
    0x313B,0xF4,
    0x3803,0x00,
    0x3804,0x17,
    0x3805,0xC0,
    0x0204,0x00,
    0x0205,0x00,
    0x020E,0x01,
    0x020F,0x00,
    0x0216,0x00,
    0x0217,0x00,
    0x0218,0x01,
    0x0219,0x00,
    0x313C,0x00,
    0x313D,0x00,
    0x313E,0x01,
    0x313F,0x00,
    0x0860,0x01,
    0x0861,0x2D,
    0x0862,0x01,
    0x0863,0x2D,
    0x30B4,0x01,
    0x3066,0x00,
    0x3067,0x30,
    0x3068,0x00,
    0x3069,0x31,
    0x33D0,0x00,
    0x33D1,0x00,
    0x33D4,0x01,
    0x33DC,0x0A,
    0x33DD,0x0A,
    0x33DE,0x0A,
    0x33DF,0x0A,
    0x3070,0x01,
    0x3077,0x01,
    0x3078,0x30,
    0x3079,0x01,
    0x307A,0x30,
    0x307B,0x01,
    0x3080,0x02,
    0x3087,0x02,
    0x3088,0x30,
    0x3089,0x02,
    0x308A,0x30,
    0x308B,0x02,
    0x3901,0x2B,
    0x3902,0x00,
    0x3903,0x12,
    0x3905,0x2B,
    0x3906,0x01,
    0x3907,0x12,
    0x3909,0x2B,
    0x390A,0x02,
    0x390B,0x12,
    0x3911,0x00,
};

static kal_uint16 imx766_normal_video_setting[] = {
//reg_Q-2-4
//QBIN(HVBIN)_4096x2304_30FPS 1372M
//H: 4096
//V: 2304
//mipi_pixel_rate:940.80M
//line_length:4794
//frame_length:15616
//PCLK:2246.40M
//MIPI output setting
//Address	value
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
//Line Length PCK Setting
//Address	value
	0x0342, 0x3D,
	0x0343, 0x00,
//Frame Length Lines Setting
//Address	value
	0x0340, 0x12,
	0x0341, 0xBA,
//ROI Setting
//Address	value
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x03,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0xFF,
	0x034A, 0x14,
	0x034B, 0xFF,
//Mode Setting
//Address	value
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3005, 0x03,
	0x3120, 0x04,
	0x3121, 0x01,
	0x3200, 0x41,
	0x3201, 0x41,
	0x32D6, 0x00,
//Digital Crop & Scaling
//Address	value
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x10,
	0x040D, 0x00,
	0x040E, 0x09,
	0x040F, 0x00,
//Output Size Setting
//Address	value
	0x034C, 0x10,
	0x034D, 0x00,
	0x034E, 0x09,
	0x034F, 0x00,
//Clock Setting
//Address	value
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xEA,
	0x030B, 0x02,
	0x030D, 0x03,
	0x030E, 0x01,
	0x030F, 0x56,
//Other Setting
//Address	value
	0x30CB, 0x00,
	0x30CC, 0x10,
	0x30CD, 0x00,
	0x30CE, 0x03,
	0x30CF, 0x00,
	0x319C, 0x01,
	0x3800, 0x01,
	0x3801, 0x01,
	0x3802, 0x02,
	0x3847, 0x03,
	0x38B0, 0x00,
	0x38B1, 0x00,
	0x38B2, 0x00,
	0x38B3, 0x00,
	0x38C4, 0x01,
	0x38C5, 0x2C,
	0x4C3A, 0x02,
	0x4C3B, 0xD2,
	0x4C68, 0x04,
	0x4C69, 0x7E,
	0x4CF8, 0x07,
	0x4CF9, 0x9E,
	0x4DB8, 0x08,
	0x4DB9, 0x98,
//Integration Setting
//Address	value
	0x0202, 0x12,
	0x0203, 0x8A,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x313A, 0x01,
	0x313B, 0xF4,
	0x3803, 0x00,
	0x3804, 0x17,
	0x3805, 0xC0,
//Gain Setting
//Address	value
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0216, 0x00,
	0x0217, 0x00,
	0x0218, 0x01,
	0x0219, 0x00,
	0x313C, 0x00,
	0x313D, 0x00,
	0x313E, 0x01,
	0x313F, 0x00,
//EPD Setting
//Address	value
	0x0860, 0x01,
	0x0861, 0x2D,
	0x0862, 0x01,
	0x0863, 0x2D,
//PHASE PIX Setting
//Address	value
	0x30B4, 0x01,
//PHASE PIX data type Setting
//Address	value
	0x3066, 0x00,
	0x3067, 0x30,
	0x3068, 0x00,
	0x3069, 0x31,
//DOL Setting
//Address	value
	0x33D0, 0x00,
	0x33D1, 0x00,
	0x33D4, 0x01,
	0x33DC, 0x0A,
	0x33DD, 0x0A,
	0x33DE, 0x0A,
	0x33DF, 0x0A,
//DOL data type Setting
//Address	value
	0x3070, 0x01,
	0x3077, 0x01,
	0x3078, 0x30,
	0x3079, 0x01,
	0x307A, 0x30,
	0x307B, 0x01,
	0x3080, 0x02,
	0x3087, 0x02,
	0x3088, 0x30,
	0x3089, 0x02,
	0x308A, 0x30,
	0x308B, 0x02,
	0x3901, 0x2B,
	0x3902, 0x00,
	0x3903, 0x12,
	0x3905, 0x2B,
	0x3906, 0x01,
	0x3907, 0x12,
	0x3909, 0x2B,
	0x390A, 0x02,
	0x390B, 0x12,
	0x3911, 0x00,
};

static kal_uint16 imx766_hs_video_setting[] = {
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x02,
    0x0342,0x22,
    0x0343,0x70,
    0x0340,0x04,
    0x0341,0xCC,
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x03,
    0x0347,0x00,
    0x0348,0x1F,
    0x0349,0xFF,
    0x034A,0x14,
    0x034B,0xFF,
    0x0900,0x01,
    0x0901,0x44,
    0x0902,0x08,
    0x3005,0x03,
    0x3120,0x04,
    0x3121,0x00,
    0x3200,0x43,
    0x3201,0x43,
    0x32D6,0x00,
    0x0408,0x00,
    0x0409,0x00,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x08,
    0x040D,0x00,
    0x040E,0x04,
    0x040F,0x80,
    0x034C,0x08,
    0x034D,0x00,
    0x034E,0x04,
    0x034F,0x80,
    0x0301,0x05,
    0x0303,0x04,
    0x0305,0x04,
    0x0306,0x01,
    0x0307,0x0F,
    0x030B,0x04,
    0x030D,0x03,
    0x030E,0x01,
    0x030F,0x79,
    0x30CB,0x00,
    0x30CC,0x10,
    0x30CD,0x00,
    0x30CE,0x03,
    0x30CF,0x00,
    0x319C,0x01,
    0x3800,0x01,
    0x3801,0x01,
    0x3802,0x02,
    0x3847,0x03,
    0x38B0,0x00,
    0x38B1,0x64,
    0x38B2,0x00,
    0x38B3,0x64,
    0x38C4,0x00,
    0x38C5,0x64,
    0x4C3A,0x02,
    0x4C3B,0xD2,
    0x4C68,0x04,
    0x4C69,0x7E,
    0x4CF8,0x07,
    0x4CF9,0x9E,
    0x4DB8,0x08,
    0x4DB9,0x98,
    0x0202,0x04,
    0x0203,0x9C,
    0x0224,0x01,
    0x0225,0xF4,
    0x313A,0x01,
    0x313B,0xF4,
    0x3803,0x00,
    0x3804,0x17,
    0x3805,0xC0,
    0x0204,0x00,
    0x0205,0x00,
    0x020E,0x01,
    0x020F,0x00,
    0x0216,0x00,
    0x0217,0x00,
    0x0218,0x01,
    0x0219,0x00,
    0x313C,0x00,
    0x313D,0x00,
    0x313E,0x01,
    0x313F,0x00,
    0x0860,0x01,
    0x0861,0x2D,
    0x0862,0x01,
    0x0863,0x2D,
    0x30B4,0x01,
    0x3066,0x00,
    0x3067,0x30,
    0x3068,0x00,
    0x3069,0x31,
    0x33D0,0x00,
    0x33D1,0x00,
    0x33D4,0x01,
    0x33DC,0x0A,
    0x33DD,0x0A,
    0x33DE,0x0A,
    0x33DF,0x0A,
    0x3070,0x01,
    0x3077,0x01,
    0x3078,0x30,
    0x3079,0x01,
    0x307A,0x30,
    0x307B,0x01,
    0x3080,0x02,
    0x3087,0x02,
    0x3088,0x30,
    0x3089,0x02,
    0x308A,0x30,
    0x308B,0x02,
    0x3901,0x2B,
    0x3902,0x00,
    0x3903,0x12,
    0x3905,0x2B,
    0x3906,0x01,
    0x3907,0x12,
    0x3909,0x2B,
    0x390A,0x02,
    0x390B,0x12,
    0x3911,0x00,
};

static kal_uint16 imx766_slim_video_setting[] = {
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x02,
    0x0342,0x3D,
    0x0343,0x00,
    0x0340,0x0C,
    0x0341,0x68,
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x00,
    0x0347,0x00,
    0x0348,0x1F,
    0x0349,0xFF,
    0x034A,0x17,
    0x034B,0xFF,
    0x0900,0x01,
    0x0901,0x22,
    0x0902,0x08,
    0x3005,0x03,
    0x3120,0x04,
    0x3121,0x01,
    0x3200,0x41,
    0x3201,0x41,
    0x32D6,0x00,
    0x0408,0x00,
    0x0409,0x00,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x10,
    0x040D,0x00,
    0x040E,0x0C,
    0x040F,0x00,
    0x034C,0x10,
    0x034D,0x00,
    0x034E,0x0C,
    0x034F,0x00,
    0x0301,0x05,
    0x0303,0x04,
    0x0305,0x04,
    0x0306,0x01,
    0x0307,0x36,
    0x030B,0x02,
    0x030D,0x03,
    0x030E,0x01,
    0x030F,0x1F,
    0x30CB,0x00,
    0x30CC,0x10,
    0x30CD,0x00,
    0x30CE,0x03,
    0x30CF,0x00,
    0x319C,0x01,
    0x3800,0x01,
    0x3801,0x01,
    0x3802,0x02,
    0x3847,0x03,
    0x38B0,0x00,
    0x38B1,0x00,
    0x38B2,0x00,
    0x38B3,0x00,
    0x38C4,0x01,
    0x38C5,0x2C,
    0x4C3A,0x02,
    0x4C3B,0xD2,
    0x4C68,0x04,
    0x4C69,0x7E,
    0x4CF8,0x07,
    0x4CF9,0x9E,
    0x4DB8,0x08,
    0x4DB9,0x98,
    0x0202,0x0C,
    0x0203,0x38,
    0x0224,0x01,
    0x0225,0xF4,
    0x313A,0x01,
    0x313B,0xF4,
    0x3803,0x00,
    0x3804,0x17,
    0x3805,0xC0,
    0x0204,0x00,
    0x0205,0x00,
    0x020E,0x01,
    0x020F,0x00,
    0x0216,0x00,
    0x0217,0x00,
    0x0218,0x01,
    0x0219,0x00,
    0x313C,0x00,
    0x313D,0x00,
    0x313E,0x01,
    0x313F,0x00,
    0x0860,0x01,
    0x0861,0x2D,
    0x0862,0x01,
    0x0863,0x2D,
    0x30B4,0x01,
    0x3066,0x00,
    0x3067,0x30,
    0x3068,0x00,
    0x3069,0x31,
    0x33D0,0x00,
    0x33D1,0x00,
    0x33D4,0x01,
    0x33DC,0x0A,
    0x33DD,0x0A,
    0x33DE,0x0A,
    0x33DF,0x0A,
    0x3070,0x01,
    0x3077,0x01,
    0x3078,0x30,
    0x3079,0x01,
    0x307A,0x30,
    0x307B,0x01,
    0x3080,0x02,
    0x3087,0x02,
    0x3088,0x30,
    0x3089,0x02,
    0x308A,0x30,
    0x308B,0x02,
    0x3901,0x2B,
    0x3902,0x00,
    0x3903,0x12,
    0x3905,0x2B,
    0x3906,0x01,
    0x3907,0x12,
    0x3909,0x2B,
    0x390A,0x02,
    0x390B,0x12,
    0x3911,0x00,
};

static kal_uint16 imx766_custom1_setting[] = {
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x02,
    0x0342,0x3D,
    0x0343,0x00,
    0x0340,0x0F,
    0x0341,0xA8,
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x00,
    0x0347,0x00,
    0x0348,0x1F,
    0x0349,0xFF,
    0x034A,0x17,
    0x034B,0xFF,
    0x0900,0x01,
    0x0901,0x22,
    0x0902,0x08,
    0x3005,0x03,
    0x3120,0x04,
    0x3121,0x01,
    0x3200,0x41,
    0x3201,0x41,
    0x32D6,0x00,
    0x0408,0x00,
    0x0409,0x00,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x10,
    0x040D,0x00,
    0x040E,0x0C,
    0x040F,0x00,
    0x034C,0x10,
    0x034D,0x00,
    0x034E,0x0C,
    0x034F,0x00,
    0x0301,0x05,
    0x0303,0x04,
    0x0305,0x04,
    0x0306,0x01,
    0x0307,0x39,
    0x030B,0x04,
    0x030D,0x03,
    0x030E,0x01,
    0x030F,0xAC,
    0x30CB,0x00,
    0x30CC,0x10,
    0x30CD,0x00,
    0x30CE,0x03,
    0x30CF,0x00,
    0x319C,0x01,
    0x3800,0x01,
    0x3801,0x01,
    0x3802,0x02,
    0x3847,0x03,
    0x38B0,0x00,
    0x38B1,0x00,
    0x38B2,0x00,
    0x38B3,0x00,
    0x38C4,0x01,
    0x38C5,0x2C,
    0x4C3A,0x02,
    0x4C3B,0xD2,
    0x4C68,0x04,
    0x4C69,0x7E,
    0x4CF8,0x07,
    0x4CF9,0x9E,
    0x4DB8,0x08,
    0x4DB9,0x98,
    0x0202,0x0F,
    0x0203,0x78,
    0x0224,0x01,
    0x0225,0xF4,
    0x313A,0x01,
    0x313B,0xF4,
    0x3803,0x00,
    0x3804,0x17,
    0x3805,0xC0,
    0x0204,0x00,
    0x0205,0x00,
    0x020E,0x01,
    0x020F,0x00,
    0x0216,0x00,
    0x0217,0x00,
    0x0218,0x01,
    0x0219,0x00,
    0x313C,0x00,
    0x313D,0x00,
    0x313E,0x01,
    0x313F,0x00,
    0x0860,0x01,
    0x0861,0x2D,
    0x0862,0x01,
    0x0863,0x2D,
    0x30B4,0x01,
    0x3066,0x00,
    0x3067,0x30,
    0x3068,0x00,
    0x3069,0x31,
    0x33D0,0x00,
    0x33D1,0x00,
    0x33D4,0x01,
    0x33DC,0x0A,
    0x33DD,0x0A,
    0x33DE,0x0A,
    0x33DF,0x0A,
    0x3070,0x01,
    0x3077,0x01,
    0x3078,0x30,
    0x3079,0x01,
    0x307A,0x30,
    0x307B,0x01,
    0x3080,0x02,
    0x3087,0x02,
    0x3088,0x30,
    0x3089,0x02,
    0x308A,0x30,
    0x308B,0x02,
    0x3901,0x2B,
    0x3902,0x00,
    0x3903,0x12,
    0x3905,0x2B,
    0x3906,0x01,
    0x3907,0x12,
    0x3909,0x2B,
    0x390A,0x02,
    0x390B,0x12,
    0x3911,0x00,
};

static kal_uint16 imx766_custom2_setting[] = {
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x02,
    0x0342,0x3D,
    0x0343,0x00,
    0x0340,0x09,
    0x0341,0x86,
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x03,
    0x0347,0x00,
    0x0348,0x1F,
    0x0349,0xFF,
    0x034A,0x14,
    0x034B,0xFF,
    0x0900,0x01,
    0x0901,0x22,
    0x0902,0x08,
    0x3005,0x03,
    0x3120,0x04,
    0x3121,0x01,
    0x3200,0x41,
    0x3201,0x41,
    0x32D6,0x00,
    0x0408,0x00,
    0x0409,0x00,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x10,
    0x040D,0x00,
    0x040E,0x09,
    0x040F,0x00,
    0x034C,0x10,
    0x034D,0x00,
    0x034E,0x09,
    0x034F,0x00,
    0x0301,0x05,
    0x0303,0x02,
    0x0305,0x04,
    0x0306,0x00,
    0x0307,0xEE,
    0x030B,0x02,
    0x030D,0x03,
    0x030E,0x01,
    0x030F,0x46,
    0x30CB,0x00,
    0x30CC,0x10,
    0x30CD,0x00,
    0x30CE,0x03,
    0x30CF,0x00,
    0x319C,0x01,
    0x3800,0x01,
    0x3801,0x01,
    0x3802,0x02,
    0x3847,0x03,
    0x38B0,0x00,
    0x38B1,0x00,
    0x38B2,0x00,
    0x38B3,0x00,
    0x38C4,0x01,
    0x38C5,0x2C,
    0x4C3A,0x02,
    0x4C3B,0xD2,
    0x4C68,0x04,
    0x4C69,0x7E,
    0x4CF8,0x07,
    0x4CF9,0x9E,
    0x4DB8,0x08,
    0x4DB9,0x98,
    0x0202,0x09,
    0x0203,0x56,
    0x0224,0x01,
    0x0225,0xF4,
    0x313A,0x01,
    0x313B,0xF4,
    0x3803,0x00,
    0x3804,0x17,
    0x3805,0xC0,
    0x0204,0x00,
    0x0205,0x00,
    0x020E,0x01,
    0x020F,0x00,
    0x0216,0x00,
    0x0217,0x00,
    0x0218,0x01,
    0x0219,0x00,
    0x313C,0x00,
    0x313D,0x00,
    0x313E,0x01,
    0x313F,0x00,
    0x0860,0x01,
    0x0861,0x2D,
    0x0862,0x01,
    0x0863,0x2D,
    0x30B4,0x01,
    0x3066,0x00,
    0x3067,0x30,
    0x3068,0x00,
    0x3069,0x31,
    0x33D0,0x00,
    0x33D1,0x00,
    0x33D4,0x01,
    0x33DC,0x0A,
    0x33DD,0x0A,
    0x33DE,0x0A,
    0x33DF,0x0A,
    0x3070,0x01,
    0x3077,0x01,
    0x3078,0x30,
    0x3079,0x01,
    0x307A,0x30,
    0x307B,0x01,
    0x3080,0x02,
    0x3087,0x02,
    0x3088,0x30,
    0x3089,0x02,
    0x308A,0x30,
    0x308B,0x02,
    0x3901,0x2B,
    0x3902,0x00,
    0x3903,0x12,
    0x3905,0x2B,
    0x3906,0x01,
    0x3907,0x12,
    0x3909,0x2B,
    0x390A,0x02,
    0x390B,0x12,
    0x3911,0x00,
};

static kal_uint16 imx766_custom3_setting[] = {
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x02,
    0x0342,0x2D,
    0x0343,0x20,
    0x0340,0x18,
    0x0341,0xAE,
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x00,
    0x0347,0x00,
    0x0348,0x1F,
    0x0349,0xFF,
    0x034A,0x17,
    0x034B,0xFF,
    0x0900,0x00,
    0x0901,0x11,
    0x0902,0x0A,
    0x3005,0x00,
    0x3120,0x00,
    0x3121,0x01,
    0x3200,0x00,
    0x3201,0x00,
    0x32D6,0x01,
    0x0408,0x00,
    0x0409,0x00,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x20,
    0x040D,0x00,
    0x040E,0x18,
    0x040F,0x00,
    0x034C,0x20,
    0x034D,0x00,
    0x034E,0x18,
    0x034F,0x00,
    0x0301,0x05,
    0x0303,0x04,
    0x0305,0x04,
    0x0306,0x01,
    0x0307,0x6D,
    0x030B,0x01,
    0x030D,0x03,
    0x030E,0x01,
    0x030F,0x3D,
    0x30CB,0x00,
    0x30CC,0x10,
    0x30CD,0x00,
    0x30CE,0x03,
    0x30CF,0x00,
    0x319C,0x00,
    0x3800,0x00,
    0x3801,0x00,
    0x3802,0x04,
    0x3847,0x00,
    0x38B0,0x00,
    0x38B1,0x00,
    0x38B2,0x00,
    0x38B3,0x00,
    0x38C4,0x02,
    0x38C5,0x26,
    0x4C3A,0x02,
    0x4C3B,0xD2,
    0x4C68,0x04,
    0x4C69,0x7E,
    0x4CF8,0x07,
    0x4CF9,0x9E,
    0x4DB8,0x08,
    0x4DB9,0x98,
    0x0202,0x18,
    0x0203,0x7E,
    0x0224,0x01,
    0x0225,0xF4,
    0x313A,0x01,
    0x313B,0xF4,
    0x3803,0x01,
    0x3804,0x16,
    0x3805,0xB0,
    0x0204,0x00,
    0x0205,0x00,
    0x020E,0x01,
    0x020F,0x00,
    0x0216,0x00,
    0x0217,0x00,
    0x0218,0x01,
    0x0219,0x00,
    0x313C,0x00,
    0x313D,0x00,
    0x313E,0x01,
    0x313F,0x00,
    0x0860,0x01,
    0x0861,0x2D,
    0x0862,0x01,
    0x0863,0x2D,
    0x30B4,0x01,
    0x3066,0x00,
    0x3067,0x30,
    0x3068,0x00,
    0x3069,0x31,
    0x33D0,0x00,
    0x33D1,0x00,
    0x33D4,0x01,
    0x33DC,0x0A,
    0x33DD,0x0A,
    0x33DE,0x0A,
    0x33DF,0x0A,
    0x3070,0x01,
    0x3077,0x01,
    0x3078,0x30,
    0x3079,0x01,
    0x307A,0x30,
    0x307B,0x01,
    0x3080,0x02,
    0x3087,0x02,
    0x3088,0x30,
    0x3089,0x02,
    0x308A,0x30,
    0x308B,0x02,
    0x3901,0x2B,
    0x3902,0x00,
    0x3903,0x12,
    0x3905,0x2B,
    0x3906,0x01,
    0x3907,0x12,
    0x3909,0x2B,
    0x390A,0x02,
    0x390B,0x12,
    0x3911,0x00,
};

static kal_uint16 imx766_custom4_setting[] = {
//reg_R-2-4
//QBIN(HVBIN)_4096x2304_30FPS 1372M, 2DOL 1ST
//H: 4096
//V: 2304
//mipi_pixel_rate:940.80M
//line_length:4792
//frame_length:15616
//PCLK:2246.40M
//MIPI output setting
//Address	value
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
//Line Length PCK Setting
//Address	value
	0x0342, 0x3D,
	0x0343, 0x00,
//Frame Length Lines Setting
//Address	value
	0x0340, 0x09,
	0x0341, 0x5C,
//ROI Setting
//Address	value
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x03,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0xFF,
	0x034A, 0x14,
	0x034B, 0xFF,
//Mode Setting
//Address	value
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3005, 0x03,
	0x3120, 0x04,
	0x3121, 0x01,
	0x3200, 0x41,
	0x3201, 0x41,
	0x32D6, 0x00,
//Digital Crop & Scaling
//Address	value
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x10,
	0x040D, 0x00,
	0x040E, 0x09,
	0x040F, 0x00,
//Output Size Setting
//Address	value
	0x034C, 0x10,
	0x034D, 0x00,
	0x034E, 0x09,
	0x034F, 0x00,
//Clock Setting
//Address	value
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xEA,
	0x030B, 0x02,
	0x030D, 0x03,
	0x030E, 0x01,
	0x030F, 0x56,
//Other Setting
//Address	value
	0x30CB, 0x00,
	0x30CC, 0x10,
	0x30CD, 0x00,
	0x30CE, 0x03,
	0x30CF, 0x00,
	0x319C, 0x01,
	0x3800, 0x01,
	0x3801, 0x01,
	0x3802, 0x02,
	0x3847, 0x03,
	0x38B0, 0x00,
	0x38B1, 0x00,
	0x38B2, 0x00,
	0x38B3, 0x00,
	0x38C4, 0x01,
	0x38C5, 0x2C,
	0x4C3A, 0x02,
	0x4C3B, 0xD2,
	0x4C68, 0x04,
	0x4C69, 0x7E,
	0x4CF8, 0x07,
	0x4CF9, 0x9E,
	0x4DB8, 0x08,
	0x4DB9, 0x98,
//Integration Setting
//Address	value
	0x0202, 0x07,
	0x0203, 0xE0,
	0x0224, 0x00,
	0x0225, 0xFC,
	0x313A, 0x01,
	0x313B, 0xF4,
	0x3803, 0x00,
	0x3804, 0x17,
	0x3805, 0xC0,
//Gain Setting
//Address	value
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0216, 0x00,
	0x0217, 0x00,
	0x0218, 0x01,
	0x0219, 0x00,
	0x313C, 0x00,
	0x313D, 0x00,
	0x313E, 0x01,
	0x313F, 0x00,
//EPD Setting
//Address	value
	0x0860, 0x01,
	0x0861, 0x2D,
	0x0862, 0x01,
	0x0863, 0x2D,
//PHASE PIX Setting
//Address	value
	0x30B4, 0x03,
//PHASE PIX data type Setting
//Address	value
	0x3066, 0x00,
	0x3067, 0x30,
	0x3068, 0x00,
	0x3069, 0x31,
//DOL Setting
//Address	value
	0x33D0, 0x01,
	0x33D1, 0x00,
	0x33D4, 0x01,
	0x33DC, 0x0A,
	0x33DD, 0x0A,
	0x33DE, 0x0A,
	0x33DF, 0x0A,
//DOL data type Setting
//Address	value
	0x3070, 0x01,
	0x3077, 0x01,
	0x3078, 0x30,
	0x3079, 0x01,
	0x307A, 0x30,
	0x307B, 0x01,
	0x3080, 0x02,
	0x3087, 0x02,
	0x3088, 0x30,
	0x3089, 0x02,
	0x308A, 0x30,
	0x308B, 0x02,
	0x3901, 0x2B,
	0x3902, 0x00,
	0x3903, 0x12,
	0x3905, 0x2B,
	0x3906, 0x01,
	0x3907, 0x12,
	0x3909, 0x2B,
	0x390A, 0x02,
	0x390B, 0x12,
	0x3911, 0x00,
};

static kal_uint16 imx766_custom5_setting[] = {
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x02,
    0x0342,0x22,
    0x0343,0x70,
    0x0340,0x05,
    0x0341,0x04,
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x03,
    0x0347,0x00,
    0x0348,0x1F,
    0x0349,0xFF,
    0x034A,0x14,
    0x034B,0xFF,
    0x0900,0x01,
    0x0901,0x44,
    0x0902,0x08,
    0x3005,0x03,
    0x3120,0x04,
    0x3121,0x00,
    0x3200,0x43,
    0x3201,0x43,
    0x32D6,0x00,
    0x0408,0x00,
    0x0409,0x00,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x08,
    0x040D,0x00,
    0x040E,0x04,
    0x040F,0x80,
    0x034C,0x08,
    0x034D,0x00,
    0x034E,0x04,
    0x034F,0x80,
    0x0301,0x05,
    0x0303,0x02,
    0x0305,0x04,
    0x0306,0x01,
    0x0307,0x1B,
    0x030B,0x02,
    0x030D,0x03,
    0x030E,0x01,
    0x030F,0x95,
    0x30CB,0x00,
    0x30CC,0x10,
    0x30CD,0x00,
    0x30CE,0x03,
    0x30CF,0x00,
    0x319C,0x01,
    0x3800,0x01,
    0x3801,0x01,
    0x3802,0x02,
    0x3847,0x03,
    0x38B0,0x00,
    0x38B1,0x64,
    0x38B2,0x00,
    0x38B3,0x64,
    0x38C4,0x00,
    0x38C5,0x64,
    0x4C3A,0x02,
    0x4C3B,0xD2,
    0x4C68,0x04,
    0x4C69,0x7E,
    0x4CF8,0x07,
    0x4CF9,0x9E,
    0x4DB8,0x08,
    0x4DB9,0x98,
    0x0202,0x04,
    0x0203,0xD4,
    0x0224,0x01,
    0x0225,0xF4,
    0x313A,0x01,
    0x313B,0xF4,
    0x3803,0x00,
    0x3804,0x17,
    0x3805,0xC0,
    0x0204,0x00,
    0x0205,0x00,
    0x020E,0x01,
    0x020F,0x00,
    0x0216,0x00,
    0x0217,0x00,
    0x0218,0x01,
    0x0219,0x00,
    0x313C,0x00,
    0x313D,0x00,
    0x313E,0x01,
    0x313F,0x00,
    0x0860,0x01,
    0x0861,0x2D,
    0x0862,0x01,
    0x0863,0x2D,
    0x30B4,0x01,
    0x3066,0x00,
    0x3067,0x30,
    0x3068,0x00,
    0x3069,0x31,
    0x33D0,0x00,
    0x33D1,0x00,
    0x33D4,0x01,
    0x33DC,0x0A,
    0x33DD,0x0A,
    0x33DE,0x0A,
    0x33DF,0x0A,
    0x3070,0x01,
    0x3077,0x01,
    0x3078,0x30,
    0x3079,0x01,
    0x307A,0x30,
    0x307B,0x01,
    0x3080,0x02,
    0x3087,0x02,
    0x3088,0x30,
    0x3089,0x02,
    0x308A,0x30,
    0x308B,0x02,
    0x3901,0x2B,
    0x3902,0x00,
    0x3903,0x12,
    0x3905,0x2B,
    0x3906,0x01,
    0x3907,0x12,
    0x3909,0x2B,
    0x390A,0x02,
    0x390B,0x12,
    0x3911,0x00,
};

static kal_uint16 imx766_custom6_setting[] = {
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x02,
    0x0342,0x15,
    0x0343,0xC0,
    0x0340,0x03,
    0x0341,0x80,
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x06,
    0x0347,0x60,
    0x0348,0x1F,
    0x0349,0xFF,
    0x034A,0x11,
    0x034B,0x9F,
    0x0900,0x01,
    0x0901,0x44,
    0x0902,0x08,
    0x3005,0x00,
    0x3120,0x00,
    0x3121,0x00,
    0x3200,0x43,
    0x3201,0x43,
    0x32D6,0x00,
    0x0408,0x01,
    0x0409,0x80,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x05,
    0x040D,0x00,
    0x040E,0x02,
    0x040F,0xD0,
    0x034C,0x05,
    0x034D,0x00,
    0x034E,0x02,
    0x034F,0xD0,
    0x0301,0x05,
    0x0303,0x02,
    0x0305,0x04,
    0x0306,0x00,
    0x0307,0xFA,
    0x030B,0x02,
    0x030D,0x03,
    0x030E,0x01,
    0x030F,0x33,
    0x30CB,0x00,
    0x30CC,0x10,
    0x30CD,0x00,
    0x30CE,0x03,
    0x30CF,0x00,
    0x319C,0x00,
    0x3800,0x00,
    0x3801,0x00,
    0x3802,0x04,
    0x3847,0x03,
    0x38B0,0x00,
    0x38B1,0x00,
    0x38B2,0x00,
    0x38B3,0x00,
    0x38C4,0x00,
    0x38C5,0x00,
    0x4C3A,0x02,
    0x4C3B,0xD2,
    0x4C68,0x04,
    0x4C69,0x7E,
    0x4CF8,0x07,
    0x4CF9,0x9E,
    0x4DB8,0x08,
    0x4DB9,0x98,
    0x0202,0x03,
    0x0203,0x50,
    0x0224,0x01,
    0x0225,0xF4,
    0x313A,0x01,
    0x313B,0xF4,
    0x3803,0x00,
    0x3804,0x17,
    0x3805,0xC0,
    0x0204,0x00,
    0x0205,0x00,
    0x020E,0x01,
    0x020F,0x00,
    0x0216,0x00,
    0x0217,0x00,
    0x0218,0x01,
    0x0219,0x00,
    0x313C,0x00,
    0x313D,0x00,
    0x313E,0x01,
    0x313F,0x00,
    0x0860,0x01,
    0x0861,0x2D,
    0x0862,0x01,
    0x0863,0x2D,
    0x30B4,0x00,
    0x3066,0x00,
    0x3067,0x30,
    0x3068,0x00,
    0x3069,0x30,
    0x33D0,0x00,
    0x33D1,0x00,
    0x33D4,0x01,
    0x33DC,0x0A,
    0x33DD,0x0A,
    0x33DE,0x0A,
    0x33DF,0x0A,
    0x3070,0x01,
    0x3077,0x01,
    0x3078,0x30,
    0x3079,0x01,
    0x307A,0x30,
    0x307B,0x01,
    0x3080,0x02,
    0x3087,0x02,
    0x3088,0x30,
    0x3089,0x02,
    0x308A,0x30,
    0x308B,0x02,
    0x3901,0x2B,
    0x3902,0x00,
    0x3903,0x12,
    0x3905,0x2B,
    0x3906,0x01,
    0x3907,0x12,
    0x3909,0x2B,
    0x390A,0x02,
    0x390B,0x12,
    0x3911,0x00,
};

static kal_uint16 imx766_custom7_setting[] = {
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x02,
    0x0342,0x2D,
    0x0343,0x20,
    0x0340,0x10,
    0x0341,0xFC,
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x06,
    0x0347,0x00,
    0x0348,0x1F,
    0x0349,0xFF,
    0x034A,0x11,
    0x034B,0xFF,
    0x0900,0x00,
    0x0901,0x11,
    0x0902,0x0A,
    0x3005,0x00,
    0x3120,0x00,
    0x3121,0x01,
    0x3200,0x00,
    0x3201,0x00,
    0x32D6,0x01,
    0x0408,0x08,
    0x0409,0x00,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x10,
    0x040D,0x00,
    0x040E,0x0C,
    0x040F,0x00,
    0x034C,0x10,
    0x034D,0x00,
    0x034E,0x0C,
    0x034F,0x00,
    0x0301,0x05,
    0x0303,0x04,
    0x0305,0x04,
    0x0306,0x01,
    0x0307,0x3A,
    0x030B,0x02,
    0x030D,0x03,
    0x030E,0x01,
    0x030F,0x23,
    0x30CB,0x00,
    0x30CC,0x10,
    0x30CD,0x00,
    0x30CE,0x03,
    0x30CF,0x00,
    0x319C,0x00,
    0x3800,0x00,
    0x3801,0x00,
    0x3802,0x04,
    0x3847,0x00,
    0x38B0,0x00,
    0x38B1,0x00,
    0x38B2,0x00,
    0x38B3,0x00,
    0x38C4,0x02,
    0x38C5,0x26,
    0x4C3A,0x02,
    0x4C3B,0xD2,
    0x4C68,0x04,
    0x4C69,0x7E,
    0x4CF8,0x07,
    0x4CF9,0x9E,
    0x4DB8,0x08,
    0x4DB9,0x98,
    0x0202,0x10,
    0x0203,0xCC,
    0x0224,0x01,
    0x0225,0xF4,
    0x313A,0x01,
    0x313B,0xF4,
    0x3803,0x01,
    0x3804,0x16,
    0x3805,0xB0,
    0x0204,0x00,
    0x0205,0x00,
    0x020E,0x01,
    0x020F,0x00,
    0x0216,0x00,
    0x0217,0x00,
    0x0218,0x01,
    0x0219,0x00,
    0x313C,0x00,
    0x313D,0x00,
    0x313E,0x01,
    0x313F,0x00,
    0x0860,0x01,
    0x0861,0x2D,
    0x0862,0x01,
    0x0863,0x2D,
    0x30B4,0x01,
    0x3066,0x00,
    0x3067,0x30,
    0x3068,0x00,
    0x3069,0x31,
    0x33D0,0x00,
    0x33D1,0x00,
    0x33D4,0x01,
    0x33DC,0x0A,
    0x33DD,0x0A,
    0x33DE,0x0A,
    0x33DF,0x0A,
    0x3070,0x01,
    0x3077,0x01,
    0x3078,0x30,
    0x3079,0x01,
    0x307A,0x30,
    0x307B,0x01,
    0x3080,0x02,
    0x3087,0x02,
    0x3088,0x30,
    0x3089,0x02,
    0x308A,0x30,
    0x308B,0x02,
    0x3901,0x2B,
    0x3902,0x00,
    0x3903,0x12,
    0x3905,0x2B,
    0x3906,0x01,
    0x3907,0x12,
    0x3909,0x2B,
    0x390A,0x02,
    0x390B,0x12,
    0x3911,0x00,
};

static void extend_frame_length(kal_uint32 extended_fl)
{
    imgsensor.frame_length = extended_fl;

    write_cmos_sensor_8(0x0104, 0x01);
    write_frame_len(imgsensor.frame_length);
    write_cmos_sensor_8(0x0104, 0x00);
    imgsensor.extend_frame_length_en = KAL_TRUE;

    LOG_INF("new frame len = %d", imgsensor.frame_length);
}

static void sensor_init(void)
{
    LOG_INF("E\n");
    imx766_table_write_cmos_sensor(imx766_init_setting,
        sizeof(imx766_init_setting)/sizeof(kal_uint16));
    /*enable temperature sensor, TEMP_SEN_CTL:*/
    write_cmos_sensor_8(0x0138, 0x01);
    set_mirror_flip(imgsensor.mirror);
    //chip version
    LOG_INF("chip ver: 0x%x\n", read_cmos_sensor_8(0x0018));
    // read fab code
    LOG_INF("fab code1: 0x%x code2: 0x%x spec code: 0x%x\n", read_cmos_sensor_8(0x3AF8), read_cmos_sensor_8(0x3AF9), read_cmos_sensor_8(0x3AFA));
    LOG_INF("X");
}/*      sensor_init  */

static void preview_setting(void)
{
    LOG_INF("E\n");
    imx766_table_write_cmos_sensor(imx766_preview_setting,
        sizeof(imx766_preview_setting)/sizeof(kal_uint16));
    LOG_INF("X");
} /* preview_setting */

/*full size 30fps*/
static void capture_setting(kal_uint16 currefps)
{
    LOG_INF("%s(PD 012515) 30 fps E! currefps:%d\n", __func__, currefps);
    /*************MIPI output setting************/
    imx766_table_write_cmos_sensor(imx766_capture_30_setting,
        sizeof(imx766_capture_30_setting)/sizeof(kal_uint16));
    LOG_INF("%s(PD 012515) 30 fpsX\n", __func__);
}

static void normal_video_setting(kal_uint16 currefps)
{
    LOG_INF("%s E! currefps:%d\n", __func__, currefps);
    imx766_table_write_cmos_sensor(imx766_normal_video_setting,
    sizeof(imx766_normal_video_setting)/sizeof(kal_uint16));
    LOG_INF("X\n");
}

static void hs_video_setting(void)
{
    LOG_INF("%s E! currefps 120\n", __func__);
    imx766_table_write_cmos_sensor(imx766_hs_video_setting,
    sizeof(imx766_hs_video_setting)/sizeof(kal_uint16));
    LOG_INF("X\n");
}

static void hdr_write_tri_shutter_w_gph(kal_uint16 le, kal_uint16 me, kal_uint16 se, kal_bool gph)
{
    kal_uint16 realtime_fps = 0;
    kal_uint16 exposure_cnt = 0;
    //kal_uint16 le_b = 0, se_b = 0;

    if (le)
        exposure_cnt++;
    if (me)
        exposure_cnt++;
    if (se) {
        exposure_cnt++;
    } else {
        se = imgsensor_info.min_shutter;
        exposure_cnt++;
    }

    le = (kal_uint16)max(imgsensor_info.min_shutter * exposure_cnt, (kal_uint32)le);
    se = (kal_uint16)max(imgsensor_info.min_shutter * exposure_cnt, (kal_uint32)se);
    if(le < se) {
        le = se;
    }
    //3exp 12 + 6, 2exp 8 + 6, 6 is fine integ time
    if (le) {
        le -= 2;   // subtract fine integ time 6
        le = round_up(le/exposure_cnt, 4);
    }

    if (me) {
        me -= 2;
        me = round_up(me/exposure_cnt, 4);
    }

    if (se) {
        se -= 2;
        se = round_up(se/exposure_cnt, 4);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length = max((kal_uint32)((le + me + se + (exposure_cnt * imgsensor_info.margin)) * exposure_cnt),
                                imgsensor.min_frame_length);
    imgsensor.frame_length = min(imgsensor.frame_length, imgsensor_info.max_frame_length);
    spin_unlock(&imgsensor_drv_lock);

    LOG_INF("E! le:0x%x, me:0x%x, se:0x%x autoflicker_en %d frame_length %d\n",
        le, me, se, imgsensor.autoflicker_en, imgsensor.frame_length);

    if (gph) {
        write_cmos_sensor_8(0x0104, 0x01);
    }

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
        else {
            write_frame_len(imgsensor.frame_length);
        }
    } else {
        write_frame_len(imgsensor.frame_length);
    }

    /* Long exposure */
    write_cmos_sensor_8(0x0202, (le >> 8) & 0xFF);
    write_cmos_sensor_8(0x0203, le & 0xFF);
    /* Muddle exposure */
    if (me != 0) {
        /*MID_COARSE_INTEG_TIME[15:8]*/
        write_cmos_sensor_8(0x313A, (me >> 8) & 0xFF);
        /*MID_COARSE_INTEG_TIME[7:0]*/
        write_cmos_sensor_8(0x313B, me & 0xFF);
    }
    /* Short exposure */
    write_cmos_sensor_8(0x0224, (se >> 8) & 0xFF);
    write_cmos_sensor_8(0x0225, se & 0xFF);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x00);
    }
    LOG_INF("L! le:0x%x, me:0x%x, se:0x%x\n", le, me, se);
#if 0
    /*read back*/
    le_b = ((read_cmos_sensor_8(0x0202) & 0xff) << 8) |  (read_cmos_sensor_8(0x0203) & 0xff);
    se_b = ((read_cmos_sensor_8(0x0224) & 0xff) << 8) |  (read_cmos_sensor_8(0x0225) & 0xff);
    LOG_INF("read from sensor le: 0x%x(%d), se: 0x%x(%d)", le_b, le_b, se_b, se_b);
#endif
}

static void hdr_write_tri_shutter(kal_uint16 le, kal_uint16 me, kal_uint16 se)
{
    LOG_INF("Fine Integ Time = %d", (read_cmos_sensor_8(0x0200) << 8) | read_cmos_sensor_8(0x0201));
    hdr_write_tri_shutter_w_gph(le, me, se, KAL_TRUE);
}

static void hdr_write_tri_gain_w_gph(kal_uint16 lgain, kal_uint16 mg, kal_uint16 sg, kal_bool gph)
{
    kal_uint16 reg_lg, reg_mg, reg_sg;
    //kal_uint16 lg_b = 0, sg_b = 0;

    if (lgain < BASEGAIN || lgain > 64 * BASEGAIN) {
        LOG_INF("Error lgain setting");
        if (lgain < BASEGAIN)
            lgain = BASEGAIN;
        else if (lgain > 64 * BASEGAIN)
            lgain = 64 * BASEGAIN;
    }

    if (sg < BASEGAIN || sg > 64 * BASEGAIN) {
        LOG_INF("Error sgain setting");
        if (sg < BASEGAIN)
            sg = BASEGAIN;
        else if (sg > 64 * BASEGAIN)
            sg = 64 * BASEGAIN;
    }

    reg_lg = gain2reg(lgain);
    if (mg != 0) {
        reg_mg = gain2reg(mg);
    }
    reg_sg = gain2reg(sg);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_lg;
    spin_unlock(&imgsensor_drv_lock);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x01);
    }
    /* Long Gian */
    write_cmos_sensor_8(0x0204, (reg_lg>>8) & 0xFF);
    write_cmos_sensor_8(0x0205, reg_lg & 0xFF);
    /* Middle Gian */
    if (mg != 0) {
        write_cmos_sensor_8(0x313C, (reg_mg>>8) & 0xFF);
        write_cmos_sensor_8(0x313D, reg_mg & 0xFF);
    }
    /* Short Gian */
    write_cmos_sensor_8(0x0216, (reg_sg>>8) & 0xFF);
    write_cmos_sensor_8(0x0217, reg_sg & 0xFF);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x00);
    }
    LOG_INF("lgain:0x%x, reg_lg:0x%x, mg:0x%x, reg_mg:0x%x, sg:0x%x, reg_sg:0x%x\n",
        lgain, reg_lg, mg, reg_mg, sg, reg_sg);
#if 0
    /*read back*/
    lg_b = ((read_cmos_sensor_8(0x0204) & 0xff) << 8) |  (read_cmos_sensor_8(0x0205) & 0xff);
    sg_b = ((read_cmos_sensor_8(0x0216) & 0xff) << 8) |  (read_cmos_sensor_8(0x0217) & 0xff);
    LOG_INF("read from sensor lg: 0x%x(%d), sg: 0x%x(%d)", lg_b, lg_b, sg_b, sg_b);
#endif
}
static void hdr_write_tri_gain(kal_uint16 lgain, kal_uint16 mg, kal_uint16 sg)
{
    hdr_write_tri_gain_w_gph(lgain, mg, sg, KAL_TRUE);
}

static void slim_video_setting(void)
{
    LOG_INF("%s E! currefps 30\n", __func__);
    imx766_table_write_cmos_sensor(imx766_slim_video_setting,
        sizeof(imx766_slim_video_setting)/sizeof(kal_uint16));
    LOG_INF("X\n");
}

/*4096x3072@24fps*/
static void custom1_setting(void)
{
    LOG_INF("%s 24 fps E! currefps\n", __func__);
    /*************MIPI output setting************/
    imx766_table_write_cmos_sensor(imx766_custom1_setting,
        sizeof(imx766_custom1_setting)/sizeof(kal_uint16));
    LOG_INF("X");
}

/*4096x3072@60fps*/
static void custom2_setting(void)
{
    LOG_INF("%s 30 fps E! currefps 4K 2DOL\n", __func__);
    /*************MIPI output setting************/
    imx766_table_write_cmos_sensor(imx766_custom2_setting,
        sizeof(imx766_custom2_setting)/sizeof(kal_uint16));
    LOG_INF("X");
}

/*QRMSC 8192x6144@24fps*/
static void custom3_setting(void)
{
    LOG_INF("%s 24 fps E! currefps\n", __func__);
    /*************MIPI output setting************/
    imx766_table_write_cmos_sensor(imx766_custom3_setting,
        sizeof(imx766_custom3_setting)/sizeof(kal_uint16));
    LOG_INF("X");
}

/*4K_2DOL_4096x2304@30fps*/
static void custom4_setting(void)
{
    LOG_INF("%s 60 fps E! currefps\n", __func__);
    /*************MIPI output setting************/
    imx766_table_write_cmos_sensor(imx766_custom4_setting,
        sizeof(imx766_custom4_setting)/sizeof(kal_uint16));
    LOG_INF("X");
}

/*2048x1152@240fps*/
static void custom5_setting(void)
{
    LOG_INF("%s 240 fps E! currefps\n", __func__);
    /*************MIPI output setting************/
    imx766_table_write_cmos_sensor(imx766_custom5_setting,
       sizeof(imx766_custom5_setting)/sizeof(kal_uint16));
    LOG_INF("X");
}

/*1280x720@480fps*/
static void custom6_setting(void)
{
    LOG_INF("%s 480 fps E! currefps\n", __func__);
    /*************MIPI output setting************/
    imx766_table_write_cmos_sensor(imx766_custom6_setting,
        sizeof(imx766_custom6_setting)/sizeof(kal_uint16));
    LOG_INF("X");
}

/*QRMSC_4096x3072@30fps*/
static void custom7_setting(void)
{
    LOG_INF("%s 30 fps E! currefps\n", __func__);
    /*************MIPI output setting************/
    imx766_table_write_cmos_sensor(imx766_custom7_setting,
        sizeof(imx766_custom7_setting)/sizeof(kal_uint16));
    LOG_INF("X");
}

/*************************************************************************
 * FUNCTION
 *    get_imgsensor_id
 *
 * DESCRIPTION
 *    This function get the sensor ID
 *
 * PARAMETERS
 *    *sensorID : return the sensor ID
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    static bool first_read = KAL_TRUE;
    /*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
     *we should detect the module used i2c address
     */

    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = ((read_cmos_sensor_8(0x0016) << 8) | read_cmos_sensor_8(0x0017));
            if (*sensor_id == IMX766_SENSOR_ID) {
                *sensor_id = imgsensor_info.sensor_id;
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
                    imgsensor.i2c_write_id, *sensor_id);
                if (first_read) {
                    LOG_INF("read module data when first poweron");
                    read_sensor_Cali();
                    read_module_data();
                    if(deviceInfo_register_value == 0x00){
                    Eeprom_DataInit(IMGSENSOR_SENSOR_IDX_MAIN, imgsensor_info.sensor_id);
                    deviceInfo_register_value = 0x01;
				}
                    first_read = KAL_FALSE;
                }
                return ERROR_NONE;
            }
            LOG_INF("Read sensor id fail, id: 0x%x,sensor_id = 0x%x,imgsensor_info.sensor_id = 0x%x\n",
                imgsensor.i2c_write_id,*sensor_id,imgsensor_info.sensor_id);
            retry--;
        } while (retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        /*if Sensor ID is not correct,
         *Must set *sensor_id to 0xFFFFFFFF
         */
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *    open
 *
 * DESCRIPTION
 *    This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint16 sensor_id = 0;

    /*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
     *we should detect the module used i2c address
     */
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = ((read_cmos_sensor_8(0x0016) << 8)
                            | read_cmos_sensor_8(0x0017));
            if (sensor_id == IMX766_SENSOR_ID) {
                sensor_id = imgsensor_info.sensor_id;
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
                    imgsensor.i2c_write_id, sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, id: 0x%x\n",
                imgsensor.i2c_write_id);
            retry--;
        } while (retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;

    if (!qsc_flag) {
        LOG_INF("write_sensor_QSC Start\n");
        write_sensor_QSC();
        LOG_INF("write_sensor_QSC End\n");
        qsc_flag = 1;
    }

    /* initail sequence write in  */
    sensor_init();

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
    imgsensor.ihdr_mode = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    imgsensor.extend_frame_length_en = KAL_FALSE;
    imgsensor.fast_mode_on = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
} /* open */

/*************************************************************************
 * FUNCTION
 *    close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
    LOG_INF("E\n");
    /*No Need to implement this function*/
    write_cmos_sensor_8(0x0100, 0x00);
    qsc_flag = 0;
    return ERROR_NONE;
} /* close */

/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *    This function start the sensor preview.
 *
 * PARAMETERS
 *    *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s E\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    preview_setting();

    return ERROR_NONE;
} /* preview */

/*************************************************************************
 * FUNCTION
 *    capture
 *
 * DESCRIPTION
 *    This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
        LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
            imgsensor.current_fps, imgsensor_info.cap.max_framerate / 10);
    imgsensor.pclk = imgsensor_info.cap.pclk;
    imgsensor.line_length = imgsensor_info.cap.linelength;
    imgsensor.frame_length = imgsensor_info.cap.framelength;
    imgsensor.min_frame_length = imgsensor_info.cap.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    capture_setting(imgsensor.current_fps);

    return ERROR_NONE;
}/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                               MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    normal_video_setting(imgsensor.current_fps);

    return ERROR_NONE;
}/*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                           MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();

    return ERROR_NONE;
}/*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                             MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    /*imgsensor.video_mode = KAL_TRUE;*/
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    /*imgsensor.current_fps = 300;*/
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();

    return ERROR_NONE;
}/* slim_video */


static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom1_setting();

    return ERROR_NONE;
}/* custom1 */

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom2_setting();

    return ERROR_NONE;
}/* custom2 */

static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom3_setting();

    return ERROR_NONE;
}/* custom3 */

static kal_uint32 custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    imgsensor.pclk = imgsensor_info.custom4.pclk;
    imgsensor.line_length = imgsensor_info.custom4.linelength;
    imgsensor.frame_length = imgsensor_info.custom4.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom4_setting();

    return ERROR_NONE;
}/* custom4 */

static kal_uint32 custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
    imgsensor.pclk = imgsensor_info.custom5.pclk;
    imgsensor.line_length = imgsensor_info.custom5.linelength;
    imgsensor.frame_length = imgsensor_info.custom5.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom5_setting();

    return ERROR_NONE;
}/* custom5 */

static kal_uint32 custom6(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM6;
    imgsensor.pclk = imgsensor_info.custom6.pclk;
    imgsensor.line_length = imgsensor_info.custom6.linelength;
    imgsensor.frame_length = imgsensor_info.custom6.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom6.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom6_setting();

    return ERROR_NONE;
}/* custom6 */

static kal_uint32 custom7(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM7;
    imgsensor.pclk = imgsensor_info.custom7.pclk;
    imgsensor.line_length = imgsensor_info.custom7.linelength;
    imgsensor.frame_length = imgsensor_info.custom7.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom7.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom7_setting();

    return ERROR_NONE;
}/* custom7 */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
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
} /* get_resolution */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_INFO_STRUCT *sensor_info,
                           MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
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

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
    sensor_info->AESensorGainDelayFrame =
        imgsensor_info.ae_sensor_gain_delay_frame;
    sensor_info->AEISPGainDelayFrame =
        imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
    sensor_info->PDAF_Support = 7;

    sensor_info->HDR_Support = HDR_SUPPORT_STAGGER_FDOL;

    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0; /* 0 is default 1x */
    sensor_info->SensorHightSampling = 0; /* 0 is default 1x */
    sensor_info->SensorPacketECCOrder = 1;
    sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;
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
        sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM2:
        sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM3:
        sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM4:
        sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom4.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM5:
        sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom5.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM6:
        sensor_info->SensorGrabStartX = imgsensor_info.custom6.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom6.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom6.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        sensor_info->SensorGrabStartX = imgsensor_info.custom7.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom7.starty;
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
}/*    get_info  */

#define PHASE_PIX_OUT_EN 0x30B4
#define FRAME_LEN_UPPER  0x0340
#define FRAME_LEN_LOWER  0x0341
#define CIT_LE_UPPER     0x0202
#define CIT_LE_LOWER     0x0203
#define CIT_ME_UPPER     0x0224
#define CIT_ME_LOWER     0x0225
#define CIT_SE_UPPER     0x313A
#define CIT_SE_LOWER     0x313B
#define DOL_EN           0x33D0
#define DOL_MODE         0x33D1
#define VSB_2ND_VCID     0x3070
#define VSB_3ND_VCID     0x3080

#define PIX_1_1ST_VCID   0x3066
#define PIX_2_1ST_VCID   0x3068
#define PIX_1_2ND_VCID   0x3077
#define PIX_2_2ND_VCID   0x3079
#define PIX_1_3RD_VCID   0x3087
#define PIX_2_3RD_VCID   0x3089

#define ZOOM2X_AGAIN_150_THRESHOLD 150
#define ZOOM2X_AGAIN_350_THRESHOLD 350
#define ZOOM2X_AGAIN_700_THRESHOLD 700
#define ZOOM2X_AGAIN_1300_THRESHOLD 1300

#define FMC_GPH_START \
do { \
        write_cmos_sensor_8(0x0104, 0x01); \
        write_cmos_sensor_8(0x3010, 0x02); \
} while (0)

#define FMC_GPH_END \
do { \
        write_cmos_sensor_8(0x0104, 0x00); \
} while (0)

enum {
    SHUTTER_NE_FRM_1 = 0,
    GAIN_NE_FRM_1,
    FRAME_LEN_NE_FRM_1,
    HDR_TYPE_FRM_1,
    SHUTTER_NE_FRM_2,
    GAIN_NE_FRM_2,
    FRAME_LEN_NE_FRM_2,
    HDR_TYPE_FRM_2,
    SHUTTER_SE_FRM_1,
    GAIN_SE_FRM_1,
    SHUTTER_SE_FRM_2,
    GAIN_SE_FRM_2,
    SHUTTER_ME_FRM_1,
    GAIN_ME_FRM_1,
    SHUTTER_ME_FRM_2,
    GAIN_ME_FRM_2,
};

static kal_uint32 seamless_switch(enum MSDK_SCENARIO_ID_ENUM scenario_id, uint32_t *ae_ctrl)
{
    imgsensor.extend_frame_length_en = KAL_FALSE;
    LOG_INF("scenario_id: %d SHUTTER_NE_FRM_1: %d GAIN_NE_FRM_1: %d SHUTTER_ME_FRM_1: %d GAIN_ME_FRM_1: %d SHUTTER_SE_FRM_1: %d GAIN_SE_FRM_1: %d",
            scenario_id, ae_ctrl[SHUTTER_NE_FRM_1], ae_ctrl[GAIN_NE_FRM_1], ae_ctrl[SHUTTER_ME_FRM_1], ae_ctrl[GAIN_ME_FRM_1], ae_ctrl[SHUTTER_SE_FRM_1], ae_ctrl[GAIN_SE_FRM_1]);

    switch (scenario_id) {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
        imgsensor.pclk = imgsensor_info.pre.pclk;
        imgsensor.line_length = imgsensor_info.pre.linelength;
        imgsensor.frame_length = imgsensor_info.pre.framelength;
        imgsensor.min_frame_length = imgsensor_info.pre.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
        spin_unlock(&imgsensor_drv_lock);

        pr_info("seamless switch to preview!\n");
        FMC_GPH_START;
        imx766_table_write_cmos_sensor(imx766_21305_seamless_preview, sizeof(imx766_21305_seamless_preview) / sizeof(kal_uint16));
        if (ae_ctrl) {
            LOG_INF("call MSDK_SCENARIO_ID_CAMERA_PREVIEW %d %d",
                    ae_ctrl[SHUTTER_NE_FRM_1] ,ae_ctrl[GAIN_NE_FRM_1]);
            set_shutter_w_gph(ae_ctrl[SHUTTER_NE_FRM_1], KAL_FALSE);
            set_gain_w_gph(ae_ctrl[GAIN_NE_FRM_1], KAL_FALSE);
        }
        FMC_GPH_END;
    }
    break;
    case MSDK_SCENARIO_ID_CUSTOM7:
    {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM7;
        imgsensor.pclk = imgsensor_info.custom7.pclk;
        imgsensor.line_length = imgsensor_info.custom7.linelength;
        imgsensor.frame_length = imgsensor_info.custom7.framelength;
        imgsensor.min_frame_length = imgsensor_info.custom7.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
        spin_unlock(&imgsensor_drv_lock);

        pr_info("seamless switch Full remosaic!-RegV4-3\n");
        FMC_GPH_START;
        if (ae_ctrl[GAIN_NE_FRM_1] >= ZOOM2X_AGAIN_1300_THRESHOLD) {
            LOG_INF("optimize capture quality with 24x again setting RegV4-3");
            imx766_table_write_cmos_sensor(imx766_21305_seamless_custom7_24x_again, sizeof(imx766_21305_seamless_custom7_24x_again) / sizeof(kal_uint16));
         } else if (ae_ctrl[GAIN_NE_FRM_1] >= ZOOM2X_AGAIN_700_THRESHOLD) {
                LOG_INF("optimize capture quality 16x again setting RegV4-3");
                imx766_table_write_cmos_sensor(imx766_21305_seamless_custom7_16x_again, sizeof(imx766_21305_seamless_custom7_16x_again) / sizeof(kal_uint16));
         } else if (ae_ctrl[GAIN_NE_FRM_1] >= ZOOM2X_AGAIN_350_THRESHOLD) {
                LOG_INF("optimize capture quality 8x again setting RegV4-3");
                imx766_table_write_cmos_sensor(imx766_21305_seamless_custom7_8x_again, sizeof(imx766_21305_seamless_custom7_8x_again) / sizeof(kal_uint16));
         } else if (ae_ctrl[GAIN_NE_FRM_1] >= ZOOM2X_AGAIN_150_THRESHOLD) {
                LOG_INF("optimize capture quality with 4x setting RegV4-3");
                imx766_table_write_cmos_sensor(imx766_21305_seamless_custom7_4x_again, sizeof(imx766_21305_seamless_custom7_4x_again) / sizeof(kal_uint16));
            } else {
                LOG_INF("optimize capture quality with 1x setting RegV4-3");
                imx766_table_write_cmos_sensor(imx766_21305_seamless_custom7_1x_again, sizeof(imx766_21305_seamless_custom7_1x_again) / sizeof(kal_uint16));
        }
        if (ae_ctrl) {
            LOG_INF("call MSDK_SCENARIO_ID_CUSTOM7 %d %d",
                    ae_ctrl[SHUTTER_NE_FRM_1] ,ae_ctrl[GAIN_NE_FRM_1]);
            set_shutter_w_gph(ae_ctrl[SHUTTER_NE_FRM_1], KAL_FALSE);
            set_gain_w_gph(ae_ctrl[GAIN_NE_FRM_1], KAL_FALSE);
        }
        FMC_GPH_END;
    }
    break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    {
        spin_lock(&imgsensor_drv_lock);
        //imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
        imgsensor.autoflicker_en = KAL_FALSE;
        imgsensor.pclk = imgsensor_info.normal_video.pclk;
        imgsensor.line_length = imgsensor_info.normal_video.linelength;
        imgsensor.frame_length = imgsensor_info.normal_video.framelength;
        imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
        spin_unlock(&imgsensor_drv_lock);

        pr_info("seamless switch normal video!\n");
        FMC_GPH_START;
        imx766_table_write_cmos_sensor(imx766_21305_seamless_normal_video, sizeof(imx766_21305_seamless_normal_video) / sizeof(kal_uint16));
        if (ae_ctrl) {
            LOG_INF("call MSDK_SCENARIO_ID_VIDEO_PREVIEW %d %d",
                    ae_ctrl[SHUTTER_NE_FRM_1] ,ae_ctrl[GAIN_NE_FRM_1]);
            set_shutter_w_gph(ae_ctrl[SHUTTER_NE_FRM_1], KAL_FALSE);
            set_gain_w_gph(ae_ctrl[GAIN_NE_FRM_1], KAL_FALSE);
        }
        FMC_GPH_END;
    }
    break;
    case MSDK_SCENARIO_ID_CUSTOM4:
    {
        spin_lock(&imgsensor_drv_lock);
        //imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
        imgsensor.autoflicker_en = KAL_FALSE;
        imgsensor.pclk = imgsensor_info.custom4.pclk;
        imgsensor.line_length = imgsensor_info.custom4.linelength;
        imgsensor.frame_length = imgsensor_info.custom4.framelength;
        imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
        spin_unlock(&imgsensor_drv_lock);

        pr_info("seamless switch dolhdr 2exp video!\n");
        FMC_GPH_START;
        imx766_table_write_cmos_sensor(imx766_21305_seamless_custom4, sizeof(imx766_21305_seamless_custom4) / sizeof(kal_uint16));
        if (ae_ctrl) {
            LOG_INF("call MSDK_SCENARIO_ID_CUSTOM4 %d %d %d %d %d %d",
                    ae_ctrl[SHUTTER_NE_FRM_1], 0, ae_ctrl[SHUTTER_SE_FRM_1],
                    ae_ctrl[GAIN_NE_FRM_1], 0, ae_ctrl[GAIN_SE_FRM_1]);
            hdr_write_tri_shutter_w_gph(ae_ctrl[SHUTTER_NE_FRM_1], 0, ae_ctrl[SHUTTER_SE_FRM_1], KAL_FALSE);
            hdr_write_tri_gain_w_gph(ae_ctrl[GAIN_NE_FRM_1], 0, ae_ctrl[GAIN_SE_FRM_1], KAL_FALSE);
        }
        FMC_GPH_END;
    }
    break;
    default:
    {
        pr_info( "error! wrong setting in set_seamless_switch = %d",scenario_id);
        return 0xff;
    }
    }

    imgsensor.fast_mode_on = KAL_TRUE;
    LOG_INF("%s success, scenario is switched to %d", __func__, scenario_id);
    return 0;
}

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);
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
        custom1(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM2:
        custom2(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM3:
        custom3(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM4:
        custom4(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM5:
        custom5(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM6:
        custom6(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        custom7(image_window, sensor_config_data);
        break;
    default:
        LOG_INF("Error ScenarioId setting");
        preview(image_window, sensor_config_data);
        return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}/* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
    LOG_INF("framerate = %d\n ", framerate);
    /* SetVideoMode Function should fix framerate */
    if (framerate == 0)
        /* Dynamic frame rate */
        return ERROR_NONE;
    spin_lock(&imgsensor_drv_lock);
    if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 296;
    else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 146;
    else
        imgsensor.current_fps = framerate;
    spin_unlock(&imgsensor_drv_lock);
    write_cmos_sensor_8(0x0104, 0x01);
    set_max_framerate(imgsensor.current_fps, 1);
    write_cmos_sensor_8(0x0104, 0x00);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
    spin_lock(&imgsensor_drv_lock);
    if (enable) /*enable auto flicker*/
        imgsensor.autoflicker_en = KAL_TRUE;
    else /*Cancel Auto flick*/
        imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);
    switch (scenario_id) {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength)
                                ? (frame_length - imgsensor_info.pre.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        if (framerate == 0)
            return ERROR_NONE;
        frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength)
                                ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
                framerate, imgsensor_info.cap.max_framerate/10);
            frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
        if (frame_length > imgsensor_info.max_frame_length) {
            LOG_INF("Warning: frame_length %d > max_frame_length %d!\n",
                frame_length, imgsensor_info.max_frame_length);
            break;
        }
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength)
                                ? (frame_length - imgsensor_info.cap.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength)
                                ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_SLIM_VIDEO:
        frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength)
                                ? (frame_length - imgsensor_info.slim_video.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM1:
        frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength)
                                ? (frame_length - imgsensor_info.custom1.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM2:
        frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength)
                                ? (frame_length - imgsensor_info.custom2.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM3:
        frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength)
                                ? (frame_length - imgsensor_info.custom3.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM4:
        /*frame_length = imgsensor_info.custom4.pclk / framerate * 10 / imgsensor_info.custom4.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength)
                                ? (frame_length - imgsensor_info.custom4.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }*/
        break;
    case MSDK_SCENARIO_ID_CUSTOM5:
        frame_length = imgsensor_info.custom5.pclk / framerate * 10 / imgsensor_info.custom5.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.custom5.framelength)
                                ? (frame_length - imgsensor_info.custom5.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.custom5.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM6:
        frame_length = imgsensor_info.custom6.pclk / framerate * 10 / imgsensor_info.custom6.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.custom6.framelength)
                                ? (frame_length - imgsensor_info.custom6.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.custom6.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        frame_length = imgsensor_info.custom7.pclk / framerate * 10 / imgsensor_info.custom7.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.custom7.framelength)
                                ? (frame_length - imgsensor_info.custom7.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.custom7.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    default:  /*coding with  preview scenario by default*/
        frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength)
                                ? (frame_length - imgsensor_info.pre.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
        break;
    }

    return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
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

static kal_uint32 set_test_pattern_mode(kal_uint8 modes, struct SET_SENSOR_PATTERN_SOLID_COLOR *pTestpatterndata)
{
    pr_debug("set_test_pattern enum: %d\n", modes);

    if (modes) {
        write_cmos_sensor_8(0x020E, 0x00);
        write_cmos_sensor_8(0x0218, 0x00);
        write_cmos_sensor_8(0x3015, 0x00);
    } else {
        write_cmos_sensor_8(0x020E, 0x01);
        write_cmos_sensor_8(0x0218, 0x01);
        write_cmos_sensor_8(0x3015, 0x40);
    }

    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = modes;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

#if 0
static kal_uint32 imx766_ana_gain_table[] = {
    100000,
    100098,
    100196,
    100294,
    100392,
    100491,
    100589,
    100688,
    100787,
    100887,
    100986,
    101086,
    101186,
    101286,
    101386,
    101487,
    101587,
    101688,
    101789,
    101891,
    101992,
    102094,
    102196,
    102298,
    102400,
    102503,
    102605,
    102708,
    102811,
    102915,
    103018,
    103122,
    103226,
    103330,
    103434,
    103539,
    103644,
    103749,
    103854,
    103959,
    104065,
    104171,
    104277,
    104383,
    104490,
    104597,
    104703,
    104811,
    104918,
    105026,
    105133,
    105242,
    105350,
    105458,
    105567,
    105676,
    105785,
    105895,
    106004,
    106114,
    106224,
    106334,
    106445,
    106556,
    106667,
    106778,
    106889,
    107001,
    107113,
    107225,
    107338,
    107450,
    107563,
    107676,
    107789,
    107903,
    108017,
    108131,
    108245,
    108360,
    108475,
    108590,
    108705,
    108820,
    108936,
    109052,
    109168,
    109285,
    109402,
    109519,
    109636,
    109753,
    109871,
    109989,
    110108,
    110226,
    110345,
    110464,
    110583,
    110703,
    110823,
    110943,
    111063,
    111183,
    111304,
    111425,
    111547,
    111668,
    111790,
    111913,
    112035,
    112158,
    112281,
    112404,
    112527,
    112651,
    112775,
    112900,
    113024,
    113149,
    113274,
    113400,
    113525,
    113651,
    113778,
    113904,
    114031,
    114158,
    114286,
    114413,
    114541,
    114670,
    114798,
    114927,
    115056,
    115186,
    115315,
    115445,
    115576,
    115706,
    115837,
    115968,
    116100,
    116232,
    116364,
    116496,
    116629,
    116762,
    116895,
    117029,
    117162,
    117297,
    117431,
    117566,
    117701,
    117837,
    117972,
    118108,
    118245,
    118382,
    118519,
    118656,
    118794,
    118931,
    119070,
    119208,
    119347,
    119487,
    119626,
    119766,
    119906,
    120047,
    120188,
    120329,
    120471,
    120612,
    120755,
    120897,
    121040,
    121183,
    121327,
    121471,
    121615,
    121760,
    121905,
    122050,
    122196,
    122342,
    122488,
    122635,
    122782,
    122929,
    123077,
    123225,
    123373,
    123522,
    123671,
    123821,
    123971,
    124121,
    124272,
    124423,
    124574,
    124726,
    124878,
    125031,
    125183,
    125337,
    125490,
    125644,
    125799,
    125953,
    126108,
    126264,
    126420,
    126576,
    126733,
    126890,
    127047,
    127205,
    127363,
    127522,
    127681,
    127840,
    128000,
    128160,
    128321,
    128482,
    128643,
    128805,
    128967,
    129130,
    129293,
    129456,
    129620,
    129785,
    129949,
    130114,
    130280,
    130446,
    130612,
    130779,
    130946,
    131114,
    131282,
    131451,
    131620,
    131789,
    131959,
    132129,
    132300,
    132471,
    132642,
    132815,
    132987,
    133160,
    133333,
    133507,
    133681,
    133856,
    134031,
    134207,
    134383,
    134560,
    134737,
    134914,
    135092,
    135271,
    135450,
    135629,
    135809,
    135989,
    136170,
    136352,
    136533,
    136716,
    136898,
    137082,
    137265,
    137450,
    137634,
    137820,
    138005,
    138192,
    138378,
    138566,
    138753,
    138942,
    139130,
    139320,
    139510,
    139700,
    139891,
    140082,
    140274,
    140466,
    140659,
    140853,
    141047,
    141241,
    141436,
    141632,
    141828,
    142025,
    142222,
    142420,
    142618,
    142817,
    143017,
    143217,
    143417,
    143619,
    143820,
    144023,
    144225,
    144429,
    144633,
    144837,
    145042,
    145248,
    145455,
    145661,
    145869,
    146077,
    146286,
    146495,
    146705,
    146915,
    147126,
    147338,
    147550,
    147763,
    147977,
    148191,
    148406,
    148621,
    148837,
    149054,
    149271,
    149489,
    149708,
    149927,
    150147,
    150367,
    150588,
    150810,
    151032,
    151256,
    151479,
    151704,
    151929,
    152155,
    152381,
    152608,
    152836,
    153064,
    153293,
    153523,
    153754,
    153985,
    154217,
    154449,
    154683,
    154917,
    155152,
    155387,
    155623,
    155860,
    156098,
    156336,
    156575,
    156815,
    157055,
    157296,
    157538,
    157781,
    158025,
    158269,
    158514,
    158760,
    159006,
    159253,
    159502,
    159750,
    160000,
    160250,
    160502,
    160754,
    161006,
    161260,
    161514,
    161769,
    162025,
    162282,
    162540,
    162798,
    163057,
    163317,
    163578,
    163840,
    164103,
    164366,
    164630,
    164895,
    165161,
    165428,
    165696,
    165964,
    166234,
    166504,
    166775,
    167047,
    167320,
    167594,
    167869,
    168144,
    168421,
    168699,
    168977,
    169256,
    169536,
    169818,
    170100,
    170383,
    170667,
    170952,
    171237,
    171524,
    171812,
    172101,
    172391,
    172681,
    172973,
    173266,
    173559,
    173854,
    174150,
    174446,
    174744,
    175043,
    175342,
    175643,
    175945,
    176248,
    176552,
    176857,
    177163,
    177470,
    177778,
    178087,
    178397,
    178709,
    179021,
    179335,
    179649,
    179965,
    180282,
    180600,
    180919,
    181239,
    181560,
    181883,
    182206,
    182531,
    182857,
    183184,
    183513,
    183842,
    184173,
    184505,
    184838,
    185172,
    185507,
    185844,
    186182,
    186521,
    186861,
    187203,
    187546,
    187890,
    188235,
    188582,
    188930,
    189279,
    189630,
    189981,
    190335,
    190689,
    191045,
    191402,
    191760,
    192120,
    192481,
    192844,
    193208,
    193573,
    193939,
    194307,
    194677,
    195048,
    195420,
    195793,
    196169,
    196545,
    196923,
    197303,
    197683,
    198066,
    198450,
    198835,
    199222,
    199610,
    200000,
    200391,
    200784,
    201179,
    201575,
    201972,
    202372,
    202772,
    203175,
    203579,
    203984,
    204391,
    204800,
    205210,
    205622,
    206036,
    206452,
    206869,
    207287,
    207708,
    208130,
    208554,
    208980,
    209407,
    209836,
    210267,
    210700,
    211134,
    211570,
    212008,
    212448,
    212890,
    213333,
    213779,
    214226,
    214675,
    215126,
    215579,
    216034,
    216490,
    216949,
    217410,
    217872,
    218337,
    218803,
    219272,
    219742,
    220215,
    220690,
    221166,
    221645,
    222126,
    222609,
    223094,
    223581,
    224070,
    224561,
    225055,
    225551,
    226049,
    226549,
    227051,
    227556,
    228062,
    228571,
    229083,
    229596,
    230112,
    230631,
    231151,
    231674,
    232200,
    232727,
    233257,
    233790,
    234325,
    234862,
    235402,
    235945,
    236490,
    237037,
    237587,
    238140,
    238695,
    239252,
    239813,
    240376,
    240941,
    241509,
    242080,
    242654,
    243230,
    243810,
    244391,
    244976,
    245564,
    246154,
    246747,
    247343,
    247942,
    248544,
    249148,
    249756,
    250367,
    250980,
    251597,
    252217,
    252840,
    253465,
    254094,
    254726,
    255362,
    256000,
    256642,
    257286,
    257935,
    258586,
    259241,
    259898,
    260560,
    261224,
    261893,
    262564,
    263239,
    263918,
    264599,
    265285,
    265974,
    266667,
    267363,
    268063,
    268766,
    269474,
    270185,
    270899,
    271618,
    272340,
    273067,
    273797,
    274531,
    275269,
    276011,
    276757,
    277507,
    278261,
    279019,
    279781,
    280548,
    281319,
    282094,
    282873,
    283657,
    284444,
    285237,
    286034,
    286835,
    287640,
    288451,
    289266,
    290085,
    290909,
    291738,
    292571,
    293410,
    294253,
    295101,
    295954,
    296812,
    297674,
    298542,
    299415,
    300293,
    301176,
    302065,
    302959,
    303858,
    304762,
    305672,
    306587,
    307508,
    308434,
    309366,
    310303,
    311246,
    312195,
    313150,
    314110,
    315077,
    316049,
    317028,
    318012,
    319003,
    320000,
    321003,
    322013,
    323028,
    324051,
    325079,
    326115,
    327157,
    328205,
    329260,
    330323,
    331392,
    332468,
    333550,
    334641,
    335738,
    336842,
    337954,
    339073,
    340199,
    341333,
    342475,
    343624,
    344781,
    345946,
    347119,
    348299,
    349488,
    350685,
    351890,
    353103,
    354325,
    355556,
    356794,
    358042,
    359298,
    360563,
    361837,
    363121,
    364413,
    365714,
    367025,
    368345,
    369675,
    371014,
    372364,
    373723,
    375092,
    376471,
    377860,
    379259,
    380669,
    382090,
    383521,
    384962,
    386415,
    387879,
    389354,
    390840,
    392337,
    393846,
    395367,
    396899,
    398444,
    400000,
    401569,
    403150,
    404743,
    406349,
    407968,
    409600,
    411245,
    412903,
    414575,
    416260,
    417959,
    419672,
    421399,
    423140,
    424896,
    426667,
    428452,
    430252,
    432068,
    433898,
    435745,
    437607,
    439485,
    441379,
    443290,
    445217,
    447162,
    449123,
    451101,
    453097,
    455111,
    457143,
    459193,
    461261,
    463348,
    465455,
    467580,
    469725,
    471889,
    474074,
    476279,
    478505,
    480751,
    483019,
    485308,
    487619,
    489952,
    492308,
    494686,
    497087,
    499512,
    501961,
    504433,
    506931,
    509453,
    512000,
    514573,
    517172,
    519797,
    522449,
    525128,
    527835,
    530570,
    533333,
    536126,
    538947,
    541799,
    544681,
    547594,
    550538,
    553514,
    556522,
    559563,
    562637,
    565746,
    568889,
    572067,
    575281,
    578531,
    581818,
    585143,
    588506,
    591908,
    595349,
    598830,
    602353,
    605917,
    609524,
    613174,
    616867,
    620606,
    624390,
    628221,
    632099,
    636025,
    640000,
    644025,
    648101,
    652229,
    656410,
    660645,
    664935,
    669281,
    673684,
    678146,
    682667,
    687248,
    691892,
    696599,
    701370,
    706207,
    711111,
    716084,
    721127,
    726241,
    731429,
    736691,
    742029,
    747445,
    752941,
    758519,
    764179,
    769925,
    775758,
    781679,
    787692,
    793798,
    800000,
    806299,
    812698,
    819200,
    825806,
    832520,
    839344,
    846281,
    853333,
    860504,
    867797,
    875214,
    882759,
    890435,
    898246,
    906195,
    914286,
    922523,
    930909,
    939450,
    948148,
    957009,
    966038,
    975238,
    984615,
    994175,
    1003922,
    1013861,
    1024000,
    1034343,
    1044898,
    1055670,
    1066667,
    1077895,
    1089362,
    1101075,
    1113043,
    1125275,
    1137778,
    1150562,
    1163636,
    1177011,
    1190698,
    1204706,
    1219048,
    1233735,
    1248780,
    1264198,
    1280000,
    1296203,
    1312821,
    1329870,
    1347368,
    1365333,
    1383784,
    1402740,
    1422222,
    1442254,
    1462857,
    1484058,
    1505882,
    1528358,
    1551515,
    1575385,
    1600000,
};
#endif
static kal_int32 get_sensor_temperature(void)
{
    UINT8 temperature = 0;
    INT32 temperature_convert = 0;

    temperature = read_cmos_sensor_8(0x013a);
    if (temperature >= 0x0 && temperature <= 0x54)
        temperature_convert = temperature;
    else if (temperature >= 0x55 && temperature <= 0x7F)
        temperature_convert = 85;
    else if (temperature >= 0x80 && temperature <= 0xEC)
        temperature_convert = -20;
    else
        temperature_convert = (INT8)temperature;

    LOG_INF("temperature_convert(%d), read_reg(0x%x)\n", temperature_convert, temperature);

    return temperature_convert;
}
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                                  UINT8 *feature_para, UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
    UINT16 *feature_data_16 = (UINT16 *) feature_para;
    UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
    UINT32 *feature_data_32 = (UINT32 *) feature_para;
    unsigned long long *feature_data = (unsigned long long *) feature_para;
    /* unsigned long long *feature_return_para
     *  = (unsigned long long *) feature_para;
     */
    uint32_t *pAeCtrls;
    uint32_t *pScenarios;
    UINT32  fpsIMX;
    UINT32 TimeIMX;
    struct SET_PD_BLOCK_INFO_T *PDAFinfo;
    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    //struct SENSOR_VC_INFO_STRUCT *pvcinfo;
    /* SET_SENSOR_AWB_GAIN *pSetSensorAWB
     *  = (SET_SENSOR_AWB_GAIN *)feature_para;
     */
    struct SENSOR_VC_INFO2_STRUCT *pvcinfo2;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
        = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    /*LOG_INF("feature_id = %d\n", feature_id);*/
    switch (feature_id) {
    case SENSOR_FEATURE_SET_SENSOR_OTP:
        {
            enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
            pr_debug("SENSOR_FEATURE_SET_SENSOR_OTP\n");
            ret = Eeprom_CallWriteService((ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(feature_para));
            if (ret == IMGSENSOR_RETURN_SUCCESS)
                return ERROR_NONE;
            else
                return ERROR_MSDK_IS_ACTIVATED;
        }
    case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM7:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
            break;
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
            break;
        }
        break;
#if 0
    case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
        if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
            *(feature_data + 0) = sizeof(imx766_ana_gain_table);
        } else {
            memcpy((void *)(uintptr_t) (*(feature_data + 1)),
                (void *)imx766_ana_gain_table, sizeof(imx766_ana_gain_table));
        }
        break;
#endif
    case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM7: {
            *(feature_data + 1) = imgsensor_info.min_gain;
            *(feature_data + 2) = 1024;
        }
        break;
        default: {
            *(feature_data + 1) = imgsensor_info.min_gain;
            *(feature_data + 2) = imgsensor_info.max_gain;
        }
        break;
        }
        break;
    case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
        *(feature_data + 0) = imgsensor_info.min_gain_iso;
        *(feature_data + 1) = imgsensor_info.gain_step;
        *(feature_data + 2) = imgsensor_info.gain_type;
        break;
    case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CUSTOM2:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:{
            *(feature_data + 1) = 8;
            *(feature_data + 2) = 2;
            }
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        case MSDK_SCENARIO_ID_CUSTOM5:
        case MSDK_SCENARIO_ID_CUSTOM6:{
            *(feature_data + 1) = 8;
            *(feature_data + 2) = 4;
            }
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:{
            *(feature_data + 1) = 16;
            *(feature_data + 2) = 8;
            }
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM7:
        default: {
            *(feature_data + 1) = 9;
            *(feature_data + 2) = 1;
            }
            break;
        }
        break;
    case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
        *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = -1478000;
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
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.slim_video.pclk;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.pre.pclk;
            break;
        }
        break;
    case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO: {
        MUINT32 ratio = 1;

        if (*(feature_data + 2) & SENSOR_GET_LINELENGTH_FOR_READOUT) {
            ratio = get_exp_cnt_by_scenario((*feature_data));
        }

        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.cap.framelength << 16)
                + (ratio * imgsensor_info.cap.linelength);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.normal_video.framelength << 16)
                + (ratio * imgsensor_info.normal_video.linelength);
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.hs_video.framelength << 16)
                + (ratio * imgsensor_info.hs_video.linelength);
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.slim_video.framelength << 16)
                + (ratio * imgsensor_info.slim_video.linelength);
            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.custom1.framelength << 16)
                + (ratio * imgsensor_info.custom1.linelength);
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.custom2.framelength << 16)
                + (ratio * imgsensor_info.custom2.linelength);
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.custom3.framelength << 16)
                + (ratio * imgsensor_info.custom3.linelength);
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.custom4.framelength << 16)
                + (ratio * imgsensor_info.custom4.linelength);
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.custom5.framelength << 16)
                + (ratio * imgsensor_info.custom5.linelength);
            break;
        case MSDK_SCENARIO_ID_CUSTOM6:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.custom6.framelength << 16)
                + (ratio * imgsensor_info.custom6.linelength);
            break;
        case MSDK_SCENARIO_ID_CUSTOM7:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.custom7.framelength << 16)
                + (ratio * imgsensor_info.custom7.linelength);
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.pre.framelength << 16)
                + (ratio * imgsensor_info.pre.linelength);
            break;
        }
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
        /* night_mode((BOOL) *feature_data); */
        break;
    case SENSOR_FEATURE_SET_GAIN:
        set_gain((UINT16) *feature_data);
        break;
    case SENSOR_FEATURE_SET_FLASHLIGHT:
        break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
        break;
    case SENSOR_FEATURE_SET_REGISTER:
        write_cmos_sensor_8(sensor_reg_data->RegAddr,
                            sensor_reg_data->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        sensor_reg_data->RegData = read_cmos_sensor_8(sensor_reg_data->RegAddr);
        break;
    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
        /*get the lens driver ID from EEPROM
         * or just return LENS_DRIVER_ID_DO_NOT_CARE
         * if EEPROM does not exist in camera module.
         */
        *feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_SET_VIDEO_MODE:
        set_video_mode(*feature_data);
        break;
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
        get_imgsensor_id(feature_return_para_32);
        break;
    case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
        set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16+1));
        break;
    case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
        set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data,
                                     *(feature_data+1));
        break;
    case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
        get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
                                         (MUINT32 *)(uintptr_t)(*(feature_data+1)));
        break;
    case SENSOR_FEATURE_GET_PDAF_DATA:
        LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
        #if 0
        read_3P8_eeprom((kal_uint16)(*feature_data),
                       (char *)(uintptr_t)(*(feature_data+1)),
                       (kal_uint32)(*(feature_data+2)));
        #endif
        break;
    case SENSOR_FEATURE_SET_TEST_PATTERN:
        set_test_pattern_mode((UINT8)*feature_data, (struct SET_SENSOR_PATTERN_SOLID_COLOR *)(feature_data+1));
        break;
    case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
        /* for factory mode auto testing */
        *feature_return_para_32 = imgsensor_info.checksum_value;
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_SET_FRAMERATE:
        LOG_INF("current fps :%d\n", (UINT32)*feature_data_32);
        spin_lock(&imgsensor_drv_lock);
        imgsensor.current_fps = *feature_data_32;
        spin_unlock(&imgsensor_drv_lock);
        break;
    case SENSOR_FEATURE_SET_HDR:
        LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data_32);
        spin_lock(&imgsensor_drv_lock);
        imgsensor.ihdr_mode = *feature_data_32;
        spin_unlock(&imgsensor_drv_lock);
        break;
    case SENSOR_FEATURE_GET_CROP_INFO:
        LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
        wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
        switch (*feature_data_32) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[5],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[6],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[7],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[8],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[9],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM6:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[10],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM7:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[11],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        }
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
            memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info, sizeof(struct SET_PD_BLOCK_INFO_T));
            break;
        case MSDK_SCENARIO_ID_CUSTOM6:
        default:
            break;
        }
        break;
    case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
        LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", (UINT16) *feature_data);
        /*PDAF capacity enable or not, 2p8 only full size support PDAF*/
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            /* video & capture use same setting */
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_CUSTOM6:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;
        case MSDK_SCENARIO_ID_CUSTOM7:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;
        }
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
        /*END OF HDR CMD */
    case SENSOR_FEATURE_GET_BINNING_TYPE:
        switch (*(feature_data + 1)) {
            case MSDK_SCENARIO_ID_CUSTOM3:
            case MSDK_SCENARIO_ID_CUSTOM7:
                *feature_return_para_32 = 1; /*full size*/
                break;
            default:
                *feature_return_para_32 = 1465; /*BINNING_AVERAGED*/
                break;
            }
        pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
            *feature_return_para_32);
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_GET_VC_INFO2:
        LOG_INF("SENSOR_FEATURE_GET_VC_INFO2 %d\n", (UINT16) (*feature_data));
        pvcinfo2 = (struct SENSOR_VC_INFO2_STRUCT *) (uintptr_t) (*(feature_data + 1));
        get_vc_info_2(pvcinfo2, *feature_data_32);
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
    case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
        *(feature_data + 1) = 1; //always 1
        /* margin info by scenario */
        *(feature_data + 2) = imgsensor_info.margin;
        break;
    case SENSOR_FEATURE_GET_STAGGER_MAX_EXP_TIME:
        if (*feature_data == MSDK_SCENARIO_ID_CUSTOM4) {
            switch (*(feature_data + 1)) {
            case VC_STAGGER_NE:
                *(feature_data + 2) = 32757;
                break;
            case VC_STAGGER_ME:
                *(feature_data + 2) = 32757;
                break;
            case VC_STAGGER_SE:
                *(feature_data + 2) = 32757;
                break;
            default:
                *(feature_data + 2) = 32757;
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
        hdr_write_tri_shutter((UINT16)*feature_data, 0,
            (UINT16)*(feature_data+1));
        break;
    case SENSOR_FEATURE_SET_DUAL_GAIN://for 2EXP
        LOG_INF("SENSOR_FEATURE_SET_DUAL_GAIN LE=%d, SE=%d\n",
            (UINT16) *feature_data, (UINT16) *(feature_data + 1));
        // implement write gain for NE/SE
        hdr_write_tri_gain((UINT16)*feature_data, 0,
            (UINT16)*(feature_data+1));
        break;
    case SENSOR_FEATURE_SET_HDR_TRI_SHUTTER://for 3EXP
        LOG_INF("SENSOR_FEATURE_SET_HDR_TRI_SHUTTER LE=%d, ME=%d, SE=%d\n",
            (UINT16) *feature_data,(UINT16) *(feature_data + 1),(UINT16) *(feature_data + 2));
        // implement write shutter for NE/ME/SE
        hdr_write_tri_shutter((UINT16)*feature_data, (UINT16)*(feature_data+1),
            (UINT16)*(feature_data+2));
        break;
    case SENSOR_FEATURE_SET_HDR_TRI_GAIN:
        LOG_INF("SENSOR_FEATURE_SET_HDR_TRI_GAIN LGain=%d, SGain=%d, MGain=%d\n",
            (UINT16) *feature_data, (UINT16) *(feature_data + 1), (UINT16) *(feature_data + 2));
        // implement write gain for NE/ME/SE
        hdr_write_tri_gain((UINT16)*feature_data, (UINT16)*(feature_data+1),
            (UINT16)*(feature_data+2));
        break;
    case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
        *feature_return_para_32 = get_sensor_temperature();
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
        LOG_INF("SENSOR_FEATURE_GET_PDAF_REG_SETTING %d", (*feature_para_len));
        imx766_get_pdaf_reg_setting((*feature_para_len) / sizeof(UINT32), feature_data_16);
        break;
    case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
        /*LOG_INF("SENSOR_FEATURE_SET_PDAF_REG_SETTING %d",
         *    (*feature_para_len));
         */
        imx766_set_pdaf_reg_setting((*feature_para_len) / sizeof(UINT32), feature_data_16);
        break;
    case SENSOR_FEATURE_SET_PDAF:
        LOG_INF("PDAF mode :%d\n", *feature_data_16);
        imgsensor.pdaf_mode = *feature_data_16;
        break;
    case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
        set_shutter_frame_length((UINT16) (*feature_data), (UINT16) (*(feature_data + 1)),
            (BOOL) (*(feature_data + 2)));
        break;
    case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
        LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,
                (UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
        break;
    case SENSOR_FEATURE_SET_AWB_GAIN:
        //LOG_INF("debug awb sensor_mode: %d\n", imgsensor.sensor_mode);
        /* modify to separate 3hdr and remosaic */
        if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3
            || imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM7) {
            /*write AWB gain to sensor*/
            feedback_awbgain((UINT32)*(feature_data_32 + 1), (UINT32)*(feature_data_32 + 2));
        } else {
            imx766_awb_gain((struct SET_SENSOR_AWB_GAIN *) feature_para);
        }
        break;
#if 0
    case SENSOR_FEATURE_SET_HDR_SHUTTER:
        LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
            (UINT16)*feature_data, (UINT16)*(feature_data+1));
        #if 0
        ihdr_write_shutter((UINT16)*feature_data,
            (UINT16)*(feature_data+1));
        #endif
        break;
#endif
    case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
        LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
        streaming_control(KAL_FALSE);
        break;
    case SENSOR_FEATURE_SET_STREAMING_RESUME:
        LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
        if (*feature_data != 0)
            set_shutter(*feature_data);
        streaming_control(KAL_TRUE);
        break;
    case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
        memcpy(feature_return_para_32, &imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
        break;
    case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
        *feature_return_para_32 = imgsensor.current_ae_effective_frame;
        break;
    case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
    {
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.cap.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.normal_video.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.hs_video.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom1.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom2.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom3.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom4.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom5.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM6:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom6.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM7:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom7.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.pre.mipi_pixel_rate;
            break;
        }
    }
    break;
    case SENSOR_FEATURE_SET_SEAMLESS_EXTEND_FRAME_LENGTH:
        pr_debug("extend_frame_len %d\n", *feature_data);
        extend_frame_length((MUINT32) *feature_data);
        pr_debug("extend_frame_len done %d\n", *feature_data);
        break;
    case SENSOR_FEATURE_SEAMLESS_SWITCH:
    {
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            fpsIMX = imgsensor_info.custom7.pclk/imgsensor.frame_length/imgsensor_info.custom7.linelength;
            TimeIMX = 1000/fpsIMX;
            mdelay(TimeIMX);
            break;
        case MSDK_SCENARIO_ID_CUSTOM7:
            fpsIMX = imgsensor_info.pre.pclk/imgsensor.frame_length/imgsensor_info.pre.linelength;
            TimeIMX = 1000/fpsIMX;
            mdelay(TimeIMX);
            break;
        default:
            break;
        }
        if ((feature_data + 1) != NULL) {
            pAeCtrls = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
        } else {
            pr_debug("warning! no ae_ctrl input");
        }
        if (feature_data == NULL) {
            pr_info("error! input scenario is null!");
            return ERROR_INVALID_SCENARIO_ID;
        }
        LOG_INF("call seamless_switch");
        seamless_switch((*feature_data), pAeCtrls);
    }
    break;
    case SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS:
        if ((feature_data + 1) != NULL) {
            pScenarios = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
        } else {
            pr_info("input pScenarios vector is NULL!\n");
            return ERROR_INVALID_SCENARIO_ID;
        }
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *pScenarios = MSDK_SCENARIO_ID_CUSTOM7;
            break;
        case MSDK_SCENARIO_ID_CUSTOM7:
            *pScenarios = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            *pScenarios = MSDK_SCENARIO_ID_VIDEO_PREVIEW;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *pScenarios = MSDK_SCENARIO_ID_CUSTOM4;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM5:
        case MSDK_SCENARIO_ID_CUSTOM6:
        default:
            *pScenarios = 0xff;
            break;
        }
        pr_debug("SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS %d %d\n",
        *feature_data, *pScenarios);
        break;
    case SENSOR_FEATURE_GET_SEAMLESS_SYSTEM_DELAY:
        {
            int i;
            *(feature_data + 2) = 0;
            for (i = 0; i < sizeof(seamless_sys_delays) / sizeof(struct SEAMLESS_SYS_DELAY); i++) {
                if (*feature_data == seamless_sys_delays[i].source_scenario &&
                    *(feature_data + 1) == seamless_sys_delays[i].target_scenario) {
                    *(feature_data + 2) = seamless_sys_delays[i].sys_delay;
                    break;
            }
        }
        default:
            break;
        }
    }

    return ERROR_NONE;
} /* feature_control() */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 IMX766_MIPI_RAW_21305_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc != NULL)
        *pfFunc = &sensor_func;
    return ERROR_NONE;
} /* IMX766_MIPI_RAW_SensorInit */
