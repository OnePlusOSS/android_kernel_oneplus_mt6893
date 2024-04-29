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
 *     IMX882mipi_Sensor.c
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
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/types.h>
#include "imgsensor_eeprom.h"
#include "imx882mipiraw_Sensor.h"
#include "imx882pd_eeprom.h"
#include "imx882pd_seamless_switch.h"
#include "imgsensor_hwcfg_custom.h"

#ifdef MAX_VC_INFO_CNT
#undef MAX_VC_INFO_CNT
#endif
#define PD_PIX_2_EN 0
#define MAX_VC_INFO_CNT 13
#define SENSOR_GET_LINELENGTH_FOR_READOUT (0x1 << 0)
#define LEFT_SIZE (0x4000)
#define READ_EEPROM_1K (1024)
#define EEPROM_I2C_ADDR 0xA0

/***************Modify Following Strings for Debug**********************/
#define PFX "imx882pd_mipi_raw"
#define LOG_1 LOG_INF("IMX882PD ,MIPI 4LANE\n")
/****************************Modify end**************************/
extern enum IMGSENSOR_RETURN Eeprom_DataInit(
            enum IMGSENSOR_SENSOR_IDX sensor_idx,
            kal_uint32 sensorID);
extern struct CAMERA_DEVICE_INFO gImgEepromInfo;
//static kal_uint8 deviceInfo_register_value = 0x00;
static kal_uint8 otp_data[LEFT_SIZE] = {0};

#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

static kal_uint32 streaming_control(kal_bool enable);
#define MODULE_ID_OFFSET 0x0000
static kal_uint16 imx882pd_table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len);
static kal_uint16 imx882pd_burst_write_cmos_sensor(kal_uint16 *para, kal_uint32 len);
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static kal_uint16 imx882pd_SPC_setting[SPC_SIZE*2];
static kal_uint16 imx882pd_QSC_setting[QSC_SIZE*2];
static kal_uint8 qsc_flag = 0, spc_flag = 0;
static uint8_t deviceInfo_register_value = 0;
static kal_uint8 imx882_hw_version = 0;

static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = IMX882PD_SENSOR_ID23231,

    .checksum_value = 0xe5ffccac,
    .pre = { //reg_A  QBIN(VBIN)_4096x3072_30FPS with PDAF VB_max
        .pclk = 878400000,
        .linelength = 7500,
        .framelength = 3900,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 700800000,  //1022.00 Msps/Trio *3*2.28/10 = 699.048
        .max_framerate = 300,
    },

    .cap = { //reg_A  QBIN(VBIN)_4096x3072_30FPS with PDAF VB_max
        .pclk = 878400000,
        .linelength = 7500,
        .framelength = 3900,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 700800000,  //1022.00 Msps/Trio *3*2.28/10 = 699.048
        .max_framerate = 300,
    },

    .normal_video = { //reg_B  QBIN(VBIN)_4096x2304 @30FPS with PDAF VB_max
        .pclk = 878400000,
        .linelength = 7500,
        .framelength = 3900,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 2304,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 699048000,//1022.00 Msps/Trio *3*2.28/10 = 699.048
        .max_framerate = 300,
    },

    .hs_video = { //reg_C-4 QBIN(VBIN)-V2H2 2048x1152_120FPS with PDAF VB_max
        .pclk = 878400000,
        .linelength = 4024,
        .framelength = 1816,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2048,
        .grabwindow_height = 1152,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 740088000, //1082.00 Msps/Trio *3*2.28/10 
        .max_framerate = 1200, /* 120fps */
    },

    .slim_video = { //reg_A  QBIN(VBIN)_4096x3072_30FPS with PDAF VB_max
        .pclk = 878400000,
        .linelength = 7500,
        .framelength = 3900,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 699048000,  //1022.00 Msps/Trio *3*2.28/10 = 699.048
        .max_framerate = 300,
    },

    .custom1 = { //reg_A-1 QBIN(VBIN)_4096x3072_24FPS with PDAF VB_max
        .pclk = 878400000,
        .linelength = 7500,
        .framelength = 4876,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 699048000,  //1022.00 Msps/Trio *3*2.28/10 = 699.048
        .max_framerate = 240,
    },

    .custom2 = { //reg_B-1 QBIN_4096x2304 @60FPS with PDAF VB_max
        .pclk = 878400000,
        .linelength = 4614,
        .framelength = 3149,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 2304,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 1069776000,  //1564.00 Msps/Trio *3*2.28/10 
        .max_framerate = 600,
    },

    .custom3 = {  //reg_D Full RMSC 15fps with PDAF VB_max
        .pclk = 878400000,
        .linelength = 8960,
        .framelength = 6504,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 8192,
        .grabwindow_height = 6144,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 1110816000, ////1624.00 Msps/Trio *3*2.28/10 = 1110.816
        .max_framerate = 150,
        },

    .custom4 = { //reg_C-5 QBIN-V2H2 2048x1152_240FPS w/o PDAF VB_max
        .pclk = 878400000,
        .linelength = 2468,
        .framelength = 1470,// 7804/2
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2048,
        .grabwindow_height = 1152,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 976752000,//1428.00 Msps/Trio *3*2.28/10 
        .max_framerate = 2400,
    },

    .custom5 = {//reg_A-3-S1 Full-RMSC-Crop_4096x3072_30FPS with PDAF VB_max seamless A-2-S1
        .pclk = 878400000,
        .linelength = 8960,
        .framelength = 3252,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 699048000, //1022 Msps/Trio *3*2.28/10 = 699.048
        .max_framerate = 300, /* 30fps */
    },

    .custom6 = {// reg_C-6 QBIN-V2H2 1280x674_480FPS w/o PDAF VB_max
        .pclk = 878400000,
        .linelength = 2468,
        .framelength = 734,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 674,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 692208000, //1012 Msps/Trio *3*2.28/10
        .max_framerate = 4800, //480fps
    },

    .custom7 = {
        .pclk = 878400000,
        .linelength = 7500,
        .framelength = 3900,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 699048000,  //1022.00 Msps/Trio *3*2.28/10 = 699.048
        .max_framerate = 300,
    },

    .custom8 = {
        .pclk = 2246400000,
        .linelength = 15616,
        .framelength = 4792,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 2304,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 858510000,
        .max_framerate = 300,
    },

    .custom9 = {//reg_C-3 QBIN(VBIN)-V2H2 2048x1536_24FPS with PDAF VB_max
        .pclk = 878400000,
        .linelength = 4024,
        .framelength = 9088,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2048,
        .grabwindow_height = 1536,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 741456000, //1084.00 Msps/Trio *3*2.28/10
        .max_framerate = 240,
    },

    .custom10 = {
        .pclk = 2246400000,
        .linelength = 15616,
        .framelength = 4792,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 2304,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 858510000,
        .max_framerate = 300,
    },


    .min_gain = BASEGAIN * 1, //1.429
    .max_gain = BASEGAIN * 64,
    .min_gain_iso = 100,
    .margin = 48,
    .min_shutter = 16,
    .gain_step = 1,
    .gain_type = 0,
    .max_frame_length = 0xffff,
    .ae_shut_delay_frame = 0,
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,    /* isp gain delay frame for AE cycle */
    .ihdr_support = 0,    /* 1: support; 0: not support */
    .ihdr_le_firstline = 0,    /* 1:le first; 0: se first */
    .temperature_support = 1, /* 1, support; 0,not support */
    .sensor_mode_num = 15,    /* support sensor mode num */
    .frame_time_delay_frame = 3,
    .pre_delay_frame = 2,    /* enter preview delay frame num */
    .cap_delay_frame = 2,    /* enter capture delay frame num */
    .video_delay_frame = 2,    /* enter video delay frame num */
    .hs_video_delay_frame = 2,    /* enter hs video delay frame num */
    .slim_video_delay_frame = 2,/* enter slim video delay frame num */
    .custom1_delay_frame = 2,    /* enter custom1 delay frame num */
    .custom2_delay_frame = 2,    /* enter custom2 delay frame num */
    .custom3_delay_frame = 2,    /* enter custom3 delay frame num */
    .custom4_delay_frame = 2,    /* enter custom4 delay frame num */
    .custom5_delay_frame = 2,    /* enter custom5 delay frame num */
    .custom6_delay_frame = 2,    /* enter custom6 delay frame num */
    .custom7_delay_frame = 2,    /* enter custom7 delay frame num */
    .custom8_delay_frame = 2,    /* enter custom8 delay frame num */
    .custom9_delay_frame = 2,    /* enter custom9 delay frame num */
    .custom10_delay_frame = 2,    /* enter custom10 delay frame num */

    .isp_driving_current = ISP_DRIVING_4MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_CPHY,
    .mipi_settle_delay_mode = 0,
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,
    .mclk = 24, /* suggest 24 or 26 for 24Mhz or 26Mhz */
    .mipi_lane_num = SENSOR_MIPI_3_LANE,
    .i2c_addr_table = {0x34, 0xff},
    /* record sensor support all write id addr,
     * only supprt 4 must end with 0xff
     */
    .i2c_speed = 1000, /* kbps */
};

static struct imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,    /* mirror and flip on information */
    .sensor_mode = IMGSENSOR_MODE_INIT,
    /* IMGSENSOR_MODE enum value,record current sensor mode,such as:
     * INIT, Preview, Capture, Video,High Speed Video, Slim Video
     */
    .shutter = 0x3D0,    /* current shutter */
    .gain = 0x100,        /* current gain */
    .dummy_pixel = 0,    /* current dummypixel */
    .dummy_line = 0,    /* current dummyline */
    .current_fps = 450,
    .autoflicker_en = KAL_FALSE,
    .test_pattern = KAL_FALSE,
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
    .ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
    .i2c_write_id = 0x34, /* record current sensor's i2c write id */
    .extend_frame_length_en = KAL_FALSE,
    .fast_mode_on = KAL_FALSE,
};

static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[] = {
    // Preview
    {8192, 6144,    0,    0, 8192, 6144, 4096, 3072,    0,    0, 4096, 3072,    0,    0, 4096, 3072},
    // Capture
    {8192, 6144,    0,    0, 8192, 6144, 4096, 3072,    0,    0, 4096, 3072,    0,    0, 4096, 3072},
    // Video
    {8192, 6144,    0,  768, 8192, 4608, 4096, 2304,    0,    0, 4096, 2304,    0,    0, 4096, 2304},
    // hs_video
    {8192, 6144,    0,  768, 8192, 4608, 2048, 1152,    0,    0, 2048, 1152,    0,    0, 2048, 1152},
    // slim_video
    {8192, 6144,    0,    0, 8192, 6144, 4096, 3072,    0,    0, 4096, 3072,    0,    0, 4096, 3072},
    // custom1 4096*3072@24FPS
    {8192, 6144,    0,    0, 8192, 6144, 4096, 3072,    0,    0, 4096, 3072,    0,    0, 4096, 3072},
    // custom2 4096*2304@60FPS
    {8192, 6144,    0,  768, 8192, 4608, 4096, 2304,    0,    0, 4096, 2304,    0,    0, 4096, 2304},
    // custom3 remosaic
    {8192, 6144,    0,    0, 8192, 6144, 8192, 6144,    0,    0, 8192, 6144,    0,    0, 8192, 6144},
    // custom4 2048x1152_240FPS
    {8192, 6144,    0,  768, 8192, 4608, 2048, 1152,    0,    0, 2048, 1152,    0,    0, 2048, 1152},
    /* custom5 full_rmsc_corp 4096*3072@30fps*/
    {8192, 6144,    0, 1536, 8192, 3072, 8192, 3072, 2048,    0, 4096, 3072,    0,    0, 4096, 3072},
    /* custom6 1280x674_480FPS*/
    {8192, 6144,    0, 1600, 8192, 2944, 2048,  736,  384,   31, 1280,  674,    0,    0, 1280,  674},
    /* custom7 preISP Reg S*/
    {8192, 6144,    0,    0, 8192, 6144, 4096, 3072,    0,    0, 4096, 3072,    0,    0, 4096, 3072},
    /* custom8 preISP Reg K 2DOL*/
    {8192, 6144,    0,  768, 8192, 4608, 4096, 2304,    0,    0, 4096, 2304,    0,    0, 4096, 2304},
    /* custom9 2048x1536 @24fps*/
    {8192, 6144,    0,    0, 8192, 6144, 2048, 1536,    0,    0, 2048, 1536,    0,    0, 2048, 1536},
    //custom10 
    {8192, 6144,    0,  768, 8192, 4608, 4096, 2304,    0,    0, 4096, 2304,    0,    0, 4096, 2304},
};

static struct SENSOR_VC_INFO2_STRUCT SENSOR_VC_INFO2[] = {
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //preivew
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
            {VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x1400, 0x300},
#if PD_PIX_2_EN
            {VC_PDAF_STATS_NE_PIX_2, 0x06, 0x2b, 0x800, 0x300},
#endif
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //capture
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
            {VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x1400, 0x300},
#if PD_PIX_2_EN
            {VC_PDAF_STATS_NE_PIX_2, 0x06, 0x2b, 0x800, 0x300},
#endif
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //normal_video
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x900},
            {VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x1400, 0x240},
#if PD_PIX_2_EN
            {VC_PDAF_STATS_NE_PIX_2, 0x06, 0x2b, 0x800, 0x240},
#endif
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //hs_video
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x0800, 0x0480},
            {VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x0A00, 0x120},
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //slim_video
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
            {VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x1400, 0x300},
#if PD_PIX_2_EN
            {VC_PDAF_STATS_NE_PIX_2, 0x06, 0x2b, 0x800, 0x300},
#endif
        },
        1
    
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom1
        {
             {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
             {VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x1400, 0x0300},
#if PD_PIX_2_EN
            {VC_PDAF_STATS_NE_PIX_2, 0x00, 0x31, 0xa00, 0x300},
#endif
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom2
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x900},
            {VC_PDAF_STATS, 0x00, 0x30, 0x0276, 0x0480},
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom3
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 8192, 6144},
            {VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x1400, 0xC00},
#if PD_PIX_2_EN
            {VC_PDAF_STATS_NE_PIX_2, 0x00, 0x31, 0xa00, 0x300},
#endif
        },
        1
    },
    {
        0x04, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom4
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x800, 0x480},
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom5
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
            {VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x0A00, 0x600},
#if PD_PIX_2_EN
            {VC_PDAF_STATS_NE_PIX_2, 0x00, 0x31, 0xa00, 0x300},
#endif
        },
        1
    },
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom6
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x500, 0x2a2},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom7
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
            {VC_PDAF_STATS_NE_PIX_1, 0x03, 0x2b, 0x1000, 0x300},
#if PD_PIX_2_EN
            {VC_PDAF_STATS_NE_PIX_2, 0x06, 0x2b, 0x800, 0x300},
#endif
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom8
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x900},
            {VC_PDAF_STATS_NE_PIX_1, 0x03, 0x2b, 0x1000, 0x240},
#if PD_PIX_2_EN
            {VC_PDAF_STATS_NE_PIX_2, 0x06, 0x2b, 0x800, 0x240},
#endif
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom9
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x800, 0x600},
            {VC_PDAF_STATS, 0x00, 0x30, 0xA00, 0x180},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom10
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x900},
            {VC_PDAF_STATS_NE_PIX_1, 0x03, 0x2b, 0x1000, 0x240},
#if PD_PIX_2_EN
            {VC_PDAF_STATS_NE_PIX_2, 0x06, 0x2b, 0x800, 0x240},
#endif
        },
        1
    }
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
    .i4OffsetX = 16,
    .i4OffsetY = 32,
    .i4PitchX = 8,
    .i4PitchY = 16,
    .i4PairNum = 4,
    .i4SubBlkW = 8,
    .i4SubBlkH = 4,
    .i4PosL = {{16, 35}, {20, 37}, {19, 42}, {23, 44}},
    .i4PosR = {{18, 33}, {22, 39}, {17, 40}, {21, 46}},
    .i4BlockNumX = 508,
    .i4BlockNumY = 188,
    .i4LeFirst = 0,
    .i4Crop = {
        {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
        {0, 0}, {0, 384}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
        {0, 0}, {0, 0}, {0, 0}, {0, 0}
    },
    .iMirrorFlip = 0,
};

static struct SEAMLESS_SYS_DELAY seamless_sys_delays[] = {
    { MSDK_SCENARIO_ID_VIDEO_PREVIEW, MSDK_SCENARIO_ID_CUSTOM5, 1 },
    { MSDK_SCENARIO_ID_CUSTOM5, MSDK_SCENARIO_ID_VIDEO_PREVIEW, 1 },
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
    case MSDK_SCENARIO_ID_CUSTOM8:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[12],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CUSTOM9:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[13],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
	case MSDK_SCENARIO_ID_CUSTOM10:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[14],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    default:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[0],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    }
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
    char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF),
            (char)(para & 0xFF)};

    return iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static kal_uint32 get_cur_exp_cnt()
{
    kal_uint32 exp_cnt = 1;

    if (0x1 == (read_cmos_sensor_8(0x3170) & 0x1)) { // DOL_EN
        exp_cnt = 2;
    }

    return exp_cnt;
}

static void read_sensor_verison()
{
	kal_uint8 chip_value = 0;

	chip_value = read_cmos_sensor_8(chip_revision_reg);
	if (chip_value > 2){
		imx882_hw_version = HW_MP;
	} else if (chip_value == 2) {
		imx882_hw_version = HW_CUT1_0;
	} else if (chip_value == 1) {
		imx882_hw_version = HW_CUT0_91;
	} else {
		imx882_hw_version = HW_CUT0_90;
	}

	LOG_INF("%s chip_vale(0x%x) hw_version(%d) \n",
		__func__, chip_value, imx882_hw_version);
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
    LOG_INF("dummyline = %d, dummypixels = %d, frame_length:%d, line_length:%d\n",
        imgsensor.dummy_line, imgsensor.dummy_pixel, imgsensor.frame_length, imgsensor.line_length);
    /* return;*/ /* for test */
    if (imgsensor.shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = imgsensor.shutter + imgsensor_info.margin;
    while(write_retry) {
        ret = write_frame_len(imgsensor.frame_length);
        if (ret == IMGSENSOR_RETURN_SUCCESS)
            break;
        write_retry--;
        mdelay(2);
    }
}    /*    set_dummy  */

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

    imgsensor.dummy_line =
            imgsensor.frame_length - imgsensor.min_frame_length;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line =
            imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if (min_framelength_en)
        imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);
    set_dummy();
}    /*    set_max_framerate  */

#define MAX_CIT_LSHIFT 9
static void write_shutter(kal_uint32 shutter, kal_bool gph)
{
    kal_uint16 realtime_fps = 0;
    kal_uint16 l_shift = 1;
    //shutter = round_up(shutter, 4);

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

    shutter = (shutter >> 1) << 1;
    imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
    //frame_length and shutter should be an even number

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10
                / imgsensor.frame_length;
        LOG_INF("autoflicker enable, realtime_fps = %d\n", realtime_fps);
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
    }

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
         write_cmos_sensor_8(0x3160, read_cmos_sensor(0x3160) | (l_shift & 0x0f));
    } else {
         write_cmos_sensor_8(0x3160, read_cmos_sensor(0x3160) & 0xf0);
    }

    write_frame_len(imgsensor.frame_length);

    /* Update Shutter */
    write_cmos_sensor_8(0x0350, 0x01); /* Enable auto extend */
    write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
    write_cmos_sensor_8(0x0203, shutter  & 0xFF);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x00);
    }

    LOG_INF("shutter =%d, framelength =%d read 0x0202: 0x%x 0x0203: 0x%x\n",
        shutter, imgsensor.frame_length, read_cmos_sensor_8(0x0202), read_cmos_sensor_8(0x0203));
}    /*    write_shutter  */

/*************************************************************************
 * FUNCTION
 *    set_shutter
 *
 * DESCRIPTION
 *    This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *    iShutter : exposured lines
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

/*************************************************************************
 * FUNCTION
 *    set_shutter_frame_length
 *
 * DESCRIPTION
 *    for frame & 3A sync
 *
 *************************************************************************/
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length,
                kal_bool auto_extend_en)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    kal_int32 dummy_line = 0;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    /*0x3500, 0x3501, 0x3502 will increase VBLANK to
     *get exposure larger than frame exposure
     *AE doesn't update sensor gain at capture mode,
     *thus extra exposure lines must be updated here.
     */

    /* OV Recommend Solution */
    /*if shutter bigger than frame_length,
     *should extend frame length first
     */
    spin_lock(&imgsensor_drv_lock);
    /* Change frame time */
    if (frame_length > 1)
        dummy_line = frame_length - imgsensor.frame_length;

    imgsensor.frame_length = imgsensor.frame_length + dummy_line;

    /*  */
    //if (shutter > imgsensor.frame_length - imgsensor_info.margin)
    //    imgsensor.frame_length = shutter + imgsensor_info.margin;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    shutter = (shutter < imgsensor_info.min_shutter)
            ? imgsensor_info.min_shutter : shutter;
    shutter =
    (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
        ? (imgsensor_info.max_frame_length - imgsensor_info.margin)
        : shutter;
    write_cmos_sensor_8(0x0104, 0x01);

    shutter = (shutter >> 1) << 1;
    imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
    //frame_length and shutter should be an even number

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
                imgsensor.frame_length;
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
    write_cmos_sensor_8(0x0203, shutter  & 0xFF);
    write_cmos_sensor_8(0x0104, 0x00);
    LOG_INF(
        "Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
        shutter, imgsensor.frame_length, frame_length,
        dummy_line, read_cmos_sensor(0x0350));

}    /* set_shutter_frame_length */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
     kal_uint16 reg_gain = 0x0;

    reg_gain = 16384 - (16384*64)/gain;
    return (kal_uint16) reg_gain;
}

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
    kal_uint16 reg_gain, max_gain = imgsensor_info.max_gain;

    if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3) {
        max_gain = 16 * BASEGAIN;//set 16x gain for full size mode.
    }

    if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
        LOG_INF("Error gain setting");

        if (gain < imgsensor_info.min_gain)
            gain = imgsensor_info.min_gain;
        else if (gain > max_gain)
            gain = max_gain;
    }

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    reg_gain = (reg_gain >> 2) << 2;
    LOG_INF(" gph:%d,gain = %d, reg_gain = 0x%x\n ", gph, gain, reg_gain);

    if (gph) {
        write_cmos_sensor_8(0x0104, 0x01);
    }
    write_cmos_sensor_8(0x0204, (reg_gain>>8) & 0xFF);
    write_cmos_sensor_8(0x0205, reg_gain & 0xFF);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x00);
    }
    LOG_INF(" 0x0204:0x%x,0x0205 = 0x%x\n ", read_cmos_sensor_8(0x0204), read_cmos_sensor_8(0x0205));
    return gain;
} /* set_gain_w_gph */

static kal_uint16 set_gain(kal_uint16 gain)
{
    return set_gain_w_gph(gain, KAL_TRUE);
} /* set_gain */

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
    LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n",
        enable);
    if (enable)
        write_cmos_sensor_8(0x0100, 0X01);
    else
        write_cmos_sensor_8(0x0100, 0x00);
    return ERROR_NONE;
}

#define I2C_BUFFER_LEN 255 /* trans# max is 255, each 3 bytes */
static kal_uint16 imx882pd_table_write_cmos_sensor(kal_uint16 *para,
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
        /* Write when remain buffer size is less than 3 bytes
         * or reach end of data
         */
        if ((I2C_BUFFER_LEN - tosend) < 3
            || IDX == len || addr != addr_last) {
            iBurstWriteReg_multi(puSendCmd,
                        tosend,
                        imgsensor.i2c_write_id,
                        3,
                        imgsensor_info.i2c_speed);
            tosend = 0;
        }
    }
    return 0;
}
#define I2C_BURST_BUFFER_LEN 255 /* trans# max is 255, each 3 bytes */
static kal_uint16 imx882pd_burst_write_cmos_sensor(
                    kal_uint16 *para, kal_uint32 len)
{
    char puSendCmd[I2C_BURST_BUFFER_LEN];
    kal_uint32 tosend, IDX;
    kal_uint16 addr = 0, addr_last = 0, data;

    tosend = 0;
    IDX = 0;
    while (len > IDX) {
        addr = para[IDX];
        if (tosend == 0)
        {
            puSendCmd[tosend++] = (char)(addr >> 8);
            puSendCmd[tosend++] = (char)(addr & 0xFF);
            data = para[IDX + 1];
            puSendCmd[tosend++] = (char)(data & 0xFF);
            IDX += 2;
            addr_last = addr;
        }
        else if (addr == addr_last + 1)
        {
            data = para[IDX + 1];
            puSendCmd[tosend++] = (char)(data & 0xFF);
            addr_last = addr;
            IDX += 2;
        }

        if ((tosend >= I2C_BURST_BUFFER_LEN)||
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

static kal_uint16 imx882pd_init_setting[] = {
//Stand-by OFF Sequence
//Power ON
//Input EXTCLK
//XCLR OFF
//External Clock Setting
//Address value
    0x0136, 0x18,
    0x0137, 0x00,
//PHY_VIF Setting
//Address    value
    0x3304, 0x00,
//Register version
//Address    value
    0x33F0, 0x06,
    0x33F1, 0x04,
//Signaling mode setting
//Address    value
    0x0111, 0x03,
//ROI Setting 2
//Address    value
    0x3855, 0x01,
//MIPI Global Timing control Setting
//Address    value
    0x0808, 0x02,
//Global Setting
//Address    value
    0x0D06, 0x82,
    0x0D07, 0x02,
    0x0D12, 0x78,
    0x0D13, 0x01,
    0x0D0D, 0x00,
    0x0D08, 0x01,
    0x1012, 0x00,
    0x2EDF, 0x07,
    0x3953, 0x01,
    0x3954, 0x01,
    0x3955, 0x01,
    0x3B30, 0x01,
    0x3B32, 0x0A,
    0x3B33, 0x04,
    0x3B34, 0x0A,
    0x3B36, 0xB2,
    0x3B40, 0x01,
    0x3B42, 0x0A,
    0x3B43, 0x04,
    0x3B45, 0x0D,
    0x3B46, 0x40,
    0x5A1D, 0x52,
    0x5A27, 0x19,
    0x5A37, 0xC8,
    0x5A39, 0x25,
    0x5A3D, 0x0E,
    0x5A41, 0x19,
    0x5A47, 0x62,
    0x5A51, 0x52,
    0x5A5B, 0x19,
    0x5A69, 0xC8,
    0x5A6B, 0x25,
    0x5A6F, 0x11,
    0x5A73, 0x19,
    0x5A79, 0x8A,
    0x5A83, 0x52,
    0x5A8D, 0x19,
    0x5A9B, 0xC8,
    0x5A9D, 0x25,
    0x5AA1, 0x52,
    0x5AA5, 0x19,
    0x5AAB, 0x93,
    0x5AB5, 0x52,
    0x5ABF, 0x19,
    0x5ACD, 0xC8,
    0x5ACF, 0x25,
    0x5AD3, 0x11,
    0x5AD7, 0x19,
    0x5ADD, 0x4B,
    0x5AE7, 0x52,
    0x5AF1, 0x19,
    0x5AF5, 0x2D,
    0x5AFF, 0xC8,
    0x5B01, 0x25,
    0x5B05, 0x0E,
    0x5B09, 0x19,
    0x5B0D, 0x9E,
    0x5B0F, 0x34,
    0x5B21, 0x19,
    0x5B27, 0x2D,
    0x5B33, 0x19,
    0x5B39, 0x9E,
    0x5B3F, 0x52,
    0x5B49, 0x19,
    0x5B4D, 0x87,
    0x5B57, 0xC8,
    0x5B59, 0x25,
    0x5B5D, 0x0E,
    0x5B61, 0x19,
    0x5B65, 0xDA,
    0x5B6D, 0x52,
    0x5B79, 0x19,
    0x5B7F, 0x2D,
    0x5B8B, 0x19,
    0x5B91, 0x9E,
    0x5B97, 0x52,
    0x5BA1, 0x19,
    0x5BA5, 0x2D,
    0x5BAF, 0xC8,
    0x5BB1, 0x25,
    0x5BB5, 0x11,
    0x5BB9, 0x19,
    0x5BBD, 0x9E,
    0x5BBF, 0x5C,
    0x5BD1, 0x19,
    0x5BD7, 0x2D,
    0x5BE3, 0x19,
    0x5BE9, 0x9E,
    0x5BEF, 0x52,
    0x5BF9, 0x19,
    0x5BFD, 0x87,
    0x5C07, 0xC8,
    0x5C09, 0x25,
    0x5C0D, 0x11,
    0x5C11, 0x19,
    0x5C15, 0xDA,
    0x5C29, 0x19,
    0x5C2F, 0x2D,
    0x5C3B, 0x19,
    0x5C41, 0x9E,
    0x5C47, 0x52,
    0x5C51, 0x19,
    0x5C57, 0x41,
    0x5C5F, 0xC8,
    0x5C61, 0x25,
    0x5C65, 0x11,
    0x5C69, 0x19,
    0x5C6E, 0x01,
    0x5C6F, 0x13,
    0x5C85, 0xC8,
    0x5C87, 0x25,
    0x5C8B, 0x1B,
    0x5C8F, 0x19,
    0x5C95, 0x41,
    0x5C9B, 0x52,
    0x5CA5, 0x19,
    0x5CAA, 0x03,
    0x5CAB, 0xDE,
    0x5CB3, 0xC8,
    0x5CB5, 0x25,
    0x5CB9, 0x11,
    0x5CBD, 0x19,
    0x5CC3, 0xB8,
    0x5CD9, 0xC8,
    0x5CDB, 0x25,
    0x5CDF, 0x1B,
    0x5CE3, 0x19,
    0x5CE8, 0x03,
    0x5CE9, 0xDE,
    0x5CEF, 0x52,
    0x5CF9, 0x19,
    0x5CFF, 0x79,
    0x5D07, 0xC8,
    0x5D09, 0x25,
    0x5D0C, 0x00,
    0x5D0D, 0x52,
    0x5D11, 0x19,
    0x5D16, 0x01,
    0x5D17, 0x28,
    0x5D2D, 0xC8,
    0x5D2F, 0x25,
    0x5D33, 0x53,
    0x5D37, 0x19,
    0x5D3D, 0x79,
    0x5D43, 0x52,
    0x5D4D, 0x19,
    0x5D51, 0x2D,
    0x5D53, 0x41,
    0x5D55, 0x6B,
    0x5D5B, 0xC8,
    0x5D5D, 0x25,
    0x5D61, 0x11,
    0x5D65, 0x19,
    0x5D69, 0x9E,
    0x5D6B, 0xD0,
    0x5D7D, 0x19,
    0x5D83, 0x2D,
    0x5D95, 0xC8,
    0x5D97, 0x25,
    0x5D9B, 0x1B,
    0x5D9F, 0x19,
    0x5DA5, 0x41,
    0x5DB1, 0x19,
    0x5DB7, 0x6B,
    0x5DC3, 0x19,
    0x5DC9, 0x9E,
    0x5DCF, 0x52,
    0x5DD9, 0x19,
    0x5DDD, 0x87,
    0x5DDF, 0x41,
    0x5DE1, 0x41,
    0x5DE7, 0xC8,
    0x5DE9, 0x25,
    0x5DED, 0x11,
    0x5DF1, 0x19,
    0x5DF5, 0xDA,
    0x5DF7, 0x4A,
    0x5E09, 0x19,
    0x5E0F, 0x2D,
    0x5E21, 0xC8,
    0x5E23, 0x25,
    0x5E27, 0x1B,
    0x5E2B, 0x19,
    0x5E31, 0x41,
    0x5E3D, 0x19,
    0x5E43, 0x6B,
    0x5E4F, 0x19,
    0x5E55, 0x9E,
    0x5E5B, 0x52,
    0x5E65, 0x19,
    0x5E69, 0x2D,
    0x5E6B, 0x41,
    0x5E73, 0xC8,
    0x5E75, 0x25,
    0x5E79, 0x11,
    0x5E7D, 0x19,
    0x5E81, 0x9E,
    0x5E83, 0xFB,
    0x5E95, 0x19,
    0x5E9B, 0x2D,
    0x5EAD, 0xC8,
    0x5EAF, 0x25,
    0x5EB3, 0x1B,
    0x5EB7, 0x19,
    0x5EBD, 0x41,
    0x5EC9, 0x19,
    0x5ECF, 0x9E,
    0x5ED5, 0x52,
    0x5EDF, 0x19,
    0x5EE3, 0x2D,
    0x5EE5, 0x5A,
    0x5EE7, 0x41,
    0x5EEF, 0xC8,
    0x5EF1, 0x25,
    0x5EF5, 0x11,
    0x5EF9, 0x19,
    0x5EFD, 0xDA,
    0x5EFF, 0xCB,
    0x5F11, 0x19,
    0x5F17, 0x2D,
    0x5F23, 0x19,
    0x5F29, 0x2D,
    0x5F3B, 0xC8,
    0x5F3D, 0x25,
    0x5F41, 0x1B,
    0x5F45, 0x19,
    0x5F4B, 0x41,
    0x5F57, 0x19,
    0x5F5D, 0x9E,
    0x5F63, 0x52,
    0x5F6D, 0x19,
    0x5F7F, 0x55,
    0x5F83, 0x0E,
    0x5F87, 0x19,
    0x5F93, 0x12,
    0x6C06, 0xFF,
    0x6C07, 0xFF,
    0x6C0E, 0x00,
    0x6C0F, 0x02,
    0x6C11, 0xD9,
    0x6C12, 0xFF,
    0x6C13, 0xFF,
    0x6C1A, 0x00,
    0x6C1B, 0x02,
    0x6C1D, 0xD9,
    0x6C1E, 0xFF,
    0x6C1F, 0xFF,
    0x6C26, 0x00,
    0x6C27, 0x02,
    0x6C29, 0xD9,
    0x6C2A, 0xFF,
    0x6C2B, 0xFF,
    0x6C32, 0x00,
    0x6C33, 0x02,
    0x6C35, 0xD9,
    0x6C36, 0xFF,
    0x6C37, 0xFF,
    0x6C3E, 0x00,
    0x6C3F, 0x02,
    0x6C41, 0xD9,
    0x6C42, 0xFF,
    0x6C43, 0xFF,
    0x6C4A, 0x00,
    0x6C4B, 0x02,
    0x6C4D, 0xD9,
    0x6C4E, 0xFF,
    0x6C4F, 0xFF,
    0x6C5A, 0xFF,
    0x6C5B, 0xFF,
    0x6C66, 0xFF,
    0x6C67, 0xFF,
    0x6C72, 0xFF,
    0x6C73, 0xFF,
    0x6C7E, 0xFF,
    0x6C7F, 0xFF,
    0x6CAA, 0x00,
    0x6CAB, 0x02,
    0x6CAD, 0xD9,
    0x6CAE, 0x00,
    0x6CAF, 0x02,
    0x6CB1, 0xD9,
    0x6CB2, 0x00,
    0x6CB3, 0x02,
    0x6CB5, 0xD9,
    0x6CB6, 0x00,
    0x6CB7, 0x02,
    0x6CB9, 0xD9,
    0x6CBA, 0x00,
    0x6CBB, 0x02,
    0x6CBD, 0xD9,
    0x6E3F, 0xDE,
    0x6E47, 0xDE,
    0x6E4F, 0xDE,
    0x6E57, 0xDE,
    0x6E5F, 0xDE,
    0x6E67, 0xDE,
    0x6EB3, 0xDE,
    0x6EB7, 0xDE,
    0x6EBB, 0xDE,
    0x6EBF, 0xDE,
    0x6EC3, 0xDE,
    0x7476, 0x00,
    0x7477, 0x00,
    0x7478, 0x00,
    0x7509, 0x00,
    0x750B, 0x00,
    0x7516, 0x01,
    0x7524, 0x0C,
    0x7528, 0x02,
    0x7530, 0x03,
    0x7616, 0x0C,
    0x7619, 0x0C,
    0x761C, 0x0C,
    0x761D, 0x0C,
    0x761E, 0x0C,
    0x761F, 0x0C,
    0x7620, 0x00,
    0x7623, 0x00,
    0x7624, 0x00,
    0x7625, 0x00,
    0x7626, 0x00,
    0x7627, 0x00,
    0x7628, 0x00,
    0x7629, 0x00,
    0x762A, 0x00,
    0x762B, 0x00,
    0x762C, 0x00,
    0x762D, 0x00,
    0x762E, 0x00,
    0x762F, 0x00,
    0x7631, 0x07,
    0x7632, 0x07,
    0x7634, 0x07,
    0x7635, 0x07,
    0x76FC, 0x44,
    0x76FD, 0x2A,
    0x76FE, 0x2C,
    0x76FF, 0x26,
    0x7700, 0x43,
    0x7701, 0x2A,
    0x7702, 0x2A,
    0x7703, 0x26,
    0x7704, 0x2C,
    0x7705, 0x2A,
    0x7706, 0x2A,
    0x7707, 0x65,
    0x7708, 0x44,
    0x7709, 0x48,
    0x770A, 0x40,
    0x770B, 0x64,
    0x770C, 0x44,
    0x770D, 0x44,
    0x770E, 0x40,
    0x770F, 0x48,
    0x7710, 0x44,
    0x7711, 0x44,
    0x7712, 0x45,
    0x7713, 0x43,
    0x7714, 0x47,
    0x7715, 0x40,
    0x7716, 0x42,
    0x7717, 0x43,
    0x7718, 0x43,
    0x7719, 0x40,
    0x771A, 0x47,
    0x771B, 0x43,
    0x771C, 0x43,
    0x771D, 0x5D,
    0x771E, 0x43,
    0x771F, 0x46,
    0x7720, 0x40,
    0x7721, 0x62,
    0x7722, 0x43,
    0x7723, 0x43,
    0x7724, 0x40,
    0x7725, 0x46,
    0x7726, 0x43,
    0x7727, 0x43,
    0x7728, 0x41,
    0x7729, 0x43,
    0x772A, 0x3F,
    0x772B, 0x41,
    0x772C, 0x41,
    0x772D, 0x3F,
    0x772E, 0x43,
    0x772F, 0x41,
    0x7730, 0x41,
    0x7731, 0x40,
    0x7732, 0x42,
    0x7733, 0x3F,
    0x7734, 0x40,
    0x7735, 0x40,
    0x7736, 0x3F,
    0x7737, 0x42,
    0x7738, 0x40,
    0x7739, 0x40,
    0x773A, 0x40,
    0x773B, 0x15,
    0x773C, 0x16,
    0x773D, 0x09,
    0x773E, 0x3B,
    0x773F, 0x15,
    0x7740, 0x15,
    0x7741, 0x09,
    0x7742, 0x16,
    0x7743, 0x15,
    0x7744, 0x15,
    0x7745, 0x63,
    0x7746, 0x2F,
    0x7747, 0x2F,
    0x7748, 0x19,
    0x7749, 0x5D,
    0x774A, 0x2F,
    0x774B, 0x2F,
    0x774C, 0x19,
    0x774D, 0x2F,
    0x774E, 0x2F,
    0x774F, 0x2F,
    0x7750, 0x45,
    0x7751, 0x2F,
    0x7752, 0x2F,
    0x7753, 0x24,
    0x7754, 0x43,
    0x7755, 0x2F,
    0x7756, 0x2F,
    0x7757, 0x24,
    0x7758, 0x2F,
    0x7759, 0x2F,
    0x775A, 0x2F,
    0x775B, 0x57,
    0x775C, 0x39,
    0x775D, 0x36,
    0x775E, 0x32,
    0x775F, 0x64,
    0x7760, 0x39,
    0x7761, 0x39,
    0x7762, 0x32,
    0x7763, 0x36,
    0x7764, 0x39,
    0x7765, 0x39,
    0x7766, 0x3B,
    0x7767, 0x3B,
    0x7768, 0x3B,
    0x7769, 0x3B,
    0x776A, 0x3B,
    0x776B, 0x3B,
    0x776C, 0x3B,
    0x776D, 0x3B,
    0x776E, 0x3B,
    0x776F, 0x41,
    0x7770, 0x40,
    0x7771, 0x3F,
    0x7772, 0x41,
    0x7773, 0x41,
    0x7774, 0x3F,
    0x7775, 0x40,
    0x7776, 0x41,
    0x7777, 0x41,
    0x7778, 0x0F,
    0x7779, 0x0A,
    0x777A, 0x0A,
    0x777B, 0x0A,
    0x777C, 0x0F,
    0x777D, 0x0A,
    0x777E, 0x0A,
    0x777F, 0x0A,
    0x7780, 0x0A,
    0x7781, 0x0A,
    0x7782, 0x0A,
    0x7783, 0x0F,
    0x7784, 0x0A,
    0x7785, 0x0A,
    0x7786, 0x0A,
    0x7787, 0x0F,
    0x7788, 0x0A,
    0x7789, 0x0A,
    0x778A, 0x0A,
    0x778B, 0x0A,
    0x778C, 0x0A,
    0x778D, 0x0A,
    0x778E, 0x14,
    0x778F, 0x0A,
    0x7790, 0x0A,
    0x7791, 0x0A,
    0x7792, 0x0F,
    0x7793, 0x0A,
    0x7794, 0x0A,
    0x7795, 0x0A,
    0x7796, 0x0A,
    0x7797, 0x0A,
    0x7798, 0x0A,
    0x7799, 0x14,
    0x779A, 0x0A,
    0x779B, 0x0A,
    0x779C, 0x0A,
    0x779D, 0x0F,
    0x779E, 0x0A,
    0x779F, 0x0A,
    0x77A0, 0x0A,
    0x77A1, 0x0A,
    0x77A2, 0x0A,
    0x77A3, 0x0A,
    0x77A4, 0x0A,
    0x77A5, 0x0A,
    0x77A6, 0x0A,
    0x77A7, 0x0A,
    0x77A8, 0x0A,
    0x77A9, 0x0A,
    0x77AA, 0x0A,
    0x77AB, 0x0A,
    0x77AC, 0x0A,
    0x77AD, 0x0B,
    0x77AE, 0x0A,
    0x77AF, 0x0A,
    0x77B0, 0x0B,
    0x77B1, 0x0B,
    0x77B2, 0x0A,
    0x77B3, 0x0A,
    0x77B4, 0x0B,
    0x77B5, 0x0B,
    0x77F4, 0x01,
    0x77F5, 0x01,
    0x77F6, 0x01,
    0x77F8, 0x01,
    0x77F9, 0x01,
    0x77FB, 0x01,
    0x77FC, 0x01,
    0x7814, 0x2A,
    0x7815, 0x01,
    0x7816, 0x28,
    0x7818, 0x2A,
    0x7819, 0x28,
    0x781A, 0x01,
    0x781C, 0x28,
    0x781D, 0x28,
    0x781E, 0x28,
    0x781F, 0x2A,
    0x7820, 0x01,
    0x7821, 0x28,
    0x7823, 0x2A,
    0x7824, 0x28,
    0x7825, 0x01,
    0x7827, 0x28,
    0x7828, 0x28,
    0x7829, 0x28,
    0x782A, 0x2A,
    0x782B, 0x0A,
    0x782C, 0x28,
    0x782E, 0x2A,
    0x782F, 0x28,
    0x7830, 0x0A,
    0x7832, 0x28,
    0x7833, 0x28,
    0x7834, 0x28,
    0x7835, 0x2A,
    0x7836, 0x19,
    0x7837, 0x2A,
    0x7838, 0x2A,
    0x7839, 0x2A,
    0x783A, 0x2A,
    0x783B, 0x19,
    0x783C, 0x2A,
    0x783D, 0x2A,
    0x783E, 0x2A,
    0x783F, 0x2A,
    0x7840, 0x1D,
    0x7841, 0x2A,
    0x7842, 0x2A,
    0x7843, 0x2A,
    0x7844, 0x1D,
    0x7845, 0x2A,
    0x7846, 0x2A,
    0x7847, 0x2A,
    0x7848, 0x2A,
    0x7849, 0x2A,
    0x784A, 0x2A,
    0x784B, 0x2A,
    0x784C, 0x2A,
    0x784D, 0x2A,
    0x784E, 0x2A,
    0x784F, 0x2A,
    0x7850, 0x2A,
    0x7851, 0x2A,
    0x7853, 0x50,
    0x7856, 0x50,
    0x7857, 0x50,
    0x785A, 0x50,
    0x785B, 0x50,
    0x785D, 0x50,
    0x7860, 0x50,
    0x7861, 0x50,
    0x7864, 0x50,
    0x7865, 0x50,
    0x7867, 0x50,
    0x786A, 0x50,
    0x786B, 0x50,
    0x7902, 0x15,
    0x7904, 0x13,
    0x7905, 0x15,
    0x7908, 0x13,
    0x7909, 0x08,
    0x790B, 0x11,
    0x790D, 0x0C,
    0x790E, 0x08,
    0x790F, 0x08,
    0x7912, 0x0B,
    0x7919, 0x06,
    0x791E, 0x06,
    0x791F, 0x06,
    0x7929, 0x00,
    0x792E, 0x00,
    0x792F, 0x00,
    0x7A2A, 0x19,
    0x7A2B, 0xF1,
    0x7A2C, 0x0F,
    0x7A48, 0x0F,
    0x7A49, 0x01,
    0x7A4A, 0x03,
    0x7A51, 0x09,
    0x7A58, 0x0F,
    0x7A5D, 0x32,
    0x7A63, 0x32,
    0x7A6B, 0x3E,
    0x7A71, 0x3E,
    0x7AAD, 0x99,
    0x7AB3, 0x99,
    0x7ABB, 0xA5,
    0x7AC1, 0xA5,
    0x7AFD, 0x66,
    0x7B05, 0x72,
    0x7B26, 0x12,
    0x7B27, 0x12,
    0x7B28, 0x12,
    0x7B29, 0x12,
    0x7B2A, 0x12,
    0x7B2B, 0x12,
    0x7B2C, 0x12,
    0x7B2D, 0x12,
    0x7B2E, 0x12,
    0x7B2F, 0x12,
    0x7B30, 0x12,
    0x7B31, 0x12,
    0x7B32, 0x12,
    0x7B33, 0x12,
    0x7B34, 0x12,
    0x7B35, 0x12,
    0x7B36, 0x12,
    0x7B37, 0x12,
    0x7B38, 0x12,
    0x7B39, 0x12,
    0x7B3A, 0x12,
    0x7B3B, 0x12,
    0x7B3C, 0x12,
    0x7B3D, 0x12,
    0x7B3E, 0x12,
    0x7B3F, 0x12,
    0x7B40, 0x12,
    0x7B41, 0x12,
    0x7B42, 0x12,
    0x7B43, 0x12,
    0x7B44, 0x12,
    0x7B45, 0x12,
    0x7B46, 0x12,
    0x7B47, 0x12,
    0x7B48, 0x12,
    0x7B49, 0x12,
    0x7B4A, 0x12,
    0x7B4B, 0x12,
    0x7B4C, 0x12,
    0x7B4D, 0x12,
    0x7C73, 0x0B,
    0x7C74, 0x0B,
    0x7C75, 0x0B,
    0x7C76, 0x0B,
    0x7C77, 0x0B,
    0x7C78, 0x0B,
    0x7C79, 0x0B,
    0x7C7C, 0x0B,
    0x7C7D, 0x0B,
    0x7C7E, 0x0B,
    0x7C7F, 0x0B,
    0x7C80, 0x0B,
    0x7C81, 0x0B,
    0x7C83, 0x0B,
    0x7C84, 0x0B,
    0x7C85, 0x0B,
    0x7C86, 0x0B,
    0x7C87, 0x0B,
    0x7C88, 0x0B,
    0x7CA5, 0x01,
    0x7CAA, 0x01,
    0x7CB0, 0x01,
    0x7CB5, 0x01,
    0x7CBB, 0x01,
    0x7CC0, 0x01,
    0x7CD1, 0x01,
    0x7CD6, 0x01,
    0x7CDC, 0x01,
    0x7CE1, 0x01,
    0x7CE7, 0x01,
    0x7CEC, 0x01,
    0x90B3, 0x80,
    0x974A, 0x09,
    0x974B, 0x08,
    0x9752, 0x0E,
    0x9753, 0xB4,
    0x975B, 0x0C,
    0x9762, 0x09,
    0x9763, 0x08,
    0x976A, 0x09,
    0x976B, 0x08,
    0x9772, 0x0F,
    0x9773, 0x7C,
    0x978A, 0x09,
    0x978B, 0xE8,
    0xDDA9, 0x4E,
    0xDE8D, 0x01,
    0xE24E, 0x01,
    0xAE24, 0x01,
    0xAE25, 0x61,
    0xAE26, 0x01,
    0xAE27, 0xDF,
    0xAE28, 0x02,
    0xAE29, 0xD0,
    0xAB55, 0x23,
    0xAB57, 0x23,
    0xAB59, 0x23,
    0xAB5B, 0x0A,
    0xAB5D, 0x0A,
    0xAB5F, 0x0A,
    0xAB73, 0x23,
    0xAB75, 0x23,
    0xAB77, 0x23,
    0xAB79, 0x0A,
    0xAB7B, 0x0A,
    0xAB7D, 0x0A,
    0x0D08, 0x00,
};

static kal_uint16 imx882pd_preview_setting[] = {
//3Lane
//reg_A
//QBIN(VBIN)_4096x3072_30FPS with PDAF VB_max
//H: 4096
//V: 3072
//MIPI output setting
//Address   value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
//Line Length PCK Setting
//Address   value
    0x0342, 0x1D,
    0x0343, 0x4C,
    0x3850, 0x00,
    0x3851, 0xCD,
//Frame Length Lines Setting
//Address   value
    0x0340, 0x0F,
    0x0341, 0x3C,
//ROI Setting
//Address   value
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x17,
    0x034B, 0xFF,
//Mode Setting
//Address   value
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x3005, 0x02,
    0x3006, 0x02,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x04,
    0x31C0, 0x41,
    0x31C1, 0x41,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
//Digital Crop & Scaling
//Address   value
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x10,
    0x040D, 0x00,
    0x040E, 0x0C,
    0x040F, 0x00,
//Output Size Setting
//Address   value
    0x034C, 0x10,
    0x034D, 0x00,
    0x034E, 0x0C,
    0x034F, 0x00,
//Clock Setting
//Address   value
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xFF,
//Other Setting
//Address   value
    0x3104, 0x01,
//Integration Setting
//Address   value
    0x0202, 0x03,
    0x0203, 0xE8,
//Gain Setting
//Address   value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
//PDAF TYPE2 Setting
//Address   value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
//PHASE PIX VCID Setting
//Address   value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
//PHASE PIX data type Setting
//Address   value
    0x30A5, 0x30,
    0x30A7, 0x30,
//PDAF TYPE2 VCID Setting
//Address   value
    0x30A2, 0x00,
    0x30F1, 0x01,
//PDAF TYPE2 data type Setting
//Address   value
    0x30A3, 0x30,
//MIPI Global Timing Setting
//Address   value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
    //IQ TUNING SETTING
    0x38A0, 0x01,
    0x38A1, 0x38,
    0x38A2, 0x01,
    0x38A3, 0x38,
    0x08D0, 0x01,
    0x38D1, 0x2C,
    0x3B00, 0x02,
    0x3B01, 0x88,
    0x38E0, 0x01,
    0x38E1, 0xEA,
};

static kal_uint16 imx882pd_capture_setting[] = {
//3Lane
//reg_A
//QBIN(VBIN)_4096x3072_30FPS with PDAF VB_max
//H: 4096
//V: 3072
//MIPI output setting
//Address	value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address	value
    0x0342, 0x1D,
    0x0343, 0x4C,
    0x3850, 0x00,
    0x3851, 0xCD,
    //Frame Length Lines Setting
    //Address	value
    0x0340, 0x0F,
    0x0341, 0x3C,
    //ROI Setting
    //Address	value
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x17,
    0x034B, 0xFF,
    //Mode Setting
    //Address	value
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x3005, 0x02,
    0x3006, 0x02,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x04,
    0x31C0, 0x41,
    0x31C1, 0x41,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
    //Digital Crop & Scaling
    //Address	value
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x10,
    0x040D, 0x00,
    0x040E, 0x0C,
    0x040F, 0x00,
    //Output Size Setting
    //Address	value
    0x034C, 0x10,
    0x034D, 0x00,
    0x034E, 0x0C,
    0x034F, 0x00,
    //Clock Setting
    //Address	value
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xFF,
    //Other Setting
    //Address	value
    0x3104, 0x01,
    //Integration Setting
    //Address	value
    0x0202, 0x03,
    0x0203, 0xE8,
    //Gain Setting
    //Address	value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
    //PDAF TYPE2 Setting
    //Address	value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
    //PHASE PIX VCID Setting
    //Address	value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address	value
    0x30A5, 0x30,
    0x30A7, 0x30,
    //PDAF TYPE2 VCID Setting
    //Address	value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address	value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address	value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
    //IQ SETTING
    0x38A0, 0x01,
	0x38A1, 0x38,
	0x38A2, 0x01,
	0x38A3, 0x38,
	0x38D0, 0x01,
	0x38D1, 0x2C,
	0x3B00, 0x02,
	0x3B01, 0x88,
	0x38E0, 0x01,
	0x38E1, 0xEA,
};

static kal_uint16 imx882pd_normal_video_setting[] = {
//3Lane
//reg_B
//QBIN(VBIN)_4096x2304 @30FPS with PDAF VB_max
//H: 4096
//V: 2304
//min shutter 8
//step 2
//margin 48
//MIPI output setting
    //Address	value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address	value
    0x0342, 0x1D,
    0x0343, 0x4C,
    0x3850, 0x00,
    0x3851, 0xCD,
    //Frame Length Lines Setting
    //Address	value
    0x0340, 0x0F,
    0x0341, 0x3C,
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
    0x0902, 0x00,
    0x3005, 0x02,
    0x3006, 0x02,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x04,
    0x31C0, 0x41,
    0x31C1, 0x41,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
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
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xFF,
    //Other Setting
    //Address	value
    0x3104, 0x01,
    //Integration Setting
    //Address	value
    0x0202, 0x03,
    0x0203, 0xE8,
    //Gain Setting
    //Address	value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
    //PDAF TYPE2 Setting
    //Address	value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
    //PHASE PIX VCID Setting
    //Address	value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address	value
    0x30A5, 0x30,
    0x30A7, 0x30,
    //PDAF TYPE2 VCID Setting
    //Address	value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address	value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address	value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
    };

    static kal_uint16 imx882pd_hs_video_setting[] = {
    //3Lane
    //reg_C-4
    //QBIN(VBIN)-V2H2 2048x1152_120FPS with PDAF VB_max
    //H: 2048
    //V: 1152
    //min shutter 9
    //step 1
    //margin 48
    //MIPI output setting
    //Address	value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address	value
    0x0342, 0x0F,
    0x0343, 0xB8,
    0x3850, 0x00,
    0x3851, 0x6E,
    //Frame Length Lines Setting
    //Address	value
    0x0340, 0x07,
    0x0341, 0x18,
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
    0x0901, 0x44,
    0x0902, 0x02,
    0x3005, 0x02,
    0x3006, 0x02,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x04,
    0x31C0, 0x43,
    0x31C1, 0x43,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
    //Digital Crop & Scaling
    //Address	value
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x08,
    0x040D, 0x00,
    0x040E, 0x04,
    0x040F, 0x80,
    //Output Size Setting
    //Address	value
    0x034C, 0x08,
    0x034D, 0x00,
    0x034E, 0x04,
    0x034F, 0x80,
    //Clock Setting
    //Address	value
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x02,
    0x030F, 0x1D,
    //Other Setting
    //Address	value
    0x3104, 0x01,
    //Integration Setting
    //Address	value
    0x0202, 0x03,
    0x0203, 0xE8,
    //Gain Setting
    //Address	value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
    //PDAF TYPE2 Setting
    //Address	value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
    //EAE-Bracketing Setting
    //0x0e00, 0x00,
    //PHASE PIX VCID Setting
    //Address	value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address	value
    0x30A5, 0x30,
    0x30A7, 0x30,
    //PDAF TYPE2 VCID Setting
    //Address	value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address	value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address	value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
};

static kal_uint16 imx882pd_slim_video_setting[] = {
//3Lane
//reg_A
//QBIN(VBIN)_4096x3072_30FPS with PDAF VB_max
//H: 4096
//V: 3072
//MIPI output setting
//Address   value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
//Line Length PCK Setting
//Address   value
    0x0342, 0x1D,
    0x0343, 0x4C,
    0x3850, 0x00,
    0x3851, 0xCD,
//Frame Length Lines Setting
//Address   value
    0x0340, 0x0F,
    0x0341, 0x3C,
//ROI Setting
//Address   value
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x17,
    0x034B, 0xFF,
//Mode Setting
//Address   value
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x3005, 0x02,
    0x3006, 0x02,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x04,
    0x31C0, 0x41,
    0x31C1, 0x41,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
//Digital Crop & Scaling
//Address   value
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x10,
    0x040D, 0x00,
    0x040E, 0x0C,
    0x040F, 0x00,
//Output Size Setting
//Address   value
    0x034C, 0x10,
    0x034D, 0x00,
    0x034E, 0x0C,
    0x034F, 0x00,
//Clock Setting
//Address   value
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xFF,
//Other Setting
//Address   value
    0x3104, 0x01,
//Integration Setting
//Address   value
    0x0202, 0x03,
    0x0203, 0xE8,
//Gain Setting
//Address   value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
//PDAF TYPE2 Setting
//Address   value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
//PHASE PIX VCID Setting
//Address   value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
//PHASE PIX data type Setting
//Address   value
    0x30A5, 0x30,
    0x30A7, 0x30,
//PDAF TYPE2 VCID Setting
//Address   value
    0x30A2, 0x00,
    0x30F1, 0x01,
//PDAF TYPE2 data type Setting
//Address   value
    0x30A3, 0x30,
//MIPI Global Timing Setting
//Address   value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
};

static kal_uint16 imx882pd_custom1_setting[] = {
//3Lane
//reg_A-1
//QBIN(VBIN)_4096x3072_24FPS with PDAF VB_max
//H: 4096
//V: 3072
//min shutter 8
//step 2
//margin 48
//MIPI output setting
    //Address	value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address	value
    0x0342, 0x1D,
    0x0343, 0x4C,
    0x3850, 0x00,
    0x3851, 0xCD,
    //Frame Length Lines Setting
    //Address	value
    0x0340, 0x13,
    0x0341, 0x0C,
    //ROI Setting
    //Address	value
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x17,
    0x034B, 0xFF,
    //Mode Setting
    //Address	value
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x3005, 0x02,
    0x3006, 0x02,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x04,
    0x31C0, 0x41,
    0x31C1, 0x41,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
    //Digital Crop & Scaling
    //Address	value
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x10,
    0x040D, 0x00,
    0x040E, 0x0C,
    0x040F, 0x00,
    //Output Size Setting
    //Address	value
    0x034C, 0x10,
    0x034D, 0x00,
    0x034E, 0x0C,
    0x034F, 0x00,
    //Clock Setting
    //Address	value
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xFF,
    //Other Setting
    //Address	value
    0x3104, 0x01,
    //Integration Setting
    //Address	value
    0x0202, 0x03,
    0x0203, 0xE8,
    //Gain Setting
    //Address	value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
    //PDAF TYPE2 Setting
    //Address	value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
    //EAE-Bracketing Setting
    //0x0e00, 0x00,
    //PHASE PIX VCID Setting
    //Address	value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address	value
    0x30A5, 0x30,
    0x30A7, 0x30,
    //PDAF TYPE2 VCID Setting
    //Address	value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address	value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address	value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
};

static kal_uint16 imx882pd_custom2_setting[] = {
//3Lane
//reg_B-1
//QBIN_4096x2304 @60FPS with PDAF VB_max
//H: 4096
//V: 2304
//min shutter 8
//step 2
//margin 48
//MIPI output setting
    //Address	value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address	value
    0x0342, 0x12,
    0x0343, 0x08,
    0x3850, 0x00,
    0x3851, 0x7F,
    //Frame Length Lines Setting
    //Address	value
    0x0340, 0x0C,
    0x0341, 0x4D,
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
    0x0902, 0x00,
    0x3005, 0x06,
    0x3006, 0x01,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x00,
    0x31C0, 0x41,
    0x31C1, 0x41,
    0x3205, 0x00,
    0x323C, 0x02,
    0x39AC, 0x01,
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
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x01,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0x87,
    //Other Setting
    //Address	value
    0x3104, 0x00,
    0x38A0, 0x00,
    0x38A1, 0x00,
    0x38A2, 0x00,
    0x38A3, 0x00,
    0x38A8, 0x00,
    0x38A9, 0x00,
    0x38AA, 0x00,
    0x38AB, 0x00,
    0x38B0, 0x03,
    0x38B1, 0xFF,
    0x38B4, 0x03,
    0x38B5, 0xFF,
    0x38B8, 0x03,
    0x38B9, 0xFF,
    0x38BC, 0x03,
    0x38BD, 0xFF,
    0x38D0, 0x06,
    0x38D1, 0x0E,
    0x38D2, 0x06,
    0x38D3, 0x0E,
    0x38D8, 0x14,
    0x38E0, 0x00,
    0x38E1, 0x00,
    0x38E2, 0x00,
    0x38E3, 0x00,
    0x38E4, 0x00,
    0x38E5, 0x00,
    0x38E6, 0x00,
    0x38E7, 0x00,
    0x3B00, 0x00,
    0x3B01, 0x00,
    0x3B04, 0x00,
    0x3B05, 0x00,
    //Integration Setting
    //Address	value
    0x0202, 0x03,
    0x0203, 0xE8,
    //Gain Setting
    //Address	value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
    //PDAF TYPE2 Setting
    //Address	value
    0x3103, 0x01,
    0x3422, 0x01,
    0x3423, 0xFC,
    //EAE-Bracketing Setting
    //Address    value
    0x0E00, 0x00,
    //PHASE PIX VCID Setting
    //Address	value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address	value
    0x30A5, 0x30,
    0x30A7, 0x30,
    //PDAF TYPE2 VCID Setting
    //Address	value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address	value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address	value
    0x084E, 0x00,
    0x084F, 0x0D,
    0x0850, 0x00,
    0x0851, 0x0B,
    0x0852, 0x00,
    0x0853, 0x17,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
    };

    static kal_uint16 imx882pd_custom3_setting[] = {
    //3Lane
    //reg_D
    //Full RMSC 15fps with PDAF VB_max
    //H: 8192
    //V: 6144
    //min shutter 9
    //step 1
    //margin 48
    //MIPI output setting
    //Address	value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address	value
    0x0342, 0x23,
    0x0343, 0x00,
    0x3850, 0x00,
    0x3851, 0xF6,
    //Frame Length Lines Setting
    //Address	value
    0x0340, 0x19,
    0x0341, 0x68,
    //ROI Setting
    //Address	value
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x17,
    0x034B, 0xFF,
    //Mode Setting
    //Address	value
    0x0900, 0x00,
    0x0901, 0x11,
    0x0902, 0x00,
    0x3005, 0x00,
    0x3006, 0x00,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x00,
    0x31C0, 0x01,
    0x31C1, 0x01,
    0x3205, 0x01,
    0x323C, 0x01,
    0x39AC, 0x01,
    //Digital Crop & Scaling
    //Address	value
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x20,
    0x040D, 0x00,
    0x040E, 0x18,
    0x040F, 0x00,
    //Output Size Setting
    //Address	value
    0x034C, 0x20,
    0x034D, 0x00,
    0x034E, 0x18,
    0x034F, 0x00,
    //Clock Setting
    //Address	value
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x01,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0x96,
    //Other Setting
    //Address	value
    0x3104, 0x01,
    //Integration Setting
    //Address	value
    0x0202, 0x03,
    0x0203, 0xE8,
    //Gain Setting
    //Address	value
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    //PDAF TYPE2 Setting
    //Address	value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
    //EAE-Bracketing Setting
    //    //0x0e00, 0x00,
    //PHASE PIX VCID Setting
    //Address	value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address	value
    0x30A5, 0x30,
    0x30A7, 0x30,
    //PDAF TYPE2 VCID Setting
    //Address	value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address	value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address	value
    0x084E, 0x00,
    0x084F, 0x0F,
    0x0850, 0x00,
    0x0851, 0x0D,
    0x0852, 0x00,
    0x0853, 0x19,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
};

static kal_uint16 imx882pd_custom4_setting[] = {
//3Lane
//reg_C-5
//QBIN-V2H2 2048x1152_240FPS w/o PDAF VB_max
//H: 2048
//V: 1152
//min shutter 9
//step 1
//margin 48
//MIPI output setting
    //Address	value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address	value
    0x0342, 0x09,
    0x0343, 0xA4,
    0x3850, 0x00,
    0x3851, 0x44,
    //Frame Length Lines Setting
    //Address	value
    0x0340, 0x05,
    0x0341, 0xBE,
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
    0x0901, 0x44,
    0x0902, 0x02,
    0x3005, 0x00,
    0x3006, 0x00,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x00,
    0x31C0, 0x43,
    0x31C1, 0x43,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
    //Digital Crop & Scaling
    //Address	value
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x08,
    0x040D, 0x00,
    0x040E, 0x04,
    0x040F, 0x80,
    //Output Size Setting
    //Address	value
    0x034C, 0x08,
    0x034D, 0x00,
    0x034E, 0x04,
    0x034F, 0x80,
    //Clock Setting
    //Address	value
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x01,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0x65,
    //Other Setting
    //Address	value
    0x3104, 0x00,
    //Integration Setting
    //Address	value
    0x0202, 0x03,
    0x0203, 0xE8,
    //Gain Setting
    //Address	value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
    //PDAF TYPE2 Setting
    //Address	value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
    //PHASE PIX VCID Setting
    //Address	value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address	value
    0x30A5, 0x30,
    0x30A7, 0x30,
    //PDAF TYPE2 VCID Setting
    //Address	value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address	value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address	value
    0x084E, 0x00,
    0x084F, 0x0D,
    0x0850, 0x00,
    0x0851, 0x0B,
    0x0852, 0x00,
    0x0853, 0x17,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
    };

    static kal_uint16 imx882pd_custom5_setting[] = {
    //3Lane
    //reg_A-3-S1
    //Full-RMSC-Crop_4096x3072_30FPS with PDAF VB_max seamless A-2-S1
    //H: 4096
    //V: 3072
    //min shutter 9
    //step 1
    //margin 48
    //MIPI output setting
    //Address	value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address	value
    0x0342, 0x23,
    0x0343, 0x00,
    0x3850, 0x00,
    0x3851, 0xF6,
    //Frame Length Lines Setting
    //Address	value
    0x0340, 0x0C,
    0x0341, 0xB4,
    //ROI Setting
    //Address	value
    0x0344, 0x08,
    0x0345, 0x00,
    0x0346, 0x06,
    0x0347, 0x00,
    0x0348, 0x17,
    0x0349, 0xFF,
    0x034A, 0x11,
    0x034B, 0xFF,
    //Mode Setting
    //Address	value
    0x0900, 0x00,
    0x0901, 0x11,
    0x0902, 0x00,
    0x3005, 0x00,
    0x3006, 0x00,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x00,
    0x31C0, 0x01,
    0x31C1, 0x01,
    0x3205, 0x01,
    0x323C, 0x01,
    0x39AC, 0x01,
    //Digital Crop & Scaling
    //Address	value
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x10,
    0x040D, 0x00,
    0x040E, 0x0C,
    0x040F, 0x00,
    //Output Size Setting
    //Address	value
    0x034C, 0x10,
    0x034D, 0x00,
    0x034E, 0x0C,
    0x034F, 0x00,
    //Clock Setting
    //Address	value
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xFF,
    //Other Setting
    //Address	value
    0x3104, 0x01,
    //Integration Setting
    //Address	value
    0x0202, 0x03,
    0x0203, 0xE8,
    //Gain Setting
    //Address	value
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    //EAE-Bracketing Setting
    //0x0e00, 0x00,
    //PDAF TYPE2 Setting
    //Address	value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
    //PHASE PIX VCID Setting
    //Address	value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address	value
    0x30A5, 0x30,
    0x30A7, 0x30,
    //PDAF TYPE2 VCID Setting
    //Address	value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address	value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address	value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,  
    };

    static kal_uint16 imx882pd_custom6_setting[] = {
    //3Lane
    //reg_C-6
    //QBIN-V2H2 1280x674_480FPS w/o PDAF VB_max
    //H: 1280
    //V: 674
    //min shutter 9
    //step 1
    //margin 48
    //MIPI output setting
    //Address	value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address	value
    0x0342, 0x09,
    0x0343, 0xA4,
    0x3850, 0x00,
    0x3851, 0x44,
    //Frame Length Lines Setting
    //Address	value
    0x0340, 0x02,
    0x0341, 0xDE,
    //ROI Setting
    //Address	value
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x06,
    0x0347, 0xA0,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x11,
    0x034B, 0x3F,
    //Mode Setting
    //Address	value
    0x0900, 0x01,
    0x0901, 0x44,
    0x0902, 0x02,
    0x3005, 0x00,
    0x3006, 0x00,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x00,
    0x31C0, 0x43,
    0x31C1, 0x43,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
    //Digital Crop & Scaling
    //Address	value
    0x0408, 0x01,
    0x0409, 0x80,
    0x040A, 0x00,
    0x040B, 0x06,
    0x040C, 0x05,
    0x040D, 0x00,
    0x040E, 0x02,
    0x040F, 0xA2,
    //Output Size Setting
    //Address	value
    0x034C, 0x05,
    0x034D, 0x00,
    0x034E, 0x02,
    0x034F, 0xA2,
    //Clock Setting
    //Address	value
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xFA,
    //Other Setting
    //Address	value
    0x3104, 0x00,
    //Integration Setting
    //Address	value
    0x0202, 0x02,
    0x0203, 0xA6,
    //Gain Setting
    //Address	value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
    //EAE-Bracketing Setting
    //0x0e00, 0x00,
    //PDAF TYPE2 Setting
    //Address	value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
    //PHASE PIX VCID Setting
    //Address	value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address	value
    0x30A5, 0x30,
    0x30A7, 0x30,
    //PDAF TYPE2 VCID Setting
    //Address	value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address	value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address	value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
};
static kal_uint16 imx882pd_custom7_setting[] = {
//3Lane
//reg_A
//QBIN(VBIN)_4096x3072_30FPS with PDAF VB_max
//H: 4096
//V: 3072
//MIPI output setting
//Address   value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
//Line Length PCK Setting
//Address   value
    0x0342, 0x1D,
    0x0343, 0x4C,
    0x3850, 0x00,
    0x3851, 0xCD,
//Frame Length Lines Setting
//Address   value
    0x0340, 0x0F,
    0x0341, 0x3C,
//ROI Setting
//Address   value
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x17,
    0x034B, 0xFF,
//Mode Setting
//Address   value
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x3005, 0x02,
    0x3006, 0x02,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x04,
    0x31C0, 0x41,
    0x31C1, 0x41,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
//Digital Crop & Scaling
//Address   value
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x10,
    0x040D, 0x00,
    0x040E, 0x0C,
    0x040F, 0x00,
//Output Size Setting
//Address   value
    0x034C, 0x10,
    0x034D, 0x00,
    0x034E, 0x0C,
    0x034F, 0x00,
//Clock Setting
//Address   value
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xFF,
//Other Setting
//Address   value
    0x3104, 0x01,
//Integration Setting
//Address   value
    0x0202, 0x03,
    0x0203, 0xE8,
//Gain Setting
//Address   value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
//PDAF TYPE2 Setting
//Address   value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
//PHASE PIX VCID Setting
//Address   value
    0x30A4, 0x03,
    0x30A6, 0x06,
    0x30F2, 0x01,
    0x30F3, 0x01,
//PHASE PIX data type Setting
//Address   value
    0x30A5, 0x2B,
    0x30A7, 0x2B,
//PDAF TYPE2 VCID Setting
//Address   value
    0x30A2, 0x00,
    0x30F1, 0x01,
//PDAF TYPE2 data type Setting
//Address   value
    0x30A3, 0x30,
//MIPI Global Timing Setting
//Address   value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
};

static kal_uint16 imx882pd_custom8_setting[] = {
//3Lane
//reg_B
//QBIN(VBIN)_4096x2304 @30FPS with PDAF VB_max
//H: 4096
//V: 2304
//min shutter 8
//step 2
//margin 48
//MIPI output setting
    //Address	value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address	value
    0x0342, 0x1D,
    0x0343, 0x4C,
    0x3850, 0x00,
    0x3851, 0xCD,
    //Frame Length Lines Setting
    //Address	value
    0x0340, 0x0F,
    0x0341, 0x3C,
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
    0x0902, 0x00,
    0x3005, 0x02,
    0x3006, 0x02,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x04,
    0x31C0, 0x41,
    0x31C1, 0x41,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
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
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xFF,
    //Other Setting
    //Address	value
    0x3104, 0x01,
    //Integration Setting
    //Address	value
    0x0202, 0x03,
    0x0203, 0xE8,
    //Gain Setting
    //Address	value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
    //PDAF TYPE2 Setting
    //Address	value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
    //PHASE PIX VCID Setting
    //Address	value
    0x30A4, 0x03,
    0x30A6, 0x06,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address	value
    0x30A5, 0x2B,
    0x30A7, 0x2B,
    //PDAF TYPE2 VCID Setting
    //Address	value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address	value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address	value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
    };
    static kal_uint16 imx882pd_custom9_setting[] = {
    //3Lane
    //reg_C-3
    //QBIN(VBIN)-V2H2 2048x1536_24FPS with PDAF VB_max
    //H: 2048
    //V: 1536
    //min shutter 9
    //step 1
    //margin 48
    //MIPI output setting
    //Address	value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address	value
    0x0342, 0x0F,
    0x0343, 0xB8,
    0x3850, 0x00,
    0x3851, 0x6E,
    //Frame Length Lines Setting
    //Address	value
    0x0340, 0x23,
    0x0341, 0x80,
    //ROI Setting
    //Address	value
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x17,
    0x034B, 0xFF,
    //Mode Setting
    //Address	value
    0x0900, 0x01,
    0x0901, 0x44,
    0x0902, 0x02,
    0x3005, 0x02,
    0x3006, 0x02,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x04,
    0x31C0, 0x43,
    0x31C1, 0x43,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
    //Digital Crop & Scaling
    //Address	value
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x08,
    0x040D, 0x00,
    0x040E, 0x06,
    0x040F, 0x00,
    //Output Size Setting
    //Address	value
    0x034C, 0x08,
    0x034D, 0x00,
    0x034E, 0x06,
    0x034F, 0x00,
    //Clock Setting
    //Address	value
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x01,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0x0F,
    //Other Setting
    //Address	value
    0x3104, 0x01,
    //Integration Setting
    //Address	value
    0x0202, 0x03,
    0x0203, 0xE8,
    //Gain Setting
    //Address	value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
    //EAE-Bracketing Setting
    //0x0e00, 0x00,
    //PDAF TYPE2 Setting
    //Address	value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
    //PHASE PIX VCID Setting
    //Address	value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address	value
    0x30A5, 0x30,
    0x30A7, 0x30,
    //PDAF TYPE2 VCID Setting
    //Address	value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address	value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address	value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
};

static kal_uint16 imx882pd_custom10_setting[] = {
//3Lane
//reg_B
//QBIN(VBIN)_4096x2304 @30FPS with PDAF VB_max
//H: 4096
//V: 2304
//min shutter 8
//step 2
//margin 48
//MIPI output setting
    //Address	value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address	value
    0x0342, 0x1D,
    0x0343, 0x4C,
    0x3850, 0x00,
    0x3851, 0xCD,
    //Frame Length Lines Setting
    //Address	value
    0x0340, 0x0F,
    0x0341, 0x3C,
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
    0x0902, 0x00,
    0x3005, 0x02,
    0x3006, 0x02,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x04,
    0x31C0, 0x41,
    0x31C1, 0x41,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
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
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xFF,
    //Other Setting
    //Address	value
    0x3104, 0x01,
    //Integration Setting
    //Address	value
    0x0202, 0x03,
    0x0203, 0xE8,
    //Gain Setting
    //Address	value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
    //PDAF TYPE2 Setting
    //Address	value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
    //PHASE PIX VCID Setting
    //Address	value
    0x30A4, 0x03,
    0x30A6, 0x06,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address	value
    0x30A5, 0x2B,
    0x30A7, 0x2B,
    //PDAF TYPE2 VCID Setting
    //Address	value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address	value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address	value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
};

static void sensor_init(void)
{
    LOG_INF("sensor_init start\n");
    LOG_INF("E imx882_hw_version(%d)\n", imx882_hw_version);
    imx882pd_table_write_cmos_sensor(imx882pd_init_setting,
        sizeof(imx882pd_init_setting)/sizeof(kal_uint16));
    set_mirror_flip(imgsensor.mirror);
    LOG_INF("sensor_init End\n");
}    /*sensor_init  */

static void preview_setting(void)
{
    LOG_INF("%s start\n", __func__);
    imx882pd_table_write_cmos_sensor(imx882pd_preview_setting,
        sizeof(imx882pd_preview_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
} /* preview_setting */


/*full size 30fps*/
static void capture_setting(kal_uint16 currefps)
{
    LOG_INF("%s start\n", __func__);
    imx882pd_table_write_cmos_sensor(imx882pd_capture_setting,
        sizeof(imx882pd_capture_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void normal_video_setting(kal_uint16 currefps)
{
    LOG_INF("%s start\n", __func__);
    imx882pd_table_write_cmos_sensor(imx882pd_normal_video_setting,
        sizeof(imx882pd_normal_video_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void hs_video_setting(void)
{
    LOG_INF("%s start\n", __func__);
    imx882pd_table_write_cmos_sensor(imx882pd_hs_video_setting,
        sizeof(imx882pd_hs_video_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void slim_video_setting(void)
{
    LOG_INF("%s start\n", __func__);
    imx882pd_table_write_cmos_sensor(imx882pd_slim_video_setting,
        sizeof(imx882pd_slim_video_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom1_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx882pd_table_write_cmos_sensor(imx882pd_custom1_setting,
        sizeof(imx882pd_custom1_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom2_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx882pd_table_write_cmos_sensor(imx882pd_custom2_setting,
        sizeof(imx882pd_custom2_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom3_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx882pd_table_write_cmos_sensor(imx882pd_custom3_setting,
        sizeof(imx882pd_custom3_setting)/sizeof(kal_uint16));
    if (Eeprom_1ByteDataRead(0x1418, 0xA8) == 1) {
        pr_info("OTP QSC Data Valid, enable qsc register");
    } else {
        pr_info("OTP no QSC Data, close qsc register");
        write_cmos_sensor_8(0x3621, 0x00);
    }
    LOG_INF("%s end\n", __func__);
}

static void custom4_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx882pd_table_write_cmos_sensor(imx882pd_custom4_setting,
        sizeof(imx882pd_custom4_setting)/sizeof(kal_uint16));
     LOG_INF("%s end\n", __func__);
}

static void custom5_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx882pd_table_write_cmos_sensor(imx882pd_custom5_setting,
        sizeof(imx882pd_custom5_setting)/sizeof(kal_uint16));
     LOG_INF("%s end\n", __func__);
}

static void custom6_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx882pd_table_write_cmos_sensor(imx882pd_custom6_setting,
        sizeof(imx882pd_custom6_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom7_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx882pd_table_write_cmos_sensor(imx882pd_custom7_setting,
        sizeof(imx882pd_custom7_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom8_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx882pd_table_write_cmos_sensor(imx882pd_custom8_setting,
        sizeof(imx882pd_custom8_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}
static void custom9_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx882pd_table_write_cmos_sensor(imx882pd_custom9_setting,
        sizeof(imx882pd_custom9_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}
static void custom10_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx882pd_table_write_cmos_sensor(imx882pd_custom10_setting,
        sizeof(imx882pd_custom10_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void hdr_write_double_shutter_w_gph(kal_uint16 le, kal_uint16 se, kal_bool gph)
{
    kal_uint16 realtime_fps = 0;
    kal_uint16 exposure_cnt = 0;

    if (le)
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

    if (le) {
        le = round_up(le/exposure_cnt, 4);
    }
    if (se) {
        se = round_up(se/exposure_cnt, 4);
    }

    spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length = max((kal_uint32)((le + se + (exposure_cnt * imgsensor_info.margin)) * exposure_cnt),
                                imgsensor.min_frame_length);
    imgsensor.frame_length = min(imgsensor.frame_length, imgsensor_info.max_frame_length);
    spin_unlock(&imgsensor_drv_lock);

    LOG_INF("E! le:0x%x, se:0x%x autoflicker_en %d frame_length %d\n",
        le, se, imgsensor.autoflicker_en, imgsensor.frame_length);

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

    // Long exposure
    write_cmos_sensor_8(0x0202, (le >> 8) & 0xFF);
    write_cmos_sensor_8(0x0203, le & 0xFF);

    // Short exposure
    write_cmos_sensor_8(0x0224, (se >> 8) & 0xFF);
    write_cmos_sensor_8(0x0225, se & 0xFF);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x00);
    }
    LOG_INF("L! le:0x%x, se:0x%x\n", le, se);
#if 0
    //read back
    le_b = ((read_cmos_sensor_8(0x0202) & 0xff) << 8) |  (read_cmos_sensor_8(0x0203) & 0xff);
    se_b = ((read_cmos_sensor_8(0x0224) & 0xff) << 8) |  (read_cmos_sensor_8(0x0225) & 0xff);
    LOG_INF("read from sensor le: 0x%x(%d), se: 0x%x(%d)", le_b, le_b, se_b, se_b);
#endif
}

static void hdr_write_double_shutter(kal_uint16 le, kal_uint16 se)
{
    LOG_INF("Fine Integ Time = %d", (read_cmos_sensor_8(0x0200) << 8) | read_cmos_sensor_8(0x0201));
    hdr_write_double_shutter_w_gph(le, se, KAL_TRUE);
}

static void hdr_write_double_gain_w_gph(kal_uint16 lgain, kal_uint16 sgain, kal_bool gph)
{
    kal_uint16 reg_lg, reg_sg;
    //kal_uint16 lg_b = 0, sg_b = 0;

    if (lgain < BASEGAIN || lgain > 32 * BASEGAIN) {
        LOG_INF("Error lgain setting");
        if (lgain < BASEGAIN)
            lgain = BASEGAIN;
        else if (lgain > 32 * BASEGAIN)
            lgain = 32 * BASEGAIN;
    }
    if (sgain < BASEGAIN || sgain > 32 * BASEGAIN) {
        LOG_INF("Error sgain setting");
        if (sgain < BASEGAIN)
            sgain = BASEGAIN;
        else if (sgain > 32 * BASEGAIN)
            sgain = 32 * BASEGAIN;
    }

    reg_lg = gain2reg(lgain);
    reg_sg = gain2reg(sgain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_lg;
    spin_unlock(&imgsensor_drv_lock);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x01);
    }
    // Long Gian
    write_cmos_sensor_8(0x0204, (reg_lg>>8) & 0xFF);
    write_cmos_sensor_8(0x0205, reg_lg & 0xFF);

    // Short Gian
    write_cmos_sensor_8(0x0216, (reg_sg>>8) & 0xFF);
    write_cmos_sensor_8(0x0217, reg_sg & 0xFF);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x00);
    }
    LOG_INF("lgain:%d, reg_lg:0x%x, sgain:%d, reg_sg:0x%x\n",
        lgain, reg_lg, sgain, reg_sg);
#if 0
    //read back
    lg_b = ((read_cmos_sensor_8(0x0204) & 0xff) << 8) |  (read_cmos_sensor_8(0x0205) & 0xff);
    sg_b = ((read_cmos_sensor_8(0x0216) & 0xff) << 8) |  (read_cmos_sensor_8(0x0217) & 0xff);
    LOG_INF("read from sensor lg: 0x%x(%d), sg: 0x%x(%d)", lg_b, lg_b, sg_b, sg_b);
#endif
}
static void hdr_write_double_gain(kal_uint16 lgain, kal_uint16 sgain)
{
    hdr_write_double_gain_w_gph(lgain, sgain, KAL_TRUE);
}

static void write_sensor_QSC(void)
{
    kal_uint8 qsc_is_valid = Eeprom_1ByteDataRead(OTP_QSC_OFFSET + QSC_SIZE, IMX882PD_EEPROM_SLAVE_ADDRESS);
    LOG_INF("%s start write_sensor_QSC, qsc_is_valid:%d, qsc_flag:%d\n", __func__, qsc_is_valid, qsc_flag);
    if (qsc_is_valid == 1 && !qsc_flag) {
        imx882pd_burst_write_cmos_sensor(imx882pd_QSC_setting,
            sizeof(imx882pd_QSC_setting) / sizeof(kal_uint16));
        qsc_flag = 1;
    }
    LOG_INF("%s end\n", __func__);

}

static void write_sensor_SPC(void)
{
    kal_uint8 spc_is_valid = Eeprom_1ByteDataRead(OTP_SPC_OFFSET + SPC_SIZE, IMX882PD_EEPROM_SLAVE_ADDRESS);
    LOG_INF("%s start write_sensor_spc, spc_is_valid:%d, spc_flag:%d", __func__, spc_is_valid, spc_flag);
    if (spc_is_valid == 1 && !spc_flag) {
        imx882pd_table_write_cmos_sensor(imx882pd_SPC_setting,
            sizeof(imx882pd_SPC_setting) / sizeof(kal_uint16));
        spc_flag = 1;
    }
    LOG_INF("%s end\n", __func__);
}

static void read_cmos_eeprom_table(kal_uint16 addr, kal_uint8 *table, kal_uint32 size)
{
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd, 2, (u8 *)table, size, EEPROM_I2C_ADDR);
}

static kal_uint16 read_otp_info(kal_uint8 *data)
{
    kal_uint16 addr = 0x00;
    kal_uint16 left_size = 0;

    left_size = LEFT_SIZE;
    while (left_size > 0) {
        if (left_size >= READ_EEPROM_1K) {
            read_cmos_eeprom_table(addr,data, READ_EEPROM_1K);
            left_size = left_size - READ_EEPROM_1K;
        } else {
            read_cmos_eeprom_table(addr,data, left_size);
            break;
        }
        addr = addr + READ_EEPROM_1K;
        data = data + READ_EEPROM_1K;
    }
    return 0;
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
    /*sensor have two i2c address 0x34 & 0x20,
     *we should detect the module used i2c address
     */
    LOG_INF("imx882 Enter %s.", __func__);
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        gImgEepromInfo.i4CurSensorIdx = 0;
        gImgEepromInfo.i4CurSensorId = imgsensor_info.sensor_id;
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = ((read_cmos_sensor_8(0x0016) << 8)
                    | read_cmos_sensor_8(0x0017));
            if (*sensor_id == IMX882_SENSOR_ID || *sensor_id == IMX882_SENSOR_ID2) {
                *sensor_id = imgsensor_info.sensor_id;
                LOG_INF("Read sensor id Success, write id:0x%x, id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
                read_sensor_verison();
                if(first_read){
                    read_imx882pd_23231_SPC(imx882pd_SPC_setting);
                    read_imx882pd_23231_QSC(imx882pd_QSC_setting);
                    first_read = KAL_FALSE;
                }
                if(deviceInfo_register_value == 0x00){
                    Eeprom_DataInit(4, IMX882PD_SENSOR_ID23231);
                    deviceInfo_register_value = 0x01;
                }
                if (otp_data[0] == 0) {
                    read_otp_info(otp_data);
                } else {
                    pr_err("otp data has already read");
                }
                return ERROR_NONE;
            }
            LOG_INF("Read sensor id fail, write id:0x%x, id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
            retry--;
        } while (retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
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
    kal_uint32 sensor_id = 0;

    LOG_INF("IMX882 open start\n");
    /*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
     *we should detect the module used i2c address
     */
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = ((read_cmos_sensor_8(0x0016) << 8) | read_cmos_sensor_8(0x0017));
            if (sensor_id == IMX882_SENSOR_ID || sensor_id == IMX882_SENSOR_ID2) {
                sensor_id = imgsensor_info.sensor_id;
                LOG_INF("Read sensor id Success, write id:0x%x, id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, write id:0x%x, id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
            retry--;
        } while (retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id) {
        return ERROR_SENSORID_READ_FAIL;
    }
    /* initail sequence write in  */
    sensor_init();
    if(oplus_is_system_camera(OPLUS_CHECK_IS_SYSTEM_CAM)){
        write_sensor_QSC();
    }

    write_sensor_SPC();
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
    LOG_INF("IMX882 open End\n");
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
    /* No Need to implement this function */
    streaming_control(KAL_FALSE);
    qsc_flag = 0;
    spc_flag = 0;
    return ERROR_NONE;
} /* close */

/*write AWB gain to sensor*/
static kal_uint16 imx882pd_feedback_awbgain[] = {
    0x0b90, 0x00,
    0x0b91, 0x01,
    0x0b92, 0x00,
    0x0b93, 0x01,
};
static void feedback_awbgain(kal_uint32 r_gain, kal_uint32 b_gain)
{
    UINT32 r_gain_int = 0;
    UINT32 b_gain_int = 0;

    r_gain_int = r_gain / 522;
    b_gain_int = b_gain / 512;

    imx882pd_feedback_awbgain[1] = r_gain_int;
    imx882pd_feedback_awbgain[3] = (
        ((r_gain*100) / 512) - (r_gain_int * 100)) * 2;
    imx882pd_feedback_awbgain[5] = b_gain_int;
    imx882pd_feedback_awbgain[7] = (
        ((b_gain * 100) / 512) - (b_gain_int * 100)) * 2;
    imx882pd_table_write_cmos_sensor(imx882pd_feedback_awbgain,
        sizeof(imx882pd_feedback_awbgain)/sizeof(kal_uint16));
}

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
    LOG_INF("imx882:Enter %s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    preview_setting();
    LOG_INF("imx882:Leave %s., w*h:%d x %d.\n", __func__, imgsensor.line_length, imgsensor.frame_length);
    mdelay(8);

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

    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate){
        LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
                imgsensor.current_fps,
                imgsensor_info.cap.max_framerate / 10);
    }
    imgsensor.pclk = imgsensor_info.cap.pclk;
    imgsensor.line_length = imgsensor_info.cap.linelength;
    imgsensor.frame_length = imgsensor_info.cap.framelength;
    imgsensor.min_frame_length = imgsensor_info.cap.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;


    spin_unlock(&imgsensor_drv_lock);
    capture_setting(imgsensor.current_fps);
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* capture() */
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
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    /*imgsensor.video_mode = KAL_TRUE;*/
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    /*imgsensor.current_fps = 300;*/
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();
    //set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s. 720P@240FPS\n", __func__);

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
    //set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* slim_video */


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
    set_mirror_flip(imgsensor.mirror);
    return ERROR_NONE;
}    /* custom1 */

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

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* custom2 */

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
    //write_sensor_QSC();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* custom2 */

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

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* custom4 */

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

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}

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

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* custom6 */

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

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}/* custom7 */

static kal_uint32 custom8(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM8;
    imgsensor.pclk = imgsensor_info.custom8.pclk;
    imgsensor.line_length = imgsensor_info.custom8.linelength;
    imgsensor.frame_length = imgsensor_info.custom8.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom8.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom8_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}/* custom8 */

static kal_uint32 custom9(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM9;
    imgsensor.pclk = imgsensor_info.custom9.pclk;
    imgsensor.line_length = imgsensor_info.custom9.linelength;
    imgsensor.frame_length = imgsensor_info.custom9.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom9.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom9_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}/* custom9 */

static kal_uint32 custom10(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM10;
    imgsensor.pclk = imgsensor_info.custom10.pclk;
    imgsensor.line_length = imgsensor_info.custom10.linelength;
    imgsensor.frame_length = imgsensor_info.custom10.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom10.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom10_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}/* custom8 */

static kal_uint32
get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
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

    sensor_resolution->SensorCustom8Width =
        imgsensor_info.custom8.grabwindow_width;
    sensor_resolution->SensorCustom8Height =
        imgsensor_info.custom8.grabwindow_height;
	
	sensor_resolution->SensorCustom9Width =
        imgsensor_info.custom9.grabwindow_width;
    sensor_resolution->SensorCustom9Height =
        imgsensor_info.custom9.grabwindow_height;
	
	sensor_resolution->SensorCustom10Width =
        imgsensor_info.custom10.grabwindow_width;
    sensor_resolution->SensorCustom10Height =
        imgsensor_info.custom10.grabwindow_height;
        return ERROR_NONE;
} /* get_resolution */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
               MSDK_SENSOR_INFO_STRUCT *sensor_info,
               MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
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
    sensor_info->Custom8DelayFrame = imgsensor_info.custom8_delay_frame;
    sensor_info->Custom9DelayFrame = imgsensor_info.custom9_delay_frame;
    sensor_info->Custom10DelayFrame = imgsensor_info.custom10_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
    sensor_info->AESensorGainDelayFrame =
        imgsensor_info.ae_sensor_gain_delay_frame;
    sensor_info->AEISPGainDelayFrame =
        imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
    sensor_info->PDAF_Support = 2;
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
    case MSDK_SCENARIO_ID_CUSTOM8:
        sensor_info->SensorGrabStartX = imgsensor_info.custom8.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom8.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom8.mipi_data_lp2hs_settle_dc;
        break;
	case MSDK_SCENARIO_ID_CUSTOM9:
        sensor_info->SensorGrabStartX = imgsensor_info.custom9.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom9.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom9.mipi_data_lp2hs_settle_dc;
        break;
	case MSDK_SCENARIO_ID_CUSTOM10:
        sensor_info->SensorGrabStartX = imgsensor_info.custom10.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom10.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom10.mipi_data_lp2hs_settle_dc;
        break;
    default:
        sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
        break;
    }

    return ERROR_NONE;
}    /*    get_info  */

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
        //imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
        imgsensor.autoflicker_en = KAL_FALSE;
        imgsensor.pclk = imgsensor_info.pre.pclk;
        imgsensor.line_length = imgsensor_info.pre.linelength;
        imgsensor.frame_length = imgsensor_info.pre.framelength;
        imgsensor.min_frame_length = imgsensor_info.pre.framelength;
        spin_unlock(&imgsensor_drv_lock);

        FMC_GPH_START;
        imx882pd_table_write_cmos_sensor(imx882pd_23231_seamless_preview, sizeof(imx882pd_23231_seamless_preview) / sizeof(kal_uint16));
        if (ae_ctrl) {
            LOG_INF("call MSDK_SCENARIO_ID_CAMERA_PREVIEW %d %d",
                    ae_ctrl[SHUTTER_NE_FRM_1] ,ae_ctrl[GAIN_NE_FRM_1]);
            set_shutter_w_gph(ae_ctrl[SHUTTER_NE_FRM_1], KAL_FALSE);
            set_gain_w_gph(ae_ctrl[GAIN_NE_FRM_1], KAL_FALSE);
        }
        FMC_GPH_END;
    }
    break;
    case MSDK_SCENARIO_ID_CUSTOM5:
    {
        spin_lock(&imgsensor_drv_lock);
        //imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
        imgsensor.autoflicker_en = KAL_FALSE;
        imgsensor.pclk = imgsensor_info.custom5.pclk;
        imgsensor.line_length = imgsensor_info.custom5.linelength;
        imgsensor.frame_length = imgsensor_info.custom5.framelength;
        imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
        spin_unlock(&imgsensor_drv_lock);

        FMC_GPH_START;
        imx882pd_table_write_cmos_sensor(imx882pd_23231_seamless_custom5, sizeof(imx882pd_23231_seamless_custom5) / sizeof(kal_uint16));
        if (ae_ctrl) {
            LOG_INF("call MSDK_SCENARIO_ID_CUSTOM5 %d %d",
                    ae_ctrl[SHUTTER_NE_FRM_1], ae_ctrl[GAIN_NE_FRM_1]);

            set_shutter_w_gph(ae_ctrl[SHUTTER_NE_FRM_1], KAL_FALSE);
            set_gain_w_gph(ae_ctrl[GAIN_NE_FRM_1], KAL_FALSE);
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

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
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
    case MSDK_SCENARIO_ID_CUSTOM8:
        custom8(image_window, sensor_config_data);
        break;
	case MSDK_SCENARIO_ID_CUSTOM9:
        custom9(image_window, sensor_config_data);
        break;
	case MSDK_SCENARIO_ID_CUSTOM10:
        custom10(image_window, sensor_config_data);
        break;
    default:
        LOG_INF("Error ScenarioId setting");
        preview(image_window, sensor_config_data);
        return ERROR_INVALID_SCENARIO_ID;
    }

    return ERROR_NONE;
}    /* control() */



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


static kal_uint32 set_max_framerate_by_scenario(
        enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        frame_length = imgsensor_info.pre.pclk / framerate * 10
                / imgsensor_info.pre.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.pre.framelength)
        ? (frame_length - imgsensor_info.pre.framelength) : 0;
        imgsensor.frame_length =
            imgsensor_info.pre.framelength
            + imgsensor.dummy_line;
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
        frame_length = imgsensor_info.normal_video.pclk /
                framerate * 10 /
                imgsensor_info.normal_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.normal_video.framelength)
        ? (frame_length - imgsensor_info.normal_video.framelength)
        : 0;
        imgsensor.frame_length =
            imgsensor_info.normal_video.framelength
            + imgsensor.dummy_line;
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
            LOG_INF(
                "Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
                , framerate, imgsensor_info.cap.max_framerate/10);
            frame_length = imgsensor_info.cap.pclk / framerate * 10
                    / imgsensor_info.cap.linelength;
            spin_lock(&imgsensor_drv_lock);
                imgsensor.dummy_line =
                (frame_length > imgsensor_info.cap.framelength)
                  ? (frame_length - imgsensor_info.cap.framelength) : 0;
                imgsensor.frame_length =
                    imgsensor_info.cap.framelength
                    + imgsensor.dummy_line;
                imgsensor.min_frame_length = imgsensor.frame_length;
                spin_unlock(&imgsensor_drv_lock);
            if (imgsensor.frame_length > imgsensor.shutter) {
                write_cmos_sensor_8(0x0104, 0x01);
                set_dummy();
                write_cmos_sensor_8(0x0104, 0x00);
            }
        break;
    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        frame_length = imgsensor_info.hs_video.pclk / framerate * 10
                / imgsensor_info.hs_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.hs_video.framelength)
              ? (frame_length - imgsensor_info.hs_video.framelength)
              : 0;
        imgsensor.frame_length =
            imgsensor_info.hs_video.framelength
                + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_SLIM_VIDEO:
        frame_length = imgsensor_info.slim_video.pclk / framerate * 10
            / imgsensor_info.slim_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.slim_video.framelength)
            ? (frame_length - imgsensor_info.slim_video.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.slim_video.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM1:
        frame_length = imgsensor_info.custom1.pclk / framerate * 10
                / imgsensor_info.custom1.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom1.framelength)
            ? (frame_length - imgsensor_info.custom1.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom1.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM2:
        frame_length = imgsensor_info.custom2.pclk / framerate * 10
                / imgsensor_info.custom2.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom2.framelength)
            ? (frame_length - imgsensor_info.custom2.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom2.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM3:
        frame_length = imgsensor_info.custom3.pclk / framerate * 10
                / imgsensor_info.custom3.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom3.framelength)
            ? (frame_length - imgsensor_info.custom3.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom3.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM4:
        /*frame_length = imgsensor_info.custom4.pclk / framerate * 10
                / imgsensor_info.custom4.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom4.framelength)
            ? (frame_length - imgsensor_info.custom4.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom4.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }*/
        break;
    case MSDK_SCENARIO_ID_CUSTOM5:
        frame_length = imgsensor_info.custom5.pclk / framerate * 10
                / imgsensor_info.custom5.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom5.framelength)
            ? (frame_length - imgsensor_info.custom5.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom5.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM6:
        frame_length = imgsensor_info.custom6.pclk / framerate * 10
                / imgsensor_info.custom6.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom6.framelength)
            ? (frame_length - imgsensor_info.custom6.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom6.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        frame_length = imgsensor_info.custom7.pclk / framerate * 10
                / imgsensor_info.custom7.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom7.framelength)
            ? (frame_length - imgsensor_info.custom7.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom7.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM8:
        frame_length = imgsensor_info.custom8.pclk / framerate * 10
                / imgsensor_info.custom8.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom8.framelength)
            ? (frame_length - imgsensor_info.custom8.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom8.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
	case MSDK_SCENARIO_ID_CUSTOM9:
        frame_length = imgsensor_info.custom9.pclk / framerate * 10
                / imgsensor_info.custom9.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom9.framelength)
            ? (frame_length - imgsensor_info.custom9.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom9.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
	case MSDK_SCENARIO_ID_CUSTOM10:
        frame_length = imgsensor_info.custom10.pclk / framerate * 10
                / imgsensor_info.custom10.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom10.framelength)
            ? (frame_length - imgsensor_info.custom10.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom10.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    default:  /*coding with  preview scenario by default*/
        frame_length = imgsensor_info.pre.pclk / framerate * 10
            / imgsensor_info.pre.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.pre.framelength)
            ? (frame_length - imgsensor_info.pre.framelength) : 0;
        imgsensor.frame_length =
            imgsensor_info.pre.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        LOG_INF("error scenario_id = %d, we use preview scenario\n",
            scenario_id);
        break;
    }
    return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
        enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

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
    case MSDK_SCENARIO_ID_CUSTOM8:
        *framerate = imgsensor_info.custom8.max_framerate;
        break;
	case MSDK_SCENARIO_ID_CUSTOM9:
        *framerate = imgsensor_info.custom9.max_framerate;
        break;
	case MSDK_SCENARIO_ID_CUSTOM10:
        *framerate = imgsensor_info.custom10.max_framerate;
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
           switch(modes) {
           case 5:
               write_cmos_sensor_8(0x020E, 0x00);
               write_cmos_sensor_8(0x0218, 0x00);
               write_cmos_sensor_8(0x3015, 0x00);
               break;
           default:
               write_cmos_sensor_8(0x0601, modes);
               break;
           }
       } else if (imgsensor.test_pattern) {
           write_cmos_sensor_8(0x0601, 0x00); /*No pattern*/
           write_cmos_sensor_8(0x020E, 0x01);
           write_cmos_sensor_8(0x0218, 0x01);
           write_cmos_sensor_8(0x3015, 0x40);
       }

    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = modes;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
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
            LOG_INF("SENSOR_FEATURE_SET_SENSOR_OTP\n");
            ret = Eeprom_CallWriteService((ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(feature_para));
            if (ret == IMGSENSOR_RETURN_SUCCESS)
                return ERROR_NONE;
            else
                return ERROR_MSDK_IS_ACTIVATED;
        }
        break;
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
        break;
    case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
        *(MINT32 *)(signed long)(*(feature_data + 1)) = -1345000;
        LOG_INF("exporsure");
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
        case MSDK_SCENARIO_ID_CUSTOM8:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom8.pclk;
                break;
		case MSDK_SCENARIO_ID_CUSTOM9:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom9.pclk;
                break;
		case MSDK_SCENARIO_ID_CUSTOM10:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom10.pclk;
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
            //ratio = get_exp_cnt_by_scenario((*feature_data));
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
        case MSDK_SCENARIO_ID_CUSTOM8:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom8.framelength << 16)
                                 + (ratio * imgsensor_info.custom8.linelength);
                break;
		case MSDK_SCENARIO_ID_CUSTOM9:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom9.framelength << 16)
                                 + (ratio * imgsensor_info.custom9.linelength);
                break;
		case MSDK_SCENARIO_ID_CUSTOM10:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom10.framelength << 16)
                                 + (ratio * imgsensor_info.custom10.linelength);
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
    case SENSOR_FEATURE_CHECK_MODULE_ID:
        *feature_return_para_32 = imgsensor_info.module_id;
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
        sensor_reg_data->RegData =
            read_cmos_sensor_8(sensor_reg_data->RegAddr);
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
    case SENSOR_FEATURE_GET_SENSOR_OTP_ALL:
    {
        memcpy(feature_return_para_32, (UINT32 *)otp_data, sizeof(otp_data));
        break;
    }
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
    case SENSOR_FEATURE_GET_PDAF_DATA:
        LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
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
        LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
            (UINT32)*feature_data);
        wininfo =
    (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

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
        case MSDK_SCENARIO_ID_CUSTOM8:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[12],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
		case MSDK_SCENARIO_ID_CUSTOM9:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[13],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
		case MSDK_SCENARIO_ID_CUSTOM10:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[14],
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
    case SENSOR_FEATURE_GET_PDAF_INFO:
        LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
            (UINT16) *feature_data);
        PDAFinfo =
            (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
            memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
            sizeof(struct SET_PD_BLOCK_INFO_T));
            switch (*feature_data) {
                case MSDK_SCENARIO_ID_CUSTOM2:
                    PDAFinfo->i4BlockNumX = 504;
                    PDAFinfo->i4BlockNumY = 144;
                break;
            default:
                break;
        }
        break;
    case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
        LOG_INF(
        "SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
            (UINT16) *feature_data);
        /*PDAF capacity enable or not, 2p8 only full size support PDAF*/
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
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
        case MSDK_SCENARIO_ID_CUSTOM1:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
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
        case MSDK_SCENARIO_ID_CUSTOM8:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_CUSTOM9:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_CUSTOM10:
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
    case SENSOR_FEATURE_GET_VC_INFO2:
         LOG_INF("SENSOR_FEATURE_GET_VC_INFO2 %d\n",
                            (UINT16) (*feature_data));
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
    case SENSOR_FEATURE_GET_STAGGER_MAX_EXP_TIME:
        if (*feature_data == MSDK_SCENARIO_ID_CUSTOM4) {
            switch (*(feature_data + 1)) {
            case VC_STAGGER_NE:
                *(feature_data + 2) = 32757;   //need repare
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
        hdr_write_double_shutter((UINT16)*feature_data,
            (UINT16)*(feature_data+1));
        break;
    case SENSOR_FEATURE_SET_DUAL_GAIN://for 2EXP
        LOG_INF("SENSOR_FEATURE_SET_DUAL_GAIN LE=%d, SE=%d\n",
            (UINT16) *feature_data, (UINT16) *(feature_data + 1));
        // implement write gain for NE/SE
        hdr_write_double_gain((UINT16)*feature_data,
            (UINT16)*(feature_data+1));
        break;
    case SENSOR_FEATURE_SET_PDAF:
        LOG_INF("PDAF mode :%d\n", *feature_data_16);
        imgsensor.pdaf_mode = *feature_data_16;
        break;
    case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
        set_shutter_frame_length((UINT16) (*feature_data),
                    (UINT16) (*(feature_data + 1)), (BOOL) (*(feature_data + 2)));
        break;
    case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
        LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
            (UINT16)*feature_data,
            (UINT16)*(feature_data+1),
            (UINT16)*(feature_data+2));
        break;
    case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
        LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
        streaming_control(KAL_FALSE);
        break;
    case SENSOR_FEATURE_SET_STREAMING_RESUME:
        LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
            *feature_data);
        if (*feature_data != 0)
            set_shutter(*feature_data);
        streaming_control(KAL_TRUE);
        break;
    case SENSOR_FEATURE_GET_BINNING_TYPE:
        switch (*(feature_data + 1)) {
        case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM5:
            *feature_return_para_32 = 1; /*full size*/
            break;
        default:
            *feature_return_para_32 = 1; /*BINNING_AVERAGED*/
            break;
        }
        pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
            *feature_return_para_32);
        *feature_para_len = 4;

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
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.slim_video.mipi_pixel_rate;
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
        case MSDK_SCENARIO_ID_CUSTOM8:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom8.mipi_pixel_rate;
            break;
		case MSDK_SCENARIO_ID_CUSTOM9:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom9.mipi_pixel_rate;
            break;
		case MSDK_SCENARIO_ID_CUSTOM10:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom10.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.pre.mipi_pixel_rate;
            break;
        }
    }
    break;

    case SENSOR_FEATURE_SET_AWB_GAIN:
        /* modify to separate 3hdr and remosaic */
        if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3) {
            /*write AWB gain to sensor*/
            feedback_awbgain((UINT32)*(feature_data_32 + 1),
                    (UINT32)*(feature_data_32 + 2));
        }
    case SENSOR_FEATURE_SET_LSC_TBL:
        break;
    case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
        *(feature_data + 1) = 1; /* margin info by scenario */
        *(feature_data + 2) = imgsensor_info.margin;
        break;
    /*case SENSOR_FEATURE_SET_SEAMLESS_EXTEND_FRAME_LENGTH:
        pr_info("extend_frame_len %d\n", *feature_data);
        extend_frame_length((MUINT32) *feature_data);
        pr_info("extend_frame_len done %d\n", *feature_data);
        break;*/
    case SENSOR_FEATURE_SEAMLESS_SWITCH:
    {
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
        case MSDK_SCENARIO_ID_CUSTOM5:
            *pScenarios = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *pScenarios = MSDK_SCENARIO_ID_CUSTOM5;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CUSTOM2:
        case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM4:
        case MSDK_SCENARIO_ID_CUSTOM6:
        case MSDK_SCENARIO_ID_CUSTOM7:
        case MSDK_SCENARIO_ID_CUSTOM8:
		case MSDK_SCENARIO_ID_CUSTOM9:
		case MSDK_SCENARIO_ID_CUSTOM10:
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
        }

    default:
        break;
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

UINT32 IMX882PD_MIPI_RAW23231_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc != NULL)
        *pfFunc = &sensor_func;
    return ERROR_NONE;
} /* IMX882PD_MIPI_RAW_SensorInit */
