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
 *   s5kjn1mipiraw_Sensor.c
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
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
#include "imgsensor_common.h"

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "imgsensor_sensor_list.h"
#include "s5kjn1mipiraw23707_Sensor.h"
#include "imgsensor_hwcfg_custom.h"

#define PFX "S5KJN1_camera_sensor"
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __func__, ##args)

#define MULTI_WRITE 1

#define MaxAnalogGain 64

#ifndef ODM_HQ_EDIT
/*Cong.Zhou@ODM_HQ Cam.Drv 20191218 for decrease poweron time*/
#define ODM_HQ_EDIT
#endif

#if MULTI_WRITE
#define I2C_BUFFER_LEN 1020 /*trans# max is 255, each 4 bytes*/
#else
#define I2C_BUFFER_LEN 4
#endif

static kal_uint16 camstate;
//static kal_uint32 revision;
static kal_uint16 lastcamstate;

static DEFINE_SPINLOCK(imgsensor_drv_lock);
static bool bIsLongExposure = KAL_FALSE;

//#ifdef OPLUS_FEATURE_CAMERA_COMMON
  extern void register_imgsensor_deviceinfo(char *name, char *version, u8 module_id);
  static kal_uint8 deviceInfo_register_value = 0x00;
  extern enum IMGSENSOR_RETURN Eeprom_DataInit(
            enum IMGSENSOR_SENSOR_IDX sensor_idx,
            kal_uint32 sensorID);
  static kal_uint32 streaming_control(kal_bool enable);
  #define MODULE_ID_OFFSET 0x0000
  #define EEPROM_I2C_ADDR 0xA0
  #define DEVICE_VERSION  "s5kjn1"
//#endif

static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = S5KJN1_SENSOR_ID23707,
    .module_id = 0x01, //0x01 Sunny
    .checksum_value = 0x362cca75,

    .pre = {
        .pclk = 560000000,
        .linelength = 4584,
        .framelength = 4054,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4080,
        .grabwindow_height = 3072,
        //grabwindow_height should be 16's N times
        .mipi_data_lp2hs_settle_dc = 0x22,/*// cphy  need to confirm from HQ*/
        .max_framerate = 300,
        .mipi_pixel_rate = 662400000,
        .jn1_binning_mode = 2,
    },
    .cap = {
        .pclk = 560000000,
        .linelength = 4584,
        .framelength = 4054,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4080,
        .grabwindow_height = 3072,
        //grabwindow_height should be 16's N times
        .mipi_data_lp2hs_settle_dc = 0x22,/*// cphy  need to confirm from HQ*/
        .max_framerate = 300,
        .mipi_pixel_rate = 662400000,
        .jn1_binning_mode = 2,
    },
    .cap1 = {
        .pclk = 960000000, /*// 4MP 69fps case*/
        .linelength = 15360, /*//0x3C00*/
        .framelength = 1262, /*//0x04EE*/
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2320, /*//0x0910*/
        .grabwindow_height = 1736, /*//0x06C8*/
        //grabwindow_height should be 16's N times
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 1200,
        .jn1_binning_mode = 0,
    },
    .cap2 = {
        .pclk = 960000000, /*// 4MP 69fps case*/
        .linelength = 15360, /*//0x3C00*/
        .framelength = 1262, /*//0x04EE*/
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2320, /*//0x0910*/
        .grabwindow_height = 1736, /*//0x06C8*/
        //grabwindow_height should be 16's N times
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 1200,
        .jn1_binning_mode = 0,
    },
    .normal_video = {/*video4k 1080P 30fps*/
        .pclk = 560000000,
        .linelength = 5632,
        .framelength = 3312,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3840,
        .grabwindow_height = 2160,
        //grabwindow_height should be 16's N times
        .mipi_data_lp2hs_settle_dc = 0x22,/*// cphy  need to confirm from HQ*/
        .max_framerate = 300,
        .mipi_pixel_rate = 513600000,
        .jn1_binning_mode = 2,
    },
    .hs_video = {//smvr 1080P 240fps
        .pclk = 600000000,
        .linelength = 2064,
        .framelength = 1208,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1920,
        .grabwindow_height = 1080,
        .mipi_data_lp2hs_settle_dc = 0x22,
        .max_framerate = 2400,
        .mipi_pixel_rate = 792000000,
        .jn1_binning_mode = 3,
    },
    .custom1 = {//stereo same as preview
        .pclk = 560000000,
        .linelength = 5910,
        .framelength = 3938,
        .startx = 8,
        .starty = 12,
        .grabwindow_width = 4064,
        .grabwindow_height = 3048,
        //grabwindow_height should be 16's N times
        .mipi_data_lp2hs_settle_dc = 0x22,/*// cphy  need to confirm from HQ*/
        .max_framerate = 240,
        .mipi_pixel_rate = 480000000,
        .jn1_binning_mode = 2,
    },
    .custom2 = {/*slow motion 1080 120fps (4SUMA2A2) */
        .pclk = 600000000,
        .linelength =4256,
        .framelength = 1174,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1920,
        .grabwindow_height =1080,
        .mipi_data_lp2hs_settle_dc = 0x22,/*// cphy  need to confirm from HQ*/
        .max_framerate =1200,
        .mipi_pixel_rate = 640000000,
        .jn1_binning_mode = 3,
    },
    .custom3 = {
        .pclk = 560000000,
        .linelength = 8688,
        .framelength = 6400,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 8160,
        .grabwindow_height = 6144,
        //grabwindow_height should be 16's N times
        .mipi_data_lp2hs_settle_dc = 0x22,/*// cphy  need to confirm from HQ*/
        .max_framerate = 100,
        .mipi_pixel_rate = 556800000,
        .jn1_binning_mode = 0,
    },
        .custom4 = {/*1920×1080 60FPS*/
        .pclk = 600000000,
        .linelength =5824,
        .framelength = 1716,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1920,
        .grabwindow_height = 1080,
        .mipi_data_lp2hs_settle_dc = 0x22,/*// cphy  need to confirm from HQ*/
        .max_framerate = 600,
        .mipi_pixel_rate =320000000,
        .jn1_binning_mode = 3,
    },
    .slim_video = {//4M 60fps (4SUMA2A2)
        .pclk = 1920000000,
        .linelength = 0x3830,
        .framelength = 0x08B0,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2320,
        .grabwindow_height = 1736,

        .mipi_data_lp2hs_settle_dc = 0x22,
        .max_framerate = 600,
        .mipi_pixel_rate = 412000000,
        .jn1_binning_mode = 3,
    },
    .custom5 = {//stereo same as preview
        .pclk = 560000000,
        .linelength = 6534,
        .framelength = 3567,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3648,
        .grabwindow_height = 2736,
        //grabwindow_height should be 16's N times
        .mipi_data_lp2hs_settle_dc = 0x22,/*// cphy  need to confirm from HQ*/
        .max_framerate = 240,
        .mipi_pixel_rate = 400000000,
        .jn1_binning_mode = 2,
    },
    .margin = 8,    /* sensor framelength & shutter margin */
    .min_gain = 64, /*1x gain*/
    .max_gain = 4096, /*64x gain*/
    .min_gain_iso = 100,
    .gain_step = 1,
    .gain_type = 0,
    .min_shutter = 4, /* min shutter */

    /* max framelength by sensor register's limitation */
    .max_frame_length = 0xffff,

    /* shutter delay frame for AE cycle,
     * 2 frame with ispGain_delay-shut_delay=2-0=2
     */
    .ae_shut_delay_frame = 0,

    /* sensor gain delay frame for AE cycle,
     * 2 frame with ispGain_delay-sensor_gain_delay=2-0=2
     */
    .ae_sensor_gain_delay_frame = 0,

    .ae_ispGain_delay_frame = 2, /* isp gain delay frame for AE cycle */
    .frame_time_delay_frame = 2, /* The delay frame of setting frame length  */
    .ihdr_support = 0, /* 1, support; 0,not support */
    .ihdr_le_firstline = 0, /* 1,le first ; 0, se first */
    .sensor_mode_num = 10, /* support sensor mode num */

    .cap_delay_frame = 3, /* enter capture delay frame num */
    .pre_delay_frame = 3, /* enter preview delay frame num */
    .video_delay_frame = 3, /* enter video delay frame num */

    /* enter high speed video  delay frame num */
    .hs_video_delay_frame = 3,

    .slim_video_delay_frame = 3, /* enter slim video delay frame num */
    .custom1_delay_frame = 3, /* enter custom1 delay frame num */
    .custom2_delay_frame = 3, /* enter custom2 delay frame num */
    .custom3_delay_frame = 3, /* enter custom3 delay frame num */
    .custom4_delay_frame = 3, /* enter custom4 delay frame num */
    .custom5_delay_frame = 3, /* enter custom5 delay frame num */

    .isp_driving_current = ISP_DRIVING_8MA, /* mclk driving current */

    /* sensor_interface_type */
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,

    /* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
    .mipi_sensor_type = MIPI_OPHY_NCSI2,

    /* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
    .mipi_settle_delay_mode = 1,
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_Gr,
    //.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
    .mclk = 24, /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
    .mipi_lane_num = SENSOR_MIPI_4_LANE,
    //.i2c_speed = 1000, /*support 1MHz write*/
    /* record sensor support all write id addr,
     * only supprt 4 must end with 0xff
     */
    //.i2c_addr_table = {0x20, 0xff},
    .i2c_addr_table = {0x5a,0xff},
};

static struct imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL, /* mirrorflip information */

    /* IMGSENSOR_MODE enum value,record current sensor mode,such as:
     * INIT, Preview, Capture, Video,High Speed Video, Slim Video
     */
    .sensor_mode = IMGSENSOR_MODE_INIT,

    .shutter = 0x3D0, /* current shutter */
    .gain = 0x100,  /* current gain */
    .dummy_pixel = 0, /* current dummypixel */
    .dummy_line = 0, /* current dummyline */
    .current_fps = 0, /* full size current fps : 24fps for PIP,
                 * 30fps for Normal or ZSD
                 */

    /* auto flicker enable: KAL_FALSE for disable auto flicker,
     * KAL_TRUE for enable auto flicker
     */
    .autoflicker_en = KAL_FALSE,

    /* test pattern mode or not.
     * KAL_FALSE for in test pattern mode,
     * KAL_TRUE for normal output
     */
    .test_pattern = KAL_FALSE,

    /* current scenario id */
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
    .ihdr_en = 0,
    /* sensor need support LE, SE with HDR feature */

    .hdr_mode = KAL_FALSE,
    /* HDR Mode : 0: disable HDR, 1:IHDR, 2:HDR, 9:ZHDR */
    .i2c_write_id = 0x5a,
    /* record current sensor's i2c write id */
    //#ifdef ODM_HQ_EDIT
    /*Cong.Zhou@ODM_HQ Cam.Drv 20191030 long exp n+2, n+1 will cause switch slowly*/
    .current_ae_effective_frame = 2,
    //#endif

};

//int chip_id;
/* VC_Num, VC_PixelNum, ModeSelect, EXPO_Ratio, ODValue, RG_STATSMODE */
/* VC0_ID, VC0_DataType, VC0_SIZEH, VC0_SIZE,
 * VC1_ID, VC1_DataType, VC1_SIZEH, VC1_SIZEV
 */
/* VC2_ID, VC2_DataType, VC2_SIZEH, VC2_SIZE,
 * VC3_ID, VC3_DataType, VC3_SIZEH, VC3_SIZEV
 */

static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[5] = {
    // Preview mode setting
    {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
    0x00, 0x2B, 0x0FF0, 0x0C00, 0x01, 0x00, 0x0000, 0x0000,
    0x01, 0x30, 0x027C, 0x0BF0, 0x03, 0x00, 0x0000, 0x0000},
    // Video mode setting
    {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
    0x00, 0x2B, 0x0F00, 0x0870, 0x01, 0x00, 0x0000, 0x0000,
    0x01, 0x30, 0x0258, 0x0860, 0x03, 0x00, 0x0000, 0x0000},
    // hs_video mode setting
    {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
    0x00, 0x2B, 0x0780, 0x0438, 0x01, 0x00, 0x0000, 0x0000,
    0x01, 0x30, 0x0258, 0x0218, 0x03, 0x00, 0x0000, 0x0000},
    // Video mode setting
    {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
    0x00, 0x2B, 0x0780, 0x0438, 0x01, 0x00, 0x0000, 0x0000,
    0x01, 0x30, 0x012C, 0x0420, 0x03, 0x00, 0x0000, 0x0000},
    // custom5 mode setting
    {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
    0x00, 0x2B, 0x0E40, 0x0AB0, 0x01, 0x00, 0x0000, 0x0000,
    0x01, 0x30, 0x01C8, 0x0AA0, 0x03, 0x00, 0x0000, 0x0000},
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
    {8160, 6144, 0, 0, 8160, 6144, 4080, 3072, 0, 0, 4080, 3072,
    0, 0, 4080, 3072},/* Preview */

    {8160, 6144, 0, 0, 8160, 6144, 4080, 3072, 0, 0, 4080, 3072,
    0, 0, 4080, 3072},/* Capture */

    {8160, 6144, 240, 912, 7680, 4320, 3840, 2160, 0, 0, 3840, 2160,
    0, 0, 3840, 2160},/* Video */

    {8160, 6144, 240, 912, 7680, 4320, 1920, 1080, 0, 0, 1920, 1080,
    0, 0, 1920, 1080},/*HS_VID SMVR 240FPS 1080P*/

    {8160, 6144, 600, 468, 6960, 5208, 2320, 1736, 0, 0, 2320, 1736,
    0, 0, 2320, 1736},/* slim_video 4M 60FPS*/

    {8160, 6144, 0, 0, 8160, 6144, 4080, 3072, 0, 0, 4080, 3072,
    8, 12, 4064, 3048},/* custom1 stereo as Preview */


    {8160, 6144, 240, 912, 7680, 4320, 1920, 1080, 0, 0, 1920, 1080,
    0, 0, 1920, 1080},/*HS_VID SMVR 120FPS 1080P*/

    {8160, 6144, 0, 0, 8160, 6144, 8160, 6144, 0, 0, 8160, 6144,
    0, 0, 8160, 6144},  /*custom3 as full size capture*/

    {8160, 6144, 240, 912, 7680, 4320, 1920, 1080, 0, 0, 1920, 1080,
    0, 0, 1920, 1080},   /*custom4 for 60fps 1080P*/

    {8160, 6144, 432, 336, 7296, 5472, 3648, 2736, 0, 0, 3648, 2736,
    0, 0, 3648, 2736},/* custom5 stereo as Preview */
};

/* no mirror flip, and no binning -revised by dj */
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
    .i4OffsetX = 8,
    .i4OffsetY = 8,
    .i4PitchX  = 8,
    .i4PitchY  = 8,
    .i4PairNum  = 4,
    .i4SubBlkW  = 8,
    .i4SubBlkH  = 2,
    .i4PosL = {
        {9, 8}, {11, 11}, {15, 12}, {13, 15}
    },
    .i4PosR = {
        {8, 8}, {10, 11}, {14, 12}, {12, 15}
    },
    .iMirrorFlip = 0,
    .i4BlockNumX = 508,
    .i4BlockNumY = 382,
    .i4Crop = { {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_normal_video_pd_info = {
    .i4OffsetX = 8,
    .i4OffsetY = 8,
    .i4PitchX  = 8,
    .i4PitchY  = 8,
    .i4PairNum  = 4,
    .i4SubBlkW  = 8,
    .i4SubBlkH  = 2,
    .i4PosL = {
        {9, 8}, {11, 11}, {15, 12}, {13, 15}
    },
    .i4PosR = {
        {8, 8}, {10, 11}, {14, 12}, {12, 15}
    },
    .iMirrorFlip = 0,
    .i4BlockNumX = 480,
    .i4BlockNumY = 268,
    .i4Crop = { {0,0}, {0,0}, {120,456}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_1920_1080 = {
    .i4OffsetX = 8,
    .i4OffsetY = 8,
    .i4PitchX  = 8,
    .i4PitchY  = 8,
    .i4PairNum  = 4,
    .i4SubBlkW  = 8,
    .i4SubBlkH  = 2,
    .i4PosL = {
        {9, 8}, {11, 11}, {15, 12}, {13, 15}
    },
    .i4PosR = {
        {8, 8}, {10, 11}, {14, 12}, {12, 15}
    },
    .iMirrorFlip = 0,
    .i4BlockNumX = 240,
    .i4BlockNumY = 132,
    .i4Crop = { {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {1080,996}},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_custom5_pd_info = {
    .i4OffsetX = 8,
    .i4OffsetY = 8,
    .i4PitchX  = 8,
    .i4PitchY  = 8,
    .i4PairNum  = 4,
    .i4SubBlkW  = 8,
    .i4SubBlkH  = 2,
    .i4PosL = {
        {9, 8}, {11, 11}, {15, 12}, {13, 15}
    },
    .i4PosR = {
        {8, 8}, {10, 11}, {14, 12}, {12, 15}
    },
    .iMirrorFlip = 0,
    .i4BlockNumX = 456,
    .i4BlockNumY = 340,
    .i4Crop = { {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {216,168}},
};

#define RWB_ID_OFFSET 0x0F73
#define EEPROM_READ_ID  0x19
#define EEPROM_WRITE_ID   0x18


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {
        (char)(addr >> 8), (char)(addr & 0xFF),
        (char)(para >> 8), (char)(para & 0xFF) };
    iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_8(kal_uint32 addr)
{
    kal_uint16 get_byte = 0;
    int ret = 0;
    char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };
    ret = iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 1, imgsensor.i2c_write_id);

    printk("[%s] read i2c: 0x%x addr:0x%x value:0x%x, ret = %d\n", __func__, imgsensor.i2c_write_id , addr, get_byte, ret);

    return get_byte;
}
/*
static kal_uint16 read_cmos_sensor_16(kal_uint32 addr)
{
    kal_uint16 get_byte = 0;
    int ret = 0;
    char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };
    ret = iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 2, imgsensor.i2c_write_id);

    printk("[%s] read i2c: (slave, reg_addr, value) = (0x%x, 0x%x, 0x%x), ret = %d\n", __func__, imgsensor.i2c_write_id, addr, get_byte, ret);

    return get_byte;
}
*/
static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[3] = {
        (char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF) };

    iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static kal_uint32 return_sensor_id(void)
{
    kal_uint32 sensor_id = 0;
    kal_uint16 sensor_id_0 = read_cmos_sensor_8(0x0000);
    kal_uint16 sensor_id_1 = read_cmos_sensor_8(0x0001);

    LOG_INF("[%s] sensor_id: sensor_id_0:0x%x sensor_id_1:0x%x", __func__, sensor_id_0, sensor_id_1);
    sensor_id = ((sensor_id_0 << 8) & 0xFF00) | (sensor_id_1 & 0xFF);

    LOG_INF("[%s] sensor_id: 0x%x", __func__, sensor_id);

    return sensor_id;
}

static void set_dummy(void)
{
    pr_debug("dummyline = %d, dummypixels = %d\n",
        imgsensor.dummy_line, imgsensor.dummy_pixel);

    /* return; //for test */
    write_cmos_sensor(0x0340, imgsensor.frame_length);
    write_cmos_sensor(0x0342, imgsensor.line_length);
}               /*      set_dummy  */

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{

    kal_uint32 frame_length = imgsensor.frame_length;

    pr_debug("framerate = %d, min framelength should enable %d\n",
        framerate, min_framelength_en);

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
}               /*      set_max_framerate  */

static void write_shutter(kal_uint32 shutter)
{

    kal_uint16 realtime_fps = 0;
    //#ifdef ODM_HQ_EDIT
    /*Jiefeng.Wang@ODM_HQ Cam.Drv 20191108 temp modify long exp no effect*/
    //int framecnt = 0;
    //#endif
    pr_debug("===brad shutter:%d\n", shutter);
    if (shutter > 65530) {  //linetime=10160/960000000<< maxshutter=3023622-line=32s
        /*enter long exposure mode */
/*
        kal_uint32 exposure_time;
        kal_uint16 new_framelength;
        kal_uint16 long_shutter;
        kal_uint16 shift = 0;
        kal_uint16 pow_shift = 0;
*/

        bIsLongExposure = KAL_TRUE;

        /*streaming_control(KAL_FALSE);*/
        write_cmos_sensor(0x6028, 0x4000);
        //#ifdef ODM_HQ_EDIT
        /*Jiefeng.Wang@ODM_HQ Cam.Drv 20191108 temp modify long exp no effect*/
        //write_cmos_sensor_8(0x0100, 0x00); /*stream off*/

/*
        while (1) {
            framecnt = read_cmos_sensor_8(0x0005);
            LOG_INF(" Stream Off oning at framecnt=0x%x.\n", framecnt);
            if (framecnt == 0xFF) {
                LOG_INF(" Stream Off OK at framecnt=0x%x.\n", framecnt);
                break;
            }
        }
*/
        //#endif

        //write_cmos_sensor(0x0340, 0x45e2);
        //write_cmos_sensor(0x0202, 0x45de);
        write_cmos_sensor(0x0340, (shutter>>6)+3);
        write_cmos_sensor(0x0202, (shutter>>6));
        write_cmos_sensor(0x0702, 0x0600);
        write_cmos_sensor(0x0704, 0x0600);

        //#ifdef ODM_HQ_EDIT
        /*Jiefeng.Wang@ODM_HQ Cam.Drv 20191108 temp modify long exp no effect*/
        //write_cmos_sensor_8(0x0100, 0x01);  /*stream on*/
        //#endif

        /* Frame exposure mode customization for LE*/
        imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
        imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
        //#ifdef ODM_HQ_EDIT
        /*Cong.Zhou@ODM_HQ Cam.Drv 20191030 long exp n+2, n+1 will cause switch slowly*/
        imgsensor.current_ae_effective_frame = 2;
        //#endif
    } else {
        //#ifdef ODM_HQ_EDIT
        /*Cong.Zhou@ODM_HQ Cam.Drv 20191030 long exp n+2, n+1 will cause switch slowly*/
        imgsensor.current_ae_effective_frame = 2;
        //#endif
        if (bIsLongExposure) {
            pr_debug("brad enter normal shutter.\n");
            write_cmos_sensor(0x6028, 0x4000);
            //#ifdef ODM_HQ_EDIT
            /*Jiefeng.Wang@ODM_HQ Cam.Drv 20191108 temp modify long exp no effect*/
            //write_cmos_sensor_8(0x0100, 0x00); /*stream off*/
/*
            while (1) {
                framecnt = read_cmos_sensor_8(0x0005);
                pr_debug("Stream Off oning at framecnt=0x%x.\n", framecnt);
                if (framecnt == 0xFF) {
                    pr_debug(" Stream Off OK at framecnt=0x%x.\n", framecnt);
                    break;
                }
            }
*/
            //#endif

            write_cmos_sensor(0x0340, imgsensor.frame_length);
            write_cmos_sensor(0x0202, shutter);
            write_cmos_sensor(0x0702, 0x0000);
            write_cmos_sensor(0x0704, 0x0000);

            //#ifdef ODM_HQ_EDIT
            /*Jiefeng.Wang@ODM_HQ Cam.Drv 20191108 temp modify long exp no effect*/
            //write_cmos_sensor_8(0x0100, 0x01);  /*stream on*/
            //#endif
            bIsLongExposure = KAL_FALSE;
            pr_debug("===brad enter normal shutter shutter = %d, imgsensor.frame_lengths = 0x%x, imgsensor.line_length = 0x%x\n",
                shutter, imgsensor.frame_length, imgsensor.line_length);
        }

        spin_lock(&imgsensor_drv_lock);
        if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
            imgsensor.frame_length = shutter + imgsensor_info.margin;
        else
            imgsensor.frame_length = imgsensor.min_frame_length;
        if (imgsensor.frame_length > imgsensor_info.max_frame_length)
            imgsensor.frame_length = imgsensor_info.max_frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (shutter < imgsensor_info.min_shutter)
            shutter = imgsensor_info.min_shutter;

        if (imgsensor.autoflicker_en) {
            realtime_fps = imgsensor.pclk
                / imgsensor.line_length * 10 / imgsensor.frame_length;

            if (realtime_fps >= 297 && realtime_fps <= 305)
                set_max_framerate(296, 0);
            else if (realtime_fps >= 147 && realtime_fps <= 150)
                set_max_framerate(146, 0);
            else {
                /* Extend frame length */
                write_cmos_sensor(0x0340, imgsensor.frame_length);
            }
        } else {
            /* Extend frame length */
            write_cmos_sensor(0x0340, imgsensor.frame_length);
            pr_debug("(else)imgsensor.frame_length = %d\n",
                imgsensor.frame_length);
        }

        if (imgsensor.jn1_binning_mode == 1)  //4sum
            shutter = (shutter>>1)<<1;
        if (imgsensor.jn1_binning_mode == 3)  //4sumA2A2
            shutter = (shutter>>2)<<2;

        /* Update Shutter */
        write_cmos_sensor(0x0202, shutter);
        pr_debug("shutter =%d, framelength =%d jn1_binning_mode= %d\n",
            shutter, imgsensor.frame_length, imgsensor.jn1_binning_mode);
    }
}               /*      write_shutter  */

/*************************************************************************
 * FUNCTION
 *  set_shutter
 *
 * DESCRIPTION
 *  This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *  iShutter : exposured lines
 *
 * RETURNS
 *  None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
    unsigned long flags;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    write_shutter(shutter);
}               /*      set_shutter */

static void set_shutter_frame_length(kal_uint16 shutter,
            kal_uint16 frame_length, kal_bool auto_extend_en)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    kal_int32 dummy_line = 0;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    spin_lock(&imgsensor_drv_lock);
    /*Change frame time*/
    if(frame_length > 1)
        dummy_line = frame_length - imgsensor.frame_length;
    imgsensor.frame_length = imgsensor.frame_length + dummy_line;
    //
    if (shutter > imgsensor.frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    shutter = (shutter < imgsensor_info.min_shutter) ?
        imgsensor_info.min_shutter : shutter;
    shutter = (shutter > (imgsensor_info.max_frame_length -
        imgsensor_info.margin)) ? (imgsensor_info.max_frame_length -
        imgsensor_info.margin) : shutter;

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk /
            imgsensor.line_length * 10 / imgsensor.frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
        else {
            // Extend frame length
            //write_cmos_sensor(0x0104, 0x01);
            write_cmos_sensor(0x0340, imgsensor.frame_length);
            //write_cmos_sensor(0x0104, 0x00);
        }
    } else {
        // Extend frame length
        //write_cmos_sensor(0x0104, 0x01);
        write_cmos_sensor(0x0340,  imgsensor.frame_length);
        //write_cmos_sensor(0x0104, 0x00);
    }

    // Update Shutter
    //write_cmos_sensor(0x0104, 0x01);
    write_cmos_sensor(0x0202, shutter);
    //write_cmos_sensor(0x0104, 0x00);
    printk("S5KJN1_set_shutter_frame_length() shutter =%d, frame_length = %d, dummy_line = %d, auto_extend_en = %d, imgsensor.frame_length =%d\n",
        shutter, frame_length, dummy_line, auto_extend_en, imgsensor.frame_length);

}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint16 reg_gain = 0x0;

    reg_gain = gain / 2;
    return (kal_uint16) reg_gain;
}

/*************************************************************************
 * FUNCTION
 *  set_gain
 *
 * DESCRIPTION
 *  This function is to set global gain to sensor.
 *
 * PARAMETERS
 *  iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *  the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;

    /* gain=1024;//for test */
    /* return; //for test */

    if (gain < BASEGAIN || gain > MaxAnalogGain * BASEGAIN) {
        pr_debug("Error gain setting");

        if (gain < BASEGAIN)
            gain = BASEGAIN;
        else if (gain > MaxAnalogGain * BASEGAIN)
            gain = MaxAnalogGain * BASEGAIN;
    }

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    pr_debug("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

    write_cmos_sensor(0x0204, reg_gain);
    /* write_cmos_sensor_8(0x0204,(reg_gain>>8)); */
    /* write_cmos_sensor_8(0x0205,(reg_gain&0xff)); */

    return gain;
}               /*      set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{

    kal_uint8 itemp;

    pr_debug("image_mirror = %d\n", image_mirror);
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

/*************************************************************************
 * FUNCTION
 *  night_mode
 *
 * DESCRIPTION
 *  This function night mode of sensor.
 *
 * PARAMETERS
 *  bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *  None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
#if 0
static void night_mode(kal_bool enable)
{
    /*No Need to implement this function*/
}               /*      night_mode      */
#endif



/*************************************************************************
 * FUNCTION
 *  check_stremoff
 *
 * DESCRIPTION
 *  waiting function until sensor streaming finish.
 *
 * PARAMETERS
 *  None
 *
 * RETURNS
 *  None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/

static void check_streamoff(void)
{
    unsigned int i = 0;
    int timeout = (10000 / imgsensor.current_fps) + 1;

    mdelay(3);
    for (i = 0; i < timeout; i++) {
        if (read_cmos_sensor_8(0x0005) != 0xFF)
            mdelay(1);
        else
            break;
    }
    pr_debug(" %s exit! %d\n", __func__, i);
}

static kal_uint32 streaming_control(kal_bool enable)
{
    int framecnt = 0;
    pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

    if (enable) {
        mdelay(5);
        write_cmos_sensor_8(0x0100, 0x01);
        mdelay(5);
    } else {
        if (bIsLongExposure) {
            pr_debug("brad interrupt in long shutter. \n");
            write_cmos_sensor(0x6028, 0x4000);
            write_cmos_sensor_8(0x0100, 0x00); /*stream off*/

            while (1) {
                mdelay(5);
                framecnt = read_cmos_sensor_8(0x0005);
                LOG_INF(" Stream Off oning at framecnt=0x%x.\n", framecnt);
                if (framecnt == 0xFF) {
                    LOG_INF("Stream Off OK at framecnt=0x%x.\n", framecnt);
                    break;
                }
            }

            write_cmos_sensor(0x0340, imgsensor.frame_length);
            write_cmos_sensor(0x0202, 4680);
            write_cmos_sensor(0x0702, 0x0000);
            write_cmos_sensor(0x0704, 0x0000);
            //bIsLongExposure = KAL_FALSE;
        } else {
            write_cmos_sensor_8(0x0100, 0x00);
            check_streamoff();
        }
    }
    return ERROR_NONE;
}

static kal_uint16 table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
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
            puSendCmd[tosend++] = (char)(data >> 8);
            puSendCmd[tosend++] = (char)(data & 0xFF);
            IDX += 2;
            addr_last = addr;

        }

#if MULTI_WRITE
    if ((I2C_BUFFER_LEN - tosend) < 4 || IDX == len || addr != addr_last) {
        iBurstWriteReg_multi(puSendCmd, tosend,
            imgsensor.i2c_write_id, 4, 1000);

            tosend = 0;
    }
#else
        iWriteRegI2CTiming(puSendCmd, 4,
            imgsensor.i2c_write_id, imgsensor_info.i2c_speed);

        tosend = 0;
#endif

    }
    return 0;
}

#ifdef ODM_HQ_EDIT
/*Cong.Zhou@ODM_HQ Cam.Drv 20191218 for decrease poweron time*/
static const u16 uTnpArrayInit1[] = {
    0x126F,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0xE92D,
    0x41F0,
    0xF000,
    0xF9F8,
    0xB920,
    0x2201,
    0x2100,
    0x4816,
    0xF7FF,
    0xFBCA,
    0x4D16,
    0x4815,
    0x2200,
    0x63A8,
    0x6C6B,
    0x4915,
    0x4815,
    0x4798,
    0x4E15,
    0x2200,
    0x4915,
    0x63B0,
    0x6C6B,
    0x4815,
    0x4798,
    0x64F0,
    0x4814,
    0xF500,
    0x64FC,
    0xF8B0,
    0x77E0,
    0x69F0,
    0xF000,
    0xF88A,
    0x8020,
    0x6C6B,
    0x2200,
    0x4910,
    0x4810,
    0x4798,
    0x61F0,
    0x490D,
    0x8027,
    0xF643,
    0x0010,
    0xF8C1,
    0x07C4,
    0x490D,
    0x6809,
    0x8348,
    0xE8BD,
    0x81F0,
    0x0120,
    0x74FD,
    0x0120,
    0x21FE,
    0x0020,
    0x209B,
    0x0120,
    0x61FE,
    0x0100,
    0xAD63,
    0x0120,
    0x80F7,
    0x0120,
    0xF5FE,
    0x0200,
    0x7327,
    0x0120,
    0x004C,
    0x0220,
    0x4700,
    0x0200,
    0x27CF,
    0x0020,
    0x600C,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0xE92D,
    0x41F0,
    0xF000,
    0xF9AD,
    0x4DC8,
    0xF24F,
    0x545E,
    0x2100,
    0x5C68,
    0xB178,
    0xEB01,
    0x02C1,
    0xEB05,
    0x0282,
    0x320A,
    0x4620,
    0x2300,
    0xF04F,
    0x4680,
    0xF832,
    0x7B02,
    0x5237,
    0x1C5B,
    0x1C80,
    0x2B12,
    0xD3F6,
    0x3424,
    0x1C49,
    0x2909,
    0xD3E9,
    0xE8BD,
    0x81F0,
    0xB510,
    0x4604,
    0xF000,
    0xF992,
    0x7020,
    0xF000,
    0xF994,
    0x7060,
    0x48B7,
    0xF890,
    0x062E,
    0xB328,
    0x2802,
    0xD029,
    0x2001,
    0x49B4,
    0x2301,
    0xF891,
    0x15C1,
    0x4341,
    0xF3C1,
    0x0047,
    0x7821,
    0x40C8,
    0x49AF,
    0x70A0,
    0x7862,
    0xF891,
    0x15C0,
    0x0849,
    0x40D1,
    0x0042,
    0xFA03,
    0xF002,
    0x70E1,
    0x1E40,
    0x7120,
    0x0049,
    0xFA03,
    0xF001,
    0x1E40,
    0x7160,
    0x48A6,
    0xF890,
    0x062E,
    0x2800,
    0xD101,
    0x49A5,
    0x80C8,
    0xBD10,
    0x4AA4,
    0xF240,
    0x5155,
    0x2001,
    0x8011,
    0xE7D6,
    0x489F,
    0xF890,
    0x097A,
    0xE7D2,
    0x0C01,
    0x00C9,
    0xB280,
    0x4242,
    0x4010,
    0xFAB0,
    0xF080,
    0xF5A1,
    0x3142,
    0xF1C0,
    0x001F,
    0x4408,
    0xB280,
    0x4770,
    0xE92D,
    0x47F0,
    0x4998,
    0x4681,
    0x8808,
    0x4A93,
    0xF892,
    0x3171,
    0xEE00,
    0x3A10,
    0xEEB8,
    0x0A40,
    0xED9F,
    0x1A94,
    0xEE31,
    0x0A40,
    0xEEC0,
    0x0A01,
    0xEE00,
    0x0A10,
    0xEEB8,
    0x0A40,
    0xEE20,
    0x0A80,
    0xEEFC,
    0x0AC0,
    0xEE10,
    0x0A90,
    0x8008,
    0x1C8B,
    0x8818,
    0xF892,
    0x1170,
    0xEE00,
    0x1A90,
    0xEEF8,
    0x0A60,
    0xEE71,
    0x0A60,
    0xEEC0,
    0x1A81,
    0xEE00,
    0x0A90,
    0xEEF8,
    0x0A60,
    0xEE61,
    0x0AA0,
    0xEEF4,
    0x0AC0,
    0xEEF1,
    0xFA10,
    0xDC01,
    0xEEF0,
    0x0A40,
    0xEEFC,
    0x0AE0,
    0xEE10,
    0x0A90,
    0x8018,
    0x497C,
    0x1E89,
    0x8808,
    0x4692,
    0xF892,
    0x2172,
    0xEE00,
    0x2A90,
    0xEEF8,
    0x0A60,
    0xEE71,
    0x0A60,
    0xEEC0,
    0x1A81,
    0xEE00,
    0x0A90,
    0xEEF8,
    0x0A60,
    0xEE61,
    0x0AA0,
    0xEEF4,
    0x0AC0,
    0xEEF1,
    0xFA10,
    0xDD01,
    0xEEB0,
    0x0A60,
    0xEEBC,
    0x0AC0,
    0xEE10,
    0x0A10,
    0x8008,
    0x486E,
    0xF890,
    0x112C,
    0x2902,
    0xD101,
    0xF8B0,
    0x912E,
    0x4E6B,
    0xF8DF,
    0x81B0,
    0x2400,
    0x4642,
    0xF818,
    0x1004,
    0xB3C9,
    0xEA4F,
    0x00D9,
    0x2902,
    0xD103,
    0xF8D6,
    0x13FC,
    0x4348,
    0x0B40,
    0xEB02,
    0x0144,
    0x884B,
    0x4283,
    0xD301,
    0x2000,
    0xE003,
    0x88C9,
    0x4281,
    0xD803,
    0x2002,
    0x1931,
    0xF881,
    0x0A94,
    0xEB04,
    0x0044,
    0x2500,
    0xEB08,
    0x1700,
    0xE016
};

static const u16 uTnpArrayInit2[] = {
    0x126F,
    0xEB04,
    0x0344,
    0x011B,
    0xEB03,
    0x0141,
    0x1933,
    0xF893,
    0x3A94,
    0xEB03,
    0x0208,
    0x440A,
    0xEB0A,
    0x1104,
    0xEB01,
    0x0145,
    0x8952,
    0xF8B1,
    0x114E,
    0xF7FF,
    0xFAD5,
    0x1C6D,
    0x2D08,
    0xD206,
    0xEB05,
    0x0145,
    0xEB07,
    0x0041,
    0x89C0,
    0x2800,
    0xD1E1,
    0x1C64,
    0x2C02,
    0xD3BE,
    0xE8BD,
    0x87F0,
    0xE92D,
    0x4FF7,
    0x468B,
    0x4948,
    0xF249,
    0x000D,
    0x4614,
    0xF8A1,
    0x0062,
    0x4659,
    0x9800,
    0xF7FD,
    0xFF1F,
    0xF89B,
    0x0000,
    0x2802,
    0xD16E,
    0x4A42,
    0xF04F,
    0x0A01,
    0xF44F,
    0x5E80,
    0xF882,
    0xA103,
    0xF8C4,
    0xE00C,
    0x8860,
    0x88E1,
    0x0200,
    0xFBB0,
    0xF0F1,
    0x4931,
    0xB285,
    0x2000,
    0xF8B1,
    0x116E,
    0x4691,
    0x030E,
    0xF899,
    0x1104,
    0xEB00,
    0x0840,
    0xEB01,
    0x0141,
    0xEB09,
    0x1101,
    0xEB01,
    0x0388,
    0xF8B3,
    0xC10C,
    0xF8B3,
    0x1108,
    0xFBB6,
    0xFCFC,
    0xF8B3,
    0x210A,
    0xF8C3,
    0xC110,
    0xEB09,
    0x0C40,
    0xF8BC,
    0x3090,
    0xB103,
    0x4619,
    0xF8BC,
    0x3098,
    0xB103,
    0x461A,
    0x020B,
    0xFBB3,
    0xF3F2,
    0xB29B,
    0xEB09,
    0x0888,
    0xFBB6,
    0xF7F3,
    0xF8A8,
    0x11C8,
    0xEB09,
    0x0180,
    0xF8A8,
    0x21CA,
    0xF8A8,
    0x31CC,
    0x1C40,
    0xF8C1,
    0x71F8,
    0x2804,
    0xDBCC,
    0x4671,
    0x428D,
    0xD904,
    0xF889,
    0xA102,
    0x60A5,
    0x4675,
    0xE003,
    0x2200,
    0xF889,
    0x2102,
    0x60A1,
    0x9A00,
    0x08E8,
    0x6010,
    0x4A13,
    0xF8A2,
    0x01D2,
    0xF89B,
    0x0000,
    0x7020,
    0x9800,
    0xF64F,
    0x72FF,
    0x6800,
    0xEA02,
    0x00C0,
    0x80A0,
    0x032A,
    0xFBB2,
    0xF0F0,
    0x4288,
    0xD005,
    0xF889,
    0xA102,
    0x68A1,
    0x4341,
    0x0B08,
    0x60A0,
    0xE8BD,
    0x8FFE,
    0x0000,
    0x0120,
    0x00FC,
    0x0120,
    0x1088,
    0x0140,
    0x00B0,
    0x0140,
    0x06BC,
    0x0140,
    0x1C99,
    0xC842,
    0x0000,
    0x0120,
    0x7080,
    0x0120,
    0x004C,
    0x0020,
    0x0020,
    0x0120,
    0xA056,
    0x0120,
    0x00F8,
    0xF640,
    0x2C0B,
    0xF2C0,
    0x0C02,
    0x4760,
    0xF24D,
    0x2C0F,
    0xF2C0,
    0x0C02,
    0x4760,
    0xF24D,
    0x3C97,
    0xF2C0,
    0x0C02,
    0x4760,
    0xF24D,
    0x3C8F,
    0xF2C0,
    0x0C02,
    0x4760
};
#endif
#ifndef ODM_HQ_EDIT
/*Cong.Zhou@ODM_HQ Cam.Drv 20191218 for decrease poweron time*/
static kal_uint16 addr_data_pair_init_jn1_SWpage[] = {
    0x6028, 0x2400,
    0x602A, 0x7700,
    0x6F12, 0x1753,
    0x6F12, 0x01FC,
    0x6F12, 0xE702,
    0x6F12, 0x6385,
    0x6F12, 0xB787,
    0x6F12, 0x0024,
    0x6F12, 0x9386,
    0x6F12, 0xC701,
    0x6F12, 0xB7B7,
    0x6F12, 0x0024,
    0x6F12, 0x3777,
    0x6F12, 0x0024,
    0x6F12, 0x9387,
    0x6F12, 0xC77F,
    0x6F12, 0x1307,
    0x6F12, 0x07C7,
    0x6F12, 0x958F,
    0x6F12, 0x2328,
    0x6F12, 0xD774,
    0x6F12, 0x231A,
    0x6F12, 0xF774,
    0x6F12, 0x012F,
    0x6F12, 0xB777,
    0x6F12, 0x0024,
    0x6F12, 0x1307,
    0x6F12, 0xD003,
    0x6F12, 0x23A0,
    0x6F12, 0xE774,
    0x6F12, 0x1753,
    0x6F12, 0x01FC,
    0x6F12, 0x6700,
    0x6F12, 0x2384,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x8547,
    0x6F12, 0x6310,
    0x6F12, 0xF506,
    0x6F12, 0x3786,
    0x6F12, 0x0024,
    0x6F12, 0x9306,
    0x6F12, 0x4601,
    0x6F12, 0x83C7,
    0x6F12, 0x4600,
    0x6F12, 0xA1CB,
    0x6F12, 0xB737,
    0x6F12, 0x0024,
    0x6F12, 0x83A7,
    0x6F12, 0x875C,
    0x6F12, 0x83D6,
    0x6F12, 0x2600,
    0x6F12, 0x83D7,
    0x6F12, 0x271E,
    0x6F12, 0x13D7,
    0x6F12, 0x2700,
    0x6F12, 0xB707,
    0x6F12, 0x0140,
    0x6F12, 0x83D5,
    0x6F12, 0x27F0,
    0x6F12, 0x8357,
    0x6F12, 0x4601,
    0x6F12, 0x1306,
    0x6F12, 0xE7FF,
    0x6F12, 0xB697,
    0x6F12, 0x8D8F,
    0x6F12, 0xC207,
    0x6F12, 0xC183,
    0x6F12, 0x9396,
    0x6F12, 0x0701,
    0x6F12, 0xC186,
    0x6F12, 0x635F,
    0x6F12, 0xD600,
    0x6F12, 0x8907,
    0x6F12, 0x998F,
    0x6F12, 0x9396,
    0x6F12, 0x0701,
    0x6F12, 0xC186,
    0x6F12, 0x9397,
    0x6F12, 0x0601,
    0x6F12, 0xC183,
    0x6F12, 0x37B7,
    0x6F12, 0x0040,
    0x6F12, 0x2311,
    0x6F12, 0xF7A0,
    0x6F12, 0x8280,
    0x6F12, 0xE3D8,
    0x6F12, 0x06FE,
    0x6F12, 0xBA97,
    0x6F12, 0xF917,
    0x6F12, 0xCDB7,
    0x6F12, 0xB717,
    0x6F12, 0x0024,
    0x6F12, 0x9387,
    0x6F12, 0x07CA,
    0x6F12, 0xAA97,
    0x6F12, 0x3387,
    0x6F12, 0xB700,
    0x6F12, 0x8D8F,
    0x6F12, 0x83C5,
    0x6F12, 0xD705,
    0x6F12, 0xB747,
    0x6F12, 0x0024,
    0x6F12, 0x9386,
    0x6F12, 0x078F,
    0x6F12, 0x83D7,
    0x6F12, 0xE670,
    0x6F12, 0x0905,
    0x6F12, 0x0347,
    0x6F12, 0xC705,
    0x6F12, 0x630B,
    0x6F12, 0xF500,
    0x6F12, 0x8567,
    0x6F12, 0xB697,
    0x6F12, 0x03A6,
    0x6F12, 0x8794,
    0x6F12, 0xB306,
    0x6F12, 0xB700,
    0x6F12, 0xB296,
    0x6F12, 0x23A4,
    0x6F12, 0xD794,
    0x6F12, 0x2207,
    0x6F12, 0x3305,
    0x6F12, 0xB700,
    0x6F12, 0x4205,
    0x6F12, 0x4181,
    0x6F12, 0x8280,
    0x6F12, 0x5D71,
    0x6F12, 0xA2C6,
    0x6F12, 0xA6C4,
    0x6F12, 0x7324,
    0x6F12, 0x2034,
    0x6F12, 0xF324,
    0x6F12, 0x1034,
    0x6F12, 0x7360,
    0x6F12, 0x0430,
    0x6F12, 0x2AD8,
    0x6F12, 0x2ED6,
    0x6F12, 0x3545,
    0x6F12, 0x9305,
    0x6F12, 0x8008,
    0x6F12, 0x22DA,
    0x6F12, 0x3ECE,
    0x6F12, 0x86C2,
    0x6F12, 0x96C0,
    0x6F12, 0x1ADE,
    0x6F12, 0x1EDC,
    0x6F12, 0x32D4,
    0x6F12, 0x36D2,
    0x6F12, 0x3AD0,
    0x6F12, 0x42CC,
    0x6F12, 0x46CA,
    0x6F12, 0x72C8,
    0x6F12, 0x76C6,
    0x6F12, 0x7AC4,
    0x6F12, 0x7EC2,
    0x6F12, 0x9710,
    0x6F12, 0x00FC,
    0x6F12, 0xE780,
    0x6F12, 0x40F5,
    0x6F12, 0x9377,
    0x6F12, 0x8500,
    0x6F12, 0x2A84,
    0x6F12, 0x85C3,
    0x6F12, 0xB737,
    0x6F12, 0x0024,
    0x6F12, 0x9387,
    0x6F12, 0x077C,
    0x6F12, 0x03D7,
    0x6F12, 0x6702,
    0x6F12, 0x0507,
    0x6F12, 0x2393,
    0x6F12, 0xE702,
    0x6F12, 0x9710,
    0x6F12, 0x00FC,
    0x6F12, 0xE780,
    0x6F12, 0x00ED,
    0x6F12, 0x0545,
    0x6F12, 0xD535,
    0x6F12, 0x1374,
    0x6F12, 0x0408,
    0x6F12, 0x11CC,
    0x6F12, 0xB737,
    0x6F12, 0x0024,
    0x6F12, 0x9387,
    0x6F12, 0x077C,
    0x6F12, 0x03D7,
    0x6F12, 0x8705,
    0x6F12, 0x0507,
    0x6F12, 0x239C,
    0x6F12, 0xE704,
    0x6F12, 0x9710,
    0x6F12, 0x00FC,
    0x6F12, 0xE780,
    0x6F12, 0x6065,
    0x6F12, 0x9640,
    0x6F12, 0x8642,
    0x6F12, 0x7253,
    0x6F12, 0xE253,
    0x6F12, 0x5254,
    0x6F12, 0x4255,
    0x6F12, 0xB255,
    0x6F12, 0x2256,
    0x6F12, 0x9256,
    0x6F12, 0x0257,
    0x6F12, 0xF247,
    0x6F12, 0x6248,
    0x6F12, 0xD248,
    0x6F12, 0x424E,
    0x6F12, 0xB24E,
    0x6F12, 0x224F,
    0x6F12, 0x924F,
    0x6F12, 0x7370,
    0x6F12, 0x0430,
    0x6F12, 0x7390,
    0x6F12, 0x1434,
    0x6F12, 0x7310,
    0x6F12, 0x2434,
    0x6F12, 0x3644,
    0x6F12, 0xA644,
    0x6F12, 0x6161,
    0x6F12, 0x7300,
    0x6F12, 0x2030,
    0x6F12, 0x1743,
    0x6F12, 0x01FC,
    0x6F12, 0xE702,
    0x6F12, 0xC369,
    0x6F12, 0xB787,
    0x6F12, 0x0024,
    0x6F12, 0x83A4,
    0x6F12, 0xC700,
    0x6F12, 0x2A84,
    0x6F12, 0x0146,
    0x6F12, 0xA685,
    0x6F12, 0x1145,
    0x6F12, 0x9790,
    0x6F12, 0xFFFB,
    0x6F12, 0xE780,
    0x6F12, 0x60AD,
    0x6F12, 0x2285,
    0x6F12, 0x97E0,
    0x6F12, 0xFFFB,
    0x6F12, 0xE780,
    0x6F12, 0x40A9,
    0x6F12, 0xB787,
    0x6F12, 0x0024,
    0x6F12, 0x9387,
    0x6F12, 0x87F5,
    0x6F12, 0x03C7,
    0x6F12, 0x0700,
    0x6F12, 0x8546,
    0x6F12, 0x6315,
    0x6F12, 0xD706,
    0x6F12, 0xB776,
    0x6F12, 0x0024,
    0x6F12, 0x03A6,
    0x6F12, 0x468D,
    0x6F12, 0x8945,
    0x6F12, 0x9386,
    0x6F12, 0xA700,
    0x6F12, 0x630F,
    0x6F12, 0xB600,
    0x6F12, 0x9386,
    0x6F12, 0x2700,
    0x6F12, 0x630B,
    0x6F12, 0xE600,
    0x6F12, 0x3717,
    0x6F12, 0x0024,
    0x6F12, 0x0356,
    0x6F12, 0x6738,
    0x6F12, 0x2D47,
    0x6F12, 0x6314,
    0x6F12, 0xE600,
    0x6F12, 0x9386,
    0x6F12, 0xA700,
    0x6F12, 0xB747,
    0x6F12, 0x0024,
    0x6F12, 0x83A5,
    0x6F12, 0xC786,
    0x6F12, 0x2946,
    0x6F12, 0x1305,
    0x6F12, 0x0404,
    0x6F12, 0x83C7,
    0x6F12, 0xC52F,
    0x6F12, 0x2148,
    0x6F12, 0x1D8E,
    0x6F12, 0x8147,
    0x6F12, 0x3387,
    0x6F12, 0xF500,
    0x6F12, 0xB388,
    0x6F12, 0xF600,
    0x6F12, 0x0317,
    0x6F12, 0xE73C,
    0x6F12, 0x8398,
    0x6F12, 0x0800,
    0x6F12, 0x8907,
    0x6F12, 0x1105,
    0x6F12, 0x4697,
    0x6F12, 0x3317,
    0x6F12, 0xC700,
    0x6F12, 0x232E,
    0x6F12, 0xE5FE,
    0x6F12, 0xE391,
    0x6F12, 0x07FF,
    0x6F12, 0x0546,
    0x6F12, 0xA685,
    0x6F12, 0x1145,
    0x6F12, 0x9790,
    0x6F12, 0xFFFB,
    0x6F12, 0xE780,
    0x6F12, 0x60A4,
    0x6F12, 0x1743,
    0x6F12, 0x01FC,
    0x6F12, 0x6700,
    0x6F12, 0x0361,
    0x6F12, 0x1743,
    0x6F12, 0x01FC,
    0x6F12, 0xE702,
    0x6F12, 0xA35C,
    0x6F12, 0xB787,
    0x6F12, 0x0024,
    0x6F12, 0x83A4,
    0x6F12, 0x8700,
    0x6F12, 0x4111,
    0x6F12, 0xAA89,
    0x6F12, 0x2E8A,
    0x6F12, 0xB28A,
    0x6F12, 0xA685,
    0x6F12, 0x0146,
    0x6F12, 0x1145,
    0x6F12, 0x36C6,
    0x6F12, 0x9790,
    0x6F12, 0xFFFB,
    0x6F12, 0xE780,
    0x6F12, 0x60A1,
    0x6F12, 0xB787,
    0x6F12, 0x0024,
    0x6F12, 0x1387,
    0x6F12, 0x87F5,
    0x6F12, 0x0347,
    0x6F12, 0x2701,
    0x6F12, 0x1384,
    0x6F12, 0x87F5,
    0x6F12, 0xB246,
    0x6F12, 0x0149,
    0x6F12, 0x11CF,
    0x6F12, 0x3767,
    0x6F12, 0x0024,
    0x6F12, 0x0357,
    0x6F12, 0x2777,
    0x6F12, 0xB777,
    0x6F12, 0x0024,
    0x6F12, 0x9387,
    0x6F12, 0x07C7,
    0x6F12, 0x0E07,
    0x6F12, 0x03D9,
    0x6F12, 0x871C,
    0x6F12, 0x2394,
    0x6F12, 0xE71C,
    0x6F12, 0x5686,
    0x6F12, 0xD285,
    0x6F12, 0x4E85,
    0x6F12, 0x97E0,
    0x6F12, 0xFFFB,
    0x6F12, 0xE780,
    0x6F12, 0x0088,
    0x6F12, 0x8347,
    0x6F12, 0x2401,
    0x6F12, 0x89C7,
    0x6F12, 0xB777,
    0x6F12, 0x0024,
    0x6F12, 0x239C,
    0x6F12, 0x27E3,
    0x6F12, 0x0546,
    0x6F12, 0xA685,
    0x6F12, 0x1145,
    0x6F12, 0x9790,
    0x6F12, 0xFFFB,
    0x6F12, 0xE780,
    0x6F12, 0xC09B,
    0x6F12, 0x4101,
    0x6F12, 0x1743,
    0x6F12, 0x01FC,
    0x6F12, 0x6700,
    0x6F12, 0xA357,
    0x6F12, 0x1743,
    0x6F12, 0x01FC,
    0x6F12, 0xE702,
    0x6F12, 0x0353,
    0x6F12, 0x3784,
    0x6F12, 0x0024,
    0x6F12, 0x9307,
    0x6F12, 0x04F4,
    0x6F12, 0x83C7,
    0x6F12, 0x4701,
    0x6F12, 0x5D71,
    0x6F12, 0x2A89,
    0x6F12, 0x8DEF,
    0x6F12, 0xB787,
    0x6F12, 0x0024,
    0x6F12, 0x03A4,
    0x6F12, 0x4700,
    0x6F12, 0x0146,
    0x6F12, 0x1145,
    0x6F12, 0xA285,
    0x6F12, 0x9790,
    0x6F12, 0xFFFB,
    0x6F12, 0xE780,
    0x6F12, 0x2098,
    0x6F12, 0x4A85,
    0x6F12, 0x97E0,
    0x6F12, 0xFFFB,
    0x6F12, 0xE780,
    0x6F12, 0x400F,
    0x6F12, 0x0546,
    0x6F12, 0xA285,
    0x6F12, 0x1145,
    0x6F12, 0x9790,
    0x6F12, 0xFFFB,
    0x6F12, 0xE780,
    0x6F12, 0xA096,
    0x6F12, 0x6161,
    0x6F12, 0x1743,
    0x6F12, 0x01FC,
    0x6F12, 0x6700,
    0x6F12, 0xE351,
    0x6F12, 0x1304,
    0x6F12, 0x04F4,
    0x6F12, 0x0347,
    0x6F12, 0x5401,
    0x6F12, 0x8547,
    0x6F12, 0x6310,
    0x6F12, 0xF716,
    0x6F12, 0xB785,
    0x6F12, 0x0024,
    0x6F12, 0x9384,
    0x6F12, 0x05F8,
    0x6F12, 0x4146,
    0x6F12, 0x9385,
    0x6F12, 0x05F8,
    0x6F12, 0x0A85,
    0x6F12, 0x9740,
    0x6F12, 0x01FC,
    0x6F12, 0xE780,
    0x6F12, 0x6058,
    0x6F12, 0x4146,
    0x6F12, 0x9385,
    0x6F12, 0x0401,
    0x6F12, 0x0808,
    0x6F12, 0x9740,
    0x6F12, 0x01FC,
    0x6F12, 0xE780,
    0x6F12, 0x6057,
    0x6F12, 0x4146,
    0x6F12, 0x9385,
    0x6F12, 0x0402,
    0x6F12, 0x0810,
    0x6F12, 0x9740,
    0x6F12, 0x01FC,
    0x6F12, 0xE780,
    0x6F12, 0x6056,
    0x6F12, 0x4146,
    0x6F12, 0x9385,
    0x6F12, 0x0403,
    0x6F12, 0x0818,
    0x6F12, 0x9740,
    0x6F12, 0x01FC,
    0x6F12, 0xE780,
    0x6F12, 0x6055,
    0x6F12, 0x1C40,
    0x6F12, 0xB7DB,
    0x6F12, 0x0040,
    0x6F12, 0x014B,
    0x6F12, 0xBEC0,
    0x6F12, 0x5C40,
    0x6F12, 0x8149,
    0x6F12, 0x854A,
    0x6F12, 0xBEC2,
    0x6F12, 0x5C44,
    0x6F12, 0x096D,
    0x6F12, 0x370C,
    0x6F12, 0x0040,
    0x6F12, 0xBEC4,
    0x6F12, 0x1C48,
    0x6F12, 0x938B,
    0x6F12, 0x0B03,
    0x6F12, 0x930C,
    0x6F12, 0x0004,
    0x6F12, 0xBEC6,
    0x6F12, 0x0347,
    0x6F12, 0x7401,
    0x6F12, 0x8967,
    0x6F12, 0x631B,
    0x6F12, 0x5701,
    0x6F12, 0x93F7,
    0x6F12, 0xF900,
    0x6F12, 0x9808,
    0x6F12, 0xBA97,
    0x6F12, 0x83C7,
    0x6F12, 0x07FB,
    0x6F12, 0x8A07,
    0x6F12, 0xCA97,
    0x6F12, 0x9C43,
    0x6F12, 0x63DE,
    0x6F12, 0x3A01,
    0x6F12, 0x1387,
    0x6F12, 0x69FE,
    0x6F12, 0x63FA,
    0x6F12, 0xEA00,
    0x6F12, 0x1387,
    0x6F12, 0xA9FD,
    0x6F12, 0x63F6,
    0x6F12, 0xEA00,
    0x6F12, 0x1387,
    0x6F12, 0x49FC,
    0x6F12, 0x63E3,
    0x6F12, 0xEA0A,
    0x6F12, 0x131A,
    0x6F12, 0x1B00,
    0x6F12, 0x9808,
    0x6F12, 0x5297,
    0x6F12, 0x8354,
    0x6F12, 0x07FF,
    0x6F12, 0x0145,
    0x6F12, 0xB384,
    0x6F12, 0xF402,
    0x6F12, 0xB580,
    0x6F12, 0x637A,
    0x6F12, 0x9D00,
    0x6F12, 0x2685,
    0x6F12, 0x9780,
    0x6F12, 0xFFFB,
    0x6F12, 0xE780,
    0x6F12, 0x6052,
    0x6F12, 0x4915,
    0x6F12, 0x1375,
    0x6F12, 0xF50F,
    0x6F12, 0x9C08,
    0x6F12, 0xD297,
    0x6F12, 0x03D7,
    0x6F12, 0x07FE,
    0x6F12, 0xB3D4,
    0x6F12, 0xA400,
    0x6F12, 0xC204,
    0x6F12, 0x6297,
    0x6F12, 0xC180,
    0x6F12, 0x2310,
    0x6F12, 0x9700,
    0x6F12, 0x03D7,
    0x6F12, 0x07FC,
    0x6F12, 0x83D7,
    0x6F12, 0x07FD,
    0x6F12, 0x050B,
    0x6F12, 0x6297,
    0x6F12, 0x8356,
    0x6F12, 0x0700,
    0x6F12, 0x3315,
    0x6F12, 0xF500,
    0x6F12, 0x558D,
    0x6F12, 0x4205,
    0x6F12, 0x4181,
    0x6F12, 0x2310,
    0x6F12, 0xA700,
    0x6F12, 0x8509,
    0x6F12, 0xE395,
    0x6F12, 0x99F7,
    0x6F12, 0x8D47,
    0x6F12, 0x37D4,
    0x6F12, 0x0040,
    0x6F12, 0x2319,
    0x6F12, 0xF40A,
    0x6F12, 0x2316,
    0x6F12, 0x040C,
    0x6F12, 0x9790,
    0x6F12, 0x00FC,
    0x6F12, 0xE780,
    0x6F12, 0x6093,
    0x6F12, 0xAA84,
    0x6F12, 0x9790,
    0x6F12, 0x00FC,
    0x6F12, 0xE780,
    0x6F12, 0x2092,
    0x6F12, 0x9307,
    0x6F12, 0x0008,
    0x6F12, 0x33D5,
    0x6F12, 0xA700,
    0x6F12, 0x9307,
    0x6F12, 0x0004,
    0x6F12, 0x1205,
    0x6F12, 0xB3D7,
    0x6F12, 0x9700,
    0x6F12, 0x1375,
    0x6F12, 0x0503,
    0x6F12, 0x8D8B,
    0x6F12, 0x5D8D,
    0x6F12, 0x2319,
    0x6F12, 0xA40C,
    0x6F12, 0x45B5,
    0x6F12, 0x1397,
    0x6F12, 0x1900,
    0x6F12, 0x9394,
    0x6F12, 0x0701,
    0x6F12, 0x5E97,
    0x6F12, 0xC180,
    0x6F12, 0x2310,
    0x6F12, 0x9700,
    0x6F12, 0x6DB7,
    0x6F12, 0x0347,
    0x6F12, 0x6401,
    0x6F12, 0xE315,
    0x6F12, 0xF7FA,
    0x6F12, 0xB784,
    0x6F12, 0x0024,
    0x6F12, 0x9384,
    0x6F12, 0x04F8,
    0x6F12, 0x4146,
    0x6F12, 0x9385,
    0x6F12, 0x0404,
    0x6F12, 0x0A85,
    0x6F12, 0x9740,
    0x6F12, 0x01FC,
    0x6F12, 0xE780,
    0x6F12, 0x2042,
    0x6F12, 0x4146,
    0x6F12, 0x9385,
    0x6F12, 0x0405,
    0x6F12, 0x0808,
    0x6F12, 0x9740,
    0x6F12, 0x01FC,
    0x6F12, 0xE780,
    0x6F12, 0x2041,
    0x6F12, 0x4146,
    0x6F12, 0x9385,
    0x6F12, 0x0406,
    0x6F12, 0x0810,
    0x6F12, 0x9740,
    0x6F12, 0x01FC,
    0x6F12, 0xE780,
    0x6F12, 0x2040,
    0x6F12, 0x4146,
    0x6F12, 0x9385,
    0x6F12, 0x0407,
    0x6F12, 0x0818,
    0x6F12, 0x9740,
    0x6F12, 0x01FC,
    0x6F12, 0xE780,
    0x6F12, 0x203F,
    0x6F12, 0x0357,
    0x6F12, 0x0400,
    0x6F12, 0x8357,
    0x6F12, 0x2400,
    0x6F12, 0x37DB,
    0x6F12, 0x0040,
    0x6F12, 0x2310,
    0x6F12, 0xE104,
    0x6F12, 0x2311,
    0x6F12, 0xF104,
    0x6F12, 0x2312,
    0x6F12, 0xE104,
    0x6F12, 0x2313,
    0x6F12, 0xF104,
    0x6F12, 0x0357,
    0x6F12, 0x8400,
    0x6F12, 0x8357,
    0x6F12, 0xA400,
    0x6F12, 0x814A,
    0x6F12, 0x2314,
    0x6F12, 0xE104,
    0x6F12, 0x2315,
    0x6F12, 0xF104,
    0x6F12, 0x2316,
    0x6F12, 0xE104,
    0x6F12, 0x2317,
    0x6F12, 0xF104,
    0x6F12, 0x8149,
    0x6F12, 0x854B,
    0x6F12, 0x096D,
    0x6F12, 0x130B,
    0x6F12, 0x0B03,
    0x6F12, 0x370C,
    0x6F12, 0x0040,
    0x6F12, 0x930C,
    0x6F12, 0x0004,
    0x6F12, 0x0347,
    0x6F12, 0x7401,
    0x6F12, 0x8967,
    0x6F12, 0x631B,
    0x6F12, 0x7701,
    0x6F12, 0x93F7,
    0x6F12, 0xF900,
    0x6F12, 0x9808,
    0x6F12, 0xBA97,
    0x6F12, 0x83C7,
    0x6F12, 0x07FB,
    0x6F12, 0x8A07,
    0x6F12, 0xCA97,
    0x6F12, 0x9C43,
    0x6F12, 0x63C4,
    0x6F12, 0x3B07,
    0x6F12, 0x139A,
    0x6F12, 0x1A00,
    0x6F12, 0x9808,
    0x6F12, 0x5297,
    0x6F12, 0x8354,
    0x6F12, 0x07FF,
    0x6F12, 0x0145,
    0x6F12, 0xB384,
    0x6F12, 0xF402,
    0x6F12, 0xB580,
    0x6F12, 0x637A,
    0x6F12, 0x9D00,
    0x6F12, 0x2685,
    0x6F12, 0x9780,
    0x6F12, 0xFFFB,
    0x6F12, 0xE780,
    0x6F12, 0xA03B,
    0x6F12, 0x4915,
    0x6F12, 0x1375,
    0x6F12, 0xF50F,
    0x6F12, 0x9C08,
    0x6F12, 0xD297,
    0x6F12, 0x03D7,
    0x6F12, 0x07FE,
    0x6F12, 0xB3D4,
    0x6F12, 0xA400,
    0x6F12, 0xC204,
    0x6F12, 0x6297,
    0x6F12, 0xC180,
    0x6F12, 0x2310,
    0x6F12, 0x9700,
    0x6F12, 0x03D7,
    0x6F12, 0x07FC,
    0x6F12, 0x83D7,
    0x6F12, 0x07FD,
    0x6F12, 0x850A,
    0x6F12, 0x6297,
    0x6F12, 0x8356,
    0x6F12, 0x0700,
    0x6F12, 0x3315,
    0x6F12, 0xF500,
    0x6F12, 0x558D,
    0x6F12, 0x4205,
    0x6F12, 0x4181,
    0x6F12, 0x2310,
    0x6F12, 0xA700,
    0x6F12, 0x8509,
    0x6F12, 0xE391,
    0x6F12, 0x99F9,
    0x6F12, 0x51BD,
    0x6F12, 0x1397,
    0x6F12, 0x1900,
    0x6F12, 0x9394,
    0x6F12, 0x0701,
    0x6F12, 0x5A97,
    0x6F12, 0xC180,
    0x6F12, 0x2310,
    0x6F12, 0x9700,
    0x6F12, 0xE5B7,
    0x6F12, 0x1743,
    0x6F12, 0x01FC,
    0x6F12, 0xE702,
    0x6F12, 0xE326,
    0x6F12, 0xB737,
    0x6F12, 0x0024,
    0x6F12, 0x83A7,
    0x6F12, 0x0761,
    0x6F12, 0xAA84,
    0x6F12, 0x2E89,
    0x6F12, 0x8297,
    0x6F12, 0xB787,
    0x6F12, 0x0024,
    0x6F12, 0x03A4,
    0x6F12, 0x0700,
    0x6F12, 0x0146,
    0x6F12, 0x1145,
    0x6F12, 0xA285,
    0x6F12, 0x9780,
    0x6F12, 0xFFFB,
    0x6F12, 0xE780,
    0x6F12, 0xC069,
    0x6F12, 0xCA85,
    0x6F12, 0x2685,
    0x6F12, 0x9730,
    0x6F12, 0x00FC,
    0x6F12, 0xE780,
    0x6F12, 0x20F5,
    0x6F12, 0x0546,
    0x6F12, 0xA285,
    0x6F12, 0x1145,
    0x6F12, 0x9780,
    0x6F12, 0xFFFB,
    0x6F12, 0xE780,
    0x6F12, 0x2068,
    0x6F12, 0x1743,
    0x6F12, 0x01FC,
    0x6F12, 0x6700,
    0x6F12, 0xC324,
    0x6F12, 0xB717,
    0x6F12, 0x0024,
    0x6F12, 0x83C7,
    0x6F12, 0x0734,
    0x6F12, 0xEDCF,
    0x6F12, 0x1743,
    0x6F12, 0x01FC,
    0x6F12, 0xE702,
    0x6F12, 0x6321,
    0x6F12, 0x9780,
    0x6F12, 0x00FC,
    0x6F12, 0xE780,
    0x6F12, 0x803E,
    0x6F12, 0xB787,
    0x6F12, 0x0024,
    0x6F12, 0x1387,
    0x6F12, 0x87F5,
    0x6F12, 0x0347,
    0x6F12, 0xF701,
    0x6F12, 0x9387,
    0x6F12, 0x87F5,
    0x6F12, 0x19EB,
    0x6F12, 0x37F7,
    0x6F12, 0x0040,
    0x6F12, 0x8356,
    0x6F12, 0x6772,
    0x6F12, 0x2391,
    0x6F12, 0xD702,
    0x6F12, 0x0357,
    0x6F12, 0xA772,
    0x6F12, 0x2392,
    0x6F12, 0xE702,
    0x6F12, 0xB776,
    0x6F12, 0x0024,
    0x6F12, 0x83C6,
    0x6F12, 0xB6F1,
    0x6F12, 0x0547,
    0x6F12, 0xA38F,
    0x6F12, 0xE700,
    0x6F12, 0x99C6,
    0x6F12, 0x83D6,
    0x6F12, 0x4701,
    0x6F12, 0x238F,
    0x6F12, 0xE700,
    0x6F12, 0x2380,
    0x6F12, 0xD702,
    0x6F12, 0x83C6,
    0x6F12, 0x0702,
    0x6F12, 0x03C7,
    0x6F12, 0xE701,
    0x6F12, 0xB9CE,
    0x6F12, 0x0DC3,
    0x6F12, 0x03D7,
    0x6F12, 0x6701,
    0x6F12, 0x0DCF,
    0x6F12, 0xB7F6,
    0x6F12, 0x0040,
    0x6F12, 0x2393,
    0x6F12, 0xE672,
    0x6F12, 0x03D7,
    0x6F12, 0x8701,
    0x6F12, 0x0DCF,
    0x6F12, 0xB7F6,
    0x6F12, 0x0040,
    0x6F12, 0x2395,
    0x6F12, 0xE672,
    0x6F12, 0x238F,
    0x6F12, 0x0700,
    0x6F12, 0x03C7,
    0x6F12, 0x0702,
    0x6F12, 0x7D17,
    0x6F12, 0x1377,
    0x6F12, 0xF70F,
    0x6F12, 0x2380,
    0x6F12, 0xE702,
    0x6F12, 0x01E7,
    0x6F12, 0x0547,
    0x6F12, 0x238F,
    0x6F12, 0xE700,
    0x6F12, 0x1743,
    0x6F12, 0x01FC,
    0x6F12, 0x6700,
    0x6F12, 0x631A,
    0x6F12, 0x83D6,
    0x6F12, 0x2702,
    0x6F12, 0x37F7,
    0x6F12, 0x0040,
    0x6F12, 0x2313,
    0x6F12, 0xD772,
    0x6F12, 0xD1B7,
    0x6F12, 0x83D6,
    0x6F12, 0x4702,
    0x6F12, 0x37F7,
    0x6F12, 0x0040,
    0x6F12, 0x2315,
    0x6F12, 0xD772,
    0x6F12, 0xD1B7,
    0x6F12, 0x71DF,
    0x6F12, 0x03D7,
    0x6F12, 0xA701,
    0x6F12, 0x19CF,
    0x6F12, 0xB7F6,
    0x6F12, 0x0040,
    0x6F12, 0x2393,
    0x6F12, 0xE672,
    0x6F12, 0x03D7,
    0x6F12, 0xC701,
    0x6F12, 0x19CF,
    0x6F12, 0xB7F6,
    0x6F12, 0x0040,
    0x6F12, 0x2395,
    0x6F12, 0xE672,
    0x6F12, 0x238F,
    0x6F12, 0x0700,
    0x6F12, 0x6DBF,
    0x6F12, 0x83D6,
    0x6F12, 0x2702,
    0x6F12, 0x37F7,
    0x6F12, 0x0040,
    0x6F12, 0x2313,
    0x6F12, 0xD772,
    0x6F12, 0xC5B7,
    0x6F12, 0x83D6,
    0x6F12, 0x4702,
    0x6F12, 0x37F7,
    0x6F12, 0x0040,
    0x6F12, 0x2315,
    0x6F12, 0xD772,
    0x6F12, 0xC5B7,
    0x6F12, 0x8280,
    0x6F12, 0x1743,
    0x6F12, 0x01FC,
    0x6F12, 0xE702,
    0x6F12, 0xC311,
    0x6F12, 0xB787,
    0x6F12, 0x0024,
    0x6F12, 0xA166,
    0x6F12, 0xB775,
    0x6F12, 0x0024,
    0x6F12, 0x9386,
    0x6F12, 0x76F7,
    0x6F12, 0x3777,
    0x6F12, 0x0024,
    0x6F12, 0x9387,
    0x6F12, 0xC701,
    0x6F12, 0x2946,
    0x6F12, 0x9385,
    0x6F12, 0xA57F,
    0x6F12, 0x3545,
    0x6F12, 0x2320,
    0x6F12, 0xF73C,
    0x6F12, 0x9710,
    0x6F12, 0x00FC,
    0x6F12, 0xE780,
    0x6F12, 0xA01F,
    0x6F12, 0xB777,
    0x6F12, 0x0024,
    0x6F12, 0xB785,
    0x6F12, 0x0024,
    0x6F12, 0x3755,
    0x6F12, 0x0020,
    0x6F12, 0x3737,
    0x6F12, 0x0024,
    0x6F12, 0x9387,
    0x6F12, 0x4774,
    0x6F12, 0x0146,
    0x6F12, 0x9385,
    0x6F12, 0xA58B,
    0x6F12, 0x1305,
    0x6F12, 0x0537,
    0x6F12, 0x232C,
    0x6F12, 0xF75E,
    0x6F12, 0x9710,
    0x6F12, 0x00FC,
    0x6F12, 0xE780,
    0x6F12, 0x4058,
    0x6F12, 0xB787,
    0x6F12, 0x0024,
    0x6F12, 0x23A6,
    0x6F12, 0xA700,
    0x6F12, 0xB785,
    0x6F12, 0x0024,
    0x6F12, 0x3765,
    0x6F12, 0x0020,
    0x6F12, 0x0146,
    0x6F12, 0x9385,
    0x6F12, 0xE59F,
    0x6F12, 0x1305,
    0x6F12, 0x45B2,
    0x6F12, 0x9710,
    0x6F12, 0x00FC,
    0x6F12, 0xE780,
    0x6F12, 0x2056,
    0x6F12, 0xB787,
    0x6F12, 0x0024,
    0x6F12, 0x23A2,
    0x6F12, 0xA700,
    0x6F12, 0xB785,
    0x6F12, 0x0024,
    0x6F12, 0x3755,
    0x6F12, 0x0020,
    0x6F12, 0x0146,
    0x6F12, 0x9385,
    0x6F12, 0x2597,
    0x6F12, 0x1305,
    0x6F12, 0x0525,
    0x6F12, 0x9710,
    0x6F12, 0x00FC,
    0x6F12, 0xE780,
    0x6F12, 0x0054,
    0x6F12, 0xB787,
    0x6F12, 0x0024,
    0x6F12, 0x23A4,
    0x6F12, 0xA700,
    0x6F12, 0xB775,
    0x6F12, 0x0024,
    0x6F12, 0x3775,
    0x6F12, 0x0020,
    0x6F12, 0x0146,
    0x6F12, 0x9385,
    0x6F12, 0x257B,
    0x6F12, 0x1305,
    0x6F12, 0x85D3,
    0x6F12, 0x9710,
    0x6F12, 0x00FC,
    0x6F12, 0xE780,
    0x6F12, 0xE051,
    0x6F12, 0xB787,
    0x6F12, 0x0024,
    0x6F12, 0x23A8,
    0x6F12, 0xA700,
    0x6F12, 0xB785,
    0x6F12, 0x0024,
    0x6F12, 0x37B5,
    0x6F12, 0x0020,
    0x6F12, 0x0146,
    0x6F12, 0x9385,
    0x6F12, 0x85CE,
    0x6F12, 0x1305,
    0x6F12, 0xA5C6,
    0x6F12, 0x9710,
    0x6F12, 0x00FC,
    0x6F12, 0xE780,
    0x6F12, 0xC04F,
    0x6F12, 0xB787,
    0x6F12, 0x0024,
    0x6F12, 0x23A0,
    0x6F12, 0xA700,
    0x6F12, 0xB787,
    0x6F12, 0x0024,
    0x6F12, 0x3737,
    0x6F12, 0x0024,
    0x6F12, 0x9387,
    0x6F12, 0x67D3,
    0x6F12, 0x2320,
    0x6F12, 0xF768,
    0x6F12, 0x1743,
    0x6F12, 0x01FC,
    0x6F12, 0x6700,
    0x6F12, 0x4304,
    0x6F12, 0x0000,
    0x6F12, 0x0020,
    0x6F12, 0x0020,
    0x6F12, 0x0020,
    0x6F12, 0x0020,
    0x6F12, 0x0020,
    0x6F12, 0x0020,
    0x6F12, 0x0020,
    0x6F12, 0x0020,
    0x6F12, 0x0020,
    0x6F12, 0x0020,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0001,
    0x6F12, 0x0203,
    0x6F12, 0x0001,
    0x6F12, 0x0203,
    0x6F12, 0x0405,
    0x6F12, 0x0607,
    0x6F12, 0x0405,
    0x6F12, 0x0607,
    0x6F12, 0x10D0,
    0x6F12, 0x10D0,
    0x6F12, 0x1CD0,
    0x6F12, 0x1CD0,
    0x6F12, 0x22D0,
    0x6F12, 0x22D0,
    0x6F12, 0x2ED0,
    0x6F12, 0x2ED0,
    0x6F12, 0x0000,
    0x6F12, 0x0400,
    0x6F12, 0x0800,
    0x6F12, 0x0C00,
    0x6F12, 0x0800,
    0x6F12, 0x0C00,
    0x6F12, 0x0000,
    0x6F12, 0x0400,
    0x6F12, 0x30D0,
    0x6F12, 0x32D0,
    0x6F12, 0x64D0,
    0x6F12, 0x66D0,
    0x6F12, 0x7CD0,
    0x6F12, 0x7ED0,
    0x6F12, 0xA8D0,
    0x6F12, 0xAAD0,
    0x6F12, 0x0001,
    0x6F12, 0x0001,
    0x6F12, 0x0001,
    0x6F12, 0x0001,
    0x6F12, 0x0405,
    0x6F12, 0x0405,
    0x6F12, 0x0405,
    0x6F12, 0x0405,
    0x6F12, 0x10D0,
    0x6F12, 0x10D0,
    0x6F12, 0x12D0,
    0x6F12, 0x12D0,
    0x6F12, 0x20D0,
    0x6F12, 0x20D0,
    0x6F12, 0x22D0,
    0x6F12, 0x22D0,
    0x6F12, 0x0000,
    0x6F12, 0x0400,
    0x6F12, 0x0000,
    0x6F12, 0x0400,
    0x6F12, 0x0000,
    0x6F12, 0x0400,
    0x6F12, 0x0000,
    0x6F12, 0x0400,
    0x6F12, 0x30D0,
    0x6F12, 0x32D0,
    0x6F12, 0x38D0,
    0x6F12, 0x3AD0,
    0x6F12, 0x70D0,
    0x6F12, 0x72D0,
    0x6F12, 0x78D0,
    0x6F12, 0x7AD0,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
};
#endif

static kal_uint16 addr_data_pair_init_jn1_Global[] = {
    0x6028, 0x2400,
    0x602A, 0x1354,
    0x6F12, 0x0100,
    0x6F12, 0x7017,
    0x602A, 0x13B2,
    0x6F12, 0x0000,
    0x602A, 0x1236,
    0x6F12, 0x0000,
    0x602A, 0x1A0A,
    0x6F12, 0x4C0A,
    0x602A, 0x2210,
    0x6F12, 0x3401,
    0x602A, 0x2176,
    0x6F12, 0x6400,
    0x602A, 0x222E,
    0x6F12, 0x0001,
    0x602A, 0x06B6,
    0x6F12, 0x0A00,
    0x602A, 0x06BC,
    0x6F12, 0x1001,
    0x602A, 0x2140,
    0x6F12, 0x0101,
    0x602A, 0x1A0E,
    0x6F12, 0x9600,
    0x6028, 0x4000,
    0xF44E, 0x0011,
    0xF44C, 0x0B0B,
    0xF44A, 0x0006,
    0x0118, 0x0002,
    0x011A, 0x0001,
    0xB134, 0x0800,
};


static void sensor_init(void)
{
    pr_debug("%s E\n", __func__);
    /* initial sequence */
    // Convert from : "InitGlobal.sset"
    pr_debug("[%s] +", __func__);


    write_cmos_sensor(0x6028, 0x4000);
    write_cmos_sensor(0x0000, 0x0003);
    write_cmos_sensor(0x0000, 0x38E1);
    write_cmos_sensor(0x001E, 0x0007);
    write_cmos_sensor(0x6028, 0x4000);
    write_cmos_sensor(0x6010, 0x0001);
  mdelay(5);
  write_cmos_sensor(0x6226, 0x0001);
  mdelay(10);

/*  table_write_cmos_sensor(addr_data_pair_init_jn1_SWpage,
    sizeof(addr_data_pair_init_jn1_SWpage) / sizeof(kal_uint16));*/

    table_write_cmos_sensor(addr_data_pair_init_jn1_Global,
    sizeof(addr_data_pair_init_jn1_Global) / sizeof(kal_uint16));

    pr_debug("[%s] -", __func__);
}               /*      sensor_init  */

static kal_uint16 addr_data_pair_pre_jn1[] = {
    0x6028, 0x2400,
    0x602A, 0x1A28,
    0x6F12, 0x4C00,
    0x602A, 0x065A,
    0x6F12, 0x0000,
    0x602A, 0x139E,
    0x6F12, 0x0100,
    0x602A, 0x139C,
    0x6F12, 0x0000,
    0x602A, 0x13A0,
    0x6F12, 0x0A00,
    0x6F12, 0x0120,
    0x602A, 0x2072,
    0x6F12, 0x0000,
    0x602A, 0x1A64,
    0x6F12, 0x0301,
    0x6F12, 0xFF00,
    0x602A, 0x19E6,
    0x6F12, 0x0200,
    0x602A, 0x1A30,
    0x6F12, 0x3401,
    0x602A, 0x19FC,
    0x6F12, 0x0B00,
    0x602A, 0x19F4,
    0x6F12, 0x0606,
    0x602A, 0x19F8,
    0x6F12, 0x1010,
    0x602A, 0x1B26,
    0x6F12, 0x6F80,
    0x6F12, 0xA060,
    0x602A, 0x1A3C,
    0x6F12, 0x6207,
    0x602A, 0x1A48,
    0x6F12, 0x6207,
    0x602A, 0x1444,
    0x6F12, 0x2000,
    0x6F12, 0x2000,
    0x602A, 0x144C,
    0x6F12, 0x3F00,
    0x6F12, 0x3F00,
    0x602A, 0x7F6C,
    0x6F12, 0x0100,
    0x6F12, 0x2F00,
    0x6F12, 0xFA00,
    0x6F12, 0x2400,
    0x6F12, 0xE500,
    0x602A, 0x0650,
    0x6F12, 0x0600,
    0x602A, 0x0654,
    0x6F12, 0x0000,
    0x602A, 0x1A46,
    0x6F12, 0x8A00,
    0x602A, 0x1A52,
    0x6F12, 0xBF00,
    0x602A, 0x0674,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x0668,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x602A, 0x0684,
    0x6F12, 0x4001,
    0x602A, 0x0688,
    0x6F12, 0x4001,
    0x602A, 0x147C,
    0x6F12, 0x1000,
    0x602A, 0x1480,
    0x6F12, 0x1000,
    0x602A, 0x19F6,
    0x6F12, 0x0904,
    0x602A, 0x0812,
    0x6F12, 0x0000,
    0x602A, 0x1A02,
    0x6F12, 0x1800,
    0x602A, 0x2148,
    0x6F12, 0x0100,
    0x602A, 0x2042,
    0x6F12, 0x1A00,
    0x602A, 0x0874,
    0x6F12, 0x0100,
    0x602A, 0x09C0,
    0x6F12, 0x2008,
    0x602A, 0x09C4,
    0x6F12, 0x2000,
    0x602A, 0x19FE,
    0x6F12, 0x0E1C,
    0x602A, 0x4D92,
    0x6F12, 0x0100,
    0x602A, 0x84C8,
    0x6F12, 0x0100,
    0x602A, 0x4D94,
    0x6F12, 0x0005,
    0x6F12, 0x000A,
    0x6F12, 0x0010,
    0x6F12, 0x0810,
    0x6F12, 0x000A,
    0x6F12, 0x0040,
    0x6F12, 0x0810,
    0x6F12, 0x0810,
    0x6F12, 0x8002,
    0x6F12, 0xFD03,
    0x6F12, 0x0010,
    0x6F12, 0x1510,
    0x602A, 0x3570,
    0x6F12, 0x0000,
    0x602A, 0x3574,
    0x6F12, 0x1201,
    0x602A, 0x21E4,
    0x6F12, 0x0400,
    0x602A, 0x21EC,
    0x6F12, 0x1F04,
    0x602A, 0x2080,
    0x6F12, 0x0101,
    0x6F12, 0xFF00,
    0x6F12, 0x7F01,
    0x6F12, 0x0001,
    0x6F12, 0x8001,
    0x6F12, 0xD244,
    0x6F12, 0xD244,
    0x6F12, 0x14F4,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x20BA,
    0x6F12, 0x141C,
    0x6F12, 0x111C,
    0x6F12, 0x54F4,
    0x602A, 0x120E,
    0x6F12, 0x1000,
    0x602A, 0x212E,
    0x6F12, 0x0200,
    0x602A, 0x13AE,
    0x6F12, 0x0101,
    0x602A, 0x0718,
    0x6F12, 0x0001,
    0x602A, 0x0710,
    0x6F12, 0x0002,
    0x6F12, 0x0804,
    0x6F12, 0x0100,
    0x602A, 0x1B5C,
    0x6F12, 0x0000,
    0x602A, 0x0786,
    0x6F12, 0x7701,
    0x602A, 0x2022,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x1360,
    0x6F12, 0x0100,
    0x602A, 0x1376,
    0x6F12, 0x0100,
    0x6F12, 0x6038,
    0x6F12, 0x7038,
    0x6F12, 0x8038,
    0x602A, 0x1386,
    0x6F12, 0x0B00,
    0x602A, 0x06FA,
    0x6F12, 0x1000,
    0x602A, 0x4A94,
    0x6F12, 0x0900,
    0x6F12, 0x0000,
    0x6F12, 0x0300,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0300,
    0x6F12, 0x0000,
    0x6F12, 0x0900,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x0A76,
    0x6F12, 0x1000,
    0x602A, 0x0AEE,
    0x6F12, 0x1000,
    0x602A, 0x0B66,
    0x6F12, 0x1000,
    0x602A, 0x0BDE,
    0x6F12, 0x1000,
    0x602A, 0x0BE8,
    0x6F12, 0x3000,
    0x6F12, 0x3000,
    0x602A, 0x0C56,
    0x6F12, 0x1000,
    0x602A, 0x0C60,
    0x6F12, 0x3000,
    0x6F12, 0x3000,
    0x602A, 0x0CB6,
    0x6F12, 0x0100,
    0x602A, 0x0CF2,
    0x6F12, 0x0001,
    0x602A, 0x0CF0,
    0x6F12, 0x0101,
    0x602A, 0x11B8,
    0x6F12, 0x0100,
    0x602A, 0x11F6,
    0x6F12, 0x0020,
    0x602A, 0x4A74,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x218E,
    0x6F12, 0x0000,
    0x602A, 0x2268,
    0x6F12, 0xF279,
    0x602A, 0x5006,
    0x6F12, 0x0000,
    0x602A, 0x500E,
    0x6F12, 0x0100,
    0x602A, 0x4E70,
    0x6F12, 0x2062,
    0x6F12, 0x5501,
    0x602A, 0x06DC,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6028, 0x4000,
    0xF46A, 0xAE80,
    0x0344, 0x0000,
    0x0346, 0x0000,
    0x0348, 0x1FFF,
    0x034A, 0x181F,
    0x034C, 0x0FF0,
    0x034E, 0x0C00,
    0x0350, 0x0008,
    0x0352, 0x0008,
    0x0900, 0x0122,
    0x0380, 0x0002,
    0x0382, 0x0002,
    0x0384, 0x0002,
    0x0386, 0x0002,
    0x0110, 0x1002,
    0x0114, 0x0301,
    0x0116, 0x3000,
    0x0136, 0x1800,
    0x013E, 0x0000,
    0x0300, 0x0006,
    0x0302, 0x0001,
    0x0304, 0x0004,
    0x0306, 0x008C,
    0x0308, 0x0008,
    0x030A, 0x0001,
    0x030C, 0x0000,
    0x030E, 0x0004,
    0x0310, 0x008A,
    0x0312, 0x0000,
    0x080E, 0x0000,
    0x0340, 0x0FD6,
    0x0342, 0x11E8,
    0x0702, 0x0000,
    0x0202, 0x0100,
    0x0200, 0x0100,
    0x0D00, 0x0101,
    0x0D02, 0x0101,
    0x0D04, 0x0102,
    0x6226, 0x0000,
};

static void preview_setting(void)
{
    LOG_INF("%s E\n", __func__);

/*
 * /  write_cmos_sensor(0x6028, 0x4000);
 *    write_cmos_sensor_8(0x0100, 0x00);
 *    if (currefps == 150){
 *   Convert from : "Init.txt"
 *   No setfile ready yet    } else if (currefps == 240){
 *   Convert from : "Init.txt"
 *
 *
 *  No setfile ready yet    } else {    //30fps
 *  Convert from : "jn1_20M_5184x3880_30fps_MIPI1680mbps.sset"
 *
 *
 *  $MV1[MCLK:24,Width:5184,Height:3880,Format:MIPI_RAW10,mipi_lane:4
 *         ,mipi_datarate:1680,pvi_pclk_inverse:0]
 *   ExtClk :   24  MHz
 *   Vt_pix_clk :   678.4   MHz
 *   MIPI_output_speed :    1680    Mbps/lane
 *   Crop_Width :   5184    px
 *   Crop_Height :  3880    px
 *   Output_Width : 5184    px
 *   Output_Height :    3880    px
 *   Frame rate :   30.01   fps
 *   Output format :    Raw10
 *   H-size :   5640    px
 *   H-blank :  456 px
 *   V-size :   4008    line
 *   V-blank :  128 line
 *   Lane : 4   lane
 *   First Pixel :  Gr  First
 */
    if (bIsLongExposure == KAL_FALSE) {
        pr_debug("[%s] +", __func__);
        table_write_cmos_sensor(addr_data_pair_pre_jn1,
            sizeof(addr_data_pair_pre_jn1) / sizeof(kal_uint16));
        pr_debug("[%s] -", __func__);
    } else {
        sensor_init();
        table_write_cmos_sensor(addr_data_pair_pre_jn1,
            sizeof(addr_data_pair_pre_jn1) / sizeof(kal_uint16));

        LOG_INF("long shutter intterupt don't need re-config setting\n");
        bIsLongExposure = KAL_FALSE;
    }
}

static kal_uint16 addr_data_pair_cap_jn1[] = {
    0x6028, 0x2400,
    0x602A, 0x1A28,
    0x6F12, 0x4C00,
    0x602A, 0x065A,
    0x6F12, 0x0000,
    0x602A, 0x139E,
    0x6F12, 0x0100,
    0x602A, 0x139C,
    0x6F12, 0x0000,
    0x602A, 0x13A0,
    0x6F12, 0x0A00,
    0x6F12, 0x0120,
    0x602A, 0x2072,
    0x6F12, 0x0000,
    0x602A, 0x1A64,
    0x6F12, 0x0301,
    0x6F12, 0xFF00,
    0x602A, 0x19E6,
    0x6F12, 0x0200,
    0x602A, 0x1A30,
    0x6F12, 0x3401,
    0x602A, 0x19FC,
    0x6F12, 0x0B00,
    0x602A, 0x19F4,
    0x6F12, 0x0606,
    0x602A, 0x19F8,
    0x6F12, 0x1010,
    0x602A, 0x1B26,
    0x6F12, 0x6F80,
    0x6F12, 0xA060,
    0x602A, 0x1A3C,
    0x6F12, 0x6207,
    0x602A, 0x1A48,
    0x6F12, 0x6207,
    0x602A, 0x1444,
    0x6F12, 0x2000,
    0x6F12, 0x2000,
    0x602A, 0x144C,
    0x6F12, 0x3F00,
    0x6F12, 0x3F00,
    0x602A, 0x7F6C,
    0x6F12, 0x0100,
    0x6F12, 0x2F00,
    0x6F12, 0xFA00,
    0x6F12, 0x2400,
    0x6F12, 0xE500,
    0x602A, 0x0650,
    0x6F12, 0x0600,
    0x602A, 0x0654,
    0x6F12, 0x0000,
    0x602A, 0x1A46,
    0x6F12, 0x8A00,
    0x602A, 0x1A52,
    0x6F12, 0xBF00,
    0x602A, 0x0674,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x0668,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x602A, 0x0684,
    0x6F12, 0x4001,
    0x602A, 0x0688,
    0x6F12, 0x4001,
    0x602A, 0x147C,
    0x6F12, 0x1000,
    0x602A, 0x1480,
    0x6F12, 0x1000,
    0x602A, 0x19F6,
    0x6F12, 0x0904,
    0x602A, 0x0812,
    0x6F12, 0x0000,
    0x602A, 0x1A02,
    0x6F12, 0x1800,
    0x602A, 0x2148,
    0x6F12, 0x0100,
    0x602A, 0x2042,
    0x6F12, 0x1A00,
    0x602A, 0x0874,
    0x6F12, 0x0100,
    0x602A, 0x09C0,
    0x6F12, 0x2008,
    0x602A, 0x09C4,
    0x6F12, 0x2000,
    0x602A, 0x19FE,
    0x6F12, 0x0E1C,
    0x602A, 0x4D92,
    0x6F12, 0x0100,
    0x602A, 0x84C8,
    0x6F12, 0x0100,
    0x602A, 0x4D94,
    0x6F12, 0x0005,
    0x6F12, 0x000A,
    0x6F12, 0x0010,
    0x6F12, 0x0810,
    0x6F12, 0x000A,
    0x6F12, 0x0040,
    0x6F12, 0x0810,
    0x6F12, 0x0810,
    0x6F12, 0x8002,
    0x6F12, 0xFD03,
    0x6F12, 0x0010,
    0x6F12, 0x1510,
    0x602A, 0x3570,
    0x6F12, 0x0000,
    0x602A, 0x3574,
    0x6F12, 0x1201,
    0x602A, 0x21E4,
    0x6F12, 0x0400,
    0x602A, 0x21EC,
    0x6F12, 0x1F04,
    0x602A, 0x2080,
    0x6F12, 0x0101,
    0x6F12, 0xFF00,
    0x6F12, 0x7F01,
    0x6F12, 0x0001,
    0x6F12, 0x8001,
    0x6F12, 0xD244,
    0x6F12, 0xD244,
    0x6F12, 0x14F4,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x20BA,
    0x6F12, 0x141C,
    0x6F12, 0x111C,
    0x6F12, 0x54F4,
    0x602A, 0x120E,
    0x6F12, 0x1000,
    0x602A, 0x212E,
    0x6F12, 0x0200,
    0x602A, 0x13AE,
    0x6F12, 0x0101,
    0x602A, 0x0718,
    0x6F12, 0x0001,
    0x602A, 0x0710,
    0x6F12, 0x0002,
    0x6F12, 0x0804,
    0x6F12, 0x0100,
    0x602A, 0x1B5C,
    0x6F12, 0x0000,
    0x602A, 0x0786,
    0x6F12, 0x7701,
    0x602A, 0x2022,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x1360,
    0x6F12, 0x0100,
    0x602A, 0x1376,
    0x6F12, 0x0100,
    0x6F12, 0x6038,
    0x6F12, 0x7038,
    0x6F12, 0x8038,
    0x602A, 0x1386,
    0x6F12, 0x0B00,
    0x602A, 0x06FA,
    0x6F12, 0x1000,
    0x602A, 0x4A94,
    0x6F12, 0x0900,
    0x6F12, 0x0000,
    0x6F12, 0x0300,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0300,
    0x6F12, 0x0000,
    0x6F12, 0x0900,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x0A76,
    0x6F12, 0x1000,
    0x602A, 0x0AEE,
    0x6F12, 0x1000,
    0x602A, 0x0B66,
    0x6F12, 0x1000,
    0x602A, 0x0BDE,
    0x6F12, 0x1000,
    0x602A, 0x0BE8,
    0x6F12, 0x3000,
    0x6F12, 0x3000,
    0x602A, 0x0C56,
    0x6F12, 0x1000,
    0x602A, 0x0C60,
    0x6F12, 0x3000,
    0x6F12, 0x3000,
    0x602A, 0x0CB6,
    0x6F12, 0x0100,
    0x602A, 0x0CF2,
    0x6F12, 0x0001,
    0x602A, 0x0CF0,
    0x6F12, 0x0101,
    0x602A, 0x11B8,
    0x6F12, 0x0100,
    0x602A, 0x11F6,
    0x6F12, 0x0020,
    0x602A, 0x4A74,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x218E,
    0x6F12, 0x0000,
    0x602A, 0x2268,
    0x6F12, 0xF279,
    0x602A, 0x5006,
    0x6F12, 0x0000,
    0x602A, 0x500E,
    0x6F12, 0x0100,
    0x602A, 0x4E70,
    0x6F12, 0x2062,
    0x6F12, 0x5501,
    0x602A, 0x06DC,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6028, 0x4000,
    0xF46A, 0xAE80,
    0x0344, 0x0000,
    0x0346, 0x0000,
    0x0348, 0x1FFF,
    0x034A, 0x181F,
    0x034C, 0x0FF0,
    0x034E, 0x0C00,
    0x0350, 0x0008,
    0x0352, 0x0008,
    0x0900, 0x0122,
    0x0380, 0x0002,
    0x0382, 0x0002,
    0x0384, 0x0002,
    0x0386, 0x0002,
    0x0110, 0x1002,
    0x0114, 0x0301,
    0x0116, 0x3000,
    0x0136, 0x1800,
    0x013E, 0x0000,
    0x0300, 0x0006,
    0x0302, 0x0001,
    0x0304, 0x0004,
    0x0306, 0x008C,
    0x0308, 0x0008,
    0x030A, 0x0001,
    0x030C, 0x0000,
    0x030E, 0x0004,
    0x0310, 0x008A,
    0x0312, 0x0000,
    0x080E, 0x0000,
    0x0340, 0x0FD6,
    0x0342, 0x11E8,
    0x0702, 0x0000,
    0x0202, 0x0100,
    0x0200, 0x0100,
    0x0D00, 0x0101,
    0x0D02, 0x0101,
    0x0D04, 0x0102,
    0x6226, 0x0000,
};

static void capture_setting(kal_uint16 currefps)
{
    LOG_INF("%s E! currefps:%d\n", __func__, currefps);

    //sensor_init();
    pr_debug("[%s] +", __func__);
    table_write_cmos_sensor(addr_data_pair_cap_jn1,
            sizeof(addr_data_pair_cap_jn1) / sizeof(kal_uint16));
    pr_debug("[%s] -", __func__);
}

static kal_uint16 addr_data_pair_video_jn1[] = {
    0x6028, 0x2400,
    0x602A, 0x1A28,
    0x6F12, 0x4C00,
    0x602A, 0x065A,
    0x6F12, 0x0000,
    0x602A, 0x139E,
    0x6F12, 0x0100,
    0x602A, 0x139C,
    0x6F12, 0x0000,
    0x602A, 0x13A0,
    0x6F12, 0x0A00,
    0x6F12, 0x0120,
    0x602A, 0x2072,
    0x6F12, 0x0000,
    0x602A, 0x1A64,
    0x6F12, 0x0301,
    0x6F12, 0xFF00,
    0x602A, 0x19E6,
    0x6F12, 0x0200,
    0x602A, 0x1A30,
    0x6F12, 0x3401,
    0x602A, 0x19FC,
    0x6F12, 0x0B00,
    0x602A, 0x19F4,
    0x6F12, 0x0606,
    0x602A, 0x19F8,
    0x6F12, 0x1010,
    0x602A, 0x1B26,
    0x6F12, 0x6F80,
    0x6F12, 0xA060,
    0x602A, 0x1A3C,
    0x6F12, 0x6207,
    0x602A, 0x1A48,
    0x6F12, 0x6207,
    0x602A, 0x1444,
    0x6F12, 0x2000,
    0x6F12, 0x2000,
    0x602A, 0x144C,
    0x6F12, 0x3F00,
    0x6F12, 0x3F00,
    0x602A, 0x7F6C,
    0x6F12, 0x0100,
    0x6F12, 0x2F00,
    0x6F12, 0xFA00,
    0x6F12, 0x2400,
    0x6F12, 0xE500,
    0x602A, 0x0650,
    0x6F12, 0x0600,
    0x602A, 0x0654,
    0x6F12, 0x0000,
    0x602A, 0x1A46,
    0x6F12, 0x8A00,
    0x602A, 0x1A52,
    0x6F12, 0xBF00,
    0x602A, 0x0674,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x0668,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x602A, 0x0684,
    0x6F12, 0x4001,
    0x602A, 0x0688,
    0x6F12, 0x4001,
    0x602A, 0x147C,
    0x6F12, 0x1000,
    0x602A, 0x1480,
    0x6F12, 0x1000,
    0x602A, 0x19F6,
    0x6F12, 0x0904,
    0x602A, 0x0812,
    0x6F12, 0x0000,
    0x602A, 0x1A02,
    0x6F12, 0x1800,
    0x602A, 0x2148,
    0x6F12, 0x0100,
    0x602A, 0x2042,
    0x6F12, 0x1A00,
    0x602A, 0x0874,
    0x6F12, 0x0100,
    0x602A, 0x09C0,
    0x6F12, 0x2008,
    0x602A, 0x09C4,
    0x6F12, 0x9800,
    0x602A, 0x19FE,
    0x6F12, 0x0E1C,
    0x602A, 0x4D92,
    0x6F12, 0x0100,
    0x602A, 0x84C8,
    0x6F12, 0x0100,
    0x602A, 0x4D94,
    0x6F12, 0x0005,
    0x6F12, 0x000A,
    0x6F12, 0x0010,
    0x6F12, 0x0810,
    0x6F12, 0x000A,
    0x6F12, 0x0040,
    0x6F12, 0x0810,
    0x6F12, 0x0810,
    0x6F12, 0x8002,
    0x6F12, 0xFD03,
    0x6F12, 0x0010,
    0x6F12, 0x1510,
    0x602A, 0x3570,
    0x6F12, 0x0000,
    0x602A, 0x3574,
    0x6F12, 0x6100,
    0x602A, 0x21E4,
    0x6F12, 0x0400,
    0x602A, 0x21EC,
    0x6F12, 0xEF03,
    0x602A, 0x2080,
    0x6F12, 0x0101,
    0x6F12, 0xFF00,
    0x6F12, 0x7F01,
    0x6F12, 0x0001,
    0x6F12, 0x8001,
    0x6F12, 0xD244,
    0x6F12, 0xD244,
    0x6F12, 0x14F4,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x20BA,
    0x6F12, 0x121C,
    0x6F12, 0x111C,
    0x6F12, 0x54F4,
    0x602A, 0x120E,
    0x6F12, 0x1000,
    0x602A, 0x212E,
    0x6F12, 0x0200,
    0x602A, 0x13AE,
    0x6F12, 0x0101,
    0x602A, 0x0718,
    0x6F12, 0x0001,
    0x602A, 0x0710,
    0x6F12, 0x0002,
    0x6F12, 0x0804,
    0x6F12, 0x0100,
    0x602A, 0x1B5C,
    0x6F12, 0x0000,
    0x602A, 0x0786,
    0x6F12, 0x7701,
    0x602A, 0x2022,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x1360,
    0x6F12, 0x0100,
    0x602A, 0x1376,
    0x6F12, 0x0100,
    0x6F12, 0x6038,
    0x6F12, 0x7038,
    0x6F12, 0x8038,
    0x602A, 0x1386,
    0x6F12, 0x0B00,
    0x602A, 0x06FA,
    0x6F12, 0x1000,
    0x602A, 0x4A94,
    0x6F12, 0x0900,
    0x6F12, 0x0000,
    0x6F12, 0x0300,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0300,
    0x6F12, 0x0000,
    0x6F12, 0x0900,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x0A76,
    0x6F12, 0x1000,
    0x602A, 0x0AEE,
    0x6F12, 0x1000,
    0x602A, 0x0B66,
    0x6F12, 0x1000,
    0x602A, 0x0BDE,
    0x6F12, 0x1000,
    0x602A, 0x0BE8,
    0x6F12, 0x3000,
    0x6F12, 0x3000,
    0x602A, 0x0C56,
    0x6F12, 0x1000,
    0x602A, 0x0C60,
    0x6F12, 0x3000,
    0x6F12, 0x3000,
    0x602A, 0x0CB6,
    0x6F12, 0x0100,
    0x602A, 0x0CF2,
    0x6F12, 0x0001,
    0x602A, 0x0CF0,
    0x6F12, 0x0101,
    0x602A, 0x11B8,
    0x6F12, 0x0100,
    0x602A, 0x11F6,
    0x6F12, 0x0020,
    0x602A, 0x4A74,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x218E,
    0x6F12, 0x0000,
    0x602A, 0x2268,
    0x6F12, 0xF279,
    0x602A, 0x5006,
    0x6F12, 0x0000,
    0x602A, 0x500E,
    0x6F12, 0x0100,
    0x602A, 0x4E70,
    0x6F12, 0x2062,
    0x6F12, 0x5501,
    0x602A, 0x06DC,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6028, 0x4000,
    0xF46A, 0xAE80,
    0x0344, 0x00F0,
    0x0346, 0x0390,
    0x0348, 0x1F0F,
    0x034A, 0x148F,
    0x034C, 0x0F00,
    0x034E, 0x0870,
    0x0350, 0x0008,
    0x0352, 0x0008,
    0x0900, 0x0122,
    0x0380, 0x0002,
    0x0382, 0x0002,
    0x0384, 0x0002,
    0x0386, 0x0002,
    0x0110, 0x1002,
    0x0114, 0x0301,
    0x0116, 0x3000,
    0x0136, 0x1800,
    0x013E, 0x0000,
    0x0300, 0x0006,
    0x0302, 0x0001,
    0x0304, 0x0004,
    0x0306, 0x008C,
    0x0308, 0x0008,
    0x030A, 0x0001,
    0x030C, 0x0000,
    0x030E, 0x0004,
    0x0310, 0x006B,
    0x0312, 0x0000,
    0x080E, 0x0000,
    0x0340, 0x0CF0,
    0x0342, 0x1600,
    0x0702, 0x0000,
    0x0202, 0x0100,
    0x0200, 0x0100,
    0x0D00, 0x0101,
    0x0D02, 0x0101,
    0x0D04, 0x0102,
    0x6226, 0x0000,
};

static void normal_video_setting(kal_uint16 currefps)
{
    LOG_INF("%s E! currefps:%d\n", __func__, currefps);

    pr_debug("[%s] +", __func__);
    /*Convert from : "jn1_20M_5184x3880_30fps_MIPI1680mbps.sset"*/

    /*Format:MIPI_RAW10,mipi_lane:4,mipi_datarate:1680,pvi_pclk_inverse:0]
     */
    /*
     * ExtClk : 24  MHz
     * Vt_pix_clk : 678.4   MHz
     * MIPI_output_speed :  1680    Mbps/lane
     * Crop_Width : 5184    px
     * Crop_Height :    3880    px
     * Output_Width :   5184    px
     * Output_Height :  3880    px
     * Frame rate : 30.01   fps
     * Output format :  Raw10
     * H-size : 5640    px
     * H-blank :    456 px
     * V-size : 4008    line
     * V-blank :    128 line
     * Lane :   4   lane
     * First Pixel :    Gr  First
     */

    table_write_cmos_sensor(addr_data_pair_video_jn1,
        sizeof(addr_data_pair_video_jn1) / sizeof(kal_uint16));

    //  table_write_cmos_sensor(addr_data_pair_pre_jn1,
    //      sizeof(addr_data_pair_pre_jn1) / sizeof(kal_uint16));


    pr_debug("[%s] -", __func__);
}

static kal_uint16 addr_data_pair_hs_jn1[] = {
    0x6028, 0x2400,
    0x602A, 0x1A28,
    0x6F12, 0x4C00,
    0x602A, 0x065A,
    0x6F12, 0x0000,
    0x602A, 0x139E,
    0x6F12, 0x0300,
    0x602A, 0x139C,
    0x6F12, 0x0000,
    0x602A, 0x13A0,
    0x6F12, 0x0A00,
    0x6F12, 0x0020,
    0x602A, 0x2072,
    0x6F12, 0x0000,
    0x602A, 0x1A64,
    0x6F12, 0x0301,
    0x6F12, 0x3F00,
    0x602A, 0x19E6,
    0x6F12, 0x0201,
    0x602A, 0x1A30,
    0x6F12, 0x3401,
    0x602A, 0x19FC,
    0x6F12, 0x0B00,
    0x602A, 0x19F4,
    0x6F12, 0x0606,
    0x602A, 0x19F8,
    0x6F12, 0x1010,
    0x602A, 0x1B26,
    0x6F12, 0x6F80,
    0x6F12, 0xA020,
    0x602A, 0x1A3C,
    0x6F12, 0x5207,
    0x602A, 0x1A48,
    0x6F12, 0x5207,
    0x602A, 0x1444,
    0x6F12, 0x2100,
    0x6F12, 0x2100,
    0x602A, 0x144C,
    0x6F12, 0x4200,
    0x6F12, 0x4200,
    0x602A, 0x7F6C,
    0x6F12, 0x0100,
    0x6F12, 0x3100,
    0x6F12, 0xF700,
    0x6F12, 0x2600,
    0x6F12, 0xE100,
    0x602A, 0x0650,
    0x6F12, 0x0600,
    0x602A, 0x0654,
    0x6F12, 0x0000,
    0x602A, 0x1A46,
    0x6F12, 0x8600,
    0x602A, 0x1A52,
    0x6F12, 0xBF00,
    0x602A, 0x0674,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x0668,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x602A, 0x0684,
    0x6F12, 0x4001,
    0x602A, 0x0688,
    0x6F12, 0x4001,
    0x602A, 0x147C,
    0x6F12, 0x1000,
    0x602A, 0x1480,
    0x6F12, 0x1000,
    0x602A, 0x19F6,
    0x6F12, 0x0904,
    0x602A, 0x0812,
    0x6F12, 0x0000,
    0x602A, 0x1A02,
    0x6F12, 0x0800,
    0x602A, 0x2148,
    0x6F12, 0x0100,
    0x602A, 0x2042,
    0x6F12, 0x1A00,
    0x602A, 0x0874,
    0x6F12, 0x1100,
    0x602A, 0x09C0,
    0x6F12, 0x9800,
    0x602A, 0x09C4,
    0x6F12, 0x9800,
    0x602A, 0x19FE,
    0x6F12, 0x0E1C,
    0x602A, 0x4D92,
    0x6F12, 0x0100,
    0x602A, 0x84C8,
    0x6F12, 0x0100,
    0x602A, 0x4D94,
    0x6F12, 0x4001,
    0x6F12, 0x0004,
    0x6F12, 0x0010,
    0x6F12, 0x0810,
    0x6F12, 0x0004,
    0x6F12, 0x0010,
    0x6F12, 0x0810,
    0x6F12, 0x0810,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0010,
    0x6F12, 0x0010,
    0x602A, 0x3570,
    0x6F12, 0x0000,
    0x602A, 0x3574,
    0x6F12, 0x9400,
    0x602A, 0x21E4,
    0x6F12, 0x0400,
    0x602A, 0x21EC,
    0x6F12, 0x4F01,
    0x602A, 0x2080,
    0x6F12, 0x0100,
    0x6F12, 0x7F00,
    0x6F12, 0x0002,
    0x6F12, 0x8000,
    0x6F12, 0x0002,
    0x6F12, 0xC244,
    0x6F12, 0xD244,
    0x6F12, 0x14F4,
    0x6F12, 0x141C,
    0x6F12, 0x111C,
    0x6F12, 0x54F4,
    0x602A, 0x20BA,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x120E,
    0x6F12, 0x1000,
    0x602A, 0x212E,
    0x6F12, 0x0A00,
    0x602A, 0x13AE,
    0x6F12, 0x0102,
    0x602A, 0x0718,
    0x6F12, 0x0005,
    0x602A, 0x0710,
    0x6F12, 0x0004,
    0x6F12, 0x0401,
    0x6F12, 0x0100,
    0x602A, 0x1B5C,
    0x6F12, 0x0300,
    0x602A, 0x0786,
    0x6F12, 0x7701,
    0x602A, 0x2022,
    0x6F12, 0x0101,
    0x6F12, 0x0101,
    0x602A, 0x1360,
    0x6F12, 0x0000,
    0x602A, 0x1376,
    0x6F12, 0x0200,
    0x6F12, 0x6038,
    0x6F12, 0x7038,
    0x6F12, 0x8038,
    0x602A, 0x1386,
    0x6F12, 0x0B00,
    0x602A, 0x06FA,
    0x6F12, 0x1000,
    0x602A, 0x4A94,
    0x6F12, 0x0C00,
    0x6F12, 0x0000,
    0x6F12, 0x0600,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0600,
    0x6F12, 0x0000,
    0x6F12, 0x0C00,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x0A76,
    0x6F12, 0x1000,
    0x602A, 0x0AEE,
    0x6F12, 0x1000,
    0x602A, 0x0B66,
    0x6F12, 0x1000,
    0x602A, 0x0BDE,
    0x6F12, 0x1000,
    0x602A, 0x0BE8,
    0x6F12, 0x3000,
    0x6F12, 0x3000,
    0x602A, 0x0C56,
    0x6F12, 0x1000,
    0x602A, 0x0C60,
    0x6F12, 0x3000,
    0x6F12, 0x3000,
    0x602A, 0x0CB6,
    0x6F12, 0x0000,
    0x602A, 0x0CF2,
    0x6F12, 0x0001,
    0x602A, 0x0CF0,
    0x6F12, 0x0101,
    0x602A, 0x11B8,
    0x6F12, 0x0000,
    0x602A, 0x11F6,
    0x6F12, 0x0010,
    0x602A, 0x4A74,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x218E,
    0x6F12, 0x0000,
    0x602A, 0x2268,
    0x6F12, 0xF279,
    0x602A, 0x5006,
    0x6F12, 0x0000,
    0x602A, 0x500E,
    0x6F12, 0x0100,
    0x602A, 0x4E70,
    0x6F12, 0x2062,
    0x6F12, 0x5501,
    0x602A, 0x06DC,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6028, 0x4000,
    0xF46A, 0xAE80,
    0x0344, 0x00F0,
    0x0346, 0x0390,
    0x0348, 0x1F0F,
    0x034A, 0x148F,
    0x034C, 0x0780,
    0x034E, 0x0438,
    0x0350, 0x0004,
    0x0352, 0x0004,
    0x0900, 0x0144,
    0x0380, 0x0002,
    0x0382, 0x0006,
    0x0384, 0x0002,
    0x0386, 0x0006,
    0x0110, 0x1002,
    0x0114, 0x0301,
    0x0116, 0x3000,
    0x0136, 0x1800,
    0x013E, 0x0000,
    0x0300, 0x0006,
    0x0302, 0x0001,
    0x0304, 0x0004,
    0x0306, 0x0096,
    0x0308, 0x0008,
    0x030A, 0x0001,
    0x030C, 0x0000,
    0x030E, 0x0004,
    0x0310, 0x00A5,
    0x0312, 0x0000,
    0x080E, 0x0000,
    0x0340, 0x04B8,
    0x0342, 0x0810,
    0x0702, 0x0000,
    0x0202, 0x0100,
    0x0200, 0x0100,
    0x0D00, 0x0101,
    0x0D02, 0x0101,
    0x0D04, 0x0102,
    0x6226, 0x0000,
};
static void hs_video_setting(void)
{
    LOG_INF("%s E\n", __func__);
    table_write_cmos_sensor(addr_data_pair_hs_jn1,
            sizeof(addr_data_pair_hs_jn1) / sizeof(kal_uint16));
}
static kal_uint16 addr_data_pair_slim_jn1[] = {
    0x6028, 0x4000,
    0x6214, 0xF9F0,
    0x6218, 0xF150,
    0x6028, 0x2000,
    0x602A, 0x36AA,
    0x6F12, 0x0060,
    0x6F12, 0x0061,
    0x6F12, 0x0070,
    0x6F12, 0x0071,
    0x602A, 0x36CA,
    0x6F12, 0x1FFF,
    0x6F12, 0x1FFF,
    0x6A12, 0x0000,
    0x602A, 0x2090,
    0x6F12, 0xFFFF,
    0x602A, 0x3670,
    0x6F12, 0x0505,
    0x6F12, 0x0505,
    0x602A, 0x368C,
    0x6F12, 0x387F,
    0x6F12, 0xFFFC,
    0x602A, 0x37B0,
    0x6F12, 0x000A,
    0x6F12, 0x0018,
    0x6F12, 0x0014,
    0x6F12, 0x0013,
    0x602A, 0x37BA,
    0x6F12, 0x0011,
    0x602A, 0x3680,
    0x6F12, 0x0000,
    0x6F12, 0x0280,
    0x6F12, 0x5008,
    0x602A, 0x235C,
    0x6F12, 0x0009,
    0x602A, 0x37A8,
    0x6F12, 0x0000,
    0x602A, 0x36D6,
    0x6F12, 0x44DE,
    0x6F12, 0x80AE,
    0x6F12, 0xC000,
    0xF446, 0x023F,
    0xF448, 0x0000,
    0x602A, 0x37A0,
    0x6F12, 0x0000,
    0x602A, 0x2594,
    0x6F12, 0x002B,
    0x602A, 0x259C,
    0x6F12, 0x05EA,
    0x602A, 0x25A4,
    0x6F12, 0x05FF,
    0x602A, 0x25AC,
    0x6F12, 0x05FF,
    0x602A, 0x25B4,
    0x6F12, 0x002B,
    0x602A, 0x25BC,
    0x6F12, 0x05EA,
    0x602A, 0x25C4,
    0x6F12, 0x05FF,
    0x602A, 0x25CC,
    0x6F12, 0x05FF,
    0x602A, 0x3690,
    0x6F12, 0xFFEA,
    0x602A, 0x285C,
    0x6F12, 0x0101,
    0x602A, 0x286C,
    0x6F12, 0x0234,
    0x602A, 0x2474,
    0x6F12, 0x0110,
    0x602A, 0x247C,
    0x6F12, 0x00F8,
    0x602A, 0x3694,
    0x6F12, 0x000F,
    0x602A, 0x2C62,
    0x6F12, 0x8001,
    0x6F12, 0x8001,
    0x6F12, 0x8001,
    0x6F12, 0x8001,
    0x6F12, 0x8030,
    0x6F12, 0x8030,
    0x6F12, 0x8030,
    0x6F12, 0x8030,
    0x602A, 0x236C,
    0x6F12, 0x0009,
    0x602A, 0x249C,
    0x6F12, 0x000A,
    0x602A, 0x3686,
    0x6F12, 0x0808,
    0x602A, 0x37A2,
    0x6F12, 0x0025,
    0x602A, 0x1388,
    0x6F12, 0x0080,
    0x602A, 0x2124,
    0x6F12, 0x0100,
    0x6F12, 0x1E00,
    0x602A, 0x3EA4,
    0x6F12, 0x0003,
    0x602A, 0x3E00,
    0x6F12, 0x0201,
    0x602A, 0x2336,
    0x6F12, 0xFFFE,
    0x602A, 0x3678,
    0x6F12, 0x0707,
    0x602A, 0x2344,
    0x6F12, 0x0023,
    0x602A, 0x2354,
    0x6F12, 0x008C,
    0x602A, 0x2384,
    0x6F12, 0x0090,
    0x602A, 0x2464,
    0x6F12, 0x0012,
    0x602A, 0x248C,
    0x6F12, 0x0011,
    0x6028, 0x2001,
    0x602A, 0x4F4C,
    0x6F12, 0x0000,
    0x602A, 0xFE44,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x00A3,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6028, 0x2000,
    0x0110, 0x1002,
    0x011C, 0x0101,
    0x0344, 0x0000,
    0x0346, 0x0000,
    0x0348, 0x243F,
    0x034A, 0x1B1F,
    0x034C, 0x0910,
    0x034E, 0x06C8,
    0x0350, 0x0000,
    0x0352, 0x0000,
    0x0342, 0x3830,
    0x0340, 0x08B0,
    0x602A, 0x3652,
    0x6F12, 0x0401,
    0x0380, 0x0002,
    0x0382, 0x0006,
    0x0384, 0x0002,
    0x0386, 0x0006,
    0x0900, 0x3344,
    0x040C, 0x0000,
    0x0404, 0x1000,
    0x0408, 0x0100,
    0x040A, 0x0100,
    0x0400, 0x1010,
    0x0136, 0x1800,
    0x0304, 0x0004,
    0x030C, 0x0000,
    0x0306, 0x00A0,
    0x0302, 0x0001,
    0x0300, 0x0004,
    0x030E, 0x0004,
    0x0312, 0x0002,
    0x0310, 0x00C8,
    0x602A, 0x368A,
    0x6F12, 0x0096,
    0x021E, 0x0000,
    0x0D00, 0x0000,
    0x0D02, 0x0000,
    0x0114, 0x0200,
    0x602A, 0x593C,
    0x6F12, 0x0104,
    0x6F12, 0x0802,
    0x6F12, 0x0100,
    0x6F12, 0x0141,
    0x6F12, 0x0182,
    0x6F12, 0x01C3,
    0x6F12, 0x0300,
    0x6F12, 0x0141,
    0x6F12, 0x0128,
    0x6F12, 0x03C3,
    0x6F12, 0x0302,
    0x602A, 0x5960,
    0x6F12, 0x0500,
    0x602A, 0x5964,
    0x6F12, 0x3F01,
    0x602A, 0x5968,
    0x6F12, 0x3F01,
    0x602A, 0x596C,
    0x6F12, 0x3F05,
    0x602A, 0x5970,
    0x6F12, 0x3F00,
    0x602A, 0x5974,
    0x6F12, 0x3F04,
    0x602A, 0x5978,
    0x6F12, 0x3F01,
    0x602A, 0x597C,
    0x6F12, 0x3F05,
    0x602A, 0x59A0,
    0x6F12, 0x3F00,
    0x602A, 0x59A4,
    0x6F12, 0x3F01,
    0x602A, 0x59A8,
    0x6F12, 0x3F01,
    0x602A, 0x59AC,
    0x6F12, 0x3F05,
    0x602A, 0x59B0,
    0x6F12, 0x3F00,
    0x602A, 0x59B4,
    0x6F12, 0x3F04,
    0x602A, 0x59B8,
    0x6F12, 0x3F01,
    0x602A, 0x59BC,
    0x6F12, 0x3F05,
    0x602A, 0x59E0,
    0x6F12, 0x3F00,
    0x602A, 0x59E4,
    0x6F12, 0x3F01,
    0x602A, 0x59E8,
    0x6F12, 0x3F3F,
    0x602A, 0x59EC,
    0x6F12, 0x3F3F,
    0x602A, 0x59F0,
    0x6F12, 0x3F3F,
    0x602A, 0x59F4,
    0x6F12, 0x3F3F,
    0x602A, 0x59F8,
    0x6F12, 0x3F3F,
    0x602A, 0x59FC,
    0x6F12, 0x3F3F,
    0x602A, 0x5A20,
    0x6F12, 0x3F00,
    0x602A, 0x5A24,
    0x6F12, 0x3F01,
    0x602A, 0x5A28,
    0x6F12, 0x3F3F,
    0x602A, 0x5A2C,
    0x6F12, 0x3F3F,
    0x602A, 0x5A30,
    0x6F12, 0x3F3F,
    0x602A, 0x5A34,
    0x6F12, 0x3F3F,
    0x602A, 0x5A38,
    0x6F12, 0x3F3F,
    0x602A, 0x5A3C,
    0x6F12, 0x3F3F,
    0x602A, 0x58EC,
    0x6F12, 0x000F,
    0x602A, 0x5930,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6028, 0x2001,
    0x602A, 0xF800,
    0x6F12, 0x02D8,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x00E4,
    0x6F12, 0x004E,
    0x6F12, 0x0169,
    0x602A, 0x8680,
    0x6F12, 0x0101,
    0x6F12, 0x0000,
    0x6028, 0x2000,
    0x602A, 0x207A,
    0x6F12, 0x0A00,
    0x0D06, 0x0248,
    0x0D08, 0x0D8C,
    0x0B06, 0x0101,
    0x0B08, 0x0000,
    0x0FEA, 0x2440,
    0x602A, 0x21BC,
    0x6F12, 0x0000,
    0x6F12, 0x0002,
    0x0202, 0x0010,
    0x0204, 0x0020,
    0x602A, 0x37BC,
    0x6F12, 0x0100,
    0x6028, 0x2001,
    0x602A, 0xF88E,
    0x6F12, 0x0100,
    0x602A, 0xF8A0,
    0x6F12, 0x0050,
    0x6F12, 0x0060,
    0x6F12, 0x0070,
    0x602A, 0xF8A6,
    0x6F12, 0x0000,
    0x602A, 0xF8D8,
    0x6F12, 0x0000,
    0x602A, 0xF8DA,
    0x6F12, 0x0000,
    0x6028, 0x2000,
    0x602A, 0x40F8,
    0x6F12, 0x0000,
    0x6028, 0x2001,
    0x602A, 0xF8DC,
    0x6F12, 0x04F0,
    0x6F12, 0x00F0,
    0x6F12, 0xAAE3,
    0x6F12, 0x200A,
    0x6F12, 0x0020,
    0x6F12, 0x551E,
    0x6F12, 0x200A,
    0x6F12, 0x0020,
    0x6F12, 0xAA39,
    0x6F12, 0x200A,
    0x6F12, 0x0020,
    0x6F12, 0x55F2,
    0x6F12, 0x200A,
    0x6F12, 0x0022,
    0x6F12, 0x00A8,
    0x6F12, 0x2000,
    0x6F12, 0x0000,
    0x6028, 0x2000,
    0x602A, 0x642C,
    0x6F12, 0x0078,
    0x6F12, 0x0052,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x6444,
    0x6F12, 0x0001,
    0x602A, 0x815E,
    0x6F12, 0x0047,
    0x6F12, 0x0004,
    0x6F12, 0x0019,
    0x6F12, 0x0019,
    0x6F12, 0x0019,
    0x6F12, 0x0019,
    0x6F12, 0x0019,
    0x6F12, 0x0019,
    0x6F12, 0x0048,
    0x6F12, 0x0004,
    0x6F12, 0x0019,
    0x6F12, 0x0019,
    0x6F12, 0x0019,
    0x6F12, 0x0019,
    0x6F12, 0x0019,
    0x6F12, 0x0019,
    0x6F12, 0x0049,
    0x6F12, 0x0004,
    0x6F12, 0x0019,
    0x6F12, 0x0019,
    0x6F12, 0x0019,
    0x6F12, 0x0019,
    0x6F12, 0x0019,
    0x6F12, 0x0019,
    0x602A, 0x81C2,
    0x6F12, 0x003C,
    0x6F12, 0x003C,
    0x6F12, 0x003C,
    0x6F12, 0x003C,
    0x6F12, 0x003C,
    0x6F12, 0x003C,
    0x602A, 0x85EE,
    0x6F12, 0x00A4,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x00A5,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x00A6,
    0x6F12, 0x0004,
    0x6F12, 0x0005,
    0x6F12, 0x0005,
    0x6F12, 0x0005,
    0x6F12, 0x0005,
    0x6F12, 0x0005,
    0x6F12, 0x0005,
    0x6F12, 0x00A7,
    0x6F12, 0x0004,
    0x6F12, 0x0006,
    0x6F12, 0x0006,
    0x6F12, 0x0006,
    0x6F12, 0x0006,
    0x6F12, 0x0006,
    0x6F12, 0x0006,
    0x6F12, 0x00A8,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x00A9,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x00AA,
    0x6F12, 0x0004,
    0x6F12, 0x0005,
    0x6F12, 0x0005,
    0x6F12, 0x0005,
    0x6F12, 0x0005,
    0x6F12, 0x0005,
    0x6F12, 0x0005,
    0x6F12, 0x00AB,
    0x6F12, 0x0004,
    0x6F12, 0x0006,
    0x6F12, 0x0006,
    0x6F12, 0x0006,
    0x6F12, 0x0006,
    0x6F12, 0x0006,
    0x6F12, 0x0006,
    0x6F12, 0x00AC,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x00AD,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x0004,
    0x6F12, 0x00AE,
    0x6F12, 0x0004,
    0x6F12, 0x0005,
    0x6F12, 0x0005,
    0x6F12, 0x0005,
    0x6F12, 0x0005,
    0x6F12, 0x0005,
    0x6F12, 0x0005,
    0x6F12, 0x00AF,
    0x6F12, 0x0004,
    0x6F12, 0x0006,
    0x6F12, 0x0006,
    0x6F12, 0x0006,
    0x6F12, 0x0006,
    0x6F12, 0x0006,
    0x6F12, 0x0006,
    0x602A, 0x5270,
    0x6F12, 0x0000,
    0x602A, 0x5C80,
    0x6F12, 0x0000,
    0x602A, 0x3D32,
    0x6F12, 0x0001,
    0x6F12, 0x0103,
    0x602A, 0x134C,
    0x6F12, 0x2000,
    0x602A, 0x2336,
    0x6F12, 0xFFFE,
    0x602A, 0x21C0,
    0x6F12, 0x0000,
    0x0FE8, 0x3600,
    0x0B00, 0x0180,
    0x602A, 0x4FB0,
    0x6F12, 0x0301,
    0x602A, 0x4FD2,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x4FCE,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x081E, 0x0A00,
    0x6B5C, 0xE222,
    0x6B5E, 0x00FF,
    0x6B76, 0xE000,
    0x081C, 0x000C,
    0x6028, 0x4000,
    0x6214, 0xF9F0,
    0x6218, 0xF9F0,
};
static void slim_video_setting(void)
{
    LOG_INF("%s E\n", __func__);
    table_write_cmos_sensor(addr_data_pair_slim_jn1,
        sizeof(addr_data_pair_slim_jn1) / sizeof(kal_uint16));
}
static kal_uint16 addr_data_pair_custom1_jn1[] = {
    0x6028, 0x2400,
    0x602A, 0x1A28,
    0x6F12, 0x4C00,
    0x602A, 0x065A,
    0x6F12, 0x0000,
    0x602A, 0x139E,
    0x6F12, 0x0100,
    0x602A, 0x139C,
    0x6F12, 0x0000,
    0x602A, 0x13A0,
    0x6F12, 0x0A00,
    0x6F12, 0x0120,
    0x602A, 0x2072,
    0x6F12, 0x0000,
    0x602A, 0x1A64,
    0x6F12, 0x0301,
    0x6F12, 0xFF00,
    0x602A, 0x19E6,
    0x6F12, 0x0200,
    0x602A, 0x1A30,
    0x6F12, 0x3401,
    0x602A, 0x19FC,
    0x6F12, 0x0B00,
    0x602A, 0x19F4,
    0x6F12, 0x0606,
    0x602A, 0x19F8,
    0x6F12, 0x1010,
    0x602A, 0x1B26,
    0x6F12, 0x6F80,
    0x6F12, 0xA060,
    0x602A, 0x1A3C,
    0x6F12, 0x6207,
    0x602A, 0x1A48,
    0x6F12, 0x6207,
    0x602A, 0x1444,
    0x6F12, 0x2000,
    0x6F12, 0x2000,
    0x602A, 0x144C,
    0x6F12, 0x3F00,
    0x6F12, 0x3F00,
    0x602A, 0x7F6C,
    0x6F12, 0x0100,
    0x6F12, 0x2F00,
    0x6F12, 0xFA00,
    0x6F12, 0x2400,
    0x6F12, 0xE500,
    0x602A, 0x0650,
    0x6F12, 0x0600,
    0x602A, 0x0654,
    0x6F12, 0x0000,
    0x602A, 0x1A46,
    0x6F12, 0xB000,
    0x602A, 0x1A52,
    0x6F12, 0xBF00,
    0x602A, 0x0674,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x0668,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x602A, 0x0684,
    0x6F12, 0x4001,
    0x602A, 0x0688,
    0x6F12, 0x4001,
    0x602A, 0x147C,
    0x6F12, 0x1000,
    0x602A, 0x1480,
    0x6F12, 0x1000,
    0x602A, 0x19F6,
    0x6F12, 0x0904,
    0x602A, 0x0812,
    0x6F12, 0x0010,
    0x602A, 0x2148,
    0x6F12, 0x0100,
    0x602A, 0x2042,
    0x6F12, 0x1A00,
    0x602A, 0x0874,
    0x6F12, 0x0100,
    0x602A, 0x09C0,
    0x6F12, 0x2008,
    0x602A, 0x09C4,
    0x6F12, 0x2000,
    0x602A, 0x19FE,
    0x6F12, 0x0E1C,
    0x602A, 0x4D92,
    0x6F12, 0x0100,
    0x602A, 0x8104,
    0x6F12, 0x0100,
    0x602A, 0x4D94,
    0x6F12, 0x0005,
    0x6F12, 0x000A,
    0x6F12, 0x0010,
    0x6F12, 0x1510,
    0x6F12, 0x000A,
    0x6F12, 0x0040,
    0x6F12, 0x1510,
    0x6F12, 0x1510,
    0x602A, 0x3570,
    0x6F12, 0x0000,
    0x602A, 0x3574,
    0x6F12, 0x1304,
    0x602A, 0x21E4,
    0x6F12, 0x0400,
    0x602A, 0x21EC,
    0x6F12, 0x1D02,
    0x602A, 0x2080,
    0x6F12, 0x0100,
    0x6F12, 0xFF00,
    0x602A, 0x2086,
    0x6F12, 0x0001,
    0x602A, 0x208E,
    0x6F12, 0x14F4,
    0x602A, 0x208A,
    0x6F12, 0xD244,
    0x6F12, 0xD244,
    0x602A, 0x120E,
    0x6F12, 0x1000,
    0x602A, 0x212E,
    0x6F12, 0x0200,
    0x602A, 0x13AE,
    0x6F12, 0x0101,
    0x602A, 0x0718,
    0x6F12, 0x0001,
    0x602A, 0x0710,
    0x6F12, 0x0002,
    0x6F12, 0x0804,
    0x6F12, 0x0100,
    0x602A, 0x1B5C,
    0x6F12, 0x0000,
    0x602A, 0x0786,
    0x6F12, 0x7701,
    0x602A, 0x2022,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x1360,
    0x6F12, 0x0100,
    0x602A, 0x1376,
    0x6F12, 0x0100,
    0x6F12, 0x6038,
    0x6F12, 0x7038,
    0x6F12, 0x8038,
    0x602A, 0x1386,
    0x6F12, 0x0B00,
    0x602A, 0x06FA,
    0x6F12, 0x1000,
    0x602A, 0x4A94,
    0x6F12, 0x0600,
    0x6F12, 0x0600,
    0x6F12, 0x0600,
    0x6F12, 0x0600,
    0x6F12, 0x0600,
    0x6F12, 0x0600,
    0x6F12, 0x0600,
    0x6F12, 0x0600,
    0x6F12, 0x0600,
    0x6F12, 0x0600,
    0x6F12, 0x0600,
    0x6F12, 0x0600,
    0x6F12, 0x0600,
    0x6F12, 0x0600,
    0x6F12, 0x0600,
    0x6F12, 0x0600,
    0x602A, 0x0A76,
    0x6F12, 0x1000,
    0x602A, 0x0AEE,
    0x6F12, 0x1000,
    0x602A, 0x0B66,
    0x6F12, 0x1000,
    0x602A, 0x0BDE,
    0x6F12, 0x1000,
    0x602A, 0x0C56,
    0x6F12, 0x1000,
    0x602A, 0x0CF2,
    0x6F12, 0x0001,
    0x602A, 0x0CF0,
    0x6F12, 0x0101,
    0x602A, 0x11B8,
    0x6F12, 0x0100,
    0x602A, 0x11F6,
    0x6F12, 0x0020,
    0x602A, 0x4A74,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6028, 0x4000,
    0xF46A, 0xAE80,
    0x0344, 0x0000,
    0x0346, 0x0000,
    0x0348, 0x1FFF,
    0x034A, 0x181F,
    0x034C, 0x0FF0,
    0x034E, 0x0C00,
    0x0350, 0x0008,
    0x0352, 0x0008,
    0x0900, 0x0122,
    0x0380, 0x0002,
    0x0382, 0x0002,
    0x0384, 0x0002,
    0x0386, 0x0002,
    0x0110, 0x1002,
    0x0114, 0x0301,
    0x0116, 0x3000,
    0x0136, 0x1800,
    0x013E, 0x0000,
    0x0300, 0x0006,
    0x0302, 0x0001,
    0x0304, 0x0004,
    0x0306, 0x008C,
    0x0308, 0x0008,
    0x030A, 0x0001,
    0x030C, 0x0000,
    0x030E, 0x0004,
    0x0310, 0x0064,
    0x0312, 0x0000,
    0x0340, 0x0F62,
    0x0342, 0x1716,
    0x0702, 0x0000,
    0x0202, 0x0100,
    0x0200, 0x0100,
    0x0D00, 0x0101,
    0x0D02, 0x0101,
    0x0D04, 0x0102,
    0x6226, 0x0000,
};

static void custom1_setting(void)
{
    printk("%s E\n", __func__);

    //sensor_init();
    /*240FPS */

    /*// Convert from : "Init.txt"*/
    /*check_streamoff();*/
    table_write_cmos_sensor(addr_data_pair_custom1_jn1,
        sizeof(addr_data_pair_custom1_jn1) / sizeof(kal_uint16));
}

static kal_uint16 addr_data_pair_custom2_jn1[] = {
    0x6028,0x2400,
    0x602A,0x1A28,
    0x6F12,0x4C00,
    0x602A,0x065A,
    0x6F12,0x0000,
    0x602A,0x139E,
    0x6F12,0x0300,
    0x602A,0x139C,
    0x6F12,0x0000,
    0x602A,0x13A0,
    0x6F12,0x0A00,
    0x6F12,0x0020,
    0x602A,0x2072,
    0x6F12,0x0000,
    0x602A,0x1A64,
    0x6F12,0x0301,
    0x6F12,0x3F00,
    0x602A,0x19E6,
    0x6F12,0x0201,
    0x602A,0x1A30,
    0x6F12,0x3401,
    0x602A,0x19FC,
    0x6F12,0x0B00,
    0x602A,0x19F4,
    0x6F12,0x0606,
    0x602A,0x19F8,
    0x6F12,0x1010,
    0x602A,0x1B26,
    0x6F12,0x6F80,
    0x6F12,0xA020,
    0x602A,0x1A3C,
    0x6F12,0x5207,
    0x602A,0x1A48,
    0x6F12,0x5207,
    0x602A,0x1444,
    0x6F12,0x2100,
    0x6F12,0x2100,
    0x602A,0x144C,
    0x6F12,0x4200,
    0x6F12,0x4200,
    0x602A,0x7F6C,
    0x6F12,0x0100,
    0x6F12,0x3100,
    0x6F12,0xF700,
    0x6F12,0x2600,
    0x6F12,0xE100,
    0x602A,0x0650,
    0x6F12,0x0600,
    0x602A,0x0654,
    0x6F12,0x0000,
    0x602A,0x1A46,
    0x6F12,0x8600,
    0x602A,0x1A52,
    0x6F12,0xBF00,
    0x602A,0x0674,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x602A,0x0668,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x602A,0x0684,
    0x6F12,0x4001,
    0x602A,0x0688,
    0x6F12,0x4001,
    0x602A,0x147C,
    0x6F12,0x1000,
    0x602A,0x1480,
    0x6F12,0x1000,
    0x602A,0x19F6,
    0x6F12,0x0904,
    0x602A,0x0812,
    0x6F12,0x0000,
    0x602A,0x1A02,
    0x6F12,0x0800,
    0x602A,0x2148,
    0x6F12,0x0100,
    0x602A,0x2042,
    0x6F12,0x1A00,
    0x602A,0x0874,
    0x6F12,0x1100,
    0x602A,0x09C0,
    0x6F12,0x9800,
    0x602A,0x09C4,
    0x6F12,0x9800,
    0x602A,0x19FE,
    0x6F12,0x0E1C,
    0x602A,0x4D92,
    0x6F12,0x0100,
    0x602A,0x84C8,
    0x6F12,0x0100,
    0x602A,0x4D94,
    0x6F12,0x4001,
    0x6F12,0x0004,
    0x6F12,0x0010,
    0x6F12,0x0810,
    0x6F12,0x0004,
    0x6F12,0x0010,
    0x6F12,0x0810,
    0x6F12,0x0810,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0010,
    0x6F12,0x0010,
    0x602A,0x3570,
    0x6F12,0x0000,
    0x602A,0x3574,
    0x6F12,0x9400,
    0x602A,0x21E4,
    0x6F12,0x0400,
    0x602A,0x21EC,
    0x6F12,0x4F01,
    0x602A,0x2080,
    0x6F12,0x0100,
    0x6F12,0x7F00,
    0x602A,0x2086,
    0x6F12,0x8000,
    0x6F12,0x0002,
    0x6F12,0xC244,
    0x6F12,0xD244,
    0x6F12,0x14F4,
    0x6F12,0x141C,
    0x602A,0x208E,
    0x6F12,0x14F4,
    0x602A,0x208A,
    0x6F12,0xC244,
    0x6F12,0xD244,
    0x6F12,0x0000,
    0x602A,0x120E,
    0x6F12,0x1000,
    0x602A,0x212E,
    0x6F12,0x0A00,
    0x602A,0x13AE,
    0x6F12,0x0102,
    0x602A,0x0718,
    0x6F12,0x0005,
    0x602A,0x0710,
    0x6F12,0x0004,
    0x6F12,0x0401,
    0x6F12,0x0100,
    0x602A,0x1B5C,
    0x6F12,0x0300,
    0x602A,0x0786,
    0x6F12,0x7701,
    0x602A,0x2022,
    0x6F12,0x0101,
    0x6F12,0x0101,
    0x602A,0x1360,
    0x6F12,0x0000,
    0x602A,0x1376,
    0x6F12,0x0200,
    0x6F12,0x6038,
    0x6F12,0x7038,
    0x6F12,0x8038,
    0x602A,0x1386,
    0x6F12,0x0B00,
    0x602A,0x06FA,
    0x6F12,0x1000,
    0x602A,0x4A94,
    0x6F12,0x0C00,
    0x6F12,0x0000,
    0x6F12,0x0600,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0600,
    0x6F12,0x0000,
    0x6F12,0x0C00,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x602A,0x0A76,
    0x6F12,0x1000,
    0x602A,0x0AEE,
    0x6F12,0x1000,
    0x602A,0x0B66,
    0x6F12,0x1000,
    0x602A,0x0BDE,
    0x6F12,0x1000,
    0x602A,0x0BE8,
    0x6F12,0x3000,
    0x6F12,0x3000,
    0x602A,0x0C56,
    0x6F12,0x1000,
    0x602A,0x0C60,
    0x6F12,0x3000,
    0x6F12,0x3000,
    0x602A,0x0CB6,
    0x6F12,0x0000,
    0x602A,0x0CF2,
    0x6F12,0x0001,
    0x602A,0x0CF0,
    0x6F12,0x0101,
    0x602A,0x11B8,
    0x6F12,0x0000,
    0x602A,0x11F6,
    0x6F12,0x0010,
    0x602A,0x4A74,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0xD8FF,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0xD8FF,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x602A,0x218E,
    0x6F12,0x0000,
    0x602A,0x2268,
    0x6F12,0xF279,
    0x602A,0x5006,
    0x6F12,0x0000,
    0x602A,0x500E,
    0x6F12,0x0100,
    0x602A,0x4E70,
    0x6F12,0x2062,
    0x6F12,0x5501,
    0x602A,0x06DC,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6028,0x4000,
    0xF46A,0xAE80,
    0x0344,0x00F0,
    0x0346,0x0390,
    0x0348,0x1F0F,
    0x034A,0x148F,
    0x034C,0x0780,
    0x034E,0x0438,
    0x0350,0x0004,
    0x0352,0x0004,
    0x0900,0x0144,
    0x0380,0x0002,
    0x0382,0x0006,
    0x0384,0x0002,
    0x0386,0x0006,
    0x0110,0x1002,
    0x0114,0x0300,
    0x0116,0x3000,
    0x0136,0x1800,
    0x013E,0x0000,
    0x0300,0x0006,
    0x0302,0x0001,
    0x0304,0x0004,
    0x0306,0x0096,
    0x0308,0x0008,
    0x030A,0x0001,
    0x030C,0x0000,
    0x030E,0x0003,
    0x0310,0x0064,
    0x0312,0x0000,
    0x080E,0x0000,
    0x0340,0x0496,
    0x0342,0x10A0,
    0x0702,0x0000,
    0x0202,0x0100,
    0x0200,0x0100,
    0x0D00,0x0101,
    0x0D02,0x0101,
    0x0D04,0x0002,
    0x6226,0x0000,
};

static void custom2_setting(void)
{
    LOG_INF("%s E\n", __func__);

    table_write_cmos_sensor(addr_data_pair_custom2_jn1,
        sizeof(addr_data_pair_custom2_jn1) / sizeof(kal_uint16));

}

static kal_uint16 addr_data_pair_custom3_jn1[] = {
    //v0.4k(Set)Shine
    //8160x6144 10fps 1392Mbps/lane
    //Reference DPHY
    //4lane DPHY setting
    //Summary   Fullsize 8160x6144
    //ExtClk :  24  MHz
    //Vt_pix_clk :  560     MHz
    //MIPI_output_speed :   1392.0  Mbps/lane
    //Crop_Width :  8192    px
    //Crop_Height : 6176    px
    //Output_Width :    8160    px  Correct
    //Output_Height :   6144    px  Correct
    //Frame rate :  10.07   fps
    //Output format :   Raw10
    //H-size :  8688    px
    //H-blank : 528 px
    //V-size :  6400    line
    //V-blank : 256 line
    //V-blank time :    3.97    ms
    //Tail X :  0
    //Tail Y :  0       0
    //Lane :    4   lane
    //First Pixel : Gr  First
    0x6028, 0x2400,
    0x602A, 0x1A28,
    0x6F12, 0x4C00,
    0x602A, 0x065A,
    0x6F12, 0x0000,
    0x602A, 0x139E,
    0x6F12, 0x0400,
    0x602A, 0x139C,
    0x6F12, 0x0100,
    0x602A, 0x13A0,
    0x6F12, 0x0500,
    0x6F12, 0x0120,
    0x602A, 0x2072,
    0x6F12, 0x0101,
    0x602A, 0x1A64,
    0x6F12, 0x0001,
    0x6F12, 0x0000,
    0x602A, 0x19E6,
    0x6F12, 0x0200,
    0x602A, 0x1A30,
    0x6F12, 0x3403,
    0x602A, 0x19FC,
    0x6F12, 0x0700,
    0x602A, 0x19F4,
    0x6F12, 0x0707,
    0x602A, 0x19F8,
    0x6F12, 0x0B0B,
    0x602A, 0x1B26,
    0x6F12, 0x6F80,
    0x6F12, 0xA060,
    0x602A, 0x1A3C,
    0x6F12, 0x8207,
    0x602A, 0x1A48,
    0x6F12, 0x8207,
    0x602A, 0x1444,
    0x6F12, 0x2000,
    0x6F12, 0x2000,
    0x602A, 0x144C,
    0x6F12, 0x3F00,
    0x6F12, 0x3F00,
    0x602A, 0x7F6C,
    0x6F12, 0x0100,
    0x6F12, 0x2F00,
    0x6F12, 0xFA00,
    0x6F12, 0x2400,
    0x6F12, 0xE500,
    0x602A, 0x0650,
    0x6F12, 0x0600,
    0x602A, 0x0654,
    0x6F12, 0x0000,
    0x602A, 0x1A46,
    0x6F12, 0x8500,
    0x602A, 0x1A52,
    0x6F12, 0x9800,
    0x602A, 0x0674,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x0668,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x602A, 0x0684,
    0x6F12, 0x4001,
    0x602A, 0x0688,
    0x6F12, 0x4001,
    0x602A, 0x147C,
    0x6F12, 0x0400,
    0x602A, 0x1480,
    0x6F12, 0x0400,
    0x602A, 0x19F6,
    0x6F12, 0x0404,
    0x602A, 0x0812,
    0x6F12, 0x0000,
    0x602A, 0x1A02,
    0x6F12, 0x1800,
    0x602A, 0x2148,
    0x6F12, 0x0100,
    0x602A, 0x2042,
    0x6F12, 0x1A00,
    0x602A, 0x0874,
    0x6F12, 0x0106,
    0x602A, 0x09C0,
    0x6F12, 0x4000,
    0x602A, 0x09C4,
    0x6F12, 0x4000,
    0x602A, 0x19FE,
    0x6F12, 0x0C1C,
    0x602A, 0x4D92,
    0x6F12, 0x0000,
    0x602A, 0x84C8,
    0x6F12, 0x0000,
    0x602A, 0x4D94,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x3570,
    0x6F12, 0x0000,
    0x602A, 0x3574,
    0x6F12, 0x7306,
    0x602A, 0x21E4,
    0x6F12, 0x0400,
    0x602A, 0x21EC,
    0x6F12, 0x6902,
    0x602A, 0x2080,
    0x6F12, 0x0100,
    0x6F12, 0xFF00,
    0x6F12, 0x0002,
    0x6F12, 0x0001,
    0x6F12, 0x0002,
    0x6F12, 0xD244,
    0x6F12, 0xD244,
    0x6F12, 0x14F4,
    0x6F12, 0x101C,
    0x6F12, 0x0D1C,
    0x6F12, 0x54F4,
    0x602A, 0x20BA,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x120E,
    0x6F12, 0x1000,
    0x602A, 0x212E,
    0x6F12, 0x0200,
    0x602A, 0x13AE,
    0x6F12, 0x0100,
    0x602A, 0x0718,
    0x6F12, 0x0000,
    0x602A, 0x0710,
    0x6F12, 0x0010,
    0x6F12, 0x0201,
    0x6F12, 0x0800,
    0x602A, 0x1B5C,
    0x6F12, 0x0000,
    0x602A, 0x0786,
    0x6F12, 0x1401,
    0x602A, 0x2022,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x1360,
    0x6F12, 0x0000,
    0x602A, 0x1376,
    0x6F12, 0x0000,
    0x6F12, 0x6038,
    0x6F12, 0x7038,
    0x6F12, 0x8038,
    0x602A, 0x1386,
    0x6F12, 0x0B00,
    0x602A, 0x06FA,
    0x6F12, 0x1000,
    0x602A, 0x4A94,
    0x6F12, 0x0400,
    0x6F12, 0x0400,
    0x6F12, 0x0400,
    0x6F12, 0x0400,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0400,
    0x6F12, 0x0400,
    0x6F12, 0x0400,
    0x6F12, 0x0400,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x602A, 0x0A76,
    0x6F12, 0x1000,
    0x602A, 0x0AEE,
    0x6F12, 0x1000,
    0x602A, 0x0B66,
    0x6F12, 0x1000,
    0x602A, 0x0BDE,
    0x6F12, 0x1000,
    0x602A, 0x0BE8,
    0x6F12, 0x5000,
    0x6F12, 0x5000,
    0x602A, 0x0C56,
    0x6F12, 0x1000,
    0x602A, 0x0C60,
    0x6F12, 0x5000,
    0x6F12, 0x5000,
    0x602A, 0x0CB6,
    0x6F12, 0x0000,
    0x602A, 0x0CF2,
    0x6F12, 0x0001,
    0x602A, 0x0CF0,
    0x6F12, 0x0101,
    0x602A, 0x11B8,
    0x6F12, 0x0000,
    0x602A, 0x11F6,
    0x6F12, 0x0010,
    0x602A, 0x4A74,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x218E,
    0x6F12, 0x0000,
    0x602A, 0x2268,
    0x6F12, 0xF279,
    0x602A, 0x5006,
    0x6F12, 0x0000,
    0x602A, 0x500E,
    0x6F12, 0x0100,
    0x602A, 0x4E70,
    0x6F12, 0x2062,
    0x6F12, 0x5501,
    0x602A, 0x06DC,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6028, 0x4000,
    0xF46A, 0xAE80,
    0x0344, 0x0000,
    0x0346, 0x0000,
    0x0348, 0x1FFF,
    0x034A, 0x181F,
    0x034C, 0x1FE0,
    0x034E, 0x1800,
    0x0350, 0x0010,
    0x0352, 0x0010,
    0x0900, 0x0111,
    0x0380, 0x0001,
    0x0382, 0x0001,
    0x0384, 0x0001,
    0x0386, 0x0001,
    0x0110, 0x1002,
    0x0114, 0x0300,
    0x0116, 0x3000,
    0x0136, 0x1800,
    0x013E, 0x0000,
    0x0300, 0x0006,
    0x0302, 0x0001,
    0x0304, 0x0004,
    0x0306, 0x008C,
    0x0308, 0x0008,
    0x030A, 0x0001,
    0x030C, 0x0000,
    0x030E, 0x0004,
    0x0310, 0x0074,
    0x0312, 0x0000,
    0x080E, 0x0000,
    0x0340, 0x1900,
    0x0342, 0x21F0,
    0x0702, 0x0000,
    0x0202, 0x0100,
    0x0200, 0x0100,
    0x0D00, 0x0100,
    0x0D02, 0x0001,
    0x0D04, 0x0002,
    0x6226, 0x0000,
};

static void custom3_setting(void)
{
    LOG_INF("%s E\n", __func__);

    table_write_cmos_sensor(addr_data_pair_custom3_jn1,
        sizeof(addr_data_pair_custom3_jn1) / sizeof(kal_uint16));

}

static kal_uint16 addr_data_pair_custom4_jn1[] = {
//Summary   FDSUM+A2A2 1920x1080
//ExtClk :  24  MHz
//Vt_pix_clk :  600     MHz
//MIPI_output_speed :   800.0   Mbps/lane
//Output_Width :    1920    px
//Output_Height :   1080    px
//Frame rate :  60.04   fps
//Output format :   Raw10
//H-size :  5824    px  944
//H-blank : 3904    px  7935
//V-size :  1716    line    5247
//V-blank : 636 line    5231
//V-blank time :    6.17    ms
//Tail X :  480
//Tail Y :  536     x_0 , y_4
//Lane :    4   lane
//First Pixel : Gr  First
    0x6028, 0x2400,
    0x602A, 0x1A28,
    0x6F12, 0x4C00,
    0x602A, 0x065A,
    0x6F12, 0x0000,
    0x602A, 0x139E,
    0x6F12, 0x0300,
    0x602A, 0x139C,
    0x6F12, 0x0000,
    0x602A, 0x13A0,
    0x6F12, 0x0A00,
    0x6F12, 0x0020,
    0x602A, 0x2072,
    0x6F12, 0x0000,
    0x602A, 0x1A64,
    0x6F12, 0x0301,
    0x6F12, 0x3F00,
    0x602A, 0x19E6,
    0x6F12, 0x0201,
    0x602A, 0x1A30,
    0x6F12, 0x3401,
    0x602A, 0x19FC,
    0x6F12, 0x0B00,
    0x602A, 0x19F4,
    0x6F12, 0x0606,
    0x602A, 0x19F8,
    0x6F12, 0x1010,
    0x602A, 0x1B26,
    0x6F12, 0x6F80,
    0x6F12, 0xA020,
    0x602A, 0x1A3C,
    0x6F12, 0x5207,
    0x602A, 0x1A48,
    0x6F12, 0x5207,
    0x602A, 0x1444,
    0x6F12, 0x2100,
    0x6F12, 0x2100,
    0x602A, 0x144C,
    0x6F12, 0x4200,
    0x6F12, 0x4200,
    0x602A, 0x7F6C,
    0x6F12, 0x0100,
    0x6F12, 0x3100,
    0x6F12, 0xF700,
    0x6F12, 0x2600,
    0x6F12, 0xE100,
    0x602A, 0x0650,
    0x6F12, 0x0600,
    0x602A, 0x0654,
    0x6F12, 0x0000,
    0x602A, 0x1A46,
    0x6F12, 0x8600,
    0x602A, 0x1A52,
    0x6F12, 0xBF00,
    0x602A, 0x0674,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x0668,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x602A, 0x0684,
    0x6F12, 0x4001,
    0x602A, 0x0688,
    0x6F12, 0x4001,
    0x602A, 0x147C,
    0x6F12, 0x1000,
    0x602A, 0x1480,
    0x6F12, 0x1000,
    0x602A, 0x19F6,
    0x6F12, 0x0904,
    0x602A, 0x0812,
    0x6F12, 0x0000,
    0x602A, 0x1A02,
    0x6F12, 0x0800,
    0x602A, 0x2148,
    0x6F12, 0x0100,
    0x602A, 0x2042,
    0x6F12, 0x1A00,
    0x602A, 0x0874,
    0x6F12, 0x1100,
    0x602A, 0x09C0,
    0x6F12, 0x9800,
    0x602A, 0x09C4,
    0x6F12, 0x9800,
    0x602A, 0x19FE,
    0x6F12, 0x0E1C,
    0x602A, 0x4D92,
    0x6F12, 0x0100,
    0x602A, 0x84C8,
    0x6F12, 0x0100,
    0x602A, 0x4D94,
    0x6F12, 0x4001,
    0x6F12, 0x0004,
    0x6F12, 0x0010,
    0x6F12, 0x0810,
    0x6F12, 0x0004,
    0x6F12, 0x0010,
    0x6F12, 0x0810,
    0x6F12, 0x0810,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0010,
    0x6F12, 0x0010,
    0x602A, 0x3570,
    0x6F12, 0x0000,
    0x602A, 0x3574,
    0x6F12, 0x0000,
    0x602A, 0x21E4,
    0x6F12, 0x0400,
    0x602A, 0x21EC,
    0x6F12, 0x0000,
    0x602A, 0x2080,
    0x6F12, 0x0100,
    0x6F12, 0x7F00,
    0x6F12, 0x0002,
    0x6F12, 0x8000,
    0x6F12, 0x0002,
    0x6F12, 0xC244,
    0x6F12, 0xD244,
    0x6F12, 0x14F4,
    0x6F12, 0x141C,
    0x6F12, 0x111C,
    0x6F12, 0x54F4,
    0x602A, 0x20BA,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x120E,
    0x6F12, 0x1000,
    0x602A, 0x212E,
    0x6F12, 0x0A00,
    0x602A, 0x13AE,
    0x6F12, 0x0102,
    0x602A, 0x0718,
    0x6F12, 0x0005,
    0x602A, 0x0710,
    0x6F12, 0x0004,
    0x6F12, 0x0401,
    0x6F12, 0x0100,
    0x602A, 0x1B5C,
    0x6F12, 0x0300,
    0x602A, 0x0786,
    0x6F12, 0x7701,
    0x602A, 0x2022,
    0x6F12, 0x0101,
    0x6F12, 0x0101,
    0x602A, 0x1360,
    0x6F12, 0x0000,
    0x602A, 0x1376,
    0x6F12, 0x0200,
    0x6F12, 0x6038,
    0x6F12, 0x7038,
    0x6F12, 0x8038,
    0x602A, 0x1386,
    0x6F12, 0x0B00,
    0x602A, 0x06FA,
    0x6F12, 0x1000,
    0x602A, 0x4A94,
    0x6F12, 0x0C00,
    0x6F12, 0x0000,
    0x6F12, 0x0600,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0600,
    0x6F12, 0x0000,
    0x6F12, 0x0C00,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x0A76,
    0x6F12, 0x1000,
    0x602A, 0x0AEE,
    0x6F12, 0x1000,
    0x602A, 0x0B66,
    0x6F12, 0x1000,
    0x602A, 0x0BDE,
    0x6F12, 0x1000,
    0x602A, 0x0BE8,
    0x6F12, 0x3000,
    0x6F12, 0x3000,
    0x602A, 0x0C56,
    0x6F12, 0x1000,
    0x602A, 0x0C60,
    0x6F12, 0x3000,
    0x6F12, 0x3000,
    0x602A, 0x0CB6,
    0x6F12, 0x0000,
    0x602A, 0x0CF2,
    0x6F12, 0x0001,
    0x602A, 0x0CF0,
    0x6F12, 0x0101,
    0x602A, 0x11B8,
    0x6F12, 0x0000,
    0x602A, 0x11F6,
    0x6F12, 0x0010,
    0x602A, 0x4A74,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x218E,
    0x6F12, 0x0000,
    0x602A, 0x2268,
    0x6F12, 0xF279,
    0x602A, 0x5006,
    0x6F12, 0x0000,
    0x602A, 0x500E,
    0x6F12, 0x0100,
    0x602A, 0x4E70,
    0x6F12, 0x2062,
    0x6F12, 0x5501,
    0x602A, 0x06DC,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6028, 0x4000,
    0xF46A, 0xAE80,
    0x0344, 0x00F0,
    0x0346, 0x0390,
    0x0348, 0x1F0F,
    0x034A, 0x148F,
    0x034C, 0x0780,
    0x034E, 0x0438,
    0x0350, 0x0004,
    0x0352, 0x0004,
    0x0900, 0x0144,
    0x0380, 0x0002,
    0x0382, 0x0006,
    0x0384, 0x0002,
    0x0386, 0x0006,
    0x0110, 0x1002,
    0x0114, 0x0301,
    0x0116, 0x3000,
    0x0136, 0x1800,
    0x013E, 0x0000,
    0x0300, 0x0006,
    0x0302, 0x0001,
    0x0304, 0x0004,
    0x0306, 0x0096,
    0x0308, 0x0008,
    0x030A, 0x0001,
    0x030C, 0x0000,
    0x030E, 0x0003,
    0x0310, 0x0064,
    0x0312, 0x0001,
    0x080E, 0x0000,
    0x0340, 0x06B4,
    0x0342, 0x16C0,
    0x0702, 0x0000,
    0x0202, 0x0100,
    0x0200, 0x0100,
    0x0D00, 0x0101,
    0x0D02, 0x0101,
    0x0D04, 0x0102,
    0x6226, 0x0000,
};


static void custom4_setting(void)
{
    LOG_INF("%s E\n", __func__);
    table_write_cmos_sensor(addr_data_pair_custom4_jn1,
            sizeof(addr_data_pair_custom4_jn1) / sizeof(kal_uint16));
}

static kal_uint16 addr_data_pair_custom5_jn1[] = {
    0x6028, 0x2400,
    0x602A, 0x1A28,
    0x6F12, 0x4C00,
    0x602A, 0x065A,
    0x6F12, 0x0000,
    0x602A, 0x139E,
    0x6F12, 0x0100,
    0x602A, 0x139C,
    0x6F12, 0x0000,
    0x602A, 0x13A0,
    0x6F12, 0x0A00,
    0x6F12, 0x0120,
    0x602A, 0x2072,
    0x6F12, 0x0000,
    0x602A, 0x1A64,
    0x6F12, 0x0301,
    0x6F12, 0xFF00,
    0x602A, 0x19E6,
    0x6F12, 0x0200,
    0x602A, 0x1A30,
    0x6F12, 0x3401,
    0x602A, 0x19FC,
    0x6F12, 0x0B00,
    0x602A, 0x19F4,
    0x6F12, 0x0606,
    0x602A, 0x19F8,
    0x6F12, 0x1010,
    0x602A, 0x1B26,
    0x6F12, 0x6F80,
    0x6F12, 0xA060,
    0x602A, 0x1A3C,
    0x6F12, 0x6207,
    0x602A, 0x1A48,
    0x6F12, 0x6207,
    0x602A, 0x1444,
    0x6F12, 0x2000,
    0x6F12, 0x2000,
    0x602A, 0x144C,
    0x6F12, 0x3F00,
    0x6F12, 0x3F00,
    0x602A, 0x7F6C,
    0x6F12, 0x0100,
    0x6F12, 0x2F00,
    0x6F12, 0xFA00,
    0x6F12, 0x2400,
    0x6F12, 0xE500,
    0x602A, 0x0650,
    0x6F12, 0x0600,
    0x602A, 0x0654,
    0x6F12, 0x0000,
    0x602A, 0x1A46,
    0x6F12, 0x8A00,
    0x602A, 0x1A52,
    0x6F12, 0xBF00,
    0x602A, 0x0674,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x0668,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x602A, 0x0684,
    0x6F12, 0x4001,
    0x602A, 0x0688,
    0x6F12, 0x4001,
    0x602A, 0x147C,
    0x6F12, 0x1000,
    0x602A, 0x1480,
    0x6F12, 0x1000,
    0x602A, 0x19F6,
    0x6F12, 0x0904,
    0x602A, 0x0812,
    0x6F12, 0x0000,
    0x602A, 0x1A02,
    0x6F12, 0x1800,
    0x602A, 0x2148,
    0x6F12, 0x0100,
    0x602A, 0x2042,
    0x6F12, 0x1A00,
    0x602A, 0x0874,
    0x6F12, 0x0100,
    0x602A, 0x09C0,
    0x6F12, 0x2008,
    0x602A, 0x09C4,
    0x6F12, 0x2000,
    0x602A, 0x19FE,
    0x6F12, 0x0E1C,
    0x602A, 0x4D92,
    0x6F12, 0x0100,
    0x602A, 0x84C8,
    0x6F12, 0x0100,
    0x602A, 0x4D94,
    0x6F12, 0x0005,
    0x6F12, 0x000A,
    0x6F12, 0x0010,
    0x6F12, 0x0810,
    0x6F12, 0x000A,
    0x6F12, 0x0040,
    0x6F12, 0x0810,
    0x6F12, 0x0810,
    0x6F12, 0x8002,
    0x6F12, 0xFD03,
    0x6F12, 0x0010,
    0x6F12, 0x1510,
    0x602A, 0x3570,
    0x6F12, 0x0000,
    0x602A, 0x3574,
    0x6F12, 0x1201,
    0x602A, 0x21E4,
    0x6F12, 0x0400,
    0x602A, 0x21EC,
    0x6F12, 0x1F04,
    0x602A, 0x2080,
    0x6F12, 0x0101,
    0x6F12, 0xFF00,
    0x6F12, 0x7F01,
    0x6F12, 0x0001,
    0x6F12, 0x8001,
    0x6F12, 0xD244,
    0x6F12, 0xD244,
    0x6F12, 0x14F4,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x20BA,
    0x6F12, 0x121C,
    0x6F12, 0x111C,
    0x6F12, 0x54F4,
    0x602A, 0x120E,
    0x6F12, 0x1000,
    0x602A, 0x212E,
    0x6F12, 0x0200,
    0x602A, 0x13AE,
    0x6F12, 0x0101,
    0x602A, 0x0718,
    0x6F12, 0x0001,
    0x602A, 0x0710,
    0x6F12, 0x0002,
    0x6F12, 0x0804,
    0x6F12, 0x0100,
    0x602A, 0x1B5C,
    0x6F12, 0x0000,
    0x602A, 0x0786,
    0x6F12, 0x7701,
    0x602A, 0x2022,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x1360,
    0x6F12, 0x0100,
    0x602A, 0x1376,
    0x6F12, 0x0100,
    0x6F12, 0x6038,
    0x6F12, 0x7038,
    0x6F12, 0x8038,
    0x602A, 0x1386,
    0x6F12, 0x0B00,
    0x602A, 0x06FA,
    0x6F12, 0x1000,
    0x602A, 0x0000,
    0x6F12, 0x0900,
    0x6F12, 0x0000,
    0x6F12, 0x0300,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0300,
    0x6F12, 0x0000,
    0x6F12, 0x0900,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x0A76,
    0x6F12, 0x1000,
    0x602A, 0x0AEE,
    0x6F12, 0x1000,
    0x602A, 0x0B66,
    0x6F12, 0x1000,
    0x602A, 0x0BDE,
    0x6F12, 0x1000,
    0x602A, 0x0BE8,
    0x6F12, 0x3000,
    0x6F12, 0x3000,
    0x602A, 0x0C56,
    0x6F12, 0x1000,
    0x602A, 0x0C60,
    0x6F12, 0x3000,
    0x6F12, 0x3000,
    0x602A, 0x0CB6,
    0x6F12, 0x0100,
    0x602A, 0x0CF2,
    0x6F12, 0x0001,
    0x602A, 0x0CF0,
    0x6F12, 0x0101,
    0x602A, 0x11B8,
    0x6F12, 0x0100,
    0x602A, 0x11F6,
    0x6F12, 0x0020,
    0x602A, 0x4A74,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x218E,
    0x6F12, 0x0000,
    0x602A, 0x2268,
    0x6F12, 0xF279,
    0x602A, 0x5006,
    0x6F12, 0x0000,
    0x602A, 0x500E,
    0x6F12, 0x0100,
    0x602A, 0x4E70,
    0x6F12, 0x2062,
    0x6F12, 0x5501,
    0x602A, 0x06DC,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6028, 0x4000,
    0xF46A, 0xAE80,
    0x0344, 0x01B0,
    0x0346, 0x0150,
    0x0348, 0x1E4F,
    0x034A, 0x16CF,
    0x034C, 0x0E40,
    0x034E, 0x0AB0,
    0x0350, 0x0008,
    0x0352, 0x0008,
    0x0900, 0x0122,
    0x0380, 0x0002,
    0x0382, 0x0002,
    0x0384, 0x0002,
    0x0386, 0x0002,
    0x0110, 0x1002,
    0x0114, 0x0301,
    0x0116, 0x3000,
    0x0136, 0x1800,
    0x013E, 0x0000,
    0x0300, 0x0006,
    0x0302, 0x0001,
    0x0304, 0x0004,
    0x0306, 0x008C,
    0x0308, 0x0008,
    0x030A, 0x0001,
    0x030C, 0x0000,
    0x030E, 0x0003,
    0x0310, 0x007D,
    0x0312, 0x0001,
    0x080E, 0x0000,
    0x0340, 0x0DEF,
    0x0342, 0x1986,
    0x0702, 0x0000,
    0x0202, 0x0100,
    0x0200, 0x0100,
    0x0D00, 0x0101,
    0x0D02, 0x0101,
    0x0D04, 0x0102,
    0x6226, 0x0000,
};

static void custom5_setting(void)
{
    printk("%s E\n", __func__);

    //sensor_init();
    /*240FPS */

    /*// Convert from : "Init.txt"*/
    /*check_streamoff();*/
    table_write_cmos_sensor(addr_data_pair_custom5_jn1,
        sizeof(addr_data_pair_custom5_jn1) / sizeof(kal_uint16));
}

// //#ifdef OPLUS_FEATURE_CAMERA_COMMON
static BYTE s5kjn1_23707_common_data[CAMERA_EEPPROM_COMDATA_LENGTH] = { 0 };

static kal_uint16 read_s5kjn1_23707_eeprom_module(kal_uint32 addr)
{
    kal_uint16 get_byte = 0;
    char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

    iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 1, EEPROM_I2C_ADDR);

    return get_byte;
}

static void read_s5kjn1_23707_module_data(UINT32 sensor_id)
{
// s5kjn1_23707_common_data
// length   : 64
// QR       : [8 ~ 26]
// vcm      : [40 ~ 41]
// lensid   : [44 ~ 45]
// macro    : [48 ~ 49]
// inf      : [52 ~ 53]
// sensorid : [30 ~ 33]

    kal_uint16 idx = 0;
    kal_uint16 sn_length = 17;
    kal_uint32 sn_starAddr = 0xB0;
    kal_uint32 vcmAddr = 0x0C;
    kal_uint32 lensAddr = 0x08;
    kal_uint32 macAddr_lsb = 0x92;
    kal_uint32 macAddr_msb = 0x93;
    kal_uint32 infAddr_lsb = 0x94;
    kal_uint32 infAddr_msb = 0x95;

    memset(s5kjn1_23707_common_data, 0,sizeof(s5kjn1_23707_common_data));
    // QR
    for(idx = 0; idx < sn_length; idx++)
    {
        s5kjn1_23707_common_data[8 + idx] = read_s5kjn1_23707_eeprom_module(sn_starAddr + idx);
    }
    //vcm
    s5kjn1_23707_common_data[40] = read_s5kjn1_23707_eeprom_module(vcmAddr);
    s5kjn1_23707_common_data[41] = read_s5kjn1_23707_eeprom_module(vcmAddr + 1);
    //lensid
    s5kjn1_23707_common_data[44] = read_s5kjn1_23707_eeprom_module(lensAddr);
    s5kjn1_23707_common_data[45] = read_s5kjn1_23707_eeprom_module(lensAddr + 1);
    //macro
    s5kjn1_23707_common_data[48] = read_s5kjn1_23707_eeprom_module(macAddr_lsb);
    s5kjn1_23707_common_data[49] = read_s5kjn1_23707_eeprom_module(macAddr_msb);
    //inf
    s5kjn1_23707_common_data[52] = read_s5kjn1_23707_eeprom_module(infAddr_lsb);
    s5kjn1_23707_common_data[53] = read_s5kjn1_23707_eeprom_module(infAddr_msb);
    //sensor id
    s5kjn1_23707_common_data[30] = ((sensor_id) >> 24) & 0xff;
    s5kjn1_23707_common_data[31] = ((sensor_id) >> 16) & 0xff;
    s5kjn1_23707_common_data[32] = ((sensor_id) >> 8) & 0xff;
    s5kjn1_23707_common_data[33] = (sensor_id) & 0xff;

    for (idx = 0; idx < CAMERA_EEPPROM_COMDATA_LENGTH; idx = idx + 4)
        LOG_INF("cam data: %02x %02x %02x %02x\n",
               s5kjn1_23707_common_data[idx],
               s5kjn1_23707_common_data[idx + 1],
               s5kjn1_23707_common_data[idx + 2],
               s5kjn1_23707_common_data[idx + 3]);
}

//#endif  /* OPLUS_FEATURE_CAMERA_COMMON */

/*************************************************************************
 * FUNCTION
 *  get_imgsensor_id
 *
 * DESCRIPTION
 *  This function get the sensor ID
 *
 * PARAMETERS
 *  *sensorID : return the sensor ID
 *
 * RETURNS
 *  None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 5;
    kal_uint8 module_id = 0;
    /* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
     *we should detect the module used i2c address
     */
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id() + 8;

            if (*sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("find sensor: 0x%x, sensor id: 0x%x\n",
                    imgsensor.i2c_write_id, *sensor_id);
// //#ifdef OPLUS_FEATURE_CAMERA_COMMON
                module_id = read_s5kjn1_23707_eeprom_module(MODULE_ID_OFFSET);
                if (deviceInfo_register_value == 0x00) {
                    Eeprom_DataInit(0, S5KJN1_SENSOR_ID23707);
                    deviceInfo_register_value = 0x01;
                }
                read_s5kjn1_23707_module_data(*sensor_id);
// //#endif  /* OPLUS_FEATURE_CAMERA_COMMON */
                return ERROR_NONE;
            }
            LOG_INF("Read sensor id fail, i2c write id: 0x%x, read sensor id: 0x%x\n",
                imgsensor.i2c_write_id, *sensor_id);
            retry--;
            mdelay(2);
        } while (retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
    /* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *  open
 *
 * DESCRIPTION
 *  This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *  None
 *
 * RETURNS
 *  None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 5;
    kal_uint16 sensor_id = 0;

    pr_debug("%s", __func__);

    /* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
     * we should detect the module used i2c address
     */
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id() + 8;
            if (sensor_id == imgsensor_info.sensor_id) {
                pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n",
                    imgsensor.i2c_write_id, sensor_id);
                break;
            }
            pr_debug("Read sensor id fail, id: 0x%x\n",
                imgsensor.i2c_write_id);
            retry--;
            mdelay(2);
        } while (retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }

    if (imgsensor_info.sensor_id !=  sensor_id) {
        return ERROR_SENSOR_CONNECT_FAIL;
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
    imgsensor.jn1_binning_mode = imgsensor_info.pre.jn1_binning_mode;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}               /*      open  */

/*************************************************************************
 * FUNCTION
 *  close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *  None
 *
 * RETURNS
 *  None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
    pr_debug("E\n");

    /*No Need to implement this function */

    return ERROR_NONE;
}               /*      close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *  This function start the sensor preview.
 *
 * PARAMETERS
 *  *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *  None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    pr_debug("%s E\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.jn1_binning_mode = imgsensor_info.pre.jn1_binning_mode;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    preview_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}               /*      preview   */

/*************************************************************************
 * FUNCTION
 *  capture
 *
 * DESCRIPTION
 *  This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *  None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    pr_debug("%s E\n", __func__);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
    /* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.jn1_binning_mode = imgsensor_info.cap1.jn1_binning_mode;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    } else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
        imgsensor.pclk = imgsensor_info.cap2.pclk;
        imgsensor.line_length = imgsensor_info.cap2.linelength;
        imgsensor.frame_length = imgsensor_info.cap2.framelength;
        imgsensor.jn1_binning_mode = imgsensor_info.cap2.jn1_binning_mode;
        imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    } else {

        /*if (imgsensor.current_fps != imgsensor_info.cap.max_framerate) {
            pr_debug("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
                imgsensor.current_fps,
                imgsensor_info.cap.max_framerate / 10);
        }*/

        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.jn1_binning_mode = imgsensor_info.cap.jn1_binning_mode;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    spin_unlock(&imgsensor_drv_lock);

    capture_setting(imgsensor.current_fps);
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}               /* capture() */

static kal_uint32 normal_video(
    MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    pr_debug("%s E\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.jn1_binning_mode = imgsensor_info.normal_video.jn1_binning_mode;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    normal_video_setting(imgsensor.current_fps);
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}               /*      normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
               MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    pr_debug("%s E\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    /* imgsensor.video_mode = KAL_TRUE; */
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.jn1_binning_mode = imgsensor_info.hs_video.jn1_binning_mode;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    /* imgsensor.current_fps = 300; */
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}               /*      hs_video   */

static kal_uint32 slim_video(
    MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    pr_debug("%s E\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    /* imgsensor.video_mode = KAL_TRUE; */
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.jn1_binning_mode = imgsensor_info.slim_video.jn1_binning_mode;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    /* imgsensor.current_fps = 300; */
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}               /*      slim_video       */

static kal_uint32 custom1(
    MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT * image_window,
    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    pr_debug("%s E\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    /* imgsensor.video_mode = KAL_TRUE; */
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength;
    imgsensor.jn1_binning_mode = imgsensor_info.custom1.jn1_binning_mode;
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    /* imgsensor.current_fps = 300; */
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom1_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}               /*      custom1       */

static kal_uint32 custom2(
    MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT * image_window,
    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s E\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    /* imgsensor.video_mode = KAL_TRUE; */
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength;
    imgsensor.jn1_binning_mode = imgsensor_info.custom2.jn1_binning_mode;
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    /* imgsensor.current_fps = 300; */
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom2_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}               /*      custom2       */

static kal_uint32 custom3(
    MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT * image_window,
    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s E\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    /* imgsensor.video_mode = KAL_TRUE; */
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength;
    imgsensor.jn1_binning_mode = imgsensor_info.custom3.jn1_binning_mode;
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    /* imgsensor.current_fps = 600; */
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom3_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}               /*      custom3       */

static kal_uint32 custom4(
    MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s E\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    imgsensor.pclk = imgsensor_info.custom4.pclk;
    /* imgsensor.video_mode = KAL_TRUE; */
    imgsensor.line_length = imgsensor_info.custom4.linelength;
    imgsensor.frame_length = imgsensor_info.custom4.framelength;
    imgsensor.jn1_binning_mode = imgsensor_info.custom4.jn1_binning_mode;
    imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    /* imgsensor.current_fps = 600; */
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom4_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}               /*      custom4       */

static kal_uint32 custom5(
    MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT * image_window,
    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    pr_debug("%s E\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
    imgsensor.pclk = imgsensor_info.custom5.pclk;
    /* imgsensor.video_mode = KAL_TRUE; */
    imgsensor.line_length = imgsensor_info.custom5.linelength;
    imgsensor.frame_length = imgsensor_info.custom5.framelength;
    imgsensor.jn1_binning_mode = imgsensor_info.custom5.jn1_binning_mode;
    imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    /* imgsensor.current_fps = 300; */
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom5_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}               /*      custom5       */

static kal_uint32 get_resolution(
    MSDK_SENSOR_RESOLUTION_INFO_STRUCT(*sensor_resolution))
{
    pr_debug("%s E\n", __func__);
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

    return ERROR_NONE;
}               /*      get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
               MSDK_SENSOR_INFO_STRUCT *sensor_info,
               MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    /*pr_debug("get_info -> scenario_id = %d\n", scenario_id);*/

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;

    /* not use */
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;

    /* inverse with datasheet */
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

    sensor_info->Custom1DelayFrame =
        imgsensor_info.custom1_delay_frame;

    sensor_info->Custom2DelayFrame =
        imgsensor_info.custom2_delay_frame;

    sensor_info->Custom3DelayFrame =
        imgsensor_info.custom3_delay_frame;

    sensor_info->Custom4DelayFrame =
        imgsensor_info.custom4_delay_frame;

    sensor_info->Custom5DelayFrame =
        imgsensor_info.custom5_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0;   /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    /* The frame of setting shutter default 0 for TG int */
    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

    /* The frame of setting sensor gain*/
    sensor_info->AESensorGainDelayFrame =
                imgsensor_info.ae_sensor_gain_delay_frame;

    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;

    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    /* change pdaf support mode to pdaf VC mode */
    sensor_info->PDAF_Support = PDAF_SUPPORT_CAMSV;
    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2;   /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2;  /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;   /* 0 is default 1x */
    sensor_info->SensorHightSampling = 0;   /* 0 is default 1x */
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
    default:
        sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
        break;
    }

    return ERROR_NONE;
}               /*      get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
              MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);
    lastcamstate = camstate;
    camstate = scenario_id;

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
    default:
        pr_debug("Error ScenarioId setting");
        preview(image_window, sensor_config_data);
        return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}               /* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
    /* //pr_debug("framerate = %d\n ", framerate); */
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
    set_max_framerate(imgsensor.current_fps, 1);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(
    kal_bool enable, UINT16 framerate)
{
    pr_debug("enable = %d, framerate = %d\n", enable, framerate);
    spin_lock(&imgsensor_drv_lock);
    if (enable)     /* enable auto flicker */
        imgsensor.autoflicker_en = KAL_TRUE;
    else            /* Cancel Auto flick */
        imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(
enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    pr_debug("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        frame_length = imgsensor_info.pre.pclk
            / framerate * 10 / imgsensor_info.pre.linelength;
        spin_lock(&imgsensor_drv_lock);

        imgsensor.dummy_line =
            (frame_length > imgsensor_info.pre.framelength)
            ? (frame_length - imgsensor_info.pre.framelength) : 0;

        imgsensor.frame_length =
            imgsensor_info.pre.framelength + imgsensor.dummy_line;

        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        else {
            /*No need to set*/
            pr_debug("frame_length %d < shutter %d",
                imgsensor.frame_length, imgsensor.shutter);
        }
        break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        if (framerate == 0)
            return ERROR_NONE;
        frame_length = imgsensor_info.normal_video.pclk
            / framerate * 10 / imgsensor_info.normal_video.linelength;

        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
        (frame_length > imgsensor_info.normal_video.framelength)
      ? (frame_length - imgsensor_info.normal_video.  framelength) : 0;

        imgsensor.frame_length =
         imgsensor_info.normal_video.framelength + imgsensor.dummy_line;

        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        else {
            /*No need to set*/
            pr_debug("frame_length %d < shutter %d",
                imgsensor.frame_length, imgsensor.shutter);
        }
        break;

    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {

        frame_length = imgsensor_info.cap1.pclk
            / framerate * 10 / imgsensor_info.cap1.linelength;

        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
              (frame_length > imgsensor_info.cap1.framelength)
            ? (frame_length - imgsensor_info.cap1.  framelength) : 0;

        imgsensor.frame_length =
            imgsensor_info.cap1.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        else {
            /*No need to set*/
            pr_debug("frame_length %d < shutter %d",
                imgsensor.frame_length, imgsensor.shutter);
        }
    } else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
        frame_length = imgsensor_info.cap2.pclk
            / framerate * 10 / imgsensor_info.cap2.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
              (frame_length > imgsensor_info.cap2.framelength)
            ? (frame_length - imgsensor_info.cap2.  framelength) : 0;

        imgsensor.frame_length =
            imgsensor_info.cap2.framelength + imgsensor.dummy_line;

        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        else {
            /*No need to set*/
            pr_debug("frame_length %d < shutter %d",
                imgsensor.frame_length, imgsensor.shutter);
        }
    } else {
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            pr_debug("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
                framerate,
                imgsensor_info.cap.max_framerate / 10);

        frame_length = imgsensor_info.cap.pclk
            / framerate * 10 / imgsensor_info.cap.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.cap.framelength)
            ? (frame_length - imgsensor_info.cap.framelength) : 0;
        imgsensor.frame_length =
            imgsensor_info.cap.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        else {
            /*No need to set*/
            pr_debug("frame_length %d < shutter %d",
                imgsensor.frame_length, imgsensor.shutter);
        }
    }
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        else {
            /*No need to set*/
            pr_debug("frame_length %d < shutter %d",
                imgsensor.frame_length, imgsensor.shutter);
        }
        break;

    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        frame_length = imgsensor_info.hs_video.pclk
            / framerate * 10 / imgsensor_info.hs_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
          (frame_length > imgsensor_info.hs_video.framelength)
        ? (frame_length - imgsensor_info.hs_video.  framelength) : 0;

        imgsensor.frame_length =
            imgsensor_info.hs_video.framelength + imgsensor.dummy_line;

        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        else {
            /*No need to set*/
            pr_debug("frame_length %d < shutter %d",
                imgsensor.frame_length, imgsensor.shutter);
        }
        break;
    case MSDK_SCENARIO_ID_SLIM_VIDEO:
        frame_length = imgsensor_info.slim_video.pclk
            / framerate * 10 / imgsensor_info.slim_video.linelength;

        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
          (frame_length > imgsensor_info.slim_video.framelength)
        ? (frame_length - imgsensor_info.slim_video.  framelength) : 0;

        imgsensor.frame_length =
          imgsensor_info.slim_video.framelength + imgsensor.dummy_line;

        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        else {
            /*No need to set*/
            pr_debug("frame_length %d < shutter %d",
                imgsensor.frame_length, imgsensor.shutter);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM1:
        frame_length = imgsensor_info.custom1.pclk
            / framerate * 10 / imgsensor_info.custom1.linelength;

        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
          (frame_length > imgsensor_info.custom1.framelength)
        ? (frame_length - imgsensor_info.custom1.  framelength) : 0;

        imgsensor.frame_length =
          imgsensor_info.custom1.framelength + imgsensor.dummy_line;

        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        else {
            /*No need to set*/
            pr_debug("frame_length %d < shutter %d",
                imgsensor.frame_length, imgsensor.shutter);
        }
        break;

    case MSDK_SCENARIO_ID_CUSTOM2:
        frame_length = imgsensor_info.custom2.pclk
            / framerate * 10 / imgsensor_info.custom2.linelength;

        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
          (frame_length > imgsensor_info.custom2.framelength)
        ? (frame_length - imgsensor_info.custom2.  framelength) : 0;

        imgsensor.frame_length =
          imgsensor_info.custom2.framelength + imgsensor.dummy_line;

        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        else {
            /*No need to set*/
            pr_debug("frame_length %d < shutter %d",
                imgsensor.frame_length, imgsensor.shutter);
        }
        break;

    case MSDK_SCENARIO_ID_CUSTOM3:
        frame_length = imgsensor_info.custom3.pclk
            / framerate * 10 / imgsensor_info.custom3.linelength;

        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
          (frame_length > imgsensor_info.custom3.framelength)
        ? (frame_length - imgsensor_info.custom3.  framelength) : 0;

        imgsensor.frame_length =
          imgsensor_info.custom3.framelength + imgsensor.dummy_line;

        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        else {
            /*No need to set*/
            pr_debug("frame_length %d < shutter %d",
                imgsensor.frame_length, imgsensor.shutter);
        }
        break;

    case MSDK_SCENARIO_ID_CUSTOM4:
        frame_length = imgsensor_info.custom4.pclk
            / framerate * 10 / imgsensor_info.custom4.linelength;

        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
          (frame_length > imgsensor_info.custom4.framelength)
        ? (frame_length - imgsensor_info.custom4.  framelength) : 0;

        imgsensor.frame_length =
          imgsensor_info.custom4.framelength + imgsensor.dummy_line;

        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        else {
            /*No need to set*/
            pr_debug("frame_length %d < shutter %d",
                imgsensor.frame_length, imgsensor.shutter);
        }
        break;

    case MSDK_SCENARIO_ID_CUSTOM5:
        frame_length = imgsensor_info.custom5.pclk
            / framerate * 10 / imgsensor_info.custom5.linelength;

        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
          (frame_length > imgsensor_info.custom5.framelength)
        ? (frame_length - imgsensor_info.custom5.  framelength) : 0;

        imgsensor.frame_length =
          imgsensor_info.custom5.framelength + imgsensor.dummy_line;

        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        else {
            /*No need to set*/
            pr_debug("frame_length %d < shutter %d",
                imgsensor.frame_length, imgsensor.shutter);
        }
        break;

    default:        /* coding with  preview scenario by default */
        frame_length = imgsensor_info.pre.pclk
            / framerate * 10 / imgsensor_info.pre.linelength;

        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.pre.framelength)
            ? (frame_length - imgsensor_info.pre.framelength) : 0;

        imgsensor.frame_length =
            imgsensor_info.pre.framelength + imgsensor.dummy_line;

        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        else {
            /*No need to set*/
            pr_debug("frame_length %d < shutter %d",
                imgsensor.frame_length, imgsensor.shutter);
        }
        pr_debug("error scenario_id = %d, we use preview scenario\n",
        scenario_id);
        break;
    }
    return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(
enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    /*pr_debug("scenario_id = %d\n", scenario_id);*/

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
    default:
        break;
    }

    return ERROR_NONE;
}

static kal_uint32 jn1_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
    UINT32 rgain_32, grgain_32, gbgain_32, bgain_32;

    pr_debug("%s\n", __func__);

    grgain_32 = (pSetSensorAWB->ABS_GAIN_GR << 8) >> 9;
    rgain_32 = (pSetSensorAWB->ABS_GAIN_R << 8) >> 9;
    bgain_32 = (pSetSensorAWB->ABS_GAIN_B << 8) >> 9;
    gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB << 8) >> 9;

    pr_debug("[%s] ABS_GAIN_GR:%d, grgain_32:%d\n",
        __func__,
        pSetSensorAWB->ABS_GAIN_GR, grgain_32);
    pr_debug("[%s] ABS_GAIN_R:%d, rgain_32:%d\n",
        __func__,
        pSetSensorAWB->ABS_GAIN_R, rgain_32);
    pr_debug("[%s] ABS_GAIN_B:%d, bgain_32:%d\n",
        __func__,
        pSetSensorAWB->ABS_GAIN_B, bgain_32);
    pr_debug("[%s] ABS_GAIN_GB:%d, gbgain_32:%d\n",
        __func__,
        pSetSensorAWB->ABS_GAIN_GB, gbgain_32);

    write_cmos_sensor(0x6028, 0x4000);

    write_cmos_sensor(0x0d82, (pSetSensorAWB->ABS_GAIN_R >> 1));

    write_cmos_sensor(0x0d84, (pSetSensorAWB->ABS_GAIN_GR >> 1));

    write_cmos_sensor(0x0d86, (pSetSensorAWB->ABS_GAIN_B >> 1));

    return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    pr_debug("enable: %d\n", enable);

    if (enable) {
/* 0 : Normal, 1 : Solid Color, 2 : Color Bar, 3 : Shade Color Bar, 4 : PN9 */
        write_cmos_sensor(0x6028, 0x4000);
        write_cmos_sensor(0x0600, 0x0001);
        write_cmos_sensor(0x0602, 0x0000);
        write_cmos_sensor(0x0604, 0x0000);
        write_cmos_sensor(0x0606, 0x0000);
        write_cmos_sensor(0x0608, 0x0000);
    } else {
        write_cmos_sensor(0x0600, 0x0000); /*No pattern*/
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}
static kal_uint32 get_sensor_temperature(void)
{
    UINT8 temperature;
    INT32 temperature_convert;

    temperature = read_cmos_sensor_8(0x013a);

    if (temperature >= 0x0 && temperature <= 0x78)
        temperature_convert = temperature;
    else
        temperature_convert = -1;

    /*pr_info("temp_c(%d), read_reg(%d), enable %d\n",
     *  temperature_convert, temperature, read_cmos_sensor_8(0x0138));
     */

    return temperature_convert;
}
#define FOUR_CELL_XTALK_CAL_SIZE 2612
#define FOUR_CELL_XTALK_CAL_ADDR 0x1AA0
static char FOUR_CELL_XTALK_CAL[FOUR_CELL_XTALK_CAL_SIZE + 2];

#define FOUR_CELL_SENSOR_XTALK_CAL_SIZE  768
#define FOUR_CELL_SENSOR_XTALK_CAL_ADDR  0x2500
static char FOUR_CELL_SENSOR_XTALK_CAL[FOUR_CELL_SENSOR_XTALK_CAL_SIZE + 2];

#define FOUR_CELL_PD_XTALK_CAL_SIZE 4000
#define FOUR_CELL_PD_XTALK_CAL_ADDR 0x2810
static char FOUR_CELL_PD_XTALK_CAL[FOUR_CELL_PD_XTALK_CAL_SIZE + 2];

#define FOUR_CELL_SW_GGC_CAL_SIZE 626
#define FOUR_CELL_SW_GGC_CAL_ADDR 0x3860
static char FOUR_CELL_SW_GGC_CAL[FOUR_CELL_SW_GGC_CAL_SIZE + 2];

#define FOUR_CELL_REMOSAIC_CAL_SIZE (FOUR_CELL_XTALK_CAL_SIZE + FOUR_CELL_SENSOR_XTALK_CAL_SIZE + FOUR_CELL_PD_XTALK_CAL_SIZE + FOUR_CELL_SW_GGC_CAL_SIZE)

static void read_4cell_from_eeprom(char* src, int addr, int len)
{
    char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };
    if (src[0] == 0 && src[1] == 0) {
        src[0] = (len & 0xff);
        src[1] = ((len >> 8) & 0xff);
        iReadRegI2C(pu_send_cmd, 2, &src[2], len, 0xA1);
    }
}
#if 0
#define FOUR_CELL_SIZE 3072//size = 3072 = 0xc00
static int Is_Read_4Cell;
static char Four_Cell_Array[FOUR_CELL_SIZE + 2];

static void read_4cell_from_eeprom(char *data)
{
    int ret;
    int addr = 0x763;/*Start of 4 cell data*/
    char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

    pu_send_cmd[0] = (char)(addr >> 8);
    pu_send_cmd[1] = (char)(addr & 0xFF);

    /* Check I2C is normal */
    ret = iReadRegI2C(pu_send_cmd, 2, data, 1, EEPROM_READ_ID);
    if (ret != 0) {
        pr_debug("iReadRegI2C error");
        return;
    }

    if (Is_Read_4Cell != 1) {
        pr_debug("Need to read i2C");

        Four_Cell_Array[0] = (FOUR_CELL_SIZE & 0xff);/*Low*/
        Four_Cell_Array[1] = ((FOUR_CELL_SIZE >> 8) & 0xff);/*High*/

        /*Multi-Read*/
        iReadRegI2C(pu_send_cmd, 2, &Four_Cell_Array[2],
                    FOUR_CELL_SIZE, EEPROM_READ_ID);
        Is_Read_4Cell = 1;
    }

    memcpy(data, Four_Cell_Array, FOUR_CELL_SIZE);
}
#endif
#if 0
static char *features[300] = {
    "SENSOR_FEATURE_BEGIN,                              ",
    "SENSOR_FEATURE_GET_RESOLUTION,                     ",
    "SENSOR_FEATURE_GET_PERIOD,                         ",
    "SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ,               ",
    "SENSOR_FEATURE_SET_ESHUTTER,                       ",
    "SENSOR_FEATURE_SET_NIGHTMODE,                      ",
    "SENSOR_FEATURE_SET_GAIN,                           ",
    "SENSOR_FEATURE_SET_DUAL_GAIN,                      ",
    "SENSOR_FEATURE_SET_GAIN_AND_ESHUTTER,              ",
    "SENSOR_FEATURE_SET_FLASHLIGHT,                     ",
    "SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ,          ",
    "SENSOR_FEATURE_SET_REGISTER,                       ",
    "SENSOR_FEATURE_GET_REGISTER,                       ",
    "SENSOR_FEATURE_SET_CCT_REGISTER,                   ",
    "SENSOR_FEATURE_GET_CCT_REGISTER,                   ",
    "SENSOR_FEATURE_SET_ENG_REGISTER,                   ",
    "SENSOR_FEATURE_GET_ENG_REGISTER,                   ",
    "SENSOR_FEATURE_GET_REGISTER_DEFAULT,               ",
    "SENSOR_FEATURE_GET_CONFIG_PARA,                    ",
    "SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR,              ",
    "SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA,              ",
    "SENSOR_FEATURE_GET_GROUP_COUNT,                    ",
    "SENSOR_FEATURE_GET_GROUP_INFO,                     ",
    "SENSOR_FEATURE_GET_ITEM_INFO,                      ",
    "SENSOR_FEATURE_SET_ITEM_INFO,                      ",
    "SENSOR_FEATURE_GET_ENG_INFO,                       ",
    "SENSOR_FEATURE_GET_LENS_DRIVER_ID,                 ",
    "SENSOR_FEATURE_SET_YUV_CMD,                        ",
    "SENSOR_FEATURE_SET_VIDEO_MODE,                     ",
    "SENSOR_FEATURE_SET_TARGET_FRAME_RATE,              ",
    "SENSOR_FEATURE_SET_CALIBRATION_DATA,               ",
    "SENSOR_FEATURE_SET_SENSOR_SYNC,                    ",
    "SENSOR_FEATURE_INITIALIZE_AF,                      ",
    "SENSOR_FEATURE_CONSTANT_AF,                        ",
    "SENSOR_FEATURE_INFINITY_AF,                        ",
    "SENSOR_FEATURE_MOVE_FOCUS_LENS,                    ",
    "SENSOR_FEATURE_GET_AF_STATUS,                      ",
    "SENSOR_FEATURE_GET_AE_STATUS,                      ",
    "SENSOR_FEATURE_GET_AWB_STATUS,                     ",
    "SENSOR_FEATURE_GET_AF_INF,                         ",
    "SENSOR_FEATURE_GET_AF_MACRO,                       ",
    "SENSOR_FEATURE_CHECK_SENSOR_ID,                    ",
    "SENSOR_FEATURE_SET_AUTO_FLICKER_MODE,              ",
    "SENSOR_FEATURE_SET_TEST_PATTERN,                   ",
    "SENSOR_FEATURE_SET_SOFTWARE_PWDN,                  ",
    "SENSOR_FEATURE_SINGLE_FOCUS_MODE,                  ",
    "SENSOR_FEATURE_CANCEL_AF,                          ",
    "SENSOR_FEATURE_SET_AF_WINDOW,                      ",
    "SENSOR_FEATURE_GET_EV_AWB_REF,                     ",
    "SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN,          ",
    "SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS,         ",
    "SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS,      ",
    "SENSOR_FEATURE_SET_AE_WINDOW,                      ",
    "SENSOR_FEATURE_GET_EXIF_INFO,                      ",
    "SENSOR_FEATURE_GET_DELAY_INFO,                     ",
    "SENSOR_FEATURE_SET_SLAVE_I2C_ID,                   ",
    "SENSOR_FEATURE_SUSPEND,                            ",
    "SENSOR_FEATURE_RESUME,                             ",
    "SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO,     ",
    "SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO, ",
    "SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO,               ",
    "SENSOR_FEATURE_AUTOTEST_CMD,                       ",
    "SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE,    ",
    "SENSOR_FEATURE_GET_TEMPERATURE_VALUE,              ",
    "SENSOR_FEATURE_GET_SENSOR_CURRENT_TEMPERATURE,     ",
    "SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO,             ",
    "SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO,        ",
    "SENSOR_FEATURE_SET_YUV_3A_CMD,                     ",
    "SENSOR_FEATURE_SET_N3D_I2C_STREAM_REGDATA,         ",
    "SENSOR_FEATURE_SET_N3D_STOP_STREAMING,             ",
    "SENSOR_FEATURE_SET_N3D_START_STREAMING,            ",
    "SENSOR_FEATURE_GET_SENSOR_N3D_STREAM_TO_VSYNC_TIME,",
    "SENSOR_FEATURE_SET_ESHUTTER_GAIN,                  ",
    "SENSOR_FEATURE_SET_OB_LOCK,                        ",
    "SENSOR_FEATURE_SET_SENSOR_OTP_AWB_CMD,             ",
    "SENSOR_FEATURE_SET_SENSOR_OTP_LSC_CMD,             ",
    "SENSOR_FEATURE_GET_YUV_CAPTURE_OUTPUT_JPEG,        ",
    "SENSOR_FEATURE_SET_YUV_JPEG_PARA,                  ",
    "SENSOR_FEATURE_GET_YUV_JPEG_INFO,                  ",
    "SENSOR_FEATURE_SET_FRAMERATE,                      ",
    "SENSOR_FEATURE_SET_HDR,                            ",
    "SENSOR_FEATURE_GET_CROP_INFO,                      ",
    "SENSOR_FEATURE_GET_VC_INFO,                        ",
    "SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN,              ",
    "SENSOR_FEATURE_SET_AWB_GAIN,                       ",
    "SENSOR_FEATURE_SET_MIN_MAX_FPS,                    ",
    "SENSOR_FEATURE_GET_PDAF_INFO,                      ",
    "SENSOR_FEATURE_GET_PDAF_DATA,                      ",
    "SENSOR_FEATURE_SET_PDFOCUS_AREA,                   ",
    "SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY,           ",
    "SENSOR_FEATURE_DEBUG_IMGSENSOR,                    ",
    "SENSOR_FEATURE_SET_HDR_SHUTTER,                    ",
    "SENSOR_FEATURE_SET_ISO,                            ",
    "SENSOR_FEATURE_SET_PDAF,                           ",
    "SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME,             ",
    "SENSOR_FEATURE_SET_SHUTTER_BUF_MODE,               ",
    "SENSOR_FEATURE_SET_GAIN_BUF_MODE,                  ",
    "SENSOR_FEATURE_SET_I2C_BUF_MODE_EN,                ",
    "SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY,            ",
    "SENSOR_FEATURE_GET_PDAF_TYPE,                      ",
    "SENSOR_FEATURE_SET_PDAF_TYPE,                      ",
    "SENSOR_FEATURE_GET_PDAF_REG_SETTING,               ",
    "SENSOR_FEATURE_SET_PDAF_REG_SETTING,               ",
    "SENSOR_FEATURE_SET_STREAMING_SUSPEND,              ",
    "SENSOR_FEATURE_SET_STREAMING_RESUME,               ",
    "SENSOR_FEATURE_OPEN,                               ",
    "SENSOR_FEATURE_CLOSE,                              ",
    "SENSOR_FEATURE_SET_DRIVER,                         ",
    "SENSOR_FEATURE_CHECK_IS_ALIVE,                     ",
    "SENSOR_FEATURE_GET_4CELL_DATA,                     ",
    "SENSOR_FEATURE_SET_WAKE_LOCK,                      ",
    "SENSOR_FEATURE_GET_MIPI_PIXEL_RATE,                ",
    "SENSOR_FEATURE_SET_HDR_ATR,                        ",
    "SENSOR_FEATURE_SET_HDR_TRI_GAIN,                   ",
    "SENSOR_FEATURE_SET_HDR_TRI_SHUTTER,                ",
    "SENSOR_FEATURE_SET_LSC_TBL,                        ",
    "SENSOR_FEATURE_GET_SENSOR_SYNC_MODE_CAPACITY,      ",
    "SENSOR_FEATURE_GET_SENSOR_SYNC_MODE,               ",
    "SENSOR_FEATURE_SET_SENSOR_SYNC_MODE,               ",
    "SENSOR_FEATURE_GET_PIXEL_RATE,                     ",
    "SENSOR_FEATURE_MAX                 "
};
#endif

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
UINT8 *feature_para, UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
    UINT16 *feature_data_16 = (UINT16 *) feature_para;
    UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
    UINT32 *feature_data_32 = (UINT32 *) feature_para;
    INT32 *feature_return_para_i32 = (INT32 *) feature_para;
    unsigned long long *feature_data = (unsigned long long *)feature_para;

    struct SET_PD_BLOCK_INFO_T *PDAFinfo;
    struct SENSOR_VC_INFO_STRUCT *pvcinfo;
    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    struct SET_SENSOR_AWB_GAIN *pSetSensorAWB =
        (struct SET_SENSOR_AWB_GAIN *) feature_para;

    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
        (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    switch (feature_id) {
    case SENSOR_FEATURE_GET_BINNING_TYPE:
        switch (*(feature_data + 1)) {
        case MSDK_SCENARIO_ID_CUSTOM2:
        case MSDK_SCENARIO_ID_CUSTOM4:
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *feature_return_para_32 = 4; /* match analog_gain ratio */
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *feature_return_para_32 = 1; /*BINNING_AVERAGED*/
            break;
        }
        pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
            *feature_return_para_32);
        *feature_para_len = 4;
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
            *(feature_data + 2) = 2;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
        default:
            *(feature_data + 2) = 1;
            break;
        }
        break;
    case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
        *(feature_data + 1) = 1;
        *(feature_data + 2) = imgsensor_info.margin;
        break;
    case SENSOR_FEATURE_GET_PERIOD:
        *feature_return_para_16++ = imgsensor.line_length;
        *feature_return_para_16 = imgsensor.frame_length;
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
#if 0
        pr_debug(
            "feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
            imgsensor.pclk, imgsensor.current_fps);
#endif
        *feature_return_para_32 = imgsensor.pclk;
        *feature_para_len = 4;
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
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.pre.framelength << 16)
                + imgsensor_info.pre.linelength;
            break;
        }
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
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = imgsensor_info.pre.pclk;
            break;
        }
        break;

    case SENSOR_FEATURE_SET_ESHUTTER:
        set_shutter(*feature_data);
        break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
    /* night_mode((BOOL) *feature_data); no need to implement this mode */
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
        /* get the lens driver ID from EEPROM or
         * just return LENS_DRIVER_ID_DO_NOT_CARE
         */
        /* if EEPROM does not exist in camera module. */
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
        set_auto_flicker_mode((BOOL) (*feature_data_16),
                    *(feature_data_16 + 1));
        break;
    case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
        set_max_framerate_by_scenario(
        (enum MSDK_SCENARIO_ID_ENUM) *feature_data, *(feature_data + 1));
        break;

    case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
        get_default_framerate_by_scenario(
            (enum MSDK_SCENARIO_ID_ENUM) *(feature_data),
              (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
        break;
    case SENSOR_FEATURE_SET_TEST_PATTERN:
        set_test_pattern_mode((BOOL) (*feature_data));
        break;

    /* for factory mode auto testing */
    case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
        *feature_return_para_32 = imgsensor_info.checksum_value;
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_SET_FRAMERATE:
        pr_debug("current fps :%d\n", *feature_data_32);
        spin_lock(&imgsensor_drv_lock);
        imgsensor.current_fps = (UINT16)*feature_data_32;
        spin_unlock(&imgsensor_drv_lock);
        break;
    case SENSOR_FEATURE_SET_HDR:
        pr_debug("ihdr enable :%d\n", *feature_data_32);
        spin_lock(&imgsensor_drv_lock);
        imgsensor.hdr_mode = (UINT8)*feature_data_32;
        // imgsensor.hdr_mode = 9;
        // force set hdr_mode to zHDR
        spin_unlock(&imgsensor_drv_lock);
        break;

    case SENSOR_FEATURE_GET_CROP_INFO:
        /* pr_debug("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
         *  (UINT32) *feature_data);
         */

        wininfo =
    (struct SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

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
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[0],
                   sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        }
        break;
    case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
        pr_debug("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
            (UINT16) *feature_data,
            (UINT16) *(feature_data + 1),
            (UINT16) *(feature_data + 2));

/* ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),
 * (UINT16)*(feature_data+2));
 */
        break;
    case SENSOR_FEATURE_SET_AWB_GAIN:
        jn1_awb_gain(pSetSensorAWB);
        break;
    case SENSOR_FEATURE_SET_HDR_SHUTTER:
        pr_debug("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
            (UINT16) *feature_data,
            (UINT16) *(feature_data + 1));
/* ihdr_write_shutter((UINT16)*feature_data,(UINT16)*(feature_data+1)); */
        break;

    case SENSOR_FEATURE_GET_4CELL_DATA:/*get 4 cell data from eeprom*/
    {

        int type = (kal_uint16)(*feature_data);
        char *data = (char *)(uintptr_t)(*(feature_data+1));

        if (type == FOUR_CELL_CAL_TYPE_GAIN_TBL) {
            pr_debug("Read remosaic Start");
            read_4cell_from_eeprom(FOUR_CELL_XTALK_CAL,
                FOUR_CELL_XTALK_CAL_ADDR, FOUR_CELL_XTALK_CAL_SIZE);
            /* for dump otp data
            for(i=0; i<FOUR_CELL_XTALK_CAL_SIZE; i++){
                printk("FOUR_CELL_CAL_TYPE_GAIN_TBL Read FOUR_CELL_XTALK_CAL[%d] = %02x \n", i, FOUR_CELL_XTALK_CAL[i] );
            }
            */
            read_4cell_from_eeprom(FOUR_CELL_SENSOR_XTALK_CAL,
                FOUR_CELL_SENSOR_XTALK_CAL_ADDR, FOUR_CELL_SENSOR_XTALK_CAL_SIZE);
            read_4cell_from_eeprom(FOUR_CELL_PD_XTALK_CAL,
                FOUR_CELL_PD_XTALK_CAL_ADDR, FOUR_CELL_PD_XTALK_CAL_SIZE);
            read_4cell_from_eeprom(FOUR_CELL_SW_GGC_CAL,
                FOUR_CELL_SW_GGC_CAL_ADDR, FOUR_CELL_SW_GGC_CAL_SIZE);

            memcpy(data, FOUR_CELL_XTALK_CAL, FOUR_CELL_XTALK_CAL_SIZE+2);
            memcpy(data+(2 + FOUR_CELL_XTALK_CAL_SIZE), FOUR_CELL_SENSOR_XTALK_CAL+2, FOUR_CELL_SENSOR_XTALK_CAL_SIZE);
            memcpy(data+(2 + FOUR_CELL_XTALK_CAL_SIZE + FOUR_CELL_SENSOR_XTALK_CAL_SIZE), FOUR_CELL_PD_XTALK_CAL+2, FOUR_CELL_PD_XTALK_CAL_SIZE);
            memcpy(data+(2 + FOUR_CELL_XTALK_CAL_SIZE + FOUR_CELL_SENSOR_XTALK_CAL_SIZE + FOUR_CELL_PD_XTALK_CAL_SIZE), FOUR_CELL_SW_GGC_CAL+2, FOUR_CELL_SW_GGC_CAL_SIZE);

            /* for dump otp data
            for(i=0; i<FOUR_CELL_REMOSAIC_CAL_SIZE + 2; i++){
                printk("Read remosaic TYPE_GAIN_TBL data[%d] = %02x \n", i, (UINT16)data[i]);
            }
            */
            pr_debug("Read remosaic = %02x %02x %02x %02x %02x %02x\n",
                (UINT16)data[0], (UINT16)data[1],
                (UINT16)data[2], (UINT16)data[3],
                (UINT16)data[4], (UINT16)data[5]);
        }
        if (type == FOUR_CELL_CAL_TYPE_ALL) {
            pr_debug("Read remosaic Start");
            read_4cell_from_eeprom(FOUR_CELL_XTALK_CAL,
                FOUR_CELL_XTALK_CAL_ADDR, FOUR_CELL_XTALK_CAL_SIZE);
            /* for dump otp data
            for(i=0; i<FOUR_CELL_XTALK_CAL_SIZE; i++){
                printk("FOUR_CELL_CAL_TYPE_ALL Read FOUR_CELL_XTALK_CAL[%d] = %02x \n", i, FOUR_CELL_XTALK_CAL[i] );
            }
            */
            read_4cell_from_eeprom(FOUR_CELL_SENSOR_XTALK_CAL,
                FOUR_CELL_SENSOR_XTALK_CAL_ADDR, FOUR_CELL_SENSOR_XTALK_CAL_SIZE);
            read_4cell_from_eeprom(FOUR_CELL_PD_XTALK_CAL,
                FOUR_CELL_PD_XTALK_CAL_ADDR, FOUR_CELL_PD_XTALK_CAL_SIZE);
            read_4cell_from_eeprom(FOUR_CELL_SW_GGC_CAL,
                FOUR_CELL_SW_GGC_CAL_ADDR, FOUR_CELL_SW_GGC_CAL_SIZE);

            memcpy(data, FOUR_CELL_XTALK_CAL, FOUR_CELL_XTALK_CAL_SIZE+2);
            memcpy(data+(2 + FOUR_CELL_XTALK_CAL_SIZE), FOUR_CELL_SENSOR_XTALK_CAL+2, FOUR_CELL_SENSOR_XTALK_CAL_SIZE);
            memcpy(data+(2 + FOUR_CELL_XTALK_CAL_SIZE + FOUR_CELL_SENSOR_XTALK_CAL_SIZE), FOUR_CELL_PD_XTALK_CAL+2, FOUR_CELL_PD_XTALK_CAL_SIZE);
            memcpy(data+(2 + FOUR_CELL_XTALK_CAL_SIZE + FOUR_CELL_SENSOR_XTALK_CAL_SIZE + FOUR_CELL_PD_XTALK_CAL_SIZE), FOUR_CELL_SW_GGC_CAL+2, FOUR_CELL_SW_GGC_CAL_SIZE);

            /* for dump otp data
            for(i=0; i<FOUR_CELL_REMOSAIC_CAL_SIZE + 2; i++){
                printk("Read remosaic type-all data[%d] = %02x \n", i, (UINT16)data[i]);
            }
            */
            pr_debug("Read remosaic = %02x %02x %02x %02x %02x %02x\n",
                (UINT16)data[0], (UINT16)data[1],
                (UINT16)data[2], (UINT16)data[3],
                (UINT16)data[4], (UINT16)data[5]);
        }

        break;
    }

    case SENSOR_FEATURE_GET_PDAF_INFO:
        pr_debug("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
            (UINT16)*feature_data);
        PDAFinfo =
            (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data + 1));
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CUSTOM4://1080P 60fps
            memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info_1920_1080,sizeof(struct SET_PD_BLOCK_INFO_T));
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW: //2x2 binning normal capture
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CUSTOM1:
            memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            memcpy((void *)PDAFinfo,(void *)&imgsensor_custom5_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW://1080P 30fps
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            memcpy((void *)PDAFinfo,(void *)&imgsensor_normal_video_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO: //full
        default:
            break;
        }
        break;

    case SENSOR_FEATURE_GET_VC_INFO:
        pr_debug("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
        pvcinfo = (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
        switch (*feature_data_32) {
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0], sizeof(struct SENSOR_VC_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1], sizeof(struct SENSOR_VC_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2], sizeof(struct SENSOR_VC_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[3], sizeof(struct SENSOR_VC_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[4], sizeof(struct SENSOR_VC_INFO_STRUCT));
            break;
        default:
            memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0], sizeof(struct SENSOR_VC_INFO_STRUCT));
            break;
        }
        break;
    case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
        pr_debug("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", (UINT16)*feature_data);
        //PDAF capacity enable or not
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            //need to check
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;
        }
        break;
    case SENSOR_FEATURE_SET_PDAF:
        pr_debug("PDAF mode :%d\n", *feature_data_16);
        imgsensor.pdaf_mode = *feature_data_16;
        break;
         /*
         * case SENSOR_FEATURE_GET_PDAF_DATA: //get cal data from eeprom
         * pr_debug("SENSOR_FEATURE_GET_PDAF_DATA\n");
         * read_2T7_eeprom((kal_uint16 )(*feature_data),
         * (char*)(uintptr_t)(*(feature_data+1)),
         * (kal_uint32)(*(feature_data+2)));
         * pr_debug("SENSOR_FEATURE_GET_PDAF_DATA success\n");
         * break;
         */
        /******************** PDAF END   <<< *********/
    case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
        set_shutter_frame_length((UINT16)(*feature_data),
                        (UINT16)(*(feature_data + 1)),(BOOL)(*(feature_data + 2)));
        break;
    case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
        pr_debug("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
        streaming_control(KAL_FALSE);
        break;
    case SENSOR_FEATURE_SET_STREAMING_RESUME:
        pr_debug("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
            *feature_data);
        if (*feature_data != 0)
            set_shutter(*feature_data);
        streaming_control(KAL_TRUE);
        break;
    case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
        *feature_return_para_i32 = get_sensor_temperature();
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_GET_PIXEL_RATE:
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
            (imgsensor_info.cap.pclk /
            (imgsensor_info.cap.linelength - 80))*
            imgsensor_info.cap.grabwindow_width;

            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
            (imgsensor_info.normal_video.pclk /
            (imgsensor_info.normal_video.linelength - 80))*
            imgsensor_info.normal_video.grabwindow_width;

            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
            (imgsensor_info.hs_video.pclk /
            (imgsensor_info.hs_video.linelength - 80))*
            imgsensor_info.hs_video.grabwindow_width;

            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
            (imgsensor_info.slim_video.pclk /
            (imgsensor_info.slim_video.linelength - 80))*
            imgsensor_info.slim_video.grabwindow_width;

            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
            (imgsensor_info.custom1.pclk /
            (imgsensor_info.custom1.linelength - 80))*
            imgsensor_info.custom1.grabwindow_width;

            break;

        case MSDK_SCENARIO_ID_CUSTOM2:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
            (imgsensor_info.custom2.pclk /
            (imgsensor_info.custom2.linelength - 80))*
            imgsensor_info.custom2.grabwindow_width;

            break;

        case MSDK_SCENARIO_ID_CUSTOM3:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
            (imgsensor_info.custom3.pclk /
            (imgsensor_info.custom3.linelength - 80))*
            imgsensor_info.custom3.grabwindow_width;

            break;

        case MSDK_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
            (imgsensor_info.custom4.pclk /
            (imgsensor_info.custom4.linelength - 80))*
            imgsensor_info.custom4.grabwindow_width;

            break;

        case MSDK_SCENARIO_ID_CUSTOM5:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
            (imgsensor_info.custom5.pclk /
            (imgsensor_info.custom5.linelength - 80))*
            imgsensor_info.custom5.grabwindow_width;

            break;

        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
            (imgsensor_info.pre.pclk /
            (imgsensor_info.pre.linelength - 80))*
            imgsensor_info.pre.grabwindow_width;
            break;
        }
        break;
    case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
        LOG_INF("SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE\n");
        memcpy(feature_return_para_32,
        &imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
        break;
    case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
        LOG_INF("SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE\n");
        *feature_return_para_32 =  imgsensor.current_ae_effective_frame;
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
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                imgsensor_info.pre.mipi_pixel_rate;
            break;
        }
        break;
//#ifdef OPLUS_FEATURE_CAMERA_COMMON
    case SENSOR_FEATURE_GET_MODULE_INFO:
        break;
    case SENSOR_FEATURE_GET_MODULE_SN:
        break;
    case SENSOR_FEATURE_SET_SENSOR_OTP:
        break;
    case SENSOR_FEATURE_CHECK_MODULE_ID:
        break;
    case SENSOR_FEATURE_GET_EEPROM_COMDATA:
        memcpy(feature_return_para_32, s5kjn1_23707_common_data,
                CAMERA_EEPPROM_COMDATA_LENGTH);
        *feature_para_len = CAMERA_EEPPROM_COMDATA_LENGTH;
        break;
    case SENSOR_FEATURE_GET_EEPROM_STEREODATA:
        break;
    case SENSOR_FEATURE_GET_DISTORTIONPARAMS:
        break;
//#endif /* OPLUS_FEATURE_CAMERA_COMMON */
    default:
        break;
    }

    return ERROR_NONE;
}               /*      feature_control()  */


static struct SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 S5KJN1_MIPI_RAW23707_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc != NULL)
        *pfFunc = &sensor_func;
    return ERROR_NONE;
}



