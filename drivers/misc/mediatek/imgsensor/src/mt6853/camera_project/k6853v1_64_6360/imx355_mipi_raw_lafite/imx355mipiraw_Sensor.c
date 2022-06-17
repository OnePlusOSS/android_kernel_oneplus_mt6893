/*
 * Copyright (C) 2018 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
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

#include "imx355mipiraw_Sensor.h"
#include "imgsensor_common.h"

#define PFX "IMX355_camera_sensor"
#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)
#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif
#define SENSOR_FUSION_THRESHOLD -16888000
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = IMX355_SENSOR_ID_LAFITE,
    .checksum_value = 0x8ac2d94a,

    .pre = {
            .pclk = 280800000,
            .linelength = 3672,
            .framelength = 2548,
            .startx = 0,
            .starty = 0,
            .grabwindow_width = 3264,
            .grabwindow_height = 2448,
            .mipi_pixel_rate = 280800000,
            .mipi_data_lp2hs_settle_dc = 85,
            .max_framerate = 300,
    },
    .cap = {
            .pclk = 280800000,
            .linelength = 3672,
            .framelength = 2548,
            .startx = 0,
            .starty = 0,
            .grabwindow_width = 3264,
            .grabwindow_height = 2448,
            .mipi_pixel_rate = 280800000,
            .mipi_data_lp2hs_settle_dc = 85,
            .max_framerate = 300,
    },
    .normal_video = {
            .pclk = 280800000,
            .linelength = 3672,
            .framelength = 2548,
            .startx = 0,
            .starty = 0,
            .grabwindow_width = 3264,
            .grabwindow_height = 2448,
            .mipi_pixel_rate = 280800000,
            .mipi_data_lp2hs_settle_dc = 85,
            .max_framerate = 300,
    },
    .hs_video = {
            .pclk = 288000000,
            .linelength = 1836,
            .framelength = 870,
            .startx = 0,
            .starty = 0,
            .grabwindow_width = 1408,
            .grabwindow_height = 792,
            .mipi_pixel_rate = 288000000,
            .mipi_data_lp2hs_settle_dc = 85,
            .max_framerate = 1800,
    },
    .slim_video = {
            .pclk = 280800000,
            .linelength = 3672,
            .framelength = 2548 ,
            .startx = 0,
            .starty = 0,
            .grabwindow_width = 3264,
            .grabwindow_height = 2448,
            .mipi_pixel_rate = 280800000,
            .mipi_data_lp2hs_settle_dc = 85,
            .max_framerate = 300,
    },
    .custom1 = {
           .pclk = 225600000,
           .linelength = 3672,
           .framelength = 2558,
           .startx = 0,
           .starty = 0,
           .grabwindow_width = 3264,
           .grabwindow_height = 2448,
           .mipi_pixel_rate = 225600000,
           .mipi_data_lp2hs_settle_dc = 85,
           .max_framerate = 240,
    },
    .custom2 = {
           .pclk = 144000000,
           .linelength = 1836,
           .framelength = 2614,
           .startx = 0,
           .starty = 0,
           .grabwindow_width = 1640,
           .grabwindow_height = 1232,
           .mipi_pixel_rate = 144000000,
           .mipi_data_lp2hs_settle_dc = 85,
           .max_framerate = 300,
     },
	 .custom3 = {
           .pclk = 280800000,
           .linelength = 3672,
           .framelength = 2548 ,
           .startx = 0,
           .starty = 0,
           .grabwindow_width = 3264,
           .grabwindow_height = 2448,
           .mipi_pixel_rate = 280800000,
           .mipi_data_lp2hs_settle_dc = 85,
           .max_framerate = 300,
    },
    .custom4 = {
           .pclk = 211200000,
           .linelength = 3672,
           .framelength = 1916,
           .startx = 0,
           .starty = 0,
           .grabwindow_width = 3264,
           .grabwindow_height = 1840,
           .mipi_pixel_rate = 211200000,
           .mipi_data_lp2hs_settle_dc = 85,
           .max_framerate = 300,
     },
	.custom5 = {
           .pclk = 144000000,
           .linelength = 1836,
           .framelength = 2614,
           .startx = 0,
           .starty = 0,
           .grabwindow_width = 1640,
           .grabwindow_height = 1232,
           .mipi_pixel_rate = 144000000,
           .mipi_data_lp2hs_settle_dc = 85,
           .max_framerate = 300,
     },

      .margin = 10,
      .min_shutter = 1,
      .min_gain = 64, /*1x gain*/
      .max_gain = 1024, /*16x gain*/
      .min_gain_iso = 100,
      .gain_step = 1,
      .gain_type = 0,
      .max_frame_length = 0xffff,
      .ae_shut_delay_frame = 0,
      .ae_sensor_gain_delay_frame = 0,
      .ae_ispGain_delay_frame = 2,
      .ihdr_support = 0,
      .ihdr_le_firstline = 0,
      .sensor_mode_num = 10,
      .cap_delay_frame = 2,
      .pre_delay_frame = 2,
      .video_delay_frame = 2,
      .hs_video_delay_frame = 2,
      .slim_video_delay_frame = 2,
      .custom1_delay_frame = 2,
      .custom2_delay_frame = 2,
      .custom3_delay_frame = 2,
      .custom4_delay_frame = 2,
      .custom5_delay_frame = 2,
      .frame_time_delay_frame = 3,
      .isp_driving_current = ISP_DRIVING_4MA,
      .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
      .mipi_sensor_type = MIPI_OPHY_NCSI2,
      .mipi_settle_delay_mode = 0,
      .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
      .mclk = 24,
      .mipi_lane_num = SENSOR_MIPI_4_LANE,
      .i2c_addr_table = {0x34, 0xff},
      .i2c_speed = 400,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,	/* current shutter */
	.gain = 0x100,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x34, /* record current sensor's i2c write id */
};

static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[] = {
{3280, 2464,   8,   8,   3264, 2448, 3264, 2448,   0,   0, 3264, 2448, 0, 0, 3264, 2448},//pre
{3280, 2464,   8,   8,   3264, 2448, 3264, 2448,   0,   0, 3264, 2448, 0, 0, 3264, 2448},//cap
{3280, 2464,   8,   8,   3264, 2448, 3264, 2448,   0,   0, 3264, 2448, 0, 0, 3264, 2448},//video
{3280, 2464,   0,   0,   3280, 2464, 1640, 1232, 116, 220, 1408,  792, 0, 0, 1408,  792},//hs
{3280, 2464,   8,   8,   3264, 2448, 3264, 2448,   0,   0, 3264, 2448, 0, 0, 3264, 2448},//slim
{3280, 2464,   8,   8,   3264, 2448, 3264, 2448,   0,   0, 3264, 2448, 0, 0, 3264, 2448},//custom1
{3280, 2464,   0,   0,   3280, 2464, 1640, 1232,   0,   0, 1640, 1232, 0, 0, 1640, 1232},//custom2
{3280, 2464,   8,   8,   3264, 2448, 3264, 2448,   0,   0, 3264, 2448, 0, 0, 3264, 2448},//custom3
{3280, 2464,   8, 312,   3264, 1840, 3264, 1840,   0,   0, 3264, 1840, 0, 0, 3264, 1840},//custom4
{3280, 2464,   0,   0,   3280, 2464, 1640, 1232,   0,   0, 1640, 1232, 0, 0, 1640, 1232},//custom5
};

static kal_uint16 sensor_init_setting_array[] = {
     0x0106, 0x01,
     0x0136, 0x18,
     0x0137, 0x00,
     0x4348, 0x16,
     0x4350, 0x19,
     0x4408, 0x0A,
     0x440C, 0x0B,
     0x4411, 0x5F,
     0x4412, 0x2C,
     0x4623, 0x00,
     0x462C, 0x0F,
     0x462D, 0x00,
     0x462E, 0x00,
     0x4684, 0x54,
     0x480A, 0x07,
     0x4908, 0x07,
     0x4909, 0x07,
     0x490D, 0x0A,
     0x491E, 0x0F,
     0x4921, 0x06,
     0x4923, 0x28,
     0x4924, 0x28,
     0x4925, 0x29,
     0x4926, 0x29,
     0x4927, 0x1F,
     0x4928, 0x20,
     0x4929, 0x20,
     0x492A, 0x20,
     0x492C, 0x05,
     0x492D, 0x06,
     0x492E, 0x06,
     0x492F, 0x06,
     0x4930, 0x03,
     0x4931, 0x04,
     0x4932, 0x04,
     0x4933, 0x05,
     0x595E, 0x01,
     0x5963, 0x01,
};

static kal_uint16 preview_setting_array[] = {
     0x0808, 0x02,
     0x0112, 0x0A,
     0x0113, 0x0A,
     0x0114, 0x03,
     0x0342, 0x0E,
     0x0343, 0x58,
     0x0340, 0x09,
     0x0341, 0xF4,
     0x0344, 0x00,
     0x0345, 0x08,
     0x0346, 0x00,
     0x0347, 0x08,
     0x0348, 0x0C,
     0x0349, 0xC7,
     0x034A, 0x09,
     0x034B, 0x97,
     0x0220, 0x00,
     0x0222, 0x01,
     0x0900, 0x00,
     0x0901, 0x11,
     0x0902, 0x00,
     0x034C, 0x0C,
     0x034D, 0xC0,
     0x034E, 0x09,
     0x034F, 0x90,
     0x0301, 0x05,
     0x0303, 0x01,
     0x0305, 0x02,
     0x0306, 0x00,
     0x0307, 0x78,
     0x030B, 0x01,
     0x030D, 0x04,
     0x030E, 0x00,
     0x030F, 0x75,
     0x0310, 0x00,
     0x0700, 0x00,
     0x0701, 0x10,
     0x0820, 0x0A,
     0x0821, 0xF8,
     0x3088, 0x04,
     0x6813, 0x02,
     0x6835, 0x07,
     0x6836, 0x00,
     0x6837, 0x04,
     0x684D, 0x07,
     0x684E, 0x00,
     0x684F, 0x04,
     0x0202, 0x09,
     0x0203, 0xEA,
     0x0204, 0x00,
     0x0205, 0x00,
     0x020E, 0x01,
     0x020F, 0x00,
     0x080A, 0x00,
     0x080B, 0x6F,
     0x080C, 0x00,
     0x080D, 0x2F,
     0x080E, 0x00,
     0x080F, 0x57,
     0x0810, 0x00,
     0x0811, 0x40,
     0x0812, 0x00,
     0x0813, 0x2F,
     0x0814, 0x00,
     0x0815, 0x2F,
     0x0816, 0x00,
     0x0817, 0xBF,
     0x0818, 0x00,
     0x0819, 0x27,
     0x30A2, 0x00,
     0x30A3, 0x4F,
     0x30A0, 0x00,
     0x30A1, 0x0F,
};
static kal_uint16 capture_setting_array[] = {
     0x0808, 0x02,
     0x0112, 0x0A,
     0x0113, 0x0A,
     0x0114, 0x03,
     0x0342, 0x0E,
     0x0343, 0x58,
     0x0340, 0x09,
     0x0341, 0xF4,
     0x0344, 0x00,
     0x0345, 0x08,
     0x0346, 0x00,
     0x0347, 0x08,
     0x0348, 0x0C,
     0x0349, 0xC7,
     0x034A, 0x09,
     0x034B, 0x97,
     0x0220, 0x00,
     0x0222, 0x01,
     0x0900, 0x00,
     0x0901, 0x11,
     0x0902, 0x00,
     0x034C, 0x0C,
     0x034D, 0xC0,
     0x034E, 0x09,
     0x034F, 0x90,
     0x0301, 0x05,
     0x0303, 0x01,
     0x0305, 0x02,
     0x0306, 0x00,
     0x0307, 0x78,
     0x030B, 0x01,
     0x030D, 0x04,
     0x030E, 0x00,
     0x030F, 0x75,
     0x0310, 0x00,
     0x0700, 0x00,
     0x0701, 0x10,
     0x0820, 0x0A,
     0x0821, 0xF8,
     0x3088, 0x04,
     0x6813, 0x02,
     0x6835, 0x07,
     0x6836, 0x00,
     0x6837, 0x04,
     0x684D, 0x07,
     0x684E, 0x00,
     0x684F, 0x04,
     0x0202, 0x09,
     0x0203, 0xEA,
     0x0204, 0x00,
     0x0205, 0x00,
     0x020E, 0x01,
     0x020F, 0x00,
     0x080A, 0x00,
     0x080B, 0x6F,
     0x080C, 0x00,
     0x080D, 0x2F,
     0x080E, 0x00,
     0x080F, 0x57,
     0x0810, 0x00,
     0x0811, 0x40,
     0x0812, 0x00,
     0x0813, 0x2F,
     0x0814, 0x00,
     0x0815, 0x2F,
     0x0816, 0x00,
     0x0817, 0xBF,
     0x0818, 0x00,
     0x0819, 0x27,
     0x30A2, 0x00,
     0x30A3, 0x4F,
     0x30A0, 0x00,
     0x30A1, 0x0F,
};
static kal_uint16 normal_video_setting_array[] = {
     0x0808, 0x02,
     0x0112, 0x0A,
     0x0113, 0x0A,
     0x0114, 0x03,
     0x0342, 0x0E,
     0x0343, 0x58,
     0x0340, 0x09,
     0x0341, 0xF4,
     0x0344, 0x00,
     0x0345, 0x08,
     0x0346, 0x00,
     0x0347, 0x08,
     0x0348, 0x0C,
     0x0349, 0xC7,
     0x034A, 0x09,
     0x034B, 0x97,
     0x0220, 0x00,
     0x0222, 0x01,
     0x0900, 0x00,
     0x0901, 0x11,
     0x0902, 0x00,
     0x034C, 0x0C,
     0x034D, 0xC0,
     0x034E, 0x09,
     0x034F, 0x90,
     0x0301, 0x05,
     0x0303, 0x01,
     0x0305, 0x02,
     0x0306, 0x00,
     0x0307, 0x78,
     0x030B, 0x01,
     0x030D, 0x04,
     0x030E, 0x00,
     0x030F, 0x75,
     0x0310, 0x00,
     0x0700, 0x00,
     0x0701, 0x10,
     0x0820, 0x0A,
     0x0821, 0xF8,
     0x3088, 0x04,
     0x6813, 0x02,
     0x6835, 0x07,
     0x6836, 0x00,
     0x6837, 0x04,
     0x684D, 0x07,
     0x684E, 0x00,
     0x684F, 0x04,
     0x0202, 0x09,
     0x0203, 0xEA,
     0x0204, 0x00,
     0x0205, 0x00,
     0x020E, 0x01,
     0x020F, 0x00,
     0x080A, 0x00,
     0x080B, 0x6F,
     0x080C, 0x00,
     0x080D, 0x2F,
     0x080E, 0x00,
     0x080F, 0x57,
     0x0810, 0x00,
     0x0811, 0x40,
     0x0812, 0x00,
     0x0813, 0x2F,
     0x0814, 0x00,
     0x0815, 0x2F,
     0x0816, 0x00,
     0x0817, 0xBF,
     0x0818, 0x00,
     0x0819, 0x27,
     0x30A2, 0x00,
     0x30A3, 0x4F,
     0x30A0, 0x00,
     0x30A1, 0x0F,
};
static kal_uint16 hs_video_setting_array[] = {
     0x0112, 0x0A,
     0x0113, 0x0A,
     0x0114, 0x03,
     0x0342, 0x07,
     0x0343, 0x2C,
     0x0340, 0x03,
     0x0341, 0x66,
     0x0344, 0x00,
     0x0345, 0xE8,
     0x0346, 0x01,
     0x0347, 0xB8,
     0x0348, 0x0B,
     0x0349, 0xE7,
     0x034A, 0x07,
     0x034B, 0xE7,
     0x0220, 0x00,
     0x0222, 0x01,
     0x0900, 0x01,
     0x0901, 0x22,
     0x0902, 0x00,
     0x034C, 0x05,
     0x034D, 0x80,
     0x034E, 0x03,
     0x034F, 0x18,
     0x0301, 0x05,
     0x0303, 0x01,
     0x0305, 0x02,
     0x0306, 0x00,
     0x0307, 0x78,
     0x030B, 0x01,
     0x030D, 0x04,
     0x030E, 0x00,
     0x030F, 0x78,
     0x0310, 0x00,
     0x0700, 0x00,
     0x0701, 0x10,
     0x0820, 0x0B,
     0x0821, 0x40,
     0x3088, 0x04,
     0x6813, 0x02,
     0x6835, 0x07,
     0x6836, 0x00,
     0x6837, 0x04,
     0x684D, 0x07,
     0x684E, 0x00,
     0x684F, 0x04,
     0x0202, 0x03,
     0x0203, 0x5C,
     0x0204, 0x00,
     0x0205, 0x00,
     0x020E, 0x01,
     0x020F, 0x00,
};
static kal_uint16 slim_video_setting_array[] = {
     0x0808, 0x02,
     0x0112, 0x0A,
     0x0113, 0x0A,
     0x0114, 0x03,
     0x0342, 0x0E,
     0x0343, 0x58,
     0x0340, 0x09,
     0x0341, 0xF4,
     0x0344, 0x00,
     0x0345, 0x08,
     0x0346, 0x00,
     0x0347, 0x08,
     0x0348, 0x0C,
     0x0349, 0xC7,
     0x034A, 0x09,
     0x034B, 0x97,
     0x0220, 0x00,
     0x0222, 0x01,
     0x0900, 0x00,
     0x0901, 0x11,
     0x0902, 0x00,
     0x034C, 0x0C,
     0x034D, 0xC0,
     0x034E, 0x09,
     0x034F, 0x90,
     0x0301, 0x05,
     0x0303, 0x01,
     0x0305, 0x02,
     0x0306, 0x00,
     0x0307, 0x78,
     0x030B, 0x01,
     0x030D, 0x04,
     0x030E, 0x00,
     0x030F, 0x75,
     0x0310, 0x00,
     0x0700, 0x00,
     0x0701, 0x10,
     0x0820, 0x0A,
     0x0821, 0xF8,
     0x3088, 0x04,
     0x6813, 0x02,
     0x6835, 0x07,
     0x6836, 0x00,
     0x6837, 0x04,
     0x684D, 0x07,
     0x684E, 0x00,
     0x684F, 0x04,
     0x0202, 0x09,
     0x0203, 0xEA,
     0x0204, 0x00,
     0x0205, 0x00,
     0x020E, 0x01,
     0x020F, 0x00,
     0x080A, 0x00,
     0x080B, 0x6F,
     0x080C, 0x00,
     0x080D, 0x2F,
     0x080E, 0x00,
     0x080F, 0x57,
     0x0810, 0x00,
     0x0811, 0x40,
     0x0812, 0x00,
     0x0813, 0x2F,
     0x0814, 0x00,
     0x0815, 0x2F,
     0x0816, 0x00,
     0x0817, 0xBF,
     0x0818, 0x00,
     0x0819, 0x27,
     0x30A2, 0x00,
     0x30A3, 0x4F,
     0x30A0, 0x00,
     0x30A1, 0x0F,
};
static kal_uint16 custom1_setting_array[] = {
     0x0112, 0x0A,
     0x0113, 0x0A,
     0x0114, 0x03,
     0x0342, 0x0E,
     0x0343, 0x58,
     0x0340, 0x09,
     0x0341, 0xFE,
     0x0344, 0x00,
     0x0345, 0x08,
     0x0346, 0x00,
     0x0347, 0x08,
     0x0348, 0x0C,
     0x0349, 0xC7,
     0x034A, 0x09,
     0x034B, 0x97,
     0x0220, 0x00,
     0x0222, 0x01,
     0x0900, 0x00,
     0x0901, 0x11,
     0x0902, 0x00,
     0x034C, 0x0C,
     0x034D, 0xC0,
     0x034E, 0x09,
     0x034F, 0x90,
     0x0301, 0x05,
     0x0303, 0x01,
     0x0305, 0x02,
     0x0306, 0x00,
     0x0307, 0x78,
     0x030B, 0x01,
     0x030D, 0x02,
     0x030E, 0x00,
     0x030F, 0x2F,
     0x0310, 0x00,
     0x0700, 0x00,
     0x0701, 0x10,
     0x0820, 0x08,
     0x0821, 0xD0,
     0x3088, 0x04,
     0x6813, 0x02,
     0x6835, 0x00,
     0x6836, 0x01,
     0x6837, 0x04,
     0x684D, 0x00,
     0x684E, 0x01,
     0x684F, 0x04,
     0x0202, 0x09,
     0x0203, 0xF4,
     0x0204, 0x00,
     0x0205, 0x00,
     0x020E, 0x01,
     0x020F, 0x00,
};
static kal_uint16 custom2_setting_array[] = {
     0x0808, 0x02,
     0x0112, 0x0A,
     0x0113, 0x0A,
     0x0114, 0x03,
     0x0342, 0x07,
     0x0343, 0x2C,
     0x0340, 0x0A,
     0x0341, 0x36,
     0x0344, 0x00,
     0x0345, 0x00,
     0x0346, 0x00,
     0x0347, 0x00,
     0x0348, 0x0C,
     0x0349, 0xCF,
     0x034A, 0x09,
     0x034B, 0x9F,
     0x0220, 0x00,
     0x0222, 0x01,
     0x0900, 0x01,
     0x0901, 0x22,
     0x0902, 0x00,
     0x034C, 0x06,
     0x034D, 0x68,
     0x034E, 0x04,
     0x034F, 0xD0,
     0x0301, 0x05,
     0x0303, 0x01,
     0x0305, 0x02,
     0x0306, 0x00,
     0x0307, 0x78,
     0x030B, 0x01,
     0x030D, 0x04,
     0x030E, 0x00,
     0x030F, 0x3C,
     0x0310, 0x00,
     0x0700, 0x00,
     0x0701, 0x10,
     0x0820, 0x05,
     0x0821, 0xA0,
     0x3088, 0x02,
     0x6813, 0x01,
     0x6835, 0x00,
     0x6836, 0x00,
     0x6837, 0x02,
     0x684D, 0x00,
     0x684E, 0x00,
     0x684F, 0x02,
     0x0202, 0x0A,
     0x0203, 0x2C,
     0x0204, 0x00,
     0x0205, 0x00,
     0x020E, 0x01,
     0x020F, 0x00,
     0x080A, 0x00,
     0x080B, 0x57,
     0x080C, 0x00,
     0x080D, 0x1F,
     0x080E, 0x00,
     0x080F, 0x2F,
     0x0810, 0x00,
     0x0811, 0x31,
     0x0812, 0x00,
     0x0813, 0x17,
     0x0814, 0x00,
     0x0815, 0x17,
     0x0816, 0x00,
     0x0817, 0x6F,
     0x0818, 0x00,
     0x0819, 0x17,
     0x30A2, 0x00,
     0x30A3, 0x2F,
     0x30A0, 0x00,
     0x30A1, 0x0F,
};

static kal_uint16 custom3_setting_array[] = {
     0x0808, 0x02,
     0x0112, 0x0A,
     0x0113, 0x0A,
     0x0114, 0x03,
     0x0342, 0x0E,
     0x0343, 0x58,
     0x0340, 0x09,
     0x0341, 0xF4,
     0x0344, 0x00,
     0x0345, 0x08,
     0x0346, 0x00,
     0x0347, 0x08,
     0x0348, 0x0C,
     0x0349, 0xC7,
     0x034A, 0x09,
     0x034B, 0x97,
     0x0220, 0x00,
     0x0222, 0x01,
     0x0900, 0x00,
     0x0901, 0x11,
     0x0902, 0x00,
     0x034C, 0x0C,
     0x034D, 0xC0,
     0x034E, 0x09,
     0x034F, 0x90,
     0x0301, 0x05,
     0x0303, 0x01,
     0x0305, 0x02,
     0x0306, 0x00,
     0x0307, 0x78,
     0x030B, 0x01,
     0x030D, 0x04,
     0x030E, 0x00,
     0x030F, 0x75,
     0x0310, 0x00,
     0x0700, 0x00,
     0x0701, 0x10,
     0x0820, 0x0A,
     0x0821, 0xF8,
     0x3088, 0x04,
     0x6813, 0x02,
     0x6835, 0x07,
     0x6836, 0x00,
     0x6837, 0x04,
     0x684D, 0x07,
     0x684E, 0x00,
     0x684F, 0x04,
     0x0202, 0x09,
     0x0203, 0xEA,
     0x0204, 0x00,
     0x0205, 0x00,
     0x020E, 0x01,
     0x020F, 0x00,
     0x080A, 0x00,
     0x080B, 0x6F,
     0x080C, 0x00,
     0x080D, 0x2F,
     0x080E, 0x00,
     0x080F, 0x57,
     0x0810, 0x00,
     0x0811, 0x40,
     0x0812, 0x00,
     0x0813, 0x2F,
     0x0814, 0x00,
     0x0815, 0x2F,
     0x0816, 0x00,
     0x0817, 0xBF,
     0x0818, 0x00,
     0x0819, 0x27,
     0x30A2, 0x00,
     0x30A3, 0x4F,
     0x30A0, 0x00,
     0x30A1, 0x0F,
};

static kal_uint16 custom4_setting_array[] = {
     0x0112, 0x0A,
     0x0113, 0x0A,
     0x0114, 0x03,
     0x0342, 0x0E,
     0x0343, 0x58,
     0x0340, 0x07,
     0x0341, 0x7C,
     0x0344, 0x00,
     0x0345, 0x08,
     0x0346, 0x01,
     0x0347, 0x38,
     0x0348, 0x0C,
     0x0349, 0xC7,
     0x034A, 0x08,
     0x034B, 0x67,
     0x0220, 0x00,
     0x0222, 0x01,
     0x0900, 0x00,
     0x0901, 0x11,
     0x0902, 0x00,
     0x034C, 0x0C,
     0x034D, 0xC0,
     0x034E, 0x07,
     0x034F, 0x30,
     0x0301, 0x05,
     0x0303, 0x01,
     0x0305, 0x02,
     0x0306, 0x00,
     0x0307, 0x78,
     0x030B, 0x01,
     0x030D, 0x04,
     0x030E, 0x00,
     0x030F, 0x58,
     0x0310, 0x00,
     0x0700, 0x00,
     0x0701, 0x10,
     0x0820, 0x08,
     0x0821, 0x40,
     0x3088, 0x04,
     0x6813, 0x01,
     0x6835, 0x00,
     0x6836, 0x00,
     0x6837, 0x02,
     0x684D, 0x00,
     0x684E, 0x00,
     0x684F, 0x02,
     0x0202, 0x07,
     0x0203, 0x72,
     0x0204, 0x00,
     0x0205, 0x00,
     0x020E, 0x01,
     0x020F, 0x00,
};

static kal_uint16 custom5_setting_array[] = {
     0x0808, 0x02,
     0x0112, 0x0A,
     0x0113, 0x0A,
     0x0114, 0x03,
     0x0342, 0x07,
     0x0343, 0x2C,
     0x0340, 0x0A,
     0x0341, 0x36,
     0x0344, 0x00,
     0x0345, 0x00,
     0x0346, 0x00,
     0x0347, 0x00,
     0x0348, 0x0C,
     0x0349, 0xCF,
     0x034A, 0x09,
     0x034B, 0x9F,
     0x0220, 0x00,
     0x0222, 0x01,
     0x0900, 0x01,
     0x0901, 0x22,
     0x0902, 0x00,
     0x034C, 0x06,
     0x034D, 0x68,
     0x034E, 0x04,
     0x034F, 0xD0,
     0x0301, 0x05,
     0x0303, 0x01,
     0x0305, 0x02,
     0x0306, 0x00,
     0x0307, 0x78,
     0x030B, 0x01,
     0x030D, 0x04,
     0x030E, 0x00,
     0x030F, 0x3C,
     0x0310, 0x00,
     0x0700, 0x00,
     0x0701, 0x10,
     0x0820, 0x05,
     0x0821, 0xA0,
     0x3088, 0x02,
     0x6813, 0x01,
     0x6835, 0x00,
     0x6836, 0x00,
     0x6837, 0x02,
     0x684D, 0x00,
     0x684E, 0x00,
     0x684F, 0x02,
     0x0202, 0x0A,
     0x0203, 0x2C,
     0x0204, 0x00,
     0x0205, 0x00,
     0x020E, 0x01,
     0x020F, 0x00,
     0x080A, 0x00,
     0x080B, 0x57,
     0x080C, 0x00,
     0x080D, 0x1F,
     0x080E, 0x00,
     0x080F, 0x2F,
     0x0810, 0x00,
     0x0811, 0x31,
     0x0812, 0x00,
     0x0813, 0x17,
     0x0814, 0x00,
     0x0815, 0x17,
     0x0816, 0x00,
     0x0817, 0x6F,
     0x0818, 0x00,
     0x0819, 0x17,
     0x30A2, 0x00,
     0x30A3, 0x2F,
     0x30A0, 0x00,
     0x30A1, 0x0F,
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte<<8)&0xff00) | ((get_byte>>8)&0x00ff);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF),
			     (char)(para >> 8), (char)(para & 0xFF)};
	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF),
			(char)(para & 0xFF)};
	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

#ifdef OPLUS_FEATURE_CAMERA_COMMON

#define  CAMERA_MODULE_INFO_LENGTH  (8)
static kal_uint8 gImx355_SN[CAMERA_MODULE_SN_LENGTH];
static kal_uint8 gImx355_CamInfo[CAMERA_MODULE_INFO_LENGTH];
static void read_eeprom_CamInfo(void)
{
	kal_uint16 idx = 0;
	kal_uint8 get_byte[12];
	for (idx = 0; idx <12; idx++) {
		char pusendcmd[2] = {0x00 , (char)((0x00 + idx) & 0xFF) };
		iReadRegI2C(pusendcmd , 2, (u8*)&get_byte[idx],1, 0xA8);
		LOG_INF("imx355_info[%d]: 0x%x\n", idx, get_byte[idx]);
	}
	gImx355_CamInfo[0] = get_byte[0];
	gImx355_CamInfo[1] = get_byte[1];
	gImx355_CamInfo[2] = get_byte[6];
	gImx355_CamInfo[3] = get_byte[7];
	gImx355_CamInfo[4] = get_byte[8];
	gImx355_CamInfo[5] = get_byte[9];
	gImx355_CamInfo[6] = get_byte[10];
	gImx355_CamInfo[7] = get_byte[11];
}

static void read_eeprom_SN(void)
{
	kal_uint16 idx = 0;
	kal_uint8 *get_byte= &gImx355_SN[0];
	for (idx = 0; idx <CAMERA_MODULE_SN_LENGTH; idx++) {
		char pusendcmd[2] = {0x00 , (char)((0xB0 + idx) & 0xFF) };
		iReadRegI2C(pusendcmd , 2, (u8*)&get_byte[idx],1, 0xA8);
		LOG_INF("imx355_SN[%d]: 0x%x  0x%x\n", idx, get_byte[idx], gImx355_SN[idx]);
	}
}
#endif

#define I2C_BUFFER_LEN 255 /* trans# max is 255, each 3 bytes */
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

/*
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
*/

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0;
	reg_gain = 1024 - (1024*64)/gain;
	return (kal_uint16) reg_gain;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor(0x0601, 0x0002); /*100% Color bar*/
	} else {
		write_cmos_sensor(0x0601, 0x0000); /*No pattern*/
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* return;*/ /* for test */
	write_cmos_sensor_8(0x0350, 0x00); /* Disable auto extend */
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor_8(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor_8(0x0343, imgsensor.line_length & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);
}	/*	set_dummy  */

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	/*kal_int16 dummy_line;*/
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF(
		"framerate = %d, min framelength should enable %d\n", framerate,
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
}	/*	set_max_framerate  */

static void write_shutter(kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;
	LOG_INF("enter shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);
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
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10
				/ imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			write_cmos_sensor_8(0x0104, 0x01);
			write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
			write_cmos_sensor_8(0x0104, 0x00);
		}
	} else {
			write_cmos_sensor_8(0x0104, 0x01);
			write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor_8(0x0341,imgsensor.frame_length & 0xFF);
			write_cmos_sensor_8(0x0104, 0x00);

	}
	/* Update Shutter */
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0350, 0x01); /* Enable auto extend */
	write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, shutter  & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);
	LOG_INF("exit shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);
}

/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
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
} /* set_shutter */

/*************************************************************************
 * FUNCTION
 *	set_shutter_frame_length
 *
 * DESCRIPTION
 *	for frame & 3A sync
 *
 *************************************************************************/

static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length, kal_bool auto_extend_en)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;

	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter)
			? imgsensor_info.min_shutter : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
				imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor_8(0x0104, 0x01);
			write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
			write_cmos_sensor_8(0x0104, 0x00);
		}
	} else {
			write_cmos_sensor_8(0x0104, 0x01);
			write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
			write_cmos_sensor_8(0x0104, 0x00);
	}

	/* Update Shutter*/
	write_cmos_sensor_8(0x0104, 0x01);
	if (auto_extend_en)
		write_cmos_sensor_8(0x0350, 0x01); /* Enable auto extend */
	else
		write_cmos_sensor_8(0x0350, 0x00); /* Disable auto extend */
	write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, shutter  & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);
	LOG_INF("shutter =%d, framelength =%d, dummy_line=%d, auto_extend=%d\n",
			shutter, imgsensor.frame_length, dummy_line, read_cmos_sensor(0x0350));
}	/* set_shutter_frame_length */

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
 kal_uint16 reg_gain, max_gain = imgsensor_info.max_gain;

 if (gain < imgsensor_info.min_gain || gain > max_gain) {
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
 LOG_INF("gain = %d, reg_gain = 0x%x, max_gain:0x%x\n ",
	 gain, reg_gain, max_gain);

 write_cmos_sensor_8(0x0104, 0x01);
 write_cmos_sensor_8(0x0204, (reg_gain>>8) & 0xFF);
 write_cmos_sensor_8(0x0205, reg_gain & 0xFF);
 write_cmos_sensor_8(0x0104, 0x00);

 return gain;
} /* set_gain */

/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 streaming_control(kal_bool enable)
{
	int timeout = 100;
	int i = 0;
	int framecnt = 0;

	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor_8(0x0100, 0X01);
		mdelay(10);
	} else {
		write_cmos_sensor_8(0x0100, 0x00);
		for (i = 0; i < timeout; i++) {
			mdelay(5);
			framecnt = read_cmos_sensor_8(0x0005);
			LOG_INF("framecnt = %d.\n", framecnt);
			if (framecnt == 0xFF) {
				LOG_INF(" Stream Off OK at i=%d.\n", i);
				return ERROR_NONE;
			}
		}
		LOG_INF("Stream Off Fail! framecnt=%d.\n", framecnt);
	}
	return ERROR_NONE;
}

static void sensor_init(void)
{
	LOG_INF("E \n");
	table_write_cmos_sensor(sensor_init_setting_array,	sizeof(sensor_init_setting_array)/sizeof(kal_uint16));
}

static void preview_setting(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(preview_setting_array,	sizeof(preview_setting_array)/sizeof(kal_uint16));
}

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(capture_setting_array,	sizeof(capture_setting_array)/sizeof(kal_uint16));
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(normal_video_setting_array,	sizeof(normal_video_setting_array)/sizeof(kal_uint16));
}

static void hs_video_setting(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(hs_video_setting_array,	sizeof(hs_video_setting_array)/sizeof(kal_uint16));
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(slim_video_setting_array,	sizeof(slim_video_setting_array)/sizeof(kal_uint16));
}

static void custom1_setting(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(custom1_setting_array,	sizeof(custom1_setting_array)/sizeof(kal_uint16));
}

static void custom2_setting(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(custom2_setting_array,	sizeof(custom2_setting_array)/sizeof(kal_uint16));
}

static void custom3_setting(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(custom3_setting_array,	sizeof(custom3_setting_array)/sizeof(kal_uint16));
}

static void custom4_setting(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(custom4_setting_array,	sizeof(custom4_setting_array)/sizeof(kal_uint16));
}

static void custom5_setting(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(custom5_setting_array,	sizeof(custom5_setting_array)/sizeof(kal_uint16));
}

/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor_8(0x0016) << 8)
					| read_cmos_sensor_8(0x0017));
            if (*sensor_id == 0x355) {
                *sensor_id = imgsensor_info.sensor_id;
                #ifdef OPLUS_FEATURE_CAMERA_COMMON
                read_eeprom_SN();
				read_eeprom_CamInfo();
                #endif
				pr_info("imx355_mipi_raw: i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			pr_info("imx355_mipi_raw: Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			get_imgsensor_id(&sensor_id);
            if (sensor_id == 0x355) {
                sensor_id = imgsensor_info.sensor_id;
				pr_info("imx355_mipi_raw: i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
				break;
			}
			pr_info("imx355_mipi_raw: Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
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
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
} /* open */

/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");
	streaming_control(KAL_FALSE);
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
} /* preview */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
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
  imgsensor.pclk = imgsensor_info.cap.pclk;
  imgsensor.line_length = imgsensor_info.cap.linelength;
  imgsensor.frame_length = imgsensor_info.cap.framelength;
  imgsensor.min_frame_length = imgsensor_info.cap.framelength;
  imgsensor.dummy_line = 0;
  imgsensor.dummy_pixel = 0;
  imgsensor.autoflicker_en = KAL_FALSE;
  spin_unlock(&imgsensor_drv_lock);
  capture_setting(imgsensor.current_fps);

  return ERROR_NONE;
}   /* capture() */
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
  imgsensor.dummy_line = 0;
  imgsensor.dummy_pixel = 0;
  imgsensor.autoflicker_en = KAL_FALSE;
  spin_unlock(&imgsensor_drv_lock);
  normal_video_setting(imgsensor.current_fps);

  return ERROR_NONE;
}   /*  normal_video	 */

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

  return ERROR_NONE;
}   /*  hs_video	 */

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
}   /* slim_video */


static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
  LOG_INF("E\n");

  spin_lock(&imgsensor_drv_lock);
  imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
  imgsensor.pclk = imgsensor_info.custom1.pclk;
  imgsensor.line_length = imgsensor_info.custom1.linelength;
  imgsensor.frame_length = imgsensor_info.custom1.framelength;
  imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
  imgsensor.dummy_line = 0;
  imgsensor.dummy_pixel = 0;
  imgsensor.autoflicker_en = KAL_FALSE;
  spin_unlock(&imgsensor_drv_lock);
  custom1_setting();

  return ERROR_NONE;
}   /* custom1 */

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
  LOG_INF("E\n");

  spin_lock(&imgsensor_drv_lock);
  imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
  imgsensor.pclk = imgsensor_info.custom2.pclk;
  imgsensor.line_length = imgsensor_info.custom2.linelength;
  imgsensor.frame_length = imgsensor_info.custom2.framelength;
  imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
  imgsensor.dummy_line = 0;
  imgsensor.dummy_pixel = 0;
  imgsensor.autoflicker_en = KAL_FALSE;
  spin_unlock(&imgsensor_drv_lock);
  custom2_setting();

  return ERROR_NONE;
}   /* custom2 */

static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
  LOG_INF("E\n");

  spin_lock(&imgsensor_drv_lock);
  imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
  imgsensor.pclk = imgsensor_info.custom3.pclk;
  imgsensor.line_length = imgsensor_info.custom3.linelength;
  imgsensor.frame_length = imgsensor_info.custom3.framelength;
  imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
  imgsensor.dummy_line = 0;
  imgsensor.dummy_pixel = 0;
  imgsensor.autoflicker_en = KAL_FALSE;
  spin_unlock(&imgsensor_drv_lock);
  custom3_setting();

  return ERROR_NONE;
}	/* custom3 */

static kal_uint32 custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
  LOG_INF("E\n");

  spin_lock(&imgsensor_drv_lock);
  imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
  imgsensor.pclk = imgsensor_info.custom4.pclk;
  imgsensor.line_length = imgsensor_info.custom4.linelength;
  imgsensor.frame_length = imgsensor_info.custom4.framelength;
  imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
  imgsensor.dummy_line = 0;
  imgsensor.dummy_pixel = 0;
  imgsensor.autoflicker_en = KAL_FALSE;
  spin_unlock(&imgsensor_drv_lock);
  custom4_setting();

  return ERROR_NONE;
}	/* custom4*/

static kal_uint32 custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
  LOG_INF("E\n");

  spin_lock(&imgsensor_drv_lock);
  imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
  imgsensor.pclk = imgsensor_info.custom5.pclk;
  imgsensor.line_length = imgsensor_info.custom5.linelength;
  imgsensor.frame_length = imgsensor_info.custom5.framelength;
  imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
  imgsensor.dummy_line = 0;
  imgsensor.dummy_pixel = 0;
  imgsensor.autoflicker_en = KAL_FALSE;
  spin_unlock(&imgsensor_drv_lock);
  custom5_setting();

  return ERROR_NONE;
}	/* custom5 */

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

	return ERROR_NONE;
} /* get_resolution */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
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
	sensor_info->PDAF_Support = 0;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	/*jiangtao.ren@Camera add for support af temperature compensation 20191101*/
	sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
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

	sensor_info->FrameTimeDelayFrame =
		imgsensor_info.frame_time_delay_frame;


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

	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}	/*	get_info  */


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
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */


static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
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
	if (framerate == 0)
		return ERROR_NONE;

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
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
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
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
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
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
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
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
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
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
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
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
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
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
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
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		frame_length = imgsensor_info.custom4.pclk / framerate * 10
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
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
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
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
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
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
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
       default:
            break;
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
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	/* unsigned long long *feature_return_para
	 *  = (unsigned long long *) feature_para;
	 */
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	/* SET_SENSOR_AWB_GAIN *pSetSensorAWB
	 *  = (SET_SENSOR_AWB_GAIN *)feature_para;
	 */
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
		= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
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
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = SENSOR_FUSION_THRESHOLD;
		break;
    case SENSOR_FEATURE_GET_MODULE_SN:
        LOG_INF("imx355 GET_MODULE_SN:%d %d\n", *feature_para_len, *feature_data_32);
        if (*feature_data_32 < CAMERA_MODULE_SN_LENGTH/4)
            *(feature_data_32 + 1) = (gImx355_SN[4*(*feature_data_32) + 3] << 24)
                        | (gImx355_SN[4*(*feature_data_32) + 2] << 16)
                        | (gImx355_SN[4*(*feature_data_32) + 1] << 8)
                        | (gImx355_SN[4*(*feature_data_32)] & 0xFF);
    break;
    case SENSOR_FEATURE_GET_MODULE_INFO:
         LOG_INF("imx355 GET_MODULE_CamInfo:%d %d\n", *feature_para_len, *feature_data_32);
         *(feature_data_32 + 1) = (gImx355_CamInfo[1] << 24)
                    | (gImx355_CamInfo[0] << 16)
                    | (gImx355_CamInfo[3] << 8)
                    | (gImx355_CamInfo[2] & 0xFF);
         *(feature_data_32 + 2) = (gImx355_CamInfo[5] << 24)
                    | (gImx355_CamInfo[4] << 16)
                    | (gImx355_CamInfo[7] << 8)
                    | (gImx355_CamInfo[6] & 0xFF);
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
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	case SENSOR_FEATURE_CHECK_MODULE_ID:
		*feature_return_para_32 = imgsensor_info.module_id;
		break;
	#endif
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
		set_test_pattern_mode((BOOL)*feature_data);
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
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[0],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_PDAF:
		LOG_INF("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data,
			(UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data), (UINT16) (*(feature_data + 1)), (UINT16) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
		#if 0
		ihdr_write_shutter((UINT16)*feature_data,
				   (UINT16)*(feature_data+1));
		#endif
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
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*feature_return_para_32 = 1; /*BINNING_NONE*/
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*feature_return_para_32 = 1; /*BINNING_SUMMED*/
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
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
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
	}
	break;
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


UINT32 IMX355_MIPI_RAW_LAFITE_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
