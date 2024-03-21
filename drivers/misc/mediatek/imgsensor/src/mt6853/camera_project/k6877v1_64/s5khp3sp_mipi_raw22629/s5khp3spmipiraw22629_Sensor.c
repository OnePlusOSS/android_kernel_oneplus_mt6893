/*
 * Copyright (C) 2020 MediaTek Inc.
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
 *	 S5KHP3SPmipiraw22629_sensor.c
 *
 * Project:
 * --------
 *	 ALPS MT6785
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
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
#include "imgsensor_eeprom.h"
#include "imgsensor_common.h"
#include "imgsensor_i2c.h"
#include "imgsensor_hwcfg_custom.h"

#include "s5khp3spmipiraw22629_Sensor.h"
#include "s5khp3sp_sensor22629_setting.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "S5KHP3SP"
#define LOG_INF_NEW(format, args...)    pr_info(PFX "[%s] " format, __func__, ##args)
#define LOG_INF LOG_INF_NEW
#define LOG_1 LOG_INF("S5KHP3SP,MIPI 3LANE\n")
#define SENSORDB LOG_INF
static kal_uint8 EepData[DUALCAM_CALI_DATA_LENGTH
			+CAMERA_MODULE_SN_LENGTH+CAMERA_MODULE_INFO_LENGTH];
/****************************   Modify end    *******************************************/
#define IMGSENSOR_I2C_1000K
static bool bNeedSetNormalMode = KAL_FALSE;

static kal_uint8 deviceInfo_register_value = 0x00;
extern enum IMGSENSOR_RETURN Eeprom_DataInit(
			enum IMGSENSOR_SENSOR_IDX sensor_idx,
			kal_uint32 sensorID);
static kal_uint32 seamless_switch(enum MSDK_SCENARIO_ID_ENUM scenario_id, uint32_t *ae_ctrl);
static kal_bool isDisableWriteInsensorZoomSetting = KAL_FALSE;
static kal_uint16 read_cmos_eeprom_8(kal_uint16 addr);
static kal_uint16 read_cmos_otp(kal_uint32 addr);

static kal_bool is_otp_sensor = KAL_FALSE;
extern struct CAMERA_DEVICE_INFO gImgEepromInfo;

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5KHP3SP_SENSOR_ID22629,	//record sensor id defined in Kd_imgsensor.h
	.checksum_value = 0xb1f1b3cc,	//checksum value for Camera Auto Test - Capture_Setting
	.pre = {
		.pclk = 2000000000,
		.linelength  = 10400,
		.framelength = 6400,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 1557942857,
	},
	.cap = {
		.pclk = 2000000000,
		.linelength  = 10400,
		.framelength = 6400,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 1557942857,
	},
	.normal_video = {
		.pclk = 2000000000,
		.linelength  = 10400,
		.framelength = 6392,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4080,
		.grabwindow_height = 2296,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 1557942857,
	},
	.hs_video = {
		.pclk = 2000000000,
		.linelength  = 5920,
		.framelength = 2812,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 1200,
		.mipi_pixel_rate = 1360457142,
	},
	.slim_video = {
		.pclk = 2000000000,
		.linelength  = 10400,
		.framelength = 6400,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 1557942857,
	},
	.custom1 = {
		.pclk = 2000000000,
		.linelength  = 10400,
		.framelength = 8000,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,
		.mipi_pixel_rate = 1557942857,
	},
	.custom2 = {
		.pclk = 2000000000,
		.linelength  = 10400,
		.framelength = 3204,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4080,
		.grabwindow_height = 2296,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 600,
		.mipi_pixel_rate = 1557942857,
	},
	.custom3 = {
		.pclk = 2000000000,
		.linelength  = 20800,
		.framelength = 12808,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 16320,
		.grabwindow_height = 12240,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 75,
		.mipi_pixel_rate = 1656685714,
	},
	.custom4 = {
		.pclk = 1760000000,
		.linelength  = 11648,
		.framelength = 6208,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 8160,
		.grabwindow_height = 6144,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,
		.mipi_pixel_rate = 1557942857,
	},
	.custom5 = {
		.pclk = 2000000000,
		.linelength  = 20320,
		.framelength = 3279,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 1557942857,
	},
	.custom6 = {
		.pclk = 1760000000,
		.linelength  = 11648,
		.framelength = 4992,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 1557942857,
	},
	.custom7 = {
		.pclk = 2000000000,
		.linelength  = 5920,
		.framelength = 1404,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 2400,
		.mipi_pixel_rate = 1360457142,
	},
	.custom8 = {
		.pclk = 2000000000,
		.linelength  = 7520,
		.framelength = 8844,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4080,
		.grabwindow_height = 2296,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 1557942857,
	},
	.custom9 = {
		.pclk = 1760000000,
		.linelength  = 11648,
		.framelength = 6296,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,
		.mipi_pixel_rate = 2272000000,
	},
	.custom10 = {
		.pclk = 1760000000,
		.linelength  = 11648,
		.framelength = 6208,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 8160,
		.grabwindow_height = 6144,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,
		.mipi_pixel_rate = 1557942857,
	},

	.margin = 25,		/* sensor framelength & shutter margin */
	.min_shutter = 12,	/* min shutter */

	.min_gain = 64,
	.max_gain = 8192,
	.min_gain_iso = 100,
	.gain_step = 2,
	.gain_type = 2, //0-SONY; 1-OV; 2 - SUMSUN; 3 -HYNIX; 4 -GC

	.max_frame_length = 0xfffd,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 15,
	.cap_delay_frame = 2,	/* enter capture delay frame num */
	.pre_delay_frame = 1,	/* enter preview delay frame num */
	.video_delay_frame = 2,	/* enter video delay frame num */
	.hs_video_delay_frame = 3,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 3,//enter slim video delay frame num
	.custom1_delay_frame = 1,
	.custom2_delay_frame = 1,
	.custom3_delay_frame = 1,
	.custom4_delay_frame = 1,
	.custom5_delay_frame = 1,
	.custom6_delay_frame = 1,
	.custom7_delay_frame = 1,
	.custom8_delay_frame = 1,
	.custom9_delay_frame = 1,
	.custom10_delay_frame = 1,
	.frame_time_delay_frame = 2,
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_settle_delay_mode = 1, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,//SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.i2c_addr_table = {0x20, 0xff},
	.i2c_speed = 1000, // i2c read/write speed
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,	/* NORMAL information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,	/* current shutter */
	.gain = 0x100,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = 0,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = KAL_FALSE, //sensor need support LE, SE with HDR feature
	.ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x20,//record current sensor's i2c write id
};

/* Sensor output window information*/
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[15] = {
	{16320, 12288,   0,    0, 16320, 12288, 4080, 3072,
		0,     0, 4080, 3072,     0,     0, 4080, 3072}, // Preview
	{16320, 12288,   0,    0, 16320, 12288, 4080, 3072,
		0,     0, 4080, 3072,     0,     0, 4080, 3072}, // capture
	{16320, 12288,   0, 1552, 16320, 9184,  4080, 2296,
		0,     0, 4080, 2296,     0,    0,  4080, 2296}, // video
	{16320, 12288, 480, 1824, 15360, 8640,  1920, 1080,
		0,     0, 1920, 1080,     0,    0,  1920, 1080}, // hight speed video
	{16320, 12288,   0,    0, 16320, 12288, 4080, 3072,
		0,     0, 4080, 3072,     0,     0, 4080, 3072}, // slim video
	{16320, 12288,   0,    0, 16320, 12288, 4080, 3072,
		0,     0, 4080, 3072,     0,     0, 4080, 3072}, //Custom1
	{16320, 12288,   0, 1552, 16320, 9184,  4080, 2296,
		0,     0, 4080, 2296,     0,    0,  4080, 2296}, // Custom2
	{16320,12288,    0,    0, 16320,12288,16320, 12288,
		0,     0,16320,12288,     0,    0,16320, 12240}, // custom3
	{16320, 12288,  0 ,    0, 16320,12288,  8160, 6144,
		0,     0, 8160, 6144,     0,    0,  8160, 6144}, // Custom4
	{16320, 12288, 6120, 4608, 4080, 3072,  4080, 3072,
         0,     0, 4080, 3072,     0,    0, 4080, 3072}, // custom5
	{16320, 12288, 4080, 3072,8160,  6144,  4080, 3072,
         0,     0, 4080, 3072,     0,    0, 4080, 3072}, //Custom6
	{16320, 12288, 480, 1824, 15360, 8640,  1920, 1080,
		0,     0, 1920, 1080,     0,    0,  1920, 1080}, // Custom7
	{16320, 12288,   0, 1552, 16320, 9184,  4080, 2296,
		0,     0, 4080, 2296,     0,    0,  4080, 2296}, // custom8
	{16320, 12288,    0,    0,16320,12288,  8160, 6144,
      2040,  1536, 4080, 3072,    0,    0,  4080, 3072}, //Custom9
	{16320, 12288,  0 ,    0, 16320,12288,  8160, 6144,
		0,     0, 8160, 6144,     0,    0,  8160, 6144}, // custom10
};

/*VC1 for HDR(DT=0X35), VC2 for PDAF(DT=0X30), unit : 10bit */
static struct SENSOR_VC_INFO2_STRUCT SENSOR_VC_INFO2[15] = {
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //preivew //QBIN(VBIN)_4080x3072_30FPS vbin + 1exp 0x02 vbin + 2exp 0x04
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0xFF0, 0xc00},
            {VC_PDAF_STATS, 0x01, 0x30, 0x13EC, 0x300},		//4080*1.25=5100=0x13EC
            //{VC_STAGGER_SE, 0x00, 0x2b, 0x1000, 0xc00},
            //{VC_PDAF_STATS_NE, 0x00, 0x30, 0x1400, 0x300},
            //{VC_PDAF_STATS_SE, 0x00, 0x31, 0x1400, 0x300},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //capture //QBIN(VBIN)_4080x3072_30FPS vbin + 1exp
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0xFF0, 0xc00},
            {VC_PDAF_STATS, 0x01, 0x30, 0x13EC, 0x300},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //normal_video //QBIN(VBIN)_4080x2296_30FPS vbin + 1exp
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0xFF0, 0x8F8},
            {VC_PDAF_STATS, 0x02, 0x30, 0x13EC, 0x23E},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //hs_video //QBIN(VBIN)_1920x1080_120FPS vbin + 1exp
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x780, 0x438},
            {VC_PDAF_STATS, 0x01, 0x30, 0x960, 0x10E},		//1920*1.25=2400=0x960
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //slim_video //QBIN(VBIN)_4080x3072_30FPS vbin + 1exp
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0xFF0, 0xc00},
            {VC_PDAF_STATS, 0x01, 0x30, 0x13EC, 0x300},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom1 //QBIN(VBIN)_4080x3072_30FPS vbin + 1exp
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0xFF0, 0xc00},
            {VC_PDAF_STATS, 0x01, 0x30, 0x13EC, 0x300},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom2 //QBIN(VBIN)_4080x2296_30FPS vbin + 1exp
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0xFF0, 0x8F8},
            {VC_PDAF_STATS, 0x01, 0x30, 0x13EC, 0x23E},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom3 //QBIN(VBIN)_16320x12288_7.5FPS vbin + 1exp  QRMSC 8192x6144_24FPS
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x3FC0, 0x3000},
            {VC_PDAF_STATS, 0x01, 0x30, 0x13EC, 0x300},
        },
        1
    },
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom4	QBIN(VBIN)_8160x6144_24FPS
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1FE0, 0x1800},
            {VC_PDAF_STATS, 0x01, 0x30, 0x13EC, 0x300},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom5 //QBIN(VBIN)_4080x3072_30FPS vbin + 1exp
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0xFF0, 0xc00},
            {VC_PDAF_STATS, 0x01, 0x30, 0x13EC, 0x300},
        },
        1
    },
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom6 //QBIN(VBIN)_4080x3072_30FPS vbin + 1exp
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0xFF0, 0xc00},
            {VC_PDAF_STATS, 0x01, 0x30, 0x13EC, 0x300},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom7 //QBIN(VBIN)_1920x1080_240FPS vbin + 1exp
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x780, 0x438},
            {VC_PDAF_STATS, 0x01, 0x30, 0x960, 0x10E},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom8 //QBIN(VBIN)_4080x2296_30FPS vbin + 1exp
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0xFF0, 0x8F8},
            {VC_STAGGER_ME, 0x01, 0x2b, 0xFF0, 0x8F8},
            {VC_PDAF_STATS, 0x02, 0x30, 0x13EC, 0x23E},
        },
        1
    },
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom9 //QBIN(VBIN)_16320x12288_7.5FPS vbin + 1exp
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0xFF0, 0xC00},
            {VC_PDAF_STATS, 0x01, 0x30, 0x9F6, 0x180},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom10 //QBIN(VBIN)_8160x6144_24FPS vbin + 1exp
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 0x1FE0, 0x1800},
            {VC_PDAF_STATS, 0x01, 0x30, 0x13EC, 0x300},
        },
        1
    }
};

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
        {0, 0}, {0, 0}, {0, 388}, {60, 108}, {0, 0},
        {0, 0}, {0, 356}, {0, 0}, {0, 356}, {0, 356},
        {0, 0}, {60, 108}, {0, 0}, {1020, 768}, {0, 0}
    },  //{0, 1632}
    .iMirrorFlip = 0,
};



static kal_uint16 read_cmos_sensor_byte(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2CTiming(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
	return get_byte;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2CTiming(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
	return get_byte;
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF),
						(char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2CTiming(pusendcmd, 4, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2CTiming(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF),
			(char)(para & 0xFF)};

	iWriteRegI2CTiming(pusendcmd, 3, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
}

static void check_streamoff(void)
{
	unsigned int i = 0;
	int timeout = (10000 / imgsensor.current_fps) + 1;

	mdelay(3);
	for (i = 0; i < timeout; i++) {
		if (read_cmos_sensor(0x0005) != 0xFF)
			mdelay(1);
		else
			break;
	}
	pr_debug("%s exit!\n", __func__);
}

static kal_uint32 streaming_control(kal_bool enable)
{
	pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable) {
		write_cmos_sensor(0xFCFC, 0x4000);
		write_cmos_sensor(0x0100, 0x0100);
	} else {
		//write_cmos_sensor(0xFCFC, 0x4000);
		//write_cmos_sensor(0xFD70, 0x4A40);
		write_cmos_sensor(0xFCFC, 0x4000);
		write_cmos_sensor(0x0100, 0x0000);
		check_streamoff();
	}
	return ERROR_NONE;
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

static void write_frame_len_test(kal_uint32 fll, kal_uint32 exp_cnt)
{
    // //write_frame_len should be called inside GRP_PARAM_HOLD (0x0104)
    // FRM_LENGTH_LINES must be multiple of 4
    //imgsensor.frame_length = round_up(fll / exp_cnt, 4) * exp_cnt;
    if(imgsensor.frame_length % 4 != 0) {
        imgsensor.frame_length = imgsensor.frame_length + 4 - imgsensor.frame_length % 4;
    }

    if (imgsensor.extend_frame_length_en == KAL_FALSE) {
        LOG_INF("fast_mode_on %d\n", imgsensor.fast_mode_on);
        write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
        //write_cmos_sensor_8(0x0341, imgsensor.frame_length / exp_cnt & 0xFF);
    }

    LOG_INF("fast_mode_on %d\n", imgsensor.fast_mode_on);
    if (imgsensor.fast_mode_on == KAL_TRUE) {
        imgsensor.fast_mode_on = KAL_FALSE;
        write_cmos_sensor_8(0x3010, 0x00);
    }
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	write_cmos_sensor(0x0342, imgsensor.line_length & 0xFFFF);
}	/*	set_dummy  */

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable(%d)\n",
		framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ?
		frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static void write_shutter(kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;
	kal_uint64 CintR = 0;
	kal_uint64 Time_Farme = 0;

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin) {
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	} else {
		imgsensor.frame_length = imgsensor.min_frame_length;
	}
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter) {
		shutter = imgsensor_info.min_shutter;
	}

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296,0);
		}else if (realtime_fps >= 593 && realtime_fps <= 607) {
 			set_max_framerate(592,0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146,0);
		} else {
			// Extend frame length
			write_cmos_sensor(0x0340, imgsensor.frame_length);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length);
	}

	if (shutter >= 0xFFF0) {  // need to modify line_length & PCLK
		bNeedSetNormalMode = KAL_TRUE;

		if (shutter >= 6153846) {  //>32s
			shutter = 6153846;
		}

		CintR = ( (unsigned long long)shutter) / 128;
		Time_Farme = CintR + 0x0002;  // 1st framelength
		LOG_INF("CintR =%d \n", CintR);

		write_cmos_sensor(0x0340, Time_Farme & 0xFFFF);  // Framelength
		write_cmos_sensor(0x0202, CintR & 0xFFFF);  //shutter
		write_cmos_sensor(0x0702, 0x0700);
		write_cmos_sensor(0x0704, 0x0700);
	} else {
		if (bNeedSetNormalMode) {
			LOG_INF("exit long shutter\n");
			write_cmos_sensor(0x0702, 0x0000);
			write_cmos_sensor(0x0704, 0x0000);
			bNeedSetNormalMode = KAL_FALSE;
		}

		write_cmos_sensor(0x0340, imgsensor.frame_length);
		write_cmos_sensor(0x0202, imgsensor.shutter);
	}
	LOG_INF("shutter =%d, framelength =%d \n", shutter,imgsensor.frame_length);
	LOG_INF("input shutter: %d, output shutter: 0x%x \n", shutter, ((read_cmos_sensor_byte(0x0202) << 8) | read_cmos_sensor_byte(0x0203)));
}	/*	write_shutter  */

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
static void set_shutter_frame_length(kal_uint32 shutter,
				     kal_uint32 frame_length,
				     kal_bool auto_extend_en)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;
	kal_uint64 CintR = 0;
	kal_uint64 Time_Farme = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;

	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
	    imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter)
			? imgsensor_info.min_shutter : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296, 0);
		}else if (realtime_fps >= 593 && realtime_fps <= 607) {
 			set_max_framerate(592,0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			/* Extend frame length */
			write_cmos_sensor(0x0340, imgsensor.frame_length);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x0340, imgsensor.frame_length);
	}

	if (shutter >= 0xFFF0) {
		bNeedSetNormalMode = KAL_TRUE;

		if (shutter >= 1538000) {
			shutter = 1538000;
		}
		CintR = (5013 * (unsigned long long)shutter) / 321536;
		Time_Farme = CintR + 0x0002;
		LOG_INF("CintR =%d \n", CintR);

		write_cmos_sensor(0x0340, Time_Farme & 0xFFFF);
		write_cmos_sensor(0x0202, CintR & 0xFFFF);
		write_cmos_sensor(0x0702, 0x0700);
		write_cmos_sensor(0x0704, 0x0700);
	} else {
		if (bNeedSetNormalMode) {
			LOG_INF("exit long shutter\n");
			write_cmos_sensor(0x0702, 0x0000);
			write_cmos_sensor(0x0704, 0x0000);
			bNeedSetNormalMode = KAL_FALSE;
		}

		write_cmos_sensor(0x0340, imgsensor.frame_length);
		write_cmos_sensor(0x0202, imgsensor.shutter);
	}

	LOG_INF("Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
		shutter, imgsensor.frame_length, frame_length, dummy_line, read_cmos_sensor(0x0350));
}	/* set_shutter_frame_length */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;

	reg_gain = gain/2;
	return (kal_uint16)reg_gain;
}

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
	kal_uint16 reg_gain;

	//gain = 64 = 1x real gain.
	if (gain < BASEGAIN || gain > 128 * BASEGAIN) {
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 128 * BASEGAIN)
			gain = 128 * BASEGAIN;
	}
	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
	write_cmos_sensor(0x0204, (reg_gain&0xFFFF));
	return gain;
}	/*	set_gain  */

static kal_uint32 s5khp3sp_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	LOG_INF("s5khp3sp_awb_gain: 0x0400\n");

	write_cmos_sensor(0x0D82, 0x0400);
	write_cmos_sensor(0x0D84, 0x0400);
	write_cmos_sensor(0x0D86, 0x0400);
	return ERROR_NONE;
}

/*write AWB gain to sensor*/
static void feedback_awbgain(kal_uint32 r_gain, kal_uint32 b_gain)
{
	UINT32 r_gain_int = 0;
	UINT32 b_gain_int = 0;

	r_gain_int = r_gain * 2;
	b_gain_int = b_gain * 2;

	/*write r_gain*/
	write_cmos_sensor(0x0D82, r_gain_int);
	/*write _gain*/
	write_cmos_sensor(0x0D86, b_gain_int);
}

/* read xtalk data */
#define S5KHP3SP_XTALK_START_ADDR  0x3980
#define S5KHP3SP_XTALK_DATA_SIZE   9216
#define S5KHP3SP_OTP_DATA_SIZE     38274
#define S5KHP3SP_OTP_DATA_PAGE_SIZE  64
#define S5KHP3SP_XTALK_DATA_transform_size   8192


static kal_uint8  s5kh3sp_data_xtalk[S5KHP3SP_XTALK_DATA_SIZE];
static kal_uint8  s5kh3sp_data_OTP[S5KHP3SP_OTP_DATA_SIZE];

static unsigned int read_4cell_eeprom_data(char *data)
{
	if (data != NULL) {
		memcpy((void*)(data), (void*)s5kh3sp_data_xtalk, 8192);
	}
	return 0;
}

static unsigned int read_4cell_eeprom_data2(char *data)
{
	if (data != NULL) {
		memcpy((void*)(data), (void*)s5kh3sp_data_xtalk + S5KHP3SP_XTALK_DATA_transform_size , 1024);
	}
	return 0;
}


static unsigned int read_4cell_otp_data(char *data,int num)
{
	if (data != NULL) {
		memcpy((void*)(data) , (void*)s5kh3sp_data_OTP + S5KHP3SP_XTALK_DATA_transform_size * num, 8192);
	}
	return 0;
}

static unsigned int read_4cell_otp_data_end(char *data)
{
	if (data != NULL) {
		memcpy((void*)(data) , (void*)s5kh3sp_data_OTP + S5KHP3SP_XTALK_DATA_transform_size * 4, 2688);
	}
	return 0;
}


static void read_4cell_from_eeprom_s5kh3sp(void)
{
	kal_uint16 idx = 0;
	for (idx = 0 ; idx <S5KHP3SP_XTALK_DATA_SIZE ; idx++) {
		s5kh3sp_data_xtalk[idx]=read_cmos_eeprom_8(S5KHP3SP_XTALK_START_ADDR+idx);
	}
}

static void read_4cell_from_OTP_s5kh3sp(void)
{
	kal_uint16 idx = 0;
	kal_uint16 page = 0;
	kal_uint16 pageIndex = 0;

	write_cmos_sensor(0xFCFC,0x4000);
	write_cmos_sensor(0x6004,0x0000);
	write_cmos_sensor(0x6012,0x0001);
	mdelay(10);
	write_cmos_sensor(0x7002,0x0080);
	write_cmos_sensor(0x6014,0x0001);
	mdelay(60);
	write_cmos_sensor(0x0136,0x1800);
	write_cmos_sensor(0x0FEA,0x0000);
	write_cmos_sensor(0xFCFC,0x2001);
	write_cmos_sensor(0x77E0,0x0000);
	write_cmos_sensor(0x77E2,0x0000);
	write_cmos_sensor(0x74A4,0x0000);
	write_cmos_sensor(0x74A6,0x0000);
	write_cmos_sensor(0x76E4,0x0000);
	write_cmos_sensor(0x76E6,0x0000);
	write_cmos_sensor(0x8B88,0x0000);
	write_cmos_sensor(0x8B8A,0x0000);
	write_cmos_sensor(0xFCFC,0x2000);
	write_cmos_sensor(0x13B0,0x0000);
	write_cmos_sensor(0x13B2,0x0000);
	write_cmos_sensor(0x13B4,0x0000);
	write_cmos_sensor(0x13B6,0x0000);
	write_cmos_sensor(0x13B8,0x0000);
	write_cmos_sensor(0x13BA,0x0000);
	write_cmos_sensor(0xFCFC,0x2001);
	write_cmos_sensor(0xAAD0,0x0000);
	write_cmos_sensor(0x2450,0x0001);
	write_cmos_sensor(0xFCFC,0x2000);
	write_cmos_sensor(0x1470,0x0001);
	write_cmos_sensor(0x1474,0x0001);
	write_cmos_sensor(0x1476,0x89C0);
	write_cmos_sensor(0x1480,0x0001);
	write_cmos_sensor(0x1482,0x89C0);
	write_cmos_sensor(0xFCFC,0x4000);
	write_cmos_sensor(0x0100,0x0100);
	mdelay(2);
	for(page = 0x0033;page<0x0275;page++) {
		write_cmos_sensor(0x0A02,page);
		write_cmos_sensor(0x0A00,0x0100);
		mdelay(1);
		for(idx=0;idx<S5KHP3SP_OTP_DATA_PAGE_SIZE;idx++){
			s5kh3sp_data_OTP[pageIndex*S5KHP3SP_OTP_DATA_PAGE_SIZE+idx] = read_cmos_otp(0x0A04+idx);
			//LOG_INF("otp data is 0x%x index %d",s5kh3sp_data_OTP[pageIndex*S5KHP3SP_OTP_DATA_PAGE_SIZE+idx],pageIndex*S5KHP3SP_OTP_DATA_PAGE_SIZE+idx);
		}
		pageIndex ++;
	}

	for(page = 0x0613;page<0x0627;page++) {
		write_cmos_sensor(0x0A02,page);
		write_cmos_sensor(0x0A00,0x0100);
		mdelay(1);
		for(idx=0;idx<S5KHP3SP_OTP_DATA_PAGE_SIZE;idx++){
			s5kh3sp_data_OTP[pageIndex*S5KHP3SP_OTP_DATA_PAGE_SIZE+idx] = read_cmos_otp(0x0A04+idx);
			//LOG_INF("otp data is 0x%x index %d",s5kh3sp_data_OTP[pageIndex*S5KHP3SP_OTP_DATA_PAGE_SIZE+idx],pageIndex*S5KHP3SP_OTP_DATA_PAGE_SIZE+idx);
		}
		pageIndex ++;
	}

	write_cmos_sensor(0x0A00,0x0000);
	write_cmos_sensor(0xFCFC,0x4000);
	write_cmos_sensor(0x6004,0x0001);

}

static kal_uint16 read_cmos_otp(kal_uint32 addr)
{
    kal_uint16 get_byte = 0;
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};

    iReadRegI2CTiming(pusendcmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
    return get_byte;

}


static kal_uint16 read_cmos_eeprom_8(kal_uint16 addr)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2CTiming(pusendcmd , 2, (u8*)&get_byte, 1, 0xA0, imgsensor_info.i2c_speed);
	return get_byte;
}


static void gm1st_set_lsc_reg_setting(
		kal_uint8 index, kal_uint16 *regDa, MUINT32 regNum)
{
	int i;
	int startAddr[4] = {0x9D88, 0x9CB0, 0x9BD8, 0x9B00};
	/*0:B,1:Gb,2:Gr,3:R*/

	LOG_INF("E! index:%d, regNum:%d\n", index, regNum);

	write_cmos_sensor_8(0x0B00, 0x01); /*lsc enable*/
	write_cmos_sensor_8(0x9014, 0x01);
	write_cmos_sensor_8(0x4439, 0x01);
	mdelay(1);
	LOG_INF("Addr 0xB870, 0x380D Value:0x%x %x\n",
		read_cmos_sensor_8(0xB870), read_cmos_sensor_8(0x380D));
	/*define Knot point, 2'b01:u3.7*/
	write_cmos_sensor_8(0x9750, 0x01);
	write_cmos_sensor_8(0x9751, 0x01);
	write_cmos_sensor_8(0x9752, 0x01);
	write_cmos_sensor_8(0x9753, 0x01);

	for (i = 0; i < regNum; i++)
		write_cmos_sensor(startAddr[index] + 2*i, regDa[i]);

	write_cmos_sensor_8(0x0B00, 0x00); /*lsc disable*/
}

static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.mirror = image_mirror;
	spin_unlock(&imgsensor_drv_lock);
	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor(0x0101, 0x00); //GR
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor(0x0101, 0x01); //R
		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor(0x0101, 0x02); //B
		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor(0x0101, 0x03); //GB
		break;
	default:
		LOG_INF("Error image_mirror setting\n");
	}

}

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


#define MULTI_WRITE 1
#if MULTI_WRITE
static const int I2C_BUFFER_LEN = 1020; /*trans# max is 255, each 4 bytes*/
#else
static const int I2C_BUFFER_LEN = 4;
#endif
static kal_uint16 table_write_cmos_sensor(
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
			puSendCmd[tosend++] = (char)(data >> 8);
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}

#if MULTI_WRITE
	if ((I2C_BUFFER_LEN - tosend) < 4 || IDX == len || addr != addr_last) {
		iBurstWriteReg_multi(puSendCmd, tosend,
			imgsensor.i2c_write_id, 4, imgsensor_info.i2c_speed);

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

void extend_frame_length_test(kal_uint32 ns)
{
    kal_uint32 per_frame_ms = (kal_uint32)(((unsigned long long)imgsensor.frame_length * (unsigned long long)imgsensor.line_length * 1000) / (unsigned long long)imgsensor.pclk);

    LOG_INF("per_frame_ms: %d / %d = %d",
        (imgsensor.frame_length * imgsensor.line_length * 1000), imgsensor.pclk, per_frame_ms);
    imgsensor.frame_length = (per_frame_ms + (ns / 1000000)) * imgsensor.frame_length / per_frame_ms;

    write_cmos_sensor_8(0x0104, 0x01);
    write_frame_len_test(imgsensor.frame_length, get_cur_exp_cnt());
    write_cmos_sensor_8(0x0104, 0x00);
    imgsensor.extend_frame_length_en = KAL_TRUE;
}

#define  I2C_Burst_BUFFER_LEN  2040
static kal_uint16 hp3_burst_write_cmos_sensor(
	kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_Burst_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;

	while (len > IDX) {
		addr = para[IDX];
		if(tosend == 0){
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data >> 8);
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;
		} else if(addr == addr_last + 2) {
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data >> 8);
			puSendCmd[tosend++] = (char)(data & 0xFF);
			addr_last = addr;
			IDX += 2;
		}

		if ((tosend >= I2C_BUFFER_LEN) || IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend,
				imgsensor.i2c_write_id, tosend, imgsensor_info.i2c_speed);
			tosend = 0;
		}
	}
	return 0;
}

static void sensor_init(void)
{
	LOG_INF("s5khp3sp--- E\n");

    write_cmos_sensor(0xFCFC, 0x4000);
    write_cmos_sensor(0x0000, 0x0004);
    write_cmos_sensor(0x0000, 0x1B73);
    write_cmos_sensor(0x6012, 0x0001);
    write_cmos_sensor(0x7002, 0x0008);
    write_cmos_sensor(0x6014, 0x0001);
	mdelay(10);

	table_write_cmos_sensor(addr_data_pair_init_hp3,
		sizeof(addr_data_pair_init_hp3) / sizeof(kal_uint16));

	LOG_INF("s5khp3sp--- X\n");
}

static void otp_sensor_init(void)
{
	LOG_INF("s5khp3sp--- E\n");

    write_cmos_sensor(0xFCFC, 0x4000);
    write_cmos_sensor(0x0000, 0x0005);
    write_cmos_sensor(0x0000, 0x1B73);
    write_cmos_sensor(0x6012, 0x0001);
    write_cmos_sensor(0x7002, 0x0008);
    write_cmos_sensor(0x6014, 0x0001);
	mdelay(10);

	table_write_cmos_sensor(addr_data_pair_init_hp3_otp,
		sizeof(addr_data_pair_init_hp3_otp) / sizeof(kal_uint16));

	hp3_burst_write_cmos_sensor(addr_data_pair_init_hp3_otp_burst1,
		sizeof(addr_data_pair_init_hp3_otp_burst1) / sizeof(kal_uint16));

	hp3_burst_write_cmos_sensor(addr_data_pair_init_hp3_otp_burst2,
		sizeof(addr_data_pair_init_hp3_otp_burst2) / sizeof(kal_uint16));
    write_cmos_sensor(0xFCFC, 0x4000);
    write_cmos_sensor(0x6226, 0x0000);

	LOG_INF("s5khp3sp--- X\n");
}

static void preview_setting(void)
{
	LOG_INF("s5khp3sp--- preview_setting_0719  E\n");
	if (is_otp_sensor) {
		write_cmos_sensor(0xFCFC, 0x2001);
		write_cmos_sensor(0x19EC, 0x0200);
		write_cmos_sensor(0x19EE, 0x0400);
		write_cmos_sensor(0x19F0, 0x0800);
		write_cmos_sensor(0xFCFC, 0x4000);
		write_cmos_sensor(0x0136, 0x1800);
		write_cmos_sensor(0x013E, 0x0000);
		write_cmos_sensor(0x0304, 0x0003);
		write_cmos_sensor(0x0306, 0x00FA);
		write_cmos_sensor(0x030C, 0x0001);
		write_cmos_sensor(0x030E, 0x0003);
		write_cmos_sensor(0x0310, 0x008E);
		write_cmos_sensor(0x0312, 0x0000);
		write_cmos_sensor(0x031A, 0x0003);
		write_cmos_sensor(0x031C, 0x002D);
		write_cmos_sensor(0x031E, 0x0002);
		write_cmos_sensor(0x0340, 0x1900);
		write_cmos_sensor(0x0B30, 0x0100);
		write_cmos_sensor(0xfcfc, 0x4000);
		write_cmos_sensor(0x0100, 0x0000);
		write_cmos_sensor(0x0a70, 0x0001);
		write_cmos_sensor(0xfcfc, 0x2000);
		write_cmos_sensor(0x147c, 0x0200);
		write_cmos_sensor(0xfcfc, 0x2001);
		write_cmos_sensor(0x9f98, 0x0000);
		write_cmos_sensor(0xfcfc, 0x4000);
	} else {
		write_cmos_sensor(0xfcfc,0x2003);
		write_cmos_sensor(0x0a36,0x0000);
		write_cmos_sensor(0x0a38,0x0000);
		write_cmos_sensor(0x0fd4,0x0000);
		write_cmos_sensor(0x0fd6,0x0000);
		write_cmos_sensor(0xfcfc,0x4000);
		write_cmos_sensor(0x0100,0x0000);
		write_cmos_sensor(0xfcfc,0x2000);
		write_cmos_sensor(0x13d8,0x2003);
		write_cmos_sensor(0x13da,0x0000);
		write_cmos_sensor(0xfcfc,0x4000);
		write_cmos_sensor(0x0b30,0x01ff);
	}
	LOG_INF("s5khp3sp--- preview_setting_0719 dont write setting  X\n");
}

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	LOG_INF("%s E! currefps:%d\n", __func__, currefps);

	LOG_INF("s5khp3sp--- capture_setting  E\n");
	if (is_otp_sensor) {
		write_cmos_sensor(0xFCFC, 0x2001);
		write_cmos_sensor(0x19EC, 0x0200);
		write_cmos_sensor(0x19EE, 0x0400);
		write_cmos_sensor(0x19F0, 0x0800);
		write_cmos_sensor(0xFCFC, 0x4000);
		write_cmos_sensor(0x0136, 0x1800);
		write_cmos_sensor(0x013E, 0x0000);
		write_cmos_sensor(0x0304, 0x0003);
		write_cmos_sensor(0x0306, 0x00FA);
		write_cmos_sensor(0x030C, 0x0001);
		write_cmos_sensor(0x030E, 0x0003);
		write_cmos_sensor(0x0310, 0x008E);
		write_cmos_sensor(0x0312, 0x0000);
		write_cmos_sensor(0x031A, 0x0003);
		write_cmos_sensor(0x031C, 0x002D);
		write_cmos_sensor(0x031E, 0x0002);
		write_cmos_sensor(0x0340, 0x1900);
		write_cmos_sensor(0x0B30, 0x0100);
		write_cmos_sensor(0xfcfc, 0x4000);
		write_cmos_sensor(0x0100, 0x0000);
		write_cmos_sensor(0x0a70, 0x0001);
		write_cmos_sensor(0xfcfc, 0x2000);
		write_cmos_sensor(0x147c, 0x0200);
		write_cmos_sensor(0xfcfc, 0x2001);
		write_cmos_sensor(0x9f98, 0x0000);
		write_cmos_sensor(0xfcfc, 0x4000);
	} else {
		write_cmos_sensor(0xfcfc,0x2003);
		write_cmos_sensor(0x0a36,0x0000);
		write_cmos_sensor(0x0a38,0x0000);
		write_cmos_sensor(0x0fd4,0x0000);
		write_cmos_sensor(0x0fd6,0x0000);
		write_cmos_sensor(0xfcfc,0x4000);
		write_cmos_sensor(0x0100,0x0000);
		write_cmos_sensor(0xfcfc,0x2000);
		write_cmos_sensor(0x13d8,0x2003);
		write_cmos_sensor(0x13da,0x0000);
		write_cmos_sensor(0xfcfc,0x4000);
		write_cmos_sensor(0x0b30,0x01ff);
	}
	LOG_INF("s5khp3sp--- capture_setting  X\n");
}

static void normal_video_setting(void)
{
    LOG_INF("E\n");
    if (is_otp_sensor) {
		write_cmos_sensor(0xFCFC, 0x2001);
		write_cmos_sensor(0x19EC, 0x0200);
		write_cmos_sensor(0x19EE, 0x0400);
		write_cmos_sensor(0x19F0, 0x0800);
		write_cmos_sensor(0xFCFC, 0x4000);
		write_cmos_sensor(0x0136, 0x1800);
		write_cmos_sensor(0x013E, 0x0000);
		write_cmos_sensor(0x0304, 0x0003);
		write_cmos_sensor(0x0306, 0x00FA);
		write_cmos_sensor(0x030C, 0x0001);
		write_cmos_sensor(0x030E, 0x0003);
		write_cmos_sensor(0x0310, 0x008E);
		write_cmos_sensor(0x0312, 0x0000);
		write_cmos_sensor(0x031A, 0x0003);
		write_cmos_sensor(0x031C, 0x002D);
		write_cmos_sensor(0x031E, 0x0002);
		write_cmos_sensor(0x0340, 0x18F8);
		write_cmos_sensor(0x0B30, 0x0103);

        write_cmos_sensor(0xfcfc, 0x4000);
        write_cmos_sensor(0x0100, 0x0000);
        mdelay(30);
        write_cmos_sensor(0x0A70, 0x0001);
        write_cmos_sensor(0x0A72, 0x0100);
        write_cmos_sensor(0xfcfc ,0x2000);
        write_cmos_sensor(0x147c, 0x0201);
        write_cmos_sensor(0xfcfc, 0x2001);
        write_cmos_sensor(0x9f98, 0x0001);
        write_cmos_sensor(0x9f9a, 0x0043);
        write_cmos_sensor(0x9f9c, 0x0000);
        write_cmos_sensor(0x9f9e, 0x0000);
        write_cmos_sensor(0x9fb0, 0x0000);
        write_cmos_sensor(0xfcfc, 0x4000);
        write_cmos_sensor(0x0100, 0x0100);
    } else {
        table_write_cmos_sensor(addr_data_pair_video_hp3,
            sizeof(addr_data_pair_video_hp3) / sizeof(kal_uint16));

        write_cmos_sensor(0xfcfc, 0x4000);
        write_cmos_sensor(0x0100, 0x0000);
        mdelay(30);
        write_cmos_sensor(0x0A70, 0x0001);
        write_cmos_sensor(0x0A72, 0x0100);
        write_cmos_sensor(0xfcfc ,0x2000);
        write_cmos_sensor(0x145c, 0x0201);
        write_cmos_sensor(0xfcfc, 0x2001);
        write_cmos_sensor(0x9a38, 0x0001);
        write_cmos_sensor(0x9a3a, 0x0043);
        write_cmos_sensor(0x9a3c, 0x0000);
        write_cmos_sensor(0x9a3e, 0x0000);
        write_cmos_sensor(0x9a50, 0x0000);
        write_cmos_sensor(0xfcfc, 0x4000);
        write_cmos_sensor(0x0100, 0x0100);
    }

}


static void hs_video_setting(void)
{
	LOG_INF("E\n");
	if (is_otp_sensor) {
		table_write_cmos_sensor(addr_data_pair_hs_hp3_otp,
			sizeof(addr_data_pair_hs_hp3_otp) / sizeof(kal_uint16));
	} else {
		table_write_cmos_sensor(addr_data_pair_hs_hp3,
			sizeof(addr_data_pair_hs_hp3) / sizeof(kal_uint16));
	}
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	if (is_otp_sensor) {
		table_write_cmos_sensor(addr_data_pair_slim_hp3_otp,
			sizeof(addr_data_pair_slim_hp3_otp) / sizeof(kal_uint16));
	} else {
		table_write_cmos_sensor(addr_data_pair_slim_hp3,
			sizeof(addr_data_pair_slim_hp3) / sizeof(kal_uint16));
	}
}

static void custom1_setting(void)
{
	LOG_INF("E\n");
	if (is_otp_sensor) {
		table_write_cmos_sensor(addr_data_pair_cus1_hp3_otp,
			sizeof(addr_data_pair_cus1_hp3_otp) / sizeof(kal_uint16));
	} else {
		table_write_cmos_sensor(addr_data_pair_cus1_hp3,
			sizeof(addr_data_pair_cus1_hp3) / sizeof(kal_uint16));
	}
}

static void custom2_setting(void)
{
	LOG_INF("E\n");
	if (is_otp_sensor) {
		table_write_cmos_sensor(addr_data_pair_cus2_hp3_otp,
			sizeof(addr_data_pair_cus2_hp3_otp) / sizeof(kal_uint16));
	} else {
		table_write_cmos_sensor(addr_data_pair_cus2_hp3,
			sizeof(addr_data_pair_cus2_hp3) / sizeof(kal_uint16));
	}
}

static void custom3_setting(void)
{
	LOG_INF("E\n");
	if (is_otp_sensor) {
		table_write_cmos_sensor(addr_data_pair_cus3_hp3_otp,
			sizeof(addr_data_pair_cus3_hp3_otp) / sizeof(kal_uint16));
	} else {
		table_write_cmos_sensor(addr_data_pair_cus3_hp3,
			sizeof(addr_data_pair_cus3_hp3) / sizeof(kal_uint16));
	}
}

static void custom4_setting(void)
{
	LOG_INF("E\n");
	if (is_otp_sensor) {
		table_write_cmos_sensor(addr_data_pair_cus4_hp3_otp,
			sizeof(addr_data_pair_cus4_hp3_otp) / sizeof(kal_uint16));
	} else {
		table_write_cmos_sensor(addr_data_pair_cus4_hp3,
			sizeof(addr_data_pair_cus4_hp3) / sizeof(kal_uint16));
	}
}

static void custom5_setting(void)
{
	LOG_INF("E\n");
	if (is_otp_sensor) {
		write_cmos_sensor(0xFCFC, 0x4000);
		write_cmos_sensor(0x0104, 0x0101);
		write_cmos_sensor(0xFCFC, 0x4000);
		write_cmos_sensor(0x0B30, 0x0102);	//4x insensorzoom
		write_cmos_sensor(0x0104, 0x0001);
	} else {
		table_write_cmos_sensor(addr_data_pair_cus5_hp3,
			sizeof(addr_data_pair_cus5_hp3) / sizeof(kal_uint16));
	}
}

static void custom6_setting(void)
{
	LOG_INF("E\n");
	if (is_otp_sensor) {
		write_cmos_sensor(0xFCFC, 0x4000);
		write_cmos_sensor(0x0104, 0x0101);
		write_cmos_sensor(0xFCFC, 0x4000);
		write_cmos_sensor(0x0B30, 0x0101);	//2x insensorzoom
		write_cmos_sensor(0x0104, 0x0001);
	} else {
		table_write_cmos_sensor(addr_data_pair_cus6_hp3,
			sizeof(addr_data_pair_cus6_hp3) / sizeof(kal_uint16));
	}
}

static void custom7_setting(void)
{
	LOG_INF("E\n");
	if (is_otp_sensor) {
		table_write_cmos_sensor(addr_data_pair_cus7_hp3_otp,
			sizeof(addr_data_pair_cus7_hp3_otp) / sizeof(kal_uint16));
	} else {
		table_write_cmos_sensor(addr_data_pair_cus7_hp3,
			sizeof(addr_data_pair_cus7_hp3) / sizeof(kal_uint16));
	}
}

static void custom8_setting(void)
{
	LOG_INF("E\n");
	if (is_otp_sensor) {
		write_cmos_sensor(0xFCFC, 0x2001);
		write_cmos_sensor(0x19EC, 0x0200);
		write_cmos_sensor(0x19EE, 0x0400);
		write_cmos_sensor(0x19F0, 0x0800);
		write_cmos_sensor(0xFCFC, 0x4000);
		write_cmos_sensor(0x0136, 0x1800);
		write_cmos_sensor(0x013E, 0x0000);
		write_cmos_sensor(0x0304, 0x0003);
		write_cmos_sensor(0x0306, 0x00FA);
		write_cmos_sensor(0x030C, 0x0001);
		write_cmos_sensor(0x030E, 0x0003);
		write_cmos_sensor(0x0310, 0x008E);
		write_cmos_sensor(0x0312, 0x0000);
		write_cmos_sensor(0x031A, 0x0003);
		write_cmos_sensor(0x031C, 0x002D);
		write_cmos_sensor(0x031E, 0x0002);
		write_cmos_sensor(0x0340, 0x228C);
		write_cmos_sensor(0x0B30, 0x0104);
		write_cmos_sensor(0xfcfc, 0x4000);
		write_cmos_sensor(0x0100, 0x0000);
		write_cmos_sensor(0x0a70, 0x0001);
		write_cmos_sensor(0xfcfc, 0x2000);
		write_cmos_sensor(0x147c, 0x0200);
		write_cmos_sensor(0xfcfc, 0x2001);
		write_cmos_sensor(0x9f98, 0x0000);
		write_cmos_sensor(0xfcfc, 0x4000);	//shdr
	} else {
		table_write_cmos_sensor(addr_data_pair_cus8_hp3,
			sizeof(addr_data_pair_cus8_hp3) / sizeof(kal_uint16));
	}
}

static void custom9_setting(void)
{
	LOG_INF("E\n");
	if (is_otp_sensor) {
		table_write_cmos_sensor(addr_data_pair_cus9_hp3_otp,
			sizeof(addr_data_pair_cus9_hp3_otp) / sizeof(kal_uint16));
	} else {
		table_write_cmos_sensor(addr_data_pair_cus9_hp3,
			sizeof(addr_data_pair_cus9_hp3) / sizeof(kal_uint16));
	}
}

static void custom10_setting(void)
{
	LOG_INF("E\n");
	if (is_otp_sensor) {
		table_write_cmos_sensor(addr_data_pair_cus10_hp3_otp,
			sizeof(addr_data_pair_cus10_hp3_otp) / sizeof(kal_uint16));
	} else {
		table_write_cmos_sensor(addr_data_pair_cus10_hp3,
			sizeof(addr_data_pair_cus10_hp3) / sizeof(kal_uint16));
	}
}

static void hdr_write_tri_shutter_no_gph(kal_uint16 le, kal_uint16 me, kal_uint16 se)
{
    kal_uint16 realtime_fps = 0;
    kal_uint16 exposure_cnt = 2;

    le = (kal_uint16)max(imgsensor_info.min_shutter, (kal_uint32)le);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length = max((kal_uint32)(le + me + se + (exposure_cnt * imgsensor_info.margin)),
                                imgsensor.min_frame_length);
    imgsensor.frame_length = min(imgsensor.frame_length, imgsensor_info.max_frame_length);
    spin_unlock(&imgsensor_drv_lock);

    LOG_INF("Enter le:%d, me:%d, se:%d autoflicker_en %d frame_length %d min_frame_length %d max_frame_length %d\n",
            le, me, se, imgsensor.autoflicker_en, imgsensor.frame_length,
            imgsensor.min_frame_length, imgsensor_info.max_frame_length);

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
        else {
            write_frame_len_test(imgsensor.frame_length, exposure_cnt);
        }
    } else {
        write_frame_len_test(imgsensor.frame_length, exposure_cnt);
    }

	if(imgsensor.frame_length % 4 != 0) {
		imgsensor.frame_length = imgsensor.frame_length + 4 - imgsensor.frame_length % 4;
	}

    if(((le + se) >= imgsensor.frame_length) || (se >= (imgsensor.frame_length) - 4851) || (se >= 4452)){
        se = imgsensor_info.min_shutter;
        LOG_INF("se set min shutter\n");
    }

    LOG_INF("X! le:%d, me:%d, se:%d autoflicker_en %d frame_length %d\n",
        le, me, se, imgsensor.autoflicker_en, imgsensor.frame_length);

    write_cmos_sensor(0xFCFC, 0x4000);
    write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);

    write_cmos_sensor(0x0226, le & 0xFFFF);
    write_cmos_sensor(0x0202, se & 0xFFFF);
    LOG_INF("EXIT! le:0x%x, se:0x%x\n", le, se);

}

static void hdr_write_tri_gain_no_gph(kal_uint16 lgain, kal_uint16 mg, kal_uint16 sg)
{
    kal_uint16 reg_lg, reg_sg;

    if (lgain < BASEGAIN || lgain > 128 * BASEGAIN) {
        LOG_INF("Error gain setting");
        if (lgain < BASEGAIN)
            lgain = BASEGAIN;
        else if (lgain > 128 * BASEGAIN)
            lgain = 128 * BASEGAIN;
    }
    if(sg <= 64)
        sg = 64;

   if(lgain <= 64)
        lgain = 64;

    reg_lg = gain2reg(lgain);
    reg_sg = gain2reg(sg);
	write_cmos_sensor(0xFCFC, 0X4000);// switch page
	write_cmos_sensor(0x020A, 0X0100);// enable long gain and short gain

	write_cmos_sensor(0x0204, reg_sg & 0xFFFF);    /* Short Gian */
	write_cmos_sensor(0x0206, reg_lg & 0xFFFF);    /* Long Gian */
	LOG_INF("X reg_sg:0x%x  reg_lg:0x%x \n", reg_sg,  reg_lg);
	write_cmos_sensor(0x020A, 0X0000);// disenable long gain and short gain
}

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor_byte(0x0000) << 8) | read_cmos_sensor_byte(0x0001));
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
	kal_uint32 otp_sensor_flagValue = 0;
	printk("s5khp3sp get_imgsensor_id----1  0x%x\n",imgsensor_info.sensor_id);	//0x1b70
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		gImgEepromInfo.i4CurSensorIdx = 0;
		gImgEepromInfo.i4CurSensorId = imgsensor_info.sensor_id;
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			//mdelay(2000);
			printk("s5khp3sp get_imgsensor_id----2  0x%x\n",*sensor_id);	//0x1b70
			//*sensor_id = 0x1b70;
			if(imgsensor_info.sensor_id != *sensor_id){
				imgsensor_info.sensor_id = 0x1b73;
			}
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
						imgsensor.i2c_write_id, *sensor_id);
				read_4cell_from_eeprom_s5kh3sp();
				read_4cell_from_OTP_s5kh3sp();
				if (deviceInfo_register_value == 0x00) {
					Eeprom_DataInit(0, S5KHP3SP_SENSOR_ID22629);
					deviceInfo_register_value = 0x01;
				}
				otp_sensor_flagValue = read_cmos_sensor(0x0002);
				printk("otp_sensor_flagValue 0x%x", otp_sensor_flagValue);
				if(otp_sensor_flagValue == 0xa2) {
					is_otp_sensor = KAL_TRUE;
					LOG_INF("s5khm6sp get_other_imgsensor_id  0x%x\n", otp_sensor_flagValue);
				}
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n",
					imgsensor.i2c_write_id);
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
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	LOG_1;
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if(imgsensor_info.sensor_id != sensor_id){
				imgsensor_info.sensor_id = 0x1b73;
			}
			if (sensor_id == imgsensor_info.sensor_id) {
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
	/* initail sequence write in  */
	if(is_otp_sensor) {
		otp_sensor_init();
	}else {
		sensor_init();
	}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = KAL_FALSE;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = 0;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}   /*  open  */

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

	/*No Need to implement this function*/
	isDisableWriteInsensorZoomSetting = KAL_FALSE;

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
	printk("s5khmsp---    preview");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}	/*	preview   */

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
	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) {
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		LOG_INF("cap1-15fps: use cap1's setting: %d fps!\n", imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
		LOG_INF("cap2-24fps: use cap2's setting: %d fps!\n", imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap2.pclk;
		imgsensor.line_length = imgsensor_info.cap2.linelength;
		imgsensor.frame_length = imgsensor_info.cap2.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else{
		LOG_INF("current_fps %d is not support,use cap1\n", imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}	/* capture() */

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
	normal_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}	/*	normal_video   */

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
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 * Custom1
 *
 * DESCRIPTION
 *   This function start the sensor Custom1.
 *
 * PARAMETERS
 *   *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *   None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
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
}   /*  Custom3   */

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
}   /*  Custom4   */
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
}   /*  Custom5   */
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
}   /*  Custom6   */

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
}   /*  Custom7   */

static kal_uint32 Custom8(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM8;
	imgsensor.pclk = imgsensor_info.custom8.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.custom8.linelength;
	imgsensor.frame_length = imgsensor_info.custom8.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom8.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom8_setting();
	return ERROR_NONE;
}   /*  Custom8   */

static kal_uint32 Custom9(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM9;
	imgsensor.pclk = imgsensor_info.custom9.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.custom9.linelength;
	imgsensor.frame_length = imgsensor_info.custom9.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom9.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom9_setting();
	return ERROR_NONE;
}   /*  Custom9   */
static kal_uint32 Custom10(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM10;
	imgsensor.pclk = imgsensor_info.custom10.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.custom10.linelength;
	imgsensor.frame_length = imgsensor_info.custom10.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom10.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom10_setting();
	return ERROR_NONE;
}   /*  Custom10   */
static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth
		= imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight
		= imgsensor_info.cap.grabwindow_height;
	sensor_resolution->SensorPreviewWidth
		= imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight
		= imgsensor_info.pre.grabwindow_height;
	sensor_resolution->SensorVideoWidth
		= imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight
		= imgsensor_info.normal_video.grabwindow_height;
	sensor_resolution->SensorHighSpeedVideoWidth
		= imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight
		= imgsensor_info.hs_video.grabwindow_height;
	sensor_resolution->SensorSlimVideoWidth
		= imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight
		= imgsensor_info.slim_video.grabwindow_height;
	sensor_resolution->SensorCustom1Width
		= imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height
		= imgsensor_info.custom1.grabwindow_height;
	sensor_resolution->SensorCustom2Width
		= imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height
		= imgsensor_info.custom2.grabwindow_height;
	sensor_resolution->SensorCustom3Width
		= imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height
		= imgsensor_info.custom3.grabwindow_height;
	sensor_resolution->SensorCustom4Width
		= imgsensor_info.custom4.grabwindow_width;
	sensor_resolution->SensorCustom4Height
		= imgsensor_info.custom4.grabwindow_height;
	sensor_resolution->SensorCustom5Width
		= imgsensor_info.custom5.grabwindow_width;
	sensor_resolution->SensorCustom5Height
		= imgsensor_info.custom5.grabwindow_height;
	sensor_resolution->SensorCustom6Width
		= imgsensor_info.custom6.grabwindow_width;
	sensor_resolution->SensorCustom6Height
		= imgsensor_info.custom6.grabwindow_height;
	sensor_resolution->SensorCustom7Width
		= imgsensor_info.custom7.grabwindow_width;
	sensor_resolution->SensorCustom7Height
		= imgsensor_info.custom7.grabwindow_height;
	sensor_resolution->SensorCustom8Width
		= imgsensor_info.custom8.grabwindow_width;
	sensor_resolution->SensorCustom8Height
		= imgsensor_info.custom8.grabwindow_height;
	sensor_resolution->SensorCustom9Width
		= imgsensor_info.custom9.grabwindow_width;
	sensor_resolution->SensorCustom9Height
		= imgsensor_info.custom9.grabwindow_height;
	sensor_resolution->SensorCustom10Width
		= imgsensor_info.custom10.grabwindow_width;
	sensor_resolution->SensorCustom10Height
		= imgsensor_info.custom10.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat
		= imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
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
	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame
		= imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    sensor_info->HDR_Support = HDR_SUPPORT_STAGGER;
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
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;

	sensor_info->PDAF_Support = 7;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_CUSTOM2:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_CUSTOM3:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_CUSTOM4:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_CUSTOM5:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_CUSTOM6:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom6.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom6.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_CUSTOM7:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom7.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom7.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_CUSTOM8:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom8.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom8.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_CUSTOM9:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom9.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom9.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_CUSTOM10:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom10.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom10.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
	break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}
	return ERROR_NONE;
}	/*	get_info  */

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
						MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d   %d\n", scenario_id, MSDK_SCENARIO_ID_CAMERA_PREVIEW);
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
	    Custom1(image_window, sensor_config_data); // Custom1
	break;
	case MSDK_SCENARIO_ID_CUSTOM2:
	    Custom2(image_window, sensor_config_data); // Custom1
	break;
	case MSDK_SCENARIO_ID_CUSTOM3:
	    Custom3(image_window, sensor_config_data); // Custom1
	break;
	case MSDK_SCENARIO_ID_CUSTOM4:
	    Custom4(image_window, sensor_config_data); // Custom4
	break;
	case MSDK_SCENARIO_ID_CUSTOM5:
	    Custom5(image_window, sensor_config_data); // Custom1
			break;
	case MSDK_SCENARIO_ID_CUSTOM6:
	    Custom6(image_window, sensor_config_data); // Custom1
	break;
	case MSDK_SCENARIO_ID_CUSTOM7:
	    Custom7(image_window, sensor_config_data); // Custom1
	break;
	case MSDK_SCENARIO_ID_CUSTOM8:
	    Custom8(image_window, sensor_config_data); // Custom1
	break;
	case MSDK_SCENARIO_ID_CUSTOM9:
	    Custom9(image_window, sensor_config_data); // Custom9
	break;
	case MSDK_SCENARIO_ID_CUSTOM10:
	    Custom10(image_window, sensor_config_data); // Custom10
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
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
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
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length
			= imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate * 10
			/ imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength)
			? (frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length
			= imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
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
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 /
			imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength)
			? (frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length
			= imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10
			/ imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength)
			? (frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length
			= imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10
			/ imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength)
			? (frame_length - imgsensor_info.custom1.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length
			= imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10
			/ imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength)
			? (frame_length - imgsensor_info.custom2.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length
			= imgsensor_info.custom2.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		// set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10
			/ imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength)
			? (frame_length - imgsensor_info.custom3.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length
			= imgsensor_info.custom3.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		frame_length = imgsensor_info.custom4.pclk / framerate * 10
			/ imgsensor_info.custom4.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength)
			? (frame_length - imgsensor_info.custom4.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length
			= imgsensor_info.custom4.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		frame_length = imgsensor_info.custom5.pclk / framerate * 10
			/ imgsensor_info.custom5.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom5.framelength)
			? (frame_length - imgsensor_info.custom5.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length
			= imgsensor_info.custom1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM6:
		frame_length = imgsensor_info.custom6.pclk / framerate * 10
			/ imgsensor_info.custom6.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom6.framelength)
			? (frame_length - imgsensor_info.custom6.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length
			= imgsensor_info.custom6.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM7:
		frame_length = imgsensor_info.custom7.pclk / framerate * 10
			/ imgsensor_info.custom7.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom7.framelength)
			? (frame_length - imgsensor_info.custom7.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length
			= imgsensor_info.custom7.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		// set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM8:
		frame_length = imgsensor_info.custom8.pclk / framerate * 10
			/ imgsensor_info.custom8.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom8.framelength)
			? (frame_length - imgsensor_info.custom8.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length
			= imgsensor_info.custom8.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM9:
		frame_length = imgsensor_info.custom9.pclk / framerate * 10
			/ imgsensor_info.custom9.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom9.framelength)
			? (frame_length - imgsensor_info.custom9.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length
			= imgsensor_info.custom9.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM10:
		frame_length = imgsensor_info.custom10.pclk / framerate * 10
			/ imgsensor_info.custom10.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom10.framelength)
			? (frame_length - imgsensor_info.custom10.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length
			= imgsensor_info.custom10.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
		break;
	default:  //coding with  preview scenario by default
		frame_length = imgsensor_info.pre.pclk / framerate * 10
			/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length
			= imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//set_dummy();
		LOG_INF("error scenario_id = %d, use preview\n", scenario_id);
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
	kal_uint16 Color_R = 0, Color_Gr = 0, Color_Gb = 0, Color_B = 0;
	pr_info("s5k3p9sp set_test_pattern enum: %d\n", modes);
	if (pTestpatterndata != NULL) {
		pr_info("R %x GR %x B %x GB %x",pTestpatterndata->COLOR_R, pTestpatterndata->COLOR_Gr, pTestpatterndata->COLOR_B, pTestpatterndata->COLOR_Gb);
		Color_R = (pTestpatterndata->COLOR_R >> 16) & 0xFFFF;
		Color_Gr = (pTestpatterndata->COLOR_Gr >> 16) & 0xFFFF;
		Color_B = (pTestpatterndata->COLOR_B >> 16) & 0xFFFF;
		Color_Gb = (pTestpatterndata->COLOR_Gb >> 16) & 0xFFFF;
	} else {
		pr_err("%s pTestpatterndata is NULL !!!", __func__);
	}

	pr_info("SETTING R %x GR %x B %x GB %x", Color_R, Color_Gr,Color_B,Color_Gb);

	if (modes) {
		write_cmos_sensor(0xFCFC, 0x4000);
		if (modes == 1) { //Solid Color
			write_cmos_sensor_8(0x0601, modes);
			write_cmos_sensor(0x0602, Color_R);
			write_cmos_sensor(0x0604, Color_Gr);
			write_cmos_sensor(0x0606, Color_B);
			write_cmos_sensor(0x0608, Color_Gb);
		} else if (modes == 5) { //test_pattern_black
			pr_info("test_pattern_black");
			write_cmos_sensor_8(0x0601, 0x1);
			write_cmos_sensor(0x0602, 0);
			write_cmos_sensor(0x0604, 0);
			write_cmos_sensor(0x0606, 0);
			write_cmos_sensor(0x0608, 0);
		}
	} else {
		write_cmos_sensor(0x0600, 0x0000); /*No pattern*/
	}
	//write_cmos_sensor_16_8(0x3200, 0x00);
	pr_info("mode:0x%x, reading R 0x%x, GR 0x%x, GB 0x%x, B 0x%x",
		read_cmos_sensor(0x0600),read_cmos_sensor(0x0602),read_cmos_sensor(0x0604),read_cmos_sensor(0x0606),
		read_cmos_sensor(0x0608));
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
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_VC_INFO2_STRUCT *pvcinfo2;
	uint32_t *pAeCtrls;
	uint32_t *pScenarios;
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
	case SENSOR_FEATURE_GET_EEPROM_DATA:
	{
		kal_uint16 data_lens =
					DUALCAM_CALI_DATA_LENGTH
					+ CAMERA_MODULE_SN_LENGTH
					+ CAMERA_MODULE_INFO_LENGTH;
		LOG_INF("SENSOR_FEATURE_GET_EEPROM_DATA:%d data_lens:%d\n",
					*feature_para_len, data_lens);
		memcpy(&feature_para[0], EepData, data_lens);
		break;
	}
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = -1823000;//-1,823,000
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
		case MSDK_SCENARIO_ID_CUSTOM8:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom8.framelength << 16)
				+ imgsensor_info.custom8.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM9:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom9.framelength << 16)
				+ imgsensor_info.custom9.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM10:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom10.framelength << 16)
				+ imgsensor_info.custom10.linelength;
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
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
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
	    // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
	    // if EEPROM does not exist in camera module.
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
	    set_max_framerate_by_scenario(
				(enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
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
	    set_test_pattern_mode((UINT8)*feature_data, (struct SET_SENSOR_PATTERN_SOLID_COLOR *) feature_data+1);
	break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
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

	    wininfo
			= (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

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
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[9],
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
		case MSDK_SCENARIO_ID_CUSTOM8:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[12],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM9:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[13],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM10:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[14],
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
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
			(UINT16) *feature_data);
		PDAFinfo =
		  (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM5:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG: //4000*3000
		case MSDK_SCENARIO_ID_CUSTOM6:
		case MSDK_SCENARIO_ID_CUSTOM7:
		case MSDK_SCENARIO_ID_CUSTOM8:
		case MSDK_SCENARIO_ID_CUSTOM9:
		case MSDK_SCENARIO_ID_CUSTOM10:
			//imgsensor_pd_info_binning.i4BlockNumX = 288;
			//imgsensor_pd_info_binning.i4BlockNumY = 216;
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info, sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		default:
			break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF(
		"SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
			(UINT16) *feature_data);
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM5:
		case MSDK_SCENARIO_ID_CUSTOM7:
		case MSDK_SCENARIO_ID_CUSTOM8:
		case MSDK_SCENARIO_ID_CUSTOM9:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
		break;
	case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
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
	case SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS:
		if ((feature_data + 1) != NULL)
			pScenarios = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
		else {
			LOG_INF("input pScenarios vector is NULL!\n");
			return ERROR_INVALID_SCENARIO_ID;
		}
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*pScenarios = MSDK_SCENARIO_ID_CUSTOM6;
			*(pScenarios + 1) = MSDK_SCENARIO_ID_CUSTOM5;
			break;
        case MSDK_SCENARIO_ID_CUSTOM8:
            *pScenarios = MSDK_SCENARIO_ID_VIDEO_PREVIEW;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *pScenarios = MSDK_SCENARIO_ID_CUSTOM8;
            break;
		case MSDK_SCENARIO_ID_CUSTOM5:
		case MSDK_SCENARIO_ID_CUSTOM6:
			*pScenarios = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
			*(pScenarios + 1) = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
			break;
		default:
			*pScenarios = 0xff;
			break;
		}
		LOG_INF("SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS %d %d\n",
				*feature_data, *pScenarios);
        break;
	case SENSOR_FEATURE_SET_SEAMLESS_EXTEND_FRAME_LENGTH:
        extend_frame_length_test((MUINT32) *feature_data);
		break;
	case SENSOR_FEATURE_SEAMLESS_SWITCH:
	{
		LOG_INF("SENSOR_FEATURE_SEAMLESS_SWITCH");

		if ((feature_data + 1) != NULL) {
			pAeCtrls = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
		} else {
			pr_debug("warning! no ae_ctrl input");
		}

		if (feature_data == NULL) {
			LOG_INF("error! input scenario is null!");
			return ERROR_INVALID_SCENARIO_ID;
		}
		LOG_INF("call seamless_switch");
		seamless_switch((*feature_data), pAeCtrls);
		break;
	}
	case SENSOR_FEATURE_DISABLE_INIT_INSENSORZOOM_SETTING:
		if ((feature_data + 1) == NULL) {
			LOG_INF("input pScenarios vector is NULL!\n");
			return ERROR_INVALID_SCENARIO_ID;
			}

		isDisableWriteInsensorZoomSetting = (*feature_data);

		LOG_INF("isDisableWriteInsensorZoomSetting %d\n",isDisableWriteInsensorZoomSetting);
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT32) (*feature_data),
					(UINT32) (*(feature_data + 1)),
					(BOOL) (*(feature_data + 2)));
		break;
    case SENSOR_FEATURE_GET_STAGGER_TARGET_SCENARIO:
        if (*feature_data == MSDK_SCENARIO_ID_VIDEO_PREVIEW) {
            switch (*(feature_data + 1)) {
            case HDR_RAW_STAGGER_2EXP:
                *(feature_data + 2) = MSDK_SCENARIO_ID_CUSTOM8;//custom8 was the 2 exp mode for preview mode
                break;
            default:
                break;
            }
        }else if (*feature_data == MSDK_SCENARIO_ID_CUSTOM8) {
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
		/*
		 * 1, if driver support new sw frame sync
		 * set_shutter_frame_length() support third para auto_extend_en
		 */
		*(feature_data + 1) = 1;
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;
    case SENSOR_FEATURE_GET_STAGGER_MAX_EXP_TIME:
        if (*feature_data == MSDK_SCENARIO_ID_CUSTOM8) {
            switch (*(feature_data + 1)) {
            case VC_STAGGER_NE:
            case VC_STAGGER_ME:
            case VC_STAGGER_SE:
            default:
                *(feature_data + 2) = 65532 - imgsensor_info.margin;
                break;
            }
        } else {
            *(feature_data + 2) = 0;
        }
        break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
        // implement write shutter for NE/SE
        hdr_write_tri_shutter_no_gph((UINT16)*feature_data, 0,
            (UINT16)*(feature_data+1));
		break;
	case SENSOR_FEATURE_SET_HDR_TRI_SHUTTER://for 3EXP
        LOG_INF("SENSOR_FEATURE_SET_HDR_TRI_SHUTTER LE=%d, ME=%d, SE=%d\n",
            (UINT16) *feature_data,(UINT16) *(feature_data + 1),(UINT16) *(feature_data + 2));
        // implement write shutter for NE/ME/SE
        hdr_write_tri_shutter_no_gph((UINT16)*feature_data, (UINT16)*(feature_data+1),
            (UINT16)*(feature_data+2));
        break;
	case SENSOR_FEATURE_SET_DUAL_GAIN://for 2EXP
        LOG_INF("SENSOR_FEATURE_SET_DUAL_GAIN LE=%d, SE=%d\n",
            (UINT16) *feature_data, (UINT16) *(feature_data + 1));
        // implement write gain for NE/SE
        hdr_write_tri_gain_no_gph((UINT16)*feature_data, 0,
            (UINT16)*(feature_data+1));
        break;
	case SENSOR_FEATURE_SET_HDR_TRI_GAIN:
        LOG_INF("SENSOR_FEATURE_SET_HDR_TRI_GAIN LGain=%d, SGain=%d, MGain=%d\n",
            (UINT16) *feature_data, (UINT16) *(feature_data + 1), (UINT16) *(feature_data + 2));
        // implement write gain for NE/ME/SE
        hdr_write_tri_gain_no_gph((UINT16)*feature_data, (UINT16)*(feature_data+1),
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
        case MSDK_SCENARIO_ID_CUSTOM8:
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
        break;
        /*END OF HDR CMD */
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
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom5.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM6:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom6.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM7:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom7.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM8:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom8.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM9:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom9.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM10:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom10.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_VC_INFO2:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO2 %d\n",
			(UINT16)*feature_data);
		pvcinfo2 =
			(struct SENSOR_VC_INFO2_STRUCT *)(uintptr_t)(*(feature_data+1));
        switch (*feature_data_32) {
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
		break;
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		/* modify to separate 3hdr and remosaic */
		if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3 ||
			imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM4 ||
			imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM5 ||
			imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM6) {
			/*write AWB gain to sensor*/
			feedback_awbgain((UINT32)*(feature_data_32 + 1),
					(UINT32)*(feature_data_32 + 2));
		} else {
			s5khp3sp_awb_gain(
				(struct SET_SENSOR_AWB_GAIN *) feature_para);
		}
		break;
	case SENSOR_FEATURE_GET_4CELL_DATA:
		{
			int type = (kal_uint16)(*feature_data);
			if (type == 0x0) {
				LOG_INF("SENSOR_FEATURE_GET_4CELL_DATA type=%d\n", type);
				read_4cell_eeprom_data((char *)(*(feature_data+1)));
			} else if (type == 0x1) {
				LOG_INF("SENSOR_FEATURE_GET_4CELL_DATA type=%d\n", type);
				read_4cell_eeprom_data2((char *)(*(feature_data+1)));
			} else if (type == 0x2) {
				read_4cell_otp_data((char *)(*(feature_data+1)),0);
			} else if (type == 0x3) {
				read_4cell_otp_data((char *)(*(feature_data+1)),1);
			} else if (type == 0x4) {
				read_4cell_otp_data((char *)(*(feature_data+1)),2);
			} else if (type == 0x5) {
				read_4cell_otp_data((char *)(*(feature_data+1)),3);
			} else if (type == 0x6) {
				read_4cell_otp_data_end((char *)(*(feature_data+1)));
			}
			break;
		}
	case SENSOR_FEATURE_SET_LSC_TBL:
	{
		kal_uint8 index =
			*(((kal_uint8 *)feature_para) + (*feature_para_len));

		gm1st_set_lsc_reg_setting(index, feature_data_16,
					  (*feature_para_len)/sizeof(UINT16));
	}
		break;
	default:
		break;
	}
	return ERROR_NONE;
}    /*    feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

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

    LOG_INF("%s set_seamless_switch = %d",
				__func__, scenario_id);
    switch (scenario_id) {
    case MSDK_SCENARIO_ID_CUSTOM6:
    {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM6;
		imgsensor.pclk = imgsensor_info.custom6.pclk;
		imgsensor.line_length = imgsensor_info.custom6.linelength;
		imgsensor.frame_length = imgsensor_info.custom6.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom6.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
		spin_unlock(&imgsensor_drv_lock);
		if (is_otp_sensor) {
			write_cmos_sensor(0xFCFC, 0x4000);
			write_cmos_sensor(0x0104, 0x0101);
			write_cmos_sensor(0xFCFC, 0x4000);
			//write_cmos_sensor(0x0104, 0x0001);
		} else {
			write_cmos_sensor(0xfcfc,0x2003);
			write_cmos_sensor(0x0a36,0xFFFE);
			write_cmos_sensor(0x0a38,0x4000);
			write_cmos_sensor(0x0fd4,0x0000);
			write_cmos_sensor(0x0fd6,0x0000);
			write_cmos_sensor(0xFcfc,0x4000);
			write_cmos_sensor(0x0104,0x0101);
			write_cmos_sensor(0xFcfc,0x2000);
			write_cmos_sensor(0x13d8,0x0000);
			write_cmos_sensor(0x13da,0x0000);
			write_cmos_sensor(0xFcfc,0x4000);
			//write_cmos_sensor(0x0b30,0x0101);
		}
		if (ae_ctrl) {
			LOG_INF("call MSDK_SCENARIO_ID_CUSTOM6 %d %d",ae_ctrl[SHUTTER_NE_FRM_1] ,ae_ctrl[GAIN_NE_FRM_1]);
			set_shutter(ae_ctrl[SHUTTER_NE_FRM_1]);
			set_gain(ae_ctrl[GAIN_NE_FRM_1]);
		}
		write_cmos_sensor(0x0B30,0x0101); //2x insensorzoom
		write_cmos_sensor(0x0104,0x0001);
	}
	break;
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
		if (is_otp_sensor) {
			write_cmos_sensor(0xFCFC, 0x4000);
			write_cmos_sensor(0x0104, 0x0101);
			write_cmos_sensor(0xFCFC, 0x4000);
			//write_cmos_sensor(0x0104, 0x0001);
		} else {
			write_cmos_sensor(0xfcfc,0x2003);
			write_cmos_sensor(0x0a36,0x0000);
			write_cmos_sensor(0x0a38,0x0000);
			write_cmos_sensor(0x0fd4,0x0000);
			write_cmos_sensor(0x0fd6,0x0000);
			write_cmos_sensor(0xFcfc,0x4000);
			write_cmos_sensor(0x0104,0x0101);
			write_cmos_sensor(0xFcfc,0x2000);
			write_cmos_sensor(0x13d8,0x0000);
			write_cmos_sensor(0x13da,0x0000);
			write_cmos_sensor(0xFcfc,0x4000);
			//write_cmos_sensor(0x0b30,0x0100);
		}
		if (ae_ctrl) {
			LOG_INF("call MSDK_SCENARIO_ID_CAMERA_PREVIEW %d %d",ae_ctrl[SHUTTER_NE_FRM_1] ,ae_ctrl[GAIN_NE_FRM_1]);
			set_shutter(ae_ctrl[SHUTTER_NE_FRM_1]);
			set_gain(ae_ctrl[GAIN_NE_FRM_1]);
		}
		write_cmos_sensor(0x0B30, 0x0100);
		write_cmos_sensor(0x0104,0x0001);

	}
	break;
	case MSDK_SCENARIO_ID_CUSTOM5:
	{
		spin_lock(&imgsensor_drv_lock);
		imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
		imgsensor.pclk = imgsensor_info.custom5.pclk;
		imgsensor.line_length = imgsensor_info.custom5.linelength;
		imgsensor.frame_length = imgsensor_info.custom5.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
		spin_unlock(&imgsensor_drv_lock);
		if (is_otp_sensor) {
			write_cmos_sensor(0xFCFC, 0x4000);
			write_cmos_sensor(0x0104, 0x0101);
			write_cmos_sensor(0xFCFC, 0x4000);
			//write_cmos_sensor(0x0104, 0x0001);
		} else {
			write_cmos_sensor(0xFCFC,0x2003);
			write_cmos_sensor(0x0a36,0xFFFE);
			write_cmos_sensor(0x0a38,0x4000);
			write_cmos_sensor(0x0Fd4,0xFFFE);
			write_cmos_sensor(0x0Fd6,0x4000);
			write_cmos_sensor(0xFcfc,0x4000);
			write_cmos_sensor(0x0104,0x0101);
			write_cmos_sensor(0xFcfc,0x2000);
			write_cmos_sensor(0x13d8,0x0000);
			write_cmos_sensor(0x13da,0x0000);
			write_cmos_sensor(0xFcfc,0x4000);
			//write_cmos_sensor(0x0b30,0x0102);
		}
		if (ae_ctrl) {
			LOG_INF("call MSDK_SCENARIO_ID_CUSTOM5 %d %d",ae_ctrl[SHUTTER_NE_FRM_1] ,ae_ctrl[GAIN_NE_FRM_1]);
			set_shutter(ae_ctrl[SHUTTER_NE_FRM_1]);
			set_gain(ae_ctrl[GAIN_NE_FRM_1]);
		}
		write_cmos_sensor(0x0B30, 0x0102); //4x insensorzoom
		write_cmos_sensor(0x0104,0x0001);
	}
	break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
        imgsensor.autoflicker_en = KAL_FALSE;
        imgsensor.pclk = imgsensor_info.normal_video.pclk;
        imgsensor.line_length = imgsensor_info.normal_video.linelength;
        imgsensor.frame_length = imgsensor_info.normal_video.framelength;
        imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
        spin_unlock(&imgsensor_drv_lock);
		if (is_otp_sensor) {
			write_cmos_sensor(0xFCFC, 0x4000);
			write_cmos_sensor(0x0104, 0x0101);
			write_cmos_sensor(0xFCFC, 0x4000);
		} else {
			write_cmos_sensor(0xFCFC,0x2003);
			write_cmos_sensor(0x0a36,0xFFFE);
			write_cmos_sensor(0x0a38,0x4000);
			write_cmos_sensor(0x0Fd4,0xFFFE);
			write_cmos_sensor(0x0Fd6,0x4000);
			write_cmos_sensor(0xFcfc,0x4000);
			write_cmos_sensor(0x0104,0x0101);
			write_cmos_sensor(0xFcfc,0x2000);
			write_cmos_sensor(0x13d8,0x0000);
			write_cmos_sensor(0x13da,0x0000);
			write_cmos_sensor(0xFcfc,0x4000);

		}
        pr_info("seamless switch normal video!\n");
        if (ae_ctrl) {
            LOG_INF("call MSDK_SCENARIO_ID_CAMERA_PREVIEW %d %d",ae_ctrl[SHUTTER_NE_FRM_1] ,ae_ctrl[GAIN_NE_FRM_1]);
			set_shutter(ae_ctrl[SHUTTER_NE_FRM_1]);
			set_gain(ae_ctrl[GAIN_NE_FRM_1]);
        }
        write_cmos_sensor(0x0b30,0x0103);
        write_cmos_sensor(0x0104,0x0001);
    }
    break;
    case MSDK_SCENARIO_ID_CUSTOM8:
    {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM8;
        imgsensor.autoflicker_en = KAL_FALSE;
        imgsensor.pclk = imgsensor_info.custom8.pclk;
        imgsensor.line_length = imgsensor_info.custom8.linelength;
        imgsensor.frame_length = imgsensor_info.custom8.framelength;
        imgsensor.min_frame_length = imgsensor_info.custom8.framelength;
        spin_unlock(&imgsensor_drv_lock);
		if (is_otp_sensor) {
			write_cmos_sensor(0xFCFC, 0x4000);
			write_cmos_sensor(0x0104, 0x0101);
			write_cmos_sensor(0xFCFC, 0x4000);
		} else {
			write_cmos_sensor(0xFCFC,0x2003);
			write_cmos_sensor(0x0a36,0xFFFE);
			write_cmos_sensor(0x0a38,0x4000);
			write_cmos_sensor(0x0Fd4,0xFFFE);
			write_cmos_sensor(0x0Fd6,0x4000);
			write_cmos_sensor(0xFcfc,0x4000);
			write_cmos_sensor(0x0104,0x0101);
			write_cmos_sensor(0xFcfc,0x2000);
			write_cmos_sensor(0x13d8,0x0000);
			write_cmos_sensor(0x13da,0x0000);
			write_cmos_sensor(0xFcfc,0x4000);
		}
        pr_info("seamless switch dolhdr 2exp video!\n");
        if (ae_ctrl) {
            LOG_INF("call MSDK_SCENARIO_ID_CUSTOM8 %d %d",ae_ctrl[SHUTTER_NE_FRM_1] ,ae_ctrl[GAIN_NE_FRM_1]);
            hdr_write_tri_shutter_no_gph(ae_ctrl[SHUTTER_NE_FRM_1], 0, ae_ctrl[SHUTTER_SE_FRM_1]);
            hdr_write_tri_gain_no_gph(ae_ctrl[GAIN_NE_FRM_1], 0, ae_ctrl[GAIN_SE_FRM_1]);
        }
        write_cmos_sensor(0x0b30,0x0104);
        write_cmos_sensor(0x0104,0x0001);
    }
    break;
	default:
	{
		LOG_INF("%s error! wrong setting in set_seamless_switch = %d",
				__func__, scenario_id);
		return 0xff;
		}
	}
	return 0;
}



UINT32 S5KHP3SP_MIPI_RAW22629_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc =  &sensor_func;
	return ERROR_NONE;
}	/* S5KHP3SP_MIPI_RAW22629_SensorInit */
