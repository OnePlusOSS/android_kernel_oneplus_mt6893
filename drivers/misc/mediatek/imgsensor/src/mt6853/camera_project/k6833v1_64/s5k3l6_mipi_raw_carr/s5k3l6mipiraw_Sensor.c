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
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5k3l6mipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
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

#include "s5k3l6mipiraw_Sensor.h"

//#include <linux/hardware_info.h>	//For hardwareinfo
static kal_uint32 streaming_control(kal_bool enable);
static bool bIsLongExposure = KAL_FALSE;

#undef ORINGNAL_VERSION

#define MULTI_WRITE 1
#if MULTI_WRITE
#define I2C_BUFFER_LEN 1024
#else
#define I2C_BUFFER_LEN 4
#endif


#define PFX "S5K3l6_camera_sensor"

#define LOG_INF(format, args...)    \
		pr_info(PFX "[%s] " format, __func__, ##args)
#define LOG_DBG(format, args...)    \
		pr_debug(PFX "[%s] " format, __func__, ##args)
#define LOG_ERR(format, args...)    \
		pr_err(PFX "[%s] " format, __func__, ##args)


static DEFINE_SPINLOCK(imgsensor_drv_lock);


static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = TRUE_S5K3L6_SENSOR_ID,
	.checksum_value = 0x44724ea1,

	.pre = {
		.pclk = 480000000,				//record different mode's pclk
		.linelength  = 4896,				//record different mode's linelength
		.framelength = 3260,			//record different mode's framelength
		.startx= 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 4160,		//record different mode's width of grabwindow
		.grabwindow_height = 3120,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
		.mipi_pixel_rate = 454400000,
	},

	.cap = {
		.pclk = 480000000,				//record different mode's pclk
		.linelength  = 4896,				//record different mode's linelength
		.framelength = 3260,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 4160,		//record different mode's width of grabwindow
		.grabwindow_height = 3120,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
		.mipi_pixel_rate = 454400000,
	},

	.normal_video = {
		.pclk = 480000000,				//record different mode's pclk
		.linelength  = 4896,			//record different mode's linelength
		.framelength = 3209,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 4160,		//record different mode's width of grabwindow
		.grabwindow_height = 2340,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
		.mipi_pixel_rate = 454400000,
	},
	.hs_video = {
		.pclk = 480000000,				//record different mode's pclk
		.linelength  = 4896,//5808,				//record different mode's linelength
		.framelength = 816,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1280,		//record different mode's width of grabwindow
		.grabwindow_height = 720,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 1200,
		.mipi_pixel_rate = 142400000,  //need change further by fenneike
	},
	.slim_video = {
		.pclk = 480000000,				//record different mode's pclk
		.linelength  = 4640,//5808,				//record different mode's linelength
		.framelength = 1724,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1920,		//record different mode's width of grabwindow
		.grabwindow_height = 1080,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 600,
		.mipi_pixel_rate = 220800000,  //need change further by fenneike
	},

    .custom1 = {
        .pclk = 480000000,              //record different mode's pclk
        .linelength  = 4896,                //record different mode's linelength
        .framelength = 4084,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width  = 4160,      //record different mode's width of grabwindow
        .grabwindow_height = 3120,      //record different mode's height of grabwindow
        /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
        .mipi_data_lp2hs_settle_dc = 85,
        /*   following for GetDefaultFramerateByScenario()  */
        .max_framerate = 240,
        .mipi_pixel_rate = 454400000,
    },
    .custom2 = {
        .pclk = 480000000,              //record different mode's pclk
        .linelength  = 4896,                //record different mode's linelength
        .framelength = 1632,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width  = 2104,      //record different mode's width of grabwindow
        .grabwindow_height = 1560,      //record different mode's height of grabwindow
        /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
        .mipi_data_lp2hs_settle_dc = 85,
        /*   following for GetDefaultFramerateByScenario()  */
        .max_framerate = 600,
        .mipi_pixel_rate = 227200000,  //need change further by fenneike
    },
    .custom3 = {
        .pclk = 480000000,              //record different mode's pclk
        .linelength  = 4896,                //record different mode's linelength
        .framelength = 3209,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width  = 4096,      //record different mode's width of grabwindow
        .grabwindow_height = 2304,      //record different mode's height of grabwindow
        /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
        .mipi_data_lp2hs_settle_dc = 85,
        /*   following for GetDefaultFramerateByScenario()  */
        .max_framerate = 300,
        .mipi_pixel_rate = 454400000,
    },
   .custom4 = {
		 .pclk = 480000000,              //record different mode's pclk
		 .linelength  = 4896,                //record different mode's linelength
		 .framelength = 3209,            //record different mode's framelength
		 .startx = 0,                    //record different mode's startx of grabwindow
		 .starty = 0,                    //record different mode's starty of grabwindow
		 .grabwindow_width  = 4000,      //record different mode's width of grabwindow
		 .grabwindow_height = 3000,      //record different mode's height of grabwindow
		 /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		 .mipi_data_lp2hs_settle_dc = 85,
		 /*   following for GetDefaultFramerateByScenario()  */
		 .max_framerate = 300,
		 .mipi_pixel_rate = 454400000,
    },
	.min_gain = 64, /*1x gain*/
	.max_gain = 1024, /*16x gain*/
	.min_gain_iso = 100,
	.gain_step = 1,
	.gain_type = 2,

	.margin = 5,			//sensor framelength & shutter margin
	.min_shutter = 4,               /*min shutter*/

	/* max framelength by sensor register's limitation */
	.max_frame_length = 0xFFFF-5,//REG0x0202 <=REG0x0340-5//max framelength by sensor register's limitation

	/* shutter delay frame for AE cycle,
	 * 2 frame with ispGain_delay-shut_delay=2-0=2
	 */
	.ae_shut_delay_frame = 0,

	/* sensor gain delay frame for AE cycle,
	 * 2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	 */
	.ae_sensor_gain_delay_frame = 0,

	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.frame_time_delay_frame = 1,//The delay frame of setting frame length
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.sensor_mode_num = 9,	  //support sensor mode num ,don't support Slow motion

	.cap_delay_frame = 1,           //enter capture delay frame num
	.pre_delay_frame = 1,           //enter preview delay frame num
	.video_delay_frame = 1,         //enter video delay frame num
	/* enter high speed video  delay frame num */
	.hs_video_delay_frame = 2,      //enter high speed video  delay frame num
	.slim_video_delay_frame = 2,//enter slim video delay frame num
	.custom1_delay_frame = 2,
	.custom2_delay_frame = 2,	/* Dual camera frame sync control */
	.custom3_delay_frame = 2,
	.custom4_delay_frame = 2,	/* Dual camera frame sync control */
	.isp_driving_current = ISP_DRIVING_8MA,   //8MA
	/* sensor_interface_type */
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	/* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
    .mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
	.mclk = 24,         /*mclk value, suggest 24 or 26 for 24Mhz or 26Mhz*/
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_speed = 400, /*support 1MHz write*/
	/* record sensor support all write id addr,
	 * only supprt 4 must end with 0xff
	 */
	.i2c_addr_table = {0x5a, 0xff},
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x0200,					//current shutter
	.gain = 0x0100,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = 0,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = KAL_FALSE, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x5a,//record current sensor's i2c write id
	.current_ae_effective_frame = 1,
	.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SKIP,
	.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SKIP,
};

/* Sensor output window information*/
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[9] =
{
 { 4208, 3120,	  24,  	0, 4160, 3120, 4160, 3120,   0,	0, 4160, 3120, 	 0, 0, 4160, 3120}, // Preview
 { 4208, 3120,	  24,  	0, 4160, 3120, 4160, 3120,   0,	0, 4160, 3120, 	 0, 0, 4160, 3120}, // capture
 { 4208, 3120,	  24,  390, 4160, 2340, 4160, 2340,  0,	0, 4160, 2340, 	 0, 0, 4160, 2340}, // video
 { 4208, 3120,	824,  840, 2560, 1440, 1280,  720,   0,	0, 1280,  720,   0, 0, 1280,  720}, // hight speed; video not support PDAF
 { 4208, 3120,	184,  480, 3840, 2160, 1920, 1080,   0,	0, 1920, 1080,   0, 0, 1920, 1080}, // slim video; video not support PDAF
 { 4208, 3120,	  24,  	0, 4160, 3120, 4160, 3120,   0,	0, 4160, 3120, 	 0, 0, 4160, 3120}, // custom1
 { 4208, 3120,   0,    0,  4208, 3120, 2104, 1560,   0, 0, 2104, 1560,   0, 0, 2104, 1560}, // custom2; video not support PDAF
 { 4208, 3120,   56, 408,  4096, 2304, 4096, 2304,   0, 0, 4096, 2304,   0, 0, 4096, 2304}, // custom3
 { 4208, 3120,   104, 60,  4000, 3000, 4000, 3000,   0, 0, 4000, 3000,   0, 0, 4000, 3000}, // custom4
};

 static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
 //for 3l6 4160x3120	// 0,1,5	// Preview,Capture,Custom1
{
    .i4OffsetX = 24,
    .i4OffsetY = 24,
    .i4PitchX = 64,
    .i4PitchY = 64,
    .i4PairNum =16,
    .i4SubBlkW =16,
    .i4SubBlkH =16,
    .i4PosL = {{28,31},{44,35},{64,35},{80,31},{32,51},{48,55},{60,55},{76,51},{32,67},{48,63},{60,63},{76,67},{28,87},{44,83},{64,83},{80,87}},
    .i4PosR = {{28,35},{44,39},{64,39},{80,35},{32,47},{48,51},{60,51},{76,47},{32,71},{48,67},{60,67},{76,71},{28,83},{44,79},{64,79},{80,83}},
    .i4BlockNumX = 65,
    .i4BlockNumY = 48,
    .i4Crop = { {24,0}, {24,0}, {24,390}, {824,840}, {184,480}, {24,0}, {0,0}, {56,408}, {104,60}},
    /* 0:IMAGE_NORMAL,1:IMAGE_H_MIRROR,2:IMAGE_V_MIRROR,3:IMAGE_HV_MIRROR */
    .iMirrorFlip = 3,
};



 static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_4160_2340 =
 //for 3l6 4160x2340	// 2		// Video
{
    .i4OffsetX = 24,
    .i4OffsetY = 24,
    .i4PitchX = 64,
    .i4PitchY = 64,
    .i4PairNum =16,
    .i4SubBlkW =16,
    .i4SubBlkH =16,
    .i4PosL = {{28,31},{44,35},{64,35},{80,31},{32,51},{48,55},{60,55},{76,51},{32,67},{48,63},{60,63},{76,67},{28,87},{44,83},{64,83},{80,87}},
    .i4PosR = {{28,35},{44,39},{64,39},{80,35},{32,47},{48,51},{60,51},{76,47},{32,71},{48,67},{60,67},{76,71},{28,83},{44,79},{64,79},{80,83}},
    .i4BlockNumX = 65,
    .i4BlockNumY = 36,
    .i4Crop = { {24,0}, {24,0}, {24,390}, {824,840}, {184,480}, {24,0}, {0,0}, {56,408}, {104,60}},
    /* 0:IMAGE_NORMAL,1:IMAGE_H_MIRROR,2:IMAGE_V_MIRROR,3:IMAGE_HV_MIRROR */
    .iMirrorFlip = 3,
};

 static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_4096_2304 =
 //for 3l6 4096x2304
{
    .i4OffsetX = 24,
    .i4OffsetY = 24,
    .i4PitchX = 64,
    .i4PitchY = 64,
    .i4PairNum =16,
    .i4SubBlkW =16,
    .i4SubBlkH =16,
    .i4PosL = {{28,31},{44,35},{64,35},{80,31},{32,51},{48,55},{60,55},{76,51},{32,67},{48,63},{60,63},{76,67},{28,87},{44,83},{64,83},{80,87}},
    .i4PosR = {{28,35},{44,39},{64,39},{80,35},{32,47},{48,51},{60,51},{76,47},{32,71},{48,67},{60,67},{76,71},{28,83},{44,79},{64,79},{80,83}},
    .i4BlockNumX = 63,
    .i4BlockNumY = 35,
    .i4Crop = { {24,0}, {24,0}, {24,390}, {824,840}, {184,480}, {24,0}, {0,0}, {56,408}, {104,60}},
    /* 0:IMAGE_NORMAL,1:IMAGE_H_MIRROR,2:IMAGE_V_MIRROR,3:IMAGE_HV_MIRROR */
    .iMirrorFlip = 3,
};

 static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_4000_3000 =
 //for 3l6 4000x3000
{
    .i4OffsetX = 24,
    .i4OffsetY = 24,
    .i4PitchX = 64,
    .i4PitchY = 64,
    .i4PairNum =16,
    .i4SubBlkW =16,
    .i4SubBlkH =16,
    .i4PosL = {{28,31},{44,35},{64,35},{80,31},{32,51},{48,55},{60,55},{76,51},{32,67},{48,63},{60,63},{76,67},{28,87},{44,83},{64,83},{80,87}},
    .i4PosR = {{28,35},{44,39},{64,39},{80,35},{32,47},{48,51},{60,51},{76,47},{32,71},{48,67},{60,67},{76,71},{28,83},{44,79},{64,79},{80,83}},
    .i4BlockNumX = 61,
    .i4BlockNumY = 46,
    .i4Crop = { {24,0}, {24,0}, {24,390}, {824,840}, {184,480}, {24,0}, {0,0}, {56,408}, {104,60}},
    /* 0:IMAGE_NORMAL,1:IMAGE_H_MIRROR,2:IMAGE_V_MIRROR,3:IMAGE_HV_MIRROR */
    .iMirrorFlip = 3,
};

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static kal_uint16 read_cmos_sensor_byte(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
    char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
	return get_byte;
}
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_byte(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {
		(char)(addr >> 8), (char)(addr & 0xFF),
		(char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}
#if 0
static kal_uint16 read_cmos_sensor_16_16(kal_uint32 addr)
{
	kal_uint16 get_byte= 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};
	 /*kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte << 8) & 0xff00) | ((get_byte >> 8) & 0x00ff);
}
#endif
static void write_cmos_sensor_16_16(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};
	/* kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_16_8(kal_uint16 addr)
{
	kal_uint16 get_byte= 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};
	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);  Add this func to set i2c speed by each sensor*/
	iReadRegI2C(pusendcmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_16_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static bool framecount_ff;
static int long_shutter_mode;
static void check_framecnt(void)
{
	unsigned int framecnt = 0;

	framecnt = read_cmos_sensor_byte(0x0005);

	if(framecnt == 0xFF){
		framecount_ff =1;
        }else{
		framecount_ff =0;
        }
}

static void check_streamoff(void)
{
	unsigned int i = 0;
	int timeout = (150000 / imgsensor.current_fps) + 1;

	mdelay(3);
	for (i = 0; i < timeout; i++) {
		if (read_cmos_sensor_byte(0x0005) != 0xFF)
			mdelay(5);
		else
			break;
	}
	LOG_INF("%s exit!\n", __func__);
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable) {
		write_cmos_sensor_byte(0x3C1E, 0x01);
		write_cmos_sensor(0x0100, 0x0103);
		write_cmos_sensor_byte(0x3C1E, 0X00);
	} else {
		write_cmos_sensor(0x0100, 0x0003);
		check_streamoff();
}
	return ERROR_NONE;
}

static kal_uint16 s5k3l6_table_write_cmos_sensor(
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
		if ((I2C_BUFFER_LEN - tosend) < 4 ||
			len == IDX ||
			addr != addr_last) {
			LOG_DBG("s5k3l6 table_write_cmos_sensor multi write tosend=%d, len=%d, IDX=%d, addr=%x, addr_last=%x\n", tosend, len, IDX, addr, addr_last);
			iBurstWriteReg_multi(puSendCmd, tosend,
				imgsensor.i2c_write_id,
				4, imgsensor_info.i2c_speed);

			tosend = 0;
		}
#else
		iWriteRegI2C(puSendCmd, 4, imgsensor.i2c_write_id);
		tosend = 0;
#endif
	}
	return 0;
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);

	write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	write_cmos_sensor(0x0342, imgsensor.line_length & 0xFFFF);
}	/*	set_dummy  */

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable(%d)\n",
		framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 /
		imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

#if 0
static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	// Update Shutter
	write_cmos_sensor(0x0202, (shutter) & 0xFFFF);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

	//LOG_INF("frame_length = %d ", frame_length);

}	/*	write_shutter  */
#endif

static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
	unsigned long flags;
	kal_int32 dummy_line = 0;
    kal_uint16 realtime_fps = 0;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	spin_lock(&imgsensor_drv_lock);
	/*Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	/*  */
	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    set_dummy()	;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	// Update Shutter
	write_cmos_sensor(0x0202, (shutter) & 0xFFFF);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

	//LOG_INF("frame_length = %d ", frame_length);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
	//gain = 64 = 1x real gain.
	reg_gain = gain/2;
	//reg_gain = reg_gain & 0xFFFF;
	//return (kal_uint16)reg_gain;
	return reg_gain;
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

	/*LOG_INF("set_gain %d\n", gain);*/
	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_ERR("Error gain setting");
		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 16 * BASEGAIN)
			gain = 16 * BASEGAIN;
	}

    reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0204, (reg_gain&0xFFFF));
	return gain;
}	/*	set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	spin_lock(&imgsensor_drv_lock);
    imgsensor.mirror= image_mirror;
    spin_unlock(&imgsensor_drv_lock);
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor_byte(0x0101,0X00); //GR
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor_byte(0x0101,0X01); //R
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor_byte(0x0101,0X02); //B
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor_byte(0x0101,0X03); //GB
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
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/


kal_uint16 addr_data_pair_init_s5k3l6_carr[] = {
	0x0A02, 0x3400,
	0x3084, 0x1314,
	0x3266, 0x0001,
	0x3242, 0x2020,
	0x306A, 0x2F4C,
	0x306C, 0xCA01,
	0x307A, 0x0D20,
	0x309E, 0x002D,
	0x3072, 0x0013,
	0x3074, 0x0977,
	0x3076, 0x9411,
	0x3024, 0x0016,
    0x3070, 0x3D00,
	0x3002, 0x0E00,
	0x3006, 0x1000,
	0x300A, 0x0C00,
	0x3010, 0x0400,
	0x3018, 0xC500,
	0x303A, 0x0204,
	0x3452, 0x0001,
	0x3454, 0x0001,
	0x3456, 0x0001,
	0x3458, 0x0001,
	0x345a, 0x0002,
	0x345C, 0x0014,
	0x345E, 0x0002,
	0x3460, 0x0014,
	0x3464, 0x0006,
	0x3466, 0x0012,
	0x3468, 0x0012,
	0x346A, 0x0012,
	0x346C, 0x0012,
	0x346E, 0x0012,
	0x3470, 0x0012,
	0x3472, 0x0008,
	0x3474, 0x0004,
	0x3476, 0x0044,
	0x3478, 0x0004,
	0x347A, 0x0044,
	0x347E, 0x0006,
	0x3480, 0x0010,
	0x3482, 0x0010,
	0x3484, 0x0010,
	0x3486, 0x0010,
	0x3488, 0x0010,
	0x348A, 0x0010,
	0x348E, 0x000C,
	0x3490, 0x004C,
	0x3492, 0x000C,
	0x3494, 0x004C,
	0x3496, 0x0020,
	0x3498, 0x0006,
	0x349A, 0x0008,
	0x349C, 0x0008,
	0x349E, 0x0008,
	0x34A0, 0x0008,
	0x34A2, 0x0008,
	0x34A4, 0x0008,
	0x34A8, 0x001A,
	0x34AA, 0x002A,
	0x34AC, 0x001A,
	0x34AE, 0x002A,
	0x34B0, 0x0080,
	0x34B2, 0x0006,
	0x32A2, 0x0000,
	0x32A4, 0x0000,
	0x32A6, 0x0000,
	0x32A8, 0x0000,
	0x3066, 0x7E00,
	0x3004, 0x0800,
};

static void sensor_init(void)
{
  LOG_INF("%s enter\n",__func__);
  write_cmos_sensor(0x0100, 0x0000);
  write_cmos_sensor(0x0000, 0x0060);
  write_cmos_sensor(0x0000, 0x30C6);
  mdelay(3);
  s5k3l6_table_write_cmos_sensor(
	  addr_data_pair_init_s5k3l6_carr,
	  sizeof(addr_data_pair_init_s5k3l6_carr) / sizeof(kal_uint16));
} 

kal_uint16 addr_data_pair_preview_s5k3l6_carr[] = {
0x0344, 0x0020,
0x0346, 0x0008,
0x0348, 0x105F,
0x034A, 0x0C37,
0x034C, 0x1040,
0x034E, 0x0C30,
0x0900, 0x0000,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0001,
0x0386, 0x0001,
0x0114, 0x0330,
0x0110, 0x0002,
0x0136, 0x1800,
0x0304, 0x0004,
0x0306, 0x0078,
0x3C1E, 0x0000,
0x030C, 0x0003,
0x030E, 0x0047,
0x3C16, 0x0000,
0x0300, 0x0006,
0x0342, 0x1320,
0x0340, 0x0CBC,
0x38C4, 0x0009,
0x38D8, 0x002A,
0x38DA, 0x000A,
0x38DC, 0x000B,
0x38C2, 0x000A,
0x38C0, 0x000F,
0x38D6, 0x000A,
0x38D4, 0x0009,
0x38B0, 0x000F,
0x3932, 0x1800,
0x3938, 0x000C,
0x0820, 0x04b0,
0x380C, 0x00A0,
0x3064, 0xFFCF,
0x309C, 0x0640,
0x3090, 0x8800,
0x3238, 0x000C,
0x314A, 0x5F00,
0x3300, 0x0000,
0x3400, 0x0000,
0x3402, 0x4E42,
0x32B2, 0x0006,
0x32B4, 0x0006,
0x32B6, 0x0006,
0x32B8, 0x0006,
0x3C34, 0x0048,
0x3C36, 0x3000,
0x3C38, 0x0020,
0x393E, 0x4000,
0x303A, 0x0204,
0x3034, 0x4B00,
0x3036, 0xFE29,
0x3032, 0x4800,
0x320E, 0x049C,
};

static void preview_setting(void)
{
  LOG_INF("%s enter\n",__func__);
  s5k3l6_table_write_cmos_sensor(
  	addr_data_pair_preview_s5k3l6_carr,
  	sizeof(addr_data_pair_preview_s5k3l6_carr) / sizeof(kal_uint16));
}

kal_uint16 addr_data_pair_capture30_s5k3l6_carr[] = {
0x0344, 0x0020,
0x0346, 0x0008,
0x0348, 0x105F,
0x034A, 0x0C37,
0x034C, 0x1040,
0x034E, 0x0C30,
0x0900, 0x0000,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0001,
0x0386, 0x0001,
0x0114, 0x0330,
0x0110, 0x0002,
0x0136, 0x1800,
0x0304, 0x0004,
0x0306, 0x0078,
0x3C1E, 0x0000,
0x030C, 0x0003,
0x030E, 0x0047,
0x3C16, 0x0000,
0x0300, 0x0006,
0x0342, 0x1320,
0x0340, 0x0CBC,
0x38C4, 0x0009,
0x38D8, 0x002A,
0x38DA, 0x000A,
0x38DC, 0x000B,
0x38C2, 0x000A,
0x38C0, 0x000F,
0x38D6, 0x000A,
0x38D4, 0x0009,
0x38B0, 0x000F,
0x3932, 0x1800,
0x3938, 0x000C,
0x0820, 0x04b0,
0x380C, 0x00A0,
0x3064, 0xFFCF,
0x309C, 0x0640,
0x3090, 0x8800,
0x3238, 0x000C,
0x314A, 0x5F00,
0x3300, 0x0000,
0x3400, 0x0000,
0x3402, 0x4E42,
0x32B2, 0x0006,
0x32B4, 0x0006,
0x32B6, 0x0006,
0x32B8, 0x0006,
0x3C34, 0x0048,
0x3C36, 0x3000,
0x3C38, 0x0020,
0x393E, 0x4000,
0x303A, 0x0204,
0x3034, 0x4B00,
0x3036, 0xFE29,
0x3032, 0x4800,
0x320E, 0x049C,
};

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("%s enter\n",__func__);
	s5k3l6_table_write_cmos_sensor(
		addr_data_pair_capture30_s5k3l6_carr,
		sizeof(addr_data_pair_capture30_s5k3l6_carr) / sizeof(kal_uint16));
}

kal_uint16 addr_data_pair_video_s5k3l6_carr[] = {
0x0344, 0x0020,
0x0346, 0x018E,
0x0348, 0x105F,
0x034A, 0x0AB1,
0x034C, 0x1040,
0x034E, 0x0924,
0x0900, 0x0000,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0001,
0x0386, 0x0001,
0x0114, 0x0330,
0x0110, 0x0002,
0x0136, 0x1800,
0x0304, 0x0004,
0x0306, 0x0077,
0x3C1E, 0x0000,
0x030C, 0x0003,
0x030E, 0x0047,
0x3C16, 0x0000,
0x0300, 0x0006,
0x0342, 0x1320,
0x0340, 0x0C89,
0x38C4, 0x0009,
0x38D8, 0x002A,
0x38DA, 0x000A,
0x38DC, 0x000B,
0x38C2, 0x000A,
0x38C0, 0x000F,
0x38D6, 0x000A,
0x38D4, 0x0009,
0x38B0, 0x000F,
0x3932, 0x1800,
0x3938, 0x000C,
0x0820, 0x04b0,
0x380C, 0x00A0,
0x3064, 0xFFCF,
0x309C, 0x0640,
0x3090, 0x8800,
0x3238, 0x000C,
0x314A, 0x5F00,
0x3300, 0x0000,
0x3400, 0x0000,
0x3402, 0x4E42,
0x32B2, 0x0006,
0x32B4, 0x0006,
0x32B6, 0x0006,
0x32B8, 0x0006,
0x3C34, 0x0048,
0x3C36, 0x3000,
0x3C38, 0x0020,
0x393E, 0x4000,
0x303A, 0x0204,
0x3034, 0x4B01,
0x3036, 0x0029,
0x3032, 0x4800,
0x320E, 0x049E,
};

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! %s currefps:%d\n",__func__,currefps);
	s5k3l6_table_write_cmos_sensor(
		addr_data_pair_video_s5k3l6_carr,
		sizeof(addr_data_pair_video_s5k3l6_carr) / sizeof(kal_uint16));
}

kal_uint16 addr_data_pair_hs_video_s5k3l6_carr[] = {
0x0344, 0x0340,
0x0346, 0x0350,
0x0348, 0x0D3F,
0x034A, 0x08EF,
0x034C, 0x0500,
0x034E, 0x02D0,
0x0900, 0x0122,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0001,
0x0386, 0x0003,
0x0114, 0x0330,
0x0110, 0x0002,
0x0136, 0x1800,
0x0304, 0x0004,
0x0306, 0x0078,
0x3C1E, 0x0000,
0x030C, 0x0003,
0x030E, 0x0059,
0x3C16, 0x0002,
0x0300, 0x0006,
0x0342, 0x1320,
0x0340, 0x0330,
0x38C4, 0x0002,
0x38D8, 0x0008,
0x38DA, 0x0003,
0x38DC, 0x0004,
0x38C2, 0x0003,
0x38C0, 0x0001,
0x38D6, 0x0003,
0x38D4, 0x0002,
0x38B0, 0x0004,
0x3932, 0x1800,
0x3938, 0x000C,
0x0820, 0x0162,
0x380C, 0x003D,
0x3064, 0xFFCF,
0x309C, 0x0640,
0x3090, 0x8000,
0x3238, 0x000B,
0x314A, 0x5F02,
0x3300, 0x0000,
0x3400, 0x0000,
0x3402, 0x4E46,
0x32B2, 0x0008,
0x32B4, 0x0008,
0x32B6, 0x0008,
0x32B8, 0x0008,
0x3C34, 0x0048,
0x3C36, 0x3000,
0x3C38, 0x0024,
0x393E, 0x4000,
0x303A, 0x0204,
0x3034, 0x4B01,
0x3036, 0x0029,
0x3032, 0x4800,
0x320E, 0x049E,
};

static void hs_video_setting(void)
{
	LOG_INF("%s enter\n",__func__);
	s5k3l6_table_write_cmos_sensor(
		addr_data_pair_hs_video_s5k3l6_carr,
		sizeof(addr_data_pair_hs_video_s5k3l6_carr) / sizeof(kal_uint16));
}

kal_uint16 addr_data_pair_slim_video_s5k3l6_carr[] = {
0x0344, 0x00C0,
0x0346, 0x01E8,
0x0348, 0x0FBF,
0x034A, 0x0A57,
0x034C, 0x0780,
0x034E, 0x0438,
0x0900, 0x0122,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0001,
0x0386, 0x0003,
0x0114, 0x0330,
0x0110, 0x0002,
0x0136, 0x1800,
0x0304, 0x0004,
0x0306, 0x0078,
0x3C1E, 0x0000,
0x030C, 0x0003,
0x030E, 0x0045,
0x3C16, 0x0001,
0x0300, 0x0006,
0x0342, 0x1220,
0x0340, 0x06bc,
0x38C4, 0x0004,
0x38D8, 0x0010,
0x38DA, 0x0005,
0x38DC, 0x0005,
0x38C2, 0x0005,
0x38C0, 0x0004,
0x38D6, 0x0004,
0x38D4, 0x0004,
0x38B0, 0x0007,
0x3932, 0x1000,
0x3938, 0x000C,
0x0820, 0x0228,
0x380C, 0x003B,
0x3064, 0xFFCF,
0x309C, 0x0640,
0x3090, 0x8000,
0x3238, 0x000B,
0x314A, 0x5F02,
0x3300, 0x0000,
0x3400, 0x0000,
0x3402, 0x4E46,
0x32B2, 0x0008,
0x32B4, 0x0008,
0x32B6, 0x0008,
0x32B8, 0x0008,
0x3C34, 0x0048,
0x3C36, 0x3000,
0x3C38, 0x0020,
0x393E, 0x4000,
0x303A, 0x0002,
0x3034, 0x4B00,
0x3036, 0xF729,
0x3032, 0x3500,
0x320E, 0x0480,
};

static void slim_video_setting(void)
{
	LOG_INF("%s enter\n",__func__);
	s5k3l6_table_write_cmos_sensor(
		addr_data_pair_slim_video_s5k3l6_carr,
		sizeof(addr_data_pair_slim_video_s5k3l6_carr) / sizeof(kal_uint16));
}

kal_uint16 addr_data_pair_custom1_s5k3l6_carr[] = {
0x0344, 0x0020,
0x0346, 0x0008,
0x0348, 0x105F,
0x034A, 0x0C37,
0x034C, 0x1040,
0x034E, 0x0C30,
0x0900, 0x0000,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0001,
0x0386, 0x0001,
0x0114, 0x0330,
0x0110, 0x0002,
0x0136, 0x1800,
0x0304, 0x0004,
0x0306, 0x0078,
0x3C1E, 0x0000,
0x030C, 0x0003,
0x030E, 0x0047,
0x3C16, 0x0000,
0x0300, 0x0006,
0x0342, 0x1320,
0x0340, 0x0FF4,
0x38C4, 0x0009,
0x38D8, 0x002A,
0x38DA, 0x000A,
0x38DC, 0x000B,
0x38C2, 0x000A,
0x38C0, 0x000F,
0x38D6, 0x000A,
0x38D4, 0x0009,
0x38B0, 0x000F,
0x3932, 0x1800,
0x3938, 0x000C,
0x0820, 0x04b0,
0x380C, 0x00A0,
0x3064, 0xFFCF,
0x309C, 0x0640,
0x3090, 0x8800,
0x3238, 0x000C,
0x314A, 0x5F00,
0x3300, 0x0000,
0x3400, 0x0000,
0x3402, 0x4E42,
0x32B2, 0x0006,
0x32B4, 0x0006,
0x32B6, 0x0006,
0x32B8, 0x0006,
0x3C34, 0x0048,
0x3C36, 0x3000,
0x3C38, 0x0020,
0x393E, 0x4000,
0x303A, 0x0204,
0x3034, 0x4B00,
0x3036, 0xFE29,
0x3032, 0x4800,
0x320E, 0x049C,

};

static void custom1_setting(void)
{
	LOG_INF("%s enter\n",__func__);
	s5k3l6_table_write_cmos_sensor(
		addr_data_pair_custom1_s5k3l6_carr,
		sizeof(addr_data_pair_custom1_s5k3l6_carr) / sizeof(kal_uint16));
}

kal_uint16 addr_data_pair_custom2_s5k3l6_carr[] = {
0x0344, 0x0008,
0x0346, 0x0008,
0x0348, 0x1077,
0x034A, 0x0C37,
0x034C, 0x0838,
0x034E, 0x0618,
0x0900, 0x0122,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0001,
0x0386, 0x0003,
0x0114, 0x0330,
0x0110, 0x0002,
0x0136, 0x1800,
0x0304, 0x0004,
0x0306, 0x0078,
0x3C1E, 0x0000,
0x030C, 0x0003,
0x030E, 0x0047,
0x3C16, 0x0001,
0x0300, 0x0006,
0x0342, 0x1320,
0x0340, 0x0660,
0x38C4, 0x0004,
0x38D8, 0x0011,
0x38DA, 0x0005,
0x38DC, 0x0005,
0x38C2, 0x0005,
0x38C0, 0x0004,
0x38D6, 0x0004,
0x38D4, 0x0004,
0x38B0, 0x0007,
0x3932, 0x1000,
0x3938, 0x000C,
0x0820, 0x0238,
0x380C, 0x0049,
0x3064, 0xFFCF,
0x309C, 0x0640,
0x3090, 0x8000,
0x3238, 0x000B,
0x314A, 0x5F02,
0x3300, 0x0000,
0x3400, 0x0000,
0x3402, 0x4E46,
0x32B2, 0x0008,
0x32B4, 0x0008,
0x32B6, 0x0008,
0x32B8, 0x0008,
0x3C34, 0x0048,
0x3C36, 0x3000,
0x3C38, 0x0020,
0x393E, 0x4000,
0x303A, 0x0204,
0x3034, 0x4B01,
0x3036, 0x0029,
0x3032, 0x4800,
0x320E, 0x049E,
};

static void custom2_setting(void)
{
	LOG_INF("%s enter\n",__func__);
	s5k3l6_table_write_cmos_sensor(
		addr_data_pair_custom2_s5k3l6_carr,
		sizeof(addr_data_pair_custom2_s5k3l6_carr) / sizeof(kal_uint16));
}

kal_uint16 addr_data_pair_custom3_s5k3l6_carr[] = {
0x0344, 0x0028,
0x0346, 0x01A0,
0x0348, 0x1027,
0x034A, 0x0A9F,
0x034C, 0x1000,
0x034E, 0x0900,
0x0900, 0x0000,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0001,
0x0386, 0x0001,
0x0114, 0x0330,
0x0110, 0x0002,
0x0136, 0x1800,
0x0304, 0x0004,
0x0306, 0x0077,
0x3C1E, 0x0000,
0x030C, 0x0003,
0x030E, 0x0047,
0x3C16, 0x0000,
0x0300, 0x0006,
0x0342, 0x1320,
0x0340, 0x0C89,
0x38C4, 0x0009,
0x38D8, 0x002A,
0x38DA, 0x000A,
0x38DC, 0x000B,
0x38C2, 0x000A,
0x38C0, 0x000F,
0x38D6, 0x000A,
0x38D4, 0x0009,
0x38B0, 0x000F,
0x3932, 0x1800,
0x3938, 0x000C,
0x0820, 0x04b0,
0x380C, 0x00A0,
0x3064, 0xFFCF,
0x309C, 0x0640,
0x3090, 0x8800,
0x3238, 0x000C,
0x314A, 0x5F00,
0x3300, 0x0000,
0x3400, 0x0000,
0x3402, 0x4E42,
0x32B2, 0x0006,
0x32B4, 0x0006,
0x32B6, 0x0006,
0x32B8, 0x0006,
0x3C34, 0x0048,
0x3C36, 0x3000,
0x3C38, 0x0020,
0x393E, 0x4000,
0x303A, 0x0204,
0x3034, 0x4B01,
0x3036, 0x0029,
0x3032, 0x4800,
0x320E, 0x049E,
};

static void custom3_setting(void)
{
	LOG_INF("%s enter\n",__func__);
	s5k3l6_table_write_cmos_sensor(
		addr_data_pair_custom3_s5k3l6_carr,
		sizeof(addr_data_pair_custom3_s5k3l6_carr) / sizeof(kal_uint16));
}

kal_uint16 addr_data_pair_custom4_s5k3l6_carr[] = {
0x0344, 0x0070,
0x0346, 0x0044,
0x0348, 0x100F,
0x034A, 0x0BFB,
0x034C, 0x0FA0,
0x034E, 0x0BB8,
0x0900, 0x0000,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0001,
0x0386, 0x0001,
0x0114, 0x0330,
0x0110, 0x0002,
0x0136, 0x1800,
0x0304, 0x0004,
0x0306, 0x0077,
0x3C1E, 0x0000,
0x030C, 0x0003,
0x030E, 0x0047,
0x3C16, 0x0000,
0x0300, 0x0006,
0x0342, 0x1320,
0x0340, 0x0C89,
0x38C4, 0x0009,
0x38D8, 0x002A,
0x38DA, 0x000A,
0x38DC, 0x000B,
0x38C2, 0x000A,
0x38C0, 0x000F,
0x38D6, 0x000A,
0x38D4, 0x0009,
0x38B0, 0x000F,
0x3932, 0x1800,
0x3938, 0x000C,
0x0820, 0x04b0,
0x380C, 0x00A0,
0x3064, 0xFFCF,
0x309C, 0x0640,
0x3090, 0x8800,
0x3238, 0x000C,
0x314A, 0x5F00,
0x3300, 0x0000,
0x3400, 0x0000,
0x3402, 0x4E42,
0x32B2, 0x0006,
0x32B4, 0x0006,
0x32B6, 0x0006,
0x32B8, 0x0006,
0x3C34, 0x0048,
0x3C36, 0x3000,
0x3C38, 0x0020,
0x393E, 0x4000,
0x303A, 0x0204,
0x3034, 0x4B01,
0x3036, 0x0029,
0x3032, 0x4800,
0x320E, 0x049E,
};

static void custom4_setting(void)
{
	LOG_INF("%s enter\n",__func__);
	s5k3l6_table_write_cmos_sensor(
		addr_data_pair_custom4_s5k3l6_carr,
		sizeof(addr_data_pair_custom4_s5k3l6_carr) / sizeof(kal_uint16));
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
	kal_uint16 realtime_fps = 0;
	//kal_uint32 frame_length = 0;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	//write_shutter(shutter);
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
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
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	//shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		LOG_DBG("set shutter autoflicker_en\n");
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			// Extend frame length
			write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
		}
	} else {
		// Extend frame length
		write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	if (shutter > 65530) {  //linetime=10160/960000000<< maxshutter=3023622-line=32s
		/*enter long exposure mode */
		kal_uint32 exposure_time;
		kal_uint32 new_framelength;
		kal_uint32 long_shutter;
		kal_uint32 temp1_030F = 0, temp2_0821 = 0, temp3_38c3 = 0, temp4_0342 = 0, temp5_0343 = 0;
		//kal_uint32 temp6_38C2 = 0;
		kal_uint32 long_shutter_linelenght = 0;
		int dat, dat1;
		LOG_INF("enter long exposure mode\n");

		bIsLongExposure = KAL_TRUE;

		//streaming_control(KAL_FALSE);
		//exposure_time = shutter*imgsensor_info.cap.linelength/960000;//ms
		//exposure_time = shutter/1000*4896/480;//ms
		exposure_time = (shutter * 1) / 100;//ms

		if (exposure_time < 6500) {
			temp1_030F = 0x78;
			temp2_0821 = 0x78;
			temp3_38c3 = 0x06;
			temp4_0342 = 0x13;
			temp5_0343 = 0x20;
			//temp6_38C2 = 0x10;
			long_shutter_linelenght = 4896;
			//imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
			//imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
			//imgsensor.current_ae_effective_frame = 1;
			long_shutter_mode = 0;
			LOG_INF("7s exposure_time = %d\n", exposure_time);
		} else if (6500 <= exposure_time  && exposure_time <= 22200) {
			temp1_030F = 0x78;
			temp2_0821 = 0x78;
			temp3_38c3 = 0x06;
			temp4_0342 = 0x3f;
			temp5_0343 = 0x90;
			//temp6_38C2 = 0x10;
			long_shutter_linelenght = 16272;
			//imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
			//imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
			//imgsensor.current_ae_effective_frame = 1;
			long_shutter_mode = 1;
			LOG_INF("7s  22.2s exposure_time = %d\n",exposure_time);
		} else if (22200 < exposure_time  && exposure_time <= 23500) {
			temp1_030F = 0x64;
			temp2_0821 = 0x64;
			temp3_38c3 = 0x05;
			temp4_0342 = 0x43;
			temp5_0343 = 0x40;
			long_shutter_linelenght = 17216;
			LOG_INF("22.2s	23.5s exposure_time = %d\n",exposure_time);
		} else if (23500 < exposure_time  && exposure_time <= 32000) {
			temp1_030F = 0x78;
			temp2_0821 = 0x78;
			temp3_38c3 = 0x06;
			temp4_0342 = 0x5B;
			temp5_0343 = 0x90;
			long_shutter_linelenght = 23439;
			LOG_INF("23.5s to 32s exposure_time = %d\n",exposure_time);
		}else{
			long_shutter_linelenght = 4896;
			LOG_INF("failed!!! long_shutter_linelenght cannot divide by zero");
		}

		//long_shutter = (exposure_time*pclk-256)/lineleght/64 = (shutter*linelength-256)/linelength/pow_shift
		long_shutter = (shutter * 48) / (long_shutter_linelenght / 10); //line_lengthpck has been changed,longshuter's linelength need to be calibration again
		new_framelength = long_shutter + 2;
		LOG_INF("long exposure Shutter = %d\n",shutter);
		LOG_INF("Calc long exposure_time=%dms, long_shutter=%d, framelength=%d.\n", exposure_time, long_shutter, new_framelength);
		imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		imgsensor.current_ae_effective_frame = 1;

		check_framecnt();
		streaming_control(KAL_FALSE);
		//write_cmos_sensor(0x0100, 0x0003); /*stream off */
		check_streamoff();
		if(framecount_ff)
			mdelay(35);
		//mdelay(200);
		write_cmos_sensor_16_8(0x0307, 0x60);
		write_cmos_sensor_16_8(0x3C1F, 0x03);
		write_cmos_sensor_16_8(0x030D, 0x03);
		write_cmos_sensor_16_8(0x030E, 0x00);
		write_cmos_sensor_16_8(0x030F, temp1_030F);
		write_cmos_sensor_16_8(0x3C17, 0x04);
		write_cmos_sensor_16_8(0x0820, 0x00);
		write_cmos_sensor_16_8(0x0821, temp2_0821);
		write_cmos_sensor_16_8(0x38C5, 0x03);
		write_cmos_sensor_16_8(0x38D9, 0x00);
		write_cmos_sensor_16_8(0x38DB, 0x08);
		write_cmos_sensor_16_8(0x38DD, 0x13);
		write_cmos_sensor_16_8(0x38C3, temp3_38c3);
		write_cmos_sensor_16_8(0x38C1, 0x00);
		write_cmos_sensor_16_8(0x38D7, 0x0F);
		write_cmos_sensor_16_8(0x38D5, 0x03);
		write_cmos_sensor_16_8(0x38B1, 0x01);
		write_cmos_sensor_16_8(0x3932, 0x20);
		write_cmos_sensor_16_8(0x3938, 0x20);

		write_cmos_sensor_16_8(0x0340, (new_framelength & 0xFF00) >> 8);
		write_cmos_sensor_16_8(0x0341, (new_framelength & 0x00FF)); //00000111
		write_cmos_sensor_16_8(0x0342, temp4_0342);
		write_cmos_sensor_16_8(0x0343, temp5_0343);
		//write_cmos_sensor_16_8(0x38C2, temp6_38C2);
		write_cmos_sensor_16_8(0x0202, (long_shutter & 0xFF00) >> 8);
		write_cmos_sensor_16_8(0x0203, (long_shutter & 0x00FF));

#if 0
		write_cmos_sensor_16_8(0x3C1E, 0x01);
		write_cmos_sensor(0x0100, 0x0103);
		write_cmos_sensor_16_8(0x3C1E, 0x00);
#else
		streaming_control(KAL_TRUE);
#endif

		dat = read_cmos_sensor_16_8(0x0342);
		dat1 = read_cmos_sensor_16_8(0x0343);
		LOG_INF("long exposure dat = %x, dat1 = %x\n", dat, dat1);
		LOG_INF("long exposure (new_framelength&0xFF00)>>8 %x\n", (new_framelength & 0xFF00) >> 8);
		LOG_INF("long exposure (new_framelength&0x00FF) %x\n", (new_framelength & 0x00FF));
		//write_cmos_sensor_16_8(0x0100, 0x01); /*stream on */
		/*streaming_control(KAL_TRUE);*/

		//LOG_INF("pengzuo long exposure	stream on-\n");
	} else {
		// Update Shutter
		if (bIsLongExposure == KAL_TRUE) {
			bIsLongExposure = KAL_FALSE;

			LOG_INF("[Exit long shutter + ]  shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
			//write_cmos_sensor(0x0100, 0x0003); /*stream off */
			//mdelay(200);
			check_framecnt();
			streaming_control(KAL_FALSE);
			check_streamoff();
			if(framecount_ff)
			{
				if(long_shutter_mode == 0)
					mdelay(250);
				else
					mdelay(1100);
			}

			//mdelay(50);
			write_cmos_sensor_16_8(0x0307, 0x78);
			write_cmos_sensor_16_8(0x3C1F, 0x00);
			write_cmos_sensor_16_8(0x030D, 0x03);
			write_cmos_sensor_16_8(0x030E, 0x00);
			write_cmos_sensor_16_8(0x030F, 0x4B);
			write_cmos_sensor_16_8(0x3C17, 0x00);
			write_cmos_sensor_16_8(0x0820, 0x04);
			write_cmos_sensor_16_8(0x0821, 0xB0);
			write_cmos_sensor_16_8(0x38C5, 0x09);
			write_cmos_sensor_16_8(0x38D9, 0x2A);
			write_cmos_sensor_16_8(0x38DB, 0x0A);
			write_cmos_sensor_16_8(0x38DD, 0x0B);
			write_cmos_sensor_16_8(0x38C3, 0x0A);
			write_cmos_sensor_16_8(0x38C1, 0x0F);
			write_cmos_sensor_16_8(0x38D7, 0x0A);
			write_cmos_sensor_16_8(0x38D5, 0x09);
			write_cmos_sensor_16_8(0x38B1, 0x0F);
			write_cmos_sensor_16_8(0x3932, 0x18);
			write_cmos_sensor_16_8(0x3938, 0x00);

			write_cmos_sensor_16_8(0x0340, 0x0C);
			write_cmos_sensor_16_8(0x0341, 0xBC);
			write_cmos_sensor_16_8(0x0342, 0x13);
			write_cmos_sensor_16_8(0x0343, 0x20);
			write_cmos_sensor_16_8(0x0202, 0x03);
			write_cmos_sensor_16_8(0x0203, 0xDE);

			//write_cmos_sensor_16_8(0x3C1E, 0x01);
			//write_cmos_sensor_16_8(0x0100, 0x01);
			//write_cmos_sensor_16_8(0x3C1E, 0x00);


			//imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
			//imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
			//imgsensor.current_ae_effective_frame = 2;
			streaming_control(KAL_TRUE);
			LOG_INF("Exit long shutter = %x, dat1 = %x\n", read_cmos_sensor_16_8(0x0342),read_cmos_sensor_16_8(0x0343));
			LOG_INF("[Exit long shutter - ] shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

		} else {
			shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
			//shutter = 3260-5;
			write_cmos_sensor_16_16(0x0202, shutter & 0xFFFF);
			LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
			LOG_INF("Exit! shutter linelength = %x, dat1 = %x\n", read_cmos_sensor_16_8(0x0342),read_cmos_sensor_16_8(0x0343));
		}
	}
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
			*sensor_id = ((read_cmos_sensor_8(0x0000) << 8) | read_cmos_sensor_8(0x0001));
			LOG_INF("i2c read sensor id : 0x%x\n", *sensor_id);
			if (*sensor_id == imgsensor_info.sensor_id) {
				*sensor_id = CARR_S5K3L6_SENSOR_ID ;
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
			retry--;
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
	kal_uint16 sensor_id = 0;

	LOG_INF("%s", __func__);

	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 * we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor_8(0x0000) << 8) | read_cmos_sensor_8(0x0001));

			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}

			LOG_INF("fenneike Read sensor id fail, id: 0x%x\n",
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
	sensor_init();

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
	imgsensor.test_pattern = 0;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}				/*      open  */



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
	streaming_control(0);
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
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(imgsensor.mirror);
	//mdelay(10);
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
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;

	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);
	//mdelay(10);
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
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

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
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}

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
}   /*  Custom3	*/
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

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;

    sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth  = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight  = imgsensor_info.slim_video.grabwindow_height;

    sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height  = imgsensor_info.custom1.grabwindow_height;

    sensor_resolution->SensorCustom2Width  = imgsensor_info.custom2.grabwindow_width;
    sensor_resolution->SensorCustom2Height  = imgsensor_info.custom2.grabwindow_height;

    sensor_resolution->SensorCustom3Width  = imgsensor_info.custom3.grabwindow_width;
    sensor_resolution->SensorCustom3Height  = imgsensor_info.custom3.grabwindow_height;

    sensor_resolution->SensorCustom4Width  = imgsensor_info.custom4.grabwindow_width;
    sensor_resolution->SensorCustom4Height  = imgsensor_info.custom4.grabwindow_height;

    return ERROR_NONE;
}/* get_resolution */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

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
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

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

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;                /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;   /* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	sensor_info->PDAF_Support = 1;	//0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;  // 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;

	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

			break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;

            break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;
			break;

		case MSDK_SCENARIO_ID_CUSTOM4:
			sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom4.mipi_data_lp2hs_settle_dc;
			break;

        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
        }

        return ERROR_NONE;
}/*get_info  */


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
            Custom1(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            Custom2(image_window, sensor_config_data); // Custom2
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            Custom3(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            Custom4(image_window, sensor_config_data); // Custom2
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
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
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
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
            break; 
		case MSDK_SCENARIO_ID_CUSTOM3:
			frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength) : 0;
			if (imgsensor.dummy_line < 0)
				imgsensor.dummy_line = 0;
			imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break; 
		case MSDK_SCENARIO_ID_CUSTOM4:
			frame_length = imgsensor_info.custom4.pclk / framerate * 10 / imgsensor_info.custom4.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength) ? (frame_length - imgsensor_info.custom4.framelength) : 0;
			if (imgsensor.dummy_line < 0)
				imgsensor.dummy_line = 0;
			imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break; 

		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
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

        default:
            break;
        }

        return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_uint8 modes, struct SET_SENSOR_PATTERN_SOLID_COLOR *pTestpatterndata)
{
    kal_uint16 Color_R, Color_Gr, Color_Gb, Color_B;
    static int cnt = 0;
    Color_R = (pTestpatterndata->COLOR_R >> 16) & 0xFFFF;
    Color_Gr = (pTestpatterndata->COLOR_Gr >> 16) & 0xFFFF;
    Color_B = (pTestpatterndata->COLOR_B >> 16) & 0xFFFF;
    Color_Gb = (pTestpatterndata->COLOR_Gb >> 16) & 0xFFFF;
    pr_info("3l6 set_test_pattern enum: %d\n", modes);
    pr_err("R %x GR %x B %x GB %x",pTestpatterndata->COLOR_R, pTestpatterndata->COLOR_Gr, pTestpatterndata->COLOR_B, pTestpatterndata->COLOR_Gb);
    pr_err("SETTING R %x GR %x B %x GB %x", Color_R, Color_Gr,Color_B,Color_Gb);
	if (modes) {
		write_cmos_sensor(0x6028, 0x4000);
		write_cmos_sensor_16_8(0x0601, modes);
		if (modes == 1 && (pTestpatterndata != NULL)) { //Solid Color
			switch (cnt) {
				case 0:
					Color_R = 0x0000;
					Color_Gr = 0x0000;
					Color_B = 0x0000;
					Color_Gb = 0x0000;
					cnt++;
				break;
				case 1:
					Color_R = 0xffff;
					Color_Gr = 0xffff;
					Color_B = 0xffff;
					Color_Gb = 0xffff;
					cnt++;
				break;
				case 2:
					Color_R = 0x0000;
					Color_Gr = 0x0000;
					Color_B = 0xffff;
					Color_Gb = 0x0000;
					cnt++;
				break;
				case 3:
					Color_R = 0x0000;
					Color_Gr = 0xffff;
					Color_B = 0x0000;
					Color_Gb = 0xffff;
					cnt++;
				break;
				case 4:
					Color_R = 0xffff;
					Color_Gr = 0x0000;
					Color_B = 0x0000;
					Color_Gb = 0x0000;
					cnt++;
				break;
				default: break;
			}

            write_cmos_sensor(0x0602, Color_R);

            write_cmos_sensor(0x0604, Color_Gr);

            write_cmos_sensor(0x0606, Color_B);

            write_cmos_sensor(0x0608, Color_Gb);

        }
    } else {
        write_cmos_sensor(0x0600, 0x0000); /*No pattern*/
    }
    //write_cmos_sensor_16_8(0x3200, 0x00);
	if (cnt > 4)
		cnt = 0;
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = modes;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

static kal_uint32 get_sensor_temperature(void)
{
    INT32 temperature_convert = 25;
    return temperature_convert;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    INT32 *feature_return_para_i32 = (INT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    //unsigned long long *feature_return_para=(unsigned long long *) feature_para;

    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
    struct SET_PD_BLOCK_INFO_T *PDAFinfo;

    LOG_DBG("feature_id = %d\n", feature_id);
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
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((UINT8)*feature_data, (struct SET_SENSOR_PATTERN_SOLID_COLOR *) feature_data+1);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data_16;
            spin_unlock(&imgsensor_drv_lock);
            LOG_INF("current fps :%d\n", imgsensor.current_fps);
            break;
        case SENSOR_FEATURE_SET_HDR:
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data_32;
            spin_unlock(&imgsensor_drv_lock);
            break;
        /******************** PDAF START >>> *********/
		case SENSOR_FEATURE_GET_PDAF_INFO:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%llu\n", *feature_data);
			PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CUSTOM1:	//4160x3120
					memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW: //4160x2340
					memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info_4160_2340,sizeof(struct SET_PD_BLOCK_INFO_T));
					break;			
				case MSDK_SCENARIO_ID_CUSTOM3:  //4096x2304		
					memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info_4096_2304,sizeof(struct SET_PD_BLOCK_INFO_T));
					break;			
				case MSDK_SCENARIO_ID_CUSTOM4: //4000x3000			
					memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info_4000_3000,sizeof(struct SET_PD_BLOCK_INFO_T));
					break;					
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
				case MSDK_SCENARIO_ID_CUSTOM2:	
				default:
					break;
			}
			break;
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu\n", *feature_data);
			//PDAF capacity enable or not, 2p8 only full size support PDAF
			switch (*feature_data) {
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
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				case MSDK_SCENARIO_ID_CUSTOM1:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				case MSDK_SCENARIO_ID_CUSTOM2:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_CUSTOM3:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				case MSDK_SCENARIO_ID_CUSTOM4:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				default:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PDAF_DATA:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
			//s5k3l6_read_otp_pdaf_data((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
			break;
		case SENSOR_FEATURE_SET_PDAF:
			LOG_INF("PDAF mode :%d\n", *feature_data_16);
			imgsensor.pdaf_mode = *feature_data_16;
			break;
        case SENSOR_FEATURE_GET_TEMPERATURE_VALUE: 
            *feature_return_para_i32 = get_sensor_temperature();
            *feature_para_len = 4;
        break;
        /******************** PDAF END   <<< *********/
		case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
			{
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					 *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =	imgsensor_info.cap.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.normal_video.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.hs_video.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.slim_video.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_CUSTOM1:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.custom1.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_CUSTOM2:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.custom2.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_CUSTOM3:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.custom3.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_CUSTOM4:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.custom4.mipi_pixel_rate;
					break;

				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.pre.mipi_pixel_rate;
					break;
				default:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
					break;
				}
				LOG_INF("SENSOR_FEATURE_GET_MIPI_PIXEL_RATE :rate:%d\n",*(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
			}
		break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

            wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(struct  SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CUSTOM1:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[5],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CUSTOM2:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[6],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CUSTOM3:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[7],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CUSTOM4:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[8],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;

                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
            break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            /*LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));*/
            break;
        case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
			LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
            streaming_control(KAL_FALSE);
            break;
        case SENSOR_FEATURE_SET_STREAMING_RESUME:
            if (*feature_data != 0)
                set_shutter(*feature_data);
			LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME\n");
            streaming_control(KAL_TRUE);
            break;
		case SENSOR_FEATURE_GET_4CELL_DATA:/*get 4 cell data from eeprom*/
			break;
        case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME: 
            set_shutter_frame_length(
                (UINT16) *feature_data, (UINT16) *(feature_data + 1));
            break;
        case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
            *feature_return_para_32 = imgsensor.current_ae_effective_frame;
            break;
        case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
            memcpy(feature_return_para_32, &imgsensor.ae_frm_mode,sizeof(struct IMGSENSOR_AE_FRM_MODE));
            break;
		case SENSOR_FEATURE_GET_PIXEL_RATE:
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					(imgsensor_info.cap.pclk /
					(imgsensor_info.cap.linelength - 80))*
					imgsensor_info.cap.grabwindow_width;
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					(imgsensor_info.pre.pclk /
					(imgsensor_info.pre.linelength - 80))*
					imgsensor_info.pre.grabwindow_width;
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

                default:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					(imgsensor_info.cap.pclk /
					(imgsensor_info.cap.linelength - 80))*
					imgsensor_info.cap.grabwindow_width;
                    break;
			}
			LOG_INF("SENSOR_FEATURE_GET_PIXEL_RATE :%ld\n", *(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
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
		LOG_INF("SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO scenarioId:%d :%ld\n", *feature_data, *(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
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
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            default:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                            = (imgsensor_info.pre.framelength << 16)
                                     + imgsensor_info.pre.linelength;
                    break;
            }
		    LOG_INF("SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO scenarioId:%d :%ld\n", *feature_data, *(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
            break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/*
		 * 1, if driver support new sw frame sync
		 * set_shutter_frame_length() support third para auto_extend_en
		 */
		*(feature_data + 1) = 1; /* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		LOG_INF("SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO scenarioId:%d\n",imgsensor_info.margin);
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


UINT32 CARR_S5K3L6_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
/*	CARR_S5K3L6_MIPI_RAW_SensorInit	*/
