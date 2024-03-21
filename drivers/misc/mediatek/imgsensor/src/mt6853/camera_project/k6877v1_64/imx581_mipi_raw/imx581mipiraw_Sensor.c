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
 *	 IMX581mipi_Sensor.c
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
#define PFX "IMX581_camera_sensor"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__


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

#include "imx581mipiraw_Sensor.h"
#include "imx581_eeprom.h"
#include "imgsensor_eeprom.h"
#include "imx581_seamless_switch.h"
#include <soc/oplus/system/oplus_project.h>
#define PCB_VERSION_T0   11

#undef OPLUS_FEATURE_CAMERA_COMMON

#define USE_BURST_MODE 1

#define I2C_BUFFER_LEN 765 /* trans# max is 255, each 3 bytes */

#define LONG_EXP 1

static kal_uint8 qsc_flag;
static kal_uint8 otp_flag;

#if USE_BURST_MODE
static kal_uint16 imx581_table_write_cmos_sensor(
		kal_uint16 * para, kal_uint32 len);
#endif
static DEFINE_SPINLOCK(imgsensor_drv_lock);


static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX581_SENSOR_ID,

	.checksum_value = 0xa4c32546,

	.pre = { /* reg_A-3 12M @60.00fps -> sensor mode0*/
		.pclk = 1728000000,
		.linelength = 8976,
		.framelength = 3208,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.mipi_pixel_rate = 998400000,
		.max_framerate = 600, /* 60fps */
	},

	.cap = { /*reg_A-3 12M@30fps -> sensor mode1*/
		.pclk = 1728000000,
		.linelength = 8976,
		.framelength = 3208,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.mipi_pixel_rate = 998400000,
		.max_framerate = 600, /* 60fps */
	},

	.normal_video = { /*reg_F-6 4000*2256@30fps -> sensor mode2*/
		.pclk = 844800000,
		.linelength = 8976,
		.framelength = 3136,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 2256,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 480000000,
		.max_framerate = 300,
	},

	.hs_video = { /* reg_D 2000x1128 @120fps (binning) -> sensor mode3*/
		.pclk = 854400000,
		.linelength = 5376,
		.framelength = 1324,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2000,
		.grabwindow_height = 1128,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 420000000,
		.max_framerate = 1200,
	},

	.slim_video = { /* reg_A 4000x2256@30fps -> sensor mode4 for MMI SFR test*/
		.pclk = 844800000,
		.linelength = 8976,
		.framelength = 3136,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 520800000,
		.max_framerate = 300,
	},

	.custom1 = { /* reg_A-1 4000x3000 @24fps -> sensor mode5*/
		.pclk = 844800000,
		.linelength = 8976,
		.framelength = 3920,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1056000000,
		.max_framerate = 240,
	},

	.custom2 = { /* reg_N 4K @ 60fps (binning) -> sensor mode6*/
		.pclk = 1257600000,
		.linelength = 8976,
		.framelength = 2334,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3840,
		.grabwindow_height = 2160,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 782400000,
		.max_framerate = 600,
	},

	.custom3 = { /* reg_B 8000x6000 @30fps -> sensor mode7*/
		.pclk = 1440000000,
		.linelength = 15504,
		.framelength = 6191,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 8000,
		.grabwindow_height = 6000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 924000000,
		.max_framerate = 150,
	},

	.custom4 = { /* reg_D-1 1280x720 @240fps -> sensor mode8*/
		.pclk = 1728000000,
		.linelength = 5376,
		.framelength = 1338,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2000,
		.grabwindow_height = 1128,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 960000000,
		.max_framerate = 2400,
	},

	.custom5 = { /* regC-1 4000x3000 @18fps -> sensor mode9*/
		.pclk = 1728000000,
		.linelength = 15504,
		.framelength = 6191,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 998400000,
		.max_framerate = 180,
	},

	.custom6 = { /* reg_F-9 4000*2256 @60.00fps -> sensor mode10*/
		.pclk = 1315200000,
		.linelength = 8976,
		.framelength = 2442,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 2256,
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.mipi_pixel_rate = 864000000,
		.max_framerate = 600, /* 60fps */
	},

	.custom7 = { /* reg_A 12M @30.00fps -> sensor mode11*/
		.pclk = 844800000,
		.linelength = 8976,
		.framelength = 3136,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.mipi_pixel_rate = 520800000,
		.max_framerate = 300, /* 30fps */
	},

	.custom8 = { /* reg_F-8 12M @60.00fps -> sensor mode12*/
		.pclk = 422400000,
		.linelength = 8976,
		.framelength = 3136,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 2256,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 223200000,
		.max_framerate = 150,
	},

	.custom9 = { /* reg_E 12M @30.00fps -> sensor mode13*/
		.pclk = 422400000,
		.linelength = 5376,
		.framelength = 2618,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2000,
		.grabwindow_height = 1500,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 175800000,
		.max_framerate = 300,
	},

	.custom10 = { /* reg_E 12M @30.00fps -> sensor mode14*/
		.pclk = 422400000,
		.linelength = 5376,
		.framelength = 2618,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2000,
		.grabwindow_height = 1500,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 175800000,
		.max_framerate = 150,
	},

	.margin = 48,		/* sensor framelength & shutter margin */
	.min_shutter = 5,	/* min shutter */
	.min_gain = 64,
	.max_gain = 4096,
	.min_gain_iso = 100,
	.exp_step = 2,
	.gain_step = 1,
	.gain_type = 0,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.temperature_support = 1,/* 1, support; 0,not support */
	.sensor_mode_num = 15,	/* support sensor mode num */

	.cap_delay_frame = 2,	/* enter capture delay frame num */
	.pre_delay_frame = 2,	/* enter preview delay frame num */
	.video_delay_frame = 2,	/* enter video delay frame num */
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,	/* enter slim video delay frame num */
	.custom1_delay_frame = 2,	/* enter custom1 delay frame num */
	.custom2_delay_frame = 2,	/* enter custom2 delay frame num */
	.custom3_delay_frame = 2,	/* enter custom3 delay frame num */
	.custom4_delay_frame = 2,	/* enter custom4 delay frame num */
	.custom5_delay_frame = 2,	/* enter custom5 delay frame num */
	.custom6_delay_frame = 2,	/* enter custom6 delay frame num */
	.custom7_delay_frame = 2,	/* enter custom7 delay frame num */
	.custom8_delay_frame = 2,	/* enter custom8 delay frame num */
	.custom9_delay_frame = 2,	/* enter custom9 delay frame num */
	.custom10_delay_frame = 2,	/* enter custom10 delay frame num */
	.frame_time_delay_frame = 3,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	/* .mipi_sensor_type = MIPI_OPHY_NCSI2, */
	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_sensor_type = MIPI_OPHY_NCSI2, /* 0,MIPI_OPHY_NCSI2; 1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 0,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	.mclk = 24, // mclk value
	//.mipi_lane_num = SENSOR_MIP suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x34,0xff},
	/* record sensor support all write id addr,
	 * only supprt 4 must end with 0xff
	 */
	.i2c_speed = 1000, /* i2c read/write speed */
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	/* IMGSENSOR_MODE enum value,record current sensor mode,such as:
	 * INIT, Preview, Capture, Video,High Speed Video, Slim Video
	 */
	.shutter = 0x3D0,	/* current shutter */
	.gain = 0x100,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = 0,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x34, /* record current sensor's i2c write id */
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[15] = {
	{8000, 6000, 0,    0, 8000, 6000, 4000, 3000,   0,   0, 4000, 3000,  0,  0, 4000, 3000},  /* Preview */
	{8000, 6000, 0,    0, 8000, 6000, 4000, 3000,   0,   0, 4000, 3000,  0,  0, 4000, 3000},  /* capture */
	{8000, 6000, 0,  744, 8000, 4512, 4000, 2256,   0,   0, 4000, 2256,  0,  0, 4000, 2256},  /* normal video */
	{8000, 6000, 0,    0, 8000, 6000, 2000, 1500,   0, 186, 2000, 1128,  0,  0, 2000, 1128},  /* hs vedio */
	{8000, 6000, 0,    0, 8000, 6000, 4000, 3000,   0,   0, 4000, 3000,  0,  0, 4000, 3000},  /* slim video */
	{8000, 6000, 0,    0, 8000, 6000, 4000, 3000,   0,   0, 4000, 3000,  0,  0, 4000, 3000},  /* Custom1 */
	{8000, 6000, 0,    0, 8000, 6000, 4000, 3000,  80, 420, 3840, 2160,  0,  0, 3840, 2160},  /* custom2 */
	{8000, 6000, 0,    0, 8000, 6000, 8000, 6000,   0,   0, 8000, 6000,  0,  0, 8000, 6000},  /* custom3 */
	{8000, 6000, 0,    0, 8000, 6000, 2000, 1500,   0, 186, 2000, 1128,  0,  0, 2000, 1128},  /* Custom4 */
	{8000, 6000, 2000,   1500, 4000, 3000, 4000, 3000,   0,   0, 4000, 3000,  0,  0, 4000, 3000},  /* Custom5 */
	{8000, 6000, 0,  744, 8000, 4512, 4000, 2256,   0,   0, 4000, 2256,  0,  0, 4000, 2256},  /* Custom6 */
	{8000, 6000, 0,    0, 8000, 6000, 4000, 3000,   0,   0, 4000, 3000,  0,  0, 4000, 3000},  /* Custom7 */
	{8000, 6000, 0,  744, 8000, 4512, 4000, 2256,   0,   0, 4000, 2256,  0,  0, 4000, 2256},  /* Custom8 */
	{8000, 6000, 0,    0, 8000, 6000, 2000, 1500,   0,   0, 2000, 1500,  0,  0, 2000, 1500},  /* Custom9 */
	{8000, 6000, 0,    0, 8000, 6000, 2000, 1500,   0,   0, 2000, 1500,  0,  0, 2000, 1500},  /* Custom10 */
};
 /*VC1 for HDR(DT=0X35), VC2 for PDAF(DT=0X36), unit : 10bit */
static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3] = {
	/* Preview mode setting 4000*3000 */
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0FA0, 0x0BB8, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x34, 0x04D8, 0x05D0, 0x00, 0x00, 0x0000, 0x0000},
	/* Normal_Video mode setting 4000*2256*/
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0FA0, 0x08D0, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x34, 0x04D8, 0x0460, 0x00, 0x00, 0x0000, 0x0000},
	/* custom2 4kVIDEO mode setting */
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0F00, 0x0870, 0x00, 0x00, 0x0000, 0x0000,
	 0x00, 0x34, 0x04B0, 0x0430, 0x00, 0x00, 0x0000, 0x0000},
};


/* If mirror flip */
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_binning = {
	.i4OffsetX = 17,
	.i4OffsetY = 12,
	.i4PitchX  =  8,
	.i4PitchY  = 16,
	.i4PairNum  = 8,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = { {20, 13}, {18, 15}, {22, 17}, {24, 19},
				{20, 21}, {18, 23}, {22, 25}, {24, 27} },
	.i4PosR = { {19, 13}, {17, 15}, {21, 17}, {23, 19},
				{19, 21}, {17, 23}, {21, 25}, {23, 27} },
	.i4BlockNumX = 496,
	.i4BlockNumY = 186,
	.iMirrorFlip = 3,
	.i4Crop = { {0, 0}, {0, 0}, {0, 372}, {0, 0}, {0, 0},
		{0, 0}, {80, 420}, {0, 0}, {0, 0}, {0, 0},
		{0, 372}, {0, 0}, {0, 372} },
};

static kal_uint16 imx581_QSC_setting[2304 * 2];
static kal_uint16 imx581_LRC_setting[384 * 2];

/* TODO: measure the delay */
static struct SEAMLESS_SYS_DELAY seamless_sys_delays[] = {
	{ MSDK_SCENARIO_ID_CAMERA_PREVIEW, MSDK_SCENARIO_ID_CUSTOM4, 1 },
	{ MSDK_SCENARIO_ID_CAMERA_PREVIEW, MSDK_SCENARIO_ID_CUSTOM5, 1 },
	{ MSDK_SCENARIO_ID_CAMERA_PREVIEW, MSDK_SCENARIO_ID_CUSTOM2, 1 },
	{ MSDK_SCENARIO_ID_CUSTOM4, MSDK_SCENARIO_ID_CAMERA_PREVIEW, 1 },
	{ MSDK_SCENARIO_ID_CUSTOM5, MSDK_SCENARIO_ID_CAMERA_PREVIEW, 1 },
	{ MSDK_SCENARIO_ID_CUSTOM4, MSDK_SCENARIO_ID_CUSTOM2, 1 },
	{ MSDK_SCENARIO_ID_CUSTOM2, MSDK_SCENARIO_ID_CAMERA_PREVIEW, 1 },
	{ MSDK_SCENARIO_ID_CUSTOM2, MSDK_SCENARIO_ID_CUSTOM4, 1 },
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

	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);*/
	/* Add this func to set i2c speed by each sensor */
	//iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
	iWriteRegI2CTiming(pusendcmd, 4, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
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

	//iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
	iWriteRegI2CTiming(pusendcmd, 3, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
}

static void imx581_get_pdaf_reg_setting(MUINT32 regNum, kal_uint16 *regDa)
{
	int i, idx;

	for (i = 0; i < regNum; i++) {
		idx = 2 * i;
		regDa[idx + 1] = read_cmos_sensor_8(regDa[idx]);
		pr_debug("%x %x", regDa[idx], regDa[idx+1]);
	}
}
static void imx581_set_pdaf_reg_setting(MUINT32 regNum, kal_uint16 *regDa)
{
	int i, idx;

	for (i = 0; i < regNum; i++) {
		idx = 2 * i;
		write_cmos_sensor_8(regDa[idx], regDa[idx + 1]);
		pr_debug("%x %x", regDa[idx], regDa[idx+1]);
	}
}

static kal_uint16 read_cmos_eeprom_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, 0xA0);
	return get_byte;
}

static void read_sensor_qsc(void)
{
	kal_uint16 idx = 0, addr_qsc = 0xfaf, sensor_lrc = 0x7F00;
	kal_uint16 eeprom_lrc_0 = 0x1620, eeprom_lrc_1 = 0x16E0;
	kal_uint16 sensor_lrc_0 = 0x7510, sensor_lrc_1 = 0x7600;
	kal_uint8 otp_data[9] = {0};
	int i = 0;

	/*read otp data to distinguish module*/
	otp_flag = OTP_QSC_NONE;

	for (i = 0; i < 7; i++)
		otp_data[i] = read_cmos_eeprom_8(0x0001 + i);


	/*Internal Module Type*/
	if ((otp_data[0] == 0xff) &&
		(otp_data[1] == 0x00) &&
		(otp_data[2] == 0x0b) &&
		(otp_data[3] == 0x01)) {
		pr_info("OTP type: Internal Only");
		otp_flag = OTP_QSC_INTERNAL;

		for (idx = 0; idx < 2304; idx++) {
			addr_qsc = 0xfaf + idx;
			sensor_lrc = 0x7F00 + idx;
			imx581_QSC_setting[2 * idx] = sensor_lrc;
			imx581_QSC_setting[2 * idx + 1] =
				read_cmos_eeprom_8(addr_qsc);
		}

		for (idx = 0; idx < 192; idx++) {
			imx581_LRC_setting[2 * idx] = sensor_lrc_0 + idx;
				imx581_LRC_setting[2 * idx + 1] =
			read_cmos_eeprom_8(eeprom_lrc_0 + idx);
			imx581_LRC_setting[2 * idx + 192 * 2] =
				sensor_lrc_1 + idx;
			imx581_LRC_setting[2 * idx + 1 + 192 * 2] =
			read_cmos_eeprom_8(eeprom_lrc_1 + idx);
		}

	} else if ((otp_data[5] == 0x0A) && (otp_data[6] == 0x00)) {
		/*Internal Module Type*/
		pr_info("OTP type: Custom Only");
		otp_flag = OTP_QSC_CUSTOM;

		for (idx = 0; idx < 2304; idx++) {
			addr_qsc = 0x1b00 + idx;
			sensor_lrc = 0x7f00 + idx;
			imx581_QSC_setting[2 * idx] = sensor_lrc;
			imx581_QSC_setting[2 * idx + 1] =
				read_cmos_eeprom_8(addr_qsc);
		}

	} else {
		pr_info("OTP type: No Data, 0x0008 = %d, 0x0009 = %d",
		read_cmos_eeprom_8(0x0008), read_cmos_eeprom_8(0x0009));
	}

}

static void Read_Eeprom_Data(void)
{
    int i = 0;

	/*Read QSC data from eeprom*/
	read_sensor_qsc();

    /*Read normal eeprom data*/
    gImgEepromInfo.camNormdata[0][0] = Eeprom_1ByteDataRead(0x00, 0xA0);
    gImgEepromInfo.camNormdata[0][1] = Eeprom_1ByteDataRead(0x01, 0xA0);
    imgsensor_info.module_id = Eeprom_1ByteDataRead(0x00, 0xA0);
    Oplusimgsensor_Registdeviceinfo(gImgEepromInfo.pCamModuleInfo[1].name,
                                    gImgEepromInfo.pCamModuleInfo[1].version,
                                    imgsensor_info.module_id);
    for(i = 2; i < 8; i++) {
        gImgEepromInfo.camNormdata[0][i] = Eeprom_1ByteDataRead(0x04+i, 0xA0);
    }
    for (i = 0; i < OPLUS_CAMERASN_LENS; i ++) {
        gImgEepromInfo.camNormdata[0][8+i] = Eeprom_1ByteDataRead(0xB0+i, 0xA0);
    }
    gImgEepromInfo.i4CurSensorIdx = 0;
    gImgEepromInfo.i4CurSensorId = imgsensor_info.sensor_id;
}

static void write_sensor_QSC(void)
{
	if ((otp_flag == OTP_QSC_CUSTOM) || (otp_flag == OTP_QSC_INTERNAL)) {
		imx581_table_write_cmos_sensor(imx581_QSC_setting,
		sizeof(imx581_QSC_setting) / sizeof(kal_uint16));
	}
}

static void set_dummy(void)
{
	pr_debug("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* return;*/ /* for test */
	write_cmos_sensor_8(0x0104, 0x01);

	write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor_8(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor_8(0x0343, imgsensor.line_length & 0xFF);

	write_cmos_sensor_8(0x0104, 0x00);

}	/*	set_dummy  */

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

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	/*kal_int16 dummy_line;*/
	kal_uint32 frame_length = imgsensor.frame_length;

	pr_debug(
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
	#ifdef LONG_EXP
	int longexposure_times = 0;
	static int long_exposure_status;
	#endif

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
			/* Extend frame length */
			write_cmos_sensor_8(0x0104, 0x01);
			write_cmos_sensor_8(0x0340,
					imgsensor.frame_length >> 8);
			write_cmos_sensor_8(0x0341,
					imgsensor.frame_length & 0xFF);
			write_cmos_sensor_8(0x0104, 0x00);
		}
	} else {
		/* Extend frame length*/
		if (read_cmos_sensor_8(0x0350) != 0x01) {
			pr_info("single cam scenario enable auto-extend");
			write_cmos_sensor_8(0x0350, 0x01);
		}
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	}
	#ifdef LONG_EXP
	while (shutter >= 65535) {
		shutter = shutter / 2;
		longexposure_times += 1;
	}

	if (longexposure_times > 0) {
		pr_debug("enter long exposure mode, time is %d",
			longexposure_times);
		long_exposure_status = 1;
		imgsensor.frame_length = shutter + 32;
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x3100, longexposure_times & 0x07);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	} else if (long_exposure_status == 1) {
		long_exposure_status = 0;
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x3100, 0x00);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);

		pr_debug("exit long exposure mode");
	}
	#endif
	if ((imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3) || (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM5)) {
		if ((shutter <= 16) && ((shutter % 2) == 0)) { //When the shutter is less than 16, the shutter value must be an odd number
			shutter = shutter - 1;
		}
	}
	/* Update Shutter */
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, shutter  & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);
	pr_debug("shutter =%d, framelength =%d\n",
		shutter, imgsensor.frame_length);

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
static void set_shutter_frame_length(kal_uint16 shutter,
				     kal_uint16 frame_length,
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

	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter)
			? imgsensor_info.min_shutter : shutter;
	shutter =
	(shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin)
		: shutter;

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
			write_cmos_sensor_8(0x0340,
					imgsensor.frame_length >> 8);
			write_cmos_sensor_8(0x0341,
					imgsensor.frame_length & 0xFF);
			write_cmos_sensor_8(0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor_8(0x0104, 0x01);
	if (auto_extend_en)
		write_cmos_sensor_8(0x0350, 0x01); /* Enable auto extend */
	else
		write_cmos_sensor_8(0x0350, 0x00); /* Disable auto extend */
	write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, shutter  & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);
	pr_debug(
		"Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
		shutter, imgsensor.frame_length, frame_length,
		dummy_line, read_cmos_sensor_8(0x0350));

}	/* set_shutter_frame_length */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	 kal_uint16 reg_gain = 0x0;

	reg_gain = 1024 - (1024*64)/gain;
	return (kal_uint16) reg_gain;
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
	kal_uint16 reg_gain, max_gain = imgsensor_info.max_gain;

	if ((imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3) || (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM5)) {
		/* 48M@30FPS */
		max_gain = 16 * BASEGAIN;
	}

	if (gain < imgsensor_info.min_gain || gain > max_gain) {
		pr_debug("Error max gain setting: %d\n", max_gain);

		if (gain < imgsensor_info.min_gain)
			gain = imgsensor_info.min_gain;
		else if (gain > max_gain)
			gain = max_gain;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	pr_debug("gain = %d, reg_gain = 0x%x, max_gain:0x%x\n ",
		gain, reg_gain, max_gain);

	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0204, (reg_gain>>8) & 0xFF);
	write_cmos_sensor_8(0x0205, reg_gain & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);

	return gain;
} /* set_gain */

static kal_uint32 imx581_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	#if 0
	UINT32 rgain_32, grgain_32, gbgain_32, bgain_32;

	grgain_32 = (pSetSensorAWB->ABS_GAIN_GR + 1) >> 1;
	rgain_32 = (pSetSensorAWB->ABS_GAIN_R + 1) >> 1;
	bgain_32 = (pSetSensorAWB->ABS_GAIN_B + 1) >> 1;
	gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB + 1) >> 1;
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

	write_cmos_sensor_8(0x0b8e, (grgain_32 >> 8) & 0xFF);
	write_cmos_sensor_8(0x0b8f, grgain_32 & 0xFF);
	write_cmos_sensor_8(0x0b90, (rgain_32 >> 8) & 0xFF);
	write_cmos_sensor_8(0x0b91, rgain_32 & 0xFF);
	write_cmos_sensor_8(0x0b92, (bgain_32 >> 8) & 0xFF);
	write_cmos_sensor_8(0x0b93, bgain_32 & 0xFF);
	write_cmos_sensor_8(0x0b94, (gbgain_32 >> 8) & 0xFF);
	write_cmos_sensor_8(0x0b95, gbgain_32 & 0xFF);

	imx581_awb_gain_table[1]  = (grgain_32 >> 8) & 0xFF;
	imx581_awb_gain_table[3]  = grgain_32 & 0xFF;
	imx581_awb_gain_table[5]  = (rgain_32 >> 8) & 0xFF;
	imx581_awb_gain_table[7]  = rgain_32 & 0xFF;
	imx581_awb_gain_table[9]  = (bgain_32 >> 8) & 0xFF;
	imx581_awb_gain_table[11] = bgain_32 & 0xFF;
	imx581_awb_gain_table[13] = (gbgain_32 >> 8) & 0xFF;
	imx581_awb_gain_table[15] = gbgain_32 & 0xFF;
	imx581_table_write_cmos_sensor(imx581_awb_gain_table,
		sizeof(imx581_awb_gain_table)/sizeof(kal_uint16));
	#endif

	return ERROR_NONE;
}

static kal_uint16 imx581_feedback_awbgain[] = {
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

	r_gain_int = r_gain / 512;
	b_gain_int = b_gain / 512;

	imx581_feedback_awbgain[1] = r_gain_int;
	imx581_feedback_awbgain[3] = (
		((r_gain*100) / 512) - (r_gain_int * 100)) * 2;
	imx581_feedback_awbgain[5] = b_gain_int;
	imx581_feedback_awbgain[7] = (
		((b_gain * 100) / 512) - (b_gain_int * 100)) * 2;
	write_cmos_sensor_8(0x0104, 0x01);
	imx581_table_write_cmos_sensor(imx581_feedback_awbgain,
		sizeof(imx581_feedback_awbgain)/sizeof(kal_uint16));
	write_cmos_sensor_8(0x0104, 0x00);

}

static void imx581_set_lsc_reg_setting(
		kal_uint8 index, kal_uint16 *regDa, MUINT32 regNum)
{
	int i;
	int startAddr[4] = {0x9D88, 0x9CB0, 0x9BD8, 0x9B00};
	/*0:B,1:Gb,2:Gr,3:R*/

	pr_debug("E! index:%d, regNum:%d\n", index, regNum);

	write_cmos_sensor_8(0x0B00, 0x01); /*lsc enable*/
	write_cmos_sensor_8(0x9014, 0x01);
	write_cmos_sensor_8(0x4439, 0x01);
	mdelay(1);
	pr_debug("Addr 0xB870, 0x380D Value:0x%x %x\n",
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
	pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n",
		enable);
	if (enable)
		write_cmos_sensor_8(0x0100, 0X01);
	else
		write_cmos_sensor_8(0x0100, 0x00);
	return ERROR_NONE;
}

#if USE_BURST_MODE
#define I2C_BUFFER_LEN 765 /* trans# max is 255, each 3 bytes */
static kal_uint16 imx581_table_write_cmos_sensor(kal_uint16 *para,
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
#endif
static kal_uint16 imx581_init_setting[] = {
	0x0136, 0x18,
	0x0137, 0x00,
	0x3C7E, 0x02,
	0x3C7F, 0x0A,
	0x0111, 0x02,
	0x380C, 0x00,
	0x3C00, 0x10,
	0x3C01, 0x10,
	0x3C02, 0x10,
	0x3C03, 0x10,
	0x3C04, 0x10,
	0x3C05, 0x01,
	0x3C06, 0x00,
	0x3C07, 0x00,
	0x3C08, 0x03,
	0x3C09, 0xFF,
	0x3C0A, 0x01,
	0x3C0B, 0x00,
	0x3C0C, 0x00,
	0x3C0D, 0x03,
	0x3C0E, 0xFF,
	0x3C0F, 0x20,
	0x3F88, 0x00,
	0x3F8E, 0x00,
	0x5282, 0x01,
	0x9004, 0x14,
	0x9200, 0xF4,
	0x9201, 0xA7,
	0x9202, 0xF4,
	0x9203, 0xAA,
	0x9204, 0xF4,
	0x9205, 0xAD,
	0x9206, 0xF4,
	0x9207, 0xB0,
	0x9208, 0xF4,
	0x9209, 0xB3,
	0x920A, 0xB7,
	0x920B, 0x34,
	0x920C, 0xB7,
	0x920D, 0x36,
	0x920E, 0xB7,
	0x920F, 0x37,
	0x9210, 0xB7,
	0x9211, 0x38,
	0x9212, 0xB7,
	0x9213, 0x39,
	0x9214, 0xB7,
	0x9215, 0x3A,
	0x9216, 0xB7,
	0x9217, 0x3C,
	0x9218, 0xB7,
	0x9219, 0x3D,
	0x921A, 0xB7,
	0x921B, 0x3E,
	0x921C, 0xB7,
	0x921D, 0x3F,
	0x921E, 0x77,
	0x921F, 0x77,
	0x9222, 0xC4,
	0x9223, 0x4B,
	0x9224, 0xC4,
	0x9225, 0x4C,
	0x9226, 0xC4,
	0x9227, 0x4D,
	0x9810, 0x14,
	0x9814, 0x14,
	0x99B2, 0x20,
	0x99B3, 0x0F,
	0x99B4, 0x0F,
	0x99B5, 0x0F,
	0x99B6, 0x0F,
	0x99E4, 0x0F,
	0x99E5, 0x0F,
	0x99E6, 0x0F,
	0x99E7, 0x0F,
	0x99E8, 0x0F,
	0x99E9, 0x0F,
	0x99EA, 0x0F,
	0x99EB, 0x0F,
	0x99EC, 0x0F,
	0x99ED, 0x0F,
	0xA569, 0x06,
	0xA56A, 0x13,
	0xA56B, 0x13,
	0xA679, 0x20,
	0xA830, 0x68,
	0xA831, 0x56,
	0xA832, 0x2B,
	0xA833, 0x55,
	0xA834, 0x55,
	0xA835, 0x16,
	0xA837, 0x51,
	0xA838, 0x34,
	0xA854, 0x4F,
	0xA855, 0x48,
	0xA856, 0x45,
	0xA857, 0x02,
	0xA85A, 0x23,
	0xA85B, 0x16,
	0xA85C, 0x12,
	0xA85D, 0x02,
	0xAC72, 0x01,
	0xAC73, 0x26,
	0xAC74, 0x01,
	0xAC75, 0x26,
	0xAC76, 0x00,
	0xAC77, 0xC4,
	0xB051, 0x02,
	0xC020, 0x01,
	0xC61D, 0x00,
	0xC625, 0x00,
	0xC638, 0x03,
	0xC63B, 0x01,
	0xE286, 0x31,
	0xE2A6, 0x32,
	0xE2C6, 0x33,
	0xEA4B, 0x00,
	0xEA4C, 0x00,
	0xEA4D, 0x00,
	0xEA4E, 0x00,
	0xF000, 0x00,
	0xF001, 0x10,
	0xF00C, 0x00,
	0xF00D, 0x40,
	0xF030, 0x00,
	0xF031, 0x10,
	0xF03C, 0x00,
	0xF03D, 0x40,
	0xF44B, 0x80,
	0xF44C, 0x10,
	0xF44D, 0x06,
	0xF44E, 0x80,
	0xF44F, 0x10,
	0xF450, 0x06,
	0xF451, 0x80,
	0xF452, 0x10,
	0xF453, 0x06,
	0xF454, 0x80,
	0xF455, 0x10,
	0xF456, 0x06,
	0xF457, 0x80,
	0xF458, 0x10,
	0xF459, 0x06,
	0xF478, 0x20,
	0xF479, 0x80,
	0xF47A, 0x80,
	0xF47B, 0x20,
	0xF47C, 0x80,
	0xF47D, 0x80,
	0xF47E, 0x20,
	0xF47F, 0x80,
	0xF480, 0x80,
	0xF481, 0x20,
	0xF482, 0x60,
	0xF483, 0x80,
	0xF484, 0x20,
	0xF485, 0x60,
	0xF486, 0x80,
};

static kal_uint16 imx581_capture_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0x10,
	0x0340, 0x0C,
	0x0341, 0x88,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x68,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0xA0,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x3C,
	0x3FFE, 0x01,
	0x3FFF, 0x8C,
	0x0202, 0x0C,
	0x0203, 0x58,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,
};


static kal_uint16 imx581_preview_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0x10,
	0x0340, 0x0C,
	0x0341, 0x88,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x68,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0xA0,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x3C,
	0x3FFE, 0x01,
	0x3FFF, 0x8C,
	0x0202, 0x0C,
	0x0203, 0x58,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,
};


static kal_uint16 imx581_normal_video_setting_4K60FPS[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0x10,
	0x0340, 0x09,
	0x0341, 0x1E,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x03,
	0x0347, 0x48,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x14,
	0x034B, 0x27,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x50,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0x00,
	0x040E, 0x08,
	0x040F, 0x70,
	0x034C, 0x0F,
	0x034D, 0x00,
	0x034E, 0x08,
	0x034F, 0x70,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x06,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x46,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x3C,
	0x3FFE, 0x01,
	0x3FFF, 0x8C,
	0x0202, 0x08,
	0x0203, 0xEE,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xE0,
};

static kal_uint16 imx581_normal_video_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0x10,
	0x0340, 0x0C,
	0x0341, 0x40,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x02,
	0x0347, 0xE8,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x14,
	0x034B, 0x87,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x08,
	0x040F, 0xD0,
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x08,
	0x034F, 0xD0,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xB0,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xC8,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x3C,
	0x3FFE, 0x01,
	0x3FFF, 0x8C,
	0x0202, 0x0C,
	0x0203, 0x10,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	/* pd data 0x00f8=496 pair */
	0x4434, 0x01,
	0x4435, 0xF0,
};

static kal_uint16 imx581_hs_video_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x15,
	0x0343, 0x00,
	0x0340, 0x05,
	0x0341, 0x2C,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x02,
	0x0347, 0xE8,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x14,
	0x034B, 0x87,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x44,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x89,
	0x3247, 0x89,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x07,
	0x040D, 0xD0,
	0x040E, 0x04,
	0x040F, 0x68,
	0x034C, 0x07,
	0x034D, 0xD0,
	0x034E, 0x04,
	0x034F, 0x68,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xB2,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xAF,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x0C,
	0x3C12, 0x05,
	0x3C13, 0x2C,
	0x3F0C, 0x00,
	0x3F14, 0x00,
	0x3F80, 0x02,
	0x3F81, 0x67,
	0x3F8C, 0x02,
	0x3F8D, 0x44,
	0x3FF8, 0x00,
	0x3FF9, 0x00,
	0x3FFE, 0x01,
	0x3FFF, 0x90,
	0x0202, 0x04,
	0x0203, 0xFC,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x01,
	0x3E37, 0x00,
	0x3E20, 0x01,
	0x3E3B, 0x00,
	0x4434, 0x00,
	0x4435, 0xF8,
};

static kal_uint16 imx581_slim_video_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0x10,
	0x0340, 0x0C,
	0x0341, 0x40,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xB0,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xD9,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x3C,
	0x3FFE, 0x01,
	0x3FFF, 0x8C,
	0x0202, 0x0C,
	0x0203, 0x10,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,
};

static kal_uint16 imx581_custom1_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0x10,
	0x0340, 0x0F,
	0x0341, 0x50,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xB0,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xD9,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x3C,
	0x3FFE, 0x01,
	0x3FFF, 0x8C,
	0x0202, 0x0F,
	0x0203, 0x20,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,
};

static kal_uint16 imx581_custom4_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x15,
	0x0343, 0x00,
	0x0340, 0x05,
	0x0341, 0x3A,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x02,
	0x0347, 0xE8,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x14,
	0x034B, 0x87,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x44,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x89,
	0x3247, 0x89,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x07,
	0x040D, 0xD0,
	0x040E, 0x04,
	0x040F, 0x68,
	0x034C, 0x07,
	0x034D, 0xD0,
	0x034E, 0x04,
	0x034F, 0x68,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x68,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x90,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x0C,
	0x3C12, 0x05,
	0x3C13, 0x2C,
	0x3F0C, 0x00,
	0x3F14, 0x00,
	0x3F80, 0x02,
	0x3F81, 0x67,
	0x3F8C, 0x02,
	0x3F8D, 0x44,
	0x3FF8, 0x00,
	0x3FF9, 0x00,
	0x3FFE, 0x01,
	0x3FFF, 0x90,
	0x0202, 0x05,
	0x0203, 0x0A,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x01,
	0x3E37, 0x00,
	0x3E20, 0x01,
	0x3E3B, 0x00,
	0x4434, 0x00,
	0x4435, 0xF8,
};

static kal_uint16 imx581_custom5_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x3C,
	0x0343, 0x90,
	0x0340, 0x18,
	0x0341, 0x2F,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x0A,
	0x3140, 0x00,
	0x3246, 0x01,
	0x3247, 0x01,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x07,
	0x0409, 0xD0,
	0x040A, 0x05,
	0x040B, 0xDC,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x68,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0xA0,
	0x0310, 0x01,
	0x3620, 0x01,
	0x3621, 0x01,
	0x3C11, 0x08,
	0x3C12, 0x08,
	0x3C13, 0x2A,
	0x3F0C, 0x00,
	0x3F14, 0x01,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x00,
	0x3FFE, 0x03,
	0x3FFF, 0x65,
	0x0202, 0x17,
	0x0203, 0xFF,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x00,
	0x4435, 0xF8,
};

static kal_uint16 imx581_custom6_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0x10,
	0x0340, 0x09,
	0x0341, 0x8A,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x02,
	0x0347, 0xE8,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x14,
	0x034B, 0x87,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x08,
	0x040F, 0xD0,
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x08,
	0x034F, 0xD0,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x12,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x68,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x3C,
	0x3FFE, 0x01,
	0x3FFF, 0x8C,
	0x0202, 0x09,
	0x0203, 0x5A,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,
};

static kal_uint16 imx581_custom7_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0x10,
	0x0340, 0x0C,
	0x0341, 0x40,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xB0,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xD9,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x3C,
	0x3FFE, 0x01,
	0x3FFF, 0x8C,
	0x0202, 0x0C,
	0x0203, 0x10,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,
};

static kal_uint16 imx581_custom8_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0x10,
	0x0340, 0x0C,
	0x0341, 0x40,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x02,
	0x0347, 0xE8,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x14,
	0x034B, 0x87,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x08,
	0x040F, 0xD0,
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x08,
	0x034F, 0xD0,
	0x0301, 0x05,
	0x0303, 0x04,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xB0,
	0x030B, 0x02,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xBA,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x3C,
	0x3FFE, 0x01,
	0x3FFF, 0x8C,
	0x0202, 0x0C,
	0x0203, 0x10,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x00,
	0x4435, 0xF8,
};


static kal_uint16 imx581_custom9_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x15,
	0x0343, 0x00,
	0x0340, 0x0A,
	0x0341, 0x3A,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x44,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x89,
	0x3247, 0x89,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x07,
	0x040D, 0xD0,
	0x040E, 0x05,
	0x040F, 0xDC,
	0x034C, 0x07,
	0x034D, 0xD0,
	0x034E, 0x05,
	0x034F, 0xDC,
	0x0301, 0x05,
	0x0303, 0x04,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xB0,
	0x030B, 0x04,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x25,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x0C,
	0x3C12, 0x05,
	0x3C13, 0x2C,
	0x3F0C, 0x00,
	0x3F14, 0x00,
	0x3F80, 0x02,
	0x3F81, 0x67,
	0x3F8C, 0x02,
	0x3F8D, 0x44,
	0x3FF8, 0x00,
	0x3FF9, 0x00,
	0x3FFE, 0x01,
	0x3FFF, 0x90,
	0x0202, 0x0A,
	0x0203, 0x0A,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x01,
	0x3E37, 0x00,
	0x3E20, 0x01,
	0x3E3B, 0x00,
	0x4434, 0x00,
	0x4435, 0xF8,
};


static kal_uint16 imx581_custom10_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x15,
	0x0343, 0x00,
	0x0340, 0x0A,
	0x0341, 0x3A,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x44,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x89,
	0x3247, 0x89,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x07,
	0x040D, 0xD0,
	0x040E, 0x05,
	0x040F, 0xDC,
	0x034C, 0x07,
	0x034D, 0xD0,
	0x034E, 0x05,
	0x034F, 0xDC,
	0x0301, 0x05,
	0x0303, 0x04,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xB0,
	0x030B, 0x04,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x25,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x0C,
	0x3C12, 0x05,
	0x3C13, 0x2C,
	0x3F0C, 0x00,
	0x3F14, 0x00,
	0x3F80, 0x02,
	0x3F81, 0x67,
	0x3F8C, 0x02,
	0x3F8D, 0x44,
	0x3FF8, 0x00,
	0x3FF9, 0x00,
	0x3FFE, 0x01,
	0x3FFF, 0x90,
	0x0202, 0x0A,
	0x0203, 0x0A,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x01,
	0x3E37, 0x00,
	0x3E20, 0x01,
	0x3E3B, 0x00,
	0x4434, 0x00,
	0x4435, 0xF8,
};

static kal_uint16 imx581_custom3_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x3C,
	0x0343, 0x90,
	0x0340, 0x18,
	0x0341, 0x2F,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x0A,
	0x3140, 0x00,
	0x3246, 0x01,
	0x3247, 0x01,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x1F,
	0x040D, 0x40,
	0x040E, 0x17,
	0x040F, 0x70,
	0x034C, 0x1F,
	0x034D, 0x40,
	0x034E, 0x17,
	0x034F, 0x70,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x2C,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x81,
	0x0310, 0x01,
	0x3620, 0x01,
	0x3621, 0x01,
	0x3C11, 0x08,
	0x3C12, 0x08,
	0x3C13, 0x2A,
	0x3F0C, 0x00,
	0x3F14, 0x01,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x00,
	0x3FFE, 0x03,
	0x3FFF, 0x65,
	0x0202, 0x17,
	0x0203, 0xFF,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,
};


static void sensor_init(void)
{
	pr_debug("[%s] start\n", __func__);
	#if USE_BURST_MODE
	imx581_table_write_cmos_sensor(imx581_init_setting,
		sizeof(imx581_init_setting)/sizeof(kal_uint16));
	#else
	/*External Clock Setting*/
	write_cmos_sensor_8(0x0136, 0x18);
	write_cmos_sensor_8(0x0137, 0x00);
	/*Register version*/
	write_cmos_sensor_8(0x3C7E, 0x04);
	write_cmos_sensor_8(0x3C7F, 0x08);
	/*Signaling mode setting*/
	write_cmos_sensor_8(0x0111, 0x03);
	/*Global Setting*/
	write_cmos_sensor_8(0x380C, 0x00);
	write_cmos_sensor_8(0x3C00, 0x10);
	write_cmos_sensor_8(0x3C01, 0x10);
	write_cmos_sensor_8(0x3C02, 0x10);
	write_cmos_sensor_8(0x3C03, 0x10);
	write_cmos_sensor_8(0x3C04, 0x10);
	write_cmos_sensor_8(0x3C05, 0x01);
	write_cmos_sensor_8(0x3C06, 0x00);
	write_cmos_sensor_8(0x3C07, 0x00);
	write_cmos_sensor_8(0x3C08, 0x03);
	write_cmos_sensor_8(0x3C09, 0xFF);
	write_cmos_sensor_8(0x3C0A, 0x01);
	write_cmos_sensor_8(0x3C0B, 0x00);
	write_cmos_sensor_8(0x3C0C, 0x00);
	write_cmos_sensor_8(0x3C0D, 0x03);
	write_cmos_sensor_8(0x3C0E, 0xFF);
	write_cmos_sensor_8(0x3C0F, 0x20);
	write_cmos_sensor_8(0x3F88, 0x00);
	write_cmos_sensor_8(0x3F8E, 0x00);
	write_cmos_sensor_8(0x5282, 0x01);
	write_cmos_sensor_8(0x9004, 0x14);
	write_cmos_sensor_8(0x9200, 0xF4);
	write_cmos_sensor_8(0x9201, 0xA7);
	write_cmos_sensor_8(0x9202, 0xF4);
	write_cmos_sensor_8(0x9203, 0xAA);
	write_cmos_sensor_8(0x9204, 0xF4);
	write_cmos_sensor_8(0x9205, 0xAD);
	write_cmos_sensor_8(0x9206, 0xF4);
	write_cmos_sensor_8(0x9207, 0xB0);
	write_cmos_sensor_8(0x9208, 0xF4);
	write_cmos_sensor_8(0x9209, 0xB3);
	write_cmos_sensor_8(0x920A, 0xB7);
	write_cmos_sensor_8(0x920B, 0x34);
	write_cmos_sensor_8(0x920C, 0xB7);
	write_cmos_sensor_8(0x920D, 0x36);
	write_cmos_sensor_8(0x920E, 0xB7);
	write_cmos_sensor_8(0x920F, 0x37);
	write_cmos_sensor_8(0x9210, 0xB7);
	write_cmos_sensor_8(0x9211, 0x38);
	write_cmos_sensor_8(0x9212, 0xB7);
	write_cmos_sensor_8(0x9213, 0x39);
	write_cmos_sensor_8(0x9214, 0xB7);
	write_cmos_sensor_8(0x9215, 0x3A);
	write_cmos_sensor_8(0x9216, 0xB7);
	write_cmos_sensor_8(0x9217, 0x3C);
	write_cmos_sensor_8(0x9218, 0xB7);
	write_cmos_sensor_8(0x9219, 0x3D);
	write_cmos_sensor_8(0x921A, 0xB7);
	write_cmos_sensor_8(0x921B, 0x3E);
	write_cmos_sensor_8(0x921C, 0xB7);
	write_cmos_sensor_8(0x921D, 0x3F);
	write_cmos_sensor_8(0x921E, 0x77);
	write_cmos_sensor_8(0x921F, 0x77);
	write_cmos_sensor_8(0x9222, 0xC4);
	write_cmos_sensor_8(0x9223, 0x4B);
	write_cmos_sensor_8(0x9224, 0xC4);
	write_cmos_sensor_8(0x9225, 0x4C);
	write_cmos_sensor_8(0x9226, 0xC4);
	write_cmos_sensor_8(0x9227, 0x4D);
	write_cmos_sensor_8(0x9810, 0x14);
	write_cmos_sensor_8(0x9814, 0x14);
	write_cmos_sensor_8(0x99B2, 0x20);
	write_cmos_sensor_8(0x99B3, 0x0F);
	write_cmos_sensor_8(0x99B4, 0x0F);
	write_cmos_sensor_8(0x99B5, 0x0F);
	write_cmos_sensor_8(0x99B6, 0x0F);
	write_cmos_sensor_8(0x99E4, 0x0F);
	write_cmos_sensor_8(0x99E5, 0x0F);
	write_cmos_sensor_8(0x99E6, 0x0F);
	write_cmos_sensor_8(0x99E7, 0x0F);
	write_cmos_sensor_8(0x99E8, 0x0F);
	write_cmos_sensor_8(0x99E9, 0x0F);
	write_cmos_sensor_8(0x99EA, 0x0F);
	write_cmos_sensor_8(0x99EB, 0x0F);
	write_cmos_sensor_8(0x99EC, 0x0F);
	write_cmos_sensor_8(0x99ED, 0x0F);
	write_cmos_sensor_8(0xA569, 0x06);
	write_cmos_sensor_8(0xA679, 0x20);
	write_cmos_sensor_8(0xC020, 0x01);
	write_cmos_sensor_8(0xC61D, 0x00);
	write_cmos_sensor_8(0xC625, 0x00);
	write_cmos_sensor_8(0xC638, 0x03);
	write_cmos_sensor_8(0xC63B, 0x01);
	write_cmos_sensor_8(0xE286, 0x31);
	write_cmos_sensor_8(0xE2A6, 0x32);
	write_cmos_sensor_8(0xE2C6, 0x33);
	#endif
	/*enable temperature sensor, TEMP_SEN_CTL:*/
	write_cmos_sensor_8(0x0138, 0x01);

	set_mirror_flip(imgsensor.mirror);
	pr_debug("[%s] End\n", __func__);
}	/*	  sensor_init  */

static void preview_setting(void)
{
	pr_debug("%s +\n", __func__);

	#if USE_BURST_MODE
	imx581_table_write_cmos_sensor(imx581_preview_setting,
		sizeof(imx581_preview_setting)/sizeof(kal_uint16));
	#else
	/*MIPI output setting*/
	write_cmos_sensor_8(0x0112, 0x0A);
	write_cmos_sensor_8(0x0113, 0x0A);
	write_cmos_sensor_8(0x0114, 0x03);
	/*Line Length PCK Setting*/
	write_cmos_sensor_8(0x0342, 0x1E);
	write_cmos_sensor_8(0x0343, 0xC0);
	/*Frame Length Lines Setting*/
	write_cmos_sensor_8(0x0340, 0x0B);
	write_cmos_sensor_8(0x0341, 0xFC);
	/*ROI Setting*/
	write_cmos_sensor_8(0x0344, 0x00);
	write_cmos_sensor_8(0x0345, 0x00);
	write_cmos_sensor_8(0x0346, 0x00);
	write_cmos_sensor_8(0x0347, 0x00);
	write_cmos_sensor_8(0x0348, 0x1F);
	write_cmos_sensor_8(0x0349, 0x3F);
	write_cmos_sensor_8(0x034A, 0x17);
	write_cmos_sensor_8(0x034B, 0x6F);
	/*Mode Setting*/
	write_cmos_sensor_8(0x0220, 0x62);
	write_cmos_sensor_8(0x0222, 0x01);
	write_cmos_sensor_8(0x0900, 0x01);
	write_cmos_sensor_8(0x0901, 0x22);
	write_cmos_sensor_8(0x0902, 0x08);
	write_cmos_sensor_8(0x3140, 0x00);
	write_cmos_sensor_8(0x3246, 0x81);
	write_cmos_sensor_8(0x3247, 0x81);
	write_cmos_sensor_8(0x3F15, 0x00);
	/*Digital Crop & Scaling*/
	write_cmos_sensor_8(0x0401, 0x00);
	write_cmos_sensor_8(0x0404, 0x00);
	write_cmos_sensor_8(0x0405, 0x10);
	write_cmos_sensor_8(0x0408, 0x00);
	write_cmos_sensor_8(0x0409, 0x00);
	write_cmos_sensor_8(0x040A, 0x00);
	write_cmos_sensor_8(0x040B, 0x00);
	write_cmos_sensor_8(0x040C, 0x0F);
	write_cmos_sensor_8(0x040D, 0xA0);
	write_cmos_sensor_8(0x040E, 0x0B);
	write_cmos_sensor_8(0x040F, 0xB8);
	/*Output Size Setting*/
	write_cmos_sensor_8(0x034C, 0x0F);
	write_cmos_sensor_8(0x034D, 0xA0);
	write_cmos_sensor_8(0x034E, 0x0B);
	write_cmos_sensor_8(0x034F, 0xB8);
	/*Clock Setting*/
	write_cmos_sensor_8(0x0301, 0x05);
	write_cmos_sensor_8(0x0303, 0x02);
	write_cmos_sensor_8(0x0305, 0x04);
	write_cmos_sensor_8(0x0306, 0x01);
	write_cmos_sensor_8(0x0307, 0x2E);
	write_cmos_sensor_8(0x030B, 0x01);
	write_cmos_sensor_8(0x030D, 0x0C);
	write_cmos_sensor_8(0x030E, 0x02);
	write_cmos_sensor_8(0x030F, 0xC6);
	write_cmos_sensor_8(0x0310, 0x01);
	/*Other Setting*/
	write_cmos_sensor_8(0x3620, 0x00);
	write_cmos_sensor_8(0x3621, 0x00);
	write_cmos_sensor_8(0x3C11, 0x04);
	write_cmos_sensor_8(0x3C12, 0x03);
	write_cmos_sensor_8(0x3C13, 0x2D);
	write_cmos_sensor_8(0x3F0C, 0x01);
	write_cmos_sensor_8(0x3F14, 0x00);
	write_cmos_sensor_8(0x3F80, 0x01);
	write_cmos_sensor_8(0x3F81, 0x90);
	write_cmos_sensor_8(0x3F8C, 0x00);
	write_cmos_sensor_8(0x3F8D, 0x14);
	write_cmos_sensor_8(0x3FF8, 0x01);
	write_cmos_sensor_8(0x3FF9, 0x2A);
	write_cmos_sensor_8(0x3FFE, 0x00);
	write_cmos_sensor_8(0x3FFF, 0x6C);
	/*Integration Setting*/
	write_cmos_sensor_8(0x0202, 0x0B);
	write_cmos_sensor_8(0x0203, 0xCC);
	write_cmos_sensor_8(0x0224, 0x01);
	write_cmos_sensor_8(0x0225, 0xF4);
	write_cmos_sensor_8(0x3FE0, 0x01);
	write_cmos_sensor_8(0x3FE1, 0xF4);
	/*Gain Setting*/
	write_cmos_sensor_8(0x0204, 0x00);
	write_cmos_sensor_8(0x0205, 0x70);
	write_cmos_sensor_8(0x0216, 0x00);
	write_cmos_sensor_8(0x0217, 0x70);
	write_cmos_sensor_8(0x0218, 0x01);
	write_cmos_sensor_8(0x0219, 0x00);
	write_cmos_sensor_8(0x020E, 0x01);
	write_cmos_sensor_8(0x020F, 0x00);
	write_cmos_sensor_8(0x0210, 0x01);
	write_cmos_sensor_8(0x0211, 0x00);
	write_cmos_sensor_8(0x0212, 0x01);
	write_cmos_sensor_8(0x0213, 0x00);
	write_cmos_sensor_8(0x0214, 0x01);
	write_cmos_sensor_8(0x0215, 0x00);
	write_cmos_sensor_8(0x3FE2, 0x00);
	write_cmos_sensor_8(0x3FE3, 0x70);
	write_cmos_sensor_8(0x3FE4, 0x01);
	write_cmos_sensor_8(0x3FE5, 0x00);
	/*PDAF TYPE1 Setting*/
	write_cmos_sensor_8(0x3E20, 0x02);
	write_cmos_sensor_8(0x3E37, 0x00);
	/*PDAF TYPE2 Setting*/
	write_cmos_sensor_8(0x3E20, 0x02);
	write_cmos_sensor_8(0x3E3B, 0x01);
	write_cmos_sensor_8(0x4434, 0x01);
	write_cmos_sensor_8(0x4435, 0xF0);
	#endif
	pr_debug("%s -\n", __func__);
} /* preview_setting */


/*full size 30fps*/
static void capture_setting(kal_uint16 currefps)
{
	pr_debug("%s 30 fps E! currefps:%d\n", __func__, currefps);
	/*************MIPI output setting************/
	imx581_table_write_cmos_sensor(imx581_capture_setting,
		sizeof(imx581_capture_setting)/sizeof(kal_uint16));

	pr_debug("%s 30 fpsX\n", __func__);
}

static void normal_video_setting(kal_uint16 currefps)
{
	pr_debug("%s E! currefps:%d\n", __func__, currefps);

	#if USE_BURST_MODE
	imx581_table_write_cmos_sensor(imx581_normal_video_setting,
		sizeof(imx581_normal_video_setting)/sizeof(kal_uint16));
	#else
	/*MIPI output setting*/
	write_cmos_sensor_8(0x0112, 0x0A);
	write_cmos_sensor_8(0x0113, 0x0A);
	write_cmos_sensor_8(0x0114, 0x02);
	/*Line Length PCK Setting*/
	write_cmos_sensor_8(0x0342, 0x1E);
	write_cmos_sensor_8(0x0343, 0xC0);
	/*Frame Length Lines Setting*/
	write_cmos_sensor_8(0x0340, 0x0E);
	write_cmos_sensor_8(0x0341, 0x9A);
	/*ROI Setting*/
	write_cmos_sensor_8(0x0344, 0x00);
	write_cmos_sensor_8(0x0345, 0x00);
	write_cmos_sensor_8(0x0346, 0x01);
	write_cmos_sensor_8(0x0347, 0x90);
	write_cmos_sensor_8(0x0348, 0x1F);
	write_cmos_sensor_8(0x0349, 0x3F);
	write_cmos_sensor_8(0x034A, 0x15);
	write_cmos_sensor_8(0x034B, 0xDF);
	/*Mode Setting*/
	write_cmos_sensor_8(0x0220, 0x62);
	write_cmos_sensor_8(0x0222, 0x01);
	write_cmos_sensor_8(0x0900, 0x01);
	write_cmos_sensor_8(0x0901, 0x22);
	write_cmos_sensor_8(0x0902, 0x08);
	write_cmos_sensor_8(0x3140, 0x00);
	write_cmos_sensor_8(0x3246, 0x81);
	write_cmos_sensor_8(0x3247, 0x81);
	write_cmos_sensor_8(0x3F15, 0x00);
	/*Digital Crop & Scaling*/
	write_cmos_sensor_8(0x0401, 0x00);
	write_cmos_sensor_8(0x0404, 0x00);
	write_cmos_sensor_8(0x0405, 0x10);
	write_cmos_sensor_8(0x0408, 0x00);
	write_cmos_sensor_8(0x0409, 0x00);
	write_cmos_sensor_8(0x040A, 0x00);
	write_cmos_sensor_8(0x040B, 0x00);
	write_cmos_sensor_8(0x040C, 0x0F);
	write_cmos_sensor_8(0x040D, 0xA0);
	write_cmos_sensor_8(0x040E, 0x0A);
	write_cmos_sensor_8(0x040F, 0x28);
	/*Output Size Setting*/
	write_cmos_sensor_8(0x034C, 0x0F);
	write_cmos_sensor_8(0x034D, 0xA0);
	write_cmos_sensor_8(0x034E, 0x0A);
	write_cmos_sensor_8(0x034F, 0x28);
	/*Clock Setting*/
	write_cmos_sensor_8(0x0301, 0x05);
	write_cmos_sensor_8(0x0303, 0x02);
	write_cmos_sensor_8(0x0305, 0x04);
	write_cmos_sensor_8(0x0306, 0x00);
	write_cmos_sensor_8(0x0307, 0xB8);
	write_cmos_sensor_8(0x030B, 0x02);
	write_cmos_sensor_8(0x030D, 0x04);
	write_cmos_sensor_8(0x030E, 0x01);
	write_cmos_sensor_8(0x030F, 0x1C);
	write_cmos_sensor_8(0x0310, 0x01);
	/*Other Setting*/
	write_cmos_sensor_8(0x3620, 0x00);
	write_cmos_sensor_8(0x3621, 0x00);
	write_cmos_sensor_8(0x3C11, 0x04);
	write_cmos_sensor_8(0x3C12, 0x03);
	write_cmos_sensor_8(0x3C13, 0x2D);
	write_cmos_sensor_8(0x3F0C, 0x01);
	write_cmos_sensor_8(0x3F14, 0x00);
	write_cmos_sensor_8(0x3F80, 0x01);
	write_cmos_sensor_8(0x3F81, 0x90);
	write_cmos_sensor_8(0x3F8C, 0x00);
	write_cmos_sensor_8(0x3F8D, 0x14);
	write_cmos_sensor_8(0x3FF8, 0x01);
	write_cmos_sensor_8(0x3FF9, 0x2A);
	write_cmos_sensor_8(0x3FFE, 0x00);
	write_cmos_sensor_8(0x3FFF, 0x6C);
	/*Integration Setting*/
	write_cmos_sensor_8(0x0202, 0x0E);
	write_cmos_sensor_8(0x0203, 0x6A);
	write_cmos_sensor_8(0x0224, 0x01);
	write_cmos_sensor_8(0x0225, 0xF4);
	write_cmos_sensor_8(0x3FE0, 0x01);
	write_cmos_sensor_8(0x3FE1, 0xF4);
	/*Gain Setting*/
	write_cmos_sensor_8(0x0204, 0x00);
	write_cmos_sensor_8(0x0205, 0x70);
	write_cmos_sensor_8(0x0216, 0x00);
	write_cmos_sensor_8(0x0217, 0x70);
	write_cmos_sensor_8(0x0218, 0x01);
	write_cmos_sensor_8(0x0219, 0x00);
	write_cmos_sensor_8(0x020E, 0x01);
	write_cmos_sensor_8(0x020F, 0x00);
	write_cmos_sensor_8(0x0210, 0x01);
	write_cmos_sensor_8(0x0211, 0x00);
	write_cmos_sensor_8(0x0212, 0x01);
	write_cmos_sensor_8(0x0213, 0x00);
	write_cmos_sensor_8(0x0214, 0x01);
	write_cmos_sensor_8(0x0215, 0x00);
	write_cmos_sensor_8(0x3FE2, 0x00);
	write_cmos_sensor_8(0x3FE3, 0x70);
	write_cmos_sensor_8(0x3FE4, 0x01);
	write_cmos_sensor_8(0x3FE5, 0x00);
	/*PDAF TYPE2 Setting*/
	write_cmos_sensor_8(0x3E20, 0x02);
	write_cmos_sensor_8(0x3E3B, 0x01);
	write_cmos_sensor_8(0x4434, 0x01);
	write_cmos_sensor_8(0x4435, 0xF0);
	#endif
	pr_debug("X\n");
}

static void hs_video_setting(void)
{
	pr_debug("%s E! currefps 120\n", __func__);

	#if USE_BURST_MODE
	imx581_table_write_cmos_sensor(imx581_hs_video_setting,
		sizeof(imx581_hs_video_setting)/sizeof(kal_uint16));
	#else
	/*MIPI output setting*/
	write_cmos_sensor_8(0x0112, 0x0A);
	write_cmos_sensor_8(0x0113, 0x0A);
	write_cmos_sensor_8(0x0114, 0x02);
	/*Line Length PCK Setting*/
	write_cmos_sensor_8(0x0342, 0x15);
	write_cmos_sensor_8(0x0343, 0x00);
	/*Frame Length Lines Setting*/
	write_cmos_sensor_8(0x0340, 0x04);
	write_cmos_sensor_8(0x0341, 0xA6);
	/*ROI Setting*/
	write_cmos_sensor_8(0x0344, 0x00);
	write_cmos_sensor_8(0x0345, 0x00);
	write_cmos_sensor_8(0x0346, 0x02);
	write_cmos_sensor_8(0x0347, 0xE8);
	write_cmos_sensor_8(0x0348, 0x1F);
	write_cmos_sensor_8(0x0349, 0x3F);
	write_cmos_sensor_8(0x034A, 0x14);
	write_cmos_sensor_8(0x034B, 0x87);
	/*Mode Setting*/
	write_cmos_sensor_8(0x0220, 0x62);
	write_cmos_sensor_8(0x0222, 0x01);
	write_cmos_sensor_8(0x0900, 0x01);
	write_cmos_sensor_8(0x0901, 0x44);
	write_cmos_sensor_8(0x0902, 0x08);
	write_cmos_sensor_8(0x3140, 0x00);
	write_cmos_sensor_8(0x3246, 0x89);
	write_cmos_sensor_8(0x3247, 0x89);
	write_cmos_sensor_8(0x3F15, 0x00);
	/*Digital Crop & Scaling*/
	write_cmos_sensor_8(0x0401, 0x00);
	write_cmos_sensor_8(0x0404, 0x00);
	write_cmos_sensor_8(0x0405, 0x10);
	write_cmos_sensor_8(0x0408, 0x00);
	write_cmos_sensor_8(0x0409, 0x00);
	write_cmos_sensor_8(0x040A, 0x00);
	write_cmos_sensor_8(0x040B, 0x00);
	write_cmos_sensor_8(0x040C, 0x07);
	write_cmos_sensor_8(0x040D, 0xD0);
	write_cmos_sensor_8(0x040E, 0x04);
	write_cmos_sensor_8(0x040F, 0x68);
	/*Output Size Setting*/
	write_cmos_sensor_8(0x034C, 0x07);
	write_cmos_sensor_8(0x034D, 0xD0);
	write_cmos_sensor_8(0x034E, 0x04);
	write_cmos_sensor_8(0x034F, 0x68);
	/*Clock Setting*/
	write_cmos_sensor_8(0x0301, 0x05);
	write_cmos_sensor_8(0x0303, 0x04);
	write_cmos_sensor_8(0x0305, 0x04);
	write_cmos_sensor_8(0x0306, 0x01);
	write_cmos_sensor_8(0x0307, 0x40);
	write_cmos_sensor_8(0x030B, 0x04);
	write_cmos_sensor_8(0x030D, 0x0C);
	write_cmos_sensor_8(0x030E, 0x03);
	write_cmos_sensor_8(0x030F, 0xF0);
	write_cmos_sensor_8(0x0310, 0x01);
	/*Other Setting*/
	write_cmos_sensor_8(0x3620, 0x00);
	write_cmos_sensor_8(0x3621, 0x00);
	write_cmos_sensor_8(0x3C11, 0x0C);
	write_cmos_sensor_8(0x3C12, 0x05);
	write_cmos_sensor_8(0x3C13, 0x2C);
	write_cmos_sensor_8(0x3F0C, 0x00);
	write_cmos_sensor_8(0x3F14, 0x00);
	write_cmos_sensor_8(0x3F80, 0x02);
	write_cmos_sensor_8(0x3F81, 0x67);
	write_cmos_sensor_8(0x3F8C, 0x02);
	write_cmos_sensor_8(0x3F8D, 0x44);
	write_cmos_sensor_8(0x3FF8, 0x00);
	write_cmos_sensor_8(0x3FF9, 0x00);
	write_cmos_sensor_8(0x3FFE, 0x01);
	write_cmos_sensor_8(0x3FFF, 0x90);
	/*Integration Setting*/
	write_cmos_sensor_8(0x0202, 0x04);
	write_cmos_sensor_8(0x0203, 0x76);
	write_cmos_sensor_8(0x0224, 0x01);
	write_cmos_sensor_8(0x0225, 0xF4);
	write_cmos_sensor_8(0x3FE0, 0x01);
	write_cmos_sensor_8(0x3FE1, 0xF4);
	/*Gain Setting*/
	write_cmos_sensor_8(0x0204, 0x00);
	write_cmos_sensor_8(0x0205, 0x70);
	write_cmos_sensor_8(0x0216, 0x00);
	write_cmos_sensor_8(0x0217, 0x70);
	write_cmos_sensor_8(0x0218, 0x01);
	write_cmos_sensor_8(0x0219, 0x00);
	write_cmos_sensor_8(0x020E, 0x01);
	write_cmos_sensor_8(0x020F, 0x00);
	write_cmos_sensor_8(0x0210, 0x01);
	write_cmos_sensor_8(0x0211, 0x00);
	write_cmos_sensor_8(0x0212, 0x01);
	write_cmos_sensor_8(0x0213, 0x00);
	write_cmos_sensor_8(0x0214, 0x01);
	write_cmos_sensor_8(0x0215, 0x00);
	write_cmos_sensor_8(0x3FE2, 0x00);
	write_cmos_sensor_8(0x3FE3, 0x70);
	write_cmos_sensor_8(0x3FE4, 0x01);
	write_cmos_sensor_8(0x3FE5, 0x00);
	/*PDAF TYPE1 Setting*/
	write_cmos_sensor_8(0x3E20, 0x01);
	write_cmos_sensor_8(0x3E37, 0x00);
	/*PDAF TYPE2 Setting*/
	write_cmos_sensor_8(0x3E20, 0x01);
	write_cmos_sensor_8(0x3E3B, 0x00);
	write_cmos_sensor_8(0x4434, 0x00);
	write_cmos_sensor_8(0x4435, 0xF8);
	#endif
	pr_debug("X\n");
}

static void slim_video_setting(void)
{
	pr_debug("%s E! 4000*2256@30fps\n", __func__);

	#if USE_BURST_MODE
	imx581_table_write_cmos_sensor(imx581_slim_video_setting,
		sizeof(imx581_slim_video_setting)/sizeof(kal_uint16));
	#else
	/*MIPI output setting*/
	write_cmos_sensor_8(0x0112, 0x0A);
	write_cmos_sensor_8(0x0113, 0x0A);
	write_cmos_sensor_8(0x0114, 0x02);
	/*Line Length PCK Setting*/
	write_cmos_sensor_8(0x0342, 0x1E);
	write_cmos_sensor_8(0x0343, 0xC0);
	/*Frame Length Lines Setting*/
	write_cmos_sensor_8(0x0340, 0x0E);
	write_cmos_sensor_8(0x0341, 0x9A);
	/*ROI Setting*/
	write_cmos_sensor_8(0x0344, 0x00);
	write_cmos_sensor_8(0x0345, 0x00);
	write_cmos_sensor_8(0x0346, 0x02);
	write_cmos_sensor_8(0x0347, 0xE8);
	write_cmos_sensor_8(0x0348, 0x1F);
	write_cmos_sensor_8(0x0349, 0x3F);
	write_cmos_sensor_8(0x034A, 0x14);
	write_cmos_sensor_8(0x034B, 0x87);
	/*Mode Setting*/
	write_cmos_sensor_8(0x0220, 0x62);
	write_cmos_sensor_8(0x0222, 0x01);
	write_cmos_sensor_8(0x0900, 0x01);
	write_cmos_sensor_8(0x0901, 0x22);
	write_cmos_sensor_8(0x0902, 0x08);
	write_cmos_sensor_8(0x3140, 0x00);
	write_cmos_sensor_8(0x3246, 0x81);
	write_cmos_sensor_8(0x3247, 0x81);
	write_cmos_sensor_8(0x3F15, 0x00);
	/*Digital Crop & Scaling*/
	write_cmos_sensor_8(0x0401, 0x00);
	write_cmos_sensor_8(0x0404, 0x00);
	write_cmos_sensor_8(0x0405, 0x10);
	write_cmos_sensor_8(0x0408, 0x00);
	write_cmos_sensor_8(0x0409, 0x00);
	write_cmos_sensor_8(0x040A, 0x00);
	write_cmos_sensor_8(0x040B, 0x00);
	write_cmos_sensor_8(0x040C, 0x0F);
	write_cmos_sensor_8(0x040D, 0xA0);
	write_cmos_sensor_8(0x040E, 0x08);
	write_cmos_sensor_8(0x040F, 0xD0);
	/*Output Size Setting*/
	write_cmos_sensor_8(0x034C, 0x0F);
	write_cmos_sensor_8(0x034D, 0xA0);
	write_cmos_sensor_8(0x034E, 0x08);
	write_cmos_sensor_8(0x034F, 0xD0);
	/*Clock Setting*/
	write_cmos_sensor_8(0x0301, 0x05);
	write_cmos_sensor_8(0x0303, 0x02);
	write_cmos_sensor_8(0x0305, 0x04);
	write_cmos_sensor_8(0x0306, 0x00);
	write_cmos_sensor_8(0x0307, 0xB8);
	write_cmos_sensor_8(0x030B, 0x02);
	write_cmos_sensor_8(0x030D, 0x04);
	write_cmos_sensor_8(0x030E, 0x01);
	write_cmos_sensor_8(0x030F, 0x1C);
	write_cmos_sensor_8(0x0310, 0x01);
	/*Other Setting*/
	write_cmos_sensor_8(0x3620, 0x00);
	write_cmos_sensor_8(0x3621, 0x00);
	write_cmos_sensor_8(0x3C11, 0x04);
	write_cmos_sensor_8(0x3C12, 0x03);
	write_cmos_sensor_8(0x3C13, 0x2D);
	write_cmos_sensor_8(0x3F0C, 0x01);
	write_cmos_sensor_8(0x3F14, 0x00);
	write_cmos_sensor_8(0x3F80, 0x01);
	write_cmos_sensor_8(0x3F81, 0x90);
	write_cmos_sensor_8(0x3F8C, 0x00);
	write_cmos_sensor_8(0x3F8D, 0x14);
	write_cmos_sensor_8(0x3FF8, 0x01);
	write_cmos_sensor_8(0x3FF9, 0x2A);
	write_cmos_sensor_8(0x3FFE, 0x00);
	write_cmos_sensor_8(0x3FFF, 0x6C);
	/*Integration Setting*/
	write_cmos_sensor_8(0x0202, 0x0E);
	write_cmos_sensor_8(0x0203, 0x6A);
	write_cmos_sensor_8(0x0224, 0x01);
	write_cmos_sensor_8(0x0225, 0xF4);
	write_cmos_sensor_8(0x3FE0, 0x01);
	write_cmos_sensor_8(0x3FE1, 0xF4);
	/*Gain Setting*/
	write_cmos_sensor_8(0x0204, 0x00);
	write_cmos_sensor_8(0x0205, 0x70);
	write_cmos_sensor_8(0x0216, 0x00);
	write_cmos_sensor_8(0x0217, 0x70);
	write_cmos_sensor_8(0x0218, 0x01);
	write_cmos_sensor_8(0x0219, 0x00);
	write_cmos_sensor_8(0x020E, 0x01);
	write_cmos_sensor_8(0x020F, 0x00);
	write_cmos_sensor_8(0x0210, 0x01);
	write_cmos_sensor_8(0x0211, 0x00);
	write_cmos_sensor_8(0x0212, 0x01);
	write_cmos_sensor_8(0x0213, 0x00);
	write_cmos_sensor_8(0x0214, 0x01);
	write_cmos_sensor_8(0x0215, 0x00);
	write_cmos_sensor_8(0x3FE2, 0x00);
	write_cmos_sensor_8(0x3FE3, 0x70);
	write_cmos_sensor_8(0x3FE4, 0x01);
	write_cmos_sensor_8(0x3FE5, 0x00);
	/*PDAF TYPE2 Setting*/
	write_cmos_sensor_8(0x3E20, 0x02);
	write_cmos_sensor_8(0x3E3B, 0x01);
	write_cmos_sensor_8(0x4434, 0x01);
	write_cmos_sensor_8(0x4435, 0xF0);
	#endif
	pr_debug("X\n");
}


/*full size 30fps*/
static void custom3_setting(void)
{
	pr_debug("%s full size 30 fps E!\n", __func__);
	/*************MIPI output setting************/
	imx581_table_write_cmos_sensor(imx581_custom3_setting,
		sizeof(imx581_custom3_setting)/sizeof(kal_uint16));

	if (otp_flag == OTP_QSC_NONE) {
		pr_info("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(0x3621, 0x00);
	}

	pr_debug("%s 30 fpsX\n", __func__);
}

static void custom4_setting(void)
{
	pr_debug("%s 720p@240 fps E! currefps\n", __func__);

	imx581_table_write_cmos_sensor(imx581_custom4_setting,
		sizeof(imx581_custom4_setting)/sizeof(kal_uint16));

	pr_debug("X\n");
}

static void custom5_setting(void)
{
	pr_debug("%s 720p@240 fps E! currefps\n", __func__);

	imx581_table_write_cmos_sensor(imx581_custom5_setting,
		sizeof(imx581_custom5_setting)/sizeof(kal_uint16));

	pr_debug("X\n");
}

static void custom6_setting(void)
{
	pr_debug("%s 720p@240 fps E! currefps\n", __func__);

	imx581_table_write_cmos_sensor(imx581_custom6_setting,
		sizeof(imx581_custom6_setting)/sizeof(kal_uint16));

	pr_debug("X\n");
}

static void custom7_setting(void)
{
	pr_debug("%s 720p@240 fps E! currefps\n", __func__);

	imx581_table_write_cmos_sensor(imx581_custom7_setting,
		sizeof(imx581_custom7_setting)/sizeof(kal_uint16));

	pr_debug("X\n");
}

static void custom8_setting(void)
{
	pr_debug("%s 720p@240 fps E! currefps\n", __func__);

	imx581_table_write_cmos_sensor(imx581_custom8_setting,
		sizeof(imx581_custom8_setting)/sizeof(kal_uint16));

	pr_debug("X\n");
}

static void custom9_setting(void)
{
	pr_debug("%s 720p@240 fps E! currefps\n", __func__);

	imx581_table_write_cmos_sensor(imx581_custom9_setting,
		sizeof(imx581_custom9_setting)/sizeof(kal_uint16));

	pr_debug("X\n");
}

static void custom10_setting(void)
{
	pr_debug("%s 720p@240 fps E! currefps\n", __func__);

	imx581_table_write_cmos_sensor(imx581_custom10_setting,
		sizeof(imx581_custom10_setting)/sizeof(kal_uint16));

	pr_debug("X\n");
}

static void custom1_setting(void)
{
	pr_debug("%s fhd 240fps E! currefps\n", __func__);
	/*************MIPI output setting************/
	imx581_table_write_cmos_sensor(imx581_custom1_setting,
		sizeof(imx581_custom1_setting)/sizeof(kal_uint16));
	pr_debug("X");
}

static void custom2_setting(void)
{
	pr_debug("%s 3840*2160@60fps E! currefps\n", __func__);
	/*************MIPI output setting************/

	imx581_table_write_cmos_sensor(imx581_normal_video_setting_4K60FPS,
		sizeof(imx581_normal_video_setting_4K60FPS)/sizeof(kal_uint16));

	pr_debug("X");
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
	int pcbVer;
	/*sensor have two i2c address 0x34 & 0x20,
	 *we should detect the module used i2c address
	 */

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor_8(0x0016) << 8)
					| read_cmos_sensor_8(0x0017));
			pr_debug(
				"read_0x0000=0x%x, 0x0001=0x%x,0x0000_0001=0x%x\n",
				read_cmos_sensor_8(0x0016),
				read_cmos_sensor_8(0x0017),
				read_cmos_sensor(0x0000));
			pr_debug("The real sensor id is %x",*sensor_id);
			pcbVer = get_PCB_Version();
			pr_debug("Now the pcb version is %d",pcbVer);
			if(PCB_VERSION_T0 == pcbVer) {
				pr_debug("Now the pcb version is T0");
				if(*sensor_id == 0 || *sensor_id == 0x0586)
					*sensor_id = imgsensor_info.sensor_id;
			}
			if (*sensor_id == imgsensor_info.sensor_id) {
				pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				//read_sensor_Cali();
				Read_Eeprom_Data();
				return ERROR_NONE;
			}

			pr_debug("Read sensor id fail, id: 0x%x\n",
				imgsensor.i2c_write_id);
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
	int pcbVer = 0;

	pr_debug("%s +\n", __func__);
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
			pr_debug("The real sensor id is %x",sensor_id);
			pcbVer = get_PCB_Version();
			pr_debug("Now the pcb version is %d",pcbVer);
			if(PCB_VERSION_T0 == pcbVer) {
				pr_debug("Now the pcb version is T0");
				if(sensor_id == 0 || sensor_id == 0x0586)
					sensor_id = imgsensor_info.sensor_id;
			}
			if (sensor_id == imgsensor_info.sensor_id) {
				pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			pr_debug("Read sensor id fail, id: 0x%x\n",
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
	if ((!qsc_flag) && (oplus_is_system_camera(OPLUS_CHECK_IS_SYSTEM_CAM))) {
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
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = 0;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	pr_debug("%s -\n", __func__);

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
	pr_debug("E\n");
	/* No Need to implement this function */
	streaming_control(KAL_FALSE);
	qsc_flag = 0;
	return ERROR_NONE;
} /* close */


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
	pr_debug("%s E\n", __func__);

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
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		pr_debug(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.cap.max_framerate / 10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;

	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

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
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

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
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s. 720P@240FPS\n", __func__);

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
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* slim_video */


static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s.\n", __func__);

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
}	/* custom1 */

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s.\n", __func__);

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
}	/* custom2 */

static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	imgsensor.pclk = imgsensor_info.custom3.pclk;
	imgsensor.line_length = imgsensor_info.custom3.linelength;
	imgsensor.frame_length = imgsensor_info.custom3.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	if (!qsc_flag) {
		pr_debug("write_sensor_QSC Start\n");
		write_sensor_QSC();
		pr_debug("write_sensor_QSC End\n");
		qsc_flag = 1;
	}
	custom3_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* custom3 */

static kal_uint32 custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s. 720P@240FPS\n", __func__);

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
}	/* custom4 */

static kal_uint32 custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s. binningP@18FPS\n", __func__);

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
}	/* custom5 */

static kal_uint32 custom6(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s. binningP@60FPS\n", __func__);

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
}	/* custom6 */

static kal_uint32 custom7(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s. binningP@60FPS\n", __func__);

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
}	/* custom7 */

static kal_uint32 custom8(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s. binningP@60FPS\n", __func__);

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
}	/* custom8 */

static kal_uint32 custom9(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s. binningP@60FPS\n", __func__);

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
}	/* custom9 */

static kal_uint32 custom10(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s. binningP@60FPS\n", __func__);

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
}	/* custom10 */

static kal_uint32
get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	pr_debug("E\n");
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
	pr_debug("scenario_id = %d\n", scenario_id);

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
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
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
}	/*	get_info  */

static kal_uint32 seamless_switch(enum MSDK_SCENARIO_ID_ENUM scenario_id,
	kal_uint32 shutter, kal_uint32 gain,
	kal_uint32 shutter_2ndframe, kal_uint32 gain_2ndframe)
{
	UINT32 delay_time = 0;
	if (scenario_id == MSDK_SCENARIO_ID_CAMERA_PREVIEW) {
		delay_time = 1000 / (imgsensor_info.custom5.pclk / imgsensor.frame_length / imgsensor_info.custom5.linelength);
	} else if (scenario_id == MSDK_SCENARIO_ID_CUSTOM5) {
		delay_time = 1000 / (imgsensor_info.pre.pclk / imgsensor.frame_length / imgsensor_info.pre.linelength);
	}
	mdelay(delay_time);
	pr_debug("delay:%u", delay_time);

	imgsensor.current_scenario_id = scenario_id;
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	{
		pr_debug("seamless switch to custom5_setting begin!\n");
		spin_lock(&imgsensor_drv_lock);
		imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
		imgsensor.pclk = imgsensor_info.pre.pclk;
		imgsensor.line_length = imgsensor_info.pre.linelength;
		imgsensor.frame_length = imgsensor_info.pre.framelength;
		imgsensor.min_frame_length = imgsensor_info.pre.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
		spin_unlock(&imgsensor_drv_lock);
		if (gain != 0) {
			imx581_seamless_preview[51] = (gain >> 8) & 0xff;
			imx581_seamless_preview[53] = gain & 0xff;
		}
		if (shutter != 0) {
			imx581_seamless_preview[47] = (shutter >> 8) & 0xff;
			imx581_seamless_preview[49] = shutter & 0xff;
		}

		write_cmos_sensor_8(0x0104, 0x01);
		imx581_table_write_cmos_sensor(imx581_seamless_preview,
		sizeof(imx581_seamless_preview) / sizeof(kal_uint16));
		write_cmos_sensor_8(0x0104, 0x00);
		pr_debug("seamless switch to custom5_setting end!\n");
	}
	break;
	case MSDK_SCENARIO_ID_CUSTOM5:
	{
		pr_debug("seamless switch to pre_setting begin!\n");
		spin_lock(&imgsensor_drv_lock);
		imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
		imgsensor.pclk = imgsensor_info.custom5.pclk;
		imgsensor.line_length = imgsensor_info.custom5.linelength;
		imgsensor.frame_length = imgsensor_info.custom5.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
		spin_unlock(&imgsensor_drv_lock);
		if (gain != 0) {
			imx581_seamless_custom5[51] = (gain >> 8) & 0xff;
			imx581_seamless_custom5[53] = gain & 0xff;
		}
		if (shutter != 0) {
			imx581_seamless_custom5[47] = (shutter >> 8) & 0xff;
			imx581_seamless_custom5[49] = shutter & 0xff;
		}

		write_cmos_sensor_8(0x0104, 0x01);
		imx581_table_write_cmos_sensor(imx581_seamless_custom5,
		sizeof(imx581_seamless_custom5) / sizeof(kal_uint16));
		write_cmos_sensor_8(0x0104, 0x00);
		pr_debug("seamless switch to pre_setting end!\n");
	}
	break;
	default:
	{
		pr_info( "error! wrong setting in set_seamless_switch = %d", scenario_id);
		return 0xff;
	}
	}

	if (shutter_2ndframe != 0)
		set_shutter(shutter_2ndframe);
	if (gain_2ndframe != 0)
		set_gain(gain_2ndframe);

	return 0;
}

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("scenario_id = %d\n", scenario_id);
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
		pr_debug("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}

	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	pr_debug("framerate = %d\n ", framerate);
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

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	spin_lock(&imgsensor_drv_lock);
	if (enable) /*enable auto flicker*/ {
		imgsensor.autoflicker_en = KAL_TRUE;
		pr_debug("enable! fps = %d", framerate);
	} else {
		 /*Cancel Auto flick*/
		imgsensor.autoflicker_en = KAL_FALSE;
	}
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
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		pr_debug(
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
		? (frame_length - imgsensor_info.custom3.framelength) : 0;
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
		? (frame_length - imgsensor_info.custom4.framelength) : 0;
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
		? (frame_length - imgsensor_info.custom5.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.custom5.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM6:
		frame_length = imgsensor_info.custom6.pclk / framerate * 10
				/ imgsensor_info.custom6.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom6.framelength)
		? (frame_length - imgsensor_info.custom6.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.custom6.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM7:
		frame_length = imgsensor_info.custom7.pclk / framerate * 10
				/ imgsensor_info.custom7.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom7.framelength)
		? (frame_length - imgsensor_info.custom7.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.custom7.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM8:
		frame_length = imgsensor_info.custom8.pclk / framerate * 10
				/ imgsensor_info.custom8.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom8.framelength)
		? (frame_length - imgsensor_info.custom8.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.custom8.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM9:
		frame_length = imgsensor_info.custom9.pclk / framerate * 10
				/ imgsensor_info.custom9.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom9.framelength)
		? (frame_length - imgsensor_info.custom9.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.custom9.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM10:
		frame_length = imgsensor_info.custom10.pclk / framerate * 10
				/ imgsensor_info.custom10.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom10.framelength)
		? (frame_length - imgsensor_info.custom10.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.custom10.framelength
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
		pr_debug("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	pr_debug("scenario_id = %d\n", scenario_id);

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
    kal_uint16 Color_R, Color_Gr, Color_Gb, Color_B;
    pr_debug("set_test_pattern enum: %d\n", modes);

    if (modes) {
        write_cmos_sensor_8(0x0600, modes>>4);
        write_cmos_sensor_8(0x0601, modes);
        if (modes == 1 && (pTestpatterndata != NULL)) { //Solid Color
            Color_R = (pTestpatterndata->COLOR_R >> 16) & 0xFFFF;
            Color_Gr = (pTestpatterndata->COLOR_Gr >> 16) & 0xFFFF;
            Color_B = (pTestpatterndata->COLOR_B >> 16) & 0xFFFF;
            Color_Gb = (pTestpatterndata->COLOR_Gb >> 16) & 0xFFFF;
            write_cmos_sensor_8(0x0602, Color_R >> 8);
            write_cmos_sensor_8(0x0603, Color_R & 0xFF);
            write_cmos_sensor_8(0x0604, Color_Gr >> 8);
            write_cmos_sensor_8(0x0605, Color_Gr & 0xFF);
            write_cmos_sensor_8(0x0606, Color_B >> 8);
            write_cmos_sensor_8(0x0607, Color_B & 0xFF);
            write_cmos_sensor_8(0x0608, Color_Gb >> 8);
            write_cmos_sensor_8(0x0609, Color_Gb & 0xFF);
        }
    } else {
        write_cmos_sensor_8(0x0600, 0x0000); /*No pattern*/
        write_cmos_sensor_8(0x0601, 0x0000);
    }

    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = modes;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

static kal_uint32 ana_gain_table[] = {
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

static kal_uint32 get_sensor_temperature(void)
{
	UINT8 temperature;
	INT32 temperature_convert;

	temperature = read_cmos_sensor_8(0x013a);

	if (temperature >= 0x0 && temperature <= 0x4F)
		temperature_convert = temperature;
	else if (temperature >= 0x50 && temperature <= 0x7F)
		temperature_convert = 80;
	else if (temperature >= 0x80 && temperature <= 0xEC)
		temperature_convert = -20;
	else
		temperature_convert = (INT8) temperature;

	/* LOG_INF("temp_c(%d), read_reg(%d)\n", */
	/* temperature_convert, temperature); */

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
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
	uint32_t *pAeCtrls;
	uint32_t *pScenarios;
	/* SET_SENSOR_AWB_GAIN *pSetSensorAWB
	 *  = (SET_SENSOR_AWB_GAIN *)feature_para;
	 */
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
		= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*pr_debug("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
	case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
		if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL
								|| *(feature_data + 0) == 0) {
			*(feature_data + 0) =
				sizeof(ana_gain_table)/sizeof(kal_uint32);
		} else {
			memcpy((void *)(uintptr_t) (*(feature_data + 1)),
			(void *)ana_gain_table,
			sizeof(ana_gain_table)/sizeof(kal_uint32));
		}
		break;
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
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
		*(feature_data + 2) = imgsensor_info.exp_step;
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

	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 2055666;
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
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_32 = get_sensor_temperature();
		*feature_para_len = 4;
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
		pr_debug("SENSOR_FEATURE_GET_PDAF_DATA\n");
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((UINT8)*feature_data, (struct SET_SENSOR_PATTERN_SOLID_COLOR *) (feature_data+1));
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		pr_debug("current fps :%d\n", (UINT32)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		pr_debug("ihdr enable :%d\n", (BOOL)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_mode = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
	#if 0
		pr_debug("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32)*feature_data);
	#endif
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
		pr_debug("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
			(UINT16) *feature_data);
		PDAFinfo =
		  (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM7:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG: //4000*3000
			imgsensor_pd_info_binning.i4BlockNumX = 496;
			imgsensor_pd_info_binning.i4BlockNumY = 186;
			memcpy((void *)PDAFinfo,
				(void *)&imgsensor_pd_info_binning,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM6:
		case MSDK_SCENARIO_ID_CUSTOM8:  //4000*2256
			imgsensor_pd_info_binning.i4BlockNumX = 496;
			imgsensor_pd_info_binning.i4BlockNumY = 140;
			memcpy((void *)PDAFinfo,
				(void *)&imgsensor_pd_info_binning,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2: // 3840*2160
			imgsensor_pd_info_binning.i4BlockNumX = 480;
			imgsensor_pd_info_binning.i4BlockNumY = 134;
			memcpy((void *)PDAFinfo,
				(void *)&imgsensor_pd_info_binning,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		default:
            memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info_binning,
                            sizeof(struct SET_PD_BLOCK_INFO_T));
            break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		pr_debug(
		"SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
			(UINT16) *feature_data);
		/*PDAF capacity enable or not, 2p8 only full size support PDAF*/
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM6:
		case MSDK_SCENARIO_ID_CUSTOM7:
        case MSDK_SCENARIO_ID_CUSTOM8:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CUSTOM4:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
		pr_debug("SENSOR_FEATURE_GET_PDAF_REG_SETTING %d",
			(*feature_para_len));
		imx581_get_pdaf_reg_setting((*feature_para_len) / sizeof(UINT32)
					   , feature_data_16);
		break;
	case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
		pr_debug("SENSOR_FEATURE_SET_PDAF_REG_SETTING %d",
			(*feature_para_len));
		imx581_set_pdaf_reg_setting((*feature_para_len) / sizeof(UINT32)
					   , feature_data_16);
		break;
	case SENSOR_FEATURE_SET_PDAF:
		pr_debug("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		pr_debug("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data,
			(UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data),
					(UINT16) (*(feature_data + 1)),
					(BOOL) (*(feature_data + 2)));
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
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		pr_debug("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
		#if 0
		ihdr_write_shutter((UINT16)*feature_data,
				   (UINT16)*(feature_data+1));
		#endif
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
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CUSTOM5:
			*feature_return_para_32 = 1; /*BINNING_NONE*/
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM4:
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
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
	}
break;

	case SENSOR_FEATURE_GET_VC_INFO:
	#if 0
		pr_debug("SENSOR_FEATURE_GET_VC_INFO %d\n",
			(UINT16)*feature_data);
	#endif
		pvcinfo =
		 (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM7:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM8:
		case MSDK_SCENARIO_ID_CUSTOM6:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		default:
			pr_info("error: get wrong vc_INFO id = %d",
			*feature_data_32);
            memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
                   sizeof(struct SENSOR_VC_INFO_STRUCT));
            break;
		}
	break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		/* modify to separate 3hdr and remosaic */
		if ((imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3) || (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM5)) {
			/*write AWB gain to sensor*/
			feedback_awbgain((UINT32)*(feature_data_32 + 1),
					(UINT32)*(feature_data_32 + 2));
		} else {
			imx581_awb_gain(
				(struct SET_SENSOR_AWB_GAIN *) feature_para);
		}
		break;
	case SENSOR_FEATURE_SET_LSC_TBL:
		{
		kal_uint8 index =
			*(((kal_uint8 *)feature_para) + (*feature_para_len));

		imx581_set_lsc_reg_setting(index, feature_data_16,
					  (*feature_para_len)/sizeof(UINT16));
		}
		break;
	case SENSOR_FEATURE_SEAMLESS_SWITCH:
	{
		if ((feature_data + 1) != NULL) {
			pAeCtrls =
			(MUINT32 *)((uintptr_t)(*(feature_data + 1)));
		} else {
			pr_debug(
			"warning! no ae_ctrl input");
		}
		if (feature_data == NULL) {
			pr_info("error! input scenario is null!");
			return ERROR_INVALID_SCENARIO_ID;
		}

		if (pAeCtrls != NULL) {
			seamless_switch((*feature_data),
					*pAeCtrls, *(pAeCtrls + 1),
					*(pAeCtrls + 4), *(pAeCtrls + 5));
		} else {
			seamless_switch((*feature_data),
					0, 0, 0, 0);
		}

	}
		break;
	case SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS:
		if ((feature_data + 1) != NULL) {
			pScenarios =
			(MUINT32 *)((uintptr_t)(*(feature_data + 1)));
		} else {
			return ERROR_INVALID_SCENARIO_ID;
		}
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*pScenarios = MSDK_SCENARIO_ID_CUSTOM5;
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			*pScenarios = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CUSTOM6:
		case MSDK_SCENARIO_ID_CUSTOM7:
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

UINT32 IMX581_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
} /* IMX581_MIPI_RAW_SensorInit */
