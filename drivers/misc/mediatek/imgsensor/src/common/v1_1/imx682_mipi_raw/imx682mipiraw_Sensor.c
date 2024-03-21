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
 *	 IMX682mipi_Sensor.c
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
#define PFX "IMX682_camera_sensor"
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
#include "imgsensor_common.h"

#include "imx682mipiraw_Sensor.h"
#include "imx682_eeprom.h"

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

/***************Modify Following Strings for Debug**********************/
#define PFX "IMX682_camera_sensor"
#define LOG_1 LOG_INF("IMX682,MIPI 4LANE\n")
/****************************   Modify end	**************************/
#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)
#define USE_BURST_MODE 1
#define SUPPORT_LRC 0  //imx682 pd cal didn't use LRC

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define DEVICE_VERSION_IMX682     "imx682"
#define MODULE_ID_OFFSET 0x0000
extern void Oplusimgsensor_Registdeviceinfo(char *name, char *version, u8 module_id);
static kal_uint8 deviceInfo_register_value = 0x00;
static char sensor_name_golden_result[16] = {0};
#endif
static kal_uint32 streaming_control(kal_bool enable);

static kal_uint8 qsc_flag = 0;
#if USE_BURST_MODE
static kal_uint16 imx682_table_write_cmos_sensor(
		kal_uint16 * para, kal_uint32 len);
static kal_uint16 imx682_burst_write_cmos_sensor(
		kal_uint16 * para, kal_uint32 len);
#endif
static DEFINE_SPINLOCK(imgsensor_drv_lock);


static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX682_SENSOR_ID,

	.checksum_value = 0x60d720a7,

	.pre = { /* reg_P 30fps   */
		.pclk = 1062000000,
		.linelength = 9432,
		.framelength = 3753,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4624,
		.grabwindow_height = 3472,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 800230000,
		.max_framerate = 300,
	},

	.cap = { /* reg_P 30fps   */
		.pclk = 1062000000,
		.linelength = 9432,
		.framelength = 3753,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4624,
		.grabwindow_height = 3472,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 800230000,
		.max_framerate = 300,
	},

	.normal_video = { /* reg_U 30fps   */
		.pclk = 798000000,
		.linelength = 9432,
		.framelength = 2820,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4608,
		.grabwindow_height = 2592,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 588340000,
		.max_framerate = 300,
	},

	.hs_video = { /* reg_M 240fps   */
		.pclk = 1098000000,
		.linelength = 2728,
		.framelength = 1676,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1296,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1213710000,
		.max_framerate = 2400,
	},

	.slim_video = { /* reg_N 30fps   */
		.pclk = 771000000,
		.linelength = 9432,
		.framelength = 2724,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4608,
		.grabwindow_height = 2592,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 514290000,
		.max_framerate = 300,
	},

	.custom1 = { /* reg_P-1 24fps   */
		.pclk = 819000000,
		.linelength = 9432,
		.framelength = 3618,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4624,
		.grabwindow_height = 3472,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 549260000,
		.max_framerate = 240,
	},

	.custom2 = { /* reg_J 120fps   */
		.pclk = 549000000,
		.linelength = 2728,
		.framelength = 1676,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1296,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 594510000,
		.max_framerate = 1200,
	},

	.custom3 = { /* reg_R 60fps   */
		.pclk = 1071000000,
		.linelength = 9432,
		.framelength = 1892,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3056,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 497830000,
		.max_framerate = 600,
	},

	.custom4 = { /* reg_O 15fps   */
		.pclk = 1098000000,
		.linelength = 10288,
		.framelength = 7115,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 9248,
		.grabwindow_height = 6944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1246630000,
		.max_framerate = 150,
	},

	.custom5 = { /* reg_S 30fps   */
		.pclk = 1062000000,
		.linelength = 9432,
		.framelength = 3753,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4624,
		.grabwindow_height = 3472,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 800230000,
		.max_framerate = 300,
	},

	.margin = 48,		/* sensor framelength & shutter margin */
	.min_shutter = 4,	/* min shutter */
	.min_gain = 64, /*1x gain*/
	.max_gain = 4096, /*64x gain*/
	.min_gain_iso = 100,
	.gain_step = 1,
	.gain_type = 0,/*to be modify,no gain table for sony*/
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	/*jiangtao.ren@Camera add for support af temperature compensation 20191101*/
	.temperature_support = 1,/* 1, support; 0,not support */
	.sensor_mode_num = 10,	/* support sensor mode num */

	.cap_delay_frame = 2,	/* enter capture delay frame num */
	.pre_delay_frame = 2,	/* enter preview delay frame num */
	.video_delay_frame = 2,	/* enter video delay frame num */
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,	/* enter slim video delay frame num */
	.custom1_delay_frame = 2,	/* enter custom1 delay frame num */
	.custom2_delay_frame = 2,	/* enter custom2 delay frame num */
	.custom3_delay_frame = 2,	/* enter custom2 delay frame num */
	.custom4_delay_frame = 2,	/* enter custom2 delay frame num */
	.custom5_delay_frame = 2,	/* enter custom2 delay frame num */
	.frame_time_delay_frame = 3,

	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	/* .mipi_sensor_type = MIPI_OPHY_NCSI2, */
	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_sensor_type = MIPI_CPHY, /* 0,MIPI_OPHY_NCSI2; 1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 0,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,//SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,
	.mclk = 24, /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */ //24 SENSOR_OUTPUT_FORMAT_RAW_Gb

	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.i2c_addr_table = {0x20, 0xff},
	/* record sensor support all write id addr,
	 * only supprt 4 must end with 0xff
	 */
	.i2c_speed = 1000, /* i2c read/write speed */
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,	/* mirrorflip information */
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
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x20, /* record current sensor's i2c write id */
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
	{9248, 6944, 0,    0, 9248, 6944, 4624, 3472,
	0, 0, 4624, 3472,  0,  0, 4624, 3472}, /* Preview */
	{9248, 6944, 0,    0, 9248, 6944, 4624, 3472,
	0, 0, 4624, 3472,  0,  0, 4624, 3472}, /* capture */
	{9248, 6944, 0,  880, 9248, 5184, 4624, 2592,
	8, 0, 4608, 2592,  0,  0, 4608, 2592}, /* normal video */
	{9248, 6944, 0,  880, 9248, 5184, 2312, 1296,
	4, 0, 2304, 1296,  0,  0, 2304, 1296}, /* hs_video */
	{9248, 6944, 0,  880, 9248, 5184, 4624, 2592,
	8, 0, 4608, 2592,  0,  0, 4608, 2592}, /* slim video */
	{9248, 6944, 0,    0, 9248, 6944, 4624, 3472,
	0, 0, 4624, 3472,  0,  0, 4624, 3472}, /* custom1 */
	{9248, 6944, 0,  880, 9248, 5184, 2312, 1296,
	4, 0, 2304, 1296,  0,  0, 2304, 1296}, /* custom2 */
	{9248, 6944, 40,  880, 9168, 5184, 3056, 1728,
	8, 0, 3056, 1728,  0,  0, 3056, 1728}, /* custom3 */
	{9248, 6944, 0,    0, 9248, 6944, 9248, 6944,
	0, 0, 9248, 6944,  0,  0, 9248, 6944}, /* custom4 */ //remosic
	{9248, 6944, 0,    0, 9248, 6944, 4624, 3472,
	0, 0, 4624, 3472,  0,  0, 4624, 3472}, /* custom5 */
};
 /*VC1 for HDR(DT=0X35), VC2 for PDAF(DT=0X36), unit : 10bit */
static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3] = {
	/* Preview mode setting */
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1200, 0x0D80, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x30, 0x05A0, 0x06B8, 0x00, 0x00, 0x0000, 0x0000},
	/* Normal_Video mode setting */
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1200, 0x0A20, 0x00, 0x00, 0x0000, 0x0000,
	 0x00, 0x30, 0x05A0, 0x0508, 0x00, 0x00, 0x0000, 0x0000},
	/* Video60fps mode setting */
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1200, 0x0A20, 0x00, 0x00, 0x0000, 0x0000,
	 0x00, 0x30, 0x03B6, 0x0358, 0x00, 0x00, 0x0000, 0x0000}
};


/* If mirror flip */
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 17,
	.i4OffsetY = 16,
	.i4PitchX  =  8,
	.i4PitchY  = 16,
	.i4PairNum  = 8,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = { {20, 17}, {18, 19}, {22, 21}, {24, 23},
		   {20, 25}, {18, 27}, {22, 29}, {24, 31} },
	.i4PosR = { {19, 17}, {17, 19}, {21, 21}, {23, 23},
		   {19, 25}, {17, 27}, {21, 29}, {23, 31} },
	.i4BlockNumX = 574,
	.i4BlockNumY = 215,
	.iMirrorFlip = 0,
	.i4Crop = { {0, 0}, {0, 0}, {8, 440}, {0, 0}, {8, 440},
		    {0, 0}, {0, 0}, {784, 872}, {0, 0}, {0, 0} },
};

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static kal_uint16 read_module_id(void)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(MODULE_ID_OFFSET >> 8) , (char)(MODULE_ID_OFFSET & 0xFF) };

	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,0xA0/*EEPROM_READ_ID*/);
	if (get_byte == 0) {
		iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, 0xA0/*EEPROM_READ_ID*/);
	}

	return get_byte;
}
#endif

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

static void imx682_get_pdaf_reg_setting(MUINT32 regNum, kal_uint16 *regDa)
{
	int i, idx;

	for (i = 0; i < regNum; i++) {
		idx = 2 * i;
		regDa[idx + 1] = read_cmos_sensor_8(regDa[idx]);
		LOG_INF("%x %x", regDa[idx], regDa[idx+1]);
	}
}
static void imx682_set_pdaf_reg_setting(MUINT32 regNum, kal_uint16 *regDa)
{
	int i, idx;

	for (i = 0; i < regNum; i++) {
		idx = 2 * i;
		write_cmos_sensor_8(regDa[idx], regDa[idx + 1]);
		LOG_INF("%x %x", regDa[idx], regDa[idx+1]);
	}
}

static kal_uint16 read_cmos_eeprom_8(kal_uint16 addr)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 1, 0xA0);
	return get_byte;
}

#ifndef OPLUS_FEATURE_CAMERA_COMMON //PHB add for compiler error
#define  CAMERA_MODULE_INFO_LENGTH  (8)
static kal_uint8 gImx682_SN[CAMERA_MODULE_SN_LENGTH];
static kal_uint8 gImx682_CamInfo[CAMERA_MODULE_INFO_LENGTH];
static void read_eeprom_CamInfo(void)
{
	kal_uint16 idx = 0;
	kal_uint8 get_byte[12];
	for (idx = 0; idx <12; idx++) {
		char pusendcmd[2] = {0x00 , (char)((0x00 + idx) & 0xFF) };
		iReadRegI2C(pusendcmd , 2, (u8*)&get_byte[idx],1, 0xA0);
		LOG_INF("imx682_info[%d]: 0x%x\n", idx, get_byte[idx]);
	}

	gImx682_CamInfo[0] = get_byte[0];
	gImx682_CamInfo[1] = get_byte[1];
	gImx682_CamInfo[2] = get_byte[6];
	gImx682_CamInfo[3] = get_byte[7];
	gImx682_CamInfo[4] = get_byte[8];
	gImx682_CamInfo[5] = get_byte[9];
	gImx682_CamInfo[6] = get_byte[10];
	gImx682_CamInfo[7] = get_byte[11];
}

static void read_eeprom_SN(void)
{
	kal_uint16 idx = 0;
	kal_uint8 *get_byte= &gImx682_SN[0];
	for (idx = 0; idx <CAMERA_MODULE_SN_LENGTH; idx++) {
		char pusendcmd[2] = {0x00 , (char)((0xB0 + idx) & 0xFF) };
		iReadRegI2C(pusendcmd , 2, (u8*)&get_byte[idx],1, 0xA0);
		LOG_INF("imx682_SN[%d]: 0x%x  0x%x\n", idx, get_byte[idx], gImx682_SN[idx]);
	}
}

#define   WRITE_DATA_MAX_LENGTH     (16)
static kal_int32 table_write_eeprom_30Bytes(kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	kal_int32 ret = IMGSENSOR_RETURN_SUCCESS;
	char pusendcmd[WRITE_DATA_MAX_LENGTH+2];
	pusendcmd[0] = (char)(addr >> 8);
	pusendcmd[1] = (char)(addr & 0xFF);

	memcpy(&pusendcmd[2], para, len);

	ret = iBurstWriteReg((kal_uint8 *)pusendcmd , (len + 2), 0xA0);

	return ret;
}

static kal_int32 write_eeprom_protect(kal_uint16 enable)
{
	kal_int32 ret = IMGSENSOR_RETURN_SUCCESS;
	char pusendcmd[3];
	pusendcmd[0] = 0x80;
	pusendcmd[1] = 0x00;
	if (enable)
		pusendcmd[2] = 0xE0;
	else
		pusendcmd[2] = 0x00;

	ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, 0xA0);

	return ret;
}

static kal_int32 write_Module_data(ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereodata)
{
	kal_int32  ret = IMGSENSOR_RETURN_SUCCESS;
	kal_uint16 data_base, data_length;
	kal_uint32 idx, idy;
	kal_uint8 *pData;
	UINT32 i = 0;
	if(pStereodata != NULL) {
		LOG_INF("imx682 SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
                       pStereodata->uSensorId,
                       pStereodata->uDeviceId,
                       pStereodata->baseAddr,
                       pStereodata->dataLength);

		data_base = pStereodata->baseAddr;
		data_length = pStereodata->dataLength;
		pData = pStereodata->uData;
		if ((pStereodata->uSensorId == IMX682_SENSOR_ID)
				&& (data_base == 0x26D0)
				&& ((data_length == DUALCAM_CALI_DATA_LENGTH) || (data_length == DUALCAM_CALI_DATA_LENGTH_QCOM_MAIN))) {
			LOG_INF("imx682 Write: %x %x %x %x %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556],
					pData[1557], pData[1558], pData[1559], pData[1560]);
			idx = data_length/WRITE_DATA_MAX_LENGTH;
			idy = data_length%WRITE_DATA_MAX_LENGTH;
			/* close write protect */
			write_eeprom_protect(0);
			msleep(6);
			for (i = 0; i < idx; i++ ) {
				ret = table_write_eeprom_30Bytes((data_base+WRITE_DATA_MAX_LENGTH*i),
					    &pData[WRITE_DATA_MAX_LENGTH*i], WRITE_DATA_MAX_LENGTH);
				if (ret != IMGSENSOR_RETURN_SUCCESS) {
				    pr_err("write_eeprom error: i= %d\n", i);
					/* open write protect */
					write_eeprom_protect(1);
					msleep(6);
					return IMGSENSOR_RETURN_ERROR;
				}
				msleep(6);
			}
			ret = table_write_eeprom_30Bytes((data_base+WRITE_DATA_MAX_LENGTH*idx),
				      &pData[WRITE_DATA_MAX_LENGTH*idx], idy);
			if (ret != IMGSENSOR_RETURN_SUCCESS) {
				pr_err("write_eeprom error: idx= %d idy= %d\n", idx, idy);
				/* open write protect */
				write_eeprom_protect(1);
				msleep(6);
				return IMGSENSOR_RETURN_ERROR;
			}
			msleep(6);
			/* open write protect */
			write_eeprom_protect(1);
			msleep(6);
			LOG_INF("com 0x24D0:0x%x\n", read_cmos_eeprom_8(0x24D0));
			msleep(6);
			LOG_INF("com 0x24F7:0x%x\n", read_cmos_eeprom_8(0x24F7));
			msleep(6);
			LOG_INF("innal 0x24F8:0x%x\n", read_cmos_eeprom_8(0x24F8));
			msleep(6);
			LOG_INF("innal 0x2AE4:0x%x\n", read_cmos_eeprom_8(0x2AE4));
			msleep(6);
			LOG_INF("tail1 0x2AE5:0x%x\n", read_cmos_eeprom_8(0x2AE5));
			msleep(6);
			LOG_INF("tail2 0x2AE6:0x%x\n", read_cmos_eeprom_8(0x2AE6));
			msleep(6);
			LOG_INF("tail3 0x2AE7:0x%x\n", read_cmos_eeprom_8(0x2AE7));
			msleep(6);
			LOG_INF("tail4 0x2AE8:0x%x\n", read_cmos_eeprom_8(0x2AE8));
			msleep(6);
			LOG_INF("s5kgm1write_Module_data Write end\n");
		}else {
			pr_err("Invalid Sensor id:0x%x write_gm1 eeprom\n", pStereodata->uSensorId);
			return IMGSENSOR_RETURN_ERROR;
		}
	} else {
		pr_err("imx682write_Module_data pStereodata is null\n");
		return IMGSENSOR_RETURN_ERROR;
	}
	return ret;
}
#endif

static kal_uint16 imx682_QSC_setting[3024*2];
#if SUPPORT_LRC
static kal_uint16 imx682_LRC_setting[504*2];
#endif
static void read_sensor_Cali(void)
{
	kal_uint16 idx = 0, addr_qsc = 0x1880, sensor_addr = 0xCA00;
	#if SUPPORT_LRC
	kal_uint16 addr_lrc = 0x2460;
	#endif

	for (idx = 0; idx < 3024; idx++) {
		addr_qsc = 0x1880 + idx;
		sensor_addr = 0xCA00 + idx;
		imx682_QSC_setting[2*idx] = sensor_addr;
		imx682_QSC_setting[2*idx + 1] = read_cmos_eeprom_8(addr_qsc);
	}
	#if SUPPORT_LRC
	for (idx = 0; idx < 252; idx++) {
		addr_lrc = 0x2460 + idx;
		sensor_addr = 0x7B00 + idx;
		imx682_LRC_setting[2*idx] = sensor_addr;
		imx682_LRC_setting[2*idx + 1] = read_cmos_eeprom_8(addr_lrc);
	}
	for (idx = 252; idx < 504; idx++) {
		addr_lrc = 0x255c + idx;
		sensor_addr = 0x7C00 + idx - 252;
		imx682_LRC_setting[2*idx] = sensor_addr;
		imx682_LRC_setting[2*idx + 1] = read_cmos_eeprom_8(addr_lrc);
	}
	#endif
}

static void write_sensor_QSC(void)
{
	#if USE_BURST_MODE
	imx682_burst_write_cmos_sensor(imx682_QSC_setting,
		sizeof(imx682_QSC_setting)/sizeof(kal_uint16));
	#else
	kal_uint16 idx = 0, addr_qsc = 0xCA00;

	for (idx = 0; idx < 3024; idx++) {
		addr_qsc = 0xCA00 + idx;
		write_cmos_sensor_8(addr_qsc, imx682_QSC_setting[2 * idx + 1]);
	}
	#endif
}

#if SUPPORT_LRC
static void write_sensor_LRC(void)
{
	#if USE_BURST_MODE
	imx682_table_write_cmos_sensor(imx682_LRC_setting,
		sizeof(imx682_LRC_setting)/sizeof(kal_uint16));
	#else
	kal_uint16 idx = 0, addr_lrc = 0x7510;
	for (idx = 0; idx <252; idx++) {
		addr_lrc = 0x7B00 + idx;
		write_cmos_sensor_8(addr_lrc, imx682_LRC_setting[2*idx + 1]);
	}
	for (idx = 252; idx < 504; idx++) {
		addr_lrc = 0x7C00 + idx - 252;
		write_cmos_sensor_8(addr_lrc, imx682_LRC_setting[2*idx + 1]);
	}
	#endif
}
#endif

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n",
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
	int longexposure_times = 0;
	static int long_exposure_status;
	LOG_INF("step into writte_shutter:shutter =%d, framelength =%d\n",
        shutter, imgsensor.frame_length);

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM4) {
		imgsensor_info.min_shutter = 11;
	} else {
		imgsensor_info.min_shutter = 4;
	}
	if (shutter < imgsensor_info.min_shutter)
		shutter = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10
				/ imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
	}
	while (shutter >= 65535) {
		shutter = shutter / 2;
		longexposure_times += 1;
	}

	if (longexposure_times > 0) {
		LOG_INF("enter long exposure mode, time is %d",
			longexposure_times);
		long_exposure_status = 1;
		imgsensor.frame_length = shutter + 48;
		write_cmos_sensor_8(0x3100, longexposure_times & 0x07);
	} else if (long_exposure_status == 1) {
		long_exposure_status = 0;
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x3100, 0x00);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);

		LOG_INF("exit long exposure mode");
	}
	/* Update Shutter */
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0350, 0x01);
	write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, shutter  & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);
	LOG_INF("shutter =%d, framelength =%d\n",
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
				     kal_uint16 frame_length, kal_bool auto_extend_en)
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

/*	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;  */

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM4) {
		imgsensor_info.min_shutter = 11;
	} else {
		imgsensor_info.min_shutter = 4;
	}
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
	LOG_INF(
		"Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
		shutter, imgsensor.frame_length, frame_length,
		dummy_line, read_cmos_sensor(0x0350));

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
	if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM4) {
		/* 64M@30FPS */
		max_gain = 16 * BASEGAIN;
	}
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

static kal_uint32 imx682_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	UINT32 rgain_32, grgain_32, gbgain_32, bgain_32;

	grgain_32 = (pSetSensorAWB->ABS_GAIN_GR + 1) >> 1;
	rgain_32 = (pSetSensorAWB->ABS_GAIN_R + 1) >> 1;
	bgain_32 = (pSetSensorAWB->ABS_GAIN_B + 1) >> 1;
	gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB + 1) >> 1;
	LOG_INF("[%s] ABS_GAIN_GR:%d, grgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_GR, grgain_32);
	LOG_INF("[%s] ABS_GAIN_R:%d, rgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_R, rgain_32);
	LOG_INF("[%s] ABS_GAIN_B:%d, bgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_B, bgain_32);
	LOG_INF("[%s] ABS_GAIN_GB:%d, gbgain_32:%d\n",
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

	#if 0
	imx682_awb_gain_table[1]  = (grgain_32 >> 8) & 0xFF;
	imx682_awb_gain_table[3]  = grgain_32 & 0xFF;
	imx682_awb_gain_table[5]  = (rgain_32 >> 8) & 0xFF;
	imx682_awb_gain_table[7]  = rgain_32 & 0xFF;
	imx682_awb_gain_table[9]  = (bgain_32 >> 8) & 0xFF;
	imx682_awb_gain_table[11] = bgain_32 & 0xFF;
	imx682_awb_gain_table[13] = (gbgain_32 >> 8) & 0xFF;
	imx682_awb_gain_table[15] = gbgain_32 & 0xFF;
	imx682_table_write_cmos_sensor(imx682_awb_gain_table,
		sizeof(imx682_awb_gain_table)/sizeof(kal_uint16));
	#endif

	return ERROR_NONE;
}

static kal_uint16 imx682_feedback_awbgain[] = {
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
	#if 0
	/*write r_gain*/
	write_cmos_sensor_8(0x0B90, r_gain_int);
	write_cmos_sensor_8(0x0B91,
		(((r_gain*100) / 512) - (r_gain_int * 100)) * 2);

	/*write _gain*/
	write_cmos_sensor_8(0x0B92, b_gain_int);
	write_cmos_sensor_8(0x0B93,
		(((b_gain * 100) / 512) - (b_gain_int * 100)) * 2);
	#else
	imx682_feedback_awbgain[1] = r_gain_int;
	imx682_feedback_awbgain[3] = (
		((r_gain*100) / 512) - (r_gain_int * 100)) * 2;
	imx682_feedback_awbgain[5] = b_gain_int;
	imx682_feedback_awbgain[7] = (
		((b_gain * 100) / 512) - (b_gain_int * 100)) * 2;
	imx682_table_write_cmos_sensor(imx682_feedback_awbgain,
		sizeof(imx682_feedback_awbgain)/sizeof(kal_uint16));
	#endif
}

static void imx682_set_lsc_reg_setting(
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
		write_cmos_sensor_8(0x0100, 0x01);
		mdelay(10);
	} else {
		write_cmos_sensor_8(0x0100, 0x00);
		for (i = 0; i < timeout; i++) {
			mdelay(5);
			framecnt = read_cmos_sensor_8(0x0005);
			if (framecnt == 0xFF) {
				LOG_INF(" Stream Off OK at i=%d.\n", i);
				return ERROR_NONE;
			}
		}
		LOG_INF("Stream Off Fail! framecnt=%d.\n", framecnt);
	}

	return ERROR_NONE;
}

#if USE_BURST_MODE
#define I2C_BUFFER_LEN 255 /* trans# max is 255, each 3 bytes */
static kal_uint16 imx682_table_write_cmos_sensor(kal_uint16 *para,
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
#define I2C_BUFFER_LEN_B  765
static kal_uint16 imx682_burst_write_cmos_sensor(kal_uint16 *para,
                         kal_uint32 len)
{
    char puSendCmd[I2C_BUFFER_LEN_B];
    kal_uint32 tosend, IDX;
    kal_uint16 addr = 0, addr_last = 0, data;

    tosend = 0;
    IDX = 0;

    while (len > IDX) {
        addr = para[IDX];
        if(tosend == 0)
        {
            puSendCmd[tosend++] = (char)(addr >> 8);
            puSendCmd[tosend++] = (char)(addr & 0xFF);
            data = para[IDX + 1];
            puSendCmd[tosend++] = (char)(data & 0xFF);
            IDX += 2;
            addr_last = addr;
        }
        else if(addr == addr_last +1)
        {
            data = para[IDX +1];
            puSendCmd[tosend++] = (char)(data & 0xFF);
            addr_last = addr;
            IDX += 2;
        }
        /* Write when remain buffer size is less than 3 bytes
         * or reach end of data
         */
        if (tosend>=I2C_BUFFER_LEN_B
            || IDX == len || addr != addr_last) {
            iBurstWriteReg_multi(puSendCmd,
                        tosend,
                        imgsensor.i2c_write_id,
                        tosend,
                        imgsensor_info.i2c_speed);
            tosend = 0;
        }
    }
    return 0;
}

static kal_uint16 imx682_init_setting[] = {
	/*External Clock Setting*/
	0x0136, 0x18,
	0x0137, 0x00,
	/*Register version*/
	0x33F0, 0x05,
	0x33F1, 0x03,
	/*Signaling mode setting*/
	0x0111, 0x03,
	/*PDAF TYPE2 data type Setting*/
	0x3076, 0x00,
	0x3077, 0x30,
	/*Global Setting*/
	0x1F06, 0x06,
	0x1F07, 0x82,
	0x1F04, 0x71,
	0x1F05, 0x01,
	0x1F08, 0x01,
	0x5BFE, 0x14,
	0x5C0D, 0x2D,
	0x5C1C, 0x30,
	0x5C2B, 0x32,
	0x5C37, 0x2E,
	0x5C40, 0x30,
	0x5C50, 0x14,
	0x5C5F, 0x28,
	0x5C6E, 0x28,
	0x5C7D, 0x32,
	0x5C89, 0x37,
	0x5C92, 0x56,
	0x5BFC, 0x12,
	0x5C0B, 0x2A,
	0x5C1A, 0x2C,
	0x5C29, 0x2F,
	0x5C36, 0x2E,
	0x5C3F, 0x2E,
	0x5C4E, 0x06,
	0x5C5D, 0x1E,
	0x5C6C, 0x20,
	0x5C7B, 0x1E,
	0x5C88, 0x32,
	0x5C91, 0x32,
	0x5C02, 0x14,
	0x5C11, 0x2F,
	0x5C20, 0x32,
	0x5C2F, 0x34,
	0x5C39, 0x31,
	0x5C42, 0x31,
	0x5C8B, 0x28,
	0x5C94, 0x28,
	0x5C00, 0x10,
	0x5C0F, 0x2C,
	0x5C1E, 0x2E,
	0x5C2D, 0x32,
	0x5C38, 0x2E,
	0x5C41, 0x2B,
	0x5C61, 0x0A,
	0x5C70, 0x0A,
	0x5C7F, 0x0A,
	0x5C8A, 0x1E,
	0x5C93, 0x2A,
	0x5BFA, 0x2B,
	0x5C09, 0x2D,
	0x5C18, 0x2E,
	0x5C27, 0x30,
	0x5C5B, 0x28,
	0x5C6A, 0x22,
	0x5C79, 0x42,
	0x5BFB, 0x2C,
	0x5C0A, 0x2F,
	0x5C19, 0x2E,
	0x5C28, 0x2E,
	0x5C4D, 0x20,
	0x5C5C, 0x1E,
	0x5C6B, 0x32,
	0x5C7A, 0x32,
	0x5BFD, 0x30,
	0x5C0C, 0x32,
	0x5C1B, 0x2E,
	0x5C2A, 0x30,
	0x5C4F, 0x28,
	0x5C5E, 0x32,
	0x5C6D, 0x37,
	0x5C7C, 0x56,
	0x5BFF, 0x2E,
	0x5C0E, 0x32,
	0x5C1D, 0x2E,
	0x5C2C, 0x2B,
	0x5C51, 0x0A,
	0x5C60, 0x0A,
	0x5C6F, 0x1E,
	0x5C7E, 0x2A,
	0x5C01, 0x32,
	0x5C10, 0x34,
	0x5C1F, 0x31,
	0x5C2E, 0x31,
	0x5C71, 0x28,
	0x5C80, 0x28,
	0x5C4C, 0x2A,
	0x33F2, 0x01,
	0x1F04, 0x73,
	0x1F05, 0x01,
	0x5BFA, 0x35,
	0x5C09, 0x38,
	0x5C18, 0x3A,
	0x5C27, 0x38,
	0x5C5B, 0x25,
	0x5C6A, 0x24,
	0x5C79, 0x47,
	0x5BFC, 0x15,
	0x5C0B, 0x2E,
	0x5C1A, 0x36,
	0x5C29, 0x38,
	0x5C36, 0x36,
	0x5C3F, 0x36,
	0x5C4E, 0x0B,
	0x5C5D, 0x20,
	0x5C6C, 0x2A,
	0x5C7B, 0x25,
	0x5C88, 0x25,
	0x5C91, 0x22,
	0x5BFE, 0x15,
	0x5C0D, 0x32,
	0x5C1C, 0x36,
	0x5C2B, 0x36,
	0x5C37, 0x3A,
	0x5C40, 0x39,
	0x5C50, 0x06,
	0x5C5F, 0x22,
	0x5C6E, 0x23,
	0x5C7D, 0x2E,
	0x5C89, 0x44,
	0x5C92, 0x51,
	0x5D7F, 0x0A,
	0x5C00, 0x17,
	0x5C0F, 0x36,
	0x5C1E, 0x38,
	0x5C2D, 0x3C,
	0x5C38, 0x38,
	0x5C41, 0x36,
	0x5C52, 0x0A,
	0x5C61, 0x21,
	0x5C70, 0x23,
	0x5C7F, 0x1B,
	0x5C8A, 0x22,
	0x5C93, 0x20,
	0x5C02, 0x1A,
	0x5C11, 0x3E,
	0x5C20, 0x3F,
	0x5C2F, 0x3D,
	0x5C39, 0x3E,
	0x5C42, 0x3C,
	0x5C54, 0x02,
	0x5C63, 0x12,
	0x5C72, 0x14,
	0x5C81, 0x24,
	0x5C8B, 0x1C,
	0x5C94, 0x4E,
	0x5D8A, 0x09,
	0x5BFB, 0x36,
	0x5C0A, 0x38,
	0x5C19, 0x36,
	0x5C28, 0x36,
	0x5C4D, 0x2A,
	0x5C5C, 0x25,
	0x5C6B, 0x25,
	0x5C7A, 0x22,
	0x5BFD, 0x36,
	0x5C0C, 0x36,
	0x5C1B, 0x3A,
	0x5C2A, 0x39,
	0x5C4F, 0x23,
	0x5C5E, 0x2E,
	0x5C6D, 0x44,
	0x5C7C, 0x51,
	0x5D63, 0x0A,
	0x5BFF, 0x38,
	0x5C0E, 0x3C,
	0x5C1D, 0x38,
	0x5C2C, 0x36,
	0x5C51, 0x23,
	0x5C60, 0x1B,
	0x5C6F, 0x22,
	0x5C7E, 0x20,
	0x5C01, 0x3F,
	0x5C10, 0x3D,
	0x5C1F, 0x3E,
	0x5C2E, 0x3C,
	0x5C53, 0x14,
	0x5C62, 0x24,
	0x5C71, 0x1C,
	0x5C80, 0x4E,
	0x5D76, 0x09,
	0x5C4C, 0x2A,
	0x33F2, 0x02,
	0x1F04, 0x78,
	0x1F05, 0x01,
	0x5BFA, 0x37,
	0x5C09, 0x36,
	0x5C18, 0x39,
	0x5C27, 0x38,
	0x5C5B, 0x27,
	0x5C6A, 0x2B,
	0x5C79, 0x48,
	0x5BFC, 0x16,
	0x5C0B, 0x32,
	0x5C1A, 0x33,
	0x5C29, 0x37,
	0x5C36, 0x36,
	0x5C3F, 0x35,
	0x5C4E, 0x0D,
	0x5C5D, 0x2D,
	0x5C6C, 0x23,
	0x5C7B, 0x25,
	0x5C88, 0x31,
	0x5C91, 0x2E,
	0x5BFE, 0x15,
	0x5C0D, 0x31,
	0x5C1C, 0x35,
	0x5C2B, 0x36,
	0x5C37, 0x35,
	0x5C40, 0x37,
	0x5C50, 0x0F,
	0x5C5F, 0x31,
	0x5C6E, 0x30,
	0x5C7D, 0x33,
	0x5C89, 0x36,
	0x5C92, 0x5B,
	0x5C00, 0x13,
	0x5C0F, 0x2F,
	0x5C1E, 0x2E,
	0x5C2D, 0x34,
	0x5C38, 0x33,
	0x5C41, 0x32,
	0x5C52, 0x0D,
	0x5C61, 0x27,
	0x5C70, 0x28,
	0x5C7F, 0x1F,
	0x5C8A, 0x25,
	0x5C93, 0x2C,
	0x5C02, 0x15,
	0x5C11, 0x36,
	0x5C20, 0x39,
	0x5C2F, 0x3A,
	0x5C39, 0x37,
	0x5C42, 0x37,
	0x5C54, 0x04,
	0x5C63, 0x1C,
	0x5C72, 0x1C,
	0x5C81, 0x1C,
	0x5C8B, 0x28,
	0x5C94, 0x24,
	0x5BFB, 0x33,
	0x5C0A, 0x37,
	0x5C19, 0x36,
	0x5C28, 0x35,
	0x5C4D, 0x23,
	0x5C5C, 0x25,
	0x5C6B, 0x31,
	0x5C7A, 0x2E,
	0x5BFD, 0x35,
	0x5C0C, 0x36,
	0x5C1B, 0x35,
	0x5C2A, 0x37,
	0x5C4F, 0x30,
	0x5C5E, 0x33,
	0x5C6D, 0x36,
	0x5C7C, 0x5B,
	0x5BFF, 0x2E,
	0x5C0E, 0x34,
	0x5C1D, 0x33,
	0x5C2C, 0x32,
	0x5C51, 0x28,
	0x5C60, 0x1F,
	0x5C6F, 0x25,
	0x5C7E, 0x2C,
	0x5C01, 0x39,
	0x5C10, 0x3A,
	0x5C1F, 0x37,
	0x5C2E, 0x37,
	0x5C53, 0x1C,
	0x5C62, 0x1C,
	0x5C71, 0x28,
	0x5C80, 0x24,
	0x5C4C, 0x2C,
	0x33F2, 0x03,
	0x1F08, 0x00,
	0x32C8, 0x00,
	0x4017, 0x40,
	0x40A2, 0x01,
	0x40AC, 0x01,
	0x4328, 0x00,
	0x4329, 0xB3,
	0x4E15, 0x10,
	0x4E19, 0x2F,
	0x4E21, 0x0F,
	0x4E2F, 0x10,
	0x4E3D, 0x10,
	0x4E41, 0x2F,
	0x4E57, 0x29,
	0x4FFB, 0x2F,
	0x5011, 0x24,
	0x501D, 0x03,
	0x505F, 0x41,
	0x5060, 0xDF,
	0x5065, 0xDF,
	0x5066, 0x37,
	0x506E, 0x57,
	0x5070, 0xC5,
	0x5072, 0x57,
	0x5075, 0x53,
	0x5076, 0x55,
	0x5077, 0xC1,
	0x5078, 0xC3,
	0x5079, 0x53,
	0x507A, 0x55,
	0x507D, 0x57,
	0x507E, 0xDF,
	0x507F, 0xC5,
	0x5081, 0x57,
	0x53C8, 0x01,
	0x53C9, 0xE2,
	0x53CA, 0x03,
	0x5422, 0x7A,
	0x548E, 0x40,
	0x5497, 0x5E,
	0x54A1, 0x40,
	0x54A9, 0x40,
	0x54B2, 0x5E,
	0x54BC, 0x40,
	0x57C6, 0x00,
	0x583D, 0x0E,
	0x583E, 0x0E,
	0x583F, 0x0E,
	0x5840, 0x0E,
	0x5841, 0x0E,
	0x5842, 0x0E,
	0x5900, 0x12,
	0x5901, 0x12,
	0x5902, 0x14,
	0x5903, 0x12,
	0x5904, 0x14,
	0x5905, 0x12,
	0x5906, 0x14,
	0x5907, 0x12,
	0x590F, 0x12,
	0x5911, 0x12,
	0x5913, 0x12,
	0x591C, 0x12,
	0x591E, 0x12,
	0x5920, 0x12,
	0x5948, 0x08,
	0x5949, 0x08,
	0x594A, 0x08,
	0x594B, 0x08,
	0x594C, 0x08,
	0x594D, 0x08,
	0x594E, 0x08,
	0x594F, 0x08,
	0x595C, 0x08,
	0x595E, 0x08,
	0x5960, 0x08,
	0x596E, 0x08,
	0x5970, 0x08,
	0x5972, 0x08,
	0x597E, 0x0F,
	0x597F, 0x0F,
	0x599A, 0x0F,
	0x59DE, 0x08,
	0x59DF, 0x08,
	0x59FA, 0x08,
	0x5A59, 0x22,
	0x5A5B, 0x22,
	0x5A5D, 0x1A,
	0x5A5F, 0x22,
	0x5A61, 0x1A,
	0x5A63, 0x22,
	0x5A65, 0x1A,
	0x5A67, 0x22,
	0x5A77, 0x22,
	0x5A7B, 0x22,
	0x5A7F, 0x22,
	0x5A91, 0x22,
	0x5A95, 0x22,
	0x5A99, 0x22,
	0x5AE9, 0x66,
	0x5AEB, 0x66,
	0x5AED, 0x5E,
	0x5AEF, 0x66,
	0x5AF1, 0x5E,
	0x5AF3, 0x66,
	0x5AF5, 0x5E,
	0x5AF7, 0x66,
	0x5B07, 0x66,
	0x5B0B, 0x66,
	0x5B0F, 0x66,
	0x5B21, 0x66,
	0x5B25, 0x66,
	0x5B29, 0x66,
	0x5B79, 0x46,
	0x5B7B, 0x3E,
	0x5B7D, 0x3E,
	0x5B89, 0x46,
	0x5B8B, 0x46,
	0x5B97, 0x46,
	0x5B99, 0x46,
	0x5C9E, 0x0A,
	0x5C9F, 0x08,
	0x5CA0, 0x0A,
	0x5CA1, 0x0A,
	0x5CA2, 0x0B,
	0x5CA3, 0x06,
	0x5CA4, 0x04,
	0x5CA5, 0x06,
	0x5CA6, 0x04,
	0x5CAD, 0x0B,
	0x5CAE, 0x0A,
	0x5CAF, 0x0C,
	0x5CB0, 0x0A,
	0x5CB1, 0x0B,
	0x5CB2, 0x08,
	0x5CB3, 0x06,
	0x5CB4, 0x08,
	0x5CB5, 0x04,
	0x5CBC, 0x0B,
	0x5CBD, 0x09,
	0x5CBE, 0x08,
	0x5CBF, 0x09,
	0x5CC0, 0x0A,
	0x5CC1, 0x08,
	0x5CC2, 0x06,
	0x5CC3, 0x08,
	0x5CC4, 0x06,
	0x5CCB, 0x0A,
	0x5CCC, 0x09,
	0x5CCD, 0x0A,
	0x5CCE, 0x08,
	0x5CCF, 0x0A,
	0x5CD0, 0x08,
	0x5CD1, 0x08,
	0x5CD2, 0x08,
	0x5CD3, 0x08,
	0x5CDA, 0x09,
	0x5CDB, 0x09,
	0x5CDC, 0x08,
	0x5CDD, 0x08,
	0x5CE3, 0x09,
	0x5CE4, 0x08,
	0x5CE5, 0x08,
	0x5CE6, 0x08,
	0x5CF4, 0x04,
	0x5D04, 0x04,
	0x5D13, 0x06,
	0x5D22, 0x06,
	0x5D23, 0x04,
	0x5D2E, 0x06,
	0x5D37, 0x06,
	0x5D6F, 0x09,
	0x5D72, 0x0F,
	0x5D88, 0x0F,
	0x5DE6, 0x01,
	0x5DE7, 0x01,
	0x5DE8, 0x01,
	0x5DE9, 0x01,
	0x5DEA, 0x01,
	0x5DEB, 0x01,
	0x5DEC, 0x01,
	0x5DF2, 0x01,
	0x5DF3, 0x01,
	0x5DF4, 0x01,
	0x5DF5, 0x01,
	0x5DF6, 0x01,
	0x5DF7, 0x01,
	0x5DF8, 0x01,
	0x5DFE, 0x01,
	0x5DFF, 0x01,
	0x5E00, 0x01,
	0x5E01, 0x01,
	0x5E02, 0x01,
	0x5E03, 0x01,
	0x5E04, 0x01,
	0x5E0A, 0x01,
	0x5E0B, 0x01,
	0x5E0C, 0x01,
	0x5E0D, 0x01,
	0x5E0E, 0x01,
	0x5E0F, 0x01,
	0x5E10, 0x01,
	0x5E16, 0x01,
	0x5E17, 0x01,
	0x5E18, 0x01,
	0x5E1E, 0x01,
	0x5E1F, 0x01,
	0x5E20, 0x01,
	0x5E6E, 0x5A,
	0x5E6F, 0x46,
	0x5E70, 0x46,
	0x5E71, 0x3C,
	0x5E72, 0x3C,
	0x5E73, 0x28,
	0x5E74, 0x28,
	0x5E75, 0x6E,
	0x5E76, 0x6E,
	0x5E81, 0x46,
	0x5E83, 0x3C,
	0x5E85, 0x28,
	0x5E87, 0x6E,
	0x5E92, 0x46,
	0x5E94, 0x3C,
	0x5E96, 0x28,
	0x5E98, 0x6E,
	0x5ECB, 0x26,
	0x5ECC, 0x26,
	0x5ECD, 0x26,
	0x5ECE, 0x26,
	0x5ED2, 0x26,
	0x5ED3, 0x26,
	0x5ED4, 0x26,
	0x5ED5, 0x26,
	0x5ED9, 0x26,
	0x5EDA, 0x26,
	0x5EE5, 0x08,
	0x5EE6, 0x08,
	0x5EE7, 0x08,
	0x6006, 0x14,
	0x6007, 0x14,
	0x6008, 0x14,
	0x6009, 0x14,
	0x600A, 0x14,
	0x600B, 0x14,
	0x600C, 0x14,
	0x600D, 0x22,
	0x600E, 0x22,
	0x600F, 0x14,
	0x601A, 0x14,
	0x601B, 0x14,
	0x601C, 0x14,
	0x601D, 0x14,
	0x601E, 0x14,
	0x601F, 0x14,
	0x6020, 0x14,
	0x6021, 0x22,
	0x6022, 0x22,
	0x6023, 0x14,
	0x602E, 0x14,
	0x602F, 0x14,
	0x6030, 0x14,
	0x6031, 0x22,
	0x6039, 0x14,
	0x603A, 0x14,
	0x603B, 0x14,
	0x603C, 0x22,
	0x6132, 0x0F,
	0x6133, 0x0F,
	0x6134, 0x0F,
	0x6135, 0x0F,
	0x6136, 0x0F,
	0x6137, 0x0F,
	0x6138, 0x0F,
	0x613E, 0x0F,
	0x613F, 0x0F,
	0x6140, 0x0F,
	0x6141, 0x0F,
	0x6142, 0x0F,
	0x6143, 0x0F,
	0x6144, 0x0F,
	0x614A, 0x0F,
	0x614B, 0x0F,
	0x614C, 0x0F,
	0x614D, 0x0F,
	0x614E, 0x0F,
	0x614F, 0x0F,
	0x6150, 0x0F,
	0x6156, 0x0F,
	0x6157, 0x0F,
	0x6158, 0x0F,
	0x6159, 0x0F,
	0x615A, 0x0F,
	0x615B, 0x0F,
	0x615C, 0x0F,
	0x6162, 0x0F,
	0x6163, 0x0F,
	0x6164, 0x0F,
	0x616A, 0x0F,
	0x616B, 0x0F,
	0x616C, 0x0F,
	0x6226, 0x00,
	0x84F8, 0x01,
	0x8501, 0x00,
	0x8502, 0x01,
	0x8505, 0x00,
	0x8744, 0x00,
	0x883C, 0x01,
	0x8845, 0x00,
	0x8846, 0x01,
	0x8849, 0x00,
	0x9004, 0x1F,
	0x9064, 0x4D,
	0x9065, 0x3D,
	0x922E, 0x91,
	0x922F, 0x2A,
	0x9230, 0xE2,
	0x9231, 0xC0,
	0x9232, 0xE2,
	0x9233, 0xC1,
	0x9234, 0xE2,
	0x9235, 0xC2,
	0x9236, 0xE2,
	0x9237, 0xC3,
	0x9238, 0xE2,
	0x9239, 0xD4,
	0x923A, 0xE2,
	0x923B, 0xD5,
	0x923C, 0x90,
	0x923D, 0x64,
	0xB0B9, 0x10,
	0xBC76, 0x00,
	0xBC77, 0x00,
	0xBC78, 0x00,
	0xBC79, 0x00,
	0xBC7B, 0x28,
	0xBC7C, 0x00,
	0xBC7D, 0x00,
	0xBC7F, 0xC0,
	0xC6B9, 0x01,
	0xECB5, 0x04,
	0xECBF, 0x04,
	/*Image Quality adjustment setting*/
	0x32D9, 0x01,
	0x85C0, 0x01,
	0xA503, 0x04,
	0xA533, 0x3F,
	0xA53C, 0x06,
	0xA53F, 0x04,
	0xA56F, 0x3F,
	0xA654, 0x01,
	0xA655, 0x68,
	0xA6D2, 0x01,
	0xA6D3, 0x68,
	0xA6D6, 0x00,
	0xA6D7, 0x4C,
	0xA737, 0x20,
	0xAC2C, 0x05,
	0xAC5C, 0x05,
	0xAC8C, 0x05,
	0xB026, 0x00,

};

static kal_uint16 imx682_preview_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x24,
	0x0343, 0xD8,
	/*Frame Length Lines Setting*/
	0x0340, 0x0E,
	0x0341, 0xA9,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x24,
	0x0349, 0x1F,
	0x034A, 0x1B,
	0x034B, 0x1F,
	/*Mode Setting*/
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x30D8, 0x04,
	0x3200, 0x41,
	0x3201, 0x41,
	/*Digital Crop & Scaling*/
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x12,
	0x040D, 0x10,
	0x040E, 0x0D,
	0x040F, 0x90,
	/*Output Size Setting*/
	0x034C, 0x12,
	0x034D, 0x10,
	0x034E, 0x0D,
	0x034F, 0x90,
	/*Clock Setting*/
	0x0301, 0x08,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x62,
	0x030B, 0x02,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x85,
	0x0310, 0x01,
	/*Other Setting*/
	0x30D9, 0x00,
	0x32D5, 0x00,
	0x32D6, 0x00,
	0x401E, 0x00,
	0x40B8, 0x01,
	0x40B9, 0x2C,
	0x40BC, 0x01,
	0x40BD, 0x18,
	0x40BE, 0x00,
	0x40BF, 0x00,
	0x41A4, 0x00,
	0x5A09, 0x01,
	0x5A17, 0x01,
	0x5A25, 0x01,
	0x5A33, 0x01,
	0x98D7, 0xB4,
	0x98D8, 0x8C,
	0x98D9, 0x0A,
	0x99C4, 0x16,
	/*Integration Setting*/
	0x0202, 0x0E,
	0x0203, 0x79,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	/*PDAF Setting*/
	0x4018, 0x04,
	0x4019, 0x80,
	0x401A, 0x00,
	0x401B, 0x01,
	/*PDAF TYPE Setting*/
	0x3400, 0x02,
	/*PDAF TYPE2 Setting*/
	0x3093, 0x01,

};

static kal_uint16 imx682_capture_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x24,
	0x0343, 0xD8,
	/*Frame Length Lines Setting*/
	0x0340, 0x0E,
	0x0341, 0xA9,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x24,
	0x0349, 0x1F,
	0x034A, 0x1B,
	0x034B, 0x1F,
	/*Mode Setting*/
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x30D8, 0x04,
	0x3200, 0x41,
	0x3201, 0x41,
	/*Digital Crop & Scaling*/
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x12,
	0x040D, 0x10,
	0x040E, 0x0D,
	0x040F, 0x90,
	/*Output Size Setting*/
	0x034C, 0x12,
	0x034D, 0x10,
	0x034E, 0x0D,
	0x034F, 0x90,
	/*Clock Setting*/
	0x0301, 0x08,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x62,
	0x030B, 0x02,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x85,
	0x0310, 0x01,
	/*Other Setting*/
	0x30D9, 0x00,
	0x32D5, 0x00,
	0x32D6, 0x00,
	0x401E, 0x00,
	0x40B8, 0x01,
	0x40B9, 0x2C,
	0x40BC, 0x01,
	0x40BD, 0x18,
	0x40BE, 0x00,
	0x40BF, 0x00,
	0x41A4, 0x00,
	0x5A09, 0x01,
	0x5A17, 0x01,
	0x5A25, 0x01,
	0x5A33, 0x01,
	0x98D7, 0xB4,
	0x98D8, 0x8C,
	0x98D9, 0x0A,
	0x99C4, 0x16,
	/*Integration Setting*/
	0x0202, 0x0E,
	0x0203, 0x79,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	/*PDAF Setting*/
	0x4018, 0x04,
	0x4019, 0x80,
	0x401A, 0x00,
	0x401B, 0x01,
	/*PDAF TYPE Setting*/
	0x3400, 0x02,
	/*PDAF TYPE2 Setting*/
	0x3093, 0x01,

};

static kal_uint16 imx682_normal_video_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x24,
	0x0343, 0xD8,
	/*Frame Length Lines Setting*/
	0x0340, 0x0B,
	0x0341, 0x04,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x03,
	0x0347, 0x70,
	0x0348, 0x24,
	0x0349, 0x1F,
	0x034A, 0x17,
	0x034B, 0xAF,
	/*Mode Setting*/
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x30D8, 0x04,
	0x3200, 0x41,
	0x3201, 0x41,
	/*Digital Crop & Scaling*/
	0x0408, 0x00,
	0x0409, 0x08,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x12,
	0x040D, 0x00,
	0x040E, 0x0A,
	0x040F, 0x20,
	/*Output Size Setting*/
	0x034C, 0x12,
	0x034D, 0x00,
	0x034E, 0x0A,
	0x034F, 0x20,
	/*Clock Setting*/
	0x0301, 0x08,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x0A,
	0x030B, 0x02,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x1E,
	0x0310, 0x01,
	/*Other Setting*/
	0x30D9, 0x00,
	0x32D5, 0x00,
	0x32D6, 0x00,
	0x401E, 0x00,
	0x40B8, 0x01,
	0x40B9, 0x2C,
	0x40BC, 0x01,
	0x40BD, 0x18,
	0x40BE, 0x00,
	0x40BF, 0x00,
	0x41A4, 0x00,
	0x5A09, 0x01,
	0x5A17, 0x01,
	0x5A25, 0x01,
	0x5A33, 0x01,
	0x98D7, 0xB4,
	0x98D8, 0x8C,
	0x98D9, 0x0A,
	0x99C4, 0x16,
	/*Integration Setting*/
	0x0202, 0x0A,
	0x0203, 0xD4,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	/*PDAF Setting*/
	0x4018, 0x04,
	0x4019, 0x80,
	0x401A, 0x00,
	0x401B, 0x01,
	/*PDAF TYPE Setting*/
	0x3400, 0x02,
	/*PDAF TYPE2 Setting*/
	0x3093, 0x01,

};

static kal_uint16 imx682_slim_video_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x24,
	0x0343, 0xD8,
	/*Frame Length Lines Setting*/
	0x0340, 0x0A,
	0x0341, 0xA4,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x03,
	0x0347, 0x70,
	0x0348, 0x24,
	0x0349, 0x1F,
	0x034A, 0x17,
	0x034B, 0xAF,
	/*Mode Setting*/
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x30D8, 0x04,
	0x3200, 0x41,
	0x3201, 0x41,
	/*Digital Crop & Scaling*/
	0x0408, 0x00,
	0x0409, 0x08,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x12,
	0x040D, 0x00,
	0x040E, 0x0A,
	0x040F, 0x20,
	/*Output Size Setting*/
	0x034C, 0x12,
	0x034D, 0x00,
	0x034E, 0x0A,
	0x034F, 0x20,
	/*Clock Setting*/
	0x0301, 0x08,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x01,
	0x030B, 0x02,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xFA,
	0x0310, 0x01,
	/*Other Setting*/
	0x30D9, 0x00,
	0x32D5, 0x00,
	0x32D6, 0x00,
	0x401E, 0x00,
	0x40B8, 0x01,
	0x40B9, 0x2C,
	0x40BC, 0x01,
	0x40BD, 0x18,
	0x40BE, 0x00,
	0x40BF, 0x00,
	0x41A4, 0x00,
	0x5A09, 0x01,
	0x5A17, 0x01,
	0x5A25, 0x01,
	0x5A33, 0x01,
	0x98D7, 0xB4,
	0x98D8, 0x8C,
	0x98D9, 0x0A,
	0x99C4, 0x16,
	/*Integration Setting*/
	0x0202, 0x0A,
	0x0203, 0x74,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	/*PDAF Setting*/
	0x4018, 0x04,
	0x4019, 0x80,
	0x401A, 0x00,
	0x401B, 0x01,
	/*PDAF TYPE Setting*/
	0x3400, 0x02,
	/*PDAF TYPE2 Setting*/
	0x3093, 0x01,

};

static kal_uint16 imx682_hs_video_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x0A,
	0x0343, 0xA8,
	/*Frame Length Lines Setting*/
	0x0340, 0x06,
	0x0341, 0x8C,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x03,
	0x0347, 0x70,
	0x0348, 0x24,
	0x0349, 0x1F,
	0x034A, 0x17,
	0x034B, 0xAF,
	/*Mode Setting*/
	0x0900, 0x01,
	0x0901, 0x44,
	0x0902, 0x08,
	0x30D8, 0x00,
	0x3200, 0x43,
	0x3201, 0x43,
	/*Digital Crop & Scaling*/
	0x0408, 0x00,
	0x0409, 0x04,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x09,
	0x040D, 0x00,
	0x040E, 0x05,
	0x040F, 0x10,
	/*Output Size Setting*/
	0x034C, 0x09,
	0x034D, 0x00,
	0x034E, 0x05,
	0x034F, 0x10,
	/*Clock Setting*/
	0x0301, 0x08,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x6E,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x27,
	0x0310, 0x01,
	/*Other Setting*/
	0x30D9, 0x01,
	0x32D5, 0x00,
	0x32D6, 0x00,
	0x401E, 0x4D,
	0x40B8, 0x00,
	0x40B9, 0xC6,
	0x40BC, 0x00,
	0x40BD, 0xB2,
	0x40BE, 0x00,
	0x40BF, 0xB2,
	0x41A4, 0x00,
	0x5A09, 0x00,
	0x5A17, 0x00,
	0x5A25, 0x00,
	0x5A33, 0x00,
	0x98D7, 0xB4,
	0x98D8, 0x8C,
	0x98D9, 0x0A,
	0x99C4, 0x16,
	/*Integration Setting*/
	0x0202, 0x06,
	0x0203, 0x5C,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	/*PDAF Setting*/
	0x4018, 0x00,
	0x4019, 0x00,
	0x401A, 0x00,
	0x401B, 0x00,
	/*PDAF TYPE Setting*/
	0x3400, 0x02,
	/*PDAF TYPE2 Setting*/
	0x3093, 0x00,

};

static kal_uint16 imx682_custom1_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x24,
	0x0343, 0xD8,
	/*Frame Length Lines Setting*/
	0x0340, 0x0E,
	0x0341, 0x22,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x24,
	0x0349, 0x1F,
	0x034A, 0x1B,
	0x034B, 0x1F,
	/*Mode Setting*/
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x30D8, 0x04,
	0x3200, 0x41,
	0x3201, 0x41,
	/*Digital Crop & Scaling*/
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x12,
	0x040D, 0x10,
	0x040E, 0x0D,
	0x040F, 0x90,
	/*Output Size Setting*/
	0x034C, 0x12,
	0x034D, 0x10,
	0x034E, 0x0D,
	0x034F, 0x90,
	/*Clock Setting*/
	0x0301, 0x08,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x11,
	0x030B, 0x02,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x0B,
	0x0310, 0x01,
	/*Other Setting*/
	0x30D9, 0x00,
	0x32D5, 0x00,
	0x32D6, 0x00,
	0x401E, 0x00,
	0x40B8, 0x01,
	0x40B9, 0x2C,
	0x40BC, 0x01,
	0x40BD, 0x18,
	0x40BE, 0x00,
	0x40BF, 0x00,
	0x41A4, 0x00,
	0x5A09, 0x01,
	0x5A17, 0x01,
	0x5A25, 0x01,
	0x5A33, 0x01,
	0x98D7, 0xB4,
	0x98D8, 0x8C,
	0x98D9, 0x0A,
	0x99C4, 0x16,
	/*Integration Setting*/
	0x0202, 0x0D,
	0x0203, 0xF2,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	/*PDAF Setting*/
	0x4018, 0x04,
	0x4019, 0x80,
	0x401A, 0x00,
	0x401B, 0x01,
	/*PDAF TYPE Setting*/
	0x3400, 0x02,
	/*PDAF TYPE2 Setting*/
	0x3093, 0x01,

};

static kal_uint16 imx682_custom2_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x0A,
	0x0343, 0xA8,
	/*Frame Length Lines Setting*/
	0x0340, 0x06,
	0x0341, 0x8C,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x03,
	0x0347, 0x70,
	0x0348, 0x24,
	0x0349, 0x1F,
	0x034A, 0x17,
	0x034B, 0xAF,
	/*Mode Setting*/
	0x0900, 0x01,
	0x0901, 0x44,
	0x0902, 0x08,
	0x30D8, 0x00,
	0x3200, 0x43,
	0x3201, 0x43,
	/*Digital Crop & Scaling*/
	0x0408, 0x00,
	0x0409, 0x04,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x09,
	0x040D, 0x00,
	0x040E, 0x05,
	0x040F, 0x10,
	/*Output Size Setting*/
	0x034C, 0x09,
	0x034D, 0x00,
	0x034E, 0x05,
	0x034F, 0x10,
	/*Clock Setting*/
	0x0301, 0x08,
	0x0303, 0x04,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x6E,
	0x030B, 0x02,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x21,
	0x0310, 0x01,
	/*Other Setting*/
	0x30D9, 0x01,
	0x32D5, 0x00,
	0x32D6, 0x00,
	0x401E, 0x4D,
	0x40B8, 0x00,
	0x40B9, 0x32,
	0x40BC, 0x00,
	0x40BD, 0x08,
	0x40BE, 0x00,
	0x40BF, 0x08,
	0x41A4, 0x00,
	0x5A09, 0x00,
	0x5A17, 0x00,
	0x5A25, 0x00,
	0x5A33, 0x00,
	0x98D7, 0xB4,
	0x98D8, 0x8C,
	0x98D9, 0x0A,
	0x99C4, 0x16,
	/*Integration Setting*/
	0x0202, 0x06,
	0x0203, 0x5C,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	/*PDAF Setting*/
	0x4018, 0x00,
	0x4019, 0x00,
	0x401A, 0x00,
	0x401B, 0x00,
	/*PDAF TYPE Setting*/
	0x3400, 0x02,
	/*PDAF TYPE2 Setting*/
	0x3093, 0x00,

};

static kal_uint16 imx682_custom3_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x24,
	0x0343, 0xD8,
	/*Frame Length Lines Setting*/
	0x0340, 0x07,
	0x0341, 0x64,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x06,
	0x0347, 0xD0,
	0x0348, 0x24,
	0x0349, 0x1F,
	0x034A, 0x14,
	0x034B, 0x4F,
	/*Mode Setting*/
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x30D8, 0x04,
	0x3200, 0x41,
	0x3201, 0x41,
	/*Digital Crop & Scaling*/
	0x0408, 0x03,
	0x0409, 0x10,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0B,
	0x040D, 0xF0,
	0x040E, 0x06,
	0x040F, 0xC0,
	/*Output Size Setting*/
	0x034C, 0x0B,
	0x034D, 0xF0,
	0x034E, 0x06,
	0x034F, 0xC0,
	/*Clock Setting*/
	0x0301, 0x08,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x65,
	0x030B, 0x02,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xF2,
	0x0310, 0x01,
	/*Other Setting*/
	0x30D9, 0x00,
	0x32D5, 0x00,
	0x32D6, 0x00,
	0x401E, 0x00,
	0x40B8, 0x01,
	0x40B9, 0x2C,
	0x40BC, 0x01,
	0x40BD, 0x18,
	0x40BE, 0x00,
	0x40BF, 0x00,
	0x41A4, 0x00,
	0x5A09, 0x01,
	0x5A17, 0x01,
	0x5A25, 0x01,
	0x5A33, 0x01,
	0x98D7, 0xB4,
	0x98D8, 0x8C,
	0x98D9, 0x0A,
	0x99C4, 0x16,
	/*Integration Setting*/
	0x0202, 0x07,
	0x0203, 0x34,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	/*PDAF Setting*/
	0x4018, 0x02,
	0x4019, 0xF8,
	0x401A, 0x00,
	0x401B, 0x01,
	/*PDAF TYPE Setting*/
	0x3400, 0x02,
	/*PDAF TYPE2 Setting*/
	0x3093, 0x01,

};

static kal_uint16 imx682_custom4_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x28,
	0x0343, 0x30,
	/*Frame Length Lines Setting*/
	0x0340, 0x1B,
	0x0341, 0xCB,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x24,
	0x0349, 0x1F,
	0x034A, 0x1B,
	0x034B, 0x1F,
	/*Mode Setting*/
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x0A,
	0x30D8, 0x00,
	0x3200, 0x01,
	0x3201, 0x01,
	/*Digital Crop & Scaling*/
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x24,
	0x040D, 0x20,
	0x040E, 0x1B,
	0x040F, 0x20,
	/*Output Size Setting*/
	0x034C, 0x24,
	0x034D, 0x20,
	0x034E, 0x1B,
	0x034F, 0x20,
	/*Clock Setting*/
	0x0301, 0x08,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x6E,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x2F,
	0x0310, 0x01,
	/*Other Setting*/
	0x30D9, 0x01,
	0x32D5, 0x01,
	0x32D6, 0x01,
	0x401E, 0x00,
	0x40B8, 0x02,
	0x40B9, 0x1C,
	0x40BC, 0x00,
	0x40BD, 0xB0,
	0x40BE, 0x00,
	0x40BF, 0xB0,
	0x41A4, 0x00,
	0x5A09, 0x01,
	0x5A17, 0x01,
	0x5A25, 0x01,
	0x5A33, 0x01,
	0x98D7, 0x14,
	0x98D8, 0x14,
	0x98D9, 0x00,
	0x99C4, 0x00,
	/*Integration Setting*/
	0x0202, 0x1B,
	0x0203, 0x9B,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	/*PDAF Setting*/
	0x4018, 0x04,
	0x4019, 0x80,
	0x401A, 0x00,
	0x401B, 0x01,
	/*PDAF TYPE Setting*/
	0x3400, 0x02,
	/*PDAF TYPE2 Setting*/
	0x3093, 0x01,

};

static kal_uint16 imx682_custom5_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x24,
	0x0343, 0xD8,
	/*Frame Length Lines Setting*/
	0x0340, 0x0E,
	0x0341, 0xA9,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x24,
	0x0349, 0x1F,
	0x034A, 0x1B,
	0x034B, 0x1F,
	/*Mode Setting*/
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x30D8, 0x04,
	0x3200, 0x41,
	0x3201, 0x41,
	/*Digital Crop & Scaling*/
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x12,
	0x040D, 0x10,
	0x040E, 0x0D,
	0x040F, 0x90,
	/*Output Size Setting*/
	0x034C, 0x12,
	0x034D, 0x10,
	0x034E, 0x0D,
	0x034F, 0x90,
	/*Clock Setting*/
	0x0301, 0x08,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x62,
	0x030B, 0x02,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x85,
	0x0310, 0x01,
	/*Other Setting*/
	0x30D9, 0x00,
	0x32D5, 0x00,
	0x32D6, 0x00,
	0x401E, 0x00,
	0x40B8, 0x01,
	0x40B9, 0x2C,
	0x40BC, 0x01,
	0x40BD, 0x18,
	0x40BE, 0x00,
	0x40BF, 0x00,
	0x41A4, 0x00,
	0x5A09, 0x01,
	0x5A17, 0x01,
	0x5A25, 0x01,
	0x5A33, 0x01,
	0x98D7, 0xB4,
	0x98D8, 0x8C,
	0x98D9, 0x0A,
	0x99C4, 0x16,
	/*Integration Setting*/
	0x0202, 0x0E,
	0x0203, 0x79,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	/*PDAF Setting*/
	0x4018, 0x04,
	0x4019, 0x80,
	0x401A, 0x00,
	0x401B, 0x01,
	/*PDAF TYPE Setting*/
	0x3400, 0x02,
	/*PDAF TYPE2 Setting*/
	0x3093, 0x01,

};

#endif

static void sensor_init(void)
{
	LOG_INF("[%s] start\n", __func__);

	imx682_table_write_cmos_sensor(imx682_init_setting,
		sizeof(imx682_init_setting)/sizeof(kal_uint16));

	LOG_INF("[%s] End\n", __func__);
}	/*	  sensor_init  */

static void preview_setting(void)
{
	LOG_INF("%s +\n", __func__);

	//Feiping.Li, 60fps
	imx682_table_write_cmos_sensor(imx682_preview_setting,
		sizeof(imx682_preview_setting)/sizeof(kal_uint16));

	LOG_INF("%s -\n", __func__);
} /* preview_setting */


/*full size 30fps*/
static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("%s 30 fps E! currefps:%d\n", __func__, currefps);
	/*************MIPI output setting************/

	imx682_table_write_cmos_sensor(imx682_capture_setting,
		sizeof(imx682_capture_setting)/sizeof(kal_uint16));

	LOG_INF("%s 30 fpsX\n", __func__);
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("%s E! currefps:%d\n", __func__, currefps);

	imx682_table_write_cmos_sensor(imx682_normal_video_setting,
		sizeof(imx682_normal_video_setting)/sizeof(kal_uint16));

	LOG_INF("X\n");
}

static void hs_video_setting(void)
{
	LOG_INF("%s E! currefps 120\n", __func__);

	imx682_table_write_cmos_sensor(imx682_hs_video_setting,
		sizeof(imx682_hs_video_setting)/sizeof(kal_uint16));

	LOG_INF("X\n");
}

static void slim_video_setting(void)
{
	LOG_INF("%s E! 4000*2256@30fps\n", __func__);

	imx682_table_write_cmos_sensor(imx682_slim_video_setting,
		sizeof(imx682_slim_video_setting)/sizeof(kal_uint16));

	LOG_INF("X\n");
}

static void custom1_setting(void)
{
	LOG_INF("%s CUS1_12M_60_FPS E! currefps\n", __func__);
	/*************MIPI output setting************/

	imx682_table_write_cmos_sensor(imx682_custom1_setting,
		sizeof(imx682_custom1_setting)/sizeof(kal_uint16));

	LOG_INF("X");
}

static void custom2_setting(void)
{
	LOG_INF("%s 3840*2160@60fps E! currefps\n", __func__);
	/*************MIPI output setting************/

	imx682_table_write_cmos_sensor(imx682_custom2_setting,
		sizeof(imx682_custom2_setting)/sizeof(kal_uint16));

	LOG_INF("X");
}

/*full size 30fps*/
static void custom3_setting(void)
{
	LOG_INF("%s 30 fps E!\n", __func__);
	/*************MIPI output setting************/

	imx682_table_write_cmos_sensor(imx682_custom3_setting,
		sizeof(imx682_custom3_setting)/sizeof(kal_uint16));

	LOG_INF("%s 30 fpsX\n", __func__);
}

static void custom4_setting(void)
{
	LOG_INF("%s 720p@240 fps E! currefps\n", __func__);

	imx682_table_write_cmos_sensor(imx682_custom4_setting,
		sizeof(imx682_custom4_setting)/sizeof(kal_uint16));

	LOG_INF("X\n");
}

static void custom5_setting(void)
{
	LOG_INF("%s 720p@240 fps E! currefps\n", __func__);

	imx682_table_write_cmos_sensor(imx682_custom5_setting,
		sizeof(imx682_custom5_setting)/sizeof(kal_uint16));

	LOG_INF("X\n");
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
static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor_8(0x0016) << 8)
			| read_cmos_sensor_8(0x0017));
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/*sensor have two i2c address 0x34 & 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			printk("IMX682 get_imgsensor_id = 0x%x\n",*sensor_id);
			if (*sensor_id == imgsensor_info.sensor_id) {
				printk("IMX682 get_imgsensor_id success: 0x%x\n", *sensor_id);
				#ifdef OPLUS_FEATURE_CAMERA_COMMON
				imgsensor_info.module_id = read_module_id();
				if (deviceInfo_register_value == 0x00) {
					Oplusimgsensor_Registdeviceinfo("Cam_r", sensor_name_golden_result, imgsensor_info.module_id);
					deviceInfo_register_value=0x01;
				}
				#endif	//OPLUS_FEATURE_CAMERA_COMMON
				read_sensor_Cali();

				return ERROR_NONE;
			}
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		printk("IMX682 get_imgsensor_id failed: 0x%x\n", *sensor_id);
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

	//LOG_INF("%s +\n", __func__);
	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				printk("IMX682 open success: 0x%x\n", sensor_id);
				break;
			}
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if (imgsensor_info.sensor_id != sensor_id) {
		printk("IMX682 open failed: 0x%x\n", sensor_id);
		return ERROR_SENSORID_READ_FAIL;
	}
	#endif
	/* initail sequence write in  */
	sensor_init();

	#if SUPPORT_LRC //imx682 pd cal didn't use LRC
		write_sensor_LRC();
	#endif
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
	//LOG_INF("%s -\n", __func__);

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
	set_mirror_flip(imgsensor.mirror);

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

	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		LOG_INF(
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
}	/*	normal_video   */

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
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s. 4k@30FPS\n", __func__);

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
}	/* custom1 */

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
}	/* custom2 */

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
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* custom3 */

static kal_uint32 custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s. 2312*1296@240FPS\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
	imgsensor.pclk = imgsensor_info.custom4.pclk;
	imgsensor.line_length = imgsensor_info.custom4.linelength;
	imgsensor.frame_length = imgsensor_info.custom4.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	if (!qsc_flag) {
		LOG_INF("write_sensor_QSC Start\n");
		mdelay(67);  //add delay to ensure QSC wirte success
		write_sensor_QSC();
		LOG_INF("write_sensor_QSC End\n");
		qsc_flag = 1;
	}

	custom4_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* custom4 */

static kal_uint32 custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s. 4600*2992@60FPS\n", __func__);

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
}	/* custom4 */

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
	if(framerate <= 100){
		framerate = 100;
	}

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

static kal_uint32 set_test_pattern_mode(kal_uint8 modes, struct SET_SENSOR_PATTERN_SOLID_COLOR *pTestpatterndata)
{
	kal_uint16 Color_R, Color_Gr, Color_Gb, Color_B;
	if (pTestpatterndata != NULL) {
		Color_R = (pTestpatterndata->COLOR_R >> 16) & 0xFFFF;
		Color_Gr = (pTestpatterndata->COLOR_Gr >> 16) & 0xFFFF;
		Color_B = (pTestpatterndata->COLOR_B >> 16) & 0xFFFF;
		Color_Gb = (pTestpatterndata->COLOR_Gb >> 16) & 0xFFFF;
		pr_info("355 set_test_pattern enum: %d\n", modes);
		pr_info("R %x GR %x B %x GB %x",pTestpatterndata->COLOR_R, pTestpatterndata->COLOR_Gr, pTestpatterndata->COLOR_B, pTestpatterndata->COLOR_Gb);
		pr_info("SETTING R %x GR %x B %x GB %x", Color_R, Color_Gr,Color_B,Color_Gb);
	}

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
		write_cmos_sensor(0x0600, 0x0000); /*No pattern*/
		write_cmos_sensor(0x0601, 0x0000);
	}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = modes;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

/*jiangtao.ren@Camera add for support af temperature compensation 20191101*/
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
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
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
		set_test_pattern_mode((UINT8)*feature_data, (struct SET_SENSOR_PATTERN_SOLID_COLOR *)feature_data+1);
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
		imgsensor.ihdr_mode = (UINT8)*feature_data_32;
		// imgsensor.hdr_mode = 9;
		// force set hdr_mode to zHDR
		spin_unlock(&imgsensor_drv_lock);
		break;

	case SENSOR_FEATURE_GET_CROP_INFO:
		/* pr_debug("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
		 *	(UINT32) *feature_data);
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
		/* modify to separate 3hdr and remosaic */
		if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM4) {
			/*write AWB gain to sensor*/
			feedback_awbgain((UINT32)*(feature_data_32 + 1),
					(UINT32)*(feature_data_32 + 2));
		} else {
			imx682_awb_gain(
				(struct SET_SENSOR_AWB_GAIN *) feature_para);
		}
		break;
	case SENSOR_FEATURE_SET_LSC_TBL:
		{
		kal_uint8 index =
			*(((kal_uint8 *)feature_para) + (*feature_para_len));

		imx682_set_lsc_reg_setting(index, feature_data_16,
					  (*feature_para_len)/sizeof(UINT16));
		}
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		pr_debug("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16) *feature_data,
			(UINT16) *(feature_data + 1));
/* ihdr_write_shutter((UINT16)*feature_data,(UINT16)*(feature_data+1)); */
		break;

	case SENSOR_FEATURE_GET_4CELL_DATA:/*get 4 cell data from eeprom*/
	{
/*
		int type = (kal_uint16)(*feature_data);
		char *data = (char *)(uintptr_t)(*(feature_data+1));

		if (type == FOUR_CELL_CAL_TYPE_XTALK_CAL) {
			pr_debug("Read Cross Talk Start");
			read_4cell_from_eeprom(data);
			pr_debug("Read Cross Talk = %02x %02x %02x %02x %02x %02x\n",
				(UINT16)data[0], (UINT16)data[1],
				(UINT16)data[2], (UINT16)data[3],
				(UINT16)data[4], (UINT16)data[5]);
		}
*/
		break;
	}

	case SENSOR_FEATURE_GET_PDAF_INFO:
		pr_debug("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
			(UINT16)*feature_data);
		PDAFinfo =
			(struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data + 1));
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CUSTOM3: //3056*1728
			imgsensor_pd_info.i4BlockNumX = 380;
			imgsensor_pd_info.i4BlockNumY = 107;
			memcpy((void *)PDAFinfo,
			(void *)&imgsensor_pd_info,
			sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM5:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW: //4624*3472
			imgsensor_pd_info.i4BlockNumX = 574;
			imgsensor_pd_info.i4BlockNumY = 215;
			memcpy((void *)PDAFinfo,
			(void *)&imgsensor_pd_info,
			sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW: //4608*2592
			imgsensor_pd_info.i4BlockNumX = 574;
			imgsensor_pd_info.i4BlockNumY = 161;
			memcpy((void *)PDAFinfo,
			(void *)&imgsensor_pd_info,
			sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		default:
			break;
		}
		break;

	case SENSOR_FEATURE_GET_VC_INFO:
		pr_debug("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
		pvcinfo = (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM5:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		default:
			break;
		}
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		pr_debug("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", (UINT16)*feature_data);
		//PDAF capacity enable or not
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_REG_SETTING %d",
			(*feature_para_len));
		imx682_get_pdaf_reg_setting((*feature_para_len) / sizeof(UINT32)
					   , feature_data_16);
		break;
	case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
		LOG_INF("SENSOR_FEATURE_SET_PDAF_REG_SETTING %d",
			(*feature_para_len));
		imx682_set_pdaf_reg_setting((*feature_para_len) / sizeof(UINT32)
					   , feature_data_16);
		break;
	case SENSOR_FEATURE_SET_PDAF:
		pr_debug("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16)(*feature_data),
						(UINT16)(*(feature_data + 1)), (BOOL) (*(feature_data + 2)));
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
		*feature_return_para_32 = get_sensor_temperature();
		*feature_para_len = 4;
		break;
    #ifdef OPLUS_FEATURE_CAMERA_COMMON
    case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
        *(feature_data + 1) = 1; /* margin info by scenario */
        *(feature_data + 2) = imgsensor_info.margin;
        break;
    #endif

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

UINT32 IMX682_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
} /* IMX682_MIPI_RAW_SensorInit */
