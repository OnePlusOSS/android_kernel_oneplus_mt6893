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
 *	 limu_shinetech_s5kjn103mipi_Sensor.c
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
#include "imgsensor_hwcfg_custom.h"

#include "limu_shinetech_s5kjn103_mipiraw_Sensor.h"
//#include "imgsensor_common.h"

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include <soc/oplus/system/oplus_project.h>
#endif

/***************Modify Following Strings for Debug**********************/
#define PFX "limu_shinetech_s5kjn103_camera_sensor"
#define LOG_1 LOG_INF("limu_shinetech_s5kjn103,MIPI 4LANE\n")
/****************************   Modify end	**************************/
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define LOG_ERR(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)

#define USE_BURST_MODE 1

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define DEVICE_VERSION_S5KJN103	 "s5kjn103"
extern void register_imgsensor_deviceinfo(char *name, char *version, u8 module_id);
static kal_uint8 deviceInfo_register_value = 0x00;
static kal_uint32 streaming_control(kal_bool enable);
#define MODULE_ID_OFFSET 0x0000
#define I2C_BUFFER_LEN 1020	/* trans# max is 255, each 4 bytes */

extern enum IMGSENSOR_RETURN Eeprom_DataInit(
    enum IMGSENSOR_SENSOR_IDX sensor_idx,
    kal_uint32 sensorID);
#endif

static kal_uint32 set_test_pattern_mode(kal_bool enable);
static kal_uint16 table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len);
static bool bNeedSetNormalMode = KAL_FALSE;
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = LIMU_SHINETECH_MAIN_S5KJN103_SENSOR_ID,
	.module_id = 0x07,	//0x01 Sunny,0x05 QTEK
	.checksum_value = 0x8ac2d94a,

	.pre = {
		.pclk = 600000000,
		.linelength = 5824,
		.framelength = 1716,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 2040,
		.grabwindow_height = 1536,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 320000000,
	},

	.cap = {
		.pclk = 560000000,
		.linelength = 5888,
		.framelength = 3168,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
		.mipi_pixel_rate = 588800000,
	},

	.normal_video = { /*4000*2252@30fps*/
		.pclk = 560000000,
		.linelength = 5888,
		.framelength = 3168,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 2296,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 588800000,
		.max_framerate = 300,
	},

	.hs_video = { /* 1920x1080 @120fps (binning)*/
		.pclk = 482000000,
		.linelength = 2512,
		.framelength = 1599,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 476800000,
		.max_framerate = 1200,
	},

	.slim_video = { /* 4080*2296@30fps */
		.pclk = 560000000,
		.linelength = 5888,
		.framelength = 3168,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 2296,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 588800000,
		.max_framerate = 300,
	},

	.custom1 = {
		.pclk = 560000000,
		.linelength = 6624,
		.framelength = 2816,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 432000000,
	},

	.custom2 = {
		.pclk = 560000000,
		.linelength = 6624,
		.framelength = 3520,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,
		.mipi_pixel_rate = 432000000,
	},

	.margin = 10,		/* sensor framelength & shutter margin */
	.min_shutter = 4,	/* min shutter */

	.min_gain = 64,
	.max_gain = 4096,
	.min_gain_iso = 100,
	.gain_step = 2,
	.gain_type = 2, //0-SONY; 1-OV; 2 - SUMSUN; 3 -HYNIX; 4 -GC

	.max_frame_length = 0xfffd,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.sensor_mode_num = 7,	/* support sensor mode num */

	.cap_delay_frame = 3,	/* enter capture delay frame num */
	.pre_delay_frame = 3,	/* enter preview delay frame num */
	.video_delay_frame = 3,	/* enter video delay frame num */
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,	/* enter slim video delay frame num */
	.custom1_delay_frame = 3,	/* enter custom1 delay frame num */
	.custom2_delay_frame = 3,	/* enter custom2 delay frame num */
	.frame_time_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, /* 0,MIPI_OPHY_NCSI2; 1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 1,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 24, /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x5a, 0xff},
	.i2c_speed = 1000, /* i2c read/write speed */
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
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x5a, /* record current sensor's i2c write id */
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[7] = {
	{4080, 3072,  0, 0, 4080, 3072, 2040, 1536, 0, 0, 2040, 1536, 0, 0, 2040, 1536}, /* Preview*/
	{4080, 3072,  0, 0, 4080, 3072, 4080, 3072, 0, 0, 4080, 3072, 0, 0, 4080, 3072}, /* capture */
	{4080, 3072,  0, 388, 4080, 2296, 4080, 2296, 0, 0, 4080, 2296, 0, 0, 4080, 2296}, /* normal video */
	{4080, 3072, 120, 456, 3840, 2160, 1920, 1080, 0, 0, 1920, 1080, 0, 0, 1920, 1080}, /* hs_video */
	{4080, 3072,  0, 388, 4080, 2296, 4080, 2296, 0, 0, 4080, 2296, 0, 0, 4080, 2296}, /* slim video */
	{4080, 3072,  408, 312, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448}, /* custom1 */
	{4080, 3072,  408, 312, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448}, /* custom2 */
};

/*VC1 for HDR(DT=0X35), VC2 for PDAF(DT=0X30), unit : 10bit */
static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[4] = {
	/* Capture mode setting */
	{0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
	 0x00, 0x2B, 0x0FF0, 0x0C00, 0x01, 0x00, 0x0000, 0x0000,
	 0x01, 0x30, 0x027C, 0x0BF0, 0x03, 0x00, 0x0000, 0x0000},
	 /* Video mode setting */
	{0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
	0x00, 0x2B, 0x0FF0, 0x0C00, 0x01, 0x00, 0x0000, 0x0000,
	0x01, 0x30, 0x027C, 0x08E8, 0x03, 0x00, 0x0000, 0x0000},
	 /* Custom1 mode setting */
	{0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
	 0x00, 0x2B, 0x0FF0, 0x0C00, 0x01, 0x00, 0x0000, 0x0000,
	 0x01, 0x30, 0x0200, 0x0980, 0x03, 0x00, 0x0000, 0x0000},
	 /* Preview mode setting */
	{0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
	 0x00, 0x2B, 0x0FF0, 0x0C00, 0x01, 0x00, 0x0000, 0x0000,
	 0x01, 0x30, 0x027C, 0x02FC, 0x03, 0x00, 0x0000, 0x0000}
};

/* If mirror flip */
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
	.i4OffsetX =  8,
	.i4OffsetY =  8,
	.i4PitchX  =  8,
	.i4PitchY  =  8,
	.i4PairNum  = 4,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = {{8, 8},{10, 11},{14, 12},{12, 15}},
	.i4PosR = {{9, 8},{11, 11},{15, 12},{13, 15}},
	.i4BlockNumX = 508,
	.i4BlockNumY = 382,
	.iMirrorFlip = 0,
	.i4Crop = { {0,0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}},
};
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_4080_2296 =
{
	.i4OffsetX = 8,
	.i4OffsetY = 8,
	.i4PitchX  = 8,
	.i4PitchY  = 8,
	.i4PairNum  = 4,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = {{8, 8},{10, 11},{14, 12},{12, 15}},
	.i4PosR = {{9, 8},{11, 11},{15, 12},{13, 15}},
	.iMirrorFlip = 0,
	.i4BlockNumX = 508,
	.i4BlockNumY = 285,
	.i4Crop = {{0,0}, {0,0}, {0,388}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_3264_2448_custom1 =
{
	.i4OffsetX =  0,
	.i4OffsetY =  8,
	.i4PitchX  =  8,
	.i4PitchY  =  8,
	.i4PairNum  = 4,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = {{0, 8},{2, 11},{6, 12},{4, 15}},
	.i4PosR = {{1, 8},{3, 11},{7, 12},{5, 15}},
	.i4BlockNumX = 408,
	.i4BlockNumY = 304,
	.i4LeFirst = 0,
	.iMirrorFlip = 0,
	.i4Crop = { {0,0}, {0, 0}, {0, 0}, {0, 0}, {408, 312}, {0, 0}, {0, 0}},
};
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_3264_2448_custom2 =
{
	.i4OffsetX = 0,
	.i4OffsetY = 8,
	.i4PitchX  = 8,
	.i4PitchY  = 8,
	.i4PairNum  =4,
	.i4SubBlkW  =8,
	.i4SubBlkH  =2,
	.i4PosL = {{0, 8},{2, 11},{6, 12},{4, 15}},
	.i4PosR = {{1, 8},{3, 11},{7, 12},{5, 15}},
	.i4BlockNumX = 408,
	.i4BlockNumY = 304,
	.i4LeFirst = 0,
	.iMirrorFlip = 0,
	.i4Crop = { {0,0}, {0, 0}, {0, 0}, {0, 0}, {408, 312}, {0, 0}, {0, 0}},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_2040_1536_preview =
{
	.i4OffsetX =  8,
	.i4OffsetY =  8,
	.i4PitchX  =  8,
	.i4PitchY  =  8,
	.i4PairNum  = 1,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 8,
	.i4PosL = {{9, 8}},
	.i4PosR = {{8, 8}},
	.i4BlockNumX = 508,
	.i4BlockNumY = 382,
	.iMirrorFlip = 0,
	.i4Crop = { {0,0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}},
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

	iReadRegI2CTiming(pusendcmd, 2, (u8 *)&get_byte, 2, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
	return ((get_byte<<8)&0xff00) | ((get_byte>>8)&0x00ff);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF),
				 (char)(para >> 8), (char)(para & 0xFF)};

	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);*/
	/* Add this func to set i2c speed by each sensor */
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
#if 0
#define  CAMERA_MODULE_INFO_LENGTH  (8)
static kal_uint8 CAM_SN[CAMERA_MODULE_SN_LENGTH];
static kal_uint8 CAM_INFO[CAMERA_MODULE_INFO_LENGTH];
static kal_uint8 CAM_DUAL_DATA[DUALCAM_CALI_DATA_LENGTH_8ALIGN];
static kal_uint16 read_cmos_eeprom_8(kal_uint16 addr)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 1, 0xA0);
	return get_byte;
}

static void read_eepromData(void)
{
	kal_uint16 idx = 0;
	for (idx = 0; idx <DUALCAM_CALI_DATA_LENGTH; idx++) {
		CAM_DUAL_DATA[idx] = read_cmos_eeprom_8(S5KJN103_STEREO_START_ADDR + idx);
	}
	for (idx = 0; idx <CAMERA_MODULE_SN_LENGTH; idx++) {
		CAM_SN[idx] = read_cmos_eeprom_8(0xB0 + idx);
		LOG_INF("CAM_SN[%d]: 0x%x  0x%x\n", idx, CAM_SN[idx]);
	}
	CAM_INFO[0] = read_cmos_eeprom_8(0x0);
	CAM_INFO[1] = read_cmos_eeprom_8(0x1);
	CAM_INFO[2] = read_cmos_eeprom_8(0x6);
	CAM_INFO[3] = read_cmos_eeprom_8(0x7);
	CAM_INFO[4] = read_cmos_eeprom_8(0x8);
	CAM_INFO[5] = read_cmos_eeprom_8(0x9);
	CAM_INFO[6] = read_cmos_eeprom_8(0xA);
	CAM_INFO[7] = read_cmos_eeprom_8(0xB);
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
		pusendcmd[2] = 0x0E;
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
		LOG_INF("SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
					   pStereodata->uSensorId,
					   pStereodata->uDeviceId,
					   pStereodata->baseAddr,
					   pStereodata->dataLength);

		data_base = pStereodata->baseAddr;
		data_length = pStereodata->dataLength;
		pData = pStereodata->uData;
		if ((pStereodata->uSensorId == S5KJN103_SENSOR_ID) && (data_length == DUALCAM_CALI_DATA_LENGTH)
			&& (data_base == S5KJN103_STEREO_START_ADDR)) {
			LOG_INF("Write: %x %x %x %x %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556],
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
					LOG_ERR("write_eeprom error: i= %d\n", i);
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
				LOG_ERR("write_eeprom error: idx= %d idy= %d\n", idx, idy);
				/* open write protect */
				write_eeprom_protect(1);
				msleep(6);
				return IMGSENSOR_RETURN_ERROR;
			}
			msleep(6);
			/* open write protect */
			write_eeprom_protect(1);
			msleep(6);
			LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(S5KJN103_STEREO_START_ADDR));
			LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(S5KJN103_STEREO_START_ADDR+39));
			LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(S5KJN103_STEREO_START_ADDR+40));
			LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(S5KJN103_STEREO_START_ADDR+1556));
			LOG_INF("tail1_1557:0x%x\n", read_cmos_eeprom_8(S5KJN103_STEREO_START_ADDR+1557));
			LOG_INF("tail2_1558:0x%x\n", read_cmos_eeprom_8(S5KJN103_STEREO_START_ADDR+1558));
			LOG_INF("tail3_1559:0x%x\n", read_cmos_eeprom_8(S5KJN103_STEREO_START_ADDR+1559));
			LOG_INF("tail4_1560:0x%x\n", read_cmos_eeprom_8(S5KJN103_STEREO_START_ADDR+1560));
			LOG_INF("write_Module_data Write end\n");
		} else {
			LOG_ERR("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
			return IMGSENSOR_RETURN_ERROR;
		}
	} else {
		LOG_ERR("s5kgh1 write_Module_data pStereodata is null\n");
		return IMGSENSOR_RETURN_ERROR;
	}
	return ret;
}
#endif
#define JN1_HW_GGC_SIZE 173
#define JN1_HW_GGC_START_ADDR 0x3AE0

static kal_uint16 JN1_HW_GGC[JN1_HW_GGC_SIZE];
static kal_uint16 JN1_HW_GGC_setting[JN1_HW_GGC_SIZE*2];

static kal_uint16 read_cmos_eeprom_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, 0xA0);
	return get_byte;
}

static void read_eepromData_HW_GGC(void)
{
	kal_uint16 idx = 0,idx_2 = 0;
	for (idx = 0; idx <(JN1_HW_GGC_SIZE*2);idx = idx + 2) {
		JN1_HW_GGC[idx_2] = ((read_cmos_eeprom_8(JN1_HW_GGC_START_ADDR + idx)<<8)&0xff00) | (read_cmos_eeprom_8(JN1_HW_GGC_START_ADDR + idx + 1)&0x00ff);
		idx_2++;
	}
	LOG_INF("read HW_GGC from eeprom data[0]: 0x%x,data[1]: 0x%x,data[2]: 0x%x,data[3]: 0x%x,data[4]: 0x%x,data[5]: 0x%x,data[6]: 0x%x,data[7]: 0x%x",
			read_cmos_eeprom_8(JN1_HW_GGC_START_ADDR),
			read_cmos_eeprom_8(JN1_HW_GGC_START_ADDR+1),
			read_cmos_eeprom_8(JN1_HW_GGC_START_ADDR+2),
			read_cmos_eeprom_8(JN1_HW_GGC_START_ADDR+3),
			read_cmos_eeprom_8(JN1_HW_GGC_START_ADDR+4),
			read_cmos_eeprom_8(JN1_HW_GGC_START_ADDR+5),
			read_cmos_eeprom_8(JN1_HW_GGC_START_ADDR+6),
			read_cmos_eeprom_8(JN1_HW_GGC_START_ADDR+7));
	for (idx = 0; idx <JN1_HW_GGC_SIZE; idx++) {
		JN1_HW_GGC_setting[2*idx] = 0x6F12;
		JN1_HW_GGC_setting[2*idx + 1] = JN1_HW_GGC[idx];
	}
	LOG_INF("JN1_HW_GGC_setting[0]:0x%x,JN1_HW_GGC_setting[1]:0x%x,JN1_HW_GGC_setting[2]:0x%x,JN1_HW_GGC_setting[3]:0x%x,JN1_HW_GGC_setting[4]:0x%x,JN1_HW_GGC_setting[5]:0x%x,JN1_HW_GGC_setting[6]:0x%x,JN1_HW_GGC_setting[7]:0x%x ",
			JN1_HW_GGC_setting[0],JN1_HW_GGC_setting[1],JN1_HW_GGC_setting[2],JN1_HW_GGC_setting[3],JN1_HW_GGC_setting[4],JN1_HW_GGC_setting[5],JN1_HW_GGC_setting[6],JN1_HW_GGC_setting[7]);
}

static void write_sensor_HW_GGC(void)
{
	write_cmos_sensor(0x6028,0x2400);
	write_cmos_sensor(0x602A,0x0CFC);
	#if USE_BURST_MODE
	table_write_cmos_sensor(JN1_HW_GGC_setting,
		sizeof(JN1_HW_GGC_setting)/sizeof(kal_uint16));
	#else
	kal_uint16 idx = 0;
	for (idx = 0; idx <JN1_HW_GGC_SIZE; idx++) {
		write_cmos_sensor_8(0x6F12, JN1_HW_GGC[idx]);
	}
	#endif
}

static void set_dummy(void)
{
	write_cmos_sensor(0x0340, imgsensor.frame_length);
	write_cmos_sensor(0x0342, imgsensor.line_length);
	LOG_INF("dummyline = %d, dummypixels = %d \n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);
}	/*	set_dummy  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
	kal_uint8 itemp;
	LOG_INF("image_mirror = %d\n", image_mirror);
	itemp=read_cmos_sensor(0x0101);
	LOG_INF("image_mirror itemp = %d\n", itemp);
	itemp &= ~0x03;

	switch(image_mirror) {
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

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d\n", framerate,
		min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length) {
		imgsensor.frame_length = frame_length;
	} else {
		imgsensor.frame_length = imgsensor.min_frame_length;
	}
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en) {
		imgsensor.min_frame_length = imgsensor.frame_length;
	}
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

		if (shutter >= 3448275) {  //>32s
			shutter = 3448275;
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
	if ( imgsensor.sensor_mode == IMGSENSOR_MODE_PREVIEW ) {
		shutter = shutter / 2 * 2;
	}
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
		write_cmos_sensor(0x0702, 0x0600);
		write_cmos_sensor(0x0704, 0x0600);
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
	 kal_uint16 reg_gain = 0x0;

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

	if (gain < BASEGAIN || gain > 64 * BASEGAIN) {
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN) {
			gain = BASEGAIN;
		} else if (gain > 64 * BASEGAIN) {
			gain = 64 * BASEGAIN;
		}
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_8(0x0204, (reg_gain >> 8));
	write_cmos_sensor_8(0x0205, (reg_gain & 0xff));

	return gain;
}	/*	set_gain  */

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static void read_limu_shinetech_main_s5kjn103_SNData(UINT8 *limu_shinetech_main_s5kjn103_SNData,
    UINT32 *feature_para_len) {

    kal_uint16 start_addr=0x00B0;
    kal_uint16 checksum = 0;
    UINT8 i;

    for(i = 0; i < 17; i++){
		limu_shinetech_main_s5kjn103_SNData[i] = (kal_int8)read_cmos_eeprom_8(start_addr + i);
		LOG_INF("read_cmos_eeprom_8(start_addr + i) = 0x%x\n", read_cmos_eeprom_8(start_addr + i));
		checksum = checksum + limu_shinetech_main_s5kjn103_SNData[i];
	}
	checksum = checksum % 255;
	if(read_cmos_eeprom_8(start_addr+17) == 1 && read_cmos_eeprom_8(start_addr+18) == checksum ) {
		*feature_para_len = i;
		LOG_INF("limu_shinetech_main_s5kjn103_SNData checksum sucess, feature_para_len；%d", *feature_para_len);
	} else {
		*feature_para_len = 0;
		LOG_INF("limu_shinetech_main_s5kjn103_SNData checksum failed");
	}
}
#endif

static void limu_shinetech_s5kjn103_set_lsc_reg_setting(
		kal_uint8 index, kal_uint16 *regDa, MUINT32 regNum)
{
#if 0
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
#endif
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
	int timeout = (10000 / imgsensor.current_fps) + 1;
	int i = 0;
	int framecnt = 0;

	LOG_INF("streaming_enable(0= Sw Standby,1= streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor_8(0x0100, 0x01);
		mDELAY(10);
	} else {
		write_cmos_sensor_8(0x0100, 0x00);
		for (i = 0; i < timeout; i++) {
			mDELAY(5);
			framecnt = read_cmos_sensor_8(0x0005);
			if (framecnt == 0xFF) {
				LOG_INF(" Stream Off OK at i=%d.\n", i);
				return ERROR_NONE;
			}
		}
		LOG_INF("Stream Off Fail! framecnt= %d.\n", framecnt);
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
		if ((I2C_BUFFER_LEN - tosend) < 4
			|| IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd,
						tosend,
						imgsensor.i2c_write_id,
						4,
						imgsensor_info.i2c_speed);
			tosend = 0;
		}
	}
	return 0;
}

static kal_uint16 limu_shinetech_s5kjn103_init_setting[] = {
	0x6028,0x2400,
	0x602A,0x1354,
	0x6F12,0x0100,
	0x6F12,0x7017,
	0x602A,0x13B2,
	0x6F12,0x0000,
	0x602A,0x1236,
	0x6F12,0x0000,
	0x602A,0x1A0A,
	0x6F12,0x4C0A,
	0x602A,0x2210,
	0x6F12,0x3401,
	0x602A,0x2176,
	0x6F12,0x6400,
	0x602A,0x222E,
	0x6F12,0x0001,
	0x602A,0x06B6,
	0x6F12,0x0A00,
	0x602A,0x06BC,
	0x6F12,0x1001,
	0x602A,0x2140,
	0x6F12,0x0101,
	0x602A,0x1A0E,
	0x6F12,0x9600,
	0x6028,0x4000,
	0xF44E,0x0011,
	0xF44C,0x0B0B,
	0xF44A,0x0006,
	0x0118,0x0002,
	0x011A,0x0001,
};

static kal_uint16 limu_shinetech_s5kjn103_capture_setting[] = {
	0x6028,0x2400,
	0x602A,0x1A28,
	0x6F12,0x4C00,
	0x602A,0x065A,
	0x6F12,0x0000,
	0x602A,0x139E,
	0x6F12,0x0100,
	0x602A,0x139C,
	0x6F12,0x0000,
	0x602A,0x13A0,
	0x6F12,0x0A00,
	0x6F12,0x0120,
	0x602A,0x2072,
	0x6F12,0x0000,
	0x602A,0x1A64,
	0x6F12,0x0301,
	0x6F12,0xFF00,
	0x602A,0x19E6,
	0x6F12,0x0200,
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
	0x6F12,0xA060,
	0x602A,0x1A3C,
	0x6F12,0x6207,
	0x602A,0x1A48,
	0x6F12,0x6207,
	0x602A,0x1444,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x602A,0x144C,
	0x6F12,0x3F00,
	0x6F12,0x3F00,
	0x602A,0x7F6C,
	0x6F12,0x0100,
	0x6F12,0x2F00,
	0x6F12,0xFA00,
	0x6F12,0x2400,
	0x6F12,0xE500,
	0x602A,0x0650,
	0x6F12,0x0600,
	0x602A,0x0654,
	0x6F12,0x0000,
	0x602A,0x1A46,
	0x6F12,0x8A00,
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
	0x6F12,0x1800,
	0x602A,0x2148,
	0x6F12,0x0100,
	0x602A,0x2042,
	0x6F12,0x1A00,
	0x602A,0x0874,
	0x6F12,0x0100,
	0x602A,0x09C0,
	0x6F12,0x2008,
	0x602A,0x09C4,
	0x6F12,0x2000,
	0x602A,0x19FE,
	0x6F12,0x0E1C,
	0x602A,0x4D92,
	0x6F12,0x0100,
	0x602A,0x84C8,
	0x6F12,0x0100,
	0x602A,0x4D94,
	0x6F12,0x0005,
	0x6F12,0x000A,
	0x6F12,0x0010,
	0x6F12,0x0810,
	0x6F12,0x000A,
	0x6F12,0x0040,
	0x6F12,0x0810,
	0x6F12,0x0810,
	0x6F12,0x8002,
	0x6F12,0xFD03,
	0x6F12,0x0010,
	0x6F12,0x1510,
	0x602A,0x3570,
	0x6F12,0x0000,
	0x602A,0x3574,
	0x6F12,0x1201,
	0x602A,0x21E4,
	0x6F12,0x0400,
	0x602A,0x21EC,
	0x6F12,0x1F04,
	0x602A,0x2080,
	0x6F12,0x0101,
	0x6F12,0xFF00,
	0x6F12,0x7F01,
	0x6F12,0x0001,
	0x6F12,0x8001,
	0x6F12,0xD244,
	0x6F12,0xD244,
	0x6F12,0x14F4,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x602A,0x20BA,
	0x6F12,0x141C,
	0x6F12,0x111C,
	0x6F12,0x54F4,
	0x602A,0x120E,
	0x6F12,0x1000,
	0x602A,0x212E,
	0x6F12,0x0200,
	0x602A,0x13AE,
	0x6F12,0x0101,
	0x602A,0x0718,
	0x6F12,0x0001,
	0x602A,0x0710,
	0x6F12,0x0002,
	0x6F12,0x0804,
	0x6F12,0x0100,
	0x602A,0x1B5C,
	0x6F12,0x0000,
	0x602A,0x0786,
	0x6F12,0x7701,
	0x602A,0x2022,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x602A,0x1360,
	0x6F12,0x0100,
	0x602A,0x1376,
	0x6F12,0x0100,
	0x6F12,0x6038,
	0x6F12,0x7038,
	0x6F12,0x8038,
	0x602A,0x1386,
	0x6F12,0x0B00,
	0x602A,0x06FA,
	0x6F12,0x0000,
	0x602A,0x4A94,
	0x6F12,0x0900,
	0x6F12,0x0000,
	0x6F12,0x0300,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0300,
	0x6F12,0x0000,
	0x6F12,0x0900,
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
	0x6F12,0x0100,
	0x602A,0x0CF2,
	0x6F12,0x0001,
	0x602A,0x0CF0,
	0x6F12,0x0101,
	0x602A,0x11B8,
	0x6F12,0x0100,
	0x602A,0x11F6,
	0x6F12,0x0020,
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
	0x0344,0x0000,
	0x0346,0x0000,
	0x0348,0x1FFF,
	0x034A,0x181F,
	0x034C,0x0FF0,
	0x034E,0x0C00,
	0x0350,0x0008,
	0x0352,0x0008,
	0x0900,0x0122,
	0x0380,0x0002,
	0x0382,0x0002,
	0x0384,0x0002,
	0x0386,0x0002,
	0x0110,0x1002,
	0x0114,0x0301,
	0x0116,0x3000,
	0x0136,0x1800,
	0x013E,0x0000,
	0x0300,0x0006,
	0x0302,0x0001,
	0x0304,0x0004,
	0x0306,0x008C,
	0x0308,0x0008,
	0x030A,0x0001,
	0x030C,0x0000,
	0x030E,0x0003,
	0x0310,0x005C,
	0x0312,0x0000,
	0x080E,0x0000,
	0x0340,0x0C60,
	0x0342,0x1700,
	0x0702,0x0000,
	0x0202,0x0100,
	0x0200,0x0100,
	0x0D00,0x0101,
	0x0D02,0x0101,
	0x0D04,0x0102,
	0x011E,0x0000,
	0x6226,0x0000,
};

static kal_uint16 limu_shinetech_s5kjn103_preview_setting[] = {
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
	0x6F12,0x0002,
	0x6F12,0x8000,
	0x6F12,0x0002,
	0x6F12,0xC244,
	0x6F12,0xD244,
	0x6F12,0x14F4,
	0x6F12,0x161C,
	0x6F12,0x111C,
	0x6F12,0x54F4,
	0x602A,0x20BA,
	0x6F12,0x0000,
	0x6F12,0x0000,
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
	0x6F12,0x0000,
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
	0x0344,0x0000,
	0x0346,0x0000,
	0x0348,0x1FFF,
	0x034A,0x181F,
	0x034C,0x07F8,
	0x034E,0x0600,
	0x0350,0x0004,
	0x0352,0x0004,
	0x0900,0x0144,
	0x0380,0x0002,
	0x0382,0x0006,
	0x0384,0x0002,
	0x0386,0x0006,
	0x0110,0x1002,
	0x0114,0x0301,
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
	0x0312,0x0001,
	0x080E,0x0000,
	0x0340,0x06B4,
	0x0342,0x16C0,
	0x0702,0x0000,
	0x0202,0x0100,
	0x0200,0x0100,
	0x0D00,0x0101,
	0x0D02,0x0101,
	0x0D04,0x0102,
	0x011E,0x0000,
	0x6226,0x0000,
};

static kal_uint16 limu_shinetech_s5kjn103_normal_video_setting[] = {
	0x6028,0x2400,
	0x602A,0x1A28,
	0x6F12,0x4C00,
	0x602A,0x065A,
	0x6F12,0x0000,
	0x602A,0x139E,
	0x6F12,0x0100,
	0x602A,0x139C,
	0x6F12,0x0000,
	0x602A,0x13A0,
	0x6F12,0x0A00,
	0x6F12,0x0120,
	0x602A,0x2072,
	0x6F12,0x0000,
	0x602A,0x1A64,
	0x6F12,0x0301,
	0x6F12,0xFF00,
	0x602A,0x19E6,
	0x6F12,0x0200,
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
	0x6F12,0xA060,
	0x602A,0x1A3C,
	0x6F12,0x6207,
	0x602A,0x1A48,
	0x6F12,0x6207,
	0x602A,0x1444,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x602A,0x144C,
	0x6F12,0x3F00,
	0x6F12,0x3F00,
	0x602A,0x7F6C,
	0x6F12,0x0100,
	0x6F12,0x2F00,
	0x6F12,0xFA00,
	0x6F12,0x2400,
	0x6F12,0xE500,
	0x602A,0x0650,
	0x6F12,0x0600,
	0x602A,0x0654,
	0x6F12,0x0000,
	0x602A,0x1A46,
	0x6F12,0x8A00,
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
	0x6F12,0x1800,
	0x602A,0x2148,
	0x6F12,0x0100,
	0x602A,0x2042,
	0x6F12,0x1A00,
	0x602A,0x0874,
	0x6F12,0x0100,
	0x602A,0x09C0,
	0x6F12,0x2008,
	0x602A,0x09C4,
	0x6F12,0x2000,
	0x602A,0x19FE,
	0x6F12,0x0E1C,
	0x602A,0x4D92,
	0x6F12,0x0100,
	0x602A,0x84C8,
	0x6F12,0x0100,
	0x602A,0x4D94,
	0x6F12,0x0005,
	0x6F12,0x000A,
	0x6F12,0x0010,
	0x6F12,0x0810,
	0x6F12,0x000A,
	0x6F12,0x0040,
	0x6F12,0x0810,
	0x6F12,0x0810,
	0x6F12,0x8002,
	0x6F12,0xFD03,
	0x6F12,0x0010,
	0x6F12,0x1510,
	0x602A,0x3570,
	0x6F12,0x0000,
	0x602A,0x3574,
	0x6F12,0x1201,
	0x602A,0x21E4,
	0x6F12,0x0400,
	0x602A,0x21EC,
	0x6F12,0x1F04,
	0x602A,0x2080,
	0x6F12,0x0101,
	0x6F12,0xFF00,
	0x6F12,0x7F01,
	0x6F12,0x0001,
	0x6F12,0x8001,
	0x6F12,0xD244,
	0x6F12,0xD244,
	0x6F12,0x14F4,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x602A,0x20BA,
	0x6F12,0x141C,
	0x6F12,0x111C,
	0x6F12,0x54F4,
	0x602A,0x120E,
	0x6F12,0x1000,
	0x602A,0x212E,
	0x6F12,0x0200,
	0x602A,0x13AE,
	0x6F12,0x0101,
	0x602A,0x0718,
	0x6F12,0x0001,
	0x602A,0x0710,
	0x6F12,0x0002,
	0x6F12,0x0804,
	0x6F12,0x0100,
	0x602A,0x1B5C,
	0x6F12,0x0000,
	0x602A,0x0786,
	0x6F12,0x7701,
	0x602A,0x2022,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x602A,0x1360,
	0x6F12,0x0100,
	0x602A,0x1376,
	0x6F12,0x0100,
	0x6F12,0x6038,
	0x6F12,0x7038,
	0x6F12,0x8038,
	0x602A,0x1386,
	0x6F12,0x0B00,
	0x602A,0x06FA,
	0x6F12,0x0000,
	0x602A,0x4A94,
	0x6F12,0x0900,
	0x6F12,0x0000,
	0x6F12,0x0300,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0300,
	0x6F12,0x0000,
	0x6F12,0x0900,
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
	0x6F12,0x0100,
	0x602A,0x0CF2,
	0x6F12,0x0001,
	0x602A,0x0CF0,
	0x6F12,0x0101,
	0x602A,0x11B8,
	0x6F12,0x0100,
	0x602A,0x11F6,
	0x6F12,0x0020,
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
	0x0344,0x0000,
	0x0346,0x0308,
	0x0348,0x1FFF,
	0x034A,0x1517,
	0x034C,0x0FF0,
	0x034E,0x08F8,
	0x0350,0x0008,
	0x0352,0x0008,
	0x0900,0x0122,
	0x0380,0x0002,
	0x0382,0x0002,
	0x0384,0x0002,
	0x0386,0x0002,
	0x0110,0x1002,
	0x0114,0x0301,
	0x0116,0x3000,
	0x0136,0x1800,
	0x013E,0x0000,
	0x0300,0x0006,
	0x0302,0x0001,
	0x0304,0x0004,
	0x0306,0x008C,
	0x0308,0x0008,
	0x030A,0x0001,
	0x030C,0x0000,
	0x030E,0x0003,
	0x0310,0x005C,
	0x0312,0x0000,
	0x080E,0x0000,
	0x0340,0x0C60,
	0x0342,0x1700,
	0x0702,0x0000,
	0x0202,0x0100,
	0x0200,0x0100,
	0x0D00,0x0101,
	0x0D02,0x0101,
	0x0D04,0x0102,
	0x011E,0x0000,
	0x6226,0x0000,
};

static kal_uint16 limu_shinetech_s5kjn103_hs_video_setting[] = {
	0x6028, 0x4000,
	0x6214, 0x7971,
	0x6218, 0x7150,
	0x0344, 0x0058,
	0x0346, 0x01AC,
	0x0348, 0x0F57,
	0x034A, 0x0A1B,
	0x034C, 0x0780,
	0x034E, 0x0438,
	0x0350, 0x0000,
	0x0352, 0x0000,
	0x0340, 0x063F,
	0x0342, 0x09D0,
	0x0900, 0x0122,
	0x0380, 0x0001,
	0x0382, 0x0003,
	0x0384, 0x0001,
	0x0386, 0x0003,
	0x0404, 0x1000,
	0x0402, 0x1010,
	0x0136, 0x1800,
	0x0304, 0x0006,
	0x030C, 0x0000,
	0x0306, 0x00F1,
	0x0302, 0x0001,
	0x0300, 0x0008,
	0x030E, 0x0003,
	0x0312, 0x0001,
	0x0310, 0x0095,
	0x6028, 0x2000,
	0x602A, 0x1492,
	0x6F12, 0x0078,
	0x602A, 0x0E4E,
	0x6F12, 0x0060,
	0x6028, 0x4000,
	0x0118, 0x0004,
	0x021E, 0x0000,
	0x6028, 0x2000,
	0x602A, 0x2126,
	0x6F12, 0x0000,
	0x602A, 0x1168,
	0x6F12, 0x0020,
	0x602A, 0x2DB6,
	0x6F12, 0x0001,
	0x602A, 0x1668,
	0x6F12, 0xFF00,
	0x602A, 0x166A,
	0x6F12, 0xFF00,
	0x602A, 0x118A,
	0x6F12, 0x0402,
	0x602A, 0x151E,
	0x6F12, 0x0002,
	0x602A, 0x217E,
	0x6F12, 0x0001,
	0x602A, 0x1520,
	0x6F12, 0x0000,
	0x602A, 0x2522,
	0x6F12, 0x1004,
	0x602A, 0x2524,
	0x6F12, 0x0200,
	0x602A, 0x2568,
	0x6F12, 0x0000,
	0x602A, 0x2588,
	0x6F12, 0x0000,
	0x602A, 0x258C,
	0x6F12, 0x0000,
	0x602A, 0x25A6,
	0x6F12, 0x0000,
	0x602A, 0x252C,
	0x6F12, 0x0601,
	0x602A, 0x252E,
	0x6F12, 0x0605,
	0x602A, 0x25A8,
	0x6F12, 0x1100,
	0x602A, 0x25AC,
	0x6F12, 0x0011,
	0x602A, 0x25B0,
	0x6F12, 0x1100,
	0x602A, 0x25B4,
	0x6F12, 0x0011,
	0x602A, 0x15A4,
	0x6F12, 0x0641,
	0x602A, 0x15A6,
	0x6F12, 0x0145,
	0x602A, 0x15A8,
	0x6F12, 0x0149,
	0x602A, 0x15AA,
	0x6F12, 0x064D,
	0x602A, 0x15AC,
	0x6F12, 0x0651,
	0x602A, 0x15AE,
	0x6F12, 0x0155,
	0x602A, 0x15B0,
	0x6F12, 0x0159,
	0x602A, 0x15B2,
	0x6F12, 0x065D,
	0x602A, 0x15B4,
	0x6F12, 0x0661,
	0x602A, 0x15B6,
	0x6F12, 0x0165,
	0x602A, 0x15B8,
	0x6F12, 0x0169,
	0x602A, 0x15BA,
	0x6F12, 0x066D,
	0x602A, 0x15BC,
	0x6F12, 0x0671,
	0x602A, 0x15BE,
	0x6F12, 0x0175,
	0x602A, 0x15C0,
	0x6F12, 0x0179,
	0x602A, 0x15C2,
	0x6F12, 0x067D,
	0x602A, 0x15C4,
	0x6F12, 0x0641,
	0x602A, 0x15C6,
	0x6F12, 0x0145,
	0x602A, 0x15C8,
	0x6F12, 0x0149,
	0x602A, 0x15CA,
	0x6F12, 0x064D,
	0x602A, 0x15CC,
	0x6F12, 0x0651,
	0x602A, 0x15CE,
	0x6F12, 0x0155,
	0x602A, 0x15D0,
	0x6F12, 0x0159,
	0x602A, 0x15D2,
	0x6F12, 0x065D,
	0x602A, 0x15D4,
	0x6F12, 0x0661,
	0x602A, 0x15D6,
	0x6F12, 0x0165,
	0x602A, 0x15D8,
	0x6F12, 0x0169,
	0x602A, 0x15DA,
	0x6F12, 0x066D,
	0x602A, 0x15DC,
	0x6F12, 0x0671,
	0x602A, 0x15DE,
	0x6F12, 0x0175,
	0x602A, 0x15E0,
	0x6F12, 0x0179,
	0x602A, 0x15E2,
	0x6F12, 0x067D,
	0x602A, 0x1A50,
	0x6F12, 0x0001,
	0x602A, 0x1A54,
	0x6F12, 0x0100,
	0x6028, 0x4000,
	0x0D00, 0x0101,
	0x0D02, 0x0001,
	0x0114, 0x0300,
	0xF486, 0x0000,
	0xF488, 0x0000,
	0xF48A, 0x0000,
	0xF48C, 0x0000,
	0xF48E, 0x0000,
	0xF490, 0x0000,
	0xF492, 0x0000,
	0xF494, 0x0000,
	0xF496, 0x0000,
	0xF498, 0x0000,
	0xF49A, 0x0000,
	0xF49C, 0x0000,
	0xF49E, 0x0000,
	0xF4A0, 0x0000,
	0xF4A2, 0x0000,
	0xF4A4, 0x0000,
	0xF4A6, 0x0000,
	0xF4A8, 0x0000,
	0xF4AA, 0x0000,
	0xF4AC, 0x0000,
	0xF4AE, 0x0000,
	0xF4B0, 0x0000,
	0xF4B2, 0x0000,
	0xF4B4, 0x0000,
	0xF4B6, 0x0000,
	0xF4B8, 0x0000,
	0xF4BA, 0x0000,
	0xF4BC, 0x0000,
	0xF4BE, 0x0000,
	0xF4C0, 0x0000,
	0xF4C2, 0x0000,
	0xF4C4, 0x0000,
	0x0202, 0x0010,
	0x0226, 0x0010,
	0x0204, 0x0020,
	0x0B06, 0x0101,
	0x6028, 0x2000,
	0x602A, 0x107A,
	0x6F12, 0x1D00,
	0x602A, 0x1074,
	0x6F12, 0x1D00,
	0x602A, 0x0E7C,
	0x6F12, 0x0000,
	0x602A, 0x1120,
	0x6F12, 0x0200,
	0x602A, 0x1122,
	0x6F12, 0x0078,
	0x602A, 0x1128,
	0x6F12, 0x0604,
	0x602A, 0x1AC0,
	0x6F12, 0x0200,
	0x602A, 0x1AC2,
	0x6F12, 0x0002,
	0x602A, 0x1494,
	0x6F12, 0x3D68,
	0x602A, 0x1498,
	0x6F12, 0xF10D,
	0x602A, 0x1488,
	0x6F12, 0x0F04,
	0x602A, 0x148A,
	0x6F12, 0x170B,
	0x602A, 0x150E,
	0x6F12, 0x40C2,
	0x602A, 0x1510,
	0x6F12, 0x80AF,
	0x602A, 0x1512,
	0x6F12, 0x00A0,
	0x602A, 0x1486,
	0x6F12, 0x1430,
	0x602A, 0x1490,
	0x6F12, 0x5009,
	0x602A, 0x149E,
	0x6F12, 0x01C4,
	0x602A, 0x11CC,
	0x6F12, 0x0008,
	0x602A, 0x11CE,
	0x6F12, 0x000B,
	0x602A, 0x11D0,
	0x6F12, 0x0006,
	0x602A, 0x11DA,
	0x6F12, 0x0012,
	0x602A, 0x11E6,
	0x6F12, 0x002A,
	0x602A, 0x125E,
	0x6F12, 0x0048,
	0x602A, 0x11F4,
	0x6F12, 0x0000,
	0x602A, 0x11F8,
	0x6F12, 0x0016,
	0x6028, 0x4000,
	0xF444, 0x05BF,
	0xF44A, 0x0016,
	0xF44C, 0x1414,
	0xF44E, 0x0014,
	0xF458, 0x0008,
	0xF46E, 0xD040,
	0xF470, 0x0008,
	0x6028, 0x2000,
	0x602A, 0x1CAA,
	0x6F12, 0x0000,
	0x602A, 0x1CAC,
	0x6F12, 0x0000,
	0x602A, 0x1CAE,
	0x6F12, 0x0000,
	0x602A, 0x1CB0,
	0x6F12, 0x0000,
	0x602A, 0x1CB2,
	0x6F12, 0x0000,
	0x602A, 0x1CB4,
	0x6F12, 0x0000,
	0x602A, 0x1CB6,
	0x6F12, 0x0000,
	0x602A, 0x1CB8,
	0x6F12, 0x0000,
	0x602A, 0x1CBA,
	0x6F12, 0x0000,
	0x602A, 0x1CBC,
	0x6F12, 0x0000,
	0x602A, 0x1CBE,
	0x6F12, 0x0000,
	0x602A, 0x1CC0,
	0x6F12, 0x0000,
	0x602A, 0x1CC2,
	0x6F12, 0x0000,
	0x602A, 0x1CC4,
	0x6F12, 0x0000,
	0x602A, 0x1CC6,
	0x6F12, 0x0000,
	0x602A, 0x1CC8,
	0x6F12, 0x0000,
	0x602A, 0x6000,
	0x6F12, 0x000F,
	0x602A, 0x6002,
	0x6F12, 0xFFFF,
	0x602A, 0x6004,
	0x6F12, 0x0000,
	0x602A, 0x6006,
	0x6F12, 0x1000,
	0x602A, 0x6008,
	0x6F12, 0x1000,
	0x602A, 0x600A,
	0x6F12, 0x1000,
	0x602A, 0x600C,
	0x6F12, 0x1000,
	0x602A, 0x600E,
	0x6F12, 0x1000,
	0x602A, 0x6010,
	0x6F12, 0x1000,
	0x602A, 0x6012,
	0x6F12, 0x1000,
	0x602A, 0x6014,
	0x6F12, 0x1000,
	0x602A, 0x6016,
	0x6F12, 0x1000,
	0x602A, 0x6018,
	0x6F12, 0x1000,
	0x602A, 0x601A,
	0x6F12, 0x1000,
	0x602A, 0x601C,
	0x6F12, 0x1000,
	0x602A, 0x601E,
	0x6F12, 0x1000,
	0x602A, 0x6020,
	0x6F12, 0x1000,
	0x602A, 0x6022,
	0x6F12, 0x1000,
	0x602A, 0x6024,
	0x6F12, 0x1000,
	0x602A, 0x6026,
	0x6F12, 0x1000,
	0x602A, 0x6028,
	0x6F12, 0x1000,
	0x602A, 0x602A,
	0x6F12, 0x1000,
	0x602A, 0x602C,
	0x6F12, 0x1000,
	0x602A, 0x1144,
	0x6F12, 0x0100,
	0x602A, 0x1146,
	0x6F12, 0x1B00,
	0x602A, 0x1080,
	0x6F12, 0x0100,
	0x602A, 0x1084,
	0x6F12, 0x00C0,
	0x602A, 0x108A,
	0x6F12, 0x00C0,
	0x602A, 0x1090,
	0x6F12, 0x0001,
	0x602A, 0x1092,
	0x6F12, 0x0000,
	0x602A, 0x1094,
	0x6F12, 0xA32E,
	0x602A, 0x602E,
	0x6F12, 0x0000,
	0x602A, 0x6038,
	0x6F12, 0x0003,
	0x602A, 0x603A,
	0x6F12, 0x005F,
	0x602A, 0x603C,
	0x6F12, 0x0060,
	0x602A, 0x603E,
	0x6F12, 0x0061,
	0x602A, 0x25D4,
	0x6F12, 0x0000,
	0x602A, 0x25D6,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0xF45A, 0x001a,
};

static kal_uint16 limu_shinetech_s5kjn103_slim_video_setting[] = {
	0x6028,0x2400,
	0x602A,0x1A28,
	0x6F12,0x4C00,
	0x602A,0x065A,
	0x6F12,0x0000,
	0x602A,0x139E,
	0x6F12,0x0100,
	0x602A,0x139C,
	0x6F12,0x0000,
	0x602A,0x13A0,
	0x6F12,0x0A00,
	0x6F12,0x0120,
	0x602A,0x2072,
	0x6F12,0x0000,
	0x602A,0x1A64,
	0x6F12,0x0301,
	0x6F12,0xFF00,
	0x602A,0x19E6,
	0x6F12,0x0200,
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
	0x6F12,0xA060,
	0x602A,0x1A3C,
	0x6F12,0x6207,
	0x602A,0x1A48,
	0x6F12,0x6207,
	0x602A,0x1444,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x602A,0x144C,
	0x6F12,0x3F00,
	0x6F12,0x3F00,
	0x602A,0x7F6C,
	0x6F12,0x0100,
	0x6F12,0x2F00,
	0x6F12,0xFA00,
	0x6F12,0x2400,
	0x6F12,0xE500,
	0x602A,0x0650,
	0x6F12,0x0600,
	0x602A,0x0654,
	0x6F12,0x0000,
	0x602A,0x1A46,
	0x6F12,0x8A00,
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
	0x6F12,0x1800,
	0x602A,0x2148,
	0x6F12,0x0100,
	0x602A,0x2042,
	0x6F12,0x1A00,
	0x602A,0x0874,
	0x6F12,0x0100,
	0x602A,0x09C0,
	0x6F12,0x2008,
	0x602A,0x09C4,
	0x6F12,0x2000,
	0x602A,0x19FE,
	0x6F12,0x0E1C,
	0x602A,0x4D92,
	0x6F12,0x0100,
	0x602A,0x84C8,
	0x6F12,0x0100,
	0x602A,0x4D94,
	0x6F12,0x0005,
	0x6F12,0x000A,
	0x6F12,0x0010,
	0x6F12,0x0810,
	0x6F12,0x000A,
	0x6F12,0x0040,
	0x6F12,0x0810,
	0x6F12,0x0810,
	0x6F12,0x8002,
	0x6F12,0xFD03,
	0x6F12,0x0010,
	0x6F12,0x1510,
	0x602A,0x3570,
	0x6F12,0x0000,
	0x602A,0x3574,
	0x6F12,0x1201,
	0x602A,0x21E4,
	0x6F12,0x0400,
	0x602A,0x21EC,
	0x6F12,0x1F04,
	0x602A,0x2080,
	0x6F12,0x0101,
	0x6F12,0xFF00,
	0x6F12,0x7F01,
	0x6F12,0x0001,
	0x6F12,0x8001,
	0x6F12,0xD244,
	0x6F12,0xD244,
	0x6F12,0x14F4,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x602A,0x20BA,
	0x6F12,0x141C,
	0x6F12,0x111C,
	0x6F12,0x54F4,
	0x602A,0x120E,
	0x6F12,0x1000,
	0x602A,0x212E,
	0x6F12,0x0200,
	0x602A,0x13AE,
	0x6F12,0x0101,
	0x602A,0x0718,
	0x6F12,0x0001,
	0x602A,0x0710,
	0x6F12,0x0002,
	0x6F12,0x0804,
	0x6F12,0x0100,
	0x602A,0x1B5C,
	0x6F12,0x0000,
	0x602A,0x0786,
	0x6F12,0x7701,
	0x602A,0x2022,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x602A,0x1360,
	0x6F12,0x0100,
	0x602A,0x1376,
	0x6F12,0x0100,
	0x6F12,0x6038,
	0x6F12,0x7038,
	0x6F12,0x8038,
	0x602A,0x1386,
	0x6F12,0x0B00,
	0x602A,0x06FA,
	0x6F12,0x0000,
	0x602A,0x4A94,
	0x6F12,0x0900,
	0x6F12,0x0000,
	0x6F12,0x0300,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0300,
	0x6F12,0x0000,
	0x6F12,0x0900,
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
	0x6F12,0x0100,
	0x602A,0x0CF2,
	0x6F12,0x0001,
	0x602A,0x0CF0,
	0x6F12,0x0101,
	0x602A,0x11B8,
	0x6F12,0x0100,
	0x602A,0x11F6,
	0x6F12,0x0020,
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
	0x0344,0x0000,
	0x0346,0x0308,
	0x0348,0x1FFF,
	0x034A,0x1517,
	0x034C,0x0FF0,
	0x034E,0x08F8,
	0x0350,0x0008,
	0x0352,0x0008,
	0x0900,0x0122,
	0x0380,0x0002,
	0x0382,0x0002,
	0x0384,0x0002,
	0x0386,0x0002,
	0x0110,0x1002,
	0x0114,0x0301,
	0x0116,0x3000,
	0x0136,0x1800,
	0x013E,0x0000,
	0x0300,0x0006,
	0x0302,0x0001,
	0x0304,0x0004,
	0x0306,0x008C,
	0x0308,0x0008,
	0x030A,0x0001,
	0x030C,0x0000,
	0x030E,0x0003,
	0x0310,0x005C,
	0x0312,0x0000,
	0x080E,0x0000,
	0x0340,0x0C60,
	0x0342,0x1700,
	0x0702,0x0000,
	0x0202,0x0100,
	0x0200,0x0100,
	0x0D00,0x0101,
	0x0D02,0x0101,
	0x0D04,0x0102,
	0x011E,0x0000,
	0x6226,0x0000,
};

static kal_uint16 limu_shinetech_s5kjn103_custom1_setting[] = {
	0x6028,0x2400,
	0x602A,0x1A28,
	0x6F12,0x4C00,
	0x602A,0x065A,
	0x6F12,0x0000,
	0x602A,0x139E,
	0x6F12,0x0100,
	0x602A,0x139C,
	0x6F12,0x0000,
	0x602A,0x13A0,
	0x6F12,0x0A00,
	0x6F12,0x0120,
	0x602A,0x2072,
	0x6F12,0x0000,
	0x602A,0x1A64,
	0x6F12,0x0301,
	0x6F12,0xFF00,
	0x602A,0x19E6,
	0x6F12,0x0200,
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
	0x6F12,0xA060,
	0x602A,0x1A3C,
	0x6F12,0x6207,
	0x602A,0x1A48,
	0x6F12,0x6207,
	0x602A,0x1444,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x602A,0x144C,
	0x6F12,0x3F00,
	0x6F12,0x3F00,
	0x602A,0x7F6C,
	0x6F12,0x0100,
	0x6F12,0x2F00,
	0x6F12,0xFA00,
	0x6F12,0x2400,
	0x6F12,0xE500,
	0x602A,0x0650,
	0x6F12,0x0600,
	0x602A,0x0654,
	0x6F12,0x0000,
	0x602A,0x1A46,
	0x6F12,0x8A00,
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
	0x6F12,0x1800,
	0x602A,0x2148,
	0x6F12,0x0100,
	0x602A,0x2042,
	0x6F12,0x1A00,
	0x602A,0x0874,
	0x6F12,0x0100,
	0x602A,0x09C0,
	0x6F12,0x2008,
	0x602A,0x09C4,
	0x6F12,0x9800,
	0x602A,0x19FE,
	0x6F12,0x0E1C,
	0x602A,0x4D92,
	0x6F12,0x0100,
	0x602A,0x84C8,
	0x6F12,0x0100,
	0x602A,0x4D94,
	0x6F12,0x0005,
	0x6F12,0x000A,
	0x6F12,0x0010,
	0x6F12,0x0810,
	0x6F12,0x000A,
	0x6F12,0x0040,
	0x6F12,0x0810,
	0x6F12,0x0810,
	0x6F12,0x8002,
	0x6F12,0xFD03,
	0x6F12,0x0010,
	0x6F12,0x1510,
	0x602A,0x3570,
	0x6F12,0x0000,
	0x602A,0x3574,
	0x6F12,0x6100,
	0x602A,0x21E4,
	0x6F12,0x0400,
	0x602A,0x21EC,
	0x6F12,0xEF03,
	0x602A,0x2080,
	0x6F12,0x0101,
	0x6F12,0xFF00,
	0x6F12,0x7F01,
	0x6F12,0x0001,
	0x6F12,0x8001,
	0x6F12,0xD244,
	0x6F12,0xD244,
	0x6F12,0x14F4,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x602A,0x20BA,
	0x6F12,0x141C,
	0x6F12,0x111C,
	0x6F12,0x54F4,
	0x602A,0x120E,
	0x6F12,0x1000,
	0x602A,0x212E,
	0x6F12,0x0200,
	0x602A,0x13AE,
	0x6F12,0x0101,
	0x602A,0x0718,
	0x6F12,0x0001,
	0x602A,0x0710,
	0x6F12,0x0002,
	0x6F12,0x0804,
	0x6F12,0x0100,
	0x602A,0x1B5C,
	0x6F12,0x0000,
	0x602A,0x0786,
	0x6F12,0x7701,
	0x602A,0x2022,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x602A,0x1360,
	0x6F12,0x0100,
	0x602A,0x1376,
	0x6F12,0x0100,
	0x6F12,0x6038,
	0x6F12,0x7038,
	0x6F12,0x8038,
	0x602A,0x1386,
	0x6F12,0x0B00,
	0x602A,0x06FA,
	0x6F12,0x1000,
	0x602A,0x4A94,
	0x6F12,0x0900,
	0x6F12,0x0000,
	0x6F12,0x0300,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0300,
	0x6F12,0x0000,
	0x6F12,0x0900,
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
	0x6F12,0x0100,
	0x602A,0x0CF2,
	0x6F12,0x0001,
	0x602A,0x0CF0,
	0x6F12,0x0101,
	0x602A,0x11B8,
	0x6F12,0x0100,
	0x602A,0x11F6,
	0x6F12,0x0020,
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
	0x0344,0x0330,
	0x0346,0x0270,
	0x0348,0x1CCF,
	0x034A,0x15AF,
	0x034C,0x0CC0,
	0x034E,0x0990,
	0x0350,0x0008,
	0x0352,0x0008,
	0x0900,0x0122,
	0x0380,0x0002,
	0x0382,0x0002,
	0x0384,0x0002,
	0x0386,0x0002,
	0x0110,0x1002,
	0x0114,0x0301,
	0x0116,0x3000,
	0x0136,0x1800,
	0x013E,0x0000,
	0x0300,0x0006,
	0x0302,0x0001,
	0x0304,0x0004,
	0x0306,0x008C,
	0x0308,0x0008,
	0x030A,0x0001,
	0x030C,0x0000,
	0x030E,0x0003,
	0x0310,0x0087,
	0x0312,0x0001,
	0x080E,0x0000,
	0x0340,0x0B00,
	0x0342,0x19E0,
	0x0702,0x0000,
	0x0202,0x0100,
	0x0200,0x0100,
	0x0D00,0x0101,
	0x0D02,0x0101,
	0x0D04,0x0102,
	0x011E,0x0000,
	0x6226,0x0000,
};

static kal_uint16 limu_shinetech_s5kjn103_custom2_setting[] = {
	0x6028,0x2400,
	0x602A,0x1A28,
	0x6F12,0x4C00,
	0x602A,0x065A,
	0x6F12,0x0000,
	0x602A,0x139E,
	0x6F12,0x0100,
	0x602A,0x139C,
	0x6F12,0x0000,
	0x602A,0x13A0,
	0x6F12,0x0A00,
	0x6F12,0x0120,
	0x602A,0x2072,
	0x6F12,0x0000,
	0x602A,0x1A64,
	0x6F12,0x0301,
	0x6F12,0xFF00,
	0x602A,0x19E6,
	0x6F12,0x0200,
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
	0x6F12,0xA060,
	0x602A,0x1A3C,
	0x6F12,0x6207,
	0x602A,0x1A48,
	0x6F12,0x6207,
	0x602A,0x1444,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x602A,0x144C,
	0x6F12,0x3F00,
	0x6F12,0x3F00,
	0x602A,0x7F6C,
	0x6F12,0x0100,
	0x6F12,0x2F00,
	0x6F12,0xFA00,
	0x6F12,0x2400,
	0x6F12,0xE500,
	0x602A,0x0650,
	0x6F12,0x0600,
	0x602A,0x0654,
	0x6F12,0x0000,
	0x602A,0x1A46,
	0x6F12,0x8A00,
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
	0x6F12,0x1800,
	0x602A,0x2148,
	0x6F12,0x0100,
	0x602A,0x2042,
	0x6F12,0x1A00,
	0x602A,0x0874,
	0x6F12,0x0100,
	0x602A,0x09C0,
	0x6F12,0x2008,
	0x602A,0x09C4,
	0x6F12,0x9800,
	0x602A,0x19FE,
	0x6F12,0x0E1C,
	0x602A,0x4D92,
	0x6F12,0x0100,
	0x602A,0x84C8,
	0x6F12,0x0100,
	0x602A,0x4D94,
	0x6F12,0x0005,
	0x6F12,0x000A,
	0x6F12,0x0010,
	0x6F12,0x0810,
	0x6F12,0x000A,
	0x6F12,0x0040,
	0x6F12,0x0810,
	0x6F12,0x0810,
	0x6F12,0x8002,
	0x6F12,0xFD03,
	0x6F12,0x0010,
	0x6F12,0x1510,
	0x602A,0x3570,
	0x6F12,0x0000,
	0x602A,0x3574,
	0x6F12,0x6100,
	0x602A,0x21E4,
	0x6F12,0x0400,
	0x602A,0x21EC,
	0x6F12,0xEF03,
	0x602A,0x2080,
	0x6F12,0x0101,
	0x6F12,0xFF00,
	0x6F12,0x7F01,
	0x6F12,0x0001,
	0x6F12,0x8001,
	0x6F12,0xD244,
	0x6F12,0xD244,
	0x6F12,0x14F4,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x602A,0x20BA,
	0x6F12,0x141C,
	0x6F12,0x111C,
	0x6F12,0x54F4,
	0x602A,0x120E,
	0x6F12,0x1000,
	0x602A,0x212E,
	0x6F12,0x0200,
	0x602A,0x13AE,
	0x6F12,0x0101,
	0x602A,0x0718,
	0x6F12,0x0001,
	0x602A,0x0710,
	0x6F12,0x0002,
	0x6F12,0x0804,
	0x6F12,0x0100,
	0x602A,0x1B5C,
	0x6F12,0x0000,
	0x602A,0x0786,
	0x6F12,0x7701,
	0x602A,0x2022,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x602A,0x1360,
	0x6F12,0x0100,
	0x602A,0x1376,
	0x6F12,0x0100,
	0x6F12,0x6038,
	0x6F12,0x7038,
	0x6F12,0x8038,
	0x602A,0x1386,
	0x6F12,0x0B00,
	0x602A,0x06FA,
	0x6F12,0x1000,
	0x602A,0x4A94,
	0x6F12,0x0900,
	0x6F12,0x0000,
	0x6F12,0x0300,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0300,
	0x6F12,0x0000,
	0x6F12,0x0900,
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
	0x6F12,0x0100,
	0x602A,0x0CF2,
	0x6F12,0x0001,
	0x602A,0x0CF0,
	0x6F12,0x0101,
	0x602A,0x11B8,
	0x6F12,0x0100,
	0x602A,0x11F6,
	0x6F12,0x0020,
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
	0x0344,0x0330,
	0x0346,0x0270,
	0x0348,0x1CCF,
	0x034A,0x15AF,
	0x034C,0x0CC0,
	0x034E,0x0990,
	0x0350,0x0008,
	0x0352,0x0008,
	0x0900,0x0122,
	0x0380,0x0002,
	0x0382,0x0002,
	0x0384,0x0002,
	0x0386,0x0002,
	0x0110,0x1002,
	0x0114,0x0301,
	0x0116,0x3000,
	0x0136,0x1800,
	0x013E,0x0000,
	0x0300,0x0006,
	0x0302,0x0001,
	0x0304,0x0004,
	0x0306,0x008C,
	0x0308,0x0008,
	0x030A,0x0001,
	0x030C,0x0000,
	0x030E,0x0003,
	0x0310,0x0087,
	0x0312,0x0001,
	0x080E,0x0000,
	0x0340,0x0DC0,
	0x0342,0x19E0,
	0x0702,0x0000,
	0x0202,0x0100,
	0x0200,0x0100,
	0x0D00,0x0101,
	0x0D02,0x0101,
	0x0D04,0x0102,
	0x011E,0x0000,
	0x6226,0x0000,
};

static void sensor_init(void)
{
	LOG_INF("sensor_init start\n");
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0000, 0x0002);
	write_cmos_sensor(0x0000, 0x38E1);
	write_cmos_sensor(0x001E, 0x0007);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6010, 0x0001);
	mdelay(5);  //delay 5ms
	write_cmos_sensor(0x6226, 0x0001);
	mdelay(10);  //delay 10ms
	table_write_cmos_sensor(limu_shinetech_s5kjn103_init_setting,
		sizeof(limu_shinetech_s5kjn103_init_setting) / sizeof(kal_uint16));
	//set_mirror_flip(imgsensor.mirror);
	LOG_INF("sensor_init End\n");
}	/*	  sensor_init  */

static void preview_setting(void)
{
	LOG_INF("preview_setting Start\n");
	table_write_cmos_sensor(limu_shinetech_s5kjn103_preview_setting,
		sizeof(limu_shinetech_s5kjn103_preview_setting)/sizeof(kal_uint16));
	LOG_INF("preview_setting End\n");
} /* preview_setting */


/*full size 30fps*/
static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("%s 30 fps E! currefps:%d\n", __func__, currefps);
	table_write_cmos_sensor(limu_shinetech_s5kjn103_capture_setting,
		sizeof(limu_shinetech_s5kjn103_capture_setting)/sizeof(kal_uint16));
	LOG_INF("%s 30 fpsX\n", __func__);
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("%s E! currefps:%d\n", __func__, currefps);
	table_write_cmos_sensor(limu_shinetech_s5kjn103_normal_video_setting,
		sizeof(limu_shinetech_s5kjn103_normal_video_setting)/sizeof(kal_uint16));
	LOG_INF("X\n");
}

static void hs_video_setting(void)
{
	LOG_INF("%s E! currefps 120\n", __func__);
	table_write_cmos_sensor(limu_shinetech_s5kjn103_hs_video_setting,
		sizeof(limu_shinetech_s5kjn103_hs_video_setting)/sizeof(kal_uint16));
	LOG_INF("X\n");
}

static void slim_video_setting(void)
{
	LOG_INF("%s E! 4608*2592@30fps\n", __func__);
	table_write_cmos_sensor(limu_shinetech_s5kjn103_slim_video_setting,
		sizeof(limu_shinetech_s5kjn103_slim_video_setting)/sizeof(kal_uint16));
	LOG_INF("X\n");
}

/*full size 10fps*/

static void custom1_setting(void)
{
	LOG_INF("%s CUS1_16M_24_FPS E! currefps\n", __func__);
	/*************MIPI output setting************/
	table_write_cmos_sensor(limu_shinetech_s5kjn103_custom1_setting,
		sizeof(limu_shinetech_s5kjn103_custom1_setting)/sizeof(kal_uint16));
	LOG_INF("X");
}

static void custom2_setting(void)
{
	LOG_INF("%s CUS1_16M_24_FPS E! currefps\n", __func__);
	/*************MIPI output setting************/
	table_write_cmos_sensor(limu_shinetech_s5kjn103_custom2_setting,
		sizeof(limu_shinetech_s5kjn103_custom2_setting)/sizeof(kal_uint16));
	LOG_INF("X");
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
	/*sensor have two i2c address 0x34 & 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = (((read_cmos_sensor_8(0x0000) << 8)
					| read_cmos_sensor_8(0x0001)) + 4);
			LOG_INF(
				"read_0x0000=0x%x, 0x0001=0x%x,0x0000_0001=0x%x *sensor_id:0x%x\n",
				read_cmos_sensor_8(0x0000),
				read_cmos_sensor_8(0x0001),
				read_cmos_sensor(0x0000), *sensor_id);
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				imgsensor_info.module_id = read_module_id();
				LOG_INF("module_id=%d\n",imgsensor_info.module_id);
				if(deviceInfo_register_value == 0x00){
					Eeprom_DataInit(0, LIMU_SHINETECH_MAIN_S5KJN103_SENSOR_ID);
					deviceInfo_register_value = 0x01;
				}
				//read_eepromData();
				read_eepromData_HW_GGC();
				return ERROR_NONE;
			}

			LOG_INF("Read sensor id fail, id: 0x%x *sensor_id:0x%x\n", imgsensor.i2c_write_id, *sensor_id);
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

	LOG_INF("open start\n");
	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = (((read_cmos_sensor_8(0x0000) << 8)
					| read_cmos_sensor_8(0x0001)) + 4);
			LOG_INF("Read sensor id [0x0000~1]: 0x%x, %x sensor_id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, sensor_id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
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
	write_sensor_HW_GGC();
	if (imgsensor.test_pattern) {
		set_test_pattern_mode(imgsensor.test_pattern);
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
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("open End\n");
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
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable)
		write_cmos_sensor(0x0600, 0x0001); /*black mode*/
	else
		write_cmos_sensor(0x0600, 0x0000); /*No pattern*/

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
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
	UINT8 feature_data_8[17] = {0};
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
		= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*LOG_INF("feature_id = %d\n", feature_id);*/
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
	#if 0
	case SENSOR_FEATURE_GET_EEPROM_DATA:
		LOG_INF("SENSOR_FEATURE_GET_EEPROM_DATA:%d\n", *feature_para_len);
		memcpy(&feature_para[0], CAM_DUAL_DATA, DUALCAM_CALI_DATA_LENGTH_8ALIGN);
		break;
	case SENSOR_FEATURE_GET_MODULE_INFO:
		LOG_INF("GET_MODULE_CamInfo:%d %d\n", *feature_para_len, *feature_data_32);
		*(feature_data_32 + 1) = (CAM_INFO[1] << 24)
					| (CAM_INFO[0] << 16)
					| (CAM_INFO[3] << 8)
					| (CAM_INFO[2] & 0xFF);
		*(feature_data_32 + 2) = (CAM_INFO[5] << 24)
					| (CAM_INFO[4] << 16)
					| (CAM_INFO[7] << 8)
					| (CAM_INFO[6] & 0xFF);
		break;
	case SENSOR_FEATURE_GET_MODULE_SN:
		LOG_INF("GET_MODULE_SN:%d %d\n", *feature_para_len, *feature_data_32);
		if (*feature_data_32 < CAMERA_MODULE_SN_LENGTH/4)
			*(feature_data_32 + 1) = (CAM_SN[4*(*feature_data_32) + 3] << 24)
						| (CAM_SN[4*(*feature_data_32) + 2] << 16)
						| (CAM_SN[4*(*feature_data_32) + 1] << 8)
						| (CAM_SN[4*(*feature_data_32)] & 0xFF);
		break;
	case SENSOR_FEATURE_SET_SENSOR_OTP:
	{
		kal_int32 ret = IMGSENSOR_RETURN_SUCCESS;
		LOG_INF("SENSOR_FEATURE_SET_SENSOR_OTP length :%d\n", (UINT32)*feature_para_len);
		ret = write_Module_data((ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(feature_para));
		if (ret == ERROR_NONE)
			return ERROR_NONE;
		else
			return ERROR_MSDK_IS_ACTIVED;
	}
		break;
	#endif
	//case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		//*(MUINT32 *)(uintptr_t)(*(feature_data + 0)) = 0;
		//break;
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
	case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
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
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info_3264_2448_custom1, sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info_3264_2448_custom2, sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info_2040_1536_preview, sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG: //4000*3000
			//imgsensor_pd_info_binning.i4BlockNumX = 288;
			//imgsensor_pd_info_binning.i4BlockNumY = 216;
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info, sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info_4080_2296, sizeof(struct SET_PD_BLOCK_INFO_T));
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
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
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
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT32) (*feature_data),
					(UINT32) (*(feature_data + 1)),
					(UINT16) (*(feature_data + 2)));
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
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
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
	#if 0
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case MSDK_SCENARIO_ID_CUSTOM3:
			*feature_return_para_32 = 1; /*BINNING_NONE*/
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
			break;
		}
		LOG_ERR("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;
		break;
	#endif
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
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
	}
		break;
	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n",
			(UINT16)*feature_data);
		pvcinfo =
			(struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[3],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		default:
			break;
		}
		break;

	case SENSOR_FEATURE_SET_LSC_TBL:
	{
		kal_uint8 index =
			*(((kal_uint8 *)feature_para) + (*feature_para_len));

		limu_shinetech_s5kjn103_set_lsc_reg_setting(index, feature_data_16,
					  (*feature_para_len)/sizeof(UINT16));
	}
		break;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	case SENSOR_FEATURE_GET_SERIANO_IC:
		read_limu_shinetech_main_s5kjn103_SNData(feature_data_8, feature_para_len);
		memcpy(feature_para,feature_data_8,*feature_para_len);
	break;
#endif
	default:
		break;
	}

	return ERROR_NONE;
}

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};


UINT32 LIMU_SHINETECH_MAIN_S5KJN103_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL) {
		*pfFunc = &sensor_func;
	}
	return ERROR_NONE;
} /* S5KJN103_MIPI_RAW_SensorInit */
