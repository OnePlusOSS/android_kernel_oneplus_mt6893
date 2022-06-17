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

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

//#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"
#include "imx471mipiraw_Sensor.h"
#include "imgsensor_eeprom.h"

#include "imgsensor_i2c.h"

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

#define PFX "IMX471_camera_sensor"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
static kal_uint32 streaming_control(kal_bool enable);

#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __func__, ##args)
#define LOG_ERR(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)


static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {

    /*record sensor id defined in Kd_imgsensor.h*/
    .sensor_id = IMX471_SENSOR_ID,

    .checksum_value = 0xb1893b4f, /*checksum value for Camera Auto Test*/
    .pre = { /* reg_M 30fps */
        .pclk = 244000000,
        .linelength = 2560,
        .framelength = 3176,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2304,
        .grabwindow_height = 1728,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 681600000,
        .max_framerate = 300,
    },
    .cap = { /* reg_M 30fps */
        .pclk = 244000000,
        .linelength = 2560,
        .framelength = 3176,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2304,
        .grabwindow_height = 1728,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 681600000,
        .max_framerate = 300,
    },
    .normal_video = {
        .pclk = 220000000,
        .linelength = 2560,
        .framelength = 2864,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2304,
        .grabwindow_height = 1296,
        .mipi_data_lp2hs_settle_dc = 120,
        .max_framerate = 300,
        .mipi_pixel_rate = 368000000,
    },
    .hs_video = { /* reg_H 120fps */
        .pclk = 580000000,
        .linelength = 2560,
        .framelength = 1888,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2304,
        .grabwindow_height = 1728,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 681600000,
        .max_framerate = 1200,
    },
    .slim_video = {
        .pclk = 220000000,
        .linelength = 2560,
        .framelength = 2864,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2304,
        .grabwindow_height = 1296,
        .mipi_data_lp2hs_settle_dc = 120,
        .max_framerate = 300,
        .mipi_pixel_rate = 368000000,
    },
    .custom1 = { /* reg_M 30fps */
        .pclk = 124000000,
        .linelength = 2560,
        .framelength = 2018,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2304,
        .grabwindow_height = 1728,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 139200000,
        .max_framerate = 240,
    },
    .custom2 = { /* reg_M 30fps */
        .pclk = 580000000,
        .linelength = 2560,
        .framelength = 7552,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2304,
        .grabwindow_height = 1728,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 681600000,
        .max_framerate = 300,
    },
    .custom3 = { /* reg_M 30fps */
        .pclk = 568000000,
        .linelength = 5120,
        .framelength = 3692,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4608,
        .grabwindow_height = 3456,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 659200000,
        .max_framerate = 300,

    },
    .custom4 = { /* reg_M-1 30fps */
        .pclk = 132000000,
        .linelength = 2560,
        .framelength = 1718,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1632,
        .grabwindow_height = 1244,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 681600000,
        .max_framerate = 300,
    },

	.margin = 18,			/*sensor framelength & shutter margin*/
	.min_shutter = 4,		/*min shutter*/
	.min_gain = 64, /*1x gain*/
	.max_gain = 1024, /*16x gain*/
	.min_gain_iso = 100,
	.gain_step = 1,
	.gain_type = 0,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2, /*isp gain delay frame for AE cycle*/
	.frame_time_delay_frame = 3,
	.ihdr_support = 0,	  /*1, support; 0,not support*/
	.ihdr_le_firstline = 0,  /*1,le first ; 0, se first*/

	/*support sensor mode num ,don't support Slow motion*/
	.sensor_mode_num = 9,
	.cap_delay_frame = 2,		/*enter capture delay frame num*/
	.pre_delay_frame = 2,		/*enter preview delay frame num*/
	.video_delay_frame = 2,		/*enter video delay frame num*/
	.hs_video_delay_frame = 2, /*enter high speed video  delay frame num*/
	.slim_video_delay_frame = 2,/*enter slim video delay frame num*/
	.custom1_delay_frame = 2,
	.custom2_delay_frame = 2,
	.custom3_delay_frame = 2,
	.custom4_delay_frame = 3,
	.isp_driving_current = ISP_DRIVING_6MA, /*mclk driving current*/

	/*Sensor_interface_type*/
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,

	/*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
	.mipi_sensor_type = MIPI_OPHY_NCSI2,

	/*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
	.mipi_settle_delay_mode = 0, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL

	/*sensor output first pixel color*/
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_B,

	.mclk = 24,/*mclk value, suggest 24 or 26 for 24Mhz or 26Mhz*/
	.mipi_lane_num = SENSOR_MIPI_4_LANE,/*mipi lane num*/

	/*record sensor support all write id addr, only supprt 4must end with 0xff*/
	.i2c_addr_table = {0x20, 0x21, 0xff},
	.i2c_speed = 1000,
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,		/*mirrorflip information*/
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,			/*current shutter*/
	.gain = 0x100,				/*current gain*/
	.dummy_pixel = 0,			/*current dummypixel*/
	.dummy_line = 0,			/*current dummyline*/

	/*full size current fps : 24fps for PIP, 30fps for Normal or ZSD*/
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = 0,

	/*current scenario id*/
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_mode = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x20, /*record current sensor's i2c write id*/
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[9] = {
    {4656, 3496, 24, 20, 4608, 3456, 2304, 1728,  0,  0, 2304, 1728, 0, 0, 2304, 1728}, /* preview */
    {4656, 3496, 24, 20, 4608, 3456, 2304, 1728,  0,  0, 2304, 1728, 0, 0, 2304, 1728}, /* capture */
    {4656, 3496, 24, 444, 4608, 2608, 2304, 1304,  0,  4,2304, 1296, 0, 0, 2304, 1296},     /* video */
    {4656, 3496, 24, 20, 4608, 3456, 2304, 1728,  0,  0, 2304, 1728, 0, 0, 2304, 1728}, /* hs-video*/
    {4656, 3496, 24, 444, 4608, 2608, 2304, 1304,  0,  4,2304, 1296, 0, 0, 2304, 1296},     /* slim video */
    {4656, 3496, 24, 20, 4608, 3456, 2304, 1728,  0,  0, 2304, 1728, 0, 0, 2304, 1728}, /*custom1*/
    {4656, 3496, 24, 20, 4608, 3456, 2304, 1728,  0,  0, 2304, 1728, 0, 0, 2304, 1728}, /*custom2*/
    {4656, 3496, 24, 20, 4608, 3456, 4608, 3456,  0,  0, 4608, 3456, 0, 0, 4608, 3456},/*custom3  remosaic*/
    {4656, 3496, 696, 504, 3264, 2488, 1632, 1244,  0,  0, 1632, 1244, 0, 0, 1632, 1244}, /*custom4*/
};

static void read_EepromQSC(void)
{
    int i = 0;
    /*Read normal eeprom data*/
    gImgEepromInfo.camNormdata[1][0] = Eeprom_1ByteDataRead(0x00, 0xA8);
    gImgEepromInfo.camNormdata[1][1] = Eeprom_1ByteDataRead(0x01, 0xA8);
    imgsensor_info.module_id = Eeprom_1ByteDataRead(0x00, 0xA8);
    Oplusimgsensor_Registdeviceinfo(gImgEepromInfo.pCamModuleInfo[1].name,
                                    gImgEepromInfo.pCamModuleInfo[1].version,
                                    imgsensor_info.module_id);
    for(i = 2; i < 8; i++) {
        gImgEepromInfo.camNormdata[1][i] = Eeprom_1ByteDataRead(0x04+i, 0xA8);
    }
    for (i = 0; i < OPLUS_CAMERASN_LENS; i ++) {
        gImgEepromInfo.camNormdata[1][8+i] = Eeprom_1ByteDataRead(0xB0+i, 0xA8);
    }
    gImgEepromInfo.i4CurSensorIdx = 1;
    gImgEepromInfo.i4CurSensorId = imgsensor_info.sensor_id;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF),
			(char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}
static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* return;*/ /* for test */
	write_cmos_sensor(0x0350, 0x00); /* Disable auto extend */
	write_cmos_sensor(0x0104, 0x01);

	write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);

	write_cmos_sensor(0x0104, 0x00);

}	/*	set_dummy  */


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
}	/*	set_max_framerate  */

static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK
	 * to get exposure larger than frame exposure
	 */
	/* AE doesn't update sensor gain at capture mode,
	 * thus extra exposure lines must be updated here.
	 */

	/* OV Recommend Solution*/
	/* if shutter bigger than frame_length, should extend frame length first*/
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

			imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
			write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x0341,imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0x00);
		}
	} else {
			imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;

			write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x0341,imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0x00);
		}
	/* Update Shutter */
	write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor(0x0350, 0x01); /* Enable auto extend */
	write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x0203, shutter  & 0xFF);
	write_cmos_sensor(0x0104, 0x00);
	LOG_INF("shutter =%d, framelength =%d\n",
		shutter, imgsensor.frame_length);


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
			write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x0340,
					imgsensor.frame_length >> 8);
			write_cmos_sensor(0x0341,
					imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0x00);
		}
	} else {
			imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;

			write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0x00);

		}

	/* Update Shutter*/
	write_cmos_sensor(0x0104, 0x01);
	if (auto_extend_en)
		write_cmos_sensor(0x0350, 0x01); /* Enable auto extend */
	else
		write_cmos_sensor(0x0350, 0x00); /* Disable auto extend */
	write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x0203, shutter  & 0xFF);
	write_cmos_sensor(0x0104, 0x00);

	LOG_INF("Add for N3D! shutterlzl =%d, framelength =%d\n", shutter, imgsensor.frame_length);

}

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

	write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor(0x0204, (reg_gain>>8) & 0xFF);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);
	write_cmos_sensor(0x0104, 0x00);

	return gain;
} /* set_gain */


static void ihdr_write_shutter_gain(
			kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
}

static void set_mirror_flip(kal_uint8 image_mirror)
{
        kal_uint8 itemp;

        LOG_INF("image_mirror = %d\n", image_mirror);
        itemp = read_cmos_sensor(0x0101);
        itemp &= ~0x03;

        switch (image_mirror) {

        case IMAGE_NORMAL:
        write_cmos_sensor(0x0101, itemp);
        break;

        case IMAGE_V_MIRROR:
        write_cmos_sensor(0x0101, itemp | 0x02);
        break;

        case IMAGE_H_MIRROR:
        write_cmos_sensor(0x0101, itemp | 0x01);
        break;

        case IMAGE_HV_MIRROR:
        write_cmos_sensor(0x0101, itemp | 0x03);
        break;
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

#define MULTI_WRITE 1
#if MULTI_WRITE
#define I2C_BUFFER_LEN 225
extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length, u16 timing);
#else
#define I2C_BUFFER_LEN 3
#endif

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
#if MULTI_WRITE
        /* Write when remain buffer size is less than 3 bytes or reach end of data */
        if ((I2C_BUFFER_LEN - tosend) < 3 || IDX == len || addr != addr_last) {
            iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id, 3, imgsensor_info.i2c_speed);
            tosend = 0;
        }
#else
        iWriteRegI2C(puSendCmd, 3, imgsensor.i2c_write_id);
        tosend = 0;

#endif
    }
    return 0;
}

static kal_uint16 addr_data_pair_global[] = {
    0x0136, 0x18,
    0x0137, 0x00,
    0x3C7E, 0x01,
    0x3C7F, 0x05,
    0x3E35, 0x00,
    0x3E36, 0x00,
    0x3E37, 0x00,
    0x3F7F, 0x01,
    0x4431, 0x04,
    0x531C, 0x01,
    0x531D, 0x02,
    0x531E, 0x04,
    0x5928, 0x00,
    0x5929, 0x2F,
    0x592A, 0x00,
    0x592B, 0x85,
    0x592C, 0x00,
    0x592D, 0x32,
    0x592E, 0x00,
    0x592F, 0x88,
    0x5930, 0x00,
    0x5931, 0x3D,
    0x5932, 0x00,
    0x5933, 0x93,
    0x5938, 0x00,
    0x5939, 0x24,
    0x593A, 0x00,
    0x593B, 0x7A,
    0x593C, 0x00,
    0x593D, 0x24,
    0x593E, 0x00,
    0x593F, 0x7A,
    0x5940, 0x00,
    0x5941, 0x2F,
    0x5942, 0x00,
    0x5943, 0x85,
    0x5F0E, 0x6E,
    0x5F11, 0xC6,
    0x5F17, 0x5E,
    0x7990, 0x01,
    0x7993, 0x5D,
    0x7994, 0x5D,
    0x7995, 0xA1,
    0x799A, 0x01,
    0x799D, 0x00,
    0x8169, 0x01,
    0x8359, 0x01,
    0x9302, 0x1E,
    0x9306, 0x1F,
    0x930A, 0x26,
    0x930E, 0x23,
    0x9312, 0x23,
    0x9316, 0x2C,
    0x9317, 0x19,
    0xB046, 0x01,
    0xB048, 0x01,
    /*Image Quality adjustment setting*/
    0xAA06, 0x3F,
    0xAA07, 0x05,
    0xAA08, 0x04,
    0xAA12, 0x3F,
    0xAA13, 0x04,
    0xAA14, 0x03,
    0xAB55, 0x02,
    0xAB57, 0x01,
    0xAB59, 0x01,
    0xABB4, 0x00,
    0xABB5, 0x01,
    0xABB6, 0x00,
    0xABB7, 0x01,
    0xABB8, 0x00,
    0xABB9, 0x01,
};

static kal_uint16 addr_data_pair_preview[] = {
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x0A,
    0x0343, 0x00,
    0x0340, 0x0C,
    0x0341, 0x68,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x12,
    0x0349, 0x2F,
    0x034A, 0x0D,
    0x034B, 0xA7,
    0x0381, 0x01,
    0x0383, 0x01,
    0x0385, 0x01,
    0x0387, 0x01,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x08,
    0x3F4C, 0x81,
    0x3F4D, 0x81,
    0x0408, 0x00,
    0x0409, 0x0C,
    0x040A, 0x00,
    0x040B, 0x0A,
    0x040C, 0x09,
    0x040D, 0x00,
    0x040E, 0x06,
    0x040F, 0xC0,
    0x034C, 0x09,
    0x034D, 0x00,
    0x034E, 0x06,
    0x034F, 0xC0,
    0x0301, 0x06,
    0x0303, 0x04,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x7A,
    0x030B, 0x01,
    0x030D, 0x02,
    0x030E, 0x00,
    0x030F, 0x8E,
    0x0310, 0x01,
    0x3F78, 0x01,
    0x3F79, 0x31,
    0x3FFE, 0x00,
    0x3FFF, 0x8A,
    0x5F0A, 0xB6,
    0x0202, 0x0C,
    0x0203, 0x56,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3F15, 0x00,
};

static kal_uint16 addr_data_pair_capture[] = {
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x0A,
    0x0343, 0x00,
    0x0340, 0x0C,
    0x0341, 0x68,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x12,
    0x0349, 0x2F,
    0x034A, 0x0D,
    0x034B, 0xA7,
    0x0381, 0x01,
    0x0383, 0x01,
    0x0385, 0x01,
    0x0387, 0x01,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x08,
    0x3F4C, 0x81,
    0x3F4D, 0x81,
    0x0408, 0x00,
    0x0409, 0x0C,
    0x040A, 0x00,
    0x040B, 0x0A,
    0x040C, 0x09,
    0x040D, 0x00,
    0x040E, 0x06,
    0x040F, 0xC0,
    0x034C, 0x09,
    0x034D, 0x00,
    0x034E, 0x06,
    0x034F, 0xC0,
    0x0301, 0x06,
    0x0303, 0x04,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x7A,
    0x030B, 0x01,
    0x030D, 0x02,
    0x030E, 0x00,
    0x030F, 0x8E,
    0x0310, 0x01,
    0x3F78, 0x01,
    0x3F79, 0x31,
    0x3FFE, 0x00,
    0x3FFF, 0x8A,
    0x5F0A, 0xB6,
    0x0202, 0x0C,
    0x0203, 0x56,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3F15, 0x00,
};

/*
static kal_uint16 addr_data_pair_normal_video[] = {
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x0A,
    0x0343, 0x00,
    0x0340, 0x1D,
    0x0341, 0x80,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x12,
    0x0349, 0x2F,
    0x034A, 0x0D,
    0x034B, 0xA7,
    0x0381, 0x01,
    0x0383, 0x01,
    0x0385, 0x01,
    0x0387, 0x01,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x08,
    0x3F4C, 0x81,
    0x3F4D, 0x81,
    0x0408, 0x00,
    0x0409, 0x0C,
    0x040A, 0x00,
    0x040B, 0x0A,
    0x040C, 0x09,
    0x040D, 0x00,
    0x040E, 0x06,
    0x040F, 0xC0,
    0x034C, 0x09,
    0x034D, 0x00,
    0x034E, 0x06,
    0x034F, 0xC0,
    0x0301, 0x06,
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x91,
    0x030B, 0x01,
    0x030D, 0x02,
    0x030E, 0x00,
    0x030F, 0x8E,
    0x0310, 0x01,
    0x3F78, 0x01,
    0x3F79, 0x31,
    0x3FFE, 0x00,
    0x3FFF, 0x8A,
    0x5F0A, 0xB6,
    0x0202, 0x1D,
    0x0203, 0x6E,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3F15, 0x00,
};
*/

static kal_uint16 addr_data_pair_hs_video[] = {
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x0A,
    0x0343, 0x00,
    0x0340, 0x07,
    0x0341, 0x60,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x12,
    0x0349, 0x2F,
    0x034A, 0x0D,
    0x034B, 0xA7,
    0x0381, 0x01,
    0x0383, 0x01,
    0x0385, 0x01,
    0x0387, 0x01,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x08,
    0x3F4C, 0x81,
    0x3F4D, 0x81,
    0x0408, 0x00,
    0x0409, 0x0C,
    0x040A, 0x00,
    0x040B, 0x0A,
    0x040C, 0x09,
    0x040D, 0x00,
    0x040E, 0x06,
    0x040F, 0xC0,
    0x034C, 0x09,
    0x034D, 0x00,
    0x034E, 0x06,
    0x034F, 0xC0,
    0x0301, 0x06,
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x91,
    0x030B, 0x01,
    0x030D, 0x02,
    0x030E, 0x00,
    0x030F, 0x8E,
    0x0310, 0x01,
    0x3F78, 0x01,
    0x3F79, 0x31,
    0x3FFE, 0x00,
    0x3FFF, 0x8A,
    0x5F0A, 0xB6,
    0x0202, 0x07,
    0x0203, 0x4E,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3F15, 0x00,
};

static kal_uint16 addr_data_pair_custom1[] = {
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x0A,
    0x0343, 0x00,
    0x0340, 0x07,
    0x0341, 0xE2,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x12,
    0x0349, 0x2F,
    0x034A, 0x0D,
    0x034B, 0xA7,
    0x0381, 0x01,
    0x0383, 0x01,
    0x0385, 0x01,
    0x0387, 0x01,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x08,
    0x3F4C, 0x81,
    0x3F4D, 0x81,
    0x0408, 0x00,
    0x0409, 0x0C,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x09,
    0x040D, 0x00,
    0x040E, 0x06,
    0x040F, 0xC0,
    0x034C, 0x09,
    0x034D, 0x00,
    0x034E, 0x06,
    0x034F, 0xC0,
    0x0301, 0x06,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x00,
    0x0307, 0x3E,
    0x030B, 0x01,
    0x030D, 0x02,
    0x030E, 0x00,
    0x030F, 0x1D,
    0x0310, 0x01,
    0x3F78, 0x01,
    0x3F79, 0x31,
    0x3FFE, 0x00,
    0x3FFF, 0x8A,
    0x5F0A, 0xB6,
    0x0202, 0x07,
    0x0203, 0xD0,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3F15, 0x00,
};

static kal_uint16 addr_data_pair_custom2[] = {
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x0A,
    0x0343, 0x00,
    0x0340, 0x1D,
    0x0341, 0x80,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x12,
    0x0349, 0x2F,
    0x034A, 0x0D,
    0x034B, 0xA7,
    0x0381, 0x01,
    0x0383, 0x01,
    0x0385, 0x01,
    0x0387, 0x01,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x08,
    0x3F4C, 0x81,
    0x3F4D, 0x81,
    0x0408, 0x00,
    0x0409, 0x0C,
    0x040A, 0x00,
    0x040B, 0x0A,
    0x040C, 0x09,
    0x040D, 0x00,
    0x040E, 0x06,
    0x040F, 0xC0,
    0x034C, 0x09,
    0x034D, 0x00,
    0x034E, 0x06,
    0x034F, 0xC0,
    0x0301, 0x06,
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x91,
    0x030B, 0x01,
    0x030D, 0x02,
    0x030E, 0x00,
    0x030F, 0x8E,
    0x0310, 0x01,
    0x3F78, 0x01,
    0x3F79, 0x31,
    0x3FFE, 0x00,
    0x3FFF, 0x8A,
    0x5F0A, 0xB6,
    0x0202, 0x1D,
    0x0203, 0x6E,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3F15, 0x00,
};

static kal_uint16 addr_data_pair_custom3[] = {
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x14,
    0x0343, 0x00,
    0x0340, 0x0E,
    0x0341, 0x6C,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x14,
    0x0348, 0x12,
    0x0349, 0x2F,
    0x034A, 0x0D,
    0x034B, 0x93,
    0x0381, 0x01,
    0x0383, 0x01,
    0x0385, 0x01,
    0x0387, 0x01,
    0x0900, 0x00,
    0x0901, 0x11,
    0x0902, 0x0A,
    0x3F4C, 0x01,
    0x3F4D, 0x01,
    0x0408, 0x00,
    0x0409, 0x18,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x12,
    0x040D, 0x00,
    0x040E, 0x0D,
    0x040F, 0x80,
    0x034C, 0x12,
    0x034D, 0x00,
    0x034E, 0x0D,
    0x034F, 0x80,
    0x0301, 0x06,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x1C,
    0x030B, 0x01,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0x9C,
    0x0310, 0x01,
    0x3F78, 0x02,
    0x3F79, 0x0A,
    0x3FFE, 0x00,
    0x3FFF, 0x18,
    0x5F0A, 0xB2,
    0x0202, 0x0E,
    0x0203, 0x5A,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3F15, 0x00,
};

static kal_uint16 addr_data_pair_custom4[] = {
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x0A,
    0x0343, 0x00,
    0x0340, 0x06,
    0x0341, 0xB6,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x01,
    0x0347, 0xF8,
    0x0348, 0x12,
    0x0349, 0x2F,
    0x034A, 0x0B,
    0x034B, 0xAF,
    0x0381, 0x01,
    0x0383, 0x01,
    0x0385, 0x01,
    0x0387, 0x01,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x08,
    0x3F4C, 0x81,
    0x3F4D, 0x81,
    0x0408, 0x01,
    0x0409, 0x5C,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x06,
    0x040D, 0x60,
    0x040E, 0x04,
    0x040F, 0xDC,
    0x034C, 0x06,
    0x034D, 0x60,
    0x034E, 0x04,
    0x034F, 0xDC,
    0x0301, 0x06,
    0x0303, 0x04,
    0x0305, 0x04,
    0x0306, 0x00,
    0x0307, 0x84,
    0x030B, 0x01,
    0x030D, 0x02,
    0x030E, 0x00,
    0x030F, 0x8E,
    0x0310, 0x01,
    0x3F78, 0x01,
    0x3F79, 0x31,
    0x3FFE, 0x00,
    0x3FFF, 0x8A,
    0x5F0A, 0xB6,
    0x0202, 0x06,
    0x0203, 0xA4,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3F15, 0x00,
};

/*************************************************************************
 * FUNCTION
 *	sensor_init
 *
 * DESCRIPTION
 *	Sensor init
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void sensor_init(void)
{
	LOG_INF("v2 E\n");

	#if 1
	table_write_cmos_sensor(addr_data_pair_global,
		sizeof(addr_data_pair_global) / sizeof(kal_uint16));
	#else
		write_cmos_sensor(0x0136, 0x18);
		write_cmos_sensor(0x0137, 0x00);
		write_cmos_sensor(0x3C7E, 0x02);
		write_cmos_sensor(0x3C7F, 0x05);
		write_cmos_sensor(0x3E35, 0x00);
		write_cmos_sensor(0x3E36, 0x00);
		write_cmos_sensor(0x3E37, 0x00);
		write_cmos_sensor(0x3F7F, 0x01);
		write_cmos_sensor(0x4431, 0x04);
		write_cmos_sensor(0x531C, 0x01);
		write_cmos_sensor(0x531D, 0x02);
		write_cmos_sensor(0x531E, 0x04);
		write_cmos_sensor(0x5928, 0x00);
		write_cmos_sensor(0x5929, 0x2F);
		write_cmos_sensor(0x592A, 0x00);
		write_cmos_sensor(0x592B, 0x85);
		write_cmos_sensor(0x592C, 0x00);
		write_cmos_sensor(0x592D, 0x32);
		write_cmos_sensor(0x592E, 0x00);
		write_cmos_sensor(0x592F, 0x88);
		write_cmos_sensor(0x5930, 0x00);
		write_cmos_sensor(0x5931, 0x3D);
		write_cmos_sensor(0x5932, 0x00);
		write_cmos_sensor(0x5933, 0x93);
		write_cmos_sensor(0x5938, 0x00);
		write_cmos_sensor(0x5939, 0x24);
		write_cmos_sensor(0x593A, 0x00);
		write_cmos_sensor(0x593B, 0x7A);
		write_cmos_sensor(0x593C, 0x00);
		write_cmos_sensor(0x593D, 0x24);
		write_cmos_sensor(0x593E, 0x00);
		write_cmos_sensor(0x593F, 0x7A);
		write_cmos_sensor(0x5940, 0x00);
		write_cmos_sensor(0x5941, 0x2F);
		write_cmos_sensor(0x5942, 0x00);
		write_cmos_sensor(0x5943, 0x85);
		write_cmos_sensor(0x5F0E, 0x6E);
		write_cmos_sensor(0x5F11, 0xC6);
		write_cmos_sensor(0x5F17, 0x5E);
		write_cmos_sensor(0x7990, 0x01);
		write_cmos_sensor(0x7993, 0x5D);
		write_cmos_sensor(0x7994, 0x5D);
		write_cmos_sensor(0x7995, 0xA1);
		write_cmos_sensor(0x799A, 0x01);
		write_cmos_sensor(0x799D, 0x00);
		write_cmos_sensor(0x8169, 0x01);
		write_cmos_sensor(0x8359, 0x01);
		write_cmos_sensor(0x9302, 0x1E);
		write_cmos_sensor(0x9306, 0x1F);
		write_cmos_sensor(0x930A, 0x26);
		write_cmos_sensor(0x930E, 0x23);
		write_cmos_sensor(0x9312, 0x23);
		write_cmos_sensor(0x9316, 0x2C);
		write_cmos_sensor(0x9317, 0x19);
		write_cmos_sensor(0xB046, 0x01);
		write_cmos_sensor(0xB048, 0x01);
		write_cmos_sensor(0xAA06, 0x3F);
		write_cmos_sensor(0xAA07, 0x05);
		write_cmos_sensor(0xAA08, 0x04);
		write_cmos_sensor(0xAA12, 0x3F);
		write_cmos_sensor(0xAA13, 0x04);
		write_cmos_sensor(0xAA14, 0x03);
		write_cmos_sensor(0xAB55, 0x02);
		write_cmos_sensor(0xAB57, 0x01);
		write_cmos_sensor(0xAB59, 0x01);
		write_cmos_sensor(0xABB4, 0x00);
		write_cmos_sensor(0xABB5, 0x01);
		write_cmos_sensor(0xABB6, 0x00);
		write_cmos_sensor(0xABB7, 0x01);
		write_cmos_sensor(0xABB8, 0x00);
		write_cmos_sensor(0xABB9, 0x01);
	#endif
}	/*	sensor_init  */

/*************************************************************************
 * FUNCTION
 *	preview_setting
 *
 * DESCRIPTION
 *	Sensor preview
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void preview_setting(void)
{
	#if 1
	table_write_cmos_sensor(addr_data_pair_preview,
		sizeof(addr_data_pair_preview) / sizeof(kal_uint16));
	#else
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0342, 0x0A);
	write_cmos_sensor(0x0343, 0x00);
	write_cmos_sensor(0x0340, 0x11);
	write_cmos_sensor(0x0341, 0xE6);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x14);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0x93);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x22);
	write_cmos_sensor(0x0902, 0x09);
	write_cmos_sensor(0x3F4C, 0x81);
	write_cmos_sensor(0x3F4D, 0x81);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x0C);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x09);
	write_cmos_sensor(0x040D, 0x00);
	write_cmos_sensor(0x040E, 0x06);
	write_cmos_sensor(0x040F, 0xC0);
	write_cmos_sensor(0x034C, 0x09);
	write_cmos_sensor(0x034D, 0x00);
	write_cmos_sensor(0x034E, 0x06);
	write_cmos_sensor(0x034F, 0xC0);
	write_cmos_sensor(0x0301, 0x06);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xB0);
	write_cmos_sensor(0x030B, 0x02);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x04);
	write_cmos_sensor(0x030F, 0x7E);
	write_cmos_sensor(0x0310, 0x01);
	write_cmos_sensor(0x3F78, 0x01);
	write_cmos_sensor(0x3F79, 0x31);
	write_cmos_sensor(0x3FFE, 0x00);
	write_cmos_sensor(0x3FFF, 0x8A);
	write_cmos_sensor(0x5F0A, 0xB6);
	write_cmos_sensor(0x0202, 0x11);
	write_cmos_sensor(0x0203, 0xD4);
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	#endif
}	/*	preview_setting  */
/*************************************************************************
 * FUNCTION
 *	Capture
 *
 * DESCRIPTION
 *	Sensor capture
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/

static void capture_setting(void)
{
	#if 1
	table_write_cmos_sensor(addr_data_pair_capture,
		sizeof(addr_data_pair_capture) / sizeof(kal_uint16));
	#else
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0342, 0x0A);
	write_cmos_sensor(0x0343, 0x00);
	write_cmos_sensor(0x0340, 0x11);
	write_cmos_sensor(0x0341, 0xE6);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x14);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0x93);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x22);
	write_cmos_sensor(0x0902, 0x09);
	write_cmos_sensor(0x3F4C, 0x81);
	write_cmos_sensor(0x3F4D, 0x81);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x0C);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x09);
	write_cmos_sensor(0x040D, 0x00);
	write_cmos_sensor(0x040E, 0x06);
	write_cmos_sensor(0x040F, 0xC0);
	write_cmos_sensor(0x034C, 0x09);
	write_cmos_sensor(0x034D, 0x00);
	write_cmos_sensor(0x034E, 0x06);
	write_cmos_sensor(0x034F, 0xC0);
	write_cmos_sensor(0x0301, 0x06);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xB0);
	write_cmos_sensor(0x030B, 0x02);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x04);
	write_cmos_sensor(0x030F, 0x7E);
	write_cmos_sensor(0x0310, 0x01);
	write_cmos_sensor(0x3F78, 0x01);
	write_cmos_sensor(0x3F79, 0x31);
	write_cmos_sensor(0x3FFE, 0x00);
	write_cmos_sensor(0x3FFF, 0x8A);
	write_cmos_sensor(0x5F0A, 0xB6);
	write_cmos_sensor(0x0202, 0x11);
	write_cmos_sensor(0x0203, 0xD4);
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	#endif
}

static void normal_video_setting(void)
{
	LOG_INF("E!\n");
	#if 0
	table_write_cmos_sensor(addr_data_pair_normal_video,
		sizeof(addr_data_pair_normal_video) / sizeof(kal_uint16));
	#else
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0342, 0x0A);
	write_cmos_sensor(0x0343, 0x00);
	write_cmos_sensor(0x0340, 0x0B);
	write_cmos_sensor(0x0341, 0x30);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x01);
	write_cmos_sensor(0x0347, 0xBC);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0B);
	write_cmos_sensor(0x034B, 0xEB);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x22);
	write_cmos_sensor(0x0902, 0x08);
	write_cmos_sensor(0x3F4C, 0x81);
	write_cmos_sensor(0x3F4D, 0x81);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x0C);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x04);
	write_cmos_sensor(0x040C, 0x09);
	write_cmos_sensor(0x040D, 0x00);
	write_cmos_sensor(0x040E, 0x05);
	write_cmos_sensor(0x040F, 0x10);
	write_cmos_sensor(0x034C, 0x09);
	write_cmos_sensor(0x034D, 0x00);
	write_cmos_sensor(0x034E, 0x05);
	write_cmos_sensor(0x034F, 0x10);
	write_cmos_sensor(0x0301, 0x06);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0x6E);
	write_cmos_sensor(0x030B, 0x02);
	write_cmos_sensor(0x030D, 0x06);
	write_cmos_sensor(0x030E, 0x01);
	write_cmos_sensor(0x030F, 0xCC);
	write_cmos_sensor(0x0310, 0x01);
	write_cmos_sensor(0x3F78, 0x01);
	write_cmos_sensor(0x3F79, 0x31);
	write_cmos_sensor(0x3FFE, 0x00);
	write_cmos_sensor(0x3FFF, 0x8A);
	write_cmos_sensor(0x5F0A, 0xB6);
	write_cmos_sensor(0x0202, 0x0B);
	write_cmos_sensor(0x0203, 0x1E);
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x3F15, 0x00);
	#endif
}

static void hs_video_setting(void)
{
	LOG_INF("E!\n");

	table_write_cmos_sensor(addr_data_pair_hs_video,
		sizeof(addr_data_pair_hs_video) / sizeof(kal_uint16));
}

static void slim_video_setting(void)
{
	LOG_INF("E!\n");

	normal_video_setting();
}

static void custom1_setting(void)
{
	LOG_INF("E!\n");
	#if 1
	table_write_cmos_sensor(addr_data_pair_custom1,
		sizeof(addr_data_pair_custom1) / sizeof(kal_uint16));
	#else
	//24fps B4  2304x1728
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0342, 0x0A);
	write_cmos_sensor(0x0343, 0x00);
	write_cmos_sensor(0x0340, 0x08);
	write_cmos_sensor(0x0341, 0xC6);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x14);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0x93);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x22);
	write_cmos_sensor(0x0902, 0x09);
	write_cmos_sensor(0x3F4C, 0x81);
	write_cmos_sensor(0x3F4D, 0x81);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x0C);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x09);
	write_cmos_sensor(0x040D, 0x00);
	write_cmos_sensor(0x040E, 0x06);
	write_cmos_sensor(0x040F, 0xC0);
	write_cmos_sensor(0x034C, 0x09);
	write_cmos_sensor(0x034D, 0x00);
	write_cmos_sensor(0x034E, 0x06);
	write_cmos_sensor(0x034F, 0xC0);
	write_cmos_sensor(0x0301, 0x06);
	write_cmos_sensor(0x0303, 0x04);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0x8A);
	write_cmos_sensor(0x030B, 0x02);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0xB6);
	write_cmos_sensor(0x0310, 0x01);
	write_cmos_sensor(0x3F78, 0x01);
	write_cmos_sensor(0x3F79, 0x31);
	write_cmos_sensor(0x3FFE, 0x00);
	write_cmos_sensor(0x3FFF, 0x8A);
	write_cmos_sensor(0x5F0A, 0xB6);
	write_cmos_sensor(0x0202, 0x08);
	write_cmos_sensor(0x0203, 0xB4);
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	#endif
}

static void custom2_setting(void)
{
	LOG_INF("E!\n");
	#if 1
	table_write_cmos_sensor(addr_data_pair_custom2,
		sizeof(addr_data_pair_custom2) / sizeof(kal_uint16));
	#else
	//24fps B4  2304x1728
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0342, 0x0A);
	write_cmos_sensor(0x0343, 0x00);
	write_cmos_sensor(0x0340, 0x08);
	write_cmos_sensor(0x0341, 0xD6);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x00);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0xA7);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x22);
	write_cmos_sensor(0x0902, 0x09);
	write_cmos_sensor(0x3F4C, 0x81);
	write_cmos_sensor(0x3F4D, 0x81);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x0C);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x0A);
	write_cmos_sensor(0x040C, 0x09);
	write_cmos_sensor(0x040D, 0x00);
	write_cmos_sensor(0x040E, 0x06);
	write_cmos_sensor(0x040F, 0xC0);
	write_cmos_sensor(0x034C, 0x09);
	write_cmos_sensor(0x034D, 0x00);
	write_cmos_sensor(0x034E, 0x06);
	write_cmos_sensor(0x034F, 0xC0);
	write_cmos_sensor(0x0301, 0x06);
	write_cmos_sensor(0x0303, 0x04);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0x8B);
	write_cmos_sensor(0x030B, 0x02);
	write_cmos_sensor(0x030D, 0x0C);
	write_cmos_sensor(0x030E, 0x01);
	write_cmos_sensor(0x030F, 0x4B);
	write_cmos_sensor(0x0310, 0x01);
	write_cmos_sensor(0x3F78, 0x01);
	write_cmos_sensor(0x3F79, 0x31);
	write_cmos_sensor(0x3FFE, 0x00);
	write_cmos_sensor(0x3FFF, 0x8A);
	write_cmos_sensor(0x5F0A, 0xB6);
	write_cmos_sensor(0x0202, 0x08);
	write_cmos_sensor(0x0203, 0xC4);
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	#endif
}

static void custom3_setting(void)
{
	LOG_INF("E!\n");
	#if 1
	table_write_cmos_sensor(addr_data_pair_custom3,
		sizeof(addr_data_pair_custom3) / sizeof(kal_uint16));
	#else
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0342, 0x14);
	write_cmos_sensor(0x0343, 0x00);
	write_cmos_sensor(0x0340, 0x0D);
	write_cmos_sensor(0x0341, 0xEE);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x14);
	write_cmos_sensor(0x0348, 0x12);
	write_cmos_sensor(0x0349, 0x2F);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0x93);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x0A);
	write_cmos_sensor(0x3F4C, 0x01);
	write_cmos_sensor(0x3F4D, 0x01);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x18);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x12);
	write_cmos_sensor(0x040D, 0x00);
	write_cmos_sensor(0x040E, 0x0D);
	write_cmos_sensor(0x040F, 0x80);
	write_cmos_sensor(0x034C, 0x12);
	write_cmos_sensor(0x034D, 0x00);
	write_cmos_sensor(0x034E, 0x0D);
	write_cmos_sensor(0x034F, 0x80);
	write_cmos_sensor(0x0301, 0x06);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x01);
	write_cmos_sensor(0x0307, 0x12);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x08);
	write_cmos_sensor(0x030E, 0x01);
	write_cmos_sensor(0x030F, 0xCD);
	write_cmos_sensor(0x0310, 0x01);
	write_cmos_sensor(0x3F78, 0x02);
	write_cmos_sensor(0x3F79, 0x0A);
	write_cmos_sensor(0x3FFE, 0x00);
	write_cmos_sensor(0x3FFF, 0x18);
	write_cmos_sensor(0x5F0A, 0xB2);
	write_cmos_sensor(0x0202, 0x0D);
	write_cmos_sensor(0x0203, 0xDC);
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	#endif
}

static void custom4_setting(void)
{
	LOG_INF("E!\n");

	table_write_cmos_sensor(addr_data_pair_custom4,
		sizeof(addr_data_pair_custom4) / sizeof(kal_uint16));
}

/*************************************************************************
 * FUNCTION
 *	Video
 *
 * DESCRIPTION
 *	Sensor video
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/


#define FOUR_CELL_SIZE 560
static kal_uint8  imx471_data_xtalk[FOUR_CELL_SIZE+2];

static void read_4cell_from_eeprom(void)
{
	int i = 0;
	int addr = 0x0C90;/*Start of 4 cell data*/
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	imx471_data_xtalk[0] = (FOUR_CELL_SIZE & 0xff);/*Low*/
	imx471_data_xtalk[1] = ((FOUR_CELL_SIZE >> 8) & 0xff);/*High*/

	for (i = 2; i < (FOUR_CELL_SIZE + 2); i++) {
		pu_send_cmd[0] = (char)(addr >> 8);
		pu_send_cmd[1] = (char)(addr & 0xFF);
		iReadRegI2C(pu_send_cmd, 2, &imx471_data_xtalk[i], 1, 0xA8);
		addr++;
	}
}

static unsigned int read_4cell_data_imx471(char *data)
{
	if (data != NULL) {
		memcpy((void*)(data) , (void*)imx471_data_xtalk, (FOUR_CELL_SIZE+2));
		pr_debug("imx471 read 4cell data[0]=%d, data[10]=%d, data[100]=%d\n", data[2], data[12], data[102]);
	}
	return 0;
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
	return ((read_cmos_sensor(0x300a) << 16) |
		(read_cmos_sensor(0x300b) << 8) | read_cmos_sensor(0x300c));
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	LOG_INF("IMX471,get_imgsensor_id Begin!!\n");
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			LOG_INF("IMX471,get_imgsensor_id: 0x%x !!\n", *sensor_id);
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
				read_EepromQSC();
				LOG_INF("module_id=%d\n",imgsensor_info.module_id);
				read_4cell_from_eeprom();

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
	kal_uint8 retry = 1;
	kal_uint32 sensor_id = 0;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
	spin_lock(&imgsensor_drv_lock);
	imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
	spin_unlock(&imgsensor_drv_lock);
	do {
	sensor_id = return_sensor_id();
	if (sensor_id == imgsensor_info.sensor_id) {
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

	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
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
	/*No Need to implement this function*/
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
	/*imgsensor.video_mode = KAL_FALSE;*/
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(imgsensor.mirror);
	mdelay(10);
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

	capture_setting();

	mdelay(10);
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
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting();
	set_mirror_flip(imgsensor.mirror);
	mdelay(10);
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
	mdelay(10);

	return ERROR_NONE;
}	/*	hs_video   */

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
	set_mirror_flip(imgsensor.mirror);
	mdelay(10);

	return ERROR_NONE;
}	/*	slim_video	 */

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
        //By wusongbai@camera
	set_mirror_flip(imgsensor.mirror);
	mdelay(10);
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
        //By wusongbai@camera
	set_mirror_flip(imgsensor.mirror);
	mdelay(10);
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
	set_mirror_flip(imgsensor.mirror);
	mdelay(10);
	return ERROR_NONE;
}   /*  Custom3   */

static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
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
    mdelay(10);

    return ERROR_NONE;
}

static kal_uint32 get_resolution(
			MSDK_SENSOR_RESOLUTION_INFO_STRUCT(*sensor_resolution))
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

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;

	sensor_resolution->SensorCustom2Width  = imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height     = imgsensor_info.custom2.grabwindow_height;

	sensor_resolution->SensorCustom3Width  = imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height     = imgsensor_info.custom3.grabwindow_height;

	sensor_resolution->SensorCustom4Width = imgsensor_info.custom4.grabwindow_width;
	sensor_resolution->SensorCustom4Height = imgsensor_info.custom4.grabwindow_height;

	return ERROR_NONE;
}	/*	get_resolution	*/


static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
				MSDK_SENSOR_INFO_STRUCT *sensor_info,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
    LOG_INF("mipi_lane_num = %d\n", imgsensor_info.mipi_lane_num);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* not use */
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* inverse with datasheet*/
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

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

	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
	sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame; /* The delay frame of setting frame length  */

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
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x*/
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

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
			Custom1(image_window, sensor_config_data);  // Custom1
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			Custom2(image_window, sensor_config_data);  // Custom2
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			Custom3(image_window, sensor_config_data);  // Custom3
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			Custom4(image_window, sensor_config_data);
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
	/* SetVideoMode Function should fix framerate*/
	if (framerate == 0) {
		/* Dynamic frame rate*/
		return ERROR_NONE;
	}
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE)) {
		imgsensor.current_fps = 296;
	} else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE)) {
		imgsensor.current_fps = 146;
	} else {
		imgsensor.current_fps = framerate;
	}
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) {/*enable auto flicker	  */
		imgsensor.autoflicker_en = KAL_TRUE;
	} else {/*Cancel Auto flick*/
		imgsensor.autoflicker_en = KAL_FALSE;
	}
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
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.pre.framelength) {
			imgsensor.dummy_line = (frame_length - imgsensor_info.pre.framelength);
		} else {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if(imgsensor.frame_length > imgsensor.shutter)
				set_dummy();
		break;

	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0) {
			return ERROR_NONE;
		}

		frame_length = imgsensor_info.normal_video.pclk / framerate * 10;

		frame_length /= imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);

		if (frame_length > imgsensor_info.normal_video.framelength) {
			imgsensor.dummy_line = frame_length - imgsensor_info.normal_video.framelength;
		} else {
			imgsensor.dummy_line = 0;
		}

		imgsensor.frame_length =
		imgsensor_info.normal_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if(imgsensor.frame_length > imgsensor.shutter)
				set_dummy();
		break;

	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.cap.framelength) {
			imgsensor.dummy_line = (frame_length - imgsensor_info.cap.framelength);
		} else {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if(imgsensor.frame_length > imgsensor.shutter)
				set_dummy();
		break;

	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10;

		frame_length /= imgsensor_info.hs_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.hs_video.framelength) {
			imgsensor.dummy_line =(frame_length - imgsensor_info.hs_video.framelength);
		} else {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if(imgsensor.frame_length > imgsensor.shutter)
				set_dummy();
		break;

	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10;

		frame_length /= imgsensor_info.slim_video.linelength;

		spin_lock(&imgsensor_drv_lock);

		if (frame_length > imgsensor_info.slim_video.framelength) {
			imgsensor.dummy_line = (frame_length - imgsensor_info.slim_video.framelength);
		} else {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if(imgsensor.frame_length > imgsensor.shutter)
				set_dummy();

		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
		if (imgsensor.dummy_line < 0) {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter) {
			set_dummy();
		}
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
		if (imgsensor.dummy_line < 0) {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter) {
			set_dummy();
		}
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength) : 0;
		if (imgsensor.dummy_line < 0) {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter) {
			set_dummy();
		}
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		frame_length = imgsensor_info.custom4.pclk / framerate * 10 / imgsensor_info.custom4.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength) ? (frame_length - imgsensor_info.custom4.framelength) : 0;
		if (imgsensor.dummy_line < 0) {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter) {
			set_dummy();
		}
		break;
	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.pre.framelength) {
			imgsensor.dummy_line = (frame_length - imgsensor_info.pre.framelength);
		} else {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if(imgsensor.frame_length > imgsensor.shutter)
				set_dummy();

		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
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
                write_cmos_sensor(0x0600, modes>>4);
                write_cmos_sensor(0x0601, modes);
                if (modes == 1 && (pTestpatterndata != NULL)) { //Solid Color
                        Color_R = (pTestpatterndata->COLOR_R >> 16) & 0xFFFF;
                        Color_Gr = (pTestpatterndata->COLOR_Gr >> 16) & 0xFFFF;
                        Color_B = (pTestpatterndata->COLOR_B >> 16) & 0xFFFF;
                        Color_Gb = (pTestpatterndata->COLOR_Gb >> 16) & 0xFFFF;
                        write_cmos_sensor(0x0602, Color_R >> 8);
                        write_cmos_sensor(0x0603, Color_R & 0xFF);
                        write_cmos_sensor(0x0602, Color_Gr >> 8);
                        write_cmos_sensor(0x0603, Color_Gr & 0xFF);
                        write_cmos_sensor(0x0602, Color_B >> 8);
                        write_cmos_sensor(0x0603, Color_B & 0xFF);
                        write_cmos_sensor(0x0602, Color_Gb >> 8);
                        write_cmos_sensor(0x0603, Color_Gb & 0xFF);
                }
        } else
                write_cmos_sensor(0x0600, 0x0000); /*No pattern*/

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = modes;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	printk("IMX471,streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor(0x0100, 0X01);
	else
		write_cmos_sensor(0x0100, 0x00);
	mdelay(10);
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
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
				(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

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
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
						= imgsensor_info.pre.pclk;
				break;
			}
		break;
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
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
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
						= (imgsensor_info.pre.framelength << 16)
								 + imgsensor_info.pre.linelength;
				break;
		}
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL) (*feature_data));
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
		set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16 + 1));
		break;

	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data + 1));
		break;

	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data),(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((UINT8)*feature_data, (struct SET_SENSOR_PATTERN_SOLID_COLOR *) feature_data+1);
		break;

	/*for factory mode auto testing*/
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;

	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_mode = (UINT8)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			LOG_INF("lch MSDK_SCENARIO_ID_CUSTOM1 \n");
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[5], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			LOG_INF("lch MSDK_SCENARIO_ID_CUSTOM2 \n");
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[6], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			LOG_INF("lch MSDK_SCENARIO_ID_CUSTOM3 \n");
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[7], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[8],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;

	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n", (UINT16)*feature_data, (UINT16)*(feature_data + 1), (UINT16)*(feature_data + 2));

		ihdr_write_shutter_gain((UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16)*(feature_data + 2));
		break;

	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data), (UINT16) (*(feature_data + 1)), (UINT16) *(feature_data + 2));
		break;

	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		LOG_INF("This sensor can't support temperature get\n");
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);

		if (*feature_data != 0) {
			set_shutter(*feature_data);
		}
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		{
			kal_uint32 rate;
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					rate = imgsensor_info.cap.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					rate = imgsensor_info.normal_video.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					rate = imgsensor_info.hs_video.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_CUSTOM1:
					rate = imgsensor_info.custom1.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_CUSTOM2:
					rate = imgsensor_info.custom2.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_CUSTOM3:
					rate = imgsensor_info.custom3.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_CUSTOM4:
					rate = imgsensor_info.custom4.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					rate = imgsensor_info.pre.mipi_pixel_rate;
					break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
		}
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
			case MSDK_SCENARIO_ID_CUSTOM3:
				*feature_return_para_32 = 2; /*BINNING_NONE*/
				break;
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
			case MSDK_SCENARIO_ID_CUSTOM1:
			case MSDK_SCENARIO_ID_CUSTOM2:
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				*feature_return_para_32 = 1; /*BINNING_SUM*/
				break;
		}
		LOG_INF("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d\n",(UINT32)*feature_return_para_32);
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_4CELL_DATA:
		{
			/*get 4 cell data from eeprom*/
			int type = (kal_uint16)(*feature_data);
			char *data = (char *)(uintptr_t)(*(feature_data+1));

			memset(data, 0, FOUR_CELL_SIZE);

			if (type == FOUR_CELL_CAL_TYPE_GAIN_TBL) {
				read_4cell_data_imx471(data);
				LOG_INF(
					"read Cross Talk = %02x %02x %02x %02x %02x %02x\n",
					(UINT16)data[0], (UINT16)data[1],
					(UINT16)data[2], (UINT16)data[3],
					(UINT16)data[4], (UINT16)data[5]);
			}
			break;
		}
    case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
        *(feature_data + 1) = 1; //margin info by scenario
        *(feature_data + 2) = imgsensor_info.margin;
        break;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		case SENSOR_FEATURE_SET_SENSOR_OTP:
		{
			kal_int32 ret = IMGSENSOR_RETURN_SUCCESS;
			LOG_INF("SENSOR_FEATURE_SET_SENSOR_OTP length :%d\n", (UINT32)*feature_para_len);
			ret = Eeprom_CallWriteService((ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(feature_para));
			if (ret == IMGSENSOR_RETURN_SUCCESS)
				return ERROR_NONE;
			else
				return ERROR_MSDK_IS_ACTIVATED;
		}
		case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = -1190000;
			break;
#endif
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

UINT32 IMX471_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL) {
		*pfFunc = &sensor_func;
	}
	return ERROR_NONE;
}	/*	imx4711q_MIPI_RAW_SensorInit	*/
