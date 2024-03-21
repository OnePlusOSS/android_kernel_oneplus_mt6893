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
#include "soc/oplus/system/oplus_project.h"
#include "parkera_shinetech_front_ov08d10mipiraw_Sensor.h"
#define EEPROM_WRITE_ID 0xA0
#define OV08D10_ECO_VALUE 3
#define PFX "PARKERA_SHINETECH_FRONT_OV08D10_camera_sensor"
#define LOG_INF(format, args...)    \
	pr_debug(PFX "[%s] " format, __func__, ##args)
#define MULTI_WRITE 1
/* Camera Hardwareinfo */
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define DEVICE_VERSION_PARKERA_SHINETECH_FRONT_OV08D10  "parkera_shinetech_front_ov08d10"
// #define MODULE_ID_OFFSET 0X0000
extern enum IMGSENSOR_RETURN Eeprom_DataInit(
    enum IMGSENSOR_SENSOR_IDX sensor_idx,
    kal_uint32 sensorID);
#endif
static kal_uint32 streaming_control(kal_bool enable);
static DEFINE_SPINLOCK(imgsensor_drv_lock);

int DVT = 0;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = PARKERA_SHINETECH_FRONT_OV08D10_SENSOR_ID_ORIGINAL,
	.checksum_value = 0xb7c53a42,

	.pre = {
			.pclk = 18000000,
			.linelength = 478,
			.framelength = 1252,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 1632,
			.grabwindow_height = 1224,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 72000000,
			.max_framerate = 300,
	},
	.cap = {
			.pclk = 36000000,
			.linelength = 460,
			.framelength = 2608,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 3264,
			.grabwindow_height = 2448,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 288000000,
			.max_framerate = 300,
	},
	.normal_video = {
			.pclk = 36000000,
			.linelength = 460,
			.framelength = 2608,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 3264,
			.grabwindow_height = 1836,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 288000000,
			.max_framerate = 300,
	},
	.margin = 16,            //sensor framelength & shutter margin
	.min_shutter = 8,        //min shutter
	.max_frame_length = 0x7fEE,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,

	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 6,

	.cap_delay_frame = 1,
	.pre_delay_frame = 1,
	.video_delay_frame = 1,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x20, 0xff},
	.i2c_speed = 400,
};

static struct imgsensor_info_struct imgsensor_info_DVT = {
	.sensor_id = PARKERA_SHINETECH_FRONT_OV08D10_SENSOR_ID_ORIGINAL,
	.checksum_value = 0xb7c53a42,

	.pre = {
			.pclk = 36000000,
			.linelength = 460,
			.framelength = 2448,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 1632,
			.grabwindow_height = 1224,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 144000000,
			.max_framerate = 300,
	},
	.cap = {
			.pclk = 36000000,
			.linelength = 460,
			.framelength = 2608,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 3264,
			.grabwindow_height = 2448,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 288000000,
			.max_framerate = 300,
	},
	.normal_video = {
			.pclk = 36000000,
			.linelength = 460,
			.framelength = 2608,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 3264,
			.grabwindow_height = 1836,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 288000000,
			.max_framerate = 300,
	},
	.margin = 16,            //sensor framelength & shutter margin
	.min_shutter = 8,        //min shutter
	.max_frame_length = 0x7fEE,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,

	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 6,

	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x20, 0xff},
	.i2c_speed = 400,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.gain = 0x100,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.ihdr_en = 0,
	.i2c_write_id = 0x20,
};

static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[3] = {
	{  3264, 2448,  0, 0, 3264, 2448, 1632, 1224, 0000, 0000, 1632, 1224,  0, 0, 1632, 1224},
	{  3264, 2448,  0, 0, 3264, 2448, 3264, 2448, 0000, 0000, 3264, 2448,  0, 0, 3264, 2448},
	{  3264, 2448,  0, 0, 3264, 2448, 3264, 2448, 0000, 0000, 3264, 1836,  0, 0, 3264, 1836},
};

#if MULTI_WRITE
#define I2C_BUFFER_LEN 225
#else
#define I2C_BUFFER_LEN 3
#endif

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	char pu_send_cmd[1] = {(char)(addr & 0xFF)};
	iReadRegI2C(pu_send_cmd, 1, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = {(char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
}

static kal_uint16 table_write_cmos_sensor(kal_uint16 *para,
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
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
#if MULTI_WRITE
		if ((I2C_BUFFER_LEN - tosend) < 2
			|| IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd,
						tosend,
						imgsensor.i2c_write_id,
						2,
						imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2C(puSendCmd, 2, imgsensor.i2c_write_id);
		tosend = 0;
#endif
	}

#if 0
	for (i = 0; i < len/2; i++)
		LOG_INF("readback addr(0x%x)=0x%x\n",
			para[2*i], read_cmos_sensor(para[2*i]));
#endif
	return 0;
}
static void set_dummy(void)
{
	if (imgsensor.frame_length%2 != 0) {
		imgsensor.frame_length = imgsensor.frame_length - imgsensor.frame_length % 2;
	}

	LOG_INF("imgsensor.frame_length = %d\n", imgsensor.frame_length);
	write_cmos_sensor(0xfd, 0x01);
	write_cmos_sensor(0x05, (imgsensor.dummy_line*2 >> 8) & 0xFF);
	write_cmos_sensor(0x06, imgsensor.dummy_line*2 & 0xFF);
	write_cmos_sensor(0xfd, 0x01);	//page1
	write_cmos_sensor(0x01, 0x01);	//fresh
}    /*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	write_cmos_sensor(0xfd, 0x00);
	return ((read_cmos_sensor(0x00) << 24) | (read_cmos_sensor(0x01) << 16)
		| (read_cmos_sensor(0x02) << 8) | read_cmos_sensor(0x03));
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;


	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
	{
		imgsensor.min_frame_length = imgsensor.frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);

	set_dummy();
}    /*    set_max_framerate  */

static void write_shutter(kal_uint32 shutter)
{
	kal_uint32 realtime_fps = 0;

	spin_lock(&imgsensor_drv_lock);

	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;

	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ?
		imgsensor_info.min_shutter : shutter;
	shutter =
		(shutter > (imgsensor_info.max_frame_length -
		imgsensor_info.margin)) ? (imgsensor_info.max_frame_length -
		imgsensor_info.margin) : shutter;

//frame_length and shutter should be an even number.
	shutter = (shutter >> 1) << 1;
	imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
//auroflicker:need to avoid 15fps and 30 fps
	if (imgsensor.autoflicker_en == KAL_TRUE) {
			realtime_fps = imgsensor.pclk /
			imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			realtime_fps = 296;
			set_max_framerate(realtime_fps, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			realtime_fps = 146;
			set_max_framerate(realtime_fps, 0);
		}
	}
		write_cmos_sensor(0xfd, 0x01);
		write_cmos_sensor(0x02, (shutter*2 >> 16) & 0xFF);
		write_cmos_sensor(0x03, (shutter*2 >> 8) & 0xFF);
		write_cmos_sensor(0x04, shutter*2  & 0xFF);
		write_cmos_sensor(0xfd, 0x01);	//page1
		write_cmos_sensor(0x01, 0x01);	//fresh

	LOG_INF("shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}

static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}
static void set_shutter_frame_length(kal_uint16 shutter,
			kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	if (frame_length > 1)
		imgsensor.frame_length=frame_length;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;

	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ?
		imgsensor_info.min_shutter : shutter;
	shutter =
		(shutter > (imgsensor_info.max_frame_length -
		imgsensor_info.margin)) ? (imgsensor_info.max_frame_length -
		imgsensor_info.margin) : shutter;

	//frame_length and shutter should be an even number.
	shutter = (shutter >> 1) << 1;
	imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
//auroflicker:need to avoid 15fps and 30 fps
	if (imgsensor.autoflicker_en) {
			realtime_fps = imgsensor.pclk /
			imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			realtime_fps = 296;
			set_max_framerate(realtime_fps, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			realtime_fps = 146;
			set_max_framerate(realtime_fps, 0);
		}
	}
		write_cmos_sensor(0xfd, 0x01);
		write_cmos_sensor(0x02, (shutter*2 >> 16) & 0xFF);
		write_cmos_sensor(0x03, (shutter*2 >> 8) & 0xFF);
		write_cmos_sensor(0x04, shutter*2  & 0xFF);
		write_cmos_sensor(0xfd, 0x01);	//page1
		write_cmos_sensor(0x01, 0x01);	//fresh

	//LOG_INF("shutter =%d, framelength =%d, realtime_fps =%d\n",
	//	shutter, imgsensor.frame_length, realtime_fps);
}				/* set_shutter_frame_length */




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
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint8  iReg;

	if((gain >= 0x40) && (gain <= (15.5*0x40))) //base gain = 0x40
	{
		iReg = 0x10 * gain/BASEGAIN;        //change mtk gain base to aptina gain base

		if(iReg<=0x10)
		{
			write_cmos_sensor(0xfd, 0x01);
			write_cmos_sensor(0x24, 0x10);//0x23
			write_cmos_sensor(0xfd, 0x01);	//page1
			write_cmos_sensor(0x01, 0x01);	//fresh
			LOG_INF("PARKERA_SHINETECH_FRONT_OV08D10MIPI_SetGain = 16");
		}
		else if(iReg>= 0xf8)//gpw
		{
			write_cmos_sensor(0xfd, 0x01);
			write_cmos_sensor(0x24, 0xf8);
			write_cmos_sensor(0xfd, 0x01);	//page1
			write_cmos_sensor(0x01, 0x01);	//fresh
			LOG_INF("PARKERA_SHINETECH_FRONT_OV08D10MIPI_SetGain = 160");
		}
		else
		{
			write_cmos_sensor(0xfd, 0x01);
			write_cmos_sensor(0x24, (kal_uint8)iReg);
			write_cmos_sensor(0xfd, 0x01);	//page1
			write_cmos_sensor(0x01, 0x01);	//fresh
			LOG_INF("PARKERA_SHINETECH_FRONT_OV08D10MIPI_SetGain = %d",iReg);
		}
	}
	else
		LOG_INF("error gain setting");

	return gain;
}    /*    set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);
}

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le: 0x%x, se: 0x%x, gain: 0x%x\n", le, se, gain);
}

static kal_uint16 parkera_shinetech_ov08d10_init_setting[] = {
	0xfd, 0x00,
	0x20, 0x0e,
	0x20, 0x0b,
	0xfd, 0x00,
	0x1d, 0x00,
	0x1c, 0x19,
	0x11, 0x2a,
	0x14, 0x22,
	0x1b, 0x78,
	0x1e, 0x13,
	0xb7, 0x02,
	0xfd, 0x01,
	0x1a, 0x0a,
	0x1b, 0x08,
	0x2a, 0x01,
	0x2b, 0x9a,
	0xfd, 0x01,
	0x12, 0x00,
	0x03, 0x08,
	0x04, 0xd4,
	0x07, 0x05,
	0x21, 0x02,
	0x24, 0x30,
	0x33, 0x03,
	0x31, 0x06,
	0x33, 0x03,
	0x01, 0x03,
	0x19, 0x10,
	0x42, 0x55,
	0x43, 0x00,
	0x47, 0x07,
	0x48, 0x08,
	0xb2, 0x3f,
	0xb3, 0x5b,
	0xbd, 0x08,
	0xd2, 0x54,
	0xd3, 0x0a,
	0xd4, 0x08,
	0xd5, 0x08,
	0xd6, 0x06,
	0xb1, 0x00,
	0xb4, 0x00,
	0xb7, 0x0a,
	0xbc, 0x44,
	0xbf, 0x48,
	0xc3, 0x24,
	0xc8, 0x03,
	0xe1, 0x33,
	0xe2, 0x33,
	0x51, 0x0c,
	0x52, 0x0a,
	0x57, 0x8c,
	0x59, 0x09,
	0x5a, 0x08,
	0x5e, 0x10,
	0x60, 0x02,
	0x6d, 0x5c,
	0x76, 0x16,
	0x7c, 0x1a,
	0x90, 0x28,
	0x91, 0x16,
	0x92, 0x1c,
	0x93, 0x24,
	0x95, 0x48,
	0x9c, 0x06,
	0xca, 0x0c,
	0xce, 0x0d,
	0xfd, 0x01,
	0xc0, 0x00,
	0xdd, 0x18,
	0xde, 0x19,
	0xdf, 0x32,
	0xe0, 0x70,
	0xfd, 0x01,
	0xc2, 0x05,
	0xd7, 0x88,
	0xd8, 0x77,
	0xd9, 0x00,
	0xfd, 0x07,
	0x00, 0xf8,
	0x01, 0x2b,
	0x05, 0x40,
	0x08, 0x03,
	0x09, 0x08,
	0x28, 0x6f,
	0x2a, 0x20,
	0x2b, 0x05,
	0x2c, 0x01,
	0x50, 0x02,
	0x51, 0x03,
	0x5e, 0x00,
	0x52, 0x00,
	0x53, 0x7c,
	0x54, 0x00,
	0x55, 0x7c,
	0x56, 0x00,
	0x57, 0x7c,
	0x58, 0x00,
	0x59, 0x7c,
	0xfd, 0x02,
	0x9a, 0x00,
	0xa8, 0x02,
	0xfd, 0x02,
	0xa9, 0x04,
	0xaa, 0xd0,
	0xab, 0x06,
	0xac, 0x68,
	0xa1, 0x04,
	0xa2, 0x04,
	0xa3, 0xc8,
	0xa5, 0x04,
	0xa6, 0x06,
	0xa7, 0x60,
	0xfd, 0x05,
	0x06, 0x80,
	0x18, 0x06,
	0x19, 0x68,
	0xfd, 0x00,
	0xa1, 0x01,
	0x24, 0x01,
	0xc0, 0x16,
	0xc1, 0x08,
	0xc2, 0x30,
	0x8e, 0x06,
	0x8f, 0x60,
	0x90, 0x04,
	0x91, 0xc8,
	0x93, 0x0e,
	0x94, 0x77,
	0x95, 0x77,
	0x96, 0x10,
	0x98, 0x88,
	0x9c, 0x1a,
	0xfd, 0x05,
	0x04, 0x40,
	0x07, 0x99,
	0x0D, 0x03,
	0x0F, 0x03,
	0x10, 0x00,
	0x11, 0x00,
	0x12, 0x0C,
	0x13, 0xCF,
	0x14, 0x00,
	0x15, 0x00,
	0xfd, 0x00,
	0x20, 0x0f,
	0xe7, 0x03,
	0xe7, 0x00,
	0xfd, 0x01,

};

static kal_uint16 parkera_shinetech_ov08d10_init_setting_DVT[] = {
	0xfd, 0x00,
	0x20, 0x0e,
	0x20, 0x0b,
	0xfd, 0x00,
	0x1d, 0x00,
	0x1c, 0x19,
	0x11, 0x2a,
	0x14, 0x43,
	0x1e, 0x13,
	0xfd, 0x01,
	0x1a, 0x0a,
	0x1b, 0x08,
	0x2a, 0x01,
	0x2b, 0x9a,
	0xfd, 0x01,
	0x12, 0x00,
	0x03, 0x0a,
	0x04, 0xa0,
	0x05, 0x09,
	0x06, 0x56,
	0x07, 0x05,
	0x21, 0x02,
	0x24, 0x10,
	0x31, 0x06,
	0x33, 0x03,
	0x01, 0x03,
	0x19, 0x10,
	0x42, 0x55,
	0x43, 0x00,
	0x47, 0x07,
	0x48, 0x08,
	0xb2, 0x3f,
	0xb3, 0x5b,
	0xbd, 0x08,
	0xd2, 0x54,
	0xd3, 0x0a,
	0xd4, 0x04,
	0xd5, 0x08,
	0xd6, 0x06,
	0xb1, 0x00,
	0xb4, 0x00,
	0xb7, 0x0a,
	0xbc, 0x44,
	0xbf, 0x48,
	0xc3, 0x24,
	0xc8, 0x03,
	0xe1, 0x33,
	0xe2, 0x33,
	0x51, 0x0d,
	0x52, 0x0d,
	0x57, 0x8c,
	0x59, 0x09,
	0x5a, 0x09,
	0x5e, 0x10,
	0x60, 0x02,
	0x6d, 0x5c,
	0x76, 0x16,
	0x7c, 0x20,
	0x90, 0x28,
	0x91, 0x16,
	0x92, 0x1c,
	0x93, 0x24,
	0x95, 0x48,
	0x9c, 0x06,
	0xca, 0x0c,
	0xce, 0x0d,
	0xfd, 0x01,
	0xc0, 0x00,
	0xdd, 0x18,
	0xde, 0x19,
	0xdf, 0x32,
	0xe0, 0x70,
	0xfd, 0x01,
	0xc2, 0x05,
	0xd7, 0x88,
	0xd8, 0x77,
	0xd9, 0x00,
	0xfd, 0x07,
	0x00, 0xf8,
	0x01, 0x2b,
	0x05, 0x40,
	0x08, 0x03,
	0x09, 0x08,
	0x10, 0x16,
	0x28, 0x6f,
	0x2a, 0x20,
	0x2b, 0x05,
	0x2c, 0x01,
	0x50, 0x02,
	0x51, 0x03,
	0x5e, 0x40,
	0x52, 0x00,
	0x53, 0x7c,
	0x54, 0x00,
	0x55, 0x7c,
	0x56, 0x00,
	0x57, 0x7c,
	0x58, 0x00,
	0x59, 0x7c,
	0xc0, 0x0f,
	0xc1, 0xf8,
	0xc2, 0x0f,
	0xc3, 0xf8,
	0xc4, 0x0f,
	0xc5, 0xf8,
	0xc6, 0x0f,
	0xc7, 0xf8,
	0xfd, 0x02,
	0x9a, 0x00,
	0xfd, 0x02,
	0xa9, 0x04,
	0xaa, 0xd0,
	0xab, 0x06,
	0xac, 0x68,
	0xa1, 0x04,
	0xa2, 0x04,
	0xa3, 0xc8,
	0xa5, 0x04,
	0xa6, 0x06,
	0xa7, 0x60,
	0xfd, 0x00,
	0x24, 0x01,
	0xc0, 0x1e,
	0xc1, 0x08,
	0xc2, 0x00,
	0x8e, 0x06,
	0x8f, 0x60,
	0x90, 0x04,
	0x91, 0xc8,

};

static kal_uint16 parkera_shinetech_ov08d10_preview_setting[] = {
	0xfd, 0x00,
	0x20, 0x0e,
	0x20, 0x0b,
	0xfd, 0x00,
	0x1d, 0x00,
	0x1c, 0x19,
	0x11, 0x2a,
	0x14, 0x22,
	0x1b, 0x78,
	0x1e, 0x13,
	0xb7, 0x02,
	0xfd, 0x01,
	0x1a, 0x0a,
	0x1b, 0x08,
	0x2a, 0x01,
	0x2b, 0x9a,
	0xfd, 0x01,
	0x12, 0x00,
	0x03, 0x08,
	0x04, 0xd4,
	0x07, 0x05,
	0x21, 0x02,
	0x24, 0x30,
	0x33, 0x03,
	0x31, 0x06,
	0x33, 0x03,
	0x01, 0x03,
	0x19, 0x10,
	0x42, 0x55,
	0x43, 0x00,
	0x47, 0x07,
	0x48, 0x08,
	0xb2, 0x7f, //3f,
	0xb3, 0x7b, //5b,
	0xbd, 0x08,
	0xd2, 0x57, //54,
	0xd3, 0x10, //0a,
	0xd4, 0x08,
	0xd5, 0x08,
	0xd6, 0x06,
	0xb1, 0x00,
	0xb4, 0x00,
	0xb7, 0x0a,
	0xbc, 0x44,
	0xbf, 0x48,
	0xc1, 0x10, //add for hband
	0xc3, 0x24,
	0xc8, 0x03,
	0xc9, 0xf8, //add for hband
	0xe1, 0x33,
	0xe2, 0x33,
	0x51, 0x0c,
	0x52, 0x0a,
	0x57, 0x8c,
	0x59, 0x09,
	0x5a, 0x08,
	0x5e, 0x10,
	0x60, 0x02,
	0x6d, 0x5c,
	0x76, 0x16,
	0x7c, 0x1a,
	0x90, 0x28,
	0x91, 0x16,
	0x92, 0x1c,
	0x93, 0x24,
	0x95, 0x48,
	0x9c, 0x06,
	0xca, 0x0c,
	0xce, 0x0d,
	0xfd, 0x01,
	0xc0, 0x00,
	0xdd, 0x18,
	0xde, 0x19,
	0xdf, 0x32,
	0xe0, 0x70,
	0xfd, 0x01,
	0xc2, 0x05,
	0xd7, 0x88,
	0xd8, 0x77,
	0xd9, 0x00,
	0xfd, 0x07,
	0x00, 0xf8,
	0x01, 0x2b,
	0x05, 0x40,
	0x08, 0x03,
	0x09, 0x08,
	0x28, 0x6f,
	0x2a, 0x20,
	0x2b, 0x05,
	0x2c, 0x01,
	0x50, 0x02,
	0x51, 0x03,
	0x5e, 0x10, //00,
	0x52, 0x00,
	0x53, 0x7c,
	0x54, 0x00,
	0x55, 0x7c,
	0x56, 0x00,
	0x57, 0x7c,
	0x58, 0x00,
	0x59, 0x7c,
	0xfd, 0x02,
	0x9a, 0x00,
	0xa8, 0x02,
	0xfd, 0x02,
	0xa9, 0x04,
	0xaa, 0xd0,
	0xab, 0x06,
	0xac, 0x68,
	0xa1, 0x04,
	0xa2, 0x04,
	0xa3, 0xc8,
	0xa5, 0x04,
	0xa6, 0x06,
	0xa7, 0x60,
	0xfd, 0x05,
	0x06, 0x80,
	0x18, 0x06,
	0x19, 0x68,
	0xfd, 0x00,
	0xa1, 0x01,
	0x24, 0x01,
	0xc0, 0x16,
	0xc1, 0x08,
	0xc2, 0x30,
	0x8e, 0x06,
	0x8f, 0x60,
	0x90, 0x04,
	0x91, 0xc8,
	0x93, 0x0e,
	0x94, 0x77,
	0x95, 0x77,
	0x96, 0x10,
	0x98, 0x88,
	0x9c, 0x1a,
	0xfd, 0x05,
	0x04, 0x40,
	0x07, 0x99,
	0x0D, 0x03,
	0x0F, 0x03,
	0x10, 0x00,
	0x11, 0x00,
	0x12, 0x0C,
	0x13, 0xCF,
	0x14, 0x00,
	0x15, 0x00,
	0xfd, 0x00,
	0x20, 0x0f,
	0xe7, 0x03,
	0xe7, 0x00,
	0xfd, 0x01,

};

static kal_uint16 parkera_shinetech_ov08d10_preview_setting_DVT[] = {
	0xfd, 0x00,
	0x20, 0x0e,
	0x20, 0x0b,
	0xfd, 0x00,
	0x1d, 0x00,
	0x1c, 0x19,
	0x11, 0x2a,
	0x14, 0x43,
	0x1e, 0x13,
	0xfd, 0x01,
	0x1a, 0x0a,
	0x1b, 0x08,
	0x2a, 0x01,
	0x2b, 0x9a,
	0xfd, 0x01,
	0x12, 0x00,
	0x03, 0x0a,
	0x04, 0xa0,
	0x05, 0x09,
	0x06, 0x56,
	0x07, 0x05,
	0x21, 0x02,
	0x24, 0x10,
	0x31, 0x06,
	0x33, 0x03,
	0x01, 0x03,
	0x19, 0x10,
	0x42, 0x55,
	0x43, 0x00,
	0x47, 0x07,
	0x48, 0x08,
	0xb2, 0x3f,
	0xb3, 0x5b,
	0xbd, 0x08,
	0xd2, 0x54,
	0xd3, 0x0a,
	0xd4, 0x04,
	0xd5, 0x08,
	0xd6, 0x06,
	0xb1, 0x00,
	0xb4, 0x00,
	0xb7, 0x0a,
	0xbc, 0x44,
	0xbf, 0x48,
	0xc3, 0x24,
	0xc8, 0x03,
	0xe1, 0x33,
	0xe2, 0x33,
	0x51, 0x0d,
	0x52, 0x0d,
	0x57, 0x8c,
	0x59, 0x09,
	0x5a, 0x09,
	0x5e, 0x10,
	0x60, 0x02,
	0x6d, 0x5c,
	0x76, 0x16,
	0x7c, 0x20,
	0x90, 0x28,
	0x91, 0x16,
	0x92, 0x1c,
	0x93, 0x24,
	0x95, 0x48,
	0x9c, 0x06,
	0xca, 0x0c,
	0xce, 0x0d,
	0xfd, 0x01,
	0xc0, 0x00,
	0xdd, 0x18,
	0xde, 0x19,
	0xdf, 0x32,
	0xe0, 0x70,
	0xfd, 0x01,
	0xc2, 0x05,
	0xd7, 0x88,
	0xd8, 0x77,
	0xd9, 0x00,
	0xfd, 0x07,
	0x00, 0xf8,
	0x01, 0x2b,
	0x05, 0x40,
	0x08, 0x03,
	0x09, 0x08,
	0x10, 0x16,
	0x28, 0x6f,
	0x2a, 0x20,
	0x2b, 0x05,
	0x2c, 0x01,
	0x50, 0x02,
	0x51, 0x03,
	0x5e, 0x40,
	0x52, 0x00,
	0x53, 0x7c,
	0x54, 0x00,
	0x55, 0x7c,
	0x56, 0x00,
	0x57, 0x7c,
	0x58, 0x00,
	0x59, 0x7c,
	0xc0, 0x0f,
	0xc1, 0xf8,
	0xc2, 0x0f,
	0xc3, 0xf8,
	0xc4, 0x0f,
	0xc5, 0xf8,
	0xc6, 0x0f,
	0xc7, 0xf8,
	0xfd, 0x02,
	0x9a, 0x00,
	0xfd, 0x02,
	0xa9, 0x04,
	0xaa, 0xd0,
	0xab, 0x06,
	0xac, 0x68,
	0xa1, 0x04,
	0xa2, 0x04,
	0xa3, 0xc8,
	0xa5, 0x04,
	0xa6, 0x06,
	0xa7, 0x60,
	0xfd, 0x00,
	0x24, 0x01,
	0xc0, 0x1e,
	0xc1, 0x08,
	0xc2, 0x00,
	0x8e, 0x06,
	0x8f, 0x60,
	0x90, 0x04,
	0x91, 0xc8,

};

static kal_uint16 parkera_shinetech_ov08d10_capture_setting[] = {
	0xfd, 0x00,
	0x20, 0x0e,
	0x20, 0x0b,
	0xfd, 0x00,
	0x11, 0x2a,
	0x14, 0x43,
	0x1e, 0x13,
	0xb7, 0x02,
	0xfd, 0x01,
	0x12, 0x00,
	0x03, 0x0a,
	0x04, 0xa0,
	0x06, 0xD0,
	0x07, 0x05,
	0x21, 0x02,
	0x24, 0x30,
	0x33, 0x03,
	0x01, 0x01,
	0x19, 0x10,
	0x42, 0x55,
	0x43, 0x00,
	0x47, 0x07,
	0x48, 0x08,
	0xb2, 0x7f, //3f,
	0xb3, 0x7b, //5b,
	0xbd, 0x08,
	0xd2, 0x57, //54,
	0xd3, 0x10, //0a,
	0xd4, 0x08,
	0xd5, 0x08,
	0xd6, 0x06,
	0xb1, 0x00,
	0xb4, 0x00,
	0xb7, 0x0a,
	0xbc, 0x44,
	0xbf, 0x48,
	0xc1, 0x10, //add for hband
	0xc3, 0x24,
	0xc9, 0xf8, //add for hband
	0xc8, 0x03,
	0xe1, 0x33,
	0xe2, 0xbb,
	0x51, 0x0c,
	0x52, 0x0a,
	0x57, 0x8c,
	0x59, 0x09,
	0x5a, 0x08,
	0x5e, 0x10,
	0x60, 0x02,
	0x6d, 0x5c,
	0x76, 0x16,
	0x7c, 0x11,
	0x90, 0x28,
	0x91, 0x16,
	0x92, 0x1c,
	0x93, 0x24,
	0x95, 0x48,
	0x9c, 0x06,
	0xca, 0x0c,
	0xce, 0x0d,
	0xfd, 0x01,
	0xc0, 0x00,
	0xdd, 0x18,
	0xde, 0x19,
	0xdf, 0x32,
	0xe0, 0x70,
	0xfd, 0x01,
	0xc2, 0x05,
	0xd7, 0x88,
	0xd8, 0x77,
	0xd9, 0x00,
	0xfd, 0x07,
	0x00, 0xf8,
	0x01, 0x2b,
	0x05, 0x40,
	0x08, 0x06,
	0x09, 0x11,
	0x28, 0x6f,
	0x2a, 0x20,
	0x2b, 0x05,
	0x5e, 0x10, //40,
	0x52, 0x00,
	0x53, 0x7c,
	0x54, 0x00,
	0x55, 0x7c,
	0x56, 0x00,
	0x57, 0x7c,
	0x58, 0x00,
	0x59, 0x7c,
	0xfd, 0x02,
	0x9a, 0x00,
	0xa8, 0x02,
	0xfd, 0x02,
	0xa1, 0x08,
	0xa2, 0x09,
	0xa3, 0x90,
	0xa5, 0x08,
	0xa6, 0x0c,
	0xa7, 0xc0,
	0xfd, 0x00,
	0x24, 0x01,
	0xc0, 0x16,
	0xc1, 0x08,
	0xc2, 0x30,
	0x8e, 0x0c,
	0x8f, 0xc0,
	0x90, 0x09,
	0x91, 0x90,
	0xfd, 0x05,
	0x04, 0x40,
	0x07, 0x00,
	0x0D, 0x01,
	0x0F, 0x01,
	0x10, 0x00,
	0x11, 0x00,
	0x12, 0x0C,
	0x13, 0xCF,
	0x14, 0x00,
	0x15, 0x00,
	0xfd, 0x00,
	0x20, 0x0f,
	0xe7, 0x03,
	0xe7, 0x00,
	0xfd, 0x01,

};

static kal_uint16 parkera_shinetech_ov08d10_capture_setting_DVT[] = {
	0xfd, 0x00,
	0x20, 0x0e,
	0x20, 0x0b,
	0xfd, 0x00,
	0x11, 0x2a,
	0x14, 0x43,
	0x1e, 0x13,
	0xfd, 0x01,
	0x12, 0x00,
	0x03, 0x0a,
	0x04, 0xa0,
	0x06, 0xD0,
	0x07, 0x05,
	0x21, 0x02,
	0x24, 0x10,
	0x01, 0x01,
	0x19, 0x10,
	0x42, 0x55,
	0x43, 0x00,
	0x47, 0x07,
	0x48, 0x08,
	0xb2, 0x3f,
	0xb3, 0x5b,
	0xbd, 0x08,
	0xd2, 0x54,
	0xd3, 0x0a,
	0xd4, 0x04,
	0xd5, 0x08,
	0xd6, 0x06,
	0xb1, 0x00,
	0xb4, 0x00,
	0xb7, 0x0a,
	0xbc, 0x44,
	0xbf, 0x48,
	0xc3, 0x24,
	0xc8, 0x03,
	0xe1, 0x33,
	0xe2, 0x33,
	0x51, 0x0d,
	0x52, 0x0d,
	0x57, 0x8c,
	0x59, 0x09,
	0x5a, 0x09,
	0x5e, 0x10,
	0x60, 0x02,
	0x6d, 0x5c,
	0x76, 0x16,
	0x7c, 0x11,
	0x90, 0x28,
	0x91, 0x16,
	0x92, 0x1c,
	0x93, 0x24,
	0x95, 0x48,
	0x9c, 0x06,
	0xca, 0x0c,
	0xce, 0x0d,
	0xfd, 0x01,
	0xc0, 0x00,
	0xdd, 0x18,
	0xde, 0x19,
	0xdf, 0x32,
	0xe0, 0x70,
	0xfd, 0x01,
	0xc2, 0x05,
	0xd7, 0x88,
	0xd8, 0x77,
	0xd9, 0x00,
	0xfd, 0x07,
	0x00, 0xf8,
	0x01, 0x2b,
	0x05, 0x40,
	0x08, 0x06,
	0x09, 0x11,
	0x10, 0x16,
	0x28, 0x6f,
	0x2a, 0x20,
	0x2b, 0x05,
	0x5e, 0x40,
	0x52, 0x00,
	0x53, 0x7c,
	0x54, 0x00,
	0x55, 0x7c,
	0x56, 0x00,
	0x57, 0x7c,
	0x58, 0x00,
	0x59, 0x7c,
	0xc0, 0x0f,
	0xc1, 0xf8,
	0xc2, 0x0f,
	0xc3, 0xf8,
	0xc4, 0x0f,
	0xc5, 0xf8,
	0xc6, 0x0f,
	0xc7, 0xf8,
	0xfd, 0x02,
	0x9a, 0x00,
	0xfd, 0x02,
	0xa1, 0x08,
	0xa2, 0x09,
	0xa3, 0x90,
	0xa5, 0x08,
	0xa6, 0x0c,
	0xa7, 0xc0,
	0xfd, 0x00,
	0x24, 0x01,
	0xc0, 0x1e,
	0xc1, 0x08,
	0xc2, 0x00,
	0x8e, 0x0c,
	0x8f, 0xc0,
	0x90, 0x09,
	0x91, 0x90,

};

static kal_uint16 parkera_shinetech_ov08d10_normal_video_setting[] = {
	0xfd, 0x00,
	0x20, 0x0e,
	0x20, 0x0b,
	0xfd, 0x00,
	0x11, 0x2a,
	0x14, 0x43,
	0x1e, 0x13,
	0xb7, 0x02,
	0xfd, 0x01,
	0x12, 0x00,
	0x03, 0x0a,
	0x04, 0xa0,
	0x06, 0xD0,
	0x07, 0x05,
	0x21, 0x02,
	0x24, 0x30,
	0x33, 0x03,
	0x01, 0x01,
	0x19, 0x10,
	0x42, 0x55,
	0x43, 0x00,
	0x47, 0x07,
	0x48, 0x08,
	0xb2, 0x7f, //3f,
	0xb3, 0x7b, //5b,
	0xbd, 0x08,
	0xd2, 0x57, //54,
	0xd3, 0x10, //0a,
	0xd4, 0x08,
	0xd5, 0x08,
	0xd6, 0x06,
	0xb1, 0x00,
	0xb4, 0x00,
	0xb7, 0x0a,
	0xbc, 0x44,
	0xbf, 0x48,
	0xc1, 0x10, //add for hband
	0xc3, 0x24,
	0xc8, 0x03,
	0xc9, 0xf8, //add for hband
	0xe1, 0x33,
	0xe2, 0xbb,
	0x51, 0x0c,
	0x52, 0x0a,
	0x57, 0x8c,
	0x59, 0x09,
	0x5a, 0x08,
	0x5e, 0x10,
	0x60, 0x02,
	0x6d, 0x5c,
	0x76, 0x16,
	0x7c, 0x11,
	0x90, 0x28,
	0x91, 0x16,
	0x92, 0x1c,
	0x93, 0x24,
	0x95, 0x48,
	0x9c, 0x06,
	0xca, 0x0c,
	0xce, 0x0d,
	0xfd, 0x01,
	0xc0, 0x00,
	0xdd, 0x18,
	0xde, 0x19,
	0xdf, 0x32,
	0xe0, 0x70,
	0xfd, 0x01,
	0xc2, 0x05,
	0xd7, 0x88,
	0xd8, 0x77,
	0xd9, 0x00,
	0xfd, 0x07,
	0x00, 0xf8,
	0x01, 0x2b,
	0x05, 0x40,
	0x08, 0x06,
	0x09, 0x11,
	0x28, 0x6f,
	0x2a, 0x20,
	0x2b, 0x05,
	0x5e, 0x10, //40,
	0x52, 0x00,
	0x53, 0x7c,
	0x54, 0x00,
	0x55, 0x7c,
	0x56, 0x00,
	0x57, 0x7c,
	0x58, 0x00,
	0x59, 0x7c,
	0xfd, 0x02,
	0x9a, 0x00,
	0xa8, 0x02,
	0xfd, 0x02,
	0xa0, 0x01,
	0xa1, 0x38,
	0xa2, 0x07,
	0xa3, 0x2c,
	0xa5, 0x08,
	0xa6, 0x0c,
	0xa7, 0xc0,
	0xfd, 0x00,
	0x24, 0x01,
	0xc0, 0x16,
	0xc1, 0x08,
	0xc2, 0x30,
	0x8e, 0x0c,
	0x8f, 0xc0,
	0x90, 0x07,
	0x91, 0x2c,
	0xfd, 0x05,
	0x04, 0x40,
	0x07, 0x00,
	0x0D, 0x01,
	0x0F, 0x01,
	0x10, 0x00,
	0x11, 0x00,
	0x12, 0x0C,
	0x13, 0xCF,
	0x14, 0x00,
	0x15, 0x00,
	0xfd, 0x00,
	0x20, 0x0f,
	0xe7, 0x03,
	0xe7, 0x00,
	0xfd, 0x01,

};

static kal_uint16 parkera_shinetech_ov08d10_normal_video_setting_DVT[] = {
	0xfd, 0x00,
	0x20, 0x0e,
	0x20, 0x0b,
	0xfd, 0x00,
	0x11, 0x2a,
	0x14, 0x43,
	0x1e, 0x13,
	0xfd, 0x01,
	0x12, 0x00,
	0x03, 0x0a,
	0x04, 0xa0,
	0x06, 0xD0,
	0x07, 0x05,
	0x21, 0x02,
	0x24, 0x10,
	0x01, 0x01,
	0x19, 0x10,
	0x42, 0x55,
	0x43, 0x00,
	0x47, 0x07,
	0x48, 0x08,
	0xb2, 0x3f,
	0xb3, 0x5b,
	0xbd, 0x08,
	0xd2, 0x54,
	0xd3, 0x0a,
	0xd4, 0x04,
	0xd5, 0x08,
	0xd6, 0x06,
	0xb1, 0x00,
	0xb4, 0x00,
	0xb7, 0x0a,
	0xbc, 0x44,
	0xbf, 0x48,
	0xc3, 0x24,
	0xc8, 0x03,
	0xe1, 0x33,
	0xe2, 0x33,
	0x51, 0x0d,
	0x52, 0x0d,
	0x57, 0x8c,
	0x59, 0x09,
	0x5a, 0x09,
	0x5e, 0x10,
	0x60, 0x02,
	0x6d, 0x5c,
	0x76, 0x16,
	0x7c, 0x11,
	0x90, 0x28,
	0x91, 0x16,
	0x92, 0x1c,
	0x93, 0x24,
	0x95, 0x48,
	0x9c, 0x06,
	0xca, 0x0c,
	0xce, 0x0d,
	0xfd, 0x01,
	0xc0, 0x00,
	0xdd, 0x18,
	0xde, 0x19,
	0xdf, 0x32,
	0xe0, 0x70,
	0xfd, 0x01,
	0xc2, 0x05,
	0xd7, 0x88,
	0xd8, 0x77,
	0xd9, 0x00,
	0xfd, 0x07,
	0x00, 0xf8,
	0x01, 0x2b,
	0x05, 0x40,
	0x08, 0x06,
	0x09, 0x11,
	0x10, 0x16,
	0x28, 0x6f,
	0x2a, 0x20,
	0x2b, 0x05,
	0x5e, 0x40,
	0x52, 0x00,
	0x53, 0x7c,
	0x54, 0x00,
	0x55, 0x7c,
	0x56, 0x00,
	0x57, 0x7c,
	0x58, 0x00,
	0x59, 0x7c,
	0xc0, 0x0f,
	0xc1, 0xf8,
	0xc2, 0x0f,
	0xc3, 0xf8,
	0xc4, 0x0f,
	0xc5, 0xf8,
	0xc6, 0x0f,
	0xc7, 0xf8,
	0xfd, 0x02,
	0x9a, 0x00,
	0xfd, 0x02,
	0xa0, 0x01,
	0xa1, 0x38,
	0xa2, 0x07,
	0xa3, 0x2c,
	0xa5, 0x08,
	0xa6, 0x0c,
	0xa7, 0xc0,
	0xfd, 0x00,
	0x24, 0x01,
	0xc0, 0x1e,
	0xc1, 0x08,
	0xc2, 0x00,
	0x8e, 0x0c,
	0x8f, 0xc0,
	0x90, 0x07,
	0x91, 0x2c,

};


static void sensor_init(void)
{
	if (DVT) {
		table_write_cmos_sensor(
			parkera_shinetech_ov08d10_init_setting_DVT,
			sizeof(parkera_shinetech_ov08d10_init_setting_DVT) / sizeof(kal_uint16));
	} else {
		table_write_cmos_sensor(
			parkera_shinetech_ov08d10_init_setting,
			sizeof(parkera_shinetech_ov08d10_init_setting) / sizeof(kal_uint16));
	}
}

static void preview_setting(void)
{
	if (DVT) {
		table_write_cmos_sensor(
			parkera_shinetech_ov08d10_preview_setting_DVT,
			sizeof(parkera_shinetech_ov08d10_preview_setting_DVT) / sizeof(kal_uint16));
	} else {
		table_write_cmos_sensor(
			parkera_shinetech_ov08d10_preview_setting,
			sizeof(parkera_shinetech_ov08d10_preview_setting) / sizeof(kal_uint16));
	}
}

static void capture_setting(void)
{
	if (DVT) {
		table_write_cmos_sensor(
			parkera_shinetech_ov08d10_capture_setting_DVT,
			sizeof(parkera_shinetech_ov08d10_capture_setting_DVT) / sizeof(kal_uint16));
	} else {
		table_write_cmos_sensor(
			parkera_shinetech_ov08d10_capture_setting,
			sizeof(parkera_shinetech_ov08d10_capture_setting) / sizeof(kal_uint16));
	}
}

static void normal_video_setting(void)
{
	if (DVT) {
		table_write_cmos_sensor(
			parkera_shinetech_ov08d10_normal_video_setting_DVT,
			sizeof(parkera_shinetech_ov08d10_normal_video_setting_DVT) / sizeof(kal_uint16));
	} else {
		table_write_cmos_sensor(
			parkera_shinetech_ov08d10_normal_video_setting,
			sizeof(parkera_shinetech_ov08d10_normal_video_setting) / sizeof(kal_uint16));
	}
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
		*sensor_id = return_sensor_id();
		if (*sensor_id == imgsensor_info.sensor_id) {
			LOG_INF("PARKERA_SHINETECH_FRONT_OV08D10 get_imgsensor_id success: 0x%x\n", *sensor_id);
			return ERROR_NONE;
		}
		retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		printk("PARKERA_SHINETECH_FRONT_OV08D10 get_imgsensor_id failed: 0x%x\n", *sensor_id);
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	return ERROR_NONE;
}

static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;
	LOG_INF("mazhuang test enter");

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("PARKERA_SHINETECH_FRONT_OV08D10 open success: 0x%x\n", sensor_id);
				break;
			}
			retry--;
		} while(retry > 0);
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

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}    /*    open  */

static kal_uint32 close(void)
{
	LOG_INF("E\n");
	streaming_control(KAL_FALSE);
	/*No Need to implement this function*/
	return ERROR_NONE;
}	/*	close  */

static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_info("[parkera_shinetech_front_ov08d10] preview mode start\n");

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
}    /*    preview   */

static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_info("[parkera_shinetech_front_ov08d10] capture mode start\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate)	{
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);

	capture_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}    /* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_info("[parkera_shinetech_front_ov08d10] normal_video mode start\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	normal_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}    /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_info("[parkera_shinetech_front_ov08d10] hs_video mode start\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	//hs_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}
static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_info("[parkera_shinetech_front_ov08d10] slim_video mode start\n");

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

	//slim_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}
static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_info("[parkera_shinetech_front_ov08d10] custom1 mode start\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	//custom1_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}
static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
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
	sensor_resolution->SensorCustom1Width = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;
	return ERROR_NONE;
}    /*    get_resolution    */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
		MSDK_SENSOR_INFO_STRUCT *sensor_info,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet */
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
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;/*add custom1*/

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	sensor_info->FrameTimeDelayFrame =
		imgsensor_info.frame_time_delay_frame;
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
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

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;    // 0 is default 1x
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
	    sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

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
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
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
	default:
		//LOG_INF("[odin]default mode\n");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}    /* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
	/*This Function not used after ROME*/
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	/***********
	 *if (framerate == 0)	 //Dynamic frame rate
	 *	return ERROR_NONE;
	 *spin_lock(&imgsensor_drv_lock);
	 *if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
	 *	imgsensor.current_fps = 296;
	 *else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
	 *	imgsensor.current_fps = 146;
	 *else
	 *	imgsensor.current_fps = framerate;
	 *spin_unlock(&imgsensor_drv_lock);
	 *set_max_framerate(imgsensor.current_fps, 1);
	 ********/
	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
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

//	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate * 10 /
			imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ?
			(frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ?
				(frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
				LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
					framerate, imgsensor_info.cap.max_framerate / 10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
				(frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ?
			(frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
			(frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		if (imgsensor.current_fps != imgsensor_info.custom1.max_framerate)
		LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.custom1.max_framerate/10);
		frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ?
			(frame_length - imgsensor_info.custom1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
		set_dummy();
	break;
	default:  //coding with  preview scenario by default
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
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
	default:
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if(enable) {
		LOG_INF("ov08d10 enter pattern_mode enable");
		write_cmos_sensor(0xfd,0x01);
		write_cmos_sensor(0x21,0x00);
		write_cmos_sensor(0x22,0x00);
		write_cmos_sensor(0x01,0x01);
		write_cmos_sensor(0xfd,0x07);
		write_cmos_sensor(0x05,0x00);
	} else {
		write_cmos_sensor(0xfd,0x01);
		write_cmos_sensor(0x21,0x02);
		write_cmos_sensor(0x22,0x00);
		write_cmos_sensor(0x01,0x01);
		write_cmos_sensor(0xfd,0x07);
		write_cmos_sensor(0x05,0x40);
	}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_control enable =%d\n", enable);
	if (DVT) {
		if (enable) {
			write_cmos_sensor(0xfd, 0x00);
			write_cmos_sensor(0xa0, 0x01);
			write_cmos_sensor(0xfd, 0x00);
			write_cmos_sensor(0x20, 0x0f);
			write_cmos_sensor(0xe7, 0x03);
			write_cmos_sensor(0xe7, 0x00);
			write_cmos_sensor(0xfd, 0x01);

		} else {
			write_cmos_sensor(0xfd, 0x00);
			write_cmos_sensor(0xa0, 0x00);
		}
	} else {
		if (enable) {
			write_cmos_sensor(0xfd, 0x00);
			write_cmos_sensor(0xa0, 0x01);
			write_cmos_sensor(0xfd, 0x01);
		} else {
			write_cmos_sensor(0xfd, 0x00);
			write_cmos_sensor(0xa0, 0x00);
			write_cmos_sensor(0xfd, 0x01);
		}
	}
	mdelay(10);

	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
		UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *)feature_para;
	UINT16 *feature_data_16 = (UINT16 *)feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *)feature_para;
	UINT32 *feature_data_32 = (UINT32 *)feature_para;
	//UINT8 feature_data_8[17] = {0};
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *)feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
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
		*feature_para_len=4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
	    *feature_return_para_32 = imgsensor.pclk;
	    *feature_para_len = 4;
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
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				rate = imgsensor_info.slim_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CUSTOM1:
				rate = imgsensor_info.custom1.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				rate = imgsensor_info.pre.mipi_pixel_rate;
				break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
		}
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		//night_mode((BOOL)*feature_data);
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
		LOG_INF("adb_i2c_read 0x%x = 0x%x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
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
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len=4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
	    LOG_INF("current fps :%d\n", (UINT32)*feature_data);
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.current_fps = *feature_data;
	    spin_unlock(&imgsensor_drv_lock);
	break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) *feature_data,
			(UINT16) *(feature_data + 1));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_SET_HDR:
	    LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.ihdr_en = (BOOL)*feature_data;
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
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[5],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE = %d, SE = %d, Gain = %d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data + 1), (UINT16)*(feature_data + 2));
		ihdr_write_shutter_gain((UINT16)*feature_data, (UINT16)*(feature_data + 1),
			(UINT16)*(feature_data + 2));
		break;
	/*case SENSOR_FEATURE_GET_SERIANO_IC:
		read_parkera_shinetech_front_ov08d10_QRSerialdata(feature_data_8, feature_para_len);
		memcpy(feature_para,feature_data_8,*feature_para_len);
		break;*/
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

UINT32 PARKERA_SHINETECH_FRONT_OV08D10_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	int pcb_version = get_PCB_Version();

	if (pcb_version >= 7 && pcb_version <= 9) // enum PCB_VERSION
		DVT = 1;

	//TODO: need to comment this line once the above API return correct value
	DVT = 1; //fixing to 1, currently not getting right PCB_Version

	if (DVT)
		imgsensor_info = imgsensor_info_DVT;

	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
