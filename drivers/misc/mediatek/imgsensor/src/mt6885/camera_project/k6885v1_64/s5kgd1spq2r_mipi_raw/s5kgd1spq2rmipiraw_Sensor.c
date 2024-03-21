/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 S5K3P9SPmipi_Sensor.c
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
//#include "../imgsensor_i2c.h"
#include "s5kgd1spq2rmipiraw_Sensor.h"

#ifdef CONFIG_MTK_CAM_SECURITY_SUPPORT
#include "imgsensor_ca.h"
#endif


#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define DEVICE_VERSION_S5KGD1SPQ2R     "s5kgd1spq2r"
//extern void register_imgsensor_deviceinfo(char *name, char *version, u8 module_id);
static kal_uint8 deviceInfo_register_value = 0x00;
static kal_uint32 cameraVgaFlag = 0;
#endif
static kal_uint32 streaming_control(kal_bool enable);
#define MODULE_ID_OFFSET 0x0000
#define PFX "S5KGD1SPQ2R_camera_sensor"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length, u16 timing);


static DEFINE_SPINLOCK(imgsensor_drv_lock);


static struct imgsensor_info_struct imgsensor_info = {
		.sensor_id = S5KGD1SPQ2R_SENSOR_ID,
		#ifdef OPLUS_FEATURE_CAMERA_COMMON
		.module_id = 0x04,	//0x01 Sunny,0x05 QTEK
		#endif
		.checksum_value = 0x70dbcd61,
		.pre = {
			.pclk = 1144000000,
			.linelength = 10656,
			.framelength = 3572,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 3264,
			.grabwindow_height = 2448,
			.mipi_data_lp2hs_settle_dc = 85,
			/*	 following for GetDefaultFramerateByScenario()	*/
			.mipi_pixel_rate = 398400000,
			.max_framerate = 300,
		},
		.cap = { // keep same with prv
			.pclk = 1144000000,
			.linelength = 10656,
			.framelength = 3572,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 3264,
			.grabwindow_height = 2448,
			.mipi_data_lp2hs_settle_dc = 85,
			/*	 following for GetDefaultFramerateByScenario()	*/
			.mipi_pixel_rate = 398400000,
			.max_framerate = 300,
		},
		.normal_video = {
			.pclk = 1144000000,
			.linelength = 10656,
			.framelength = 3572,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 3264,
			.grabwindow_height = 2448,
			.mipi_data_lp2hs_settle_dc = 85,
			/*	 following for GetDefaultFramerateByScenario()	*/
			.mipi_pixel_rate = 398400000,
			.max_framerate = 300,
		},
		.hs_video = {
			.pclk = 1144000000,
			.linelength = 10656,
			.framelength = 3572,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 3264,
			.grabwindow_height = 2448,
			.mipi_data_lp2hs_settle_dc = 85,
			/*	 following for GetDefaultFramerateByScenario()	*/
			.mipi_pixel_rate = 398400000,
			.max_framerate = 300,
		},
		.slim_video = {
			.pclk = 1144000000,
			.linelength = 14528,
			.framelength = 2624,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 3264,
			.grabwindow_height = 1840,
			.mipi_data_lp2hs_settle_dc = 85,
			/*	 following for GetDefaultFramerateByScenario()	*/
			.mipi_pixel_rate = 302400000,
			.max_framerate = 300,
		},
		.custom1 = { /*reg_A-1 1280x720 @30fps for android*/
			.pclk = 1144000000,
			.linelength = 14528,
			.framelength = 2624,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 2304,
			.grabwindow_height = 1728,
			.mipi_data_lp2hs_settle_dc = 85,
			/* following for GetDefaultFramerateByScenario() */
			.mipi_pixel_rate = 302400000,
			.max_framerate = 300, /* 30fps */
		},
		.custom2 = { /*clone from custom1 -- just to fill custom2 blank, who want use must specify*/
			.pclk = 1144000000,
			.linelength = 14528,
			.framelength = 2624,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 2304,
			.grabwindow_height = 1728,
			.mipi_data_lp2hs_settle_dc = 85,
			/* following for GetDefaultFramerateByScenario() */
			.mipi_pixel_rate = 302400000,
			.max_framerate = 300, /* 30fps */
		},
		.custom3 = { //for remosaic
			.pclk = 1144000000,
			.linelength = 7344,
			.framelength = 5192,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 6560,
			.grabwindow_height = 4928,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 1161600000,
			.max_framerate = 300,
		},

		.margin = 3,
		.min_shutter = 5,
		.min_gain = 64, /*1x gain*/
		.max_gain = 1024, /*16x gain*/
		.min_gain_iso = 100,
		.gain_step = 32,
		.gain_type = 2,
		.max_frame_length = 0xffff-5,
		.ae_shut_delay_frame = 0,
		.ae_sensor_gain_delay_frame = 0,
		.ae_ispGain_delay_frame = 2,
		.ihdr_support = 0,   /*1, support; 0,not support*/
		.ihdr_le_firstline = 0,  /*1,le first; 0, se first*/
		.sensor_mode_num = 8,    /*support sensor mode num*/

		.cap_delay_frame = 2,  /*3 guanjd modify for cts*/
		.pre_delay_frame = 2,  /*3 guanjd modify for cts*/
		.video_delay_frame = 3,
		.hs_video_delay_frame = 3,
		.slim_video_delay_frame = 3,
		.custom1_delay_frame = 2,
		.custom2_delay_frame = 2,
		.custom3_delay_frame = 2,

		.isp_driving_current = ISP_DRIVING_4MA,
		.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
		.mipi_sensor_type = MIPI_OPHY_NCSI2,  /*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
		.mipi_settle_delay_mode = 1,  /*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,//3p9sp remosaic awb  by wuxiao 20180822
		.mclk = 24,
		.mipi_lane_num = SENSOR_MIPI_4_LANE,
		.i2c_addr_table = {0x20, 0xff},
		.i2c_speed = 400,
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, /*IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video*/
	.shutter = 0x3D0,					/*current shutter*/
	.gain = 0x100,						/*current gain*/
	.dummy_pixel = 0,					/*current dummypixel*/
	.dummy_line = 0,					/*current dummyline*/
	.current_fps = 0,  /*full size current fps : 24fps for PIP, 30fps for Normal or ZSD*/
	.autoflicker_en = KAL_FALSE,  /*auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker*/
	.test_pattern = KAL_FALSE,		/*test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output*/
	.enable_secure = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/*current scenario id*/
	.ihdr_mode = 0, /*sensor need support LE, SE with HDR feature*/
	.i2c_write_id = 0x20,

};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[8] =
{
	{ 6560, 4928,  0,   0, 6560, 4928, 3280, 2464, 0, 0, 3280, 2464, 8, 8, 3264, 2448}, /*Preview*/
	{ 6560, 4928,  0,   0, 6560, 4928, 3280, 2464, 0, 0, 3280, 2464, 8, 8, 3264, 2448}, // capture
	{ 6560, 4928,  0,   0, 6560, 4928, 3280, 2464, 0, 0, 3280, 2464, 8, 8, 3264, 2448}, /*video*/
	{ 6560, 4928,  0,   0, 6560, 4928, 3280, 2464, 0, 0, 3280, 2464, 8, 8, 3264, 2448}, /*hs_video,don't use*/
	{ 6560, 4928, 16, 624, 6520, 3696, 3260, 1848, 0, 4, 3280, 1840, 0, 0, 3264, 1840}, /* slim video*/
	{ 6560, 4928,976, 736, 4608, 3456, 2304, 1728, 0, 0, 2304, 1728, 0, 0, 2304, 1728}, /* custom1 */
	{ 6560, 4928,976, 736, 4608, 3456, 2304, 1728, 0, 0, 2304, 1728, 0, 0, 2304, 1728}, /* custom2 */
	{ 6560, 4928,  0,   0, 6560, 4928, 6560, 4928, 0, 0, 6560, 4928, 0, 0, 6560, 4928}, // remosic
}; /*cpy from preview*/


/*no mirror flip*/

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
/*extern void kdSetI2CSpeed(u16 i2cSpeed);*/
/*extern bool read_2l9_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size);*/
/****hope  add for CameraEM otp errorcode****/
/*
extern int S5K3P9SP_otp_read(void);
static int vivo_otp_read_when_power_on;
extern otp_error_code_t S5K3P9SP_OTP_ERROR_CODE;
MUINT32  sn_inf_sub_S5K3P9SP[13];
*/
/*0 flag   1-12 data*/
/****hope add end****/

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static kal_uint16 read_module_id(void)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(MODULE_ID_OFFSET >> 8) , (char)(MODULE_ID_OFFSET & 0xFF) };
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,0xA8/*EEPROM_READ_ID*/);
	return get_byte;

}
#endif

static kal_uint16 read_cmos_sensor_16_16(kal_uint32 addr)
{
	kal_uint16 get_byte= 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};
	/*kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
	iReadRegI2C(pusendcmd, 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte << 8) & 0xff00) | ((get_byte >> 8) & 0x00ff);
}


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
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
	 /* kdSetI2CSpeed(imgsensor_info.i2c_speed);Add this func to set i2c speed by each sensor*/
	iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}

#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 225	/* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 4

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
			puSendCmd[tosend++] = (char)(data >> 8);
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
		#if MULTI_WRITE
		/* Write when remain buffer size is less than 4 bytes or reach end of data */
		if ((I2C_BUFFER_LEN - tosend) < 4 || IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id,
								4, imgsensor_info.i2c_speed);
			tosend = 0;
		}
		#else
		iWriteRegI2CTiming(puSendCmd, 4, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
		tosend = 0;

		#endif
	}
	return 0;
}



static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
	write_cmos_sensor_16_16(0x0342, imgsensor.line_length);
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d \n", framerate,min_framelength_en);

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

static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;

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
			/* Extend frame length*/
			write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
		}
	} else {
		/* Extend frame length*/
		write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
	}

	/* Update Shutter*/
	write_cmos_sensor_16_16(0x0202, shutter);
	LOG_INF("shutter = %d, framelength = %d\n", shutter,imgsensor.frame_length);

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
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */

/*	write_shutter  */
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	if (frame_length > 1) {
		dummy_line = frame_length - imgsensor.frame_length;
	}
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;


	if (shutter > imgsensor.frame_length - imgsensor_info.margin) {
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	}

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}

	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			/* Extend frame length */
			write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	/* Update Shutter */
	write_cmos_sensor_16_16(0x0202, shutter & 0xFFFF);

	LOG_INF("shutter = %d, framelength = %d/%d, dummy_line= %d\n", shutter, imgsensor.frame_length,
		frame_length, dummy_line);

}/*      write_shutter  */


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

	/*gain= 1024;for test*/
	/*return; for test*/

	if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
		pr_debug("Error gain setting");

		if (gain < imgsensor_info.min_gain)
			gain = imgsensor_info.min_gain;
		else if (gain > imgsensor_info.max_gain)
			gain = imgsensor_info.max_gain;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_16_16(0x0204,reg_gain);
	/*write_cmos_sensor_16_8(0x0204,(reg_gain>>8));*/
	/*write_cmos_sensor_16_8(0x0205,(reg_gain&0xff));*/
	/*Add for VGA voltage set low*/
	if (cameraVgaFlag > 0) {
		if (cameraVgaFlag == 5) {
			write_cmos_sensor_16_16(0x6028, 0x4000);
			write_cmos_sensor_16_16(0xF44A, 0x000E);
		}
		cameraVgaFlag -- ;
	}


	return gain;
}	/*	set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
	switch (image_mirror) {

		case IMAGE_NORMAL:
			write_cmos_sensor_16_8(0x0101, 0x00);   /* Gr*/
			break;

		case IMAGE_H_MIRROR:
			write_cmos_sensor_16_8(0x0101, 0x01);
			break;

		case IMAGE_V_MIRROR:
			write_cmos_sensor_16_8(0x0101, 0x02);
			break;

		case IMAGE_HV_MIRROR:
			write_cmos_sensor_16_8(0x0101, 0x03);  /*Gb*/
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
#if 0
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/
#endif

static kal_uint32 streaming_control(kal_bool enable)
{
	int timeout = (10000 / imgsensor.current_fps) + 1;
	int i = 0;
	int framecnt = 0;

	LOG_INF("streaming_enable(0= Sw Standby,1= streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor_16_8(0x0100, 0x01);
		mDELAY(10);
		cameraVgaFlag = 10;
	} else {
		write_cmos_sensor_16_8(0x0100, 0x00);
		for (i = 0; i < timeout; i++) {
			mDELAY(5);
			framecnt = read_cmos_sensor_16_8(0x0005);
			if (framecnt == 0xFF) {
				LOG_INF(" Stream Off OK at i=%d.\n", i);
				return ERROR_NONE;
			}
		}
		LOG_INF("Stream Off Fail! framecnt= %d.\n", framecnt);
		cameraVgaFlag = 0;
	}
	LOG_INF("After streaming_enable: %d\n", enable);
	return ERROR_NONE;
}

static const u16 uTnpArrayInit[] = {
0x126F,
0x0000,
0x0000,
0x4905,
0x4804,
0x4A05,
0xF8C1,
0x06D0,
0x1A10,
0xF8A1,
0x06D4,
0xF000,
0xBA59,
0x0120,
0x8058,
0x0020,
0xC068,
0x0120,
0x00F8,
0xE92D,
0x4FFF,
0x4604,
0x7808,
0xB085,
0x4692,
0x460F,
0x2800,
0xD06F,
0xF000,
0xFAAB,
0x2800,
0xD06B,
0x2600,
0xF000,
0xFAA6,
0x2803,
0xD069,
0xF000,
0xFAA7,
0x0A05,
0xFAB6,
0xF086,
0xF1C0,
0x0220,
0x2A09,
0xD200,
0x2209,
0x3A09,
0x40D6,
0xFAB5,
0xF085,
0xF1C0,
0x0020,
0xB2F6,
0x2809,
0xD200,
0x2009,
0x3809,
0x40C5,
0xEB00,
0x0880,
0x9808,
0xF005,
0x09FF,
0xEB00,
0x03C0,
0xEB03,
0x1300,
0x2100,
0x46EB,
0xEB07,
0x0CC3,
0xEB01,
0x00C1,
0xEB00,
0x1001,
0xEB0C,
0x0040,
0xEB00,
0x0048,
0x1C85,
0x1C53,
0x300C,
0xF935,
0x7013,
0xF930,
0x3013,
0xF935,
0x5012,
0xF930,
0x0012,
0x1B7F,
0x1A1B,
0x4377,
0x4373,
0x3780,
0x3380,
0xEB05,
0x2527,
0xEB00,
0x2023,
0x1B40,
0xFB00,
0xF009,
0x3080,
0xEB05,
0x2020,
0xF84B,
0x0021,
0xFA00,
0xF00A,
0xF84B,
0x0021,
0x1C49,
0x2904,
0xD3D3,
0xF8D4,
0x0100,
0x9900,
0x4408,
0xF8C4,
0x0100,
0xF8D4,
0x0104,
0x9901,
0x4408,
0xF8C4,
0x0104,
0xF8D4,
0x0108,
0x9902,
0x4408,
0xF8C4,
0x0108,
0xF8D4,
0x010C,
0x9903,
0x4408,
0xF8C4,
0x010C,
0xB009,
0xE8BD,
0x8FF0,
0xF000,
0xFA42,
0x0A05,
0xF000,
0xFA44,
0x0A06,
0xE791,
0xE92D,
0x43F8,
0x461C,
0x9300,
0x460E,
0x4615,
0x4613,
0x4607,
0x2200,
0x49FD,
0xF000,
0xFA3B,
0x49FC,
0x462B,
0x2204,
0x3118,
0x4638,
0x9400,
0xF000,
0xFA33,
0x49F8,
0x462B,
0x2208,
0x3130,
0x4638,
0x9400,
0xF000,
0xFA2B,
0xF896,
0x025A,
0x2800,
0xD069,
0xF000,
0xFA11,
0x2800,
0xD065,
0x4DF1,
0xF04F,
0x0800,
0xF000,
0xFA0A,
0x2803,
0xD060,
0x2309,
0x2210,
0x6829,
0x6A68,
0xF000,
0xFA1B,
0x0A07,
0xFAB7,
0xF087,
0xF1C0,
0x0120,
0x2909,
0xD200,
0x2109,
0xFAB8,
0xF088,
0xF1C0,
0x0520,
0x3909,
0x2D09,
0xD200,
0x2509,
0x2000,
0xEB01,
0x0781,
0x3D09,
0xEB00,
0x01C0,
0xEB01,
0x1100,
0xEB06,
0x0141,
0xEB01,
0x0147,
0xEB01,
0x0145,
0xF854,
0x2020,
0xF8B1,
0x125C,
0x434A,
0x0B11,
0xF844,
0x1020,
0x1C40,
0x2804,
0xD3EA,
0x2000,
0xEB00,
0x01C0,
0xEB01,
0x1100,
0xEB06,
0x0141,
0xEB01,
0x0147,
0xEB01,
0x0145,
0xF8B1,
0x23EC,
0xEB04,
0x0180,
0x1C40,
0x690B,
0x4353,
0x0B1A,
0x610A,
0x2804,
0xD3EA,
0xF000,
0xF9C1,
0x2803,
0xD115,
0x2000,
0xEB00,
0x01C0,
0xEB01,
0x1100,
0xEB06,
0x0141,
0xEB01,
0x0147,
0xEB01,
0x0145,
0xF8B1,
0x2324,
0xEB04,
0x0180,
0x1C40,
0x6A0B,
0x4353,
0x0B1A,
0x620A,
0x2804,
0xD3EA,
0xE8BD,
0x83F8,
0x2309,
0x2210,
0x6CA9,
0x6A68,
0xF000,
0xF9BA,
0x0A07,
0x2309,
0x2210,
0x6829,
0x6CA8,
0xF000,
0xF9B3,
0xEA4F,
0x2810,
0xE795,
0xB570,
0x4606,
0x48B5,
0x2200,
0x68C1,
0x0C0C,
0xB28D,
0x4629,
0x4620,
0xF000,
0xF9AA,
0x4630,
0xF000,
0xF9AC,
0x2201,
0x4629,
0x4620,
0xF000,
0xF9A2,
0x49AD,
0xF44F,
0x6080,
0x8008,
0x4AAB,
0xF44F,
0x61D6,
0x1C92,
0x8011,
0x1C91,
0x8008,
0x1D12,
0xF240,
0x519C,
0x8011,
0x1C92,
0x8010,
0x1C92,
0x8011,
0x1C92,
0x8010,
0x1C92,
0x8011,
0x1C92,
0x8010,
0x1C90,
0x8001,
0xBD70,
0x4BA0,
0xB510,
0xF8D3,
0x2474,
0x2A00,
0xD00E,
0xF8D3,
0x4308,
0xFB00,
0x1004,
0x499C,
0x888C,
0x4344,
0x88C8,
0xF8D3,
0x1314,
0xFBB4,
0xF0F0,
0x1A40,
0xFBB0,
0xF0F2,
0xBD10,
0xE92D,
0x4FFF,
0xB089,
0x4691,
0xAA16,
0x4991,
0xE892,
0x1101,
0xE9DD,
0x471A,
0xE9DD,
0x6A1D,
0x7809,
0xF8DD,
0xB064,
0x461D,
0xB111,
0x4621,
0xF7FF,
0xFFD6,
0x498D,
0xF00A,
0x03FF,
0xFA1F,
0xF289,
0xF8A1,
0x519E,
0xF8C1,
0x018C,
0xF8A1,
0x8190,
0xF8A1,
0xC1C2,
0xF8C1,
0xB1B0,
0xF8A1,
0x41B4,
0xF8A1,
0x71E6,
0x981C,
0xF8C1,
0x01D4,
0xF8A1,
0x61D8,
0xF501,
0x71C2,
0x460C,
0x4668,
0xF000,
0xF948,
0x4621,
0x4668,
0x9A1F,
0xF000,
0xF948,
0x487D,
0x7800,
0xB138,
0xE9DD,
0x1209,
0x4668,
0xF000,
0xF945,
0xB108,
0xF000,
0xF947,
0x2100,
0x4871,
0xF000,
0xF948,
0x486F,
0x2101,
0x3024,
0xF000,
0xF943,
0x486D,
0x2102,
0x3048,
0xF000,
0xF93E,
0xB00D,
0xE6C3,
0x486E,
0xF890,
0x10C1,
0xB119,
0xF890,
0x0057,
0x2811,
0xD001,
0x2000,
0x4770,
0x2001,
0x4770,
0xB510,
0xB1E8,
0x4969,
0x2228,
0xF5A1,
0x7056,
0x4604,
0xF000,
0xF92C,
0x4A60,
0x4962,
0x2000,
0x7010,
0x8808,
0xB150,
0x8848,
0xB930,
0xF7FF,
0xFFE1,
0xB128,
0x485C,
0xF890,
0x047E,
0xB908,
0x2001,
0x7010,
0xF000,
0xF91E,
0x4620,
0xF000,
0xF920,
0xE006,
0x495A,
0x220E,
0x3118,
0xF5A1,
0x7056,
0xF000,
0xF90E,
0xF000,
0xF91B,
0xF000,
0xF91E,
0xF000,
0xF921,
0xE8BD,
0x4010,
0xF000,
0xB922,
0xB570,
0x4D52,
0x4C53,
0x8828,
0x8120,
0x81A0,
0xF000,
0xF91F,
0x8020,
0x8828,
0x80E0,
0x2003,
0xF000,
0xF91E,
0x2008,
0xF000,
0xF920,
0x4D4C,
0x80A0,
0x7D28,
0x2803,
0xD106,
0x2000,
0x7528,
0x484A,
0xF000,
0xF91B,
0xF000,
0xF91E,
0x7D28,
0xBB58,
0x4C47,
0xF894,
0x0967,
0xB968,
0x4846,
0x7800,
0xB950,
0xF894,
0x096C,
0xB938,
0x483A,
0x7A00,
0xB110,
0xF894,
0x095C,
0xB908,
0x7D68,
0xB118,
0x2110,
0x483D,
0xF000,
0xF90B,
0xF894,
0x0969,
0xB908,
0x7D68,
0xB118,
0x2140,
0x4838,
0xF000,
0xF902,
0xF000,
0xF905,
0xB908,
0x7D68,
0xB138,
0x210C,
0x4834,
0xF000,
0xF8F9,
0x2101,
0x4832,
0xF000,
0xF8F5,
0xF000,
0xF8FD,
0x7D68,
0x2800,
0xD004,
0xE8BD,
0x4070,
0x4830,
0xF000,
0xB8FA,
0xBD70,
0x0889,
0x0089,
0xEA41,
0x0040,
0xB282,
0x213E,
0xF246,
0x2044,
0xF000,
0xB8F4,
0xB510,
0x2200,
0xF2AF,
0x41AB,
0x4828,
0xF000,
0xF8F2,
0x4C18,
0x2200,
0xF2AF,
0x31B5,
0x6060,
0x4825,
0xF000,
0xF8EA,
0x2200,
0xF2AF,
0x2189,
0x60A0,
0x4822,
0xF000,
0xF8E3,
0x2200,
0xF2AF,
0x210F,
0x60E0,
0x4820,
0xF000,
0xF8DC,
0x2200,
0xF2AF,
0x115F,
0x481E,
0xF000,
0xF8D6,
0x2000,
0x4621,
0x2201,
0x7008,
0xF2AF,
0x1111,
0x481A,
0xF000,
0xF8CD,
0x2200,
0xF2AF,
0x0171,
0xE8BD,
0x4010,
0x4817,
0xF000,
0xB8C5,
0x0000,
0x0020,
0x689C,
0x0120,
0xB446,
0x0120,
0x7058,
0x0040,
0x629D,
0x0020,
0xC068,
0x0120,
0x00F0,
0x0120,
0xB045,
0x0020,
0xF012,
0x0020,
0x346D,
0x0020,
0x50ED,
0x0020,
0xF061,
0x0020,
0x9023,
0x0020,
0x703A,
0x0020,
0x00BA,
0x0020,
0x60ED,
0x0020,
0x2094,
0x0100,
0x4775,
0x0100,
0x6173,
0x0100,
0x4D57,
0x0200,
0x2D43,
0x0000,
0x27B1,
0x0100,
0x157E,
0x0000,
0x836D,
0xF640,
0x1CD9,
0xF2C0,
0x0C00,
0x4760,
0xF244,
0x3CCD,
0xF2C0,
0x0C02,
0x4760,
0xF244,
0x3CDD,
0xF2C0,
0x0C02,
0x4760,
0xF244,
0x3CD5,
0xF2C0,
0x0C02,
0x4760,
0xF247,
0x3C3D,
0xF2C0,
0x0C01,
0x4760,
0xF647,
0x5CE9,
0xF2C0,
0x0C00,
0x4760,
0xF248,
0x1C39,
0xF2C0,
0x0C00,
0x4760,
0xF245,
0x7C4D,
0xF2C0,
0x0C01,
0x4760,
0xF643,
0x6CB7,
0xF2C0,
0x0C02,
0x4760,
0xF643,
0x6CE9,
0xF2C0,
0x0C02,
0x4760,
0xF244,
0x0CA9,
0xF2C0,
0x0C02,
0x4760,
0xF244,
0x2CB9,
0xF2C0,
0x0C02,
0x4760,
0xF244,
0x2CF7,
0xF2C0,
0x0C02,
0x4760,
0xF246,
0x1CED,
0xF2C0,
0x0C02,
0x4760,
0xF645,
0x1CE5,
0xF2C0,
0x0C02,
0x4760,
0xF24B,
0x0C77,
0xF2C0,
0x0C00,
0x4760,
0xF640,
0x6CC7,
0xF2C0,
0x0C00,
0x4760,
0xF24A,
0x7CC1,
0xF2C0,
0x0C00,
0x4760,
0xF24A,
0x7C79,
0xF2C0,
0x0C00,
0x4760,
0xF248,
0x1C69,
0xF2C0,
0x0C01,
0x4760,
0xF241,
0x1C33,
0xF2C0,
0x0C00,
0x4760,
0xF244,
0x3CE5,
0xF2C0,
0x0C02,
0x4760,
0xF248,
0x4C55,
0xF2C0,
0x0C00,
0x4760,
0xF240,
0x7C6D,
0xF2C0,
0x0C00,
0x4760,
0xF647,
0x5C97,
0xF2C0,
0x0C01,
0x4760,
0xF240,
0x6C47,
0xF2C0,
0x0C00,
0x4760,
0xF640,
0x6CBF,
0xF2C0,
0x0C00,
0x4760,
0xF647,
0x5CC5,
0xF2C0,
0x0C01,
0x4760,
0xF643,
0x1CBB,
0xF2C0,
0x0C01,
0x4760,
0xF248,
0x1C55,
0xF2C0,
0x0C00,
0x4760,
0xF64B,
0x2C15,
0xF2C0,
0x0C00,
0x4760,
0x0000,
0x0000,
0x0000,
0x0000,
0x0000,
0x0000,
0x0000,
0x4108,
0xBA02,
0x0000,
0x3F00,

};

static kal_uint16 addr_data_pair_global[] = {
//global
0x6028, 0x2000,
0x602A, 0x2500,
0x6F12, 0x0080,
0x602A, 0x10B8,
0x6F12, 0x0020,
0x602A, 0x1EE0,
0x6F12, 0x0078,
0x602A, 0x2870,
0x6F12, 0x0100,
0x602A, 0x250A,
0x6F12, 0x0000,
0x602A, 0x23A0,
0x6F12, 0x0001,
0x602A, 0x3022,
0x6F12, 0x1281,
0x602A, 0x32E8,
0x6F12, 0x0100,
0x602A, 0x54A2,
0x6F12, 0x0000,
0x602A, 0x120E,
0x6F12, 0x0000,
0x602A, 0x1212,
0x6F12, 0x0000,
0x602A, 0x2860,
0x6F12, 0x0001,
0x602A, 0x3220,
0x6F12, 0x0000,
0x602A, 0x1226,
0x6F12, 0x0301,
0x602A, 0x29C8,
0x6F12, 0x0000,
0x602A, 0x32EC,
0x6F12, 0x0000,
0x602A, 0x12BE,
0x6F12, 0x0101,
0x602A, 0x3034,
0x6F12, 0x049B,
0x602A, 0x1230,
0x6F12, 0x0100,
0x602A, 0x1232,
0x6F12, 0x00F0,
0x602A, 0x1236,
0x6F12, 0x01FF,
0x602A, 0x123A,
0x6F12, 0x0004,
0x602A, 0x123E,
0x6F12, 0xF45A,
0x602A, 0x1EE2,
0x6F12, 0x19CD,
0x602A, 0x115E,
0x6F12, 0x0048,
0x602A, 0x131C,
0x6F12, 0x2400,
0x602A, 0x2872,
0x6F12, 0x0001,
0x602A, 0x1314,
0x6F12, 0x0100,
0x602A, 0x20DE,
0x6F12, 0x0003,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x602A, 0x2108,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x602A, 0x1EDC,
0x6F12, 0x5008,
0x602A, 0x138E,
0x6F12, 0x13C0,
0x602A, 0x1392,
0x6F12, 0x0038,
0x602A, 0x21B6,
0x6F12, 0x0002,
0x6F12, 0x0000,
0x602A, 0x2550,
0x6F12, 0x193C,
0x6028, 0x4000,
0x0BC0, 0x0040,
0x0FE8, 0x49C1,
0x0FEA, 0x0040,
0x0BC8, 0x0001,
0x0B0A, 0x0101,
0x0BC6, 0x0000,
0x0B06, 0x0101,
0xF446, 0x000C,
0xF448, 0x0018,
0xF450, 0x0010,
0xF44E, 0x0000,
0xF468, 0xE000,
//functional
0x6028, 0x2000,
0x602A, 0x3778,
0x6F12, 0x0100,
0x602A, 0x37FC,
0x6F12, 0x0000,
0x602A, 0x4BFC,
0x6F12, 0xD2D2,
0x6F12, 0xD2D2,
0x6F12, 0xD2D2,
0x602A, 0x465C,
0x6F12, 0x1414,
0x6F12, 0x1414,
0x6F12, 0x1414,
0x602A, 0x4652,
0x6F12, 0x1023,
0x6F12, 0x2323,
0x6F12, 0x2323,
0x6F12, 0x2300,
0x602A, 0x466E,
0x6F12, 0x1313,
0x6F12, 0x1313,
0x6F12, 0x1313,
0x602A, 0x469A,
0x6F12, 0x1014,
0x6F12, 0x1414,
0x6F12, 0x1414,
0x6F12, 0x1400,
0x602A, 0x46AC,
0x6F12, 0x1013,
0x6F12, 0x1313,
0x6F12, 0x1313,
0x6F12, 0x1300,
0x602A, 0x4676,
0x6F12, 0x100A,
0x6F12, 0x0A0A,
0x6F12, 0x0A0A,
0x6F12, 0x0A00,
0x602A, 0x4688,
0x6F12, 0x101D,
0x6F12, 0x1D1D,
0x6F12, 0x1D1D,
0x6F12, 0x1D00,
0x602A, 0x4C0E,
0x6F12, 0x7878,
0x6F12, 0x7878,
0x6F12, 0x7878,
0x602A, 0x3B1E,
0x6F12, 0x008C,
0x602A, 0x4C20,
0x6F12, 0x1D1D,
0x6F12, 0x1D1D,
0x6F12, 0x1D1D,
0x602A, 0x3B12,
0x6F12, 0x0002,
0x602A, 0x3AF2,
0x6F12, 0x0002,
0x602A, 0x3AF6,
0x6F12, 0x0005,
0x602A, 0x3AFA,
0x6F12, 0x0007,
0x602A, 0x3AFE,
0x6F12, 0x0064,
0x602A, 0x3B02,
0x6F12, 0x00AF,
0x602A, 0x3B06,
0x6F12, 0x00C8,
0x602A, 0x46BE,
0x6F12, 0x10D4,
0x6F12, 0xD4D4,
0x6F12, 0xD4D4,
0x6F12, 0xD400,
0x602A, 0x46C8,
0x6F12, 0xFAFA,
0x6F12, 0xFAFA,
0x6F12, 0xFAFA,
0x602A, 0x3B2E,
0x6F12, 0x0008,
0x602A, 0x3B32,
0x6F12, 0x0070,
0x602A, 0x4C28,
0x6F12, 0x1033,
0x6F12, 0x3333,
0x6F12, 0x3333,
0x6F12, 0x3300,
0x602A, 0x4C32,
0x6F12, 0x1919,
0x6F12, 0x1919,
0x6F12, 0x1919,
0x602A, 0x4C3A,
0x6F12, 0x10CC,
0x6F12, 0xCCCC,
0x6F12, 0xCCCC,
0x6F12, 0xCC00,
0x602A, 0x4C44,
0x6F12, 0x3333,
0x6F12, 0x3333,
0x6F12, 0x3333,
0x602A, 0x4C4C,
0x6F12, 0x1066,
0x6F12, 0x6666,
0x6F12, 0x6666,
0x6F12, 0x6600,
0x602A, 0x4C56,
0x6F12, 0x2222,
0x6F12, 0x2222,
0x6F12, 0x2222,
0x602A, 0x3E06,
0x6F12, 0x0000,
0x602A, 0x3E0A,
0x6F12, 0x0000,
0x602A, 0x3E2E,
0x6F12, 0x0060,
0x602A, 0x37FE,
0x6F12, 0x0001,
0x6F12, 0x0001,
0x6F12, 0x0001,
0x6F12, 0x0001,
0x602A, 0x3E0E,
0x6F12, 0x0019,
0x602A, 0x3E12,
0x6F12, 0x00FE,
0x602A, 0x3E16,
0x6F12, 0x0019,
0x602A, 0x3E1A,
0x6F12, 0x00FE,
0x602A, 0x3E1E,
0x6F12, 0x001E,
0x602A, 0x3E22,
0x6F12, 0x00FF,
0x602A, 0x3E26,
0x6F12, 0x0014,
0x602A, 0x3E2A,
0x6F12, 0x00DA,
0x602A, 0x3CB2,
0x6F12, 0x0000,
0x602A, 0x3BA2,
0x6F12, 0x0000,
0x602A, 0x4C5E,
0x6F12, 0x4078,
0x6F12, 0x785E,
0x6F12, 0x4236,
0x6F12, 0x3601,
0x602A, 0x4C68,
0x6F12, 0x7878,
0x6F12, 0x5E42,
0x6F12, 0x3636,
0x602A, 0x4C70,
0x6F12, 0x405A,
0x6F12, 0x5A78,
0x6F12, 0x96B4,
0x6F12, 0xB401,
0x602A, 0x4C7A,
0x6F12, 0x6464,
0x6F12, 0x85A7,
0x602A, 0x4C82,
0x6F12, 0x4053,
0x6F12, 0x5370,
0x6F12, 0x8BA7,
0x6F12, 0xA701,
0x602A, 0x4C8C,
0x6F12, 0x5353,
0x6F12, 0x708B,
0x6F12, 0xA7A7,
0x602A, 0x4C94,
0x6F12, 0x4064,
0x6F12, 0x6486,
0x6F12, 0xA7C8,
0x6F12, 0xC801,
0x602A, 0x4C9E,
0x6F12, 0x1414,
0x6F12, 0x1B21,
0x6F12, 0x2828,
0x602A, 0x4CA6,
0x6F12, 0x4014,
0x6F12, 0x141B,
0x6F12, 0x2128,
0x6F12, 0x2801,
0x602A, 0x4CB0,
0x6F12, 0x1B1B,
0x6F12, 0x232D,
0x6F12, 0x3636,
0x602A, 0x4CB8,
0x6F12, 0x403C,
0x6F12, 0x3C50,
0x6F12, 0x6478,
0x6F12, 0x7801,
0x602A, 0x4CC2,
0x6F12, 0x3C3C,
0x6F12, 0x5064,
0x6F12, 0x7878,
0x602A, 0x3DA6,
0x6F12, 0x0035,
0x602A, 0x3DAA,
0x6F12, 0x0028,
0x602A, 0x3DB0,
0x6F12, 0x01AB,
0x6F12, 0x0001,
0x6F12, 0x01AC,
0x6F12, 0x0050,
0x6F12, 0x01AD,
0x6F12, 0x0064,
0x6F12, 0x01AE,
0x6F12, 0x0064,
0x6F12, 0x01AF,
0x6F12, 0x00C8,
0x6F12, 0x01B0,
0x6F12, 0x00C8,
0x602A, 0x3DD4,
0x6F12, 0x01B4,
0x6F12, 0x0032,
0x6F12, 0x01B5,
0x6F12, 0x0050,
0x6F12, 0x01B6,
0x6F12, 0x0050,
0x6F12, 0x01B7,
0x6F12, 0x00C8,
0x6F12, 0x01B8,
0x6F12, 0x00C8,
0x6F12, 0x01B9,
0x6F12, 0x0081,
};

static kal_uint16 addr_data_pair_preview[] = {
0x6028, 0x4000,
0x6214, 0xF9F0,
0x6218, 0xE150,
0x6242, 0x0E00,
0x6028, 0x2000,
0x602A, 0x12F2,
0x6F12, 0x0D10,
0x6F12, 0x0A18,
0x6F12, 0x19B0,
0x6F12, 0x1350,
0x602A, 0x1EB6,
0x6F12, 0x0206,
0x602A, 0x3770,
0x6F12, 0x0100,
0x602A, 0x1EB8,
0x6F12, 0x0300,
0x602A, 0x131E,
0x6F12, 0x0100,
0x602A, 0x3DEA,
0x6F12, 0x0081,
0x602A, 0x11A6,
0x6F12, 0x0000,
0x6F12, 0x0004,
0x602A, 0x11AE,
0x6F12, 0x0003,
0x602A, 0x13FC,
0x6F12, 0x0064,
0x6F12, 0x0044,
0x6F12, 0x0044,
0x602A, 0x3302,
0x6F12, 0x0100,
0x6F12, 0x0100,
0x6F12, 0x0001,
0x602A, 0x27D2,
0x6F12, 0x0203,
0x602A, 0x1EC8,
0x6F12, 0x0503,
0x6F12, 0x0504,
0x602A, 0x1ED2,
0x6F12, 0x080F,
0x602A, 0x1ED6,
0x6F12, 0x0307,
0x602A, 0x123C,
0x6F12, 0x0009,
0x602A, 0x21BE,
0x6F12, 0x04D2,
0x6F12, 0x41A6,
0x602A, 0x145C,
0x6F12, 0x0049,
0x6F12, 0x0035,
0x6F12, 0x0035,
0x602A, 0x140E,
0x6F12, 0x0001,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6028, 0x4000,
0xF466, 0x0E0D,
0x0344, 0x0008,
0x0346, 0x0000,
0x0348, 0x19A7,
0x034A, 0x134E,
0x034C, 0x0CD0,
0x034E, 0x09A0,
0x0350, 0x0000,
0x0352, 0x0004,
0x0900, 0x0112,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0002,
0x0386, 0x0002,
0x0400, 0x2010,
0x0404, 0x1000,
0x0402, 0x1010,
0x0114, 0x0300,
0x0116, 0x3000,
0x0110, 0x1002,
0x011C, 0x0100,
0x0136, 0x1800,
0x0300, 0x0002,
0x0302, 0x0003,
0x0304, 0x0004,
0x0306, 0x011E,
0x0308, 0x0008,
0x030A, 0x0002,
0x030C, 0x0000,
0x030E, 0x0004,
0x0310, 0x014C,
0x0312, 0x0002,
0x0340, 0x0DF4,
0x0342, 0x29A0,
0x0202, 0x0100,
0x0200, 0x0100,
0x022C, 0x0100,
0x0226, 0x0100,
0x021E, 0x0000,
0x6028, 0x2000,
0x602A, 0x3020,
0x6F12, 0x0000,
0x6028, 0x4000,
0x0B00, 0x0080,
0x0B08, 0x0000,
0x0D00, 0x0000,
0x0D02, 0x0000,
0x0D04, 0x0000,
0x6028, 0x4000,
0xF44A, 0x001F,
};

static kal_uint16 addr_data_pair_capture[] = {
0x6028, 0x4000,
0x6214, 0xF9F0,
0x6218, 0xE150,
0x6242, 0x0E00,
0x6028, 0x2000,
0x602A, 0x12F2,
0x6F12, 0x0D10,
0x6F12, 0x0A18,
0x6F12, 0x19B0,
0x6F12, 0x1350,
0x602A, 0x1EB6,
0x6F12, 0x0206,
0x602A, 0x3770,
0x6F12, 0x0100,
0x602A, 0x1EB8,
0x6F12, 0x0300,
0x602A, 0x131E,
0x6F12, 0x0100,
0x602A, 0x3DEA,
0x6F12, 0x0081,
0x602A, 0x11A6,
0x6F12, 0x0000,
0x6F12, 0x0004,
0x602A, 0x11AE,
0x6F12, 0x0003,
0x602A, 0x13FC,
0x6F12, 0x0064,
0x6F12, 0x0044,
0x6F12, 0x0044,
0x602A, 0x3302,
0x6F12, 0x0100,
0x6F12, 0x0100,
0x6F12, 0x0001,
0x602A, 0x27D2,
0x6F12, 0x0203,
0x602A, 0x1EC8,
0x6F12, 0x0503,
0x6F12, 0x0504,
0x602A, 0x1ED2,
0x6F12, 0x080F,
0x602A, 0x1ED6,
0x6F12, 0x0307,
0x602A, 0x123C,
0x6F12, 0x0009,
0x602A, 0x21BE,
0x6F12, 0x04D2,
0x6F12, 0x41A6,
0x602A, 0x145C,
0x6F12, 0x0049,
0x6F12, 0x0035,
0x6F12, 0x0035,
0x602A, 0x140E,
0x6F12, 0x0001,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6028, 0x4000,
0xF466, 0x0E0D,
0x0344, 0x0008,
0x0346, 0x0000,
0x0348, 0x19A7,
0x034A, 0x134E,
0x034C, 0x0CD0,
0x034E, 0x09A0,
0x0350, 0x0000,
0x0352, 0x0004,
0x0900, 0x0112,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0002,
0x0386, 0x0002,
0x0400, 0x2010,
0x0404, 0x1000,
0x0402, 0x1010,
0x0114, 0x0300,
0x0116, 0x3000,
0x0110, 0x1002,
0x011C, 0x0100,
0x0136, 0x1800,
0x0300, 0x0002,
0x0302, 0x0003,
0x0304, 0x0004,
0x0306, 0x011E,
0x0308, 0x0008,
0x030A, 0x0002,
0x030C, 0x0000,
0x030E, 0x0004,
0x0310, 0x014C,
0x0312, 0x0002,
0x0340, 0x0DF4,
0x0342, 0x29A0,
0x0202, 0x0100,
0x0200, 0x0100,
0x022C, 0x0100,
0x0226, 0x0100,
0x021E, 0x0000,
0x6028, 0x2000,
0x602A, 0x3020,
0x6F12, 0x0000,
0x6028, 0x4000,
0x0B00, 0x0080,
0x0B08, 0x0000,
0x0D00, 0x0000,
0x0D02, 0x0000,
0x0D04, 0x0000,
0x6028, 0x4000,
0xF44A, 0x001F,
};

static kal_uint16 addr_data_pair_custom1[] = {
0x6028, 0x4000,
0x6214, 0xF9F0,
0x6218, 0xE150,
0x6242, 0x0E00,
0x6028, 0x2000,
0x602A, 0x12F2,
0x6F12, 0x0D10,
0x6F12, 0x0A18,
0x6F12, 0x19B0,
0x6F12, 0x1350,
0x602A, 0x1EB6,
0x6F12, 0x0206,
0x602A, 0x3770,
0x6F12, 0x0100,
0x602A, 0x1EB8,
0x6F12, 0x0300,
0x602A, 0x131E,
0x6F12, 0x0100,
0x602A, 0x3DEA,
0x6F12, 0x0081,
0x602A, 0x11A6,
0x6F12, 0x0000,
0x6F12, 0x0004,
0x602A, 0x11AE,
0x6F12, 0x0003,
0x602A, 0x13FC,
0x6F12, 0x0064,
0x6F12, 0x0044,
0x6F12, 0x0044,
0x602A, 0x3302,
0x6F12, 0x0100,
0x6F12, 0x0100,
0x6F12, 0x0001,
0x602A, 0x27D2,
0x6F12, 0x0203,
0x602A, 0x1EC8,
0x6F12, 0x0503,
0x6F12, 0x0504,
0x602A, 0x1ED2,
0x6F12, 0x080F,
0x602A, 0x1ED6,
0x6F12, 0x0307,
0x602A, 0x123C,
0x6F12, 0x0009,
0x602A, 0x21BE,
0x6F12, 0x04D2,
0x6F12, 0x41A6,
0x602A, 0x145C,
0x6F12, 0x0049,
0x6F12, 0x0035,
0x6F12, 0x0035,
0x602A, 0x140E,
0x6F12, 0x0001,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6028, 0x4000,
0xF466, 0x0E0D,
0x0344, 0x03D8,
0x0346, 0x02E0,
0x0348, 0x15D7,
0x034A, 0x106E,
0x034C, 0x0900,
0x034E, 0x06C0,
0x0350, 0x0000,
0x0352, 0x0004,
0x0900, 0x0112,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0002,
0x0386, 0x0002,
0x0400, 0x2010,
0x0404, 0x1000,
0x0402, 0x1010,
0x0114, 0x0300,
0x0116, 0x3000,
0x0110, 0x1002,
0x011C, 0x0100,
0x0136, 0x1800,
0x0300, 0x0002,
0x0302, 0x0003,
0x0304, 0x0004,
0x0306, 0x011E,
0x0308, 0x0008,
0x030A, 0x0002,
0x030C, 0x0000,
0x030E, 0x0004,
0x0310, 0x00FC,
0x0312, 0x0002,
0x0340, 0x0A40,
0x0342, 0x38C0,
0x0202, 0x0100,
0x0200, 0x0100,
0x022C, 0x0100,
0x0226, 0x0100,
0x021E, 0x0000,
0x6028, 0x2000,
0x602A, 0x3020,
0x6F12, 0x0000,
0x6028, 0x4000,
0x0B00, 0x0080,
0x0B08, 0x0000,
0x0D00, 0x0000,
0x0D02, 0x0000,
0x0D04, 0x0000,
0x6028, 0x4000,
0xF44A, 0x001F,
};

static kal_uint16 addr_data_pair_custom3[] = {
0x6028, 0x4000,
0x6214, 0xF9F0,
0x6218, 0xE150,
0x6242, 0x0E00,
0x6028, 0x2000,
0x602A, 0x12F2,
0x6F12, 0x0D10,
0x6F12, 0x0A18,
0x6F12, 0x19B0,
0x6F12, 0x1350,
0x602A, 0x1EB6,
0x6F12, 0x0206,
0x602A, 0x3770,
0x6F12, 0x0000,
0x602A, 0x1EB8,
0x6F12, 0x0300,
0x602A, 0x131E,
0x6F12, 0x0100,
0x602A, 0x3DEA,
0x6F12, 0x0081,
0x602A, 0x11A6,
0x6F12, 0x0200,
0x6F12, 0x0098,
0x602A, 0x11AE,
0x6F12, 0x0088,
0x602A, 0x13FC,
0x6F12, 0x0064,
0x6F12, 0x0044,
0x6F12, 0x0044,
0x602A, 0x3302,
0x6F12, 0x0101,
0x6F12, 0x0100,
0x6F12, 0x0001,
0x602A, 0x27D2,
0x6F12, 0x0101,
0x602A, 0x1EC8,
0x6F12, 0x0603,
0x6F12, 0x0504,
0x602A, 0x1ED2,
0x6F12, 0x080F,
0x602A, 0x1ED6,
0x6F12, 0x0307,
0x602A, 0x123C,
0x6F12, 0x0004,
0x602A, 0x21BE,
0x6F12, 0x04D2,
0x6F12, 0x41A6,
0x602A, 0x145C,
0x6F12, 0x0049,
0x6F12, 0x0035,
0x6F12, 0x0035,
0x602A, 0x140E,
0x6F12, 0x0001,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6028, 0x4000,
0xF466, 0x0FFD,
0x0344, 0x0008,
0x0346, 0x0000,
0x0348, 0x19A7,
0x034A, 0x134F,
0x034C, 0x19A0,
0x034E, 0x1340,
0x0350, 0x0000,
0x0352, 0x0008,
0x0900, 0x0111,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0001,
0x0386, 0x0001,
0x0400, 0x1010,
0x0404, 0x1000,
0x0402, 0x1010,
0x0114, 0x0300,
0x0116, 0x3000,
0x0110, 0x1002,
0x011C, 0x0100,
0x0136, 0x1800,
0x0300, 0x0002,
0x0302, 0x0003,
0x0304, 0x0004,
0x0306, 0x011E,
0x0308, 0x0008,
0x030A, 0x0002,
0x030C, 0x0000,
0x030E, 0x0004,
0x0310, 0x00F2,
0x0312, 0x0000,
0x0340, 0x1448,
0x0342, 0x1CB0,
0x0202, 0x0100,
0x0200, 0x0100,
0x022C, 0x0100,
0x0226, 0x0100,
0x021E, 0x0000,
0x6028, 0x2000,
0x602A, 0x3020,
0x6F12, 0x0001,
0x6028, 0x4000,
0x0B00, 0x0080,
0x0B08, 0x0001,
0x0D00, 0x0000,
0x0D02, 0x0000,
0x0D04, 0x0000,
0x6028, 0x4000,
0xF44A, 0x001F,
};

static kal_uint16 addr_data_pair_slimvideo[] = {
0x6028, 0x4000,
0x6214, 0xF9F0,
0x6218, 0xE150,
0x6242, 0x0E00,
0x6028, 0x2000,
0x602A, 0x12F2,
0x6F12, 0x0D10,
0x6F12, 0x0A18,
0x6F12, 0x19B0,
0x6F12, 0x1350,
0x602A, 0x1EB6,
0x6F12, 0x0206,
0x602A, 0x3770,
0x6F12, 0x0100,
0x602A, 0x1EB8,
0x6F12, 0x0301,
0x602A, 0x131E,
0x6F12, 0x0100,
0x602A, 0x3DEA,
0x6F12, 0x0081,
0x602A, 0x11A6,
0x6F12, 0x0000,
0x6F12, 0x0004,
0x602A, 0x11AE,
0x6F12, 0x0003,
0x602A, 0x13FC,
0x6F12, 0x0044,
0x6F12, 0x0064,
0x6F12, 0x0044,
0x602A, 0x3302,
0x6F12, 0x0100,
0x6F12, 0x0100,
0x6F12, 0x0001,
0x602A, 0x27D2,
0x6F12, 0x0203,
0x602A, 0x1EC8,
0x6F12, 0x0503,
0x6F12, 0x0504,
0x602A, 0x1ED2,
0x6F12, 0x080F,
0x602A, 0x1ED6,
0x6F12, 0x0307,
0x602A, 0x123C,
0x6F12, 0x0009,
0x602A, 0x21BE,
0x6F12, 0x04D2,
0x6F12, 0x41A6,
0x602A, 0x1EE0,
0x6F12, 0x006C,
0x602A, 0x145C,
0x6F12, 0x0035,
0x6F12, 0x0049,
0x6F12, 0x0035,
0x602A, 0x140E,
0x6F12, 0x0000,
0x6F12, 0x0001,
0x6F12, 0x0000,
0x6028, 0x4000,
0xF466, 0x0E0D,
0x0344, 0x0018,
0x0346, 0x0270,
0x0348, 0x1997,
0x034A, 0x10DE,
0x034C, 0x0CC0,
0x034E, 0x0730,
0x0350, 0x0000,
0x0352, 0x0004,
0x0900, 0x0112,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0002,
0x0386, 0x0002,
0x0400, 0x2010,
0x0404, 0x1000,
0x0402, 0x1010,
0x0114, 0x0300,
0x0116, 0x3000,
0x0110, 0x1002,
0x011C, 0x0100,
0x0136, 0x1800,
0x0300, 0x0002,
0x0302, 0x0003,
0x0304, 0x0004,
0x0306, 0x011E,
0x0308, 0x0008,
0x030A, 0x0002,
0x030C, 0x0000,
0x030E, 0x0004,
0x0310, 0x00FC,
0x0312, 0x0002,
0x0340, 0x0A40,
0x0342, 0x38C0,
0x0202, 0x0100,
0x0200, 0x0100,
0x022C, 0x0100,
0x0226, 0x0100,
0x021E, 0x0000,
0x6028, 0x2000,
0x602A, 0x3020,
0x6F12, 0x0000,
0x6028, 0x4000,
0x0B00, 0x0080,
0x0B08, 0x0000,
0x0D00, 0x0000,
0x0D02, 0x0000,
0x0D04, 0x0000,
0x6028, 0x4000,
0xF44A, 0x001F
};

static void sensor_init(void)
{
	kal_uint8 ret = 0;
	kal_uint8 i = 0;
	/*Global setting */
	LOG_INF("sensor_init E\n");

	write_cmos_sensor_16_16(0x6028, 0x4000);	 // Page pointer HW
	write_cmos_sensor_16_16(0x0000, 0x0010);	 // Version
	write_cmos_sensor_16_16(0x0000, 0x0841);
	write_cmos_sensor_16_16(0x6010, 0x0001);	 // Reset
	mdelay(24); // must add 24ms!
	write_cmos_sensor_16_16(0x6214, 0xF9F0);
	write_cmos_sensor_16_16(0x6218, 0xE150);
	write_cmos_sensor_16_16(0x6242, 0x0E00);
	write_cmos_sensor_16_16(0x6028, 0x4000); //TNP burst start
	write_cmos_sensor_16_16(0x6004, 0x0001);
	write_cmos_sensor_16_16(0x6028, 0x2001);
	write_cmos_sensor_16_16(0x602A, 0x518C);
	LOG_INF("Before uTnpArrayInit\n");
	for (i = 0; i < 3; i++) {
		LOG_INF("i = %d\n", i);
		ret = iWriteRegI2C((u8 *)uTnpArrayInit,
				   (u16)sizeof(uTnpArrayInit),
				   imgsensor.i2c_write_id); /* TNP burst */
		LOG_INF("ret = %d\n", ret);
		if (ret == 0)
			break;
	}
	LOG_INF("After uTnpArrayInit\n");
	write_cmos_sensor_16_16(0x6028, 0x4000);
	write_cmos_sensor_16_16(0x6004, 0x0000); //TNP burst end
	write_cmos_sensor_16_16(0x6028, 0x2001);
	write_cmos_sensor_16_16(0x602A, 0xF000);
	write_cmos_sensor_16_16(0x6F12, 0x0000);
	write_cmos_sensor_16_16(0x6F12, 0x0000);
	write_cmos_sensor_16_16(0x6F12, 0x0004);
	write_cmos_sensor_16_16(0x6F12, 0x0001);
	write_cmos_sensor_16_16(0x6F12, 0x0100);
	LOG_INF("Before addr_data_pair_globalX\n");
	table_write_cmos_sensor(addr_data_pair_global,
		sizeof(addr_data_pair_global) / sizeof(kal_uint16)); //Global & Functional
	LOG_INF("After addr_data_pair_global\n");

}	/*	sensor_init  */

static void capture_setting(void)
{
	table_write_cmos_sensor(addr_data_pair_capture,
		   sizeof(addr_data_pair_capture) / sizeof(kal_uint16));
}	/*	preview_setting  */



static void preview_setting(void)
{
	table_write_cmos_sensor(addr_data_pair_preview,
		   sizeof(addr_data_pair_preview) / sizeof(kal_uint16));
}	/*	preview_setting  */

static void custom1_setting(void)
{
	table_write_cmos_sensor(addr_data_pair_custom1,
		   sizeof(addr_data_pair_custom1) / sizeof(kal_uint16));
}

static void custom3_setting(void)
{
	table_write_cmos_sensor(addr_data_pair_custom3,
		   sizeof(addr_data_pair_custom3) / sizeof(kal_uint16));
}	/*	preview_setting  */


/* Pll Setting - VCO = 280Mhz*/
#if 0
static void capture_setting(kal_uint16 currefps)
{
}
#endif
#if 0
static void normal_video_setting(kal_uint16 currefps)
{

}
#endif
static void hs_video_setting(void)
{
	LOG_INF("E\n");
}

static void slim_video_setting(void)
{
	table_write_cmos_sensor(addr_data_pair_slimvideo,
		sizeof(addr_data_pair_slimvideo) / sizeof(kal_uint16));
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
	/*
	int I2C_BUS = -1;
	I2C_BUS = i2c_adapter_id(pgi2c_cfg_legacy->pinst->pi2c_client->adapter);
	LOG_INF("S5K3P9SPmipiraw_Sensor I2C_BUS = %d\n", I2C_BUS);
	if(I2C_BUS != 4){
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	*/
	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor_16_8(0x0000) << 8) | read_cmos_sensor_16_8(0x0001));
			LOG_INF("read out sensor id 0x%x \n", *sensor_id);
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
				#ifdef OPLUS_FEATURE_CAMERA_COMMON
				imgsensor_info.module_id = read_module_id();
				LOG_INF("s5kgd1spq2r_module_id=%d\n",imgsensor_info.module_id);
				if(deviceInfo_register_value == 0x00){
					//register_imgsensor_deviceinfo("Cam_f", DEVICE_VERSION_S5KGD1SPQ2R, imgsensor_info.module_id);
					deviceInfo_register_value = 0x01;
				}
				#endif
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id !=  imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF*/
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
	LOG_INF("S5KGD1 open start\n");
	#ifdef CONFIG_MTK_CAM_SECURITY_SUPPORT
	struct command_params c_params = {0};
	MUINT32 ret = 0;

	LOG_INF("%s imgsensor.enable_secure %d\n", __func__, imgsensor.enable_secure);

	if (imgsensor.enable_secure) {
		if (imgsensor_ca_invoke_command(IMGSENSOR_TEE_CMD_OPEN, c_params, &ret) == 0) {
			return ret;
		} else {
			return ERROR_TEE_CA_TA_FAIL;
		}
	}
	#endif


	LOG_INF("PLATFORM:MT6750,MIPI 4LANE\n");
	LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
				sensor_id = ((read_cmos_sensor_16_8(0x0000) << 8) | read_cmos_sensor_16_8(0x0001));
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id) {
			break;
		}
		retry = 2;
	}
	#ifndef OPLUS_FEATURE_CAMERA_COMMON
	if (imgsensor_info.sensor_id !=  sensor_id) {
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	#else
	if (imgsensor_info.sensor_id != sensor_id) {
		return ERROR_SENSORID_READ_FAIL;
	}
	#endif
	LOG_INF("S5KGD1 sensor_init start\n");
	/* initail sequence write in  */
	sensor_init();
	LOG_INF("S5KGD1 sensor_init End\n");
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
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
	LOG_INF("S5KGD1 open End\n");
	return ERROR_NONE;
}	/*	open  */



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
	#ifdef CONFIG_MTK_CAM_SECURITY_SUPPORT
	struct command_params c_params = {0};
	MUINT32 ret = 0;
	LOG_INF("%s imgsensor.enable_secure %d\n", __func__, imgsensor.enable_secure);

	if (imgsensor.enable_secure) {
		if (imgsensor_ca_invoke_command(IMGSENSOR_TEE_CMD_CLOSE, c_params, &ret) != 0) {
			return ERROR_TEE_CA_TA_FAIL;
		}
	}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.enable_secure = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("%s enable_secure = %d\n", __func__, imgsensor.enable_secure);

	/*rest all variable if necessary*/
	#endif
	/* No Need to implement this function */
	streaming_control(KAL_FALSE);
	LOG_INF("E\n");

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
	LOG_INF("S5KGD1 preview start\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
//	sensor_init();
	preview_setting();
	set_mirror_flip(imgsensor.mirror);
	LOG_INF("S5KGD1 preview End\n");
	//burst_read_to_check();
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
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
	LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n", imgsensor.current_fps, imgsensor_info.cap.max_framerate / 10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	/*	 capture_setting(imgsensor.current_fps);*/
	capture_setting();
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

	/* normal_video_setting(imgsensor.current_fps);*/
	preview_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("Don't use, no setting E\n");

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

	return ERROR_NONE;
}	/*	slim_video	 */

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

static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	imgsensor.pclk = imgsensor_info.custom3.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.custom3.linelength;
	imgsensor.frame_length = imgsensor_info.custom3.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom3_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width =  imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;

	sensor_resolution->SensorCustom2Width =  imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height = imgsensor_info.custom2.grabwindow_height;

	sensor_resolution->SensorCustom3Width	 = imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height	 = imgsensor_info.custom3.grabwindow_height;

	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet*/
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

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 	 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = 0;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x*/
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x*/
	sensor_info->SensorPacketECCOrder = 1;
	//sensor_info->sensorSecureType = 2;

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
		case MSDK_SCENARIO_ID_CUSTOM2:
			sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;

			break;
		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	#ifdef CONFIG_MTK_CAM_SECURITY_SUPPORT

	struct command_params c_params = {0};
	MUINT32 ret = 0;
	LOG_INF("scenario_id = %d\n", scenario_id);


	LOG_INF("%s imgsensor.enable_secure %d\n", __func__, imgsensor.enable_secure);
	c_params.param0 = (void *)scenario_id;
	c_params.param1 = (void *)image_window;
	c_params.param2 = (void *)sensor_config_data;

	if (imgsensor.enable_secure) {
		if (imgsensor_ca_invoke_command(IMGSENSOR_TEE_CMD_CONTROL, c_params, &ret) == 0) {
			return ret;
		} else {
			return ERROR_TEE_CA_TA_FAIL;
		}
	}

	#endif


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
		case MSDK_SCENARIO_ID_CUSTOM2:
			custom1(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			custom3(image_window, sensor_config_data);
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
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) {/*enable auto flicker*/
		imgsensor.autoflicker_en = KAL_TRUE;
	} else {/*Cancel Auto flick*/
		imgsensor.autoflicker_en = KAL_FALSE;
	}
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
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
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
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n", framerate,imgsensor_info.cap.max_framerate / 10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	    break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength): 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength): 0;
		imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
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
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		*framerate = imgsensor_info.custom3.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		/* 0x5E00[8]: 1 enable,  0 disable*/
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK*/
		write_cmos_sensor_16_16(0x0600, 0x0001);
		write_cmos_sensor_16_16(0x0602, 0x0000);
		write_cmos_sensor_16_16(0x0604, 0x0000);
		write_cmos_sensor_16_16(0x0606, 0x0000);
		write_cmos_sensor_16_16(0x0608, 0x0000);
	} else {
		/* 0x5E00[8]: 1 enable,  0 disable*/
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK*/
		write_cmos_sensor_16_16(0x0600, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}
static kal_uint32 s5kgd1spq2r_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	LOG_INF("s5kgd1spq2r_awb_gain: 0x100\n");

	write_cmos_sensor_16_16(0x0D82, 0x100);
	write_cmos_sensor_16_16(0x0D84, 0x100);
	write_cmos_sensor_16_16(0x0D86, 0x100);
	/*
	LOG_INF("gd1 [%s] read 0x0D82:0x%x, 0x0D84:0x%x, 0x0D86:0x%x\n",
		__func__,
		read_cmos_sensor_16_16(0x0D82),
		read_cmos_sensor_16_16(0x0D84),
		read_cmos_sensor_16_16(0x0D86));
	*/
	return ERROR_NONE;
}

/*write AWB gain to sensor*/
static void feedback_awbgain(kal_uint32 r_gain, kal_uint32 b_gain)
{
	UINT32 r_gain_int = 0;
	UINT32 b_gain_int = 0;

	r_gain_int = r_gain / 2;
	b_gain_int = b_gain / 2;

	/*write r_gain*/
	write_cmos_sensor_16_16(0x0D82, r_gain_int);
	/*write _gain*/
	write_cmos_sensor_16_16(0x0D86, b_gain_int);
	/*
	LOG_INF("gd1 [%s] write r_gain:%d, r_gain_int:%d, b_gain:%d, b_gain_int:%d\n",
		__func__, r_gain, r_gain_int, b_gain, b_gain_int);

	LOG_INF("gd1 [%s] read 0x0D82:0x%x, 0x0D84:0x%x, 0x0D86:0x%x\n",
		__func__, read_cmos_sensor_16_16(0x0D82),
		read_cmos_sensor_16_16(0x0D84), read_cmos_sensor_16_16(0x0D86));
	*/
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16= (UINT16 *) feature_para;
	UINT16 *feature_data_16= (UINT16 *) feature_para;
	UINT32 *feature_return_para_32= (UINT32 *) feature_para;
	UINT32 *feature_data_32= (UINT32 *) feature_para;
	unsigned long long *feature_data= (unsigned long long *) feature_para;

	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;


	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	#ifdef CONFIG_MTK_CAM_SECURITY_SUPPORT

	struct command_params c_params;
	MUINT32 ret = 0;
	/*LOG_INF("feature_id = %d %p %p %llu\n", feature_id, feature_para, feature_para_len, *feature_data);*/
	if (feature_id == SENSOR_FEATURE_SET_AS_SECURE_DRIVER) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.enable_secure = ((kal_bool) *feature_data);
		spin_unlock(&imgsensor_drv_lock);
		LOG_INF("imgsensor.enable_secure :%d\n", imgsensor.enable_secure);
	}

	if (imgsensor.enable_secure) {
		c_params.param0 = (void *)feature_id;
		c_params.param1 = feature_para;
		c_params.param2 = feature_para_len;
		if (imgsensor_ca_invoke_command(IMGSENSOR_TEE_CMD_FEATURE_CONTROL, c_params, &ret) == 0) {
			return ret;
		} else {
			return ERROR_TEE_CA_TA_FAIL;
		}
	}

	/*LOG_INF("feature_id = %d %p %p %llu\n", feature_id, feature_para, feature_para_len, *feature_data);*/
	#endif



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
	case MSDK_SCENARIO_ID_CUSTOM2:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom1.pclk;
                break;
	case MSDK_SCENARIO_ID_CUSTOM3:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom3.pclk;
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
		*feature_para_len= 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len= 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
	             set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		/*night_mode((BOOL) *feature_data);*/
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor_16_16(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor_16_16(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE*/
		/* if EEPROM does not exist in camera module.*/
		*feature_return_para_32= LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len= 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	/*Caohua.Lin@CAmera, modify for different module 20180723*/
	case SENSOR_FEATURE_CHECK_MODULE_ID:
		*feature_return_para_32 = imgsensor_info.module_id;
		break;
        case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
            {
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = -1200100;
                break;
        }
	#endif
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(
			   (enum MSDK_SCENARIO_ID_ENUM)*feature_data,
			   *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	/*case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
		read_2L9_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
		break;*/
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: /*for factory mode auto testing*/
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len= 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
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
	case MSDK_SCENARIO_ID_CUSTOM2:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom1.framelength << 16)
                                 + imgsensor_info.custom1.linelength;
                break;
	case MSDK_SCENARIO_ID_CUSTOM3:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom3.framelength << 16)
                                 + imgsensor_info.custom3.linelength;
                break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.pre.framelength << 16)
                                 + imgsensor_info.pre.linelength;
                break;
        }
        break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data_32);
		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));

		switch (*feature_data_32) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
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
			case MSDK_SCENARIO_ID_CUSTOM2:
				memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[5],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_CUSTOM3:
				memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[7],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n", (UINT16)*feature_data);
		PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data + 1));

		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				//memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(SET_PD_BLOCK_INFO_T));
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
			case MSDK_SCENARIO_ID_CUSTOM1:
			case MSDK_SCENARIO_ID_CUSTOM2:
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", (UINT16)*feature_data);
		/*PDAF capacity enable or not, 2p8 only full size support PDAF*/
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0; /* video & capture use same setting*/
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
				break;
			case MSDK_SCENARIO_ID_CUSTOM1:
			case MSDK_SCENARIO_ID_CUSTOM2:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
				break;
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
				break;
		}
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) *feature_data, (UINT16) *(feature_data + 1));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
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
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
					rate = imgsensor_info.slim_video.mipi_pixel_rate;
					break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					rate = imgsensor_info.pre.mipi_pixel_rate;
					break;
			case MSDK_SCENARIO_ID_CUSTOM1:
			case MSDK_SCENARIO_ID_CUSTOM2:
					rate = imgsensor_info.custom1.mipi_pixel_rate;
					break;
			case MSDK_SCENARIO_ID_CUSTOM3:
					rate = imgsensor_info.custom3.mipi_pixel_rate;
					break;
			default:
					rate = 0;
					break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
		}
		break;
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
			*feature_return_para_32 = 2; /*BINNING_AVERAGED*/
			break;
		}

		LOG_INF("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d\n",
			(UINT32)*feature_return_para_32);
		break;
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		/* modify to separate 3hdr and remosaic */
		if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3) {
			/*write AWB gain to sensor*/
			feedback_awbgain((UINT32)*(feature_data_32 + 1),
					(UINT32)*(feature_data_32 + 2));
		} else {
			s5kgd1spq2r_awb_gain(
				(struct SET_SENSOR_AWB_GAIN *) feature_para);
		}
		break;
	case SENSOR_FEATURE_GET_4CELL_DATA:
		{
			int type = (kal_uint16)(*feature_data);
			if (type == FOUR_CELL_CAL_TYPE_ALL) {
				LOG_INF("SENSOR_FEATURE_GET_4CELL_DATA type=%d\n", type);
				//brcb032gwz_read_4cell_from_eeprom_s5k3p9sp((char *)(*(feature_data+1)));
			} else if (type == FOUR_CELL_CAL_TYPE_GAIN_TBL) {
				LOG_INF("SENSOR_FEATURE_GET_4CELL_DATA type=%d\n", type);
				//brcb032gwz_read_4cell_from_eeprom_s5k3p9sp((char *)(*(feature_data+1)));
			} else {
				memset((void *)(*(feature_data+1)), 0, 4);
				LOG_INF("No type %d buffer on this sensor\n", type);
			}
			break;
		}

	default:
		break;
	}

	return ERROR_NONE;
}	/*	feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5KGD1SPQ2R_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!= NULL) {
		*pfFunc= &sensor_func;
	}
	return ERROR_NONE;
}	/*	S5K3P9SP_MIPI_RAW_SensorInit	*/
