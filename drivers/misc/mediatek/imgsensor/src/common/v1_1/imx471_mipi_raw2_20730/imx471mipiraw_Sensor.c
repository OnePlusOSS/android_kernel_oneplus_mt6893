/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 IMX471mipi_Sensor.c
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
#include <soc/oplus/system/oplus_project.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "imgsensor_i2c.h"
#include "imx471mipiraw_Sensor.h"

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define DEVICE_VERSION_IMX471    "imx471"
#define MODULE_ID_OFFSET 0x0000
#define FOUR_CELL_QSC_START 0x0C90
#define FOUR_CELL_QSC_SIZE 560

extern void Oplusimgsensor_Registdeviceinfo(char *name, char *version, u8 module_id);
static kal_uint8 deviceInfo_register_value = 0x00;
static char sensor_name_golden_result[16] = {0};
#endif

#define PFX "IMX471_camera_sensor"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
static kal_uint32 streaming_control(kal_bool enable);

#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __func__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);


static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX471_SENSOR_ID2_20730,

	.checksum_value = 0xffb1ec31,

	.pre = { /* reg_M 30fps */
		.pclk = 140000000,
		.linelength = 2560,
		.framelength = 1822,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 144000000,
		.max_framerate = 300,
	},
	.cap = { /* reg_M 30fps */
		.pclk = 140000000,
		.linelength = 2560,
		.framelength = 1822,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 144000000,
		.max_framerate = 300,
	},
	.normal_video = { /* reg_M 30fps */
		.pclk = 116000000,
		.linelength = 2560,
		.framelength = 1510,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1296,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 333600000,
		.max_framerate = 300,
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
		.mipi_pixel_rate = 585600000,
		.max_framerate = 1200,
	},
	.slim_video = { /* reg_M 30fps */
		.pclk = 140000000,
		.linelength = 2560,
		.framelength = 1822,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 144000000,
		.max_framerate = 300,
	},
	.custom1 = { /* reg_M 30fps */
		.pclk = 140000000,
		.linelength = 2560,
		.framelength = 1822,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 144000000,
		.max_framerate = 300,
	},
	.custom2 = { /* reg_M 30fps */
		.pclk = 142000000,
		.linelength = 2560,
		.framelength = 1848,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 146400000,
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

	.margin = 18,		/* sensor framelength & shutter margin */
	.min_shutter = 4,	/* min shutter */
	.max_frame_length = 0xffec,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.frame_time_delay_frame = 3,
	.ihdr_support = 0,	  /*1, support; 0,not support*/
	.ihdr_le_firstline = 0,  /*1,le first; 0, se first*/
	.sensor_mode_num = 9,	  /*support sensor mode num*/

	.cap_delay_frame = 3,	/* enter capture delay frame num */
	.pre_delay_frame = 2,	/* enter preview delay frame num */
	.video_delay_frame = 2,	/* enter video delay frame num */
	.hs_video_delay_frame = 2,	/* enter high speed video  delay frame num */
	.slim_video_delay_frame = 2,	/* enter slim video delay frame num */
	.custom1_delay_frame = 2,
	.custom2_delay_frame = 2,
	.custom3_delay_frame = 2,
	.custom4_delay_frame = 3,

	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, /*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
	.mipi_settle_delay_mode = 1, /*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_B, /*SENSOR_OUTPUT_FORMAT_RAW_Gr*/
	.mclk = 24,		/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x20, 0xff},
	.i2c_speed = 400,
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,				/*mirrorflip information IMAGE_NORMAL*/
	.sensor_mode = IMGSENSOR_MODE_INIT, /*IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video*/
	.shutter = 0x3D0,					/*current shutter*/
	.gain = 0x100,						/*current gain*/
	.dummy_pixel = 0,					/*current dummypixel*/
	.dummy_line = 0,					/*current dummyline*/
	.current_fps = 0,  /*full size current fps : 24fps for PIP, 30fps for Normal or ZSD*/
	.autoflicker_en = KAL_FALSE,  /*auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker*/
	.test_pattern = 0,		/*test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output*/
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/*current scenario id*/
	.ihdr_mode = 0, /*sensor need support LE, SE with HDR feature*/
	.i2c_write_id = 0x20, /*record current sensor's i2c write id*/
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[9] = {
{4656, 3496, 24, 20, 4608, 3456, 2304, 1728,  0,  0, 2304, 1728, 0, 0, 2304, 1728}, /* preview */
{4656, 3496, 24, 20, 4608, 3456, 2304, 1728,  0,  0, 2304, 1728, 0, 0, 2304, 1728}, /* capture */
{4656, 3496, 24, 444, 4608, 2608, 2304, 1304,  0,  4,2304, 1296, 0, 0, 2304, 1296}, /*normal-video*/
{4656, 3496, 24, 20, 4608, 3456, 2304, 1728,  0,  0, 2304, 1728, 0, 0, 2304, 1728}, /* hs-video*/
{4656, 3496, 24, 20, 4608, 3456, 2304, 1728,  0,  0, 2304, 1728, 0, 0, 2304, 1728}, /* slim-video*/
{4656, 3496, 24, 20, 4608, 3456, 2304, 1728,  0,  0, 2304, 1728, 0, 0, 2304, 1728}, /*custom1*/
{4656, 3496, 24, 20, 4608, 3456, 2304, 1728,  0,  0, 2304, 1728, 0, 0, 2304, 1728}, /*custom2*/
{4656, 3496, 24, 20, 4608, 3456, 4608, 3456,  0,  0, 4608, 3456, 0, 0, 4608, 3456}, /*custom3*/
{4656, 3496, 24, 20, 4608, 3456, 4608, 3456,  0,  0, 4608, 3456, 0, 0, 4608, 3456}, /*custom4 remosaic*/
};


/*no mirror flip*/

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static kal_uint16 read_module_id(void)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(MODULE_ID_OFFSET >> 8) , (char)(MODULE_ID_OFFSET & 0xFF) };

	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,0xA8/*EEPROM_READ_ID*/);
	if (get_byte == 0) {
		iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, 0xA8/*EEPROM_READ_ID*/);
	}

	return get_byte;
}
#endif

static kal_uint8 read_cmos_sensor_16_8(kal_uint16 addr)
{
	kal_uint8 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_16_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {
		(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_16_16(kal_uint32 addr)
{
	kal_uint16 get_byte= 0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	 /*kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}

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
			iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id, 3,
					     imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2C(puSendCmd, 3, imgsensor.i2c_write_id);
		tosend = 0;

#endif
	}
	return 0;
}


static void set_dummy(void)
{
	LOG_INF("frame_length = %d, line_length = %d\n",
	    imgsensor.frame_length,
	    imgsensor.line_length);

	write_cmos_sensor_16_8(0x0104, 0x01);

	write_cmos_sensor_16_8(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor_16_8(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor_16_8(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor_16_8(0x0343, imgsensor.line_length & 0xFF);

	write_cmos_sensor_16_8(0x0104, 0x00);
} /* set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d \n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

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

static void write_shutter(kal_uint16 shutter)
{

	kal_uint16 realtime_fps = 0;

		/* if shutter bigger than frame_length, extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps =
	imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 237 && realtime_fps <= 243)
			set_max_framerate(236, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor_16_8(0x0104, 0x01);
			write_cmos_sensor_16_8(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor_16_8(0x0341,imgsensor.frame_length & 0xFF);
			write_cmos_sensor_16_8(0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_16_8(0x0104, 0x01);
		write_cmos_sensor_16_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_16_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_16_8(0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor_16_8(0x0104, 0x01);
	write_cmos_sensor_16_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_16_8(0x0203, shutter & 0xFF);
	write_cmos_sensor_16_8(0x0104, 0x00);
	LOG_INF( "Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);
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
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor_16_8(0x0104, 0x01);
			write_cmos_sensor_16_8(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor_16_8(0x0341,imgsensor.frame_length & 0xFF);
			write_cmos_sensor_16_8(0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_16_8(0x0104, 0x01);
		write_cmos_sensor_16_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_16_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_16_8(0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor_16_8(0x0104, 0x01);
	write_cmos_sensor_16_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_16_8(0x0203, shutter & 0xFF);
	write_cmos_sensor_16_8(0x0104, 0x00);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);

}				/*      write_shutter  */


static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain;
	reg_gain = 1024 - (1024*64)/gain;
	LOG_INF("imx474 gain =%d, reg_gain =%d\n", gain, reg_gain);
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

	/*  */
	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");

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

	write_cmos_sensor_16_8(0x0104, 0x01);
	/* Global analog Gain for Long expo */
	write_cmos_sensor_16_8(0x0204, (reg_gain >> 8) & 0xFF);
	write_cmos_sensor_16_8(0x0205, reg_gain & 0xFF);
	write_cmos_sensor_16_8(0x0104, 0x00);


	return gain;
}				/*    set_gain  */



static void set_mirror_flip(kal_uint8 image_mirror)
{
	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor_16_8(0x0101, image_mirror);
		break;

	case IMAGE_V_MIRROR:
		write_cmos_sensor_16_8(0x0101, image_mirror);
		break;

	case IMAGE_H_MIRROR:
		write_cmos_sensor_16_8(0x0101, image_mirror);
		break;

	case IMAGE_HV_MIRROR:
		write_cmos_sensor_16_8(0x0101, image_mirror);
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
#if 0
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/
#endif

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
	{
		write_cmos_sensor_16_8(0x0100, 0x01);
	}
	else
	{
		write_cmos_sensor_16_8(0x0100, 0x00);
	}
	return ERROR_NONE;
}

static kal_uint16 addr_data_pair_init[] = {
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
	0x0340, 0x07,
	0x0341, 0x1E,
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
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0x46,
	0x030B, 0x01,
	0x030D, 0x02,
	0x030E, 0x00,
	0x030F, 0x1E,
	0x0310, 0x01,
	0x3F78, 0x01,
	0x3F79, 0x31,
	0x3FFE, 0x00,
	0x3FFF, 0x8A,
	0x5F0A, 0xB6,
	0x0202, 0x07,
	0x0203, 0x0C,
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
	0x0340, 0x07,
	0x0341, 0x1E,
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
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0x46,
	0x030B, 0x01,
	0x030D, 0x02,
	0x030E, 0x00,
	0x030F, 0x1E,
	0x0310, 0x01,
	0x3F78, 0x01,
	0x3F79, 0x31,
	0x3FFE, 0x00,
	0x3FFF, 0x8A,
	0x5F0A, 0xB6,
	0x0202, 0x07,
	0x0203, 0x0C,
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x3F15, 0x00,
};

static kal_uint16 addr_data_pair_normal_video[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x0A,
	0x0343, 0x00,
	0x0340, 0x05,
	0x0341, 0xE6,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x01,
	0x0347, 0xBC,
	0x0348, 0x12,
	0x0349, 0x2F,
	0x034A, 0x0B,
	0x034B, 0xEB,
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
	0x040B, 0x04,
	0x040C, 0x09,
	0x040D, 0x00,
	0x040E, 0x05,
	0x040F, 0x10,
	0x034C, 0x09,
	0x034D, 0x00,
	0x034E, 0x05,
	0x034F, 0x10,
	0x0301, 0x06,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0x3A,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0x8B,
	0x0310, 0x01,
	0x3F78, 0x01,
	0x3F79, 0x31,
	0x3FFE, 0x00,
	0x3FFF, 0x8A,
	0x5F0A, 0xB6,
	0x0202, 0x05,
	0x0203, 0xD4,
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x3F15, 0x00,
};

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
	0x0347, 0x14,
	0x0348, 0x12,
	0x0349, 0x2F,
	0x034A, 0x0D,
	0x034B, 0x93,
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
	0x0306, 0x01,
	0x0307, 0x22,
	0x030B, 0x01,
	0x030D, 0x06,
	0x030E, 0x01,
	0x030F, 0x6E,
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
	0x0341, 0x1E,
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
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0x46,
	0x030B, 0x01,
	0x030D, 0x02,
	0x030E, 0x00,
	0x030F, 0x1E,
	0x0310, 0x01,
	0x3F78, 0x01,
	0x3F79, 0x31,
	0x3FFE, 0x00,
	0x3FFF, 0x8A,
	0x5F0A, 0xB6,
	0x0202, 0x07,
	0x0203, 0x0C,
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
	0x0340, 0x07,
	0x0341, 0x38,
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
	0x0307, 0x47,
	0x030B, 0x02,
	0x030D, 0x06,
	0x030E, 0x00,
	0x030F, 0xB7,
	0x0310, 0x01,
	0x3F78, 0x01,
	0x3F79, 0x31,
	0x3FFE, 0x00,
	0x3FFF, 0x8A,
	0x5F0A, 0xB6,
	0x0202, 0x07,
	0x0203, 0x26,
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

static void sensor_init(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(addr_data_pair_init,
		   sizeof(addr_data_pair_init) / sizeof(kal_uint16));
}				/*    sensor_init  */

static void preview_setting(void)
{
	LOG_INF("2304x1728_30fps E\n");
	table_write_cmos_sensor(addr_data_pair_preview,
		   sizeof(addr_data_pair_preview) / sizeof(kal_uint16));

}				/*    preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! 4608x3456_30fps currefps:%d\n", currefps);
	table_write_cmos_sensor(addr_data_pair_capture,
		   sizeof(addr_data_pair_capture) / sizeof(kal_uint16));

}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! 4608x2592_30fps  currefps:%d\n", currefps);
	table_write_cmos_sensor(addr_data_pair_normal_video,
		   sizeof(addr_data_pair_normal_video) / sizeof(kal_uint16));
}

static void hs_video_setting(void)
{
	LOG_INF("E 1920x1080_120fps \n");
	table_write_cmos_sensor(addr_data_pair_hs_video,
		   sizeof(addr_data_pair_hs_video) / sizeof(kal_uint16));
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(addr_data_pair_normal_video,
		   sizeof(addr_data_pair_normal_video) / sizeof(kal_uint16));
}

static void custom1_setting(void)
{
	LOG_INF("4:3 full size start\n");
	table_write_cmos_sensor(addr_data_pair_custom1,
		   sizeof(addr_data_pair_custom1) / sizeof(kal_uint16));
	LOG_INF("4:3 full size end\n");
}

static void custom2_setting(void)
{
	LOG_INF("4:3 binning size start\n");
	table_write_cmos_sensor(addr_data_pair_custom2,
		   sizeof(addr_data_pair_custom2) / sizeof(kal_uint16));
	LOG_INF("4:3 binning size end\n");
}

static void custom3_setting(void)
{
	LOG_INF("4:3 binning size start\n");
	table_write_cmos_sensor(addr_data_pair_custom3,
		   sizeof(addr_data_pair_custom3) / sizeof(kal_uint16));
	LOG_INF("4:3 binning size end\n");
}

static void custom4_setting(kal_uint16 currefps)
{
	LOG_INF("E! 4608x3456_30fps currefps:%d\n", currefps);
	table_write_cmos_sensor(addr_data_pair_custom4,
		   sizeof(addr_data_pair_custom4) / sizeof(kal_uint16));

}


static kal_uint32 set_test_pattern_mode(kal_uint8 modes, struct SET_SENSOR_PATTERN_SOLID_COLOR *pTestpatterndata)
{
    kal_uint16 Color_R, Color_Gr, Color_Gb, Color_B;
    pr_debug("set_test_pattern enum: %d\n", modes);

    if (modes){
        write_cmos_sensor_16_8(0x0600, modes>>4);
        write_cmos_sensor_16_8(0x0601, modes);
        if (modes == 1 && (pTestpatterndata != NULL)) { //Solid Color
            Color_R = (pTestpatterndata->COLOR_R >> 16) & 0xFFFF;
            Color_Gr = (pTestpatterndata->COLOR_Gr >> 16) & 0xFFFF;
            Color_B = (pTestpatterndata->COLOR_B >> 16) & 0xFFFF;
            Color_Gb = (pTestpatterndata->COLOR_Gb >> 16) & 0xFFFF;
            write_cmos_sensor_16_8(0x0602, Color_R >> 8);
            write_cmos_sensor_16_8(0x0603, Color_R & 0xFF);
            write_cmos_sensor_16_8(0x0602, Color_Gr >> 8);
            write_cmos_sensor_16_8(0x0603, Color_Gr & 0xFF);
            write_cmos_sensor_16_8(0x0602, Color_B >> 8);
            write_cmos_sensor_16_8(0x0603, Color_B & 0xFF);
            write_cmos_sensor_16_8(0x0602, Color_Gb >> 8);
            write_cmos_sensor_16_8(0x0603, Color_Gb & 0xFF);
        }
    }
    else {
        write_cmos_sensor_16_8(0x0600, 0x0000); /*No pattern*/
    }
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = modes;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}



static kal_uint32 return_sensor_id(void)
{
	return read_cmos_sensor_16_16(0x0016);
}

#if 0
static void read_dpc_data(void)
{
	int i;
	DPC_DATA[0] = read_cmos_sensor_16_8(0x7678); /* FD_DFCT_NUM size */
	/*LOG_INF("DPC_DATA[0]  = 0x%x, read_cmos_sensor_16_8(0x7678) = 0x%x\n", DPC_DATA[0], read_cmos_sensor_16_8(0x7678));	*/
	DPC_DATA[1] = read_cmos_sensor_16_8(0x767A); /* SG_DFCT_NUM size */
	/*LOG_INF("DPC_DATA[1]  = 0x%x, read_cmos_sensor_16_8(0x767A) = 0x%x\n", DPC_DATA[1], read_cmos_sensor_16_8(0x767A));	*/
	DPC_DATA[2] = read_cmos_sensor_16_8(0x767B); /* SG_DFCT_NUM size */
	/*LOG_INF("DPC_DATA[2]  = 0x%x, read_cmos_sensor_16_8(0x767B) = 0x%x\n", DPC_DATA[2], read_cmos_sensor_16_8(0x767B));	*/
	for(i = 3 ; i < (0x3B7+3);i ++){
		DPC_DATA[i] = read_cmos_sensor_16_8(i -3 + 0x8B00);
		LOG_INF("DPC_DATA[%d]  = 0x%x\n", i, DPC_DATA[i]);
	}
}
#endif

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
		*sensor_id = return_sensor_id();
		printk("imx471_i2c_write_id = 0x%x,imx471_sensor_id = 0x%x\n",imgsensor.i2c_write_id,*sensor_id);
		if (*sensor_id == IMX471_SENSOR_ID1) {
				*sensor_id = imgsensor_info.sensor_id;
				printk("IMX471 get_imgsensor_id success: 0x%x\n", *sensor_id);
				#ifdef OPLUS_FEATURE_CAMERA_COMMON
				imgsensor_info.module_id = read_module_id();
				if (deviceInfo_register_value == 0x00) {
					Oplusimgsensor_Registdeviceinfo("Cam_f", sensor_name_golden_result, imgsensor_info.module_id);
					deviceInfo_register_value=0x01;
				}
				#endif	//OPLUS_FEATURE_CAMERA_COMMON

				return ERROR_NONE;
		}
		retry--;
	} while (retry > 0);
	i++;
	retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		printk("IMX471 get_imgsensor_id failed: 0x%x\n", *sensor_id);
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

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == IMX471_SENSOR_ID1) {
				sensor_id = imgsensor_info.sensor_id;
				printk("IMX471 open success: 0x%x\n", sensor_id);
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
		printk("IMX471 open failed: 0x%x\n", sensor_id);
		return ERROR_SENSORID_READ_FAIL;
	}
	#else
	if (imgsensor_info.sensor_id !=  sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	#endif
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
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = 0;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

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
	/* imgsensor.video_mode = KAL_FALSE; */
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
	/*preview_setting();*/
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

static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

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
}	/*	custom1   */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

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
}	/*	custom2   */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

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
}	/*	custom3   */

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
	custom4_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* custom4 */

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
	sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame; /* The delay frame of setting frame length  */

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
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x*/
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x*/
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
		Custom1(image_window, sensor_config_data);
		break;
    case MSDK_SCENARIO_ID_CUSTOM2:
		Custom2(image_window, sensor_config_data);
		break;
    case MSDK_SCENARIO_ID_CUSTOM3:
		Custom3(image_window, sensor_config_data);
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
	if (framerate == 0)
		/* Dynamic frame rate*/
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
	if (enable) /*enable auto flicker*/
		imgsensor.autoflicker_en = KAL_TRUE;
	else /*Cancel Auto flick*/
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
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			LOG_INF("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
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
		else {
			/*No need to set*/
			LOG_INF("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
		    set_dummy();
		else {
			/*No need to set*/
			LOG_INF("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
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
		else {
			/*No need to set*/
			LOG_INF("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
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
		else {
			/*No need to set*/
			LOG_INF("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		frame_length = imgsensor_info.custom4.pclk / framerate * 10 / imgsensor_info.custom4.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength) ? (frame_length - imgsensor_info.custom4.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	/* coding with  preview scenario by default */
	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			LOG_INF("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
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

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static void read_4cell_from_eeprom(char *data)
{
	int i = 0;
	int addr = FOUR_CELL_QSC_START;/*Start of 4 cell data*/
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	data[0] = (FOUR_CELL_QSC_SIZE & 0xff);/*Low*/
	data[1] = ((FOUR_CELL_QSC_SIZE >> 8) & 0xff);/*High*/

	for (i = 2; i < (FOUR_CELL_QSC_SIZE + 2); i++) {
		pu_send_cmd[0] = (char)(addr >> 8);
		pu_send_cmd[1] = (char)(addr & 0xFF);
		iReadRegI2C(pu_send_cmd, 2, &data[i], 1, 0xA8);
		addr++;
	}
}
#endif

#if 0 //PHB
static void read_4cell_gain_tbl_from_eeprom(char *data)
{
    int i = 0;

    data[0] = (FOUR_CELL_QSC_SIZE & 0xff);/*Low*/
    data[1] = ((FOUR_CELL_QSC_SIZE >> 8) & 0xff);/*High*/

    for (i = 2; i < (FOUR_CELL_QSC_SIZE + 2); i++){
		data[i] = QSC_DATA[i -2];
		LOG_INF("read QSC data[%d] = %02x\n",i,data[i]);
    }
}


static void read_4cell_dpc_from_eeprom(char *data)
{
  	int i = 0;

  	data[0] = DPC_DATA[2];/*Low*/
  	data[1] = DPC_DATA[1];/*High*/

  	for (i = 2; i < (FOUR_CELL_DPC_SIZE + 2); i++){
		data[i] = DPC_DATA[i-2+19];
		LOG_INF("read dpc data[%d] = %02x\n",i,data[i]);
  	}
}

static void read_4cell_fd_dpc_from_eeprom(char *data)
{
  	int i = 0;

  	data[0] = DPC_DATA[0] ;/*Low*/
  	data[1] = 0;/*High*/

  	for (i = 2; i < (FOUR_CELL_FD_DPC_SIZE + 2); i++){
		data[i] = DPC_DATA[i-2+3];
		LOG_INF("read fd dpc data[%d] = 0x%02x\n",i,data[i]);
  	}
}
#endif
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

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
    	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
    		*(feature_data + 1) = 1; /* margin info by scenario */
    		*(feature_data + 2) = imgsensor_info.margin;
    		break;
	#endif
        case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
            *(MINT32 *)(signed long)(*(feature_data + 1)) = 6590000;
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
		write_cmos_sensor_16_8(
			sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor_16_8(sensor_reg_data->RegAddr);
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
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
	             set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
	             get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	/*case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
		read_2L9_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
		break;*/

	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((UINT8)*feature_data, (struct SET_SENSOR_PATTERN_SOLID_COLOR *) feature_data+1);
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
	case SENSOR_FEATURE_GET_CROP_INFO:
	    LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data_32);
	    wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

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
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[5],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[6],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[7],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[8],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
		}
					break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
		PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				//memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(SET_PD_BLOCK_INFO_T));
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
		/*PDAF capacity enable or not, 2p8 only full size support PDAF*/
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0; /* video & capture use same setting*/
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
		}
		break;
#if 0
	case SENSOR_FEATURE_GET_CUSTOM_INFO:
	#if 1
	    LOG_INF("SENSOR_FEATURE_GET_CUSTOM_INFO information type:%lld  IMX471_OTP_ERROR_CODE:%d \n", *feature_data,IMX471_OTP_ERROR_CODE);
		switch (*feature_data) {
			case 0:    //info type: otp state
			LOG_INF("*feature_para_len = %d, sizeof(MUINT32)*13 + 2 =%ld, \n", *feature_para_len, sizeof(MUINT32)*13 + 2);
			if (*feature_para_len >= sizeof(MUINT32)*13 + 2) {
			    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = IMX471_OTP_ERROR_CODE;//otp_state
				memcpy( feature_data+2, sn_inf_sub_imx471, sizeof(MUINT32)*13);
				#if 0
						for (i = 0 ; i<12 ; i++ ){
						LOG_INF("sn_inf_sub_imx471[%d]= 0x%x\n", i, sn_inf_sub_imx471[i]);
						}
				#endif
			}
				break;
		}
		break;
    #endif
#endif
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
	case SENSOR_FEATURE_GET_4CELL_DATA:/*get 4 cell data from eeprom*/
	{
		/*get 4 cell data from eeprom*/
		int type = (kal_uint16)(*feature_data);
		char *data = (char *)(uintptr_t)(*(feature_data+1));

		memset(data, 0, FOUR_CELL_QSC_SIZE);

		if (type == FOUR_CELL_CAL_TYPE_GAIN_TBL) {
			read_4cell_from_eeprom(data);
			LOG_INF(
				"read Cross Talk = %02x %02x %02x %02x %02x %02x\n",
				(UINT16)data[0], (UINT16)data[1],
				(UINT16)data[2], (UINT16)data[3],
				(UINT16)data[4], (UINT16)data[5]);
		}
		break;
	}

#if 0 //PHB
	case SENSOR_FEATURE_GET_4CELL_DATA:/*get 4 cell data from eeprom*/
	{
		/*get 4 cell data from eeprom*/
		int type = (kal_uint16)(*feature_data);
		char *data = (char *)(uintptr_t)(*(feature_data+1));

		/*only copy QSC calibration data*/
		if (type == FOUR_CELL_CAL_TYPE_GAIN_TBL) {
			memset(data, 0, FOUR_CELL_QSC_SIZE);
			read_4cell_gain_tbl_from_eeprom(data);
			LOG_INF("read QSC calibration data  size data[0] = 0x%02x, data[1] = 0x%02x\n",(UINT16)data[0], (UINT16)data[1]);
		} else if (type == FOUR_CELL_CAL_TYPE_DPC) {
			memset(data, 0, FOUR_CELL_DPC_SIZE);
			read_4cell_dpc_from_eeprom(data);
			LOG_INF("read DPC calibration data size data[0] = 0x%02x, data[1] = 0x%02x\n",(UINT16)data[0], (UINT16)data[1]);
		} else if (type == FOUR_CELL_CAL_TYPE_FD_DPC) {
			memset(data, 0, FOUR_CELL_FD_DPC_SIZE);
			read_4cell_fd_dpc_from_eeprom(data);
			LOG_INF("read FD DPC calibration data size data[0] = 0x%02x, data[1] = 0x%02x\n",(UINT16)data[0], (UINT16)data[1]);
		}
		break;
	}
#endif

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
               default:
                       rate = 0;
                       break;
               }
               *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
        }
        break;

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


UINT32 IMX471_MIPI_RAW2_20730_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!= NULL)
		*pfFunc= &sensor_func;
	return ERROR_NONE;
}	/*	IMX471_MIPI_RAW_SensorInit	*/
