/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 Hi846_mipi_raw_Sensor.c
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

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"
#include "hi846q2rmipiraw_Sensor.h"
#include "imgsensor_common.h"
#include "imgsensor_eeprom.h"

#define PFX "HI846Q2R_camera_sensor"
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#define Hi846_MaxGain 16
#define HI846Q2R_OTP_OB 64
#define USE_BURST_MODE  1

static DEFINE_SPINLOCK(imgsensor_drv_lock);

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

#define LONGEXP1	0xFFFF9   /* 最大曝光行（20bit） */
#define LONGEXP2	0xFFF9   /* 65529 lines, =0.83s */


#ifdef OPLUS_FEATURE_CAMERA_COMMON
static kal_uint32 streaming_control(kal_bool enable);
#define MODULE_ID_OFFSET 0x0000
#endif

static imgsensor_info_struct imgsensor_info = {
	.sensor_id = HI846Q2R_SENSOR_ID,
	.checksum_value = 0xdf4593fd,

	.pre = {
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 2526,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate =  288000000,//720Mbps*4/10=720 000 000 *4/10=288000000
	/*	.pclk = 288000000,				//record different mode's pclk
		.linelength = 3800,				//record different mode's linelength
		.framelength = 2526,			//record different mode's framelength
		.startx =0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1632,		//record different mode's width of grabwindow
		.grabwindow_height = 1224,		//record different mode's height of grabwindow
		//	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario
		.mipi_data_lp2hs_settle_dc = 85,
		//	 following for GetDefaultFramerateByScenario()
		.max_framerate = 300,
		.mipi_pixel_rate =  144000000,//360Mbps*4/10=360 000 000 *4/10=144000000
		//V-blanking=(2526-1224)*3800/288000000=0.0171791666666667s=17.2ms*/
	},

	.cap = {
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 2526,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate =  288000000,//720Mbps*4/10=720 000 000 *4/10=288000000
		//V-blanking=(2526-2448)*3800/288000000=0.0010291666666667s=1.03ms
	},

	.cap1 = {                            //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 5052,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 150,
		.mipi_pixel_rate =  288000000,//720Mbps*4/10=720 000 000 *4/10=288000000
		//V-blanking=(5052-2448)*3800/288000000=0.0343583333333333s=34.4ms
    },

	.normal_video = {
		.pclk = 288000000,				//record different mode's pclk
		.linelength = 3800,				//record different mode's linelength
		.framelength = 2526,			//record different mode's framelength
		.startx =0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 3264,		//record different mode's width of grabwindow
		.grabwindow_height = 1840,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
		.mipi_pixel_rate =  288000000,//720Mbps*4/10=720 000 000 *4/10=288000000
		//V-blanking=(2526-2448)*3800/288000000=0.0010291666666667s=1.03ms
	},

	.hs_video = {
		.pclk = 288000000,				//record different mode's pclk
		.linelength = 3800, 			//record different mode's linelength
		.framelength = 2526,			//record different mode's framelength
		.startx =0, 				//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1632,		//record different mode's width of grabwindow
		.grabwindow_height = 1224,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
		.mipi_pixel_rate =	144000000,//360Mbps*4/10=360 000 000 *4/10=144000000
		//V-blanking=(2526-1224)*3800/288000000=0.0171791666666667s=17.2ms

	},

    .slim_video = {
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 842,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 900,
		.mipi_pixel_rate =	144000000,//360Mbps*4/10=360 000 000 *4/10=288000000
		//V-blanking=(631-480)*3800/288000000=0.0016097222222222s=1.6ms
    },

	.custom1 = {
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 3156,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,
		.mipi_pixel_rate =  288000000,//720Mbps*4/10=720 000 000 *4/10=288000000
		//V-blanking=(2526-2448)*3800/288000000=0.0010291666666667s=1.03ms
	},
	.custom2 = {
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 1263,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 600,
		.mipi_pixel_rate =  720000000,//720Mbps*4/10=720 000 000 *4/10=288000000
		//V-blanking=(2526-2448)*3800/288000000=0.0010291666666667s=1.03ms
	},

	.margin = 6,
	.min_shutter = 6,
	.min_gain = 64, /*1x gain*/
	.max_gain = 1024, /*16x gain*/
	.min_gain_iso = 100,
	.gain_step = 64,
	.gain_type = 3,/*to be modify,no gain table for hynix*/
	.max_frame_length = 0xfffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0, // per-frame : 0 , No perframe : 1
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,      //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 7,	  //support sensor mode num

	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 3,    //enter high speed video  delay frame num
	.slim_video_delay_frame = 3,//enter slim video delay frame num
	.custom1_delay_frame = 3,//enter slim video delay frame num
	.custom2_delay_frame = 3,//custom2 video delay frame num
	.frame_time_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x45, 0x44,0xff},
	.i2c_speed = 400,
};

static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x0100,					//current shutter
	.gain = 0xe0,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_TRUE for in test pattern mode, KAL_FALSE for normal output
	.enable_secure = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0,
	.i2c_write_id = 0x44,
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[7] =
{
  //{ 3264, 2448,   0,   0, 3264, 2448, 1632, 1224,   0, 0, 1632, 1224, 0, 0, 1632, 1224}, // previeow
  { 3264, 2448,   0,   0, 3264, 2448, 3264, 2448,   0, 0, 3264, 2448, 0, 0, 3264, 2448}, // capture
  { 3264, 2448,   0,   0, 3264, 2448, 3264, 2448,   0, 0, 3264, 2448, 0, 0, 3264, 2448}, // capture
  { 3264, 2448,   0, 304, 3264, 1840, 3264, 1840,   0, 0, 3264, 1840, 0, 0, 3264, 1840}, // video
  { 3264, 2448,   0,   0, 3264, 2448, 1632, 1224,   0, 0, 1632, 1224, 0, 0, 1632, 1224}, //hight speed video
  { 3264, 2448,   0, 504, 3264, 1440, 1632,  720, 176, 0, 1280,  720, 0, 0, 1280,  720}, // slim video
  { 3264, 2448,   0, 360, 3264, 1728, 3264, 1728, 480, 0, 2304, 1728, 0, 0, 2304, 1728}, // custom1
  { 3264, 2448,   0, 684, 3264, 1080, 3264, 1080, 672, 0, 1920, 1080, 0, 0, 1920, 1080}  // custom2
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };

	//kdSetI2CSpeed(400);

	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8),(char)(para & 0xFF)};
	//kdSetI2CSpeed(400);
	iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id);
}

static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	//kdSetI2CSpeed(400);
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
static kal_uint8 gHi846_SN[CAMERA_MODULE_SN_LENGTH];
static void read_eeprom_SN(void)
{
	kal_uint16 idx = 0;
	kal_uint8 *get_byte= &gHi846_SN[0];
	for (idx = 0; idx <CAMERA_MODULE_SN_LENGTH; idx++) {
		char pusendcmd[2] = {0x00 , (char)((0xB0 + idx) & 0xFF) };
		iReadRegI2C(pusendcmd , 2, (u8*)&get_byte[idx],1, 0xA0);
		LOG_INF("gHi846_SN[%d]: 0x%x  0x%x\n", idx, get_byte[idx], gHi846_SN[idx]);
	}
}

#define   WRITE_DATA_MAX_LENGTH                (16)
#define   HI846Q2R_STEREP_DATA_ADDR_START         (0x1E80)
#define   HI846Q2R_STEREP_DATA_LENGTH             (1561)
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

static kal_uint16 read_cmos_eeprom_8(kal_uint16 addr)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 1, 0xA0);
	return get_byte;
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
		pr_debug("HI846Q2R SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
                       pStereodata->uSensorId,
                       pStereodata->uDeviceId,
                       pStereodata->baseAddr,
                       pStereodata->dataLength);

		data_base = pStereodata->baseAddr;
		data_length = pStereodata->dataLength;
		pData = pStereodata->uData;
		if ((pStereodata->uSensorId == HI846Q2R_SENSOR_ID)
				&& (data_base == HI846Q2R_STEREP_DATA_ADDR_START)
				&& (data_length == HI846Q2R_STEREP_DATA_LENGTH)) {
			pr_debug("HI846Q2R Write: %x %x %x %x %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556],
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
			pr_debug("com 0x1600:0x%x\n", read_cmos_eeprom_8(HI846Q2R_STEREP_DATA_ADDR_START));
			msleep(6);
			pr_debug("com 0x1627:0x%x\n", read_cmos_eeprom_8(HI846Q2R_STEREP_DATA_ADDR_START+39));
			msleep(6);
			pr_debug("innal 0x1628:0x%x\n", read_cmos_eeprom_8(HI846Q2R_STEREP_DATA_ADDR_START+40));
			msleep(6);
			pr_debug("innal 0x1C14:0x%x\n", read_cmos_eeprom_8(HI846Q2R_STEREP_DATA_ADDR_START+1556));
			msleep(6);
			pr_debug("tail1 0x1C15:0x%x\n", read_cmos_eeprom_8(HI846Q2R_STEREP_DATA_ADDR_START+1557));
			msleep(6);
			pr_debug("tail2 0x1C16:0x%x\n", read_cmos_eeprom_8(HI846Q2R_STEREP_DATA_ADDR_START+1558));
			msleep(6);
			pr_debug("tail3 0x1C17:0x%x\n", read_cmos_eeprom_8(HI846Q2R_STEREP_DATA_ADDR_START+1559));
			msleep(6);
			pr_debug("tail4 0x1C18:0x%x\n", read_cmos_eeprom_8(HI846Q2R_STEREP_DATA_ADDR_START+1560));
			msleep(6);
			pr_debug("HI846Q2R write_Module_data Write end\n");
		}else {
			pr_err("Invalid Sensor id:0x%x write_gm1 eeprom\n", pStereodata->uSensorId);
			return IMGSENSOR_RETURN_ERROR;
		}
	} else {
		pr_err("HI846Q2R write_Module_data pStereodata is null\n");
		return IMGSENSOR_RETURN_ERROR;
	}
	return ret;
}
#endif

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor(0x0006, imgsensor.frame_length & 0xFFFF );
	write_cmos_sensor(0x0008, imgsensor.line_length & 0xFFFF );

}	/*	set_dummy  */
/*
static void set_mirror_flip(kal_uint8 image_mirror)
{
	kal_uint8 itemp;

	LOG_INF("image_mirror = %d\n", image_mirror);
	itemp = read_cmos_sensor(0x000E);
	itemp &= ~0x0300;

	switch (image_mirror) {

	case IMAGE_NORMAL:
	write_cmos_sensor(0x000E, itemp);
	break;

	case IMAGE_V_MIRROR:
	write_cmos_sensor_8(0x000E, itemp | 0x0200);
	break;

	case IMAGE_H_MIRROR:
	write_cmos_sensor_8(0x000E, itemp | 0x0100);
	break;

	case IMAGE_HV_MIRROR:
	write_cmos_sensor_8(0x000E, itemp | 0x0300);
	break;
	}
}
*/
static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x0F17) << 8) | read_cmos_sensor(0x0F16));
}
static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	//kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable = %d\n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
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

	//kal_uint32 frame_length = 0;

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	LOG_INF("shutter %d line %d\n", shutter, __LINE__);
	spin_lock(&imgsensor_drv_lock);
	if (shutter > LONGEXP1) {
		shutter = LONGEXP1;
		imgsensor.frame_length = 2526;
	} else if (shutter > LONGEXP2) {
		imgsensor.frame_length = 2526;
	} else if (shutter > imgsensor.min_frame_length - imgsensor_info.margin) {
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	} else {
		imgsensor.frame_length = imgsensor.min_frame_length;
	}
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk * 10 / (imgsensor.line_length * imgsensor.frame_length);
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			write_cmos_sensor(0x0006, imgsensor.frame_length);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0006, imgsensor.frame_length);
	}

	// Update Shutter
	// Update Shutter
	write_cmos_sensor_8(0x0073, ((shutter & 0xFF0000) >> 16));
	write_cmos_sensor(0x0074, shutter & 0x00FFFF);
	LOG_INF("shutter =%d, framelength =%d", shutter,imgsensor.frame_length);

}	/* write_shutter  */

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

	LOG_INF("set_shutter");

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */

/*************************************************************************
 * FUNCTION
 *	set_shutter_frame_length
 *
 * DESCRIPTION
 *	for frame & 3A sync
 *
 *************************************************************************/
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	printk("shutter %d line %d\n", shutter, __LINE__);
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
		realtime_fps = imgsensor.pclk * 10 / (imgsensor.line_length * imgsensor.frame_length);
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			write_cmos_sensor(0x0006, imgsensor.frame_length);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0006, imgsensor.frame_length);
	}

	// Update Shutter
	write_cmos_sensor_8(0x0073, ((shutter & 0xFF0000) >> 16));
	write_cmos_sensor(0x0074, shutter & 0x00FFFF);
	LOG_INF("shutter =%d, framelength =%d", shutter,imgsensor.frame_length);

}	/* set_shutter_frame_length */


static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
	reg_gain = gain / 4 - 16;

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

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X    */
	/* [4:9] = M meams M X         */
	/* Total gain = M + N /16 X   */

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

	write_cmos_sensor_8(0x0077, reg_gain);

	return gain;

}	/*	set_gain  */

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
static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor(0x0a00, 0x0100);
		mdelay(1);
	} else {
		write_cmos_sensor(0x0a00, 0x0000);
	}
	return ERROR_NONE;
}
#if USE_BURST_MODE

#define I2C_BUFFER_LEN 225	/* trans# max is 255, each 3 bytes */
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

		/* Write when remain buffer size is less than 4 bytes or reach end of data */
		if ((I2C_BUFFER_LEN - tosend) < 4 || IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id,
								4, imgsensor_info.i2c_speed);
			tosend = 0;
		}
	}
	return 0;
}

static kal_uint16 addr_data_pair_global[] = {
0x2000, 0x987A,
0x2002, 0x00FF,
0x2004, 0x0047,
0x2006, 0x3FFF,
0x2008, 0x3FFF,
0x200A, 0xC216,
0x200C, 0x1292,
0x200E, 0xC01A,
0x2010, 0x403D,
0x2012, 0x000E,
0x2014, 0x403E,
0x2016, 0x0B80,
0x2018, 0x403F,
0x201A, 0x82AE,
0x201C, 0x1292,
0x201E, 0xC00C,
0x2020, 0x4130,
0x2022, 0x43E2,
0x2024, 0x0180,
0x2026, 0x4130,
0x2028, 0x7400,
0x202A, 0x5000,
0x202C, 0x0253,
0x202E, 0x0AD1,
0x2030, 0x2360,
0x2032, 0x0009,
0x2034, 0x5020,
0x2036, 0x000B,
0x2038, 0x0002,
0x203A, 0x0044,
0x203C, 0x0016,
0x203E, 0x1792,
0x2040, 0x7002,
0x2042, 0x154F,
0x2044, 0x00D5,
0x2046, 0x000B,
0x2048, 0x0019,
0x204A, 0x1698,
0x204C, 0x000E,
0x204E, 0x099A,
0x2050, 0x0058,
0x2052, 0x7000,
0x2054, 0x1799,
0x2056, 0x0310,
0x2058, 0x03C3,
0x205A, 0x004C,
0x205C, 0x064A,
0x205E, 0x0001,
0x2060, 0x0007,
0x2062, 0x0BC7,
0x2064, 0x0055,
0x2066, 0x7000,
0x2068, 0x1550,
0x206A, 0x158A,
0x206C, 0x0004,
0x206E, 0x1488,
0x2070, 0x7010,
0x2072, 0x1508,
0x2074, 0x0004,
0x2076, 0x0016,
0x2078, 0x03D5,
0x207A, 0x0055,
0x207C, 0x08CA,
0x207E, 0x2019,
0x2080, 0x0007,
0x2082, 0x7057,
0x2084, 0x0FC7,
0x2086, 0x5041,
0x2088, 0x12C8,
0x208A, 0x5060,
0x208C, 0x5080,
0x208E, 0x2084,
0x2090, 0x12C8,
0x2092, 0x7800,
0x2094, 0x0802,
0x2096, 0x040F,
0x2098, 0x1007,
0x209A, 0x0803,
0x209C, 0x080B,
0x209E, 0x3803,
0x20A0, 0x0807,
0x20A2, 0x0404,
0x20A4, 0x0400,
0x20A6, 0xFFFF,
0x20A8, 0xF0B2,
0x20AA, 0xFFEF,
0x20AC, 0x0A84,
0x20AE, 0x1292,
0x20B0, 0xC02E,
0x20B2, 0x4130,
0x20B4, 0xF0B2,
0x20B6, 0xFFBF,
0x20B8, 0x2004,
0x20BA, 0x403F,
0x20BC, 0x00C3,
0x20BE, 0x4FE2,
0x20C0, 0x8318,
0x20C2, 0x43CF,
0x20C4, 0x0000,
0x20C6, 0x9382,
0x20C8, 0xC314,
0x20CA, 0x2003,
0x20CC, 0x12B0,
0x20CE, 0xCAB0,
0x20D0, 0x4130,
0x20D2, 0x12B0,
0x20D4, 0xC90A,
0x20D6, 0x4130,
0x20D8, 0x42D2,
0x20DA, 0x8318,
0x20DC, 0x00C3,
0x20DE, 0x9382,
0x20E0, 0xC314,
0x20E2, 0x2009,
0x20E4, 0x120B,
0x20E6, 0x120A,
0x20E8, 0x1209,
0x20EA, 0x1208,
0x20EC, 0x1207,
0x20EE, 0x1206,
0x20F0, 0x4030,
0x20F2, 0xC15E,
0x20F4, 0x4130,
0x20F6, 0x1292,
0x20F8, 0xC008,
0x20FA, 0x4130,
0x20FC, 0x42D2,
0x20FE, 0x82A1,
0x2100, 0x00C2,
0x2102, 0x1292,
0x2104, 0xC040,
0x2106, 0x4130,
0x2108, 0x1292,
0x210A, 0xC006,
0x210C, 0x42A2,
0x210E, 0x7324,
0x2110, 0x9382,
0x2112, 0xC314,
0x2114, 0x2011,
0x2116, 0x425F,
0x2118, 0x82A1,
0x211A, 0xF25F,
0x211C, 0x00C1,
0x211E, 0xF35F,
0x2120, 0x2406,
0x2122, 0x425F,
0x2124, 0x00C0,
0x2126, 0xF37F,
0x2128, 0x522F,
0x212A, 0x4F82,
0x212C, 0x7324,
0x212E, 0x425F,
0x2130, 0x82D4,
0x2132, 0xF35F,
0x2134, 0x4FC2,
0x2136, 0x01B3,
0x2138, 0x93C2,
0x213A, 0x829F,
0x213C, 0x2421,
0x213E, 0x403E,
0x2140, 0xFFFE,
0x2142, 0x40B2,
0x2144, 0xEC78,
0x2146, 0x831C,
0x2148, 0x40B2,
0x214A, 0xEC78,
0x214C, 0x831E,
0x214E, 0x40B2,
0x2150, 0xEC78,
0x2152, 0x8320,
0x2154, 0xB3D2,
0x2156, 0x008C,
0x2158, 0x2405,
0x215A, 0x4E0F,
0x215C, 0x503F,
0x215E, 0xFFD8,
0x2160, 0x4F82,
0x2162, 0x831C,
0x2164, 0x90F2,
0x2166, 0x0003,
0x2168, 0x008C,
0x216A, 0x2401,
0x216C, 0x4130,
0x216E, 0x421F,
0x2170, 0x831C,
0x2172, 0x5E0F,
0x2174, 0x4F82,
0x2176, 0x831E,
0x2178, 0x5E0F,
0x217A, 0x4F82,
0x217C, 0x8320,
0x217E, 0x3FF6,
0x2180, 0x432E,
0x2182, 0x3FDF,
0x2184, 0x421F,
0x2186, 0x7100,
0x2188, 0x4F0E,
0x218A, 0x503E,
0x218C, 0xFFD8,
0x218E, 0x4E82,
0x2190, 0x7A04,
0x2192, 0x421E,
0x2194, 0x831C,
0x2196, 0x5F0E,
0x2198, 0x4E82,
0x219A, 0x7A06,
0x219C, 0x0B00,
0x219E, 0x7304,
0x21A0, 0x0050,
0x21A2, 0x40B2,
0x21A4, 0xD081,
0x21A6, 0x0B88,
0x21A8, 0x421E,
0x21AA, 0x831E,
0x21AC, 0x5F0E,
0x21AE, 0x4E82,
0x21B0, 0x7A0E,
0x21B2, 0x521F,
0x21B4, 0x8320,
0x21B6, 0x4F82,
0x21B8, 0x7A10,
0x21BA, 0x0B00,
0x21BC, 0x7304,
0x21BE, 0x007A,
0x21C0, 0x40B2,
0x21C2, 0x0081,
0x21C4, 0x0B88,
0x21C6, 0x4392,
0x21C8, 0x7A0A,
0x21CA, 0x0800,
0x21CC, 0x7A0C,
0x21CE, 0x0B00,
0x21D0, 0x7304,
0x21D2, 0x022B,
0x21D4, 0x40B2,
0x21D6, 0xD081,
0x21D8, 0x0B88,
0x21DA, 0x0B00,
0x21DC, 0x7304,
0x21DE, 0x0255,
0x21E0, 0x40B2,
0x21E2, 0x0081,
0x21E4, 0x0B88,
0x21E6, 0x4130,
0x23FE, 0xC056,
0x3232, 0xFC0C,
0x3236, 0xFC22,
0x3238, 0xFCFC,
0x323A, 0xFD84,
0x323C, 0xFD08,
0x3246, 0xFCD8,
0x3248, 0xFCA8,
0x324E, 0xFCB4,
0x326A, 0x8302,
0x326C, 0x830A,
0x326E, 0x0000,
0x32CA, 0xFC28,
0x32CC, 0xC3BC,
0x32CE, 0xC34C,
0x32D0, 0xC35A,
0x32D2, 0xC368,
0x32D4, 0xC376,
0x32D6, 0xC3C2,
0x32D8, 0xC3E6,
0x32DA, 0x0003,
0x32DC, 0x0003,
0x32DE, 0x00C7,
0x32E0, 0x0031,
0x32E2, 0x0031,
0x32E4, 0x0031,
0x32E6, 0xFC28,
0x32E8, 0xC3BC,
0x32EA, 0xC384,
0x32EC, 0xC392,
0x32EE, 0xC3A0,
0x32F0, 0xC3AE,
0x32F2, 0xC3C4,
0x32F4, 0xC3E6,
0x32F6, 0x0003,
0x32F8, 0x0003,
0x32FA, 0x00C7,
0x32FC, 0x0031,
0x32FE, 0x0031,
0x3300, 0x0031,
0x3302, 0x82CA,
0x3304, 0xC164,
0x3306, 0x82E6,
0x3308, 0xC19C,
0x330A, 0x001F,
0x330C, 0x001A,
0x330E, 0x0034,
0x3310, 0x0000,
0x3312, 0x0000,
0x3314, 0xFC94,
0x3316, 0xC3D8,
0x0A00, 0x0000,
0x0E04, 0x0012,
0x002E, 0x1111,
0x0032, 0x1111,
0x0022, 0x0008,
0x0026, 0x0040,
0x0028, 0x0017,
0x002C, 0x09CF,
0x005C, 0x2101,
0x0006, 0x09DE,
0x0008, 0x0ED8,
0x000E, 0x0100,
0x000C, 0x0022,
0x0A22, 0x0000,
0x0A24, 0x0000,
0x0804, 0x0000,
0x0A12, 0x0CC0,
0x0A14, 0x0990,
0x0074, 0x09D8,
0x0076, 0x0000,
0x051E, 0x0000,
0x0200, 0x0400,
0x0A1A, 0x0C00,
0x0A0C, 0x0010,
0x0A1E, 0x0CCF,
0x0402, 0x0110,
0x0404, 0x00F4,
0x0408, 0x0000,
0x0410, 0x008D,
0x0412, 0x011A,
0x0414, 0x864C,
0x021C, 0x0001,
0x021E, 0x0235,
0x0C00, 0x9950,
0x0C06, 0x0021,
0x0C10, 0x0040,
0x0C12, 0x0040,
0x0C14, 0x0040,
0x0C16, 0x0040,
0x0A02, 0x0100,
0x0A04, 0x014A,
0x0418, 0x0000,
0x0128, 0x0028,
0x012A, 0xFFFF,
0x0120, 0x0046,
0x0122, 0x0376,
0x012C, 0x0020,
0x012E, 0xFFFF,
0x0124, 0x0040,
0x0126, 0x0378,
0x0746, 0x0050,
0x0748, 0x01D5,
0x074A, 0x022B,
0x074C, 0x03B0,
0x0756, 0x043F,
0x0758, 0x3F1D,
0x0B02, 0xE04D,
0x0B10, 0x6821,
0x0B12, 0x0120,
0x0B14, 0x0001,
0x2008, 0x38FD,
0x326E, 0x0000,
0x0900, 0x0300,
0x0902, 0xC319,
0x0914, 0xC109,
0x0916, 0x061A,
0x0918, 0x0407,
0x091A, 0x0A0B,
0x091C, 0x0E08,
0x091E, 0x0A00,
0x090C, 0x0427,
0x090E, 0x0059,
0x0954, 0x0089,
0x0956, 0x0000,
0x0958, 0xCA80,
0x095A, 0x9240,
0x0F08, 0x2F04,
0x0F30, 0x001F,
0x0F36, 0x001F,
0x0F04, 0x3A00,
0x0F32, 0x025A,
0x0F38, 0x025A,
0x0F2A, 0x4124,
0x006A, 0x0100,
0x004C, 0x0100,
0x0044, 0x0001,
};
#endif

static void sensor_init(void)
{
	LOG_INF("Enter init v16\n");
	#if USE_BURST_MODE
	table_write_cmos_sensor(addr_data_pair_global,
		sizeof(addr_data_pair_global) / sizeof(kal_uint16));
	#else
	write_cmos_sensor(0x2000, 0x987A);
	write_cmos_sensor(0x2002, 0x00FF);
	write_cmos_sensor(0x2004, 0x0047);
	write_cmos_sensor(0x2006, 0x3FFF);
	write_cmos_sensor(0x2008, 0x3FFF);
	write_cmos_sensor(0x200A, 0xC216);
	write_cmos_sensor(0x200C, 0x1292);
	write_cmos_sensor(0x200E, 0xC01A);
	write_cmos_sensor(0x2010, 0x403D);
	write_cmos_sensor(0x2012, 0x000E);
	write_cmos_sensor(0x2014, 0x403E);
	write_cmos_sensor(0x2016, 0x0B80);
	write_cmos_sensor(0x2018, 0x403F);
	write_cmos_sensor(0x201A, 0x82AE);
	write_cmos_sensor(0x201C, 0x1292);
	write_cmos_sensor(0x201E, 0xC00C);
	write_cmos_sensor(0x2020, 0x4130);
	write_cmos_sensor(0x2022, 0x43E2);
	write_cmos_sensor(0x2024, 0x0180);
	write_cmos_sensor(0x2026, 0x4130);
	write_cmos_sensor(0x2028, 0x7400);
	write_cmos_sensor(0x202A, 0x5000);
	write_cmos_sensor(0x202C, 0x0253);
	write_cmos_sensor(0x202E, 0x0AD1);
	write_cmos_sensor(0x2030, 0x2360);
	write_cmos_sensor(0x2032, 0x0009);
	write_cmos_sensor(0x2034, 0x5020);
	write_cmos_sensor(0x2036, 0x000B);
	write_cmos_sensor(0x2038, 0x0002);
	write_cmos_sensor(0x203A, 0x0044);
	write_cmos_sensor(0x203C, 0x0016);
	write_cmos_sensor(0x203E, 0x1792);
	write_cmos_sensor(0x2040, 0x7002);
	write_cmos_sensor(0x2042, 0x154F);
	write_cmos_sensor(0x2044, 0x00D5);
	write_cmos_sensor(0x2046, 0x000B);
	write_cmos_sensor(0x2048, 0x0019);
	write_cmos_sensor(0x204A, 0x1698);
	write_cmos_sensor(0x204C, 0x000E);
	write_cmos_sensor(0x204E, 0x099A);
	write_cmos_sensor(0x2050, 0x0058);
	write_cmos_sensor(0x2052, 0x7000);
	write_cmos_sensor(0x2054, 0x1799);
	write_cmos_sensor(0x2056, 0x0310);
	write_cmos_sensor(0x2058, 0x03C3);
	write_cmos_sensor(0x205A, 0x004C);
	write_cmos_sensor(0x205C, 0x064A);
	write_cmos_sensor(0x205E, 0x0001);
	write_cmos_sensor(0x2060, 0x0007);
	write_cmos_sensor(0x2062, 0x0BC7);
	write_cmos_sensor(0x2064, 0x0055);
	write_cmos_sensor(0x2066, 0x7000);
	write_cmos_sensor(0x2068, 0x1550);
	write_cmos_sensor(0x206A, 0x158A);
	write_cmos_sensor(0x206C, 0x0004);
	write_cmos_sensor(0x206E, 0x1488);
	write_cmos_sensor(0x2070, 0x7010);
	write_cmos_sensor(0x2072, 0x1508);
	write_cmos_sensor(0x2074, 0x0004);
	write_cmos_sensor(0x2076, 0x0016);
	write_cmos_sensor(0x2078, 0x03D5);
	write_cmos_sensor(0x207A, 0x0055);
	write_cmos_sensor(0x207C, 0x08CA);
	write_cmos_sensor(0x207E, 0x2019);
	write_cmos_sensor(0x2080, 0x0007);
	write_cmos_sensor(0x2082, 0x7057);
	write_cmos_sensor(0x2084, 0x0FC7);
	write_cmos_sensor(0x2086, 0x5041);
	write_cmos_sensor(0x2088, 0x12C8);
	write_cmos_sensor(0x208A, 0x5060);
	write_cmos_sensor(0x208C, 0x5080);
	write_cmos_sensor(0x208E, 0x2084);
	write_cmos_sensor(0x2090, 0x12C8);
	write_cmos_sensor(0x2092, 0x7800);
	write_cmos_sensor(0x2094, 0x0802);
	write_cmos_sensor(0x2096, 0x040F);
	write_cmos_sensor(0x2098, 0x1007);
	write_cmos_sensor(0x209A, 0x0803);
	write_cmos_sensor(0x209C, 0x080B);
	write_cmos_sensor(0x209E, 0x3803);
	write_cmos_sensor(0x20A0, 0x0807);
	write_cmos_sensor(0x20A2, 0x0404);
	write_cmos_sensor(0x20A4, 0x0400);
	write_cmos_sensor(0x20A6, 0xFFFF);
	write_cmos_sensor(0x20A8, 0xF0B2);
	write_cmos_sensor(0x20AA, 0xFFEF);
	write_cmos_sensor(0x20AC, 0x0A84);
	write_cmos_sensor(0x20AE, 0x1292);
	write_cmos_sensor(0x20B0, 0xC02E);
	write_cmos_sensor(0x20B2, 0x4130);
	write_cmos_sensor(0x20B4, 0xF0B2);
	write_cmos_sensor(0x20B6, 0xFFBF);
	write_cmos_sensor(0x20B8, 0x2004);
	write_cmos_sensor(0x20BA, 0x403F);
	write_cmos_sensor(0x20BC, 0x00C3);
	write_cmos_sensor(0x20BE, 0x4FE2);
	write_cmos_sensor(0x20C0, 0x8318);
	write_cmos_sensor(0x20C2, 0x43CF);
	write_cmos_sensor(0x20C4, 0x0000);
	write_cmos_sensor(0x20C6, 0x9382);
	write_cmos_sensor(0x20C8, 0xC314);
	write_cmos_sensor(0x20CA, 0x2003);
	write_cmos_sensor(0x20CC, 0x12B0);
	write_cmos_sensor(0x20CE, 0xCAB0);
	write_cmos_sensor(0x20D0, 0x4130);
	write_cmos_sensor(0x20D2, 0x12B0);
	write_cmos_sensor(0x20D4, 0xC90A);
	write_cmos_sensor(0x20D6, 0x4130);
	write_cmos_sensor(0x20D8, 0x42D2);
	write_cmos_sensor(0x20DA, 0x8318);
	write_cmos_sensor(0x20DC, 0x00C3);
	write_cmos_sensor(0x20DE, 0x9382);
	write_cmos_sensor(0x20E0, 0xC314);
	write_cmos_sensor(0x20E2, 0x2009);
	write_cmos_sensor(0x20E4, 0x120B);
	write_cmos_sensor(0x20E6, 0x120A);
	write_cmos_sensor(0x20E8, 0x1209);
	write_cmos_sensor(0x20EA, 0x1208);
	write_cmos_sensor(0x20EC, 0x1207);
	write_cmos_sensor(0x20EE, 0x1206);
	write_cmos_sensor(0x20F0, 0x4030);
	write_cmos_sensor(0x20F2, 0xC15E);
	write_cmos_sensor(0x20F4, 0x4130);
	write_cmos_sensor(0x20F6, 0x1292);
	write_cmos_sensor(0x20F8, 0xC008);
	write_cmos_sensor(0x20FA, 0x4130);
	write_cmos_sensor(0x20FC, 0x42D2);
	write_cmos_sensor(0x20FE, 0x82A1);
	write_cmos_sensor(0x2100, 0x00C2);
	write_cmos_sensor(0x2102, 0x1292);
	write_cmos_sensor(0x2104, 0xC040);
	write_cmos_sensor(0x2106, 0x4130);
	write_cmos_sensor(0x2108, 0x1292);
	write_cmos_sensor(0x210A, 0xC006);
	write_cmos_sensor(0x210C, 0x42A2);
	write_cmos_sensor(0x210E, 0x7324);
	write_cmos_sensor(0x2110, 0x9382);
	write_cmos_sensor(0x2112, 0xC314);
	write_cmos_sensor(0x2114, 0x2011);
	write_cmos_sensor(0x2116, 0x425F);
	write_cmos_sensor(0x2118, 0x82A1);
	write_cmos_sensor(0x211A, 0xF25F);
	write_cmos_sensor(0x211C, 0x00C1);
	write_cmos_sensor(0x211E, 0xF35F);
	write_cmos_sensor(0x2120, 0x2406);
	write_cmos_sensor(0x2122, 0x425F);
	write_cmos_sensor(0x2124, 0x00C0);
	write_cmos_sensor(0x2126, 0xF37F);
	write_cmos_sensor(0x2128, 0x522F);
	write_cmos_sensor(0x212A, 0x4F82);
	write_cmos_sensor(0x212C, 0x7324);
	write_cmos_sensor(0x212E, 0x425F);
	write_cmos_sensor(0x2130, 0x82D4);
	write_cmos_sensor(0x2132, 0xF35F);
	write_cmos_sensor(0x2134, 0x4FC2);
	write_cmos_sensor(0x2136, 0x01B3);
	write_cmos_sensor(0x2138, 0x93C2);
	write_cmos_sensor(0x213A, 0x829F);
	write_cmos_sensor(0x213C, 0x2421);
	write_cmos_sensor(0x213E, 0x403E);
	write_cmos_sensor(0x2140, 0xFFFE);
	write_cmos_sensor(0x2142, 0x40B2);
	write_cmos_sensor(0x2144, 0xEC78);
	write_cmos_sensor(0x2146, 0x831C);
	write_cmos_sensor(0x2148, 0x40B2);
	write_cmos_sensor(0x214A, 0xEC78);
	write_cmos_sensor(0x214C, 0x831E);
	write_cmos_sensor(0x214E, 0x40B2);
	write_cmos_sensor(0x2150, 0xEC78);
	write_cmos_sensor(0x2152, 0x8320);
	write_cmos_sensor(0x2154, 0xB3D2);
	write_cmos_sensor(0x2156, 0x008C);
	write_cmos_sensor(0x2158, 0x2405);
	write_cmos_sensor(0x215A, 0x4E0F);
	write_cmos_sensor(0x215C, 0x503F);
	write_cmos_sensor(0x215E, 0xFFD8);
	write_cmos_sensor(0x2160, 0x4F82);
	write_cmos_sensor(0x2162, 0x831C);
	write_cmos_sensor(0x2164, 0x90F2);
	write_cmos_sensor(0x2166, 0x0003);
	write_cmos_sensor(0x2168, 0x008C);
	write_cmos_sensor(0x216A, 0x2401);
	write_cmos_sensor(0x216C, 0x4130);
	write_cmos_sensor(0x216E, 0x421F);
	write_cmos_sensor(0x2170, 0x831C);
	write_cmos_sensor(0x2172, 0x5E0F);
	write_cmos_sensor(0x2174, 0x4F82);
	write_cmos_sensor(0x2176, 0x831E);
	write_cmos_sensor(0x2178, 0x5E0F);
	write_cmos_sensor(0x217A, 0x4F82);
	write_cmos_sensor(0x217C, 0x8320);
	write_cmos_sensor(0x217E, 0x3FF6);
	write_cmos_sensor(0x2180, 0x432E);
	write_cmos_sensor(0x2182, 0x3FDF);
	write_cmos_sensor(0x2184, 0x421F);
	write_cmos_sensor(0x2186, 0x7100);
	write_cmos_sensor(0x2188, 0x4F0E);
	write_cmos_sensor(0x218A, 0x503E);
	write_cmos_sensor(0x218C, 0xFFD8);
	write_cmos_sensor(0x218E, 0x4E82);
	write_cmos_sensor(0x2190, 0x7A04);
	write_cmos_sensor(0x2192, 0x421E);
	write_cmos_sensor(0x2194, 0x831C);
	write_cmos_sensor(0x2196, 0x5F0E);
	write_cmos_sensor(0x2198, 0x4E82);
	write_cmos_sensor(0x219A, 0x7A06);
	write_cmos_sensor(0x219C, 0x0B00);
	write_cmos_sensor(0x219E, 0x7304);
	write_cmos_sensor(0x21A0, 0x0050);
	write_cmos_sensor(0x21A2, 0x40B2);
	write_cmos_sensor(0x21A4, 0xD081);
	write_cmos_sensor(0x21A6, 0x0B88);
	write_cmos_sensor(0x21A8, 0x421E);
	write_cmos_sensor(0x21AA, 0x831E);
	write_cmos_sensor(0x21AC, 0x5F0E);
	write_cmos_sensor(0x21AE, 0x4E82);
	write_cmos_sensor(0x21B0, 0x7A0E);
	write_cmos_sensor(0x21B2, 0x521F);
	write_cmos_sensor(0x21B4, 0x8320);
	write_cmos_sensor(0x21B6, 0x4F82);
	write_cmos_sensor(0x21B8, 0x7A10);
	write_cmos_sensor(0x21BA, 0x0B00);
	write_cmos_sensor(0x21BC, 0x7304);
	write_cmos_sensor(0x21BE, 0x007A);
	write_cmos_sensor(0x21C0, 0x40B2);
	write_cmos_sensor(0x21C2, 0x0081);
	write_cmos_sensor(0x21C4, 0x0B88);
	write_cmos_sensor(0x21C6, 0x4392);
	write_cmos_sensor(0x21C8, 0x7A0A);
	write_cmos_sensor(0x21CA, 0x0800);
	write_cmos_sensor(0x21CC, 0x7A0C);
	write_cmos_sensor(0x21CE, 0x0B00);
	write_cmos_sensor(0x21D0, 0x7304);
	write_cmos_sensor(0x21D2, 0x022B);
	write_cmos_sensor(0x21D4, 0x40B2);
	write_cmos_sensor(0x21D6, 0xD081);
	write_cmos_sensor(0x21D8, 0x0B88);
	write_cmos_sensor(0x21DA, 0x0B00);
	write_cmos_sensor(0x21DC, 0x7304);
	write_cmos_sensor(0x21DE, 0x0255);
	write_cmos_sensor(0x21E0, 0x40B2);
	write_cmos_sensor(0x21E2, 0x0081);
	write_cmos_sensor(0x21E4, 0x0B88);
	write_cmos_sensor(0x21E6, 0x4130);
	write_cmos_sensor(0x23FE, 0xC056);
	write_cmos_sensor(0x3232, 0xFC0C);
	write_cmos_sensor(0x3236, 0xFC22);
	write_cmos_sensor(0x3238, 0xFCFC);
	write_cmos_sensor(0x323A, 0xFD84);
	write_cmos_sensor(0x323C, 0xFD08);
	write_cmos_sensor(0x3246, 0xFCD8);
	write_cmos_sensor(0x3248, 0xFCA8);
	write_cmos_sensor(0x324E, 0xFCB4);
	write_cmos_sensor(0x326A, 0x8302);
	write_cmos_sensor(0x326C, 0x830A);
	write_cmos_sensor(0x326E, 0x0000);
	write_cmos_sensor(0x32CA, 0xFC28);
	write_cmos_sensor(0x32CC, 0xC3BC);
	write_cmos_sensor(0x32CE, 0xC34C);
	write_cmos_sensor(0x32D0, 0xC35A);
	write_cmos_sensor(0x32D2, 0xC368);
	write_cmos_sensor(0x32D4, 0xC376);
	write_cmos_sensor(0x32D6, 0xC3C2);
	write_cmos_sensor(0x32D8, 0xC3E6);
	write_cmos_sensor(0x32DA, 0x0003);
	write_cmos_sensor(0x32DC, 0x0003);
	write_cmos_sensor(0x32DE, 0x00C7);
	write_cmos_sensor(0x32E0, 0x0031);
	write_cmos_sensor(0x32E2, 0x0031);
	write_cmos_sensor(0x32E4, 0x0031);
	write_cmos_sensor(0x32E6, 0xFC28);
	write_cmos_sensor(0x32E8, 0xC3BC);
	write_cmos_sensor(0x32EA, 0xC384);
	write_cmos_sensor(0x32EC, 0xC392);
	write_cmos_sensor(0x32EE, 0xC3A0);
	write_cmos_sensor(0x32F0, 0xC3AE);
	write_cmos_sensor(0x32F2, 0xC3C4);
	write_cmos_sensor(0x32F4, 0xC3E6);
	write_cmos_sensor(0x32F6, 0x0003);
	write_cmos_sensor(0x32F8, 0x0003);
	write_cmos_sensor(0x32FA, 0x00C7);
	write_cmos_sensor(0x32FC, 0x0031);
	write_cmos_sensor(0x32FE, 0x0031);
	write_cmos_sensor(0x3300, 0x0031);
	write_cmos_sensor(0x3302, 0x82CA);
	write_cmos_sensor(0x3304, 0xC164);
	write_cmos_sensor(0x3306, 0x82E6);
	write_cmos_sensor(0x3308, 0xC19C);
	write_cmos_sensor(0x330A, 0x001F);
	write_cmos_sensor(0x330C, 0x001A);
	write_cmos_sensor(0x330E, 0x0034);
	write_cmos_sensor(0x3310, 0x0000);
	write_cmos_sensor(0x3312, 0x0000);
	write_cmos_sensor(0x3314, 0xFC94);
	write_cmos_sensor(0x3316, 0xC3D8);
	write_cmos_sensor(0x0A00, 0x0000);
	write_cmos_sensor(0x0E04, 0x0012);
	write_cmos_sensor(0x002E, 0x1111);
	write_cmos_sensor(0x0032, 0x1111);
	write_cmos_sensor(0x0022, 0x0008);
	write_cmos_sensor(0x0026, 0x0040);
	write_cmos_sensor(0x0028, 0x0017);
	write_cmos_sensor(0x002C, 0x09CF);
	write_cmos_sensor(0x005C, 0x2101);
	write_cmos_sensor(0x0006, 0x09DE);
	write_cmos_sensor(0x0008, 0x0ED8);
	write_cmos_sensor(0x000E, 0x0100);
	write_cmos_sensor(0x000C, 0x0022);
	write_cmos_sensor(0x0A22, 0x0000);
	write_cmos_sensor(0x0A24, 0x0000);
	write_cmos_sensor(0x0804, 0x0000);
	write_cmos_sensor(0x0A12, 0x0CC0);
	write_cmos_sensor(0x0A14, 0x0990);
	write_cmos_sensor(0x0074, 0x09D8);
	write_cmos_sensor(0x0076, 0x0000);
	write_cmos_sensor(0x051E, 0x0000);
	write_cmos_sensor(0x0200, 0x0400);
	write_cmos_sensor(0x0A1A, 0x0C00);
	write_cmos_sensor(0x0A0C, 0x0010);
	write_cmos_sensor(0x0A1E, 0x0CCF);
	write_cmos_sensor(0x0402, 0x0110);
	write_cmos_sensor(0x0404, 0x00F4);
	write_cmos_sensor(0x0408, 0x0000);
	write_cmos_sensor(0x0410, 0x008D);
	write_cmos_sensor(0x0412, 0x011A);
	write_cmos_sensor(0x0414, 0x864C);
	write_cmos_sensor(0x021C, 0x0001);
	write_cmos_sensor(0x021E, 0x0235);
	write_cmos_sensor(0x0C00, 0x9950);
	write_cmos_sensor(0x0C06, 0x0021);
	write_cmos_sensor(0x0C10, 0x0040);
	write_cmos_sensor(0x0C12, 0x0040);
	write_cmos_sensor(0x0C14, 0x0040);
	write_cmos_sensor(0x0C16, 0x0040);
	write_cmos_sensor(0x0A02, 0x0100);
	write_cmos_sensor(0x0A04, 0x014A);
	write_cmos_sensor(0x0418, 0x0000);
	write_cmos_sensor(0x0128, 0x0028);
	write_cmos_sensor(0x012A, 0xFFFF);
	write_cmos_sensor(0x0120, 0x0046);
	write_cmos_sensor(0x0122, 0x0376);
	write_cmos_sensor(0x012C, 0x0020);
	write_cmos_sensor(0x012E, 0xFFFF);
	write_cmos_sensor(0x0124, 0x0040);
	write_cmos_sensor(0x0126, 0x0378);
	write_cmos_sensor(0x0746, 0x0050);
	write_cmos_sensor(0x0748, 0x01D5);
	write_cmos_sensor(0x074A, 0x022B);
	write_cmos_sensor(0x074C, 0x03B0);
	write_cmos_sensor(0x0756, 0x043F);
	write_cmos_sensor(0x0758, 0x3F1D);
	write_cmos_sensor(0x0B02, 0xE04D);
	write_cmos_sensor(0x0B10, 0x6821);
	write_cmos_sensor(0x0B12, 0x0120);
	write_cmos_sensor(0x0B14, 0x0001);
	write_cmos_sensor(0x2008, 0x38FD);
	write_cmos_sensor(0x326E, 0x0000);
	write_cmos_sensor(0x0900, 0x0300);
	write_cmos_sensor(0x0902, 0xC319);
	write_cmos_sensor(0x0914, 0xC109);
	write_cmos_sensor(0x0916, 0x061A);
	write_cmos_sensor(0x0918, 0x0407);
	write_cmos_sensor(0x091A, 0x0A0B);
	write_cmos_sensor(0x091C, 0x0E08);
	write_cmos_sensor(0x091E, 0x0A00);
	write_cmos_sensor(0x090C, 0x0427);
	write_cmos_sensor(0x090E, 0x0059);
	write_cmos_sensor(0x0954, 0x0089);
	write_cmos_sensor(0x0956, 0x0000);
	write_cmos_sensor(0x0958, 0xCA80);
	write_cmos_sensor(0x095A, 0x9240);
	write_cmos_sensor(0x0F08, 0x2F04);
	write_cmos_sensor(0x0F30, 0x001F);
	write_cmos_sensor(0x0F36, 0x001F);
	write_cmos_sensor(0x0F04, 0x3A00);
	write_cmos_sensor(0x0F32, 0x025A);
	write_cmos_sensor(0x0F38, 0x025A);
	write_cmos_sensor(0x0F2A, 0x4124);
	write_cmos_sensor(0x006A, 0x0100);
	write_cmos_sensor(0x004C, 0x0100);
	write_cmos_sensor(0x0044, 0x0001);
	#endif
	LOG_INF("E init done\n");
}
#if USE_BURST_MODE
static kal_uint16 addr_data_pair_preview[] = {
/*0x002E, 0x3311,
0x0032, 0x3311,
0x0026, 0x0040,
0x002C, 0x09CF,
0x005C, 0x4202,
0x0006, 0x09DE,
0x0008, 0x0ED8,
0x000C, 0x0122,
0x0A22, 0x0100,
0x0A24, 0x0000,
0x0804, 0x0000,
0x0A12, 0x0660,
0x0A14, 0x04C8,
0x0074, 0x09D8,
0x0A04, 0x016A,
0x021C, 0x0001,
0x021E, 0x0235,
0x0418, 0x0000,
0x0B02, 0xE04D,
0x0B10, 0x6C21,
0x0B12, 0x0120,
0x0B14, 0x0005,
0x2008, 0x38FD,
0x326E, 0x0000,
0x0900, 0x0300,
0x0902, 0xC319,
0x0914, 0xC105,
0x0916, 0x030C,
0x0918, 0x0304,
0x091A, 0x0708,
0x091C, 0x0B04,
0x091E, 0x0500,
0x090C, 0x0208,
0x090E, 0x001C,
0x0954, 0x0089,
0x0956, 0x0000,
0x0958, 0xCA80,
0x095A, 0x9240,
0x0F2A, 0x4924,
0x004C, 0x0100,*/
	0x002E, 0x1111,
	0x0032, 0x1111,
	0x0026, 0x0040,
	0x002C, 0x09CF,
	0x005C, 0x2101,
	0x0006, 0x09DE,
	0x0008, 0x0ED8,
	0x000C, 0x0022,
	0x0A22, 0x0000,
	0x0A24, 0x0000,
	0x0804, 0x0000,
	0x0A12, 0x0CC0,
	0x0A14, 0x0990,
	0x0074, 0x09D8,
	0x021C, 0x0001,
	0x021E, 0x0235,
	0x0A04, 0x014A,
	0x0418, 0x0000,
	0x0B02, 0xE04D,
	0x0B10, 0x6821,
	0x0B12, 0x0120,
	0x0B14, 0x0001,
	0x2008, 0x38FD,
	0x326E, 0x0000,
	0x0900, 0x0300,
	0x0902, 0xC319,
	0x0914, 0xC109,
	0x0916, 0x061A,
	0x0918, 0x0407,
	0x091A, 0x0A0B,
	0x091C, 0x0E08,
	0x091E, 0x0A00,
	0x090C, 0x0427,
	0x090E, 0x0059,
	0x0954, 0x0089,
	0x0956, 0x0000,
	0x0958, 0xCA80,
	0x095A, 0x9240,
	0x0F2A, 0x4124,
	0x004C, 0x0100,

};
#endif
static void preview_setting(void)
{
	LOG_INF("E preview\n");
	#if USE_BURST_MODE
	table_write_cmos_sensor(addr_data_pair_preview,
		sizeof(addr_data_pair_preview) / sizeof(kal_uint16));
	#else
	write_cmos_sensor(0x002E, 0x3311);
	write_cmos_sensor(0x0032, 0x3311);
	write_cmos_sensor(0x0026, 0x0040);
	write_cmos_sensor(0x002C, 0x09CF);
	write_cmos_sensor(0x005C, 0x4202);
	write_cmos_sensor(0x0006, 0x09DE);
	write_cmos_sensor(0x0008, 0x0ED8);
	write_cmos_sensor(0x000C, 0x0122);
	write_cmos_sensor(0x0A22, 0x0100);
	write_cmos_sensor(0x0A24, 0x0000);
	write_cmos_sensor(0x0804, 0x0000);
	write_cmos_sensor(0x0A12, 0x0660);
	write_cmos_sensor(0x0A14, 0x04C8);
	write_cmos_sensor(0x0074, 0x09D8);
	write_cmos_sensor(0x0A04, 0x016A);
	write_cmos_sensor(0x021C, 0x0001);
	write_cmos_sensor(0x021E, 0x0235);
	write_cmos_sensor(0x0418, 0x0000);
	write_cmos_sensor(0x0B02, 0xE04D);
	write_cmos_sensor(0x0B10, 0x6C21);
	write_cmos_sensor(0x0B12, 0x0120);
	write_cmos_sensor(0x0B14, 0x0005);
	write_cmos_sensor(0x2008, 0x38FD);
	write_cmos_sensor(0x326E, 0x0000);
	//=============================================//
	//      MIPI 4lane 360Mbps
	//=============================================//
	write_cmos_sensor(0x0900, 0x0300);
	write_cmos_sensor(0x0902, 0xC319);
	write_cmos_sensor(0x0914, 0xC105);
	write_cmos_sensor(0x0916, 0x030C);
	write_cmos_sensor(0x0918, 0x0304);
	write_cmos_sensor(0x091A, 0x0708);
	write_cmos_sensor(0x091C, 0x0B04);
	write_cmos_sensor(0x091E, 0x0500);
	write_cmos_sensor(0x090C, 0x0208);
	write_cmos_sensor(0x090E, 0x001C);
	write_cmos_sensor(0x0954, 0x0089);
	write_cmos_sensor(0x0956, 0x0000);
	write_cmos_sensor(0x0958, 0xCA80);
	write_cmos_sensor(0x095A, 0x9240);
	write_cmos_sensor(0x0F2A, 0x4924);
	write_cmos_sensor(0x004C, 0x0100);
	#endif
}

#if USE_BURST_MODE
static kal_uint16 addr_data_pair_capture_30fps[] = {
0x002E, 0x1111,
0x0032, 0x1111,
0x0026, 0x0040,
0x002C, 0x09CF,
0x005C, 0x2101,
0x0006, 0x09DE,
0x0008, 0x0ED8,
0x000C, 0x0022,
0x0A22, 0x0000,
0x0A24, 0x0000,
0x0804, 0x0000,
0x0A12, 0x0CC0,
0x0A14, 0x0990,
0x0074, 0x09D8,
0x021C, 0x0001,
0x021E, 0x0235,
0x0A04, 0x014A,
0x0418, 0x0000,
0x0B02, 0xE04D,
0x0B10, 0x6821,
0x0B12, 0x0120,
0x0B14, 0x0001,
0x2008, 0x38FD,
0x326E, 0x0000,
0x0900, 0x0300,
0x0902, 0xC319,
0x0914, 0xC109,
0x0916, 0x061A,
0x0918, 0x0407,
0x091A, 0x0A0B,
0x091C, 0x0E08,
0x091E, 0x0A00,
0x090C, 0x0427,
0x090E, 0x0059,
0x0954, 0x0089,
0x0956, 0x0000,
0x0958, 0xCA80,
0x095A, 0x9240,
0x0F2A, 0x4124,
0x004C, 0x0100,
};

static kal_uint16 addr_data_pair_capture_24fps[] = {
0x002E, 0x1111,
0x0032, 0x1111,
0x0026, 0x0040,
0x002C, 0x09CF,
0x005C, 0x2101,
0x0006, 0x13BC,
0x0008, 0x0ED8,
0x000C, 0x0022,
0x0A22, 0x0000,
0x0A24, 0x0000,
0x0804, 0x0000,
0x0A12, 0x0CC0,
0x0A14, 0x0990,
0x0074, 0x13B6,
0x021C, 0x0001,
0x021E, 0x0235,
0x0A04, 0x014A,
0x0418, 0x0000,
0x0B02, 0xE04D,
0x0B10, 0x6821,
0x0B12, 0x0120,
0x0B14, 0x0001,
0x2008, 0x38FD,
0x326E, 0x0000,
0x0900, 0x0300,
0x0902, 0xC319,
0x0914, 0xC109,
0x0916, 0x061A,
0x0918, 0x0407,
0x091A, 0x0A0B,
0x091C, 0x0E08,
0x091E, 0x0A00,
0x090C, 0x0427,
0x090E, 0x0059,
0x0954, 0x0089,
0x0956, 0x0000,
0x0958, 0xCA80,
0x095A, 0x9240,
0x0F2A, 0x4124,
0x004C, 0x0100,
};
#endif

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E capture, currefps = %d\n",currefps);
	#if USE_BURST_MODE
	if( currefps == 300 ) {
		table_write_cmos_sensor(addr_data_pair_capture_30fps,
			sizeof(addr_data_pair_capture_30fps) / sizeof(kal_uint16));
	} else {
		table_write_cmos_sensor(addr_data_pair_capture_24fps,
			sizeof(addr_data_pair_capture_24fps) / sizeof(kal_uint16));
	}
	#else
	if( currefps == 300 )
	{
		write_cmos_sensor(0x002E, 0x1111);
		write_cmos_sensor(0x0032, 0x1111);
		write_cmos_sensor(0x0026, 0x0040);
		write_cmos_sensor(0x002C, 0x09CF);
		write_cmos_sensor(0x005C, 0x2101);
		write_cmos_sensor(0x0006, 0x09DE);
		write_cmos_sensor(0x0008, 0x0ED8);
		write_cmos_sensor(0x000C, 0x0022);
		write_cmos_sensor(0x0A22, 0x0000);
		write_cmos_sensor(0x0A24, 0x0000);
		write_cmos_sensor(0x0804, 0x0000);
		write_cmos_sensor(0x0A12, 0x0CC0);
		write_cmos_sensor(0x0A14, 0x0990);
		write_cmos_sensor(0x0074, 0x09D8);
		write_cmos_sensor(0x021C, 0x0001);
		write_cmos_sensor(0x021E, 0x0235);
		write_cmos_sensor(0x0A04, 0x014A);
		write_cmos_sensor(0x0418, 0x0000);
		write_cmos_sensor(0x0B02, 0xE04D);
		write_cmos_sensor(0x0B10, 0x6821);
		write_cmos_sensor(0x0B12, 0x0120);
		write_cmos_sensor(0x0B14, 0x0001);
		write_cmos_sensor(0x2008, 0x38FD);
		write_cmos_sensor(0x326E, 0x0000);
		//=============================================//
		//      MIPI 4lane 720Mbps
		//=============================================//
		write_cmos_sensor(0x0900, 0x0300);
		write_cmos_sensor(0x0902, 0xC319);
		write_cmos_sensor(0x0914, 0xC109);
		write_cmos_sensor(0x0916, 0x061A);
		write_cmos_sensor(0x0918, 0x0407);
		write_cmos_sensor(0x091A, 0x0A0B);
		write_cmos_sensor(0x091C, 0x0E08);
		write_cmos_sensor(0x091E, 0x0A00);
		write_cmos_sensor(0x090C, 0x0427);
		write_cmos_sensor(0x090E, 0x0059);
		write_cmos_sensor(0x0954, 0x0089);
		write_cmos_sensor(0x0956, 0x0000);
		write_cmos_sensor(0x0958, 0xCA80);
		write_cmos_sensor(0x095A, 0x9240);
		write_cmos_sensor(0x0F2A, 0x4124);
		write_cmos_sensor(0x004C, 0x0100);
	} else {
		write_cmos_sensor(0x002E, 0x1111);
		write_cmos_sensor(0x0032, 0x1111);
		write_cmos_sensor(0x0026, 0x0040);
		write_cmos_sensor(0x002C, 0x09CF);
		write_cmos_sensor(0x005C, 0x2101);
		write_cmos_sensor(0x0006, 0x13BC);
		write_cmos_sensor(0x0008, 0x0ED8);
		write_cmos_sensor(0x000C, 0x0022);
		write_cmos_sensor(0x0A22, 0x0000);
		write_cmos_sensor(0x0A24, 0x0000);
		write_cmos_sensor(0x0804, 0x0000);
		write_cmos_sensor(0x0A12, 0x0CC0);
		write_cmos_sensor(0x0A14, 0x0990);
		write_cmos_sensor(0x0074, 0x13B6);
		write_cmos_sensor(0x021C, 0x0001);
		write_cmos_sensor(0x021E, 0x0235);
		write_cmos_sensor(0x0A04, 0x014A);
		write_cmos_sensor(0x0418, 0x0000);
		write_cmos_sensor(0x0B02, 0xE04D);
		write_cmos_sensor(0x0B10, 0x6821);
		write_cmos_sensor(0x0B12, 0x0120);
		write_cmos_sensor(0x0B14, 0x0001);
		write_cmos_sensor(0x2008, 0x38FD);
		write_cmos_sensor(0x326E, 0x0000);
		//=============================================//
		//      MIPI 4lane 720Mbps
		//=============================================//
		write_cmos_sensor(0x0900, 0x0300);
		write_cmos_sensor(0x0902, 0xC319);
		write_cmos_sensor(0x0914, 0xC109);
		write_cmos_sensor(0x0916, 0x061A);
		write_cmos_sensor(0x0918, 0x0407);
		write_cmos_sensor(0x091A, 0x0A0B);
		write_cmos_sensor(0x091C, 0x0E08);
		write_cmos_sensor(0x091E, 0x0A00);
		write_cmos_sensor(0x090C, 0x0427);
		write_cmos_sensor(0x090E, 0x0059);
		write_cmos_sensor(0x0954, 0x0089);
		write_cmos_sensor(0x0956, 0x0000);
		write_cmos_sensor(0x0958, 0xCA80);
		write_cmos_sensor(0x095A, 0x9240);
		write_cmos_sensor(0x0F2A, 0x4124);
		write_cmos_sensor(0x004C, 0x0100);
	}
	#endif
}

#if USE_BURST_MODE
static kal_uint16 addr_data_pair_normal_video[] = {
0x002E, 0x1111,
0x0032, 0x1111,
0x0026, 0x0170,
0x002C, 0x089F,
0x005C, 0x2101,
0x0006, 0x09DE,
0x0008, 0x0ED8,
0x000C, 0x0022,
0x0A22, 0x0000,
0x0A24, 0x0000,
0x0804, 0x0000,
0x0A12, 0x0CC0,
0x0A14, 0x0730,
0x0074, 0x09D8,
0x021C, 0x0001,
0x0A04, 0x014A,
0x0418, 0x0000,
0x0B02, 0xE04D,
0x0B10, 0x6821,
0x0B12, 0x0120,
0x0B14, 0x0001,
0x2008, 0x38FD,
0x326E, 0x0000,
0x0900, 0x0300,
0x0902, 0xC319,
0x0914, 0xC109,
0x0916, 0x061A,
0x0918, 0x0407,
0x091A, 0x0A0B,
0x091C, 0x0E08,
0x091E, 0x0A00,
0x090C, 0x0427,
0x090E, 0x0059,
0x0954, 0x0089,
0x0956, 0x0000,
0x0958, 0xCA80,
0x095A, 0x9240,
0x0F2A, 0x4124,
0x004C, 0x0100,
};
#endif

static void normal_video_setting(void)
{
	LOG_INF("E hs_video_setting\n");
	#if USE_BURST_MODE
	table_write_cmos_sensor(addr_data_pair_normal_video,
		sizeof(addr_data_pair_normal_video) / sizeof(kal_uint16));
	#else
	write_cmos_sensor(0x002E, 0x1111);
	write_cmos_sensor(0x0032, 0x1111);
	write_cmos_sensor(0x0026, 0x0170);
	write_cmos_sensor(0x002C, 0x089F);
	write_cmos_sensor(0x005C, 0x2101);
	write_cmos_sensor(0x0006, 0x09DE);
	write_cmos_sensor(0x0008, 0x0ED8);
	write_cmos_sensor(0x000C, 0x0022);
	write_cmos_sensor(0x0A22, 0x0000);
	write_cmos_sensor(0x0A24, 0x0000);
	write_cmos_sensor(0x0804, 0x0000);
	write_cmos_sensor(0x0A12, 0x0CC0);
	write_cmos_sensor(0x0A14, 0x0730);
	write_cmos_sensor(0x0074, 0x09D8);
	write_cmos_sensor(0x021C, 0x0001);
	write_cmos_sensor(0x0A04, 0x014A);
	write_cmos_sensor(0x0418, 0x0000);
	write_cmos_sensor(0x0B02, 0xE04D);
	write_cmos_sensor(0x0B10, 0x6821);
	write_cmos_sensor(0x0B12, 0x0120);
	write_cmos_sensor(0x0B14, 0x0001);
	write_cmos_sensor(0x2008, 0x38FD);
	write_cmos_sensor(0x326E, 0x0000);
	//=============================================//
	//      MIPI 4lane 360Mbps
	//=============================================//
	write_cmos_sensor(0x0900, 0x0300);
	write_cmos_sensor(0x0902, 0xC319);
	write_cmos_sensor(0x0914, 0xC109);
	write_cmos_sensor(0x0916, 0x061A);
	write_cmos_sensor(0x0918, 0x0407);
	write_cmos_sensor(0x091A, 0x0A0B);
	write_cmos_sensor(0x091C, 0x0E08);
	write_cmos_sensor(0x091E, 0x0A00);
	write_cmos_sensor(0x090C, 0x0427);
	write_cmos_sensor(0x090E, 0x0059);
	write_cmos_sensor(0x0954, 0x0089);
	write_cmos_sensor(0x0956, 0x0000);
	write_cmos_sensor(0x0958, 0xCA80);
	write_cmos_sensor(0x095A, 0x9240);
	write_cmos_sensor(0x0F2A, 0x4124);
	write_cmos_sensor(0x004C, 0x0100);
	#endif
};

#if USE_BURST_MODE
static kal_uint16 addr_data_pair_hs_video[] = {
0x002E, 0x3311,
0x0032, 0x3311,
0x0026, 0x0040,
0x002C, 0x09CF,
0x005C, 0x4202,
0x0006, 0x09DE,
0x0008, 0x0ED8,
0x000C, 0x0122,
0x0A22, 0x0100,
0x0A24, 0x0000,
0x0804, 0x0000,
0x0A12, 0x0660,
0x0A14, 0x04C8,
0x0074, 0x09D8,
0x0A04, 0x016A,
0x021C, 0x0001,
0x021E, 0x0235,
0x0418, 0x0000,
0x0B02, 0xE04D,
0x0B10, 0x6C21,
0x0B12, 0x0120,
0x0B14, 0x0005,
0x2008, 0x38FD,
0x326E, 0x0000,
0x0900, 0x0300,
0x0902, 0xC319,
0x0914, 0xC105,
0x0916, 0x030C,
0x0918, 0x0304,
0x091A, 0x0708,
0x091C, 0x0B04,
0x091E, 0x0500,
0x090C, 0x0208,
0x090E, 0x001C,
0x0954, 0x0089,
0x0956, 0x0000,
0x0958, 0xCA80,
0x095A, 0x9240,
0x0F2A, 0x4924,
0x004C, 0x0100,
};
#endif

static void hs_video_setting(void)
{
	LOG_INF("E hs_video_setting\n");
	#if USE_BURST_MODE
	table_write_cmos_sensor(addr_data_pair_hs_video,
		sizeof(addr_data_pair_hs_video) / sizeof(kal_uint16));
	#else
	write_cmos_sensor(0x002E, 0x3311);
	write_cmos_sensor(0x0032, 0x3311);
	write_cmos_sensor(0x0026, 0x0040);
	write_cmos_sensor(0x002C, 0x09CF);
	write_cmos_sensor(0x005C, 0x4202);
	write_cmos_sensor(0x0006, 0x09DE);
	write_cmos_sensor(0x0008, 0x0ED8);
	write_cmos_sensor(0x000C, 0x0122);
	write_cmos_sensor(0x0A22, 0x0100);
	write_cmos_sensor(0x0A24, 0x0000);
	write_cmos_sensor(0x0804, 0x0000);
	write_cmos_sensor(0x0A12, 0x0660);
	write_cmos_sensor(0x0A14, 0x04C8);
	write_cmos_sensor(0x0074, 0x09D8);
	write_cmos_sensor(0x0A04, 0x016A);
	write_cmos_sensor(0x021C, 0x0001);
	write_cmos_sensor(0x021E, 0x0235);
	write_cmos_sensor(0x0418, 0x0000);
	write_cmos_sensor(0x0B02, 0xE04D);
	write_cmos_sensor(0x0B10, 0x6C21);
	write_cmos_sensor(0x0B12, 0x0120);
	write_cmos_sensor(0x0B14, 0x0005);
	write_cmos_sensor(0x2008, 0x38FD);
	write_cmos_sensor(0x326E, 0x0000);
	//=============================================//
	//      MIPI 4lane 360Mbps
	//=============================================//
	write_cmos_sensor(0x0900, 0x0300);
	write_cmos_sensor(0x0902, 0xC319);
	write_cmos_sensor(0x0914, 0xC105);
	write_cmos_sensor(0x0916, 0x030C);
	write_cmos_sensor(0x0918, 0x0304);
	write_cmos_sensor(0x091A, 0x0708);
	write_cmos_sensor(0x091C, 0x0B04);
	write_cmos_sensor(0x091E, 0x0500);
	write_cmos_sensor(0x090C, 0x0208);
	write_cmos_sensor(0x090E, 0x001C);
	write_cmos_sensor(0x0954, 0x0089);
	write_cmos_sensor(0x0956, 0x0000);
	write_cmos_sensor(0x0958, 0xCA80);
	write_cmos_sensor(0x095A, 0x9240);
	write_cmos_sensor(0x0F2A, 0x4924);
	write_cmos_sensor(0x004C, 0x0100);
	#endif
};

#if USE_BURST_MODE
static kal_uint16 addr_data_pair_slim_video[] = {
0x002E, 0x3311,
0x0032, 0x3311,
0x0026, 0x0238,
0x002C, 0x07D7,
0x005C, 0x4202,
0x0006, 0x034A,
0x0008, 0x0ED8,
0x000C, 0x0122,
0x0A22, 0x0100,
0x0A24, 0x0000,
0x0804, 0x00B0,
0x0A12, 0x0500,
0x0A14, 0x02D0,
0x0074, 0x0344,
0x021C, 0x0001,
0x021E, 0x0235,
0x0A04, 0x016A,
0x0418, 0x0410,
0x0B02, 0xE04D,
0x0B10, 0x6C21,
0x0B12, 0x0120,
0x0B14, 0x0005,
0x2008, 0x38FD,
0x326E, 0x0000,
0x0900, 0x0300,
0x0902, 0xC319,
0x0914, 0xC105,
0x0916, 0x030C,
0x0918, 0x0304,
0x091A, 0x0708,
0x091C, 0x0B04,
0x091E, 0x0500,
0x090C, 0x0208,
0x090E, 0x008A,
0x0954, 0x0089,
0x0956, 0x0000,
0x0958, 0xCA80,
0x095A, 0x9240,
0x0F2A, 0x4924,
0x004C, 0x0100,
};
#endif

static void slim_video_setting(void)
{
	#if USE_BURST_MODE
	table_write_cmos_sensor(addr_data_pair_slim_video,
		sizeof(addr_data_pair_slim_video) / sizeof(kal_uint16));
	#else
	write_cmos_sensor(0x002E, 0x3311);
	write_cmos_sensor(0x0032, 0x3311);
	write_cmos_sensor(0x0026, 0x0238);
	write_cmos_sensor(0x002C, 0x07D7);
	write_cmos_sensor(0x005C, 0x4202);
	write_cmos_sensor(0x0006, 0x034A);
	write_cmos_sensor(0x0008, 0x0ED8);
	write_cmos_sensor(0x000C, 0x0122);
	write_cmos_sensor(0x0A22, 0x0100);
	write_cmos_sensor(0x0A24, 0x0000);
	write_cmos_sensor(0x0804, 0x00B0);
	write_cmos_sensor(0x0A12, 0x0500);
	write_cmos_sensor(0x0A14, 0x02D0);
	write_cmos_sensor(0x0074, 0x0344);
	write_cmos_sensor(0x021C, 0x0001);
	write_cmos_sensor(0x021E, 0x0235);
	write_cmos_sensor(0x0A04, 0x016A);
	write_cmos_sensor(0x0418, 0x0410);
	write_cmos_sensor(0x0B02, 0xE04D);
	write_cmos_sensor(0x0B10, 0x6C21);
	write_cmos_sensor(0x0B12, 0x0120);
	write_cmos_sensor(0x0B14, 0x0005);
	write_cmos_sensor(0x2008, 0x38FD);
	write_cmos_sensor(0x326E, 0x0000);
	//=============================================//
	//      MIPI 4lane 360Mbps
	//=============================================//
	write_cmos_sensor(0x0900, 0x0300);
	write_cmos_sensor(0x0902, 0xC319);
	write_cmos_sensor(0x0914, 0xC105);
	write_cmos_sensor(0x0916, 0x030C);
	write_cmos_sensor(0x0918, 0x0304);
	write_cmos_sensor(0x091A, 0x0708);
	write_cmos_sensor(0x091C, 0x0B04);
	write_cmos_sensor(0x091E, 0x0500);
	write_cmos_sensor(0x090C, 0x0208);
	write_cmos_sensor(0x090E, 0x008A);
	write_cmos_sensor(0x0954, 0x0089);
	write_cmos_sensor(0x0956, 0x0000);
	write_cmos_sensor(0x0958, 0xCA80);
	write_cmos_sensor(0x095A, 0x9240);
	write_cmos_sensor(0x0F2A, 0x4924);
	write_cmos_sensor(0x004C, 0x0100);
	#endif
};

#if USE_BURST_MODE
static kal_uint16 addr_data_pair_custom1[] = {
0x002E, 0x1111,
0x0032, 0x1111,
0x0026, 0x01A8,
0x002C, 0x0867,
0x005C, 0x2101,
0x0006, 0x0C54,
0x0008, 0x0ED8,
0x000C, 0x0022,
0x0A22, 0x0000,
0x0A24, 0x0000,
0x0804, 0x01E0,
0x0A12, 0x0900,
0x0A14, 0x06C0,
0x0074, 0x0C4E,
0x021C, 0x0001,
0x0A04, 0x014A,
0x0418, 0x0000,
0x0B02, 0xE04D,
0x0B10, 0x6821,
0x0B12, 0x0120,
0x0B14, 0x0001,
0x2008, 0x38FD,
0x326E, 0x0000,
0x0900, 0x0300,
0x0902, 0xC319,
0x0914, 0xC109,
0x0916, 0x061A,
0x0918, 0x0407,
0x091A, 0x0A0B,
0x091C, 0x0E08,
0x091E, 0x0A00,
0x090C, 0x0427,
0x090E, 0x0059,
0x0954, 0x0089,
0x0956, 0x0000,
0x0958, 0xCA80,
0x095A, 0x9240,
0x0F2A, 0x4124,
0x004C, 0x0100,
};
#endif

static void custom1_setting(void)
{
	#if USE_BURST_MODE
	table_write_cmos_sensor(addr_data_pair_custom1,
		sizeof(addr_data_pair_custom1) / sizeof(kal_uint16));
	#else
	write_cmos_sensor(0x002E, 0x1111);
	write_cmos_sensor(0x0032, 0x1111);
	write_cmos_sensor(0x0026, 0x01A8);
	write_cmos_sensor(0x002C, 0x0867);
	write_cmos_sensor(0x005C, 0x2101);
	write_cmos_sensor(0x0006, 0x0C54);
	write_cmos_sensor(0x0008, 0x0ED8);
	write_cmos_sensor(0x000C, 0x0022);
	write_cmos_sensor(0x0A22, 0x0000);
	write_cmos_sensor(0x0A24, 0x0000);
	write_cmos_sensor(0x0804, 0x01E0);
	write_cmos_sensor(0x0A12, 0x0900);
	write_cmos_sensor(0x0A14, 0x06C0);
	write_cmos_sensor(0x0074, 0x0C4E);
	write_cmos_sensor(0x021C, 0x0001);
	write_cmos_sensor(0x0A04, 0x014A);
	write_cmos_sensor(0x0418, 0x0000);
	write_cmos_sensor(0x0B02, 0xE04D);
	write_cmos_sensor(0x0B10, 0x6821);
	write_cmos_sensor(0x0B12, 0x0120);
	write_cmos_sensor(0x0B14, 0x0001);
	write_cmos_sensor(0x2008, 0x38FD);
	write_cmos_sensor(0x326E, 0x0000);
	//=============================================//
	//      MIPI 4lane 360Mbps
	//=============================================//
	write_cmos_sensor(0x0900, 0x0300);
	write_cmos_sensor(0x0902, 0xC319);
	write_cmos_sensor(0x0914, 0xC109);
	write_cmos_sensor(0x0916, 0x061A);
	write_cmos_sensor(0x0918, 0x0407);
	write_cmos_sensor(0x091A, 0x0A0B);
	write_cmos_sensor(0x091C, 0x0E08);
	write_cmos_sensor(0x091E, 0x0A00);
	write_cmos_sensor(0x090C, 0x0427);
	write_cmos_sensor(0x090E, 0x0059);
	write_cmos_sensor(0x0954, 0x0089);
	write_cmos_sensor(0x0956, 0x0000);
	write_cmos_sensor(0x0958, 0xCA80);
	write_cmos_sensor(0x095A, 0x9240);
	write_cmos_sensor(0x0F2A, 0x4124);
	write_cmos_sensor(0x004C, 0x0100);
	#endif
};

#if USE_BURST_MODE
static kal_uint16 addr_data_pair_custom2[] = {
0x002E, 0x1111,
0x0032, 0x1111,
0x0026, 0x02EC,
0x002C, 0x0723,
0x005C, 0x2101,
0x0006, 0x04EF,
0x0008, 0x0ED8,
0x000C, 0x0022,
0x0A22, 0x0000,
0x0A24, 0x0000,
0x0804, 0x02A0,
0x0A12, 0x0780,
0x0A14, 0x0438,
0x0074, 0x04E9,
0x0A04, 0x014A,
0x0418, 0x04C4,
0x0B02, 0xE04D,
0x0B10, 0x6821,
0x0B12, 0x0120,
0x0B14, 0x0001,
0x2008, 0x38FD,
0x326E, 0x0000,
0x0900, 0x0300,
0x0902, 0xC319,
0x0914, 0xC109,
0x0916, 0x061A,
0x0918, 0x0407,
0x091A, 0x0A0B,
0x091C, 0x0E08,
0x091E, 0x0A00,
0x090C, 0x0427,
0x090E, 0x020D,
0x0954, 0x0089,
0x0956, 0x0000,
0x0958, 0xCA80,
0x095A, 0x9240,
0x0F2A, 0x4124,
0x004C, 0x0100,
};
#endif

static void custom2_setting(void)
{
	#if USE_BURST_MODE
	table_write_cmos_sensor(addr_data_pair_custom2,
		sizeof(addr_data_pair_custom2) / sizeof(kal_uint16));
	#else
	write_cmos_sensor(0x002E, 0x1111);
	write_cmos_sensor(0x0032, 0x1111);
	write_cmos_sensor(0x0026, 0x02EC);
	write_cmos_sensor(0x002C, 0x0723);
	write_cmos_sensor(0x005C, 0x2101);
	write_cmos_sensor(0x0006, 0x04EF);
	write_cmos_sensor(0x0008, 0x0ED8);
	write_cmos_sensor(0x000C, 0x0022);
	write_cmos_sensor(0x0A22, 0x0000);
	write_cmos_sensor(0x0A24, 0x0000);
	write_cmos_sensor(0x0804, 0x02A0);
	write_cmos_sensor(0x0A12, 0x0780);
	write_cmos_sensor(0x0A14, 0x0438);
	write_cmos_sensor(0x0074, 0x04E9);
	write_cmos_sensor(0x0A04, 0x014A);
	write_cmos_sensor(0x0418, 0x04C4);
	write_cmos_sensor(0x0B02, 0xE04D);
	write_cmos_sensor(0x0B10, 0x6821);
	write_cmos_sensor(0x0B12, 0x0120);
	write_cmos_sensor(0x0B14, 0x0001);
	write_cmos_sensor(0x2008, 0x38FD);
	write_cmos_sensor(0x326E, 0x0000);
	//=============================================//
	//      MIPI 4lane 720Mbps
	//=============================================//
	write_cmos_sensor(0x0900, 0x0300);
	write_cmos_sensor(0x0902, 0xC319);
	write_cmos_sensor(0x0914, 0xC109);
	write_cmos_sensor(0x0916, 0x061A);
	write_cmos_sensor(0x0918, 0x0407);
	write_cmos_sensor(0x091A, 0x0A0B);
	write_cmos_sensor(0x091C, 0x0E08);
	write_cmos_sensor(0x091E, 0x0A00);
	write_cmos_sensor(0x090C, 0x0427);
	write_cmos_sensor(0x090E, 0x020D);
	write_cmos_sensor(0x0954, 0x0089);
	write_cmos_sensor(0x0956, 0x0000);
	write_cmos_sensor(0x0958, 0xCA80);
	write_cmos_sensor(0x095A, 0x9240);
	write_cmos_sensor(0x0F2A, 0x4124);
	write_cmos_sensor(0x004C, 0x0100);
	#endif
};

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static void read_EepromQSC(void)
{
    int i = 0;
    float externStereo[6] = {-0.0107, -0.0020, 0.0011, 0.0009, 11.3750, -0.6378};

    gImgEepromInfo.camNormdata[2][0] = Eeprom_1ByteDataRead(0x00, 0xA0);
    gImgEepromInfo.camNormdata[2][1] = Eeprom_1ByteDataRead(0x01, 0xA0);

    imgsensor_info.module_id = Eeprom_1ByteDataRead(0x00, 0xA0);
    Oplusimgsensor_Registdeviceinfo(gImgEepromInfo.pCamModuleInfo[2].name,
                                    gImgEepromInfo.pCamModuleInfo[2].version,
                                    imgsensor_info.module_id);
    for(i = 2; i < 8; i++) {
        gImgEepromInfo.camNormdata[2][i] = Eeprom_1ByteDataRead(0x04+i, 0xA0);
    }
    for (i = 0; i < OPLUS_CAMERASN_LENS; i ++) {
        gImgEepromInfo.camNormdata[2][8+i] = Eeprom_1ByteDataRead(0xB0+i, 0xA0);
    }

    gImgEepromInfo.camNormdata[2][39] = 2;
    gImgEepromInfo.i4CurSensorIdx = 2;
    gImgEepromInfo.i4CurSensorId = imgsensor_info.sensor_id;
    for (i = 0; i < CALI_DATA_SLAVE_LENGTH; i++) {
        gImgEepromInfo.stereoMWdata[CALI_DATA_MASTER_LENGTH + i] =
            Eeprom_1ByteDataRead(HI846Q2R_STEREO_START_ADDR + i, 0xA0);
    }
    memcpy(&gImgEepromInfo.stereoMWdata[DUALCAM_CALI_DATA_LENGTH_TOTAL - 28], externStereo, 24);
}
#endif

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    LOG_INF("[get_imgsensor_id] ");
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id =return_sensor_id();
            if (*sensor_id == imgsensor_info.sensor_id) {
                #ifdef OPLUS_FEATURE_CAMERA_COMMON
                read_EepromQSC();
                read_eeprom_SN();
                #endif
                LOG_INF("i2c write id  : 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
                return ERROR_NONE;
            }
            LOG_INF("get_imgsensor_id Read sensor id fail, i2c write id: 0x%x,sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
            retry--;
        } while(retry > 0);
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
	kal_uint16 sensor_id = 0;

	LOG_INF("[open]: PLATFORM:MT6735,MIPI 4LANE\n");
	LOG_INF("preview 1632*1224@30fps,360Mbps/lane; capture 3264*2448@30fps,720Mbps/lane\n");
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("open:Read sensor id fail open i2c write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id) {
			break;
		}
		retry = 2;
	}
	#ifndef OPLUS_FEATURE_CAMERA_COMMON
	if (imgsensor_info.sensor_id != sensor_id) {
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	#else
	if (imgsensor_info.sensor_id != sensor_id) {
		return ERROR_SENSORID_READ_FAIL;
	}
	#endif
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
}	/*	open  */
static kal_uint32 close(void)
{
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
	LOG_INF("E");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();

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
	LOG_INF("E");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) // 30fps
	{
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}

	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("Caputre fps:%d\n",imgsensor.current_fps);
	capture_setting(imgsensor.current_fps);

	return ERROR_NONE;

}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("normal_video");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting();
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("hs_video\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("slim_video\n");
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
	return ERROR_NONE;
}    /*    slim_video     */

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
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
	LOG_INF("custom1 fps:%d\n",imgsensor.current_fps);
	custom1_setting();

	return ERROR_NONE;

}	/* custom1() */

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
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
	LOG_INF("custom2 fps:%d\n",imgsensor.current_fps);
	custom2_setting();

	return ERROR_NONE;

}	/* custom2() */


static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width     = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;

	sensor_resolution->SensorCustom2Width     = imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height     = imgsensor_info.custom2.grabwindow_height;

	return ERROR_NONE;
}    /*    get_resolution    */


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
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
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
		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}    /*    get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			LOG_INF("preview\n");
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CAMERA_ZSD:
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
	// SetVideoMode Function should fix framerate
	if (framerate == 0) {
		// Dynamic frame rate
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
	set_max_framerate(imgsensor.current_fps,1);
	set_dummy();
	return ERROR_NONE;
}


static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) {
		imgsensor.autoflicker_en = KAL_TRUE;
	} else { //Cancel Auto flick
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
			if (imgsensor.frame_length>imgsensor.shutter) {
				set_dummy();
			}
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if (framerate == 0) {
				return ERROR_NONE;
			}
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length>imgsensor.shutter) {
				set_dummy();
			}

			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
				frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			} else {
				if (imgsensor.current_fps != imgsensor_info.cap.max_framerate) {
					LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
				}
				frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			}
			if (imgsensor.frame_length>imgsensor.shutter) {
				set_dummy();
			}

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length>imgsensor.shutter) {
				set_dummy();
			}
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length>imgsensor.shutter) {
				set_dummy();
			}
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength): 0;
			imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length>imgsensor.shutter) {
				set_dummy();
			}
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength): 0;
			imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length>imgsensor.shutter) {
				set_dummy();
			}
			break;
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length>imgsensor.shutter) {
				set_dummy();
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
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d", enable);

	if (enable) {
		LOG_INF("enter color bar");
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0a04, 0x0141);
		write_cmos_sensor(0x020a, 0x0100);
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0a04, 0x0142);
		write_cmos_sensor(0x020a, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
	unsigned long long *feature_data=(unsigned long long *) feature_para;
	//unsigned long long *feature_return_para=(unsigned long long *) feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
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
        case SENSOR_FEATURE_GET_MODULE_SN:
        LOG_INF("gc5035 GET_MODULE_SN:%d %d\n", *feature_para_len, *feature_data_32);
        if (*feature_data_32 < CAMERA_MODULE_SN_LENGTH/4)
            *(feature_data_32 + 1) = (gHi846_SN[4*(*feature_data_32) + 3] << 24)
                        | (gHi846_SN[4*(*feature_data_32) + 2] << 16)
                        | (gHi846_SN[4*(*feature_data_32) + 1] << 8)
                        | (gHi846_SN[4*(*feature_data_32)] & 0xFF);
        break;
        case SENSOR_FEATURE_SET_SENSOR_OTP:
        {
            kal_int32 ret = IMGSENSOR_RETURN_SUCCESS;
            LOG_INF("SENSOR_FEATURE_SET_SENSOR_OTP length :%d\n", (UINT32)*feature_para_len);
            ret = write_Module_data((ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(feature_para));

            #ifdef OPLUS_FEATURE_CAMERA_COMMON
            if (ret == ERROR_NONE)
            return ERROR_NONE;
            else
            return ERROR_MSDK_IS_ACTIVATED;
            #endif
        }
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
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
			case MSDK_SCENARIO_ID_CUSTOM2:
				rate = imgsensor_info.custom2.mipi_pixel_rate;
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
			set_test_pattern_mode((BOOL)*feature_data);
			break;

		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_FRAMERATE:
			LOG_INF("current fps :%d\n", (UINT32)*feature_data_32);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.current_fps = *feature_data_32;
			spin_unlock(&imgsensor_drv_lock);
			break;
			//case SENSOR_FEATURE_SET_HDR:
			//LOG_INF("ihdr enable :%d\n", *feature_data_16);
			//spin_lock(&imgsensor_drv_lock);
			//imgsensor.ihdr_en = *feature_data_16;
			//spin_unlock(&imgsensor_drv_lock);
			//break;
		case SENSOR_FEATURE_GET_CROP_INFO:
			//LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data_32);
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
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[5],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CUSTOM2:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[6],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
			break;
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
			//LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			//ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			break;
		case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
			set_shutter_frame_length((UINT16) (*feature_data),
						(UINT16) (*(feature_data + 1)));
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

UINT32 HI846Q2R_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	HI846Q2R_MIPI_RAW_SensorInit	*/
