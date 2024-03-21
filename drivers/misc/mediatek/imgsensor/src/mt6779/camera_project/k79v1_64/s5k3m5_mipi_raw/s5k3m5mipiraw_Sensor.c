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
#include "s5k3m5mipiraw_Sensor.h"
#include "imgsensor_common.h"
#include "imgsensor_eeprom_v1.h"
#include "imgsensor_hwcfg_custom.h"
#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define DEVICE_VERSION_S5K3M5     "s5k3m5"
static kal_uint8 deviceInfo_register_value = 0x00;
#endif
static kal_uint32 streaming_control(kal_bool enable);
#define MODULE_ID_OFFSET 0x0000
#define PFX "S5K3M5_camera_sensor"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length, u16 timing);

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static bool bNeedSetNormalMode = KAL_FALSE;
static kal_uint8 EepData[DUALCAM_CALI_EEPROM_DATA_LENGTH];
#define SLAVEADDR 0xA0
#endif
static DEFINE_SPINLOCK(imgsensor_drv_lock);
static int badOtpFlag = 1;

static struct imgsensor_info_struct imgsensor_info = {
		.sensor_id = S5K3M5_SENSOR_ID,
		.module_id = 0x04,	//0x01 Sunny,0x05 QTEK
		.checksum_value = 0xffb1ec31,
		.pre = {
			.pclk = 482000000,
			.linelength = 4848,
			.framelength = 3312,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 4000,
			.grabwindow_height = 3000,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 576000000,
			.max_framerate = 300,
		},
		.cap = {
			.pclk = 482000000,
			.linelength = 4848,
			.framelength = 3312,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 4000,
			.grabwindow_height = 3000,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 576000000,
			.max_framerate = 300,
		},
		.normal_video = {
			.pclk = 482000000,
			.linelength = 4848,
			.framelength = 3312,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 4000,
			.grabwindow_height = 3000,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 576000000,
			.max_framerate = 300,
		},
		.hs_video = {
			.pclk = 482000000,
			.linelength = 4848,
			.framelength = 3312,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 4000,
			.grabwindow_height = 3000,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 576000000,
			.max_framerate = 300,
		},
		.slim_video = {
			.pclk = 482000000,
			.linelength = 4848,
			.framelength = 3312,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 4208,
			.grabwindow_height = 3120,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 576000000,
			.max_framerate = 300,
		},

		.margin = 2,
		.min_shutter = 3,
		.max_frame_length = 0xffff-5,
		.ae_shut_delay_frame = 0,
		.ae_sensor_gain_delay_frame = 0,
		.ae_ispGain_delay_frame = 2,
		.ihdr_support = 0,   /*1, support; 0,not support*/
		.ihdr_le_firstline = 0,  /*1,le first; 0, se first*/
		.sensor_mode_num = 5,    /*support sensor mode num*/

		.cap_delay_frame = 2,  /*3 guanjd modify for cts*/
		.pre_delay_frame = 2,  /*3 guanjd modify for cts*/
		.video_delay_frame = 3,
		.hs_video_delay_frame = 3,
		.slim_video_delay_frame = 3,

		.isp_driving_current = ISP_DRIVING_2MA,
		.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
		.mipi_sensor_type = MIPI_OPHY_NCSI2,  /*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
		.mipi_settle_delay_mode = 1,  /*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDEBAY_MANNUAL*/
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,//3p9sp remosaic awb  by wuxiao 20180822
		.mclk = 24,
		.mipi_lane_num = SENSOR_MIPI_4_LANE,
		.i2c_addr_table = {0x5a, 0x20, 0xff},
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
	.test_pattern = 0,		/*test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output*/
	.enable_secure = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/*current scenario id*/
	.ihdr_mode = 0, /*sensor need support LE, SE with HDR feature*/
	.i2c_write_id = 0x20,
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	.current_ae_effective_frame = 2,
	#endif
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
	{4208, 3120,  104,   60, 4000, 3000, 4000, 3000, 0, 0, 4000, 3000, 0, 0, 4000, 3000}, /*Preview*/
	{4208, 3120,  104,   60, 4000, 3000, 4000, 3000, 0, 0, 4000, 3000, 0, 0, 4000, 3000}, // capture remosic
	{4208, 3120,  104,   60, 4000, 3000, 4000, 3000, 0, 0, 4000, 3000, 0, 0, 4000, 3000}, /*video*/
	{4208, 3120,  104,   60, 4000, 3000, 4000, 3000, 0, 0, 4000, 3000, 0, 0, 4000, 3000}, /*hs_video,don't use*/
	{4208, 3120,    0,    0, 4208, 3120, 4208, 3120, 0, 0, 4208, 3120, 0, 0, 4208, 3120}, /* slim video*/
}; /*cpy from preview*/

 /*VC1 for HDR(DT=0X35), VC2 for PDAF(DT=0X36), unit : 10bit */
static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[2] = {
	/* Preview mode setting */
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0FA0, 0x0BB8, 0x00, 0x00, 0x00, 0x00,
	 0x01, 0x30, 0x026C, 0x02E0, 0x00, 0x00, 0x0000, 0x0000},
	/* FullSize mode setting */
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1F40, 0x1770, 0x00, 0x00, 0x00, 0x00,
	 0x01, 0x30, 0x028A, 0x0300, 0x00, 0x00, 0x0000, 0x0000}
};

/* If mirror flip */
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
	.i4OffsetX =  24,
	.i4OffsetY =  24,
	.i4PitchX  =  32,
	.i4PitchY  =  32,
	.i4PairNum  = 16,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 8,
	.i4PosL = {{26,25},{34,25},{42,25},{50,25},{30,37},{38,37},{46,37},{54,37},
		{26,45},{34,45},{42,45},{50,45},{30,49},{38,49},{46,49},{54,49}},
	.i4PosR = {{26,29},{34,29},{42,29},{50,29},{30,33},{38,33},{46,33},{54,33},
		{26,41},{34,41},{42,41},{50,41},{30,53},{38,53},{46,53},{54,53}},
	.i4BlockNumX = 124,
	.i4BlockNumY = 92,
	.iMirrorFlip = 0,
	.i4Crop = { {104,60}, {104,60}, {104,60}, {104,60}, {0,0}},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_fullsize =
{
	.i4OffsetX =  24,
	.i4OffsetY =  24,
	.i4PitchX  =  32,
	.i4PitchY  =  32,
	.i4PairNum  = 16,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 8,
	.i4PosL = {{26,25},{34,25},{42,25},{50,25},{30,37},{38,37},{46,37},{54,37},
		{26,45},{34,45},{42,45},{50,45},{30,49},{38,49},{46,49},{54,49}},
	.i4PosR = {{26,29},{34,29},{42,29},{50,29},{30,33},{38,33},{46,33},{54,33},
		{26,41},{34,41},{42,41},{50,41},{30,53},{38,53},{46,53},{54,53}},
	.i4BlockNumX = 130,
	.i4BlockNumY = 96,
	.iMirrorFlip = 0,
	.i4Crop = { {104,60}, {104,60}, {104,60}, {104,60}, {0,0}},
};
/*no mirror flip*/
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
/****hope add end****/


static kal_uint16 read_module_id(void)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(MODULE_ID_OFFSET >> 8) , (char)(MODULE_ID_OFFSET & 0xFF) };
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,0xA0/*EEPROM_READ_ID*/);
	return get_byte;

}

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

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static void check_badEeprom(void)
{
	kal_uint16 idx = 0;
	kal_uint8 get_byte[8]= {0};
	for (idx = 0; idx < 8; idx++) {
		char pusendcmd[2] = {0x24 , (char)((0x94 + idx) & 0xFF) };
		iReadRegI2C(pusendcmd , 2, (u8*)&get_byte[idx],1, 0xA0);
		LOG_INF("gBadOtp[%d]: 0x%x\n", idx, get_byte[idx]);
		if (get_byte[idx] != 0xFF) {
			badOtpFlag = 0;
			return ;
		}
	}
}
#endif
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

static void write_shutter(kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	kal_uint64 CintR = 0;
	kal_uint64 Time_Farme = 0;
	#endif
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
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	// 1H=4848/4820000000 =10.058 us
	// 16s=16000000/10.058 =1590773
	// 32s=16000000/10.058 =3181547
	//1s=1000000/10.058=99423
	if (shutter >= 0xFFF0) {  // need to modify line_length & PCLK
		bNeedSetNormalMode = KAL_TRUE;

		if (shutter >= 3181547) {  //>32s
			shutter = 3181547;
		}
		//longshutter 0x0202
		//1 S=	0611
		//2 S=	0C22
		//3 S=	1234
		//4 S=	1845
		//5 S=	1E57
		//6 S=	2468
		//7 S=	2A7A
		//8 S=	308B
		//9 S=	369D
		//10S=	3CAE
		//11S=	42C0
		//12S=	48D1
		//13S=	4EE3
		//14S=	54F4
		//15S=	5B06
		//16S=	6117
		//17S=	6729
		//18S=	6D3A
		//19S=	734C
		//20S=	795D
		//21S=	7F6E
		//22S=	8580
		//23S=	8B91
		//24S=	91A3
		//25S=	97B4
		//26S=	9DC6
		//27S=	A3D7
		//28S=	A9E9
		//29S=	AFFA
		//30S=	B60C
		//31S=	BC1D
		//32S=	C22F
		//CintR = (482000000*shutter*0.000010058)/(4848*64);
		CintR = (4848 * (unsigned long long)shutter) / 310272;
		Time_Farme = CintR + 0x0002;  // 1st framelength
		LOG_INF("CintR =%d \n", CintR);

		write_cmos_sensor_16_16(0x0340, Time_Farme & 0xFFFF);  // Framelength
		write_cmos_sensor_16_16(0x0202, CintR & 0xFFFF);  //shutter
		write_cmos_sensor_16_16(0x0702, 0x0600);
		write_cmos_sensor_16_16(0x0704, 0x0600);

		imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		imgsensor.current_ae_effective_frame = 2;
	} else {
		if (bNeedSetNormalMode) {
			LOG_INF("exit long shutter\n");

			write_cmos_sensor_16_16(0x0702, 0x0000);
			write_cmos_sensor_16_16(0x0704, 0x0000);

			bNeedSetNormalMode = KAL_FALSE;
		}

		write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
		write_cmos_sensor_16_16(0x0202, imgsensor.shutter);

		imgsensor.current_ae_effective_frame = 2;
	}
	#else
	write_cmos_sensor_16_16(0x0202, shutter);
	#endif
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
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */

/*	write_shutter  */
static void set_shutter_frame_length(kal_uint32 shutter, kal_uint32 frame_length,
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
	if (shutter >= 0xFFF0) {  // need to modify line_length & PCLK
		bNeedSetNormalMode = KAL_TRUE;

		if (shutter >= 1590773) {  //>16s
			shutter = 1590773;
		}
		//longshutter 0x0202
		//1 S=	0611
		//2 S=	0C22
		//3 S=	1234
		//4 S=	1845
		//5 S=	1E57
		//6 S=	2468
		//7 S=	2A7A
		//8 S=	308B
		//9 S=	369D
		//10S=	3CAE
		//11S=	42C0
		//12S=	48D1
		//13S=	4EE3
		//14S=	54F4
		//15S=	5B06
		//16S=	6117
		//17S=	6729
		//18S=	6D3A
		//19S=	734C
		//20S=	795D
		//21S=	7F6E
		//22S=	8580
		//23S=	8B91
		//24S=	91A3
		//25S=	97B4
		//26S=	9DC6
		//27S=	A3D7
		//28S=	A9E9
		//29S=	AFFA
		//30S=	B60C
		//31S=	BC1D
		//32S=	C22F
		//CintR = (482000000*shutter*0.000010058)/(4848*64);
		CintR = (4848 * (unsigned long long)shutter) / 310272;
		Time_Farme = CintR + 0x0002;  // 1st framelength
		LOG_INF("CintR =%d \n", CintR);

		write_cmos_sensor_16_16(0x0340, Time_Farme & 0xFFFF);  // Framelength
		write_cmos_sensor_16_16(0x0202, CintR & 0xFFFF);  //shutter
		write_cmos_sensor_16_16(0x0702, 0x0600);
		write_cmos_sensor_16_16(0x0704, 0x0600);

		imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		imgsensor.current_ae_effective_frame = 2;
	} else {
		if (bNeedSetNormalMode) {
			LOG_INF("exit long shutter\n");

			write_cmos_sensor_16_16(0x0702, 0x0000);
			write_cmos_sensor_16_16(0x0704, 0x0000);

			bNeedSetNormalMode = KAL_FALSE;
		}

		write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
		write_cmos_sensor_16_16(0x0202, imgsensor.shutter);

		imgsensor.current_ae_effective_frame = 2;
	}

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

	if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (gain < BASEGAIN) {
			gain = BASEGAIN;
		} else if (gain > 32 * BASEGAIN) {
			gain = 32 * BASEGAIN;
		}
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_16_16(0x0204,reg_gain);
	/*write_cmos_sensor_16_8(0x0204,(reg_gain>>8));*/
	/*write_cmos_sensor_16_8(0x0205,(reg_gain&0xff));*/

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
static kal_uint32 streaming_control(kal_bool enable)
{
	int timeout = (10000 / imgsensor.current_fps) + 1;
	int i = 0;
	int framecnt = 0;

	LOG_INF("streaming_enable(0= Sw Standby,1= streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor_16_8(0x0100, 0X01);
		mDELAY(10);
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
	}
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
0x04C8,
0x1A10,
0xF8A1,
0x04CC,
0xF000,
0xBA70,
0x0020,
0x9445,
0x0020,
0x502E,
0x0020,
0x0070,
0xB510,
0xF000,
0xFAB7,
0x49FF,
0x2001,
0x8008,
0xBD10,
0xE92D,
0x41F0,
0x4CFD,
0x4FFB,
0x2600,
0xF8B4,
0x526A,
0x8838,
0xB108,
0xF8A4,
0x626A,
0xF000,
0xFAAB,
0x803E,
0xF8A4,
0x526A,
0xE8BD,
0x81F0,
0xE92D,
0x41F0,
0x4607,
0x48F2,
0x460E,
0x2200,
0x6840,
0xB284,
0x0C05,
0x4621,
0x4628,
0xF000,
0xFA9E,
0x4631,
0x4638,
0xF000,
0xFA9F,
0x4FED,
0xF24D,
0x260C,
0xF44F,
0x6180,
0x783A,
0x4630,
0xF000,
0xFA91,
0x7878,
0xB3B8,
0x2200,
0x2180,
0x4630,
0xF000,
0xFA8A,
0x48E6,
0x8800,
0x4BE6,
0xF8A3,
0x025C,
0x48E4,
0x1D00,
0x8800,
0xF8A3,
0x025E,
0xF8B3,
0x025C,
0xF8B3,
0x125E,
0x1842,
0xD002,
0x0280,
0xFBB0,
0xF2F2,
0xB291,
0x4ADE,
0xF8A3,
0x1260,
0xF8B2,
0x0216,
0xF8B2,
0x2214,
0xF8A3,
0x0598,
0xF8A3,
0x259A,
0x1880,
0xD004,
0x0292,
0xFBB2,
0xF0F0,
0xF8A3,
0x059C,
0xF8B3,
0x059C,
0x180A,
0xFB01,
0x2010,
0xF340,
0x1095,
0x2810,
0xDC06,
0x2800,
0xDA05,
0x2000,
0xE003,
0xE7FF,
0x2201,
0xE7C5,
0x2010,
0x49CE,
0x8008,
0x4621,
0x4628,
0xE8BD,
0x41F0,
0x2201,
0xF000,
0xBA4A,
0xB5F0,
0x4CCA,
0xE9DD,
0x6505,
0xB108,
0x8827,
0x6007,
0xB109,
0x8860,
0x6008,
0xB112,
0x88A0,
0x1C40,
0x6010,
0xB10B,
0x88E0,
0x6018,
0xB10E,
0x7BA0,
0x6030,
0x2D00,
0xD001,
0x7BE0,
0x6028,
0xBDF0,
0xB570,
0x4606,
0x48B6,
0x2200,
0x6880,
0xB284,
0x0C05,
0x4621,
0x4628,
0xF000,
0xFA26,
0x4630,
0xF000,
0xFA2D,
0x48B8,
0x6803,
0xF8B3,
0x0174,
0x0A01,
0x48B6,
0x6842,
0xF882,
0x1050,
0xF893,
0x1175,
0xF882,
0x1052,
0xF8B3,
0x1178,
0x0A09,
0xF882,
0x1058,
0xF893,
0x1179,
0xF882,
0x105A,
0xF833,
0x1FF0,
0x6800,
0x0A09,
0xF800,
0x1FCE,
0x7859,
0x7081,
0x8859,
0x0A09,
0x7101,
0x78D9,
0x7181,
0x8C98,
0x0A00,
0x7490,
0xF893,
0x0025,
0x7510,
0x8CD8,
0x0A00,
0x7590,
0xF893,
0x0027,
0x7610,
0xF8B3,
0x00B0,
0x0A00,
0xF882,
0x007E,
0xF893,
0x00B1,
0xF882,
0x0080,
0x4895,
0xF890,
0x13B3,
0xF882,
0x1082,
0xF890,
0x03B1,
0xF882,
0x0084,
0xF893,
0x00B4,
0xF882,
0x0086,
0x2000,
0xF882,
0x0088,
0xF893,
0x1162,
0xF882,
0x1096,
0xF893,
0x1201,
0xF882,
0x109E,
0xF893,
0x1202,
0xF882,
0x10A0,
0xF882,
0x00A2,
0xF882,
0x00A4,
0xF893,
0x1205,
0xF882,
0x10A6,
0xF893,
0x1206,
0xF882,
0x10A8,
0xF893,
0x1207,
0xF882,
0x10AA,
0xF882,
0x00AC,
0x205A,
0xF882,
0x00AD,
0xF893,
0x0209,
0xF882,
0x00AE,
0x4621,
0x4628,
0xE8BD,
0x4070,
0x2201,
0xF000,
0xB9AF,
0xB570,
0x4875,
0x2200,
0x6901,
0x0C0C,
0xB28D,
0x4629,
0x4620,
0xF000,
0xF9A5,
0xF000,
0xF9B2,
0x4872,
0x7880,
0xB108,
0x224F,
0xE000,
0x2225,
0x4877,
0xF890,
0x00E4,
0x2803,
0xD107,
0xF042,
0x0280,
0xF44F,
0x6183,
0xF648,
0x207A,
0xF000,
0xF9A4,
0x4629,
0x4620,
0xE8BD,
0x4070,
0x2201,
0xF000,
0xB989,
0xB510,
0x2102,
0x2076,
0xF000,
0xF99D,
0x2102,
0x2044,
0xF000,
0xF999,
0x2140,
0x2045,
0xF000,
0xF995,
0x495D,
0x2004,
0xF8A1,
0x063A,
0xBD10,
0x4770,
0x4770,
0xB508,
0x4959,
0x2031,
0x466A,
0xF881,
0x0643,
0x205A,
0xF88D,
0x0000,
0x2101,
0x2075,
0xF000,
0xF986,
0xF89D,
0x0000,
0xBD08,
0x4770,
0x4852,
0xB510,
0x7880,
0xB118,
0x2100,
0x2044,
0xF000,
0xF975,
0x494D,
0x2002,
0xF8A1,
0x063A,
0xBD10,
0x4854,
0xF890,
0x00E4,
0x2803,
0xD001,
0xF000,
0xB973,
0xF2AF,
0x012B,
0x4846,
0xF8C0,
0x164C,
0x4946,
0x7889,
0xB111,
0xF2AF,
0x0185,
0xE001,
0xF2AF,
0x0165,
0xF8C0,
0x1648,
0xF2AF,
0x016B,
0xF8C0,
0x1644,
0xF2AF,
0x0171,
0xF8C0,
0x1650,
0xF2AF,
0x0159,
0xF8C0,
0x1654,
0x4770,
0xE92D,
0x41F0,
0x4C43,
0x4942,
0x4606,
0xF8B4,
0x7066,
0x89C9,
0xF8B4,
0x207E,
0x2000,
0xB1C1,
0x4621,
0xF8D1,
0x1090,
0xB172,
0xB18F,
0x4608,
0xF000,
0xF948,
0x4605,
0x6FA0,
0xF000,
0xF944,
0x4285,
0xD202,
0xF8D4,
0x0090,
0xE026,
0x6FA0,
0xE024,
0x2F00,
0xD1FB,
0x2A00,
0xD024,
0x4608,
0xE01E,
0x4928,
0x888D,
0x6889,
0x424B,
0xB177,
0x482F,
0x6F40,
0xE010,
0x4242,
0xE000,
0x4602,
0x2900,
0xDB0F,
0x428A,
0xDD0F,
0x4630,
0xE8BD,
0x41F0,
0xF000,
0xB928,
0x2A00,
0xD00C,
0x4827,
0xF8D0,
0x0088,
0xB125,
0x2800,
0xDAED,
0xE7EA,
0x4619,
0xE7ED,
0xF000,
0xF920,
0x60E0,
0x2001,
0xE63D,
0xE92D,
0x47F0,
0x4681,
0x460F,
0x4608,
0xF000,
0xF91B,
0x4C1B,
0x2600,
0x8A60,
0xB110,
0xF000,
0xF91A,
0x8266,
0x4D19,
0x8828,
0x2801,
0xD160,
0x8BA0,
0x2800,
0xD15D,
0x2F00,
0xD15B,
0x4F10,
0x6838,
0xF8B0,
0x0314,
0xB138,
0x8928,
0x1C40,
0xB280,
0x8128,
0x28FF,
0xD901,
0x8CA0,
0x8128,
0x480F,
0x60EE,
0xF8B0,
0x805E,
0xE01B,
0x0020,
0x8045,
0x0020,
0x502E,
0x0020,
0x0062,
0x0040,
0x0494,
0x0020,
0xE038,
0x0040,
0x00D0,
0x0040,
0x10A4,
0x0020,
0x662C,
0x0020,
0x9008,
0x0020,
0x2036,
0x0020,
0xE00D,
0x0020,
0xC02B,
0x0020,
0x8035,
0x0040,
0x0070,
0xF240,
0x31FF,
0x200B,
0xF000,
0xF8E2,
0x6838,
0xF8B0,
0x1312,
0xB119,
0x4648,
0xF000,
0xF8C7,
0xE00A,
0xF8B0,
0x0314,
0xB1C0,
0x8928,
0xF9B4,
0x1024,
0x4288,
0xDB13,
0x4648,
0xF7FF,
0xFF5A,
0xB178,
0x812E,
0xF000,
0xF8D0,
0x68E8,
0x6128,
0x8E20,
0xB118,
0x8E60,
0xB918,
0xF000,
0xF8CD,
0x8E60,
0xB110,
0x89E8,
0x8728,
0x8666,
0x4640,
0xE8BD,
0x47F0,
0xF000,
0xB8C8,
0xE8BD,
0x87F0,
0xB510,
0x2160,
0x200B,
0xF000,
0xF8C6,
0x0681,
0xEA4F,
0x6140,
0xD505,
0x2900,
0xDA0B,
0xE8BD,
0x4010,
0xF000,
0xB8C1,
0x2900,
0xDA03,
0xE8BD,
0x4010,
0xF000,
0xB8C0,
0x0680,
0xD503,
0xE8BD,
0x4010,
0xF000,
0xB8BF,
0xBD10,
0xB570,
0x4C1E,
0x2000,
0x8020,
0xF2AF,
0x40DF,
0x4D1C,
0x6128,
0xF2AF,
0x40D9,
0x2200,
0xF2AF,
0x41B9,
0x61A8,
0x4819,
0xF000,
0xF8B2,
0x2200,
0xF2AF,
0x31D5,
0x6060,
0x4817,
0xF000,
0xF8AB,
0x2200,
0xF2AF,
0x4113,
0x60A0,
0x4814,
0xF000,
0xF8A4,
0x2200,
0xF2AF,
0x21ED,
0x60E0,
0x4812,
0xF000,
0xF89D,
0x6120,
0xF2AF,
0x2049,
0x2200,
0xF2AF,
0x210B,
0x63E8,
0x480E,
0xF000,
0xF893,
0x2200,
0xF2AF,
0x1185,
0x480C,
0xF000,
0xF88D,
0x2200,
0xF2AF,
0x01A7,
0xE8BD,
0x4070,
0x4809,
0xF000,
0xB885,
0x0020,
0x8045,
0x0020,
0x4008,
0x0100,
0x0D02,
0x0000,
0xCD67,
0x0000,
0xE13A,
0x0000,
0xB172,
0x0000,
0xD756,
0x0000,
0x3557,
0x0000,
0x3106,
0xF645,
0x0C25,
0xF2C0,
0x0C00,
0x4760,
0xF645,
0x1CF3,
0xF2C0,
0x0C00,
0x4760,
0xF24A,
0x4CD7,
0xF2C0,
0x0C00,
0x4760,
0xF240,
0x2C0D,
0xF2C0,
0x0C01,
0x4760,
0xF246,
0x7CCD,
0xF2C0,
0x0C00,
0x4760,
0xF247,
0x2CB1,
0xF2C0,
0x0C00,
0x4760,
0xF247,
0x2C4F,
0xF2C0,
0x0C00,
0x4760,
0xF647,
0x7C01,
0xF2C0,
0x0C00,
0x4760,
0xF647,
0x6C63,
0xF2C0,
0x0C00,
0x4760,
0xF247,
0x0C0D,
0xF2C0,
0x0C00,
0x4760,
0xF24A,
0x4C5F,
0xF2C0,
0x0C00,
0x4760,
0xF245,
0x6CA5,
0xF2C0,
0x0C00,
0x4760,
0xF245,
0x5C1F,
0xF2C0,
0x0C00,
0x4760,
0xF245,
0x5C7F,
0xF2C0,
0x0C00,
0x4760,
0xF245,
0x2C31,
0xF2C0,
0x0C00,
0x4760,
0xF240,
0x2CAB,
0xF2C0,
0x0C00,
0x4760,
0xF245,
0x4CF3,
0xF2C0,
0x0C00,
0x4760,
0xF245,
0x5C39,
0xF2C0,
0x0C00,
0x4760,
0xF240,
0x7C11,
0xF2C0,
0x0C00,
0x4760,
0xF240,
0x2CD9,
0xF2C0,
0x0C00,
0x4760,
0xF245,
0x4C05,
0xF2C0,
0x0C00,
0x4760,
0xF245,
0x3CAF,
0xF2C0,
0x0C00,
0x4760,
0xF245,
0x2C4B,
0xF2C0,
0x0C00,
0x4760,
0xF64A,
0x5CE7,
0xF2C0,
0x0C00,
0x4760,
0x0000,
0x0000,
0x0000,
0x0000,
0x0000,
0x0000,
0xD530,
0x0301,
0x0000,
0x5E00,

};


static kal_uint16 addr_data_pair_global[] = {
0x602A, 0x1662,
0x6F12, 0x1E00,
0x602A, 0x1C9A,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x602A, 0x0FF2,
0x6F12, 0x0020,
0x602A, 0x0EF6,
0x6F12, 0x0100,
0x602A, 0x23B2,
0x6F12, 0x0001,
0x602A, 0x0FE4,
0x6F12, 0x0107,
0x6F12, 0x07D0,
0x602A, 0x12F8,
0x6F12, 0x3D09,
0x602A, 0x0E18,
0x6F12, 0x0040,
0x602A, 0x1066,
0x6F12, 0x000C,
0x602A, 0x13DE,
0x6F12, 0x0000,
0x602A, 0x12F2,
0x6F12, 0x0F0F,
0x602A, 0x13DC,
0x6F12, 0x806F,
0xF46E, 0x00C3,
0xF46C, 0xBFA0,
0xF44A, 0x0007,
0xF456, 0x000A,
0x6028, 0x2000,
0x602A, 0x12F6,
0x6F12, 0x7008,
0x0BC6, 0x0000,
0x0B36, 0x0001,
0x6028, 0x2000,
0x602A, 0x2BC2,
0x6F12, 0x0020,
0x602A, 0x2BC4,
0x6F12, 0x0020,
0x602A, 0x6204,
0x6F12, 0x0001,
0x602A, 0x6208,
0x6F12, 0x0000,
0x6F12, 0x0030,
0x6028, 0x2000,
0x602A, 0x17C0,
0x6F12, 0x143C,
};

static kal_uint16 addr_data_pair_preview_nopd[] = {
0x6214, 0x7971,
0x6218, 0x7150,
0x0344, 0x0070,
0x0346, 0x0044,
0x0348, 0x100F,
0x034A, 0x0BFB,
0x034C, 0x0FA0,
0x034E, 0x0BB8,
0x0340, 0x0CF0,
0x0342, 0x12F0,
0x0900, 0x0011,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0001,
0x0386, 0x0001,
0x0402, 0x1010,
0x0404, 0x1000,
0x0350, 0x0000,
0x0352, 0x0000,
0x0136, 0x1800,
0x013E, 0x0000,
0x0300, 0x0008,
0x0302, 0x0001,
0x0304, 0x0006,
0x0306, 0x00F1,
0x0308, 0x0008,
0x030A, 0x0001,
0x030C, 0x0000,
0x030E, 0x0003,
0x0310, 0x005A,
0x0312, 0x0000,
0x0B06, 0x0101,
0x6028, 0x2000,
0x602A, 0x1FF6,
0x6F12, 0x0000,
0x021E, 0x0000,
0x0202, 0x0100,
0x0204, 0x0020,
0x0D00, 0x0100, //normal mode
0x0D02, 0x0001,
0x0114, 0x0301,
0x0D06, 0x01F0,
0x0D08, 0x02E0,
0x6028, 0x2000,
0x602A, 0x0F10,
0x6F12, 0x0003,
0x602A, 0x0F12,
0x6F12, 0x0200,
0x602A, 0x2BC0,
0x6F12, 0x0001,
0x0B30, 0x0000,
0x0B32, 0x0000,
0x0B34, 0x0001,
};

static kal_uint16 addr_data_pair_preview[] = {
0x6214, 0x7971,
0x6218, 0x7150,
0x0344, 0x0070,
0x0346, 0x0044,
0x0348, 0x100F,
0x034A, 0x0BFB,
0x034C, 0x0FA0,
0x034E, 0x0BB8,
0x0340, 0x0CF0,
0x0342, 0x12F0,
0x0900, 0x0011,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0001,
0x0386, 0x0001,
0x0402, 0x1010,
0x0404, 0x1000,
0x0350, 0x0000,
0x0352, 0x0000,
0x0136, 0x1800,
0x013E, 0x0000,
0x0300, 0x0008,
0x0302, 0x0001,
0x0304, 0x0006,
0x0306, 0x00F1,
0x0308, 0x0008,
0x030A, 0x0001,
0x030C, 0x0000,
0x030E, 0x0003,
0x0310, 0x005A,
0x0312, 0x0000,
0x0B06, 0x0101,
0x6028, 0x2000,
0x602A, 0x1FF6,
0x6F12, 0x0000,
0x021E, 0x0000,
0x0202, 0x0100,
0x0204, 0x0020,
0x0D00, 0x0101, //tail mode
0x0D02, 0x0101,
0x0114, 0x0301,
0x0D06, 0x01F0,
0x0D08, 0x02E0,
0x6028, 0x2000,
0x602A, 0x0F10,
0x6F12, 0x0003,
0x602A, 0x0F12,
0x6F12, 0x0200,
0x602A, 0x2BC0,
0x6F12, 0x0001,
0x0B30, 0x0000,
0x0B32, 0x0000,
0x0B34, 0x0001,
};

static kal_uint16 addr_data_pair_slimvideo_nopd[] = {
0x6214, 0x7971,
0x6218, 0x7150,
0x0344, 0x0000,
0x0346, 0x0008,
0x0348, 0x1077,
0x034A, 0x0C37,
0x034C, 0x1070,
0x034E, 0x0C30,
0x0340, 0x0CF2,
0x0342, 0x12F0,
0x0900, 0x0011,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0001,
0x0386, 0x0001,
0x0402, 0x1010,
0x0404, 0x1000,
0x0350, 0x0008,
0x0352, 0x0000,
0x0136, 0x1800,
0x013E, 0x0000,
0x0300, 0x0008,
0x0302, 0x0001,
0x0304, 0x0006,
0x0306, 0x00F1,
0x0308, 0x0008,
0x030A, 0x0001,
0x030C, 0x0000,
0x030E, 0x0003,
0x0310, 0x005A,
0x0312, 0x0000,
0x0B06, 0x0101,
0x6028, 0x2000,
0x602A, 0x1FF6,
0x6F12, 0x0000,
0x021E, 0x0000,
0x0202, 0x0100,
0x0204, 0x0020,
0x0D00, 0x0100, //normal mode
0x0D02, 0x0001,
0x0114, 0x0301,
0x0D06, 0x0208,
0x0D08, 0x0300,
0x6028, 0x2000,
0x602A, 0x0F10,
0x6F12, 0x0003,
0x602A, 0x0F12,
0x6F12, 0x0200,
0x602A, 0x2BC0,
0x6F12, 0x0001,
0x0B30, 0x0000,
0x0B32, 0x0000,
0x0B34, 0x0001,
};

static kal_uint16 addr_data_pair_slimvideo[] = {
0x6214, 0x7971,
0x6218, 0x7150,
0x0344, 0x0000,
0x0346, 0x0008,
0x0348, 0x1077,
0x034A, 0x0C37,
0x034C, 0x1070,
0x034E, 0x0C30,
0x0340, 0x0CF2,
0x0342, 0x12F0,
0x0900, 0x0011,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0001,
0x0386, 0x0001,
0x0402, 0x1010,
0x0404, 0x1000,
0x0350, 0x0008,
0x0352, 0x0000,
0x0136, 0x1800,
0x013E, 0x0000,
0x0300, 0x0008,
0x0302, 0x0001,
0x0304, 0x0006,
0x0306, 0x00F1,
0x0308, 0x0008,
0x030A, 0x0001,
0x030C, 0x0000,
0x030E, 0x0003,
0x0310, 0x005A,
0x0312, 0x0000,
0x0B06, 0x0101,
0x6028, 0x2000,
0x602A, 0x1FF6,
0x6F12, 0x0000,
0x021E, 0x0000,
0x0202, 0x0100,
0x0204, 0x0020,
0x0D00, 0x0101, //tail mode
0x0D02, 0x0101,
0x0114, 0x0301,
0x0D06, 0x0208,
0x0D08, 0x0300,
0x6028, 0x2000,
0x602A, 0x0F10,
0x6F12, 0x0003,
0x602A, 0x0F12,
0x6F12, 0x0200,
0x602A, 0x2BC0,
0x6F12, 0x0001,
0x0B30, 0x0000,
0x0B32, 0x0000,
0x0B34, 0x0001,
};

static void sensor_init(void)
{
	/*Global setting */
	LOG_INF("sensor_init E\n");

	write_cmos_sensor_16_16(0x6028, 0x4000);	 // Page pointer HW
	write_cmos_sensor_16_16(0x0000, 0x0006);	 // Version
	write_cmos_sensor_16_16(0x0000, 0x30D5);
	write_cmos_sensor_16_16(0x6214, 0x7971);	 // Reset
	write_cmos_sensor_16_16(0x6218, 0x7150);	 // Reset
	mdelay(3); // must add 3ms!
	write_cmos_sensor_16_16(0x0A02, 0x7800);
	write_cmos_sensor_16_16(0x6028, 0x4000); //TNP burst start
	write_cmos_sensor_16_16(0x6004, 0x0001);
	write_cmos_sensor_16_16(0x6028, 0x2000);
	write_cmos_sensor_16_16(0x602A, 0x3EAC);
	LOG_INF("Before uTnpArrayInit\n");
	iWriteRegI2C((u8*)uTnpArrayInit , (u16)sizeof(uTnpArrayInit), imgsensor.i2c_write_id); // TNP burst
	LOG_INF("After uTnpArrayInit\n");
	write_cmos_sensor_16_16(0x6028, 0x4000);
	write_cmos_sensor_16_16(0x6004, 0x0002); //TNP burst end
	LOG_INF("Before addr_data_pair_globalX\n");
	table_write_cmos_sensor(addr_data_pair_global,
		sizeof(addr_data_pair_global) / sizeof(kal_uint16)); //Global & Functional
	LOG_INF("After addr_data_pair_global\n");

}	/*	sensor_init  */

static void preview_setting(void)
{
	if (badOtpFlag) {
		table_write_cmos_sensor(addr_data_pair_preview_nopd,
			sizeof(addr_data_pair_preview_nopd) / sizeof(kal_uint16));
	} else {
		table_write_cmos_sensor(addr_data_pair_preview,
			sizeof(addr_data_pair_preview) / sizeof(kal_uint16));
	}
}	/*	preview_setting  */

#if 0
/* Pll Setting - VCO = 280Mhz*/
static void capture_setting(void)
{
	table_write_cmos_sensor(addr_data_pair_capture,
		   sizeof(addr_data_pair_capture) / sizeof(kal_uint16));
}

static void normal_video_setting(kal_uint16 currefps)
{

}

static void hs_video_setting(void)
{
	LOG_INF("E\n");
}
#endif

static void slim_video_setting(void)
{
	if (badOtpFlag) {
		table_write_cmos_sensor(addr_data_pair_slimvideo_nopd,
			sizeof(addr_data_pair_slimvideo_nopd) / sizeof(kal_uint16));
	} else {
		table_write_cmos_sensor(addr_data_pair_slimvideo,
			sizeof(addr_data_pair_slimvideo) / sizeof(kal_uint16));
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
	kal_uint16 data_index = 0;
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
				*sensor_id = S5K3M5_SENSOR_ID;
				check_badEeprom();
				imgsensor_info.module_id = read_module_id();
				LOG_INF("s5k3m5_module_id=%d\n",imgsensor_info.module_id);
				if(deviceInfo_register_value == 0x00){
					Oplusimgsensor_Registdeviceinfo("Cam_r2", DEVICE_VERSION_S5K3M5, imgsensor_info.module_id);
					deviceInfo_register_value = 0x01;
				}
				/*Readout Master+wide stereo data*/
				data_index = 0;
				read_eepromData(&EepData[data_index],
								S5K3M5_STEREO_START_ADDR,
								DUALCAM_CALI_DATA_LENGTH, SLAVEADDR);
				data_index += DUALCAM_CALI_DATA_LENGTH;
				/*Readout SN data*/
				read_eepromData(&EepData[data_index],
								0xB0, CAMERA_MODULE_SN_LENGTH, SLAVEADDR);
				data_index += CAMERA_MODULE_SN_LENGTH;
				read_eepromData(&EepData[data_index], 0x0, 2, SLAVEADDR);
				data_index += 2;
				read_eepromData(&EepData[data_index], 0x6, 6, SLAVEADDR);
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
	LOG_INF("S5K3M5 open start\n");

	LOG_INF("PLATFORM:MT6750,MIPI 4LANE\n");

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
	if (imgsensor_info.sensor_id !=  sensor_id) {
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	LOG_INF("S5K3M5 sensor_init start\n");
	/* initail sequence write in  */
	sensor_init();
	LOG_INF("S5K3M5 sensor_init End\n");
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
	imgsensor.test_pattern = 0;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("S5K3M5 open End\n");
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
	/* No Need to implement this function */
	streaming_control(KAL_FALSE);
	LOG_INF("close\n");

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
	LOG_INF("S5K3m5 preview start\n");

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
	LOG_INF("S5K3m5 preview End\n");
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

	preview_setting();
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
	preview_setting();
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

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 	 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	if (badOtpFlag) {
		sensor_info->PDAF_Support = 0;
	} else {
		sensor_info->PDAF_Support = 2;
	}
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
	    set_dummy();
	    break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
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
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_uint8 enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		/* 0x5E00[8]: 1 enable,  0 disable*/
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK*/
		write_cmos_sensor_16_16(0x0600, 0x0001);
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
static kal_uint32 s5k3m5_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	LOG_INF("s5k3m5_awb_gain: 0x100\n");

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
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	case SENSOR_FEATURE_GET_EEPROM_DATA:
	{
		kal_uint16 data_lens = DUALCAM_CALI_EEPROM_DATA_LENGTH;
		LOG_INF("SENSOR_FEATURE_GET_EEPROM_DATA:%d data_lens:%d\n",
					*feature_para_len, data_lens);
		memcpy(&feature_para[0], EepData, data_lens);
		break;
	}
	case SENSOR_FEATURE_SET_SENSOR_OTP:
	{
		kal_int32 ret = IMGSENSOR_RETURN_SUCCESS;
		LOG_INF("SENSOR_FEATURE_SET_SENSOR_OTP length :%d\n", (UINT32)*feature_para_len);
		ret = call_eepromwrite_Service((ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(feature_para), SLAVEADDR);
		if (ret == IMGSENSOR_RETURN_SUCCESS)
			return ERROR_NONE;
		else
			return ERROR_MSDK_IS_ACTIVATED;
	}
	#endif
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
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 8000000;
		break;
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
		set_test_pattern_mode((kal_uint8)*feature_data);
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
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info_fullsize,sizeof(struct SET_PD_BLOCK_INFO_T));
				break;
			default:
				break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", (UINT16)*feature_data);
		/*PDAF capacity enable or not, 2p8 only full size support PDAF*/
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
				break;
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
				break;
		}
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT32) *feature_data, (UINT32) *(feature_data + 1),
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
			default:
					rate = 0;
					break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
		}
		break;
	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n",
			(UINT16)*feature_data);
		pvcinfo =
		 (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		default:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		}
	break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*feature_return_para_32 = 1; /*BINNING_NONE*/
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
			break;
		}

		LOG_INF("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d\n",
			(UINT32)*feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		/* modify to separate 3hdr and remosaic */
		if (imgsensor.sensor_mode == IMGSENSOR_MODE_CAPTURE) {
			/*write AWB gain to sensor*/
			feedback_awbgain((UINT32)*(feature_data_32 + 1),
					(UINT32)*(feature_data_32 + 2));
		} else {
			s5k3m5_awb_gain(
				(struct SET_SENSOR_AWB_GAIN *) feature_para);
		}
		break;
	case SENSOR_FEATURE_GET_4CELL_DATA:
		{
			int type = (kal_uint16)(*feature_data);
			if (type == FOUR_CELL_CAL_TYPE_ALL) {
				LOG_INF("SENSOR_FEATURE_GET_4CELL_DATA type=%d\n", type);
			} else if (type == FOUR_CELL_CAL_TYPE_GAIN_TBL) {
				LOG_INF("SENSOR_FEATURE_GET_4CELL_DATA type=%d\n", type);
			} else {
				memset((void *)(*(feature_data+1)), 0, 4);
				LOG_INF("No type %d buffer on this sensor\n", type);
			}
			break;
		}
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
		*feature_return_para_32 = imgsensor.current_ae_effective_frame;
		break;
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
		memcpy(feature_return_para_32, &imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
		break;
	#endif
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

UINT32 S5K3M5_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!= NULL) {
		*pfFunc= &sensor_func;
	}
	return ERROR_NONE;
}	/*	S5K3M5_MIPI_RAW_SensorInit	*/
