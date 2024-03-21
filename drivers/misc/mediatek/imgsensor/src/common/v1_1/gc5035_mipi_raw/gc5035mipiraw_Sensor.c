 /*
 *
 * Filename:
 * ---------
 *     GC5035mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *-----------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
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

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "imgsensor_common.h"
#include "imgsensor_eeprom_v1.h"
#include "gc5035mipiraw_Sensor.h"

/************************** Modify Following Strings for Debug **************************/
#define PFX "gc5035_camera_sensor"
#define LOG_1 LOG_INF("GC5035MIPI, 2LANE\n")
/****************************   Modify end    *******************************************/
#define GC5035_DEBUG
#if defined(GC5035_DEBUG)
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include <soc/oplus/system/oplus_project.h>
#endif
#ifdef OPLUS_FEATURE_CAMERA_COMMON
static kal_uint8 EepData[CALI_DATA_SLAVE_LENGTH_8ALIGN
			+CAMERA_MODULE_SN_LENGTH+CAMERA_MODULE_INFO_LENGTH];
#define SLAVEADDR 0xA0
#endif
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static kal_uint32 Dgain_ratio = 256;
extern u32 pinSetIdx;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = GC5035_SENSOR_ID,    /*record sensor id defined in Kd_imgsensor.h*/
	.checksum_value = 0xcde448ca,         /*checksum value for Camera Auto Test*/

	.pre = {
		.pclk = 87600000,                 /*record different mode's pclk*/
		.linelength = 1460,               /*record different mode's linelength*/
		.framelength = 2008,              /*record different mode's framelength*/
		.startx = 0,                      /*record different mode's startx of grabwindow*/
		.starty = 0,                      /*record different mode's starty of grabwindow*/
		.grabwindow_width = 1296,         /*record different mode's width of grabwindow */
		.grabwindow_height = 972,         /*record different mode's height of grabwindow */
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,  /* unit , ns */
		.mipi_pixel_rate = 87600000,
		/* following for GetDefaultFramerateByScenario() */
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 175200000,                /*record different mode's pclk*/
		.linelength = 2920,               /*record different mode's linelength*/
		.framelength = 2008,              /*record different mode's framelength*/
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 175200000,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 87600000,
		.linelength = 1460,
		.framelength = 2008,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1296,
		.grabwindow_height = 972,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 87600000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 175200000,
		.linelength = 1436,
		.framelength = 1016,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 640,
		.grabwindow_height = 480,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 175200000,
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 87600000,
		.linelength = 1436,
		.framelength = 2008,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 87600000,
		.max_framerate = 300,
	},
	.custom1 = {
		/*capture for PIP 24fps relative information*/
		/*capture1 mode must use same framelength, linelength with Capture mode for shutter calculate*/
		.pclk = 141600000,
		.linelength = 2920,
		.framelength = 2008,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 141600000,
		.max_framerate = 240,             /*less than 13M(include 13M)*/
		/*cap1 max framerate is 24fps, 16M max framerate is 20fps, 20M max framerate is 15fps*/
	},
	.margin = 16,                                            /*sensor framelength & shutter margin*/
	.min_shutter = 4,                                       /*min shutter*/
	.max_frame_length = 12000,
	/*max framelength by sensor register's limitation*/     /*5fps*/
	.ae_shut_delay_frame = 0,
	/*shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2*/
	.ae_sensor_gain_delay_frame = 0,
	/*sensor gain delay frame for AE cycle, 2 frame with ispGain_delay-sensor_gain_delay=2-0=2*/
	.ae_ispGain_delay_frame = 2,                            /*isp gain delay frame for AE cycle*/
	.ihdr_support = 0,                                      /*1 support; 0 not support*/
	.ihdr_le_firstline = 0,                                 /*1 le first ; 0, se first*/
	.sensor_mode_num = 6,                                   /*support sensor mode num*/

	.cap_delay_frame = 2,                                   /*enter capture delay frame num*/
	.pre_delay_frame = 2,                                   /*enter preview delay frame num*/
	.video_delay_frame = 2,                                 /*enter video delay frame num*/
	.hs_video_delay_frame = 2,                              /*enter high speed video  delay frame num*/
	.slim_video_delay_frame = 2,                            /*enter slim video delay frame num*/
	.custom1_delay_frame = 2,                               /*enter custom1 delay frame num*/
	.frame_time_delay_frame = 2,                            /*enter custom1 delay frame num*/

	.isp_driving_current = ISP_DRIVING_4MA,                 /*mclk driving current*/
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,    /*sensor_interface_type*/
	.mipi_sensor_type = MIPI_OPHY_NCSI2,                    /*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
	/*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
	#if defined(GC5035_MIRROR_NORMAL)
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R, /*sensor output first pixel color*/
	#elif defined(GC5035_MIRROR_H)
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr, /*sensor output first pixel color*/
	#elif defined(GC5035_MIRROR_V)
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb, /*sensor output first pixel color*/
	#elif defined(GC5035_MIRROR_HV)
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B, /*sensor output first pixel color*/
	#else
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R, /*sensor output first pixel color*/
	#endif
	.mclk = 24,                                             /*mclk value, suggest 24 or 26 for 24Mhz or 26Mhz*/
	.mipi_lane_num = SENSOR_MIPI_2_LANE,                    /*mipi lane num*/
	.i2c_addr_table = {0x6e, 0x7e, 0xff},
	/*record sensor support all write id addr, only supprt 4must end with 0xff*/
};

static struct imgsensor_struct imgsensor = {
	#if defined(GC5035_MIRROR_NORMAL)
	.mirror = IMAGE_NORMAL,       /*mirrorflip information*/
	#elif defined(GC5035_MIRROR_H)
	.mirror = IMAGE_H_MIRROR,       /*mirrorflip information*/
	#elif defined(GC5035_MIRROR_V)
	.mirror = IMAGE_V_MIRROR,       /*mirrorflip information*/
	#elif defined(GC5035_MIRROR_HV)
	.mirror = IMAGE_HV_MIRROR,       /*mirrorflip information*/
	#else
	.mirror = IMAGE_NORMAL,       /*mirrorflip information*/
	#endif
	.sensor_mode = IMGSENSOR_MODE_INIT,
	/*IMGSENSOR_MODE enum value, record current sensor mode,
	such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video*/
	.shutter = 0x258,             /*current shutter*/
	.gain = 0x40,                 /*current gain*/
	.dummy_pixel = 0,             /*current dummypixel*/
	.dummy_line = 0,              /*current dummyline*/
	.current_fps = 300,           /*full size current fps : 24fps for PIP, 30fps for Normal or ZSD*/
	.autoflicker_en = KAL_FALSE,
	/*auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker*/
	.test_pattern = KAL_FALSE,
	/*test pattern mode or not KAL_FALSE for in test pattern mode, KAL_TRUE for normal output*/
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW, /*current scenario id*/
	.ihdr_en = 0,                 /*sensor need support LE, SE with HDR feature*/
	.i2c_write_id = 0x6e,         /*record current sensor's i2c write id*/
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[6] = {
	{ 2592, 1944,    0,    0, 2592, 1944, 1296,  972,    0,    0, 1296,  972,    0,    0, 1296,  972 }, /*preview*/
	{ 2592, 1944,    0,    0, 2592, 1944, 2592, 1944,    0,    0, 2592, 1944,    0,    0, 2592, 1944 }, /*capture*/
	{ 2592, 1944,    0,    0, 2592, 1944, 1296,  972,    0,    0, 1296,  972,    0,    0, 1296,  972 }, /*video*/
	{ 2592, 1944,  656,  492, 1280,  960,  640,  480,    0,    0,  640,  480,    0,    0,  640,  480 }, /*hs video*/
	{ 2592, 1944,   16,  252, 2560, 1440, 1280,  720,    0,    0, 1280,  720,    0,    0, 1280,  720 }, /*sl video*/
	{ 2592, 1944,    0,    0, 2592, 1944, 2592, 1944,    0,    0, 2592, 1944,    0,    0, 2592, 1944 } /*custom1*/
};


static struct gc5035_otp gc5035_otp_data = { 0 };
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[1] = { (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 1, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = { (char)(addr & 0xFF), (char)(para & 0xFF) };

	iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
}

static kal_uint8 gc5035_otp_read_byte(kal_uint16 addr)
{
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x69, (addr >> 8) & 0x1f);
	write_cmos_sensor(0x6a, addr & 0xff);
	write_cmos_sensor(0xf3, 0x20);
	return read_cmos_sensor(0x6c);
}

#if defined(GC5035_OTP_CUSTOMER)
static void gc5035_otp_read_group(kal_uint16 addr, kal_uint8 *data, kal_uint16 length)
{
	kal_uint16 i = 0;

	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x69, (addr >> 8) & 0x1f);
	write_cmos_sensor(0x6a, addr & 0xff);
	write_cmos_sensor(0xf3, 0x20);
	write_cmos_sensor(0xf3, 0x12);

	for (i = 0; i < length; i++) {
		data[i] = read_cmos_sensor(0x6c);
	}

	write_cmos_sensor(0xf3, 0x00);
}
#endif

static void gc5035_otp_read_sensor_info(void)
{
	kal_uint8 flag_dd = 0;
	kal_uint16 temp = 0, i = 0, j = 0;


	/* DPC */
	flag_dd = gc5035_otp_read_byte(DPC_FLAG_OFFSET);
	LOG_INF("flag_dd = 0x%x\n", flag_dd);
	switch (flag_dd & 0x03) {
	case 0x00:
		LOG_INF("DD is empty !!\n");
		gc5035_otp_data.dd_flag = 0x00;
		break;
	case 0x01:
		LOG_INF("DD is valid!!\n");
		gc5035_otp_data.dd_num = gc5035_otp_read_byte(DPC_TOTAL_NUMBER_OFFSET)
						+ gc5035_otp_read_byte(DPC_ERROR_NUMBER_OFFSET);
		gc5035_otp_data.dd_flag = 0x01;
		LOG_INF("total_num = %d\n", gc5035_otp_data.dd_num);
		break;
	case 0x02:
	case 0x03:
		LOG_INF("DD is invalid !!\n");
		gc5035_otp_data.dd_flag = 0x02;
		break;
	default:
		break;
	}

	/*Register Update*/
	gc5035_otp_data.reg_flag = gc5035_otp_read_byte(REG_INFO_FLAG_OFFSET);
	LOG_INF("Register value flag = 0x%x\n", gc5035_otp_data.reg_flag);
	if ((gc5035_otp_data.reg_flag & 0x03) == 0x01)
		for (i = 0; i < REG_INFO_SIZE; i++) {
			temp = gc5035_otp_read_byte(REG_INFO_PAGE_OFFSET + REG_INFO_SIZE * 8 * i);
			for (j = 0; j < 2; j++) {
				if (((temp >> (4 * j + 3)) & 0x01) == 0x01) {// check
					gc5035_otp_data.regs[gc5035_otp_data.reg_num][0] = (temp >> (4 * j)) & 0x07;// page
					gc5035_otp_data.regs[gc5035_otp_data.reg_num][1] =
						gc5035_otp_read_byte(REG_INFO_ADDR_OFFSET + REG_INFO_SIZE * 8 * i + 0x10 * j);
					gc5035_otp_data.regs[gc5035_otp_data.reg_num][2] =
						gc5035_otp_read_byte(REG_INFO_VALUE_OFFSET + REG_INFO_SIZE * 8 * i + 0x10 * j);
					gc5035_otp_data.reg_num++;
				}
			}
		}
}

static void gc5035_otp_update_dd(void)
{
	kal_uint8 state = 0;
	kal_uint8 n = 0;

	if (0x01 == gc5035_otp_data.dd_flag) {
		LOG_INF("DD auto load start!\n");
		write_cmos_sensor(0xfe, 0x02);
		write_cmos_sensor(0xbe, 0x00);//allow DD write
		write_cmos_sensor(0xa9, 0x01);// clear sram
		write_cmos_sensor(0x09, 0x33);
		write_cmos_sensor(0x01, (gc5035_otp_data.dd_num >> 8) & 0x07);
		write_cmos_sensor(0x02, gc5035_otp_data.dd_num & 0xff);
		write_cmos_sensor(0x03, 0x00);
		write_cmos_sensor(0x04, 0x80);//otp bad pixel start pos
		write_cmos_sensor(0x95, 0x0a);
		write_cmos_sensor(0x96, 0x30);
		write_cmos_sensor(0x97, 0x0a);
		write_cmos_sensor(0x98, 0x32);
		write_cmos_sensor(0x99, 0x07);
		write_cmos_sensor(0x9a, 0xa9);
		write_cmos_sensor(0xf3, 0x80);// auto load start pulse
		while (n < 3) {
			state = read_cmos_sensor(0x06);
			if ((state | 0xfe) == 0xff) {
				mdelay(10);
			} else {
				n = 3;
			}
			n++;
		}
		write_cmos_sensor(0xbe, 0x01);
		write_cmos_sensor(0x09, 0x00);//close auto load
		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0x80, 0x02);/* [1]DD_EN */
		write_cmos_sensor(0xfe, 0x00);
	}
}

static void gc5035_otp_update_reg(void)
{
	kal_uint8 i = 0;

	LOG_INF("register update start!");

	/* Register Update */

	if ((gc5035_otp_data.reg_flag & 0x03) == 0x01) {
		for (i = 0; i < gc5035_otp_data.reg_num; i++) {
			write_cmos_sensor(0xfe, gc5035_otp_data.regs[i][0]);
			write_cmos_sensor(gc5035_otp_data.regs[i][1], gc5035_otp_data.regs[i][2]);
		}
	}
	LOG_INF("register update finish!");

}

#if defined(GC5035_OTP_CUSTOMER)
static void gc5035_otp_read_module_info(void)
{
	kal_uint8 index = 0, flag_module = 0;
	kal_uint8 module_id = 0, lens_id = 0, year = 0, month = 0, day = 0;
	kal_uint8 info[MODULE_INFO_SIZE] = { 0 };
	kal_uint16 check = 0, i = 0;

	memset(&info, 0, MODULE_INFO_SIZE * sizeof(kal_uint8));
	flag_module = gc5035_otp_read_byte(MODULE_INFO_FLAG_OFFSET);
	LOG_INF("flag_module = 0x%x\n", flag_module);

	for (index = 0; index < 2; index++)
		switch ((flag_module << (2 * index)) & 0x0c) {
		case 0x00:
			LOG_INF("module info group %d is empty!!\n", index + 1);
			break;
		case 0x04:
			LOG_INF("module info group %d is valid!!\n", index + 1);
			gc5035_otp_read_group(MODULE_INFO_OFFSET + MODULE_INFO_SIZE * 8 * index, &info[0], MODULE_INFO_SIZE);
			for (i = 0; i < 6; i++)
				LOG_INF("addr = 0x%x, data = 0x%x\n", MODULE_INFO_OFFSET + MODULE_INFO_SIZE * 8 * index + i * 8, info[i]);

			for (i = 0; i < MODULE_INFO_SIZE - 1; i++)
				check += info[i];

			if ((check % 255 + 1) == info[MODULE_INFO_SIZE - 1]) {
				module_id = info[0];
				lens_id = info[1];
				year = info[2];
				month = info[3];
				day = info[4];

				LOG_INF("module_id = 0x%x\n", module_id);
				LOG_INF("lens_id = 0x%x\n", lens_id);
				LOG_INF("data = %d-%d-%d\n", year, month, day);
			} else
				LOG_INF("module info Check sum %d error!! check sum = %d, sum = %d\n",
					index + 1, info[MODULE_INFO_SIZE - 1], (check % 255 + 1));
			break;
		case 0x08:
		case 0x0c:
			LOG_INF("module info group %d is invalid!!\n", index + 1);
			break;
		default:
			break;
		}
}

static void gc5035_otp_read_wb_info(void)
{
	kal_uint8  flag_wb = 0;
	kal_uint8  wb[WB_INFO_SIZE] = { 0 };
	kal_uint8  golden[WB_GOLDEN_INFO_SIZE] = { 0 };
	kal_uint16 checkwb = 0, checkgolden = 0;
	kal_uint8  index = 0, i = 0;

	memset(&wb, 0, WB_INFO_SIZE * sizeof(kal_uint8));
	memset(&golden, 0, WB_GOLDEN_INFO_SIZE * sizeof(kal_uint8));
	flag_wb = gc5035_otp_read_byte(WB_INFO_FLAG_OFFSET);
	LOG_INF("flag_wb = 0x%x\n", flag_wb);

	for (index = 0; index < 2; index++) {
		switch ((flag_wb << (2 * index)) & 0x0c) {
		case 0x00:
			LOG_INF("wb unit group %d is empty!!\n", index + 1);
			gc5035_otp_data.wb_flag = gc5035_otp_data.wb_flag | 0x00;
			break;
		case 0x04:
			LOG_INF("wb unit group %d is valid!!\n", index + 1);
			gc5035_otp_read_group(WB_INFO_OFFSET + WB_INFO_SIZE * 8 * index, &wb[0], WB_INFO_SIZE);
			for (i = 0; i < WB_INFO_SIZE; i++) {
				LOG_INF("addr = 0x%x, data = 0x%x\n", WB_INFO_OFFSET +  WB_INFO_SIZE * 8 * index + i * 8, wb[i]);
			}
			for (i = 0; i < WB_INFO_SIZE - 1; i++) {
				checkwb += wb[i];
			}

			LOG_INF("cal_checkwb = 0x%x, otp_checkwb = 0x%x\n", checkwb % 255 + 1, wb[3]);

			if ((checkwb % 255 + 1) == wb[WB_INFO_SIZE - 1]) {
				gc5035_otp_data.rg_gain = (wb[0] | ((wb[1] & 0xf0) << 4)) > 0 ?
					(wb[0] | ((wb[1] & 0xf0) << 4)) : 0x400;
				gc5035_otp_data.bg_gain = (((wb[1] & 0x0f) << 8) | wb[2]) > 0 ?
					(((wb[1] & 0x0f) << 8) | wb[2]) : 0x400;
				gc5035_otp_data.wb_flag = gc5035_otp_data.wb_flag | 0x01;
			} else
				LOG_INF("wb unit check sum %d error!!\n", index + 1);
			break;
		case 0x08:
		case 0x0c:
			LOG_INF("wb unit group %d is invalid!!\n", index + 1);
			gc5035_otp_data.wb_flag = gc5035_otp_data.wb_flag | 0x02;
			break;
		default:
			break;
		}

		switch ((flag_wb << (2 * index)) & 0xc0) {
		case 0x00:
			LOG_INF("wb golden group %d is empty!!\n", index + 1);
			gc5035_otp_data.golden_flag = gc5035_otp_data.golden_flag | 0x00;
			break;
		case 0x40:
			LOG_INF("wb golden group %d is valid!!\n", index + 1);
			gc5035_otp_read_group(WB_GOLDEN_INFO_OFFSET + WB_GOLDEN_INFO_SIZE * 8 * index, &golden[0], WB_GOLDEN_INFO_SIZE);
			for (i = 0; i < 4; i++) {
				LOG_INF("addr = 0x%x, data = 0x%x\n", WB_GOLDEN_INFO_OFFSET + WB_GOLDEN_INFO_SIZE * 8 * index + i * 8, golden[i]);
			}
			for (i = 0; i < WB_GOLDEN_INFO_SIZE - 1; i++) {
				checkgolden += golden[i];
			}

			LOG_INF("cal_checkgolden = 0x%x, otp_checkgolden = 0x%x\n", checkgolden % 255 + 1, golden[3]);

			if ((checkgolden % 255 + 1) == golden[WB_GOLDEN_INFO_SIZE - 1]) {
				gc5035_otp_data.golden_rg = (golden[0] | ((golden[1] & 0xf0) << 4)) > 0 ?
					(golden[0] | ((golden[1] & 0xf0) << 4)) : RG_TYPICAL;
				gc5035_otp_data.golden_bg = (((golden[1] & 0x0f) << 8) | golden[2]) > 0 ?
					(((golden[1] & 0x0f) << 8) | golden[2]) : BG_TYPICAL;
				gc5035_otp_data.golden_flag = gc5035_otp_data.golden_flag | 0x01;
			} else
				LOG_INF("wb golden check sum %d error!!\n", index + 1);
			break;
		case 0x80:
		case 0xc0:
			LOG_INF("wb golden group %d is invalid !!\n", index + 1);
			gc5035_otp_data.golden_flag = gc5035_otp_data.golden_flag | 0x02;
			break;
		default:
			break;
		}
	}
}

static void gc5035_otp_update_wb(void)
{
	kal_uint16 r_gain_current = 0, g_gain_current = 0, b_gain_current = 0, base_gain = 0;
	kal_uint16 r_gain = 1024, g_gain = 1024, b_gain = 1024;
	kal_uint16 rg_typical = 0, bg_typical = 0;

	if (0x02 == gc5035_otp_data.golden_flag) {
		return;
	}
	if (0x01 == (gc5035_otp_data.golden_flag & 0x01)) {
		rg_typical = gc5035_otp_data.golden_rg;
		bg_typical = gc5035_otp_data.golden_bg;
		LOG_INF("golden_flag = %d, rg_typical = 0x%x, bg_typical = 0x%x\n",
			gc5035_otp_data.golden_flag, rg_typical, bg_typical);
	} else {
		rg_typical = RG_TYPICAL;
		bg_typical = BG_TYPICAL;
		LOG_INF("golden_flag = %d, rg_typical = 0x%x, bg_typical = 0x%x\n",
			gc5035_otp_data.golden_flag, rg_typical, bg_typical);
	}

	if (0x01 == (gc5035_otp_data.wb_flag & 0x01)) {
		r_gain_current = 2048 * rg_typical / gc5035_otp_data.rg_gain;
		b_gain_current = 2048 * bg_typical / gc5035_otp_data.bg_gain;
		g_gain_current = 2048;

		base_gain = (r_gain_current < b_gain_current) ? r_gain_current : b_gain_current;
		base_gain = (base_gain < g_gain_current) ? base_gain : g_gain_current;

		r_gain = 0x400 * r_gain_current / base_gain;
		g_gain = 0x400 * g_gain_current / base_gain;
		b_gain = 0x400 * b_gain_current / base_gain;
		LOG_INF("r_gain = 0x%x, g_gain = 0x%x, b_gain = 0x%x\n", r_gain, g_gain, b_gain);

		write_cmos_sensor(0xfe, 0x04);
		write_cmos_sensor(0x40, g_gain & 0xff);
		write_cmos_sensor(0x41, r_gain & 0xff);
		write_cmos_sensor(0x42, b_gain & 0xff);
		write_cmos_sensor(0x43, g_gain & 0xff);
		write_cmos_sensor(0x44, g_gain & 0xff);
		write_cmos_sensor(0x45, r_gain & 0xff);
		write_cmos_sensor(0x46, b_gain & 0xff);
		write_cmos_sensor(0x47, g_gain & 0xff);
		write_cmos_sensor(0x48, (g_gain >> 8) & 0x07);
		write_cmos_sensor(0x49, (r_gain >> 8) & 0x07);
		write_cmos_sensor(0x4a, (b_gain >> 8) & 0x07);
		write_cmos_sensor(0x4b, (g_gain >> 8) & 0x07);
		write_cmos_sensor(0x4c, (g_gain >> 8) & 0x07);
		write_cmos_sensor(0x4d, (r_gain >> 8) & 0x07);
		write_cmos_sensor(0x4e, (b_gain >> 8) & 0x07);
		write_cmos_sensor(0x4f, (g_gain >> 8) & 0x07);
		write_cmos_sensor(0xfe, 0x00);
	}
}
#endif

#if defined(GC5035_OTP_DEBUG)
	//char pu_send_cmd[1] = { (char)(0xc0 + (addr & 0xFF)) };

	//iReadRegI2C(pu_send_cmd, 1, (u8 *)data, length, imgsensor.i2c_write_id);
static void gc5035_otp_debug(void)
{
	kal_uint8 debugdata[OTP_DATA_LENGTH] = {0};
	kal_uint16 i;

	memset(&debugdata, 0, OTP_DATA_LENGTH * sizeof(kal_uint8));
	gc5035_otp_read_group(OTP_START_ADDR, &debugdata[0], OTP_DATA_LENGTH);
	for(i = 0; i < OTP_DATA_LENGTH; i++) {
		LOG_INF("addr = 0x%x, data = 0x%x\n", OTP_START_ADDR + i * 8, debugdata[i]);
	}
}
#endif
static void gc5035_otp_read(void)
{
	gc5035_otp_read_sensor_info();
#if defined(GC5035_OTP_CUSTOMER)
	gc5035_otp_read_module_info();
	gc5035_otp_read_wb_info();
#endif
#if defined(GC5035_OTP_DEBUG)
	gc5035_otp_debug();
#endif
}

static void gc5035_otp_update(void)
{
	gc5035_otp_update_dd();
#if defined(GC5035_OTP_CUSTOMER)
	gc5035_otp_update_wb();
#endif
	gc5035_otp_update_reg();
}

static void gc5035_otp_function(void)
{
	memset(&gc5035_otp_data, 0, sizeof(gc5035_otp_data));
	write_cmos_sensor(0xfa, 0x10); //otp_clk_en
	write_cmos_sensor(0xf5, 0xc1);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x67, 0xc0); //[7]otp_en [6]otpcpclk_en
	write_cmos_sensor(0x59, 0x3f);//charge pump voltage 7.43V
	write_cmos_sensor(0x55, 0x80);//[7]high voltage protect
	write_cmos_sensor(0x65, 0x30);//read time
	write_cmos_sensor(0x66, 0x03);
	write_cmos_sensor(0xfe, 0x00);

	gc5035_otp_read();
	gc5035_otp_update();

	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x67, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfa, 0x00);
}

static void set_dummy(void)
{
	kal_uint32 frame_length = imgsensor.frame_length >> 2;

	frame_length = frame_length << 2;
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x41, (frame_length >> 8) & 0x3f);
	write_cmos_sensor(0x42, frame_length & 0xff);
	LOG_INF("Exit! framelength = %d\n", frame_length);
}



static kal_uint32 return_sensor_id(void)
{
	kal_uint32 rtn_id = 0, i = 0;
	kal_uint32 idg[8] = {
		0x5035, 0x5045, 0x5055, 0x5065,
		0x5075, 0x5085, 0x5095, 0x50a5
	};
	rtn_id = (read_cmos_sensor(0xf0) << 8) | read_cmos_sensor(0xf1);
	LOG_INF("Sensor ID 0xf0 = 0x%x\n", read_cmos_sensor(0xf0));
	LOG_INF("Sensor ID 0xf1 = 0x%x\n", read_cmos_sensor(0xf1));
	LOG_INF("Sensor ID = 0x%x\n", rtn_id);
	for (i = 0; i < 8; i++) {
		if (rtn_id == idg[i]) {
			return 0x5035;
		}
	}
	return 0;
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length)
		? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	LOG_INF("framelength = %d\n", imgsensor.frame_length);
	if (min_framelength_en) {
		imgsensor.min_frame_length = imgsensor.frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}

/*************************************************************************
* FUNCTION
*    set_shutter
*
* DESCRIPTION
*    This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*    iShutter : exposured lines
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0, cal_shutter = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/*if shutter bigger than frame_length, should extend frame length first*/
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
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

	if (imgsensor.autoflicker_en) {
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			set_max_framerate(realtime_fps, 0);
		}
	} else {
		set_max_framerate(realtime_fps, 0);
	}

	cal_shutter = shutter >> 2;
	cal_shutter = cal_shutter << 2;
	Dgain_ratio = 256 * shutter / cal_shutter;

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x03, (cal_shutter >> 8) & 0x3F);
	write_cmos_sensor(0x04, cal_shutter & 0xFF);

	LOG_INF("Exit! shutter = %d, framelength = %d\n", shutter, imgsensor.frame_length);
	LOG_INF("Exit! cal_shutter = %d, ", cal_shutter);
}

static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length,
			kal_bool auto_extend_en)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0, cal_shutter = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/*if shutter bigger than frame_length, should extend frame length first*/
	spin_lock(&imgsensor_drv_lock);
	if(frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;

	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

	if (imgsensor.autoflicker_en) {
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			set_max_framerate(realtime_fps, 0);
		}
	} else {
		set_max_framerate(realtime_fps, 0);
	}

	cal_shutter = shutter >> 2;
	cal_shutter = cal_shutter << 2;
	Dgain_ratio = 256 * shutter / cal_shutter;

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x03, (cal_shutter >> 8) & 0x3F);
	write_cmos_sensor(0x04, cal_shutter & 0xFF);

	LOG_INF("Exit! shutter = %d, framelength = %d\n", shutter, imgsensor.frame_length);
	LOG_INF("Exit! cal_shutter = %d, ", cal_shutter);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = gain << 2;

	if (reg_gain < SENSOR_BASE_GAIN) {
		reg_gain = SENSOR_BASE_GAIN;
	} else if (reg_gain > SENSOR_MAX_GAIN) {
		reg_gain = SENSOR_MAX_GAIN;
	}

	return (kal_uint16)reg_gain;
}

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
	kal_uint16 reg_gain;
	kal_uint32 temp_gain;
	kal_int16 gain_index;
	kal_uint16 GC5035_AGC_Param[MAX_GAIN_INDEX][2] = {
		{  256,  0 },
		{  302,  1 },
		{  358,  2 },
		{  425,  3 },
		{  502,  8 },
		{  599,  9 },
		{  717, 10 },
		{  845, 11 },
		{ 998,  12 },
		{ 1203, 13 },
		{ 1434, 14 },
		{ 1710, 15 },
		{ 1997, 16 },
		{ 2355, 17 },
		{ 2816, 18 },
		{ 3318, 19 },
		{ 3994, 20 },
	};

	reg_gain = gain2reg(gain);

	for (gain_index = E_GAIN_INDEX - 1; gain_index >= 0; gain_index--) {
		if (reg_gain >= GC5035_AGC_Param[gain_index][0]) {
			break;
		}
	}

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xb6, GC5035_AGC_Param[gain_index][1]);
	temp_gain = reg_gain * Dgain_ratio / GC5035_AGC_Param[gain_index][0];
	write_cmos_sensor(0xb1, (temp_gain >> 8) & 0x0f);
	write_cmos_sensor(0xb2, temp_gain & 0xfc);
	LOG_INF("Exit! GC5035_AGC_Param[gain_index][1] = 0x%x, temp_gain = 0x%x, reg_gain = %d\n",
		GC5035_AGC_Param[gain_index][1], temp_gain,reg_gain);

	return reg_gain;
}

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le: 0x%x, se: 0x%x, gain: 0x%x\n", le, se, gain);
}

/*
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);
}
*/
/*************************************************************************
* FUNCTION
*    night_mode
*
* DESCRIPTION
*    This function night mode of sensor.
*
* PARAMETERS
*    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
	/* No Need to implement this function */
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x3e, 0x91);
		mDELAY(10);
	} else {
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x3e, 0x01);
		mDELAY(10);
	}
	return ERROR_NONE;
}

static void sensor_init(void)
{
	LOG_INF("sensor_init:fixel_pixel: %d\n", imgsensor_info.sensor_output_dataformat);
	/* SYSTEM */
	write_cmos_sensor(0xfc, 0x01);
	write_cmos_sensor(0xf4, 0x40);
	write_cmos_sensor(0xf5, 0xc1);
	write_cmos_sensor(0xf6, 0x14);
	write_cmos_sensor(0xf8, 0x49);
	write_cmos_sensor(0xf9, 0x82);
	write_cmos_sensor(0xfa, 0x00);
	write_cmos_sensor(0xfc, 0x81);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x36, 0x01);
	write_cmos_sensor(0xd3, 0x87);
	write_cmos_sensor(0x36, 0x00);
	write_cmos_sensor(0x33, 0x00);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0xe7);
	write_cmos_sensor(0xf7, 0x01);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xee, 0x30);
	write_cmos_sensor(0x87, 0x18);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x90);
	write_cmos_sensor(0xfe, 0x00);

	/* Analog & CISCTL */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x05, 0x02);
	write_cmos_sensor(0x06, 0xda);
	write_cmos_sensor(0x9d, 0x0c);
	write_cmos_sensor(0x09, 0x00);
	write_cmos_sensor(0x0a, 0x04);
	write_cmos_sensor(0x0b, 0x00);
	write_cmos_sensor(0x0c, 0x03);
	write_cmos_sensor(0x0d, 0x07);
	write_cmos_sensor(0x0e, 0xa8);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x30);
	write_cmos_sensor(0x11, 0x02);
	if (imgsensor.mirror == IMAGE_HV_MIRROR)
		write_cmos_sensor(0x17, 0x83);
	else if ( imgsensor.mirror == IMAGE_V_MIRROR)
		write_cmos_sensor(0x17, 0x82);
	else if ( imgsensor.mirror == IMAGE_H_MIRROR)
		write_cmos_sensor(0x17, 0x81);
	else
		write_cmos_sensor(0x17, 0x80);

	//write_cmos_sensor(0x17, GC5035_MIRROR);
	//write_cmos_sensor(0x17, 0x83);
	write_cmos_sensor(0x19, 0x05);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x30, 0x03);
	write_cmos_sensor(0x31, 0x03);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xd9, 0xc0);
	write_cmos_sensor(0x1b, 0x20);
	write_cmos_sensor(0x21, 0x48);
	write_cmos_sensor(0x28, 0x22);
	write_cmos_sensor(0x29, 0x58);
	write_cmos_sensor(0x44, 0x20);
	write_cmos_sensor(0x4b, 0x10);
	write_cmos_sensor(0x4e, 0x1a);
	write_cmos_sensor(0x50, 0x11);
	write_cmos_sensor(0x52, 0x33);
	write_cmos_sensor(0x53, 0x44);
	write_cmos_sensor(0x55, 0x10);
	write_cmos_sensor(0x5b, 0x11);
	write_cmos_sensor(0xc5, 0x02);
	write_cmos_sensor(0x8c, 0x1a);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x33, 0x05);
	write_cmos_sensor(0x32, 0x38);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x91, 0x80);
	write_cmos_sensor(0x92, 0x28);
	write_cmos_sensor(0x93, 0x20);
	write_cmos_sensor(0x95, 0xa0);
	write_cmos_sensor(0x96, 0xe0);
	write_cmos_sensor(0xd5, 0xfc);
	write_cmos_sensor(0x97, 0x28);
	write_cmos_sensor(0x16, 0x0c);
	write_cmos_sensor(0x1a, 0x1a);
	write_cmos_sensor(0x1f, 0x11);
	write_cmos_sensor(0x20, 0x10);
	write_cmos_sensor(0x46, 0xe3);
	write_cmos_sensor(0x4a, 0x04);
	if (imgsensor.mirror == IMAGE_HV_MIRROR || imgsensor.mirror == IMAGE_V_MIRROR)
		write_cmos_sensor(0x54, 0x03);
	else
		write_cmos_sensor(0x54, 0x02);

	//write_cmos_sensor(0x54, GC5035_RSTDUMMY1);
	//write_cmos_sensor(0x54, 0x03);
	write_cmos_sensor(0x62, 0x00);
	write_cmos_sensor(0x72, 0xcf);
	write_cmos_sensor(0x73, 0xc9);
	write_cmos_sensor(0x7a, 0x05);
	write_cmos_sensor(0x7d, 0xcc);
    write_cmos_sensor(0x90, 0x00);
	write_cmos_sensor(0xce, 0x98);
	write_cmos_sensor(0xd0, 0xb2);
	write_cmos_sensor(0xd2, 0x40);
	write_cmos_sensor(0xe6, 0xe0);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x12, 0x01);
	write_cmos_sensor(0x13, 0x01);
	write_cmos_sensor(0x14, 0x01);
	write_cmos_sensor(0x15, 0x02);
	if (imgsensor.mirror == IMAGE_HV_MIRROR || imgsensor.mirror == IMAGE_V_MIRROR)
		write_cmos_sensor(0x22, 0xfc);
	else
		write_cmos_sensor(0x22, 0x7c);
	//write_cmos_sensor(0x22, GC5035_RSTDUMMY2);
	//write_cmos_sensor(0x22, 0xfc);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);

	/* Gain */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xb0, 0x6e);
	write_cmos_sensor(0xb1, 0x01);
	write_cmos_sensor(0xb2, 0x00);
	write_cmos_sensor(0xb3, 0x00);
	write_cmos_sensor(0xb4, 0x00);
	write_cmos_sensor(0xb6, 0x00);

	/* ISP */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x53, 0x00);
	write_cmos_sensor(0x89, 0x03);
	write_cmos_sensor(0x60, 0x40);

	/* BLK */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x42, 0x21);
	write_cmos_sensor(0x49, 0x03);
	write_cmos_sensor(0x4a, 0xff);
	write_cmos_sensor(0x4b, 0xc0);
	write_cmos_sensor(0x55, 0x00);

	/* Anti_blooming */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x41, 0x28);
	write_cmos_sensor(0x4c, 0x00);
	write_cmos_sensor(0x4d, 0x00);
	write_cmos_sensor(0x4e, 0x3c);
	write_cmos_sensor(0x44, 0x08);
	write_cmos_sensor(0x48, 0x01);

	/* Crop */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x91, 0x00);
	write_cmos_sensor(0x92, 0x08);
	write_cmos_sensor(0x93, 0x00);
	write_cmos_sensor(0x94, 0x07);
	write_cmos_sensor(0x95, 0x07);
	write_cmos_sensor(0x96, 0x98);
	write_cmos_sensor(0x97, 0x0a);
	write_cmos_sensor(0x98, 0x20);
	write_cmos_sensor(0x99, 0x00);

	/* MIPI */
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x02, 0x57);
	write_cmos_sensor(0x03, 0xb7);
	write_cmos_sensor(0x15, 0x14);
	write_cmos_sensor(0x18, 0x0f);
	write_cmos_sensor(0x21, 0x22);
	write_cmos_sensor(0x22, 0x06);
	write_cmos_sensor(0x23, 0x48);
	write_cmos_sensor(0x24, 0x12);
	write_cmos_sensor(0x25, 0x28);
	write_cmos_sensor(0x26, 0x08);
	write_cmos_sensor(0x29, 0x06);
	write_cmos_sensor(0x2a, 0x58);
	write_cmos_sensor(0x2b, 0x08);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x10);

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x01);
}

static void preview_setting(void)
{
	LOG_INF("E!\n");
	/* System */
	/*write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x01);*/
	write_cmos_sensor(0xfc, 0x01);
	write_cmos_sensor(0xf4, 0x40);
	write_cmos_sensor(0xf5, 0xc1);
	write_cmos_sensor(0xf6, 0x14);
	write_cmos_sensor(0xf8, 0x49);
	write_cmos_sensor(0xf9, 0x12);
	write_cmos_sensor(0xfa, 0x01);
	write_cmos_sensor(0xfc, 0x81);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x36, 0x01);
	write_cmos_sensor(0xd3, 0x87);
	write_cmos_sensor(0x36, 0x00);
	write_cmos_sensor(0x33, 0x20);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0x87);
	write_cmos_sensor(0xf7, 0x11);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xee, 0x30);
	write_cmos_sensor(0x87, 0x18);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x90);
	write_cmos_sensor(0xfe, 0x00);

	/* Analog & CISCTL */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x05, 0x02);
	write_cmos_sensor(0x06, 0xda);
	write_cmos_sensor(0x9d, 0x0c);
	write_cmos_sensor(0x09, 0x00);
	write_cmos_sensor(0x0a, 0x04);
	write_cmos_sensor(0x0b, 0x00);
	write_cmos_sensor(0x0c, 0x03);
	write_cmos_sensor(0x0d, 0x07);
	write_cmos_sensor(0x0e, 0xa8);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x30);
	write_cmos_sensor(0x21, 0x60);
	write_cmos_sensor(0x29, 0x30);
	write_cmos_sensor(0x44, 0x18);
	write_cmos_sensor(0x4e, 0x20);
	write_cmos_sensor(0x8c, 0x20);
	write_cmos_sensor(0x91, 0x15);
	write_cmos_sensor(0x92, 0x3a);
	write_cmos_sensor(0x93, 0x20);
	write_cmos_sensor(0x95, 0x45);
	write_cmos_sensor(0x96, 0x35);
	write_cmos_sensor(0xd5, 0xf0);
	write_cmos_sensor(0x97, 0x20);
	write_cmos_sensor(0x1f, 0x19);
	write_cmos_sensor(0xce, 0x9b);
	write_cmos_sensor(0xd0, 0xb3);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x14, 0x02);
	write_cmos_sensor(0x15, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);

	/* BLK */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x49, 0x00);
	write_cmos_sensor(0x4a, 0x01);
	write_cmos_sensor(0x4b, 0xf8);

	/* Anti_blooming */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x4e, 0x06);
	write_cmos_sensor(0x44, 0x02);

	/* Crop */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x91, 0x00);
	write_cmos_sensor(0x92, 0x04);
	write_cmos_sensor(0x93, 0x00);
	write_cmos_sensor(0x94, 0x03);
	write_cmos_sensor(0x95, 0x03);
	write_cmos_sensor(0x96, 0xcc);
	write_cmos_sensor(0x97, 0x05);
	write_cmos_sensor(0x98, 0x10);
	write_cmos_sensor(0x99, 0x00);

	/* MIPI */
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x02, 0x58);
	write_cmos_sensor(0x22, 0x03);
	write_cmos_sensor(0x26, 0x06);
	write_cmos_sensor(0x29, 0x03);
	write_cmos_sensor(0x2b, 0x06);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x10);
	/*write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x91);*/
}

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps: %d\n", currefps);
	/* System */
	/*write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x01);*/
	write_cmos_sensor(0xfc, 0x01);
	write_cmos_sensor(0xf4, 0x40);
	write_cmos_sensor(0xf5, 0xc1);
	write_cmos_sensor(0xf6, 0x14);
	if (currefps == 240) { /* PIP */
		write_cmos_sensor(0xf8, 0x3b);
	} else {
		write_cmos_sensor(0xf8, 0x49);
	}
	write_cmos_sensor(0xf9, 0x82);
	write_cmos_sensor(0xfa, 0x00);
	write_cmos_sensor(0xfc, 0x81);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x36, 0x01);
	write_cmos_sensor(0xd3, 0x87);
	write_cmos_sensor(0x36, 0x00);
	write_cmos_sensor(0x33, 0x00);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0xe7);
	write_cmos_sensor(0xf7, 0x01);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xee, 0x30);
	write_cmos_sensor(0x87, 0x18);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x90);
	write_cmos_sensor(0xfe, 0x00);
	/* Analog & CISCTL */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x05, 0x02);
	write_cmos_sensor(0x06, 0xda);
	write_cmos_sensor(0x9d, 0x0c);
	write_cmos_sensor(0x09, 0x00);
	write_cmos_sensor(0x0a, 0x04);
	write_cmos_sensor(0x0b, 0x00);
	write_cmos_sensor(0x0c, 0x03);
	write_cmos_sensor(0x0d, 0x07);
	write_cmos_sensor(0x0e, 0xa8);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x30);
	write_cmos_sensor(0x21, 0x48);
	write_cmos_sensor(0x29, 0x58);
	write_cmos_sensor(0x44, 0x20);
	write_cmos_sensor(0x4e, 0x1a);
	write_cmos_sensor(0x8c, 0x1a);
	write_cmos_sensor(0x91, 0x80);
	write_cmos_sensor(0x92, 0x28);
	write_cmos_sensor(0x93, 0x20);
	write_cmos_sensor(0x95, 0xa0);
	write_cmos_sensor(0x96, 0xe0);
	write_cmos_sensor(0xd5, 0xfc);
	write_cmos_sensor(0x97, 0x28);
	write_cmos_sensor(0x1f, 0x11);//51
	write_cmos_sensor(0xce, 0x98);
	write_cmos_sensor(0xd0, 0xb2);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x14, 0x01);
	write_cmos_sensor(0x15, 0x02);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);

	/* BLK */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x49, 0x03);
	write_cmos_sensor(0x4a, 0xff);
	write_cmos_sensor(0x4b, 0xc0);

	/* Anti_blooming */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x4e, 0x3c);
	write_cmos_sensor(0x44, 0x08);

    /* Crop */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x91, 0x00);
	write_cmos_sensor(0x92, 0x08);
	write_cmos_sensor(0x93, 0x00);
	write_cmos_sensor(0x94, 0x07);
	write_cmos_sensor(0x95, 0x07);
	write_cmos_sensor(0x96, 0x98);
	write_cmos_sensor(0x97, 0x0a);
	write_cmos_sensor(0x98, 0x20);
	write_cmos_sensor(0x99, 0x00);

	/* MIPI */
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x02, 0x57);
	write_cmos_sensor(0x22, 0x06);
	write_cmos_sensor(0x26, 0x08);
	write_cmos_sensor(0x29, 0x06);
	write_cmos_sensor(0x2b, 0x08);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x10);
	/*write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x91);*/
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps: %d\n", currefps);
	/* System */
	/*write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x01);*/
	write_cmos_sensor(0xfc, 0x01);
	write_cmos_sensor(0xf4, 0x40);
	write_cmos_sensor(0xf5, 0xc1);
	write_cmos_sensor(0xf6, 0x14);
	write_cmos_sensor(0xf8, 0x49);
	write_cmos_sensor(0xf9, 0x12);
	write_cmos_sensor(0xfa, 0x01);
	write_cmos_sensor(0xfc, 0x81);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x36, 0x01);
	write_cmos_sensor(0xd3, 0x87);
	write_cmos_sensor(0x36, 0x00);
	write_cmos_sensor(0x33, 0x20);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0x87);
	write_cmos_sensor(0xf7, 0x11);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xee, 0x30);
	write_cmos_sensor(0x87, 0x18);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x90);
	write_cmos_sensor(0xfe, 0x00);

	/* Analog & CISCTL */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x05, 0x02);
	write_cmos_sensor(0x06, 0xda);
	write_cmos_sensor(0x9d, 0x0c);
	write_cmos_sensor(0x09, 0x00);
	write_cmos_sensor(0x0a, 0x04);
	write_cmos_sensor(0x0b, 0x00);
	write_cmos_sensor(0x0c, 0x03);
	write_cmos_sensor(0x0d, 0x07);
	write_cmos_sensor(0x0e, 0xa8);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x30);
	write_cmos_sensor(0x21, 0x60);
	write_cmos_sensor(0x29, 0x30);
	write_cmos_sensor(0x44, 0x18);
	write_cmos_sensor(0x4e, 0x20);
	write_cmos_sensor(0x8c, 0x20);
	write_cmos_sensor(0x91, 0x15);
	write_cmos_sensor(0x92, 0x3a);
	write_cmos_sensor(0x93, 0x20);
	write_cmos_sensor(0x95, 0x45);
	write_cmos_sensor(0x96, 0x35);
	write_cmos_sensor(0xd5, 0xf0);
	write_cmos_sensor(0x97, 0x20);
	write_cmos_sensor(0x1f, 0x19);
	write_cmos_sensor(0xce, 0x9b);
	write_cmos_sensor(0xd0, 0xb3);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x14, 0x02);
	write_cmos_sensor(0x15, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);

	/* BLK */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x49, 0x00);
	write_cmos_sensor(0x4a, 0x01);
	write_cmos_sensor(0x4b, 0xf8);

	/* Anti_blooming */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x4e, 0x06);
	write_cmos_sensor(0x44, 0x02);

	/* Crop */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x91, 0x00);
	write_cmos_sensor(0x92, 0x04);
	write_cmos_sensor(0x93, 0x00);
	write_cmos_sensor(0x94, 0x03);
	write_cmos_sensor(0x95, 0x03);
	write_cmos_sensor(0x96, 0xcc);
	write_cmos_sensor(0x97, 0x05);
	write_cmos_sensor(0x98, 0x10);
	write_cmos_sensor(0x99, 0x00);

	/* MIPI */
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x02, 0x58);
	write_cmos_sensor(0x22, 0x03);
	write_cmos_sensor(0x26, 0x06);
	write_cmos_sensor(0x29, 0x03);
	write_cmos_sensor(0x2b, 0x06);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x10);

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x91);
	/*write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x91);*/
}

static void hs_video_setting(void)
{
	LOG_INF("E\n");
	/* System */
	/*write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x01);*/
	write_cmos_sensor(0xfc, 0x01);
	write_cmos_sensor(0xf4, 0x40);
	write_cmos_sensor(0xf5, 0xc1);
	write_cmos_sensor(0xf6, 0x14);
	write_cmos_sensor(0xf8, 0x49);
	write_cmos_sensor(0xf9, 0x82);
	write_cmos_sensor(0xfa, 0x00);
	write_cmos_sensor(0xfc, 0x81);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x36, 0x01);
	write_cmos_sensor(0xd3, 0x87);
	write_cmos_sensor(0x36, 0x00);
	write_cmos_sensor(0x33, 0x20);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0x87);
	write_cmos_sensor(0xf7, 0x11);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xee, 0x30);
	write_cmos_sensor(0x87, 0x18);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x90);
	write_cmos_sensor(0xfe, 0x00);

	/* Analog & CISCTL */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x05, 0x02);
	write_cmos_sensor(0x06, 0xce);
	write_cmos_sensor(0x9d, 0x0c);
	write_cmos_sensor(0x09, 0x01);
	write_cmos_sensor(0x0a, 0xf0);
	write_cmos_sensor(0x0b, 0x00);
	write_cmos_sensor(0x0c, 0x03);
	write_cmos_sensor(0x0d, 0x03);
	write_cmos_sensor(0x0e, 0xd0);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x30);
	write_cmos_sensor(0x21, 0x50);
	write_cmos_sensor(0x29, 0x50);
	write_cmos_sensor(0x44, 0x18);
	write_cmos_sensor(0x4e, 0x18);
	write_cmos_sensor(0x8c, 0x18);
	write_cmos_sensor(0x91, 0x15);
	write_cmos_sensor(0x92, 0x3a);
	write_cmos_sensor(0x93, 0x20);
	write_cmos_sensor(0x95, 0x45);
	write_cmos_sensor(0x96, 0x35);
	write_cmos_sensor(0xd5, 0xf0);
	write_cmos_sensor(0x97, 0x20);
	write_cmos_sensor(0x1f, 0x19);
	write_cmos_sensor(0xce, 0x98);
	write_cmos_sensor(0xd0, 0xb3);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x14, 0x02);
	write_cmos_sensor(0x15, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);

	/* BLK */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x49, 0x00);
	write_cmos_sensor(0x4a, 0x01);
	write_cmos_sensor(0x4b, 0xf8);

	/* Anti_blooming */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x4e, 0x06);
	write_cmos_sensor(0x44, 0x02);

	/* Crop */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x91, 0x00);
	write_cmos_sensor(0x92, 0x04);
	write_cmos_sensor(0x93, 0x01);
	write_cmos_sensor(0x94, 0x4b);
	write_cmos_sensor(0x95, 0x01);
	write_cmos_sensor(0x96, 0xe0);
	write_cmos_sensor(0x97, 0x02);
	write_cmos_sensor(0x98, 0x80);
	write_cmos_sensor(0x99, 0x00);

	/* MIPI */
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x02, 0x58);
	write_cmos_sensor(0x22, 0x03);
	write_cmos_sensor(0x26, 0x06);
	write_cmos_sensor(0x29, 0x03);
	write_cmos_sensor(0x2b, 0x06);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x10);
	/*write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x91);*/
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	/* System */
	/*write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x01);*/
	write_cmos_sensor(0xfc, 0x01);
	write_cmos_sensor(0xf4, 0x40);
	write_cmos_sensor(0xf5, 0xc1);
	write_cmos_sensor(0xf6, 0x14);
	write_cmos_sensor(0xf8, 0x49);
	write_cmos_sensor(0xf9, 0x12);
	write_cmos_sensor(0xfa, 0x01);
	write_cmos_sensor(0xfc, 0x81);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x36, 0x01);
	write_cmos_sensor(0xd3, 0x87);
	write_cmos_sensor(0x36, 0x00);
	write_cmos_sensor(0x33, 0x20);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0x87);
	write_cmos_sensor(0xf7, 0x11);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xee, 0x30);
	write_cmos_sensor(0x87, 0x18);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x90);
	write_cmos_sensor(0xfe, 0x00);

	/* Analog & CISCTL */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x05, 0x02);
	write_cmos_sensor(0x06, 0xda);
	write_cmos_sensor(0x9d, 0x0c);
	write_cmos_sensor(0x09, 0x00);
	write_cmos_sensor(0x0a, 0x04);
	write_cmos_sensor(0x0b, 0x00);
	write_cmos_sensor(0x0c, 0x03);
	write_cmos_sensor(0x0d, 0x07);
	write_cmos_sensor(0x0e, 0xa8);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x30);
	write_cmos_sensor(0x21, 0x60);
	write_cmos_sensor(0x29, 0x30);
	write_cmos_sensor(0x44, 0x18);
	write_cmos_sensor(0x4e, 0x20);
	write_cmos_sensor(0x8c, 0x20);
	write_cmos_sensor(0x91, 0x15);
	write_cmos_sensor(0x92, 0x3a);
	write_cmos_sensor(0x93, 0x20);
	write_cmos_sensor(0x95, 0x45);
	write_cmos_sensor(0x96, 0x35);
	write_cmos_sensor(0xd5, 0xf0);
	write_cmos_sensor(0x97, 0x20);
	write_cmos_sensor(0x1f, 0x19);
	write_cmos_sensor(0xce, 0x9b);
	write_cmos_sensor(0xd0, 0xb3);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x14, 0x02);
	write_cmos_sensor(0x15, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);

	/* BLK */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x49, 0x00);
	write_cmos_sensor(0x4a, 0x01);
	write_cmos_sensor(0x4b, 0xf8);

	/* Anti_blooming */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x4e, 0x06);
	write_cmos_sensor(0x44, 0x02);

	/* Crop */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x91, 0x00);
	write_cmos_sensor(0x92, 0x0a);
	write_cmos_sensor(0x93, 0x00);
	write_cmos_sensor(0x94, 0x0b);
	write_cmos_sensor(0x95, 0x02);
	write_cmos_sensor(0x96, 0xd0);
	write_cmos_sensor(0x97, 0x05);
	write_cmos_sensor(0x98, 0x00);
	write_cmos_sensor(0x99, 0x00);

	/* MIPI */
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x02, 0x58);
	write_cmos_sensor(0x22, 0x03);
	write_cmos_sensor(0x26, 0x06);
	write_cmos_sensor(0x29, 0x03);
	write_cmos_sensor(0x2b, 0x06);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x10);
	/*write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x91);*/
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	write_cmos_sensor(0xfe, 0x01);
	if (enable) {
		write_cmos_sensor(0x8c, 0x11);
	} else {
		write_cmos_sensor(0x8c, 0x10);
	}
	write_cmos_sensor(0xfe, 0x00);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*    get_imgsensor_id
*
* DESCRIPTION
*    This function get the sensor ID
*
* PARAMETERS
*    *sensorID : return the sensor ID
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 data_index = 0;
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
				#ifdef OPLUS_FEATURE_CAMERA_COMMON
				if (is_project(18073) || is_project(18593)) {
					LOG_INF("18073 18593: 5035 set default mirrorflip\n");
					imgsensor.mirror = IMAGE_HV_MIRROR;
					imgsensor_info.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B;
				}
				#endif
				data_index = 0;
				read_eepromData(&EepData[data_index],
								GC5035_STEREO_START_ADDR,
								CALI_DATA_SLAVE_LENGTH_8ALIGN, SLAVEADDR);
				data_index += CALI_DATA_SLAVE_LENGTH_8ALIGN;
				/*Readout SN data*/
				read_eepromData(&EepData[data_index],
								0xB0, CAMERA_MODULE_SN_LENGTH, SLAVEADDR);
				data_index += CAMERA_MODULE_SN_LENGTH;
				read_eepromData(&EepData[data_index], 0x0, 2, SLAVEADDR);
				data_index += 2;
				read_eepromData(&EepData[data_index], 0x6, 6, SLAVEADDR);
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}

	if (*sensor_id != imgsensor_info.sensor_id) {
		/*if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF*/
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*    open
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	LOG_1;

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
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id) {
			break;
		}
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id) {
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	/* initail sequence write in  */
	sensor_init();

	gc5035_otp_function();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
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
}

/*************************************************************************
* FUNCTION
*    close
*
* DESCRIPTION
*
*
* PARAMETERS
*    None
*
* RETURNS
*    None
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
}

/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*    This function start the sensor preview.
*
* PARAMETERS
*    *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*    None
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

	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*    capture
*
* DESCRIPTION
*    This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*    None
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
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate) {
			LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				imgsensor.current_fps, imgsensor_info.cap.max_framerate / 10);
		}
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	return ERROR_NONE;
}

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
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	normal_video_setting(imgsensor.current_fps);
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

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
	hs_video_setting();
	return ERROR_NONE;
}

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

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
}

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("custom1 E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	return ERROR_NONE;
}

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

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;
	return ERROR_NONE;
}

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	/*sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10;*/ /*not use*/
	/*sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10;*/    /*not use*/
	/*imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate;*/     /*not use*/

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;                  /*not use*/
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;                         /*inverse with datasheet*/
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;                                           /*not use*/
	sensor_info->SensorResetActiveHigh = FALSE;                                           /*not use*/
	sensor_info->SensorResetDelayCount = 5;                                               /*not use*/

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

	sensor_info->SensorMasterClockSwitch = 0;                                             /*not use*/
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	/*The frame of setting shutter default 0 for TG int*/
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	/*The frame of setting sensor gain*/
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;                                               /*not use*/
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;                                             /*not use*/
	sensor_info->SensorPixelClockCount = 3;                                               /*not use*/
	sensor_info->SensorDataLatchCount = 2;                                                /*not use*/

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;                                                 /*0 is default 1x*/
	sensor_info->SensorHightSampling = 0;                                                 /*0 is default 1x*/
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
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}

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
		custom1(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}

static kal_uint32 set_video_mode(UINT16 framerate)
{
	/* This Function not used after ROME */
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0) /* Dynamic frame rate */
		return ERROR_NONE;
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
	if (enable) {/* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	} else {        /* Cancel Auto flick */
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
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0) {
			return ERROR_NONE;
		}
		frame_length =
			imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
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
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate) {
			LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				framerate, imgsensor_info.cap.max_framerate / 10);
		}
		frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
			(frame_length - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
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
	default:  /*coding with  preview scenario by default*/
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

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
	UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *)feature_para;
	UINT16 *feature_data_16 = (UINT16 *)feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *)feature_para;
	UINT32 *feature_data_32 = (UINT32 *)feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	/* unsigned long long *feature_return_para=(unsigned long long *) feature_para; */

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *)feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
    #ifdef OPLUS_FEATURE_CAMERA_COMMON
	case SENSOR_FEATURE_GET_EEPROM_DATA:
	{
		kal_uint16 data_lens = CALI_DATA_SLAVE_LENGTH_8ALIGN
					+ CAMERA_MODULE_SN_LENGTH
					+ CAMERA_MODULE_INFO_LENGTH;
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
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.pre.framelength << 16)
                                 + imgsensor_info.pre.linelength;
                break;
        }
        break;
	#endif
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
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
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16)*feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data), (UINT16) (*(feature_data + 1)),
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
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
		LOG_INF("adb_i2c_read 0x%x = 0x%x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
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
		set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16 + 1));
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
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: /*for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", (UINT32)*feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (BOOL)*feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32)*feature_data);
		wininfo =
			(struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[5],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data + 1),
				(UINT16)*(feature_data + 2));
		ihdr_write_shutter_gain
			((UINT16)*feature_data, (UINT16)*(feature_data + 1),
				(UINT16)*(feature_data + 2));
		break;
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
UINT32 GC5035_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
