// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
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

#include "hi556mipiraw_Sensor.h"
#include "imgsensor_common.h"
#include "imgsensor_hwcfg_custom.h"
#define PFX "hi556_camera_sensor"
#define LOG_INF(format, args...)    \
	pr_info(PFX "[%s] " format, __func__, ##args)

#define MULTI_WRITE 1
static DEFINE_SPINLOCK(imgsensor_drv_lock);

#define per_frame 1

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = HI556_SENSOR_ID_BLADE,
	.checksum_value = 0x55e2a82f,
	.pre = {
		.pclk = 170300000,
		.linelength = 2816,
		.framelength = 2010,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,

		.mipi_data_lp2hs_settle_dc = 14,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
		.mipi_pixel_rate = 170300000,
	},
	.cap = {
		.pclk = 170300000,
		.linelength = 2816,
		.framelength = 2010,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 300,
		.mipi_pixel_rate = 170300000,
	},
	.cap1 = {
		.pclk = 170300000,
		.linelength = 2816,
		.framelength = 4020,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 150,
		.mipi_pixel_rate = 170300000,
	},
	.normal_video = {
		.pclk = 170300000,
		.linelength = 2816,
		.framelength = 2010,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1458,
/* MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 14,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
		.mipi_pixel_rate = 170300000,
	},
	.hs_video = {
		.pclk = 170300000,
		.linelength = 2816,
		.framelength = 520,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 640,
		.grabwindow_height = 480,
		.mipi_data_lp2hs_settle_dc = 14,//unit , ns
		.max_framerate = 1200,
		.mipi_pixel_rate = 42575000,
	},
	.slim_video = {
		.pclk = 170300000,
		.linelength = 2816,
		.framelength = 2010,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 14,//unit , ns
		.max_framerate = 300,
		.mipi_pixel_rate = 85150000,
	},

	.margin = 2,
	.min_shutter = 2,
	.max_frame_length = 0xFFFF,

	.min_gain = 64,
	.max_gain = 1024,
	.min_gain_iso = 100,
	.gain_step = 4,
	.gain_type = 3,
	//.temperature_support=
	//.frame_time_delay_frame=
#if per_frame
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
#else
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 1,
	.ae_ispGain_delay_frame = 2,
#endif

	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,      //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num

	.cap_delay_frame = 3,
	.pre_delay_frame = 3,
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,

	//.cap_delay_frame = 2,
	//.pre_delay_frame = 2,
	//.video_delay_frame = 2,
	//enter high speed video  delay frame num
	//.hs_video_delay_frame = 2,
	//enter slim video delay frame num
	//.slim_video_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = 1,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 26,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x40, 0x51, 0x50, 0xff},
	.i2c_speed = 400,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x0100,
	.gain = 0xe0,
	.dummy_pixel = 0,
	.dummy_line = 0,
//full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	.i2c_write_id = 0x40,
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	// preview
{ 2592, 1944,    0,    0, 2592, 1944, 2592, 1944,
	 0,    0, 2592, 1944,    0,    0, 2592, 1944},
	// capture
{ 2592, 1944,    0,    0, 2592, 1944, 2592, 1944,
	 0,    0, 2592, 1944,    0,    0, 2592, 1944},
	// video
{ 2592, 1944,    0,    241, 2592, 1462, 2592, 1462,
	 0,    0, 2592, 1462,    0,    2, 2592, 1458},
	// high speed video
{ 2592, 1944,   16,   12, 2560, 1920, 640, 480,
	 0,   0,  640,  480,    0,    0,  640,  480},
	// slim video
{ 2592, 1944,   16,  252, 2560, 1440, 1280, 720,
	 0,    0, 1280,  720,    0,    0, 1280,  720},
};

#if MULTI_WRITE
#define I2C_BUFFER_LEN 765

static kal_uint16 hi556_table_write_cmos_sensor(
					kal_uint16 *para, kal_uint32 len)
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

		if ((I2C_BUFFER_LEN - tosend) < 4 ||
			len == IDX ||
			addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend,
				imgsensor.i2c_write_id,
				4, imgsensor_info.i2c_speed);

			tosend = 0;
		}
	}
	return 0;
}
#endif

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id);
}

static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

//+bug584789 chenbocheng.wt, add, 2020/10/30, add main camera s5kgm1 and wide camera hi556 otp code
//#define HI556_OTP_DUMP

#ifdef HI556_OTP_DUMP
void dumpEEPROMData1(int startAdd, int u4Length,u8* pu1Params)
{
	int i = 0;
	for(i = 0; i < u4Length; i += 16){
		if(u4Length - i  >= 16){
			LOG_INF("eeprom[0x%04x 0x%04x]:0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x ",
			startAdd + i,startAdd + i+15,pu1Params[i],pu1Params[i+1],pu1Params[i+2],pu1Params[i+3],pu1Params[i+4],pu1Params[i+5],pu1Params[i+6]
			,pu1Params[i+7],pu1Params[i+8],pu1Params[i+9],pu1Params[i+10],pu1Params[i+11],pu1Params[i+12],pu1Params[i+13],pu1Params[i+14]
			,pu1Params[i+15]);
		}else{
			int j = i;
			for(;j < u4Length;j++)
			LOG_INF("eeprom[0x%04x] = 0x%02x ",startAdd + j,pu1Params[j]);
		}
	}
	LOG_INF("\n");
}

#endif

#define MODULE_INFO_SIZE 15
#define AWB_DATA_SIZE 16
#define LSC_DATA_SIZE 1868
#define SN_DATA_SIZE 18
#define SN_DATA_SIZE_HI556W 24

//+bug604664, liuxingyu, add, 2020/12/18, add for hi556 eeprom bring up
#define OTP_SET_ADDR_H 0x010a
#define OTP_SET_ADDR_L 0x010b
#define OTP_RW_FLAG 0x0102
#define OTP_READ_FLAG 0x0108

#define MAX_GROUP_NUM 3
static int module_flag_addr_tbl[MAX_GROUP_NUM] = {0x0410, 0x0C58, 0x14A0};
static int module_data_addr_tbl[MAX_GROUP_NUM] = {0x0401, 0x0C49, 0x1491};

static int awb_flag_addr_tbl[MAX_GROUP_NUM] = {0x0431, 0x0C79, 0x14C1};
static int awb_data_addr_tbl[MAX_GROUP_NUM] = {0x0421, 0x0C69, 0x14B1};

static int lsc_flag_addr_tbl[MAX_GROUP_NUM] = {0x0B7F, 0x13C8, 0x1C10};
static int lsc_data_addr_tbl[MAX_GROUP_NUM] = {0x0433, 0x0C7B, 0x14C3};

static int SN_flag_addr_tbl[MAX_GROUP_NUM] = {0x0C47, 0x148F, 0x1CD7};
static int SN_data_addr_tbl[MAX_GROUP_NUM] = {0x0C35, 0x147D, 0x1CC5};

/*Add for hi556W*/
static int module_flag_addr_tbl_hi556w[MAX_GROUP_NUM] = {0x0410, 0x0C5E, 0x14AC}; // 0x0410 0x0C5E 0x14AC
static int module_data_addr_tbl_hi556w[MAX_GROUP_NUM] = {0x0401, 0x0C4F, 0x149D}; // 0x0401 0x0C4F 0x149D

static int awb_flag_addr_tbl_hi556w[MAX_GROUP_NUM] = {0x0431, 0x0C7F, 0x14CD}; // 0x0431 0x0C7F 0x14CD
static int awb_data_addr_tbl_hi556w[MAX_GROUP_NUM] = {0x0421, 0x0C6F, 0x14BD}; // 0x0421 0x0C6F 0x14BD

static int lsc_flag_addr_tbl_hi556w[MAX_GROUP_NUM] = {0x0B7F, 0x13CD, 0x1C1B}; // 0x0B7F 0x13CD 0x1C1B
static int lsc_data_addr_tbl_hi556w[MAX_GROUP_NUM] = {0x0433, 0x0C81, 0x14CF}; // 0x0433 0X0C81 0x14CF

static int SN_flag_addr_tbl_hi556w[MAX_GROUP_NUM] = {0x0C4D, 0x149B, 0x1CE9}; // 0x0C4D 0x149B 0x1CE9
static int SN_data_addr_tbl_hi556w[MAX_GROUP_NUM] = {0x0C35, 0x1483, 0x1CD1}; // 0x0C35 0x1483 0x1CD1

static int Sensor_id_hi556W[MAX_GROUP_NUM] = {0x0407,0x0C55,0x14A3}; // 0x0407 0x0C55 0x14A3

//-bug604664, liuxingyu, add, 2020/12/18, add for hi556 eeprom bring up

#define VALID_OTP_FLAG 0x55
//#define OTP_GROUP_OFFSET 0x76A

unsigned char hi556_data_lsc[LSC_DATA_SIZE + 2] = {0};/*Add flag and check sum*/
unsigned char hi556_data_awb[AWB_DATA_SIZE + 2] = {0};/*Add  flag andcheck sum*/
unsigned char hi556_data_info[MODULE_INFO_SIZE + 2] = {0};/*Add  flag and check sum*/
unsigned char hi556_data_SN[SN_DATA_SIZE + 2] = {0};
/*Add for Hi-556w*/
unsigned char hi556_data_SN_HI556W[SN_DATA_SIZE_HI556W + 2] = {0};
unsigned char hi556_module_id = 0;
unsigned char hi556_lsc_valid = 0;
unsigned char hi556_awb_valid = 0;
unsigned char hi556w_flag = 0;
unsigned char sn_length = 0;

static void hi556_disable_otp_func(void)
{
	write_cmos_sensor(0x0a00, 0x00);
	mdelay(10);
	write_cmos_sensor(0x003e, 0x00);
	write_cmos_sensor(0x0a00, 0x00);
}

static void check_is_hi556w(void)
{
	unsigned char i = 0;
	unsigned char hi_556w_sensorid = 0;
	for(i = 0; i < MAX_GROUP_NUM; i++){
		write_cmos_sensor_8(OTP_SET_ADDR_H, (Sensor_id_hi556W[i]>>8)&0xff);
		write_cmos_sensor_8(OTP_SET_ADDR_L, (Sensor_id_hi556W[i])&0xff);
		write_cmos_sensor_8(OTP_RW_FLAG, 0x01);
		hi_556w_sensorid = read_cmos_sensor(OTP_READ_FLAG);
		if (hi_556w_sensorid == 0x01) {
			hi556w_flag = 1;
			LOG_INF("=== THIS IS HI556W! sensor id : 0x%x, group : %d ===\n",hi_556w_sensorid,i);
			break;
		}
	}
}

static int read_hi556_module_info(void)
{
	int otp_grp_flag = 0, minfo_start_addr = 0;
	int year = 0, month = 0, day = 0;
	int sensor_id = 0,lens_id = 0,vcm_id = 0,vcm_id2 = 0,mirror_flip= 0 ;
	int check_sum = 0, check_sum_cal = 0;
	int i;

	if(hi556w_flag == 1){
		memcpy(&module_flag_addr_tbl[0],&module_flag_addr_tbl_hi556w[0],sizeof(module_flag_addr_tbl_hi556w));
		memcpy(&module_data_addr_tbl[0],&module_data_addr_tbl_hi556w[0],sizeof(module_data_addr_tbl_hi556w));
		LOG_INF("hi_556w module info memcpy, flag: 0x%x,0x%x,0x%x;data: 0x%x,0x%x,0x%x\n",
				module_flag_addr_tbl[0],module_flag_addr_tbl[1],module_flag_addr_tbl[2],module_data_addr_tbl[0],module_data_addr_tbl[1],module_data_addr_tbl[2]);
	}
	/* read flag */
	for(i = 0; i < MAX_GROUP_NUM; i++){
		write_cmos_sensor_8(OTP_SET_ADDR_H, (module_flag_addr_tbl[i]>>8)&0xff);
		write_cmos_sensor_8(OTP_SET_ADDR_L, (module_flag_addr_tbl[i])&0xff);
		write_cmos_sensor_8(OTP_RW_FLAG, 0x01);
		otp_grp_flag = read_cmos_sensor(OTP_READ_FLAG);
		if (otp_grp_flag == 0x01) {
			LOG_INF("module info: group %d selected\n", i);
			minfo_start_addr = module_data_addr_tbl[i];
			break;
		}
	}
	if (minfo_start_addr != 0) {

		/* read & checksum */
		write_cmos_sensor_8(OTP_SET_ADDR_H, (minfo_start_addr>>8)&0xff);
		write_cmos_sensor_8(OTP_SET_ADDR_L, minfo_start_addr&0xff);
		write_cmos_sensor_8(OTP_RW_FLAG, 0x01);
		for(i = 0; i < MODULE_INFO_SIZE + 2; i++){  // add flag and checksum
			hi556_data_info[i]=read_cmos_sensor(OTP_READ_FLAG);
		}
		for(i = 0; i < MODULE_INFO_SIZE; i++){
			check_sum_cal += hi556_data_info[i];
		}

		check_sum_cal = check_sum_cal % 255;
		hi556_module_id = (hi556_data_info[1] << 8) || hi556_data_info[0];
		day = hi556_data_info[2];
		month = hi556_data_info[3];
		year = (hi556_data_info[5] << 8) || hi556_data_info[4];
		sensor_id = (hi556_data_info[7] << 8) || hi556_data_info[6];
		lens_id = (hi556_data_info[9] << 8) || hi556_data_info[8];
		vcm_id = (hi556_data_info[11] << 8) || hi556_data_info[10];
		vcm_id2 = (hi556_data_info[13] << 8) || hi556_data_info[12];
		mirror_flip = hi556_data_info[14];
		check_sum = hi556_data_info[MODULE_INFO_SIZE + 1];

	#ifdef HI556_OTP_DUMP
		dumpEEPROMData1(module_data_addr_tbl[i], MODULE_INFO_SIZE, &hi556_data_info[0]);
	#endif
		LOG_INF("=== HI556 INFO module_id=0x%x ===\n", hi556_module_id);
		LOG_INF("=== HI556 INFO date is %d-%d-%d ===\n",year,month,day);
		LOG_INF("=== HI556 INFO sensor_id=0x%x, lens_id=0x%x,vcm_id=0x%x,vcm_id2=0x%x ===\n",sensor_id, lens_id, vcm_id, vcm_id2);
		LOG_INF("=== HI556 INFO mirror_flip=0x%x ===\n", mirror_flip);
		LOG_INF("=== HI556 INFO check_sum=0x%x,check_sum_cal=0x%x ===\n", check_sum, check_sum_cal);
		if(check_sum == check_sum_cal){
			return 1;
		}else{
			return 0;
		}
	} else {
		return 0;
	}
}

static int read_hi556_awb_info(void)
{
	int otp_grp_flag = 0, awb_start_addr=0;
	int check_sum_awb = 0, check_sum_awb_cal = 0;
	int r = 0,b = 0,gr = 0, gb = 0, golden_r = 0, golden_b = 0, golden_gr = 0, golden_gb = 0;
	int i;

	if(hi556w_flag == 1){
		memcpy(&awb_flag_addr_tbl[0],&awb_flag_addr_tbl_hi556w[0],sizeof(awb_flag_addr_tbl_hi556w));
		memcpy(&awb_data_addr_tbl[0],&awb_data_addr_tbl_hi556w[0],sizeof(awb_data_addr_tbl_hi556w));
		LOG_INF("hi_556w awb info memcpy, flag: 0x%x,0x%x,0x%x;data: 0x%x,0x%x,0x%x\n",
				awb_flag_addr_tbl[0],awb_flag_addr_tbl[1],awb_flag_addr_tbl[2],awb_data_addr_tbl[0],awb_data_addr_tbl[1],awb_data_addr_tbl[2]);	
	}
	/* read flag */
	for(i = 0; i < MAX_GROUP_NUM; i++){
		write_cmos_sensor_8(OTP_SET_ADDR_H, (awb_flag_addr_tbl[i]>>8)&0xff);
		write_cmos_sensor_8(OTP_SET_ADDR_L, (awb_flag_addr_tbl[i])&0xff);
		write_cmos_sensor_8(OTP_RW_FLAG, 0x01);
		otp_grp_flag = read_cmos_sensor(OTP_READ_FLAG);
		if (otp_grp_flag == 0x01) {
			LOG_INF("awb: group %d selected\n", i);
			awb_start_addr = awb_data_addr_tbl[i];
			break;
		}
	}

	if (awb_start_addr != 0) {
		/* read & checksum */
		write_cmos_sensor_8(OTP_SET_ADDR_H, ((awb_start_addr)>>8)&0xff);
		write_cmos_sensor_8(OTP_SET_ADDR_L, (awb_start_addr)&0xff);
		write_cmos_sensor_8(OTP_RW_FLAG, 0x01);
		for(i = 0; i < AWB_DATA_SIZE + 2; i++){ // add flag and checksum
			hi556_data_awb[i]=read_cmos_sensor(OTP_READ_FLAG);
		}
		for(i = 0; i < AWB_DATA_SIZE; i++){
			check_sum_awb_cal += hi556_data_awb[i];
		}
		LOG_INF("check_sum_awb_cal =0x%x \n",check_sum_awb_cal);
		r = ((hi556_data_awb[1]<<8)&0xff00)|(hi556_data_awb[0]&0xff);
		b = ((hi556_data_awb[3]<<8)&0xff00)|(hi556_data_awb[2]&0xff);
		gr = ((hi556_data_awb[5]<<8)&0xff00)|(hi556_data_awb[4]&0xff);
		gb = ((hi556_data_awb[7]<<8)&0xff00)|(hi556_data_awb[6]&0xff);
		golden_r = ((hi556_data_awb[9]<<8)&0xff00)|(hi556_data_awb[8]&0xff);
		golden_b = ((hi556_data_awb[11]<<8)&0xff00)|(hi556_data_awb[10]&0xff);
		golden_gr = ((hi556_data_awb[13]<<8)&0xff00)|(hi556_data_awb[12]&0xff);
		golden_gb = ((hi556_data_awb[15]<<8)&0xff00)|(hi556_data_awb[14]&0xff);
		check_sum_awb = hi556_data_awb[AWB_DATA_SIZE + 1];
		check_sum_awb_cal = check_sum_awb_cal % 255;

	#ifdef HI556_OTP_DUMP
		dumpEEPROMData1(awb_start_addr, AWB_DATA_SIZE,&hi556_data_awb[0]);
	#endif
		LOG_INF("=== HI556 AWB r=0x%x, b=0x%x, gr=%x, gb=0x%x ===\n", r, b,gb, gr);
		LOG_INF("=== HI556 AWB gr=0x%x,gb=0x%x,gGr=%x, gGb=0x%x ===\n", golden_r, golden_b, golden_gr, golden_gb);
		LOG_INF("=== HI556 AWB check_sum_awb=0x%x,check_sum_awb_cal=0x%x ===\n",check_sum_awb,check_sum_awb_cal);
		if(check_sum_awb == check_sum_awb_cal){
			return 1;
		}else{
			return 0;
		}
	} else {
		return 0;
	}
}
static int read_hi556_lsc_info(void)
{
	int otp_grp_flag = 0, lsc_start_addr = 0;
	int check_sum_lsc = 0, check_sum_lsc_cal = 0;
	int i;

	if(hi556w_flag == 1){
		memcpy(&lsc_flag_addr_tbl[0],&lsc_flag_addr_tbl_hi556w[0],sizeof(lsc_flag_addr_tbl_hi556w));
		memcpy(&lsc_data_addr_tbl[0],&lsc_data_addr_tbl_hi556w[0],sizeof(lsc_data_addr_tbl_hi556w));
		LOG_INF("hi_556w lsc info memcpy, flag: 0x%x,0x%x,0x%x;data: 0x%x,0x%x,0x%x\n",
				lsc_flag_addr_tbl[0],lsc_flag_addr_tbl[1],lsc_flag_addr_tbl[2],lsc_data_addr_tbl[0],lsc_data_addr_tbl[1],lsc_data_addr_tbl[2]);
	}
	/* read flag */
	for(i = 0; i < MAX_GROUP_NUM; i++){
		write_cmos_sensor_8(OTP_SET_ADDR_H, (lsc_flag_addr_tbl[i]>>8)&0xff);
		write_cmos_sensor_8(OTP_SET_ADDR_L, (lsc_flag_addr_tbl[i])&0xff);
		write_cmos_sensor_8(OTP_RW_FLAG, 0x01);
		otp_grp_flag = read_cmos_sensor(OTP_READ_FLAG);
		if (otp_grp_flag == 0x01) {
			LOG_INF("lsc: group %d selected\n", i);
			lsc_start_addr = lsc_data_addr_tbl[i];
			break;
		}
	}

	if (lsc_start_addr != 0) {
		/* read & checksum */

		write_cmos_sensor_8(OTP_SET_ADDR_H, ((lsc_start_addr)>>8)&0xff);
		write_cmos_sensor_8(OTP_SET_ADDR_L, (lsc_start_addr)&0xff);
		write_cmos_sensor_8(OTP_RW_FLAG, 0x01);
		for(i = 0; i < LSC_DATA_SIZE + 2; i++)  // add flag and checksum
		{
			hi556_data_lsc[i] = read_cmos_sensor(OTP_READ_FLAG);
		}
		for(i = 0; i < LSC_DATA_SIZE; i++){
			check_sum_lsc_cal += hi556_data_lsc[i];
		}
		LOG_INF("check_sum_awb_cal =0x%x \n",check_sum_lsc_cal);
		check_sum_lsc = hi556_data_lsc[LSC_DATA_SIZE + 1];
		check_sum_lsc_cal = check_sum_lsc_cal % 255;

	#ifdef HI556_OTP_DUMP
		dumpEEPROMData1(lsc_start_addr, LSC_DATA_SIZE,&hi556_data_lsc[0]);
	#endif
		LOG_INF("=== HI556 LSC check_sum_lsc=0x%x, check_sum_lsc_cal=0x%x ===\n", check_sum_lsc, check_sum_lsc_cal);
		if(check_sum_lsc == check_sum_lsc_cal){
			return 1;
		}else{
			return 0;
		}
	} else {
		return 0;
	}
}

static int read_hi556_SN_Code(void)
{
    int otp_grp_flag = 0, SN_start_addr = 0;
    int i;

    if(hi556w_flag == 1){
        sn_length = SN_DATA_SIZE_HI556W;
        memcpy(&SN_flag_addr_tbl[0],&SN_flag_addr_tbl_hi556w[0],sizeof(SN_flag_addr_tbl_hi556w));
        memcpy(&SN_data_addr_tbl[0],&SN_data_addr_tbl_hi556w[0],sizeof(SN_data_addr_tbl_hi556w));
        LOG_INF("hi_556w sn info memcpy, sn_length %d, flag: 0x%x,0x%x,0x%x;data: 0x%x,0x%x,0x%x\n",sn_length,
                SN_flag_addr_tbl[0],SN_flag_addr_tbl[1],SN_flag_addr_tbl[2],SN_data_addr_tbl[0],SN_data_addr_tbl[1],SN_data_addr_tbl[2]);
    } else {
        sn_length = SN_DATA_SIZE;
        LOG_INF("HI556 sn_length: %d",sn_length);
    }
    /* read flag */
    for(i = 0; i < MAX_GROUP_NUM; i++){
        write_cmos_sensor_8(OTP_SET_ADDR_H, (SN_flag_addr_tbl[i]>>8)&0xff);
        write_cmos_sensor_8(OTP_SET_ADDR_L, (SN_flag_addr_tbl[i])&0xff);
        write_cmos_sensor_8(OTP_RW_FLAG, 0x01);
        otp_grp_flag = read_cmos_sensor(OTP_READ_FLAG);
        if (otp_grp_flag == 0x01) {
            LOG_INF("SN: group %d selected\n", i);
            SN_start_addr = SN_data_addr_tbl[i];
            break;
        }
    }
    if (SN_start_addr != 0) {
        /* read & checksum */

        write_cmos_sensor_8(OTP_SET_ADDR_H, ((SN_start_addr)>>8)&0xff);
        write_cmos_sensor_8(OTP_SET_ADDR_L, (SN_start_addr)&0xff);
        write_cmos_sensor_8(OTP_RW_FLAG, 0x01);
        Oplusimgsensor_Registdeviceinfo(gImgEepromInfo.pCamModuleInfo[IMGSENSOR_SENSOR_IDX_SUB].name,
                                        gImgEepromInfo.pCamModuleInfo[IMGSENSOR_SENSOR_IDX_SUB].version,
                                        hi556_data_info[0]);
        if (hi556w_flag == 1) {
            for(i = 0; i < sn_length + 2; i++)  // add flag and checksum
            {
                hi556_data_SN_HI556W[i] = read_cmos_sensor(OTP_READ_FLAG);
            }
            #ifdef HI556_OTP_DUMP
            dumpEEPROMData1(SN_start_addr,sn_length,&hi556_data_SN_HI556W[0]);
            #endif
            LOG_INF("=== HI556W SN SN_flag=0x%x ===\n", hi556_data_SN_HI556W[sn_length]);
            if (hi556_data_SN_HI556W[sn_length] == 0x01) {
                memcpy(&(gImgEepromInfo.camNormdata[IMGSENSOR_SENSOR_IDX_SUB][8]), hi556_data_SN_HI556W, sn_length + 2);
                gImgEepromInfo.i4CurSensorIdx = IMGSENSOR_SENSOR_IDX_SUB;
                gImgEepromInfo.i4CurSensorId = imgsensor_info.sensor_id;
                return 1;
            } else {
                LOG_INF("HI556W SN is not invalid.");
                return 0;
            }
        } else {
            for(i = 0; i < sn_length + 2; i++)  // add flag and checksum
            {
                hi556_data_SN[i] = read_cmos_sensor(OTP_READ_FLAG);
            }
            #ifdef HI556_OTP_DUMP
            dumpEEPROMData1(SN_start_addr,sn_length,&hi556_data_SN[0]);
            #endif
            LOG_INF("=== HI556 SN SN_flag=0x%x ===\n", hi556_data_SN[sn_length]);
            if (hi556_data_SN[sn_length] == 0x01) {
                memcpy(&(gImgEepromInfo.camNormdata[IMGSENSOR_SENSOR_IDX_SUB][8]), hi556_data_SN, sn_length + 2);
                gImgEepromInfo.i4CurSensorIdx = IMGSENSOR_SENSOR_IDX_SUB;
                gImgEepromInfo.i4CurSensorId = imgsensor_info.sensor_id;
                return 1;
            } else {
                LOG_INF("HI556 SN is not invalid.");
                return 0;
            }
        }
    } else {
        return 0;
    }
}

static int hi556_sensor_otp_info(void)
{
	int ret = 0;

	LOG_INF("come to %s:%d E!\n", __func__, __LINE__);

	/* 1. sensor init */
	write_cmos_sensor(0x0e00, 0x0102); //tg_pmem_sckpw/sdly
	write_cmos_sensor(0x0e02, 0x0102); //tg_pmem_sckpw/sdly
	write_cmos_sensor(0x0e0c, 0x0100); //tg_pmem_rom_dly
	write_cmos_sensor(0x27fe, 0xe000); // firmware start address-ROM
	write_cmos_sensor(0x0b0e, 0x8600); // BGR enable
	write_cmos_sensor(0x0d04, 0x0100); // STRB(OTP Busy) output enable
	write_cmos_sensor(0x0d02, 0x0707); // STRB(OTP Busy) output drivability
	write_cmos_sensor(0x0f30, 0x6e25); // Analog PLL setting
	write_cmos_sensor(0x0f32, 0x7067); // Analog CLKGEN setting
	write_cmos_sensor(0x0f02, 0x0106); // PLL enable
	write_cmos_sensor(0x0a04, 0x0000); // mipi disable
	write_cmos_sensor(0x0e0a, 0x0001); // TG PMEM CEN anable
	write_cmos_sensor(0x004a, 0x0100); // TG MCU enable
	write_cmos_sensor(0x003e, 0x1000); // ROM OTP Continuous W/R mode enable
	write_cmos_sensor(0x0a00, 0x0100); // Stream ON

	/* 2. init OTP setting*/
	write_cmos_sensor_8(0x0A02, 0x01); //Fast sleep on
	write_cmos_sensor_8(0x0A00, 0x00);//stand by on
	mdelay(10);
	write_cmos_sensor_8(0x0f02, 0x00);//pll disable
	write_cmos_sensor_8(0x011a, 0x01);//CP TRIM_H
	write_cmos_sensor_8(0x011b, 0x09);//IPGM TRIM_H
	write_cmos_sensor_8(0x0d04, 0x01);//Fsync(OTP busy)Output Enable
	write_cmos_sensor_8(0x0d02, 0x07);//Fsync(OTP busy)Output Drivability
	write_cmos_sensor_8(0x003e, 0x10);//OTP r/w mode
	write_cmos_sensor_8(0x0a00, 0x01);//standby off

	/* check sensor*/
	check_is_hi556w();

	/* 3. read eeprom data */
	//minfo && awb group
	ret = read_hi556_module_info();
	if(ret != 1){
		hi556_module_id = 0;
		LOG_INF("=== hi556_data_info invalid ===\n");
	}
	ret = read_hi556_awb_info();
	if(ret != 1){
		hi556_awb_valid = 0;
		LOG_INF("=== hi556_data_awb invalid ===\n");
	}else{
		hi556_awb_valid = 1;
	}
	ret = read_hi556_lsc_info();
	if(ret != 1){
		hi556_lsc_valid = 0;
		LOG_INF("=== hi556_data_lsc invalid ===\n");
	}else{
		hi556_lsc_valid = 1;
	}
	ret = read_hi556_SN_Code();
	/* 4. disable otp function */
	hi556_disable_otp_func();
	if(hi556_module_id == 0 || hi556_lsc_valid == 0 || hi556_awb_valid == 0){
		return 0;
	}else{
		return 1;
	}
}
//-bug584789 chenbocheng.wt, add, 2020/10/30, add main camera s5kgm1 and wide camera hi556 otp code

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor(0x0006, imgsensor.frame_length & 0xFFFF);
	write_cmos_sensor(0x0008, imgsensor.line_length & 0xFFFF);

}	/*	set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	return (((read_cmos_sensor(0x0F16) << 8) | read_cmos_sensor(0x0F17)) + SENSOR_ID_OFFSET_BLADE);
}


static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ?
			frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length -
		imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length -
			imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;

	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static void write_shutter(kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;

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
	shutter = (shutter >
		(imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) :
		shutter;
	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk * 10 /
			(imgsensor.line_length * imgsensor.frame_length);
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else
			write_cmos_sensor(0x0006, imgsensor.frame_length);

	} else{
		// Extend frame length

		// ADD ODIN
		realtime_fps = imgsensor.pclk * 10 /
			(imgsensor.line_length * imgsensor.frame_length);
		if (realtime_fps > 300 && realtime_fps < 320)
			set_max_framerate(300, 0);
		// ADD END
		write_cmos_sensor(0x0006, imgsensor.frame_length);
	}

	// Update Shutter
	write_cmos_sensor_8(0x0073, ((shutter & 0xFF0000) >> 16));
	write_cmos_sensor(0x0074, shutter & 0x00FFFF);
	LOG_INF("shutter =%d, framelength =%d",
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

	LOG_INF("set_shutter");
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */

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

	if (gain < BASEGAIN)
		gain = BASEGAIN;
	else if (gain > 16 * BASEGAIN)
		gain = 16 * BASEGAIN;

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_8(0x0077, reg_gain);

	return gain;

}	/*	set_gain  */

#if 0
static void ihdr_write_shutter_gain(kal_uint16 le,
				kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
	if (imgsensor.ihdr_en) {
		spin_lock(&imgsensor_drv_lock);
		if (le > imgsensor.min_frame_length - imgsensor_info.margin)
			imgsensor.frame_length = le + imgsensor_info.margin;
		else
			imgsensor.frame_length = imgsensor.min_frame_length;
		if (imgsensor.frame_length > imgsensor_info.max_frame_length)
			imgsensor.frame_length =
				imgsensor_info.max_frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (le < imgsensor_info.min_shutter)
			le = imgsensor_info.min_shutter;
		if (se < imgsensor_info.min_shutter)
			se = imgsensor_info.min_shutter;
		// Extend frame length first
		write_cmos_sensor(0x0006, imgsensor.frame_length);
		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		write_cmos_sensor(0x3508, (se << 4) & 0xFF);
		write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3506, (se >> 12) & 0x0F);
		set_gain(gain);
	}
}
#endif


#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d", image_mirror);

	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor(0x0000, 0x0000);
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor(0x0000, 0x0100);

		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor(0x0000, 0x0200);

		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor(0x0000, 0x0300);

		break;
	default:
		LOG_INF("Error image_mirror setting");
		break;
	}

}
#endif
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

#if MULTI_WRITE
kal_uint16 addr_data_pair_init_hi556[] = {
	0x0a00, 0x0000,
	0x0e00, 0x0102,
	0x0e02, 0x0102,
	0x0e0c, 0x0100,
	0x2000, 0x7400,
	0x2002, 0x001c,
	0x2004, 0x0242,
	0x2006, 0x0942,
	0x2008, 0x7007,
	0x200a, 0x0fd9,
	0x200c, 0x0259,
	0x200e, 0x7008,
	0x2010, 0x160e,
	0x2012, 0x0047,
	0x2014, 0x2118,
	0x2016, 0x0041,
	0x2018, 0x00d8,
	0x201a, 0x0145,
	0x201c, 0x0006,
	0x201e, 0x0181,
	0x2020, 0x13cc,
	0x2022, 0x2057,
	0x2024, 0x7001,
	0x2026, 0x0fca,
	0x2028, 0x00cb,
	0x202a, 0x009f,
	0x202c, 0x7002,
	0x202e, 0x13cc,
	0x2030, 0x019b,
	0x2032, 0x014d,
	0x2034, 0x2987,
	0x2036, 0x2766,
	0x2038, 0x0020,
	0x203a, 0x2060,
	0x203c, 0x0e5d,
	0x203e, 0x181d,
	0x2040, 0x2066,
	0x2042, 0x20c4,
	0x2044, 0x5000,
	0x2046, 0x0005,
	0x2048, 0x0000,
	0x204a, 0x01db,
	0x204c, 0x025a,
	0x204e, 0x00c0,
	0x2050, 0x0005,
	0x2052, 0x0006,
	0x2054, 0x0ad9,
	0x2056, 0x0259,
	0x2058, 0x0618,
	0x205a, 0x0258,
	0x205c, 0x2266,
	0x205e, 0x20c8,
	0x2060, 0x2060,
	0x2062, 0x707b,
	0x2064, 0x0fdd,
	0x2066, 0x81b8,
	0x2068, 0x5040,
	0x206a, 0x0020,
	0x206c, 0x5060,
	0x206e, 0x3143,
	0x2070, 0x5081,
	0x2072, 0x025c,
	0x2074, 0x7800,
	0x2076, 0x7400,
	0x2078, 0x001c,
	0x207a, 0x0242,
	0x207c, 0x0942,
	0x207e, 0x0bd9,
	0x2080, 0x0259,
	0x2082, 0x7008,
	0x2084, 0x160e,
	0x2086, 0x0047,
	0x2088, 0x2118,
	0x208a, 0x0041,
	0x208c, 0x00d8,
	0x208e, 0x0145,
	0x2090, 0x0006,
	0x2092, 0x0181,
	0x2094, 0x13cc,
	0x2096, 0x2057,
	0x2098, 0x7001,
	0x209a, 0x0fca,
	0x209c, 0x00cb,
	0x209e, 0x009f,
	0x20a0, 0x7002,
	0x20a2, 0x13cc,
	0x20a4, 0x019b,
	0x20a6, 0x014d,
	0x20a8, 0x2987,
	0x20aa, 0x2766,
	0x20ac, 0x0020,
	0x20ae, 0x2060,
	0x20b0, 0x0e5d,
	0x20b2, 0x181d,
	0x20b4, 0x2066,
	0x20b6, 0x20c4,
	0x20b8, 0x50a0,
	0x20ba, 0x0005,
	0x20bc, 0x0000,
	0x20be, 0x01db,
	0x20c0, 0x025a,
	0x20c2, 0x00c0,
	0x20c4, 0x0005,
	0x20c6, 0x0006,
	0x20c8, 0x0ad9,
	0x20ca, 0x0259,
	0x20cc, 0x0618,
	0x20ce, 0x0258,
	0x20d0, 0x2266,
	0x20d2, 0x20c8,
	0x20d4, 0x2060,
	0x20d6, 0x707b,
	0x20d8, 0x0fdd,
	0x20da, 0x86b8,
	0x20dc, 0x50e0,
	0x20de, 0x0020,
	0x20e0, 0x5100,
	0x20e2, 0x3143,
	0x20e4, 0x5121,
	0x20e6, 0x7800,
	0x20e8, 0x3140,
	0x20ea, 0x01c4,
	0x20ec, 0x01c1,
	0x20ee, 0x01c0,
	0x20f0, 0x01c4,
	0x20f2, 0x2700,
	0x20f4, 0x3d40,
	0x20f6, 0x7800,
	0x20f8, 0xffff,
	0x27fe, 0xe000,
	0x3000, 0x60f8,
	0x3002, 0x187f,
	0x3004, 0x7060,
	0x3006, 0x0114,
	0x3008, 0x60b0,
	0x300a, 0x1473,
	0x300c, 0x0013,
	0x300e, 0x140f,
	0x3010, 0x0040,
	0x3012, 0x100f,
	0x3014, 0x60f8,
	0x3016, 0x187f,
	0x3018, 0x7060,
	0x301a, 0x0114,
	0x301c, 0x60b0,
	0x301e, 0x1473,
	0x3020, 0x0013,
	0x3022, 0x140f,
	0x3024, 0x0040,
	0x3026, 0x000f,
	0x0b00, 0x0000,
	0x0b02, 0x0045,
	0x0b04, 0xb405,
	0x0b06, 0xc403,
	0x0b08, 0x0081,
	0x0b0a, 0x8252,
	0x0b0c, 0xf814,
	0x0b0e, 0xc618,
	0x0b10, 0xa828,
	0x0b12, 0x000c,
	0x0b14, 0x4068,
	0x0b16, 0x0000,
	0x0f30, 0x8335,
	0x0f32, 0x7067,
	0x0954, 0x0009,
	0x0956, 0x1100,
	0x0958, 0x8880,
	0x095a, 0x0001,
	0x0c00, 0x1110,
	0x0c02, 0x0011,
	0x0c04, 0x0000,
	0x0c06, 0x0200,
	0x0c10, 0x0040,
	0x0c12, 0x0040,
	0x0c14, 0x0040,
	0x0c16, 0x0040,
	0x0a10, 0x4000,
	0x3068, 0xf800,
	0x306a, 0xf876,
	0x006c, 0x0000,
	0x005e, 0x0200,
	0x000e, 0x0200,
	0x0e0a, 0x0001,
	0x004a, 0x0100,
	0x004c, 0x0000,
	0x000c, 0x0022,
	0x0008, 0x0b00,
	0x005a, 0x0202,
	0x0012, 0x000e,
	0x0018, 0x0a31,
	0x0022, 0x0008,
	0x0028, 0x0017,
	0x0024, 0x0028,
	0x002a, 0x002d,
	0x0026, 0x0030,
	0x002c, 0x07c7,
	0x002e, 0x1111,
	0x0030, 0x1111,
	0x0032, 0x1111,
	0x0006, 0x07bc,
	0x0a22, 0x0000,
	0x0a12, 0x0a20,
	0x0a14, 0x0798,
	0x003e, 0x0000,
	0x0074, 0x080e,
	0x0070, 0x0407,
	0x0002, 0x0000,
	0x0a02, 0x0100,
	0x0a24, 0x0100,
	0x0046, 0x0000,
	0x0076, 0x0000,
	0x0060, 0x0000,
	0x0062, 0x0530,
	0x0064, 0x0500,
	0x0066, 0x0530,
	0x0068, 0x0500,
	0x0122, 0x0300,
	0x015a, 0xff08,
	0x0804, 0x0200,
	0x005c, 0x0102,
	0x0a1a, 0x0800,
	0x004e, 0x0100
};
#endif

static void sensor_init(void)
{
#if MULTI_WRITE
	hi556_table_write_cmos_sensor(
		addr_data_pair_init_hi556,
		sizeof(addr_data_pair_init_hi556) /
		sizeof(kal_uint16));
#else
	write_cmos_sensor(0x0a00, 0x0000); // stream off
	write_cmos_sensor(0x0e00, 0x0102);
	write_cmos_sensor(0x0e02, 0x0102);
	write_cmos_sensor(0x0e0c, 0x0100);
	write_cmos_sensor(0x2000, 0x7400);
	write_cmos_sensor(0x2002, 0x001c);
	write_cmos_sensor(0x2004, 0x0242);
	write_cmos_sensor(0x2006, 0x0942);
	write_cmos_sensor(0x2008, 0x7007);
	write_cmos_sensor(0x200a, 0x0fd9);
	write_cmos_sensor(0x200c, 0x0259);
	write_cmos_sensor(0x200e, 0x7008);
	write_cmos_sensor(0x2010, 0x160e);
	write_cmos_sensor(0x2012, 0x0047);
	write_cmos_sensor(0x2014, 0x2118);
	write_cmos_sensor(0x2016, 0x0041);
	write_cmos_sensor(0x2018, 0x00d8);
	write_cmos_sensor(0x201a, 0x0145);
	write_cmos_sensor(0x201c, 0x0006);
	write_cmos_sensor(0x201e, 0x0181);
	write_cmos_sensor(0x2020, 0x13cc);
	write_cmos_sensor(0x2022, 0x2057);
	write_cmos_sensor(0x2024, 0x7001);
	write_cmos_sensor(0x2026, 0x0fca);
	write_cmos_sensor(0x2028, 0x00cb);
	write_cmos_sensor(0x202a, 0x009f);
	write_cmos_sensor(0x202c, 0x7002);
	write_cmos_sensor(0x202e, 0x13cc);
	write_cmos_sensor(0x2030, 0x019b);
	write_cmos_sensor(0x2032, 0x014d);
	write_cmos_sensor(0x2034, 0x2987);
	write_cmos_sensor(0x2036, 0x2766);
	write_cmos_sensor(0x2038, 0x0020);
	write_cmos_sensor(0x203a, 0x2060);
	write_cmos_sensor(0x203c, 0x0e5d);
	write_cmos_sensor(0x203e, 0x181d);
	write_cmos_sensor(0x2040, 0x2066);
	write_cmos_sensor(0x2042, 0x20c4);
	write_cmos_sensor(0x2044, 0x5000);
	write_cmos_sensor(0x2046, 0x0005);
	write_cmos_sensor(0x2048, 0x0000);
	write_cmos_sensor(0x204a, 0x01db);
	write_cmos_sensor(0x204c, 0x025a);
	write_cmos_sensor(0x204e, 0x00c0);
	write_cmos_sensor(0x2050, 0x0005);
	write_cmos_sensor(0x2052, 0x0006);
	write_cmos_sensor(0x2054, 0x0ad9);
	write_cmos_sensor(0x2056, 0x0259);
	write_cmos_sensor(0x2058, 0x0618);
	write_cmos_sensor(0x205a, 0x0258);
	write_cmos_sensor(0x205c, 0x2266);
	write_cmos_sensor(0x205e, 0x20c8);
	write_cmos_sensor(0x2060, 0x2060);
	write_cmos_sensor(0x2062, 0x707b);
	write_cmos_sensor(0x2064, 0x0fdd);
	write_cmos_sensor(0x2066, 0x81b8);
	write_cmos_sensor(0x2068, 0x5040);
	write_cmos_sensor(0x206a, 0x0020);
	write_cmos_sensor(0x206c, 0x5060);
	write_cmos_sensor(0x206e, 0x3143);
	write_cmos_sensor(0x2070, 0x5081);
	write_cmos_sensor(0x2072, 0x025c);
	write_cmos_sensor(0x2074, 0x7800);
	write_cmos_sensor(0x2076, 0x7400);
	write_cmos_sensor(0x2078, 0x001c);
	write_cmos_sensor(0x207a, 0x0242);
	write_cmos_sensor(0x207c, 0x0942);
	write_cmos_sensor(0x207e, 0x0bd9);
	write_cmos_sensor(0x2080, 0x0259);
	write_cmos_sensor(0x2082, 0x7008);
	write_cmos_sensor(0x2084, 0x160e);
	write_cmos_sensor(0x2086, 0x0047);
	write_cmos_sensor(0x2088, 0x2118);
	write_cmos_sensor(0x208a, 0x0041);
	write_cmos_sensor(0x208c, 0x00d8);
	write_cmos_sensor(0x208e, 0x0145);
	write_cmos_sensor(0x2090, 0x0006);
	write_cmos_sensor(0x2092, 0x0181);
	write_cmos_sensor(0x2094, 0x13cc);
	write_cmos_sensor(0x2096, 0x2057);
	write_cmos_sensor(0x2098, 0x7001);
	write_cmos_sensor(0x209a, 0x0fca);
	write_cmos_sensor(0x209c, 0x00cb);
	write_cmos_sensor(0x209e, 0x009f);
	write_cmos_sensor(0x20a0, 0x7002);
	write_cmos_sensor(0x20a2, 0x13cc);
	write_cmos_sensor(0x20a4, 0x019b);
	write_cmos_sensor(0x20a6, 0x014d);
	write_cmos_sensor(0x20a8, 0x2987);
	write_cmos_sensor(0x20aa, 0x2766);
	write_cmos_sensor(0x20ac, 0x0020);
	write_cmos_sensor(0x20ae, 0x2060);
	write_cmos_sensor(0x20b0, 0x0e5d);
	write_cmos_sensor(0x20b2, 0x181d);
	write_cmos_sensor(0x20b4, 0x2066);
	write_cmos_sensor(0x20b6, 0x20c4);
	write_cmos_sensor(0x20b8, 0x50a0);
	write_cmos_sensor(0x20ba, 0x0005);
	write_cmos_sensor(0x20bc, 0x0000);
	write_cmos_sensor(0x20be, 0x01db);
	write_cmos_sensor(0x20c0, 0x025a);
	write_cmos_sensor(0x20c2, 0x00c0);
	write_cmos_sensor(0x20c4, 0x0005);
	write_cmos_sensor(0x20c6, 0x0006);
	write_cmos_sensor(0x20c8, 0x0ad9);
	write_cmos_sensor(0x20ca, 0x0259);
	write_cmos_sensor(0x20cc, 0x0618);
	write_cmos_sensor(0x20ce, 0x0258);
	write_cmos_sensor(0x20d0, 0x2266);
	write_cmos_sensor(0x20d2, 0x20c8);
	write_cmos_sensor(0x20d4, 0x2060);
	write_cmos_sensor(0x20d6, 0x707b);
	write_cmos_sensor(0x20d8, 0x0fdd);
	write_cmos_sensor(0x20da, 0x86b8);
	write_cmos_sensor(0x20dc, 0x50e0);
	write_cmos_sensor(0x20de, 0x0020);
	write_cmos_sensor(0x20e0, 0x5100);
	write_cmos_sensor(0x20e2, 0x3143);
	write_cmos_sensor(0x20e4, 0x5121);
	write_cmos_sensor(0x20e6, 0x7800);
	write_cmos_sensor(0x20e8, 0x3140);
	write_cmos_sensor(0x20ea, 0x01c4);
	write_cmos_sensor(0x20ec, 0x01c1);
	write_cmos_sensor(0x20ee, 0x01c0);
	write_cmos_sensor(0x20f0, 0x01c4);
	write_cmos_sensor(0x20f2, 0x2700);
	write_cmos_sensor(0x20f4, 0x3d40);
	write_cmos_sensor(0x20f6, 0x7800);
	write_cmos_sensor(0x20f8, 0xffff);
	write_cmos_sensor(0x27fe, 0xe000);
	write_cmos_sensor(0x3000, 0x60f8);
	write_cmos_sensor(0x3002, 0x187f);
	write_cmos_sensor(0x3004, 0x7060);
	write_cmos_sensor(0x3006, 0x0114);
	write_cmos_sensor(0x3008, 0x60b0);
	write_cmos_sensor(0x300a, 0x1473);
	write_cmos_sensor(0x300c, 0x0013);
	write_cmos_sensor(0x300e, 0x140f);
	write_cmos_sensor(0x3010, 0x0040);
	write_cmos_sensor(0x3012, 0x100f);
	write_cmos_sensor(0x3014, 0x60f8);
	write_cmos_sensor(0x3016, 0x187f);
	write_cmos_sensor(0x3018, 0x7060);
	write_cmos_sensor(0x301a, 0x0114);
	write_cmos_sensor(0x301c, 0x60b0);
	write_cmos_sensor(0x301e, 0x1473);
	write_cmos_sensor(0x3020, 0x0013);
	write_cmos_sensor(0x3022, 0x140f);
	write_cmos_sensor(0x3024, 0x0040);
	write_cmos_sensor(0x3026, 0x000f);
	write_cmos_sensor(0x0b00, 0x0000);
	write_cmos_sensor(0x0b02, 0x0045);
	write_cmos_sensor(0x0b04, 0xb405);
	write_cmos_sensor(0x0b06, 0xc403);
	write_cmos_sensor(0x0b08, 0x0081);
	write_cmos_sensor(0x0b0a, 0x8252);
	write_cmos_sensor(0x0b0c, 0xf814);
	write_cmos_sensor(0x0b0e, 0xc618);
	write_cmos_sensor(0x0b10, 0xa828);
	write_cmos_sensor(0x0b12, 0x000c);
	write_cmos_sensor(0x0b14, 0x4068);
	write_cmos_sensor(0x0b16, 0x0000);
	write_cmos_sensor(0x0f30, 0x8335);
	write_cmos_sensor(0x0f32, 0x7067);
	write_cmos_sensor(0x0954, 0x0009);
	write_cmos_sensor(0x0956, 0x1100);
	write_cmos_sensor(0x0958, 0x8880);
	write_cmos_sensor(0x095a, 0x0001);
	write_cmos_sensor(0x0c00, 0x1110);
	write_cmos_sensor(0x0c02, 0x0011);
	write_cmos_sensor(0x0c04, 0x0000);
	write_cmos_sensor(0x0c06, 0x0200);
	write_cmos_sensor(0x0c10, 0x0040);
	write_cmos_sensor(0x0c12, 0x0040);
	write_cmos_sensor(0x0c14, 0x0040);
	write_cmos_sensor(0x0c16, 0x0040);
	write_cmos_sensor(0x0a10, 0x4000);
	write_cmos_sensor(0x3068, 0xf800);
	write_cmos_sensor(0x306a, 0xf876);
	write_cmos_sensor(0x006c, 0x0000);
	write_cmos_sensor(0x005e, 0x0200);
	write_cmos_sensor(0x000e, 0x0200); //org
	//write_cmos_sensor(0x000e, 0x0000); //odin
	write_cmos_sensor(0x0e0a, 0x0001);
	write_cmos_sensor(0x004a, 0x0100);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x000c, 0x0022);
	write_cmos_sensor(0x0008, 0x0b00);
	write_cmos_sensor(0x005a, 0x0202);
	write_cmos_sensor(0x0012, 0x000e);
	write_cmos_sensor(0x0018, 0x0a31);
	write_cmos_sensor(0x0022, 0x0008);
	write_cmos_sensor(0x0028, 0x0017);
	write_cmos_sensor(0x0024, 0x0028);
	write_cmos_sensor(0x002a, 0x002d);
	write_cmos_sensor(0x0026, 0x0030);
	write_cmos_sensor(0x002c, 0x07c7);
	write_cmos_sensor(0x002e, 0x1111);
	write_cmos_sensor(0x0030, 0x1111);
	write_cmos_sensor(0x0032, 0x1111);
	write_cmos_sensor(0x0006, 0x07bc);
	write_cmos_sensor(0x0a22, 0x0000);
	write_cmos_sensor(0x0a12, 0x0a20);
	write_cmos_sensor(0x0a14, 0x0798);
	write_cmos_sensor(0x003e, 0x0000);
	write_cmos_sensor(0x0074, 0x080e);
	write_cmos_sensor(0x0070, 0x0407);
	write_cmos_sensor(0x0002, 0x0000);
	write_cmos_sensor(0x0a02, 0x0100);
	write_cmos_sensor(0x0a24, 0x0100);
	write_cmos_sensor(0x0046, 0x0000);
	write_cmos_sensor(0x0076, 0x0000);
	write_cmos_sensor(0x0060, 0x0000);
	write_cmos_sensor(0x0062, 0x0530);
	write_cmos_sensor(0x0064, 0x0500);
	write_cmos_sensor(0x0066, 0x0530);
	write_cmos_sensor(0x0068, 0x0500);
	write_cmos_sensor(0x0122, 0x0300);
	write_cmos_sensor(0x015a, 0xff08);
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x005c, 0x0102);
	write_cmos_sensor(0x0a1a, 0x0800);
	write_cmos_sensor(0x004e, 0x0100);
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_preview_30fps_hi556[] = {
	0x0b0a, 0x8252,
	0x0f30, 0x8335,//6925,
	0x0f32, 0x7067,
	0x004a, 0x0100,
	0x004c, 0x0000,
	0x004e, 0x0100,
	0x000c, 0x0022,
	0x0008, 0x0b00,
	0x005a, 0x0202,
	0x0012, 0x000e,
	0x0018, 0x0a31,
	0x0022, 0x0008,
	0x0028, 0x0017,
	0x0024, 0x0028,
	0x002a, 0x002d,
	0x0026, 0x0030,
	0x002c, 0x07c7,
	0x002e, 0x1111,
	0x0030, 0x1111,
	0x0032, 0x1111,
	0x0006, 0x07DA,//0x0801
	0x0a22, 0x0000,
	0x0a12, 0x0a20,
	0x0a14, 0x0798,
	0x003e, 0x0000,
	0x0074, 0x07D8,
	0x0070, 0x0411,
	0x0804, 0x0200,
	0x0a04, 0x014a,
	0x090c, 0x0fdc,
	0x090e, 0x002d,
	0x0902, 0x4319,
	0x0914, 0xc10a,
	0x0916, 0x071f,
	0x0918, 0x0408,
	0x091a, 0x0c0d,
	0x091c, 0x0f09,
	0x091e, 0x0a00
};
#endif


static void preview_setting(void)
{
#if MULTI_WRITE
	hi556_table_write_cmos_sensor(
		addr_data_pair_preview_30fps_hi556,
		sizeof(addr_data_pair_preview_30fps_hi556) /
		sizeof(kal_uint16));
#else
	LOG_INF("%s fps = 300\n", __func__);
	write_cmos_sensor(0x0b0a, 0x8252);
	write_cmos_sensor(0x0f30, 0x8335);//6925);
	write_cmos_sensor(0x0f32, 0x7067);
	write_cmos_sensor(0x004a, 0x0100);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x004e, 0x0100); //perframe enable
	write_cmos_sensor(0x000c, 0x0022);
	write_cmos_sensor(0x0008, 0x0b00);
	write_cmos_sensor(0x005a, 0x0202);
	write_cmos_sensor(0x0012, 0x000e);
	write_cmos_sensor(0x0018, 0x0a31);
	write_cmos_sensor(0x0022, 0x0008);
	write_cmos_sensor(0x0028, 0x0017);
	write_cmos_sensor(0x0024, 0x0028);
	write_cmos_sensor(0x002a, 0x002d);
	write_cmos_sensor(0x0026, 0x0030);
	write_cmos_sensor(0x002c, 0x07c7);
	write_cmos_sensor(0x002e, 0x1111);
	write_cmos_sensor(0x0030, 0x1111);
	write_cmos_sensor(0x0032, 0x1111);
	write_cmos_sensor(0x0006, 0x07DA);
	write_cmos_sensor(0x0a22, 0x0000);
	write_cmos_sensor(0x0a12, 0x0a20);
	write_cmos_sensor(0x0a14, 0x0798);
	write_cmos_sensor(0x003e, 0x0000);
	write_cmos_sensor(0x0074, 0x07D8);
	write_cmos_sensor(0x0070, 0x0411);
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x0a04, 0x014a);
	write_cmos_sensor(0x090c, 0x0fdc);
	write_cmos_sensor(0x090e, 0x002d);
//===============================================
//             mipi 2 lane 880Mbps
//===============================================
	write_cmos_sensor(0x0902, 0x4319);
	write_cmos_sensor(0x0914, 0xc10a);
	write_cmos_sensor(0x0916, 0x071f);
	write_cmos_sensor(0x0918, 0x0408);
	write_cmos_sensor(0x091a, 0x0c0d);
	write_cmos_sensor(0x091c, 0x0f09);
	write_cmos_sensor(0x091e, 0x0a00);
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_capture_fps_hi556[] = {
	0x0b0a, 0x8252,
	0x0f30, 0x8335,//6925,
	0x0f32, 0x7067,
	0x004a, 0x0100,
	0x004c, 0x0000,
	0x004e, 0x0100,
	0x000c, 0x0022,
	0x0008, 0x0b00,
	0x005a, 0x0202,
	0x0012, 0x000e,
	0x0018, 0x0a31,
	0x0022, 0x0008,
	0x0028, 0x0017,
	0x0024, 0x0028,
	0x002a, 0x002d,
	0x0026, 0x0030,
	0x002c, 0x07c7,
	0x002e, 0x1111,
	0x0030, 0x1111,
	0x0032, 0x1111,
	0x0006, 0x0FB4,
	0x0a22, 0x0000,
	0x0a12, 0x0a20,
	0x0a14, 0x0798,
	0x003e, 0x0000,
	0x0074, 0x0FB2,
	0x0070, 0x0411,
	0x0804, 0x0200,
	0x0a04, 0x014a,
	0x090c, 0x0fdc,
	0x090e, 0x002d,
	0x0902, 0x4319,
	0x0914, 0xc10a,
	0x0916, 0x071f,
	0x0918, 0x0408,
	0x091a, 0x0c0d,
	0x091c, 0x0f09,
	0x091e, 0x0a00
};

kal_uint16 addr_data_pair_capture_30fps_hi556[] = {
	0x0b0a, 0x8252,
	0x0f30, 0x8335,//6925,
	0x0f32, 0x7067,
	0x004a, 0x0100,
	0x004c, 0x0000,
	0x004e, 0x0100,
	0x000c, 0x0022,
	0x0008, 0x0b00,
	0x005a, 0x0202,
	0x0012, 0x000e,
	0x0018, 0x0a31,
	0x0022, 0x0008,
	0x0028, 0x0017,
	0x0024, 0x0028,
	0x002a, 0x002d,
	0x0026, 0x0030,
	0x002c, 0x07c7,
	0x002e, 0x1111,
	0x0030, 0x1111,
	0x0032, 0x1111,
	0x0006, 0x07DA,
	0x0a22, 0x0000,
	0x0a12, 0x0a20,
	0x0a14, 0x0798,
	0x003e, 0x0000,
	0x0074, 0x07D8,
	0x0070, 0x0411,
	0x0804, 0x0200,
	0x0a04, 0x014a,
	0x090c, 0x0fdc,
	0x090e, 0x002d,
	0x0902, 0x4319,
	0x0914, 0xc10a,
	0x0916, 0x071f,
	0x0918, 0x0408,
	0x091a, 0x0c0d,
	0x091c, 0x0f09,
	0x091e, 0x0a00
};
#endif

kal_uint16 addr_data_pair_normal_video_30fps_hi556[] = {
0x0a00, 0x0000,
0x0b0a, 0x8252,
0x0f30, 0x8335,//6925,
0x0f32, 0x7067,
0x004a, 0x0100,
0x004c, 0x0000,
0x000c, 0x0022,
0x0008, 0x0b00,
0x005a, 0x0202,
0x0012, 0x000e,
0x0018, 0x0a31,
0x0022, 0x0008,
0x0028, 0x0017,
0x0024, 0x011a,
0x002a, 0x011f,
0x0026, 0x0122,
0x002c, 0x06d3,
0x002e, 0x1111,
0x0030, 0x1111,
0x0032, 0x1111,
0x0006, 0x07DA,
0x0a22, 0x0000,
0x0a12, 0x0a20,
0x0a14, 0x05b2,
0x003e, 0x0000,
0x0074, 0x07D8,
0x0070, 0x0410,
0x0804, 0x0200,
0x0a04, 0x014a,
0x090c, 0x0fdc,
0x090e, 0x002d,
0x0902, 0x4319,
0x0914, 0xc10a,
0x0916, 0x071f,
0x0918, 0x0408,
0x091a, 0x0c0d,
0x091c, 0x0f09,
0x091e, 0x0a00,
0x0958, 0x8880,

};

static void video_setting(kal_uint16 currefps)
{
#if MULTI_WRITE
	if (currefps == 300) {
		hi556_table_write_cmos_sensor(
			addr_data_pair_normal_video_30fps_hi556,
			sizeof(addr_data_pair_normal_video_30fps_hi556) /
			sizeof(kal_uint16));
	} else {
		hi556_table_write_cmos_sensor(
			addr_data_pair_capture_fps_hi556,
			sizeof(addr_data_pair_capture_fps_hi556) /
			sizeof(kal_uint16));
	}
#else
	if (currefps == 300) {
		LOG_INF("%s fps = 300\n", __func__);
		write_cmos_sensor(0x0b0a, 0x8252);
		write_cmos_sensor(0x0f30, 0x8335);//6925,);
		write_cmos_sensor(0x0f32, 0x7067);
		write_cmos_sensor(0x004a, 0x0100);
		write_cmos_sensor(0x004c, 0x0000);
		write_cmos_sensor(0x004e, 0x0100); //perframe enable
		write_cmos_sensor(0x000c, 0x0022);
		write_cmos_sensor(0x0008, 0x0b00);
		write_cmos_sensor(0x005a, 0x0202);
		write_cmos_sensor(0x0012, 0x000e);
		write_cmos_sensor(0x0018, 0x0a31);
		write_cmos_sensor(0x0022, 0x0008);
		write_cmos_sensor(0x0028, 0x0017);
		write_cmos_sensor(0x0024, 0x0028);
		write_cmos_sensor(0x002a, 0x002d);
		write_cmos_sensor(0x0026, 0x0030);
		write_cmos_sensor(0x002c, 0x07c7);
		write_cmos_sensor(0x002e, 0x1111);
		write_cmos_sensor(0x0030, 0x1111);
		write_cmos_sensor(0x0032, 0x1111);
		write_cmos_sensor(0x0006, 0x0801);
		write_cmos_sensor(0x0a22, 0x0000);
		write_cmos_sensor(0x0a12, 0x0a20);
		write_cmos_sensor(0x0a14, 0x0798);
		write_cmos_sensor(0x003e, 0x0000);
		write_cmos_sensor(0x0074, 0x07ff);
		write_cmos_sensor(0x0070, 0x0411);
		write_cmos_sensor(0x0804, 0x0200);
		write_cmos_sensor(0x0a04, 0x014a);
		write_cmos_sensor(0x090c, 0x0fdc);
		write_cmos_sensor(0x090e, 0x002d);
//===============================================
//             mipi 2 lane 880Mbps
//===============================================
		write_cmos_sensor(0x0902, 0x4319);
		write_cmos_sensor(0x0914, 0xc10a);
		write_cmos_sensor(0x0916, 0x071f);
		write_cmos_sensor(0x0918, 0x0408);
		write_cmos_sensor(0x091a, 0x0c0d);
		write_cmos_sensor(0x091c, 0x0f09);
		write_cmos_sensor(0x091e, 0x0a00);
	} else	{
		LOG_INF("%s fps not 300\n", __func__);
		//Sensor Information////////////////////////////
		//Sensor	  : Hi-556
		//Date		  : 2016-10-19
		//Customer        : MTK_validation
		//Image size	  : 2592x1944
		//MCLK		  : 24MHz
		//MIPI speed(Mbps): 880Mbps x 2Lane
		//Frame Length	  : 4166
		//Line Length	  : 2816
		//Max Fps	  : 15.0fps
		//Pixel order	  : Green 1st (=GB)
		//X/Y-flip	  : X-flip
		//BLC offset	  : 64code
		////////////////////////////////////////////////
		write_cmos_sensor(0x0b0a, 0x8252);
		write_cmos_sensor(0x0f30, 0x8335);
		write_cmos_sensor(0x0f32, 0x7067);
		write_cmos_sensor(0x004a, 0x0100);
		write_cmos_sensor(0x004c, 0x0000);
		write_cmos_sensor(0x004e, 0x0100); //perframe enable
		write_cmos_sensor(0x000c, 0x0022);
		write_cmos_sensor(0x0008, 0x0b00);
		write_cmos_sensor(0x005a, 0x0202);
		write_cmos_sensor(0x0012, 0x000e);
		write_cmos_sensor(0x0018, 0x0a31);
		write_cmos_sensor(0x0022, 0x0008);
		write_cmos_sensor(0x0028, 0x0017);
		write_cmos_sensor(0x0024, 0x0028);
		write_cmos_sensor(0x002a, 0x002d);
		write_cmos_sensor(0x0026, 0x0030);
		write_cmos_sensor(0x002c, 0x07c7);
		write_cmos_sensor(0x002e, 0x1111);
		write_cmos_sensor(0x0030, 0x1111);
		write_cmos_sensor(0x0032, 0x1111);
		write_cmos_sensor(0x0006, 0x0FB4);
		write_cmos_sensor(0x0a22, 0x0000);
		write_cmos_sensor(0x0a12, 0x0a20);
		write_cmos_sensor(0x0a14, 0x0798);
		write_cmos_sensor(0x003e, 0x0000);
		write_cmos_sensor(0x0074, 0x0FB2);
		write_cmos_sensor(0x0070, 0x0411);
		write_cmos_sensor(0x0804, 0x0200);
		write_cmos_sensor(0x0a04, 0x014a);
		write_cmos_sensor(0x090c, 0x0fdc);
		write_cmos_sensor(0x090e, 0x002d);
		//===============================================
		//             mipi 2 lane 880Mbps
		//===============================================
		write_cmos_sensor(0x0902, 0x4319);
		write_cmos_sensor(0x0914, 0xc10a);
		write_cmos_sensor(0x0916, 0x071f);
		write_cmos_sensor(0x0918, 0x0408);
		write_cmos_sensor(0x091a, 0x0c0d);
		write_cmos_sensor(0x091c, 0x0f09);
		write_cmos_sensor(0x091e, 0x0a00);
	}
#endif
}


static void capture_setting(kal_uint16 currefps)
{
#if MULTI_WRITE
	if (currefps == 300) {
		hi556_table_write_cmos_sensor(
			addr_data_pair_capture_30fps_hi556,
			sizeof(addr_data_pair_capture_30fps_hi556) /
			sizeof(kal_uint16));
	} else {
		hi556_table_write_cmos_sensor(
			addr_data_pair_capture_fps_hi556,
			sizeof(addr_data_pair_capture_fps_hi556) /
			sizeof(kal_uint16));
	}
#else
	if (currefps == 300) {
		LOG_INF("%s fps = 300\n", __func__);
		write_cmos_sensor(0x0b0a, 0x8252);
		write_cmos_sensor(0x0f30, 0x8335);
		write_cmos_sensor(0x0f32, 0x7067);
		write_cmos_sensor(0x004a, 0x0100);
		write_cmos_sensor(0x004c, 0x0000);
		write_cmos_sensor(0x004e, 0x0100); //perframe enable
		write_cmos_sensor(0x000c, 0x0022);
		write_cmos_sensor(0x0008, 0x0b00);
		write_cmos_sensor(0x005a, 0x0202);
		write_cmos_sensor(0x0012, 0x000e);
		write_cmos_sensor(0x0018, 0x0a31);
		write_cmos_sensor(0x0022, 0x0008);
		write_cmos_sensor(0x0028, 0x0017);
		write_cmos_sensor(0x0024, 0x0028);
		write_cmos_sensor(0x002a, 0x002d);
		write_cmos_sensor(0x0026, 0x0030);
		write_cmos_sensor(0x002c, 0x07c7);
		write_cmos_sensor(0x002e, 0x1111);
		write_cmos_sensor(0x0030, 0x1111);
		write_cmos_sensor(0x0032, 0x1111);
		write_cmos_sensor(0x0006, 0x07DA);
		write_cmos_sensor(0x0a22, 0x0000);
		write_cmos_sensor(0x0a12, 0x0a20);
		write_cmos_sensor(0x0a14, 0x0798);
		write_cmos_sensor(0x003e, 0x0000);
		write_cmos_sensor(0x0074, 0x07D8);
		write_cmos_sensor(0x0070, 0x0411);
		write_cmos_sensor(0x0804, 0x0200);
		write_cmos_sensor(0x0a04, 0x014a);
		write_cmos_sensor(0x090c, 0x0fdc);
		write_cmos_sensor(0x090e, 0x002d);
//===============================================
//             mipi 2 lane 880Mbps
//===============================================
		write_cmos_sensor(0x0902, 0x4319);
		write_cmos_sensor(0x0914, 0xc10a);
		write_cmos_sensor(0x0916, 0x071f);
		write_cmos_sensor(0x0918, 0x0408);
		write_cmos_sensor(0x091a, 0x0c0d);
		write_cmos_sensor(0x091c, 0x0f09);
		write_cmos_sensor(0x091e, 0x0a00);
	} else	{
		LOG_INF("%s fps not 300\n", __func__);
		//Sensor Information////////////////////////////
		//Sensor	  : Hi-556
		//Date		  : 2016-10-19
		//Customer        : MTK_validation
		//Image size	  : 2592x1944
		//MCLK		  : 24MHz
		//MIPI speed(Mbps): 880Mbps x 2Lane
		//Frame Length	  : 4166
		//Line Length	  : 2816
		//Max Fps	  : 15.0fps
		//Pixel order	  : Green 1st (=GB)
		//X/Y-flip	  : X-flip
		//BLC offset	  : 64code
		////////////////////////////////////////////////
		write_cmos_sensor(0x0b0a, 0x8252);
		write_cmos_sensor(0x0f30, 0x8335);
		write_cmos_sensor(0x0f32, 0x7067);
		write_cmos_sensor(0x004a, 0x0100);
		write_cmos_sensor(0x004c, 0x0000);
		write_cmos_sensor(0x004e, 0x0100); //perframe enable
		write_cmos_sensor(0x000c, 0x0022);
		write_cmos_sensor(0x0008, 0x0b00);
		write_cmos_sensor(0x005a, 0x0202);
		write_cmos_sensor(0x0012, 0x000e);
		write_cmos_sensor(0x0018, 0x0a31);
		write_cmos_sensor(0x0022, 0x0008);
		write_cmos_sensor(0x0028, 0x0017);
		write_cmos_sensor(0x0024, 0x0028);
		write_cmos_sensor(0x002a, 0x002d);
		write_cmos_sensor(0x0026, 0x0030);
		write_cmos_sensor(0x002c, 0x07c7);
		write_cmos_sensor(0x002e, 0x1111);
		write_cmos_sensor(0x0030, 0x1111);
		write_cmos_sensor(0x0032, 0x1111);
		write_cmos_sensor(0x0006, 0x0FB4);
		write_cmos_sensor(0x0a22, 0x0000);
		write_cmos_sensor(0x0a12, 0x0a20);
		write_cmos_sensor(0x0a14, 0x0798);
		write_cmos_sensor(0x003e, 0x0000);
		write_cmos_sensor(0x0074, 0x0FB2);
		write_cmos_sensor(0x0070, 0x0411);
		write_cmos_sensor(0x0804, 0x0200);
		write_cmos_sensor(0x0a04, 0x014a);
		write_cmos_sensor(0x090c, 0x0fdc);
		write_cmos_sensor(0x090e, 0x002d);
		//===============================================
		//             mipi 2 lane 880Mbps
		//===============================================
		write_cmos_sensor(0x0902, 0x4319);
		write_cmos_sensor(0x0914, 0xc10a);
		write_cmos_sensor(0x0916, 0x071f);
		write_cmos_sensor(0x0918, 0x0408);
		write_cmos_sensor(0x091a, 0x0c0d);
		write_cmos_sensor(0x091c, 0x0f09);
		write_cmos_sensor(0x091e, 0x0a00);
	}
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_hs_video_hi556[] = {
	0x0b0a, 0x8252,
	0x0f30, 0x8335,
	0x0f32, 0x7267,
	0x004a, 0x0100,
	0x004c, 0x0000,
	0x004e, 0x0100,
	0x000c, 0x0022,
	0x0008, 0x0b00,
	0x005a, 0x0208,
	0x0012, 0x0018,
	0x0018, 0x0a27,
	0x0022, 0x0008,
	0x0028, 0x0017,
	0x0024, 0x002e,
	0x002a, 0x0033,
	0x0026, 0x003c,
	0x002c, 0x07bb,
	0x002e, 0x1111,
	0x0030, 0x1111,
	0x0032, 0x7711,
	0x0006, 0x0208,
	0x0a22, 0x0100,
	0x0a12, 0x0280,
	0x0a14, 0x01e0,
	0x003e, 0x0000,
	0x0074, 0x0206,
	0x0070, 0x0411,
	0x0804, 0x0200,
	0x0a04, 0x016a,
	0x090c, 0x0270,
	0x090e, 0x000c,
	0x0902, 0x4319,
	0x0914, 0xc103,
	0x0916, 0x0207,
	0x0918, 0x0302,
	0x091a, 0x0406,
	0x091c, 0x0903,
	0x091e, 0x0300
};
#endif

static void hs_video_setting(void)
{
	//Sensor Information////////////////////////////
	//Sensor	  : hi-556
	//Date		  : 2016-10-19
	//Customer        : MTK_validation
	//Image size	  : 640x480
	//MCLK		  : 24MHz
	//MIPI speed(Mbps): 220Mbps x 2Lane
	//Frame Length	  : 520
	//Line Length	  : 2816
	//Max Fps	  : 120.19fps
	//Pixel order	  : Green 1st (=GB)
	//X/Y-flip	  : X-flip
	//BLC offset	  : 64code
	////////////////////////////////////////////////
#if MULTI_WRITE
	hi556_table_write_cmos_sensor(
		addr_data_pair_hs_video_hi556,
		sizeof(addr_data_pair_hs_video_hi556) /
		sizeof(kal_uint16));
#else
	write_cmos_sensor(0x0b0a, 0x8252);
	write_cmos_sensor(0x0f30, 0x8335);
	write_cmos_sensor(0x0f32, 0x7267);
	write_cmos_sensor(0x004a, 0x0100);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x004e, 0x0100); //perframe enable
	write_cmos_sensor(0x000c, 0x0022);
	write_cmos_sensor(0x0008, 0x0b00);
	write_cmos_sensor(0x005a, 0x0208);
	write_cmos_sensor(0x0012, 0x0018);
	write_cmos_sensor(0x0018, 0x0a27);
	write_cmos_sensor(0x0022, 0x0008);
	write_cmos_sensor(0x0028, 0x0017);
	write_cmos_sensor(0x0024, 0x002e);
	write_cmos_sensor(0x002a, 0x0033);
	write_cmos_sensor(0x0026, 0x003c);
	write_cmos_sensor(0x002c, 0x07bb);
	write_cmos_sensor(0x002e, 0x1111);
	write_cmos_sensor(0x0030, 0x1111);
	write_cmos_sensor(0x0032, 0x7711);
	write_cmos_sensor(0x0006, 0x0208);
	write_cmos_sensor(0x0a22, 0x0100);
	write_cmos_sensor(0x0a12, 0x0280);
	write_cmos_sensor(0x0a14, 0x01e0);
	write_cmos_sensor(0x003e, 0x0000);
	write_cmos_sensor(0x0074, 0x0206);
	write_cmos_sensor(0x0070, 0x0411);
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x0a04, 0x016a);
	write_cmos_sensor(0x090c, 0x0270);
	write_cmos_sensor(0x090e, 0x000c);
	//===============================================
	//             mipi 2 lane 220Mbps
	//===============================================
	write_cmos_sensor(0x0902, 0x4319);
	write_cmos_sensor(0x0914, 0xc103);
	write_cmos_sensor(0x0916, 0x0207);
	write_cmos_sensor(0x0918, 0x0302);
	write_cmos_sensor(0x091a, 0x0406);
	write_cmos_sensor(0x091c, 0x0903);
	write_cmos_sensor(0x091e, 0x0300);
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_slim_video_hi556[] = {
	0x0b0a, 0x8252,
	0x0f30, 0x8335,
	0x0f32, 0x7167,
	0x004a, 0x0100,
	0x004c, 0x0000,
	0x004e, 0x0100,
	0x000c, 0x0022,
	0x0008, 0x0b00,
	0x005a, 0x0204,
	0x0012, 0x001c,
	0x0018, 0x0a23,
	0x0022, 0x0008,
	0x0028, 0x0017,
	0x0024, 0x0122,
	0x002a, 0x0127,
	0x0026, 0x012c,
	0x002c, 0x06cb,
	0x002e, 0x1111,
	0x0030, 0x1111,
	0x0032, 0x3311,
	0x0006, 0x07DA,
	0x0a22, 0x0000,
	0x0a12, 0x0500,
	0x0a14, 0x02d0,
	0x003e, 0x0000,
	0x0074, 0x07D8,
	0x0070, 0x0411,
	0x0804, 0x0200,
	0x0a04, 0x016a,
	0x090e, 0x0010,
	0x090c, 0x09c0,
	0x0902, 0x4319,
	0x0914, 0xc106,
	0x0916, 0x040e,
	0x0918, 0x0304,
	0x091c, 0x0e06,
	0x091a, 0x0709,
	0x091e, 0x0300
};
#endif


static void slim_video_setting(void)
{
	//Sensor Information////////////////////////////
	//Sensor	  : hi-556
	//Date		  : 2016-10-19
	//Customer        : MTK_validation
	//Image size	  : 1280x720
	//MCLK		  : 24MHz
	//MIPI speed(Mbps): 440Mbps x 2Lane
	//Frame Length	  : 2083
	//Line Length	  : 2816
	//Max Fps	  : 30.0fps
	//Pixel order	  : Green 1st (=GB)
	//X/Y-flip	  : X-flip
	//BLC offset	  : 64code
	////////////////////////////////////////////////
#if MULTI_WRITE
	hi556_table_write_cmos_sensor(
		addr_data_pair_slim_video_hi556,
		sizeof(addr_data_pair_slim_video_hi556) /
		sizeof(kal_uint16));
#else
	write_cmos_sensor(0x0b0a, 0x8252);
	write_cmos_sensor(0x0f30, 0x8335);
	write_cmos_sensor(0x0f32, 0x7167);
	write_cmos_sensor(0x004a, 0x0100);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x004e, 0x0100); //perframe enable
	write_cmos_sensor(0x000c, 0x0022);
	write_cmos_sensor(0x0008, 0x0b00);
	write_cmos_sensor(0x005a, 0x0204);
	write_cmos_sensor(0x0012, 0x001c);
	write_cmos_sensor(0x0018, 0x0a23);
	write_cmos_sensor(0x0022, 0x0008);
	write_cmos_sensor(0x0028, 0x0017);
	write_cmos_sensor(0x0024, 0x0122);
	write_cmos_sensor(0x002a, 0x0127);
	write_cmos_sensor(0x0026, 0x012c);
	write_cmos_sensor(0x002c, 0x06cb);
	write_cmos_sensor(0x002e, 0x1111);
	write_cmos_sensor(0x0030, 0x1111);
	write_cmos_sensor(0x0032, 0x3311);
	write_cmos_sensor(0x0006, 0x07DA);
	write_cmos_sensor(0x0a22, 0x0000);
	write_cmos_sensor(0x0a12, 0x0500);
	write_cmos_sensor(0x0a14, 0x02d0);
	write_cmos_sensor(0x003e, 0x0000);
	write_cmos_sensor(0x0074, 0x07D8);
	write_cmos_sensor(0x0070, 0x0411);
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x0a04, 0x016a);
	write_cmos_sensor(0x090e, 0x0010);
	write_cmos_sensor(0x090c, 0x09c0);
	//===============================================
	//             mipi 2 lane 440Mbps
	//===============================================
	write_cmos_sensor(0x0902, 0x4319);
	write_cmos_sensor(0x0914, 0xc106);
	write_cmos_sensor(0x0916, 0x040e);
	write_cmos_sensor(0x0918, 0x0304);
	write_cmos_sensor(0x091c, 0x0e06);
	write_cmos_sensor(0x091a, 0x0709);
	write_cmos_sensor(0x091e, 0x0300);
#endif
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	int rc = 0;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF(
					"get_imgsensor_id i2c write id : 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				    //+bug584789 chenbocheng.wt, add, 2020/10/30, add main camera s5kgm1 and wide camera hi556 otp code
				    rc = hi556_sensor_otp_info();
					if(rc == 0){
						//*sensor_id = 0xFFFFFFFF;
						LOG_INF("Hi556 read OTP:NOK");
						//return ERROR_SENSOR_CONNECT_FAIL;
				}
				    //+bug584789 chenbocheng.wt, add, 2020/10/30, add main camera s5kgm1 and wide camera hi556 otp code
				return ERROR_NONE;
			}

			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		LOG_INF("get_imgsensor_id Read id fail,sensor id: 0x%x\n", *sensor_id);
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

	LOG_INF("[%s]: PLATFORM:MT6737,MIPI 24LANE\n", __func__);
	LOG_INF("preview 2592*1944@30fps,880Mbps/lane;"
		"capture 2592*1944@30fps,880Mbps/lane\n");
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("open i2c write id: 0x%x, sensor id: 0x%x\n",
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
		LOG_INF("open %s sensor id fail: 0x%x\n", __func__, sensor_id);
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	/* initail sequence write in  */
	sensor_init();

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
	LOG_INF("Caputre fps:%d\n", imgsensor.current_fps);
	capture_setting(imgsensor.current_fps);

	return ERROR_NONE;

}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	video_setting(imgsensor.current_fps);
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
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

static kal_uint32 get_resolution(
		MSDK_SENSOR_RESOLUTION_INFO_STRUCT * sensor_resolution)
{
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
	return ERROR_NONE;
}    /*    get_resolution    */


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

	sensor_info->SensroInterfaceType =
	imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame =
		imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent =
		imgsensor_info.isp_driving_current;
/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame =
		imgsensor_info.ae_shut_delay_frame;
/* The frame of setting sensor gain */
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine =
		imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum =
		imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber =
		imgsensor_info.mipi_lane_num;
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
	default:
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);

	if ((framerate == 30) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 15) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = 10 * framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);
	set_dummy();
	return ERROR_NONE;
}


static kal_uint32 set_auto_flicker_mode(kal_bool enable,
			UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
			enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n",
				scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	    frame_length = imgsensor_info.pre.pclk / framerate * 10 /
			imgsensor_info.pre.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.pre.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
	    frame_length = imgsensor_info.normal_video.pclk /
			framerate * 10 / imgsensor_info.normal_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.normal_video.framelength) ?
		(frame_length - imgsensor_info.normal_video.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.normal_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps ==
		    imgsensor_info.cap1.max_framerate) {
			frame_length = imgsensor_info.cap1.pclk /
				framerate * 10 /
				imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length >
				imgsensor_info.cap1.framelength) ?
				(frame_length - imgsensor_info.cap1.framelength)
				: 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength
				+ imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps !=
			    imgsensor_info.cap.max_framerate)
				LOG_INF(
					"fps %d fps not support,use cap: %d fps!\n",
					framerate,
					imgsensor_info.cap.max_framerate / 10);
			frame_length = imgsensor_info.cap.pclk /
				framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length >
				imgsensor_info.cap.framelength) ?
			(frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap.framelength +
				imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
	    frame_length = imgsensor_info.hs_video.pclk /
			framerate * 10 / imgsensor_info.hs_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.hs_video.framelength) ? (frame_length -
			imgsensor_info.hs_video.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.hs_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
	    frame_length = imgsensor_info.slim_video.pclk /
			framerate * 10 / imgsensor_info.slim_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.slim_video.framelength) ? (frame_length -
			imgsensor_info.slim_video.framelength) : 0;
	    imgsensor.frame_length =
			imgsensor_info.slim_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	default:  //coding with  preview scenario by default
	    frame_length = imgsensor_info.pre.pclk / framerate * 10 /
						imgsensor_info.pre.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.pre.framelength +
				imgsensor.dummy_line;
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
				enum MSDK_SCENARIO_ID_ENUM scenario_id,
				MUINT32 *framerate)
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

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("%s enable: %d", __func__, enable);

	if (enable) {
// 0x5E00[8]: 1 enable,  0 disable
// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0a04, 0x0143);
		write_cmos_sensor(0x0200, 0x0001);
	}
#if 1
	else {
// 0x5E00[8]: 1 enable,  0 disable
// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0a04, 0x0142);
		write_cmos_sensor(0x0200, 0x0000);
	}
#endif
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable)
		write_cmos_sensor(0x0a00, 0x0100); // stream on
	else
		write_cmos_sensor(0x0a00, 0x0000); // stream off

	mdelay(10);
	return ERROR_NONE;
}

static kal_uint32 feature_control(
			MSDK_SENSOR_FEATURE_ENUM feature_id,
			UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data =
		(unsigned long long *) feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
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
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*feature_return_para_32 = 1; /*BINNING_NONE*/
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if (*(feature_data + 2))/* HDR on */
				*feature_return_para_32 = 1;/*BINNING_NONE*/
			else
				*feature_return_para_32 = 1;/*BINNING_AVERAGED*/
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		default:
			*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
			break;
		}
		pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d\n",
			*feature_return_para_32);

		*feature_para_len = 4;
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
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
	break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL) * feature_data);
	break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
	break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
	break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
	break;
	case SENSOR_FEATURE_SET_REGISTER:
	    write_cmos_sensor(sensor_reg_data->RegAddr,
						sensor_reg_data->RegData);
	break;
	case SENSOR_FEATURE_GET_REGISTER:
	    sensor_reg_data->RegData =
				read_cmos_sensor(sensor_reg_data->RegAddr);
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
	case SENSOR_FEATURE_SET_TEST_PATTERN:
	    set_test_pattern_mode((BOOL)*feature_data);
	break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
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

	    wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)
			(uintptr_t)(*(feature_data+1));

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
    		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    		default:
    			memcpy((void *)wininfo,
    				(void *)&imgsensor_winsize_info[0],
    				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
    		break;
		}
	break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
	    LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
	#if 0
	    ihdr_write_shutter_gain((UINT16)*feature_data,
			(UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
	#endif
	break;
	//+bug 558061, zhanglinfeng.wt, modify, 2020/06/19, modify codes for mipi rate is 0
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
    		//+bug 558061, shaozhuchao.wt, modify, 2020/07/23, modify codes for main camera hw remosaic
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
    		//-bug 558061, shaozhuchao.wt, modify, 2020/07/23, modify codes for main camera hw remosaic
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
		//+bug 558061, shaozhuchao.wt, modify, 2020/07/23, modify codes for main camera hw remosaic
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
		//-bug 558061, shaozhuchao.wt, modify, 2020/07/23, modify codes for main camera hw remosaic
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;
	//-bug 558061, zhanglinfeng.wt, modify, 2020/06/19, modify codes for mipi rate is 0
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
        default:
            rate = imgsensor_info.pre.mipi_pixel_rate;
            break;
        }
        LOG_INF("hi556 SENSOR_FEATURE_GET_MIPI_PIXEL_RATE");
        *(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = rate;
    }
    break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_i32 = 0;
		*feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		streaming_control(KAL_FALSE);
	break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		if (*feature_data != 0)
			set_shutter(*feature_data);
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

UINT32 HI556_MIPI_RAW_BLADE_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc =  &sensor_func;
	return ERROR_NONE;
}	/*	HI556_MIPI_RAW_SensorInit	*/
