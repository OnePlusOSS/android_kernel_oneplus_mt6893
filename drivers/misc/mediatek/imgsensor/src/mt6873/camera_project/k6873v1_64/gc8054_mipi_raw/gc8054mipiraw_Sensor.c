 /*
 *
 * Filename:
 * ---------
 *     GC8054mipi_Sensor.c
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

#include "gc8054mipiraw_Sensor.h"

/************************** Modify Following Strings for Debug **************************/
#define PFX "gc8054_camera_sensor"
#define LOG_1 LOG_INF("GC8054MIPI, 2LANE\n")
/****************************   Modify end    *******************************************/
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __func__, ##args)

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define DEVICE_VERSION_GC8054     "gc8054"
//extern void register_imgsensor_deviceinfo(char *name, char *version, u8 module_id);
static kal_uint8 deviceInfo_register_value = 0x00;
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static bool bNeedSetNormalMode = KAL_FALSE;
#endif
static DEFINE_SPINLOCK(imgsensor_drv_lock);

extern u32 pinSetIdx;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = GC8054_SENSOR_ID,
	.checksum_value = 0x1b375588,
	.pre = {
		.pclk = 403200000,
		.linelength = 5248,
		.framelength = 2532,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 275200000,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 403200000,
		.linelength = 5248,
		.framelength = 2532,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 275200000,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 403200000,
		.linelength = 5248,
		.framelength = 2532,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 1836,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 275200000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 201600000,
		.linelength = 5248,
		.framelength = 1266,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1632,
		.grabwindow_height = 1224,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 70080000,
		.max_framerate = 300,
	},
	.slim_video = {
		.pclk = 201600000,
		.linelength = 5248,
		.framelength = 1266,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1632,
		.grabwindow_height = 1224,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 70080000,
		.max_framerate = 300,
	},
	.custom1 = {
		.pclk = 403200000,
		.linelength = 5248,
		.framelength = 3200,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 264960000,
		.max_framerate = 240,
	},
	.margin = 16,
	.min_shutter = 4,
	.min_gain = 64, /*1x gain*/
	.max_gain = 1024, /*16x gain*/
	.min_gain_iso = 100,
	.gain_step = 1,
	.gain_type = 4,/*to be modify,no gain table for hynix*/
	.max_frame_length = 0xfffe,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 6,                                   /*support sensor mode num*/

	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
	.custom1_delay_frame = 2,                               /*enter custom1 delay frame num*/
	.frame_time_delay_frame = 2,                            /*enter custom1 delay frame num*/

	.isp_driving_current = ISP_DRIVING_4MA,                 /*mclk driving current*/
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,    /*sensor_interface_type*/
	.mipi_sensor_type = MIPI_OPHY_NCSI2,                    /*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x21, 0x20, 0xff},
	.i2c_speed = 400,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x900,
	.gain = 0x40,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = 0,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	.i2c_write_id = 0x62,
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	.current_ae_effective_frame = 2,
	#endif
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[6] = {
	{ 3264, 2448,   0,   0, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448}, /* Preview */
	{ 3264, 2448,   0,   0, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448}, /* capture */
	{ 3264, 2448,   0, 306, 3264, 1836, 3264, 1836, 0, 0, 3264, 1836, 0, 0, 3264, 1836}, /* video */
	{ 3264, 2448,   0,   0, 3264, 2448, 1632, 1224, 0, 0, 1632, 1224, 0, 0, 1632, 1224}, /* hs video */
	{ 3264, 2448,   0,   0, 3264, 2448, 1632, 1224, 0, 0, 1632, 1224, 0, 0, 1632, 1224}, /* slim video */
	{ 3264, 2448, 480, 360, 2304, 1728, 2304, 1728, 0, 0, 2304, 1728, 0, 0, 2304, 1728}  /* custom1 */
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = { (char)((addr >> 8) & 0xff), (char)(addr & 0xff) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor_16(kal_uint32 addr, kal_uint32 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF),
			     (char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}
#define I2C_BUFFER_LEN 255 /* trans# max is 255, each 4 bytes */
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
		/* Write when remain buffer size is less than  bytes or reach end of data */
		if ((I2C_BUFFER_LEN - tosend) < 3 || IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id,
								3, imgsensor_info.i2c_speed);
			tosend = 0;
		}
	}
	return 0;
}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define MODULE_ID_OFFSET 0x0000
static kal_uint16 read_module_id(void)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(MODULE_ID_OFFSET >> 8) , (char)(MODULE_ID_OFFSET & 0xFF) };
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,0xA2/*EEPROM_READ_ID*/);
	if (get_byte == 0) {
		iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, 0xA2/*EEPROM_READ_ID*/);
	}
	return get_byte;
}

static kal_uint8 gGc8054_SN[CAMERA_MODULE_SN_LENGTH];
static void read_eeprom_SN(void)
{
	kal_uint16 idx = 0;
	kal_uint8 *get_byte= &gGc8054_SN[0];
	for (idx = 0; idx <CAMERA_MODULE_SN_LENGTH; idx++) {
		char pusendcmd[2] = {0x00 , (char)((0xB0 + idx) & 0xFF) };
		iReadRegI2C(pusendcmd , 2, (u8*)&get_byte[idx],1, 0xA2);
		LOG_INF("gGc8054_SN[%d]: 0x%x  0x%x\n", idx, get_byte[idx], gGc8054_SN[idx]);
	}
}

#define  CAMERA_MODULE_INFO_LENGTH  (8)
static kal_uint8 gGC8054_CamInfo[CAMERA_MODULE_INFO_LENGTH];
static void read_eeprom_CamInfo(void)
{
	kal_uint16 idx = 0;
	kal_uint8 get_byte[12];
	for (idx = 0; idx <12; idx++) {
		char pusendcmd[2] = {0x00 , (char)((0x00 + idx) & 0xFF) };
		iReadRegI2C(pusendcmd , 2, (u8*)&get_byte[idx],1, 0xA2);
		LOG_INF("OV48B_info[%d]: 0x%x\n", idx, get_byte[idx]);
	}

	gGC8054_CamInfo[0] = get_byte[0];
	gGC8054_CamInfo[1] = get_byte[1];
	gGC8054_CamInfo[2] = get_byte[6];
	gGC8054_CamInfo[3] = get_byte[7];
	gGC8054_CamInfo[4] = get_byte[8];
	gGC8054_CamInfo[5] = get_byte[9];
	gGC8054_CamInfo[6] = get_byte[10];
	gGC8054_CamInfo[7] = get_byte[11];
}

#define   WRITE_DATA_MAX_LENGTH     (16)
static kal_int32 table_write_eeprom_30Bytes(kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	kal_int32 ret = IMGSENSOR_RETURN_SUCCESS;
	char pusendcmd[WRITE_DATA_MAX_LENGTH+2];
	pusendcmd[0] = (char)(addr >> 8);
	pusendcmd[1] = (char)(addr & 0xFF);

	memcpy(&pusendcmd[2], para, len);

	ret = iBurstWriteReg((kal_uint8 *)pusendcmd , (len + 2), 0xA2);

	return ret;
}

static kal_uint16 read_cmos_eeprom_8(kal_uint16 addr)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 1, 0xA2);
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

	ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, 0xA2);
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
		pr_debug("gc8054 SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
                       pStereodata->uSensorId,
                       pStereodata->uDeviceId,
                       pStereodata->baseAddr,
                       pStereodata->dataLength);

		data_base = pStereodata->baseAddr;
		data_length = pStereodata->dataLength;
		pData = pStereodata->uData;
		if ((pStereodata->uSensorId == GC8054_SENSOR_ID)
				&& (data_base == GC8054_STEREO_START_ADDR)
				&& (data_length == DUALCAM_CALI_DATA_LENGTH)) {
			pr_debug("gc8054 Write: %x %x %x %x %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556],
					pData[1557], pData[1558], pData[1559], pData[1560]);
			/* close write protect */
			write_eeprom_protect(0);
			msleep(6);
			idx = data_length/WRITE_DATA_MAX_LENGTH;
			idy = data_length%WRITE_DATA_MAX_LENGTH;
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
			pr_debug("com_0:0x%x\n", read_cmos_eeprom_8(GC8054_STEREO_START_ADDR));
			pr_debug("com_39:0x%x\n", read_cmos_eeprom_8(GC8054_STEREO_START_ADDR+39));
			pr_debug("innal_40:0x%x\n", read_cmos_eeprom_8(GC8054_STEREO_START_ADDR+40));
			pr_debug("innal_1556:0x%x\n", read_cmos_eeprom_8(GC8054_STEREO_START_ADDR+1556));
			pr_debug("tail1_1557:0x%x\n", read_cmos_eeprom_8(GC8054_STEREO_START_ADDR+1557));
			pr_debug("tail2_1558:0x%x\n", read_cmos_eeprom_8(GC8054_STEREO_START_ADDR+1558));
			pr_debug("tail3_1559:0x%x\n", read_cmos_eeprom_8(GC8054_STEREO_START_ADDR+1559));
			pr_debug("tail4_1560:0x%x\n", read_cmos_eeprom_8(GC8054_STEREO_START_ADDR+1560));
			pr_debug("gc8054write_Module_data Write end\n");
		} else if ((pStereodata->uSensorId == GC8054_SENSOR_ID)
			&& (data_base == GC8054_AESYNC_START_ADDR)
			&& (data_length < AESYNC_DATA_LENGTH_TOTAL)) {
			pr_debug("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
				pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);
			/* close write protect */
			write_eeprom_protect(0);
			msleep(6);
			idx = data_length/WRITE_DATA_MAX_LENGTH;
			idy = data_length%WRITE_DATA_MAX_LENGTH;
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
			/* open write protect */
			write_eeprom_protect(1);
			msleep(6);
			if (ret != IMGSENSOR_RETURN_SUCCESS) {
				pr_err("write TELE_aesync_eeprom error\n");
				return IMGSENSOR_RETURN_ERROR;
			}
			pr_debug("readback main aesync: %x %x %x %x %x %x %x %x\n"
				, read_cmos_eeprom_8(GC8054_AESYNC_START_ADDR)
				, read_cmos_eeprom_8(GC8054_AESYNC_START_ADDR+1)
				, read_cmos_eeprom_8(GC8054_AESYNC_START_ADDR+2)
				, read_cmos_eeprom_8(GC8054_AESYNC_START_ADDR+3)
				, read_cmos_eeprom_8(GC8054_AESYNC_START_ADDR+4)
				, read_cmos_eeprom_8(GC8054_AESYNC_START_ADDR+5)
				, read_cmos_eeprom_8(GC8054_AESYNC_START_ADDR+6)
				, read_cmos_eeprom_8(GC8054_AESYNC_START_ADDR+7));
		} else {
			pr_err("Invalid Sensor id:0x%x write_gm1 eeprom\n", pStereodata->uSensorId);
			return IMGSENSOR_RETURN_ERROR;
		}
	} else {
		pr_err("gc8054write_Module_data pStereodata is null\n");
		return IMGSENSOR_RETURN_ERROR;
	}
	return ret;
}
#endif


static void set_dummy(void)
{
	LOG_INF("frame length = %d\n", imgsensor.frame_length);
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if(imgsensor.frame_length < 0xfffe)
	#endif
		write_cmos_sensor_16(0x0340, imgsensor.frame_length & 0xfffe);
}

static kal_uint32 return_sensor_id(void)
{
	kal_uint32 sensor_id = 0;

	sensor_id = (read_cmos_sensor(0x03f0) << 8) | read_cmos_sensor(0x03f1);
	return sensor_id;
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
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	/*kal_uint32 frame_length = 0;*/
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	kal_uint32 exp_line = 0;
	kal_uint16 long_exp_h = 0;
	kal_uint16 long_exp_m = 0;
	kal_uint16 long_exp_l = 0;
	#endif
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	LOG_INF("E\n");

	/* if shutter bigger than frame_length, should extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	#ifndef OPLUS_FEATURE_CAMERA_COMMON
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

	if (imgsensor.autoflicker_en) {
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else
			set_max_framerate(realtime_fps, 0);
	} else
		set_max_framerate(realtime_fps, 0);

	/* Update Shutter */
	write_cmos_sensor_16(0x0202, shutter & 0xffff);
	#else
	if (shutter >= 0xFFEE) {
		bNeedSetNormalMode = KAL_TRUE;

		if (shutter >= 2304873) {  //>30s 30000000/13.0159 =2304873
			shutter = 2304873;
		}

		exp_line = (shutter - 0xA00) / 4 - 1;
		long_exp_h = (exp_line >> 16) & 0xF;
		long_exp_m = (exp_line >> 8) & 0xFF;
		long_exp_l = exp_line & 0xFF;
		//write_cmos_sensor(0x0218,0x06);
		//write_cmos_sensor(0x0282,0x18);
		write_cmos_sensor(0x0549,long_exp_h);
		write_cmos_sensor(0x054a,long_exp_m);
		write_cmos_sensor(0x054b,long_exp_l);
		write_cmos_sensor(0x0282,0x18);
		write_cmos_sensor(0x0282,0x10);
		write_cmos_sensor(0x0202,0x0a);
		write_cmos_sensor(0x0203,0x00);
		write_cmos_sensor(0x0340,0x0a);
		write_cmos_sensor(0x0341,0x10);
		write_cmos_sensor(0x0218,0x06);

		imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		imgsensor.current_ae_effective_frame = 2;
	} else {
	    if (bNeedSetNormalMode) {
			LOG_INF("exit long shutter\n");
			write_cmos_sensor(0x0311, 0x28);
			write_cmos_sensor(0x031d, 0x01);
			write_cmos_sensor(0x0218, 0x02);
			write_cmos_sensor(0x031d, 0x00);
			write_cmos_sensor(0x0202, 0x0A);
			write_cmos_sensor(0x0203, 0x00);
			write_cmos_sensor(0x0340, 0x0A);
			write_cmos_sensor(0x0341, 0x10);
			write_cmos_sensor(0x0549, 0x00);
			write_cmos_sensor(0x054A, 0x00);
			write_cmos_sensor(0x054B, 0x00);

			bNeedSetNormalMode = KAL_FALSE;
		}

		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

		if (imgsensor.autoflicker_en) {
			if (realtime_fps >= 297 && realtime_fps <= 305)
				set_max_framerate(296, 0);
			else if (realtime_fps >= 147 && realtime_fps <= 150)
				set_max_framerate(146, 0);
			else
				set_max_framerate(realtime_fps, 0);
		} else
			set_max_framerate(realtime_fps, 0);


		write_cmos_sensor_16(0x0202, shutter & 0xffff);

		imgsensor.current_ae_effective_frame = 2;

	}
	#endif

	LOG_INF("shutter = %d, framelength = %d\n",shutter, imgsensor.frame_length);
}

static void set_shutter_frame_length(kal_uint32 shutter, kal_uint32 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	kal_uint32 exp_line = 0;
	kal_uint16 long_exp_h = 0;
	kal_uint16 long_exp_m = 0;
	kal_uint16 long_exp_l = 0;
	#endif
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
	#ifndef OPLUS_FEATURE_CAMERA_COMMON
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

	/* Update Shutter */
	write_cmos_sensor_16(0x0202, shutter & 0xffff);
	#else
	if (shutter >= 0xFFEE) {
		bNeedSetNormalMode = KAL_TRUE;

		if (shutter >= 1229266) {  //>16s
			shutter = 1229266;
		}

		exp_line = (shutter - 0xA00) / 4 - 1;
		long_exp_h = (exp_line >> 16) & 0xF;
		long_exp_m = (exp_line >> 8) & 0xFF;
		long_exp_l = exp_line & 0xFF;
		//write_cmos_sensor(0x0218,0x06);
		//write_cmos_sensor(0x0282,0x18);
		write_cmos_sensor(0x0549,long_exp_h);
		write_cmos_sensor(0x054a,long_exp_m);
		write_cmos_sensor(0x054b,long_exp_l);
		write_cmos_sensor(0x0282,0x18);
		write_cmos_sensor(0x0282,0x10);
		write_cmos_sensor(0x0202,0x0a);
		write_cmos_sensor(0x0203,0x00);
		write_cmos_sensor(0x0340,0x0a);
		write_cmos_sensor(0x0341,0x10);
		write_cmos_sensor(0x0218,0x06);

		imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		imgsensor.current_ae_effective_frame = 2;
	} else {
	    if (bNeedSetNormalMode) {
			LOG_INF("exit long shutter\n");
			write_cmos_sensor(0x0311, 0x28);
			write_cmos_sensor(0x031d, 0x01);
			write_cmos_sensor(0x0218, 0x02);
			write_cmos_sensor(0x031d, 0x00);
			write_cmos_sensor(0x0202, 0x0A);
			write_cmos_sensor(0x0203, 0x00);
			write_cmos_sensor(0x0340, 0x0A);
			write_cmos_sensor(0x0341, 0x10);
			write_cmos_sensor(0x0549, 0x00);
			write_cmos_sensor(0x054A, 0x00);
			write_cmos_sensor(0x054B, 0x00);

			bNeedSetNormalMode = KAL_FALSE;
		}

		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

		if (imgsensor.autoflicker_en) {
			if (realtime_fps >= 297 && realtime_fps <= 305)
				set_max_framerate(296, 0);
			else if (realtime_fps >= 147 && realtime_fps <= 150)
				set_max_framerate(146, 0);
			else
				set_max_framerate(realtime_fps, 0);
		} else
			set_max_framerate(realtime_fps, 0);

		write_cmos_sensor_16(0x0202, shutter & 0xffff);

		imgsensor.current_ae_effective_frame = 2;

	}
	#endif

	LOG_INF("shutter = %d, framelength = %d\n",shutter, imgsensor.frame_length);
}

static kal_uint16 gain2reg(kal_uint16 gain)
{
	kal_uint16 reg_gain = gain << 4;

	reg_gain = (reg_gain < SENSOR_BASE_GAIN) ? SENSOR_BASE_GAIN : reg_gain;
	reg_gain = (reg_gain > SENSOR_MAX_GAIN) ? SENSOR_MAX_GAIN : reg_gain;

	return reg_gain;
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
	kal_uint32 reg_gain = 0;

	LOG_INF("E\n");
	reg_gain = gain2reg(gain);
	LOG_INF("gain = %d, reg_gain = %d\n", gain, reg_gain);
	write_cmos_sensor_16(0x0204, reg_gain & 0xffff);
	return gain;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor(0x0102, 0x09);
		mDELAY(10);
	} else {
		write_cmos_sensor(0x0102, 0x00);
		mDELAY(10);
	}
	return ERROR_NONE;
}

static kal_uint16 addr_data_pair_global[] = {
	0x031c, 0x60,
	0x0320, 0xbb,
	0x0337, 0x05,
	0x0335, 0x51,
	0x0336, 0x8a,
	0x031a, 0x00,
	0x0321, 0x10,
	0x0327, 0x05,
	0x0325, 0x41,
	0x0326, 0x54,
	0x0314, 0x01,
	0x0315, 0xe9,
	0x0317, 0x00,
	0x0115, 0x10,
	0x0180, 0x69,
	0x0334, 0xc0,
	0x0324, 0x44,
	0x031c, 0x00,
	0x031c, 0x9f,
	0x0288, 0x03,
	0x0084, 0x30,
	0x0265, 0x00,
	0x04e0, 0x01,
	0x0100, 0x01,
	0x0101, 0x03,
	0x0342, 0x05,
	0x0343, 0x20,
	0x0344, 0x00,
	0x0345, 0x06,
	0x0346, 0x00,
	0x0347, 0x04,
	0x0348, 0x0c,
	0x0349, 0xd0,
	0x034a, 0x09,
	0x034b, 0x9c,
	0x0257, 0x06,
	0x0290, 0x00,
	0x0291, 0x00,
	0x0292, 0x28,
	0x0295, 0x10,
	0x0213, 0x12,
	0x02a9, 0x18,
	0x0221, 0x22,
	0x028b, 0x18,
	0x028c, 0x18,
	0x0229, 0x64,
	0x024b, 0x16,
	0x0255, 0x21,
	0x0280, 0x38,
	0x0500, 0x18,
	0x0501, 0xd0,
	0x0502, 0x15,
	0x0211, 0x02,
	0x0216, 0x06,
	0x0219, 0x08,
	0x021a, 0x04,
	0x021f, 0x10,
	0x0224, 0x00,
	0x0234, 0x00,
	0x0220, 0x15,
	0x024a, 0x02,
	0x0281, 0x30,
	0x0282, 0x13,
	0x028d, 0x92,
	0x028f, 0x01,
	0x0390, 0x6f,
	0x0392, 0x5c,
	0x0394, 0x20,
	0x039a, 0x90,
	0x0506, 0x19,
	0x0514, 0x00,
	0x0515, 0x1c,
	0x031c, 0x80,
	0x03fe, 0x10,
	0x03fe, 0x00,
	0x031c, 0x9f,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0x80,
	0x03fe, 0x10,
	0x03fe, 0x00,
	0x031c, 0x9f,
	0x0360, 0x01,
	0x0360, 0x00,
	0x0317, 0x08,
	0x0a67, 0x80,
	0x0313, 0x00,
	0x0a54, 0x04,
	0x0a65, 0x05,
	0x0a68, 0x11,
	0x0a59, 0x00,
	0x00be, 0x00,
	0x00a9, 0x01,
	0x00d7, 0xd0,
	0x00d8, 0x0c,
	0x009c, 0x09,
	0x009d, 0x9c,
	0x009e, 0x0c,
	0x009f, 0xd0,
	0x0a82, 0x00,
	0x0a83, 0x88,
	0x0a84, 0x10,
	0x0a85, 0x10,
	0x0a88, 0x0a,
	0x0a89, 0x0d,
	0x0a8a, 0x20,
	0x0a71, 0x52,
	0x0a72, 0x12,
	0x0a73, 0x60,
	0x0a75, 0x41,
	0x0a70, 0x07,
	0x0313, 0x80,
	0x0080, 0xd0,
	0x0087, 0x51,
	0x0089, 0x03,
	0x0096, 0x82,
	0x0040, 0x22,
	0x0041, 0x20,
	0x0043, 0x05,
	0x0044, 0x00,
	0x0046, 0x08,
	0x0048, 0x01,
	0x0049, 0xf0,
	0x004a, 0x0f,
	0x004c, 0x0f,
	0x004d, 0x00,
	0x0414, 0x80,
	0x0415, 0x80,
	0x0416, 0x80,
	0x0417, 0x80,
	0x009a, 0x40,
	0x00c0, 0x80,
	0x00c1, 0x80,
	0x00c2, 0x02,
	0x0470, 0x04,
	0x0471, 0x1c,
	0x0472, 0x14,
	0x0473, 0x12,
	0x0474, 0x1a,
	0x0475, 0x1a,
	0x0476, 0x18,
	0x0477, 0x16,
	0x0480, 0x03,
	0x0481, 0x03,
	0x0482, 0x03,
	0x0483, 0x03,
	0x0484, 0x04,
	0x0485, 0x04,
	0x0486, 0x04,
	0x0487, 0x04,
	0x0478, 0x04,
	0x0479, 0x10,
	0x047a, 0x26,
	0x047b, 0x38,
	0x047c, 0x10,
	0x047d, 0x20,
	0x047e, 0x30,
	0x047f, 0x60,
	0x0488, 0x04,
	0x0489, 0x04,
	0x048a, 0x04,
	0x048b, 0x04,
	0x048c, 0x03,
	0x048d, 0x03,
	0x048e, 0x03,
	0x048f, 0x03,
};

static void sensor_init(void)
{
	LOG_INF("sensor_init:fixel_pixel: %d\n", imgsensor_info.sensor_output_dataformat);
	table_write_cmos_sensor(addr_data_pair_global,
		sizeof(addr_data_pair_global) / sizeof(kal_uint16)); //Global & Functional
}

static kal_uint16 addr_data_pair_preview[] = {
	0x031c,0x60,
	0x0320,0xbb,
	0x0337,0x06,
	0x0335,0x51,
	0x0336,0xac,
	0x031a,0x00,
	0x0321,0x10,
	0x0327,0x05,
	0x0325,0x41,
	0x0326,0x54,
	0x0314,0x01,
	0x0315,0xe9,
	0x0317,0x00,
	0x0115,0x10,
	0x0180,0x69,
	0x0334,0xc0,
	0x0324,0x44,
	0x031c,0x00,
	0x031c,0x9f,
	0x0202,0x09,
	0x0203,0x00,
	0x0340,0x09,
	0x0341,0xe4,
	0x0342,0x05,
	0x0343,0x20,
	0x0344,0x00,
	0x0345,0x06,
	0x0346,0x00,
	0x0347,0x04,
	0x0348,0x0c,
	0x0349,0xd0,
	0x034a,0x09,
	0x034b,0x9c,
	0x0291,0x00,
	0x0292,0x28,
	0x0213,0x12,
	0x02a9,0x18,
	0x0221,0x22,
	0x028b,0x18,
	0x028c,0x18,
	0x0229,0x64,
	0x024b,0x16,
	0x0255,0x21,
	0x0280,0x38,
	0x021f,0x10,
	0x0224,0x00,
	0x0234,0x00,
	0x024a,0x02,
	0x0282,0x13,
	0x028d,0x92,
	0x039a,0x90,
	0x031c,0x80,
	0x03fe,0x10,
	0x03fe,0x00,
	0x031c,0x9f,
	0x03fe,0x00,
	0x03fe,0x00,
	0x03fe,0x00,
	0x03fe,0x00,
	0x031c,0x80,
	0x03fe,0x10,
	0x03fe,0x00,
	0x031c,0x9f,
	0x00d3,0x00,
	0x00d4,0x00,
	0x00d5,0x00,
	0x00d6,0x00,
	0x05a0,0x83,
	0x05a3,0x06,
	0x05a4,0x04,
	0x0597,0x27,
	0x059a,0x00,
	0x059b,0x00,
	0x059c,0x01,
	0x05ab,0x09,
	0x05ae,0x00,
	0x05af,0x00,
	0x05ac,0x00,
	0x05ad,0x01,
	0x05b1,0x03,
	0x05b1,0x04,
	0x05b1,0x05,
	0x05b1,0x0c,
	0x05b1,0x1c,
	0x05b1,0x05,
	0x05b1,0x12,
	0x05b1,0x05,
	0x05b1,0x00,
	0x05b1,0x05,
	0x05b1,0x06,
	0x05b1,0x05,
	0x05b1,0x14,
	0x05b1,0x18,
	0x05b1,0x18,
	0x05b1,0x19,
	0x05b1,0x40,
	0x05b1,0x14,
	0x05b1,0x38,
	0x05b1,0x29,
	0x05b1,0x40,
	0x05b1,0x08,
	0x05b1,0x38,
	0x05b1,0x29,
	0x05b1,0x40,
	0x05b1,0x08,
	0x05b1,0x38,
	0x05b1,0x29,
	0x05b1,0x00,
	0x05b1,0x08,
	0x05b1,0x68,
	0x05b1,0x41,
	0x05b1,0x00,
	0x05b1,0x08,
	0x05b1,0x88,
	0x05b1,0x51,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x01,
	0x05b1,0x00,
	0x05b1,0x01,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x01,
	0x05b1,0x94,
	0x05b1,0x03,
	0x05b1,0x01,
	0x05b1,0x02,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x02,
	0x05b1,0x02,
	0x05b1,0xd9,
	0x05b1,0x00,
	0x05b1,0x03,
	0x05b1,0x03,
	0x05b1,0xf8,
	0x05b1,0x00,
	0x05b1,0x04,
	0x05b1,0x05,
	0x05b1,0x9e,
	0x05b1,0x00,
	0x05b1,0x05,
	0x05b1,0x07,
	0x05b1,0xf0,
	0x05b1,0x00,
	0x05b1,0x0c,
	0x05b1,0x0b,
	0x05b1,0x3d,
	0x05b1,0x00,
	0x05b1,0x0d,
	0x05b1,0x0f,
	0x05b1,0xc6,
	0x05b1,0x09,
	0x05b1,0x6d,
	0x05ac,0x01,
	0x029f,0xc4,
	0x05a0,0xc3,
	0x02b0,0x70,
	0x0206,0xc0,
	0x02b3,0x00,
	0x02b4,0x00,
	0x0204,0x04,
	0x0205,0x00,
	0x0099,0x00,
	0x0351,0x00,
	0x0352,0x06,
	0x0353,0x00,
	0x0354,0x08,
	0x034c,0x0c,
	0x034d,0xc0,
	0x034e,0x09,
	0x034f,0x90,
	0x0108,0x08,
	0x0112,0x0a,
	0x0113,0x0a,
	0x0114,0x01,
	0x0181,0x30,
	0x0185,0x01,
	0x0188,0x00,
	0x0121,0x0b,
	0x0122,0x0d,
	0x0123,0x2f,
	0x0124,0x01,
	0x0125,0x12,
	0x0126,0x0f,
	0x0129,0x0c,
	0x012a,0x13,
	0x012b,0x0f,
	0x0a70,0x11,
	0x0313,0x80,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0a70,0x00,
	0x00be,0x01,
	0x0317,0x00,
	0x0a67,0x00,
	0x0084,0x10,
};

static void preview_setting(void)
{
	LOG_INF("preview_setting Start!\n");
	table_write_cmos_sensor(addr_data_pair_preview,
		sizeof(addr_data_pair_preview) / sizeof(kal_uint16)); //Global & Functional
	LOG_INF("preview_setting End!\n");
}

static kal_uint16 addr_data_pair_capture[] = {
	0x031c,0x60,
	0x0320,0xbb,
	0x0337,0x06,
	0x0335,0x51,
	0x0336,0xac,
	0x031a,0x00,
	0x0321,0x10,
	0x0327,0x05,
	0x0325,0x41,
	0x0326,0x54,
	0x0314,0x01,
	0x0315,0xe9,
	0x0317,0x00,
	0x0115,0x10,
	0x0180,0x69,
	0x0334,0xc0,
	0x0324,0x44,
	0x031c,0x00,
	0x031c,0x9f,
	0x0202,0x09,
	0x0203,0x00,
	0x0340,0x09,
	0x0341,0xe4,
	0x0342,0x05,
	0x0343,0x20,
	0x0344,0x00,
	0x0345,0x06,
	0x0346,0x00,
	0x0347,0x04,
	0x0348,0x0c,
	0x0349,0xd0,
	0x034a,0x09,
	0x034b,0x9c,
	0x0291,0x00,
	0x0292,0x28,
	0x0213,0x12,
	0x02a9,0x18,
	0x0221,0x22,
	0x028b,0x18,
	0x028c,0x18,
	0x0229,0x64,
	0x024b,0x16,
	0x0255,0x21,
	0x0280,0x38,
	0x021f,0x10,
	0x0224,0x00,
	0x0234,0x00,
	0x024a,0x02,
	0x0282,0x13,
	0x028d,0x92,
	0x039a,0x90,
	0x031c,0x80,
	0x03fe,0x10,
	0x03fe,0x00,
	0x031c,0x9f,
	0x03fe,0x00,
	0x03fe,0x00,
	0x03fe,0x00,
	0x03fe,0x00,
	0x031c,0x80,
	0x03fe,0x10,
	0x03fe,0x00,
	0x031c,0x9f,
	0x00d3,0x00,
	0x00d4,0x00,
	0x00d5,0x00,
	0x00d6,0x00,
	0x05a0,0x83,
	0x05a3,0x06,
	0x05a4,0x04,
	0x0597,0x27,
	0x059a,0x00,
	0x059b,0x00,
	0x059c,0x01,
	0x05ab,0x09,
	0x05ae,0x00,
	0x05af,0x00,
	0x05ac,0x00,
	0x05ad,0x01,
	0x05b1,0x03,
	0x05b1,0x04,
	0x05b1,0x05,
	0x05b1,0x0c,
	0x05b1,0x1c,
	0x05b1,0x05,
	0x05b1,0x12,
	0x05b1,0x05,
	0x05b1,0x00,
	0x05b1,0x05,
	0x05b1,0x06,
	0x05b1,0x05,
	0x05b1,0x14,
	0x05b1,0x18,
	0x05b1,0x18,
	0x05b1,0x19,
	0x05b1,0x40,
	0x05b1,0x14,
	0x05b1,0x38,
	0x05b1,0x29,
	0x05b1,0x40,
	0x05b1,0x08,
	0x05b1,0x38,
	0x05b1,0x29,
	0x05b1,0x40,
	0x05b1,0x08,
	0x05b1,0x38,
	0x05b1,0x29,
	0x05b1,0x00,
	0x05b1,0x08,
	0x05b1,0x68,
	0x05b1,0x41,
	0x05b1,0x00,
	0x05b1,0x08,
	0x05b1,0x88,
	0x05b1,0x51,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x01,
	0x05b1,0x00,
	0x05b1,0x01,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x01,
	0x05b1,0x94,
	0x05b1,0x03,
	0x05b1,0x01,
	0x05b1,0x02,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x02,
	0x05b1,0x02,
	0x05b1,0xd9,
	0x05b1,0x00,
	0x05b1,0x03,
	0x05b1,0x03,
	0x05b1,0xf8,
	0x05b1,0x00,
	0x05b1,0x04,
	0x05b1,0x05,
	0x05b1,0x9e,
	0x05b1,0x00,
	0x05b1,0x05,
	0x05b1,0x07,
	0x05b1,0xf0,
	0x05b1,0x00,
	0x05b1,0x0c,
	0x05b1,0x0b,
	0x05b1,0x3d,
	0x05b1,0x00,
	0x05b1,0x0d,
	0x05b1,0x0f,
	0x05b1,0xc6,
	0x05b1,0x09,
	0x05b1,0x6d,
	0x05ac,0x01,
	0x029f,0xc4,
	0x05a0,0xc3,
	0x02b0,0x70,
	0x0206,0xc0,
	0x02b3,0x00,
	0x02b4,0x00,
	0x0204,0x04,
	0x0205,0x00,
	0x0099,0x00,
	0x0351,0x00,
	0x0352,0x06,
	0x0353,0x00,
	0x0354,0x08,
	0x034c,0x0c,
	0x034d,0xc0,
	0x034e,0x09,
	0x034f,0x90,
	0x0108,0x08,
	0x0112,0x0a,
	0x0113,0x0a,
	0x0114,0x01,
	0x0181,0x30,
	0x0185,0x01,
	0x0188,0x00,
	0x0121,0x0b,
	0x0122,0x0d,
	0x0123,0x2f,
	0x0124,0x01,
	0x0125,0x12,
	0x0126,0x0f,
	0x0129,0x0c,
	0x012a,0x13,
	0x012b,0x0f,
	0x0a70,0x11,
	0x0313,0x80,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0a70,0x00,
	0x00be,0x01,
	0x0317,0x00,
	0x0a67,0x00,
	0x0084,0x10,
};

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("capture_setting:%d Start!\n", currefps);
	table_write_cmos_sensor(addr_data_pair_capture,
		sizeof(addr_data_pair_capture) / sizeof(kal_uint16)); //capture
	LOG_INF("capture_setting End!\n");
}

static kal_uint16 addr_data_pair_normal_video[] = {
	0x031c, 0x60,
	0x0320, 0xbb,
	0x0337, 0x06,
	0x0335, 0x51,
	0x0336, 0xac,
	0x031a, 0x00,
	0x0321, 0x10,
	0x0327, 0x05,
	0x0325, 0x41,
	0x0326, 0x54,
	0x0314, 0x01,
	0x0315, 0xe9,
	0x0317, 0x00,
	0x0115, 0x10,
	0x0180, 0x69,
	0x0334, 0xc0,
	0x0324, 0x44,
	0x031c, 0x00,
	0x031c, 0x9f,
	0x0202, 0x09,
	0x0203, 0x00,
	0x0340, 0x09,
	0x0341, 0xe4,
	0x0342, 0x05,
	0x0343, 0x20,
	0x0344, 0x00,
	0x0345, 0x06,
	0x0346, 0x01,
	0x0347, 0x36,
	0x0348, 0x0c,
	0x0349, 0xd0,
	0x034a, 0x07,
	0x034b, 0x38,
	0x0291, 0x02,
	0x0292, 0x8c,
	0x0213, 0x12,
	0x02a9, 0x18,
	0x0221, 0x22,
	0x028b, 0x18,
	0x028c, 0x18,
	0x0229, 0x64,
	0x024b, 0x16,
	0x0255, 0x21,
	0x0280, 0x38,
	0x021f, 0x10,
	0x0224, 0x00,
	0x0234, 0x00,
	0x024a, 0x02,
	0x0282, 0x13,
	0x028d, 0x92,
	0x039a, 0x90,
	0x031c, 0x80,
	0x03fe, 0x10,
	0x03fe, 0x00,
	0x031c, 0x9f,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0x80,
	0x03fe, 0x10,
	0x03fe, 0x00,
	0x031c, 0x9f,
	0x00d3, 0x32,
	0x00d4, 0x01,
	0x00d5, 0x00,
	0x00d6, 0x00,
	0x05a0, 0x83,
	0x05a3, 0x06,
	0x05a4, 0x04,
	0x0597, 0x27,
	0x059a, 0x00,
	0x059b, 0x00,
	0x059c, 0x01,
	0x05ab, 0x09,
	0x05ae, 0x00,
	0x05af, 0x00,
	0x05ac, 0x00,
	0x05ad, 0x01,
	0x05b1, 0x03,
	0x05b1, 0x04,
	0x05b1, 0x05,
	0x05b1, 0x0c,
	0x05b1, 0x1c,
	0x05b1, 0x05,
	0x05b1, 0x12,
	0x05b1, 0x05,
	0x05b1, 0x00,
	0x05b1, 0x05,
	0x05b1, 0x06,
	0x05b1, 0x05,
	0x05b1, 0x14,
	0x05b1, 0x18,
	0x05b1, 0x18,
	0x05b1, 0x19,
	0x05b1, 0x40,
	0x05b1, 0x14,
	0x05b1, 0x38,
	0x05b1, 0x29,
	0x05b1, 0x40,
	0x05b1, 0x08,
	0x05b1, 0x38,
	0x05b1, 0x29,
	0x05b1, 0x40,
	0x05b1, 0x08,
	0x05b1, 0x38,
	0x05b1, 0x29,
	0x05b1, 0x00,
	0x05b1, 0x08,
	0x05b1, 0x68,
	0x05b1, 0x41,
	0x05b1, 0x00,
	0x05b1, 0x08,
	0x05b1, 0x88,
	0x05b1, 0x51,
	0x05b1, 0x00,
	0x05b1, 0x00,
	0x05b1, 0x00,
	0x05b1, 0x01,
	0x05b1, 0x00,
	0x05b1, 0x01,
	0x05b1, 0x00,
	0x05b1, 0x00,
	0x05b1, 0x00,
	0x05b1, 0x01,
	0x05b1, 0x94,
	0x05b1, 0x03,
	0x05b1, 0x01,
	0x05b1, 0x02,
	0x05b1, 0x00,
	0x05b1, 0x00,
	0x05b1, 0x02,
	0x05b1, 0x02,
	0x05b1, 0xd9,
	0x05b1, 0x00,
	0x05b1, 0x03,
	0x05b1, 0x03,
	0x05b1, 0xf8,
	0x05b1, 0x00,
	0x05b1, 0x04,
	0x05b1, 0x05,
	0x05b1, 0x9e,
	0x05b1, 0x00,
	0x05b1, 0x05,
	0x05b1, 0x07,
	0x05b1, 0xf0,
	0x05b1, 0x00,
	0x05b1, 0x0c,
	0x05b1, 0x0b,
	0x05b1, 0x3d,
	0x05b1, 0x00,
	0x05b1, 0x0d,
	0x05b1, 0x0f,
	0x05b1, 0xc6,
	0x05b1, 0x09,
	0x05b1, 0x6d,
	0x05ac, 0x01,
	0x029f, 0xc4,
	0x05a0, 0xc3,
	0x02b0, 0x70,
	0x0206, 0xc0,
	0x02b3, 0x00,
	0x02b4, 0x00,
	0x0204, 0x04,
	0x0205, 0x00,
	0x0099, 0x00,
	0x0351, 0x00,
	0x0352, 0x06,
	0x0353, 0x00,
	0x0354, 0x08,
	0x034c, 0x0c,
	0x034d, 0xc0,
	0x034e, 0x07,
	0x034f, 0x2c,
	0x0108, 0x08,
	0x0112, 0x0a,
	0x0113, 0x0a,
	0x0114, 0x01,
	0x0181, 0x30,
	0x0185, 0x01,
	0x0188, 0x00,
	0x0121, 0x0b,
	0x0122, 0x0d,
	0x0123, 0x2f,
	0x0124, 0x01,
	0x0125, 0x12,
	0x0126, 0x0f,
	0x0129, 0x0c,
	0x012a, 0x13,
	0x012b, 0x0f,
	0x0a70, 0x11,
	0x0313, 0x80,
	0x0aff, 0x10,
	0x0aff, 0x10,
	0x0aff, 0x10,
	0x0aff, 0x10,
	0x0aff, 0x10,
	0x0aff, 0x10,
	0x0aff, 0x10,
	0x0aff, 0x10,
	0x0a70, 0x00,
	0x00be, 0x01,
	0x0317, 0x00,
	0x0a67, 0x00,
	0x0084, 0x10,
};

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("normal_video_setting:%d Start!\n", currefps);
	table_write_cmos_sensor(addr_data_pair_normal_video,
		sizeof(addr_data_pair_normal_video) / sizeof(kal_uint16)); //Global & Functional
	LOG_INF("normal_video_setting End!\n");
}

static kal_uint16 addr_data_pair_hs_video[] = {
	0x031c,0x60,
	0x0320,0xbb,
	0x0337,0x05,
	0x0335,0x59,
	0x0336,0x92,
	0x031a,0x01,
	0x0321,0x10,
	0x0327,0x05,
	0x0325,0x43,
	0x0326,0x54,
	0x0314,0x01,
	0x0315,0xe4,
	0x0317,0x00,
	0x0115,0x10,
	0x0180,0x69,
	0x0334,0xc0,
	0x0324,0x44,
	0x031c,0x00,
	0x031c,0x9f,
	0x0202,0x04,
	0x0203,0x80,
	0x0340,0x04,
	0x0341,0xf2,
	0x0342,0x05,
	0x0343,0x20,
	0x0344,0x00,
	0x0345,0x06,
	0x0346,0x00,
	0x0347,0x04,
	0x0348,0x0c,
	0x0349,0xd0,
	0x034a,0x09,
	0x034b,0x9c,
	0x0291,0x00,
	0x0292,0x28,
	0x0213,0x14,
	0x02a9,0x18,
	0x0221,0x22,
	0x028b,0x18,
	0x028c,0x18,
	0x0229,0x64,
	0x024b,0x16,
	0x0255,0x21,
	0x0280,0x38,
	0x021f,0x18,
	0x0224,0x41,
	0x0234,0x20,
	0x024a,0x04,
	0x0282,0x10,
	0x028d,0x93,
	0x039a,0xc0,
	0x031c,0x80,
	0x03fe,0x10,
	0x03fe,0x00,
	0x031c,0x9f,
	0x03fe,0x00,
	0x03fe,0x00,
	0x03fe,0x00,
	0x03fe,0x00,
	0x031c,0x80,
	0x03fe,0x10,
	0x03fe,0x00,
	0x031c,0x9f,
	0x00d3,0x00,
	0x00d4,0x00,
	0x00d5,0x00,
	0x00d6,0x00,
	0x05a0,0x83,
	0x05a3,0x06,
	0x05a4,0x04,
	0x0597,0x27,
	0x059a,0x00,
	0x059b,0x00,
	0x059c,0x01,
	0x05ab,0x09,
	0x05ae,0x00,
	0x05af,0x00,
	0x05ac,0x00,
	0x05ad,0x01,
	0x05b1,0x03,
	0x05b1,0x04,
	0x05b1,0x05,
	0x05b1,0x0c,
	0x05b1,0x1c,
	0x05b1,0x05,
	0x05b1,0x12,
	0x05b1,0x05,
	0x05b1,0x00,
	0x05b1,0x05,
	0x05b1,0x06,
	0x05b1,0x05,
	0x05b1,0x14,
	0x05b1,0x1c,
	0x05b1,0x18,
	0x05b1,0x19,
	0x05b1,0x40,
	0x05b1,0x18,
	0x05b1,0x38,
	0x05b1,0x29,
	0x05b1,0x40,
	0x05b1,0x08,
	0x05b1,0x38,
	0x05b1,0x29,
	0x05b1,0x40,
	0x05b1,0x08,
	0x05b1,0x38,
	0x05b1,0x29,
	0x05b1,0x00,
	0x05b1,0x08,
	0x05b1,0x68,
	0x05b1,0x41,
	0x05b1,0x00,
	0x05b1,0x08,
	0x05b1,0x88,
	0x05b1,0x51,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x01,
	0x05b1,0x00,
	0x05b1,0x01,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x01,
	0x05b1,0x94,
	0x05b1,0x03,
	0x05b1,0x01,
	0x05b1,0x02,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x02,
	0x05b1,0x02,
	0x05b1,0xd9,
	0x05b1,0x00,
	0x05b1,0x03,
	0x05b1,0x03,
	0x05b1,0xf8,
	0x05b1,0x00,
	0x05b1,0x04,
	0x05b1,0x05,
	0x05b1,0x9e,
	0x05b1,0x00,
	0x05b1,0x05,
	0x05b1,0x07,
	0x05b1,0xf0,
	0x05b1,0x00,
	0x05b1,0x0c,
	0x05b1,0x0b,
	0x05b1,0x3d,
	0x05b1,0x00,
	0x05b1,0x0d,
	0x05b1,0x0f,
	0x05b1,0xc6,
	0x05b1,0x09,
	0x05b1,0x6d,
	0x05ac,0x01,
	0x029f,0xc4,
	0x05a0,0xc3,
	0x02b0,0x70,
	0x0206,0xc0,
	0x02b3,0x00,
	0x02b4,0x00,
	0x0204,0x04,
	0x0205,0x00,
	0x0099,0x00,
	0x0351,0x00,
	0x0352,0x04,
	0x0353,0x00,
	0x0354,0x04,
	0x034c,0x06,
	0x034d,0x60,
	0x034e,0x04,
	0x034f,0xc8,
	0x0108,0x08,
	0x0112,0x0a,
	0x0113,0x0a,
	0x0114,0x01,
	0x0181,0x30,
	0x0185,0x01,
	0x0188,0x00,
	0x0121,0x02,
	0x0122,0x03,
	0x0123,0x09,
	0x0124,0x00,
	0x0125,0x08,
	0x0126,0x05,
	0x0129,0x03,
	0x012a,0x03,
	0x012b,0x05,
	0x0a70,0x11,
	0x0313,0x80,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0a70,0x00,
	0x00be,0x01,
	0x0317,0x00,
	0x0a67,0x00,
	0x0084,0x10,
};

static void hs_video_setting(void)
{
	LOG_INF("hs_video_setting Start!\n");
	table_write_cmos_sensor(addr_data_pair_hs_video,
		sizeof(addr_data_pair_hs_video) / sizeof(kal_uint16)); //hs_video
	LOG_INF("hs_video_setting End!\n");
}

static kal_uint16 addr_data_pair_slim_video[] = {
	0x031c,0x60,
	0x0320,0xbb,
	0x0337,0x05,
	0x0335,0x59,
	0x0336,0x92,
	0x031a,0x01,
	0x0321,0x10,
	0x0327,0x05,
	0x0325,0x43,
	0x0326,0x54,
	0x0314,0x01,
	0x0315,0xe4,
	0x0317,0x00,
	0x0115,0x10,
	0x0180,0x69,
	0x0334,0xc0,
	0x0324,0x44,
	0x031c,0x00,
	0x031c,0x9f,
	0x0202,0x04,
	0x0203,0x80,
	0x0340,0x04,
	0x0341,0xf2,
	0x0342,0x05,
	0x0343,0x20,
	0x0344,0x00,
	0x0345,0x06,
	0x0346,0x00,
	0x0347,0x04,
	0x0348,0x0c,
	0x0349,0xd0,
	0x034a,0x09,
	0x034b,0x9c,
	0x0291,0x00,
	0x0292,0x28,
	0x0213,0x14,
	0x02a9,0x18,
	0x0221,0x22,
	0x028b,0x18,
	0x028c,0x18,
	0x0229,0x64,
	0x024b,0x16,
	0x0255,0x21,
	0x0280,0x38,
	0x021f,0x18,
	0x0224,0x41,
	0x0234,0x20,
	0x024a,0x04,
	0x0282,0x10,
	0x028d,0x93,
	0x039a,0xc0,
	0x031c,0x80,
	0x03fe,0x10,
	0x03fe,0x00,
	0x031c,0x9f,
	0x03fe,0x00,
	0x03fe,0x00,
	0x03fe,0x00,
	0x03fe,0x00,
	0x031c,0x80,
	0x03fe,0x10,
	0x03fe,0x00,
	0x031c,0x9f,
	0x00d3,0x00,
	0x00d4,0x00,
	0x00d5,0x00,
	0x00d6,0x00,
	0x05a0,0x83,
	0x05a3,0x06,
	0x05a4,0x04,
	0x0597,0x27,
	0x059a,0x00,
	0x059b,0x00,
	0x059c,0x01,
	0x05ab,0x09,
	0x05ae,0x00,
	0x05af,0x00,
	0x05ac,0x00,
	0x05ad,0x01,
	0x05b1,0x03,
	0x05b1,0x04,
	0x05b1,0x05,
	0x05b1,0x0c,
	0x05b1,0x1c,
	0x05b1,0x05,
	0x05b1,0x12,
	0x05b1,0x05,
	0x05b1,0x00,
	0x05b1,0x05,
	0x05b1,0x06,
	0x05b1,0x05,
	0x05b1,0x14,
	0x05b1,0x1c,
	0x05b1,0x18,
	0x05b1,0x19,
	0x05b1,0x40,
	0x05b1,0x18,
	0x05b1,0x38,
	0x05b1,0x29,
	0x05b1,0x40,
	0x05b1,0x08,
	0x05b1,0x38,
	0x05b1,0x29,
	0x05b1,0x40,
	0x05b1,0x08,
	0x05b1,0x38,
	0x05b1,0x29,
	0x05b1,0x00,
	0x05b1,0x08,
	0x05b1,0x68,
	0x05b1,0x41,
	0x05b1,0x00,
	0x05b1,0x08,
	0x05b1,0x88,
	0x05b1,0x51,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x01,
	0x05b1,0x00,
	0x05b1,0x01,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x01,
	0x05b1,0x94,
	0x05b1,0x03,
	0x05b1,0x01,
	0x05b1,0x02,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x02,
	0x05b1,0x02,
	0x05b1,0xd9,
	0x05b1,0x00,
	0x05b1,0x03,
	0x05b1,0x03,
	0x05b1,0xf8,
	0x05b1,0x00,
	0x05b1,0x04,
	0x05b1,0x05,
	0x05b1,0x9e,
	0x05b1,0x00,
	0x05b1,0x05,
	0x05b1,0x07,
	0x05b1,0xf0,
	0x05b1,0x00,
	0x05b1,0x0c,
	0x05b1,0x0b,
	0x05b1,0x3d,
	0x05b1,0x00,
	0x05b1,0x0d,
	0x05b1,0x0f,
	0x05b1,0xc6,
	0x05b1,0x09,
	0x05b1,0x6d,
	0x05ac,0x01,
	0x029f,0xc4,
	0x05a0,0xc3,
	0x02b0,0x70,
	0x0206,0xc0,
	0x02b3,0x00,
	0x02b4,0x00,
	0x0204,0x04,
	0x0205,0x00,
	0x0099,0x00,
	0x0351,0x00,
	0x0352,0x04,
	0x0353,0x00,
	0x0354,0x04,
	0x034c,0x06,
	0x034d,0x60,
	0x034e,0x04,
	0x034f,0xc8,
	0x0108,0x08,
	0x0112,0x0a,
	0x0113,0x0a,
	0x0114,0x01,
	0x0181,0x30,
	0x0185,0x01,
	0x0188,0x00,
	0x0121,0x02,
	0x0122,0x03,
	0x0123,0x09,
	0x0124,0x00,
	0x0125,0x08,
	0x0126,0x05,
	0x0129,0x03,
	0x012a,0x03,
	0x012b,0x05,
	0x0a70,0x11,
	0x0313,0x80,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0a70,0x00,
	0x00be,0x01,
	0x0317,0x00,
	0x0a67,0x00,
	0x0084,0x10,
};

static void slim_video_setting(void)
{
	LOG_INF("slim_video_setting Start!\n");
	table_write_cmos_sensor(addr_data_pair_slim_video,
		sizeof(addr_data_pair_slim_video) / sizeof(kal_uint16)); //hs_video
	LOG_INF("slim_video_setting End!\n");
}

static kal_uint16 addr_data_pair_custom1[] = {
	0x031c,0x60,
	0x0320,0xbb,
	0x0337,0x05,
	0x0335,0x51,
	0x0336,0x8a,
	0x031a,0x00,
	0x0321,0x10,
	0x0327,0x05,
	0x0325,0x41,
	0x0326,0x54,
	0x0314,0x01,
	0x0315,0xe9,
	0x0317,0x00,
	0x0115,0x10,
	0x0180,0x69,
	0x0334,0xc0,
	0x0324,0x44,
	0x031c,0x00,
	0x031c,0x9f,
	0x0202,0x09,
	0x0203,0x00,
	0x0340,0x0c,
	0x0341,0x80,
	0x0342,0x05,
	0x0343,0x20,
	0x0344,0x01,
	0x0345,0xe6,
	0x0346,0x01,
	0x0347,0x6c,
	0x0348,0x09,
	0x0349,0x10,
	0x034a,0x06,
	0x034b,0xcc,
	0x0291,0x05,
	0x0292,0x94,
	0x0213,0x12,
	0x02a9,0x18,
	0x0221,0x22,
	0x028b,0x18,
	0x028c,0x18,
	0x0229,0x64,
	0x024b,0x16,
	0x0255,0x21,
	0x0280,0x38,
	0x021f,0x10,
	0x0224,0x00,
	0x0234,0x00,
	0x024a,0x02,
	0x0282,0x13,
	0x028d,0x92,
	0x039a,0x90,
	0x031c,0x80,
	0x03fe,0x10,
	0x03fe,0x00,
	0x031c,0x9f,
	0x03fe,0x00,
	0x03fe,0x00,
	0x03fe,0x00,
	0x03fe,0x00,
	0x031c,0x80,
	0x03fe,0x10,
	0x03fe,0x00,
	0x031c,0x9f,
	0x00d3,0x68,
	0x00d4,0x01,
	0x00d5,0xe0,
	0x00d6,0x01,
	0x05a0,0x83,
	0x05a3,0x06,
	0x05a4,0x04,
	0x0597,0x27,
	0x059a,0x00,
	0x059b,0x00,
	0x059c,0x01,
	0x05ab,0x09,
	0x05ae,0x00,
	0x05af,0x00,
	0x05ac,0x00,
	0x05ad,0x01,
	0x05b1,0x03,
	0x05b1,0x04,
	0x05b1,0x05,
	0x05b1,0x0c,
	0x05b1,0x1c,
	0x05b1,0x05,
	0x05b1,0x12,
	0x05b1,0x05,
	0x05b1,0x00,
	0x05b1,0x05,
	0x05b1,0x06,
	0x05b1,0x05,
	0x05b1,0x14,
	0x05b1,0x18,
	0x05b1,0x18,
	0x05b1,0x19,
	0x05b1,0x40,
	0x05b1,0x14,
	0x05b1,0x38,
	0x05b1,0x29,
	0x05b1,0x40,
	0x05b1,0x08,
	0x05b1,0x38,
	0x05b1,0x29,
	0x05b1,0x40,
	0x05b1,0x08,
	0x05b1,0x38,
	0x05b1,0x29,
	0x05b1,0x00,
	0x05b1,0x08,
	0x05b1,0x68,
	0x05b1,0x41,
	0x05b1,0x00,
	0x05b1,0x08,
	0x05b1,0x88,
	0x05b1,0x51,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x01,
	0x05b1,0x00,
	0x05b1,0x01,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x01,
	0x05b1,0x94,
	0x05b1,0x03,
	0x05b1,0x01,
	0x05b1,0x02,
	0x05b1,0x00,
	0x05b1,0x00,
	0x05b1,0x02,
	0x05b1,0x02,
	0x05b1,0xd9,
	0x05b1,0x00,
	0x05b1,0x03,
	0x05b1,0x03,
	0x05b1,0xf8,
	0x05b1,0x00,
	0x05b1,0x04,
	0x05b1,0x05,
	0x05b1,0x9e,
	0x05b1,0x00,
	0x05b1,0x05,
	0x05b1,0x07,
	0x05b1,0xf0,
	0x05b1,0x00,
	0x05b1,0x0c,
	0x05b1,0x0b,
	0x05b1,0x3d,
	0x05b1,0x00,
	0x05b1,0x0d,
	0x05b1,0x0f,
	0x05b1,0xc6,
	0x05b1,0x09,
	0x05b1,0x6d,
	0x05ac,0x01,
	0x029f,0xc4,
	0x05a0,0xc3,
	0x02b0,0x70,
	0x0206,0xc0,
	0x02b3,0x00,
	0x02b4,0x00,
	0x0204,0x04,
	0x0205,0x00,
	0x0099,0x00,
	0x0351,0x00,
	0x0352,0x06,
	0x0353,0x00,
	0x0354,0x08,
	0x034c,0x09,
	0x034d,0x00,
	0x034e,0x06,
	0x034f,0xc0,
	0x0108,0x08,
	0x0112,0x0a,
	0x0113,0x0a,
	0x0114,0x01,
	0x0181,0x30,
	0x0185,0x01,
	0x0188,0x00,
	0x0121,0x0b,
	0x0122,0x0d,
	0x0123,0x2f,
	0x0124,0x01,
	0x0125,0x12,
	0x0126,0x0f,
	0x0129,0x0c,
	0x012a,0x13,
	0x012b,0x0f,
	0x0a70,0x11,
	0x0313,0x80,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0aff,0x10,
	0x0a70,0x00,
	0x00be,0x01,
	0x0317,0x00,
	0x0a67,0x00,
	0x0084,0x10,
};

static void custom1_setting(void)
{
	LOG_INF("custom1_setting Start!\n");
	table_write_cmos_sensor(addr_data_pair_custom1,
		sizeof(addr_data_pair_custom1) / sizeof(kal_uint16)); //custom1
	LOG_INF("custom1_setting End!\n");
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor(0x8c, 0x01);
		write_cmos_sensor(0x8d, 0x00);
	} else {
		write_cmos_sensor(0x8c, 0x00);
		write_cmos_sensor(0x8d, 0x10);
	}
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

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
				#ifdef OPLUS_FEATURE_CAMERA_COMMON
				imgsensor_info.module_id = read_module_id();
				read_eeprom_CamInfo();
				read_eeprom_SN();
				LOG_INF("RENM0_module_id=%d\n",imgsensor_info.module_id);
				if(deviceInfo_register_value == 0x00){
					//register_imgsensor_deviceinfo("Cam_r1", DEVICE_VERSION_GC8054, imgsensor_info.module_id);
					deviceInfo_register_value = 0x01;
			}
				#endif
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
	imgsensor.test_pattern = 0;
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

	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate) {
		LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			imgsensor.current_fps, imgsensor_info.cap.max_framerate / 10);
	}
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;

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
	custom1_setting();
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
    #ifdef OPLUS_FEATURE_CAMERA_COMMON
	case SENSOR_FEATURE_GET_MODULE_INFO:
		LOG_INF("GC8054 GET_MODULE_CamInfo:%d %d\n", *feature_para_len, *feature_data_32);
		*(feature_data_32 + 1) = (gGC8054_CamInfo[1] << 24)
					| (gGC8054_CamInfo[0] << 16)
					| (gGC8054_CamInfo[3] << 8)
					| (gGC8054_CamInfo[2] & 0xFF);
		*(feature_data_32 + 2) = (gGC8054_CamInfo[5] << 24)
					| (gGC8054_CamInfo[4] << 16)
					| (gGC8054_CamInfo[7] << 8)
					| (gGC8054_CamInfo[6] & 0xFF);
		break;
    case SENSOR_FEATURE_GET_MODULE_SN:
        LOG_INF("GC8054 GET_MODULE_SN:%d %d\n", *feature_para_len, *feature_data_32);
        if (*feature_data_32 < CAMERA_MODULE_SN_LENGTH/4)
            *(feature_data_32 + 1) = (gGc8054_SN[4*(*feature_data_32) + 3] << 24)
                        | (gGc8054_SN[4*(*feature_data_32) + 2] << 16)
                        | (gGc8054_SN[4*(*feature_data_32) + 1] << 8)
                        | (gGc8054_SN[4*(*feature_data_32)] & 0xFF);
        break;
    case SENSOR_FEATURE_SET_SENSOR_OTP:
        {
            kal_int32 ret = IMGSENSOR_RETURN_SUCCESS;
            LOG_INF("SENSOR_FEATURE_SET_SENSOR_OTP length :%d\n", (UINT32)*feature_para_len);
            ret = write_Module_data((ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(feature_para));

            if (ret == ERROR_NONE)
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
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16)*feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT32) (*feature_data), (UINT32) (*(feature_data + 1)));
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
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	/*Caohua.Lin@CAmera, modify for different module 20180723*/
	case SENSOR_FEATURE_CHECK_MODULE_ID:
		*feature_return_para_32 = imgsensor_info.module_id;
		break;
	#endif
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
		set_test_pattern_mode((kal_uint8)*feature_data);
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
		break;
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
}

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};
UINT32 GC8054_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
