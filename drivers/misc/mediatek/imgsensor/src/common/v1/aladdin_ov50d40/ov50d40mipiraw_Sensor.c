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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     OV50D40mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 * Setting version:
 * ------------
 *   update full pd setting for OV50D40EB_03B
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#define PFX "ALADDIN_OV50D40_camera_sensor"

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/types.h>
#include "imgsensor_eeprom.h"
#include "ov50d40mipiraw_Sensor.h"
#include "ov50d40_Sensor_setting.h"

#define LOG_INF(format, args...)    \
    pr_info(PFX "[%s] " format, __func__, ##args)
#define LOG_ERR(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)

#define PDAF_SUPPORT 1
#define I2C_BUFFER_LEN 765    /*trans# max is 255, each 3 bytes*/
#define VCM_REG_ALADDIN 0xB5
extern enum IMGSENSOR_RETURN Eeprom_DataInit(
    enum IMGSENSOR_SENSOR_IDX sensor_idx,
    kal_uint32 sensorID);
static kal_uint32 streaming_control(kal_bool enable);
static bool _is_binning_size = true;
static DEFINE_SPINLOCK(imgsensor_drv_lock);
static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = ALADDIN_OV50D40_SENSOR_ID,

    .checksum_value = 0xcd9966da,

    .pre = {
        .pclk = 100000000, /*2040x1536@30.3fps*/
        .linelength = 425,
        .framelength = 7842,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2040,
        .grabwindow_height = 1536,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 540000000,
        .max_framerate = 303,
    },

    .cap = {
        .pclk = 100000000, /*4080x3072@30.3fps*/
        .linelength = 850,
        .framelength = 3920,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4080,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 760800000,
        .max_framerate = 303,
    },

    .normal_video = {
        .pclk = 100000000, /*4080x2296@30.0fps*/
        .linelength = 850,
        .framelength = 3440,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4080,
        .grabwindow_height = 2296,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 672000000,
        .max_framerate = 300,
    },

    .hs_video = {
        .pclk = 100000000, /*1920x1080@120fps*/
        .linelength = 425,
        .framelength = 1962,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1920,
        .grabwindow_height = 1080,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 768000000,
        .max_framerate = 12000,
    },

    .slim_video = {
        .pclk = 100000000, /*4080x2296@30.3fps*/
        .linelength = 850,
        .framelength = 3440,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4080,
        .grabwindow_height = 2296,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 672000000,
        .max_framerate = 303,
    },

    .custom1 = {
        .pclk = 100000000, /*4080x3072@30.3fps*/
        .linelength = 850,
        .framelength = 3920,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4080,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 303,
        .mipi_pixel_rate = 760800000,
    },

    .custom2 = {
        .pclk = 100000000, /*4080x3072@24.3fps*/
        .linelength = 1300,
        .framelength = 3204,
        .startx =0,
        .starty = 0,
        .grabwindow_width = 4080,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 240,
        .mipi_pixel_rate = 518400000,
    },
    .margin = 31,                    /* sensor framelength & shutter margin */
    .min_shutter = 10,                /* min shutter */
    .min_gain = 64, /*1x gain*/
    .max_gain = 3968, /*64x gain*/
    .min_gain_iso = 100,
    .gain_step = 1,
    .gain_type = 1,/*to be modify,no gain table for sony*/
    .max_frame_length = 0x7fffff,     /* max framelength by sensor register's limitation */
    .ae_shut_delay_frame = 0,        //check
    .ae_sensor_gain_delay_frame = 0,//check
    .ae_ispGain_delay_frame = 2,
	.frame_time_delay_frame = 1,
    .ihdr_support = 0,
    .ihdr_le_firstline = 0,
    .sensor_mode_num = 7,            //support sensor mode num

    .cap_delay_frame = 3,            //enter capture delay frame num
    .pre_delay_frame = 3,            //enter preview delay frame num
    .video_delay_frame = 3,            //enter video delay frame num
    .hs_video_delay_frame = 3,        //enter high speed video  delay frame num
    .slim_video_delay_frame = 3,    //enter slim video delay frame num
    .custom1_delay_frame = 3,        //enter custom1 delay frame num
    .custom2_delay_frame = 3,        //enter custom2 delay frame num
    .frame_time_delay_frame = 2,

    .isp_driving_current = ISP_DRIVING_2MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2,
    .mipi_settle_delay_mode = 0,
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
    .mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
    .i2c_addr_table = {0x20, 0xff},
    .i2c_speed = 1000,
};

static struct imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,
    .sensor_mode = IMGSENSOR_MODE_INIT,
    .shutter = 0x3D0,
    .gain = 0x100,
    .dummy_pixel = 0,
    .dummy_line = 0,
    .current_fps = 300,
    .autoflicker_en = KAL_FALSE,
    .test_pattern = KAL_FALSE,
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
    .ihdr_en = 0,
    .i2c_write_id = 0x20,
};
/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[7] = {
	{ 8192,  6144,  16,   0,  8160,  6144,  2040,  1536,  0000,  0000,  2040,  1536,  0,  0,  2040,  1536},	/*Preview*/
	{ 8192,  6144,  16,   0,  8160,  6144,  4080,  3072,  0000,  0000,  4080,  3072,  0,  0,  4080,  3072},	/*Capture*/
	{ 8192,  6144,  16, 776,  8160,  4592,  4080,  2296,  0000,  0000,  4080,  2296,  0,  0,  4080,  2296},	/*normal video*/
	{ 8192,  6144, 256, 912,  7680,  4320,  1920,  1080,  0000,  0000,  1920,  1080,  0,  0,  1920,  1080},	/*hs video*/
	{ 8192,  6144,  16, 776,  8160,  4592,  4080,  2296,  0000,  0000,  4080,  2296,  0,  0,  4080,  2296},	/*slim video*/
	{ 8192,  6144,  16,   0,  8160,  6144,  4080,  3072,  0000,  0000,  4080,  3072,  0,  0,  4080,  3072},	/*custom1 DualCam */
	{ 8192,  6144,  16,   0,  8160,  6144,  4080,  3072,  0000,  0000,  4080,  3072,  0,  0,  4080,  3072},	/*custom2 DualCam */
};

#if PDAF_SUPPORT
static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3] = {
    /* preview mode setting */
    {
        0x02, 0x0a, 0x0000, 0x0008, 0x40, 0x00,
        0x00, 0x2b, 0x07F8, 0x0600, 0x00, 0x00, 0x0000, 0x0000,//2040*1536
        0x01, 0x2B, 0x04D8, 0x02F8, 0x03, 0x00, 0x0000, 0x0000 //992*760
    },
    /* capture mode setting */
    {
        0x02, 0x0a, 0x0000, 0x0008, 0x40, 0x00,
        0x00, 0x2b, 0x0FF0, 0x0C00, 0x00, 0x00, 0x0000, 0x0000,//4080*3072
        0x01, 0x2B, 0x04D8, 0x02F8, 0x03, 0x00, 0x0000, 0x0000 //992*760
    },
    /* normal/slim video mode setting */
    {
        0x02, 0x0a, 0x0000, 0x0008, 0x40, 0x00,
        0x00, 0x2b, 0x0FF0, 0x08F8, 0x00, 0x00, 0x0000, 0x0000, //4080*2296
        0x01, 0x2B, 0x04D8, 0x023E, 0x03, 0x00, 0x0000, 0x0000 //992x574
    },
};
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_cap = {
    .i4OffsetX = 64,
    .i4OffsetY = 16,
    .i4PitchX = 16,
    .i4PitchY = 16,
    .i4PairNum = 8,
    .i4SubBlkW = 8,
    .i4SubBlkH = 4,
    .i4PosL = {{70, 18},
                {78, 18},
                {66, 22},
                {74, 22},
                {70, 26},
                {78, 26},
                {66, 30},
                {74, 30} },
    .i4PosR = {{69, 18},
                {77, 18},
                {65, 22},
                {73, 22},
                {69, 26},
                {77, 26},
                {65, 30},
                {73, 30} },
    .iMirrorFlip = 0,
    .i4BlockNumX = 248,
    .i4BlockNumY = 190,
    .i4Crop = { {8, 0}, {8, 0}, {8, 388}, {0, 0}, {8, 388},
        {8, 0}, {8, 0} },
};
//imgsensor_pd_info for normal/slim video
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_video = {
    .i4OffsetX = 64,
    .i4OffsetY = 16,
    .i4PitchX = 16,
    .i4PitchY = 16,
    .i4PairNum = 8,
    .i4SubBlkW = 8,
    .i4SubBlkH = 4,
    .i4PosL = {{70, 18},
                {78, 18},
                {66, 22},
                {74, 22},
                {70, 26},
                {78, 26},
                {66, 30},
                {74, 30} },
    .i4PosR = {{69, 18},
                {77, 18},
                {65, 22},
                {73, 22},
                {69, 26},
                {77, 26},
                {65, 30},
                {73, 30} },
    .iMirrorFlip = 0,
    .i4BlockNumX = 248,
    .i4BlockNumY = 143,
    .i4Crop = { {8, 0}, {8, 0}, {8, 388}, {0, 0}, {8, 388},
        {8, 0}, {8, 0} },
};
#endif

static kal_uint16 ov50d40_table_write_cmos_sensor(
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
            puSendCmd[tosend++] = (char)(data & 0xFF);
            IDX += 2;
            addr_last = addr;
        }

        if ((I2C_BUFFER_LEN - tosend) < 3 ||
            len == IDX ||
            addr != addr_last) {
            iBurstWriteReg_multi(puSendCmd, tosend,
                imgsensor.i2c_write_id,
                3, imgsensor_info.i2c_speed);
            tosend = 0;
        }
    }
    return 0;
}
#if 0
static kal_uint16 ov50d40_burst_write_cmos_sensor(
                    kal_uint16 *para, kal_uint32 len)
{
    char puSendCmd[I2C_BUFFER_LEN];
    kal_uint32 tosend, IDX;
    kal_uint16 addr = 0, addr_last = 0, data;

    tosend = 0;
    IDX = 0;
    while (len > IDX) {
        addr = para[IDX];
        if(tosend==0)
        {
            puSendCmd[tosend++] = (char)(addr >> 8);
            puSendCmd[tosend++] = (char)(addr & 0xFF);
            data = para[IDX + 1];
            puSendCmd[tosend++] = (char)(data & 0xFF);
            IDX += 2;
            addr_last = addr;
        }
        else if(addr == addr_last+1)
        {
            data = para[IDX + 1];
            puSendCmd[tosend++] = (char)(data & 0xFF);
            addr_last = addr;
            IDX += 2;
        }

        if (( tosend>=I2C_BUFFER_LEN)||
            len == IDX ||
            addr != addr_last) {
            iBurstWriteReg_multi(puSendCmd, tosend,
                imgsensor.i2c_write_id,
                tosend, imgsensor_info.i2c_speed);
            tosend = 0;
        }
    }
    return 0;
}
#endif

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte = 0;
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pusendcmd[4] = {(char)(addr >> 8),
        (char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2CTiming(pusendcmd, 3, imgsensor.i2c_write_id,
        imgsensor_info.i2c_speed);
}

static void set_dummy(void)
{
    if (_is_binning_size){
        write_cmos_sensor(0x3840, imgsensor.frame_length*2 >> 16);
        write_cmos_sensor(0x380e, imgsensor.frame_length*2 >>  8);
        write_cmos_sensor(0x380f, imgsensor.frame_length*2 & 0xFF);
    }else{
        write_cmos_sensor(0x3840, imgsensor.frame_length >> 16);
        write_cmos_sensor(0x380e, imgsensor.frame_length >>  8);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
    }
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
}


static void write_shutter(kal_uint32 shutter)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    // OV Recommend Solution
    // if shutter bigger than frame_length, should extend frame length first
    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
    imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;

    shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
    // Framelength should be an even number
    shutter = (shutter >> 1) << 1;
    imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
        else {
             // Extend frame length
             write_cmos_sensor(0x3208, 0x00);
             if (_is_binning_size){
                 write_cmos_sensor(0x3840, imgsensor.frame_length*2 >> 16);
                 write_cmos_sensor(0x380e, imgsensor.frame_length*2 >>  8);
                 write_cmos_sensor(0x380f, imgsensor.frame_length*2 & 0xFF);
             }else{
                 write_cmos_sensor(0x3840, imgsensor.frame_length >> 16);
                 write_cmos_sensor(0x380e, imgsensor.frame_length >>  8);
                 write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
             }
              write_cmos_sensor(0x3208, 0x10);
              write_cmos_sensor(0x3208, 0xa0);
        }
    } else {
        // Extend frame length
        write_cmos_sensor(0x3208, 0x00);
            if (_is_binning_size){
                write_cmos_sensor(0x3840, imgsensor.frame_length*2 >> 16);
                write_cmos_sensor(0x380e, imgsensor.frame_length*2 >>  8);
                write_cmos_sensor(0x380f, imgsensor.frame_length*2 & 0xFF);
            }else{
                write_cmos_sensor(0x3840, imgsensor.frame_length >> 16);
                write_cmos_sensor(0x380e, imgsensor.frame_length >>  8);
                write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
            }
        write_cmos_sensor(0x3208, 0x10);
        write_cmos_sensor(0x3208, 0xa0);
    }
    // Update Shutter
    write_cmos_sensor(0x3208, 0x01);
    if (_is_binning_size){
        write_cmos_sensor(0x3840, imgsensor.frame_length*2 >> 16);
        write_cmos_sensor(0x380e, imgsensor.frame_length*2 >>  8);
        write_cmos_sensor(0x380f, imgsensor.frame_length*2 & 0xFF);
        write_cmos_sensor(0x3500, (shutter*2 >> 16) & 0xFF);
        write_cmos_sensor(0x3501, (shutter*2 >>  8) & 0xFF);
        write_cmos_sensor(0x3502, (shutter*2)  & 0xFF);
    }else{
        write_cmos_sensor(0x3840, imgsensor.frame_length >> 16);
        write_cmos_sensor(0x380e, imgsensor.frame_length >>  8);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
        write_cmos_sensor(0x3500, (shutter >> 16) & 0xFF);
        write_cmos_sensor(0x3501, (shutter >>  8) & 0xFF);
        write_cmos_sensor(0x3502, (shutter)  & 0xFF);
    }
    write_cmos_sensor(0x3208, 0x11);
    write_cmos_sensor(0x3208, 0xa1);
    LOG_INF("shutter =%d, framelength =%d, realtime_fps =%d\n",
            shutter, imgsensor.frame_length, realtime_fps);
}
static void set_shutter(kal_uint32 shutter)  //should not be kal_uint16 -- can't reach long exp
{
    unsigned long flags;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
    write_shutter(shutter);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 iReg = 0x0000;

	//platform 1xgain = 64, sensor driver 1*gain = 0x100
	iReg = gain*256/BASEGAIN;

	if(iReg < 0x100)	//sensor 1xGain
	{
		iReg = 0x100;
	}
	if(iReg > 0x3e00)	//sensor 62xGain
	{
		iReg = 0x3e00;
	}
	return iReg;		/* sensorGlobalGain */

}

static kal_uint16 set_gain(kal_uint16 gain)
{
   	kal_uint16 reg_gain;
	unsigned long flags;

	reg_gain = gain2reg(gain);
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.gain = reg_gain;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
	write_cmos_sensor(0x3508, (reg_gain >> 8));
	write_cmos_sensor(0x3509, (reg_gain&0xff));

	return gain;
}

/* ITD: Modify Dualcam By Jesse 190924 Start */
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 target_frame_length, kal_bool auto_extend_en)
{
    spin_lock(&imgsensor_drv_lock);
    if(target_frame_length > 1)
        imgsensor.dummy_line = target_frame_length - imgsensor.frame_length;
    imgsensor.frame_length = imgsensor.frame_length + imgsensor.dummy_line;
    imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);
    set_shutter(shutter);
}

static void sensor_init(void)
{
    // write_cmos_sensor(0x0103, 0x01);//SW Reset, need delay
    // mdelay(5);
    LOG_INF("sensor_init start\n");
    ov50d40_table_write_cmos_sensor(
        addr_data_pair_init_ov50d40,
        sizeof(addr_data_pair_init_ov50d40) / sizeof(kal_uint16));
    LOG_INF("sensor_init end\n");
}

static void preview_setting(void)
{
    int _length = 0;

    LOG_INF("preview_setting begin\n");
    _length = sizeof(addr_data_pair_preview_ov50d40) / sizeof(kal_uint16);

    ov50d40_table_write_cmos_sensor(
        addr_data_pair_preview_ov50d40,
        _length);
    LOG_INF("preview_setting_end\n");
}

static void capture_setting(kal_uint16 currefps)
{
    int _length = 0;

    LOG_INF("capture_setting currefps = %d\n", currefps);
    _length = sizeof(addr_data_pair_capture_ov50d40) / sizeof(kal_uint16);
    ov50d40_table_write_cmos_sensor(
        addr_data_pair_capture_ov50d40,
        _length);
}

static void normal_video_setting(kal_uint16 currefps)
{
    int _length = 0;

    LOG_INF("normal_video_setting \n");
    _length = sizeof(addr_data_pair_video_ov50d40) / sizeof(kal_uint16);
    ov50d40_table_write_cmos_sensor(
        addr_data_pair_video_ov50d40,
        _length);

}

static void hs_video_setting(void)
{
    int _length = 0;

    LOG_INF("hs_video_setting \n");
    _length = sizeof(addr_data_pair_hs_video_ov50d40) / sizeof(kal_uint16);
        ov50d40_table_write_cmos_sensor(
        addr_data_pair_hs_video_ov50d40,
        _length);

}

static void slim_video_setting(void)
{
    int _length = 0;
    LOG_INF("slim_video_setting s\n");
    _length = sizeof(addr_data_pair_slim_video_ov50d40) / sizeof(kal_uint16);
        ov50d40_table_write_cmos_sensor(
        addr_data_pair_slim_video_ov50d40,
        _length);
}

/* ITD: Modify Dualcam By Jesse 190924 Start */
static void custom1_setting(void)
{
    int _length = 0;

    LOG_INF("custom1_setting_start\n");
    _length = sizeof(addr_data_pair_custom1) / sizeof(kal_uint16);
        ov50d40_table_write_cmos_sensor(
        addr_data_pair_custom1,
        _length);
    LOG_INF("custom1_setting_end\n");
}    /*    custom1_setting  */

/* ITD: Modify Dualcam By Jesse 190924 Start */
static void custom2_setting(void)
{
    int _length = 0;

    LOG_INF("custom2_setting_start\n");
    _length = sizeof(addr_data_pair_custom2) / sizeof(kal_uint16);
        ov50d40_table_write_cmos_sensor(
        addr_data_pair_custom2,
        _length);
    LOG_INF("custom2_setting_end\n");
}    /*    custom2_setting  */

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x300a) << 16) |
		(read_cmos_sensor(0x300b) << 8) | read_cmos_sensor(0x300c));
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
	kal_uint8 ov50d40_vcm_id = 0;

    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id();
            LOG_INF("[aladdin_ov50d40] get_imgsensor_id:0x%x  i2c:0x%x\n", *sensor_id, imgsensor.i2c_write_id);
            if (*sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("[aladdin_ov50d40] i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
                ov50d40_vcm_id = Eeprom_1ByteDataRead(0x0A, 0xA0);
                LOG_INF("[aladdin_ov50d40] vcm_id = 0x%x\n", ov50d40_vcm_id);
                if(ov50d40_vcm_id == VCM_REG_ALADDIN) {
                    Eeprom_DataInit(IMGSENSOR_SENSOR_IDX_MAIN, *sensor_id);
                    LOG_INF("[aladdin_ov50d40] Eeprom_DataInit successful \n");
                } else {
                    LOG_INF("[aladdin_ov50d40] Not Find vcm_id! \n");
                    *sensor_id = 0xFFFFFFFF;
                    return ERROR_SENSOR_CONNECT_FAIL;
                }
                return ERROR_NONE;
            }
            retry--;
        } while (retry > 0);
            i++;
            retry = 1;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        LOG_INF("sensor_id = 0x%x imgsensor_info.sensor_id: 0x%x fail\n", *sensor_id, imgsensor_info.sensor_id);
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    return ERROR_NONE;
}

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
                LOG_INF("open success: i2c write id: 0x%x, sensor id: 0x%x\n",
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
    imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}

static kal_uint32 close(void)
{
    streaming_control(KAL_FALSE);
    return ERROR_NONE;
}   /*  close  */

static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.current_fps = imgsensor.current_fps;
    //imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    _is_binning_size = false;
    return ERROR_NONE;
}
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    imgsensor.pclk = imgsensor_info.cap.pclk;
    imgsensor.line_length = imgsensor_info.cap.linelength;
    imgsensor.frame_length = imgsensor_info.cap.framelength;
    imgsensor.min_frame_length = imgsensor_info.cap.framelength;
    //imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    capture_setting(imgsensor.current_fps);
    _is_binning_size = true;
    return ERROR_NONE;
} /* capture() */

static kal_uint32 normal_video(
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    //imgsensor.current_fps = 300;
    //imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    normal_video_setting(imgsensor.current_fps);
    _is_binning_size = true;
    return ERROR_NONE;
}

static kal_uint32 hs_video(
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();
    _is_binning_size = false;
    return ERROR_NONE;
}

static kal_uint32 slim_video(
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();
    _is_binning_size = true;
    return ERROR_NONE;
}

/* ITD: Modify Dualcam By Jesse 190924 Start */
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
    _is_binning_size = true;
    return ERROR_NONE;
}   /*  Custom1   */

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
    _is_binning_size = true;
    return ERROR_NONE;
}   /*  Custom2   */

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

/* ITD: Modify Dualcam By Jesse 190924 Start */
    sensor_resolution->SensorCustom1Width =
        imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height =
        imgsensor_info.custom1.grabwindow_height;
/* ITD: Modify Dualcam By Jesse 190924 End */
    sensor_resolution->SensorCustom2Width =
        imgsensor_info.custom2.grabwindow_width;
    sensor_resolution->SensorCustom2Height =
        imgsensor_info.custom2.grabwindow_height;
    return ERROR_NONE;
}   /*  get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
              MSDK_SENSOR_INFO_STRUCT *sensor_info,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    if (scenario_id == 0)
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

/* ITD: Modify Dualcam By Jesse 190924 Start */
    sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
/* ITD: Modify Dualcam By Jesse 190924 End */
    sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
/* The frame of setting shutter default 0 for TG int */
    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
    /* The frame of setting sensor gain */
    sensor_info->AESensorGainDelayFrame =
        imgsensor_info.ae_sensor_gain_delay_frame;
    sensor_info->AEISPGainDelayFrame =
        imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
#if PDAF_SUPPORT
    sensor_info->PDAF_Support = 2;

#endif
    //sensor_info->HDR_Support = 0; /*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR*/
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
    sensor_info->SensorHightSampling = 0;   // 0 is default 1x
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
/* ITD: Modify Dualcam By Jesse 190924 Start */
    case MSDK_SCENARIO_ID_CUSTOM1:
        sensor_info->SensorGrabStartX =
            imgsensor_info.custom1.startx;
        sensor_info->SensorGrabStartY =
            imgsensor_info.custom1.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
    break;
/* ITD: Modify Dualcam By Jesse 190924 End */
 case MSDK_SCENARIO_ID_CUSTOM2:
        sensor_info->SensorGrabStartX =
            imgsensor_info.custom2.startx;
        sensor_info->SensorGrabStartY =
            imgsensor_info.custom2.starty;
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
}   /*  get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
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
/* ITD: Modify Dualcam By Jesse 190924 Start */
    case MSDK_SCENARIO_ID_CUSTOM1:
        Custom1(image_window, sensor_config_data);
    break;
/* ITD: Modify Dualcam By Jesse 190924 End */
    case MSDK_SCENARIO_ID_CUSTOM2:
        Custom2(image_window, sensor_config_data);
    break;
    default:
        LOG_INF("Error ScenarioId setting");
        preview(image_window, sensor_config_data);
    return ERROR_INVALID_SCENARIO_ID;
    }

    return ERROR_NONE;
}   /* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
    // SetVideoMode Function should fix framerate
    if (framerate == 0)
    // Dynamic frame rate
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

static kal_uint32 set_auto_flicker_mode(kal_bool enable,
            UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d\n",
        enable, framerate);

    spin_lock(&imgsensor_drv_lock);
    if (enable) //enable auto flicker
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
    kal_uint32 frameHeight;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    if (framerate == 0)
        return ERROR_NONE;

    switch (scenario_id) {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        frameHeight = imgsensor_info.pre.pclk / framerate * 10 /
            imgsensor_info.pre.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frameHeight > imgsensor_info.pre.framelength) ?
            (frameHeight - imgsensor_info.pre.framelength):0;
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
        frameHeight = imgsensor_info.normal_video.pclk / framerate * 10 /
                imgsensor_info.normal_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frameHeight >
            imgsensor_info.normal_video.framelength) ?
        (frameHeight - imgsensor_info.normal_video.framelength):0;
        imgsensor.frame_length = imgsensor_info.normal_video.framelength +
            imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
    break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        frameHeight = imgsensor_info.cap.pclk / framerate * 10 /
            imgsensor_info.cap.linelength;
        spin_lock(&imgsensor_drv_lock);

        imgsensor.dummy_line =
            (frameHeight > imgsensor_info.cap.framelength) ?
            (frameHeight - imgsensor_info.cap.framelength):0;
        imgsensor.frame_length = imgsensor_info.cap.framelength +
            imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
    break;
    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        frameHeight = imgsensor_info.hs_video.pclk / framerate * 10 /
            imgsensor_info.hs_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frameHeight > imgsensor_info.hs_video.framelength) ?
            (frameHeight - imgsensor_info.hs_video.framelength):0;
        imgsensor.frame_length = imgsensor_info.hs_video.framelength +
            imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
    break;
    case MSDK_SCENARIO_ID_SLIM_VIDEO:
        frameHeight = imgsensor_info.slim_video.pclk / framerate * 10 /
            imgsensor_info.slim_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frameHeight >
            imgsensor_info.slim_video.framelength) ?
            (frameHeight - imgsensor_info.slim_video.framelength):0;
        imgsensor.frame_length = imgsensor_info.slim_video.framelength +
            imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
    break;
/* ITD: Modify Dualcam By Jesse 190924 Start */
    case MSDK_SCENARIO_ID_CUSTOM1:
        frameHeight = imgsensor_info.custom1.pclk / framerate * 10 /
            imgsensor_info.custom1.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frameHeight >
            imgsensor_info.custom1.framelength) ?
            (frameHeight - imgsensor_info.custom1.framelength):0;
        imgsensor.frame_length = imgsensor_info.custom1.framelength +
            imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
    break;
/* ITD: Modify Dualcam By Jesse 190924 End */
    case MSDK_SCENARIO_ID_CUSTOM2:
        frameHeight = imgsensor_info.custom2.pclk / framerate * 10 /
            imgsensor_info.custom2.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frameHeight >
            imgsensor_info.custom2.framelength) ?
            (frameHeight - imgsensor_info.custom2.framelength):0;
        imgsensor.frame_length = imgsensor_info.custom2.framelength +
            imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
    break;
    default:  //coding with  preview scenario by default
        frameHeight = imgsensor_info.pre.pclk / framerate * 10 /
            imgsensor_info.pre.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frameHeight >
            imgsensor_info.pre.framelength) ?
            (frameHeight - imgsensor_info.pre.framelength):0;
        imgsensor.frame_length = imgsensor_info.pre.framelength +
            imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
    break;
    }
    return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(
            enum MSDK_SCENARIO_ID_ENUM scenario_id,
            MUINT32 *framerate)
{
    if (scenario_id == 0)
    LOG_INF("[3058]scenario_id = %d\n", scenario_id);

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
/* ITD: Modify Dualcam By Jesse 190924 Start */
    case MSDK_SCENARIO_ID_CUSTOM1:
        *framerate = imgsensor_info.custom1.max_framerate;
    break;
/* ITD: Modify Dualcam By Jesse 190924 End */
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
    LOG_INF("Jesse+ enable: %d\n", enable);
    if (enable) { // for solid color

        write_cmos_sensor(0x430b, 0x00);
        write_cmos_sensor(0x430c, 0x00);
        write_cmos_sensor(0x4310, 0x00);
        write_cmos_sensor(0x4311, 0x00);
        } else {
        write_cmos_sensor(0x430b, 0xff);
        write_cmos_sensor(0x430c, 0xff);
        write_cmos_sensor(0x4310, 0xff);
        write_cmos_sensor(0x4311, 0xff);
        }

    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

static kal_uint32 get_sensor_temperature(void)
{
    UINT32 temperature = 0;
    INT32 temperature_convert = 0;

    /*TEMP_SEN_CTL */
    write_cmos_sensor(0x4d12, 0x01);
    temperature = (read_cmos_sensor(0x4d13) << 8) |
        read_cmos_sensor(0x4d13);
    if (temperature < 0xc000)
        temperature_convert = temperature / 256;
    else
        temperature_convert = 192 - temperature / 256;

    if (temperature_convert > 192) {
        //LOG_INF("Temperature too high: %d\n",
                //temperature_convert);
        temperature_convert = 192;
    } else if (temperature_convert < -64) {
        //LOG_INF("Temperature too low: %d\n",
                //temperature_convert);
        temperature_convert = -64;
    }

    return 20;
    //return temperature_convert;
}

static kal_uint32 streaming_control(kal_bool enable)
{
    LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
    if (enable)
        write_cmos_sensor(0x0100, 0x01);
    else
        write_cmos_sensor(0x0100, 0x00);
    return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
            UINT8 *feature_para, UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
    UINT16 *feature_data_16 = (UINT16 *) feature_para;
    UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
    UINT32 *feature_data_32 = (UINT32 *) feature_para;
    INT32 *feature_return_para_i32 = (INT32 *) feature_para;
    unsigned long long *feature_data = (unsigned long long *) feature_para;

    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
#if PDAF_SUPPORT
    struct SENSOR_VC_INFO_STRUCT *pvcinfo;

    struct SET_PD_BLOCK_INFO_T *PDAFinfo;
#endif
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
        (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
    if (!((feature_id == 3040) || (feature_id == 3058)))
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
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CUSTOM2:
        default:

            *(feature_data + 2) = 2;
            break;
        }
        break;
#if 0
    case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
        *(MINT32 *)(signed long)(*(feature_data + 1)) = -17200000;
        break;
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
    case SENSOR_FEATURE_SET_NIGHTMODE:
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
#if 0
    case SENSOR_FEATURE_CHECK_MODULE_ID:
        *feature_return_para_32 = imgsensor_info.module_id;
        break;
#endif
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
        spin_lock(&imgsensor_drv_lock);
        imgsensor.current_fps = *feature_data_32;
        spin_unlock(&imgsensor_drv_lock);
        LOG_INF("current fps :%d\n", imgsensor.current_fps);
    break;
    case SENSOR_FEATURE_GET_CROP_INFO:
        LOG_INF("GET_CROP_INFO scenarioId:%d\n",
            *feature_data_32);

        wininfo = (struct  SENSOR_WINSIZE_INFO_STRUCT *)
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
    case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
        LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
            (UINT16)*feature_data, (UINT16)*(feature_data+1),
            (UINT16)*(feature_data+2));
    break;
    case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
            switch (*feature_data) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.cap.mipi_pixel_rate;
                break;
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.normal_video.mipi_pixel_rate;
                break;
            case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.hs_video.mipi_pixel_rate;
                break;
            case MSDK_SCENARIO_ID_SLIM_VIDEO:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.slim_video.mipi_pixel_rate;
                break;
            case MSDK_SCENARIO_ID_CUSTOM1:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.custom1.mipi_pixel_rate;
                break;
            case MSDK_SCENARIO_ID_CUSTOM2:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.custom2.mipi_pixel_rate;
                break;
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            default:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.pre.mipi_pixel_rate;
                break;
            }
    break;
    case SENSOR_FEATURE_GET_BINNING_TYPE:
        switch (*(feature_data + 1)) {
            default:
                *feature_return_para_32 = 1470; /*BINNING_AVERAGED*/
                break;
            }
        LOG_INF("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
            *feature_return_para_32);
        *feature_para_len = 4;

        break;
#if PDAF_SUPPORT
    case SENSOR_FEATURE_GET_VC_INFO:
        LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n",
            (UINT16) *feature_data);
        pvcinfo =
        (struct SENSOR_VC_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

        switch (*feature_data_32) {
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
                       sizeof(struct SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            case MSDK_SCENARIO_ID_CUSTOM1:
            case MSDK_SCENARIO_ID_CUSTOM2:
                memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
                       sizeof(struct SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            case MSDK_SCENARIO_ID_SLIM_VIDEO:
                memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
                       sizeof(struct SENSOR_VC_INFO_STRUCT));
                break;

            default:
                break;
            }
        break;
    case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
        switch (*feature_data) {
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            case MSDK_SCENARIO_ID_SLIM_VIDEO:
            case MSDK_SCENARIO_ID_CUSTOM1:
            case MSDK_SCENARIO_ID_CUSTOM2:
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
                break;
            case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            default:
                *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                break;
        }
        break;
    case SENSOR_FEATURE_GET_PDAF_INFO:
        PDAFinfo = (struct SET_PD_BLOCK_INFO_T *)
            (uintptr_t)(*(feature_data+1));

        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CUSTOM2:
            memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info_cap,
                sizeof(struct SET_PD_BLOCK_INFO_T));
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info_video,
                sizeof(struct SET_PD_BLOCK_INFO_T));
            break;
        default:
            break;
        }
        break;
    case SENSOR_FEATURE_GET_PDAF_DATA:
        break;
    case SENSOR_FEATURE_SET_PDAF:
            imgsensor.pdaf_mode = *feature_data_16;
        break;
#endif
    case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
        *feature_return_para_i32 = get_sensor_temperature();
        *feature_para_len = 4;
    break;
    case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
        LOG_INF("SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME\n");
        set_shutter_frame_length((UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16) *(feature_data + 2));
        break;
    case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
        streaming_control(KAL_FALSE);
        break;

    case SENSOR_FEATURE_SET_STREAMING_RESUME:
        if (*feature_data != 0)
            set_shutter(*feature_data);
        streaming_control(KAL_TRUE);
        break;
    case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
        /*
         * 1, if driver support new sw frame sync
         * set_shutter_frame_length() support third para auto_extend_en
         */
        *(feature_data + 1) = 1; /* margin info by scenario */
        *(feature_data + 2) = imgsensor_info.margin;
        break;
    default:
    break;
    }

    return ERROR_NONE;
}   /*  feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 ALADDIN_OV50D40_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc != NULL)
    *pfFunc =  &sensor_func;
    return ERROR_NONE;
}
