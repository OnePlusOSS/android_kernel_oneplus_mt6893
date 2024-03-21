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
 *     OV32A1Qmipi_Sensor.c
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 * Setting version:
 * ------------
 *   update full pd setting for OV32a1QEB_03B
 *---------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#define PFX "ov32a1q_camera_sensor"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "ov32a1qmipiraw_Sensor.h"
#define LOG_INF(format, args...)    \
    pr_debug(PFX "[%s] " format, __func__, ##args)

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static kal_uint8 xtalk_flag = 0;
#endif


#define MULTI_WRITE 1
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = ATHENSB_OV32A1Q_SENSOR_ID,

    .checksum_value = 0x8b86a64,

    .pre = {  //3264*2448,30fps
        .pclk =   90000000,    //jack yan check
        .linelength =  840,
        .framelength = 3570,
        .startx = 0,
        .starty = 0,
        .grabwindow_width =  3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 21,
        .max_framerate = 300,
        .mipi_pixel_rate = 393600000,
    },

    .cap = {   //3264*2448,30fps
        .pclk =   90000000,
        .linelength =  840,
        .framelength = 3570,
        .startx = 0,
        .starty = 0,
        .grabwindow_width =  3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 21,
        .max_framerate = 300,
        .mipi_pixel_rate = 393600000,
    },


    .normal_video = {   //3264*2448,30fps
        .pclk =   90000000,
        .linelength =  840,
        .framelength = 3570,
        .startx = 0,
        .starty = 0,
        .grabwindow_width =  3264,
        .grabwindow_height = 1836,
        .mipi_data_lp2hs_settle_dc = 21,
        .max_framerate = 300,
        .mipi_pixel_rate = 398400000,
    },

    .hs_video = {
        .pclk = 90000000,
        .linelength  = 390,
        .framelength =  962,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 21,
        .max_framerate = 2400,
        .mipi_pixel_rate = 336000000,
    },
    .slim_video = {
        .pclk = 90000000,
        .linelength =  420,
        .framelength = 3575,
        .startx = 0,
        .starty = 0,
        .grabwindow_width  = 3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 21,
        .max_framerate = 600,
        .mipi_pixel_rate = 787200000,
    },
    .custom1 = {  //2688*1512,120fps
        .pclk = 90000000,
        .linelength =  420,
        .framelength = 1784,
        .startx = 0,
        .starty = 0,
        .grabwindow_width  = 2688,
        .grabwindow_height = 1512,
        .mipi_data_lp2hs_settle_dc = 21,
        .max_framerate = 1200,
        .mipi_pixel_rate = 787200000,
    },
    .custom2 = { //3264*2448,30fps
        .pclk =   90000000,
        .linelength =  840,
        .framelength = 3570,
        .startx = 0,
        .starty = 0,
        .grabwindow_width =  3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 21,
        .max_framerate = 300,
        .mipi_pixel_rate = 393600000,
    },
    .custom3 = { //6258*4896,15fps
        .pclk =  90000000,
        .linelength =  1200,
        .framelength = 5120,
        .startx = 0,
        .starty = 0,
        .grabwindow_width =  6528,
        .grabwindow_height = 4896,
        .mipi_data_lp2hs_settle_dc = 21,
        .max_framerate = 146,
        .mipi_pixel_rate = 595200000,
    },
    .custom4 = {  //3264*2448,30fps
        .pclk =   90000000,
        .linelength =  506,
        .framelength = 5928,
        .startx = 0,
        .starty = 0,
        .grabwindow_width =  1632,
        .grabwindow_height = 1224,
        .mipi_data_lp2hs_settle_dc = 120,
        .max_framerate = 300,
        .mipi_pixel_rate = 384000000,
    },
    .margin = 8,
    .min_shutter = 0x4,        //min shutter check
    .min_gain = 64, /*1x gain*/
    .max_gain = 1024, /*16x gain*/
    .min_gain_iso = 100,
    .gain_step = 1,
    .gain_type = 1,/*to be modify,no gain table for sony*/
    .max_frame_length = 0x7fff,
    .ae_shut_delay_frame = 0,  //check
    .ae_sensor_gain_delay_frame = 0,  //check
    .ae_ispGain_delay_frame = 2,
    .frame_time_delay_frame = 1,
    .ihdr_support = 0,
    .ihdr_le_firstline = 0,
    .sensor_mode_num = 9,

    .cap_delay_frame = 3,
    .pre_delay_frame = 2,
    .video_delay_frame = 2,
    .hs_video_delay_frame = 2,
    .slim_video_delay_frame = 2,
    .custom1_delay_frame = 2,
    .custom2_delay_frame = 2,
    .custom3_delay_frame = 2,
    .custom4_delay_frame = 2,

    .isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2,
    .mipi_settle_delay_mode = 1,
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
    .mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
    .i2c_addr_table = { 0x6C, 0x20, 0xff},
    .i2c_speed = 400,
};


//how to make sure of this shutter and gain jack ???
static struct imgsensor_struct imgsensor = {
    .mirror = IMAGE_HV_MIRROR,
    .sensor_mode = IMGSENSOR_MODE_INIT,
    .shutter = 0x3D0,
    .gain = 0x100,
    .dummy_pixel = 0,
    .dummy_line = 0,
    .current_fps = 30,
    .autoflicker_en = KAL_FALSE,
    .test_pattern = KAL_FALSE,
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
    .ihdr_en = 0,
    .i2c_write_id = 0x20,
};

static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[9] = {
//Preview OK
    { 6528, 4896,    0,    0, 6528, 4896, 3264, 2448,
    0,    0, 3264, 2448,    0,    0, 3264, 2448},

//capture
    { 6528, 4896,    0,    0, 6528, 4896, 3264, 2448,
    0,    0, 3264, 2448,    0,    0, 3264, 2448},
//normal-video
    { 6528, 4896,    0,    612, 6528, 3672, 3264, 1836,
    0,    0, 3264, 1836,    0,    0, 3264, 1836},
//  { 6528, 4896,    0,    0, 6400, 3600, 1280, 720,
//  0,    0, 1280, 720,    0,    0, 1280, 720},
//hs-video
    { 6528, 4896,    0,    0, 6400, 3600, 1280, 720,
    0,    0, 1280, 720,    0,    0, 1280, 720},
//slim-video
    { 6528, 4896,    0,    0, 6528, 4896, 3264, 2448,
    0,    0, 3264, 2448,    0,    0, 3264, 2448},
//custom1
    { 6528, 4896,    0,    0, 6528, 4896, 3264, 2448,
    288,    468, 2688, 1512,    0,    0, 2688, 1512},
//custom2
    { 6528, 4896,    0,    0, 6528, 4896, 3264, 2448,
    0,    0, 3264, 2448,    0,    0, 3264, 2448},
//custom3
    { 6528, 4896,    0,    0, 6528, 4896, 6528, 4896,
    0,    0, 6528, 4896,    0,    0, 6528, 4896},
//custom4
    { 6528, 4896,    0,    0, 6528, 4896, 1632, 1224,
    0,    0,   1632, 1224,    0,    0, 1632, 1224},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
     .i4OffsetX = 0,
     .i4OffsetY = 0,
     .i4PitchX = 32,
     .i4PitchY = 32,
     .i4PairNum = 8,
     .i4SubBlkW = 16,
     .i4SubBlkH = 8,
     .i4BlockNumX = 128,
     .i4BlockNumY = 96,
     .i4PosL = {{14, 6}, {30, 6}, {6, 10}, {22, 10},
            {14, 22}, {30, 22}, {6, 26}, {22, 26} },
     .i4PosR = {{14, 2}, {30, 2}, {6, 14}, {22, 14},
            {14, 18}, {30, 18}, {6, 30}, {22, 30} },
     .iMirrorFlip = 0,
     .i4BlockNumX = 128,
     .i4BlockNumY = 96,
};

#if MULTI_WRITE
#define I2C_BUFFER_LEN 225
#else
#define I2C_BUFFER_LEN 3
#endif

static kal_uint16 ov32a1q_table_write_cmos_sensor(
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
#if MULTI_WRITE
        if ((I2C_BUFFER_LEN - tosend) < 3 ||
            len == IDX ||
            addr != addr_last) {
            iBurstWriteReg_multi(puSendCmd, tosend,
                imgsensor.i2c_write_id,
                3, imgsensor_info.i2c_speed);

            tosend = 0;
        }
#else
        iWriteRegI2C(puSendCmd, 3, imgsensor.i2c_write_id);
        tosend = 0;

#endif
    }
    return 0;
}

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

    iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static kal_uint16 read_cmos_eeprom_8(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 1, 0xA8);
    return get_byte;
}
static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF),(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static kal_uint16 ov32a_xtalk_setting[288*2];
static void read_sensor_xtalk(void)
{
    kal_uint16 idx = 0, addr_xtalk = 0x0C90, sensor_addr = 0x53C0;

    for (idx = 0; idx <288; idx++) {
        addr_xtalk = 0x0C90 + idx;
        sensor_addr = 0x53C0 + idx;
        ov32a_xtalk_setting[2*idx] = sensor_addr;
        ov32a_xtalk_setting[2*idx + 1] = read_cmos_eeprom_8(addr_xtalk);
    }
}

static void write_sensor_xtalk(void)
{
    kal_uint16 idx = 0, addr_xtalk = 0x53C0;

    for (idx = 0; idx < 288; idx++) {
        addr_xtalk = 0x53C0 + idx;
        write_cmos_sensor_8(addr_xtalk, ov32a_xtalk_setting[2 * idx + 1]);
    }

}

#endif

static void set_dummy(void)
{
    //check

    write_cmos_sensor(0x380c, imgsensor.line_length >> 8);
    write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);
    write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
    write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
    //kal_int16 dummy_line;
    kal_uint32 frame_length = imgsensor.frame_length;

    frame_length = imgsensor.pclk / framerate * 10 /
        imgsensor.line_length;
    //LOG_INF("frame_length =%d\n", frame_length);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length =
        (frame_length > imgsensor.min_frame_length) ?
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

    set_dummy();  //adjust the fps and write it to the sensor
}

static void write_shutter(kal_uint16 shutter)
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

    shutter =
        (shutter < imgsensor_info.min_shutter) ?
        imgsensor_info.min_shutter : shutter;
    shutter =
    (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
    (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

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
        } else {
            imgsensor.frame_length =
                (imgsensor.frame_length  >> 1) << 1;
            write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
            write_cmos_sensor(0x380f,
                imgsensor.frame_length & 0xFF);
        }
    } else {
        imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;

        write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
    }

    /*Warning : shutter must be even. Odd might happen Unexpected Results */
    /*
    write_cmos_sensor(0x3500, (shutter >> 12) & 0x0F);   //need to verify
    write_cmos_sensor(0x3501, (shutter >> 4) & 0xFF);
    write_cmos_sensor(0x3502, (shutter<<4)  & 0xF0);
    */
    write_cmos_sensor(0x3501, (shutter >> 8) & 0xFF);
    write_cmos_sensor(0x3502, (shutter)  & 0xFF);
    LOG_INF("shutter =%d, framelength =%d, realtime_fps =%d\n",
        shutter, imgsensor.frame_length, realtime_fps);
}

static void set_shutter(kal_uint16 shutter)//check out
{
    unsigned long flags;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
    write_shutter(shutter);
}

static kal_uint32 get_sensor_temperature(void)
{
    UINT32 temperature = 0;
    INT32 temperature_convert = 0;

    /*TEMP_SEN_CTL */
    write_cmos_sensor(0x4d12, 0x01);
    temperature = (read_cmos_sensor(0x4d13) << 8) |
        read_cmos_sensor(0x4d13);

    temperature_convert = 192 - temperature / 256;

    if (temperature_convert > 192)
        temperature_convert = 192;
    else if (temperature_convert < -64)
        temperature_convert = -64;

    return 20;
    //return temperature_convert;
}

static kal_uint32 streaming_control(kal_bool enable)
{
    LOG_INF("%s enable =%d\n", __func__, enable);
    if (enable)
        write_cmos_sensor(0x0100, 0x01);
    else
        write_cmos_sensor(0x0100, 0x00);

    mdelay(10);

    return ERROR_NONE;
}

static void set_shutter_frame_length(kal_uint16 shutter,
            kal_uint16 frame_length)
{
    kal_uint16 realtime_fps = 0;

    unsigned long flags;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    if (frame_length > 1)
        imgsensor.frame_length = frame_length;

    spin_lock(&imgsensor_drv_lock);
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
    if (imgsensor.autoflicker_en == KAL_TRUE) {
        realtime_fps = imgsensor.pclk /
            imgsensor.line_length * 10 / imgsensor.frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305) {
            realtime_fps = 296;
            set_max_framerate(realtime_fps, 0);
        } else if (realtime_fps >= 147 && realtime_fps <= 150) {
            realtime_fps = 146;
            set_max_framerate(realtime_fps, 0);
        } else {
            imgsensor.frame_length =
                (imgsensor.frame_length  >> 1) << 1;
            write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
            write_cmos_sensor(0x380f,
                      imgsensor.frame_length & 0xFF);
        }
    } else {
        imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;

        write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
    }

    /*
    write_cmos_sensor(0x3500, (shutter >> 12) & 0x0F);   //need to verify
    write_cmos_sensor(0x3501, (shutter >> 4) & 0xFF);
    write_cmos_sensor(0x3502, (shutter<<4)  & 0xF0);
    */
    write_cmos_sensor(0x3501, (shutter >> 8) & 0xFF);
    write_cmos_sensor(0x3502, (shutter)  & 0xFF);

    LOG_INF("shutter =%d, framelength =%d, realtime_fps =%d\n",
        shutter, imgsensor.frame_length, realtime_fps);
}               /* set_shutter_frame_length */


static kal_uint16 gain2reg(const kal_uint16 gain)//check out
{
    kal_uint16 iReg = 0x0000;

    //platform 1xgain = 64, sensor driver 1*gain = 0x100
    iReg = gain*256/BASEGAIN;

    if(iReg < 0x100)    //sensor 1xGain
    {
        iReg = 0X100;
    }
    if(iReg > 0xf80)    //sensor 15.5xGain
    {
        iReg = 0Xf80;
    }
    return iReg;        /* sensorGlobalGain */
}

static kal_uint16 set_gain(kal_uint16 gain)//check out
{
    kal_uint16 reg_gain;
    unsigned long flags;

    reg_gain = gain2reg(gain);
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.gain = reg_gain;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
    write_cmos_sensor(0x03508, (reg_gain >> 8));
    write_cmos_sensor(0x03509, (reg_gain&0xff));

    return gain;
}

static void ihdr_write_shutter_gain(kal_uint16 le,
            kal_uint16 se, kal_uint16 gain)
{
    //LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
}

static void night_mode(kal_bool enable)
{
    /* LOG_INF("night_mode do nothing"); */
}

#if MULTI_WRITE
static kal_uint16 addr_data_pair_init_ov32a1q[] = {
    0x0103, 0x01,
    0x0102, 0x01,
    0x0302, 0x31,
    0x0323, 0x05,
    0x0324, 0x01,
    0x0325, 0x68,
    0x0326, 0xcb,
    0x0327, 0x05,
    0x0343, 0x05,
    0x0346, 0xcf,
    0x300e, 0x22,
    0x3016, 0x96,
    0x3017, 0x78,
    0x3018, 0x70,
    0x3019, 0xd2,
    0x301b, 0x16,
    0x3022, 0xd0,
    0x3028, 0xc3,
    0x3102, 0x00,
    0x3103, 0x0a,
    0x3221, 0x0a,
    0x3400, 0x04,
    0x3408, 0x06,
    0x3508, 0x08,
    0x3548, 0x08,
    0x360b, 0xfc,
    0x360c, 0x11,
    0x3619, 0x80,
    0x361c, 0x80,
    0x361e, 0x87,
    0x3623, 0x55,
    0x3626, 0x89,
    0x3627, 0x11,
    0x3628, 0x88,
    0x3629, 0xce,
    0x3634, 0x0c,
    0x363b, 0x09,
    0x363c, 0x14,
    0x363d, 0x24,
    0x363e, 0x43,
    0x3641, 0x0a,
    0x3642, 0xc8,
    0x3654, 0x0a,
    0x3655, 0xdc,
    0x3656, 0x0f,
    0x3663, 0x81,
    0x3664, 0x00,
    0x3681, 0x08,
    0x3683, 0x11,
    0x3684, 0x40,
    0x3685, 0x10,
    0x3686, 0x11,
    0x368b, 0x06,
    0x3704, 0x22,
    0x3706, 0x4f,
    0x3709, 0x8e,
    0x3711, 0x00,
    0x3724, 0x40,
    0x373f, 0x02,
    0x374f, 0x05,
    0x3765, 0x08,
    0x3767, 0x00,
    0x37cb, 0x11,
    0x37cc, 0x0f,
    0x37d9, 0x08,
    0x37dc, 0x20,
    0x37e3, 0x18,
    0x3823, 0x04,
    0x382a, 0x01,
    0x3834, 0x04,
    0x383d, 0x00,
    0x3860, 0x00,
    0x3861, 0x00,
    0x3889, 0x03,
    0x388b, 0x02,
    0x388d, 0x01,
    0x3d85, 0x85,
    0x3d8c, 0x77,
    0x3d8d, 0xa0,
    0x3d96, 0x0a,
    0x3f01, 0x12,
    0x4009, 0x02,
    0x4021, 0x00,
    0x4023, 0x00,
    0x4024, 0x04,
    0x4025, 0x00,
    0x4026, 0x04,
    0x4027, 0x00,
    0x40c3, 0x0a,
    0x4506, 0x0a,
    0x460a, 0x0a,
    0x464a, 0x0a,
    0x4850, 0x41,
    0x4885, 0x0f, //add for mipi signal integrity fail
    0x4a00, 0x10,
    0x4d01, 0x00,
    0x4d02, 0xb7,
    0x4d03, 0xca,
    0x4d04, 0x30,
    0x4d05, 0x1d,
    0x5004, 0x02,
    0x5181, 0x10,
    0x5182, 0x05,
    0x5183, 0x8f,
    0x5184, 0x01,
    0x522a, 0x44,
    0x522b, 0x44,
    0x522c, 0x14,
    0x522d, 0x44,
    0x5300, 0x7b,
    0x5311, 0x01,
    0x5312, 0x01,
    0x5313, 0x02,
    0x5314, 0x04,
    0x5315, 0x06,
    0x5316, 0x08,
    0x5317, 0x0a,
    0x5318, 0x0c,
    0x5319, 0x0e,
    0x531a, 0x10,
    0x531b, 0x12,
    0x531c, 0x14,
    0x531d, 0x16,
    0x531e, 0x18,
    0x5330, 0x12,
    0x5331, 0x15,
    0x5332, 0x17,
    0x5333, 0x19,
    0x5334, 0x1b,
    0x5335, 0x1d,
    0x5336, 0x1e,
    0x5337, 0x1f,
    0x5338, 0x20,
    0x5339, 0x20,
    0x533a, 0x0c,
    0x533b, 0x02,
    0x533c, 0x68,
    0x5d80, 0x21,
    0x5d82, 0x04,
    0x5d85, 0x19,
    0x5e07, 0x0a,
    0x600b, 0x03,
    0x4853, 0x04,
    0x5500, 0x82, //xtalk step2
    0x5501, 0x89,
    0x5502, 0x88,
    0x5503, 0x8a,
    0x5504, 0x76,
    0x5505, 0x7c,
    0x5506, 0x80,
    0x5507, 0x8d,
    0x5508, 0x74,
    0x5509, 0x71,
    0x550a, 0x77,
    0x550b, 0x7c,
    0x550c, 0x9e,
    0x550d, 0x84,
    0x550e, 0x88,
    0x550f, 0x74,
    0x5510, 0x77,
    0x5511, 0x73,
    0x5512, 0x9e,
    0x5513, 0x9a,
    0x5514, 0x8c,
    0x5515, 0x89,
    0x5516, 0x74,
    0x5517, 0x72,
    0x5518, 0x86,
    0x5519, 0x8d,
    0x551a, 0x75,
    0x551b, 0x77,
    0x551c, 0x71,
    0x551d, 0x73,
    0x551e, 0x83,
    0x551f, 0x89,
    0x5520, 0x73,
    0x5521, 0x7c,
    0x5522, 0x72,
    0x5523, 0x71,
    0x5524, 0x7c,
    0x5525, 0x7c,
    0x5526, 0x64,
    0x5527, 0x7b,
    0x5528, 0x70,
    0x5529, 0x8f,
    0x552a, 0x71,
    0x552b, 0x70,
    0x552c, 0x73,
    0x552d, 0x76,
    0x552e, 0x8a,
    0x552f, 0x81,
    0x5530, 0x7f,
    0x5531, 0x72,
    0x5532, 0x76,
    0x5533, 0x8a,
    0x5534, 0x8d,
    0x5535, 0x87,
    0x5536, 0x7e,
    0x5537, 0x7f,
    0x5538, 0x72,
    0x5539, 0x71,
    0x553a, 0x8c,
    0x553b, 0x86,
    0x553c, 0x7d,
    0x553d, 0x71,
    0x553e, 0x76,
    0x553f, 0x74,
    0x5540, 0x8f,
    0x5541, 0x80,
    0x5542, 0x7d,
    0x5543, 0x77,
    0x5544, 0x8b,
    0x5545, 0x8e,
    0x5546, 0x8c,
    0x5547, 0x8e,
    0x5548, 0x8f,
    0x5549, 0x8f,
    0x554a, 0x9e,
    0x554b, 0x98,
    0x554c, 0x80,
    0x554d, 0x8e,
    0x554e, 0x72,
    0x554f, 0x70,
    0x5550, 0x8d,
    0x5551, 0x8c,
    0x5552, 0x8b,
    0x5553, 0x71,
    0x5554, 0x7c,
    0x5555, 0x7f,
    0x5556, 0x75,
    0x5557, 0x74,
    0x5558, 0x70,
    0x5559, 0x72,
    0x555a, 0x72,
    0x555b, 0x70,
    0x555c, 0x8e,
    0x555d, 0x8e,
    0x555e, 0x77,
    0x555f, 0x70,
    0x5560, 0x77,
    0x5561, 0x74,
    0x5562, 0x8d,
    0x5563, 0x8f,
    0x5564, 0x8b,
    0x5565, 0x77,
    0x5566, 0x89,
    0x5567, 0x8a,
    0x5568, 0x8f,
    0x5569, 0x89,
    0x556a, 0x8b,
    0x556b, 0x8a,
    0x556c, 0x67,
    0x556d, 0x7f,
    0x556e, 0x76,
    0x556f, 0x75,
    0x5570, 0x88,
    0x5571, 0x8a,
    0x5572, 0x60,
    0x5573, 0x7f,
    0x5574, 0x70,
    0x5575, 0x74,
    0x5576, 0x8e,
    0x5577, 0x8a,
    0x5578, 0x68,
    0x5579, 0x78,
    0x557a, 0x71,
    0x557b, 0x8b,
    0x557c, 0x81,
    0x557d, 0x82,
    0x557e, 0x6c,
    0x557f, 0x79,
    0x5580, 0x8a,
    0x5581, 0x8f,
    0x5582, 0x87,
    0x5583, 0x82,
    0x5584, 0x63,
    0x5585, 0x65,
    0x5586, 0x73,
    0x5587, 0x77,
    0x5588, 0x8e,
    0x5589, 0x8b,
    0x558a, 0x6c,
    0x558b, 0x61,
    0x558c, 0x7b,
    0x558d, 0x7e,
    0x558e, 0x76,
    0x558f, 0x71,
    0x5590, 0x85,
    0x5591, 0x8b,
    0x5592, 0x78,
    0x5593, 0x7a,
    0x5594, 0x7d,
    0x5595, 0x77,
    0x5596, 0x93,
    0x5597, 0x82,
    0x5598, 0x76,
    0x5599, 0x72,
    0x559a, 0x70,
    0x559b, 0x89,
    0x559c, 0xae,
    0x559d, 0x98,
    0x559e, 0x88,
    0x559f, 0x71,
    0x55a0, 0x72,
    0x55a1, 0x77,
    0x55a2, 0x94,
    0x55a3, 0x9a,
    0x55a4, 0x76,
    0x55a5, 0x7d,
    0x55a6, 0x7f,
    0x55a7, 0x70,
    0x55a8, 0x93,
    0x55a9, 0x9a,
    0x55aa, 0x8a,
    0x55ab, 0x71,
    0x55ac, 0x70,
    0x55ad, 0x74,
    0x55ae, 0x90,
    0x55af, 0x9e,
    0x55b0, 0x82,
    0x55b1, 0x8e,
    0x55b2, 0x8b,
    0x55b3, 0x88,
    0x55b4, 0x70,
    0x55b5, 0x8c,
    0x55b6, 0x9b,
    0x55b7, 0x99,
    0x55b8, 0x8f,
    0x55b9, 0x75,
    0x55ba, 0x7b,
    0x55bb, 0x77,
    0x55bc, 0x8c,
    0x55bd, 0x82,
    0x55be, 0x76,
    0x55bf, 0x7f,
    0x55c0, 0x61,
    0x55c1, 0x7c,
    0x55c2, 0x75,
    0x55c3, 0x8a,
    0x55c4, 0x72,
    0x55c5, 0x7c,
    0x55c6, 0x7b,
    0x55c7, 0x70,
    0x55c8, 0x8f,
    0x55c9, 0x8c,
    0x55ca, 0x77,
    0x55cb, 0x70,
    0x55cc, 0x7c,
    0x55cd, 0x77,
    0x55ce, 0x8c,
    0x55cf, 0x82,
    0x55d0, 0x8b,
    0x55d1, 0x77,
    0x55d2, 0x7f,
    0x55d3, 0x70,
    0x55d4, 0x88,
    0x55d5, 0x8f,
    0x55d6, 0x8b,
    0x55d7, 0x74,
    0x55d8, 0x8c,
    0x55d9, 0x83,
    0x55da, 0x75,
    0x55db, 0x74,
    0x55dc, 0x88,
    0x55dd, 0x75,
    0x55de, 0x9a,
    0x55df, 0x84,
    0x55e0, 0x8e,
    0x55e1, 0x88,
    0x55e2, 0x8b,
    0x55e3, 0x74,
    0x55e4, 0x9d,
    0x55e5, 0x9c,
    0x55e6, 0x80,
    0x55e7, 0x82,
    0x55e8, 0x88,
    0x55e9, 0x76,
    0x55ea, 0x9c,
    0x55eb, 0x9b,
    0x55ec, 0x8e,
    0x55ed, 0x75,
    0x55ee, 0x75,
    0x55ef, 0x73,
    0x55f0, 0x83,
    0x55f1, 0x8f,
    0x55f2, 0x72,
    0x55f3, 0x7e,
    0x55f4, 0x70,
    0x55f5, 0x72,
    0x55f6, 0x8e,
    0x55f7, 0x75,
    0x55f8, 0x72,
    0x55f9, 0x7c,
    0x55fa, 0x73,
    0x55fb, 0x73,
    0x55fc, 0x74,
    0x55fd, 0x73,
    0x55fe, 0x74,
    0x55ff, 0x75,
    0x5600, 0x74,
    0x5601, 0x88,
    0x5602, 0x7c,
    0x5603, 0x7e,
    0x5604, 0x73,
    0x5605, 0x76,
    0x5606, 0x8b,
    0x5607, 0x8f,
    0x5608, 0x7a,
    0x5609, 0x7a,
    0x560a, 0x7f,
    0x560b, 0x72,
    0x560c, 0x8b,
    0x560d, 0x83,
    0x560e, 0x7e,
    0x560f, 0x7e,
    0x5610, 0x72,
    0x5611, 0x77,
    0x5612, 0x89,
    0x5613, 0x86,
    0x5614, 0x71,
    0x5615, 0x70,
    0x5616, 0x72,
    0x5617, 0x76,
    0x5618, 0x75,
    0x5619, 0x82,
    0x561a, 0x71,
    0x561b, 0x72,
    0x561c, 0x7a,
    0x561d, 0x7b,
    0x561e, 0x7d,
    0x561f, 0x8a,
    0x5620, 0x7d,
    0x5621, 0x74,
    0x5622, 0x89,
    0x5623, 0x8b,
    0x5624, 0x75,
    0x5625, 0x76,
    0x5626, 0x72,
    0x5627, 0x77,
    0x5628, 0x8f,
    0x5629, 0x89,
    0x562a, 0x74,
    0x562b, 0x71,
    0x562c, 0x7d,
    0x562d, 0x70,
    0x562e, 0x89,
    0x562f, 0x89,
    0x5630, 0x77,
    0x5631, 0x72,
    0x5632, 0x7e,
    0x5633, 0x72,
    0x5634, 0x8b,
    0x5635, 0x8b,
    0x5636, 0x76,
    0x5637, 0x7c,
    0x5638, 0x7f,
    0x5639, 0x77,
    0x563a, 0x86,
    0x563b, 0x86,
    0x563c, 0x8e,
    0x563d, 0x76,
    0x563e, 0x76,
    0x563f, 0x8d,
    0x5640, 0x93,
    0x5641, 0x93,
    0x5642, 0x85,
    0x5643, 0x8e,
    0x5644, 0x62,
    0x5645, 0x78,
    0x5646, 0x7f,
    0x5647, 0x7c,
    0x5648, 0x74,
    0x5649, 0x77,
    0x564a, 0x63,
    0x564b, 0x79,
    0x564c, 0x71,
    0x564d, 0x74,
    0x564e, 0x8f,
    0x564f, 0x88,
    0x5650, 0x6c,
    0x5651, 0x7f,
    0x5652, 0x88,
    0x5653, 0x8d,
    0x5654, 0x85,
    0x5655, 0x80,
    0x5656, 0x6a,
    0x5657, 0x7a,
    0x5658, 0x72,
    0x5659, 0x77,
    0x565a, 0x81,
    0x565b, 0x8f,
    0x565c, 0x68,
    0x565d, 0x64,
    0x565e, 0x65,
    0x565f, 0x79,
    0x5660, 0x77,
    0x5661, 0x70,
    0x5662, 0x54,
    0x5663, 0x66,
    0x5664, 0x78,
    0x5665, 0x7f,
    0x5666, 0x7d,
    0x5667, 0x7e,
    0x5668, 0x90,
    0x5669, 0x84,
    0x566a, 0x8d,
    0x566b, 0x8f,
    0x566c, 0x75,
    0x566d, 0x8b,
    0x566e, 0x90,
    0x566f, 0x85,
    0x5670, 0x8a,
    0x5671, 0x76,
    0x5672, 0x70,
    0x5673, 0x74,
    0x5674, 0x94,
    0x5675, 0x87,
    0x5676, 0x71,
    0x5677, 0x7f,
    0x5678, 0x79,
    0x5679, 0x72,
    0x567a, 0xac,
    0x567b, 0x98,
    0x567c, 0x88,
    0x567d, 0x76,
    0x567e, 0x72,
    0x567f, 0x75,
    0x5680, 0x94,
    0x5681, 0x80,
    0x5682, 0x75,
    0x5683, 0x76,
    0x5684, 0x74,
    0x5685, 0x8d,
    0x5686, 0x91,
    0x5687, 0x89,
    0x5688, 0x7e,
    0x5689, 0x78,
    0x568a, 0x71,
    0x568b, 0x8c,
    0x568c, 0x64,
    0x568d, 0x73,
    0x568e, 0x75,
    0x568f, 0x8b,
    0x5690, 0x75,
    0x5691, 0x73,
    0x5692, 0x7a,
    0x5693, 0x70,
    0x5694, 0x89,
    0x5695, 0x8f,
    0x5696, 0x74,
    0x5697, 0x73,
    0x5698, 0x67,
    0x5699, 0x73,
    0x569a, 0x8e,
    0x569b, 0x8f,
    0x569c, 0x76,
    0x569d, 0x72,
    0x569e, 0x62,
    0x569f, 0x7c,
    0x56a0, 0x8a,
    0x56a1, 0x8b,
    0x56a2, 0x72,
    0x56a3, 0x79,
    0x56a4, 0x65,
    0x56a5, 0x8a,
    0x56a6, 0x81,
    0x56a7, 0x86,
    0x56a8, 0x74,
    0x56a9, 0x7e,
    0x56aa, 0x7e,
    0x56ab, 0x81,
    0x56ac, 0x93,
    0x56ad, 0x90,
    0x56ae, 0x83,
    0x56af, 0x76,
    0x56b0, 0x98,
    0x56b1, 0x82,
    0x56b2, 0x86,
    0x56b3, 0x9c,
    0x56b4, 0x98,
    0x56b5, 0x85,
    0x56b6, 0x85,
    0x56b7, 0x8e,
    0x56b8, 0x8d,
    0x56b9, 0x9a,
    0x56ba, 0x84,
    0x56bb, 0x81,
    0x56bc, 0x84,
    0x56bd, 0x76,
    0x56be, 0x73,
    0x56bf, 0x8e,
    0x56c0, 0x8c,
    0x56c1, 0x8a,
    0x56c2, 0x81,
    0x56c3, 0x70,
    0x56c4, 0x7b,
    0x56c5, 0x74,
    0x56c6, 0x77,
    0x56c7, 0x76,
    0x56c8, 0x88,
    0x56c9, 0x72,
    0x56ca, 0x78,
    0x56cb, 0x76,
    0x56cc, 0x71,
    0x56cd, 0x74,
    0x56ce, 0x8a,
    0x56cf, 0x7c,
    0x56d0, 0x7b,
    0x56d1, 0x71,
    0x56d2, 0x8a,
    0x56d3, 0x8a,
    0x56d4, 0x6d,
    0x56d5, 0x7d,
    0x56d6, 0x76,
    0x56d7, 0x78,
    0x56d8, 0x78,
    0x56d9, 0x7e,
    0x56da, 0x6f,
    0x56db, 0x7f,
    0x56dc, 0x77,
    0x56dd, 0x7c,
    0x56de, 0x79,
    0x56df, 0x7f,
    0x56e0, 0x6b,
    0x56e1, 0x7f,
    0x56e2, 0x8f,
    0x56e3, 0x77,
    0x56e4, 0x70,
    0x56e5, 0x75,
    0x56e6, 0x68,
    0x56e7, 0x7d,
    0x56e8, 0x84,
    0x56e9, 0x89,
    0x56ea, 0x89,
    0x56eb, 0x89,
    0x56ec, 0x64,
    0x56ed, 0x77,
    0x56ee, 0x98,
    0x56ef, 0x80,
    0x56f0, 0x8d,
    0x56f1, 0x89,
    0x56f2, 0x7f,
    0x56f3, 0x8f,
    0x56f4, 0x9d,
    0x56f5, 0x85,
    0x56f6, 0x8c,
    0x56f7, 0x8e,
    0x56f8, 0x9b,
    0x56f9, 0x75,
    0x56fa, 0x78,
    0x56fb, 0x7c,
    0x56fc, 0x76,
    0x56fd, 0x75,
    0x56fe, 0x9c,
    0x56ff, 0x8c,
    0x5700, 0x7c,
    0x5701, 0x70,
    0x5702, 0x8b,
    0x5703, 0x88,
    0x5704, 0x91,
    0x5705, 0x87,
    0x5706, 0x71,
    0x5707, 0x74,
    0x5708, 0x89,
    0x5709, 0x8a,
    0x570a, 0x97,
    0x570b, 0x85,
    0x570c, 0x73,
    0x570d, 0x77,
    0x570e, 0x75,
    0x570f, 0x74,
    0x5710, 0x9d,
    0x5711, 0x80,
    0x5712, 0x7e,
    0x5713, 0x72,
    0x5714, 0x71,
    0x5715, 0x77,
    0x5716, 0x9b,
    0x5717, 0x88,
    0x5718, 0x65,
    0x5719, 0x79,
    0x571a, 0x73,
    0x571b, 0x71,
    0x571c, 0x9b,
    0x571d, 0x9a,
    0x571e, 0x99,
    0x571f, 0x81,
    0x5720, 0x8b,
    0x5721, 0x88,
    0x5722, 0x88,
    0x5723, 0x82,
    0x5724, 0x85,
    0x5725, 0x8f,
    0x5726, 0x70,
    0x5727, 0x75,
    0x5728, 0x7c,
    0x5729, 0x76,
    0x572a, 0x88,
    0x572b, 0x7c,
    0x572c, 0x64,
    0x572d, 0x70,
    0x572e, 0x78,
    0x572f, 0x7c,
    0x5730, 0x77,
    0x5731, 0x65,
    0x5732, 0x62,
    0x5733, 0x73,
    0x5734, 0x7e,
    0x5735, 0x7c,
    0x5736, 0x73,
    0x5737, 0x7a,
    0x5738, 0x66,
    0x5739, 0x7f,
    0x573a, 0x73,
    0x573b, 0x73,
    0x573c, 0x7d,
    0x573d, 0x7a,
    0x573e, 0x64,
    0x573f, 0x7b,
    0x5740, 0x67,
    0x5741, 0x78,
    0x5742, 0x78,
    0x5743, 0x71,
    0x5744, 0x77,
    0x5745, 0x7d,
    0x5746, 0x73,
    0x5747, 0x7d,
    0x5748, 0x7f,
    0x5749, 0x77,
    0x574a, 0x75,
    0x574b, 0x7c,
    0x574c, 0x8e,
    0x574d, 0x8a,
    0x574e, 0x77,
    0x574f, 0x8f,
    0x5750, 0x88,
    0x5751, 0x7f,
    0x5752, 0x82,
    0x5753, 0x8d,
    0x5754, 0x88,
    0x5755, 0x87,
    0x5756, 0x82,
    0x5757, 0x7f,
    0x5758, 0x82,
    0x5759, 0x80,
    0x575a, 0x83,
    0x575b, 0x98,
    0x575c, 0x81,
    0x575d, 0x77,
    0x575e, 0x8f,
    0x575f, 0x82,
    0x5760, 0x85,
    0x5761, 0x9d,
    0x5762, 0x9a,
    0x5763, 0x82,
    0x5764, 0x8c,
    0x5765, 0x77,
    0x5766, 0x72,
    0x5767, 0x7e,
    0x5768, 0x76,
    0x5769, 0x8d,
    0x576a, 0x8f,
    0x576b, 0x8a,
    0x576c, 0x76,
    0x576d, 0x72,
    0x576e, 0x8b,
    0x576f, 0x87,
    0x5770, 0x89,
    0x5771, 0x88,
    0x5772, 0x8a,
    0x5773, 0x76,
    0x5774, 0x82,
    0x5775, 0x98,
    0x5776, 0x8a,
    0x5777, 0x75,
    0x5778, 0x75,
    0x5779, 0x71,
    0x577a, 0x8d,
    0x577b, 0x99,
    0x577c, 0x8a,
    0x577d, 0x76,
    0x577e, 0x70,
    0x577f, 0x7c,
    0x5780, 0x8b,
    0x5781, 0x84,
    0x5782, 0x8a,
    0x5783, 0x76,
    0x5784, 0x7f,
    0x5785, 0x7b,
    0x5786, 0x71,
    0x5787, 0x8e,
    0x5788, 0x77,
    0x5789, 0x7c,
    0x578a, 0x64,
    0x578b, 0x7c,
    0x578c, 0x76,
    0x578d, 0x8f,
    0x578e, 0x8e,
    0x578f, 0x7c,
    0x5790, 0x7b,
    0x5791, 0x72,
    0x5792, 0x77,
    0x5793, 0x8b,
    0x5794, 0x87,
    0x5795, 0x76,
    0x5796, 0x7f,
    0x5797, 0x74,
    0x5798, 0x75,
    0x5799, 0x77,
    0x579a, 0x9b,
    0x579b, 0x8b,
    0x579c, 0x75,
    0x579d, 0x82,
    0x579e, 0x82,
    0x579f, 0x89,
    0x57a0, 0x9d,
    0x57a1, 0x87,
    0x57a2, 0x9b,
    0x57a3, 0x90,
    0x57a4, 0x9c,
    0x57a5, 0x9a,
    0x57a6, 0x9c,
    0x57a7, 0x99,
    0x57a8, 0x93,
    0x57a9, 0x95,
    0x57aa, 0x91,
    0x57ab, 0x90,
    0x57ac, 0x71,
    0x57ad, 0x8c,
    0x57ae, 0x93,
    0x57af, 0x9b,
    0x57b0, 0x80,
    0x57b1, 0x75,
    0x57b2, 0x65,
    0x57b3, 0x8a,
    0x57b4, 0x98,
    0x57b5, 0x86,
    0x57b6, 0x8c,
    0x57b7, 0x88,
    0x57b8, 0x69,
    0x57b9, 0x7d,
    0x57ba, 0x80,
    0x57bb, 0x8e,
    0x57bc, 0x88,
    0x57bd, 0x8e,
    0x57be, 0x6a,
    0x57bf, 0x7e,
    0x57c0, 0x89,
    0x57c1, 0x76,
    0x57c2, 0x70,
    0x57c3, 0x75,
    0x57c4, 0x68,
    0x57c5, 0x78,
    0x57c6, 0x71,
    0x57c7, 0x79,
    0x57c8, 0x7b,
    0x57c9, 0x7e,
    0x57ca, 0x63,
    0x57cb, 0x79,
    0x57cc, 0x72,
    0x57cd, 0x78,
    0x57ce, 0x65,
    0x57cf, 0x67,
    0x57d0, 0x82,
    0x57d1, 0x8a,
    0x57d2, 0x7a,
    0x57d3, 0x7e,
    0x57d4, 0x72,
    0x57d5, 0x73,
    0x57d6, 0x9b,
    0x57d7, 0x8d,
    0x57d8, 0x7f,
    0x57d9, 0x72,
    0x57da, 0x71,
    0x57db, 0x73,
    0x57dc, 0x93,
    0x57dd, 0x87,
    0x57de, 0x73,
    0x57df, 0x71,
    0x57e0, 0x77,
    0x57e1, 0x73,
    0x57e2, 0x9c,
    0x57e3, 0x80,
    0x57e4, 0x72,
    0x57e5, 0x71,
    0x57e6, 0x75,
    0x57e7, 0x71,
    0x57e8, 0x84,
    0x57e9, 0x75,
    0x57ea, 0x65,
    0x57eb, 0x79,
    0x57ec, 0x71,
    0x57ed, 0x76,
    0x57ee, 0x8e,
    0x57ef, 0x7d,
    0x57f0, 0x63,
    0x57f1, 0x67,
    0x57f2, 0x7f,
    0x57f3, 0x71,
    0x57f4, 0x7f,
    0x57f5, 0x72,
    0x57f6, 0x72,
    0x57f7, 0x64,
    0x57f8, 0x66,
    0x57f9, 0x7a,
    0x57fa, 0x7c,
    0x57fb, 0x72,
    0x57fc, 0x71,
    0x57fd, 0x78,
    0x57fe, 0x66,
    0x57ff, 0x72,
    0x5800, 0x7e,
    0x5801, 0x73,
    0x5802, 0x8a,
    0x5803, 0x79,
    0x5804, 0x61,
    0x5805, 0x70,
    0x5806, 0x70,
    0x5807, 0x8e,
    0x5808, 0x80,
    0x5809, 0x74,
    0x580a, 0x7c,
    0x580b, 0x8a,
    0x580c, 0x81,
    0x580d, 0x9e,
    0x580e, 0x91,
    0x580f, 0x98,
    0x5810, 0x8f,
    0x5811, 0x81,
    0x5812, 0x9d,
    0x5813, 0x96,
    0x5814, 0xa8,
    0x5815, 0x93,
    0x5816, 0x83,
    0x5817, 0x81,
    0x5818, 0x81,
    0x5819, 0x81,
    0x581a, 0x87,
    0x581b, 0x9c,
    0x581c, 0x9b,
    0x581d, 0x8d,
    0x581e, 0x82,
    0x581f, 0x8d,
    0x5820, 0x8c,
    0x5821, 0x84,
    0x5822, 0x80,
    0x5823, 0x70,
    0x5824, 0x83,
    0x5825, 0x8e,
    0x5826, 0x75,
    0x5827, 0x83,
    0x5828, 0x8c,
    0x5829, 0x7c,
    0x582a, 0x89,
    0x582b, 0x71,
    0x582c, 0x72,
    0x582d, 0x8b,
    0x582e, 0x77,
    0x582f, 0x7b,
    0x5830, 0x7d,
    0x5831, 0x7b,
    0x5832, 0x65,
    0x5833, 0x72,
    0x5834, 0x73,
    0x5835, 0x64,
    0x5836, 0x67,
    0x5837, 0x66,
    0x5838, 0x66,
    0x5839, 0x7f,
    0x583a, 0x71,
    0x583b, 0x79,
    0x583c, 0x7f,
    0x583d, 0x73,
    0x583e, 0x72,
    0x583f, 0x79,
    0x5840, 0x71,
    0x5841, 0x8b,
    0x5842, 0x71,
    0x5843, 0x77,
    0x5844, 0x77,
    0x5845, 0x73,
    0x5846, 0x88,
    0x5847, 0x86,
    0x5848, 0x76,
    0x5849, 0x77,
    0x584a, 0x75,
    0x584b, 0x71,
    0x584c, 0x8c,
    0x584d, 0x85,
    0x584e, 0x74,
    0x584f, 0x75,
    0x5850, 0x77,
    0x5851, 0x70,
    0x5852, 0x8c,
    0x5853, 0x85,
    0x5854, 0x76,
    0x5855, 0x71,
    0x5856, 0x7c,
    0x5857, 0x78,
    0x5858, 0x77,
    0x5859, 0x83,
    0x585a, 0x76,
    0x585b, 0x7d,
    0x585c, 0x7b,
    0x585d, 0x67,
    0x585e, 0x7e,
    0x585f, 0x8a
};
#endif
#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
    static kal_uint16 value_3820=0;
    static kal_uint16 value_3821=0;
    value_3820 = read_cmos_sensor(0x3820);
    value_3821 = read_cmos_sensor(0x3821);
    switch (image_mirror) {
    case IMAGE_NORMAL:
        write_cmos_sensor(0x3820, value_3820&(~(1<<2)));//bit2��0
        write_cmos_sensor(0x3821, value_3821|(1<<2));//bit2��1
        break;

    case IMAGE_V_MIRROR:
        write_cmos_sensor(0x3820, value_3820&(~(1<<2)));
        write_cmos_sensor(0x3821, value_3821&(~(1<<2)));
        break;

    case IMAGE_H_MIRROR:
        write_cmos_sensor(0x3820, value_3820|(1<<2));
        write_cmos_sensor(0x3821, value_3821|(1<<2));
        break;

    case IMAGE_HV_MIRROR:
        write_cmos_sensor(0x3820, value_3820|(1<<2));
        write_cmos_sensor(0x3821, value_3821&(~(1<<2)));
        break;
    }
}
#endif
static void sensor_init(void) //check out
{
#if MULTI_WRITE
    LOG_INF("%s init_setting MULTI_WRITE\n", __func__);
    ov32a1q_table_write_cmos_sensor(
        addr_data_pair_init_ov32a1q,
        sizeof(addr_data_pair_init_ov32a1q) /
        sizeof(kal_uint16));
#endif

    /*set_mirror_flip(imgsensor.mirror);*/

}

#if MULTI_WRITE
static kal_uint16 addr_data_pair_preview_ov32a1q[] = {
    0x0305, 0x64,
    0x0344, 0x01,
    0x0345, 0xe0,
    0x034a, 0x06,
    0x034b, 0x00,
    0x3501, 0x0d,
    0x3502, 0xce,
    0x3603, 0x0b,
    0x3608, 0x4a,
    0x360d, 0x4a,
    0x3622, 0x66,
    0x3633, 0x06,
    0x3635, 0x2c,
    0x3636, 0x2c,
    0x3639, 0x44,
    0x363a, 0x33,
    0x366b, 0x00,
    0x370b, 0xb0,
    0x3712, 0x00,
    0x3714, 0x61,
    0x3800, 0x00,
    0x3801, 0x00,
    0x3802, 0x00,
    0x3803, 0x00,
    0x3804, 0x19,
    0x3805, 0x9f,
    0x3806, 0x13,
    0x3807, 0x3f,
    0x3808, 0x0c,
    0x3809, 0xc0,
    0x380a, 0x09,
    0x380b, 0x90,
    0x380c, 0x03,
    0x380d, 0x48,
    0x380e, 0x0d,
    0x380f, 0xf2,
    0x3811, 0x0a,
    0x3813, 0x09,
    0x3814, 0x22,
    0x3815, 0x22,
    0x3820, 0x05,//vertical flip on
    0x3821, 0x09,//mirror on
    0x3822, 0x00,
    0x4012, 0x0d,
    0x4015, 0x02,
    0x4016, 0x0d,
    0x4018, 0x03,
    0x401e, 0x01,
    0x401f, 0x0c,
    0x4837, 0x06,
    0x5000, 0x89,
    0x5001, 0x02,
    0x5002, 0x01,
    0x5003, 0x7a,
    0x5004, 0x02,
    0x5005, 0x08,
    0x5014, 0x30,
    0x5015, 0x06,
    0x5035, 0x08,
    0x5037, 0x08,
    0x5038, 0x0c,
    0x5039, 0xc0,
    0x503a, 0x09,
    0x503b, 0x90,
    0x5185, 0x0b,
    0x518c, 0x01,
    0x518d, 0x01,
    0x518e, 0x01,
    0x518f, 0x01,
    0x5207, 0xff,
    0x5208, 0xc1,
    0x5380, 0x0c,
    0x5381, 0x06,
    0x5386, 0x14,
    0x5387, 0x60,
    0x5388, 0x0f,
    0x5389, 0xc8,
    0x5880, 0xc5,
    0x5884, 0x18,
    0x5885, 0x08,
    0x5886, 0x08,
    0x5887, 0x18,
    0x5889, 0x05,
    0x588a, 0x00,
    0x58c0, 0x10,
    0x58c2, 0x0e,
    0x58c3, 0x0c,
    0x58c4, 0x04,
    0x58c5, 0x01,
    0x58c6, 0xf7,
    0x58c8, 0x6f,
    0x58ca, 0x1d,
    0x58cb, 0x01,
    0x58cc, 0xfd,
    0x0305, 0x29,
    0x4837, 0x10
};
#endif

static void preview_setting(void)//jack yan check
{
    LOG_INF("%s PREVIEW_RES_3264*2448_30fps\n", __func__);
#if MULTI_WRITE
    ov32a1q_table_write_cmos_sensor(
        addr_data_pair_preview_ov32a1q,
        sizeof(addr_data_pair_preview_ov32a1q) /
        sizeof(kal_uint16));

#endif
#if 0
    #ifdef OV32A1Q_SYNC_OPEN
    write_cmos_senor(0x3002, 0x21);
    write_cmos_senor(0x3832, 0x08);
    write_cmos_senor(0x3833, 0x30);
    #endif
#endif

}



static kal_uint16 addr_data_pair_capture_ov32a1q[] = {
    0x0305, 0x64,
    0x0344, 0x01,
    0x0345, 0xe0,
    0x034a, 0x06,
    0x034b, 0x00,
    0x3501, 0x0d,
    0x3502, 0xce,
    0x3603, 0x0b,
    0x3608, 0x4a,
    0x360d, 0x4a,
    0x3622, 0x66,
    0x3633, 0x06,
    0x3635, 0x2c,
    0x3636, 0x2c,
    0x3639, 0x44,
    0x363a, 0x33,
    0x366b, 0x00,
    0x370b, 0xb0,
    0x3712, 0x00,
    0x3714, 0x61,
    0x3800, 0x00,
    0x3801, 0x00,
    0x3802, 0x00,
    0x3803, 0x00,
    0x3804, 0x19,
    0x3805, 0x9f,
    0x3806, 0x13,
    0x3807, 0x3f,
    0x3808, 0x0c,
    0x3809, 0xc0,
    0x380a, 0x09,
    0x380b, 0x90,
    0x380c, 0x03,
    0x380d, 0x48,
    0x380e, 0x0d,
    0x380f, 0xf2,
    0x3811, 0x0a,
    0x3813, 0x09,
    0x3814, 0x22,
    0x3815, 0x22,
    0x3820, 0x05,//vertical flip on
    0x3821, 0x09,//mirror on
    0x3822, 0x00,
    0x4012, 0x0d,
    0x4015, 0x02,
    0x4016, 0x0d,
    0x4018, 0x03,
    0x401e, 0x01,
    0x401f, 0x0c,
    0x4837, 0x06,
    0x5000, 0x89,
    0x5001, 0x02,
    0x5002, 0x01,
    0x5003, 0x7a,
    0x5004, 0x02,
    0x5005, 0x08,
    0x5014, 0x30,
    0x5015, 0x06,
    0x5035, 0x08,
    0x5037, 0x08,
    0x5038, 0x0c,
    0x5039, 0xc0,
    0x503a, 0x09,
    0x503b, 0x90,
    0x5185, 0x0b,
    0x518c, 0x01,
    0x518d, 0x01,
    0x518e, 0x01,
    0x518f, 0x01,
    0x5207, 0xff,
    0x5208, 0xc1,
    0x5380, 0x0c,
    0x5381, 0x06,
    0x5386, 0x14,
    0x5387, 0x60,
    0x5388, 0x0f,
    0x5389, 0xc8,
    0x5880, 0xc5,
    0x5884, 0x18,
    0x5885, 0x08,
    0x5886, 0x08,
    0x5887, 0x18,
    0x5889, 0x05,
    0x588a, 0x00,
    0x58c0, 0x10,
    0x58c2, 0x0e,
    0x58c3, 0x0c,
    0x58c4, 0x04,
    0x58c5, 0x01,
    0x58c6, 0xf7,
    0x58c8, 0x6f,
    0x58ca, 0x1d,
    0x58cb, 0x01,
    0x58cc, 0xfd,
    0x0305, 0x29,
    0x4837, 0x10
};
static void capture_setting(kal_uint16 currefps)//jack yan check
{
    LOG_INF("capture RES_6528*4896_15FPS currefps = %d\n",
        currefps);
#if MULTI_WRITE

    ov32a1q_table_write_cmos_sensor(
        addr_data_pair_capture_ov32a1q,
        sizeof(addr_data_pair_capture_ov32a1q) /
        sizeof(kal_uint16));

#if 0
    #ifdef OV32A1Q_SYNC_OPEN

    write_cmos_sensor(0x3002, 0x61);
    write_cmos_sensor(0x3832, 0x18);
    write_cmos_sensor(0x3833, 0x10);
    write_cmos_sensor(0x3818, 0x02);
    write_cmos_sensor(0x3819, 0x44);
    write_cmos_sensor(0x381a, 0x1c);
    write_cmos_sensor(0x381b, 0xe0);

    #endif
#endif

#endif
}
#if MULTI_WRITE
static kal_uint16 addr_data_pair_normal_video_ov32a1q[] = {
    0x0100, 0x00,
    0x0305, 0x64,
    0x0344, 0x01,
    0x0345, 0xe0,
    0x034a, 0x06,
    0x034b, 0x00,
    0x3501, 0x0d,
    0x3502, 0xce,
    0x3603, 0x0b,
    0x3608, 0x4a,
    0x360d, 0x4a,
    0x3622, 0x66,
    0x3633, 0x06,
    0x3635, 0x2c,
    0x3636, 0x2c,
    0x3639, 0x44,
    0x363a, 0x33,
    0x366b, 0x00,
    0x370b, 0xb0,
    0x3712, 0x00,
    0x3714, 0x61,
    0x3800, 0x00,
    0x3801, 0x00,
    0x3802, 0x02,
    0x3803, 0x64,
    0x3804, 0x19,
    0x3805, 0x9f,
    0x3806, 0x10,
    0x3807, 0xdb,
    0x3808, 0x0c,
    0x3809, 0xc0,
    0x380a, 0x07,
    0x380b, 0x2c,
    0x380c, 0x03,
    0x380d, 0x48,
    0x380e, 0x0d,
    0x380f, 0xf2,
    0x3811, 0x0a,
    0x3813, 0x0a,
    0x3814, 0x22,
    0x3815, 0x22,
    0x3820, 0x05,
    0x3821, 0x09,
    0x3822, 0x00,
    0x4012, 0x0d,
    0x4015, 0x02,
    0x4016, 0x0d,
    0x4018, 0x03,
    0x401e, 0x01,
    0x401f, 0x0c,
    0x4837, 0x06,
    0x5000, 0x89,
    0x5001, 0x02,
    0x5002, 0x01,
    0x5003, 0x7a,
    0x5005, 0x08,
    0x5014, 0x30,
    0x5015, 0x06,
    0x5035, 0x08,
    0x5037, 0x08,
    0x5038, 0x0c,
    0x5039, 0xc0,
    0x503a, 0x09,
    0x503b, 0x90,
    0x5185, 0x0b,
    0x518c, 0x01,
    0x518d, 0x01,
    0x518e, 0x01,
    0x518f, 0x01,
    0x5207, 0xff,
    0x5208, 0xc1,
    0x5380, 0x0c,
    0x5381, 0x06,
    0x5386, 0x14,
    0x5387, 0x60,
    0x5388, 0x0f,
    0x5389, 0xc8,
    0x5880, 0xc5,
    0x5884, 0x18,
    0x5885, 0x08,
    0x5886, 0x08,
    0x5887, 0x18,
    0x5889, 0x05,
    0x588a, 0x00,
    0x58c0, 0x10,
    0x58c2, 0x0e,
    0x58c3, 0x0c,
    0x58c4, 0x04,
    0x58c5, 0x01,
    0x58c6, 0xf7,
    0x58c8, 0x6f,
    0x58ca, 0x1d,
    0x58cb, 0x01,
    0x58cc, 0xfd,
    0x0305, 0x29,
    0x4837, 0x10,
    0x0100, 0x01,
};
#endif
static void normal_video_setting(kal_uint16 currefps)
{
    LOG_INF("%s NORMAL_VIDEO_SETTING_RES_1280*720_120FPS\n", __func__);
#if MULTI_WRITE
    ov32a1q_table_write_cmos_sensor(
        addr_data_pair_normal_video_ov32a1q,
        sizeof(addr_data_pair_normal_video_ov32a1q) /
        sizeof(kal_uint16));

#endif
#if 0
    #ifdef OV32A1Q_SYNC_OPEN
    write_cmos_senor(0x3002, 0x21);
    write_cmos_senor(0x3832, 0x08);
    write_cmos_senor(0x3833, 0x30);
    #endif
#endif
}

#if MULTI_WRITE
static kal_uint16 addr_data_pair_hs_video_ov32a1q[] = {
    0x0305, 0x3f,
    0x0344, 0x02,
    0x0345, 0x00,
    0x034a, 0x03,
    0x034b, 0x02,
    0x3501, 0x03,
    0x3502, 0x9e,
    0x3603, 0x2b,
    0x3608, 0x4a,
    0x360d, 0x4a,
    0x3622, 0x66,
    0x3633, 0x06,
    0x3635, 0x2c,
    0x3636, 0x2c,
    0x3639, 0x44,
    0x363a, 0x33,
    0x366b, 0x00,
    0x370b, 0xb0,
    0x3712, 0x00,
    0x3714, 0x61,
    0x3800, 0x02,
    0x3801, 0xc0,
    0x3802, 0x03,
    0x3803, 0xf0,
    0x3804, 0x16,
    0x3805, 0xdf,
    0x3806, 0x0f,
    0x3807, 0x4f,
    0x3808, 0x05,
    0x3809, 0x00,
    0x380a, 0x02,
    0x380b, 0xd0,
    0x380c, 0x01,
    0x380d, 0x86,
    0x380e, 0x03,
    0x380f, 0xc2,
    0x3811, 0x06,
    0x3813, 0x05,
    0x3814, 0x22,
    0x3815, 0x26,
    0x3820, 0x05,//vertical flip on
    0x3821, 0x09,//mirror on
    0x3822, 0x01,
    0x4012, 0x0d,
    0x4015, 0x02,
    0x4016, 0x05,
    0x4018, 0x01,
    0x401e, 0x01,
    0x401f, 0x0c,
    0x4837, 0x0a,
    0x5000, 0xa9,
    0x5001, 0x22,
    0x5002, 0x11,
    0x5003, 0x7a,
    0x5004, 0x02,
    0x5005, 0x00,
    0x5014, 0x00,
    0x5015, 0x08,
    0x5035, 0x08,
    0x5037, 0x04,
    0x5038, 0x0c,
    0x5039, 0xc0,
    0x503a, 0x04,
    0x503b, 0xc8,
    0x5185, 0x0b,
    0x518c, 0x01,
    0x518d, 0x01,
    0x518e, 0x01,
    0x518f, 0x01,
    0x5207, 0xff,
    0x5208, 0xc1,
    0x5380, 0x0c,
    0x5381, 0x06,
    0x5386, 0x14,
    0x5387, 0x60,
    0x5388, 0x0f,
    0x5389, 0xc8,
    0x5880, 0xc1,
    0x5884, 0x00,
    0x5885, 0x00,
    0x5886, 0x10,
    0x5887, 0x10,
    0x5889, 0x04,
    0x588a, 0x00,
    0x58c0, 0x34,
    0x58c2, 0x2e,
    0x58c3, 0x02,
    0x58c4, 0x0e,
    0x58c5, 0x00,
    0x58c6, 0x00,
    0x58c8, 0x50,
    0x58ca, 0x30,
    0x58cb, 0x00,
    0x58cc, 0x00,
    0x0305, 0x23,
    0x4837, 0x13
};
#endif

static void hs_video_setting(void)
{
    LOG_INF("%s HS_VIDEO_RES_1280x720_240fps\n", __func__);
#if MULTI_WRITE
    ov32a1q_table_write_cmos_sensor(
        addr_data_pair_hs_video_ov32a1q,
        sizeof(addr_data_pair_hs_video_ov32a1q) /
        sizeof(kal_uint16));

#endif
#if 0
    #ifdef OV32A1Q_SYNC_OPEN
    write_cmos_senor(0x3002, 0x21);
    write_cmos_senor(0x3832, 0x08);
    write_cmos_senor(0x3833, 0x30);
    #endif
#endif
}

#if MULTI_WRITE
static kal_uint16 addr_data_pair_slim_video_ov32a1q[] = {
    0x0305, 0x64,
    0x0344, 0x01,
    0x0345, 0xe0,
    0x034a, 0x06,
    0x034b, 0x00,
    0x3501, 0x0d,
    0x3502, 0xce,
    0x3603, 0x0b,
    0x3608, 0x4a,
    0x360d, 0x4a,
    0x3622, 0x66,
    0x3633, 0x06,
    0x3635, 0x2c,
    0x3636, 0x2c,
    0x3639, 0x44,
    0x363a, 0x33,
    0x366b, 0x00,
    0x370b, 0xb0,
    0x3712, 0x00,
    0x3714, 0x61,
    0x3800, 0x00,
    0x3801, 0x00,
    0x3802, 0x00,
    0x3803, 0x00,
    0x3804, 0x19,
    0x3805, 0x9f,
    0x3806, 0x13,
    0x3807, 0x3f,
    0x3808, 0x0c,
    0x3809, 0xc0,
    0x380a, 0x09,
    0x380b, 0x90,
    0x380c, 0x01,
    0x380d, 0xa4,
    0x380e, 0x0d,
    0x380f, 0xf2,
    0x3811, 0x0a,
    0x3813, 0x09,
    0x3814, 0x22,
    0x3815, 0x22,
    0x3820, 0x05,//vertical flip on
    0x3821, 0x09,//mirror on
    0x3822, 0x00,
    0x4012, 0x0d,
    0x4015, 0x02,
    0x4016, 0x0d,
    0x4018, 0x03,
    0x401e, 0x01,
    0x401f, 0x0c,
    0x4837, 0x06,
    0x5000, 0x89,
    0x5001, 0x02,
    0x5002, 0x01,
    0x5003, 0x7a,
    0x5004, 0x02,
    0x5005, 0x08,
    0x5014, 0x30,
    0x5015, 0x06,
    0x5035, 0x08,
    0x5037, 0x08,
    0x5038, 0x0c,
    0x5039, 0xc0,
    0x503a, 0x09,
    0x503b, 0x90,
    0x5185, 0x0b,
    0x518c, 0x01,
    0x518d, 0x01,
    0x518e, 0x01,
    0x518f, 0x01,
    0x5207, 0xff,
    0x5208, 0xc1,
    0x5380, 0x0c,
    0x5381, 0x06,
    0x5386, 0x14,
    0x5387, 0x60,
    0x5388, 0x0f,
    0x5389, 0xc8,
    0x5880, 0xc5,
    0x5884, 0x18,
    0x5885, 0x08,
    0x5886, 0x08,
    0x5887, 0x18,
    0x5889, 0x05,
    0x588a, 0x00,
    0x58c0, 0x10,
    0x58c2, 0x0e,
    0x58c3, 0x0c,
    0x58c4, 0x04,
    0x58c5, 0x01,
    0x58c6, 0xf7,
    0x58c8, 0x6f,
    0x58ca, 0x1d,
    0x58cb, 0x01,
    0x58cc, 0xfd,
    0x0305, 0x52,
    0x4837, 0x08
};
#endif

static void slim_video_setting(void)
{
    LOG_INF("%s RES_3260x2448_60fps\n", __func__);
#if MULTI_WRITE
    ov32a1q_table_write_cmos_sensor(
        addr_data_pair_slim_video_ov32a1q,
        sizeof(addr_data_pair_slim_video_ov32a1q) /
        sizeof(kal_uint16));

#endif
#if 0
    #ifdef OV32A1Q_SYNC_OPEN
    write_cmos_senor(0x3002, 0x21);
    write_cmos_senor(0x3832, 0x08);
    write_cmos_senor(0x3833, 0x30);
    #endif
#endif
}

#if MULTI_WRITE
static kal_uint16 addr_data_pair_custom1_ov32a1q[] = {
    0x0305, 0x64,
    0x0344, 0x01,
    0x0345, 0xe0,
    0x034a, 0x06,
    0x034b, 0x00,
    0x3501, 0x06,
    0x3502, 0xea,
    0x3603, 0x0b,
    0x3608, 0x4a,
    0x360d, 0x4a,
    0x3622, 0x66,
    0x3633, 0x06,
    0x3635, 0x2c,
    0x3636, 0x2c,
    0x3639, 0x44,
    0x363a, 0x33,
    0x366b, 0x00,
    0x370b, 0xb0,
    0x3712, 0x00,
    0x3714, 0x61,
    0x3800, 0x02,
    0x3801, 0x40,
    0x3802, 0x03,
    0x3803, 0xa8,
    0x3804, 0x17,
    0x3805, 0x5f,
    0x3806, 0x0f,
    0x3807, 0x97,
    0x3808, 0x0a,
    0x3809, 0x80,
    0x380a, 0x05,
    0x380b, 0xe8,
    0x380c, 0x01,
    0x380d, 0x1a4,
    0x380e, 0x06,
    0x380f, 0xf8,
    0x3811, 0x0a,
    0x3813, 0x09,
    0x3814, 0x22,
    0x3815, 0x22,
    0x3820, 0x05,//vertical flip on
    0x3821, 0x09,//mirror on
    0x3822, 0x00,
    0x4012, 0x0d,
    0x4015, 0x02,
    0x4016, 0x0d,
    0x4018, 0x03,
    0x401e, 0x01,
    0x401f, 0x0c,
    0x4837, 0x06,
    0x5000, 0x89,
    0x5001, 0x02,
    0x5002, 0x01,
    0x5003, 0x7a,
    0x5004, 0x02,
    0x5005, 0x08,
    0x5014, 0x30,
    0x5015, 0x06,
    0x5035, 0x08,
    0x5037, 0x08,
    0x5038, 0x0c,
    0x5039, 0xc0,
    0x503a, 0x09,
    0x503b, 0x90,
    0x5185, 0x0b,
    0x518c, 0x01,
    0x518d, 0x01,
    0x518e, 0x01,
    0x518f, 0x01,
    0x5207, 0xff,
    0x5208, 0xc1,
    0x5380, 0x0c,
    0x5381, 0x06,
    0x5386, 0x14,
    0x5387, 0x60,
    0x5388, 0x0f,
    0x5389, 0xc8,
    0x5880, 0xc5,
    0x5884, 0x18,
    0x5885, 0x08,
    0x5886, 0x08,
    0x5887, 0x18,
    0x5889, 0x05,
    0x588a, 0x00,
    0x58c0, 0x10,
    0x58c2, 0x0e,
    0x58c3, 0x0c,
    0x58c4, 0x04,
    0x58c5, 0x01,
    0x58c6, 0xf7,
    0x58c8, 0x6f,
    0x58ca, 0x1d,
    0x58cb, 0x01,
    0x58cc, 0xfd,
    0x0305, 0x52,
    0x4837, 0x08,
    0x0100, 0x01
};
#endif

static void custom1_setting(void)
{
    LOG_INF("%s CUSTOM1_RES_1920*1080_120fps\n", __func__);
#if MULTI_WRITE
    ov32a1q_table_write_cmos_sensor(
        addr_data_pair_custom1_ov32a1q,
        sizeof(addr_data_pair_custom1_ov32a1q) /
        sizeof(kal_uint16));

#endif
#if 0
    #ifdef OV32A1Q_SYNC_OPEN
    write_cmos_senor(0x3002, 0x21);
    write_cmos_senor(0x3832, 0x08);
    write_cmos_senor(0x3833, 0x30);
    #endif
#endif
}

static kal_uint16 addr_data_pair_custom2_ov32a1q[] = {
    0x0305, 0x64,
    0x0344, 0x01,
    0x0345, 0xe0,
    0x034a, 0x06,
    0x034b, 0x00,
    0x3501, 0x0d,
    0x3502, 0xce,
    0x3603, 0x0b,
    0x3608, 0x4a,
    0x360d, 0x4a,
    0x3622, 0x66,
    0x3633, 0x06,
    0x3635, 0x2c,
    0x3636, 0x2c,
    0x3639, 0x44,
    0x363a, 0x33,
    0x366b, 0x00,
    0x370b, 0xb0,
    0x3712, 0x00,
    0x3714, 0x61,
    0x3800, 0x00,
    0x3801, 0x00,
    0x3802, 0x00,
    0x3803, 0x00,
    0x3804, 0x19,
    0x3805, 0x9f,
    0x3806, 0x13,
    0x3807, 0x3f,
    0x3808, 0x0c,
    0x3809, 0xc0,
    0x380a, 0x09,
    0x380b, 0x90,
    0x380c, 0x03,
    0x380d, 0x48,
    0x380e, 0x0d,
    0x380f, 0xf2,
    0x3811, 0x0a,
    0x3813, 0x09,
    0x3814, 0x22,
    0x3815, 0x22,
    0x3820, 0x05,//vertical flip on
    0x3821, 0x09,//mirror on
    0x3822, 0x00,
    0x4012, 0x0d,
    0x4015, 0x02,
    0x4016, 0x0d,
    0x4018, 0x03,
    0x401e, 0x01,
    0x401f, 0x0c,
    0x4837, 0x06,
    0x5000, 0x89,
    0x5001, 0x02,
    0x5002, 0x01,
    0x5003, 0x7a,
    0x5004, 0x02,
    0x5005, 0x08,
    0x5014, 0x30,
    0x5015, 0x06,
    0x5035, 0x08,
    0x5037, 0x08,
    0x5038, 0x0c,
    0x5039, 0xc0,
    0x503a, 0x09,
    0x503b, 0x90,
    0x5185, 0x0b,
    0x518c, 0x01,
    0x518d, 0x01,
    0x518e, 0x01,
    0x518f, 0x01,
    0x5207, 0xff,
    0x5208, 0xc1,
    0x5380, 0x0c,
    0x5381, 0x06,
    0x5386, 0x14,
    0x5387, 0x60,
    0x5388, 0x0f,
    0x5389, 0xc8,
    0x5880, 0xc5,
    0x5884, 0x18,
    0x5885, 0x08,
    0x5886, 0x08,
    0x5887, 0x18,
    0x5889, 0x05,
    0x588a, 0x00,
    0x58c0, 0x10,
    0x58c2, 0x0e,
    0x58c3, 0x0c,
    0x58c4, 0x04,
    0x58c5, 0x01,
    0x58c6, 0xf7,
    0x58c8, 0x6f,
    0x58ca, 0x1d,
    0x58cb, 0x01,
    0x58cc, 0xfd,
    0x0305, 0x29,
    0x4837, 0x10
};


static void custom2_setting(void)
{
    LOG_INF("%s CUSTOM2_RES_1920*1080_120fps\n", __func__);
#if MULTI_WRITE
    ov32a1q_table_write_cmos_sensor(
        addr_data_pair_custom2_ov32a1q,
        sizeof(addr_data_pair_custom2_ov32a1q) /
        sizeof(kal_uint16));

#endif
#if 0
    #ifdef OV32A1Q_SYNC_OPEN
    write_cmos_senor(0x3002, 0x21);
    write_cmos_senor(0x3832, 0x08);
    write_cmos_senor(0x3833, 0x30);
    #endif
#endif
}

static kal_uint16 addr_data_pair_custom3_ov32a1q[] = {
    0x0303, 0x00,
    0x0305, 0x38,
    0x0305, 0x3f,
    0x0344, 0x01,
    0x0345, 0xce,
    0x034a, 0x0a,
    0x034b, 0x00,
    0x3501, 0x13,
    0x3502, 0x64,
    0x3603, 0x0b,
    0x3608, 0x63,
    0x360d, 0x61,
    0x3622, 0x55,
    0x3633, 0x03,
    0x3635, 0x0c,
    0x3636, 0x0c,
    0x3639, 0xcc,
    0x363a, 0xcc,
    0x366b, 0x02,
    0x370b, 0xb0,
    0x3712, 0x00,
    0x3714, 0x67,
    0x3800, 0x00,
    0x3801, 0x00,
    0x3802, 0x00,
    0x3803, 0x00,
    0x3804, 0x19,
    0x3805, 0x9f,
    0x3806, 0x13,
    0x3807, 0x3f,
    0x3808, 0x19,
    0x3809, 0x80,
    0x380a, 0x13,
    0x380b, 0x20,
    0x380c, 0x04,
    0x380d, 0xb0,
    0x380e, 0x14,
    0x380f, 0x00,
    0x3811, 0x12,
    0x3813, 0x11,
    0x3814, 0x11,
    0x3815, 0x11,
    0x3820, 0x04,//vertical flip on
    0x3821, 0x00,//mirror on
    0x3822, 0x00,
    0x4012, 0x7d,
    0x4015, 0x04,
    0x4016, 0x1b,
    0x4018, 0x07,
    0x401e, 0x01,
    0x401f, 0x0c,
    0x4837, 0x0a,
    0x5000, 0xc9,
    0x5001, 0x42,
    0x5002, 0x01,
    0x5003, 0x7a,
    0x5004, 0x02,
    0x5005, 0x00,
    0x5014, 0x00,
    0x5015, 0x06,
    0x5035, 0x10,
    0x5037, 0x10,
    0x5038, 0x19,
    0x5039, 0x80,
    0x503a, 0x13,
    0x503b, 0x20,
    0x5185, 0x0c,
    0x518c, 0x01,
    0x518d, 0x01,
    0x518e, 0x01,
    0x518f, 0x01,
    0x5207, 0x05,
    0x5208, 0x41,
    0x5380, 0x0f,
    0x5381, 0x00,
    0x5386, 0x19,
    0x5387, 0xa0,
    0x5388, 0x13,
    0x5389, 0x40,
    0x5880, 0xc5,
    0x5884, 0x18,
    0x5885, 0x08,
    0x5886, 0x08,
    0x5887, 0x18,
    0x5889, 0x05,
    0x588a, 0x00,
    0x58c0, 0x10,
    0x58c2, 0x0e,
    0x58c3, 0x0c,
    0x58c4, 0x04,
    0x58c5, 0x01,
    0x58c6, 0xf7,
    0x58c8, 0x6f,
    0x58ca, 0x1d,
    0x58cb, 0x01,
    0x58cc, 0xfd,
    0x5c6f, 0x02,
    0x5c71, 0x00,
    0x0305, 0x3e,
    0x4837, 0x0a
};
static void custom3_setting(void)
{
    LOG_INF("%s CUSTOM3_RES_6528*4896_15fps\n", __func__);
#if MULTI_WRITE
    ov32a1q_table_write_cmos_sensor(
        addr_data_pair_custom3_ov32a1q,
        sizeof(addr_data_pair_custom3_ov32a1q) /
        sizeof(kal_uint16));

#endif
#if 0
    #ifdef OV32A1Q_SYNC_OPEN
    write_cmos_senor(0x3002, 0x21);
    write_cmos_senor(0x3832, 0x08);
    write_cmos_senor(0x3833, 0x30);
    #endif
#endif
}
static kal_uint16 addr_data_pair_custom4_ov32a1q[] = {
    0x0305, 0x64,
    0x0344, 0x01,
    0x0345, 0xe0,
    0x034a, 0x05,
    0x034b, 0x00,
    0x3501, 0x22,
    0x3502, 0xbe,
    0x3603, 0x0b,
    0x3608, 0x4a,
    0x360d, 0x4a,
    0x3622, 0x66,
    0x3633, 0x06,
    0x3635, 0x2c,
    0x3636, 0x2c,
    0x3639, 0x44,
    0x363a, 0x33,
    0x366b, 0x00,
    0x370b, 0xb0,
    0x3712, 0x00,
    0x3714, 0x61,
    0x3800, 0x00,
    0x3801, 0x00,
    0x3802, 0x00,
    0x3803, 0x00,
    0x3804, 0x19,
    0x3805, 0x9f,
    0x3806, 0x13,
    0x3807, 0x3f,
    0x3808, 0x06,
    0x3809, 0x60,
    0x380a, 0x04,
    0x380b, 0xc8,
    0x380c, 0x01,
    0x380d, 0xfa,
    0x380e, 0x17,
    0x380f, 0x28,
    0x3811, 0x06,
    0x3813, 0x05,
    0x3814, 0x22,
    0x3815, 0x22,
    0x3820, 0x05,//vertical flip on
    0x3821, 0x09,//mirror on
    0x3822, 0x11,
    0x4012, 0x0d,
    0x4015, 0x02,
    0x4016, 0x0d,
    0x4018, 0x03,
    0x401e, 0x01,
    0x401f, 0x0c,
    0x4837, 0x08,
    0x5000, 0xa9,
    0x5001, 0x22,
    0x5002, 0x11,
    0x5003, 0x7a,
    0x5005, 0x00,
    0x5014, 0x00,
    0x5015, 0x06,
    0x5035, 0x10,
    0x5037, 0x10,
    0x5038, 0x19,
    0x5039, 0x80,
    0x503a, 0x13,
    0x503b, 0x20,
    0x5185, 0x0b,
    0x518c, 0x01,
    0x518d, 0x01,
    0x518e, 0x01,
    0x518f, 0x01,
    0x5207, 0xff,
    0x5208, 0xc1,
    0x5380, 0x0c,
    0x5381, 0x06,
    0x5386, 0x14,
    0x5387, 0x60,
    0x5388, 0x0f,
    0x5389, 0xc8,
    0x5880, 0xc7,
    0x5884, 0x18,
    0x5885, 0x08,
    0x5886, 0x08,
    0x5887, 0x18,
    0x5889, 0x05,
    0x588a, 0x00,
    0x58c0, 0x10,
    0x58c2, 0x0e,
    0x58c3, 0x0c,
    0x58c4, 0x04,
    0x58c5, 0x01,
    0x58c6, 0xf7,
    0x58c8, 0x6f,
    0x58ca, 0x1d,
    0x58cb, 0x01,
    0x58cc, 0xfd,
    0x5c6f, 0x10,
    0x5c71, 0x10,
    0x0305, 0x28,
    0x4837, 0x10,
};

static void custom4_setting(void)
{
    LOG_INF("E\n");
    ov32a1q_table_write_cmos_sensor(
        addr_data_pair_custom4_ov32a1q,
        sizeof(addr_data_pair_custom4_ov32a1q) /
        sizeof(kal_uint16));
}
static kal_uint32 return_sensor_id(void)
{
    return ((read_cmos_sensor(0x300a) << 16) |(read_cmos_sensor(0x300b) << 8) |read_cmos_sensor(0x300c));

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
            LOG_INF("* ov32a get_sensor_id 0x%x!",*sensor_id);
            if (*sensor_id == OV32A1Q_SENSOR_ID_20630) {
                *sensor_id = imgsensor_info.sensor_id;
                LOG_INF("ov32a1q read[%s] sensor id: 0x%x\n",
                    __func__, *sensor_id);
                #ifdef OPLUS_FEATURE_CAMERA_COMMON
                read_sensor_xtalk();
                #endif
                return ERROR_NONE;
            }

            retry--;
        } while (retry > 0);
        i++;
        retry = 1;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        LOG_INF("%s failed: 0x%x\n", __func__, *sensor_id);
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    return ERROR_NONE;
}

static kal_uint32 open(void) //check out
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
            if (sensor_id == OV32A1Q_SENSOR_ID_20630) {
                sensor_id = imgsensor_info.sensor_id;
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
        LOG_INF("Open sensor id fail: 0x%x\n", sensor_id);
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    sensor_init();//jack_yan

    spin_lock(&imgsensor_drv_lock);
    imgsensor.autoflicker_en = KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.shutter = 0x3D0;   //jack
    imgsensor.gain    = 0x100;   //jack
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    imgsensor.pdaf_mode = 0;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}

static kal_uint32 close(void)
{
    #ifdef OPLUS_FEATURE_CAMERA_COMMON
    xtalk_flag = 0;
    #endif
    return ERROR_NONE;
}   /*  close  */

static kal_uint32 preview(
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s setting E\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.current_fps = imgsensor.current_fps;
    //imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}

static kal_uint32 capture(
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
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

    return ERROR_NONE;
}   /* capture() */

static kal_uint32 normal_video(
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length =
        imgsensor_info.normal_video.linelength;
    imgsensor.frame_length =
        imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length =
        imgsensor_info.normal_video.framelength;
    spin_unlock(&imgsensor_drv_lock);
    normal_video_setting(imgsensor.current_fps);
    return ERROR_NONE;
}

static kal_uint32 hs_video(
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
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
}

static kal_uint32 slim_video(
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length =
        imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();
    return ERROR_NONE;
}

static kal_uint32 custom1(
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.current_fps = imgsensor.current_fps;
    spin_unlock(&imgsensor_drv_lock);
    custom1_setting();
    return ERROR_NONE;
}

static kal_uint32 custom2(
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.current_fps = imgsensor.current_fps;
    spin_unlock(&imgsensor_drv_lock);
    custom2_setting();
    return ERROR_NONE;
}

static kal_uint32 custom3(
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    #ifdef OPLUS_FEATURE_CAMERA_COMMON
    if (!xtalk_flag) {
        pr_debug("write_sensor_xtalk Start\n");
                mdelay(67);  //add delay to ensure xtalk wirte success
        write_sensor_xtalk();
        pr_debug("write_sensor_xtlak End\n");
        xtalk_flag = 1;
    }
    #endif
    custom3_setting();
    return ERROR_NONE;
}

static kal_uint32 custom4(
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    imgsensor.pclk = imgsensor_info.custom4.pclk;
    imgsensor.line_length = imgsensor_info.custom4.linelength;
    imgsensor.frame_length = imgsensor_info.custom4.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom4_setting();
    return ERROR_NONE;
}
static kal_uint32  get_resolution(
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
}   /*  get_resolution  */

static kal_uint32 get_info(
            enum MSDK_SCENARIO_ID_ENUM scenario_id,
            MSDK_SENSOR_INFO_STRUCT *sensor_info,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    if (scenario_id == 0)
        LOG_INF("scenario_id = %d\n", scenario_id);

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
     /* not use */
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
    // inverse with datasheet
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType =
        imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode =
        imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat =
        imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame =
        imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame =
        imgsensor_info.slim_video_delay_frame;
    sensor_info->FrameTimeDelayFrame =
        imgsensor_info.frame_time_delay_frame;
    sensor_info->Custom1DelayFrame =
        imgsensor_info.custom1_delay_frame;
    sensor_info->Custom2DelayFrame =
        imgsensor_info.custom2_delay_frame;
    sensor_info->Custom3DelayFrame =
        imgsensor_info.custom3_delay_frame;
    sensor_info->Custom4DelayFrame =
        imgsensor_info.custom4_delay_frame;
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
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
/*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode*/
    sensor_info->PDAF_Support = 0;

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
    case MSDK_SCENARIO_ID_CUSTOM1:
        sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM2:
        sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM3:
        sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM4:
        sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom4.mipi_data_lp2hs_settle_dc;
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


static kal_uint32 control(
            enum MSDK_SCENARIO_ID_ENUM scenario_id,
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
    case MSDK_SCENARIO_ID_CUSTOM2:
        custom2(image_window, sensor_config_data);
    break;
    case MSDK_SCENARIO_ID_CUSTOM3:
        custom3(image_window, sensor_config_data);
    break;
    case MSDK_SCENARIO_ID_CUSTOM4:
        custom4(image_window, sensor_config_data);
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
    LOG_INF("%s framerate = %d\n ", __func__, framerate);
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

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
    LOG_INF("%s enable = %d, framerate = %d\n",
        __func__, enable, framerate);

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
        frameHeight = imgsensor_info.normal_video.pclk /
            framerate * 10 / imgsensor_info.normal_video.linelength;
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
        frameHeight = imgsensor_info.cap.pclk /
            framerate * 10 / imgsensor_info.cap.linelength;
        spin_lock(&imgsensor_drv_lock);

        imgsensor.dummy_line = (frameHeight >
            imgsensor_info.cap.framelength) ? (frameHeight -
            imgsensor_info.cap.framelength):0;
        imgsensor.frame_length =
            imgsensor_info.cap.framelength +
            imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
    break;

    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        frameHeight = imgsensor_info.hs_video.pclk /
            framerate * 10 / imgsensor_info.hs_video.linelength;
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
        frameHeight = imgsensor_info.slim_video.pclk /
            framerate * 10 / imgsensor_info.slim_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frameHeight > imgsensor_info.slim_video.framelength) ?
            (frameHeight - imgsensor_info.slim_video.framelength):0;
        imgsensor.frame_length =
            imgsensor_info.slim_video.framelength +
            imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
    break;
    case MSDK_SCENARIO_ID_CUSTOM1:
        frameHeight = imgsensor_info.custom1.pclk /
            framerate * 10 / imgsensor_info.custom1.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frameHeight > imgsensor_info.custom1.framelength) ?
            (frameHeight - imgsensor_info.custom1.framelength):0;
        imgsensor.frame_length =
            imgsensor_info.custom1.framelength +
            imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
    break;
    case MSDK_SCENARIO_ID_CUSTOM2:
        frameHeight = imgsensor_info.custom2.pclk /
            framerate * 10 / imgsensor_info.custom2.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frameHeight > imgsensor_info.custom2.framelength) ?
            (frameHeight - imgsensor_info.custom2.framelength):0;
        imgsensor.frame_length =
            imgsensor_info.custom2.framelength +
            imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
    break;
    case MSDK_SCENARIO_ID_CUSTOM3:
        frameHeight = imgsensor_info.custom3.pclk /
            framerate * 10 / imgsensor_info.custom3.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frameHeight > imgsensor_info.custom3.framelength) ?
            (frameHeight - imgsensor_info.custom3.framelength):0;
        imgsensor.frame_length =
            imgsensor_info.custom3.framelength +
            imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
    break;
    case MSDK_SCENARIO_ID_CUSTOM4:
        frameHeight = imgsensor_info.custom4.pclk /
            framerate * 10 / imgsensor_info.custom4.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frameHeight > imgsensor_info.custom4.framelength) ?
            (frameHeight - imgsensor_info.custom4.framelength):0;
        imgsensor.frame_length =
            imgsensor_info.custom4.framelength +
            imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
    break;
    default:
        //coding with  preview scenario by default
        frameHeight = imgsensor_info.pre.pclk /
            framerate * 10 / imgsensor_info.pre.linelength;
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
        LOG_INF("error scenario_id = %d,use preview scenario\n",
            scenario_id);
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

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("Jesse+ enable: %d\n", enable);
    if (enable) {

        write_cmos_sensor(0x3019, 0xf0);
        write_cmos_sensor(0x4308, 0x01);
        write_cmos_sensor(0x4300, 0x00);
        write_cmos_sensor(0x4302, 0x00);
        write_cmos_sensor(0x4304, 0x00);
        write_cmos_sensor(0x4306, 0x00);

        } else {

        write_cmos_sensor(0x3019, 0xd2);
        write_cmos_sensor(0x4308, 0x00);
        write_cmos_sensor(0x4300, 0x00);
        write_cmos_sensor(0x4302, 0x00);
        write_cmos_sensor(0x4304, 0x00);
        write_cmos_sensor(0x4306, 0x00);
        }

    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
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
    struct SET_PD_BLOCK_INFO_T *PDAFinfo;

    if (!((feature_id == 3040) || (feature_id == 3058)))
        LOG_INF("feature_id = %d\n", feature_id);

    switch (feature_id) {
    case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
        *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = -26540000;
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
    case SENSOR_FEATURE_GET_PERIOD:
        *feature_return_para_16++ = imgsensor.line_length;
        *feature_return_para_16 = imgsensor.frame_length;
        *feature_para_len = 4;
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
        case MSDK_SCENARIO_ID_CUSTOM3:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom3.pclk;
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
    case SENSOR_FEATURE_GET_BINNING_TYPE:
        switch (*(feature_data +1 )) {
        case MSDK_SCENARIO_ID_CUSTOM3:
            *feature_return_para_32 = 2;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *feature_return_para_32 = 1;
            break;
        }
        pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
            *feature_return_para_32);
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
                case MSDK_SCENARIO_ID_CUSTOM1:
                    rate = imgsensor_info.custom1.mipi_pixel_rate;
                    break;
                case MSDK_SCENARIO_ID_CUSTOM2:
                    rate = imgsensor_info.custom2.mipi_pixel_rate;
                    break;
                case MSDK_SCENARIO_ID_CUSTOM3:
                    rate = imgsensor_info.custom3.mipi_pixel_rate;
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    rate = imgsensor_info.slim_video.mipi_pixel_rate;
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    rate = imgsensor_info.pre.mipi_pixel_rate;
                    break;
            }
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
        }
    break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        *feature_return_para_32 = imgsensor.pclk;
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
    case SENSOR_FEATURE_SET_ESHUTTER:
        set_shutter(*feature_data);//jack yan
    break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
        night_mode((BOOL) (*feature_data));
    break;
    case SENSOR_FEATURE_SET_GAIN:
        set_gain((UINT16) *feature_data);//jack yan
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
    case SENSOR_FEATURE_GET_PDAF_DATA:
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
    break;
    case SENSOR_FEATURE_GET_CROP_INFO:
        LOG_INF("GET_CROP_INFO scenarioId:%d\n", *feature_data_32);

        wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)
            (uintptr_t)(*(feature_data+1));
        switch (*feature_data_32) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[1],
                sizeof(struct  SENSOR_WINSIZE_INFO_STRUCT));
        break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[2],
                sizeof(struct  SENSOR_WINSIZE_INFO_STRUCT));
        break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[3],
                sizeof(struct  SENSOR_WINSIZE_INFO_STRUCT));
        break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[4],
                sizeof(struct  SENSOR_WINSIZE_INFO_STRUCT));
        break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[5],
                sizeof(struct  SENSOR_WINSIZE_INFO_STRUCT));
        break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[6],
                sizeof(struct  SENSOR_WINSIZE_INFO_STRUCT));
        break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[7],
                sizeof(struct  SENSOR_WINSIZE_INFO_STRUCT));
        break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[8],
                sizeof(struct  SENSOR_WINSIZE_INFO_STRUCT));
        break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[0],
                sizeof(struct  SENSOR_WINSIZE_INFO_STRUCT));
        break;
        }
    break;
    case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
        LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
            (UINT16)*feature_data, (UINT16)*(feature_data+1),
            (UINT16)*(feature_data+2));
        ihdr_write_shutter_gain((UINT16)*feature_data,
            (UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
    break;

    case SENSOR_FEATURE_GET_PDAF_INFO:
        PDAFinfo = (struct SET_PD_BLOCK_INFO_T *)
            (uintptr_t)(*(feature_data+1));

        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
                sizeof(struct SET_PD_BLOCK_INFO_T));
            break;

        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CUSTOM2:
        case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM4:
        default:
            break;
        }
    break;
    case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1;
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
    case SENSOR_FEATURE_SET_PDAF:
        imgsensor.pdaf_mode = *feature_data_16;
    break;
    case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
        set_shutter_frame_length((UINT16) *feature_data,
            (UINT16) *(feature_data + 1));
        break;
    case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
        *feature_return_para_i32 = get_sensor_temperature();
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
}   /*  feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 ATHENSB_OV32A1Q_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc != NULL)
        *pfFunc =  &sensor_func;
    return ERROR_NONE;
}
