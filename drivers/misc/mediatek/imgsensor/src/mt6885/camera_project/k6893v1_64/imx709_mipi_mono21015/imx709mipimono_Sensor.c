/*
 * Copyright (C) 2016 MediaTek Inc.
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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     IMX709mipi_Sensor.c
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
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/types.h>
#include "imgsensor_eeprom.h"
#include "imx709mipimono_Sensor.h"

/***************Modify Following Strings for Debug**********************/
#define PFX "imx709_camera_mono_sensor"
#define LOG_1 LOG_INF("IMX709,MIPI 4LANE\n")
/****************************Modify end**************************/

#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

static kal_uint32 streaming_control(kal_bool enable);
#define MODULE_ID_OFFSET 0x0000
static kal_uint16 imx709_table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len);
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = IMX709_MONO_SENSOR_ID_21015,
    .checksum_value = 0x8ac2d94a,

    .pre = { /*reg H1 VGA+ 640x480@10fps*/
        .pclk = 150000000,
        .linelength = 936,
        .framelength = 12816,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 640,
        .grabwindow_height = 480,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 150010501,
        .max_framerate = 100, /* 10fps */
    },

    .cap = { /*reg H1 VGA+ 640x480@10fps*/
        .pclk = 150000000,
        .linelength = 936,
        .framelength = 12816,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 640,
        .grabwindow_height = 480,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 150010501,
        .max_framerate = 100, /* 10fps */
    },

    .normal_video = { /*reg H1 VGA+ 640x480@10fps*/
        .pclk = 150000000,
        .linelength = 936,
        .framelength = 12816,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 640,
        .grabwindow_height = 480,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 150010501,
        .max_framerate = 100, /* 10fps */
    },

    .hs_video = { /*reg H1 VGA+ 640x480@10fps*/
        .pclk = 150000000,
        .linelength = 936,
        .framelength = 12816,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 640,
        .grabwindow_height = 480,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 150010501,
        .max_framerate = 100, /* 10fps */
    },

    .slim_video = { /*reg H1 VGA+ 640x480@10fps*/
        .pclk = 150000000,
        .linelength = 936,
        .framelength = 12816,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 640,
        .grabwindow_height = 480,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 150010501,
        .max_framerate = 100, /* 10fps */
    },

    .margin = 24,        /* sensor framelength & shutter margin */
    .min_shutter = 8,    /* min shutter */
    .min_gain = 64, /*1x gain*/
    .max_gain = 2048, /*64x gain*/
    .min_gain_iso = 100,
    .gain_step = 1,
    .gain_type = 0,
    .max_frame_length = 0xffff-5,
    .ae_shut_delay_frame = 0,
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,    /* isp gain delay frame for AE cycle */
    .ihdr_support = 0,    /* 1, support; 0,not support */
    .ihdr_le_firstline = 0,    /* 1,le first ; 0, se first */
    .sensor_mode_num = 5,    /*support sensor mode num*/

    .cap_delay_frame = 2,  /*3 guanjd modify for cts*/
    .pre_delay_frame = 2,  /*3 guanjd modify for cts*/
    .video_delay_frame = 3,
    .hs_video_delay_frame = 3,
    .slim_video_delay_frame = 3,

    .isp_driving_current = ISP_DRIVING_6MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2, /*only concern if it's cphy*/
    .mipi_settle_delay_mode = 0,
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_MONO,
    .mclk = 24, /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
    .mipi_lane_num = SENSOR_MIPI_4_LANE,
    .i2c_addr_table = {0x20, 0xff},
    /* record sensor support all write id addr,
     * only supprt 4 must end with 0xff
     */
    .i2c_speed = 1000, /* i2c read/write speed */
};

static struct imgsensor_struct imgsensor = {
    .mirror = IMAGE_HV_MIRROR,    /* mirror and flip on information */
    .sensor_mode = IMGSENSOR_MODE_INIT,
    /* IMGSENSOR_MODE enum value,record current sensor mode,such as:
     * INIT, Preview, Capture, Video,High Speed Video, Slim Video
     */
    .shutter = 0x3D0,    /* current shutter */
    .gain = 0x100,        /* current gain */
    .dummy_pixel = 0,    /* current dummypixel */
    .dummy_line = 0,    /* current dummyline */
    .current_fps = 100,
    .autoflicker_en = KAL_FALSE,
    .test_pattern = 0,
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
    .ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
    .i2c_write_id = 0x20, /* record current sensor's i2c write id */
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
    {6560, 4928, 0, 544, 6560, 3840, 820, 480,    90,  0, 640,  480,  0,  0, 640, 480}, /* Preview  reg_B1*/
    {6560, 4928, 0, 544, 6560, 3840, 820, 480,    90,  0, 640,  480,  0,  0, 640, 480}, /* capture  reg_B2*/
    {6560, 4928, 0, 544, 6560, 3840, 820, 480,    90,  0, 640,  480,  0,  0, 640, 480}, /* normal video reg_G*/
    {6560, 4928, 0, 544, 6560, 3840, 820, 480,    90,  0, 640,  480,  0,  0, 640, 480}, /*None hs_video  reg_C1_2*/
    {6560, 4928, 0, 544, 6560, 3840, 820, 480,    90,  0, 640,  480,  0,  0, 640, 480}, /* slim video reg_G2*/
};

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

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte = 0;
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};

    iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 2, imgsensor.i2c_write_id);
    return ((get_byte<<8)&0xff00) | ((get_byte>>8)&0x00ff);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte = 0;
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

    iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF),
            (char)(para & 0xFF)};

    iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
    LOG_INF("dummyline = %d, dummypixels = %d, frame_length:%d, line_length:%d\n",
        imgsensor.dummy_line, imgsensor.dummy_pixel, imgsensor.frame_length, imgsensor.line_length);
    /* return;*/ /* for test */
    write_cmos_sensor_8(0x0104, 0x01);

    write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
    write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
    write_cmos_sensor_8(0x0342, imgsensor.line_length >> 8);
    write_cmos_sensor_8(0x0343, imgsensor.line_length & 0xFF);

    write_cmos_sensor_8(0x0104, 0x00);

}    /*    set_dummy  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
    kal_uint8 itemp;

    LOG_INF("image_mirror = %d\n", image_mirror);
    itemp = read_cmos_sensor_8(0x3874);
    itemp &= ~0x03;

    switch (image_mirror) {

    case IMAGE_NORMAL:
    write_cmos_sensor_8(0x3874, itemp);
    break;

    case IMAGE_V_MIRROR:
    write_cmos_sensor_8(0x3874, itemp | 0x02);
    break;

    case IMAGE_H_MIRROR:
    write_cmos_sensor_8(0x3874, itemp | 0x01);
    break;

    case IMAGE_HV_MIRROR:
    write_cmos_sensor_8(0x3874, itemp | 0x03);
    break;
    }
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
    /*kal_int16 dummy_line;*/
    kal_uint32 frame_length = imgsensor.frame_length;

    LOG_INF("framerate = %d, min framelength should enable %d\n", framerate,
        min_framelength_en);

    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
    spin_lock(&imgsensor_drv_lock);
    if (frame_length >= imgsensor.min_frame_length)
        imgsensor.frame_length = frame_length;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;

    imgsensor.dummy_line =
            imgsensor.frame_length - imgsensor.min_frame_length;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line =
            imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if (min_framelength_en)
        imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);
    set_dummy();
}    /*    set_max_framerate  */

static void write_shutter(kal_uint32 shutter)
{
    kal_uint16 realtime_fps = 0;
    int longexposure_times = 0;
    static int long_exposure_status;

    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    if (shutter < imgsensor_info.min_shutter)
        shutter = imgsensor_info.min_shutter;

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10
                / imgsensor.frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
    } else {
        LOG_INF("(else)imgsensor.frame_length = %d\n",
            imgsensor.frame_length);
    }

    while (shutter >= 65535) {
        shutter = shutter / 2;
        longexposure_times += 1;
    }

    if (longexposure_times > 0) {
        LOG_INF("enter long exposure mode, time is %d",
            longexposure_times);
        long_exposure_status = 1;
        imgsensor.frame_length = shutter + 32;
        write_cmos_sensor_8(0x3100, longexposure_times & 0x07);
    } else if (long_exposure_status == 1) {
        long_exposure_status = 0;
        write_cmos_sensor_8(0x0104, 0x01);
        write_cmos_sensor_8(0x3100, 0x00);
        write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
        write_cmos_sensor_8(0x0104, 0x00);
        LOG_INF("exit long exposure mode");
    }
    /* Update Shutter */
    write_cmos_sensor_8(0x0104, 0x01);
    write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
    write_cmos_sensor_8(0x0203, shutter  & 0xFF);
    write_cmos_sensor_8(0x0104, 0x00);
    LOG_INF("shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);
}    /*    write_shutter  */

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

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    write_shutter(shutter);
} /* set_shutter */


/*************************************************************************
 * FUNCTION
 *    set_shutter_frame_length
 *
 * DESCRIPTION
 *    for frame & 3A sync
 *
 *************************************************************************/
static void set_shutter_frame_length(kal_uint16 shutter,
                     kal_uint16 frame_length)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    kal_int32 dummy_line = 0;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    /*0x3500, 0x3501, 0x3502 will increase VBLANK to
     *get exposure larger than frame exposure
     *AE doesn't update sensor gain at capture mode,
     *thus extra exposure lines must be updated here.
     */

    /* OV Recommend Solution */
    /*if shutter bigger than frame_length,
     *should extend frame length first
     */
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
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
                imgsensor.frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
        else {
            /* Extend frame length */
            write_cmos_sensor_8(0x0104, 0x01);
            write_cmos_sensor_8(0x0340,
                    imgsensor.frame_length >> 8);
            write_cmos_sensor_8(0x0341,
                    imgsensor.frame_length & 0xFF);
            write_cmos_sensor_8(0x0104, 0x00);
        }
    } else {
        /* Extend frame length */
        write_cmos_sensor_8(0x0104, 0x01);
        write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
        write_cmos_sensor_8(0x0104, 0x00);
    }

    /* Update Shutter */
    write_cmos_sensor_8(0x0104, 0x01);
    write_cmos_sensor_8(0x0350, 0x00); /* Disable auto extend */
    write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
    write_cmos_sensor_8(0x0203, shutter  & 0xFF);
    write_cmos_sensor_8(0x0104, 0x00);
    LOG_INF(
        "Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
        shutter, imgsensor.frame_length, frame_length,
        dummy_line, read_cmos_sensor(0x0350));

}    /* set_shutter_frame_length */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
     kal_uint16 reg_gain = 0x0;

    reg_gain = 16384 - (16384*64)/gain;

    return (kal_uint16) reg_gain;
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
    kal_uint16 reg_gain, max_gain = imgsensor_info.max_gain;

    if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
        LOG_INF("Error gain setting");

        if (gain < imgsensor_info.min_gain)
            gain = imgsensor_info.min_gain;
        else if (gain > max_gain)
            gain = max_gain;
    }

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("gain = %d, reg_gain = 0x%x\n ", gain, reg_gain);

    write_cmos_sensor_8(0x0104, 0x01);
    write_cmos_sensor_8(0x0204, (reg_gain>>8) & 0xFF);
    write_cmos_sensor_8(0x0205, reg_gain & 0xFF);
    write_cmos_sensor_8(0x0104, 0x00);


    return gain;
} /* set_gain */

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
static kal_uint32 streaming_control(kal_bool enable)
{
    LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n",
        enable);
    if (enable)
        write_cmos_sensor_8(0x0100, 0X01);
    else
        write_cmos_sensor_8(0x0100, 0x00);
    return ERROR_NONE;
}

#define I2C_BUFFER_LEN 255 /* trans# max is 255, each 3 bytes */
static kal_uint16 imx709_table_write_cmos_sensor(kal_uint16 *para,
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
            puSendCmd[tosend++] = (char)(addr >> 8);
            puSendCmd[tosend++] = (char)(addr & 0xFF);
            data = para[IDX + 1];
            puSendCmd[tosend++] = (char)(data & 0xFF);
            IDX += 2;
            addr_last = addr;

        }
        /* Write when remain buffer size is less than 3 bytes
         * or reach end of data
         */
        if ((I2C_BUFFER_LEN - tosend) < 3
            || IDX == len || addr != addr_last) {
            iBurstWriteReg_multi(puSendCmd,
                        tosend,
                        imgsensor.i2c_write_id,
                        3,
                        imgsensor_info.i2c_speed);
            tosend = 0;
        }
    }
    return 0;
}

static kal_uint16 imx709_init_setting[] = {
    //stream off
    0x0100,0x00,
    //External Clock Setting
    0x0136, 0x18,
    0x0137, 0x00,
    //PHY_VIF Setting
    0x3304, 0x00,
    //Register version
    0x33F0, 0x01,
    0x33F1, 0x01,
    //Signaling mode setting
    0x0111, 0x02,
    //Global setting
    0x3140, 0x08,
    0x3A42, 0x01,
    0x4C24, 0x3F,
    0x4CFC, 0x00,
    0x4F0E, 0x01,
    0x4F0F, 0x01,
    0x45B8, 0x17,
    0x45DC, 0x25,
    0x640D, 0x9B,
    0x6419, 0x9B,
    0x6425, 0x9B,
    0x6431, 0x9B,
    0x643D, 0x9B,
    0x6455, 0x9B,
    0x646D, 0x9B,
    0x69D3, 0x27,
    0x69DF, 0x27,
    0x7B52, 0x14,
    0x7B53, 0x14,
    0x7B54, 0x14,
    0x7B55, 0x14,
    0x7B56, 0x14,
    0x7B57, 0x14,
    0x7B58, 0x14,
    0x7B59, 0x14,
    0x7B5A, 0x14,
    0x7B5C, 0x14,
    0x7B5E, 0x14,
    0x7B60, 0x14,
    0x7B61, 0x14,
    0x7B62, 0x14,
    0x7B63, 0x14,
    0x7B64, 0x14,
    0x7B65, 0x14,
    0x7B66, 0x14,
    0x7B67, 0x14,
    0x7B68, 0x14,
    0x7B69, 0x14,
    0x7B6A, 0x14,
    0x7B6B, 0x14,
    0x7B6C, 0x14,
    0x7B6D, 0x14,
    0x7B6E, 0x14,
    0x7B6F, 0x14,
    0x7B70, 0x14,
    0x7B71, 0x14,
    0x7B72, 0x14,
    0x7B73, 0x14,
    0x7B74, 0x14,
    0x7B76, 0x14,
    0x7B78, 0x14,
    0x7B7A, 0x14,
    0x7B7B, 0x14,
    0x7B7C, 0x14,
    0x7B7D, 0x14,
    0x7B7E, 0x14,
    0x7B7F, 0x14,
    0x7B80, 0x14,
    0x7B81, 0x14,
    0x7B82, 0x14,
    0x7B83, 0x14,
    0x7B84, 0x14,
    0x7B85, 0x14,
    0x7B86, 0x14,
    0x7B87, 0x14,
    0x7B88, 0x14,
    0x7B89, 0x14,
    0x7B8A, 0x14,
    0x7B8B, 0x14,
    0x7B8C, 0x14,
    0x7B8D, 0x14,
    0x7B8E, 0x14,
    0x7B90, 0x14,
    0x7B92, 0x14,
    0x7B94, 0x14,
    0x7B95, 0x14,
    0x7B96, 0x14,
    0x7B97, 0x14,
    0x7B98, 0x14,
    0x7B99, 0x14,
    0x7B9A, 0x14,
    0x7B9B, 0x14,
    0x7B9C, 0x14,
    0x7B9D, 0x14,
    0x7B9E, 0x14,
    0x7B9F, 0x14,
    0x7BA0, 0x14,
    0x7BA1, 0x14,
    0x7BA2, 0x14,
    0x7BA3, 0x14,
    0x7BA4, 0x14,
    0x7BA5, 0x14,
    0x7BA6, 0x14,
    0x7BA7, 0x14,
    0x7BA8, 0x14,
    0x7BAA, 0x14,
    0x7BAC, 0x14,
    0x7BAE, 0x14,
    0x7BAF, 0x14,
    0x7BB0, 0x14,
    0x7BB1, 0x14,
    0x7BB2, 0x14,
    0x7BB3, 0x14,
    0x7BB4, 0x14,
    0x7BB5, 0x14,
    0x7BB6, 0x14,
    0x7BB7, 0x14,
    0x7BB8, 0x14,
    0x7BB9, 0x14,
    0x7BBA, 0x14,
    0x7BBB, 0x14,
    0x7BBC, 0x14,
    0x7BBD, 0x14,
    0x7BBE, 0x14,
    0x7BC0, 0x14,
    0x7BC1, 0x14,
    0x7BC2, 0x14,
    0x7BC3, 0x14,
    0x7BC4, 0x14,
    0x7BC5, 0x14,
    0x7BC7, 0x14,
    0x7BC8, 0x14,
    0x7BC9, 0x14,
    0x7BCA, 0x14,
    0x7BCB, 0x14,
    0xEB00, 0x18,
    0xEB01, 0x0B,
    0xEB02, 0x04,
};

/*reg_H1 640x480@10fps*/
static kal_uint16 imx709_preview_setting[] = {
    //MIPI output setting
    0x0112, 0x08,
    0x0113, 0x08,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x3762, 0x05,
    0x3763, 0x00,
    0x3766, 0x00,
    0x3770, 0x00,
    0x3771, 0x05,
    0x3812, 0x05,
    0x3813, 0x00,
    0x3816, 0x00,
    0x3820, 0x00,
    0x3821, 0x05,
    0x3882, 0x03,
    0x3883, 0xA8,
    0x3886, 0x00,
    0x389E, 0x00,
    0x389F, 0xBB,
    0x38C6, 0x05,
    0x38C7, 0x00,
    0x38CA, 0x00,
    0x38E0, 0x00,
    0x38E1, 0x05,
    //Frame Length Lines Setting
    0x3760, 0x0A,
    0x3761, 0xF8,
    0x3810, 0x0A,
    0x3811, 0xF8,
    0x3880, 0x32,
    0x3881, 0x10,
    0x38C4, 0x0A,
    0x38C5, 0xF8,
    //ROI Setting
    0x3888, 0x00,
    0x3889, 0x00,
    0x388A, 0x02,
    0x388B, 0x20,
    0x388C, 0x19,
    0x388D, 0x9F,
    0x388E, 0x11,
    0x388F, 0x1F,
    0x38CC, 0x00,
    0x38CD, 0x00,
    0x38CE, 0x00,
    0x38CF, 0x00,
    0x38D0, 0x19,
    0x38D1, 0x9F,
    0x38D2, 0x13,
    0x38D3, 0x3F,
    //Mode Setting
    0x0900, 0x01,
    0x0902, 0x00,
    0x3004, 0x01,
    0x31D4, 0x08,
    0x31D5, 0x08,
    //Digital Crop & Scaling
    0x3894, 0x00,
    0x3895, 0x5A,
    0x3896, 0x00,
    0x3897, 0x00,
    0x3898, 0x02,
    0x3899, 0x80,
    0x389A, 0x01,
    0x389B, 0xE0,
    0x38D8, 0x00,
    0x38D9, 0x00,
    0x38DA, 0x00,
    0x38DB, 0x00,
    0x38DC, 0x01,
    0x38DD, 0x9A,
    0x38DE, 0x01,
    0x38DF, 0x34,
    //Output Size Setting
    0x3890, 0x02,
    0x3891, 0x80,
    0x3892, 0x01,
    0x3893, 0xE0,
    0x38D4, 0x01,
    0x38D5, 0x98,
    0x38D6, 0x01,
    0x38D7, 0x34,
    //Clock Setting
    0x0303, 0x04,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x64,
    0x387C, 0x08,
    0x387D, 0x04,
    0x387E, 0x01,
    0x387F, 0x90,
    //Other Setting
    0x31CC, 0x00,
    0x3213, 0x01,
    0xBD24, 0x01,
    0xBD25, 0x30,
    //Integration Setting
    0x3750, 0x00,
    0x3751, 0x40,
    0x3756, 0x00,
    0x3757, 0x01,
    0x3788, 0x00,
    0x3789, 0xE0,
    0x378E, 0x00,
    0x378F, 0x04,
    0x3794, 0x01,
    0x3795, 0x80,
    0x379A, 0x00,
    0x379B, 0x06,
    0x3800, 0x00,
    0x3801, 0x0E,
    0x3806, 0x00,
    0x3807, 0x0E,
    0x3838, 0x00,
    0x3839, 0xE0,
    0x383E, 0x00,
    0x383F, 0xE0,
    0x3844, 0x06,
    0x3845, 0x62,
    0x384A, 0x06,
    0x384B, 0x62,
    0x3878, 0x00,
    0x3879, 0x40,
    0x38B6, 0x00,
    0x38B7, 0xE0,
    0x38BA, 0x01,
    0x38BB, 0x80,
    0x38C0, 0x00,
    0x38C1, 0x40,
    0x38F8, 0x00,
    0x38F9, 0xE0,
    0x38FC, 0x01,
    0x38FD, 0x80,
    //Gain Setting
    0x020E, 0x01,
    0x020F, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    0x3166, 0x01,
    0x3167, 0x00,
    0x3752, 0x00,
    0x3753, 0x00,
    0x3758, 0x01,
    0x3759, 0x00,
    0x378A, 0x32,
    0x378B, 0x00,
    0x3790, 0x32,
    0x3791, 0x00,
    0x3796, 0x48,
    0x3797, 0x00,
    0x379C, 0x48,
    0x379D, 0x00,
    0x3802, 0x60,
    0x3803, 0x00,
    0x3808, 0x32,
    0x3809, 0x00,
    0x383A, 0x60,
    0x383B, 0x00,
    0x3840, 0x32,
    0x3841, 0x00,
    0x3846, 0x60,
    0x3847, 0x00,
    0x384C, 0x32,
    0x384D, 0x00,
    0x387A, 0x00,
    0x387B, 0x00,
    0x38B8, 0x32,
    0x38B9, 0x00,
    0x38BC, 0x48,
    0x38BD, 0x00,
    0x38C2, 0x00,
    0x38C3, 0x00,
    0x38FA, 0x32,
    0x38FB, 0x00,
    0x38FE, 0x48,
    0x38FF, 0x00,
};

/*reg_H1 640x480@10fps*/
static kal_uint16 imx709_capture_setting[] = {
    //MIPI output setting
    0x0112, 0x08,
    0x0113, 0x08,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x3762, 0x05,
    0x3763, 0x00,
    0x3766, 0x00,
    0x3770, 0x00,
    0x3771, 0x05,
    0x3812, 0x05,
    0x3813, 0x00,
    0x3816, 0x00,
    0x3820, 0x00,
    0x3821, 0x05,
    0x3882, 0x03,
    0x3883, 0xA8,
    0x3886, 0x00,
    0x389E, 0x00,
    0x389F, 0xBB,
    0x38C6, 0x05,
    0x38C7, 0x00,
    0x38CA, 0x00,
    0x38E0, 0x00,
    0x38E1, 0x05,
    //Frame Length Lines Setting
    0x3760, 0x0A,
    0x3761, 0xF8,
    0x3810, 0x0A,
    0x3811, 0xF8,
    0x3880, 0x32,
    0x3881, 0x10,
    0x38C4, 0x0A,
    0x38C5, 0xF8,
    //ROI Setting
    0x3888, 0x00,
    0x3889, 0x00,
    0x388A, 0x02,
    0x388B, 0x20,
    0x388C, 0x19,
    0x388D, 0x9F,
    0x388E, 0x11,
    0x388F, 0x1F,
    0x38CC, 0x00,
    0x38CD, 0x00,
    0x38CE, 0x00,
    0x38CF, 0x00,
    0x38D0, 0x19,
    0x38D1, 0x9F,
    0x38D2, 0x13,
    0x38D3, 0x3F,
    //Mode Setting
    0x0900, 0x01,
    0x0902, 0x00,
    0x3004, 0x01,
    0x31D4, 0x08,
    0x31D5, 0x08,
    //Digital Crop & Scaling
    0x3894, 0x00,
    0x3895, 0x5A,
    0x3896, 0x00,
    0x3897, 0x00,
    0x3898, 0x02,
    0x3899, 0x80,
    0x389A, 0x01,
    0x389B, 0xE0,
    0x38D8, 0x00,
    0x38D9, 0x00,
    0x38DA, 0x00,
    0x38DB, 0x00,
    0x38DC, 0x01,
    0x38DD, 0x9A,
    0x38DE, 0x01,
    0x38DF, 0x34,
    //Output Size Setting
    0x3890, 0x02,
    0x3891, 0x80,
    0x3892, 0x01,
    0x3893, 0xE0,
    0x38D4, 0x01,
    0x38D5, 0x98,
    0x38D6, 0x01,
    0x38D7, 0x34,
    //Clock Setting
    0x0303, 0x04,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x64,
    0x387C, 0x08,
    0x387D, 0x04,
    0x387E, 0x01,
    0x387F, 0x90,
    //Other Setting
    0x31CC, 0x00,
    0x3213, 0x01,
    0xBD24, 0x01,
    0xBD25, 0x30,
    //Integration Setting
    0x3750, 0x00,
    0x3751, 0x40,
    0x3756, 0x00,
    0x3757, 0x01,
    0x3788, 0x00,
    0x3789, 0xE0,
    0x378E, 0x00,
    0x378F, 0x04,
    0x3794, 0x01,
    0x3795, 0x80,
    0x379A, 0x00,
    0x379B, 0x06,
    0x3800, 0x00,
    0x3801, 0x0E,
    0x3806, 0x00,
    0x3807, 0x0E,
    0x3838, 0x00,
    0x3839, 0xE0,
    0x383E, 0x00,
    0x383F, 0xE0,
    0x3844, 0x06,
    0x3845, 0x62,
    0x384A, 0x06,
    0x384B, 0x62,
    0x3878, 0x00,
    0x3879, 0x40,
    0x38B6, 0x00,
    0x38B7, 0xE0,
    0x38BA, 0x01,
    0x38BB, 0x80,
    0x38C0, 0x00,
    0x38C1, 0x40,
    0x38F8, 0x00,
    0x38F9, 0xE0,
    0x38FC, 0x01,
    0x38FD, 0x80,
    //Gain Setting
    0x020E, 0x01,
    0x020F, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    0x3166, 0x01,
    0x3167, 0x00,
    0x3752, 0x00,
    0x3753, 0x00,
    0x3758, 0x01,
    0x3759, 0x00,
    0x378A, 0x32,
    0x378B, 0x00,
    0x3790, 0x32,
    0x3791, 0x00,
    0x3796, 0x48,
    0x3797, 0x00,
    0x379C, 0x48,
    0x379D, 0x00,
    0x3802, 0x60,
    0x3803, 0x00,
    0x3808, 0x32,
    0x3809, 0x00,
    0x383A, 0x60,
    0x383B, 0x00,
    0x3840, 0x32,
    0x3841, 0x00,
    0x3846, 0x60,
    0x3847, 0x00,
    0x384C, 0x32,
    0x384D, 0x00,
    0x387A, 0x00,
    0x387B, 0x00,
    0x38B8, 0x32,
    0x38B9, 0x00,
    0x38BC, 0x48,
    0x38BD, 0x00,
    0x38C2, 0x00,
    0x38C3, 0x00,
    0x38FA, 0x32,
    0x38FB, 0x00,
    0x38FE, 0x48,
    0x38FF, 0x00,
};

/*reg_H1 640x480@10fps*/
static kal_uint16 imx709_normal_video_setting[] = {
    //MIPI output setting
    0x0112, 0x08,
    0x0113, 0x08,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x3762, 0x05,
    0x3763, 0x00,
    0x3766, 0x00,
    0x3770, 0x00,
    0x3771, 0x05,
    0x3812, 0x05,
    0x3813, 0x00,
    0x3816, 0x00,
    0x3820, 0x00,
    0x3821, 0x05,
    0x3882, 0x03,
    0x3883, 0xA8,
    0x3886, 0x00,
    0x389E, 0x00,
    0x389F, 0xBB,
    0x38C6, 0x05,
    0x38C7, 0x00,
    0x38CA, 0x00,
    0x38E0, 0x00,
    0x38E1, 0x05,
    //Frame Length Lines Setting
    0x3760, 0x0A,
    0x3761, 0xF8,
    0x3810, 0x0A,
    0x3811, 0xF8,
    0x3880, 0x32,
    0x3881, 0x10,
    0x38C4, 0x0A,
    0x38C5, 0xF8,
    //ROI Setting
    0x3888, 0x00,
    0x3889, 0x00,
    0x388A, 0x02,
    0x388B, 0x20,
    0x388C, 0x19,
    0x388D, 0x9F,
    0x388E, 0x11,
    0x388F, 0x1F,
    0x38CC, 0x00,
    0x38CD, 0x00,
    0x38CE, 0x00,
    0x38CF, 0x00,
    0x38D0, 0x19,
    0x38D1, 0x9F,
    0x38D2, 0x13,
    0x38D3, 0x3F,
    //Mode Setting
    0x0900, 0x01,
    0x0902, 0x00,
    0x3004, 0x01,
    0x31D4, 0x08,
    0x31D5, 0x08,
    //Digital Crop & Scaling
    0x3894, 0x00,
    0x3895, 0x5A,
    0x3896, 0x00,
    0x3897, 0x00,
    0x3898, 0x02,
    0x3899, 0x80,
    0x389A, 0x01,
    0x389B, 0xE0,
    0x38D8, 0x00,
    0x38D9, 0x00,
    0x38DA, 0x00,
    0x38DB, 0x00,
    0x38DC, 0x01,
    0x38DD, 0x9A,
    0x38DE, 0x01,
    0x38DF, 0x34,
    //Output Size Setting
    0x3890, 0x02,
    0x3891, 0x80,
    0x3892, 0x01,
    0x3893, 0xE0,
    0x38D4, 0x01,
    0x38D5, 0x98,
    0x38D6, 0x01,
    0x38D7, 0x34,
    //Clock Setting
    0x0303, 0x04,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x64,
    0x387C, 0x08,
    0x387D, 0x04,
    0x387E, 0x01,
    0x387F, 0x90,
    //Other Setting
    0x31CC, 0x00,
    0x3213, 0x01,
    0xBD24, 0x01,
    0xBD25, 0x30,
    //Integration Setting
    0x3750, 0x00,
    0x3751, 0x40,
    0x3756, 0x00,
    0x3757, 0x01,
    0x3788, 0x00,
    0x3789, 0xE0,
    0x378E, 0x00,
    0x378F, 0x04,
    0x3794, 0x01,
    0x3795, 0x80,
    0x379A, 0x00,
    0x379B, 0x06,
    0x3800, 0x00,
    0x3801, 0x0E,
    0x3806, 0x00,
    0x3807, 0x0E,
    0x3838, 0x00,
    0x3839, 0xE0,
    0x383E, 0x00,
    0x383F, 0xE0,
    0x3844, 0x06,
    0x3845, 0x62,
    0x384A, 0x06,
    0x384B, 0x62,
    0x3878, 0x00,
    0x3879, 0x40,
    0x38B6, 0x00,
    0x38B7, 0xE0,
    0x38BA, 0x01,
    0x38BB, 0x80,
    0x38C0, 0x00,
    0x38C1, 0x40,
    0x38F8, 0x00,
    0x38F9, 0xE0,
    0x38FC, 0x01,
    0x38FD, 0x80,
    //Gain Setting
    0x020E, 0x01,
    0x020F, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    0x3166, 0x01,
    0x3167, 0x00,
    0x3752, 0x00,
    0x3753, 0x00,
    0x3758, 0x01,
    0x3759, 0x00,
    0x378A, 0x32,
    0x378B, 0x00,
    0x3790, 0x32,
    0x3791, 0x00,
    0x3796, 0x48,
    0x3797, 0x00,
    0x379C, 0x48,
    0x379D, 0x00,
    0x3802, 0x60,
    0x3803, 0x00,
    0x3808, 0x32,
    0x3809, 0x00,
    0x383A, 0x60,
    0x383B, 0x00,
    0x3840, 0x32,
    0x3841, 0x00,
    0x3846, 0x60,
    0x3847, 0x00,
    0x384C, 0x32,
    0x384D, 0x00,
    0x387A, 0x00,
    0x387B, 0x00,
    0x38B8, 0x32,
    0x38B9, 0x00,
    0x38BC, 0x48,
    0x38BD, 0x00,
    0x38C2, 0x00,
    0x38C3, 0x00,
    0x38FA, 0x32,
    0x38FB, 0x00,
    0x38FE, 0x48,
    0x38FF, 0x00,
};

/*reg_H1 640x480@10fps*/
static kal_uint16 imx709_hs_video_setting[] = {
   //MIPI output setting
    0x0112, 0x08,
    0x0113, 0x08,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x3762, 0x05,
    0x3763, 0x00,
    0x3766, 0x00,
    0x3770, 0x00,
    0x3771, 0x05,
    0x3812, 0x05,
    0x3813, 0x00,
    0x3816, 0x00,
    0x3820, 0x00,
    0x3821, 0x05,
    0x3882, 0x03,
    0x3883, 0xA8,
    0x3886, 0x00,
    0x389E, 0x00,
    0x389F, 0xBB,
    0x38C6, 0x05,
    0x38C7, 0x00,
    0x38CA, 0x00,
    0x38E0, 0x00,
    0x38E1, 0x05,
    //Frame Length Lines Setting
    0x3760, 0x0A,
    0x3761, 0xF8,
    0x3810, 0x0A,
    0x3811, 0xF8,
    0x3880, 0x32,
    0x3881, 0x10,
    0x38C4, 0x0A,
    0x38C5, 0xF8,
    //ROI Setting
    0x3888, 0x00,
    0x3889, 0x00,
    0x388A, 0x02,
    0x388B, 0x20,
    0x388C, 0x19,
    0x388D, 0x9F,
    0x388E, 0x11,
    0x388F, 0x1F,
    0x38CC, 0x00,
    0x38CD, 0x00,
    0x38CE, 0x00,
    0x38CF, 0x00,
    0x38D0, 0x19,
    0x38D1, 0x9F,
    0x38D2, 0x13,
    0x38D3, 0x3F,
    //Mode Setting
    0x0900, 0x01,
    0x0902, 0x00,
    0x3004, 0x01,
    0x31D4, 0x08,
    0x31D5, 0x08,
    //Digital Crop & Scaling
    0x3894, 0x00,
    0x3895, 0x5A,
    0x3896, 0x00,
    0x3897, 0x00,
    0x3898, 0x02,
    0x3899, 0x80,
    0x389A, 0x01,
    0x389B, 0xE0,
    0x38D8, 0x00,
    0x38D9, 0x00,
    0x38DA, 0x00,
    0x38DB, 0x00,
    0x38DC, 0x01,
    0x38DD, 0x9A,
    0x38DE, 0x01,
    0x38DF, 0x34,
    //Output Size Setting
    0x3890, 0x02,
    0x3891, 0x80,
    0x3892, 0x01,
    0x3893, 0xE0,
    0x38D4, 0x01,
    0x38D5, 0x98,
    0x38D6, 0x01,
    0x38D7, 0x34,
    //Clock Setting
    0x0303, 0x04,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x64,
    0x387C, 0x08,
    0x387D, 0x04,
    0x387E, 0x01,
    0x387F, 0x90,
    //Other Setting
    0x31CC, 0x00,
    0x3213, 0x01,
    0xBD24, 0x01,
    0xBD25, 0x30,
    //Integration Setting
    0x3750, 0x00,
    0x3751, 0x40,
    0x3756, 0x00,
    0x3757, 0x01,
    0x3788, 0x00,
    0x3789, 0xE0,
    0x378E, 0x00,
    0x378F, 0x04,
    0x3794, 0x01,
    0x3795, 0x80,
    0x379A, 0x00,
    0x379B, 0x06,
    0x3800, 0x00,
    0x3801, 0x0E,
    0x3806, 0x00,
    0x3807, 0x0E,
    0x3838, 0x00,
    0x3839, 0xE0,
    0x383E, 0x00,
    0x383F, 0xE0,
    0x3844, 0x06,
    0x3845, 0x62,
    0x384A, 0x06,
    0x384B, 0x62,
    0x3878, 0x00,
    0x3879, 0x40,
    0x38B6, 0x00,
    0x38B7, 0xE0,
    0x38BA, 0x01,
    0x38BB, 0x80,
    0x38C0, 0x00,
    0x38C1, 0x40,
    0x38F8, 0x00,
    0x38F9, 0xE0,
    0x38FC, 0x01,
    0x38FD, 0x80,
    //Gain Setting
    0x020E, 0x01,
    0x020F, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    0x3166, 0x01,
    0x3167, 0x00,
    0x3752, 0x00,
    0x3753, 0x00,
    0x3758, 0x01,
    0x3759, 0x00,
    0x378A, 0x32,
    0x378B, 0x00,
    0x3790, 0x32,
    0x3791, 0x00,
    0x3796, 0x48,
    0x3797, 0x00,
    0x379C, 0x48,
    0x379D, 0x00,
    0x3802, 0x60,
    0x3803, 0x00,
    0x3808, 0x32,
    0x3809, 0x00,
    0x383A, 0x60,
    0x383B, 0x00,
    0x3840, 0x32,
    0x3841, 0x00,
    0x3846, 0x60,
    0x3847, 0x00,
    0x384C, 0x32,
    0x384D, 0x00,
    0x387A, 0x00,
    0x387B, 0x00,
    0x38B8, 0x32,
    0x38B9, 0x00,
    0x38BC, 0x48,
    0x38BD, 0x00,
    0x38C2, 0x00,
    0x38C3, 0x00,
    0x38FA, 0x32,
    0x38FB, 0x00,
    0x38FE, 0x48,
    0x38FF, 0x00,
};

/*reg_H1 640x480@10fps*/
static kal_uint16 imx709_slim_video_setting[] = {
    //MIPI output setting
    0x0112, 0x08,
    0x0113, 0x08,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x3762, 0x05,
    0x3763, 0x00,
    0x3766, 0x00,
    0x3770, 0x00,
    0x3771, 0x05,
    0x3812, 0x05,
    0x3813, 0x00,
    0x3816, 0x00,
    0x3820, 0x00,
    0x3821, 0x05,
    0x3882, 0x03,
    0x3883, 0xA8,
    0x3886, 0x00,
    0x389E, 0x00,
    0x389F, 0xBB,
    0x38C6, 0x05,
    0x38C7, 0x00,
    0x38CA, 0x00,
    0x38E0, 0x00,
    0x38E1, 0x05,
    //Frame Length Lines Setting
    0x3760, 0x0A,
    0x3761, 0xF8,
    0x3810, 0x0A,
    0x3811, 0xF8,
    0x3880, 0x32,
    0x3881, 0x10,
    0x38C4, 0x0A,
    0x38C5, 0xF8,
    //ROI Setting
    0x3888, 0x00,
    0x3889, 0x00,
    0x388A, 0x02,
    0x388B, 0x20,
    0x388C, 0x19,
    0x388D, 0x9F,
    0x388E, 0x11,
    0x388F, 0x1F,
    0x38CC, 0x00,
    0x38CD, 0x00,
    0x38CE, 0x00,
    0x38CF, 0x00,
    0x38D0, 0x19,
    0x38D1, 0x9F,
    0x38D2, 0x13,
    0x38D3, 0x3F,
    //Mode Setting
    0x0900, 0x01,
    0x0902, 0x00,
    0x3004, 0x01,
    0x31D4, 0x08,
    0x31D5, 0x08,
    //Digital Crop & Scaling
    0x3894, 0x00,
    0x3895, 0x5A,
    0x3896, 0x00,
    0x3897, 0x00,
    0x3898, 0x02,
    0x3899, 0x80,
    0x389A, 0x01,
    0x389B, 0xE0,
    0x38D8, 0x00,
    0x38D9, 0x00,
    0x38DA, 0x00,
    0x38DB, 0x00,
    0x38DC, 0x01,
    0x38DD, 0x9A,
    0x38DE, 0x01,
    0x38DF, 0x34,
    //Output Size Setting
    0x3890, 0x02,
    0x3891, 0x80,
    0x3892, 0x01,
    0x3893, 0xE0,
    0x38D4, 0x01,
    0x38D5, 0x98,
    0x38D6, 0x01,
    0x38D7, 0x34,
    //Clock Setting
    0x0303, 0x04,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x64,
    0x387C, 0x08,
    0x387D, 0x04,
    0x387E, 0x01,
    0x387F, 0x90,
    //Other Setting
    0x31CC, 0x00,
    0x3213, 0x01,
    0xBD24, 0x01,
    0xBD25, 0x30,
    //Integration Setting
    0x3750, 0x00,
    0x3751, 0x40,
    0x3756, 0x00,
    0x3757, 0x01,
    0x3788, 0x00,
    0x3789, 0xE0,
    0x378E, 0x00,
    0x378F, 0x04,
    0x3794, 0x01,
    0x3795, 0x80,
    0x379A, 0x00,
    0x379B, 0x06,
    0x3800, 0x00,
    0x3801, 0x0E,
    0x3806, 0x00,
    0x3807, 0x0E,
    0x3838, 0x00,
    0x3839, 0xE0,
    0x383E, 0x00,
    0x383F, 0xE0,
    0x3844, 0x06,
    0x3845, 0x62,
    0x384A, 0x06,
    0x384B, 0x62,
    0x3878, 0x00,
    0x3879, 0x40,
    0x38B6, 0x00,
    0x38B7, 0xE0,
    0x38BA, 0x01,
    0x38BB, 0x80,
    0x38C0, 0x00,
    0x38C1, 0x40,
    0x38F8, 0x00,
    0x38F9, 0xE0,
    0x38FC, 0x01,
    0x38FD, 0x80,
    //Gain Setting
    0x020E, 0x01,
    0x020F, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    0x3166, 0x01,
    0x3167, 0x00,
    0x3752, 0x00,
    0x3753, 0x00,
    0x3758, 0x01,
    0x3759, 0x00,
    0x378A, 0x32,
    0x378B, 0x00,
    0x3790, 0x32,
    0x3791, 0x00,
    0x3796, 0x48,
    0x3797, 0x00,
    0x379C, 0x48,
    0x379D, 0x00,
    0x3802, 0x60,
    0x3803, 0x00,
    0x3808, 0x32,
    0x3809, 0x00,
    0x383A, 0x60,
    0x383B, 0x00,
    0x3840, 0x32,
    0x3841, 0x00,
    0x3846, 0x60,
    0x3847, 0x00,
    0x384C, 0x32,
    0x384D, 0x00,
    0x387A, 0x00,
    0x387B, 0x00,
    0x38B8, 0x32,
    0x38B9, 0x00,
    0x38BC, 0x48,
    0x38BD, 0x00,
    0x38C2, 0x00,
    0x38C3, 0x00,
    0x38FA, 0x32,
    0x38FB, 0x00,
    0x38FE, 0x48,
    0x38FF, 0x00,
};


static void sensor_init(void)
{
    LOG_INF("sensor_init start\n");
    imx709_table_write_cmos_sensor(imx709_init_setting,
        sizeof(imx709_init_setting)/sizeof(kal_uint16));
    set_mirror_flip(imgsensor.mirror);
    LOG_INF("sensor_init End\n");
}    /*sensor_init  */

static void preview_setting(void)
{
    LOG_INF("%s start\n", __func__);
    imx709_table_write_cmos_sensor(imx709_preview_setting,
        sizeof(imx709_preview_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
} /* preview_setting */


/*full size 30fps*/
static void capture_setting(kal_uint16 currefps)
{
    LOG_INF("%s start\n", __func__);
    imx709_table_write_cmos_sensor(imx709_capture_setting,
        sizeof(imx709_capture_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void normal_video_setting(kal_uint16 currefps)
{
    LOG_INF("%s start\n", __func__);
    imx709_table_write_cmos_sensor(imx709_normal_video_setting,
        sizeof(imx709_normal_video_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void hs_video_setting(void)
{
    LOG_INF("%s start\n", __func__);
    imx709_table_write_cmos_sensor(imx709_hs_video_setting,
        sizeof(imx709_hs_video_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void slim_video_setting(void)
{
    LOG_INF("%s start\n", __func__);
    imx709_table_write_cmos_sensor(imx709_slim_video_setting,
        sizeof(imx709_slim_video_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
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
    /*sensor have two i2c address 0x34 & 0x20,
     *we should detect the module used i2c address
     */
    LOG_INF("imx709 Enter %s.", __func__);
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = ((read_cmos_sensor_8(0x0016) << 8)
                    | read_cmos_sensor_8(0x0017));
            LOG_INF(
                "read_0x0000=0x%x, 0x0001=0x%x,0x0000_0001=0x%x\n",
                read_cmos_sensor_8(0x0016),
                read_cmos_sensor_8(0x0017),
                read_cmos_sensor(0x0000));
            if (*sensor_id == imgsensor_info.sensor_id) {
                *sensor_id = imgsensor_info.sensor_id;
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
                    imgsensor.i2c_write_id, *sensor_id);
                imgsensor_info.module_id = read_module_id();
                LOG_INF("imx709_module_id=%d\n",imgsensor_info.module_id);
                return ERROR_NONE;
            }

            LOG_INF("Read sensor id fail, id: 0x%x\n",
                imgsensor.i2c_write_id);
            retry--;
        } while (retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        /*if Sensor ID is not correct,
         *Must set *sensor_id to 0xFFFFFFFF
         */
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
    kal_uint16 sensor_id = 0;

    LOG_INF("IMX709 open start\n");
    /*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
     *we should detect the module used i2c address
     */
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = ((read_cmos_sensor_8(0x0016) << 8)
                    | read_cmos_sensor_8(0x0017));
            if (sensor_id == imgsensor_info.sensor_id) {
                sensor_id = imgsensor_info.sensor_id;
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
                    imgsensor.i2c_write_id, sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, id: 0x%x\n",
                imgsensor.i2c_write_id);
            retry--;
        } while (retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id) {
        return ERROR_SENSORID_READ_FAIL;
    }
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
    LOG_INF("IMX709 open End\n");
    return ERROR_NONE;
} /* open */

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
} /* close */

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
    LOG_INF("imx709:Enter %s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    preview_setting();
    LOG_INF("imx709:Leave %s., w*h:%d x %d.\n", __func__, imgsensor.line_length, imgsensor.frame_length);
    mdelay(8);

    return ERROR_NONE;
} /* preview */

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

    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate){
        LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
                imgsensor.current_fps,
                imgsensor_info.cap.max_framerate / 10);
    }
    imgsensor.pclk = imgsensor_info.cap.pclk;
    imgsensor.line_length = imgsensor_info.cap.linelength;
    imgsensor.frame_length = imgsensor_info.cap.framelength;
    imgsensor.min_frame_length = imgsensor_info.cap.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;


    spin_unlock(&imgsensor_drv_lock);
    capture_setting(imgsensor.current_fps);
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* capture() */
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
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

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
    //set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s. 720P@240FPS\n", __func__);

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
    //set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* slim_video */

static kal_uint32
get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
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
} /* get_resolution */

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
    
    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
    sensor_info->AESensorGainDelayFrame =
        imgsensor_info.ae_sensor_gain_delay_frame;
    sensor_info->AEISPGainDelayFrame =
        imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
    sensor_info->PDAF_Support = 2;
    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0; /* 0 is default 1x */
    sensor_info->SensorHightSampling = 0; /* 0 is default 1x */
    sensor_info->SensorPacketECCOrder = 1;

    sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;

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
    default:
        LOG_INF("Error ScenarioId setting");
        preview(image_window, sensor_config_data);
        return ERROR_INVALID_SCENARIO_ID;
    }

    return ERROR_NONE;
}    /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
    LOG_INF("framerate = %d\n ", framerate);
    /* SetVideoMode Function should fix framerate */
    if (framerate == 0)
        /* Dynamic frame rate */
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
    LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
    spin_lock(&imgsensor_drv_lock);
    if (enable) /*enable auto flicker*/
        imgsensor.autoflicker_en = KAL_TRUE;
    else /*Cancel Auto flick*/
        imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
        enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        frame_length = imgsensor_info.pre.pclk / framerate * 10
                / imgsensor_info.pre.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.pre.framelength)
        ? (frame_length - imgsensor_info.pre.framelength) : 0;
        imgsensor.frame_length =
            imgsensor_info.pre.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        if (framerate == 0)
            return ERROR_NONE;
        frame_length = imgsensor_info.normal_video.pclk /
                framerate * 10 /
                imgsensor_info.normal_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.normal_video.framelength)
        ? (frame_length - imgsensor_info.normal_video.framelength)
        : 0;
        imgsensor.frame_length =
            imgsensor_info.normal_video.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF(
                "Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
                , framerate, imgsensor_info.cap.max_framerate/10);
            frame_length = imgsensor_info.cap.pclk / framerate * 10
                    / imgsensor_info.cap.linelength;
            spin_lock(&imgsensor_drv_lock);
                imgsensor.dummy_line =
                (frame_length > imgsensor_info.cap.framelength)
                  ? (frame_length - imgsensor_info.cap.framelength) : 0;
                imgsensor.frame_length =
                    imgsensor_info.cap.framelength
                    + imgsensor.dummy_line;
                imgsensor.min_frame_length = imgsensor.frame_length;
                spin_unlock(&imgsensor_drv_lock);
            if (imgsensor.frame_length > imgsensor.shutter)
                set_dummy();
        break;
    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        frame_length = imgsensor_info.hs_video.pclk / framerate * 10
                / imgsensor_info.hs_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.hs_video.framelength)
              ? (frame_length - imgsensor_info.hs_video.framelength)
              : 0;
        imgsensor.frame_length =
            imgsensor_info.hs_video.framelength
                + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        break;
    case MSDK_SCENARIO_ID_SLIM_VIDEO:
        frame_length = imgsensor_info.slim_video.pclk / framerate * 10
            / imgsensor_info.slim_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.slim_video.framelength)
            ? (frame_length - imgsensor_info.slim_video.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.slim_video.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        break;
    default:  /*coding with  preview scenario by default*/
        frame_length = imgsensor_info.pre.pclk / framerate * 10
            / imgsensor_info.pre.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.pre.framelength)
            ? (frame_length - imgsensor_info.pre.framelength) : 0;
        imgsensor.frame_length =
            imgsensor_info.pre.framelength + imgsensor.dummy_line;
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
        enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
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

static kal_uint32 set_test_pattern_mode(kal_uint8 modes, struct SET_SENSOR_PATTERN_SOLID_COLOR *pTestpatterndata)
{
    kal_uint16 Color_R, Color_Gr, Color_Gb, Color_B;
    pr_debug("set_test_pattern enum: %d\n", modes);

    if (modes) {
        write_cmos_sensor_8(0x0600, modes>>4);
        write_cmos_sensor_8(0x0601, modes);
        if (modes == 1 && (pTestpatterndata != NULL)) { //Solid Color
            Color_R = (pTestpatterndata->COLOR_R >> 16) & 0xFFFF;
            Color_Gr = (pTestpatterndata->COLOR_Gr >> 16) & 0xFFFF;
            Color_B = (pTestpatterndata->COLOR_B >> 16) & 0xFFFF;
            Color_Gb = (pTestpatterndata->COLOR_Gb >> 16) & 0xFFFF;
            write_cmos_sensor_8(0x0602, Color_R >> 8);
            write_cmos_sensor_8(0x0603, Color_R & 0xFF);
            write_cmos_sensor_8(0x0604, Color_Gr >> 8);
            write_cmos_sensor_8(0x0605, Color_Gr & 0xFF);
            write_cmos_sensor_8(0x0606, Color_B >> 8);
            write_cmos_sensor_8(0x0607, Color_B & 0xFF);
            write_cmos_sensor_8(0x0608, Color_Gb >> 8);
            write_cmos_sensor_8(0x0609, Color_Gb & 0xFF);
        }
    } else {
        write_cmos_sensor(0x0600, 0x0000); /*No pattern*/
        write_cmos_sensor(0x0601, 0x0000);
    }

    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = modes;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                 UINT8 *feature_para, UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
    UINT16 *feature_data_16 = (UINT16 *) feature_para;
    UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
    UINT32 *feature_data_32 = (UINT32 *) feature_para;
    unsigned long long *feature_data = (unsigned long long *) feature_para;
    /* unsigned long long *feature_return_para
     *  = (unsigned long long *) feature_para;
     */
    struct SET_PD_BLOCK_INFO_T *PDAFinfo;
    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    struct SENSOR_VC_INFO_STRUCT *pvcinfo;
    /* SET_SENSOR_AWB_GAIN *pSetSensorAWB
     *  = (SET_SENSOR_AWB_GAIN *)feature_para;
     */
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
        = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    /*LOG_INF("feature_id = %d\n", feature_id);*/
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
    case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
        *(MINT32 *)(signed long)(*(feature_data + 1)) = -27000000;
        LOG_INF("exporsure");
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
         /* night_mode((BOOL) *feature_data); */
        break;
    case SENSOR_FEATURE_CHECK_MODULE_ID:
        *feature_return_para_32 = imgsensor_info.module_id;
        break;
    case SENSOR_FEATURE_SET_GAIN:
        set_gain((UINT16) *feature_data);
        break;
    case SENSOR_FEATURE_SET_FLASHLIGHT:
        break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
        break;
    case SENSOR_FEATURE_SET_REGISTER:
        write_cmos_sensor_8(sensor_reg_data->RegAddr,
                    sensor_reg_data->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        sensor_reg_data->RegData =
            read_cmos_sensor_8(sensor_reg_data->RegAddr);
        break;
    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
        /*get the lens driver ID from EEPROM
         * or just return LENS_DRIVER_ID_DO_NOT_CARE
         * if EEPROM does not exist in camera module.
         */
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
        LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
        break;
    case SENSOR_FEATURE_SET_TEST_PATTERN:
        set_test_pattern_mode((UINT8)*feature_data, (struct SET_SENSOR_PATTERN_SOLID_COLOR *)feature_data+1);
        break;
    case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
        /* for factory mode auto testing */
        *feature_return_para_32 = imgsensor_info.checksum_value;
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_SET_FRAMERATE:
        LOG_INF("current fps :%d\n", (UINT32)*feature_data_32);
        spin_lock(&imgsensor_drv_lock);
        imgsensor.current_fps = *feature_data_32;
        spin_unlock(&imgsensor_drv_lock);
        break;
    case SENSOR_FEATURE_SET_HDR:
        LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data_32);
        spin_lock(&imgsensor_drv_lock);
        imgsensor.ihdr_mode = *feature_data_32;
        spin_unlock(&imgsensor_drv_lock);
        break;
    case SENSOR_FEATURE_GET_CROP_INFO:
        LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
            (UINT32)*feature_data);
        wininfo =
    (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

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
    case SENSOR_FEATURE_GET_PDAF_INFO:
        LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
            (UINT16) *feature_data);
        PDAFinfo =
          (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
        break;
    case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
        LOG_INF(
        "SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
            (UINT16) *feature_data);
        /*PDAF capacity enable or not, 2p8 only full size support PDAF*/
        switch (*feature_data) {
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;
        }
        break;
    case SENSOR_FEATURE_SET_PDAF:
        LOG_INF("PDAF mode :%d\n", *feature_data_16);
        imgsensor.pdaf_mode = *feature_data_16;
        break;
    case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
        LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
            (UINT16)*feature_data,
            (UINT16)*(feature_data+1),
            (UINT16)*(feature_data+2));
        break;
    case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
        set_shutter_frame_length((UINT16) (*feature_data),
                    (UINT16) (*(feature_data + 1)));
        break;
    case SENSOR_FEATURE_SET_HDR_SHUTTER:
        LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
            (UINT16)*feature_data, (UINT16)*(feature_data+1));
        break;
    case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
        LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
        streaming_control(KAL_FALSE);
        break;
    case SENSOR_FEATURE_SET_STREAMING_RESUME:
        LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
            *feature_data);
        if (*feature_data != 0)
            set_shutter(*feature_data);
        streaming_control(KAL_TRUE);
        break;
    case SENSOR_FEATURE_GET_BINNING_TYPE:
        switch (*(feature_data + 1)) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *feature_return_para_32 = 1; /*BINNING_AVERAGED*/
            break;
        }
        pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
            *feature_return_para_32);
        *feature_para_len = 4;

        break;
    case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
    {
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.cap.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.normal_video.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.hs_video.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.slim_video.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.pre.mipi_pixel_rate;
            break;
        }
    }
    break;
    case SENSOR_FEATURE_GET_VC_INFO:
        LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n",
            (UINT16)*feature_data);
        pvcinfo =
         (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
        break;
    case SENSOR_FEATURE_SET_AWB_GAIN:
        break;
    case SENSOR_FEATURE_SET_LSC_TBL:
        break;
    case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
        *(feature_data + 1) = 0; /* margin info by scenario */
        *(feature_data + 2) = imgsensor_info.margin;
        break;
    default:
        break;
    }

    return ERROR_NONE;
} /* feature_control() */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 IMX709_MIPI_MONO_21015_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc != NULL)
        *pfFunc = &sensor_func;
    return ERROR_NONE;
} /* IMX709_MIPI_RAW_SensorInit */
