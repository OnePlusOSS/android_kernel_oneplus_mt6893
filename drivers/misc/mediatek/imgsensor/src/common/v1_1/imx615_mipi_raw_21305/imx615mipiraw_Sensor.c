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
 *     IMX615mipi_Sensor.c
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
#include "imx615mipiraw_Sensor.h"

/***************Modify Following Strings for Debug**********************/
#define PFX "imx615_camera_sensor"
#define LOG_1 LOG_INF("IMX615,MIPI 4LANE\n")
/****************************Modify end**************************/

#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

static kal_uint32 streaming_control(kal_bool enable);
#define MODULE_ID_OFFSET 0x0000
static kal_uint16 imx615_table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len);
static kal_uint16 imx615_burst_write_cmos_sensor(kal_uint16 *para, kal_uint32 len);
static uint8_t deviceInfo_register_value = 0;
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = IMX615_SENSOR_ID_21305,
    .checksum_value = 0x8ac2d94a,

    .pre = { /* reg_B-1 3280x2464 @30fps*/
        .pclk = 597600000,
        .linelength = 3768,
        .framelength = 2642,
        .startx = 0,
        .starty = 8,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 567410000,
        .max_framerate = 600, /* 60fps */
    },

    .cap = { /* reg_B-2 3280x2464@60fps*/
        .pclk = 597600000,
        .linelength = 3768,
        .framelength = 2642,
        .startx = 0,
        .starty = 8,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 567410000,
        .max_framerate = 600, /* 60fps */
    },

    .normal_video = { /* reg_C-1 3280x1856@30fps*/
        .pclk = 350400000,
        .linelength = 3768,
        .framelength = 3098,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1856,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 364000000,
        .max_framerate = 300, /* 30fps */
    },

    .hs_video = { /* reg_C-3 3264*1856@120fps*/
        .pclk = 864000000,
        .linelength = 3768,
        .framelength = 1910,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1856,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 842400000,
        .max_framerate = 1200, /* 120fps */
    },

    .slim_video = { /* reg_C-2 3264x1856@60fps*/
        .pclk = 451200000,
        .linelength = 3768,
        .framelength = 1994,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1856,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 421530000,
        .max_framerate = 600, /* 60fps */
    },

    .custom1 = { /*reg_J 1640x1224@15FPS */
        .pclk = 211200000,
        .linelength = 6262,
        .framelength = 2248,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1632,
        .grabwindow_height = 1224,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 211200000,
        .max_framerate = 150, /* 15fps */
    },
    .custom2 = { /*reg_F-1 1640x1232  @30FPS*/
        .pclk = 211200000,
        .linelength = 2248,
        .framelength = 3130,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1632,
        .grabwindow_height = 1224,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 211200000,
        .max_framerate = 300, /* 30fps */
    },
    .custom3 = { //REG_C-1 for remosaic
        .pclk =864000000,
        .linelength = 11480,
        .framelength = 5017,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 6560,
        .grabwindow_height = 4928,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 514450000,
        .max_framerate = 150,
    },
    .custom4 = { //for 720p@240fps
        .pclk = 674400000,
        .linelength = 2248,
        .framelength = 1250,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1632,
        .grabwindow_height = 918,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 578400000,
        .max_framerate = 2400, /* 240fps */
    },
    .custom5 = { /* reg_B-1 3280x2464 @30fps*/
        .pclk = 289200000,
        .linelength = 3768,
        .framelength = 2558,
        .startx = 0,
        .starty = 8,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 267320000,
        .max_framerate = 300, /* 30fps */
    },

    .margin = 48,        /* sensor framelength & shutter margin */
    .min_shutter = 16,    /* min shutter */
    .min_gain = 64, /*1x gain*/
    .max_gain = 4096, /*64x gain*/
    .min_gain_iso = 100,
    .gain_step = 1,
    .gain_type = 0,
    .max_frame_length = 0xffff-5,
    .ae_shut_delay_frame = 0,
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,    /* isp gain delay frame for AE cycle */
    .ihdr_support = 0,    /* 1, support; 0,not support */
    .ihdr_le_firstline = 0,    /* 1,le first ; 0, se first */
    .sensor_mode_num = 10,    /*support sensor mode num*/

    .cap_delay_frame = 2,  /*3 guanjd modify for cts*/
    .pre_delay_frame = 2,  /*3 guanjd modify for cts*/
    .video_delay_frame = 3,
    .hs_video_delay_frame = 3,
    .slim_video_delay_frame = 3,
    .custom1_delay_frame = 2,
    .custom2_delay_frame = 2,
    .custom3_delay_frame = 2,
    .custom4_delay_frame = 2,
    .custom5_delay_frame = 2,

    .isp_driving_current = ISP_DRIVING_6MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2, /*only concern if it's cphy*/
    .mipi_settle_delay_mode = 0,
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,
    .mclk = 24, /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
    /*.mipi_lane_num = SENSOR_MIPI_4_LANE,*/
    .mipi_lane_num = SENSOR_MIPI_4_LANE,
    .i2c_addr_table = {0x20, 0x21, 0xff},
    /* record sensor support all write id addr,
     * only supprt 4 must end with 0xff
     */
    .i2c_speed = 1000, /* i2c read/write speed */
};

static struct imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,    /* NORMAL information */
    .sensor_mode = IMGSENSOR_MODE_INIT,
    /* IMGSENSOR_MODE enum value,record current sensor mode,such as:
     * INIT, Preview, Capture, Video,High Speed Video, Slim Video
     */
    .shutter = 0x3D0,    /* current shutter */
    .gain = 0x100,        /* current gain */
    .dummy_pixel = 0,    /* current dummypixel */
    .dummy_line = 0,    /* current dummyline */
    .current_fps = 300,
    .autoflicker_en = KAL_FALSE,
    .test_pattern = 0,
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
    .ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
    .i2c_write_id = 0x20, /* record current sensor's i2c write id */
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
    {6560, 4928, 0,   0, 6560, 4928, 3280, 2464,  8,   0, 3264, 2464,  0,  8, 3264, 2448}, /* Preview */
    {6560, 4928, 0,   0, 6560, 4928, 3280, 2464,  8,   0, 3264, 2464,  0,  8, 3264, 2448}, /* capture */
    {6560, 4928, 0, 608, 6560, 3712, 3280, 1856,  8,   0, 3264, 1856,  0,  0, 3264, 1856}, /* normal video */
    {6560, 4928, 0, 608, 6560, 3712, 3280, 1856,  8,   0, 3264, 1856,  0,  0, 3264, 1856}, /* hs_video */
    {6560, 4928, 0, 608, 6560, 3712, 3280, 1856,  8,   0, 3264, 1856,  0,  0, 3264, 1856}, /* slim video */
    {6560, 4928, 0,   0, 6560, 4928, 1640, 1232,  0,   0, 1640, 1232,  4,  4, 1632, 1224}, /* custom1 */
    {6560, 4928, 0,  16, 6560, 4896, 1640, 1224,  4,   0, 1632, 1224,  0,  0, 1632, 1224}, /* custom2 */
    {6560, 4928, 0,   0, 6560, 4928, 6560, 4928,  0,   0, 6560, 4928,  0,  0, 6560, 4928}, /* custom3 */
    {6560, 4928, 0, 628, 6560, 3672, 1640,  918,  4,   0, 1632,  918,  0,  0, 1632,  918}, /* custom4 */
    {6560, 4928, 0,   0, 6560, 4928, 3280, 2464,  8,   0, 3264, 2464,  0,  8, 3264, 2448}, /* custom5 */
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

static kal_uint16 imx615_QSC_setting[1560*2];
static kal_uint8 qsc_flag = 0;
static void read_EepromQSC(void)
{
    kal_uint16 addr_qsc = 0xE00, sensoraddr_qsc = 0xC500;
    int i = 0;
    for (i = 0; i < 1560; i++) {
        imx615_QSC_setting[2*i] = sensoraddr_qsc+i;
        imx615_QSC_setting[2*i+1] = Eeprom_1ByteDataRead((addr_qsc+i), 0xA8);
    }

    /*Read normal eeprom data*/
    gImgEepromInfo.camNormdata[1][0] = Eeprom_1ByteDataRead(0x00, 0xA8);
    gImgEepromInfo.camNormdata[1][1] = Eeprom_1ByteDataRead(0x01, 0xA8);
    imgsensor_info.module_id = Eeprom_1ByteDataRead(0x00, 0xA8);
    Oplusimgsensor_Registdeviceinfo(gImgEepromInfo.pCamModuleInfo[1].name,
                                    gImgEepromInfo.pCamModuleInfo[1].version,
                                    imgsensor_info.module_id);
    for(i = 2; i < 8; i++) {
        gImgEepromInfo.camNormdata[1][i] = Eeprom_1ByteDataRead(0x04+i, 0xA8);
    }
    for (i = 0; i < OPLUS_CAMERASN_LENS; i++) {
        gImgEepromInfo.camNormdata[1][8+i] = Eeprom_1ByteDataRead(0xB0+i, 0xA8);
    }
    gImgEepromInfo.i4CurSensorIdx = 1;
    gImgEepromInfo.i4CurSensorId = imgsensor_info.sensor_id;
}

static void write_sensor_QSC(void)
{
    LOG_INF("%s start %x %d\n", __func__, Eeprom_1ByteDataRead(0x1418, 0xA8), qsc_flag);
    if (Eeprom_1ByteDataRead(0x1418, 0xA8) == 1 && !qsc_flag) {
        imx615_burst_write_cmos_sensor(imx615_QSC_setting,
            sizeof(imx615_QSC_setting) / sizeof(kal_uint16));
        qsc_flag = 1;
    }
    LOG_INF("%s end\n", __func__);
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
    LOG_INF("dummyline = %d, dummypixels = %d\n",
        imgsensor.dummy_line, imgsensor.dummy_pixel);
    /* return;*/ /* for test */
    if (imgsensor.shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = imgsensor.shutter + imgsensor_info.margin;
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
    itemp = read_cmos_sensor_8(0x0101);
    itemp &= ~0x03;

    switch (image_mirror) {

    case IMAGE_NORMAL:
    write_cmos_sensor_8(0x0101, itemp);
    break;

    case IMAGE_V_MIRROR:
    write_cmos_sensor_8(0x0101, itemp | 0x02);
    break;

    case IMAGE_H_MIRROR:
    write_cmos_sensor_8(0x0101, itemp | 0x01);
    break;

    case IMAGE_HV_MIRROR:
    write_cmos_sensor_8(0x0101, itemp | 0x03);
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

    reg_gain = 4096 - (1024*64)/gain;
    if (imgsensor.current_scenario_id == IMGSENSOR_MODE_CUSTOM3) {
        reg_gain = 1024 - (1024*64)/gain;
    }
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

    if (imgsensor.current_scenario_id == IMGSENSOR_MODE_CUSTOM3) {
        max_gain = 16 * BASEGAIN;
    }

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
static kal_uint16 imx615_table_write_cmos_sensor(kal_uint16 *para,
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

static kal_uint16 imx615_burst_write_cmos_sensor(kal_uint16 *para,
                            kal_uint32 len)
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

static kal_uint16 imx615_init_setting[] = {
    0x0100,0x00,
    //External Clock Setting
    0x0136,0x18,
    0x0137,0x00,
    //register version
    0x3C7E,0x02,
    0x3C7F,0x09,
    //signaling mode setting
    0x0111,0x02,
    //Global Setting
    0x380C,0x00,
    0x3C00,0x01,
    0x3C01,0x00,
    0x3C02,0x00,
    0x3C03,0x03,
    0x3C04,0xFF,
    0x3C05,0x01,
    0x3C06,0x00,
    0x3C07,0x00,
    0x3C08,0x03,
    0x3C09,0xFF,
    0x3C0A,0x00,
    0x3C0B,0x00,
    0x3C0C,0x10,
    0x3C0D,0x10,
    0x3C0E,0x10,
    0x3C0F,0x10,
    0x3C10,0x10,
    0x3C11,0x20,
    0x3C15,0x00,
    0x3C16,0x00,
    0x3C17,0x00,
    0x3C18,0x00,
    0x3C19,0x01,
    0x3C1A,0x00,
    0x3C1B,0x01,
    0x3C1C,0x00,
    0x3C1D,0x01,
    0x3C1E,0x00,
    0x3C1F,0x00,
    0x3F89,0x01,
    0x3F8F,0x01,
    0x53B9,0x01,
    0x62C4,0x04,
    0x658F,0x07,
    0x6590,0x05,
    0x6591,0x07,
    0x6592,0x05,
    0x6593,0x07,
    0x6594,0x05,
    0x6595,0x07,
    0x6596,0x05,
    0x6597,0x05,
    0x6598,0x05,
    0x6599,0x05,
    0x659A,0x05,
    0x659B,0x05,
    0x659C,0x05,
    0x659D,0x05,
    0x659E,0x07,
    0x659F,0x05,
    0x65A0,0x07,
    0x65A1,0x05,
    0x65A2,0x07,
    0x65A3,0x05,
    0x65A4,0x07,
    0x65A5,0x05,
    0x65A6,0x05,
    0x65A7,0x05,
    0x65A8,0x05,
    0x65A9,0x05,
    0x65AA,0x05,
    0x65AB,0x05,
    0x65AC,0x05,
    0x65AD,0x07,
    0x65AE,0x07,
    0x65AF,0x07,
    0x65B0,0x05,
    0x65B1,0x05,
    0x65B2,0x05,
    0x65B3,0x05,
    0x65B4,0x07,
    0x65B5,0x07,
    0x65B6,0x07,
    0x65B7,0x07,
    0x65B8,0x05,
    0x65B9,0x05,
    0x65BA,0x05,
    0x65BB,0x05,
    0x65BC,0x05,
    0x65BD,0x05,
    0x65BE,0x05,
    0x65BF,0x05,
    0x65C0,0x05,
    0x65C1,0x05,
    0x65C2,0x05,
    0x65C3,0x05,
    0x65C4,0x05,
    0x65C5,0x05,
    0x6E1C,0x00,
    0x6E1D,0x00,
    0x6E25,0x00,
    0x6E38,0x03,
    0x895C,0x01,
    0x895D,0x00,
    0x8966,0x00,
    0x8967,0x4E,
    0x896A,0x00,
    0x896B,0x24,
    0x896F,0x34,
    0x8976,0x00,
    0x8977,0x00,
    0x9004,0x15,
    0x9200,0xB7,
    0x9201,0x34,
    0x9202,0xB7,
    0x9203,0x36,
    0x9204,0xB7,
    0x9205,0x37,
    0x9206,0xB7,
    0x9207,0x38,
    0x9208,0xB7,
    0x9209,0x39,
    0x920A,0xB7,
    0x920B,0x3A,
    0x920C,0xB7,
    0x920D,0x3C,
    0x920E,0xB7,
    0x920F,0x3D,
    0x9210,0xB7,
    0x9211,0x3E,
    0x9212,0xB7,
    0x9213,0x3F,
    0x9214,0xF6,
    0x9215,0x13,
    0x9216,0xF6,
    0x9217,0x34,
    0x9218,0xF4,
    0x9219,0xA7,
    0x921A,0xF4,
    0x921B,0xAA,
    0x921C,0xF4,
    0x921D,0xAD,
    0x921E,0xF4,
    0x921F,0xB0,
    0x9220,0xF4,
    0x9221,0xB3,
    0x9222,0x85,
    0x9223,0x77,
    0x9224,0xC4,
    0x9225,0x4B,
    0x9226,0xC4,
    0x9227,0x4C,
    0x9228,0xC4,
    0x9229,0x4D,
    0x922A,0xF5,
    0x922B,0x5E,
    0x922C,0xF5,
    0x922D,0x5F,
    0x922E,0xF5,
    0x922F,0x64,
    0x9230,0xF5,
    0x9231,0x65,
    0x9232,0xF5,
    0x9233,0x6A,
    0x9234,0xF5,
    0x9235,0x6B,
    0x9236,0xF5,
    0x9237,0x70,
    0x9238,0xF5,
    0x9239,0x71,
    0x923A,0xF5,
    0x923B,0x76,
    0x923C,0xF5,
    0x923D,0x77,
    0x9810,0x14,
    0x9814,0x14,
    0xC020,0x00,
    0xC026,0x00,
    0xC027,0x00,
    0xC448,0x01,
    0xC44F,0x01,
    0xC450,0x00,
    0xC451,0x00,
    0xC452,0x01,
    0xC455,0x00,
    0xE206,0x35,
    0xE226,0x33,
    0xE266,0x34,
    0xE2A6,0x31,
    0xE2C6,0x37,
    0xE2E6,0x32,
    //image quality setting
    0x88D6,0x60,
    0x9852,0x00,
    0xA569,0x06,
    0xA56A,0x13,
    0xA56B,0x13,
    0xA56C,0x01,
    0xA678,0x00,
    0xA679,0x20,
    0xA812,0x00,
    0xA813,0x3F,
    0xA814,0x3F,
    0xA830,0x68,
    0xA831,0x56,
    0xA832,0x2B,
    0xA833,0x55,
    0xA834,0x55,
    0xA835,0x16,
    0xA837,0x51,
    0xA838,0x34,
    0xA854,0x4F,
    0xA855,0x48,
    0xA856,0x45,
    0xA857,0x02,
    0xA85A,0x23,
    0xA85B,0x16,
    0xA85C,0x12,
    0xA85D,0x02,
    0xAA55,0x00,
    0xAA56,0x01,
    0xAA57,0x30,
    0xAA58,0x01,
    0xAA59,0x30,
    0xAC72,0x01,
    0xAC73,0x26,
    0xAC74,0x01,
    0xAC75,0x26,
    0xAC76,0x00,
    0xAC77,0xC4,
    0xAE09,0xFF,
    0xAE0A,0xFF,
    0xAE12,0x58,
    0xAE13,0x58,
    0xAE15,0x10,
    0xAE16,0x10,
    0xAF05,0x48,
    0xB069,0x02,
    0xEA4B,0x00,
    0xEA4C,0x00,
    0xEA4D,0x00,
    0xEA4E,0x00,
    //disable embedded line
    0xBCF1,0x00,
};

static kal_uint16 imx615_capture_setting[] = {
    //MIPI output setting
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x03,
    //Line Length PCK Setting
    0x0342,0x0E,
    0x0343,0xB8,
    //Frame Length Lines Setting
    0x0340,0x09,
    0x0341,0xFE,
    //ROI Setting
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x00,
    0x0347,0x00,
    0x0348,0x19,
    0x0349,0x9F,
    0x034A,0x13,
    0x034B,0x3F,
    //Mode Setting
    0x0900,0x01,
    0x0901,0x22,
    0x0902,0x08,
    0x3246,0x81,
    0x3247,0x81,
    //Digital Crop & Scaling
    0x0401,0x00,
    0x0404,0x00,
    0x0405,0x10,
    0x0408,0x00,
    0x0409,0x00,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x0C,
    0x040D,0xD0,
    0x040E,0x09,
    0x040F,0xA0,
    //Output Size Setting
    0x034C,0x0C,
    0x034D,0xD0,
    0x034E,0x09,
    0x034F,0xA0,
    //Clock Setting
    0x0301,0x05,
    0x0303,0x04,
    0x0305,0x04,
    0x0306,0x00,
    0x0307,0xF1,
    0x030B,0x02,
    0x030D,0x0D,
    0x030E,0x02,
    0x030F,0xD4,
    0x0310,0x01,
    //Other Setting
    0x3620,0x00,
    0x3621,0x00,
    0x3C12,0x56,
    0x3C13,0x52,
    0x3C14,0x3E,
    0x3F0C,0x00,
    0x3F14,0x01,
    0x3F80,0x00,
    0x3F81,0xA0,
    0x3F8C,0x00,
    0x3F8D,0x00,
    0x3FFC,0x00,
    0x3FFD,0x1E,
    0x3FFE,0x00,
    0x3FFF,0xDC,
    //Integration Setting
    0x0202,0x09,
    0x0203,0xCE,
    //Gain Setting
    0x0204,0x00,
    0x0205,0x70,
    0x020E,0x01,
    0x020F,0x00,
    0x0210,0x01,
    0x0211,0x00,
    0x0212,0x01,
    0x0213,0x00,
    0x0214,0x01,
    0x0215,0x00,
};

static kal_uint16 imx615_preview_setting[] = {
    //MIPI output setting
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x03,
    //Line Length PCK Setting
    0x0342,0x0E,
    0x0343,0xB8,
    //Frame Length Lines Setting
    0x0340,0x0A,
    0x0341,0x52,
    //ROI Setting
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x00,
    0x0347,0x00,
    0x0348,0x19,
    0x0349,0x9F,
    0x034A,0x13,
    0x034B,0x3F,
    //Mode Setting
    0x0900,0x01,
    0x0901,0x22,
    0x0902,0x08,
    0x3246,0x81,
    0x3247,0x81,
    //Digital Crop & Scaling
    0x0401,0x00,
    0x0404,0x00,
    0x0405,0x10,
    0x0408,0x00,
    0x0409,0x00,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x0C,
    0x040D,0xD0,
    0x040E,0x09,
    0x040F,0xA0,
    //Output Size Setting
    0x034C,0x0C,
    0x034D,0xD0,
    0x034E,0x09,
    0x034F,0xA0,
    //Clock Setting
    0x0301,0x05,
    0x0303,0x02,
    0x0305,0x04,
    0x0306,0x00,
    0x0307,0xF9,
    0x030B,0x01,
    0x030D,0x13,
    0x030E,0x04,
    0x030F,0x63,
    0x0310,0x01,
    //Other Setting
    0x3620,0x00,
    0x3621,0x00,
    0x3C12,0x56,
    0x3C13,0x52,
    0x3C14,0x3E,
    0x3F0C,0x00,
    0x3F14,0x01,
    0x3F80,0x00,
    0x3F81,0xA0,
    0x3F8C,0x00,
    0x3F8D,0x00,
    0x3FFC,0x00,
    0x3FFD,0x1E,
    0x3FFE,0x00,
    0x3FFF,0xDC,
    //Integration Setting
    0x0202,0x0A,
    0x0203,0x22,
    //Gain Setting
    0x0204,0x00,
    0x0205,0x70,
    0x020E,0x01,
    0x020F,0x00,
    0x0210,0x01,
    0x0211,0x00,
    0x0212,0x01,
    0x0213,0x00,
    0x0214,0x01,
    0x0215,0x00,
};

static kal_uint16 imx615_normal_video_setting[] = {
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x0E,
    0x0343, 0xB8,
    0x0340, 0x0C,
    0x0341, 0x1A,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x60,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0xDF,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x08,
    0x3246, 0x81,
    0x3247, 0x81,
    0x0401, 0x00,
    0x0404, 0x00,
    0x0405, 0x10,
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x07,
    0x040F, 0x40,
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x07,
    0x034F, 0x40,
    0x0301, 0x05,
    0x0303, 0x04,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x24,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xC7,
    0x0310, 0x01,
    0x3620, 0x00,
    0x3621, 0x00,
    0x3C12, 0x56,
    0x3C13, 0x52,
    0x3C14, 0x3E,
    0x3F0C, 0x00,
    0x3F14, 0x01,
    0x3F80, 0x00,
    0x3F81, 0xA0,
    0x3F8C, 0x00,
    0x3F8D, 0x00,
    0x3FFC, 0x00,
    0x3FFD, 0x1E,
    0x3FFE, 0x00,
    0x3FFF, 0xDC,
    0x0202, 0x0B,
    0x0203, 0xEA,
    0x0204, 0x00,
    0x0205, 0x70,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0210, 0x01,
    0x0211, 0x00,
    0x0212, 0x01,
    0x0213, 0x00,
    0x0214, 0x01,
    0x0215, 0x00,
};

static kal_uint16 imx615_hs_video_setting[] = {
/*MIPI output setting*/
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
/*Line Length PCK Setting*/
    0x0342, 0x0E,
    0x0343, 0xB8,
/*Frame Length Lines Setting*/
    0x0340, 0x07,
    0x0341, 0x76,
/*ROI Setting*/
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x60,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0xDF,
/*Mode Setting*/
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x08,
    0x3246, 0x81,
    0x3247, 0x81,
/*Digital Crop & Scaling*/
    0x0401, 0x00,
    0x0404, 0x00,
    0x0405, 0x10,
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xD0,
    0x040E, 0x07,
    0x040F, 0x40,
/*Output Size Setting*/
    0x034C, 0x0C,
    0x034D, 0xD0,
    0x034E, 0x07,
    0x034F, 0x40,
/*Clock Setting*/
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x68,
    0x030B, 0x01,
    0x030D, 0x04,
    0x030E, 0x01,
    0x030F, 0x5F,
    0x0310, 0x01,
/*Other Setting*/
    0x3620, 0x00,
    0x3621, 0x00,
    0x3C12, 0x56,
    0x3C13, 0x52,
    0x3C14, 0x3E,
    0x3F0C, 0x00,
    0x3F14, 0x01,
    0x3F80, 0x00,
    0x3F81, 0xA0,
    0x3F8C, 0x00,
    0x3F8D, 0x00,
    0x3FFC, 0x00,
    0x3FFD, 0x1E,
    0x3FFE, 0x00,
    0x3FFF, 0xDC,
/*Integration Setting*/
    0x0202, 0x07,
    0x0203, 0x46,
/*Gain Setting*/
    0x0204, 0x00,
    0x0205, 0x70,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0210, 0x01,
    0x0211, 0x00,
    0x0212, 0x01,
    0x0213, 0x00,
    0x0214, 0x01,
    0x0215, 0x00,
};

static kal_uint16 imx615_slim_video_setting[] = {
    //MIPI output setting
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x03,
    //Line Length PCK Setting
    0x0342,0x0E,
    0x0343,0xB8,
    //Frame Length Lines Setting
    0x0340,0x07,
    0x0341,0xCA,
    //ROI Setting
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x02,
    0x0347,0x60,
    0x0348,0x19,
    0x0349,0x9F,
    0x034A,0x10,
    0x034B,0xDF,
    //Mode Setting
    0x0900,0x01,
    0x0901,0x22,
    0x0902,0x08,
    0x3246,0x81,
    0x3247,0x81,
    //Digital Crop & Scaling
    0x0401,0x00,
    0x0404,0x00,
    0x0405,0x10,
    0x0408,0x00,
    0x0409,0x00,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x0C,
    0x040D,0xD0,
    0x040E,0x07,
    0x040F,0x40,
    //Output Size Setting
    0x034C,0x0C,
    0x034D,0xD0,
    0x034E,0x07,
    0x034F,0x40,
    //Clock Setting
    0x0301,0x05,
    0x0303,0x02,
    0x0305,0x04,
    0x0306,0x00,
    0x0307,0xBC,
    0x030B,0x02,
    0x030D,0x0B,
    0x030E,0x03,
    0x030F,0xC6,
    0x0310,0x01,
    //Other Setting
    0x3620,0x00,
    0x3621,0x00,
    0x3C12,0x56,
    0x3C13,0x52,
    0x3C14,0x3E,
    0x3F0C,0x00,
    0x3F14,0x01,
    0x3F80,0x00,
    0x3F81,0xA0,
    0x3F8C,0x00,
    0x3F8D,0x00,
    0x3FFC,0x00,
    0x3FFD,0x1E,
    0x3FFE,0x00,
    0x3FFF,0xDC,
    //Integration Setting
    0x0202,0x07,
    0x0203,0x9A,
    //Gain Setting
    0x0204,0x00,
    0x0205,0x70,
    0x020E,0x01,
    0x020F,0x00,
    0x0210,0x01,
    0x0211,0x00,
    0x0212,0x01,
    0x0213,0x00,
    0x0214,0x01,
    0x0215,0x00,
};

static kal_uint16 imx615_custom1_setting[] = {
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x03,
    0x0342,0x08,
    0x0343,0xC8,
    0x0340,0x18,
    0x0341,0x76,
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x00,
    0x0347,0x10,
    0x0348,0x19,
    0x0349,0x9F,
    0x034A,0x13,
    0x034B,0x2F,
    0x0900,0x01,
    0x0901,0x44,
    0x0902,0x08,
    0x3246,0x89,
    0x3247,0x89,
    0x0401,0x00,
    0x0404,0x00,
    0x0405,0x10,
    0x0408,0x00,
    0x0409,0x04,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x06,
    0x040D,0x60,
    0x040E,0x04,
    0x040F,0xC8,
    0x034C,0x06,
    0x034D,0x60,
    0x034E,0x04,
    0x034F,0xC8,
    0x0301,0x05,
    0x0303,0x04,
    0x0305,0x04,
    0x0306,0x00,
    0x0307,0xB0,
    0x030B,0x02,
    0x030D,0x04,
    0x030E,0x00,
    0x030F,0xB0,
    0x0310,0x01,
    0x3620,0x00,
    0x3621,0x00,
    0x3C12,0x3E,
    0x3C13,0x3A,
    0x3C14,0x22,
    0x3F0C,0x00,
    0x3F14,0x00,
    0x3F80,0x00,
    0x3F81,0x00,
    0x3F8C,0x00,
    0x3F8D,0x00,
    0x3FFC,0x00,
    0x3FFD,0x55,
    0x3FFE,0x00,
    0x3FFF,0x78,
    0x0202,0x18,
    0x0203,0x46,
    0x0204,0x00,
    0x0205,0x70,
    0x020E,0x01,
    0x020F,0x00,
    0x0210,0x01,
    0x0211,0x00,
    0x0212,0x01,
    0x0213,0x00,
    0x0214,0x01,
    0x0215,0x00,
};

static kal_uint16 imx615_custom2_setting[] = {
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x03,
    0x0342,0x08,
    0x0343,0xC8,
    0x0340,0x0C,
    0x0341,0x3A,
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x00,
    0x0347,0x10,
    0x0348,0x19,
    0x0349,0x9F,
    0x034A,0x13,
    0x034B,0x2F,
    0x0900,0x01,
    0x0901,0x44,
    0x0902,0x08,
    0x3246,0x89,
    0x3247,0x89,
    0x0401,0x00,
    0x0404,0x00,
    0x0405,0x10,
    0x0408,0x00,
    0x0409,0x04,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x06,
    0x040D,0x60,
    0x040E,0x04,
    0x040F,0xC8,
    0x034C,0x06,
    0x034D,0x60,
    0x034E,0x04,
    0x034F,0xC8,
    0x0301,0x05,
    0x0303,0x04,
    0x0305,0x04,
    0x0306,0x00,
    0x0307,0xB0,
    0x030B,0x02,
    0x030D,0x04,
    0x030E,0x00,
    0x030F,0xB0,
    0x0310,0x01,
    0x3620,0x00,
    0x3621,0x00,
    0x3C12,0x3E,
    0x3C13,0x3A,
    0x3C14,0x22,
    0x3F0C,0x00,
    0x3F14,0x00,
    0x3F80,0x00,
    0x3F81,0x00,
    0x3F8C,0x00,
    0x3F8D,0x00,
    0x3FFC,0x00,
    0x3FFD,0x55,
    0x3FFE,0x00,
    0x3FFF,0x78,
    0x0202,0x0C,
    0x0203,0x0A,
    0x0204,0x00,
    0x0205,0x70,
    0x020E,0x01,
    0x020F,0x00,
    0x0210,0x01,
    0x0211,0x00,
    0x0212,0x01,
    0x0213,0x00,
    0x0214,0x01,
    0x0215,0x00,
};

static kal_uint16 imx615_custom3_setting[] = {
    //MIPI output setting
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x03,
    //Line Length PCK Setting
    0x0342,0x2C,
    0x0343,0xD8,
    //Frame Length Lines Setting
    0x0340,0x13,
    0x0341,0x99,
    //ROI Setting
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x00,
    0x0347,0x00,
    0x0348,0x19,
    0x0349,0x9F,
    0x034A,0x13,
    0x034B,0x3F,
    //Mode Setting
    0x0900,0x00,
    0x0901,0x11,
    0x0902,0x0A,
    0x3246,0x01,
    0x3247,0x01,
    //Digital Crop & Scaling
    0x0401,0x00,
    0x0404,0x00,
    0x0405,0x10,
    0x0408,0x00,
    0x0409,0x00,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x19,
    0x040D,0xA0,
    0x040E,0x13,
    0x040F,0x40,
    //Output Size Setting
    0x034C,0x19,
    0x034D,0xA0,
    0x034E,0x13,
    0x034F,0x40,
    //Clock Setting
    0x0301,0x05,
    0x0303,0x02,
    0x0305,0x04,
    0x0306,0x01,
    0x0307,0x68,
    0x030B,0x01,
    0x030D,0x11,
    0x030E,0x03,
    0x030F,0x8F,
    0x0310,0x01,
    //Other Setting
    0x3620,0x01,
    0x3621,0x01,
    0x3C12,0x62,
    0x3C13,0x32,
    0x3C14,0x20,
    0x3F0C,0x00,
    0x3F14,0x01,
    0x3F80,0x00,
    0x3F81,0x46,
    0x3F8C,0x00,
    0x3F8D,0x00,
    0x3FFC,0x00,
    0x3FFD,0x6E,
    0x3FFE,0x00,
    0x3FFF,0x64,
    //Integration Setting
    0x0202,0x13,
    0x0203,0x69,
    //Gain Setting
    0x0204,0x00,
    0x0205,0x70,
    0x020E,0x01,
    0x020F,0x00,
    0x0210,0x01,
    0x0211,0x00,
    0x0212,0x01,
    0x0213,0x00,
    0x0214,0x01,
// line broken correction
    0xC47E,0x01,
    0xC47F,0x01,
// False color correction
    0x9856,0x78,
    0x9857,0x00,
    0x9858,0x00,
    0xAF00,0x00,
    0xAF01,0x04,
    0xAF02,0x00,
    0xAF03,0x04,
    0xAF04,0x00,
    0xAF05,0x08,
// LPF
    0xAE06,0x04,
    0xAE09,0x16,
    0xAE07,0x16,
    0xAE0A,0x16,
    0xAE08,0x16,
    0xAE0B,0x16,
};

static kal_uint16 imx615_custom4_setting[] = {
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x08,
    0x0343, 0xC8,
    0x0340, 0x04,
    0x0341, 0xE2,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x70,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0xCF,
    0x0900, 0x01,
    0x0901, 0x44,
    0x0902, 0x08,
    0x3246, 0x89,
    0x3247, 0x89,
    0x0401, 0x00,
    0x0404, 0x00,
    0x0405, 0x10,
    0x0408, 0x00,
    0x0409, 0x04,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x06,
    0x040D, 0x60,
    0x040E, 0x03,
    0x040F, 0x96,
    0x034C, 0x06,
    0x034D, 0x60,
    0x034E, 0x03,
    0x034F, 0x96,
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x19,
    0x030B, 0x01,
    0x030D, 0x04,
    0x030E, 0x00,
    0x030F, 0xF1,
    0x0310, 0x01,
    0x3620, 0x00,
    0x3621, 0x00,
    0x3C12, 0x3E,
    0x3C13, 0x3A,
    0x3C14, 0x22,
    0x3F0C, 0x00,
    0x3F14, 0x00,
    0x3F80, 0x00,
    0x3F81, 0x00,
    0x3F8C, 0x00,
    0x3F8D, 0x00,
    0x3FFC, 0x00,
    0x3FFD, 0x55,
    0x3FFE, 0x00,
    0x3FFF, 0x78,
    0x0202, 0x04,
    0x0203, 0xB2,
    0x0204, 0x00,
    0x0205, 0x70,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0210, 0x01,
    0x0211, 0x00,
    0x0212, 0x01,
    0x0213, 0x00,
    0x0214, 0x01,
    0x0215, 0x00,
};
static kal_uint16 imx615_custom5_setting[] = {
    //MIPI output setting
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x03,
    //Line Length PCK Setting
    0x0342,0x0E,
    0x0343,0xB8,
    //Frame Length Lines Setting
    0x0340,0x09,
    0x0341,0xFE,
    //ROI Setting
    0x0344,0x00,
    0x0345,0x00,
    0x0346,0x00,
    0x0347,0x00,
    0x0348,0x19,
    0x0349,0x9F,
    0x034A,0x13,
    0x034B,0x3F,
    //Mode Setting
    0x0900,0x01,
    0x0901,0x22,
    0x0902,0x08,
    0x3246,0x81,
    0x3247,0x81,
    //Digital Crop & Scaling
    0x0401,0x00,
    0x0404,0x00,
    0x0405,0x10,
    0x0408,0x00,
    0x0409,0x00,
    0x040A,0x00,
    0x040B,0x00,
    0x040C,0x0C,
    0x040D,0xD0,
    0x040E,0x09,
    0x040F,0xA0,
    //Output Size Setting
    0x034C,0x0C,
    0x034D,0xD0,
    0x034E,0x09,
    0x034F,0xA0,
    //Clock Setting
    0x0301,0x05,
    0x0303,0x04,
    0x0305,0x04,
    0x0306,0x00,
    0x0307,0xF1,
    0x030B,0x02,
    0x030D,0x0D,
    0x030E,0x02,
    0x030F,0xD4,
    0x0310,0x01,
    //Other Setting
    0x3620,0x00,
    0x3621,0x00,
    0x3C12,0x56,
    0x3C13,0x52,
    0x3C14,0x3E,
    0x3F0C,0x00,
    0x3F14,0x01,
    0x3F80,0x00,
    0x3F81,0xA0,
    0x3F8C,0x00,
    0x3F8D,0x00,
    0x3FFC,0x00,
    0x3FFD,0x1E,
    0x3FFE,0x00,
    0x3FFF,0xDC,
    //Integration Setting
    0x0202,0x09,
    0x0203,0xCE,
    //Gain Setting
    0x0204,0x00,
    0x0205,0x70,
    0x020E,0x01,
    0x020F,0x00,
    0x0210,0x01,
    0x0211,0x00,
    0x0212,0x01,
    0x0213,0x00,
    0x0214,0x01,
    0x0215,0x00,
    //hs_trail
    0x0808,0x02,
    0x080A,0x00,
    0x080B,0x6F,
    0x080C,0x00,
    0x080D,0x2F,
    0x080E,0x00,
    0x080F,0x57,
    0x0810,0x00,
    0x0811,0x2F,
    0x0812,0x00,
    0x0813,0x2F,
    0x0814,0x00,
    0x0815,0x2F,
    0x0816,0x00,
    0x0817,0xBF,
    0x0818,0x00,
    0x0819,0x27,
    0x0824,0x00,
    0x0825,0x4F,
    0x0826,0x00,
    0x0827,0x0F,
};

static void sensor_init(void)
{
    LOG_INF("sensor_init start\n");
    imx615_table_write_cmos_sensor(imx615_init_setting,
        sizeof(imx615_init_setting)/sizeof(kal_uint16));
    set_mirror_flip(imgsensor.mirror);
    LOG_INF("sensor_init End\n");
}    /*      sensor_init  */

static void preview_setting(void)
{
    LOG_INF("%s start\n", __func__);
    imx615_table_write_cmos_sensor(imx615_preview_setting,
        sizeof(imx615_preview_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
} /* preview_setting */


/*full size 30fps*/
static void capture_setting(kal_uint16 currefps)
{
    LOG_INF("%s start\n", __func__);
    imx615_table_write_cmos_sensor(imx615_capture_setting,
        sizeof(imx615_capture_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void normal_video_setting(kal_uint16 currefps)
{
    LOG_INF("%s start\n", __func__);
    imx615_table_write_cmos_sensor(imx615_normal_video_setting,
        sizeof(imx615_normal_video_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void hs_video_setting(void)
{
    LOG_INF("%s start\n", __func__);
    imx615_table_write_cmos_sensor(imx615_hs_video_setting,
        sizeof(imx615_hs_video_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void slim_video_setting(void)
{
    LOG_INF("%s start\n", __func__);
    imx615_table_write_cmos_sensor(imx615_slim_video_setting,
        sizeof(imx615_slim_video_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom1_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx615_table_write_cmos_sensor(imx615_custom1_setting,
        sizeof(imx615_custom1_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom2_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx615_table_write_cmos_sensor(imx615_custom2_setting,
        sizeof(imx615_custom2_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom3_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx615_table_write_cmos_sensor(imx615_custom3_setting,
        sizeof(imx615_custom3_setting)/sizeof(kal_uint16));
    if (Eeprom_1ByteDataRead(0x1418, 0xA8) == 1) {
        pr_info("OTP QSC Data Valid, enable qsc register");
    } else {
        pr_info("OTP no QSC Data, close qsc register");
        write_cmos_sensor_8(0x3621, 0x00);
    }
    LOG_INF("%s end\n", __func__);
}

static void custom4_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx615_table_write_cmos_sensor(imx615_custom4_setting,
        sizeof(imx615_custom4_setting)/sizeof(kal_uint16));
     LOG_INF("%s end\n", __func__);
}

static void custom5_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx615_table_write_cmos_sensor(imx615_custom5_setting,
        sizeof(imx615_custom5_setting)/sizeof(kal_uint16));
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
            if (*sensor_id == IMX615_SENSOR_ID) {
                *sensor_id = imgsensor_info.sensor_id;
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
                    imgsensor.i2c_write_id, *sensor_id);
                imgsensor_info.module_id = read_module_id();
                read_EepromQSC();
                if(deviceInfo_register_value == 0x00){
                    Eeprom_DataInit(IMGSENSOR_SENSOR_IDX_SUB, imgsensor_info.sensor_id);
                    deviceInfo_register_value = 0x01;
                }
                LOG_INF("imx615_module_id=%d\n",imgsensor_info.module_id);
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

    LOG_INF("IMX615 open start\n");
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
            if (sensor_id == IMX615_SENSOR_ID) {
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
    LOG_INF("IMX615 open End\n");
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
    qsc_flag = 0;
    return ERROR_NONE;
} /* close */

/*write AWB gain to sensor*/
static kal_uint16 imx615_feedback_awbgain[] = {
    0x0b90, 0x00,
    0x0b91, 0x01,
    0x0b92, 0x00,
    0x0b93, 0x01,
};
static void feedback_awbgain(kal_uint32 r_gain, kal_uint32 b_gain)
{
    UINT32 r_gain_int = 0;
    UINT32 b_gain_int = 0;

    r_gain_int = r_gain / 522;
    b_gain_int = b_gain / 512;

    imx615_feedback_awbgain[1] = r_gain_int;
    imx615_feedback_awbgain[3] = (
        ((r_gain*100) / 512) - (r_gain_int * 100)) * 2;
    imx615_feedback_awbgain[5] = b_gain_int;
    imx615_feedback_awbgain[7] = (
        ((b_gain * 100) / 512) - (b_gain_int * 100)) * 2;
    imx615_table_write_cmos_sensor(imx615_feedback_awbgain,
        sizeof(imx615_feedback_awbgain)/sizeof(kal_uint16));
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
    LOG_INF("%s E\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    preview_setting();
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
#if 0
    if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
    /* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    } else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF(
            "Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",
        imgsensor.current_fps, imgsensor_info.cap1.max_framerate/10);
        imgsensor.pclk = imgsensor_info.cap2.pclk;
        imgsensor.line_length = imgsensor_info.cap2.linelength;
        imgsensor.frame_length = imgsensor_info.cap2.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    } else {
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF(
            "Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
            imgsensor.current_fps,
            imgsensor_info.cap.max_framerate / 10);
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
#else
    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
        LOG_INF(
            "Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
            imgsensor.current_fps,
            imgsensor_info.cap.max_framerate / 10);
    imgsensor.pclk = imgsensor_info.cap.pclk;
    imgsensor.line_length = imgsensor_info.cap.linelength;
    imgsensor.frame_length = imgsensor_info.cap.framelength;
    imgsensor.min_frame_length = imgsensor_info.cap.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
#endif

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
    set_mirror_flip(imgsensor.mirror);

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
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* slim_video */


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
}    /* custom1 */

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

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
}    /* custom2 */

static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    write_sensor_QSC();
    custom3_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* custom2 */

static kal_uint32 custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    imgsensor.pclk = imgsensor_info.custom4.pclk;
    imgsensor.line_length = imgsensor_info.custom4.linelength;
    imgsensor.frame_length = imgsensor_info.custom4.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom4_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* custom4 */

static kal_uint32 custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
    imgsensor.pclk = imgsensor_info.custom5.pclk;
    imgsensor.line_length = imgsensor_info.custom5.linelength;
    imgsensor.frame_length = imgsensor_info.custom5.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom5_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}

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

    sensor_resolution->SensorCustom5Width =
        imgsensor_info.custom5.grabwindow_width;
    sensor_resolution->SensorCustom5Height =
        imgsensor_info.custom5.grabwindow_height;

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
    sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
    sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
    sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
    sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
    sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;

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
    case MSDK_SCENARIO_ID_CUSTOM5:
        sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom5.mipi_data_lp2hs_settle_dc;
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
    case MSDK_SCENARIO_ID_CUSTOM5:
        custom5(image_window, sensor_config_data);
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
    case MSDK_SCENARIO_ID_CUSTOM1:
        frame_length = imgsensor_info.custom1.pclk / framerate * 10
                / imgsensor_info.custom1.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom1.framelength)
            ? (frame_length - imgsensor_info.custom1.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom1.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        break;
    case MSDK_SCENARIO_ID_CUSTOM2:
        frame_length = imgsensor_info.custom2.pclk / framerate * 10
                / imgsensor_info.custom2.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom2.framelength)
            ? (frame_length - imgsensor_info.custom2.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom2.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        break;
    case MSDK_SCENARIO_ID_CUSTOM3:
        frame_length = imgsensor_info.custom3.pclk / framerate * 10
                / imgsensor_info.custom3.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom3.framelength)
            ? (frame_length - imgsensor_info.custom3.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom3.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        break;
    case MSDK_SCENARIO_ID_CUSTOM4:
        frame_length = imgsensor_info.custom4.pclk / framerate * 10
                / imgsensor_info.custom4.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom4.framelength)
            ? (frame_length - imgsensor_info.custom4.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom4.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
        break;
    case MSDK_SCENARIO_ID_CUSTOM5:
        frame_length = imgsensor_info.custom5.pclk / framerate * 10
                / imgsensor_info.custom5.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom5.framelength)
            ? (frame_length - imgsensor_info.custom5.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom5.framelength
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
    case MSDK_SCENARIO_ID_CUSTOM5:
        *framerate = imgsensor_info.custom5.max_framerate;
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
        write_cmos_sensor_8(0x0600, 0x0000); /*No pattern*/
        write_cmos_sensor_8(0x0601, 0x0000);
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
        *(MINT32 *)(signed long)(*(feature_data + 1)) = -2500000;
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
        case MSDK_SCENARIO_ID_CUSTOM5:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom5.pclk;
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
    case MSDK_SCENARIO_ID_CUSTOM5:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom5.framelength << 16)
                                 + imgsensor_info.custom5.linelength;
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
        set_test_pattern_mode((UINT8)*feature_data, (struct SET_SENSOR_PATTERN_SOLID_COLOR *)(feature_data+1));
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
        case MSDK_SCENARIO_ID_CUSTOM3:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[7],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[8],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[9],
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
        case MSDK_SCENARIO_ID_CUSTOM1:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom1.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom2.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom3.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom4.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom5.mipi_pixel_rate;
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
        /* modify to separate 3hdr and remosaic */
        if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3) {
            /*write AWB gain to sensor*/
            feedback_awbgain((UINT32)*(feature_data_32 + 1),
                    (UINT32)*(feature_data_32 + 2));
        }
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

UINT32 IMX615_MIPI_RAW_21305_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc != NULL)
        *pfFunc = &sensor_func;
    return ERROR_NONE;
} /* IMX615_MIPI_RAW_SensorInit */
