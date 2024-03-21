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
#include "imx709mipiraw_Sensor.h"
#include "imx709_eeprom.h"
#include "imx709_seamless_switch.h"

#ifdef MAX_VC_INFO_CNT
#undef MAX_VC_INFO_CNT
#endif
#define MAX_VC_INFO_CNT 13
#define SENSOR_GET_LINELENGTH_FOR_READOUT (0x1 << 0)

/***************Modify Following Strings for Debug**********************/
#define PFX "imx709_camera_sensor"
#define LOG_1 LOG_INF("IMX709,MIPI 4LANE\n")
/****************************Modify end**************************/

#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

static kal_uint32 streaming_control(kal_bool enable);
#define MODULE_ID_OFFSET 0x0000
static kal_uint16 imx709_table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len);
static kal_uint16 imx709_burst_write_cmos_sensor(kal_uint16 *para, kal_uint32 len);
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static kal_uint16 imx709_LRC_setting[LRC_SIZE*2];
static kal_uint16 imx709_QSC_setting[QSC_SIZE*2];
static kal_uint8 qsc_flag = 0, lrc_flag = 0;
static uint8_t deviceInfo_register_value = 0;

static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = IMX709_SENSOR_ID_21305,
    .checksum_value = 0x8ac2d94a,

    .pre = { /*reg_B1_3 3264x2448@45*/
        .pclk = 864000000,
        .linelength = 7400,
        .framelength = 2592,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 418400000,
        .max_framerate = 450, /* 45fps */
    },

    .cap = { /*reg_B1_3 3264x2448@45*/
        .pclk = 864000000,
        .linelength = 7400,
        .framelength = 2592,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 418400000,
        .max_framerate = 450, /* 45fps */
    },

    .normal_video = { //reg_C7 3264x1856 @30FPS (Seamless switch with reg_I7)
        .pclk = 864000000,
        .linelength = 7400,
        .framelength = 3888,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1856,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 480800000,
        .max_framerate = 300, /* 30fps */
    },

    .hs_video = { /*reg_G-3 3264x1856@60ps*/
        .pclk = 864000000,
        .linelength = 7400,
        .framelength = 1952,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1856,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 419600000,
        .max_framerate = 600, /* 600fps */
    },

    .slim_video = { /*reg_G-2 3264x1856@60ps*/
        .pclk = 864000000,
        .linelength = 7400,
        .framelength = 1952,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1856,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 419600000,
        .max_framerate = 600, /* 600fps */
    },

    .custom1 = { /*reg H 1632x1224@15FPS */
        .pclk = 240000000,
        .linelength = 5880,
        .framelength = 2720,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1632,
        .grabwindow_height = 1224,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 120000000,
        .max_framerate = 150, /* 15fps */
    },

    .custom2 = { /*reg H1 1632x1224 @30FPS*/
        .pclk = 374400000,
        .linelength = 5880,
        .framelength = 2120,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1632,
        .grabwindow_height = 1224,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 120000000,
        .max_framerate = 300, /* 30fps */
    },

    .custom3 = { /*reg-A_5: 6528x4896@15 for remosaic*/
        .pclk =537600000,
        .linelength = 7120,
        .framelength = 5032,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 6528,
        .grabwindow_height = 4896,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 521200000,
        .max_framerate = 150,
    },

    .custom4 = {  /*reg_I7 3264x1856 @30FPS    (Dbin 2-DOL), F-DOL 1ST*/
        .pclk = 864000000,
        .linelength = 7456,
        .framelength = 3856,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1856,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 480800000,
        .max_framerate = 300, /* 30fps */
    },

    .custom5 = { /*reg B3 3264x2448@30fps*/
        .pclk = 720000000,
        .linelength = 7400,
        .framelength = 3240,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 348000000,
        .max_framerate = 300, /* 30fps */
    },

    .custom6 = { /*AON: reg E1_1 1640x928@10FPS*/
        .pclk = 372000000,
        .linelength = 5880,
        .framelength = 6320,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1640,
        .grabwindow_height = 928,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 120000000,
        .max_framerate = 100, /* 10fps */
    },

    .custom7 = {/*DFov 80 degree for thirdparty app, reg_J1 2704x2032@30fps*/
        .pclk = 864000000,
        .linelength = 7400,
        .framelength = 3888,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2704,
        .grabwindow_height = 2032,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 439200000,
        .max_framerate = 300, /* 30fps */
    },

    .custom8 = {/*DFov 80 degree for thirdparty app, reg_K1 2944x1660@30fps*/
        .pclk = 864000000,
        .linelength = 7400,
        .framelength = 3888,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2944,
        .grabwindow_height = 1660,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 439200000,
        .max_framerate = 300, /* 30fps */
    },

    .margin = 24,        /* sensor framelength & shutter margin */
    .min_shutter = 8,    /* min shutter */
    .min_gain = 64, /*1x gain*/
    .max_gain = 2048, /*32x gain*/
    .min_gain_iso = 100,
    .gain_step = 1,
    .gain_type = 0,
    .max_frame_length = 0xffff-5,
    .ae_shut_delay_frame = 0,
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,    /* isp gain delay frame for AE cycle */
    .ihdr_support = 0,    /* 1, support; 0,not support */
    .ihdr_le_firstline = 0,    /* 1,le first ; 0, se first */
    .sensor_mode_num = 13,    /*support sensor mode num*/

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
    .custom6_delay_frame = 2,
    .custom7_delay_frame = 2,
    .custom8_delay_frame = 2,

    .isp_driving_current = ISP_DRIVING_6MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2, /*only concern if it's cphy*/
    .mipi_settle_delay_mode = 0,
    //.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gb,
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
    .mclk = 24, /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
    /*.mipi_lane_num = SENSOR_MIPI_4_LANE,*/
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
    .current_fps = 450,
    .autoflicker_en = KAL_FALSE,
    .test_pattern = KAL_FALSE,
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
    .ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
    .i2c_write_id = 0x20, /* record current sensor's i2c write id */
    .extend_frame_length_en = KAL_FALSE,
    .fast_mode_on = KAL_FALSE,
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[] = {
    {6560, 4928, 0,  16, 6560, 4896, 3280, 2448,  8,   0, 3264, 2448,  0,  0, 3264, 2448}, /* Preview  reg_B1_3*/
    {6560, 4928, 0,  16, 6560, 4896, 3280, 2448,  8,   0, 3264, 2448,  0,  0, 3264, 2448}, /* capture  reg_B1_3*/
    {6560, 4928, 0, 608, 6560, 3712, 3280, 1856,  8,   0, 3264, 1856,  0,  0, 3264, 1856}, /* normal video reg_C3*/
    {6560, 4928, 0, 608, 6560, 3712, 3280, 1856,  8,   0, 3264, 1856,  0,  0, 3264, 1856}, /* hs_video  reg_G2*/
    {6560, 4928, 0, 608, 6560, 3712, 3280, 1856,  8,   0, 3264, 1856,  0,  0, 3264, 1856}, /* slim video reg_G2*/
    {6560, 4928, 0,   0, 6560, 4928, 1640, 1232,  4,   4, 1632, 1224,  0,  0, 1632, 1224}, /* custom1  reg_H*/
    {6560, 4928, 0,   0, 6560, 4928, 1640, 1232,  4,   4, 1632, 1224,  0,  0, 1632, 1224}, /* custom2  reg_H1*/
    {6560, 4928, 0,  16, 6560, 4896, 6560, 4896, 16,   0, 6528, 4896,  0,  0, 6528, 4896}, /* custom3  reg_A5*/
    {6560, 4928, 0, 608, 6560, 3712, 3280, 1856,  8,   0, 3264, 1856,  0,  0, 3264, 1856}, /* custom4  reg_I3*/
    {6560, 4928, 0,  16, 6560, 4896, 3280, 2448,  8,   0, 3264, 2448,  0,  0, 3264, 2448}, /* custom5 reg_B3*/
    {6560, 4928, 0, 608, 6560, 3712, 1640, 928 ,  0,   0, 1640, 928 ,  0,  0, 1640, 928 }, /* custom6 reg E1_1*/
    {6560, 4928, 0, 432, 6560, 4064, 3280, 2032, 288,  0, 2704, 2032,  0,  0, 2704, 2032}, /* custom7 reg J1*/
    {6560, 4928, 0, 800, 6560, 3328, 3280, 1664, 168,  2, 2944, 1660,  0,  0, 2944, 1660}, /* custom8 reg K1*/
};

//the index order of VC_STAGGER_NE/ME/SE in array identify the order they are read out in MIPI transfer
static struct SENSOR_VC_INFO2_STRUCT SENSOR_VC_INFO2[] = {
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //preivew
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 3264, 2448},
        },
        1
    },
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //capture
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 3264, 2448},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //normal video
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 3264, 1856},
        },
        1
    },
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //hs_video
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 3264, 1856},
        },
        1
    },
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //slim video
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 3264, 1856},
        },
        1
    },
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom1
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 1632, 1224},
        },
        1
    },
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom2
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 1632, 1224},
        },
        1
    },
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00,  //custom3
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 6528, 4896},
        },
        1
    },
    {
        0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom4
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 3264, 1856},
            {VC_STAGGER_ME, 0x01, 0x2b, 3264, 1856},
        },
        1
    },
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom5
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 3264, 2448},
        },
        1
    },
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom6
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 1640, 928},
        },
        1
    },
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom7
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 2704, 2032},
        },
        1
    },
    {
        0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom8
        {
            {VC_STAGGER_NE, 0x00, 0x2b, 2944, 1660},
        },
        1
    },
};
static struct SEAMLESS_SYS_DELAY seamless_sys_delays[] = {
    { MSDK_SCENARIO_ID_VIDEO_PREVIEW, MSDK_SCENARIO_ID_CUSTOM4, 1 },
    { MSDK_SCENARIO_ID_CUSTOM4, MSDK_SCENARIO_ID_VIDEO_PREVIEW, 1 },
};

static void get_vc_info_2(struct SENSOR_VC_INFO2_STRUCT *pvcinfo2, kal_uint32 scenario)
{
    switch (scenario) {
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[1],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[2],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[3],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_SLIM_VIDEO:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[4],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CUSTOM1:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[5],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CUSTOM2:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[6],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CUSTOM3:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[7],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CUSTOM4:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[8],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CUSTOM5:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[9],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CUSTOM6:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[10],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[11],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CUSTOM8:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[12],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    default:
        memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[0],
            sizeof(struct SENSOR_VC_INFO2_STRUCT));
        break;
    }
}
static void read_module_data(void){
    kal_uint16 i = 0;
    kal_uint32 vcmID = 0, lensID = 0;
    /*Read normal eeprom data*/
    gImgEepromInfo.camNormdata[IMGSENSOR_SENSOR_IDX_SUB][0] = Eeprom_1ByteDataRead(0x00, IMX709_EEPROM_SLAVE_ADDRESS);
    gImgEepromInfo.camNormdata[IMGSENSOR_SENSOR_IDX_SUB][1] = Eeprom_1ByteDataRead(0x01, IMX709_EEPROM_SLAVE_ADDRESS);
    imgsensor_info.module_id = Eeprom_1ByteDataRead(0x00, IMX709_EEPROM_SLAVE_ADDRESS);
    Oplusimgsensor_Registdeviceinfo(gImgEepromInfo.pCamModuleInfo[IMGSENSOR_SENSOR_IDX_SUB].name,
        gImgEepromInfo.pCamModuleInfo[IMGSENSOR_SENSOR_IDX_SUB].version, imgsensor_info.module_id);
    /*Read SensorId LensId and VCMId: 2~7*/
    for(i = 2; i < 8; i++) {
        gImgEepromInfo.camNormdata[IMGSENSOR_SENSOR_IDX_SUB][i] = Eeprom_1ByteDataRead(0x04+i, IMX709_EEPROM_SLAVE_ADDRESS);
    }
    /*Read QR Code from eeprom: 8~27*/
    for (i = 0; i < OPLUS_CAMERASN_LENS; i ++) {
        gImgEepromInfo.camNormdata[IMGSENSOR_SENSOR_IDX_SUB][8+i] = Eeprom_1ByteDataRead(0xB0+i, IMX709_EEPROM_SLAVE_ADDRESS);
    }
    vcmID = (Eeprom_1ByteDataRead(0xb, IMX709_EEPROM_SLAVE_ADDRESS) << 8) | Eeprom_1ByteDataRead(0xa, IMX709_EEPROM_SLAVE_ADDRESS);
    lensID = (Eeprom_1ByteDataRead(0x9, IMX709_EEPROM_SLAVE_ADDRESS) << 8) | Eeprom_1ByteDataRead(0x8, IMX709_EEPROM_SLAVE_ADDRESS);
    memcpy(&gImgEepromInfo.camNormdata[IMGSENSOR_SENSOR_IDX_SUB][40], &vcmID, 4);
    memcpy(&gImgEepromInfo.camNormdata[IMGSENSOR_SENSOR_IDX_SUB][44], &lensID, 4);
    gImgEepromInfo.i4CurSensorIdx = IMGSENSOR_SENSOR_IDX_SUB;
    gImgEepromInfo.i4CurSensorId = imgsensor_info.sensor_id;
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

static enum IMGSENSOR_RETURN write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF),
            (char)(para & 0xFF)};

    return iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static kal_uint32 get_exp_cnt_by_scenario(kal_uint32 scenario)
{
    kal_uint32 exp_cnt = 0, i = 0;
    struct SENSOR_VC_INFO2_STRUCT vcinfo2;

    get_vc_info_2(&vcinfo2, scenario);

    for (i = 0; i < MAX_VC_INFO_CNT; ++i) {
        if (vcinfo2.vc_info[i].VC_FEATURE > VC_STAGGER_MIN_NUM &&
            vcinfo2.vc_info[i].VC_FEATURE < VC_STAGGER_MAX_NUM) {
            exp_cnt++;
        }
    }

    LOG_INF("%s exp_cnt %d\n", __func__, exp_cnt);
    return max(exp_cnt, (kal_uint32)1);
}

static kal_uint32 get_cur_exp_cnt()
{
    kal_uint32 exp_cnt = 1;

    if (0x1 == (read_cmos_sensor_8(0x3170) & 0x1)) { // DOL_EN
        exp_cnt = 2;
    }

    return exp_cnt;
}

static enum IMGSENSOR_RETURN write_frame_len(kal_uint32 fll)
{
    // //write_frame_len should be called inside GRP_PARAM_HOLD (0x0104)
    // FRM_LENGTH_LINES must be multiple of 4
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint32 exp_cnt = get_cur_exp_cnt();
    imgsensor.frame_length = round_up(fll / exp_cnt, 4) * exp_cnt;

    LOG_INF("extend_frame_length_en %d\n", imgsensor.extend_frame_length_en);
    if (imgsensor.extend_frame_length_en == KAL_FALSE) {
        LOG_INF("fll %d exp_cnt %d\n", imgsensor.frame_length, exp_cnt);
        ret += write_cmos_sensor_8(0x0340, imgsensor.frame_length / exp_cnt >> 8);
        ret += write_cmos_sensor_8(0x0341, imgsensor.frame_length / exp_cnt & 0xFF);
    }

    LOG_INF("fast_mode_on %d\n", imgsensor.fast_mode_on);
    if (imgsensor.fast_mode_on == KAL_TRUE) {
        imgsensor.fast_mode_on = KAL_FALSE;
        ret += write_cmos_sensor_8(0x3010, 0x00);
    }
    return ret;
}

static void set_dummy(void)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint8 write_retry = 2;
    LOG_INF("dummyline = %d, dummypixels = %d, frame_length:%d, line_length:%d\n",
        imgsensor.dummy_line, imgsensor.dummy_pixel, imgsensor.frame_length, imgsensor.line_length);
    /* return;*/ /* for test */
    if (imgsensor.shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = imgsensor.shutter + imgsensor_info.margin;
    while(write_retry) {
        ret = write_frame_len(imgsensor.frame_length);
        if (ret == IMGSENSOR_RETURN_SUCCESS)
            break;
        write_retry--;
        mdelay(2);
    }
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

static void write_shutter(kal_uint32 shutter, kal_bool gph)
{
    kal_uint16 realtime_fps = 0;
    //shutter = round_up(shutter, 4);

    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    if (shutter < imgsensor_info.min_shutter) {
        shutter = imgsensor_info.min_shutter;
    }

    if (gph) {
        write_cmos_sensor_8(0x0104, 0x01);
    }

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10
                / imgsensor.frame_length;
        LOG_INF("autoflicker enable, realtime_fps = %d\n", realtime_fps);
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
    }

    write_frame_len(imgsensor.frame_length);

    /* Update Shutter */
    write_cmos_sensor_8(0x0350, 0x01); /* Enable auto extend */
    write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
    write_cmos_sensor_8(0x0203, shutter  & 0xFF);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x00);
    }

    LOG_INF("shutter =%d, framelength =%d read shutter h: 0x%x l: 0x%x\n",
        shutter, imgsensor.frame_length, read_cmos_sensor_8(0x0202), read_cmos_sensor_8(0x0203));
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
static void set_shutter_w_gph(kal_uint32 shutter, kal_bool gph)
{
    unsigned long flags;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    write_shutter(shutter, gph);
}

static void set_shutter(kal_uint32 shutter)
{
    LOG_INF("Fine Integ Time = %d", (read_cmos_sensor_8(0x0200) << 8) | read_cmos_sensor_8(0x0201));
    set_shutter_w_gph(shutter, KAL_TRUE);
} /* set_shutter */

/*************************************************************************
 * FUNCTION
 *    set_shutter_frame_length
 *
 * DESCRIPTION
 *    for frame & 3A sync
 *
 *************************************************************************/
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length,
                kal_bool auto_extend_en)
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

    /*  */
    //if (shutter > imgsensor.frame_length - imgsensor_info.margin)
    //    imgsensor.frame_length = shutter + imgsensor_info.margin;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    shutter = (shutter < imgsensor_info.min_shutter)
            ? imgsensor_info.min_shutter : shutter;
    shutter =
    (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
        ? (imgsensor_info.max_frame_length - imgsensor_info.margin)
        : shutter;
    write_cmos_sensor_8(0x0104, 0x01);
    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
                imgsensor.frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
        else {
            /* Extend frame length */
            write_frame_len(imgsensor.frame_length);
        }
    } else {
        /* Extend frame length */
        write_frame_len(imgsensor.frame_length);
    }

    /* Update Shutter */
    if (auto_extend_en)
        write_cmos_sensor_8(0x0350, 0x01); /* Enable auto extend */
    else
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
static kal_uint16 set_gain_w_gph(kal_uint16 gain, kal_bool gph)
{
    kal_uint16 reg_gain, max_gain = imgsensor_info.max_gain;

    if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3) {
        max_gain = 16 * BASEGAIN;//set 16x gain for full size mode.
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

    if (gph) {
        write_cmos_sensor_8(0x0104, 0x01);
    }
    write_cmos_sensor_8(0x0204, (reg_gain>>8) & 0xFF);
    write_cmos_sensor_8(0x0205, reg_gain & 0xFF);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x00);
    }

    return gain;
} /* set_gain_w_gph */

static kal_uint16 set_gain(kal_uint16 gain)
{
    return set_gain_w_gph(gain, KAL_TRUE);
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
#define I2C_BURST_BUFFER_LEN 255 /* trans# max is 255, each 3 bytes */
static kal_uint16 imx709_burst_write_cmos_sensor(
                    kal_uint16 *para, kal_uint32 len)
{
    char puSendCmd[I2C_BURST_BUFFER_LEN];
    kal_uint32 tosend, IDX;
    kal_uint16 addr = 0, addr_last = 0, data;

    tosend = 0;
    IDX = 0;
    while (len > IDX) {
        addr = para[IDX];
        if (tosend == 0)
        {
            puSendCmd[tosend++] = (char)(addr >> 8);
            puSendCmd[tosend++] = (char)(addr & 0xFF);
            data = para[IDX + 1];
            puSendCmd[tosend++] = (char)(data & 0xFF);
            IDX += 2;
            addr_last = addr;
        }
        else if (addr == addr_last + 1)
        {
            data = para[IDX + 1];
            puSendCmd[tosend++] = (char)(data & 0xFF);
            addr_last = addr;
            IDX += 2;
        }

        if ((tosend >= I2C_BURST_BUFFER_LEN)||
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

static kal_uint16 imx709_rmsc_ip_setting[] = {
    //RMSC_IP_reg_210628
    0xD400, 0x00,
    0xD401, 0x00,
    0xD402, 0x00,
    0xD403, 0x00,
    0xD404, 0x00,
    0xD405, 0x00,
    0xD406, 0x00,
    0xD407, 0x00,
    0xD408, 0x00,
    0xD409, 0x00,
    0xD40A, 0x00,
    0xD40B, 0x00,
    0xD40C, 0x00,
    0xD40D, 0x00,
    0xD40E, 0x00,
    0xD40F, 0x00,
    0xD410, 0x00,
    0xD411, 0x00,
    0xD412, 0x00,
    0xD413, 0x00,
    0xD414, 0x00,
    0xD415, 0x00,
    0xD416, 0x00,
    0xD417, 0x00,
    0x393C, 0x00,
    0x393D, 0x00,
    0x393E, 0x04,
    0x393F, 0x00,
    0xD414, 0x00,
    0xD415, 0x00,
    0xD416, 0x00,
    0xD417, 0x00,
    0xD41C, 0x00,
    0xD41D, 0x00,
    0xD41E, 0x00,
    0xD41F, 0x00,
    0xD458, 0x0C,
    0xD459, 0x86,
    0xD45A, 0x42,
    0xD45B, 0x58,
    0xD45C, 0x04,
    0xD45D, 0xB1,
    0xD45E, 0x90,
    0xD45F, 0x96,
    0xD460, 0x03,
    0xD461, 0x00,
    0xD462, 0x60,
    0xD463, 0x32,
    0xD464, 0x0A,
    0xD465, 0x81,
    0xD466, 0xE0,
    0xD467, 0x50,
    0xD468, 0x16,
    0xD469, 0x84,
    0xD46A, 0x80,
    0xD46B, 0xE0,
    0xD46C, 0x27,
    0xD46D, 0x18,
    0xD46E, 0x41,
    0xD46F, 0xB8,
    0xD470, 0x3C,
    0xD471, 0x0D,
    0xD472, 0x22,
    0xD473, 0xD8,
    0x3900, 0x19,
    0x3901, 0x01,
    0x3902, 0x93,
    0x3903, 0xF7,
    0x3904, 0x03,
    0x3905, 0x20,
    0x3906, 0xC8,
    0x3907, 0x64,
    0x3908, 0x09,
    0x3909, 0x61,
    0x390A, 0x90,
    0x390B, 0xC8,
    0x390C, 0x01,
    0x390D, 0xE0,
    0x390E, 0xC9,
    0x390F, 0xF4,
    0x3910, 0x01,
    0x3911, 0xE0,
    0x3912, 0x78,
    0x3913, 0x1E,
    0x3914, 0x03,
    0x3915, 0x29,
    0x3916, 0x62,
    0x3917, 0x58,
    0x3918, 0x00,
    0x3919, 0x1E,
    0x391A, 0x00,
    0x391B, 0x1E,
    0x391C, 0x00,
    0x391D, 0x1E,
    0x391E, 0x00,
    0x391F, 0x1E,
    0x3920, 0x01,
    0x3921, 0xF4,
    0x3922, 0x01,
    0x3923, 0xF4,
    0x3924, 0x01,
    0x3925, 0xF4,
    0x3926, 0x01,
    0x3927, 0xF4,
    0x3928, 0x3F,
    0x3929, 0xA7,
    0x392A, 0xD3,
    0x392B, 0x84,
    0x392C, 0x07,
    0x392D, 0x8F,
    0x392E, 0xCA,
    0x392F, 0xEE,
    0x3930, 0x05,
    0x3931, 0xA0,
    0x3932, 0x78,
    0x3933, 0x78,
    0x3934, 0x05,
    0x3935, 0x01,
    0x3936, 0x40,
    0x3937, 0x50,
    0xD47C, 0x01,
    0xD47D, 0x60,
    0xD47E, 0x58,
    0xD47F, 0x34,
    0xD480, 0x00,
    0xD481, 0x80,
    0xD482, 0x20,
    0xD483, 0x14,
    0xD484, 0x01,
    0xD485, 0x80,
    0xD486, 0x60,
    0xD487, 0x20,
    0xD488, 0x02,
    0xD489, 0x40,
    0xD48A, 0x40,
    0xD48B, 0x20,
    0xD48C, 0x01,
    0xD48D, 0x40,
    0xD48E, 0x18,
    0xD48F, 0x28,
    0xD490, 0x01,
    0xD491, 0x40,
    0xD492, 0x50,
    0xD493, 0x12,
    0xD494, 0x01,
    0xD495, 0x40,
    0xD496, 0x30,
    0xD497, 0x12,
    0xD498, 0x00,
    0xD499, 0xC0,
    0xD49A, 0x30,
    0xD49B, 0x12,
    0xD49C, 0x00,
    0xD49D, 0xC0,
    0xD49E, 0x30,
    0xD49F, 0x12,
    0x3938, 0x01,
    0x3939, 0x2C,
    0x393A, 0x07,
    0x393B, 0xD0,
    0x3948, 0x00,
    0x3949, 0x00,
    0x394A, 0x03,
    0x394B, 0xD7,
    0x3960, 0x00,
    0x3961, 0x01,
    0x3962, 0x00,
    0x3963, 0x36,
    0x3964, 0x00,
    0x3965, 0x01,
    0x3966, 0x00,
    0x3967, 0x36,
    0x3954, 0x00,
    0x3955, 0x02,
    0x3956, 0x00,
    0x3957, 0xCC,
    0x3958, 0x00,
    0x3959, 0x02,
    0x395A, 0x00,
    0x395B, 0xCC,
    0xD500, 0x00,
    0xD501, 0x00,
    0xD502, 0x00,
    0xD503, 0x00,
    0xD504, 0x00,
    0xD505, 0x00,
    0xD506, 0x00,
    0xD507, 0x00,
    0xD508, 0x00,
    0xD509, 0x00,
    0xD50A, 0x00,
    0xD50B, 0x00,
    0xD50C, 0x00,
    0xD50D, 0x01,
    0xD50E, 0x21,
    0xD50F, 0x00,
    0xD510, 0x24,
    0xD511, 0x21,
    0xD512, 0x00,
    0xD513, 0x00,
    0xD514, 0x21,
    0xD515, 0x00,
    0xD516, 0x00,
    0xD517, 0x01,
    0xD518, 0x00,
    0xD519, 0x01,
    0xD51A, 0x24,
    0xD51B, 0x84,
    0xD51C, 0x24,
    0xD51D, 0x88,
    0xD51E, 0x84,
    0xD51F, 0x20,
    0xD520, 0x84,
    0xD521, 0x21,
    0xD522, 0x00,
    0xD523, 0x00,
    0xD524, 0x00,
    0xD525, 0x00,
    0xD526, 0x01,
    0xD527, 0x24,
    0xD528, 0x00,
    0xD529, 0x01,
    0xD52A, 0x24,
    0xD52B, 0x21,
    0xD52C, 0x01,
    0xD52D, 0x21,
    0xD52E, 0x00,
    0xD52F, 0x00,
    0xD530, 0x00,
    0xD531, 0x00,
    0xD532, 0x00,
    0xD533, 0x00,
    0xD534, 0x00,
    0xD535, 0x00,
    0xD536, 0x00,
    0xD537, 0x00,
    0xD538, 0x00,
    0xD539, 0x00,
    0xD53A, 0x00,
    0xD53B, 0x00,
    0xD53C, 0x00,
    0xD53D, 0x00,
    0xD53E, 0x00,
    0xD53F, 0x00,
    0xD540, 0x00,
    0xD541, 0x00,
    0xD542, 0x00,
    0xD543, 0x00,
    0xD544, 0x11,
    0xD545, 0x11,
    0xD546, 0x00,
    0xD547, 0x00,
    0xD548, 0x10,
    0xD549, 0x02,
    0xD54A, 0x22,
    0xD54B, 0x00,
    0xD54C, 0x28,
    0xD54D, 0x20,
    0xD54E, 0x01,
    0xD54F, 0x44,
    0xD550, 0x00,
    0xD551, 0x14,
    0xD552, 0x41,
    0xD553, 0x00,
    0xD554, 0x11,
    0xD555, 0x10,
    0xD556, 0x02,
    0xD557, 0x22,
    0xD558, 0x00,
    0xD559, 0x00,
    0xD55A, 0x00,
    0xD55B, 0x01,
    0xD55C, 0x00,
    0xD55D, 0x00,
    0xD55E, 0x00,
    0xD55F, 0x00,
    0xD560, 0x00,
    0xD561, 0x00,
    0xD562, 0x00,
    0xD563, 0x00,
    0xD564, 0x00,
    0xD565, 0x00,
    0xD566, 0x00,
    0xD567, 0x00,
    0xD568, 0x00,
    0xD569, 0x01,
    0xD56A, 0x11,
    0xD56B, 0x00,
    0xD56C, 0x14,
    0xD56D, 0x10,
    0xD56E, 0x00,
    0xD56F, 0x22,
    0xD570, 0x00,
    0xD571, 0x02,
    0xD572, 0x20,
    0xD573, 0x00,
    0xD574, 0x00,
    0xD575, 0x00,
    0xD576, 0x01,
    0xD577, 0x11,
    0xD578, 0x00,
    0xD579, 0x00,
    0xD57A, 0x00,
    0xD57B, 0x00,
    0xD57C, 0x00,
    0xD57D, 0x00,
    0xD57E, 0x00,
    0xD57F, 0x00,
    0xD580, 0x00,
    0xD581, 0x00,
    0xD582, 0x00,
    0xD583, 0x00,
    0xD584, 0x00,
    0xD585, 0x10,
    0xD586, 0x00,
    0xD587, 0x00,
    0xD588, 0x10,
    0xD589, 0x00,
    0xD58A, 0x22,
    0xD58B, 0x00,
    0xD58C, 0x28,
    0xD58D, 0x20,
    0xD58E, 0x00,
    0xD58F, 0x44,
    0xD590, 0x00,
    0xD591, 0x14,
    0xD592, 0x40,
    0xD593, 0x00,
    0xD594, 0x10,
    0xD595, 0x00,
    0xD596, 0x02,
    0xD597, 0x20,
    0xD598, 0x00,
    0xD599, 0x00,
    0xD59A, 0x00,
    0xD59B, 0x00,
    0xD59C, 0x00,
    0xD59D, 0x00,
    0xD59E, 0x00,
    0xD59F, 0x00,
    0xD5A0, 0x00,
    0xD5A1, 0x00,
    0xD5A2, 0x00,
    0xD5A3, 0x00,
    0xD5A4, 0x00,
    0xD5A5, 0x00,
    0xD5A6, 0x00,
    0xD5A7, 0x00,
    0xD5A8, 0x00,
    0xD5A9, 0x02,
    0xD5AA, 0x20,
    0xD5AB, 0x00,
    0xD5AC, 0x28,
    0xD5AD, 0x20,
    0xD5AE, 0x00,
    0xD5AF, 0x42,
    0xD5B0, 0x00,
    0xD5B1, 0x02,
    0xD5B2, 0x40,
    0xD5B3, 0x00,
    0xD5B4, 0x00,
    0xD5B5, 0x00,
    0xD5B6, 0x00,
    0xD5B7, 0x22,
    0xD5B8, 0x00,
    0xD5B9, 0x00,
    0xD5BA, 0x00,
    0xD5BB, 0x00,
    0xD5BC, 0x00,
    0xD5BD, 0x00,
    0xD5BE, 0x00,
    0xD5BF, 0x00,
    0xD5C0, 0x00,
    0xD5C1, 0x00,
    0xD5C2, 0x00,
    0xD5C3, 0x00,
    0xD5C4, 0x00,
    0xD5C5, 0x00,
    0xD5C6, 0x00,
    0xD5C7, 0x00,
    0xD5C8, 0x00,
    0xD5C9, 0x02,
    0xD5CA, 0x20,
    0xD5CB, 0x00,
    0xD5CC, 0x28,
    0xD5CD, 0x20,
    0xD5CE, 0x00,
    0xD5CF, 0x42,
    0xD5D0, 0x00,
    0xD5D1, 0x02,
    0xD5D2, 0x40,
    0xD5D3, 0x00,
    0xD5D4, 0x00,
    0xD5D5, 0x00,
    0xD5D6, 0x00,
    0xD5D7, 0x22,
    0xD5D8, 0x00,
    0xD5D9, 0x00,
    0xD5DA, 0x00,
    0xD5DB, 0x00,
    0xD5DC, 0x00,
    0xD5DD, 0x00,
    0xD5DE, 0x00,
    0xD5DF, 0x00,
    0xD5E0, 0x00,
    0xD5E1, 0x00,
    0xD5E2, 0x00,
    0xD5E3, 0x00,
    0xD5E4, 0x01,
    0xD5E5, 0x10,
    0xD5E6, 0x00,
    0xD5E7, 0x00,
    0xD5E8, 0x00,
    0xD5E9, 0x00,
    0xD5EA, 0x20,
    0xD5EB, 0x00,
    0xD5EC, 0x14,
    0xD5ED, 0x10,
    0xD5EE, 0x00,
    0xD5EF, 0x20,
    0xD5F0, 0x00,
    0xD5F1, 0x00,
    0xD5F2, 0x20,
    0xD5F3, 0x00,
    0xD5F4, 0x11,
    0xD5F5, 0x00,
    0xD5F6, 0x00,
    0xD5F7, 0x20,
    0xD5F8, 0x00,
    0xD5F9, 0x00,
    0xD5FA, 0x00,
    0xD5FB, 0x00,
    0xD5FC, 0x00,
    0xD5FD, 0x00,
    0xD5FE, 0x00,
    0xD5FF, 0x00,
    0xD600, 0x00,
    0xD601, 0x00,
    0xD602, 0x00,
    0xD603, 0x00,
    0xD604, 0x00,
    0xD605, 0x00,
    0xD606, 0x00,
    0xD607, 0x00,
    0xD608, 0x10,
    0xD609, 0x00,
    0xD60A, 0x10,
    0xD60B, 0x00,
    0xD60C, 0x24,
    0xD60D, 0x20,
    0xD60E, 0x01,
    0xD60F, 0x20,
    0xD610, 0x00,
    0xD611, 0x10,
    0xD612, 0x21,
    0xD613, 0x00,
    0xD614, 0x00,
    0xD615, 0x00,
    0xD616, 0x00,
    0xD617, 0x10,
    0xD618, 0x00,
    0xD619, 0x00,
    0xD61A, 0x00,
    0xD61B, 0x00,
    0xD61C, 0x00,
    0xD61D, 0x00,
    0xD61E, 0x00,
    0xD61F, 0x00,
    0xD620, 0x00,
    0xD621, 0x00,
    0xD622, 0x00,
    0xD623, 0x00,
    0xD624, 0x00,
    0xD625, 0x00,
    0xD626, 0x00,
    0xD627, 0x00,
    0xD628, 0x10,
    0xD629, 0x00,
    0xD62A, 0x10,
    0xD62B, 0x00,
    0xD62C, 0x24,
    0xD62D, 0x20,
    0xD62E, 0x01,
    0xD62F, 0x02,
    0xD630, 0x00,
    0xD631, 0x12,
    0xD632, 0x01,
    0xD633, 0x00,
    0xD634, 0x00,
    0xD635, 0x00,
    0xD636, 0x00,
    0xD637, 0x10,
    0xD638, 0x00,
    0xD639, 0x00,
    0xD63A, 0x00,
    0xD63B, 0x00,
    0xD63C, 0x00,
    0xD63D, 0x00,
    0xD63E, 0x00,
    0xD63F, 0x00,
    0xD640, 0x00,
    0xD641, 0x00,
    0xD642, 0x00,
    0xD643, 0x00,
    0xD644, 0x01,
    0xD645, 0x10,
    0xD646, 0x00,
    0xD647, 0x00,
    0xD648, 0x00,
    0xD649, 0x00,
    0xD64A, 0x20,
    0xD64B, 0x00,
    0xD64C, 0x14,
    0xD64D, 0x10,
    0xD64E, 0x00,
    0xD64F, 0x02,
    0xD650, 0x00,
    0xD651, 0x02,
    0xD652, 0x00,
    0xD653, 0x00,
    0xD654, 0x11,
    0xD655, 0x00,
    0xD656, 0x00,
    0xD657, 0x20,
    0xD658, 0x00,
    0xD659, 0x00,
    0xD65A, 0x00,
    0xD65B, 0x00,
    0xD65C, 0x00,
    0xD65D, 0x00,
    0xD65E, 0x00,
    0xD65F, 0x00,
    0xD660, 0x11,
    0xD661, 0x00,
    0xD662, 0x00,
    0xD663, 0x00,
    0xD664, 0x40,
    0xD665, 0x04,
    0xD666, 0x00,
    0xD667, 0x21,
    0xD668, 0x10,
    0xD669, 0x02,
    0xD66A, 0x12,
    0xD66B, 0x21,
    0xD66C, 0x00,
    0xD66D, 0x00,
    0xD66E, 0x00,
    0xD66F, 0x00,
    0xD670, 0x01,
    0xD671, 0x00,
    0xD672, 0x00,
    0xD673, 0x00,
    0xD674, 0x04,
    0xD675, 0x12,
    0xD676, 0x21,
    0xD677, 0x20,
    0xD678, 0x11,
    0xD679, 0x12,
    0xD67A, 0x00,
    0xD67B, 0x40,
    0xD67C, 0x00,
    0xD67D, 0x00,
    0xD67E, 0x00,
    0xD67F, 0x00,
    0xD680, 0x00,
    0xD681, 0x00,
    0xD682, 0x00,
    0xD683, 0x00,
    0xD684, 0x20,
    0xD685, 0x02,
    0xD686, 0x00,
    0xD687, 0x10,
    0xD688, 0x00,
    0xD689, 0x01,
    0xD68A, 0x01,
    0xD68B, 0x10,
    0xD68C, 0x00,
    0xD68D, 0x00,
    0xD68E, 0x00,
    0xD68F, 0x00,
    0xD690, 0x00,
    0xD691, 0x00,
    0xD692, 0x00,
    0xD693, 0x00,
    0xD694, 0x02,
    0xD695, 0x01,
    0xD696, 0x10,
    0xD697, 0x10,
    0xD698, 0x00,
    0xD699, 0x01,
    0xD69A, 0x00,
    0xD69B, 0x20,
    0xD69C, 0x00,
    0xD69D, 0x00,
    0xD69E, 0x00,
    0xD69F, 0x00,
    0xD6A0, 0x00,
    0xD6A1, 0x00,
    0xD6A2, 0x00,
    0xD6A3, 0x00,
    0xD6A4, 0x20,
    0xD6A5, 0x02,
    0xD6A6, 0x00,
    0xD6A7, 0x00,
    0xD6A8, 0x00,
    0xD6A9, 0x01,
    0xD6AA, 0x01,
    0xD6AB, 0x00,
    0xD6AC, 0x00,
    0xD6AD, 0x00,
    0xD6AE, 0x00,
    0xD6AF, 0x00,
    0xD6B0, 0x01,
    0xD6B1, 0x00,
    0xD6B2, 0x00,
    0xD6B3, 0x00,
    0xD6B4, 0x04,
    0xD6B5, 0x12,
    0xD6B6, 0x21,
    0xD6B7, 0x20,
    0xD6B8, 0x01,
    0xD6B9, 0x02,
    0xD6BA, 0x00,
    0xD6BB, 0x40,
    0xD6BC, 0x00,
    0xD6BD, 0x00,
    0xD6BE, 0x00,
    0xD6BF, 0x00,
    0xD6C0, 0x00,
    0xD6C1, 0x00,
    0xD6C2, 0x00,
    0xD6C3, 0x00,
    0xD6C4, 0x40,
    0xD6C5, 0x04,
    0xD6C6, 0x00,
    0xD6C7, 0x20,
    0xD6C8, 0x20,
    0xD6C9, 0x00,
    0xD6CA, 0x00,
    0xD6CB, 0x22,
    0xD6CC, 0x00,
    0xD6CD, 0x00,
    0xD6CE, 0x00,
    0xD6CF, 0x00,
    0xD6D0, 0x04,
    0xD6D1, 0x00,
    0xD6D2, 0x00,
    0xD6D3, 0x00,
    0xD6D4, 0x08,
    0xD6D5, 0x44,
    0xD6D6, 0x02,
    0xD6D7, 0x00,
    0xD6D8, 0x02,
    0xD6D9, 0x04,
    0xD6DA, 0x00,
    0xD6DB, 0x80,
    0xD6DC, 0x00,
    0xD6DD, 0x00,
    0xD6DE, 0x00,
    0xD6DF, 0x00,
    0xD6E0, 0x00,
    0xD6E1, 0x00,
    0xD6E2, 0x00,
    0xD6E3, 0x00,
    0xD6E4, 0x40,
    0xD6E5, 0x04,
    0xD6E6, 0x00,
    0xD6E7, 0x20,
    0xD6E8, 0x00,
    0xD6E9, 0x00,
    0xD6EA, 0x00,
    0xD6EB, 0x20,
    0xD6EC, 0x00,
    0xD6ED, 0x00,
    0xD6EE, 0x00,
    0xD6EF, 0x00,
    0xD6F0, 0x00,
    0xD6F1, 0x00,
    0xD6F2, 0x00,
    0xD6F3, 0x00,
    0xD6F4, 0x04,
    0xD6F5, 0x02,
    0xD6F6, 0x00,
    0xD6F7, 0x00,
    0xD6F8, 0x00,
    0xD6F9, 0x02,
    0xD6FA, 0x00,
    0xD6FB, 0x40,
    0xD6FC, 0x00,
    0xD6FD, 0x00,
    0xD6FE, 0x00,
    0xD6FF, 0x00,
    0xD700, 0x00,
    0xD701, 0x00,
    0xD702, 0x00,
    0xD703, 0x00,
    0xD704, 0x00,
    0xD705, 0x04,
    0xD706, 0x00,
    0xD707, 0x40,
    0xD708, 0x20,
    0xD709, 0x00,
    0xD70A, 0x00,
    0xD70B, 0x20,
    0xD70C, 0x00,
    0xD70D, 0x00,
    0xD70E, 0x00,
    0xD70F, 0x00,
    0xD710, 0x02,
    0xD711, 0x00,
    0xD712, 0x00,
    0xD713, 0x00,
    0xD714, 0x00,
    0xD715, 0x02,
    0xD716, 0x00,
    0xD717, 0x00,
    0xD718, 0x00,
    0xD719, 0x04,
    0xD71A, 0x00,
    0xD71B, 0x40,
    0xD71C, 0x00,
    0xD71D, 0x00,
    0xD71E, 0x00,
    0xD71F, 0x00,
    0xD720, 0x00,
    0xD721, 0x00,
    0xD722, 0x00,
    0xD723, 0x00,
    0xD724, 0x40,
    0xD725, 0x00,
    0xD726, 0x00,
    0xD727, 0x20,
    0xD728, 0x00,
    0xD729, 0x00,
    0xD72A, 0x00,
    0xD72B, 0x42,
    0xD72C, 0x00,
    0xD72D, 0x00,
    0xD72E, 0x00,
    0xD72F, 0x00,
    0xD730, 0x00,
    0xD731, 0x00,
    0xD732, 0x00,
    0xD733, 0x00,
    0xD734, 0x04,
    0xD735, 0x24,
    0xD736, 0x00,
    0xD737, 0x00,
    0xD738, 0x00,
    0xD739, 0x02,
    0xD73A, 0x00,
    0xD73B, 0x00,
    0xD73C, 0x00,
    0xD73D, 0x00,
    0xD73E, 0x00,
    0xD73F, 0x00,
    0xD740, 0x00,
    0xD741, 0x00,
    0xD742, 0x00,
    0xD743, 0x00,
    0xD744, 0x40,
    0xD745, 0x04,
    0xD746, 0x00,
    0xD747, 0x00,
    0xD748, 0x00,
    0xD749, 0x00,
    0xD74A, 0x02,
    0xD74B, 0x02,
    0xD74C, 0x00,
    0xD74D, 0x00,
    0xD74E, 0x00,
    0xD74F, 0x00,
    0xD750, 0x00,
    0xD751, 0x00,
    0xD752, 0x00,
    0xD753, 0x00,
    0xD754, 0x04,
    0xD755, 0x20,
    0xD756, 0x20,
    0xD757, 0x00,
    0xD758, 0x00,
    0xD759, 0x00,
    0xD75A, 0x00,
    0xD75B, 0x40,
    0xD75C, 0x00,
    0xD75D, 0x00,
    0xD75E, 0x00,
    0xD75F, 0x00,
    0xD760, 0x00,
    0xD761, 0x00,
    0xD762, 0x00,
    0xD763, 0x00,
    0xD764, 0x40,
    0xD765, 0x04,
    0xD766, 0x00,
    0xD767, 0x00,
    0xD768, 0x20,
    0xD769, 0x02,
    0xD76A, 0x00,
    0xD76B, 0x00,
    0xD76C, 0x00,
    0xD76D, 0x00,
    0xD76E, 0x00,
    0xD76F, 0x00,
    0xD770, 0x02,
    0xD771, 0x00,
    0xD772, 0x00,
    0xD773, 0x00,
    0xD774, 0x04,
    0xD775, 0x00,
    0xD776, 0x00,
    0xD777, 0x20,
    0xD778, 0x00,
    0xD779, 0x00,
    0xD77A, 0x00,
    0xD77B, 0x40,
    0xD77C, 0x00,
    0xD77D, 0x00,
    0xD77E, 0x00,
    0xD77F, 0x00,
    0xD780, 0x00,
    0xD781, 0x00,
    0xD782, 0x00,
    0xD783, 0x00,
    0xD784, 0x00,
    0xD785, 0x00,
    0xD786, 0x00,
    0xD787, 0x00,
    0xD788, 0x00,
    0xD789, 0x00,
    0xD78A, 0x00,
    0xD78B, 0x00,
    0xD78C, 0x00,
    0xD78D, 0x01,
    0xD78E, 0x21,
    0xD78F, 0x00,
    0xD790, 0x24,
    0xD791, 0x21,
    0xD792, 0x00,
    0xD793, 0x00,
    0xD794, 0x21,
    0xD795, 0x00,
    0xD796, 0x00,
    0xD797, 0x01,
    0xD798, 0x00,
    0xD799, 0x01,
    0xD79A, 0x24,
    0xD79B, 0x84,
    0xD79C, 0x24,
    0xD79D, 0x88,
    0xD79E, 0x84,
    0xD79F, 0x20,
    0xD7A0, 0x84,
    0xD7A1, 0x21,
    0xD7A2, 0x00,
    0xD7A3, 0x00,
    0xD7A4, 0x00,
    0xD7A5, 0x00,
    0xD7A6, 0x01,
    0xD7A7, 0x24,
    0xD7A8, 0x00,
    0xD7A9, 0x01,
    0xD7AA, 0x24,
    0xD7AB, 0x21,
    0xD7AC, 0x01,
    0xD7AD, 0x21,
    0xD7AE, 0x00,
    0xD7AF, 0x00,
    0xD7B0, 0x00,
    0xD7B1, 0x00,
    0xD7B2, 0x00,
    0xD7B3, 0x00,
    0xD7B4, 0x00,
    0xD7B5, 0x00,
    0xD7B6, 0x00,
    0xD7B7, 0x00,
    0xD7B8, 0x00,
    0xD7B9, 0x00,
    0xD7BA, 0x00,
    0xD7BB, 0x00,
    0xD7BC, 0x00,
    0xD7BD, 0x00,
    0xD7BE, 0x00,
    0xD7BF, 0x00,
    0xD7C0, 0x00,
    0xD7C1, 0x00,
    0xD7C2, 0x40,
    0xD7C3, 0x40,
    0xD7C4, 0x00,
    0xD7C5, 0x00,
    0xD7C6, 0x04,
    0xD7C7, 0x04,
    0xD7C8, 0x40,
    0xD7C9, 0x40,
    0xD7CA, 0x00,
    0xD7CB, 0x00,
    0xD7CC, 0x00,
    0xD7CD, 0x00,
    0xD7CE, 0x00,
    0xD7CF, 0x00,
    0xD7D0, 0x24,
    0xD7D1, 0x42,
    0xD7D2, 0x40,
    0xD7D3, 0x20,
    0xD7D4, 0x00,
    0xD7D5, 0x00,
    0xD7D6, 0x02,
    0xD7D7, 0x04,
    0x3940, 0x01,
    0x3941, 0x03,
    0x3942, 0x00,
    0x3943, 0x0A,
    0x3944, 0x01,
    0x3945, 0x03,
    0x3946, 0x00,
    0x3947, 0x0A,
    0xD7E0, 0x00,
    0xD7E1, 0x00,
    0xD7E2, 0x00,
    0xD7E3, 0x01,
    0x3A1A, 0x00,
    0x3A1B, 0x28,
    0x3A1C, 0x00,
    0x3A1D, 0x28,
    0xD7E8, 0x00,
    0xD7E9, 0x00,
    0xD7EA, 0x00,
    0xD7EB, 0x01,
    0x395C, 0x01,
    0x395D, 0x45,
    0x395E, 0x01,
    0x395F, 0xEE,
    0x394C, 0x03,
    0x394D, 0xCE,
    0x394E, 0x03,
    0x394F, 0xCE,
    0x3950, 0x00,
    0x3951, 0x01,
    0x3952, 0x00,
    0x3953, 0x01,
    0xD7FC, 0x00,
    0xD7FD, 0x00,
    0xD7FE, 0x00,
    0xD7FF, 0x01,
};

static kal_uint16 imx709_init_setting[] = {
    //stream off
    0x0100,0x00,
    //External Clock Setting
    0x0136, 0x18,
    0x0137, 0x00,
    //PHY_VIF Setting
    0x3304, 0x00,
    //Register version
    0x33F0, 0x09,
    0x33F1, 0x04,
    //Signaling mode setting
    0x0111, 0x02,
    //DIG_CROP_1PIX_RSHIFT_EN 1
    0x3215, 0x01,
    //Global setting
    0x31FC, 0x03,
    0x3379, 0x00,
    0x3724, 0x03,
    0x3732, 0x05,
    0x3733, 0xDC,
    0x3736, 0x09,
    0x3737, 0xC4,
    0x373A, 0x07,
    0x373B, 0xD0,
    0x373E, 0x03,
    0x373F, 0xE8,
    0x37A2, 0x8C,
    0x37B4, 0x03,
    0x37BA, 0x0F,
    0x37BB, 0xA0,
    0x37BE, 0x0D,
    0x37BF, 0xAC,
    0x37C2, 0x04,
    0x37C3, 0xB0,
    0x37C6, 0x3E,
    0x37C7, 0x80,
    0x37CA, 0x3A,
    0x37CB, 0x20,
    0x37CE, 0x03,
    0x37CF, 0x84,
    0x37D6, 0x09,
    0x37DB, 0x0F,
    0x37DC, 0x0F,
    0x37DD, 0x01,
    0x385E, 0x05,
    0x385F, 0xDC,
    0x3862, 0x09,
    0x3863, 0xC4,
    0x3866, 0x07,
    0x3867, 0xD0,
    0x386A, 0x03,
    0x386B, 0xE8,
    0x3A1B, 0x40,
    0x3A1D, 0x40,
    0x3A1E, 0x01,
    0x3A92, 0x01,
    0x3AA2, 0x01,
    0x640C, 0xFF,
    0x640D, 0xFF,
    0x6414, 0x00,
    0x6415, 0x9D,
    0x6418, 0xFF,
    0x6419, 0xFF,
    0x6420, 0x00,
    0x6421, 0x9D,
    0x6424, 0xFF,
    0x6425, 0xFF,
    0x642C, 0x00,
    0x642D, 0x9D,
    0x6430, 0xFF,
    0x6431, 0xFF,
    0x6438, 0x02,
    0x6439, 0x24,
    0x643C, 0xFF,
    0x643D, 0xFF,
    0x6444, 0x02,
    0x6445, 0x24,
    0x6454, 0xFF,
    0x6455, 0xFF,
    0x645C, 0x00,
    0x645D, 0x9D,
    0x646C, 0xFF,
    0x646D, 0xFF,
    0x6474, 0x00,
    0x6475, 0x9D,
    0x68B0, 0x00,
    0x68B2, 0x1A,
    0x68B5, 0x01,
    0x68B6, 0x00,
    0x68B8, 0x1A,
    0x69D3, 0x27,
    0x69DF, 0x27,
    0x6FD1, 0x09,
    0x6FD3, 0x09,
    0x6FD5, 0x09,
    0x6FD7, 0x09,
    0x6FD9, 0x09,
    0x710A, 0x17,
    0x710C, 0x18,
    0x73E9, 0x53,
    0x73ED, 0x50,
    0x73F1, 0x4B,
    0x73F5, 0x4D,
    0x73F9, 0x4E,
    0x741D, 0x56,
    0x7421, 0x55,
    0x7425, 0x53,
    0x7429, 0x56,
    0x742D, 0x56,
    0x7451, 0x58,
    0x7455, 0x56,
    0x7459, 0x57,
    0x745D, 0x58,
    0x7461, 0x58,
    0x7485, 0x5A,
    0x7489, 0x58,
    0x748D, 0x58,
    0x7491, 0x5B,
    0x7495, 0x5B,
    0x74B9, 0x5D,
    0x74BB, 0x5E,
    0x74BD, 0x5F,
    0x74BF, 0x60,
    0x74E0, 0x16,
    0x74E2, 0x19,
    0x74E4, 0x16,
    0x74E6, 0x17,
    0x74E8, 0x17,
    0x74FA, 0x20,
    0x74FC, 0x24,
    0x74FE, 0x1E,
    0x7500, 0x1F,
    0x7502, 0x30,
    0x7514, 0x3F,
    0x7516, 0x1E,
    0x7518, 0x19,
    0x751A, 0x1E,
    0x751C, 0x1E,
    0x752E, 0x57,
    0x7530, 0x26,
    0x7532, 0x14,
    0x7534, 0x1D,
    0x7536, 0x1D,
    0x7548, 0x53,
    0x7549, 0x19,
    0x754A, 0x52,
    0x754B, 0x53,
    0x757E, 0x07,
    0x7590, 0x0A,
    0x75AA, 0x0A,
    0x75AC, 0x05,
    0x75C4, 0x0A,
    0x75C5, 0x0D,
    0x75C6, 0x0A,
    0x75C7, 0x0A,
    0x7656, 0x28,
    0x765A, 0x14,
    0x765E, 0x05,
    0x7660, 0x05,
    0x7678, 0x0A,
    0x767A, 0x0A,
    0x7692, 0x14,
    0x7694, 0x14,
    0x76EA, 0x01,
    0x76EC, 0x01,
    0x76EE, 0x01,
    0x76F0, 0x01,
    0x76F2, 0x01,
    0x787A, 0x05,
    0x787C, 0x05,
    0x787E, 0x05,
    0x7880, 0x05,
    0x7B52, 0x14,
    0x7B53, 0x14,
    0x7B54, 0x14,
    0x7B55, 0x14,
    0x7B56, 0x14,
    0x7B57, 0x14,
    0x7B58, 0x14,
    0x7B59, 0x14,
    0x7B5A, 0x14,
    0x7B5C, 0x05,
    0x7B5E, 0x05,
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
    0x9002, 0x0A,
    0x9003, 0x0A,
    0x9004, 0x0A,
    0x90E4, 0x01,
    0x90E7, 0x01,
    0x9200, 0x65,
    0x9201, 0xCE,
    0x9202, 0x65,
    0x9203, 0xC4,
    0x9204, 0x00,
    0x9205, 0x00,
    0x9206, 0x00,
    0x9207, 0x00,
    0x9208, 0x00,
    0x9209, 0x00,
    0x920A, 0x00,
    0x920B, 0x00,
    0x920C, 0x00,
    0x920D, 0x00,
    0x920E, 0x00,
    0x920F, 0x00,
    0x9210, 0x00,
    0x9211, 0x00,
    0x9212, 0x00,
    0x9213, 0x00,
    0x9214, 0x00,
    0x9215, 0x00,
    0x9216, 0x00,
    0x9217, 0x00,
    0x9218, 0x00,
    0x9219, 0x00,
    0x921A, 0x00,
    0x921B, 0x00,
    0x921C, 0x00,
    0x921D, 0x00,
    0x921E, 0x00,
    0x921F, 0x00,
    0x9220, 0x00,
    0x9221, 0x00,
    0x9222, 0x00,
    0x9223, 0x00,
    0x9224, 0x00,
    0x9225, 0x00,
    0x9226, 0x00,
    0x9227, 0x00,
    0x9228, 0x00,
    0x9229, 0x00,
    0x922A, 0x00,
    0x922B, 0x00,
    0x922C, 0x00,
    0x922D, 0x00,
    0x922E, 0x00,
    0x922F, 0x00,
    0x9230, 0x00,
    0x9231, 0x00,
    0x9232, 0x00,
    0x9233, 0x00,
    0x9234, 0xD7,
    0x9235, 0xE4,
    0x9236, 0xD7,
    0x9237, 0xE5,
    0x9238, 0xD7,
    0x9239, 0xE6,
    0x923A, 0xD7,
    0x923B, 0xE7,
    0x923C, 0x15,
    0x923D, 0x67,
    0xB0F9, 0x06,
    0xBC51, 0xC8,
    0xBD27, 0x28,
    0xBDC6, 0x00,
    0xBDC7, 0x00,
    0xBDC8, 0x00,
    0xBDC9, 0x00,
    0xBDCA, 0x00,
    0xBDCB, 0x00,
    0xBDCC, 0x00,
    0xBDCD, 0x00,
    0xBDCE, 0x00,
    0xBDCF, 0x00,
    0xBDD0, 0x10,
    0xBDD1, 0xDC,
    0xBDD2, 0x08,
    0xBDD3, 0x58,
    0xBDD4, 0x17,
    0xBDD5, 0xDC,
    0xBDD6, 0x0B,
    0xBDD7, 0x80,
    0xBDE0, 0x00,
    0xBDE1, 0x00,
    0xBDE2, 0x00,
    0xBDE3, 0x00,
    0xBDE4, 0x00,
    0xBDE5, 0xE0,
    0xBDE7, 0xC8,
    0xBDE8, 0x00,
    0xBDEA, 0x01,
    0xBDEB, 0xC8,
    0xC0D9, 0x12,
    0xC0DB, 0x28,
    0xC0DE, 0x28,
    0xC0E1, 0x2D,
    0xC0E3, 0x00,
    0xC0F2, 0x18,
    0xC0F3, 0x18,
    0xC546, 0x0A,
    0xC547, 0x08,
    0xC548, 0x28,
    0xC54B, 0x28,
    0xC54E, 0x3C,
    0xC551, 0x01,
    0xC55E, 0x18,
    0xC55F, 0x18,
    0xE981, 0x02,
    0xE982, 0x05,
    0xE983, 0x33,
    0xE984, 0x19,
    0xE985, 0x13,
    0xE986, 0x11,
    0xE987, 0x10,
    0xE98A, 0x01,
    0xEB00, 0x18,
    0xEB01, 0x0B,
    0xEB02, 0x04,
    //OB Setting
    0x3AC8, 0x04,
    0x3AC9, 0x74,
    0x3ACA, 0x04,
    0x3ACB, 0x74,
    0x3A90, 0x03,
    0x3A91, 0x66,
    0x3A94, 0x03,
    0x3A95, 0x66,
    0x65CE, 0x59,
    0x7020, 0x03,
    0x65C4, 0x48,
    0xBCAF, 0x01 //DOL_FRM_LENGTH_CTL
};

/*reg_B1_3 3264x2448@45fps*/
static kal_uint16 imx709_preview_setting[] = {
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x1C,
    0x0343, 0xE8,
    //Frame Length Lines Setting, need adjust base on the platform
    0x0340, 0x0A,
    0x0341, 0x20,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x10,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x13,
    0x034B, 0x2F,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x40,
    0x31D1, 0x41,
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x09,
    0x040F, 0x90,
    //Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x09,
    0x034F, 0x90,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB4,
    0x030B, 0x04,
    0x030D, 0x0C,
    0x030E, 0x08,
    0x030F, 0x2C,
    //Other Setting
    0x310C, 0x00,   //WHITE_OUT_EN 
    0x3207, 0x00,
    0x3214, 0x01,  //RGBW_RMSC_EN(RGBW Bayer Coding Re-mosaic enable): 0h disable
    0x3A00, 0x59,
    0x3A01, 0x48,
    0x3A96, 0x03,
    0x3A97, 0x34,
    0x3A9A, 0x03,
    0x3A9B, 0x34,
    0x3AA0, 0x01,
    0x3AA1, 0x8B,
    0x3AA4, 0x01,
    0x3AA5, 0x8B,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0xF0,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0xF0,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x0A,
    0x0203, 0x08,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00, //turn off PDAF
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL
    0x3170, 0x00,//DOL disable
    0x317C, 0x0A,
    0x317D, 0x0A,
    //Global setting
    0x3207, 0x00, //Turn off the QSC Setting
    //OB Setting Just for 45fps
    0x3AD0, 0x01,
    0x3AD1, 0x80,
    0x3AD2, 0x01,
    0x3AD3, 0x80,
    0x3AD8, 0x01,
    0x3AD9, 0x80,
    0x3ADC, 0x01,
    0x3ADD, 0x80,
    //IP setting
    0x3963, 0x5A,
    0x3967, 0x5A,
};

/*reg_B1_3 3264x2448@45fps*/
static kal_uint16 imx709_capture_setting[] = {
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x1C,
    0x0343, 0xE8,
    //Frame Length Lines Setting, need adjust base on the platform
    0x0340, 0x0A,
    0x0341, 0x20,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x10,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x13,
    0x034B, 0x2F,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x40,
    0x31D1, 0x41,
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x09,
    0x040F, 0x90,
    //Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x09,
    0x034F, 0x90,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB4,
    0x030B, 0x04,
    0x030D, 0x0C,
    0x030E, 0x08,
    0x030F, 0x2C,
    //Other Setting
    0x310C, 0x00,   //WHITE_OUT_EN 
    0x3207, 0x00,
    0x3214, 0x01,  //RGBW_RMSC_EN(RGBW Bayer Coding Re-mosaic enable): 0h disable
    0x3A00, 0x59,
    0x3A01, 0x48,
    0x3A96, 0x03,
    0x3A97, 0x34,
    0x3A9A, 0x03,
    0x3A9B, 0x34,
    0x3AA0, 0x01,
    0x3AA1, 0x8B,
    0x3AA4, 0x01,
    0x3AA5, 0x8B,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0xF0,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0xF0,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x0A,
    0x0203, 0x08,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00, //turn off PDAF
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL
    0x3170, 0x00,//DOL disable
    0x317C, 0x0A,
    0x317D, 0x0A,
    //Global setting
    0x3207, 0x00, //Turn off the QSC Setting
    //OB Setting Just for 45fps
    0x3AD0, 0x01,
    0x3AD1, 0x80,
    0x3AD2, 0x01,
    0x3AD3, 0x80,
    0x3AD8, 0x01,
    0x3AD9, 0x80,
    0x3ADC, 0x01,
    0x3ADD, 0x80,
    //IP setting
    0x3963, 0x5A,
    0x3967, 0x5A,
};

//reg_C7 3264x1856 @30FPS (Seamless switch with reg_I7)
static kal_uint16 imx709_normal_video_setting[] = {
//H: 3264
//V: 1856
// Address value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
//Line Length PCK Setting
    0x0342, 0x1C,
    0x0343, 0xE8,
//Frame Length Lines Setting
    0x0340, 0x0F,
    0x0341, 0x30,
//ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x60,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0xDF,
//Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x40,
    0x31D1, 0x41,
//Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x07,
    0x040F, 0x40,
//Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x07,
    0x034F, 0x40,
//Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB4,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x04,
    0x030F, 0xB2,
//Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x59,
    0x3A01, 0x48,
    0x3A96, 0x03,
    0x3A97, 0x34,
    0x3A9A, 0x03,
    0x3A9B, 0x34,
    0x3AA0, 0x01,
    0x3AA1, 0x8B,
    0x3AA4, 0x01,
    0x3AA5, 0x8B,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0xF0,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0xF0,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
//Integration Setting
    0x0202, 0x0F,
    0x0203, 0x18,
    0x0224, 0x01,
    0x0225, 0xF4,
//Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
//PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00,
    0x347A, 0x03,
    0x347B, 0x2C,
//DOL Setting
    0x3170, 0x00,
    0x317C, 0x0A,
    0x317D, 0x0A,
    //Global setting
    0x3207, 0x00, //Turn off the QSC Setting
    //IP setting
    0x3963, 0x0A,
    0x3967, 0x0A,
};

/*reg_G-3 3264x1856@60ps*/
static kal_uint16 imx709_hs_video_setting[] = {
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x1C,
    0x0343, 0xE8,
    //Frame Length Lines Setting
    0x0340, 0x07,
    0x0341, 0xA0,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x60,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0xDF,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x40,
    0x31D1, 0x41,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xD0,
    0x040E, 0x07,
    0x040F, 0x40,
    //Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xD0,
    0x034E, 0x07,
    0x034F, 0x40,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB4,
    0x030B, 0x04,
    0x030D, 0x0C,
    0x030E, 0x08,
    0x030F, 0x32,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x59,
    0x3A01, 0x48,
    0x3A96, 0x03,
    0x3A97, 0x34,
    0x3A9A, 0x03,
    0x3A9B, 0x34,
    0x3AA0, 0x01,
    0x3AA1, 0x8B,
    0x3AA4, 0x01,
    0x3AA5, 0x8B,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0xF0,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0xF0,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x07,
    0x0203, 0x88,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00, //turn off PDAF
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL
    0x3170, 0x00, //DOL disable
    0x317C, 0x0A,
    0x317D, 0x0A,
    //Global setting
    0x3207, 0x00, //Turn off the QSC Setting
    //IP setting
    0x3963, 0x36,
    0x3967, 0x36,
};

/*reg_G-3 3264x1856@60ps*/
static kal_uint16 imx709_slim_video_setting[] = {
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x1C,
    0x0343, 0xE8,
    //Frame Length Lines Setting
    0x0340, 0x07,
    0x0341, 0xA0,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x60,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0xDF,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x40,
    0x31D1, 0x41,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xD0,
    0x040E, 0x07,
    0x040F, 0x40,
    //Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xD0,
    0x034E, 0x07,
    0x034F, 0x40,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB4,
    0x030B, 0x04,
    0x030D, 0x0C,
    0x030E, 0x08,
    0x030F, 0x32,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x59,
    0x3A01, 0x48,
    0x3A96, 0x03,
    0x3A97, 0x34,
    0x3A9A, 0x03,
    0x3A9B, 0x34,
    0x3AA0, 0x01,
    0x3AA1, 0x8B,
    0x3AA4, 0x01,
    0x3AA5, 0x8B,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0xF0,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0xF0,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x07,
    0x0203, 0x88,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00, //turn off PDAF
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL
    0x3170, 0x00, //DOL disable
    0x317C, 0x0A,
    0x317D, 0x0A,
    //Global setting
    0x3207, 0x00, //Turn off the QSC Setting
    //IP setting
    0x3963, 0x36,
    0x3967, 0x36,
};

/*reg H 1632x1224@15FPS */
static kal_uint16 imx709_custom1_setting[] = {
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x16,
    0x0343, 0xF8,
    //Frame Length Lines Setting
    0x0340, 0x0A,
    0x0341, 0xA0,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x13,
    0x034B, 0x3F,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x24,
    0x0902, 0x00,
    0x3148, 0x04,
    0x31D0, 0x42,
    0x31D1, 0x43,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x04,
    0x040A, 0x00,
    0x040B, 0x04,
    0x040C, 0x06,
    0x040D, 0x60,
    0x040E, 0x04,
    0x040F, 0xC8,
    //Output Size Setting
    0x034C, 0x06,
    0x034D, 0x60,
    0x034E, 0x04,
    0x034F, 0xC8,
    //Clock Setting
    0x0303, 0x04,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x64,
    0x030B, 0x08,
    0x030D, 0x0C,
    0x030E, 0x04,
    0x030F, 0xB0,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A96, 0x02,
    0x3A97, 0xC2,
    0x3A9A, 0x02,
    0x3A9B, 0xC2,
    0x3AA0, 0x00,
    0x3AA1, 0x96,
    0x3AA4, 0x00,
    0x3AA5, 0x96,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0x64,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0x64,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x0A,
    0x0203, 0x88,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00, //turn off PDAF
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL
    0x3170, 0x00, //DOL disable
    0x317C, 0x0A,
    0x317D, 0x0A,
    //Global setting
    0x3207, 0x00, //Turn off the QSC Setting
    //IP setting
    0x3963, 0x36,
    0x3967, 0x36,
};

/*reg H1 1632x1224 @30FPS*/
static kal_uint16 imx709_custom2_setting[] = {
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x16,
    0x0343, 0xF8,
    //Frame Length Lines Setting
    0x0340, 0x08,
    0x0341, 0x48,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x13,
    0x034B, 0x3F,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x24,
    0x0902, 0x00,
    0x3148, 0x04,
    0x31D0, 0x42,
    0x31D1, 0x43,
    0x0408, 0x00,
    0x0409, 0x04,
    0x040A, 0x00,
    0x040B, 0x04,
    0x040C, 0x06,
    0x040D, 0x60,
    0x040E, 0x04,
    0x040F, 0xC8,
    //Output Size Setting
    0x034C, 0x06,
    0x034D, 0x60,
    0x034E, 0x04,
    0x034F, 0xC8,
    //Clock Setting
    0x0303, 0x04,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x9C,
    0x030B, 0x08,
    0x030D, 0x0C,
    0x030E, 0x04,
    0x030F, 0xB0,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A96, 0x02,
    0x3A97, 0xC2,
    0x3A9A, 0x02,
    0x3A9B, 0xC2,
    0x3AA0, 0x00,
    0x3AA1, 0x96,
    0x3AA4, 0x00,
    0x3AA5, 0x96,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0x64,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0x64,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x08,
    0x0203, 0x30,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00,//turn off PDAF
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL
    0x3170, 0x00, //DOL disable
    0x317C, 0x0A,
    0x317D, 0x0A,
    //Global setting
    0x3207, 0x00, //Turn off the QSC Setting
    //IP setting
    0x3963, 0x36,
    0x3967, 0x36,
};

/*reg-A_5: 6528x4896@15 for remosaic*/
static kal_uint16 imx709_custom3_setting[] = {
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x1B,
    0x0343, 0xD0,
    //Frame Length Lines Setting
    0x0340, 0x13,
    0x0341, 0xA8,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x10,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x13,
    0x034B, 0x2F,
    //Mode Setting
    0x0900, 0x00,
    0x0901, 0x11,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x4B,
    0x31D1, 0x4B,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x10,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x19,
    0x040D, 0x80,
    0x040E, 0x13,
    0x040F, 0x20,
    //Output Size Setting
    0x034C, 0x19,
    0x034D, 0x80,
    0x034E, 0x13,
    0x034F, 0x20,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x70,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x05,
    0x030F, 0x17,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x01,
    0x3214, 0x01,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A96, 0x02,
    0x3A97, 0xF8,
    0x3A9A, 0x02,
    0x3A9B, 0xF8,
    0x3AA0, 0x03,
    0x3AA1, 0x70,
    0x3AA4, 0x03,
    0x3AA5, 0x70,
    0x3AB0, 0x00,
    0x3AB1, 0xDC,
    0x3AB2, 0x00,
    0x3AB3, 0xDC,
    0x3AB4, 0x00,
    0x3AB5, 0xDC,
    0x3AB6, 0x00,
    0x3AB7, 0xDC,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x13,
    0x0203, 0x90,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00, //turn off PDAF
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL
    0x3170, 0x00, //DOL disable
    0x317C, 0x0A,
    0x317D, 0x0A,
    //Global setting
    0x3207, 0x01, //Turn on the QSC Setting
    //IP setting
    0x3963, 0x36,
    0x3967, 0x36,
};

/*reg_I7 3264x1856 @30FPS    (Dbin 2-DOL), F-DOL 1ST    */
static kal_uint16 imx709_custom4_setting[] = {
//reg_I7
//3264x1856 @30FPS    (Dbin 2-DOL), F-DOL 1ST
//H: 3264
//V: 1856
//MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
//Line Length PCK Setting
    0x0342, 0x1D,
    0x0343, 0x20,
//Frame Length Lines Setting
    0x0340, 0x07,
    0x0341, 0x88,
//ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x60,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0xDF,
//Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x40,
    0x31D1, 0x41,
//Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x07,
    0x040F, 0x40,
//Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x07,
    0x034F, 0x40,
//Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB4,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x04,
    0x030F, 0xB2,
//Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A96, 0x03,
    0x3A97, 0x88,
    0x3A9A, 0x03,
    0x3A9B, 0x88,
    0x3AA0, 0x01,
    0x3AA1, 0xAC,
    0x3AA4, 0x01,
    0x3AA5, 0xAC,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0xF0,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0xF0,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
//Integration Setting
    0x0202, 0x06,
    0x0203, 0x80,
    0x0224, 0x00,
    0x0225, 0xD0,
//Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
//PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00,
    0x347A, 0x03,
    0x347B, 0x2C,
//DOL Setting
    0x3170, 0x01,
    0x317C, 0x0A,
    0x317D, 0x0A,
//Global setting
    0x3207, 0x00, //Turn on the QSC Setting
    //IP setting
    0x3963, 0x0A,
    0x3967, 0x0A,
};

/*reg B3 3264x2448@30fps*/
static kal_uint16 imx709_custom5_setting[] = {
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x1C,
    0x0343, 0xE8,
    //Frame Length Lines Setting, need adjust base on the platform
    0x0340, 0x0C,
    0x0341, 0xA8,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x10,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x13,
    0x034B, 0x2F,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x40,
    0x31D1, 0x41,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x09,
    0x040F, 0x90,
    //Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x09,
    0x034F, 0x90,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x96,
    0x030B, 0x04,
    0x030D, 0x0C,
    0x030E, 0x06,
    0x030F, 0xCC,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x59,
    0x3A01, 0x48,
    0x3A96, 0x03,
    0x3A97, 0x34,
    0x3A9A, 0x03,
    0x3A9B, 0x34,
    0x3AA0, 0x01,
    0x3AA1, 0x8B,
    0x3AA4, 0x01,
    0x3AA5, 0x8B,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0xF0,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0xF0,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x0C,
    0x0203, 0x90,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00, //turn off PDAF
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL Setting
    0x3170, 0x00, //DOL disable
    0x317C, 0x0A,
    0x317D, 0x0A,
    //Global setting
    0x3207, 0x00, //Turn off the QSC Setting
    //IP setting
    0x3963, 0x36,
    0x3967, 0x36,
};

/*AON: reg E1_1 1640x928@10fps*/
static kal_uint16 imx709_custom6_setting[] = {
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x16,
    0x0343, 0xF8,
    //Frame Length Lines Setting
    0x0340, 0x18,
    0x0341, 0xB0,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x60,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0xDF,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x24,
    0x0902, 0x00,
    0x3148, 0x04,
    0x31D0, 0x42,
    0x31D1, 0x43,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x06,
    0x040D, 0x68,
    0x040E, 0x03,
    0x040F, 0xA0,
    //Output Size Setting
    0x034C, 0x06,
    0x034D, 0x68,
    0x034E, 0x03,
    0x034F, 0xA0,
    //Clock Setting
    0x0303, 0x04,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x9B,
    0x030B, 0x08,
    0x030D, 0x0C,
    0x030E, 0x04,
    0x030F, 0xB0,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A96, 0x02,
    0x3A97, 0xC2,
    0x3A9A, 0x02,
    0x3A9B, 0xC2,
    0x3AA0, 0x00,
    0x3AA1, 0x96,
    0x3AA4, 0x00,
    0x3AA5, 0x96,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0x64,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0x64,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x18,
    0x0203, 0x98,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00, //turn off PDAF
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL
    0x3170, 0x00,//DOL disable
    0x317C, 0x0A,
    0x317D, 0x0A,
    //Global setting
    0x3207, 0x00, //Turn off the QSC Setting
    //IP setting
    0x3963, 0x36,
    0x3967, 0x36,
};

/*DFov 80 degree for thirdparty app, reg_J1 2704x2032@30fps*/
static kal_uint16 imx709_custom7_setting[] = {
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x1C,
    0x0343, 0xE8,
    //Frame Length Lines Setting, need adjust base on the platform
    0x0340, 0x0F,
    0x0341, 0x30,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x01,
    0x0347, 0xB0,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x11,
    0x034B, 0x8F,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x40,
    0x31D1, 0x41,
    //Digital Crop & Scaling
    0x0408, 0x01,
    0x0409, 0x20,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0A,
    0x040D, 0x90,
    0x040E, 0x07,
    0x040F, 0xF0,
    //Output Size Setting
    0x034C, 0x0A,
    0x034D, 0x90,
    0x034E, 0x07,
    0x034F, 0xF0,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB4,
    0x030B, 0x04,
    0x030D, 0x0C,
    0x030E, 0x08,
    0x030F, 0x94,
    //Other Setting
    0x310C, 0x00,   //WHITE_OUT_EN 
    0x3207, 0x00,
    0x3214, 0x01,  //RGBW_RMSC_EN(RGBW Bayer Coding Re-mosaic enable): 0h disable
    0x3A00, 0x59,
    0x3A01, 0x48,
    0x3A96, 0x03,
    0x3A97, 0x34,
    0x3A9A, 0x03,
    0x3A9B, 0x34,
    0x3AA0, 0x01,
    0x3AA1, 0x8B,
    0x3AA4, 0x01,
    0x3AA5, 0x8B,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0xF0,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0xF0,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x0F,
    0x0203, 0x18,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00, //turn off PDAF
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL
    0x3170, 0x00,//DOL disable
    0x317C, 0x0A,
    0x317D, 0x0A,
    //Global setting
    0x3207, 0x00, //Turn off the QSC Setting
    //IP setting
    0x3963, 0x36,
    0x3967, 0x36,
};

/*DFov 80 degree for thirdparty app, reg_K1 2944x1660@30fps*/
static kal_uint16 imx709_custom8_setting[] = {
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x1C,
    0x0343, 0xE8,
    //Frame Length Lines Setting
    0x0340, 0x0F,
    0x0341, 0x30,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x03,
    0x0347, 0x20,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0x1F,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x40,
    0x31D1, 0x41,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0xA8,
    0x040A, 0x00,
    0x040B, 0x02,
    0x040C, 0x0B,
    0x040D, 0x80,
    0x040E, 0x06,
    0x040F, 0x7C,
    //Output Size Setting
    0x034C, 0x0B,
    0x034D, 0x80,
    0x034E, 0x06,
    0x034F, 0x7C,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB4,
    0x030B, 0x04,
    0x030D, 0x0C,
    0x030E, 0x08,
    0x030F, 0x94,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x59,
    0x3A01, 0x48,
    0x3A96, 0x03,
    0x3A97, 0x34,
    0x3A9A, 0x03,
    0x3A9B, 0x34,
    0x3AA0, 0x01,
    0x3AA1, 0x8B,
    0x3AA4, 0x01,
    0x3AA5, 0x8B,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0xF0,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0xF0,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x0F,
    0x0203, 0x18,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00,
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL Setting
    0x3170, 0x00,
    0x317C, 0x0A,
    0x317D, 0x0A,
    //Global setting
    0x3207, 0x00, //Turn off the QSC Setting
    //IP setting
    0x3963, 0x36,
    0x3967, 0x36,
};

/*static void extend_frame_length(kal_uint32 extended_fl)
{
    imgsensor.frame_length = extended_fl;

    write_cmos_sensor_8(0x0104, 0x01);
    write_frame_len(imgsensor.frame_length, get_cur_exp_cnt());
    write_cmos_sensor_8(0x0104, 0x00);
    imgsensor.extend_frame_length_en = KAL_TRUE;

    LOG_INF("new frame len = %d", imgsensor.frame_length);
}*/

static void sensor_init(void)
{
    LOG_INF("sensor_init start\n");
    imx709_table_write_cmos_sensor(imx709_init_setting,
        sizeof(imx709_init_setting)/sizeof(kal_uint16));
    imx709_table_write_cmos_sensor(imx709_rmsc_ip_setting,
        sizeof(imx709_rmsc_ip_setting)/sizeof(kal_uint16));
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

static void custom1_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx709_table_write_cmos_sensor(imx709_custom1_setting,
        sizeof(imx709_custom1_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom2_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx709_table_write_cmos_sensor(imx709_custom2_setting,
        sizeof(imx709_custom2_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom3_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx709_table_write_cmos_sensor(imx709_custom3_setting,
        sizeof(imx709_custom3_setting)/sizeof(kal_uint16));
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
    imx709_table_write_cmos_sensor(imx709_custom4_setting,
        sizeof(imx709_custom4_setting)/sizeof(kal_uint16));
     LOG_INF("%s end\n", __func__);
}

static void custom5_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx709_table_write_cmos_sensor(imx709_custom5_setting,
        sizeof(imx709_custom5_setting)/sizeof(kal_uint16));
     LOG_INF("%s end\n", __func__);
}

static void custom6_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx709_table_write_cmos_sensor(imx709_custom6_setting,
        sizeof(imx709_custom6_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom7_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx709_table_write_cmos_sensor(imx709_custom7_setting,
        sizeof(imx709_custom7_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom8_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx709_table_write_cmos_sensor(imx709_custom8_setting,
        sizeof(imx709_custom8_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void hdr_write_double_shutter_w_gph(kal_uint16 le, kal_uint16 se, kal_bool gph)
{
    kal_uint16 realtime_fps = 0;
    kal_uint16 exposure_cnt = 0;

    if (le)
        exposure_cnt++;
    if (se) {
        exposure_cnt++;
    } else {
        se = imgsensor_info.min_shutter;
        exposure_cnt++;
    }

    le = (kal_uint16)max(imgsensor_info.min_shutter * exposure_cnt, (kal_uint32)le);
    se = (kal_uint16)max(imgsensor_info.min_shutter * exposure_cnt, (kal_uint32)se);
    if(le < se) {
        le = se;
    }

    if (le) {
        le = round_up(le/exposure_cnt, 4);
    }
    if (se) {
        se = round_up(se/exposure_cnt, 4);
    }

    spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length = max((kal_uint32)((le + se + (exposure_cnt * imgsensor_info.margin)) * exposure_cnt),
                                imgsensor.min_frame_length);
    imgsensor.frame_length = min(imgsensor.frame_length, imgsensor_info.max_frame_length);
    spin_unlock(&imgsensor_drv_lock);

    LOG_INF("E! le:0x%x, se:0x%x autoflicker_en %d frame_length %d\n",
        le, se, imgsensor.autoflicker_en, imgsensor.frame_length);

    if (gph) {
        write_cmos_sensor_8(0x0104, 0x01);
    }

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
        else {
            write_frame_len(imgsensor.frame_length);
        }
    } else {
        write_frame_len(imgsensor.frame_length);
    }

    // Long exposure
    write_cmos_sensor_8(0x0202, (le >> 8) & 0xFF);
    write_cmos_sensor_8(0x0203, le & 0xFF);

    // Short exposure
    write_cmos_sensor_8(0x0224, (se >> 8) & 0xFF);
    write_cmos_sensor_8(0x0225, se & 0xFF);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x00);
    }
    LOG_INF("L! le:0x%x, se:0x%x\n", le, se);
#if 0
    //read back
    le_b = ((read_cmos_sensor_8(0x0202) & 0xff) << 8) |  (read_cmos_sensor_8(0x0203) & 0xff);
    se_b = ((read_cmos_sensor_8(0x0224) & 0xff) << 8) |  (read_cmos_sensor_8(0x0225) & 0xff);
    LOG_INF("read from sensor le: 0x%x(%d), se: 0x%x(%d)", le_b, le_b, se_b, se_b);
#endif
}

static void hdr_write_double_shutter(kal_uint16 le, kal_uint16 se)
{
    LOG_INF("Fine Integ Time = %d", (read_cmos_sensor_8(0x0200) << 8) | read_cmos_sensor_8(0x0201));
    hdr_write_double_shutter_w_gph(le, se, KAL_TRUE);
}

static void hdr_write_double_gain_w_gph(kal_uint16 lgain, kal_uint16 sgain, kal_bool gph)
{
    kal_uint16 reg_lg, reg_sg;
    //kal_uint16 lg_b = 0, sg_b = 0;

    if (lgain < BASEGAIN || lgain > 32 * BASEGAIN) {
        LOG_INF("Error lgain setting");
        if (lgain < BASEGAIN)
            lgain = BASEGAIN;
        else if (lgain > 32 * BASEGAIN)
            lgain = 32 * BASEGAIN;
    }
    if (sgain < BASEGAIN || sgain > 32 * BASEGAIN) {
        LOG_INF("Error sgain setting");
        if (sgain < BASEGAIN)
            sgain = BASEGAIN;
        else if (sgain > 32 * BASEGAIN)
            sgain = 32 * BASEGAIN;
    }

    reg_lg = gain2reg(lgain);
    reg_sg = gain2reg(sgain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_lg;
    spin_unlock(&imgsensor_drv_lock);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x01);
    }
    // Long Gian
    write_cmos_sensor_8(0x0204, (reg_lg>>8) & 0xFF);
    write_cmos_sensor_8(0x0205, reg_lg & 0xFF);

    // Short Gian
    write_cmos_sensor_8(0x0216, (reg_sg>>8) & 0xFF);
    write_cmos_sensor_8(0x0217, reg_sg & 0xFF);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x00);
    }
    LOG_INF("lgain:%d, reg_lg:0x%x, sgain:%d, reg_sg:0x%x\n",
        lgain, reg_lg, sgain, reg_sg);
#if 0
    //read back
    lg_b = ((read_cmos_sensor_8(0x0204) & 0xff) << 8) |  (read_cmos_sensor_8(0x0205) & 0xff);
    sg_b = ((read_cmos_sensor_8(0x0216) & 0xff) << 8) |  (read_cmos_sensor_8(0x0217) & 0xff);
    LOG_INF("read from sensor lg: 0x%x(%d), sg: 0x%x(%d)", lg_b, lg_b, sg_b, sg_b);
#endif
}
static void hdr_write_double_gain(kal_uint16 lgain, kal_uint16 sgain)
{
    hdr_write_double_gain_w_gph(lgain, sgain, KAL_TRUE);
}

static void write_sensor_QSC(void)
{
    kal_uint8 qsc_is_valid = Eeprom_1ByteDataRead(OTP_QSC_OFFSET + QSC_SIZE, IMX709_EEPROM_SLAVE_ADDRESS);
    LOG_INF("%s start write_sensor_QSC, qsc_is_valid:%d, qsc_flag:%d\n", __func__, qsc_is_valid, qsc_flag);
    if (qsc_is_valid == 1 && !qsc_flag) {
        imx709_burst_write_cmos_sensor(imx709_QSC_setting,
            sizeof(imx709_QSC_setting) / sizeof(kal_uint16));
        qsc_flag = 1;
    }
    LOG_INF("%s end\n", __func__);

}

static void write_sensor_lrc(void)
{
    kal_uint8 lrc_is_valid = Eeprom_1ByteDataRead(OTP_LRC_OFFSET + LRC_SIZE, IMX709_EEPROM_SLAVE_ADDRESS);
    LOG_INF("%s start write_sensor_LRC, lrc_is_valid:%d, lrc_flag:%d", __func__, lrc_is_valid, lrc_flag);
    if (lrc_is_valid == 1 && !lrc_flag) {
        imx709_table_write_cmos_sensor(imx709_LRC_setting,
            sizeof(imx709_LRC_setting) / sizeof(kal_uint16));
        lrc_flag = 1;
    }
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
    static bool first_read = KAL_TRUE;
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
            if (*sensor_id == IMX709_SENSOR_ID) {
                *sensor_id = imgsensor_info.sensor_id;
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
                    imgsensor.i2c_write_id, *sensor_id);
                if(first_read){
                    read_module_data();
                    read_imx709_21305_LRC(imx709_LRC_setting);
                    read_imx709_21305_QSC(imx709_QSC_setting);
                    first_read = KAL_FALSE;
                }
                if(deviceInfo_register_value == 0x00){
                    Eeprom_DataInit(IMGSENSOR_SENSOR_IDX_SUB, imgsensor_info.sensor_id);
                    deviceInfo_register_value = 0x01;
                }
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
    kal_uint32 sensor_id = 0;

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
            if (sensor_id == IMX709_SENSOR_ID) {
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
    if(oplus_is_system_camera(OPLUS_CHECK_IS_SYSTEM_CAM)){
        write_sensor_QSC();
    }

    write_sensor_lrc();
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
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    imgsensor.extend_frame_length_en = KAL_FALSE;
    imgsensor.fast_mode_on = KAL_FALSE;
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
    qsc_flag = 0;
    lrc_flag = 0;
    return ERROR_NONE;
} /* close */

/*write AWB gain to sensor*/
static kal_uint16 imx709_feedback_awbgain[] = {
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

    imx709_feedback_awbgain[1] = r_gain_int;
    imx709_feedback_awbgain[3] = (
        ((r_gain*100) / 512) - (r_gain_int * 100)) * 2;
    imx709_feedback_awbgain[5] = b_gain_int;
    imx709_feedback_awbgain[7] = (
        ((b_gain * 100) / 512) - (b_gain_int * 100)) * 2;
    imx709_table_write_cmos_sensor(imx709_feedback_awbgain,
        sizeof(imx709_feedback_awbgain)/sizeof(kal_uint16));
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
    custom3_setting();
    //write_sensor_QSC();
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

static kal_uint32 custom6(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM6;
    imgsensor.pclk = imgsensor_info.custom6.pclk;
    imgsensor.line_length = imgsensor_info.custom6.linelength;
    imgsensor.frame_length = imgsensor_info.custom6.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom6.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom6_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* custom6 */

static kal_uint32 custom7(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM6;
    imgsensor.pclk = imgsensor_info.custom7.pclk;
    imgsensor.line_length = imgsensor_info.custom7.linelength;
    imgsensor.frame_length = imgsensor_info.custom7.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom7.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom7_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}/* custom7 */

static kal_uint32 custom8(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM6;
    imgsensor.pclk = imgsensor_info.custom8.pclk;
    imgsensor.line_length = imgsensor_info.custom8.linelength;
    imgsensor.frame_length = imgsensor_info.custom8.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom8.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom8_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}/* custom8 */

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

    sensor_resolution->SensorCustom6Width =
        imgsensor_info.custom6.grabwindow_width;
    sensor_resolution->SensorCustom6Height =
        imgsensor_info.custom6.grabwindow_height;

    sensor_resolution->SensorCustom7Width =
        imgsensor_info.custom7.grabwindow_width;
    sensor_resolution->SensorCustom7Height =
        imgsensor_info.custom7.grabwindow_height;

    sensor_resolution->SensorCustom8Width =
        imgsensor_info.custom8.grabwindow_width;
    sensor_resolution->SensorCustom8Height =
        imgsensor_info.custom8.grabwindow_height;
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
    sensor_info->Custom6DelayFrame = imgsensor_info.custom6_delay_frame;
    sensor_info->Custom7DelayFrame = imgsensor_info.custom7_delay_frame;
    sensor_info->Custom8DelayFrame = imgsensor_info.custom8_delay_frame;

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
    sensor_info->HDR_Support = HDR_SUPPORT_STAGGER_FDOL;
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
    case MSDK_SCENARIO_ID_CUSTOM6:
        sensor_info->SensorGrabStartX = imgsensor_info.custom6.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom6.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom6.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        sensor_info->SensorGrabStartX = imgsensor_info.custom7.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom7.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom7.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM8:
        sensor_info->SensorGrabStartX = imgsensor_info.custom8.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom8.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom8.mipi_data_lp2hs_settle_dc;
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

#define FMC_GPH_START \
do { \
        write_cmos_sensor_8(0x0104, 0x01); \
        write_cmos_sensor_8(0x3010, 0x02); \
} while (0)

#define FMC_GPH_END \
do { \
        write_cmos_sensor_8(0x0104, 0x00); \
} while (0)

enum {
    SHUTTER_NE_FRM_1 = 0,
    GAIN_NE_FRM_1,
    FRAME_LEN_NE_FRM_1,
    HDR_TYPE_FRM_1,
    SHUTTER_NE_FRM_2,
    GAIN_NE_FRM_2,
    FRAME_LEN_NE_FRM_2,
    HDR_TYPE_FRM_2,
    SHUTTER_SE_FRM_1,
    GAIN_SE_FRM_1,
    SHUTTER_SE_FRM_2,
    GAIN_SE_FRM_2,
    SHUTTER_ME_FRM_1,
    GAIN_ME_FRM_1,
    SHUTTER_ME_FRM_2,
    GAIN_ME_FRM_2,
};

static kal_uint32 seamless_switch(enum MSDK_SCENARIO_ID_ENUM scenario_id, uint32_t *ae_ctrl)
{
    imgsensor.extend_frame_length_en = KAL_FALSE;
    LOG_INF("scenario_id: %d SHUTTER_NE_FRM_1: %d GAIN_NE_FRM_1: %d SHUTTER_ME_FRM_1: %d GAIN_ME_FRM_1: %d SHUTTER_SE_FRM_1: %d GAIN_SE_FRM_1: %d",
            scenario_id, ae_ctrl[SHUTTER_NE_FRM_1], ae_ctrl[GAIN_NE_FRM_1], ae_ctrl[SHUTTER_ME_FRM_1], ae_ctrl[GAIN_ME_FRM_1], ae_ctrl[SHUTTER_SE_FRM_1], ae_ctrl[GAIN_SE_FRM_1]);

    switch (scenario_id) {
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    {
        spin_lock(&imgsensor_drv_lock);
        //imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
        imgsensor.autoflicker_en = KAL_FALSE;
        imgsensor.pclk = imgsensor_info.normal_video.pclk;
        imgsensor.line_length = imgsensor_info.normal_video.linelength;
        imgsensor.frame_length = imgsensor_info.normal_video.framelength;
        imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
        spin_unlock(&imgsensor_drv_lock);

        pr_info("seamless switch normal video!\n");
        FMC_GPH_START;
        imx709_table_write_cmos_sensor(imx709_21305_seamless_normal_video, sizeof(imx709_21305_seamless_normal_video) / sizeof(kal_uint16));
        if (ae_ctrl) {
            LOG_INF("call MSDK_SCENARIO_ID_VIDEO_PREVIEW %d %d",
                    ae_ctrl[SHUTTER_NE_FRM_1] ,ae_ctrl[GAIN_NE_FRM_1]);
            set_shutter_w_gph(ae_ctrl[SHUTTER_NE_FRM_1], KAL_FALSE);
            set_gain_w_gph(ae_ctrl[GAIN_NE_FRM_1], KAL_FALSE);
        }
        FMC_GPH_END;
    }
    break;
    case MSDK_SCENARIO_ID_CUSTOM4:
    {
        spin_lock(&imgsensor_drv_lock);
        //imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
        imgsensor.autoflicker_en = KAL_FALSE;
        imgsensor.pclk = imgsensor_info.custom4.pclk;
        imgsensor.line_length = imgsensor_info.custom4.linelength;
        imgsensor.frame_length = imgsensor_info.custom4.framelength;
        imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
        spin_unlock(&imgsensor_drv_lock);

        pr_info("seamless switch dolhdr 2exp video!\n");
        FMC_GPH_START;
        imx709_table_write_cmos_sensor(imx709_21305_seamless_custom4, sizeof(imx709_21305_seamless_custom4) / sizeof(kal_uint16));
        if (ae_ctrl) {
            LOG_INF("call MSDK_SCENARIO_ID_CUSTOM4 %d %d %d %d %d %d",
                    ae_ctrl[SHUTTER_NE_FRM_1], 0, ae_ctrl[SHUTTER_SE_FRM_1],
                    ae_ctrl[GAIN_NE_FRM_1], 0, ae_ctrl[GAIN_SE_FRM_1]);
            hdr_write_double_shutter_w_gph(ae_ctrl[SHUTTER_NE_FRM_1], ae_ctrl[SHUTTER_SE_FRM_1], KAL_FALSE);
            hdr_write_double_gain_w_gph(ae_ctrl[GAIN_NE_FRM_1], ae_ctrl[GAIN_SE_FRM_1], KAL_FALSE);
        }
        FMC_GPH_END;
    }
    break;
    default:
    {
        pr_info( "error! wrong setting in set_seamless_switch = %d",scenario_id);
        return 0xff;
    }
    }
    imgsensor.fast_mode_on = KAL_TRUE;
    LOG_INF("%s success, scenario is switched to %d", __func__, scenario_id);
    return 0;
}

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
    case MSDK_SCENARIO_ID_CUSTOM6:
        custom6(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        custom7(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM8:
        custom8(image_window, sensor_config_data);
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
    write_cmos_sensor_8(0x0104, 0x01);
    set_max_framerate(imgsensor.current_fps, 1);
    write_cmos_sensor_8(0x0104, 0x00);

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
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
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
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
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
            if (imgsensor.frame_length > imgsensor.shutter) {
                write_cmos_sensor_8(0x0104, 0x01);
                set_dummy();
                write_cmos_sensor_8(0x0104, 0x00);
            }
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
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
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
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
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
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
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
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
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
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM4:
        /*frame_length = imgsensor_info.custom4.pclk / framerate * 10
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
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }*/
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
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM6:
        frame_length = imgsensor_info.custom6.pclk / framerate * 10
                / imgsensor_info.custom6.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom6.framelength)
            ? (frame_length - imgsensor_info.custom6.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom6.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        frame_length = imgsensor_info.custom7.pclk / framerate * 10
                / imgsensor_info.custom7.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom7.framelength)
            ? (frame_length - imgsensor_info.custom7.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom7.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM8:
        frame_length = imgsensor_info.custom8.pclk / framerate * 10
                / imgsensor_info.custom8.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom8.framelength)
            ? (frame_length - imgsensor_info.custom8.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom8.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
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
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
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
    case MSDK_SCENARIO_ID_CUSTOM6:
        *framerate = imgsensor_info.custom6.max_framerate;
        break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        *framerate = imgsensor_info.custom7.max_framerate;
        break;
    case MSDK_SCENARIO_ID_CUSTOM8:
        *framerate = imgsensor_info.custom8.max_framerate;
        break;
    default:
        break;
    }

    return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_uint8 modes, struct SET_SENSOR_PATTERN_SOLID_COLOR *pTestpatterndata)
{
    pr_debug("set_test_pattern enum: %d\n", modes);

    if (modes) {
        write_cmos_sensor_8(0x020E, 0x00);
        write_cmos_sensor_8(0x0218, 0x00);
        write_cmos_sensor_8(0x3021, 0x00);
    } else {
        write_cmos_sensor_8(0x020E, 0x01);
        write_cmos_sensor_8(0x0218, 0x01);
        write_cmos_sensor_8(0x3021, 0x40);
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
    uint32_t *pAeCtrls;
    uint32_t *pScenarios;
    struct SET_PD_BLOCK_INFO_T *PDAFinfo;
    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    //struct SENSOR_VC_INFO_STRUCT *pvcinfo;
    /* SET_SENSOR_AWB_GAIN *pSetSensorAWB
     *  = (SET_SENSOR_AWB_GAIN *)feature_para;
     */
    struct SENSOR_VC_INFO2_STRUCT *pvcinfo2;
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
        *(MINT32 *)(signed long)(*(feature_data + 1)) = -1345000;
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
        case MSDK_SCENARIO_ID_CUSTOM6:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom6.pclk;
                break;
        case MSDK_SCENARIO_ID_CUSTOM7:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom7.pclk;
                break;
        case MSDK_SCENARIO_ID_CUSTOM8:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom8.pclk;
                break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.pre.pclk;
                break;
        }
        break;
    case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO: {
        MUINT32 ratio = 1;

        if (*(feature_data + 2) & SENSOR_GET_LINELENGTH_FOR_READOUT) {
            ratio = get_exp_cnt_by_scenario((*feature_data));
        }
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.cap.framelength << 16)
                                 + (ratio * imgsensor_info.cap.linelength);
                break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.normal_video.framelength << 16)
                                + (ratio * imgsensor_info.normal_video.linelength);
                break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.hs_video.framelength << 16)
                                 + (ratio * imgsensor_info.hs_video.linelength);
                break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.slim_video.framelength << 16)
                                 + (ratio * imgsensor_info.slim_video.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM1:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom1.framelength << 16)
                                 + (ratio * imgsensor_info.custom1.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM2:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom2.framelength << 16)
                                 + (ratio * imgsensor_info.custom2.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM3:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom3.framelength << 16)
                                 + (ratio * imgsensor_info.custom3.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM4:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom4.framelength << 16)
                                 + (ratio * imgsensor_info.custom4.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM5:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom5.framelength << 16)
                                 + (ratio * imgsensor_info.custom5.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM6:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom6.framelength << 16)
                                 + (ratio * imgsensor_info.custom6.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM7:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom7.framelength << 16)
                                 + (ratio * imgsensor_info.custom7.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM8:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom8.framelength << 16)
                                 + (ratio * imgsensor_info.custom8.linelength);
                break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.pre.framelength << 16)
                                 + (ratio * imgsensor_info.pre.linelength);
                break;
        }
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
        case MSDK_SCENARIO_ID_CUSTOM6:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[10],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM7:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[11],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM8:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[12],
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
    case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
        /*
            HDR_NONE = 0,
            HDR_RAW = 1,
            HDR_CAMSV = 2,
            HDR_RAW_ZHDR = 9,
            HDR_MultiCAMSV = 10,
            HDR_RAW_STAGGER_2EXP = 0xB,
            HDR_RAW_STAGGER_MIN = HDR_RAW_STAGGER_2EXP,
            HDR_RAW_STAGGER_3EXP = 0xC,
            HDR_RAW_STAGGER_MAX = HDR_RAW_STAGGER_3EXP,
         */
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0xb;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
        default:
            *(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
            break;
        }
        LOG_INF("SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY scenarioId:%llu, HDR:%llu\n",
            *feature_data, *(feature_data+1));
        break;
    case SENSOR_FEATURE_GET_VC_INFO2:
        LOG_INF("SENSOR_FEATURE_GET_VC_INFO2 %d\n", (UINT16) (*feature_data));
        pvcinfo2 = (struct SENSOR_VC_INFO2_STRUCT *) (uintptr_t) (*(feature_data + 1));
        get_vc_info_2(pvcinfo2, *feature_data_32);
        break;
    case SENSOR_FEATURE_GET_STAGGER_TARGET_SCENARIO:
        if (*feature_data == MSDK_SCENARIO_ID_VIDEO_PREVIEW) {
            switch (*(feature_data + 1)) {
            case HDR_RAW_STAGGER_2EXP:
                *(feature_data + 2) = MSDK_SCENARIO_ID_CUSTOM4;//custom4 was the 2 exp mode for preview mode
                break;
            default:
                break;
            }
        }else if (*feature_data == MSDK_SCENARIO_ID_CUSTOM4) {
            switch (*(feature_data + 1)) {
            case HDR_NONE:
                *(feature_data + 2) = MSDK_SCENARIO_ID_VIDEO_PREVIEW;//normal_video mode for video preview mode
                break;
            default:
                break;
            }
        }
        break;
    case SENSOR_FEATURE_GET_STAGGER_MAX_EXP_TIME:
        if (*feature_data == MSDK_SCENARIO_ID_CUSTOM4) {
            switch (*(feature_data + 1)) {
            case VC_STAGGER_NE:
                *(feature_data + 2) = 32757;   //need repare
                break;
            case VC_STAGGER_ME:
                *(feature_data + 2) = 32757;
                break;
            case VC_STAGGER_SE:
                *(feature_data + 2) = 32757;
                break;
            default:
                *(feature_data + 2) = 32757;
                break;
            }
        } else {
            *(feature_data + 2) = 0;
        }
        break;
    case SENSOR_FEATURE_SET_HDR_SHUTTER://for 2EXP
        LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
            (UINT16) *feature_data, (UINT16) *(feature_data + 1));
        // implement write shutter for NE/SE
        hdr_write_double_shutter((UINT16)*feature_data,
            (UINT16)*(feature_data+1));
        break;
    case SENSOR_FEATURE_SET_DUAL_GAIN://for 2EXP
        LOG_INF("SENSOR_FEATURE_SET_DUAL_GAIN LE=%d, SE=%d\n",
            (UINT16) *feature_data, (UINT16) *(feature_data + 1));
        // implement write gain for NE/SE
        hdr_write_double_gain((UINT16)*feature_data,
            (UINT16)*(feature_data+1));
        break;
    case SENSOR_FEATURE_SET_PDAF:
        LOG_INF("PDAF mode :%d\n", *feature_data_16);
        imgsensor.pdaf_mode = *feature_data_16;
        break;
    case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
        set_shutter_frame_length((UINT16) (*feature_data),
                    (UINT16) (*(feature_data + 1)), (BOOL) (*(feature_data + 2)));
        break;
    case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
        LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
            (UINT16)*feature_data,
            (UINT16)*(feature_data+1),
            (UINT16)*(feature_data+2));
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
        case MSDK_SCENARIO_ID_CUSTOM6:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom6.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM7:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom7.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM8:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom8.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.pre.mipi_pixel_rate;
            break;
        }
    }
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
    /*case SENSOR_FEATURE_SET_SEAMLESS_EXTEND_FRAME_LENGTH:
        pr_info("extend_frame_len %d\n", *feature_data);
        extend_frame_length((MUINT32) *feature_data);
        pr_info("extend_frame_len done %d\n", *feature_data);
        break;*/
    case SENSOR_FEATURE_SEAMLESS_SWITCH:
    {
        if ((feature_data + 1) != NULL) {
            pAeCtrls = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
        } else {
            pr_debug("warning! no ae_ctrl input");
        }
        if (feature_data == NULL) {
            pr_info("error! input scenario is null!");
            return ERROR_INVALID_SCENARIO_ID;
        }
        LOG_INF("call seamless_switch");
        seamless_switch((*feature_data), pAeCtrls);
    }
    break;
    case SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS:
        if ((feature_data + 1) != NULL) {
            pScenarios = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
        } else {
            pr_info("input pScenarios vector is NULL!\n");
            return ERROR_INVALID_SCENARIO_ID;
        }
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CUSTOM4:
            *pScenarios = MSDK_SCENARIO_ID_VIDEO_PREVIEW;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *pScenarios = MSDK_SCENARIO_ID_CUSTOM4;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CUSTOM2:
        case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM5:
        case MSDK_SCENARIO_ID_CUSTOM6:
        case MSDK_SCENARIO_ID_CUSTOM7:
        case MSDK_SCENARIO_ID_CUSTOM8:
        default:
            *pScenarios = 0xff;
            break;
        }
        pr_debug("SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS %d %d\n",
        *feature_data, *pScenarios);
        break;
    case SENSOR_FEATURE_GET_SEAMLESS_SYSTEM_DELAY:
        {
            int i;
            *(feature_data + 2) = 0;
            for (i = 0; i < sizeof(seamless_sys_delays) / sizeof(struct SEAMLESS_SYS_DELAY); i++) {
                if (*feature_data == seamless_sys_delays[i].source_scenario &&
                    *(feature_data + 1) == seamless_sys_delays[i].target_scenario) {
                    *(feature_data + 2) = seamless_sys_delays[i].sys_delay;
                    break;
                }
            }
        }

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

UINT32 IMX709_MIPI_RAW_21305_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc != NULL)
        *pfFunc = &sensor_func;
    return ERROR_NONE;
} /* IMX709_MIPI_RAW_SensorInit */
