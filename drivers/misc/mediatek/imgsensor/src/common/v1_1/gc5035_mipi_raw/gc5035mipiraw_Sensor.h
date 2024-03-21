/*****************************************************************************
 *
 * Filename:
 * ---------
 *     GC5035mipi_Sensor.h
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     CMOS sensor header file
 *
 ****************************************************************************/
#ifndef __GC5035MIPI_SENSOR_H__
#define __GC5035MIPI_SENSOR_H__

/* mirror/flip */

#define  GC5035_MIRROR_NORMAL
#undef  GC5035_MIRROR_H
#undef  GC5035_MIRROR_V
#undef  GC5035_MIRROR_HV
#undef GC5035_MIRROR_HV

#if defined(GC5035_MIRROR_NORMAL)
#define GC5035_MIRROR    0x80
#define GC5035_RSTDUMMY1 0x02
#define GC5035_RSTDUMMY2 0x7c
#elif defined(GC5035_MIRROR_H)
#define GC5035_MIRROR    0x81
#define GC5035_RSTDUMMY1 0x02
#define GC5035_RSTDUMMY2 0x7c
#elif defined(GC5035_MIRROR_V)
#define GC5035_MIRROR    0x82
#define GC5035_RSTDUMMY1 0x03
#define GC5035_RSTDUMMY2 0xfc
#elif defined(GC5035_MIRROR_HV)
#define GC5035_MIRROR    0x83
#define GC5035_RSTDUMMY1 0x03
#define GC5035_RSTDUMMY2 0xfc
#else
#define GC5035_MIRROR    0x80
#define GC5035_RSTDUMMY1 0x02
#define GC5035_RSTDUMMY2 0x7c
#endif

/* Gain */
#define SENSOR_BASE_GAIN 256
#define MAX_GAIN         16
#define SENSOR_MAX_GAIN  (MAX_GAIN * SENSOR_BASE_GAIN)
#define E_GAIN_INDEX     17
#define MAX_GAIN_INDEX   17

/* OTP */
#define GC5035_OTP_CUSTOMER
#undef GC5035_OTP_DEBUG

#if defined(GC5035_OTP_DEBUG)
#define OTP_START_ADDR 0x0000
#define OTP_DATA_LENGTH 1024
#endif
#define DPC_FLAG_OFFSET         0x0068
#define DPC_TOTAL_NUMBER_OFFSET 0x0070
#define DPC_ERROR_NUMBER_OFFSET 0x0078
#define REG_INFO_FLAG_OFFSET 0x0880
#define REG_INFO_PAGE_OFFSET 0x0888
#define REG_INFO_ADDR_OFFSET 0x0890
#define REG_INFO_VALUE_OFFSET 0x0898
#define REG_INFO_SIZE 5
#if defined(GC5035_OTP_CUSTOMER)
#define MODULE_INFO_FLAG_OFFSET 0x1f10
#define MODULE_INFO_OFFSET      0x1f18
#define MODULE_INFO_SIZE 6
#define WB_INFO_FLAG_OFFSET     0x1f78
#define WB_INFO_OFFSET          0x1f80
#define WB_INFO_SIZE            4
#define WB_GOLDEN_INFO_OFFSET   0x1fc0
#define WB_GOLDEN_INFO_SIZE     4

#define RG_TYPICAL       0x0400
#define BG_TYPICAL       0x0400
#endif

struct gc5035_otp {
	kal_uint8  dd_flag;
	kal_uint16 dd_num;
	kal_uint8  reg_flag;
	kal_uint8  reg_num;
	kal_uint8  regs[10][3];
#if defined(GC5035_OTP_CUSTOMER)
	kal_uint8  wb_flag;
	kal_uint16 rg_gain;
	kal_uint16 bg_gain;
	kal_uint8  golden_flag;
	kal_uint16 golden_rg;
	kal_uint16 golden_bg;
#endif
};// gc5035_otp_struct;

enum {
	IMGSENSOR_MODE_INIT,
	IMGSENSOR_MODE_PREVIEW,
	IMGSENSOR_MODE_CAPTURE,
	IMGSENSOR_MODE_VIDEO,
	IMGSENSOR_MODE_HIGH_SPEED_VIDEO,
	IMGSENSOR_MODE_SLIM_VIDEO,
	IMGSENSOR_MODE_CUSTOM1,
};

struct imgsensor_mode_struct {
	kal_uint32 pclk;                /* record different mode's pclk */
	kal_uint32 linelength;          /* record different mode's linelength */
	kal_uint32 framelength;         /* record different mode's framelength */
	kal_uint8  startx;              /* record different mode's startx of grabwindow */
	kal_uint8  starty;              /* record different mode's startx of grabwindow */
	kal_uint16 grabwindow_width;    /* record different mode's width of grabwindow */
	kal_uint16 grabwindow_height;   /* record different mode's height of grabwindow */
	kal_uint32 mipi_pixel_rate;
	/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
	kal_uint8  mipi_data_lp2hs_settle_dc;
	/* following for GetDefaultFramerateByScenario() */
	kal_uint16 max_framerate;
};

/* SENSOR PRIVATE STRUCT FOR VARIABLES */
struct imgsensor_struct {
	kal_uint8  mirror;                /* mirrorflip information */
	kal_uint8  sensor_mode;           /* record IMGSENSOR_MODE enum value */
	kal_uint32 shutter;               /* current shutter */
	kal_uint16 gain;                  /* current gain */
	kal_uint32 pclk;                  /* current pclk */
	kal_uint32 frame_length;          /* current framelength */
	kal_uint32 line_length;           /* current linelength */
	kal_uint32 min_frame_length;      /* current min  framelength to max framerate */
	kal_uint16 dummy_pixel;           /* current dummypixel */
	kal_uint16 dummy_line;            /* current dummline */
	kal_uint16 current_fps;           /* current max fps */
	kal_bool   autoflicker_en;        /* record autoflicker enable or disable */
	kal_bool   test_pattern;          /* record test pattern mode or not */
	enum MSDK_SCENARIO_ID_ENUM current_scenario_id; /* current scenario id */
	kal_uint8  ihdr_en;               /* ihdr enable or disable */
	kal_uint8  i2c_write_id;          /* record current sensor's i2c write id */
};

/* SENSOR PRIVATE STRUCT FOR CONSTANT */
struct imgsensor_info_struct {
	kal_uint32 sensor_id;                     /* record sensor id defined in Kd_imgsensor.h */
	kal_uint32 checksum_value;                /* checksum value for Camera Auto Test */
	struct imgsensor_mode_struct pre;         /* preview scenario relative information */
	struct imgsensor_mode_struct cap;         /* capture scenario relative information */
	struct imgsensor_mode_struct cap1;        /* capture for PIP 24fps relative information */
	/* capture1 mode must use same framelength, linelength with Capture mode for shutter calculate */
	struct imgsensor_mode_struct normal_video;/* normal video  scenario relative information */
	struct imgsensor_mode_struct hs_video;    /* high speed video scenario relative information */
	struct imgsensor_mode_struct slim_video;  /* slim video for VT scenario relative information */
	struct imgsensor_mode_struct custom1;     /* custom1 for stereo scenario relative information */
	kal_uint8 ae_shut_delay_frame;            /* shutter delay frame for AE cycle */
	kal_uint8 ae_sensor_gain_delay_frame;     /* sensor gain delay frame for AE cycle */
	kal_uint8 ae_ispGain_delay_frame;         /* isp gain delay frame for AE cycle */
	kal_uint8 ihdr_support;                   /* 1, support; 0,not support */
	kal_uint8 ihdr_le_firstline;              /* 1,le first ; 0, se first */
	kal_uint8 sensor_mode_num;                /* support sensor mode num */
	kal_uint8 cap_delay_frame;                /* enter capture delay frame num */
	kal_uint8 pre_delay_frame;                /* enter preview delay frame num */
	kal_uint8 video_delay_frame;              /* enter video delay frame num */
	kal_uint8 hs_video_delay_frame;           /* enter high speed video  delay frame num */
	kal_uint8 slim_video_delay_frame;         /* enter slim video delay frame num */
	kal_uint8 custom1_delay_frame;            /* enter custom1 delay frame num */
	kal_uint8 frame_time_delay_frame;         /* enter frame_time_delay_frame num */
	kal_uint8 margin;                         /* sensor framelength & shutter margin */
	kal_uint32 min_shutter;                   /* min shutter */
	kal_uint32 max_frame_length;              /* max framelength by sensor register's limitation */
	kal_uint8 isp_driving_current;            /* mclk driving current */
	kal_uint8 sensor_interface_type;          /* sensor_interface_type */
	kal_uint8 mipi_sensor_type;
	/* 0, MIPI_OPHY_NCSI2; 1, MIPI_OPHY_CSI2, default is NCSI2, don't modify this para */
	kal_uint8 mipi_settle_delay_mode;
	/* 0, high speed signal auto detect; 1, use settle delay, unit is ns*/
	/* default is auto detect, don't modify this para */
	kal_uint8 sensor_output_dataformat;       /* sensor output first pixel color */
	kal_uint8 mclk;                           /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	kal_uint8 mipi_lane_num;                  /* mipi lane num */
	kal_uint8 i2c_addr_table[5];
	/* record sensor support all write id addr, only supprt 4must end with 0xff */
};

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iWriteReg(u16 a_u2Addr, u32 a_u4Data, u32 a_u4Bytes, u16 i2cId);
extern int iMultiReadReg(u16 a_u2Addr, u8 *a_puBuff, u16 i2cId, u8 number);
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId);

#endif
