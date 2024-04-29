/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _KD_IMGSENSOR_H
#define _KD_IMGSENSOR_H

#include <linux/ioctl.h>

#ifndef ASSERT
#define ASSERT(expr)        WARN_ON(!(expr))
#endif
#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif
#define IMGSENSORMAGIC 'i'
/* IOCTRL(inode * ,file * ,cmd ,arg ) */
/* S means "set through a ptr" */
/* T means "tell by a arg value" */
/* G means "get by a ptr" */
/* Q means "get by return a value" */
/* X means "switch G and S atomically" */
/* H means "switch T and Q atomically" */

/******************************************************************************
 *
 ******************************************************************************/

/* sensorOpen */
#define KDIMGSENSORIOC_T_OPEN \
	_IO(IMGSENSORMAGIC, 0)
/* sensorGetInfo */
#define KDIMGSENSORIOC_X_GET_CONFIG_INFO \
	_IOWR(IMGSENSORMAGIC, 5, struct IMGSENSOR_GET_CONFIG_INFO_STRUCT)

#define KDIMGSENSORIOC_X_GETINFO \
	_IOWR(IMGSENSORMAGIC, 5, struct ACDK_SENSOR_GETINFO_STRUCT)
/* sensorGetResolution */
#define KDIMGSENSORIOC_X_GETRESOLUTION \
	_IOWR(IMGSENSORMAGIC, 10, struct ACDK_SENSOR_RESOLUTION_INFO_STRUCT)
/* For kernel 64-bit */
#define KDIMGSENSORIOC_X_GETRESOLUTION2 \
	_IOWR(IMGSENSORMAGIC, 10, struct ACDK_SENSOR_PRESOLUTION_STRUCT)
/* sensorFeatureControl */
#define KDIMGSENSORIOC_X_FEATURECONCTROL \
	_IOWR(IMGSENSORMAGIC, 15, struct ACDK_SENSOR_FEATURECONTROL_STRUCT)
/* sensorControl */
#define KDIMGSENSORIOC_X_CONTROL \
	_IOWR(IMGSENSORMAGIC, 20, struct ACDK_SENSOR_CONTROL_STRUCT)
/* sensorClose */
#define KDIMGSENSORIOC_T_CLOSE \
	_IO(IMGSENSORMAGIC, 25)
/* sensorSearch */
#define KDIMGSENSORIOC_T_CHECK_IS_ALIVE \
	_IO(IMGSENSORMAGIC, 30)
/* set sensor driver */
#define KDIMGSENSORIOC_X_SET_DRIVER \
	_IOWR(IMGSENSORMAGIC, 35, struct SENSOR_DRIVER_INDEX_STRUCT)
/* get socket postion */
#define KDIMGSENSORIOC_X_GET_SOCKET_POS \
	_IOWR(IMGSENSORMAGIC, 40, u32)
/* set I2C bus */
#define KDIMGSENSORIOC_X_SET_I2CBUS \
	_IOWR(IMGSENSORMAGIC, 45, u32)
/* set I2C bus */
#define KDIMGSENSORIOC_X_RELEASE_I2C_TRIGGER_LOCK \
	_IO(IMGSENSORMAGIC, 50)
/* Set Shutter Gain Wait Done */
#define KDIMGSENSORIOC_X_SET_SHUTTER_GAIN_WAIT_DONE \
	_IOWR(IMGSENSORMAGIC, 55, u32)
/* set mclk */
#define KDIMGSENSORIOC_X_SET_MCLK_PLL \
	_IOWR(IMGSENSORMAGIC, 60, struct ACDK_SENSOR_MCLK_STRUCT)
#define KDIMGSENSORIOC_X_GETINFO2 \
	_IOWR(IMGSENSORMAGIC, 65, struct IMAGESENSOR_GETINFO_STRUCT)
/* set open/close sensor index */
#define KDIMGSENSORIOC_X_SET_CURRENT_SENSOR \
	_IOWR(IMGSENSORMAGIC, 70, u32)
/* set GPIO */
#define KDIMGSENSORIOC_X_SET_GPIO \
	_IOWR(IMGSENSORMAGIC, 75, struct IMGSENSOR_GPIO_STRUCT)
/* Get ISP CLK */
#define KDIMGSENSORIOC_X_GET_ISP_CLK \
	_IOWR(IMGSENSORMAGIC, 80, u32)
/* Get CSI CLK */
#define KDIMGSENSORIOC_X_GET_CSI_CLK \
	_IOWR(IMGSENSORMAGIC, 85, u32)

/* Get ISP CLK via MMDVFS*/
#define KDIMGSENSORIOC_DFS_UPDATE \
	_IOWR(IMGSENSORMAGIC, 90, unsigned int)
#define KDIMGSENSORIOC_GET_SUPPORTED_ISP_CLOCKS \
	_IOWR(IMGSENSORMAGIC, 95, struct IMAGESENSOR_GET_SUPPORTED_ISP_CLK)
#define KDIMGSENSORIOC_GET_CUR_ISP_CLOCK \
	_IOWR(IMGSENSORMAGIC, 100, unsigned int)

#ifdef CONFIG_COMPAT
#define COMPAT_KDIMGSENSORIOC_X_GET_CONFIG_INFO \
	_IOWR(IMGSENSORMAGIC, 5, struct COMPAT_IMGSENSOR_GET_CONFIG_INFO_STRUCT)

#define COMPAT_KDIMGSENSORIOC_X_GETINFO \
	_IOWR(IMGSENSORMAGIC, 5, struct COMPAT_ACDK_SENSOR_GETINFO_STRUCT)
#define COMPAT_KDIMGSENSORIOC_X_FEATURECONCTROL \
	_IOWR(IMGSENSORMAGIC, 15, \
		struct COMPAT_ACDK_SENSOR_FEATURECONTROL_STRUCT)
#define COMPAT_KDIMGSENSORIOC_X_CONTROL \
	_IOWR(IMGSENSORMAGIC, 20, struct COMPAT_ACDK_SENSOR_CONTROL_STRUCT)
#define COMPAT_KDIMGSENSORIOC_X_GETINFO2 \
	_IOWR(IMGSENSORMAGIC, 65, struct COMPAT_IMAGESENSOR_GETINFO_STRUCT)
#define COMPAT_KDIMGSENSORIOC_X_GETRESOLUTION2 \
	_IOWR(IMGSENSORMAGIC, 10, struct COMPAT_ACDK_SENSOR_PRESOLUTION_STRUCT)
#endif

/************************************************************************
 *
 ************************************************************************/

/* Onyx */
#define OV13B10LN_SENSOR_ID                       0x0D42
#define SENSOR_DRVNAME_OV13B10LN_MIPI_RAW         "ov13b10ln_mipi_raw"
#define S5K4H7LN_SENSOR_ID                        0x487B
#define SENSOR_DRVNAME_S5K4H7LN_MIPI_RAW          "s5k4h7ln_mipi_raw"

#define SENSOR_DRVNAME_OV13B10_MIPI_RAW         "ov13b10_mipi_raw"
#define OV13B10TXD_SENSOR_ID                    0x560D43
#define SENSOR_DRVNAME_OV13B10TXD_MIPI_RAW      "ov13b10txd_mipi_raw"
#define S5K4H7_SENSOR_ID                        0x487B
#define SENSOR_DRVNAME_S5K4H7_MIPI_RAW          "s5k4h7_mipi_raw"
#define S5K4H7SUB_SENSOR_ID                     0x487C
#define SENSOR_DRVNAME_S5K4H7SUB_MIPI_RAW       "s5k4h7sub_mipi_raw"
#define GC08A3MIPI_SENSOR_ID                    0x08a3
#define SENSOR_DRVNAME_GC08A3_MIPI_RAW          "gc08a3_mipi_raw"
#define GC08A3SUBMIPI_SENSOR_ID                 0x08a4
#define SENSOR_DRVNAME_GC08A3SUB_MIPI_RAW       "gc08a3sub_mipi_raw"


/* SENSOR CHIP VERSION */
/*IMX*/
#define IMX766_SENSOR_ID                        0x0766
#define IMX499_SENSOR_ID                        0x0499
#define IMX481_SENSOR_ID                        0x0481
#define IMX486_SENSOR_ID                        0x0486
#define IMX586_SENSOR_ID                        0x0586
#define IMX519_SENSOR_ID                        0x0519
#define IMX576_SENSOR_ID                        0x0576
#define IMX350_SENSOR_ID                        0x0350
#define IMX398_SENSOR_ID                        0x0398
#define IMX268_SENSOR_ID                        0x0268
#define IMX386_SENSOR_ID                        0x0386
#define IMX300_SENSOR_ID                        0x0300
#define IMX386_MONO_SENSOR_ID                   0x0286
#define IMX362_SENSOR_ID                        0x0362
#define IMX338_SENSOR_ID                        0x0338
#define IMX376_SENSOR_ID                        0x0376
#define IMX318_SENSOR_ID                        0x0318
#define IMX319_SENSOR_ID                        0x0319
#define IMX377_SENSOR_ID                        0x0377
#define IMX278_SENSOR_ID                        0x0278
#define IMX258_SENSOR_ID                        0x0258
#define IMX258_MONO_SENSOR_ID                   0x0259
#define IMX230_SENSOR_ID                        0x0230
#define IMX220_SENSOR_ID                        0x0220
#define IMX219_SENSOR_ID                        0x0219
#define IMX214_SENSOR_ID                        0x0214
#define IMX214_MONO_SENSOR_ID                   0x0215
#define IMX179_SENSOR_ID                        0x0179
#define IMX178_SENSOR_ID                        0x0178
#define IMX135_SENSOR_ID                        0x0135
#define IMX132MIPI_SENSOR_ID                    0x0132
#define IMX119_SENSOR_ID                        0x0119
#define IMX105_SENSOR_ID                        0x0105
#define IMX091_SENSOR_ID                        0x0091
#define IMX073_SENSOR_ID                        0x0046
#define IMX058_SENSOR_ID                        0x0058
#define IMX882_SENSOR_ID                        0x8206
#define IMX882_SENSOR_ID2                       0x8202
/*OV*/
#define OV23850_SENSOR_ID                       0x023850
#define OV16880_SENSOR_ID                       0x016880
#define OV16825MIPI_SENSOR_ID                   0x016820
#define OV13855_SENSOR_ID                       0xD855
#define OV13850_SENSOR_ID                       0xD850
#define OV12A10_SENSOR_ID                       0x1241
#define OV13870_SENSOR_ID                       0x013870
#define OV13850_SENSOR_ID                       0xD850
#define OV13855_SENSOR_ID                       0xD855
#define OV16885_SENSOR_ID                       0x16885
#define OV13855MAIN2_SENSOR_ID                  0xD856
#define OV12830_SENSOR_ID                       0xC830
#define OV9760MIPI_SENSOR_ID                    0x9760
#define OV9740MIPI_SENSOR_ID                    0x9740
#define OV9726_SENSOR_ID                        0x9726
#define OV9726MIPI_SENSOR_ID                    0x9726
#define OV8865_SENSOR_ID                        0x8865
#define OV8858_SENSOR_ID                        0x8858
#define OV8858S_SENSOR_ID                      (0x8858+1)
#define OV8856_SENSOR_ID                        0x885A
#define OV8830_SENSOR_ID                        0x8830
#define OV8825_SENSOR_ID                        0x8825
#define OV7675_SENSOR_ID                        0x7673
#define OV5693_SENSOR_ID                        0x5690
#define OV5670MIPI_SENSOR_ID                    0x5670
#define OV5670MIPI_SENSOR_ID_2                  (0x5670+010000)
#define OV2281MIPI_SENSOR_ID                    0x5670
#define OV5675MIPI_SENSOR_ID                    0x5675
#define OV5671MIPI_SENSOR_ID                    0x5671
#define OV5650_SENSOR_ID                        0x5651
#define OV5650MIPI_SENSOR_ID                    0x5651
#define OV5648MIPI_SENSOR_ID                    0x5648
#define OV5647_SENSOR_ID                        0x5647
#define OV5647MIPI_SENSOR_ID                    0x5647
#define OV5645MIPI_SENSOR_ID                    0x5645
#define OV5642_SENSOR_ID                        0x5642
#define OV4688MIPI_SENSOR_ID                    0x4688
#define OV3640_SENSOR_ID                        0x364C
#define OV2724MIPI_SENSOR_ID                    0x2724
#define OV2722MIPI_SENSOR_ID                    0x2722
#define OV2680MIPI_SENSOR_ID                    0x2680
#define OV2680_SENSOR_ID                        0x2680
#define OV2659_SENSOR_ID                        0x2656
#define OV2655_SENSOR_ID                        0x2656
#define OV2650_SENSOR_ID                        0x2652
#define OV2650_SENSOR_ID_1                      0x2651
#define OV2650_SENSOR_ID_2                      0x2652
#define OV2650_SENSOR_ID_3                      0x2655
#define OV20880MIPI_SENSOR_ID                   0x20880
#define OV05A20_SENSOR_ID	                    0x5305
/*S5K*/
#define S5KJD1_SENSOR_ID                        0x3841
#define S5K2LQSX_SENSOR_ID                      0x2c1a
#define S5K4H7_SENSOR_ID                        0x487B
#define S5K4H7YXSUB_SENSOR_ID                   0x487A
#define S5K3P8SP_SENSOR_ID                      0x3108
#define S5K2T7SP_SENSOR_ID                      0x2147
#define S5K3P8SX_SENSOR_ID                      0x3108
#define S5K2L7_SENSOR_ID                        0x20C7
#define S5K3L6_SENSOR_ID                        0x30C6
#define S5K3L8_SENSOR_ID                        0x30C8
#define S5K3M3_SENSOR_ID                        0x30D3
#define S5K3M5SX_SENSOR_ID                      0x30D5
#define S5K2X8_SENSOR_ID                        0x2188
#define S5K2P7_SENSOR_ID                        0x2107
#define S5K2P8_SENSOR_ID                        0x2108
#define S5K3P3_SENSOR_ID                        0x3103
#define S5K3P3SX_SENSOR_ID                      0x3103
#define S5K3P8_SENSOR_ID                        0x3108
#define S5K3P8STECH_SENSOR_ID                   0xf3108
#define S5K3M2_SENSOR_ID                        0x30D2
#define S5K4E6_SENSOR_ID                        0x4e60
#define S5K3AAEA_SENSOR_ID                      0x07AC
#define S5K3BAFB_SENSOR_ID                      0x7070
#define S5K3H7Y_SENSOR_ID                       0x3087
#define S5K3H2YX_SENSOR_ID                      0x382b
#define S5KA3DFX_SENSOR_ID                      0x00AB
#define S5K3E2FX_SENSOR_ID                      0x3E2F
#define S5K4B2FX_SENSOR_ID                      0x5080
#define S5K4E1GA_SENSOR_ID                      0x4E10
#define S5K4ECGX_SENSOR_ID                      0x4EC0
#define S5K53BEX_SENSOR_ID                      0x45A8
#define S5K53BEB_SENSOR_ID                      0x87A8
#define S5K5BAFX_SENSOR_ID                      0x05BA
#define S5K5E2YA_SENSOR_ID                      0x5e20
#define S5K4H5YX_2LANE_SENSOR_ID                0x485B
#define S5K4H5YC_SENSOR_ID                      0x485B
#define S5K83AFX_SENSOR_ID                      0x01C4
#define S5K5CAGX_SENSOR_ID                      0x05ca
#define S5K8AAYX_MIPI_SENSOR_ID                 0x08aa
#define S5K8AAYX_SENSOR_ID                      0x08aa
#define S5K5E8YX_SENSOR_ID                      0x5e80
#define S5K5E8YXREAR2_SENSOR_ID                 0x5e81
#define S5K5E9_SENSOR_ID                        0x559b
#define S5KGD1SP_SENSOR_ID                      0x0841
#define S5KHM2SP_SENSOR_ID                      0x1AD2
/*HI*/
#define HI841_SENSOR_ID                         0x0841
#define HI707_SENSOR_ID                         0x00b8
#define HI704_SENSOR_ID                         0x0096
#define HI556_SENSOR_ID                         0x0556
#define HI551_SENSOR_ID                         0x0551
#define HI553_SENSOR_ID                         0x0553
#define HI545MIPI_SENSOR_ID                     0x0545
#define HI544MIPI_SENSOR_ID                     0x0544
#define HI542_SENSOR_ID                         0x00B1
#define HI542MIPI_SENSOR_ID                     0x00B1
#define HI253_SENSOR_ID                         0x0092
#define HI251_SENSOR_ID                         0x0084
#define HI191MIPI_SENSOR_ID                     0x0191
#define HIVICF_SENSOR_ID                        0x0081
/*MT*/
#define MT9D011_SENSOR_ID                       0x1511
#define MT9D111_SENSOR_ID                       0x1511
#define MT9D112_SENSOR_ID                       0x1580
#define MT9M011_SENSOR_ID                       0x1433
#define MT9M111_SENSOR_ID                       0x143A
#define MT9M112_SENSOR_ID                       0x148C
#define MT9M113_SENSOR_ID                       0x2480
#define MT9P012_SENSOR_ID                       0x2800
#define MT9P012_SENSOR_ID_REV7                  0x2801
#define MT9T012_SENSOR_ID                       0x1600
#define MT9T013_SENSOR_ID                       0x2600
#define MT9T113_SENSOR_ID                       0x4680
#define MT9V112_SENSOR_ID                       0x1229
#define MT9DX11_SENSOR_ID                       0x1519
#define MT9D113_SENSOR_ID                       0x2580
#define MT9D115_SENSOR_ID                       0x2580
#define MT9D115MIPI_SENSOR_ID                   0x2580
#define MT9V113_SENSOR_ID                       0x2280
#define MT9V114_SENSOR_ID                       0x2283
#define MT9V115_SENSOR_ID                       0x2284
#define MT9P015_SENSOR_ID                       0x2803
#define MT9P017_SENSOR_ID                       0x4800
#define MT9P017MIPI_SENSOR_ID                   0x4800
#define MT9T113MIPI_SENSOR_ID                   0x4680
/*GC*/
#define GC5035_SENSOR_ID                        0x5035
#define GC2375_SENSOR_ID                        0x2375
#define GC2375H_SENSOR_ID                       0x2375
#define GC2375SUB_SENSOR_ID                     0x2376
#define GC2365_SENSOR_ID                        0x2365
#define GC2366_SENSOR_ID                        0x2366
#define GC2355_SENSOR_ID                        0x2355
#define GC2235_SENSOR_ID                        0x2235
#define GC2035_SENSOR_ID                        0x2035
#define GC2145_SENSOR_ID                        0x2145
#define GC0330_SENSOR_ID                        0xC1
#define GC0329_SENSOR_ID                        0xC0
#define GC0310_SENSOR_ID                        0xa310
#define GC0313MIPI_YUV_SENSOR_ID                0xD0
#define GC0312_SENSOR_ID                        0xb310
#define GC8034_SENSOR_ID                        0x8044
#define GC8C34_SENSOR_ID                        0x80C4
/*SP*/
#define SP0A19_YUV_SENSOR_ID                    0xA6
#define SP2518_YUV_SENSOR_ID                    0x53
#define SP2509_SENSOR_ID                        0x2509
#define SP250A_SENSOR_ID                        0x250A
/*A*/
#define A5141MIPI_SENSOR_ID                     0x4800
#define A5142MIPI_SENSOR_ID                     0x4800
/*HM*/
#define HM3451_SENSOR_ID                        0x345
/*AR*/
#define AR0833_SENSOR_ID                        0x4B03
/*SIV*/
#define SID020A_SENSOR_ID                       0x12B4
#define SIV100B_SENSOR_ID                       0x0C11
#define SIV100A_SENSOR_ID                       0x0C10
#define SIV120A_SENSOR_ID                       0x1210
#define SIV120B_SENSOR_ID                       0x0012
#define SIV121D_SENSOR_ID                       0xDE
#define SIM101B_SENSOR_ID                       0x09A0
#define SIM120C_SENSOR_ID                       0x0012
#define SID130B_SENSOR_ID                       0x001b
#define SIC110A_SENSOR_ID                       0x000D
#define SIV120B_SENSOR_ID                       0x0012
/*PAS (PixArt Image)*/
#define PAS105_SENSOR_ID                        0x0065
#define PAS302_SENSOR_ID                        0x0064
#define PAS5101_SENSOR_ID                       0x0067
#define PAS6180_SENSOR_ID                       0x6179
/*Panasoic*/
#define MN34152_SENSOR_ID                       0x01
/*Toshiba*/
#define T4KA7_SENSOR_ID                         0x2c30
/*Others*/
#define SHARP3D_SENSOR_ID                       0x003d
#define T8EV5_SENSOR_ID                         0x1011

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define S5KJN103_SENSOR_ID_LIJING                         0x38E6//(0x38E1+5)
#define S5K3L6_SENSOR_ID_LIJING                           0x30CA//(0x30C6+4)
#define SC800CS_SENSOR_ID_LIJING                          0xd127 //(0xd126+1)
#define HI846_SENSOR_ID_LIJING                            0x084f //(0x0846+9)
#define SC201CS_SENSOR_ID_LIJING                          0xeb2e //(0XEB2C+2)
#define C2519_SENSOR_ID_LIJING                            0x020F //(0x020E+1)
#define SENSOR_DRVNAME_S5KJN103_MIPI_RAW_LIJING       "s5kjn103_mipi_raw_lijing"
#define SENSOR_DRVNAME_S5K3L6_MIPI_RAW_LIJING         "s5k3l6_mipi_raw_lijing"
#define SENSOR_DRVNAME_SC800CS_MIPI_RAW_LIJING        "sc800cs_mipi_raw_lijing"
#define SENSOR_DRVNAME_HI846_MIPI_RAW_LIJING          "hi846_mipi_raw_lijing"
#define SENSOR_DRVNAME_SC201CS_MIPI_RAW_LIJING        "sc201cs_mipi_raw_lijing"
#define SENSOR_DRVNAME_C2519_MIPI_RAW_LIJING          "c2519_mipi_raw_lijing"
#endif /*OPLUS_FEATURE_CAMERA_COMMON*/

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define S5KJN103_SENSOR_ID_FANLI                         0x38E7//(0x38E1+6)
#define S5K3L6_SENSOR_ID_FANLI                           0x30CB//(0x30C6+5)
#define OV08D10_SENSOR_ID_FANLI                          0x56084702  /*0x56084700+2*/
#define OV02B1B_SENSOR_ID_FANLI                          0x002F //(0x002E+1)
#define GC02M1_SENSOR_ID_FANLI                           0x02e3 //(0x02e0+3)
#define SENSOR_DRVNAME_GC02M1_MIPI_RAW_FANLI         "gc02m1_mipi_raw_fanli"
#define SENSOR_DRVNAME_OV02B1B_MIPI_MONO_FANLI       "ov02b1b_mipi_mono_fanli"
#define SENSOR_DRVNAME_OV08D10_MIPI_RAW_FANLI        "ov08d10_mipi_raw_fanli"
#define SENSOR_DRVNAME_S5KJN103_MIPI_RAW_FANLI       "s5kjn103_mipi_raw_fanli"
#define SENSOR_DRVNAME_S5K3L6_MIPI_RAW_FANLI         "s5k3l6_mipi_raw_fanli"
#endif /*OPLUS_FEATURE_CAMERA_COMMON*/

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define SENSOR_DRVNAME_HI846P2Q_MIPI_RAW         "hi846p2q_mipi_raw"
#define SENSOR_DRVNAME_OV02A10P2Q_MIPI_MONO      "ov02a10p2q_mipi_mono"
#define SENSOR_DRVNAME_OV02A10P2Q_MIPI_MONO1     "ov02a10p2q_mipi_mono1"
#define SENSOR_DRVNAME_GC02M0P2Q_MIPI_MONO       "gc02m0p2q_mipi_mono"
#define SENSOR_DRVNAME_GC02M0P2Q_MIPI_MONO1      "gc02m0p2q_mipi_mono1"
#define SENSOR_DRVNAME_GC5035_MIPI_RAW          "gc5035_mipi_raw"
#define SENSOR_DRVNAME_S5KGD1SP_MIPI_RAW        "s5kgd1sp_mipi_raw"
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define S5K3L6_SENSOR_ID_LIJINGA                           0x30CC//(0x30C6+6)
#define HI846_SENSOR_ID_LIJINGA                            0x0850 //(0x0846+10)
#define HI556_SENSOR_ID_LIJINGA                            0x0561 //(0x0556+11)
#define SC201CS_SENSOR_ID_LIJINGA                          0xeb2f //(0XEB2C+3)
#define SC520CS_SENSOR_ID_LIJINGA                          0xee4b //(0XEE4B)

#define SENSOR_DRVNAME_S5K3L6_MIPI_RAW_LIJINGA         "s5k3l6_mipi_raw_lijinga"
#define SENSOR_DRVNAME_HI846_MIPI_RAW_LIJINGA          "hi846_mipi_raw_lijinga"
#define SENSOR_DRVNAME_HI556_MIPI_RAW_LIJINGA          "hi556_mipi_raw_lijinga"
#define SENSOR_DRVNAME_SC201CS_MIPI_RAW_LIJINGA        "sc201cs_mipi_raw_lijinga"
#define SENSOR_DRVNAME_SC520CS_MIPI_RAW_LIJINGA        "sc520cs_mipi_raw_lijinga"

#endif /*OPLUS_FEATURE_CAMERA_COMMON*/

#define S5KGD1SP_SENSOR_ID                      0x0841
#define HI846_SENSOR_ID                         0x0846
#define OV02A10_MONO_SENSOR_ID                  0x2509
#define IMX686_SENSOR_ID                        0X0686
#define IMX616_SENSOR_ID                        0x0616
#define OV48C_SENSOR_ID                         0x564843
#define IMX355_SENSOR_ID                        0x0355
#define OV13B10_SENSOR_ID                       0x560d42
#define IMX355_SENSOR_ID_LAFITE                  0xa355


#define OV48B_SENSOR_ID                         0x564842
#define S5KGM1SP_SENSOR_ID                      0x08D1
#define GC8054_SENSOR_ID                        0x8054
#define GC02K0_SENSOR_ID                        0x2385
#define OV16A10_SENSOR_ID                       0x561641
#define GC02M1B_SENSOR_ID                       0x02e0

/* CAMERA DRIVER NAME */
#define CAMERA_HW_DEVNAME                       "kd_camera_hw"
/* SENSOR DEVICE DRIVER NAME */
/*IMX*/
#define SENSOR_DRVNAME_IMX499_MIPI_RAW          "imx499_mipi_raw"
#define SENSOR_DRVNAME_IMX499_MIPI_RAW_13M      "imx499_mipi_raw_13m"
#define SENSOR_DRVNAME_IMX481_MIPI_RAW          "imx481_mipi_raw"
#define SENSOR_DRVNAME_IMX486_MIPI_RAW          "imx486_mipi_raw"
#define SENSOR_DRVNAME_IMX586_MIPI_RAW          "imx586_mipi_raw"
#define SENSOR_DRVNAME_IMX586P2Q2_MIPI_RAW       "imx586p2q2_mipi_raw"
#define SENSOR_DRVNAME_IMX586P2Q_MIPI_RAW       "imx586p2q_mipi_raw"
#define SENSOR_DRVNAME_IMX519_MIPI_RAW          "imx519_mipi_raw"
#define SENSOR_DRVNAME_IMX519DUAL_MIPI_RAW      "imx519dual_mipi_raw"
#define SENSOR_DRVNAME_IMX576_MIPI_RAW          "imx576_mipi_raw"
#define SENSOR_DRVNAME_IMX350_MIPI_RAW          "imx350_mipi_raw"
#define SENSOR_DRVNAME_IMX398_MIPI_RAW          "imx398_mipi_raw"
#define SENSOR_DRVNAME_IMX268_MIPI_RAW          "imx268_mipi_raw"
#define SENSOR_DRVNAME_IMX386_MIPI_RAW          "imx386_mipi_raw"
#define SENSOR_DRVNAME_IMX300_MIPI_RAW          "imx300_mipi_raw"
#define SENSOR_DRVNAME_IMX386_MIPI_MONO         "imx386_mipi_mono"
#define SENSOR_DRVNAME_IMX362_MIPI_RAW          "imx362_mipi_raw"
#define SENSOR_DRVNAME_IMX338_MIPI_RAW          "imx338_mipi_raw"
#define SENSOR_DRVNAME_IMX376_MIPI_RAW          "imx376_mipi_raw"
#define SENSOR_DRVNAME_IMX318_MIPI_RAW          "imx318_mipi_raw"
#define SENSOR_DRVNAME_IMX319_MIPI_RAW          "imx319_mipi_raw"
#define SENSOR_DRVNAME_IMX377_MIPI_RAW          "imx377_mipi_raw"
#define SENSOR_DRVNAME_IMX278_MIPI_RAW          "imx278_mipi_raw"
#define SENSOR_DRVNAME_IMX258_MIPI_RAW          "imx258_mipi_raw"
#define SENSOR_DRVNAME_IMX258_MIPI_MONO         "imx258_mipi_mono"
#define SENSOR_DRVNAME_IMX230_MIPI_RAW          "imx230_mipi_raw"
#define SENSOR_DRVNAME_IMX220_MIPI_RAW          "imx220_mipi_raw"
#define SENSOR_DRVNAME_IMX219_MIPI_RAW          "imx219_mipi_raw"
#define SENSOR_DRVNAME_IMX214_MIPI_MONO         "imx214_mipi_mono"
#define SENSOR_DRVNAME_IMX214_MIPI_RAW          "imx214_mipi_raw"
#define SENSOR_DRVNAME_IMX179_MIPI_RAW          "imx179_mipi_raw"
#define SENSOR_DRVNAME_IMX178_MIPI_RAW          "imx178_mipi_raw"
#define SENSOR_DRVNAME_IMX135_MIPI_RAW          "imx135_mipi_raw"
#define SENSOR_DRVNAME_IMX132_MIPI_RAW          "imx132_mipi_raw"
#define SENSOR_DRVNAME_IMX119_MIPI_RAW          "imx119_mipi_raw"
#define SENSOR_DRVNAME_IMX105_MIPI_RAW          "imx105_mipi_raw"
#define SENSOR_DRVNAME_IMX091_MIPI_RAW          "imx091_mipi_raw"
#define SENSOR_DRVNAME_IMX073_MIPI_RAW          "imx073_mipi_raw"
#define SENSOR_DRVNAME_IMX766_MIPI_RAW          "imx766_mipi_raw"
/*OV*/
#define SENSOR_DRVNAME_OV23850_MIPI_RAW         "ov23850_mipi_raw"
#define SENSOR_DRVNAME_OV16880_MIPI_RAW         "ov16880_mipi_raw"
#define SENSOR_DRVNAME_OV16885_MIPI_RAW         "ov16885_mipi_raw"
#define SENSOR_DRVNAME_OV16825_MIPI_RAW         "ov16825_mipi_raw"
#define SENSOR_DRVNAME_OV13855_MIPI_RAW         "ov13855_mipi_raw"
#define SENSOR_DRVNAME_OV13870_MIPI_RAW         "ov13870_mipi_raw"
#define SENSOR_DRVNAME_OV13855_MIPI_RAW         "ov13855_mipi_raw"
#define SENSOR_DRVNAME_OV13855MAIN2_MIPI_RAW    "ov13855main2_mipi_raw"
#define SENSOR_DRVNAME_OV13850_MIPI_RAW         "ov13850_mipi_raw"
#define SENSOR_DRVNAME_OV12A10_MIPI_RAW         "ov12a10_mipi_raw"
#define SENSOR_DRVNAME_OV12830_MIPI_RAW         "ov12830_mipi_raw"
#define SENSOR_DRVNAME_OV9760_MIPI_RAW          "ov9760_mipi_raw"
#define SENSOR_DRVNAME_OV9740_MIPI_YUV          "ov9740_mipi_yuv"
#define SENSOR_DRVNAME_0V9726_RAW               "ov9726_raw"
#define SENSOR_DRVNAME_OV9726_MIPI_RAW          "ov9726_mipi_raw"
#define SENSOR_DRVNAME_OV8865_MIPI_RAW          "ov8865_mipi_raw"
#define SENSOR_DRVNAME_OV8858_MIPI_RAW          "ov8858_mipi_raw"
#define SENSOR_DRVNAME_OV8858S_MIPI_RAW         "ov8858s_mipi_raw"
#define SENSOR_DRVNAME_OV8856_MIPI_RAW          "ov8856_mipi_raw"
#define SENSOR_DRVNAME_OV8830_RAW               "ov8830_raw"
#define SENSOR_DRVNAME_OV8825_MIPI_RAW          "ov8825_mipi_raw"
#define SENSOR_DRVNAME_OV7675_YUV               "ov7675_yuv"
#define SENSOR_DRVNAME_OV5693_MIPI_RAW          "ov5693_mipi_raw"
#define SENSOR_DRVNAME_OV5670_MIPI_RAW          "ov5670_mipi_raw"
#define SENSOR_DRVNAME_OV5670_MIPI_RAW_2        "ov5670_mipi_raw_2"
#define SENSOR_DRVNAME_OV2281_MIPI_RAW          "ov2281_mipi_raw"
#define SENSOR_DRVNAME_OV5675_MIPI_RAW          "ov5675mipiraw"
#define SENSOR_DRVNAME_OV5671_MIPI_RAW          "ov5671_mipi_raw"
#define SENSOR_DRVNAME_OV5647MIPI_RAW           "ov5647_mipi_raw"
#define SENSOR_DRVNAME_OV5645_MIPI_RAW          "ov5645_mipi_raw"
#define SENSOR_DRVNAME_OV5645_MIPI_YUV          "ov5645_mipi_yuv"
#define SENSOR_DRVNAME_OV5650MIPI_RAW           "ov5650_mipi_raw"
#define SENSOR_DRVNAME_OV5650_RAW               "ov5650_raw"
#define SENSOR_DRVNAME_OV5648_MIPI_RAW          "ov5648_mipi_raw"
#define SENSOR_DRVNAME_OV5647_RAW               "ov5647_raw"
#define SENSOR_DRVNAME_OV5642_RAW               "ov5642_raw"
#define SENSOR_DRVNAME_OV5642_MIPI_YUV          "ov5642_mipi_yuv"
#define SENSOR_DRVNAME_OV5642_MIPI_RGB          "ov5642_mipi_rgb"
#define SENSOR_DRVNAME_OV5642_MIPI_JPG          "ov5642_mipi_jpg"
#define SENSOR_DRVNAME_OV5642_YUV               "ov5642_yuv"
#define SENSOR_DRVNAME_OV5642_YUV_SWI2C         "ov5642_yuv_swi2c"
#define SENSOR_DRVNAME_OV4688_MIPI_RAW          "ov4688_mipi_raw"
#define SENSOR_DRVNAME_OV3640_RAW               "ov3640_raw"
#define SENSOR_DRVNAME_OV3640_YUV               "ov3640_yuv"
#define SENSOR_DRVNAME_OV2724_MIPI_RAW          "ov2724_mipi_raw"
#define SENSOR_DRVNAME_OV2722_MIPI_RAW          "ov2722_mipi_raw"
#define SENSOR_DRVNAME_OV2680_MIPI_RAW          "ov2680_mipi_raw"
#define SENSOR_DRVNAME_OV2659_YUV               "ov2659_yuv"
#define SENSOR_DRVNAME_OV2655_YUV               "ov2655_yuv"
#define SENSOR_DRVNAME_OV2650_RAW               "ov265x_raw"
#define SENSOR_DRVNAME_OV20880_MIPI_RAW         "ov20880_mipi_raw"
#define SENSOR_DRVNAME_OV13B10_MIPI_RAW         "ov13b10_mipi_raw"
#define SENSOR_DRVNAME_OV05A20_MIPI_RAW         "ov05a20_mipi_raw"
/*S5K*/
#define SENSOR_DRVNAME_S5KJD1_MIPI_RAW        "s5kjd1_mipi_raw"
#define SENSOR_DRVNAME_S5K2LQSX_MIPI_RAW        "s5k2lqsx_mipi_raw"
#define SENSOR_DRVNAME_S5K4H7_MIPI_RAW          "s5k4h7_mipi_raw"
#define SENSOR_DRVNAME_S5K4H7YXSUB_MIPI_RAW     "s5k4h7yxsub_mipi_raw"
#define SENSOR_DRVNAME_S5K3P8SP_MIPI_RAW        "s5k3p8sp_mipi_raw"
#define SENSOR_DRVNAME_S5K2T7SP_MIPI_RAW        "s5k2t7sp_mipi_raw"
#define SENSOR_DRVNAME_S5K2T7SP_MIPI_RAW_5M     "s5k2t7sp_mipi_raw_5m"
#define SENSOR_DRVNAME_S5K3P8SX_MIPI_RAW        "s5k3p8sx_mipi_raw"
#define SENSOR_DRVNAME_S5K2L7_MIPI_RAW          "s5k2l7_mipi_raw"
#define SENSOR_DRVNAME_S5K3L6_MIPI_RAW          "s5k3l6_mipi_raw"
#define SENSOR_DRVNAME_S5K3L8_MIPI_RAW          "s5k3l8_mipi_raw"
#define SENSOR_DRVNAME_S5K3M3_MIPI_RAW          "s5k3m3_mipi_raw"
#define SENSOR_DRVNAME_S5K3M5SX_MIPI_RAW        "s5k3m5sx_mipi_raw"
#define SENSOR_DRVNAME_S5K2X8_MIPI_RAW          "s5k2x8_mipi_raw"
#define SENSOR_DRVNAME_S5K2P7_MIPI_RAW          "s5k2p7_mipi_raw"
#define SENSOR_DRVNAME_S5K2P8_MIPI_RAW          "s5k2p8_mipi_raw"
#define SENSOR_DRVNAME_S5K3P3SX_MIPI_RAW        "s5k3p3sx_mipi_raw"
#define SENSOR_DRVNAME_S5K3P3_MIPI_RAW          "s5k3p3_mipi_raw"
#define SENSOR_DRVNAME_S5K3P8_MIPI_RAW          "s5k3p8_mipi_raw"
#define SENSOR_DRVNAME_S5K3M2_MIPI_RAW          "s5k3m2_mipi_raw"
#define SENSOR_DRVNAME_S5K4E6_MIPI_RAW          "s5k4e6_mipi_raw"
#define SENSOR_DRVNAME_S5K3H2YX_MIPI_RAW        "s5k3h2yx_mipi_raw"
#define SENSOR_DRVNAME_S5K3H7Y_MIPI_RAW         "s5k3h7y_mipi_raw"
#define SENSOR_DRVNAME_S5K4H5YC_MIPI_RAW        "s5k4h5yc_mipi_raw"
#define SENSOR_DRVNAME_S5K4E1GA_MIPI_RAW        "s5k4e1ga_mipi_raw"
#define SENSOR_DRVNAME_S5K4ECGX_MIPI_YUV        "s5k4ecgx_mipi_yuv"
#define SENSOR_DRVNAME_S5K5CAGX_YUV             "s5k5cagx_yuv"
#define SENSOR_DRVNAME_S5K4H5YX_2LANE_MIPI_RAW  "s5k4h5yx_2lane_mipi_raw"
#define SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW        "s5k5e2ya_mipi_raw"
#define SENSOR_DRVNAME_S5K8AAYX_MIPI_YUV        "s5k8aayx_mipi_yuv"
#define SENSOR_DRVNAME_S5K8AAYX_YUV             "s5k8aayx_yuv"
#define SENSOR_DRVNAME_S5K5E8YX_MIPI_RAW        "s5k5e8yx_mipi_raw"
#define SENSOR_DRVNAME_S5K5E8YXREAR2_MIPI_RAW   "s5k5e8yxrear2_mipi_raw"
#define SENSOR_DRVNAME_S5K5E9_MIPI_RAW          "s5k5e9_mipi_raw"
#define SENSOR_DRVNAME_S5KHM2SP_MIPI_RAW        "s5khm2sp_mipi_raw"
/*HI*/
#define SENSOR_DRVNAME_HI841_MIPI_RAW           "hi841_mipi_raw"
#define SENSOR_DRVNAME_HI707_YUV                "hi707_yuv"
#define SENSOR_DRVNAME_HI704_YUV                "hi704_yuv"
#define SENSOR_DRVNAME_HI556_MIPI_RAW           "hi556_mipi_raw"
#define SENSOR_DRVNAME_HI551_MIPI_RAW           "hi551_mipi_raw"
#define SENSOR_DRVNAME_HI553_MIPI_RAW           "hi553_mipi_raw"
#define SENSOR_DRVNAME_HI545_MIPI_RAW           "hi545_mipi_raw"
#define SENSOR_DRVNAME_HI542_RAW                "hi542_raw"
#define SENSOR_DRVNAME_HI542MIPI_RAW            "hi542_mipi_raw"
#define SENSOR_DRVNAME_HI544_MIPI_RAW           "hi544_mipi_raw"
#define SENSOR_DRVNAME_HI253_YUV                "hi253_yuv"
#define SENSOR_DRVNAME_HI191_MIPI_RAW           "hi191_mipi_raw"
/*MT*/
#define SENSOR_DRVNAME_MT9P012_RAW              "mt9p012_raw"
#define SENSOR_DRVNAME_MT9P015_RAW              "mt9p015_raw"
#define SENSOR_DRVNAME_MT9P017_RAW              "mt9p017_raw"
#define SENSOR_DRVNAME_MT9P017_MIPI_RAW         "mt9p017_mipi_raw"
#define SENSOR_DRVNAME_MT9D115_MIPI_RAW         "mt9d115_mipi_raw"
#define SENSOR_DRVNAME_MT9V114_YUV              "mt9v114_yuv"
#define SENSOR_DRVNAME_MT9V115_YUV              "mt9v115_yuv"
#define SENSOR_DRVNAME_MT9T113_YUV              "mt9t113_yuv"
#define SENSOR_DRVNAME_MT9V113_YUV              "mt9v113_yuv"
#define SENSOR_DRVNAME_MT9T113_MIPI_YUV         "mt9t113_mipi_yuv"
/*GC*/
#define SENSOR_DRVNAME_GC02M0_MIPI_RAW          "gc02m0_mipi_raw"
#define SENSOR_DRVNAME_GC5035_MIPI_RAW          "gc5035_mipi_raw"
#define SENSOR_DRVNAME_GC2375_MIPI_RAW          "gc2375_mipi_raw"
#define SENSOR_DRVNAME_GC2375H_MIPI_RAW         "gc2375h_mipi_raw"
#define SENSOR_DRVNAME_GC2375SUB_MIPI_RAW       "gc2375sub_mipi_raw"
#define SENSOR_DRVNAME_GC2365_MIPI_RAW          "gc2365_mipi_raw"
#define SENSOR_DRVNAME_GC2366_MIPI_RAW          "gc2366_mipi_raw"
#define SENSOR_DRVNAME_GC2035_YUV               "gc2035_yuv"
#define SENSOR_DRVNAME_GC2235_RAW               "gc2235_raw"
#define SENSOR_DRVNAME_GC2355_MIPI_RAW          "gc2355_mipi_raw"
#define SENSOR_DRVNAME_GC2355_RAW               "gc2355_raw"
#define SENSOR_DRVNAME_GC0330_YUV               "gc0330_yuv"
#define SENSOR_DRVNAME_GC0329_YUV               "gc0329_yuv"
#define SENSOR_DRVNAME_GC2145_MIPI_YUV          "gc2145_mipi_yuv"
#define SENSOR_DRVNAME_GC0310_MIPI_YUV          "gc0310_mipi_yuv"
#define SENSOR_DRVNAME_GC0310_YUV               "gc0310_yuv"
#define SENSOR_DRVNAME_GC0312_YUV               "gc0312_yuv"
#define SENSOR_DRVNAME_GC0313MIPI_YUV           "gc0313_mipi_yuv"
#define SENSOR_DRVNAME_GC8C34_MIPI_RAW          "gc8c34_mipi_raw"
#define SENSOR_DRVNAME_GC8034_MIPI_RAW          "gc8034_mipi_raw"
/*SP*/
#define SENSOR_DRVNAME_SP0A19_YUV               "sp0a19_yuv"
#define SENSOR_DRVNAME_SP2518_YUV               "sp2518_yuv"
#define SENSOR_DRVNAME_SP2509_MIPI_RAW          "sp2509_mipi_raw"
#define SENSOR_DRVNAME_SP250A_MIPI_RAW          "sp250a_mipi_raw"
/*A*/
#define SENSOR_DRVNAME_A5141_MIPI_RAW           "a5141_mipi_raw"
#define SENSOR_DRVNAME_A5142_MIPI_RAW           "a5142_mipi_raw"
/*HM*/
#define SENSOR_DRVNAME_HM3451_RAW               "hm3451_raw"
/*AR*/
#define SENSOR_DRVNAME_AR0833_MIPI_RAW          "ar0833_mipi_raw"
/*SIV*/
#define SENSOR_DRVNAME_SIV121D_YUV              "siv121d_yuv"
#define SENSOR_DRVNAME_SIV120B_YUV              "siv120b_yuv"
/*PAS (PixArt Image)*/
#define SENSOR_DRVNAME_PAS6180_SERIAL_YUV       "pas6180_serial_yuv"
/*Panasoic*/
#define SENSOR_DRVNAME_MN34152_MIPI_RAW         "mn34152_mipi_raw"
/*Toshiba*/
#define SENSOR_DRVNAME_T4KA7_MIPI_RAW           "t4ka7_mipi_raw"
/*Others*/
#define SENSOR_DRVNAME_SHARP3D_MIPI_YUV         "sharp3d_mipi_yuv"
#define SENSOR_DRVNAME_T8EV5_YUV                "t8ev5_yuv"
/*Test*/
#define SENSOR_DRVNAME_IMX135_MIPI_RAW_5MP      "imx135_mipi_raw_5mp"
#define SENSOR_DRVNAME_IMX135_MIPI_RAW_8MP      "imx135_mipi_raw_8mp"
#define SENSOR_DRVNAME_OV13870_MIPI_RAW_5MP     "ov13870_mipi_raw_5mp"
#define SENSOR_DRVNAME_OV8856_MIPI_RAW_5MP      "ov8856_mipi_raw_5mp"

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define S5KGW1_SENSOR_ID                        0x0971
#define S5KGH1_SENSOR_ID                        0x0881
#define S5K3M5_SENSOR_ID                        0x30D5
//#define S5K3M5SX_SENSOR_ID                      0x30D6
#define GC8054_SENSOR_ID                        0x8054
#define GC8054F_SENSOR_ID                       0x8055
#define HI846_SENSOR_ID                         0x0846
#define GC2375H_SENSOR_ID                       0x2375
#define OV02A10_SENSOR_ID                       0x2509
#define GC02M0F_SENSOR_ID                       0x02D1
#define SENSOR_DRVNAME_S5KGW1_MIPI_RAW          "s5kgw1_mipi_raw"
#define SENSOR_DRVNAME_S5KGH1_MIPI_RAW          "s5kgh1_mipi_raw"
#define SENSOR_DRVNAME_S5K3M5_MIPI_RAW          "s5k3m5_mipi_raw"
#define SENSOR_DRVNAME_S5K3M5SX_MIPI_RAW        "s5k3m5sx_mipi_raw"
#define SENSOR_DRVNAME_GC8054_MIPI_RAW          "gc8054_mipi_raw"
#define SENSOR_DRVNAME_GC8054F_MIPI_RAW         "gc8054f_mipi_raw"
#define SENSOR_DRVNAME_HI846_MIPI_RAW           "hi846_mipi_raw"
#define SENSOR_DRVNAME_GC2375H_MIPI_RAW         "gc2375h_mipi_raw"
#define SENSOR_DRVNAME_OV02A10_MIPI_MONO        "ov02a10_mipi_mono"
#define SENSOR_DRVNAME_GC02M0_MIPI_MONO         "gc02m0_mipi_mono"
#define SENSOR_DRVNAME_GC02M0F_MIPI_MONO        "gc02m0f_mipi_mono"
#define GC02M0_SENSOR_ID0                       0x02d0
#define GC02M0_SENSOR_ID2                       0x02d2
#define SENSOR_DRVNAME_GC02M0B_MIPI_MONO0       "gc02m0b_mipi_mono0"
#define SENSOR_DRVNAME_GC02M0B_MIPI_MONO1       "gc02m0b_mipi_mono1"
#define SENSOR_DRVNAME_GC02M0B_MIPI_MONO2       "gc02m0b_mipi_mono2"
#define GC02M1_SENSOR_ID0                       0x02e0
#define GC02M1_SENSOR_ID1                       0x02e1
#define GC02M1_SENSOR_ID2                       0x02e2
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO0       "gc02m1b_mipi_mono0"
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO1       "gc02m1b_mipi_mono1"
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO2       "gc02m1b_mipi_mono2"
#define GM1ST_SENSOR_ID                         0xF8D1
#define SENSOR_DRVNAME_GM1ST_MIPI_RAW           "gm1st_mipi_raw"
#define IMX471_SENSOR_ID                        0x20000
#define SENSOR_DRVNAME_IMX471_MIPI_RAW          "imx471_mipi_raw"

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define OV02A10_MONO_SENSOR_ID                  0x2509
#define OV48B_SENSOR_ID                         0x564842
#define GC02K0_SENSOR_ID                        0x2385
#define OV16A10_SENSOR_ID                       0x561641
#define SENSOR_DRVNAME_GC02M0_MIPI_RAW          "gc02m0_mipi_raw"
#define IMX616_SENSOR_ID                     0x0616
#define SENSOR_DRVNAME_GC02M0B_MIPI_MONO     "gc02m0b_mipi_mono"
#define GC02M1B_SENSOR_ID                     0x02e0
#define GC02M1B_SENSOR_ID1                    0x02e1
#define S5KGW3_SENSOR_ID                      0x0973
#define OV32A_SENSOR_ID                       0x563241
#define IMX481_SENSOR_ID                        0x0481
#define IMX319_SENSOR_ID                        0x0319
#define S5K3M5SX_SENSOR_ID                      0x30D5
#define OV02B10_SENSOR_ID                       0x002B
#define SENSOR_DRVNAME_IMX481_MIPI_RAW          "imx481_mipi_raw"
#define SENSOR_DRVNAME_IMX319_MIPI_RAW          "imx319_mipi_raw"
#define SENSOR_DRVNAME_GC02M0_MIPI_RAW          "gc02m0_mipi_raw"
#define SENSOR_DRVNAME_IMX686_MIPI_RAW          "imx686_mipi_raw"
#define SENSOR_DRVNAME_IMX616_MIPI_RAW          "imx616_mipi_raw"
#define SENSOR_DRVNAME_OV48B_MIPI_RAW           "ov48b_mipi_raw"
#define SENSOR_DRVNAME_GC8054_MIPI_RAW          "gc8054_mipi_raw"
#define SENSOR_DRVNAME_GC02M0B_MIPI_MONO1       "gc02m0b_mipi_mono1"
#define SENSOR_DRVNAME_GC02K0B_MIPI_MONO        "gc02k0b_mipi_mono"
#define SENSOR_DRVNAME_OV16A10_MIPI_RAW         "ov16a10_mipi_raw"
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO        "gc02m1b_mipi_mono"
#define SENSOR_DRVNAME_S5KGW3_MIPI_RAW          "s5kgw3_mipi_raw"
#define SENSOR_DRVNAME_OV32A_MIPI_RAW           "ov32a_mipi_raw"
#define SENSOR_DRVNAME_OV02B10_MIPI_RAW         "ov02b10_mipi_raw"

#define SENSOR_DRVNAME_GC02K0_MIPI_RAW          "gc02k0_mipi_raw"
#endif
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define PASCALD_TRULY_MAIN_OV13B10_SENSOR_ID               0x560D43
#define PASCALD_QTECH_MAIN_OV13B10_SENSOR_ID               0x560D44
#define PASCALD_HLT_FRONT_GC5035_SENSOR_ID                 0x5036
#define PASCALD_SHENGTAI_FRONT_GC5035_SENSOR_ID            0x5037
#define PASCALD_SHENGTAI2_FRONT_GC5035_SENSOR_ID           0x5038
#define PASCALD_HLT_FRONT_S5K4H7_SENSOR_ID                 0x487B
#define PASCALD_HLT_DEPTH_GC02K0B_SENSOR_ID                0x2385
#define PASCALD_HLT_DEPTH_GC02M1B_MIPI_SENSOR_ID           0x02E1
#define PASCALD_CXT_DEPTH_GC02M1B_SENSOR_ID                0x02E3
#define PASCALD_LHYX_DEPTH_OV02B1B_SENSOR_ID               0x002D
#define SENSOR_DRVNAME_PASCALD_TRULY_MAIN_OV13B10          "pascald_truly_main_ov13b10"
#define SENSOR_DRVNAME_PASCALD_QTECH_MAIN_OV13B10          "pascald_qtech_main_ov13b10"
#define SENSOR_DRVNAME_PASCALD_HLT_FRONT_GC5035            "pascald_hlt_front_gc5035"
#define SENSOR_DRVNAME_PASCALD_SHENGTAI_FRONT_GC5035       "pascald_shengtai_front_gc5035"
#define SENSOR_DRVNAME_PASCALD_SHENGTAI2_FRONT_GC5035      "pascald_shengtai2_front_gc5035"
#define SENSOR_DRVNAME_PASCALD_HLT_FRONT_S5K4H7            "pascald_hlt_front_s5k4h7"
#define SENSOR_DRVNAME_PASCALD_HLT_DEPTH_GC02K0B           "pascald_hlt_depth_gc02k0b"
#define SENSOR_DRVNAME_PASCALD_HLT_DEPTH_GC02M1B           "pascald_hlt_depth_gc02m1b"
#define SENSOR_DRVNAME_PASCALD_CXT_DEPTH_GC02M1B           "pascald_cxt_depth_gc02m1b"
#define SENSOR_DRVNAME_PASCALD_LHYX_DEPTH_OV02B1B          "pascald_lhyx_depth_ov02b1b"
#define PASCAL_HLT_MONO_GC02M1B_MIPI_SENSOR_ID             0x02E2
#define PASCAL_SHENGTAI_MONO1_OV02B1B_SENSOR_ID            0x002b
#define PASCAL_LHYX_MONO1_OV02B1B_SENSOR_ID                0x002c
#define PASCAL_CXT_MONO1_GC02M1B_SENSOR_ID                 0x02E4
#define PASCAL_CXT_MONO_OV02B1B_SENSOR_ID                  0x002E
#define PASCAL_SHENGTAI_WIDE_OV8856_SENSOR_ID              0x885A
#define PASCAL_SUNNY_WIDE_OV8856_SENSOR_ID                 0x885B
#define SENSOR_DRVNAME_PASCAL_HLT_MONO_GC02M1B             "pascal_hlt_mono_gc02m1b"
#define SENSOR_DRVNAME_PASCAL_SHENGTAI_MONO1_OV02B1B       "pascal_shengtai_mono1_ov02b1b"
#define SENSOR_DRVNAME_PASCAL_LHYX_MONO1_OV02B1B           "pascal_lhyx_mono1_ov02b1b"
#define SENSOR_DRVNAME_PASCAL_CXT_MONO1_GC02M1B            "pascal_cxt_mono1_gc02m1b"
#define SENSOR_DRVNAME_PASCAL_CXT_MONO_OV02B1B             "pascal_cxt_mono_ov02b1b"
#define SENSOR_DRVNAME_PASCAL_SHENGTAI_WIDE_OV8856         "pascal_shengtai_wide_ov8856"
#define SENSOR_DRVNAME_PASCAL_SUNNY_WIDE_OV8856            "pascal_sunny_wide_ov8856"
#define PASCALE_HLT_MACRO_OV02B10_SENSOR_ID                0x002F
#define SENSOR_DRVNAME_PASCALE_HLT_MACRO_OV02B10           "pascale_hlt_macro_ov02b10"
#define OV02B1B_MIPI_SENSOR_ID                             0x002B
#define GC02M1B_MIPI_SENSOR_ID                             0x02E0
#endif //OPLUS_FEATURE_CAMERA_COMMON

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define ZHAOYUN_QTECH_MAIN_OV13B10_SENSOR_ID               0x560D45 /*0x560D42+3*/
#define SENSOR_DRVNAME_ZHAOYUN_QTECH_MAIN_OV13B10          "zhaoyun_qtech_main_ov13b10"
#define ZHAOYUNLITE_QTECH_MAIN_OV13B10_SENSOR_ID               0x560D46 /*0x560D42+4*/
#define SENSOR_DRVNAME_ZHAOYUNLITE_QTECH_MAIN_OV13B10          "zhaoyunlite_qtech_main_ov13b10"
#define ZHAOYUN_SHINETECH_MONO_GC02M1B_SENSOR_ID           0x02E7  /*0x02E0 + 7*/
#define SENSOR_DRVNAME_ZHAOYUN_SHINETECH_MONO_GC02M1B      "zhaoyun_shinetech_mono_gc02m1b"
#define ZHAOYUN_OPTICS_FRONT_GC5035_SENSOR_ID           0x5035  /*0x5035*/
#define SENSOR_DRVNAME_ZHAOYUN_OPTICS_FRONT_GC5035      "zhaoyun_optics_front_gc5035"
#define ZHAOYUNLITE_QTECH_MAIN_S5K3L6_SENSOR_ID            0x30CA /*0x30C6 + 4*/
#define SENSOR_DRVNAME_ZHAOYUNLITE_QTECH_MAIN_S5K3L6       "zhaoyunlite_qtech_main_s5k3l6"
#define ZHAOYUN_SHINETECH_FRONT_OV08D10_SENSOR_ID          0x56084700
#define SENSOR_DRVNAME_ZHAOYUN_SHINETECH_FRONT_OV08D10     "zhaoyun_shinetech_front_ov08d10"
#define ZHAOYUN_SHINETECH_MAIN_S5KJN103_SENSOR_ID          0x38E2
#define SENSOR_DRVNAME_ZHAOYUN_SHINETECH_MAIN_S5KJN103     "zhaoyun_shinetech_main_s5kjn103"
#define ZHAOYUN_SHINETECH_MONO_GC02M1B_50M_SENSOR_ID           0x02E9  /*0x02E0 + 9*/
#define SENSOR_DRVNAME_ZHAOYUN_SHINETECH_MONO_GC02M1B_50M      "zhaoyun_shinetech_mono_gc02m1b_50m"
#define ZHAOYUN_SHINETECH_MAIN_S5K3L6_SENSOR_ID            0x30C9 /*0x30C6 + 3*/
#define SENSOR_DRVNAME_ZHAOYUN_SHINETECH_MAIN_S5K3L6       "zhaoyun_shinetech_main_s5k3l6"
#define ZHAOYUN_PROJECT                                    12
#endif //OPLUS_FEATURE_CAMERA_COMMON
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define LIMU_QTECH_MAIN_OV13B10_SENSOR_ID               0x560D49 /*0x560D42+7*/
#define SENSOR_DRVNAME_LIMU_QTECH_MAIN_OV13B10          "limu_qtech_main_ov13b10"
#define LIMU_OPTICS_FRONT_GC5035_SENSOR_ID              0x5036  /*0x5035 + 1*/
#define SENSOR_DRVNAME_LIMU_OPTICS_FRONT_GC5035         "limu_optics_front_gc5035"
#define LIMU_VGA_SP0A09_SENSOR_ID                       0x0a09
#define SENSOR_DRVNAME_LIMU_VGA_SP0A09                  "limu_vga_sp0a09"
#define LIMU_SHINETECH_MAIN_S5KJN103_SENSOR_ID          0x38E5 /*0x0x38E1 + 4*/
#define SENSOR_DRVNAME_LIMU_SHINETECH_MAIN_S5KJN103     "limu_shinetech_main_s5kjn103"
#define LIMU_SHINETECH_MAIN_IMX355_SENSOR_ID            0x355 /*0x355*/
#define SENSOR_DRVNAME_LIMU_SHINETECH_MAIN_IMX355       "limu_shinetech_main_imx355"
#define LIMU_OPTICS_VGA_GC030A_SENSOR_ID                0x030a /*0x030a*/
#define SENSOR_DRVNAME_LIMU_OPTICS_VGA_GC030A           "limu_optics_vga_gc030a"
#define LIMU_PROJECT                                    10
#endif //OPLUS_FEATURE_CAMERA_COMMON

#ifdef OPLUS_FEATURE_CAMERA_COMMON
/* Parker-A. These sensor IDs should match with those in kd_imgsensor.h in user space */
#define PARKERA_QTECH_MAIN_OV13B10_SENSOR_ID               0x560D48 /*0x560D42+6*/
#define SENSOR_DRVNAME_PARKERA_QTECH_MAIN_OV13B10          "parkera_qtech_main_ov13b10"
#define PARKERA_QTECH_FRONT_IMX355_SENSOR_ID               0x035B /*0x0355 + 6*/
#define SENSOR_DRVNAME_PARKERA_QTECH_FRONT_IMX355          "parkera_qtech_front_imx355"
#define PARKERA_SHINETECH_FRONT_OV08D10_SENSOR_ID          0x5608470A /*0x56084700+A*/
#define SENSOR_DRVNAME_PARKERA_SHINETECH_FRONT_OV08D10     "parkera_shinetech_front_ov08d10"
#define PARKERA_SHINETECH_MONO_GC02M1B_SENSOR_ID           0x02EC  /*0x02E0 + C*/
#define SENSOR_DRVNAME_PARKERA_SHINETECH_MONO_GC02M1B      "parkera_shinetech_mono_gc02m1b"
#define PARKERA_SHINETECH_MONO_OV02B1B_SENSOR_ID           0x002A/*0x2B - 1*/
#define SENSOR_DRVNAME_PARKERA_SHINETECH_MONO_OV02B1B      "parkera_shinetech_mono_ov02b1b"
#define PARKERA_HOLITECH_MACRO_GC02M1_SENSOR_ID            0x02E6/*0x02e0 + 6*/
#define SENSOR_DRVNAME_PARKERA_HOLITECH_MACRO_GC02M1       "parkera_holitech_macro_gc02m1"
#define PARKERA_SHINETECH_MACRO_OV02B10_SENSOR_ID          0x0029/*0x2B - 2*/
#define SENSOR_DRVNAME_PARKERA_SHINETECH_MACRO_OV02B10     "parkera_shinetech_macro_ov02b10"
#define PARKERA_HOLITECH_FRONT_S5K4H7_SENSOR_ID            0x487E/*0x487B + 3*/
#define SENSOR_DRVNAME_PARKERA_HOLITECH_FRONT_S5K4H7       "parkera_holitech_front_s5k4h7"
#define PARKERA_SHINETECH_MAIN_S5KJN103_SENSOR_ID          0x38E1
#define SENSOR_DRVNAME_PARKERA_SHINETECH_MAIN_S5KJN103     "parkera_shinetech_main_s5kjn103"
#define PARKERA_QTECH_MAIN_OV13B2A_SENSOR_ID               0x560D47 /*0x560D42+5*/
#define SENSOR_DRVNAME_PARKERA_QTECH_MAIN_OV13B2A          "parkera_qtech_main_ov13b2a"
#define PARKERA_SHINETECH_MONO_GC02M1B_50M_SENSOR_ID       0x02EB  /*0x02E0 + B*/
#define SENSOR_DRVNAME_PARKERA_SHINETECH_MONO_GC02M1B_50M  "parkera_shinetech_mono_gc02m1b_50m"
#define PARKERA_SHINETECH_MONO_OV02B1B_50M_SENSOR_ID       0x0028  /*0x2B - 3*/
#define SENSOR_DRVNAME_PARKERA_SHINETECH_MONO_OV02B1B_50M  "parkera_shinetech_mono_ov02b1b_50m"

/* Parker-B. These sensor IDs should match with those in kd_imgsensor.h in user space */
#define PARKERB_SHINETECH_MAIN_S5KJN103_SENSOR_ID          0x38E3  /*0x38E1 + 2*/
#define SENSOR_DRVNAME_PARKERB_SHINETECH_MAIN_S5KJN103     "parkerb_shinetech_main_s5kjn103"
#define PARKERB_SUNNY_FRONT_S5K3P9SP_SENSOR_ID             0x310C /*0x310B +1 */
#define SENSOR_DRVNAME_PARKERB_SUNNY_FRONT_S5K3P9SP        "parkerb_sunny_front_s5k3p9sp"
#define PARKERB_SHINETECH_MONO_GC02M1B_SENSOR_ID           0x02EF  /*0x02E0 + 15*/
#define SENSOR_DRVNAME_PARKERB_SHINETECH_MONO_GC02M1B      "parkerb_shinetech_mono_gc02m1b"
#define PARKERB_HLT_MICRO_GC02M1_SENSOR_ID                 0x02F0  /*0x02E0 + 16*/
#define SENSOR_DRVNAME_PARKERB_HLT_MICRO_GC02M1            "parkerb_hlt_micro_gc02m1"
#define PARKERB_SHINETECH_MACRO_OV02B10_SENSOR_ID          0x0032  /*0x002B + 7 */
#define SENSOR_DRVNAME_PARKERB_SHINETECH_MACRO_OV02B10     "parkerb_shinetech_macro_ov02b10"

/* These sensor IDs are used in sensor driver, used in probing. Hence, should not be changed */
#define PARKERA_QTECH_MAIN_OV13B10_SENSOR_ID_ORIGINAL               0x560D45 /*0x560D42+3*/
#define PARKERA_QTECH_FRONT_IMX355_SENSOR_ID_ORIGINAL               0x0357 /*0x0355 + 2*/
#define PARKERA_SHINETECH_MONO_GC02M1B_SENSOR_ID_ORIGINAL           0x02E7  /*0x02E0 + 7*/
#define PARKERA_SHINETECH_MONO_OV02B1B_SENSOR_ID_ORIGINAL           0x002A /*0x2B - 1*/
#define PARKERA_HOLITECH_MACRO_GC02M1_SENSOR_ID_ORIGINAL            0x02E6 /*0x02E0 + 6*/
#define PARKERA_SHINETECH_MACRO_OV02B10_SENSOR_ID_ORIGINAL          0x0029 /*0x2B - 2*/
#define PARKERA_HOLITECH_FRONT_S5K4H7_SENSOR_ID_ORIGINAL            0x487E /*0x487B + 3*/
#define PARKERA_SHINETECH_FRONT_OV08D10_SENSOR_ID_ORIGINAL          0x56084700
#define PARKERA_SHINETECH_MAIN_S5KJN103_SENSOR_ID_ORIGINAL          0x38E1
#define PARKERA_QTECH_MAIN_OV13B2A_SENSOR_ID_ORIGINAL               0x560D46 /*0x560D43+3*/
#define PARKERA_SHINETECH_MONO_GC02M1B_50M_SENSOR_ID_ORIGINAL       0x02E9 /*0x02E0 + 9*/
#define PARKERA_SHINETECH_MONO_OV02B1B_50M_SENSOR_ID_ORIGINAL       0x0028 /*0x2B - 3*/

#define PARKERB_SHINETECH_MAIN_S5KJN103_SENSOR_ID_ORIGINAL          0x38E2 /*0x38E1 + 1*/
#define PARKERB_SUNNY_FRONT_S5K3P9SP_SENSOR_ID_ORIGINAL             0x310B
#define PARKERB_SHINETECH_MONO_GC02M1B_SENSOR_ID_ORIGINAL           0x02EF /*0x02E0 + 15*/
#define PARKERB_HLT_MICRO_GC02M1_SENSOR_ID_ORIGINAL                 0x02F0 /*0x02E0 + 16*/
#define PARKERB_SHINETECH_MACRO_OV02B10_SENSOR_ID_ORIGINAL          0x0030 /*0x002B + 5 */
#endif //OPLUS_FEATURE_CAMERA_COMMON

#define SENSOR_DRVNAME_S5KGD1SP_MIPI_RAW        "s5kgd1sp_mipi_raw"
#define SENSOR_DRVNAME_HI846_MIPI_RAW           "hi846_mipi_raw"
#define SENSOR_DRVNAME_GC02M0_MIPI_RAW          "gc02m0_mipi_raw"
#define SENSOR_DRVNAME_OV02A10_MIPI_MONO        "ov02a10_mipi_mono"
#define SENSOR_DRVNAME_IMX686_MIPI_RAW          "imx686_mipi_raw"
#define SENSOR_DRVNAME_IMX616_MIPI_RAW          "imx616_mipi_raw"
#define SENSOR_DRVNAME_OV48B_MIPI_RAW           "ov48b_mipi_raw"
#define SENSOR_DRVNAME_GC8054_MIPI_RAW          "gc8054_mipi_raw"
#define SENSOR_DRVNAME_GC02M0B_MIPI_MONO        "gc02m0b_mipi_mono"
#define SENSOR_DRVNAME_GC02M0B_MIPI_MONO1       "gc02m0b_mipi_mono1"
#define SENSOR_DRVNAME_GC02M0B_MIPI_MONO2       "gc02m0b_mipi_mono2"
#define SENSOR_DRVNAME_GC02K0B_MIPI_MONO        "gc02k0b_mipi_mono"
#define SENSOR_DRVNAME_OV16A10_MIPI_RAW         "ov16a10_mipi_raw"
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO        "gc02m1b_mipi_mono"
#define SENSOR_DRVNAME_OV48C_MIPI_RAW           "ov48c_mipi_raw"
#define SENSOR_DRVNAME_IMX355_MIPI_RAW_LAFITE    "imx355_mipi_raw_lafite"
#define SENSOR_DRVNAME_OV02B10_MIPI_RAW         "ov02b10_mipi_raw"

#define OV64B_SENSOR_ID_20645                         0x5665
#define SENSOR_DRVNAME_OV64B_MIPI_RAW_20645           "ov64b_mipi_raw_20645"
#define OV02B10_SENSOR_ID_20645                       0x0033
#define SENSOR_DRVNAME_OV02B10_MIPI_RAW_20645         "ov02b10_mipi_raw_20645"
#define HI846_SENSOR_ID_20645                         0x849
#define SENSOR_DRVNAME_HI846_MIPI_RAW_20645           "hi846_mipi_raw_20645"
#define OV32A1Q_SENSOR_ID_20645                       0x563244
#define SENSOR_DRVNAME_OV32A1Q_MIPI_RAW_20645         "ov32a1q_mipi_raw_20645"

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define OV64B_SENSOR_ID                         0x5664
#define SENSOR_DRVNAME_OV64B_MIPI_RAW           "ov64b_mipi_raw"
#define IMX615_SENSOR_ID                              0x0615
#define SENSOR_DRVNAME_IMX615_MIPI_RAW                "imx615_mipi_raw"
/*20201224 add for 20817 sensor porting*/
#define SENSOR_ID_OFFSET_20817                        0xf000
#define IMX766_SENSOR_ID_20817                        0x0766
#define SENSOR_DRVNAME_IMX766_MIPI_RAW_20817          "imx766_mipi_raw20817"
#define IMX615_SENSOR_ID_20817                        (0x0615 + 0xf000)  //add offset to avoid tuning params mapping error
#define SENSOR_DRVNAME_IMX615_MIPI_RAW_20817          "imx615_mipi_raw20817"
#define IMX355_SENSOR_ID_20817                        (0x0355 + 0xf000)  //add offset to avoid tuning params mapping error
#define SENSOR_DRVNAME_IMX355_MIPI_RAW_20817          "imx355_mipi_raw20817"
#define GC02M1B_SENSOR_ID_20817                       (0x02e0 + 0xf000)  //add offset to avoid tuning params mapping error
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO_20817        "gc02m1b_mipi_mono20817"

#define SENSOR_ID_OFFSET_21881                        0xd000
#define IMX766_SENSOR_ID_21881                        (0x0766 + 0xd000)  //add offset to avoid tuning params mapping error
#define SENSOR_DRVNAME_IMX766_MIPI_RAW_21881          "imx766_mipi_raw21881"
#define IMX615_SENSOR_ID_21881                        (0x0615 + 0xd000)  //add offset to avoid tuning params mapping error
#define SENSOR_DRVNAME_IMX615_MIPI_RAW_21881          "imx615_mipi_raw21881"
#define IMX355_SENSOR_ID_21881                        (0x0355 + 0xd000)  //add offset to avoid tuning params mapping error
#define SENSOR_DRVNAME_IMX355_MIPI_RAW_21881          "imx355_mipi_raw21881"
#define GC02M1B_SENSOR_ID_21881                       (0x02e0 + 0xd000)  //add offset to avoid tuning params mapping error
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO_21881        "gc02m1b_mipi_mono21881"
#endif
#define OV64B_SENSOR_ID                               0x5664
#define SENSOR_DRVNAME_OV64B_MIPI_RAW                 "ov64b_mipi_raw"
#define OV48B_SENSOR_ID                         0x564842
#define SENSOR_DRVNAME_OV48B_MIPI_RAW           "ov48b_mipi_raw"
#define OV32A_SENSOR_ID                      0x563241
#define SENSOR_DRVNAME_OV32A_MIPI_RAW      "ov32a_mipi_raw"
#define HI846_SENSOR_ID                         0x0846
#define SENSOR_DRVNAME_HI846_MIPI_RAW           "hi846_mipi_raw"



#define OV02B1B_SENSOR_ID                       0x002c
#define ATHENSC_IMX471_SENSOR_ID                0x472
#define IMX471_SENSOR_ID                        0x20000
#define OV02A1B_SENSOR_ID                       0x250a
#define ATHENSB_GC02M1B_SENSOR_ID                       0x02E1
#define GC02M1B_SENSOR_ID_20630                       0x02e0
#define HI846_SENSOR_ID_ANNA                        0x084B
#define HI846_SENSOR_ID_20633                         0x0846
#define IMX471_SENSOR_ID_ANNA                       0x20002
#define ATHENSC_IMX471_SENSOR_ID                      0x472
#define IMX471_SENSOR_ID_20633                        0x471

#define GC02M1_SENSOR_ID_APOLLOT                      0x02ee
#define SENSOR_DRVNAME_GC02M1_MIPI_RAW_APOLLOT        "gc02m1_mipi_raw_apollot"

#define SENSOR_DRVNAME_IMX471_MIPI_RAW_ANNA         "imx471_mipi_raw_anna"
#define SENSOR_DRVNAME_HI846_MIPI_RAW_ANNA          "hi846_mipi_raw_anna"
#define SENSOR_DRVNAME_IMX471_MIPI_RAW1          "imx471_mipi_raw1"
#define SENSOR_DRVNAME_OV02A1B_MIPI_MONO         "ov02a1b_mipi_mono"
#define SENSOR_DRVNAME_OV02B1B_MIPI_MONO        "ov02b1b_mipi_mono"
#define SENSOR_DRVNAME_IMX471_MIPI_RAW          "imx471_mipi_raw"
#define SENSOR_DRVNAME_GC02K0_MIPI_RAW          "gc02k0_mipi_raw"
#define SENSOR_DRVNAME_OV32A_MIPI_RAW           "ov32a_mipi_raw"
#define SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW        "s5kgm1st_mipi_raw"
#define S5KGW1_SENSOR_ID                        0x0971
#define OV32A1Q_SENSOR_ID_20630                       0x563241
#define ATHENSB_OV32A1Q_SENSOR_ID                       0x563243
#define HI846_SENSOR_ID_20630                       0x0846
#define ATHENSB_HI846_SENSOR_ID                       0x0847
#define GC02K_SENSOR_ID_20630                       0x2395
#define ATHENSB_GC02K_SENSOR_ID                       0x2396
#define GC02M1B_SENSOR_ID_20630                       0x02e0
#define ATHENSB_GC02M1B_SENSOR_ID                       0x02E1
#define ATHENSC_HI846_SENSOR_ID                       0x0848
#define OV02B1B_SENSOR_ID1                      0x002C
#define OV02B10_SENSOR_ID_ANNA                      0x0032
#if defined(MT6853)
#define S5KGM1ST_SENSOR_ID                      0xF8D1
#endif
#define S5KGM1ST_SENSOR_ID_20633                       0xF8D1
#define S5KGM1ST_SENSOR_ID_ANNA                     0xF8D4 // (0xF8D1 + 3)
#define S5KGW3_SENSOR_ID_ANNA                       0x730B
#define S5KGW3_SENSOR_ID_20630                        0x7309
#define ATHENSC_S5KGM1ST_SENSOR_ID                    0xF8D2
#define S5KGW3_SENSOR_ID                        0x0973
#define GC02M1B_SENSOR_ID1                      0x02e1
#define IMX766_SENSOR_ID_MOSS                        0x0766
#define SENSOR_DRVNAME_IMX766_MOSSMIPI_RAW          "imx766_mossmipi_raw"
#define IMX471_SENSOR_ID_MOSS                       0x20000
#define SENSOR_DRVNAME_IMX471_MOSSMIPI_RAW         "imx471_mossmipi_raw"
#define IMX355_SENSOR_ID_MOSS                        (0x0355 + 1)
#define SENSOR_DRVNAME_IMX355_MOSSMIPI_RAW          "imx355_mossmipi_raw"
#define GC02M1_SENSOR_ID_MOSS                         0x02e0
#define SENSOR_DRVNAME_GC02M1_MOSSMIPI_RAW           "gc02m1_mossmipi_raw"
#define S5KGM1ST_SENSOR_ID_MOSSA                         0xF8D1
#define SENSOR_DRVNAME_S5KGM1ST_MOSSAMIPI_RAW           "s5kgm1st_mossamipi_raw"
#define GC02M1_SENSOR_ID_MOSSA                         (0x02e0+0xa)
#define SENSOR_DRVNAME_GC02M1_MOSSAMIPI_RAW           "gc02m1_mossamipi_raw"

#define IMX766_SENSOR_ID22693                         (0x0766 + 0x3)//0x0769
#define SENSOR_DRVNAME_IMX766_MIPI_RAW22693           "imx766_mipi_raw22693"
#define S5KHM6SP_SENSOR_ID22693                       (0x1ad6)
#define SENSOR_DRVNAME_S5KHM6SP_MIPI_RAW22693         "s5khm6sp_mipi_raw22693"
#define S5K3P9SP_SENSOR_ID22693                       (0x3109 + 0x6)//0x310f
#define SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW22693         "s5k3p9sp_mipi_raw22693"
#define IMX355_SENSOR_ID22693                         (0x0355 + 0x7)//0x035c
#define SENSOR_DRVNAME_IMX355_MIPI_RAW22693           "imx355_mipi_raw22693"
#define GC02M1_SENSOR_ID22693                         (0x02e0 + 0xd)//0x02ed
#define SENSOR_DRVNAME_GC02M1_MIPI_RAW22693           "gc02m1_mipi_raw22693"

/*Cam.Drv add for tacoo 20230922*/
#define IMX882_SENSOR_ID23687                       0x8202
#define SENSOR_DRVNAME_IMX882_MIPI_RAW23687         "imx882_mipi_raw23687"
#define IMX890_SENSOR_ID23689                       0x0890
#define SENSOR_DRVNAME_IMX890_MIPI_RAW23689         "imx890_mipi_raw23689"
#define HI1634Q_SENSOR_ID23687                      0x1634
#define SENSOR_DRVNAME_HI1634Q_MIPI_RAW23687       "hi1634q_mipi_raw23687"
#define HI846W_SENSOR_ID23687                       0x0856
#define SENSOR_DRVNAME_HI846W_MIPI_RAW23687         "hi846w_mipi_raw23687"
#define GC02M1_SENSOR_ID23687                       (0x02e0 + 0xf)//0x02ef
#define SENSOR_DRVNAME_GC02M1_MIPI_RAW23687         "gc02m1_mipi_raw23687"
#define IMX882PD_SENSOR_ID23687                     (0x8202 + 0x1) // 0x8203
#define SENSOR_DRVNAME_IMX882PD_MIPI_RAW23687       "imx882pd_mipi_raw23687"
/*Cam.Drv add for tacoo 20230922*/

/*Cam.Drv add for tacoo-b 20231129*/
#define S5KJN1_SENSOR_ID23707                         0x38E9 /*0x0x38E1 + 8*/
#define SENSOR_DRVNAME_S5KJN1_MIPI_RAW23707           "s5kjn1_mipi_raw23707"
#define GC02M1_SENSOR_ID23707                         (0x02e0 + 0x13)//0x02f3
#define SENSOR_DRVNAME_GC02M1_MIPI_RAW23707           "gc02m1_mipi_raw23707"
#define HI1634Q_SENSOR_ID23707                        0x1635
#define SENSOR_DRVNAME_HI1634Q_MIPI_RAW23707          "hi1634q_mipi_raw23707"
/*Cam.Drv add for tacoo-b 20231129*/

/*Cam.Drv add for chengdu 20221213*/
#define S5KHP3SP_SENSOR_ID22629                       (0x1b73)
#define SENSOR_DRVNAME_S5KHP3SP_MIPI_RAW22629         "s5khp3sp_mipi_raw22629"
#define IMX615_SENSOR_ID22629                         (0x0615 + 0xe)//0x0623
#define SENSOR_DRVNAME_IMX615_MIPI_RAW22629           "imx615_mipi_raw22629"
#define IMX355_SENSOR_ID22629                         (0x0355 + 0x8)//0x035d
#define SENSOR_DRVNAME_IMX355_MIPI_RAW22629           "imx355_mipi_raw22629"
#define GC02M1_SENSOR_ID22629                         (0x02e0 + 0xe)//0x02ee
#define SENSOR_DRVNAME_GC02M1_MIPI_RAW22629           "gc02m1_mipi_raw22629"
#define OVA0B4_SENSOR_ID22633                         (0xa042 + 0x1)//0xa043
#define SENSOR_DRVNAME_OVA0B4_MIPI_RAW22633           "ova0b4_mipi_raw22633"
#define S5K3P9SP_SENSOR_ID22633                       0x3121
#define SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW22633         "s5k3p9sp_mipi_raw22633"
#define SC202CS_SENSOR_ID22633                        (0xeb52 + 0x1)//0xeb53
#define SENSOR_DRVNAME_SC202CS_MIPI_MONO22633         "sc202cs_mipi_mono22633"
#define OV02B1B_SENSOR_ID22633                        (0x002b + 0xc)//0X0037
#define SENSOR_DRVNAME_OV02B1B_MIPI_MONO22633         "ov02b1b_mipi_mono22633"
/*Cam.Drv add for chengdu 20221213*/

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define S5KGM1ST_SENSOR_ID                           0xF8D1
#define S5K3L6_SENSOR_ID                             0x30C6
#define S5KJN103_SENSOR_ID                           0x38E1
#define S5K3P9SP_SENSOR_ID                           0x3109
#define SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW      "s5kgm1st_mipi_raw"
#define SENSOR_DRVNAME_S5K3L6_MIPI_RAW        "s5k3l6_mipi_raw"
#define SENSOR_DRVNAME_IMX355_MIPI_RAW        "imx355_mipi_raw"
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO      "gc02m1b_mipi_mono"
#define SENSOR_DRVNAME_S5KJN103_MIPI_RAW      "s5kjn103_mipi_raw"
#define SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW      "s5k3p9sp_mipi_raw"
#endif /*OPLUS_FEATURE_CAMERA_COMMON*/

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define S5K3L6_SENSOR_ID_ALICERT                             0x30C9//(0x30C6+3)
#define IMX355_SENSOR_ID_ALICERT                             0x035A//(0x0355+5)
#define GC02M1B_SENSOR_ID_ALICERT                            0x02E9//(0x02E0+9)
#define SENSOR_DRVNAME_S5K3L6_MIPI_RAW_ALICERT        "s5k3l6_mipi_raw_alicert"
#define SENSOR_DRVNAME_IMX355_MIPI_RAW_ALICERT        "imx355_mipi_raw_alicert"
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO_ALICERT      "gc02m1b_mipi_mono_alicert"
#endif /*OPLUS_FEATURE_CAMERA_COMMON*/

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define S5KJN103_SENSOR_ID_ALICER                            0x38E2//(0x38E1+1)
#define IMX355_SENSOR_ID_ALICER                              0x0358//(0x0355+3)
#define GC02M1B_SENSOR_ID_ALICER                             0x02E8//(0x02E0+8)
#define S5KGM1ST_SENSOR_ID_ALICER                            0xF8D5//(0xF8D1+4)
#define SENSOR_DRVNAME_S5KJN103_MIPI_RAW_ALICER       "s5kjn103_mipi_raw_alicer"
#define SENSOR_DRVNAME_IMX355_MIPI_RAW_ALICER         "imx355_mipi_raw_alicer"
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO_ALICER       "gc02m1b_mipi_mono_alicer"
#define SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_ALICER       "s5kgm1st_mipi_raw_alicer"
#endif /*OPLUS_FEATURE_CAMERA_COMMON*/

#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO0       "gc02m1b_mipi_mono0"
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO1       "gc02m1b_mipi_mono1"
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO2       "gc02m1b_mipi_mono2"

#define IMX581_SENSOR_ID                          0x0581
#define GC02M1_SENSOR_ID_DUFU                    (0x02e0+4)
#define SENSOR_DRVNAME_IMX581_MIPI_RAW           "imx581_mipi_raw"
#define SENSOR_DRVNAME_GC02M1_MIPI_RAW           "gc02m1_mipi_raw"

#define CHOPIN_OFFSET                               0x5
#define OV64B_SENSOR_ID_212A1                      (0x5664 + CHOPIN_OFFSET)
#define SENSOR_DRVNAME_OV64B_MIPI_RAW212A1         "ov64b_mipi_raw212a1"
#define IMX615_SENSOR_ID_212A1                     (0x0615 + CHOPIN_OFFSET)
#define SENSOR_DRVNAME_IMX615_MIPI_RAW212A1        "imx615_mipi_raw212a1"
#define IMX355_SENSOR_ID_212A1                     (0x0355 + CHOPIN_OFFSET)
#define SENSOR_DRVNAME_IMX355_MIPI_RAW212A1         "imx355_mipi_raw212a1"
#define GC02M1_SENSOR_ID_212A1                     (0x02e0 + CHOPIN_OFFSET)
#define SENSOR_DRVNAME_GC02M1_MIPI_RAW212A1        "gc02m1_mipi_raw212a1"

#define IVAN_OFFSET                                 0xA
#define OV64B_SENSOR_ID_21851                      (0x5664 + IVAN_OFFSET)
#define SENSOR_DRVNAME_OV64B_MIPI_RAW21851         "ov64b_mipi_raw21851"
#define IMX471_SENSOR_ID_21851                     (0x20000 + IVAN_OFFSET)
#define SENSOR_DRVNAME_IMX471_MIPI_RAW21851        "imx471_mipi_raw21851"
#define IMX355_SENSOR_ID_21851                     (0x0355 + IVAN_OFFSET)
#define SENSOR_DRVNAME_IMX355_MIPI_RAW21851         "imx355_mipi_raw21851"
#define GC02M1_SENSOR_ID_21851                     (0x02e0 + IVAN_OFFSET)
#define SENSOR_DRVNAME_GC02M1_MIPI_RAW21851        "gc02m1_mipi_raw21851"

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define S5KGW3_SENSOR_ID_20631                        0x731A
#define OV32A1Q_SENSOR_ID_20631                       0x563245
#define HI846_SENSOR_ID_20631                         0x084D
#define GC02K_SENSOR_ID_20631                         0x2398
#define GC02M1B_SENSOR_ID_20631                       0x02e8

#define S5KGM1ST_SENSOR_ID_20633c                      0xF8D6
#define IMX471_SENSOR_ID_20633c                        0x0476
#define HI846_SENSOR_ID_20633c                         0x084E

#define SENSOR_DRVNAME_S5KGW3_MIPI_RAW_ATHENSB      "s5kgw3_mipi_raw_athensb"
#define SENSOR_DRVNAME_OV32A1Q_MIPI_RAW_ATHENSB      "ov32a1q_mipi_raw_athensb"
#define SENSOR_DRVNAME_HI846_MIPI_RAW_ATHENSB        "hi846_mipi_raw_athensb"
#define SENSOR_DRVNAME_GC02K_MIPI_RAW_ATHENSB        "gc02k_mipi_raw_athensb"
#define SENSOR_DRVNAME_GC02M1B_MIPI_RAW_ATHENSB      "gc02m1b_mipi_raw_athensb"

#define SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_ATHENSC      "s5kgm1st_mipi_raw_athensc"
#define SENSOR_DRVNAME_IMX471_MIPI_RAW_ATHENSC        "imx471_mipi_raw_athensc"
#define SENSOR_DRVNAME_HI846_MIPI_RAW_ATHENSC         "hi846_mipi_raw_athensc"

#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define IMX682_SENSOR_ID_20615                        0x0683 /*0x0682+1*/
#define SENSOR_DRVNAME_IMX682_MIPI_RAW_20615          "imx682_mipi_raw_20615"
#define IMX471_SENSOR_ID_20615                        0x20001 /*0x20000+1*/
#define SENSOR_DRVNAME_IMX471_MIPI_RAW_20615          "imx471_mipi_raw_20615"
#define HI846_SENSOR_ID_20615                         0x0848 /*0x0846+2*/
#define SENSOR_DRVNAME_HI846_MIPI_RAW_20615           "hi846_mipi_raw_20615"
#define OV02B10_SENSOR_ID_20615                       0x0032 /*0x002B+7*/
#define SENSOR_DRVNAME_OV02B10_MIPI_RAW_20615         "ov02b10_mipi_raw_20615"

#define IMX682_SENSOR_ID                        0x0682
#define SENSOR_DRVNAME_IMX682_MIPI_RAW          "imx682_mipi_raw"
#define SALAA_QTECH_MAIN_S5KGM1SP_SENSOR_ID           0x08D5
#define SENSOR_DRVNAME_SALAA_QTECH_MAIN_S5KGM1SP     "salaa_qtech_main_s5kgm1sp"
#define SALA_OV02B10_SENSOR_ID                       0x0034
#define SENSOR_DRVNAME_SALA_OV02B10_MIPI_RAW         "sala_ov02b10_mipi_raw"
#define SALA_WIDE_OV8856_SENSOR_ID                    0x8856
#define SENSOR_DRVNAME_SALA_WIDE_OV8856               "sala_wide_ov8856"
#define SENSOR_DRVNAME_S5KGM1SP_MIPI_RAW        "s5kgm1sp_mipi_raw"

#define SENSOR_DRVNAME_IMX471_MIPI_RAW2          "imx471_mipi_raw2"
#define SENSOR_DRVNAME_OV8856_MIPI_RAW2          "ov8856_mipi_raw2"

#define OV64B_SENSOR_ID_20730                    0x566b /*0x5664+6*/
#define SENSOR_DRVNAME_OV64B_MIPI_RAW_20730      "ov64b_mipi_raw_20730"
#define IMX471_SENSOR_ID1                        0x0471
#define IMX471_SENSOR_ID2                        0x0473
#define IMX471_SENSOR_ID2_20730                  0x0474 /*0x471+3*/
#define SENSOR_DRVNAME_IMX471_MIPI_RAW2_20730    "imx471_mipi_raw2_20730"
#define OV8856_SENSOR_ID2                        0x885C
#define OV8856_SENSOR_ID2_20730                  0x885f/*0x885a+5*/
#define SENSOR_DRVNAME_OV8856_MIPI_RAW2_20730    "ov8856_mipi_raw2_20730"
#define GC02M1B_SENSOR_ID_20730                  0x02f1 /*0x02e0+17*/
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO_20730   "gc02m1b_mipi_mono_20730"
#define GC02M1_SENSOR_ID_20730                   0x02f2 /*0x02e0+18*/
#define SENSOR_DRVNAME_GC02M1_MIPI_RAW_20730     "gc02m1_mipi_raw_20730"
#define GC2375H_SENSOR_ID_19661                  0x2376
#define SENSOR_DRVNAME_GC2375H_MIPI_RAW_19661    "gc2375h_mipi_raw_19661"

#define OV64B40_SENSOR_ID_21690                       0x566442
#define SENSOR_DRVNAME_OV64B40_MIPI_RAW_21690         "ov64b40_mipi_raw_21690"
#define S5KJN1_SENSOR_ID_21690                        0x38E7 /*0x38E1+6*/
#define SENSOR_DRVNAME_S5KJN1_MIPI_RAW_21690          "s5kjn1_mipi_raw_21690"
#define HI1634Q_SENSOR_ID_21690                        0x1634
#define SENSOR_DRVNAME_HI1634Q_MIPI_RAW_21690         "hi1634q_mipi_raw_21690"
#define S5K3P9SP_SENSOR_ID_21690                      0x310D
#define SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW_21690        "s5k3p9sp_mipi_raw_21690"
#define GC02M1HLT_SENSOR_ID_21690                     0x02f1
#define SENSOR_DRVNAME_GC02M1HLT_MIPI_RAW_21690       "gc02m1hlt_mipi_raw_21690"
#define GC02M1SHINE_SENSOR_ID_21690                   0x02e3
#define SENSOR_DRVNAME_GC02M1SHINE_MIPI_RAW_21690     "gc02m1shine_mipi_raw_21690"
#define OV02B1B_SENSOR_ID_21690                       0x0037 /*0x002c+11*/
#define SENSOR_DRVNAME_OV02B1B_MIPI_MONO_21690        "ov02b1b_mipi_mono_21690"
#define GC02M1BSY_SENSOR_ID_21690                     0x02eb /*0x02e1+10*/
#define SENSOR_DRVNAME_GC02M1BSY_MIPI_RAW_21690       "gc02m1bsy_mipi_raw_21690"
#endif /*OPLUS_FEATURE_CAMERA_COMMON*/

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define OV64B_SENSOR_ID_20619                         0x5667 /*0x5665+2*/
#define SENSOR_DRVNAME_OV64B_MIPI_RAW_20619           "ov64b_mipi_raw_20619"
#define IMX471_SENSOR_ID_20619                        0x20004 /*0x20000+4*/
#define SENSOR_DRVNAME_IMX471_MIPI_RAW_20619          "imx471_mipi_raw_20619"
#define HI846_SENSOR_ID_20619                         0x0849 /*0x0846+2*/
#define SENSOR_DRVNAME_HI846_MIPI_RAW_20619           "hi846_mipi_raw_20619"
#define OV02B10_SENSOR_ID_20619                       0x0038 /*0x002B+13*/
#define SENSOR_DRVNAME_OV02B10_MIPI_RAW_20619         "ov02b10_mipi_raw_20619"
#endif /*OPLUS_FEATURE_CAMERA_COMMON*/

#define CARR_OV13B10_SENSOR_ID                         0x560D43 // (0x560D42 + 1)
#define CARR_S5K3L6_SENSOR_ID                          0x30C7 // (0x30C6 + 1)
#define CARR_IMX355_SENSOR_ID                          0x0359 //(0x355 + 4)
#define CARR_GC02M1B_SENSOR_ID                         0x02e3 // (0x02e0 + 3)
#define CARR_OV02B10_SENSOR_ID                         0x002C // (0x002B + 1)
#define CARR_GC02M1_SENSOR_ID                          0x02e4 // (0x02e0 + 4)
#define SENSOR_DRVNAME_OV13B10_MIPI_RAW_CARR           "ov13b10_mipi_raw_carr"
#define SENSOR_DRVNAME_S5K3L6_MIPI_RAW_CARR            "s5k3l6_mipi_raw_carr"
#define SENSOR_DRVNAME_IMX355_MIPI_RAW_CARR            "imx355_mipi_raw_carr"
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO_CARR          "gc02m1b_mipi_mono_carr"
#define SENSOR_DRVNAME_OV02B10_MIPI_RAW_CARR           "ov02b10_mipi_raw_carr"
#define SENSOR_DRVNAME_GC02M1_MIPI_RAW_CARR            "gc02m1_mipi_raw_carr"
#define S5KGM1ST_SENSOR_ID_ODINA                       0xF8D3 // (0xF8D1 + 1)
#define IMX355_SENSOR_ID_ODINA                         0x357  // (0x355 + 1)
#define GC02M1B_SENSOR_ID_ODINA                        0x02e6 // (0x02e0 + 5)
#define OV02B10_SENSOR_ID_ODINA                        0x002E // (0x002B + 2)
#define SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_ODINA      "s5kgm1st_mipi_raw_odina"
#define SENSOR_DRVNAME_IMX355_MIPI_RAW_ODINA         "imx355_mipi_raw_odina"
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO_ODINA       "gc02m1b_mipi_mono_odina"
#define SENSOR_DRVNAME_OV02B10_MIPI_RAW_ODINA       "ov02b10_mipi_raw_odina"

#define OV64B_SENSOR_ID_21651                         0x5668 /*0x5665+3*/
#define SENSOR_DRVNAME_OV64B_MIPI_RAW_21651           "ov64b_mipi_raw_21651"
#define IMX471_SENSOR_ID_21651                        0x20003 /*0x20000+3*/
#define SENSOR_DRVNAME_IMX471_MIPI_RAW_21651          "imx471_mipi_raw_21651"
#define IMX355_SENSOR_ID_21651                        0x357  // (0x355 + 2)
#define SENSOR_DRVNAME_IMX355_MIPI_RAW_21651          "imx355_mipi_raw_21651"
#define GC02M1_SENSOR_ID_21651                        0x02e6 /*0x02e0+6*/
#define SENSOR_DRVNAME_GC02M1_MIPI_RAW_21651          "gc02m1_mipi_raw_21651"

#ifdef OPLUS_FEATURE_CAMERA_COMMON
/*even*/
#define EVEN_QTECH_MAIN_S5KGM1ST03_SENSOR_ID                   0xF8D4
#define SENSOR_DRVNAME_EVEN_QTECH_MAIN_S5KGM1ST03     "even_qtech_main_s5kgm1st03"
#define EVEN_QTECH_MAIN_OV13B10_SENSOR_ID                      0x560D45
#define SENSOR_DRVNAME_EVEN_QTECH_MAIN_OV13B10        "even_qtech_main_ov13b10"
#define EVEN_HLT_FRONT_S5K4H7_SENSOR_ID                       0x487D
#define SENSOR_DRVNAME_EVEN_HLT_FRONT_S5K4H7         "even_hlt_front_s5k4h7"
#define EVEN_SHENGTAI_FRONT_OV8856_SENSOR_ID                       0x885D
#define SENSOR_DRVNAME_EVEN_SHENGTAI_FRONT_OV8856         "even_shengtai_front_ov8856"
#define EVEN_HLT_DEPTH_GC02M1B_MIPI_SENSOR_ID                      0x02E6
#define SENSOR_DRVNAME_EVEN_HLT_DEPTH_GC02M1B        "even_hlt_depth_gc02m1b"
#define EVEN_SHENGTAI_MACRO_OV02B10_SENSOR_ID                      0x0032
#define SENSOR_DRVNAME_EVEN_SHENGTAI_MACRO_OV02B10        "even_shengtai_macro_ov02b10"
#define EVEN_SHINETECH_MAIN_S5KJN103_SENSOR_ID          0x38E4
#define SENSOR_DRVNAME_EVEN_SHINETECH_MAIN_S5KJN103     "even_shinetech_main_s5kjn103"
#define EVENC_SHENGTAI_FRONT_OV8856_SENSOR_ID                       0x885E
#define SENSOR_DRVNAME_EVENC_SHENGTAI_FRONT_OV8856         "evenc_shengtai_front_ov8856"
#define EVENC_SHINETECH_DEPTH_GC02M1B_SENSOR_ID         0x02EA
#define SENSOR_DRVNAME_EVENC_SHINETECH_DEPTH_GC02M1B    "evenc_shinetech_depth_gc02m1b"
#define EVENC_SHENGTAI_MACRO_OV02B10_SENSOR_ID          0x0035
#define SENSOR_DRVNAME_EVENC_SHENGTAI_MACRO_OV02B10      "evenc_shengtai_macro_ov02b10"
#define OV48B_SENSOR_ID_CHIVAS                       0x564843
#define SENSOR_DRVNAME_OV48B_CHIVASMIPI_RAW          "ov48b_chivasmipi_raw"
#define OV32A_SENSOR_ID_CHIVAS                       0x563242
#define SENSOR_DRVNAME_OV32A_CHIVASMIPI_RAW          "ov32a_chivasmipi_raw"
#define HI846_SENSOR_ID_CHIVAS                       0x084C
#define SENSOR_DRVNAME_HI846_CHIVASMIPI_RAW          "hi846_chivasmipi_raw"
#define GC02K0_SENSOR_ID_CHIVAS                      0x2386
#define SENSOR_DRVNAME_GC02K0_CHIVASMIPI_RAW         "gc02k0_chivasmipi_raw"
#endif /*OPLUS_FEATURE_CAMERA_COMMON*/

/*aladdin*/
#define ALADDIN_OV50D40_SENSOR_ID                  0x565044
#define SENSOR_DRVNAME_ALADDIN_OV50D40              "aladdin_ov50d40"
#define ALADDIN_OV08D10_SENSOR_ID                  0x5608470B /*0x5608470B+B*/
#define SENSOR_DRVNAME_ALADDIN_OV08D10              "aladdin_ov08d10"
#define ALADDIN_OV02B1B_SENSOR_ID                  0x003A/*0x2B + F*/
#define SENSOR_DRVNAME_ALADDIN_OV02B1B             "aladdin_ov02b1b"

/*atom*/
#define ATOM_OV50D40_SENSOR_ID                  0x565045/*0x565045 + 1*/
#define SENSOR_DRVNAME_ATOM_OV50D40              "atom_ov50d40"
#define ATOM_OV02B1B_SENSOR_ID                  0x003B/*0x2B + 0x10*/
#define SENSOR_DRVNAME_ATOM_OV02B1B              "atom_ov02b1b"
#define ATOM_IMX355_SENSOR_ID                   0x35A /*0x355 + 5*/
#define SENSOR_DRVNAME_ATOM_IMX355              "atom_imx355"
#define ATOM_S5K5E9YX04_SENSOR_ID               0x559D /*0x559b + 2*/
#define SENSOR_DRVNAME_ATOM_S5K5E9YX04          "atom_s5k5e9yx04"
#define ATOM_22365_OV02B1B_SENSOR_ID            0x003C/*0x2B + 0x11*/
#define SENSOR_DRVNAME_ATOM_22365_OV02B1B       "atom_22365_ov02b1b"

/*miami*/
#define MIAMI_OV64B_SENSOR_ID                   0x5664
#define SENSOR_DRVNAME_MIAMI_OV64B              "miami_ov64b"
#define MIAMI_AAC2_OV64B_SENSOR_ID              0x5665
#define SENSOR_DRVNAME_MIAMI_AAC2_OV64B         "miami_aac2_ov64b"
#define MIAMI_S5KJN1_SENSOR_ID                  0x38E1
#define SENSOR_DRVNAME_MIAMI_S5KJN1             "miami_s5kjn1"
#define MIAMI_SC800CS_SENSOR_ID                 0xD126
#define SENSOR_DRVNAME_SC800CS_MIPI_RAW_MIAMI   "miami_sc800cs"
#define MIAMI_OV8856_SENSOR_ID                  0x885A
#define SENSOR_DRVNAME_OV8856_MIPI_RAW_MIAMI    "miami_ov8856"
#define MIAMI_GC02M1_SENSOR_ID                  0x2E0
#define SENSOR_DRVNAME_MIAMI_GC02M1             "miami_gc02m1"
#define MIAMI_SP2507_SENSOR_ID                   0x002B
#define SENSOR_DRVNAME_MIAMI_SP2507             "miami_sp2507"

#define ATHENSD_S5KGW3_SENSOR_ID                        0x730A
#define ATHENSD_S5KGW3P1_SENSOR_ID                      0x730C
#define ATHENSD_IMX471_SENSOR_ID                        0x473
#define ATHENSD_HI846_SENSOR_ID                         0x0842
#define ATHENSD_GC02K_SENSOR_ID                         0x2398

#define SENSOR_DRVNAME_S5KGW3_MIPI_RAW_ATHENSD          "s5kgw3_mipi_raw_athensd"
#define SENSOR_DRVNAME_S5KGW3P1_MIPI_RAW_ATHENSD        "s5kgw3p1_mipi_raw_athensd"
#define SENSOR_DRVNAME_IMX471_MIPI_RAW_ATHENSD          "imx471_mipi_raw_athensd"
#define SENSOR_DRVNAME_HI846_MIPI_RAW_ATHENSD           "hi846_mipi_raw_athensd"
#define SENSOR_DRVNAME_GC02K_MIPI_RAW_ATHENSD           "gc02k_mipi_raw_athensd"

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define IMX686_SENSOR_ID_20601                   0x687
#define OV32A1Q_SENSOR_ID_20601                  0x563242
#define HI846_SENSOR_ID_20601                    0x848
#define GC02M1B_SENSOR_ID_20601                  0x02e3
#define GC2385_SENSOR_ID_20601                   0x2395

#define SENSOR_DRVNAME_IMX686_MIPI_RAW_20601      "imx686_mipi_raw_20601"
#define SENSOR_DRVNAME_OV32A1Q_MIPI_RAW_20601     "ov32a1q_mipi_raw_20601"
#define SENSOR_DRVNAME_HI846_MIPI_RAW_20601       "hi846_mipi_raw_20601"
#define SENSOR_DRVNAME_GC02M1B_MIPI_RAW_20601     "gc02m1b_mipi_raw_20601"
#define SENSOR_DRVNAME_GC2385_MIPI_RAW_20601      "gc2385_mipi_raw_20601"
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define SENSOR_ID_OFFSET_BLADE                     0xa
#define HI556_SENSOR_ID_BLADE                      (0x0556 + SENSOR_ID_OFFSET_BLADE)
#define SENSOR_DRVNAME_HI556_MIPI_RAW_BLADE        "hi556_mipi_raw_blade"
#define HI1336_SENSOR_ID_BLADE                     0x01336
#define SENSOR_DRVNAME_HI1336_MIPI_RAW_BLADE       "hi1336_mipi_raw_blade"
#define BF20A1_SENSOR_ID_BLADE                     0x020a1
#define SENSOR_DRVNAME_BF20A1_MIPI_RAW_BLADE       "bf20a1_mipi_raw_blade"
#define HI5021SQT_SENSOR_ID_BLADE                  0x5021
#define SENSOR_DRVNAME_HI5021SQT_MIPI_RAW_BLADE    "hi5021sqt_mipi_raw_blade"
#define S5KJN103_SENSOR_ID_BLADE                   0x38E8//(0x38E1+7)
#define SENSOR_DRVNAME_S5KJN103_MIPI_RAW_BLADE     "s5kjn103_mipi_raw_blade"
#define SC800CS_SENSOR_ID_BLADE                    0xd126
#define SENSOR_DRVNAME_SC800CS_MIPI_RAW_BLADE      "sc800cs_mipi_raw_blade"
#define SC201CS_SENSOR_ID_BLADE                    0xeb2d
#define SENSOR_DRVNAME_SC201CS_MIPI_MONO_BLADE     "sc201cs_mipi_mono_blade"
#define GC030A_SENSOR_ID_BLADE                     0x030a
#define SENSOR_DRVNAME_GC030A_MIPI_RAW_BLADE       "gc030a_mipi_raw_blade"
#define C2599_SENSOR_ID_BLADE                      0x020e
#define SENSOR_DRVNAME_C2599_MIPI_MONO_BLADE       "c2599_mipi_mono_blade"
#define SC201CS1_SENSOR_ID_BLADE                   0xeb2c
#define SENSOR_DRVNAME_SC201CS_MIPI_RAW_BLADE "sc201cs_mipi_raw_blade"
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define S5KGM1ST_SENSOR_ID_APOLLOF                     0xF8D7
#define S5K3P9SP_SENSOR_ID_APOLLOF                     0x310C
#define OV02B10_SENSOR_ID_APOLLOF                      0x0034
#define OV02B1B_SENSOR_ID_APOLLOF                      0x0035
#define OV64B_SENSOR_ID_APOLLOF                        0x5669

#define SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_APOLLOF         "s5kgm1st_mipi_raw_apollof"
#define SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW_APOLLOF         "s5k3p9sp_mipi_raw_apollof"
#define SENSOR_DRVNAME_OV02B10_MIPI_RAW_APOLLOF          "ov02b10_mipi_raw_apollof"
#define SENSOR_DRVNAME_OV02B1B_MIPI_MONO_APOLLOF         "ov02b1b_mipi_mono_apollof"
#define SENSOR_DRVNAME_OV64B_MIPI_RAW_APOLLOF            "ov64b_mipi_raw_apollof"
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define OV64B40_SENSOR_ID_HAWAII                           0x5667  /*0x5664+3*/
#define SENSOR_DRVNAME_OV64B40_MIPI_RAW_HAWAII             "ov64b40_mipi_raw_hawaii"
#define OV08D10_SENSOR_ID_HAWAII                           0x56084701  /*0x56084700+1*/
#define SENSOR_DRVNAME_OV08D10_MIPI_RAW_HAWAII             "ov08d10_mipi_raw_hawaii"
#define OV16A1Q_SENSOR_ID_HAWAII                           0x1641
#define SENSOR_DRVNAME_OV16A1Q_MIPI_RAW_HAWAII             "ov16a1q_mipi_raw_hawaii"
#define OV02B1B_SENSOR_ID_HAWAII                           0x002E
#define SENSOR_DRVNAME_OV02B1B_MIPI_MONO_HAWAII            "ov02b1b_mipi_mono_hawaii"
#endif


#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define S5KGM1ST_SENSOR_ID_APOLLOB                     0xF8D6 // (0xF8D1 + 4)
#define SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_APOLLOB       "s5kgm1st_mipi_raw_apollob"
#define IMX355_SENSOR_ID_APOLLOB                       0x356  // (0x355 + 1)
#define SENSOR_DRVNAME_IMX355_MIPI_RAW_APOLLOB         "imx355_mipi_raw_apollob"
#define GC02M1B_SENSOR_ID_APOLLOB                      0x02e5 // (0x02e0 + 5)
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO_APOLLOB       "gc02m1b_mipi_mono_apollob"
#define OV02B10_SENSOR_ID_APOLLOB                      0x0036 // (0x002B + 9)
#define SENSOR_DRVNAME_OV02B10_MIPI_RAW_APOLLOB        "ov02b10_mipi_raw_apollob"
#define S5K3P9SP_SENSOR_ID_APOLLOB                     0x310A // (0x3109 + 1)
#define SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW_APOLLOB       "s5k3p9sp_mipi_raw_apollob"
#define OV13B10_SENSOR_ID_APOLLOB                      0x560D45 // (0x560D42 + 3)
#define SENSOR_DRVNAME_OV13B10_MIPI_RAW_APOLLOB        "ov13b10_mipi_raw_apollob"
#endif

#define SENSOR_ID_OFFSET_21015                  0xE000
#define IMX709_SENSOR_ID_21015                  (0x0709 + SENSOR_ID_OFFSET_21015)
#define SENSOR_DRVNAME_IMX709_MIPI_RAW_21015    "imx709_mipi_raw21015"
#define IMX766_SENSOR_ID_21015                  (0x0766 + SENSOR_ID_OFFSET_21015)
#define SENSOR_DRVNAME_IMX766_MIPI_RAW_21015    "imx766_mipi_raw21015"
#define IMX355_SENSOR_ID_21015                  (0x0355 + SENSOR_ID_OFFSET_21015)
#define SENSOR_DRVNAME_IMX355_MIPI_RAW_21015    "imx355_mipi_raw21015"
#define OV02B10_SENSOR_ID_21015                 (0x002B + SENSOR_ID_OFFSET_21015)
#define SENSOR_DRVNAME_OV02B10_MIPI_RAW_21015   "ov02b10_mipi_raw21015"
#define IMX709_MONO_SENSOR_ID_21015              0xF709
#define SENSOR_DRVNAME_IMX709_MIPI_MONO_21015    "imx709_mipi_mono21015"
#define OV50A_SENSOR_ID                               (0x5650)
#define IMX709_SENSOR_ID                              (0x709)
#define SENSOR_ID_OFFSET_21127                        (0x10000)
#define OV50A_SENSOR_ID_21127                         (OV50A_SENSOR_ID + SENSOR_ID_OFFSET_21127)
#define SENSOR_DRVNAME_OV50A_MIPI_RAW_21127           "ov50a_mipi_raw_21127"
#define IMX471_SENSOR_ID_21127                        (IMX471_SENSOR_ID + SENSOR_ID_OFFSET_21127) /*0x20000*/
#define SENSOR_DRVNAME_IMX471_MIPI_RAW_21127          "imx471_mipi_raw_21127"
#define IMX709_SENSOR_ID_21127                        (IMX709_SENSOR_ID + SENSOR_ID_OFFSET_21127)
#define SENSOR_DRVNAME_IMX709_MIPI_RAW_21127          "imx709_mipi_raw_21127"
#define GC02M1B_SENSOR_ID_21127                       (GC02M1B_SENSOR_ID + SENSOR_ID_OFFSET_21127)
#define SENSOR_DRVNAME_GC02M1B_MIPI_MONO_21127        "gc02m1b_mipi_mono_21127"
#define GC02M1_SENSOR_ID                              0x02e0
#define GC02M1_SENSOR_ID_21127                        (GC02M1_SENSOR_ID + 1 + SENSOR_ID_OFFSET_21127)
#define SENSOR_DRVNAME_GC02M1_MIPI_RAW_21127          "gc02m1_mipi_raw_21127"
#define SENSOR_ID_OFFSET_21305                      (0x10)
#define IMX766_SENSOR_ID_21305                      (IMX766_SENSOR_ID + SENSOR_ID_OFFSET_21305)
#define SENSOR_DRVNAME_IMX766_MIPI_RAW_21305        "imx766_mipi_raw_21305"
#define IMX615_SENSOR_ID_21305                      (IMX615_SENSOR_ID + SENSOR_ID_OFFSET_21305) /*0x20000*/
#define SENSOR_DRVNAME_IMX615_MIPI_RAW_21305        "imx615_mipi_raw_21305"
#define IMX709_SENSOR_ID_21305                      (IMX709_SENSOR_ID + SENSOR_ID_OFFSET_21305)
#define SENSOR_DRVNAME_IMX709_MIPI_RAW_21305        "imx709_mipi_raw_21305"
#define IMX355_SENSOR_ID_21305                      (IMX355_SENSOR_ID + SENSOR_ID_OFFSET_21305)
#define SENSOR_DRVNAME_IMX355_MIPI_RAW_21305        "imx355_mipi_raw_21305"
#define GC02M1_SENSOR_ID                            0x02e0
#define GC02M1_SENSOR_ID_21305                      (GC02M1_SENSOR_ID + SENSOR_ID_OFFSET_21305)
#define SENSOR_DRVNAME_GC02M1_MIPI_RAW_21305        "gc02m1_mipi_raw_21305"

#define IMX686Q2R_SENSOR_ID                        0X0686
#define SENSOR_DRVNAME_IMX686Q2R_MIPI_RAW          "imx686q2r_mipi_raw"
#define HI846Q2R_SENSOR_ID                         0x0846
#define SENSOR_DRVNAME_HI846Q2R_MIPI_RAW           "hi846q2r_mipi_raw"
#define S5KGD1SPQ2R_SENSOR_ID                      0x0841
#define SENSOR_DRVNAME_S5KGD1SPQ2R_MIPI_RAW        "s5kgd1spq2r_mipi_raw"
#define IMX616Q2R_SENSOR_ID                        0x0616
#define SENSOR_DRVNAME_IMX616Q2R_MIPI_RAW          "imx616q2r_mipi_raw"
#define GC02M0_SENSOR_ID                        0x02d0
#define GC02M0_SENSOR_ID1                       0x02d1
#define SENSOR_DRVNAME_GC02M0B_MIPI_MONO        "gc02m0b_mipi_mono"
#define SENSOR_DRVNAME_GC02M0B_MIPI_MONO1       "gc02m0b_mipi_mono1"

#define S5KGM1ST_SENSOR_ID_20611                0xF8D9
//#define OV16A10_SENSOR_ID_20611                 0x561642
#define S5K3P9SP_SENSOR_ID_20611                0x310B
#define HI846_SENSOR_ID_20611                   0x084A
#define GC02K_SENSOR_ID_20611                   0x2397
#define OV02B1B_SENSOR_ID_20611                 0x0031

#define SENSOR_DRVNAME_S5KGM1ST_MIPI_RAW_20611  "s5kgm1st_mipi_raw_20611"
//#define SENSOR_DRVNAME_OV16A10_MIPI_RAW_20611   "ov16a10_mipi_raw_20611"
#define SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW_20611  "s5k3p9sp_mipi_raw_20611"
#define SENSOR_DRVNAME_HI846_MIPI_RAW_20611     "hi846_mipi_raw_20611"
#define SENSOR_DRVNAME_GC02K_MIPI_RAW_20611     "gc02k_mipi_raw_20611"
#define SENSOR_DRVNAME_OV02B1B_MIPI_RAW_20611   "ov02b1b_mipi_raw_20611"

#define OV02B10_SENSOR_ID_APOLLOW                      0x0032 // (0x002B + 7)
#define OV02B1B_SENSOR_ID_APOLLOW                      0x0033 // (0x002B + 8)
#define OV64B_SENSOR_ID_APOLLOW                        0x5666 // (0x5664 + 2)
#define S5K3P9SP_SENSOR_ID_APOLLOW                     0x310E //(0x310C + 2)
#define SENSOR_DRVNAME_OV02B10_MIPI_RAW_APOLLOW          "ov02b10_mipi_raw_apollow"
#define SENSOR_DRVNAME_OV02B1B_MIPI_MONO_APOLLOW         "ov02b1b_mipi_mono_apollow"
#define SENSOR_DRVNAME_OV64B_MIPI_RAW_APOLLOW            "ov64b_mipi_raw_apollow"
#define SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW_APOLLOW         "s5k3p9sp_mipi_raw_apollow"

//chanel
#define SENSOR_ID_OFFSET_22277                         0x20

#define OV32C_SENSOR_ID22277                         (0x3243 + SENSOR_ID_OFFSET_22277)
#define SENSOR_DRVNAME_OV32C_MIPI_RAW22277           "ov32c_mipi_raw22277"
#define IMX355_SENSOR_ID22277                        (0x0355 + SENSOR_ID_OFFSET_22277)
#define SENSOR_DRVNAME_IMX355_MIPI_RAW22277          "imx355_mipi_raw22277"
#define OV64B_SENSOR_ID22277                        (0x5664 + SENSOR_ID_OFFSET_22277)
#define SENSOR_DRVNAME_OV64B_MIPI_RAW22277          "ov64b_mipi_raw22277"
#define IMX709_SENSOR_ID22277                        (0x0709 + SENSOR_ID_OFFSET_22277)
#define SENSOR_DRVNAME_IMX709_MIPI_RAW22277          "imx709_mipi_raw22277"

//Givenchy
#define SENSOR_ID_OFFSET_23031                       0x21
#define OV64B_SENSOR_ID23031                         (0x5664 + SENSOR_ID_OFFSET_23031)
#define SENSOR_DRVNAME_OV64B_MIPI_RAW23031           "ov64b_mipi_raw23031"
#define OV08D_SENSOR_ID23031                         (0x56084700 + SENSOR_ID_OFFSET_23031)
#define SENSOR_DRVNAME_OV08D_MIPI_RAW23031           "ov08d_mipi_raw23031"
#define GC02M1_SENSOR_ID23031                        (0x02e0 + SENSOR_ID_OFFSET_23031)
#define SENSOR_DRVNAME_GC02M1_MIPI_RAW23031          "gc02m1_mipi_raw23031"

//Burberry
#define SENSOR_ID_OFFSET_23231                       0x22
#define OV32C_SENSOR_ID23231                         (0x3243 + SENSOR_ID_OFFSET_23231)
#define SENSOR_DRVNAME_OV32C_MIPI_RAW23231           "ov32c_mipi_raw23231"
#define IMX355_SENSOR_ID23231                        (0x0355 + SENSOR_ID_OFFSET_23231)
#define SENSOR_DRVNAME_IMX355_MIPI_RAW23231          "imx355_mipi_raw23231"
#define IMX709_SENSOR_ID23231                        (0x0709 + SENSOR_ID_OFFSET_23231)
#define SENSOR_DRVNAME_IMX709_MIPI_RAW23231          "imx709_mipi_raw23231"
#define IMX882_SENSOR_ID23231                        (0x0882 + SENSOR_ID_OFFSET_23231)
#define SENSOR_DRVNAME_IMX882_MIPI_RAW23231          "imx882_mipi_raw23231"
#define IMX882PD_SENSOR_ID23231                      (0x0882 + 0x23)
#define SENSOR_DRVNAME_IMX882PD_MIPI_RAW23231        "imx882pd_mipi_raw23231"


/******************************************************************************
 *
 ******************************************************************************/
void KD_IMGSENSOR_PROFILE_INIT(void);
void KD_IMGSENSOR_PROFILE(char *tag);
void KD_IMGSENSOR_PROFILE_INIT_I2C(void);
void KD_IMGSENSOR_PROFILE_I2C(char *tag, int trans_num);

#define mDELAY(ms)     mdelay(ms)
#define uDELAY(us)       udelay(us)
#endif              /* _KD_IMGSENSOR_H */
