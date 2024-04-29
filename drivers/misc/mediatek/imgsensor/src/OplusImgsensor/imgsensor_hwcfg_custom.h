/*
 * Copyright (C) 2017 MediaTek Inc.
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
#ifndef __IMGSENSOR_HWCFG_CUSTOM_CTRL_H__
#define __IMGSENSOR_HWCFG_CUSTOM_CTRL_H__

#include "imgsensor.h"
#include "imgsensor_sensor_list.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"
#include "mclk/mclk.h"
#include "regulator/regulator.h"
#include "gpio/gpio.h"
#include <soc/oplus/system/oplus_project.h>
#include <soc/oplus/device_info.h>
//#include <linux/of_gpio.h>
#include <linux/gpio.h>

#ifdef CONFIG_LDO_AW37004
#include "../../../aw37004/aw37004.h"
#endif

#define  OPLUS_SUPPORT_CAMERA_NUMMAX        (6)
#define  OPLUS_STEREO_LENGTH_QCOMHEADER     (128)
#define  WRITE_DATA_MAX_LENGTH             (16)
#define  OPLUS_CAMCOMDATA_LENS              (40)
#define  OPLUS_CAMMODULEINFO_LENS           (8)
#define  OPLUS_CAMERASN_LENS                (20)
#define  OPLUS_CAMERASN_LENS_23            (23)
#define  OPLUS_CAMAFCODE_LENS               (8)

#define DEVICE_MANUFACUTRE_NA           "None"
#define DEVICE_MANUFACUTRE_SUNNY        "Sunny"
#define DEVICE_MANUFACUTRE_TRULY        "Truly"
#define DEVICE_MANUFACUTRE_SEMCO        "Semco"
#define DEVICE_MANUFACUTRE_LITEON       "Liteon"
#define DEVICE_MANUFACUTRE_QTECH        "Qtech"
#define DEVICE_MANUFACUTRE_OFILM        "Ofilm"
#define DEVICE_MANUFACUTRE_SHINE        "Shine"
#define DEVICE_MANUFACUTRE_HOLITECH     "Holitech"
#define DEVICE_MANUFACUTRE_AAC          "AAC"

#define IMGSENSOR_MODULE_ID_SUNNY       0x01
#define IMGSENSOR_MODULE_ID_TRULY       0x02
#define IMGSENSOR_MODULE_ID_SEMCO       0x03
#define IMGSENSOR_MODULE_ID_LITEON      0x04
#define IMGSENSOR_MODULE_ID_QTECH       0x05
#define IMGSENSOR_MODULE_ID_OFILM       0x06
#define IMGSENSOR_MODULE_ID_SHINE       0x07
#define IMGSENSOR_MODULE_ID_HOLITECH    0x09
#define IMGSENSOR_MODULE_ID_AAC         0x1F
#define OPLUS_CHECK_IS_SYSTEM_CAM       0xFFFF

#define PMIC_AVDD_VOLTAGE_MV_1100       (1100)
#define PMIC_AVDD_VOLTAGE_MV_1200       (1200)
#define PMIC_AVDD_VOLTAGE_MV_1300       (1300)
#define PMIC_AVDD_VOLTAGE_MV_2800       (2800)
#define PMIC_AVDD_VOLTAGE_MV_2900       (2900)

enum IMGSENSOR_POWER_ACTION_INDEX {
    IMGSENSOR_POWER_ACTION_EVENT_START = 0,
    IMGSENSOR_POWER_ACTION_AFNOSISE_ADD_19551,
    IMGSENSOR_POWER_ACTION_AFNOSISE_DEC_19551,
    IMGSENSOR_POWER_ACTION_GC02AVDD_PULLUP_19301,

    IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX = 81,
    IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX,
    IMGSENSOR_POWER_EXTERNLDO_ENABLE_INDEX,

    IMGSENSOR_POWER_ACTION_EVENT_MAX,
};

enum wl2868c_ldo_num {
    WL2868C_LDO1 = 1,
    WL2868C_LDO2,
    WL2868C_LDO3,
    WL2868C_LDO4,
    WL2868C_LDO5,
    WL2868C_LDO6,
    WL2868C_LDO7,
};

enum FAN_53870_PMIC_STATUS{
    FAN_53870_PMIC_FAILED = -1,
    FAN_53870_PMIC_OFF,
    FAN_53870_PMIC_ON,
};

struct IMGSENSOR_EEPROM_MODULE_INFO {
    u32 i4Sensorid;         /*eeprom IIC Slave addr*/
    u16 i4SlaveAddr;        /*eeprom IIC Slave addr*/
    u16 i4CamInfoAddr[2];   /*2Bytes + 6Bytes*/
    u16 i4CamSNAddr;        /*20Bytes*/
    u16 i4AfSupport;        /*afSupport*/
    u16 i4AFCodeAddr[4];    /*Mac+50cm+100cm+InfDac*/
    char *name;
    char *version;
};

struct CAMERA_DEVICE_INFO {
    u32 i4SensorNum;        /*Sensor Num < 6*/
    struct IMGSENSOR_EEPROM_MODULE_INFO pCamModuleInfo[OPLUS_SUPPORT_CAMERA_NUMMAX];
    u16 i4NormDataLen;
    u16 i4MWDataLen;
    u16 i4MTDataLen;
    u16 i4FrontDataLen;
    u16 i4MWStereoAddr[2];
    u16 i4MTStereoAddr[2];
    u16 i4FrontStereoAddr[2];
    u16 i4DistortionAddr;
    u32 i4CurSensorId;
    enum IMGSENSOR_SENSOR_IDX i4CurSensorIdx;
    enum IMGSENSOR_SENSOR_IDX i4MWDataIdx;  /*2: main+main2   0xFF:Invalid*/
    enum IMGSENSOR_SENSOR_IDX i4MTDataIdx;  /*3: main+sub2*/
    enum IMGSENSOR_SENSOR_IDX i4FrontDataIdx;/*5: sub+main4*/
    u8 camNormdata[OPLUS_SUPPORT_CAMERA_NUMMAX][OPLUS_CAMCOMDATA_LENS];
    u8 stereoMWdata[DUALCAM_CALI_DATA_LENGTH_TOTAL_QCOM];
    u8 stereoMTdata[DUALCAM_CALI_DATA_LENGTH_TOTAL_TELE];
    u8 stereoFrontdata[DUALCAM_CALI_DATA_LENGTH_TOTAL_QCOM];
    u8 distortionParams[CAMERA_DISTORTIONPARAMS_LENGTH];
};


struct IMGSENSOR_HW_POWER_SEQ *Oplusimgsensor_matchhwcfg_power(
        enum   IMGSENSOR_POWER_ACTION_INDEX  pwr_actidx);
struct IMGSENSOR_SENSOR_LIST *Oplusimgsensor_Sensorlist(void);
struct IMGSENSOR_HW_CFG *Oplusimgsensor_Custom_Config(void);
enum IMGSENSOR_RETURN Oplusimgsensor_i2c_init(
        struct IMGSENSOR_SENSOR_INST *psensor_inst);
enum IMGSENSOR_RETURN Oplusimgsensor_ldoenable_power(
        struct IMGSENSOR_HW             *phw,
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status);
enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status);
void Oplusimgsensor_Registdeviceinfo(
        char         *name,
        char         *version,
        kal_uint8    module_id);
void Oplusimgsensor_powerstate_notify(bool val);
#ifdef SENSOR_PLATFORM_5G_H
extern struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20817[];
extern struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20817_EVB2[];
extern struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_20817_EVT[];
extern struct IMGSENSOR_HW_CFG imgsensor_custom_config_20817[];
extern struct IMGSENSOR_HW_CFG imgsensor_custom_config_20817_EVB2[];
extern struct IMGSENSOR_HW_CFG imgsensor_custom_config_20817_EVT[];
extern struct IMGSENSOR_SENSOR_LIST gimgsensor_sensor_list_20817[];

extern struct CAMERA_DEVICE_INFO gImgEepromInfo_20817;

extern struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence_21881[];
extern struct IMGSENSOR_HW_CFG imgsensor_custom_config_21881[];
extern struct IMGSENSOR_SENSOR_LIST gimgsensor_sensor_list_21881[];

extern struct CAMERA_DEVICE_INFO gImgEepromInfo_21881;
#endif
extern struct IMGSENSOR_HW_POWER_SEQ *oplus_sensor_power_on_sequence;
extern struct IMGSENSOR_HW_POWER_SEQ *oplus_sensor_power_sequence;
extern struct IMGSENSOR_HW_CFG *oplus_imgsensor_custom_config;
extern struct IMGSENSOR_SENSOR_LIST *oplus_gimgsensor_sensor_list;
extern struct CAMERA_DEVICE_INFO gImgEepromInfo;
extern void oplus_imgsensor_hwcfg(void);
extern void oplus_imgsensor_delay_set(struct IMGSENSOR_HW_POWER_INFO *ppwr_info, struct IMGSENSOR_HW_POWER_SEQ *ppwr_seq);
#ifdef SENSOR_PLATFORM_5G_A
extern struct IMGSENSOR_HW_POWER_SEQ *oplus_platform_power_sequence;
#endif

#ifdef SENSOR_PLATFORM_4G_20682
extern struct IMGSENSOR_HW_POWER_SEQ *oplus_platform_power_sequence;
#endif

#ifdef SENSOR_PLATFORM_5G_B
extern struct IMGSENSOR_HW_POWER_SEQ *oplus_platform_power_sequence;
#endif

#ifdef CONFIG_LDO_AW37004
extern int aw37004_camera_power_up(int out_iotype);
extern int aw37004_camera_power_down(int out_iotype);
#endif

extern int pmic_ldo_get_type(void);
extern int pmic_ldo_set_voltage_mv(unsigned int ldo_num, int set_mv);
extern int pmic_ldo_set_disable(unsigned int ldo_num);
#ifdef SENSOR_PLATFORM_5G_B
extern struct IMGSENSOR_HW_POWER_SEQ *oplus_platform_power_sequence;
#endif
extern int is_fan53870_pmic(void);
extern int fan53870_cam_ldo_set_voltage(int LDO_NUM, int set_mv);
extern int fan53870_cam_ldo_disable(int LDO_NUM);
extern void pmic_gpio_enable(int pwr_status);

extern void oplus_chg_set_camera_status(bool val);
extern void oplus_chg_set_camera_on(bool val);
extern bool oplus_is_system_camera(unsigned int val);
#ifdef CONFIG_PROJECT_20601
extern int is_check_lcm_status(void);
#endif /* CONFIG_PROJECT_20601 */
#endif
