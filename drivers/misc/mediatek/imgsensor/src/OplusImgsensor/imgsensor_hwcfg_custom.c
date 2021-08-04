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
#include "imgsensor_hwcfg_custom.h"

#define PCB_VERSION_EVB  (3)
#define PCB_VERSION_EVB2 (2)
#define PCB_VERSION_T0   (2)
#define PCB_VERSION_T1   (1)

struct IMGSENSOR_SENSOR_LIST *Oplusimgsensor_Sensorlist(void)
{
    struct IMGSENSOR_SENSOR_LIST *pOplusImglist = gimgsensor_sensor_list;
    pr_debug("Oplusimgsensor_Sensorlist enter:\n");
    if (is_project(20615)) {
        pOplusImglist =  gimgsensor_sensor_list_20615;
        pr_debug("gimgsensor_sensor_list_20615 Selected\n");
    }
#ifdef SENSOR_PLATFORM_5G_H
    if (is_project(20131) || is_project(20133)
        || is_project(20255) || is_project(20257)
        || is_project(20171) || is_project(20172)
        || is_project(20353)) {
        pOplusImglist =  gimgsensor_sensor_list_20131;
        pr_debug("gimgsensor_sensor_list_20131 Selected\n");
    }else if (is_project(20817) || is_project(20827)
              || is_project(20831)) {
        /*20201224 add for 20817 sensor porting*/
        pOplusImglist =  gimgsensor_sensor_list_20817;
        pr_debug("gimgsensor_sensor_list_20817 Selected\n");
    }
#endif
#ifdef SENSOR_PLATFORM_5G_B
if (is_project(20075)||is_project(20076)){
        pOplusImglist =  gimgsensor_sensor_list_20075;
        pr_debug("gimgsensor_sensor_list_20075 Selected\n");
    }
#endif
    return pOplusImglist;
}

struct IMGSENSOR_HW_CFG *Oplusimgsensor_Custom_Config(void)
{
    int pcbVer = 0;
    struct IMGSENSOR_HW_CFG *pOplusImgHWCfg = imgsensor_custom_config;

    if (is_project(20615)) {
        pOplusImgHWCfg = imgsensor_custom_config_20615;
        pr_debug("imgsensor_custom_config_20615 Selected\n");
        return pOplusImgHWCfg;
    } else if (is_project(20817) || is_project(20827)
               || is_project(20831)) {
        /*20210119 add for EVB2*/
        pcbVer = get_PCB_Version();
        pr_debug("Oplusimgsensor_Custom_Config pcbVer: %d\n", pcbVer);
        if (PCB_VERSION_EVB == pcbVer) {
            /*20201224 add for 20817 sensor porting*/
            pOplusImgHWCfg = imgsensor_custom_config_20817;
            pr_debug("imgsensor_custom_config_20817 Selected\n");
        } else if(PCB_VERSION_EVB2 == pcbVer || PCB_VERSION_T0 == pcbVer
                  || PCB_VERSION_T1 == pcbVer) {
            pOplusImgHWCfg = imgsensor_custom_config_20817_EVB2;
            pr_debug("imgsensor_custom_config_20817_EVB2 Selected\n");
        } else {
            /*20210311 add for 20817 sensor porting support EVT*/
            pOplusImgHWCfg = imgsensor_custom_config_20817_EVT;
            pr_debug("imgsensor_custom_config_20817_EVT Selected\n");
        }
    }

    return pOplusImgHWCfg;
#ifdef SENSOR_PLATFORM_5G_H
    if (is_project(20131) || is_project(20133)
        || is_project(20255) || is_project(20257)
        || is_project(20171) || is_project(20172)
        || is_project(20353)) {
        pOplusImgHWCfg = imgsensor_custom_config_20131;
        pr_debug("imgsensor_custom_config_20131 Selected\n");
    }
#endif
#ifdef SENSOR_PLATFORM_5G_B
if (is_project(20075) || is_project(20076)) {
        pOplusImgHWCfg = imgsensor_custom_config_20075;
        pr_info("imgsensor_custom_config_20075 Selected\n");
    }
#endif
    return pOplusImgHWCfg;
}

enum IMGSENSOR_RETURN Oplusimgsensor_i2c_init(
        struct IMGSENSOR_SENSOR_INST *psensor_inst)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    struct IMGSENSOR_HW_CFG *pOplusImgHWCfg = imgsensor_custom_config;

    if (psensor_inst == NULL) {
        pr_info("Oplusimgsensor_i2c_init psensor_inst is NULL\n");
        return IMGSENSOR_RETURN_ERROR;
    }

    pOplusImgHWCfg = Oplusimgsensor_Custom_Config();
    ret = imgsensor_i2c_init(&psensor_inst->i2c_cfg,
                             pOplusImgHWCfg[psensor_inst->sensor_idx].i2c_dev);
    pr_debug("[%s] sensor_idx:%d name:%s ret: %d\n",
        __func__,
        psensor_inst->sensor_idx,
        psensor_inst->psensor_list->name,
        ret);

    return ret;
}

struct IMGSENSOR_HW_POWER_SEQ *Oplusimgsensor_matchhwcfg_power(
        enum  IMGSENSOR_POWER_ACTION_INDEX  pwr_actidx)
{
    struct IMGSENSOR_HW_POWER_SEQ *ppwr_seq = NULL;
    int pcbVer = 0;
    pr_debug("[%s] pwr_actidx:%d\n", __func__, pwr_actidx);
    if ((pwr_actidx != IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX)
        && (pwr_actidx != IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX)) {
        return NULL;
    }
    if (is_project(20615)) {
        if (pwr_actidx == IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX) {
            ppwr_seq = sensor_power_sequence_20615;
            pr_debug("[%s] match sensor_power_sequence_20615\n", __func__);
        } else {
            pr_debug("[%s] 20615 NOT Support MIPISWITCH\n", __func__);
        }
        return ppwr_seq;
    }
#ifdef SENSOR_PLATFORM_5G_H
    if (is_project(20131) || is_project(20133)
          || is_project(20255) || is_project(20257) || is_project(20171)
          || is_project(20172) || is_project(20353)) {
        if (pwr_actidx == IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX) {
            ppwr_seq = sensor_power_sequence_20131;
            pr_debug("[%s] match sensor_power_sequence_20131\n", __func__);
        } else {
            pr_debug("[%s] 20131 NOT Support MIPISWITCH\n", __func__);
        }
    }else if (is_project(20817) || is_project(20827)
              || is_project(20831)) {
        if (pwr_actidx == IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX) {
            pcbVer = get_PCB_Version();
            pr_debug("Oplusimgsensor_ldo_powerset pcbVer: %d\n", pcbVer);
            if (PCB_VERSION_EVB == pcbVer)
                ppwr_seq = sensor_power_sequence_20817;
            else if (PCB_VERSION_EVB2 == pcbVer || PCB_VERSION_T0 == pcbVer
                     || PCB_VERSION_T1 == pcbVer)
                ppwr_seq = sensor_power_sequence_20817_EVB2;
            else
                ppwr_seq = sensor_power_sequence_20817_EVT;
            pr_debug("[%s] match sensor_power_sequence_20817%s\n", __func__,
                     ((pcbVer == PCB_VERSION_EVB) ? "" : ((pcbVer == PCB_VERSION_EVB2 || pcbVer == PCB_VERSION_T0 || pcbVer == PCB_VERSION_T1) ? "_EVB2":"_EVT")));
        } else {
            pr_debug("[%s] 20817 NOT Support MIPISWITCH\n", __func__);
        }
    }
#endif
#ifdef SENSOR_PLATFORM_5G_B
if (is_project(20075) || is_project(20076)) {
        if (pwr_actidx == IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX) {
            ppwr_seq = sensor_power_sequence_20075;
            pr_info("[%s] match sensor_power_sequence_20075\n", __func__);
        } else if (pwr_actidx == IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX){
            pr_info("[%s] enter for 20075 IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX \n", __func__);
            return NULL;
        }
    }
#endif
    return ppwr_seq;
}

enum IMGSENSOR_RETURN Oplusimgsensor_ldoenable_power(
        struct IMGSENSOR_HW             *phw,
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    struct IMGSENSOR_HW_DEVICE *pdev = phw->pdev[IMGSENSOR_HW_ID_GPIO];
    pr_debug("[%s] sensor_idx:%d pwr_status: %d\n", __func__, sensor_idx, pwr_status);
    if ((pwr_status == IMGSENSOR_HW_POWER_STATUS_ON)
        && (is_project(20131) || is_project(20133)
              || is_project(20255) || is_project(20257)
              || is_project(20615) || is_project(20171)
              || is_project(20172) || is_project(20353)
              || is_project(20817) || is_project(20827)
              || is_project(20831))) {
        if (pdev->set != NULL) {
            pr_debug("set GPIO29 to enable fan53870");
            pdev->set(pdev->pinstance, sensor_idx, IMGSENSOR_HW_PIN_FAN53870_ENABLE, Vol_High);
        } else {
            pr_debug("[%s] sensor_idx:%d pdev->set is ERROR\n", __func__, sensor_idx );
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_gpioenable_power(
        struct IMGSENSOR_HW             *phw,
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    int pcbVer = 0;
    struct IMGSENSOR_HW_DEVICE *pdev = phw->pdev[IMGSENSOR_HW_ID_GPIO];
    /*20210122 add for EVB2*/
    pcbVer = get_PCB_Version();
    pr_debug("[%s] sensor_idx:%d pwr_status: %d project(20817): %s pcbVer: %d\n", __func__, sensor_idx, pwr_status, (is_project(20817) ? "true" : "false"), pcbVer);
    if (PCB_VERSION_EVB == pcbVer)
        return IMGSENSOR_RETURN_SUCCESS;

    if ((pwr_status == IMGSENSOR_HW_POWER_STATUS_ON)
            && (is_project(20817) || is_project(20827)
                || is_project(20831))) {
        if (pdev->set != NULL) {
            pr_debug("set GPIO133 to enable power");
            pdev->set(pdev->pinstance, sensor_idx, IMGSENSOR_HW_PIN_GPIO_POWER_ENABLE, Vol_High);
        } else {
            pr_debug("[%s] sensor_idx:%d pdev->set is ERROR\n", __func__, sensor_idx );
        }
    } else {
        if (is_project(20817) || is_project(20827)
            || is_project(20831)) {
            if (pdev->set != NULL) {
                pr_debug("set GPIO133 to disable power");
                pdev->set(pdev->pinstance, sensor_idx, IMGSENSOR_HW_PIN_GPIO_POWER_ENABLE, Vol_Low);
            } else {
                pr_debug("[%s] sensor_idx:%d pdev->set is ERROR\n", __func__, sensor_idx );
            }
        }
    }

    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_power_fan53870_20615(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    int fan53870_avdd_20615[4][2] = {{7,2900},{0,0},{4,2800},{4,2800}};
    int fan53870_avdd1_20615[2] = {5,1800};
    int fan53870_dvdd_20615[3][2] = {{0,0},{1,1050},{2,1200}};
    int avddIdx = sensor_idx > IMGSENSOR_SENSOR_IDX_SUB2 ?
                    IMGSENSOR_SENSOR_IDX_SUB2 : sensor_idx;
    pr_debug("[%s] is_fan53870_pmic:%d pwr_status:%d avddIdx:%d", __func__, is_fan53870_pmic(), pwr_status, avddIdx);

    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        if ((pin == IMGSENSOR_HW_PIN_AVDD) &&  (avddIdx != IMGSENSOR_SENSOR_IDX_SUB)) {
            fan53870_cam_ldo_set_voltage(fan53870_avdd_20615[avddIdx][0], fan53870_avdd_20615[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_AVDD_1){
            fan53870_cam_ldo_set_voltage(fan53870_avdd1_20615[0],fan53870_avdd1_20615[1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && ((avddIdx == IMGSENSOR_SENSOR_IDX_MAIN2) || (avddIdx == IMGSENSOR_SENSOR_IDX_SUB)) ){
            fan53870_cam_ldo_set_voltage(fan53870_dvdd_20615[avddIdx][0],fan53870_dvdd_20615[avddIdx][1]);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        if ((pin == IMGSENSOR_HW_PIN_AVDD) &&  (avddIdx != IMGSENSOR_SENSOR_IDX_SUB)) {
            fan53870_cam_ldo_disable(fan53870_avdd_20615[avddIdx][0]);
        }else if(pin == IMGSENSOR_HW_PIN_AVDD_1){
            fan53870_cam_ldo_disable(fan53870_avdd1_20615[0]);
        }else if (pin == IMGSENSOR_HW_PIN_DVDD && ((avddIdx == IMGSENSOR_SENSOR_IDX_MAIN2) || (avddIdx == IMGSENSOR_SENSOR_IDX_SUB)) ){
            fan53870_cam_ldo_disable(fan53870_dvdd_20615[avddIdx][0]);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;

}

enum IMGSENSOR_RETURN Oplusimgsensor_power_fan53870_20817(
        enum   IMGSENSOR_SENSOR_IDX 	 sensor_idx,
        enum   IMGSENSOR_HW_PIN 		 pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    int fan53870_avdd_20817[3][2] = {{3,2800}, {7,2900}, {4,2700}};
    int fan53870_avdd1_20817[2] = {5,1800};
    int fan53870_afvdd_20817[2] = {6,2800};
    int fan53870_dvdd_20817[3][2] = {{0,0},{0,0},{2,1200}};
    int avddIdx = sensor_idx;
    pr_debug("[%s] is_fan53870_pmic:%d pwr_status:%d avddIdx:%d pin: %d", __func__, is_fan53870_pmic(), pwr_status, avddIdx, pin);
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        if ((pin == IMGSENSOR_HW_PIN_AVDD) && (avddIdx >= IMGSENSOR_SENSOR_IDX_MAIN && avddIdx <= IMGSENSOR_SENSOR_IDX_MAIN2) ) {
            fan53870_cam_ldo_set_voltage(fan53870_avdd_20817[avddIdx][0], fan53870_avdd_20817[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_AVDD_1 && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN)){
            fan53870_cam_ldo_set_voltage(fan53870_avdd1_20817[0], fan53870_avdd1_20817[1]);
        } else if(pin == IMGSENSOR_HW_PIN_AFVDD && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN)) {
            fan53870_cam_ldo_set_voltage(fan53870_afvdd_20817[0], fan53870_afvdd_20817[1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && avddIdx == IMGSENSOR_SENSOR_IDX_MAIN2){
            fan53870_cam_ldo_set_voltage(fan53870_dvdd_20817[avddIdx][0],fan53870_dvdd_20817[avddIdx][1]);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        if ((pin == IMGSENSOR_HW_PIN_AVDD) && (avddIdx >= IMGSENSOR_SENSOR_IDX_MAIN && avddIdx <= IMGSENSOR_SENSOR_IDX_MAIN2) ) {
            fan53870_cam_ldo_disable(fan53870_avdd_20817[avddIdx][0]);
        } else if(pin == IMGSENSOR_HW_PIN_AVDD_1 && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN)){
            fan53870_cam_ldo_disable(fan53870_avdd1_20817[0]);
        } else if(pin == IMGSENSOR_HW_PIN_AFVDD && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN)) {
            fan53870_cam_ldo_disable(fan53870_afvdd_20817[0]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && avddIdx == IMGSENSOR_SENSOR_IDX_MAIN2){
            fan53870_cam_ldo_disable(fan53870_dvdd_20817[avddIdx][0]);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }

    return IMGSENSOR_RETURN_SUCCESS;

}

enum IMGSENSOR_RETURN Oplusimgsensor_power_fan53870_20817_EVB2(
        enum   IMGSENSOR_SENSOR_IDX 	 sensor_idx,
        enum   IMGSENSOR_HW_PIN 		 pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    int fan53870_avdd_20817[5][2] = {{3,2800}, {7,2900}, {4,2800}, {0,0}, {4,2800}};
    int fan53870_dvdd_20817[3][2] = {{0,0},{1,1100},{2,1200}};
    int fan53870_afvdd_20817[2] = {6,2800};
    int avddIdx = sensor_idx;
    pr_debug("[%s] is_fan53870_pmic:%d pwr_status:%d avddIdx:%d pin: %d", __func__, is_fan53870_pmic(), pwr_status, avddIdx, pin);
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        if (((pin == IMGSENSOR_HW_PIN_AVDD_1) && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN))
            || ((pin == IMGSENSOR_HW_PIN_AVDD) && (avddIdx > IMGSENSOR_SENSOR_IDX_MAIN && avddIdx <= IMGSENSOR_SENSOR_IDX_MAIN3))) {
            pmic_ldo_set_voltage_mv(fan53870_avdd_20817[avddIdx][0], fan53870_avdd_20817[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN2 || avddIdx == IMGSENSOR_SENSOR_IDX_SUB)){
            pmic_ldo_set_voltage_mv(fan53870_dvdd_20817[avddIdx][0],fan53870_dvdd_20817[avddIdx][1]);
        } else if(pin == IMGSENSOR_HW_PIN_AFVDD && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN)) {
            pmic_ldo_set_voltage_mv(fan53870_afvdd_20817[0], fan53870_afvdd_20817[1]);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        if (((pin == IMGSENSOR_HW_PIN_AVDD_1) && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN))
            || ((pin == IMGSENSOR_HW_PIN_AVDD) && (avddIdx > IMGSENSOR_SENSOR_IDX_MAIN && avddIdx <= IMGSENSOR_SENSOR_IDX_MAIN3))) {
            pmic_ldo_set_disable(fan53870_avdd_20817[avddIdx][0]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN2 || avddIdx == IMGSENSOR_SENSOR_IDX_SUB)){
            pmic_ldo_set_disable(fan53870_dvdd_20817[avddIdx][0]);
        } else if(pin == IMGSENSOR_HW_PIN_AFVDD && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN)) {
            pmic_ldo_set_disable(fan53870_afvdd_20817[0]);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }

    return IMGSENSOR_RETURN_SUCCESS;

}

enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    int pcbVer = 0;
    int pmic_avdd[3][2] = {{3, 2800}, {4, 2900}, {6, 2800}};
    int avddIdx = sensor_idx > IMGSENSOR_SENSOR_IDX_SUB ?
                    IMGSENSOR_SENSOR_IDX_MAIN2 : sensor_idx;

    pr_debug("[%s] pmic_ldo_get_type:%d pwr_status:%d avddIdx:%d",
                    __func__, pmic_ldo_get_type(), pwr_status, avddIdx);
#ifdef SENSOR_PLATFORM_5G_B
    if (is_project(20075) || is_project(20076))
        return IMGSENSOR_RETURN_ERROR;
#endif

    if(is_project(20615))
        return Oplusimgsensor_power_fan53870_20615(sensor_idx,pin,pwr_status);
    /*20201224 add for 20817 sensor porting*/
    if(is_project(20817) || is_project(20827)
       || is_project(20831)) {
        /*20210119 add for EVB2*/
        pcbVer = get_PCB_Version();
        pr_debug("Oplusimgsensor_ldo_powerset pcbVer: %d\n", pcbVer);
        if (PCB_VERSION_EVB == pcbVer) {
            return Oplusimgsensor_power_fan53870_20817(sensor_idx,pin,pwr_status);
        } else {
            return Oplusimgsensor_power_fan53870_20817_EVB2(sensor_idx,pin,pwr_status);
        }
    }

    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_voltage_mv(
                pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD
                && sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2){
            if(is_project(20171) || is_project(20172) || is_project(20353)) {
                pmic_ldo_set_voltage_mv(1, 1200);
            } else {
                pmic_ldo_set_voltage_mv(1, 1050);
            }
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_disable(pmic_avdd[avddIdx][0]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD
                && sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2){
            pmic_ldo_set_disable(1);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }

    return IMGSENSOR_RETURN_SUCCESS;
}

void Oplusimgsensor_Registdeviceinfo(char *name, char *version, kal_uint8 module_id)
{
    char *manufacture;
    if (name == NULL || version == NULL)
    {
        pr_info("name or version is NULL");
        return;
    }
    switch (module_id)
    {
        case IMGSENSOR_MODULE_ID_SUNNY:  /* Sunny */
            manufacture = DEVICE_MANUFACUTRE_SUNNY;
            break;
        case IMGSENSOR_MODULE_ID_TRULY:  /* Truly */
            manufacture = DEVICE_MANUFACUTRE_TRULY;
            break;
        case IMGSENSOR_MODULE_ID_SEMCO:  /* Semco */
            manufacture = DEVICE_MANUFACUTRE_SEMCO;
            break;
        case IMGSENSOR_MODULE_ID_LITEON:  /* Lite-ON */
            manufacture = DEVICE_MANUFACUTRE_LITEON;
            break;
        case IMGSENSOR_MODULE_ID_QTECH:  /* Q-Tech */
            manufacture = DEVICE_MANUFACUTRE_QTECH;
            break;
        case IMGSENSOR_MODULE_ID_OFILM:  /* O-Film */
            manufacture = DEVICE_MANUFACUTRE_OFILM;
            break;
        case IMGSENSOR_MODULE_ID_SHINE:  /* Shine */
            manufacture = DEVICE_MANUFACUTRE_SHINE;
            break;
        default:
            manufacture = DEVICE_MANUFACUTRE_NA;
    }
    register_device_proc(name, version, manufacture);
}

void Oplusimgsensor_powerstate_notify(bool val)
{
    if (is_project(20131) || is_project(20133)
          || is_project(20255) || is_project(20257)
          || is_project(20615) || is_project(20171)
          || is_project(20172) || is_project(20353)
          || is_project(20817) || is_project(20827)
          || is_project(20831)) {
        pr_info("[%s] val:%d", __func__, val);
        oplus_chg_set_camera_status(val);
        oplus_chg_set_camera_on(val);
    }
}
