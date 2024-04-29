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
#define PCB_VERSION_10   (10)

#ifdef CONFIG_PROJECT_20601
#include <linux/mutex.h>

static DEFINE_MUTEX(fan53870_drv_mutex);
int camera_status = 0;

enum FAN53870_MUTEX_CONTROL {
    FAN53870_MUTEX_UNLOCK,
    FAN53870_MUTEX_LOCK
};
int fan53870_lock(int val)
{
    if (val == FAN53870_MUTEX_LOCK) {
        mutex_lock(&fan53870_drv_mutex);
    } else {
        mutex_unlock(&fan53870_drv_mutex);
    }

    return 1;
}
#endif /* CONFIG_PROJECT_20601 */
struct IMGSENSOR_SENSOR_LIST *Oplusimgsensor_Sensorlist(void)
{
    struct IMGSENSOR_SENSOR_LIST *pOplusImglist = gimgsensor_sensor_list;

    pOplusImglist =  oplus_gimgsensor_sensor_list;
    pr_info("oplus_gimgsensor_sensor_list Selected\n");


    return pOplusImglist;
}

struct IMGSENSOR_HW_CFG *Oplusimgsensor_Custom_Config(void)
{
    struct IMGSENSOR_HW_CFG *pOplusImgHWCfg = imgsensor_custom_config;
    pOplusImgHWCfg = oplus_imgsensor_custom_config;
    pr_info("oplus_imgsensor_custom_config Selected\n");

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
#ifdef SENSOR_PLATFORM_5G_H
    int pcbVer = 0;
#endif
    PK_PR_ERR("[%s] pwr_actidx:%d\n", __func__, pwr_actidx);
    if ((pwr_actidx != IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX)
        && (pwr_actidx != IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX)) {
        pr_info("[%s] Invalid pwr_actidx:%d\n", __func__, pwr_actidx);
        return NULL;
    }


        if (pwr_actidx == IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX) {
            ppwr_seq = oplus_sensor_power_sequence;
            pr_info("[%s] match oplus_sensor_power_sequence\n", __func__);
#ifdef SENSOR_PLATFORM_5G_B
		} else if (pwr_actidx == IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX){
			 if (is_project(20095) || is_project(20610) || is_project(20611) || is_project(20680)
                            || is_project(20639) || is_project(20638) || is_project(0x206B7)) {
		        ppwr_seq = oplus_platform_power_sequence;
		        pr_info("[%s] enter for 20075 IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX \n", __func__);
		        return NULL;
            }
    } else if (pwr_actidx == IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX){
            ppwr_seq = oplus_platform_power_sequence;
            pr_info("[%s] enter for 20631 IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX \n", __func__);
            return NULL;
#endif

#ifdef SENSOR_PLATFORM_4G_20682
        } else if (pwr_actidx == IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX){
            if (is_project(20730) || is_project(20731) || is_project(20732) || is_project(20682)) {
                ppwr_seq = oplus_platform_power_sequence;
                pr_info("[%s] match 20730 20682 IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX \n", __func__);
            }
#endif
#ifdef SENSOR_PLATFORM_5G_A
        } else if (pwr_actidx == IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX) {
            ppwr_seq = oplus_platform_power_sequence;
            pr_info("[%s] match 19131 19420 IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX \n", __func__);
#endif
#ifdef SENSOR_PLATFORM_5G_H
    } else if (pwr_actidx == IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX){
            if(is_project(19165)){
            pr_info("[%s] enter for 19165 IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX \n", __func__);
            return NULL;
            }
#endif
    } else if (is_project(21690) || is_project(21691) || is_project(21692)) {
            if (pwr_actidx == IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX) {
                pr_info("[%s] match sensor_power_sequence_21684\n", __func__);
                ppwr_seq = oplus_sensor_power_sequence;
            } else if (pwr_actidx == IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX){
                pr_info("[%s] enter for SPACE IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX \n", __func__);
                return NULL;
            }
        } else {
            pr_info("[%s] OT Support MIPISWITCH\n", __func__);
        }

#ifdef SENSOR_PLATFORM_5G_H
    if (is_project(20817) || is_project(20827) || is_project(20831)) {
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

    if (is_project(21881) || is_project(21882)) {
        if (pwr_actidx == IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX) {
            ppwr_seq = sensor_power_sequence_21881;
            pr_debug("[%s] match sensor_power_sequence_21881\n", __func__);
        } else {
            pr_debug("[%s] 21881/21882 NOT Support MIPISWITCH\n", __func__);
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
    static int pmic_enable_cnt = 0;
#ifdef CONFIG_PROJECT_20601
    int last_camera_status = 0;
    fan53870_lock(FAN53870_MUTEX_LOCK);
#endif /* CONFIG_PROJECT_20601 */
    if (is_project(19165)) {
        pr_debug("Is project 19165,non-fan53870 project.");
        return IMGSENSOR_RETURN_SUCCESS;
    }
    pr_debug("[%s] sensor_idx:%d pwr_status: %d\n", __func__, sensor_idx, pwr_status);
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        if (pdev->set != NULL) {
            if (is_project(20171) || is_project(20353) || is_project(20131)
                || is_project(20133) || is_project(20255) || is_project(20257) || is_project(21061)
                || is_project(20615) || is_project(21609) || is_project(20662) || is_project(21305)
                || is_project(20619) || is_project(21651) || is_project(20645) || is_project(21127)
                || is_project(21015) || is_project(21217) || is_project(21016) || is_project(21218)
                || is_project(20827) || is_project(20831) || is_project(21881) || is_project(21882)) {
                pr_debug("set GPIO29 to enable fan53870");
                pdev->set(pdev->pinstance, sensor_idx, IMGSENSOR_HW_PIN_FAN53870_ENABLE, Vol_High);
            }
            if (is_project(20730) || is_project(20731) || is_project(20732)) {
                pr_debug("set GPIO122 to enable fan53870");
                pdev->set(pdev->pinstance, sensor_idx, IMGSENSOR_HW_PIN_FAN53870_ENABLE, Vol_High);
            }
            if (is_project(20181) || is_project(20355) || is_project(21081) || is_project(22277)
                || is_project(22693) || is_project(22694) || is_project(22612) || is_project(0x226B1)
                || is_project(23031) || is_project(23231)) {
                printk("pmic_gpio_enable");
                pmic_gpio_enable(pwr_status);
            }
            if (is_project(22629) || is_project(22710) || is_project(22711)) {
                printk("pmic_gpio_enable 22629");
                pmic_gpio_enable(pwr_status);
            }
            if (is_project(22633) || is_project(22712) || is_project(22713)) {
                printk("pmic_gpio_enable 22633");
                if (sensor_idx == 0 || sensor_idx == 1) {
                    pdev->set(pdev->pinstance, sensor_idx, IMGSENSOR_HW_PIN_PDN, Vol_High);
                    pmic_enable_cnt++;
                    mdelay(10);
                }
                pmic_gpio_enable(pwr_status);
            }
            if (is_project(23613) || is_project(23686) || is_project(23687) || is_project(23689)) {
                if (sensor_idx == 0 || sensor_idx == 1 || sensor_idx == 2 || sensor_idx == 6) {
                    printk("HW_PIN_PDN powerup\n");
                    pdev->set(pdev->pinstance, sensor_idx, IMGSENSOR_HW_PIN_PDN, Vol_High);
                    pmic_enable_cnt++;
                    mdelay(10);
                }
            }
        } else {
            pr_debug("[%s] sensor_idx:%d pdev->set is ERROR\n", __func__, sensor_idx );
        }
    } else {
        if (is_project(22633) || is_project(22712) || is_project(22713)) {
            printk("set GPIO158 to disable DVDD");
            if ((sensor_idx == 0 || sensor_idx == 1) && (pmic_enable_cnt-- == 1)) {
                pdev->set(pdev->pinstance, sensor_idx, IMGSENSOR_HW_PIN_PDN, Vol_Low);
            }
        } else if (is_project(23613) || is_project(23686) || is_project(23687) || is_project(23689)) {
            if ((sensor_idx == 0 || sensor_idx == 1 || sensor_idx == 2 || sensor_idx == 6) && (pmic_enable_cnt-- == 1)) {
                printk("HW_PIN_PDN powerdown\n");
                pdev->set(pdev->pinstance, sensor_idx, IMGSENSOR_HW_PIN_PDN, Vol_Low);
            }
        }
    }
#ifdef CONFIG_PROJECT_20601
    else if ((pwr_status == IMGSENSOR_HW_POWER_STATUS_ON)
        && (is_project(20601) || is_project(20602)|| is_project(20660))) {
        last_camera_status = camera_status;
        if (is_check_lcm_status() || last_camera_status != 0) {
            pr_err("camera enable: the gpiod_set_value is 1, last_camera_status%d, lcm_status%d\n", last_camera_status, is_check_lcm_status());
        } else {
            pdev = phw->pdev[IMGSENSOR_HW_ID_GPIO];
            if (pdev->set != NULL) {
                pr_err("camera enable: the gpiod_set_value is 1, last_camera_status%d, lcm_status%d\n", last_camera_status, is_check_lcm_status());
                pdev->set(pdev->pinstance, IMGSENSOR_SENSOR_IDX_MAIN, IMGSENSOR_HW_PIN_PMIC_ENABLE, Vol_High);
            }
        }
        camera_status = 1;
    } else if ((pwr_status == IMGSENSOR_HW_POWER_STATUS_OFF)
        && (is_project(20601) || is_project(20602)|| is_project(20660))) {
        last_camera_status = camera_status;
        if(is_check_lcm_status()) {
            pr_err("camera disable: the gpiod_set_value is 1, last_camera_status%d, lcm_status%d\n", last_camera_status, is_check_lcm_status());
        } else if(last_camera_status == 0) {
            pr_err("camera disable: the gpiod_set_value is 0, last_camera_status%d, lcm_status%d\n", last_camera_status, is_check_lcm_status());
        } else {
            pdev = phw->pdev[IMGSENSOR_HW_ID_GPIO];
            if (pdev->set != NULL) {
                pr_err("camera disable: the gpiod_set_value is 0, last_camera_status%d, lcm_status%d\n", last_camera_status, is_check_lcm_status());
                pdev->set(pdev->pinstance, IMGSENSOR_SENSOR_IDX_MAIN, IMGSENSOR_HW_PIN_PMIC_ENABLE, Vol_Low);
            }
        }
        camera_status = 0;
    }
    fan53870_lock(FAN53870_MUTEX_UNLOCK);
#endif /* CONFIG_PROJECT_20601 */
    return IMGSENSOR_RETURN_SUCCESS;
}


enum IMGSENSOR_RETURN Oplusimgsensor_power_fan53870_20645(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    int fan53870_avdd_20645[3][2] = {{3,2800},{4,2800},{6,2800}};
    int fan53870_dvdd_20645[2] = {1,1200};
    int avddIdx = sensor_idx > IMGSENSOR_SENSOR_IDX_SUB ?
                    IMGSENSOR_SENSOR_IDX_MAIN2 : sensor_idx;
    pr_debug("[%s] is_fan53870_pmic:%d pwr_status:%d avddIdx:%d", __func__, is_fan53870_pmic(), pwr_status, avddIdx);

    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            fan53870_cam_ldo_set_voltage(fan53870_avdd_20645[avddIdx][0], fan53870_avdd_20645[avddIdx][1]);
        } else if ((pin == IMGSENSOR_HW_PIN_DVDD) && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN2)) {
            fan53870_cam_ldo_set_voltage(fan53870_dvdd_20645[0], fan53870_dvdd_20645[1]);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            fan53870_cam_ldo_disable(fan53870_avdd_20645[avddIdx][0]);
        } else if ((pin == IMGSENSOR_HW_PIN_DVDD) && (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2)) {
            fan53870_cam_ldo_disable(1);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_power_fan53870_21061(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    int fan53870_avdd[4][2] = {{7,2850},{0,0},{5,2800},{5,2800}};
    int fan53870_avdd1[2] = {5,1800};
    int fan53870_dvdd[3][2] = {{0,0},{1,1100},{2,1250}};
    int avddIdx = sensor_idx > IMGSENSOR_SENSOR_IDX_SUB2 ?
                    IMGSENSOR_SENSOR_IDX_SUB2 : sensor_idx;
    pr_debug("[%s] is_fan53870_pmic:%d pwr_status:%d avddIdx:%d", __func__, is_fan53870_pmic(), pwr_status, avddIdx);

    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        if ((pin == IMGSENSOR_HW_PIN_AVDD) &&  (avddIdx != IMGSENSOR_SENSOR_IDX_SUB)) {
            fan53870_cam_ldo_set_voltage(fan53870_avdd[avddIdx][0], fan53870_avdd[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_AVDD_1){
            fan53870_cam_ldo_set_voltage(fan53870_avdd1[0],fan53870_avdd1[1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && ((avddIdx == IMGSENSOR_SENSOR_IDX_MAIN2) || (avddIdx == IMGSENSOR_SENSOR_IDX_SUB)) ){
            fan53870_cam_ldo_set_voltage(fan53870_dvdd[avddIdx][0],fan53870_dvdd[avddIdx][1]);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        if ((pin == IMGSENSOR_HW_PIN_AVDD) &&  (avddIdx != IMGSENSOR_SENSOR_IDX_SUB)) {
            fan53870_cam_ldo_disable(fan53870_avdd[avddIdx][0]);
        }else if(pin == IMGSENSOR_HW_PIN_AVDD_1){
            fan53870_cam_ldo_disable(fan53870_avdd1[0]);
        }else if (pin == IMGSENSOR_HW_PIN_DVDD && ((avddIdx == IMGSENSOR_SENSOR_IDX_MAIN2) || (avddIdx == IMGSENSOR_SENSOR_IDX_SUB)) ){
            fan53870_cam_ldo_disable(fan53870_dvdd[avddIdx][0]);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;

}

enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset_21081(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    int pmic_avdd_1[3][2] = {{3, 2900}, {4, 2800}, {6, 2800}};
    int avddIdx = sensor_idx > 1 ? 2 : sensor_idx;

    pr_debug("[%s] pmic_ldo_get_type:%d pwr_status:%d",
            __func__, pmic_ldo_get_type(), pwr_status);
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd_1:%d %d", __func__, avddIdx, pmic_avdd_1[avddIdx][0], pmic_avdd_1[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_voltage_mv(pmic_avdd_1[avddIdx][0], pmic_avdd_1[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_voltage_mv(1, 1100);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1){
            pmic_ldo_set_voltage_mv(2, 1050);
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx == 0) {
            pmic_ldo_set_voltage_mv(7, 2800);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_disable(pmic_avdd_1[avddIdx][0]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_disable(1);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1){
            pmic_ldo_set_disable(2);
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx == 0) {
            pmic_ldo_set_disable(7);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset_20181(
    enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
    enum   IMGSENSOR_HW_PIN          pin,
    enum   IMGSENSOR_HW_POWER_STATUS pwr_status,
    int pcb)
{
    int pmic_avdd[3][2] = {{IMGSENSOR_SENSOR_IDX_SUB2, PMIC_AVDD_VOLTAGE_MV_2800}, {IMGSENSOR_SENSOR_IDX_MAIN3, PMIC_AVDD_VOLTAGE_MV_2900}, {IMGSENSOR_SENSOR_IDX_MAIN4, PMIC_AVDD_VOLTAGE_MV_2800}};
    int avddIdx = sensor_idx > IMGSENSOR_SENSOR_IDX_SUB ?
        IMGSENSOR_SENSOR_IDX_MAIN2 : sensor_idx;

    pr_debug("[%s] pmic_ldo_get_type:%d pwr_status:%d avddIdx:%d",
        __func__, pmic_ldo_get_type(), pwr_status, avddIdx);

    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_voltage_mv(
                pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD
            && sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2) {
            pmic_ldo_set_voltage_mv(IMGSENSOR_SENSOR_IDX_SUB, PMIC_AVDD_VOLTAGE_MV_1200);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == IMGSENSOR_SENSOR_IDX_SUB) {
            pr_err("[%s] pcb:%d", __func__, pcb);
            if (is_project(20181) && pcb == PCB_VERSION_10 && is_fan53870_pmic() != FAN_53870_PMIC_FAILED) {
                pmic_ldo_set_voltage_mv(IMGSENSOR_SENSOR_IDX_MAIN2, PMIC_AVDD_VOLTAGE_MV_1300);
            } else {
                pmic_ldo_set_voltage_mv(IMGSENSOR_SENSOR_IDX_MAIN2, PMIC_AVDD_VOLTAGE_MV_1100);
            }
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_disable(pmic_avdd[avddIdx][0]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD
            && sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2){
            pmic_ldo_set_disable(WL2868C_LDO1);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == IMGSENSOR_SENSOR_IDX_SUB){
            pmic_ldo_set_disable(WL2868C_LDO2);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }

    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset_22693(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{                            //
    static int pmic_avdd[4][2] = {{6, 2200}, {4, 2800}, {3, 2700}};
    int avddIdx = sensor_idx > 1 ? 2 : sensor_idx;

    pr_debug("[%s] pmic_ldo_get_type:%d pwr_status:%d",
            __func__, pmic_ldo_get_type(), pwr_status);
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {     //IMGSENSOR_HW_PIN_AVDD:3
            pmic_ldo_set_voltage_mv(pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_voltage_mv(1, 1000);   //IMGSENSOR_HW_PIN_DVDD:4
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 1)){
            pmic_ldo_set_voltage_mv(2, 1050);
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx == 0){
            pmic_ldo_set_voltage_mv(5, 2800);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_disable(pmic_avdd[avddIdx][0]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_disable(1);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 1)){
            pmic_ldo_set_disable(2);
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx == 0){
            pmic_ldo_set_disable(5);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset_22629(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{                            //
    static int pmic_avdd[4][2] = {{6, 2200}, {4, 2900}, {3, 2700}, {3, 2800}};
    int avddIdx = sensor_idx;

    pr_debug("[%s] pmic_ldo_get_type:%d pwr_status:%d",
            __func__, pmic_ldo_get_type(), pwr_status);
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD && sensor_idx == 3){
            pmic_ldo_set_voltage_mv(6, 2200);
            pmic_ldo_set_voltage_mv(3, 2800);
        }else if (pin == IMGSENSOR_HW_PIN_AVDD) {     //IMGSENSOR_HW_PIN_AVDD:3
            pmic_ldo_set_voltage_mv(pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_voltage_mv(1, 900);   //IMGSENSOR_HW_PIN_DVDD:4
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 1)){
            pmic_ldo_set_voltage_mv(6, 2200);
            pmic_ldo_set_voltage_mv(2, 1100);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 2)){
            pmic_ldo_set_voltage_mv(6, 2200);
            pmic_ldo_set_voltage_mv(2, 1200);
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx == 0){
            pmic_ldo_set_voltage_mv(5, 3000);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        if (pin == IMGSENSOR_HW_PIN_AVDD && sensor_idx == 3) {
            pmic_ldo_set_disable(3);
            pmic_ldo_set_disable(6);
        } else if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_disable(pmic_avdd[avddIdx][0]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_disable(1);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 1)){
            pmic_ldo_set_disable(2);
            pmic_ldo_set_disable(6);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 2)){
            pmic_ldo_set_disable(2);
            pmic_ldo_set_disable(6);
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx == 0){
            pmic_ldo_set_disable(5);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset_23687(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    #ifdef CONFIG_LDO_AW37004
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 0 || sensor_idx == 6)){
            aw37004_camera_power_up(0);
        }else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1) {
            aw37004_camera_power_up(1);
        } else if (pin == IMGSENSOR_HW_PIN_AVDD && (sensor_idx == 0 || sensor_idx == 6)){
            aw37004_camera_power_up(2);
        } else if (pin == IMGSENSOR_HW_PIN_AVDD && (sensor_idx == 1)){
            aw37004_camera_power_up(3);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 0 || sensor_idx == 6)){
            aw37004_camera_power_down(0);
        }else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1) {
            aw37004_camera_power_down(1);
        } else if (pin == IMGSENSOR_HW_PIN_AVDD && (sensor_idx == 0 || sensor_idx == 6)){
            aw37004_camera_power_down(2);
        } else if (pin == IMGSENSOR_HW_PIN_AVDD && sensor_idx == 1){
            aw37004_camera_power_down(3);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    #endif
    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset_22633(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{                            //
    static int pmic_avdd[4][2] = {{6, 2800}, {4, 2800}, {3, 2800}};
    int avddIdx = sensor_idx > 1 ? 2 : sensor_idx;

    pr_debug("[%s] pmic_ldo_get_type:%d pwr_status:%d",
            __func__, pmic_ldo_get_type(), pwr_status);
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {     //  IMGSENSOR_HW_PIN_AVDD:3
            pmic_ldo_set_voltage_mv(pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_voltage_mv(1, 1100);   //  IMGSENSOR_HW_PIN_DVDD:4
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 1)){
            pmic_ldo_set_voltage_mv(2, 1050);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 2)){
            pmic_ldo_set_voltage_mv(2, 1200);
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx == 0){
            pmic_ldo_set_voltage_mv(5, 2800);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_disable(pmic_avdd[avddIdx][0]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_disable(1);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 1)){
            pmic_ldo_set_disable(2);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 2)){
            pmic_ldo_set_disable(2);
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx == 0){
            pmic_ldo_set_disable(5);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_power_fan53870_20730(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    int fan53870_avdd_20730[5][2] = {{3,2800},{4,2800},{7,2800},{7,2800},{7,2800}};
    int fan53870_dvdd_20730[3][2] = {{2,1100},{1,1100},{1,1200}};
    int fan53870_afvdd_20730[2]= {6,2800};
    int avddIdx = sensor_idx;
    int dvddIdx= sensor_idx;
    pr_debug("[%s] is_fan53870_pmic:%d pwr_status:%d avddIdx:%d", __func__, is_fan53870_pmic(), pwr_status, avddIdx);
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            printk("IMGSENSOR_HW_PIN_AVDD on");
            fan53870_cam_ldo_set_voltage(fan53870_avdd_20730[avddIdx][0], fan53870_avdd_20730[avddIdx][1]);
        }
        else if (pin == IMGSENSOR_HW_PIN_AFVDD){
            fan53870_cam_ldo_set_voltage(fan53870_afvdd_20730[0],fan53870_afvdd_20730[1]);
        }
        else if (pin == IMGSENSOR_HW_PIN_DVDD){
            printk("IMGSENSOR_HW_PIN_DVDD on");
            fan53870_cam_ldo_set_voltage(fan53870_dvdd_20730[sensor_idx][0],fan53870_dvdd_20730[sensor_idx][1]);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            printk("IMGSENSOR_HW_PIN_AVDD off");
            fan53870_cam_ldo_disable(fan53870_avdd_20730[avddIdx][0]);
        }
        else if (pin == IMGSENSOR_HW_PIN_DVDD /*&& ((sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2) || (sensor_idx == IMGSENSOR_SENSOR_IDX_SUB2)) */){
            printk("IMGSENSOR_HW_PIN_DVDD off");
            fan53870_cam_ldo_disable(fan53870_dvdd_20730[dvddIdx][0]);
        }
        else if (pin == IMGSENSOR_HW_PIN_AFVDD){
            fan53870_cam_ldo_disable(fan53870_afvdd_20730[0]);
        }
        else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;

}


enum IMGSENSOR_RETURN Oplusimgsensor_power_fan53870_20615(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    int PCB_Version;
    int fan53870_avdd_20615[4][2] = {{7,2900},{0,0},{5,2800},{5,2800}};
    int fan53870_avdd1_20615[2] = {4,1800};
    int fan53870_dvdd_20615[3][2] = {{0,0},{1,1050},{2,1200}};
    int avddIdx = sensor_idx > IMGSENSOR_SENSOR_IDX_SUB2 ?
                    IMGSENSOR_SENSOR_IDX_SUB2 : sensor_idx;
    pr_debug("[%s] is_fan53870_pmic:%d pwr_status:%d avddIdx:%d", __func__, is_fan53870_pmic(), pwr_status, avddIdx);
    pr_debug("%s GetPcbVer:%d",__func__,get_PCB_Version());
    PCB_Version = get_PCB_Version();
    if(((PCB_Version == 4)||(PCB_Version == 2)) && !is_project(20619)){
        fan53870_avdd1_20615[0] = 5;
        fan53870_avdd_20615[2][0] = 4;
        fan53870_avdd_20615[3][0] = 4;
    }
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
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN                  pin,
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
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN                  pin,
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

enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset_212A1(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    static int pmic_avdd[4][2] = {{3, 2800}, {4, 2900}, {6, 2800}, {6, 2800}};
    int avddIdx = sensor_idx > 2 ? 3 : sensor_idx;

    pr_debug("[%s] pmic_ldo_get_type:%d pwr_status:%d",
            __func__, pmic_ldo_get_type(), pwr_status);
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_voltage_mv(pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_voltage_mv(1, 1100);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1){
            pmic_ldo_set_voltage_mv(2, 1100);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 2){
            pmic_ldo_set_voltage_mv(2, 1200);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_disable(pmic_avdd[avddIdx][0]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_disable(1);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 1 || sensor_idx == 2)){
            pmic_ldo_set_disable(2);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset_22277(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    static int pmic_avdd[4][2] = {{4, 2800}, {3, 2800}, {6, 2880}, {6, 2880}};
    int avddIdx = sensor_idx > 2 ? 3 : sensor_idx;
    static int avdd_flag = 0;

    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            if (sensor_idx == 2 || sensor_idx == 4) {
                avdd_flag ++;
                pr_info("[%s] power on avdd_flag:%d", __func__, avdd_flag);
            }
            pmic_ldo_set_voltage_mv(pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_voltage_mv(1, 1100);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1){
            pmic_ldo_set_voltage_mv(2, 1120);
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx == 0){
            pmic_ldo_set_voltage_mv(7, 2800);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            if (sensor_idx == 2 || sensor_idx ==4) {
                avdd_flag --;
                pr_info("[%s] power off avdd_flag:%d", __func__, avdd_flag);
                if (avdd_flag == 0) {
                    pmic_ldo_set_disable(pmic_avdd[avddIdx][0]);
                }
            } else {
                pmic_ldo_set_disable(pmic_avdd[avddIdx][0]);
            }
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_disable(1);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1){
            pmic_ldo_set_disable(2);
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx == 0){
            pmic_ldo_set_disable(7);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset_23231(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    static int pmic_avdd[5][2] = {{4, 2800}, {6, 2800}, {6, 2880}, {6, 2800}, {4, 2880}};
    int avddIdx = sensor_idx > 3 ? (sensor_idx - 1) : sensor_idx;
    static int avdd_flag = 0;
    if(avddIdx > 4){
        avddIdx = 4;
    }
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            if (sensor_idx == 1 || sensor_idx == 2 || sensor_idx == 4) {
                avdd_flag ++;
                pr_info("[%s] power on avdd_flag:%d", __func__, avdd_flag);
            }
            pmic_ldo_set_voltage_mv(pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 0 || sensor_idx == 6)){
            pmic_ldo_set_voltage_mv(1, 1100);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1){
            pmic_ldo_set_voltage_mv(1, 1120);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 4){
            pmic_ldo_set_voltage_mv(2, 800);
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD && (sensor_idx == 0 || sensor_idx == 6 || sensor_idx == 4)){
            pmic_ldo_set_voltage_mv(7, 2800);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            if (sensor_idx == 1 || sensor_idx == 2 || sensor_idx ==4) {
                avdd_flag --;
                pr_info("[%s] power off avdd_flag:%d", __func__, avdd_flag);
                if (avdd_flag == 0) {
                    pmic_ldo_set_disable(pmic_avdd[avddIdx][0]);
                }
            } else {
                pmic_ldo_set_disable(pmic_avdd[avddIdx][0]);
            }
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 0 || sensor_idx == 6)){
            pmic_ldo_set_disable(1);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1){
			pmic_ldo_set_disable(1);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 4){
            pmic_ldo_set_disable(2);
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD && (sensor_idx == 0 || sensor_idx == 6 || sensor_idx == 4)){
            pmic_ldo_set_disable(7);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset_21851(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    static int pmic_avdd[4][2] = {{3, 2800}, {4, 2900}, {6, 2800}, {6, 2800}};
    int avddIdx = sensor_idx > 2 ? 3 : sensor_idx;

    pr_debug("[%s] pmic_ldo_get_type:%d pwr_status:%d",
            __func__, pmic_ldo_get_type(), pwr_status);
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_voltage_mv(pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_voltage_mv(1, 1100);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1){
            pmic_ldo_set_voltage_mv(2, 1100);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 2){
            pmic_ldo_set_voltage_mv(2, 1200);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_disable(pmic_avdd[avddIdx][0]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_disable(1);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 1 || sensor_idx == 2)){
            pmic_ldo_set_disable(2);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_power_fan53870_21881(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN                  pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    int fan53870_avdd_21881[5][2] = {{3,2800}, {7,2900}, {4,2800}, {0,0}, {4,2800}};
    int fan53870_dvdd_21881[3][2] = {{0,0},{1,1100},{2,1200}};
    int fan53870_afvdd_21881[2] = {6,2800};
    int avddIdx = sensor_idx;
    pr_debug("[%s] is_fan53870_pmic:%d pwr_status:%d avddIdx:%d pin: %d", __func__, is_fan53870_pmic(), pwr_status, avddIdx, pin);
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        if (((pin == IMGSENSOR_HW_PIN_AVDD_1) && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN))
            || ((pin == IMGSENSOR_HW_PIN_AVDD) && (avddIdx > IMGSENSOR_SENSOR_IDX_MAIN && avddIdx <= IMGSENSOR_SENSOR_IDX_MAIN3))) {
            pmic_ldo_set_voltage_mv(fan53870_avdd_21881[avddIdx][0], fan53870_avdd_21881[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN2 || avddIdx == IMGSENSOR_SENSOR_IDX_SUB)){
            pmic_ldo_set_voltage_mv(fan53870_dvdd_21881[avddIdx][0],fan53870_dvdd_21881[avddIdx][1]);
        } else if(pin == IMGSENSOR_HW_PIN_AFVDD && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN)) {
            pmic_ldo_set_voltage_mv(fan53870_afvdd_21881[0], fan53870_afvdd_21881[1]);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        if (((pin == IMGSENSOR_HW_PIN_AVDD_1) && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN))
            || ((pin == IMGSENSOR_HW_PIN_AVDD) && (avddIdx > IMGSENSOR_SENSOR_IDX_MAIN && avddIdx <= IMGSENSOR_SENSOR_IDX_MAIN3))) {
            pmic_ldo_set_disable(fan53870_avdd_21881[avddIdx][0]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN2 || avddIdx == IMGSENSOR_SENSOR_IDX_SUB)){
            pmic_ldo_set_disable(fan53870_dvdd_21881[avddIdx][0]);
        } else if(pin == IMGSENSOR_HW_PIN_AFVDD && (avddIdx == IMGSENSOR_SENSOR_IDX_MAIN)) {
            pmic_ldo_set_disable(fan53870_afvdd_21881[0]);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }

    return IMGSENSOR_RETURN_SUCCESS;

}

enum IMGSENSOR_RETURN Oplusimgsensor_power_fan53870_21015(
        enum   IMGSENSOR_SENSOR_IDX   sensor_idx,
        enum   IMGSENSOR_HW_PIN       pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    //These avdd and dvdd are used by 20171 board tmply.
    /*avdd[0]:main; avdd[1]:sub; avdd[2]:main2; avdd[3]:sub2. Note:-1 indicates that the sensor don't use fan53870 powertype.*/
    int pmic_avdd[][2] = {{3, 2800},{4,2900},{6,2800},{6, 2800},{4,2900}};
    int pmic_dvdd[][2] = {{-1,-1},  {2, 800},{1,1200},{-1,-1},  {2,800}};
    int pmic_afvdd[][2] = {{7,2800}};

    pr_debug("[%s] pmic_ldo_get_type:%d pwr_status:%d sensorIdx:%d, pin:%d[RST:2, AVDD:3, DVDD:4, DOVDD:5, MCLK:14]",
                    __func__, pmic_ldo_get_type(), pwr_status , sensor_idx, pin);
    if(IMGSENSOR_SENSOR_IDX_MAIN3 < sensor_idx)
    {
        pr_err("Invalid sensor_idx:%d", sensor_idx);
        return IMGSENSOR_RETURN_ERROR;
    }
    if(pin != IMGSENSOR_HW_PIN_AVDD && pin != IMGSENSOR_HW_PIN_AVDD_1
        && pin != IMGSENSOR_HW_PIN_DVDD && pin != IMGSENSOR_HW_PIN_AFVDD){
        pr_debug("can't use the fan53870 to power this powertype.");
        return IMGSENSOR_RETURN_ERROR;
    }
    if (pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx != IMGSENSOR_SENSOR_IDX_MAIN) {
        pr_debug("can't use the fan53870 to power afvdd.");
        return IMGSENSOR_RETURN_ERROR;
    }
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        //Power on the sensor.
        if((pin == IMGSENSOR_HW_PIN_AVDD || pin == IMGSENSOR_HW_PIN_AVDD_1) && pmic_avdd[sensor_idx][0] != -1){
            pmic_ldo_set_voltage_mv(pmic_avdd[sensor_idx][0], pmic_avdd[sensor_idx][1]);
        }else if(pin == IMGSENSOR_HW_PIN_DVDD &&  pmic_dvdd[sensor_idx][0] != -1){
            pmic_ldo_set_voltage_mv(pmic_dvdd[sensor_idx][0], pmic_dvdd[sensor_idx][1]);
        }else if(pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN) {
            pmic_ldo_set_voltage_mv(pmic_afvdd[sensor_idx][0], pmic_afvdd[sensor_idx][1]);
        }else{
            return IMGSENSOR_RETURN_ERROR;
        }
    }else{
        //Power off the sensor.
        if((pin == IMGSENSOR_HW_PIN_AVDD || pin == IMGSENSOR_HW_PIN_AVDD_1) && pmic_avdd[sensor_idx][0] != -1){
            pmic_ldo_set_disable(pmic_avdd[sensor_idx][0]);
        }else if(pin == IMGSENSOR_HW_PIN_DVDD && pmic_dvdd[sensor_idx][0] != -1){
            pmic_ldo_set_disable(pmic_dvdd[sensor_idx][0]);
        }else if(pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN) {
            pmic_ldo_set_disable(pmic_afvdd[sensor_idx][0]);
        }else{
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_power_fan53870_20601(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    int fan53870_avdd_20601[5][2] = {{3,2900},{4,2800},{4,2800},{4,2800},{4,2800}};
    int fan53870_avdd1_20601[2] = {5,1800};
    int fan53870_dvdd_20601[2] = {2,1200};
    int avddIdx = sensor_idx;
    pr_debug("[%s] is_fan53870_pmic:%d pwr_status:%d avddIdx:%d", __func__, is_fan53870_pmic(), pwr_status, avddIdx);

    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            fan53870_cam_ldo_set_voltage(fan53870_avdd_20601[avddIdx][0], fan53870_avdd_20601[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_AVDD_1 && sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN){
            fan53870_cam_ldo_set_voltage(fan53870_avdd1_20601[0],fan53870_avdd1_20601[1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && ((sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2) || (sensor_idx == IMGSENSOR_SENSOR_IDX_SUB2) || (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN3)) ){
            fan53870_cam_ldo_set_voltage(fan53870_dvdd_20601[0],fan53870_dvdd_20601[1]);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            fan53870_cam_ldo_disable(fan53870_avdd_20601[avddIdx][0]);
        }else if(pin == IMGSENSOR_HW_PIN_AVDD_1 && sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN){
            fan53870_cam_ldo_disable(fan53870_avdd1_20601[0]);
        }else if (pin == IMGSENSOR_HW_PIN_DVDD && ((sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2) || (sensor_idx == IMGSENSOR_SENSOR_IDX_SUB2) || (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN3)) ){
            fan53870_cam_ldo_disable(fan53870_dvdd_20601[0]);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;

}

enum IMGSENSOR_RETURN Oplusimgsensor_power_fan53870_20630(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    int fan53870_avdd_20630[5][2] = {{4,2800},{6,2800},{3,2800},{3,2800},{3,2800}};
    int fan53870_dvdd_20630[2][2] = {{1,1100}, {2,1200}};
    int fan53870_afvdd_20630[2] = {5, 2800};
    int avddIdx = sensor_idx;
    pr_debug("[%s] is_fan53870_pmic:%d pwr_status:%d avddIdx:%d", __func__, is_fan53870_pmic(), pwr_status, avddIdx);

    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            fan53870_cam_ldo_set_voltage(fan53870_avdd_20630[avddIdx][0], fan53870_avdd_20630[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD){
            if((sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN)||(sensor_idx == IMGSENSOR_SENSOR_IDX_SUB)){
                fan53870_cam_ldo_set_voltage(fan53870_dvdd_20630[0][0],fan53870_dvdd_20630[0][1]);
            }else if(sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2){
                fan53870_cam_ldo_set_voltage(fan53870_dvdd_20630[1][0],fan53870_dvdd_20630[1][1]);
            }
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD) {
            if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN) {
                fan53870_cam_ldo_set_voltage(fan53870_afvdd_20630[0],fan53870_afvdd_20630[1]);
            }
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            fan53870_cam_ldo_disable(fan53870_avdd_20630[avddIdx][0]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD){
            if((sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN)||(sensor_idx == IMGSENSOR_SENSOR_IDX_SUB)){
                fan53870_cam_ldo_disable(fan53870_dvdd_20630[0][0]);
            } else if(sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2){
                fan53870_cam_ldo_disable(fan53870_dvdd_20630[1][0]);
            }
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD) {
            if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN) {
                fan53870_cam_ldo_disable(fan53870_afvdd_20630[0]);
            }
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;

}

enum IMGSENSOR_RETURN Oplusimgsensor_power_fan53870_21127(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
	//    int PCB_Version;
	int fan53870_avdd[IMGSENSOR_SENSOR_IDX_MAX_NUM + 1][2]  = {{3,2800},{4,2900},{6,2800},{6,2800}};
	int fan53870_avdd1[IMGSENSOR_SENSOR_IDX_MAX_NUM + 1][2] = {{0,0   },{0,0   },{0,0   },{0,0   }};
	int fan53870_dvdd[IMGSENSOR_SENSOR_IDX_MAX_NUM + 1][2]  = {{1,1100},{2,800 },{0,0   },{0,0   }};
	int fan53870_afvdd[IMGSENSOR_SENSOR_IDX_MAX_NUM + 1][2] = {{7,2800},{0,  0 },{0,0   },{0,0   }};
	if(sensor_idx > IMGSENSOR_SENSOR_IDX_MAX_NUM) {
		return IMGSENSOR_RETURN_ERROR;
	}
	pr_debug("[%s] is_fan53870_pmic:%d pwr_status:%d sensor_idx:%d", __func__, is_fan53870_pmic(), pwr_status, sensor_idx);
	if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
		switch(pin) {
		case IMGSENSOR_HW_PIN_AVDD:
			if(fan53870_avdd[sensor_idx][1]) {
				fan53870_cam_ldo_set_voltage(fan53870_avdd[sensor_idx][0], fan53870_avdd[sensor_idx][1]);
				//pmic_ldo_set_voltage_mv
				return IMGSENSOR_RETURN_SUCCESS;
			}
			break;
		case IMGSENSOR_HW_PIN_AVDD_1:
			if(fan53870_avdd1[sensor_idx][1]) {
				fan53870_cam_ldo_set_voltage(fan53870_avdd1[sensor_idx][0], fan53870_avdd1[sensor_idx][1]);
				return IMGSENSOR_RETURN_SUCCESS;
			}
			break;
		case IMGSENSOR_HW_PIN_DVDD:
			if(fan53870_dvdd[sensor_idx][1]) {
				fan53870_cam_ldo_set_voltage(fan53870_dvdd[sensor_idx][0], fan53870_dvdd[sensor_idx][1]);
				return IMGSENSOR_RETURN_SUCCESS;
			}
			break;
		case IMGSENSOR_HW_PIN_AFVDD:
			if(fan53870_afvdd[sensor_idx][1]) {
				fan53870_cam_ldo_set_voltage(fan53870_afvdd[sensor_idx][0], fan53870_afvdd[sensor_idx][1]);
				return IMGSENSOR_RETURN_SUCCESS;
			}
			break;
		default :
			return IMGSENSOR_RETURN_ERROR;
		}
	} else {
		switch(pin) {
		case IMGSENSOR_HW_PIN_AVDD:
			if(fan53870_avdd[sensor_idx][1]) {
				fan53870_cam_ldo_disable(fan53870_avdd[sensor_idx][0]);
				return IMGSENSOR_RETURN_SUCCESS;
			}
			break;
		case IMGSENSOR_HW_PIN_AVDD_1:
			if(fan53870_avdd1[sensor_idx][1]) {
				fan53870_cam_ldo_disable(fan53870_avdd1[sensor_idx][0]);
				return IMGSENSOR_RETURN_SUCCESS;
			}
			break;
		case IMGSENSOR_HW_PIN_DVDD:
			if(fan53870_dvdd[sensor_idx][1]) {
				fan53870_cam_ldo_disable(fan53870_dvdd[sensor_idx][0]);
				return IMGSENSOR_RETURN_SUCCESS;
			}
			break;
		case IMGSENSOR_HW_PIN_AFVDD:
			if(fan53870_afvdd[sensor_idx][1]) {
				fan53870_cam_ldo_disable(fan53870_afvdd[sensor_idx][0]);
				return IMGSENSOR_RETURN_SUCCESS;
			}
			break;
		default :
			return IMGSENSOR_RETURN_ERROR;
		break;
		}
	}
	return IMGSENSOR_RETURN_ERROR;
}

enum IMGSENSOR_RETURN Oplusimgsensor_power_fan53870_21305(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
	//    int PCB_Version;
	int fan53870_avdd[IMGSENSOR_SENSOR_IDX_MAX_NUM + 1][2]  = {{3,2800},{4,2900},{6,2800},{6,2800}};
	int fan53870_avdd1[IMGSENSOR_SENSOR_IDX_MAX_NUM + 1][2] = {{5,1800},{0,0   },{0,0   },{0,0   }};
	int fan53870_dvdd[IMGSENSOR_SENSOR_IDX_MAX_NUM + 1][2]  = {{1,1100},{2,800},{0,0   },{0,0   }};
	int fan53870_afvdd[IMGSENSOR_SENSOR_IDX_MAX_NUM + 1][2] = {{7,2800},{0,  0 },{0,0   },{0,0   }};
	if(sensor_idx > IMGSENSOR_SENSOR_IDX_MAX_NUM) {
		return IMGSENSOR_RETURN_ERROR;
	}
	pr_debug("[%s] is_fan53870_pmic:%d pwr_status:%d sensor_idx:%d", __func__, is_fan53870_pmic(), pwr_status, sensor_idx);
	if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
		switch(pin) {
		case IMGSENSOR_HW_PIN_AVDD:
			if(fan53870_avdd[sensor_idx][1]) {
				fan53870_cam_ldo_set_voltage(fan53870_avdd[sensor_idx][0], fan53870_avdd[sensor_idx][1]);
				//pmic_ldo_set_voltage_mv
				return IMGSENSOR_RETURN_SUCCESS;
			}
			break;
		case IMGSENSOR_HW_PIN_AVDD_1:
			if(fan53870_avdd1[sensor_idx][1]) {
				fan53870_cam_ldo_set_voltage(fan53870_avdd1[sensor_idx][0], fan53870_avdd1[sensor_idx][1]);
				return IMGSENSOR_RETURN_SUCCESS;
			}
			break;
		case IMGSENSOR_HW_PIN_DVDD:
            if (sensor_idx == 2) {
                return IMGSENSOR_RETURN_ERROR;
            }
			if(fan53870_dvdd[sensor_idx][1]) {
				fan53870_cam_ldo_set_voltage(fan53870_dvdd[sensor_idx][0], fan53870_dvdd[sensor_idx][1]);
				return IMGSENSOR_RETURN_SUCCESS;
				}
			break;
		case IMGSENSOR_HW_PIN_AFVDD:
			if(fan53870_afvdd[sensor_idx][1]) {
				fan53870_cam_ldo_set_voltage(fan53870_afvdd[sensor_idx][0], fan53870_afvdd[sensor_idx][1]);
				return IMGSENSOR_RETURN_SUCCESS;
			}
			break;
		default :
			return IMGSENSOR_RETURN_ERROR;
		}
	} else {
		switch(pin) {
		case IMGSENSOR_HW_PIN_AVDD:
			if(fan53870_avdd[sensor_idx][1]) {
				fan53870_cam_ldo_disable(fan53870_avdd[sensor_idx][0]);
				return IMGSENSOR_RETURN_SUCCESS;
			}
			break;
		case IMGSENSOR_HW_PIN_AVDD_1:
			if(fan53870_avdd1[sensor_idx][1]) {
				fan53870_cam_ldo_disable(fan53870_avdd1[sensor_idx][0]);
				return IMGSENSOR_RETURN_SUCCESS;
			}
			break;
		case IMGSENSOR_HW_PIN_DVDD:
            if (sensor_idx == 2) {
                return IMGSENSOR_RETURN_ERROR;
            }
			if(fan53870_dvdd[sensor_idx][1]) {
				fan53870_cam_ldo_disable(fan53870_dvdd[sensor_idx][0]);
				return IMGSENSOR_RETURN_SUCCESS;
			}
			break;
		case IMGSENSOR_HW_PIN_AFVDD:
			if(fan53870_afvdd[sensor_idx][1]) {
				fan53870_cam_ldo_disable(fan53870_afvdd[sensor_idx][0]);
				return IMGSENSOR_RETURN_SUCCESS;
			}
			break;
		default :
			return IMGSENSOR_RETURN_ERROR;
		break;
		}
	}
	return IMGSENSOR_RETURN_ERROR;
}

enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset_23031(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    static int pmic_avdd[3][2] = {{3, 2800}, {4, 2800}, {6, 2800}};
    int avddIdx = sensor_idx > 2 ? 2 : sensor_idx;

    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_voltage_mv(pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_voltage_mv(1, 1100);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1){
            pmic_ldo_set_voltage_mv(2, 1200);
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx == 0){
            pmic_ldo_set_voltage_mv(7, 2800);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_disable(pmic_avdd[avddIdx][0]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_disable(1);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1){
            pmic_ldo_set_disable(2);
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx == 0){
            pmic_ldo_set_disable(7);
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
    int pcb;
    int pcbVer = 0;
    int pmic_avdd[3][2] = {{3, 2800}, {4, 2900}, {6, 2800}};
    int avddIdx = sensor_idx > IMGSENSOR_SENSOR_IDX_SUB ?
                    IMGSENSOR_SENSOR_IDX_MAIN2 : sensor_idx;
    pcb = get_PCB_Version();

    if (is_project(23231)) {
        return Oplusimgsensor_ldo_powerset_23231(sensor_idx,pin,pwr_status);
    }
#ifdef SENSOR_PLATFORM_NEXT_A
    if(is_project(21081)) {
        return Oplusimgsensor_ldo_powerset_21081(sensor_idx,pin,pwr_status);
    }

    if (is_project(21851) || is_project(21876)) {
        return Oplusimgsensor_ldo_powerset_21851(sensor_idx,pin,pwr_status);
    }

    if (is_project(0x212A1)) {
        return Oplusimgsensor_ldo_powerset_212A1(sensor_idx,pin,pwr_status);
    }

    if (is_project(20181) || is_project(20355)) {
        return Oplusimgsensor_ldo_powerset_20181(sensor_idx,pin,pwr_status,pcb);
    }
#endif

    if (is_project(0x2169E) || is_project(0x2169F) || is_project(0x216C9)
     || is_project(0x216CA) || is_project(21711) || is_project(21712))
        return IMGSENSOR_RETURN_ERROR;

    if (is_project(22693) || is_project(22694) || is_project(22612) || is_project(0x226B1))
        return Oplusimgsensor_ldo_powerset_22693(sensor_idx,pin,pwr_status);
    if (is_project(22629) || is_project(22710) || is_project(22711))
        return Oplusimgsensor_ldo_powerset_22629(sensor_idx,pin,pwr_status);
    if (is_project(23613) || is_project(23686) || is_project(23687) || is_project(23689) || is_project(23707) || is_project(23709))
        return Oplusimgsensor_ldo_powerset_23687(sensor_idx,pin,pwr_status);
    if (is_project(22633) || is_project(22712) || is_project(22713))
        return Oplusimgsensor_ldo_powerset_22633(sensor_idx,pin,pwr_status);

    pr_debug("[%s] pmic_ldo_get_type:%d pwr_status:%d avddIdx:%d",
                    __func__, pmic_ldo_get_type(), pwr_status, avddIdx);

    if (is_project(21690) || is_project(21691) || is_project(21692)
        || is_project(21684) || is_project(21685) || is_project(21686))
        return IMGSENSOR_RETURN_ERROR;

    if (is_project(22083) || is_project(22084) || is_project(22291) || is_project(22292))
        return IMGSENSOR_RETURN_ERROR;

    if (is_project(22087) || is_project(22088) || is_project(22331) || is_project(22332) || is_project(22333) || is_project(22334) || is_project(22869))
        return IMGSENSOR_RETURN_ERROR;

    if (is_project(20291) || is_project(20292) || is_project(20293)
        || is_project(20294) || is_project(20295))
        return IMGSENSOR_RETURN_ERROR;

    if (is_project(20391) || is_project(20392))
        return IMGSENSOR_RETURN_ERROR;

    if (is_project(19165))
        return IMGSENSOR_RETURN_ERROR;

    if ( is_project(0x210A0))
        return IMGSENSOR_RETURN_ERROR;

    if (is_project(20041) || is_project(20042) || is_project(20043))
        return IMGSENSOR_RETURN_ERROR;

    if (is_project(20151) || is_project(20301) || is_project(20302))
        return IMGSENSOR_RETURN_ERROR;

    if(is_project(20633) || is_project(20631))
        return Oplusimgsensor_power_fan53870_20630(sensor_idx,pin,pwr_status);

    if(is_project(20730) || is_project(20731) || is_project(20732))
        return Oplusimgsensor_power_fan53870_20730(sensor_idx,pin,pwr_status);

    if( is_project(20638) || is_project(20639) || is_project(0x206B7))
        return IMGSENSOR_RETURN_ERROR;

    if (is_project(20609) || is_project(0x2060B) || is_project(0x2060A)
        || is_project(0x206FF) || is_project(20796) || is_project(0x2070C) || is_project(20795) || is_project(21747) || is_project(21748)){
        return IMGSENSOR_RETURN_ERROR;
    }

    if (is_project(22277)) {
        return Oplusimgsensor_ldo_powerset_22277(sensor_idx,pin,pwr_status);
    }

    if (is_project(23031)) {
        return Oplusimgsensor_ldo_powerset_23031(sensor_idx,pin,pwr_status);
    }

#ifdef SENSOR_PLATFORM_5G_B
    if (is_project(20075) || is_project(20076))
        return IMGSENSOR_RETURN_ERROR;
    if((is_project(20001)) || (is_project(20002)) || (is_project(20003)) || (is_project(20200)))
        return IMGSENSOR_RETURN_ERROR;
#endif

#ifdef SENSOR_PLATFORM_5G_A
    if ((is_project(19131)) || (is_project(19132)) || (is_project(19420)))
        return IMGSENSOR_RETURN_ERROR;
#endif

    if (is_project(21101) || is_project(21102) || is_project(21235)
        || is_project(21236) || is_project(21831)) {
        return IMGSENSOR_RETURN_ERROR;
    }

    if ( is_project(20015) || is_project(20016) || is_project(21037)
        || is_project(20108) || is_project(20109) || is_project(20307) || is_project(20013))
        return IMGSENSOR_RETURN_ERROR;

#ifdef SENSOR_PLATFORM_4G_20682
    if (is_project(20682) || is_project(20683)|| is_project(19661))
        return IMGSENSOR_RETURN_ERROR;
#endif

    if ( is_project(21041) || is_project(21042))
        return IMGSENSOR_RETURN_ERROR;

    if (is_project(0x2163B) || is_project(0x2163C) || is_project(21639) || is_project(0x2163D) || is_project(0x216CD) || is_project(0x216CE)) {
        return IMGSENSOR_RETURN_ERROR;
    }

    if (is_project(22603) || is_project(22604) || is_project(22609) || is_project(22631)|| is_project(22632)
        || is_project(0x2260A) || is_project(0x2260B) || is_project(22669) || is_project(23602)
        || is_project(0x2266A) || is_project(0x2266B) || is_project(0x2266C)) {
            return IMGSENSOR_RETURN_ERROR;
    }

    if (is_project(22083) || is_project(22291) || is_project(22292) || is_project(22084)) {
        return IMGSENSOR_RETURN_ERROR;
    }

    if (is_project(23053) || is_project(23054) || is_project(23253) || is_project(23055)) {
        return IMGSENSOR_RETURN_ERROR;
    }

    if(is_project(21061)) {
        return Oplusimgsensor_power_fan53870_21061(sensor_idx,pin,pwr_status);
    }

    if(is_project(20615) || is_project(21609) || is_project(20662)
        || is_project(20619) || is_project(21651)) {
        return Oplusimgsensor_power_fan53870_20615(sensor_idx,pin,pwr_status);
    }

    if(is_project(21015) || is_project(21217) || is_project(21016) || is_project(21218)) {
        return Oplusimgsensor_power_fan53870_21015(sensor_idx, pin, pwr_status);
    }

/*20201224 add for 20817 sensor porting*/
    if(is_project(20817) || is_project(20827) || is_project(20831)) {
        /*20210119 add for EVB2*/
        pcbVer = get_PCB_Version();
        pr_debug("Oplusimgsensor_ldo_powerset pcbVer: %d\n", pcbVer);
        if (PCB_VERSION_EVB == pcbVer) {
            return Oplusimgsensor_power_fan53870_20817(sensor_idx,pin,pwr_status);
        } else {
            return Oplusimgsensor_power_fan53870_20817_EVB2(sensor_idx,pin,pwr_status);
        }
    }

    if(is_project(21881) || is_project(21882)) {
        return Oplusimgsensor_power_fan53870_21881(sensor_idx,pin,pwr_status);
        pr_debug("match Oplusimgsensor_power_fan53870_21881\n");
    }

    if(is_project(20601) || is_project(20602) || is_project(20660)){
        return Oplusimgsensor_power_fan53870_20601(sensor_idx,pin,pwr_status);
    }

    if(is_project(20645)){
        return Oplusimgsensor_power_fan53870_20645(sensor_idx,pin,pwr_status);
	}
    if(is_project(21127)) {
        return Oplusimgsensor_power_fan53870_21127(sensor_idx,pin,pwr_status);
    }
    if(is_project(21305)) {
        return Oplusimgsensor_power_fan53870_21305(sensor_idx,pin,pwr_status);
    }

    if(is_project(0x216A0)){
        return IMGSENSOR_RETURN_ERROR;
    }
    if(is_project(22610) || is_project(22705) || is_project(22706)){
        return IMGSENSOR_RETURN_ERROR;
    }
    if(is_project(20095) || is_project(20610) || is_project(20611) || is_project(20680) || is_project(20613) || is_project(20686))
        return IMGSENSOR_RETURN_ERROR;

    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_voltage_mv(
                pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD
            && sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2) {
            pmic_ldo_set_voltage_mv(1, 1200);
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
        case IMGSENSOR_MODULE_ID_HOLITECH:  /* Holitech */
            manufacture = DEVICE_MANUFACUTRE_HOLITECH;
            break;
        case IMGSENSOR_MODULE_ID_AAC:   /*aac optics*/
            manufacture = DEVICE_MANUFACUTRE_AAC;
            break;
        default:
            manufacture = DEVICE_MANUFACUTRE_NA;
    }
    register_device_proc(name, version, manufacture);
}

void Oplusimgsensor_powerstate_notify(bool val)
{
    if (is_project(20171) || is_project(20353)
          || is_project(20391) || is_project(20392) || is_project(21061) || is_project(21305)
          || is_project(20151) || is_project(20301) || is_project(20302) || is_project(21127)
          || is_project(20131) || is_project(20133) || is_project(20255) || is_project(20257)
          || is_project(0x2169E) || is_project(0x2169F) || is_project(0x216C9) || is_project(0x216CA)
          || is_project(19131) || is_project(19132) || is_project(19133) || is_project(19420)
          || is_project(20001) || is_project(20075) || is_project(20076) || is_project(20002)
          || is_project(20003) || is_project(20200) || is_project(20041) || is_project(20042)
          || is_project(20043) || is_project(21235)
          || is_project(20181) || is_project(21081) || is_project(20355)
          || is_project(21101) || is_project(21102) || is_project(21236) || is_project(21831)
          || is_project(20827) || is_project(20831) || is_project(21881) || is_project(21882)
          || is_project(0x2163B) || is_project(0x2163C) || is_project(0x2163D)
          || is_project(21639) || is_project(0x216CD) || is_project(0x216CE)
          || is_project(21711) || is_project(21712) || is_project(0x212A1)
          || is_project(22603) || is_project(22604) || is_project(22609) || is_project(22631)
          || is_project(0x2260A) || is_project(0x2260B) || is_project(22669) || is_project(22632)
          || is_project(21851) || is_project(21876) || is_project(23602)
          || is_project(0x2266A) || is_project(0x2266B) || is_project(0x2266C)
          || is_project(22612) || is_project(22693) || is_project(22694)
          || is_project(22087) || is_project(22088) || is_project(22331) || is_project(22332) || is_project(22333) || is_project(22334) || is_project(22869)
          || is_project(22083) || is_project(22291) || is_project(22292) || is_project(22084) || is_project(22629)
          || is_project(22710) || is_project(22711) || is_project(22633) || is_project(22712) || is_project(22713)
          || is_project(22277) || is_project(23031) || is_project(23231)
          || is_project(23053) || is_project(23054) || is_project(23253) || is_project(23055)
          || is_project(23613) || is_project(23686) || is_project(23687) || is_project(23689) || is_project(23707) || is_project(23709)) {

        static int notify_cnt = 0;

        if (val && (notify_cnt == 0)) {
            pr_info("[%s] val:%d", __func__, val);
            oplus_chg_set_camera_status(val);
            oplus_chg_set_camera_on(val);
            notify_cnt++;
            return;
        }

        if (!val && (notify_cnt != 0)) {
            pr_info("[%s] val:%d", __func__, val);
            oplus_chg_set_camera_status(val);
            oplus_chg_set_camera_on(val);
            notify_cnt = 0;
            return;
        }
    }

    if (is_project(21015) || is_project(21217) || is_project(20609) || is_project(0x2060A) || is_project(0x2060B)
        || is_project(0x2070C) || is_project(20796) || is_project(20795) || is_project(0x206FF)) {
        pr_info("[%s] val:%d", __func__, val);
        oplus_chg_set_camera_status(val);
        oplus_chg_set_camera_on(val);
    }
}

bool oplus_is_system_camera(unsigned int val)
{
    static bool is_system_cam = true;
    if(val != 0xFFFF) {
        is_system_cam = (val != 0);
    }
    pr_debug("oplus_is_system_camera:in %d out %d\n", val, is_system_cam);

    return is_system_cam;
}
