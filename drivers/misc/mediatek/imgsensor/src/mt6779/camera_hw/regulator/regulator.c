// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "regulator.h"

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include <soc/oplus/system/oplus_project.h>
#endif
static const int regulator_voltage[] = {
	REGULATOR_VOLTAGE_0,
	REGULATOR_VOLTAGE_1000,
	REGULATOR_VOLTAGE_1100,
	REGULATOR_VOLTAGE_1200,
	REGULATOR_VOLTAGE_1210,
	REGULATOR_VOLTAGE_1220,
	REGULATOR_VOLTAGE_1500,
	REGULATOR_VOLTAGE_1800,
	REGULATOR_VOLTAGE_2500,
	REGULATOR_VOLTAGE_2800,
	REGULATOR_VOLTAGE_2900,
};

struct REGULATOR_CTRL regulator_control[REGULATOR_TYPE_MAX_NUM] = {
	{"vcama"},
	{"vcamd"},
	{"vcamio"},
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{"vcama1"},
	{"vcamd1"},
	{"vcamaf"},
	#endif
};

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static struct regulator *gVCamA = NULL;

struct regulator *regulator_get_regVCAMAF(void)
{
	struct IMGSENSOR *pimgsensor = &gimgsensor;
	return regulator_get(&((pimgsensor->hw.common.pplatform_device)->dev), "vcamaf");
}
EXPORT_SYMBOL(regulator_get_regVCAMAF);
#endif
static struct REGULATOR reg_instance;

static enum IMGSENSOR_RETURN regulator_init(
	void *pinstance,
	struct IMGSENSOR_HW_DEVICE_COMMON *pcommon)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int type, idx, ret;
	char str_regulator_name[LENGTH_FOR_SNPRINTF];

	for (idx = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		idx < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		idx++) {
		for (type = 0;
			type < REGULATOR_TYPE_MAX_NUM;
			type++) {
			memset(str_regulator_name, 0,
				sizeof(str_regulator_name));
			ret = snprintf(str_regulator_name,
				sizeof(str_regulator_name),
				"cam%d_%s",
				idx,
				regulator_control[type].pregulator_type);
			if (ret < 0)
				return ret;
			preg->pregulator[idx][type] = regulator_get_optional(
					&pcommon->pplatform_device->dev,
					str_regulator_name);
			if (IS_ERR(preg->pregulator[idx][type])) {
				preg->pregulator[idx][type] = NULL;
				pr_info("regulator[%d][%d] %s fail!\n",
					idx, type, str_regulator_name);
			}
			atomic_set(&preg->enable_cnt[idx][type], 0);
		}
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_release(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int type, idx;
	struct regulator *pregulator = NULL;
	atomic_t *enable_cnt = NULL;

	for (idx = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		idx < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		idx++) {

		for (type = 0; type < REGULATOR_TYPE_MAX_NUM; type++) {
			pregulator = preg->pregulator[idx][type];
			enable_cnt = &preg->enable_cnt[idx][type];
			if (pregulator != NULL) {
				for (; atomic_read(enable_cnt) > 0; ) {
					regulator_disable(pregulator);
					atomic_dec(enable_cnt);
				}
			}
		}
	}
	return IMGSENSOR_RETURN_SUCCESS;
}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
static int kdVAPowerOn(enum IMGSENSOR_HW_PIN_STATE pin_state)
{
    struct IMGSENSOR *pimgsensor = &gimgsensor;
    gVCamA = regulator_get(&((pimgsensor->hw.common.pplatform_device)->dev), "vcama");

    if (pin_state == IMGSENSOR_HW_PIN_STATE_LEVEL_0)
    {
        if (regulator_set_voltage(gVCamA,
            regulator_voltage[IMGSENSOR_HW_PIN_STATE_LEVEL_0],
            regulator_voltage[IMGSENSOR_HW_PIN_STATE_LEVEL_0]))
        {
            pr_info("[regulator]fail to regulator_set_voltage, powerId:%d\n",
            regulator_voltage[IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
        }
        if (regulator_disable(gVCamA))
        {
            pr_info("[regulator]fail to regulator_disable gVCamIO\n");
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    else
    {
        if (regulator_set_voltage(gVCamA,
            regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0],
            regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]))
            {
                pr_info("[regulator]fail to regulator_set_voltage, powerId:%d\n",
                regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
            }
        if (regulator_enable(gVCamA))
            {
                pr_info("[regulator]fail to regulator_enable\n");
                return IMGSENSOR_RETURN_ERROR;
            }
    }
    return IMGSENSOR_RETURN_SUCCESS;
}
#endif
#ifdef OPLUS_FEATURE_CAMERA_COMMON
extern struct IMGSENSOR gimgsensor;
static struct regulator *regulator_reinit(void *pinstance, int sensor_idx, int type)
{
    struct REGULATOR *preg = (struct REGULATOR *)pinstance;
    char str_regulator_name[LENGTH_FOR_SNPRINTF];
    struct IMGSENSOR *pimgsensor = &gimgsensor;
    snprintf(str_regulator_name,
            sizeof(str_regulator_name),
            "cam%d_%s",
            sensor_idx,
            regulator_control[type].pregulator_type);
    preg->pregulator[sensor_idx][type] =
        regulator_get(&((pimgsensor->hw.common.pplatform_device)->dev), str_regulator_name);
    pr_err("reinit %s regulator[%d][%d] = %pK\n", str_regulator_name,
            sensor_idx, type, preg->pregulator[sensor_idx][type]);
    msleep(10);
    if(IS_ERR(preg->pregulator[sensor_idx][type])){
        pr_err("regulator reinit fail %pK\n",preg->pregulator[sensor_idx][type]);
        return NULL;
    } else {
    	pr_err("regulator reinit success");
    	return preg->pregulator[sensor_idx][type];
    }
}
#endif

static enum IMGSENSOR_RETURN regulator_set(
	void *pinstance,
	enum IMGSENSOR_SENSOR_IDX   sensor_idx,
	enum IMGSENSOR_HW_PIN       pin,
	enum IMGSENSOR_HW_PIN_STATE pin_state)
{
	struct regulator     *pregulator;
	struct REGULATOR     *preg = (struct REGULATOR *)pinstance;
	int reg_type_offset;
	atomic_t             *enable_cnt;

	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if (pin > IMGSENSOR_HW_PIN_AFVDD   ||
		pin < IMGSENSOR_HW_PIN_AVDD    ||
		pin_state < IMGSENSOR_HW_PIN_STATE_LEVEL_0 ||
		pin_state >= IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH)
		return IMGSENSOR_RETURN_ERROR;
	#else
	if (pin > IMGSENSOR_HW_PIN_DOVDD   ||
	    pin < IMGSENSOR_HW_PIN_AVDD    ||
	    pin_state < IMGSENSOR_HW_PIN_STATE_LEVEL_0 ||
	    pin_state >= IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH ||
	    sensor_idx < 0)
		return IMGSENSOR_RETURN_ERROR;
	#endif

    #ifdef OPLUS_FEATURE_CAMERA_COMMON
	if ((is_project(19357) || is_project(19354) || is_project(19358) || is_project(19359))
		&& (sensor_idx == IMGSENSOR_SENSOR_IDX_SUB || sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2 || sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN3) && (pin == IMGSENSOR_HW_PIN_AVDD))
	{
		int ret = kdVAPowerOn(pin_state);
		if (ret)
			pr_info("kdVAPowerOn failed ret:%d", ret);
		return ret;
	}
    #endif

	reg_type_offset = REGULATOR_TYPE_VCAMA;

	pregulator =
		preg->pregulator[sensor_idx][
			reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if(IS_ERR(pregulator)){
            	pregulator = regulator_reinit(preg, sensor_idx, reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD);
		if (IS_ERR(pregulator)) {
                	return IMGSENSOR_RETURN_ERROR;
		}
        }
	#endif

	enable_cnt =
		&preg->enable_cnt[sensor_idx][
			reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

	if (pregulator) {
		if (pin_state != IMGSENSOR_HW_PIN_STATE_LEVEL_0) {
			if (regulator_set_voltage(pregulator,
				regulator_voltage[
				pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0],
				regulator_voltage[
				pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0])) {

				pr_info(
				  "[regulator]fail to regulator_set_voltage, powertype:%d powerId:%d\n",
				  pin,
				  regulator_voltage[
				  pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
			}
			if (regulator_enable(pregulator)) {
				pr_info(
				"[regulator]fail to regulator_enable, powertype:%d powerId:%d\n",
				pin,
				regulator_voltage[
				  pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
				return IMGSENSOR_RETURN_ERROR;
			}
			atomic_inc(enable_cnt);
		} else {
			if (regulator_is_enabled(pregulator))
				PK_DBG("[regulator]%d is enabled\n", pin);

			if (regulator_disable(pregulator)) {
				pr_info(
					"[regulator]fail to regulator_disable, powertype: %d\n",
					pin);
				return IMGSENSOR_RETURN_ERROR;
			}
			atomic_dec(enable_cnt);
		}
	} else {
		pr_info("regulator == NULL %d %d %d\n",
				reg_type_offset,
				pin,
				IMGSENSOR_HW_PIN_AVDD);
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_dump(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int i, j;

	for (j = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		j < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		j++) {

		for (i = REGULATOR_TYPE_VCAMA;
		i < REGULATOR_TYPE_MAX_NUM;
		i++) {
			if (!preg->pregulator[j][i])
				continue;
			if (regulator_is_enabled(preg->pregulator[j][i]) &&
				atomic_read(&preg->enable_cnt[j][i]) != 0)
				PK_DBG("index= %d %s = %d\n",
					j,
					regulator_control[i].pregulator_type,
					regulator_get_voltage(
						preg->pregulator[j][i]));
		}
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

static struct IMGSENSOR_HW_DEVICE device = {
	.id        = IMGSENSOR_HW_ID_REGULATOR,
	.pinstance = (void *)&reg_instance,
	.init      = regulator_init,
	.set       = regulator_set,
	.release   = regulator_release,
	.dump      = regulator_dump
};

enum IMGSENSOR_RETURN imgsensor_hw_regulator_open(
	struct IMGSENSOR_HW_DEVICE **pdevice)
{
	*pdevice = &device;
	return IMGSENSOR_RETURN_SUCCESS;
}

