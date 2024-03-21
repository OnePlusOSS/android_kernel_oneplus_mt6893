// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "regulator.h"
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include "imgsensor.h"
#include <soc/oplus/system/oplus_project.h>
#endif
#ifdef OPLUS_FEATURE_CAMERA_COMMON
static bool regulator_status[IMGSENSOR_SENSOR_IDX_MAX_NUM][REGULATOR_TYPE_MAX_NUM] = {{false}};
static void check_for_regulator_get(struct REGULATOR *preg,
struct device *pdevice, unsigned int sensor_index, unsigned int regulator_index);
static void check_for_regulator_put(struct REGULATOR *preg, unsigned int sensor_index, unsigned int regulator_index);
static struct device_node *of_node_record;
static struct IMGSENSOR_HW_DEVICE_COMMON *ghw_device_common;
static DEFINE_MUTEX(g_regulator_state_mutex);
#endif /*OPLUS_FEATURE_CAMERA_COMMON*/
static const int regulator_voltage[] = {
	REGULATOR_VOLTAGE_0,
	REGULATOR_VOLTAGE_1000,
	REGULATOR_VOLTAGE_1050,
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
	{"vcamaf"},
	{"vcama1"},
	{"vcamd1"}
	#endif
};

static struct REGULATOR reg_instance;

#ifdef OPLUS_FEATURE_CAMERA_COMMON
extern struct IMGSENSOR gimgsensor;
struct regulator *regulator_get_regVCAMAF(void)
{
	struct IMGSENSOR *pimgsensor = &gimgsensor;
	return regulator_get(&((pimgsensor->hw.common.pplatform_device)->dev), "vcamaf");
}
EXPORT_SYMBOL(regulator_get_regVCAMAF);
struct regulator *regulator_get_regVCAMAF_Chaka(int sensor_idx)
{
	struct IMGSENSOR *pimgsensor = &gimgsensor;
	char str_regulator_name[LENGTH_FOR_SNPRINTF];
	snprintf(str_regulator_name, sizeof(str_regulator_name), "cam%d_%s", sensor_idx, "vcamaf");
	return regulator_get(&((pimgsensor->hw.common.pplatform_device)->dev), str_regulator_name);
}
EXPORT_SYMBOL(regulator_get_regVCAMAF_Chaka);

static struct regulator *regVCAMAF = NULL;
struct regulator *regulator_get_regVCAMAF_ALICE(void)
{
	struct IMGSENSOR *pimgsensor = &gimgsensor;
	return regulator_get(&((pimgsensor->hw.common.pplatform_device)->dev), "vcammainaf");
}
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
extern struct IMGSENSOR gimgsensor;
struct regulator *regulator_get_regVCAMAF_20609(void)
{
	struct IMGSENSOR *pimgsensor = &gimgsensor;
	return regulator_get(&((pimgsensor->hw.common.pplatform_device)->dev), "vcammainaf");
}
EXPORT_SYMBOL(regulator_get_regVCAMAF_20609);

struct regulator *regulator_get_regVCAMAF_bladeb(void)
{
	struct IMGSENSOR *pimgsensor = &gimgsensor;
	return regulator_get(&((pimgsensor->hw.common.pplatform_device)->dev), "vtp");
}
EXPORT_SYMBOL(regulator_get_regVCAMAF_bladeb);
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
extern unsigned int pmic_config_interface(unsigned int RegNum,
				   unsigned int val,
				   unsigned int MASK,
				   unsigned int SHIFT);
#endif /*OPLUS_FEATURE_CAMERA_COMMON*/

static enum IMGSENSOR_RETURN regulator_init(
	void *pinstance,
	struct IMGSENSOR_HW_DEVICE_COMMON *pcommon)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int type, idx, ret = 0;
	char str_regulator_name[LENGTH_FOR_SNPRINTF];

	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	ghw_device_common = pcommon;
	of_node_record = pcommon->pplatform_device->dev.of_node;
	#endif /*OPLUS_FEATURE_CAMERA_COMMON*/
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if (is_project(22629) || is_project(22710) || is_project(22711)) {
		pmic_config_interface(0x1b14, 0xffff, 0xFFFF, 0x0);
		pmic_config_interface(0x1c54, 0x1c72, 0xFFFF, 0x0);
	}
	#endif /*OPLUS_FEATURE_CAMERA_COMMON*/
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
				PK_INFO("ERROR: regulator[%d][%d]  %s fail!\n",
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
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if (is_project(22629) || is_project(22710) || is_project(22711)) {
		pmic_config_interface(0x1b14, 0xfbff, 0xFFFF, 0x0);
	}
	#endif /*OPLUS_FEATURE_CAMERA_COMMON*/
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
	if (is_project(22083) || is_project(22084) || is_project(22291) || is_project(22292) || is_project(22631) || is_project(22632) || is_project(23602)
	|| is_project(23053) || is_project(23054) || is_project(23253)
	|| is_project(21101) || is_project(21102) || is_project(21235) || is_project(21236) || is_project(21041) || is_project(21042)
	|| is_project(22087) || is_project(22088) || is_project(22331) || is_project(22332) || is_project(22333) || is_project(22334) || is_project(22869)
	|| is_project(0x216A0) || is_project(22610) || is_project(22705) || is_project(22706) || is_project(21831) || is_project(0x2163B) || is_project(0x2163C) || is_project(0x2163D)
	|| is_project(21639) || is_project(0x216CD) || is_project(0x216CE) || is_project(22603) || is_project(22604) || is_project(22609)
	|| is_project(0x2260A) || is_project(0x2260B) || is_project(22669) || is_project(0x2266A) || is_project(0x2266B) || is_project(0x2266C)) {
		PK_DBG("to set current regulator pin:%d", pin);
		regVCAMAF = regulator_get_regVCAMAF_ALICE();
		if (IS_ERR(regVCAMAF)) {
			PK_PR_ERR("get main af regulator fail");
			regVCAMAF = NULL;
		}
		if (pin == IMGSENSOR_HW_PIN_AFVDD)
		{
			if (pin_state == IMGSENSOR_HW_PIN_STATE_LEVEL_0)
			{
				if (regulator_set_voltage(regVCAMAF,regulator_voltage[IMGSENSOR_HW_PIN_STATE_LEVEL_0],regulator_voltage[IMGSENSOR_HW_PIN_STATE_LEVEL_0]))
				{
					PK_PR_ERR("[regulator]fail to regulator_set_voltage, powerId:%d\n", regulator_voltage[IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
				}
				if (regulator_disable(regVCAMAF))
				{
					PK_PR_ERR("[regulator]fail to regulator_disable gVCamIO\n");
					return IMGSENSOR_RETURN_ERROR;
				}
			}
			else
			{
				if (regulator_set_voltage(regVCAMAF, regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0],regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]))
				{
					PK_PR_ERR("[regulator]fail to regulator_set_voltage, powerId:%d\n",regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
				}
				if (regulator_enable(regVCAMAF))
				{
					PK_PR_ERR("[regulator]fail to regulator_enable\n");
					return IMGSENSOR_RETURN_ERROR;
				}
			}
			return IMGSENSOR_RETURN_SUCCESS;
		}
	}
	if (is_project(20609) || is_project(0x2060A) || is_project(0x2060B) || is_project(0x2070C) || is_project(0x206FF) || is_project(20795) || is_project(20796) || is_project(0x2070C) || is_project(0x2070B) || is_project(0x2070E) || is_project(0x210A0) || is_project(21747) || is_project(21748))
	{
		regVCAMAF = regulator_get_regVCAMAF_20609();
		if (IS_ERR(regVCAMAF)) {
			PK_PR_ERR("get main af regulator fail");
			regVCAMAF = NULL;
		}
		if (pin == IMGSENSOR_HW_PIN_AFVDD)
		{
			if (pin_state == IMGSENSOR_HW_PIN_STATE_LEVEL_0)
			{
				if (regulator_set_voltage(regVCAMAF,regulator_voltage[IMGSENSOR_HW_PIN_STATE_LEVEL_0],regulator_voltage[IMGSENSOR_HW_PIN_STATE_LEVEL_0]))
				{
					PK_PR_ERR("[regulator]fail to regulator_set_voltage, powerId:%d\n", regulator_voltage[IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
				}
				if (regulator_disable(regVCAMAF))
				{
					PK_PR_ERR("[regulator]fail to regulator_disable gVCamIO\n");
					return IMGSENSOR_RETURN_ERROR;
				}
			}
			else
			{
				if (regulator_set_voltage(regVCAMAF, regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0],regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]))
				{
					PK_PR_ERR("[regulator]fail to regulator_set_voltage, powerId:%d\n",regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
				}
				if (regulator_enable(regVCAMAF))
				{
					PK_PR_ERR("[regulator]fail to regulator_enable\n");
					return IMGSENSOR_RETURN_ERROR;
				}
			}
			return IMGSENSOR_RETURN_SUCCESS;
		}
	}
	if (is_project(21711) || is_project(21712))
	{
		regVCAMAF = regulator_get_regVCAMAF();
		if (IS_ERR(regVCAMAF)) {
			PK_PR_ERR("get main af regulator fail");
			regVCAMAF = NULL;
		}
		if (pin == IMGSENSOR_HW_PIN_AFVDD)
		{
			if (pin_state == IMGSENSOR_HW_PIN_STATE_LEVEL_0)
			{
				if (regulator_set_voltage(regVCAMAF,regulator_voltage[IMGSENSOR_HW_PIN_STATE_LEVEL_0],regulator_voltage[IMGSENSOR_HW_PIN_STATE_LEVEL_0]))
				{
					PK_PR_ERR("[regulator] fail to regulator_set_voltage, powerId:%d\n", regulator_voltage[IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
				}
				if (regulator_disable(regVCAMAF))
				{
					PK_PR_ERR("[regulator] fail to regulator_disable gVCamIO\n");
					return IMGSENSOR_RETURN_ERROR;
				}
			}
			else
			{
				if (regulator_set_voltage(regVCAMAF, regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0],regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]))
				{
					PK_PR_ERR("[regulator] fail to regulator_set_voltage, powerId:%d\n",regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
				}
				if (regulator_enable(regVCAMAF))
				{
					PK_PR_ERR("[regulator] fail to regulator_enable\n");
					return IMGSENSOR_RETURN_ERROR;
				}
			}
			return IMGSENSOR_RETURN_SUCCESS;
		}
	}
	if (is_project(22609) || is_project(0x2266B) || is_project(0x2266C) || is_project(22669) || is_project(0x2266A) || is_project(0x2260B)) {
		regVCAMAF = regulator_get_regVCAMAF_bladeb();
		if (IS_ERR(regVCAMAF)) {
			PK_PR_ERR("get bladeb main af regulator fail");
			regVCAMAF = NULL;
		}
		if (pin == IMGSENSOR_HW_PIN_AFVDD)
		{
			if (pin_state == IMGSENSOR_HW_PIN_STATE_LEVEL_0)
			{
				if (regulator_set_voltage(regVCAMAF,regulator_voltage[IMGSENSOR_HW_PIN_STATE_LEVEL_0],regulator_voltage[IMGSENSOR_HW_PIN_STATE_LEVEL_0]))
				{
					PK_PR_ERR("[regulator] fail to regulator_set_voltage, powerId:%d\n", regulator_voltage[IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
				}
				if (regulator_disable(regVCAMAF))
				{
					PK_PR_ERR("[regulator] fail to regulator_disable gVCamIO\n");
					return IMGSENSOR_RETURN_ERROR;
				}
			}
			else
			{
				if (regulator_set_voltage(regVCAMAF, regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0],regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]))
				{
					PK_PR_ERR("[regulator] fail to regulator_set_voltage, powerId:%d\n",regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
				}
				if (regulator_enable(regVCAMAF))
				{
					PK_PR_ERR("[regulator] fail to regulator_enable\n");
					return IMGSENSOR_RETURN_ERROR;
				}
			}
			return IMGSENSOR_RETURN_SUCCESS;
		}
	}
#endif
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if (pin > IMGSENSOR_HW_PIN_FAN53870_ENABLE   ||
	#else
	if (pin > IMGSENSOR_HW_PIN_DOVDD   ||
	#endif
	    pin < IMGSENSOR_HW_PIN_AVDD    ||
	    pin_state < IMGSENSOR_HW_PIN_STATE_LEVEL_0 ||
	    pin_state >= IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH)
		return IMGSENSOR_RETURN_ERROR;

	reg_type_offset = REGULATOR_TYPE_VCAMA;

	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if (ghw_device_common)
		check_for_regulator_get(preg, &ghw_device_common->pplatform_device->dev, sensor_idx,(reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));
	#endif /*OPLUS_FEATURE_CAMERA_COMMON*/
	pregulator = preg->pregulator[(unsigned int)sensor_idx][
		reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if(IS_ERR(pregulator)) {
		pregulator = regulator_reinit(preg, sensor_idx, reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD);
		if (IS_ERR(pregulator)) {
			PK_PR_ERR(
				"[regulator]fail to regulator_reinit, sensor_idx:%d pin:%d reg_type_offset:%d\n",
				sensor_idx,
				pin,
				reg_type_offset);
			return IMGSENSOR_RETURN_ERROR;
		}
	}
	#endif
	enable_cnt = &preg->enable_cnt[(unsigned int)sensor_idx][
		reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

	if (pregulator) {
		if (pin_state != IMGSENSOR_HW_PIN_STATE_LEVEL_0) {
			if (regulator_set_voltage(pregulator,
				regulator_voltage[
				pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0],
				regulator_voltage[
				pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0])) {

				PK_PR_ERR(
				  "[regulator]fail to regulator_set_voltage, powertype:%d powerId:%d\n",
				  pin,
				  regulator_voltage[
				  pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
			}
			#ifdef OPLUS_FEATURE_CAMERA_COMMON
			if (!regulator_set_load(pregulator, 10000)) {
				PK_DBG("set load success");
			}
			#endif

			if (regulator_enable(pregulator)) {
				PK_PR_ERR(
				"[regulator]fail to regulator_enable, powertype:%d powerId:%d\n",
				pin,
				regulator_voltage[
				  pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
				#ifdef OPLUS_FEATURE_CAMERA_COMMON
				check_for_regulator_put(preg, sensor_idx, (reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));
				#endif /*OPLUS_FEATURE_CAMERA_COMMON*/
				return IMGSENSOR_RETURN_ERROR;
			}
			atomic_inc(enable_cnt);
		} else {
			if (regulator_is_enabled(pregulator))
				PK_DBG("[regulator]%d is enabled\n", pin);

			if (regulator_disable(pregulator)) {
				PK_PR_ERR(
					"[regulator]fail to regulator_disable, powertype: %d\n",
					pin);
				#ifdef OPLUS_FEATURE_CAMERA_COMMON
				check_for_regulator_put(preg, sensor_idx,(reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));
				#endif /*OPLUS_FEATURE_CAMERA_COMMON*/
				return IMGSENSOR_RETURN_ERROR;
			}
			#ifdef OPLUS_FEATURE_CAMERA_COMMON
			check_for_regulator_put(preg, sensor_idx,(reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));
			#endif /*OPLUS_FEATURE_CAMERA_COMMON*/
			atomic_dec(enable_cnt);
		}
	} else {
		PK_PR_ERR("regulator == NULL %d %d %d\n",
				reg_type_offset,
				pin,
				IMGSENSOR_HW_PIN_AVDD);
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static void check_for_regulator_get(struct REGULATOR *preg,
	struct device *pdevice, unsigned int sensor_index,
	unsigned int regulator_index)
{
	struct device_node *pof_node = NULL;
	char str_regulator_name[LENGTH_FOR_SNPRINTF];

	PK_PR_ERR("[%s]sensor_idx:%d regulator_idx: %d\n", __func__, sensor_index, regulator_index);

	if (!preg || !pdevice) {
		PK_PR_ERR("Fatal: Null ptr.preg:%pK,pdevice:%pK\n", preg, pdevice);
		return;
	}

	if (sensor_index >= IMGSENSOR_SENSOR_IDX_MAX_NUM ||
			regulator_index >= REGULATOR_TYPE_MAX_NUM ) {
		PK_PR_ERR("[%s]Invalid sensor_idx:%d regulator_idx: %d\n", __func__, sensor_index, regulator_index);
		return;
	}

	mutex_lock(&g_regulator_state_mutex);

	if (regulator_status[sensor_index][regulator_index] == false) {
		pof_node = pdevice->of_node;
		pdevice->of_node = of_node_record;

		snprintf(str_regulator_name, sizeof(str_regulator_name), "cam%d_%s", sensor_index, regulator_control[regulator_index].pregulator_type);
		preg->pregulator[sensor_index][regulator_index] = regulator_get(pdevice, str_regulator_name);

		if (preg != NULL)
			regulator_status[sensor_index][regulator_index] = true;
		else
			PK_PR_ERR("get regulator failed.\n");

		pdevice->of_node = pof_node;
	}

	mutex_unlock(&g_regulator_state_mutex);

	return;
}

static void check_for_regulator_put(struct REGULATOR *preg,
	unsigned int sensor_index, unsigned int regulator_index)
{
	if (!preg) {
		PK_PR_ERR("Fatal: Null ptr.\n");
		return;
	}

	PK_PR_ERR("[%s]sensor_idx:%d regulator_idx: %d\n", __func__, sensor_index, regulator_index);

	if (sensor_index >= IMGSENSOR_SENSOR_IDX_MAX_NUM || regulator_index >= REGULATOR_TYPE_MAX_NUM ) {
		PK_PR_ERR("[%s]Invalid sensor_idx:%d regulator_idx: %d\n", __func__, sensor_index, regulator_index);
		return;
	}

	mutex_lock(&g_regulator_state_mutex);

	if (regulator_status[sensor_index][regulator_index] == true) {
		regulator_put(preg->pregulator[sensor_index][regulator_index]);
		preg->pregulator[sensor_index][regulator_index]=NULL;
		regulator_status[sensor_index][regulator_index] = false;
	}

	mutex_unlock(&g_regulator_state_mutex);
	return;
}
#endif /*OPLUS_FEATURE_CAMERA_COMMON*/
static enum IMGSENSOR_RETURN regulator_dump(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int i, j;
	#ifndef OPLUS_FEATURE_CAMERA_COMMON
	int enable = 0;
	#endif

	for (j = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		j < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		j++) {

		for (i = REGULATOR_TYPE_VCAMA;
		i < REGULATOR_TYPE_MAX_NUM;
		i++) {
			#ifdef OPLUS_FEATURE_CAMERA_COMMON
			if (IS_ERR_OR_NULL(preg->pregulator[j][i])) {
				PK_DBG("regulator idx(%d, %d) alread release ", j, i);
				return IMGSENSOR_RETURN_SUCCESS;
			} else {
				if (regulator_is_enabled(preg->pregulator[j][i]) &&
					atomic_read(&preg->enable_cnt[j][i]) != 0)
					PK_DBG("index= %d %s = %d\n",
						j,
						regulator_control[i].pregulator_type,
						regulator_get_voltage(
						preg->pregulator[j][i]));
			}
			#else
			if (!preg->pregulator[j][i])
				continue;

			if (regulator_is_enabled(preg->pregulator[j][i]) &&
				atomic_read(&preg->enable_cnt[j][i]) != 0)
				enable = 1;
			else
				enable = 0;

			PK_DBG("[sensor_dump][regulator] index= %d, %s = %d, enable = %d\n",
				j,
				regulator_control[i].pregulator_type,
				regulator_get_voltage(preg->pregulator[j][i]),
				enable);
			#endif
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

