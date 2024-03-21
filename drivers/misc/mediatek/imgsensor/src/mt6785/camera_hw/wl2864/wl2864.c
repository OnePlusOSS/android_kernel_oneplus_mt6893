/*
 * Copyright (C) 2017 MediaTek Inc.
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

#include "wl2864.h"
#include "../../../../../camera_ldo/camera_ldo.h"

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include <soc/oplus/system/oplus_project.h>
#include <soc/oplus/system/oplus_project_oldcdt.h>

/*Houbing.Peng@ODM Cam.Drv 20200117 avoid wl2864 power up confict*/
static struct wl2864 wl2864_instance;
#endif

struct WL2864_LDOMAP  ldolist[] = {
	{IMGSENSOR_SENSOR_IDX_MAIN, AVDD, CAMERA_LDO_AVDD2},//for rear main(AVDD)
	{IMGSENSOR_SENSOR_IDX_MAIN, AFVDD, CAMERA_LDO_VDDAF},//for main(AFVDD)
	{IMGSENSOR_SENSOR_IDX_SUB, AVDD, CAMERA_LDO_VDDOIS},//for front(AVDD)
	{IMGSENSOR_SENSOR_IDX_SUB, DVDD, CAMERA_LDO_DVDD2},//for front(DVDD)
	{IMGSENSOR_SENSOR_IDX_MAIN2, DVDD, CAMERA_LDO_DVDD1},//for main2(DVDD)
	{IMGSENSOR_SENSOR_IDX_MAIN2, AVDD, CAMERA_LDO_AVDD1},//for main2(AVDD)
	{IMGSENSOR_SENSOR_IDX_SUB2, AVDD, CAMERA_LDO_VDDIO},//for rear main3 and main4(sub2)(AVDD)
	{IMGSENSOR_SENSOR_IDX_MAIN3, AVDD, CAMERA_LDO_VDDIO},//for rear main3 and main4(sub2)(AVDD)
};

#ifdef OPLUS_FEATURE_CAMERA_COMMON
struct WL2864_LDOMAP  ldolist_20682[] = {
	{IMGSENSOR_SENSOR_IDX_MAIN, AVDD, CAMERA_LDO_AVDD1},//for rear main(AVDD)
	{IMGSENSOR_SENSOR_IDX_MAIN, AVDD2, CAMERA_LDO_AVDD2},//for rear main(AVDD2)
	{IMGSENSOR_SENSOR_IDX_MAIN, AFVDD, CAMERA_LDO_VDDAF},//for main(AFVDD)
	{IMGSENSOR_SENSOR_IDX_SUB, AVDD, CAMERA_LDO_VDDOIS},//for front(AVDD)
	{IMGSENSOR_SENSOR_IDX_SUB, DVDD, CAMERA_LDO_DVDD2},//for front(DVDD)
	{IMGSENSOR_SENSOR_IDX_MAIN2, DVDD, CAMERA_LDO_DVDD1},//for main2(DVDD)
	{IMGSENSOR_SENSOR_IDX_MAIN2, AVDD, CAMERA_LDO_VDDOIS},//for main2(AVDD)
	{IMGSENSOR_SENSOR_IDX_SUB2, AVDD, CAMERA_LDO_VDDIO},//for rear main3 and main4(sub2)(AVDD)
	{IMGSENSOR_SENSOR_IDX_MAIN3, AVDD, CAMERA_LDO_VDDIO},//for rear main3 and main4(sub2)(AVDD)
};
#endif

static const int extldo_regulator_voltage[] = {
	EXTLDO_REGULATOR_VOLTAGE_0,
	EXTLDO_REGULATOR_VOLTAGE_1000,
	EXTLDO_REGULATOR_VOLTAGE_1050,
	EXTLDO_REGULATOR_VOLTAGE_1100,
	EXTLDO_REGULATOR_VOLTAGE_1200,
	EXTLDO_REGULATOR_VOLTAGE_1210,
	EXTLDO_REGULATOR_VOLTAGE_1220,
	EXTLDO_REGULATOR_VOLTAGE_1500,
	EXTLDO_REGULATOR_VOLTAGE_1800,
	EXTLDO_REGULATOR_VOLTAGE_2500,
	EXTLDO_REGULATOR_VOLTAGE_2800,
	EXTLDO_REGULATOR_VOLTAGE_2900,
};


static enum IMGSENSOR_RETURN wl2864_init(void *pinstance, struct IMGSENSOR_HW_DEVICE_COMMON *pcommon)
{
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	/*Houbing.Peng@ODM Cam.Drv 20200117 avoid wl2864 power up confict*/
	struct wl2864 *pinst = (struct wl2864 *)pinstance;
	pinst->pwl2864_mutex = &pcommon->pinctrl_mutex;
	#endif

	return IMGSENSOR_RETURN_SUCCESS;
}
static enum IMGSENSOR_RETURN wl2864_release(void *instance)
{

	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN wl2864_set(
	void *pinstance,
	enum IMGSENSOR_SENSOR_IDX   sensor_idx,
	enum IMGSENSOR_HW_PIN       pin,
	enum IMGSENSOR_HW_PIN_STATE pin_state)
{
	int i,ret;
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	/*Houbing.Peng@ODM Cam.Drv 20200117 avoid wl2864 power up confict*/
	struct wl2864 *pinst = (struct wl2864 *)pinstance;
	#endif
	pr_debug("%s stoneadd hwpin=%d idx=%d pinstate=%d\n", __func__, pin,sensor_idx, pin_state);

	if (pin < IMGSENSOR_HW_PIN_AVDD ||
		   pin > IMGSENSOR_HW_PIN_DOVDD ||
		   pin_state < IMGSENSOR_HW_PIN_STATE_LEVEL_0 ||
		   pin_state > IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH)
			ret = IMGSENSOR_RETURN_ERROR;

	if (is_project(OPLUS_20682)) {
		static int operator = OPERATOR_UNKOWN;
		operator = get_Operator_Version();
		if (operator == OPERATOR_20682_SALA_A_ASIA_SIMPLE
			|| operator == OPERATOR_20682_SALA_A_All_BAND
			|| operator == OPERATOR_20682_SALA_A_All_BAND_VIETNAM
			|| operator == OPERATOR_20682_SALA_A_INTERNATIONAL
			|| operator == OPERATOR_20682_SALA_LITE_INTERNATIONAL
			/*|| operator == OPERATOR_20682_SALA_LITE_VODAFONE*/) {
			ldolist_20682[6].wl2864ldo = CAMERA_LDO_AVDD1;
		}
		pr_debug("%s ++++++wl2864 sala-a compatible with sala main2(AVDD) ldo(%d) operator=%d\n",__func__,
							ldolist_20682[6].wl2864ldo, operator);
		for(i=0;i<(sizeof(ldolist_20682)/sizeof(ldolist_20682[0]));i++) {
			if ((pin == ldolist_20682[i].hwpin) && (sensor_idx == ldolist_20682[i].idx)) {
				if(pin_state > IMGSENSOR_HW_PIN_STATE_LEVEL_0) {
					pr_debug("%s stoneadd power on ++++++wl2864  ldo(%d) to  %d mV !\n",__func__,
							ldolist_20682[i].wl2864ldo,
							extldo_regulator_voltage[pin_state-EXTLDO_REGULATOR_VOLTAGE_0]);
					#ifdef OPLUS_FEATURE_CAMERA_COMMON
					/*Houbing.Peng@ODM Cam.Drv 20200117 avoid wl2864 power up confict*/
					mutex_lock(pinst->pwl2864_mutex);
					camera_ldo_set_ldo_value(ldolist_20682[i].wl2864ldo,extldo_regulator_voltage[pin_state-EXTLDO_REGULATOR_VOLTAGE_0]);
					camera_ldo_set_en_ldo(ldolist_20682[i].wl2864ldo,1);
					mutex_unlock(pinst->pwl2864_mutex);
					#else
					camera_ldo_set_ldo_value(ldolist_20682[i].wl2864ldo,extldo_regulator_voltage[pin_state-EXTLDO_REGULATOR_VOLTAGE_0]);
					camera_ldo_set_en_ldo(ldolist_20682[i].wl2864ldo,1);
					#endif
				} else {
					pr_debug("%s stoneadd power off ++++++wl2864  ldo(%d) \n",__func__, ldolist_20682[i].wl2864ldo);
					camera_ldo_set_en_ldo(ldolist_20682[i].wl2864ldo,0);
				}
				break;
			}
		}
	} else {
		for(i=0;i<(sizeof(ldolist)/sizeof(ldolist[0]));i++) {
			if ((pin == ldolist[i].hwpin) && (sensor_idx == ldolist[i].idx)) {
				//pr_debug("%s stoneadd got the wl2864 ldo(%d) to %d mV ! \n",__func__, ldolist[i].wl2864ldo,extldo_regulator_voltage[pin_state-EXTLDO_REGULATOR_VOLTAGE_0]);

				if(pin_state > IMGSENSOR_HW_PIN_STATE_LEVEL_0) {
					pr_debug("%s stoneadd power on ++++++wl2864  ldo(%d) to  %d mV !\n",__func__,
							ldolist[i].wl2864ldo,
							extldo_regulator_voltage[pin_state-EXTLDO_REGULATOR_VOLTAGE_0]);
					#ifdef ODM_HQ_EDIT
					/*Houbing.Peng@ODM Cam.Drv 20200117 avoid wl2864 power up confict*/
					mutex_lock(pinst->pwl2864_mutex);
					camera_ldo_set_ldo_value(ldolist[i].wl2864ldo,extldo_regulator_voltage[pin_state-EXTLDO_REGULATOR_VOLTAGE_0]);
					camera_ldo_set_en_ldo(ldolist[i].wl2864ldo,1);
					mutex_unlock(pinst->pwl2864_mutex);
					#else
					camera_ldo_set_ldo_value(ldolist[i].wl2864ldo,extldo_regulator_voltage[pin_state-EXTLDO_REGULATOR_VOLTAGE_0]);
					camera_ldo_set_en_ldo(ldolist[i].wl2864ldo,1);
					#endif
				} else {
					pr_debug("%s stoneadd power off ++++++wl2864  ldo(%d) \n",__func__, ldolist[i].wl2864ldo);
					camera_ldo_set_en_ldo(ldolist[i].wl2864ldo,0);
				}
				break;
			}
		}
	}
	ret = IMGSENSOR_RETURN_SUCCESS;

	return ret;
}

static struct IMGSENSOR_HW_DEVICE device = {
	.init      = wl2864_init,
	.set       = wl2864_set,
	.release   = wl2864_release,
	.id        = IMGSENSOR_HW_ID_WL2864,
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	/*Houbing.Peng@ODM Cam.Drv 20200117 avoid wl2864 power up confict*/
	.pinstance = (void *)&wl2864_instance,
	#endif
};

enum IMGSENSOR_RETURN imgsensor_hw_wl2864_open(
	struct IMGSENSOR_HW_DEVICE **pdevice)
{
	*pdevice = &device;
	return IMGSENSOR_RETURN_SUCCESS;
}

