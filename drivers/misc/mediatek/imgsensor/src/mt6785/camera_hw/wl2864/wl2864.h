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

#ifndef __IMGSENSOR_HW_WL2864_h__
#define __IMGSENSOR_HW_WL2864_h__

#include "imgsensor_hw.h"
#include "imgsensor_common.h"
#include "../../../../../camera_ldo/camera_ldo.h"

typedef enum  {
	EXTLDO_REGULATOR_VOLTAGE_0    = 0,
	EXTLDO_REGULATOR_VOLTAGE_1000 = 1000,
	EXTLDO_REGULATOR_VOLTAGE_1050 = 1050,
	EXTLDO_REGULATOR_VOLTAGE_1100 = 1100,
	EXTLDO_REGULATOR_VOLTAGE_1200 = 1200,
	EXTLDO_REGULATOR_VOLTAGE_1210 = 1210,
	EXTLDO_REGULATOR_VOLTAGE_1220 = 1220,
	EXTLDO_REGULATOR_VOLTAGE_1500 = 1500,
	EXTLDO_REGULATOR_VOLTAGE_1800 = 1800,
	EXTLDO_REGULATOR_VOLTAGE_2500 = 2500,
	EXTLDO_REGULATOR_VOLTAGE_2800 = 2800,
	EXTLDO_REGULATOR_VOLTAGE_2900 = 2900,
}EXTLDO_REGULATOR_VOLTAGE;

struct WL2864_LDOMAP{
	enum IMGSENSOR_SENSOR_IDX idx;
	enum IMGSENSOR_HW_PIN hwpin;
	CAMERA_LDO_SELECT wl2864ldo;
};

#ifdef OPLUS_FEATURE_CAMERA_COMMON
/*Houbing.Peng@ODM Cam.Drv 20200117 avoid wl2864 power up confict*/
struct wl2864 {
	struct mutex         *pwl2864_mutex;
};
#endif

enum IMGSENSOR_RETURN imgsensor_hw_wl2864_open(
	struct IMGSENSOR_HW_DEVICE **pdevice);

#endif

