/*
 * Copyright (C) 2018 MediaTek Inc.
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
#include "eeprom_i2c_dev.h"
#include <linux/types.h>
#include <soc/oplus/system/oplus_project.h>

static enum EEPROM_I2C_DEV_IDX gi2c_dev_sel[IMGSENSOR_SENSOR_IDX_MAX_NUM] = {
	I2C_DEV_IDX_1, /* main */
	I2C_DEV_IDX_2, /* sub */
	I2C_DEV_IDX_3, /* main2 */
	I2C_DEV_IDX_1, /* sub2 */
};

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static enum EEPROM_I2C_DEV_IDX gi2c_dev_sel_even[IMGSENSOR_SENSOR_IDX_MAX_NUM] = {
	I2C_DEV_IDX_1, /* main */
	I2C_DEV_IDX_2, /* sub */
	I2C_DEV_IDX_1, /* main2 */
	I2C_DEV_IDX_2, /* sub2 */
	I2C_DEV_IDX_3, /* main3 */
};
#endif

enum EEPROM_I2C_DEV_IDX get_i2c_dev_sel(enum IMGSENSOR_SENSOR_IDX idx)
{
	if (idx >= IMGSENSOR_SENSOR_IDX_MIN_NUM &&
		idx < IMGSENSOR_SENSOR_IDX_MAX_NUM){
		#ifdef OPLUS_FEATURE_CAMERA_COMMON
		if (is_project(20761) || is_project(20762) || is_project(20764) || is_project(20766) || is_project(20767) ||
		    is_project(0x2167A) || is_project(0x2167B) || is_project(0x2167C) || is_project(0x2167D) ||
		    is_project(0x216AF) || is_project(0x216B0) || is_project(0x216B1))
			return gi2c_dev_sel_even[idx];
		else
			return gi2c_dev_sel[idx];
		#else
		return gi2c_dev_sel[idx];
		#endif
	}
	return I2C_DEV_IDX_1;
}

int gi2c_dev_timing[I2C_DEV_IDX_MAX] = {
	100, /* dev1, 100k */
	100, /* dev2, 100k */
	100, /* dev3, 100k */
};

