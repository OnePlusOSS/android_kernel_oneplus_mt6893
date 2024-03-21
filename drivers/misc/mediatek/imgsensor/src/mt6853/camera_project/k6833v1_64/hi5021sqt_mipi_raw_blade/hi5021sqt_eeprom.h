/*
 * Copyright (C) 2022 MediaTek Inc.
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

#ifndef __hi5021sqt_EEPROM_H__
#define __hi5021sqt_EEPROM_H__

#include "kd_camera_typedef.h"

/*
 * XGC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_hi5021sqt_XGC(BYTE *data);

/*
 * QGC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_hi5021sqt_QGC(BYTE *data);

#endif

