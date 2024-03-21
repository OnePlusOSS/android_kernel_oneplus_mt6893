/*
 * Copyright (C) 2016 MediaTek Inc.
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

#ifndef __IMX882_EEPROM_H__
#define __IMX882_EEPROM_H__

#include "kd_camera_typedef.h"

#define Sleep(ms) mdelay(ms)

#define IMX882_EEPROM_SLAVE_ADDRESS 0xA0
#define IMX882_MAX_OFFSET      0xFFFF

#define OTP_SPC_OFFSET 0x2E20
#define SPC_SIZE 384

#define OTP_QSC_OFFSET 0x2210
#define QSC_SIZE 3072

struct EEPROM_PDAF_INFO {
    kal_uint16 SPC_addr;
    unsigned int SPC_size;
};

/*
 * LRC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_imx882_23231_SPC(kal_uint16 *data);

unsigned int read_imx882_23231_QSC(kal_uint16 *data);

#endif

