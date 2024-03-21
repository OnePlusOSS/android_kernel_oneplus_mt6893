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

#ifndef __IMX766_EEPROM_H__
#define __IMX766_EEPROM_H__

#include "kd_camera_typedef.h"

#define Sleep(ms) mdelay(ms)
#define IMX766_EEPROM_SLAVE_ADDRESS 0xA2
#define IMX766_EEPROM_READ_ID  0xA3
#define IMX766_EEPROM_WRITE_ID 0xA2
#define IMX766_MAX_OFFSET      0xFFFF

#define MTK_IDENTITY_VALUE 0x010B00FF
#define LRC_SIZE 140
#define DCC_SIZE 96

struct EEPROM_PDAF_INFO {
    kal_uint16 LRC_addr;
    unsigned int LRC_size;
    kal_uint16 DCC_addr;
    unsigned int DCC_size;
};

enum EEPROM_PDAF_INFO_FMT {
    MTK_FMT = 0,
    OP_FMT,
    FMT_MAX
};
/*
 * LRC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_imx766_21015_LRC(BYTE *data);

/*
 * DCC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_imx766_21015_DCC(BYTE *data);

#endif

