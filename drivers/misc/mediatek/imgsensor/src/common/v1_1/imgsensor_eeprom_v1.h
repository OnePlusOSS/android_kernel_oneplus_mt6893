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

#ifndef __IMGSENSOR_EEPROM_CTRL_H__
#define __IMGSENSOR_EEPROM_CTRL_H__

#include <linux/mutex.h>

#include "imgsensor_sensor.h"
#include "imgsensor_common.h"
#include "kd_camera_typedef.h"
#include "kd_camera_feature.h"
#include "kd_imgsensor_define.h"

typedef struct {
    /*eeprom IIC Slave addr*/
    u16 i4SlaveAddr[IMGSENSOR_SENSOR_IDX_MAX_NUM];
}IMGSENSOR_EEPROM_MODULE_INFO;

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
                       u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId);

extern kal_uint16 read_cmos_eeprom_8(kal_uint16 addr, kal_uint16 slaveaddr);
extern kal_int32 call_eepromwrite_Service(ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereodata, kal_uint16 i4SlaveAddr);
extern void read_eepromData(kal_uint8 *uData, kal_uint16 dataAddr,
                            kal_uint16 dataLens, kal_uint16 slaveAddr);
#endif
