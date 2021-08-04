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
#include "imgsensor_hwcfg_custom.h"

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
                       u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId);

kal_uint16 Eeprom_1ByteDataRead(kal_uint16 addr, kal_uint16 slaveaddr);
enum CUSTOM_CAMERA_ERROR_CODE_ENUM Eeprom_Control(
            enum IMGSENSOR_SENSOR_IDX sensor_idx,
            MSDK_SENSOR_FEATURE_ENUM feature_id,
            unsigned char *feature_para,
            kal_uint32 sensorID);
void Eeprom_DataRead(kal_uint8 *uData,
                     kal_uint16 dataAddr,
                     kal_uint16 dataLens,
                     kal_uint16 slaveAddr);
enum IMGSENSOR_RETURN Eeprom_CallWriteService(
                    ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereoData);
#endif
