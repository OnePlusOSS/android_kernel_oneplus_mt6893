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

#include "imgsensor_eeprom.h"
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>
#define DUMP_EEPROM 0

/*GC8054: 0xA2*/
/*static IMGSENSOR_EEPROM_MODULE_INFO gEpInfoObj = {
    .i4SlaveAddr = {0xA0, 0xA8, 0xA2, 0xA8},
};*/

kal_uint16 Eeprom_1ByteDataRead(kal_uint16 addr, kal_uint16 slaveaddr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 1, slaveaddr);
    return get_byte;
}

void Eeprom_DataRead(kal_uint8 *uData, kal_uint16 dataAddr,
                            kal_uint16 dataLens, kal_uint16 slaveAddr)
{
    int dataCnt = 0;
    for (dataCnt = 0; dataCnt < dataLens; dataCnt++) {
        uData[dataCnt] = (u8)Eeprom_1ByteDataRead(dataAddr+dataCnt, slaveAddr);
    }
}

#if 0
static enum IMGSENSOR_RETURN Eeprom_QcomHeaderCheck(kal_uint8 *header)
{
    unsigned int   sum = 0, i = 0;
    if (header == NULL) {
        pr_debug("Eeprom_QcomHeaderCheck is NULL");
        return IMGSENSOR_RETURN_ERROR;
    }

    for (i = 0; i < OPLUS_STEREO_LENGTH_QCOMHEADER - 1; i++)
    {
        sum += header[i];
    }

    sum &= 0xFF;
    sum ^= 0xFF;

    if (sum == header[OPLUS_STEREO_LENGTH_QCOMHEADER - 1])
        return IMGSENSOR_RETURN_SUCCESS;
    else
        return IMGSENSOR_RETURN_ERROR;
}
#endif

static enum IMGSENSOR_RETURN Eeprom_TableWrite(kal_uint16 addr, kal_uint8 *para,
                                    kal_uint32 len, kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    char pusendcmd[WRITE_DATA_MAX_LENGTH+2];
    pusendcmd[0] = (char)(addr >> 8);
    pusendcmd[1] = (char)(addr & 0xFF);

    memcpy(&pusendcmd[2], para, len);

    ret = iBurstWriteReg((kal_uint8 *)pusendcmd , (len + 2), i4SlaveAddr);

    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteProtectEnable(kal_uint16 enable, kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    char pusendcmd[3];
    pusendcmd[0] = 0x80;
    pusendcmd[1] = 0x00;
    if (enable)
        pusendcmd[2] = 0x0E;
    else
        pusendcmd[2] = 0x00;

    ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
    pr_info("[%s] enable: %d ret: %d", enable, ret);
    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteProtectEnable22277OV64B(kal_uint16 enable, kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    char pusendcmd[3];
    if (enable){
        pusendcmd[0] = 0x80;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = 0x01;
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
        pr_info("[Eeprom_WriteProtectEnable22277OV64B] device lock enable: %d ret: %d", enable, ret);

    } else{
        pusendcmd[0] = 0x80;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = 0x00;
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
        pr_info("[Eeprom_WriteProtect] device unlock enable: %d ret: %d,i4SlaveAddr:0x%x", enable, ret,i4SlaveAddr);
    }
    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteProtectEnable22277IMX355(kal_uint16 enable, kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    char pusendcmd[3];
    if (enable){
        pusendcmd[0] = 0xE0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = 0xA3;
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
        pr_info("[Eeprom_WriteProtectEnable22277IMX355] device lock enable: %d ret: %d", enable, ret);
        msleep(6);
        pusendcmd[0] = 0xA0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = 0xFF;
    }
    else{
        pusendcmd[0] = 0xC0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = 0x00;
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
        pr_info("[Eeprom_WriteProtectEnable22277IMX355] device unlock enable: %d ret: %d", enable, ret);
        msleep(6);
        pusendcmd[0] = 0xE0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = 0xA2;
    }

    ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
    pr_info("[Eeprom_WriteProtectEnable22277IMX355] enable: %d ret: %d", enable, ret);
    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteProtectEnable22277IMX709(kal_uint16 enable, kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    char pusendcmd[3];
    if (enable) {
        pusendcmd[0] = 0xE0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = (i4SlaveAddr & 0xFE) | 0x01;
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
        pr_info("[Eeprom_WriteProtectIMX709] device lock enable: %d ret: %d,i4SlaveAddr:0x%x", enable, ret,i4SlaveAddr);
    } else {
        pusendcmd[0] = 0xE0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = (i4SlaveAddr & 0xFE);
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
        pr_info("[Eeprom_WriteProtectIMX709] device unlock enable: %d ret: %d,i4SlaveAddr:0x%x", enable, ret,i4SlaveAddr);
    }
    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteProtectEnable23231IMX882(kal_uint16 enable, kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    char pusendcmd[3];
    if (enable){
        pusendcmd[0] = 0xE0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = (i4SlaveAddr & 0xFE) | 0x01;
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
        pr_info("[Eeprom_WriteProtectEnable23231IMX882] device lock enable: %d ret: %d", enable, ret);

    } else{
        pusendcmd[0] = 0xE0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = (i4SlaveAddr & 0xFE);
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
        pr_info("[Eeprom_WriteProtect] device unlock enable: %d ret: %d,i4SlaveAddr:0x%x", enable, ret,i4SlaveAddr);
    }
    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteProtectEnable23231IMX355(kal_uint16 enable, kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    char pusendcmd[3];
    if (enable){
        pusendcmd[0] = 0xE0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = 0xA3;
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
        pr_info("[Eeprom_WriteProtectEnable23231IMX355] device lock enable: %d ret: %d", enable, ret);
        msleep(6);
        pusendcmd[0] = 0xA0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = 0xFF;
    }
    else{
        pusendcmd[0] = 0xC0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = 0x00;
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
        pr_info("[Eeprom_WriteProtectEnable23231IMX355] device unlock enable: %d ret: %d", enable, ret);
        msleep(6);
        pusendcmd[0] = 0xE0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = 0xA2;
    }

    ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
    pr_info("[Eeprom_WriteProtectEnable23231IMX355] enable: %d ret: %d", enable, ret);
    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteProtectEnable23231IMX709(kal_uint16 enable, kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    char pusendcmd[3];
    if (enable) {
        pusendcmd[0] = 0xE0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = (i4SlaveAddr & 0xFE) | 0x01;
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
        pr_info("[Eeprom_WriteProtectIMX709] device lock enable: %d ret: %d,i4SlaveAddr:0x%x", enable, ret,i4SlaveAddr);
    } else {
        pusendcmd[0] = 0xE0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = (i4SlaveAddr & 0xFE);
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
        pr_info("[Eeprom_WriteProtectIMX709] device unlock enable: %d ret: %d,i4SlaveAddr:0x%x", enable, ret,i4SlaveAddr);
    }
    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteProtectEnable21015(kal_uint16 enable, kal_uint16 i4SlaveAddr)
{
    kal_uint8 eeprom_vendor_id;
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    //M:pusendcmd[0], L:pusendcmd[1] is RegAddress.
    char pusendcmd[3];
    eeprom_vendor_id = Eeprom_1ByteDataRead(0x11, i4SlaveAddr);
    pr_debug("[Eeprom_WriteProtectEnable21015]eeprom_vendor_id:0x%x, enable:%d", eeprom_vendor_id, enable);
    if(0x01 == eeprom_vendor_id){
        //P24C256C eeprom.
        pusendcmd[0] = 0xA0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = 0x00;
        if(enable){
            //Open write-protection
            pusendcmd[2] = 0x0E;
        }
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
        pr_debug("[Eeprom_WriteProtectEnable21015]P24C256C Open/Close value[%d], ret(%d), slaveId:0x%x", pusendcmd[2], ret, i4SlaveAddr);
    }else{
        //FM24C256E eeprom
        //1��enable write
        i4SlaveAddr = 0xB0|(i4SlaveAddr&0x0E);
        pusendcmd[0] = 0xFF;
        pusendcmd[1] = 0x35;
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 2, i4SlaveAddr);
        pr_debug("[Eeprom_WriteProtectEnable21015]FM24C256E write 0xFF35 ret(%d) slaveId(0x%x)", ret, i4SlaveAddr);
        msleep(6);
        //2��write protective flag on or off
        pusendcmd[0] = 0x06;
        pusendcmd[1] = 0xCA;
        if (enable){
            pusendcmd[2] = ((i4SlaveAddr&0x0E)<<4)|0X02;
        }else{
            pusendcmd[2] = (i4SlaveAddr&0x0E)<<4;
        }
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
        pr_debug("[Eeprom_WriteProtectEnable21015]FM24C256E write 0x06CA value:0x%x, ret(%d),slaveId(0x%x)", pusendcmd[2], ret, i4SlaveAddr);
    }
    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteProtectEnable20817(kal_uint16 enable, kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    char pusendcmd[3];

    //1、enable write
    pusendcmd[0] = 0xFF;
    pusendcmd[1] = 0x35;
    ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 2, i4SlaveAddr);
    pr_debug("[Eeprom_WriteProtectEnable20817] write 0xFF35 ret(%d) addr(0x%x)", ret, i4SlaveAddr);
    msleep(6);

    //2、write protective flag on or off
    pusendcmd[0] = 0x06;
    pusendcmd[1] = 0xCA;
    if (enable)
        pusendcmd[2] = 0x02;
    else
        pusendcmd[2] = 0x00;
    ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
    pr_debug("[Eeprom_WriteProtectEnable20817] enable: %d ret: %d", enable, ret);
    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteProtectEnable21881(kal_uint16 enable, kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    char pusendcmd[3];

    //1、enable write
    pusendcmd[0] = 0xFF;
    pusendcmd[1] = 0x35;
    ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 2, i4SlaveAddr);
    pr_debug("[Eeprom_WriteProtectEnable21881] write 0xFF35 ret(%d) addr(0x%x)", ret, i4SlaveAddr);
    msleep(6);

    //2、write protective flag on or off
    pusendcmd[0] = 0x06;
    pusendcmd[1] = 0xCA;
    if (enable)
        pusendcmd[2] = 0x02;
    else
        pusendcmd[2] = 0x00;
    ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
    pr_debug("[Eeprom_WriteProtectEnable21881] enable: %d ret: %d", enable, ret);
    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteProtectEnable21305(kal_uint16 enable, kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    char pusendcmd[3];

    pusendcmd[0] = 0xA0;
    pusendcmd[1] = 0x00;
    pusendcmd[2] = 0x00;
    if(enable){
        //Open write-protection
        pusendcmd[2] = 0x0E;
    }
    ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
    pr_debug("[Eeprom_WriteProtectEnable21305]P24C256C Open/Close value[%d], ret(%d), slaveId:0x%x", pusendcmd[2], ret, i4SlaveAddr);
    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteProtectEnable21305IMX355(kal_uint16 enable, kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    char pusendcmd[3];
    if (enable){
        pusendcmd[0] = 0xE0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = 0xA3;
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
        pr_info("[Eeprom_WriteProtectEnable21305IMX355] device lock enable: %d ret: %d", enable, ret);
        msleep(6);
        pusendcmd[0] = 0xA0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = 0xFF;
    }
    else{
        pusendcmd[0] = 0xC0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = 0x00;
        ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
        pr_info("[Eeprom_WriteProtectEnable21305IMX355] device unlock enable: %d ret: %d", enable, ret);
        msleep(6);
        pusendcmd[0] = 0xE0;
        pusendcmd[1] = 0x00;
        pusendcmd[2] = 0xA2;
    }

    ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);
    pr_info("[Eeprom_WriteProtectEnable21305IMX355] enable: %d ret: %d", enable, ret);
    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteInfomatch(
                ACDK_SENSOR_ENGMODE_STEREO_STRUCT *pStereoData)
{
    kal_uint16 data_base, data_length;
    kal_uint32 uSensorId;
    if(pStereoData == NULL) {
        pr_debug("SET_SENSOR_OTP pStereoData is NULL!");
        return IMGSENSOR_RETURN_ERROR;
    }

    uSensorId   = pStereoData->uSensorId;
    data_base   = pStereoData->baseAddr;
    data_length = pStereoData->dataLength;

    pr_info("Eeprom_WriteInfomatch uSensorId 0x%x DA:0x%x DL:%d",uSensorId, data_base, data_length);

    switch (uSensorId) {
        #if defined(OV64B_MIPI_RAW)
        case OV64B_SENSOR_ID:
        {
            if (data_length == CALI_DATA_MASTER_LENGTH_20730
                && data_base == OV64B_STEREO_START_ADDR_20730) {
                pr_debug("ov64b DA:0x%x DL:%d", data_base, data_length);
            } else if (data_length == CALI_DATA_MASTER_LENGTH_QCOM
                && data_base == OV64B_STEREO_START_ADDR) {
                pr_debug("ov64b DA:0x%x DL:%d", data_base, data_length);
            } else if (data_length == CALI_DATA_MASTER_LENGTH
                && data_base == OV64B_STEREO_START_ADDR) {
                pr_debug("ov64b DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("ov64b Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(OV64B_MIPI_RAW212A1)
        case OV64B_SENSOR_ID_212A1:
        {
            if (data_length == CALI_DATA_MASTER_LENGTH
                && data_base == OV64B_STEREO_START_ADDR) {
                pr_debug("ov64b 212A1 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("ov64b 212A1 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(IMX355_MIPI_RAW212A1)
        case IMX355_SENSOR_ID_212A1:
        {
            if (data_length == CALI_DATA_SLAVE_LENGTH
                && data_base == IMX355_STEREO_START_ADDR) {
                pr_debug("imx355 212A1 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("imx355 212A1 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(OV64B_MIPI_RAW21851)
        case OV64B_SENSOR_ID_21851:
        {
            if (data_length == CALI_DATA_MASTER_LENGTH
                && data_base == OV64B_STEREO_START_ADDR) {
                pr_debug("ov64b 21851 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("ov64b 21851 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(IMX355_MIPI_RAW21851)
        case IMX355_SENSOR_ID_21851:
        {
            if (data_length == CALI_DATA_SLAVE_LENGTH
                && data_base == IMX355_STEREO_START_ADDR) {
                pr_debug("imx355 21851 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("imx355 21851 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(IMX319_MIPI_RAW)
        case IMX319_SENSOR_ID:
        {
            if (data_length == CALI_DATA_SLAVE_LENGTH
                && data_base == IMX319_STEREO_START_ADDR) {
                pr_debug("imx319 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("imx319 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(OV48B_MIPI_RAW)
        case OV48B_SENSOR_ID:
        {
            if (data_length == CALI_DATA_MASTER_LENGTH
                && data_base == OV48B_STEREO_START_ADDR) {
                pr_debug("ov48b DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("ov48b Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(HI846_MIPI_RAW)
        case HI846_SENSOR_ID:
        {
            if (data_length == CALI_DATA_SLAVE_LENGTH
                && (data_base == HI846_STEREO_START_ADDR_19131
                || data_base == HI846_STEREO_START_ADDR)) {
                pr_debug("hi846 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("hi846 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(IMX355_MIPI_RAW)
        case IMX355_SENSOR_ID:
        {
            if (data_length == CALI_DATA_SLAVE_LENGTH
                && data_base == IMX355_STEREO_START_ADDR) {
                pr_debug("imx355 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("imx355 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif

        #if defined(IMX766_MIPI_RAW21015)
        case IMX766_SENSOR_ID_21015:
        {
            if (data_length == CALI_DATA_MASTER_LENGTH
                && data_base == IMX766_STEREO_START_ADDR) {
                pr_debug("21015 imx766 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("21015 imx766 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(IMX355_MIPI_RAW21015)
        case IMX355_SENSOR_ID_21015:
        {
            if (data_length == CALI_DATA_SLAVE_LENGTH
                && data_base == IMX355_STEREO_START_ADDR) {
                pr_debug("21015 imx355 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("21015 imx355 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(IMX766_MIPI_RAW20817)
        case IMX766_SENSOR_ID_20817:
        {
            if (data_length == CALI_DATA_MASTER_LENGTH
                && data_base == IMX766_STEREO_START_ADDR) {
                pr_debug("imx766 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("imx766 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(IMX355_MIPI_RAW20817)
        case IMX355_SENSOR_ID_20817:
        {
            if (data_length == CALI_DATA_SLAVE_LENGTH
                && data_base == IMX355_STEREO_START_ADDR) {
                pr_debug("imx355 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("imx355 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
	#if defined(IMX766_MIPI_RAW21881)
        case IMX766_SENSOR_ID_21881:
        {
            if (data_length == CALI_DATA_MASTER_LENGTH
                && data_base == IMX766_STEREO_START_ADDR) {
                pr_debug("21881 imx766 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("21881 imx766 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(IMX355_MIPI_RAW21881)
        case IMX355_SENSOR_ID_21881:
        {
            if (data_length == CALI_DATA_SLAVE_LENGTH
                && data_base == IMX355_STEREO_START_ADDR) {
                pr_debug("21881 imx355 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("21881 imx355 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif

        #if defined(IMX766_MIPI_RAW_21305)
        case IMX766_SENSOR_ID_21305:
        {
            if (data_length == CALI_DATA_MASTER_LENGTH
                && data_base == IMX766_STEREO_START_ADDR) {
                pr_debug("21305 imx766 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("21305 imx766 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(IMX355_MIPI_RAW_21305)
        case IMX355_SENSOR_ID_21305:
        {
            if (data_length == CALI_DATA_SLAVE_LENGTH
                && data_base == IMX355_STEREO_START_ADDR) {
                pr_debug("21305 imx355 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("21305 imx355 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(OV64B_MIPI_RAW22277)
        case OV64B_SENSOR_ID22277:
        {
            if (data_length == AESYNC_DATA_LENGTH_SINGLE
                && data_base == OV64B_AESYNC_START_ADDR_MASTER) {
                pr_info("OV64B Aesync DA:0x%x DL:%d", data_base, data_length);
            } else if (data_length == CALI_DATA_MASTER_LENGTH
                && data_base == OV64B_STEREO_START_ADDR_MW_22277) {
                pr_info("OV64B DualCal_mw DA:0x%x DL:%d", data_base, data_length);
            } else if (data_length == CALI_DATA_MASTER_LENGTH
                && data_base == OV64B_STEREO_START_ADDR_MT_22277) {
                pr_info("OV64B DualCal_mt DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_info("OV64B Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(IMX355_MIPI_RAW22277)
        case IMX355_SENSOR_ID22277:
        {
            if (data_length == AESYNC_DATA_LENGTH_SINGLE
                && data_base == IMX355_AESYNC_START_ADDR_WIDE) {
                pr_info("IMX355 Aesync DA:0x%x DL:%d", data_base, data_length);
            } else if (data_length == CALI_DATA_SLAVE_LENGTH
                && data_base == IMX355_STEREO_START_ADDR_22277) {
                pr_info("IMX355 DualCal_mw DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_info("IMX355 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(IMX709_MIPI_RAW22277)
        case IMX709_SENSOR_ID22277:
        {
            if (data_length == AESYNC_DATA_LENGTH_SINGLE
                && data_base == IMX709_AESYNC_START_ADDR_TELE) {
                pr_info("IMX709 Aesync DA:0x%x DL:%d", data_base, data_length);
            } else if (data_length == CALI_DATA_SLAVE_TELE_LENGTH_22277
                && data_base == IMX709_STEREO_START_ADDR_22277) {
                pr_info("IMX709 DualCal_mt DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_info("IMX709 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(IMX882_MIPI_RAW23231)
        case IMX882_SENSOR_ID23231:
        {
            if (data_length == AESYNC_DATA_LENGTH_SINGLE
                && data_base == IMX882_AESYNC_START_ADDR_MASTER) {
                pr_info("IMX882 Aesync DA:0x%x DL:%d", data_base, data_length);
            } else if (data_length == CALI_DATA_MASTER_LENGTH
                && data_base == IMX882_STEREO_START_ADDR_MW_23231) {
                pr_info("IMX882 DualCal_mw DA:0x%x DL:%d", data_base, data_length);
            } else if (data_length == CALI_DATA_MASTER_LENGTH
                && data_base == IMX882_STEREO_START_ADDR_MT_23231) {
                pr_info("IMX882 DualCal_mt DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_info("IMX882 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(IMX355_MIPI_RAW23231)
        case IMX355_SENSOR_ID23231:
        {
            if (data_length == AESYNC_DATA_LENGTH_SINGLE
                && data_base == IMX355_AESYNC_START_ADDR_WIDE) {
                pr_info("IMX355 Aesync DA:0x%x DL:%d", data_base, data_length);
            } else if (data_length == CALI_DATA_SLAVE_LENGTH
                && data_base == IMX355_STEREO_START_ADDR_23231) {
                pr_info("IMX355 DualCal_mw DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_info("IMX355 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(IMX709_MIPI_RAW23231)
        case IMX709_SENSOR_ID23231:
        {
            if (data_length == AESYNC_DATA_LENGTH_SINGLE
                && data_base == IMX709_AESYNC_START_ADDR_TELE) {
                pr_info("IMX709 Aesync DA:0x%x DL:%d", data_base, data_length);
            } else if (data_length == CALI_DATA_SLAVE_TELE_LENGTH_23231
                && data_base == IMX709_STEREO_START_ADDR_23231) {
                pr_info("IMX709 DualCal_mt DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_info("IMX709 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        default:
            pr_debug("Invalid SensorID:0x%x", uSensorId);
            return IMGSENSOR_RETURN_ERROR;
    }

    return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN Eeprom_WriteData(kal_uint16 data_base, kal_uint16 data_length,
                                   kal_uint8 *pData,  kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint16 max_Lens = WRITE_DATA_MAX_LENGTH;
    kal_uint32 idx, idy, i = 0;
    if (pData == NULL || data_length < WRITE_DATA_MAX_LENGTH) {
        pr_err("Eeprom_WriteData pData is NULL or invalid DL:%d",data_length );
        return IMGSENSOR_RETURN_ERROR;
    }

    idx = data_length/max_Lens;
    idy = data_length%max_Lens;
    pr_debug("Eeprom_WriteData: 0x%x %d i4SlaveAddr:0x%x\n",
                data_base,
                data_length,
                i4SlaveAddr);
    /* close write protect */
    Eeprom_WriteProtectEnable(0, i4SlaveAddr);
    msleep(6);
    for (i = 0; i < idx; i++ ) {
        ret = Eeprom_TableWrite((data_base+max_Lens*i),
                &pData[max_Lens*i], max_Lens, i4SlaveAddr);
        if (ret != IMGSENSOR_RETURN_SUCCESS) {
            pr_err("write_eeprom error: i= %d\n", i);
            /* open write protect */
            Eeprom_WriteProtectEnable(1, i4SlaveAddr);
            msleep(6);
            return IMGSENSOR_RETURN_ERROR;
        }
        msleep(6);
    }
    ret = Eeprom_TableWrite((data_base+max_Lens*idx),
            &pData[max_Lens*idx], idy, i4SlaveAddr);
    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("Eeprom_TableWrite error: idx= %d idy= %d\n", idx, idy);
        /* open write protect */
        Eeprom_WriteProtectEnable(1, i4SlaveAddr);
        msleep(6);
        return IMGSENSOR_RETURN_ERROR;
    }
    msleep(6);
    /* open write protect */
    Eeprom_WriteProtectEnable(1, i4SlaveAddr);
    msleep(6);

    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteData_22277_OV64B(kal_uint16 data_base, kal_uint16 data_length,
                                   kal_uint8 *pData,  kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint16 max_Lens = WRITE_DATA_MAX_LENGTH;
    kal_uint32 idx, idy, i = 0;
    if (pData == NULL || data_length < WRITE_DATA_MAX_LENGTH) {
        pr_err("Eeprom_WriteData OV64B pData is NULL or invalid DL:%d",data_length );
        return IMGSENSOR_RETURN_ERROR;
    }

    idx = data_length/max_Lens;
    idy = data_length%max_Lens;
    pr_info("Eeprom_WriteData: 0x%x %d i4SlaveAddr:0x%x\n",
                data_base,
                data_length,
                i4SlaveAddr);
    /* close write protect */
    Eeprom_WriteProtectEnable22277OV64B(0, i4SlaveAddr);
    msleep(6);
    for (i = 0; i < idx; i++ ) {
        ret = Eeprom_TableWrite((data_base+max_Lens*i),
                &pData[max_Lens*i], max_Lens, i4SlaveAddr);
        if (ret != IMGSENSOR_RETURN_SUCCESS) {
            pr_err("write_eeprom OV64B error: i= %d\n", i);
            /* open write protect */
            Eeprom_WriteProtectEnable22277OV64B(1, i4SlaveAddr);
            msleep(6);
            return IMGSENSOR_RETURN_ERROR;
        }
        msleep(6);
    }
    ret = Eeprom_TableWrite((data_base+max_Lens*idx),
            &pData[max_Lens*idx], idy, i4SlaveAddr);
    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("Eeprom_TableWrite OV64B error: idx= %d idy= %d\n", idx, idy);
        /* open write protect */
        Eeprom_WriteProtectEnable22277OV64B(1, i4SlaveAddr);
        msleep(6);
        return IMGSENSOR_RETURN_ERROR;
    }
    msleep(6);
    /* open write protect */
    Eeprom_WriteProtectEnable22277OV64B(1, i4SlaveAddr);
    msleep(6);

    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteData_22277_IMX355(kal_uint16 data_base, kal_uint16 data_length,
                                   kal_uint8 *pData,  kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint16 max_Lens = WRITE_DATA_MAX_LENGTH;
    kal_uint32 idx, idy, i = 0;
    if (pData == NULL || data_length < WRITE_DATA_MAX_LENGTH) {
        pr_err("Eeprom_WriteData IMX355 pData is NULL or invalid DL:%d",data_length );
        return IMGSENSOR_RETURN_ERROR;
    }

    idx = data_length/max_Lens;
    idy = data_length%max_Lens;
    pr_debug("Eeprom_WriteData: 0x%x %d i4SlaveAddr:0x%x\n",
                data_base,
                data_length,
                i4SlaveAddr);
    /* close write protect */
    Eeprom_WriteProtectEnable22277IMX355(0, i4SlaveAddr);
    msleep(6);
    for (i = 0; i < idx; i++ ) {
        ret = Eeprom_TableWrite((data_base+max_Lens*i),
                &pData[max_Lens*i], max_Lens, i4SlaveAddr);
        if (ret != IMGSENSOR_RETURN_SUCCESS) {
            pr_err("write_eeprom IMX355 error: i= %d\n", i);
            /* open write protect */
            Eeprom_WriteProtectEnable22277IMX355(1, i4SlaveAddr);
            msleep(6);
            return IMGSENSOR_RETURN_ERROR;
        }
        msleep(6);
    }
    ret = Eeprom_TableWrite((data_base+max_Lens*idx),
            &pData[max_Lens*idx], idy, i4SlaveAddr);
    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("Eeprom_TableWrite IMX355 error: idx= %d idy= %d\n", idx, idy);
        /* open write protect */
        Eeprom_WriteProtectEnable22277IMX355(1, i4SlaveAddr);
        msleep(6);
        return IMGSENSOR_RETURN_ERROR;
    }
    msleep(6);
    /* open write protect */
    Eeprom_WriteProtectEnable22277IMX355(1, i4SlaveAddr);
    msleep(6);

    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteData_22277_IMX709(kal_uint16 data_base, kal_uint16 data_length,
                                   kal_uint8 *pData,  kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint16 max_Lens = WRITE_DATA_MAX_LENGTH;
    kal_uint32 idx, idy, i = 0;
    if (pData == NULL || data_length < WRITE_DATA_MAX_LENGTH) {
        pr_err("Eeprom_WriteData IMX709 pData is NULL or invalid DL:%d",data_length );
        return IMGSENSOR_RETURN_ERROR;
    }

    idx = data_length/max_Lens;
    idy = data_length%max_Lens;
    pr_debug("Eeprom_WriteData: 0x%x %d i4SlaveAddr:0x%x\n",
                data_base,
                data_length,
                i4SlaveAddr);
    /* close write protect */
    Eeprom_WriteProtectEnable22277IMX709(0, i4SlaveAddr);
    msleep(6);
    for (i = 0; i < idx; i++ ) {
        ret = Eeprom_TableWrite((data_base+max_Lens*i),
                &pData[max_Lens*i], max_Lens, i4SlaveAddr);
        if (ret != IMGSENSOR_RETURN_SUCCESS) {
            pr_err("write_eeprom IMX709 error: i= %d\n", i);
            /* open write protect */
            Eeprom_WriteProtectEnable22277IMX709(1, i4SlaveAddr);
            msleep(6);
            return IMGSENSOR_RETURN_ERROR;
        }
        msleep(6);
    }
    ret = Eeprom_TableWrite((data_base+max_Lens*idx),
            &pData[max_Lens*idx], idy, i4SlaveAddr);
    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("Eeprom_TableWrite IMX709 error: idx= %d idy= %d\n", idx, idy);
        /* open write protect */
        Eeprom_WriteProtectEnable22277IMX709(1, i4SlaveAddr);
        msleep(6);
        return IMGSENSOR_RETURN_ERROR;
    }
    msleep(6);
    /* open write protect */
    Eeprom_WriteProtectEnable22277IMX709(1, i4SlaveAddr);
    msleep(6);

    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteData_23231_IMX882(kal_uint16 data_base, kal_uint16 data_length,
                                   kal_uint8 *pData,  kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint16 max_Lens = WRITE_DATA_MAX_LENGTH;
    kal_uint32 idx, idy, i = 0;
    if (pData == NULL || data_length < WRITE_DATA_MAX_LENGTH) {
        pr_err("Eeprom_WriteData IMX882 pData is NULL or invalid DL:%d",data_length );
        return IMGSENSOR_RETURN_ERROR;
    }

    idx = data_length/max_Lens;
    idy = data_length%max_Lens;
    pr_info("Eeprom_WriteData: 0x%x %d i4SlaveAddr:0x%x\n",
                data_base,
                data_length,
                i4SlaveAddr);
    /* close write protect */
    Eeprom_WriteProtectEnable23231IMX882(0, i4SlaveAddr);
    msleep(6);
    for (i = 0; i < idx; i++ ) {
        ret = Eeprom_TableWrite((data_base+max_Lens*i),
                &pData[max_Lens*i], max_Lens, i4SlaveAddr);
        if (ret != IMGSENSOR_RETURN_SUCCESS) {
            pr_err("write_eeprom IMX882 error: i= %d\n", i);
            /* open write protect */
            Eeprom_WriteProtectEnable23231IMX882(1, i4SlaveAddr);
            msleep(6);
            return IMGSENSOR_RETURN_ERROR;
        }
        msleep(6);
    }
    ret = Eeprom_TableWrite((data_base+max_Lens*idx),
            &pData[max_Lens*idx], idy, i4SlaveAddr);
    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("Eeprom_TableWrite IMX882 error: idx= %d idy= %d\n", idx, idy);
        /* open write protect */
        Eeprom_WriteProtectEnable23231IMX882(1, i4SlaveAddr);
        msleep(6);
        return IMGSENSOR_RETURN_ERROR;
    }
    msleep(6);
    /* open write protect */
    Eeprom_WriteProtectEnable23231IMX882(1, i4SlaveAddr);
    msleep(6);

    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteData_23231_IMX355(kal_uint16 data_base, kal_uint16 data_length,
                                   kal_uint8 *pData,  kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint16 max_Lens = WRITE_DATA_MAX_LENGTH;
    kal_uint32 idx, idy, i = 0;
    if (pData == NULL || data_length < WRITE_DATA_MAX_LENGTH) {
        pr_err("Eeprom_WriteData IMX355 pData is NULL or invalid DL:%d",data_length );
        return IMGSENSOR_RETURN_ERROR;
    }

    idx = data_length/max_Lens;
    idy = data_length%max_Lens;
    pr_debug("Eeprom_WriteData: 0x%x %d i4SlaveAddr:0x%x\n",
                data_base,
                data_length,
                i4SlaveAddr);
    /* close write protect */
    Eeprom_WriteProtectEnable23231IMX355(0, i4SlaveAddr);
    msleep(6);
    for (i = 0; i < idx; i++ ) {
        ret = Eeprom_TableWrite((data_base+max_Lens*i),
                &pData[max_Lens*i], max_Lens, i4SlaveAddr);
        if (ret != IMGSENSOR_RETURN_SUCCESS) {
            pr_err("write_eeprom IMX355 error: i= %d\n", i);
            /* open write protect */
            Eeprom_WriteProtectEnable23231IMX355(1, i4SlaveAddr);
            msleep(6);
            return IMGSENSOR_RETURN_ERROR;
        }
        msleep(6);
    }
    ret = Eeprom_TableWrite((data_base+max_Lens*idx),
            &pData[max_Lens*idx], idy, i4SlaveAddr);
    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("Eeprom_TableWrite IMX355 error: idx= %d idy= %d\n", idx, idy);
        /* open write protect */
        Eeprom_WriteProtectEnable23231IMX355(1, i4SlaveAddr);
        msleep(6);
        return IMGSENSOR_RETURN_ERROR;
    }
    msleep(6);
    /* open write protect */
    Eeprom_WriteProtectEnable23231IMX355(1, i4SlaveAddr);
    msleep(6);

    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteData_23231_IMX709(kal_uint16 data_base, kal_uint16 data_length,
                                   kal_uint8 *pData,  kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint16 max_Lens = WRITE_DATA_MAX_LENGTH;
    kal_uint32 idx, idy, i = 0;
    if (pData == NULL || data_length < WRITE_DATA_MAX_LENGTH) {
        pr_err("Eeprom_WriteData IMX709 pData is NULL or invalid DL:%d",data_length );
        return IMGSENSOR_RETURN_ERROR;
    }

    idx = data_length/max_Lens;
    idy = data_length%max_Lens;
    pr_debug("Eeprom_WriteData: 0x%x %d i4SlaveAddr:0x%x\n",
                data_base,
                data_length,
                i4SlaveAddr);
    /* close write protect */
    Eeprom_WriteProtectEnable23231IMX709(0, i4SlaveAddr);
    msleep(6);
    for (i = 0; i < idx; i++ ) {
        ret = Eeprom_TableWrite((data_base+max_Lens*i),
                &pData[max_Lens*i], max_Lens, i4SlaveAddr);
        if (ret != IMGSENSOR_RETURN_SUCCESS) {
            pr_err("write_eeprom IMX709 error: i= %d\n", i);
            /* open write protect */
            Eeprom_WriteProtectEnable23231IMX709(1, i4SlaveAddr);
            msleep(6);
            return IMGSENSOR_RETURN_ERROR;
        }
        msleep(6);
    }
    ret = Eeprom_TableWrite((data_base+max_Lens*idx),
            &pData[max_Lens*idx], idy, i4SlaveAddr);
    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("Eeprom_TableWrite IMX709 error: idx= %d idy= %d\n", idx, idy);
        /* open write protect */
        Eeprom_WriteProtectEnable23231IMX709(1, i4SlaveAddr);
        msleep(6);
        return IMGSENSOR_RETURN_ERROR;
    }
    msleep(6);
    /* open write protect */
    Eeprom_WriteProtectEnable23231IMX709(1, i4SlaveAddr);
    msleep(6);

    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteData_21015(kal_uint16 data_base, kal_uint16 data_length,
                                   kal_uint8 *pData,  kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint16 max_Lens = WRITE_DATA_MAX_LENGTH;
    kal_uint32 idx, idy, i = 0;
    if (pData == NULL || data_length < WRITE_DATA_MAX_LENGTH) {
        pr_err("Eeprom_WriteData_21015 pData is NULL or invalid DataLen:%d",data_length );
        return IMGSENSOR_RETURN_ERROR;
    }

    idx = data_length/max_Lens;
    idy = data_length%max_Lens;

    //close the eeprom write protection
    Eeprom_WriteProtectEnable21015(0, i4SlaveAddr);

    //Write OPT data.
    msleep(6);
    for (i = 0; i < idx; i++ ) {
        ret = Eeprom_TableWrite((data_base+max_Lens*i),
                &pData[max_Lens*i], max_Lens, i4SlaveAddr);
        if (ret != IMGSENSOR_RETURN_SUCCESS) {
            pr_err("write_eeprom data:0x%x, error: i= %d\n", pData[max_Lens*i], i);
            /* open write protect */
            Eeprom_WriteProtectEnable21015(1, i4SlaveAddr);
            msleep(6);
            return IMGSENSOR_RETURN_ERROR;
        }
        msleep(6);
    }
    ret = Eeprom_TableWrite((data_base+max_Lens*idx),
            &pData[max_Lens*idx], idy, i4SlaveAddr);
    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("Eeprom_TableWrite error: idx= %d idy= %d\n", idx, idy);
        /* open write protect */
        Eeprom_WriteProtectEnable21015(1, i4SlaveAddr);
        msleep(6);
        return IMGSENSOR_RETURN_ERROR;
    }
    msleep(6);
    /* open write protect */
    Eeprom_WriteProtectEnable21015(1, i4SlaveAddr);
    msleep(6);
    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteData_20817(kal_uint16 data_base, kal_uint16 data_length,
                                   kal_uint8 *pData,  kal_uint16 i4DataSlaveAddr, kal_uint16 i4ProtectSlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint16 max_Lens = WRITE_DATA_MAX_LENGTH;
    kal_uint32 idx, idy, i = 0;
    if (pData == NULL || data_length < WRITE_DATA_MAX_LENGTH) {
        pr_err("Eeprom_WriteData_20817 pData is NULL or invalid DL:%d",data_length );
        return IMGSENSOR_RETURN_ERROR;
    }

    idx = data_length/max_Lens;
    idy = data_length%max_Lens;
    pr_debug("Eeprom_WriteData_20817: 0x%x %d i4DataSlaveAddr:0x%x i4ProtectSlaveAddr:0x%x\n",
                data_base,
                data_length,
                i4DataSlaveAddr,
                i4ProtectSlaveAddr);

    /* close write protect */
    Eeprom_WriteProtectEnable20817(0, i4ProtectSlaveAddr);
    msleep(6);
    for (i = 0; i < idx; i++ ) {
        ret = Eeprom_TableWrite((data_base+max_Lens*i),
                &pData[max_Lens*i], max_Lens, i4DataSlaveAddr);
        if (ret != IMGSENSOR_RETURN_SUCCESS) {
            pr_err("write_eeprom error: i= %d\n", i);
            /* open write protect */
            Eeprom_WriteProtectEnable20817(1, i4ProtectSlaveAddr);
            msleep(6);
            return IMGSENSOR_RETURN_ERROR;
        }
        msleep(6);
    }
    ret = Eeprom_TableWrite((data_base+max_Lens*idx),
            &pData[max_Lens*idx], idy, i4DataSlaveAddr);
    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("Eeprom_TableWrite error: idx= %d idy= %d\n", idx, idy);
        /* open write protect */
        Eeprom_WriteProtectEnable20817(1, i4ProtectSlaveAddr);
        msleep(6);
        return IMGSENSOR_RETURN_ERROR;
    }
    msleep(6);
    /* open write protect */
    Eeprom_WriteProtectEnable20817(1, i4ProtectSlaveAddr);
    msleep(6);

    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteData_21881(kal_uint16 data_base, kal_uint16 data_length,
                                   kal_uint8 *pData,  kal_uint16 i4DataSlaveAddr, kal_uint16 i4ProtectSlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint16 max_Lens = WRITE_DATA_MAX_LENGTH;
    kal_uint32 idx, idy, i = 0;
    if (pData == NULL || data_length < WRITE_DATA_MAX_LENGTH) {
        pr_err("Eeprom_WriteData_21881 pData is NULL or invalid DL:%d",data_length );
        return IMGSENSOR_RETURN_ERROR;
    }

    idx = data_length/max_Lens;
    idy = data_length%max_Lens;
    pr_debug("Eeprom_WriteData_21881: 0x%x %d i4DataSlaveAddr:0x%x i4ProtectSlaveAddr:0x%x\n",
                data_base,
                data_length,
                i4DataSlaveAddr,
                i4ProtectSlaveAddr);

    /* close write protect */
    Eeprom_WriteProtectEnable21881(0, i4ProtectSlaveAddr);
    msleep(6);
    for (i = 0; i < idx; i++ ) {
        ret = Eeprom_TableWrite((data_base+max_Lens*i),
                &pData[max_Lens*i], max_Lens, i4DataSlaveAddr);
        if (ret != IMGSENSOR_RETURN_SUCCESS) {
            pr_err("write_eeprom error: i= %d\n", i);
            /* open write protect */
            Eeprom_WriteProtectEnable21881(1, i4ProtectSlaveAddr);
            msleep(6);
            return IMGSENSOR_RETURN_ERROR;
        }
        msleep(6);
    }
    ret = Eeprom_TableWrite((data_base+max_Lens*idx),
            &pData[max_Lens*idx], idy, i4DataSlaveAddr);
    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("Eeprom_TableWrite error: idx= %d idy= %d\n", idx, idy);
        /* open write protect */
        Eeprom_WriteProtectEnable21881(1, i4ProtectSlaveAddr);
        msleep(6);
        return IMGSENSOR_RETURN_ERROR;
    }
    msleep(6);
    /* open write protect */
    Eeprom_WriteProtectEnable21881(1, i4ProtectSlaveAddr);
    msleep(6);

    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteData_21305(kal_uint16 data_base, kal_uint16 data_length,
                                   kal_uint8 *pData,  kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint16 max_Lens = WRITE_DATA_MAX_LENGTH;
    kal_uint32 idx, idy, i = 0;
    if (pData == NULL || data_length < WRITE_DATA_MAX_LENGTH) {
        pr_err("Eeprom_WriteData_21305 pData is NULL or invalid DataLen:%d",data_length );
        return IMGSENSOR_RETURN_ERROR;
    }

    idx = data_length/max_Lens;
    idy = data_length%max_Lens;

    //close the eeprom write protection
    Eeprom_WriteProtectEnable21305(0, i4SlaveAddr);

    //Write OPT data.
    msleep(6);
    for (i = 0; i < idx; i++ ) {
        ret = Eeprom_TableWrite((data_base+max_Lens*i),
                &pData[max_Lens*i], max_Lens, i4SlaveAddr);
        if (ret != IMGSENSOR_RETURN_SUCCESS) {
            pr_err("write_eeprom data:0x%x, error: i= %d\n", pData[max_Lens*i], i);
            /* open write protect */
            Eeprom_WriteProtectEnable21305(1, i4SlaveAddr);
            msleep(6);
            return IMGSENSOR_RETURN_ERROR;
        }
        msleep(6);
    }
    ret = Eeprom_TableWrite((data_base+max_Lens*idx),
            &pData[max_Lens*idx], idy, i4SlaveAddr);
    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("Eeprom_TableWrite error: idx= %d idy= %d\n", idx, idy);
        /* open write protect */
        Eeprom_WriteProtectEnable21305(1, i4SlaveAddr);
        msleep(6);
        return IMGSENSOR_RETURN_ERROR;
    }
    msleep(6);
    /* open write protect */
    Eeprom_WriteProtectEnable21305(1, i4SlaveAddr);
    msleep(6);
    return ret;
}

static enum IMGSENSOR_RETURN Eeprom_WriteData_21305_Imx355(kal_uint16 data_base, kal_uint16 data_length,
                                   kal_uint8 *pData,  kal_uint16 i4SlaveAddr)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint16 max_Lens = WRITE_DATA_MAX_LENGTH;
    kal_uint32 idx, idy, i = 0;
    if (pData == NULL || data_length < WRITE_DATA_MAX_LENGTH) {
        pr_err("Eeprom_WriteData_21305_Imx355 pData is NULL or invalid DataLen:%d",data_length );
        return IMGSENSOR_RETURN_ERROR;
    }

    idx = data_length/max_Lens;
    idy = data_length%max_Lens;

    //close the eeprom write protection
    Eeprom_WriteProtectEnable21305IMX355(0, i4SlaveAddr);

    //Write OPT data.
    msleep(6);
    for (i = 0; i < idx; i++ ) {
        ret = Eeprom_TableWrite((data_base+max_Lens*i),
                &pData[max_Lens*i], max_Lens, i4SlaveAddr);
        if (ret != IMGSENSOR_RETURN_SUCCESS) {
            pr_err("write_eeprom data:0x%x, error: i= %d\n", pData[max_Lens*i], i);
            //open write protect
            Eeprom_WriteProtectEnable21305IMX355(1, i4SlaveAddr);
            msleep(6);
            return IMGSENSOR_RETURN_ERROR;
        }
        msleep(6);
    }
    ret = Eeprom_TableWrite((data_base+max_Lens*idx),
            &pData[max_Lens*idx], idy, i4SlaveAddr);
    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("Eeprom_TableWrite error: idx= %d idy= %d\n", idx, idy);
        //open write protect
        Eeprom_WriteProtectEnable21305IMX355(1, i4SlaveAddr);
        msleep(6);
        return IMGSENSOR_RETURN_ERROR;
    }
    msleep(6);
    //open write protect
    Eeprom_WriteProtectEnable21305IMX355(1, i4SlaveAddr);
    msleep(6);
    return ret;
}

enum IMGSENSOR_RETURN Eeprom_SensorInfoValid(
            enum IMGSENSOR_SENSOR_IDX sensor_idx,
            kal_uint32 sensorID)
{
    struct CAMERA_DEVICE_INFO *pCamDeviceObj = &gImgEepromInfo;
#ifdef SENSOR_PLATFORM_5G_H
    /*2021/12/28, Bringup camera for 20817.*/
    if (is_project(20817) || is_project(20827) || is_project(20831)){
        pCamDeviceObj = &gImgEepromInfo_20817;
    }

    if (is_project(21881) || is_project(21882)){
        pCamDeviceObj = &gImgEepromInfo_21881;
    }
#endif

    if (sensor_idx > pCamDeviceObj->i4SensorNum - 1) {
        pr_info("[%s] sensor_idx:%d > i4SensorNum: %d", __func__, sensor_idx, pCamDeviceObj->i4SensorNum);
        return IMGSENSOR_RETURN_ERROR;
    }

    if ((is_project(0x2260B) || is_project(22669) || is_project(0x2266A) || is_project(22609))
        && (sensorID == pCamDeviceObj->pCamModuleInfo[sensor_idx + 1].i4Sensorid)) {
        return IMGSENSOR_RETURN_SUCCESS;
    }

    if (sensorID != pCamDeviceObj->pCamModuleInfo[sensor_idx].i4Sensorid) {
        pr_info("[%s] sensorID:%d mismatch i4Sensorid: %d",
                __func__, sensorID, pCamDeviceObj->pCamModuleInfo[sensor_idx].i4Sensorid);
        return IMGSENSOR_RETURN_ERROR;
    }

    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Eeprom_CallWriteService(ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereoData)
{
    enum IMGSENSOR_RETURN  ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint16 data_base = 0, data_length = 0;
    kal_uint8 *pData;
    kal_uint16 i4SlaveAddr = 0xFF;
    kal_uint32 uSensorId = 0x0, uDeviceId = DUAL_CAMERA_MAIN_SENSOR;
    enum IMGSENSOR_SENSOR_IDX sensor_idx = IMGSENSOR_SENSOR_IDX_MAIN;
    struct CAMERA_DEVICE_INFO *pCamDeviceObj = &gImgEepromInfo;
#ifdef SENSOR_PLATFORM_5G_H
    /*2021/12/28, Bringup camera for 20817.*/
    if (is_project(20817) || is_project(20827) || is_project(20831)){
        pCamDeviceObj = &gImgEepromInfo_20817;
    }

    if (is_project(21881) || is_project(21882)){
        pCamDeviceObj = &gImgEepromInfo_21881;
    }
#endif

    if (pStereoData == NULL) {
        pr_err("Eeprom_CallWriteService pStereoData is NULL");
        return IMGSENSOR_RETURN_ERROR;
    }
    pr_debug("SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
                pStereoData->uSensorId,
                pStereoData->uDeviceId,
                pStereoData->baseAddr,
                pStereoData->dataLength);

    data_base   = pStereoData->baseAddr;
    data_length = pStereoData->dataLength;
    uSensorId   = pStereoData->uSensorId;
    uDeviceId   = pStereoData->uDeviceId;
    pData       = pStereoData->uData;

    /*match iic slave addr*/
    if (uDeviceId == DUAL_CAMERA_MAIN_SENSOR) {
        sensor_idx = IMGSENSOR_SENSOR_IDX_MAIN;
    } else if (uDeviceId == DUAL_CAMERA_SUB_SENSOR) {
        sensor_idx = IMGSENSOR_SENSOR_IDX_SUB;
    } else if (uDeviceId == DUAL_CAMERA_MAIN_2_SENSOR) {
        sensor_idx = IMGSENSOR_SENSOR_IDX_MAIN2;
    } else if (uDeviceId == DUAL_CAMERA_SUB_2_SENSOR) {
        sensor_idx = IMGSENSOR_SENSOR_IDX_SUB2;
    } else if (uDeviceId == DUAL_CAMERA_MAIN_3_SENSOR) {
        sensor_idx = IMGSENSOR_SENSOR_IDX_MAIN3;
        if (is_project(22277) && uSensorId == IMX709_SENSOR_ID22277) {
            sensor_idx = IMGSENSOR_SENSOR_IDX_SUB2;
        }else if (is_project(23231) && uSensorId == IMX709_SENSOR_ID23231){
            sensor_idx = IMGSENSOR_SENSOR_IDX_SUB2;
        }
    } else if (uDeviceId == DUAL_CAMERA_MAIN_4_SENSOR) {
        sensor_idx = IMGSENSOR_SENSOR_IDX_MAIN4;
    }

    /*eeprom info check*/
    ret = Eeprom_WriteInfomatch(pStereoData);
    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("Eeprom_WriteInfomatch failed: %d", ret);
        return IMGSENSOR_RETURN_ERROR;
    }

    /*Check sensorid & senidx*/
    if (Eeprom_SensorInfoValid(sensor_idx, uSensorId) != IMGSENSOR_RETURN_SUCCESS) {
        pr_info("[%s]sensor_idx:%d, uSensorId:0x%x Invalid", __func__, sensor_idx, uSensorId);
        return IMGSENSOR_RETURN_ERROR;
    }
    /*Match EEPROM IIC-ADDR EEPROM OFFSET*/
    i4SlaveAddr = pCamDeviceObj->pCamModuleInfo[sensor_idx].i4SlaveAddr;
    if (i4SlaveAddr == 0xFF || i4SlaveAddr == 0) {
        pr_info("[%s] uSensorId:%d is not support Eeprom Hardware", __func__, uSensorId);
        return IMGSENSOR_RETURN_ERROR;
    }

    if (is_project(20817) || is_project(20827) || is_project(20831)
        || is_project(21881) || is_project(21882)){
        // 0xB0为imx766 eeprom写保护的slave addr
    	if ( uSensorId == IMX766_SENSOR_ID_20817) {
    		ret = Eeprom_WriteData_20817(data_base, data_length, pData, i4SlaveAddr, 0xB0);
    	}else if (uSensorId == IMX766_SENSOR_ID_21881) {
    		ret = Eeprom_WriteData_21881(data_base, data_length, pData, i4SlaveAddr, 0xB0);
    	}else {
    		pr_err("Other projects.");
    	}
    }else if((is_project(21015) || is_project(21217) || is_project(21016) || is_project(21218)) && uSensorId == (IMX766_SENSOR_ID_21015-SENSOR_ID_OFFSET_21015)){
        ret = Eeprom_WriteData_21015(data_base, data_length, pData, i4SlaveAddr);
    }else if(is_project(21305) && uSensorId == IMX766_SENSOR_ID_21305){
        ret = Eeprom_WriteData_21305(data_base, data_length, pData, i4SlaveAddr);
    }else if(is_project(21305) && uSensorId == IMX355_SENSOR_ID_21305){
        ret = Eeprom_WriteData_21305_Imx355(data_base, data_length, pData, i4SlaveAddr);
    } else if (is_project(22277) && uSensorId == IMX709_SENSOR_ID22277) {
        ret = Eeprom_WriteData_22277_IMX709(data_base, data_length, pData, i4SlaveAddr);
    } else if (is_project(22277) && uSensorId == IMX355_SENSOR_ID22277) {
        ret = Eeprom_WriteData_22277_IMX355(data_base, data_length, pData, i4SlaveAddr);
    } else if (is_project(22277) && uSensorId == OV64B_SENSOR_ID22277) {
        ret = Eeprom_WriteData_22277_OV64B(data_base, data_length, pData, i4SlaveAddr);
    } else if (is_project(23231) && uSensorId == IMX709_SENSOR_ID23231) {
        ret = Eeprom_WriteData_23231_IMX709(data_base, data_length, pData, i4SlaveAddr);
    } else if (is_project(23231) && uSensorId == IMX355_SENSOR_ID23231) {
        ret = Eeprom_WriteData_23231_IMX355(data_base, data_length, pData, i4SlaveAddr);
    } else if (is_project(23231) && uSensorId == IMX882_SENSOR_ID23231) {
        ret = Eeprom_WriteData_23231_IMX882(data_base, data_length, pData, i4SlaveAddr);
    } else {
        ret = Eeprom_WriteData(data_base, data_length, pData, i4SlaveAddr);
    }

    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("Eeprom_WriteData failed: %d", ret);
        return IMGSENSOR_RETURN_ERROR;
    }
    pr_debug("Eeprom_CallWriteService return SCUESS:%d\n", ret);
    return ret;
}

void Eeprom_CamInfoDataRead(enum IMGSENSOR_SENSOR_IDX sensor_idx, kal_uint16 slaveAddr)
{
    kal_uint8 module_id = 0xFF;
    kal_uint16 dataLens = 2, dataAddr = 0xFFFF, i = 0;
    struct CAMERA_DEVICE_INFO *pCamDeviceObj = &gImgEepromInfo;
#ifdef SENSOR_PLATFORM_5G_H
    /*2021/12/28, Bringup camera for 20817.*/
    if (is_project(20817) || is_project(20827) || is_project(20831)){
        pCamDeviceObj = &gImgEepromInfo_20817;
    }

    if (is_project(21881) || is_project(21882)){
        pCamDeviceObj = &gImgEepromInfo_21881;
    }
#endif

    /*Read Module Info Low-2bytes*/
    dataAddr = pCamDeviceObj->pCamModuleInfo[sensor_idx].i4CamInfoAddr[0];
    module_id = (kal_uint8)Eeprom_1ByteDataRead(dataAddr, slaveAddr);
    pCamDeviceObj->camNormdata[sensor_idx][0] = module_id;
    pCamDeviceObj->camNormdata[sensor_idx][1] =
                (kal_uint8)Eeprom_1ByteDataRead(dataAddr+1, slaveAddr);
    /*Register Module Info*/
    Oplusimgsensor_Registdeviceinfo(pCamDeviceObj->pCamModuleInfo[sensor_idx].name,
                                    pCamDeviceObj->pCamModuleInfo[sensor_idx].version,
                                    module_id);
    /*Read Module Info High-6bytes*/
    dataAddr = pCamDeviceObj->pCamModuleInfo[sensor_idx].i4CamInfoAddr[1];
    dataLens = 6;
    for (i = 0; i < 6; i ++) {
        pCamDeviceObj->camNormdata[sensor_idx][2+i] =
            (kal_uint8)Eeprom_1ByteDataRead(dataAddr+i, slaveAddr);
    }
}

void Eeprom_CamSNDataRead(enum IMGSENSOR_SENSOR_IDX sensor_idx, kal_uint16 slaveAddr)
{
    kal_uint16 dataAddr = 0xFFFF, i = 0;
    kal_uint16 snLength = OPLUS_CAMERASN_LENS;
    struct CAMERA_DEVICE_INFO *pCamDeviceObj = &gImgEepromInfo;
#ifdef SENSOR_PLATFORM_5G_H
    /*2021/12/28, Bringup camera for 20817.*/
    if (is_project(20817) || is_project(20827) || is_project(20831)){
        pCamDeviceObj = &gImgEepromInfo_20817;
    }

    if (is_project(21881) || is_project(21882)){
        pCamDeviceObj = &gImgEepromInfo_21881;
    }
#endif
    if (is_project(22610) || is_project(23231)
        || is_project(23613) || is_project(23686) || is_project(23687) || is_project(23689)
        || is_project(23707) || is_project(23709)){
        /*Read camera SN-23bytes*/
        snLength = OPLUS_CAMERASN_LENS_23;
    }
    /*Read camera SN-20bytes*/
    dataAddr = pCamDeviceObj->pCamModuleInfo[sensor_idx].i4CamSNAddr;
    for (i = 0; i < snLength; i++) {
        pCamDeviceObj->camNormdata[sensor_idx][OPLUS_CAMMODULEINFO_LENS+i] =
            (kal_uint8)Eeprom_1ByteDataRead(dataAddr+i, slaveAddr);
    }
}

void Eeprom_CamAFCodeDataRead(enum IMGSENSOR_SENSOR_IDX sensor_idx, kal_uint16 slaveAddr)
{
    kal_uint16 dataAddr = 0xFFFF, dataCnt = 0;
    kal_uint16 i = 0;
    kal_uint16 snLength = OPLUS_CAMERASN_LENS;
    struct CAMERA_DEVICE_INFO *pCamDeviceObj = &gImgEepromInfo;
#ifdef SENSOR_PLATFORM_5G_H
    /*2021/12/28, Bringup camera for 20817.*/
    if (is_project(20817) || is_project(20827) || is_project(20831)){
        pCamDeviceObj = &gImgEepromInfo_20817;
    }

    if (is_project(21881) || is_project(21882)){
        pCamDeviceObj = &gImgEepromInfo_21881;
    }
#endif
    if (is_project(22610) || is_project(23231)
        || is_project(23613) || is_project(23686) || is_project(23687) || is_project(23689)
        || is_project(23707) || is_project(23709)){
        /*Read camera SN-23bytes*/
        snLength = OPLUS_CAMERASN_LENS_23;
    }
    /*Read AF MAC+50+100+Inf+StereoDac*/
    if (pCamDeviceObj->pCamModuleInfo[sensor_idx].i4AfSupport) {
        dataCnt = OPLUS_CAMMODULEINFO_LENS + snLength;
        for (i = 0; i < 4; i ++) {
            dataAddr = pCamDeviceObj->pCamModuleInfo[sensor_idx].i4AFCodeAddr[i];
            pCamDeviceObj->camNormdata[sensor_idx][dataCnt] =
                (kal_uint8)Eeprom_1ByteDataRead(dataAddr, slaveAddr);
            pCamDeviceObj->camNormdata[sensor_idx][dataCnt+1] =
                (kal_uint8)Eeprom_1ByteDataRead(dataAddr+1, slaveAddr);
            dataCnt += 2;
        }
    }

    /*Set stereoFlag*/
    if ((pCamDeviceObj->i4MWDataIdx == sensor_idx)
         || (pCamDeviceObj->i4MTDataIdx == sensor_idx)
         || (pCamDeviceObj->i4FrontDataIdx == sensor_idx)) {
        dataCnt = OPLUS_CAMCOMDATA_LENS - 1;
        pCamDeviceObj->camNormdata[sensor_idx][dataCnt] = (kal_uint8)(sensor_idx);
    }
}

void Eeprom_StereoDataRead(enum IMGSENSOR_SENSOR_IDX sensor_idx, kal_uint16 slaveAddr)
{
    kal_uint16 dataLens = CALI_DATA_MASTER_LENGTH, dataAddr = 0xFFFF, dataCnt = 0;
    kal_uint16 i = 0;
    struct CAMERA_DEVICE_INFO *pCamDeviceObj = &gImgEepromInfo;
#ifdef SENSOR_PLATFORM_5G_H
    /*2021/12/28, Bringup camera for 20817.*/
    if (is_project(20817) || is_project(20827) || is_project(20831)){
        pCamDeviceObj = &gImgEepromInfo_20817;
    }

    if (is_project(21881) || is_project(21882)){
        pCamDeviceObj = &gImgEepromInfo_21881;
    }
#endif

    /*Read Single StereoParamsData and joint stereoData*/
    if (pCamDeviceObj->i4MWDataIdx == IMGSENSOR_SENSOR_IDX_MAIN2) {
        if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN) {
            if (is_project(20615) || is_project(21609) ||
                is_project(20662) || is_project(20619)){
                dataLens = CALI_DATA_MASTER_LENGTH_20615;
            } else if (is_project(20730) || is_project(20731) || is_project(20732)){
                dataLens = CALI_DATA_MASTER_LENGTH_20730;
            } else {
                dataLens = CALI_DATA_MASTER_LENGTH;
            }
            dataAddr = pCamDeviceObj->i4MWStereoAddr[0];
        } else if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2) {
            dataLens = CALI_DATA_SLAVE_LENGTH;
            dataAddr = pCamDeviceObj->i4MWStereoAddr[1];
            if (is_project(20615) || is_project(21609) ||
                is_project(20662) || is_project(20619)){
                dataCnt = CALI_DATA_MASTER_LENGTH_20615;
            } else if (is_project(20730) || is_project(20731) || is_project(20732)){
                dataCnt = CALI_DATA_MASTER_LENGTH_20730;
                dataLens = CALI_DATA_SLAVE_LENGTH_20730;
            } else {
                dataCnt = CALI_DATA_MASTER_LENGTH;
            }
        }
        if(sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN || sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2) {
            for (i = 0; i < dataLens; i++) {
                pCamDeviceObj->stereoMWdata[dataCnt+i] =
                            (kal_uint8)Eeprom_1ByteDataRead(dataAddr+i, slaveAddr);
            }
        }
    } else if (pCamDeviceObj->i4MWDataIdx == IMGSENSOR_SENSOR_IDX_SUB2) {
        if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN) {
            dataLens = CALI_DATA_MASTER_LENGTH;
            dataAddr = pCamDeviceObj->i4MTStereoAddr[0];
        } else if (sensor_idx == IMGSENSOR_SENSOR_IDX_SUB2) {
            dataLens = CALI_DATA_SLAVE_LENGTH;
            dataAddr = pCamDeviceObj->i4MTStereoAddr[1];
            dataCnt = CALI_DATA_MASTER_LENGTH;
        }
        for (i = 0; i < dataLens; i++) {
            pCamDeviceObj->stereoMTdata[dataCnt+i] =
                        (kal_uint8)Eeprom_1ByteDataRead(dataAddr+i, slaveAddr);
        }
    } else if (pCamDeviceObj->i4MWDataIdx == IMGSENSOR_SENSOR_IDX_MAIN4) {
        if (sensor_idx == IMGSENSOR_SENSOR_IDX_SUB) {
            dataLens = CALI_DATA_MASTER_LENGTH;
            dataAddr = pCamDeviceObj->i4FrontStereoAddr[0];
        } else if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN4) {
            dataLens = CALI_DATA_SLAVE_LENGTH;
            dataAddr = pCamDeviceObj->i4FrontStereoAddr[1];
            dataCnt = CALI_DATA_MASTER_LENGTH;
        }
        for (i = 0; i < dataLens; i++) {
            pCamDeviceObj->stereoFrontdata[dataCnt+i] =
                        (kal_uint8)Eeprom_1ByteDataRead(dataAddr+i, slaveAddr);
        }
    }

    if (pCamDeviceObj->i4MTDataIdx == IMGSENSOR_SENSOR_IDX_SUB2) {
        if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN) {
            dataLens = CALI_DATA_MASTER_LENGTH;
            dataAddr = pCamDeviceObj->i4MTStereoAddr[0];
        } else if (sensor_idx == IMGSENSOR_SENSOR_IDX_SUB2) {
            if (is_project(22277)) {
                dataLens = CALI_DATA_SLAVE_TELE_LENGTH_22277;
            } else if (is_project(23231)){
                dataLens = CALI_DATA_SLAVE_TELE_LENGTH_23231;
            } else {
                dataLens = CALI_DATA_SLAVE_LENGTH;
            }
            dataAddr = pCamDeviceObj->i4MTStereoAddr[1];
            dataCnt = CALI_DATA_MASTER_LENGTH ;
        }
        if(sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN || sensor_idx == IMGSENSOR_SENSOR_IDX_SUB2) {
            for (i = 0; i < dataLens; i++) {
                pCamDeviceObj->stereoMTdata[dataCnt+i] =
                            (kal_uint8)Eeprom_1ByteDataRead(dataAddr+i, slaveAddr);
            }
        }
    }
}

void Eeprom_DistortionParamsRead(enum IMGSENSOR_SENSOR_IDX sensor_idx, kal_uint16 slaveAddr)
{
    kal_uint16 dataAddr = 0xFFFF, dataLens = CAMERA_DISTORTIONPARAMS_LENGTH;
    kal_uint16 i = 0;
    struct CAMERA_DEVICE_INFO *pCamDeviceObj = &gImgEepromInfo;
#ifdef SENSOR_PLATFORM_5G_H
    /*2021/12/28, Bringup camera for 20817.*/
    if (is_project(20817) || is_project(20827) || is_project(20831)){
        pCamDeviceObj = &gImgEepromInfo_20817;
    }

    if (is_project(21881) || is_project(21882)){
        pCamDeviceObj = &gImgEepromInfo_21881;
    }
#endif

    if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2 && pCamDeviceObj->i4DistortionAddr) {
        dataAddr = pCamDeviceObj->i4DistortionAddr;
        for (i = 0; i < dataLens; i++) {
            pCamDeviceObj->distortionParams[i] = (kal_uint8)Eeprom_1ByteDataRead(dataAddr+i, slaveAddr);
        }
    }
}

enum IMGSENSOR_RETURN Eeprom_DataInit(
            enum IMGSENSOR_SENSOR_IDX sensor_idx,
            kal_uint32 sensorID)
{

    kal_uint16 slaveAddr = 0xA0;
    kal_uint8 module_id = 0xFF;
    struct CAMERA_DEVICE_INFO *pCamDeviceObj = &gImgEepromInfo;
#ifdef SENSOR_PLATFORM_5G_H
    /*2021/12/28, Bringup camera for 20817.*/
    if (is_project(20817) || is_project(20827) || is_project(20831)){
        pCamDeviceObj = &gImgEepromInfo_20817;
    }

    if (is_project(21881) || is_project(21882)){
        pCamDeviceObj = &gImgEepromInfo_21881;
    }
#endif

    if (Eeprom_SensorInfoValid(sensor_idx, sensorID) != IMGSENSOR_RETURN_SUCCESS) {
        pr_info("[%s]sensor_idx:%d, sensorID:0x%x Invalid", __func__, sensor_idx, sensorID);
        return IMGSENSOR_RETURN_ERROR;
    }
    /*Match EEPROM IIC-ADDR EEPROM OFFSET*/
    slaveAddr = pCamDeviceObj->pCamModuleInfo[sensor_idx].i4SlaveAddr;
    if (slaveAddr == 0xFF) {
        pr_info("[%s] sensorID:%d is not support Eeprom Hardware", __func__, sensorID);
        Oplusimgsensor_Registdeviceinfo(pCamDeviceObj->pCamModuleInfo[sensor_idx].name,
                                        pCamDeviceObj->pCamModuleInfo[sensor_idx].version,
                                        module_id);
        return IMGSENSOR_RETURN_ERROR;
    }
    pCamDeviceObj->i4CurSensorIdx = sensor_idx;
    pCamDeviceObj->i4CurSensorId = sensorID;
    /*Read ModuleInfo Data*/
    Eeprom_CamInfoDataRead(sensor_idx, slaveAddr);
    /*Read camera sn*/
    Eeprom_CamSNDataRead(sensor_idx, slaveAddr);
    /*Read SteroData*/
    Eeprom_StereoDataRead(sensor_idx, slaveAddr);
    /*Read af code*/
    Eeprom_CamAFCodeDataRead(sensor_idx, slaveAddr);
    /*Read distortionParams*/
    Eeprom_DistortionParamsRead(sensor_idx, slaveAddr);

    return IMGSENSOR_RETURN_SUCCESS;
}

enum CUSTOM_CAMERA_ERROR_CODE_ENUM Eeprom_Control(
            enum IMGSENSOR_SENSOR_IDX sensor_idx,
            MSDK_SENSOR_FEATURE_ENUM feature_id,
            unsigned char *feature_para,
            kal_uint32 sensorID)
{
    kal_uint32 i4SensorID = 0;
    kal_uint32 dataCnt = 0;
    enum IMGSENSOR_SENSOR_IDX i4Sensor_idx = 0;
    struct CAMERA_DEVICE_INFO *pCamDeviceObj = &gImgEepromInfo;
#ifdef SENSOR_PLATFORM_5G_H
    /*2021/12/28, Bringup camera for 20817.*/
    if (is_project(20817) || is_project(20827) || is_project(20831)){
        pCamDeviceObj = &gImgEepromInfo_20817;
    }

    if (is_project(21881) || is_project(21882)){
        pCamDeviceObj = &gImgEepromInfo_21881;
    }
#endif

    i4Sensor_idx = pCamDeviceObj->i4CurSensorIdx;
    i4SensorID = pCamDeviceObj->i4CurSensorId;

    switch (feature_id) {
        case SENSOR_FEATURE_GET_EEPROM_COMDATA:
        {
            if (Eeprom_SensorInfoValid(i4Sensor_idx, i4SensorID) != IMGSENSOR_RETURN_SUCCESS) {
                pr_info("[%s]_COMDATA i4Sensor_idx:%d, i4SensorID:0x%x Invalid",
                                                __func__, i4Sensor_idx, i4SensorID);
                return ERROR_MSDK_IS_ACTIVATED;
            }
            dataCnt = (kal_uint32)(CAMERA_EEPPROM_COMDATA_LENGTH*i4Sensor_idx);
            pr_info("GET_EEPROM_COMDATA i4Sensor_idx %d, i4SensorID:0x%x, dataCnt:%d\n",
                        i4Sensor_idx, i4SensorID, dataCnt);
            memcpy(&feature_para[0], pCamDeviceObj->camNormdata[i4Sensor_idx], CAMERA_EEPPROM_COMDATA_LENGTH);
            break;
        }
        case SENSOR_FEATURE_GET_EEPROM_STEREODATA:
        {
            if (Eeprom_SensorInfoValid(i4Sensor_idx, i4SensorID) != IMGSENSOR_RETURN_SUCCESS) {
                pr_info("[%s]_STEREODATA i4Sensor_idx:%d, i4SensorID:0x%x Invalid",
                                                __func__, i4Sensor_idx, i4SensorID);
                return ERROR_MSDK_IS_ACTIVATED;
            }
            pr_info("GET_EEPROM_STEREODATA i4Sensor_idx %d, stereoDataIdx:%d %d %d\n",
                        i4Sensor_idx,
                        pCamDeviceObj->i4MWDataIdx,
                        pCamDeviceObj->i4MTDataIdx,
                        pCamDeviceObj->i4FrontDataIdx);
            if (i4Sensor_idx == pCamDeviceObj->i4MWDataIdx) {
                if (is_project(20171) || is_project(20353)
                    || is_project(20615) || is_project(20619)
                    || is_project(21015) || is_project(21217)
                    || is_project(21061) || is_project(21218)
                    || is_project(21609) || is_project(20662)
                    || is_project(21305)) {
                    memcpy(&feature_para[0], pCamDeviceObj->stereoMWdata, DUALCAM_CALI_DATA_LENGTH_TOTAL_QCOM);
                } else if (is_project(20817) || is_project(20827) || is_project(20831)
                           || is_project(21881) || is_project(21882)) {
                    memcpy(&feature_para[0], pCamDeviceObj->stereoMWdata, DUALCAM_CALI_DATA_LENGTH_TOTAL);
                } else {
                    memcpy(&feature_para[0], pCamDeviceObj->stereoMWdata, DUALCAM_CALI_DATA_LENGTH_TOTAL);
                }
            } else if (i4Sensor_idx == pCamDeviceObj->i4MTDataIdx) {
                memcpy(&feature_para[0], pCamDeviceObj->stereoMTdata, DUALCAM_CALI_DATA_LENGTH_TOTAL_TELE);
            } else if (i4Sensor_idx == pCamDeviceObj->i4FrontDataIdx) {
                memcpy(&feature_para[0], pCamDeviceObj->stereoFrontdata, DUALCAM_CALI_DATA_LENGTH_TOTAL);
            }
            break;
        }
        case SENSOR_FEATURE_SET_SENSOR_OTP:
        {
            enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
            pr_debug("SENSOR_FEATURE_SET_SENSOR_OTP\n");
            if (is_project(20391) || is_project(20392) || is_project(20075) || is_project(20076)
              || is_project(20151) || is_project(20301) || is_project(20302)
              || is_project(20131) || is_project(20133) || is_project(20255) || is_project(20257)
              || is_project(20001) || is_project(20002) || is_project(20003) || is_project(20200)
              || is_project(20015) || is_project(20016) || is_project(20108) || is_project(20109)
              || is_project(20307) || is_project(21037) || is_project(20013)
              || is_project(21101) || is_project(21102) || is_project(21235) || is_project(21236)
              || is_project(0x2163B) || is_project(0x2163C) || is_project(0x2163D)
              || is_project(21639) || is_project(0x216CD) || is_project(0x216CE)
              || is_project(21831) || is_project(19165) || is_project(20095) || is_project(20610)
              || is_project(20611) || is_project(20680) || is_project(20613) || is_project(20686)
              || is_project(20041) || is_project(20042) || is_project(20043) || is_project(22631)
              || is_project(22603) || is_project(22604) || is_project(22609) || is_project(22632)
              || is_project(0x2260A) || is_project(0x2260B) || is_project(22669) || is_project(23602)
              || is_project(0x2266A) || is_project(0x2266B) || is_project(0x2266C))
                return ERROR_NONE;
            ret = Eeprom_CallWriteService((ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(feature_para));
            if (ret == IMGSENSOR_RETURN_SUCCESS)
                return ERROR_NONE;
            else
                return ERROR_MSDK_IS_ACTIVATED;
        }
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            pr_debug("SENSOR_FEATURE_CHECK_SENSOR_ID: %d 0x%x\n", sensor_idx, sensorID);
            Eeprom_DataInit(sensor_idx, sensorID);
        break;
        case SENSOR_FEATURE_GET_DISTORTIONPARAMS:
            pr_debug("SENSOR_FEATURE_GET_DISTORTIONPARAMS: %d 0x%x\n", sensor_idx, sensorID);
            if (pCamDeviceObj->i4DistortionAddr == 0) {
                pr_debug("Invalid distortionParams");
                return ERROR_MSDK_IS_ACTIVATED;
            } else {
                memcpy(&feature_para[0], pCamDeviceObj->distortionParams, CAMERA_DISTORTIONPARAMS_LENGTH);
            }
        break;
        default:
            pr_err("Invalid feature_id: %d", feature_id);
        break;
    }

    return ERROR_NONE;
}
