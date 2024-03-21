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

#include <linux/delay.h>
#include <linux/string.h>

#include "kd_imgsensor.h"
#include "imgsensor_eeprom_v1.h"

#define DUMP_EEPROM 0

/*GC8054: 0xA2*/
/*static IMGSENSOR_EEPROM_MODULE_INFO gEpInfoObj = {
    .i4SlaveAddr = {0xA0, 0xA8, 0xA2, 0xA8},
};*/

kal_uint16 read_cmos_eeprom_8(kal_uint16 addr, kal_uint16 slaveaddr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 1, slaveaddr);
    return get_byte;
}

void read_eepromData(kal_uint8 *uData, kal_uint16 dataAddr,
                            kal_uint16 dataLens, kal_uint16 slaveAddr)
{
    int dataCnt = 0;
    for (dataCnt = 0; dataCnt < dataLens; dataCnt++) {
        uData[dataCnt] = (u8)read_cmos_eeprom_8(dataAddr+dataCnt, slaveAddr);
    }
}

static kal_int32 table_write_eeprom(kal_uint16 addr, kal_uint8 *para,
                                    kal_uint32 len, kal_uint16 i4SlaveAddr)
{
    kal_int32 ret = IMGSENSOR_RETURN_SUCCESS;
    char pusendcmd[WRITE_DATA_MAX_LENGTH+2];
    pusendcmd[0] = (char)(addr >> 8);
    pusendcmd[1] = (char)(addr & 0xFF);

    memcpy(&pusendcmd[2], para, len);

    ret = iBurstWriteReg((kal_uint8 *)pusendcmd , (len + 2), i4SlaveAddr);

    return ret;
}

static kal_int32 write_eeprom_protect(kal_uint16 enable, kal_uint16 i4SlaveAddr)
{
    kal_int32 ret = IMGSENSOR_RETURN_SUCCESS;
    char pusendcmd[3];
    pusendcmd[0] = 0x80;
    pusendcmd[1] = 0x00;
    if (enable)
        pusendcmd[2] = 0xE0;
    else
        pusendcmd[2] = 0x00;

    ret = iBurstWriteReg((kal_uint8 *)pusendcmd , 3, i4SlaveAddr);

    return ret;
}

static enum IMGSENSOR_RETURN eeprom_EngInfomatch(
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

    switch (uSensorId) {
        #if defined(GM1ST_MIPI_RAW)
        case GM1ST_SENSOR_ID:
        {
            if (data_length == DUALCAM_CALI_DATA_LENGTH
                && data_base == GM1ST_STEREO_START_ADDR) {
                pr_debug("gm1st DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("gm1st Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(IMX471_MIPI_RAW)
        case IMX471_SENSOR_ID:
        {
            if (data_length == DUALCAM_CALI_DATA_LENGTH
                && data_base == IMX471_STEREO_START_ADDR) {
                pr_debug("imx471 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("imx471 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(HI846_MIPI_RAW)
        case HI846_SENSOR_ID:
        {
            if (data_length == DUALCAM_CALI_DATA_LENGTH
                && data_base == HI846_STEREO_START_ADDR_19537) {
                pr_debug("hi846 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("hi846 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(GC02M0B_MIPI_MONO0)
        case GC02M0_SENSOR_ID0:
        {
            if (data_length == DUALCAM_CALI_DATA_LENGTH
                && data_base == GC02M0B_STEREO_START_ADDR) {
                pr_debug("gc02m0b DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("gc02m0b Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(GC02M1B_MIPI_MONO0)  //check here
        case GC02M1_SENSOR_ID0:
        {
            if (data_length == DUALCAM_CALI_DATA_LENGTH
                && data_base == GC02M1B_STEREO_START_ADDR) {
                pr_debug("gc02m1b DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("gc02m1b Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(S5KGW1_MIPI_RAW)  //check here
        case S5KGW1_SENSOR_ID:
        {
            if (data_length == AESYNC_DATA_LENGTH_MASTER
                && data_base == S5KGW1_AESYNC_START_ADDR) {
                pr_debug("s5kgw1 Aesync DA:0x%x DL:%d", data_base, data_length);
            } else if (data_length == DUALCAM_CALI_DATA_LENGTH
                && data_base == S5KGW1_STEREO_START_ADDR_WIDE) {
                pr_debug("s5kgw1 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("s5kgw1 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(S5KGH1_MIPI_RAW)
        case S5KGH1_SENSOR_ID:
        {
            if (data_length == DUALCAM_CALI_DATA_LENGTH
                && data_base == S5KGH1_STEREO_START_ADDR) {
                pr_debug("s5kgh1 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("s5kgh1 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(GC5035_MIPI_RAW)
        case GC5035_SENSOR_ID:
        {
            if (data_length == CALI_DATA_SLAVE_LENGTH
                && data_base == GC5035_STEREO_START_ADDR) {
                pr_debug("GC5035 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("GC5035 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(GC8054_MIPI_RAW)
        case GC8054_SENSOR_ID:
        {
            if (data_length == AESYNC_DATA_LENGTH_WIDE
                && data_base == GC8054_AESYNC_START_ADDR) {
                pr_debug("GC8054 Aesync DA:0x%x DL:%d", data_base, data_length);
            }else if (data_length == DUALCAM_CALI_DATA_LENGTH
                && data_base == GC8054_STEREO_START_ADDR) {
                pr_debug("GC8054 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("GC8054 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(S5K3M5_MIPI_RAW)
        case S5K3M5_SENSOR_ID:
        {
            if (data_length == AESYNC_DATA_LENGTH_TELE
                && data_base == S5K3M5_AESYNC_START_ADDR) {
                pr_debug("S5K3M5 Aesync DA:0x%x DL:%d", data_base, data_length);
            }else if (data_length == DUALCAM_CALI_DATA_LENGTH
                && data_base == S5K3M5_STEREO_START_ADDR) {
                pr_debug("S5K3M5 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("S5K3M5 Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(GC02M0F_MIPI_MONO)
        case GC02M0F_SENSOR_ID:
        {
            if (data_length == CALI_DATA_SLAVE_LENGTH
                && data_base == GC02M0F_STEREO_START_ADDR) {
                pr_debug("GC02M0F DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("GC02M0F Invalid DA:0x%x DL:%d", data_base, data_length);
                return IMGSENSOR_RETURN_ERROR;
            }
            break;
        }
        #endif
        #if defined(S5KGW1_MIPI_RAW)  //check here
        case IMX586_SENSOR_ID:
        {
            if (data_length ==  DUALCAM_CALI_DATA_LENGTH
                && data_base == IMX586_STEREO_START_ADDR_WIDE) {
                pr_debug("imx586 DA:0x%x DL:%d", data_base, data_length);
            } else {
                pr_debug("IMX586 Invalid DA:0x%x DL:%d", data_base, data_length);
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

static kal_int32 write_Module_data(kal_uint16 data_base, kal_uint16 data_length,
                                   kal_uint8 *pData,  kal_uint16 i4SlaveAddr)
{
    kal_int32  ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint16 max_Lens = WRITE_DATA_MAX_LENGTH;
    kal_uint32 idx, idy;
    UINT32 i = 0;
    if (pData == NULL || data_length < WRITE_DATA_MAX_LENGTH) {
        pr_err("write_Module_data pData is NULL or invalid DL:%d",data_length );
        return IMGSENSOR_RETURN_ERROR;
    }

    idx = data_length/max_Lens;
    idy = data_length%max_Lens;
    pr_debug("write_Module_data: 0x%x %d i4SlaveAddr:0x%x\n",
                data_base,
                data_length,
                i4SlaveAddr);
    /* close write protect */
    write_eeprom_protect(0, i4SlaveAddr);
    msleep(6);
    for (i = 0; i < idx; i++ ) {
        ret = table_write_eeprom((data_base+max_Lens*i),
                &pData[max_Lens*i], max_Lens, i4SlaveAddr);
        if (ret != IMGSENSOR_RETURN_SUCCESS) {
            pr_err("write_eeprom error: i= %d\n", i);
            /* open write protect */
            write_eeprom_protect(1, i4SlaveAddr);
            msleep(6);
            return IMGSENSOR_RETURN_ERROR;
        }
        msleep(6);
    }
    ret = table_write_eeprom((data_base+max_Lens*idx),
            &pData[max_Lens*idx], idy, i4SlaveAddr);
    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("write_eeprom error: idx= %d idy= %d\n", idx, idy);
        /* open write protect */
        write_eeprom_protect(1, i4SlaveAddr);
        msleep(6);
        return IMGSENSOR_RETURN_ERROR;
    }
    msleep(6);
    /* open write protect */
    write_eeprom_protect(1, i4SlaveAddr);
    msleep(6);

    return ret;
}

kal_int32 call_eepromwrite_Service(ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereoData, kal_uint16 i4SlaveAddr)
{
    kal_int32  ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint16 data_base = 0, data_length = 0;
    kal_uint8 *pData;
    kal_uint32 uSensorId = 0xA0, uDeviceId = DUAL_CAMERA_MAIN_SENSOR;
	#if DUMP_EEPROM
	kal_uint8 uData[CALI_DATA_MASTER_LENGTH];
	int i = 0;
	#endif
    if (pStereoData == NULL) {
        pr_err("call_eepromwrite_Service pStereoData is NULL");
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
    /*if (uDeviceId == DUAL_CAMERA_MAIN_SENSOR) {
        i4SlaveAddr = pEpObj->i4SlaveAddr[IMGSENSOR_SENSOR_IDX_MAIN];
    } else if (uDeviceId == DUAL_CAMERA_SUB_SENSOR) {
        i4SlaveAddr = pEpObj->i4SlaveAddr[IMGSENSOR_SENSOR_IDX_SUB];
    } else if (uDeviceId == DUAL_CAMERA_MAIN_2_SENSOR) {
        i4SlaveAddr = pEpObj->i4SlaveAddr[IMGSENSOR_SENSOR_IDX_MAIN2];
    } else if (uDeviceId == DUAL_CAMERA_SUB_2_SENSOR) {
        i4SlaveAddr = pEpObj->i4SlaveAddr[IMGSENSOR_SENSOR_IDX_SUB2];
    }*/
    /*eeprom info check*/
    ret = eeprom_EngInfomatch(pStereoData);
    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("eeprom_EngInfomatch failed: %d", ret);
        return IMGSENSOR_RETURN_ERROR;
    }

    /*write eeprom action*/
    /*if (uSensorId == S5K3P9SP_SENSOR_ID) {*/
        /*s5k3p9sp special write*/
/*        data_length = 1280;
        ret = write_Module_data(data_base, data_length, pData, i4SlaveAddr);
        if (ret != IMGSENSOR_RETURN_SUCCESS) {
            pr_err("write_Module_data step1 failed: %d", ret);
            return IMGSENSOR_RETURN_ERROR;
        }
        data_base = 0x1D00;
        data_length = 277;
        ret = write_Module_data(data_base, data_length, &pData[1280], i4SlaveAddr);
        if (ret != IMGSENSOR_RETURN_SUCCESS) {
            pr_err("write_Module_data step2 failed: %d", ret);
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        ret = write_Module_data(data_base, data_length, pData, i4SlaveAddr);
        if (ret != IMGSENSOR_RETURN_SUCCESS) {
            pr_err("write_Module_data failed: %d", ret);
            return IMGSENSOR_RETURN_ERROR;
        }
    }*/

    ret = write_Module_data(data_base, data_length, pData, i4SlaveAddr);
    if (ret != IMGSENSOR_RETURN_SUCCESS) {
        pr_err("write_Module_data failed: %d", ret);
        return IMGSENSOR_RETURN_ERROR;
    }

    pr_debug("call_eepromwrite_Service return SCUESS:%d\n", ret);
	#if DUMP_EEPROM
		read_eepromData(uData, data_base, data_length, i4SlaveAddr);
		ret = strncmp(uData, pData, data_length);
		pr_info("cmp uData pData, ret = %d", ret);
		if (ret) {
			for (i - 0; i < data_length; ++i) {
				pr_debug("pData addr: 0x%x, data: 0x%x", i, pData[i]);
				pr_debug("uData addr: 0x%x, data: 0x%x", i, uData[i]);
			}
		}
	#endif
    return ret;
}
