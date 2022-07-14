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

#define PFX "IMX709_pdafotp"
#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/slab.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "imx709mipiraw_Sensor.h"
#include "imx709_eeprom.h"

static struct EEPROM_PDAF_INFO eeprom_pdaf_info[] = {
    {
        .LRC_addr = OTP_LRC_OFFSET,
        .LRC_size = LRC_SIZE,
    },
};

static DEFINE_MUTEX(gimx709_eeprom_mutex);

static bool selective_read_eeprom(kal_uint16 addr, BYTE *data)
{
    char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

    if (addr > IMX709_MAX_OFFSET)
        return false;

    if (iReadRegI2C(pu_send_cmd, 2, (u8 *) data,
            1, IMX709_EEPROM_SLAVE_ADDRESS) < 0) {
        return false;
    }
    return true;
}

static bool read_imx709_eeprom(kal_uint16 addr, BYTE *data, int size)
{
    int i = 0;
    int offset = addr;

    /*LOG_INF("enter read_eeprom size = %d\n", size);*/
    for (i = 0; i < size; i++) {
        if (!selective_read_eeprom(offset, &data[i]))
            return false;
        /*LOG_INF("read_eeprom 0x%0x %d\n", offset, data[i]);*/
        offset++;
    }
    return true;
}


unsigned int read_imx709_21127_LRC(kal_uint16 *data)
{
    kal_uint16 idx = 0, sensor_startL_reg = 0xCE00, sensor_startR_reg = 0xCF00;
    static BYTE imx709_LRC_data[LRC_SIZE] = { 0 };
    static unsigned int readed_size = 0;
    struct EEPROM_PDAF_INFO *pinfo = (struct EEPROM_PDAF_INFO *)&eeprom_pdaf_info[0];
    kal_uint8 lrc_flag = 0;

    LOG_INF("read imx709 LRC, otp_offset = %d, size = %u\n",
        pinfo->LRC_addr, pinfo->LRC_size);

    mutex_lock(&gimx709_eeprom_mutex);
    if ((readed_size == 0) &&
        read_imx709_eeprom(pinfo->LRC_addr,
            imx709_LRC_data, pinfo->LRC_size)) {
        readed_size = pinfo->LRC_size;
    }
    mutex_unlock(&gimx709_eeprom_mutex);

    for (idx = 0; idx < LRC_SIZE; idx++) {
        if(idx < LRC_SIZE/2){
            //LRC_Left
            data[2 * idx] = sensor_startL_reg++;
        }else{
            //LRC_Right
            data[2 * idx] = sensor_startR_reg++;
        }
        data[2 * idx + 1] = imx709_LRC_data[idx];
    }

    for (idx = 0; idx < LRC_SIZE; idx++) {
        if(idx < LRC_SIZE/2){
            //LRC_Left
            pr_debug("In %s: LRC_Left value[0x%x]:0x%x", __func__, data[2 * idx], data[2 * idx + 1]);
        }else{
            //LRC_RIGHT
            pr_debug("In %s: LRC_Right value[0x%x]:0x%x", __func__, data[2 * idx], data[2 * idx + 1]);
        }
    }
    read_imx709_eeprom(0x162E, &lrc_flag, 1);
    pr_info("LRC flag0x%x[1:valid, other:Invalid]", lrc_flag);
    return readed_size;
}

unsigned int read_imx709_21127_QSC(kal_uint16 * data)
{
    kal_uint16 idx = 0, sensor_qsc_address = 0x1000;
    kal_uint8 tmp_QSC_setting[QSC_SIZE];
    kal_uint8 qsc_ver = 0;

    read_imx709_eeprom(OTP_QSC_OFFSET, tmp_QSC_setting, QSC_SIZE);
    for (idx = 0; idx < QSC_SIZE; idx++) {
        data[2 * idx] = sensor_qsc_address;
        data[2 * idx + 1] = tmp_QSC_setting[idx];
        sensor_qsc_address += 1;
    }

    for (idx = 0; idx < QSC_SIZE; idx++) {
        pr_debug("alex qsc data imx709_QSC_setting[0x%x] = 0x%x",
            data[2 * idx], data[2 * idx + 1]);
    }

    read_imx709_eeprom(0x2A32, &qsc_ver, 1);
    pr_info("QSC Version: 0x%x", qsc_ver);
    return 0;
}
