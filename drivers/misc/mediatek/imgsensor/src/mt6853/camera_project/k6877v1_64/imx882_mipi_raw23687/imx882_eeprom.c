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

#define PFX "IMX882_pdafotp"
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
#include "imx882mipiraw_Sensor.h"
#include "imx882_eeprom.h"

static struct EEPROM_PDAF_INFO eeprom_pdaf_info[] = {
    {
        .SPC_addr = OTP_SPC_OFFSET,
        .SPC_size = SPC_SIZE,
    },
};

static DEFINE_MUTEX(gimx882_eeprom_mutex);

static bool selective_read_eeprom(kal_uint16 addr, BYTE *data)
{
    char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

    if (addr > IMX882_MAX_OFFSET)
        return false;

    if (iReadRegI2C(pu_send_cmd, 2, (u8 *) data,
            1, IMX882_EEPROM_SLAVE_ADDRESS) < 0) {
        return false;
    }
    return true;
}

static bool read_imx882_eeprom(kal_uint16 addr, BYTE *data, int size)
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


unsigned int read_imx882_23687_SPC(kal_uint16 *data)
{
    kal_uint16 idx = 0, sensor_startL_reg = 0xD200, sensor_startR_reg = 0xD300;
    static BYTE imx882_SPC_data[SPC_SIZE] = { 0 };
    static unsigned int readed_size = 0;
    struct EEPROM_PDAF_INFO *pinfo = (struct EEPROM_PDAF_INFO *)&eeprom_pdaf_info[0];
    kal_uint8 SPC_flag = 0;

    LOG_INF("read imx882 SPC, otp_offset = %d, size = %u\n",
        pinfo->SPC_addr, pinfo->SPC_size);

    mutex_lock(&gimx882_eeprom_mutex);
    if ((readed_size == 0) &&
        read_imx882_eeprom(pinfo->SPC_addr,
            imx882_SPC_data, pinfo->SPC_size)) {
        readed_size = pinfo->SPC_size;
    }
    mutex_unlock(&gimx882_eeprom_mutex);

    for (idx = 0; idx < SPC_SIZE; idx++) {
        if(idx < SPC_SIZE/2){
            //SPC_Left
            data[2 * idx] = sensor_startL_reg++;
        }else{
            //SPC_Right
            data[2 * idx] = sensor_startR_reg++;
        }
        data[2 * idx + 1] = imx882_SPC_data[idx];
    }

    for (idx = 0; idx < SPC_SIZE; idx++) {
        if(idx < SPC_SIZE/2){
            //SPC_Left
            pr_debug("In %s: SPC_Left value[0x%x]:0x%x", __func__, data[2 * idx], data[2 * idx + 1]);
        }else{
            //SPC_RIGHT
            pr_debug("In %s: SPC_Right value[0x%x]:0x%x", __func__, data[2 * idx], data[2 * idx + 1]);
        }
    }
    read_imx882_eeprom(0x162E, &SPC_flag, 1);
    pr_info("SPC flag0x%x[1:valid, other:Invalid]", SPC_flag);
    return readed_size;
}

unsigned int read_imx882_23687_QSC(kal_uint16 * data)
{
    kal_uint16 idx = 0, sensor_qsc_address = 0xC000;
    kal_uint8 tmp_QSC_setting[QSC_SIZE];
    kal_uint8 qsc_ver = 0;

    read_imx882_eeprom(OTP_QSC_OFFSET, tmp_QSC_setting, QSC_SIZE);
    for (idx = 0; idx < QSC_SIZE; idx++) {
        data[2 * idx] = sensor_qsc_address;
        data[2 * idx + 1] = tmp_QSC_setting[idx];
        sensor_qsc_address += 1;
    }

    for (idx = 0; idx < QSC_SIZE; idx++) {
        pr_debug("alex qsc data imx882_QSC_setting[0x%x] = 0x%x",
            data[2 * idx], data[2 * idx + 1]);
    }

    read_imx882_eeprom(0x2A32, &qsc_ver, 1);
    pr_info("QSC Version: 0x%x", qsc_ver);
    return 0;
}
