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

#define PFX "hi5021sqt_caliotp"
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
#include "hi5021sqtmipiraw_Sensor.h"


#define Sleep(ms) mdelay(ms)

#define hi5021sqt_EEPROM_READ_ID  0xA0
#define hi5021sqt_EEPROM_WRITE_ID 0xA1
#define hi5021sqt_I2C_SPEED       100
#define hi5021sqt_MAX_OFFSET      0xFFFF
#define XGC_SIZE 1920
#define QGC_SIZE 1280

struct EEPROM_cali_INFO {
	kal_uint16 XGC_addr;
	unsigned int XGC_size;
	kal_uint16 QGC_addr;
	unsigned int QGC_size;
};

enum EEPROM_cali_INFO_FMT {
	MTK_FMT = 0,
	OP_FMT,
	FMT_MAX
};

static struct EEPROM_cali_INFO eeprom_cali_info[] = {
	{/* MTK_FMT */
		.XGC_addr = 0x2900,
		.XGC_size = XGC_SIZE,
		.QGC_addr = 0x3080,
		.QGC_size = QGC_SIZE
	},
};

//static DEFINE_MUTEX(ghi5021sqt_eeprom_mutex);

static bool hi5021sqt_selective_read_eeprom(kal_uint16 addr, BYTE *data)
{
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	if (addr > hi5021sqt_MAX_OFFSET)
		return false;
	
	if (iReadRegI2C(pu_send_cmd, 2, (u8 *) data, 1, 0xA0 ) < 0) {
		return false;
	} 

	//LOG_INF("addr = 0x%x data = %d\n", addr, *data);
	return true;
}

static bool read_hi5021sqt_eeprom(kal_uint16 addr, BYTE *data, int size)
{
	int i = 0;
	int offset = addr;

	//LOG_INF("enter read_eeprom size = %d\n", size);
	for (i = 0; i < size; i++) {
		if (!hi5021sqt_selective_read_eeprom(offset, &data[i]))
			return false;
		//LOG_INF("read_eeprom 0x%0x %d\n", offset, data[i]);
		offset++;
	}
	return true;
}

static struct EEPROM_cali_INFO *get_eeprom_cali_info(void)
{
	static struct EEPROM_cali_INFO *pinfo;

	if (pinfo == NULL) {
		pinfo = &eeprom_cali_info[MTK_FMT];
	}

	return pinfo;
}

unsigned int read_hi5021sqt_XGC(BYTE *data)
{
	BYTE hi5021sqt_XGC_data[XGC_SIZE] = { 0 };
	unsigned int readed_size = 0;
	struct EEPROM_cali_INFO *pinfo = get_eeprom_cali_info();

	LOG_INF("read hi5021sqt XGC, addr = 0x%x, size = %u readed_size-addr = 0x%x\n",
		pinfo->XGC_addr, pinfo->XGC_size, &readed_size);

	//mutex_lock(&ghi5021sqt_eeprom_mutex);
	if ((readed_size == 0) &&
	    read_hi5021sqt_eeprom(pinfo->XGC_addr,
			       hi5021sqt_XGC_data, pinfo->XGC_size)) {
		readed_size = pinfo->XGC_size;
	}
	//mutex_unlock(&ghi5021sqt_eeprom_mutex);

	memcpy(data, hi5021sqt_XGC_data, pinfo->XGC_size);
	return readed_size;
}


unsigned int read_hi5021sqt_QGC(BYTE *data)
{
	BYTE hi5021sqt_QGC_data[QGC_SIZE] = { 0 };
	unsigned int readed_size = 0;
	struct EEPROM_cali_INFO *pinfo = get_eeprom_cali_info();

	LOG_INF("read hi5021sqt QGC, addr = 0x%x, size = %u\n",
		pinfo->QGC_addr, pinfo->QGC_size);

	//mutex_lock(&ghi5021sqt_eeprom_mutex);
	if ((readed_size == 0) &&
	    read_hi5021sqt_eeprom(pinfo->QGC_addr,
			       hi5021sqt_QGC_data, pinfo->QGC_size)) {
		readed_size = pinfo->QGC_size;
	}
	//mutex_unlock(&ghi5021sqt_eeprom_mutex);

	memcpy(data, hi5021sqt_QGC_data, pinfo->QGC_size);
	return readed_size;
}

