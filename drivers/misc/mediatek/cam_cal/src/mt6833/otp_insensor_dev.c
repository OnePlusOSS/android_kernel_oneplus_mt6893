/***********************************************************
 * ** Copyright (C), 2008-2016, OPLUS Mobile Comm Corp., Ltd.
 * ** ODM_HQ_EDIT
 * ** File: - otp_insensor_dev.c
 * ** Description: Source file for CBufferList.
 * **           To allocate and free memory block safely.
 * ** Version: 1.0
 * ** Date : 2018/12/07
 * ** Author: YanKun.Zhai@Mutimedia.camera.driver.otp
 * **
 * ** ------------------------------- Revision History: -------------------------------
 * **   <author>History<data>      <version >version       <desc>
 * **  YanKun.Zhai 2018/12/07     1.0     build this module
 * **
 * ****************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>

#include "cam_cal.h"
#include "cam_cal_define.h"
#include "cam_cal_list.h"

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#ifdef ODM_HQ_EDIT
/*Houbing.Peng@ODM_HQ Cam.Drv 20200915 add for otp*/
#include <soc/oplus/system/oplus_project.h>
#endif

#define LOG_TAG "OTP_InSensor"
#define LOG_ERR(format, ...) pr_err(LOG_TAG "Line:%d  FUNC: %s:  "format, __LINE__, __func__, ## __VA_ARGS__)
#define LOG_INFO(format, ...) pr_info(LOG_TAG "  %s:  "format, __func__, ## __VA_ARGS__)
#define LOG_DEBUG(format, ...) pr_debug(LOG_TAG "  %s:  "format, __func__, ## __VA_ARGS__)

#define ERROR_I2C        1
#define ERROR_CHECKSUM   2
#define ERROR_READ_FLAG  3
#define ERROR_GROUP_FLAG 4
#define OTP_THREAHOLD    3



static DEFINE_SPINLOCK(g_spinLock);

static struct i2c_client *g_pstI2CclientG;
/* add for linux-4.4 */
#ifndef I2C_WR_FLAG
#define I2C_WR_FLAG		(0x1000)
#define I2C_MASK_FLAG	(0x00ff)
#endif

struct i2c_client *g_pstI2Cclients[3]; /* I2C_DEV_IDX_MAX */


#define MAX_EEPROM_BYTE 0x1FFF
#define CHECKSUM_FLAG_ADDR MAX_EEPROM_BYTE-1
#define READ_FLAG_ADDR MAX_EEPROM_BYTE-2

#define OTP_DATA_GOOD_FLAG 0x88
char g_otp_buf[3][MAX_EEPROM_BYTE] = {{0}, {0}, {0}};

#define MAX_NAME_LENGTH 20
#define MAX_GROUP_NUM 8
#define MAX_GROUP_ADDR_NUM 3


typedef struct
{
	int group_start_addr;
	int group_end_addr;
	int group_flag_addr;
	int group_checksum_addr;
}GROUP_ADDR_INFO;

typedef struct
{
	char group_name[MAX_NAME_LENGTH];
	GROUP_ADDR_INFO group_addr_info[MAX_GROUP_ADDR_NUM];
}GROUP_INFO;


typedef struct
{
	char module_name[MAX_NAME_LENGTH];
	int group_num;
	int group_addr_info_num;
	GROUP_INFO group_info[MAX_GROUP_NUM];
	int  (* readFunc) (u16 addr, u8 *data);
}OTP_MAP;


int hi556_read_data(u16 addr, u8 *data);

OTP_MAP hi556_otp_map_blade = {
		.module_name = "HI556_Blade",
		.group_num = 3,
		.group_addr_info_num = 3,
		.readFunc = hi556_read_data,
		.group_info = {
						{"info",
							{
								{0x0401, 0x040F, 0x0410, 0x0411},
								{0x0C49, 0x0C57, 0x0C58, 0x0C59},
								{0x1491, 0x149F, 0x14A0, 0x14A1},
							},
						},
						{"awb",
							{
								{0x0421, 0x0430, 0x0431, 0x0432},
								{0x0C69, 0x0C78, 0x0C79, 0x0C7A},
								{0x14B1, 0x14C0, 0x14C1, 0x14C2},
							 },
						},
						{"lsc",
							{
								{0x0433, 0x0B7E, 0x0B7F, 0x0B80},
								{0X0C7B, 0x13C6, 0x13C7, 0x13C8},
								{0x14C3, 0x1C0E, 0x1C0F, 0x1C10},
							},
						},
			},
};

OTP_MAP hi556_otp_map_lijinga = {
		.module_name = "HI556_LIJINGA",
		.group_num = 4,
		.group_addr_info_num = 3,
		.readFunc = hi556_read_data,
		.group_info = {
						{"info",
							{
								{0x0401, 0x040F, 0x0410, 0x0411},
								{0x0C4F, 0x0C5D, 0x0C5E, 0x0C5F},
								{0x149D, 0x14AB, 0x14AC, 0x14AD},
							},
						},
						{"awb",
							{
								{0x0421, 0x0430, 0x0431, 0x0432},
								{0x0C6F, 0x0C7E, 0x0C7F, 0x0C80},
								{0x14BD, 0x14CC, 0x14CD, 0x14CE},
							 },
						},
						{"lsc",
							{
								{0x0433, 0x0B7E, 0x0B7F, 0x0B80},
								{0X0C81, 0x13CC, 0x13CD, 0x13CE},
								{0x14CF, 0x1C1A, 0x1C1B, 0x1C1C},
							},
						},
						{"sn",
							{
								{0x0C35, 0x0C4C, 0x0C4D, 0x0C4E},
								{0x1483, 0x149A, 0x149B, 0x149C},
								{0x1CD1, 0x1CE8, 0x1CE9, 0x1CEA},
							},
						},
			},
};

int hi846_read_data(u16 addr, u8 *data);

OTP_MAP hi846_otp_map_lijing = {
		.module_name = "HI846_LIJING",
		.group_num = 4,
		.group_addr_info_num = 3,
		.readFunc = hi846_read_data,
		.group_info = {
						{"info",
							{
								{0x0201, 0x020F, 0x0210, 0x0211},
								{0x0212, 0x0220, 0x0221, 0x0222},
								{0x0223, 0x0231, 0x0232, 0x0233},
							},
						},
						{"awb",
							{
								{0x0234, 0x0243, 0x0244, 0x0245},
								{0x0246, 0x0255, 0x0256, 0x0257},
								{0x0258, 0x0267, 0x0268, 0x0269},
							 },
						},
						{"lsc",
							{
								{0x026A, 0x09B5, 0x09B6, 0x09B7},
								{0x09B8, 0x1103, 0x1104, 0x1105},
								{0x1106, 0x1851, 0x1852, 0x1853},
							},
						},
						{"sn",
							{
								{0x1854, 0x1865, 0x1866, 0x1867},
								{0x1868, 0x1879, 0x187A, 0x187B},
								{0x187C, 0x188D, 0x188E, 0x188F},
							},
						},
			},
};

OTP_MAP hi846_otp_map_lijinga = {
		.module_name = "HI846_LIJINGA",
		.group_num = 4,
		.group_addr_info_num = 3,
		.readFunc = hi846_read_data,
		.group_info = {
						{"info",
							{
								{0x0201, 0x020F, 0x0210, 0x0211},
								{0x0212, 0x0220, 0x0221, 0x0222},
								{0x0223, 0x0231, 0x0232, 0x0233},
							},
						},
						{"awb",
							{
								{0x0234, 0x0243, 0x0244, 0x0245},
								{0x0246, 0x0255, 0x0256, 0x0257},
								{0x0258, 0x0267, 0x0268, 0x0269},
							 },
						},
						{"lsc",
							{
								{0x026A, 0x09B5, 0x09B6, 0x09B7},
								{0x09B8, 0x1103, 0x1104, 0x1105},
								{0x1106, 0x1851, 0x1852, 0x1853},
							},
						},
						{"sn",
							{
								{0x1854, 0x186B, 0x186C, 0x186D},
								{0x186E, 0x1885, 0x1886, 0x1887},
								{0x1888, 0x189F, 0x18A0, 0x18A1},
							},
						},
			},
};

/*This sensor only have two group otp data.in diffrent page in sensor*/
int sc520cs_read_data(u16 addr, u8 *data);
OTP_MAP sc520cs_otp_map_lijinga = {
		.module_name = "SC520CS_LIJINGA",
		.group_num = 4,
		.group_addr_info_num = 3,
		.readFunc = sc520cs_read_data,
		.group_info = {
						{"info",
							{
								{0x8826, 0x8834, 0x8835, 0x8836}, /*this part is for first materials*/
								{0x8827, 0x8835, 0x8826, 0x8836}, /*this part is for new materials group 1*/
								{0x8827, 0x8835, 0x8826, 0x8836}, /*this part is for new materials group 2*/
							},
						},
						{"awb",
							{
								{0x8837, 0x8846, 0x8847, 0x8848},
								{0x885D, 0x886C, 0x885C, 0x886D},
								{0x886E, 0x887D, 0x885C, 0x887E},
							 },
						},
						{"lsc",
							{
								{0x8849, 0x8F94, 0x8F95, 0x8F96},
								{0x8880, 0x8FCB, 0x887F, 0x8FCC},
								{0x8FCD, 0x87FE, 0x887F, 0x87FF}, /*this part is for new materials group 2,Consists of two pages*/
							},
						},
						{"sn",
							{
								{0x8F97, 0x8FAE, 0x8FAF, 0x8FB0},
								{0x8838, 0x884F, 0x8837, 0x8850},
								{0x8838, 0x885A, 0x8837, 0x885B},/*this group consists of two parts,part 1:0x8838 - 0x8845;part 2:0x8851 - 0x885A*/
							},
						},
			},
};

static int read_reg16_data8(u16 addr, u8 *data)
{
	int ret = 0;
	char puSendCmd[2] = {(char)(addr >> 8), (char)(addr & 0xff)};

	spin_lock(&g_spinLock);
		g_pstI2CclientG->addr =
			g_pstI2CclientG->addr & (I2C_MASK_FLAG | I2C_WR_FLAG);
	spin_unlock(&g_spinLock);

	ret = i2c_master_send(g_pstI2CclientG, puSendCmd, 2);
	if (ret != 2) {
		pr_err("I2C send failed!!, Addr = 0x%x\n", addr);
		return -1;
	}
	ret = i2c_master_recv(g_pstI2CclientG, (char *)data, 1);
	if (ret != 1) {
		pr_err("I2C read failed!!\n");
		return -1;
	}
	spin_lock(&g_spinLock);
	g_pstI2CclientG->addr = g_pstI2CclientG->addr & I2C_MASK_FLAG;
	spin_unlock(&g_spinLock);

	return 0;
}
static int write_reg16_data8(u16 addr, u8 data)
{
	int ret = 0;
	char puSendCmd[3] = {(char)(addr >> 8), (char)(addr & 0xff),
						(char)(data & 0xff)};
	spin_lock(&g_spinLock);
	g_pstI2CclientG->addr =
			g_pstI2CclientG->addr & (I2C_MASK_FLAG | I2C_WR_FLAG);
	spin_unlock(&g_spinLock);

	ret = i2c_master_send(g_pstI2CclientG, puSendCmd, 3);

	if (ret != 3) {
		pr_err("I2C send failed!!, Addr = 0x%x\n", addr);
		return -1;
	}
	spin_lock(&g_spinLock);
	g_pstI2CclientG->addr = g_pstI2CclientG->addr & I2C_MASK_FLAG;
	spin_unlock(&g_spinLock);

	return 0;
}

static int parse_otp_map_data(OTP_MAP * map, char * data)
{
	int i = 0, j = 0;
	int addr = 0, size = 0, curr_addr = 0;
	int  ret = 0;
	char readbyte = 0;
	char group_flag = 0;
	int checksum = -1;

	LOG_INFO("module: %s ......", map->module_name);

	for (i = 0; i < map->group_num; i++) {
		checksum = 0;
		size = 0;

		LOG_INFO("groupinfo: %s, start_addr 0x%04x(%04d)", map->group_info[i].group_name, curr_addr, curr_addr);

		 for (j = 0; j < map->group_addr_info_num; j++) {
			ret = map->readFunc(map->group_info[i].group_addr_info[j].group_flag_addr, &group_flag);
			if (ret < 0) {
				LOG_ERR("  read group flag error addr 0x%04x", map->group_info[i].group_addr_info[j].group_flag_addr);
				return -ERROR_I2C;
			}
			LOG_INFO("readFunc: group %d flag_addr = 0x%04x, value = 0x%02x\n", j, map->group_info[i].group_addr_info[j].group_flag_addr, group_flag);
			if (group_flag == 0x01) {
				LOG_INFO(" group %d selected\n", j);
				break;
			}
		}

		if (j == map->group_addr_info_num) {
			LOG_ERR("Can't found group flag 0x%x", readbyte);
			return -ERROR_READ_FLAG;
		}

		for (addr = map->group_info[i].group_addr_info[j].group_start_addr;
				addr <= map->group_info[i].group_addr_info[j].group_end_addr;
				addr++) {
			ret = map->readFunc(addr, data);
			if (ret < 0) {
				LOG_ERR(" read data error");
			}
			LOG_DEBUG(" group: %s, addr: 0x%04x, viraddr: 0x%04x(%04d), data: 0x%04x(%04d) ",
						map->group_info[i].group_name, addr, curr_addr, curr_addr, *data, *data);

			checksum += *data;
			curr_addr++;
			size++;
			data++;
		}

		checksum = checksum % 0xFF;
		ret = map->readFunc(map->group_info[i].group_addr_info[j].group_checksum_addr, &readbyte);
		if (checksum == readbyte) {
			LOG_INFO("groupinfo: %s, checksum OK c(%04d) r(%04d)", map->group_info[i].group_name, checksum, readbyte);
		} else {
			LOG_ERR("groupinfo: %s, checksum ERROR ret=%d, checksum=%04d readbyte=%04d", map->group_info[i].group_name, ret, checksum, readbyte);
			ret = -ERROR_CHECKSUM;
		}
		LOG_INFO("groupinfo: %s, end_addr 0x%04x(%04d) size 0x%04x(%04d)",
						map->group_info[i].group_name, curr_addr-1, curr_addr-1, size, size);
	}
	return ret;
}

int hi556_read_data(u16 addr, u8 *data)
{
	static u16 last_addr = 0;
	if (addr != last_addr+ 1) {
		write_reg16_data8(0x010a, (addr >> 8) & 0xff);
		write_reg16_data8(0x010b, addr & 0xff);
		write_reg16_data8(0x0102, 0x01);
	}

	last_addr = addr;
	return read_reg16_data8(0x0108, data);
}
void hi556_otp_read_enable()
{
	write_reg16_data8(0x0a02, 0x01);
	write_reg16_data8(0x0a00, 0x00);
	mdelay(10);
	write_reg16_data8(0x0f02, 0x00);
	write_reg16_data8(0x011a, 0x01);
	write_reg16_data8(0x011b, 0x09);
	write_reg16_data8(0x0d04, 0x01);
	write_reg16_data8(0x0d02, 0x07);
	write_reg16_data8(0x003e, 0x10);
	write_reg16_data8(0x0a00, 0x01);
}
void hi556_otp_read_disable()
{
	write_reg16_data8(0x0a00, 0x00);
	mdelay(10);
	write_reg16_data8(0x004a, 0x00);
	write_reg16_data8(0x0d04, 0x00);
	write_reg16_data8(0x003e, 0x00);
	write_reg16_data8(0x004a, 0x01);
	write_reg16_data8(0x0a00, 0x01);
}

unsigned int Hi556_read_region(struct i2c_client *client, unsigned int addr, unsigned char *data, unsigned int size)
{
	int ret = 0;

	if (g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][READ_FLAG_ADDR]) {
		LOG_INFO("read otp data from g_otp_buf: addr 0x%x, size %d", addr, size);
		memcpy((void *)data, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][addr], size);
		return size;
	}
	if (client != NULL) {
		g_pstI2CclientG = client;
	} else if (g_pstI2Cclients[IMGSENSOR_SENSOR_IDX_SUB] != NULL) {
		g_pstI2CclientG = g_pstI2Cclients[IMGSENSOR_SENSOR_IDX_SUB];
		g_pstI2CclientG->addr = 0x40 >> 1;
	}
	hi556_otp_read_enable();
	ret = parse_otp_map_data(&hi556_otp_map_blade, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][0]);
	if (!ret) {
		g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][CHECKSUM_FLAG_ADDR] = OTP_DATA_GOOD_FLAG;
		g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][READ_FLAG_ADDR] = 1;
	}
	hi556_otp_read_disable();

	if (NULL != data) {
		memcpy((void *)data, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][addr], size);
	}
	return ret;
}

unsigned int Hi556a_read_region(struct i2c_client *client, unsigned int addr, unsigned char *data, unsigned int size)
{
	int ret = 0;

	if (g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][READ_FLAG_ADDR]) {
		LOG_INFO("read otp data from g_otp_buf: addr 0x%x, size %d", addr, size);
		memcpy((void *)data, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][addr], size);
		return size;
	}
	if (client != NULL) {
		g_pstI2CclientG = client;
	} else if (g_pstI2Cclients[IMGSENSOR_SENSOR_IDX_SUB] != NULL) {
		g_pstI2CclientG = g_pstI2Cclients[IMGSENSOR_SENSOR_IDX_SUB];
		g_pstI2CclientG->addr = 0x40 >> 1;
	}
	hi556_otp_read_enable();
	ret = parse_otp_map_data(&hi556_otp_map_lijinga, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][0]);
	if (!ret) {
		g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][CHECKSUM_FLAG_ADDR] = OTP_DATA_GOOD_FLAG;
		g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][READ_FLAG_ADDR] = 1;
	}
	hi556_otp_read_disable();

	if (NULL != data) {
		memcpy((void *)data, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][addr], size);
	}
	return ret;
}

int hi846_read_data(u16 addr, u8 *data)
{
	static u16 last_addr = 0;
	if (addr != last_addr+ 1) {
		write_reg16_data8(0x070a, (addr >> 8) & 0xff);
		write_reg16_data8(0x070b, addr & 0xff);
		write_reg16_data8(0x0702, 0x01);
	}

	last_addr = addr;
	return read_reg16_data8(0x0708, data);
}

void hi846_otp_read_enable()
{
	write_reg16_data8(0x0A02, 0x01);
	write_reg16_data8(0x0A00, 0x00);
	mdelay(10);
	write_reg16_data8(0x0F02, 0x00);
	write_reg16_data8(0x071A, 0x01);
	write_reg16_data8(0x071B, 0x09);
	write_reg16_data8(0x0D04, 0x01);
	write_reg16_data8(0x0D00, 0x07);
	write_reg16_data8(0x003E, 0x10);
	write_reg16_data8(0x0A00, 0x01);
}

void hi846_otp_read_disable()
{
	write_reg16_data8(0x0a00, 0x00);
	mdelay(10);
	write_reg16_data8(0x003e, 0x00);
	write_reg16_data8(0x004a, 0x01);
}

unsigned int Hi846_read_region(struct i2c_client *client, unsigned int addr, unsigned char *data, unsigned int size)
{
	int ret = 0;

	if (g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][READ_FLAG_ADDR]) {
		LOG_INFO("read otp data from g_otp_buf: addr 0x%x, size %d", addr, size);
		memcpy((void *)data, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][addr], size);
		return size;
	}
	if (client != NULL) {
		g_pstI2CclientG = client;
	} else if (g_pstI2Cclients[IMGSENSOR_SENSOR_IDX_SUB] != NULL) {
		g_pstI2CclientG = g_pstI2Cclients[IMGSENSOR_SENSOR_IDX_SUB];
		g_pstI2CclientG->addr = 0x40 >> 1;
	}
	hi846_otp_read_enable();
	ret = parse_otp_map_data(&hi846_otp_map_lijing, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][0]);
	if (!ret) {
		g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][CHECKSUM_FLAG_ADDR] = OTP_DATA_GOOD_FLAG;
		g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][READ_FLAG_ADDR] = 1;
	}
	hi846_otp_read_disable();

	if (NULL != data) {
		memcpy((void *)data, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][addr], size);
	}
	return ret;
}

unsigned int Hi846a_read_region(struct i2c_client *client, unsigned int addr, unsigned char *data, unsigned int size)
{
	int ret = 0;

	if (g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][READ_FLAG_ADDR]) {
		LOG_INFO("read otp data from g_otp_buf: addr 0x%x, size %d", addr, size);
		memcpy((void *)data, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][addr], size);
		return size;
	}
	if (client != NULL) {
		g_pstI2CclientG = client;
	} else if (g_pstI2Cclients[IMGSENSOR_SENSOR_IDX_SUB] != NULL) {
		g_pstI2CclientG = g_pstI2Cclients[IMGSENSOR_SENSOR_IDX_SUB];
		g_pstI2CclientG->addr = 0x40 >> 1;
	}
	hi846_otp_read_enable();
	ret = parse_otp_map_data(&hi846_otp_map_lijinga, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][0]);
	if (!ret) {
		g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][CHECKSUM_FLAG_ADDR] = OTP_DATA_GOOD_FLAG;
		g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][READ_FLAG_ADDR] = 1;
	}
	hi846_otp_read_disable();

	if (NULL != data) {
		memcpy((void *)data, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][addr], size);
	}
	return ret;
}

void sc520cs_sensor_otp_init(int section, kal_uint8 threshold)
{
	/*page 1 0x8000 - 0x87ff : 0x4412,0x01; 0x4407,0x0c
	  page 2 0x8800 - 0x8FFF : 0x4412,0x03; 0x4407,0x00
	*/
	write_reg16_data8(0x36b0, 0x4c);
	write_reg16_data8(0x36b1, threshold);	/* set threshold */
	write_reg16_data8(0x36b2, 0xc1);
	if (section == 1) {
		write_reg16_data8(0x4408, (0x8000 >> 8)&0xff);        /* High byte start address for OTP load Group1 */
		write_reg16_data8(0x4409, (0x8000)&0xff);             /* Low byte start address for OTP load  Group1 */
		write_reg16_data8(0x440a, (0x87FF >> 8)&0xff);        /* High byte end address for OTP load Group1 */
		write_reg16_data8(0x440b, (0x87FF)&0xff);             /* Low byte end address for OTP load Group1 */
		write_reg16_data8(0x4401, 0x13);
		write_reg16_data8(0x4412, 0x01);
		write_reg16_data8(0x4407, 0x0c);
		write_reg16_data8(0x4400, 0x11);                      /* 0x4400, 0x11 manual load */
		LOG_INFO("load section 1 \n");
	} else if (section == 2) {
		write_reg16_data8(0x4408, (0x8800 >> 8)&0xff);        /* High byte start address for OTP load Group1 */
		write_reg16_data8(0x4409, (0x8800)&0xff);             /* Low byte start address for OTP load  Group1 */
		write_reg16_data8(0x440a, (0x8FFF >> 8)&0xff);        /* High byte end address for OTP load Group1 */
		write_reg16_data8(0x440b, (0x8FFF)&0xff);             /* Low byte end address for OTP load Group1 */
		write_reg16_data8(0x4401, 0x13);
		write_reg16_data8(0x4412, 0x03);
		write_reg16_data8(0x4407, 0x00);
		write_reg16_data8(0x4400, 0x11);	                     /* 0x4400, 0x11 manual load */
		LOG_INFO("load section 2 \n");
	} else {
		LOG_INFO("section input error \n");
	}
	mdelay(30);                                              /* wait page load finished */
}

int sc520cs_read_data(u16 addr, u8 *data)
{
	static u16 last_addr = 0;
	last_addr = addr;
	return read_reg16_data8(addr, data);
}

static int parse_otp_map_data_sc520cs(OTP_MAP * map, char * data, int datagroup, char otpThreshold)
{
	int i = 0;
	int addr = 0, size = 0, curr_addr = 0;
	int  ret = 0;
	char readbyte = 0;
	char group_flag = 0;
	int checksum = -1;
	LOG_INFO("module: %s, datagroup : 0x%d......", map->module_name, datagroup);

	for (i = 0; i < map->group_num; i++) {
		checksum = 0;
		size = 0;
		LOG_INFO("groupinfo: %s, start_addr 0x%0x(0x%0x)", map->group_info[i].group_name, curr_addr, curr_addr);
        /* in grupe 2. the lsc data Composed of two parts,part one in page2 .another part in page1,but flage&checksum is inpage1 */
		ret = map->readFunc(map->group_info[i].group_addr_info[datagroup].group_flag_addr, &group_flag);
		LOG_INFO("parse_otp_map_data_sc520cs 0x%x", ret);
		if (ret < 0) {
			LOG_ERR("read group flag error addr 0x%04x", map->group_info[i].group_addr_info[datagroup].group_flag_addr);
			return -ERROR_I2C;
		}
		LOG_INFO("readFunc: group %d flag_addr = 0x%04x, value = 0x%02x\n", datagroup, map->group_info[i].group_addr_info[datagroup].group_flag_addr, group_flag);
		if (group_flag != 0x01 && group_flag != 0x13) {
			LOG_INFO("group %d selected\n", datagroup);
			return -ERROR_GROUP_FLAG;
		}
		if (!strcmp(map->group_info[i].group_name, "lsc") && datagroup == 2) {	/*to get lsc data of  group 2 part 1, addrs from 0x8FCD to 0x8FFF*/
			for (addr = map->group_info[i].group_addr_info[datagroup].group_start_addr;
					addr <= 0x8FFF; addr++) {
				ret = map->readFunc(addr, data);
				if (ret < 0) {
					LOG_ERR("read data error");
					return -ERROR_I2C;
				}
				LOG_DEBUG("Get SC520CS OTP Data of group: %s, addr: 0x%04x, viraddr: 0x%04x(%04d), data: 0x%04x(%04d) ",
							map->group_info[i].group_name, addr, curr_addr, curr_addr, *data, *data);

				checksum += *data;
				curr_addr++;
				size++;
				data++;
			}
			/*to get lsc data of  group 2 part 2, addrs from 0x80E6 to 0x87FE, checksum addrs 0x87FF*/
			sc520cs_sensor_otp_init(1, otpThreshold);
			for (addr = 0x80E6; addr <= map->group_info[i].group_addr_info[datagroup].group_end_addr;
					addr++) {
				ret = map->readFunc(addr, data);
				if (ret < 0) {
					LOG_ERR("read data error");
					return -ERROR_I2C;
				}
				LOG_DEBUG("Get SC520CS OTP Data of group: %s, addr: 0x%04x, viraddr: 0x%04x(%04d), data: 0x%04x(%04d) ",
							map->group_info[i].group_name, addr, curr_addr, curr_addr, *data, *data);

				checksum += *data;
				curr_addr++;
				size++;
				data++;
			}
			ret = map->readFunc(map->group_info[i].group_addr_info[datagroup].group_checksum_addr, &readbyte);
			sc520cs_sensor_otp_init(2, otpThreshold);
		} else if (!strcmp(map->group_info[i].group_name, "sn") && datagroup == 2) {
			/*to get sn data of  group 2 part 1, addrs from 0x8838 to 0x8845*/
			for (addr = map->group_info[i].group_addr_info[datagroup].group_start_addr;
					addr <= 0x8845; addr++) {
				ret = map->readFunc(addr, data);
				if (ret < 0) {
					LOG_ERR("read data error");
					return -ERROR_I2C;
				}
				LOG_DEBUG("Get SC520CS OTP Data of group: %s, addr: 0x%04x, viraddr: 0x%04x(%04d), data: 0x%04x(%04d) ",
							map->group_info[i].group_name, addr, curr_addr, curr_addr, *data, *data);

				checksum += *data;
				curr_addr++;
				size++;
				data++;
			}
			/*to get sn data of  group 2 part 2, addrs from 0x8851 to 0x885A, checksum addr 0x885B*/
			for (addr = 0x8851;addr <= map->group_info[i].group_addr_info[datagroup].group_end_addr; addr++) {
				ret = map->readFunc(addr, data);
				if (ret < 0) {
					LOG_ERR("read data error");
					return -ERROR_I2C;
				}
				LOG_DEBUG("Get SC520CS OTP Data of group: %s, addr: 0x%04x, viraddr: 0x%04x(%04d), data: 0x%04x(%04d) ",
							map->group_info[i].group_name, addr, curr_addr, curr_addr, *data, *data);

				checksum += *data;
				curr_addr++;
				size++;
				data++;
			}
			ret = map->readFunc(map->group_info[i].group_addr_info[datagroup].group_checksum_addr, &readbyte);
		} else {
			/*for first materials and new materials group 1 data. Since its addresses are continuous, there is no need to switch pages to read.  */
			for (addr = map->group_info[i].group_addr_info[datagroup].group_start_addr;
					addr <= map->group_info[i].group_addr_info[datagroup].group_end_addr; addr++) {
				ret = map->readFunc(addr, data);
				if (ret < 0) {
					LOG_ERR("read data error");
					return -ERROR_I2C;
				}
				LOG_DEBUG("Get SC520CS OTP Data of group: %s, addr: 0x%04x, viraddr: 0x%04x(%04d), data: 0x%04x(%04d) ",
							map->group_info[i].group_name, addr, curr_addr, curr_addr, *data, *data);

				checksum += *data;
				curr_addr++;
				size++;
				data++;
			}
			ret = map->readFunc(map->group_info[i].group_addr_info[datagroup].group_checksum_addr, &readbyte);
		}

		checksum = checksum % 0xFF;
		if (checksum == readbyte) {
			LOG_INFO("groupinfo: %s, checksum OK c(%04d) r(%04d)", map->group_info[i].group_name, checksum, readbyte);
		} else {
			LOG_ERR("groupinfo: %s, checksum ERROR ret=%d, checksum=%04d readbyte=%04d", map->group_info[i].group_name, ret, checksum, readbyte);
			return -ERROR_CHECKSUM;
		}
		LOG_INFO("groupinfo: %s, end_addr 0x%04x(%04d) size 0x%04x(%04d)",
						map->group_info[i].group_name, curr_addr-1, curr_addr-1, size, size);
	}
	return ret;
}

unsigned int sc520cs_read_region(struct i2c_client *client, unsigned int addr, unsigned char *data, unsigned int size)
{
	int ret = 0;
	int i;
	int otpThreshold[3] = {0x38, 0x28, 0x48}; /* the Threshold for sc520cs*/
	char forcompatible = 0;
	if (g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][READ_FLAG_ADDR]) {
		LOG_INFO("read otp data from g_otp_buf: addr 0x%x, size %d", addr, size);
		memcpy((void *)data, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][addr], size);
		return size;
	}
	if (client != NULL) {
		g_pstI2CclientG = client;
	} else if (g_pstI2Cclients[IMGSENSOR_SENSOR_IDX_SUB] != NULL) {
		g_pstI2CclientG = g_pstI2Cclients[IMGSENSOR_SENSOR_IDX_SUB];
		g_pstI2CclientG->addr = 0x20 >> 1;
	}
	for (i = 0; i < OTP_THREAHOLD; i++) {
		sc520cs_sensor_otp_init(2, otpThreshold[i]);  /*we need reload the data page before read anther page data.*/
		ret = sc520cs_read_data(0x8826, &forcompatible); /*Check if it is the first batch of materials */
		LOG_INFO("SC520CS Select threshold : 0x%x to read data", otpThreshold[i]);
		if (forcompatible == 0x06) {	/*this part is for first materials*/
			LOG_INFO("SC520CS for first batch of materials");
			ret = parse_otp_map_data_sc520cs(&sc520cs_otp_map_lijinga, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][0], 0, otpThreshold[i]);
		} else if (forcompatible == 0x01) { /*this part is for new materials group 1*/
			LOG_INFO("SC520CS for new materials group 1");
			ret = parse_otp_map_data_sc520cs(&sc520cs_otp_map_lijinga, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][0], 1, otpThreshold[i]);
		} else if (forcompatible == 0x13) { /*this part is for new materials group 2*/
			LOG_INFO("SC520CS for new materials group 2");
			ret = parse_otp_map_data_sc520cs(&sc520cs_otp_map_lijinga, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][0], 2, otpThreshold[i]);
		} else {
			LOG_INFO("No valid OTP group");
			ret = -ERROR_GROUP_FLAG;
		}
		if(!ret)
			break;
	}
	if (!ret) {
		g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][CHECKSUM_FLAG_ADDR] = OTP_DATA_GOOD_FLAG;
		g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][READ_FLAG_ADDR] = 1;
	}

	if (NULL != data) {
		memcpy((void *)data, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][addr], size);
	}
	return ret;
}

