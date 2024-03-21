/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*
 * FP5516AF voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"


#define AF_DRVNAME "FP5516AF_DRV"
#define AF_I2C_SLAVE_ADDR        0x18

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...)                                               \
	pr_info(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif


static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;


static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4CurrPosition;

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include <linux/errno.h>
extern void gpio_dump_regs_range(int start, int end);
extern void AFRegulatorCtrl(int Stage);

static void i2c_errorhandle(int status)
{
	LOG_INF("Start (%d)\n", status);

	gpio_dump_regs_range(112,113);

	if (status == 0)
		AFRegulatorCtrl(2);
	else
		AFRegulatorCtrl(1);

	usleep_range(30000, 30500);

	//check i2c status
	gpio_dump_regs_range(112,113);
	LOG_INF("End (%d)\n", status);
}

static int s4AF_WriteRegForDebug(u8 a_uAddr, u16 a_u2Data)
{
	u8 puSendCmd[2] = {a_uAddr, (u8)(a_u2Data & 0xFF)};
	struct timespec mTS1;
	struct timespec mTS2;

	int ret = -1;

	g_pstAF_I2Cclient->addr = (AF_I2C_SLAVE_ADDR) >> 1;

	/* LOG_INF("WRI2C 0x%04x, 0x%x\n", a_uAddr, a_u2Data); */
	mTS1 = current_kernel_time();
	ret = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);
	mTS2 = current_kernel_time();

	if (ret != 2) {
		unsigned long long start_ms, end_ms;
		unsigned int diff_ms;

		start_ms = (mTS1.tv_sec * NSEC_PER_SEC +
				mTS1.tv_nsec) / 1000000;
		end_ms = (mTS2.tv_sec * NSEC_PER_SEC +
				mTS2.tv_nsec) / 1000000;

		diff_ms = (unsigned int)(end_ms - start_ms);
		if (diff_ms > 600)
			i2c_errorhandle(0);

		LOG_INF("ReadI2C send failed!!\n");

		return -1;
	}

	return 0;
}
#endif /*OPLUS_FEATURE_CAMERA_COMMON*/

#if 0
static int s4AF_ReadReg(unsigned short *a_pu2Result)
{
	int i4RetValue = 0;
	char pBuff[2];

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, pBuff, 2);

	if (i4RetValue < 0) {
		LOG_INF("I2C read failed!!\n");
		return -1;
	}

	*a_pu2Result = (((u16)pBuff[0]) << 4) + (pBuff[1] >> 4);

	return 0;
}
#endif

static int s4AF_WriteReg(u16 a_u2Data)
{
	int i4RetValue = 0;

	char puSendCmd[3] = { 0x03, (char)(a_u2Data >> 8), (char)(a_u2Data & 0xFF) };

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 3);

	if (i4RetValue < 0) {
		LOG_INF("I2C send failed!!\n");
		return -1;
	}

	return 0;
}

static inline int getAFInfo(__user struct stAF_MotorInfo *pstMotorInfo)
{
	struct stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo,
			 sizeof(struct stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

/* initAF include driver initialization and standby mode */
static int initAF(void)
{
	int i4RetValue = 0;
	char puSendCmdArray[7][2] = {
	{0x02, 0x01}, {0x02, 0x00}, {0xFE, 0xFE},
	{0x02, 0x02}, {0x06, 0x40}, {0x07, 0x60}, {0xFE, 0xFE},
	};
	unsigned char cmd_number;

	LOG_INF("InitDrv[1] %p, %p\n", &(puSendCmdArray[1][0]), puSendCmdArray[1]);
	LOG_INF("InitDrv[2] %p, %p\n", &(puSendCmdArray[2][0]), puSendCmdArray[2]);

	#ifndef OPLUS_FEATURE_CAMERA_COMMON
	for (cmd_number = 0; cmd_number < 7; cmd_number++) {
		if (puSendCmdArray[cmd_number][0] != 0xFE) {
			i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmdArray[cmd_number], 2);

			if (i4RetValue < 0)
				return -1;
		} else {
			udelay(100);
		}
	}
	#else /*OPLUS_FEATURE_CAMERA_COMMON*/
	mdelay(4);
	for (cmd_number = 0; cmd_number < 7; cmd_number++) {
		if (puSendCmdArray[cmd_number][0] != 0xFE) {
			//i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmdArray[cmd_number], 2);
			i4RetValue = s4AF_WriteRegForDebug(puSendCmdArray[cmd_number][0],puSendCmdArray[cmd_number][1]);

			if (i4RetValue < 0)
				return -1;
		} else {
			mdelay(5);
		}
	}
	#endif /*OPLUS_FEATURE_CAMERA_COMMON*/

	return i4RetValue;
}


/* moveAF only use to control moving the motor */
static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;

	if (s4AF_WriteReg((unsigned short)a_u4Position) == 0) {
		g_u4CurrPosition = a_u4Position;
		ret = 0;
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
		ret = -1;
	}

	return ret;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
long FP5516AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
		    unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue =
			getAFInfo((__user struct stAF_MotorInfo *)(a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;

	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int FP5516AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2)
		LOG_INF("Wait\n");

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	LOG_INF("End\n");

	return 0;
}

int FP5516AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
			  spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	#ifndef OPLUS_FEATURE_CAMERA_COMMON
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;

	initAF();
	#else /*OPLUS_FEATURE_CAMERA_COMMON*/
	int i = 0;
	int ret = 0;

	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;

	ret = initAF();
	if (ret < 0) {
		LOG_INF("re-init1 +\n");
		for (i = 0; i < 20; i++) {
			msleep(200);
			gpio_dump_regs_range(112,113);
		}

		ret = initAF();
		LOG_INF("re-init1 -\n");
	}
	#endif /*OPLUS_FEATURE_CAMERA_COMMON*/

	return 1;
}

int FP5516AF_GetFileName(unsigned char *pFileName)
{
	#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char FilePath[256];
	char *FileString;

	sprintf(FilePath, "%s", __FILE__);
	FileString = strrchr(FilePath, '/');
	*FileString = '\0';
	FileString = (strrchr(FilePath, '/') + 1);
	strncpy(pFileName, FileString, AF_MOTOR_NAME);
	LOG_INF("FileName : %s\n", pFileName);
	#else
	pFileName[0] = '\0';
	#endif
	return 1;
}
