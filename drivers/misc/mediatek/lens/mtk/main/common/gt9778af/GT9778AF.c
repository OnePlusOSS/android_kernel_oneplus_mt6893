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

/*Yanjie for HQ
 * gt9778AF voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"


#define AF_DRVNAME "gt9778AF_DRV"
#define AF_I2C_SLAVE_ADDR        0x18

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif


static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;


static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;

static int i2c_read(u8 a_u2Addr, u8 *a_puBuff)
{
    int i4RetValue = 0;
    char puReadCmd[1] = { (char)(a_u2Addr) };

    g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

    g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puReadCmd, 1);
    if (i4RetValue != 1) {
        LOG_INF(" I2C write failed!!\n");
        return -1;
    }
    i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, (char *)a_puBuff, 1);
    if (i4RetValue != 1) {
        LOG_INF(" I2C read failed!!\n");
        return -1;
    }

    return 0;
}

static u8 read_data(u8 addr)
{
    u8 get_byte = 0;
    i2c_read(addr, &get_byte);
    LOG_INF("[gt9778AF] get_byte %d\n", get_byte);
    return get_byte;
}

static int s4AF_ReadReg(unsigned short *a_pu2Result)
{

    *a_pu2Result = (read_data(0x03) << 8) + (read_data(0x04) & 0xff);
    LOG_INF("[gt9778AF]  s4gt9778AF_ReadReg %d\n", *a_pu2Result);
    return 0;
}

static int s4AF_WriteReg(u16 a_u2Data)
{
    int i4RetValue = 0;

    char puSendCmd[3] = {0x03 , (char)(a_u2Data >> 8) , (char)(a_u2Data & 0xFF)};

    g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

    g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 3);

    if (i4RetValue < 0)
    {
        LOG_INF("I2C send failed!! \n");
        return -1;
    }

    return 0;
}

static inline int getAFInfo(struct stAF_MotorInfo *pstMotorInfo)
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

    if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(struct stAF_MotorInfo)))
        LOG_INF("copy to user failed when getting motor information\n");

    return 0;
}



static inline int moveAF(unsigned long a_u4Position)
{
    if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
        LOG_INF("out of range\n");
        return -EINVAL;
    }

    if (g_u4CurrPosition == a_u4Position)
        return 0;

    spin_lock(g_pAF_SpinLock);
    g_u4TargetPosition = a_u4Position;
    spin_unlock(g_pAF_SpinLock);

    if (s4AF_WriteReg((unsigned short)g_u4TargetPosition) == 0) {
        spin_lock(g_pAF_SpinLock);
        g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
        spin_unlock(g_pAF_SpinLock);
    } else {
        LOG_INF("set I2C failed when moving the motor\n");
    }

    return 0;
}

static int initAF(void)
{

    int i4RetValue=0;

    char puSendCmd0[2] = { (char)(0x02), (char)(0x00)};
    char puSendCmd1[2] = { (char)(0x02), (char)(0x02)};
    char puSendCmd2[2] = { (char)(0x06), (char)(0x60)};
    char puSendCmd3[2] = { (char)(0x07), (char)(0x04)};
    char puSendCmd4[2] = { (char)(0x0B), (char)(0x00)};//100mA:0x00, 130mA:0x01

    g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

    g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd0, 2);
    msleep(1);
    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd1, 2);
    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2);
    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd3, 2);
    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd4, 2);

    return 0;
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
long GT9778AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
    long i4RetValue = 0;

    switch (a_u4Command) {
    case AFIOC_G_MOTORINFO:
        i4RetValue = getAFInfo((struct stAF_MotorInfo *) (a_u4Param));
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
int GT9778AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
    LOG_INF("Start\n");

    if (*g_pAF_Opened == 2) {
        LOG_INF("Wait\n");
        s4AF_WriteReg(512);
        msleep(20);
    }
    if (*g_pAF_Opened) {
        LOG_INF("Free\n");

        spin_lock(g_pAF_SpinLock);
        *g_pAF_Opened = 0;
        spin_unlock(g_pAF_SpinLock);
    }

    LOG_INF("End\n");

    return 0;
}

int GT9778AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
            int *pAF_Opened)
{
    g_pstAF_I2Cclient = pstAF_I2Cclient;
    g_pAF_Opened = pAF_Opened;

    LOG_INF(" +\n");
    if (*g_pAF_Opened == 0) {
        LOG_INF("Set power donw +\n");
        LOG_INF("Set power donw -\n");
    }
    LOG_INF(" -\n");

    return 0;
}

int GT9778AF_GetFileName(unsigned char *pFileName)
{
    #if SUPPORT_GETTING_LENS_FOLDER_NAME
    char FilePath[256];
    char *FileString = NULL;

    if (snprintf(FilePath, sizeof(FilePath), "%s", __FILE__) < 0)
        return 0;
    FileString = strrchr(FilePath, '/');
    if (FileString == NULL)
        return 0;
    *FileString = '\0';
    FileString = (strrchr(FilePath, '/') + 1);
    strncpy(pFileName, FileString, AF_MOTOR_NAME);
    LOG_INF("FileName : %s\n", pFileName);
    #else
    pFileName[0] = '\0';
    #endif
    return 1;
}

int GT9778AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
    int ret = 0;
    unsigned short InitPos;
    g_pstAF_I2Cclient = pstAF_I2Cclient;
    g_pAF_SpinLock = pAF_SpinLock;
    g_pAF_Opened = pAF_Opened;
    initAF();

        ret = s4AF_ReadReg(&InitPos);

        if (ret == 0) {
            LOG_INF("Init Pos %6d\n", InitPos);
            spin_lock(g_pAF_SpinLock);
            g_u4CurrPosition = (unsigned long)InitPos;
            spin_unlock(g_pAF_SpinLock);

        } else {
            spin_lock(g_pAF_SpinLock);
            g_u4CurrPosition = 0;
            spin_unlock(g_pAF_SpinLock);
        }

        spin_lock(g_pAF_SpinLock);
        *g_pAF_Opened = 2;
        spin_unlock(g_pAF_SpinLock);


    return 1;
}
