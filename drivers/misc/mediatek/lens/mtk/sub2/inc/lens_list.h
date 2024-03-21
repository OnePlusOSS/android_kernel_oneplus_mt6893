/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */



#ifndef _LENS_LIST_H

#define _LENS_LIST_H

#ifdef OPLUS_FEATURE_CAMERA_COMMON

#define BU64253TEAF_SetI2Cclient BU64253TEAF_SetI2Cclient_Sub2
#define BBU64253TEAF_Ioctl BU64253TEAF_Ioctl_Sub2
#define BBU64253TEAF_Release BU64253TEAF_Release_Sub2
#define BU64253TEAF_GetFileName BU64253TEAF_GetFileName_Sub2
extern int BU64253TEAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long BU64253TEAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int BU64253TEAF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int BU64253TEAF_GetFileName(unsigned char *pFileName);
#endif
extern void AFRegulatorCtrl(int Stage);
#endif
