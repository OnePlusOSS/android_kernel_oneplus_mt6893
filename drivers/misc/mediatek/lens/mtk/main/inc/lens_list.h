/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */



#ifndef _LENS_LIST_H

#define _LENS_LIST_H
#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif
extern void MAIN2AF_PowerDown(void);

#define FP5513E4AF_SetI2Cclient FP5513E4AF_SetI2Cclient_Main
#define FP5513E4AF_Ioctl FP5513E4AF_Ioctl_Main
#define FP5513E4AF_Release FP5513E4AF_Release_Main
#define FP5513E4AF_GetFileName FP5513E4AF_GetFileName_Main
extern int FP5513E4AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
        spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long FP5513E4AF_Ioctl(struct file *a_pstFile,
        unsigned int a_u4Command, unsigned long a_u4Param);
extern int FP5513E4AF_Release(struct inode *a_pstInode,
        struct file *a_pstFile);
extern int FP5513E4AF_GetFileName(unsigned char *pFileName);


#define FP5513E4AF_S5KJN103_SetI2Cclient FP5513E4AF_S5KJN103_SetI2Cclient_Main
#define FP5513E4AF_S5KJN103_Ioctl FP5513E4AF_S5KJN103_Ioctl_Main
#define FP5513E4AF_S5KJN103_Release FP5513E4AF_S5KJN103_Release_Main
#define FP5513E4AF_S5KJN103_GetFileName FP5513E4AF_S5KJN103_GetFileName_Main
extern int FP5513E4AF_S5KJN103_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
        spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long FP5513E4AF_S5KJN103_Ioctl(struct file *a_pstFile,
        unsigned int a_u4Command, unsigned long a_u4Param);
extern int FP5513E4AF_S5KJN103_Release(struct inode *a_pstInode,
        struct file *a_pstFile);
extern int FP5513E4AF_S5KJN103_GetFileName(unsigned char *pFileName);


#define FP5513AF_SetI2Cclient FP5513AF_SetI2Cclient_Main
#define FP5513AF_Ioctl FP5513AF_Ioctl_Main
#define FP5513AF_Release FP5513AF_Release_Main
#define FP5513AF_GetFileName FP5513AF_GetFileName_Main
extern int FP5513AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
        spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long FP5513AF_Ioctl(struct file *a_pstFile,
        unsigned int a_u4Command, unsigned long a_u4Param);
extern int FP5513AF_Release(struct inode *a_pstInode,
        struct file *a_pstFile);
extern int FP5513AF_GetFileName(unsigned char *pFileName);

#define DW9718TAF_SetI2Cclient DW9718TAF_SetI2Cclient_Main
#define DW9718TAF_Ioctl DW9718TAF_Ioctl_Main
#define DW9718TAF_Release DW9718TAF_Release_Main
#define DW9718TAF_GetFileName DW9718TAF_GetFileName_Main
extern int DW9718TAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9718TAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9718TAF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9718TAF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int DW9718TAF_GetFileName(unsigned char *pFileName);

#define AK7371AF_SetI2Cclient AK7371AF_SetI2Cclient_Main
#define AK7371AF_Ioctl AK7371AF_Ioctl_Main
#define AK7371AF_Release AK7371AF_Release_Main
#define AK7371AF_PowerDown AK7371AF_PowerDown_Main
#define AK7371AF_GetFileName AK7371AF_GetFileName_Main
extern int AK7371AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long AK7371AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int AK7371AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int AK7371AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int AK7371AF_GetFileName(unsigned char *pFileName);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
extern int AK7314AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long AK7314AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int AK7314AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int AK7314AF_GetFileName(unsigned char *pFileName);
extern int AK7375CAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long AK7375CAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int AK7375CAF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int AK7375CAF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int AK7375CAF_GetFileName(unsigned char *pFileName);
extern int AK7375CAF_DoExtLdo(int enable);
/*20201225 add for 20817 sensor af*/
extern int AK7377AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long AK7377AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int AK7377AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int AK7377AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int AK7377AF_GetFileName(unsigned char *pFileName);
extern int AK7377AF_DoExtLdo(int enable);
extern int AK7375CAF_updatePIDparam(void);
extern int AK7375CAF_checkPIDparam(void);
#endif

#define BU6424AF_SetI2Cclient BU6424AF_SetI2Cclient_Main
#define BU6424AF_Ioctl BU6424AF_Ioctl_Main
#define BU6424AF_Release BU6424AF_Release_Main
#define BU6424AF_GetFileName BU6424AF_GetFileName_Main
extern int BU6424AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long BU6424AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int BU6424AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int BU6424AF_GetFileName(unsigned char *pFileName);

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define BU64253AF_SetI2Cclient BU64253AF_SetI2Cclient_Main
#define BBU64253AF_Ioctl BU64253AF_Ioctl_Main
#define BBU64253AF_Release BU64253AF_Release_Main
#define BU64253AF_GetFileName BU64253AF_GetFileName_Main
extern int BU64253AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long BU64253AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int BU64253AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int BU64253AF_GetFileName(unsigned char *pFileName);

#define DW9718TAF_ZHAOYUN_SetI2Cclient DW9718TAF_ZHAOYUN_SetI2Cclient_Main
#define DW9718TAF_ZHAOYUN_Ioctl DW9718TAF_ZHAOYUN_Ioctl_Main
#define DW9718TAF_ZHAOYUN_Release DW9718TAF_ZHAOYUN_Release_Main
#define DW9718TAF_ZHAOYUN_GetFileName DW9718TAF_ZHAOYUN_GetFileName_Main
extern int DW9718TAF_ZHAOYUN_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9718TAF_ZHAOYUN_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9718TAF_ZHAOYUN_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9718TAF_ZHAOYUN_GetFileName(unsigned char *pFileName);

#define GT9772AF_LIMU_IMX355_SetI2Cclient GT9772AF_LIMU_IMX355_SetI2Cclient_Main
#define GT9772AF_LIMU_IMX355_Ioctl GT9772AF_LIMU_IMX355_Ioctl_Main
#define GT9772AF_LIMU_IMX355_Release GT9772AF_LIMU_IMX355_Release_Main
#define GT9772AF_LIMU_IMX355_GetFileName GT9772AF_LIMU_IMX355_GetFileName_Main
extern int GT9772AF_LIMU_IMX355_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long GT9772AF_LIMU_IMX355_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int GT9772AF_LIMU_IMX355_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int GT9772AF_LIMU_IMX355_GetFileName(unsigned char *pFileName);
#define DW9718TAF_LIMU_SetI2Cclient DW9718TAF_LIMU_SetI2Cclient_Main
#define DW9718TAF_LIMU_Ioctl DW9718TAF_LIMU_Ioctl_Main
#define DW9718TAF_LIMU_Release DW9718TAF_LIMU_Release_Main
#define DW9718TAF_LIMU_GetFileName DW9718TAF_LIMU_GetFileName_Main
extern int DW9718TAF_LIMU_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9718TAF_LIMU_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9718TAF_LIMU_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9718TAF_LIMU_GetFileName(unsigned char *pFileName);
#define FP5513AF_LIMU_SetI2Cclient FP5513AF_LIMU_SetI2Cclient_Main
#define FP5513AF_LIMU_Ioctl FP5513AF_LIMU_Ioctl_Main
#define FP5513AF_LIMU_Release FP5513AF_LIMU_Release_Main
#define FP5513AF_LIMU_GetFileName FP5513AF_LIMU_GetFileName_Main
extern int FP5513AF_LIMU_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long FP5513AF_LIMU_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int FP5513AF_LIMU_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int FP5513AF_LIMU_GetFileName(unsigned char *pFileName);
#endif

#define FP5516AF_SetI2Cclient FP5516AF_SetI2Cclient_Main
#define FP5516AF_Ioctl FP5516AF_Ioctl_Main
#define FP5516AF_Release FP5516AF_Release_Main
#define FP5516AF_PowerDown FP5516AF_PowerDown_Main
#define FP5516AF_GetFileName FP5516AF_GetFileName_Main
extern int FP5516AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long FP5516AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int FP5516AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int FP5516AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int FP5516AF_GetFileName(unsigned char *pFileName);
extern int bu64748af_SetI2Cclient_Main(struct i2c_client *pstAF_I2Cclient,
				       spinlock_t *pAF_SpinLock,
				       int *pAF_Opened);
extern long bu64748af_Ioctl_Main(struct file *a_pstFile,
				 unsigned int a_u4Command,
				 unsigned long a_u4Param);
extern int bu64748af_Release_Main(struct inode *a_pstInode,
				  struct file *a_pstFile);
extern int bu64748af_PowerDown_Main(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int bu64748af_GetFileName_Main(unsigned char *pFileName);

#define BU64253AF_SetI2Cclient BU64253AF_SetI2Cclient_Main
#define BU64253AF_Ioctl BU64253AF_Ioctl_Main
#define BU64253AF_Release BU64253AF_Release_Main
#define BU64253AF_GetFileName BU64253AF_GetFileName_Main
extern int BU64253AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long BU64253AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,   unsigned long a_u4Param);
extern int BU64253AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int BU64253AF_GetFileName(unsigned char *pFileName);

#define BU64253GWZAF_SetI2Cclient BU64253GWZAF_SetI2Cclient_Main
#define BU64253GWZAF_Ioctl BU64253GWZAF_Ioctl_Main
#define BU64253GWZAF_Release BU64253GWZAF_Release_Main
#define BU64253GWZAF_GetFileName BU64253GWZAF_GetFileName_Main
extern int BU64253GWZAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long BU64253GWZAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int BU64253GWZAF_Release(struct inode *a_pstInode,
				 struct file *a_pstFile);
extern int BU64253GWZAF_GetFileName(unsigned char *pFileName);

#define BU6429AF_SetI2Cclient BU6429AF_SetI2Cclient_Main
#define BU6429AF_Ioctl BU6429AF_Ioctl_Main
#define BU6429AF_Release BU6429AF_Release_Main
#define BU6429AF_GetFileName BU6429AF_GetFileName_Main
extern int BU6429AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long BU6429AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int BU6429AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int BU6429AF_GetFileName(unsigned char *pFileName);

#ifdef CONFIG_MTK_LENS_BU63165AF_SUPPORT
#define BU63165AF_SetI2Cclient BU63165AF_SetI2Cclient_Main
#define BU63165AF_Ioctl BU63165AF_Ioctl_Main
#define BU63165AF_Release BU63165AF_Release_Main
#define BU63165AF_GetFileName BU63165AF_GetFileName_Main
extern int BU63165AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				  spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long BU63165AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			    unsigned long a_u4Param);
extern int BU63165AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int BU63165AF_GetFileName(unsigned char *pFileName);
#else
#define BU63169AF_SetI2Cclient BU63169AF_SetI2Cclient_Main
#define BU63169AF_Ioctl BU63169AF_Ioctl_Main
#define BU63169AF_Release BU63169AF_Release_Main
#define BU63169AF_PowerDown BU63169AF_PowerDown_Main
#define BU63169AF_GetFileName BU63169AF_GetFileName_Main
extern int BU63169AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				  spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long BU63169AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			    unsigned long a_u4Param);
extern int BU63169AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int BU63169AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int BU63169AF_GetFileName(unsigned char *pFileName);
#endif

#define DW9714AF_SetI2Cclient DW9714AF_SetI2Cclient_Main
#define DW9714AF_Ioctl DW9714AF_Ioctl_Main
#define DW9714AF_Release DW9714AF_Release_Main
#define DW9714AF_GetFileName DW9714AF_GetFileName_Main
extern int DW9714AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9714AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9714AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9714AF_GetFileName(unsigned char *pFileName);

#define DW9718TAF_SetI2Cclient DW9718TAF_SetI2Cclient_Main
#define DW9718TAF_Ioctl DW9718TAF_Ioctl_Main
#define DW9718TAF_Release DW9718TAF_Release_Main
#define DW9718TAF_GetFileName DW9718TAF_GetFileName_Main
extern int DW9718TAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9718TAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9718TAF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9718TAF_GetFileName(unsigned char *pFileName);
#define DW9718TAF_PARKERA_OV13B10_SetI2Cclient DW9718TAF_PARKERA_OV13B10_SetI2Cclient_Main
#define DW9718TAF_PARKERA_OV13B10_Ioctl DW9718TAF_PARKERA_OV13B10_Ioctl_Main
#define DW9718TAF_PARKERA_OV13B10_Release DW9718TAF_PARKERA_OV13B10_Release_Main
#define DW9718TAF_PARKERA_OV13B10_GetFileName DW9718TAF_PARKERA_OV13B10_GetFileName_Main
extern int DW9718TAF_PARKERA_OV13B10_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9718TAF_PARKERA_OV13B10_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9718TAF_PARKERA_OV13B10_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9718TAF_PARKERA_OV13B10_GetFileName(unsigned char *pFileName);

#define DW9718TAF_PARKERA_OV13B2A_SetI2Cclient DW9718TAF_PARKERA_OV13B2A_SetI2Cclient_Main
#define DW9718TAF_PARKERA_OV13B2A_Ioctl DW9718TAF_PARKERA_OV13B2A_Ioctl_Main
#define DW9718TAF_PARKERA_OV13B2A_Release DW9718TAF_PARKERA_OV13B2A_Release_Main
#define DW9718TAF_PARKERA_OV13B2A_GetFileName DW9718TAF_PARKERA_OV13B2A_GetFileName_Main
extern int DW9718TAF_PARKERA_OV13B2A_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9718TAF_PARKERA_OV13B2A_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9718TAF_PARKERA_OV13B2A_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9718TAF_PARKERA_OV13B2A_GetFileName(unsigned char *pFileName);

#define DW9718TAF_PARKERA_S5KJN103_SetI2Cclient DW9718TAF_PARKERA_S5KJN103_SetI2Cclient_Main
#define DW9718TAF_PARKERA_S5KJN103_Ioctl DW9718TAF_PARKERA_S5KJN103_Ioctl_Main
#define DW9718TAF_PARKERA_S5KJN103_Release DW9718TAF_PARKERA_S5KJN103_Release_Main
#define DW9718TAF_PARKERA_S5KJN103_GetFileName DW9718TAF_PARKERA_S5KJN103_GetFileName_Main
extern int DW9718TAF_PARKERA_S5KJN103_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9718TAF_PARKERA_S5KJN103_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9718TAF_PARKERA_S5KJN103_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9718TAF_PARKERA_S5KJN103_GetFileName(unsigned char *pFileName);

#define DW9718TAF_PARKERB_S5KJN103_SetI2Cclient DW9718TAF_PARKERB_S5KJN103_SetI2Cclient_Main
#define DW9718TAF_PARKERB_S5KJN103_Ioctl DW9718TAF_PARKERB_S5KJN103_Ioctl_Main
#define DW9718TAF_PARKERB_S5KJN103_Release DW9718TAF_PARKERB_S5KJN103_Release_Main
#define DW9718TAF_PARKERB_S5KJN103_GetFileName DW9718TAF_PARKERB_S5KJN103_GetFileName_Main
extern int DW9718TAF_PARKERB_S5KJN103_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9718TAF_PARKERB_S5KJN103_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9718TAF_PARKERB_S5KJN103_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9718TAF_PARKERB_S5KJN103_GetFileName(unsigned char *pFileName);

#define DW9718TAF_EVENC_S5KJN103_SetI2Cclient DW9718TAF_EVENC_S5KJN103_SetI2Cclient_Main
#define DW9718TAF_EVENC_S5KJN103_Ioctl DW9718TAF_EVENC_S5KJN103_Ioctl_Main
#define DW9718TAF_EVENC_S5KJN103_Release DW9718TAF_EVENC_S5KJN103_Release_Main
#define DW9718TAF_EVENC_S5KJN103_GetFileName DW9718TAF_EVENC_S5KJN103_GetFileName_Main
extern int DW9718TAF_EVENC_S5KJN103_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9718TAF_EVENC_S5KJN103_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9718TAF_EVENC_S5KJN103_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9718TAF_EVENC_S5KJN103_GetFileName(unsigned char *pFileName);
#define DW9763AF_SetI2Cclient DW9763AF_SetI2Cclient_Main
#define DW9763AF_Ioctl DW9763AF_Ioctl_Main
#define DW9763AF_Release DW9763AF_Release_Main
#define DW9763AF_GetFileName DW9763AF_GetFileName_Main
extern int DW9763AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9763AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9763AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9763AF_GetFileName(unsigned char *pFileName);

#define DW9800AF_SetI2Cclient DW9800AF_SetI2Cclient_Main
#define DW9800AF_Ioctl DW9800AF_Ioctl_Main
#define DW9800AF_Release DW9800AF_Release_Main
#define DW9800AF_PowerDown DW9800AF_PowerDown_Main
#define DW9800AF_GetFileName DW9800AF_GetFileName_Main
extern int DW9800AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9800AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9800AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9800AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int DW9800AF_GetFileName(unsigned char *pFileName);

#define B0954P65AF_SetI2Cclient B0954P65AF_SetI2Cclient_Main
#define B0954P65AF_Ioctl B0954P65AF_Ioctl_Main
#define B0954P65AF_Release B0954P65AF_Release_Main
#define B0954P65AF_PowerDown B0954P65AF_PowerDown_Main
#define B0954P65AF_GetFileName B0954P65AF_GetFileName_Main
extern int B0954P65AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long B0954P65AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int B0954P65AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int B0954P65AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
                int *pAF_Opened);
extern int B0954P65AF_GetFileName(unsigned char *pFileName);

#define FP5510E2AF_SetI2Cclient FP5510E2AF_SetI2Cclient_Main
#define FP5510E2AF_Ioctl FP5510E2AF_Ioctl_Main
#define FP5510E2AF_Release FP5510E2AF_Release_Main
#define FP5510E2AF_GetFileName FP5510E2AF_GetFileName_Main
extern int FP5510E2AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
	spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long FP5510E2AF_Ioctl(struct file *a_pstFile,
	unsigned int a_u4Command, unsigned long a_u4Param);
extern int FP5510E2AF_Release(struct inode *a_pstInode,
	struct file *a_pstFile);
extern int FP5510E2AF_GetFileName(unsigned char *pFileName);

#define DW9814AF_SetI2Cclient DW9814AF_SetI2Cclient_Main
#define DW9814AF_Ioctl DW9814AF_Ioctl_Main
#define DW9814AF_Release DW9814AF_Release_Main
#define DW9814AF_GetFileName DW9814AF_GetFileName_Main
extern int DW9814AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9814AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9814AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9814AF_GetFileName(unsigned char *pFileName);

#define DW9800WAF_SetI2Cclient DW9800WAF_SetI2Cclient_Main
#define DW9800WAF_Ioctl DW9800WAF_Ioctl_Main
#define DW9800WAF_Release DW9800WAF_Release_Main
#define DW9800WAF_GetFileName DW9800WAF_GetFileName_Main
extern int DW9800WAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
	spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9800WAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
	unsigned long a_u4Param);
extern int DW9800WAF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9800WAF_GetFileName(unsigned char *pFileName);

#define DW9827AF_SetI2Cclient DW9827AF_SetI2Cclient_Main
#define DW9827AF_Ioctl DW9827AF_Ioctl_Main
#define DW9827AF_Release DW9827AF_Release_Main
#define DW9827AF_GetFileName DW9827AF_GetFileName_Main
extern int DW9827AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
	spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9827AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
	unsigned long a_u4Param);
extern int DW9827AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9827AF_GetFileName(unsigned char *pFileName);

#define AK7316AF_SetI2Cclient AK7316AF_SetI2Cclient_Main
#define AK7316AF_Ioctl AK7316AF_Ioctl_Main
#define AK7316AF_Release AK7316AF_Release_Main
#define AK7316AF_GetFileName AK7316AF_GetFileName_Main
extern int AK7316AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
	spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long AK7316AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
	unsigned long a_u4Param);
extern int AK7316AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int AK7316AF_GetFileName(unsigned char *pFileName);

extern struct regulator *regulator_get_regVCAMAF(void);

#define DW9718AF_SetI2Cclient DW9718AF_SetI2Cclient_Main
#define DW9718AF_Ioctl DW9718AF_Ioctl_Main
#define DW9718AF_Release DW9718AF_Release_Main
#define DW9718AF_GetFileName DW9718AF_GetFileName_Main
extern int DW9718AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9718AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9718AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9718AF_GetFileName(unsigned char *pFileName);

#define DW9718TAF_SetI2Cclient DW9718TAF_SetI2Cclient_Main
#define DW9718TAF_Ioctl DW9718TAF_Ioctl_Main
#define DW9718TAF_Release DW9718TAF_Release_Main
#define DW9718TAF_PowerDown DW9718TAF_PowerDown_Main
#define DW9718TAF_GetFileName DW9718TAF_GetFileName_Main
extern int DW9718TAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				  spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9718TAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			    unsigned long a_u4Param);
extern int DW9718TAF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9718TAF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int DW9718TAF_GetFileName(unsigned char *pFileName);

#define DW9718SAF_SetI2Cclient DW9718SAF_SetI2Cclient_Main
#define DW9718SAF_Ioctl DW9718SAF_Ioctl_Main
#define DW9718SAF_Release DW9718SAF_Release_Main
#define DW9718SAF_PowerDown DW9718SAF_PowerDown_Main
#define DW9718SAF_GetFileName DW9718SAF_GetFileName_Main
extern int DW9718SAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				  spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9718SAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			    unsigned long a_u4Param);
extern int DW9718SAF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9718SAF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int DW9718SAF_GetFileName(unsigned char *pFileName);

#define DW9719TAF_SetI2Cclient DW9719TAF_SetI2Cclient_Main
#define DW9719TAF_Ioctl DW9719TAF_Ioctl_Main
#define DW9719TAF_Release DW9719TAF_Release_Main
#define DW9719TAF_GetFileName DW9719TAF_GetFileName_Main
extern int DW9719TAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				  spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9719TAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			    unsigned long a_u4Param);
extern int DW9719TAF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9719TAF_GetFileName(unsigned char *pFileName);

#define DW9763AF_SetI2Cclient DW9763AF_SetI2Cclient_Main
#define DW9763AF_Ioctl DW9763AF_Ioctl_Main
#define DW9763AF_Release DW9763AF_Release_Main
#define DW9763AF_GetFileName DW9763AF_GetFileName_Main
extern int DW9763AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				  spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9763AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			    unsigned long a_u4Param);
extern int DW9763AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9763AF_GetFileName(unsigned char *pFileName);

#define DW9839AF_SetI2Cclient DW9839AF_SetI2Cclient_Main
#define DW9839AF_Ioctl DW9839AF_Ioctl_Main
#define DW9839AF_Release DW9839AF_Release_Main
#define DW9839AF_PowerDown DW9839AF_PowerDown_Main
#define DW9839AF_GetFileName DW9839AF_GetFileName_Main
extern int DW9839AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9839AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9839AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9839AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int DW9839AF_GetFileName(unsigned char *pFileName);

#define GT9768AF_SetI2Cclient GT9768AF_SetI2Cclient_Main
#define GT9768AF_Ioctl GT9768AF_Ioctl_Main
#define GT9768AF_Release GT9768AF_Release_Main
#define GT9768AF_PowerDown GT9768AF_PowerDown_Main
#define GT9768AF_GetFileName GT9768AF_GetFileName_Main
extern int GT9768AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long GT9768AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
				unsigned long a_u4Param);
extern int GT9768AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int GT9768AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int GT9768AF_GetFileName(unsigned char *pFileName);

#define GT9764AF_SetI2Cclient GT9764AF_SetI2Cclient_Main
#define GT9764AF_Ioctl GT9764AF_Ioctl_Main
#define GT9764AF_Release GT9764AF_Release_Main
#define GT9764AF_PowerDown GT9764AF_PowerDown_Main
#define GT9764AF_GetFileName GT9764AF_GetFileName_Main
extern int GT9764AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long GT9764AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
				unsigned long a_u4Param);
extern int GT9764AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int GT9764AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int GT9764AF_GetFileName(unsigned char *pFileName);

#define GT9764AFI_SetI2Cclient GT9764AFI_SetI2Cclient_Main
#define GT9764AFI_Ioctl GT9764AFI_Ioctl_Main
#define GT9764AFI_Release GT9764AFI_Release_Main
#define GT9764AFI_PowerDown GT9764AFI_PowerDown_Main
#define GT9764AFI_GetFileName GT9764AFI_GetFileName_Main
extern int GT9764AFI_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long GT9764AFI_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
				unsigned long a_u4Param);
extern int GT9764AFI_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int GT9764AFI_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int GT9764AFI_GetFileName(unsigned char *pFileName);
#define GT9764AFII_SetI2Cclient GT9764AFII_SetI2Cclient_Main
#define GT9764AFII_Ioctl GT9764AFII_Ioctl_Main
#define GT9764AFII_Release GT9764AFII_Release_Main
#define GT9764AFII_PowerDown GT9764AFII_PowerDown_Main
#define GT9764AFII_GetFileName GT9764AFII_GetFileName_Main
extern int GT9764AFII_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long GT9764AFII_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
				unsigned long a_u4Param);
extern int GT9764AFII_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int GT9764AFII_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int GT9764AFII_GetFileName(unsigned char *pFileName);

#define LC898122AF_SetI2Cclient LC898122AF_SetI2Cclient_Main
#define LC898122AF_Ioctl LC898122AF_Ioctl_Main
#define LC898122AF_Release LC898122AF_Release_Main
#define LC898122AF_GetFileName LC898122AF_GetFileName_Main
extern int LC898122AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				   spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long LC898122AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			     unsigned long a_u4Param);
extern int LC898122AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int LC898122AF_GetFileName(unsigned char *pFileName);

#define LC898212AF_SetI2Cclient LC898212AF_SetI2Cclient_Main
#define LC898212AF_Ioctl LC898212AF_Ioctl_Main
#define LC898212AF_Release LC898212AF_Release_Main
#define LC898212AF_GetFileName LC898212AF_GetFileName_Main
extern int LC898212AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				   spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long LC898212AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			     unsigned long a_u4Param);
extern int LC898212AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int LC898212AF_GetFileName(unsigned char *pFileName);

#define LC898212XDAF_SetI2Cclient LC898212XDAF_SetI2Cclient_Main
#define LC898212XDAF_Ioctl LC898212XDAF_Ioctl_Main
#define LC898212XDAF_Release LC898212XDAF_Release_Main
#define LC898212XDAF_GetFileName LC898212XDAF_GetFileName_Main
extern int LC898212XDAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				     spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long LC898212XDAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			       unsigned long a_u4Param);
extern int LC898212XDAF_Release(struct inode *a_pstInode,
				struct file *a_pstFile);
extern int LC898212XDAF_GetFileName(unsigned char *pFileName);

#define LC898214AF_SetI2Cclient LC898214AF_SetI2Cclient_Main
#define LC898214AF_Ioctl LC898214AF_Ioctl_Main
#define LC898214AF_Release LC898214AF_Release_Main
#define LC898214AF_GetFileName LC898214AF_GetFileName_Main
extern int LC898214AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				   spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long LC898214AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			     unsigned long a_u4Param);
extern int LC898214AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int LC898214AF_GetFileName(unsigned char *pFileName);

#define LC898217AF_SetI2Cclient LC898217AF_SetI2Cclient_Main
#define LC898217AF_Ioctl LC898217AF_Ioctl_Main
#define LC898217AF_Release LC898217AF_Release_Main
#define LC898217AF_PowerDown LC898217AF_PowerDown_Main
#define LC898217AF_GetFileName LC898217AF_GetFileName_Main
extern int LC898217AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				   spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long LC898217AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			     unsigned long a_u4Param);
extern int LC898217AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int LC898217AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int LC898217AF_GetFileName(unsigned char *pFileName);

#define LC898217AFA_SetI2Cclient LC898217AFA_SetI2Cclient_Main
#define LC898217AFA_Ioctl LC898217AFA_Ioctl_Main
#define LC898217AFA_Release LC898217AFA_Release_Main
#define LC898217AFA_PowerDown LC898217AFA_PowerDown_Main
#define LC898217AFA_GetFileName LC898217AFA_GetFileName_Main
extern int LC898217AFA_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				   spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long LC898217AFA_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			     unsigned long a_u4Param);
extern int LC898217AFA_Release(struct inode *a_pstInode,
			       struct file *a_pstFile);
extern int LC898217AFA_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int LC898217AFA_GetFileName(unsigned char *pFileName);

#define OV5645AF_SetI2Cclient OV5645AF_SetI2Cclient_Main
#define OV5645AF_Ioctl OV5645AF_Ioctl_Main
#define OV5645AF_Release OV5645AF_Release_Main
extern int OV5645AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
			spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long OV5645AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			unsigned long a_u4Param);
extern int OV5645AF_Release(struct inode *a_pstInode, struct file *a_pstFile);

#define LC898217AFB_SetI2Cclient LC898217AFB_SetI2Cclient_Main
#define LC898217AFB_Ioctl LC898217AFB_Ioctl_Main
#define LC898217AFB_Release LC898217AFB_Release_Main
#define LC898217AFB_PowerDown LC898217AFB_PowerDown_Main
#define LC898217AFB_GetFileName LC898217AFB_GetFileName_Main
extern int LC898217AFB_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				   spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long LC898217AFB_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			     unsigned long a_u4Param);
extern int LC898217AFB_Release(struct inode *a_pstInode,
			       struct file *a_pstFile);
extern int LC898217AFB_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int LC898217AFB_GetFileName(unsigned char *pFileName);

#define LC898217AFC_SetI2Cclient LC898217AFC_SetI2Cclient_Main
#define LC898217AFC_Ioctl LC898217AFC_Ioctl_Main
#define LC898217AFC_Release LC898217AFC_Release_Main
#define LC898217AFC_PowerDown LC898217AFC_PowerDown_Main
#define LC898217AFC_GetFileName LC898217AFC_GetFileName_Main
extern int LC898217AFC_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				   spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long LC898217AFC_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			     unsigned long a_u4Param);
extern int LC898217AFC_Release(struct inode *a_pstInode,
			       struct file *a_pstFile);
extern int LC898217AFC_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int LC898217AFC_GetFileName(unsigned char *pFileName);

#define LC898229AF_SetI2Cclient LC898229AF_SetI2Cclient_Main
#define LC898229AF_Ioctl LC898229AF_Ioctl_Main
#define LC898229AF_Release LC898229AF_Release_Main
#define LC898229AF_PowerDown LC898229AF_PowerDown_Main
#define LC898229AF_GetFileName LC898229AF_GetFileName_Main
extern int LC898229AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				   spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long LC898229AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			     unsigned long a_u4Param);
extern int LC898229AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int LC898229AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int LC898229AF_GetFileName(unsigned char *pFileName);

#define WV511AAF_SetI2Cclient WV511AAF_SetI2Cclient_Main
#define WV511AAF_Ioctl WV511AAF_Ioctl_Main
#define WV511AAF_Release WV511AAF_Release_Main
#define WV511AAF_GetFileName WV511AAF_GetFileName_Main
extern int WV511AAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long WV511AAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int WV511AAF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int WV511AAF_GetFileName(unsigned char *pFileName);

#define GT9772AF_SetI2Cclient GT9772AF_SetI2Cclient_Main
#define GT9772AF_Ioctl GT9772AF_Ioctl_Main
#define GT9772AF_Release GT9772AF_Release_Main
#define GT9772AF_PowerDown GT9772AF_PowerDown_Main
#define GT9772AF_GetFileName GT9772AF_GetFileName_Main
extern int GT9772AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long GT9772AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
				unsigned long a_u4Param);
extern int GT9772AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int GT9772AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int GT9772AF_GetFileName(unsigned char *pFileName);

#define GT9778AF_SetI2Cclient GT9778AF_SetI2Cclient_Main
#define GT9778AF_Ioctl GT9778AF_Ioctl_Main
#define GT9778AF_Release GT9778AF_Release_Main
#define GT9778AF_PowerDown GT9778AF_PowerDown_Main
#define GT9778AF_GetFileName GT9778AF_GetFileName_Main
extern int GT9778AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long GT9778AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
				unsigned long a_u4Param);
extern int GT9778AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int GT9778AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int GT9778AF_GetFileName(unsigned char *pFileName);

#define GT9778AF_S5KJN103_SetI2Cclient GT9778AF_S5KJN103_SetI2Cclient_Main
#define GT9778AF_S5KJN103_Ioctl GT9778AF_S5KJN103_Ioctl_Main
#define GT9778AF_S5KJN103_Release GT9778AF_S5KJN103_Release_Main
#define GT9778AF_S5KJN103_PowerDown GT9778AF_S5KJN103_PowerDown_Main
#define GT9778AF_S5KJN103_GetFileName GT9778AF_S5KJN103_GetFileName_Main
extern int GT9778AF_S5KJN103_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long GT9778AF_S5KJN103_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
				unsigned long a_u4Param);
extern int GT9778AF_S5KJN103_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int GT9778AF_S5KJN103_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int GT9778AF_S5KJN103_GetFileName(unsigned char *pFileName);

#define FP5516AF_SetI2Cclient FP5516AF_SetI2Cclient_Main
#define FP5516AF_Ioctl FP5516AF_Ioctl_Main
#define FP5516AF_Release FP5516AF_Release_Main
#define FP5516AF_PowerDown FP5516AF_PowerDown_Main
#define FP5516AF_GetFileName FP5516AF_GetFileName_Main
extern int FP5516AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
                                spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long FP5516AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
                                unsigned long a_u4Param);
extern int FP5516AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int FP5516AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
                                int *pAF_Opened);

extern int FP5516AF_GetFileName(unsigned char *pFileName);

#define JD5516WE4_SetI2Cclient JD5516WE4_SetI2Cclient_Main
#define JD5516WE4_Ioctl JD5516WE4_Ioctl_Main
#define JD5516WE4_Release JD5516WE4_Release_Main
#define JD5516WE4_GetFileName JD5516WE4_GetFileName_Main
extern int JD5516WE4_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
	spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long JD5516WE4_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
	unsigned long a_u4Param);
extern int JD5516WE4_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int JD5516WE4_GetFileName(unsigned char *pFileName);

#define JD5516WE4AF_23689_IMX890_SetI2Cclient JD5516WE4AF_23689_IMX890_SetI2Cclient_Main
#define JD5516WE4AF_23689_IMX890_Ioctl JD5516WE4AF_23689_IMX890_Ioctl_Main
#define JD5516WE4AF_23689_IMX890_Release JD5516WE4AF_23689_IMX890_Release_Main
#define JD5516WE4AF_23689_IMX890_GetFileName JD5516WE4AF_23689_IMX890_GetFileName_Main
extern int JD5516WE4AF_23689_IMX890_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
	spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long JD5516WE4AF_23689_IMX890_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
	unsigned long a_u4Param);
extern int JD5516WE4AF_23689_IMX890_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int JD5516WE4AF_23689_IMX890_GetFileName(unsigned char *pFileName);

extern struct regulator *regulator_get_regVCAMAF(void);
#define AW8601_CHANEL_OV64B_SetI2Cclient AW8601_CHANEL_OV64B_SetI2Cclient_Main
#define AW8601_CHANEL_OV64B_Ioctl AW8601_CHANEL_OV64B_Ioctl_Main
#define AW8601_CHANEL_OV64B_Release AW8601_CHANEL_OV64B_Release_Main
#define AW8601_CHANEL_OV64B_PowerDown AW8601_CHANEL_OV64B_PowerDown_Main
#define AW8601_CHANEL_OV64B_GetFileName AW8601_CHANEL_OV64B_GetFileName_Main
extern int AW8601_CHANEL_OV64B_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
                                spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long AW8601_CHANEL_OV64B_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
                                unsigned long a_u4Param);
extern int AW8601_CHANEL_OV64B_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int AW8601_CHANEL_OV64B_PowerDown(struct i2c_client *pstAF_I2Cclient,
                                int *pAF_Opened);

extern int AW8601_CHANEL_OV64B_GetFileName(unsigned char *pFileName);
#define GT9772AF_ALADDIN_SetI2Cclient GT9772AF_ALADDIN_SetI2Cclient_Main
#define GT9772AF_ALADDIN_Ioctl GT9772AF_ALADDIN_Ioctl_Main
#define GT9772AF_ALADDIN_Release GT9772AF_ALADDIN_Release_Main
#define GT9772AF_ALADDIN_PowerDown GT9772AF_ALADDIN_PowerDown_Main
#define GT9772AF_ALADDIN_GetFileName GT9772AF_ALADDIN_GetFileName_Main
extern int GT9772AF_ALADDIN_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
                                spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long GT9772AF_ALADDIN_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
                                unsigned long a_u4Param);
extern int GT9772AF_ALADDIN_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int GT9772AF_ALADDIN_PowerDown(struct i2c_client *pstAF_I2Cclient,
                                int *pAF_Opened);
extern int GT9772AF_ALADDIN_GetFileName(unsigned char *pFileName);

#define GT9772AF_ATOM_SetI2Cclient GT9772AF_ATOM_SetI2Cclient_Main
#define GT9772AF_ATOM_Ioctl GT9772AF_ATOM_Ioctl_Main
#define GT9772AF_ATOM_Release GT9772AF_ATOM_Release_Main
#define GT9772AF_ATOM_PowerDown GT9772AF_ATOM_PowerDown_Main
#define GT9772AF_ATOM_GetFileName GT9772AF_ATOM_GetFileName_Main
extern int GT9772AF_ATOM_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
                                spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long GT9772AF_ATOM_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
                                unsigned long a_u4Param);
extern int GT9772AF_ATOM_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int GT9772AF_ATOM_PowerDown(struct i2c_client *pstAF_I2Cclient,
                                int *pAF_Opened);
extern int GT9772AF_ATOM_GetFileName(unsigned char *pFileName);

#define GT9772AF_ATOM_IMX355_SetI2Cclient GT9772AF_ATOM_IMX355_SetI2Cclient_Main
#define GT9772AF_ATOM_IMX355_Ioctl GT9772AF_ATOM_IMX355_Ioctl_Main
#define GT9772AF_ATOM_IMX355_Release GT9772AF_ATOM_IMX355_Release_Main
#define GT9772AF_ATOM_IMX355_PowerDown GT9772AF_ATOM_IMX355_PowerDown_Main
#define GT9772AF_ATOM_IMX355_GetFileName GT9772AF_ATOM_IMX355_GetFileName_Main
extern int GT9772AF_ATOM_IMX355_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
                                spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long GT9772AF_ATOM_IMX355_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
                                unsigned long a_u4Param);
extern int GT9772AF_ATOM_IMX355_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int GT9772AF_ATOM_IMX355_PowerDown(struct i2c_client *pstAF_I2Cclient,
                                int *pAF_Opened);
extern int GT9772AF_ATOM_IMX355_GetFileName(unsigned char *pFileName);

#define DW9827AF_23231_IMX882_SetI2Cclient DW9827AF_23231_IMX882_SetI2Cclient_Main
#define DW9827AF_23231_IMX882_Ioctl DW9827AF_23231_IMX882_Ioctl_Main
#define DW9827AF_23231_IMX882_Release DW9827AF_23231_IMX882_Release_Main
#define DW9827AF_23231_IMX882_GetFileName DW9827AF_23231_IMX882_GetFileName_Main
extern int DW9827AF_23231_IMX882_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
	spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9827AF_23231_IMX882_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
	unsigned long a_u4Param);
extern int DW9827AF_23231_IMX882_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9827AF_23231_IMX882_GetFileName(unsigned char *pFileName);

#define DW9827AF_23687_IMX882_SetI2Cclient DW9827AF_23687_IMX882_SetI2Cclient_Main
#define DW9827AF_23687_IMX882_Ioctl DW9827AF_23687_IMX882_Ioctl_Main
#define DW9827AF_23687_IMX882_Release DW9827AF_23687_IMX882_Release_Main
#define DW9827AF_23687_IMX882_GetFileName DW9827AF_23687_IMX882_GetFileName_Main
extern int DW9827AF_23687_IMX882_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
	spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9827AF_23687_IMX882_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
	unsigned long a_u4Param);
extern int DW9827AF_23687_IMX882_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9827AF_23687_IMX882_GetFileName(unsigned char *pFileName);

#define DW9827AF_23231_IMX882PD_SetI2Cclient DW9827AF_23231_IMX882PD_SetI2Cclient_Main
#define DW9827AF_23231_IMX882PD_Ioctl DW9827AF_23231_IMX882PD_Ioctl_Main
#define DW9827AF_23231_IMX882PD_Release DW9827AF_23231_IMX882PD_Release_Main
#define DW9827AF_23231_IMX882PD_GetFileName DW9827AF_23231_IMX882PD_GetFileName_Main
extern int DW9827AF_23231_IMX882PD_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
	spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9827AF_23231_IMX882PD_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
	unsigned long a_u4Param);
extern int DW9827AF_23231_IMX882PD_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9827AF_23231_IMX882PD_GetFileName(unsigned char *pFileName);

#define DW9800SAF_SetI2Cclient DW9800SAF_SetI2Cclient_Main
#define DW9800SAF_Ioctl DW9800SAF_Ioctl_Main
#define DW9800SAF_Release DW9800SAF_Release_Main
#define DW9800SAF_GetFileName DW9800SAF_GetFileName_Main
extern int DW9800SAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
	spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9800SAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
	unsigned long a_u4Param);
extern int DW9800SAF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9800SAF_GetFileName(unsigned char *pFileName);
#endif

#define DW9827AF_23687_IMX882PD_SetI2Cclient DW9827AF_23687_IMX882PD_SetI2Cclient_Main
#define DW9827AF_23687_IMX882PD_Ioctl DW9827AF_23687_IMX882PD_Ioctl_Main
#define DW9827AF_23687_IMX882PD_Release DW9827AF_23687_IMX882PD_Release_Main
#define DW9827AF_23687_IMX882PD_GetFileName DW9827AF_23687_IMX882PD_GetFileName_Main
extern int DW9827AF_23687_IMX882PD_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
	spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9827AF_23687_IMX882PD_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
	unsigned long a_u4Param);
extern int DW9827AF_23687_IMX882PD_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9827AF_23687_IMX882PD_GetFileName(unsigned char *pFileName);

#define DW9800SAFPD_SetI2Cclient DW9800SAFPD_SetI2Cclient_Main
#define DW9800SAFPD_Ioctl DW9800SAFPD_Ioctl_Main
#define DW9800SAFPD_Release DW9800SAFPD_Release_Main
#define DW9800SAFPD_GetFileName DW9800SAFPD_GetFileName_Main
extern int DW9800SAFPD_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
	spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9800SAFPD_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
	unsigned long a_u4Param);
extern int DW9800SAFPD_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9800SAFPD_GetFileName(unsigned char *pFileName);