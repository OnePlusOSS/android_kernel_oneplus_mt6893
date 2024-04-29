// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

/*
 * MAIN AF voice coil motor driver
 *
 *
 */

#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

/* kernel standard */
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>

/* OIS/EIS Timer & Workqueue */
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/ktime.h>
/* ------------------------- */

#if defined(CONFIG_MACH_MT6779)
#include <archcounter_timesync.h>
#endif

#include "lens_info.h"
#include "lens_list.h"
#include <soc/oplus/system/oplus_project.h>

#define AF_DRVNAME "MAINAF"

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

#if defined(CONFIG_MTK_LEGACY)
#define I2C_CONFIG_SETTING 1
#elif defined(CONFIG_OF)
#define I2C_CONFIG_SETTING 2 /* device tree */
#else

#define I2C_CONFIG_SETTING 1
#endif

#if I2C_CONFIG_SETTING == 1
#define LENS_I2C_BUSNUM 0
#define I2C_REGISTER_ID 0x28
#endif

#define PLATFORM_DRIVER_NAME "lens_actuator_main_af"
#define AF_DRIVER_CLASS_NAME "actuatordrv_main_af"

#if I2C_CONFIG_SETTING == 1
static struct i2c_board_info kd_lens_dev __initdata = {
	I2C_BOARD_INFO(AF_DRVNAME, I2C_REGISTER_ID)};
#endif

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...)                                               \
	pr_info(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif


static struct hrtimer ois_timer;

static DEFINE_MUTEX(ois_mutex);
static int g_EnableTimer;
static int g_GetOisInfoCnt;
static int g_OisPosIdx;
static struct stAF_OisPosInfo OisPosInfo;
/* ------------------------- */

static struct stAF_DrvList g_stAF_DrvList[MAX_NUM_OF_LENS] = {
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{1, AFDRV_FP5513E4AF, FP5513E4AF_SetI2Cclient, FP5513E4AF_Ioctl,
	 FP5513E4AF_Release, FP5513E4AF_GetFileName, NULL},
	{1, AFDRV_FP5513E4AF_S5KJN103, FP5513E4AF_S5KJN103_SetI2Cclient, FP5513E4AF_S5KJN103_Ioctl,
        FP5513E4AF_S5KJN103_Release, FP5513E4AF_S5KJN103_GetFileName, NULL},
	{1, AFDRV_FP5513AF, FP5513AF_SetI2Cclient, FP5513AF_Ioctl,
        FP5513AF_Release, FP5513AF_GetFileName, NULL},
	{1, AFDRV_LC898217AF, LC898217AF_SetI2Cclient, LC898217AF_Ioctl,
	 LC898217AF_Release, LC898217AF_GetFileName, NULL},
	{1, AFDRV_DW9718AF, DW9718AF_SetI2Cclient, DW9718AF_Ioctl,
	 DW9718AF_Release, DW9718AF_GetFileName, NULL},
	{1, AFDRV_DW9718TAF, DW9718TAF_SetI2Cclient, DW9718TAF_Ioctl,
	 DW9718TAF_Release, DW9718TAF_GetFileName, NULL},
	{1, AFDRV_LC898229AF, LC898229AF_SetI2Cclient, LC898229AF_Ioctl,
	 LC898229AF_Release, LC898229AF_GetFileName, NULL},
	{1, AFDRV_AK7375CAF, AK7375CAF_SetI2Cclient, AK7375CAF_Ioctl,
	 AK7375CAF_Release, AK7375CAF_GetFileName, NULL},
	{1, AFDRV_DW9800AF, DW9800AF_SetI2Cclient, DW9800AF_Ioctl,
	 DW9800AF_Release, DW9800AF_GetFileName, NULL},
	{1, AFDRV_AK7377AF, AK7377AF_SetI2Cclient, AK7377AF_Ioctl,
	 AK7377AF_Release, AK7377AF_GetFileName, NULL},
	{1, AFDRV_FP5516AF, FP5516AF_SetI2Cclient, FP5516AF_Ioctl,
	 FP5516AF_Release, FP5516AF_GetFileName, NULL},
	{1, AFDRV_DW9718TAF_EVENC_S5KJN103, DW9718TAF_EVENC_S5KJN103_SetI2Cclient, DW9718TAF_EVENC_S5KJN103_Ioctl,
	 DW9718TAF_EVENC_S5KJN103_Release, DW9718TAF_EVENC_S5KJN103_GetFileName, NULL},
	 {1, AFDRV_B0954P65AF, B0954P65AF_SetI2Cclient, B0954P65AF_Ioctl,
	 B0954P65AF_Release, B0954P65AF_GetFileName, NULL},
	{1, AFDRV_BU64253GWZAF, BU64253GWZAF_SetI2Cclient, BU64253GWZAF_Ioctl,
	 BU64253GWZAF_Release, BU64253GWZAF_GetFileName, NULL},
	{1, AFDRV_AK7314AF, AK7314AF_SetI2Cclient, AK7314AF_Ioctl,
	 AK7314AF_Release, AK7314AF_GetFileName, NULL},
	{1, AFDRV_BU64253AF, BU64253AF_SetI2Cclient, BU64253AF_Ioctl,
	 BU64253AF_Release, BU64253AF_GetFileName, NULL},
	{1, AFDRV_DW9800WAF, DW9800WAF_SetI2Cclient, DW9800WAF_Ioctl,
	DW9800WAF_Release, DW9800WAF_GetFileName, NULL},
	{1, AFDRV_DW9827AF, DW9827AF_SetI2Cclient, DW9827AF_Ioctl,
	DW9827AF_Release, DW9827AF_GetFileName, NULL},
	{1, AFDRV_DW9827AF_23231_IMX882, DW9827AF_23231_IMX882_SetI2Cclient, DW9827AF_23231_IMX882_Ioctl,
	DW9827AF_23231_IMX882_Release, DW9827AF_23231_IMX882_GetFileName, NULL},
	{1, AFDRV_DW9827AF_23687_IMX882, DW9827AF_23687_IMX882_SetI2Cclient, DW9827AF_23687_IMX882_Ioctl,
	DW9827AF_23687_IMX882_Release, DW9827AF_23687_IMX882_GetFileName, NULL},
	{1, AFDRV_DW9827AF_23231_IMX882PD, DW9827AF_23231_IMX882PD_SetI2Cclient, DW9827AF_23231_IMX882PD_Ioctl,
	DW9827AF_23231_IMX882PD_Release, DW9827AF_23231_IMX882PD_GetFileName, NULL},
	{1, AFDRV_DW9827AF_23687_IMX882PD, DW9827AF_23687_IMX882PD_SetI2Cclient, DW9827AF_23687_IMX882PD_Ioctl,
	DW9827AF_23687_IMX882PD_Release, DW9827AF_23687_IMX882PD_GetFileName, NULL},
	{1, AFDRV_DW9800SAF, DW9800SAF_SetI2Cclient, DW9800SAF_Ioctl,
	DW9800SAF_Release, DW9800SAF_GetFileName, NULL},
	{1, AFDRV_DW9800SAFPD, DW9800SAFPD_SetI2Cclient, DW9800SAFPD_Ioctl,
	DW9800SAFPD_Release, DW9800SAFPD_GetFileName, NULL},
	{1, AFDRV_AK7316AF, AK7316AF_SetI2Cclient, AK7316AF_Ioctl,
	AK7316AF_Release, AK7316AF_GetFileName, NULL},
	{1, AFDRV_GT9772AF, GT9772AF_SetI2Cclient, GT9772AF_Ioctl,
	 GT9772AF_Release, GT9772AF_GetFileName, NULL},
	{1, AFDRV_GT9772AF_ALADDIN, GT9772AF_ALADDIN_SetI2Cclient, GT9772AF_ALADDIN_Ioctl,
	 GT9772AF_ALADDIN_Release, GT9772AF_ALADDIN_GetFileName, NULL},
	{1, AFDRV_GT9772AF_ATOM, GT9772AF_ATOM_SetI2Cclient, GT9772AF_ATOM_Ioctl,
	 GT9772AF_ATOM_Release, GT9772AF_ATOM_GetFileName, NULL},
	{1, AFDRV_GT9772AF_ATOM_IMX355, GT9772AF_ATOM_IMX355_SetI2Cclient, GT9772AF_ATOM_IMX355_Ioctl,
	 GT9772AF_ATOM_IMX355_Release, GT9772AF_ATOM_IMX355_GetFileName, NULL},
	{1, AFDRV_DW9718TAF_ZHAOYUN, DW9718TAF_ZHAOYUN_SetI2Cclient, DW9718TAF_ZHAOYUN_Ioctl,
	 DW9718TAF_ZHAOYUN_Release, DW9718TAF_ZHAOYUN_GetFileName, NULL},
	{1, AFDRV_GT9772AF_LIMU_IMX355, GT9772AF_LIMU_IMX355_SetI2Cclient, GT9772AF_LIMU_IMX355_Ioctl,
	 GT9772AF_LIMU_IMX355_Release, GT9772AF_LIMU_IMX355_GetFileName, NULL},
	{1, AFDRV_DW9718TAF_LIMU, DW9718TAF_LIMU_SetI2Cclient, DW9718TAF_LIMU_Ioctl,
	 DW9718TAF_LIMU_Release, DW9718TAF_LIMU_GetFileName, NULL},
	{1, AFDRV_FP5513AF_LIMU, FP5513AF_LIMU_SetI2Cclient, FP5513AF_LIMU_Ioctl,
	 FP5513AF_LIMU_Release, FP5513AF_LIMU_GetFileName, NULL},
	{1, AFDRV_DW9718TAF_PARKERA_OV13B10, DW9718TAF_PARKERA_OV13B10_SetI2Cclient, DW9718TAF_PARKERA_OV13B10_Ioctl,
	 DW9718TAF_PARKERA_OV13B10_Release, DW9718TAF_PARKERA_OV13B10_GetFileName, NULL},
	{1, AFDRV_DW9718TAF_PARKERA_OV13B2A, DW9718TAF_PARKERA_OV13B2A_SetI2Cclient, DW9718TAF_PARKERA_OV13B2A_Ioctl,
	 DW9718TAF_PARKERA_OV13B2A_Release, DW9718TAF_PARKERA_OV13B2A_GetFileName, NULL},
	{1, AFDRV_DW9718TAF_PARKERA_S5KJN103, DW9718TAF_PARKERA_S5KJN103_SetI2Cclient, DW9718TAF_PARKERA_S5KJN103_Ioctl,
	 DW9718TAF_PARKERA_S5KJN103_Release, DW9718TAF_PARKERA_S5KJN103_GetFileName, NULL},
	{1, AFDRV_DW9718TAF_PARKERB_S5KJN103, DW9718TAF_PARKERB_S5KJN103_SetI2Cclient, DW9718TAF_PARKERB_S5KJN103_Ioctl,
	 DW9718TAF_PARKERB_S5KJN103_Release, DW9718TAF_PARKERB_S5KJN103_GetFileName, NULL},
	{1, AFDRV_GT9778AF, GT9778AF_SetI2Cclient, GT9778AF_Ioctl,
	 GT9778AF_Release, GT9778AF_GetFileName, NULL},
	{1, AFDRV_GT9778AF_S5KJN103, GT9778AF_S5KJN103_SetI2Cclient, GT9778AF_S5KJN103_Ioctl,
	 GT9778AF_S5KJN103_Release, GT9778AF_S5KJN103_GetFileName, NULL},
	{1, AFDRV_GT9764AF, GT9764AF_SetI2Cclient, GT9764AF_Ioctl,
	 GT9764AF_Release, GT9764AF_GetFileName, NULL},
	{1, AFDRV_GT9764AFI, GT9764AFI_SetI2Cclient, GT9764AFI_Ioctl,
	 GT9764AFI_Release, GT9764AFI_GetFileName, NULL},
	{1, AFDRV_GT9764AFII, GT9764AFII_SetI2Cclient, GT9764AFII_Ioctl,
	 GT9764AFII_Release, GT9764AFII_GetFileName, NULL},
	{1, AFDRV_FP5513E4AF, FP5513E4AF_SetI2Cclient, FP5513E4AF_Ioctl,
	 FP5513E4AF_Release, FP5513E4AF_GetFileName, NULL},
	{1, AFDRV_JD5516WE4, JD5516WE4_SetI2Cclient, JD5516WE4_Ioctl,
	 JD5516WE4_Release, JD5516WE4_GetFileName, NULL},
	 {1, AFDRV_AW8601, AW8601_CHANEL_OV64B_SetI2Cclient, AW8601_CHANEL_OV64B_Ioctl,
	 AW8601_CHANEL_OV64B_Release, AW8601_CHANEL_OV64B_GetFileName, NULL},
	{1, AFDRV_JD5516WE4AF_23689_IMX890, JD5516WE4AF_23689_IMX890_SetI2Cclient, JD5516WE4AF_23689_IMX890_Ioctl,
	 JD5516WE4AF_23689_IMX890_Release, JD5516WE4AF_23689_IMX890_GetFileName, NULL},
#else

	{1, AFDRV_DW9718TAF, DW9718TAF_SetI2Cclient, DW9718TAF_Ioctl,
	 DW9718TAF_Release, DW9718TAF_GetFileName, NULL},
	{1, AFDRV_GT9772AF, GT9772AF_SetI2Cclient, GT9772AF_Ioctl,
	 GT9772AF_Release, GT9772AF_GetFileName, NULL},
	{1, AFDRV_AK7371AF, AK7371AF_SetI2Cclient, AK7371AF_Ioctl,
	 AK7371AF_Release, AK7371AF_GetFileName, NULL},
	{1, AFDRV_LC898229AF, LC898229AF_SetI2Cclient, LC898229AF_Ioctl,
	LC898229AF_Release, LC898229AF_GetFileName, NULL},
	{1, AFDRV_BU6424AF, BU6424AF_SetI2Cclient, BU6424AF_Ioctl,
	 BU6424AF_Release, BU6424AF_GetFileName, NULL},
	{1, AFDRV_BU6429AF, BU6429AF_SetI2Cclient, BU6429AF_Ioctl,
	 BU6429AF_Release, BU6429AF_GetFileName, NULL},
	{1, AFDRV_BU64748AF, bu64748af_SetI2Cclient_Main, bu64748af_Ioctl_Main,
	 bu64748af_Release_Main, bu64748af_GetFileName_Main, NULL},
	{1, AFDRV_BU64253GWZAF, BU64253GWZAF_SetI2Cclient, BU64253GWZAF_Ioctl,
	 BU64253GWZAF_Release, BU64253GWZAF_GetFileName, NULL},
	{1,
#ifdef CONFIG_MTK_LENS_BU63165AF_SUPPORT
	 AFDRV_BU63165AF, BU63165AF_SetI2Cclient, BU63165AF_Ioctl,
	 BU63165AF_Release, BU63165AF_GetFileName, NULL
#else
	 AFDRV_BU63169AF, BU63169AF_SetI2Cclient, BU63169AF_Ioctl,
	 BU63169AF_Release, BU63169AF_GetFileName, NULL
#endif
	},
	{1, AFDRV_DW9714AF, DW9714AF_SetI2Cclient, DW9714AF_Ioctl,
	 DW9714AF_Release, DW9714AF_GetFileName, NULL},
	{1, AFDRV_DW9718SAF, DW9718SAF_SetI2Cclient, DW9718SAF_Ioctl,
	 DW9718SAF_Release, DW9718SAF_GetFileName, NULL},
	{1, AFDRV_DW9719TAF, DW9719TAF_SetI2Cclient, DW9719TAF_Ioctl,
	 DW9719TAF_Release, DW9719TAF_GetFileName, NULL},
	{1, AFDRV_DW9763AF, DW9763AF_SetI2Cclient, DW9763AF_Ioctl,
	 DW9763AF_Release, DW9763AF_GetFileName, NULL},
	{1, AFDRV_LC898212XDAF, LC898212XDAF_SetI2Cclient, LC898212XDAF_Ioctl,
	 LC898212XDAF_Release, LC898212XDAF_GetFileName, NULL},
	{1, AFDRV_DW9800WAF, DW9800WAF_SetI2Cclient, DW9800WAF_Ioctl,
	DW9800WAF_Release, DW9800WAF_GetFileName, NULL},
	{1, AFDRV_DW9827AF, DW9827AF_SetI2Cclient, DW9827AF_Ioctl,
	DW9827AF_Release, DW9827AF_GetFileName, NULL},
	{1, AFDRV_AK7316AF, AK7316AF_SetI2Cclient, AK7316AF_Ioctl,
	AK7316AF_Release, AK7316AF_GetFileName, NULL},
	{1, AFDRV_DW9814AF, DW9814AF_SetI2Cclient, DW9814AF_Ioctl,
	 DW9814AF_Release, DW9814AF_GetFileName, NULL},
	{1, AFDRV_DW9839AF, DW9839AF_SetI2Cclient, DW9839AF_Ioctl,
	 DW9839AF_Release, DW9839AF_GetFileName, NULL},
	{1, AFDRV_FP5510E2AF, FP5510E2AF_SetI2Cclient, FP5510E2AF_Ioctl,
	 FP5510E2AF_Release, FP5510E2AF_GetFileName, NULL},
	{1, AFDRV_DW9718AF, DW9718AF_SetI2Cclient, DW9718AF_Ioctl,
	 DW9718AF_Release, DW9718AF_GetFileName, NULL},
	{1, AFDRV_GT9764AF, GT9764AF_SetI2Cclient, GT9764AF_Ioctl,
	GT9764AF_Release, GT9764AF_GetFileName, NULL},
//#ifdef SUPPORT_GT9768AF
	{1, AFDRV_GT9768AF, GT9768AF_SetI2Cclient, GT9768AF_Ioctl,
	GT9768AF_Release, GT9768AF_GetFileName, NULL},
//#endif
	{1, AFDRV_LC898212AF, LC898212AF_SetI2Cclient, LC898212AF_Ioctl,
	 LC898212AF_Release, LC898212AF_GetFileName, NULL},
	{1, AFDRV_LC898214AF, LC898214AF_SetI2Cclient, LC898214AF_Ioctl,
	 LC898214AF_Release, LC898214AF_GetFileName, NULL},
	{1, AFDRV_LC898217AF, LC898217AF_SetI2Cclient, LC898217AF_Ioctl,
	 LC898217AF_Release, LC898217AF_GetFileName, NULL},
	{1, AFDRV_LC898217AFA, LC898217AFA_SetI2Cclient, LC898217AFA_Ioctl,
	 LC898217AFA_Release, LC898217AFA_GetFileName, NULL},
	{1, AFDRV_LC898217AFB, LC898217AFB_SetI2Cclient, LC898217AFB_Ioctl,
	 LC898217AFB_Release, LC898217AFB_GetFileName, NULL},
	{1, AFDRV_LC898217AFC, LC898217AFC_SetI2Cclient, LC898217AFC_Ioctl,
	 LC898217AFC_Release, LC898217AFC_GetFileName, NULL},
	{1, AFDRV_LC898229AF, LC898229AF_SetI2Cclient, LC898229AF_Ioctl,
	 LC898229AF_Release, LC898229AF_GetFileName, NULL},
	{1, AFDRV_LC898122AF, LC898122AF_SetI2Cclient, LC898122AF_Ioctl,
	 LC898122AF_Release, LC898122AF_GetFileName, NULL},
	{1, AFDRV_WV511AAF, WV511AAF_SetI2Cclient, WV511AAF_Ioctl,
	 WV511AAF_Release, WV511AAF_GetFileName, NULL},
	{1, AFDRV_JD5516WE4, JD5516WE4_SetI2Cclient, JD5516WE4_Ioctl,
	 JD5516WE4_Release, JD5516WE4_GetFileName, NULL},
#endif
};

static struct stAF_DrvList *g_pstAF_CurDrv;

static spinlock_t g_AF_SpinLock;

static int g_s4AF_Opened;

static struct i2c_client *g_pstAF_I2Cclient;

static dev_t g_AF_devno;
static struct cdev *g_pAF_CharDrv;
static struct class *actuator_class;
static struct device *lens_device;

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
#define AF_PINCTRL_PIN_HWEN 0
#define AF_PINCTRL_PINSTATE_LOW 0
#define AF_PINCTRL_PINSTATE_HIGH 1

#if defined(CONFIG_MACH_MT6885)
#define AF_PINCTRL_STATE_HWEN_HIGH     "camafen_high"
#define AF_PINCTRL_STATE_HWEN_LOW      "camafen_low"
#else
#define AF_PINCTRL_STATE_HWEN_HIGH     "cam0_ldo_vcamaf_1"
#define AF_PINCTRL_STATE_HWEN_LOW      "cam0_ldo_vcamaf_0"
#endif
#define CAMAF_PMIC     "camaf_m1_pmic"
#define CAMAF_GPIO_ON  "cam0_ldo_vcamaf_1"
#define CAMAF_GPIO_OFF "cam0_ldo_vcamaf_0"

static struct pinctrl *af_pinctrl;
static struct pinctrl_state *af_hwen_high;
static struct pinctrl_state *af_hwen_low;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#if defined(CONFIG_MACH_MT6853) || defined(CONFIG_MACH_MT6873) || defined(CONFIG_MACH_MT6779)
extern struct regulator *regulator_get_regVCAMAF(void);
#endif /* CONFIG_MACH_MT6853 */
#endif /* OPLUS_FEATURE_CAMERA_COMMON */
static int af_pinctrl_init(struct device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	if (af_pinctrl == NULL) {
		af_pinctrl = devm_pinctrl_get(pdev);
		/* LOG_INF("pinctrl(%x) +", af_pinctrl); */
		if (IS_ERR(af_pinctrl)) {
			LOG_INF("Failed to get af pinctrl.\n");
			ret = PTR_ERR(af_pinctrl);
			af_pinctrl = NULL;
			return ret;
		}

		/* AF HWEN pin initialization */
		af_hwen_high = pinctrl_lookup_state(
				af_pinctrl, AF_PINCTRL_STATE_HWEN_HIGH);
		if (IS_ERR(af_hwen_high)) {
			LOG_INF("Failed to init (%s)\n",
				AF_PINCTRL_STATE_HWEN_HIGH);
			ret = PTR_ERR(af_hwen_high);
			af_hwen_high = NULL;
		}
		af_hwen_low = pinctrl_lookup_state(
			af_pinctrl, AF_PINCTRL_STATE_HWEN_LOW);
		if (IS_ERR(af_hwen_low)) {
			LOG_INF("Failed to init (%s)\n",
				AF_PINCTRL_STATE_HWEN_LOW);
			ret = PTR_ERR(af_hwen_low);
			af_hwen_low = NULL;
		}
	}
	LOG_INF("-");
	return ret;
}


#ifdef OPLUS_FEATURE_CAMERA_COMMON
#if !defined(CONFIG_MACH_MT6765)
extern int pmic_ldo_set_voltage_mv(unsigned int ldo_num, int set_mv);
extern int pmic_ldo_set_disable(unsigned int ldo_num);
extern int pmic_ldo_get_type(void);
extern int fan53870_cam_ldo_set_voltage(int LDO_NUM, int set_mv);
extern int fan53870_cam_ldo_disable(int LDO_NUM);
#endif
void Other_AFRegulatorCtrl(int Stage);
//temp code for Project differentiation
#endif

/* PMIC */
#if !defined(CONFIG_MTK_LEGACY)
static struct regulator *regVCAMAF;
static int g_regVCAMAFEn;
static int fail_update_cnt = 0, success_check_cnt = 0;
void Oplus_Lens_PIDparam_update(struct stAF_MotorName* stMotorName)
{
	if (fail_update_cnt < 3 && (is_project(20181) || is_project(20355))
	&& !(strcmp(stMotorName->uMotorName,
		AFDRV_AK7375CAF))) {
		if (AK7375CAF_checkPIDparam() && !success_check_cnt) {
			if (AK7375CAF_updatePIDparam()) {
				fail_update_cnt++;
				return;
			}
			//reset power to enable new PID param
			#if !defined(CONFIG_MACH_MT6765)
			pmic_ldo_set_disable(7);
			pmic_ldo_set_voltage_mv(7, 2800);
			#endif
			msleep(20);
			if (AK7375CAF_checkPIDparam()) {
				fail_update_cnt++;
				return;
			}
			success_check_cnt++;
		}
	}
}
void AFRegulatorCtrl(int Stage)
{
#if !defined(CONFIG_MACH_MT6765)
    int Status = -1;
#endif
#ifdef OPLUS_FEATURE_CAMERA_COMMON

    #if defined(CONFIG_MACH_MT6779) || defined(CONFIG_MACH_MT6765)
        Other_AFRegulatorCtrl(Stage);
        return;
    #endif /* CONFIG_MACH_MT6853 */

    if (is_project(0x2167A) || is_project(0x2167B) || is_project(0x2167C) || is_project(0x2167D) || is_project(0x226AD) || is_project(0x226AE) || is_project(0x226AF) || is_project(0x226B0) || is_project(0x226BC) || is_project(0x226BD) || is_project(0x226BE) || is_project(0x226BF)) {
        Other_AFRegulatorCtrl(Stage);
        return;
    }
    if (is_project(0x216AF) || is_project(0x216B0) || is_project(0x216B1)){
        Other_AFRegulatorCtrl(Stage);
        return;
    }
    if (is_project(20761) || is_project(20762) || is_project(20764) || is_project(20767)){
        Other_AFRegulatorCtrl(Stage);
        return;
    }

	if(is_project(0x2169E) || is_project(0x2169F) || is_project(0x216C9) || is_project(0x216CA) || is_project(23602)
        || is_project(22603) || is_project(22604) || is_project(22609) || is_project(22631) || is_project(22632)) {
            Other_AFRegulatorCtrl(Stage);
            return;
    }

	if(is_project(22083) || is_project(22291) || is_project(22292) || is_project(22084)) {
            //Other_AFRegulatorCtrl(Stage);
            return;
    }

	if(is_project(23053) || is_project(23054) || is_project(23253) || is_project(23055)) {
            //Other_AFRegulatorCtrl(Stage);
            return;
    }

	if(is_project(22087) || is_project(22088) || is_project(22331) || is_project(22332) || is_project(22333) || is_project(22334) || is_project(22869)) {
            return;
    }

	if (is_project(22693) || is_project(22694) || is_project(22612) || is_project(0x226B1)) {
		Other_AFRegulatorCtrl(Stage);
		return;
	}

	if (is_project(22629) || is_project(22710) || is_project(22711) || is_project(22633) || is_project(22712) || is_project(22713)) {
		Other_AFRegulatorCtrl(Stage);
		return;
	}
	if (is_project(23613) || is_project(23686) || is_project(23687)) {
		Other_AFRegulatorCtrl(Stage);
		return;
	}
	if (is_project(21711) || is_project(21712)) {
		return;
	}

    if (is_project(19165)) {
            return;
    }

	if(is_project(19131) || (is_project(19420)) || is_project(19132)) {
            Other_AFRegulatorCtrl(Stage);
            return;
    }

	if(is_project(20075) || is_project(20076)) {
            Other_AFRegulatorCtrl(Stage);
            return;
    }
	if(is_project(20609) || is_project(0x2060B) || is_project(0x2060A)
    || is_project(0x206FF) || is_project(20796) || is_project(0x2070C) || is_project(20795) || is_project(21747) || is_project(21748)) {
		// Other_AFRegulatorCtrl(Stage);
		return;
	}
    if (is_project(20151) || is_project(20301) || is_project(20302) ) {
            Other_AFRegulatorCtrl(Stage);
            return;
    }

    if (is_project(20391) || is_project(20392)) {
        Other_AFRegulatorCtrl(Stage);
        return;
    }
    if (is_project(20001) || is_project(20002) || is_project(20003) || is_project(20200)
        || is_project(20041) || is_project(20042) || is_project(20043)) {
            Other_AFRegulatorCtrl(Stage);
            return;
    }
    if (is_project(20095) || is_project(20610) || is_project(20611) || is_project(20613) || is_project(20680) || is_project(20686) ) {
            return;
    }
    if (is_project(21101) || is_project(21102) || is_project(21235)
        || is_project(21236) || is_project(21831)) {
        return;
    }
	if (is_project(20015) || is_project(20016) || is_project(21037)
		|| is_project(20108) || is_project(20109) || is_project(20307)
		|| is_project(20730) || is_project(20731) || is_project(20732)
		|| is_project(20013)) {
		Other_AFRegulatorCtrl(Stage);
		return;
	}
	if( is_project(21625) || is_project(0x216A0)|| is_project(0x22610)|| is_project(0x22705)|| is_project(0x22706)
        || is_project(0x216A1)) {
		//Other_AFRegulatorCtrl(Stage);
		return;
	}


    if(is_project(20631) || is_project(20630) || is_project(20633)) {
            return;
    }

	if(is_project(21041) || is_project(21042)) {
		//21041 21042 chang to control ldo in imgsensor
		//Other_AFRegulatorCtrl(Stage);
		LOG_INF("control ldo in imgsensor\n");
		return;
	}

    if (is_project(0x2163B) || is_project(0x2163C) || is_project(21639) || is_project(0x2163D) || is_project(0x216CD) || is_project(0x216CE)
        || is_project(22603) || is_project(22604) || is_project(22609) || is_project(22631) || is_project(22632) || is_project(23602)
	    || is_project(22083) || is_project(22291) || is_project(22292) || is_project(22084)
	    || is_project(23053) || is_project(23054) || is_project(23253) || is_project(23055)) {
        return;
    }
    if(is_project(21690) || is_project(21691) || is_project(21692)) {
            Other_AFRegulatorCtrl(Stage);
            return;
    }

	if( is_project(0x210A0)) {
		LOG_INF("control ldo in imgsensor\n");
		//Other_AFRegulatorCtrl(Stage);
		return;
	}

	if (is_project(20638) || is_project(20639) || is_project(0x206B7) ) {
            return;
    }
    if(is_project(22351) || is_project(22352) || is_project(22353) || is_project(22361) || is_project(22362) || is_project(22363)
        ||is_project(22364) || is_project(22365) || is_project(22366) || is_project(22367) || is_project(22368)) {
        //aladdin&atom chang to control ldo in imgsensor
        //Other_AFRegulatorCtrl(Stage);
        LOG_INF("control ldo in imgsensor\n");
        return;
    }
	LOG_INF("AFIOC_S_SETPOWERCTRL Stage %d\n", Stage);
#if !defined(CONFIG_MACH_MT6765)
	if (Stage == 0) {
		LOG_INF("AFRegulatorCtrl(%d) init\n", Stage);
	} else if (Stage == 1) {
		if (g_regVCAMAFEn == 0) {
			if (is_project(20615) || is_project(20817)
                            || is_project(20827) || is_project(20831)
                            || is_project(21881) || is_project(21882)
                            || is_project(20662) || is_project(21609)
                            || is_project(20619) || is_project(21651)) {
				Status = fan53870_cam_ldo_set_voltage(6, 2800);
			} else if (is_project(21061)) {
				Status = fan53870_cam_ldo_set_voltage(6, 2900);
				LOG_INF("AFRegulator power on status is %d\n", Status);
			} else if (is_project(20601) || is_project(20602) || is_project(20660)) {
				pmic_ldo_get_type();
				Status = pmic_ldo_set_voltage_mv(6, 2800);
			} else if(is_project(21015) || is_project(21217)
				|| is_project(21016) || is_project(21218)) {
				LOG_INF("AFRegulator power on nothing\n");
			} else if (is_project(21127) || is_project(21305)) {
				LOG_INF("AFRegulator power on nothing\n");
			} else {
				pmic_ldo_get_type();
				Status = pmic_ldo_set_voltage_mv(7, 2800);
			}
			if (Status < 0) {
				LOG_INF("pmic_camaf set 2800 fail\n");
			} else {
				LOG_INF("pmic_camaf set %d\n", Status);
				g_regVCAMAFEn = 1;
				if (is_project(23231)) {
					usleep_range(10000, 10500);
				} else {
					usleep_range(20000, 20500);
				}
			}
		} else {
			LOG_INF("pmic_camaf already set!\n");
		}
	} else {
		if (g_regVCAMAFEn == 1) {
			if (is_project(20615) || is_project(20817) || is_project(21061)
                            || is_project(20827) || is_project(20831)
                            || is_project(21881) || is_project(21882)
                            || is_project(20619) || is_project(21651)
                            || is_project(20662) || is_project(21609)) {
				Status = fan53870_cam_ldo_disable(6);
			} else if (is_project(20601) || is_project(20602) || is_project(20660)){
				pmic_ldo_get_type();
				Status = pmic_ldo_set_disable(6);
			} else if(is_project(21015) || is_project(21217)
				|| is_project(21016) || is_project(21218)) {
				LOG_INF("AFRegulator power off nothing\n");
			} else if (is_project(21127) || is_project(21305)) {
				Status = fan53870_cam_ldo_disable(7);
			} else {
                                pmic_ldo_get_type();
				Status = pmic_ldo_set_disable(7);
			}
			if (Status < 0) {
				LOG_INF("Camera Power disable error\n");
			} else {
				g_regVCAMAFEn = 0;
			}
		} else {
			LOG_INF("Camera Power already disable\n");
		}
	}
#endif
#else
	LOG_INF("AFIOC_S_SETPOWERCTRL regulator_put %p, stage: %d\n", regVCAMAF, Stage);

	if (Stage == 0) {
		if (regVCAMAF == NULL) {
			struct device_node *node, *kd_node;

			/* check if customer camera node defined */
			node = of_find_compatible_node(
				NULL, NULL, "mediatek,CAMERA_MAIN_AF");

			if (node) {
				kd_node = lens_device->of_node;
				lens_device->of_node = node;

				#if defined(CONFIG_MACH_MT6765)
				regVCAMAF =
					regulator_get(lens_device, "vldo28");
				#elif defined(CONFIG_MACH_MT6768)
				regVCAMAF =
					regulator_get(lens_device, "vldo28");
				#elif defined(CONFIG_MACH_MT6771)
				regVCAMAF =
					regulator_get(lens_device, "vldo28");
				#elif defined(CONFIG_MACH_MT6853)
				if (strncmp(CONFIG_ARCH_MTK_PROJECT,
					"k6853v1_64_6360_alpha", 20) == 0) {
					regVCAMAF =
					regulator_get(lens_device, "vmch");
				} else {
					regVCAMAF =
					regulator_get(lens_device, "vcamio");
				}
				#elif defined(CONFIG_MACH_MT6873)
				if (strncmp(CONFIG_ARCH_MTK_PROJECT,
					"k6873v1_64_alpha", 16) == 0) {
					regVCAMAF =
					regulator_get(lens_device, "vmch");
				} else {
					regVCAMAF =
					regulator_get(lens_device, "vcamio");
				}
				#elif defined(CONFIG_MACH_MT6877) || defined(CONFIG_MACH_MT6781)
				regVCAMAF =
					regulator_get(lens_device, "rt5133-ldo3");
				#elif defined(CONFIG_MACH_MT6885) || defined(CONFIG_MACH_MT6893)
				if (strncmp(CONFIG_ARCH_MTK_PROJECT,
					"k6885v1_64_alpha", 16) == 0) {
					regVCAMAF =
					regulator_get(lens_device, "vmc");
				} else {
					regVCAMAF =
					regulator_get(lens_device, "vcamio");
				}
				#else
				regVCAMAF =
					regulator_get(lens_device, "vcamaf");
				#endif

				LOG_INF("[Init] regulator_get %p\n", regVCAMAF);

				lens_device->of_node = kd_node;
			}
		}
	} else if (Stage == 1) {
		if (regVCAMAF != NULL && g_regVCAMAFEn == 0) {
			int Status = regulator_is_enabled(regVCAMAF);

			LOG_INF("regulator_is_enabled %d\n", Status);

			if (!Status) {
				Status = regulator_set_voltage(
					regVCAMAF, 2800000, 2800000);

				LOG_INF("regulator_set_voltage %d\n", Status);

				if (Status != 0)
					LOG_INF("regulator_set_voltage fail\n");

				Status = regulator_enable(regVCAMAF);
				LOG_INF("regulator_enable %d\n", Status);

				if (Status != 0)
					LOG_INF("regulator_enable fail\n");

				g_regVCAMAFEn = 1;
				usleep_range(5000, 5500);
			} else {
				LOG_INF("AF Power on\n");
			}
		}
	} else {
		if (regVCAMAF != NULL && g_regVCAMAFEn == 1) {
			int Status = regulator_is_enabled(regVCAMAF);

			LOG_INF("regulator_is_enabled %d\n", Status);

			if (Status) {
				LOG_INF("Camera Power enable\n");

				Status = regulator_disable(regVCAMAF);
				LOG_INF("regulator_disable %d\n", Status);
				if (Status != 0)
					LOG_INF("Fail to regulator_disable\n");
			}
			/* regulator_put(regVCAMAF); */
			LOG_INF("AFIOC_S_SETPOWERCTRL regulator_put %p\n",
				regVCAMAF);
			/* regVCAMAF = NULL; */
			g_regVCAMAFEn = 0;
		}
	}
#endif
}
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
void Other_AFRegulatorCtrl(int Stage)
{
	LOG_INF("AFIOC_S_SETPOWERCTRL regulator_put %p, stage: %d\n", regVCAMAF, Stage);

	if (Stage == 0) {
		if (regVCAMAF == NULL) {
			#ifndef OPLUS_FEATURE_CAMERA_COMMON
			struct device_node *node, *kd_node;

			/* check if customer camera node defined */
			node = of_find_compatible_node(
				NULL, NULL, "mediatek,CAMERA_MAIN_AF");

			if (node) {
				kd_node = lens_device->of_node;
				lens_device->of_node = node;

				#if defined(CONFIG_MACH_MT6765)
				regVCAMAF =
					regulator_get(lens_device, "vldo28");
				#elif defined(CONFIG_MACH_MT6768)
				regVCAMAF =
					regulator_get(lens_device, "vldo28");
				#elif defined(CONFIG_MACH_MT6771)
				regVCAMAF =
					regulator_get(lens_device, "vldo28");
				#elif defined(CONFIG_MACH_MT6853)
				if (strncmp(CONFIG_ARCH_MTK_PROJECT,
					"k6853v1_64_6360_alpha", 20) == 0) {
					regVCAMAF =
					regulator_get(lens_device, "vmch");
				} else {
					regVCAMAF =
					regulator_get(lens_device, "vcamio");
				}
				#elif defined(CONFIG_MACH_MT6873)
				if (strncmp(CONFIG_ARCH_MTK_PROJECT,
					"k6873v1_64_alpha", 16) == 0) {
					regVCAMAF =
					regulator_get(lens_device, "vmch");
				} else {
					regVCAMAF =
					regulator_get(lens_device, "vcamio");
				}
				#elif defined(CONFIG_MACH_MT6885) || defined(CONFIG_MACH_MT6893)
				if (strncmp(CONFIG_ARCH_MTK_PROJECT,
					"k6885v1_64_alpha", 16) == 0) {
					regVCAMAF =
					regulator_get(lens_device, "vmc");
				} else {
					regVCAMAF =
					regulator_get(lens_device, "vcamio");
				}
				#else
				regVCAMAF =
					regulator_get(lens_device, "vcamaf");
				#endif

				LOG_INF("[Init] regulator_get %p\n", regVCAMAF);

				lens_device->of_node = kd_node;
			}
                        #else /* OPLUS_FEATURE_CAMERA_COMMON */
                            #if defined(CONFIG_MACH_MT6853) || defined(CONFIG_MACH_MT6873) || defined(CONFIG_MACH_MT6779)
                            if (is_project(20075) || is_project(20076) || is_project(20041) || is_project(20042) || is_project(20043)) {
                                LOG_INF("project 20075,20076.\n");
                                regVCAMAF = regulator_get_regVCAMAF();
                            } else if (is_project(20391)||is_project(20392) || is_project(20001)
                                || is_project(20002) || is_project(20003) || is_project(20200) ) {
                                LOG_INF("project 20391/20392.\n");
                                regVCAMAF = regulator_get_regVCAMAF();
                            } else if (is_project(20151) || (is_project(20301)) || (is_project(20302))) {
                                LOG_INF("project 20151,20301,20302.\n");
                                regVCAMAF = regulator_get_regVCAMAF();
                            } else if (is_project(19131) || (is_project(19420)) || is_project(19132)) {
                                LOG_INF("project 19131,19420,19132.\n");
                                regVCAMAF = regulator_get_regVCAMAF();
                            } else {
                                regVCAMAF = regulator_get_regVCAMAF();
                            }
                            #else /* CONFIG_MACH_MT6853 */
                            if (is_project(0x2169E) || is_project(0x2169F) || is_project(0x216C9) || is_project(0x216CA)) {
                                regVCAMAF = regulator_get(lens_device, "VFP");
                            } else if (is_project(22693) || is_project(22694) || is_project(22612) || is_project(0x226B1)) {
                                regVCAMAF = regulator_get(lens_device, "VFP");
                            } else if (is_project(22629) || is_project(22710) || is_project(22711)
                                || is_project(23613) || is_project(23686) || is_project(23687)
                                || is_project(22633) || is_project(22712) || is_project(22713)) {
                                regVCAMAF = regulator_get(lens_device, "VFP");
                            } else if (is_project(22603) || is_project(22604) || is_project(22609)) {
                                LOG_INF("project BLADE,\n");
                                regVCAMAF = regulator_get(lens_device, "vtp");
                            } else if (is_project(22083) || is_project(22291) || is_project(22292) || is_project(22084)) {
                                LOG_INF("project LIJING,\n");
                            } else if (is_project(23053) || is_project(23054) || is_project(23253) || is_project(23055)) {
                                LOG_INF("project LIJINGA,\n");
                                //regVCAMAF = regulator_get(lens_device, "vtp");
                            } else if (is_project(22087) || is_project(22088) || is_project(22331) || is_project(22332) || is_project(22333) || is_project(22334) || is_project(22869)) {
                                LOG_INF("project fanli,\n");
                                //regVCAMAF = regulator_get(lens_device, "vtp");
                            } else if (is_project(20015) || is_project(20016) || is_project(21037)
                                || is_project(20108) || is_project(20109) || is_project(20307) || is_project(20013)) {
                                LOG_INF("project 20015,21037.\n");
                                regVCAMAF = regulator_get(lens_device, "vtp");
                            } else if (is_project(20761) || is_project(20762) || is_project(20764)
                                || is_project(20766) || is_project(20767)) {
                                LOG_INF("project EVEN.\n");
                                regVCAMAF = regulator_get(lens_device, "vldo28");
                            } else if (is_project(20682)) {
                                LOG_INF("project SALA.\n");
                                regVCAMAF = regulator_get(lens_device, "vldo28");
                            } else if (is_project(0x2167A) || is_project(0x2167B) || is_project(0x2167C)
                                || is_project(0x2167D)) {
                                LOG_INF("project EVEN_B.\n");
                                regVCAMAF = regulator_get(lens_device, "vldo28"); 
                            }else if (is_project(0x226AD) || is_project(0x226AE) || is_project(0x226AF) || is_project(0x226B0) || is_project(0x226BC) || is_project(0x226BD) || is_project(0x226BE) || is_project(0x226BF) ) {
                                LOG_INF("project MIAMI.\n");
                                regVCAMAF = regulator_get(lens_device, "vldo28");
                            } else if(is_project(0x216AF) || is_project(0x216B0) || is_project(0x216B1)) {
                                LOG_INF("project EVEN_C.\n");
                                regVCAMAF = regulator_get(lens_device, "vldo28");
                            } else
                                regVCAMAF = regulator_get(lens_device, "vcamaf");
                            LOG_INF("[Init] regulator_get %p\n", regVCAMAF);
                            #endif /* CONFIG_MACH_MT6853 */
                        if (IS_ERR(regVCAMAF)) {
                            pr_err("get main af regulator fail");
                        }
                        #endif /* OPLUS_FEATURE_CAMERA_COMMON */
		}
	} else if (Stage == 1) {
		if (regVCAMAF != NULL && g_regVCAMAFEn == 0) {
			int Status = regulator_is_enabled(regVCAMAF);

			LOG_INF("regulator_is_enabled %d\n", Status);

			if (!Status) {
				Status = regulator_set_voltage(
					regVCAMAF, 2800000, 2800000);

				LOG_INF("regulator_set_voltage %d\n", Status);

				if (Status != 0)
					LOG_INF("regulator_set_voltage fail\n");

				Status = regulator_enable(regVCAMAF);
				LOG_INF("regulator_enable %d\n", Status);

				if (Status != 0)
					LOG_INF("regulator_enable fail\n");

				g_regVCAMAFEn = 1;
				usleep_range(5000, 5500);
			} else {
				LOG_INF("AF Power on\n");
			}
		}
	} else {
		if (regVCAMAF != NULL && g_regVCAMAFEn == 1) {
			int Status = regulator_is_enabled(regVCAMAF);

			LOG_INF("regulator_is_enabled %d\n", Status);

			if (Status) {
				LOG_INF("Camera Power enable\n");

				Status = regulator_disable(regVCAMAF);
				LOG_INF("regulator_disable %d\n", Status);
				if (Status != 0)
					LOG_INF("Fail to regulator_disable\n");
			}
			/* regulator_put(regVCAMAF); */
			LOG_INF("AFIOC_S_SETPOWERCTRL regulator_put %p\n",
				regVCAMAF);
			/* regVCAMAF = NULL; */
			g_regVCAMAFEn = 0;
		}
	}
}
#endif

#ifdef CONFIG_MACH_MT6765
static int DrvPwrDn1 = 1;
static int DrvPwrDn2 = 1;
static int DrvPwrDn3 = 1;
#endif

void AF_PowerDown(void)
{
	if (g_pstAF_I2Cclient != NULL) {
#if defined(CONFIG_MACH_MT6771) ||              \
	defined(CONFIG_MACH_MT6775)
		LC898217AF_PowerDown(g_pstAF_I2Cclient, &g_s4AF_Opened);
#endif

#ifdef CONFIG_MTK_LENS_AK7371AF_SUPPORT
		AK7371AF_PowerDown(g_pstAF_I2Cclient, &g_s4AF_Opened);
#endif

#ifdef CONFIG_MACH_MT6758
		AK7371AF_PowerDown(g_pstAF_I2Cclient, &g_s4AF_Opened);

		BU63169AF_PowerDown(g_pstAF_I2Cclient, &g_s4AF_Opened);
#endif

#ifdef CONFIG_MACH_MT6765
		int Ret1 = 0, Ret2 = 0, Ret3 = 0;

		if (DrvPwrDn1) {
			Ret1 = LC898217AF_PowerDown(g_pstAF_I2Cclient,
						&g_s4AF_Opened);
		}

		if (DrvPwrDn2) {
			Ret2 = DW9718SAF_PowerDown(g_pstAF_I2Cclient,
						&g_s4AF_Opened);
		}

		if (DrvPwrDn3) {
			Ret3 = bu64748af_PowerDown_Main(g_pstAF_I2Cclient,
						&g_s4AF_Opened);
		}

		if (DrvPwrDn1 && DrvPwrDn2 && DrvPwrDn3) {
			if (Ret1 < 0)
				DrvPwrDn1 = 0;
			if (Ret2 < 0)
				DrvPwrDn2 = 0;
			if (Ret3 < 0)
				DrvPwrDn3 = 0;

		}
			LOG_INF("%d/%d , %d/%d, %d/%d\n", Ret1, DrvPwrDn1,
				Ret2, DrvPwrDn2, Ret3, DrvPwrDn3);
#endif

#ifdef CONFIG_MACH_MT6761
		DW9718SAF_PowerDown(g_pstAF_I2Cclient, &g_s4AF_Opened);
#endif
	}
	MAIN2AF_PowerDown();
}
EXPORT_SYMBOL(AF_PowerDown);

static long AF_SetMotorName(__user struct stAF_MotorName *pstMotorName)
{
	long i4RetValue = -1;
	int i;
	struct stAF_MotorName stMotorName;

	if (copy_from_user(&stMotorName, pstMotorName,
			   sizeof(struct stAF_MotorName)))
		LOG_INF("copy to user failed when getting motor information\n");

	stMotorName.uMotorName[sizeof(stMotorName.uMotorName) - 1] = '\0';

	for (i = 0; i < MAX_NUM_OF_LENS; i++) {
		if (g_stAF_DrvList[i].uEnable != 1)
			break;

		LOG_INF("Search Motor Name : %s\n", g_stAF_DrvList[i].uDrvName);
		if (strcmp(stMotorName.uMotorName,
			   g_stAF_DrvList[i].uDrvName) == 0) {
			LOG_INF("Motor Name : %s\n", stMotorName.uMotorName);
			g_pstAF_CurDrv = &g_stAF_DrvList[i];
			i4RetValue = g_pstAF_CurDrv->pAF_SetI2Cclient(
				g_pstAF_I2Cclient, &g_AF_SpinLock,
				&g_s4AF_Opened);
			Oplus_Lens_PIDparam_update(&stMotorName);
			break;
		}
	}
	return i4RetValue;
}


static long AF_ControlParam(unsigned long a_u4Param)
{
	long i4RetValue = -1;
	__user struct stAF_CtrlCmd *pCtrlCmd =
			(__user struct stAF_CtrlCmd *)a_u4Param;
	struct stAF_CtrlCmd CtrlCmd;

	if (copy_from_user(&CtrlCmd, pCtrlCmd, sizeof(struct stAF_CtrlCmd)))
		LOG_INF("copy to user failed\n");

	switch (CtrlCmd.i8CmdID) {
	case CONVERT_CCU_TIMESTAMP:
		{
#if defined(CONFIG_MACH_MT6779)
		long long monotonicTime = 0;
		long long hwTickCnt     = 0;

		hwTickCnt     = CtrlCmd.i8Param[0];
		monotonicTime = archcounter_timesync_to_monotonic(hwTickCnt);
		/* do_div(monotonicTime, 1000); */ /* ns to us */
		CtrlCmd.i8Param[0] = monotonicTime;

		hwTickCnt     = CtrlCmd.i8Param[1];
		monotonicTime = archcounter_timesync_to_monotonic(hwTickCnt);
		/* do_div(monotonicTime, 1000); */ /* ns to us */
		CtrlCmd.i8Param[1] = monotonicTime;

		if (copy_to_user(pCtrlCmd, &CtrlCmd,
			sizeof(struct stAF_CtrlCmd)))
			LOG_INF("copy to user failed\n");
#endif
		}
		i4RetValue = 1;
		break;
	default:
		i4RetValue = -1;
		break;
	}

	return i4RetValue;
}

/* ////////////////////////////////////////////////////////////// */
static long AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
		     unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_S_SETDRVNAME:
		i4RetValue = AF_SetMotorName(
			(__user struct stAF_MotorName *)(a_u4Param));
		break;

	case AFIOC_G_GETDRVNAME:
		{
	/* Set Driver Name */
	int i;
	struct stAF_MotorName stMotorName;
	struct stAF_DrvList *pstAF_CurDrv = NULL;
	__user struct stAF_MotorName *pstMotorName =
			(__user struct stAF_MotorName *)a_u4Param;

	if (copy_from_user(&stMotorName, pstMotorName,
			   sizeof(struct stAF_MotorName)))
		LOG_INF("copy to user failed when getting motor information\n");

	stMotorName.uMotorName[sizeof(stMotorName.uMotorName) - 1] = '\0';

	LOG_INF("GETDRVNAME : set driver name(%s)\n", stMotorName.uMotorName);

	for (i = 0; i < MAX_NUM_OF_LENS; i++) {
		if (g_stAF_DrvList[i].uEnable != 1)
			break;

		LOG_INF("Search Motor Name : %s\n", g_stAF_DrvList[i].uDrvName);
		if (strcmp(stMotorName.uMotorName,
			   g_stAF_DrvList[i].uDrvName) == 0) {
			LOG_INF("Motor Name : %s\n", stMotorName.uMotorName);
			pstAF_CurDrv = &g_stAF_DrvList[i];
			break;
		}
	}

	/* Get File Name */
	if (pstAF_CurDrv) {
		if (pstAF_CurDrv->pAF_GetFileName) {
			__user struct stAF_MotorName *pstMotorName =
			(__user struct stAF_MotorName *)a_u4Param;
			struct stAF_MotorName MotorFileName;

			pstAF_CurDrv->pAF_GetFileName(
					MotorFileName.uMotorName);
			i4RetValue = 1;
			LOG_INF("GETDRVNAME : get file name(%s)\n",
				MotorFileName.uMotorName);
			if (copy_to_user(
				    pstMotorName, &MotorFileName,
				    sizeof(struct stAF_MotorName)))
				LOG_INF("copy to user failed\n");
		}
	}
		}
		break;

	case AFIOC_S_SETDRVINIT:
		spin_lock(&g_AF_SpinLock);
		g_s4AF_Opened = 1;
		spin_unlock(&g_AF_SpinLock);
		break;

	case AFIOC_S_SETPOWERDOWN:
		AF_PowerDown();
		i4RetValue = 1;
		break;

#if !defined(CONFIG_MTK_LEGACY)
	case AFIOC_S_SETPOWERCTRL:
		AFRegulatorCtrl(0);

		if (a_u4Param > 0)
			AFRegulatorCtrl(1);
		break;
#endif

	case AFIOC_G_OISPOSINFO:
		if (g_pstAF_CurDrv) {
			if (g_pstAF_CurDrv->pAF_OisGetHallPos) {
				__user struct stAF_OisPosInfo *pstOisPosInfo =
					(__user struct stAF_OisPosInfo *)
						a_u4Param;

				mutex_lock(&ois_mutex);

				if (copy_to_user(
					    pstOisPosInfo, &OisPosInfo,
					    sizeof(struct stAF_OisPosInfo)))
					LOG_INF("copy to user failed\n");

				g_OisPosIdx = 0;
				g_GetOisInfoCnt = 100;
				memset(&OisPosInfo, 0, sizeof(OisPosInfo));
				mutex_unlock(&ois_mutex);

				if (g_EnableTimer == 0) {
					/* Start Timer */
					hrtimer_start(&ois_timer,
						      ktime_set(0, 50000000),
						      HRTIMER_MODE_REL);
					g_EnableTimer = 1;
				}
			}
		}
		break;

	case AFIOC_X_CTRLPARA:
		if (AF_ControlParam(a_u4Param) <= 0) {
			if (g_pstAF_CurDrv)
				i4RetValue = g_pstAF_CurDrv->pAF_Ioctl(
					a_pstFile, a_u4Command, a_u4Param);
		}
		break;

	default:
		if (g_pstAF_CurDrv) {
			if (g_pstAF_CurDrv->pAF_Ioctl)
				i4RetValue = g_pstAF_CurDrv->pAF_Ioctl(
					a_pstFile, a_u4Command, a_u4Param);
		}
		break;
	}

	return i4RetValue;
}

#ifdef CONFIG_COMPAT
static long AF_Ioctl_Compat(struct file *a_pstFile, unsigned int a_u4Command,
			    unsigned long a_u4Param)
{
	long i4RetValue = 0;

	i4RetValue = AF_Ioctl(a_pstFile, a_u4Command,
			      (unsigned long)compat_ptr(a_u4Param));

	return i4RetValue;
}
#endif

/* Main jobs: */
/* 1.check for device-specified errors, device not ready. */
/* 2.Initialize the device if it is opened for the first time. */
/* 3.Update f_op pointer. */
/* 4.Fill data structures into private_data */
/* CAM_RESET */
#if defined(CONFIG_MACH_MT6765) || defined(CONFIG_MACH_MT6779)
static int AF_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	spin_lock(&g_AF_SpinLock);
	if (g_s4AF_Opened) {
		/* spin_unlock(&g_AF_SpinLock); */
		LOG_INF("The device is opened (%d)\n", g_s4AF_Opened);
		/* return -EBUSY; */
	}
	g_s4AF_Opened = 1;
	spin_unlock(&g_AF_SpinLock);

#if !defined(CONFIG_MTK_LEGACY)
	AFRegulatorCtrl(0);  //init regulator first
	AFRegulatorCtrl(1);
#endif

	LOG_INF("End\n");

	return 0;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
static int AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (g_pstAF_CurDrv) {
		g_pstAF_CurDrv->pAF_Release(a_pstInode, a_pstFile);
		g_pstAF_CurDrv = NULL;
	}

	spin_lock(&g_AF_SpinLock);
	g_s4AF_Opened = 0;
	spin_unlock(&g_AF_SpinLock);

#if !defined(CONFIG_MTK_LEGACY)
	AFRegulatorCtrl(2);
#endif

	LOG_INF("End\n");

	return 0;
}
#else

/* OIS/EIS Timer & Workqueue */
static struct workqueue_struct *ois_workqueue;
static struct work_struct ois_work;

static int af_pinctrl_set(int pin, int state)
{
	int ret = 0;

	LOG_INF("+");
	if (af_pinctrl == NULL) {
		LOG_INF("pinctrl is not available\n");
		return -1;
	}

	if (af_hwen_high == NULL) {
		LOG_INF("af_hwen_high is not available\n");
		return -1;
	}

	if (af_hwen_low == NULL) {
		LOG_INF("af_hwen_low is not available\n");
		return -1;
	}

	switch (pin) {
	case AF_PINCTRL_PIN_HWEN:
		if (state == AF_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(af_hwen_low))
			pinctrl_select_state(af_pinctrl, af_hwen_low);
		else if (state == AF_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(af_hwen_high))
			pinctrl_select_state(af_pinctrl, af_hwen_high);
		else
			LOG_INF("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		LOG_INF("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	LOG_INF("pin(%d) state(%d)\n", pin, state);

	LOG_INF("-");

	return ret;
}

static inline int64_t getCurNS(void)
{
	int64_t ns;
	struct timespec time;

	time.tv_sec = 0;
        time.tv_nsec = 0;
	get_monotonic_boottime(&time);
	ns = time.tv_sec * 1000000000LL + time.tv_nsec;

	return ns;
}

/* OIS/EIS Timer & Workqueue */
static void ois_pos_polling(struct work_struct *data)
{
	mutex_lock(&ois_mutex);
	if (g_pstAF_CurDrv) {
		if (g_pstAF_CurDrv->pAF_OisGetHallPos) {
			int PosX = 0, PosY = 0;

			g_pstAF_CurDrv->pAF_OisGetHallPos(&PosX, &PosY);
			OisPosInfo.TimeStamp[g_OisPosIdx] = getCurNS();
			OisPosInfo.i4OISHallPosX[g_OisPosIdx] = PosX;
			OisPosInfo.i4OISHallPosY[g_OisPosIdx] = PosY;
			g_OisPosIdx++;
			g_OisPosIdx &= OIS_DATA_MASK;
		}
	}
	mutex_unlock(&ois_mutex);
}

static enum hrtimer_restart ois_timer_func(struct hrtimer *timer)
{
	g_GetOisInfoCnt--;

	if (ois_workqueue != NULL && g_GetOisInfoCnt > 11)
		queue_work(ois_workqueue, &ois_work);

	if (g_GetOisInfoCnt < 10) {
		g_EnableTimer = 0;
		return HRTIMER_NORESTART;
	}

	hrtimer_forward_now(timer, ktime_set(0, 5000000));
	return HRTIMER_RESTART;
}

static int AF_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	spin_lock(&g_AF_SpinLock);
	if (g_s4AF_Opened) {
		spin_unlock(&g_AF_SpinLock);
		LOG_INF("The device is opened\n");
		return -EBUSY;
	}
	g_s4AF_Opened = 1;
	spin_unlock(&g_AF_SpinLock);

        #ifndef OPLUS_FEATURE_CAMERA_COMMON
	af_pinctrl_set(AF_PINCTRL_PIN_HWEN,
			AF_PINCTRL_PINSTATE_HIGH);
            #if !defined(CONFIG_MTK_LEGACY)
	    AFRegulatorCtrl(0);
	    AFRegulatorCtrl(1);
            #endif /* CONFIG_MTK_LEGACY */
        #else /* OPLUS_FEATURE_CAMERA_COMMON */
        if(!is_project(19165)) {
			af_pinctrl_set(AF_PINCTRL_PIN_HWEN, AF_PINCTRL_PINSTATE_HIGH);
            #if !defined(CONFIG_MTK_LEGACY)
	    AFRegulatorCtrl(0);
	    AFRegulatorCtrl(1);
            #endif /* CONFIG_MTK_LEGACY */
        } else {
            af_pinctrl_set(AF_PINCTRL_PIN_HWEN, AF_PINCTRL_PINSTATE_HIGH);
	}
        #endif /* OPLUS_FEATURE_CAMERA_COMMON */
	/* OIS/EIS Timer & Workqueue */
	/* init work queue */
	INIT_WORK(&ois_work, ois_pos_polling);

#if 0
	if (ois_workqueue == NULL)
		ois_workqueue = create_singlethread_workqueue("ois_polling");
#endif

	/* init timer */
	hrtimer_init(&ois_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ois_timer.function = ois_timer_func;

	g_EnableTimer = 0;
	/* ------------------------- */

	LOG_INF("End\n");

	return 0;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
static int AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (g_pstAF_CurDrv) {
		g_pstAF_CurDrv->pAF_Release(a_pstInode, a_pstFile);
		g_pstAF_CurDrv = NULL;
	} else {
		spin_lock(&g_AF_SpinLock);
		g_s4AF_Opened = 0;
		spin_unlock(&g_AF_SpinLock);
	}
        #ifndef OPLUS_FEATURE_CAMERA_COMMON
	af_pinctrl_set(AF_PINCTRL_PIN_HWEN,
			AF_PINCTRL_PINSTATE_LOW);
            #if !defined(CONFIG_MTK_LEGACY)
	    AFRegulatorCtrl(2);
            #endif /* CONFIG_MTK_LEGACY */
        #else /* OPLUS_FEATURE_CAMERA_COMMON */
        if (!is_project(19165)) {
            #if !defined(CONFIG_MTK_LEGACY)
            AFRegulatorCtrl(2);
            #endif /* CONFIG_MTK_LEGACY */
        } else {
                af_pinctrl_set(AF_PINCTRL_PIN_HWEN, AF_PINCTRL_PINSTATE_LOW);
        }
        #endif /* OPLUS_FEATURE_CAMERA_COMMON */
	/* OIS/EIS Timer & Workqueue */
	/* Cancel Timer */
	hrtimer_cancel(&ois_timer);

	/* flush work queue */
	flush_work(&ois_work);

	if (ois_workqueue) {
		flush_workqueue(ois_workqueue);
		destroy_workqueue(ois_workqueue);
		ois_workqueue = NULL;
	}
	/* ------------------------- */

	LOG_INF("End\n");

	return 0;
}
#endif

static const struct file_operations g_stAF_fops = {
	.owner = THIS_MODULE,
	.open = AF_Open,
	.release = AF_Release,
	.unlocked_ioctl = AF_Ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = AF_Ioctl_Compat,
#endif
};

static inline int Register_AF_CharDrv(void)
{
	LOG_INF("Start\n");

	/* Allocate char driver no. */
	if (alloc_chrdev_region(&g_AF_devno, 0, 1, AF_DRVNAME)) {
		LOG_INF("Allocate device no failed\n");

		return -EAGAIN;
	}
	/* Allocate driver */
	g_pAF_CharDrv = cdev_alloc();

	if (g_pAF_CharDrv == NULL) {
		unregister_chrdev_region(g_AF_devno, 1);

		LOG_INF("Allocate mem for kobject failed\n");

		return -ENOMEM;
	}
	/* Attatch file operation. */
	cdev_init(g_pAF_CharDrv, &g_stAF_fops);

	g_pAF_CharDrv->owner = THIS_MODULE;

	/* Add to system */
	if (cdev_add(g_pAF_CharDrv, g_AF_devno, 1)) {
		LOG_INF("Attatch file operation failed\n");

		unregister_chrdev_region(g_AF_devno, 1);

		return -EAGAIN;
	}

	actuator_class = class_create(THIS_MODULE, AF_DRIVER_CLASS_NAME);
	if (IS_ERR(actuator_class)) {
		int ret = PTR_ERR(actuator_class);

		LOG_INF("Unable to create class, err = %d\n", ret);
		return ret;
	}

	lens_device = device_create(actuator_class, NULL, g_AF_devno, NULL,
				    AF_DRVNAME);

	if (lens_device == NULL)
		return -EIO;

	LOG_INF("End\n");
	return 0;
}

static inline void Unregister_AF_CharDrv(void)
{
	LOG_INF("Start\n");

	/* Release char driver */
	cdev_del(g_pAF_CharDrv);

	unregister_chrdev_region(g_AF_devno, 1);

	device_destroy(actuator_class, g_AF_devno);

	class_destroy(actuator_class);

	LOG_INF("End\n");
}

/* //////////////////////////////////////////////////////////////////// */

static int AF_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id);
static int AF_i2c_remove(struct i2c_client *client);
static const struct i2c_device_id AF_i2c_id[] = {{AF_DRVNAME, 0}, {} };

/* TOOL : kernel-3.10\tools\dct */
/* PATH : vendor\mediatek\proprietary\custom\#project#\kernel\dct\dct */
#if I2C_CONFIG_SETTING == 2
static const struct of_device_id MAINAF_of_match[] = {
	{.compatible = "mediatek,CAMERA_MAIN_AF"}, {},
};
#endif

static struct i2c_driver AF_i2c_driver = {
	.probe = AF_i2c_probe,
	.remove = AF_i2c_remove,
	.driver.name = AF_DRVNAME,
#if I2C_CONFIG_SETTING == 2
	.driver.of_match_table = MAINAF_of_match,
#endif
	.id_table = AF_i2c_id,
};

static int AF_i2c_remove(struct i2c_client *client)
{
	Unregister_AF_CharDrv();
	return 0;
}

/* Kirby: add new-style driver {*/
static int AF_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int i4RetValue = 0;

	LOG_INF("Start\n");

	/* Kirby: add new-style driver { */
	g_pstAF_I2Cclient = client;

	/* Register char driver */
	i4RetValue = Register_AF_CharDrv();

	if (i4RetValue) {

		LOG_INF(" register char device failed!\n");

		return i4RetValue;
	}

	spin_lock_init(&g_AF_SpinLock);

	LOG_INF("Attached!!\n");

	return 0;
}

static int AF_probe(struct platform_device *pdev)
{
	if (af_pinctrl_init(&pdev->dev))
		LOG_INF("Failed to init pinctrl.\n");
	return i2c_add_driver(&AF_i2c_driver);
}

static int AF_remove(struct platform_device *pdev)
{
	i2c_del_driver(&AF_i2c_driver);
	return 0;
}

static int AF_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int AF_resume(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id gaf_of_device_id[] = {
	{.compatible = "mediatek,camera_af_lens",},
	{}
};
#endif

/* platform structure */
static struct platform_driver g_stAF_Driver = {
	.probe = AF_probe,
	.remove = AF_remove,
	.suspend = AF_suspend,
	.resume = AF_resume,
	.driver = {
		.name = PLATFORM_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = gaf_of_device_id,
#endif
	} };

static struct platform_device g_stAF_device = {
	.name = PLATFORM_DRIVER_NAME, .id = 0, .dev = {} };

static int __init MAINAF_i2C_init(void)
{
#if I2C_CONFIG_SETTING == 1
	i2c_register_board_info(LENS_I2C_BUSNUM, &kd_lens_dev, 1);
#endif

	if (platform_device_register(&g_stAF_device)) {
		LOG_INF("failed to register AF driver\n");
		return -ENODEV;
	}

	if (platform_driver_register(&g_stAF_Driver)) {
		LOG_INF("Failed to register AF driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit MAINAF_i2C_exit(void)
{
	platform_driver_unregister(&g_stAF_Driver);
	platform_device_unregister(&g_stAF_device);
}
module_init(MAINAF_i2C_init);
module_exit(MAINAF_i2C_exit);

MODULE_DESCRIPTION("MAINAF lens module driver");
MODULE_AUTHOR("KY Chen <ky.chen@Mediatek.com>");
MODULE_LICENSE("GPL");
