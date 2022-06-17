/********************************************
 ** Copyright (C) 2019 OPLUS Mobile Comm Corp. Ltd.
 ** ODM_HQ_EDIT
 ** File: aw87339_audio.h
 ** Description: header file of aw87339 speaker pa
 ** Version: 1.0
 ** Date : 2019/10/09
 ** ---------------- Revision History: --------------------------
 ** <version>    <date>          < author >              <desc>
 ********************************************/

#ifndef __AW87339_H__
#define __AW87339_H__


unsigned char aw87339_kspk_cfg_default[] = {
	/*0x00, 0x39,  CHIPID REG*/
	0x01, 0x0E,
	0x02, 0xA3,
	0x03, 0x06,
	0x04, 0x05,
	0x05, 0x10,
	0x06, 0x07,
	0x07, 0x52,
	0x08, 0x06,
	0x09, 0x08,
	0x0A, 0x96
};

unsigned char aw87339_drcv_cfg_default[] = {
	/*0x00, 0x39,  CHIPID REG */
	0x01, 0x0A,
	0x02, 0xAB,
	0x03, 0x06,
	0x04, 0x05,
	0x05, 0x00,
	0x06, 0x0F,
	0x07, 0x52,
	0x08, 0x09,
	0x09, 0x08,
	0x0A, 0x97
};
unsigned char aw87339_abrcv_cfg_default[] = {
	/*0x00, 0x39,  CHIPID REG*/
	0x01, 0x0A,
	0x02, 0xAF,
	0x03, 0x06,
	0x04, 0x05,
	0x05, 0x00,
	0x06, 0x0F,
	0x07, 0x52,
	0x08, 0x09,
	0x09, 0x08,
	0x0A, 0x97
};
unsigned char aw87339_rcvspk_cfg_default[] = {
	/*0x00, 0x39,  CHIPID REG*/
	0x01, 0x0E,
	0x02, 0xB3,
	0x03, 0x06,
	0x04, 0x05,
	0x05, 0x00,
	0x06, 0x07,
	0x07, 0x52,
	0x08, 0x06,
	0x09, 0x08,
	0x0A, 0x96
};

unsigned char aw87339_voicespk_cfg_default[] = {
	/*0x00, 0x39,  CHIPID REG*/
	0x01, 0x0E,
	0x02, 0xA3,
	0x03, 0x06,
	0x04, 0x05,
	0x05, 0x10,
	0x06, 0x07,
	0x07, 0x52,
	0x08, 0x06,
	0x09, 0x08,
	0x0A, 0x96
};
/******************************************************
 *
 *Load config function
 *This driver will use load firmware if AW20036_BIN_CONFIG be defined
 *****************************************************/
#define AWINIC_CFG_UPDATE_DELAY
#define AW_I2C_RETRIES 5
#define AW_I2C_RETRY_DELAY 2
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 2

#define REG_CHIPID            0x00
#define REG_SYSCTRL           0x01
#define REG_BATSAFE           0x02
#define REG_BSTOVR            0x03
#define REG_BSTVPR            0x04
#define REG_PAGR              0x05
#define REG_PAGC3OPR          0x06
#define REG_PAGC3PR           0x07
#define REG_PAGC2OPR          0x08
#define REG_PAGC2PR           0x09
#define REG_PAGC1PR           0x0A

#define AW87339_CHIPID      0x39
#define AW87339_REG_MIN     11

#define AWINIC_CFG_UPDATE_DELAY

#define AW_I2C_RETRIES 5
#define AW_I2C_RETRY_DELAY 2
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 2

#define AW87339_REG_CHIPID      0x00
#define AW87339_REG_SYSCTRL     0x01
#define AW87339_REG_MODECTRL    0x02
#define AW87339_REG_CPOVP       0x03
#define AW87339_REG_CPP         0x04
#define AW87339_REG_GAIN        0x05
#define AW87339_REG_AGC3_PO     0x06
#define AW87339_REG_AGC3        0x07
#define AW87339_REG_AGC2_PO     0x08
#define AW87339_REG_AGC2        0x09
#define AW87339_REG_AGC1        0x0A
#define AW87339_REG_DFT1        0x62
#define AW87339_REG_DFT2        0x63
#define AW87339_REG_ENCRY       0x64

#define AW87339_CHIP_DISABLE    0x0c

#define REG_NONE_ACCESS         0
#define REG_RD_ACCESS           (1 << 0)
#define REG_WR_ACCESS           (1 << 1)
#define AW87339_REG_MAX         0xFF

const unsigned char aw87339_reg_access[AW87339_REG_MAX] = {
	[AW87339_REG_CHIPID] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW87339_REG_SYSCTRL] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW87339_REG_MODECTRL] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW87339_REG_CPOVP] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW87339_REG_CPP] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW87339_REG_GAIN] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW87339_REG_AGC3_PO] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW87339_REG_AGC3] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW87339_REG_AGC2_PO] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW87339_REG_AGC2] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW87339_REG_AGC1] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW87339_REG_DFT1] = REG_RD_ACCESS,
	[AW87339_REG_DFT2] = REG_RD_ACCESS,
	[AW87339_REG_ENCRY] = REG_RD_ACCESS,
};

struct aw87339_container {
	int len;
	unsigned char data[];
};

struct aw87339 {
	struct i2c_client *i2c_client;
	int reset_gpio;
	unsigned char init_flag;
	unsigned char hwen_flag;
	unsigned char kspk_cfg_update_flag;
	unsigned char drcv_cfg_update_flag;
	unsigned char abrcv_cfg_update_flag;
	unsigned char rcvspk_cfg_update_flag;
	unsigned char voicespk_cfg_update_flag;
	struct hrtimer cfg_timer;
	struct mutex cfg_lock;
	struct work_struct cfg_work;
	struct delayed_work ram_work;
};

/****************************************************
 * aw87339 functions
 ****************************************************/
extern unsigned char aw87339_audio_off(void);
extern unsigned char aw87339_audio_kspk(void);
extern unsigned char aw87339_audio_drcv(void);
extern unsigned char aw87339_audio_abrcv(void);
extern unsigned char aw87339_audio_rcvspk(void);
extern int aw87339_audio_probe_get(void);
#endif /* __AW87339_H__ */

