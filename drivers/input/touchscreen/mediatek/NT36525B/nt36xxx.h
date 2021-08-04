/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 *
 * $Revision$
 * $Date$
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef 	_LINUX_NVT_TOUCH_H
#define		_LINUX_NVT_TOUCH_H

#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
//#include <linux/fb_notify.h>

#ifdef ODM_WT_EDIT
#include<linux/input/touch-info.h>
#include<linux/workqueue.h>
#endif

#ifdef ODM_WT_EDIT
#include <linux/regulator/consumer.h>
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "nt36xxx_mem_map.h"

#ifdef CONFIG_MTK_SPI
/* Please copy mt_spi.h file under mtk spi driver folder */
#include "mt_spi.h"
#endif

#ifdef CONFIG_SPI_MT65XX
#include <linux/platform_data/spi-mt65xx.h>
#endif

#define NVT_DEBUG 1

//---GPIO number---
#define NVTTOUCH_RST_PIN 980
#define NVTTOUCH_INT_PIN 943


//---INT trigger mode---
//#define IRQ_TYPE_EDGE_RISING 1
//#define IRQ_TYPE_EDGE_FALLING 2
#define INT_TRIGGER_TYPE IRQ_TYPE_EDGE_RISING

#ifdef ODM_WT_EDIT
extern int tp_gesture;
#endif

//---SPI driver info.---
#define NVT_SPI_NAME "NVT-ts"

#if 1
#define NVT_LOG(fmt, args...)    pr_err("[TP][%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)
#else
#define NVT_LOG(fmt, args...)    pr_info("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)
#endif
#define NVT_ERR(fmt, args...)    pr_err("[TP][%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)

//---Input device info.---
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"


//---Touch info.---
#define TOUCH_DEFAULT_MAX_WIDTH 720
#define TOUCH_DEFAULT_MAX_HEIGHT 1600
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_KEY_NUM 0
#if TOUCH_KEY_NUM > 0
extern const uint16_t touch_key_array[TOUCH_KEY_NUM];
#endif


#ifdef ODM_WT_EDIT
#define TOUCH_FORCE_NUM 255
#else
#define TOUCH_FORCE_NUM 1000
#endif

/* Enable only when module have tp reset pin and connected to host */
#define NVT_TOUCH_SUPPORT_HW_RST 1

//---Customerized func.---
#define NVT_TOUCH_PROC 1
#define NVT_TOUCH_EXT_PROC 1
#define NVT_TOUCH_MP 1
#define MT_PROTOCOL_B 1
#define NVT_TOUCH_FW_DEBUG_INFO 1
#define WAKEUP_GESTURE 1
#if WAKEUP_GESTURE
//extern const uint16_t gesture_key_array[];
#endif
struct coordinate {
	uint16_t x;
	uint16_t y;
};

struct gesture_info {
	uint8_t gesture_type;
	uint8_t clockwise;
	struct coordinate Point_start;
	struct coordinate Point_end;
	struct coordinate Point_1st;
	struct coordinate Point_2nd;
	struct coordinate Point_3rd;
	struct coordinate Point_4th;
};

struct gesture_black {
	int flag;
	uint8_t gesture_backup;
	char *message;
};

typedef enum {
    CORNER_TOPLEFT,      /*When Phone Face you in portrait top left corner*/
    CORNER_TOPRIGHT,     /*When Phone Face you in portrait top right corner*/
    CORNER_BOTTOMLEFT,   /*When Phone Face you in portrait bottom left corner*/
    CORNER_BOTTOMRIGHT,  /*When Phone Face you in portrait bottom right corner*/
} nvt_corner_type;

enum nvt_area_type {
	NVT_AREA_NORMAL,
	NVT_AREA_CORNER,
	NVT_AREA_DEFAULT,
};

struct nvt_edge_limit_data {
	unsigned int limit_control;
	char limit_00;
	char limit_lu;
	char limit_ru;
	char limit_lb;
	char limit_rb;
};
struct nvt_limit_area_data {
	int limit_area;
	int area_xlu;
	int area_xru;
	int area_xlb;
	int area_xrb;
	int area_ylu;
	int area_yru;
	int area_ylb;
	int area_yrb;
	int which_area;
};
struct Nvt_point_info {
	int x;
	int y;
	int type;
};

struct Nvt_coner_info {
	int id;
	bool flag;
	struct Nvt_point_info point_info;
};


struct oplus_debug_info {
	struct coordinate coordinate[10];
};

struct oplus_debug_gesture_record_info {
	struct coordinate coordinate[1024];
};

#define BOOT_UPDATE_FIRMWARE 1
#ifdef ODM_WT_EDIT
#define BOOT_UPDATE_FIRMWARE_HLT_BOE_NAME      "/tp/19741/hlt_boe_novatek_fw.bin"
#define BOOT_UPDATE_FIRMWARE_HLT_BOE_GG3_NAME  "/tp/19741/hlt_boe_gg3_novatek_fw.bin"
#define BOOT_UPDATE_FIRMWARE_HLT_BOE_B3_NAME  "/tp/206AC/hlt_boe_b3_novatek_fw.bin"

#define MP_UPDATE_FIRMWARE_HLT_BOE_NAME        "/tp/19741/hlt_boe_novatek_mp_fw.bin"
#define MP_UPDATE_FIRMWARE_HLT_BOE_GG3_NAME    "/tp/19741/hlt_boe_gg3_novatek_mp_fw.bin"
#define MP_UPDATE_FIRMWARE_HLT_BOE_B3_NAME    "/tp/206AC/hlt_boe_b3_novatek_mp_fw.bin"

#define OPLUS_BOOT_UPDATE_FIRMWARE_HLT_BOE_NAME "tp/19741/hlt_boe_novatek_signed_fw.bin"
#define OPLUS_BOOT_UPDATE_FIRMWARE_HLT_BOE_GG3_NAME "tp/19741/hlt_boe_gg3_novatek_signed_fw.bin"
#define OPLUS_BOOT_UPDATE_FIRMWARE_HLT_BOE_B3_NAME "tp/206AC/hlt_boe_b3_novatek_signed_fw.bin"

#endif

//---ESD Protect.---
#define NVT_TOUCH_ESD_PROTECT 1
#define NVT_TOUCH_ESD_CHECK_PERIOD 2000	/* ms */
#define NVT_TOUCH_WDT_RECOVERY 1
struct nvt_oplus_data {
	struct proc_dir_entry *touchpanel_dir_entry;
	struct proc_dir_entry *baseline_test_entry;
	struct proc_dir_entry *coordinate_entry;
	struct proc_dir_entry *debug_level_entry;
	struct proc_dir_entry *double_tap_enable_entry;
	struct proc_dir_entry *irq_depath_entry;
	struct proc_dir_entry *oplus_register_info_entry;
	struct proc_dir_entry *oplus_tp_limit_area_entry;
	struct proc_dir_entry *oplus_tp_limit_enable_entry;
	struct proc_dir_entry *oplus_tp_direction_entry;
#ifdef ODM_WT_EDIT
	struct proc_dir_entry *freq_hop_simulate_entry;
	struct proc_dir_entry *fix_hop_simulate_entry;

#endif
	//struct proc_dir_entry *ps_status_entry;
	struct proc_dir_entry *tp_fw_update_entry;
	struct proc_dir_entry *black_screen_test_entry;
	struct proc_dir_entry *i2c_device_test_entry;
	struct proc_dir_entry *game_switch_enable_entry;

	struct proc_dir_entry *debug_info_dir_entry;
	struct proc_dir_entry *delta_entry;
	struct proc_dir_entry *baseline_entry;
	struct proc_dir_entry *main_register_entry;

	struct proc_dir_entry *touchscreen_dir_entry;
	struct proc_dir_entry *ctp_openshort_test_entry;

	struct gesture_black gesture_test;
	struct nvt_edge_limit_data edge_limit;
	struct nvt_limit_area_data nvt_limit_area;
	struct Nvt_point_info nvt_point_info[10];
	struct Nvt_coner_info nvt_coner_info[4];

};

struct nvt_ts_data {
	struct spi_device *client;
	struct input_dev *input_dev;
	struct work_struct nvt_work;
	struct delayed_work nvt_fwu_work;
	uint16_t addr;
	int8_t phys[32];
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	uint8_t fw_ver;
	uint8_t x_num;
	uint8_t y_num;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint32_t int_trigger_type;
	int32_t irq_gpio;
	uint32_t irq_flags;
	int32_t reset_gpio;
	uint32_t reset_flags;
	int32_t reset_global_gpio;
	uint32_t reset_global_flags;
	uint32_t irq_enable_time;
	uint32_t irq_enable_flag;
	spinlock_t spinlock_int;
	struct mutex lock;
	const struct nvt_ts_mem_map *mmap;
	uint8_t carrier_system;
	uint8_t hw_crc;
	uint16_t nvt_pid;
	uint8_t rbuf[1025];
	uint8_t *xbuf;
	struct mutex xbuf_lock;
	bool irq_enabled;
	uint8_t debug_level;
	bool oplus_baseline_test_flag;
	int boot_mode;
	bool recovery_flag;
	int is_suspended;
	int g_gesture_bak;
	struct gesture_info gesture;
	struct nvt_oplus_data *nvt_oplus_proc_data;
	uint8_t gesture_enable;
	struct oplus_debug_info oplus_debug_info;
	struct notifier_block notifier_update_fw;
	struct notifier_block notifier_tp_usb;
	struct notifier_block notifier_headset;
	struct work_struct resume_work_queue;
	struct work_struct tp_usb_work_queue;
	struct work_struct headset_work_queue;
	struct touch_info_dev *tid;
#ifdef CONFIG_MTK_SPI
	struct mt_chip_conf spi_ctrl;
#endif
#ifdef CONFIG_SPI_MT65XX
    struct mtk_chip_config spi_ctrl;
#endif
#ifdef ODM_WT_EDIT
	int vendor_id;
#endif
#ifdef ODM_WT_EDIT
	struct regulator *iovcc_pwr;
	struct regulator *vsp_pwr;
	struct regulator *vsn_pwr;
#endif
};

#if NVT_TOUCH_PROC
struct nvt_flash_data{
	rwlock_t lock;
};
#endif

#ifdef ODM_WT_EDIT
typedef enum {
	HLT_BOE  = 0,
	HLT_BOE_GG3 = 1,
	HLT_BOE_B3 = 2,
} NVT_MODULE_ID;

struct upgrade_fw_info {
	NVT_MODULE_ID id;
	char module_vendor[20];
	char *firmware_name;
	char *firmware_mp_name;
	const char * fw_file;
	uint32_t fw_len;
	char *firmware_sign_name;
};
#endif

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,		// ReK baseline
	RESET_STATE_REK_FINISH,	// baseline is ready
	RESET_STATE_NORMAL_RUN,	// normal run
	RESET_STATE_MAX  = 0xAF
} RST_COMPLETE_STATE;

typedef enum {
    EVENT_MAP_HOST_CMD                      = 0x50,
    EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
    EVENT_MAP_RESET_COMPLETE                = 0x60,
    EVENT_MAP_FWINFO                        = 0x78,
    EVENT_MAP_PROJECTID                     = 0x9A,
} SPI_EVENT_MAP;

typedef enum {
	MODE_EDGE = 0,
	MODE_CHARGE,
	MODE_GAME,
	MODE_HEADSET,
#ifdef ODM_WT_EDIT
	MODE_HOPPING_POLLING
#endif
} NVT_CUSTOMIZED_MODE;

typedef enum {
    EDGE_REJECT_L = 0,
    EDGE_REJECT_H,
    PWR_FLAG,
#ifdef ODM_WT_EDIT
    HOPPING_POLLING_FLAG = 4,
#endif
	JITTER_FLAG = 6,
	HEADSET_FLAG = 7,

} CMD_OFFSET;

//---customized command---
#define HOST_CMD_PWR_PLUG_IN    (0x53)
#define HOST_CMD_PWR_PLUG_OUT   (0x51)

#ifdef ODM_WT_EDIT
#define HOST_CMD_HOPPING_POLLING_ON   (0x73)
#define HOST_CMD_HOPPING_POLLING_OFF  (0x74)
#define HOST_CMD_HOPPING_FIX_FREQ_ON  (0x75)
#define HOST_CMD_HOPPING_FIX_FREQ_OFF (0x76)

#endif

#define HOST_CMD_JITTER_ON      (0x7D)
#define HOST_CMD_JITTER_OFF     (0x7E)

#define HOST_CMD_HEADSET_PLUG_IN      (0x77)
#define HOST_CMD_HEADSET_PLUG_OUT     (0x78)
#define HOST_CMD_EDGE_LIMIT_VERTICAL  (0x7A)
#define HOST_CMD_EDGE_LIMIT_LEFT_UP   (0x7B)
#define HOST_CMD_EDGE_LIMIT_RIGHT_UP  (0x7C)


//---SPI READ/WRITE---
#define SPI_WRITE_MASK(a)	(a | 0x80)
#define SPI_READ_MASK(a)	(a & 0x7F)

#define DUMMY_BYTES (1)
#define NVT_TANSFER_LEN		(PAGE_SIZE << 4)

typedef enum {
	NVTWRITE = 0,
	NVTREAD  = 1
} NVT_SPI_RW;

//---extern structures---
extern struct nvt_ts_data *ts;
#ifdef ODM_WT_EDIT
extern struct upgrade_fw_info *fw;
#endif

//---extern functions---
int32_t CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len);
int32_t CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len);
void nvt_bootloader_reset(void);
void nvt_eng_reset(void);
void nvt_sw_reset(void);
void nvt_sw_reset_idle(void);
void nvt_boot_ready(void);
void nvt_bld_crc_enable(void);
void nvt_fw_crc_enable(void);
void nvt_update_firmware(char *firmware_name);
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state);
int32_t nvt_get_fw_info(void);
int32_t nvt_clear_fw_status(void);
int32_t nvt_check_fw_status(void);
#if NVT_TOUCH_ESD_PROTECT
extern void nvt_esd_check_enable(uint8_t enable);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

int32_t nvt_set_page(uint32_t addr);
int32_t nvt_write_addr(uint32_t addr, uint8_t data);
int32_t nvt_mode_switch(NVT_CUSTOMIZED_MODE mode, uint8_t flag);
void nvt_read_mdata(uint32_t xdata_addr, uint32_t xdata_btn_addr);
void nvt_read_mdata_rss(uint32_t xdata_i_addr, uint32_t xdata_q_addr,
		uint32_t xdata_btn_i_addr, uint32_t xdata_btn_q_addr);
#endif /* _LINUX_NVT_TOUCH_H */
