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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/input/mt.h>

#ifdef ODM_WT_EDIT
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include "nt36xxx.h"
#if NVT_TOUCH_ESD_PROTECT
#include <linux/jiffies.h>
#endif /* #if NVT_TOUCH_ESD_PROTECT */
#include <linux/update_tpfw_notifier.h>
#include <linux/tp_usb_notifier.h>
#include <linux/headset_notifier.h>

#ifdef ODM_WT_EDIT
#include <linux/hardware_info.h>
#endif

#define FTS_UPGRADE_HLT_BOE_FILE "firmware_nvt/hlt_boe_fw.h"
#define FTS_UPGRADE_HLT_BOE_GG3_FILE "firmware_nvt/hlt_boe_gg3_fw.h"
#define FTS_UPGRADE_HLT_BOE_B3_FILE "firmware_nvt/hlt_boe_b3_fw.h"

const u8 up_fw_kernel_hlt_boe[] = {
#include FTS_UPGRADE_HLT_BOE_FILE
};

const u8 up_fw_kernel_hlt_boe_gg3[] = {
#include FTS_UPGRADE_HLT_BOE_GG3_FILE
};

const u8 up_fw_kernel_hlt_boe_b3[] = {
#include FTS_UPGRADE_HLT_BOE_B3_FILE
};

extern int get_boot_mode(void);
extern void devinfo_info_tp_set(char *version, char *manufacture, char *fw_path);
//extern int register_tp_proc(char *name, char *version, char *manufacture ,char *fw_path);

#if NVT_TOUCH_ESD_PROTECT
static struct delayed_work nvt_esd_check_work;
static struct workqueue_struct *nvt_esd_check_wq;
static unsigned long irq_timer = 0;

uint8_t esd_check = false;
uint8_t esd_retry = 0;
#endif /* #if NVT_TOUCH_ESD_PROTECT */

static unsigned long nvt_tp_usb_state = 0;
static unsigned long nvt_tp_headset_state = 0;
static char Ctp_name[64];

#ifdef ODM_WT_EDIT
static char version[20] = {"0"};
#endif

#ifdef ODM_WT_EDIT
//int	power_flag =  0; /* 0 disable ; 1 enable;no unblance*/
#endif

#if NVT_TOUCH_EXT_PROC
extern int32_t nvt_extra_proc_init(void);
#endif

#ifdef ODM_WT_EDIT
extern char Ctp_name[HARDWARE_MAX_ITEM_LONGTH];
#endif

#if NVT_TOUCH_MP
extern int32_t nvt_mp_proc_init(void);
#endif

struct nvt_ts_data *ts;

int gloal_reset_flag;
//static struct workqueue_struct *nvt_wq;

#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
extern void Boot_Update_Firmware(struct work_struct *work);
#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void nvt_ts_early_suspend(struct early_suspend *h);
static void nvt_ts_late_resume(struct early_suspend *h);
#endif

uint32_t ENG_RST_ADDR  = 0x7FFF80;
uint32_t SWRST_N8_ADDR = 0; //read from dtsi

#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM] = {
	KEY_BACK,
	KEY_HOME,
	KEY_MENU
};
#endif

#ifdef CONFIG_MTK_SPI
const struct mt_chip_conf spi_ctrdata = {
	.setuptime = 25,
	.holdtime = 25,
	.high_time = 5,	/* 10MHz (SPI_SPEED=100M / (high_time+low_time(10ns)))*/
	.low_time = 5,
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,
	.cpol = 0,
	.cpha = 0,
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = DMA_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
#endif

#ifdef CONFIG_SPI_MT65XX
const struct mtk_chip_config spi_ctrdata = {
    .rx_mlsb = 1,
    .tx_mlsb = 1,
 //   .cs_pol = 0,
};
#endif

#ifdef ODM_WT_EDIT
struct upgrade_fw_info nvt_fw_list[] = {
    {HLT_BOE, "HLT", BOOT_UPDATE_FIRMWARE_HLT_BOE_NAME,  MP_UPDATE_FIRMWARE_HLT_BOE_NAME, up_fw_kernel_hlt_boe, sizeof(up_fw_kernel_hlt_boe), OPLUS_BOOT_UPDATE_FIRMWARE_HLT_BOE_NAME},
    {HLT_BOE_GG3, "HLT", BOOT_UPDATE_FIRMWARE_HLT_BOE_GG3_NAME,  MP_UPDATE_FIRMWARE_HLT_BOE_GG3_NAME, up_fw_kernel_hlt_boe_gg3, sizeof(up_fw_kernel_hlt_boe_gg3), OPLUS_BOOT_UPDATE_FIRMWARE_HLT_BOE_GG3_NAME},
    {HLT_BOE_B3, "HLT_B3", BOOT_UPDATE_FIRMWARE_HLT_BOE_B3_NAME,  MP_UPDATE_FIRMWARE_HLT_BOE_B3_NAME, up_fw_kernel_hlt_boe_b3, sizeof(up_fw_kernel_hlt_boe_b3), OPLUS_BOOT_UPDATE_FIRMWARE_HLT_BOE_B3_NAME},
};
struct upgrade_fw_info *fw;
int this_is_nvt_touch;
#endif

static uint8_t bTouchIsAwake = 0;

/*******************************************************
Description:
	Novatek touchscreen spi read/write core function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/

static inline int32_t spi_read_write(struct spi_device *client, uint8_t *buf, size_t len , NVT_SPI_RW rw)
{
	struct spi_message m;
	struct spi_transfer t = {
		.len    = len,
	};

	memcpy(ts->xbuf, buf, len + DUMMY_BYTES);

	switch (rw) {
		case NVTREAD:
			t.tx_buf = ts->xbuf;
			t.rx_buf = ts->rbuf;
			t.len    = (len + DUMMY_BYTES);
			break;

		case NVTWRITE:
			t.tx_buf = ts->xbuf;
			break;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(client, &m);
}

/*******************************************************
Description:
	Novatek touchscreen spi read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ts->xbuf_lock);

	buf[0] = SPI_READ_MASK(buf[0]);

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, NVTREAD);
		if (ret == 0) break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("read error, ret=%d\n", ret);
		ret = -EIO;
	} else {
		memcpy((buf+1), (ts->rbuf+2), (len-1));
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen spi write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ts->xbuf_lock);

	buf[0] = SPI_WRITE_MASK(buf[0]);

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, NVTWRITE);
		if (ret == 0)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen RESET DDI reset function.

return:
	N/A.
*******************************************************/
void nvt_rest_ddi(void)
{
	gpio_set_value(ts->reset_global_gpio, 0);
	NVT_LOG("%s : Pull down reset_global_gpio",__func__);
	mdelay(1);
	gpio_set_value(ts->reset_global_gpio, 1);
	NVT_LOG("%s : Pull up reset_global_gpio",__func__);
}

#ifdef ODM_WT_EDIT
#if 0
/*******************************************************
Description:
	Novatek touchscreen POWER ON function.

return:
	N/A.

*******************************************************/
static int power_init(struct nvt_ts_data *power_idev, bool on)
{
    int rc = 0;
    if ( !on ) {
		NVT_ERR("power_init is deny\n");
        goto pwr_deny;
	}
    power_idev->iovcc_pwr = regulator_get(&ts->client->dev, "vddio");
	if ( IS_ERR(power_idev->iovcc_pwr) ) {
        rc = PTR_ERR(power_idev->iovcc_pwr);
		NVT_ERR("Regulator get failed power_idev->iovcc_pwr rc=%d\n",rc);
		goto gpio_pwr_err;
    }
    power_idev->vsp_pwr = regulator_get(&ts->client->dev, "lab");
	if ( IS_ERR(power_idev->vsp_pwr) ) {
        rc = PTR_ERR(power_idev->vsp_pwr);
		NVT_ERR("Regulator get failed power_idev->vsp_pwr rc=%d\n",rc);
		goto lab_pwr_err;
    }
    power_idev->vsn_pwr = regulator_get(&ts->client->dev, "ibb");
	if ( IS_ERR(power_idev->vsn_pwr) ) {
        rc = PTR_ERR(power_idev->vsn_pwr);
		NVT_ERR("Regulator get failed power_idev->vsn_pwr rc=%d\n",rc);
		goto ibb_pwr_err;
    }
    return rc;

pwr_deny:
ibb_pwr_err:
	if (power_idev->vsn_pwr) {
		regulator_put(power_idev->vsn_pwr);
		power_idev->vsn_pwr = NULL;
	}
lab_pwr_err:
	if (power_idev->vsp_pwr) {
		regulator_put(power_idev->vsp_pwr);
		power_idev->vsp_pwr = NULL;
	}
gpio_pwr_err:
	if (power_idev->iovcc_pwr) {
		regulator_put(power_idev->iovcc_pwr);
		power_idev->iovcc_pwr = NULL;
	}

    return rc;

}

int  nvt_power_on(struct nvt_ts_data *power_idev, bool on)
{
    int rc = 0;
    if (!on) {
       NVT_LOG("ctp power_off\n");
	   if(power_flag == 0){
		   NVT_LOG("ctp is already power_off\n");
		   return 0;
	   }
       goto power_off;
	}
	NVT_LOG("ctp nvt_power_on start\n");
	if(power_flag == 1){
		NVT_LOG("ctp is already nvt_power_on\n");
		return 0;
	}
    rc = regulator_enable(power_idev->iovcc_pwr);
    if ( rc != 0 ) {
		NVT_ERR("enable iovcc fail\n");
        goto gpio_pwr_err;
    }
	rc = regulator_enable(power_idev->vsp_pwr);
    if ( rc != 0 ) {
		NVT_ERR("enable vsp fail\n");
        goto lab_pwr_err;
    }
    rc = regulator_enable(power_idev->vsn_pwr);
    if ( rc != 0 ) {
		NVT_ERR("enable vsn fail\n");
        goto ibb_pwr_err;
    }
	NVT_LOG("ctp nvt_power_on end\n");

    return rc;

power_off:
ibb_pwr_err:
	if (power_idev->vsn_pwr) {
		regulator_disable(power_idev->vsn_pwr);
	}
lab_pwr_err:
	if (power_idev->vsp_pwr) {
		regulator_disable(power_idev->vsp_pwr);
	}
gpio_pwr_err:
	if (power_idev->iovcc_pwr) {
		regulator_disable(power_idev->iovcc_pwr);
	}

    return rc;
}
#endif
#endif

/*******************************************************
Description:
	Novatek touchscreen set index/page/addr address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_set_page(uint32_t addr)
{
	uint8_t buf[4] = {0};

	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;

	return CTP_SPI_WRITE(ts->client, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen write data to specify address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_write_addr(uint32_t addr, uint8_t data)
{
	int32_t ret = 0;
	uint8_t buf[4] = {0};

	//---set xdata index---
	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;
	ret = CTP_SPI_WRITE(ts->client, buf, 3);
	if (ret) {
		NVT_ERR("set page 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	//---write data to index---
	buf[0] = addr & (0x7F);
	buf[1] = data;
	ret = CTP_SPI_WRITE(ts->client, buf, 2);
	if (ret) {
		NVT_ERR("write data to 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen enable hw bld crc function.

return:
	N/A.
*******************************************************/
void nvt_bld_crc_enable(void)
{
	uint8_t buf[4] = {0};
	//---set xdata index to BLD_CRC_EN_ADDR---
	nvt_set_page(ts->mmap->BLD_CRC_EN_ADDR);

	//---read data from index---
	buf[0] = ts->mmap->BLD_CRC_EN_ADDR & (0x7F);
	buf[1] = 0xFF;
	CTP_SPI_READ(ts->client, buf, 2);

	//---write data to index---
	buf[0] = ts->mmap->BLD_CRC_EN_ADDR & (0x7F);
	buf[1] = buf[1] | (0x01 << 7);
	CTP_SPI_WRITE(ts->client, buf, 2);
}

/*******************************************************
Description:
	Novatek touchscreen clear status & enable fw crc function.

return:
	N/A.
*******************************************************/
void nvt_fw_crc_enable(void)
{
	uint8_t buf[4] = {0};
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	//---clear fw reset status---
	buf[0] = EVENT_MAP_RESET_COMPLETE & (0x7F);
	buf[1] = 0x00;
	CTP_SPI_WRITE(ts->client, buf, 2);

	//---enable fw crc---
	buf[0] = EVENT_MAP_HOST_CMD & (0x7F);
	buf[1] = 0xAE;	//enable fw crc command
	CTP_SPI_WRITE(ts->client, buf, 2);
}

/*******************************************************
Description:
	Novatek touchscreen set boot ready function.

return:
	N/A.
*******************************************************/
void nvt_boot_ready(void)
{
	//---write BOOT_RDY status cmds---
	nvt_write_addr(ts->mmap->BOOT_RDY_ADDR, 1);

	mdelay(5);

	if (!ts->hw_crc) {
		//---write BOOT_RDY status cmds---
		nvt_write_addr(ts->mmap->BOOT_RDY_ADDR, 0);

		//---write POR_CD cmds---
		nvt_write_addr(ts->mmap->POR_CD_ADDR, 0xA0);
	}
}

/*******************************************************
Description:
	Novatek touchscreen eng reset cmd
    function.

return:
	n.a.
*******************************************************/
void nvt_eng_reset(void)
{
	//---eng reset cmds to ENG_RST_ADDR---
	nvt_write_addr(ENG_RST_ADDR, 0x5A);

	mdelay(1);	//wait tMCU_Idle2TP_REX_Hi after TP_RST
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset(void)
{
	//---software reset cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0x55);

	msleep(10);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle(void)
{
	//---MCU idle cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0xAA);

	msleep(15);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
void nvt_bootloader_reset(void)
{
	//---reset cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0x69);

	mdelay(5);	//wait tBRST2FR after Bootload RST
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---clear fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_WRITE(ts->client, buf, 2);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

void nvt_irq_enable(bool enable)
{
	unsigned long nIrqFlag;
	spin_lock_irqsave(&ts->spinlock_int, nIrqFlag);

	if (enable == 1 && ts->irq_enable_flag == 0) {
		enable_irq(ts->client->irq);
		ts->irq_enable_time++;
		ts->irq_enable_flag = 1;
	} else if (enable == 0 && ts->irq_enable_flag == 1) {
		disable_irq_nosync(ts->client->irq);
		ts->irq_enable_time--;
		ts->irq_enable_flag = 0;
	}

	NVT_LOG("ts->irq_enable_time = %d,ts->irq_enable_flag = %d\n", ts->irq_enable_time,ts->irq_enable_flag);
	spin_unlock_irqrestore(&ts->spinlock_int, nIrqFlag);
}

/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;
	int32_t retry_max = check_reset_state == RESET_STATE_INIT ? 10 : 50;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE);

	while (1) {
		//---read reset state---
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > retry_max)) {
			NVT_ERR("error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}

		msleep(10);
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get novatek project id information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_read_pid(void)
{
	uint8_t buf[5] = {0};
	int32_t ret = 0;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_PROJECTID);

	//---read project id---
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	CTP_SPI_READ(ts->client, buf, 3);

	ts->nvt_pid = (buf[2] << 8) + buf[1];

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	NVT_LOG("PID=%04X\n", ts->nvt_pid);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_get_fw_info(void)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int32_t ret = 0;
//	int rmtp = 0;

info_retry:
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_FWINFO);

	//---read fw info---
	buf[0] = EVENT_MAP_FWINFO;
	CTP_SPI_READ(ts->client, buf, 17);
	ts->fw_ver = buf[1];
	ts->x_num = buf[3];
	ts->y_num = buf[4];
	ts->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	ts->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
	ts->max_button_num = buf[11];

	//---clear x_num, y_num if fw info is broken---
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		ts->fw_ver = 0;
		ts->x_num = 18;
		ts->y_num = 32;
		ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
		ts->max_button_num = TOUCH_KEY_NUM;

		if(retry_count < 3) {
			retry_count++;
			NVT_ERR("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			NVT_ERR("Set default fw_ver=%d, x_num=%d, y_num=%d, \
					abs_x_max=%d, abs_y_max=%d, max_button_num=%d!\n",
					ts->fw_ver, ts->x_num, ts->y_num,
					ts->abs_x_max, ts->abs_y_max, ts->max_button_num);
			ret = -1;
		}
	} else {
		NVT_LOG("fw_ver=%02X\n", ts->fw_ver);
		ret = 0;
	}

	if ( ts->vendor_id == 0 ){
		sprintf(Ctp_name,"HLT,NT525B,FW:0x%x\n",ts->fw_ver);
	    sprintf(version,"HLT_NT525B_0x%02x",ts->fw_ver);
	    devinfo_info_tp_set(version, "HLT",OPLUS_BOOT_UPDATE_FIRMWARE_HLT_BOE_NAME);
		//rmtp = register_tp_proc("tp",version, "HLT",OPLUS_BOOT_UPDATE_FIRMWARE_NAME_HLT);
	} else if (ts->vendor_id == 2){
		sprintf(Ctp_name,"HLT_B3,NT525B,FW:0x%x\n",ts->fw_ver);
		sprintf(version,"HLT-B3_NT525B_0x%02x",ts->fw_ver);
		devinfo_info_tp_set(version, "HLT",OPLUS_BOOT_UPDATE_FIRMWARE_HLT_BOE_B3_NAME);
		//rmtp = register_tp_proc("tp",version, "HLT",OPLUS_BOOT_UPDATE_FIRMWARE_NAME_SK);
	} else {
		sprintf(Ctp_name,"HLT,NT525B,FW:0x%x\n",ts->fw_ver);
        sprintf(version,"HLT_NT525B_0x%02x",ts->fw_ver);
	    devinfo_info_tp_set(version, "HLT",OPLUS_BOOT_UPDATE_FIRMWARE_HLT_BOE_GG3_NAME);
		//rmtp = register_tp_proc("tp",version, "HLT",OPLUS_BOOT_UPDATE_FIRMWARE_NAME_SK);
	}

	//---Get Novatek PID---
	nvt_read_pid();

	return ret;
}

/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_PROC
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME	"NVTSPI"

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI read function.

return:
	Executive outcomes. 2---succeed. -5,-14---failed.
*******************************************************/
static ssize_t nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	uint8_t *str = NULL;
	int32_t ret = 0;
	int32_t retries = 0;
	int8_t spi_wr = 0;
	uint8_t *buf;

	if (count > NVT_TANSFER_LEN) {
		NVT_ERR("invalid transfer len!\n");
		return -EFAULT;
	}

	/* allocate buffer for spi transfer */
	str = (uint8_t *)kzalloc((count), GFP_KERNEL);
	if(str == NULL) {
		NVT_ERR("kzalloc for buf failed!\n");
		ret = -ENOMEM;
		goto kzalloc_failed;
	}

	buf = (uint8_t *)kzalloc((count), GFP_KERNEL | GFP_DMA);
	if(buf == NULL) {
		NVT_ERR("kzalloc for buf failed!\n");
		ret = -ENOMEM;
		kfree(str);
		str = NULL;
		goto kzalloc_failed;
	}

	if (copy_from_user(str, buff, count)) {
		NVT_ERR("copy from user error\n");
		ret = -EFAULT;
		goto out;
	}

#if NVT_TOUCH_ESD_PROTECT
	/*
	 * stop esd check work to avoid case that 0x77 report righ after here to enable esd check again
	 * finally lead to trigger esd recovery bootloader reset
	 */
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	spi_wr = str[0] >> 7;
	memcpy(buf, str+2, ((str[0] & 0x7F) << 8) | str[1]);

	if (spi_wr == NVTWRITE) {	//SPI write
		while (retries < 20) {
			ret = CTP_SPI_WRITE(ts->client, buf, ((str[0] & 0x7F) << 8) | str[1]);
			if (!ret)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			ret = -EIO;
			goto out;
		}
	} else if (spi_wr == NVTREAD) {	//SPI read
		while (retries < 20) {
			ret = CTP_SPI_READ(ts->client, buf, ((str[0] & 0x7F) << 8) | str[1]);
			if (!ret)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		memcpy(str+2, buf, ((str[0] & 0x7F) << 8) | str[1]);
		// copy buff to user if spi transfer
		if (retries < 20) {
			if (copy_to_user(buff, str, count)) {
				ret = -EFAULT;
				goto out;
			}
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			ret = -EIO;
			goto out;
		}
	} else {
		NVT_ERR("Call error, str[0]=%d\n", str[0]);
		ret = -EFAULT;
		goto out;
	}

out:
	kfree(str);
    kfree(buf);
kzalloc_failed:
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI open function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		NVT_ERR("Failed to allocate memory for nvt flash data\n");
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI close function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if (dev)
		kfree(dev);

	return 0;
}

static const struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.read = nvt_flash_read,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_proc_init(void)
{
	NVT_proc_entry = proc_create(DEVICE_NAME, 0444, NULL,&nvt_flash_fops);
	if (NVT_proc_entry == NULL) {
		NVT_ERR("Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("Succeeded!\n");
	}

	NVT_LOG("============================================================\n");
	NVT_LOG("Create /proc/NVTSPI\n");
	NVT_LOG("============================================================\n");

	return 0;
}
#endif

#if WAKEUP_GESTURE
enum {	/* oplus gesture type */
	UnkownGesture = 0,
	DouTap        = 1,
	UpVee,
	DownVee,
	LeftVee,	//>
	RightVee,	//<
	Circle,
	DouSwip,
	Right2LeftSwip,
	Left2RightSwip,
	Up2DownSwip,
	Down2UpSwip,
	Mgestrue,
	Wgestrue,
};

#define W_DETECT                13
#define UP_VEE_DETECT           14
#define DTAP_DETECT             15
#define M_DETECT                17
#define CIRCLE_DETECT           18
#define UP_SLIDE_DETECT         21
#define DOWN_SLIDE_DETECT       22
#define LEFT_SLIDE_DETECT       23
#define RIGHT_SLIDE_DETECT      24
#define LEFT_VEE_DETECT         31	//>
#define RIGHT_VEE_DETECT        32	//<
#define DOWN_VEE_DETECT         33
#define DOUSWIP_DETECT          34

/* customized gesture id */
#define DATA_PROTOCOL           30

/* function page definition */
#define FUNCPAGE_GESTURE         1

#ifdef ODM_WT_EDIT
//static struct wake_lock gestrue_wakelock;
#else
static struct wake_lock gestrue_wakelock;
#endif

/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
*******************************************************/
void nvt_ts_wakeup_gesture_report(uint8_t gesture_id, uint8_t *data, struct gesture_info *gesture)
{
	uint32_t keycode = 0;
	uint8_t func_type = data[2];
	uint8_t func_id = data[3];

	/* support fw specifal data protocol */
	if ((gesture_id == DATA_PROTOCOL) && (func_type == FUNCPAGE_GESTURE)) {
		gesture_id = func_id;
	} else if (gesture_id > DATA_PROTOCOL) {
		NVT_ERR("gesture_id %d is invalid, func_type=%d, func_id=%d\n", gesture_id, func_type, func_id);
		return;
	}

	NVT_LOG("gesture_id = %d\n", gesture_id);
	gesture->clockwise = 1;	//set default clockwise is 1.

	switch (gesture_id) {
		case RIGHT_SLIDE_DETECT :
			//gesture->gesture_type  = Left2RightSwip;
			gesture->gesture_type  = Right2LeftSwip;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_end.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			break;

		case LEFT_SLIDE_DETECT :
			//gesture->gesture_type  = Right2LeftSwip;
			gesture->gesture_type  = Left2RightSwip;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_end.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			break;

		case DOWN_SLIDE_DETECT  :
			gesture->gesture_type  = Up2DownSwip;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_end.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			break;

		case UP_SLIDE_DETECT :
			gesture->gesture_type  = Down2UpSwip;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_end.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			break;

		case DTAP_DETECT:
			gesture->gesture_type  = DouTap;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end     = gesture->Point_start;
			break;

		case UP_VEE_DETECT :
			gesture->gesture_type  = UpVee;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[12] & 0xFF) | (data[13] & 0x0F) << 8;
			gesture->Point_end.y   = (data[14] & 0xFF) | (data[15] & 0x0F) << 8;
			gesture->Point_1st.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_1st.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			break;

		case DOWN_VEE_DETECT :
			gesture->gesture_type  = DownVee;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[12] & 0xFF) | (data[13] & 0x0F) << 8;
			gesture->Point_end.y   = (data[14] & 0xFF) | (data[15] & 0x0F) << 8;
			gesture->Point_1st.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_1st.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			break;

		case LEFT_VEE_DETECT:
			gesture->gesture_type = LeftVee;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[12] & 0xFF) | (data[13] & 0x0F) << 8;
			gesture->Point_end.y   = (data[14] & 0xFF) | (data[15] & 0x0F) << 8;
			gesture->Point_1st.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_1st.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			break;

		case RIGHT_VEE_DETECT:
			gesture->gesture_type  = RightVee;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[12] & 0xFF) | (data[13] & 0x0F) << 8;
			gesture->Point_end.y   = (data[14] & 0xFF) | (data[15] & 0x0F) << 8;
			gesture->Point_1st.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_1st.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			break;

		case CIRCLE_DETECT:
			gesture->gesture_type = Circle;
			gesture->clockwise = (data[43] == 0x20) ? 1 : 0;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_1st.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;    //ymin
			gesture->Point_1st.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			gesture->Point_2nd.x   = (data[12] & 0xFF) | (data[13] & 0x0F) << 8;  //xmin
			gesture->Point_2nd.y   = (data[14] & 0xFF) | (data[15] & 0x0F) << 8;
			gesture->Point_3rd.x   = (data[16] & 0xFF) | (data[17] & 0x0F) << 8;  //ymax
			gesture->Point_3rd.y   = (data[18] & 0xFF) | (data[19] & 0x0F) << 8;
			gesture->Point_4th.x   = (data[20] & 0xFF) | (data[21] & 0x0F) << 8;  //xmax
			gesture->Point_4th.y   = (data[22] & 0xFF) | (data[23] & 0x0F) << 8;
			gesture->Point_end.x   = (data[24] & 0xFF) | (data[25] & 0x0F) << 8;
			gesture->Point_end.y   = (data[26] & 0xFF) | (data[27] & 0x0F) << 8;
			break;

		case DOUSWIP_DETECT:
			gesture->gesture_type  = DouSwip;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[12] & 0xFF) | (data[13] & 0x0F) << 8;
			gesture->Point_end.y   = (data[14] & 0xFF) | (data[15] & 0x0F) << 8;
			gesture->Point_1st.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_1st.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			gesture->Point_2nd.x   = (data[16] & 0xFF) | (data[17] & 0x0F) << 8;
			gesture->Point_2nd.y   = (data[18] & 0xFF) | (data[19] & 0x0F) << 8;
			break;

		case M_DETECT:
			gesture->gesture_type  = Mgestrue;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_1st.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_1st.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			gesture->Point_2nd.x   = (data[12] & 0xFF) | (data[13] & 0x0F) << 8;
			gesture->Point_2nd.y   = (data[14] & 0xFF) | (data[15] & 0x0F) << 8;
			gesture->Point_3rd.x   = (data[16] & 0xFF) | (data[17] & 0x0F) << 8;
			gesture->Point_3rd.y   = (data[18] & 0xFF) | (data[19] & 0x0F) << 8;
			gesture->Point_end.x   = (data[20] & 0xFF) | (data[21] & 0x0F) << 8;
			gesture->Point_end.y   = (data[22] & 0xFF) | (data[23] & 0x0F) << 8;
			break;

		case W_DETECT:
			gesture->gesture_type  = Wgestrue;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_1st.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_1st.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			gesture->Point_2nd.x   = (data[12] & 0xFF) | (data[13] & 0x0F) << 8;
			gesture->Point_2nd.y   = (data[14] & 0xFF) | (data[15] & 0x0F) << 8;
			gesture->Point_3rd.x   = (data[16] & 0xFF) | (data[17] & 0x0F) << 8;
			gesture->Point_3rd.y   = (data[18] & 0xFF) | (data[19] & 0x0F) << 8;
			gesture->Point_end.x   = (data[20] & 0xFF) | (data[21] & 0x0F) << 8;
			gesture->Point_end.y   = (data[22] & 0xFF) | (data[23] & 0x0F) << 8;
			break;

		default:
			gesture->gesture_type = UnkownGesture;
			break;
	}

	NVT_LOG("gesture_id: 0x%x, func_type: 0x%x, gesture_type: %d, clockwise: %d, points: (%d, %d)(%d, %d) (%d, %d)(%d, %d)(%d, %d)(%d, %d)\n",
			gesture_id, func_type, gesture->gesture_type, gesture->clockwise,
			gesture->Point_start.x, gesture->Point_start.y,
			gesture->Point_end.x, gesture->Point_end.y,
			gesture->Point_1st.x, gesture->Point_1st.y,
			gesture->Point_2nd.x, gesture->Point_2nd.y,
			gesture->Point_3rd.x, gesture->Point_3rd.y,
			gesture->Point_4th.x, gesture->Point_4th.y);

	if (gesture->gesture_type != UnkownGesture) {
		keycode = KEY_F4;

		input_report_key(ts->input_dev, keycode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keycode, 0);
		input_sync(ts->input_dev);
	}

}
#endif

/*******************************************************
Description:
	Novatek touchscreen parse device tree function.

return:
	n.a.
*******************************************************/
#ifdef CONFIG_OF
static int32_t nvt_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int32_t ret = 0;

#if NVT_TOUCH_SUPPORT_HW_RST
	ts->reset_gpio = of_get_named_gpio_flags(np, "novatek,reset-gpio", 0, &ts->reset_flags);
	NVT_LOG("novatek,reset-gpio=%d\n", ts->reset_gpio);
	ts->reset_global_gpio = of_get_named_gpio_flags(np, "novatek,reset_global_gpio", 0, &ts->reset_global_flags);
	NVT_LOG("novatek,reset_global_gpio=%d\n", ts->reset_global_gpio);
#endif
	ts->irq_gpio = of_get_named_gpio_flags(np, "novatek,irq-gpio", 0, &ts->irq_flags);
	NVT_LOG("novatek,irq-gpio=%d\n", ts->irq_gpio);

	ret = of_property_read_u32(np, "novatek,swrst-n8-addr", &SWRST_N8_ADDR);
	if (ret) {
		NVT_ERR("error reading novatek,swrst-n8-addr. ret=%d\n", ret);
		return ret;
	} else {
		NVT_LOG("SWRST_N8_ADDR=0x%06X\n", SWRST_N8_ADDR);
	}

	return ret;
}
#else
static int32_t nvt_parse_dt(struct device *dev)
{
#if NVT_TOUCH_SUPPORT_HW_RST
	ts->reset_gpio = NVTTOUCH_RST_PIN;
#endif
	ts->irq_gpio = NVTTOUCH_INT_PIN;
	return 0;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen config and request gpio

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int nvt_gpio_config(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

#if NVT_TOUCH_SUPPORT_HW_RST
	/* request RST-pin (Output/High) */
	if (gpio_is_valid(ts->reset_gpio)) {
		ret = gpio_request_one(ts->reset_gpio, GPIOF_OUT_INIT_LOW, "NVT-tp-rst");
		if (ret) {
			NVT_ERR("Failed to request NVT-tp-rst GPIO\n");
			goto err_request_reset_gpio;
		}
	}
#endif

	/* request INT-pin (Input) */
	if (gpio_is_valid(ts->irq_gpio)) {
		ret = gpio_request_one(ts->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret) {
			NVT_ERR("Failed to request NVT-int GPIO\n");
			goto err_request_irq_gpio;
		}
	}

	return ret;

err_request_irq_gpio:
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_free(ts->reset_gpio);
err_request_reset_gpio:
#endif
	return ret;
}
void nvt_mode_change_cmd(uint8_t cmd)
{
    int32_t i, retry = 5;
    uint8_t buf[3] = {0};

	//---set cmd status---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = cmd;
	CTP_SPI_WRITE(ts->client, buf, 2);

    for (i = 0; i < retry; i++) {
        msleep(20);

        //---read cmd status---
        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = 0xFF;
        CTP_SPI_READ(ts->client, buf, 2);
        if (buf[1] == 0x00)
            break;
    }

    if (i == retry) {
        NVT_ERR("send Cmd 0x%02X failed, buf[1]=0x%02X\n", cmd, buf[1]);
    } else {
        NVT_LOG("send Cmd 0x%02X success, tried %d times\n", cmd, i);
    }

    return;
}

static int32_t nvt_cmd_store(uint8_t cmd)
{
    int32_t i, retry = 5;
    uint8_t buf[3] = {0};

    //---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

    for (i = 0; i < retry; i++) {
        //---set cmd status---
        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = cmd;
        CTP_SPI_WRITE(ts->client, buf, 2);

        msleep(20);

        //---read cmd status---
        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = 0xFF;
        CTP_SPI_READ(ts->client, buf, 2);
        if (buf[1] == 0x00)
            break;
    }

    if (i == retry) {
        NVT_ERR("send Cmd 0x%02X failed, buf[1]=0x%02X\n", cmd, buf[1]);
        return -1;
    } else {
        NVT_LOG("send Cmd 0x%02X success, tried %d times\n", cmd, i);
    }

    return 0;
}
static int32_t nvt_enable_edge_limit(uint8_t state)
{
    int32_t ret = -1;

    NVT_LOG("state = %d\n", state);

    if (state == 0)
        ret = nvt_cmd_store(HOST_CMD_EDGE_LIMIT_VERTICAL);
    else if (state == 1)
        ret = nvt_cmd_store(HOST_CMD_EDGE_LIMIT_RIGHT_UP);
    else if (state == 2)
        ret = nvt_cmd_store(HOST_CMD_EDGE_LIMIT_LEFT_UP);
	else
		NVT_ERR("unknown state %d\n", state);

    return ret;
}
static int32_t nvt_enable_headset_mode(bool enable)
{
    int32_t ret = -1;

    NVT_LOG("%s:enable = %d\n", __func__, enable);

    if (enable)
        ret = nvt_cmd_store(HOST_CMD_HEADSET_PLUG_IN);
    else
        ret = nvt_cmd_store(HOST_CMD_HEADSET_PLUG_OUT);

    return ret;
}

static int32_t nvt_enable_charge_mode(bool enable)
{
    int32_t ret = -1;

    NVT_LOG("%s:enable = %d\n", __func__, enable);

    if (enable)
        ret = nvt_cmd_store(HOST_CMD_PWR_PLUG_IN);
    else
        ret = nvt_cmd_store(HOST_CMD_PWR_PLUG_OUT);

    return ret;
}

static int32_t nvt_enable_jitter_mode(bool enable)
{
    int32_t ret = -1;

    NVT_LOG("%s:enable = %d\n", __func__, enable);

    if (enable)
        ret = nvt_cmd_store(HOST_CMD_JITTER_ON);
    else
        ret = nvt_cmd_store(HOST_CMD_JITTER_OFF);

    return ret;
}

#ifdef ODM_WT_EDIT
int32_t nvt_enable_hopping_polling_mode(bool enable)
{
    int32_t ret = -1;

    NVT_LOG("%s:enable = %d\n", __func__, enable);

    if (enable)
        ret = nvt_cmd_store(HOST_CMD_HOPPING_POLLING_ON);
    else
        ret = nvt_cmd_store(HOST_CMD_HOPPING_POLLING_OFF);

    return ret;
}
int32_t nvt_enable_hopping_fix_freq_mode(bool enable)
{
    int32_t ret = -1;

    NVT_LOG("%s:enable = %d\n", __func__, enable);

    if (enable)
        ret = nvt_cmd_store(HOST_CMD_HOPPING_FIX_FREQ_ON);
    else
        ret = nvt_cmd_store(HOST_CMD_HOPPING_FIX_FREQ_OFF);

    return ret;
}


#endif

int32_t nvt_mode_switch(NVT_CUSTOMIZED_MODE mode, uint8_t flag)
{
	int32_t ret = -1;

	switch(mode) {
		case MODE_EDGE:
			ret = nvt_enable_edge_limit(flag);
			if (ret < 0) {
				NVT_ERR("%s: nvt enable edg limit failed.\n", __func__);
				return ret;
			}
			break;

		case MODE_CHARGE:
			ret = nvt_enable_charge_mode(flag);
			if (ret < 0) {
				NVT_ERR("%s: enable charge mode : %d failed\n", __func__, flag);
			}
			break;

		case MODE_GAME:
			ret = nvt_enable_jitter_mode(flag);
			if (ret < 0) {
				NVT_ERR("enable jitter mode : %d failed\n", flag);
			}
			break;

		case MODE_HEADSET:
			ret = nvt_enable_headset_mode(flag);
			if (ret < 0) {
				NVT_ERR("enable headset mode : %d failed\n", flag);
			}
			break;
		default:
			NVT_ERR("%s: Wrong mode %d.\n", __func__, mode);
	}

	return ret;
}

#if NVT_TOUCH_ESD_PROTECT
void nvt_esd_check_enable(uint8_t enable)
{
	/* update interrupt timer */
	irq_timer = jiffies;
	/* clear esd_retry counter, if protect function is enabled */
	esd_retry = enable ? 0 : esd_retry;
	/* enable/disable esd check flag */
	esd_check = enable;
}

static uint8_t nvt_fw_recovery(uint8_t *point_data)
{
	uint8_t i = 0;
	uint8_t detected = true;

	/* check pattern */
	for (i=1 ; i<7 ; i++) {
		if (point_data[i] != 0x77) {
			detected = false;
			break;
		}
	}

	return detected;
}

static void nvt_esd_check_func(struct work_struct *work)
{
	unsigned int timer = jiffies_to_msecs(jiffies - irq_timer);

	//NVT_LOG("esd_check = %d (retry %d)\n", esd_check, esd_retry);	//DEBUG

	if ((timer > NVT_TOUCH_ESD_CHECK_PERIOD) && esd_check) {
		mutex_lock(&ts->lock);
		NVT_ERR("do ESD recovery, timer = %d, retry = %d\n", timer, esd_retry);
		/* do esd recovery, reload fw */
#ifdef ODM_WT_EDIT
		nvt_update_firmware(fw->firmware_name);
#endif
		mutex_unlock(&ts->lock);
		/* update interrupt timer */
		irq_timer = jiffies;
		/* update esd_retry counter */
		esd_retry++;
	}

	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

static bool nvt_corner_point_process(int i)
{
    int j;
	int flag = 0;
	if (ts->nvt_oplus_proc_data->edge_limit.limit_00 == 0) {
		//节点/proc/touchpanel/oplus_tp_limit_enable的bit1位来控制控制
		if ((ts->nvt_oplus_proc_data->edge_limit.limit_lu) &&
			(ts->nvt_oplus_proc_data->nvt_point_info[i].x < ts->nvt_oplus_proc_data->nvt_limit_area.area_xlu &&
			ts->nvt_oplus_proc_data->nvt_point_info[i].y < ts->nvt_oplus_proc_data->nvt_limit_area.area_ylu)) {

		    ts->nvt_oplus_proc_data->nvt_point_info[i].type  = NVT_AREA_CORNER;
			if (ts->nvt_oplus_proc_data->nvt_limit_area.which_area == NVT_AREA_NORMAL)
				return true;
		    ts->nvt_oplus_proc_data->nvt_coner_info[CORNER_TOPLEFT].id = i;
		    ts->nvt_oplus_proc_data->nvt_coner_info[CORNER_TOPLEFT].point_info = ts->nvt_oplus_proc_data->nvt_point_info[i];
		    ts->nvt_oplus_proc_data->nvt_coner_info[CORNER_TOPLEFT].flag = true;
			ts->nvt_oplus_proc_data->nvt_limit_area.which_area = ts->nvt_oplus_proc_data->nvt_point_info[i].type;
			flag = 1;

        }
         //节点/proc/touchpanel/oplus_tp_limit_enable的bit2位来控制控制
        if ((ts->nvt_oplus_proc_data->edge_limit.limit_ru)  &&
			(ts->nvt_oplus_proc_data->nvt_point_info[i].x > ts->nvt_oplus_proc_data->nvt_limit_area.area_xru &&
			ts->nvt_oplus_proc_data->nvt_point_info[i].y < ts->nvt_oplus_proc_data->nvt_limit_area.area_yru)) {

			ts->nvt_oplus_proc_data->nvt_point_info[i].type  = NVT_AREA_CORNER;
			if (ts->nvt_oplus_proc_data->nvt_limit_area.which_area == NVT_AREA_NORMAL)
				return true;
			ts->nvt_oplus_proc_data->nvt_limit_area.which_area = ts->nvt_oplus_proc_data->nvt_point_info[i].type;
			ts->nvt_oplus_proc_data->nvt_coner_info[CORNER_TOPRIGHT].id = i;
			ts->nvt_oplus_proc_data->nvt_coner_info[CORNER_TOPRIGHT].point_info = ts->nvt_oplus_proc_data->nvt_point_info[i];
			ts->nvt_oplus_proc_data->nvt_coner_info[CORNER_TOPRIGHT].flag = true;
			flag = 1;

        }
         //节点/proc/touchpanel/oplus_tp_limit_enable的bit3位来控制控制
       if ((ts->nvt_oplus_proc_data->edge_limit.limit_lb) &&
	   	(ts->nvt_oplus_proc_data->nvt_point_info[i].x < ts->nvt_oplus_proc_data->nvt_limit_area.area_xlb &&
	   	ts->nvt_oplus_proc_data->nvt_point_info[i].y > ts->nvt_oplus_proc_data->nvt_limit_area.area_ylb)) {

            ts->nvt_oplus_proc_data->nvt_point_info[i].type  = NVT_AREA_CORNER;
			if (ts->nvt_oplus_proc_data->nvt_limit_area.which_area == NVT_AREA_NORMAL)
				return true;
  			ts->nvt_oplus_proc_data->nvt_limit_area.which_area = ts->nvt_oplus_proc_data->nvt_point_info[i].type;
   			ts->nvt_oplus_proc_data->nvt_coner_info[CORNER_BOTTOMLEFT].id = i;
   			ts->nvt_oplus_proc_data->nvt_coner_info[CORNER_BOTTOMLEFT].point_info = ts->nvt_oplus_proc_data->nvt_point_info[i];
   			ts->nvt_oplus_proc_data->nvt_coner_info[CORNER_BOTTOMLEFT].flag = true;
			flag = 1;

        }
         //节点/proc/touchpanel/oplus_tp_limit_enable的bit4位来控制控制
       if ((ts->nvt_oplus_proc_data->edge_limit.limit_rb) &&
	   	(ts->nvt_oplus_proc_data->nvt_point_info[i].x > ts->nvt_oplus_proc_data->nvt_limit_area.area_xrb &&
	   	ts->nvt_oplus_proc_data->nvt_point_info[i].y > ts->nvt_oplus_proc_data->nvt_limit_area.area_yrb)) {

			ts->nvt_oplus_proc_data->nvt_point_info[i].type  = NVT_AREA_CORNER;
			if (ts->nvt_oplus_proc_data->nvt_limit_area.which_area == NVT_AREA_NORMAL)
				return true;

			ts->nvt_oplus_proc_data->nvt_limit_area.which_area = ts->nvt_oplus_proc_data->nvt_point_info[i].type;
			ts->nvt_oplus_proc_data->nvt_coner_info[CORNER_BOTTOMRIGHT].id = i;
			ts->nvt_oplus_proc_data->nvt_coner_info[CORNER_BOTTOMRIGHT].point_info = ts->nvt_oplus_proc_data->nvt_point_info[i];
			ts->nvt_oplus_proc_data->nvt_coner_info[CORNER_BOTTOMRIGHT].flag = true;
			flag = 1;

        }
        //坐标点为非边角区域时，弹起前面记录的边角坐标点
        if (ts->nvt_oplus_proc_data->nvt_point_info[i].type != NVT_AREA_CORNER) {

            if (ts->nvt_oplus_proc_data->nvt_limit_area.which_area == NVT_AREA_CORNER) {
                for (j = 0; j < 4; j++) {
                    if (ts->nvt_oplus_proc_data->nvt_coner_info[j].flag) {
#ifdef MT_PROTOCOL_B
                        input_mt_slot(ts->input_dev, ts->nvt_oplus_proc_data->nvt_coner_info[j].id);
                        input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
#endif
                    }
                }
            }
        }
		if ( flag == 0 ) {
			ts->nvt_oplus_proc_data->nvt_limit_area.which_area = NVT_AREA_NORMAL;
		} else {
			ts->nvt_oplus_proc_data->nvt_limit_area.which_area = NVT_AREA_CORNER;
		}
    }

    return false;
}


#if NVT_TOUCH_WDT_RECOVERY
static uint8_t recovery_cnt = 0;
static uint8_t nvt_wdt_fw_recovery(uint8_t *point_data)
{
   uint32_t recovery_cnt_max = 10;
   uint8_t recovery_enable = false;
   uint8_t i = 0;

   recovery_cnt++;

   /* check pattern */
   for (i=1 ; i<7 ; i++) {
       if ((point_data[i] != 0xFD) && (point_data[i] != 0xFE)) {
           recovery_cnt = 0;
           break;
       }
   }

   if (recovery_cnt > recovery_cnt_max){
       recovery_enable = true;
       recovery_cnt = 0;
   }

   return recovery_enable;
}
#endif	/* #if NVT_TOUCH_WDT_RECOVERY */

#define POINT_DATA_LEN 78
uint8_t last_st = 0;
uint8_t last_finger_cnt = 0;
uint32_t finger_last = 0;

/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
uint32_t input_p_bak = 0;
int32_t frame = 60;
static void nvt_ts_work_func(void)
{
	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + 1 + DUMMY_BYTES] = {0};//spi_read_write,t.len    = (len + DUMMY_BYTES);
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint32_t input_p = 0;
	uint8_t input_id = 0;
#if MT_PROTOCOL_B
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};
#endif /* MT_PROTOCOL_B */
	int32_t i = 0;
	int32_t finger_cnt = 0;
	if (ts->nvt_oplus_proc_data == NULL) {
		ts->nvt_oplus_proc_data = kzalloc(sizeof(struct nvt_oplus_data), GFP_KERNEL);
		if (ts->nvt_oplus_proc_data == NULL) {
			NVT_LOG("alloc the ts->nvt_oplus_proc_data memery fail \n");
			return;
		}
	}

	mutex_lock(&ts->lock);
	
	ret = CTP_SPI_READ(ts->client, point_data, POINT_DATA_LEN + 1);
	if (ret < 0) {
		NVT_ERR("CTP_SPI_READ failed.(%d)\n", ret);
		goto XFER_ERROR;
	}
/*
	//--- dump SPI buf ---
	for (i = 0; i < 10; i++) {
		printk("%02X %02X %02X %02X %02X %02X  ", point_data[1+i*6], point_data[2+i*6], point_data[3+i*6], point_data[4+i*6], point_data[5+i*6], point_data[6+i*6]);
	}
	printk("\n");
*/
#if NVT_TOUCH_WDT_RECOVERY
   /* ESD protect by WDT */
   if (nvt_wdt_fw_recovery(point_data)) {
       NVT_ERR("Recover for fw reset, %02X\n", point_data[1]);
#ifdef ODM_WT_EDIT
       nvt_update_firmware(fw->firmware_name);
#endif
       goto XFER_ERROR;
   }
#endif /* #if NVT_TOUCH_WDT_RECOVERY */
#if NVT_TOUCH_ESD_PROTECT
	/* ESD protect by FW handshake */
	if (nvt_fw_recovery(point_data)) {
		nvt_esd_check_enable(true);
		goto XFER_ERROR;
	}
#endif /* #if NVT_TOUCH_ESD_PROTECT */
#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {
		input_id = (uint8_t)(point_data[1] >> 3);
		memset(&ts->gesture, 0, sizeof(struct gesture_info));
		nvt_ts_wakeup_gesture_report(input_id, point_data, &ts->gesture);
		//enable_irq(ts->client->irq);
		mutex_unlock(&ts->lock);
		return;
	}
#endif

	finger_cnt = 0;
	ts->nvt_oplus_proc_data->nvt_limit_area.which_area = NVT_AREA_DEFAULT;
	for (i = 0; i < ts->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if ((input_id == 0) || (input_id > ts->max_touch_num))
			continue;

		if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
#if NVT_TOUCH_ESD_PROTECT
			/* update interrupt timer */
			irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
			input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
			input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
			if ((input_x < 0) || (input_y < 0))
				continue;
			if ((input_x > ts->abs_x_max) || (input_y > ts->abs_y_max))
				continue;
			input_w = (uint32_t)(point_data[position + 4]);
			if (input_w < 12)
				input_w = 5;
			if (input_w == 0)
				input_w = 1;
			if (i < 2) {
				input_p = (uint32_t)(point_data[position + 5]) + (uint32_t)(point_data[i + 63] << 8);
				if (input_p > TOUCH_FORCE_NUM)
					input_p = TOUCH_FORCE_NUM;
				frame--;
				if(frame ==0){
					if(input_p_bak == input_p)
					input_p++;
#ifdef ODM_WT_EDIT
					input_w++;
#endif
					input_p_bak = input_p;
					frame = 60;
				}
			} else {
				input_p = (uint32_t)(point_data[position + 5]);
			}
			if (input_p == 0)
				input_p = 1;

#if MT_PROTOCOL_B
			press_id[input_id - 1] = 1;
			input_mt_slot(ts->input_dev, input_id - 1);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
#else /* MT_PROTOCOL_B */
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id - 1);
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif /* MT_PROTOCOL_B */

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_p);

#if MT_PROTOCOL_B
#else /* MT_PROTOCOL_B */
			input_mt_sync(ts->input_dev);
#endif /* MT_PROTOCOL_B */

			finger_cnt++;

			/* backup oplus debug coordinate info */
			finger_last = input_id - 1;
			ts->oplus_debug_info.coordinate[input_id-1].x = (uint16_t) input_x;
			ts->oplus_debug_info.coordinate[input_id-1].y = (uint16_t) input_y;

			ts->nvt_oplus_proc_data->nvt_point_info[i].x = (uint16_t) input_x;
			ts->nvt_oplus_proc_data->nvt_point_info[i].y = (uint16_t) input_y;
			ts->nvt_oplus_proc_data->nvt_point_info[i].type = NVT_AREA_NORMAL;
			if(ts->nvt_oplus_proc_data->edge_limit.limit_lu == 1 || ts->nvt_oplus_proc_data->edge_limit.limit_ru == 1 ||
				ts->nvt_oplus_proc_data->edge_limit.limit_lb == 1 || ts->nvt_oplus_proc_data->edge_limit.limit_rb == 1) {
				nvt_corner_point_process(i);
			}


		}
	}

#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		if (press_id[i] != 1) {
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}
	}

	input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));
#else /* MT_PROTOCOL_B */
	if (finger_cnt == 0) {
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_mt_sync(ts->input_dev);
	}
#endif /* MT_PROTOCOL_B */

#if TOUCH_KEY_NUM > 0
	if (point_data[61] == 0xF8) {
#if NVT_TOUCH_ESD_PROTECT
		/* update interrupt timer */
		ts->irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i], ((point_data[62] >> i) & 0x01));
		}
	} else {
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i], 0);
		}
	}
#endif

	input_sync(ts->input_dev);

	/* avoid to print invaild message */
	if ((last_finger_cnt == finger_cnt) && (finger_cnt == 0)) {
		goto NO_DEBUG;
	}

	for (i = 0; i < ts->max_touch_num; i++) {
		if ((ts->debug_level == 1) || (ts->debug_level == 2)) {
			/* debug touch coordinate info */
			if ((press_id[0] == 1) && (last_st == 0)) {	//finger down
				last_st = press_id[0];
				NVT_LOG("Touchpanel id %d :Down [%4d, %4d]\n",
						i,
						ts->oplus_debug_info.coordinate[0].x,
						ts->oplus_debug_info.coordinate[0].y);
			}
			else if ((finger_cnt == 0) && (i == finger_last)) { //finger up
				finger_last = 0;
				last_st = press_id[0];
				NVT_LOG("Touchpanel id %d :Up   [%4d, %4d]\n",
						i,
						ts->oplus_debug_info.coordinate[i].x,
						ts->oplus_debug_info.coordinate[i].y);
			}
		}

		if (ts->debug_level == 2) {
			if (press_id[i] == 1) {
				NVT_LOG("Touchpanel id %d :     [%4d, %4d]\n",
						i,
						ts->oplus_debug_info.coordinate[i].x,
						ts->oplus_debug_info.coordinate[i].y);
			}
		}
	}

NO_DEBUG:
	/* backup finger_cnt */
	last_finger_cnt = finger_cnt;

XFER_ERROR:
	//enable_irq(ts->client->irq);

	mutex_unlock(&ts->lock);
}

/*******************************************************
Description:
	External interrupt service routine.

return:
	irq execute status.
*******************************************************/
static irqreturn_t nvt_ts_irq_handler(int32_t irq, void *dev_id)
{

#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {
#ifdef ODM_WT_EDIT
		pm_wakeup_event(&ts->input_dev->dev, 5000);
#else
		wake_lock_timeout(&gestrue_wakelock, msecs_to_jiffies(5000));
#endif
	}
#endif
	//queue_work(nvt_wq, &ts->nvt_work);
	nvt_ts_work_func();
	return IRQ_HANDLED;
}

/*******************************************************
Description:
	Novatek touchscreen check chip version trim function.

return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
	int32_t list = 0;
	int32_t i = 0;
	int32_t found_nvt_chip = 0;
	int32_t ret = -1;

	//---Check for 5 times---
	for (retry = 5; retry > 0; retry--) {

		nvt_bootloader_reset();

		//---set xdata index to 0x1F600---
		nvt_set_page(0x1F600);

		buf[0] = 0x4E;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_SPI_READ(ts->client, buf, 7);
		NVT_LOG("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
			buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		// compare read chip id on supported list
		for (list = 0; list < (sizeof(trim_id_table) / sizeof(struct nvt_ts_trim_id_table)); list++) {
			found_nvt_chip = 0;

			// compare each byte
			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				if (trim_id_table[list].mask[i]) {
					if (buf[i + 1] != trim_id_table[list].id[i])
						break;
				}
			}

			if (i == NVT_ID_BYTE_MAX) {
				found_nvt_chip = 1;
			}

			if (found_nvt_chip) {
				NVT_LOG("This is NVT touch IC ,list = %d \n",list);
				ts->mmap = trim_id_table[list].mmap;
				ts->carrier_system = trim_id_table[list].hwinfo->carrier_system;
				ts->hw_crc = trim_id_table[list].hwinfo->hw_crc;
				ret = 0;
				goto out;
			} else {
				ts->mmap = NULL;
				ret = -1;
			}
		}

		msleep(10);
	}

out:
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen for product check
return:
	touchscreen version and lockdown info
********************************************************/
static int __maybe_unused tid_get_version(struct device *dev, int *major, int *minor)
{
	*major = 0;
	*minor = ts->fw_ver;
	return 0;
}
static int __maybe_unused tid_get_lockdown_info(struct device *dev, char *out_values)
{
	if(ts->vendor_id ==0){
		out_values[LOCKDOWN_INFO_PANEL_MAKER_INDEX] = 0x50;
	}else{
		out_values[LOCKDOWN_INFO_PANEL_MAKER_INDEX] = 0x55;
	}
	return 0;
}


static void nvt_resume_workqueue(struct work_struct *work)
{
#ifdef ODM_WT_EDIT
//	int pwr;
#endif

	mutex_lock(&ts->lock);
	ts->is_suspended = 0;

	if (bTouchIsAwake) {
		NVT_LOG("Touch is already resume\n");
		mutex_unlock(&ts->lock);
		return;
	}

	NVT_LOG("start\n");

	// please make sure display reset(RESX) sequence and mipi dsi cmds sent before this
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_set_value(ts->reset_gpio, 1);
#endif
#ifdef ODM_WT_EDIT
	nvt_update_firmware(fw->firmware_name);
#endif
	nvt_check_fw_reset_state(RESET_STATE_REK);

#if WAKEUP_GESTURE
	if (!ts->gesture_enable) {
		//enable_irq(ts->client->irq);
		nvt_irq_enable(1);
#ifdef ODM_WT_EDIT
//		pwr = nvt_power_on(ts, true);
//		if(pwr != 0)
//			NVT_ERR("nvt_power_on failed\n");
//		power_flag = 1;
#endif
	}
#else
	//enable_irq(ts->client->irq);
	nvt_irq_enable(1);
#endif
#ifdef ODM_WT_EDIT
//	if(power_flag == 0){
//		pwr = nvt_power_on(ts, true);
//		if(pwr != 0)
//			NVT_ERR("nvt_power_on failed\n");
//		power_flag = 1;
//	}
#endif
	nvt_irq_enable(1);
#if NVT_TOUCH_ESD_PROTECT
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */
	if (nvt_tp_usb_state == 1) {
		if (nvt_mode_switch(MODE_CHARGE, true)) {
				NVT_LOG("%s:usb plug in fail\n",__func__);
		}
	}
	if (nvt_tp_headset_state == 1) {
		if (nvt_mode_switch(MODE_HEADSET,true)) {
				NVT_LOG("%s:headset plug in fail\n",__func__);
		}
	}

	bTouchIsAwake = 1;

	mutex_unlock(&ts->lock);

}
void nvt_lcd_resume_func(void)
{
	schedule_work(&ts->resume_work_queue);
}

void nvt_lcd_esd_resume_func(void)
{
	//	schedule_work(&ts->resume_work_queue);
	mutex_lock(&ts->lock);
#if NVT_TOUCH_ESD_PROTECT
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	NVT_LOG("do ESD recovery, Begain to load firmware");
	/* do esd recovery, reload fw */
#ifdef ODM_WT_EDIT
	nvt_update_firmware(fw->firmware_name);
#endif
	nvt_check_fw_reset_state(RESET_STATE_REK);

	if (nvt_tp_usb_state == 1) {
		if (nvt_mode_switch(MODE_CHARGE, true)) {
				NVT_LOG("%s:plug in fail\n",__func__);
		}
	}
#if NVT_TOUCH_ESD_PROTECT
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */
	mutex_unlock(&ts->lock);
}

static int nvt_notifie_update_fw(struct notifier_block *nb,unsigned long value, void *data)
{
	unsigned long update_fw_state = value;
	NVT_LOG("update_fw_state == %ld \n",update_fw_state);
	if(update_fw_state == 1) {
		nvt_lcd_resume_func();
	}else if(update_fw_state == 2){
		nvt_lcd_esd_resume_func();
	}else
		NVT_ERR("invalid update_fw_state == %ld \n",update_fw_state);
	return 0;
}

static void nvt_update_fw_notifier_init(void)
{
	int res = 0;

	NVT_LOG("nvt_update_fw_notifier_init\n");

	ts->notifier_update_fw.notifier_call = nvt_notifie_update_fw;
	res = update_tpfw_register_client(&ts->notifier_update_fw);

}
static void nvt_tp_usb_workqueue(struct work_struct *work)
{
	int ret = 0;
	mutex_lock(&ts->lock);
	if(bTouchIsAwake == 1) {
		if (nvt_tp_usb_state == 1) {
			ret = nvt_mode_switch(MODE_CHARGE, true);
			if (ret) {
				NVT_LOG("%s:plug in fail\n",__func__);
			}
		}
		if (nvt_tp_usb_state == 0) {
			ret = nvt_mode_switch(MODE_CHARGE, false);
			if (ret) {
				NVT_LOG("%s:plug out fail\n",__func__);
			}
		}

	} else {

		NVT_LOG("%s:is suspended,do nothing",__func__);
	}
	mutex_unlock(&ts->lock);
	return;


}

void nvt_tp_usb_func(void)
{

	schedule_work(&ts->tp_usb_work_queue);

}

static int nvt_notifie_tp_usb(struct notifier_block *nb,unsigned long value, void *data)
{
	nvt_tp_usb_state = value;
	NVT_LOG("%s:nvt_tp_usb_state == %ld \n",__func__,nvt_tp_usb_state);

	nvt_tp_usb_func();

	return 0;
}
static void nvt_tp_usb_notifier_init(void)
{
	int res = 0;

	NVT_LOG("%s \n",__func__);

	ts->notifier_tp_usb.notifier_call = nvt_notifie_tp_usb;
	res = tp_usb_register_client(&ts->notifier_tp_usb);


}

static void nvt_tp_headset_workqueue(struct work_struct *work)
{
	int ret = 0;
	mutex_lock(&ts->lock);
	if(bTouchIsAwake == 1) {
		if (nvt_tp_headset_state == 1) {
			ret = nvt_mode_switch(MODE_HEADSET,true);
			if (ret) {
				NVT_LOG("%s:plug in fail\n",__func__);
			}
		}
		if (nvt_tp_headset_state == 0) {
			ret = nvt_mode_switch(MODE_HEADSET,false);
			if (ret) {
				NVT_LOG("%s:plug out fail\n",__func__);
			}
		}

	} else {

		NVT_LOG("%s:is suspended,do nothing",__func__);
	}
	mutex_unlock(&ts->lock);

	return;
}

void nvt_tp_headset_func(void)
{

	schedule_work(&ts->headset_work_queue);

}

static int nvt_notifie_tp_headset(struct notifier_block *nb,unsigned long value, void *data)
{
	nvt_tp_headset_state = value;
	NVT_LOG("%s:nvt_tp_headset_state == %ld \n",__func__,nvt_tp_headset_state);

	nvt_tp_headset_func();

	return 0;
}

static void nvt_tp_headset_notifier_init(void)
{
	int res = 0;

	NVT_LOG("%s \n",__func__);

	ts->notifier_headset.notifier_call = nvt_notifie_tp_headset;
	res = headset_register_client(&ts->notifier_headset);


}

/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
//extern char Lcm_name1[256];
static char Lcm_name1[256];
static int32_t nvt_ts_probe(struct spi_device *client)
{

#ifdef ODM_WT_EDIT
//	struct touch_info_dev *tid;
//	struct touch_info_dev_operations *tid_ops;
//	struct device *dev= &client->dev;
#endif
//	char *oplus_ftmmode_start;
	int32_t ret = 0;
#if (TOUCH_KEY_NUM > 0)
	int32_t retry = 0;
#endif

#ifdef ODM_WT_EDIT
	char *temp = NULL;
	//char * cmdline_tp = NULL;
	int nt_ctpmodule = 0;
#if 1
	temp = Lcm_name1;
	NVT_LOG("samir : %s",temp);
	nt_ctpmodule = strncmp(temp,"nt36525b_hlt_hdp_dsi_vdo_lcm_drv",strlen("nt36525b_hlt_hdp_dsi_vdo_lcm_drv"));
	nt_ctpmodule = 0;
	if ( nt_ctpmodule == 0 ) {
		NVT_LOG("this is hlt_boe nt36525b touchscreen\n");
	}else{
		nt_ctpmodule = strncmp(temp,"nt36525b_hlt_psc_ac_boe_vdo",strlen("nt36525b_hlt_psc_ac_boe_vdo"));
		if (nt_ctpmodule == 0) {
			NVT_LOG("this is hlt_boe_b3 nt36525b touchscreen\n");
			nt_ctpmodule = 2;
		} else {
			NVT_LOG("nothing match lcd name\n");
			return -1;
		}
	}
#endif
#if 0
	cmdline_tp = strstr(saved_command_line,"nt36525b_");
	NVT_LOG("cmdline_tp = %s\n",cmdline_tp);
	if ( cmdline_tp == NULL ){
		NVT_LOG("get nt36525b_ fail ");
		//return -1;
	}
	temp = cmdline_tp + strlen("nt36525b_");
	NVT_LOG("temp = %s\n",temp);
	nt_ctpmodule = strncmp(temp,"hlt_boe_video_display",strlen("hlt_boe_video_display"));
	if ( nt_ctpmodule == 0 ) {
		NVT_LOG("this is hlt_boe nt36525b touchscreen\n");
	}else{
		nt_ctpmodule == 0;
		NVT_LOG("this is another nt36525b touchscreen\n");
	}
#endif
#endif
	NVT_LOG("start\n");

	ts = kmalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		NVT_ERR("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}

	ts->xbuf = (uint8_t *)kzalloc((NVT_TANSFER_LEN+1), GFP_KERNEL);
	if(ts->xbuf == NULL) {
		NVT_ERR("kzalloc for xbuf failed!\n");
		if (ts) {
			kfree(ts);
			ts = NULL;
		}
		return -ENOMEM;
	}

	ts->client = client;
	spi_set_drvdata(client, ts);
#ifdef ODM_WT_EDIT
/*	tid = devm_tid_and_ops_allocate(dev);
	if (unlikely(!tid))
		return -ENOMEM;
	ts->tid = tid;
	tid_ops = tid->tid_ops;
	tid_ops->get_version		   = tid_get_version;
	tid_ops->get_lockdown_info 	   = tid_get_lockdown_info;
*/
	ts->oplus_baseline_test_flag = 0;
	ts->irq_enable_time = 1;
	ts->irq_enable_flag = 0;
	ts->vendor_id = nt_ctpmodule;
	NVT_LOG("vendor_id is %d",ts->vendor_id);

	fw = kzalloc(sizeof(nvt_fw_list[ts->vendor_id]),GFP_KERNEL);

	fw = &nvt_fw_list[ts->vendor_id];
	NVT_LOG("fw->id = %d,fw->module_vendor =%s,fw->firmware_name =%s, fw->firmware_mp_name = %s, fw->fw_len = %d, fw->firmware_sign_name = %s\n",
		fw->id,fw->module_vendor, fw->firmware_name, fw->firmware_mp_name,fw->fw_len, fw->firmware_sign_name);
#endif
	//---prepare for spi parameter---
	if (ts->client->master->flags & SPI_MASTER_HALF_DUPLEX) {
		NVT_ERR("Full duplex not supported by master\n");
		ret = -EIO;
		goto err_ckeck_full_duplex;
	}
	ts->client->bits_per_word = 8;
	ts->client->mode = SPI_MODE_0;

	ret = spi_setup(ts->client);
	if (ret < 0) {
		NVT_ERR("Failed to perform SPI setup\n");
		goto err_spi_setup;
	}
    
#ifdef CONFIG_MTK_SPI
    /* old usage of MTK spi API */
    memcpy(&ts->spi_ctrl, &spi_ctrdata, sizeof(struct mt_chip_conf));
    ts->client->controller_data = (void *)&ts->spi_ctrl;
#endif

#ifdef CONFIG_SPI_MT65XX
    /* new usage of MTK spi API */
    memcpy(&ts->spi_ctrl, &spi_ctrdata, sizeof(struct mtk_chip_config));
    ts->client->controller_data = (void *)&ts->spi_ctrl;
#endif

	NVT_LOG("mode=%d, max_speed_hz=%d\n", ts->client->mode, ts->client->max_speed_hz);

	//---parse dts---
	ret = nvt_parse_dt(&client->dev);
	if (ret) {
		NVT_ERR("parse dt error\n");
		goto err_spi_setup;
	}

	//---request and config GPIOs---
	ret = nvt_gpio_config(ts);
	if (ret) {
		NVT_ERR("gpio config error!\n");
		goto err_gpio_config_failed;
	}

#ifdef ODM_WT_EDIT
//	ret = power_init(ts, true);
//	if (ret) {
//		NVT_ERR("Novtek power init fail\n");
//	}
//	ret = nvt_power_on(ts, true);
//	if (ret) {
//		NVT_ERR("Novtek power on fail\n");
//	} else {
//		power_flag = 1;
//	}
#endif

	mutex_init(&ts->lock);
	mutex_init(&ts->xbuf_lock);

	//---eng reset before TP_RESX high
	nvt_eng_reset();

#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_set_value(ts->reset_gpio, 1);
#endif

	// need 10ms delay after POR(power on reset)
	msleep(10);

	//---check chip version trim---
	ret = nvt_ts_check_chip_ver_trim();
	if (ret) {
		NVT_ERR("chip is not identified\n");
		ret = -EINVAL;
		goto err_chipvertrim_failed;
	}

	ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
	ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;

	//---create workqueue---
	/*nvt_wq = create_workqueue("nvt_wq");
	if (!nvt_wq) {
		NVT_ERR("nvt_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_wq_failed;
	}
	INIT_WORK(&ts->nvt_work, nvt_ts_work_func);*/
	INIT_WORK(&ts->resume_work_queue, nvt_resume_workqueue);
	INIT_WORK(&ts->tp_usb_work_queue, nvt_tp_usb_workqueue);
	INIT_WORK(&ts->headset_work_queue, nvt_tp_headset_workqueue);

	//---allocate input device---
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		NVT_ERR("allocate input device failed\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

#if TOUCH_KEY_NUM > 0
	ts->max_button_num = TOUCH_KEY_NUM;
#endif

	ts->int_trigger_type = INT_TRIGGER_TYPE;


	//---set input device info.---
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

#if MT_PROTOCOL_B
	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM, 0, 0);    //pressure = TOUCH_FORCE_NUM

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max-1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max-1, 0, 0);
#if MT_PROTOCOL_B
	// no need to set ABS_MT_TRACKING_ID, input_mt_init_slots() already set it
#else
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif //MT_PROTOCOL_B
#endif //TOUCH_MAX_FINGER_NUM > 1

#if TOUCH_KEY_NUM > 0
	for (retry = 0; retry < ts->max_button_num; retry++) {
		input_set_capability(ts->input_dev, EV_KEY, touch_key_array[retry]);
	}
#endif

#if WAKEUP_GESTURE
	ts->gesture_enable = 0;
#ifdef ODM_WT_EDIT
	tp_gesture = 0;
#endif
	memset(&ts->gesture, 0, sizeof(struct gesture_info));
	input_set_capability(ts->input_dev, EV_KEY, KEY_F4);
#ifdef ODM_WT_EDIT
	//device_init_wakeup(&ts->input_dev->dev, true);
#else
	wake_lock_init(&gestrue_wakelock, WAKE_LOCK_SUSPEND, "poll-wake-lock");
#endif
#endif
	memset(&ts->oplus_debug_info, 0, sizeof(struct oplus_debug_info));

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_SPI;

	//---register input device---
	ret = input_register_device(ts->input_dev);
	if (ret) {
		NVT_ERR("register input device (%s) failed. ret=%d\n", ts->input_dev->name, ret);
		goto err_input_register_device_failed;
	}

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 1);
#endif

	//---set int-pin & request irq---
	spin_lock_init(&ts->spinlock_int);
	client->irq = gpio_to_irq(ts->irq_gpio);
	if (client->irq) {
		NVT_LOG("int_trigger_type=%d\n", ts->int_trigger_type);
#if WAKEUP_GESTURE
		ret = request_threaded_irq(client->irq, NULL, nvt_ts_irq_handler,
				ts->int_trigger_type | IRQF_ONESHOT, NVT_SPI_NAME, ts);
#else
		ret = request_threaded_irq(client->irq, NULL, nvt_ts_irq_handler,
				ts->int_trigger_type | IRQF_ONESHOT, NVT_SPI_NAME, ts);
#endif
		if (ret != 0) {
			NVT_ERR("request irq failed. ret=%d\n", ret);
			goto err_int_request_failed;
		} else {
			//disable_irq(client->irq);
			ts->irq_enable_flag = 1;
			nvt_irq_enable(0);
			NVT_LOG("request irq %d succeed\n", client->irq);
		}
	}
#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = create_singlethread_workqueue("nvt_fwu_wq");
	if (!nvt_fwu_wq) {
		NVT_ERR("nvt_fwu_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	// please make sure boot update start after display reset(RESX) sequence
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(500));
#endif
	NVT_LOG("NVT_TOUCH_ESD_PROTECT is %d\n", NVT_TOUCH_ESD_PROTECT);
#if NVT_TOUCH_ESD_PROTECT
	INIT_DELAYED_WORK(&nvt_esd_check_work, nvt_esd_check_func);
	nvt_esd_check_wq = create_workqueue("nvt_esd_check_wq");
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	//---set device node---
#if NVT_TOUCH_PROC
	ret = nvt_flash_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt flash proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_EXT_PROC
	ret = nvt_extra_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt extra proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_MP
	ret = nvt_mp_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt mp proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif
	nvt_update_fw_notifier_init();
	nvt_tp_usb_notifier_init();
	nvt_tp_headset_notifier_init();
#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret =  fb_register_client(&ts->fb_notif);
	if(ret) {
		NVT_ERR("register fb_notifier failed. ret=%d\n", ret);
		goto err_register_fb_notif_failed;
	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = nvt_ts_early_suspend;
	ts->early_suspend.resume = nvt_ts_late_resume;
	ret = register_early_suspend(&ts->early_suspend);
	if(ret) {
		NVT_ERR("register early suspend failed. ret=%d\n", ret);
		goto err_register_early_suspend_failed;
	}
#endif

#ifdef ODM_WT_EDIT
//	ret = devm_tid_register(dev, tid);
//	if (unlikely(ret))
//		return ret;
#endif
	bTouchIsAwake = 1;
	ts->is_suspended = 0;
	NVT_LOG("end\n");
/*
//#if 0	
	oplus_ftmmode_start = strstr(saved_command_line,"oplus_ftm_mode=");
	if ( oplus_ftmmode_start != NULL ) {
		oplus_ftmmode_start += strlen("oplus_ftm_mode=");
		if (strncmp(oplus_ftmmode_start, "factory2", strlen("factory2")) == 0) {
			ts->boot_mode = 3;
			NVT_LOG("boot_mode is FACTORY,RF and WLAN not need to enable irq\n");
		} else {
			nvt_irq_enable(1);
		}
	} else {
		NVT_LOG("boot_mode is failed\n");
		nvt_irq_enable(1);
	}
*/

	ts->boot_mode = get_boot_mode();
	NVT_LOG("==========ts->boot_mode = %d\n",ts->boot_mode);
	//if ((ts->boot_mode == MSM_BOOT_MODE__FACTORY || ts->boot_mode == MSM_BOOT_MODE__RF || ts->boot_mode == MSM_BOOT_MODE__WLAN))
	if ((ts->boot_mode == 3 || ts->boot_mode == 4 || ts->boot_mode == 5)){
		NVT_LOG("boot_mode is FACTORY,RF and WLAN not need to enable irq\n");
	} else {
		nvt_irq_enable(1);
	}

//#endif		
	this_is_nvt_touch = 1;
	return 0;

#if defined(CONFIG_FB)
err_register_fb_notif_failed:
#elif defined(CONFIG_HAS_EARLYSUSPEND)
err_register_early_suspend_failed:
#endif
#if (NVT_TOUCH_PROC || NVT_TOUCH_EXT_PROC || NVT_TOUCH_MP)
err_init_NVT_ts:
#endif
	free_irq(client->irq, ts);
#if BOOT_UPDATE_FIRMWARE
err_create_nvt_fwu_wq_failed:
#endif
err_int_request_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
//err_create_nvt_wq_failed:
	mutex_destroy(&ts->lock);
err_chipvertrim_failed:
	gpio_free(ts->irq_gpio);
err_gpio_config_failed:
err_spi_setup:
err_ckeck_full_duplex:
	spi_set_drvdata(client, NULL);
	kfree(ts);
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_remove(struct spi_device *client)
{
#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		NVT_ERR("Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif

	mutex_destroy(&ts->lock);

	NVT_LOG("Removing driver...\n");

	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	spi_set_drvdata(client, NULL);
	kfree(ts);

	return 0;
}

unsigned int shutdown_tp_flag = 0;//0:normal work; 1:TP shutdown

static void nvt_ts_shutdown(struct spi_device *client)
{
	NVT_LOG("Shutdown driver...\n");
	bTouchIsAwake = 0;
	shutdown_tp_flag = 1;
	nvt_irq_enable(false);

#if NVT_TOUCH_ESD_PROTECT

	nvt_esd_check_enable(false);

#endif /* #if NVT_TOUCH_ESD_PROTECT */

}

/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
//extern void msm_spi_pinctrl_select_deep_sleep(struct spi_master *master);
static int32_t nvt_ts_suspend(struct device *dev)
{
	//uint8_t buf[4] = {0};
#ifdef ODM_WT_EDIT
//	int pwr;
#endif
#if MT_PROTOCOL_B
	uint32_t i = 0;
#endif

	mutex_lock(&ts->lock);
	if (!bTouchIsAwake) {
		NVT_LOG("Touch is already suspend\n");
		mutex_unlock(&ts->lock);
		return 0;
	}
	mutex_unlock(&ts->lock);

#if WAKEUP_GESTURE
		if (ts->gesture_enable) {
			/* Do nothing */
		} else {
			nvt_irq_enable(false);
//#ifdef ODM_WT_EDIT
#if 0
			pwr = nvt_power_on(ts, false);
			if(pwr != 0) {
				NVT_ERR("power off fail\n");
			}else{
				power_flag =  0;
			}
#endif
		}
#else
	nvt_irq_enable(false);
#endif

#if NVT_TOUCH_ESD_PROTECT
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	bTouchIsAwake = 0;
	ts->is_suspended = 1;

#if WAKEUP_GESTURE
	if (ts->gesture_enable) {
		/*---write spi command to enter "wakeup gesture mode"---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x13;
		CTP_SPI_WRITE(ts->client, buf, 2);*/
		nvt_mode_change_cmd(0x13);
		enable_irq_wake(ts->client->irq);

		NVT_LOG("Enabled touch wakeup gesture\n");
	} else {
		/*disable_irq(ts->client->irq);

		---write spi command to enter "deep sleep mode"---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x11;
		CTP_SPI_WRITE(ts->client, buf, 2);*/
		nvt_mode_change_cmd(0x11);
		NVT_LOG("Enabled touch deep sleep mode\n");
	}

#else // WAKEUP_GESTURE
	/*disable_irq(ts->client->irq);
	---write spi command to enter "deep sleep mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x11;
	CTP_SPI_WRITE(ts->client, buf, 2);*/
	nvt_mode_change_cmd(0x11);
	NVT_LOG("Enabled touch deep sleep mode\n");
#endif // WAKEUP_GESTURE

	/* release all touches */
#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
#if !MT_PROTOCOL_B
	input_mt_sync(ts->input_dev);
#endif
	input_sync(ts->input_dev);

	//msleep(50);	//nvt_mode_change_cmd() will check cmd is received

	mutex_unlock(&ts->lock);

	NVT_LOG("end\n");

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/

static int32_t nvt_ts_resume(struct device *dev)
{
	//update_tpfw_notifier_call_chain(1,NULL);
	NVT_LOG("%s  do nothing in this function\n",__func__);
	return 0;
}


#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct nvt_ts_data *ts =
		container_of(self, struct nvt_ts_data, fb_notif);

	NVT_LOG("Notifier's event = %ld\n", event);
	if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_POWERDOWN) {
			NVT_LOG("%s : FB_BLANK_POWERDOWN nvt_ts_suspend\n",__func__);
			nvt_ts_suspend(&ts->client->dev);
		}else if(*blank == FB_BLANK_UNBLANK){
			gloal_reset_flag = 1;//early resume
		}
	} else if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			NVT_LOG("%s : FB_BLANK_UNBLANK nvt_ts_resume\n",__func__);
			nvt_ts_resume(&ts->client->dev);
		}else if (*blank == FB_BLANK_POWERDOWN) {
			gloal_reset_flag = 0;//early resume
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*******************************************************
Description:
	Novatek touchscreen driver early suspend function.

return:
	n.a.
*******************************************************/
static void nvt_ts_early_suspend(struct early_suspend *h)
{
	nvt_ts_suspend(ts->client, PMSG_SUSPEND);
}

/*******************************************************
Description:
	Novatek touchscreen driver late resume function.

return:
	n.a.
*******************************************************/
static void nvt_ts_late_resume(struct early_suspend *h)
{
	nvt_ts_resume(ts->client);
}
#endif

static const struct spi_device_id nvt_ts_id[] = {
	{ NVT_SPI_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id nvt_match_table[] = {
	{ .compatible = "novatek,NVT-ts-spi",},
	{ },
};
#endif

static struct spi_driver nvt_spi_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.shutdown	= nvt_ts_shutdown,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_SPI_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
	},
};

/*******************************************************
Description:
	Driver Install function.

return:
	Executive Outcomes. 0---succeed. not 0---failed.
********************************************************/
static int32_t __init nvt_driver_init(void)
{
	int32_t ret = 0;

	NVT_LOG("start\n");

	//---add spi driver---
	ret = spi_register_driver(&nvt_spi_driver);
	if (ret) {
		pr_err("%s: failed to add spi driver", __func__);
		goto err_driver;
	}

	NVT_LOG("%s: finished\n", __func__);

err_driver:
	return ret;
}

/*******************************************************
Description:
	Driver uninstall function.

return:
	n.a.
********************************************************/
static void __exit nvt_driver_exit(void)
{
	spi_unregister_driver(&nvt_spi_driver);

	//if (nvt_wq)
		//destroy_workqueue(nvt_wq);

#if BOOT_UPDATE_FIRMWARE
#ifdef ODM_WT_EDIT
	cancel_delayed_work_sync(&ts->nvt_fwu_work);
#endif
	if (nvt_fwu_wq)
		destroy_workqueue(nvt_fwu_wq);
#endif

#if NVT_TOUCH_ESD_PROTECT
#ifdef ODM_WT_EDIT
	cancel_delayed_work_sync(&nvt_esd_check_work);
#endif
	if (nvt_esd_check_wq)
		destroy_workqueue(nvt_esd_check_wq);
#endif /* #if NVT_TOUCH_ESD_PROTECT */
}

//late_initcall(nvt_driver_init);
module_init(nvt_driver_init);
module_exit(nvt_driver_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
