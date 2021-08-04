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


#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>

#include "nt36xxx.h"
#include <linux/uaccess.h>

#if NVT_TOUCH_EXT_PROC

#define NVT_FW_VERSION "nvt_fw_version"
#define NVT_BASELINE "nvt_baseline"
#define NVT_RAW "nvt_raw"
#define NVT_DIFF "nvt_diff"

#if 0
#define OPLUS_TOUCHPANEL_NAME "touchpanel"
#define OPLUS_BASELINE_TEST "baseline_test"
#define OPLUS_COORDINATE "coordinate"
#define OPLUS_DEBUG_INFO "debug_info"
#define OPLUS_DELTA "delta"
#define OPLUS_BASELINE "baseline"
#define OPLUS_MAIN_REGISTER "main_register"
#define OPLUS_DEBUG_LEVEL "debug_level"
#define OPLUS_GESTURE "double_tap_enable"
#define OPLUS_IRQ_DEPATH "irq_depth"
#define OPLUS_REGISTER_INFO "oplus_register_info"
#define OPLUS_FW_UPDATE "tp_fw_update"
#define OPLUS_GAME_SWITCH "game_switch_enable"
#define OPLUS_TP_LIMIT_ENABLE "oplus_tp_limit_enable"
#ifdef ODM_WT_EDIT
#define OPLUS_TP_DIRECTION "oplus_tp_direction"
#endif

extern struct file_operations oplus_nvt_selftest_fops;

static struct proc_dir_entry *oplus_baseline_test;
static struct proc_dir_entry *oplus_coordinate;
static struct proc_dir_entry *oplus_delta;
static struct proc_dir_entry *oplus_baseline;
static struct proc_dir_entry *oplus_main_register;
static struct proc_dir_entry *oplus_debug_level;
static struct proc_dir_entry *oplus_gesture;
static struct proc_dir_entry *oplus_irq_depath;
static struct proc_dir_entry *register_info_oplus;
static struct proc_dir_entry *oplus_fw_update;
static struct proc_dir_entry *oplus_game_switch;
static struct proc_dir_entry *oplus_tp_limit_enable;

#ifdef ODM_WT_EDIT
static struct proc_dir_entry *oplus_tp_direction;
#endif

static struct proc_dir_entry *NVT_proc_diff_entry;
#endif
extern int32_t nvt_selftest_open(struct inode *inode, struct file *file);
extern int oplus_nvt_blackscreen_test(void);
extern void nvt_irq_enable(bool enable);
extern void nvt_mode_change_cmd(uint8_t cmd);
extern int nvt_enable_hopping_polling_mode(bool enable);
extern int nvt_enable_hopping_fix_freq_mode(bool enable);



#define SPI_TANSFER_LENGTH  256

#define NORMAL_MODE 0x00
#define TEST_MODE_1 0x21
#define TEST_MODE_2 0x22
#define HANDSHAKING_HOST_READY 0xBB

#define XDATA_SECTOR_SIZE   256

static uint8_t xdata_tmp[2048] = {0};
static int32_t xdata[2048] = {0};
static int32_t xdata_i[2048] = {0};
static int32_t xdata_q[2048] = {0};

static struct proc_dir_entry *NVT_proc_fw_version_entry;
static struct proc_dir_entry *NVT_proc_baseline_entry;
static struct proc_dir_entry *NVT_proc_raw_entry;
static struct proc_dir_entry *NVT_proc_diff_entry;

extern void nvt_rest_ddi(void);
extern int gloal_reset_flag;
int limit_enable;
int limit_direction;

int NT_SIGN = 0;

#ifdef ODM_WT_EDIT
bool hop_status = false;
bool fix_status = false;
#endif

#ifdef ODM_WT_EDIT
extern int power_flag;
extern int  nvt_power_on(struct nvt_ts_data *power_idev, bool on);
#endif

/*******************************************************
Description:
	Novatek touchscreen change mode function.

return:
	n.a.
*******************************************************/
void nvt_change_mode(uint8_t mode)
{
	uint8_t buf[8] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	//---set mode---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = mode;
	CTP_SPI_WRITE(ts->client, buf, 2);

	if (mode == NORMAL_MODE) {
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = HANDSHAKING_HOST_READY;
		CTP_SPI_WRITE(ts->client, buf, 2);
		msleep(20);
	}
}

/*******************************************************
Description:
	Novatek touchscreen get firmware pipe function.

return:
	Executive outcomes. 0---pipe 0. 1---pipe 1.
*******************************************************/
uint8_t nvt_get_fw_pipe(void)
{
	uint8_t buf[8]= {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

	//---read fw status---
	buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
	buf[1] = 0x00;
	CTP_SPI_READ(ts->client, buf, 2);

	//NVT_LOG("FW pipe=%d, buf[1]=0x%02X\n", (buf[1]&0x01), buf[1]);

	return (buf[1] & 0x01);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data function.

return:
	n.a.
*******************************************************/
void nvt_read_mdata(uint32_t xdata_addr, uint32_t xdata_btn_addr)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	uint8_t buf[SPI_TANSFER_LENGTH + 3] = {0};
	uint32_t head_addr = 0;
	int32_t dummy_len = 0;
	int32_t data_len = 0;
	int32_t residual_len = 0;

	//---set xdata sector address & length---
	head_addr = xdata_addr - (xdata_addr % XDATA_SECTOR_SIZE);
	dummy_len = xdata_addr - head_addr;
	data_len = ts->x_num * ts->y_num * 2;
	residual_len = (head_addr + dummy_len + data_len) % XDATA_SECTOR_SIZE;

	//printk("head_addr=0x%05X, dummy_len=0x%05X, data_len=0x%05X, residual_len=0x%05X\n", head_addr, dummy_len, data_len, residual_len);

	//read xdata : step 1
	for (i = 0; i < ((dummy_len + data_len) / XDATA_SECTOR_SIZE); i++) {
		//---read xdata by SPI_TANSFER_LENGTH
		for (j = 0; j < (XDATA_SECTOR_SIZE / SPI_TANSFER_LENGTH); j++) {
			//---change xdata index---
			nvt_set_page(head_addr + (XDATA_SECTOR_SIZE * i) + (SPI_TANSFER_LENGTH * j));

			//---read data---
			buf[0] = SPI_TANSFER_LENGTH * j;
			CTP_SPI_READ(ts->client, buf, SPI_TANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < SPI_TANSFER_LENGTH; k++) {
				xdata_tmp[XDATA_SECTOR_SIZE * i + SPI_TANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04X\n", buf[k+1], (XDATA_SECTOR_SIZE*i + SPI_TANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (head_addr+XDATA_SECTOR_SIZE*i));
	}

	//read xdata : step2
	if (residual_len != 0) {
		//---read xdata by SPI_TANSFER_LENGTH
		for (j = 0; j < (residual_len / SPI_TANSFER_LENGTH + 1); j++) {
			//---change xdata index---
			nvt_set_page(xdata_addr + data_len - residual_len + (SPI_TANSFER_LENGTH * j));

			//---read data---
			buf[0] = SPI_TANSFER_LENGTH * j;
			CTP_SPI_READ(ts->client, buf, SPI_TANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < SPI_TANSFER_LENGTH; k++) {
				xdata_tmp[(dummy_len + data_len - residual_len) + SPI_TANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04x\n", buf[k+1], ((dummy_len+data_len-residual_len) + SPI_TANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (xdata_addr+data_len-residual_len));
	}

	//---remove dummy data and 2bytes-to-1data---
	for (i = 0; i < (data_len / 2); i++) {
		xdata[i] = (int16_t)(xdata_tmp[dummy_len + i * 2] + 256 * xdata_tmp[dummy_len + i * 2 + 1]);
	}

#if TOUCH_KEY_NUM > 0
	//read button xdata : step3
	//---change xdata index---
	nvt_set_page(xdata_btn_addr);
	//---read data---
	buf[0] = (xdata_btn_addr & 0xFF);
	CTP_SPI_READ(ts->client, buf, (TOUCH_KEY_NUM * 2 + 1));

	//---2bytes-to-1data---
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		xdata[ts->x_num * ts->y_num + i] = (int16_t)(buf[1 + i * 2] + 256 * buf[1 + i * 2 + 1]);
	}
#endif

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data from IQ to rss function.

return:
	n.a.
*******************************************************/
void nvt_read_mdata_rss(uint32_t xdata_i_addr, uint32_t xdata_q_addr, uint32_t xdata_btn_i_addr, uint32_t xdata_btn_q_addr)
{
	int i = 0;

	nvt_read_mdata(xdata_i_addr, xdata_btn_i_addr);
	memcpy(xdata_i, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));

	nvt_read_mdata(xdata_q_addr, xdata_btn_q_addr);
	memcpy(xdata_q, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));

	for (i = 0; i < (ts->x_num * ts->y_num + TOUCH_KEY_NUM); i++) {
		xdata[i] = (int32_t)int_sqrt((unsigned long)(xdata_i[i] * xdata_i[i]) + (unsigned long)(xdata_q[i] * xdata_q[i]));
	}
}

/*******************************************************
Description:
    Novatek touchscreen get meta data function.

return:
    n.a.
*******************************************************/
void nvt_get_mdata(int32_t *buf, uint8_t *m_x_num, uint8_t *m_y_num)
{
    *m_x_num = ts->x_num;
    *m_y_num = ts->y_num;
    memcpy(buf, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));
}

/*******************************************************
Description:
	Novatek touchscreen firmware version show function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_fw_version_show(struct seq_file *m, void *v)
{
	seq_printf(m, "fw_ver=%d, x_num=%d, y_num=%d, button_num=%d\n", ts->fw_ver, ts->x_num, ts->y_num, ts->max_button_num);
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print show
	function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_show(struct seq_file *m, void *v)
{
	int32_t i = 0;
	int32_t j = 0;

	for (i = 0; i < ts->y_num; i++) {
		for (j = 0; j < ts->x_num; j++) {
			seq_printf(m, "%5d, ", xdata[i * ts->x_num + j]);
		}
		seq_puts(m, "\n");
	}

#if TOUCH_KEY_NUM > 0
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		seq_printf(m, "%5d, ", xdata[ts->x_num * ts->y_num + i]);
	}
	seq_puts(m, "\n");
#endif

	seq_printf(m, "\n\n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print start
	function.

return:
	Executive outcomes. 1---call next function.
	NULL---not call next function and sequence loop
	stop.
*******************************************************/
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print next
	function.

return:
	Executive outcomes. NULL---no next and call sequence
	stop function.
*******************************************************/
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print stop
	function.

return:
	n.a.
*******************************************************/
static void c_stop(struct seq_file *m, void *v)
{
	return;
}

const struct seq_operations nvt_fw_version_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_fw_version_show
};

const struct seq_operations nvt_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_show
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_fw_version open
	function.

return:
	n.a.
*******************************************************/
static int32_t nvt_fw_version_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_fw_version_seq_ops);
}

static const struct file_operations nvt_fw_version_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fw_version_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_baseline open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_baseline_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (ts->carrier_system) {
		nvt_read_mdata_rss(ts->mmap->BASELINE_ADDR, ts->mmap->BASELINE_Q_ADDR,
				ts->mmap->BASELINE_BTN_ADDR, ts->mmap->BASELINE_BTN_Q_ADDR);
	} else {
		nvt_read_mdata(ts->mmap->BASELINE_ADDR, ts->mmap->BASELINE_BTN_ADDR);
	}

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations baseline_fops = {
	.owner = THIS_MODULE,
	.open = nvt_baseline_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_raw open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_raw_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (ts->carrier_system) {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata_rss(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_PIPE0_Q_ADDR,
				ts->mmap->RAW_BTN_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_Q_ADDR);
		else
			nvt_read_mdata_rss(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_PIPE1_Q_ADDR,
				ts->mmap->RAW_BTN_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_Q_ADDR);
	} else {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_ADDR);
	}

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_raw_fops = {
	.owner = THIS_MODULE,
	.open = nvt_raw_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_diff open function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_diff_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (ts->carrier_system) {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata_rss(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_PIPE0_Q_ADDR,
				ts->mmap->DIFF_BTN_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_Q_ADDR);
		else
			nvt_read_mdata_rss(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_PIPE1_Q_ADDR,
				ts->mmap->DIFF_BTN_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_Q_ADDR);
	} else {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_ADDR);
	}

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations delta_fops = {
	.owner = THIS_MODULE,
	.open = nvt_diff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen extra function proc. file node
	initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
/* coordinate */
static int32_t c_oplus_coordinate_show(struct seq_file *m, void *v)
{
	struct gesture_info *gesture = &ts->gesture;
	char tmp[256] = {0};

	sprintf(tmp, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d",
		gesture->gesture_type,
		gesture->Point_start.x, gesture->Point_start.y,
		gesture->Point_end.x, gesture->Point_end.y,
		gesture->Point_1st.x, gesture->Point_1st.y,
		gesture->Point_2nd.x, gesture->Point_2nd.y,
		gesture->Point_3rd.x, gesture->Point_3rd.y,
		gesture->Point_4th.x, gesture->Point_4th.y,
		gesture->clockwise);

	/* oplus gesture formate */
	seq_printf(m, "%s\n", tmp);

	return 0;
}

const struct seq_operations oplus_coordinate_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_oplus_coordinate_show
};

static int32_t oplus_coordinate_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &oplus_coordinate_seq_ops);
}

static const struct file_operations coordinate_fops = {
	.owner = THIS_MODULE,
	.open = oplus_coordinate_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/* main_register */
static int32_t c_main_register_show(struct seq_file *m, void *v)
{
    uint8_t buf[4] = {0};
	struct irq_desc *desc = irq_to_desc(gpio_to_irq(ts->irq_gpio));

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	//---read cmd status---
	buf[0] = 0x5E;
	buf[1] = 0xFF;
	CTP_SPI_READ(ts->client, buf, 2);

	NVT_LOG("PWR_FLAG:%d\n", (buf[1]>> PWR_FLAG) & 0x01);
	seq_printf(m, "PWR_FLAG:%d\n", (buf[1]>> PWR_FLAG) & 0x01);

	NVT_LOG("EDGE_REJECT:%d\n", (buf[1]>> EDGE_REJECT_L) & 0x03);
	seq_printf(m, "EDGE_REJECT:%d\n", (buf[1]>> EDGE_REJECT_L) & 0x03);


	NVT_LOG("JITTER_FLAG:%d\n", (buf[1]>> JITTER_FLAG) & 0x01);
	seq_printf(m, "JITTER_FLAG:%d\n", (buf[1]>> JITTER_FLAG) & 0x01);

	NVT_LOG("HEADSET_FLAG:%d\n", (buf[1]>> HEADSET_FLAG) & 0x01);
	seq_printf(m, "HEADSET_FLAG:%d\n", (buf[1]>> HEADSET_FLAG) & 0x01);

#ifdef ODM_WT_EDIT
	NVT_LOG("HOPPING_POLLING_FLAG:%d\n", (buf[1]>> HOPPING_POLLING_FLAG) & 0x01);
	seq_printf(m, "HOPPING_POLLING_FLAG:%d\n", (buf[1]>> HOPPING_POLLING_FLAG) & 0x01);
#endif
	NVT_ERR("IRQ_DEPTH:%d\n", desc->depth);
	seq_printf(m, "IRQ_DEPTH:%d\n", desc->depth);

	return 0;
}

const struct seq_operations oplus_main_register_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_main_register_show
};

static int32_t nvt_main_register_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &oplus_main_register_seq_ops);
}

static const struct file_operations main_register_fops = {
	.owner = THIS_MODULE,
	.open = nvt_main_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/* debug_level */
static ssize_t oplus_debug_level_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int tmp = 0;
	char cmd[128] = {0};

	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &tmp)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	ts->debug_level = tmp;

	NVT_LOG("debug_level is %d\n", ts->debug_level);

	if ((ts->debug_level != 0) && (ts->debug_level != 1) && (ts->debug_level != 2)) {
		NVT_ERR("debug level error %d\n", ts->debug_level);
		ts->debug_level = 0;
	}

	return count;
};

static const struct file_operations debug_level_fops =
{
	.write = oplus_debug_level_write,
	.owner = THIS_MODULE,
};

/* double_tap_enable */
#define GESTURE_ENABLE   (1)
#define GESTURE_DISABLE  (0)
static ssize_t oplus_gesture_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
	//unsigned int tmp = 0;
	//char cmd[5] = {0};
	//int len = 0;
	//uint8_t buff[4] = {0};
	char *ptr = NULL;
	/*if (ts->sleep_flag !=0) {
		NVT_LOG("%s, is already suspend",__func__);
		return -1;
	}*/
#ifdef ODM_WT_EDIT
//	int pwr;
#endif
	ptr = kzalloc(count,GFP_KERNEL);
	if (ptr == NULL){
		NVT_LOG("allocate memory fail\n");
		return -1;
	}
	if (copy_from_user(ptr, buf, count)) {
		NVT_LOG("input value error\n");
		return -EINVAL;
	}
	if (ptr[0] == '1') {
		NVT_LOG("%s--gloal_reset_flag = %d\n",__func__,gloal_reset_flag);
		if ((ts->is_suspended == 1)&&(gloal_reset_flag == 0)){
			mutex_lock(&ts->lock);
			NVT_LOG("far away psensor\n");
			ts->gesture_enable = 1;
			tp_gesture = 1;
#ifdef ODM_WT_EDIT
//			if(power_flag == 0){
//				pwr = nvt_power_on(ts, true);
//				if(pwr != 0)
//					NVT_ERR("nvt_power_on failed\n");
//			}
#endif
			nvt_rest_ddi();
			nvt_update_firmware(fw->firmware_name);

			//buff[0] = EVENT_MAP_HOST_CMD;
			//buff[1] = 0x13;
			//CTP_SPI_WRITE(ts->client, buff, 2);
			nvt_mode_change_cmd(0x13);
			NVT_LOG("ts->gesture_enable =%d:need to download fw\n",ts->gesture_enable);
			mutex_unlock(&ts->lock);
		} else {
			ts->gesture_enable = 1;
			tp_gesture = 1;
			NVT_LOG("normal to open gesture:Gesture %d\n",ts->gesture_enable);
		}
	} else if (ptr[0] == '0') {
		ts->gesture_enable = 0;
		tp_gesture = 0;
		NVT_LOG("normal clsoe gesture:Gesture %d\n",ts->gesture_enable);

	} else if (ptr[0] == '2') {
		if(ts->is_suspended == 1) {
			if(ts->gesture_enable == 0){
				NVT_LOG("%s: prt[0] = 2 --nvt mode alread is sleep",__func__);
				return count;
			}
			ts->gesture_enable = 0;
			tp_gesture = 0;
			NVT_LOG("need psensor close Gesture \n");
			//buff[0] = EVENT_MAP_HOST_CMD;
			//buff[1] = 0x11;
			//CTP_SPI_WRITE(ts->client, buff, 2);
			nvt_mode_change_cmd(0x11);
		}
	}
	else {
		NVT_LOG("can not get the correct gesture control\n");
	}
	/*if (kstrtouint(cmd, 0, &tmp)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	ts->gesture_enable = tmp > 0 ? GESTURE_ENABLE : GESTURE_DISABLE;

	NVT_LOG("Gesture %s\n", ts->gesture_enable ? "enable" : "disable");
	*/
	kfree(ptr);
	return count;


}

static ssize_t oplus_gesture_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	int ret = 0;
	int len;
	uint8_t *ptr = NULL;

    if(*ppos) {
        return 0;
    }

	ptr = kzalloc(count, GFP_KERNEL);
	if (ptr == NULL) {
		NVT_ERR("failed to allocate memory for ptr\n");
		return 0;
	}
	len = snprintf(ptr,count, "%d\n", ts->gesture_enable);
	ret = copy_to_user(buf, ptr, len);

	*ppos += len;
	kfree(ptr);
	return len;


}

static const struct file_operations double_tap_enable_fops =
{
	.write = oplus_gesture_write,
	.read = oplus_gesture_read,
	.owner = THIS_MODULE,
};

/* tp_fw_update */
extern uint8_t request_and_download_normal_complete;
extern uint8_t request_and_download_sign_complete;
static ssize_t oplus_fw_update_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int tmp = 0;
	uint8_t update_type = 0;
	char cmd[128] = {0};

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("RM to write value to firmware\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &tmp)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	update_type = tmp;

	NVT_LOG("update_type is %d\n", update_type);
	switch (update_type) {
		case 0:	/* noflash: force update. flash: force update */
            NT_SIGN = 0;
            request_and_download_normal_complete = false;
#ifdef ODM_WT_EDIT
			nvt_update_firmware(fw->firmware_name);
#endif
			break;
		case 1: /* noflash: do nothing. flash: check fw version and update */
			NVT_ERR("update_type %d. Do nothing for noflash\n", update_type);
   			NT_SIGN = 0;
            request_and_download_normal_complete = false;
			nvt_update_firmware(fw->firmware_name);
			break;
		case 2:
            NVT_ERR("update_type %d. Do nothing for sign firmware\n", update_type);
            NT_SIGN = 1;
            request_and_download_sign_complete = false;
			nvt_update_firmware(fw->firmware_sign_name);
			break;
		default:
			NVT_ERR("update_type %d error\n", update_type);
	}

	NVT_LOG("--\n");
	mutex_unlock(&ts->lock);

	return count;
};

static const struct file_operations tp_fw_update_fops =
{
	.write = oplus_fw_update_write,
	.owner = THIS_MODULE,
};

/* irq_depth */
static int32_t c_irq_depath_show(struct seq_file *m, void *v)
{
	struct irq_desc *desc = irq_to_desc(gpio_to_irq(ts->irq_gpio));
	NVT_ERR("depth %d\n", desc->depth);

	seq_printf(m, "%d\n", desc->depth);

	return 0;
}

const struct seq_operations oplus_irq_depath_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_irq_depath_show
};

static int32_t nvt_irq_depath_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &oplus_irq_depath_seq_ops);
}

static const struct file_operations irq_depath_fops = {
	.owner = THIS_MODULE,
	.open = nvt_irq_depath_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/* oplus_register_info */
struct oplus_register_info {
	uint32_t addr;
	uint32_t len;
} oplus_reg;

/*
 * Example data format: echo 11A60,2 > file_node
 */
static ssize_t oplus_register_info_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	uint8_t tmp[6] = {0};
	char cmd[128] = {0};

	/* Error handler */
	if (count != 8) {
		NVT_ERR("count %ld error\n", count);
		return count;
	}

	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}

	/* parsing address (Novatek address length: 5 bit) */
	sprintf(tmp, "%c%c%c%c%c", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);

	if (kstrtouint(tmp, 16, &oplus_reg.addr)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	NVT_LOG("address: 0x%05X\n", oplus_reg.addr);

	/* parsing length */
	sprintf(tmp, "%c", cmd[6]);
	if (kstrtouint(tmp, 10, &oplus_reg.len)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	NVT_LOG("len %d\n", oplus_reg.len);

	return count;
}

static ssize_t oplus_register_info_read(struct file *file, char __user *buff,size_t count, loff_t *ppos)
{
	uint8_t *buf = NULL;
	uint8_t *ptr = NULL;
	uint8_t len = 0;
	uint8_t i = 0;
	int32_t ret = 0;

	if(*ppos) {
		return 0;	/* the end */
	}

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	if (oplus_reg.len == 0) {
		NVT_ERR("len = %d\n", oplus_reg.len);
		goto fail;
	}

	buf = (uint8_t *)kzalloc(sizeof(uint8_t)*(oplus_reg.len), GFP_KERNEL);
	if (buf == NULL) {
		NVT_ERR("failed to allocate memory for buf\n");
		goto fail;
	}

	ptr = (uint8_t *)kzalloc(sizeof(uint8_t)*(oplus_reg.len)*3+1, GFP_KERNEL);
	if (ptr == NULL) {
		NVT_ERR("failed to allocate memory for ptr\n");
		goto fail;
	}

	/* read data */
	nvt_set_page(oplus_reg.addr);
	buf[0] = oplus_reg.addr & 0x7F;
	CTP_SPI_READ(ts->client, buf, oplus_reg.len + 1);

	/* set index to EVENT_BUF_ADDR */
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	/* copy hex data to string */
	for (i=0 ; i<oplus_reg.len ; i++) {
		len += sprintf(ptr+len, "%02X ", buf[i+1]);
		//NVT_ERR("[%d] buf %02X\n", i, buf[i+1]);
	}

	/* new line */
	len += sprintf(ptr+len, "\n");

	ret = copy_to_user(buff,ptr,len);

	*ppos += len;

fail:
	mutex_unlock(&ts->lock);

	return len;
}

static const struct file_operations oplus_register_info_fops =
{
	.write = oplus_register_info_write,
	.read = oplus_register_info_read,
	.owner = THIS_MODULE,
};

/* oplus_tp_limit_enable */
static ssize_t oplus_tp_limit_enable_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int tmp = 0;
	char cmd[128] = {0};

	if(ts->is_suspended == 1){
		NVT_ERR("limit_enable write: the NT device is already suspend\n");
		return -1;
	}
	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}
	NVT_LOG("%s +++ \n",__func__);
#ifdef ODM_WT_EDIT
	sscanf(cmd, "%x", &tmp);
#endif
	NVT_LOG("%s ---limit_enable tmp= %d\n",__func__,tmp);
	limit_enable = !!tmp;
	NVT_LOG("%s ---limit_enable = %d\n",__func__,limit_enable);
	mutex_lock(&ts->lock);
	if(nvt_mode_switch(MODE_EDGE,limit_direction)) {
		NVT_ERR("edge reject enable fail!\n");
	}
	mutex_unlock(&ts->lock);
	return count;

};
static ssize_t oplus_tp_limit_enable_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char *ptr = NULL;
	int lens = 0;
	if (*ppos){
		printk("is not at the page start \n");
		return 0;
	}
	if(ts->is_suspended == 1){
		NVT_ERR("limit_enable read: the NT device is already suspend\n");
		return -1;
	}
	*ppos +=count;

	NVT_LOG("%s +++ \n",__func__);

	ptr = kzalloc(256,GFP_KERNEL);
	if (ptr == NULL) {
		NVT_LOG("alloc the ptr fail \n");
		return -1;

	}
/*
	if (ts->nvt_oplus_proc_data == NULL) {
		ts->nvt_oplus_proc_data = kzalloc(sizeof(struct nvt_oplus_data), GFP_KERNEL);
		if (ts->nvt_oplus_proc_data == NULL) {
			NVT_LOG("alloc the ts->nvt_oplus_proc_data memery fail \n");
			return -1;
		}
	}

	lens += snprintf(ptr + lens,count-lens,"limit_control = %d\n",ts->nvt_oplus_proc_data->edge_limit.limit_control);
	lens += snprintf(ptr + lens,count-lens,"limit_00 = %d\n",ts->nvt_oplus_proc_data->edge_limit.limit_00);
	lens += snprintf(ptr + lens,count-lens,"limit_lu = %d\n",ts->nvt_oplus_proc_data->edge_limit.limit_lu);
	lens += snprintf(ptr + lens,count-lens,"limit_ru = %d\n",ts->nvt_oplus_proc_data->edge_limit.limit_ru);
	lens += snprintf(ptr + lens,count-lens,"limit_lb = %d\n",ts->nvt_oplus_proc_data->edge_limit.limit_lb);
	lens += snprintf(ptr + lens,count-lens,"limit_rb = %d\n",ts->nvt_oplus_proc_data->edge_limit.limit_rb);
*/
	lens = snprintf(ptr, count,"%x\n",limit_enable);
	ret = copy_to_user(buf,ptr,lens);
	if (ret)
		printk("copy_to_user fail \n");
	return lens;
}

/*oplus_tp_direction*/
static ssize_t oplus_tp_direction_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int tmp = 0;
	char cmd[128] = {0};
	if(ts->is_suspended == 1){
		NVT_ERR("oplus_tp_direction write: the NT device is already suspend\n");
		return -1;
	}
	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}

#ifdef ODM_WT_EDIT
	sscanf(cmd, "%x", &tmp);
#endif
	if((tmp != 0)&&(tmp != 1)&&(tmp != 2))
		NVT_ERR("edge reject enable invalid value tmp = %d\n",tmp);
	limit_direction = tmp;
	mutex_lock(&ts->lock);
	if(nvt_mode_switch(MODE_EDGE,limit_direction)) {
		NVT_ERR("edge reject enable fail!\n");
	}
	mutex_unlock(&ts->lock);
	NVT_LOG("edge reject limit_direction = %d\n",limit_direction);
	return count;
}

static ssize_t oplus_tp_direction_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	uint8_t len;
	char *ptr = NULL;
	if (*ppos){
		printk("is not at the page start \n");
		return 0;
	}
	*ppos +=count;

	if(ts->is_suspended == 1){
		NVT_ERR("oplus_tp_direction read : the NT device is already suspend\n");
		return -1;
	}

	NVT_LOG("%s +++ \n",__func__);

	ptr = kzalloc(256,GFP_KERNEL);
	if (ptr == NULL) {
		NVT_ERR("alloc the ptr fail \n");
		return -1;

	}
	len = snprintf(ptr, count,"%x\n",limit_direction);
	ret = copy_to_user(buf,ptr,len);
	if (ret)
		printk("copy_to_user fail \n");
	return len;
}

/*oplus_tp_hop_test*/
#ifdef ODM_WT_EDIT
static ssize_t oplus_tp_hop_test_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int tmp = 0;
	char cmd[128] = {0};

	if(ts->is_suspended == 1){
		NVT_ERR("oplus_tp_direction write: the NT device is already suspend\n");
		return -1;
	}
	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}
#ifdef ODM_WT_EDIT
	sscanf(cmd, "%x", &tmp);
#endif
	if((tmp != 0)&&(tmp != 1)){
		NVT_ERR("hop_test invalid value tmp = %d\n",tmp);
		return -EINVAL;
	}
	if(tmp == 0)
		hop_status = false;
	if(tmp == 1)
		hop_status = true;
	NVT_LOG("edge reject limit_direction = %d\n",hop_status);
	mutex_lock(&ts->lock);
	nvt_enable_hopping_polling_mode(hop_status);
	mutex_unlock(&ts->lock);
	return count;
}
static ssize_t oplus_tp_hop_test_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	uint8_t len;
	char *ptr = NULL;
	if (*ppos){
		printk("is not at the page start \n");
		return 0;
	}
	*ppos +=count;

	if(ts->is_suspended == 1){
		NVT_ERR("oplus_tp_hop_test read : the NT device is already suspend\n");
		return -1;
	}

	NVT_LOG("%s +++ \n",__func__);

	ptr = kzalloc(256,GFP_KERNEL);
	if (ptr == NULL) {
		NVT_ERR("alloc the ptr fail \n");
		return -1;

	}
	len = snprintf(ptr, count,"%x\n",hop_status);
	ret = copy_to_user(buf,ptr,len);
	if (ret)
		printk("copy_to_user fail \n");
	return len;
}
#endif

#ifdef ODM_WT_EDIT
static ssize_t oplus_tp_fix_test_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int tmp = 0;
	char cmd[128] = {0};

	if(ts->is_suspended == 1){
		NVT_ERR("oplus_tp_direction write: the NT device is already suspend\n");
		return -1;
	}
	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}
#ifdef ODM_WT_EDIT
	sscanf(cmd, "%x", &tmp);
#endif
	if((tmp != 0)&&(tmp != 1)){
		NVT_ERR("fix_test invalid value tmp = %d\n",tmp);
		return -EINVAL;
	}
	if(tmp == 0)
		fix_status = false;
	if(tmp == 1)
		fix_status = true;
	NVT_LOG("edge reject limit_direction = %d\n",fix_status);
	mutex_lock(&ts->lock);
	nvt_enable_hopping_fix_freq_mode(fix_status);
	mutex_unlock(&ts->lock);
	return count;
}
static ssize_t oplus_tp_fix_test_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	uint8_t len;
	char *ptr = NULL;
	if (*ppos){
		printk("is not at the page start \n");
		return 0;
	}
	*ppos +=count;

	if(ts->is_suspended == 1){
		NVT_ERR("oplus_tp_fix_test read : the NT device is already suspend\n");
		return -1;
	}

	NVT_LOG("%s +++ \n",__func__);

	ptr = kzalloc(256,GFP_KERNEL);
	if (ptr == NULL) {
		NVT_ERR("alloc the ptr fail \n");
		return -1;

	}
	len = snprintf(ptr, count,"%x\n",fix_status);
	ret = copy_to_user(buf,ptr,len);
	if (ret)
		printk("copy_to_user fail \n");
	return len;
}
#endif

static const struct file_operations oplus_tp_limit_enable_fops =
{
	.write = oplus_tp_limit_enable_write,
	.read = oplus_tp_limit_enable_read,
	.owner = THIS_MODULE,
};

static const struct file_operations oplus_tp_direction_fops =
{
	.write = oplus_tp_direction_write,
	.read = oplus_tp_direction_read,
	.owner = THIS_MODULE,
};

#ifdef ODM_WT_EDIT
static const struct file_operations freq_hop_simulate_fops =
{
	.write = oplus_tp_hop_test_write,
	.read = oplus_tp_hop_test_read,
	.owner = THIS_MODULE,
};
static const struct file_operations fix_hop_simulate_fops =
{
	.write = oplus_tp_fix_test_write,
	.read = oplus_tp_fix_test_read,
	.owner = THIS_MODULE,
};

#endif

static int32_t openshort_open(struct inode *inode, struct file *file)
{
    int32_t ret = 0;
	ts->oplus_baseline_test_flag = 0;
	NVT_LOG("%s,ts->oplus_baseline_test_flag = %d\n",__func__,ts->oplus_baseline_test_flag);
	ret = nvt_selftest_open(inode,file);

	return ret;
}

static const struct file_operations ctp_openshort_test_fops = {
	.owner = THIS_MODULE,
	.open = openshort_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
static int32_t baseline_test_open(struct inode *inode, struct file *file)
{
    int32_t ret = 0;
	ts->oplus_baseline_test_flag = 1;
	NVT_LOG("%s,ts->oplus_baseline_test_flag = %d\n",__func__,ts->oplus_baseline_test_flag);
	ret = nvt_selftest_open(inode,file);

	return ret;
}

static const struct file_operations baseline_test_fops = {
	.owner = THIS_MODULE,
	.open = baseline_test_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
static ssize_t oplus_tp_limit_area_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{

	int ret = 0;
	int len = 0;
	char *ptr = NULL;
	if ( *ppos ) {
    	printk("is already read the file\n");
    	return 0;
	}

	if (ts->nvt_oplus_proc_data == NULL) {
		ts->nvt_oplus_proc_data = kzalloc(sizeof(struct nvt_oplus_data), GFP_KERNEL);
		if (ts->nvt_oplus_proc_data == NULL) {
			NVT_LOG("alloc the ts->nvt_oplus_proc_data memery fail \n");
			return -1;
		}
	}

	ptr = (char*)kzalloc(count,GFP_KERNEL);

	len += snprintf(ptr+len, count-len,"left_u(x1,y1)=(%d,%d)    ",ts->nvt_oplus_proc_data->nvt_limit_area.area_xlu,
		ts->nvt_oplus_proc_data->nvt_limit_area.area_ylu);

	len += snprintf(ptr+len, count-len,"right_u(x2,y2)=(%d,%d)\n",ts->nvt_oplus_proc_data->nvt_limit_area.area_xru,
		ts->nvt_oplus_proc_data->nvt_limit_area.area_yru);

	len += snprintf(ptr+len, count-len,"left_b(x1,y1)=(%d,%d)    ",ts->nvt_oplus_proc_data->nvt_limit_area.area_xlb,
		ts->nvt_oplus_proc_data->nvt_limit_area.area_ylb);

	len += snprintf(ptr+len, count-len,"right_b(x2,y2)=(%d,%d)\n",ts->nvt_oplus_proc_data->nvt_limit_area.area_xrb,
		ts->nvt_oplus_proc_data->nvt_limit_area.area_yrb);

	ret = copy_to_user(buf,ptr,len);
	*ppos = len;
	kfree(ptr);
	return len;


}
static ssize_t oplus_tp_limit_area_write(struct file *filp, const char __user *buffer,size_t count, loff_t *ppos)
{
    char buf[8] = {0};
    int  temp;
	if (buffer != NULL)
	{
	    if (copy_from_user(buf, buffer, count)) {
	        printk("%s: read proc input error.\n", __func__);
	        return count;
	    }
	}

	if (ts->nvt_oplus_proc_data == NULL) {
		ts->nvt_oplus_proc_data = kzalloc(sizeof(struct nvt_oplus_data), GFP_KERNEL);
		if (ts->nvt_oplus_proc_data == NULL) {
			NVT_LOG("alloc the ts->nvt_oplus_proc_data memery fail \n");
			return -1;
		}
	}

    sscanf(buf, "%x", &temp);

    if (temp < 0 || temp > 10) {
        return count;
    }
	ts->nvt_oplus_proc_data->nvt_limit_area.limit_area = temp;
    ts->nvt_oplus_proc_data->nvt_limit_area.area_xlu   = (ts->nvt_oplus_proc_data->nvt_limit_area.limit_area*1000)/100;
    ts->nvt_oplus_proc_data->nvt_limit_area.area_xru   = TOUCH_DEFAULT_MAX_WIDTH - ts->nvt_oplus_proc_data->nvt_limit_area.area_xlu;
    ts->nvt_oplus_proc_data->nvt_limit_area.area_xlb    = 2 * ts->nvt_oplus_proc_data->nvt_limit_area.area_xlu;
    ts->nvt_oplus_proc_data->nvt_limit_area.area_xrb   = TOUCH_DEFAULT_MAX_WIDTH - (2 * ts->nvt_oplus_proc_data->nvt_limit_area.area_xlu);

    NVT_LOG("limit_area = %d; left_x1 = %d; right_x1 = %d; left_x2 = %d; right_x2 = %d\n",
           ts->nvt_oplus_proc_data->nvt_limit_area.limit_area, ts->nvt_oplus_proc_data->nvt_limit_area.area_xlu,
           ts->nvt_oplus_proc_data->nvt_limit_area.area_xru,ts->nvt_oplus_proc_data->nvt_limit_area.area_xlb, ts->nvt_oplus_proc_data->nvt_limit_area.area_xrb);

    ts->nvt_oplus_proc_data->nvt_limit_area.area_ylu   = (ts->nvt_oplus_proc_data->nvt_limit_area.limit_area*1000)/100;
    ts->nvt_oplus_proc_data->nvt_limit_area.area_yru   =  ts->nvt_oplus_proc_data->nvt_limit_area.area_ylu;
    ts->nvt_oplus_proc_data->nvt_limit_area.area_ylb   = TOUCH_DEFAULT_MAX_HEIGHT-2 * ts->nvt_oplus_proc_data->nvt_limit_area.area_ylu;
    ts->nvt_oplus_proc_data->nvt_limit_area.area_yrb   = TOUCH_DEFAULT_MAX_HEIGHT - (2 * ts->nvt_oplus_proc_data->nvt_limit_area.area_ylu);

    return count;

}

static const struct file_operations oplus_tp_limit_area_fops =
{
	.write = oplus_tp_limit_area_write,
	.read = oplus_tp_limit_area_read,
	.owner = THIS_MODULE,
};

static int32_t oplus_devices_test_show(struct seq_file *m, void *v)
{
	uint8_t buf[4] = {0};
	int flag = 0;
	char *ptr = NULL;
	ptr = kzalloc(sizeof(char)*50, GFP_KERNEL);
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);
	//---read cmd status---
	buf[0] = 0x5E;
	buf[1] = 0xFF;
	CTP_SPI_READ(ts->client, buf, 2);
	flag = (buf[1]>> 4) & 0x01;
	if ( flag ) {
        ptr = "Noatek Spi Device";
	} else {
        ptr = "Error!";
	}
    seq_printf(m, "%s\n",ptr);
	return 0;
}

const struct seq_operations oplus_devices_test_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = oplus_devices_test_show
};

static int32_t oplus_devices_test_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &oplus_devices_test_ops);
}
static const struct file_operations i2c_device_test_fops = {
	.owner = THIS_MODULE,
	.open = oplus_devices_test_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};


static ssize_t oplus_blackscreen_test_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    int retry = 20;
	int len = 0;
	char *ptr = NULL;
    NVT_LOG("%s %ld %lld\n", __func__, count, *ppos);
	if ( *ppos ) {
    	printk("is already read the file\n");
    	return 0;
	}
    *ppos += count;

	if (ts->nvt_oplus_proc_data == NULL) {
		ts->nvt_oplus_proc_data = kzalloc(sizeof(struct nvt_oplus_data), GFP_KERNEL);
		if (ts->nvt_oplus_proc_data == NULL) {
			NVT_LOG("alloc the ts->nvt_oplus_proc_data memery fail \n");
			return -1;
		}
	}

    if (!ts->nvt_oplus_proc_data->gesture_test.flag) {
		printk("ts->gesture_test.flag ==== %d\n",ts->nvt_oplus_proc_data->gesture_test.flag);
		//return 0;
	}

    ts->nvt_oplus_proc_data->gesture_test.message = kzalloc(256, GFP_KERNEL);
    if (ts->nvt_oplus_proc_data->gesture_test.message==NULL) {
        NVT_LOG("failed to alloc gesture_test.message memory\n");
        return 0;
    }
    ptr = kzalloc(256, GFP_KERNEL);
    if (ptr == NULL) {
        NVT_LOG("failed to alloc ptr memory\n");
        return 0;
    }

    //wait until tp is in sleep, then sleep 500ms to make sure tp is in gesture mode
    do {
        if (ts->is_suspended) {
            msleep(500);
            break;
        }
        msleep(200);
    } while(--retry);

    NVT_LOG("%s retry times %d\n", __func__, retry);
    if (retry == 0 && !ts->is_suspended) {
		len = snprintf(ts->nvt_oplus_proc_data->gesture_test.message,count,"%s\n", "1 errors: not in sleep");
        goto OUT;
    }

    //mutex_lock(&ts->lock);
    ret = oplus_nvt_blackscreen_test();
	if (ret){
		NVT_LOG("can not complete blackscreen test\n");
	}
    //mutex_unlock(&ts->lock);
	len = snprintf(ptr, count,"%s\n",ts->nvt_oplus_proc_data->gesture_test.message);
OUT:
    ts->nvt_oplus_proc_data->gesture_test.flag = 0;
	tp_gesture = ts->g_gesture_bak;
    ts->gesture_enable = ts->nvt_oplus_proc_data->gesture_test.gesture_backup;

	ret=copy_to_user(buf,ptr,len);

    //kfree(ts->nvt_oplus_proc_data);
	kfree(ptr);
    return len;
}

static ssize_t oplus_blackscreen_test_write(struct file *file, const char __user *userbuf, size_t count, loff_t *ppos)
{
    int value = 0;
	char ptr[4] = {0};
#if 0
	char *ptr = NULL;	
	ptr = kzalloc(count,GFP_KERNEL);
	if ( ptr == NULL ) {
		return -1;
	}
#endif
	if (ts->nvt_oplus_proc_data == NULL) {
		ts->nvt_oplus_proc_data = kzalloc(sizeof(struct nvt_oplus_data), GFP_KERNEL);
		if (ts->nvt_oplus_proc_data == NULL) {
			NVT_LOG("alloc the ts->nvt_oplus_proc_data memery fail \n");
			return -1;
		}
	}
	if ( copy_from_user(ptr, userbuf, count) ) {
		NVT_LOG("%s: copy from user error.", __func__);
		return -1;
	}
	NVT_LOG("samir userbuf = %s\n",ptr);
	sscanf(ptr, "%d", &value);

	NVT_LOG("%s:value ============%d\n",__func__,value);

    ts->nvt_oplus_proc_data->gesture_test.gesture_backup = ts->gesture_enable;
	ts->g_gesture_bak = tp_gesture;
    ts->gesture_enable = 1;
	tp_gesture = 1;
    ts->nvt_oplus_proc_data->gesture_test.flag = !!value;
	//kfree(ptr);
    return count;
}

static const struct file_operations black_screen_test_fops = {
    .owner = THIS_MODULE,
    .read  = oplus_blackscreen_test_read,
    .write = oplus_blackscreen_test_write,
};

/* game_switch_enable */
static unsigned int oplus_game_switch_flag = 0;
static ssize_t oplus_game_switch_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int tmp = 0;
	char cmd[128] = {0};

	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}
	if (kstrtouint(cmd, 0, &tmp)) {
		if (cmd[0] - '0') {
			tmp = 1;
		} else {  
			NVT_ERR("kstrtouint error\n");
			return -EINVAL;
		}
	}
	NVT_LOG("game switch enable is %d\n", tmp);
	tmp = !!tmp;
	oplus_game_switch_flag = tmp;
	mutex_lock(&ts->lock);
	if(nvt_mode_switch(MODE_GAME, tmp)) {
		NVT_ERR("game switch enable fail!\n");
	}
	mutex_unlock(&ts->lock);

	return count;
};

static ssize_t oplus_game_switch_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	int len = 0;
	char *ptr = NULL;

	if ( *ppos ) {
	    printk("is already read the file\n");
        return 0;
	}
    *ppos += count;

    ptr = kzalloc(count,GFP_KERNEL);
	if(ptr == NULL){
		printk("allocate memory fail\n");
		return -1;
	}

    len = snprintf(ptr, count,"%d\n",oplus_game_switch_flag);

    ret = copy_to_user(buf,ptr,len);

	kfree(ptr);
	return len;
}


static const struct file_operations game_switch_enable_fops =
{
	.read = oplus_game_switch_read,
	.write = oplus_game_switch_write,
	.owner = THIS_MODULE,
};

int32_t nvt_extra_proc_init(void)
{
	ts->nvt_oplus_proc_data = kzalloc(sizeof(struct nvt_oplus_data), GFP_KERNEL);
	if (ts->nvt_oplus_proc_data == NULL) {
		NVT_LOG("alloc the ts->nvt_oplus_proc_data memery fail \n");
		return -1;
	}

#define NVT_MK_PROC_DIR(name, parent) \
			ts->nvt_oplus_proc_data->name##_dir_entry = proc_mkdir(#name, parent); \
			if (IS_ERR(ts->nvt_oplus_proc_data->name##_dir_entry)) { \
				NVT_ERR("Create proc dir entry '%s' failed %ld \n", \
			#name, PTR_ERR(ts->nvt_oplus_proc_data->name##_dir_entry)); \
				ts->nvt_oplus_proc_data->name##_dir_entry = NULL; \
				return PTR_ERR(ts->nvt_oplus_proc_data->name##_dir_entry); \
			}

#define NVT_MK_PROC_ENTRY(name, mode, parent) \
			ts->nvt_oplus_proc_data->name##_entry = proc_create(#name, mode, parent, &name##_fops); \
			if (IS_ERR(ts->nvt_oplus_proc_data->name##_entry)) { \
				NVT_ERR("Create proc entry '%s' failed %ld \n", \
			#name, PTR_ERR(ts->nvt_oplus_proc_data->name##_entry)); \
				ts->nvt_oplus_proc_data->name##_entry = NULL; \
				return PTR_ERR(ts->nvt_oplus_proc_data->name##_entry); \
			}


		NVT_LOG("oplus proc Init start +++\n");

		NVT_MK_PROC_DIR(touchpanel, NULL)
		NVT_MK_PROC_ENTRY(baseline_test, 0666, ts->nvt_oplus_proc_data->touchpanel_dir_entry)
		NVT_MK_PROC_ENTRY(coordinate, 0444, ts->nvt_oplus_proc_data->touchpanel_dir_entry)
		NVT_MK_PROC_ENTRY(debug_level, 0644, ts->nvt_oplus_proc_data->touchpanel_dir_entry)
		NVT_MK_PROC_ENTRY(double_tap_enable, 0666, ts->nvt_oplus_proc_data->touchpanel_dir_entry)
		NVT_MK_PROC_ENTRY(irq_depath, 0666, ts->nvt_oplus_proc_data->touchpanel_dir_entry)
		NVT_MK_PROC_ENTRY(oplus_register_info, 0664, ts->nvt_oplus_proc_data->touchpanel_dir_entry)
		NVT_MK_PROC_ENTRY(oplus_tp_limit_area, 0664, ts->nvt_oplus_proc_data->touchpanel_dir_entry)
		NVT_MK_PROC_ENTRY(oplus_tp_limit_enable, 0666, ts->nvt_oplus_proc_data->touchpanel_dir_entry)
		NVT_MK_PROC_ENTRY(oplus_tp_direction, 0666, ts->nvt_oplus_proc_data->touchpanel_dir_entry)
		NVT_MK_PROC_ENTRY(tp_fw_update, 0644, ts->nvt_oplus_proc_data->touchpanel_dir_entry)
		NVT_MK_PROC_ENTRY(i2c_device_test, 0644, ts->nvt_oplus_proc_data->touchpanel_dir_entry)
		NVT_MK_PROC_ENTRY(black_screen_test, 0666, ts->nvt_oplus_proc_data->touchpanel_dir_entry)
		NVT_MK_PROC_ENTRY(game_switch_enable, 0666, ts->nvt_oplus_proc_data->touchpanel_dir_entry)

		NVT_MK_PROC_DIR(debug_info, ts->nvt_oplus_proc_data->touchpanel_dir_entry)
		NVT_MK_PROC_ENTRY(delta, 0664, ts->nvt_oplus_proc_data->debug_info_dir_entry)
		NVT_MK_PROC_ENTRY(baseline, 0664, ts->nvt_oplus_proc_data->debug_info_dir_entry)
		NVT_MK_PROC_ENTRY(main_register, 0664, ts->nvt_oplus_proc_data->debug_info_dir_entry)
		#ifdef ODM_WT_EDIT
		NVT_MK_PROC_ENTRY(freq_hop_simulate, 0666, ts->nvt_oplus_proc_data->debug_info_dir_entry)
		NVT_MK_PROC_ENTRY(fix_hop_simulate, 0666, ts->nvt_oplus_proc_data->debug_info_dir_entry)
		#endif

		NVT_MK_PROC_DIR(touchscreen, NULL)
		NVT_MK_PROC_ENTRY(ctp_openshort_test, 0666, ts->nvt_oplus_proc_data->touchscreen_dir_entry)

		NVT_LOG("oplus proc Init end ---\n");


#if 0
    struct proc_dir_entry *oplus_touchpanel_proc = NULL;
	struct proc_dir_entry *debug_info = NULL;

    oplus_touchpanel_proc = proc_mkdir(OPLUS_TOUCHPANEL_NAME, NULL);
    if(oplus_touchpanel_proc == NULL) {
        NVT_ERR("create oplus_touchpanel_proc fail\n");
        return -ENOMEM;
    }

	debug_info = proc_mkdir(OPLUS_DEBUG_INFO, oplus_touchpanel_proc);
    if(debug_info == NULL) {
        NVT_ERR("create debug_info fail\n");
        return -ENOMEM;
    }

	oplus_baseline_test = proc_create(OPLUS_BASELINE_TEST, 0664, oplus_touchpanel_proc, &oplus_nvt_selftest_fops);
	if (oplus_baseline_test == NULL) {
        NVT_ERR("create proc/touchpanel/baseline_test Failed!\n");
		return -ENOMEM;
	}

	oplus_coordinate= proc_create(OPLUS_COORDINATE, 0664, oplus_touchpanel_proc, &oplus_coordinate_fops);
	if (oplus_coordinate == NULL) {
        NVT_ERR("create proc/touchpanel/coordinate Failed!\n");
		return -ENOMEM;
	}

	oplus_delta= proc_create(OPLUS_DELTA, 0664, debug_info, &nvt_diff_fops);
	if (oplus_delta == NULL) {
        NVT_ERR("create proc/touchpanel/debug_info/delta Failed!\n");
		return -ENOMEM;
	}

	oplus_baseline = proc_create(OPLUS_BASELINE, 0664, debug_info, &nvt_baseline_fops);
	if (oplus_baseline == NULL) {
        NVT_ERR("create proc/touchpanel/debug_info/baseline Failed!\n");
		return -ENOMEM;
	}

	oplus_main_register= proc_create(OPLUS_MAIN_REGISTER, 0664, debug_info, &oplus_main_register_fops);
	if (oplus_main_register == NULL) {
        NVT_ERR("create proc/touchpanel/debug_info/main_register Failed!\n");
		return -ENOMEM;
	}

	oplus_debug_level= proc_create(OPLUS_DEBUG_LEVEL, 0664, oplus_touchpanel_proc, &oplus_debug_level_fops);
	if (oplus_debug_level == NULL) {
        NVT_ERR("create proc/touchpanel/debug_level Failed!\n");
		return -ENOMEM;
	}

	oplus_gesture= proc_create(OPLUS_GESTURE,0664, oplus_touchpanel_proc, &oplus_gesture_fops);
	if (oplus_gesture == NULL) {
        NVT_ERR("create proc/touchpanel/double_tap_enable Failed!\n");
		return -ENOMEM;
	}

	oplus_irq_depath = proc_create(OPLUS_IRQ_DEPATH, 0444, oplus_touchpanel_proc, &oplus_irq_depath_fops);
	if (oplus_irq_depath == NULL) {
        NVT_ERR("create proc/touchpanel/irq_depath Failed!\n");
		return -ENOMEM;
	}

	register_info_oplus = proc_create(OPLUS_REGISTER_INFO,0664, oplus_touchpanel_proc, &oplus_register_info_fops);
	if (register_info_oplus == NULL) {
        NVT_ERR("create proc/touchpanel/oplus_register_info Failed!\n");
		return -ENOMEM;
	}

	oplus_fw_update = proc_create(OPLUS_FW_UPDATE,0664, oplus_touchpanel_proc, &oplus_fw_update_fops);
	if (oplus_fw_update == NULL) {
        NVT_ERR("create proc/touchpanel/tp_fw_update Failed!\n");
		return -ENOMEM;
	}

	oplus_game_switch = proc_create(OPLUS_GAME_SWITCH,0222, oplus_touchpanel_proc, &oplus_game_switch_fops);
	if (oplus_game_switch == NULL) {
        NVT_ERR("create proc/touchpanel/oplus_game_switch_enable Failed!\n");
		return -ENOMEM;
	}

	oplus_tp_limit_enable = proc_create(OPLUS_TP_LIMIT_ENABLE,0222, oplus_touchpanel_proc, &oplus_tp_limit_fops);
	if (oplus_tp_limit_enable == NULL) {
        NVT_ERR("create proc/touchpanel/oplus_tp_limit_enable Failed!\n");
		return -ENOMEM;
	}
#endif

	NVT_proc_fw_version_entry = proc_create(NVT_FW_VERSION, 0444, NULL, &nvt_fw_version_fops);
	if (NVT_proc_fw_version_entry == NULL) {
		NVT_ERR("create proc/nvt_fw_version Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_fw_version Succeeded!\n");
	}

	NVT_proc_baseline_entry = proc_create(NVT_BASELINE, 0444, NULL, &baseline_fops);
	if (NVT_proc_baseline_entry == NULL) {
		NVT_ERR("create proc/nvt_baseline Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_baseline Succeeded!\n");
	}

	NVT_proc_raw_entry = proc_create(NVT_RAW, 0444, NULL, &nvt_raw_fops);
	if (NVT_proc_raw_entry == NULL) {
		NVT_ERR("create proc/nvt_raw Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_raw Succeeded!\n");
	}

	NVT_proc_diff_entry = proc_create(NVT_DIFF, 0444, NULL, &delta_fops);
	if (NVT_proc_diff_entry == NULL) {
		NVT_ERR("create proc/nvt_diff Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_diff Succeeded!\n");
	}

	return 0;

}
#endif
