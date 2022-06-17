/********************************************
 ** Copyright (C) 2019 OPLUS Mobile Comm Corp. Ltd.
 ** ODM_HQ_EDIT
 ** File: aw87339_audio.c
 ** Description: source file of aw87339 speaker pa
 ** Version: 1.0
 ** Date : 2019/10/09
 ** ---------------- Revision History: --------------------------
 ** <version>    <date>          < author >              <desc>
 ********************************************/

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include "aw87339_audio.h"

/***************************************************
 * aw87339 marco
 ***************************************************/
#define AW87339_I2C_NAME        "aw87339_pa"
#define AW87339_DRIVER_VERSION  "v1.3.2"

/****************************************************
 * aw87339 variable
 ***************************************************/
struct aw87339 *aw87339;
struct aw87339_container *aw87339_kspk_cnt;
struct aw87339_container *aw87339_drcv_cnt;
struct aw87339_container *aw87339_abrcv_cnt;
struct aw87339_container *aw87339_rcvspk_cnt;
struct aw87339_container *aw87339_voicespk_cnt;

static char *aw87339_kspk_name = "../../odm/firmware/aw87339/aw87339_kspk.bin";
static char *aw87339_drcv_name = "../../odm/firmware/aw87339/aw87339_drcv.bin";
static char *aw87339_abrcv_name = "../../odm/firmware/aw87339/aw87339_abrcv.bin";
static char *aw87339_rcvspk_name = "../../odm/firmware/aw87339/aw87339_rcvspk.bin";
static char *aw87339_voicespk_name = "../../odm/firmware/aw87339/aw87339_voicespk.bin";

unsigned int kspk_load_cont;
unsigned int drcv_load_cont;
unsigned int abrcv_load_cont;
unsigned int rcvspk_load_cont;
unsigned int voicespk_load_cont;

static int aw87339_probed = 0;

/**********************************************************
 * i2c write and read
*********************************************************/
static int aw87339_i2c_write(struct aw87339 *aw87339,
	unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw87339->i2c_client,
						reg_addr,
						reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n",
				__func__, cnt, ret);
	} else {
		break;
	}
	cnt++;
	mdelay(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw87339_i2c_read(struct aw87339 *aw87339,
	unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = 1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
	ret = i2c_smbus_read_byte_data(aw87339->i2c_client, reg_addr);
	if (ret < 0) {
		pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
		} else {
		*reg_data = ret;
		break;
	}
	cnt++;
	mdelay(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

/******************************************************
 * aw87339 hardware control
*******************************************************/
unsigned int aw87339_hw_on(struct aw87339 *aw87339)
{
	pr_info("%s enter\n", __func__);

	if (aw87339 && gpio_is_valid(aw87339->reset_gpio)) {
		gpio_set_value_cansleep(aw87339->reset_gpio, 0);
		mdelay(2);
		gpio_set_value_cansleep(aw87339->reset_gpio, 1);
		mdelay(2);
		aw87339->hwen_flag = 1;
	} else {
		dev_err(&aw87339->i2c_client->dev, "%s: failed\n", __func__);
	}

	return 0;
}

unsigned int aw87339_hw_off(struct aw87339 *aw87339)
{
	pr_info("%s enter\n", __func__);

	if (aw87339 && gpio_is_valid(aw87339->reset_gpio)) {
		gpio_set_value_cansleep(aw87339->reset_gpio, 0);
		mdelay(2);
		aw87339->hwen_flag = 0;
	} else {
		dev_err(&aw87339->i2c_client->dev, "%s: failed\n", __func__);
	}
	return 0;
}

/**********************************************************
 * aw87339 control interface
**********************************************************/
unsigned char aw87339_audio_kspk(void)
{
	unsigned int i;
	unsigned int length;

	if(0 == aw87339_probed)
		return 0;

	pr_info("%s enter\n", __func__);

	if (aw87339 == NULL)
		return 2;

	if (!aw87339->hwen_flag)
		aw87339_hw_on(aw87339);

	length = sizeof(aw87339_kspk_cfg_default)/sizeof(char);
	if (aw87339->kspk_cfg_update_flag == 0) { /*update default data*/
		for (i = 0; i < length; i = i+2) {
			aw87339_i2c_write(aw87339,
					aw87339_kspk_cfg_default[i],
					aw87339_kspk_cfg_default[i+1]);
		}
	}
	if (aw87339->kspk_cfg_update_flag == 1) {  /*update firmware data*/
		for (i = 0; i < aw87339_kspk_cnt->len; i = i+2) {
			aw87339_i2c_write(aw87339,
					aw87339_kspk_cnt->data[i],
					aw87339_kspk_cnt->data[i+1]);
		}
	}

	return 0;
}

unsigned char aw87339_audio_drcv(void)
{
	unsigned int i;
	unsigned int length;

	pr_info("%s enter\n", __func__);

	if (aw87339 == NULL)
		return 2;

	if (!aw87339->hwen_flag)
		aw87339_hw_on(aw87339);

	length = sizeof(aw87339_drcv_cfg_default)/sizeof(char);
	if (aw87339->drcv_cfg_update_flag == 0) { /*send default data*/
		for (i = 0; i < length; i = i+2) {
			aw87339_i2c_write(aw87339,
					aw87339_drcv_cfg_default[i],
					aw87339_drcv_cfg_default[i+1]);
		}
	}

	if (aw87339->drcv_cfg_update_flag == 1) {  /*send firmware data*/
		for (i = 0; i < aw87339_drcv_cnt->len; i = i+2) {
			aw87339_i2c_write(aw87339,
					aw87339_drcv_cnt->data[i],
					aw87339_drcv_cnt->data[i+1]);
		}
	}
	return 0;
}

unsigned char aw87339_audio_abrcv(void)
{
	unsigned int i;
	unsigned int length;

	pr_info("%s enter\n", __func__);

	if (aw87339 == NULL)
		return 2;

	if (!aw87339->hwen_flag)
		aw87339_hw_on(aw87339);


	length = sizeof(aw87339_abrcv_cfg_default)/sizeof(char);
	if (aw87339->abrcv_cfg_update_flag == 0) { /*send default data*/
		for (i = 0; i < length; i = i+2) {
			aw87339_i2c_write(aw87339,
					aw87339_abrcv_cfg_default[i],
					aw87339_abrcv_cfg_default[i+1]);
		}
	}

	if (aw87339->abrcv_cfg_update_flag == 1) {  /*send firmware data*/
		for (i = 0; i < aw87339_abrcv_cnt->len; i = i+2) {
			aw87339_i2c_write(aw87339,
					aw87339_abrcv_cnt->data[i],
					aw87339_abrcv_cnt->data[i+1]);
		}
}
	return 0;
}

unsigned char aw87339_audio_rcvspk(void)
{
	unsigned int i;
	unsigned int length;

	pr_info("%s enter\n", __func__);

	if (aw87339 == NULL)
		return 2;

	if (!aw87339->hwen_flag)
		aw87339_hw_on(aw87339);

	length = sizeof(aw87339_rcvspk_cfg_default)/sizeof(char);
	if (aw87339->rcvspk_cfg_update_flag == 0) { /*send default data*/
		for (i = 0; i < length; i = i+2) {
			aw87339_i2c_write(aw87339,
					aw87339_rcvspk_cfg_default[i],
					aw87339_rcvspk_cfg_default[i+1]);
		}
	}

	if (aw87339->rcvspk_cfg_update_flag == 1) {  /*send firmware data*/
		for (i = 0; i < aw87339_rcvspk_cnt->len; i = i+2) {
			aw87339_i2c_write(aw87339,
					aw87339_rcvspk_cnt->data[i],
					aw87339_rcvspk_cnt->data[i+1]);
		}
	}

	return 0;
}

unsigned char aw87339_audio_voicespk(void)
{
	unsigned int i;
	unsigned int length;

	if(0 == aw87339_probed)
		return 0;

	pr_info("%s enter\n", __func__);

	if (aw87339 == NULL)
		return 2;

	if (!aw87339->hwen_flag)
		aw87339_hw_on(aw87339);

	length = sizeof(aw87339_voicespk_cfg_default) / sizeof(char);
	if (aw87339->voicespk_cfg_update_flag == 0) {	/*send default data */
		for (i = 0; i < length; i = i + 2) {
			aw87339_i2c_write(aw87339,
					  aw87339_voicespk_cfg_default[i],
					  aw87339_voicespk_cfg_default[i + 1]);
		}
	}

	if (aw87339->voicespk_cfg_update_flag == 1) {	/*send firmware data */
		for (i = 0; i < aw87339_voicespk_cnt->len; i = i + 2) {
			aw87339_i2c_write(aw87339,
					  aw87339_voicespk_cnt->data[i],
					  aw87339_voicespk_cnt->data[i + 1]);
		}
	}

	return 0;
}

unsigned char aw87339_audio_off(void)
{
	if(0 == aw87339_probed)
		return 0;

	if (aw87339 == NULL)
		return 2;

	if (aw87339->hwen_flag)
		aw87339_i2c_write(aw87339, 0x01, 0x00);    /*CHIP Disable*/

	aw87339_hw_off(aw87339);

	return 0;
}

int aw87339_audio_probe_get(void)
{
	if (aw87339_probed == 1) {
		return 1;
	}
	return 0;
}

/**********************************************************
 * aw87339 firmware cfg update
**********************************************************/
static void aw87339_rcvspk_cfg_loaded(const struct firmware *cont,
					void *context)
{
	int i = 0;
	int ram_timer_val = 2000;

	pr_info("%s enter\n", __func__);

	rcvspk_load_cont++;
	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__,
			aw87339_rcvspk_name);
		release_firmware(cont);
		if (rcvspk_load_cont <= 2) {
			schedule_delayed_work(&aw87339->ram_work,
					msecs_to_jiffies(ram_timer_val));
			pr_info("%s:restart hrtimer to load firmware\n",
			__func__);
		}
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw87339_rcvspk_name,
				cont ? cont->size : 0);

	for (i = 0; i < cont->size; i = i+2) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n",
		__func__, *(cont->data+i), *(cont->data+i+1));
	}
	/* aw87339 ram update */
	aw87339_rcvspk_cnt = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw87339_rcvspk_cnt) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw87339_rcvspk_cnt->len = cont->size;
	memcpy(aw87339_rcvspk_cnt->data, cont->data, cont->size);
	release_firmware(cont);
	aw87339->rcvspk_cfg_update_flag = 1;

	pr_info("%s: all fw update complete\n", __func__);
}

static int aw87339_rcvspk_update(struct aw87339 *aw87339)
{
	pr_info("%s enter\n", __func__);
	return request_firmware_nowait(THIS_MODULE,
					FW_ACTION_HOTPLUG,
					aw87339_rcvspk_name,
					&aw87339->i2c_client->dev,
					GFP_KERNEL,
					aw87339,
					aw87339_rcvspk_cfg_loaded);
}

static void aw87339_abrcv_cfg_loaded(const struct firmware *cont,
					void *context)
{
	int i = 0;
	int ram_timer_val = 2000;

	pr_info("%s enter\n", __func__);

	abrcv_load_cont++;
	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__,
			aw87339_abrcv_name);
		release_firmware(cont);
		if (abrcv_load_cont <= 2) {
			schedule_delayed_work(&aw87339->ram_work,
					msecs_to_jiffies(ram_timer_val));
			pr_info("%s:restart hrtimer to load firmware\n",
			__func__);
		}
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw87339_abrcv_name,
					cont ? cont->size : 0);

	for (i = 0; i < cont->size; i = i+2) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n",
		__func__, *(cont->data+i), *(cont->data+i+1));
	}


	/* aw87339 ram update */
	aw87339_abrcv_cnt = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw87339_abrcv_cnt) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw87339_abrcv_cnt->len = cont->size;
	memcpy(aw87339_abrcv_cnt->data, cont->data, cont->size);
	release_firmware(cont);
	aw87339->abrcv_cfg_update_flag = 1;

	pr_info("%s: fw update complete\n", __func__);
}

static int aw87339_abrcv_update(struct aw87339 *aw87339)
{
	pr_info("%s enter\n", __func__);
	return request_firmware_nowait(THIS_MODULE,
					FW_ACTION_HOTPLUG,
					aw87339_abrcv_name,
					&aw87339->i2c_client->dev,
					GFP_KERNEL,
					aw87339,
					aw87339_abrcv_cfg_loaded);
}

static void aw87339_drcv_cfg_loaded(const struct firmware *cont, void *context)
{
	int i = 0;
	int ram_timer_val = 2000;

	pr_info("%s enter\n", __func__);

	drcv_load_cont++;
	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw87339_drcv_name);
		release_firmware(cont);
		if (drcv_load_cont <= 2) {
			schedule_delayed_work(&aw87339->ram_work,
					msecs_to_jiffies(ram_timer_val));
			pr_info("%s:restart hrtimer to load firmware\n",
			__func__);
		}
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw87339_drcv_name,
					cont ? cont->size : 0);

	for (i = 0; i < cont->size; i = i+2) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n",
		__func__, *(cont->data+i), *(cont->data+i+1));
	}

	if (aw87339_drcv_cnt != NULL)
		aw87339_drcv_cnt = NULL;

	/* aw87339 ram update */
	aw87339_drcv_cnt = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw87339_drcv_cnt) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw87339_drcv_cnt->len = cont->size;
	memcpy(aw87339_drcv_cnt->data, cont->data, cont->size);
	release_firmware(cont);
	aw87339->drcv_cfg_update_flag = 1;

	pr_info("%s: fw update complete\n", __func__);
}

static int aw87339_drcv_update(struct aw87339 *aw87339)
{
	pr_info("%s enter\n", __func__);
	return request_firmware_nowait(THIS_MODULE,
					FW_ACTION_HOTPLUG,
					aw87339_drcv_name,
					&aw87339->i2c_client->dev,
					GFP_KERNEL,
					aw87339,
					aw87339_drcv_cfg_loaded);
}

static void aw87339_kspk_cfg_loaded(const struct firmware *cont, void *context)
{
	int i = 0;
	int ram_timer_val = 2000;

	pr_info("%s enter\n", __func__);
	kspk_load_cont++;
	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw87339_kspk_name);
		release_firmware(cont);
		if (kspk_load_cont <= 2) {
			schedule_delayed_work(&aw87339->ram_work,
					msecs_to_jiffies(ram_timer_val));
			pr_info("%s:restart hrtimer to load firmware\n",
			__func__);
		}
	return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw87339_kspk_name,
				  cont ? cont->size : 0);

	for (i = 0; i < cont->size; i = i+2) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n",
		__func__, *(cont->data+i), *(cont->data+i+1));
	}

	if (aw87339_kspk_cnt != NULL)
		aw87339_kspk_cnt = NULL;

	/* aw87339 ram update */
	aw87339_kspk_cnt = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw87339_kspk_cnt) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw87339_kspk_cnt->len = cont->size;
	memcpy(aw87339_kspk_cnt->data, cont->data, cont->size);
	release_firmware(cont);
	aw87339->kspk_cfg_update_flag = 1;

	pr_info("%s: fw update complete\n", __func__);
}

#ifdef AWINIC_CFG_UPDATE_DELAY
static int aw87339_kspk_update(struct aw87339 *aw87339)
{
	pr_info("%s enter\n", __func__);

	return request_firmware_nowait(THIS_MODULE,
					FW_ACTION_HOTPLUG,
					aw87339_kspk_name,
					&aw87339->i2c_client->dev,
					GFP_KERNEL,
					aw87339,
					aw87339_kspk_cfg_loaded);
}

static void aw87339_voicespk_cfg_loaded(const struct firmware *cont,
				      void *context)
{
	int i = 0;
	int ram_timer_val = 2000;

	pr_info("%s enter\n", __func__);

	voicespk_load_cont++;
	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__,
		       aw87339_voicespk_name);
		release_firmware(cont);
		if (voicespk_load_cont <= 2) {
			schedule_delayed_work(&aw87339->ram_work,
					      msecs_to_jiffies(ram_timer_val));
			pr_info("%s:restart hrtimer to load firmware\n",
				__func__);
		}
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw87339_voicespk_name,
		cont ? cont->size : 0);

	for (i = 0; i < cont->size; i = i + 2) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n",
			__func__, *(cont->data + i), *(cont->data + i + 1));
	}
	/* aw87339 ram update */
	aw87339_voicespk_cnt = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!aw87339_voicespk_cnt) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw87339_voicespk_cnt->len = cont->size;
	memcpy(aw87339_voicespk_cnt->data, cont->data, cont->size);
	release_firmware(cont);
	aw87339->voicespk_cfg_update_flag = 1;

	pr_info("%s: all fw update complete\n", __func__);
}

static int aw87339_voicespk_update(struct aw87339 *aw87339)
{
	pr_info("%s enter\n", __func__);
	return request_firmware_nowait(THIS_MODULE,
				       FW_ACTION_HOTPLUG,
				       aw87339_voicespk_name,
				       &aw87339->i2c_client->dev,
				       GFP_KERNEL,
				       aw87339, aw87339_voicespk_cfg_loaded);
}

static void aw87339_cfg_work_routine(struct work_struct *work)
{
	pr_info("%s enter\n", __func__);
	if (aw87339->kspk_cfg_update_flag == 0)
		aw87339_kspk_update(aw87339);
	if (0) /*aw87339->drcv_cfg_update_flag == 0)*/
		aw87339_drcv_update(aw87339);
	if (0) /*aw87339->abrcv_cfg_update_flag == 0)*/
		aw87339_abrcv_update(aw87339);
	if (0) /*aw87339->rcvspk_cfg_update_flag == 0)*/
		aw87339_rcvspk_update(aw87339);
	if (aw87339->voicespk_cfg_update_flag == 0)
		aw87339_voicespk_update(aw87339);
}
#endif


static int aw87339_cfg_init(struct aw87339 *aw87339)
{
	int ret = -1;
#ifdef AWINIC_CFG_UPDATE_DELAY
	int cfg_timer_val = 5000;

	INIT_DELAYED_WORK(&aw87339->ram_work, aw87339_cfg_work_routine);
	schedule_delayed_work(&aw87339->ram_work,
				msecs_to_jiffies(cfg_timer_val));
	ret = 0;
#else
	ret = request_firmware_nowait(THIS_MODULE,
					FW_ACTION_HOTPLUG,
					aw87339_kspk_name,
					&aw87339->i2c_client->dev,
					GFP_KERNEL,
					aw87339,
					aw87339_kspk_cfg_loaded);
	if (ret) {
		pr_err("%s: request_firmware_nowait failed with read %s.\n",
				__func__, aw87339_kspk_name);
	}
#endif
	return ret;
}

/**********************************************************
 * aw87339 attribute
***********************************************************/
static ssize_t aw87339_get_reg(struct device *cd,
		struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW87339_REG_MAX; i++) {
		if (aw87339_reg_access[i] & REG_RD_ACCESS) {
			aw87339_i2c_read(aw87339, i, &reg_val);
			len += snprintf(buf+len, PAGE_SIZE-len,
					"reg:0x%02x=0x%02x\n", i, reg_val);
		}
	}

	return len;
}

static ssize_t aw87339_set_reg(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[2] = {0, 0};

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		aw87339_i2c_write(aw87339, databuf[0], databuf[1]);

	return len;
}


static ssize_t aw87339_get_hwen(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "hwen: %d\n",
			aw87339->hwen_flag);

	return len;
}

static ssize_t aw87339_set_hwen(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	ssize_t ret;
	unsigned int state;

	ret = kstrtouint(buf, 10, &state);
	if (ret)
		goto out_strtoint;
	if (state == 0)
		aw87339_hw_off(aw87339);
	else
		aw87339_hw_on(aw87339);

	return len;

 out_strtoint:
	dev_err(&aw87339->i2c_client->dev,
		"%s: fail to change str to int\n", __func__);
	return ret;
}

static ssize_t aw87339_get_update(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	return len;
}

static ssize_t aw87339_set_update(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	ssize_t ret;
	unsigned int state;
	int cfg_timer_val = 10;

	ret = kstrtouint(buf, 10, &state);
	if (ret)
		goto out_strtoint;
	if (state == 0) {
	} else {
		aw87339->kspk_cfg_update_flag = 0;
		aw87339->drcv_cfg_update_flag = 0;
		aw87339->abrcv_cfg_update_flag = 0;
		aw87339->rcvspk_cfg_update_flag = 0;
		aw87339->voicespk_cfg_update_flag = 0;
		schedule_delayed_work(&aw87339->ram_work,
					msecs_to_jiffies(cfg_timer_val));
	}

	return len;

 out_strtoint:
	dev_err(&aw87339->i2c_client->dev,
		"%s: fail to change str to int\n", __func__);
	return ret;
}

static ssize_t aw87339_get_mode(struct device *cd,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "0: off mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "1: kspk mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "2: drcv mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "3: abrcv mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "4: rcvspk mode\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "5: voicespk mode\n");

	return len;
}

static ssize_t aw87339_set_mode(struct device *cd,
		struct device_attribute *attr, const char *buf, size_t len)
{
	ssize_t ret;
	unsigned int state;

	ret = kstrtouint(buf, 10, &state);
	if (ret)
		goto out_strtoint;
	if (state == 0)
		aw87339_audio_off();
	else if (state == 1)
		aw87339_audio_kspk();
	else if (state == 2)
		aw87339_audio_drcv();
	else if (state == 3)
		aw87339_audio_abrcv();
	else if (state == 4)
		aw87339_audio_rcvspk();
	else if (state == 5)
		aw87339_audio_voicespk();
	else
		aw87339_audio_off();

	if (ret < 0)
		goto out;

	return len;
 out:
	dev_err(&aw87339->i2c_client->dev,
		"%s: i2c access fail to register\n", __func__);
 out_strtoint:
	dev_err(&aw87339->i2c_client->dev,
		"%s: fail to change str to int\n", __func__);
	return ret;
}

static DEVICE_ATTR(reg, 0664, aw87339_get_reg,  aw87339_set_reg);
static DEVICE_ATTR(hwen, 0660, aw87339_get_hwen,  aw87339_set_hwen);
static DEVICE_ATTR(update, 0660, aw87339_get_update,  aw87339_set_update);
static DEVICE_ATTR(mode, 0660, aw87339_get_mode,  aw87339_set_mode);

static struct attribute *aw87339_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_hwen.attr,
	&dev_attr_update.attr,
	&dev_attr_mode.attr,
	NULL
};

static struct attribute_group aw87339_attribute_group = {
	.attrs = aw87339_attributes
};


/*****************************************************
 * device tree
 *****************************************************/
static int aw87339_parse_dt(struct device *dev, struct device_node *np)
{
	aw87339->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw87339->reset_gpio < 0) {
		dev_err(dev,
			"%s: no reset gpio provided .\n", __func__);
		return -1;
	}
	dev_info(dev, "%s: reset gpio provided ok\n", __func__);

	return 0;
}

int aw87339_hw_reset(struct aw87339 *aw87339)
{
	pr_info("%s enter\n", __func__);

	if (aw87339 && gpio_is_valid(aw87339->reset_gpio)) {
		gpio_set_value_cansleep(aw87339->reset_gpio, 0);
		mdelay(2);
		gpio_set_value_cansleep(aw87339->reset_gpio, 1);
		mdelay(2);
		aw87339->hwen_flag = 1;
	} else {
		aw87339->hwen_flag = 0;
		dev_err(&aw87339->i2c_client->dev, "%s:  failed\n", __func__);
	}
	return 0;
}

/*****************************************************
 * check chip id
 *****************************************************/
int aw87339_read_chipid(struct aw87339 *aw87339)
{
	unsigned int cnt = 0;
	int ret = -1;
	unsigned char reg_val = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw87339_i2c_read(aw87339, AW87339_REG_CHIPID, &reg_val);
		if (reg_val == AW87339_CHIPID) {
			pr_info("%s This Chip is  AW87339 chipid=0x%x\n",
				__func__, reg_val);
			return 0;
		}
		cnt++;

		mdelay(AW_READ_CHIPID_RETRY_DELAY);
	}
	pr_info("%s: aw87339 chipid=0x%x error\n", __func__, reg_val);
	return -EINVAL;
}



/*********************************************************
 * aw87339 i2c driver
 *********************************************************/
static int
aw87339_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	int ret = -1;

	pr_info("%s Enter\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: check_functionality failed\n",
		__func__);
		ret = -ENODEV;
		goto exit_check_functionality_failed;
	}

	aw87339 = devm_kzalloc(&client->dev,
				sizeof(struct aw87339),
				GFP_KERNEL);
	if (aw87339 == NULL) {
		ret = -ENOMEM;
		goto exit_devm_kzalloc_failed;
	}

	aw87339->i2c_client = client;
	i2c_set_clientdata(client, aw87339);

	/* aw87339 rst */
	if (np) {
		ret = aw87339_parse_dt(&client->dev, np);
		if (ret) {
			dev_err(&client->dev,
			"%s: failed to parse device tree node\n", __func__);
			goto exit_gpio_get_failed;
		}
	} else {
		aw87339->reset_gpio = -1;
	}

	if (gpio_is_valid(aw87339->reset_gpio)) {
		ret = devm_gpio_request_one(&client->dev, aw87339->reset_gpio,
			GPIOF_OUT_INIT_LOW, "aw87339_rst");
		if (ret) {
			dev_err(&client->dev, "%s: rst request failed\n",
			__func__);
			goto exit_gpio_request_failed;
		}
	}

	/* hardware reset */
	aw87339_hw_reset(aw87339);

	/* aw87339 chip id */
	ret = aw87339_read_chipid(aw87339);
	if (ret < 0) {
		dev_err(&client->dev, "%s:aw87339_read_chipid failed ret=%d\n",
		__func__, ret);
		goto exit_i2c_check_id_failed;
	}

	ret = sysfs_create_group(&client->dev.kobj, &aw87339_attribute_group);
	if (ret < 0) {
		dev_info(&client->dev, "%s error creating sysfs attr files\n",
			__func__);
	}

	aw87339_probed = 1;

	/* aw87339 cfg update */
	kspk_load_cont = 0;
	drcv_load_cont = 0;
	abrcv_load_cont = 0;
	rcvspk_load_cont = 0;
	voicespk_load_cont = 0;
	aw87339->kspk_cfg_update_flag = 0;
	aw87339->drcv_cfg_update_flag = 0;
	aw87339->abrcv_cfg_update_flag = 0;
	aw87339->rcvspk_cfg_update_flag = 0;
	aw87339->voicespk_cfg_update_flag = 0;
	aw87339_cfg_init(aw87339);

	/* aw87339 hardware off */
	aw87339_hw_off(aw87339);

	return 0;

exit_i2c_check_id_failed:
	gpio_set_value_cansleep(aw87339->reset_gpio, 0);
	devm_gpio_free(&client->dev, aw87339->reset_gpio);
exit_gpio_request_failed:
exit_gpio_get_failed:
	devm_kfree(&client->dev, aw87339);
	aw87339 = NULL;
exit_devm_kzalloc_failed:
exit_check_functionality_failed:
	return ret;
}

static int aw87339_i2c_remove(struct i2c_client *client)
{
	struct aw87339 *aw87339 = i2c_get_clientdata(client);

	if (gpio_is_valid(aw87339->reset_gpio))
		devm_gpio_free(&client->dev, aw87339->reset_gpio);

	return 0;
}

static const struct i2c_device_id aw87339_i2c_id[] = {
	{ AW87339_I2C_NAME, 0 },
	{ }
};


static const struct of_device_id extpa_of_match[] = {
	{.compatible = "awinic,aw87339_pa"},
	{},
};


static struct i2c_driver aw87339_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = AW87339_I2C_NAME,
		.of_match_table = extpa_of_match,
	},
	.probe = aw87339_i2c_probe,
	.remove = aw87339_i2c_remove,
	.id_table	= aw87339_i2c_id,
};

#if 1
#else
static int __init aw87339_pa_init(void)
{
	int ret;

	pr_info("%s enter\n", __func__);
	pr_info("%s: driver version: %s\n", __func__, AW87339_DRIVER_VERSION);

	ret = i2c_add_driver(&aw87339_i2c_driver);
	if (ret) {
		pr_info("****[%s] Unable to register driver (%d)\n",
				__func__, ret);
		return ret;
	}
	return 0;
}

static void __exit aw87339_pa_exit(void)
{
	pr_info("%s enter\n", __func__);
	i2c_del_driver(&aw87339_i2c_driver);
}

module_init(aw87339_pa_init);
module_exit(aw87339_pa_exit);
#endif
module_i2c_driver(aw87339_i2c_driver);

MODULE_AUTHOR("<zhangzeta@awinic.com.cn>");
MODULE_DESCRIPTION("AWINIC aw87339 PA driver");
MODULE_LICENSE("GPL v2");

