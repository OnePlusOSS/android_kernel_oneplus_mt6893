/*
 * aw87319_audio.c   aw87319 pa module
 *
 * Version: v1.2.2
 *
 * Copyright (c) 2016 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

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
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
//#include <linux/wakelock.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include "aw87319_audio.h"
/*******************************************************************************
 * aw87319 marco
 ******************************************************************************/
#define AW87319_I2C_NAME    "AW87319_PA"

#define AW87319_DRIVER_VERSION  "v1.2.2"

/* #define AWINIC_CFG_UPDATE_DELAY */

#define AW_I2C_RETRIES 5
#define AW_I2C_RETRY_DELAY 2
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 2

/*******************************************************************************
 * aw87319 variable
 ******************************************************************************/
struct aw87319_t {
	struct i2c_client *i2c_client;
	int reset_gpio;
	unsigned char init_flag;
	unsigned char hwen_flag;
	unsigned char spk_cfg_update_flag;
	unsigned char vspk_cfg_update_flag;
	unsigned char rcv_cfg_update_flag;
	struct hrtimer cfg_timer;
	struct work_struct cfg_work;
};
struct aw87319_t *aw87319;

struct aw87319_container {
	int len;
	unsigned char data[];
};
struct aw87319_container *aw87319_spk_cnt;
struct aw87319_container *aw87319_vspk_cnt;
struct aw87319_container *aw87319_rcv_cnt;

static char *aw87319_spk_name = "aw87319_spk.bin";
static char *aw87319_vspk_name = "aw87319_vspk.bin";
static char *aw87319_rcv_name = "aw87319_rcv.bin";

static unsigned char aw87319_spk_cfg_default[] = {
	0x9B, 0x07, 0x28, 0x05, 0x04, 0x0D, 0x03, 0x52, 0x28, 0x02
};

static unsigned char aw87319_vspk_cfg_default[] = {
	0x9B, 0x07, 0x28, 0x05, 0x04, 0x0D, 0x03, 0x52, 0x28, 0x02
};

static unsigned char aw87319_rcv_cfg_default[] = {
	0x9B, 0x06, 0x00, 0x05, 0x04, 0x02, 0x0B, 0x52, 0xA8, 0x03
};

/*******************************************************************************
 * i2c write and read
 ******************************************************************************/
static int i2c_write_reg(unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret =
		    i2c_smbus_write_byte_data(aw87319->i2c_client, reg_addr,
					      reg_data);
		if (ret < 0)
			pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt,
			       ret);
		else
			break;
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static unsigned char i2c_read_reg(unsigned char reg_addr)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw87319->i2c_client, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

/*******************************************************************************
 * aw87319 hardware control
 ******************************************************************************/
unsigned char aw87319_hw_on(void)
{
	pr_info("%s enter\n", __func__);

	if (aw87319 && gpio_is_valid(aw87319->reset_gpio)) {
		gpio_set_value_cansleep(aw87319->reset_gpio, 0);
		mdelay(2);
		gpio_set_value_cansleep(aw87319->reset_gpio, 1);
		mdelay(2);
		aw87319->hwen_flag = 1;
		i2c_write_reg(0x64, 0x2C);
	} else {
		dev_err(&aw87319->i2c_client->dev, "%s:  failed\n", __func__);
	}

	return 0;
}

unsigned char aw87319_hw_off(void)
{
	pr_info("%s enter\n", __func__);

	if (aw87319 && gpio_is_valid(aw87319->reset_gpio)) {
		gpio_set_value_cansleep(aw87319->reset_gpio, 0);
		mdelay(2);
		aw87319->hwen_flag = 0;
	} else {
		dev_err(&aw87319->i2c_client->dev, "%s:  failed\n", __func__);
	}
	return 0;
}

/*******************************************************************************
 * aw87319 control interface
 ******************************************************************************/
unsigned char aw87319_spk_reg_val(unsigned char reg)
{
	if (aw87319->spk_cfg_update_flag)
		return *(aw87319_spk_cnt->data + reg);
	else
		return aw87319_spk_cfg_default[reg];
}

unsigned char aw87319_vspk_reg_val(unsigned char reg)
{
	if (aw87319->vspk_cfg_update_flag)
		return *(aw87319_vspk_cnt->data + reg);
	else
		return aw87319_vspk_cfg_default[reg];
}

unsigned char aw87319_rcv_reg_val(unsigned char reg)
{
	if (aw87319->rcv_cfg_update_flag)
		return *(aw87319_rcv_cnt->data + reg);
	else
		return aw87319_rcv_cfg_default[reg];
}

unsigned char aw87319_audio_receiver(void)
{
	if (aw87319 == NULL)
		return 2;
	if (!aw87319->init_flag)
		return 1;
	if (!aw87319->hwen_flag)
		aw87319_hw_on();

	i2c_write_reg(0x01, aw87319_rcv_reg_val(0x01) & 0xFB);
	/* Class D Enable; Boost Disable */

	i2c_write_reg(0x05, aw87319_rcv_reg_val(0x05));	/* Gain */

	i2c_write_reg(0x01, aw87319_rcv_reg_val(0x01));
	/* CHIP Enable; Class D Enable; Boost Disable */

	return 0;
}

unsigned char aw87319_audio_speaker(void)
{
	if (aw87319 == NULL){
		return 2;
	}
	if (!aw87319->init_flag) {
		return 1;
        }
	if (!aw87319->hwen_flag)
		aw87319_hw_on();

	i2c_write_reg(0x01, aw87319_spk_reg_val(0x01) & 0xFB);
	/* Class D Enable; Boost Enable */
	i2c_write_reg(0x02, aw87319_spk_reg_val(0x02));	/* BATSAFE */
	i2c_write_reg(0x03, aw87319_spk_reg_val(0x03));	/* BOV */
	i2c_write_reg(0x04, aw87319_spk_reg_val(0x04));	/* BP */
	i2c_write_reg(0x05, aw87319_spk_reg_val(0x05));	/* Gain */
	i2c_write_reg(0x06, aw87319_spk_reg_val(0x06));	/* AGC3_Po */
	i2c_write_reg(0x07, aw87319_spk_reg_val(0x07));	/* AGC3 */
	i2c_write_reg(0x08, aw87319_spk_reg_val(0x08));	/* AGC2 */
	i2c_write_reg(0x09, aw87319_spk_reg_val(0x09));	/* AGC1 */
	i2c_write_reg(0x01, aw87319_spk_reg_val(0x01));
	/* CHIP Enable; Class D Enable; Boost Enable */

	return 0;
}

unsigned char aw87319_audio_voice_speaker(void)
{
	if (aw87319 == NULL)
		return 2;
	if (!aw87319->init_flag)
		return 1;
	if (!aw87319->hwen_flag)
		aw87319_hw_on();

	i2c_write_reg(0x01, aw87319_vspk_reg_val(0x01) & 0xFB);
	/*lass D Enable; Boost Enable */
	i2c_write_reg(0x02, aw87319_vspk_reg_val(0x02));	/* BATSAFE */
	i2c_write_reg(0x03, aw87319_vspk_reg_val(0x03));	/* BOV */
	i2c_write_reg(0x04, aw87319_vspk_reg_val(0x04));	/* BP */
	i2c_write_reg(0x05, aw87319_vspk_reg_val(0x05));	/* Gain */
	i2c_write_reg(0x06, aw87319_vspk_reg_val(0x06));	/* AGC3_Po */
	i2c_write_reg(0x07, aw87319_vspk_reg_val(0x07));	/* AGC3 */
	i2c_write_reg(0x08, aw87319_vspk_reg_val(0x08));	/* AGC2 */
	i2c_write_reg(0x09, aw87319_vspk_reg_val(0x09));	/* AGC1 */
	i2c_write_reg(0x01, aw87319_vspk_reg_val(0x01));
	/* CHIP Enable; Class D Enable; Boost Enable */

	return 0;
}

unsigned char aw87319_audio_off(void)
{
	if (aw87319 == NULL)
		return 2;
	if (!aw87319->init_flag)
		return 1;
	if (aw87319->hwen_flag)
		i2c_write_reg(0x01, 0x00);	/* CHIP Disable */
	aw87319_hw_off();

	return 0;
}

/*******************************************************************************
 * aw87319 firmware cfg update
 ******************************************************************************/
static void aw87319_voice_speaker_cfg_loaded(const struct firmware *cont,
					     void *context)
{
	unsigned int i;

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw87319_vspk_name);
		release_firmware(cont);
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw87319_vspk_name,
		cont ? cont->size : 0);

	for (i = 0; i < cont->size; i++) {
		pr_info("%s: cont: addr:0x%02x, data:0x%02x\n", __func__, i,
			*(cont->data + i));
	}

	aw87319_vspk_cnt = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!aw87319_vspk_cnt) {
		release_firmware(cont);
		pr_err("%s: error allocating memory\n", __func__);
		return;
	}
	aw87319_vspk_cnt->len = cont->size;
	memcpy(aw87319_vspk_cnt->data, cont->data, cont->size);
	release_firmware(cont);

	for (i = 0; i < aw87319_vspk_cnt->len; i++) {
		pr_info("%s: vspk_cnt: addr:0x%02x, data:0x%02x\n", __func__, i,
			*(aw87319_vspk_cnt->data + i));
	}

	aw87319->vspk_cfg_update_flag = 1;
}

static void aw87319_receiver_cfg_loaded(const struct firmware *cont,
					void *context)
{
	unsigned int i;
	int ret;

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw87319_rcv_name);
		release_firmware(cont);
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw87319_rcv_name,
		cont ? cont->size : 0);

	for (i = 0; i < cont->size; i++) {
		pr_info("%s: cont: addr:0x%02x, data:0x%02x\n", __func__, i,
			*(cont->data + i));
	}

	aw87319_rcv_cnt = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!aw87319_rcv_cnt) {
		release_firmware(cont);
		pr_err("%s: error allocating memory\n", __func__);
		return;
	}
	aw87319_rcv_cnt->len = cont->size;
	memcpy(aw87319_rcv_cnt->data, cont->data, cont->size);
	release_firmware(cont);

	for (i = 0; i < aw87319_rcv_cnt->len; i++) {
		pr_info("%s: rcv_cnt: addr:0x%02x, data:0x%02x\n", __func__, i,
			*(aw87319_rcv_cnt->data + i));
	}

	for (i = 0; i < aw87319_rcv_cnt->len; i++) {
		pr_info("%s: rcv_cnt: addr:0x%02x, data:0x%02x\n", __func__, i,
			aw87319_rcv_reg_val(i));
	}

	aw87319->rcv_cfg_update_flag = 1;
	ret =
	    request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				    aw87319_vspk_name,
				    &aw87319->i2c_client->dev, GFP_KERNEL, NULL,
				    aw87319_voice_speaker_cfg_loaded);
	if (ret) {
		aw87319->vspk_cfg_update_flag = 0;
		pr_err("%s: request_firmware_nowait failed with read %s",
		       __func__, aw87319_vspk_name);
	}
}

static void aw87319_speaker_cfg_loaded(const struct firmware *cont,
				       void *context)
{
	unsigned int i;
	int ret;

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw87319_spk_name);
		release_firmware(cont);
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw87319_spk_name,
		cont ? cont->size : 0);

	for (i = 0; i < cont->size; i++) {
		pr_info("%s: cont: addr:0x%02x, data:0x%02x\n", __func__, i,
			*(cont->data + i));
	}

	aw87319_spk_cnt = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!aw87319_spk_cnt) {
		release_firmware(cont);
		pr_err("%s: error allocating memory\n", __func__);
		return;
	}
	aw87319_spk_cnt->len = cont->size;
	memcpy(aw87319_spk_cnt->data, cont->data, cont->size);
	release_firmware(cont);

	for (i = 0; i < aw87319_spk_cnt->len; i++) {
		pr_info("%s: spk_cnt: addr:0x%02x, data:0x%02x\n", __func__, i,
			*(aw87319_spk_cnt->data + i));
	}

	aw87319->spk_cfg_update_flag = 1;

	ret = request_firmware_nowait(THIS_MODULE,
								  FW_ACTION_HOTPLUG,
				    			  aw87319_rcv_name,
								  &aw87319->i2c_client->dev,
				    			  GFP_KERNEL,
								  NULL,
				    			  aw87319_receiver_cfg_loaded);
	if (ret) {
		aw87319->rcv_cfg_update_flag = 0;
		pr_err("%s: request_firmware_nowait failed with read %s",
		       __func__, aw87319_rcv_name);
	}
}

#ifdef AWINIC_CFG_UPDATE_DELAY
static enum hrtimer_restart cfg_timer_func(struct hrtimer *timer)
{
	pr_info("%s enter\n", __func__);

	schedule_work(&aw87319->cfg_work);

	return HRTIMER_NORESTART;
}

static void cfg_work_routine(struct work_struct *work)
{
	int ret = -1;

	pr_info("%s enter\n", __func__);

	ret =
	    request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				    aw87319_spk_name, &aw87319->i2c_client->dev,
				    GFP_KERNEL, NULL,
				    aw87319_speaker_cfg_loaded);
	if (ret) {
		pr_err("%s: request_firmware_nowait failed with read %s",
		       __func__, aw87319_spk_name);
	}

}
#endif

static int aw87319_cfg_init(void)
{
	int ret = -1;
#ifdef AWINIC_CFG_UPDATE_DELAY
	int cfg_timer_val = 5000;

	hrtimer_init(&aw87319->cfg_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw87319->cfg_timer.function = cfg_timer_func;
	INIT_WORK(&aw87319->cfg_work, cfg_work_routine);
	hrtimer_start(&aw87319->cfg_timer,
		      ktime_set(cfg_timer_val / 1000,
				(cfg_timer_val % 1000) * 1000000),
		      HRTIMER_MODE_REL);
	ret = 0;
#else
	ret =
	    request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				    aw87319_spk_name, &aw87319->i2c_client->dev,
				    GFP_KERNEL, NULL,
				    aw87319_speaker_cfg_loaded);
	if (ret) {
		pr_err("%s: request_firmware_nowait failed with read %s",
		       __func__, aw87319_spk_name);
	}
#endif
	return ret;
}

/*******************************************************************************
 * aw87319 attribute
 ******************************************************************************/
static ssize_t aw87319_get_reg(struct device *cd, struct device_attribute *attr,
			       char *buf)
{
	unsigned char reg_val;
	ssize_t len = 0;
	u8 i;

	for (i = 0; i < 0x10; i++) {
		reg_val = i2c_read_reg(i);
		len +=
		    snprintf(buf + len, PAGE_SIZE - len, "reg:0x%02x=0x%02x, ",
			     i, reg_val);
	}

	return len;
}

static ssize_t aw87319_set_reg(struct device *cd, struct device_attribute *attr,
			       const char *buf, size_t len)
{
	unsigned int databuf[2];

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		i2c_write_reg(databuf[0], databuf[1]);
	return len;
}

static ssize_t aw87319_get_hwen(struct device *cd,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "hwen: %d\n",
		     aw87319->hwen_flag);

	return len;
}

static ssize_t aw87319_set_hwen(struct device *cd,
				struct device_attribute *attr, const char *buf,
				size_t len)
{
	unsigned int databuf[16];
	unsigned int ret = 0;

	ret = kstrtouint(buf, 10, &databuf[0]);
	if (ret == 0)
		return ret;

	if (databuf[0] == 0)	/* OFF */
		aw87319_hw_off();
	else			/* ON  */
		aw87319_hw_on();

	return len;
}

static ssize_t aw87319_get_update(struct device *cd,
				  struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	return len;
}

static ssize_t aw87319_set_update(struct device *cd,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	unsigned int databuf[16];
	int ret;

	ret = kstrtouint(buf, 10, &databuf[0]);
	if (ret == 0)
		return ret;
	if (databuf[0] == 0) {
	} else {
		aw87319->spk_cfg_update_flag = 0;
		aw87319->vspk_cfg_update_flag = 0;
		aw87319->rcv_cfg_update_flag = 0;
		ret =
		    request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
					    aw87319_spk_name,
					    &aw87319->i2c_client->dev,
					    GFP_KERNEL, NULL,
					    aw87319_speaker_cfg_loaded);
		if (ret) {

			pr_err
			    ("%s: request_firmware_nowait failed with read %s",
			     __func__, aw87319_spk_name);
		}
	}

	return len;
}

static ssize_t aw87319_get_mode(struct device *cd,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "0: off mode\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "1: spk mode\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "2: rcv mode\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "3: vspk mode\n");

	return len;
}

static ssize_t aw87319_set_mode(struct device *cd,
				struct device_attribute *attr, const char *buf,
				size_t len)
{
	unsigned int databuf[16];
	unsigned int ret = 0;

	ret = kstrtouint(buf, 10, &databuf[0]);
	if (ret == 0)
		return ret;
	if (databuf[0] == 0)
		aw87319_audio_off();
	else if (databuf[0] == 1)
		aw87319_audio_speaker();
	else if (databuf[0] == 2)
		aw87319_audio_receiver();
	else if (databuf[0] == 3)
		aw87319_audio_voice_speaker();
	else
		aw87319_audio_off();
	return len;
}

static DEVICE_ATTR(reg, 0660, aw87319_get_reg, aw87319_set_reg);
static DEVICE_ATTR(hwen, 0660, aw87319_get_hwen, aw87319_set_hwen);
static DEVICE_ATTR(update, 0660, aw87319_get_update, aw87319_set_update);
static DEVICE_ATTR(mode, 0660, aw87319_get_mode, aw87319_set_mode);

static struct attribute *aw87319_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_hwen.attr,
	&dev_attr_update.attr,
	&dev_attr_mode.attr,
	NULL
};

static struct attribute_group aw87319_attribute_group = {
	.attrs = aw87319_attributes
};

/*****************************************************
 * device tree
 *****************************************************/
static int aw87319_parse_dt(struct device *dev, struct device_node *np)
{
	aw87319->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw87319->reset_gpio >= 0) {
		dev_info(dev, "%s: reset gpio provided ok\n", __func__);
	} else {
		dev_err(dev,
			"%s: no reset gpio provided, will not HW reset device\n",
			__func__);
		return -1;
	}
	return 0;
}

int aw87319_hw_reset(void)
{
	pr_info("%s enter\n", __func__);

	if (aw87319 && gpio_is_valid(aw87319->reset_gpio)) {
		gpio_set_value_cansleep(aw87319->reset_gpio, 0);
		mdelay(2);
		gpio_set_value_cansleep(aw87319->reset_gpio, 1);
		mdelay(2);
		aw87319->hwen_flag = 1;
	} else {
		aw87319->hwen_flag = 0;
		dev_err(&aw87319->i2c_client->dev, "%s:  failed\n", __func__);
	}
	return 0;
}

/*****************************************************
 * check chip id
 *****************************************************/
int aw87319_read_chipid(void)
{
	unsigned int cnt = 0;
	unsigned int reg = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		i2c_write_reg(0x64, 0x2C);
		reg = i2c_read_reg(0x00);
		if (reg != 0x9B) {
			pr_info("%s: aw87319 chipid=0x%x error\n", __func__,
				reg);
		} else {
			pr_info("%s aw87319 chipid=0x%x\n", __func__, reg);
			return 0;
		}
		cnt++;

		msleep(AW_READ_CHIPID_RETRY_DELAY);
	}

	return -EINVAL;
}

/*******************************************************************************
 * aw87319 i2c driver
 ******************************************************************************/
static int aw87319_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
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

	aw87319 =
	    devm_kzalloc(&client->dev, sizeof(struct aw87319_t), GFP_KERNEL);
	if (aw87319 == NULL) {
		ret = -ENOMEM;
		goto exit_devm_kzalloc_failed;
	}

	aw87319->i2c_client = client;
	i2c_set_clientdata(client, aw87319);

	/* aw87319 rst */
	if (np) {
		ret = aw87319_parse_dt(&client->dev, np);
		if (ret) {
			dev_err(&client->dev,
				"%s: failed to parse device tree node\n",
				__func__);
			goto exit_gpio_get_failed;
		}
	} else {
		aw87319->reset_gpio = -1;
	}

	if (gpio_is_valid(aw87319->reset_gpio)) {
		ret = devm_gpio_request_one(&client->dev, aw87319->reset_gpio,
					    GPIOF_OUT_INIT_LOW, "aw87319_rst");
		if (ret) {
			dev_err(&client->dev, "%s: rst request failed\n",
				__func__);
			goto exit_gpio_request_failed;
		}
	}

	/* hardware reset */
	aw87319_hw_reset();

	/* aw87319 chip id */
	ret = aw87319_read_chipid();
	if (ret < 0) {
		dev_err(&client->dev, "%s: aw87319_read_chipid failed ret=%d\n",
			__func__, ret);
		goto exit_i2c_check_id_failed;
	}

	ret = sysfs_create_group(&client->dev.kobj, &aw87319_attribute_group);
	if (ret < 0) {
		dev_info(&client->dev, "%s error creating sysfs attr files\n",
			 __func__);
	}

	/* aw87319 cfg update */
	aw87319->spk_cfg_update_flag = 0;
	aw87319->vspk_cfg_update_flag = 0;
	aw87319->rcv_cfg_update_flag = 0;
	aw87319_cfg_init();

	/* aw87319 hardware off */
	aw87319_hw_off();

	aw87319->init_flag = 1;

	return 0;

 exit_i2c_check_id_failed:
	devm_gpio_free(&client->dev, aw87319->reset_gpio);
 exit_gpio_request_failed:
 exit_gpio_get_failed:
	devm_kfree(&client->dev, aw87319);
	aw87319 = NULL;
 exit_devm_kzalloc_failed:
 exit_check_functionality_failed:
	return ret;
}

static int aw87319_i2c_remove(struct i2c_client *client)
{
	if (gpio_is_valid(aw87319->reset_gpio))
		devm_gpio_free(&client->dev, aw87319->reset_gpio);

	return 0;
}

static const struct i2c_device_id aw87319_i2c_id[] = {
	{AW87319_I2C_NAME, 0},
	{}
};

static const struct of_device_id extpa_of_match[] = {
	{.compatible = "awinic,aw87319_pa"},
	{},
};

static struct i2c_driver aw87319_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = AW87319_I2C_NAME,
		   .of_match_table = extpa_of_match,
		   },
	.probe = aw87319_i2c_probe,
	.remove = aw87319_i2c_remove,
	.id_table = aw87319_i2c_id,
};

static int __init aw87319_pa_init(void)
{
	int ret;

	pr_info("%s enter\n", __func__);
	pr_info("%s: driver version: %s\n", __func__, AW87319_DRIVER_VERSION);

	ret = i2c_add_driver(&aw87319_i2c_driver);
	if (ret) {
		pr_info("****[%s] Unable to register driver (%d)\n", __func__,
			ret);
		return ret;
	}
	return 0;
}

static void __exit aw87319_pa_exit(void)
{
	pr_info("%s enter\n", __func__);
	i2c_del_driver(&aw87319_i2c_driver);
}

module_init(aw87319_pa_init);
module_exit(aw87319_pa_exit);

MODULE_AUTHOR("<liweilei@awinic.com.cn>");
MODULE_DESCRIPTION("AWINIC AW87319 PA driver");
MODULE_LICENSE("GPL v2");
