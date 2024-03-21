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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"
/* device tree should be defined in flashlight-dt.h */
#ifndef ATOM_DTNAME
#define ATOM_DTNAME "mediatek,flashlights_atom"
#endif
#ifndef ATOM_DTNAME_I2C
#define ATOM_DTNAME_I2C   "mediatek,strobe_main"
#endif

#define ATOM_NAME "flashlights-atom"

/* define registers */
#define ATOM_REG_ENABLE           (0x01)
#define ATOM_MASK_ENABLE_LED1     (0x01)
#define ATOM_MASK_ENABLE_LED2     (0x02)
#define ATOM_DISABLE              (0x00)
#define ATOM_TORCH_MODE           (0x08)
#define ATOM_FLASH_MODE           (0x0C)
#define ATOM_ENABLE_LED1          (0x01)
#define ATOM_ENABLE_LED1_TORCH    (0x09)
#define ATOM_ENABLE_LED1_FLASH    (0x0D)
#define ATOM_ENABLE_LED2          (0x02)
#define ATOM_ENABLE_LED2_TORCH    (0x0A)
#define ATOM_ENABLE_LED2_FLASH    (0x0E)

#define ATOM_REG_TORCH_LEVEL_LED1 (0x05)
#define ATOM_REG_FLASH_LEVEL_LED1 (0x03)
#define ATOM_REG_TORCH_LEVEL_LED2 (0x06)
#define ATOM_REG_FLASH_LEVEL_LED2 (0x04)

#define ATOM_REG_TIMING_CONF      (0x08)
#define ATOM_TORCH_RAMP_TIME      (0x10)
#define ATOM_FLASH_TIMEOUT        (0x0F)

#define ATOM_AW36515_SOFT_RESET_ENABLE (0x80)
#define ATOM_AW36515_REG_BOOST_CONFIG (0x07)


/* define channel, level */
#define ATOM_CHANNEL_NUM          2
#define ATOM_CHANNEL_CH1          0
#define ATOM_CHANNEL_CH2          1
/* define level */
#define ATOM_LEVEL_NUM 28
#define ATOM_LEVEL_TORCH 7

#define ATOM_HW_TIMEOUT 400 /* ms */

/* define mutex and work queue */
static DEFINE_MUTEX(atom_mutex);
static struct work_struct atom_work_ch1;
static struct work_struct atom_work_ch2;

/* define pinctrl */
#define ATOM_PINCTRL_PIN_HWEN 0
#define ATOM_PINCTRL_PINSTATE_LOW 0
#define ATOM_PINCTRL_PINSTATE_HIGH 1
#define ATOM_PINCTRL_STATE_HWEN_HIGH "atom_hwen_high"
#define ATOM_PINCTRL_STATE_HWEN_LOW  "atom_hwen_low"

/* define device id */
#define USE_AW36515_IC  0x1111

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t atom_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t atom_set_reg(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t atom_set_hwen(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t atom_get_hwen(struct device* cd, struct device_attribute *attr, char* buf);

static DEVICE_ATTR(reg, 0660, atom_get_reg,  atom_set_reg);
static DEVICE_ATTR(hwen, 0660, atom_get_hwen,  atom_set_hwen);

struct i2c_client *atom_flashlight_client;

static struct pinctrl *atom_pinctrl;
static struct pinctrl_state *atom_hwen_high;
static struct pinctrl_state *atom_hwen_low;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *atom_i2c_client;

/* platform data */
struct atom_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* atom chip data */
struct atom_chip_data {
	struct i2c_client *client;
	struct atom_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};

enum FLASHLIGHT_DEVICE {
	AW36515_SM = 0x02,
};

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int atom_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	pr_err("atom_pinctrl_init start\n");
	/* get pinctrl */
	atom_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(atom_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(atom_pinctrl);
		atom_pinctrl = NULL ;
		return ret;
	}
	pr_err("devm_pinctrl_get finish %p\n", atom_pinctrl);

	/* Flashlight HWEN pin initialization */
	atom_hwen_high = pinctrl_lookup_state(atom_pinctrl, ATOM_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(atom_hwen_high)) {
		pr_err("Failed to init (%s)\n", ATOM_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(atom_hwen_high);
	}
	pr_err("pinctrl_lookup_state atom_hwen_high finish %p\n", atom_hwen_high);
	atom_hwen_low = pinctrl_lookup_state(atom_pinctrl, ATOM_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(atom_hwen_low)) {
		pr_err("Failed to init (%s)\n", ATOM_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(atom_hwen_low);
	}
	pr_err("pinctrl_lookup_state atom_hwen_low finish\n");

	return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int atom_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct atom_chip_data *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}

/* i2c wrapper function */
static int atom_read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	struct atom_chip_data *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (val < 0)
		pr_err("failed read at 0x%02x\n", reg);

	return val;
}

static int atom_pinctrl_set(int pin, int state)
{
	int ret = 0;
	unsigned char reg, val;

	if (IS_ERR(atom_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case ATOM_PINCTRL_PIN_HWEN:
		if (state == ATOM_PINCTRL_PINSTATE_LOW && !IS_ERR(atom_hwen_low)){
			reg = ATOM_REG_ENABLE;
			val = 0x00;
			atom_write_reg(atom_i2c_client, reg, val);
			//pinctrl_select_state(atom_pinctrl, atom_hwen_low);//rm to keep HWEN high
		}
		else if (state == ATOM_PINCTRL_PINSTATE_HIGH && !IS_ERR(atom_hwen_high))
			pinctrl_select_state(atom_pinctrl, atom_hwen_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_info("pin(%d) state(%d)\n", pin, state);

	return ret;
}
/******************************************************************************
 * atom operations
 *****************************************************************************/

static const int *atom_current;
static const unsigned char *atom_torch_level;
static const unsigned char *atom_flash_level;

static const int AW36515_current[ATOM_LEVEL_NUM] = {
	24,   46,   70,   93,   116,  141,  164,  199,  246,  304,
	351,  398,  445,  504,  551,  598,  656,  703,  750,  797,
	856,  903,  949,  996,  1055, 1100, 1147, 1202
};

/*Offset: 0.98mA(00000000)
Step:1.96mA
Range: 0.98mA(00000000)~500mA(11111111)*/
static const unsigned char AW36515_torch_level[ATOM_LEVEL_NUM] = {
	0x0C, 0x17, 0x23, 0x31, 0x3B, 0x47, 0x53, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*Offset: 3.91mA(00000000)
Step:7.83mA
Range: 3.91 mA(00000000)~2.0 A(11111111)*/
static const unsigned char AW36515_flash_level[ATOM_LEVEL_NUM] = {
	0x03, 0x05, 0x08, 0x0C, 0x0E, 0x12, 0x14, 0x19, 0x1F, 0x26,
	0x2C, 0x32, 0x38, 0x40, 0x46, 0x4C, 0x53, 0x59, 0x5F, 0x65,
	0x6D, 0x73, 0x79, 0x7F, 0x86, 0x8C, 0x92, 0x99
};


static volatile unsigned char atom_reg_enable;
static volatile int atom_level_ch1 = -1;
static volatile int atom_level_ch2 = -1;

static int atom_is_torch(int level)
{
	if (level >= ATOM_LEVEL_TORCH)
		return -1;

	return 0;
}

static int atom_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= ATOM_LEVEL_NUM)
		level = ATOM_LEVEL_NUM - 1;

	return level;
}

/* flashlight enable function */
static int atom_enable_ch1(void)
{
	unsigned char reg, val;

	reg = ATOM_REG_ENABLE;
	if (!atom_is_torch(atom_level_ch1)) {
		/* torch mode */
		atom_reg_enable |= ATOM_ENABLE_LED1_TORCH;
	} else {
		/* flash mode */
		atom_reg_enable |= ATOM_ENABLE_LED1_FLASH;
	}
	val = atom_reg_enable;

	return atom_write_reg(atom_i2c_client, reg, val);
}

static int atom_enable_ch2(void)
{
	unsigned char reg, val;

	reg = ATOM_REG_ENABLE;
	if (!atom_is_torch(atom_level_ch2)) {
		/* torch mode */
		atom_reg_enable |= ATOM_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		atom_reg_enable |= ATOM_ENABLE_LED2_FLASH;
	}
	val = atom_reg_enable;

	return atom_write_reg(atom_i2c_client, reg, val);
}

static int atom_enable(int channel)
{
	if (channel == ATOM_CHANNEL_CH1)
		atom_enable_ch1();
	else if (channel == ATOM_CHANNEL_CH2)
		atom_enable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}
	return 0;
}

/* flashlight disable function */
static int atom_disable_ch1(void)
{
	unsigned char reg, val;

	reg = ATOM_REG_ENABLE;
	if (atom_reg_enable & ATOM_MASK_ENABLE_LED2) {
		/* if LED 2 is enable, disable LED 1 */
		atom_reg_enable &= (~ATOM_ENABLE_LED1);
	} else {
		/* if LED 2 is enable, disable LED 1 and clear mode */
		atom_reg_enable &= (~ATOM_ENABLE_LED1_FLASH);
	}
	val = atom_reg_enable;
	return atom_write_reg(atom_i2c_client, reg, val);
}

static int atom_disable_ch2(void)
{
	unsigned char reg, val;

	reg = ATOM_REG_ENABLE;
	if (atom_reg_enable & ATOM_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		atom_reg_enable &= (~ATOM_ENABLE_LED2);
	} else {
		/* if LED 1 is enable, disable LED 2 and clear mode */
		atom_reg_enable &= (~ATOM_ENABLE_LED2_FLASH);
	}
	val = atom_reg_enable;

	return atom_write_reg(atom_i2c_client, reg, val);
}

static int atom_disable(int channel)
{
	if (channel == ATOM_CHANNEL_CH1) {
		atom_disable_ch1();
		pr_info("ATOM_CHANNEL_CH1\n");
	} else if (channel == ATOM_CHANNEL_CH2) {
		atom_disable_ch2();
		pr_info("ATOM_CHANNEL_CH2\n");
	} else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* set flashlight level */
static int atom_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val;

	level = atom_verify_level(level);

	/* set torch brightness level */
	reg = ATOM_REG_TORCH_LEVEL_LED1;
	val = atom_torch_level[level];
	ret = atom_write_reg(atom_i2c_client, reg, val);

	atom_level_ch1 = level;

	/* set flash brightness level */
	reg = ATOM_REG_FLASH_LEVEL_LED1;
	val = atom_flash_level[level];
	ret = atom_write_reg(atom_i2c_client, reg, val);

	return ret;
}

int atom_set_level_ch2(int level)
{
	int ret;
	unsigned char reg, val;

	level = atom_verify_level(level);

	/* set torch brightness level */
	reg = ATOM_REG_TORCH_LEVEL_LED2;
	val = atom_torch_level[level];
	ret = atom_write_reg(atom_i2c_client, reg, val);

	atom_level_ch2 = level;

	/* set flash brightness level */
	reg = ATOM_REG_FLASH_LEVEL_LED2;
	val = atom_flash_level[level];
	ret = atom_write_reg(atom_i2c_client, reg, val);

	return ret;
}

static int atom_set_level(int channel, int level)
{
	if (channel == ATOM_CHANNEL_CH1)
		atom_set_level_ch1(level);
	else if (channel == ATOM_CHANNEL_CH2)
		atom_set_level_ch2(level);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}
/* flashlight init */
int atom_init(void)
{
	int ret;
	unsigned char reg, val, reg_val;
	int chip_id;

	atom_pinctrl_set(ATOM_PINCTRL_PIN_HWEN, ATOM_PINCTRL_PINSTATE_HIGH);
	chip_id = atom_read_reg(atom_i2c_client, 0x0c);
	msleep(2);
	pr_info("flashlight chip id: reg:0x0c, chip_id 0x%x",chip_id);
	if ( chip_id == AW36515_SM ) {
		reg_val = atom_read_reg(atom_i2c_client, ATOM_AW36515_REG_BOOST_CONFIG);
		reg_val |= ATOM_AW36515_SOFT_RESET_ENABLE;
		pr_info("flashlight chip id: reg:0x0c, data:0x%x;boost confgiuration: reg:0x07, reg_val: 0x%x", chip_id, reg_val);
		ret = atom_write_reg(atom_i2c_client, ATOM_AW36515_REG_BOOST_CONFIG, reg_val);
		msleep(2);
	}
	/* clear enable register */
	reg = ATOM_REG_ENABLE;
	val = ATOM_DISABLE;
	ret = atom_write_reg(atom_i2c_client, reg, val);

	atom_reg_enable = val;

	/* set torch current ramp time and flash timeout */
	reg = ATOM_REG_TIMING_CONF;
	val = ATOM_TORCH_RAMP_TIME | ATOM_FLASH_TIMEOUT;
	ret = atom_write_reg(atom_i2c_client, reg, val);

	return ret;
}

/* flashlight uninit */
int atom_uninit(void)
{
	atom_disable(ATOM_CHANNEL_CH1);
	atom_disable(ATOM_CHANNEL_CH2);
	atom_pinctrl_set(ATOM_PINCTRL_PIN_HWEN, ATOM_PINCTRL_PINSTATE_LOW);

	return 0;
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer atom_timer_ch1;
static struct hrtimer atom_timer_ch2;
static unsigned int atom_timeout_ms[ATOM_CHANNEL_NUM];

static void atom_work_disable_ch1(struct work_struct *data)
{
	pr_info("ht work queue callback\n");
	atom_disable_ch1();
}

static void atom_work_disable_ch2(struct work_struct *data)
{
	pr_info("lt work queue callback\n");
	atom_disable_ch2();
}

static enum hrtimer_restart atom_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&atom_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart atom_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&atom_work_ch2);
	return HRTIMER_NORESTART;
}

int atom_timer_start(int channel, ktime_t ktime)
{
	if (channel == ATOM_CHANNEL_CH1)
		hrtimer_start(&atom_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == ATOM_CHANNEL_CH2)
		hrtimer_start(&atom_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

int atom_timer_cancel(int channel)
{
	if (channel == ATOM_CHANNEL_CH1)
		hrtimer_cancel(&atom_timer_ch1);
	else if (channel == ATOM_CHANNEL_CH2)
		hrtimer_cancel(&atom_timer_ch2);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int atom_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= ATOM_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}
	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_info("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		atom_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		atom_pinctrl_set(ATOM_PINCTRL_PIN_HWEN, ATOM_PINCTRL_PINSTATE_HIGH);
		atom_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		atom_pinctrl_set(ATOM_PINCTRL_PIN_HWEN, ATOM_PINCTRL_PINSTATE_HIGH);
		if (fl_arg->arg == 1) {
			if (atom_timeout_ms[channel]) {
				ktime = ktime_set(atom_timeout_ms[channel] / 1000,
						(atom_timeout_ms[channel] % 1000) * 1000000);
				atom_timer_start(channel, ktime);
			}
			atom_enable(channel);
		} else {
			atom_disable(channel);
			atom_timer_cancel(channel);
			atom_pinctrl_set(ATOM_PINCTRL_PIN_HWEN, ATOM_PINCTRL_PINSTATE_LOW);
		}
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_info("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = ATOM_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_info("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = ATOM_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = atom_verify_level(fl_arg->arg);
		pr_info("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = atom_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_info("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = ATOM_HW_TIMEOUT;
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int atom_open(void)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int atom_release(void)
{
	/* uninit chip and clear usage count */
/*
	mutex_lock(&atom_mutex);
	use_count--;
	if (!use_count)
		atom_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&atom_mutex);

	pr_info("Release: %d\n", use_count);
*/
	return 0;
}

static int atom_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&atom_mutex);
	if (set) {
		if (!use_count)
			ret = atom_init();
		use_count++;
		pr_info("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = atom_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_info("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&atom_mutex);

	return ret;
}

static ssize_t atom_strobe_store(struct flashlight_arg arg)
{
	atom_set_driver(1);
	atom_set_level(arg.channel, arg.level);
	atom_timeout_ms[arg.channel] = 0;
	atom_enable(arg.channel);
	msleep(arg.dur);
	atom_disable(arg.channel);
	//atom_release(NULL);
	atom_set_driver(0);
	return 0;
}

static struct flashlight_operations atom_ops = {
	atom_open,
	atom_release,
	atom_ioctl,
	atom_strobe_store,
	atom_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int atom_chip_init(struct atom_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * atom_init();
	 */

	return 0;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ATOM Debug file
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t atom_get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char reg_val;
	unsigned char i;
	ssize_t len = 0;
	for(i=0;i<0x0E;i++)
	{
		reg_val = atom_read_reg(atom_i2c_client,i);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg%2X = 0x%2X \n, ", i,reg_val);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\r\n");
	return len;
}

static ssize_t atom_set_reg(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned int databuf[2];
	if(2 == sscanf(buf,"%x %x",&databuf[0], &databuf[1]))
	{
		//i2c_write_reg(databuf[0],databuf[1]);
		atom_write_reg(atom_i2c_client,databuf[0],databuf[1]);
	}
	return len;
}

static ssize_t atom_get_hwen(struct device* cd,struct device_attribute *attr, char* buf)
{
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "//atom_hwen_on(void)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 1 > hwen\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "//atom_hwen_off(void)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 0 > hwen\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");

	return len;
}

static ssize_t atom_set_hwen(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned char databuf[16];

	sscanf(buf,"%c",&databuf[0]);
#if 1
	if(databuf[0] == 0) {			// OFF
		//atom_hwen_low();
	} else {				// ON
		//atom_hwen_high();
	}
#endif
	return len;
}

static int atom_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_reg);
	err = device_create_file(dev, &dev_attr_hwen);

	return err;
}

static int atom_parse_dt(struct device *dev,
		struct atom_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				ATOM_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int atom_chip_id(void)
{
	int chip_id;
	int reg00_id = -1;
	atom_pinctrl_set(ATOM_PINCTRL_PIN_HWEN, ATOM_PINCTRL_PINSTATE_HIGH);
	msleep(1);
	chip_id = atom_read_reg(atom_i2c_client, 0x0c);
	pr_info("flashlight chip id: reg:0x0c, data:0x%x", chip_id);
	if (chip_id == AW36515_SM) {
		reg00_id = atom_read_reg(atom_i2c_client, 0x00);
		pr_info("flashlight chip id: reg:0x00, data:0x%x", reg00_id);
		if (reg00_id == 0x30) {
			chip_id = AW36515_SM;
			pr_info("flashlight reg00_id = 0x%x, set chip_id to AW36515_SM", reg00_id);
		}
	}
	atom_pinctrl_set(ATOM_PINCTRL_PIN_HWEN, ATOM_PINCTRL_PINSTATE_LOW);
    if (chip_id == AW36515_SM){
		pr_info(" the device's flashlight driver IC is AW36515\n");
		return USE_AW36515_IC;
	} else {
		pr_err(" the device's flashlight driver IC is not used in our project!\n");
		return USE_AW36515_IC;
	}
}

static int atom_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct atom_chip_data *chip;
	struct atom_platform_data *pdata = client->dev.platform_data;
	int err;
	int i;
	int chip_id;

	pr_info("atom_i2c_probe Probe start.\n");

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct atom_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pr_err("Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct atom_platform_data), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_free;
		}
		client->dev.platform_data = pdata;
		err = atom_parse_dt(&client->dev, pdata);
		if (err)
			goto err_free;
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	atom_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&atom_work_ch1, atom_work_disable_ch1);
	INIT_WORK(&atom_work_ch2, atom_work_disable_ch2);

	/* init timer */
	hrtimer_init(&atom_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	atom_timer_ch1.function = atom_timer_func_ch1;
	hrtimer_init(&atom_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	atom_timer_ch2.function = atom_timer_func_ch2;
	atom_timeout_ms[ATOM_CHANNEL_CH1] = 100;
	atom_timeout_ms[ATOM_CHANNEL_CH2] = 100;

	/* init chip hw */
	atom_chip_init(chip);
	chip_id = atom_chip_id();
	if (chip_id == USE_AW36515_IC){
		atom_current = AW36515_current;
		atom_torch_level = AW36515_torch_level;
		atom_flash_level = AW36515_flash_level;
	}

	/* register flashlight operations */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&atom_ops)) {
                pr_err("Failed to register flashlight device.\n");
				err = -EFAULT;
				goto err_free;
			}
	} else {
		if (flashlight_dev_register(ATOM_NAME, &atom_ops)) {
			pr_err("Failed to register flashlight device.\n");
			err = -EFAULT;
			goto err_free;
		}
	}

    atom_create_sysfs(client);

	pr_info("Probe done.\n");

	return 0;

err_free:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int atom_i2c_remove(struct i2c_client *client)
{
	struct atom_platform_data *pdata = dev_get_platdata(&client->dev);
	struct atom_chip_data *chip = i2c_get_clientdata(client);
	int i;

	pr_info("Remove start.\n");

	client->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(ATOM_NAME);
	/* flush work queue */
	flush_work(&atom_work_ch1);
	flush_work(&atom_work_ch2);

	/* unregister flashlight operations */
	flashlight_dev_unregister(ATOM_NAME);

	/* free resource */
	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);

	pr_info("Remove done.\n");

	return 0;
}

static const struct i2c_device_id atom_i2c_id[] = {
	{ATOM_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id atom_i2c_of_match[] = {
	{.compatible = ATOM_DTNAME_I2C},
	{},
};
MODULE_DEVICE_TABLE(of, atom_i2c_of_match);
#endif

static struct i2c_driver atom_i2c_driver = {
	.driver = {
		   .name = ATOM_NAME,
#ifdef CONFIG_OF
		   .of_match_table = atom_i2c_of_match,
#endif
		   },
	.probe = atom_i2c_probe,
	.remove = atom_i2c_remove,
	.id_table = atom_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int atom_probe(struct platform_device *dev)
{
	pr_info("Probe start %s.\n", ATOM_DTNAME_I2C);

	/* init pinctrl */
	if (atom_pinctrl_init(dev)) {
		pr_err("Failed to init pinctrl.\n");
		return -1;
	}

	if (i2c_add_driver(&atom_i2c_driver)) {
		pr_err("Failed to add i2c driver.\n");
		return -1;
	}

	pr_info("Probe done.\n");

	return 0;
}

static int atom_remove(struct platform_device *dev)
{
	pr_info("Remove start.\n");

	i2c_del_driver(&atom_i2c_driver);

	pr_info("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id atom_of_match[] = {
	{.compatible = ATOM_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, atom_of_match);
#else
static struct platform_device atom_platform_device[] = {
	{
		.name = ATOM_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, atom_platform_device);
#endif

static struct platform_driver atom_platform_driver = {
	.probe = atom_probe,
	.remove = atom_remove,
	.driver = {
		.name = ATOM_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = atom_of_match,
#endif
	},
};

static int __init flashlight_atom_init(void)
{
	int ret;

	pr_info("flashlight_atom-Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&atom_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&atom_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_info("flashlight_atom Init done.\n");

	return 0;
}

static void __exit flashlight_atom_exit(void)
{
	pr_info("flashlight_atom-Exit start.\n");

	platform_driver_unregister(&atom_platform_driver);

	pr_info("flashlight_atom Exit done.\n");
}


module_init(flashlight_atom_init);
module_exit(flashlight_atom_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joseph <zhangzetao@awinic.com.cn>");
MODULE_DESCRIPTION("AW Flashlight ATOM Driver");

