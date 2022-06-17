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
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"
#include <mt-plat/mtk_pwm.h>

/* define device tree */
#ifndef EVEN_DTNAME
#define EVEN_DTNAME "mediatek,flashlights_even"
#endif

#define EVEN_NAME "flashlights_even"

#ifndef FLASHLIGHT_BRIGHTNESS_ADD
#define FLASHLIGHT_BRIGHTNESS_ADD
#endif

#ifdef CONFIG_PM_WAKELOCKS
#include <linux/pm_wakeup.h>
#endif

/* define registers */

/* define mutex and work queue */
static DEFINE_MUTEX(even_mutex);
static struct work_struct even_work;
//static struct pwm_spec_config pwm_setting;

#define EVEN_LED_MODE_DUTY 26
#define EVEN_TORCH_MODE_DUTY 27

#define EVEN_LEVEL_NUM 24
#define EVEN_LEVEL_TORCH 1
static int g_duty_array[EVEN_LEVEL_NUM] = {4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96};

static const int *even_current;
static const int sgm3785_current[EVEN_LEVEL_NUM] = {
        40,80,120,160,200,240,280,320,360,400,440,480,520,560,600,640,680,720,760,
        800,840,880,920,960
};

/* define pinctrl */
#define EVEN_PINCTRL_PIN_FLASH_EN 0
#define EVEN_PINCTRL_PIN_PWM_EN 1
#define EVEN_PINCTRL_PIN_PWM_GPIO 2
#define EVEN_PINCTRL_PIN_TEMP_GPIO 3
#define EVEN_PINCTRL_PIN_STATE_LOW 0
#define EVEN_PINCTRL_PIN_STATE_HIGH 1
#define EVEN_PINCTRL_STATE_FLASH_EN_HIGH "flash_light_en_pin_1"
#define EVEN_PINCTRL_STATE_FLASH_EN_LOW  "flash_light_en_pin_0"
#define EVEN_PINCTRL_STATE_PWM_GPIO_HIGH "flash_light_flash_pin_1"
#define EVEN_PINCTRL_STATE_PWM_GPIO_LOW  "flash_light_flash_pin_0"
#define EVEN_PINCTRL_STATE_PWM "flash_light_pwm_pin"
#define EVEN_PINCTRL_STATE_FLASH_TEMP_HIGH "flash_light_temp_pin_1"
#define EVEN_PINCTRL_STATE_FLASH_TEMP_LOW  "flash_light_temp_pin_0"


static struct pinctrl *even_pinctrl;
static struct pinctrl_state *even_flash_en_high;
static struct pinctrl_state *even_flash_en_low;
static struct pinctrl_state *even_pwm_gpio_high;
static struct pinctrl_state *even_pwm_gpio_low;
static struct pinctrl_state *even_flash_pwm;
static struct pinctrl_state *even_flash_temp_high;
static struct pinctrl_state *even_flash_temp_low;


/* define usage count */
static int use_count;

static int g_flash_duty = -1;

#ifdef CONFIG_PM_WAKELOCKS
struct wakeup_source flashlight_wake_lock;
#endif

/* platform data */
struct even_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int even_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	even_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(even_pinctrl)) {
		printk("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(even_pinctrl);
	}

	/*  Flashlight pin initialization */
	even_flash_en_high = pinctrl_lookup_state(
			even_pinctrl, EVEN_PINCTRL_STATE_FLASH_EN_HIGH);
	if (IS_ERR(even_flash_en_high)) {
		printk("Failed to init (%s)\n", EVEN_PINCTRL_STATE_FLASH_EN_HIGH);
		ret = PTR_ERR(even_flash_en_high);
	}
	even_flash_en_low = pinctrl_lookup_state(
			even_pinctrl, EVEN_PINCTRL_STATE_FLASH_EN_LOW);
	if (IS_ERR(even_flash_en_low)) {
		printk("Failed to init (%s)\n", EVEN_PINCTRL_STATE_FLASH_EN_LOW);
		ret = PTR_ERR(even_flash_en_low);
	}
	even_pwm_gpio_high = pinctrl_lookup_state(
			even_pinctrl, EVEN_PINCTRL_STATE_PWM_GPIO_HIGH);
	if (IS_ERR(even_pwm_gpio_high)) {
		printk("Failed to init (%s)\n", EVEN_PINCTRL_STATE_PWM_GPIO_HIGH);
		ret = PTR_ERR(even_pwm_gpio_high);
	}
	even_pwm_gpio_low = pinctrl_lookup_state(
			even_pinctrl, EVEN_PINCTRL_STATE_PWM_GPIO_LOW);
	if (IS_ERR(even_pwm_gpio_low)) {
		printk("Failed to init (%s)\n", EVEN_PINCTRL_STATE_PWM_GPIO_LOW);
		ret = PTR_ERR(even_pwm_gpio_low);
	}
	even_flash_pwm = pinctrl_lookup_state(
			even_pinctrl, EVEN_PINCTRL_STATE_PWM);
	if (IS_ERR(even_flash_pwm)) {
		printk("Failed to init (%s)\n", EVEN_PINCTRL_STATE_PWM);
		ret = PTR_ERR(even_flash_pwm);
	}
	even_flash_temp_high = pinctrl_lookup_state(
			even_pinctrl, EVEN_PINCTRL_STATE_FLASH_TEMP_HIGH);
	if (IS_ERR(even_flash_temp_high)) {
		printk("Failed to init (%s)\n", EVEN_PINCTRL_STATE_FLASH_TEMP_HIGH);
		ret = PTR_ERR(even_flash_temp_high);
	}
	even_flash_temp_low = pinctrl_lookup_state(
			even_pinctrl, EVEN_PINCTRL_STATE_FLASH_TEMP_LOW);
	if (IS_ERR(even_flash_temp_low)) {
		printk("Failed to init (%s)\n", EVEN_PINCTRL_STATE_FLASH_TEMP_LOW);
		ret = PTR_ERR(even_flash_temp_low);
	}

	return ret;
}

static int even_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(even_pinctrl)) {
		printk("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
		case EVEN_PINCTRL_PIN_FLASH_EN:
			if (state == EVEN_PINCTRL_PIN_STATE_LOW &&!IS_ERR(even_flash_en_low))
				ret = pinctrl_select_state(even_pinctrl, even_flash_en_low);
			else if (state == EVEN_PINCTRL_PIN_STATE_HIGH &&!IS_ERR(even_flash_en_high))
				ret = pinctrl_select_state(even_pinctrl, even_flash_en_high);
			else
				printk("set err, pin(%d) state(%d)\n", pin, state);
			break;
		case EVEN_PINCTRL_PIN_PWM_EN:
			ret =pinctrl_select_state(even_pinctrl, even_flash_pwm);
			break;
		case EVEN_PINCTRL_PIN_PWM_GPIO:
			if (state == EVEN_PINCTRL_PIN_STATE_LOW &&!IS_ERR(even_pwm_gpio_low))
				ret = pinctrl_select_state(even_pinctrl, even_pwm_gpio_low);
			else if (state == EVEN_PINCTRL_PIN_STATE_HIGH &&!IS_ERR(even_pwm_gpio_high))
				ret = pinctrl_select_state(even_pinctrl, even_pwm_gpio_high);
			else
				printk("set err, pin(%d) state(%d)\n", pin, state);
			break;
		case EVEN_PINCTRL_PIN_TEMP_GPIO:
			if (state == EVEN_PINCTRL_PIN_STATE_LOW &&!IS_ERR(even_flash_temp_low))
				ret = pinctrl_select_state(even_pinctrl, even_flash_temp_low);
			else if (state == EVEN_PINCTRL_PIN_STATE_HIGH &&!IS_ERR(even_flash_temp_high))
				ret = pinctrl_select_state(even_pinctrl, even_flash_temp_high);
			else
				printk("set err, pin(%d) state(%d)\n", pin, state);
			break;
		default:
			printk("set err, pin(%d) state(%d)\n", pin, state);
			break;
	}
	printk("pin(%d) state(%d), ret:%d\n", pin, state, ret);

	return ret;
}


/* flashlight enable  pwm function */
/* 52M/32/100 = 16KHZ  actually 21KHZ for SGM3785*/
int mt_flashlight_led_set_pwm(int pwm_num,u32 level )
{
	struct pwm_spec_config pwm_setting;
	memset(&pwm_setting, 0, sizeof(struct pwm_spec_config));
	pwm_setting.pwm_no = pwm_num; /* PWM0 set 0,PWM1 set 1,PWM2 set 2,PWM3 set 3 */
	pwm_setting.mode = PWM_MODE_OLD;
	pwm_setting.pmic_pad = 0;
	pwm_setting.clk_div = CLK_DIV32;
	pwm_setting.clk_src = PWM_CLK_OLD_MODE_BLOCK;
	pwm_setting.PWM_MODE_OLD_REGS.IDLE_VALUE = 0;
	pwm_setting.PWM_MODE_OLD_REGS.GUARD_VALUE = 0;
	pwm_setting.PWM_MODE_OLD_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_OLD_REGS.WAVE_NUM = 0;
	pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 100;
	pwm_setting.PWM_MODE_OLD_REGS.THRESH = level;

	pwm_set_spec_config(&pwm_setting);

	return 0;
}



/******************************************************************************
 * even operations
 *****************************************************************************/
static int even_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= EVEN_LEVEL_NUM)
		level = EVEN_LEVEL_NUM - 1;

	return level;
}

/* flashlight enable function */
#ifdef FLASHLIGHT_BRIGHTNESS_ADD
bool fl_state=false;
#endif

static int even_enable(void)
{
	int tempPWM = 0;

#ifdef CONFIG_PM_WAKELOCKS
	__pm_stay_awake(&flashlight_wake_lock);
#endif

#ifdef FLASHLIGHT_BRIGHTNESS_ADD
	if (fl_state){
		g_flash_duty = EVEN_LEVEL_NUM - 1;
		printk("fl_state=true\n");
	}
#endif

	//g_flash_duty = 25;                                                               //for test
	if ((g_flash_duty == EVEN_LED_MODE_DUTY) || (g_flash_duty == EVEN_TORCH_MODE_DUTY)) {//led mode
		even_pinctrl_set(EVEN_PINCTRL_PIN_FLASH_EN,0);
		even_pinctrl_set(EVEN_PINCTRL_PIN_PWM_GPIO, 1);
		mdelay(6);                                          //delay more than 5ms
		even_pinctrl_set(EVEN_PINCTRL_PIN_PWM_EN,1);    //set pwm mode
		mt_flashlight_led_set_pwm(0,80);                    //torch pwm 40%
	}  else if (g_flash_duty < EVEN_LEVEL_TORCH) {//flash mode
		//even_pinctrl_set(EVEN_PINCTRL_PIN_FLASH_EN,0);
		//even_pinctrl_set(EVEN_PINCTRL_PIN_PWM_GPIO,1);//torch max
		even_pinctrl_set(EVEN_PINCTRL_PIN_FLASH_EN,0);
		even_pinctrl_set(EVEN_PINCTRL_PIN_PWM_GPIO, 1);
		mdelay(6);                                          //delay more than 5ms
		even_pinctrl_set(EVEN_PINCTRL_PIN_PWM_EN,1);    //set pwm mode
		mt_flashlight_led_set_pwm(0,40);                    //torch pwm 40%
	} else {//flash mode

		if (g_flash_duty < EVEN_LEVEL_NUM)
			tempPWM = g_duty_array[g_flash_duty];
		else
			tempPWM = g_duty_array[EVEN_LEVEL_NUM - 1];   //flash mode max

		even_pinctrl_set(EVEN_PINCTRL_PIN_FLASH_EN,0);
		even_pinctrl_set(EVEN_PINCTRL_PIN_PWM_EN,1);
		mt_flashlight_led_set_pwm(0,tempPWM);               //flash pwm tempPWM%
		udelay(500);
		even_pinctrl_set(EVEN_PINCTRL_PIN_FLASH_EN,1);
	}

		printk("EVEN Flash %s g_flash_duty = %d,%d\n",__FUNCTION__,g_flash_duty,tempPWM);

	return 0;
}

/* flashlight disable function */
static int even_disable(void)
{
	int state = EVEN_PINCTRL_PIN_STATE_LOW;

	even_pinctrl_set(EVEN_PINCTRL_PIN_FLASH_EN, state);
	even_pinctrl_set(EVEN_PINCTRL_PIN_PWM_GPIO, state);

#ifdef CONFIG_PM_WAKELOCKS
	__pm_relax(&flashlight_wake_lock);
#endif

	return 0;
}

int even_temperature_enable(bool On)
{
    if(On) {
        even_pinctrl_set(EVEN_PINCTRL_PIN_TEMP_GPIO, EVEN_PINCTRL_PIN_STATE_HIGH);
    } else {
        even_pinctrl_set(EVEN_PINCTRL_PIN_TEMP_GPIO, EVEN_PINCTRL_PIN_STATE_LOW);
    }

    return 0;
}


#ifdef FLASHLIGHT_BRIGHTNESS_ADD
int even_enable_flash(int level)
{
	if (level==0) {
		fl_state=false;
		printk("flash mode close\n");
	}
	else {
		fl_state=true;//flash 1A
		printk("flash mode open\n");
	}
	return 0;
}
EXPORT_SYMBOL(even_enable_flash);
#endif

/* set flashlight level */
static int even_set_level(int level)
{
	g_flash_duty = level;
	return 0;
}

/* flashlight init */
static int even_init(void)
{
	int state = EVEN_PINCTRL_PIN_STATE_LOW;

	even_pinctrl_set(EVEN_PINCTRL_PIN_FLASH_EN, state);
	even_pinctrl_set(EVEN_PINCTRL_PIN_PWM_GPIO, state);
	return 0;
}

/* flashlight uninit */
static int even_uninit(void)
{
	int state = EVEN_PINCTRL_PIN_STATE_LOW;

	even_pinctrl_set(EVEN_PINCTRL_PIN_FLASH_EN, state);
	even_pinctrl_set(EVEN_PINCTRL_PIN_PWM_GPIO, state);
	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer even_timer;
static unsigned int even_timeout_ms;

static void even_work_disable(struct work_struct *data)
{
	printk("work queue callback\n");
	even_disable();
}

static enum hrtimer_restart even_timer_func(struct hrtimer *timer)
{
	schedule_work(&even_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int even_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		printk("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		even_timeout_ms = 0;//fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		printk("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		even_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		printk("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (even_timeout_ms) {
				s = even_timeout_ms / 1000;
				ns = even_timeout_ms % 1000 * 1000000;
				ktime = ktime_set(s, ns);
				hrtimer_start(&even_timer, ktime,
						HRTIMER_MODE_REL);
			}
			even_enable();
		} else {
			even_disable();
			hrtimer_cancel(&even_timer);
		}
		break;
	case FLASH_IOC_GET_DUTY_NUMBER:
		printk("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = EVEN_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		printk("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = EVEN_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = even_verify_level(fl_arg->arg);
		printk("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = even_current[fl_arg->arg];
		break;

	default:
		printk("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int even_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int even_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int even_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&even_mutex);
	if (set) {
		if (!use_count)
			ret = even_init();
		use_count++;
		printk("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = even_uninit();
		if (use_count < 0)
			use_count = 0;
		printk("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&even_mutex);

	return ret;
}

static ssize_t even_strobe_store(struct flashlight_arg arg)
{
	even_set_driver(1);
	even_set_level(arg.level);
	even_timeout_ms = 0;
	even_enable();
	msleep(arg.dur);
	even_disable();
	even_set_driver(0);

	return 0;
}

static struct flashlight_operations even_ops = {
	even_open,
	even_release,
	even_ioctl,
	even_strobe_store,
	even_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int even_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" for power saving.
	 * even_init();
	 */

	return 0;
}

static int even_parse_dt(struct device *dev,
		struct even_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		printk("Parse no dt, node.\n");
		return 0;
	}
	printk("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		printk("Parse no dt, decouple.\n");

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
				EVEN_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		printk("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
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

static int even_probe(struct platform_device *pdev)
{
	struct even_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	printk("Probe start.\n");

	/* init pinctrl */
	if (even_pinctrl_init(pdev)) {
		printk("Failed to init pinctrl.\n");
		err = -EFAULT;
		goto err;
	}

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err;
		}
		pdev->dev.platform_data = pdata;
		err = even_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err;
	}

	/* init work queue */
	INIT_WORK(&even_work, even_work_disable);

	/* init timer */
	hrtimer_init(&even_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	even_timer.function = even_timer_func;
	even_timeout_ms = 100;

	/* init chip hw */
	even_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&even_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(EVEN_NAME, &even_ops)) {
			err = -EFAULT;
			goto err;
		}
	}
	even_current = sgm3785_current;

#ifdef CONFIG_PM_WAKELOCKS
	wakeup_source_init(&flashlight_wake_lock, "flashlight_lock_wakelock");
#endif

	printk("Probe done.\n");

	return 0;
err:
	return err;
}

static int even_remove(struct platform_device *pdev)
{
	struct even_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	printk("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(EVEN_NAME);

	/* flush work queue */
	flush_work(&even_work);

	printk("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id even_pwm_of_match[] = {
	{.compatible = EVEN_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, even_pwm_of_match);
#else
static struct platform_device even_pwm_platform_device[] = {
	{
		.name = EVEN_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, even_pwm_platform_device);
#endif

static struct platform_driver even_platform_driver = {
	.probe = even_probe,
	.remove = even_remove,
	.driver = {
		.name = EVEN_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = even_pwm_of_match,
#endif
	},
};

static int __init flashlight_even_init(void)
{
	int ret;

	printk("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&even_pwm_platform_device);
	if (ret) {
		printk("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&even_platform_driver);
	if (ret) {
		printk("Failed to register platform driver\n");
		return ret;
	}

	printk("Init done.\n");

	return 0;
}



static void __exit flashlight_even_exit(void)
{
	printk("Exit start.\n");

	platform_driver_unregister(&even_platform_driver);

	printk("Exit done.\n");
}

int high_cct_led_strobe_enable_part1(void)
{
    printk("!!!\n");
    return 0;
}

int high_cct_led_strobe_setduty_part1(int duty)
{

    even_set_level(EVEN_TORCH_MODE_DUTY);
    return 0;

}

int high_cct_led_strobe_on_part1(int onoff)
{
    if(onoff)
        even_enable();
    else
        even_disable();
    return 0;
}

int low_cct_led_strobe_enable_part1(void)
{
    printk("!!!\n");
    return 0;
}

int low_cct_led_strobe_setduty_part1(int duty)
{

  even_set_level(EVEN_TORCH_MODE_DUTY);
  return 0;

}

int low_cct_led_strobe_on_part1(int onoff)
{
    if(onoff)
        even_enable();
    else
        even_disable();
    return 0;
}



module_init(flashlight_even_init);
module_exit(flashlight_even_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight EVEN PWM Driver");

