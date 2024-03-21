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
#include <linux/leds.h>
#include <mt-plat/mtk_pwm.h>
#include <linux/pm_wakeup.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"


#include <mt-plat/mtk_boot_common.h>


/* define device tree */
/* TODO: modify temp device tree name */
#ifndef DUMMY_GPIO_DTNAME
#define DUMMY_GPIO_DTNAME "mediatek,flashlights_limu"
#endif

/* TODO: define driver name */
#define DUMMY_NAME "flashlights_limu"

/* define registers */
/* TODO: define register */

/* define mutex and work queue */
static DEFINE_MUTEX(dummy_mutex);
static struct work_struct dummy_work;
/* define pinctrl */
/* TODO: define pinctrl */
#define DUMMY_PINCTRL_PIN_STROBE 0
#define DUMMY_PINCTRL_PIN_TORCH 1
#define DUMMY_PINCTRL_PINSTATE_LOW 0
#define DUMMY_PINCTRL_PINSTATE_HIGH 1
#define DUMMY_PINCTRL_PINSTATE_PWM 2

#define DUMMY_PINCTRL_STATE_STROBE_HIGH "strobe_on_gpio"
#define DUMMY_PINCTRL_STATE_STROBE_LOW  "strobe_off_gpio"
#define DUMMY_PINCTRL_STATE_TORCH_HIGH "torch_on_gpio"
#define DUMMY_PINCTRL_STATE_TORCH_LOW  "torch_off_gpio"
#define DUMMY_PINCTRL_STATE_TORCH_PWM  "torch_pwm_gpio"

static struct pinctrl *dummy_pinctrl;
static struct pinctrl_state *dummy_strobe_high;
static struct pinctrl_state *dummy_strobe_low;
static struct pinctrl_state *dummy_torch_high;
static struct pinctrl_state *dummy_torch_low;
static struct pinctrl_state *dummy_torch_pwm;

static int global_level = 0;
static unsigned int dummy_timeout_ms;

static struct wakeup_source flashlight_wake_lock;
static int is_relaxed = 1;

/*Houbing.Peng@ODM Cam.Drv 20200319 set flash mode to driver*/
static int global_flash_mode = -1;
#define FLASH_MODE_TORCH 0
#define FLASH_MODE_VIDEO_TORCH 1
#define FLASH_MODE_PRE_FLASH 3
#define FLASH_MODE_MAIN_FLASH 4

/* define usage count */
static int use_count;

/* platform data */
struct dummy_platform_data {
    int channel_num;
    struct flashlight_device_id *dev_id;
};

#define FLASHLIGHT_VIDEO_TORCH_DUTY 23
#define FLASHLIGHT_DUTY_NUM 28//30

// duty_num: 28,  torch duty: 2
int fl_current_28[FLASHLIGHT_DUTY_NUM] = {102,130,158,190,218,250,278,310,342,374,406,438,470,
                                           502,538,565,600,630,662,700,730,762,800,835,907,947,972,1000};

// freq_pwm = clk_src /clk_div /DATA_WIDTH
// 52M / 16 /30 = 108.33 KHZ
struct pwm_spec_config fl_pwm_config = {
    .pwm_no = 5,  /* PWM0 set 0,PWM1 set 1,PWM2 set 2,PWM5 set 5  you master be set corrected*/
    .mode = PWM_MODE_OLD,
    .clk_div = CLK_DIV16,
    .clk_src = PWM_CLK_OLD_MODE_BLOCK,
    .pmic_pad = 0,
    .PWM_MODE_OLD_REGS.IDLE_VALUE = 0,
    .PWM_MODE_OLD_REGS.GUARD_VALUE = 0,
    .PWM_MODE_OLD_REGS.GDURATION = 0,
    .PWM_MODE_OLD_REGS.WAVE_NUM = 0,
    .PWM_MODE_OLD_REGS.DATA_WIDTH = 30,    /* 30 level */
    .PWM_MODE_OLD_REGS.THRESH = 15, /*default*/
};

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int dummy_pinctrl_init(struct platform_device *pdev)
{
    int ret = 0;

    /* get pinctrl */
    dummy_pinctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(dummy_pinctrl)) {
        pr_err("Failed to get flashlight pinctrl.\n");
        ret = PTR_ERR(dummy_pinctrl);
        return ret;
    }

    /* TODO: Flashlight XXX pin initialization */
    dummy_strobe_high = pinctrl_lookup_state(
            dummy_pinctrl, DUMMY_PINCTRL_STATE_STROBE_HIGH);
    if (IS_ERR(dummy_strobe_high)) {
        pr_err("Failed to init (%s)\n", DUMMY_PINCTRL_STATE_STROBE_HIGH);
        ret = PTR_ERR(dummy_strobe_high);
    }
    dummy_strobe_low = pinctrl_lookup_state(
            dummy_pinctrl, DUMMY_PINCTRL_STATE_STROBE_LOW);
    if (IS_ERR(dummy_strobe_low)) {
        pr_err("Failed to init (%s)\n", DUMMY_PINCTRL_STATE_STROBE_LOW);
        ret = PTR_ERR(dummy_strobe_low);
    }

    dummy_torch_high = pinctrl_lookup_state(
            dummy_pinctrl, DUMMY_PINCTRL_STATE_TORCH_HIGH);
    if (IS_ERR(dummy_torch_high)) {
        pr_err("Failed to init (%s)\n", DUMMY_PINCTRL_STATE_TORCH_HIGH);
        ret = PTR_ERR(dummy_torch_high);
    }
    dummy_torch_low = pinctrl_lookup_state(
            dummy_pinctrl, DUMMY_PINCTRL_STATE_TORCH_LOW);
    if (IS_ERR(dummy_torch_low)) {
        pr_err("Failed to init (%s)\n", DUMMY_PINCTRL_STATE_TORCH_LOW);
        ret = PTR_ERR(dummy_torch_low);
    }

    dummy_torch_pwm = pinctrl_lookup_state(
            dummy_pinctrl, DUMMY_PINCTRL_STATE_TORCH_PWM);
    if (IS_ERR(dummy_torch_pwm)) {
        pr_err("Failed to init (%s)\n", DUMMY_PINCTRL_STATE_TORCH_PWM);
        ret = PTR_ERR(dummy_torch_pwm);
    }

    return ret;
}

static int dummy_pinctrl_set(int pin, int state)
{
    int ret = 0;

    if (IS_ERR(dummy_pinctrl)) {
        pr_err("pinctrl is not available\n");
        return -1;
    }

    switch (pin) {
    case DUMMY_PINCTRL_PIN_STROBE:
        if (state == DUMMY_PINCTRL_PINSTATE_LOW &&
                !IS_ERR(dummy_strobe_low))
            pinctrl_select_state(dummy_pinctrl, dummy_strobe_low);
        else if (state == DUMMY_PINCTRL_PINSTATE_HIGH &&
                !IS_ERR(dummy_strobe_high))
            pinctrl_select_state(dummy_pinctrl, dummy_strobe_high);
        else
            pr_err("set err, pin(%d) state(%d)\n", pin, state);
        break;
    case DUMMY_PINCTRL_PIN_TORCH:
        if (state == DUMMY_PINCTRL_PINSTATE_LOW &&
                !IS_ERR(dummy_torch_low))
            pinctrl_select_state(dummy_pinctrl, dummy_torch_low);
        else if (state == DUMMY_PINCTRL_PINSTATE_HIGH &&
                !IS_ERR(dummy_torch_high))
            pinctrl_select_state(dummy_pinctrl, dummy_torch_high);
        else if(state == DUMMY_PINCTRL_PINSTATE_PWM &&
                !IS_ERR(dummy_torch_pwm))
            pinctrl_select_state(dummy_pinctrl,dummy_torch_pwm);
        break;
    default:
        pr_err("set err, pin(%d) state(%d)\n", pin, state);
        break;
    }
    pr_debug("pin(%d) state(%d)\n", pin, state);

    return ret;
}


/******************************************************************************
 * dummy operations
 *****************************************************************************/
/* flashlight enable function */
static int dummy_enable(void)
{
    int ret = 0;
    printk("limu dummy_enable duty = %d, mode = %d, timeout:%d\n",
        global_level, global_flash_mode, dummy_timeout_ms);
    __pm_stay_awake(&flashlight_wake_lock);
    is_relaxed = 0;
    if (get_boot_mode() == FACTORY_BOOT) {
        pr_info("dummy_enable force use torch mode!\n");
        dummy_pinctrl_set(DUMMY_PINCTRL_PIN_STROBE, DUMMY_PINCTRL_PINSTATE_LOW);
        dummy_pinctrl_set(DUMMY_PINCTRL_PIN_TORCH, DUMMY_PINCTRL_PINSTATE_HIGH);
        return ret;
    }

    if (((dummy_timeout_ms == 0) && (global_flash_mode != FLASH_MODE_MAIN_FLASH))
        || (global_flash_mode == FLASH_MODE_VIDEO_TORCH)) {
        global_flash_mode = FLASH_MODE_TORCH;
    } else if (dummy_timeout_ms > 0) {
        global_flash_mode = FLASH_MODE_MAIN_FLASH;
    }

    switch (global_flash_mode) {
    case FLASH_MODE_TORCH:
        dummy_pinctrl_set(DUMMY_PINCTRL_PIN_STROBE, DUMMY_PINCTRL_PINSTATE_LOW);
        dummy_pinctrl_set(DUMMY_PINCTRL_PIN_TORCH, DUMMY_PINCTRL_PINSTATE_HIGH);
        break;
    case FLASH_MODE_VIDEO_TORCH:
        dummy_pinctrl_set(DUMMY_PINCTRL_PIN_STROBE, DUMMY_PINCTRL_PINSTATE_LOW);
        dummy_pinctrl_set(DUMMY_PINCTRL_PIN_TORCH, DUMMY_PINCTRL_PINSTATE_HIGH);
        mdelay(6);  //make sure to enter movie/torch mode

        fl_pwm_config.PWM_MODE_OLD_REGS.THRESH = FLASHLIGHT_VIDEO_TORCH_DUTY;
        ret = pwm_set_spec_config(&fl_pwm_config);
        if (ret < 0) {
            pr_err("ERROR: dummy_enable pwm config fail!! ret = %d\n", ret);
        }
        dummy_pinctrl_set(DUMMY_PINCTRL_PIN_TORCH, DUMMY_PINCTRL_PINSTATE_PWM);
        break;
    case FLASH_MODE_PRE_FLASH:
    case FLASH_MODE_MAIN_FLASH:
        fl_pwm_config.PWM_MODE_OLD_REGS.THRESH = global_level + 3; //skip low current duty
        ret = pwm_set_spec_config(&fl_pwm_config);
        if (ret < 0) {
            pr_err("ERROR: dummy_enable pwm config fail!! ret = %d\n", ret);
        }

        dummy_pinctrl_set(DUMMY_PINCTRL_PIN_TORCH, DUMMY_PINCTRL_PINSTATE_PWM);
        dummy_pinctrl_set(DUMMY_PINCTRL_PIN_STROBE, DUMMY_PINCTRL_PINSTATE_HIGH);
        break;
    default:
        pr_info("dummy_enable no such flash mode (%d)!\n", global_flash_mode);
        ret = -1;
        break;
    }

    return ret;
}
/* flashlight disable function */
static int dummy_disable(void)
{
    int ret = 0;
    if (is_relaxed == 0){
        __pm_relax(&flashlight_wake_lock);
        is_relaxed = 1;
    }

    if (FLASH_MODE_TORCH != global_flash_mode) {
        mt_pwm_disable(fl_pwm_config.pwm_no, 2);
    }

    dummy_pinctrl_set(DUMMY_PINCTRL_PIN_STROBE, DUMMY_PINCTRL_PINSTATE_LOW);
    dummy_pinctrl_set(DUMMY_PINCTRL_PIN_TORCH, DUMMY_PINCTRL_PINSTATE_LOW);
    if (get_boot_mode() == FACTORY_BOOT){
        mdelay(6000);
    }

    return ret;
}


/*Houbing.Peng@ODM Cam.Drv 20200319 set flash mode to driver*/
static int dummy_set_flash_mode(int mode)
{
    global_flash_mode = mode;
    return 0;
}



/* set flashlight level */
static int dummy_set_level(int level)
{
    global_level = level;
    return 0;
}
/* flashlight init */
static int dummy_init(void)
{
    int pin = 0, state = 0;

    /* TODO: wrap init function */

    return dummy_pinctrl_set(pin, state);
}

/* flashlight uninit */
static int dummy_uninit(void)
{
    int pin = 0, state = 0;

    /* TODO: wrap uninit function */

    return dummy_pinctrl_set(pin, state);
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer dummy_timer;
static unsigned int dummy_timeout_ms;

static void dummy_work_disable(struct work_struct *data)
{
    pr_debug("work queue callback\n");
    dummy_disable();
}

static enum hrtimer_restart dummy_timer_func(struct hrtimer *timer)
{
    schedule_work(&dummy_work);
    return HRTIMER_NORESTART;
}

static int verify_level(int level)
{
    if (level < 0)
        level = 0;
    else if (level >= FLASHLIGHT_DUTY_NUM)
        level = FLASHLIGHT_DUTY_NUM - 1;

    return level;
}

/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int dummy_ioctl(unsigned int cmd, unsigned long arg)
{
    struct flashlight_dev_arg *fl_arg;
    int channel;
    ktime_t ktime;
    unsigned int s;
    unsigned int ns;

    fl_arg = (struct flashlight_dev_arg *)arg;
    channel = fl_arg->channel;

    /* verify channel */
    if (channel < 0 || channel >= 1) {
        pr_err("Failed with error channel\n");
        return -EINVAL;
    }
    switch (cmd) {
    case FLASH_IOC_SET_TIME_OUT_TIME_MS:
        pr_info("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
                channel, (int)fl_arg->arg);
        dummy_timeout_ms = fl_arg->arg;
        break;

    case FLASH_IOC_SET_DUTY:
        pr_info("FLASH_IOC_SET_DUTY(%d): %d\n",
                channel, (int)fl_arg->arg);
        fl_arg->arg = verify_level(fl_arg->arg);
        dummy_set_level(fl_arg->arg);
        break;


    /*Houbing.Peng@ODM Cam.Drv 20200319 set flash mode to driver*/
    case FLASH_IOC_SET_FLASH_MODE:
        pr_info("FLASH_IOC_SET_FLASH_MODE(%d): %d\n",
                channel, (int)fl_arg->arg);
        dummy_set_flash_mode(fl_arg->arg);
        break;

    case FLASH_IOC_SET_ONOFF:
        pr_info("FLASH_IOC_SET_ONOFF(%d): %d\n",
                channel, (int)fl_arg->arg);
        if (fl_arg->arg == 1) {
            if (dummy_timeout_ms) {
                s = dummy_timeout_ms / 1000;
                ns = dummy_timeout_ms % 1000 * 1000000;
                ktime = ktime_set(s, ns);
                hrtimer_start(&dummy_timer, ktime,
                        HRTIMER_MODE_REL);
            }
            dummy_enable();
        } else {
            dummy_disable();
            hrtimer_cancel(&dummy_timer);
        }
        break;
    case FLASH_IOC_GET_DUTY_NUMBER:
        pr_info("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
        fl_arg->arg = FLASHLIGHT_DUTY_NUM;
        break;
    case FLASH_IOC_GET_MAX_TORCH_DUTY:
        pr_info("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
        fl_arg->arg = 2;//3;
        break;
    case FLASH_IOC_GET_DUTY_CURRENT:
        pr_info("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
                channel, (int)fl_arg->arg);
        fl_arg->arg = verify_level(fl_arg->arg);
        fl_arg->arg = fl_current_28[fl_arg->arg];
        break;
    case FLASH_IOC_GET_HW_TIMEOUT:
        pr_info("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
        fl_arg->arg = 600;
        break;
    default:
        pr_info("No such command and arg(%d): (%d, %d)\n",
                channel, _IOC_NR(cmd), (int)fl_arg->arg);
        return -ENOTTY;
    }

    return 0;
}

static int dummy_open(void)
{
    /* Move to set driver for saving power */
    return 0;
}

static int dummy_release(void)
{
    /* Move to set driver for saving power */
    return 0;
}

static int dummy_set_driver(int set)
{
    int ret = 0;

    /* set chip and usage count */
    mutex_lock(&dummy_mutex);
    if (set) {
        if (!use_count)
            ret = dummy_init();
        use_count++;
        pr_debug("Set driver: %d\n", use_count);
    } else {
        use_count--;
        if (!use_count)
            ret = dummy_uninit();
        if (use_count < 0)
            use_count = 0;
        pr_debug("Unset driver: %d\n", use_count);
    }
    mutex_unlock(&dummy_mutex);

    return ret;
}

static ssize_t dummy_strobe_store(struct flashlight_arg arg)
{
    dummy_set_driver(1);
    dummy_set_level(arg.level);
    dummy_timeout_ms = 0;
    dummy_enable();
    msleep(arg.dur);
    dummy_disable();
    dummy_set_driver(0);

    return 0;
}

static struct flashlight_operations dummy_ops = {
    dummy_open,
    dummy_release,
    dummy_ioctl,
    dummy_strobe_store,
    dummy_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int dummy_chip_init(void)
{
    /* NOTE: Chip initialication move to "set driver" for power saving.
     * dummy_init();
     */

    return 0;
}

static int dummy_parse_dt(struct device *dev,
        struct dummy_platform_data *pdata)
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
                DUMMY_NAME);
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

static void mtk_flashlight_brightness_set(struct led_classdev *led_cdev,
        enum led_brightness value)
{
   int temp_level =  global_level;

   if((value < 0) || (value > 29/*30*/)){
        pr_err("Error brightness value %d",value);
        return ;
   }
   pr_info("mtk_flashlight_brightness_set value= %d",value);
   if(0 == value){
       dummy_disable();
   }else{
       global_level = value;
       if(dummy_enable()< 0)
         pr_err("error enable failed global_level=%d",global_level);
   }
   global_level = temp_level;
}

static enum led_brightness mtk_flashlight_brightness_get(struct led_classdev *led_cdev)
{
    return (global_level > 0) ? global_level : 0;
}
//for /sys/class/leds/flashlight
static struct led_classdev flashlight_led = {
       .name           = "flashlight",
       .brightness_set = mtk_flashlight_brightness_set,
       .brightness_get = mtk_flashlight_brightness_get,
       .brightness     = LED_OFF,
};
int32_t mtk_flashlight_create_classdev(struct platform_device *pdev)
{
    int32_t rc = 0;

    rc = led_classdev_register(&pdev->dev, &flashlight_led);
    if (rc) {
        pr_err("Failed to register  led dev. rc = %d\n", rc);
        return rc;
    }
    return 0;
}

static int dummy_probe(struct platform_device *pdev)
{
    struct dummy_platform_data *pdata = dev_get_platdata(&pdev->dev);
    int err;
    int i;
    pr_debug("Probe start.\n");

    /* init pinctrl */
    if (dummy_pinctrl_init(pdev)) {
        pr_debug("Failed to init pinctrl.\n");
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
        err = dummy_parse_dt(&pdev->dev, pdata);
        if (err)
            goto err;
    }

    /* init work queue */
    INIT_WORK(&dummy_work, dummy_work_disable);

    /* init timer */
    hrtimer_init(&dummy_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    dummy_timer.function = dummy_timer_func;
    dummy_timeout_ms = 100;

    /* init chip hw */
    dummy_chip_init();

    /* clear usage count */
    use_count = 0;

    /* register flashlight device */
    if (pdata->channel_num) {
        for (i = 0; i < pdata->channel_num; i++)
            if (flashlight_dev_register_by_device_id(
                        &pdata->dev_id[i],
                        &dummy_ops)) {
                err = -EFAULT;
                goto err;
            }
    } else {
        if (flashlight_dev_register(DUMMY_NAME, &dummy_ops)) {
            err = -EFAULT;
            goto err;
        }
    }
    mtk_flashlight_create_classdev(pdev);
    wakeup_source_init(&flashlight_wake_lock, "flashlight wakelock");
    pr_debug("Probe done.\n");

    return 0;
err:
    return err;
}

static int dummy_remove(struct platform_device *pdev)
{
    struct dummy_platform_data *pdata = dev_get_platdata(&pdev->dev);
    int i;

    pr_debug("Remove start.\n");

    pdev->dev.platform_data = NULL;

    /* unregister flashlight device */
    if (pdata && pdata->channel_num)
        for (i = 0; i < pdata->channel_num; i++)
            flashlight_dev_unregister_by_device_id(
                    &pdata->dev_id[i]);
    else
        flashlight_dev_unregister(DUMMY_NAME);

    /* flush work queue */
    flush_work(&dummy_work);

    pr_debug("Remove done.\n");

    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id dummy_gpio_of_match[] = {
    {.compatible = DUMMY_GPIO_DTNAME},
    {},
};
MODULE_DEVICE_TABLE(of, dummy_gpio_of_match);
#else
static struct platform_device dummy_gpio_platform_device[] = {
    {
        .name = DUMMY_NAME,
        .id = 0,
        .dev = {}
    },
    {}
};
MODULE_DEVICE_TABLE(platform, dummy_gpio_platform_device);
#endif

static struct platform_driver dummy_platform_driver = {
    .probe = dummy_probe,
    .remove = dummy_remove,
    .driver = {
        .name = DUMMY_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = dummy_gpio_of_match,
#endif
    },
};

static int __init flashlight_dummy_init(void)
{
    int ret;

    pr_debug("Init start.\n");

#ifndef CONFIG_OF
    ret = platform_device_register(&dummy_gpio_platform_device);
    if (ret) {
        pr_err("Failed to register platform device\n");
        return ret;
    }
#endif

    ret = platform_driver_register(&dummy_platform_driver);
    if (ret) {
        pr_err("Failed to register platform driver\n");
        return ret;
    }

    pr_debug("Init done.\n");

    return 0;
}

static void __exit flashlight_dummy_exit(void)
{
    pr_debug("Exit start.\n");

    platform_driver_unregister(&dummy_platform_driver);

    pr_debug("Exit done.\n");
}

module_init(flashlight_dummy_init);
module_exit(flashlight_dummy_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight DUMMY GPIO Driver");