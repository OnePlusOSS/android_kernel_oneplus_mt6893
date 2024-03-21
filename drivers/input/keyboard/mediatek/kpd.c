/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2021 MediaTek Inc.
 */

#include "kpd.h"
#ifdef CONFIG_PM_SLEEP
#include <linux/pm_wakeup.h>
#endif
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/pinctrl/consumer.h>
//#ifdef OPLUS_FEATURE_TP_BASIC
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <soc/oplus/system/oplus_project.h>
//#endif /*OPLUS_FEATURE_TP_BASIC*/

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_THEIA)
#include <soc/oplus/system/oplus_bscheck.h>
#include <soc/oplus/system/oplus_brightscreen_check.h>
#endif

#define KPD_NAME	"mtk-kpd"

#ifdef CONFIG_LONG_PRESS_MODE_EN
struct timer_list Long_press_key_timer;
atomic_t vol_down_long_press_flag = ATOMIC_INIT(0);
#endif
//#ifdef OPLUS_FEATURE_TP_BASIC
//#define KPD_HOME_NAME 		"mtk-kpd-home"
#define KPD_VOL_UP_NAME		"mtk-kpd-vol-up"
#define KPD_VOL_DOWN_NAME	"mtk-kpd-vol-down"

#define KEY_LEVEL_DEFAULT				1
//Add for volume_down debounce
#define VOL_DOWN_DELAY_TIME    35*1000*1000


struct vol_info {
	unsigned int vol_up_irq;
	unsigned int vol_down_irq;
	unsigned int vol_up_gpio;
	unsigned int vol_down_gpio;
	int vol_up_val;
	int vol_down_val;
	int vol_up_irq_enabled;
	int vol_down_irq_enabled;
	int vol_up_irq_type;
	int vol_down_irq_type;
	struct device *dev;
	struct platform_device *pdev;
	bool homekey_as_vol_up;
}vol_key_info;

//Add for volume_down debounce
static struct hrtimer vol_down_timer;
static int vol_down_last_val = 0xff;
static void kpd_set_volumedown_irq_type(void);
static enum hrtimer_restart vol_down_timer_func(struct hrtimer *timer);


static irqreturn_t kpd_volumeup_irq_handler(int irq, void *dev_id);
static void kpd_volumeup_task_process(unsigned long data);
static DECLARE_TASKLET(kpd_volumekey_up_tasklet, kpd_volumeup_task_process, 0);
static irqreturn_t kpd_volumedown_irq_handler(int irq, void *dev_id);
static void kpd_volumedown_task_process(unsigned long data);
static DECLARE_TASKLET(kpd_volumekey_down_tasklet, kpd_volumedown_task_process, 0);

static int aee_kpd_enable = 0;
static void kpd_aee_handler(u32 keycode, u16 pressed);
static inline void kpd_update_aee_state(void);

#define VOLKEYPASSWORD 17331	//0x43b3
int door_open = 0;
static unsigned int vol_key_password = 0;
static unsigned long start_timer_last = 0;
static void kpd_set_vol_key_state(int key, int key_val)
{
	unsigned long start_timer_current = jiffies;

	if (key == KEY_VOLUMEUP && key_val)
		vol_key_password = (vol_key_password << 1)|0x01;

	if(key == KEY_VOLUMEDOWN && key_val)
		vol_key_password = (vol_key_password << 1)&~0x01;

	if (key_val) {
		if (door_open) {
			door_open = 0;
			vol_key_password = 0;
			pr_err("vol_Key_password door_close \n");
		}
		start_timer_current = jiffies;
		if(start_timer_last != 0){
			if (time_after(start_timer_current,start_timer_last + msecs_to_jiffies(1000))){
				vol_key_password = 0;
			}

			if((VOLKEYPASSWORD == vol_key_password) && (door_open == 0))
			{
				pr_err("vol_key_password door_open \n");
				door_open = 1;
			}
		}
		start_timer_last = start_timer_current;
	}

}

static void kpd_volumeup_task_process(unsigned long data)
{
	pr_err("%s vol_up_val: %d\n", __func__, vol_key_info.vol_up_val);
	input_report_key(kpd_input_dev, KEY_VOLUMEUP, !vol_key_info.vol_up_val);
	input_sync(kpd_input_dev);
	kpd_set_vol_key_state(KEY_VOLUMEUP, vol_key_info.vol_up_val);
	enable_irq(vol_key_info.vol_up_irq);

	if (aee_kpd_enable) {
		kpd_aee_handler(KEY_VOLUMEUP, !vol_key_info.vol_up_val);
	}
}

static irqreturn_t kpd_volumeup_irq_handler(int irq, void *dev_id)
{
#if 0
	if (vol_key_info.vol_up_irq_type == IRQ_TYPE_EDGE_FALLING) {
		mdelay(5);
		vol_key_info.vol_up_val = gpio_get_value(vol_key_info.vol_up_gpio);
		if(vol_key_info.vol_up_val) {
			pr_err("%s irq_type falling, vol_up_val: 1, return\n", __func__);
			return IRQ_HANDLED;
		}
	} else if(vol_key_info.vol_up_irq_type == IRQ_TYPE_EDGE_RISING) {
		mdelay(5);
		vol_key_info.vol_up_val = gpio_get_value(vol_key_info.vol_up_gpio);
		if(!vol_key_info.vol_up_val) {
			pr_err("%s irq_type rising, vol_up_val: 0, return\n", __func__);
			return IRQ_HANDLED;
		}
	} else {
		return IRQ_HANDLED;
	}
#endif
    kpd_notice("kpd_volumeup_irq_handler called\n");
	disable_irq_nosync(vol_key_info.vol_up_irq);

#if 1
	vol_key_info.vol_up_val = gpio_get_value(vol_key_info.vol_up_gpio);
#endif
	if (vol_key_info.vol_up_val) {
		irq_set_irq_type(vol_key_info.vol_up_irq, IRQ_TYPE_EDGE_FALLING);
		vol_key_info.vol_up_irq_type = IRQ_TYPE_EDGE_FALLING;
	} else {
		irq_set_irq_type(vol_key_info.vol_up_irq, IRQ_TYPE_EDGE_RISING);
		vol_key_info.vol_up_irq_type = IRQ_TYPE_EDGE_RISING;
	}
	//pr_err("%s irq_type:%d, val:%d\n", __func__,
		//vol_key_info.vol_up_irq_type, vol_key_info.vol_up_val);
	tasklet_schedule(&kpd_volumekey_up_tasklet);
	return IRQ_HANDLED;
}

static void kpd_volumedown_task_process(unsigned long data)
{
	pr_err("%s vol_down val:%d, last val:%d\n", __func__, vol_key_info.vol_down_val, vol_down_last_val);
	if (vol_down_last_val != vol_key_info.vol_down_val) {
		input_report_key(kpd_input_dev, KEY_VOLUMEDOWN, !vol_key_info.vol_down_val);
		input_sync(kpd_input_dev);
		vol_down_last_val = vol_key_info.vol_down_val;
		kpd_set_vol_key_state(KEY_VOLUMEDOWN, vol_key_info.vol_down_val);
	}

	enable_irq(vol_key_info.vol_down_irq);

	if (aee_kpd_enable) {
		kpd_aee_handler(KEY_VOLUMEDOWN, !vol_key_info.vol_down_val);
	}
}

static void kpd_set_volumedown_irq_type(void)
{
	vol_key_info.vol_down_val = gpio_get_value(vol_key_info.vol_down_gpio);

	if (vol_key_info.vol_down_val) {
		irq_set_irq_type(vol_key_info.vol_down_irq, IRQ_TYPE_EDGE_FALLING);
		vol_key_info.vol_down_irq_type = IRQ_TYPE_EDGE_FALLING;
	} else {
		irq_set_irq_type(vol_key_info.vol_down_irq, IRQ_TYPE_EDGE_RISING);
		vol_key_info.vol_down_irq_type = IRQ_TYPE_EDGE_RISING;
	}

	pr_err("%s irq_type:%d, val:%d\n", __func__,vol_key_info.vol_down_irq_type, vol_key_info.vol_down_val);

}

static enum hrtimer_restart vol_down_timer_func(struct hrtimer *timer)
{
    kpd_set_volumedown_irq_type();
    tasklet_schedule(&kpd_volumekey_down_tasklet);
    return HRTIMER_NORESTART;
}


static irqreturn_t kpd_volumedown_irq_handler(int irq, void *dev_id)
{
#if 0
	if (vol_key_info.vol_down_irq_type == IRQ_TYPE_EDGE_FALLING) {
		mdelay(5);
		vol_key_info.vol_down_val = gpio_get_value(vol_key_info.vol_down_gpio);
		if(vol_key_info.vol_down_val) {
			pr_err("%s irq_type falling, vol_down_val: 1, return\n", __func__);
			return IRQ_HANDLED;
		}
	} else if(vol_key_info.vol_down_irq_type == IRQ_TYPE_EDGE_RISING) {
		mdelay(5);
		vol_key_info.vol_down_val = gpio_get_value(vol_key_info.vol_down_gpio);
		if(!vol_key_info.vol_down_val) {
			pr_err("%s irq_type rising, vol_down_val: 0, return\n", __func__);
			return IRQ_HANDLED;
		}
	} else {
		return IRQ_HANDLED;
	}
#endif
    kpd_notice("kpd_volumedown_irq_handler called\n");
	disable_irq_nosync(vol_key_info.vol_down_irq);

	//tasklet_schedule(&kpd_volumekey_down_tasklet);
	hrtimer_start(&vol_down_timer, ktime_set(0, VOL_DOWN_DELAY_TIME), HRTIMER_MODE_REL);

	return IRQ_HANDLED;
}
int kpd_klog_en;
void __iomem *kp_base;
static unsigned int kp_irqnr;
struct input_dev *kpd_input_dev;
static struct dentry *kpd_droot;
static struct dentry *kpd_dklog;
unsigned long call_status;
static bool kpd_suspend;
static unsigned int kp_irqnr;
static u32 kpd_keymap[KPD_NUM_KEYS];
static u16 kpd_keymap_state[KPD_NUM_MEMS];

struct input_dev *kpd_input_dev;
#ifdef CONFIG_PM_SLEEP
struct wakeup_source* kpd_suspend_lock;
#endif
struct keypad_dts_data kpd_dts_data;

/* for keymap handling */
static void kpd_keymap_handler(unsigned long data);
static DECLARE_TASKLET(kpd_keymap_tasklet, kpd_keymap_handler, 0);

static void kpd_memory_setting(void);
static int kpd_pdrv_probe(struct platform_device *pdev);
static int kpd_pdrv_suspend(struct platform_device *pdev, pm_message_t state);
static int kpd_pdrv_resume(struct platform_device *pdev);
static struct platform_driver kpd_pdrv;

static void kpd_memory_setting(void)
{
	kpd_init_keymap(kpd_keymap);
	kpd_init_keymap_state(kpd_keymap_state);
}

#if 1
static ssize_t kpd_call_state_store(struct device_driver *ddri,
		const char *buf, size_t count)
{
	int ret;

	ret = kstrtoul(buf, 10, &call_status);
	if (ret) {
		kpd_print("kpd call state: Invalid values\n");
		return -EINVAL;
	}

	switch (call_status) {
	case 1:
		kpd_print("kpd call state: Idle state!\n");
		break;
	case 2:
		kpd_print("kpd call state: ringing state!\n");
		break;
	case 3:
		kpd_print("kpd call state: active or hold state!\n");
		break;

	default:
		kpd_print("kpd call state: Invalid values\n");
		break;
	}
	return count;
}

static ssize_t kpd_call_state_show(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%ld\n", call_status);
	return res;
}

static DRIVER_ATTR_RW(kpd_call_state);
static struct driver_attribute *kpd_attr_list[] = {
	&driver_attr_kpd_call_state,
};


static int kpd_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = ARRAY_SIZE(kpd_attr_list);

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, kpd_attr_list[idx]);
		if (err) {
			kpd_info("driver_create_file (%s) = %d\n",
				kpd_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int kpd_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = ARRAY_SIZE(kpd_attr_list);

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, kpd_attr_list[idx]);

	return err;
}
#endif
/*----------------------------------------------------------------------------*/
/* for AEE manual dump */
#define AEE_VOLUMEUP_BIT	0
#define AEE_VOLUMEDOWN_BIT	1
#define AEE_DELAY_TIME		15
/* enable volup + voldown was pressed 5~15 s Trigger aee manual dump */
#define AEE_ENABLE_5_15		1
static struct hrtimer aee_timer;
static unsigned long aee_pressed_keys;
static bool aee_timer_started;

#if AEE_ENABLE_5_15
#define AEE_DELAY_TIME_5S	5
static struct hrtimer aee_timer_5s;
static bool aee_timer_5s_started;
static bool flags_5s;
#endif
static inline void kpd_update_aee_state(void)
{
	if (aee_pressed_keys == ((1 << AEE_VOLUMEUP_BIT) | (1 << AEE_VOLUMEDOWN_BIT))) {
		/* if volumeup and volumedown was pressed the same time then start the time of ten seconds */
		aee_timer_started = true;

#if AEE_ENABLE_5_15
		aee_timer_5s_started = true;
		hrtimer_start(&aee_timer_5s, ktime_set(AEE_DELAY_TIME_5S, 0), HRTIMER_MODE_REL);
#endif
		hrtimer_start(&aee_timer, ktime_set(AEE_DELAY_TIME, 0), HRTIMER_MODE_REL);
		kpd_print("aee_timer started\n");
	} else {
		/*
		  * hrtimer_cancel - cancel a timer and wait for the handler to finish.
		  * Returns:
		  * 0 when the timer was not active.
		  * 1 when the timer was active.
		 */
		if (aee_timer_started) {
			if (hrtimer_cancel(&aee_timer)) {
				kpd_print("try to cancel hrtimer\n");
#if AEE_ENABLE_5_15
				if (flags_5s) {
					kpd_print("Pressed Volup + Voldown5s~15s then trigger aee manual dump.\n");
					/*ZH CHEN*/
					/*aee_kernel_reminding("manual dump", "Trigger Vol Up +Vol Down 5s");*/
				}
#endif

			}
#if AEE_ENABLE_5_15
			flags_5s = false;
#endif
			aee_timer_started = false;
			kpd_print("aee_timer canceled\n");
		}
#if AEE_ENABLE_5_15
		/*
		  * hrtimer_cancel - cancel a timer and wait for the handler to finish.
		  * Returns:
		  * 0 when the timer was not active.
		  * 1 when the timer was active.
		 */
		if (aee_timer_5s_started) {
			if (hrtimer_cancel(&aee_timer_5s))
				kpd_print("try to cancel hrtimer (5s)\n");
			aee_timer_5s_started = false;
			kpd_print("aee_timer canceled (5s)\n");
		}
#endif
	}
}
static void kpd_aee_handler(u32 keycode, u16 pressed)
{
	if (pressed) {
		if (keycode == KEY_VOLUMEUP)
			__set_bit(AEE_VOLUMEUP_BIT, &aee_pressed_keys);
		else if (keycode == KEY_VOLUMEDOWN)
			__set_bit(AEE_VOLUMEDOWN_BIT, &aee_pressed_keys);
		else
			return;
		kpd_update_aee_state();
	} else {
		if (keycode == KEY_VOLUMEUP)
			__clear_bit(AEE_VOLUMEUP_BIT, &aee_pressed_keys);
		else if (keycode == KEY_VOLUMEDOWN)
			__clear_bit(AEE_VOLUMEDOWN_BIT, &aee_pressed_keys);
		else
			return;
		kpd_update_aee_state();
	}
}
static enum hrtimer_restart aee_timer_func(struct hrtimer *timer)
{
        /* kpd_info("kpd: vol up+vol down AEE manual dump!\n"); */
        /* aee_kernel_reminding("manual dump ", "Triggered by press KEY_VOLUMEUP+KEY_VOLUMEDO
WN"); */
        /*ZH CHEN*/
        /*aee_trigger_kdb();*/
        if (aee_kpd_enable) {
                pr_err("%s call bug for aee manual dump.", __func__);
                BUG();
        }

        return HRTIMER_NORESTART;
}

#if AEE_ENABLE_5_15
static enum hrtimer_restart aee_timer_5s_func(struct hrtimer *timer)
{

        /* kpd_info("kpd: vol up+vol down AEE manual dump timer 5s !\n"); */
        flags_5s = true;
        return HRTIMER_NORESTART;
}
#endif
/****************************************/
#ifdef CONFIG_LONG_PRESS_MODE_EN
void vol_down_long_press(unsigned long pressed)
{
	atomic_set(&vol_down_long_press_flag, 1);
}
#endif
/*****************************************/

#ifdef CONFIG_KPD_PWRKEY_USE_PMIC
void kpd_pwrkey_pmic_handler(unsigned long pressed)
{
	kpd_print("Power Key generate, pressed=%ld\n", pressed);
	if (!kpd_input_dev) {
		kpd_print("KPD input device not ready\n");
		return;
	}
	kpd_pmic_pwrkey_hal(pressed);

	#if IS_ENABLED(CONFIG_OPLUS_FEATURE_THEIA)
	if(pressed){
		//we should canel per work
		black_screen_timer_restart();
		bright_screen_timer_restart();
	}
	#endif
}
#endif

void kpd_pmic_rstkey_handler(unsigned long pressed)
{
	kpd_print("PMIC reset Key generate, pressed=%ld\n", pressed);
	if (!kpd_input_dev) {
		kpd_print("KPD input device not ready\n");
		return;
	}
	kpd_pmic_rstkey_hal(pressed);
#ifdef KPD_PMIC_RSTKEY_MAP
	kpd_aee_handler(KPD_PMIC_RSTKEY_MAP, pressed);
#endif

//#ifdef OPLUS_FEATURE_TP_BASIC
if(vol_key_info.homekey_as_vol_up) {
	kpd_set_vol_key_state(kpd_dts_data.kpd_sw_rstkey, !pressed);

	if (aee_kpd_enable) {
		kpd_aee_handler(kpd_dts_data.kpd_sw_rstkey, pressed);
	}
}
//#endif /* OPLUS_FEATURE_TP_BASIC */
}

static void kpd_keymap_handler(unsigned long data)
{
	u16 i, j;
	int32_t pressed;
	u16 new_state[KPD_NUM_MEMS], change, mask;
	u16 hw_keycode, linux_keycode;

	kpd_get_keymap_state(new_state);
#ifdef CONFIG_PM_SLEEP
	__pm_wakeup_event(kpd_suspend_lock, 500);
#endif
	for (i = 0; i < KPD_NUM_MEMS; i++) {
		change = new_state[i] ^ kpd_keymap_state[i];
		if (change == 0U)
			continue;

		for (j = 0; j < 16U; j++) {
			mask = (u16) 1 << j;
			if ((change & mask) == 0U)
				continue;

			hw_keycode = (i << 4) + j;

			if (hw_keycode >= KPD_NUM_KEYS)
				continue;

			/* bit is 1: not pressed, 0: pressed */
			pressed = ((new_state[i] & mask) == 0U) ? 1 : 0;
			kpd_print("(%s) HW keycode = %d\n",
				(pressed == 1) ? "pressed" : "released",
					hw_keycode);

			linux_keycode = kpd_keymap[hw_keycode];
			if (linux_keycode == 0U)
				continue;
			input_report_key(kpd_input_dev, linux_keycode, pressed);
			input_sync(kpd_input_dev);
			kpd_print("report Linux keycode = %d\n", linux_keycode);

#ifdef CONFIG_LONG_PRESS_MODE_EN
			if (pressed) {
				init_timer(&Long_press_key_timer);
				Long_press_key_timer.expires = jiffies + 5*HZ;
				Long_press_key_timer.data =
					(unsigned long)pressed;
				Long_press_key_timer.function =
					vol_down_long_press;
				add_timer(&Long_press_key_timer);
			} else {
				del_timer_sync(&Long_press_key_timer);
			}
			if (!pressed &&
				atomic_read(&vol_down_long_press_flag)) {
				atomic_set(&vol_down_long_press_flag, 0);
			}
#endif
		}
	}

	memcpy(kpd_keymap_state, new_state, sizeof(new_state));
	enable_irq(kp_irqnr);
}

static irqreturn_t kpd_irq_handler(int irq, void *dev_id)
{
	/* use _nosync to avoid deadlock */
	disable_irq_nosync(kp_irqnr);
	tasklet_schedule(&kpd_keymap_tasklet);
	return IRQ_HANDLED;
}

static int kpd_open(struct input_dev *dev)
{
	/* void __user *uarg = (void __user *)arg; */
	return 0;
}

void kpd_get_dts_info(struct device_node *node)
{
	int32_t ret;

	of_property_read_u32(node, "mediatek,kpd-key-debounce",
		&kpd_dts_data.kpd_key_debounce);
	of_property_read_u32(node, "mediatek,kpd-sw-pwrkey",
		&kpd_dts_data.kpd_sw_pwrkey);
	of_property_read_u32(node, "mediatek,kpd-hw-pwrkey",
		&kpd_dts_data.kpd_hw_pwrkey);
	of_property_read_u32(node, "mediatek,kpd-sw-rstkey",
		&kpd_dts_data.kpd_sw_rstkey);
	of_property_read_u32(node, "mediatek,kpd-hw-rstkey",
		&kpd_dts_data.kpd_hw_rstkey);
	of_property_read_u32(node, "mediatek,kpd-use-extend-type",
		&kpd_dts_data.kpd_use_extend_type);
	of_property_read_u32(node, "mediatek,kpd-hw-dl-key1",
		&kpd_dts_data.kpd_hw_dl_key1);
	of_property_read_u32(node, "mediatek,kpd-hw-dl-key2",
		&kpd_dts_data.kpd_hw_dl_key2);
	of_property_read_u32(node, "mediatek,kpd-hw-dl-key3",
		&kpd_dts_data.kpd_hw_dl_key3);
	of_property_read_u32(node, "mediatek,kpd-hw-recovery-key",
		&kpd_dts_data.kpd_hw_recovery_key);
	of_property_read_u32(node, "mediatek,kpd-hw-factory-key",
		&kpd_dts_data.kpd_hw_factory_key);
	of_property_read_u32(node, "mediatek,kpd-hw-map-num",
		&kpd_dts_data.kpd_hw_map_num);
	of_property_read_u32(node, "mediatek,boot_mode",
		&kpd_dts_data.boot_mode);
	ret = of_property_read_u32_array(node, "mediatek,kpd-hw-init-map",
		kpd_dts_data.kpd_hw_init_map,
			kpd_dts_data.kpd_hw_map_num);

	if (ret) {
		kpd_print("kpd-hw-init-map was not defined in dts.\n");
		memset(kpd_dts_data.kpd_hw_init_map, 0,
			sizeof(kpd_dts_data.kpd_hw_init_map));
	}

	kpd_print("deb= %d, sw-pwr= %d, hw-pwr= %d, hw-rst= %d, sw-rst= %d\n",
		  kpd_dts_data.kpd_key_debounce, kpd_dts_data.kpd_sw_pwrkey,
			kpd_dts_data.kpd_hw_pwrkey, kpd_dts_data.kpd_hw_rstkey,
				kpd_dts_data.kpd_sw_rstkey);
}

//#ifdef OPLUS_FEATURE_TP_BASIC
static int kpd_request_named_gpio(struct vol_info *kpd,
		const char *label, int *gpio)
{
	struct device *dev = kpd->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);
	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return rc;
	}

	*gpio = rc;
	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}

	//dev_info(dev, "%s - gpio: %d\n", label, *gpio);
	return 0;
}
static int init_custom_gpio_state(struct platform_device *client) {
	struct pinctrl *pinctrl1;
	struct pinctrl_state *volume_up_as_int, *volume_down_as_int;
	struct device_node *node = NULL;
	u32 intr[4] = {0};
	int ret;
	u32 debounce_time = 0;

	pinctrl1 = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pinctrl1)) {
		ret = PTR_ERR(pinctrl1);
		kpd_print("can not find keypad pintrl1");
		return ret;
	}

	/*for key volume up*/
	if (!vol_key_info.homekey_as_vol_up) {
		volume_up_as_int = pinctrl_lookup_state(pinctrl1, "volume_up_as_int");
		if (IS_ERR(volume_up_as_int)) {
			ret = PTR_ERR(volume_up_as_int);
			kpd_print("can not find gpio of volume up\n");
			return ret;
		} else {
			ret = pinctrl_select_state(pinctrl1, volume_up_as_int);
			if (ret < 0){
				kpd_print("error to set gpio state\n");
				return ret;
			}

			node = of_find_compatible_node(NULL, NULL, "mediatek, VOLUME_UP-eint");
			if (node) {
				of_property_read_u32_array(node , "interrupts", intr, ARRAY_SIZE(intr));
				pr_info("volume up intr[0-3]  = %d %d %d %d\r\n", intr[0] ,intr[1], intr[2] ,intr[3]);
				//vol_key_info.vol_up_gpio = intr[0];
				vol_key_info.vol_up_irq = irq_of_parse_and_map(node, 0);
				ret = of_property_read_u32(node, "debounce", &debounce_time);
				if (ret) {
					pr_err("%s get debounce_time fail\n", __func__);
				}
				pr_err("%s debounce_time:%d\n", __func__, debounce_time);
			} else {
				pr_err("%d volume up irp node not exist\n", __LINE__);
				return -1;
			}
			vol_key_info.vol_up_irq_type = IRQ_TYPE_EDGE_FALLING;
			ret = request_irq(vol_key_info.vol_up_irq, (irq_handler_t)kpd_volumeup_irq_handler, IRQF_TRIGGER_FALLING, KPD_VOL_UP_NAME, NULL);
			if(ret){
				pr_err("%d request irq failed\n", __LINE__);
				return -1;
			}
			if (vol_key_info.vol_up_gpio > 0 && debounce_time)
				gpio_set_debounce(vol_key_info.vol_up_gpio, debounce_time);
		}
	}

	/*for key of volume down*/
	volume_down_as_int = pinctrl_lookup_state(pinctrl1, "volume_down_as_int");
	if (IS_ERR(volume_down_as_int)) {
		ret = PTR_ERR(volume_down_as_int);
		kpd_print("can not find gpio of  volume down\n");
		return ret;
	} else {
		ret = pinctrl_select_state(pinctrl1, volume_down_as_int);
		if (ret < 0){
			kpd_print("error to set gpio state\n");
			return ret;
		}

		node = of_find_compatible_node(NULL, NULL, "mediatek, VOLUME_DOWN-eint");
		if (node) {
			of_property_read_u32_array(node , "interrupts", intr, ARRAY_SIZE(intr));
			pr_info("volume down intr[0-3] = %d %d %d %d\r\n", intr[0] ,intr[1], intr[2], intr[3]);
			//vol_key_info.vol_down_gpio = intr[0];
			vol_key_info.vol_down_irq = irq_of_parse_and_map(node, 0);
		} else {
			pr_err("%d volume down irp node not exist\n", __LINE__);
			return -1;
		}

		hrtimer_init(&vol_down_timer,CLOCK_MONOTONIC,HRTIMER_MODE_REL);
		vol_down_timer.function = vol_down_timer_func;

		ret = of_property_read_u32(node, "debounce", &debounce_time);
		if (ret) {
			pr_err("%s vol_down get debounce_time fail\n", __func__);
		}
		pr_err("%s vol_down debounce_time:%d\n", __func__, debounce_time);
		vol_key_info.vol_down_irq_type = IRQ_TYPE_EDGE_FALLING;
		ret = request_irq(vol_key_info.vol_down_irq, (irq_handler_t)kpd_volumedown_irq_handler, IRQF_TRIGGER_FALLING, KPD_VOL_DOWN_NAME, NULL);
		if(ret){
			pr_err("%d request irq failed\n", __LINE__);
			return -1;
		}
		if (vol_key_info.vol_down_gpio > 0 && debounce_time)
			gpio_set_debounce(vol_key_info.vol_down_gpio, debounce_time);
	}

	kpd_print(" init_custom_gpio_state End\n");
    return 0;

}
//#endif /*OPLUS_FEATURE_TP_BASIC*/
//#ifdef OPLUS_FEATURE_TP_BASIC
static ssize_t aee_kpd_enable_read(struct file *filp, char __user *buff,
				size_t count, loff_t *off)
{
	char page[256] = {0};
	char read_data[16] = {0};
	int len = 0;

	if (aee_kpd_enable)
		read_data[0] = '1';
	else
		read_data[0] = '0';

	len = sprintf(page, "%s", read_data);

	if(len > *off)
		len -= *off;
	else
		len = 0;
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

static ssize_t aee_kpd_enable_write(struct file *filp, const char __user *buff,
				size_t len, loff_t *data)
{
	char temp[16] = {0};

	if (copy_from_user(temp, buff, len)) {
		pr_err("aee_kpd_enable_write error.\n");
		return -EFAULT;
	}
	sscanf(temp, "%d", &aee_kpd_enable);
	pr_err("%s enable:%d\n", __func__, aee_kpd_enable);

	return len;
}

static const struct file_operations aee_kpd_enable_proc_fops = {
	.write = aee_kpd_enable_write,
	.read = aee_kpd_enable_read,
};
static void init_proc_aee_kpd_enable(void)
{
	struct proc_dir_entry *p = NULL;

	p = proc_create("aee_kpd_enable", 0664,
					NULL, &aee_kpd_enable_proc_fops);
	if (!p)
		pr_err("proc_create aee_kpd_enable ops fail!\n");

}
//#endif /* OPLUS_FEATURE_TP_BASIC */
static int32_t kpd_gpio_init(struct device *dev)
{
	struct pinctrl *keypad_pinctrl;
	struct pinctrl_state *kpd_default;
	int32_t ret;

	if (dev == NULL) {
		kpd_print("kpd device is NULL!\n");
		ret = -1;
	} else {
		keypad_pinctrl = devm_pinctrl_get(dev);
		if (IS_ERR(keypad_pinctrl)) {
			ret = -1;
			kpd_print("Cannot find keypad_pinctrl!\n");
		} else {
			kpd_default = pinctrl_lookup_state(keypad_pinctrl,
				"default");
			if (IS_ERR(kpd_default)) {
				ret = -1;
				kpd_print("Cannot find ecall_state!\n");
			} else
				ret = pinctrl_select_state(keypad_pinctrl,
					kpd_default);
		}
	}
	return ret;
}

static int mt_kpd_debugfs(void)
{
#ifdef CONFIG_MTK_ENG_BUILD
	kpd_klog_en = 1;
#else
	kpd_klog_en = 0;
#endif
	kpd_droot = debugfs_create_dir("keypad", NULL);
	if (IS_ERR_OR_NULL(kpd_droot))
		return PTR_ERR(kpd_droot);

	kpd_dklog = debugfs_create_u32("debug", 0600, kpd_droot, &kpd_klog_en);

	return 0;
}

static int kpd_pdrv_probe(struct platform_device *pdev)
{
	struct clk *kpd_clk = NULL;
	u32 i;
	int32_t err = 0;
//#ifdef OPLUS_FEATURE_TP_BASIC
	struct device *dev = &pdev->dev;
	struct vol_info *kpd_oplus = NULL;

	kpd_oplus = devm_kzalloc(dev, sizeof(*kpd_oplus), GFP_KERNEL);
	if (!kpd_oplus) {
		kpd_notice("kpd: alloc vol_info fail\n");
		return -ENOMEM;
	}
//#endif /*OPLUS_FEATURE_TP_BASIC*/
	printk("kpd_pdrv_probe\n");
	if (!pdev->dev.of_node) {
		kpd_notice("no kpd dev node\n");
		return -ENODEV;
	}

	kpd_clk = devm_clk_get(&pdev->dev, "kpd-clk");
	if (!IS_ERR(kpd_clk)) {
		err = clk_prepare_enable(kpd_clk);
		if (err)
			kpd_notice("get kpd-clk fail: %d\n", err);
	} else {
		kpd_notice("kpd-clk is default set by ccf.\n");
	}

	kp_base = of_iomap(pdev->dev.of_node, 0);
	if (!kp_base) {
		kpd_notice("KP iomap failed\n");
		return -ENODEV;
	};

	kp_irqnr = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!kp_irqnr) {
		kpd_notice("KP get irqnr failed\n");
		return -ENODEV;
	}
	kpd_info("kp base: 0x%p, addr:0x%p,  kp irq: %d\n",
			kp_base, &kp_base, kp_irqnr);
	err = kpd_gpio_init(&pdev->dev);
	if (err != 0)
		kpd_print("gpio init failed\n");

	kpd_get_dts_info(pdev->dev.of_node);

	kpd_memory_setting();

	kpd_input_dev = devm_input_allocate_device(&pdev->dev);
	if (!kpd_input_dev) {
		kpd_notice("input allocate device fail.\n");
		return -ENOMEM;
	}

	kpd_input_dev->name = KPD_NAME;
	kpd_input_dev->id.bustype = BUS_HOST;
	kpd_input_dev->id.vendor = 0x2454;
	kpd_input_dev->id.product = 0x6500;
	kpd_input_dev->id.version = 0x0010;
	kpd_input_dev->open = kpd_open;
	kpd_input_dev->dev.parent = &pdev->dev;

	__set_bit(EV_KEY, kpd_input_dev->evbit);
#if defined(CONFIG_KPD_PWRKEY_USE_PMIC)
	__set_bit(kpd_dts_data.kpd_sw_pwrkey, kpd_input_dev->keybit);
	kpd_keymap[8] = 0;
#endif
	if (!kpd_dts_data.kpd_use_extend_type) {
		for (i = 17; i < KPD_NUM_KEYS; i += 9)
			kpd_keymap[i] = 0;
	}
	for (i = 0; i < KPD_NUM_KEYS; i++) {
		if (kpd_keymap[i] != 0)
			__set_bit(kpd_keymap[i], kpd_input_dev->keybit);
	}

	if (kpd_dts_data.kpd_sw_rstkey)
		__set_bit(kpd_dts_data.kpd_sw_rstkey, kpd_input_dev->keybit);
#ifdef KPD_KEY_MAP
	__set_bit(KPD_KEY_MAP, kpd_input_dev->keybit);
#endif
#ifdef CONFIG_MTK_MRDUMP_KEY
	__set_bit(KEY_RESTART, kpd_input_dev->keybit);
#endif

	err = input_register_device(kpd_input_dev);
	if (err) {
		kpd_notice("register input device failed (%d)\n", err);
		return err;
	}
#ifdef CONFIG_PM_SLEEP
	kpd_suspend_lock = wakeup_source_register(NULL, "kpd wakelock");
#endif
	/* register IRQ and EINT */
	kpd_set_debounce(kpd_dts_data.kpd_key_debounce);
	err = request_irq(kp_irqnr, kpd_irq_handler, IRQF_TRIGGER_NONE,
			KPD_NAME, NULL);
	if (err) {
		kpd_notice("register IRQ failed (%d)\n", err);
		input_unregister_device(kpd_input_dev);
		return err;
	}

	if (enable_irq_wake(kp_irqnr) < 0)
		kpd_notice("irq %d enable irq wake fail\n", kp_irqnr);

#ifdef CONFIG_MTK_MRDUMP_KEY
	mt_eint_register();
#endif
#ifdef CONFIG_MTK_PMIC_NEW_ARCH
	long_press_reboot_function_setting();
#endif
	hrtimer_init(&aee_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aee_timer.function = aee_timer_func;
#if AEE_ENABLE_5_15
	hrtimer_init(&aee_timer_5s, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aee_timer_5s.function = aee_timer_5s_func;
#endif
	err = kpd_create_attr(&kpd_pdrv.driver);
	if (err) {
		kpd_notice("create attr file fail\n");
		kpd_delete_attr(&kpd_pdrv.driver);
		return err;
	}
//#ifdef OPLUS_FEATURE_TP_BASIC
	kpd_oplus->dev = dev;
	dev_set_drvdata(dev, kpd_oplus);
	kpd_oplus->pdev = pdev;

	if (kpd_dts_data.kpd_sw_rstkey == KEY_VOLUMEUP) {
		vol_key_info.homekey_as_vol_up = true;
	} else {
		vol_key_info.homekey_as_vol_up = false;
	}

	if (!vol_key_info.homekey_as_vol_up) {  // means not home key as volume up, defined on dws
		err = kpd_request_named_gpio(kpd_oplus, "keypad,volume-up",
				&vol_key_info.vol_up_gpio);

		if (err) {
			pr_err("%s lfc request keypad,volume-up fail\n", __func__);
			return -1;
		}
		err = gpio_direction_input(vol_key_info.vol_up_gpio);

		if (err < 0) {
			dev_err(&kpd_oplus->pdev->dev,
				"gpio_direction_input failed for vol_up INT.\n");
			return -1;
		}
	}

	err = kpd_request_named_gpio(kpd_oplus, "keypad,volume-down",
			&vol_key_info.vol_down_gpio);
	if (err) {
		pr_err("%s request keypad,volume-down fail\n", __func__);
		return -1;
	}
	err = gpio_direction_input(vol_key_info.vol_down_gpio);

	if (err < 0) {
		dev_err(&kpd_oplus->pdev->dev,
			"gpio_direction_input failed for vol_down INT.\n");
		return -1;
	}

	if (init_custom_gpio_state(pdev) < 0) {
		pr_err("init gpio state failed\n");
		return -1;
	}

	//disable keypad scan function
	kpd_wakeup_src_setting(0);

	//enable_irq(vol_key_info.vol_up_irq);
	vol_key_info.vol_up_irq_enabled = 1;
	//enable_irq(vol_key_info.vol_down_irq);
	vol_key_info.vol_down_irq_enabled = 1;

	__set_bit(KEY_VOLUMEDOWN, kpd_input_dev->keybit);
	__set_bit(KEY_VOLUMEUP, kpd_input_dev->keybit);
	__set_bit(KEY_POWER, kpd_input_dev->keybit);
//#endif /*OPLUS_FEATURE_TP_BASIC*/
//#ifdef OPLUS_FEATURE_TP_BASIC
	init_proc_aee_kpd_enable();
	if(get_eng_version() == AGING ||
	   get_eng_version() == PREVERSION ||
	   get_eng_version() == HIGH_TEMP_AGING ||
	   get_eng_version() == FACTORY) {
		aee_kpd_enable = 1;
	} else {
		aee_kpd_enable = 0;
	}
//#endif /* OPLUS_FEATURE_TP_BASIC */
	/* Add kpd debug node */
	mt_kpd_debugfs();

	kpd_info("kpd_probe OK.\n");

	return err;
}

static int kpd_pdrv_suspend(struct platform_device *pdev, pm_message_t state)
{
	kpd_suspend = true;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		kpd_print("kpd_early_suspend wake up source enable!! (%d)\n",
				kpd_suspend);
	} else {
		kpd_wakeup_src_setting(0);
		kpd_print("kpd_early_suspend wake up source disable!! (%d)\n",
				kpd_suspend);
	}
#endif
	enable_irq_wake(vol_key_info.vol_down_irq);

	kpd_print("suspend!! (%d)\n", kpd_suspend);
	return 0;
}

static int kpd_pdrv_resume(struct platform_device *pdev)
{
	kpd_suspend = false;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		kpd_print("kpd_early_suspend wake up source enable!! (%d)\n",
				kpd_suspend);
	} else {
		kpd_print("kpd_early_suspend wake up source resume!! (%d)\n",
				kpd_suspend);
		kpd_wakeup_src_setting(1);
	}
#endif
	disable_irq_wake(vol_key_info.vol_down_irq);

	kpd_print("resume!! (%d)\n", kpd_suspend);
	return 0;
}

static const struct of_device_id kpd_of_match[] = {
	{.compatible = "mediatek,mt8167-keypad"},
	{.compatible = "mediatek,kp"},
	{},
};

static struct platform_driver kpd_pdrv = {
	.probe = kpd_pdrv_probe,
	.suspend = kpd_pdrv_suspend,
	.resume = kpd_pdrv_resume,
	.driver = {
		   .name = KPD_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = kpd_of_match,
		   },
};

module_platform_driver(kpd_pdrv);

MODULE_AUTHOR("Mediatek Corporation");
MODULE_DESCRIPTION("MTK Keypad (KPD) Driver");
MODULE_LICENSE("GPL");
