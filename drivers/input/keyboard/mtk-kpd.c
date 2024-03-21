// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 MediaTek Inc.
 * Author Terry Chang <terry.chang@mediatek.com>
 */
#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>


//#ifdef OPLUS_BUG_STABILITY
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/of_gpio.h>
//#endif /*OPLUS_BUG_STABILITY*/

#define KPD_NAME	"mtk-kpd"

#define KP_STA			(0x0000)
#define KP_MEM1			(0x0004)
#define KP_MEM2			(0x0008)
#define KP_MEM3			(0x000c)
#define KP_MEM4			(0x0010)
#define KP_MEM5			(0x0014)
#define KP_DEBOUNCE		(0x0018)
#define KP_SEL			(0x0020)
#define KP_EN			(0x0024)

#define KP_COL0_SEL		(1 << 10)
#define KP_COL1_SEL		(1 << 11)
#define KP_COL2_SEL		(1 << 12)

#define KPD_DEBOUNCE_MASK	((1U << 14) - 1)
#define KPD_DOUBLE_KEY_MASK	(1U << 0)

#define KPD_NUM_MEMS	5
#define KPD_MEM5_BITS	8
#define KPD_NUM_KEYS	72	/* 4 * 16 + KPD_MEM5_BITS */
//Add for volume_down debounce
#define VOL_DOWN_DELAY_TIME    35*1000*1000

struct mtk_keypad {
	struct input_dev *input_dev;
	struct wakeup_source *suspend_lock;
	struct tasklet_struct tasklet;
	struct clk *clk;
	void __iomem *base;
	unsigned int irqnr;
	u32 key_debounce;
	u32 hw_map_num;
	//#ifdef OPLUS_BUG_STABILITY
	u32 kpd_sw_rstkey;
	//#endif
	u32 hw_init_map[KPD_NUM_KEYS];
	u16 keymap_state[KPD_NUM_MEMS];
};


/* for keymap handling */
static void kpd_keymap_handler(unsigned long data);

static int kpd_pdrv_probe(struct platform_device *pdev);

//#ifdef OPLUS_BUG_STABILITY
//#define KPD_HOME_NAME 		"mtk-kpd-home"
#define KPD_VOL_UP_NAME		"mtk-kpd-vol-up"
#define KPD_VOL_DOWN_NAME	"mtk-kpd-vol-down"

#define KEY_LEVEL_DEFAULT				1

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
	bool oplus_vol_up_flag;
}vol_key_info;

//Add for volume_down debounce
static struct hrtimer vol_down_timer;
static int vol_down_last_val = 0xff;
static void kpd_set_volumedown_irq_type(void);
static enum hrtimer_restart vol_down_timer_func(struct hrtimer *timer);


static struct mtk_keypad *g_keypad = NULL;

static irqreturn_t kpd_volumeup_irq_handler(int irq, void *dev_id);
static void kpd_volumeup_task_process(unsigned long data);
static DECLARE_TASKLET(kpd_volumekey_up_tasklet, kpd_volumeup_task_process, 0);
static irqreturn_t kpd_volumedown_irq_handler(int irq, void *dev_id);
static void kpd_volumedown_task_process(unsigned long data);
static DECLARE_TASKLET(kpd_volumekey_down_tasklet, kpd_volumedown_task_process, 0);

int aee_kpd_enable = 0;

void kpd_aee_handler(u32 keycode, u16 pressed);
static inline void kpd_update_aee_state(void);

static void kpd_volumeup_task_process(unsigned long data)
{
	pr_err("%s vol_up_val: %d\n", __func__, vol_key_info.vol_up_val);
	input_report_key(g_keypad->input_dev, KEY_VOLUMEUP, !vol_key_info.vol_up_val);
	input_sync(g_keypad->input_dev);
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
		input_report_key(g_keypad->input_dev, KEY_VOLUMEDOWN, !vol_key_info.vol_down_val);
		input_sync(g_keypad->input_dev);
		vol_down_last_val = vol_key_info.vol_down_val;
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
	disable_irq_nosync(vol_key_info.vol_down_irq);
#if 1
	vol_key_info.vol_down_val = gpio_get_value(vol_key_info.vol_down_gpio);
#endif
	if (vol_key_info.vol_down_val) {
		irq_set_irq_type(vol_key_info.vol_down_irq, IRQ_TYPE_EDGE_FALLING);
		vol_key_info.vol_down_irq_type = IRQ_TYPE_EDGE_FALLING;
	} else {
		irq_set_irq_type(vol_key_info.vol_down_irq, IRQ_TYPE_EDGE_RISING);
		vol_key_info.vol_down_irq_type = IRQ_TYPE_EDGE_RISING;
	}
	//pr_err("%s irq_type:%d, val:%d\n", __func__,
		//vol_key_info.vol_down_irq_type, vol_key_info.vol_down_val);

	//tasklet_schedule(&kpd_volumekey_down_tasklet);
	hrtimer_start(&vol_down_timer, ktime_set(0, VOL_DOWN_DELAY_TIME), HRTIMER_MODE_REL);
	return IRQ_HANDLED;
}
//#endif /*OPLUS_BUG_STABILITY*/

/* for AEE manual dump */
#define AEE_VOLUMEUP_BIT	0
#define AEE_VOLUMEDOWN_BIT	1
#define AEE_DELAY_TIME		20
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
		pr_info("aee_timer started\n");
	} else {
		/*
		  * hrtimer_cancel - cancel a timer and wait for the handler to finish.
		  * Returns:
		  * 0 when the timer was not active.
		  * 1 when the timer was active.
		 */
		if (aee_timer_started) {
			if (hrtimer_cancel(&aee_timer)) {
				pr_info("try to cancel hrtimer\n");
#if AEE_ENABLE_5_15
				if (flags_5s) {
					pr_info("Pressed Volup + Voldown5s~15s then trigger aee manual dump.\n");
					/*ZH CHEN*/
					/*aee_kernel_reminding("manual dump", "Trigger Vol Up +Vol Down 5s");*/
				}
#endif

			}
#if AEE_ENABLE_5_15
			flags_5s = false;
#endif
			aee_timer_started = false;
			pr_info("aee_timer canceled\n");
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
				pr_info("try to cancel hrtimer (5s)\n");
			aee_timer_5s_started = false;
			pr_info("aee_timer canceled (5s)\n");
		}
#endif
	}
}
void kpd_aee_handler(u32 keycode, u16 pressed)
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
	/* aee_kernel_reminding("manual dump ", "Triggered by press KEY_VOLUMEUP+KEY_VOLUMEDOWN"); */
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

static void kpd_get_keymap_state(void __iomem *kp_base, u16 state[])
{
	state[0] = readw(kp_base + KP_MEM1);
	state[1] = readw(kp_base + KP_MEM2);
	state[2] = readw(kp_base + KP_MEM3);
	state[3] = readw(kp_base + KP_MEM4);
	state[4] = readw(kp_base + KP_MEM5);
	pr_debug("kpd register = %x %x %x %x %x\n",
		state[0], state[1], state[2], state[3], state[4]);
}

static void kpd_keymap_handler(unsigned long data)
{
	int i, j;
	int pressed;
	u16 new_state[KPD_NUM_MEMS], change, mask;
	u16 hw_keycode, keycode;
	void *dest;
	struct mtk_keypad *keypad = (struct mtk_keypad *)data;

	kpd_get_keymap_state(keypad->base, new_state);

	__pm_wakeup_event(keypad->suspend_lock, 500);

	for (i = 0; i < KPD_NUM_MEMS; i++) {
		change = new_state[i] ^ keypad->keymap_state[i];
		if (!change)
			continue;

		for (j = 0; j < 16U; j++) {
			mask = (u16) 1 << j;
			if (!(change & mask))
				continue;

			hw_keycode = (i << 4) + j;

			if (hw_keycode >= KPD_NUM_KEYS)
				continue;

			/* bit is 1: not pressed, 0: pressed */
			pressed = (new_state[i] & mask) == 0U;
			pr_err("(%s) HW keycode = %d\n",
				(pressed) ? "pressed" : "released",
					hw_keycode);

			keycode = keypad->hw_init_map[hw_keycode];
			if (!keycode)
				continue;
			input_report_key(keypad->input_dev, keycode, pressed);
			input_sync(keypad->input_dev);
			pr_debug("report Linux keycode = %d\n", keycode);
		}
	}

	dest = memcpy(keypad->keymap_state, new_state, sizeof(new_state));
	enable_irq(keypad->irqnr);
}

static irqreturn_t kpd_irq_handler(int irq, void *dev_id)
{
	/* use _nosync to avoid deadlock */
	struct mtk_keypad *keypad = dev_id;

	disable_irq_nosync(keypad->irqnr);
	tasklet_schedule(&keypad->tasklet);
	return IRQ_HANDLED;
}

static int kpd_get_dts_info(struct mtk_keypad *keypad,
				struct device_node *node)
{
	int ret;

	ret = of_property_read_u32(node, "mediatek,kpd-key-debounce",
		&keypad->key_debounce);
	if (ret) {
		pr_debug("read mediatek,key-debounce-ms error.\n");
		return ret;
	}

	ret = of_property_read_u32(node, "mediatek,kpd-hw-map-num",
		&keypad->hw_map_num);
	if (ret) {
		pr_debug("read mediatek,hw-map-num error.\n");
		return ret;
	}

	if (keypad->hw_map_num > KPD_NUM_KEYS) {
		pr_debug("hw-map-num error, it cannot bigger than %d.\n",
			KPD_NUM_KEYS);
		return -EINVAL;
	}

	ret = of_property_read_u32_array(node, "mediatek,kpd-hw-init-map",
		keypad->hw_init_map, keypad->hw_map_num);

	if (ret) {
		pr_debug("hw-init-map was not defined in dts.\n");
		return ret;
	}

	//#ifdef OPLUS_BUG_STABILITY
	ret = of_property_read_u32(node, "mediatek,kpd-sw-rstkey",
		&keypad->kpd_sw_rstkey);
	if (ret) {
		pr_err("kpd-sw-rstkey was not defined in dts.\n");
		return ret;
	}
	//#endif

	pr_debug("deb= %d\n", keypad->key_debounce);

	return 0;
}

static int kpd_gpio_init(struct device *dev)
{
	struct pinctrl *keypad_pinctrl;
	struct pinctrl_state *kpd_default;

	keypad_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(keypad_pinctrl)) {
		pr_debug("Cannot find keypad_pinctrl!\n");

		return (int)PTR_ERR(keypad_pinctrl);
	}

	kpd_default = pinctrl_lookup_state(keypad_pinctrl, "default");
	if (IS_ERR(kpd_default)) {
		pr_debug("Cannot find ecall_state!\n");

		return (int)PTR_ERR(kpd_default);
	}

	return pinctrl_select_state(keypad_pinctrl,
				kpd_default);
}

//#ifdef OPLUS_BUG_STABILITY
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
		pr_err("can not find keypad pintrl1");
		return ret;
	}

	/*for key volume up*/
	if (!vol_key_info.homekey_as_vol_up) {
		volume_up_as_int = pinctrl_lookup_state(pinctrl1, "volume_up_as_int");
		if (IS_ERR(volume_up_as_int)) {
			ret = PTR_ERR(volume_up_as_int);
			pr_err("can not find gpio of volume up\n");
			return ret;
		} else {
			ret = pinctrl_select_state(pinctrl1, volume_up_as_int);
			if (ret < 0){
				pr_err("error to set gpio state\n");
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

	if (vol_key_info.homekey_as_vol_up) {
		vol_key_info.oplus_vol_up_flag = false;
	}

	/*for key of volume down*/
	volume_down_as_int = pinctrl_lookup_state(pinctrl1, "volume_down_as_int");
	if (IS_ERR(volume_down_as_int)) {
		ret = PTR_ERR(volume_down_as_int);
		pr_err("can not find gpio of  volume down\n");
		return ret;
	} else {
		ret = pinctrl_select_state(pinctrl1, volume_down_as_int);
		if (ret < 0){
			pr_err("error to set gpio state\n");
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

	pr_err(" init_custom_gpio_state End\n");
    return 0;

}

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
	if (len >= 16) {
		pr_err("aee_kpd_enable_write get an illegal value over 16 characters.\n");
		return -EFAULT;
	}
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
//#endif

static int kpd_pdrv_probe(struct platform_device *pdev)
{
	struct mtk_keypad *keypad;
	struct resource *res;
	int i;
	int err;
	//#ifdef OPLUS_BUG_STABILITY
	struct device *dev = &pdev->dev;
	struct vol_info *kpd_oplus;

	pr_err("Keypad probe start!!!\n");

	kpd_oplus = devm_kzalloc(dev, sizeof(*kpd_oplus), GFP_KERNEL);
	if (!pdev->dev.of_node) {
		pr_err("no kpd dev node\n");
		return -ENODEV;
	}
//#endif /*OPLUS_BUG_STABILITY*/

	keypad = devm_kzalloc(&pdev->dev, sizeof(*keypad), GFP_KERNEL);
	if (!keypad)
		return -ENOMEM;

	keypad->clk = devm_clk_get(&pdev->dev, "kpd");
	if (IS_ERR(keypad->clk)) {
		pr_debug("get kpd-clk fail: %d\n", (int)PTR_ERR(keypad->clk));
		return (int)PTR_ERR(keypad->clk);
	}

	err = clk_prepare_enable(keypad->clk);
	if (err) {
		pr_debug("kpd-clk prepare enable failed.\n");
		return err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err = -ENODEV;
		goto err_unprepare_clk;
	}

	keypad->base = devm_ioremap(&pdev->dev, res->start,
			resource_size(res));
	if (!keypad->base) {
		pr_debug("KP iomap failed\n");
		err = -EBUSY;
		goto err_unprepare_clk;
	}

	keypad->irqnr = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!keypad->irqnr) {
		pr_debug("KP get irqnr failed\n");
		err = -ENODEV;
		goto err_unprepare_clk;
	}

	pr_info("kp base: 0x%p, addr:0x%p,  kp irq: %d\n",
			keypad->base, &keypad->base, keypad->irqnr);
	err = kpd_gpio_init(&pdev->dev);
	if (err) {
		pr_debug("gpio init failed\n");
		goto err_unprepare_clk;
	}

	err = kpd_get_dts_info(keypad, pdev->dev.of_node);
	if (err) {
		pr_debug("get dts info failed.\n");
		goto err_unprepare_clk;
	}

	memset(keypad->keymap_state, 0xff, sizeof(keypad->keymap_state));

	keypad->input_dev = devm_input_allocate_device(&pdev->dev);
	if (!keypad->input_dev) {
		pr_notice("input allocate device fail.\n");
		err = -ENOMEM;
		goto err_unprepare_clk;
	}

	keypad->input_dev->name = KPD_NAME;
	keypad->input_dev->id.bustype = BUS_HOST;
	keypad->input_dev->dev.parent = &pdev->dev;

	__set_bit(EV_KEY, keypad->input_dev->evbit);

	for (i = 0; i < KPD_NUM_KEYS; i++) {
		if (keypad->hw_init_map[i])
			__set_bit(keypad->hw_init_map[i],
				keypad->input_dev->keybit);
	}

	err = input_register_device(keypad->input_dev);
	if (err) {
		pr_notice("register input device failed (%d)\n", err);
		goto err_unprepare_clk;
	}

	input_set_drvdata(keypad->input_dev, keypad);

	keypad->suspend_lock = wakeup_source_register(NULL, "kpd wakelock");
	if (!keypad->suspend_lock) {
		pr_notice("wakeup source init failed.\n");
		goto err_unregister_device;
	}

	tasklet_init(&keypad->tasklet, kpd_keymap_handler,
					(unsigned long)keypad);

	writew((u16)(keypad->key_debounce & KPD_DEBOUNCE_MASK),
			keypad->base + KP_DEBOUNCE);

	/* register IRQ */
	err = request_irq(keypad->irqnr, kpd_irq_handler, IRQF_TRIGGER_NONE,
			KPD_NAME, keypad);
	if (err) {
		pr_notice("register IRQ failed (%d)\n", err);
		goto err_irq;
	}

	hrtimer_init(&aee_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aee_timer.function = aee_timer_func;

#if AEE_ENABLE_5_15
	hrtimer_init(&aee_timer_5s, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aee_timer_5s.function = aee_timer_5s_func;
#endif

//#ifdef OPLUS_BUG_STABILITY
	g_keypad = keypad;
	kpd_oplus->dev = dev;
	dev_set_drvdata(dev, kpd_oplus);
	kpd_oplus->pdev = pdev;

	vol_key_info.oplus_vol_up_flag = true;

	if (keypad->kpd_sw_rstkey == KEY_VOLUMEUP) {
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
	//kpd_wakeup_src_setting(0);		//for kernel-4.19 without this function

	//enable_irq(vol_key_info.vol_up_irq);
	vol_key_info.vol_up_irq_enabled = 1;
	//enable_irq(vol_key_info.vol_down_irq);
	vol_key_info.vol_down_irq_enabled = 1;

	__set_bit(KEY_VOLUMEDOWN, keypad->input_dev->keybit);
	//__set_bit(KEY_VOLUMEUP, keypad->input_dev->keybit);

	//__set_bit(KEY_POWER, keypad->input_dev->keybit);

	if(vol_key_info.oplus_vol_up_flag == true) {
		pr_err("KEY_VOLUMEUP is set\n");
		__set_bit(KEY_VOLUMEUP, keypad->input_dev->keybit);
	}
	init_proc_aee_kpd_enable();
//#endif /* OPLUS_BUG_STABILITY */
	pr_info("kpd_probe OK.\n");

	return 0;

err_irq:
	tasklet_kill(&keypad->tasklet);

err_unregister_device:
	input_unregister_device(keypad->input_dev);

err_unprepare_clk:
	clk_disable_unprepare(keypad->clk);

	return err;
}


static int kpd_pdrv_remove(struct platform_device *pdev)
{
	struct mtk_keypad *keypad = platform_get_drvdata(pdev);

	tasklet_kill(&keypad->tasklet);
	wakeup_source_unregister(keypad->suspend_lock);
	input_unregister_device(keypad->input_dev);
	clk_disable_unprepare(keypad->clk);

	return 0;
}

static const struct of_device_id kpd_of_match[] = {
	{.compatible = "mediatek,mt6779-keypad"},
	{.compatible = "mediatek,kp"},
	{},
};

static struct platform_driver kpd_pdrv = {
	.probe = kpd_pdrv_probe,
	.remove = kpd_pdrv_remove,
	.driver = {
		   .name = KPD_NAME,
		   .of_match_table = kpd_of_match,
		   },
};

module_platform_driver(kpd_pdrv);

MODULE_AUTHOR("Mediatek Corporation");
MODULE_DESCRIPTION("MTK Keypad (KPD) Driver");
MODULE_LICENSE("GPL");
