/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "ilitek.h"

#define DTS_INT_GPIO	"touch,irq-gpio"
#define DTS_RESET_GPIO	"touch,reset-gpio"
#define DTS_OF_NAME	"tchip,ilitek"
extern struct ili_gesture_info * gesture_report_data;
bool is_resume = false;
bool is_ilitek_tp = false;
struct wakeup_source *reload_fw_ws = NULL;
#ifdef ODM_WT_EDIT
 int ili_ctpmodule = -1;
unsigned char fw_xl[] = {
#include "firmware_ili/AUO/RA105A0.ili"
};

unsigned char fw_inx[] = {
#include "firmware_ili/INX6500/RA105X0.ili"
};

unsigned char fw_inx_6217[] = {
#include "firmware_ili/INX6217/RA105X6217.ili"
};

#ifdef ODM_WT_EDIT
struct upgrade_ili_fw_info ili_fw_list[] = {
    {XL, "TRULY", fw_xl,(int)sizeof(fw_xl), REQUEST_FW_PATH_AUO, XL_INI_NAME_PATH,OPLUS_SIGN_AUO},
    {INX, "inx", fw_inx, (int)sizeof(fw_inx),REQUEST_FW_PATH_INX, INX_INI_NAME_PATH,OPLUS_SIGN_INX},
	{INX6217, "inx6217", fw_inx_6217, (int)sizeof(fw_inx_6217),REQUEST_FW_PATH_INX_6217, INX_INI_NAME_PATH_6217,NULL},
};
#endif

unsigned char* CTPM_FW =NULL;
struct upgrade_ili_fw_info *ili_fw;
#endif

void lcd_resume_load_ili_fw(void)
{
	int	pwr, ret;

	ipio_info("lcd resume load ili fw begin idev->power_status = %d\n",idev->power_status);
	mutex_lock(&idev->touch_mutex);
	if (idev->gesture)
	{
		disable_irq_wake(idev->irq_num);
		if(idev->power_status ==false) {
			pwr = power_on(idev,true);
			idev->power_status = true;
			if(pwr != 0) {
				ipio_info("power off fail\n");
			}			
		}
	}
	else
	{
		if(idev->power_status ==false) {
			pwr = power_on(idev,true);
			idev->power_status = true;
		}

		if(pwr != 0) {
			ipio_info("power off fail\n");
		}
	}
	ilitek_tddi_reset_ctrl(idev->reset);
	idev->ignore_first_irq = true;
	ret = ilitek_ice_mode_ctrl(ENABLE, OFF);
	if (ret < 0){
		is_resume = false;
	}else{
		is_resume = true;
	}
	mutex_unlock(&idev->touch_mutex);
	schedule_work(&idev->resume_work_queue);

	ipio_info("lcd resume load ili fw end 20190505");
	return;
}



int  power_on(struct ilitek_tddi_dev *power_idev, bool on)
{
    int rc = 0;
    if (!on) {
       ipio_info("ctp power_off\n");
       goto power_off;
	}
	ipio_info("ctp power_on start\n");
    rc = regulator_enable(power_idev->iovcc_pwr);
    if ( rc != 0 ) {
		ipio_err("enable iovcc fail\n");
        goto gpio_pwr_err;
    }
	rc = regulator_enable(power_idev->vsp_pwr);
    if ( rc != 0 ) {
		ipio_err("enable vsp fail\n");
        goto lab_pwr_err;
    }
    rc = regulator_enable(power_idev->vsn_pwr);
    if ( rc != 0 ) {
		ipio_err("enable vsn fail\n");
        goto ibb_pwr_err;
    }
	printk("ctp power_on end\n");

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

static int power_init(struct ilitek_tddi_dev *power_idev, bool on)
{
    int rc = 0;
    if ( !on ) {
		ipio_info("power_init is deny\n");
        goto pwr_deny;
	}
    power_idev->iovcc_pwr = regulator_get(&idev->spi->dev, "vddio");
	if ( IS_ERR(power_idev->iovcc_pwr) ) {
        rc = PTR_ERR(power_idev->iovcc_pwr);
		ipio_err("Regulator get failed power_idev->iovcc_pwr rc=%d\n",rc);
		goto gpio_pwr_err;
    }
    power_idev->vsp_pwr = regulator_get(&idev->spi->dev, "lab");
	if ( IS_ERR(power_idev->vsp_pwr) ) {
        rc = PTR_ERR(power_idev->vsp_pwr);
		ipio_err("Regulator get failed power_idev->vsp_pwr rc=%d\n",rc);
		goto lab_pwr_err;
    }
    power_idev->vsn_pwr = regulator_get(&idev->spi->dev, "ibb");
	if ( IS_ERR(power_idev->vsn_pwr) ) {
        rc = PTR_ERR(power_idev->vsn_pwr);
		ipio_err("Regulator get failed power_idev->vsn_pwr rc=%d\n",rc);
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

void ilitek_plat_tp_reset(void)
{
	ipio_info("edge delay = %d\n", idev->rst_edge_delay);
	gpio_direction_output(idev->tp_rst, 1);
	mdelay(1);
	gpio_set_value(idev->tp_rst, 0);
	mdelay(5);
	gpio_set_value(idev->tp_rst, 1);
	mdelay(idev->rst_edge_delay);
}

void ilitek_plat_input_register(void)
{
	ipio_info();

	idev->input = input_allocate_device();
	if (ERR_ALLOC_MEM(idev->input)) {
		ipio_err("Failed to allocate touch input device\n");
		input_free_device(idev->input);
		return;
	}

	idev->input->name = idev->hwif->name;
	idev->input->phys = idev->phys;
	idev->input->dev.parent = idev->dev;
	idev->input->id.bustype = idev->hwif->bus_type;

	/* set the supported event type for input device */
	set_bit(EV_ABS, idev->input->evbit);
	set_bit(EV_SYN, idev->input->evbit);
	set_bit(EV_KEY, idev->input->evbit);
	set_bit(BTN_TOUCH, idev->input->keybit);
	set_bit(BTN_TOOL_FINGER, idev->input->keybit);
	set_bit(INPUT_PROP_DIRECT, idev->input->propbit);

	input_set_abs_params(idev->input, ABS_MT_POSITION_X, TOUCH_SCREEN_X_MIN, TOUCH_SCREEN_X_MAX - 1, 0, 0);
	input_set_abs_params(idev->input, ABS_MT_POSITION_Y, TOUCH_SCREEN_Y_MIN, TOUCH_SCREEN_Y_MAX - 1, 0, 0);
	input_set_abs_params(idev->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(idev->input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	if (MT_PRESSURE)
		input_set_abs_params(idev->input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	if (MT_B_TYPE) {
#if KERNEL_VERSION(3, 7, 0) <= LINUX_VERSION_CODE
		input_mt_init_slots(idev->input, MAX_TOUCH_NUM, INPUT_MT_DIRECT);
#else
		input_mt_init_slots(idev->input, MAX_TOUCH_NUM);
#endif /* LINUX_VERSION_CODE */
	} else {
		input_set_abs_params(idev->input, ABS_MT_TRACKING_ID, 0, MAX_TOUCH_NUM, 0, 0);
	}

	/* Gesture keys register */
	input_set_capability(idev->input, EV_KEY, KEY_POWER);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_UP);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_DOWN);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_LEFT);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_RIGHT);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_O);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_E);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_M);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_W);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_S);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_V);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_Z);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_C);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_F);
	input_set_capability(idev->input, EV_KEY, KEY_F4);

	__set_bit(KEY_GESTURE_POWER, idev->input->keybit);
	__set_bit(KEY_GESTURE_UP, idev->input->keybit);
	__set_bit(KEY_GESTURE_DOWN, idev->input->keybit);
	__set_bit(KEY_GESTURE_LEFT, idev->input->keybit);
	__set_bit(KEY_GESTURE_RIGHT, idev->input->keybit);
	__set_bit(KEY_GESTURE_O, idev->input->keybit);
	__set_bit(KEY_GESTURE_E, idev->input->keybit);
	__set_bit(KEY_GESTURE_M, idev->input->keybit);
	__set_bit(KEY_GESTURE_W, idev->input->keybit);
	__set_bit(KEY_GESTURE_S, idev->input->keybit);
	__set_bit(KEY_GESTURE_V, idev->input->keybit);
	__set_bit(KEY_GESTURE_Z, idev->input->keybit);
	__set_bit(KEY_GESTURE_C, idev->input->keybit);
	__set_bit(KEY_GESTURE_F, idev->input->keybit);
	__set_bit(KEY_F4, idev->input->keybit);

	/* register the input device to input sub-system */
	if (input_register_device(idev->input) < 0) {
		ipio_err("Failed to register touch input device\n");
		input_unregister_device(idev->input);
		input_free_device(idev->input);
	}
}

void ilitek_plat_regulator_power_on(bool status)
{
	ipio_info("%s\n", status ? "POWER ON" : "POWER OFF");

	if (status) {
		if (idev->vdd) {
			if (regulator_enable(idev->vdd) < 0)
				ipio_err("regulator_enable VDD fail\n");
		}
		if (idev->vcc) {
			if (regulator_enable(idev->vcc) < 0)
				ipio_err("regulator_enable VCC fail\n");
		}
	} else {
		if (idev->vdd) {
			if (regulator_disable(idev->vdd) < 0)
				ipio_err("regulator_enable VDD fail\n");
		}
		if (idev->vcc) {
			if (regulator_disable(idev->vcc) < 0)
				ipio_err("regulator_enable VCC fail\n");
		}
	}
	atomic_set(&idev->ice_stat, DISABLE);
	mdelay(5);
}

static void ilitek_plat_regulator_power_init(void)
{
	const char *vdd_name = "vdd";
	const char *vcc_name = "vcc";

	idev->vdd = regulator_get(idev->dev, vdd_name);
	if (ERR_ALLOC_MEM(idev->vdd)) {
		ipio_err("regulator_get VDD fail\n");
		idev->vdd = NULL;
	}
	if (regulator_set_voltage(idev->vdd, VDD_VOLTAGE, VDD_VOLTAGE) < 0)
		ipio_err("Failed to set VDD %d\n", VDD_VOLTAGE);

	idev->vcc = regulator_get(idev->dev, vcc_name);
	if (ERR_ALLOC_MEM(idev->vcc)) {
		ipio_err("regulator_get VCC fail.\n");
		idev->vcc = NULL;
	}
	if (regulator_set_voltage(idev->vcc, VCC_VOLTAGE, VCC_VOLTAGE) < 0)
		ipio_err("Failed to set VCC %d\n", VCC_VOLTAGE);

	ilitek_plat_regulator_power_on(true);
}

static int ilitek_plat_gpio_register(void)
{
	int ret = 0;
	u32 flag;
	struct device_node *dev_node = idev->dev->of_node;

	idev->tp_int = of_get_named_gpio_flags(dev_node, DTS_INT_GPIO, 0, &flag);
	idev->tp_rst = of_get_named_gpio_flags(dev_node, DTS_RESET_GPIO, 0, &flag);

	ipio_info("TP INT: %d\n", idev->tp_int);
	ipio_info("TP RESET: %d\n", idev->tp_rst);

	if (!gpio_is_valid(idev->tp_int)) {
		ipio_err("Invalid INT gpio: %d\n", idev->tp_int);
		return -EBADR;
	}

	if (!gpio_is_valid(idev->tp_rst)) {
		ipio_err("Invalid RESET gpio: %d\n", idev->tp_rst);
		return -EBADR;
	}

	ret = gpio_request(idev->tp_int, "TP_INT");
	if (ret < 0) {
		ipio_err("Request IRQ GPIO failed, ret = %d\n", ret);
		gpio_free(idev->tp_int);
		ret = gpio_request(idev->tp_int, "TP_INT");
		if (ret < 0) {
			ipio_err("Retrying request INT GPIO still failed , ret = %d\n", ret);
			goto out;
		}
	}

	ret = gpio_request(idev->tp_rst, "TP_RESET");
	if (ret < 0) {
		ipio_err("Request RESET GPIO failed, ret = %d\n", ret);
		gpio_free(idev->tp_rst);
		ret = gpio_request(idev->tp_rst, "TP_RESET");
		if (ret < 0) {
			ipio_err("Retrying request RESET GPIO still failed , ret = %d\n", ret);
			goto out;
		}
	}

	gpio_direction_input(idev->tp_int);
out:
	return ret;
}

void ilitek_plat_irq_disable(void)
{
	unsigned long flag;

	spin_lock_irqsave(&idev->irq_spin, flag);

	if (atomic_read(&idev->irq_stat) == DISABLE)
		goto out;

	if (!idev->irq_num) {
		ipio_err("gpio_to_irq (%d) is incorrect\n", idev->irq_num);
		goto out;
	}

	disable_irq_nosync(idev->irq_num);
	atomic_set(&idev->irq_stat, DISABLE);
	ipio_debug(DEBUG_PLAT, "Disable irq success\n");

out:
	spin_unlock_irqrestore(&idev->irq_spin, flag);
}

void ilitek_plat_irq_enable(void)
{
	unsigned long flag;

	spin_lock_irqsave(&idev->irq_spin, flag);

	if (atomic_read(&idev->irq_stat) == ENABLE)
		goto out;

	if (!idev->irq_num) {
		ipio_err("gpio_to_irq (%d) is incorrect\n", idev->irq_num);
		goto out;
	}

	enable_irq(idev->irq_num);
	atomic_set(&idev->irq_stat, ENABLE);
	ipio_debug(DEBUG_PLAT, "Enable irq success\n");

out:
	spin_unlock_irqrestore(&idev->irq_spin, flag);
}

static irqreturn_t ilitek_plat_isr_top_half(int irq, void *dev_id)
{
	ipio_debug(DEBUG_PLAT, "report: %d, rst: %d, fw: %d, switch: %d, mp: %d, sleep: %d, esd: %d\n",
			idev->report,
			atomic_read(&idev->tp_reset),
			atomic_read(&idev->fw_stat),
			atomic_read(&idev->tp_sw_mode),
			atomic_read(&idev->mp_stat),
			atomic_read(&idev->tp_sleep),
			atomic_read(&idev->esd_stat));

	if (irq != idev->irq_num) {
		ipio_err("Incorrect irq number (%d)\n", irq);
		return IRQ_NONE;
	}
	if (atomic_read(&idev->mp_int_check) == ENABLE) {
		atomic_set(&idev->mp_int_check, DISABLE);
		ipio_info("Get an INT for mp, ignore\n");
		return IRQ_HANDLED;
	}

	if (!idev->report || atomic_read(&idev->tp_reset) ||
		atomic_read(&idev->fw_stat) || atomic_read(&idev->tp_sw_mode) ||
		atomic_read(&idev->mp_stat) || atomic_read(&idev->tp_sleep) ||
		atomic_read(&idev->esd_stat)) {
			ipio_debug(DEBUG_PLAT, "ignore interrupt !\n");
			return IRQ_HANDLED;
	}
	return IRQ_WAKE_THREAD;
}

static irqreturn_t ilitek_plat_isr_bottom_half(int irq, void *dev_id)
{
	if (mutex_is_locked(&idev->touch_mutex)) {
		ipio_debug(DEBUG_PLAT, "touch is locked, ignore\n");
		return IRQ_HANDLED;
	}

    if (idev->ignore_first_irq) {
        ipio_info("ignore_first_irq\n");
        idev->ignore_first_irq = false;
        return IRQ_HANDLED;
    }
	mutex_lock(&idev->touch_mutex);
	if((idev->suspend)&&(idev->gesture)) {
		if (reload_fw_ws) {
			__pm_stay_awake(reload_fw_ws);
		}
	}
	ilitek_tddi_report_handler();
	if(idev->suspend) {
		if (reload_fw_ws) {
			__pm_relax(reload_fw_ws);
		}
	}
	mutex_unlock(&idev->touch_mutex);
	return IRQ_HANDLED;
}

static int ilitek_plat_irq_register(void)
{
	int ret = 0;

	idev->irq_num  = gpio_to_irq(idev->tp_int);

	ipio_info("idev->irq_num = %d\n", idev->irq_num);

	ret = devm_request_threaded_irq(idev->dev, idev->irq_num,
				   ilitek_plat_isr_top_half,
				   ilitek_plat_isr_bottom_half,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ilitek", NULL);

	if (ret != 0)
		ipio_err("Failed to register irq handler, irq = %d, ret = %d\n", idev->irq_num, ret);

	atomic_set(&idev->irq_stat, ENABLE);
	return ret;
}
#ifdef CONFIG_FB
static int ilitek_plat_notifier_msm_drm(struct notifier_block *self, unsigned long event, void *data)
{
	int *blank;
	struct msm_drm_notifier *evdata = data;

	ipio_info("Notifier's event = %ld\n", event);
	/*
	 *	FB_EVENT_BLANK(0x09): A hardware display blank change occurred.
	 *	FB_EARLY_EVENT_BLANK(0x10): A hardware display blank early change occurred.
	 */
	if (evdata && evdata->data) {
		blank = evdata->data;
		switch (*blank) {
		case MSM_DRM_BLANK_POWERDOWN:
			if (TP_SUSPEND_PRIO) {
				if (event != MSM_DRM_EARLY_EVENT_BLANK)
					return NOTIFY_DONE;
			} else {
				if (event != MSM_DRM_EVENT_BLANK)
					return NOTIFY_DONE;
			}
			ilitek_tddi_sleep_handler(TP_SUSPEND);
			break;
		case MSM_DRM_BLANK_UNBLANK:
		//case MSM_DRM_BLANK_NORMAL:
			if (event == MSM_DRM_EVENT_BLANK)
				ilitek_tddi_sleep_handler(TP_RESUME);
			break;
		default:
			ipio_err("Unknown event, blank = %d\n", *blank);
			break;
		}
	}
	return NOTIFY_OK;
}
#else
static void ilitek_plat_early_suspend(struct early_suspend *h)
{
	ilitek_tddi_sleep_handler(TP_SUSPEND);
}

static void ilitek_plat_late_resume(struct early_suspend *h)
{
	ilitek_tddi_sleep_handler(TP_RESUME);
}
#endif

static void ilitek_plat_sleep_init(void)
{
	int ret = 0;
#ifdef CONFIG_FB
	ipio_info("Init notifier_fb struct\n");
	idev->notifier_fb.notifier_call = ilitek_plat_notifier_msm_drm;
#ifdef CONFIG_PLAT_SPRD
	if (adf_register_client(&idev->notifier_fb))
		ipio_err("Unable to register notifier_fb\n");
#else
	ret = msm_drm_register_client(&idev->notifier_fb);
	//ipio_err("Unable to register notifier_fb\n");
#endif /* CONFIG_PLAT_SPRD */
#else
	ipio_info("Init eqarly_suspend struct\n");
	idev->early_suspend.suspend = ilitek_plat_early_suspend;
	idev->early_suspend.resume = ilitek_plat_late_resume;
	idev->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	register_early_suspend(&idev->early_suspend);
#endif
}

#ifdef ODM_WT_EDIT
/**
 * add aer phone function
 */
static void ipd_headset_work_queue(struct work_struct *work)
 {
	ipio_info("ipd_headset_work_queue +++++headset_state = %ld\n",idev->headset_state);
	if ((idev->suspend)){
		ipio_info("suspend = %d,isUpgrading = %d,don't need headset function",idev->suspend,idev->fw_upgrade_mode);
		return;
	}
	mutex_lock(&idev->touch_mutex);
	if(idev->headset_state == 1) {
		//core_config_ear_phone_ctrl(true);
		ilitek_tddi_ic_func_ctrl("headset", ENABLE);
	}
	else if(idev->headset_state == 0) {
		//	core_config_ear_phone_ctrl(false);
		ilitek_tddi_ic_func_ctrl("headset", DISABLE);
	}
		else
			ipio_info("invild headset statues value");
	mutex_unlock(&idev->touch_mutex);
 }

static int ipd_notifie_headset(struct notifier_block *nb,unsigned long value, void *data)
{
	idev->headset_state = value;
	ipio_info("value == %ld,ipd->headset_state = %ld data = %p",value,idev->headset_state,data);
	schedule_work(&idev->headset_work_queue);
	return 0;
}

static void ipd_headset_notifier_init(void)
{
	int res = 0;
	ipio_info("ipd_headset_notifier_init\n");
	idev->notifier_headset.notifier_call = ipd_notifie_headset;
	res = headset_register_client(&idev->notifier_headset);
}
#endif

#ifdef ODM_WT_EDIT
/**
 * add aer phone function
 */
static void ipd_usb_work_queue(struct work_struct *work)
 {
	ipio_info("ipd_usb_work_queue +++++usb_state = %ld\n",idev->usb_state);
	if ((idev->suspend)){
		ipio_info("suspend = %d,isUpgrading = %d,don't need usb function",idev->suspend,idev->fw_upgrade_mode);
		return;
	}
	mutex_lock(&idev->touch_mutex);
	if(idev->usb_state == 1) {
		ilitek_tddi_ic_func_ctrl("plug", DISABLE);
	}
	else if(idev->usb_state == 0) {
		ilitek_tddi_ic_func_ctrl("plug", ENABLE);
	}
		else
			ipio_info("invild usb statues value");
	mutex_unlock(&idev->touch_mutex);
 }

static int ipd_notifie_usb(struct notifier_block *nb,unsigned long value, void *data)
{
	idev->usb_state = value;
	ipio_info("value == %ld,ipd->usb_state = %ld data = %p",value,idev->usb_state,data);
	schedule_work(&idev->usb_work_queue);
	return 0;
}

static void ipd_usb_notifier_init(void)
{
	int res = 0;
	ipio_info("ipd_usb_notifier_init\n");
	idev->notifier_usb.notifier_call = ipd_notifie_usb;
	res = usb_register_client(&idev->notifier_usb);
}
#endif

static int ilitek_plat_probe(void)
{
	int ret;
#ifdef ODM_WT_EDIT
	char *temp = NULL;
	char * cmdline_tp = NULL;
	char *oplus_ftmmode_start;
#endif

	ipio_info("platform probe\n");
#ifdef ODM_WT_EDIT
/*
dsi_ili9881h_truly_video_display  	 	（群创的临时屏）
dsi_ili9881h_truly_auo_video_display 	（信利德正式屏）
dsi_ili9881h_innolux_inx_video_display  （群创的正式屏）
*/
	cmdline_tp = strstr(saved_command_line,"dsi_ili9881h_");
	printk("cmdline_tp = %s\n",cmdline_tp);
	if ( cmdline_tp == NULL ) {
		printk("get qcom,dsi_ili9881h_ fail,This not ilitek");
		return -1;
	}
	temp = cmdline_tp + strlen("dsi_ili9881h_");
	printk("temp = %s\n",temp);

	ili_ctpmodule = strncmp(temp,"truly_auo_video",strlen("truly_auo_video"));
	if(ili_ctpmodule == 0){
		printk("this is XL AUO panel");
	} else{
		ili_ctpmodule = strncmp(temp,"innolux_inx_video",strlen("innolux_inx_video"));
		if(ili_ctpmodule == 0){
			printk("this is INX  panel");
			ili_ctpmodule = 1;
		}else{
			ili_ctpmodule = strncmp(temp,"truly_video",strlen("truly_video"));
			if(ili_ctpmodule == 0){
				printk("this is INX 6.217 TEMP panel");
				ili_ctpmodule = 2;
			}else{
				ili_ctpmodule = -1;
				printk("This ilitek panenl is not used in this project");
				return -1;
			}
		}
	}
	ili_fw = &ili_fw_list[ili_ctpmodule];
	ipio_info("ili_fw->id = %d,ili_fw->module_vendor =%s,ili_fw->fw_len =%d, \
		ili_fw->firmware_bin_name = %s,ili_fw->mp_ini_name = %s\n", \
		ili_fw->id,ili_fw->module_vendor, ili_fw->fw_len, \
		ili_fw->firmware_bin_name,ili_fw->mp_ini_name);
	CTPM_FW = ili_fw->firmware_i;
#endif

	if (REGULATOR_POWER)
		ilitek_plat_regulator_power_init();
    ret = power_init(idev, true);
	if (ret) {
        ipio_info("ilitek power init fail\n");
	}
	ret = power_on(idev, true);
	if (ret) {
        ipio_info("ilitek power on fail\n");
	}
	#ifdef ODM_WT_EDIT
	idev->power_status = true;
	ipio_info("power_status = %d\n",idev->power_status);
	#endif
	ilitek_plat_gpio_register();

	if (ilitek_tddi_init() < 0) {
		gpio_free(idev->tp_int);
		gpio_free(idev->tp_rst);
		ipio_err("platform probe failed\n");
		return -ENODEV;
	}
//	ilitek_plat_input_register();

	ilitek_plat_irq_register();
	ilitek_plat_sleep_init();
	is_ilitek_tp = true;

	oplus_ftmmode_start = strstr(saved_command_line,"oplus_ftm_mode=");
	if ( oplus_ftmmode_start != NULL ) {
		oplus_ftmmode_start += strlen("oplus_ftm_mode=");
		if (strncmp(oplus_ftmmode_start, "factory2", strlen("factory2")) == 0) {
			idev->boot_mode = 3;
			ipio_info("boot_mode is FACTORY,RF and WLAN not need to enable irq\n");
			disable_irq_nosync(idev->irq_num);
		}
	}

#ifdef ODM_WT_EDIT
		INIT_WORK(&idev->headset_work_queue, ipd_headset_work_queue);
		ipd_headset_notifier_init();
#endif
#ifdef ODM_WT_EDIT
					INIT_WORK(&idev->usb_work_queue, ipd_usb_work_queue);
					ipd_usb_notifier_init();
#endif



	return 0;
}

static int ilitek_plat_remove(void)
{
	ipio_info();
	ilitek_tddi_dev_remove();
	return 0;
}

static const struct of_device_id tp_match_table[] = {
	{.compatible = DTS_OF_NAME},
	{},
};

static struct ilitek_hwif_info hwif = {
	.bus_type = TDDI_INTERFACE,
	.plat_type = TP_PLAT_QCOM,
	.owner = THIS_MODULE,
	.name = TDDI_DEV_ID,
	.of_match_table = of_match_ptr(tp_match_table),
	.plat_probe = ilitek_plat_probe,
	.plat_remove = ilitek_plat_remove,
};

static int __init ilitek_plat_dev_init(void)
{
	ipio_info("ILITEK TP driver init for QCOM\n");
	if (ilitek_tddi_dev_init(&hwif) < 0) {
		ipio_err("Failed to register i2c/spi bus driver\n");
		return -ENODEV;
	}
	return 0;
}

static void __exit ilitek_plat_dev_exit(void)
{
	ipio_info("ilitek driver has been removed\n");
}

module_init(ilitek_plat_dev_init);
module_exit(ilitek_plat_dev_exit);
MODULE_AUTHOR("ILITEK");
MODULE_LICENSE("GPL");
