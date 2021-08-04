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
#include "tpd.h"

#define DTS_INT_GPIO	"touch,irq-gpio"
#define DTS_RESET_GPIO	"touch,reset-gpio"
#define DTS_OF_NAME	"ilitek,ilitek-ts-spi"
#define MTK_RST_GPIO	GTP_RST_PORT
#define MTK_INT_GPIO	GTP_INT_PORT
bool is_resume = false;

extern int get_boot_mode(void);

extern struct tpd_device *tpd;

int ili_ctpmodule = 0;

unsigned char fw_xl_auo[] = {
#include "firmware_ili/xl_auo_fw.ili"
};
unsigned char fw_txd_auo[] = {
#include "firmware_ili/txd_auo_fw.ili"
};

struct upgrade_ili_fw_info ili_fw_list[] = {
	{XL_AUO, "TRULY", fw_xl_auo,(int)sizeof(fw_xl_auo), REQUEST_XL_AUO_FW_PATH,FILP_OPEN_XL_AUO_FW_PATH,XL_AUO_ILITEK_INI_PATH,OPLUS_SIGN_XL_AUO_FW_PATH},
	{TXD_AUO, "TXD", fw_txd_auo, (int)sizeof(fw_txd_auo),REQUEST_TXD_AUO_FW_PATH,FILP_OPEN_TXD_AUO_FW_PATH,TXD_AUO_ILITEK_INI_PATH,OPLUS_SIGN_TXD_AUO_FW_PATH},
};
unsigned char* CTPM_FW =NULL;
struct upgrade_ili_fw_info *ili_fw;

#ifdef ODM_WT_EDIT
void lcd_resume_load_ili_fw(void)
{
	int	ret;

	ipio_info("lcd resume load ili fw begin idev->power_status = %d\n",idev->power_status);
	mutex_lock(&idev->touch_mutex);

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
EXPORT_SYMBOL(lcd_resume_load_ili_fw);
#endif

void ilitek_plat_tp_reset(void)
{
	ipio_info("edge delay = %d\n", idev->rst_edge_delay);
	tpd_gpio_output(idev->tp_rst, 1);
	mdelay(10);
	tpd_gpio_output(idev->tp_rst, 0);
	mdelay(5);
	tpd_gpio_output(idev->tp_rst, 1);
	mdelay(idev->rst_edge_delay);
}

void ilitek_plat_input_register(void)
{
	int i;

	idev->input = tpd->dev;

	if (tpd_dts_data.use_tpd_button) {
		for (i = 0; i < tpd_dts_data.tpd_key_num; i++)
			input_set_capability(idev->input, EV_KEY, tpd_dts_data.tpd_key_local[i]);
	}

	/* set the supported event type for input device */
	set_bit(EV_ABS, idev->input->evbit);
	set_bit(EV_SYN, idev->input->evbit);
	set_bit(EV_KEY, idev->input->evbit);
	set_bit(BTN_TOUCH, idev->input->keybit);
	set_bit(BTN_TOOL_FINGER, idev->input->keybit);
	set_bit(INPUT_PROP_DIRECT, idev->input->propbit);

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

	idev->vdd = regulator_get(tpd->tpd_dev, vdd_name);
	if (ERR_ALLOC_MEM(idev->vdd)) {
		ipio_err("regulator_get VDD fail\n");
		idev->vdd = NULL;
	}

	tpd->reg = idev->vdd;

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

	idev->tp_int = MTK_INT_GPIO;
	idev->tp_rst = MTK_RST_GPIO;

	ipio_info("TP INT: %d\n", idev->tp_int);
	ipio_info("TP RESET: %d\n", idev->tp_rst);
#if 0
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

out:
	gpio_direction_input(idev->tp_int);
#endif
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
		ipio_info("Get an INT for mp test, ignore\n");
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
	mutex_lock(&idev->touch_mutex);
	ilitek_tddi_report_handler();
	mutex_unlock(&idev->touch_mutex);
	return IRQ_HANDLED;
}

static int ilitek_plat_irq_register(void)
{
	int ret = 0;

	struct device_node *node;

	node = of_find_matching_node(NULL, touch_of_match);
	if (node)
		idev->irq_num = irq_of_parse_and_map(node, 0);

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

static void tpd_resume(struct device *h)
{
	ilitek_tddi_sleep_handler(TP_RESUME);
}

static void tpd_suspend(struct device *h)
{
	ilitek_tddi_sleep_handler(TP_SUSPEND);
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
	res = tp_usb_register_client(&idev->notifier_usb);
}
#endif
static int ilitek_plat_probe(void)
{
	ipio_info("platform probe\n");

	ili_fw = &ili_fw_list[ili_ctpmodule];
	ipio_info("ili_fw->id = %d,ili_fw->module_vendor =%s,ili_fw->fw_len =%d,ili_fw->firmware_bin_name = %s,ili_fw->mp_ini_name = %s\n",ili_fw->id,ili_fw->module_vendor, ili_fw->fw_len,ili_fw->firmware_bin_name,ili_fw->mp_ini_name);
	CTPM_FW = ili_fw->firmware_i;
	if (REGULATOR_POWER)
		ilitek_plat_regulator_power_init();

	ilitek_plat_gpio_register();

	if (ilitek_tddi_init() < 0) {
		ipio_err("platform probe failed\n");
		return -ENODEV;
	}
	ilitek_plat_irq_register();
	tpd_load_status = 1;
	
	idev->boot_mode = get_boot_mode();
	ipio_info("==========idev->boot_mode = %d\n",idev->boot_mode);
	//if ((ts->boot_mode == MSM_BOOT_MODE__FACTORY || ts->boot_mode == MSM_BOOT_MODE__RF || ts->boot_mode == MSM_BOOT_MODE__WLAN))
	if ((idev->boot_mode == 3 || idev->boot_mode == 4 || idev->boot_mode == 5)) {
		ipio_info("boot_mode is FACTORY,RF and WLAN not need to enable irq\n");
		disable_irq_nosync(idev->irq_num);
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
	.plat_type = TP_PLAT_MTK,
	.owner = THIS_MODULE,
	.name = TDDI_DEV_ID,
	.of_match_table = of_match_ptr(tp_match_table),
	.plat_probe = ilitek_plat_probe,
	.plat_remove = ilitek_plat_remove,
};

static int tpd_local_init(void)
{
	ipio_info("TPD init device driver\n");

	if (ilitek_tddi_dev_init(&hwif) < 0) {
		ipio_err("Failed to register i2c/spi bus driver\n");
		return -ENODEV;
	}
	if (tpd_load_status == 0) {
		ipio_err("Add error touch panel driver\n");
		return -1;
	}
	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
				   tpd_dts_data.tpd_key_dim_local);
	}
	tpd_type_cap = 1;
	return 0;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = TDDI_DEV_ID,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
};

//extern char Lcm_name1[256];
 static char Lcm_name1[256];
static int __init ilitek_plat_dev_init(void)
{
	int ret = 0;
	char *temp;
	int nt_ctpmodule =0;

	ipio_info("ILITEK TP driver init for MTK\n");
	/*samir add*/
	temp = Lcm_name1;
	ipio_info("samir : %s",temp);
	nt_ctpmodule= strncmp(temp,"ilt9881h_truly_hdp_dsi_vdo_lcm_drv",strlen("ilt9881h_truly_hdp_dsi_vdo_lcm_drv"));
	if ( nt_ctpmodule == 0 ) {
		ili_ctpmodule = 0;
		ipio_info("this is ilt9881h_xl touchscreen\n");
	}else{
		nt_ctpmodule = strncmp(temp,"ilt9881h_txd_hdp_dsi_vdo_lcm_drv",strlen("ilt9881h_txd_hdp_dsi_vdo_lcm_drv"));
		if(nt_ctpmodule == 0){
			ili_ctpmodule = 1;
			ipio_info("this is ilt9881h_xl touchscreen\n");
		}else{
			ipio_info("not found touchscreen mould\n");
			return -1;
		}
	}
	tpd_get_dts_info();
	ret = tpd_driver_add(&tpd_device_driver);
	if (ret < 0) {
		ipio_err("ILITEK add TP driver failed\n");
		tpd_driver_remove(&tpd_device_driver);
		return -ENODEV;
	}
	return 0;
}

static void __exit ilitek_plat_dev_exit(void)
{
	ipio_info("ilitek driver has been removed\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(ilitek_plat_dev_init);
module_exit(ilitek_plat_dev_exit);
MODULE_AUTHOR("ILITEK");
MODULE_LICENSE("GPL");
