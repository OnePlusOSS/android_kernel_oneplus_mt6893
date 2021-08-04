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

/* Debug level */
s32 ipio_debug_level = DEBUG_OUTPUT;
EXPORT_SYMBOL(ipio_debug_level);
s32 oplus_debug_level = 3;
EXPORT_SYMBOL(oplus_debug_level);
extern struct ili_gesture_info * gesture_report_data;
static struct workqueue_struct *esd_wq;
static struct workqueue_struct *bat_wq;
static struct delayed_work esd_work;
static struct delayed_work bat_work;

void ilitek_set_gesture_fail_reason(bool enable)
{

    u8 cmd[24] = {0};

    /* set symbol */
    if (ilitek_tddi_ic_func_ctrl("knock_en", 0x8) < 0)
        ipio_err("set symbol failed");

    /* enable gesture fail reason */
    cmd[0] = 0x01;
    cmd[1] = 0x0A;
    cmd[2] = 0x10;
    if (enable)
        cmd[3] = 0x01;
    else
        cmd[3] = 0x00;
    cmd[4] = 0xFF;
    cmd[5] = 0xFF;
    if ((idev->write(cmd, 6)) < 0)
        ipio_err("enable gesture fail reason failed");

    /* set gesture parameters */
    cmd[0] = 0x01;
    cmd[1] = 0x0A;
    cmd[2] = 0x12;
    cmd[3] = 0x01;
    memset(cmd + 4, 0xFF, 20);
    if ((idev->write(cmd, 24)) < 0)
        ipio_err("set gesture parameters failed");

    /* get gesture parameters */
    cmd[0] = 0x01;
    cmd[1] = 0x0A;
    cmd[2] = 0x11;
    cmd[3] = 0x01;
    if ((idev->write(cmd, 4)) < 0)
        ipio_err("get gesture parameters failed");

}

static void ilitek_resume_work_queue(struct work_struct *work)
{
	u8 tp_mode = P5_X_FW_DEMO_MODE;
	ilitek_tddi_switch_mode(&tp_mode);
/*	ilitek_plat_irq_enable();
	ipio_info("TP resume end\n");
	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);*/
	idev->suspend = false;

	return;


}


int ilitek_tddi_mp_test_handler(char *apk, bool lcm_on)
{
	int ret = 0;
	u8 tp_mode = P5_X_FW_TEST_MODE;

	if (atomic_read(&idev->fw_stat)) {
		ipio_err("fw upgrade processing, ignore\n");
		return 0;
	}

	if (!idev->chip->open_c_formula ||
		!idev->chip->open_sp_formula) {
		ipio_err("formula is null\n");
		return -1;
	}

	ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, DISABLE);
	mutex_lock(&idev->touch_mutex);
	atomic_set(&idev->mp_stat, ENABLE);

	if (idev->actual_tp_mode != P5_X_FW_TEST_MODE) {
		if (ilitek_tddi_switch_mode(&tp_mode) < 0)
			goto out;
	}

	ret = ilitek_tddi_mp_test_main(apk, lcm_on);

out:
	tp_mode = P5_X_FW_DEMO_MODE;
	ilitek_tddi_switch_mode(&tp_mode);

	atomic_set(&idev->mp_stat, DISABLE);
	mutex_unlock(&idev->touch_mutex);
	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
	return ret;
}

int ilitek_tddi_switch_mode(u8 *data)
{
	int ret = 0, mode;
	u8 cmd[4] = {0};

	if (!data) {
		ipio_err("data is null\n");
		return -EINVAL;
	}

	atomic_set(&idev->tp_sw_mode, START);

	mode = data[0];
	idev->actual_tp_mode = mode;

	switch (idev->actual_tp_mode) {
	case P5_X_FW_I2CUART_MODE:
		ipio_info("Not implemented yet\n");
		break;
	case P5_X_FW_DEMO_MODE:
		if (idev->fw_upgrade_mode == UPGRADE_IRAM)
			ilitek_tddi_fw_upgrade_handler(NULL);
		break;
	case P5_X_FW_DEBUG_MODE:
		cmd[0] = P5_X_MODE_CONTROL;
		cmd[1] = mode;
		ipio_info("Switch to Debug mode\n");
		ret = idev->write(cmd, 2);
		if (ret < 0)
			ipio_err("Failed to switch Debug mode\n");
		break;
	case P5_X_FW_GESTURE_MODE:
		ipio_info("Switch to Gesture mode, lpwg cmd = %d\n",  idev->gesture_mode);
		ret = ilitek_tddi_ic_func_ctrl("lpwg", idev->gesture_mode);
		break;
	case P5_X_FW_TEST_MODE:
		ipio_info("Switch to Test mode\n");
		ret = idev->mp_move_code();
		break;
	case P5_X_FW_DEMO_DEBUG_INFO_MODE:
		ipio_info("Switch to debug info mode\n");
		cmd[0] = P5_X_MODE_CONTROL;
		cmd[1] = mode;
		ret = idev->write(cmd, 2);
		if (ret < 0)
			ipio_err("Failed to switch debug info mode\n");
		break;
	case P5_X_FW_SOP_FLOW_MODE:
		ipio_info("Not implemented SOP flow mode yet\n");
		break;
	case P5_X_FW_ESD_MODE:
		ipio_info("Not implemented ESD mode yet\n");
		break;
	default:
		ipio_err("Unknown TP mode: %x\n", mode);
		ret = -1;
		break;
	}

	if (ret < 0)
		ipio_err("Switch mode failed\n");

	ipio_debug(DEBUG_MAIN, "Actual TP mode = %d\n", idev->actual_tp_mode);
	atomic_set(&idev->tp_sw_mode, END);
	return ret;
}

static void ilitek_tddi_wq_ges_recover(void)
{
	//mutex_lock(&idev->touch_mutex);
	atomic_set(&idev->esd_stat, START);
	idev->ges_recover();
	if(idev->gameSwitch) {
		ilitek_tddi_ic_func_ctrl("game_switch", DISABLE);
	}
	else {
		ilitek_tddi_ic_func_ctrl("game_switch", ENABLE);
	}
	schedule_work(&idev->headset_work_queue);
	schedule_work(&idev->usb_work_queue);
	atomic_set(&idev->esd_stat, END);
	//mutex_unlock(&idev->touch_mutex);
}

static void ilitek_tddi_wq_spi_recover(void)
{
	//ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
	//mutex_lock(&idev->touch_mutex);
	atomic_set(&idev->esd_stat, START);
    idev->actual_tp_mode = P5_X_FW_DEMO_MODE;
	ilitek_tddi_fw_upgrade_handler(NULL);
	if(idev->gameSwitch) {
		ilitek_tddi_ic_func_ctrl("game_switch", DISABLE);
	}
	else {
		ilitek_tddi_ic_func_ctrl("game_switch", ENABLE);
	}
	schedule_work(&idev->headset_work_queue);
	schedule_work(&idev->usb_work_queue);	
	atomic_set(&idev->esd_stat, END);
	//mutex_unlock(&idev->touch_mutex);
	//ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
}

int ilitek_tddi_wq_esd_spi_check(void)
{
	u8 tx = SPI_WRITE, rx = 0;
	mutex_lock(&idev->touch_mutex);
	idev->spi_write_then_read(idev->spi, &tx, 1, &rx, 1);
	mutex_unlock(&idev->touch_mutex);
	ipio_debug(DEBUG_MAIN, "spi esd check = 0x%x\n", rx);
	if (rx != SPI_ACK) {
		ipio_err("rx = 0x%x\n", rx);
		return -1;
	}
	return 0;
}

int ilitek_tddi_wq_esd_i2c_check(void)
{
	ipio_debug(DEBUG_MAIN, "");
	return 0;
}

static void ilitek_tddi_wq_esd_check(struct work_struct *work)
{
	if (idev->esd_recover() < 0) {
		ipio_err("SPI ACK failed, doing spi recovery\n");
		mutex_lock(&idev->touch_mutex);
		ilitek_tddi_wq_spi_recover();
		mutex_unlock(&idev->touch_mutex);
		ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
		return;
	}
	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
}

static int read_power_status(u8 *buf)
{
	struct file *f = NULL;
	mm_segment_t old_fs;
	ssize_t byte = 0;

	old_fs = get_fs();
	set_fs(get_ds());

	f = filp_open(POWER_STATUS_PATH, O_RDONLY, 0);
	if (ERR_ALLOC_MEM(f)) {
		ipio_err("Failed to open %s\n", POWER_STATUS_PATH);
		return -1;
	}

	f->f_op->llseek(f, 0, SEEK_SET);
	byte = f->f_op->read(f, buf, 20, &f->f_pos);

	ipio_debug(DEBUG_MAIN, "Read %d bytes\n", (int)byte);

	set_fs(old_fs);
	filp_close(f, NULL);
	return 0;
}

static void ilitek_tddi_wq_bat_check(struct work_struct *work)
{
	u8 str[20] = {0};
	static int charge_mode;

	read_power_status(str);
	ipio_debug(DEBUG_MAIN, "Batter Status: %s\n", str);

	if (strstr(str, "Charging") != NULL || strstr(str, "Full") != NULL
		|| strstr(str, "Fully charged") != NULL) {
		if (charge_mode != 1) {
			ipio_debug(DEBUG_MAIN, "Charging mode\n");
			ilitek_tddi_ic_func_ctrl("plug", DISABLE);// plug in
			charge_mode = 1;
		}
	} else {
		if (charge_mode != 2) {
			ipio_debug(DEBUG_MAIN, "Not charging mode\n");
			ilitek_tddi_ic_func_ctrl("plug", ENABLE);// plug out
			charge_mode = 2;
		}
	}
	ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
}

void ilitek_tddi_wq_ctrl(int type, int ctrl)
{
	switch (type) {
	case WQ_ESD:
		if (ENABLE_WQ_ESD) {
			if (!esd_wq) {
				ipio_err("wq esd is null\n");
				break;
			}
			idev->wq_esd_ctrl = ctrl;
			if (ctrl == ENABLE) {
				ipio_debug(DEBUG_MAIN, "execute esd check\n");
				if (!queue_delayed_work(esd_wq, &esd_work, msecs_to_jiffies(WQ_ESD_DELAY)))
					ipio_debug(DEBUG_MAIN, "esd check was already on queue\n");
			} else {
				cancel_delayed_work_sync(&esd_work);
				flush_workqueue(esd_wq);
				ipio_debug(DEBUG_MAIN, "cancel esd wq\n");
			}
		}
		break;
	case WQ_BAT:
		if (ENABLE_WQ_BAT) {
			if (!bat_wq) {
				ipio_err("WQ BAT is null\n");
				break;
			}
			idev->wq_bat_ctrl = ctrl;
			if (ctrl == ENABLE) {
				ipio_debug(DEBUG_MAIN, "execute bat check\n");
				if (!queue_delayed_work(bat_wq, &bat_work, msecs_to_jiffies(WQ_BAT_DELAY)))
					ipio_debug(DEBUG_MAIN, "bat check was already on queue\n");
			} else {
				cancel_delayed_work_sync(&bat_work);
				flush_workqueue(bat_wq);
				ipio_debug(DEBUG_MAIN, "cancel bat wq\n");
			}
		}
		break;
	case WQ_SPI_RECOVER:
		ipio_err("Not implement yet\n");
		break;
	case WQ_GES_RECOVER:
		ipio_err("Not implement yet\n");
		break;
	case WQ_SUSPEND:
		ipio_err("Not implement yet\n");
		break;
	default:
		ipio_err("Unknown WQ type, %d\n", type);
		break;
	}
}

static void ilitek_tddi_wq_init(void)
{
	esd_wq = alloc_workqueue("esd_check", WQ_MEM_RECLAIM, 0);
	bat_wq = alloc_workqueue("bat_check", WQ_MEM_RECLAIM, 0);

	WARN_ON(!esd_wq);
	WARN_ON(!bat_wq);

	INIT_DELAYED_WORK(&esd_work, ilitek_tddi_wq_esd_check);
	INIT_DELAYED_WORK(&bat_work, ilitek_tddi_wq_bat_check);

    INIT_WORK(&idev->resume_work_queue, ilitek_resume_work_queue);

}

int ilitek_tddi_sleep_handler(int mode)
{

	int ret = 0, i; //pwr;

	atomic_set(&idev->tp_sleep, START);

/*	if (atomic_read(&idev->fw_stat) ||
		atomic_read(&idev->mp_stat)) {
		ipio_info("fw upgrade or mp still running, ignore sleep requst\n");
		atomic_set(&idev->tp_sleep, END);
		mutex_unlock(&idev->touch_mutex);
		return 0;
	}*/

	ipio_info("Sleep Mode = %d\n", mode);
	ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, DISABLE);
	ilitek_plat_irq_disable();

	switch (mode) {
	case TP_SUSPEND:
		ipio_info("TP normal suspend start power_status =%d\n",idev->power_status);
		for (i = 0; i < 40; i++) {
			if (!atomic_read(&idev->fw_stat)) {
				ipio_err("fw not in upgrading\n");
				break;
			}
			msleep(5);
		}
		if( i>=39 ) {
			ipio_err("resume timeout\n");
		}
		mutex_lock(&idev->touch_mutex);	
		ilitek_tddi_ic_func_ctrl("sense", DISABLE);
		ilitek_tddi_ic_check_busy(5, 35);

		if (idev->gesture) {
			idev->gesture_move_code(idev->gesture_mode);
			enable_irq_wake(idev->irq_num);
			if(idev->psensor_close == true) {
				idev->gesture = false;
				tp_gesture = 0;
				ilitek_tddi_ic_func_ctrl("sleep", SLEEP_IN);
				ipio_info("suspend set sleep ctrl redo\n");
			}
			idev->gesture_done =true;
			ilitek_plat_irq_enable();
		} else {
			ilitek_tddi_ic_func_ctrl("sleep", DEEP_SLEEP_IN);
			mdelay(50);
			idev->gesture_done =false;
//			pwr = power_on(idev,false);
//			idev->power_status = false;
//			if(pwr != 0) {
//				ipio_info("power off fail\n");
//			}
			ipio_info("power_status = %d\n",idev->power_status);
		}
        idev->suspend = true;
		mutex_unlock(&idev->touch_mutex);
		ipio_info("TP normal suspend end 20190505\n");
		break;
	case TP_DEEP_SLEEP:
		ipio_info("TP deep suspend start\n");
		idev->suspend = true;
		ilitek_tddi_ic_func_ctrl("sense", DISABLE);
		ilitek_tddi_ic_check_busy(50, 50);
		ilitek_tddi_ic_func_ctrl("sleep", DEEP_SLEEP_IN);
//		pwr = power_on(idev,false);
//		if(pwr != 0) {
//			ipio_info("power off fail\n");
//		}
		ipio_info("TP deep suspend end\n");
		break;
	case TP_RESUME:
		ipio_info("TP resume start\n");
		idev->suspend = false;
		idev->gesture_done =false;
		idev->psensor_close =false;
		/*
		if (idev->gesture)
		{
			disable_irq_wake(idev->irq_num);
		}
		else
		{
			pwr = power_on(idev,true);
			if(pwr != 0) {
				ipio_info("power off fail\n");
			}
		}

		ilitek_tddi_switch_mode(&tp_mode);
		ilitek_plat_irq_enable();
		ipio_info("TP resume end\n");
		ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
		ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
		*/
		ilitek_plat_irq_enable();
		//ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
		//ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
		schedule_work(&idev->headset_work_queue);
		schedule_work(&idev->usb_work_queue);
		break;
	default:
		ipio_err("Unknown sleep mode, %d\n", mode);
		ret = -EINVAL;
		break;
	}

	ilitek_tddi_touch_release_all_point();
	atomic_set(&idev->tp_sleep, END);
	return ret;
}

int ilitek_tddi_fw_upgrade_handler(void *data)
{
	int ret = 0;
	bool get_lock = false;

	atomic_set(&idev->fw_stat, START);

	if (!atomic_read(&idev->tp_sleep) &&
		!atomic_read(&idev->mp_stat) &&
		!atomic_read(&idev->esd_stat)) {
		ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
		ilitek_tddi_wq_ctrl(WQ_BAT, DISABLE);
	}

	if (!mutex_is_locked(&idev->touch_mutex)) {
		mutex_lock(&idev->touch_mutex);
		get_lock = true;
		ipio_info("get touch lock\n");
	}

	idev->fw_update_stat = 0;
	ret = ilitek_tddi_fw_upgrade(idev->fw_upgrade_mode, HEX_FILE, idev->fw_open);
	if (ret != 0)
		idev->fw_update_stat = -1;
	else
		idev->fw_update_stat = 100;



	if (!atomic_read(&idev->tp_sleep) &&
		!atomic_read(&idev->mp_stat) &&
		!atomic_read(&idev->esd_stat)) {
		ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
		ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
	}

	atomic_set(&idev->fw_stat, END);
	if (get_lock)
		mutex_unlock(&idev->touch_mutex);
	return ret;
}

void ilitek_tddi_report_handler(void)
{
	int ret = 0, pid = 0;
	u8 *buf = NULL, checksum = 0;
	int rlen = 0;
	//u16 self_key = 2;
	int tmp = ipio_debug_level;
	int i = 0;
	int res;
	/* Just in case these stats couldn't be blocked in top half context */
	if (!idev->report || atomic_read(&idev->tp_reset) ||
		atomic_read(&idev->fw_stat) || atomic_read(&idev->tp_sw_mode) ||
		atomic_read(&idev->mp_stat) || atomic_read(&idev->tp_sleep)) {
		ipio_info("ignore report request\n");
		return;
	}
#if 0
	switch (idev->actual_tp_mode) {
	case P5_X_FW_DEMO_MODE:
		rlen = P5_X_DEMO_MODE_PACKET_LENGTH;
		break;
	case P5_X_FW_DEBUG_MODE:
		rlen = (2 * idev->xch_num * idev->ych_num) + (idev->stx * 2) + (idev->srx * 2);
		rlen += 2 * self_key + (8 * 2) + 1 + 35;
		break;
	case P5_X_FW_I2CUART_MODE:
		rlen = P5_X_DEMO_MODE_PACKET_LENGTH;
		break;
	case P5_X_FW_GESTURE_MODE:
		if (idev->gesture_mode == P5_X_FW_GESTURE_INFO_MODE)
			rlen = P5_X_GESTURE_INFO_LENGTH;
		else
			rlen = P5_X_GESTURE_NORMAL_LENGTH;
		break;
	case P5_X_FW_DEMO_DEBUG_INFO_MODE:
		/*only suport SPI interface now, so defult use size 1024 buffer*/
		rlen = 1024;
		break;
	default:
		ipio_err("Unknown fw mode, %d\n", idev->actual_tp_mode);
		rlen = 0;
		break;
	}
#endif
    /* odm_wt add 20191204 */
	if (idev->actual_tp_mode == P5_X_FW_GESTURE_MODE) {
		__pm_stay_awake(idev->ws);
		/* Waiting for pm resume completed */
		mdelay(40);
	}
	rlen = core_spi_check_read_size();

	ipio_debug(DEBUG_MAIN, "Packget length = %d\n", rlen);

	if ((rlen <= 0)||(rlen > 2048)) {
		//ipio_err("Length of packet is invaild rlen = %d\n",rlen);
			//ipio_err("Read report packet failed\n");
			if (idev->actual_tp_mode == P5_X_FW_GESTURE_MODE && idev->gesture && rlen == DO_SPI_RECOVER) {
				ipio_err("Gesture failed, doing gesture recovery\n");
				ilitek_tddi_wq_ges_recover();
				goto recover;
			} else if (rlen == DO_SPI_RECOVER) {
				ipio_err("SPI ACK failed, doing spi recovery\n");
				if (idev->dis_esd_recovery){
					goto out;
				}
				ilitek_tddi_wq_spi_recover();
				goto recover;
			}
			goto out;
	}
		buf = kcalloc(rlen, sizeof(u8), GFP_ATOMIC);
		if (ERR_ALLOC_MEM(buf)) {
			ipio_err("Failed to allocate packet memory, %ld\n", PTR_ERR(buf));
			return;
		}
		ret = core_spi_read_data_after_checksize(buf, rlen);
#if 0
		ret = idev->read(buf, rlen);
#endif

	ipio_debug(DEBUG_MAIN, "Read data fail = %d\n", (ret));

	//rlen = ret;

	ilitek_dump_data(buf, 8, rlen, 0, "finger report");

	checksum = ilitek_calc_packet_checksum(buf, rlen - 1);
	if (checksum != buf[rlen-1] && idev->fw_uart_en == DISABLE) {
		ipio_err("Wrong checksum, checksum = %x, buf = %x\n", checksum, buf[rlen-1]);
		ipio_debug_level = DEBUG_ALL;
		ilitek_dump_data(buf, 8, rlen, 0, "finger report");
		ipio_debug_level = tmp;
		goto out;
	}

	pid = buf[0];
	ipio_debug(DEBUG_MAIN, "Packet ID = %x\n", pid);

	switch (pid) {
	case P5_X_DEMO_PACKET_ID:
		ilitek_tddi_report_ap_mode(buf, rlen);
		break;
	case P5_X_DEBUG_PACKET_ID:
		ilitek_tddi_report_debug_mode(buf, rlen);
		break;
	case P5_X_I2CUART_PACKET_ID:
		ilitek_tddi_report_i2cuart_mode(buf, rlen);
		break;
	case P5_X_GESTURE_FAIL_ID:
		 ilitek_dump_data(buf, 8, rlen, 0, "fail reason");
		 break;
	case P5_X_GESTURE_PACKET_ID:
		if(rlen != GESTURE_INFO_LENGTH ) {
			ipio_err("gesture lens is not correct");
			break;
		}
		for(i=0;i<rlen;i++)
			{
				buf_gesture[i] = buf[i];
			}
		
		res = ilitek_get_gesture_info(gesture_report_data);
		if(res)
			{
			input_report_key(idev->input, KEY_F4, 1);
			input_sync(idev->input);
			input_report_key(idev->input, KEY_F4, 0);
			input_sync(idev->input);
			}
		break;
	default:
		ipio_err("Unknown packet id, %x\n", pid);
		break;
	}

out:

recover:
    /* odm_wt add 20191204 */
	if (idev->actual_tp_mode == P5_X_FW_GESTURE_MODE)
		__pm_relax(idev->ws);
	ipio_kfree((void **)&buf);
}

int ilitek_tddi_reset_ctrl(int mode)
{
	int ret = 0;

	atomic_set(&idev->tp_reset, START);

	if (mode != TP_IC_CODE_RST)
		ilitek_tddi_ic_check_otp_prog_mode();

	switch (mode) {
	case TP_IC_CODE_RST:
		ipio_info("TP IC Code RST \n");
		ret = ilitek_tddi_ic_code_reset();
		break;
	case TP_IC_WHOLE_RST:
		ipio_info("TP IC whole RST\n");
		ret = ilitek_tddi_ic_whole_reset();
		break;
	case TP_HW_RST_ONLY:
		ipio_info("TP HW RST\n");
		ilitek_plat_tp_reset();
		break;
	default:
		ipio_err("Unknown reset mode, %d\n", mode);
		ret = -EINVAL;
		break;
	}

	/*
	 * Since OTP must be folloing with reset, except for code rest,
	 * the stat of ice mode should be set as 0.
	 */
	if (mode != TP_IC_CODE_RST)
		atomic_set(&idev->ice_stat, DISABLE);
	idev->fw_uart_en = DISABLE;
	atomic_set(&idev->tp_reset, END);
	return ret;
}

int ilitek_tddi_init(void)
{
	struct task_struct *fw_boot_th;

	ipio_info("ilitek tddi main init\n");

	mutex_init(&idev->io_mutex);
	mutex_init(&idev->touch_mutex);
	mutex_init(&idev->debug_mutex);
	mutex_init(&idev->debug_read_mutex);
	init_waitqueue_head(&(idev->inq));
	spin_lock_init(&idev->irq_spin);

	atomic_set(&idev->irq_stat, DISABLE);
	atomic_set(&idev->ice_stat, DISABLE);
	atomic_set(&idev->tp_reset, END);
	atomic_set(&idev->fw_stat, END);
	atomic_set(&idev->mp_stat, DISABLE);
	atomic_set(&idev->tp_sleep, END);
	atomic_set(&idev->mp_int_check, DISABLE);
	atomic_set(&idev->esd_stat, END);
//	reload_fw_ws = wakeup_source_register("wrap_wake_lock");

	ilitek_tddi_ic_init();
	ilitek_tddi_wq_init();

	if (idev->reset == TP_IC_WHOLE_RST)
		idev->do_otp_check = ENABLE;
	idev->fw_uart_en = DISABLE;
	/* Must do hw reset once in first time for work normally if tp reset is avaliable */
	if (!TDDI_RST_BIND)
		ilitek_tddi_reset_ctrl(idev->reset);

	idev->do_otp_check = ENABLE;

	if (ilitek_tddi_ic_get_info() < 0) {
		ipio_err("Not found ilitek chipes\n");
		return -ENODEV;
	}
	oplus_proc_init();
	ilitek_tddi_node_init();
	ilitek_tddi_fw_read_flash_info(idev->fw_upgrade_mode);
	
	ilitek_plat_input_register();
	fw_boot_th = kthread_run(ilitek_tddi_fw_upgrade_handler, NULL, "ili_fw_boot");
	if (fw_boot_th == (struct task_struct *)ERR_PTR) {
		fw_boot_th = NULL;
		WARN_ON(!fw_boot_th);
		ipio_err("Failed to create fw upgrade thread\n");
	}

    /*odm_wt add 20191204*/
	idev->ws = wakeup_source_register(NULL, "ili_wakelock");
	if (!idev->ws)
		ipio_err("wakeup source request failed\n");
	
	return 0;
}

void ilitek_tddi_dev_remove(void)
{
	ipio_info("remove ilitek dev\n");

	if (!idev)
		return;

	gpio_free(idev->tp_int);
	gpio_free(idev->tp_rst);

	if (esd_wq != NULL) {
		cancel_delayed_work_sync(&esd_work);
		flush_workqueue(esd_wq);
		destroy_workqueue(esd_wq);
	}
	if (bat_wq != NULL) {
		cancel_delayed_work_sync(&bat_work);
		flush_workqueue(bat_wq);
		destroy_workqueue(bat_wq);
	}

    /* odm_wt add 20191204 */
	idev->ws = wakeup_source_register(NULL, "ili_wakelock");
	if (!idev->ws)
		ipio_err("wakeup source request failed\n");
}

int ilitek_tddi_dev_init(struct ilitek_hwif_info *hwif)
{
	ipio_info("TP Interface: %s\n", (hwif->bus_type == BUS_I2C) ? "I2C" : "SPI");
	return ilitek_tddi_interface_dev_init(hwif);
}
