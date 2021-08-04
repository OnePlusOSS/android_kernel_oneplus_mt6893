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
bool mp_test_result = false;
static uint32_t temp[5] = {0};
extern struct ili_gesture_info * gesture_report_data;
extern unsigned char g_user_buf[PAGE_SIZE];
#ifdef ODM_WT_EDIT
int sign_firmware = 0;
#endif
static ssize_t wt_mptest_read(struct file *filp, char __user *buff, size_t size, loff_t *pos)
{
	int ret = 0;
	int len = 0;
	size_t count = 0;
	char apk_ret[100] = {0};
	char *ptr = NULL;
	char result_pass[20] = "result=1";
	char result_fail[20] = "result=0";
	ipio_info("Run MP test with LCM on\n");

	if (*pos != 0)
		return 0;
	ptr = (char*)kzalloc(256,GFP_KERNEL);
	if (ptr == NULL) {
	ipio_err("failed to alloc ptr memory\n");
		return 0;
	}
	if(idev->suspend == true) {
		ipio_info("%s,not in resume,can not to do mp test\n",__func__);
		count = ARRAY_SIZE(result_pass);
		len = snprintf(ptr, count,"%s\n",result_pass);
		ret = copy_to_user(buff,ptr,len);
		*pos += len;
		kfree(ptr);
		return len;
	}
	/* Create the directory for mp_test result */
	ret = dev_mkdir(CSV_LCM_ON_PATH, 0766);
	if (ret != 0)
		ipio_err("Failed to create directory for mp_test\n");
	mp_test_result = true;
	ilitek_tddi_mp_test_handler(apk_ret, ON);
	if(mp_test_result) {
		ipio_debug(DEBUG_MP, "wt mp test pass\n");
		count = ARRAY_SIZE(result_pass);
		len = snprintf(ptr, count,"%s\n",result_pass);
		ret = copy_to_user(buff,ptr,len);
		*pos += len;
	}
	else {
		ipio_debug(DEBUG_MP, "wt mp test fail\n");
		count = ARRAY_SIZE(result_fail);
		len = snprintf(ptr, count,"%s\n",result_fail);
		ret = copy_to_user(buff,ptr,len);
		*pos += len;
	}
	return len;

}
static ssize_t oplus_proc_mp_lcm_on_read(struct file *filp, char __user *buff, size_t size, loff_t *pos)
{
	int ret = 0;
	int len = 0;
	size_t count = 0;
	char apk_ret[100] = {0};
	char *ptr = NULL;
	char result_pass[40] = "0 error(s),All test passed";
	char result_fail[20] = "MP test fail";
	ipio_info("Run MP test with LCM on\n");

	if (*pos != 0)
		return 0;
	ptr = (char*)kzalloc(256,GFP_KERNEL);
	if (ptr == NULL) {
		ipio_err("failed to alloc ptr memory\n");
		return 0;
	}
	if(idev->suspend == true) {
		ipio_info("%s,not in resume,can not to do mp test\n",__func__);
		count = ARRAY_SIZE(result_pass);
		len = snprintf(ptr, count,"%s\n",result_pass);
		ret = copy_to_user(buff,ptr,len);
		*pos += len;
		kfree(ptr);
		return len;
	}
	/* Create the directory for mp_test result */
	ret = dev_mkdir(CSV_LCM_ON_PATH, 0766);
	if (ret != 0)
		ipio_err("Failed to create directory for mp_test\n");
	mp_test_result = true;
	ilitek_tddi_mp_test_handler(apk_ret, ON);
	if(mp_test_result) {
		ipio_debug(DEBUG_MP, "oplus lcm on mp test pass\n");
		count = ARRAY_SIZE(result_pass);
		len = snprintf(ptr, count,"%s\n",result_pass);
		ret = copy_to_user(buff,ptr,len);
		*pos += len;
	}
	else {
		ipio_debug(DEBUG_MP, "oplus lcm on mp test fail\n");
		count = ARRAY_SIZE(result_fail);
		len = snprintf(ptr, count,"%s\n",result_fail);
		ret = copy_to_user(buff,ptr,len);
		*pos += len;
	}
	return len;

}
static ssize_t oplus_proc_black_screen_test_read(struct file *filp, char __user *buff, size_t size, loff_t *pos)
{
	int ret = 0;
	int len = 0;
	size_t count = 0;
	char apk_ret[100] = {0};
	char *ptr = NULL;
	char result_pass[40] = "0 error(s),All test passed";
	char result_fail[20] = "MP test fail";
	ipio_info("Run MP test with LCM off\n");

	if (*pos != 0)
		return 0;
	ptr = (char*)kzalloc(256,GFP_KERNEL);
	if (ptr == NULL) {
		ipio_err("failed to alloc ptr memory\n");
		return 0;
	}
	if(idev->suspend == false) {
		ipio_info("not in suspend,can not to do black mp test\n");
		len = snprintf(ptr, 100,"please sleep in\n");
		ret = copy_to_user(buff,ptr,len);
		*pos += len;
		kfree(ptr);
		return len;
	}
	if(idev->gesture== false) {
		ipio_info("gesture is off,can not to do black mp test\n");
		len = snprintf(ptr, 100,"please open gesture mode\n");
		ret = copy_to_user(buff,ptr,len);
		*pos += len;
		kfree(ptr);
		return len;
	}
	
	/* Create the directory for mp_test result */
	ret = dev_mkdir(CSV_LCM_OFF_PATH,0766);
	if (ret != 0)
		ipio_err("Failed to create directory for mp_test\n");
	mp_test_result = true;
	ilitek_tddi_mp_test_handler(apk_ret, OFF);
	if(mp_test_result) {
		ipio_debug(DEBUG_MP, "oplus lcm off mp test pass\n");
		count = ARRAY_SIZE(result_pass);
		len = snprintf(ptr, count,"%s\n",result_pass);
		ret = copy_to_user(buff,ptr,len);
		*pos += len;
	}
	else {
		ipio_debug(DEBUG_MP, "oplus lcm off mp test fail\n");
		count = ARRAY_SIZE(result_fail);
		len = snprintf(ptr, count,"%s\n",result_fail);
		ret = copy_to_user(buff,ptr,len);
		*pos += len;
	}
	return len;

}

static ssize_t oplus_proc_black_screen_test_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int value = 0;
	char buf[4] = {0};
	if ( copy_from_user(buf, buffer, count) ) {
		printk("%s: copy from user error.", __func__);
		count = -1;
		goto OUT;
	}
	sscanf(buf, "%d", &value);

	idev->gesture_backup = idev->gesture;
    idev->gesture = true;
	tp_gesture = 1;
OUT:
    return count;
}

static ssize_t oplus_proc_game_switch_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	int len = 0;
	char *ptr = NULL;

	if ( *ppos ) {
	    printk("is already read the file\n");
        return 0;
	}
    *ppos += count;

    ptr = kzalloc(count,GFP_KERNEL);
	if(ptr == NULL){
		printk("allocate memory fail\n");
		return -1;
	}

    len = snprintf(ptr, count,"%x\n",idev->gameSwitch);

    ret = copy_to_user(buf,ptr,len);

	kfree(ptr);
	return len;
}
static ssize_t oplus_proc_game_switch_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	char *ptr = NULL;
	ptr = kzalloc(size,GFP_KERNEL);
	if (ptr == NULL) {
		ipio_err("allocate the memory fail\n");
		return -1;
	}
	res = copy_from_user(ptr, buff, size);
	if (res) {
		ipio_err("copy data from user space, failed\n");
		size = -1;
		goto OUT;
	}
	//ipio_info("size = %d, cmd = %s data %d\n", (int)size, cmd[0],data);
	mutex_lock(&idev->touch_mutex);
	if (ptr[0] == '0') {
		ipio_info("disable game play mode\n");
		idev->gameSwitch = 0;
		ilitek_tddi_ic_func_ctrl("game_switch", DISABLE);
	} else if (!ptr[0]) {
		ipio_err("Unknown command\n");
	} else {
		ipio_info("enable game play mode\n");
		idev->gameSwitch= 1;
		ilitek_tddi_ic_func_ctrl("game_switch", ENABLE);

	}
	mutex_unlock(&idev->touch_mutex);
OUT:
	kfree(ptr);
	return size;
}

static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void c_stop(struct seq_file *m, void *v)
{
	return;
}

static int32_t c_oplus_ili_coordinate_show(struct seq_file *m, void *v)
{
	struct ili_gesture_info *gesture = gesture_report_data;
	char tmp[256] = {0};
	printk("c_oplus_coordinate_show\n");
	sprintf(tmp, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d",
		gesture->gesture_type,
		gesture->Point_start.x, gesture->Point_start.y,
		gesture->Point_end.x, gesture->Point_end.y,
		gesture->Point_1st.x, gesture->Point_1st.y,
		gesture->Point_2nd.x, gesture->Point_2nd.y,
		gesture->Point_3rd.x, gesture->Point_3rd.y,
		gesture->Point_4th.x, gesture->Point_4th.y,
		gesture->clockwise);

	/* oplus gesture formate */
	seq_printf(m, "%s\n", tmp);

	return 0;
}
static ssize_t ilitek_proc_CDC_delta_read(struct file *pFile, char __user *buf, size_t size, loff_t *pos)
	{
		s16 *delta = NULL;
		int row = 0, col = 0,  index = 0;
		int ret, i, x, y;
		int read_length = 0;
		u8 cmd[2] = {0};
		u8 *data = NULL;	
		struct irq_desc *desc = irq_to_desc(idev->irq_num);
		if (*pos != 0)
			return 0;
	
		memset(g_user_buf, 0, PAGE_SIZE * sizeof(unsigned char));
		if(idev->suspend == 1){
			size = snprintf(g_user_buf, PAGE_SIZE, "======== get Delta in suspend fail========\n");
			ret = copy_to_user(buf, g_user_buf, size);
			if (ret < 0) {
				ipio_err("Failed to copy data to user space");
			}
			return ret;
		}
		ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
		ilitek_tddi_wq_ctrl(WQ_BAT, DISABLE);
		mutex_lock(&idev->touch_mutex);

		row = idev->ych_num;
		col = idev->xch_num;
		read_length = 4 + 2 * row * col + 1 ;

		ipio_info("read length = %d\n", read_length);

		data = kcalloc(read_length + 1, sizeof(u8), GFP_KERNEL);
		if (ERR_ALLOC_MEM(data)) {
			ipio_err("Failed to allocate data mem\n");
			size = -1;
			goto out;
		}

		delta = kcalloc(P5_X_DEBUG_MODE_PACKET_LENGTH, sizeof(s32), GFP_KERNEL);
		if (ERR_ALLOC_MEM(delta)) {
			ipio_err("Failed to allocate delta mem\n");
			size = -1;
			goto out;
		}
		msleep(100);
		cmd[0] = 0xB7;
		cmd[1] = 0x1; //get delta
		ret = idev->write(cmd, sizeof(cmd));
		if (ret < 0) {
			ipio_err("Failed to write 0xB7,0x1 command, %d\n", ret);
			size = -1;
			goto out;
		}

		msleep(20);

		/* read debug packet header */
		ret = idev->read(data, read_length);

		cmd[1] = 0x03; //switch to normal mode
		ret = idev->write(cmd, sizeof(cmd));
		if (ret < 0) {
			ipio_err("Failed to write 0xB7,0x3 command, %d\n", ret);
			size = -1;
			goto out;
		}

		for (i = 4, index = 0; index < row * col * 2; i += 2, index++)
			delta[index] = (data[i] << 8) + data[i + 1];

		size = snprintf(g_user_buf + size, PAGE_SIZE - size, "======== Deltadata ========\n");
		size += snprintf(g_user_buf + size, PAGE_SIZE - size, "now depth= %d\n",desc->depth);		
		ipio_info("======== Deltadata ========\n");

		size += snprintf(g_user_buf + size, PAGE_SIZE - size,
			"Header 0x%x ,Type %d, Length %d\n", data[0], data[1], (data[2] << 8) | data[3]);
		ipio_info("Header 0x%x ,Type %d, Length %d\n", data[0], data[1], (data[2] << 8) | data[3]);

		// print delta data
		for (y = 0; y < row; y++) {
			size += snprintf(g_user_buf + size, PAGE_SIZE - size, "[%2d] ", (y+1));
			ipio_info("[%2d] ", (y+1));

			for (x = 0; x < col; x++) {
				int shift = y * col + x;
				size += snprintf(g_user_buf + size, PAGE_SIZE - size, "%5d", delta[shift]);
				printk(KERN_CONT "%5d", delta[shift]);
			}
			size += snprintf(g_user_buf + size, PAGE_SIZE - size, "\n");
			printk(KERN_CONT "\n");
		}

		ret = copy_to_user(buf, g_user_buf, size);
		if (ret < 0) {
			ipio_err("Failed to copy data to user space");
		}

		*pos += size;

	out:
		mutex_unlock(&idev->touch_mutex);
		ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
		ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
		ipio_kfree((void **)&data);
		ipio_kfree((void **)&delta);
		return size;
	}

static ssize_t ilitek_proc_rawdata_read(struct file *pFile, char __user *buf, size_t size, loff_t *pos)
	{
		s16 *rawdata = NULL;
		int row = 0, col = 0,  index = 0;
		int ret, i, x, y;
		int read_length = 0;
		u8 cmd[2] = {0};
		u8 *data = NULL;
		struct irq_desc *desc = irq_to_desc(idev->irq_num);
		if (*pos != 0)
			return 0;

		memset(g_user_buf, 0, PAGE_SIZE * sizeof(unsigned char));

		if(idev->suspend == 1){
			size = snprintf(g_user_buf, PAGE_SIZE, "======== get RawData in suspend fail========\n");
			ret = copy_to_user(buf, g_user_buf, size);
			if (ret < 0) {
				ipio_err("Failed to copy data to user space");
			}
			return ret;
		}

		ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
		ilitek_tddi_wq_ctrl(WQ_BAT, DISABLE);
		mutex_lock(&idev->touch_mutex);

		row = idev->ych_num;
		col = idev->xch_num;
		read_length = 4 + 2 * row * col + 1 ;

		ipio_info("read length = %d\n", read_length);

		data = kcalloc(read_length + 1, sizeof(u8), GFP_KERNEL);
		if (ERR_ALLOC_MEM(data)) {
				ipio_err("Failed to allocate data mem\n");
				size = -1;
				goto out;
		}

		rawdata = kcalloc(P5_X_DEBUG_MODE_PACKET_LENGTH, sizeof(s32), GFP_KERNEL);
		if (ERR_ALLOC_MEM(rawdata)) {
				ipio_err("Failed to allocate rawdata mem\n");
				size = -1;
				goto out;
		}
		msleep(100);
		cmd[0] = 0xB7;
		cmd[1] = 0x2; //get rawdata
		ret = idev->write(cmd, sizeof(cmd));
		if (ret < 0) {
			ipio_err("Failed to write 0xB7,0x2 command, %d\n", ret);
			size = -1;
			goto out;
		}

		msleep(20);

		/* read debug packet header */
		ret = idev->read(data, read_length);

		cmd[1] = 0x03; //switch to normal mode
		ret = idev->write(cmd, sizeof(cmd));
		if (ret < 0) {
			ipio_err("Failed to write 0xB7,0x3 command, %d\n", ret);
			size = -1;
			goto out;
		}

		for (i = 4, index = 0; index < row * col * 2; i += 2, index++)
			rawdata[index] = (data[i] << 8) + data[i + 1];

		size = snprintf(g_user_buf, PAGE_SIZE, "======== RawData ========\n");
		size += snprintf(g_user_buf + size, PAGE_SIZE - size, "now depth= %d\n",desc->depth);		
		ipio_info("======== RawData ========\n");

		size += snprintf(g_user_buf + size, PAGE_SIZE - size,
				"Header 0x%x ,Type %d, Length %d\n", data[0], data[1], (data[2] << 8) | data[3]);
		ipio_info("Header 0x%x ,Type %d, Length %d\n", data[0], data[1], (data[2] << 8) | data[3]);

		// print raw data
		for (y = 0; y < row; y++) {
			size += snprintf(g_user_buf + size, PAGE_SIZE - size, "[%2d] ", (y+1));
			ipio_info("[%2d] ", (y+1));

			for (x = 0; x < col; x++) {
				int shift = y * col + x;
				size += snprintf(g_user_buf + size, PAGE_SIZE - size, "%5d", rawdata[shift]);
				printk(KERN_CONT "%5d", rawdata[shift]);
			}
			size += snprintf(g_user_buf + size, PAGE_SIZE - size, "\n");
			printk(KERN_CONT "\n");
		}

		ret = copy_to_user(buf, g_user_buf, size);
		if (ret < 0) {
			ipio_err("Failed to copy data to user space");
		}

		*pos += size;

	out:
		mutex_unlock(&idev->touch_mutex);
		ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
		ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
		ipio_kfree((void **)&data);
		ipio_kfree((void **)&rawdata);
		return size;
	}



static ssize_t ilitek_proc_main_register_read(struct file *filp, char __user *buff, size_t count, loff_t *pPos)
{
	int ret = 0;
	int len = 0;
	char *ptr = NULL;
	if (*pPos != 0)
		return 0;	
	ptr = (char*)kzalloc(count,GFP_KERNEL);
	len += snprintf(ptr+len, count-len,"0x%08x\n",idev->chip->fw_ver);
	ret = copy_to_user(buff,ptr,len);
	*pPos += len;	
	return len;
}

static ssize_t ilitek_proc_oplus_debug_level_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	char cmd[10] = { 0 };

	if (buff != NULL) {
		res = copy_from_user(cmd, buff, size - 1);
		if (res < 0) {
			ipio_info("copy data from user space, failed\n");
			return -1;
		}
	}

	oplus_debug_level = katoi(cmd);

	ipio_info("oplus_debug_level = %d\n", oplus_debug_level);

	return size;
}
static ssize_t ilitek_proc_read_write_register_read(struct file *pFile, char __user *buf, size_t nCount, loff_t *pos)
{
	int ret = 0;
	uint32_t type, addr, read_data, write_data, write_len,read_len;
	char *ptr = NULL;
	int i;
	int len = 0;
	struct file *f = NULL;
	mm_segment_t fs;
	loff_t Ppos;
	if (*pos != 0)
		return 0;
	type = temp[0];
	addr = temp[1];
	write_data = temp[2];
	write_len = temp[3];
	read_len = temp[4];
	ptr = (char*)kzalloc(nCount,GFP_KERNEL);
	if (ERR_ALLOC_MEM(ptr)) {
		ipio_err("Failed to allocate ptr mem, %ld\n", PTR_ERR(ptr));
		return -ENOMEM;
	}
	
	ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, DISABLE);
	mutex_lock(&idev->touch_mutex);
	ipio_info("read type = %d\n", temp[0]);
			ret = ilitek_ice_mode_ctrl(ENABLE, OFF);
		if (ret != 0) {
			ipio_err("Failed to enter ICE mode, ret = %d\n", ret);
			len += snprintf(ptr + len, nCount - len, "fail enter ice mode\n");
			mutex_unlock(&idev->touch_mutex);
			ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
			ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);			
			return ret;
		}

	if (type == 0) {
		len += snprintf(ptr + len, nCount - len, "READ:addr = 0x%06x, read len = 0x%08x\n\n", addr,read_len);
		for(i=0;i<read_len;i++) {
			read_data = ilitek_ice_mode_read((addr+i*4), sizeof(u32));
			ipio_info("READ:addr = 0x%06x, read = 0x%08x\n", addr+i*4, read_data);
			len += snprintf(ptr+len, nCount - len, "READ:addr = 0x%06x, read = 0x%08x\n", addr+i*4, read_data);
		}
		f = filp_open("/sdcard/register_data.txt", O_WRONLY | O_CREAT | O_TRUNC, 644);		
		if (ERR_ALLOC_MEM(f)) {
			ipio_err("Failed to open register file");
			goto out;
		}
		
		fs = get_fs();
		set_fs(KERNEL_DS);
		Ppos = 0;
		vfs_write(f, ptr, len, &Ppos);
		set_fs(fs);
		filp_close(f, NULL);
	} else {
		ilitek_ice_mode_write(addr, write_data, write_len);
		ipio_info("WRITE:addr = 0x%06x, write = 0x%08x, len =%d byte\n", addr, write_data, write_len);
		len += snprintf(ptr + len, nCount - len, "WRITE:addr = 0x%06x, write = 0x%08x, len =%d byte\n", addr, write_data, write_len);
	}
	out:	
	ret = copy_to_user(buf, ptr, len);
	if (ret < 0) {
		ipio_err("Failed to copy data to user space");
	}
	ret = ilitek_ice_mode_ctrl(DISABLE, OFF);	
	mutex_unlock(&idev->touch_mutex);
	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);

	*pos += len;
	kfree(ptr);
	return len;

}

static ssize_t ilitek_proc_read_write_register_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int ret = 0;
	char *token = NULL, *cur = NULL;
	char cmd[256] = { 0 };
	uint32_t count = 0;

	if (buff != NULL) {
		ret = copy_from_user(cmd, buff, size - 1);
		if (ret < 0) {
			ipio_info("copy data from user space, failed\n");
			return -1;
		}
	}

	token = cur = cmd;

	while ((token = strsep(&cur, ",")) != NULL) {
		temp[count] = str2hex(token);
		ipio_info("data[%d] = 0x%x\n", count, temp[count]);
		count++;
		if(count > 4) {
			break;
		}
	}

	return size;
}

#ifdef ODM_WT_EDIT
/*node to control if upload fireware with sign bin
*default value is ture
*when echo 0 to this node ,upload fireware with unsign bin
*/
static ssize_t ilitek_proc_sign_firmware_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	char cmd[10] = { 0 };
	if (*pPos != 0)
		return 0;
	if (buff != NULL) {
		res = copy_from_user(cmd, buff, size - 1);
		if (res < 0) {
			ipio_err("copy data from user space, failed\n");
			return -1;
		}
	}
	*pPos += size;
	ipio_info("size = %d, cmd = %d\n", (int)size, cmd[0]);
	if (strcmp(cmd, "0") == 0) {
		sign_firmware = 0;
		idev->need_request_fw = true;
	}else{
		ipio_info("ilitek_proc_sign_firmware_write %d useless value\n",sign_firmware);
	}
	return size;
}
#endif

/*
*0：触发版本号不同固件升级
*1：触发固件强制升级
*2：触发带签名固件升级
*/
static ssize_t ilitek_proc_oplus_upgrade_fw_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	char cmd[10] = { 0 };
	u8 tp_mode = P5_X_FW_DEMO_MODE;	
	if (*pPos != 0)
		return 0;

	if (buff != NULL) {
		res = copy_from_user(cmd, buff, size);
		if (res < 0) {
			ipio_err("copy data from user space, failed\n");
			return -1;
		}
	}
	*pPos += size;

	ipio_info("size = %d, cmd = %d\n", (int)size, cmd[0]);
	ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, DISABLE);
	ilitek_plat_irq_disable();
	mutex_lock(&idev->touch_mutex);

#ifdef ODM_WT_EDIT
	if ((strncmp(cmd, "0", 1) == 0)||(strncmp(cmd, "1", 1) == 0)) {
		sign_firmware = 0;
		idev->need_request_fw = true;
		ilitek_tddi_switch_mode(&tp_mode);
	}else if(strncmp(cmd, "2", 1) == 0){
		ipio_info("goto sign bin firmware\n");
		sign_firmware = 1;
		idev->need_request_fw = true;
		ilitek_tddi_switch_mode(&tp_mode);
	}else{
		ipio_info("ilitek_proc_oplus_upgrade_fw_write is not right\n");
		size = -1;
	}
#endif
	mutex_unlock(&idev->touch_mutex);
	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
	ilitek_plat_irq_enable();
	return size;
}

static ssize_t oplus_proc_gesture_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	char *ptr = NULL;
	uint8_t temp[64] = {0};
	ptr = kzalloc(size,GFP_KERNEL);
	if (ptr == NULL) {
		ipio_err("allocate the memory fail\n");
		return -1;
	}
	res = copy_from_user(ptr, buff, size);
	if (res) {
		ipio_err("copy data from user space, failed\n");
		kfree(ptr);
		return -1;
	}
	//data = cmd[0]-48;
	//ipio_info("size = %d, cmd = %s data %d\n", (int)size, cmd[0],data);
	mutex_lock(&idev->touch_mutex);	
	ipio_debug(DEBUG_TOUCH, "core_gesture->suspend = %d, gesture_done = %d\n", idev->suspend, idev->gesture_done);
	if (ptr[0] == '1') {
		if (idev->suspend) {
			idev->psensor_close = false;
			idev->gesture = true;
            idev->spi_gesture_cmd = true;
			tp_gesture = 1;
			temp[0] = 0xF6;
			temp[1] = 0x0A;
			 ipio_info("write prepare gesture command 0xF6 0x0A \n");
			if ((idev->write(temp, 2)) < 0) {
				ipio_info("write prepare gesture command error\n");
			}
			temp[0] = 0x01;
			temp[1] = 0x0A;
			temp[2] = idev->gesture_mode;
			if ((idev->write(temp, 3)) < 0) {
				ipio_info("write gesture command error\n");
			}
            idev->spi_gesture_cmd = false;
		} else {
			ipio_info("enable gesture mode\n");
			idev->gesture = true;
			tp_gesture = 1;
		}


	} else if (ptr[0] == '0') {
		ipio_info("disable gesture mode\n");
		idev->gesture = false;
		tp_gesture = 0;
	} else if(ptr[0] == '2') {
		if ((idev->suspend) && (idev->gesture_done == true)) {
			/* sleep in */
			ipio_debug(DEBUG_TOUCH, "core_config->isEnableGesture = false, enter gesture sleep\n");
			idev->gesture = false;
            idev->spi_gesture_cmd = true;
			tp_gesture = 0;
			ilitek_tddi_ic_func_ctrl("sleep", SLEEP_IN);
            idev->spi_gesture_cmd = false;
		} else {
			ipio_debug(DEBUG_TOUCH, "gesture sleep doing nothing\n");
		}
		ipio_info("psensor_close = true\n");
		idev->psensor_close = true;
	}
	else {
		ipio_err("Unknown command\n");
	}
	mutex_unlock(&idev->touch_mutex);
	kfree(ptr);
	return size;
}

static ssize_t oplus_proc_gesture_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	int len = 0;
	char *ptr = NULL;
	if ( *ppos ) {
	    printk("is already read the file\n");
        return 0;
	}
    *ppos += count;
    ptr = kzalloc(count,GFP_KERNEL);
	if(ptr == NULL){
		printk("allocate memory fail\n");
		return -1;
	}
    len = snprintf(ptr, count,"%x\n",idev->gesture);

    ret = copy_to_user(buf,ptr,len);

	kfree(ptr);
	return len;
}

static ssize_t oplus_proc_irq_depth_read(struct file *file, char *buf,
								  size_t len, loff_t *pos)
{
	size_t ret = 0;
	int res;
	char *temp_buf;
	struct irq_desc *desc = irq_to_desc(idev->irq_num);
	if ( *pos ) {
	    printk("is already read the file\n");
        return 0;
	}
    *pos += len;
	temp_buf = kzalloc(len, GFP_KERNEL);
	if(temp_buf == NULL){
		ipio_err("allocate memory fail\n");
		return -1;
	}
	
	ret += snprintf(temp_buf + ret, len - ret, "now depth=%d\n", desc->depth);
	res = copy_to_user(buf, temp_buf, len);
	kfree(temp_buf);
	return ret;
}

static ssize_t oplus_proc_i2c_device_read(struct file *file, char *buf,
								  size_t len, loff_t *pos)
{
	size_t ret = 0;
	int res;
	char *temp_buf;
	if ( *pos ) {
		printk("is already read the file\n");
		return 0;
	}
	*pos += len;
	temp_buf = kzalloc(len, GFP_KERNEL);
	if (ERR_ALLOC_MEM(temp_buf)) {
		ipio_err("Failed to allocate temp_buf mem, %ld\n", PTR_ERR(temp_buf));
		return -ENOMEM;
	}
	ret = snprintf(temp_buf, len, "ILITEK SPI devices,driver ver =%s\n", DRIVER_VERSION);
	res = copy_to_user(buf, temp_buf, len);
	kfree(temp_buf);

	return ret;
}

  static ssize_t oplus_proc_tpcell_read(struct file *file, char *buf,
									size_t len, loff_t *pos)
  {
	  size_t ret = 0;
	  int res;
	  char *temp_buf;
	  int tp_cell_info = 1;
	  if ( *pos ) {
		  printk("is already read the file\n");
		  return 0;
	  }
	  *pos += len;
	  temp_buf = kzalloc(len, GFP_KERNEL);
	  if (ERR_ALLOC_MEM(temp_buf)) {
		  ipio_err("Failed to allocate temp_buf mem, %ld\n", PTR_ERR(temp_buf));
		  return -ENOMEM;
	  }
	  ret = snprintf(temp_buf, len, "%d\n", tp_cell_info);
	  res = copy_to_user(buf, temp_buf, len);
	  kfree(temp_buf);
  
	  return ret;
  }

/*
*边缘抑制：
*direction 0 : 竖屏       固件指令01
*direction 1 : 横屏90度   固件指令02
*direction 2 : 横屏270度  固件指令00
*/
static ssize_t ilitek_limit_control_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[8] = {0};
	int  temp;
	ipio_info("\n");
	if ( *ppos ) {
		printk("is already read the file\n");
		return 0;
	}
	*ppos += count;
	
	if (buffer != NULL)
	{
		if (copy_from_user(buf, buffer, count)) {
			ipio_err("%s: read proc input error.\n", __func__);
			return count;
		}
	}
	sscanf(buf, "%x", &temp);
	if (temp > 0x1F) {
        ipio_info("%s: temp = 0x%x > 0x1F \n", __func__, temp);
        return count;
    }
	ipio_info("%s,direction = %d\n",__func__,idev->direction);
	mutex_lock(&idev->touch_mutex);
#ifdef ODM_WT_EDIT
		if (idev->suspend==false) {
			if (idev->direction == 0) {
				ilitek_tddi_ic_func_ctrl("edge_palm", 0x1);
			} else if (idev->direction == 1) {
				ilitek_tddi_ic_func_ctrl("edge_palm", 0x2);
			} else if (idev->direction == 2) {
				ilitek_tddi_ic_func_ctrl("edge_palm", 0x0);
			}
		}
#endif
	mutex_unlock(&idev->touch_mutex);
    return count;
}

static ssize_t ilitek_limit_control_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	int ret = 0;
	int len = 0;
	char *ptr = NULL;
	if ( *ppos ) {
    ipio_info("is already read the file\n");
    return 0;
	}
    *ppos += count;
	ptr = (char*)kzalloc(count,GFP_KERNEL);
	if (ERR_ALLOC_MEM(ptr)) {
		ipio_err("Failed to allocate temp_buf mem, %ld\n", PTR_ERR(ptr));
		return -ENOMEM;
	}
	len += snprintf(ptr+len, count-len,"not need\n");
	ret = copy_to_user(buffer,ptr,len);
	*ppos = len;
	kfree(ptr);
	return len;
}

static ssize_t ilitek_direction_read(struct file *file, char __user *userbuf, size_t count, loff_t *ppos)
{
       uint8_t buf[8];
       uint8_t len;
       int ret;
       len = 2;
       if(*ppos) {
		return 0;    /* the end */
       }
       sprintf(buf,"%d",idev->direction);
       len += sprintf(buf+len, "\n");
       ret = copy_to_user(userbuf,buf,len);
       if(ret<0)
              ipio_err("copy to user error\n");

       *ppos += len;
       return len;
}

static ssize_t ilitek_direction_write(struct file *file, const char __user *userbuf, size_t count, loff_t *ppos)
{
       uint8_t buf[5] = {0};
       if(count == 0) {
              ipio_err("count is 0 error\n");
              return -EINVAL;
       }
       ipio_info("count:%zd\n",count);
       if(copy_from_user(buf, userbuf, count)) {
              ipio_err("input value error\n");
              return -EINVAL;
       }

       idev->direction = buf[0] -'0';
       mutex_lock(&idev->touch_mutex);
#ifdef ODM_WT_EDIT
       if (idev->suspend==false) {
              if (idev->direction == 0) {
                     ilitek_tddi_ic_func_ctrl("edge_palm", 0x1);
	      } else if (idev->direction == 1) {
		     ilitek_tddi_ic_func_ctrl("edge_palm", 0x2);
	      } else if (idev->direction == 2) {
		     ilitek_tddi_ic_func_ctrl("edge_palm", 0x0);
	      }
       }
#endif
       mutex_unlock(&idev->touch_mutex);
       ipio_info("%s,core_config->direction:%d",__func__,idev->direction);
       return count;
}

static ssize_t oplus_proc_hopping_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	int len = 0;
	char *ptr = NULL;

	if ( *ppos ) {
	    printk("is already read the file\n");
        return 0;
	}
    *ppos += count;

    ptr = kzalloc(count,GFP_KERNEL);
	if(ptr == NULL){
		printk("allocate memory fail\n");
		return -1;
	}

    len = snprintf(ptr, count,"%x\n",idev->hopping);

    ret = copy_to_user(buf,ptr,len);

	kfree(ptr);
	return len;
}
static ssize_t oplus_proc_hopping_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	char buf[8] = {0};
	int  temp;
	if ( *pPos ) {
		ipio_err("is already read the file\n");
		return 0;
	}
	*pPos += size;
	if (buff != NULL)
	{
		if (copy_from_user(buf, buff, size)) {
			ipio_err("%s: read proc input error.\n", __func__);
			return size;
		}
	}
	sscanf(buf, "%x", &temp);
	if (temp > 0x1F) {
        ipio_info("%s: temp = 0x%x > 0x1F \n", __func__, temp);
        return size;
    }
	//ipio_info("size = %d, cmd = %s data %d\n", (int)size, cmd[0],data);
	mutex_lock(&idev->touch_mutex);
	if ((0x00 < temp)&&( temp <= 0x0A)) {
		ipio_info("enable hopping mode%x\n",temp);
		idev->hopping= true;
		ilitek_tddi_ic_func_ctrl("hopping_ctrl", temp);
	} else if (temp == 0x00){
		ipio_info("disable hopping mode\n");
		idev->hopping = false;
		ilitek_tddi_ic_func_ctrl("hopping_ctrl", temp);
	} else {
		ipio_info("unknow cmd\n");
	}
	mutex_unlock(&idev->touch_mutex);
	return size;
}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

struct file_operations wt_open_test_fops = {
	.read = wt_mptest_read,
};
struct file_operations proc_oplus_mp_lcm_on_fops = {
	.read = oplus_proc_mp_lcm_on_read,
};
struct file_operations proc_black_screen_test_fops = {
	.read  = oplus_proc_black_screen_test_read,
	.write = oplus_proc_black_screen_test_write,
};
struct file_operations proc_game_switch_enable_fops = {
	.read  = oplus_proc_game_switch_read,
	.write = oplus_proc_game_switch_write,
};

const struct seq_operations oplus_ili_coordinate_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_oplus_ili_coordinate_show
};

static int32_t oplus_ili_coordinate_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &oplus_ili_coordinate_seq_ops);
}

static const struct file_operations oplus_ili_coordinate_fops = {
	.owner = THIS_MODULE,
	.open = oplus_ili_coordinate_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

struct file_operations proc_CDC_delta_fops = {
	.read = ilitek_proc_CDC_delta_read,
};

struct file_operations proc_rawdata_fops = {
	.read = ilitek_proc_rawdata_read,
};

struct file_operations proc_main_register_fops = {
	.read = ilitek_proc_main_register_read,
};

struct file_operations proc_oplus_debug_level_fops = {
	.write = ilitek_proc_oplus_debug_level_write,
};

struct file_operations proc_oplus_register_info_fops = {
	.read = ilitek_proc_read_write_register_read,
	.write = ilitek_proc_read_write_register_write,
};

struct file_operations proc_sign_firmware_fops = {
	.write = ilitek_proc_sign_firmware_write,
};

struct file_operations proc_oplus_upgrade_fw_fops = {
	.write = ilitek_proc_oplus_upgrade_fw_write,
};

struct file_operations proc_oplus_gesture_fops = {
	.write = oplus_proc_gesture_write,
	.read = oplus_proc_gesture_read,
};

struct file_operations proc_irq_depth_fops = {
	.read = oplus_proc_irq_depth_read,
};

struct file_operations proc_i2c_device_fops = {
	.read = oplus_proc_i2c_device_read,
};
struct file_operations proc_tpcell_fops = {
	.read = oplus_proc_tpcell_read,
};

struct file_operations ilitek_limit_control_ops =
{
    .read  = ilitek_limit_control_read,
    .write = ilitek_limit_control_write,
    .owner = THIS_MODULE,
};
struct file_operations proc_oplus_debug_hopping_fops = {
	.read  = oplus_proc_hopping_read,
	.write = oplus_proc_hopping_write,
};

static const struct file_operations ilitek_direction_fops = {
       .owner = THIS_MODULE,
       .read = ilitek_direction_read,
       .write = ilitek_direction_write,
};

int oplus_proc_init(void)
{
	int res = 0;
	struct proc_dir_entry *proc_oplus_upgrade_fw_dir;
	struct proc_dir_entry *proc_oplus_gesture_dir;
	struct proc_dir_entry *proc_oplus_irq_depth_dir;
	struct proc_dir_entry *proc_oplus_i2c_device_test_dir;
	struct proc_dir_entry *proc_game_switch_enable;
	struct proc_dir_entry *proc_oplus_edge_limit_enable;
	struct proc_dir_entry *proc_dir_oplus;
	struct proc_dir_entry *proc_dir_debug_info;
	struct proc_dir_entry *proc_baseline_test;
	struct proc_dir_entry *proc_blackscreen_test;
	struct proc_dir_entry *proc_coordinate;
	struct proc_dir_entry *proc_CDC_delta;
	struct proc_dir_entry *proc_CDC_rawdata;
	struct proc_dir_entry *proc_main_register;
	struct proc_dir_entry *proc_oplus_debug_level;
	struct proc_dir_entry *proc_oplus_register_info;
	struct proc_dir_entry *proc_oplus_tp_direction_dir;
	struct proc_dir_entry *proc_oplus_tpcell_info;
	struct proc_dir_entry *proc_wt_dir;
	struct proc_dir_entry *proc_mptest_node;
	struct proc_dir_entry *proc_oplus_sign_firmware_dir;
	struct proc_dir_entry *proc_hopping_node;
	//for WT factory test
	proc_wt_dir = proc_mkdir("touchscreen", NULL);
	if ( proc_wt_dir == NULL )
	{
		ipio_err("create proc/touchscreen Failed!\n");
		res = -1;
	}
	proc_mptest_node = proc_create("ctp_openshort_test", 0666, proc_wt_dir, &wt_open_test_fops);
	if ( proc_mptest_node == NULL )
	{
		ipio_err("create proc/touchscreen/ctp_openshort_test Failed!\n");
		res = -1;
	}
	//for oplus
    proc_dir_oplus = proc_mkdir("touchpanel", NULL);
	if ( proc_dir_oplus == NULL )
	{
		ipio_err("create proc/touchpanel Failed!\n");
		res = -1;
	}
	proc_dir_debug_info = proc_mkdir("debug_info", proc_dir_oplus);
	if ( proc_dir_debug_info == NULL )
	{
		ipio_err("create proc/touchpanel/debug_info Failed!\n");
		res = -1;
	}
	proc_baseline_test = proc_create("baseline_test", 0666, proc_dir_oplus, &proc_oplus_mp_lcm_on_fops);
	if ( proc_baseline_test == NULL )
	{
		ipio_err("create proc/touchpanel/baseline_test Failed!\n");
		res = -1;
	}
	proc_blackscreen_test =proc_create("black_screen_test", 0666, proc_dir_oplus, &proc_black_screen_test_fops);
	if ( proc_blackscreen_test == NULL )
	{
		ipio_err("create proc/touchpanel/black_screen_test Failed!\n");
		res = -1;
	}
	proc_game_switch_enable =proc_create("game_switch_enable", 0666, proc_dir_oplus, &proc_game_switch_enable_fops);
	if ( proc_game_switch_enable == NULL )
	{
		ipio_err("create proc/touchpanel/game_switch_enable Failed!\n");
		res = -1;
	}
	proc_coordinate= proc_create("coordinate",0444,proc_dir_oplus,&oplus_ili_coordinate_fops);
	if ( proc_coordinate == NULL )
	{
		ipio_err("create proc/touchpanel/coordinate Failed!\n");
		res = -1;
	}
	proc_CDC_delta = proc_create("delta", 0666, proc_dir_debug_info, &proc_CDC_delta_fops);
	if ( proc_CDC_delta == NULL )
	{
		ipio_err("create proc/touchpanel/debug_info/delta Failed!\n");
		res = -1;
	}	
	proc_CDC_rawdata = proc_create("baseline", 0666, proc_dir_debug_info, &proc_rawdata_fops);
	if ( proc_CDC_rawdata == NULL )
	{
		ipio_err("create proc/touchpanel/debug_info/baseline Failed!\n");
		res = -1;
	}
	proc_main_register= proc_create("main_register", 0666, proc_dir_debug_info, &proc_main_register_fops);
	if ( proc_main_register == NULL )
	{
		ipio_err("create proc/touchpaneldebug_info/main_register Failed!\n");
		res = -1;
	}
	proc_oplus_debug_level =proc_create("debug_level", 0644, proc_dir_oplus, &proc_oplus_debug_level_fops);
	if ( proc_oplus_debug_level == NULL )
	{
		ipio_err("create proc/touchpanel/debug_level Failed!\n");
		res = -1;
	}
	proc_oplus_register_info =proc_create("oplus_register_info", 0664, proc_dir_oplus, &proc_oplus_register_info_fops);
	if ( proc_oplus_register_info == NULL )
	{
		ipio_err("create proc/touchpanel/oplus_register_info Failed!\n");
		res = -1;
	}
	proc_oplus_upgrade_fw_dir =proc_create("tp_fw_update", 0644, proc_dir_oplus, &proc_oplus_upgrade_fw_fops);
	if ( proc_oplus_upgrade_fw_dir == NULL )
	{
		ipio_err("create proc/touchpanel/tp_fw_update Failed!\n");
		res = -1;
	}
	proc_oplus_sign_firmware_dir =proc_create("sign_firmware", 0644, proc_dir_oplus, &proc_sign_firmware_fops);
	if ( proc_oplus_sign_firmware_dir == NULL )
	{
		ipio_err("create proc/touchpanel/sign_firmware Failed!\n");
		res = -1;
	}
	proc_oplus_gesture_dir =proc_create("double_tap_enable", 0666, proc_dir_oplus, &proc_oplus_gesture_fops);
	if ( proc_oplus_gesture_dir == NULL )
	{
		ipio_err("create proc/touchpanel/double_tap_enable Failed!\n");
		res = -1;
	}
	proc_oplus_irq_depth_dir =proc_create("irq_depth", 0666, proc_dir_oplus, &proc_irq_depth_fops);
	if ( proc_oplus_irq_depth_dir == NULL )
	{
		ipio_err("create proc/touchpanel/irq_depth Failed!\n");
		res = -1;
	}
	proc_oplus_i2c_device_test_dir =proc_create("i2c_device_test", 0666, proc_dir_oplus, &proc_i2c_device_fops);
	if ( proc_oplus_i2c_device_test_dir == NULL )
	{
		ipio_err("create proc/touchpanel/i2c_device_test Failed!\n");
		res = -1;
	}
	proc_oplus_edge_limit_enable = proc_create("oplus_tp_limit_enable", 0666, proc_dir_oplus, &ilitek_limit_control_ops);
	if ( proc_oplus_edge_limit_enable == NULL )
	{
		ipio_err("create proc/touchpanel/oplus_tp_limit_enable Failed!\n");
		res = -1;
	}
	proc_oplus_tp_direction_dir = proc_create("oplus_tp_direction", 0666, proc_dir_oplus, &ilitek_direction_fops);
	if ( proc_oplus_tp_direction_dir == NULL )
	{
		ipio_err("create proc/touchpanel/oplus_tp_direction Failed!\n");
		res = -1;
	}
	proc_oplus_tpcell_info =proc_create("incell_panel", 0666, proc_dir_oplus, &proc_tpcell_fops);
	if ( proc_oplus_i2c_device_test_dir == NULL )
	{
		ipio_err("create proc/touchpanel/incell_panel Failed!\n");
		res = -1;
	}

	proc_hopping_node =proc_create("freq_hop_simulate", 0666, proc_dir_debug_info, &proc_oplus_debug_hopping_fops);
	if ( proc_hopping_node == NULL )
	{
		ipio_err("create proc/touchpanel/hopping_ctrl Failed!\n");
		res = -1;
	}


	return res;
}

