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
#include <linux/firmware.h>

/* Firmware data with static array */


#define UPDATE_PASS		0
#define UPDATE_FAIL		-1
#define TIMEOUT_SECTOR		500
#define TIMEOUT_PAGE		3500
#define TIMEOUT_PROGRAM		10
bool is_first_boot = true;
extern int ili_ctpmodule ;
extern unsigned char* CTPM_FW;
extern struct upgrade_ili_fw_info *ili_fw;
#ifdef ODM_WT_EDIT
extern int sign_firmware;
#endif

struct touch_fw_data {
	u8 block_number;
	u32 start_addr;
	u32 end_addr;
	u32 new_fw_cb;
	int delay_after_upgrade;
	bool isCRC;
	bool isboot;
	int hex_tag;
} tfd;

struct flash_block_info {
	char *name;
	u32 start;
	u32 end;
	u32 len;
	u32 mem_start;
	u32 fix_mem_start;
	u8 mode;
} fbi[FW_BLOCK_INFO_NUM];

u8 gestrue_fw[(10 * K)];


static u32 HexToDec(char *phex, s32 len)
{
	u32 ret = 0, temp = 0, i;
	s32 shift = (len - 1) * 4;

	for (i = 0; i < len; shift -= 4, i++) {
		if ((phex[i] >= '0') && (phex[i] <= '9'))
			temp = phex[i] - '0';
		else if ((phex[i] >= 'a') && (phex[i] <= 'f'))
			temp = (phex[i] - 'a') + 10;
		else if ((phex[i] >= 'A') && (phex[i] <= 'F'))
			temp = (phex[i] - 'A') + 10;
		else
			return -1;

		ret |= (temp << shift);
	}
	return ret;
}

static u32 CalculateCRC32(u32 start_addr, u32 len, u8 *pfw)
{
	u32 i = 0, j = 0;
	u32 crc_poly = 0x04C11DB7;
	u32 tmp_crc = 0xFFFFFFFF;

	for (i = start_addr; i < start_addr + len; i++) {
		tmp_crc ^= (pfw[i] << 24);

		for (j = 0; j < 8; j++) {
			if ((tmp_crc & 0x80000000) != 0)
				tmp_crc = tmp_crc << 1 ^ crc_poly;
			else
				tmp_crc = tmp_crc << 1;
		}
	}
	return tmp_crc;
}

static int host_download_dma_check(u32 start_addr, u32 block_size)
{
	int count = 50;
	u32 busy = 0;

	/* dma1 src1 adress */
	ilitek_ice_mode_write(0x072104, start_addr, 4);
	/* dma1 src1 format */
	ilitek_ice_mode_write(0x072108, 0x80000001, 4);
	/* dma1 dest address */
	ilitek_ice_mode_write(0x072114, 0x00030000, 4);
	/* dma1 dest format */
	ilitek_ice_mode_write(0x072118, 0x80000000, 4);
	/* Block size*/
	ilitek_ice_mode_write(0x07211C, block_size, 4);

	idev->chip->hd_dma_check_crc_off();

	/* crc on */
	ilitek_ice_mode_write(0x041016, 0x01, 1);
	/* Dma1 stop */
	ilitek_ice_mode_write(0x072100, 0x00000000, 4);
	/* clr int */
	ilitek_ice_mode_write(0x048006, 0x1, 1);
	/* Dma1 start */
	ilitek_ice_mode_write(0x072100, 0x01000000, 4);

	/* Polling BIT0 */
	while (count > 0) {
		mdelay(1);
		busy = ilitek_ice_mode_read(0x048006, sizeof(u8));
		ipio_debug(DEBUG_FW, "busy = %x\n", busy);
		if ((busy & 0x01) == 1)
			break;
		count--;
	}

	if (count <= 0) {
		ipio_err("BIT0 is busy\n");
		return -1;
	}
	return ilitek_ice_mode_read(0x04101C, sizeof(u32));
}

static int ilitek_tddi_fw_iram_program(u32 start, u8 *w_buf, u32 w_len)
{
	if (!w_buf) {
		ipio_err("fw buffer is null\n");
		return -ENOMEM;
	}
	if (idev->fw_buf_dma == NULL) {
		ipio_info("The dma fw buf is null!");
		return -ENOMEM;
	}
	
	idev->fw_buf_dma[0] = SPI_WRITE;
	idev->fw_buf_dma[1] = 0x25;
	idev->fw_buf_dma[4] = (char)((start & 0x00FF0000) >> 16);
	idev->fw_buf_dma[3] = (char)((start & 0x0000FF00) >> 8);
	idev->fw_buf_dma[2] = (char)((start & 0x000000FF));
	
	
	memcpy(&idev->fw_buf_dma[5], w_buf, w_len);
	
	
	if (core_spi_upgrade_write(idev->spi, idev->fw_buf_dma, w_len + 5, NULL, 0) < 0) {
		return -1;
	}
	/* holding the status until finish this upgrade. */
	idev->fw_update_stat = 90;
	return 0;
}

static int ilitek_tddi_fw_check_hex_hw_crc(u8 *pfw)
{
	u32 i = 0, len = 0;
	u32 hex_crc = 0, hw_crc;

	for (i = 0; i < ARRAY_SIZE(fbi); i++) {
		if (fbi[i].end == 0)
			continue;

		len = fbi[i].end - fbi[i].start + 1 - 4;

		hex_crc = CalculateCRC32(fbi[i].start, len, pfw);
		hw_crc = ilitek_tddi_fw_read_hw_crc(fbi[i].start, len);

		ipio_info("Block = %d, Hex CRC = %x, HW CRC = %x\n", i, hex_crc, hw_crc);

		if (hex_crc != hw_crc) {
			ipio_err("Hex and HW CRC NO matched !!!\n");
			return UPDATE_FAIL;
		}
	}

	ipio_info("Hex and HW CRC match!\n");
	return UPDATE_PASS;
}

static int ilitek_tddi_flash_poll_busy(int timer)
{
	int ret = UPDATE_PASS, retry = timer;
	u8 cmd = 0x5, temp = 0;

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write(FLASH2_ADDR, cmd, 1);

	do {
		ilitek_ice_mode_write(FLASH2_ADDR, 0xFF, 1); /* Dummy */
		mdelay(1);
		temp = ilitek_ice_mode_read(FLASH4_ADDR, sizeof(u8));
		if ((temp & 0x3) == 0)
			break;
	} while (--retry >= 0);

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

	if (retry <= 0) {
		ipio_err("Flash polling busy timeout ! tmp = %x\n", temp);
		ret = UPDATE_FAIL;
	}

	return ret;
}

void ilitek_tddi_flash_clear_dma(void)
{
	ilitek_ice_mode_bit_mask_write(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_bit_mask_write(FLASH0_ADDR, FLASH0_reg_preclk_sel, (2 << 16));
	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x01, 1);	/* CS high */

	ilitek_ice_mode_bit_mask_write(FLASH4_ADDR, FLASH4_reg_flash_dma_trigger_en, (0 << 24));
	ilitek_ice_mode_bit_mask_write(FLASH0_ADDR, FLASH0_reg_rx_dual, (0 << 24));

	ilitek_ice_mode_write(FLASH3_reg_rcv_cnt, 0x00, 1);
	ilitek_ice_mode_write(FLASH4_reg_rcv_data, 0xFF, 1);
}

void ilitek_tddi_flash_dma_write(u32 start, u32 end, u32 len)
{
	ilitek_ice_mode_bit_mask_write(FLASH0_ADDR, FLASH0_reg_preclk_sel, 1 << 16);

	ilitek_ice_mode_write(FLASH0_reg_flash_csb, 0x00, 1);	/* CS low */
	ilitek_ice_mode_write(FLASH1_reg_flash_key1, 0x66aa55, 3);	/* Key */

	ilitek_ice_mode_write(FLASH2_reg_tx_data, 0x0b, 1);
	while (!(ilitek_ice_mode_read(INTR1_ADDR & BIT(25), sizeof(u32))))
			continue;
	ilitek_ice_mode_bit_mask_write(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_write(FLASH2_reg_tx_data, (start & 0xFF0000) >> 16, 1);
	while (!(ilitek_ice_mode_read(INTR1_ADDR & BIT(25), sizeof(u32))))
			continue;
	ilitek_ice_mode_bit_mask_write(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_write(FLASH2_reg_tx_data, (start & 0x00FF00) >> 8, 1);
	while (!(ilitek_ice_mode_read(INTR1_ADDR & BIT(25), sizeof(u32))))
			continue;
	ilitek_ice_mode_bit_mask_write(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_write(FLASH2_reg_tx_data, (start & 0x0000FF), 1);
	while (!(ilitek_ice_mode_read(INTR1_ADDR & BIT(25), sizeof(u32))))
			continue;
	ilitek_ice_mode_bit_mask_write(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_bit_mask_write(FLASH0_ADDR, FLASH0_reg_rx_dual, 0 << 24);

	ilitek_ice_mode_write(FLASH2_reg_tx_data, 0x00, 1);	/* Dummy */
	while (!(ilitek_ice_mode_read(INTR1_ADDR & BIT(25), sizeof(u32))))
		continue;
	ilitek_ice_mode_bit_mask_write(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_write(FLASH3_reg_rcv_cnt, len, 4);	/* Write Length */
}

static void ilitek_tddi_flash_write_enable(void)
{
	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write(FLASH2_ADDR, 0x6, 1);
	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */
}

u32 ilitek_tddi_fw_read_hw_crc(u32 start, u32 end)
{
	int retry = 500;
	u8 busy = 0;
	u32 write_len = end;
	u32 iram_check = 0;

	if (write_len > idev->chip->max_count) {
		ipio_err("The length (%x) written into firmware is greater than max count (%x)\n",
			write_len, idev->chip->max_count);
		return -1;
	}

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write(FLASH2_ADDR, 0x3b, 1);
	ilitek_ice_mode_write(FLASH2_ADDR, (start & 0xFF0000) >> 16, 1);
	ilitek_ice_mode_write(FLASH2_ADDR, (start & 0x00FF00) >> 8, 1);
	ilitek_ice_mode_write(FLASH2_ADDR, (start & 0x0000FF), 1);
	ilitek_ice_mode_write(0x041003, 0x01, 1); /* Enable Dio_Rx_dual */
	ilitek_ice_mode_write(FLASH2_ADDR, 0xFF, 1); /* Dummy */
	ilitek_ice_mode_write(0x04100C, write_len, 3); /* Set Receive count */
	ilitek_ice_mode_write(0x048007, 0x02, 1);/* Clear Int Flag */
	ilitek_ice_mode_write(0x041016, 0x00, 1);
	ilitek_ice_mode_write(0x041016, 0x01, 1);	/* Checksum_En */

	ilitek_ice_mode_write(FLASH4_ADDR, 0xFF, 1); /* Start to receive */

	do {
		busy = ilitek_ice_mode_read(0x048007, sizeof(u8));
		ipio_debug(DEBUG_FW, "busy = %x\n", busy);
		if (((busy >> 1) & 0x01) == 0x01)
			break;
	} while (--retry >= 0);

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

	if (retry <= 0) {
		ipio_err("Read HW CRC timeout !, busy = 0x%x\n", busy);
		return -1;
	}

	ilitek_ice_mode_write(0x041003, 0x0, 1); /* Disable dio_Rx_dual */
	iram_check = ilitek_ice_mode_read(0x04101C, sizeof(u32));
	return iram_check;
}

int ilitek_tddi_fw_read_flash_data(u32 start, u32 end, u8 *data, int len)
{
	u32 i, index = 0, precent;

	if (end - start > len) {
		ipio_err("the length (%d) reading crc is over than len(%d)\n", end - start, len);
		return -1;
	}

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write(FLASH2_ADDR, 0x03, 1);
	ilitek_ice_mode_write(FLASH2_ADDR, (start & 0xFF0000) >> 16, 1);
	ilitek_ice_mode_write(FLASH2_ADDR, (start & 0x00FF00) >> 8, 1);
	ilitek_ice_mode_write(FLASH2_ADDR, (start & 0x0000FF), 1);

	for (i = start; i <= end; i++) {
		ilitek_ice_mode_write(FLASH2_ADDR, 0xFF, 1); /* Dummy */
		data[index] = ilitek_ice_mode_read(FLASH4_ADDR, sizeof(u8));
		index++;
		precent = (i * 100) / end;
		ipio_debug(DEBUG_FW, "Reading flash data .... %d%c", precent, '%');
	}

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */
	return 0;
}

int ilitek_tddi_fw_dump_flash_data(u32 start, u32 end, bool user)
{
	struct file *f = NULL;
	u8 *hex_buffer = NULL;
	mm_segment_t old_fs;
	loff_t pos = 0;
	u32 start_addr, end_addr;
	int ret, length;

	f = filp_open(DUMP_FLASH_PATH, O_WRONLY | O_CREAT | O_TRUNC, 644);
	if (ERR_ALLOC_MEM(f)) {
		ipio_err("Failed to open the file at %ld.\n", PTR_ERR(f));
		return -1;
	}

	ret = ilitek_ice_mode_ctrl(ENABLE, OFF);
	if (ret < 0)
		return ret;

	if (user) {
		start_addr = 0x0;
		end_addr = 0x1FFFF;
	} else {
		start_addr = start;
		end_addr = end;
	}

	length = end_addr - start_addr + 1;
	ipio_info("len = %d\n", length);

	hex_buffer = vmalloc(length * sizeof(u8));
	if (ERR_ALLOC_MEM(hex_buffer)) {
		ipio_err("Failed to allocate hex_buffer memory, %ld\n", PTR_ERR(hex_buffer));
		filp_close(f, NULL);
		ilitek_ice_mode_ctrl(DISABLE, OFF);
		return -1;
	}

	ilitek_tddi_fw_read_flash_data(start_addr, end_addr, hex_buffer, length);

	old_fs = get_fs();
	set_fs(get_ds());
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_write(f, hex_buffer, length, &pos);
	set_fs(old_fs);
	filp_close(f, NULL);
	ipio_vfree((void **)&hex_buffer);
	ilitek_ice_mode_ctrl(DISABLE, OFF);
	ipio_info("dump flash success\n");
	return 0;
}

static void ilitek_tddi_flash_protect(bool enable)
{
	ipio_info("%s flash protection\n", enable ? "Enable" : "Disable");

	ilitek_tddi_flash_write_enable();

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write(FLASH2_ADDR, 0x1, 1);
	ilitek_ice_mode_write(FLASH2_ADDR, 0x0, 1);

	switch (idev->flash_mid) {
	case 0xEF:
		if (idev->flash_devid == 0x6012 || idev->flash_devid == 0x6011) {
			if (enable)
				ilitek_ice_mode_write(FLASH2_ADDR, 0x7E, 1);
			else
				ilitek_ice_mode_write(FLASH2_ADDR, 0x0, 1);
		}
		break;
	case 0xC8:
		if (idev->flash_devid == 0x6012 || idev->flash_devid == 0x6013) {
			if (enable)
				ilitek_ice_mode_write(FLASH2_ADDR, 0x7A, 1);
			else
				ilitek_ice_mode_write(FLASH2_ADDR, 0x0, 1);
		}
		break;
	default:
		ipio_err("Can't find flash id(0x%x), ignore protection\n", idev->flash_mid);
		break;
	}

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */
}
static int ilitek_tddi_fw_iram_read(u8 *buf, u32 start, u32 end)
{
    int i;
    int addr = 0, r_len = SPI_UPGRADE_LEN;
    u8 cmd[4] = {0};

    if (!buf) {
        ipio_err("buf in null\n");
        return -ENOMEM;
    }

    for (addr = start, i = 0; addr < end; i += r_len, addr += r_len) {
        if ((addr + r_len) > (end + 1))
            r_len = end % r_len;

        cmd[0] = 0x25;
        cmd[3] = (char)((addr & 0x00FF0000) >> 16);
        cmd[2] = (char)((addr & 0x0000FF00) >> 8);
        cmd[1] = (char)((addr & 0x000000FF));

        if (idev->write(cmd, 4)) {
            ipio_err("Failed to write iram data\n");
            return -ENODEV;
        }

        if (idev->read(buf + i, r_len)) {
            ipio_err("Failed to Read iram data\n");
            return -ENODEV;
        }
    }
    return 0;
}

int ilitek_tddi_fw_dump_iram_data(u32 start, u32 end)
{
    struct file *f = NULL;
    u8 *buf = NULL;
    mm_segment_t old_fs;
    loff_t pos = 0;
    int ret, wdt, i;
    int len;

    f = filp_open(DUMP_IRAM_PATH, O_WRONLY | O_CREAT | O_TRUNC, 644);
    if (ERR_ALLOC_MEM(f)) {
        ipio_err("Failed to open the file at %ld.\n", PTR_ERR(f));
        return -1;
    }

    ret = ilitek_ice_mode_ctrl(ENABLE, OFF);
    if (ret < 0) {
        filp_close(f, NULL);
        return ret;
    }

    wdt = ilitek_tddi_ic_watch_dog_ctrl(ILI_READ, DISABLE);
    if (wdt)
        ilitek_tddi_ic_watch_dog_ctrl(ILI_WRITE, DISABLE);

    len = end - start + 1;

    buf = vmalloc(len * sizeof(u8));
    if (ERR_ALLOC_MEM(buf)) {
        ipio_err("Failed to allocate buf memory, %ld\n", PTR_ERR(buf));
        filp_close(f, NULL);
        ret = ENOMEM;
        goto out;
    }

    for (i = 0; i < len; i++)
        buf[i] = 0xFF;

    if (ilitek_tddi_fw_iram_read(buf, start, end) < 0)
        ipio_err("Read IRAM data failed\n");

    old_fs = get_fs();
    set_fs(get_ds());
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_write(f, buf, len, &pos);
    set_fs(old_fs);

out:
    if (wdt)
        ilitek_tddi_ic_watch_dog_ctrl(ILI_WRITE, ENABLE);

    ilitek_ice_mode_ctrl(DISABLE, OFF);
    filp_close(f, NULL);
    ipio_vfree((void **)&buf);
    ipio_info("dump iram data success\n");
    return 0;
}

static int ilitek_tddi_fw_check_ver(u8 *pfw)
{
	int i, crc_byte_len = 4;
	u8 flash_crc[4] = {0};
	u32 start_addr = 0, end_addr = 0;
	u32 block_crc, flash_crc_cb;

	/* Get current firmware version from chip */
	ilitek_tddi_ic_get_protocl_ver();
	ilitek_tddi_ic_get_fw_ver();

	/* Check FW version */
	ipio_info("New FW ver = 0x%x, Current FW ver = 0x%x\n", tfd.new_fw_cb, idev->chip->fw_ver);
	if (tfd.new_fw_cb != idev->chip->fw_ver) {
		ipio_info("FW version is different, do upgrade\n");
		return UPDATE_FAIL;
	}

	ipio_info("FW version is the same, check Flash and HW CRC if there's corruption.\n");

	/* Check Flash and HW CRC with last 4 bytes in each block */
	for (i = 0; i < ARRAY_SIZE(fbi); i++) {
		start_addr = fbi[i].start;
		end_addr = fbi[i].end;

		/* Invaild end address */
		if (end_addr == 0)
			continue;

		if (ilitek_tddi_fw_read_flash_data(end_addr - crc_byte_len + 1, end_addr,
					flash_crc, sizeof(flash_crc)) < 0) {
			ipio_err("Read Flash failed\n");
			return UPDATE_FAIL;
		}

		flash_crc_cb = flash_crc[0] << 24 | flash_crc[1] << 16 | flash_crc[2] << 8 | flash_crc[3];

		block_crc = ilitek_tddi_fw_read_hw_crc(start_addr, end_addr - start_addr - crc_byte_len + 1);

		ipio_info("Block = %d, HW CRC = 0x%06x, Flash CRC = 0x%06x\n", i, block_crc, flash_crc_cb);

		/* Compare Flash CRC with HW CRC */
		if (flash_crc_cb != block_crc) {
			ipio_info("Both are different, do update\n");
			return UPDATE_FAIL;
		}
		memset(flash_crc, 0, sizeof(flash_crc));
	}

	ipio_info("Both are the same, no need to update\n");
	return UPDATE_PASS;
}

static int ilitek_tddi_fw_iram_upgrade(u8 *pfw)
{
	int i, ret = UPDATE_PASS;
	u32 mode, crc, dma;
	u8 *fw_ptr = NULL;

	if(!is_resume){
		if (idev->actual_tp_mode != P5_X_FW_GESTURE_MODE) {
			ilitek_tddi_reset_ctrl(idev->reset);
			idev->ignore_first_irq = true;
		}
		else {
			idev->ignore_first_irq = false;
		}
		ret = ilitek_ice_mode_ctrl(ENABLE, OFF);
		if (ret < 0)
			return ret;
	}
	is_resume = false;
	ret = ilitek_tddi_ic_watch_dog_ctrl(ILI_WRITE, DISABLE);
	if (ret < 0)
		return ret;

	fw_ptr = pfw;
	if (idev->actual_tp_mode == P5_X_FW_TEST_MODE) {
		mode = MP;
	} else if (idev->actual_tp_mode == P5_X_FW_GESTURE_MODE) {
		mode = GESTURE;
		fw_ptr = gestrue_fw;
	} else {
		mode = AP;
	}

	/* Program data to iram acorrding to each block */
	for (i = 0; i < ARRAY_SIZE(fbi); i++) {
		if (fbi[i].mode == mode && fbi[i].len != 0) {
			ipio_info("Download %s code from hex 0x%x to IRAN 0x%x len = 0x%x\n", fbi[i].name, fbi[i].start, fbi[i].mem_start, fbi[i].len);
			ilitek_tddi_fw_iram_program(fbi[i].mem_start,(fw_ptr + fbi[i].start), fbi[i].len);
			crc = CalculateCRC32(fbi[i].start, fbi[i].len - 4, fw_ptr);
			dma = host_download_dma_check(fbi[i].mem_start, fbi[i].len - 4);

			ipio_info("%s CRC is %s (%x) : (%x)\n", fbi[i].name, (crc != dma ? "Invalid !" : "Correct !"), crc, dma);

			if (crc != dma) {
				ret = UPDATE_FAIL;
				return ret;
			}
	 }
	}

	if (idev->actual_tp_mode != P5_X_FW_GESTURE_MODE){
		ilitek_tddi_reset_ctrl(TP_IC_CODE_RST);
	}
	ilitek_ice_mode_ctrl(DISABLE, OFF);
	mdelay(60);
	return ret;
}

static int ilitek_tddi_fw_flash_program(u8 *pfw)
{
	u8 buf[512] = {0};
	u32 i = 0, addr = 0, k = 0, recv_addr = 0;
	bool skip = true;

	for (i = 0; i < FW_BLOCK_INFO_NUM; i++) {
		if (fbi[i].end == 0)
			continue;

		if (fbi[i].start >= RESERVE_BLOCK_START_ADDR &&
			fbi[i].end <= RESERVE_BLOCK_END_ADDR)
			continue;

		ipio_info("Block[%d]: Programing from (0x%x) to (0x%x)\n", i, fbi[i].start, fbi[i].end);

		for (addr = fbi[i].start; addr < fbi[i].end; addr += idev->program_page) {
			buf[0] = 0x25;
			buf[3] = 0x04;
			buf[2] = 0x10;
			buf[1] = 0x08;

			for (k = 0; k < idev->program_page; k++) {
				if (addr + k <= tfd.end_addr)
					buf[4 + k] = pfw[addr + k];
				else
					buf[4 + k] = 0xFF;

				if (buf[4 + k] != 0xFF)
					skip = false;
			}

			if (skip) {
				ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */
				return UPDATE_FAIL;
			}

			ilitek_tddi_flash_write_enable();

			ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
			ilitek_ice_mode_write(FLASH1_ADDR, 0x66aa55, 3); /* Key */
			ilitek_ice_mode_write(FLASH2_ADDR, 0x2, 1);
			recv_addr = ((addr & 0xFF0000) >> 16) | (addr & 0x00FF00) | ((addr & 0x0000FF) << 16);
			ilitek_ice_mode_write(FLASH2_ADDR, recv_addr, 3);

			if (idev->write(buf, idev->program_page + 4) < 0) {
				ipio_err("Failed to program data at start_addr = 0x%X, k = 0x%X, addr = 0x%x\n",
				addr, k, addr + k);
				ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */
				return UPDATE_FAIL;
			}

			ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

			if (idev->flash_mid == 0xEF) {
				mdelay(1);
			} else {
				if (ilitek_tddi_flash_poll_busy(TIMEOUT_PROGRAM) < 0)
					return UPDATE_FAIL;
			}

			/* holding the status until finish this upgrade. */
			idev->fw_update_stat = (addr * 101) / tfd.end_addr;
			if (idev->fw_update_stat > 90)
				idev->fw_update_stat = 90;
		}
	}
	return UPDATE_PASS;
}

static int ilitek_tddi_fw_flash_erase(void)
{
	int ret = 0;
	u32 i = 0, addr = 0, recv_addr = 0;

	for (i = 0; i < FW_BLOCK_INFO_NUM; i++) {
		if (fbi[i].end == 0)
			continue;

		if (fbi[i].start >= RESERVE_BLOCK_START_ADDR &&
			fbi[i].end <= RESERVE_BLOCK_END_ADDR)
			continue;

		ipio_info("Block[%d]: Erasing from (0x%x) to (0x%x) \n", i, fbi[i].start, fbi[i].end);

		for (addr = fbi[i].start; addr <= fbi[i].end; addr += idev->flash_sector) {
			ilitek_tddi_flash_write_enable();

			ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
			ilitek_ice_mode_write(FLASH1_ADDR, 0x66aa55, 3); /* Key */

			if (addr == fbi[AP].start)
				ilitek_ice_mode_write(FLASH2_ADDR, 0xD8, 1);
			else
				ilitek_ice_mode_write(FLASH2_ADDR, 0x20, 1);

			recv_addr = ((addr & 0xFF0000) >> 16) | (addr & 0x00FF00) | ((addr & 0x0000FF) << 16);
			ilitek_ice_mode_write(FLASH2_ADDR, recv_addr, 3);

			ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

			mdelay(1);

			if (addr == fbi[AP].start)
				ret = ilitek_tddi_flash_poll_busy(TIMEOUT_PAGE);
			else
				ret = ilitek_tddi_flash_poll_busy(TIMEOUT_SECTOR);

			if (ret < 0)
				return UPDATE_FAIL;

			ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

			if (fbi[i].start == fbi[AP].start)
				break;
		}
	}
	return UPDATE_PASS;
}

static int ilitek_tddi_fw_flash_upgrade(u8 *pfw)
{
	int ret = UPDATE_PASS;

	ilitek_tddi_reset_ctrl(idev->reset);

	ret = ilitek_ice_mode_ctrl(ENABLE, OFF);
	if (ret < 0)
		return UPDATE_FAIL;

	ret = ilitek_tddi_ic_watch_dog_ctrl(ILI_WRITE, DISABLE);
	if (ret < 0)
		return ret;

	ret = ilitek_tddi_fw_check_ver(pfw);
	if (ret == UPDATE_PASS)
		goto out;

	ret = ilitek_tddi_fw_flash_erase();
	if (ret == UPDATE_FAIL)
		goto out;

	ret = ilitek_tddi_fw_flash_program(pfw);
	if (ret == UPDATE_FAIL)
		goto out;

	/* We do have to reset chip in order to move new code from flash to iram. */
	ilitek_tddi_reset_ctrl(idev->reset);

	/* the delay time moving code depends on what the touch IC you're using. */
	mdelay(200);

	ret = ilitek_ice_mode_ctrl(ENABLE, OFF);
	if (ret < 0)
		goto out;

	ret = ilitek_tddi_fw_check_hex_hw_crc(pfw);

out:
	ilitek_ice_mode_ctrl(DISABLE, OFF);
	return ret;
}

static void ilitek_tddi_fw_update_block_info(u8 *pfw, u8 type)
{
	u32 ges_area_section, ges_info_addr, ges_fw_start, ges_fw_end;

	ipio_info("Upgarde = %s, Tag = %x\n", type ? "IRAM" : "Flash", tfd.hex_tag);

	if (type == UPGRADE_IRAM) {
		if (tfd.hex_tag == BLOCK_TAG_AF) {
			fbi[AP].mem_start = (fbi[AP].fix_mem_start != INT_MAX) ? fbi[AP].fix_mem_start : 0;
			fbi[DATA].mem_start = (fbi[DATA].fix_mem_start != INT_MAX) ? fbi[DATA].fix_mem_start : DLM_START_ADDRESS;
			fbi[TUNING].mem_start = (fbi[TUNING].fix_mem_start != INT_MAX) ? fbi[TUNING].fix_mem_start :  fbi[DATA].mem_start + fbi[DATA].len;
			fbi[MP].mem_start = (fbi[MP].fix_mem_start != INT_MAX) ? fbi[MP].fix_mem_start :  0;
			fbi[GESTURE].mem_start = (fbi[GESTURE].fix_mem_start != INT_MAX) ? fbi[GESTURE].fix_mem_start :	 0;

			/* Parsing gesture info form AP code */
			ges_info_addr = (fbi[AP].end + 1 - 60);
			ges_area_section = (pfw[ges_info_addr + 3] << 24) + (pfw[ges_info_addr + 2] << 16) + (pfw[ges_info_addr + 1] << 8) + pfw[ges_info_addr];
			fbi[GESTURE].mem_start = (pfw[ges_info_addr + 7] << 24) + (pfw[ges_info_addr + 6] << 16) + (pfw[ges_info_addr + 5] << 8) + pfw[ges_info_addr + 4];
			fbi[GESTURE].len = MAX_GESTURE_FIRMWARE_SIZE;
			ges_fw_start = (pfw[ges_info_addr + 15] << 24) + (pfw[ges_info_addr + 14] << 16) + (pfw[ges_info_addr + 13] << 8) + pfw[ges_info_addr + 12];
			ges_fw_end = fbi[GESTURE].end;
			fbi[GESTURE].start = 0;
		} else {
			fbi[AP].start = 0;
			fbi[AP].mem_start = 0;
			fbi[AP].len = MAX_AP_FIRMWARE_SIZE;

			fbi[DATA].start = DLM_HEX_ADDRESS;
			fbi[DATA].mem_start = DLM_START_ADDRESS;
			fbi[DATA].len = MAX_DLM_FIRMWARE_SIZE;

			fbi[MP].start = MP_HEX_ADDRESS;
			fbi[MP].mem_start = 0;
			fbi[MP].len = MAX_MP_FIRMWARE_SIZE;

			/* Parsing gesture info form AP code */
			ges_area_section = (pfw[0xFFCF] << 24) + (pfw[0xFFCE] << 16) + (pfw[0xFFCD] << 8) + pfw[0xFFCC];
			fbi[GESTURE].mem_start = (pfw[0xFFCB] << 24) + (pfw[0xFFCA] << 16) + (pfw[0xFFC9] << 8) + pfw[0xFFC8];
			fbi[GESTURE].len = MAX_GESTURE_FIRMWARE_SIZE;
			ges_fw_start = (pfw[0xFFD3] << 24) + (pfw[0xFFD2] << 16) + (pfw[0xFFD1] << 8) + pfw[0xFFD0];
			ges_fw_end = (pfw[0xFFD3] << 24) + (pfw[0xFFD2] << 16) + (pfw[0xFFD1] << 8) + pfw[0xFFD0];
			fbi[GESTURE].start = 0;
		}

		memset(gestrue_fw, 0xff, sizeof(gestrue_fw));

		/* Copy gesture data */
		if (fbi[GESTURE].mem_start != 0xffffffff && ges_fw_start != 0xffffffff && fbi[GESTURE].mem_start != 0 && ges_fw_start != 0)
			ipio_memcpy(gestrue_fw, (pfw + ges_fw_start), fbi[GESTURE].len, sizeof(gestrue_fw));
		else
			ipio_err("There is no gesture data inside fw\n");

		ipio_info("gesture memory start = 0x%x, upgrade lenth = 0x%x, hex area = %d, ap_start_addr = 0x%x, ap_end_addr = 0x%x",
					fbi[GESTURE].mem_start, MAX_GESTURE_FIRMWARE_SIZE, ges_area_section, ges_fw_start, ges_fw_end);

		fbi[AP].name = "AP";
		fbi[DATA].name = "DATA";
		fbi[TUNING].name = "TUNING";
		fbi[MP].name = "MP";
		fbi[GESTURE].name = "GESTURE";

		/* upgrade mode define */
		if (tfd.hex_tag == BLOCK_TAG_AF) {
			fbi[DATA].mode = fbi[AP].mode = fbi[TUNING].mode = AP;
		}
		else {
			fbi[DATA].mode = fbi[AP].mode = AP;
		}
		fbi[MP].mode = MP;
		fbi[GESTURE].mode = GESTURE;
	}

	/* Get hex fw vers */
	tfd.new_fw_cb = (pfw[FW_VER_ADDR] << 24) | (pfw[FW_VER_ADDR + 1] << 16) |
			(pfw[FW_VER_ADDR + 2] << 8) | (pfw[FW_VER_ADDR + 3]);

	/* Calculate update adress	*/
	ipio_info("New FW ver = 0x%x\n", tfd.new_fw_cb);
	ipio_info("star_addr = 0x%06X, end_addr = 0x%06X, Block Num = %d\n", tfd.start_addr, tfd.end_addr, tfd.block_number);
}
static void ilitek_tddi_fw_ili_convert_xl(u8 *pfw)
{
	int i = 0, block_enable = 0, num = 0;
	u8 block;
	u32 Addr;
	ipio_info("Start to parse ILI file, type = %d, block_count = %d\n", CTPM_FW[32], CTPM_FW[33]);
	memset(fbi, 0x0, sizeof(fbi));

	tfd.start_addr = 0;
	tfd.end_addr = 0;
	tfd.hex_tag = 0;
	block_enable = CTPM_FW[32];

	if (block_enable == 0) {
		tfd.hex_tag = BLOCK_TAG_AE;
		goto out;
	}

	tfd.hex_tag = BLOCK_TAG_AF;
	for (i = 0; i < FW_BLOCK_INFO_NUM; i++) {
		if (((block_enable >> i) & 0x01) == 0x01) {
			num = i + 1;
			if ((num) == 6) {
				fbi[num].start = (CTPM_FW[0] << 16) + (CTPM_FW[1] << 8) + (CTPM_FW[2]);
				fbi[num].end = (CTPM_FW[3] << 16) + (CTPM_FW[4] << 8) + (CTPM_FW[5]);
				fbi[num].fix_mem_start = INT_MAX;
			} else {
				fbi[num].start = (CTPM_FW[34 + i * 6] << 16) + (CTPM_FW[35 + i * 6] << 8) + (CTPM_FW[36 + i * 6]);
				fbi[num].end = (CTPM_FW[37 + i * 6] << 16) + (CTPM_FW[38 + i * 6] << 8) + (CTPM_FW[39 + i * 6]);
				fbi[num].fix_mem_start = INT_MAX;
			}
			fbi[num].len = fbi[num].end - fbi[num].start + 1;
			ipio_info("Block[%d]: start_addr = %x, end = %x\n", num, fbi[num].start, fbi[num].end);
		}
	}

	if ((block_enable & 0x80) == 0x80) {
		for (i = 0; i < 3; i++) {
			Addr = (CTPM_FW[6 + i * 4] << 16) + (CTPM_FW[7 + i * 4] << 8) + (CTPM_FW[8 + i * 4]);
			block = CTPM_FW[9 + i * 4];

			if ((block != 0) && (Addr != 0x000000)) {
				fbi[block].fix_mem_start = Addr;
				ipio_info("Tag 0xB0: change Block[%d] to addr = 0x%x\n", block, fbi[block].fix_mem_start);
			}
		}
	}

out:
	tfd.block_number = CTPM_FW[33];
	memcpy(pfw, CTPM_FW + ILI_FILE_HEADER, (ili_fw->fw_len - ILI_FILE_HEADER));
	tfd.end_addr =(ili_fw->fw_len - ILI_FILE_HEADER);
}

static void ilitek_tddi_fw_ili_convert_inx(u8 *pfw)
{
	int i = 0, block_enable = 0, num = 0;
	u8 block;
	u32 Addr;
	ipio_info("Start to parse ILI file, type = %d, block_count = %d\n", CTPM_FW[32], CTPM_FW[33]);
	memset(fbi, 0x0, sizeof(fbi));

	tfd.start_addr = 0;
	tfd.end_addr = 0;
	tfd.hex_tag = 0;
	block_enable = CTPM_FW[32];

	if (block_enable == 0) {
		tfd.hex_tag = BLOCK_TAG_AE;
		goto out;
	}

	tfd.hex_tag = BLOCK_TAG_AF;
	for (i = 0; i < FW_BLOCK_INFO_NUM; i++) {
		if (((block_enable >> i) & 0x01) == 0x01) {
			num = i + 1;
			if ((num) == 6) {
				fbi[num].start = (CTPM_FW[0] << 16) + (CTPM_FW[1] << 8) + (CTPM_FW[2]);
				fbi[num].end = (CTPM_FW[3] << 16) + (CTPM_FW[4] << 8) + (CTPM_FW[5]);
				fbi[num].fix_mem_start = INT_MAX;
			} else {
				fbi[num].start = (CTPM_FW[34 + i * 6] << 16) + (CTPM_FW[35 + i * 6] << 8) + (CTPM_FW[36 + i * 6]);
				fbi[num].end = (CTPM_FW[37 + i * 6] << 16) + (CTPM_FW[38 + i * 6] << 8) + (CTPM_FW[39 + i * 6]);
				fbi[num].fix_mem_start = INT_MAX;
			}
			fbi[num].len = fbi[num].end - fbi[num].start + 1;
			ipio_info("Block[%d]: start_addr = %x, end = %x\n", num, fbi[num].start, fbi[num].end);
		}
	}

	if ((block_enable & 0x80) == 0x80) {
		for (i = 0; i < 3; i++) {
			Addr = (CTPM_FW[6 + i * 4] << 16) + (CTPM_FW[7 + i * 4] << 8) + (CTPM_FW[8 + i * 4]);
			block = CTPM_FW[9 + i * 4];

			if ((block != 0) && (Addr != 0x000000)) {
				fbi[block].fix_mem_start = Addr;
				ipio_info("Tag 0xB0: change Block[%d] to addr = 0x%x\n", block, fbi[block].fix_mem_start);
			}
		}
	}

out:
	tfd.block_number = CTPM_FW[33];
	memcpy(pfw, CTPM_FW + ILI_FILE_HEADER, (ili_fw->fw_len - ILI_FILE_HEADER));
	tfd.end_addr = (ili_fw->fw_len - ILI_FILE_HEADER);
}


static int ilitek_tddi_fw_hex_convert(u8 *phex, int size, u8 *pfw)
{
	int block = 0;
	u32 i = 0, j = 0, k = 0, num = 0;
	u32 len = 0, addr = 0, type = 0;
	u32 start_addr = 0x0, end_addr = 0x0, ex_addr = 0;
	u32 offset, hex_crc, data_crc;

	memset(fbi, 0x0, sizeof(fbi));

	/* Parsing HEX file */
	for (; i < size;) {
		len = HexToDec(&phex[i + 1], 2);
		addr = HexToDec(&phex[i + 3], 4);
		type = HexToDec(&phex[i + 7], 2);

		if (type == 0x04) {
			ex_addr = HexToDec(&phex[i + 9], 4);
		} else if (type == 0x02) {
			ex_addr = HexToDec(&phex[i + 9], 4);
			ex_addr = ex_addr >> 12;
		} else if (type == BLOCK_TAG_AE || type == BLOCK_TAG_AF) {
			/* insert block info extracted from hex */
			tfd.hex_tag = type;
			if (tfd.hex_tag == BLOCK_TAG_AF)
				num = HexToDec(&phex[i + 9 + 6 + 6], 2);
			else
				num = block;

			fbi[num].start = HexToDec(&phex[i + 9], 6);
			fbi[num].end = HexToDec(&phex[i + 9 + 6], 6);
			fbi[num].fix_mem_start = INT_MAX;
			fbi[num].len = fbi[num].end - fbi[num].start + 1;
			ipio_info("Block[%d]: start_addr = %x, end = %x", num, fbi[num].start, fbi[num].end);

			block++;
		} else if (type == BLOCK_TAG_B0 && tfd.hex_tag == BLOCK_TAG_AF) {
			num = HexToDec(&phex[i + 9 + 6], 2);
			fbi[num].fix_mem_start = HexToDec(&phex[i + 9], 6);
			ipio_info("Tag 0xB0: change Block[%d] to addr = 0x%x\n", num, fbi[num].fix_mem_start);
		}

		addr = addr + (ex_addr << 16);

		if (phex[i + 1 + 2 + 4 + 2 + (len * 2) + 2] == 0x0D)
			offset = 2;
		else
			offset = 1;

		if (addr > MAX_HEX_FILE_SIZE) {
			ipio_err("Invalid hex format %d\n", addr);
			return -1;
		}

		if (type == 0x00) {
			end_addr = addr + len;
			if (addr < start_addr)
				start_addr = addr;
			/* fill data */
			for (j = 0, k = 0; j < (len * 2); j += 2, k++)
				pfw[addr + k] = HexToDec(&phex[i + 9 + j], 2);
		}
		i += 1 + 2 + 4 + 2 + (len * 2) + 2 + offset;
	}

	/* Check the content of hex file by comparsing parsed data to the crc at last 4 bytes */
	for (i = 0; i < ARRAY_SIZE(fbi); i++) {
		if (fbi[i].end == 0)
			continue;
		ex_addr = fbi[i].end;
		data_crc = CalculateCRC32(fbi[i].start, fbi[i].len - 4, pfw);
		hex_crc = pfw[ex_addr - 3] << 24 | pfw[ex_addr - 2] << 16 | pfw[ex_addr - 1] << 8 | pfw[ex_addr];
		ipio_debug(DEBUG_FW, "data crc = %x, hex crc = %x\n", data_crc, hex_crc);
		if (data_crc != hex_crc) {
			ipio_err("Content of hex file is broken. (%d, %x, %x)\n",
				i, data_crc, hex_crc);
			return -1;
		}
	}

	ipio_info("Contect of hex file is correct\n");
	tfd.start_addr = start_addr;
	tfd.end_addr = end_addr;
	tfd.block_number = block;
	return 0;
}

static int ilitek_tdd_fw_hex_open(u8 open_file_method, u8 *pfw)
{
	int fsize = 1;
	u8 *hex_buffer = NULL;
	const struct firmware *fw = NULL;
	struct file *f = NULL;
	mm_segment_t old_fs;
	loff_t pos = 0;

#ifdef ODM_WT_EDIT
	if (sign_firmware == 0) {
		ipio_info("Open file method = %s, path = %s,sign_firmware = %d,ili_fw->oplus_bin = %s\n",
			open_file_method ? "FILP_OPEN" : "REQUEST_FIRMWARE", open_file_method ? ili_fw->filp_open_bin_name:ili_fw->firmware_bin_name,sign_firmware,ili_fw->firmware_bin_name);
	} else {
		ipio_info("Open file method = %s, path = %s,sign_firmware = %d,ili_fw->oplus_sign_bin = %s\n",
			open_file_method ? "FILP_OPEN" : "REQUEST_FIRMWARE", open_file_method ? ili_fw->filp_open_bin_name:ili_fw->firmware_bin_name,sign_firmware,ili_fw->oplus_sign_bin);
	}
#endif
	switch (open_file_method) {
	case REQUEST_FIRMWARE:
		if(idev->need_request_fw) {
			if(sign_firmware){
				if (request_firmware_select(&fw, ili_fw->oplus_sign_bin, idev->dev) < 0){
					ipio_err("Rquest ili_fw->oplus_sign_bin :%s firmware failed\n",ili_fw->oplus_sign_bin);
					return -ENOMEM;
				}
				ipio_info("ilitek_tdd_fw_hex_open ili_fw->oplus_sign_bin:%s success",ili_fw->oplus_sign_bin);
			}else{
				if (request_firmware(&fw, ili_fw->firmware_bin_name, idev->dev) < 0){
					ipio_err("Rquest ili_fw->firmware_bin_name = %s firmware failed\n",ili_fw->firmware_bin_name);
					return -ENOMEM;
				}
				ipio_info("ilitek_tdd_fw_hex_open ili_fw->firmware_bin_name :%s",ili_fw->firmware_bin_name);
			}
			ipio_vfree((void **) & (idev->tp_firmware.data));
			idev->tp_firmware.size = 0;
			//new fw data buffer
			fsize = fw->size;
			idev->tp_firmware.size = fsize;
			ipio_info("fsize = %d\n", fsize);
			if (fsize <= 0) {
				ipio_err("The size of file is zero\n");
				release_firmware(fw);
				return -ENOMEM;
			}
			
			idev->tp_firmware.data = vmalloc(fsize);
			if (idev->tp_firmware.data == NULL) {
				ipio_info("kmalloc tp firmware data error\n");
			
				idev->tp_firmware.data = vmalloc(fsize);
				if (idev->tp_firmware.data == NULL) {
					ipio_info("retry kmalloc tp firmware data error\n");
					release_firmware(fw);
					return -ENOMEM;
				}
			}
			idev->need_request_fw = false;
			ipio_memcpy((u8*)idev->tp_firmware.data, fw->data, fsize * sizeof(*fw->data), fsize);
			release_firmware(fw);
			break;
		}
		else {
			ipio_info("read fw from buf\n");
			break;
		}
	case FILP_OPEN:
		f = filp_open(ili_fw->filp_open_bin_name, O_RDONLY, 0644);
		if (ERR_ALLOC_MEM(f)) {
			ipio_err("Failed to open the file at %ld.\n", PTR_ERR(f));
			return -ENOMEM;
		}

		fsize = f->f_inode->i_size;
		ipio_info("fsize = %d\n", fsize);
		if (fsize <= 0) {
			ipio_err("The size of file is invaild\n");
			filp_close(f, NULL);
			return -ENOMEM;
		}

		hex_buffer = vmalloc(fsize * sizeof(u8));
		if (ERR_ALLOC_MEM(hex_buffer)) {
			ipio_err("Failed to allocate hex_buffer memory, %ld\n", PTR_ERR(hex_buffer));
			filp_close(f, NULL);
			return -ENOMEM;
		}

		/* ready to map user's memory to obtain data by reading files */
		old_fs = get_fs();
		set_fs(get_ds());
		set_fs(KERNEL_DS);
		pos = 0;
		vfs_read(f, hex_buffer, fsize, &pos);
		set_fs(old_fs);
		filp_close(f, NULL);
		break;
	default:
		ipio_err("Unknown open file method, %d\n", open_file_method);
		break;
	}

	/* Convert hex and copy data from hex_buffer to pfw */
	if (ilitek_tddi_fw_hex_convert((u8*)idev->tp_firmware.data, idev->tp_firmware.size, pfw) < 0) {
		ipio_err("Convert hex file failed\n");
		return -1;
	}
	return 0;
}

static void ilitek_tddi_fw_update_tp_info(int ret)
{
	ipio_info("FW upgrade %s\n", (ret == UPDATE_PASS ? "PASS" : "FAIL"));

	if (ret == UPDATE_FAIL) {
		if (atomic_read(&idev->mp_stat)) {
			ipio_info("No need to erase data during mp test\n");
			return;
		}
		ipio_info("Erase all fw data\n");
		if (idev->fw_upgrade_mode == UPGRADE_IRAM) {
			ilitek_tddi_reset_ctrl(idev->reset);
		} else {
			ilitek_ice_mode_ctrl(ENABLE, OFF);
			ilitek_tddi_fw_flash_erase();
			ilitek_ice_mode_ctrl(DISABLE, OFF);
			ilitek_tddi_reset_ctrl(idev->reset);
		}
		return;
	}
	ilitek_tddi_ic_get_fw_ver();
	if(is_first_boot) {
	is_first_boot = false;
	ilitek_tddi_ic_get_core_ver();
	ilitek_tddi_ic_get_protocl_ver();
	ilitek_tddi_ic_get_tp_info();
	ilitek_tddi_ic_get_panel_info();
	}
}

int ilitek_tddi_fw_upgrade(int upgrade_type, int file_type, int open_file_method)
{
	int ret = 0, retry = 3;
	u8 *pfw = NULL;

	pfw = vmalloc(MAX_HEX_FILE_SIZE * sizeof(u8));
	if (ERR_ALLOC_MEM(pfw)) {
		ipio_err("Failed to allocate pfw memory, %ld\n", PTR_ERR(pfw));
		ret = -ENOMEM;
		goto out;
	}

	memset(pfw, 0xFF, MAX_HEX_FILE_SIZE * sizeof(u8));

	ipio_info("Convert FW file from %s\n", (file_type == ILI_FILE ? "ILI_FILE" : "HEX_FILE"));

	if (idev->actual_tp_mode != P5_X_FW_GESTURE_MODE) {
		if (ilitek_tdd_fw_hex_open(open_file_method, pfw) < 0) {
			ipio_err("Open hex file fail, try upgrade from ILI file ili_ctpmodule = %d\n",ili_ctpmodule);
			switch (ili_ctpmodule){
				case 0 :
				case 1 :
					ilitek_tddi_fw_ili_convert_xl(pfw);
					ipio_info("load New panel ILI file");
					break;
				case 2 :
					ilitek_tddi_fw_ili_convert_inx(pfw);
					ipio_info("load old panel ILI file");
					break;
				default:
					ipio_info("load panel ILI file failed");
					ili_ctpmodule = -1;
					break;
			}
			if(ili_ctpmodule == -1){
				goto out;
			}
		}
			ilitek_tddi_fw_update_block_info(pfw, upgrade_type);
	}

	do {
		if (upgrade_type == UPGRADE_FLASH)
			ret = ilitek_tddi_fw_flash_upgrade(pfw);
		else
			ret = ilitek_tddi_fw_iram_upgrade(pfw);

		if (ret == UPDATE_PASS)
			break;
	} while (--retry > 0);

	if (ret != UPDATE_PASS) {
		ipio_err("Upgrade firmware failed after retry 3 times\n");
		ret = UPDATE_FAIL;
	}
	if (idev->actual_tp_mode == P5_X_FW_DEMO_MODE) {
		if (idev->direction == 0) {
			ilitek_tddi_ic_func_ctrl("edge_palm", 0x1);
		} else if (idev->direction == 1) {
			ilitek_tddi_ic_func_ctrl("edge_palm", 0x2);
		} else if (idev->direction == 2) {
			ilitek_tddi_ic_func_ctrl("edge_palm", 0x0);
		}
	}
	ilitek_tddi_fw_update_tp_info(ret);
out:
	ipio_vfree((void **)&pfw);
	return ret;
}

struct flash_table {
	u16 mid;
	u16 dev_id;
	int mem_size;
	int program_page;
	int sector;
} flashtab[] = {
	[0] = {0x00, 0x0000, (256 * K), 256, (4 * K)}, /* Default */
	[1] = {0xEF, 0x6011, (128 * K), 256, (4 * K)}, /* W25Q10EW	*/
	[2] = {0xEF, 0x6012, (256 * K), 256, (4 * K)}, /* W25Q20EW	*/
	[3] = {0xC8, 0x6012, (256 * K), 256, (4 * K)}, /* GD25LQ20B */
	[4] = {0xC8, 0x6013, (512 * K), 256, (4 * K)}, /* GD25LQ40 */
	[5] = {0x85, 0x6013, (4 * M), 256, (4 * K)},
	[6] = {0xC2, 0x2812, (256 * K), 256, (4 * K)},
	[7] = {0x1C, 0x3812, (256 * K), 256, (4 * K)},
};

void ilitek_tddi_fw_read_flash_info(bool mode)
{
	int i = 0;
	u8 buf[4] = {0};
	u8 cmd = 0x9F;
	u16 flash_id = 0, flash_mid = 0;

	if (mode == UPGRADE_IRAM)
		return;

	ilitek_ice_mode_ctrl(ENABLE, OFF);

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write(FLASH2_ADDR, cmd, 1);

	for (i = 0; i < ARRAY_SIZE(buf); i++) {
		ilitek_ice_mode_write(FLASH2_ADDR, 0xFF, 1);
		buf[i] = ilitek_ice_mode_read(FLASH4_ADDR, sizeof(u8));
	}

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

	flash_mid = buf[0];
	flash_id = buf[1] << 8 | buf[2];

	for (i = 0; i < ARRAY_SIZE(flashtab); i++) {
		if (flash_mid == flashtab[i].mid && flash_id == flashtab[i].dev_id) {
			idev->flash_mid = flashtab[i].mid;
			idev->flash_devid = flashtab[i].dev_id;
			idev->program_page = flashtab[i].program_page;
			idev->flash_sector = flashtab[i].sector;
			break;
		}
	}

	if (i >= ARRAY_SIZE(flashtab)) {
		ipio_info("Not found flash id in tab, use default\n");
		idev->flash_mid = flashtab[0].mid;
		idev->flash_devid = flashtab[0].dev_id;
		idev->program_page = flashtab[0].program_page;
		idev->flash_sector = flashtab[0].sector;
	}

	ipio_info("Flash MID = %x, Flash DEV_ID = %x\n", idev->flash_mid, idev->flash_devid);
	ipio_info("Flash program page = %d\n", idev->program_page);
	ipio_info("Flash sector = %d\n", idev->flash_sector);

	ilitek_tddi_flash_protect(DISABLE);
	ilitek_ice_mode_ctrl(DISABLE, OFF);
}
