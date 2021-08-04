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

#ifdef ODM_WT_EDIT
#include <linux/hardware_info.h>
//extern char Ctp_name[HARDWARE_MAX_ITEM_LONGTH];
extern int ili_ctpmodule;
extern unsigned char* CTPM_FW;
#endif

#ifdef ODM_WT_EDIT
	char ili_version[20] = {"0"};
#endif

#ifdef ODM_WT_EDIT
extern void devinfo_info_tp_set(char *version, char *manufacture, char *fw_path);

#endif

#define PROTOCL_VER_NUM		7
static struct ilitek_protocol_info protocol_info[PROTOCL_VER_NUM] = {
	/* length -> fw, protocol, tp, key, panel, core, func, window, cdc, mp_info */
	[0] = {PROTOCOL_VER_500, 4, 4, 14, 30, 5, 5, 2, 8, 3, 8},
	[1] = {PROTOCOL_VER_510, 4, 3, 14, 30, 5, 5, 3, 8, 3, 8},
	[2] = {PROTOCOL_VER_520, 4, 4, 14, 30, 5, 5, 3, 8, 3, 8},
	[3] = {PROTOCOL_VER_530, 9, 4, 14, 30, 5, 5, 3, 8, 3, 8},
	[4] = {PROTOCOL_VER_540, 9, 4, 14, 30, 5, 5, 3, 8, 15, 8},
	[5] = {PROTOCOL_VER_550, 9, 4, 14, 30, 5, 5, 3, 8, 15, 14},
	[6] = {PROTOCOL_VER_560, 9, 4, 14, 30, 5, 5, 3, 8, 15, 14},
};
static bool first_in_ice = false;
static char Ctp_name[64];
#define FUNC_CTRL_NUM	32
static struct ilitek_ic_func_ctrl func_ctrl[FUNC_CTRL_NUM] = {
	/* cmd[3] = cmd, func, ctrl */
	[0] = {"sense",        {0x1, 0x1,  0x0}, 3},
	[1] = {"sleep",        {0x1, 0x2,  0x0}, 3},
	[2] = {"glove",        {0x1, 0x6,  0x0}, 3},
	[3] = {"stylus",       {0x1, 0x7,  0x0}, 3},
	[4] = {"tp_scan_mode", {0x1, 0x8,  0x0}, 3},
	[5] = {"lpwg",         {0x1, 0xA,  0x0}, 3},
	[6] = {"gesture",      {0x1, 0xB,  0x3F},3},
	[7] = {"phone_cover",  {0x1, 0xC,  0x0}, 3},
	[8] = {"finger_sense", {0x1, 0xF,  0x0}, 3},
	[9] = {"proximity",    {0x1, 0x10, 0x0}, 3},
	[10] = {"plug",        {0x1, 0x11, 0x0}, 3},
	[11] = {"lock_point",  {0x1, 0x13, 0x0}, 3},
	[12] = {"game_switch", {0x1, 0x13, 0x0}, 3},
	[13] = {"headset",     {0x1, 0x17, 0x0}, 3},
	[14] = {"edge_palm",   {0x1, 0x12, 0x01},3},//01: ˙∆¡£ª02:∫·∆¡90∂» 00:∫·∆¡270∂»
	[15] = {"phone_cover_window", {0xE, 0x0, 0x0}, 3},
	[16] = {"knock_en", {0x1, 0xA, 0x8, 0x03, 0x0, 0x0}, 6},
	[17] = {"hopping_ctrl", {0x1, 0x15, 0x0}, 3},
};
#define CHIP_SUP_NUM		6
static u32 ic_sup_list[CHIP_SUP_NUM] = {
	[0] = ILI9881_CHIP,
	[1] = ILI9881H_AD,
	[2] = ILI9881H_AE,
	[3] = ILI7807_CHIP,
	[4] = ILI7807G_AA,
	[5] = ILI7807G_AB,
};

static int ilitek_tddi_ic_check_support(u32 pid, u16 id)
{
	int i = 0;

	for (i = 0; i < CHIP_SUP_NUM; i++) {
		if ((pid == ic_sup_list[i]) || (id == ic_sup_list[i]))
		break;
	}

	if (i >= CHIP_SUP_NUM) {
		ipio_info("ERROR, ILITEK CHIP (%x, %x) Not found !!\n", pid, id);
		return -1;
	}

	ipio_info("ILITEK CHIP (%x, %x) found.\n", pid, id);

	if (id == ILI9881_CHIP) {
		idev->chip->reset_key = 0x00019881;
		idev->chip->wtd_key = 0x9881;
		idev->chip->open_sp_formula = open_sp_formula_ili9881;
		idev->chip->hd_dma_check_crc_off = firmware_hd_dma_crc_off_ili9881;

		/*
		 * Since spi speed has been enabled previsouly whenever enter to ICE mode,
		 * we have to disable if find out the ic is ili9881.
		 */
		if (idev->spi_speed != NULL && idev->chip->spi_speed_ctrl) {
			idev->spi_speed(OFF);
			idev->chip->spi_speed_ctrl = DISABLE;
		}

		if (pid == ILI9881F_AA)
			idev->chip->no_bk_shift = RAWDATA_NO_BK_SHIFT_9881F;
		else
			idev->chip->no_bk_shift = RAWDATA_NO_BK_SHIFT_9881H;
	} else {
		idev->chip->reset_key = 0x00019878;
		idev->chip->wtd_key = 0x9878;
		idev->chip->open_sp_formula = open_sp_formula_ili7807;
		idev->chip->hd_dma_check_crc_off = firmware_hd_dma_crc_off_ili7807;
		idev->chip->no_bk_shift = RAWDATA_NO_BK_SHIFT_9881H;
	}

	idev->chip->max_count = 0x1FFFF;
	idev->chip->open_c_formula = open_c_formula;
	return 0;
}

int ilitek_ice_mode_bit_mask_write(u32 addr, u32 mask, u32 value)
{
	int ret = 0;
	u32 data = 0;

	data = ilitek_ice_mode_read(addr, sizeof(u32));

	data &= (~mask);
	data |= (value & mask);

	ipio_debug(DEBUG_IC, "mask value data = %x\n", data);

	ret = ilitek_ice_mode_write(addr, data, sizeof(u32));
	if (ret < 0)
		ipio_err("Failed to re-write data in ICE mode, ret = %d\n", ret);

	return ret;
}

int ilitek_ice_mode_write(u32 addr, u32 data, int len)
{
	int ret = 0, i;
	u8 txbuf[64] = {0};

	if (!atomic_read(&idev->ice_stat)) {
		ipio_err("ice mode not enabled\n");
		return -1;
	}

	txbuf[0] = 0x25;
	txbuf[1] = (char)((addr & 0x000000FF) >> 0);
	txbuf[2] = (char)((addr & 0x0000FF00) >> 8);
	txbuf[3] = (char)((addr & 0x00FF0000) >> 16);

	for (i = 0; i < len; i++)
		txbuf[i + 4] = (char)(data >> (8 * i));

	ret = idev->write(txbuf, len + 4);
	if (ret < 0)
		ipio_err("Failed to write data in ice mode, ret = %d\n", ret);

	return ret;
}

u32 ilitek_ice_mode_read(u32 addr, int len)
{
	u32 ret = 0;
	u8 *rxbuf = NULL;
	u8 txbuf[4] = {0};

	if (!atomic_read(&idev->ice_stat)) {
		ipio_err("ice mode not enabled\n");
		return -1;
	}

	txbuf[0] = 0x25;
	txbuf[1] = (char)((addr & 0x000000FF) >> 0);
	txbuf[2] = (char)((addr & 0x0000FF00) >> 8);
	txbuf[3] = (char)((addr & 0x00FF0000) >> 16);

	ret = idev->write(txbuf, 4);
	if (ret < 0)
		goto out;

	rxbuf = kcalloc(len, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(rxbuf)) {
		ipio_err("Failed to allocate rxbuf, %ld\n", PTR_ERR(rxbuf));
		ret = -ENOMEM;
		goto out;
	}

	ret = idev->read(rxbuf, len);
	if (ret < 0)
		goto out;

	if (len == sizeof(u8))
		ret = rxbuf[0];
	else
		ret = (rxbuf[0] | rxbuf[1] << 8 | rxbuf[2] << 16 | rxbuf[3] << 24);

out:
	if (ret < 0)
		ipio_err("Failed to read data in ice mode, ret = %d\n", ret);

	ipio_kfree((void **)&rxbuf);
	return ret;
}

int ilitek_ice_mode_ctrl(bool enable, bool mcu)
{
	int ret = 0, retry = 3;
	u8 cmd_open[4] = {0x25, 0x62, 0x10, 0x18};
	u8 cmd_close[4] = {0x1B, 0x62, 0x10, 0x18};
	u32 pid;

	ipio_info("%s ICE mode, mcu on = %d\n", (enable ? "Enable" : "Disable"), mcu);

	if (enable) {
		if (atomic_read(&idev->ice_stat)) {
			ipio_info("ice mode already enabled\n");
			return 0;
		}

		if (mcu)
			cmd_open[0] = 0x1F;

		atomic_set(&idev->ice_stat, ENABLE);

		do {
			ret = idev->write(cmd_open, sizeof(cmd_open));
			if (ret < 0)
				continue;
			if (idev->spi_speed != NULL && idev->chip->spi_speed_ctrl)
				idev->spi_speed(ON);

			/* Read chip id to ensure that ice mode is enabled successfully */
			pid = ilitek_ice_mode_read(idev->chip->pid_addr, sizeof(u32));
			ret = ilitek_tddi_ic_check_support(pid, pid >> 16);
			if (ret == 0)
				break;
			} while (--retry > 0);

		if (retry <= 0) {
			ipio_err("Enter to ICE Mode failed !!\n");
			atomic_set(&idev->ice_stat, DISABLE);
			ret = -1;
			goto out;
		}

		/* Patch to resolve the issue of i2c nack after exit to ice mode */
		ilitek_ice_mode_write(0x47002, 0x00, 1);
	} else {
		if (!atomic_read(&idev->ice_stat)) {
			ipio_info("ice mode already disabled\n");
			return 0;
		}

		ret = idev->write(cmd_close, sizeof(cmd_close));
		if (ret < 0)
			ipio_err("Exit to ICE Mode failed !!\n");
		atomic_set(&idev->ice_stat, DISABLE);
	}
out:
	return ret;
}

int ilitek_tddi_ic_watch_dog_ctrl(bool write, bool enable)
{
	int timeout = 50, ret = 0;

	if (!atomic_read(&idev->ice_stat)) {
		ipio_err("ice mode wasn't enabled\n");
		return -1;
	}

	if (idev->chip->wdt_addr <= 0 || idev->chip->id <= 0) {
		ipio_err("WDT/CHIP ID is invalid\n");
		return -EINVAL;
	}

	/* FW will automatiacally disable WDT in I2C */
	if (idev->wtd_ctrl == OFF) {
		ipio_info("WDT ctrl is off, do nothing\n");
		return 0;
	}

	if (!write) {
		ret = ilitek_ice_mode_read(idev->chip->wdt_addr, sizeof(u8));
		ipio_info("Read WDT: %s\n", (ret ? "ON" : "OFF"));
		return ret;
	}

	ipio_info("%s WDT, key = %x\n", (enable ? "Enable" : "Disable"), idev->chip->wtd_key);

	if (enable) {
		ilitek_ice_mode_write(idev->chip->wdt_addr, 1, 1);
	} else {
		ilitek_ice_mode_write(idev->chip->wdt_addr, (idev->chip->wtd_key & 0xff), 1);
		ilitek_ice_mode_write(idev->chip->wdt_addr, (idev->chip->wtd_key >> 8), 1);
		/* need to delay 300us after stop mcu to wait fw relaod */
		udelay(300);
	}

	while (timeout > 0) {
		ret = ilitek_ice_mode_read(TDDI_WDT_ACTIVE_ADDR, sizeof(u8));
		ipio_debug(DEBUG_IC, "ret = %x\n", ret);
		if (enable) {
			if (ret == TDDI_WDT_ON)
				break;
		} else {
			if (ret == TDDI_WDT_OFF)
				break;

			/* If WDT can't be disabled, try to command and wait to see */
			ilitek_ice_mode_write(idev->chip->wdt_addr, 0x00, 1);
			ilitek_ice_mode_write(idev->chip->wdt_addr, 0x98, 1);
		}
		timeout--;
		mdelay(5);
	}

	if (timeout <= 0) {
		ipio_err("WDT turn on/off timeout !, ret = %x, pc = 0x%x\n",
				ret, ilitek_tddi_ic_get_pc_counter());
		return -EINVAL;
	}

	if (enable) {
		ipio_info("WDT turn on succeed\n");
	} else {
		ipio_info("WDT turn off succeed\n");
		ilitek_ice_mode_write(idev->chip->wdt_addr, 0, 1);
	}
	return 0;
}

int ilitek_tddi_ic_func_ctrl(const char *name, int ctrl)
{
	int i = 0;

	for (i = 0; i < FUNC_CTRL_NUM; i++) {
		if (strncmp(name, func_ctrl[i].name, strlen(name)) == 0) {
			if (strlen(name) != strlen(func_ctrl[i].name))
				continue;
			break;
		}
	}

	if (i >= FUNC_CTRL_NUM) {
		ipio_err("Not found function ctrl, %s\n", name);
		return -1;
	}

	if (idev->protocol->ver == PROTOCOL_VER_500) {
		ipio_err("Non support function ctrl with protocol v5.0\n");
		return -1;
	}

	if (idev->protocol->ver >= PROTOCOL_VER_560) {
		if (strncmp(func_ctrl[i].name, "gesture", strlen("gesture")) == 0 ||
			strncmp(func_ctrl[i].name, "phone_cover_window", strlen("phone_cover_window")) == 0) {
			ipio_info("Non support %s function ctrl\n", func_ctrl[i].name);
			return -1;
		}
	}

	func_ctrl[i].cmd[2] = ctrl;

	ipio_info("func = %s, len = %d, cmd = 0x%x, 0x%x, 0x%x\n", func_ctrl[i].name, func_ctrl[i].len,
		func_ctrl[i].cmd[0], func_ctrl[i].cmd[1], func_ctrl[i].cmd[2]);

	return idev->write(func_ctrl[i].cmd, func_ctrl[i].len);;
}

int ilitek_tddi_ic_code_reset(void)
{
	int ret;
	bool ice = atomic_read(&idev->ice_stat);

	if (!ice)
		ilitek_ice_mode_ctrl(ENABLE, OFF);

	ret = ilitek_ice_mode_write(0x40040, 0xAE, 1);
	if (ret < 0)
		ipio_err("ic code reset failed\n");

	if (!ice)
		ilitek_ice_mode_ctrl(DISABLE, OFF);
	return ret;
}

int ilitek_tddi_ic_whole_reset(void)
{
	ipio_info("ic whole reset key = 0x%x, edge_delay = %d\n",
		idev->chip->reset_key, idev->rst_edge_delay);

	if (ilitek_ice_mode_write(idev->chip->reset_key,
				idev->chip->reset_addr,
				sizeof(u32)) < 0) {
		ipio_err("ic whole reset failed\n");
		return -1;
	}
	msleep(idev->rst_edge_delay);
	return 0;
}

static void ilitek_tddi_ic_wr_pack(int packet)
{
	int retry = 100;
	u32 reg_data = 0;

	while (retry--) {
		reg_data = ilitek_ice_mode_read(0x73010, sizeof(u8));
		if ((reg_data & 0x02) == 0) {
			ipio_info("check ok 0x73010 read 0x%X retry = %d\n", reg_data, retry);
			break;
		}
		mdelay(10);
	}

	if (retry <= 0)
		ipio_info("check 0x73010 error read 0x%X\n", reg_data);

	ilitek_ice_mode_write(0x73000, packet, 4);
}

static u32 ilitek_tddi_ic_rd_pack(int packet)
{
	int retry = 100;
	u32 reg_data = 0;

	ilitek_tddi_ic_wr_pack(packet);

	while (retry--) {
		reg_data = ilitek_ice_mode_read(0x4800A, sizeof(u8));
		if ((reg_data & 0x02) == 0x02) {
			ipio_info("check  ok 0x4800A read 0x%X retry = %d\n", reg_data, retry);
			break;
		}
		mdelay(10);
	}
	if (retry <= 0)
		ipio_info("check 0x4800A error read 0x%X\n", reg_data);

	ilitek_ice_mode_write(0x4800A, 0x02, 1);
	reg_data = ilitek_ice_mode_read(0x73016, sizeof(u32));
	return reg_data;
}

void ilitek_tddi_ic_set_ddi_reg_onepage(u8 page, u8 reg, u8 data)
{
	int wdt;
	u32 setpage = 0x1FFFFF00 | page;
	u32 setreg = 0x1F000100 | (reg << 16) | data;
	bool ice = atomic_read(&idev->ice_stat);

	ipio_info("setpage =  0x%X setreg = 0x%X\n", setpage, setreg);

	if (!ice)
		ilitek_ice_mode_ctrl(ENABLE, OFF);

	wdt = ilitek_tddi_ic_watch_dog_ctrl(ILI_READ, DISABLE);
	if (wdt)
		ilitek_tddi_ic_watch_dog_ctrl(ILI_WRITE, DISABLE);

	/*TDI_WR_KEY*/
	ilitek_tddi_ic_wr_pack(0x1FFF9527);
	/*Switch to Page*/
	ilitek_tddi_ic_wr_pack(setpage);
	/* Page*/
	ilitek_tddi_ic_wr_pack(setreg);
	/*TDI_WR_KEY OFF*/
	ilitek_tddi_ic_wr_pack(0x1FFF9500);

	if (wdt)
		ilitek_tddi_ic_watch_dog_ctrl(ILI_WRITE, ENABLE);

	if (!ice)
		ilitek_ice_mode_ctrl(DISABLE, OFF);
}

void ilitek_tddi_ic_get_ddi_reg_onepage(u8 page, u8 reg)
{
	int wdt;
	u32 reg_data = 0;
	u32 setpage = 0x1FFFFF00 | page;
	u32 setreg = 0x2F000100 | (reg << 16);
	bool ice = atomic_read(&idev->ice_stat);

	ipio_info("setpage = 0x%X setreg = 0x%X\n", setpage, setreg);

	if (!ice)
		ilitek_ice_mode_ctrl(ENABLE, OFF);

	wdt = ilitek_tddi_ic_watch_dog_ctrl(ILI_READ, DISABLE);
	if (wdt)
		ilitek_tddi_ic_watch_dog_ctrl(ILI_WRITE, DISABLE);

	/*TDI_WR_KEY*/
	ilitek_tddi_ic_wr_pack(0x1FFF9527);
	/*Set Read Page reg*/
	ilitek_tddi_ic_wr_pack(setpage);

	/*TDI_RD_KEY*/
	ilitek_tddi_ic_wr_pack(0x1FFF9487);
	/*( *( __IO uint8 *)	(0x4800A) ) =0x2*/
	ilitek_ice_mode_write(0x4800A, 0x02, 1);

	reg_data = ilitek_tddi_ic_rd_pack(setreg);
	ipio_info("check page = 0x%X, reg = 0x%X, read 0x%X\n", page, reg, reg_data);

	/*TDI_RD_KEY OFF*/
	ilitek_tddi_ic_wr_pack(0x1FFF9400);
	/*TDI_WR_KEY OFF*/
	ilitek_tddi_ic_wr_pack(0x1FFF9500);

	if (wdt)
		ilitek_tddi_ic_watch_dog_ctrl(ILI_WRITE, ENABLE);

	if (!ice)
		ilitek_ice_mode_ctrl(DISABLE, OFF);
}

void ilitek_tddi_ic_check_otp_prog_mode(void)
{
	int prog_mode, prog_done, retry = 5;

	if (!idev->do_otp_check)
		return;

	if (ilitek_ice_mode_ctrl(ENABLE, OFF) < 0) {
		ipio_err("enter ice mode failed in otp\n");
		return;
	}

	if (ilitek_tddi_ic_watch_dog_ctrl(ILI_WRITE, DISABLE) < 0) {
		ipio_err("disable WDT failed in otp\n");
		return;
	}

	do {
		ilitek_ice_mode_write(0x43008, 0x80, 1);
		ilitek_ice_mode_write(0x43030, 0x0, 1);
		ilitek_ice_mode_write(0x4300C, 0x4, 1);

		mdelay(1);

		ilitek_ice_mode_write(0x4300C, 0x4, 1);

		prog_done = ilitek_ice_mode_read(0x43030, sizeof(u8));
		prog_mode = ilitek_ice_mode_read(0x43008, sizeof(u8));
		ipio_info("otp prog_mode = 0x%x, prog_done = 0x%x\n", prog_mode, prog_done);

		if (prog_done == 0x0 && prog_mode == 0x80)
			break;
	} while (--retry > 0);

	if (retry <= 0)
		ipio_err("OTP Program mode error!\n");
}

void ilitek_tddi_ic_spi_speed_ctrl(bool enable)
{
	ipio_info("%s spi speed up\n", (enable ? "Enable" : "Disable"));

	if (enable) {
		ilitek_ice_mode_write(0x063820, 0x00000101, 4);
		ilitek_ice_mode_write(0x042c34, 0x00000008, 4);
		ilitek_ice_mode_write(0x063820, 0x00000000, 4);
	} else {
		ilitek_ice_mode_write(0x063820, 0x00000101, 4);
		ilitek_ice_mode_write(0x042c34, 0x00000000, 4);
		ilitek_ice_mode_write(0x063820, 0x00000000, 4);
	}
}

u32 ilitek_tddi_ic_get_pc_counter(void)
{
	bool ice = atomic_read(&idev->ice_stat);
	u32 pc = 0;

	if (!ice)
		ilitek_ice_mode_ctrl(ENABLE, OFF);

	pc = ilitek_ice_mode_read(idev->chip->pc_counter_addr, sizeof(u32));

	ipio_info("pc counter = 0x%x\n", pc);

	if (!ice)
		ilitek_ice_mode_ctrl(DISABLE, OFF);

	return pc;
}

int ilitek_tddi_ic_check_int_stat(void)
{
	int timer = 5000;

	/* From FW request, timeout should at least be 5 sec */
	while (--timer > 0) {
		if (atomic_read(&idev->mp_int_check) == DISABLE)
			break;
		mdelay(1);
	}

	if (timer > 0) {
		ipio_info("Interrupt for MP is active\n");
		return 0;
	}

	ipio_err("Error! Interrupt for MP isn't received\n");
	atomic_set(&idev->mp_int_check, DISABLE);
	return -1;
}

int ilitek_tddi_ic_check_busy(int count, int delay)
{
	u8 cmd[2] = {0};
	u8 busy = 0, rby = 0;
	int ret = 0;
	cmd[0] = P5_X_READ_DATA_CTRL;
	cmd[1] = P5_X_CDC_BUSY_STATE;

	if (idev->actual_tp_mode == P5_X_FW_DEMO_MODE)
		rby = 0x41;
	else if (idev->actual_tp_mode == P5_X_FW_TEST_MODE)
		rby = 0x51;
	else {
		ipio_err("Unknown TP mode (0x%x)\n", idev->actual_tp_mode);
		return -EINVAL;
	}

	ipio_info("read byte = %x, delay = %d\n", rby, delay);

	do {
		mdelay(delay);
		
		ret = idev->write(cmd, sizeof(cmd));
		if ( ret < 0) {
			continue;
		}
		ret = idev->write(&cmd[1], sizeof(u8));
		if ( ret < 0) {
			continue;
		}

		ret = idev->read(&busy, sizeof(u8));
		if ( ret < 0) {
			continue;
		}

		ipio_debug(DEBUG_IC, "busy = 0x%x\n", busy);

		if (busy == rby) {
			ipio_info("Check busy free\n");
			return 0;
		}
		
	} while (--count > 0);

	ipio_err("Check busy (0x%x) timeout ! pc = 0x%x\n", busy,
		ilitek_tddi_ic_get_pc_counter());
	return -1;
}

int ilitek_tddi_ic_get_project_id(u8 *pdata, int size)
{
	int i;

	if (!pdata) {
		ipio_err("pdata is null\n");
		return -ENOMEM;
	}

	ipio_info("Read size = %d\n", size);

	if (ilitek_ice_mode_ctrl(ENABLE, OFF) < 0)
		return -1;

	ilitek_ice_mode_write(0x041000, 0x0, 1);   /* CS low */
	ilitek_ice_mode_write(0x041004, 0x66aa55, 3);  /* Key */

	ilitek_ice_mode_write(0x041008, 0x03, 1);

	ilitek_ice_mode_write(0x041008, (RESERVE_BLOCK_START_ADDR & 0xFF0000) >> 16, 1);
	ilitek_ice_mode_write(0x041008, (RESERVE_BLOCK_START_ADDR & 0x00FF00) >> 8, 1);
	ilitek_ice_mode_write(0x041008, (RESERVE_BLOCK_START_ADDR & 0x0000FF), 1);

	for (i = 0; i < size; i++) {
		ilitek_ice_mode_write(0x041008, 0xFF, 1);
		pdata[i] = ilitek_ice_mode_read(0x41010, sizeof(u8));
		ipio_info("project_id[%d] = 0x%x\n", i, pdata[i]);
	}

	ilitek_tddi_flash_clear_dma();

	ilitek_ice_mode_write(0x041000, 0x1, 1);   /* CS high */

	ilitek_ice_mode_ctrl(DISABLE, OFF);
	return 0;
}

int ilitek_tddi_ic_get_core_ver(void)
{
	u8 cmd[2] = {0};
	u8 buf[10] = {0};

	cmd[0] = P5_X_READ_DATA_CTRL;
	cmd[1] = P5_X_GET_CORE_VERSION;

	if (idev->write(cmd, sizeof(cmd)) < 0) {
		ipio_err("write core ver err\n");
		return -1;
	}

	if (idev->write(&cmd[1], sizeof(u8)) < 0) {
		ipio_err("write core ver err\n");
		return -1;
	}

	if (idev->read(buf, idev->protocol->core_ver_len) < 0) {
		ipio_err("i2c/spi read core ver err\n");
		return -1;
	}

	if (buf[0] != P5_X_GET_CORE_VERSION) {
		ipio_err("Invalid core ver\n");
		return -EINVAL;
	}

	ipio_info("Core version = %d.%d.%d.%d\n", buf[1], buf[2], buf[3], buf[4]);
	idev->chip->core_ver = buf[1] << 24 | buf[2] << 16 | buf[3] << 8 | buf[4];
	return 0;
}

int ilitek_tddi_ic_get_fw_ver(void)
{
	u8 cmd[2] = {0};
	u8 buf[10] = {0};
	cmd[0] = P5_X_READ_DATA_CTRL;
	cmd[1] = P5_X_GET_FW_VERSION;

	if (idev->write(cmd, sizeof(cmd)) < 0) {
		ipio_err("write firmware ver err\n");
		return -1;
	}

	if (idev->write(&cmd[1], sizeof(u8)) < 0) {
		ipio_err("write firmware ver err\n");
		return -1;
	}

	if (idev->read(buf, idev->protocol->fw_ver_len) < 0) {
		ipio_err("i2c/spi read firmware ver err\n");
		return -1;
	}

	if (buf[0] != P5_X_GET_FW_VERSION) {
		ipio_err("Invalid firmware ver\n");
		return -EINVAL;
	}

	ipio_info("Firmware version = %d.%d.%d.%d\n", buf[1], buf[2], buf[3], buf[4]);
	idev->chip->fw_ver = buf[1] << 24 | buf[2] << 16 | buf[3] << 8 | buf[4];
	#ifdef ODM_WT_EDIT
	if ( ili_ctpmodule == 0 ){
		sprintf(Ctp_name,"XL,ILI9881,FW:0x%x\n",buf[3]);
	}
	else if(ili_ctpmodule == 1){
		sprintf(Ctp_name,"TXD,ILI9881,FW:0x%x\n",buf[3]);
	}else{
		ipio_info("Not fund touchscreen !");
	}
#endif
#ifdef ODM_WT_EDIT
	if ( ili_ctpmodule == 0 ) {
	    sprintf(ili_version,"XL_Ili_%0x",buf[3]);
	    devinfo_info_tp_set(ili_version, "XL",OPLUS_FIRMWARE_XL_AUO_NAME_PATH);
	} else if(ili_ctpmodule == 1){
	    sprintf(ili_version,"TXD_Ili_%0x",buf[3]);
	    devinfo_info_tp_set(ili_version, "TXD",OPLUS_FIRMWARE_TXD_AUO_NAME_PATH);
	}else{
		ipio_info("Not fund touchscreen !");
	}
#endif
	return 0;
}

int ilitek_tddi_ic_get_panel_info(void)
{
	int ret = 0;
	u8 cmd = P5_X_GET_PANEL_INFORMATION;
	u8 buf[10] = {0};

	ret = idev->write(&cmd, sizeof(u8));
	if (ret < 0) {
		ipio_err("Write panel info error\n");
		goto out;
	}

	ret = idev->read(buf, idev->protocol->panel_info_len);
	if (ret < 0)
		ipio_err("Read panel info error\n");

out:

		idev->panel_wid = TOUCH_SCREEN_X_MAX;
		idev->panel_hei = TOUCH_SCREEN_Y_MAX;


	ipio_info("Panel info: width = %d, height = %d\n", idev->panel_wid, idev->panel_hei);
	return ret;
}

int ilitek_tddi_ic_get_tp_info(void)
{
	u8 cmd[2] = {0};
	u8 buf[20] = {0};

	cmd[0] = P5_X_READ_DATA_CTRL;
	cmd[1] = P5_X_GET_TP_INFORMATION;

	if (idev->write(cmd, sizeof(cmd)) < 0) {
		ipio_err("Write tp info error\n");
		return -1;
	}

	if (idev->write(&cmd[1], sizeof(u8)) < 0) {
		ipio_err("Write tp info error\n");
		return -1;
	}

	if (idev->read(buf, idev->protocol->tp_info_len) < 0) {
		ipio_err("Read tp info error\n");
		return -1;
	}

	if (buf[0] != P5_X_GET_TP_INFORMATION) {
		ipio_err("Invalid tp info\n");
		return -EINVAL;
	}

	idev->min_x = buf[1];
	idev->min_y = buf[2];
	idev->max_x = buf[4] << 8 | buf[3];
	idev->max_y = buf[6] << 8 | buf[5];
	idev->xch_num = buf[7];
	idev->ych_num = buf[8];
	idev->stx = buf[11];
	idev->srx = buf[12];

	ipio_info("TP Info: min_x = %d, min_y = %d, max_x = %d, max_y = %d\n", idev->min_x, idev->min_y, idev->max_x, idev->max_y);
	ipio_info("TP Info: xch = %d, ych = %d, stx = %d, srx = %d\n", idev->xch_num, idev->ych_num, idev->stx, idev->srx);
	return 0;
}

static void ilitek_tddi_ic_check_protocol_ver(u32 pver)
{
	int i = 0;

	if (idev->protocol->ver == pver) {
		ipio_info("same procotol version, do nothing\n");
		return;
	}

	for (i = 0; i < PROTOCL_VER_NUM - 1; i++) {
		if (protocol_info[i].ver == pver) {
			idev->protocol = &protocol_info[i];
			ipio_info("update protocol version = %x\n", idev->protocol->ver);
			return;
		}
	}

	ipio_info("Not found a correct protocol version in list, use newest version\n");
	idev->protocol = &protocol_info[PROTOCL_VER_NUM - 1];
}

int ilitek_tddi_edge_palm_ctrl(u8 type)
{
	u8 cmd[4] = { 0 };

	ipio_info("edge palm ctrl, type = %d\n", type);

	cmd[0] = P5_X_READ_DATA_CTRL;
	cmd[1] = P5_X_EDGE_PLAM_CTRL_1;
	cmd[2] = P5_X_EDGE_PLAM_CTRL_2;
	cmd[3] = type;

	if (idev->write(cmd, sizeof(cmd)) < 0) {
		ipio_err("Write edge plam ctrl error\n");
		return -1;
	}

	if (idev->write(&cmd[1], (sizeof(cmd) - 1)) < 0) {
		ipio_err("Write edge plam ctrl error\n");
		return -1;
	}
	return 0;
}

int ilitek_tddi_ic_get_protocl_ver(void)
{
	u8 cmd[2] = {0};
	u8 buf[10] = {0};
	u32 ver;

	cmd[0] = P5_X_READ_DATA_CTRL;
	cmd[1] = P5_X_GET_PROTOCOL_VERSION;

	if (idev->write(cmd, sizeof(cmd)) < 0) {
		ipio_err("Write protocol version error\n");
		return -1;
	}

	if (idev->write(&cmd[1], sizeof(u8)) < 0) {
		ipio_err("Write protocol version error\n");
		return -1;
	}

	if (idev->read(buf, idev->protocol->pro_ver_len) < 0) {
		ipio_err("Read protocol version error\n");
		return -1;
	}

	if (buf[0] != P5_X_GET_PROTOCOL_VERSION) {
		ipio_err("Invalid protocol ver\n");
		return -EINVAL;
	}

	ver = buf[1] << 16 | buf[2] << 8 | buf[3];

	ilitek_tddi_ic_check_protocol_ver(ver);

	ipio_info("Protocol version = %d.%d.%d\n", idev->protocol->ver >> 16,
		(idev->protocol->ver >> 8) & 0xFF, idev->protocol->ver & 0xFF);
	return 0;
}

int ilitek_tddi_ic_get_info(void)
{
	int ret = 0;
	bool ice = atomic_read(&idev->ice_stat);

	if (!ice)
		ilitek_ice_mode_ctrl(ENABLE, OFF);

	idev->chip->pid = ilitek_ice_mode_read(idev->chip->pid_addr, sizeof(u32));
	idev->chip->id = idev->chip->pid >> 16;
	idev->chip->type_hi = idev->chip->pid & 0x0000FF00;
	idev->chip->type_low = idev->chip->pid	& 0xFF;

	idev->chip->otp_id = ilitek_ice_mode_read(idev->chip->otp_addr, sizeof(u32));
	idev->chip->ana_id = ilitek_ice_mode_read(idev->chip->ana_addr, sizeof(u32));
	idev->chip->otp_id &= 0xFF;
	idev->chip->ana_id &= 0xFF;

	ipio_info("CHIP INFO: PID = %x, ID = %x, TYPE = %x, OTP = %x, ANA = %x\n",
		idev->chip->pid,
		idev->chip->id,
		((idev->chip->type_hi << 8) | idev->chip->type_low),
		idev->chip->otp_id,
		idev->chip->ana_id);

	ret = ilitek_tddi_ic_check_support(idev->chip->pid, idev->chip->id);

	if (!ice && first_in_ice)
		ilitek_ice_mode_ctrl(DISABLE, OFF);
	first_in_ice = true;
	return ret;
}

static struct ilitek_ic_info chip;

void ilitek_tddi_ic_init(void)
{
	chip.pid_addr =		   	TDDI_PID_ADDR;
	chip.wdt_addr =		   	TDDI_WDT_ADDR;
	chip.pc_counter_addr = 		TDDI_PC_COUNTER_ADDR;
	chip.otp_addr =		   	TDDI_OTP_ID_ADDR;
	chip.ana_addr =		   	TDDI_ANA_ID_ADDR;
	chip.reset_addr =	   	TDDI_CHIP_RESET_ADDR;

	idev->protocol = &protocol_info[PROTOCL_VER_NUM - 1];
	idev->chip = &chip;
}
