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

#define DMA_TRANSFER_MAX_TIMES		2
#define DMA_TRANSFER_MAX_SIZE		1024
#define SPI_TX_BUF_SIZE			4096
#define SPI_RX_BUF_SIZE			4096


/* plus 5 for IC Mode :(Head + Address) 0x82,0x25,Addr_L,Addr_M,Addr_H */
#define SPI_WRITE_BUFF_MAXSIZE		(1024 * DMA_TRANSFER_MAX_TIMES + 5)
#define SPI_READ_BUFF_MAXSIZE		(1024 * DMA_TRANSFER_MAX_TIMES)

struct touch_bus_info {
	struct spi_driver bus_driver;
	struct ilitek_hwif_info *hwif;
};

struct ilitek_tddi_dev *idev;
extern struct ili_gesture_info * gesture_report_data;

/*
 * As spi_write_then_read() in kernel can't guarante the data we
 * want to send to or read from is always 4 bytes aligned via DMA transimission.
 *
 * This function works for that and limits the buffer of DMA is at 1024byte. You
 * can change it by request.
 */
#if 0//def CONFIG_MTK_SPI
static int core_mtk_spi_write_then_read(struct spi_device *spi,
		const void *txbuf, unsigned n_tx,
		void *rxbuf, unsigned n_rx)
{
	static DEFINE_MUTEX(lock);

	int status = -1;
	int xfercnt = 0, xferlen = 0, xferloop = 0;
	u8 *dma_txbuf = NULL, *dma_rxbuf = NULL;
	u8 cmd, temp1[1] = {0}, temp2[1] = {0};
	struct spi_message	message;
	struct spi_transfer	xfer[DMA_TRANSFER_MAX_TIMES + 1];

	if (n_tx > (SPI_WRITE_BUFF_MAXSIZE)) {
		ipio_err("Exceeded length (%d) > %d\n", n_tx, SPI_WRITE_BUFF_MAXSIZE);
		goto out;
	}
	if (n_rx > (SPI_READ_BUFF_MAXSIZE)) {
		ipio_err("Exceeded length (%d) > %d\n", n_rx, SPI_READ_BUFF_MAXSIZE);
		goto out;
	}

	dma_txbuf = kzalloc(SPI_WRITE_BUFF_MAXSIZE, GFP_KERNEL);
	if (ERR_ALLOC_MEM(dma_txbuf)) {
		ipio_err("Failed to allocate dma_txbuf, %ld\n", PTR_ERR(dma_txbuf));
		goto out;
	}

	dma_rxbuf = kzalloc(SPI_READ_BUFF_MAXSIZE, GFP_KERNEL);
	if (ERR_ALLOC_MEM(dma_rxbuf)) {
		ipio_err("Failed to allocate dma_rxbuf, %ld\n", PTR_ERR(dma_rxbuf));
		goto out;
	}

	mutex_trylock(&lock);

	spi_message_init(&message);
	memset(xfer, 0, sizeof(xfer));

	if ((n_tx == 1) && (n_rx == 1))
		cmd = SPI_READ;
	else
		cmd = *((u8 *)txbuf);

	switch (cmd) {
	case SPI_WRITE:
		if (n_tx % DMA_TRANSFER_MAX_SIZE)
			xferloop = (n_tx / DMA_TRANSFER_MAX_SIZE) + 1;
		else
			xferloop = n_tx / DMA_TRANSFER_MAX_SIZE;

		xferlen = n_tx;
		memcpy(dma_txbuf, (u8 *)txbuf, xferlen);

		for (xfercnt = 0; xfercnt < xferloop; xfercnt++) {
			if (xferlen > DMA_TRANSFER_MAX_SIZE)
				xferlen = DMA_TRANSFER_MAX_SIZE;

			xfer[xfercnt].len = xferlen;
			xfer[xfercnt].tx_buf = dma_txbuf + xfercnt * DMA_TRANSFER_MAX_SIZE;
			spi_message_add_tail(&xfer[xfercnt], &message);

			xferlen = n_tx - (xfercnt+1) * DMA_TRANSFER_MAX_SIZE;
		}
		status = spi_sync(spi, &message);
		break;
	case SPI_READ:
		/* for write cmd and head */
		memcpy(dma_txbuf, (u8 *)txbuf, n_tx);
		xfer[0].len = n_tx;
		xfer[0].tx_buf = dma_txbuf;
		xfer[0].rx_buf = temp1;
		spi_message_add_tail(&xfer[0], &message);

		/* for read data */
		if (n_rx % DMA_TRANSFER_MAX_SIZE)
			xferloop = (n_rx / DMA_TRANSFER_MAX_SIZE) + 1;
		else
			xferloop = n_rx / DMA_TRANSFER_MAX_SIZE;

		xferlen = n_rx;
		for (xfercnt = 0; xfercnt < xferloop; xfercnt++) {
			if (xferlen > DMA_TRANSFER_MAX_SIZE)
				xferlen = DMA_TRANSFER_MAX_SIZE;

			xfer[xfercnt+1].len = xferlen;
			xfer[xfercnt+1].tx_buf = temp2;
			xfer[xfercnt+1].rx_buf = dma_rxbuf + xfercnt * DMA_TRANSFER_MAX_SIZE;
			spi_message_add_tail(&xfer[xfercnt+1], &message);

			xferlen = n_rx - (xfercnt+1) * DMA_TRANSFER_MAX_SIZE;
		}
		status = spi_sync(spi, &message);
		if (status == 0)
			memcpy((u8 *)rxbuf, dma_rxbuf, n_rx);
		break;
	default:
		ipio_info("Unknown command 0x%x\n", cmd);
		break;
	}

	mutex_unlock(&lock);
out:
	ipio_kfree((void **)&dma_txbuf);
	ipio_kfree((void **)&dma_rxbuf);
	return status;
}
#else
int core_mtk_spi_write_then_read(struct spi_device *spi,
		const void *txbuf, unsigned n_tx,
		void *rxbuf, unsigned n_rx)
{
	int status = -1;
	int duplex_len = 0;
	u8 cmd;
	struct spi_message	message;
	struct spi_transfer	xfer;

	spi_message_init(&message);
	memset(&xfer, 0, sizeof(xfer));

	if ((n_tx > 0) && (n_rx > 0))
		cmd = SPI_READ;
	else
		cmd = SPI_WRITE;

	switch (cmd) {
	case SPI_WRITE:
		xfer.len = n_tx;
		xfer.tx_buf = txbuf;
		spi_message_add_tail(&xfer, &message);
		status = spi_sync(spi, &message);
		break;
	case SPI_READ:
		duplex_len = n_tx + n_rx;
		if ((duplex_len > SPI_TX_BUF_SIZE) ||
			(duplex_len > SPI_RX_BUF_SIZE)) {
			ipio_err("duplex_len is over than dma buf, abort\n");
			status = -ENOMEM;
			break;
		}

		memset(idev->spi_tx, 0x0, SPI_TX_BUF_SIZE);
		memset(idev->spi_rx, 0x0, SPI_RX_BUF_SIZE);

		xfer.len = duplex_len;
		memcpy(idev->spi_tx, txbuf, n_tx);
		xfer.tx_buf = idev->spi_tx;
		xfer.rx_buf = idev->spi_rx;

		spi_message_add_tail(&xfer, &message);
		status = spi_sync(spi, &message);
		if (status != 0)
			break;

		memcpy((u8 *)rxbuf, &idev->spi_rx[1], n_rx);
		break;
	default:
		ipio_info("Unknown command 0x%x\n", cmd);
		break;
	}

	if (status != 0)
		ipio_err("spi transfer failed\n");

	return status;
}

#endif /* CONFIG_MTK_SPI */

static int core_rx_lock_check(int *ret_size)
{
	int i, count = 1;
	u8 txbuf[5] = {SPI_WRITE, 0x25, 0x94, 0x0, 0x2};
	u8 rxbuf[4] = {0};
	u16 status = 0, lock = 0x5AA5;

	for (i = 0; i < count; i++) {
		txbuf[0] = SPI_WRITE;
		if (idev->spi_write_then_read(idev->spi, txbuf, 5, txbuf, 0) < 0) {
			ipio_err("spi write (0x25,0x94,0x0,0x2) error\n");
			goto out;
		}

		txbuf[0] = SPI_READ;
		if (idev->spi_write_then_read(idev->spi, txbuf, 1, rxbuf, 4) < 0) {
			ipio_err("spi read error\n");
			goto out;
		}

		status = (rxbuf[2] << 8) + rxbuf[3];
		*ret_size = (rxbuf[0] << 8) + rxbuf[1];

		ipio_debug(DEBUG_SPI, "Rx lock = 0x%x, size = %d\n", status, *ret_size);

		if (status == lock)
			return 0;

	}

out:
	ipio_err("Rx check lock error, lock = 0x%x, size = %d\n", status, *ret_size);
	return -EIO;
}

static int core_tx_unlock_check(void)
{
	int i, count = 100;
	u8 txbuf[5] = {SPI_WRITE, 0x25, 0x0, 0x0, 0x2};
	u8 rxbuf[4] = {0};
	u16 status = 0, unlock = 0x9881;

	for (i = 0; i < count; i++) {
		txbuf[0] = SPI_WRITE;
		if (idev->spi_write_then_read(idev->spi, txbuf, 5, txbuf, 0) < 0) {
			ipio_err("spi write (0x25,0x0,0x0,0x2) error\n");
			goto out;
		}

		txbuf[0] = SPI_READ;
		if (idev->spi_write_then_read(idev->spi, txbuf, 1, rxbuf, 4) < 0) {
			ipio_err("spi read error\n");
			goto out;
		}

		status = (rxbuf[2] << 8) + rxbuf[3];

		ipio_debug(DEBUG_SPI, "Tx unlock = 0x%x\n", status);

		if (status == unlock)
			return 0;

		mdelay(1);
	}

out:
	ipio_err("Tx check unlock error, unlock = 0x%x\n", status);
	return -EIO;
}

static int core_spi_ice_mode_unlock_read(u8 *data, int size)
{
	int ret = 0;
	u8 txbuf[64] = { 0 };

	/* set read address */
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x98;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;
	if (idev->spi_write_then_read(idev->spi, txbuf, 5, txbuf, 0) < 0) {
		ipio_info("spi write (0x25,0x98,0x00,0x2) error\n");
		ret = -EIO;
		return ret;
	}

	/* read data */
	txbuf[0] = SPI_READ;
	if (idev->spi_write_then_read(idev->spi, txbuf, 1, data, size) < 0) {
		ret = -EIO;
		return ret;
	}

	/* write data unlock */
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x94;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;
	txbuf[5] = (size & 0xFF00) >> 8;
	txbuf[6] = size & 0xFF;
	txbuf[7] = (char)0x98;
	txbuf[8] = (char)0x81;
	if (idev->spi_write_then_read(idev->spi, txbuf, 9, txbuf, 0) < 0) {
		ipio_err("spi write unlock (0x9881) error, ret = %d\n", ret);
		ret = -EIO;
	}
	return ret;
}

static int core_spi_ice_mode_lock_write(u8 *data, int size)
{
	int ret = 0;
	int safe_size = size;
	u8 check_sum = 0, wsize = 0;
	u8 *txbuf = NULL;

	txbuf = kcalloc(size + 9, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(txbuf)) {
		ipio_err("Failed to allocate txbuf\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Write data */
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x4;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;

	/* Calcuate checsum and fill it in the last byte */
	check_sum = ilitek_calc_packet_checksum(data, size);
	ipio_memcpy(txbuf + 5, data, size, safe_size + 9);
	txbuf[5 + size] = check_sum;
	size++;
	wsize = size;
	if (wsize % 4 != 0)
		wsize += 4 - (wsize % 4);

	if (idev->spi_write_then_read(idev->spi, txbuf, wsize + 5, txbuf, 0) < 0) {
		ipio_info("spi write (0x25,0x4,0x00,0x2) error\n");
		ret = -EIO;
		goto out;
	}

	/* write data lock */
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x0;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;
	txbuf[5] = (size & 0xFF00) >> 8;
	txbuf[6] = size & 0xFF;
	txbuf[7] = (char)0x5A;
	txbuf[8] = (char)0xA5;
	if (idev->spi_write_then_read(idev->spi, txbuf, 9, txbuf, 0) < 0) {
		ipio_err("spi write lock (0x5AA5) error, ret = %d\n", ret);
		ret = -EIO;
	}

out:
	ipio_kfree((void **)&txbuf);
	return ret;
}

static int core_spi_ice_mode_disable(void)
{
	u8 txbuf[5] = {0x82, 0x1B, 0x62, 0x10, 0x18};

	if (idev->spi_write_then_read(idev->spi, txbuf, 5, txbuf, 0) < 0) {
		ipio_err("spi write ice mode disable failed\n");
		return -EIO;
	}
	return 0;
}

static int core_spi_ice_mode_enable(void)
{
	u8 txbuf[5] = {0x82, 0x1F, 0x62, 0x10, 0x18};
	u8 rxbuf[2] = {0};

	if (idev->spi_write_then_read(idev->spi, txbuf, 1, rxbuf, 1) < 0) {
		ipio_err("spi write 0x82 error\n");
		return -EIO;
	}

	/* check recover data */
	if (rxbuf[0] != SPI_ACK) {
		ipio_err("Check SPI_ACK failed (0x%x)\n", rxbuf[0]);
		return DO_SPI_RECOVER;
	}

	if (idev->spi_write_then_read(idev->spi, txbuf, 5, rxbuf, 0) < 0) {
		ipio_err("spi write ice mode enable failed\n");
		return -EIO;
	}
	return 0;
}

static int core_spi_ice_mode_write(u8 *data, int len)
{
	int ret = 0;
	ret = core_spi_ice_mode_enable();
	if (ret < 0)
		return ret;

	/* send data and change lock status to 0x5AA5. */
	ret = core_spi_ice_mode_lock_write(data, len);
	if (ret < 0)
		goto out;

	/*
	 * Check FW if they already received the data we sent.
	 * They change lock status from 0x5AA5 to 0x9881 if they did.
	 */
	ret = core_tx_unlock_check();
	if (ret < 0)
		goto out;

out:
	if (core_spi_ice_mode_disable() < 0)
		return -EIO;

	return ret;
}

static int core_spi_ice_mode_read(u8 *data, int len)
{
	int size = 0, ret = 0;

	ret = core_spi_ice_mode_enable();
	if (ret < 0) {
		return ret;
	}
	/*
	 * Check FW if they already send their data to rxbuf.
	 * They change lock status from 0x9881 to 0x5AA5 if they did.
	 */
	ret = core_rx_lock_check(&size);
	if (ret < 0)
		goto out;

	if (len < size) {
		ipio_info("WARRING! size(%d) > len(%d), use len to get data\n", size, len);
		size = len;
	}

	/* receive data from rxbuf and change lock status to 0x9881. */
	ret = core_spi_ice_mode_unlock_read(data, size);
	if (ret < 0)
		goto out;

out:
	if (core_spi_ice_mode_disable() < 0)
		return -EIO;

	if (ret >= 0)
		return size;

	return ret;
}
int core_spi_upgrade_write(struct spi_device *spi,
		const void *txbuf, unsigned n_tx,
		void *rxbuf, unsigned n_rx)
{
	static DEFINE_MUTEX(lock);

	int status = -1;
	u8 cmd;
	struct spi_message	message;
	struct spi_transfer xfer[2];

	ipio_info("n_tx = %d\n", n_tx);

	mutex_trylock(&lock);

	spi_message_init(&message);
	memset(xfer, 0, sizeof(xfer));

	if ((n_tx == 1) && (n_rx == 1))
		cmd = SPI_READ;
	else
		cmd = *((u8 *)txbuf);

	switch (cmd) {
	case SPI_WRITE:

        xfer[0].len = n_tx;
        xfer[0].tx_buf = txbuf;
        spi_message_add_tail(&xfer[0], &message);

		status = spi_sync(spi, &message);
		break;
	default:
		ipio_info("Unknown command 0x%x\n", cmd);
		break;
	}

	mutex_unlock(&lock);
	return status;
}

static int core_spi_write(u8 *data, int len)
{
	int ret = 0, count = 2;
	u8 *txbuf = NULL;
	int safe_size = len;

	u8 wakeup[10] = {0x82, 0x25, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3};
	
	/* if system is suspended, wake up our spi pll clock before communication. */
	if (idev->suspend && idev->spi_gesture_cmd) {
		   ipio_info("Write dummy cmd to wake up spi pll clock\n");
		   if (idev->spi_write_then_read(idev->spi, wakeup, sizeof(wakeup), NULL, 0) < 0) {
				   ipio_err("spi write wake up cmd failed\n");
				   return -EIO;
		   }
	}
	


	if (atomic_read(&idev->ice_stat) == DISABLE) {
		do {
			ret = core_spi_ice_mode_write(data, len);
			if (ret >= 0)
				break;
		} while (--count > 0);
		goto out;
	}

	txbuf = kcalloc(len + 1, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(txbuf)) {
		ipio_err("Failed to allocate txbuf\n");
		return -ENOMEM;
	}

	txbuf[0] = SPI_WRITE;
	ipio_memcpy(txbuf+1, data, len, safe_size + 1);

	if (idev->spi_write_then_read(idev->spi, txbuf, len+1, txbuf, 0) < 0) {
		ipio_err("spi write data error in ice mode\n");
		ret = -EIO;
		goto out;
	}

out:
	ipio_kfree((void **)&txbuf);
	return ret;
}
EXPORT_SYMBOL(core_spi_write);

int core_spi_check_read_size(void)
{
	int size = 0;
	int count = 1;
	int ret = 0;
	if (atomic_read(&idev->ice_stat) == DISABLE) {
		do {
			ret = core_spi_ice_mode_enable();
			if (ret < 0) {
				continue;
			}
			ret = core_rx_lock_check(&size);
			if (ret < 0) {
				core_spi_ice_mode_disable();
				continue;
			}
			if(size > 0) {
				return size;
			}
			} while (--count > 0);
	}
	return ret;
}

int core_spi_read_data_after_checksize(uint8_t *pBuf, uint16_t nSize)
{
	int res = 0;
		res = core_spi_ice_mode_unlock_read(pBuf, nSize);
		if (res < 0) {
			core_spi_ice_mode_disable();
			return res;
		}
		if (core_spi_ice_mode_disable() < 0) {
			return -EIO;
		}
		return res;
}

static int core_spi_read(u8 *rxbuf, int len)
{
	int ret = 0, count = 5;
	u8 txbuf[1] = {0};

	txbuf[0] = SPI_READ;

	if (atomic_read(&idev->ice_stat) == DISABLE) {
		do {
			ret = core_spi_ice_mode_read(rxbuf, len);
			if (ret >= 0)
				break;
		} while (--count > 0);
		goto out;
	}

	if (idev->spi_write_then_read(idev->spi, txbuf, 1, rxbuf, len) < 0) {
		ipio_err("spi read data error in ice mode\n");
		ret = -EIO;
		goto out;
	}

out:
	return ret;
}

static int ilitek_spi_write(void *buf, int len)
{
	int ret = 0;

	if (!len) {
		ipio_err("spi write len is invaild\n");
		return -EINVAL;
	}

//	mutex_lock(&idev->io_mutex);

	ret = core_spi_write(buf, len);
	if (ret < 0) {
		if (atomic_read(&idev->tp_reset) == START) {
			ret = 0;
			goto out;
		}
		ipio_err("spi write error, ret = %d\n", ret);
	}

out:
//	mutex_unlock(&idev->io_mutex);
	return ret;
}

/* If ilitek_spi_read success ,this function will return read length */
static int ilitek_spi_read(void *buf, int len)
{
	int ret = 0;

	if (!len) {
		ipio_err("spi read len is invaild\n");
		return -EINVAL;
	}

//	mutex_lock(&idev->io_mutex);

	ret = core_spi_read(buf, len);
	if (ret < 0) {
		if (atomic_read(&idev->tp_reset) == START) {
			ret = 0;
			goto out;
		}
		ipio_err("spi read error, ret = %d\n", ret);
	}

out:
//	mutex_unlock(&idev->io_mutex);
	return ret;
}

static int core_spi_setup(u32 freq)
{
#ifdef CONFIG_MTK_SPI
	struct mt_chip_conf *chip_config;
	u32 temp_pulse_width = 0;

	chip_config = (struct mt_chip_conf *)idev->spi->controller_data;
	if (!chip_config) {
		ipio_err("chip_config is NULL.\n");
		chip_config = kzalloc(sizeof(struct mt_chip_conf), GFP_KERNEL);
		if (!chip_config)
			return -ENOMEM;
	}

	temp_pulse_width = ((112 * 1000000) / freq);
	temp_pulse_width = temp_pulse_width / 2;

	chip_config->setuptime = temp_pulse_width * 2;// for CS
	chip_config->holdtime = temp_pulse_width * 2;// for CS
	chip_config->high_time = temp_pulse_width;// for CLK = 1M
	chip_config->low_time = temp_pulse_width;// for CLK= 1M
	chip_config->cs_idletime = temp_pulse_width * 2;// for CS
	chip_config->rx_mlsb = 1;
	chip_config->tx_mlsb = 1;
	chip_config->tx_endian = 0;
	chip_config->rx_endian = 0;
	chip_config->cpol = 0;
	chip_config->cpha = 0;
	chip_config->com_mod = DMA_TRANSFER;
	//chip_config->com_mod = FIFO_TRANSFER;
	chip_config->pause = 1;
	chip_config->finish_intr = 1;
	chip_config->deassert = 0;

	idev->spi->controller_data = chip_config;
	idev->spi_write_then_read = core_mtk_spi_write_then_read;
#else
	idev->spi_write_then_read = core_mtk_spi_write_then_read;
#endif /* CONFIG_MTK_SPI */

	ipio_info("spi clock = %d\n", freq);

	idev->spi->mode = SPI_MODE_0;
	idev->spi->bits_per_word = 8;
	idev->spi->max_speed_hz = freq;

	if (spi_setup(idev->spi) < 0) {
		ipio_err("Failed to setup spi device\n");
		return -ENODEV;
	}

	ipio_info("name = %s, bus_num = %d,cs = %d, mode = %d, speed = %d\n",
			idev->spi->modalias,
			idev->spi->master->bus_num,
			idev->spi->chip_select,
			idev->spi->mode,
			idev->spi->max_speed_hz);
	return 0;
}

static int ilitek_spi_probe(struct spi_device *spi)
{
	struct touch_bus_info *info =
	container_of(to_spi_driver(spi->dev.driver),
		struct touch_bus_info, bus_driver);

	ipio_info("ilitek spi probe\n");

	if (!spi) {
		ipio_err("spi device is NULL\n");
		return -ENODEV;
	}
	gesture_report_data = kzalloc(sizeof(struct ili_gesture_info), GFP_KERNEL);
	if (ERR_ALLOC_MEM(gesture_report_data)) {
		ipio_err("Failed to allocate gesture_report_data mem, %ld\n", PTR_ERR(gesture_report_data));
		return -ENOMEM;
	}
	idev = devm_kzalloc(&spi->dev, sizeof(struct ilitek_tddi_dev), GFP_KERNEL);
	if (ERR_ALLOC_MEM(idev)) {
		ipio_err("Failed to allocate idev memory, %ld\n", PTR_ERR(idev));
		if (gesture_report_data != NULL) {
			ipio_kfree((void **)&gesture_report_data);
		}
		return -ENOMEM;
	}
    idev->fw_buf_dma = kzalloc(128 * 1024, GFP_KERNEL | GFP_DMA);
    if (idev->fw_buf_dma == NULL) {
        ipio_err("fw kzalloc error\n");
        return -ENOMEM;
    }
	idev->spi_tx = kzalloc(SPI_TX_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(idev->spi_tx)) {
		ipio_err("Failed to allocate spi tx buffer\n");
		return -ENOMEM;
	}

	idev->spi_rx = kzalloc(SPI_RX_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(idev->spi_rx)) {
		ipio_err("Failed to allocate spi rx buffer\n");
		return -ENOMEM;
	}
	idev->i2c = NULL;
	idev->spi = spi;
	idev->dev = &spi->dev;
	idev->hwif = info->hwif;
	idev->phys = "SPI";

	idev->write = ilitek_spi_write;
	idev->read = ilitek_spi_read;

	idev->spi_speed = ilitek_tddi_ic_spi_speed_ctrl;
	idev->actual_tp_mode = P5_X_FW_DEMO_MODE;
	idev->presure_speed = 60;
	idev->need_request_fw = true;
	if (TDDI_RST_BIND)
		idev->reset = TP_IC_WHOLE_RST;
	else
		idev->reset = TP_HW_RST_ONLY;

	idev->rst_edge_delay = 5;
	idev->fw_open = REQUEST_FIRMWARE;
	idev->fw_upgrade_mode = UPGRADE_IRAM;
	idev->mp_move_code = ilitek_tddi_move_mp_code_iram;
	idev->gesture_move_code = ilitek_tddi_move_gesture_code_iram;
	idev->esd_recover = ilitek_tddi_wq_esd_spi_check;
	idev->ges_recover = ilitek_tddi_touch_esd_gesture_iram;
	idev->gesture_mode = P5_X_FW_GESTURE_INFO_MODE;
	idev->wtd_ctrl = ON;
	idev->report = ENABLE;
	idev->netlink = DISABLE;
	idev->debug_node_open = DISABLE;

	if (ENABLE_GESTURE){
		idev->gesture = ENABLE;
		tp_gesture = 1;
	}
	core_spi_setup(SPI_CLK);
	return info->hwif->plat_probe();
}

static int ilitek_spi_remove(struct spi_device *spi)
{
	struct touch_bus_info *info =
	container_of(to_spi_driver(spi->dev.driver),
		struct touch_bus_info, bus_driver);
	kfree(idev->spi_tx);
	kfree(idev->spi_rx);

	ipio_info();
	spi_unregister_driver(&info->bus_driver);
	return info->hwif->plat_remove();
}

static const struct spi_device_id tp_spi_id[] = {
	{TDDI_DEV_ID, 0},
	{},
};

int ilitek_tddi_interface_dev_init(struct ilitek_hwif_info *hwif)
{
	struct touch_bus_info *info;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		ipio_err("faied to allocate spi_driver\n");
		return -ENOMEM;
	}

	if (hwif->bus_type != BUS_SPI) {
		ipio_err("incorrect interface\n");
		return -EINVAL;
	}

	hwif->info = info;

	info->bus_driver.driver.name = hwif->name;
	info->bus_driver.driver.owner = hwif->owner;
	info->bus_driver.driver.of_match_table = hwif->of_match_table;

	info->bus_driver.probe = ilitek_spi_probe;
	info->bus_driver.remove = ilitek_spi_remove;
	info->bus_driver.id_table = tp_spi_id;

	info->hwif = hwif;
	return spi_register_driver(&info->bus_driver);
}
