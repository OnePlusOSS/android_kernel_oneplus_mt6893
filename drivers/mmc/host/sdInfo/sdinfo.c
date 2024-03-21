/****************************************************************
 ** Copyright (C), 2021-2030, OPLUS Mobile Comm Corp., Ltd.
 ** File: - sdinfo.c
 ** Description: record sdcard information
 ** Version: 1.0
 ** Date : 2021-07-13
 ** Author:
 **
 ** ----------------------Revision History: --------------------
 **  <author>      <date>      <version>      <desc>
 **                2021-07-13  1.0            sdcard info
 ****************************************************************/

#ifdef OPLUS_FEATURE_SDCARD_INFO

#include <linux/types.h>
#include <linux/device.h>
#include <linux/mmc/card.h>
#include "sdinfo.h"

#define UNSTUFF_BITS(resp, start, size)					\
	({								\
		const int __size = size;				\
		const u32 __mask = (__size < 32 ? 1 << __size : 0) - 1;	\
		const int __off = 3 - ((start) / 32);			\
		const int __shft = (start) & 31;			\
		u32 __res;						\
									\
		__res = resp[__off] >> __shft;				\
		if (__size + __shft > 32)				\
			__res |= resp[__off-1] << ((32 - __shft) % 32);	\
		__res & __mask;						\
	})

struct sd_info sdinfo;
static int remove_sdcard = 0;
static struct delayed_work remove_sdcard_reset_work;
static int set_dma_data_timeout = 0;

static void sdcard_remove_reset(struct work_struct *work)
{
	remove_sdcard = 0;
	pr_err("mmc reset sdcard_remove\n");
}

ssize_t sdcard_remove_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "echo 1 > /sys/devices/platform/soc/externdevice/sdcard_remove\nremove_sdcard=%d\n", remove_sdcard);
}

ssize_t sdcard_remove_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u32 value;

	value = (u32)buf[0] - 48;

	pr_err("mmc sdcard_remove value: %u\n", value);
	if (value) {
		switch(value) {
		case SET_SDCARD_QUICK_RETURN :
			remove_sdcard = value;
			sdinfo.vold_timeout_count += 1;
			break;
		case RESET_SDCARD_QUICK_RETURN :
			schedule_delayed_work(&remove_sdcard_reset_work, msecs_to_jiffies(2000));
			break;
		case SET_DMA_DATA_TIMEOUT :
			set_dma_data_timeout = 1;
			break;
		case RESET_DMA_DATA_TIMEOUT :
			set_dma_data_timeout = 0;
			break;
		default :
			pr_err("mmc sdinfo: value(%d) error\n", value);
		}
	} else
		remove_sdcard = 0;

	return count;
}

static struct device_attribute sdcard_remove_attr;
void sdcard_remove_attr_init_sysfs(struct device *dev)
{
	sdcard_remove_attr.show = sdcard_remove_show;
	sdcard_remove_attr.store = sdcard_remove_store;
	sysfs_attr_init(&sdcard_remove_attr.attr);
	sdcard_remove_attr.attr.name = "sdcard_remove";
	sdcard_remove_attr.attr.mode = 0644;
	if (device_create_file(dev, &sdcard_remove_attr))
		dev_err(dev, "Failed to create sysfs for sdcard_remove_attr\n");

	INIT_DELAYED_WORK(&remove_sdcard_reset_work, sdcard_remove_reset);
}

void reset_sdinfo(void)
{
        char *tmp = (char*)&sdinfo;
        int len = sizeof(sdinfo);

        while (len--)
                *(tmp+len) = 0;
	remove_sdcard = 0;
	set_dma_data_timeout = 0;
}

void set_sdinfo(struct mmc_card *card)
{
	sdinfo.manfid = card->cid.manfid;
	sdinfo.csd_version = UNSTUFF_BITS(card->raw_csd, 126, 2);
	sdinfo.capacity = card->csd.capacity;
	sdinfo.supported_bus_mode = card->sw_caps.sd3_bus_mode & (SD_MODE_UHS_SDR104 | SD_MODE_UHS_SDR50 | SD_MODE_UHS_DDR50 | SD_MODE_UHS_SDR25 | SD_MODE_UHS_SDR12);
	sdinfo.sd_bus_speed = card->sd_bus_speed;
	sdinfo.taac_ns = card->csd.taac_ns;
	sdinfo.taac_clks = card->csd.taac_clks;
	sdinfo.r2w_factor = card->csd.r2w_factor;
}

int get_sdcard_remove(void)
{
	return remove_sdcard;
}

int get_dma_data_timeout(void)
{
	return set_dma_data_timeout;
}

#endif
