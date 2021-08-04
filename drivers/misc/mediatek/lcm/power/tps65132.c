/******************************************************************
** Copyright (C), 2004-2017, OPLUS Mobile Comm Corp., Ltd.
** File: - tps65132.c
** Description: Source file for lcd drvier.
** lcd driver including parameter and power control.
** Version: 1.0
** Date : 2017/05/06
** Author: Rongchun.Zhang@EXP.MultiMedia.Display.LCD.Machine
**
** ------------------------------- Revision History:---------------
** zhangrongchun 2017/05/06 1.0 build this module
*******************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif

#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif


#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif

#define lcm_power_I2C_BUSNUM  3
#define lcm_power_SLAVE_ADDR_WRITE	0x7C

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#define I2C_ID_NAME "I2C_LCD_BIAS"
#if defined(CONFIG_MTK_LEGACY)
static struct i2c_board_info __initdata tps65132_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, lcm_power_SLAVE_ADDR_WRITE>>1)};
#endif
static struct i2c_client *i2c_tps65132 = NULL;
static const struct i2c_device_id device_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};
#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id lcm_of_match[] = {
	{.compatible = "mediatek,I2C_LCD_BIAS"},
	{},
};
#endif
static int lcm_power_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lcm_power_remove(struct i2c_client *client);


static struct i2c_driver lcm_power_i2c_driver = {
	.id_table = device_id,
	.probe = lcm_power_probe,
	.remove = lcm_power_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "I2C_LCD_BIAS",
		#if !defined(CONFIG_MTK_LEGACY)
		.of_match_table = lcm_of_match,
		#endif
	},
};

static int lcm_power_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	i2c_tps65132 = client;
	printk("Tps65132 probe success !!\n");
	return 0;
}
static int lcm_power_remove(struct i2c_client *client)
{
	i2c_tps65132 = NULL;
	i2c_unregister_device(client);
	return 0;
}

int lcm_power_write_byte(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = i2c_tps65132;
	unsigned char write_data[2] = {0};

	if (!client) {
		printk("Tps65132 i2c client is NULL!\n");
		return -1;
	}
	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret<0) {
		printk("Tps65132 Write I2C fail !!\n");
	}
	return ret;
}
EXPORT_SYMBOL(lcm_power_write_byte);


static int __init lcm_power_i2c_init(void)
{
	#if defined(CONFIG_MTK_LEGACY)
	i2c_register_board_info(lcm_power_I2C_BUSNUM, &tps65132_board_info, 1);
	#endif
	i2c_add_driver(&lcm_power_i2c_driver);
	return 0;
}

static void __exit lcm_power_i2c_exit(void)
{
  i2c_del_driver(&lcm_power_i2c_driver);
}

module_init(lcm_power_i2c_init);
module_exit(lcm_power_i2c_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL");

#else /* BUILD_LK */

static struct mt_i2c_t i2c_lm3630a;

int lcm_power_write_byte(unsigned char addr, unsigned char value)
{
    kal_uint32 ret_code = I2C_OK;
    unsigned char write_data[2];
    kal_uint16 len;

    write_data[0] = addr;
    write_data[1] = value;

    i2c_lm3630a.id = lcm_power_I2C_BUSNUM;
    i2c_lm3630a.addr = (lcm_power_SLAVE_ADDR_WRITE >>1);
    i2c_lm3630a.mode = ST_MODE;
    i2c_lm3630a.speed = 100;
    len = 2;

    ret_code = i2c_write(&i2c_lm3630a, write_data, len);

    dprintf(0, "%s ret_code is %d\n", __func__, ret_code);

    return ret_code;
}

#endif /* BUILD_LK */
