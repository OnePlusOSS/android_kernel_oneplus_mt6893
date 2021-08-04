// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 . All rights reserved.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>

#ifdef CONFIG_OPLUS_CHARGER_MTK


#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/module.h>
//#include <upmu_common.h>
//nclude <mt-plat/mtk_gpio.h>
//#include <mtk_boot_common.h>
#include <mt-plat/mtk_rtc.h>
//#include <mt-plat/charging.h>
#include <mt-plat/charger_type.h>
#include <soc/oplus/device_info.h>

extern void mt_power_off(void); 
#else

#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/rtc.h>
#include <soc/oplus/device_info.h>


#endif

#include "../oplus_wrap.h"
#include "../oplus_gauge.h"
#include <oplus_da9313.h>
#include <linux/proc_fs.h>

static struct chip_da9313 *the_chip = NULL;
static DEFINE_MUTEX(da9313_i2c_access);

static int __da9313_read_reg(int reg, int *returnData)
{
    int ret = 0;
    int retry = 3;
    struct chip_da9313 *chip = the_chip;

    ret = i2c_smbus_read_byte_data(chip->client, reg);
    if (ret < 0) {
        while(retry > 0) {
            msleep(10);
            ret = i2c_smbus_read_byte_data(chip->client, reg);
            if (ret < 0) {
                retry--;
            } else {
                *returnData = ret;
                return 0;
            }
        }
        chg_err("i2c read fail: can't read from %02x: %d\n", reg, ret);
        return ret;
    } else {
        *returnData = ret;
    }

    return 0;
}

static int da9313_read_reg(int reg, int *returnData)
{
    int ret = 0;

    mutex_lock(&da9313_i2c_access);
    ret = __da9313_read_reg(reg, returnData);
    mutex_unlock(&da9313_i2c_access);
    return ret;
}

static int __da9313_write_reg(int reg, int val)
{
    int ret = 0;
    struct chip_da9313 *chip = the_chip;
	int retry = 3;

    ret = i2c_smbus_write_byte_data(chip->client, reg, val);

	if (ret < 0) {
		while(retry > 0) {
			usleep_range(5000, 5000);
			ret = i2c_smbus_write_byte_data(chip->client, reg, val);
			if (ret < 0) {
				retry--;
			} else {
				break;
			}
		}
	}

    if (ret < 0) {
        chg_err("i2c write fail: can't write %02x to %02x: %d\n",
        val, reg, ret);
        return ret;
    }

    return 0;
}

/**********************************************************
  *
  *   [Read / Write Function] 
  *
  *********************************************************/
/*
static int da9313_read_interface (int RegNum, int *val, int MASK, int SHIFT)
{
    int da9313_reg = 0;
    int ret = 0;

   //chg_err("--------------------------------------------------\n");
	
    ret = da9313_read_reg(RegNum, &da9313_reg);
	
   //chg_err(" Reg[%x]=0x%x\n", RegNum, da9313_reg);
	
    da9313_reg &= (MASK << SHIFT);
    *val = (da9313_reg >> SHIFT);
	
   //chg_err(" val=0x%x\n", *val);
	
    return ret;
}
*/

static int da9313_config_interface (int RegNum, int val, int MASK)
{
    int da9313_reg = 0;
    int ret = 0;

    mutex_lock(&da9313_i2c_access);
    ret = __da9313_read_reg(RegNum, &da9313_reg);

    if (ret >= 0) {
        da9313_reg &= ~MASK;
        da9313_reg |= val;

        if (RegNum == REG04_DA9313_ADDRESS) {
            if ((da9313_reg & 0x01) == 0 && the_chip->hwid == HWID_DA9313) {
                chg_err("[REG04_DA9313_ADDRESS] can't write 0 to bit0, da9313_reg[0x%x] bug here\n", da9313_reg);
                dump_stack();
                da9313_reg |= 0x01;
            }
        }
        ret = __da9313_write_reg(RegNum, da9313_reg);
    }
    __da9313_read_reg(RegNum, &da9313_reg);

    mutex_unlock(&da9313_i2c_access);

    return ret;
}

void da9313_dump_registers(void)
{
    int rc;
    int addr;

    unsigned int val_buf[DA9313_REG2_NUMBER] = {0x0};

    for (addr = DA9313_FIRST_REG; addr <= DA9313_LAST_REG; addr++) {
        rc = da9313_read_reg(addr, &val_buf[addr]);
        if (rc) {
            chg_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
        } else {
            pr_err("%s success addr = %d, value = 0x%x\n", 
            __func__, addr, val_buf[addr]);
        }
    }

    for (addr = DA9313_FIRST2_REG; addr <= DA9313_LAST2_REG; addr++) {
        rc = da9313_read_reg(addr, &val_buf[addr]);
        if (rc) {
            chg_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
        }
    }
}

static int da9313_work_mode_set(int work_mode)
{
    int rc = 0;
    struct chip_da9313 *divider_ic = the_chip;

    if (divider_ic->hwid != HWID_DA9313) {
        chg_err("%s: non da9313 chip skip to set work mode\n", __func__);
        return rc;
    }
    if(divider_ic == NULL) {
        chg_err("%s: da9313 driver is not ready\n", __func__);
        return rc;
    }
    if (atomic_read(&divider_ic->suspended) == 1) {
        return 0;
    }
    if(divider_ic->fixed_mode_set_by_dev_file == true 
       && work_mode == DA9313_WORK_MODE_AUTO) {
        chg_err("%s: maybe in high GSM work fixed mode ,return here\n", __func__);
        return rc;
    }

    if (oplus_wrap_get_allow_reading() == false) {
        return -1;
    }
    chg_err("%s: work_mode [%d]\n", __func__, work_mode);
    if(work_mode != 0) {
        rc = da9313_config_interface(REG04_DA9313_ADDRESS, REG04_DA9313_PVC_MODE_AUTO, REG04_DA9313_PVC_MODE_MASK);
    } else {
        rc = da9313_config_interface(REG04_DA9313_ADDRESS, REG04_DA9313_PVC_MODE_FIXED, REG04_DA9313_PVC_MODE_MASK);
    }
    return rc;
}

int oplus_set_divider_work_mode(int work_mode)
{
    return da9313_work_mode_set(work_mode);
}
EXPORT_SYMBOL(oplus_set_divider_work_mode);

int da9313_hardware_init(void)
{
    int rc = 0;
    struct chip_da9313 *divider_ic = the_chip;

    if(divider_ic == NULL) {
        chg_err("%s: da9313 driver is not ready\n", __func__);
        return 0;
    }
    if (atomic_read(&divider_ic->suspended) == 1) {
        return 0;
    }

    rc = da9313_config_interface(REG04_DA9313_ADDRESS, REG04_DA9313_PVC_MODE_AUTO, REG04_DA9313_PVC_MODE_MASK);
    da9313_dump_registers();
    return rc;
}

int max77932_hardware_init(void)
{
    int rc = 0;
    struct chip_da9313 *divider_ic = the_chip;

    if(divider_ic == NULL) {
        chg_err("%s: max77932 driver is not ready\n", __func__);
        return 0;
    }
    if (atomic_read(&divider_ic->suspended) == 1) {
        return 0;
    }

    da9313_config_interface(0x4, BIT(4), 0xFF);
    /*set frequency to 250M HZ*/
    da9313_config_interface(0x5, 0, (BIT(2) | BIT(1) | BIT(0)));
    /*set input ovp to 10.5V, default is 9.5V*/
    da9313_config_interface(0x6, BIT(5), (BIT(5) | BIT(4)));
    /*set output ovp to 5.5V, default is 5V*/
    da9313_config_interface(0x9, (BIT(4) | BIT(2) | BIT(1) | BIT(0)), (BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0)));

    return rc;
}

int oz1c313_hardware_init(void)
{
    int rc = 0;
    struct chip_da9313 *divider_ic = the_chip;

    if(divider_ic == NULL) {
        chg_err("%s: oz1c313 driver is not ready\n", __func__);
        return 0;
    }
    if (atomic_read(&divider_ic->suspended) == 1) {
        return 0;
    }

    rc = da9313_config_interface(REG04_DA9313_ADDRESS, REG04_DA9313_PVC_MODE_AUTO, REG04_DA9313_PVC_MODE_MASK);
    rc = da9313_config_interface(REG04_DA9313_ADDRESS, (BIT(3)), (BIT(4) | BIT(3)));
    return rc;
}


static int get_hwid(struct chip_da9313 *chip) {
    chip->hwid = HWID_DA9313;

    if (!chip) {
        chg_err("[OPLUS_CHG][%s]: chip_mp2650 not ready!\n", __func__);
        return chip->hwid;
    }

    if (chip->da9313_hwid_gpio <= 0) {
        chg_err("da9313_hwid_gpio not exist, return\n");
        return chip->hwid;
    }

    if (IS_ERR_OR_NULL(chip->pinctrl)
        || IS_ERR_OR_NULL(chip->da9313_hwid_active)
        || IS_ERR_OR_NULL(chip->da9313_hwid_sleep)
        || IS_ERR_OR_NULL(chip->da9313_hwid_default)) {
        chg_err("pinctrl null, return\n");
        return chip->hwid;
    }

    pinctrl_select_state(chip->pinctrl, chip->da9313_hwid_active);
    usleep_range(10000, 10000);
    if(gpio_get_value(chip->da9313_hwid_gpio) == 0) {
        chg_debug("it is OZ1C313\n");
        pinctrl_select_state(chip->pinctrl, chip->da9313_hwid_sleep);
        chip->hwid = HWID_OZ1C313;
        return chip->hwid;
    } else {
        pinctrl_select_state(chip->pinctrl, chip->da9313_hwid_sleep);
        usleep_range(10000, 10000);
        if(gpio_get_value(chip->da9313_hwid_gpio) == 1) {
            chg_debug("it is MAX77932\n");
            pinctrl_select_state(chip->pinctrl, chip->da9313_hwid_active);
            chip->hwid = HWID_MAX77932;
            return chip->hwid;
        }
    }
    chg_debug("it is DA9313\n");
    chip->hwid = HWID_DA9313;
    return chip->hwid;
}

static int halfv_chip_init(struct chip_da9313 *chip)
{
    int rc = 0;
    struct device_node *node = chip->dev->of_node;
    chip->da9313_hwid_gpio = of_get_named_gpio(node, "oplus,da9313-hwid-gpio", 0);
    if (chip->da9313_hwid_gpio < 0) {
        pr_err("da9313_hwid_gpio not specified\n");
        goto HWID_HANDLE;
    }

    chip->pinctrl = devm_pinctrl_get(chip->dev);
    if (IS_ERR_OR_NULL(chip->pinctrl)) {
        chg_err("get da9313 pinctrl fail\n");
        goto HWID_HANDLE;
    }

    chip->da9313_hwid_active = pinctrl_lookup_state(chip->pinctrl, "da9313_hwid_active");
    if (IS_ERR_OR_NULL(chip->da9313_hwid_active)) {
        chg_err("get da9313_hwid_active fail\n");
        goto HWID_HANDLE;
    }

    chip->da9313_hwid_sleep = pinctrl_lookup_state(chip->pinctrl, "da9313_hwid_sleep");
    if (IS_ERR_OR_NULL(chip->da9313_hwid_sleep)) {
        chg_err("get da9313_hwid_sleep fail\n");
        goto HWID_HANDLE;
    }

    chip->da9313_hwid_default = pinctrl_lookup_state(chip->pinctrl, "da9313_hwid_default");
    if (IS_ERR_OR_NULL(chip->da9313_hwid_default)) {
        chg_err("get da9313_hwid_default fail\n");
        goto HWID_HANDLE;
    }
HWID_HANDLE:
    get_hwid(chip);
    switch (chip->hwid) {
    case HWID_DA9313:
        da9313_hardware_init();
        break;
    case HWID_MAX77932:
        max77932_hardware_init();
        break;
    case HWID_OZ1C313:
        oz1c313_hardware_init();
        break;
    default:
        chg_err("No half voltage chip hwid matched!!!\n");
        break;
    }
    return rc;
}

static ssize_t proc_work_mode_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    uint8_t ret = 0;
    char page[10];
    int work_mode = 0;
    struct chip_da9313 *divider_ic = the_chip;

    if(divider_ic == NULL) {
        chg_err("%s: da9313 driver is not ready\n", __func__);
        return 0;
    }
    if (atomic_read(&divider_ic->suspended) == 1) {
        return 0;
    }

    if (oplus_wrap_get_allow_reading() == false) {
        return 0;
    }

    ret = da9313_read_reg(REG04_DA9313_ADDRESS, &work_mode);
    work_mode = ((work_mode & 0x02)? 1:0);

    chg_err("%s: work_mode = %d.\n", __func__, work_mode);
    sprintf(page, "%d", work_mode);
    ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

    return ret;
}

static ssize_t proc_work_mode_write(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
        char buffer[2] = {0};
        int work_mode = 0;
        struct chip_da9313 *divider_ic = the_chip;

        if (divider_ic == NULL) {
                chg_err("%s: da9313 driver is not ready\n", __func__);
                return -ENODEV;
        }

        if (atomic_read(&divider_ic->suspended) == 1) {
                return -EBUSY;
        }

        if (copy_from_user(buffer, buf, 2)) {
                chg_err("%s: read proc input error.\n", __func__);
                return -EFAULT;
        }

	if (buffer[0] < '0' || buffer[0] > '9') {
		return -EINVAL;
	}

        if (1 != sscanf(buffer, "%d", &work_mode)) {
                chg_err("invalid content: '%s', length = %zd\n", buf, count);
                return -EFAULT;
        }

        if (oplus_wrap_get_allow_reading() == false) {
                return count;
        }

        if (work_mode != 0) {
                divider_ic->fixed_mode_set_by_dev_file = false;
        } else {
                divider_ic->fixed_mode_set_by_dev_file = true;
        }

        if (work_mode != 0) {
                da9313_config_interface(REG04_DA9313_ADDRESS, REG04_DA9313_PVC_MODE_AUTO, REG04_DA9313_PVC_MODE_MASK);
        } else {
                da9313_config_interface(REG04_DA9313_ADDRESS, REG04_DA9313_PVC_MODE_FIXED, REG04_DA9313_PVC_MODE_MASK);
        }
        chg_err("new work_mode -> %s\n", ((work_mode != 0) ? "auto" : "fixed"));

        return count;
}

static const struct file_operations proc_work_mode_ops =
{
    .read = proc_work_mode_read,
    .write  = proc_work_mode_write,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static int init_da9313_proc(struct chip_da9313 *da)
{
    int ret = 0;
    struct proc_dir_entry *prEntry_da = NULL;
    struct proc_dir_entry *prEntry_tmp = NULL;

    prEntry_da = proc_mkdir("da9313", NULL);
    if (prEntry_da == NULL) {
        ret = -ENOMEM;
        chg_debug("%s: Couldn't create da9313 proc entry\n", __func__);
    }

    prEntry_tmp = proc_create_data("work_mode", 0644, prEntry_da, &proc_work_mode_ops, da);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        chg_debug("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }
    return 0;
}

static int da9313_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{             
    struct chip_da9313 *divider_ic;

    divider_ic = devm_kzalloc(&client->dev, sizeof(struct chip_da9313), GFP_KERNEL);
    if (!divider_ic) {
        dev_err(&client->dev, "failed to allocate divider_ic\n");
        return -ENOMEM;
    }

    chg_debug( " call \n");
    divider_ic->client = client;
    divider_ic->dev = &client->dev;
    the_chip = divider_ic;
    divider_ic->fixed_mode_set_by_dev_file = false;


    halfv_chip_init(divider_ic);
    init_da9313_proc(divider_ic);

    return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int da9313_pm_resume(struct device *dev)
{
        if (!the_chip) {
                return 0;
        }
        atomic_set(&the_chip->suspended, 0);
        return 0;
}

static int da9313_pm_suspend(struct device *dev)
{
        if (!the_chip) {
                return 0;
        }
        atomic_set(&the_chip->suspended, 1);
        return 0;
}

static const struct dev_pm_ops da9313_pm_ops = {
        .resume                = da9313_pm_resume,
        .suspend                = da9313_pm_suspend,
};
#else /*(LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))*/
static int da9313_resume(struct i2c_client *client)
{
        if (!the_chip) {
            return 0;
        }
        atomic_set(&the_chip->suspended, 0);
        return 0;
}

static int da9313_suspend(struct i2c_client *client, pm_message_t mesg)
{
        if (!the_chip) {
            return 0;
        }
        atomic_set(&the_chip->suspended, 1);
        return 0;
}
#endif /*(LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))*/

static struct i2c_driver da9313_i2c_driver;

static int da9313_driver_remove(struct i2c_client *client)
{
    int ret=0;

    chg_debug( "  ret = %d\n", ret);
    return 0;
}

static void da9313_shutdown(struct i2c_client *client)
{
    int rc = 0;
    struct chip_da9313 *divider_ic = the_chip;

    if (divider_ic == NULL) {
        chg_err("%s: da9313 driver is not ready\n", __func__);
        return;
    }
    if (atomic_read(&divider_ic->suspended) == 1) {
        return;
    }
    if (divider_ic->hwid != HWID_DA9313) {
        return;
    }

    rc = da9313_config_interface(REG04_DA9313_ADDRESS, REG04_DA9313_PVC_MODE_AUTO, REG04_DA9313_PVC_MODE_MASK);

    return;
}


/**********************************************************
  *
  *   [platform_driver API] 
  *
  *********************************************************/

static const struct of_device_id da9313_match[] = {
    { .compatible = "oplus,da9313-divider"},
    { },
};

static const struct i2c_device_id da9313_id[] = {
    {"da9313-divider", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, da9313_id);


static struct i2c_driver da9313_i2c_driver = {
    .driver		= {
        .name = "da9313-divider",
        .owner	= THIS_MODULE,
        .of_match_table = da9313_match,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
        .pm = &da9313_pm_ops,
#endif
    },
    .probe		= da9313_driver_probe,
    .remove		= da9313_driver_remove,
    .id_table	= da9313_id,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
    .resume         = da9313_resume,
    .suspend        = da9313_suspend,
#endif
	.shutdown	= da9313_shutdown,
};


module_i2c_driver(da9313_i2c_driver);
MODULE_DESCRIPTION("Driver for da9313 divider chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:da9313-divider");
