/*
 * Copyright (C) 2015 HUAQIN Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/pinctrl/devinfo.h>
#include <linux/pinctrl/consumer.h>
#include "camera_ldo.h"

/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/

static struct i2c_client *camera_ldo_i2c_client;
static struct pinctrl *camera_ldo_pctrl; /* static pinctrl instance */

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int camera_ldo_dts_probe(struct platform_device *pdev);
static int camera_ldo_dts_remove(struct platform_device *pdev);
static int camera_ldo_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int camera_ldo_i2c_remove(struct i2c_client *client);

/*****************************************************************************
 * Extern Area
 *****************************************************************************/
void camera_ldo_set_en_ldo(CAMERA_LDO_SELECT ldonum,unsigned int en)
{
	s32 ret=0;
	unsigned int value =0;

	 if (NULL == camera_ldo_i2c_client) {
	        printk("[camera_ldo] camera_ldo_i2c_client is null!!\n");
	        return ;
        }
	ret= i2c_smbus_read_byte_data(camera_ldo_i2c_client, CAMERA_LDO_LDO_EN_ADDR);

	if(ret <0)
	{
		printk("[camera_ldo] camera_ldo_set_en_ldo read error!\n");
		return;
	}

	if(en == 0)
	{
		value = ret & (~(0x01<<ldonum));
	}
	else
	{
		value = ret|(0x01<<ldonum);
	}

	i2c_smbus_write_byte_data(camera_ldo_i2c_client,CAMERA_LDO_LDO_EN_ADDR,value);
	printk("[camera_ldo] camera_ldo_set_en_ldo enable before:%x after set :%x  \n",ret,value);
       return;

}


//Voutx=0.6v+LDOX_OUT[7:0]*0.0125V   LDO1/LDO2
//Voutx=1.2v+LDOX_OUT[7:0]*0.0125V   LDO3~LDO7
void camera_ldo_set_ldo_value(CAMERA_LDO_SELECT ldonum,unsigned int value)
{
	unsigned int  Ldo_out =0;
       unsigned char regaddr =0;
       s32 ret =0;
	printk("[camera_ldo] %s enter!!!\n",__FUNCTION__);

	 if (NULL == camera_ldo_i2c_client) {
	        printk("[camera_ldo] camera_ldo_i2c_client is null!!\n");
	        return ;
        }
	if(ldonum >= CAMERA_LDO_MAX)
	{
		printk("[camera_ldo] error ldonum not support!!!\n");
		return;
	}

	switch(ldonum)
	{
		case CAMERA_LDO_DVDD1:
		case CAMERA_LDO_DVDD2:
			if(value<600)
			{
				printk("[camera_ldo] error vol!!!\n");
				goto exit;
			}
			else
			{
			 	Ldo_out = (value-600)*80/1000;
			}
	       break;
		case CAMERA_LDO_AVDD1:
		case CAMERA_LDO_AVDD2:
		case CAMERA_LDO_VDDAF:
		case CAMERA_LDO_VDDOIS:
		case CAMERA_LDO_VDDIO:
			if(value<1200)
			{
				printk("[camera_ldo] error vol!!!\n");
				goto exit;

			}
			else
			{
				Ldo_out = (value-1200)*80/1000;
			}
			break;
		default:
			goto exit;
			break;
	}
	regaddr = ldonum+CAMERA_LDO_LDO1_OUT_ADDR;

	printk("[camera_ldo] ldo=%d,value=%d,Ldo_out:%d,regaddr=0x%x \n",ldonum,value,Ldo_out,regaddr);
	i2c_smbus_write_byte_data(camera_ldo_i2c_client,regaddr,Ldo_out);
	ret=i2c_smbus_read_byte_data(camera_ldo_i2c_client,regaddr);
	printk("[camera_ldo] after write ret=0x%x\n",ret);

exit:
	printk("[camera_ldo] %s exit!!!\n",__FUNCTION__);

}


/*****************************************************************************
 * Data Structure
 *****************************************************************************/
static const char *camera_ldo_state_name[CAMERA_LDO_GPIO_STATE_MAX] = {
    "camera_ldo_gpio_enp0",
    "camera_ldo_gpio_enp1"
};/* DTS state mapping name */

static const struct of_device_id gpio_of_match[] = {
    { .compatible = "mediatek,gpio_camera_ldo", },
    {},
};

static const struct of_device_id i2c_of_match[] = {
    { .compatible = "mediatek,i2c_camera_ldo", },
    {},
};

static const struct i2c_device_id camera_ldo_i2c_id[] = {
    {"camera_ldo_I2C", 0},
    {},
};

static struct platform_driver camera_ldo_platform_driver = {
    .probe = camera_ldo_dts_probe,
    .remove = camera_ldo_dts_remove,
    .driver = {
        .name = "camera_ldo_DTS",
        .of_match_table = gpio_of_match,
    },
};

static struct i2c_driver camera_ldo_i2c_driver = {
/************************************************************
Attention:
Althouh i2c_bus do not use .id_table to match, but it must be defined,
otherwise the probe function will not be executed!
************************************************************/
    .id_table = camera_ldo_i2c_id,
    .probe = camera_ldo_i2c_probe,
    .remove = camera_ldo_i2c_remove,
    .driver = {
        .name = "camera_ldo_I2C",
        .of_match_table = i2c_of_match,
    },
};

/*****************************************************************************
 * Function
 *****************************************************************************/
static long camera_ldo_set_state(const char *name)
{
    int ret = 0;
    struct pinctrl_state *pState = 0;

    BUG_ON(!camera_ldo_pctrl);

    pState = pinctrl_lookup_state(camera_ldo_pctrl, name);
    if (IS_ERR(pState)) {
        pr_err("set state '%s' failed\n", name);
        ret = PTR_ERR(pState);
        goto exit;
    }

    /* select state! */
    pinctrl_select_state(camera_ldo_pctrl, pState);

exit:
    return ret; /* Good! */
}

void camera_ldo_gpio_select_state(CAMERA_LDO_GPIO_STATE s)
{
    printk("[camera_ldo]%s,%d\n",__FUNCTION__,s);

    BUG_ON(!((unsigned int)(s) < (unsigned int)(CAMERA_LDO_GPIO_STATE_MAX)));
    camera_ldo_set_state(camera_ldo_state_name[s]);
}

static long camera_ldo_dts_init(struct platform_device *pdev)
{
    int ret = 0;
    struct pinctrl *pctrl;

    /* retrieve */
    pctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(pctrl)) {
        dev_err(&pdev->dev, "Cannot find disp pinctrl!");
        ret = PTR_ERR(pctrl);
        goto exit;
    }

    camera_ldo_pctrl = pctrl;

exit:
    return ret;
}

static int camera_ldo_dts_probe(struct platform_device *pdev)
{
    int ret = 0;

    ret = camera_ldo_dts_init(pdev);
    if (ret) {
        printk("[camera_ldo]camera_ldo_dts_probe failed\n");
        return ret;
    }

    printk("[camera_ldo] camera_ldo_dts_probe success\n");

    return 0;
}

static int camera_ldo_dts_remove(struct platform_device *pdev)
{
    platform_driver_unregister(&camera_ldo_platform_driver);

    return 0;
}

static int camera_ldo_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    if (NULL == client) {
        printk("[camera_ldo] i2c_client is NULL\n");
        return -1;
    }
    camera_ldo_i2c_client = client;
    //camera_ldo_gpio_select_state(CAMERA_LDO_GPIO_STATE_ENP0);

    printk("[camera_ldo]camera_ldo_i2c_probe success addr = 0x%x\n", client->addr);
    return 0;
}

static int camera_ldo_i2c_remove(struct i2c_client *client)
{
    camera_ldo_i2c_client = NULL;
    i2c_unregister_device(client);

    return 0;
}

static int __init camera_ldo_init(void)
{
    if (i2c_add_driver(&camera_ldo_i2c_driver)) {
        printk("[camera_ldo]Failed to register camera_ldo_i2c_driver!\n");
        return -1;
    }

    if (platform_driver_register(&camera_ldo_platform_driver)) {
        printk("[camera_ldo]Failed to register camera_ldo_platform_driver!\n");
        i2c_del_driver(&camera_ldo_i2c_driver);
        return -1;
    }

    return 0;
}

static void __exit camera_ldo_exit(void)
{
    platform_driver_unregister(&camera_ldo_platform_driver);
    i2c_del_driver(&camera_ldo_i2c_driver);
}

module_init(camera_ldo_init);
module_exit(camera_ldo_exit);

MODULE_AUTHOR("AmyGuo <guohuiqing@huaqin.com>");
MODULE_DESCRIPTION("MTK EXT CAMERA LDO Driver");
MODULE_LICENSE("GPL");
