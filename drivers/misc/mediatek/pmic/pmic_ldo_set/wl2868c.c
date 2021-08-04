// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#define wl2868c_IO_REG_LIMIT 20
#define wl2868c_IO_BUFFER_LIMIT 128

#define WL2868C_ADDR 0x28
#define WL2868C_REG_ID0 0x00
#define WL2868C_REG_ID1 0x01
#define WL2868C_REG_LDO1_VOUT 0x03
#define WL2868C_REG_LDO2_VOUT 0x04
#define WL2868C_REG_LDO3_VOUT 0x05
#define WL2868C_REG_LDO4_VOUT 0x06
#define WL2868C_REG_LDO5_VOUT 0x07
#define WL2868C_REG_LDO6_VOUT 0x08
#define WL2868C_REG_LDO7_VOUT 0x09
#define WL2868C_REG_LDOX_EN 0x0e

/*
Hardware operate:
RST_N=VSYS=3.8V
VIN12=1.5V
VIN34=VIN5=VIN6=VIN7=3.6V
*/
#define LDO12_VOUT_MAX 1512
#define LDO34567_VOUT_MAX 3544
/*
LDO1-2 voltage output Range 0.496v ~ 1.512v
VOUT1/2 = 0.496v+LDO1/2_VOUT[6:0]*0.008V
LDO3-7 voltage output Range 1.504v ~ 3.544v
VOUT3/7 = 1.504V+LDO3/7_VOUT[7:0]*0.008V
*/

#undef pr_debug
#define pr_debug pr_err
/*!
 * reg_value struct
 */
struct reg_value {
    u8 u8Add;
    u8 u8Val;
};

/*!
 * wl2868c_data_t struct
 */
struct wl2868c_data_t {
    struct i2c_client *i2c_client;
    int en_gpio;
    u8 chip_id;
    u8 id_reg;
    u8 id_val;
    int wl2868_status_flag;
    int v_ldo[7];
    u8 ldo_en_select;
};

enum wl2868c_ldo_num {
    WL2868C_LDO1 = 1,
    WL2868C_LDO2 = 2,
    WL2868C_LDO3 = 3,
    WL2868C_LDO4 = 4,
    WL2868C_LDO5 = 5,
    WL2868C_LDO6 = 6,
    WL2868C_LDO7 = 7,
};

const unsigned int LDO12_voltage_base = 496; // 496mv
const unsigned int LDO34567_voltage_base = 1504; // 1504mv

/*!
 * wl2868c_data
 */
static struct wl2868c_data_t wl2868c_data;


/*!
 * wl2868c write reg function
 *
 * @param reg u8
 * @param val u8
 * @return  Error code indicating success or failure
 */
static int wl2868c_write_reg(u8 reg, u8 val)
{
    u8 au8Buf[2] = {0};
    au8Buf[0] = reg;
    au8Buf[1] = val;
    if (i2c_master_send(wl2868c_data.i2c_client, au8Buf, 2) < 0)
    {
        pr_err("%s:write reg error:reg=%x,val=%x\n",
            __func__, reg, val);
        return -1;
    }
    pr_err("%s:write reg 0x%x val 0x%x ok\n", __func__,reg,val);
    return 0;
}

/*!
 * wl2868c read reg function
 *
 * @param reg u8
 * @param val u8 *
 * @return  Error code indicating success or failure
 */
static int wl2868c_read_reg(u8 reg, u8 *val)
{
    u8 au8RegBuf[1] = {0};
    u8 u8RdVal = 0;
    au8RegBuf[0] = reg;
    if (1 != i2c_master_send(wl2868c_data.i2c_client, au8RegBuf, 1))
    {
        pr_err("%s:write reg error:reg=%x\n", __func__, reg);
        return -1;
    }
    if (1 != i2c_master_recv(wl2868c_data.i2c_client, &u8RdVal, 1))
    {
        pr_err("%s:read reg error:reg=%x,val=%x\n",__func__, reg, u8RdVal);
        return -1;
    }
    *val = u8RdVal;
    pr_err("%s: read reg 0x%x val 0x%x ok\n", __func__,reg,*val);
    return 0;
}

/*!
 * wl2868c get i2c enable status
 *
 * @param *
 * @return  wl2868_status_flag -1 means i2c error
 *     0 means chip_id not reight, 1 means ldo ic is ok *
 *
 */
int wl2868c_test_i2c_enable(void)
{
    return wl2868c_data.wl2868_status_flag;
}

/*!
 * wl2868c VOUTPUT_T
 *
 * @param ldo_num ldo number *
 * @param low_or_high low means ldo_num < 3,high means ldo_num >=3 *
 * @param vol ldo output voltage mv *
 * @param ldo_select select ldo number en, reg_ldo_en code in hex *
 * @return  Error code indicating success or failure
 */
static int wl2868c_voltage_output_t(u8 ldo_num,int low_or_high, int vol, u8 ldo_select)
{
    u8 ldo_vout_value = 0;
    int ret = 0;
    pr_err("%s,en = %d,set ldo_num = %d,vol = %d\n",__func__, wl2868c_data.ldo_en_select, ldo_num, vol);
    if (low_or_high) {
        if (vol > 0) {
            if (vol < LDO12_voltage_base)
                goto vol_err;
            vol = vol > LDO12_VOUT_MAX ? LDO12_VOUT_MAX : vol;
            ldo_vout_value = (vol-LDO12_voltage_base) / 8;
            wl2868c_data.ldo_en_select |= (ldo_select + 0x80);
            ret = wl2868c_write_reg(ldo_num, ldo_vout_value);
            ret = wl2868c_write_reg(WL2868C_REG_LDOX_EN, wl2868c_data.ldo_en_select);
        } else {
            if (vol != -1)
                goto vol_err;
            wl2868c_data.ldo_en_select &= (0xFF - ldo_select);
            ret = wl2868c_write_reg(WL2868C_REG_LDOX_EN, wl2868c_data.ldo_en_select);
        }
    } else {
        if (vol > 0) {
            if (vol < LDO34567_voltage_base)
                goto vol_err;
            vol = vol > LDO34567_VOUT_MAX ? LDO34567_VOUT_MAX : vol;
            ldo_vout_value = (vol-LDO34567_voltage_base) / 8;
            wl2868c_data.ldo_en_select |= (ldo_select + 0x80);
            ret = wl2868c_write_reg(ldo_num, ldo_vout_value);
            ret = wl2868c_write_reg(WL2868C_REG_LDOX_EN, wl2868c_data.ldo_en_select);
        } else {
            if (vol != -1)
                goto vol_err;
            wl2868c_data.ldo_en_select &= (0xFF - ldo_select);
            ret = wl2868c_write_reg(WL2868C_REG_LDOX_EN, wl2868c_data.ldo_en_select);
        }
    }
    return ret;

vol_err:
    pr_err("%s,set vol err vol:%d,ldo_num:%d\n",__func__,vol,ldo_num);
    return -1;
}

/*!
 * wl2868c VOUTPUT
 *
 * @param ldo_num ldo number *
 * @param vol ldo output voltage mv *
 * @return  Error code indicating success or failure
 */
int wl2868c_voltage_output(unsigned int ldo_num, int vol)
{
    int ret = 0;
    switch (ldo_num)
    {
        case WL2868C_LDO1:
            ret = wl2868c_voltage_output_t(WL2868C_REG_LDO1_VOUT,1,vol,0x01);
            break;
        case WL2868C_LDO2:
            ret = wl2868c_voltage_output_t(WL2868C_REG_LDO2_VOUT,1,vol,0x02);
            break;
        case WL2868C_LDO3:
            ret = wl2868c_voltage_output_t(WL2868C_REG_LDO3_VOUT,0,vol,0x04);
            break;
        case WL2868C_LDO4:
            ret = wl2868c_voltage_output_t(WL2868C_REG_LDO4_VOUT,0,vol,0x08);
            break;
        case WL2868C_LDO5:
            ret = wl2868c_voltage_output_t(WL2868C_REG_LDO5_VOUT,0,vol,0x10);
            break;
        case WL2868C_LDO6:
            ret = wl2868c_voltage_output_t(WL2868C_REG_LDO6_VOUT,0,vol,0x20);
            break;
        case WL2868C_LDO7:
            ret = wl2868c_voltage_output_t(WL2868C_REG_LDO7_VOUT,0,vol,0x40);
            break;
        default:
            return -1;
    }
    return ret;
}
/*!
 * wl2868c power on function
 *
 * @param dev struct device *
 * @return  Error code indicating success or failure
 */
static int wl2868c_power_on(struct device *dev)
{
    int ret = 0;
    wl2868c_data.en_gpio = of_get_named_gpio(dev->of_node, "en-gpios", 0);
    if (wl2868c_data.en_gpio < 0) {
        pr_err("wl2868c_data.en_gpio not specified\n");
    }
    if (!gpio_is_valid(wl2868c_data.en_gpio) ) {
        pr_err("wl2868c_data en gpio\n");
        return -EINVAL;
    }
    ret = devm_gpio_request_one(dev, wl2868c_data.en_gpio, GPIOF_OUT_INIT_HIGH, "wl2868c_en");
    if (ret < 0)
        pr_err("wl2868c_en request failed %d\n", ret);
    else
        pr_err("%s: en request ok\n", __func__);
    return ret;
}

/*!
 * wl2868c match id function
 *
 * @param dev struct device *
 * @return  Error code indicating success or failure
 */
static int wl2868c_get_match_id(struct device *dev)
{
    int ret = 0;
    int access_time = 3;
    ret = of_property_read_u32(dev->of_node, "id_reg", (u32 *)&(wl2868c_data.id_reg));
    pr_err("wl2868c_get_match_id\n");
    if (ret) {
        pr_err("id_reg missing or invalid\n");
        return ret;
    }
    ret = of_property_read_u32(dev->of_node, "id_val", (u32 *)&(wl2868c_data.id_val));
    if (ret) {
        pr_err("id_val missing or invalid\n");
        return ret;
    }
    pr_err("%s, id_val %d\n",__func__, wl2868c_data.id_val);
    ret = wl2868c_read_reg(wl2868c_data.id_reg, &(wl2868c_data.chip_id));
    pr_err("%s, chip_id %d\n",__func__, wl2868c_data.chip_id);
    while (ret == -1 && --access_time) {
        mdelay(2);
        ret = wl2868c_read_reg(wl2868c_data.id_reg, &(wl2868c_data.chip_id));
    }
    return ret;
}

static ssize_t v_ldo_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t len = 0;
    int i = 0;
    u8 reg[7];
    u8 reg_en;
    for (i = 0; i < 7; i++) {
        wl2868c_read_reg(i + 3, &reg[i]);
    }
    wl2868c_read_reg(WL2868C_REG_LDOX_EN, &reg_en);

    for (i = 0;i < 7; i++) {
        len += snprintf(buf + len, PAGE_SIZE, "%d, setvol = %d, reg = %x\n", i+1, wl2868c_data.v_ldo[i], reg[i]);
    }
    len += snprintf(buf + len, PAGE_SIZE, "reg_en = %x\n", reg_en);
    return len;
}
static ssize_t v_ldo_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    int ret = 0;
    int i = 0;
    int temp_ldo_vol = 0;
    if (strlen(buf) < 2 || strlen(buf) > 16)
        return size;

    ret = sscanf(buf, "%d,%d", &i, &temp_ldo_vol);
    if (!ret) {
        pr_err("input v_ldo_store format error!\n");
    } else {
        pr_err("input v_ldo_store i %d , vol = %d\n",i,temp_ldo_vol);
    }
    if (i > 0 && i < 8) {
        wl2868c_data.v_ldo[i - 1] = temp_ldo_vol;
        pr_err("v_ldo[i] v_ldo_store i %d , vol = %d\n",i,wl2868c_data.v_ldo[i - 1]);
        wl2868c_voltage_output(i, wl2868c_data.v_ldo[i - 1]);
    }

    return size;
}
static DEVICE_ATTR(v_ldo, S_IWUSR | S_IRUGO, v_ldo_show, v_ldo_store);

static struct device_attribute *wl2868c_attributes[] = {
    &dev_attr_v_ldo,
    NULL,
};

int wl2868c_ldo_2_set_voltage(unsigned int set_uV)
{
    return wl2868c_voltage_output(2, set_uV/1000); /* ldo 2 */
}

int wl2868c_ldo_set_disable(unsigned int ldo_num)
{
    return wl2868c_voltage_output(ldo_num, -1);
}
int wl2868c_ldo_2_set_disable(void)
{
    return wl2868c_voltage_output(WL2868C_LDO2, -1);
}

/*!
 * wl2868c I2C probe function
 *
 * @param client struct i2c_client *
 * @param id struct i2c_device_id *
 * @return  Error code indicating success or failure
 */
static int wl2868c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    struct device_attribute *attr;
    struct device_attribute **attrs = wl2868c_attributes;
    pr_err("wl2868c_probe strart attr ok!\n");
    memset(&wl2868c_data, 0, sizeof(struct wl2868c_data_t));
    wl2868c_data.i2c_client = client;
    wl2868c_data.ldo_en_select = 0;
    ret = wl2868c_power_on(&client->dev);
    if (ret) {
        pr_err("wl2868c_power_on failed %d\n", ret);
        return ret;
    }
    ret = wl2868c_get_match_id(&client->dev);
    if (ret) {
        wl2868c_data.wl2868_status_flag = -1;
        pr_err("wl2868c_get_match_id i2c read failed  probe failed%d\n", ret);
        return ret;
    } else {
        wl2868c_data.wl2868_status_flag = (wl2868c_data.chip_id == wl2868c_data.id_val) ? 1 : 0;
        pr_err("wl2868c_get_match_id i2c read success  %d\n", ret);
    }
    while ((attr = *attrs++)) {
        ret = device_create_file(&client->dev,attr);
        if (ret) {
            pr_err(" device_create_file error\n");
        }
    }
    pr_err("wl2868c_probe successed!\n");
    return 0;
}

/*!
 * wl2868c I2C remove function
 *
 * @param client struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int wl2868c_remove(struct i2c_client *client)
{
    pr_debug("deregister wl2868c device ok\n");
    return 0;
}

/*!
 * i2c_device_id struct
 */
static const struct i2c_device_id wl2868c_id[] =
{
    {"wl2868c-i2c", 0},
    {},
};

MODULE_DEVICE_TABLE(i2c, wl2868c_id);

/*!
 * i2c_driver struct
 */
static struct i2c_driver wl2868c_i2c_driver =
{
    .driver = {
        .owner = THIS_MODULE,
        .name  = "wl2868c-i2c",},
    .probe  = wl2868c_probe,
    .remove = wl2868c_remove,
    .id_table = wl2868c_id,
};

/*!
 * wl2868c init function
 *
 * @return  Error code indicating success or failure
 */
static __init int wl2868c_init(void)
{
    u8 ret = 0;
    ret = i2c_add_driver(&wl2868c_i2c_driver);
    if (ret != 0) {
        pr_err("%s: add driver failed, error=%d\n",__func__, ret);
        return ret;
    }
    pr_debug("%s: add driver success\n", __func__);
    return ret;
}

/*!
 * wl2868c cleanup function
 */
static void __exit wl2868c_clean(void)
{
    i2c_del_driver(&wl2868c_i2c_driver);
}

module_init(wl2868c_init);
module_exit(wl2868c_clean);

MODULE_DESCRIPTION("wl2868c Power IC Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
