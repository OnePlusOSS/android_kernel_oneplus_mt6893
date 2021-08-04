// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>

enum pmic_ldo_online_ic{
    LDO_UNKNOW,
    FAN53870,
    WL2868C,
};

static enum pmic_ldo_online_ic ldo_type = LDO_UNKNOW;
extern int is_fan53870_pmic(void);
extern int wl2868c_test_i2c_enable(void);

extern int fan53870_ldo1_set_voltage(unsigned int set_uV);
extern int fan53870_ldo1_disable(void);
extern int fan53870_cam_ldo_disable(unsigned int LDO_NUM);
extern int fan53870_cam_ldo_set_voltage(unsigned int LDO_NUM, int set_mv);

extern int wl2868c_ldo_2_set_voltage(unsigned int set_uV);
extern int wl2868c_ldo_2_set_disable(void);
extern int wl2868c_ldo_set_disable(unsigned int ldo_num);
extern int wl2868c_voltage_output(unsigned int ldo_num, int vol);

struct pmic_ldo_operations{
    int (*ldo_set_voltage_mv)(unsigned int ldo_num, int set_mv);
    int (*ldo_2_set_voltage_uv)(unsigned int set_uv);
    int (*ldo_set_disable)(unsigned int ldo_num);
    int (*ldo_2_set_disable)(void);
};

static struct pmic_ldo_operations ldo_ops = {};

int pmic_ldo_get_type(void){
    int ret = 0;
    if(ldo_type != LDO_UNKNOW){
        return ret;
    }
    ret = is_fan53870_pmic();
    pr_err("%s,ldo_ops:%d,%s\n",__func__,ldo_ops,ldo_ops.ldo_set_voltage_mv);
    if(ret == 1){
        ldo_type = FAN53870;
        ldo_ops.ldo_set_voltage_mv = fan53870_cam_ldo_set_voltage;
        ldo_ops.ldo_2_set_voltage_uv = fan53870_ldo1_set_voltage;
        ldo_ops.ldo_set_disable = fan53870_cam_ldo_disable;
        ldo_ops.ldo_2_set_disable = fan53870_ldo1_disable;
        return 0;
    }else{
        pr_err("%s, fan53870 no ok,  fan53870_pmic status %d\n",__func__, ret);
        ret = wl2868c_test_i2c_enable();
        if(ret == 1){
            ldo_type = WL2868C;
            ldo_ops.ldo_set_voltage_mv = wl2868c_voltage_output;
            ldo_ops.ldo_2_set_voltage_uv = wl2868c_ldo_2_set_voltage;
            ldo_ops.ldo_set_disable = wl2868c_ldo_set_disable;
            ldo_ops.ldo_2_set_disable = wl2868c_ldo_2_set_disable;
            return 0;
        }else{
            pr_err("%s, wl2868 no ok,  wl2868 status %d\n",__func__, ret);
            return -1;
        }
    }
}
EXPORT_SYMBOL(pmic_ldo_get_type);

int pmic_ldo_set_voltage_mv(unsigned int ldo_num, int set_mv)
{
    if(ldo_type != LDO_UNKNOW){
        return ldo_ops.ldo_set_voltage_mv(ldo_num,set_mv);
    }else{
        if(!pmic_ldo_get_type())
            return ldo_ops.ldo_set_voltage_mv(ldo_num,set_mv);
    }
    return -1;
}
EXPORT_SYMBOL(pmic_ldo_set_voltage_mv);

int pmic_ldo_set_disable(unsigned int ldo_num)
{
    if(ldo_type != LDO_UNKNOW){
        return ldo_ops.ldo_set_disable(ldo_num);
    }else{
        if(!pmic_ldo_get_type())
            return ldo_ops.ldo_set_disable(ldo_num);
    }
    return -1;
}
EXPORT_SYMBOL(pmic_ldo_set_disable);


/*
* set ldo 2 voltage.
* return : reg write error code;
*/
int pmic_ldo_2_set_voltage_uv(unsigned int set_uV)
{
    pr_err("%s,ldo_ops:%d,%s\n",__func__,ldo_ops,ldo_ops.ldo_set_voltage_mv);
    if(ldo_type != LDO_UNKNOW){
        return ldo_ops.ldo_2_set_voltage_uv(set_uV);
    }else{
        if(!pmic_ldo_get_type())
            return ldo_ops.ldo_2_set_voltage_uv(set_uV);
    }
    return -1;
}
EXPORT_SYMBOL(pmic_ldo_2_set_voltage_uv);

/*
* set pmic_ldo IC disable,shutdown ldo2 output.
*/
int pmic_ldo_2_set_disable()
{
    if(ldo_type != LDO_UNKNOW){
        return ldo_ops.ldo_2_set_disable();
    }else{
        if(!pmic_ldo_get_type())
            return ldo_ops.ldo_2_set_disable();
    }
    return -1;
}
EXPORT_SYMBOL(pmic_ldo_2_set_disable);