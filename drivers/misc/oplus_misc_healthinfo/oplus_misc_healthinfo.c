// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/iio/consumer.h>
#include <soc/oplus/system/oplus_project.h>
#include <linux/export.h>
#include <linux/notifier.h>

#include "oplus_misc_healthinfo.h"

#define DRIVER_NAME "oplus_misc_healthinfo"
#define MAX_CNT     2592000
#define CHG_TIME    1000
#define PAGESIZE    512
#define MAX_SECOND  60
#define MAX_MINUTE  60
#define MAX_HOUR    60
#define MAX_LIGHTS  3
#define ON          1
#define OFF         0
#define CNT_CLEAR   0

unsigned int omh_debug;

static unsigned long long time_cnr[TIME_COUNT] = {0};
static unsigned int my_para[TIME_COUNT] = {0};
static unsigned int my_status[TIME_COUNT] = {0};

static unsigned long long lights_health_para[TIME_COUNT][TIME_COUNT] = {{0}};

struct work_struct			my_work;
struct workqueue_struct 	*my_wq;

static void lights_time_record_work(struct work_struct *work)
{
	unsigned long long time_cnt = 0;
	unsigned long long power_cnt = 0;
	int i = 0;

	for (i = 0;i < MAX_LIGHTS;i++) {
		if (lights_health_para[i][1] == LIGHTS_ON && lights_health_para[i][2] == OFF) {
			lights_health_para[i][2] = ON;
			lights_health_para[i][3] = ktime_to_ms(ktime_get());
			lights_health_para[i][7] = lights_health_para[i][6];

			OMH_DEBUG("%s ON",
			lights_health_para[i][0] > LIGHTS_GREEN ? "LIGHTS_BLUE" : \
			lights_health_para[i][0] ==  LIGHTS_GREEN ? "LIGHTS_GREEN" : "LIGHTS_RED");
		} else if (lights_health_para[i][1] == LIGHTS_OFF && lights_health_para[i][2] == ON) {
			lights_health_para[i][2] = OFF;
			lights_health_para[i][4] = ktime_to_ms(ktime_get());

			OMH_DEBUG("%s OFF",
			lights_health_para[i][0] > LIGHTS_GREEN ? "LIGHTS_BLUE" : \
			lights_health_para[i][0] ==  LIGHTS_GREEN ? "LIGHTS_GREEN" : "LIGHTS_RED");

			if (lights_health_para[i][5] < MAX_CNT) {
				time_cnt = lights_health_para[i][4] - lights_health_para[i][3];
				lights_health_para[i][5] += time_cnt;
			} else {
				lights_health_para[i][5] = CNT_CLEAR;
			}

			if (lights_health_para[i][5] > MAX_SECOND) {
				lights_health_para[i][9] = lights_health_para[i][5] / MAX_SECOND;
				if (lights_health_para[i][9] > MAX_MINUTE)
					lights_health_para[i][10] = lights_health_para[i][9] / MAX_MINUTE;
			}

			power_cnt = lights_health_para[i][7] * 4 * (time_cnt / CHG_TIME);
			lights_health_para[i][8] = power_cnt;

			OMH_DEBUG("debug : [%d][%d][%d]",
				lights_health_para[i][5], time_cnt, lights_health_para[i][7]);
		}
	}

	return;
}

int oplus_misc_healthinfo(int type, int para, int bright)
{
	int ret = 0;

	switch (type) {
	case LIGHTS_RED:
		lights_health_para[0][0] = LIGHTS_RED;
		lights_health_para[0][1] = para;
		lights_health_para[0][6] = bright;
	break;
	case LIGHTS_GREEN:
		lights_health_para[1][0] = LIGHTS_GREEN;
		lights_health_para[1][1] = para;
		lights_health_para[1][6] = bright;
	break;
	case LIGHTS_BLUE:
		lights_health_para[2][0] = LIGHTS_BLUE;
		lights_health_para[2][1] = para;
		lights_health_para[2][6] = bright;
	break;
	default:
		OMH_INFO("nothing get\n");
	break;
	}

	my_para[0] = type;
	my_para[1] = para;
	queue_work(my_wq, &my_work);

	return ret;
}
EXPORT_SYMBOL(oplus_misc_healthinfo);

static int proc_misc_healthinfo_read(struct seq_file *s, void *v) {
	struct oplus_misc_healthinfo_para *omhp = s->private;
	ssize_t ret = 0;
	int i = 0;
	unsigned long long cnt_time;
	unsigned long long lcd_para[TIME_COUNT][TIME_COUNT] = {{0}};

	mutex_lock(&omhp->my_lock);

	cnt_time = lights_health_para[0][5] + lights_health_para[1][5] + lights_health_para[2][5];

	cnt_time = cnt_time / CHG_TIME;


	OMH_DEBUG("0 lights_health_para[%d]", lights_health_para[0][5]);

	for (i = 0; i < MAX_LIGHTS; i++) {
		if ((lights_health_para[i][5] / CHG_TIME) > MAX_SECOND) {
			lcd_para[i][1] = lights_health_para[i][8];
			lcd_para[i][2] = CNT_CLEAR;
			lcd_para[i][3] = lights_health_para[i][9] / CHG_TIME;
			lcd_para[i][4] = lights_health_para[i][10] / CHG_TIME;
		} else {
			lcd_para[i][1] = lights_health_para[i][8];
			lcd_para[i][2] = lights_health_para[i][5] / CHG_TIME;
			lcd_para[i][3] = CNT_CLEAR;
			lcd_para[i][4] = CNT_CLEAR;
		}
	}

	seq_printf(s, "name:lights_all:%d\n", cnt_time);

	for (i = 0; i < MAX_LIGHTS; i++) {
		seq_printf(s, "name:light_%s, power:%d, times_sec:%d, times_min:%d, times_hour:%d\n",
			lights_health_para[i][0] > LIGHTS_GREEN ? "blue" : \
			lights_health_para[i][0] ==  LIGHTS_GREEN ? "green" : "red", \
			lcd_para[i][1],
			lcd_para[i][2],
			lcd_para[i][3],
			lcd_para[i][4]);
		OMH_DEBUG("read lcd_para[%d][2][3][4] = %d, %d, %d", i,
			lcd_para[i][2],
			lcd_para[i][3],
			lcd_para[i][4]);
	}

	mutex_unlock(&omhp->my_lock);

	return ret;
}

static ssize_t proc_misc_healthinfo_write(struct file *file, const char __user *buffer,
				size_t count, loff_t *ppos)
{
	struct oplus_misc_healthinfo_para *omhp = PDE_DATA(file_inode(file));
	char buf[PAGESIZE] = {0};
	int tmp = 0;
	int i = 0, j = 0;

	mutex_lock(&omhp->my_lock);

	if (!omhp || count >= sizeof(buf)) {
		OMH_INFO("! cd or count over size");
		goto OUT;
	}

	if (copy_from_user(buf, buffer, count)) {
		OMH_INFO("write proc input error.");
		goto OUT;
	}

	if(!sscanf(buf, "%d", &tmp)) {
		OMH_INFO("write proc input error.");
		goto OUT;
	}

	OMH_INFO(" tmp is %d \n", tmp);

	if (tmp == 1) {
		OMH_INFO("clear all\n");
		for (i = 0; i < TIME_COUNT; i++)
			for (j = 0; j < TIME_COUNT; j++)
				lights_health_para[i][j] = 0;
	}
	mutex_unlock(&omhp->my_lock);
	return count;
OUT:
	mutex_unlock(&omhp->my_lock);
	OMH_INFO(" write ec_debug_level failed \n");
	return -1;
}

static int proc_misc_healthinfo_open(struct inode *inode, struct file *file) {
	return single_open(file, proc_misc_healthinfo_read, PDE_DATA(inode));
}

static const struct file_operations proc_misc_healthinfo_ops = {
	.owner = THIS_MODULE,
	.open  = proc_misc_healthinfo_open,
	.read  = seq_read,
	.write = proc_misc_healthinfo_write,
	.release = single_release,
};

static int oplus_misc_healthinfo_init_proc(struct oplus_misc_healthinfo_para *omhp)
{
	int ret = 0;
	struct proc_dir_entry *prEntry_cr = NULL;
	struct proc_dir_entry *prEntry_tmp = NULL;

	OMH_INFO("entry");
	/*proc files-step1:/proc/oplus_misc*/

	prEntry_cr = proc_mkdir("oplus_misc", NULL);

	if (prEntry_cr == NULL) {
		ret = -ENOMEM;
		OMH_INFO("Couldn't create oplus_misc_healthinfo_init_proc entry");
	}

	/*proc files-step2:/proc/oplus_misc/healthinfo (misc_healthinfo interface)*/
	prEntry_tmp = proc_create_data("misc_feedback", 0666, prEntry_cr, &proc_misc_healthinfo_ops, omhp);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		OMH_INFO("Couldn't create proc_misc_healthinfo_ops entry");
	}

	return ret;
}

static oplus_misc_healthinfo_parse_dt(struct device *dev, struct oplus_misc_healthinfo_para *omhp)
{
	struct device_node *dn = dev->of_node;
	if (!dn) {
		OMH_INFO("Don't has device of_node.");
		return -1;
	}

	omhp->omhp_support =  of_property_read_bool(dn, "oplus_misc_healthinfo_support");

	return 0;
}

static int oplus_misc_healthinfo_probe(struct platform_device *pdev)
{
	int ret = 0, i = 0, j = 0;

	struct oplus_misc_healthinfo_para *omhp = NULL;
	pr_err("start to probe oplus_misc_healthinfo driver.");

	omhp = devm_kzalloc(&pdev->dev, sizeof(struct oplus_misc_healthinfo_para), GFP_KERNEL);
	if (!omhp) {
		OMH_INFO("Malloc memory for oplus_misc_healthinfo fail.");
		ret = -ENOMEM;
		goto PROBE_ERR;
	}

	ret = oplus_misc_healthinfo_parse_dt(&pdev->dev, omhp);

	omhp->my_pdev = pdev;
	omhp->my_dev = &pdev->dev;
	mutex_init(&omhp->my_lock);
	platform_set_drvdata(pdev, omhp);

	OMH_INFO("probe clear all\n");

	for (i = 0; i < TIME_COUNT; i++)
		for (j = 0; j < TIME_COUNT; j++)
			lights_health_para[i][j] = 0;

	my_wq = create_singlethread_workqueue("oplus_misc_healthinfo_wq");
	if (!my_wq) {
		ret = -ENOMEM;
		OMH_INFO("Malloc memory for oplus_misc_healthinfo fail.");
		goto PROBE_ERR;
	}
	INIT_WORK(&my_work, lights_time_record_work);


	ret = oplus_misc_healthinfo_init_proc(omhp);

	if (ret) {
		OMH_INFO("creat oplus_misc_healthinfo_init_proc proc error.");
		goto PROBE_ERR;
	}

PROBE_ERR:
	return 0;
}


static int oplus_misc_healthinfo_remove(struct platform_device *dev) {
	struct oplus_misc_healthinfo_para *omhp = platform_get_drvdata(dev);
	OMH_INFO("start remove the oplus_misc_healthinfo platform dev.");

	if (omhp) {
		proc_remove(omhp->my_prEntry_cr);
		omhp->my_prEntry_cr = NULL;
	}

	return 0;
}

static const struct of_device_id oplus_misc_healthinfo_match_table[] = {
	{.compatible = "oplus,misc_healthinfo"},
	{}
};

static struct platform_driver oplus_misc_healthinfo_driver = {
	.probe = oplus_misc_healthinfo_probe,
	.remove = oplus_misc_healthinfo_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name  = DRIVER_NAME,
		.of_match_table = oplus_misc_healthinfo_match_table,
	},
};

static int __init oplus_misc_healthinfo_init(void) {
	return platform_driver_register(&oplus_misc_healthinfo_driver);
}

static void __exit oplus_misc_healthinfo_exit(void) {
	return platform_driver_unregister(&oplus_misc_healthinfo_driver);
}


late_initcall(oplus_misc_healthinfo_init);
module_exit(oplus_misc_healthinfo_exit);

MODULE_DESCRIPTION("oplus_misc_healthinfo Driver Module");
MODULE_AUTHOR("Xuhang.Li");
MODULE_LICENSE("GPL v2");
