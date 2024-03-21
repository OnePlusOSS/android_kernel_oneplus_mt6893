/************************************************************************************
 ** OPLUS_FEATURE_SENSOR
 ** OPLUS Coding Static Checking Skip
 ** Copyright (C), 2008-2018, OPLUS Mobile Comm Corp., Ltd
 **
 ** Description:
 **      oplus_als_cali_value.c (sw23)
 **
 ** Version: 1.0
 ** Date created: 18:03:11,11/21/2018
 ** --------------------------- Revision History: --------------------------------
 **  <author>      <data>            <desc>
 **  Chao.Zeng       11/21/2018      create the file
 ************************************************************************************/

#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/workqueue.h>
#include "../system/include/oplus_project.h"
#include "../../../misc/mediatek/sensors-1.0/oplus_sensor_devinfo/sensor_devinfo.h"
#define SENSOR_ALSPARA_DELAY_FIRST_TIME  10000
#define SENSOR_ALSPARA_DELAY_TIME  3000

struct delayed_work sensor_als_calipara_work;

/*struct oplus_als_cali_data {
    int red_max_lux;
    int green_max_lux;
    int blue_max_lux;
    int white_max_lux;
    int cali_coe;
    int row_coe;
    struct proc_dir_entry         *proc_oplus_als;
};*/
static struct oplus_als_cali_data *gdata = NULL;

static ssize_t red_max_lux_read_proc(struct file *file, char __user *buf,
        size_t count,loff_t *off)
{
    char page[256] = {0};
    int len = 0;

    if (!gdata){
        return -ENOMEM;
    }

    len = sprintf(page,"%d",gdata->red_max_lux);

    if(len > *off)
        len -= *off;
    else
        len = 0;

    if(copy_to_user(buf,page,(len < count ? len : count))){
        return -EFAULT;
    }
    *off += len < count ? len : count;
    return (len < count ? len : count);
}
static ssize_t red_max_lux_write_proc(struct file *file, const char __user *buf,
        size_t count, loff_t *off)

{
    char page[256] = {0};
    unsigned int input = 0;

    if(!gdata){
        return -ENOMEM;
    }


    if (count > 256)
        count = 256;
    if(count > *off)
        count -= *off;
    else
        count = 0;

    if (copy_from_user(page, buf, count))
        return -EFAULT;
    *off += count;

    if (sscanf(page, "%u", &input) != 1) {
        count = -EINVAL;
        return count;
    }

    if(input != gdata->red_max_lux){
        gdata->red_max_lux= input;
    }

    return count;
}
static struct file_operations red_max_lux_fops = {
    .read = red_max_lux_read_proc,
    .write = red_max_lux_write_proc,
};
static ssize_t white_max_lux_read_proc(struct file *file, char __user *buf,
        size_t count,loff_t *off)
{
    char page[256] = {0};
    int len = 0;

    if (!gdata){
        return -ENOMEM;
    }

    len = sprintf(page,"%d",gdata->white_max_lux);

    if(len > *off)
        len -= *off;
    else
        len = 0;

    if(copy_to_user(buf,page,(len < count ? len : count))){
        return -EFAULT;
    }
    *off += len < count ? len : count;
    return (len < count ? len : count);
}
static ssize_t white_max_lux_write_proc(struct file *file, const char __user *buf,
        size_t count, loff_t *off)

{
    char page[256] = {0};
    unsigned int input = 0;

    if(!gdata){
        return -ENOMEM;
    }


    if (count > 256)
        count = 256;
    if(count > *off)
        count -= *off;
    else
        count = 0;

    if (copy_from_user(page, buf, count))
        return -EFAULT;
    *off += count;

    if (sscanf(page, "%u", &input) != 1) {
        count = -EINVAL;
        return count;
    }

    if(input != gdata->white_max_lux){
        gdata->white_max_lux= input;
    }

    return count;
}
static struct file_operations white_max_lux_fops = {
    .read = white_max_lux_read_proc,
    .write = white_max_lux_write_proc,
};
static ssize_t blue_max_lux_read_proc(struct file *file, char __user *buf,
        size_t count,loff_t *off)
{
    char page[256] = {0};
    int len = 0;

    if (!gdata){
        return -ENOMEM;
    }

    len = sprintf(page,"%d",gdata->blue_max_lux);

    if(len > *off)
        len -= *off;
    else
        len = 0;

    if(copy_to_user(buf,page,(len < count ? len : count))){
        return -EFAULT;
    }
    *off += len < count ? len : count;
    return (len < count ? len : count);
}
static ssize_t blue_max_lux_write_proc(struct file *file, const char __user *buf,
        size_t count, loff_t *off)

{
    char page[256] = {0};
    unsigned int input = 0;

    if(!gdata){
        return -ENOMEM;
    }


    if (count > 256)
        count = 256;
    if(count > *off)
        count -= *off;
    else
        count = 0;

    if (copy_from_user(page, buf, count))
        return -EFAULT;
    *off += count;

    if (sscanf(page, "%u", &input) != 1) {
        count = -EINVAL;
        return count;
    }

    if(input != gdata->blue_max_lux){
        gdata->blue_max_lux= input;
    }

    return count;
}
static struct file_operations blue_max_lux_fops = {
    .read = blue_max_lux_read_proc,
    .write = blue_max_lux_write_proc,
};
static ssize_t green_max_lux_read_proc(struct file *file, char __user *buf,
        size_t count,loff_t *off)
{
    char page[256] = {0};
    int len = 0;

    if (!gdata){
        return -ENOMEM;
    }

    len = sprintf(page,"%d",gdata->green_max_lux);

    if(len > *off)
        len -= *off;
    else
        len = 0;

    if(copy_to_user(buf,page,(len < count ? len : count))){
        return -EFAULT;
    }
    *off += len < count ? len : count;
    return (len < count ? len : count);
}
static ssize_t green_max_lux_write_proc(struct file *file, const char __user *buf,
        size_t count, loff_t *off)

{
    char page[256] = {0};
    unsigned int input = 0;

    if(!gdata){
        return -ENOMEM;
    }


    if (count > 256)
        count = 256;
    if(count > *off)
        count -= *off;
    else
        count = 0;

    if (copy_from_user(page, buf, count))
        return -EFAULT;
    *off += count;

    if (sscanf(page, "%u", &input) != 1) {
        count = -EINVAL;
        return count;
    }

    if(input != gdata->green_max_lux){
        gdata->green_max_lux= input;
    }

    return count;
}
static struct file_operations green_max_lux_fops = {
    .read = green_max_lux_read_proc,
    .write = green_max_lux_write_proc,
};
static ssize_t cali_coe_read_proc(struct file *file, char __user *buf,
        size_t count,loff_t *off)
{
    char page[256] = {0};
    int len = 0;

    if (!gdata){
        return -ENOMEM;
    }

    len = sprintf(page,"%d",gdata->cali_coe);

    if(len > *off)
        len -= *off;
    else
        len = 0;

    if(copy_to_user(buf,page,(len < count ? len : count))){
        return -EFAULT;
    }
    *off += len < count ? len : count;
    return (len < count ? len : count);
}
static ssize_t cali_coe_write_proc(struct file *file, const char __user *buf,
        size_t count, loff_t *off)

{
    char page[256] = {0};
    unsigned int input = 0;

    if(!gdata){
        return -ENOMEM;
    }


    if (count > 256)
        count = 256;
    if(count > *off)
        count -= *off;
    else
        count = 0;

    if (copy_from_user(page, buf, count))
        return -EFAULT;
    *off += count;

    if (sscanf(page, "%u", &input) != 1) {
        count = -EINVAL;
        return count;
    }

    if(input != gdata->cali_coe){
        gdata->cali_coe= input;
    }

    return count;
}
static struct file_operations cali_coe_fops = {
    .read = cali_coe_read_proc,
    .write = cali_coe_write_proc,
};
static ssize_t row_coe_read_proc(struct file *file, char __user *buf,
        size_t count,loff_t *off)
{
    char page[256] = {0};
    int len = 0;

    if (!gdata){
        return -ENOMEM;
    }

    len = sprintf(page,"%d",gdata->row_coe);

    if(len > *off)
        len -= *off;
    else
        len = 0;

    if(copy_to_user(buf,page,(len < count ? len : count))){
        return -EFAULT;
    }
    *off += len < count ? len : count;
    return (len < count ? len : count);
}
static ssize_t row_coe_write_proc(struct file *file, const char __user *buf,
        size_t count, loff_t *off)

{
    char page[256] = {0};
    unsigned int input = 0;

    if(!gdata){
        return -ENOMEM;
    }


    if (count > 256)
        count = 256;
    if(count > *off)
        count -= *off;
    else
        count = 0;

    if (copy_from_user(page, buf, count))
        return -EFAULT;
    *off += count;

    if (sscanf(page, "%u", &input) != 1) {
        count = -EINVAL;
        return count;
    }

    if(input != gdata->row_coe){
        gdata->row_coe= input;
    }

    return count;
}
static struct file_operations row_coe_fops = {
    .read = row_coe_read_proc,
    .write = row_coe_write_proc,
};
static void sensor_para_work(struct work_struct *work)
{
    int rc = 0;
	int count = 0;
    struct proc_dir_entry *pentry;
    unsigned int prj = 0;

    struct oplus_als_cali_data *data = NULL;
    if(gdata){
        printk("%s:just can be call one time\n",__func__);
        return;
    }
    data = kzalloc(sizeof(struct oplus_als_cali_data),GFP_KERNEL);
    if(data == NULL){
        rc = -ENOMEM;
        printk("%s:kzalloc fail %d\n",__func__,rc);
        return;
    }
    gdata = data;
    gdata->row_coe = 540;
    prj = get_project();
    if ( (prj == 19550) || (prj == 19551) || (prj == 19553) || (prj == 19556)) {
        gdata->row_coe = 140;
    }
    else if((prj == 19354) || (prj == 19357) || (prj == 19358) || (prj == 19359)){
        gdata->row_coe = 110;
    }
	if ((prj == 19536) || (prj == 19537) || (prj == 19538) || (prj == 19539) || (prj == 19541)) {
		while (count <= 3) {
			if (is_sensor_available("tcs3701")) {
				gdata->row_coe = 110;
				break;
			} else if (is_sensor_available("stk32600")) {
				gdata->row_coe = 65;
				break;
			}
			schedule_delayed_work(&sensor_als_calipara_work, msecs_to_jiffies(SENSOR_ALSPARA_DELAY_TIME));
			count++;
		}
    }
    if (gdata->proc_oplus_als) {
        printk("proc_oplus_als has alread inited\n");
        return;
    }

    gdata->proc_oplus_als =  proc_mkdir("oplusAls", NULL);
    if(!gdata->proc_oplus_als) {
        pr_err("can't create proc_oplus_als proc\n");
        rc = -EFAULT;
        return;
    }

    pentry = proc_create("red_max_lux",0666, gdata->proc_oplus_als,
        &red_max_lux_fops);
    if(!pentry) {
        pr_err("create red_max_lux proc failed.\n");
        rc = -EFAULT;
        return;
    }

    pentry = proc_create("green_max_lux",0666, gdata->proc_oplus_als,
        &green_max_lux_fops);
    if(!pentry) {
        pr_err("create green_max_lux proc failed.\n");
        rc = -EFAULT;
        return;
    }

    pentry = proc_create("blue_max_lux",0666, gdata->proc_oplus_als,
        &blue_max_lux_fops);
    if(!pentry) {
        pr_err("create blue_max_lux proc failed.\n");
        rc = -EFAULT;
        return;
    }

    pentry = proc_create("white_max_lux",0666, gdata->proc_oplus_als,
        &white_max_lux_fops);
    if(!pentry) {
        pr_err("create white_max_lux proc failed.\n");
        rc = -EFAULT;
        return;
    }

    pentry = proc_create("cali_coe",0666, gdata->proc_oplus_als,
        &cali_coe_fops);
    if(!pentry) {
        pr_err("create cali_coe proc failed.\n");
        rc = -EFAULT;
        return;
    }

    pentry = proc_create("row_coe",0666, gdata->proc_oplus_als,
        &row_coe_fops);
    if(!pentry) {
        pr_err("create row_coe proc failed.\n");
        rc = -EFAULT;
        return;
    }
    return;
}

static int __init oplus_als_cali_data_init(void)
{
	INIT_DELAYED_WORK(&sensor_als_calipara_work, sensor_para_work);
	schedule_delayed_work(&sensor_als_calipara_work, msecs_to_jiffies(SENSOR_ALSPARA_DELAY_FIRST_TIME));
	return 0;
}

void oplus_als_cali_data_clean(void)
{
    if (gdata){
        kfree(gdata);
        gdata = NULL;
    }
}
module_init(oplus_als_cali_data_init);
module_exit(oplus_als_cali_data_clean);
MODULE_DESCRIPTION("OPLUS custom version");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Chao.Zeng");
