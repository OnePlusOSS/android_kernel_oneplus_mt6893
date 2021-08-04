/***********************************************************
** Copyright (C), 2008-2016, OPLUS Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** File: - devinfo.c
** Description: source  devinfo
**
** Version: 1.0
** Date : 2018/05/10
**
** ------------------------------- Revision History: -------------------------------
**  	<author>		    <data> 	         <version >	       <desc>
**
****************************************************************/
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#define ITEM_LENGTH 50
#define DEVINFO_PATH "devinfo"

struct devinfo{
char device_name[ITEM_LENGTH];
char device_version [ITEM_LENGTH];
char device_manufacture[ITEM_LENGTH];
char fw_path[ITEM_LENGTH];
bool need_fw_path;
};

struct devinfo g_devinfo_items[] = {
	{"Cam_f", "Null", "Null","Null",0},
	{"Cam_b", "Null", "Null","Null",0},
	{"Cam_b1", "Null", "Null","Null",0},
	{"Cam_b2", "Null", "Null","Null",0},
	{"Cam_b3", "Null", "Null","Null",0},
	{"Battery", "Null", "Null","Null",0},
    {"Emmc", "Null", "Null","Null",0},
    {"tp", "Null", "Null","Null",1},
	//{"lcd", "Null", "Null","Null",0},
#ifdef ODM_WT_EDIT
	{"Sensor_alsps", "STK33562", "STK", "Null", 0},
    {"Sensor_alsps", "STK3331", "STK", "Null", 0},
	{"Sensor_gyro", "GYROSCOPE", "MTK", "Null", 0},
    {"Sensor_accel", "BMA253", "BOSCH", "Null", 0},
    {"Sensor_msensor", "AKM09918", "AKM", "Null", 0}
#else
    {"Sensor_alsps", "Null", "Null","Null",0},
	{"Sensor_gyro", "Null", "Null","Null",0},
    {"Sensor_accel", "Null", "Null","Null",0},
    {"Sensor_msensor", "Null", "Null","Null",0}
#endif
};

int devinfo_search_item(char *item_name)
{
    int i = 0;

    for(i = 0; i < ARRAY_SIZE(g_devinfo_items); i++)
    {
        if(strcmp(item_name, g_devinfo_items[i].device_name) == 0)
        {
            return i;
        }
    }

    return -1;
}

static ssize_t devinfo_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int item_index = -1;
    char devinfo_temp[ITEM_LENGTH*3];

    if(*ppos)
        return 0;
    *ppos += count;

    item_index = devinfo_search_item(file->f_path.dentry->d_iname);

    if(item_index < 0)
    {
        pr_err("[devinfo][ERR]: devinfo_proc_read %s fail\n", file->f_path.dentry->d_iname);
        return 0;
    }

    snprintf(devinfo_temp, ITEM_LENGTH*3, "Device version: %s\n", g_devinfo_items[item_index].device_version);
    snprintf(devinfo_temp, ITEM_LENGTH*3, "%sDevice manufacture: %s\n", devinfo_temp, g_devinfo_items[item_index].device_manufacture);
	if(g_devinfo_items[item_index].need_fw_path)
		snprintf(devinfo_temp, ITEM_LENGTH*3, "%sDevice fw_path:\t\t%s\n", devinfo_temp, g_devinfo_items[item_index].fw_path);

    if (copy_to_user(buf, devinfo_temp, strlen(devinfo_temp) + 1)) {
        pr_err("%s: copy to user error.", __func__);
        return -1;
    }

    return strlen(devinfo_temp);
}

static void _devinfo_info_set(char *name, char *version, char *manufacture,char *fw_path)
{
    int item_index = -1;

    item_index = devinfo_search_item(name);

    if(item_index < 0)
    {
        pr_err("[devinfo][ERR]: devinfo_info_set %s fail\n", name);
        return;
    }

    memset(g_devinfo_items[item_index].device_version, 0, ITEM_LENGTH);
    strncpy(g_devinfo_items[item_index].device_version, version, ITEM_LENGTH);
    memset(g_devinfo_items[item_index].device_manufacture, 0, ITEM_LENGTH);
    strncpy(g_devinfo_items[item_index].device_manufacture, manufacture, ITEM_LENGTH);
    memset(g_devinfo_items[item_index].fw_path, 0, ITEM_LENGTH);
	strncpy(g_devinfo_items[item_index].fw_path, fw_path, ITEM_LENGTH);

    return;
}

void devinfo_info_tp_set(char *version, char *manufacture, char *fw_path)
{
    _devinfo_info_set("tp", version, manufacture, fw_path);
    return;
}
EXPORT_SYMBOL_GPL(devinfo_info_tp_set);

void devinfo_info_set(char *name, char *version, char *manufacture)
{
	_devinfo_info_set(name, version, manufacture, "Null");
	return;
}
EXPORT_SYMBOL_GPL(devinfo_info_set);

static const struct file_operations devinfo_fops =
{
    .write = NULL,
    .read  = devinfo_proc_read,
    .owner = THIS_MODULE,
};

#ifdef ODM_WT_EDIT
int proc_devinfo_init( struct proc_dir_entry *devinfo_dir)
{
    int i;
    //struct proc_dir_entry *devinfo_dir;
    struct proc_dir_entry *devinfo_item;

   // devinfo_dir = proc_mkdir(DEVINFO_PATH, NULL);
    //if (!devinfo_dir)
   // {
   //     pr_err("[proc_devinfo_init][ERR]: create %s dir fail\n", DEVINFO_PATH);
   //     return -1;
   // }

    for(i = 0; i < ARRAY_SIZE(g_devinfo_items); i++)
    {
        devinfo_item = proc_create(g_devinfo_items[i].device_name, 0444, devinfo_dir, &devinfo_fops);
        if (devinfo_item == NULL)
            pr_err("[proc_devinfo_init][ERR]: create %s fail\n", g_devinfo_items[i].device_name);
    }
	
    return 0;
}
//fs_initcall(proc_devinfo_init);
#endif /*ODM_WT_EDIT*/
