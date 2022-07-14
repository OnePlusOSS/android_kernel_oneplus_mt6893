/************************************************************************************
** File: - drivers/soc/oplus/oplus_criticallog/criticallog_class.c
** Copyright (C), 2008-2012, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**      critical log dev
**
** Version: 0.1
** Date created: 11:28:11,16/01/2019
**
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
**
************************************************************************************/

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <criticallog_class.h>

//#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
#define MAX_REASON_LEN 256

struct class *criticallog_class;
static atomic_t device_count;

static ssize_t get_keylog_logid(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct criticallog_dev *sdev = (struct criticallog_dev *)
	dev_get_drvdata(dev);

	if (sdev->print_logId) {
		int ret = sdev->print_logId(sdev, buf);
		if (ret >= 0)
			return ret;
	}
	return sprintf(buf, "%d\n", sdev->logid);
}

static ssize_t get_modem_reason(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct criticallog_dev *sdev = (struct criticallog_dev *)
	dev_get_drvdata(dev);

	if (sdev->print_logInfo) {
		int ret = sdev->print_logInfo(sdev, buf);
		if (ret >= 0)
			return ret;
	}
	return sprintf(buf, "%s\n", sdev->name);
}

static DEVICE_ATTR(logid, S_IRUGO, get_keylog_logid, NULL);
static DEVICE_ATTR(name, S_IRUGO, get_modem_reason, NULL);

void criticallog_set_state(struct criticallog_dev *sdev, int logid, char *subsys)
{
	char modemReason_buf[MAX_REASON_LEN];
	char modemHashReason_buf[MAX_REASON_LEN];
	char modem_event[] = "MODEM_EVENT=modem_failure";
	char *prop_buf;
	char *envp[4];
	int length;

	memset(modemReason_buf, 0, sizeof(modemReason_buf));
	memset(modemHashReason_buf, 0, sizeof(modemHashReason_buf));

	if (strcmp(subsys, "modem") == 0) {
		envp[0] = (char *)&modem_event;
		prop_buf = (char *)get_zeroed_page(GFP_KERNEL);
		if (prop_buf) {
			length = get_modem_reason(sdev->dev, NULL, prop_buf);
			if (length > 0) {
				if (prop_buf[length - 1] == '\n')
					prop_buf[length - 1] = 0;
				snprintf(modemReason_buf, sizeof(modemReason_buf), "MODEM_REASON=%s", prop_buf);
				envp[1] = modemReason_buf;
				snprintf(modemHashReason_buf, sizeof(modemHashReason_buf), "MODEM_HASH_REASON=%s", prop_buf);
				envp[2] = modemHashReason_buf;
			}
			printk(KERN_ERR "criticallog: criticallog_set_state length  = %d, modemHashReason_buf = %s\n", length, modemHashReason_buf);
			envp[3] = NULL;
			kobject_uevent_env(&sdev->dev->kobj, KOBJ_CHANGE, envp);
			free_page((unsigned long)prop_buf);
		} else {
			printk(KERN_ERR "out of memory in criticallog_set_state\n");
			kobject_uevent(&sdev->dev->kobj, KOBJ_CHANGE);
		}
	}
}
EXPORT_SYMBOL_GPL(criticallog_set_state);

static int create_criticallog_class(void)
{
	if (!criticallog_class) {
		criticallog_class = class_create(THIS_MODULE, "critical_log");
		if (IS_ERR(criticallog_class))
			return PTR_ERR(criticallog_class);
		atomic_set(&device_count, 0);
	}
	return 0;
}

int criticallog_dev_register(struct criticallog_dev *sdev)
{
	int ret;
	if (!criticallog_class) {
		ret = create_criticallog_class();
		if (ret < 0)
			return ret;
	}
	sdev->index = atomic_inc_return(&device_count);
	sdev->dev = device_create(criticallog_class, NULL,
	MKDEV(0, sdev->index), NULL, sdev->name);
	if (IS_ERR(sdev->dev))
		return PTR_ERR(sdev->dev);
	ret = device_create_file(sdev->dev, &dev_attr_logid);
	if (ret < 0)
		goto err_create_file_1;
	ret = device_create_file(sdev->dev, &dev_attr_name);
	if (ret < 0)
		goto err_create_file_2;
	dev_set_drvdata(sdev->dev, sdev);
	sdev->logid = 0;
	return 0;

err_create_file_2:
	device_remove_file(sdev->dev, &dev_attr_logid);
	return ret;

err_create_file_1:
	device_destroy(criticallog_class, MKDEV(0, sdev->index));
	printk(KERN_ERR "criticallog: Failed to register driver %s\n", sdev->name);

	return ret;
}
EXPORT_SYMBOL_GPL(criticallog_dev_register);

void criticallog_dev_unregister(struct criticallog_dev *sdev)
{
	device_remove_file(sdev->dev, &dev_attr_name);
	device_remove_file(sdev->dev, &dev_attr_logid);
	device_destroy(criticallog_class, MKDEV(0, sdev->index));
	dev_set_drvdata(sdev->dev, NULL);
}
EXPORT_SYMBOL_GPL(criticallog_dev_unregister);

static int __init criticallog_class_init(void)
{
	return create_criticallog_class();
}

static void __exit criticallog_class_exit(void)
{
	class_destroy(criticallog_class);
}

module_init(criticallog_class_init);
module_exit(criticallog_class_exit);

MODULE_AUTHOR("Zhaoan.Xu");
MODULE_DESCRIPTION("criticallog class driver");
MODULE_LICENSE("GPL");
//#endif /*OPLUS_FEATURE_MODEM_MINIDUMP*/
