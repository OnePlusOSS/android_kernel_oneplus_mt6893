/************************************************************************************
** File: - drivers/soc/oplus/oplus_criticallog/criticallog_class.h
** Copyright (C), 2008-2019, OPLUS Mobile Comm Corp., Ltd
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

#ifndef __LINUX_CRITICALLOG_H__
#define __LINUX_CRITICALLOG_H__

//#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
struct criticallog_dev {
	const char *name;
	struct device *dev;
	int index;
	int logid;
	ssize_t (*print_logInfo)(struct criticallog_dev *sdev, char *buf);
	ssize_t (*print_logId)(struct criticallog_dev *sdev, char *buf);
};

struct gpio_criticallog_platform_data {
	const char *name;
	unsigned gpio;
	/* if NULL, criticallog_dev.name will be printed */
	const char *name_on;
	const char *name_off;
	/* if NULL, "0" or "1" will be printed */
	const char *state_on;
	const char *state_off;
};

extern int criticallog_dev_register(struct criticallog_dev *sdev);
extern void criticallog_dev_unregister(struct criticallog_dev *sdev);

static inline int criticallog_get_logid(struct criticallog_dev *sdev)
{
	return sdev->logid;
}

extern void criticallog_set_state(struct criticallog_dev *sdev, int logid, char *subsys);
//#endif /*OPLUS_FEATURE_MODEM_MINIDUMP*/

#endif /* __LINUX_CRITICALLOG_H__ */
