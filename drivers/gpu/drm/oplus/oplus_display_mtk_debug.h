/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_mtk_debug.h
** Description : oplus display mtk debug
** Version : 1.0
** Date : 2020/06/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Xiaolei.Gao       2020/12/06        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DISPLAY_MTK_DEBUG_H_
#define _OPLUS_DISPLAY_MTK_DEBUG_H_

#include <linux/fs.h>
#include <linux/printk.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>

enum {
	MTK_LOG_LEVEL_MOBILE_LOG = 0x1,
	MTK_LOG_LEVEL_DETAIL_LOG = 0x2,
	MTK_LOG_LEVEL_FENCE_LOG = 0x4,
	MTK_LOG_LEVEL_IRQ_LOG = 0x8,
	MTK_LOG_LEVEL_TRACE_LOG = 0x10,
};

enum oplus_disp_drv_log_level {
	OPLUS_DISP_DRV_LOG_LEVEL_ERR = 0,
	OPLUS_DISP_DRV_LOG_LEVEL_WARN = 1,
	OPLUS_DISP_DRV_LOG_LEVEL_INFO = 2,
	OPLUS_DISP_DRV_LOG_LEVEL_DEBUG = 3,
};

/* log level config */
extern int oplus_disp_drv_log_level;
extern bool g_mobile_log;
extern bool g_detail_log;

/* just use for ofp */
#define DISP_ERR(fmt, arg...)	\
	do {	\
		if (oplus_disp_drv_log_level >= OPLUS_DISP_DRV_LOG_LEVEL_ERR)	\
			pr_err("[DISP_DRV][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define DISP_WARN(fmt, arg...)	\
	do {	\
		if (oplus_disp_drv_log_level >= OPLUS_DISP_DRV_LOG_LEVEL_WARN)	\
			pr_warn("[DISP_DRV][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define DISP_INFO(fmt, arg...)	\
	do {	\
		if ((oplus_disp_drv_log_level >= OPLUS_DISP_DRV_LOG_LEVEL_INFO) || (g_mobile_log)) \
			pr_info("[DISP_DRV][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define DISP_DEBUG(fmt, arg...)	\
	do {	\
		if ((oplus_disp_drv_log_level >= OPLUS_DISP_DRV_LOG_LEVEL_DEBUG) || (g_detail_log)) \
			pr_info("[DISP_DRV][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#endif /*_OPLUS_DISPLAY_MTK_DEBUG_H_*/
