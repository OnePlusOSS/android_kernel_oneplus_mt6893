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
};

#endif /*_OPLUS_DISPLAY_MTK_DEBUG_H_*/
