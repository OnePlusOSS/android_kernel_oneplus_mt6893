/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_mtk_debug.c
** Description : oplus display mtk debug
** Version : 1.0
** Date : 2020/12/06
**
** ------------------------------- Revision History: -----------
**  <author>        <date>        <version >        <desc>
**  Xiaolei.Gao    2020/12/06        1.0           Build this moudle
******************************************************************/
#include "oplus_display_mtk_debug.h"
#include "display_panel/oplus_display_panel.h"

extern bool g_mobile_log;
extern bool g_detail_log;
extern bool g_irq_log;
extern bool g_fence_log;

int oplus_display_set_mtk_loglevel(void *buf)
{
	struct kernel_loglevel *loginfo = buf;
	unsigned int enabled = 0;
	unsigned int loglevel = 0;

	enabled = loginfo->enable;
	loglevel = loginfo->log_level;

	printk("%s,mtk log level is 0x%x,enable=%d", __func__, loglevel, enabled);

	if (enabled == 1) {
		if ((loglevel & MTK_LOG_LEVEL_MOBILE_LOG) == MTK_LOG_LEVEL_MOBILE_LOG)
			g_mobile_log = true;
		if ((loglevel & MTK_LOG_LEVEL_DETAIL_LOG) == MTK_LOG_LEVEL_DETAIL_LOG)
			g_detail_log = true;
		if ((loglevel & MTK_LOG_LEVEL_FENCE_LOG) == MTK_LOG_LEVEL_FENCE_LOG)
			g_fence_log = true;
		if ((loglevel & MTK_LOG_LEVEL_IRQ_LOG) == MTK_LOG_LEVEL_IRQ_LOG)
			g_irq_log = true;
	} else {
		if ((loglevel & MTK_LOG_LEVEL_MOBILE_LOG) == MTK_LOG_LEVEL_MOBILE_LOG)
			g_mobile_log = false;
		if ((loglevel & MTK_LOG_LEVEL_DETAIL_LOG) == MTK_LOG_LEVEL_DETAIL_LOG)
			g_detail_log = false;
		if ((loglevel & MTK_LOG_LEVEL_FENCE_LOG) == MTK_LOG_LEVEL_FENCE_LOG)
			g_fence_log = false;
		if ((loglevel & MTK_LOG_LEVEL_IRQ_LOG) == MTK_LOG_LEVEL_IRQ_LOG)
			g_irq_log = false;
	}

	return 0;
}

