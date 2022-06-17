/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef _OPLUS_MISC_HEALTHONFO_
#define _OPLUS_MISC_HEALTHONFO_
#include <linux/time.h>
#include <linux/printk.h>

extern unsigned int omh_debug;

#define TIME_COUNT	12

#define OMH_INFO(fmt, args...) \
	pr_info("OPLUS_MISC: healthinfo :%s:" fmt "\n", __func__, ##args)

#define LEVEL_DEBUG 1

#define OMH_DEBUG(fmt, args...) \
	do { \
		if (LEVEL_DEBUG == omh_debug) \
		pr_info("OPLUS_MISC: healthinfo :%s:" fmt "\n", __func__, ##args);\
	}while(0)

enum lights_status {
	LIGHTS_RED = 0,
	LIGHTS_GREEN,
	LIGHTS_BLUE,
	LIGHTS_ON,
	LIGHTS_OFF,
	DEFAULT,
};

struct oplus_misc_healthinfo_para {
	struct platform_device		*my_pdev;
	struct proc_dir_entry		*my_prEntry_cr;
	struct device				*my_dev;
	struct mutex				my_lock;
	unsigned int				omhp_support;
};

#endif /* _OPLUS_MISC_HEALTHONFO_ */

