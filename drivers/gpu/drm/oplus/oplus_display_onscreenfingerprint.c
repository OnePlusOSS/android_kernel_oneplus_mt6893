/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_onscreenfingerprint.h
** Description : oplus_display_onscreenfingerprint. implement
** Version : 1.0
** Date : 2020/05/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Zhang.JianBin2020/05/13        1.0          Modify for MT6779_R
******************************************************************/
#include "oplus_display_onscreenfingerprint.h"

extern bool oplus_fp_notify_down_delay;
extern bool oplus_fp_notify_up_delay;
bool oplus_doze_fpd_nodelay;

int oplus_display_panel_set_finger_print(void *buf)
{
	unsigned int *fingerprint_op_mode = buf;

	if ((*fingerprint_op_mode) == 1) {
		oplus_fp_notify_down_delay = true;
	} else {
		oplus_fp_notify_up_delay = true;
	}

	printk(KERN_ERR "%s receive uiready %d\n", __func__, (*fingerprint_op_mode));
	return 0;
}

int notify_display_fpd(bool mode) {
	pr_debug("lcm mode = %d\n", mode);
	oplus_doze_fpd_nodelay = mode;
	return 0;
}

/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
