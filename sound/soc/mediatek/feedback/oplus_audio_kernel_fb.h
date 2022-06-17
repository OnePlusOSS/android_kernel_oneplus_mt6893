/************************************************************************************
** File: - vendor\oplus\kernel\audio\include\oplus_audio_kernel_fb.h
** CONFIG_OPLUS_FEATURE_MM_FEEDBACK
** Copyright (C), 2022-2024, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**     add oplus audio kernel feedback
** Version: 1.0
** --------------------------- Revision History: --------------------------------
**      <author>                                        <date>              <desc>
************************************************************************************/
#ifndef OPLUS_AUDIO_KERNEL_FB_H
#define OPLUS_AUDIO_KERNEL_FB_H

#include <linux/device.h>
#include <soc/oplus/system/oplus_mm_kevent_fb.h>

#define OPLUS_AUDIO_EVENTID_AUDIO_KERNEL_ERR   (10047)
#define FEEDBACK_RATELIMIT_INTERVAL            (300 * HZ)
#define FEEDBACK_RATELIMIT_BURST               1

#define ratelimited_fb(fmt, ...) \
	do { \
		static DEFINE_RATELIMIT_STATE(_rs, \
						  FEEDBACK_RATELIMIT_INTERVAL, \
						  FEEDBACK_RATELIMIT_BURST); \
		if (__ratelimit(&_rs)) \
			mm_fb_audio_kevent_named(OPLUS_AUDIO_EVENTID_AUDIO_KERNEL_ERR, \
					MM_FB_KEY_RATELIMIT_30MIN, fmt, ##__VA_ARGS__); \
	} while (0)

/* print error log and feedback */
#define pr_err_fb(fmt, ...) \
	do { \
		printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__); \
		ratelimited_fb("payload@@"fmt, ##__VA_ARGS__); \
	} while (0)

#endif /* OPLUS_AUDIO_KERNEL_FB_H */

