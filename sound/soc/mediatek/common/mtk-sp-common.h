/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mtk-sp-common.h  --  Mediatek Smart Phone Common
 *
 * Copyright (c) 2017 MediaTek Inc.
 * Author: Kai Chieh Chuang <kaichieh.chuang@mediatek.com>
 */

#ifndef _MTK_SP_COMMON_H_
#define _MTK_SP_COMMON_H_

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
#include "../feedback/oplus_audio_kernel_fb.h"
#endif

#if defined(CONFIG_MTK_AEE_FEATURE)
#include <mt-plat/aee.h>

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
#define AUDIO_AEE(message) \
	do { \
		ratelimited_fb("payload@@AUDIO_AEE:"message); \
		(aee_kernel_exception_api(__FILE__, \
					  __LINE__, \
					  DB_OPT_FTRACE, message, \
					  "audio assert")); \
	} while (0)
#else /* CONFIG_OPLUS_FEATURE_MM_FEEDBACK */
#define AUDIO_AEE(message) \
	(aee_kernel_exception_api(__FILE__, \
				  __LINE__, \
				  DB_OPT_FTRACE, message, \
				  "audio assert"))
#endif /* CONFIG_OPLUS_FEATURE_MM_FEEDBACK */
#else
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
#define AUDIO_AEE(message) \
	do { \
		ratelimited_fb("payload@@AUDIO_AEE:"message); \
		WARN_ON(true); \
	} while (0)
#else /* CONFIG_OPLUS_FEATURE_MM_FEEDBACK */
#define AUDIO_AEE(message) WARN_ON(true)
#endif /*CONFIG_OPLUS_FEATURE_MM_FEEDBACK*/
#endif

bool mtk_get_speech_status(void);

#endif
