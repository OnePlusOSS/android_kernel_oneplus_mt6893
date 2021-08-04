/************************************************************************************
** File: -
** Copyright (C), 2020-2025, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**     add hal feedback
** Version: 1.0
** --------------------------- Revision History: --------------------------------
**               <author>                                <date>          <desc>
**
************************************************************************************/
#include <sound/control.h>
#include <linux/uaccess.h>
#include <soc/oplus/system/oplus_mm_kevent_fb.h>

#define HAL_FEEDBACK_MAX_BYTES         (256)
#define AUDIO_EVENTID_HAL_ERR          (10008)
#define OPLUS_FB_HAL_ERR_RATELIMIT     (60*1000)

int hal_feedback_config_get(struct snd_kcontrol *kcontrol,
			unsigned int __user *bytes,
			unsigned int size)
{
	return 0;
}
EXPORT_SYMBOL(hal_feedback_config_get);

int hal_feedback_config_set(struct snd_kcontrol *kcontrol,
			const unsigned int __user *bytes,
			unsigned int size)
{
	int ret = 0;
	char info[HAL_FEEDBACK_MAX_BYTES + 1] = {0};
	unsigned int len = size;

	if (len > HAL_FEEDBACK_MAX_BYTES) {
		len = HAL_FEEDBACK_MAX_BYTES;
		pr_info("%s(), size(%d) > Max bytes(%d)",
				__func__, size, HAL_FEEDBACK_MAX_BYTES);
	}

	if (copy_from_user(info, bytes, len)) {
		pr_err("%s(), Fail copy to user Ptr:(%p),len:%d",
				__func__, bytes, len);
		ret = -EFAULT;
	} else {
                info[len] = '\0';
		ret = upload_mm_fb_kevent_to_atlas_limit(AUDIO_EVENTID_HAL_ERR, info,
				OPLUS_FB_HAL_ERR_RATELIMIT);
	}

	return ret;
}
EXPORT_SYMBOL(hal_feedback_config_set);

