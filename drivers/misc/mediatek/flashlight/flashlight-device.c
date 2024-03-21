// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "flashlight-core.h"

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#if defined(mt6739)
const struct flashlight_device_id flashlight_id[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-rt4505", 0, 0},
};
#elif defined(mt6757)
	#if defined(evb6757p_dm_64) || defined(k57pv1_dm_64) || \
	defined(k57pv1_64_baymo) || defined(k57pv1_dm_64_bif) || \
	defined(k57pv1_dm_64_baymo) || defined(k57pv1_dm_teei_2g) || \
	defined(k57pv1_dm_64_zoom)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-rt5081", 0, 0},
		{0, 1, 0, "flashlights-rt5081", 1, 0},
	};
	#elif defined(CONFIG_MTK_FLASHLIGHT_RT5081)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-rt5081", 0, 0},
		{0, 1, 0, "flashlights-rt5081", 1, 0},
	};
	#else
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-lm3643", 0, 0},
		{0, 1, 0, "flashlights-lm3643", 1, 0},
	};
	#endif
#elif defined(mt6758)
const struct flashlight_device_id flashlight_id[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-mt6370", 0, 0},
	{0, 1, 0, "flashlights-mt6370", 1, 0},
};
#elif defined(mt6759)
const struct flashlight_device_id flashlight_id[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-rt5081", 0, 0},
	{0, 1, 0, "flashlights-rt5081", 1, 0},
};
#elif defined(mt6761)
	#ifdef CONFIG_MTK_FLASHLIGHT_AW3644
	const struct flashlight_device_id flashlight_id[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-aw3644", 0, 1},
		{1, 0, 0, "flashlights-aw3644", 1, 1},
	};
	#else
	const struct flashlight_device_id flashlight_id[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-none", -1, 0},
		{0, 1, 0, "flashlights-none", -1, 0},
		{1, 0, 0, "flashlights-none", -1, 0},
		{1, 1, 0, "flashlights-none", -1, 0},
		{0, 0, 1, "flashlights-none", -1, 0},
		{0, 1, 1, "flashlights-none", -1, 0},
		{1, 0, 1, "flashlights-none", -1, 0},
		{1, 1, 1, "flashlights-none", -1, 0},
	};
	#endif
#elif defined(mt6763)
const struct flashlight_device_id flashlight_id[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-mt6370", 0, 0},
	{0, 1, 0, "flashlights-mt6370", 1, 0},
};
#elif defined(mt6799)
const struct flashlight_device_id flashlight_id[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-mt6336", 0, 0},
	{0, 1, 0, "flashlights-mt6336", 1, 0},
};
#elif defined(mt8167)
const struct flashlight_device_id flashlight_id[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-lm3642", 0, 0},
};
#elif defined(mt6785)
const struct flashlight_device_id flashlight_id[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-mt6360", 0, 1},
};
#elif defined(CONFIG_MTK_FLASHLIGHT_SYWT78)
const struct flashlight_device_id flashlight_id[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-sywt78", 0, 0},
};
#else
const struct flashlight_device_id flashlight_id[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-none", -1, 0},
	{0, 1, 0, "flashlights-none", -1, 0},
	{1, 0, 0, "flashlights-none", -1, 0},
	{1, 1, 0, "flashlights-none", -1, 0},
	{0, 0, 1, "flashlights-none", -1, 0},
	{0, 1, 1, "flashlights-none", -1, 0},
	{1, 0, 1, "flashlights-none", -1, 0},
	{1, 1, 1, "flashlights-none", -1, 0},
};
#endif
const int flashlight_device_num =
	sizeof(flashlight_id) / sizeof(struct flashlight_device_id);
#else /*OPLUS_FEATURE_CAMERA_COMMON*/
const struct flashlight_device_id flashlight_id_dual[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-mt6360", 0, 0},
	{0, 1, 0, "flashlights-mt6360", 1, 0},
};
const struct flashlight_device_id flashlight_id_single[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-mt6360", 0, 1},
};
const struct flashlight_device_id flashlight_id_zhaoyun[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-zhaoyun", 0, 1},
};

const struct flashlight_device_id flashlight_id_limu[] = {
	{0, 0, 0, "flashlights_limu", -1, 1},
};
const struct flashlight_device_id flashlight_id_parkera[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-parkera", 0, 1},
};

const struct flashlight_device_id flashlight_id_parkerb[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-parkerb", 0, 1},
};

const struct flashlight_device_id flashlight_id_miami[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights_miami", 0, 1},
};

const struct flashlight_device_id flashlight_id_even[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights_even", 0, 1},
};

const struct flashlight_device_id flashlight_id_aladdin[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-aladdin", 0, 1},
};

const struct flashlight_device_id flashlight_id_sy6560[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-sy6560", 0, 1},
};

const struct flashlight_device_id flashlight_id_atom[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-atom", 0, 1},
};

#endif //OPLUS_FEATURE_CAMERA_COMMON
