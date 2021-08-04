/*
 * Copyright (C) 2014 NXP Semiconductors, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/*
 *internal functions for TFA layer (not shared with SRV and HAL layer!)
 */

#ifndef __TFA_DSP_H__
#define __TFA_DSP_H__

extern int ftm_mode;
extern char ftm_SpeakerCalibration[17];
extern char ftm_spk_resistance[24];

#ifndef BOOT_MODE_FACTORY
#define BOOT_MODE_FACTORY 3
#endif

bool g_speaker_resistance_fail;

#endif /* __TFA_DSP_H__ */

