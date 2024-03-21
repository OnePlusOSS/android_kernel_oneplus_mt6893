/*
 * Copyright (C) 2019 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __PARKERA_HOLITECH_S5K4H7_OTP_H__
#define __PARKERA_HOLITECH_S5K4H7_OTP_H__
#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif
extern  void PARKERA_HOLITECH_S5K4H7_write_cmos_sensor(u16 addr, u32 para);
extern  unsigned char PARKERA_HOLITECH_S5K4H7_read_cmos_sensor(u32 addr);

struct PARKERA_HOLITECH_S5K4H7_otp_struct {
int flag;
int MID;
int LID;
int RGr_ratio;
int BGr_ratio;
int GbGr_ratio;
int VCM_start;
int VCM_end;
};
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define RGr_ratio_Typical    287
#define BGr_ratio_Typical    325
#define GbGr_ratio_Typical   515
#else
#define RGr_ratio_Typical    583
#define BGr_ratio_Typical    687
#define GbGr_ratio_Typical  1024
#endif
#endif
