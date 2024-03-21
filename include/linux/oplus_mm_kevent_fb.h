/***************************************************************
** Copyright (C),  2018,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_mm_kevent_fb.h
** Description : MM kevent fb data
** Version : 1.0
** Date : 2018/12/03
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Guo.Ling          2018/12/03        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_MM_KEVENT_FB_
#define _OPLUS_MM_KEVENT_FB_

enum OPLUS_MM_DIRVER_FB_EVENT_ID {
	OPLUS_MM_DIRVER_FB_EVENT_ID_ESD = 401,
	OPLUS_MM_DIRVER_FB_EVENT_ID_VSYNC,
	OPLUS_MM_DIRVER_FB_EVENT_ID_HBM,
	OPLUS_MM_DIRVER_FB_EVENT_ID_FFLSET,
	OPLUS_MM_DIRVER_FB_EVENT_ID_MTK_CMDQ,
	OPLUS_MM_DIRVER_FB_EVENT_ID_MTK_UNDERFLOW,
	OPLUS_MM_DIRVER_FB_EVENT_ID_MTK_FENCE,
	OPLUS_MM_DIRVER_FB_EVENT_ID_MTK_GPU_JS,
	OPLUS_MM_DIRVER_FB_EVENT_ID_MTK_GPU_SOFT_RESET,
	OPLUS_MM_DIRVER_FB_EVENT_ID_MTK_GPU_MMU_FAULT,
	OPLUS_MM_DIRVER_FB_EVENT_ID_MTK_GPU_FAULT,
	OPLUS_MM_DIRVER_FB_EVENT_ID_AUDIO = 801,
};

enum OPLUS_MM_DIRVER_FB_EVENT_MODULE {
	OPLUS_MM_DIRVER_FB_EVENT_MODULE_DISPLAY = 0,
	OPLUS_MM_DIRVER_FB_EVENT_MODULE_AUDIO
};

enum OPLUS_MM_DIRVER_FB_EVENT_REPORTLEVEL {
	OPLUS_MM_DIRVER_FB_EVENT_REPORTLEVEL_LOW = 0,
	OPLUS_MM_DIRVER_FB_EVENT_REPORTLEVEL_HIGH
};

int upload_mm_kevent_fb_data(enum OPLUS_MM_DIRVER_FB_EVENT_MODULE module, unsigned char *payload);

#endif /* _OPLUS_MM_KEVENT_FB_ */

