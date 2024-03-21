/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#ifndef __CCCI_FSM_SYS_H__
#define __CCCI_FSM_SYS_H__

#define AED_STR_LEN		(2048)

struct mdee_info_collect {
	spinlock_t mdee_info_lock;
	char mdee_info[AED_STR_LEN];
};

void fsm_sys_mdee_info_notify(char *buf);
int fsm_sys_init(void);
//#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
#define MODEM_MONITOR_ID          509    //modem crash
#define BUF_LOG_LENGTH            2148
int getSubstrIndex(char *srcStr, char *subStr);
void deleteChar(char* str, int strLen, int index);
unsigned int BKDRHash(const char* str, unsigned int len, int md_id);
//#endif /*OPLUS_FEATURE_MODEM_MINIDUMP*/
#endif /* __CCCI_FSM_SYS_H__ */
