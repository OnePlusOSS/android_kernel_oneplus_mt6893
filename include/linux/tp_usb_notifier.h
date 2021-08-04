/*******************************************************************************
** Copyright (C), 2008-2016, OPLUS Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - update_tpfw_notifier.h
** Description : This program is for touch firmware update notifier
** Version: 1.0
** Date : 2018/11/27
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/
 
#include <linux/notifier.h>
#include <linux/export.h>




extern struct atomic_notifier_head tp_usb_list;

extern int tp_usb_register_client(struct notifier_block *nb);
extern int tp_usb_unregister_client(struct notifier_block *nb);
extern int tp_usb_notifier_call_chain(unsigned long val, void *v);

