/*******************************************************************************
** Copyright (C), 2008-2016, OPLUS Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - update_tpfw_notifier.c
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
#include <linux/tp_usb_notifier.h>

ATOMIC_NOTIFIER_HEAD(tp_usb_list);
EXPORT_SYMBOL_GPL(tp_usb_list);

int tp_usb_register_client(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&tp_usb_list, nb);

}
EXPORT_SYMBOL(tp_usb_register_client);

int tp_usb_unregister_client(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&tp_usb_list, nb);
}
EXPORT_SYMBOL(tp_usb_unregister_client);


int tp_usb_notifier_call_chain(unsigned long val, void *v)
{
	return atomic_notifier_call_chain(&tp_usb_list, val, v);

}
EXPORT_SYMBOL_GPL(tp_usb_notifier_call_chain);


