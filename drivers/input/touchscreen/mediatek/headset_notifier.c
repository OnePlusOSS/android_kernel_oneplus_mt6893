/*******************************************************************************
** Copyright (C), 2008-2016, OPLUS Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - hiamx_common.c
** Description : This program is for headset notifier
** Version: 1.0
** Date : 2018/11/27
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/

#include <linux/notifier.h>
#include <linux/headset_notifier.h>

ATOMIC_NOTIFIER_HEAD(headset_notifier_list);
EXPORT_SYMBOL_GPL(headset_notifier_list);

int headset_register_client(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&headset_notifier_list, nb);

}
EXPORT_SYMBOL(headset_register_client);

int headset_unregister_client(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&headset_notifier_list, nb);
}
EXPORT_SYMBOL(headset_unregister_client);


int headset_notifier_call_chain(unsigned long val, void *v)
{
	return atomic_notifier_call_chain(&headset_notifier_list, val, v);

}
EXPORT_SYMBOL_GPL(headset_notifier_call_chain);


