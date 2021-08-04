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
#include <linux/update_tpfw_notifier.h>

ATOMIC_NOTIFIER_HEAD(tpfw_notifier_list);
EXPORT_SYMBOL_GPL(tpfw_notifier_list);

int update_tpfw_register_client(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&tpfw_notifier_list, nb);

}
EXPORT_SYMBOL(update_tpfw_register_client);

int update_tpfw_unregister_client(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&tpfw_notifier_list, nb);
}
EXPORT_SYMBOL(update_tpfw_unregister_client);


int update_tpfw_notifier_call_chain(unsigned long val, void *v)
{
	return atomic_notifier_call_chain(&tpfw_notifier_list, val, v);

}
EXPORT_SYMBOL_GPL(update_tpfw_notifier_call_chain);


