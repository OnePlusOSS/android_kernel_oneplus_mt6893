/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_TCPCI_TYPEC_H
#define __LINUX_TCPCI_TYPEC_H
#include "tcpci.h"

struct tcpc_device;

/******************************************************************************
 *  Call following function to trigger TYPEC Connection State Change
 *
 * 1. H/W -> CC/PS Change.
 * 2. Timer -> CCDebounce or PDDebounce or others Timeout
 * 3. Policy Engine -> PR_SWAP, Error_Recovery, PE_Idle
 *****************************************************************************/

/*
 * [BLOCK] TYPEC Connection State Definition
 */

enum TYPEC_CONNECTION_STATE {
	typec_disabled = 0,
	typec_errorrecovery,

	typec_unattached_snk,
	typec_unattached_src,

	typec_attachwait_snk,
	typec_attachwait_src,

	typec_attached_snk,
	typec_attached_src,

#ifdef CONFIG_TYPEC_CAP_TRY_SOURCE
	/* Require : Assert Rp
	 * Exit(-> Attached.SRC) : Detect Rd (tPDDebounce).
	 * Exit(-> TryWait.SNK) : Not detect Rd after tDRPTry
	 */
	typec_try_src,

	/* Require : Assert Rd
	 * Exit(-> Attached.SNK) : Detect Rp (tCCDebounce) and Vbus present.
	 * Exit(-> Unattached.SNK) : Not detect Rp (tPDDebounce)
	 */

	typec_trywait_snk,
	typec_trywait_snk_pe,
#endif

#ifdef CONFIG_TYPEC_CAP_TRY_SINK

	/* Require : Assert Rd
	 * Wait for tDRPTry and only then begin monitoring CC.
	 * Exit (-> Attached.SNK) : Detect Rp (tPDDebounce) and Vbus present.
	 * Exit (-> TryWait.SRC) : Not detect Rp for tPDDebounce.
	 */
	typec_try_snk,

	/*
	 * Require : Assert Rp
	 * Exit (-> Attached.SRC) : Detect Rd (tCCDebounce)
	 * Exit (-> Unattached.SNK) : Not detect Rd after tDRPTry
	 */

	typec_trywait_src,
	typec_trywait_src_pe,
#endif	/* CONFIG_TYPEC_CAP_TRY_SINK */

	typec_audioaccessory,
	typec_debugaccessory,

#ifdef CONFIG_TYPEC_CAP_DBGACC_SNK
	typec_attached_dbgacc_snk,
#endif	/* CONFIG_TYPEC_CAP_DBGACC_SNK */

#ifdef CONFIG_TYPEC_CAP_CUSTOM_SRC
	typec_attached_custom_src,
#endif	/* CONFIG_TYPEC_CAP_CUSTOM_SRC */

#ifdef CONFIG_TYPEC_CAP_NORP_SRC
	typec_attached_norp_src,
#endif	/* CONFIG_TYPEC_CAP_NORP_SRC */

#ifdef CONFIG_TYPEC_CAP_ROLE_SWAP
	typec_role_swap,
#endif	/* CONFIG_TYPEC_CAP_ROLE_SWAP */

#ifdef CONFIG_WATER_DETECTION
	typec_water_protection_wait,
	typec_water_protection,
#endif /* CONFIG_WATER_DETECTION */

	typec_unattachwait_pe,	/* Wait Policy Engine go to Idle */
};
extern int tcpc_typec_enter_lpm_again(struct tcpc_device *tcpc_dev);
extern int tcpc_typec_handle_cc_change(struct tcpc_device *tcpc_dev);

extern int tcpc_typec_handle_ps_change(
		struct tcpc_device *tcpc_dev, int vbus_level);

extern int tcpc_typec_handle_timeout(
		struct tcpc_device *tcpc_dev, uint32_t timer_id);

extern int tcpc_typec_handle_vsafe0v(struct tcpc_device *tcpc_dev);

extern int tcpc_typec_set_rp_level(struct tcpc_device *tcpc_dev, uint8_t res);

extern int tcpc_typec_error_recovery(struct tcpc_device *tcpc_dev);

extern int tcpc_typec_disable(struct tcpc_device *tcpc_dev);
extern int tcpc_typec_enable(struct tcpc_device *tcpc_dev);

extern int tcpc_typec_change_role(
	struct tcpc_device *tcpc_dev, uint8_t typec_role);

#ifdef CONFIG_USB_POWER_DELIVERY
extern int tcpc_typec_handle_pe_pr_swap(struct tcpc_device *tcpc_dev);
#endif /* CONFIG_USB_POWER_DELIVERY */

#ifdef CONFIG_TYPEC_CAP_ROLE_SWAP
extern int tcpc_typec_swap_role(struct tcpc_device *tcpc_dev);
#endif /* CONFIG_TYPEC_CAP_ROLE_SWAP */

#ifdef CONFIG_WATER_DETECTION
extern int tcpc_typec_handle_wd(struct tcpc_device *tcpc_dev, bool wd);
#endif /* CONFIG_WATER_DETECTION */

#ifdef CONFIG_CABLE_TYPE_DETECTION
extern int tcpc_typec_handle_ctd(struct tcpc_device *tcpc_dev,
				 enum tcpc_cable_type cable_type);
#endif /* CONFIG_CABLE_TYPEC_DETECTION */

#endif /* #ifndef __LINUX_TCPCI_TYPEC_H */
