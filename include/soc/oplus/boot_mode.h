/************************************************************************************
** File: - android\kernel\arch\arm\mach-msm\include\mach\oplus_boot.h
** Copyright (C), 2008-2012, OPLUS Mobile Comm Corp., Ltd
** 
** Description:  
**     change define of boot_mode here for other place to use it
** Version: 1.0 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
************************************************************************************/
#ifndef _OPLUS_BOOT_H
#define _OPLUS_BOOT_H
#include <soc/oplus/boot_mode_types.h>
//extern static int get_boot_mode(void);
extern int get_boot_mode(void);
//#endif /* VENDOR_EDIT */
extern bool qpnp_is_power_off_charging(void);
extern bool qpnp_is_charger_reboot(void);
#endif
