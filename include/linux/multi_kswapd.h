/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef __OPLUS_MULTI_KSWAPD__
#define __OPLUS_MULTI_KSWAPD__

#define MAX_KSWAPD_THREADS 16

extern int kswapd_threads;
extern int max_kswapd_threads;

extern int kswapd_threads_sysctl_handler(struct ctl_table *, int,
					void __user *, size_t *, loff_t *);
extern void update_kswapd_threads(void);
extern int kswapd_cpu_online_ext(unsigned int cpu);
extern int cpu_callback_ext(struct notifier_block *nfb, unsigned long action,
			void *hcpu);
extern int kswapd_run_ext(int nid);
extern void kswapd_stop_ext(int nid);
extern int kswapd(void *p);


#endif /*__OPLUS_MULTI_KSWAPD__*/
