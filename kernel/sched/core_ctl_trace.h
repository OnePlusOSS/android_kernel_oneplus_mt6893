/*
 * Copyright (c) 2019, Guangdong OPLUS Mobile Communication(Shanghai)
 * Corp.,Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM core_ctl
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE core_ctl_trace

#if !defined(__HYPNUS_TRACE_H__) || defined(TRACE_HEADER_MULTI_READ)
#define __CORE_CTL_TRACE_H__

#include <linux/tracepoint.h>
/*
 * Tracepoint for core ctl:
 */
TRACE_EVENT(core_ctl_eval_need,

	TP_PROTO(unsigned int cpu, unsigned int old_need,
		 unsigned int new_need, unsigned int updated),
	TP_ARGS(cpu, old_need, new_need, updated),
	TP_STRUCT__entry(
		__field(u32, cpu)
		__field(u32, old_need)
		__field(u32, new_need)
		__field(u32, updated)
	),
	TP_fast_assign(
		__entry->cpu = cpu;
		__entry->old_need = old_need;
		__entry->new_need = new_need;
		__entry->updated = updated;
	),
	TP_printk("cpu=%u, old_need=%u, new_need=%u, updated=%u", __entry->cpu,
		  __entry->old_need, __entry->new_need, __entry->updated)
);

TRACE_EVENT(core_ctl_set_busy,

	TP_PROTO(unsigned int cpu, unsigned int busy,
		 unsigned int old_is_busy, unsigned int is_busy),
	TP_ARGS(cpu, busy, old_is_busy, is_busy),
	TP_STRUCT__entry(
		__field(u32, cpu)
		__field(u32, busy)
		__field(u32, old_is_busy)
		__field(u32, is_busy)
	),
	TP_fast_assign(
		__entry->cpu = cpu;
		__entry->busy = busy;
		__entry->old_is_busy = old_is_busy;
		__entry->is_busy = is_busy;
	),
	TP_printk("cpu=%u, busy=%u, old_is_busy=%u, new_is_busy=%u",
		  __entry->cpu, __entry->busy, __entry->old_is_busy,
		  __entry->is_busy)
);

TRACE_EVENT(core_ctl_update_running_avg,

	TP_PROTO(int avg, int iowait_avg,
		 int max_nr, int big_max_nr,
		 int l_big_avg, int big_avg),
	TP_ARGS(avg, iowait_avg, max_nr, big_max_nr, l_big_avg, big_avg),
	TP_STRUCT__entry(
		__field(s32, avg)
		__field(s32, iowait_avg)
		__field(s32, max_nr)
		__field(s32, big_max_nr)
		__field(s32, l_big_avg)
		__field(s32, big_avg)

	),
	TP_fast_assign(
		__entry->avg = avg;
		__entry->iowait_avg = iowait_avg;
		__entry->max_nr = max_nr;
		__entry->big_max_nr = big_max_nr;
		__entry->l_big_avg = l_big_avg;
		__entry->big_avg = big_avg;
	),
	TP_printk("avg=%d, iowait_avg=%d, max_nr=%d, big_max_nr=%d, l_big_avg=%d, big_avg=%d",
		  __entry->avg, __entry->iowait_avg, __entry->max_nr,
		  __entry->big_max_nr, __entry->l_big_avg, __entry->big_avg)
);

TRACE_EVENT(core_ctl_set_boost,

	TP_PROTO(u32 refcount, s32 ret),
	TP_ARGS(refcount, ret),
	TP_STRUCT__entry(
		__field(u32, refcount)
		__field(s32, ret)
	),
	TP_fast_assign(
		__entry->refcount = refcount;
		__entry->ret = ret;
	),
	TP_printk("refcount=%u, ret=%d", __entry->refcount, __entry->ret)
);

#endif /* __CORE_CTL_TRACE_H__ */
/* This part must be outside protection */
#include <trace/define_trace.h>
