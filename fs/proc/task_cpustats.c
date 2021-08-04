#include <linux/kernel_stat.h>
#include <linux/cpufreq.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched/topology.h>
#include <linux/sched/energy.h>

#include "../../drivers/misc/mediatek/base/power/include/mtk_upower.h"

/* FIXME get max_pid on the runtime.*/
#define MAX_PID (32768)
#define CTP_WINDOW_SZ (5)
extern unsigned int sysctl_task_cpustats_enable;

struct acct_cpustat {
	pid_t tgid;
	unsigned int pwr;
	char comm[TASK_COMM_LEN];
};

static struct acct_cpustat cpustats[MAX_PID];

#ifdef CONFIG_MTK_UNIFY_POWER
int get_power(int cpu, unsigned long cap) {
	int i;
	unsigned long pwr,dyn_pwr,stc_pwr;
	const struct sched_group_energy *sge = cpu_core_energy(cpu);
	if (!sge)
		goto err_found;
	for (i = sge->nr_cap_states - 1; i > -1; i--) {

		struct upower_tbl_row* cap_states = sge->cap_states + i;
		if (cap_states->cap == cap){
			dyn_pwr = cap_states->dyn_pwr;
			stc_pwr = cap_states->lkg_pwr[sge->lkg_idx];
			pwr = dyn_pwr + stc_pwr;

			return pwr;
		}

	}
err_found:
	pr_err("not found %d %d in sge.\n", cpu);
	return 0;
}
#else
int get_power(int cpu, int freq) {
	int i;
	struct sched_group_energy *sge = sge_array[cpu][SD_LEVEL0];
	if (!sge)
		goto err_found;
	for (i = sge->nr_cap_states - 1; i > -1; i--) {
		struct capacity_state* cs = sge->cap_states + i;
		if (cs->frequency == freq)
			return cs->power;
	}
err_found:
	pr_err("not found %d %d in sge.\n", cpu, freq);
	return 0;
}

#endif
EXPORT_SYMBOL(get_power);

static int task_cpustats_show(struct seq_file *m, void *v)
{
	int *idx = (int *) v;
	struct acct_cpustat *s = cpustats + *idx;
	seq_printf(m, "%d\t%d\t%d\t%s\n", *idx, s->tgid,
			s->pwr, s->comm);
	return 0;
}

static void *task_cpustats_next(struct seq_file *m, void *v, loff_t *ppos)
{
	int *idx = (int *)v;
	(*idx)++;
	(*ppos)++;
	for (; *idx < MAX_PID; (*idx)++, (*ppos)++) {
		struct acct_cpustat *s = cpustats + *idx;
		if (s->pwr)
			return idx;
	}
	return NULL;
}

static void *task_cpustats_start(struct seq_file *m, loff_t *ppos)
{
	int i, j;
	unsigned long begin = jiffies - CTP_WINDOW_SZ * HZ, end = jiffies;
	int *idx = m->private;
	if (!sysctl_task_cpustats_enable)
		return NULL;
	*idx = *ppos;
	if (*idx >= MAX_PID)
		goto start_error;
	memset(cpustats, 0, sizeof(cpustats));
	for_each_possible_cpu(i) {
		struct kernel_task_cpustat* kstat = &per_cpu(ktask_cpustat, i);
		for (j = 0; j < MAX_CTP_WINDOW; j++) {
			struct task_cpustat *ts = kstat->cpustat + j;
			unsigned long r_time = ts->end - ts->begin;
			if (ts->pid >= MAX_PID)
				continue;
			if (ts->begin >= begin && ts->end <= end) {
				struct acct_cpustat *as = cpustats + ts->pid;
				if (as->pwr == 0)
					memcpy(as->comm, ts->comm, TASK_COMM_LEN);
				/* 4 ms each tick */
#ifdef CONFIG_MTK_UNIFY_POWER
				as->pwr += get_power(i, ts->cap) * jiffies_to_msecs(r_time);
#else
				as->pwr += get_power(i, ts->freq) * jiffies_to_msecs(r_time);
#endif
				as->tgid = ts->tgid;
			}
		}
	}
	for (; *idx < MAX_PID; (*idx)++, (*ppos)++) {
		struct acct_cpustat *as = cpustats + *idx;
		if (as->pwr)
			return idx;
	}
start_error:
	return NULL;
}

static void task_cpustats_stop(struct seq_file *m, void *v)
{
}

static const struct seq_operations seq_ops = {
	.start	= task_cpustats_start,
	.next	= task_cpustats_next,
	.stop	= task_cpustats_stop,
	.show	= task_cpustats_show
};

static int sge_show(struct seq_file *m, void *v)
{
#ifdef CONFIG_MTK_UNIFY_POWER
	unsigned long dyn_pwr;
	unsigned long stc_pwr;
	unsigned long pwr;
#else
	unsigned long pwr;
	unsigned long freq;
#endif
	const struct sched_group_energy *core_eng;
	int cpu;
	int core_nr_cap = 0;
	unsigned long cap_idx;
	int i;

	for_each_possible_cpu(cpu) {
		core_eng = cpu_core_energy(cpu);
		core_nr_cap = core_eng->nr_cap_states;
		seq_printf(m, "cpu %d\n", cpu);

		for (i = 0; i < core_nr_cap; i++) {
#ifdef CONFIG_MTK_UNIFY_POWER
			dyn_pwr =
			core_eng->cap_states[i].dyn_pwr;
			stc_pwr =
			core_eng->cap_states[i].lkg_pwr[core_eng->lkg_idx];
			pwr = dyn_pwr + stc_pwr;
			cap_idx =
			core_eng->cap_states[i].cap;

			seq_printf(m, "cap %lu pwr %lu \n", cap_idx, pwr);
#else
			pwr = core_eng->cap_states[i].power;

			seq_printf(m, "freq: %lu pwr: %lu\n", freq, pwr);
#endif
		}
	}
	return 0;
}

static int sge_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sge_show, NULL);
}

static int task_cpustats_open(struct inode *inode, struct file *file)
{
	int *offs = __seq_open_private(file, &seq_ops, sizeof(int));

	if (!offs)
		return -ENOMEM;
	return 0;
}

static int task_cpustats_release(struct inode *inode, struct file *file)
{
	return seq_release_private(inode, file);
}

static const struct file_operations sge_proc_fops = {
	.open		= sge_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations task_cpustats_proc_fops = {
	.open		= task_cpustats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= task_cpustats_release,
};

static int __init proc_task_cpustat_init(void)
{
	proc_create("sgeinfo", 0, NULL, &sge_proc_fops);
	proc_create("task_cpustats", 0, NULL, &task_cpustats_proc_fops);
	return 0;
}
fs_initcall(proc_task_cpustat_init);

