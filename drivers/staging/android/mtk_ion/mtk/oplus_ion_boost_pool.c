#define pr_fmt(fmt) "boostpool: " fmt

#include <asm/page.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/sizes.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/vmstat.h>
#include <linux/oom.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>
#include <../../../../kernel/sched/sched.h>

#include "oplus_ion_boost_pool.h"

#ifdef OPLUS_FEATURE_HEALTHINFO
/* #include <linux/healthinfo/ion.h> wait for healthinfo*/
#endif /* OPLUS_FEATURE_HEALTHINFO */

#define MAX_BOOST_POOL_HIGH (1024 * 256)

static bool boost_pool_enable = true;
static void kcrit_scene_wakeup_lmkd(void);

#define K(x) ((x) << (PAGE_SHIFT-10))
#define M(x) (K(x) >> 10)

/* copy from ion_mm_heap.c */
static const unsigned int orders[] = { 4, 1, 0 };
static unsigned int order_gfp_flags[] = {
	(GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN | __GFP_NORETRY) &
	    ~__GFP_RECLAIM,
	(GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN | __GFP_NORETRY) &
	    ~__GFP_RECLAIM,
	(GFP_HIGHUSER | __GFP_ZERO)
};

#define NUM_ORDERS ARRAY_SIZE(orders)

bool autofill_enable = false;
static bool autofill_enable_inited = false;

static int order_to_index(unsigned int order)
{
	int i;

	for (i = 0; i < NUM_ORDERS; i++)
		if (order == orders[i])
			return i;
	BUG();
	return -1;
}

static inline unsigned int order_to_size(int order)
{
	return PAGE_SIZE << order;
}

static int page_pool_nr_pages(struct ion_page_pool *pool)
{
	if (NULL == pool) {
		pr_err("%s: pool is NULL!\n", __func__);
		return 0;
	}
	return (pool->low_count + pool->high_count) << pool->order;
}

static int boost_pool_nr_pages(struct ion_boost_pool *pool)
{
	int i;
	int count = 0;

	if (NULL == pool) {
		pr_err("%s: pool is NULL!\n", __func__);
		return 0;
	}

	for (i = 0; i < NUM_ORDERS; i++)
		count += page_pool_nr_pages(pool->pools[i]);

	return count;
}

static int fill_boost_page_pool(struct ion_page_pool *pool)
{
	struct page *page;

	if (NULL == pool) {
		pr_err("%s: pool is NULL!\n", __func__);
		return -ENOENT;
	}

	page = ion_page_pool_alloc_pages(pool);
	if (NULL == page)
		return -ENOMEM;

	ion_page_pool_free(pool, page);

	return 0;
}

void boost_pool_dump(struct ion_boost_pool *pool)
{
	int i;

	pr_info("Name:%s: %dMib, low: %dMib, high:%dMib\n",
		pool->name,
		boost_pool_nr_pages(pool) >> 8,
		pool->low >> 8,
		pool->high >> 8);
	for (i = 0; i < NUM_ORDERS; i++) {
		struct ion_page_pool *page_pool = pool->pools[i];

		pr_info("%d order %u highmem pages in boost pool = %lu total\n",
			page_pool->high_count, page_pool->order,
			(PAGE_SIZE << page_pool->order) * page_pool->high_count);
		pr_info("%d order %u lowmem pages in boost pool = %lu total\n",
			page_pool->low_count, page_pool->order,
			(PAGE_SIZE << page_pool->order) * page_pool->low_count);
	}
}

static int boost_pool_kworkthread(void *p)
{
	int i;
	struct ion_boost_pool *pool;
	int ret;

	if (NULL == p) {
		pr_err("%s: p is NULL!\n", __func__);
		return 0;
	}

	pool = (struct ion_boost_pool *)p;
	while (true) {
		ret = wait_event_interruptible(pool->waitq,
					       (pool->wait_flag == 1));
		if (ret < 0)
			continue;

		pool->wait_flag = 0;

		for (i = 0; i < NUM_ORDERS; i++) {
			while (!pool->force_stop && boost_pool_nr_pages(pool) < pool->high) {
				if (fill_boost_page_pool(pool->pools[i]) < 0)
					break;
			}
		}
	}

	return 0;
}

#define BOOSTPOOL_DEBUG
/* extern int direct_vm_swappiness; */
static int boost_prefill_kworkthread(void *p)
{
	int i;
	struct ion_boost_pool *pool;
	u64 timeout_jiffies;
	int ret;
#ifdef BOOSTPOOL_DEBUG
	unsigned long begin;
#endif /* BOOSTPOOL_DEBUG */

	if (NULL == p) {
		pr_err("%s: p is NULL!\n", __func__);
		return 0;
	}

	pool = (struct ion_boost_pool *)p;
	while (true) {
		ret = wait_event_interruptible(pool->prefill_waitq,
					       (pool->prefill_wait_flag == 1));
		if (ret < 0)
			continue;

		pool->prefill_wait_flag = 0;

		mutex_lock(&pool->prefill_mutex);
		timeout_jiffies = get_jiffies_64() + 2 * HZ;
		/* direct_vm_swappiness = 20; */
#ifdef BOOSTPOOL_DEBUG
		begin = jiffies;

		pr_info("prefill start >>>>> nr_page: %dMib high: %dMib.\n",
			M(boost_pool_nr_pages(pool)), M(pool->high));
#endif /* BOOSTPOOL_DEBUG */

		for (i = 0; i < NUM_ORDERS; i++) {
			while (!pool->force_stop &&
			       boost_pool_nr_pages(pool) < pool->high) {

				/* support timeout to limit alloc pages. */
				if (time_after64(get_jiffies_64(), timeout_jiffies)) {
					pr_warn("prefill timeout.\n");
					break;
				}

				if (fill_boost_page_pool(pool->pools[i]) < 0)
					break;
			}
		}

#ifdef BOOSTPOOL_DEBUG
		pr_info("prefill end <<<<< nr_page: %dMib high:%dMib use %dms\n",
			M(boost_pool_nr_pages(pool)), M(pool->high),
			jiffies_to_msecs(jiffies - begin));
#endif /* BOOSTPOOL_DEBUG */

		pool->high = max(boost_pool_nr_pages(pool), pool->low);
		pool->prefill = false;
		/* direct_vm_swappiness = 60; */

		mutex_unlock(&pool->prefill_mutex);
	}

	return 0;
}

struct page *boost_pool_allocate(struct ion_boost_pool *pool,
				 unsigned long size, unsigned int max_order)
{
	int i;
	struct page *page;

	if (NULL == pool) {
		pr_err("%s: pool is NULL!\n", __func__);
		return NULL;
	}

	for (i = 0; i < NUM_ORDERS; i++) {
		if (size < order_to_size(orders[i]))
			continue;
		if (max_order < orders[i])
			continue;

		page = ion_page_pool_alloc(pool->pools[i]);

		if (page) {
			BUG_ON(compound_order(page) != orders[i]);
		} else {
			continue;
		}
		return page;
	}

	return NULL;
}

void boost_pool_wakeup_process(struct ion_boost_pool *pool)
{
	if (!boost_pool_enable)
		return;

	if (NULL == pool) {
		pr_err("%s: boost_pool is NULL!\n", __func__);
		return;
	}

	if (!pool->force_stop) {
		pool->wait_flag = 1;
		wake_up_interruptible(&pool->waitq);
	}
}

static void boost_pool_all_free(struct ion_boost_pool *pool, gfp_t gfp_mask,
				int nr_to_scan)
{
	int i;

	if (NULL == pool) {
		pr_err("%s: boost_pool is NULL!\n", __func__);
		return;
	}

	for (i = 0; i < NUM_ORDERS; i++)
		ion_page_pool_shrink(pool->pools[i], gfp_mask, nr_to_scan);
}

int boost_pool_free(struct ion_boost_pool *pool, struct page *page,
		    int order)
{
	if (!boost_pool_enable) {
		boost_pool_all_free(pool, __GFP_HIGHMEM, MAX_POOL_SIZE);
		return -1;
	}

	if ((NULL == pool) || (NULL == page)) {
		pr_err("%s: pool/page is NULL!\n", __func__);
		return -1;
	}

	if (order == 0)
		return -1;

	if (!((order == orders[0]) || (order == orders[1])
	      || (order == orders[2]))) {
		pr_err("%s: order:%d is error!\n", __func__, order);
		return -1;
	}

	if (boost_pool_nr_pages(pool) <  pool->low + (SZ_64M >> PAGE_SHIFT)) {
		ion_page_pool_free(pool->pools[order_to_index(order)], page);
		return 0;
	}

	return -1;
}

int boost_pool_shrink(struct ion_boost_pool *boost_pool,
		      struct ion_page_pool *pool, gfp_t gfp_mask,
		      int nr_to_scan)
{
	int nr_max_free;
	int nr_to_free;
	int nr_total = 0;

	if ((NULL == boost_pool) || (NULL == pool)) {
		pr_err("%s: boostpool/pool is NULL!\n", __func__);
		return 0;
	}

	/* don't shrink self.. */
	if (boost_pool->tsk->pid == current->pid ||
	    boost_pool->prefill_tsk->pid == current->pid)
		return 0;

	if (nr_to_scan == 0)
		return ion_page_pool_shrink(pool, gfp_mask, 0);

	nr_max_free = boost_pool_nr_pages(boost_pool) -
		(boost_pool->high + LOWORDER_WATER_MASK);
	nr_to_free = min(nr_max_free, nr_to_scan);

	if (nr_to_free <= 0)
		return 0;

	nr_total = ion_page_pool_shrink(pool, gfp_mask, nr_to_free);
	return nr_total;
}

void boost_pool_dec_high(struct ion_boost_pool *pool, int nr_pages)
{
	if (pool->prefill)
		return;

	if (unlikely(nr_pages < 0))
		return;

	pool->high = max(pool->low,
			 pool->high - nr_pages);
}

static int boost_pool_proc_show(struct seq_file *s, void *v)
{
	struct ion_boost_pool *boost_pool = s->private;
	int i;

	seq_printf(s, "Name:%s: %dMib, prefill: %d origin: %dMib low: %dMib high: %dMib\n",
		   boost_pool->name,
		   M(boost_pool_nr_pages(boost_pool)),
		   boost_pool->prefill,
		   M(boost_pool->origin),
		   M(boost_pool->low),
		   M(boost_pool->high));

	for (i = 0; i < NUM_ORDERS; i++) {
		struct ion_page_pool *pool = boost_pool->pools[i];

		seq_printf(s, "%d order %u highmem pages in boost pool = %lu total\n",
			   pool->high_count, pool->order,
			   (PAGE_SIZE << pool->order) * pool->high_count);
		seq_printf(s, "%d order %u lowmem pages in boost pool = %lu total\n",
			   pool->low_count, pool->order,
			   (PAGE_SIZE << pool->order) * pool->low_count);
	}
	return 0;
}

static int boost_pool_proc_open(struct inode *inode, struct file *file)
{
	struct ion_boost_pool *data = PDE_DATA(inode);
	return single_open(file, boost_pool_proc_show, data);
}

static inline void set_cpumask(int end_cpu, struct cpumask *mask)
{
	int i;

        cpumask_clear(mask);
        for (i = 0; i <= end_cpu; i++)
                cpumask_set_cpu(i, mask);
}

static ssize_t boost_pool_proc_write(struct file *file,
				     const char __user *buf,
				     size_t count, loff_t *ppos)
{
	char buffer[13];
	int err, num;
	struct ion_boost_pool *boost_pool = PDE_DATA(file_inode(file));
	int end_cpu, nr_pages;

	if (IS_ERR_OR_NULL(boost_pool)) {
		pr_err("%s: boost pool is NULL.\n");
		return -EFAULT;
	}

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	err = kstrtoint(strstrip(buffer), 0, &num);
	if(err)
		return err;

	if (num == 0) {
		pr_info("reset flag.\n");
		boost_pool->high = boost_pool->low = boost_pool->origin;
		boost_pool->force_stop = false;
		return count;
	}

	if (num == -1) {
		pr_info("force stop.\n");
		boost_pool->force_stop = true;
		return count;
	}

	/* ugly code. */
	nr_pages = num & (0xfffff);
	end_cpu = (num >> 20);
	pr_info("dbg %d %d.\n", nr_pages, end_cpu);

	if (nr_pages < 0 || nr_pages >= MAX_BOOST_POOL_HIGH ||
	    nr_pages <= boost_pool->low) {
		pr_err("nr_page param invalid %d.\n", nr_pages);
		return -EINVAL;
	}

	if (end_cpu >= NR_CPUS || end_cpu < 0) {
		pr_err("end_cpu param invalid %d.\n", end_cpu);
		return -EINVAL;
	}

	if (mutex_trylock(&boost_pool->prefill_mutex)) {
		long mem_avail = si_mem_available();
		struct cpumask mask;

		pr_info("%s: set high wm => %dMib. current avail => %dMib end_cpu => %d\n",
			current->comm, M(nr_pages), M(mem_avail), end_cpu);

		boost_pool->prefill = true;
		boost_pool->force_stop = false;
		boost_pool->high = nr_pages;

		/* kill one heavy process on low end target. */
		if (mem_avail < (SZ_2G >> PAGE_SHIFT))
			kcrit_scene_wakeup_lmkd();

		if (end_cpu > 0) {
			set_cpumask(end_cpu, &mask);
			set_cpus_allowed_ptr(boost_pool->prefill_tsk, &mask);
		}

		boost_pool->prefill_wait_flag = 1;
		wake_up_interruptible(&boost_pool->prefill_waitq);

		mutex_unlock(&boost_pool->prefill_mutex);
	} else {
		pr_err("%s: prefill already running. \n");
		return -EBUSY;
	}

	return count;
}

static const struct file_operations boost_pool_proc_ops = {
	.owner          = THIS_MODULE,
	.open           = boost_pool_proc_open,
	.read           = seq_read,
	.write          = boost_pool_proc_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int boost_pool_low_proc_show(struct seq_file *s, void *v)
{
	struct ion_boost_pool *boost_pool = s->private;

	seq_printf(s, "low %dMib.\n", M(boost_pool->low));

	return 0;
}

static int boost_pool_low_proc_open(struct inode *inode, struct file *file)
{
	struct ion_boost_pool *data = PDE_DATA(inode);
	return single_open(file, boost_pool_low_proc_show, data);
}

static ssize_t boost_pool_low_proc_write(struct file *file,
				     const char __user *buf,
				     size_t count, loff_t *ppos)
{
	char buffer[13];
	int err, nr_pages;
	struct ion_boost_pool *boost_pool = PDE_DATA(file_inode(file));

	if (IS_ERR_OR_NULL(boost_pool)) {
		pr_err("%s: boost pool is NULL.\n");
		return -EFAULT;
	}

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	err = kstrtoint(strstrip(buffer), 0, &nr_pages);
	if(err)
		return err;

	if (nr_pages <= 0 || nr_pages >= MAX_BOOST_POOL_HIGH)
		return -EINVAL;

	boost_pool->low = boost_pool->high = nr_pages;

	return count;
}

static const struct file_operations boost_pool_low_proc_ops = {
	.owner          = THIS_MODULE,
	.open           = boost_pool_low_proc_open,
	.read           = seq_read,
	.write          = boost_pool_low_proc_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static void ion_heap_destroy_pools(struct ion_page_pool **pools)
{
	int i;

	for (i = 0; i < NUM_ORDERS; i++)
		if (pools[i])
			ion_page_pool_destroy(pools[i]);
}

static int ion_heap_create_pools(struct ion_page_pool **pools,
				 bool cached, bool boost_flag)
{
	int i;
	gfp_t gfp_flags;

	for (i = 0; i < NUM_ORDERS; i++) {
		struct ion_page_pool *pool;

		gfp_flags = order_gfp_flags[i];

		pool = ion_page_pool_create(gfp_flags, orders[i], cached,
					    boost_flag);
		if (!pool)
			goto err_create_pool;
		pools[i] = pool;
	}
	return 0;

err_create_pool:
	ion_heap_destroy_pools(pools);
	return -ENOMEM;
}

static int boost_pool_stat_proc_show(struct seq_file *s, void *v)
{
	/*struct ion_boost_pool *boost_pool = s->private;

	seq_printf(s, "%d,%d,%d,%d\n",
		   M(boost_pool_nr_pages(boost_pool)),
		   boost_pool->prefill, M(si_mem_available()),
		   M(global_zone_page_state(NR_IONCACHE_PAGES)));*/

	return 0;
}

static int boost_pool_stat_proc_open(struct inode *inode, struct file *file)
{
	struct ion_boost_pool *data = PDE_DATA(inode);
	return single_open(file, boost_pool_stat_proc_show, data);
}

static const struct file_operations boost_pool_stat_proc_ops = {
	.owner          = THIS_MODULE,
	.open           = boost_pool_stat_proc_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

struct ion_boost_pool *boost_pool_create(unsigned int ion_flag,
					 unsigned int nr_pages,
					 struct proc_dir_entry *root_dir,
					 char *name)
{
	bool graphic_buffer_flag = true;
	struct task_struct *tsk;
	struct ion_boost_pool *boost_pool;
	char buf[128];
	struct cpumask mask;
	int end_cpu = 3;

	if (NULL == root_dir) {
		pr_err("boost_pool dir not exits.\n");
		return NULL;
	}
	pr_info("%d boost_pool.\n", NUM_ORDERS);

	boost_pool = kzalloc(sizeof(struct ion_boost_pool) +
			     sizeof(struct ion_page_pool *) * NUM_ORDERS,
			     GFP_KERNEL);

	if (NULL == boost_pool) {
		pr_err("%s: boost_pool is NULL!\n", __func__);
		return NULL;
	}

	if (ion_heap_create_pools(boost_pool->pools,
				  ion_flag == ION_FLAG_CACHED,
				  graphic_buffer_flag))
		goto free_heap;

	boost_pool->origin = boost_pool->high = boost_pool->low = nr_pages;
	boost_pool->name = name;
	boost_pool->flag = ion_flag;

	boost_pool->proc_info = proc_create_data(name, 0666,
						 root_dir,
						 &boost_pool_proc_ops,
						 boost_pool);
	if (IS_ERR_OR_NULL(boost_pool->proc_info)) {
		pr_info("Unable to initialise /proc/%s\n",
			name);
		goto destroy_pools;
	} else {
		pr_info("procfs entry /proc/%s allocated. data=%p\n",
			name, boost_pool);
	}

	snprintf(buf, 128, "%s_low", name);
	boost_pool->proc_low_info = proc_create_data(buf, 0666,
						 root_dir,
						 &boost_pool_low_proc_ops,
						 boost_pool);
	if (IS_ERR_OR_NULL(boost_pool->proc_low_info)) {
		pr_info("Unable to initialise /proc/boost_pool/%s_low\n",
			name);
		goto destroy_proc_info;
	} else {
		pr_info("procfs entry /proc/boost_pool/%s_low allocated.\n",
			name);
	}

	snprintf(buf, 128, "%s_stat", name);
	boost_pool->proc_stat = proc_create_data(buf, 0444,
						 root_dir,
						 &boost_pool_stat_proc_ops,
						 boost_pool);
	if (IS_ERR_OR_NULL(boost_pool->proc_stat)) {
		pr_info("Unable to initialise /proc/boost_pool/%s_stat\n",
			name);
		goto destroy_proc_low_info;
	} else {
		pr_info("procfs entry /proc/boost_pool/%s_stat allocated.\n",
			name);
	}

	init_waitqueue_head(&boost_pool->waitq);
	tsk = kthread_run(boost_pool_kworkthread, boost_pool,
			  "%s", name);
	if (IS_ERR_OR_NULL(tsk)) {
		pr_err("%s: kthread_create failed!\n", __func__);
		goto destroy_proc_stat;
	}
	boost_pool->tsk = tsk;
	set_cpumask(end_cpu, &mask);
	set_cpus_allowed_ptr(tsk, &mask);

	mutex_init(&boost_pool->prefill_mutex);
	init_waitqueue_head(&boost_pool->prefill_waitq);

	tsk = kthread_run(boost_prefill_kworkthread, boost_pool,
			  "bp_prefill_%s", name);
	if (IS_ERR_OR_NULL(tsk)) {
		pr_err("%s: kthread_create failed!\n", __func__);
		goto destroy_proc_stat;
	}
	boost_pool->prefill_tsk = tsk;
	set_cpus_allowed_ptr(tsk, &mask);

	boost_pool_wakeup_process(boost_pool);

	return boost_pool;

destroy_proc_stat:
	proc_remove(boost_pool->proc_stat);
destroy_proc_low_info:
	proc_remove(boost_pool->proc_low_info);
destroy_proc_info:
	proc_remove(boost_pool->proc_info);
destroy_pools:
	ion_heap_destroy_pools(boost_pool->pools);
free_heap:
	kfree(boost_pool);
	boost_pool = NULL;

	return NULL;
}

static wait_queue_head_t kcrit_scene_wait;
static int kcrit_scene_flag = 0;
static unsigned int kcrit_scene_proc_poll(struct file *file, poll_table *table)
{
	int mask = 0;

	poll_wait(file, &kcrit_scene_wait, table);

	if (kcrit_scene_flag == 1) {
		mask |= POLLIN | POLLRDNORM;
		kcrit_scene_flag = 0;
	}
	return mask;
}

static int kcrit_scene_proc_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int kcrit_scene_proc_release(struct inode *inode, struct file *file)
{
	return 0;
}

/* TODO add read ops. */
static const struct file_operations kcrit_scene_proc_fops = {
	.owner = THIS_MODULE,
	.open = kcrit_scene_proc_open,
	.release = kcrit_scene_proc_release,
	.poll = kcrit_scene_proc_poll,
};

bool kcrit_scene_init(void)
{
	init_waitqueue_head(&kcrit_scene_wait);
	return proc_create("kcritical_scene", S_IRUGO, NULL,
			   &kcrit_scene_proc_fops) != NULL;
}

static void kcrit_scene_wakeup_lmkd(void)
{
	kcrit_scene_flag = 1;
	wake_up_interruptible(&kcrit_scene_wait);
}

static int autofill_enable_proc_show(struct seq_file *s, void *v)
{
	seq_printf(s, "boostpool autofill %s\n", autofill_enable ? "enabled" : "disabled");
	return 0;
}

static int autofill_enable_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, autofill_enable_proc_show, NULL);
}

static ssize_t autofill_enable_proc_write(struct file *file,
				     const char __user *buf,
				     size_t count, loff_t *ppos)
{
	char buffer[13];
	int err; 

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	err = kstrtobool(strstrip(buffer), &autofill_enable);
	pr_info("write %d to autofill_enable\n", autofill_enable);
	if (err)
		return err;

	return count;
}

static const struct file_operations autofill_enable_proc_ops = {
	.owner          = THIS_MODULE,
	.open           = autofill_enable_proc_open,
	.read           = seq_read,
	.write          = autofill_enable_proc_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};

bool autofill_enable_init(struct proc_dir_entry *root_dir)
{
	if (autofill_enable_inited)
		return true;
	else {
		if (totalram_pages < GIGA_TO_PAGE(3))
			autofill_enable = true;
		autofill_enable_inited = true;
		return proc_create("autofill_enable", 0666, root_dir, &autofill_enable_proc_ops) != NULL;
	}
}

module_param_named(debug_boost_pool_enable, boost_pool_enable, bool, 0644);
