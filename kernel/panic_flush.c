// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#define DEBUG
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/blkdev.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/mount.h>
#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/statfs.h>

#define PANIC_FLUSH_POLL_MS (10)

#define NEED_TO_CONFIRM 0

extern int kern_path(const char *name, unsigned int flags, struct path *path);
extern int vfs_statfs(const struct path *path, struct kstatfs *buf);
extern int blkdev_issue_flush(struct block_device *bdev, gfp_t gfp_mask, sector_t *error_sector);
#ifdef CONFIG_TMEMORY
extern void tmemory_close(void);
#endif


static struct super_block *get_f2fs_sb()
{
	struct path kpath;
	struct kstatfs st;

	if (kern_path("/data", LOOKUP_FOLLOW, &kpath))
		return NULL;
	if (vfs_statfs(&kpath, &st))
		return NULL;
	if (kpath.dentry && kpath.dentry->d_sb &&
			kpath.dentry->d_sb->s_magic == F2FS_SUPER_MAGIC) {
		pr_debug("%s: found data sb.", __func__);
		return kpath.dentry->d_sb;
	}

	return NULL;
}

struct panic_flush_control {
	struct task_struct *flush_thread;
	wait_queue_head_t flush_wq;
	atomic_t flush_issuing;
	atomic_t flush_issued;
};

static struct panic_flush_control *pfc;

static void panic_issue_flush()
{
	struct super_block *sb = NULL;
	int ret = -1;

	sb = get_f2fs_sb();
	if (!sb) {
		pr_err("%s: get_f2fs_sb failed\n", __func__);
		return;
	}
#ifdef CONFIG_TMEMORY
	pr_emerg("tmemory_close start\n");
	tmemory_close();
	pr_emerg("tmemory_close end\n");
#endif
	if (!(sb->s_flags & MS_RDONLY) && NULL != sb->s_bdev) {
		ret = blkdev_issue_flush(sb->s_bdev, GFP_KERNEL, NULL);
	}
	if (!ret)
		pr_emerg("blkdev_issue_flush before panic return success\n");
}

static int panic_flush_thread(void *data)
{
repeat:
	if (kthread_should_stop())
		return 0;
	wait_event(pfc->flush_wq, kthread_should_stop() ||
			atomic_read(&pfc->flush_issuing) > 0);
	if (atomic_read(&pfc->flush_issuing) > 0) {
		panic_issue_flush();
		atomic_inc(&pfc->flush_issued);
		atomic_dec(&pfc->flush_issuing);
	}
	goto repeat;
}

#if NEED_TO_CONFIRM
extern bool is_fulldump_enable(void);

static inline bool need_flush_device_cache(void)
{
	if (is_fulldump_enable())
		return false;
	return true;
}
#endif

void panic_flush_device_cache_handler(int timeout)
{
	pr_emerg("%s\n", __func__);
#if NEED_TO_CONFIRM
	if (!need_flush_device_cache())
		pr_emerg("%s: skip flush device cache\n", __func__);
#endif

	if (atomic_inc_return(&pfc->flush_issuing) == 1 &&
		waitqueue_active(&pfc->flush_wq)) {
		pr_emerg("%s: flush device cache\n", __func__);
		atomic_set(&pfc->flush_issued, 0);
		wake_up(&pfc->flush_wq);
		while (timeout > 0 && atomic_read(&pfc->flush_issued) == 0) {
			mdelay(PANIC_FLUSH_POLL_MS);
			timeout -= PANIC_FLUSH_POLL_MS;
		}
		pr_emerg("%s: remaining timeout = %d\n", __func__, timeout);
	}
}

static int __init create_panic_flush_control(void)
{
	int err = 0;
	pr_debug("%s\n", __func__);
	pfc = kzalloc(sizeof(*pfc), GFP_KERNEL);
	if (!pfc) {
		pr_err("%s: fail to allocate memory\n", __func__);
		return -ENOMEM;
	}

	init_waitqueue_head(&pfc->flush_wq);
	atomic_set(&pfc->flush_issuing, 0);
	atomic_set(&pfc->flush_issued, 0);
	pfc->flush_thread = kthread_run(panic_flush_thread, pfc, "panic_flush");
	if (IS_ERR(pfc->flush_thread)) {
		err = PTR_ERR(pfc->flush_thread);
		kfree(pfc);
		pfc = NULL;
	}
	return err;
}

static void __exit destroy_panic_flush_control(void)
{
	pr_debug("%s\n", __func__);
	if (pfc && pfc->flush_thread) {
		pr_debug("%s: stop panic_flush thread\n", __func__);
		kthread_stop(pfc->flush_thread);
		kfree(pfc);
		pfc = NULL;
	}
}
module_init(create_panic_flush_control);
module_exit(destroy_panic_flush_control);
MODULE_DESCRIPTION("OPLUS panic flush control");
MODULE_LICENSE("GPL v2");

