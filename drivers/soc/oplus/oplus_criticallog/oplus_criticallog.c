#include <linux/module.h>
#include <linux/slab.h>
#include <soc/oplus/mmkey_log.h>
#include <criticallog_class.h>
#include <linux/kthread.h>
#include <linux/wait.h>

//#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
static int keylog_registered = 0;
static struct criticallog_dev keylog_swthdev;
static wait_queue_head_t keylog_thread_wq;
static struct task_struct *keylog_thread_task = NULL;

#define KEYLOG_BUF_MAX 5
#define LOGNAME_BUF_LEN 2148
#define SUBSYSTME_NAME_LEN 10

struct keylog_data{
	char buf[LOGNAME_BUF_LEN];
	int id;
	char subsystem[SUBSYSTME_NAME_LEN];
};

struct keylog_buf{
    spinlock_t          keylog_lock;
    unsigned int        index_write;
    unsigned int        index_read;
    struct keylog_data  data[KEYLOG_BUF_MAX];
};

static struct keylog_buf gKeylog_buf;

void mm_keylog_write_modemdump(unsigned int hashId, const char *cause, int id, char *subsys) {
   if(keylog_thread_task && keylog_registered){
        struct keylog_data * keylog_data;
        spin_lock(&gKeylog_buf.keylog_lock);
        keylog_data = &gKeylog_buf.data[gKeylog_buf.index_write%KEYLOG_BUF_MAX];
        snprintf(keylog_data->buf, LOGNAME_BUF_LEN,"fid:%u;cause:%s\n", hashId, cause);
        keylog_data->id = id;
		memset(keylog_data->subsystem, 0, sizeof(char)*SUBSYSTME_NAME_LEN);
		strcpy(keylog_data->subsystem, subsys);
        gKeylog_buf.index_write++;
        spin_unlock(&gKeylog_buf.keylog_lock);
        wake_up_interruptible(&keylog_thread_wq);
   }
}
EXPORT_SYMBOL_GPL(mm_keylog_write_modemdump);

static ssize_t keylog_snprintf_logInfo(struct criticallog_dev *sdev, char *buf) {
    struct keylog_data *keylog_data = &gKeylog_buf.data[gKeylog_buf.index_read%KEYLOG_BUF_MAX];
    pr_err("name = %s\n", keylog_data->buf);
    return snprintf(buf, LOGNAME_BUF_LEN,"%s", keylog_data->buf);
}

static int keylog_thread(void *data){
    while(1){
        wait_event_interruptible(keylog_thread_wq, (gKeylog_buf.index_write - gKeylog_buf.index_read));
        while(gKeylog_buf.index_write > gKeylog_buf.index_read){
            struct keylog_data *keylog_data = &gKeylog_buf.data[gKeylog_buf.index_read%KEYLOG_BUF_MAX];
            pr_info("keylog_data->buf lenth = %d\n", (int)strlen(keylog_data->buf));
            criticallog_set_state(&keylog_swthdev, keylog_data->id, keylog_data->subsystem);
            gKeylog_buf.index_read++;
        }
    }
    keylog_thread_task = NULL;
    return 0;
}


static int __init oplus_criticallog_init(void)
{
    int ret = 0;
    init_waitqueue_head(&keylog_thread_wq);
    keylog_swthdev.name = "oplus_critical_log";
    keylog_swthdev.print_logInfo = keylog_snprintf_logInfo;

    ret = criticallog_dev_register(&keylog_swthdev);

    if(ret){
        goto keylog_err;
    }
    spin_lock_init(&gKeylog_buf.keylog_lock);
    gKeylog_buf.index_write = 0;
    gKeylog_buf.index_read = 0;
    keylog_thread_task = kthread_create(keylog_thread, (void *)&gKeylog_buf,"keylog");
    if(!keylog_thread_task){
        ret = -1;
        goto keylog_err;
    }
    keylog_registered = 1;
    wake_up_process(keylog_thread_task);
    return 0;
keylog_err:
    criticallog_dev_unregister(&keylog_swthdev);
    keylog_registered = 0;
    return ret;
}
arch_initcall(oplus_criticallog_init);

MODULE_DESCRIPTION("OPLUS critical log");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("John.Xu");
//#endif /*OPLUS_FEATURE_MODEM_MINIDUMP*/
