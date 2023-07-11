#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/syscalls.h>
#include <linux/preempt.h>
#include <linux/workqueue.h>
#include <linux/his_debug_base.h>

#define FS_SYNC_TIMEOUT_MS 2000

static struct work_struct fs_sync_work;
static DECLARE_COMPLETION(sync_compl);

static void fs_sync_work_func(struct work_struct *work)
{
	pr_emerg("fs_sync: call sys_sync for data\n");
	FILE_SYNC;
	complete(&sync_compl);
}

void run_fs_sync_work(void)
{
	if (!in_atomic()) {
		pr_emerg("fs_sync: try sys sync when system down\n");
		INIT_WORK(&fs_sync_work, fs_sync_work_func);
		reinit_completion(&sync_compl);

		schedule_work(&fs_sync_work);
		if (wait_for_completion_timeout(&sync_compl, msecs_to_jiffies(FS_SYNC_TIMEOUT_MS)) == 0)
			pr_emerg("fs_sync: wait sync complete timeout\n");
	}
}
EXPORT_SYMBOL(run_fs_sync_work);
