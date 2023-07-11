/*
 * Copyright (C) 2018 Hisense, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/pm_wakeup.h>
#include <linux/syscalls.h>
#include <linux/suspend.h>
#include <linux/his_debug_base.h>

#define SUSPEND_SYS_SYNC_TIMEOUT (HZ/4)

static void suspend_sys_sync(struct work_struct *work);

static int suspend_sys_sync_count;
static bool suspend_sys_sync_abort;
static struct workqueue_struct *suspend_sys_sync_workqueue;

static DEFINE_SPINLOCK(suspend_sys_sync_lock);
static DECLARE_COMPLETION(suspend_sys_sync_comp);

static void suspend_sys_sync_handler(struct timer_list *unused);
static DEFINE_TIMER(suspend_sys_sync_timer, suspend_sys_sync_handler);

static void suspend_sys_sync(struct work_struct *work)
{
	ktime_t start;
	long elapsed_msecs;

	pr_info("PM: Syncing filesystems ... ");
	start = ktime_get();
	ksys_sync();
	pr_cont("done.\n");

	spin_lock(&suspend_sys_sync_lock);
	suspend_sys_sync_count--;
	spin_unlock(&suspend_sys_sync_lock);

	elapsed_msecs = ktime_to_ms(ktime_sub(ktime_get(), start));
	pr_info("Filesystems sync: %ld.%03ld seconds\n",
			elapsed_msecs / MSEC_PER_SEC, elapsed_msecs % MSEC_PER_SEC);
}
static DECLARE_WORK(suspend_sys_sync_work, suspend_sys_sync);

void his_suspend_sys_sync_queue(void)
{
	int ret = 0;

	spin_lock(&suspend_sys_sync_lock);
	ret = queue_work(suspend_sys_sync_workqueue, &suspend_sys_sync_work);
	if (ret)
		suspend_sys_sync_count++;
	spin_unlock(&suspend_sys_sync_lock);
}
EXPORT_SYMBOL(his_suspend_sys_sync_queue);

static void suspend_sys_sync_handler(struct timer_list *unused)
{
	if (suspend_sys_sync_count == 0) {
		complete(&suspend_sys_sync_comp);
	} else if (pm_wakeup_pending()) {
		suspend_sys_sync_abort = true;
		complete(&suspend_sys_sync_comp);
	} else {
		mod_timer(&suspend_sys_sync_timer, jiffies +
				SUSPEND_SYS_SYNC_TIMEOUT);
	}
}

int his_suspend_sys_sync_wait(void)
{
	suspend_sys_sync_abort = false;

	if (suspend_sys_sync_count != 0) {
		mod_timer(&suspend_sys_sync_timer, jiffies +
				SUSPEND_SYS_SYNC_TIMEOUT);
		wait_for_completion(&suspend_sys_sync_comp);
	}

	if (suspend_sys_sync_abort) {
		pr_buf_err("suspend aborted....while waiting for sys_sync\n");
		return -EBUSY;
	}

	return 0;
}
EXPORT_SYMBOL(his_suspend_sys_sync_wait);

static int __init sys_sync_queue_init(void)
{
	int ret = 0;

	init_completion(&suspend_sys_sync_comp);

	suspend_sys_sync_workqueue =
			create_singlethread_workqueue("suspend_sys_sync");
	if (suspend_sys_sync_workqueue == NULL)
		ret = -ENOMEM;

	return ret;
}

core_initcall(sys_sync_queue_init);
