#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/sched/clock.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <linux/his_debug_base.h>

#define MAX_DEVICES_NUM   128

static int debug_resume_enable;
struct debug_device_resume
{
	const char *dev_name;
	u64 start_time;
	u64 resume_time;
    enum suspend_stat_step step;
};

static int device_cnt;
static struct debug_device_resume resume_stat[MAX_DEVICES_NUM];

void clear_resume_device_count(void)
{
	device_cnt = 0;
}

void debug_show_time_step(bool starttime, const char *name, enum suspend_stat_step step)
{
	u64 end_time;

	if (!debug_resume_enable)
		return;

	if (device_cnt >= MAX_DEVICES_NUM)
		return;

	if (starttime) {
		resume_stat[device_cnt].start_time = local_clock();
	} else {
		end_time = local_clock();

		resume_stat[device_cnt].resume_time = end_time -
			resume_stat[device_cnt].start_time;
		if (resume_stat[device_cnt].resume_time > 1000000) {
			resume_stat[device_cnt].dev_name = name;
            resume_stat[device_cnt].step = step;
			device_cnt ++;
		}
  	}
}

/* Backward compatibility */
void debug_show_time(bool starttime, const char *name)
{
    debug_show_time_step(starttime, name, SUSPEND_RESUME);
}

static int device_resume_time_show(struct seq_file *m, void *v)
{
	int i = 0;
	u64 msec = 0;
	unsigned long rem_nsec = 0;

	pr_err("Device resume time(count:%d):\n", device_cnt);
	for (i = 0; i < device_cnt; i++) {
		msec = resume_stat[i].resume_time;
		rem_nsec = do_div(msec, 1000000);
		seq_printf(m, "%s step %d resume_time : [%3lu.%6lu ms]\n",
				resume_stat[i].dev_name, resume_stat[i].step,
				(unsigned long)msec, rem_nsec);
	}

	return 0;
}

static int device_resume_time_open(struct inode *inode, struct file *file)
{
	return single_open(file, device_resume_time_show, NULL);
}

static ssize_t device_resume_time_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	char c = 0;

	if (get_user(c, buffer))
		return -EFAULT;

	if (c == '1')
		debug_resume_enable = 1;
	else
		debug_resume_enable = 0;

	pr_err("Set debug_resume_enable to %d\n", debug_resume_enable);
	return count;
}

static const struct file_operations device_resume_time_fops= {
	.open     = device_resume_time_open,
	.read     = seq_read,
	.llseek   = seq_lseek,
	.release  = single_release,
	.write    = device_resume_time_write,
};

static int __init init_debug_device_resume_time(void)
{
	int ret;

	ret = his_create_procfs_file("dev_resume_time", S_IRUGO,
			&device_resume_time_fops);
	if (ret < 0)
		return -ENOMEM;

	return 0;
}
late_initcall(init_debug_device_resume_time);

