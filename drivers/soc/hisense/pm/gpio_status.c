#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/his_debug_base.h>

int __weak print_all_gpio_status(struct seq_file *m, void *v)
{
	return 0;
}

static int gpio_status_proc_show(struct seq_file *m, void *v)
{
	return print_all_gpio_status(m, v);
}

void gpio_status_sleep_show(void)
{
	print_all_gpio_status(NULL, NULL);
}

static int gpio_status_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, gpio_status_proc_show, NULL);
}

static const struct file_operations gpio_status_proc_fops = {
	.open     = gpio_status_proc_open,
	.read     = seq_read,
	.llseek   = seq_lseek,
	.release  = single_release,
};

static int __init gpio_status_proc_init(void)
{
	his_create_procfs_file("gpio_status", 0, &gpio_status_proc_fops);
	return 0;
}
module_init(gpio_status_proc_init);
