
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/his_debug_base.h>

/* set_debug must be called after pon register */
#define HS_SET_DEBUG_DELAY_MS 8000
void __weak reset_level_debug_set(void) { }

static struct delayed_work set_debug_work;

int __weak hs_disable_sdi(void)
{
	return 0;
}

static ssize_t debug_flag_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t len)
{
	int value;

	if (sscanf(buf, "%d", &value) != 1) {
		pr_err("debug_store: sscanf is wrong!\n");
		return -EINVAL;
	}

	pr_buf_err("%s debug_flag\n", value ? "enable" : "disable");
	if (value == 1)
		set_debug_flag_bit(DEBUG_ENABLE_BIT);
	else
		clear_debug_flag_bit(DEBUG_ENABLE_BIT);

	reset_level_debug_set();

	return len;
}

static ssize_t debug_flag_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	bool enable = false;

	enable = get_debug_flag_bit(DEBUG_ENABLE_BIT);
	return snprintf(buf, PAGE_SIZE, "%d\n", enable);
}
static struct kobj_attribute debug_flag_attr = __ATTR_RW(debug_flag);

static ssize_t disable_sdi_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t len)
{
	int value;

	if (sscanf(buf, "%d", &value) != 1) {
		pr_err("disable sdi: sscanf is wrong!\n");
		return -EINVAL;
	}

	pr_buf_err("sdi disabled %d\n", value);
	if (value == 1)
		hs_disable_sdi();

	return len;
}
static struct kobj_attribute disable_sdi_attr = __ATTR_WO(disable_sdi);

static struct attribute *his_debug_attrs[] = {
	&debug_flag_attr.attr,
	&disable_sdi_attr.attr,
	NULL,
};

static struct attribute_group his_debug_attr_group = {
	.attrs = his_debug_attrs,
};

static void set_boot_debug_flag_work(struct work_struct *work)
{

	reset_level_debug_set();
}

void debug_flag_control_init(void)
{
	int ret;

	INIT_DELAYED_WORK(&set_debug_work, set_boot_debug_flag_work);
	schedule_delayed_work(&set_debug_work, msecs_to_jiffies(HS_SET_DEBUG_DELAY_MS));
	ret = his_register_sysfs_attr_group(&his_debug_attr_group);
	if (ret < 0) {
		pr_err("Error creating debug_flag sysfs node\n");
		return;
	}

	pr_info("%s: OK.\n", __func__);
}

