
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/buffer_head.h>
#include <linux/slab.h>

#define MISC_ROOT_NAME    "misc_dev"

static struct kobject *miscfs_root;

static struct kobject *create_miscfs_root(void)
{
	static int create_ok = 0;

	if (create_ok == 1)
		goto out;

	miscfs_root = kobject_create_and_add(MISC_ROOT_NAME, NULL);
	if (!miscfs_root) {
		pr_err("%s: kobject create failed!\n", __func__);
		return NULL;
	}

	create_ok = 1;
out:
	return miscfs_root;
}

struct kobject *his_register_miscdev_dir(const char *miscname)
{
	struct kobject *kobj_root = NULL;
	struct kobject *creat_kobj = NULL;

	kobj_root = create_miscfs_root();
	if (!kobj_root) {
		pr_buf_err("%s create miscfs root failed\n", __func__);
		return NULL;
	}

	creat_kobj = kobject_create_and_add(miscname, kobj_root);
	if (!creat_kobj) {
		pr_buf_err("%s create miscdev name failed\n", __func__);
		return NULL;
	}

	return creat_kobj;
}

int his_register_miscdev_attr(struct attribute *attr)
{
	int ret = 0;
	struct kobject *kobj_root = NULL;

	if (!attr) {
		pr_err("%s attr is NULL\n", __func__);
		return -EINVAL;
	}

	kobj_root = create_miscfs_root();
	if (!kobj_root) {
		pr_err("%s kobject create failed\n", __func__);
		return -ENOMEM;
	}

	ret = sysfs_create_file(kobj_root, attr);
	if (ret < 0) {
		pr_err("%s Error creating sysfs attr %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

