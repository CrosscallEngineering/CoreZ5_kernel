/*
 * rs_recorder (Runtime State Recorder) Module
 *
 * Copyright (C) 2015 Hisense, Inc.
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

#define pr_fmt(fmt) "rs-recorder: " fmt

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>
#include <linux/export.h>
#include <linux/vmalloc.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/timex.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/dma-mapping.h>
#include <linux/productinfo.h>
#include <linux/his_debug_base.h>
#include <linux/rs_recorder.h>
#include <hiconfig/kdebug_parti_layout.h>
#include "rs_common.h"

/* global recorder buf data */
static struct rs_recorder_log s_record_buf;

static struct rs_dump_info rs_dump_info;
static struct rs_common_info g_common_info;
static struct work_struct dump_work;
static struct delayed_work check_dump_work;

#define CHECK_DUMP_TIMEOUT      (60*1000)

static DEFINE_MUTEX(rs_mutex_lock);
static int rs_recorder_ready = 0;

struct rs_module_ops {
	char *name;
	int level;
	char *filename;
	dump_func dump_ops;
	struct rs_module_ops *next;
};

static struct rs_module_ops *dump_sets;

int rs_register_dumpops(char *name, int level,
		char *savename, dump_func dump_ops)
{
	struct rs_module_ops **priv = NULL;
	struct rs_module_ops **end  = NULL;

	end = &dump_sets;
	/* find the list endpoint */
	while (*end) {
		priv = end;
		end = &((*end)->next);
	}

	*end = (struct rs_module_ops *)kzalloc(sizeof(struct rs_module_ops),
			GFP_KERNEL);
	if (NULL == *end) {
		pr_err("register %s dump_ops failed\n", name);
		return -ENOMEM;
	}

	(*end)->name = name;
	(*end)->filename = savename;
	(*end)->level = level;
	(*end)->dump_ops = dump_ops;
	(*end)->next = NULL;
	if (priv)
		(*priv)->next = *end;

	return 0;
}

static char *rs_get_product_name(void)
{
	return (char *)CONFIG_HISENSE_PRODUCT_NAME;
}

static void rs_get_kernel_ver(char *buf, int buf_len)
{
	snprintf(buf, buf_len, "%s%s%s", utsname()->sysname,
		utsname()->release, utsname()->version);
}

static void rs_get_productinfo(char *buf)
{
	productinfo_dump(buf, 0);
}

void rs_update_buf_header(struct rs_recorder_log *log, char *fname, u32 len)
{
	struct rs_buf_header *pheader;

	mutex_lock(&rs_mutex_lock);
	pheader = (struct rs_buf_header *)log->head_ptr;

	memset(pheader->items[pheader->num].filename, 0, RS_ARCHIVE_FNAME_LEN);
	memcpy(pheader->items[pheader->num].filename, fname, strlen(fname));
	pheader->items[pheader->num].off = log->w_pos;
	pheader->items[pheader->num].orig_len = len;
	pheader->num++;
	pheader->check_sum += log->w_pos;
	pheader->check_sum += len;
	pr_err("the '%s' offset is %d, len is %d\n", fname, log->w_pos, len);

	log->w_pos += len;
	log->left  -= len;
	mutex_unlock(&rs_mutex_lock);
}

static int rs_dump_common_init(struct rs_recorder_log *log, char *fname)
{
	u32 len = 0;
	char *pbuf = log->log_buf;
	char *vstr = "version.txt";
	char *pinfo = "productinfo.txt";

	rs_get_productinfo(g_common_info.productinfo);

	if (!log->is_panic) {
		rs_save_file(log->path, vstr, g_common_info.kernel_ver,
			strlen(g_common_info.kernel_ver));
		rs_save_file(log->path, pinfo, g_common_info.productinfo,
			strlen(g_common_info.productinfo));
	} else {
		len = strlen(g_common_info.kernel_ver);
		memcpy(pbuf + log->w_pos, g_common_info.kernel_ver, len);
		rs_update_buf_header(log, vstr, len);

		len = strlen(g_common_info.productinfo);
		memcpy(pbuf + log->w_pos, g_common_info.productinfo, len);
		rs_update_buf_header(log, pinfo, len);
	}

	return 0;
}

void rs_recorder_dump_log(char *reason, int level, int line, int is_panic)
{
	int ret = 0;
	struct timespec64 ts;
	struct rtc_time tm;
	char time_str[32] = {0};
	struct rs_buf_header *pheader;
	char fname[RS_ARCHIVE_FNAME_LEN] = {0};
	struct rs_module_ops *m_ops;

	mutex_lock(&rs_mutex_lock);
	if (rs_recorder_ready == 0) {
		mutex_unlock(&rs_mutex_lock);
		return;
	}

	s_record_buf.w_pos = 0;
	s_record_buf.left = s_record_buf.buf_size;

	s_record_buf.log_dump_pos = line;
	s_record_buf.is_panic = is_panic;

	pr_info("enter dump log, reason:%s,level:%d\n", reason, level);
	/* init path by reason and time */
	ktime_get_real_ts64(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	snprintf(time_str, sizeof(time_str), "%d-%d-%d_%d_%d_%d",
		tm.tm_year + 1900,
		tm.tm_mon+1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec);

	snprintf(fname, sizeof(fname), "%s_%s", reason, time_str);
	memset(s_record_buf.path, 0, RS_LOG_PATH_LEN);
	snprintf(s_record_buf.path, RS_LOG_PATH_LEN, "%s%s/",
			RS_DUMP_PATH, fname);
	mutex_unlock(&rs_mutex_lock);

	pheader = (struct rs_buf_header *)s_record_buf.head_ptr;
	if (is_panic)
		pheader->magic = RS_LOG_PANIC_MAGIC;
	else
		pheader->magic = RS_LOG_GENERIC_MAGIC;

	/* add 1 for copy string end char */
	memcpy(pheader->path, s_record_buf.path, strlen(s_record_buf.path)+1);
	pheader->num = 0;
	pheader->check_sum = 0;

	if (!is_panic) {
		ret = rs_create_dir(s_record_buf.path);
		if (ret < 0)
			return;
	}

	m_ops = dump_sets;
	while (m_ops) {
		if (m_ops->level <= level) {
			char *fname = m_ops->filename;
			m_ops->dump_ops(&s_record_buf, fname);
		}

		m_ops = m_ops->next;
	}

	if (!is_panic) {
		create_archive_file(s_record_buf.path, fname);
		rs_remove_whole_dir(s_record_buf.path);
		rs_rm_file_bylimit(RS_DUMP_PATH);
		memset(s_record_buf.path, 0, RS_LOG_PATH_LEN);
	}
}

static int rs_check_header_check_sum(struct rs_buf_header *pheader)
{
	int i = 0;
	u32 curr_check_sum = 0;

	for (i = 0; i < pheader->num; i++) {
		curr_check_sum += pheader->items[i].orig_len;
		curr_check_sum += pheader->items[i].off;
	}

	if (pheader->check_sum == curr_check_sum)
		return 0;

	pr_err("The calculated check sum is not matching\n");
	return 1;
}

static void rs_check_dump_work(struct work_struct *work)
{
	int ret = 0;
	int head_sz = 0;
	struct rs_buf_header *pheader;
	int buf_sz;
#ifndef CONFIG_SUPPORT_WARM_RESET
	char partition[128] = {0};
#endif /* CONFIG_SUPPORT_WARM_RESET */

	head_sz = sizeof(struct rs_buf_header);
	pheader = (struct rs_buf_header *)s_record_buf.head_ptr;

#ifndef CONFIG_SUPPORT_WARM_RESET
	buf_sz = s_record_buf.buf_size + head_sz;

	his_get_partition_path(partition, 128, SYS_DEBUG_PARTITION);
	ret = his_read_file(partition, RS_RECORDER_OFFSET_IN_KDEBUG,
			(char *)pheader, buf_sz);
	pr_info("the buf_size %d, read size is %d\n", buf_sz, ret);
	if (ret < 0)
		goto exit;
#endif /* CONFIG_SUPPORT_WARM_RESET */

	pr_info("the header magic is 0x%x\n", pheader->magic);
	if (pheader->magic != RS_LOG_PANIC_MAGIC) {
		pr_info("no panic dump!!\n");
		goto exit;
	}

	ret = rs_check_header_check_sum(pheader);
	if (0 == ret)
		rs_save_dump_to_file(pheader);

	/* delete information after saved */
	memset((char *)pheader, 0, head_sz);
#ifndef CONFIG_SUPPORT_WARM_RESET
	his_write_file(partition, RS_RECORDER_OFFSET_IN_KDEBUG,
			(char *)pheader, head_sz);
#endif /* CONFIG_SUPPORT_WARM_RESET */

exit:
	rs_recorder_ready = 1;
}

static void dump_kernel_info_work(struct work_struct *work)
{
	rs_recorder_dump_log(rs_dump_info.reason, rs_dump_info.level,
			rs_dump_info.line, rs_dump_info.is_panic);

	memset((char *)&rs_dump_info, 0, sizeof(struct rs_dump_info));
}

void rs_exec_dump_task(const char *reason, int level, int line, int is_panic)
{
	if (rs_recorder_ready == 0)
		return;

	memcpy(rs_dump_info.reason, reason, strlen(reason));
	rs_dump_info.level = level;
	rs_dump_info.line = line;
	rs_dump_info.is_panic = is_panic;

	schedule_work(&dump_work);
}

#ifdef CONFIG_DEBUG_FS
static int manual_dumplog_show(struct seq_file *m, void *v)
{
	char *show_msg = "use 'echo 1 > manual_dump' to dump.\n";

	seq_write(m, show_msg, strlen(show_msg));
	return 0;
}

static ssize_t manual_dump_write(struct file *file, const char __user *data,
							size_t size, loff_t *offset)
{
	int value;
	char buf[4];

	if (size > sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, data, size))
		return -EINVAL;

	sscanf(buf, "%u\n", &value);
	if (value == 1)
		rs_exec_dump_task("manual_dump", 3, __LINE__, false);
	else if (2 == value)
		rs_rm_file_bylimit(RS_DUMP_PATH);

	return size;
}

static int test_dump_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, manual_dumplog_show, NULL);
}

static const struct file_operations dev_manual_dump_fops = {
	.open       = test_dump_proc_open,
	.read       = seq_read,
	.write      = manual_dump_write,
	.llseek     = seq_lseek,
	.release    = single_release,
};
#endif

static int rs_recorder_parse_dt(struct device *dev, u32 *data)
{
	struct device_node *pnode;
	const u32 *addr;
	u64 size;

	pnode = of_parse_phandle(dev->of_node, "linux,contiguous-region", 0);
	if (pnode == NULL) {
		pr_err("mem reservation for rs-recorder not present\n");
		return -EINVAL;
	}

	addr = of_get_address(pnode, 0, &size, NULL);
	if (!addr) {
		pr_err("failed to parse the reserve memory address\n");
		of_node_put(pnode);
		return -EINVAL;
	}

	data[0] = (u32) of_read_ulong(addr, 2);
	data[1] = (u32) size;

	return 0;
}

static int
rs_task_panic(struct notifier_block *this, unsigned long event, void *ptr)
{
	pr_info("%s: panic notify rs_recorder to save log\n", __func__);
	rs_recorder_dump_log("panic", 2, __LINE__, true);

	return NOTIFY_DONE;
}

static struct notifier_block panic_block = {
	.notifier_call = rs_task_panic,
};

static int rs_register_common_ops(void)
{
	rs_register_dumpops("common",        1,  NULL,
			rs_dump_common_init);
	rs_register_dumpops("blocked_task",  3,  NULL,
			rs_print_blocked_task);
	rs_register_dumpops("meminfo",       1,  "meminfo.txt",
			rs_dump_meminfo);
	rs_register_dumpops("dmesg",         1,  "dmesg.txt",
			rs_dump_dmesg_log);
	rs_register_dumpops("ps_info",       2,  "ps.txt",
			rs_dump_allps_info);
	rs_register_dumpops("interrupts",    2,  "irqs.txt",
			rs_dump_interrupts);
#ifdef CONFIG_SLUB_DEBUG
	rs_register_dumpops("slabinfo",      3,  "slabinfo.txt",
			rs_dump_all_slabinfo);
#endif /* CONFIG_SLUB_DEBUG */

	return 0;
}

static int rs_recorder_probe(struct platform_device *pdev)
{
	int rc = 0;
	u32 offsets[2];
	void *vaddr;
	u32 buf_size = 0;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_dir;
#endif

	pr_info("%s enter\n", __func__);
	if (!pdev->dev.of_node) {
		pr_err("can not find of_node of device\n");
		return -EINVAL;
	}

	rc = rs_recorder_parse_dt(&pdev->dev, offsets);
	if (rc < 0) {
		pr_err("rs_recorder parse dts failed %d\n", rc);
		return -EINVAL;
	}

	buf_size = offsets[1];

	vaddr = ioremap_wc(offsets[0], buf_size);
	pr_info("The phys_base=0x%x, size=0x%x vaddr=%p\n",
			offsets[0], buf_size, vaddr);

	/* init rs_recorder buffer */
	s_record_buf.head_ptr = vaddr;
	s_record_buf.log_buf = vaddr + sizeof(struct rs_buf_header);
	memset(s_record_buf.path, 0, RS_LOG_PATH_LEN);
	s_record_buf.w_pos = 0;
	s_record_buf.buf_size = buf_size - sizeof(struct rs_buf_header);
	s_record_buf.left = s_record_buf.buf_size;
	s_record_buf.log_dump_pos = 0;

	rs_register_common_ops();

	atomic_notifier_chain_register(&panic_notifier_list, &panic_block);

	INIT_WORK(&dump_work, dump_kernel_info_work);
	INIT_DELAYED_WORK(&check_dump_work, rs_check_dump_work);

#ifdef CONFIG_DEBUG_FS
	debugfs_dir = his_register_debugfs_dir("rs_recorder");
	debugfs_create_file("manual_dump", 0660, debugfs_dir,
			NULL, &dev_manual_dump_fops);
#endif

	if (dev_bi.bootmode == BOOT_NORMAL_MODE)
		schedule_delayed_work(&check_dump_work,
				msecs_to_jiffies(CHECK_DUMP_TIMEOUT));

	return rc;
}

static struct of_device_id rs_recorder_match_table[] = {
	{.compatible = "rs-recorder"},
	{},
};

static struct platform_driver rs_recorder_driver = {
	.probe		= rs_recorder_probe,
	.driver		= {
		.name	= "rs-recorder",
		.owner	= THIS_MODULE,
		.of_match_table = rs_recorder_match_table,
	},
};

static int __init rs_recorder_main_init(void)
{
	struct rs_common_info *ptr = &g_common_info;
	char *pname = NULL;

	/* common infomation init */
	pname = rs_get_product_name();
	strlcpy((char *)ptr->product_name, pname, RS_PRODUCT_NAME_LEN);

	rs_get_kernel_ver(ptr->kernel_ver, RS_KERNEL_VER_LEN);
	init_last_kmsg_proc_node();

	return platform_driver_register(&rs_recorder_driver);
}

static void __exit rs_recorder_main_exit(void)
{
	iounmap(s_record_buf.head_ptr);
}

module_init(rs_recorder_main_init);
module_exit(rs_recorder_main_exit);

