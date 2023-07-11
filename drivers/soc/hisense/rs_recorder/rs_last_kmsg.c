/*
 * rs_recorder (Runtime State Recorder) Module
 *
 * Copyright (C) 2016 Hisense, Inc.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/fdtable.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include "rs_common.h"

static char * rs_last_kmsg_ptr;
static int rs_last_kmsg_exist;
static int s_read_times;

#define RS_RECORDER_BUF_SIZE   ((1 << CONFIG_LOG_BUF_SHIFT) + 1024)
#define RS_MAX_READ_TIMES      RS_RECORDER_BUF_SIZE/PAGE_SIZE

static ssize_t rs_last_kmsg_read(struct file *file, char __user *ubuf,
				       size_t count, loff_t *ppos)
{
	int ret = 0;
	int read_size = 0;
	int buf_size = PAGE_SIZE;
	char *tmpbuf = NULL;
	char *readbuf = NULL;

	if (rs_last_kmsg_exist == 0) {
		pr_err("no last kmsg information\n");
		return 0;
	}

	tmpbuf = kzalloc(buf_size, GFP_KERNEL);
	if (tmpbuf == NULL) {
		pr_err("%s: alloc mem failed\n", __func__);
		s_read_times = 0;
		return -ENOMEM;
	}

	if (s_read_times < RS_MAX_READ_TIMES) {
		readbuf = rs_last_kmsg_ptr + PAGE_SIZE * s_read_times;
		read_size = scnprintf(tmpbuf, buf_size, "%s\n", readbuf);
		s_read_times ++;
	} else {
		pr_err("%s: read the last_kmsg\n", __func__);
		s_read_times = 0;
		ret = 0;
		goto last_call;
	}

	*ppos = 0;
	ret = simple_read_from_buffer(ubuf, count, ppos, tmpbuf, read_size);

last_call:
	kfree(tmpbuf);
	return ret;
}

static const struct file_operations rs_last_kmsg_proc_fops = {
	.read       = rs_last_kmsg_read,
};

int init_last_kmsg_buffer(const char *msg_buf, int len)
{
	int offset = 0;
	int copy_len = 0;

	if (rs_last_kmsg_ptr != NULL) {

		if (len > RS_RECORDER_BUF_SIZE) {
			offset = len - RS_RECORDER_BUF_SIZE;
			copy_len = RS_RECORDER_BUF_SIZE - 1;
		} else {
			offset = 0;
			copy_len = len;
		}

		memcpy(rs_last_kmsg_ptr, msg_buf + offset, copy_len);
		rs_last_kmsg_exist = 1;
		pr_err("%s: last panic happen, save last kmsg\n", __func__);
	}

	return 0;
}

int init_last_kmsg_proc_node(void)
{
	rs_last_kmsg_ptr = kzalloc(RS_RECORDER_BUF_SIZE, GFP_KERNEL);
	if (rs_last_kmsg_ptr == NULL)
		return -ENOMEM;

	proc_create("last_kmsg", 0, NULL, &rs_last_kmsg_proc_fops);

	return 0;
}
