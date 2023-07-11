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

#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unistd.h>
#include <linux/vmalloc.h>
#include <linux/sort.h>
#include <linux/utsname.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/time64.h>
#include <linux/rtc.h>
#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/his_debug_base.h>
#include "rs_common.h"

int rs_save_file(const char *dir, const char *name, const void *buff, u32 len)
{
	long bytes;
	char xname[RS_LOG_PATH_LEN] = {0};

	if (NULL == dir || NULL == name) {
		pr_err("file name and dir should not null\n");
		return -EINVAL;
	}

	if ((strlen((const char *)dir) + strlen((const char *)name)  + 1)
			>= RS_LOG_PATH_LEN) {
		pr_err("error: dir is too long, exit\n");
		return -EINVAL;
	}

	memset(xname, 0, sizeof(xname));
	snprintf(xname, sizeof(xname), "%s/%s", dir, name);

	bytes = his_write_file(xname, 0, (void *)buff, len);
	if (bytes != len) {
		pr_err("save file failed, write %ld orig %d\n", bytes, len);
		return -EIO;
	}
	pr_info("save file %s success\n", xname);

	return 0;
}

static int __rs_mkdir(const char *name, umode_t mode)
{
	struct dentry *dentry;
	struct path path;
	int err;

	dentry = kern_path_create(AT_FDCWD, name, &path, LOOKUP_DIRECTORY);
	if (IS_ERR(dentry))
		return PTR_ERR(dentry);

	err = vfs_mkdir(path.dentry->d_inode, dentry, mode);
	done_path_create(&path, dentry);

	return err;
}

int rs_create_dir(const char *nodepath)
{
	int err = 0;
	char *s;
	char *cur_path = NULL;

	if ((*nodepath != '/') || (strlen(nodepath) >= RS_LOG_PATH_LEN))
		return -EINVAL;

	/* parent directories do not exist, create them */
	cur_path = kstrdup(nodepath, GFP_KERNEL);
	if (!cur_path)
		return -ENOMEM;

	s = cur_path + sizeof(RS_DUMP_PATH);
	for (;;) {
		s = strchr(s, '/');
		if (!s)
			break;
		s[0] = '\0';
		err = __rs_mkdir(cur_path, 0755);
		if (err && err != -EEXIST)
			break;
		s[0] = '/';
		s++;
	}
	kfree(cur_path);

	return err;
}

static int rs_rm_file(char *filename, u64 arg1, u64 arg2, int size)
{
	int ret;
	mm_segment_t old_fs;

	if (filename == NULL)
		return 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	ret = ksys_access(filename, 0);
	if (0 == ret) {
		if (ksys_unlink(filename)) {
			pr_err("%s: unlink %s failed\n", __func__, filename);
			set_fs(old_fs);
			return -ENOENT;
		}
	}

	set_fs(old_fs);
	return 0;
}

static int rs_rm_dir(char *path)
{
	char *pdst = path;
	int ret = 0;

	while (*pdst)
		pdst++;
	pdst--;
	if (*pdst == '/')
		*pdst = '\0';

	ret = ksys_rmdir(path);
	if (ret != 0)
		pr_err("%s: remove %s failed %d\n", __func__, path, ret);

	return ret;
}

#define RDRDIRSIZ            1024
#define RS_COMPRESS_BUFF_SZ  512*1024
typedef int (*rs_funcptr)(char *, u64, u64, int);

int rs_list_dir(char *path, rs_funcptr fn, u64 arg1, u64 arg2, int arg3,
		int *cnt, int type)
{
	int fd = -1, nread, bpos, ret = 0, tmp_cnt = 0;
	char *buf = NULL;
	struct linux_dirent64 *d;
	char d_type;
	mm_segment_t old_fs;
	char fullname[RS_LOG_PATH_LEN] = {0};

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = ksys_open(path, O_RDONLY, 0664);
	if (fd < 0) {
		pr_err("%s: open %s fail, fd:%d\n", __func__, path, fd);
		ret = -1;
		goto out;
	}

	buf = vmalloc(RDRDIRSIZ);
	if (buf == NULL) {
		pr_err("%s: vmalloc failed\n", __func__);
		ret = -1;
		goto out;
	}

	for (;;) {
		nread = ksys_getdents64(fd, (struct linux_dirent64 *)buf, RDRDIRSIZ);
		if (nread == -1) {
			pr_err("%s: sys_getdents failed\n", __func__);
			ret = -1;
			break;
		}

		if (nread == 0) {
			ret = 0;
			break;
		}

		for (bpos = 0; bpos < nread;) {
			d = (struct linux_dirent64 *)(buf + bpos);
			d_type = d->d_type;
			if ((d_type == type) && (fn != NULL)) {
				snprintf(fullname, sizeof(fullname), "%s%s",
						path, d->d_name);
				fn(fullname, arg1, arg2, arg3);
			}
			if (d_type == type)
				tmp_cnt++;
			bpos += d->d_reclen;
		}
	}
	if (cnt != (int *)0)
		*cnt = tmp_cnt;
	vfree(buf);

out:
	if (fd >= 0)
		ksys_close(fd);
	set_fs(old_fs);

	return ret;
}

int rs_remove_whole_dir(char *path)
{
	int ret;

	rs_list_dir(path, (rs_funcptr)rs_rm_file,
				0, 0, 0, &ret, DT_REG);
	rs_rm_dir(path);

	return 0;
}

static char *rs_get_file_name(char *filename_withpath)
{
	char *p = filename_withpath;

	if (*p == '\0')
		return NULL;

	while (*p)
		p++;

	while ((*p != '/') && (p != filename_withpath))
		p--;

	if (*p == '/') {
		p++;
		return p;
	}

	return NULL;
}

static void *rs_allocmem(size_t size)
{
	if (size <= PAGE_SIZE)
		return kzalloc(size, GFP_KERNEL);
	else
		return vzalloc(size);
}

static void rs_freemem(void *ptr)
{
	if (ptr != NULL && is_vmalloc_addr(ptr))
		return vfree(ptr);

	kfree(ptr);
}

int rs_file2zfile(char *fullname, u64 zipfd, u64 zip_head, int size)
{
	struct kstat m_stat;
	int ret, i, out_cnt, in_len;
	struct file *zfilp = (struct file *)zipfd;
	struct rs_archive_file *zfile = (struct rs_archive_file *)zip_head;
	struct rs_archive_head *head = &zfile->head;
	char *zip_buf = (char *)(zip_head + size);
	char *name = rs_get_file_name(fullname);
	char *orig_data = NULL;

	if (name == NULL) {
		pr_err("%s: name is null...return.\n", __func__);
		return -EINVAL;
	}

	ret = vfs_stat(fullname, &m_stat);
	if (ret) {
		pr_err("%s: file %s not exist!\n", __func__, fullname);
		return -ENOENT;
	}

	if (m_stat.size <= 0) {
		pr_err("%s: %s size is error!\n", __func__, fullname);
		return -EINVAL;
	}

	orig_data = rs_allocmem(m_stat.size + 1);

	if (!orig_data) {
		pr_err("%s: alloc mem failed\n", __func__);
		return -ENOMEM;
	}

	in_len = his_read_file(fullname, 0, orig_data, m_stat.size);
	if (in_len <= 0) {
		pr_info("%s: read the file to zip end.", __func__);
		rs_freemem(orig_data);
		return 0;
	}

	for (i = 0; i < zfile->file_num; i++) {
		if (*head[i].filename == '\0')
			break;
	}

	out_cnt = rs_recorder_compress(orig_data, zip_buf,
			in_len, RS_COMPRESS_BUFF_SZ);
	if (out_cnt < 0) {
		pr_err("%s: after compress size %d, inlen:%d.\n",
				__func__, out_cnt, in_len);
		if (in_len < 100) {
			memcpy(zip_buf, (char *)orig_data, in_len);
			out_cnt = in_len;
		} else {
			rs_freemem(orig_data);
			return out_cnt;
		}
	}

	ret = vfs_write(zfilp, zip_buf, out_cnt, &zfilp->f_pos);
	if (ret < out_cnt) {
		pr_err("%s: sys_write return %d.\n", __func__, ret);
		rs_freemem(orig_data);
		return -EIO;
	}

	if (i == 0)
		head[i].off = size;
	else
		head[i].off = head[i - 1].off + head[i - 1].zip_len;

	strlcpy(head[i].filename, name, RS_ARCHIVE_FNAME_LEN);
	head[i].filename[RS_ARCHIVE_FNAME_LEN - 1] = '\0';
	head[i].orig_len = in_len;
	head[i].zip_len = out_cnt;
	rs_freemem(orig_data);

	return 0;
}

static int rs_compress_dir(char *path, char *db_name)
{
	int ret;
	struct kstat m_stat;
	mm_segment_t old_fs;
	struct file *filp = NULL;
	struct rs_archive_file *dst;
	char fname[RS_LOG_PATH_LEN];

	if (path == NULL) {
		pr_err("The path pointer is NULL\n");
		return -EINVAL;
	}

	ret = vfs_stat(path, &m_stat);
	if (ret) {
		pr_err("%s: dir not exist, exit\n", __func__);
		return -ENOENT;
	}

	/* check path file count, if count=0, del and return */
	rs_list_dir(path, NULL, 0, 0, 0, &ret, DT_REG);
	pr_info("%s: file cnt is %d\n", __func__, ret);
	if (ret > 0) {
		size_t siz = sizeof(struct rs_archive_file) +
				sizeof(struct rs_archive_head) * (ret - 1);
		size_t siz2 = siz + RS_COMPRESS_BUFF_SZ;

		dst = vzalloc(siz2);
		if (dst == NULL) {
			pr_err("%s: vmalloc dst failed\n", __func__);
			return -ENOMEM;
		}

		dst->file_magic = RS_ARCHIVE_FILE_MAGIC;
		dst->file_num = ret;

		snprintf(fname, sizeof(fname), "%s%s.db", RS_DUMP_PATH,
				db_name);
		filp = filp_open(fname, O_RDWR | O_CREAT, 0644);
		if (IS_ERR(filp)) {
			pr_err("create zipfile %s err.\n", fname);
			vfree(dst);
			return PTR_ERR(filp);
		}

		old_fs = get_fs();
		set_fs(KERNEL_DS);

		filp->f_op->llseek(filp, siz, SEEK_SET);
		rs_list_dir(path, (rs_funcptr)rs_file2zfile, (u64)filp,
				(u64)dst, siz, &ret, DT_REG);

		/* write head to file: */
		filp->f_op->llseek(filp, 0, SEEK_SET);
		ret = vfs_write(filp, (char *)dst, siz, &filp->f_pos);
		if (ret < siz) {
			pr_err("%s: write head failed\n", __func__);
			set_fs(old_fs);
			filp_close(filp, NULL);
			vfree(dst);
			return -EIO;
		}

		set_fs(old_fs);
		filp_close(filp, NULL);
		vfree(dst);
	} else {
		pr_info("delete empty dir %s", path);
		ret = ksys_rmdir(path);
		if (ret != 0)
			pr_err("%s: delete dir %s fail\n", __func__, path);
	}

	return 0;
}

void create_archive_file(char *path, char *name)
{
	pr_info("create archive file: %s\n", name);
	rs_compress_dir(path, name);
}

static struct file_ctime_info oldest_file;

static int rs_find_oldest_file(char *filename, u64 arg1, u64 arg2, int size)
{
	int ret = 0;
	struct kstat m_stat;
	static int run_times;

	/* can not delete the reboot_reason file */
	if (strstr(filename, "reboot_reason")) {
		pr_err("skip the reboot_reason file\n");
		return 0;
	}

	ret = vfs_stat(filename, &m_stat);
	if (ret) {
		pr_err("%s: file %s not exist!\n", __func__, filename);
		return -ENOENT;
	}

	if (run_times == 0) {
		strlcpy(oldest_file.filename, filename, RS_LOG_PATH_LEN);
		oldest_file.ctime = m_stat.ctime;
		run_times++;
		return 0;
	}

	run_times++;
	if (run_times >= size) {
		pr_info("run_times is %d, reset run_times\n", run_times);
		run_times = 0;
	}

	/*
	 * lhs < rhs:  return <0
	 * lhs == rhs: return 0
	 * lhs > rhs:  return >0
	 */
	if (timespec64_compare(&oldest_file.ctime, &m_stat.ctime) > 0) {
		strlcpy(oldest_file.filename, filename, RS_LOG_PATH_LEN);
		oldest_file.ctime = m_stat.ctime;
	}

	return 0;
}

int rs_rm_file_bylimit(char *path)
{
	int ret = 0;
	int num = 0;
	struct kstat m_stat;

	ret = vfs_stat(path, &m_stat);
	if (ret) {
		pr_err("%s: dir not exist!\n", __func__);
		return -ENOENT;
	}

	rs_list_dir(path, NULL, 0, 0, 0, &num, DT_REG);
	pr_info("The current dir exist %d file\n", num);
	if (num > RS_FILE_NUM_LIMIT) {
		/* find the oldest file */
		rs_list_dir(path, (rs_funcptr)rs_find_oldest_file,
				0, 0, num, &ret, DT_REG);

		rs_rm_file(oldest_file.filename, 0, 0, 0);
		memset(&oldest_file, 0, sizeof(struct file_ctime_info));
	}

	return 0;
}

