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

#include <linux/stddef.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/version.h>
#include <linux/memcontrol.h>
#include <linux/uaccess.h>
#include <mm/slab.h>
#include <linux/slub_def.h>
#include "rs_common.h"

/* need support CONFIG_SLUB */
#define OO_SHIFT 16
#define OO_MASK ((1 << OO_SHIFT) - 1)

#ifdef CONFIG_SLUB_DEBUG
static inline int rs_oo_order(struct kmem_cache_order_objects x)
{
	return x.x >> OO_SHIFT;
}

static inline int rs_oo_objects(struct kmem_cache_order_objects x)
{
	return x.x & OO_MASK;
}

static int rs_count_free(struct page *page)
{
	if (unlikely(NULL == page))
		return 0;

	return page->objects - page->inuse;
}

static unsigned long rs_count_partial(struct kmem_cache_node *n,
		int (*get_count)(struct page *))
{
	unsigned long flags;
	unsigned long x = 0;
	struct page *page;

	if (unlikely(NULL == n || NULL == get_count))
		return 0;

	if (spin_trylock_irqsave(&n->list_lock, flags)) {
		list_for_each_entry(page, &n->partial, lru) {
			x += get_count(page);
		}

		spin_unlock_irqrestore(&n->list_lock, flags);
	}

	return x;
}

#ifdef CONFIG_MEMCG_KMEM
static inline bool rs_is_root_cache(struct kmem_cache *s)
{
	return !s->memcg_params.root_cache;
}

static inline const char *rs_cache_name(struct kmem_cache *s)
{
	if (!rs_is_root_cache(s))
		s = s->memcg_params.root_cache;

	return s->name;
}
#else
static bool rs_is_root_cache(struct kmem_cache *s)  { return true; }
static const char *rs_cache_name(struct kmem_cache *s) { return s->name; }
#endif

static void rs_do_get_slabinfo(struct kmem_cache *s, struct slabinfo *sinfo)
{
	unsigned long nr_slabs = 0;
	unsigned long nr_objs = 0;
	unsigned long nr_free = 0;
	int node;
	struct kmem_cache_node *n;

	for_each_kmem_cache_node(s, node, n) {
		nr_slabs += atomic_long_read(&n->nr_slabs);
		nr_objs += atomic_long_read(&n->total_objects);
		nr_free += rs_count_partial(n, rs_count_free);
	}

	sinfo->active_objs = nr_objs - nr_free;
	sinfo->num_objs = nr_objs;
	sinfo->active_slabs = nr_slabs;
	sinfo->num_slabs = nr_slabs;
	sinfo->objects_per_slab = rs_oo_objects(s->oo);
	sinfo->cache_order = rs_oo_order(s->oo);
}

static void rs_memcg_accumulate_slabinfo(struct kmem_cache *s, struct slabinfo *info)
{
	struct kmem_cache *c;
	struct slabinfo sinfo;

	if (!rs_is_root_cache(s))
		return;

	for_each_memcg_cache(c, s) {
		memset(&sinfo, 0, sizeof(sinfo));
		rs_do_get_slabinfo(c, &sinfo);

		info->active_slabs += sinfo.active_slabs;
		info->num_slabs += sinfo.num_slabs;
		info->shared_avail += sinfo.shared_avail;
		info->active_objs += sinfo.active_objs;
		info->num_objs += sinfo.num_objs;
	}
}

static int rs_write_slabinfo_header(struct rs_recorder_log *log)
{
	int size = 0;
	char *buff = log->log_buf + log->w_pos;

	size = snprintf(buff, log->left, "%s",
			"slabinfo - version: 2.1\n"
			"# name            <active_objs> <num_objs> <objsize> <objperslab> <pagesperslab>"
			" : tunables <limit> <batchcount> <sharedfactor>"
			" : slabdata <active_slabs> <num_slabs> <sharedavail>\n");

	return size;
}

static void *rs_slabinfo_start(struct list_head *cache_chain, loff_t *pos)
{
	return seq_list_start(cache_chain, *pos);
}

static void *rs_slabinfo_next(void *p, struct list_head *cache_chain, loff_t *pos)
{
	return seq_list_next(p, cache_chain, pos);
}

static int rs_get_slabinfo(struct rs_recorder_log *log, void *p, int size)
{
	int info_len = 0;
	char *buff = NULL;
	struct slabinfo sinfo = {0};
	struct kmem_cache *s = list_entry(p, struct kmem_cache, list);

	buff = log->log_buf + log->w_pos + size;
	memset(&sinfo, 0, sizeof(sinfo));

	rs_do_get_slabinfo(s, &sinfo);
	rs_memcg_accumulate_slabinfo(s, &sinfo);
	info_len = snprintf(buff, log->left,
		"%-17s %6lu %6lu %6u %4u %4d"
		" : tunables %4u %4u %4u"
		" : slabdata %6lu %6lu %6lu\n",
		rs_cache_name(s), sinfo.active_objs, sinfo.num_objs, s->size, sinfo.objects_per_slab, (1 << sinfo.cache_order),
		sinfo.limit, sinfo.batchcount, sinfo.shared,
		sinfo.active_slabs, sinfo.num_slabs, sinfo.shared_avail);

	return info_len;
}

int rs_dump_all_slabinfo(struct rs_recorder_log *log, char *fname)
{
	loff_t pos = 0;
	void *ptr = NULL;
	char *buff;
	u32 data_size = 0;
	/* The slab cache mutex protects the management structures during changes */
	extern struct mutex slab_mutex;
	/* The list of all slab caches on the system */
	extern struct list_head slab_caches;

	buff = log->log_buf + log->w_pos;
	data_size = rs_write_slabinfo_header(log);

	if (mutex_trylock(&slab_mutex)) {
		ptr = rs_slabinfo_start(&slab_caches, &pos);

		while (1) {
			if (ptr == NULL)
				break;

			data_size += rs_get_slabinfo(log, ptr, data_size);
			ptr = rs_slabinfo_next(ptr, &slab_caches, &pos);
		}

		mutex_unlock(&slab_mutex);
	}

	if (!log->is_panic)
		rs_save_file(log->path, fname, buff, data_size);

	rs_update_buf_header(log, fname, data_size);

	return 0;
}
#endif /* CONFIG_SLUB_DEBUG */

