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
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/hugetlb.h>
#include <linux/mman.h>
#include <linux/mmzone.h>
#include <linux/memory.h>
#include <linux/proc_fs.h>
#include <linux/percpu.h>
#include <linux/seq_file.h>
#include <linux/swap.h>
#include <linux/vmstat.h>
#include <linux/atomic.h>
#include <linux/vmalloc.h>
#ifdef CONFIG_CMA
#include <linux/cma.h>
#endif
#include <asm/page.h>
#include <asm/pgtable.h>
#include "rs_common.h"

static int rs_show_val_kb(char *b, const char *s, unsigned long num, int rest)
{
	char v[32] = {0};
	int out_len = 0;

	num_to_str(v, sizeof(v), num << (PAGE_SHIFT - 10), 0);
	out_len = snprintf(b, rest, "%s%s KB\n", s, v);

	return out_len;
}

#define SHOW_MEM_VALUE(b, len, name, value, left) do {\
		len += rs_show_val_kb(b + len, name, value, left); \
		left = all_left - len; \
	} while(0)

static int rs_get_meminfo(struct rs_recorder_log *log)
{
	struct sysinfo i;
	unsigned long committed;
	long cached;
	long available;
	unsigned long pages[NR_LRU_LISTS];
	unsigned long sreclaimable, sunreclaim;
	int lru;
	int info_len = 0;
	char *buf = NULL;
	int left = 0;
	int all_left = 0;

	buf = log->log_buf + log->w_pos;

	si_meminfo(&i);
	si_swapinfo(&i);
	committed = percpu_counter_read_positive(&vm_committed_as);

	cached = global_node_page_state(NR_FILE_PAGES) -
			total_swapcache_pages() - i.bufferram;
	if (cached < 0)
		cached = 0;

	for (lru = LRU_BASE; lru < NR_LRU_LISTS; lru++)
		pages[lru] = global_node_page_state(NR_LRU_BASE + lru);

	available = si_mem_available();

	left = all_left = log->left;

	sreclaimable = global_node_page_state(NR_SLAB_RECLAIMABLE);
	sunreclaim = global_node_page_state(NR_SLAB_UNRECLAIMABLE);

	SHOW_MEM_VALUE(buf, info_len, "MemTotal:       ", i.totalram, left);
	SHOW_MEM_VALUE(buf, info_len, "MemFree:        ", i.freeram, left);
	SHOW_MEM_VALUE(buf, info_len, "MemAvailable:   ", available, left);
	SHOW_MEM_VALUE(buf, info_len, "Buffers:        ", i.bufferram, left);
	SHOW_MEM_VALUE(buf, info_len, "Cached:         ", cached, left);
	SHOW_MEM_VALUE(buf, info_len, "SwapCached:     ", total_swapcache_pages(), left);
	SHOW_MEM_VALUE(buf, info_len, "Active:         ", pages[LRU_ACTIVE_ANON] +
					   pages[LRU_ACTIVE_FILE], left);
	SHOW_MEM_VALUE(buf, info_len, "Inactive:       ", pages[LRU_INACTIVE_ANON] +
					   pages[LRU_INACTIVE_FILE], left);
	SHOW_MEM_VALUE(buf, info_len, "Active(anon):   ", pages[LRU_ACTIVE_ANON], left);
	SHOW_MEM_VALUE(buf, info_len, "Inactive(anon): ", pages[LRU_INACTIVE_ANON], left);
	SHOW_MEM_VALUE(buf, info_len, "Active(file):   ", pages[LRU_ACTIVE_FILE], left);
	SHOW_MEM_VALUE(buf, info_len, "Inactive(file): ", pages[LRU_INACTIVE_FILE], left);
	SHOW_MEM_VALUE(buf, info_len, "Unevictable:    ", pages[LRU_UNEVICTABLE], left);
	SHOW_MEM_VALUE(buf, info_len, "Mlocked:        ", global_zone_page_state(NR_MLOCK), left);

#ifdef CONFIG_HIGHMEM
	SHOW_MEM_VALUE(buf, info_len, "HighTotal:      ", i.totalhigh, left);
	SHOW_MEM_VALUE(buf, info_len, "HighFree:       ", i.freehigh, left);
	SHOW_MEM_VALUE(buf, info_len, "LowTotal:       ", i.totalram - i.totalhigh, left);
	SHOW_MEM_VALUE(buf, info_len, "LowFree:        ", i.freeram - i.freehigh, left);
#endif

#ifndef CONFIG_MMU
	SHOW_MEM_VALUE(buf, info_len, "MmapCopy:       ",
		    (unsigned long)atomic_long_read(&mmap_pages_allocated), left);
#endif

	SHOW_MEM_VALUE(buf, info_len, "SwapTotal:      ", i.totalswap, left);
	SHOW_MEM_VALUE(buf, info_len, "SwapFree:       ", i.freeswap, left);
	SHOW_MEM_VALUE(buf, info_len, "Dirty:          ",
		    global_node_page_state(NR_FILE_DIRTY), left);
	SHOW_MEM_VALUE(buf, info_len, "Writeback:      ",
		    global_node_page_state(NR_WRITEBACK), left);
	SHOW_MEM_VALUE(buf, info_len, "AnonPages:      ",
		    global_node_page_state(NR_ANON_MAPPED), left);
	SHOW_MEM_VALUE(buf, info_len, "Mapped:         ",
		    global_node_page_state(NR_FILE_MAPPED), left);
	SHOW_MEM_VALUE(buf, info_len, "Shmem:          ", i.sharedram, left);
	SHOW_MEM_VALUE(buf, info_len, "KReclaimable:   ", sreclaimable +
		    global_node_page_state(NR_KERNEL_MISC_RECLAIMABLE), left);
	SHOW_MEM_VALUE(buf, info_len, "Slab:           ", sreclaimable + sunreclaim, left);
	SHOW_MEM_VALUE(buf, info_len, "SReclaimable:   ", sreclaimable, left);
	SHOW_MEM_VALUE(buf, info_len, "SUnreclaim:     ", sunreclaim, left);

	info_len += snprintf(buf + info_len, left, "KernelStack:    %8lu kB\n",
		   global_zone_page_state(NR_KERNEL_STACK_KB));
	left = all_left - info_len;
#ifdef CONFIG_SHADOW_CALL_STACK
	info_len += snprintf(buf + info_len, left, "ShadowCallStack:%8lu kB\n",
		   global_zone_page_state(NR_KERNEL_SCS_BYTES) / 1024);
	left = all_left - info_len;
#endif

	SHOW_MEM_VALUE(buf, info_len, "PageTables:     ",
		    global_zone_page_state(NR_PAGETABLE), left);

	SHOW_MEM_VALUE(buf, info_len, "NFS_Unstable:   ",
		    global_node_page_state(NR_UNSTABLE_NFS), left);
	SHOW_MEM_VALUE(buf, info_len, "Bounce:         ",
		    global_zone_page_state(NR_BOUNCE), left);
	SHOW_MEM_VALUE(buf, info_len, "WritebackTmp:   ",
		    global_node_page_state(NR_WRITEBACK_TEMP), left);
	SHOW_MEM_VALUE(buf, info_len, "CommitLimit:    ", vm_commit_limit(), left);
	SHOW_MEM_VALUE(buf, info_len, "Committed_AS:   ", committed, left);

	info_len += snprintf(buf + info_len, left, "VmallocTotal:   %8lu kB\n",
		   (unsigned long)VMALLOC_TOTAL >> 10);
	left = all_left - info_len;

	SHOW_MEM_VALUE(buf, info_len, "VmallocUsed:    ", vmalloc_nr_pages(), left);
	SHOW_MEM_VALUE(buf, info_len, "VmallocChunk:   ", 0ul, left);
	SHOW_MEM_VALUE(buf, info_len, "Percpu:         ", pcpu_nr_pages(), left);

#ifdef CONFIG_MEMORY_FAILURE
	info_len += snprintf(buf + info_len, left, "HardwareCorrupted: %5lu kB\n",
		   atomic_long_read(&num_poisoned_pages) << (PAGE_SHIFT - 10));
	left = all_left - info_len;
#endif

#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	SHOW_MEM_VALUE(buf, info_len, "AnonHugePages:  ",
		    global_node_page_state(NR_ANON_THPS) * HPAGE_PMD_NR, left);
	SHOW_MEM_VALUE(buf, info_len, "ShmemHugePages: ",
		    global_node_page_state(NR_SHMEM_THPS) * HPAGE_PMD_NR, left);
	SHOW_MEM_VALUE(buf, info_len, "ShmemPmdMapped: ",
		    global_node_page_state(NR_SHMEM_PMDMAPPED) * HPAGE_PMD_NR, left);
	SHOW_MEM_VALUE(buf, info_len, "FileHugePages:  ",
		    global_node_page_state(NR_FILE_THPS) * HPAGE_PMD_NR, left);
	SHOW_MEM_VALUE(buf, info_len, "FilePmdMapped:  ",
		    global_node_page_state(NR_FILE_PMDMAPPED) * HPAGE_PMD_NR, left);
#endif

#ifdef CONFIG_CMA
	SHOW_MEM_VALUE(buf, info_len, "CmaTotal:       ", totalcma_pages, left);
	SHOW_MEM_VALUE(buf, info_len, "CmaFree:        ",
		    global_zone_page_state(NR_FREE_CMA_PAGES), left);
#endif

	return info_len;
}

int rs_dump_meminfo(struct rs_recorder_log *log, char *fname)
{
	char *buff;
	u32 data_size = 0;

	buff = log->log_buf + log->w_pos;
	data_size = rs_get_meminfo(log);

	if (!log->is_panic)
		rs_save_file(log->path, fname, buff, data_size);

	rs_update_buf_header(log, fname, data_size);

	return 0;
}
