/*
 * Add configfs and memory store: Kyungchan Koh <kkc6196@fb.com> and
 * Shaohua Li <shli@fb.com>
 */
#include <linux/module.h>

#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/blk-mq.h>
#include <linux/hrtimer.h>
#include <linux/lightnvm.h>
#include <linux/configfs.h>
#include <linux/badblocks.h>

#include "sd_err_check.h"
#include "../../block/blk.h"
struct gendisk *virtual_T_disk;

#define SECTOR_SHIFT		9
#define PAGE_SECTORS_SHIFT	(PAGE_SHIFT - SECTOR_SHIFT)
#define PAGE_SECTORS		(1 << PAGE_SECTORS_SHIFT)
#define SECTOR_SIZE		(1 << SECTOR_SHIFT)
#define SECTOR_MASK		(PAGE_SECTORS - 1)

#define FREE_BATCH		16

#define TICKS_PER_SEC		50ULL
#define TIMER_INTERVAL		(NSEC_PER_SEC / TICKS_PER_SEC)

static inline u64 mb_per_tick(int mbps)
{
	return (1 << 20) / TICKS_PER_SEC * ((u64) mbps);
}

struct virtualb_cmd {
	struct list_head list;
	struct llist_node ll_list;
	call_single_data_t csd;
	struct request *rq;
	struct bio *bio;
	unsigned int tag;
	struct virtualb_queue *nq;
	struct hrtimer timer;
	blk_status_t error;
};

struct virtualb_queue {
	unsigned long *tag_map;
	wait_queue_head_t wait;
	unsigned int queue_depth;
	struct virtualb_device *dev;
	unsigned int requeue_selection;

	struct virtualb_cmd *cmds;
};

/*
 * Status flags for virtualb_device.
 *
 * CONFIGURED:	Device has been configured and turned on. Cannot reconfigure.
 * UP:		Device is currently on and visible in userspace.
 * THROTTLED:	Device is being throttled.
 * CACHE:	Device is using a write-back cache.
 */
enum virtualb_device_flags {
	VIRTUALB_DEV_FL_CONFIGURED	= 0,
	VIRTUALB_DEV_FL_UP		= 1,
	VIRTUALB_DEV_FL_THROTTLED	= 2,
	VIRTUALB_DEV_FL_CACHE	= 3,
};

#define MAP_SZ		((PAGE_SIZE >> SECTOR_SHIFT) + 2)
/*
 * virtualb_page is a page in memory for virtualb devices.
 *
 * @page:	The page holding the data.
 * @bitmap:	The bitmap represents which sector in the page has data.
 *		Each bit represents one block size. For example, sector 8
 *		will use the 7th bit
 * The highest 2 bits of bitmap are for special purpose. LOCK means the cache
 * page is being flushing to storage. FREE means the cache page is freed and
 * should be skipped from flushing to storage. Please see
 * virtual_make_cache_space
 */
struct virtualb_page {
	struct page *page;
	DECLARE_BITMAP(bitmap, MAP_SZ);
};
#define VIRTUALB_PAGE_LOCK (MAP_SZ - 1)
#define VIRTUALB_PAGE_FREE (MAP_SZ - 2)

struct virtualb_device {
	struct virtualb *virtualb;
	struct config_item item;
	struct radix_tree_root data; /* data stored in the disk */
	struct radix_tree_root cache; /* disk cache data */
	unsigned long flags; /* device flags */
	unsigned int curr_cache;
	struct badblocks badblocks;

	unsigned long size; /* device size in MB */
	unsigned long completion_nsec; /* time in ns to complete a request */
	unsigned long cache_size; /* disk cache size in MB */
	unsigned int submit_queues; /* number of submission queues */
	unsigned int home_node; /* home node for the device */
	unsigned int queue_mode; /* block interface */
	unsigned int blocksize; /* block size */
	unsigned int irqmode; /* IRQ completion handler */
	unsigned int hw_queue_depth; /* queue depth */
	unsigned int index; /* index of the disk, only valid with a disk */
	unsigned int mbps; /* Bandwidth throttle cap (in MB/s) */
	bool use_lightnvm; /* register as a LightNVM device */
	bool blocking; /* blocking blk-mq device */
	bool use_per_node_hctx; /* use per-node allocation for hardware context */
	bool power; /* power on/off the device */
	bool memory_backed; /* if data is stored in memory */
	bool discard; /* if support discard */
};

struct virtualb {
	struct virtualb_device *dev;
	struct list_head list;
	unsigned int index;
	struct request_queue *q;
	struct gendisk *disk;
	struct nvm_dev *ndev;
	struct blk_mq_tag_set *tag_set;
	struct blk_mq_tag_set __tag_set;
	unsigned int queue_depth;
	atomic_long_t cur_bytes;
	struct hrtimer bw_timer;
	unsigned long cache_flush_pos;
	spinlock_t lock;

	struct virtualb_queue *queues;
	unsigned int nr_queues;
	char disk_name[DISK_NAME_LEN];
};

static LIST_HEAD(virtualb_list);
static struct mutex lock;
static int virtual_major;
static DEFINE_IDA(virtualb_indexes);
static struct kmem_cache *ppa_cache;
static struct blk_mq_tag_set tag_set;

enum {
	VIRTUAL_IRQ_NONE		= 0,
	VIRTUAL_IRQ_SOFTIRQ	= 1,
	VIRTUAL_IRQ_TIMER		= 2,
};

enum {
	VIRTUAL_Q_BIO		= 0,
	VIRTUAL_Q_RQ		= 1,
	VIRTUAL_Q_MQ		= 2,
};

static int g_submit_queues = 1;
module_param_named(submit_queues, g_submit_queues, int, S_IRUGO);
MODULE_PARM_DESC(submit_queues, "Number of submission queues");

static int g_home_node = NUMA_NO_NODE;
module_param_named(home_node, g_home_node, int, S_IRUGO);
MODULE_PARM_DESC(home_node, "Home node for the device");

static int g_queue_mode = VIRTUAL_Q_MQ;

static int virtual_param_store_val(const char *str, int *val, int min, int max)
{
	int ret, new_val;

	ret = kstrtoint(str, 10, &new_val);
	if (ret)
		return -EINVAL;

	if (new_val < min || new_val > max)
		return -EINVAL;

	*val = new_val;
	return 0;
}

static int virtual_set_queue_mode(const char *str, const struct kernel_param *kp)
{
	return virtual_param_store_val(str, &g_queue_mode, VIRTUAL_Q_BIO, VIRTUAL_Q_MQ);
}

static const struct kernel_param_ops virtual_queue_mode_param_ops = {
	.set	= virtual_set_queue_mode,
	.get	= param_get_int,
};

device_param_cb(queue_mode, &virtual_queue_mode_param_ops, &g_queue_mode, S_IRUGO);
MODULE_PARM_DESC(queue_mode, "Block interface to use (0=bio,1=rq,2=multiqueue)");

static int g_gb = 250;
module_param_named(gb, g_gb, int, S_IRUGO);
MODULE_PARM_DESC(gb, "Size in GB");

static int g_bs = 512;
module_param_named(bs, g_bs, int, S_IRUGO);
MODULE_PARM_DESC(bs, "Block size (in bytes)");

static int nr_devices = 1;
module_param(nr_devices, int, S_IRUGO);
MODULE_PARM_DESC(nr_devices, "Number of devices to register");

static bool g_use_lightnvm;
module_param_named(use_lightnvm, g_use_lightnvm, bool, S_IRUGO);
MODULE_PARM_DESC(use_lightnvm, "Register as a LightNVM device");

static bool g_blocking;
module_param_named(blocking, g_blocking, bool, S_IRUGO);
MODULE_PARM_DESC(blocking, "Register as a blocking blk-mq driver device");

static bool shared_tags;
module_param(shared_tags, bool, S_IRUGO);
MODULE_PARM_DESC(shared_tags, "Share tag set between devices for blk-mq");

static int g_irqmode = VIRTUAL_IRQ_SOFTIRQ;

static int virtual_set_irqmode(const char *str, const struct kernel_param *kp)
{
	return virtual_param_store_val(str, &g_irqmode, VIRTUAL_IRQ_NONE,
					VIRTUAL_IRQ_TIMER);
}

static const struct kernel_param_ops virtual_irqmode_param_ops = {
	.set	= virtual_set_irqmode,
	.get	= param_get_int,
};

device_param_cb(irqmode, &virtual_irqmode_param_ops, &g_irqmode, S_IRUGO);
MODULE_PARM_DESC(irqmode, "IRQ completion handler. 0-none, 1-softirq, 2-timer");

static unsigned long g_completion_nsec = 10000;
module_param_named(completion_nsec, g_completion_nsec, ulong, S_IRUGO);
MODULE_PARM_DESC(completion_nsec, "Time in ns to complete a request in hardware. Default: 10,000ns");

static int g_hw_queue_depth = 64;
module_param_named(hw_queue_depth, g_hw_queue_depth, int, S_IRUGO);
MODULE_PARM_DESC(hw_queue_depth, "Queue depth for each hardware queue. Default: 64");

static bool g_use_per_node_hctx;
module_param_named(use_per_node_hctx, g_use_per_node_hctx, bool, S_IRUGO);
MODULE_PARM_DESC(use_per_node_hctx, "Use per-node allocation for hardware context queues. Default: false");

static struct virtualb_device *virtual_alloc_dev(void);
static void virtual_free_dev(struct virtualb_device *dev);
static void virtual_del_dev(struct virtualb *virtualb);
static int virtual_add_dev(struct virtualb_device *dev);
static void virtual_free_device_storage(struct virtualb_device *dev, bool is_cache);

static inline struct virtualb_device *to_virtualb_device(struct config_item *item)
{
	return item ? container_of(item, struct virtualb_device, item) : NULL;
}

static inline ssize_t virtualb_device_uint_attr_show(unsigned int val, char *page)
{
	return snprintf(page, PAGE_SIZE, "%u\n", val);
}

static inline ssize_t virtualb_device_ulong_attr_show(unsigned long val,
	char *page)
{
	return snprintf(page, PAGE_SIZE, "%lu\n", val);
}

static inline ssize_t virtualb_device_bool_attr_show(bool val, char *page)
{
	return snprintf(page, PAGE_SIZE, "%u\n", val);
}

static ssize_t virtualb_device_uint_attr_store(unsigned int *val,
	const char *page, size_t count)
{
	unsigned int tmp;
	int result;

	result = kstrtouint(page, 0, &tmp);
	if (result)
		return result;

	*val = tmp;
	return count;
}

static ssize_t virtualb_device_ulong_attr_store(unsigned long *val,
	const char *page, size_t count)
{
	int result;
	unsigned long tmp;

	result = kstrtoul(page, 0, &tmp);
	if (result)
		return result;

	*val = tmp;
	return count;
}

static ssize_t virtualb_device_bool_attr_store(bool *val, const char *page,
	size_t count)
{
	bool tmp;
	int result;

	result = kstrtobool(page,  &tmp);
	if (result)
		return result;

	*val = tmp;
	return count;
}

/* The following macro should only be used with TYPE = {uint, ulong, bool}. */
#define VIRTUALB_DEVICE_ATTR(NAME, TYPE)						\
static ssize_t									\
virtualb_device_##NAME##_show(struct config_item *item, char *page)		\
{										\
	return virtualb_device_##TYPE##_attr_show(					\
				to_virtualb_device(item)->NAME, page);		\
}										\
static ssize_t									\
virtualb_device_##NAME##_store(struct config_item *item, const char *page,		\
			    size_t count)					\
{										\
	if (test_bit(VIRTUALB_DEV_FL_CONFIGURED, &to_virtualb_device(item)->flags))	\
		return -EBUSY;							\
	return virtualb_device_##TYPE##_attr_store(				\
			&to_virtualb_device(item)->NAME, page, count);		\
}										\
CONFIGFS_ATTR(virtualb_device_, NAME);

VIRTUALB_DEVICE_ATTR(size, ulong);
VIRTUALB_DEVICE_ATTR(completion_nsec, ulong);
VIRTUALB_DEVICE_ATTR(submit_queues, uint);
VIRTUALB_DEVICE_ATTR(home_node, uint);
VIRTUALB_DEVICE_ATTR(queue_mode, uint);
VIRTUALB_DEVICE_ATTR(blocksize, uint);
VIRTUALB_DEVICE_ATTR(irqmode, uint);
VIRTUALB_DEVICE_ATTR(hw_queue_depth, uint);
VIRTUALB_DEVICE_ATTR(index, uint);
VIRTUALB_DEVICE_ATTR(use_lightnvm, bool);
VIRTUALB_DEVICE_ATTR(blocking, bool);
VIRTUALB_DEVICE_ATTR(use_per_node_hctx, bool);
VIRTUALB_DEVICE_ATTR(memory_backed, bool);
VIRTUALB_DEVICE_ATTR(discard, bool);
VIRTUALB_DEVICE_ATTR(mbps, uint);
VIRTUALB_DEVICE_ATTR(cache_size, ulong);

static ssize_t virtualb_device_power_show(struct config_item *item, char *page)
{
	return virtualb_device_bool_attr_show(to_virtualb_device(item)->power, page);
}

static ssize_t virtualb_device_power_store(struct config_item *item,
				     const char *page, size_t count)
{
	struct virtualb_device *dev = to_virtualb_device(item);
	bool newp = false;
	ssize_t ret;

	ret = virtualb_device_bool_attr_store(&newp, page, count);
	if (ret < 0)
		return ret;

	if (!dev->power && newp) {
		if (test_and_set_bit(VIRTUALB_DEV_FL_UP, &dev->flags))
			return count;
		if (virtual_add_dev(dev)) {
			clear_bit(VIRTUALB_DEV_FL_UP, &dev->flags);
			return -ENOMEM;
		}

		set_bit(VIRTUALB_DEV_FL_CONFIGURED, &dev->flags);
		dev->power = newp;
	} else if (dev->power && !newp) {
		mutex_lock(&lock);
		dev->power = newp;
		virtual_del_dev(dev->virtualb);
		mutex_unlock(&lock);
		clear_bit(VIRTUALB_DEV_FL_UP, &dev->flags);
	}

	return count;
}

CONFIGFS_ATTR(virtualb_device_, power);

static ssize_t virtualb_device_badblocks_show(struct config_item *item, char *page)
{
	struct virtualb_device *t_dev = to_virtualb_device(item);

	return badblocks_show(&t_dev->badblocks, page, 0);
}

static ssize_t virtualb_device_badblocks_store(struct config_item *item,
				     const char *page, size_t count)
{
	struct virtualb_device *t_dev = to_virtualb_device(item);
	char *orig, *buf, *tmp;
	u64 start, end;
	int ret;

	orig = kstrndup(page, count, GFP_KERNEL);
	if (!orig)
		return -ENOMEM;

	buf = strstrip(orig);

	ret = -EINVAL;
	if (buf[0] != '+' && buf[0] != '-')
		goto out;
	tmp = strchr(&buf[1], '-');
	if (!tmp)
		goto out;
	*tmp = '\0';
	ret = kstrtoull(buf + 1, 0, &start);
	if (ret)
		goto out;
	ret = kstrtoull(tmp + 1, 0, &end);
	if (ret)
		goto out;
	ret = -EINVAL;
	if (start > end)
		goto out;
	/* enable badblocks */
	cmpxchg(&t_dev->badblocks.shift, -1, 0);
	if (buf[0] == '+')
		ret = badblocks_set(&t_dev->badblocks, start,
			end - start + 1, 1);
	else
		ret = badblocks_clear(&t_dev->badblocks, start,
			end - start + 1);
	if (ret == 0)
		ret = count;
out:
	kfree(orig);
	return ret;
}
CONFIGFS_ATTR(virtualb_device_, badblocks);

static struct configfs_attribute *virtualb_device_attrs[] = {
	&virtualb_device_attr_size,
	&virtualb_device_attr_completion_nsec,
	&virtualb_device_attr_submit_queues,
	&virtualb_device_attr_home_node,
	&virtualb_device_attr_queue_mode,
	&virtualb_device_attr_blocksize,
	&virtualb_device_attr_irqmode,
	&virtualb_device_attr_hw_queue_depth,
	&virtualb_device_attr_index,
	&virtualb_device_attr_use_lightnvm,
	&virtualb_device_attr_blocking,
	&virtualb_device_attr_use_per_node_hctx,
	&virtualb_device_attr_power,
	&virtualb_device_attr_memory_backed,
	&virtualb_device_attr_discard,
	&virtualb_device_attr_mbps,
	&virtualb_device_attr_cache_size,
	&virtualb_device_attr_badblocks,
	NULL,
};

static void virtualb_device_release(struct config_item *item)
{
	struct virtualb_device *dev = to_virtualb_device(item);

	virtual_free_device_storage(dev, false);
	virtual_free_dev(dev);
}

static struct configfs_item_operations virtualb_device_ops = {
	.release	= virtualb_device_release,
};

static struct config_item_type virtualb_device_type = {
	.ct_item_ops	= &virtualb_device_ops,
	.ct_attrs	= virtualb_device_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct
config_item *virtualb_group_make_item(struct config_group *group, const char *name)
{
	struct virtualb_device *dev;

	dev = virtual_alloc_dev();
	if (!dev)
		return ERR_PTR(-ENOMEM);

	config_item_init_type_name(&dev->item, name, &virtualb_device_type);

	return &dev->item;
}

static void
virtualb_group_drop_item(struct config_group *group, struct config_item *item)
{
	struct virtualb_device *dev = to_virtualb_device(item);

	if (test_and_clear_bit(VIRTUALB_DEV_FL_UP, &dev->flags)) {
		mutex_lock(&lock);
		dev->power = false;
		virtual_del_dev(dev->virtualb);
		mutex_unlock(&lock);
	}

	config_item_put(item);
}

static ssize_t memb_group_features_show(struct config_item *item, char *page)
{
	return snprintf(page, PAGE_SIZE, "memory_backed,discard,bandwidth,cache,badblocks\n");
}

CONFIGFS_ATTR_RO(memb_group_, features);

static struct configfs_attribute *virtualb_group_attrs[] = {
	&memb_group_attr_features,
	NULL,
};

static struct configfs_group_operations virtualb_group_ops = {
	.make_item	= virtualb_group_make_item,
	.drop_item	= virtualb_group_drop_item,
};

static struct config_item_type virtualb_group_type = {
	.ct_group_ops	= &virtualb_group_ops,
	.ct_attrs	= virtualb_group_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct configfs_subsystem virtualb_subsys = {
	.su_group = {
		.cg_item = {
			.ci_namebuf = "virtualb",
			.ci_type = &virtualb_group_type,
		},
	},
};

static inline int virtual_cache_active(struct virtualb *virtualb)
{
	return test_bit(VIRTUALB_DEV_FL_CACHE, &virtualb->dev->flags);
}

static struct virtualb_device *virtual_alloc_dev(void)
{
	struct virtualb_device *dev;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return NULL;
	INIT_RADIX_TREE(&dev->data, GFP_ATOMIC);
	INIT_RADIX_TREE(&dev->cache, GFP_ATOMIC);
	if (badblocks_init(&dev->badblocks, 0)) {
		kfree(dev);
		return NULL;
	}

	dev->size = g_gb * 1024;
	dev->completion_nsec = g_completion_nsec;
	dev->submit_queues = g_submit_queues;
	dev->home_node = g_home_node;
	dev->queue_mode = g_queue_mode;
	dev->blocksize = g_bs;
	dev->irqmode = g_irqmode;
	dev->hw_queue_depth = g_hw_queue_depth;
	dev->use_lightnvm = g_use_lightnvm;
	dev->blocking = g_blocking;
	dev->use_per_node_hctx = g_use_per_node_hctx;
	return dev;
}

static void virtual_free_dev(struct virtualb_device *dev)
{
	if (!dev)
		return;

	badblocks_exit(&dev->badblocks);
	kfree(dev);
}

static void put_tag(struct virtualb_queue *nq, unsigned int tag)
{
	clear_bit_unlock(tag, nq->tag_map);

	if (waitqueue_active(&nq->wait))
		wake_up(&nq->wait);
}

static unsigned int get_tag(struct virtualb_queue *nq)
{
	unsigned int tag;

	do {
		tag = find_first_zero_bit(nq->tag_map, nq->queue_depth);
		if (tag >= nq->queue_depth)
			return -1U;
	} while (test_and_set_bit_lock(tag, nq->tag_map));

	return tag;
}

static void free_cmd(struct virtualb_cmd *cmd)
{
	put_tag(cmd->nq, cmd->tag);
}

static enum hrtimer_restart virtual_cmd_timer_expired(struct hrtimer *timer);

static struct virtualb_cmd *__alloc_cmd(struct virtualb_queue *nq)
{
	struct virtualb_cmd *cmd;
	unsigned int tag;

	tag = get_tag(nq);
	if (tag != -1U) {
		cmd = &nq->cmds[tag];
		cmd->tag = tag;
		cmd->nq = nq;
		if (nq->dev->irqmode == VIRTUAL_IRQ_TIMER) {
			hrtimer_init(&cmd->timer, CLOCK_MONOTONIC,
				     HRTIMER_MODE_REL);
			cmd->timer.function = virtual_cmd_timer_expired;
		}
		return cmd;
	}

	return NULL;
}

static struct virtualb_cmd *alloc_cmd(struct virtualb_queue *nq, int can_wait)
{
	struct virtualb_cmd *cmd;
	DEFINE_WAIT(wait);

	cmd = __alloc_cmd(nq);
	if (cmd || !can_wait)
		return cmd;

	do {
		prepare_to_wait(&nq->wait, &wait, TASK_UNINTERRUPTIBLE);
		cmd = __alloc_cmd(nq);
		if (cmd)
			break;

		io_schedule();
	} while (1);

	finish_wait(&nq->wait, &wait);
	return cmd;
}

static void end_cmd(struct virtualb_cmd *cmd)
{
	struct request_queue *q = NULL;
	int queue_mode = cmd->nq->dev->queue_mode;

	if (cmd->rq)
		q = cmd->rq->q;

	switch (queue_mode)  {
	case VIRTUAL_Q_MQ:
		blk_mq_end_request(cmd->rq, cmd->error);
		return;
	case VIRTUAL_Q_BIO:
		cmd->bio->bi_status = cmd->error;
		bio_endio(cmd->bio);
		break;
	}

	free_cmd(cmd);
}

static enum hrtimer_restart virtual_cmd_timer_expired(struct hrtimer *timer)
{
	end_cmd(container_of(timer, struct virtualb_cmd, timer));

	return HRTIMER_NORESTART;
}

static void virtual_cmd_end_timer(struct virtualb_cmd *cmd)
{
	ktime_t kt = cmd->nq->dev->completion_nsec;

	hrtimer_start(&cmd->timer, kt, HRTIMER_MODE_REL);
}

static void virtual_softirq_done_fn(struct request *rq)
{
	end_cmd(blk_mq_rq_to_pdu(rq));
}

static struct virtualb_page *virtual_alloc_page(gfp_t gfp_flags)
{
	struct virtualb_page *t_page;

	t_page = kmalloc(sizeof(struct virtualb_page), gfp_flags);
	if (!t_page)
		goto out;

	t_page->page = alloc_pages(gfp_flags, 0);
	if (!t_page->page)
		goto out_freepage;

	memset(t_page->bitmap, 0, sizeof(t_page->bitmap));
	return t_page;
out_freepage:
	kfree(t_page);
out:
	return NULL;
}

static void virtual_free_page(struct virtualb_page *t_page)
{
	__set_bit(VIRTUALB_PAGE_FREE, t_page->bitmap);
	if (test_bit(VIRTUALB_PAGE_LOCK, t_page->bitmap))
		return;
	__free_page(t_page->page);
	kfree(t_page);
}

static bool virtual_page_empty(struct virtualb_page *page)
{
	int size = MAP_SZ - 2;

	return find_first_bit(page->bitmap, size) == size;
}

static void virtual_free_sector(struct virtualb *virtualb, sector_t sector,
	bool is_cache)
{
	unsigned int sector_bit;
	u64 idx;
	struct virtualb_page *t_page, *ret;
	struct radix_tree_root *root;

	root = is_cache ? &virtualb->dev->cache : &virtualb->dev->data;
	idx = sector >> PAGE_SECTORS_SHIFT;
	sector_bit = (sector & SECTOR_MASK);

	t_page = radix_tree_lookup(root, idx);
	if (t_page) {
		__clear_bit(sector_bit, t_page->bitmap);

		if (virtual_page_empty(t_page)) {
			ret = radix_tree_delete_item(root, idx, t_page);
			WARN_ON(ret != t_page);
			virtual_free_page(ret);
			if (is_cache)
				virtualb->dev->curr_cache -= PAGE_SIZE;
		}
	}
}

static struct virtualb_page *virtual_radix_tree_insert(struct virtualb *virtualb, u64 idx,
	struct virtualb_page *t_page, bool is_cache)
{
	struct radix_tree_root *root;

	root = is_cache ? &virtualb->dev->cache : &virtualb->dev->data;

	if (radix_tree_insert(root, idx, t_page)) {
		virtual_free_page(t_page);
		t_page = radix_tree_lookup(root, idx);
		WARN_ON(!t_page || t_page->page->index != idx);
	} else if (is_cache)
		virtualb->dev->curr_cache += PAGE_SIZE;

	return t_page;
}

static void virtual_free_device_storage(struct virtualb_device *dev, bool is_cache)
{
	unsigned long pos = 0;
	int nr_pages;
	struct virtualb_page *ret, *t_pages[FREE_BATCH];
	struct radix_tree_root *root;

	root = is_cache ? &dev->cache : &dev->data;

	do {
		int i;

		nr_pages = radix_tree_gang_lookup(root,
				(void **)t_pages, pos, FREE_BATCH);

		for (i = 0; i < nr_pages; i++) {
			pos = t_pages[i]->page->index;
			ret = radix_tree_delete_item(root, pos, t_pages[i]);
			WARN_ON(ret != t_pages[i]);
			virtual_free_page(ret);
		}

		pos++;
	} while (nr_pages == FREE_BATCH);

	if (is_cache)
		dev->curr_cache = 0;
}

static struct virtualb_page *__virtual_lookup_page(struct virtualb *virtualb,
	sector_t sector, bool for_write, bool is_cache)
{
	unsigned int sector_bit;
	u64 idx;
	struct virtualb_page *t_page;
	struct radix_tree_root *root;

	idx = sector >> PAGE_SECTORS_SHIFT;
	sector_bit = (sector & SECTOR_MASK);

	root = is_cache ? &virtualb->dev->cache : &virtualb->dev->data;
	t_page = radix_tree_lookup(root, idx);
	WARN_ON(t_page && t_page->page->index != idx);

	if (t_page && (for_write || test_bit(sector_bit, t_page->bitmap)))
		return t_page;

	return NULL;
}

static struct virtualb_page *virtual_lookup_page(struct virtualb *virtualb,
	sector_t sector, bool for_write, bool ignore_cache)
{
	struct virtualb_page *page = NULL;

	if (!ignore_cache)
		page = __virtual_lookup_page(virtualb, sector, for_write, true);
	if (page)
		return page;
	return __virtual_lookup_page(virtualb, sector, for_write, false);
}

static struct virtualb_page *virtual_insert_page(struct virtualb *virtualb,
	sector_t sector, bool ignore_cache)
{
	u64 idx;
	struct virtualb_page *t_page;

	t_page = virtual_lookup_page(virtualb, sector, true, ignore_cache);
	if (t_page)
		return t_page;

	spin_unlock_irq(&virtualb->lock);

	t_page = virtual_alloc_page(GFP_NOIO);
	if (!t_page)
		goto out_lock;

	if (radix_tree_preload(GFP_NOIO))
		goto out_freepage;

	spin_lock_irq(&virtualb->lock);
	idx = sector >> PAGE_SECTORS_SHIFT;
	t_page->page->index = idx;
	t_page = virtual_radix_tree_insert(virtualb, idx, t_page, !ignore_cache);
	radix_tree_preload_end();

	return t_page;
out_freepage:
	virtual_free_page(t_page);
out_lock:
	spin_lock_irq(&virtualb->lock);
	return virtual_lookup_page(virtualb, sector, true, ignore_cache);
}

static int virtual_flush_cache_page(struct virtualb *virtualb, struct virtualb_page *c_page)
{
	int i;
	unsigned int offset;
	u64 idx;
	struct virtualb_page *t_page, *ret;
	void *dst, *src;

	idx = c_page->page->index;

	t_page = virtual_insert_page(virtualb, idx << PAGE_SECTORS_SHIFT, true);

	__clear_bit(VIRTUALB_PAGE_LOCK, c_page->bitmap);
	if (test_bit(VIRTUALB_PAGE_FREE, c_page->bitmap)) {
		virtual_free_page(c_page);
		if (t_page && virtual_page_empty(t_page)) {
			ret = radix_tree_delete_item(&virtualb->dev->data,
				idx, t_page);
			virtual_free_page(t_page);
		}
		return 0;
	}

	if (!t_page)
		return -ENOMEM;

	src = kmap_atomic(c_page->page);
	dst = kmap_atomic(t_page->page);

	for (i = 0; i < PAGE_SECTORS;
			i += (virtualb->dev->blocksize >> SECTOR_SHIFT)) {
		if (test_bit(i, c_page->bitmap)) {
			offset = (i << SECTOR_SHIFT);
			memcpy(dst + offset, src + offset,
				virtualb->dev->blocksize);
			__set_bit(i, t_page->bitmap);
		}
	}

	kunmap_atomic(dst);
	kunmap_atomic(src);

	ret = radix_tree_delete_item(&virtualb->dev->cache, idx, c_page);
	virtual_free_page(ret);
	virtualb->dev->curr_cache -= PAGE_SIZE;

	return 0;
}

static int virtual_make_cache_space(struct virtualb *virtualb, unsigned long n)
{
	int i, err, nr_pages;
	struct virtualb_page *c_pages[FREE_BATCH];
	unsigned long flushed = 0, one_round;

again:
	if ((virtualb->dev->cache_size * 1024 * 1024) >
	     virtualb->dev->curr_cache + n || virtualb->dev->curr_cache == 0)
		return 0;

	nr_pages = radix_tree_gang_lookup(&virtualb->dev->cache,
			(void **)c_pages, virtualb->cache_flush_pos, FREE_BATCH);
	/*
	 * virtualb_flush_cache_page could unlock before using the c_pages. To
	 * avoid race, we don't allow page free
	 */
	for (i = 0; i < nr_pages; i++) {
		virtualb->cache_flush_pos = c_pages[i]->page->index;
		/*
		 * We found the page which is being flushed to disk by other
		 * threads
		 */
		if (test_bit(VIRTUALB_PAGE_LOCK, c_pages[i]->bitmap))
			c_pages[i] = NULL;
		else
			__set_bit(VIRTUALB_PAGE_LOCK, c_pages[i]->bitmap);
	}

	one_round = 0;
	for (i = 0; i < nr_pages; i++) {
		if (c_pages[i] == NULL)
			continue;
		err = virtual_flush_cache_page(virtualb, c_pages[i]);
		if (err)
			return err;
		one_round++;
	}
	flushed += one_round << PAGE_SHIFT;

	if (n > flushed) {
		if (nr_pages == 0)
			virtualb->cache_flush_pos = 0;
		if (one_round == 0) {
			/* give other threads a chance */
			spin_unlock_irq(&virtualb->lock);
			spin_lock_irq(&virtualb->lock);
		}
		goto again;
	}
	return 0;
}

static int copy_to_virtualb(struct virtualb *virtualb, struct page *source,
	unsigned int off, sector_t sector, size_t n, bool is_fua)
{
	size_t temp, count = 0;
	unsigned int offset;
	struct virtualb_page *t_page;
	void *dst, *src;

	while (count < n) {
		temp = min_t(size_t, virtualb->dev->blocksize, n - count);

		if (virtual_cache_active(virtualb) && !is_fua)
			virtual_make_cache_space(virtualb, PAGE_SIZE);

		offset = (sector & SECTOR_MASK) << SECTOR_SHIFT;
		t_page = virtual_insert_page(virtualb, sector,
			!virtual_cache_active(virtualb) || is_fua);
		if (!t_page)
			return -ENOSPC;

		src = kmap_atomic(source);
		dst = kmap_atomic(t_page->page);
		memcpy(dst + offset, src + off + count, temp);
		kunmap_atomic(dst);
		kunmap_atomic(src);

		__set_bit(sector & SECTOR_MASK, t_page->bitmap);

		if (is_fua)
			virtual_free_sector(virtualb, sector, true);

		count += temp;
		sector += temp >> SECTOR_SHIFT;
	}
	return 0;
}

static int copy_from_virtualb(struct virtualb *virtualb, struct page *dest,
	unsigned int off, sector_t sector, size_t n)
{
	size_t temp, count = 0;
	unsigned int offset;
	struct virtualb_page *t_page;
	void *dst, *src;

	while (count < n) {
		temp = min_t(size_t, virtualb->dev->blocksize, n - count);

		offset = (sector & SECTOR_MASK) << SECTOR_SHIFT;
		t_page = virtual_lookup_page(virtualb, sector, false,
			!virtual_cache_active(virtualb));

		dst = kmap_atomic(dest);
		if (!t_page) {
			memset(dst + off + count, 0, temp);
			goto next;
		}
		src = kmap_atomic(t_page->page);
		memcpy(dst + off + count, src + offset, temp);
		kunmap_atomic(src);
next:
		kunmap_atomic(dst);

		count += temp;
		sector += temp >> SECTOR_SHIFT;
	}
	return 0;
}

static void virtual_handle_discard(struct virtualb *virtualb, sector_t sector, size_t n)
{
	size_t temp;

	spin_lock_irq(&virtualb->lock);
	while (n > 0) {
		temp = min_t(size_t, n, virtualb->dev->blocksize);
		virtual_free_sector(virtualb, sector, false);
		if (virtual_cache_active(virtualb))
			virtual_free_sector(virtualb, sector, true);
		sector += temp >> SECTOR_SHIFT;
		n -= temp;
	}
	spin_unlock_irq(&virtualb->lock);
}

static int virtual_handle_flush(struct virtualb *virtualb)
{
	int err;

	if (!virtual_cache_active(virtualb))
		return 0;

	spin_lock_irq(&virtualb->lock);
	while (true) {
		err = virtual_make_cache_space(virtualb,
			virtualb->dev->cache_size * 1024 * 1024);
		if (err || virtualb->dev->curr_cache == 0)
			break;
	}

	WARN_ON(!radix_tree_empty(&virtualb->dev->cache));
	spin_unlock_irq(&virtualb->lock);
	return err;
}

static int virtual_transfer(struct virtualb *virtualb, struct page *page,
	unsigned int len, unsigned int off, bool is_write, sector_t sector,
	bool is_fua)
{
	int err = 0;

	if (!is_write) {
		err = copy_from_virtualb(virtualb, page, off, sector, len);
		flush_dcache_page(page);
	} else {
		flush_dcache_page(page);
		err = copy_to_virtualb(virtualb, page, off, sector, len, is_fua);
	}

	return err;
}

static int virtual_handle_rq(struct virtualb_cmd *cmd)
{
	struct request *rq = cmd->rq;
	struct virtualb *virtualb = cmd->nq->dev->virtualb;
	int err;
	unsigned int len;
	sector_t sector;
	struct req_iterator iter;
	struct bio_vec bvec;

	sector = blk_rq_pos(rq);

	if (req_op(rq) == REQ_OP_DISCARD) {
		virtual_handle_discard(virtualb, sector, blk_rq_bytes(rq));
		return 0;
	}

	spin_lock_irq(&virtualb->lock);
	rq_for_each_segment(bvec, rq, iter) {
		len = bvec.bv_len;
		err = virtual_transfer(virtualb, bvec.bv_page, len, bvec.bv_offset,
				     op_is_write(req_op(rq)), sector,
				     req_op(rq) & REQ_FUA);
		if (err) {
			spin_unlock_irq(&virtualb->lock);
			return err;
		}
		sector += len >> SECTOR_SHIFT;
	}
	spin_unlock_irq(&virtualb->lock);

	return 0;
}

static int virtual_handle_bio(struct virtualb_cmd *cmd)
{
	struct bio *bio = cmd->bio;
	struct virtualb *virtualb = cmd->nq->dev->virtualb;
	int err;
	unsigned int len;
	sector_t sector;
	struct bio_vec bvec;
	struct bvec_iter iter;

	sector = bio->bi_iter.bi_sector;

	if (bio_op(bio) == REQ_OP_DISCARD) {
		virtual_handle_discard(virtualb, sector,
			bio_sectors(bio) << SECTOR_SHIFT);
		return 0;
	}

	spin_lock_irq(&virtualb->lock);
	bio_for_each_segment(bvec, bio, iter) {
		len = bvec.bv_len;
		err = virtual_transfer(virtualb, bvec.bv_page, len, bvec.bv_offset,
				     op_is_write(bio_op(bio)), sector,
				     bio_op(bio) & REQ_FUA);
		if (err) {
			spin_unlock_irq(&virtualb->lock);
			return err;
		}
		sector += len >> SECTOR_SHIFT;
	}
	spin_unlock_irq(&virtualb->lock);
	return 0;
}

static void virtual_stop_queue(struct virtualb *virtualb)
{
	struct request_queue *q = virtualb->q;

	if (virtualb->dev->queue_mode == VIRTUAL_Q_MQ)
		blk_mq_stop_hw_queues(q);
}

static void virtual_restart_queue_async(struct virtualb *virtualb)
{
	struct request_queue *q = virtualb->q;

	if (virtualb->dev->queue_mode == VIRTUAL_Q_MQ)
		blk_mq_start_stopped_hw_queues(q, true);
}

static inline blk_status_t virtual_handle_throttled(struct virtualb_cmd *cmd)
{
	struct virtualb_device *dev = cmd->nq->dev;
	struct virtualb *virtualb = dev->virtualb;
	blk_status_t sts = BLK_STS_OK;
	struct request *rq = cmd->rq;

	if (!hrtimer_active(&virtualb->bw_timer))
		hrtimer_restart(&virtualb->bw_timer);

	if (atomic_long_sub_return(blk_rq_bytes(rq), &virtualb->cur_bytes) < 0) {
		virtual_stop_queue(virtualb);
		/* race with timer */
		if (atomic_long_read(&virtualb->cur_bytes) > 0)
			virtual_restart_queue_async(virtualb);
		/* requeue request */
		sts = BLK_STS_DEV_RESOURCE;
	}
	return sts;
}

static inline blk_status_t virtual_handle_badblocks(struct virtualb_cmd *cmd,
						 sector_t sector,
						 sector_t nr_sectors)
{
	struct badblocks *bb = &cmd->nq->dev->badblocks;
	sector_t first_bad;
	int bad_sectors;

	if (badblocks_check(bb, sector, nr_sectors, &first_bad, &bad_sectors))
		return BLK_STS_IOERR;

	return BLK_STS_OK;
}

static inline blk_status_t virtual_handle_memory_backed(struct virtualb_cmd *cmd,
						     enum req_opf op)
{
	struct virtualb_device *dev = cmd->nq->dev;
	int err;

	if (dev->queue_mode == VIRTUAL_Q_BIO)
		err = virtual_handle_bio(cmd);
	else
		err = virtual_handle_rq(cmd);

	return errno_to_blk_status(err);
}

static inline void virtualb_complete_cmd(struct virtualb_cmd *cmd)
{
	/* Complete IO by inline, softirq or timer */
	switch (cmd->nq->dev->irqmode) {
	case VIRTUAL_IRQ_SOFTIRQ:
		switch (cmd->nq->dev->queue_mode) {
		case VIRTUAL_Q_MQ:
			blk_mq_complete_request(cmd->rq);
			break;
		case VIRTUAL_Q_BIO:
			/*
			 * XXX: no proper submitting cpu information available.
			 */
			end_cmd(cmd);
			break;
		}
		break;
	case VIRTUAL_IRQ_NONE:
		end_cmd(cmd);
		break;
	case VIRTUAL_IRQ_TIMER:
		virtual_cmd_end_timer(cmd);
		break;
	}
}

static blk_status_t virtual_handle_cmd(struct virtualb_cmd *cmd, sector_t sector,
				    sector_t nr_sectors, enum req_opf op)
{
	struct virtualb_device *dev = cmd->nq->dev;
	struct virtualb *virtualb = dev->virtualb;
	blk_status_t sts;

	if (test_bit(VIRTUALB_DEV_FL_THROTTLED, &dev->flags)) {
		sts = virtual_handle_throttled(cmd);
		if (sts != BLK_STS_OK)
			return sts;
	}

	if (op == REQ_OP_FLUSH) {
		cmd->error = errno_to_blk_status(virtual_handle_flush(virtualb));
		goto out;
	}

	if (virtualb->dev->badblocks.shift != -1) {
		cmd->error = virtual_handle_badblocks(cmd, sector, nr_sectors);
		if (cmd->error != BLK_STS_OK)
			goto out;
	}

	if (dev->memory_backed)
		cmd->error = virtual_handle_memory_backed(cmd, op);

out:
	virtualb_complete_cmd(cmd);
	return BLK_STS_OK;
}


static enum hrtimer_restart virtualb_bwtimer_fn(struct hrtimer *timer)
{
	struct virtualb *virtualb = container_of(timer, struct virtualb, bw_timer);
	ktime_t timer_interval = ktime_set(0, TIMER_INTERVAL);
	unsigned int mbps = virtualb->dev->mbps;

	if (atomic_long_read(&virtualb->cur_bytes) == mb_per_tick(mbps))
		return HRTIMER_NORESTART;

	atomic_long_set(&virtualb->cur_bytes, mb_per_tick(mbps));
	virtual_restart_queue_async(virtualb);

	hrtimer_forward_now(&virtualb->bw_timer, timer_interval);

	return HRTIMER_RESTART;
}

static void virtualb_setup_bwtimer(struct virtualb *virtualb)
{
	ktime_t timer_interval = ktime_set(0, TIMER_INTERVAL);

	hrtimer_init(&virtualb->bw_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	virtualb->bw_timer.function = virtualb_bwtimer_fn;
	atomic_long_set(&virtualb->cur_bytes, mb_per_tick(virtualb->dev->mbps));
	hrtimer_start(&virtualb->bw_timer, timer_interval, HRTIMER_MODE_REL);
}

static struct virtualb_queue *virtualb_to_queue(struct virtualb *virtualb)
{
	int index = 0;

	if (virtualb->nr_queues != 1)
		index = raw_smp_processor_id() / ((nr_cpu_ids + virtualb->nr_queues - 1) / virtualb->nr_queues);

	return &virtualb->queues[index];
}

static blk_qc_t virtual_queue_bio(struct request_queue *q, struct bio *bio)
{
	sector_t sector = bio->bi_iter.bi_sector;
	sector_t nr_sectors = bio_sectors(bio);
	struct virtualb *virtualb = q->queuedata;
	struct virtualb_queue *nq = virtualb_to_queue(virtualb);
	struct virtualb_cmd *cmd;

	cmd = alloc_cmd(nq, 1);
	cmd->bio = bio;

	virtual_handle_cmd(cmd, sector, nr_sectors, bio_op(bio));
	return BLK_QC_T_NONE;
}

static bool should_timeout_request(struct request *rq)
{
#ifdef CONFIG_BLK_DEV_NULL_BLK_FAULT_INJECTION
	if (g_timeout_str[0])
		return should_fail(&null_timeout_attr, 1);
#endif
	return false;
}

static bool should_requeue_request(struct request *rq)
{
#ifdef CONFIG_BLK_DEV_NULL_BLK_FAULT_INJECTION
	if (g_requeue_str[0])
		return should_fail(&null_requeue_attr, 1);
#endif
	return false;
}


static blk_status_t virtual_queue_rq(struct blk_mq_hw_ctx *hctx,
			 const struct blk_mq_queue_data *bd)
{
	struct virtualb_cmd *cmd = blk_mq_rq_to_pdu(bd->rq);
	struct virtualb_queue *nq = hctx->driver_data;
	sector_t nr_sectors = blk_rq_sectors(bd->rq);
	sector_t sector = blk_rq_pos(bd->rq);

	might_sleep_if(hctx->flags & BLK_MQ_F_BLOCKING);

	if (nq->dev->irqmode == VIRTUAL_IRQ_TIMER) {
		hrtimer_init(&cmd->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		cmd->timer.function = virtual_cmd_timer_expired;
	}
	cmd->rq = bd->rq;
	cmd->error = BLK_STS_OK;
	cmd->nq = nq;

	blk_mq_start_request(bd->rq);

	if (should_requeue_request(bd->rq)) {
		/*
		 * Alternate between hitting the core BUSY path, and the
		 * driver driven requeue path
		 */
		nq->requeue_selection++;
		if (nq->requeue_selection & 1)
			return BLK_STS_RESOURCE;
		else {
			blk_mq_requeue_request(bd->rq, true);
			return BLK_STS_OK;
		}
	}
	if (should_timeout_request(bd->rq))
		return BLK_STS_OK;

	return virtual_handle_cmd(cmd, sector, nr_sectors, req_op(bd->rq));
}

static const struct blk_mq_ops virtual_mq_ops = {
	.queue_rq       = virtual_queue_rq,
	.complete	= virtual_softirq_done_fn,
};

static void cleanup_queue(struct virtualb_queue *nq)
{
	kfree(nq->tag_map);
	kfree(nq->cmds);
}

static void cleanup_queues(struct virtualb *virtualb)
{
	int i;

	for (i = 0; i < virtualb->nr_queues; i++)
		cleanup_queue(&virtualb->queues[i]);

	kfree(virtualb->queues);
}

#ifdef CONFIG_NVM

static void virtual_lnvm_end_io(struct request *rq, blk_status_t status)
{
	struct nvm_rq *rqd = rq->end_io_data;

	/* XXX: lighnvm core seems to expect NVM_RSP_* values here.. */
	rqd->error = status ? -EIO : 0;
	nvm_end_io(rqd);

	blk_put_request(rq);
}

static int virtual_lnvm_submit_io(struct nvm_dev *dev, struct nvm_rq *rqd)
{
	struct request_queue *q = dev->q;
	struct request *rq;
	struct bio *bio = rqd->bio;

	rq = blk_mq_alloc_request(q,
		op_is_write(bio_op(bio)) ? REQ_OP_DRV_OUT : REQ_OP_DRV_IN, 0);
	if (IS_ERR(rq))
		return -ENOMEM;

	blk_init_request_from_bio(rq, bio);

	rq->end_io_data = rqd;

	blk_execute_rq_nowait(q, NULL, rq, 0, virtual_lnvm_end_io);

	return 0;
}

static int virtual_lnvm_id(struct nvm_dev *dev, struct nvm_id *id)
{
	struct virtualb *virtualb = dev->q->queuedata;
	sector_t size = (sector_t)virtualb->dev->size * 1024 * 1024ULL;
	sector_t blksize;
	struct nvm_id_group *grp;

	id->ver_id = 0x1;
	id->vmnt = 0;
	id->cap = 0x2;
	id->dom = 0x1;

	id->ppaf.blk_offset = 0;
	id->ppaf.blk_len = 16;
	id->ppaf.pg_offset = 16;
	id->ppaf.pg_len = 16;
	id->ppaf.sect_offset = 32;
	id->ppaf.sect_len = 8;
	id->ppaf.pln_offset = 40;
	id->ppaf.pln_len = 8;
	id->ppaf.lun_offset = 48;
	id->ppaf.lun_len = 8;
	id->ppaf.ch_offset = 56;
	id->ppaf.ch_len = 8;

	sector_div(size, virtualb->dev->blocksize); /* convert size to pages */
	size >>= 8; /* concert size to pgs pr blk */
	grp = &id->grp;
	grp->mtype = 0;
	grp->fmtype = 0;
	grp->num_ch = 1;
	grp->num_pg = 256;
	blksize = size;
	size >>= 16;
	grp->num_lun = size + 1;
	sector_div(blksize, grp->num_lun);
	grp->num_blk = blksize;
	grp->num_pln = 1;

	grp->fpg_sz = virtualb->dev->blocksize;
	grp->csecs = virtualb->dev->blocksize;
	grp->trdt = 25000;
	grp->trdm = 25000;
	grp->tprt = 500000;
	grp->tprm = 500000;
	grp->tbet = 1500000;
	grp->tbem = 1500000;
	grp->mpos = 0x010101; /* single plane rwe */
	grp->cpar = virtualb->dev->hw_queue_depth;

	return 0;
}

static void *virtual_lnvm_create_dma_pool(struct nvm_dev *dev, char *name)
{
	mempool_t *virtmem_pool;

	virtmem_pool = mempool_create_slab_pool(64, ppa_cache);
	if (!virtmem_pool) {
		pr_err("virtual_blk: Unable to create virtual memory pool\n");
		return NULL;
	}

	return virtmem_pool;
}

static void virtual_lnvm_destroy_dma_pool(void *pool)
{
	mempool_destroy(pool);
}

static void *virtual_lnvm_dev_dma_alloc(struct nvm_dev *dev, void *pool,
				gfp_t mem_flags, dma_addr_t *dma_handler)
{
	return mempool_alloc(pool, mem_flags);
}

static void virtual_lnvm_dev_dma_free(void *pool, void *entry,
							dma_addr_t dma_handler)
{
	mempool_free(entry, pool);
}

static struct nvm_dev_ops virtual_lnvm_dev_ops = {
	.identity		= virtual_lnvm_id,
	.submit_io		= virtual_lnvm_submit_io,

	.create_dma_pool	= virtual_lnvm_create_dma_pool,
	.destroy_dma_pool	= virtual_lnvm_destroy_dma_pool,
	.dev_dma_alloc		= virtual_lnvm_dev_dma_alloc,
	.dev_dma_free		= virtual_lnvm_dev_dma_free,

	/* Simulate nvme protocol restriction */
	.max_phys_sect		= 64,
};

static int virtual_nvm_register(struct virtualb *virtualb)
{
	struct nvm_dev *dev;
	int rv;

	dev = nvm_alloc_dev(0);
	if (!dev)
		return -ENOMEM;

	dev->q = virtualb->q;
	memcpy(dev->name, virtualb->disk_name, DISK_NAME_LEN);
	dev->ops = &virtual_lnvm_dev_ops;

	rv = nvm_register(dev);
	if (rv) {
		kfree(dev);
		return rv;
	}
	virtualb->ndev = dev;
	return 0;
}

static void virtual_nvm_unregister(struct virtualb *virtualb)
{
	nvm_unregister(virtualb->ndev);
}
#else
static int virtual_nvm_register(struct virtualb *virtualb)
{
	pr_err("virtual_blk: CONFIG_NVM needs to be enabled for LightNVM\n");
	return -EINVAL;
}
static void virtual_nvm_unregister(struct virtualb *virtualb) {}
#endif /* CONFIG_NVM */

static void virtual_del_dev(struct virtualb *virtualb)
{
	struct virtualb_device *dev = virtualb->dev;

	ida_simple_remove(&virtualb_indexes, virtualb->index);

	list_del_init(&virtualb->list);

	if (dev->use_lightnvm)
		virtual_nvm_unregister(virtualb);
	else
		del_gendisk(virtualb->disk);

	if (test_bit(VIRTUALB_DEV_FL_THROTTLED, &virtualb->dev->flags)) {
		hrtimer_cancel(&virtualb->bw_timer);
		atomic_long_set(&virtualb->cur_bytes, LONG_MAX);
		virtual_restart_queue_async(virtualb);
	}

	blk_cleanup_queue(virtualb->q);
	if (dev->queue_mode == VIRTUAL_Q_MQ &&
	    virtualb->tag_set == &virtualb->__tag_set)
		blk_mq_free_tag_set(virtualb->tag_set);
	if (!dev->use_lightnvm)
		put_disk(virtualb->disk);
	cleanup_queues(virtualb);
	if (virtual_cache_active(virtualb))
		virtual_free_device_storage(virtualb->dev, true);
	kfree(virtualb);
	dev->virtualb = NULL;
}

static void virtual_config_discard(struct virtualb *virtualb)
{
	if (virtualb->dev->discard == false)
		return;
	virtualb->q->limits.discard_granularity = virtualb->dev->blocksize;
	virtualb->q->limits.discard_alignment = virtualb->dev->blocksize;
	blk_queue_max_discard_sectors(virtualb->q, UINT_MAX >> 9);
	blk_queue_flag_set(QUEUE_FLAG_DISCARD, virtualb->q);
}

static int virtual_open(struct block_device *bdev, fmode_t mode)
{
	return 0;
}

static void virtual_release(struct gendisk *disk, fmode_t mode)
{
}

static const struct block_device_operations virtual_fops = {
	.owner =	THIS_MODULE,
	.open =		virtual_open,
	.release =	virtual_release,
};

static void virtual_init_queue(struct virtualb *virtualb, struct virtualb_queue *nq)
{
	BUG_ON(!virtualb);
	BUG_ON(!nq);

	init_waitqueue_head(&nq->wait);
	nq->queue_depth = virtualb->queue_depth;
	nq->dev = virtualb->dev;
}

static void virtual_init_queues(struct virtualb *virtualb)
{
	struct request_queue *q = virtualb->q;
	struct blk_mq_hw_ctx *hctx;
	struct virtualb_queue *nq;
	int i;

	queue_for_each_hw_ctx(q, hctx, i) {
		if (!hctx->nr_ctx || !hctx->tags)
			continue;
		nq = &virtualb->queues[i];
		hctx->driver_data = nq;
		virtual_init_queue(virtualb, nq);
		virtualb->nr_queues++;
	}
}

static int setup_commands(struct virtualb_queue *nq)
{
	struct virtualb_cmd *cmd;
	int i, tag_size;

	nq->cmds = kzalloc(nq->queue_depth * sizeof(*cmd), GFP_KERNEL);
	if (!nq->cmds)
		return -ENOMEM;

	tag_size = ALIGN(nq->queue_depth, BITS_PER_LONG) / BITS_PER_LONG;
	nq->tag_map = kzalloc(tag_size * sizeof(unsigned long), GFP_KERNEL);
	if (!nq->tag_map) {
		kfree(nq->cmds);
		return -ENOMEM;
	}

	for (i = 0; i < nq->queue_depth; i++) {
		cmd = &nq->cmds[i];
		INIT_LIST_HEAD(&cmd->list);
		cmd->ll_list.next = NULL;
		cmd->tag = -1U;
	}

	return 0;
}

static int setup_queues(struct virtualb *virtualb)
{
	virtualb->queues = kzalloc(virtualb->dev->submit_queues *
		sizeof(struct virtualb_queue), GFP_KERNEL);
	if (!virtualb->queues)
		return -ENOMEM;

	virtualb->nr_queues = 0;
	virtualb->queue_depth = virtualb->dev->hw_queue_depth;

	return 0;
}

static int init_driver_queues(struct virtualb *virtualb)
{
	struct virtualb_queue *nq;
	int i, ret = 0;

	for (i = 0; i < virtualb->dev->submit_queues; i++) {
		nq = &virtualb->queues[i];

		virtual_init_queue(virtualb, nq);

		ret = setup_commands(nq);
		if (ret)
			return ret;
		virtualb->nr_queues++;
	}
	return 0;
}

static int virtual_gendisk_register(struct virtualb *virtualb)
{
	struct gendisk *disk;
	sector_t size;

	disk = virtualb->disk = alloc_disk_node(1, virtualb->dev->home_node);
	if (!disk)
		return -ENOMEM;
	size = (sector_t)virtualb->dev->size * 1024 * 1024ULL;
	set_capacity(disk, size >> 9);

	disk->flags |= GENHD_FL_EXT_DEVT | GENHD_FL_SUPPRESS_PARTITION_INFO;
	disk->major		= virtual_major;
	disk->first_minor	= virtualb->index;
	disk->fops		= &virtual_fops;
	disk->private_data	= virtualb;
	disk->queue		= virtualb->q;
	strncpy(disk->disk_name, virtualb->disk_name, DISK_NAME_LEN);

	add_disk(disk);
	virtual_T_disk = disk;
	return 0;
}

static int virtual_init_tag_set(struct virtualb *virtualb, struct blk_mq_tag_set *set)
{
	set->ops = &virtual_mq_ops;
	set->nr_hw_queues = virtualb ? virtualb->dev->submit_queues :
						g_submit_queues;
	set->queue_depth = virtualb ? virtualb->dev->hw_queue_depth :
						g_hw_queue_depth;
	set->numa_node = virtualb ? virtualb->dev->home_node : g_home_node;
	set->cmd_size	= sizeof(struct virtualb_cmd);
	set->flags = BLK_MQ_F_SHOULD_MERGE;
	set->driver_data = NULL;

	if ((virtualb && virtualb->dev->blocking) || g_blocking)
		set->flags |= BLK_MQ_F_BLOCKING;

	return blk_mq_alloc_tag_set(set);
}

static void virtual_validate_conf(struct virtualb_device *dev)
{
	dev->blocksize = round_down(dev->blocksize, 512);
	dev->blocksize = clamp_t(unsigned int, dev->blocksize, 512, 4096);
	if (dev->use_lightnvm && dev->blocksize != 4096)
		dev->blocksize = 4096;

	if (dev->use_lightnvm && dev->queue_mode != VIRTUAL_Q_MQ)
		dev->queue_mode = VIRTUAL_Q_MQ;

	if (dev->queue_mode == VIRTUAL_Q_MQ && dev->use_per_node_hctx) {
		if (dev->submit_queues != nr_online_nodes)
			dev->submit_queues = nr_online_nodes;
	} else if (dev->submit_queues > nr_cpu_ids)
		dev->submit_queues = nr_cpu_ids;
	else if (dev->submit_queues == 0)
		dev->submit_queues = 1;

	dev->queue_mode = min_t(unsigned int, dev->queue_mode, VIRTUAL_Q_MQ);
	dev->irqmode = min_t(unsigned int, dev->irqmode, VIRTUAL_IRQ_TIMER);

	/* Do memory allocation, so set blocking */
	if (dev->memory_backed)
		dev->blocking = true;
	else /* cache is meaningless */
		dev->cache_size = 0;
	dev->cache_size = min_t(unsigned long, ULONG_MAX / 1024 / 1024,
						dev->cache_size);
	dev->mbps = min_t(unsigned int, 1024 * 40, dev->mbps);
	/* can not stop a queue */
	if (dev->queue_mode == VIRTUAL_Q_BIO)
		dev->mbps = 0;
}

static int virtual_add_dev(struct virtualb_device *dev)
{
	struct virtualb *virtualb;
	int rv;

	virtual_validate_conf(dev);

	virtualb = kzalloc_node(sizeof(*virtualb), GFP_KERNEL, dev->home_node);
	if (!virtualb) {
		rv = -ENOMEM;
		goto out;
	}
	virtualb->dev = dev;
	dev->virtualb = virtualb;

	spin_lock_init(&virtualb->lock);

	rv = setup_queues(virtualb);
	if (rv)
		goto out_free_virtualb;

	if (dev->queue_mode == VIRTUAL_Q_MQ) {
		if (shared_tags) {
			virtualb->tag_set = &tag_set;
			rv = 0;
		} else {
			virtualb->tag_set = &virtualb->__tag_set;
			rv = virtual_init_tag_set(virtualb, virtualb->tag_set);
		}

		if (rv)
			goto out_cleanup_queues;

		virtualb->q = blk_mq_init_queue(virtualb->tag_set);
		if (IS_ERR(virtualb->q)) {
			rv = -ENOMEM;
			goto out_cleanup_tags;
		}
		virtual_init_queues(virtualb);
	} else if (dev->queue_mode == VIRTUAL_Q_BIO) {
		virtualb->q = blk_alloc_queue_node(GFP_KERNEL, dev->home_node);
		if (!virtualb->q) {
			rv = -ENOMEM;
			goto out_cleanup_queues;
		}
		blk_queue_make_request(virtualb->q, virtual_queue_bio);
		rv = init_driver_queues(virtualb);
		if (rv)
			goto out_cleanup_blk_queue;
	} 
	if (dev->mbps) {
		set_bit(VIRTUALB_DEV_FL_THROTTLED, &dev->flags);
		virtualb_setup_bwtimer(virtualb);
	}

	if (dev->cache_size > 0) {
		set_bit(VIRTUALB_DEV_FL_CACHE, &virtualb->dev->flags);
		blk_queue_write_cache(virtualb->q, true, true);
	}

	virtualb->q->queuedata = virtualb;
	blk_queue_flag_set(QUEUE_FLAG_NONROT, virtualb->q);
	blk_queue_flag_clear(QUEUE_FLAG_ADD_RANDOM, virtualb->q);

	mutex_lock(&lock);
	virtualb->index = ida_simple_get(&virtualb_indexes, 0, 0, GFP_KERNEL);
	dev->index = virtualb->index;
	mutex_unlock(&lock);

	blk_queue_logical_block_size(virtualb->q, dev->blocksize);
	blk_queue_physical_block_size(virtualb->q, dev->blocksize);

	virtual_config_discard(virtualb);

	sprintf(virtualb->disk_name, "virtualb%d", virtualb->index);

	if (dev->use_lightnvm)
		rv = virtual_nvm_register(virtualb);
	else
		rv = virtual_gendisk_register(virtualb);

	if (rv)
		goto out_cleanup_blk_queue;

	mutex_lock(&lock);
	list_add_tail(&virtualb->list, &virtualb_list);
	mutex_unlock(&lock);

	return 0;
out_cleanup_blk_queue:
	blk_cleanup_queue(virtualb->q);
out_cleanup_tags:
	if (dev->queue_mode == VIRTUAL_Q_MQ && virtualb->tag_set == &virtualb->__tag_set)
		blk_mq_free_tag_set(virtualb->tag_set);
out_cleanup_queues:
	cleanup_queues(virtualb);
out_free_virtualb:
	kfree(virtualb);
out:
	return rv;
}

static int __init virtual_init(void)
{
	int ret = 0;
	unsigned int i;
	struct virtualb *virtualb;
	struct virtualb_device *dev;

	if (g_bs > PAGE_SIZE) {
		pr_warn("virtual_blk: invalid block size\n");
		pr_warn("virtual_blk: defaults block size to %lu\n", PAGE_SIZE);
		g_bs = PAGE_SIZE;
	}

	if (g_use_lightnvm && g_bs != 4096) {
		pr_warn("virtual_blk: LightNVM only supports 4k block size\n");
		pr_warn("virtual_blk: defaults block size to 4k\n");
		g_bs = 4096;
	}

	if (g_use_lightnvm && g_queue_mode != VIRTUAL_Q_MQ) {
		pr_warn("virtual_blk: LightNVM only supported for blk-mq\n");
		pr_warn("virtual_blk: defaults queue mode to blk-mq\n");
		g_queue_mode = VIRTUAL_Q_MQ;
	}

	if (g_queue_mode == VIRTUAL_Q_MQ && g_use_per_node_hctx) {
		if (g_submit_queues != nr_online_nodes) {
			pr_warn("virtual_blk: submit_queues param is set to %u.\n",
							nr_online_nodes);
			g_submit_queues = nr_online_nodes;
		}
	} else if (g_submit_queues > nr_cpu_ids)
		g_submit_queues = nr_cpu_ids;
	else if (g_submit_queues <= 0)
		g_submit_queues = 1;

	if (g_queue_mode == VIRTUAL_Q_MQ && shared_tags) {
		ret = virtual_init_tag_set(NULL, &tag_set);
		if (ret)
			return ret;
	}

	config_group_init(&virtualb_subsys.su_group);
	mutex_init(&virtualb_subsys.su_mutex);

	ret = configfs_register_subsystem(&virtualb_subsys);
	if (ret)
		goto err_tagset;

	mutex_init(&lock);

	virtual_major = register_blkdev(0, "virtualb");
	if (virtual_major < 0) {
		ret = virtual_major;
		goto err_conf;
	}

	if (g_use_lightnvm) {
		ppa_cache = kmem_cache_create("ppa_cache", 64 * sizeof(u64),
								0, 0, NULL);
		if (!ppa_cache) {
			pr_err("virtual_blk: unable to create ppa cache\n");
			ret = -ENOMEM;
			goto err_ppa;
		}
	}

	for (i = 0; i < nr_devices; i++) {
		dev = virtual_alloc_dev();
		if (!dev) {
			ret = -ENOMEM;
			goto err_dev;
		}
		ret = virtual_add_dev(dev);
		if (ret) {
			virtual_free_dev(dev);
			goto err_dev;
		}
	}

	pr_info("virtual: module loaded\n");
	return 0;

err_dev:
	while (!list_empty(&virtualb_list)) {
		virtualb = list_entry(virtualb_list.next, struct virtualb, list);
		dev = virtualb->dev;
		virtual_del_dev(virtualb);
		virtual_free_dev(dev);
	}
	kmem_cache_destroy(ppa_cache);
err_ppa:
	unregister_blkdev(virtual_major, "virtualb");
err_conf:
	configfs_unregister_subsystem(&virtualb_subsys);
err_tagset:
	if (g_queue_mode == VIRTUAL_Q_MQ && shared_tags)
		blk_mq_free_tag_set(&tag_set);
	return ret;
}

static void __exit virtual_exit(void)
{
	struct virtualb *virtualb;

	configfs_unregister_subsystem(&virtualb_subsys);

	unregister_blkdev(virtual_major, "virtualb");

	mutex_lock(&lock);
	while (!list_empty(&virtualb_list)) {
		struct virtualb_device *dev;

		virtualb = list_entry(virtualb_list.next, struct virtualb, list);
		dev = virtualb->dev;
		virtual_del_dev(virtualb);
		virtual_free_dev(dev);
	}
	mutex_unlock(&lock);

	if (g_queue_mode == VIRTUAL_Q_MQ && shared_tags)
		blk_mq_free_tag_set(&tag_set);

	kmem_cache_destroy(ppa_cache);
}

module_init(virtual_init);
module_exit(virtual_exit);

MODULE_AUTHOR("Jens Axboe <axboe@kernel.dk>");
MODULE_LICENSE("GPL");
