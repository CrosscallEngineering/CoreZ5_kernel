
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#define DETECT_DELAY_TIMEOUT (HZ)
#define MAX_ANT_CTRL_GPIO    16

static int ant_ctrl_pin[MAX_ANT_CTRL_GPIO];
static int ant_ctrl_irq[MAX_ANT_CTRL_GPIO];
static int ant_ctrl_sts[MAX_ANT_CTRL_GPIO];

struct detect_timer {
	struct timer_list timer;
	int idx;
};
static struct detect_timer det_timers[MAX_ANT_CTRL_GPIO];

static int current_size = 0;

static void ant_ctrl_det_timer(struct timer_list *timer)
{
	struct detect_timer *temp = container_of(timer,
				struct detect_timer, timer);
	int idx = temp->idx;
	int sts = 0;

	if (idx >= current_size) {
		pr_err("the idx value is error\n");
		return;
	}

	sts = gpio_get_value(ant_ctrl_pin[idx]);
	if (sts == 0) {
		ant_ctrl_sts[idx] = 0;
		pr_err("The gpio=%d is remove\n", ant_ctrl_pin[idx]);
	} else {
		ant_ctrl_sts[idx] = 1;
		pr_err("The gpio=%d is insert\n", ant_ctrl_pin[idx]);
	}
}

static irqreturn_t ant_ctrl_detect_irq(int irq, void *data)
{
	struct timer_list *cur_timer = data;

	del_timer(cur_timer);
	cur_timer->expires = jiffies + DETECT_DELAY_TIMEOUT;
	add_timer(cur_timer);

	return IRQ_HANDLED;
}

static int ant_ctrl_proc_show(struct seq_file *m, void *v)
{
	int i = 0;

	seq_printf(m, "ANT cable insert state:\n");
	for (i = 0; i < current_size; i++) {
		seq_printf(m, "GPIO[%d] state %d\n",
			ant_ctrl_pin[i],
			gpio_get_value(ant_ctrl_pin[i]));
	}

	return 0;
}
static int ant_ctrl_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ant_ctrl_proc_show, NULL);
}

static const struct file_operations ant_ctrl_proc_fops = {
	.open     = ant_ctrl_proc_open,
	.read     = seq_read,
	.llseek   = seq_lseek,
	.release  = single_release,
};

static int ant_ctrl_detect_probe(struct platform_device *pdev)
{
	int i = 0;
	int ret = 0;
	int gpio_array_size = 0;
	struct device *dev = &pdev->dev;
	struct device_node *of_node = NULL;

	if (!pdev->dev.of_node) {
		pr_err("the of node is NULL\n");
		return -EINVAL;
	}
	of_node = pdev->dev.of_node;

	gpio_array_size = of_gpio_count(of_node);
	if (gpio_array_size <= 0 || gpio_array_size > MAX_ANT_CTRL_GPIO) {
		pr_err("read gpio array size failed\n");
		return -EINVAL;
	}

	current_size = gpio_array_size;
	pr_err("read gpio count is %d\n", gpio_array_size);

	for (i = 0; i < gpio_array_size; i++) {
		int irq = 0;

		ant_ctrl_pin[i] = of_get_gpio(of_node, i);
		pr_err("ant gpio array[%d] = %d\n", i, ant_ctrl_pin[i]);
		irq = gpio_to_irq(ant_ctrl_pin[i]);

		/* add timer for delay to detect state */
		det_timers[i].idx = i;
		timer_setup(&det_timers[i].timer, ant_ctrl_det_timer, 0);

		ret = devm_request_threaded_irq(dev, irq, NULL, ant_ctrl_detect_irq,
				IRQF_ONESHOT|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
				"ANT_CTRL", &det_timers[i].timer);
		if (ret)
			pr_err("request irq failed, gpio %d\n", ant_ctrl_pin[i]);

		ant_ctrl_irq[i] = irq;

		ant_ctrl_sts[i] = gpio_get_value(ant_ctrl_pin[i]);
	}

	proc_create("ant_state", 0, NULL, &ant_ctrl_proc_fops);
 
	return 0;
}

static int ant_ctrl_detect_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id ant_ctrl_detect_match[] = {
	{ .compatible = "ant_ctrl_detect" },
	{ }
};
MODULE_DEVICE_TABLE(of, ant_ctrl_detect_match);

static struct platform_driver ant_ctrl_detect_driver = {
	.probe = ant_ctrl_detect_probe,
	.remove = ant_ctrl_detect_remove,

	.driver = {
		.name = "ant_ctrl_detect",
		.of_match_table = ant_ctrl_detect_match,
	},
};

module_platform_driver(ant_ctrl_detect_driver);

MODULE_ALIAS("SAR ANT	 CABLE Detect Driver");
MODULE_LICENSE("GPL v2");

