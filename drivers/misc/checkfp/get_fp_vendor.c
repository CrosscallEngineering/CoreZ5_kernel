/*
 * Copyright (C) 2017-2018 Hisense, Inc.
 *
 * Author:
 *	sunjunfeng <sunjunfeng1@hisense.com>
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
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/export.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#ifdef CONFIG_HISENSE_CHECK_HARDWARE_VERSION
#include <linux/check_version.h>
static int hardware_id = 0;
#endif/*CONFIG_HISENSE_CHECK_HARDWARE_VERSION*/

static int fp_check_pin_value = 0;
const char *fpname[] = {NULL,NULL};


static const char * const pctl_names[] = {
	"sleep",
	"active",
};
#define FPPIN_PCTL_NAME_SLEEP "sleep"
#define FPPIN_PCTL_NAME_ACTIVE "active"

struct fppin_proc_data {
	struct device *dev;

	struct pinctrl *fppin_proc_pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_sleep;
	int fpvendor_gpio;
};
int get_fppin_val(void)
{
	return fp_check_pin_value;
}
EXPORT_SYMBOL(get_fppin_val);

static int set_fppin_val(struct fppin_proc_data *fppin_proc)
{
	int pin_num = fppin_proc->fpvendor_gpio;
    msleep(1);
    fp_check_pin_value = gpio_get_value(pin_num);
	printk("%s: success, pin_num = %d, fp_check_pin_value = %d\n", __FUNCTION__, pin_num, fp_check_pin_value);
	return 0;
}
static int set_fppin_state(struct fppin_proc_data *fppin_proc)
{
	int pin_num = fppin_proc->fpvendor_gpio;
	int rc;
	struct device *dev = fppin_proc->dev;

	if (gpio_is_valid(pin_num)) {
	    rc = gpio_request(pin_num, "fp_vendor_check");
	    if (rc) {
	        printk("Could not request fp_pin gpio.\n");
	        return rc;
	    }
    } else {
	    printk("not valid fp_pin gpio\n");
	    return -EIO;
    }
	rc = gpio_direction_input(pin_num);
	if (rc) {		
		printk("can not set pin to input\n");
		return -1;
	}

	rc = pinctrl_select_state(fppin_proc->fppin_proc_pinctrl,
					fppin_proc->pins_active);
	if (rc)
		dev_err(dev, "cannot select '%s'\n", FPPIN_PCTL_NAME_ACTIVE);
	else
		dev_info(dev, "Selected '%s'\n", FPPIN_PCTL_NAME_ACTIVE);

	return rc;
}

static int fppin_proc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc = 0;
	struct device_node *np = dev->of_node;
	struct fppin_proc_data *fppin_proc = devm_kzalloc(dev, sizeof(*fppin_proc),
			GFP_KERNEL);
	dev_info(dev, "%s: enter\n", __func__);
	if (!fppin_proc) {
		dev_err(dev,
			"failed to allocate memory for struct fppin_proc_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	fppin_proc->dev = dev;
	platform_set_drvdata(pdev, fppin_proc);

	if (!np) {
		dev_err(dev, "no of node found\n");
		rc = -EINVAL;
		goto exit;
	}

	rc = of_get_named_gpio(np, "fpvendor,pin", 0);
	if(rc < 0){
		printk("%s: fail to get fp check pin_num\n", __func__);
		return (-ENODEV);
	}
	fppin_proc->fpvendor_gpio = rc;

	rc = of_property_read_string(np, "fpname-0", &fpname[0]);
	if (rc < 0){
		printk ("%s: fail to get fpname-0\n",__func__);
		return (-ENODEV);
	}

	rc = of_property_read_string(np, "fpname-1", &fpname[1]);
	if (rc < 0){
		printk ("%s: fail to get fpname-1\n",__func__);
		return (-ENODEV);
	}

	fppin_proc->fppin_proc_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fppin_proc->fppin_proc_pinctrl)) {
		if (PTR_ERR(fppin_proc->fppin_proc_pinctrl) == -EPROBE_DEFER) {
			dev_info(dev, "pinctrl not ready\n");
			rc = -EPROBE_DEFER;
			goto exit;
		}
		dev_err(dev, "Target does not use pinctrl\n");
		fppin_proc->fppin_proc_pinctrl = NULL;
		rc = -EINVAL;
		goto exit;
	}

	fppin_proc->pins_active = pinctrl_lookup_state(fppin_proc->fppin_proc_pinctrl, FPPIN_PCTL_NAME_ACTIVE);
	if (IS_ERR(fppin_proc->pins_active)) {
		dev_err(dev, "cannot find '%s'\n", FPPIN_PCTL_NAME_ACTIVE);
		rc = -EINVAL;
		goto exit;
	}
	dev_info(dev, "found pin control %s\n", FPPIN_PCTL_NAME_ACTIVE);

	fppin_proc->pins_sleep = pinctrl_lookup_state(fppin_proc->fppin_proc_pinctrl, FPPIN_PCTL_NAME_SLEEP);
	if (IS_ERR(fppin_proc->pins_sleep)) {
		dev_err(dev, "cannot find '%s'\n", FPPIN_PCTL_NAME_SLEEP);
		rc = -EINVAL;
		goto exit;
	}
	dev_info(dev, "found pin control %s\n", FPPIN_PCTL_NAME_SLEEP);

	rc = set_fppin_state(fppin_proc);
	if (rc)
		goto exit;
	set_fppin_val(fppin_proc);
	dev_info(dev, "%s: ok\n", __func__);

exit:
	return rc;
}

static int fppin_proc_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "%s\n", __func__);

	return 0;
}

static int fppin_proc_show(struct seq_file *m, void *v)
{
	int len;
	if (fp_check_pin_value)
		len = strlen(fpname[1]);
	else
		len = strlen(fpname[0]);
	seq_write(m, fp_check_pin_value? fpname[1] : fpname[0], len);
	return 0;
}


static int fppin_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fppin_proc_show, NULL);
}

static const struct file_operations checkfppin_fops = {
	.open       = fppin_proc_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static struct of_device_id fppin_proc_of_match[] = {
	{ .compatible = "fpcheckvendor", },
	{}
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct platform_driver fppin_proc_driver = {
	.driver = {
		.name	= "fp_idpin",
		.owner	= THIS_MODULE,
		.of_match_table = fppin_proc_of_match,
	},
	.probe	= fppin_proc_probe,
	.remove	= fppin_proc_remove,
};

static int __init fppin_proc_init(void)
{

	int rc;
	printk("%s(): enter\n", __func__);

	proc_create("fp_vendor_check", 0, NULL, &checkfppin_fops);
	rc = platform_driver_register(&fppin_proc_driver);
	
	if (!rc)
		pr_info("%s OK\n", __func__);
	else
		pr_err("%s %d\n", __func__, rc);
	
	return rc;
}

static void __exit fppin_proc_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&fppin_proc_driver);
}

module_init(fppin_proc_init);
module_exit(fppin_proc_exit);
MODULE_DESCRIPTION("Fingerprint Compatible based on hardware gpio.");


