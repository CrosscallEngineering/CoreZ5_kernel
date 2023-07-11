/*
 * Copyright (C) 2016-2018 Hisense, Inc.
 *
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

struct bd_gps_driver_data {
	int reset_pin;
	int prtrg_pin;
	int beidou_on;
	int serial_id;
	struct regulator *vdd_vreg;
	char serial_if[10];
};
static struct bd_gps_driver_data bd_gps_pdata;

int bd_gps_serial_id(void)
{
	pr_debug("%s(): bd serial id %d\n", __func__, bd_gps_pdata.serial_id);
	return bd_gps_pdata.serial_id;
}

int bd_gps_serial_on(void)
{
	pr_debug("%s(): bd_gps_serial_on %d\n", __func__, bd_gps_pdata.beidou_on);
	return bd_gps_pdata.beidou_on;
}

static int bd_gps_power_ctrl(struct bd_gps_driver_data *pdata, int on)
{
	int err = 0;

	if (on) {
		if (pdata->beidou_on) {
			pr_info("%s(): already enabled\n", __func__);
			return err;
		}

		err = regulator_enable(pdata->vdd_vreg);
		if (err) {
			pr_err("%s: regulator vdd enable failed rc=%d\n",
					__func__, err);
			return err;
		}
		
		if (gpio_is_valid(pdata->reset_pin)) {
			gpio_set_value(pdata->reset_pin, 0);
			msleep(5);
			gpio_set_value(pdata->reset_pin, 1);
		}
		pdata->beidou_on = 1;
	} else {
		if (!pdata->beidou_on) {
			pr_info("%s(): already disabled\n", __func__);
			return err;
		}

		if (gpio_is_valid(pdata->reset_pin))
			gpio_set_value(pdata->reset_pin, 0);

		pdata->beidou_on = 0;
		regulator_disable(pdata->vdd_vreg);
	}

	return err;
}


static ssize_t beidou_on_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "beidou on=%d\n", bd_gps_pdata.beidou_on);
}

static ssize_t beidou_on_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t len)
{
	int ret;
	long value;

	ret = kstrtol(buf, 10, &value);
	if (ret) {
		pr_err("%s: sscanf is wrong!\n", __func__);
		return -EINVAL;
	}

	pr_info("%s: enable beidou chip %ld\n", __func__, value);
	bd_gps_power_ctrl(&bd_gps_pdata, !!value);

	return len;
}
static struct kobj_attribute beidou_on_attr = __ATTR_RW(beidou_on);

static ssize_t beidou_if_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", bd_gps_pdata.serial_if);
}
static struct kobj_attribute beidou_if_attr = __ATTR_RO(beidou_if);

static int serail_if_get(struct bd_gps_driver_data *pdata,
	struct device_node *node)
{
	if (of_property_read_u32(node, "bd,serial-id", &pdata->serial_id)) {
		pr_err("no serial id prop in dts\n");
		return -EINVAL;
	}
	sprintf(pdata->serial_if, "%u", pdata->serial_id);

	return 0;
}

static struct attribute *beidou_attrs[] = {
	&beidou_on_attr.attr,
	&beidou_if_attr.attr,
	NULL
};

static const struct attribute_group beidou_group = {
	.attrs = beidou_attrs,
};

static int bd_gps_probe(struct platform_device *pdev)
{
	int err = 0;
	struct bd_gps_driver_data *pdata = &bd_gps_pdata;
	enum of_gpio_flags gpio_flags;
	struct device_node *np = pdev->dev.of_node;
	struct kobject *root_kobjs;

	pr_info("%s enter\n", __func__);
	pdata->reset_pin = -1;

	pdata->vdd_vreg = devm_regulator_get(&pdev->dev, "avdd");
	if (IS_ERR(pdata->vdd_vreg)) {
		err = PTR_ERR(pdata->vdd_vreg);
		pr_err("%s: vdd vreg get failed rc=%d\n", __func__, err);
		goto exit0;
	}

	pdata->reset_pin = of_get_named_gpio_flags(np, "bd,reset-pin",
				0, &gpio_flags);
	if (gpio_is_valid(pdata->reset_pin)) {
		err = gpio_request_one(pdata->reset_pin, GPIOF_DIR_OUT,
				"beidou_reset");
		if (err)
			pr_err("bd_gps: reset gpio request failed %d\n", err);
		gpio_set_value(pdata->reset_pin, 0);
	} else {
		pr_err("bd_gps: no reset pin define in dts\n");
	}

	pdata->prtrg_pin = of_get_named_gpio_flags(np, "bd,prtrg-pin",
				0, &gpio_flags);
	if (gpio_is_valid(pdata->prtrg_pin)) {
		err = gpio_request_one(pdata->prtrg_pin, GPIOF_DIR_OUT,
				"beidou_prtrg");
		if (err)
			pr_err("bd_gps: prtrg pin request failed %d\n", err);
		gpio_set_value(pdata->prtrg_pin, 0);
	} else {
		pr_err("bd_gps: no prtrg pin define in dts\n");
	}

	/* Get serail if*/
	err = serail_if_get(pdata, np);
	if(err)
		goto exit0;

	/* create ctrl node */
	root_kobjs = kobject_create_and_add("gps_ctrl", NULL);
	err = sysfs_create_group(root_kobjs, &beidou_group);
	if (err < 0)
		pr_err("Error create beidou on/if sysfs %d\n", err);

exit0:
	return err;
}

static int bd_gps_remove(struct platform_device *pdev)
{
	struct bd_gps_driver_data *pdata = &bd_gps_pdata;

	bd_gps_power_ctrl(pdata, 0);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id bd_gps_of_match[] = {
	{ .compatible = "bd,uc221", },
	{ .compatible = "bd,hd8040", },
	{ .compatible = "bd,uc6226", },
	{},
};
MODULE_DEVICE_TABLE(of, bd_gps_of_match);
#endif

static struct platform_driver bd_gps_driver = {
	.probe = bd_gps_probe,
	.remove = bd_gps_remove,
	.driver = {
		.name = "bd_gps_driver",
#ifdef CONFIG_OF
		.of_match_table	= of_match_ptr(bd_gps_of_match),
#endif
	}
};

static int __init bd_gps_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&bd_gps_driver);
	return ret;
}

static void __exit bd_gps_exit(void)
{
	platform_driver_unregister(&bd_gps_driver);
	return;
}

module_init(bd_gps_init);
module_exit(bd_gps_exit);
