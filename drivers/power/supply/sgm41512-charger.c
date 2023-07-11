/*
 * SGM41512 charger driver
 *
 * Version: v1.0.0
 *
 * Copyright (C) 2021 Hisense Corporation, All rights reserved
 *
 *  Author: lishaoxiang <lishaoxiang@hisense.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define pr_fmt(fmt) "--PM-SGM41512: %s: " fmt, __func__

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/workqueue.h>
#include <linux/pmic-voter.h>
#include <linux/string.h>
#include <linux/extcon.h>
#include <linux/extcon-provider.h>

#define SGM41512_REG_0				0x0
#define SGM41512_REG_1				0x1
#define SGM41512_REG_2				0x2
#define SGM41512_REG_3				0x3
#define SGM41512_REG_4				0x4
#define SGM41512_REG_5				0x5
#define SGM41512_REG_6				0x6
#define SGM41512_REG_7				0x7
#define SGM41512_REG_8				0x8
#define SGM41512_REG_9				0x9
#define SGM41512_REG_A				0xa
#define SGM41512_REG_B				0xb

#define SGM41512_BATTERY_NAME			"battery"
#define SGM41512_USB_NAME			    "usb"
#define BIT_DP_DM_BC_ENB				BIT(0)

#define SGM41512_REG_IINLIM_BASE			100

#define SGM41512_REG_ICHG_LSB			60

#define SGM41512_REG_ICHG_MASK			GENMASK(5, 0)

#define SGM41512_REG_CHG_MASK			GENMASK(4, 4)

#define SGM41512_REG_CHG_SHIFT			4

#define SGM41512_REG_RESET_MASK		GENMASK(6, 6)

#define SGM41512_REG_OTG_MASK			GENMASK(5, 5)

#define SGM41512_REG_WATCHDOG_MASK		GENMASK(6, 6)

#define SGM41512_REG_WATCHDOG_CTL_MASK		GENMASK(5, 4)

#define SGM41512_REG_TERMINAL_VOLTAGE_MASK	GENMASK(7, 3)
#define SGM41512_REG_TERMINAL_VOLTAGE_SHIFT	3

#define SGM41512_REG_TERMINAL_CUR_MASK		GENMASK(3, 0)

#define SGM41512_REG_VINDPM_VOLTAGE_MASK		GENMASK(3, 0)
#define SGM41512_REG_OVP_MASK					GENMASK(7, 6)
#define SGM41512_REG_OVP_SHIFT					6

#define SGM41512_REG_LIMIT_CURRENT_MASK		GENMASK(4, 0)

#define SGM41512_REG_EN_TIMER_MASK				GENMASK(3, 3)
#define SGM41512_REG_EN_TIMER_SHIFT      			3

#define SNK_DAM_MASK				GENMASK(6, 4)
#define SNK_DAM_500MA_BIT			BIT(6)

//REG08 status bits, Read only.
#define SGM41512_REG_VBUS_STAT_MASK			GENMASK(7, 5)
#define SGM41512_REG_VBUS_STAT_SHIFT			5
#define SGM41512_REG_CHRG_STAT_MASK			GENMASK(4, 3)
#define SGM41512_REG_CHRG_STAT_SHIFT			3
#define SGM41512_REG_VBUS_GOOD_STAT_BIT		BIT(2)
#define SGM41512_REG_THERM_STAT_BIT			BIT(1)
#define SGM41512_REG_VSYS_STAT_BIT				BIT(0)

//REG09 fault bits, Read only.
#define SGM41512_REG_WATCHDOG_FAULT_BIT      	BIT(7)
#define SGM41512_REG_BOOST_FAULT_BIT			BIT(6)
#define SGM41512_REG_CHRG_FAULT_MASK			GENMASK(5, 4)
#define SGM41512_REG_BATTERY_FAULT_BIT			BIT(3)
#define SGM41512_REG_NTC_FAULT_MASK				GENMASK(2, 0)

//REG0A stat bits, Read and R/W.
#define SGM41512_REG_VBUS_GOOD_BIT      			BIT(7)
#define SGM41512_REG_INPUT_VOLTAGE_STAT_BIT	BIT(6)
#define SGM41512_REG_INPUT_CURRENT_STAT_BIT	BIT(5)
#define SGM41512_REG_INPUT_OVLO_STAT_BIT		BIT(2)
#define SGM41512_REG_VIN_INT_PULSE_BIT			BIT(1)
#define SGM41512_REG_IIN_INT_PULSE_BIT			BIT(0)

#define SGM41512_OTG_VALID_MS					500
#define SGM41512_FEED_WATCHDOG_VALID_MS		50
#define SGM41512_OTG_RETRY_TIMES				5
#define SGM41512_LIMIT_CURRENT_MAX				3000000

#define FCHG_OVP_6P5V					6500

#define SDP_CURRENT_500MA				500000
#define DCP_CURRENT_2000MA				2000000
#define CDP_CURRENT_1500MA				1500000
#define FLOAT_CURRENT_900MA				900000
#define FCHG_CURRENT_2000MA				2000000
#define UNKNOWN_CURRENT_500MA		500000

#define FCC_ICHG_CURRENT_500MA			500000
#define FCC_ICHG_CURRENT_2000MA			2000000
#define FCC_WARM_CURRENT				1100000
#define FCC_COLD_CURRENT					780000

#define TERMINATION_CURRENT_LIMIT		500
#define CHARGE_WARM_VOLTAGE_LIMIT		4110
#define CHARGE_NORMAL_VOLTAGE_LIMIT	4400

enum sgm41512_charger_type {
	NO_INPUT = 0,
	SDP_TYPE = 1,
	CDP_TYPE = 2,
	DCP_TYPE = 3,
	UNKNOWN_TYPE = 5,
	NON_STANDARD_TYPE = 6,
	OTG_TYPE = 7,
};

enum sgm41512_charger_state {
	CHARGER_DISABLE = 0,
	CHARGER_PRE_CHARGE = 1,
	CHARGER_FAST_CHARGING = 2,
	CHARGER_TERMINATED = 3,
};

enum sgm41512_batt_state {
	BATT_NORMAL = 0,
	BATT_WARM = 2,
	BATT_COOL = 3,
	BATT_COLD = 5,
	BATT_HOT = 6,
};

struct sgm41512_charge_current {
	int sdp_cur;
	int dcp_cur;
	int cdp_cur;
	int float_cur;
	int unknown_cur;
};

struct sgm41512_charger_info {
	struct i2c_client *client;
	struct device *dev;
	struct mutex lock;
	struct extcon_dev *extcon;
	struct power_supply *psy_sgm;
	struct power_supply *batt_psy;
	struct power_supply *usb_phy;
	struct delayed_work chg_work;
	struct sgm41512_charge_current cur;
	bool good_vbus_detected;
	bool charging;
	bool usb_mode;
	u32 limit;
	u32 charger_type;
	u32 charger_status;
	u32 batt_health_status;
	bool therm_status;
	bool vsys_status;
	u32 chg_en_gpio;
	u32 int_gpio;
	int int_irq;
	int voltage_max_microvolt;
	bool need_redet;
	struct wakeup_source *sgm41512_ws;
};

extern bool sgm_typec_charger_ok(void);

static struct sgm41512_charger_info *global_sgm41512_info = NULL;

static bool sgm41512_charger_is_bat_present(struct sgm41512_charger_info *info)
{
	union power_supply_propval val;
	bool present = false;
	int ret;

	if (!info->batt_psy) {
		info->batt_psy = power_supply_get_by_name(SGM41512_BATTERY_NAME);
		if (!info->batt_psy) {
			dev_err(info->dev, "Could not get battery power_supply\n");
			return PTR_ERR(info->batt_psy);
		}
	}

	ret = power_supply_get_property(info->batt_psy, POWER_SUPPLY_PROP_PRESENT,
					&val);
	if (ret == 0 && val.intval)
		present = true;

	return present;
}

static int sgm41512_get_batt_capacity(struct sgm41512_charger_info *info, int *capacity)
{
	union power_supply_propval val;
	int ret;

	if (!info->batt_psy) {
		info->batt_psy = power_supply_get_by_name(SGM41512_BATTERY_NAME);
		if (!info->batt_psy) {
			dev_err(info->dev, "Could not get battery power_supply\n");
			return PTR_ERR(info->batt_psy);
		}
	}

	ret = power_supply_get_property(info->batt_psy,
			POWER_SUPPLY_PROP_CAPACITY, &val);
	if (ret) {
		dev_err(info->dev, "get failed to get battery capacity now %d\n", ret);
		return ret;
	}

	*capacity = val.intval;

	return 0;
}

static int sgm41512_read(struct sgm41512_charger_info *info, u8 reg, u8 *data)
{
	int ret;

	ret = i2c_smbus_read_byte_data(info->client, reg);
	if (ret < 0)
		return ret;

	*data = ret;
	return 0;
}

static int sgm41512_write(struct sgm41512_charger_info *info, u8 reg, u8 data)
{
	return i2c_smbus_write_byte_data(info->client, reg, data);
}

static int sgm41512_update_bits(struct sgm41512_charger_info *info, u8 reg,
			       u8 mask, u8 data)
{
	u8 v;
	int ret;

	ret = sgm41512_read(info, reg, &v);
	if (ret < 0)
		return ret;

	v &= ~mask;
	v |= (data & mask);

	return sgm41512_write(info, reg, v);
}

static int
sgm41512_charger_set_vindpm(struct sgm41512_charger_info *info, u32 vol)
{
	u8 reg_val;

	if (vol <= 3900)
		reg_val = 0x0;
	else if (vol >= 5400)
		reg_val = 0x0f;
	else
		reg_val = (vol - 3900) / 100;

	pr_err("reg_val is 0x%x\n", reg_val);
	return sgm41512_update_bits(info, SGM41512_REG_6,
				   SGM41512_REG_VINDPM_VOLTAGE_MASK, reg_val);
}

static int
sgm41512_charger_set_ovp(struct sgm41512_charger_info *info, u32 vol)
{
	u8 reg_val;

	if (vol <= 5500)
		reg_val = 0x0;
	else if (vol > 5500 && vol <= 6500)
		reg_val = 0x01;
	else if (vol > 6500 && vol <= 10500)
		reg_val = 0x02;
	else
		reg_val = 0x03;

	return sgm41512_update_bits(info, SGM41512_REG_6,
				   SGM41512_REG_OVP_MASK,
				   reg_val << SGM41512_REG_OVP_SHIFT);
}

static int
sgm41512_charger_set_termina_vol(struct sgm41512_charger_info *info, u32 vol)
{
	u8 reg_val;

	if (vol <= 3850) {
		reg_val = 0x0;
	} else if (vol >= 4610) {
		reg_val = 0x2d;
	} else {
		reg_val = (vol - 3848) / 32;
	}

	pr_err("reg_val is 0x%x\n", reg_val);
	return sgm41512_update_bits(info, SGM41512_REG_4,
				   SGM41512_REG_TERMINAL_VOLTAGE_MASK,
				   reg_val << SGM41512_REG_TERMINAL_VOLTAGE_SHIFT);
}

#if 0
static int
sgm41512_charger_set_termina_cur(struct sgm41512_charger_info *info, u32 cur)
{
	u8 reg_val;

	if (cur <= 60)
		reg_val = 0x0;
	else if (cur >= 960)
		reg_val = 0xf;
	else
		reg_val = (cur - 60) / 60;

	return sgm41512_update_bits(info, SGM41512_REG_3,
				   SGM41512_REG_TERMINAL_CUR_MASK,
				   reg_val);
}
#endif

extern struct power_supply_desc usb_psy_desc;
static int sgm41512_update_charger_status(struct sgm41512_charger_info *info)
{
	u8 reg_val;
	int ret;

	ret = sgm41512_read(info, SGM41512_REG_8, &reg_val);
	if (ret < 0)
		return ret;

	//pr_err("REG_8 val is 0x%x\n", reg_val);
	switch ((reg_val & SGM41512_REG_VBUS_STAT_MASK) >> SGM41512_REG_VBUS_STAT_SHIFT) {
	case SDP_TYPE:
		info->charger_type = POWER_SUPPLY_TYPE_USB;
		break;
	case CDP_TYPE:
		info->charger_type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case DCP_TYPE:
		info->charger_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case UNKNOWN_TYPE:
	case NON_STANDARD_TYPE:
		info->charger_type = POWER_SUPPLY_TYPE_USB_FLOAT;
		break;
	case OTG_TYPE:
		//info->charger_type = POWER_SUPPLY_TYPE_UFP;
		break;
	case NO_INPUT:
	default:
		info->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	}
	if (!sgm_typec_charger_ok()) {
		usb_psy_desc.type = info->charger_type;
		pr_err("usb_psy_desc.type set to %d\n", usb_psy_desc.type);
	}
	switch ((reg_val & SGM41512_REG_CHRG_STAT_MASK) >> SGM41512_REG_CHRG_STAT_SHIFT) {
	case CHARGER_PRE_CHARGE:
	case CHARGER_FAST_CHARGING:
		info->charger_status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case CHARGER_TERMINATED:
		info->charger_status = POWER_SUPPLY_STATUS_FULL;
		pr_err("charging is completed, status set to full\n");
		break;
	case CHARGER_DISABLE:
	default:
		info->charger_status = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	}

	//Input power source is good.
	if (reg_val & SGM41512_REG_VBUS_GOOD_STAT_BIT)
		info->good_vbus_detected = true;
	else
		info->good_vbus_detected = false;

	if (reg_val & SGM41512_REG_THERM_STAT_BIT) {
		info->therm_status = true;
	} else
		info->therm_status = false;

	if (reg_val & SGM41512_REG_VSYS_STAT_BIT) {
		info->vsys_status = true;
	} else
		info->vsys_status = false;

	pr_err("REG_8 val is 0x%x, charger_type %d, charger_status %d, good_vbus %d, therm_status %d, vsys_status %d\n",
		reg_val, info->charger_type, info->charger_status, info->good_vbus_detected, info->therm_status, info->vsys_status);

	return ret;
}

static int sgm41512_update_fault_status(struct sgm41512_charger_info *info)
{
	u8 reg_val;
	int ret;

	ret = sgm41512_read(info, SGM41512_REG_9, &reg_val);
	if (ret < 0)
		return ret;

	pr_err("REG_9 val is 0x%x\n", reg_val);

	if (reg_val & SGM41512_REG_WATCHDOG_FAULT_BIT)
		pr_err("REG_9 WATCHDOG_FAULT\n");

	if (reg_val & SGM41512_REG_BOOST_FAULT_BIT)
		pr_err("REG_9 BOOST_FAULT\n");

	if (reg_val & SGM41512_REG_CHRG_FAULT_MASK)
		pr_err("REG_9 CHRG_FAULT 0x%x\n",
			reg_val & SGM41512_REG_CHRG_FAULT_MASK);

	if (reg_val & SGM41512_REG_BATTERY_FAULT_BIT)
		pr_err("REG_9 BATTERY_FAULT\n");

	return ret;
}

static int sgm41512_charger_set_fcc_current(struct sgm41512_charger_info *info,
				       u32 cur)
{
	u8 reg_val;

	if (sgm_typec_charger_ok())
		cur = FCC_ICHG_CURRENT_500MA;

	pr_err("FCC current = %d\n", cur);
	cur = cur / 1000;
	if (cur > 3000) {
		reg_val = 0x32;
	} else {
		reg_val = cur / SGM41512_REG_ICHG_LSB;
		reg_val &= SGM41512_REG_ICHG_MASK;
	}

	return sgm41512_update_bits(info, SGM41512_REG_2,
				   SGM41512_REG_ICHG_MASK,
				   reg_val);
}

static int sgm41512_charger_get_fcc_current(struct sgm41512_charger_info *info,
				       u32 *cur)
{
	u8 reg_val;
	int ret;

	ret = sgm41512_read(info, SGM41512_REG_2, &reg_val);
	if (ret < 0)
		return ret;

	reg_val &= SGM41512_REG_ICHG_MASK;
	*cur = reg_val * SGM41512_REG_ICHG_LSB * 1000;
	pr_err("get_current = %d\n", *cur);

	return 0;
}

static int sgm41512_charger_get_charge_voltage(struct sgm41512_charger_info *info,
				       u32 *charge_voltage)
{
	u8 reg_val;
	int ret;

	ret = sgm41512_read(info, SGM41512_REG_4, &reg_val);
	if (ret < 0)
		return ret;

	reg_val &= SGM41512_REG_TERMINAL_VOLTAGE_MASK;
	*charge_voltage = 3848 + 32 * (reg_val >> 3);
	pr_err("get charge_voltage = %d mV\n", *charge_voltage);

	return 0;
}

static int sgm41512_charger_set_input_current(struct sgm41512_charger_info *info,
				  u32 limit_cur)
{
	u8 reg_val;
	int ret;

	pr_err("USBIN limit_current = %d\n", limit_cur);
	if (limit_cur >= SGM41512_LIMIT_CURRENT_MAX)
		limit_cur = SGM41512_LIMIT_CURRENT_MAX;

	limit_cur = limit_cur / 1000;
	//input current limit value = 100 + 100 * reg_val
	reg_val = (limit_cur / SGM41512_REG_IINLIM_BASE) - 1;

	ret = sgm41512_update_bits(info, SGM41512_REG_0,
				  SGM41512_REG_LIMIT_CURRENT_MASK,
				  reg_val);
	if (ret)
		dev_err(info->dev, "set SGM41512 limit cur failed\n");

	return ret;
}

static u32 sgm41512_charger_get_input_current(struct sgm41512_charger_info *info,
				  u32 *limit_cur)
{
	u8 reg_val;
	int ret;

	ret = sgm41512_read(info, SGM41512_REG_0, &reg_val);
	if (ret < 0)
		return ret;

	reg_val &= SGM41512_REG_LIMIT_CURRENT_MASK;
	*limit_cur = (reg_val + 1) * SGM41512_REG_IINLIM_BASE * 1000;
	if (*limit_cur >= SGM41512_LIMIT_CURRENT_MAX)
		*limit_cur = SGM41512_LIMIT_CURRENT_MAX;
	pr_err("get_limit_current = %d\n",*limit_cur);

	return 0;
}

static int sgm41512_charger_start_charge(struct sgm41512_charger_info *info)
{
	int usbin_cur, ret;

	if (!info->good_vbus_detected)
		return -EINVAL;

	//mutex_lock(&info->lock);

	/* set current limitation and start to charge */
	switch (info->charger_type) {
	case POWER_SUPPLY_TYPE_USB:
		usbin_cur = info->cur.sdp_cur;
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		usbin_cur = info->cur.dcp_cur;
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		usbin_cur = info->cur.cdp_cur;
		break;
	case POWER_SUPPLY_TYPE_USB_FLOAT:
		usbin_cur = info->cur.float_cur;
		break;
	default:
		usbin_cur = info->cur.unknown_cur;
	}

	ret = sgm41512_charger_set_input_current(info, usbin_cur);
	if (ret)
		goto out;

	if (!info->chg_en_gpio) {
		ret = sgm41512_update_bits(info, SGM41512_REG_1,
			   SGM41512_REG_CHG_MASK, 1 << SGM41512_REG_CHG_SHIFT);
		if (ret) {
			dev_err(info->dev, "enable SGM41512 charge failed\n");
			goto out;
		}
		pr_err("cmd to set enable charge\n");

	} else {
		gpio_direction_output(info->chg_en_gpio, 0);
		pr_err("set chg_en_gpio 0\n");
	}

	info->charging = true;
out:
	//mutex_unlock(&info->lock);
	//pr_err("start charge, ret = %d.\n", ret);

	return ret;

}

static void sgm41512_charger_stop_charge(struct sgm41512_charger_info *info)
{
	int ret = 0;

	if(!info->chg_en_gpio) {
		ret = sgm41512_update_bits(info, SGM41512_REG_1,
				SGM41512_REG_CHG_MASK, 0 << SGM41512_REG_CHG_SHIFT);
		if(ret) {
			dev_err(info->dev, "disable SGM41512 charge failed\n");
			return;
		}
		pr_err("###STOP charger### cmd to set disable charge\n");
	} else {
		gpio_direction_output(info->chg_en_gpio, 1);
		pr_err("###STOP charger### set chg_en_gpio to high\n");
	}

	info->charging = false;
}

static int sgm41512_batt_health_sync(struct sgm41512_charger_info *info)
{
	union power_supply_propval pval = {0, };

	if (!info->batt_psy) {
		info->batt_psy = power_supply_get_by_name(SGM41512_BATTERY_NAME);
		if (!info->batt_psy) {
			dev_err(info->dev, "Could not get battery power_supply\n");
			return PTR_ERR(info->batt_psy);
		}
	}

	power_supply_get_property(info->batt_psy, POWER_SUPPLY_PROP_HEALTH, &pval);
	info->batt_health_status = pval.intval;
	pr_err("batt_health_status is %d\n", info->batt_health_status);

	return 0;
}

static int sgm41512_set_prop_temp_control(struct sgm41512_charger_info *info)
{
	int ret = 0;

	// update battery health status.
	ret = sgm41512_batt_health_sync(info);
	if (ret) {
		pr_err("get batt_health_sync %d\n", ret);
		return ret;
	}

	pr_err("batt_health is %d, gd_vbus %d\n",
		info->batt_health_status, info->good_vbus_detected);

	//check charging status.
	if (!info->good_vbus_detected)
		return 0;

	switch (info->batt_health_status) {
	case POWER_SUPPLY_HEALTH_OVERHEAT:
	case POWER_SUPPLY_HEALTH_COLD:
		sgm41512_charger_stop_charge(info);
		break;
	case POWER_SUPPLY_HEALTH_WARM:
		sgm41512_charger_start_charge(info);
		sgm41512_charger_set_fcc_current(info, FCC_WARM_CURRENT);
		sgm41512_charger_set_termina_vol(info, CHARGE_WARM_VOLTAGE_LIMIT);
		break;
	case POWER_SUPPLY_HEALTH_COOL:
		sgm41512_charger_start_charge(info);
		sgm41512_charger_set_fcc_current(info, FCC_COLD_CURRENT);
		break;
	case POWER_SUPPLY_HEALTH_GOOD:
	default:
		sgm41512_charger_set_fcc_current(info, FCC_ICHG_CURRENT_2000MA);
		sgm41512_charger_set_termina_vol(info, CHARGE_NORMAL_VOLTAGE_LIMIT);
		break;
	}

	return 0;
}
static int sgm41512_charger_get_health(struct sgm41512_charger_info *info,
				      u32 *health)
{
	int ret = 0;

	// update battery health status.
	ret = sgm41512_batt_health_sync(info);
	if (ret) {
		pr_err("get batt_health_sync %d\n", ret);
		return ret;
	}

	*health = info->batt_health_status;

	return 0;
}


static int sgm41512_charger_get_online(struct sgm41512_charger_info *info,
				      u32 *online)
{
	if (info->charging)
		*online = true;
	else
		*online = false;

	return 0;
}

static int sgm41512_charger_get_status(struct sgm41512_charger_info *info,
				      u32 *status)
{
	int capacity = 0;
	/*Update FULL while battery capacity is 100%.*/
	if (info->charger_status == POWER_SUPPLY_STATUS_CHARGING) {
		sgm41512_get_batt_capacity(info, &capacity);
		if(capacity == 100) {
			*status = POWER_SUPPLY_STATUS_FULL;
			return 0;
		}
	}

	/*Resolved the issue of UI display battery already full
	 *while battery capacity is not 100%(such as warm JEITA triggered).*/
	if (info->charger_status == POWER_SUPPLY_STATUS_FULL) {
		sgm41512_get_batt_capacity(info, &capacity);
		if(capacity != 100) {
			*status = POWER_SUPPLY_STATUS_CHARGING;
			return 0;
		}
	}

	*status = info->charger_status;

	return 0;
}

static int sgm41512_watchdog_ctl(struct sgm41512_charger_info *info, int enable)
{
	int ret;

	if(enable)
		ret = sgm41512_update_bits(info, SGM41512_REG_5,
				  SGM41512_REG_WATCHDOG_CTL_MASK, 0x10);
	else
		ret = sgm41512_update_bits(info, SGM41512_REG_5,
				  SGM41512_REG_WATCHDOG_CTL_MASK, 0x0);
	if (ret)
		dev_err(info->dev, "set watchdog failed\n");

	return ret;
}

static int sgm41512_charger_set_status(struct sgm41512_charger_info *info,
				      int val)
{
	int ret = 0;

	if (val < 2)
		return 0;

	pr_err("info->charging = %d, val = %d\n", info->charging, val);
	if (val == 3 && info->charging) {
		sgm41512_charger_stop_charge(info);
		pr_err("sgm41512_charger_stop_charge\n");
	}
	else if (val == 2)
	{
		ret = sgm41512_charger_start_charge(info);
		if (ret)
			dev_err(info->dev, "start charge failed\n");
		else{
		    pr_err("sgm41512_charger start charge\n");
		}
	}

	return 0;
}
/*
static int sgm41512_get_safety_timer_enable(struct sgm41512_charger_info *info, u32 *en)
{
	int ret = 0;
	u8 val = 0;

	ret = sgm41512_read(info, SGM41512_REG_5, &val);
	if (ret < 0) {
		dev_err(info->dev, "read SGM41512 safety timer enable failed\n");
		return ret;
	}
	*en = !! (val & SGM41512_REG_EN_TIMER_MASK);

	pr_err("get_safety_timer_enable=%x\n",*en);
	return ret;
}

static int sgm41512_enable_safety_timer(struct sgm41512_charger_info *info,u8 val)
{
	int ret = 0;
	val = val << SGM41512_REG_EN_TIMER_SHIFT;

	ret = sgm41512_update_bits(info, SGM41512_REG_5,SGM41512_REG_EN_TIMER_MASK,
			val);
	if (ret)
		dev_err(info->dev, "set enable_safety_timer failed\n");

	pr_err("set_safety_timer_enable=%x rc=%d\n", val, ret);
	return ret;
}
*/
static int sgm41512_charger_enable_otg(struct sgm41512_charger_info *info, u32 enable)
{
	int ret;
	//union extcon_property_value val = {1};

	ret = sgm41512_update_bits(info, SGM41512_REG_1,
				SGM41512_REG_OTG_MASK,
				enable ? SGM41512_REG_OTG_MASK : 0);
	if (ret) {
		dev_err(info->dev, "%s SGM41512 otg failed\n", enable ? "enable" : "disable");
		return ret;
	}

	if (enable) {
		//extcon_set_property(info->extcon, EXTCON_USB_HOST, EXTCON_PROP_USB_TYPEC_POLARITY, val);
		//extcon_set_property(info->extcon, EXTCON_USB_HOST, EXTCON_PROP_USB_SS, val);
		extcon_set_state_sync(info->extcon, EXTCON_USB_HOST, 1);
	} else
		extcon_set_state_sync(info->extcon, EXTCON_USB_HOST, 0);

	pr_err("%s success.\n", enable ? "enable" : "disable");

	return 0;
}

static int sgm41512_charger_otg_status(struct sgm41512_charger_info *info, u32 *en)
{
	int ret;
	u8 val;

	ret = sgm41512_read(info, SGM41512_REG_1, &val);
	if (ret) {
		dev_err(info->dev, "failed to get SGM41512 otg status\n");
		return ret;
	}

	*en = !!(val & SGM41512_REG_OTG_MASK);
	pr_err("charger_otg_status=%x\n", *en);

	return ret;
}

static irqreturn_t sgm41512_irq_handler(int irq, void *data)
{
	struct sgm41512_charger_info *info = data;

	/*This would allow the userspace process
	* to be able to read power supply uevents to take
	* appropriate actions.
	*/
	__pm_wakeup_event(info->sgm41512_ws, 300);
	//pr_err("start\n");
	//delay 300ms to make sure charger type detected.
	cancel_delayed_work_sync(&info->chg_work);
	schedule_delayed_work(&info->chg_work, msecs_to_jiffies(300));

	return IRQ_HANDLED;
}
static char *maginput[2] = {"MAG_EVENT=MAG_IN", NULL };
static char *magoutput[2] = {"MAG_EVENT=MAG_OUT", NULL };

static void sgm41512_charger_work(struct work_struct *data)
{
	struct sgm41512_charger_info *info =
		container_of(data, struct sgm41512_charger_info, chg_work.work);
	int ret;
	bool present = sgm41512_charger_is_bat_present(info);

	sgm41512_update_charger_status(info);
	sgm41512_update_fault_status(info);

	if (present && info->good_vbus_detected) {
		ret = sgm41512_charger_start_charge(info);
		if (ret)
			goto out;

		if ((info->charger_type == POWER_SUPPLY_TYPE_USB
			|| info->charger_type == POWER_SUPPLY_TYPE_USB_CDP)) {
			pr_err("USB detected, set to USB mode.\n");
			if(((sgm_typec_charger_ok()&&(usb_psy_desc.type != POWER_SUPPLY_TYPE_USB && usb_psy_desc.type != POWER_SUPPLY_TYPE_USB_CDP))
				|| !sgm_typec_charger_ok()) && !info->usb_mode) {
				pr_err("sgm41512:send uevent mag_in\n");
				kobject_uevent_env(&info->dev->kobj, KOBJ_CHANGE, maginput);
				extcon_set_state_sync(info->extcon, EXTCON_USB, 1);
				info->usb_mode = true;
			}
		}

		ret = sgm41512_set_prop_temp_control(info);
		if (ret)
			goto out;

	} else if ((!info->good_vbus_detected && info->charging) || !present) {
		if (info->usb_mode == true) {
			/* Disconnect notification */
				info->usb_mode = false;
				extcon_set_state_sync(info->extcon,
						EXTCON_USB, 0);
				pr_err("sgm41512:send uevent mag_out\n");
				kobject_uevent_env(&info->dev->kobj, KOBJ_CHANGE, magoutput);
			}

		info->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
		info->charger_status = POWER_SUPPLY_STATUS_DISCHARGING;
		info->charging = false;
	}

out:
	if (info->batt_psy)
		power_supply_changed(info->batt_psy);
	pr_err("##batt_present=%d, chg_type=%d, good_vbus_det=%d, usb_mode=%d\n",
		 present, info->charger_type, info->good_vbus_detected, info->usb_mode);
}

static int sgm41512_charger_usb_get_property(struct power_supply *psy,
					    enum power_supply_property psp,
					    union power_supply_propval *val)
{
	struct sgm41512_charger_info *info = power_supply_get_drvdata(psy);
	u32 cur, online, status, voltage,health;
	int ret = 0;
	int en;
	mutex_lock(&info->lock);

	switch (psp) {

	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = info->charging;
		break;

	case POWER_SUPPLY_PROP_STATUS:
		ret = sgm41512_charger_get_status(info, &status);
		if (ret)
			goto out;

		val->intval = status;

		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sgm41512_charger_get_fcc_current(info, &cur);
		if (ret)
			goto out;

		val->intval = cur;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sgm41512_charger_get_input_current(info, &cur);
		if (ret)
			goto out;

		val->intval = cur;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		ret = sgm41512_charger_get_online(info, &online);
		if (ret)
			goto out;

		val->intval = online;

		break;

	case POWER_SUPPLY_PROP_HEALTH:
		ret = sgm41512_charger_get_health(info, &health);
		if (ret)
			goto out;

		val->intval = health;
		break;

	case POWER_SUPPLY_PROP_TYPE:
		val->intval = info->charger_type;
		break;
/*
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		ret = sgm41512_get_safety_timer_enable(info, &en);
		if (ret)
			goto out;
		val->intval = en;
		break;
*/
	case POWER_SUPPLY_PROP_USB_OTG:
		ret = sgm41512_charger_otg_status(info, &en);
		if (ret)
			goto out;
		val->intval = en;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = sgm41512_charger_get_charge_voltage(info, &voltage);
		if (ret)
			goto out;
		val->intval = voltage;
		break;

	default:
		pr_err("POWER_SUPPLY_PROP=%d\n",psp);
		break;
	}

out:
	mutex_unlock(&info->lock);
	return ret;
}

static int sgm41512_charger_usb_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct sgm41512_charger_info *info = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&info->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sgm41512_charger_set_fcc_current(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "set charge current failed\n");
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sgm41512_charger_set_input_current(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "set input current limit failed\n");
		break;

	case POWER_SUPPLY_PROP_STATUS:
		ret = sgm41512_charger_set_status(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "set charge status failed\n");
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = sgm41512_charger_set_termina_vol(info, val->intval / 1000);
		if (ret < 0)
			dev_err(info->dev, "failed to set terminate voltage\n");
		break;
/*
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		ret = sgm41512_enable_safety_timer(info,val->intval);
		if (ret < 0)
			dev_err(info->dev, "failed to set enable safety timer\n");
		break;
*/
	case POWER_SUPPLY_PROP_USB_OTG:
		ret = sgm41512_charger_enable_otg(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "failed to set enable otg\n");
		break;

	case POWER_SUPPLY_PROP_FORCE_RECHARGE:
		//Recheck the charger status and update charger type, status.
		schedule_delayed_work(&info->chg_work, msecs_to_jiffies(200));
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		ret = sgm41512_set_prop_temp_control(info);
		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&info->lock);
	return ret;
}

static int sgm41512_charger_property_is_writeable(struct power_supply *psy,
						 enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_STATUS:
	case	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	//case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
	case POWER_SUPPLY_PROP_USB_OTG:
	case POWER_SUPPLY_PROP_FORCE_RECHARGE:
	case POWER_SUPPLY_PROP_HEALTH:
		ret = 1;
		break;

	default:
		ret = 0;
	}

	return ret;
}

static enum power_supply_property sgm41512_usb_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TYPE,
	//POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_USB_OTG,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_FORCE_RECHARGE,
};

static const struct power_supply_desc sgm41512_charger_desc = {
	.name			= "sgm_charger",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= sgm41512_usb_props,
	.num_properties		= ARRAY_SIZE(sgm41512_usb_props),
	.get_property		= sgm41512_charger_usb_get_property,
	.set_property		= sgm41512_charger_usb_set_property,
	.property_is_writeable	= sgm41512_charger_property_is_writeable,
};

void sgm41512_determine_initial_status(void)
{
	if (!global_sgm41512_info) {
		pr_err("no sgm41512 chip\n");
		return;
	}
	pr_err("sgm41512_determine_initial_status\n");
	schedule_delayed_work(&global_sgm41512_info->chg_work, 0);
}
EXPORT_SYMBOL_GPL(sgm41512_determine_initial_status);

static int sgm41512_charger_hw_init(struct sgm41512_charger_info *info)
{
	int ret = 0;

	pr_err("start\n");

	info->cur.sdp_cur = SDP_CURRENT_500MA;
	info->cur.dcp_cur = DCP_CURRENT_2000MA;
	info->cur.cdp_cur = CDP_CURRENT_1500MA;
	info->cur.float_cur = FLOAT_CURRENT_900MA;
	info->cur.unknown_cur = UNKNOWN_CURRENT_500MA;

	//set REG0A bit0, bit1, to not allow INT pulse.
	ret = sgm41512_update_bits(info, SGM41512_REG_A,
				  SGM41512_REG_VIN_INT_PULSE_BIT,
				  SGM41512_REG_VIN_INT_PULSE_BIT);
	if (ret) {
		dev_err(info->dev, "set SGM41512_REG_VIN_INT_PULSE_BIT failed\n");
		return ret;
	}

	ret = sgm41512_update_bits(info, SGM41512_REG_A,
				  SGM41512_REG_IIN_INT_PULSE_BIT,
				  SGM41512_REG_IIN_INT_PULSE_BIT);
	if (ret) {
		dev_err(info->dev, "set SGM41512_REG_IIN_INT_PULSE_BIT failed\n");
		return ret;
	}

	//set vindpm threshold(aicl).
	info->voltage_max_microvolt = CHARGE_NORMAL_VOLTAGE_LIMIT;
	ret = sgm41512_charger_set_vindpm(info, info->voltage_max_microvolt);
	if (ret) {
		dev_err(info->dev, "set SGM41512 vindpm vol failed\n");
		return ret;
	}

	//set terminated voltage 4.4v
	ret = sgm41512_charger_set_termina_vol(info, info->voltage_max_microvolt);
	if (ret) {
		dev_err(info->dev, "set SGM41512 terminal vol failed\n");
		return ret;
	}

#if 0	//iterm current use default 180mA.
	//set iterm current to 500mA
	ret = sgm41512_charger_set_termina_cur(info, TERMINATION_CURRENT_LIMIT);
	if (ret) {
		dev_err(info->dev, "set SGM41512 terminal cur failed\n");
		return ret;
	}
#endif

	//set fcc to 2A
	ret = sgm41512_charger_set_fcc_current(info, FCC_ICHG_CURRENT_2000MA);
	if (ret) {
		dev_err(info->dev, "set SGM41512 fcc cur failed\n");
		return ret;
	}

	//set usbin ovp threshold to 6.5V
	ret = sgm41512_charger_set_ovp(info, FCHG_OVP_6P5V);
	if (ret) {
		dev_err(info->dev, "set SGM41512 fcc cur failed\n");
		return ret;
	}

	//disable watchdog.
	ret = sgm41512_watchdog_ctl(info, 0);
	if (ret) {
		dev_err(info->dev, "set sgm41512_watchdog_ctl failed\n");
		return ret;
	}

	return ret;
}

static int sgm41512_parse_dt(struct device *dev,
		struct sgm41512_charger_info *info)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	/*sgm41512_int_gpio*/
	ret = of_get_named_gpio(np, "sgm,int-gpio", 0);
	info->int_gpio = ret;
	if(!gpio_is_valid(info->int_gpio)) {
		dev_err(dev, "invalid sgm41512_int_gpio. ret=%d\n", ret);
		return ret;
	}
	ret = gpio_request(info->int_gpio, "sgm41512_int_gpio");
	if (ret < 0) {
		dev_err(dev,
			"gpio %d request failed. ret=%d\n", info->int_gpio, ret);
		return ret;
	}

	/*sgm41512_chg_en_gpio*/
	ret = of_get_named_gpio(np, "sgm,chg-en-gpio", 0);
	info->chg_en_gpio = ret;
	if(!gpio_is_valid(info->chg_en_gpio)) {
		dev_err(dev, "invalid chg_en_gpio gpio: %d\n", ret);
		return ret;
	}
	ret = gpio_request(info->chg_en_gpio, "sgm41512_chg_en_gpio");
	if (ret < 0) {
		dev_err(dev,
			"gpio %d request failed. ret=%d\n", info->chg_en_gpio, ret);
		return ret;
	}

	return ret;
}

static const unsigned int usb_phy_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static int sgm41512_usb_extcon_register(struct sgm41512_charger_info *info)
{
	int ret;

	/* Register extcon to notify USB driver */
	info->extcon = devm_extcon_dev_allocate(info->dev, usb_phy_extcon_cable);
	if (IS_ERR(info->extcon)) {
		dev_err(info->dev, "failed to allocate extcon device\n");
		return PTR_ERR(info->extcon);
	}

	ret = devm_extcon_dev_register(info->dev, info->extcon);
	if (ret) {
		dev_err(info->dev, "failed to register extcon device\n");
		return ret;
	}

	if (ret < 0)
		pr_err("failed to configure extcon capabilities ret=%d\n", ret);
	return 0;
}

static int sgm41512_charger_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct power_supply_config charger_cfg = { };
	struct sgm41512_charger_info *info;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "No support for SMBUS_BYTE_DATA\n");
		return -ENODEV;
	}

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->client = client;
	info->dev = dev;
	i2c_set_clientdata(client, info);
	ret = sgm41512_parse_dt(dev, info);
	if (ret) {
		dev_err(dev,"sgm41512_parse_dt() err\n");
		goto err_parse_dt;
	}

	mutex_init(&info->lock);
	INIT_DELAYED_WORK(&info->chg_work, sgm41512_charger_work);

	info->int_irq = gpio_to_irq(info->int_gpio);
	ret = request_threaded_irq(info->int_irq,
			NULL,
			sgm41512_irq_handler,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			 "sgm41512_irq", info);
	if (ret) {
		dev_err(dev, "request sgm41512_irq failed\n");
		goto err_parse_dt;
	}

	charger_cfg.drv_data = info;
	charger_cfg.of_node = dev->of_node;
	info->psy_sgm = devm_power_supply_register(dev,
						   &sgm41512_charger_desc,
						   &charger_cfg);
	if (IS_ERR(info->psy_sgm)) {
		dev_err(dev, "failed to register power supply\n");
		goto err_parse_dt;
	}

	ret = sgm41512_usb_extcon_register(info);
	if (ret) {
		dev_err(dev, "request sgm41512_usb_extcon_register failed\n");
		goto err_parse_dt;
	}
	ret = sgm41512_charger_hw_init(info);
	if (ret) {
		dev_err(dev, "sgm41512_charger_hw_init failed\n");
		goto err_parse_dt;
	}
	info->sgm41512_ws = wakeup_source_register(info->dev, "sgm41512_ws");
	enable_irq_wake(info->int_irq);

	global_sgm41512_info = info;

	return 0;

err_parse_dt:
	wakeup_source_unregister(info->sgm41512_ws);
	devm_kfree(dev, info);

	return ret;
}

static int sgm41512_charger_remove(struct i2c_client *client)
{
	struct sgm41512_charger_info *info = i2c_get_clientdata(client);

	if (!IS_ERR_OR_NULL(info)) {
		cancel_delayed_work_sync(&info->chg_work);
		devm_kfree(&client->dev, info);
		global_sgm41512_info = NULL;
	}

	return 0;
}
static void sgm41512_charger_shutdown(struct i2c_client *client)
{
	struct sgm41512_charger_info *info = i2c_get_clientdata(client);
	int ret;
	pr_err("sgm41512 otg mode %d\n",IS_ERR_OR_NULL(info));
	if (!IS_ERR_OR_NULL(info))
	{
		ret = sgm41512_update_bits(info, SGM41512_REG_1,
					SGM41512_REG_OTG_MASK, 0);
		//ret  = sgm41512_write(info,SGM41512_REG_1,0x1a);
		if (ret) {
			dev_err(info->dev, "Force Disable SGM41512 otg failed\n" "disable");
		}
		global_sgm41512_info = NULL;
	}
	pr_err("sgm41512 shutdown\n");
}
static const struct i2c_device_id sgm41512_i2c_id[] = {
	{"sgm41512_chg", 0},
	{}
};

static const struct of_device_id sgm41512_charger_of_match[] = {
	{ .compatible = "sgm,sgm41512_chg", },
	{ }
};

MODULE_DEVICE_TABLE(of, sgm41512_charger_of_match);

static struct i2c_driver sgm41512_charger_driver = {
	.driver = {
		.name = "sgm41512_chg",
		.of_match_table = sgm41512_charger_of_match,
	},
	.probe = sgm41512_charger_probe,
	.remove = sgm41512_charger_remove,
	.shutdown =sgm41512_charger_shutdown ,
	.id_table = sgm41512_i2c_id,
};

module_i2c_driver(sgm41512_charger_driver);

MODULE_DESCRIPTION("SGM41512 Charger Driver");
MODULE_AUTHOR("lishaoxiang@hisense.com");
MODULE_LICENSE("GPL v2");
