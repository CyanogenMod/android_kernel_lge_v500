/*
 * BQ51051B Wireless Charging(WLC) control driver
 *
 * Copyright (C) 2012 LG Electronics, Inc
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/mfd/pm8xxx/pm8921-charger.h>
#include <linux/mfd/pm8xxx/pm8921-bms.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/leds-pm8xxx.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/power_supply.h>

#include <linux/power_supply.h>
#include <linux/power/bq51051b_charger.h>

#include <linux/debugfs.h>
#include <mach/board_lge.h>
#include <linux/gpio.h>
#include <linux/mfd/pm8xxx/gpio.h>
#include "../../arch/arm/mach-msm/lge/L05E/board-L05E.h"

struct bq51051b_wlc_chip {
	struct device *dev;
	struct power_supply wireless_psy;
	struct work_struct wireless_interrupt_work;
	struct wake_lock wireless_chip_wake_lock;
	unsigned int active_n_gpio;
	unsigned int wc_ts_ctrl_gpio;
#ifdef CONFIG_WIRELESS_CHARGER_TEMP_SCENARIO
	unsigned int discharging_temp;
	unsigned int discharging_clear_temp;
#endif
	bool wlc_ts_ctrl_gpio_req;
	bool discharging_enabled;
	bool wireless_charging;
	bool wireless_charge_done;
	int (*wlc_is_plugged)(void);
	struct dentry			*dent;
};

static const struct platform_device_id bq51051b_id[] = {
	{BQ51051B_WLC_DEV_NAME, 0},
	{},
};

static struct bq51051b_wlc_chip *the_chip;

static void bms_notify(struct bq51051b_wlc_chip *chip, int value)
{
	if (value)
		pm8921_bms_charging_began();
	else
		pm8921_bms_charging_end(0);
}

static enum power_supply_property pm_power_props_wireless[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static char *pm_power_supplied_to[] = {
	"battery",
};

static int pm_power_get_property_wireless(struct power_supply *psy,
					  enum power_supply_property psp,
					  union power_supply_propval *val)
{
	/* Check if called before init */
	if (!the_chip) {
		WLC_DBG_ERR("called before init\n");
		return -EINVAL;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;	//always battery_on
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = the_chip->wireless_charging;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

#ifdef CONFIG_WIRELESS_CHARGER_TEMP_SCENARIO
static int wireless_discharging(bool disable)
{
	int rc =0;

	WLC_DBG_INFO("wireless_discharging disable =%d \n",disable);

	/* Check if called before init */
	if (!the_chip) {
		WLC_DBG_ERR("called before init\n");
		return -EINVAL;
	}

	if(the_chip->wlc_ts_ctrl_gpio_req ==false){
		WLC_DBG_ERR("wireless_discharging ts_ctrl_gpio is DNI\n");
		return -EINVAL;
	}

	if(disable)
		rc = gpio_direction_output(the_chip->wc_ts_ctrl_gpio, 0);
	else
		rc = gpio_direction_output(the_chip->wc_ts_ctrl_gpio, 1);

	if(rc == 0){
		WLC_DBG_INFO("wireless_discharging success. \n");
		the_chip->discharging_enabled =disable;
	}else{
		WLC_DBG_INFO("wireless_discharging fail. retry wlc discharging. \n");
		the_chip->discharging_enabled =!disable;
	}
	return 0;
}

void  wireless_check_batt_temp(int batt_temp)
{
	/* Check if called before init */
	if (!the_chip) {
		WLC_DBG_ERR("called before init\n");
		return ;
	}

	if(batt_temp >= the_chip->discharging_temp && !the_chip->discharging_enabled )
		wireless_discharging(true);
	else if(batt_temp <= the_chip->discharging_clear_temp && the_chip->discharging_enabled)
		wireless_discharging(false);
}
#endif

static void wireless_set(struct bq51051b_wlc_chip *chip)
{
	/* Check if called before init */
	if (!the_chip) {
		WLC_DBG_ERR("called before init\n");
		return ;
	}

	WLC_DBG_INFO("wireless_set\n");

	wake_lock(&chip->wireless_chip_wake_lock);

	the_chip->wireless_charging = true;
	the_chip->wireless_charge_done = false;

	bms_notify(chip, 1);

	power_supply_changed(&chip->wireless_psy);
	set_wireless_power_supply_control(the_chip->wireless_charging);
}

static void wireless_reset(struct bq51051b_wlc_chip *chip)
{
	/* Check if called before init */
	if (!the_chip) {
		WLC_DBG_ERR("called before init\n");
		return ;
	}

	WLC_DBG_INFO("wireless_reset\n");

	the_chip->wireless_charging = false;
	the_chip->wireless_charge_done = false;

	bms_notify(chip, 0);

	power_supply_changed(&chip->wireless_psy);
	set_wireless_power_supply_control(the_chip->wireless_charging);

#ifdef CONFIG_LGE_PM
	/* Wake lock for delever Uevnet before suspend */
	wake_lock_timeout(&chip->wireless_chip_wake_lock, round_jiffies_relative(msecs_to_jiffies(10000)));
#else
	wake_unlock(&chip->wireless_chip_wake_lock);
#endif
}

static void wireless_interrupt_worker(struct work_struct *work)
{
	struct bq51051b_wlc_chip *chip =
	    container_of(work, struct bq51051b_wlc_chip,
			 wireless_interrupt_work);

	if (chip->wlc_is_plugged())
		wireless_set(chip);
	else
		wireless_reset(chip);
}

static irqreturn_t wireless_interrupt_handler(int irq, void *data)
{
	int chg_state;
	struct bq51051b_wlc_chip *chip = data;

	chg_state = chip->wlc_is_plugged();
	WLC_DBG_INFO("\nwireless is plugged state = %d\n\n", chg_state);
	schedule_work(&chip->wireless_interrupt_work);
	return IRQ_HANDLED;
}

static int set_wlc_ts_ctrl(void *data, u64 val)
{
	int gpio;
	struct pm_gpio param = {
		.direction        = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_OPEN_DRAIN,
		.output_value   = 0,
		.pull    = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_S4,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function         = PM_GPIO_FUNC_NORMAL,
	};

	if (!the_chip) {
		 WLC_DBG_ERR("called before init\n");
		 return -EINVAL;
	}

	gpio = (int) data;

	if(!the_chip->wlc_ts_ctrl_gpio_req){
		WLC_DBG_INFO("adb commed, test =wlc_ts_ctrl_gpio_req %d \n",the_chip->wlc_ts_ctrl_gpio_req);
		pm8xxx_gpio_config(gpio, &param);
		the_chip->wlc_ts_ctrl_gpio_req =true;
		if (gpio_request_one(gpio, GPIOF_OPEN_DRAIN|GPIOF_OUT_INIT_HIGH, "wc_ts_ctrl_gpio")) {
			WLC_DBG_ERR("failed to request GPIO %d\n", gpio);
			return -EINVAL;
		}
	}

	gpio_direction_output(gpio, (int) val);
	WLC_DBG_INFO("adb set_wlc_ts_ctrl, gpio(%d) set(%d) \n",gpio, (int) val);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(wlc_ts_ctrl_fops, NULL, set_wlc_ts_ctrl, "%llu\n");

#ifdef CONFIG_LGE_PM
static bool charging_enable = 1;
static int param_set_charging_enable(const char * val, struct kernel_param * kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	charging_enable = !!charging_enable;
	pr_info("set charging_enable to %d\n", charging_enable);

	wireless_discharging(!charging_enable);

	return 0;
}
module_param_call(charging_enable, param_set_charging_enable,
	param_get_uint, &charging_enable, 0644);
#endif

#ifdef CONFIG_WIRELESS_CHARGER_TEMP_SCENARIO
static int get_wlc_temp(void *data, u64 *  val)
{
	if (!the_chip) {
		WLC_DBG_ERR("called before init\n");
		return -EINVAL;
	}
	if ((int)data == DISCHARGING_TEMP)
		*val = the_chip->discharging_temp;
	if ((int)data == DISCHARGING_CLEAR_TEMP)
		*val = the_chip->discharging_clear_temp;

	WLC_DBG_INFO("get_wlc_temp discharging_temp =%d, discharging_clear_temp =%d  \n",the_chip->discharging_temp,the_chip->discharging_clear_temp );

	return 0;
}

static int set_wlc_temp(void *data, u64  val)
{
	if (!the_chip) {
		WLC_DBG_ERR("called before init\n");
		return -EINVAL;
	}
	if ((int)data == DISCHARGING_TEMP)
		the_chip->discharging_temp = (int)val;
	if ((int)data == DISCHARGING_CLEAR_TEMP)
		the_chip->discharging_clear_temp = (int)val;

	WLC_DBG_INFO("set_wlc_temp discharging_temp =%d, discharging_clear_temp =%d  \n",the_chip->discharging_temp,the_chip->discharging_clear_temp );

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(wlc_temp_fops, get_wlc_temp, set_wlc_temp, "%llu\n");
#endif

static void create_debugfs_entries(struct bq51051b_wlc_chip *chip)
{

	chip->dent = debugfs_create_dir("bq51051b_wlc", NULL);

	if (IS_ERR(chip->dent)) {
		WLC_DBG_ERR("bq51051b wlc charger couldnt create debugfs dir\n");
		return;
	}

	debugfs_create_file("WLC_TS_CTRL", 0644, chip->dent,
			    (int *) chip->wc_ts_ctrl_gpio, &wlc_ts_ctrl_fops);

#ifdef CONFIG_WIRELESS_CHARGER_TEMP_SCENARIO
	debugfs_create_file("discharging_temp", 0644, chip->dent,
			    (int *) DISCHARGING_TEMP, &wlc_temp_fops);
	debugfs_create_file("discharging_clear_temp", 0644, chip->dent,
			    (int *) DISCHARGING_CLEAR_TEMP, &wlc_temp_fops);
#endif
}


static int __devinit bq51051b_wlc_hw_init(struct bq51051b_wlc_chip *chip)
{
	int ret;
	WLC_DBG_INFO("hw_init");

	/* active_n pin must be monitoring the bq51051b status */
	ret = request_irq(gpio_to_irq(chip->active_n_gpio),
			wireless_interrupt_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"wireless_charger", chip);
	if (ret < 0) {
		WLC_DBG_ERR("wireless_charger request irq failed\n");
		return ret;
	}
	enable_irq_wake(gpio_to_irq(chip->active_n_gpio));


	return 0;
}

static int bq51051b_wlc_resume(struct device *dev)
{
	return 0;
}

static int bq51051b_wlc_suspend(struct device *dev)
{
	return 0;
}

static int __devinit bq51051b_wlc_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct bq51051b_wlc_chip *chip;
	const struct bq51051b_wlc_platform_data *pdata =
		pdev->dev.platform_data;

	WLC_DBG_INFO("probe\n");

	if (!pdata) {
		WLC_DBG_ERR("missing platform data\n");
		return -ENODEV;
	}

	chip = kzalloc(sizeof(struct bq51051b_wlc_chip), GFP_KERNEL);
	if (!chip) {
		WLC_DBG_ERR(" Cannot allocate bq51051b_wlc_chip\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;

	chip->active_n_gpio = pdata->active_n_gpio;
	chip->wc_ts_ctrl_gpio = pdata->wc_ts_ctrl_gpio;
#ifdef CONFIG_WIRELESS_CHARGER_TEMP_SCENARIO
	chip->discharging_temp = pdata->discharging_temp;
	chip->discharging_clear_temp = pdata->discharging_clear_temp;
#endif
	chip->wlc_ts_ctrl_gpio_req =false;
	chip->wireless_charging =false;
	chip->wireless_charge_done=false;

	if (lge_get_board_revno() >= HW_REV_D){
		chip->wlc_ts_ctrl_gpio_req =true;
		if (gpio_request_one(chip->wc_ts_ctrl_gpio, GPIOF_OPEN_DRAIN|GPIOF_OUT_INIT_HIGH, "wc_ts_ctrl_gpio")) {
			WLC_DBG_ERR("failed to request GPIO %d\n", chip->wc_ts_ctrl_gpio);
		}
	}

	chip->wlc_is_plugged = pdata->wlc_is_plugged;

	rc = bq51051b_wlc_hw_init(chip);
	if (rc) {
		WLC_DBG_ERR("couldn't init hardware rc = %d\n", rc);
		goto free_chip;
	}

	chip->wireless_psy.name = "wireless";
	chip->wireless_psy.type = POWER_SUPPLY_TYPE_WIRELESS;
	chip->wireless_psy.supplied_to = pm_power_supplied_to;
	chip->wireless_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to);
	chip->wireless_psy.properties = pm_power_props_wireless;
	chip->wireless_psy.num_properties = ARRAY_SIZE(pm_power_props_wireless);
	chip->wireless_psy.get_property = pm_power_get_property_wireless;

	rc = power_supply_register(chip->dev, &chip->wireless_psy);
	if (rc < 0) {
		WLC_DBG_ERR("power_supply_register wireless failed rx = %d\n",
			      rc);
		goto free_chip;
	}

	platform_set_drvdata(pdev, chip);
	the_chip = chip;

	INIT_WORK(&chip->wireless_interrupt_work, wireless_interrupt_worker);
	wake_lock_init(&chip->wireless_chip_wake_lock, WAKE_LOCK_SUSPEND,
		       "bq51051b_wireless_chip");

	create_debugfs_entries(chip);

	/* For Booting Wireless_charging and For Power Charging Logo In Wireless Charging */
	if (chip->wlc_is_plugged())
		wireless_set(chip);

	return 0;

free_chip:
	kfree(chip);
	return rc;
}

static int __devexit bq51051b_wlc_remove(struct platform_device *pdev)
{
	struct bq51051b_wlc_chip *chip = platform_get_drvdata(pdev);

	WLC_DBG_INFO("remove\n");
	wake_lock_destroy(&chip->wireless_chip_wake_lock);
	the_chip = NULL;
	platform_set_drvdata(pdev, NULL);
	power_supply_unregister(&chip->wireless_psy);
	free_irq(gpio_to_irq(chip->active_n_gpio), chip);
	gpio_free(chip->active_n_gpio);
	kfree(chip);
	return 0;
}

static const struct dev_pm_ops bq51051b_pm_ops = {
	.suspend = bq51051b_wlc_suspend,
	.resume = bq51051b_wlc_resume,
};

static struct platform_driver bq51051b_wlc_driver = {
	.probe = bq51051b_wlc_probe,
	.remove = __devexit_p(bq51051b_wlc_remove),
	.id_table = bq51051b_id,
	.driver = {
		.name = BQ51051B_WLC_DEV_NAME,
		.owner = THIS_MODULE,
		.pm = &bq51051b_pm_ops,
	},
};

static int __init bq51051b_wlc_init(void)
{
	return platform_driver_register(&bq51051b_wlc_driver);
}

static void __exit bq51051b_wlc_exit(void)
{
	platform_driver_unregister(&bq51051b_wlc_driver);
}

late_initcall(bq51051b_wlc_init);
module_exit(bq51051b_wlc_exit);

MODULE_AUTHOR("Kyungtae Oh <kyungtae.oh@lge.com>");
MODULE_DESCRIPTION("BQ51051B Wireless Charger Control Driver");
MODULE_LICENSE("GPL v2");
