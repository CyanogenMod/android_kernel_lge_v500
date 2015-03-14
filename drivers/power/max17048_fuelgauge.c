/*
 *  MAX17048_fuelgauge.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 LG Electronics
 *  Dajin Kim <dajin.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/max17048_fuelgauge.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <mach/msm_iomap.h>
#ifdef CONFIG_MACH_APQ8064_ALTEV
#include <linux/power/bq24262_charger.h>
#else
#include <linux/mfd/pm8xxx/pm8921-charger.h>
#endif
/*                                                                                           */
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#if defined (CONFIG_LGE_PM) || defined (CONFIG_LGE_PM_BATTERY_ID_CHECKER)
#include <mach/board_lge.h>
#endif
/*                                   */
#define RCOMP_BL44JN	(0xB8)	/* Default Value for LGP970 Battery */
#define RCOMP_BL53QH	(0x44)  /* Default Value for BL-53QH Battery */
#define MONITOR_LEVEL	(2)

#define MAX17048_VCELL_REG	0x02
#define MAX17048_SOC_REG	0x04
#define MAX17048_MODE_REG	0x06
#define MAX17048_VER_REG	0x08
#define MAX17048_CONFIG_REG	0x0C
#define MAX17048_CMD_REG	0xFE

#define MAX17048_VCELL_MSB	0x02
#define MAX17048_VCELL_LSB	0x03
#define MAX17048_SOC_MSB	0x04
#define MAX17048_SOC_LSB	0x05
#define MAX17048_MODE_MSB	0x06
#define MAX17048_MODE_LSB	0x07
#define MAX17048_VER_MSB	0x08
#define MAX17048_VER_LSB	0x09
#define MAX17048_CONFIG_MSB	0x0C
#define MAX17048_CONFIG_LSB	0x0D
#define MAX17048_CMD_MSB	0xFE
#define MAX17048_CMD_LSB	0xFF

#define MAX17048_WORK_DELAY		(30 * HZ)	/* 30s */
#define MAX17048_BATTERY_FULL		95		/* Tuning Value */
#define MAX17048_TOLERANCE		10		/* Tuning Value */

#ifdef CONFIG_MACH_APQ8064_ALTEV
extern void bq24262_charger_force_update_batt_psy(void);
#else
extern void pm8921_charger_force_update_batt_psy(void);
#endif

struct max17048_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
#if 0 /* D1L does not use alert_work */
	struct work_struct		alert_work;
#endif
	struct power_supply		battery;

	/* Max17048 Registers.(Raw Data) */
	int vcell;				/* VCELL Register vaule */
	int soc;				/* SOC Register value */
	int version;			/* Max17048 Chip version */
	int config;				/* RCOMP, Sleep, ALRT, ATHD */

	/* Interface with Android */
	int voltage;		/* Battery Voltage   (Calculated from vcell) */
	int capacity;		/* Battery Capacity  (Calculated from soc) */
#ifdef CONFIG_MACH_APQ8064_ALTEV
	int orig_capacity;
#endif
	max17048_status status;	/* State Of max17048 */
/*                                                                                           */
	u8			starting_rcomp;
	int			temp_co_hot;
	int			temp_co_cold;
/*                                   */
/*                                                                                          */
	//struct max17048_ocv_to_soc_data	*cal_data;
/*                                   */
};

#if 0
/*
 * Voltage Calibration Data
 *   voltage = (capacity * gradient) + intercept
 *   voltage must be in +-15%
 */
struct max17048_calibration_data {
	int voltage;	/* voltage in mA */
	int capacity;	/* capacity in % */
	int gradient;	/* gradient * 1000 */
	int intercept;	/* intercept * 1000 */
};

/* 180mA Load for Battery */
static struct max17048_calibration_data without_charger[] = {
	{3953,		81,		9,		3242},
	{3800,		58,		7,		3403},
	{3740,		40,		3,		3611},
	{3695,		20,		2,		3650},
	{3601,		4,		6,		3574},
	{3300,		0,		55,		3548},
	{ -1, -1, -1, -1},	/* End of Data */
};
/* 770mA Charging Battery */
static struct max17048_calibration_data with_charger[] = {
	{3865,		2,		66,		3709},
	{3956,		19,		5,		3851},
	{4021,		46,		2,		3912},
	{4088,		61,		5,		3813},
	{4158,		71,		7,		3689},
	{4200,		100,		2,		4042},
	{ -1, -1, -1, -1},	/* End of Data */
};
#endif

/*                                                                           */
extern int lge_battery_info;
/*                                                                        */
static struct max17048_chip *reference;

/*                                                              */
extern int lge_power_test_flag;
/*                                    */

int need_to_quickstart;
EXPORT_SYMBOL(need_to_quickstart);
static int max17048_write_data(struct i2c_client *client, int reg, const u8 *values, int length)
{
		int ret;

		ret = i2c_smbus_write_i2c_block_data(client, reg, length, values);

		if (ret < 0)
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);

		return ret;
}

static int max17048_read_data(struct i2c_client *client, int reg, u8 *values, int length)
{
		int ret;

		ret = i2c_smbus_read_i2c_block_data(client, reg, length, values);

		if (ret < 0)
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);

		return ret;
}
static int max17048_write_reg(struct i2c_client *client, int reg, u16 value)
{
	int ret = 0;

	value = ((value & 0xFF00) >> 8) | ((value & 0xFF) << 8);
	ret = i2c_smbus_write_word_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}
static int max17048_read_reg(struct i2c_client *client, int reg)
{
	int ret = 0;

	ret = i2c_smbus_read_word_data(client, reg);

	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return ((ret & 0xFF00) >> 8) | ((ret & 0xFF) << 8);
}

static int max17048_reset(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
#ifdef CONFIG_LGE_PM
	/*                                                              */
	max17048_write_reg(client, MAX17048_MODE_REG, 0x4000);
#else
	max17048_write_reg(client, MAX17048_CMD_REG, 0x5400);
#endif
	chip->status = MAX17048_RESET;

	dev_info(&client->dev, "MAX17048 Fuel-Gauge Reset(quickstart)\n");

	return 0;
}

static int max17048_quickstart(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	max17048_write_reg(client, MAX17048_MODE_REG, 0x4000);

	chip->status = MAX17048_QUICKSTART;

	dev_info(&client->dev, "MAX17048 Fuel-Gauge Quick-Start\n");

	return 0;
}
static int max17048_read_vcell(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	u16 value;

	value = max17048_read_reg(client, MAX17048_VCELL_REG);

	if (value < 0)
		return value;

	chip->vcell = value >> 4;

	return 0;
}
static int max17048_read_soc(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	u16 value;

	value = max17048_read_reg(client, MAX17048_SOC_REG);
	if (value < 0)
		return value;

	chip->soc = value;

	return 0;
}

static int max17048_read_version(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	u16 value;

	value = max17048_read_reg(client, MAX17048_VER_REG);

	chip->version = value;

	dev_info(&client->dev, "MAX17048 Fuel-Gauge Ver %d\n", value);

	return 0;
}
static int max17048_read_config(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	u16 value;

	value = max17048_read_reg(client, MAX17048_CONFIG_REG);

	if (value < 0)
		return value;

	pr_info("Sleep-%d, ALSC-%d, ALRT-%d, ATHD-%d\n", value&0x80, value&0x40, value&0x20, value&0x1F);
	chip->config = value;

	return 0;
}
static int max17048_write_config(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	max17048_write_reg(client, MAX17048_CONFIG_REG, chip->config);

	return 0;
}

#ifdef CONFIG_LGE_PM
#ifdef CONFIG_MACH_APQ8064_ALTEV
/*
 * This function is used to get capacity of maxim custom fuel gauge model.
 */
#define EMPTY_SOC	0
static int max17048_get_capacity_from_soc(int *orig_soc)
{
	long batt_soc = 0;
	uint32_t buf[2];
	long adjust_soc = 0;

	if (reference == NULL) {
		return 100;
	}

	buf[0] = (reference->soc & 0x0000FF00) >> 8;
	buf[1] = (reference->soc & 0x000000FF);

/*                                                                 */
	batt_soc = ((buf[0] * 256) + buf[1]) * 19531; /* 0.001953125 */
	pr_info("%s: batt_soc is %d(0x%02x:0x%02x):%ld\n", __func__, (int)(batt_soc/10000000), buf[0], buf[1], batt_soc);

	*orig_soc = batt_soc / 10000000;

	adjust_soc = 9300000;  //GK, CMCC, Awifi

	batt_soc /= adjust_soc;
	pr_info("%s: adjust_soc = %ld, batt_soc = %ld orig = %d\n", __func__, adjust_soc, batt_soc, *orig_soc);

	if(batt_soc < 0)
		batt_soc = 0;

	if(batt_soc > 100)
		batt_soc = 100;

/*                                                               */
	return batt_soc;
}
#else
/*
 * This function is used to get capacity of maxim custom fuel gauge model.
 */
#define EMPTY_SOC	0
static int max17048_get_capacity_from_soc(void)
{
	long batt_soc = 0;
	uint32_t buf[2];
	long adjust_soc = 0;

	if (reference == NULL) {
		return 100;
	}

	buf[0] = (reference->soc & 0x0000FF00) >> 8;
	buf[1] = (reference->soc & 0x000000FF);

/*                                                                 */
	batt_soc = ((buf[0] * 256) + buf[1]) * 19531; /* 0.001953125 */
	pr_info("%s: batt_soc is %d(0x%02x:0x%02x):%ld\n", __func__, (int)(batt_soc/10000000), buf[0], buf[1], batt_soc);

#if defined(CONFIG_MACH_APQ8064_L05E)
	adjust_soc = 9700000;  //L05E
#else
	adjust_soc = 9300000;  //GK, CMCC
#endif

	batt_soc /= adjust_soc;
	pr_info("%s: adjust_soc = %ld, batt_soc = %ld\n", __func__, adjust_soc, batt_soc);

	if(batt_soc < 0)
		batt_soc = 0;

	if(batt_soc > 100)
		batt_soc = 100;

/*                                                               */
	return batt_soc;
}
#endif
#endif

static int max17048_need_quickstart(int charging)
{
#if 0
/*                                                              */
#ifdef CONFIG_BATTERY_MAX17048
	struct max17048_ocv_to_soc_data *data;
	int i = 0;
	int expected;
	int diff;
	int ratio_soc = 0;
	int ratio_vol = 0;
#else
	struct max17048_calibration_data *data;
#endif
	int vol;
	int level;

	if (reference == NULL)
		return 0;

	/* Get Current Data */
	vol = reference->voltage;
#ifdef CONFIG_BATTERY_MAX17048
	level = max17048_get_capacity_from_soc();

	if(level != 0){
		level = (level * 95) / 100;
	}
#else
	level = reference->soc >> 8;
#endif
	if (level > 100)
		level = 100;
	else if (level < 0)
		level = 0;

#ifdef CONFIG_BATTERY_MAX17048
	printk(KERN_INFO "[Battery] %s : Check SOC.\n",__func__);
#if 0
	data = reference->cal_data;

	if( data == NULL ){
		printk(KERN_INFO "[Battery] %s : NO DATA.\n",__func__);
		return 0;
	}

	while(data[i].voltage != -1){
		if(vol <= data[i].voltage)
			break;
		i++;
	}

	if(data[i].voltage == -1){
		if((level == 100) || (level > data[i-1].soc - MAX17048_TOLERANCE) || (need_to_quickstart > 2)){
			need_to_quickstart = 0;
			return 0;
		}else{
			need_to_quickstart += 1;
			return 1;
		}
	}
	else if(i == 0){
		need_to_quickstart = 0;
		return 0;
	}

	ratio_vol = (vol - data[i-1].voltage) * (data[i].soc - data[i-1].soc);
	ratio_soc = ratio_vol /	((data[i].voltage - data[i-1].voltage));

	expected = ratio_soc + data[i-1].soc;
#endif
#else
	/* choose data to use */
	if (charging) {
		data = with_charger;
		while (data[i].voltage != -1) {
			if (vol <= data[i].voltage)
				break;
			i++;
		}
	} else {
		data = without_charger;
		while (data[i].voltage != -1) {
			if (vol >= data[i].voltage)
				break;
			i++;
		}
	}

	/* absense of data */
	if (data[i].voltage == -1) {
		if (charging) {
			if (level == 100)
				return 0;
			else
				return 1;
		} else {
			if (level == 0)
				return 0;
			else
				return 1;
		}
	}

	/* calculate diff */
	expected = (vol - data[i].intercept) / data[i].gradient;
#endif
/*                          */

	if (expected > 100)
		expected = 100;
	else if (expected < 0)
		expected = 0;
	diff = expected - level;

	printk(KERN_INFO "[Battery] quickstart voltage(%d) base soc : expected(%d)/read(%d)\n",vol,expected,level);

	/* judge */
	if (diff < -MAX17048_TOLERANCE || diff > MAX17048_TOLERANCE)
		need_to_quickstart += 1;
	else
		need_to_quickstart = 0;

	/* Maximum continuous reset time is 2.
	 * If reset over 2 times, discard it.
	 */
	if (need_to_quickstart > 2)
		need_to_quickstart = 0;
#endif
	//return need_to_quickstart;
	return 0;
}
static int max17048_set_rcomp(int rcomp)
{
	if (reference == NULL)
		return -1;

	rcomp &= 0xff;
	reference->config = ((reference->config & 0x00ff) | (rcomp << 8));

	max17048_write_config(reference->client);

	return 0;
}
static int max17048_set_athd(int level)
{
	if (reference == NULL)
		return -1;

	if (level > 32)
		level = 32;
	else if (level < 1)
		level = 1;

	level = 32 - level;
	if (level == (reference->config & 0x1F))
		return level;

	reference->config = ((reference->config & 0xffe0) | level);
	max17048_write_config(reference->client);

	return level;
}
static int max17048_clear_interrupt(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	if (chip->config & 0x20) {
		chip->config &= 0xffdf;
		max17048_write_config(chip->client);
	}

	return 0;
}
static int max17048_update(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int ret;
#ifdef CONFIG_MACH_APQ8064_ALTEV
	int orig_cap;
#endif

/*                                                                                           */
	max17048_set_rcomp_by_temperature();
/*                                   */

#if 0 /*For Debug */
{
	u8 org_ocv_msb, org_ocv_lsb;
	u8 values[2];
	int batt_mv;

	/* Unlock Model Access */
	values[0] = 0x4A; values[1] = 0x57;
	max17048_write_data(client, 0x3E, &values[0], 2);

	/*Read OCV */
	max17048_read_data(client, 0x0E, &values[0], 2);
	org_ocv_msb = values[0]; org_ocv_lsb = values[1];
	batt_mv = ((values[0] << 4) + (values[1] >> 4));
	batt_mv = (batt_mv*125)/100;
	pr_info("%s : ocv is 0x%02x%02x - %d\n", __func__, org_ocv_msb, org_ocv_lsb, batt_mv);

	/*Lock Model Access */
	values[0] = 0x00; values[1] = 0x00;
	max17048_write_data(client, 0x3E, &values[0], 2);
}
#endif

	ret = max17048_read_vcell(client);
	if (ret < 0)
		return ret;
	ret = max17048_read_soc(client);
	if (ret < 0)
		return ret;

	/* convert raw data to usable data */
	chip->voltage = (chip->vcell * 5) >> 2;	/* vcell * 1.25 mV */
#ifdef CONFIG_LGE_PM
#ifdef CONFIG_MACH_APQ8064_ALTEV
	chip->capacity = max17048_get_capacity_from_soc(&orig_cap);
	chip->orig_capacity = orig_cap;
#else
	chip->capacity = max17048_get_capacity_from_soc();
#endif
#else
	chip->capacity = chip->soc >> 8;
#endif

	printk(KERN_INFO "%s: reg vcell %d, soc %d, calc volt %d, capa %d\n",
						__func__, chip->vcell, chip->soc, chip->voltage, chip->capacity);

	/* Adjust 100% Condition */
//	chip->capacity = (chip->capacity * 100) / MAX17048_BATTERY_FULL;

	if (chip->capacity > 100)
		chip->capacity = 100;
	else if (chip->capacity < 0)
		chip->capacity = 0;

	chip->status = MAX17048_WORKING;

	return 0;
}

static void max17048_work(struct work_struct *work)
{
	struct max17048_chip *chip;
#ifdef CONFIG_MACH_APQ8064_ALTEV
	int charging = 0;
#else
	int charging = 0, source = 0;
#endif

	chip = container_of(work, struct max17048_chip, work.work);

#ifdef CONFIG_MACH_APQ8064_ALTEV
	charging = bq24262_prop_is_charging();
#else
	charging = pm8921_is_battery_charging(&source);
#endif
	printk(KERN_INFO "%s: chip->status %d, charging %d\n", __func__, chip->status, charging);
	switch (chip->status) {
	case MAX17048_RESET:
		max17048_read_version(chip->client);
		max17048_read_config(chip->client);
		max17048_set_rcomp(chip->starting_rcomp);
	case MAX17048_QUICKSTART:
		max17048_update(chip->client);
		if (max17048_need_quickstart(charging)) {
			max17048_reset(chip->client);
			schedule_delayed_work(&chip->work,  HZ);
			return;
		}
		max17048_set_athd(MONITOR_LEVEL);
		max17048_clear_interrupt(chip->client);
		need_to_quickstart = 0;
		break;
	case MAX17048_WORKING:
	default:
		max17048_update(chip->client);
		break;
	}
	schedule_delayed_work(&chip->work, MAX17048_WORK_DELAY);

	printk(KERN_INFO "[Battery] max17048_work : soc(%d), volt(%d)\n",
				chip->capacity, chip->voltage);

}

#if 0 /* D1L does not use alert_work */
static void max17048_alert_work(struct work_struct *work)
{
	if (reference == NULL)
		return;

	if (reference->status == MAX17048_WORKING) {
		cancel_delayed_work_sync(&reference->work);
		schedule_delayed_work(&reference->work, MAX17048_WORK_DELAY);
	}
	max17048_update(reference->client);

	max17048_read_config(reference->client);
	max17048_clear_interrupt(reference->client);

	if (!pm8921_is_dc_chg_plugged_in() && !pm8921_is_usb_chg_plugged_in())
}
#endif

static irqreturn_t max17048_interrupt_handler(int irq, void *data)
{
#if 0 /* D1L does not use alert_work */
	if (reference == NULL)
		return IRQ_HANDLED;
	schedule_work(&reference->alert_work);
#endif
#ifdef CONFIG_MACH_APQ8064_ALTEV
	bq24262_charger_force_update_batt_psy();
#endif
	//pm8921_charger_force_update_batt_psy();
	max17048_clear_interrupt(reference->client);
	return IRQ_HANDLED;
}

/*                                                              */
void max17048_power_quickstart(void)
{
	int charging = 0;
#ifndef CONFIG_MACH_APQ8064_ALTEV
	int source = 0;
#endif

	pr_err("[mansu.lee] %s.\n",__func__);

#ifdef CONFIG_MACH_APQ8064_ALTEV
	charging = bq24262_prop_is_charging();
#else
	charging = pm8921_is_battery_charging(&source);
#endif


	max17048_quickstart(reference->client);
	/* best effort delay time for read new soc */
	msleep(300);

	max17048_update(reference->client);

	do{
		max17048_need_quickstart(charging);
	}while(need_to_quickstart != 0);
}
/*                          */

ssize_t max17048_show_volt(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	int voltage;
	if (reference == NULL)
		return snprintf(buf, PAGE_SIZE, "ERROR\n");
#ifdef CONFIG_MACH_APQ8064_ALTEV
	/*                                                              */
	if(lge_power_test_flag == 1){
		cancel_delayed_work(&reference->work);

//		bq24262_charger_enable(0);
//		pm8921_disable_source_current(1);

		max17048_power_quickstart();
		voltage = ((reference->vcell * 5) >> 2);

//		pm8921_disable_source_current(0);
//		bq24262_charger_enable(1);

		schedule_delayed_work(&reference->work, HZ);

		return snprintf(buf, PAGE_SIZE, "%d\n", voltage);
	}
	/*                          */
#else
#ifdef CONFIG_BATTERY_MAX17048
	/*                                                              */
	if(lge_power_test_flag == 1){
		cancel_delayed_work(&reference->work);

		pm8921_charger_enable(0);
		pm8921_disable_source_current(1);

		max17048_power_quickstart();
		voltage = ((reference->vcell * 5) >> 2);

		pm8921_disable_source_current(0);
		pm8921_charger_enable(1);

		schedule_delayed_work(&reference->work, HZ);

		return snprintf(buf, PAGE_SIZE, "%d\n", voltage);
	}
	/*                          */
#endif
#endif
	return snprintf(buf, PAGE_SIZE, "%d\n", (reference->vcell * 5) >> 2);
}
DEVICE_ATTR(volt, 0444, max17048_show_volt, NULL);

ssize_t max17048_show_version(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	if (reference == NULL)
		return snprintf(buf, PAGE_SIZE, "ERROR\n");

	return snprintf(buf, PAGE_SIZE, "%d\n", reference->version);
}
DEVICE_ATTR(version, 0444, max17048_show_version, NULL);

ssize_t max17048_show_config(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	if (reference == NULL)
		return snprintf(buf, PAGE_SIZE, "ERROR\n");

	return snprintf(buf, PAGE_SIZE, "%d\n", reference->config);
}
DEVICE_ATTR(config, 0444, max17048_show_config, NULL);

ssize_t max17048_show_soc(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	int level;
#ifdef CONFIG_MACH_APQ8064_ALTEV
	int orig_cap;
#endif

	if (reference == NULL)
		return snprintf(buf, PAGE_SIZE, "ERROR\n");
#ifdef CONFIG_MACH_APQ8064_ALTEV
	/*                                                              */
	if(lge_power_test_flag == 1){
		cancel_delayed_work(&reference->work);

//		bq24262_charger_enable(0);
//		pm8921_disable_source_current(1);

		max17048_power_quickstart();
		level = max17048_get_capacity_from_soc(&orig_cap);

//		pm8921_disable_source_current(0);
//		bq24262_charger_enable(1);

		schedule_delayed_work(&reference->work, HZ);

		if(level != 0){
			if(level >= 99)
				level = (level * 915) /1000;
			else
				level = (level * 100) /100;
		}

		if (level > 100)
			level = 100;
		else if (level < 0)
			level = 0;

		return snprintf(buf, PAGE_SIZE, "%d\n", level);
	}
	/*                                                              */
#else
#ifdef CONFIG_BATTERY_MAX17048
	/*                                                              */
	if(lge_power_test_flag == 1){
		cancel_delayed_work(&reference->work);

		pm8921_charger_enable(0);
		pm8921_disable_source_current(1);

		max17048_power_quickstart();
		level = max17048_get_capacity_from_soc();

		pm8921_disable_source_current(0);
		pm8921_charger_enable(1);

		schedule_delayed_work(&reference->work, HZ);

		if(level != 0){
			if(level >= 99)
				level = (level * 915) /1000;
			else
				level = (level * 100) /100;
		}

		if (level > 100)
			level = 100;
		else if (level < 0)
			level = 0;

		return snprintf(buf, PAGE_SIZE, "%d\n", level);
	}
	/*                                                              */
#endif
#endif
	/*                                                             */
	/* accordig to battery SOC calculate method change. */

	level = reference->capacity;
	/*                        */

	return snprintf(buf, PAGE_SIZE, "%d\n", level);
}
DEVICE_ATTR(soc, 0444, max17048_show_soc, NULL);

ssize_t max17048_show_status(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	if (reference == NULL)
		return snprintf(buf, PAGE_SIZE, "ERROR\n");

	switch (reference->status) {
	case MAX17048_RESET:
		return snprintf(buf, PAGE_SIZE, "reset\n");
	case MAX17048_QUICKSTART:
		return snprintf(buf, PAGE_SIZE, "quickstart\n");
	case MAX17048_WORKING:
		return snprintf(buf, PAGE_SIZE, "working\n");
	default:
		return snprintf(buf, PAGE_SIZE, "ERROR\n");
	}
}
ssize_t max17048_store_status(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t count)
{
	if (reference == NULL)
		return -1;

	if (strncmp(buf, "reset", 5) == 0) {
		cancel_delayed_work(&reference->work);
		max17048_reset(reference->client);
		schedule_delayed_work(&reference->work, HZ);
	} else if (strncmp(buf, "quickstart", 10) == 0) {
		cancel_delayed_work(&reference->work);
		max17048_quickstart(reference->client);
		schedule_delayed_work(&reference->work, HZ);
	} else if (strncmp(buf, "working", 7) == 0) {
		/* do nothing */
	} else {
		return -1;
	}
	return count;
}
DEVICE_ATTR(state, 0664, max17048_show_status, max17048_store_status);
/* sysfs interface : for AT Commands [END] */

#ifdef CONFIG_MACH_APQ8064_ALTEV
/* SYMBOLS to use outside of this module */
int __max17048_get_orig_capacity(void)
{
	if (reference == NULL)	/* if fuel gauge is not initialized, */
		return 100;			/* return Dummy Value */

		return reference->orig_capacity;
}
EXPORT_SYMBOL(__max17048_get_orig_capacity);
#endif


/* SYMBOLS to use outside of this module */
int __max17048_get_capacity(void)
{
	if (reference == NULL)	/* if fuel gauge is not initialized, */
		return 100;			/* return Dummy Value */
	return reference->capacity;
}
EXPORT_SYMBOL(__max17048_get_capacity);
int __max17048_get_voltage(void)
{
	if (reference == NULL)	/* if fuel gauge is not initialized, */
		return 4350;		/* return Dummy Value */
	return reference->voltage;
}
EXPORT_SYMBOL(__max17048_get_voltage);
int max17048_do_calibrate(void)
{
	if (reference == NULL)
		return -1;

	cancel_delayed_work(&reference->work);
	max17048_quickstart(reference->client);
	schedule_delayed_work(&reference->work, HZ);

	return 0;
}
EXPORT_SYMBOL(max17048_do_calibrate);

/*                                                                                           */
int max17048_set_rcomp_by_temperature(void)
{
	u8 startingRcomp = reference->starting_rcomp;
	int PreviousRcomp = 0;
	int tempCoHot = reference->temp_co_hot;		/* -775*/
	int tempCoCold = reference->temp_co_cold;	/* -710 */
	// D1lA --END:
	int newRcomp;
	int temp;
	int ret;
	struct pm8xxx_adc_chan_result result;

	if(reference == NULL)
	{
		pr_err("error : reference pointer is NULL!!");
		return -1;
	}
	ret = pm8xxx_adc_read(CHANNEL_BATT_THERM, &result);
	if (ret)
	{
		pr_err("error reading adc channel = %d, ret = %d\n",
		CHANNEL_BATT_THERM, ret);
		return ret;
	}
	temp = result.physical;
	temp /= 10;

	PreviousRcomp = max17048_read_reg(reference->client, MAX17048_CONFIG_REG);
	PreviousRcomp = (PreviousRcomp & 0xFF00) >> 8;
	if (PreviousRcomp < 0)
		return PreviousRcomp;

	pr_info(" max17048 check temp = %d, PreviousRcomp =0x%02X\n", temp, PreviousRcomp);

	if (temp > 20)
		newRcomp = startingRcomp + (int)((temp - 20)*tempCoHot/1000);
	else if (temp < 20){
		newRcomp = startingRcomp + (int)((temp - 20)*tempCoCold/100);
		}
	else
		newRcomp = startingRcomp;

	if (newRcomp > 0xFF)
		newRcomp = 0xFF;
	else if (newRcomp < 0)
		newRcomp = 0;

	if (newRcomp != PreviousRcomp)
	{
		pr_info("RCOMP: new rcomp is 0x%02X(0x%02X)\n", newRcomp, startingRcomp);
		max17048_set_rcomp(newRcomp);
	}
	return 0;
}
/*                                   */

EXPORT_SYMBOL(max17048_set_rcomp_by_temperature);
int max17048_set_alert_level(int alert_level)
{
	return max17048_set_athd(alert_level);
}
EXPORT_SYMBOL(max17048_set_alert_level);
/* End SYMBOLS */
/*                                                                           */
int max17048_set_operation(void)
{
	int ret = ENABLE_MAX17048_WORK;

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	if (lge_battery_info == BATT_ID_DS2704_N || lge_battery_info == BATT_ID_DS2704_L ||
		lge_battery_info == BATT_ID_ISL6296_N || lge_battery_info == BATT_ID_ISL6296_L ||
		lge_battery_info == BATT_ID_DS2704_C || lge_battery_info == BATT_ID_ISL6296_C)
	{
		ret = ENABLE_MAX17048_WORK;
	}
	else
	{
		ret = DISABLE_MAX17048_WORK;
	}
#else

	/* d1l korea will use battery id checker soon. It shuould be changed */
	/*
	int rc;

	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(CHANNEL_BATT_ID, &result);

	pr_info("kwon -  pm8xxx_adc_read rc = %d, result.physical = %lld\n",
		rc, result.physical);

	if (rc < 0) {
		pr_err("error reading batt id : rc = %d, result.physical = %lld\n",
					rc, result.physical);
		ret = DISABLE_MAX17048_WORK;
	}
	else
	{
		fuel_gauge_work = ENABLE_MAX17048_WORK;
	}
	//if (rc < chip->batt_id_min || rc > chip->batt_id_max) {
		//pr_err("batt_id phy =%lld is not valid\n", rc);
		//ret = DISABLE_MAX17048_WORK;
	*/

	ret = ENABLE_MAX17048_WORK;

#endif
	return ret;

}
/*                                                                        */

static int max17048_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
/*                                                                                           */
	struct max17048_platform_data *pdata = client->dev.platform_data;
/*                                   */
	struct max17048_chip *chip;
	int ret = 0;
	u8 values[4];

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		printk(KERN_ERR " [MAX17048] i2c_check_functionality fail\n");
		/*return -EIO;*/
	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		printk(KERN_ERR " [MAX17048] kzalloc fail\n");
		return -ENOMEM;
	}

	ret = gpio_request(client->irq, "max17048_alert");
	if (ret < 0) {
		printk(KERN_ERR " [MAX17048] GPIO Request Failed\n");
		goto err_gpio_request_failed;
	}
	gpio_direction_input(client->irq);

	ret = request_irq(gpio_to_irq(client->irq),
			max17048_interrupt_handler,
			IRQF_TRIGGER_FALLING,
			"MAX17048_Alert", NULL);
	if (ret < 0) {
		printk(KERN_ERR " [MAX17048] IRQ Request Failed\n");
		goto err_request_irq_failed;
	}

	ret = enable_irq_wake(gpio_to_irq(client->irq));
	if (ret < 0) {
		printk(KERN_ERR "[MAX17048] set irq to wakeup source failed.\n");
		goto err_request_wakeup_irq_failed;
	}

	chip->client = client;

	i2c_set_clientdata(client, chip);

#if 1
{
	u8 org_ocv_msb, org_ocv_lsb;
	u8 org_rcomp_msb, org_rcomp_lsb;
	u8 values[2];
	int batt_mv;
	long batt_soc = 0;

	/* Unlock Model Access */
	values[0] = 0x4A; values[1] = 0x57;
	max17048_write_data(client, 0x3E, &values[0], 2);

	/*Read OCV */
	max17048_read_data(client, 0x0E, &values[0], 2);
	org_ocv_msb = values[0]; org_ocv_lsb = values[1];
	batt_mv = ((values[0] << 4) + (values[1] >> 4));
	batt_mv = (batt_mv*125)/100;
	pr_info("%s : Read OCV 0x%02x%02x - %d\n", __func__, org_ocv_msb, org_ocv_lsb, batt_mv);

	/* Read SOC */
	max17048_read_soc(client);
	values[0] = (chip->soc & 0x0000FF00) >> 8;
	values[1] = (chip->soc & 0x000000FF);

	batt_soc = ((values[0]*256)+values[1])*19531;
	pr_info("%s: Read SOC %d(0x%02x:0x%02x):%ld\n", __func__, (int)(batt_soc/10000000), values[0], values[1], batt_soc);

	/*Read RComp */
	max17048_read_data(client, MAX17048_CONFIG_REG, &values[0], 2);
	org_rcomp_msb = values[0]; org_rcomp_lsb = values[1];
	pr_info("%s : Read RCOMP = 0x%02x%02x \n", __func__, org_rcomp_msb, org_rcomp_lsb);

	/*Lock Model Access */
	values[0] = 0x00; values[1] = 0x00;
	max17048_write_data(client, 0x3E, &values[0], 2);
}
#endif

	/* sysfs path : /sys/devices/platform/i2c_omap.2/i2c-2/2-0036/soc */
	ret = device_create_file(&client->dev, &dev_attr_soc);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_soc_failed;
	}
	/* sysfs path : /sys/devices/platform/i2c_omap.2/i2c-2/2-0036/state */
	ret = device_create_file(&client->dev, &dev_attr_state);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_state_failed;
	}
	ret = device_create_file(&client->dev, &dev_attr_volt);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_soc_failed;
	}
	ret = device_create_file(&client->dev, &dev_attr_config);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_state_failed;
	}
	ret = device_create_file(&client->dev, &dev_attr_version);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_state_failed;
	}
	chip->vcell = 3480;
	chip->soc = 100 << 8;
	chip->voltage = 4350;
	chip->capacity = 100;
	chip->config = 0x971C;
/*                                                                                           */
	chip->starting_rcomp = pdata->starting_rcomp;
	chip->temp_co_hot = pdata->temp_co_hot;
	chip->temp_co_cold = pdata->temp_co_cold;
/*                                   */

/*                                                                                     */
	//chip->cal_data = pdata->soc_cal_data;
/*                                   */

	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17048_work);
#if 0 /* D1L does not use alert_work */
	INIT_WORK(&chip->alert_work, max17048_alert_work);
#endif

	reference = chip;
	max17048_read_version(client);
	max17048_read_config(client);
	max17048_set_rcomp(chip->starting_rcomp);
	max17048_set_athd(MONITOR_LEVEL);
	max17048_clear_interrupt(client);
	//Set VRESET
	values[0] = 0x82; values[1] = 0x00;
	max17048_write_data(client, 0x18, &values[0], 2);
	// RCOMP and Low Battery SOC setting
	//max17048_write_reg(client, 0x0C, 0x47);

/*                                                                           */
	ret = max17048_set_operation();
	if(!ret)
	{
		pr_err("%s: battery is not present : %d\n", __func__, ret);
		return ret;
	}
/*                                                                        */

	if (need_to_quickstart == -1) {
		max17048_quickstart(client);
		need_to_quickstart = 0;
		schedule_delayed_work(&chip->work, HZ);
		return 0;
	} else {
		max17048_update(client);
		schedule_delayed_work(&chip->work, MAX17048_WORK_DELAY);
	}

	return 0;

err_create_file_state_failed:
	device_remove_file(&client->dev, &dev_attr_soc);
err_create_file_soc_failed:
#if 0	/* B-Project. Does not use fuel gauge as a battery driver */
err_power_supply_register_failed:
	i2c_set_clientdata(client, NULL);
#endif
	kfree(chip);
	disable_irq_wake(gpio_to_irq(client->irq));
err_request_wakeup_irq_failed:
	free_irq(gpio_to_irq(client->irq), NULL);
err_request_irq_failed:
	gpio_free(client->irq);
err_gpio_request_failed:

	return ret;
}

static int __devexit max17048_remove(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->battery);
	cancel_delayed_work(&chip->work);
#if 0 /* D1L does not use alert_work */
	flush_work(&chip->alert_work);
#endif
	i2c_set_clientdata(client, NULL);
	pr_info("max17048_remove -  kfree(chip) !!");
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int max17048_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chip->work);
#if 0 /* D1L does not use alert_work */
	flush_work(&chip->alert_work);
#endif
	client->dev.power.power_state = state;

	return 0;
}

static int max17048_resume(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	printk("%s\n", __func__);
/*                                                                           */
	ret = max17048_set_operation();
	if(!ret)
	{
		pr_err("%s: battery is not present or not valid battery : %d\n", __func__, ret);
		return ret;
	}
/*                                                                        */

	schedule_delayed_work(&chip->work, HZ/2);
	client->dev.power.power_state = PMSG_ON;
	/*                                                        */
	max17048_read_config(client);
	max17048_clear_interrupt(client);
	return 0;
}
#else
#define max17048_suspend NULL
#define max17048_resume NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id max17048_id[] = {
	{ "max17048", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17048_id);

static struct i2c_driver max17048_i2c_driver = {
	.driver	= {
		.name	= "max17048",
		.owner	= THIS_MODULE,
	},
	.probe		= max17048_probe,
	.remove		= __devexit_p(max17048_remove),
	.suspend	= max17048_suspend,
	.resume		= max17048_resume,
	.id_table	= max17048_id,
};

/* boot argument from boot loader */
static s32 __init max17048_state(char *str)
{
	switch (str[0]) {
	case 'g':	/* fuel gauge value is good */
	case 'q':	/* did quikcstart. */
		need_to_quickstart = 0;
		break;
	case 'b':	/* battery not connected when booting */
		need_to_quickstart = 1;
		break;
	case 'e':	/* quickstart needed. but error occured. */
		need_to_quickstart = -1;
		break;
	default:
		/* can not enter here */
		break;
	}
	return 0;
}
__setup("fuelgauge=", max17048_state);
#if 0
static int __init max17048_init(void)
{
	return i2c_add_driver(&max17048_i2c_driver);
}
module_init(max17048_init);

static void __exit max17048_exit(void)
{
	i2c_del_driver(&max17048_i2c_driver);
}
module_exit(max17048_exit);
#endif
module_i2c_driver(max17048_i2c_driver);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("MAX17048 Fuel Gauge");
MODULE_LICENSE("GPL");
