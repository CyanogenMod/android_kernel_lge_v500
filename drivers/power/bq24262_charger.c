/* Copyright (c) 2013 LGE Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/bitops.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/reboot.h>
#include <linux/max17048_fuelgauge.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/usb/otg.h>
#include <linux/power/bq24262_charger.h>
#include <mach/board_lge.h>

#include "../../arch/arm/mach-msm/smd_private.h"
#include "../usb/dwc3/dwc3_otg.h"
#include "../usb/dwc3/core.h"
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <mach/rpm-regulator.h>
#include <linux/slimport.h>

#ifdef CONFIG_LGE_DOCK
#include <linux/switch.h>

enum {
	EXTRA_DOCK_STATE_UNDOCKED = 0,
	EXTRA_DOCK_STATE_DESK = 1,
	EXTRA_DOCK_STATE_CAR = 2,
	EXTRA_DOCK_STATE_LE_DESK = 3,
	EXTRA_DOCK_STATE_HE_DESK = 4
#if defined(CONFIG_MACH_APQ8064_AWIFI070U)
	,
	EXTRA_DOCK_STATE_070_DESK = 5
#endif
};
struct switch_dev dockdev;
#endif

#ifdef CONFIG_MACH_APQ8064_ALTEV
int vzw_fast_chg_ma = 0;
EXPORT_SYMBOL(vzw_fast_chg_ma);
extern bool usb_connected_flag;
enum {
	DPM_CHECK_START = 0,
	DPM_CHECK_FIRST = 1,
	DPM_CHECK_SECOND = 2,
	DPM_CHECK_DONE = 3,
};
static int dpm_count;
#endif

//                                       
//                                       
//#define MONITOR_BATTEMP_POLLING_PERIOD          (10*HZ)
#define MONITOR_BATTEMP_POLLING_PERIOD          (60*1000)

//#endif

#ifndef BIT
#define BIT(x)	(1 << (x))
#endif

#ifdef CONFIG_MACH_APQ8064_ALTEV
#define INPUT_CURRENT_LIMIT_MAX_MA	1500
#else
#define INPUT_CURRENT_LIMIT_MAX_MA	3000
#endif

/* Register definitions */
#define R00_STATUS_CONTROL_REG	          	0x00
#define R01_CONTROL_REG			0x01
#define R02_CONTROL_BAT_VOL_REG		0x02
#define R03_VENDER_PART_REV_REG		0x03
#define R04_BAT_TERM_FAST_CHARGE_CUR_REG	0x04
#define R05_VINDPM_VOL_DPPM_STAT_REG		0x05
#define R06_SAFETY_TMR_NTC_MON_REG		0x06

/* REG00 Status Control Register */
#define TMR_RST                 BIT(7)
#define BOOST_MODE_MASK         BIT(6)
#define CHRG_STAT_MASK          (BIT(5)|BIT(4))
#define EN_SHIPMODE             BIT(3)
#define CHG_FAULT_MASK          (BIT(2)|BIT(1)|BIT(0))

/* REG01 Control Register */
#define IINLIM_MASK             (BIT(7)|BIT(6)|BIT(5)|BIT(4))
#define EN_STAT                 (BIT(7)|BIT(3))
#define EN_CHG_TERM_MASK        (BIT(7)|BIT(2))
#define CHG_CONFIG_MASK         (BIT(7)|BIT(1))
#define HZ_MODE                 (BIT(7)|BIT(0))
//#define ALL_REG_CLEAR      BIT(7)

/* REG02 Control Battery Voltage Register */
#define VBREG_MASK              (BIT(7)|BIT(6)|BIT(5)|BIT(4)|BIT(3)|BIT(2))
#define MOD_FREQ                (BIT(1)|BIT(0))


/* REG03 Control INFO Register */
#define VENDER                  (BIT(7)|BIT(6)|BIT(5))
#define PART_NUMBER             (BIT(4)|BIT(3))
#define REVISION                (BIT(2)|BIT(1)|BIT(0))


/* REG04 Battery Termination Fast Charge Current Register */
#define ICHG_MASK               (BIT(7)|BIT(6)|BIT(5)|BIT(4)|BIT(3))
#define ITERM_MASK              (BIT(2)|BIT(1)|BIT(0))


/* REG05 VINDPM Voltage DPPM Status Register */
#define MINSYS_STATUS           BIT(7)
#define DPM_STATUS              BIT(6)
#define LOW_CHG                 BIT(5)
#define DPDM_EN                 BIT(4)
#define CD_STATUS               BIT(3)
#define VINDPM_MASK             (BIT(2)|BIT(1)|BIT(0))


/* REG06 Safety Timer/ NTC Monitor Register */
#define XTMR_EN                 BIT(7)
#define TMR_BIT                 (BIT(6)|BIT(5))
#define BOOST_ILIM              BIT(4)
#define TS_EN                   BIT(3)
#define TS_FAULT                (BIT(2)|BIT(1))
#define BINDPM_OFF              BIT(0)

#if 0
/* BQ05 Charge Termination, Timer-Control Register MASK */
#define I2C_TIMER_MASK          (BIT(5)|BIT(4))
#define EN_CHG_TIMER_MASK	BIT(3)
#define CHG_TIMER_MASK 		(BIT(2)|BIT(1))

/* BQ06 IR Compensation, Thermal Regulation Control Register MASK */
#define IR_COMP_R_MASK		(BIT(7)|BIT(6)|BIT(5))
#define IR_COMP_VCLAMP_MASK 	(BIT(4)|BIT(3)|BIT(2))

/* BQ07 Misc-Operation Control Register MASK */
#define BATFET_DISABLE_MASK 	BIT(5)

/* BQ08 SYSTEM_STATUS_REG Mask */
#define VBUS_STAT_MASK 		(BIT(7)|BIT(6))
#define PRE_CHARGE_MASK 	BIT(4)
#define FAST_CHARGE_MASK 	BIT(5)
#define DPM_STAT_MASK		BIT(3)
#define PG_STAT_MASK		BIT(2)
#define THERM_STAT_MASK 	BIT(1)
#define VSYS_STAT_MASK 		BIT(0)

/* BQ09 FAULT_REG Mask */
#define CHRG_FAULT_MASK 	(BIT(5)|BIT(4))
#endif

#define LEVEL_CH_UV_TO_MV 1000

#define LT_CABLE_56K		6
#define LT_CABLE_130K		7
#define LT_CABLE_910K		11


#define INPUT_CURRENT_LIMIT_100mA	100
#define INPUT_CURRENT_LIMIT_150mA	150
#define INPUT_CURRENT_LIMIT_500mA	500
#define INPUT_CURRENT_LIMIT_900mA	900
#define INPUT_CURRENT_LIMIT_1500mA	1500
#define INPUT_CURRENT_LIMIT_2500mA	2500


/* VINDPM:: 4200mV ~ 4788mV, 84mV step */
#define INPUT_VOLTAGE_LIMIT 	4620


//#define REGISGER_DUMP

enum bq24262_chg_status {
	BQ_CHG_STATUS_NONE 		= 0,
	BQ_CHG_STATUS_PRE_CHARGE	= 1,
	BQ_CHG_STATUS_FAST_CHARGE 	= 2,
	BQ_CHG_STATUS_EXCEPTION		= 3,
};

static const char * const bq24262_chg_status[] = {
	"none",
	"pre-charge",
	"fast-charge",
	"exception"
};

static const char * const chg_state_str[] = {
	"Ready",
	"Charge in progress",
	"Charge done",
	"Fault"
};

enum bq24262_chg_fault_state {
	BQ_FAULT_NORMAL,
	BQ_FAULT_VIN_OVP,
	BQ_FAULT_LOW_SUPPLY,
	BQ_FAULT_THERMAL_SHUTDOWN,
	BQ_FAULT_BATT_TEMP,
	BQ_FAULT_TIMER,
	BQ_FAULT_BATT_OVP,
	BQ_FAULT_NO_BATT,
};

static const char * const fault_str[] = {
	"Normal",
	"Boost Mode OVP",
	"Low Supply or Boost Mode Over",
	"Thermal Shutdown"
	"Battery Temp Fault"
	"Timer Fault Watchdoc"
	"Battery OVP"
	"No Battery"
};

struct bq24262_chip {
	struct i2c_client  *client;
	struct dentry  *dent;

	int  chg_current_ma;
	int  term_current_ma;
	int  int_gpio;
	int  irq;

	int old_ac_present;
	int host_mode;
	int usb_online;
	int ac_present;
	int ac_online;
	int chg_type;
	int charging_disabled;
	int full_design;
	bool chg_timeout;

	enum bq24262_chg_status	chg_status;
   	enum bq24262_chg_status	chg_cable_status;
	enum bq24262_chg_fault_state fault_state;

	struct delayed_work  irq_work;
	struct delayed_work  vbat_work;
	struct delayed_work  update_heartbeat_work;
	struct delayed_work  dpm_detect_work;
	struct wake_lock        monitor_batt_temp_wake_lock;
	struct wake_lock	    eoc_wake_lock;
	struct wake_lock        uevent_wake_lock;

	struct delayed_work	battemp_work;
	struct wake_lock		lcs_wake_lock;
	int set_chg_step;
//	int pseudo_ui_chg;
	int temp_level_1;
	int temp_level_2;
	int temp_level_3;
	int temp_level_4;
	int temp_level_5;
#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL
	int chg_current_te;
#endif
	struct wake_lock  chg_wake_lock;
	struct power_supply  *usb_psy;
	struct power_supply  ac_psy;
	struct power_supply  batt_psy;

#ifdef CONFIG_THERMAL_QPNP_ADC_TM
	struct qpnp_adc_tm_btm_param  adc_param;
#endif
	int  set_chg_current_ma;
	int  vbat_noti_stat;
	int  step_dwn_thr_mv;
	int  step_dwn_currnet_ma;
	unsigned int *thermal_mitigation;
	int thermal_levels;
	int max_bat_chg_current;
//	int eoc_check_count;
	int regulation_mV;

};

#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
int last_batt_temp;
int last_batt_current;
#endif

static struct bq24262_chip *the_chip;

struct pseudo_batt_info_type pseudo_batt_info_new = {
	.mode = 0,
};

struct debug_reg {
	char  *name;
	u8  reg;
};
#define FACTORY_CABLE_CAPACITY 75
#define FACTORY_CABLE_VOLTAGE 3850
#define FACTORY_CABLE_TEMP        400

#define BQ24262_DEBUG_REG(x) {#x, x##_REG}

static struct debug_reg bq24262_debug_regs[] = {
	BQ24262_DEBUG_REG(R00_STATUS_CONTROL),
	BQ24262_DEBUG_REG(R01_CONTROL),
	BQ24262_DEBUG_REG(R02_CONTROL_BAT_VOL),
	BQ24262_DEBUG_REG(R03_VENDER_PART_REV),
	BQ24262_DEBUG_REG(R04_BAT_TERM_FAST_CHARGE_CUR),
	BQ24262_DEBUG_REG(R05_VINDPM_VOL_DPPM_STAT),
	BQ24262_DEBUG_REG(R06_SAFETY_TMR_NTC_MON),
};

static unsigned int last_stop_charging;
static unsigned int chg_batt_temp_state;
static int pseudo_ui_charging;
static int dcreasing_charging;
static int last_usb_chg_current;
static int thermal_mitigation;
static int pre_mitigation;
static int soc_limit;
static int reg_vol_set_count;
static int dpm_retry_count;
static int batt_low_cnt;
//unsigned int set_default;

#ifdef CONFIG_LGE_PM_LOW_BATT_CHG
/* this is indicator that chargerlogo app is running. */
extern int chargerlogo_state;
#endif

#ifdef CONFIG_MACH_APQ8064_ALTEV
enum vzw_chg_state {
	VZW_COMPATIBLE_CHG =0,
//	VZW_SLOW_CHG = 1,
	VZW_INCOMPATIBLE_CHG=2,
	VZW_UNDER_CURRENT_CHG=3,
	VZW_TA_DETECTING=6,
};
static enum vzw_chg_state chg_state = VZW_COMPATIBLE_CHG;
#endif

static int block_charging_state = 1; /* 1 : charing, 0 : block charging */
int lge_power_test_flag = 0;

static int bq24262_enable_charging(struct bq24262_chip *chip, bool enable);
static int bq24262_set_hz_mode(struct bq24262_chip *chip, bool enable);
static int bq24262_get_hz_mode(struct bq24262_chip *chip);
static void bq24262_charging_setting(struct bq24262_chip *chip);
//static int bq24262_is_eoc_state(struct bq24262_chip * chip);
void block_charging_set(int block);
static int bq24262_get_prop_batt_present(struct bq24262_chip *chip);
static int bq24262_get_prop_charge_type(struct bq24262_chip *chip);
static int bq24262_set_input_i_limit(struct bq24262_chip *chip, int ma);
static int bq24262_get_input_i_limit(struct bq24262_chip *chip);
static bool bq24262_is_charger_present(struct bq24262_chip *chip);
static bool bq24262_is_otg_mode(struct bq24262_chip *chip);
static int bq24262_set_ibat_max(struct bq24262_chip *chip, int ma);
static int bq24262_enable_otg(struct bq24262_chip *chip, bool enable);
static int bq24262_set_vbat_max(struct bq24262_chip *chip, int mv);
static int bq24262_get_dpm_state(struct bq24262_chip * chip);
static void bq24262_set_clear_reg(struct bq24262_chip *chip);
//static int set_register_to_default(struct bq24262_chip *chip, bool enable);

#ifdef CONFIG_MACH_APQ8064_ALTEV
extern void write_high_temp_power_off(void);
extern int read_high_temp_power_off(char *filename);
#endif

static unsigned int cable_type;
static bool is_factory_cable(void)
{
	unsigned int cable_info;
	cable_info = lge_pm_get_cable_type();

	if ((cable_info == CABLE_56K ||
		cable_info == CABLE_130K ||
		cable_info == CABLE_910K) ||
		(cable_type == LT_CABLE_56K ||
		cable_type == LT_CABLE_130K ||
		cable_type == LT_CABLE_910K))
		return true;
	else
		return false;
}

static bool is_factory_cable_56k(void)
{
	unsigned int cable_info;
	cable_info = lge_pm_get_cable_type();

	if (cable_info == CABLE_56K ||
		cable_type == LT_CABLE_56K)
		return true;
	else
		return false;
}

static bool is_factory_cable_130k(void)
{
	unsigned int cable_info;
	cable_info = lge_pm_get_cable_type();

	if (cable_info == CABLE_130K ||
		cable_type == LT_CABLE_130K)
		return true;
	else
		return false;
}

static bool is_factory_cable_910k(void)
{
	unsigned int cable_info;
	cable_info = lge_pm_get_cable_type();

	if (cable_info == CABLE_910K ||
		cable_type == LT_CABLE_910K)
		return true;
	else
		return false;
}


static void (*bq24262_notify_vbus_state_func_ptr)(int);
int bq24262_charger_register_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	bq24262_notify_vbus_state_func_ptr = callback;
	return 0;
}
EXPORT_SYMBOL_GPL(bq24262_charger_register_vbus_sn);

/* this is passed to the hsusb via platform_data msm_otg_pdata */
void bq24262_charger_unregister_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	bq24262_notify_vbus_state_func_ptr = NULL;
}
EXPORT_SYMBOL_GPL(bq24262_charger_unregister_vbus_sn);

void bq24262_charger_force_update_batt_psy(void)
{
	struct bq24262_chip *chip = the_chip;

	if (!chip) {
		pr_err("called before init\n");
		return;
	}

	chip = the_chip;
	pr_debug("%s\n",__func__);
	power_supply_changed(&chip->batt_psy);
}
EXPORT_SYMBOL_GPL(bq24262_charger_force_update_batt_psy);

void batt_block_charging_set(int block)
{
	struct bq24262_chip *chip = the_chip;

	block_charging_state = block;
	block_charging_set(block);

	power_supply_changed(&chip->batt_psy);
}
EXPORT_SYMBOL(batt_block_charging_set);


int32_t bq24262_is_ready(void)
{
	struct bq24262_chip *chip = the_chip;

	if (!chip)
		return -EPROBE_DEFER;
	return 0;
}
EXPORT_SYMBOL(bq24262_is_ready);


static void bq24262_notify_usb_of_the_plugin_event(int plugin)
{
	plugin = !!plugin;
	if (bq24262_notify_vbus_state_func_ptr) {
		pr_debug("notifying plugin\n");
		(*bq24262_notify_vbus_state_func_ptr) (plugin);
	} else {
		pr_debug("unable to notify plugin\n");
	}
}


void block_charging_set(int block)
{
	struct bq24262_chip *chip = the_chip;
    int ret;

    ret = bq24262_set_hz_mode(chip, !block);
    if (ret) {
        pr_err("failed to set HZ_MODE ret=%d\n", ret);
    }

	if(block)
	{
		//pm_chg_auto_disable(0);
		//pm_chg_charge_dis(the_chip, 0);
		bq24262_enable_charging(chip,true);
	        bq24262_set_input_i_limit(chip, chip->set_chg_step);
	        bq24262_set_ibat_max(chip, chip->chg_current_ma);
	}
	else
	{
		//pm_chg_auto_disable(1);
		//pm_chg_auto_enable(the_chip, 1);
		bq24262_enable_charging(chip,false);
	}
}


static void bq24262_get_state(struct bq24262_chip *chip, u8 reg_r00)
{
	u8 reg_chgstate_mask = 0;

	reg_chgstate_mask = reg_r00 & CHRG_STAT_MASK;
	chip->chg_cable_status = reg_chgstate_mask >> 4;
	chip->fault_state = reg_r00 & CHG_FAULT_MASK;

	pr_info("chg_state [%s] fault state [%s]\n",
		bq24262_chg_status[chip->chg_status],fault_str[chip->fault_state]);

	return;
}


static unsigned int cable_smem_size;
static int bq24262_read_reg(struct i2c_client *client, int reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev,
			"i2c read fail: can't read from %02x: %d\n",
			reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int bq24262_write_reg(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;
//	pr_debug("%s reg = %x val = %x\n",__func__, reg,val);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int bq24262_masked_write(struct i2c_client *client, int reg,
			       u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	rc = bq24262_read_reg(client, reg, &temp);
	if (rc) {
		pr_err("bq24262_read_reg failed: reg=%03X, rc=%d\n",
				reg, rc);
		return rc;
	}

	pr_info("%s 1 reg = %x mask %x val = %x temp = %x\n",__func__,reg,mask,val,temp);

	temp &= ~mask;
	temp |= val & mask;

	pr_info("%s 2 reg = %x mask %x val = %x temp = %x\n",__func__,reg,mask,val,temp);

	rc = bq24262_write_reg(client, reg, temp);
	if (rc) {
		pr_err("bq24262_write_reg failed: reg=%03X, rc=%d\n",
				reg, rc);
		return rc;
	}

	return 0;
}



static int set_reg(void *data, u64 val)
{
	u32 addr = (u32) data;
	int ret;
	struct i2c_client *client = the_chip->client;

	ret = bq24262_write_reg(client, addr, (u8) val);

	return ret;
}

static int get_reg(void *data, u64 *val)
{
	u32 addr = (u32) data;
	u8 temp;
	int ret;
	struct i2c_client *client = the_chip->client;

	ret = bq24262_read_reg(client, addr, &temp);
	if (ret < 0)
		return ret;

	*val = temp;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_fops, get_reg, set_reg, "0x%02llx\n");


static char *bq24262_power_supplied_to[] = {
	"battery",
};

static enum power_supply_property bq24262_power_props_usb[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
//	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_HEALTH,
#if 0//def CONFIG_MACH_APQ8064_AWIFI
	POWER_SUPPLY_PROP_VCHG,
	POWER_SUPPLY_PROP_IUSB,
#endif
};

static enum power_supply_property bq24262_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
};

static enum power_supply_property bq24262_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_PSEUDO_BATT,
	POWER_SUPPLY_PROP_BLOCK_CHARGING,
	POWER_SUPPLY_PROP_EXT_PWR_CHECK,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	POWER_SUPPLY_PROP_BATTERY_ID_CHECK,
#endif
	POWER_SUPPLY_PROP_REAL_BATT_PRESENT,
#ifdef CONFIG_MACH_APQ8064_ALTEV
	POWER_SUPPLY_PROP_BATT_TEMP_ADC,
	POWER_SUPPLY_PROP_ORIG_CAPACITY,
	POWER_SUPPLY_PROP_CABLE_INFO_ADC,
	POWER_SUPPLY_PROP_REV_ADC,
	POWER_SUPPLY_PROP_PA_THERM_VAL,
	POWER_SUPPLY_PROP_PA_THERM_ADC,
	POWER_SUPPLY_PROP_GET_SET_CUR,
	POWER_SUPPLY_PROP_VZW_CHG_STATE,
#endif
};

#if 1
#define CHARGER_TYPE_USB 1
#define CHARGER_TYPE_AC  1
int bq24262_set_usb_power_supply_type(enum power_supply_type type)
{
	struct bq24262_chip *chip = the_chip;
	extern bool slimport_is_connected(void);

	pr_info("%s : %d\n", __func__,type);
	if(type == POWER_SUPPLY_TYPE_USB_CDP || type == POWER_SUPPLY_TYPE_USB
		|| type == POWER_SUPPLY_TYPE_USB_ACA){
		power_supply_set_online(chip->usb_psy, CHARGER_TYPE_USB);
	}else if(type == POWER_SUPPLY_TYPE_USB_DCP){
		power_supply_set_online(&(chip->ac_psy), CHARGER_TYPE_AC);
	}

	bq24262_charging_setting(chip);

        if((type == POWER_SUPPLY_TYPE_USB_DCP) && (!is_factory_cable())){
	        queue_delayed_work(system_nrt_wq, &chip->dpm_detect_work, msecs_to_jiffies(100));
        }

#ifdef CONFIG_LGE_DOCK
	/* Dock detect and generate uEvent */

	if ( dockdev.dev == NULL ) {
		dockdev.name = "dock";
		if (switch_dev_register(&dockdev) < 0) {
			printk("Dock switch registration failed\n");
			dockdev.dev = NULL;
		}
	}

	if ( lge_pm_get_cable_type() == CABLE_270K && !slimport_is_connected() && type ) {
		if (dockdev.dev) {
			pr_debug("%s=switch set DOCKED\n",__func__);
			switch_set_state(&dockdev, EXTRA_DOCK_STATE_DESK);
		}
	}
	else {
		if (dockdev.dev){
			pr_debug("%s=switch set UNDOCKED\n",__func__);
			switch_set_state(&dockdev, EXTRA_DOCK_STATE_UNDOCKED);
		}
	}
#endif


#if  0 //defined(CONFIG_MACH_APQ8064_AWIFI) || defined(CONFIG_MACH_APQ8064_ALTEV)
	{
		extern void i2c_bl_lcd_backlight_set_level_scale(int percentage, unsigned long duration);
		if ( type ) {
			/* If usb plugged-in, Downscale Backlight brightness for thermal */
			printk(" - Backlight Downscale \n");
			i2c_bl_lcd_backlight_set_level_scale(95, 100);
		}
		else {
			/* Restore Backlight brightness */
			printk(" - Backlight Restore \n");
			i2c_bl_lcd_backlight_set_level_scale(100, 100);
		}
	}
#endif

#if 0
	/* Wake lock for delever Uevnet before suspend */
	if(chargerlogo_state)
		/* need more time during chargerlogo sleep state to check unplug */
		wake_lock_timeout(&the_chip->deliver_uevent_wake_lock, round_jiffies_relative(msecs_to_jiffies(10000)));
	else
		wake_lock_timeout(&the_chip->deliver_uevent_wake_lock, round_jiffies_relative(msecs_to_jiffies(1000)));
#endif

	power_supply_changed(&chip->batt_psy);

#ifdef CONFIG_LGE_PM
	if (type < POWER_SUPPLY_TYPE_USB)
#else
	if (type < POWER_SUPPLY_TYPE_USB && type > POWER_SUPPLY_TYPE_BATTERY)
#endif
		return -EINVAL;

	chip->usb_psy->type= type;
	power_supply_changed(chip->usb_psy);
	power_supply_changed(&chip->ac_psy);


#if 0
	/* Adapive USB draw current limit */
	if (type == POWER_SUPPLY_TYPE_USB_DCP)
		search_iusb_max_status = IUSB_MAX_INCREASE;
	else
		search_iusb_max_status = IUSB_MAX_NONE;

	if(!delayed_work_pending(&the_chip->adaptive_usb_current_work))

	{
		printk("queueing adaptive_usb_current_work\n");
		/* max 2-sec delay time needed until fastchg-irq */
		schedule_delayed_work(&the_chip->adaptive_usb_current_work,
			  round_jiffies_relative(msecs_to_jiffies(ADAPTIVE_USB_CURRENT_CHECK_PERIOD_MS*2)));

	}
#endif

	return 0;
}

#endif



#if 1//def REGISGER_DUMP
static int bq24262_get_register(struct bq24262_chip *chip)
{
	int ret;
	u8 sys_status = 0;

	pr_info("=========================================================================\n");
	ret = bq24262_read_reg(chip->client, 0x00, &sys_status);
	if (ret) {
		pr_err("0x00 fail to read R00_STATUS_CONTROL_REG. ret=%d\n", ret);
	}
	pr_info(" 0x00 %x\n",sys_status);

	ret = bq24262_read_reg(chip->client, 0x01, &sys_status);
	if (ret) {
		pr_err("0x01 fail to read R01_STATUS_CONTROL_REG. ret=%d\n", ret);
	}
	pr_info(" 0x01 %x\n",sys_status);

	ret = bq24262_read_reg(chip->client, 0x02, &sys_status);
	if (ret) {
		pr_err("0x02 fail to read R02_STATUS_CONTROL_REG. ret=%d\n", ret);
	}
	pr_info(" 0x02 %x\n",sys_status);


	ret = bq24262_read_reg(chip->client, 0x03, &sys_status);
	if (ret) {
		pr_err("0x03 fail to read R03_STATUS_CONTROL_REG. ret=%d\n", ret);
	}
	pr_info(" 0x03 %x\n",sys_status);


	ret = bq24262_read_reg(chip->client, 0x04, &sys_status);
	if (ret) {
		pr_err("0x04 fail to read R04_STATUS_CONTROL_REG. ret=%d\n", ret);
	}
	pr_info(" 0x04 %x\n",sys_status);


	ret = bq24262_read_reg(chip->client, 0x05, &sys_status);
	if (ret) {
		pr_err("0x05 fail to read R05_STATUS_CONTROL_REG. ret=%d\n", ret);
	}
	pr_info(" 0x05 %x\n",sys_status);


	ret = bq24262_read_reg(chip->client, 0x06, &sys_status);
	if (ret) {
		pr_err("0x06 fail to read R06_STATUS_CONTROL_REG. ret=%d\n", ret);
	}
	pr_info(" 0x06 %x\n",sys_status);
	pr_info("=========================================================================\n");


	return ret;
}
#endif


#define DEFAULT_TEMP		250
static int bq24262_get_prop_batt_temp(struct bq24262_chip *chip, int *temp)
{
#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
	int rc = 0;
	struct qpnp_vadc_result results;

#ifdef CONFIG_MACH_MSM8974_Z_KR
	if(lge_get_board_revno() == HW_REV_D)
		return DEFAULT_TEMP;
#endif
	if (pseudo_batt_info_new.mode) {
		pr_debug("battery fake mode : %d \n", pseudo_batt_info_new.mode);
		return pseudo_batt_info_new.temp * 10;
	} else if (is_factory_cable()) {
		pr_debug("factory cable : %d \n", DEFAULT_TEMP / 10);
		return DEFAULT_TEMP;
	}

	if (!bq24262_get_prop_batt_present(chip)) {
		pr_err("Battery is missed, report default capacity\n");
		return DEFAULT_TEMP;
	}

	rc = qpnp_vadc_read(LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_debug("Unable to read batt temperature rc=%d\n", rc);
		pr_debug("Report last_bat_temp %d again\n", last_batt_temp);
		return last_batt_temp;
	}
	else{
		pr_debug("get_bat_temp %d %lld\n", results.adc_code, results.physical);
		last_batt_temp =(int)results.physical;
		return (int)results.physical;
	}
#else
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(CHANNEL_BATT_THERM , &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					CHANNEL_BATT_THERM, rc);
		return rc;
	}

	pr_debug(" pseudo_batt_mode is %d \n",pseudo_batt_info_new.mode);
	*temp = result.physical;

	if(pseudo_batt_info_new.mode)
		*temp = pseudo_batt_info_new.temp * 10;

	return rc;
#endif
}

static int bq24262_get_prop_batt_temp_adc(struct bq24262_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(CHANNEL_BATT_THERM , &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					CHANNEL_BATT_THERM, rc);
		return rc;
	}

	pr_debug("batt_temp phy = %lld\n", result.adc_value);

	return result.adc_value;
}

static int bq24262_get_prop_pa0_therm(void)
{
    int rc;
    struct pm8xxx_adc_chan_result result;

    rc = pm8xxx_adc_read(ADC_MPP_1_AMUX3 , &result);
    if (rc) {
        pr_err("error reading adc channel = %d, rc = %d\n",
                      ADC_MPP_1_AMUX3, rc);
        return rc;
    }

    pr_debug("pa0_therm phy = %lld\n", result.physical);

    return result.physical;
}

static int bq24262_get_prop_pa0_therm_adc(void)
{
    int rc;
    struct pm8xxx_adc_chan_result result;

    rc = pm8xxx_adc_read(ADC_MPP_1_AMUX3 , &result);
    if (rc) {
        pr_err("error reading adc channel = %d, rc = %d\n",
                      ADC_MPP_1_AMUX3, rc);
        return rc;
    }

    pr_debug("pa0_therm phy = %lld\n", result.adc_value);

    return result.adc_value;
}
static int bq24262_get_prop_batt_health(struct bq24262_chip *chip)
{
	int rc, batt_temp;

	rc = bq24262_get_prop_batt_temp(chip, &batt_temp);

	if (batt_temp >= 550)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	if (batt_temp <= -100)
		return POWER_SUPPLY_HEALTH_COLD;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

#define DEFAULT_VOLTAGE		4000000
static int bq24262_get_prop_batt_voltage_now(void)
{
#ifdef CONFIG_BATTERY_MAX17048
	int voltage = 0;
	voltage = __max17048_get_voltage() * 1000;
	return voltage;
#else
	pr_err("CONFIG_MAX17048_FUELGAUGE is not defined.\n");
	return DEFAULT_VOLTAGE;
#endif
}


#define DEFAULT_CAPACITY	50
static int bq24262_get_prop_batt_capacity(struct bq24262_chip *chip)
{
    int get_cap;
    int get_vol;
    int charger_plugin;

#ifdef CONFIG_BATTERY_MAX17048
	if(pseudo_batt_info_new.mode)
		return FACTORY_CABLE_CAPACITY;

    get_cap = __max17048_get_capacity();

	/*To determine Device Power off, battery level '0' */
	/*Check the low batt voltage and count 10 times */
	if((get_cap == 0) && (lge_get_boot_mode() != LGE_BOOT_MODE_CHARGERLOGO)){
        get_vol = bq24262_get_prop_batt_voltage_now();
        charger_plugin = bq24262_is_charger_plugin();

	    pr_info("Real soc is 0 and voltage is %d\n", get_vol );
		if( charger_plugin == true && get_vol <= (3350*1000) ){
			batt_low_cnt++;
			pr_info("Low batter check count is %d\n", batt_low_cnt);
			if(batt_low_cnt>=9){
				pr_info("Battery level is 0, Power off\n");
				return 0;
			}
			else{
				pr_info("Battery voltage is enough, Battery level is 1\n");
				return 1;
			}
		}
		else{
			batt_low_cnt = 0;
			if(charger_plugin == true ){
				pr_info("Charger is present, Battery level is 1\n");
				return 1;
			}
			else{
				pr_info("Charger is not present, Device power off\n");
				return 0;
			}
		}
	}
	else{
		batt_low_cnt=0;
		return get_cap;
	}
#else
	pr_err("CONFIG_MAX17048_FUELGAUGE is not defined.\n");
	return DEFAULT_CAPACITY;
#endif
}

static int bq24262_get_prop_batt_orig_capacity(struct bq24262_chip *chip)
{
#ifdef CONFIG_BATTERY_MAX17048
		return __max17048_get_orig_capacity();
#else
		pr_err("CONFIG_MAX17048_FUELGAUGE is not defined.\n");
		return DEFAULT_CAPACITY;
#endif

}

int bq24262_prop_is_charging(void)
{
	struct bq24262_chip *chip = the_chip;
	int chg_type = bq24262_get_prop_charge_type(chip);
	int batt_present = bq24262_get_prop_batt_present(chip);
	enum bq24262_charger_source is_charging = BQ24262_CHG_SRC_NONE;

	if(!batt_present)
		return is_charging;

	if (chg_type == POWER_SUPPLY_CHARGE_TYPE_TRICKLE ||
		chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST){
		if (chip->usb_online)
			is_charging = BQ24262_CHG_SRC_USB;
		else if(chip->ac_online)
			is_charging = BQ24262_CHG_SRC_DC;
		else
			is_charging = BQ24262_CHG_SRC_NONE;
	}

	return is_charging;
}


static int bq24262_get_prop_batt_status(struct bq24262_chip *chip)
{
	int chg_type = bq24262_get_prop_charge_type(chip);
	int batt_present = bq24262_get_prop_batt_present(chip);
	int capacity = bq24262_get_prop_batt_capacity(chip);
    int batt_status = POWER_SUPPLY_STATUS_DISCHARGING;

#if 0 /*                                    */
	if (chip->usb_present && chip->pseudo_ui_chg)
		return POWER_SUPPLY_STATUS_CHARGING;
#endif

	if (chg_type == POWER_SUPPLY_CHARGE_TYPE_UNKNOWN) {
		if (chip->ac_present)
			batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	if (chg_type == POWER_SUPPLY_CHARGE_TYPE_NONE) {
		if (capacity >= 100 && batt_present)
			batt_status = POWER_SUPPLY_STATUS_FULL;
		if (!chip->ac_present)
			batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		else if((pseudo_ui_charging == 1 && (dcreasing_charging == 1))
			&& (bq24262_get_prop_batt_health(chip) != POWER_SUPPLY_HEALTH_COLD))
            batt_status = POWER_SUPPLY_STATUS_CHARGING;
		else
			batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	if (capacity >= 100 && batt_present)
		batt_status = POWER_SUPPLY_STATUS_FULL;

	if (chg_type == POWER_SUPPLY_CHARGE_TYPE_TRICKLE ||
		chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST)
		batt_status = POWER_SUPPLY_STATUS_CHARGING;

    if((chg_type == POWER_SUPPLY_CHARGE_TYPE_TRICKLE ||
        chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST)
        && ((chg_batt_temp_state == CHG_BATT_STOP_CHARGING_STATE)
        && (pseudo_ui_charging == 0))
        && (chip->chg_cable_status == BQ_CHG_STATUS_PRE_CHARGE)
        && capacity < 100) {

        batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        return batt_status;
    }

	pr_debug("type = %d status = %d tmp stat = %d pui = %d dc = %d\n",
	chg_type,batt_status,chg_batt_temp_state, pseudo_ui_charging,dcreasing_charging);
    //bq24262_get_register(chip);

    return batt_status;
}

static int is_battery_valid(void)
{
	pr_debug("%s\n",__func__);
	return 1;
}

static int bq24262_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct bq24262_chip *chip = container_of(psy,
					struct bq24262_chip, batt_psy);
//	pr_debug("%s\n",__func__);
	int value;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq24262_get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq24262_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq24262_get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq24262_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 4350 * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 4350 * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if(is_factory_cable()){
			val->intval =FACTORY_CABLE_VOLTAGE ;
		}
		else
			val->intval = bq24262_get_prop_batt_voltage_now();
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if(pseudo_batt_info_new.mode)
			value = pseudo_batt_info_new.temp *10;
		else
			 bq24262_get_prop_batt_temp(chip, &value);
			if(lge_get_boot_mode() != LGE_BOOT_MODE_CHARGERLOGO) {

				if(value > 600)
				{
					//pr_info("===== Turned off cuase of High temperature =====\n");
					write_high_temp_power_off();
				}
			}
			val->intval = value;
		break;
	case POWER_SUPPLY_PROP_BATT_TEMP_ADC:
		val->intval = bq24262_get_prop_batt_temp_adc(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (pseudo_batt_info_new.mode) {
				val->intval = pseudo_batt_info_new.capacity;
			break;
			}else if(is_factory_cable()) {
				if(is_factory_cable_130k())
					val->intval = __max17048_get_orig_capacity();
				else
					val->intval = FACTORY_CABLE_CAPACITY;
			}
			else
					val->intval = bq24262_get_prop_batt_capacity(chip);
			break;
	case POWER_SUPPLY_PROP_ORIG_CAPACITY:
			val->intval = bq24262_get_prop_batt_orig_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CABLE_INFO_ADC:
			val->intval = (lge_pm_get_cable_type_adc() / LEVEL_CH_UV_TO_MV);
		break;
	case POWER_SUPPLY_PROP_REV_ADC:
			val->intval = (lge_get_board_revno_adc() /LEVEL_CH_UV_TO_MV);
		break;
    case POWER_SUPPLY_PROP_PA_THERM_VAL:
            val->intval = bq24262_get_prop_pa0_therm();
        break;
    case POWER_SUPPLY_PROP_PA_THERM_ADC:
            val->intval = bq24262_get_prop_pa0_therm_adc();
        break;
    case POWER_SUPPLY_PROP_GET_SET_CUR:
            val->intval = bq24262_get_input_i_limit(chip);
        break;
#if 1
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = 0;//bq24262_get_prop_batt_current_now(chip);
		break;
#else
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq24262_get_prop_batt_current_now(chip);
		break;
#endif
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = 4600;//bq24262_get_prop_batt_full_design(chip);
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		pr_debug("%d \n", chip->set_chg_current_ma);
		val->intval = chip->set_chg_current_ma;
		break;

	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		if (chip->charging_disabled) {
			val->intval = 0;
			break;
		}
		val->intval = chip->ac_online | chip->usb_online;
		break;
#if 0
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		/*                                                    
                                                       
                                                            
                                                     
   */
		val->intval = 0;
		break;
#endif
	case POWER_SUPPLY_PROP_PSEUDO_BATT:
		val->intval = pseudo_batt_info_new.mode;
		break;
	case POWER_SUPPLY_PROP_EXT_PWR_CHECK:
		val->intval = lge_pm_get_cable_type();
		break;
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	case POWER_SUPPLY_PROP_BATTERY_ID_CHECK:
		val->intval = is_battery_valid();
		break;
#endif
	case POWER_SUPPLY_PROP_BLOCK_CHARGING:
		val->intval = block_charging_state;
		break;
	case POWER_SUPPLY_PROP_REAL_BATT_PRESENT:
		val->intval = is_battery_valid();
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = chip->ac_online | chip->usb_online;
		break;
	case POWER_SUPPLY_PROP_VZW_CHG_STATE:
		val->intval = chg_state;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int bq24262_batt_power_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct bq24262_chip *chip = container_of(psy,
					struct bq24262_chip, batt_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		pr_debug("the charging value change = %d\n",val->intval);
		bq24262_enable_charging(chip, val->intval);
		break;
#if 0
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		/*                                                    
                                                       
                                                            
                                                     
   */
		break;
#endif
	default:
		return -EINVAL;
	}
	pr_debug("%s\n",__func__);
	power_supply_changed(&chip->batt_psy);
	return 0;
}


static int
bq24262_batt_power_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
//	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		return 1;
	default:
		break;
	}

	return 0;
}

static int set_therm_mitigation_level(const char *val, struct kernel_param *kp)
{
	int ret;
	struct bq24262_chip *chip = the_chip;

	printk("set_therm_mitigation_level\n");

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if (!chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (!chip->thermal_mitigation) {
		pr_err("no thermal mitigation\n");
		return -EINVAL;
	}

	if(delayed_work_pending(&chip->battemp_work))
		cancel_delayed_work(&chip->battemp_work);

	schedule_delayed_work(&chip->battemp_work,0);

	return ret;
}

module_param_call(thermal_mitigation, set_therm_mitigation_level,
					param_get_uint,
					&thermal_mitigation, 0644);

#ifdef CONFIG_LGE_PM_LOW_BATT_CHG
/* this is indicator that chargerlogo app is running. */
/* sysfs : /sys/module/bq24262_charger/parameters/chargerlogo_state */
static int param_set_chargerlogo_state(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	pr_debug("set chargerlogo_state to %d\n", chargerlogo_state);
	return 0;
}
module_param_call(chargerlogo_state, param_set_chargerlogo_state,
	param_get_uint, &chargerlogo_state, 0644);
#endif

#if 0
/* this is indicator that  set the register to default */
/* sysfs : /sys/module/bq24262_charger/parameters/set_default */
static int param_set_default(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	set_register_to_default(the_chip, true);
	pr_info("set the register to default. val=%d\n",set_default);
	return 0;
}
module_param_call(set_default, param_set_default,
	param_get_uint, &set_default, 0664);

#endif

static int set_reg_default(struct notifier_block *, unsigned long, void *);

static struct notifier_block set_default_notifier = {
	.notifier_call = set_reg_default,
};
static int notifier_disabled = 0;

static int set_reg_default(struct notifier_block *nb, unsigned long event, void *buf)
{
	char *txt;

	if (notifier_disabled)
		return NOTIFY_OK;

	notifier_disabled = 1;
	switch (event) {
	case SYS_RESTART:
		txt = "SYSTEM RESTART";
		break;

	case SYS_HALT:
		txt = "SYSTEM HALT";
		break;

	case SYS_POWER_OFF:
		txt = "SYSTEM POWER OFF";
		pr_info("%s : Power OFF -> Set the reg to default\n",__func__);
		bq24262_set_input_i_limit(the_chip, INPUT_CURRENT_LIMIT_500mA);
		bq24262_set_ibat_max(the_chip, INPUT_CURRENT_LIMIT_500mA);
		the_chip->set_chg_step = INPUT_CURRENT_LIMIT_500mA;

		break;
	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}


static void bq24262_charging_setting(struct bq24262_chip *chip)
{
	int ret_limit, ret_ibat;
//	union power_supply_propval usb_ret = {0,};
	pr_debug("%s \n",__func__);
	if (is_factory_cable()) {
		ret_limit = bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_1500mA);
		ret_ibat = bq24262_set_ibat_max(chip, INPUT_CURRENT_LIMIT_500mA);
		chip->set_chg_step = INPUT_CURRENT_LIMIT_1500mA;
		pr_debug("Factory cable limit = %d, ibat = %d\n",ret_limit,ret_ibat);
	}
	else if (chip->usb_online &&
		bq24262_is_charger_present(chip)) {
		wake_lock(&chip->eoc_wake_lock);
		ret_limit = bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_500mA);
		ret_ibat = bq24262_set_ibat_max(chip, INPUT_CURRENT_LIMIT_500mA);
		chip->set_chg_step = INPUT_CURRENT_LIMIT_500mA;
		pr_debug("usb online limit = %d, ibat = %d\n",ret_limit,ret_ibat);
	}
	else if (chip->ac_online &&
		bq24262_is_charger_present(chip)) {
		wake_lock(&chip->eoc_wake_lock);
		ret_limit = bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_500mA);
		ret_ibat = bq24262_set_ibat_max(chip, INPUT_CURRENT_LIMIT_500mA);
		chip->set_chg_step = INPUT_CURRENT_LIMIT_500mA;
		pr_debug("ac online limit = %d, ibat = %d\n",ret_limit,ret_ibat);
	}
	else { /* default */
		ret_limit = bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_500mA);
		ret_ibat = bq24262_set_ibat_max(chip, INPUT_CURRENT_LIMIT_500mA);
		chip->set_chg_step = INPUT_CURRENT_LIMIT_500mA;
		pr_debug("default limit = %d, ibat = %d\n",ret_limit,ret_ibat);
		if(slimport_is_connected()){
			pr_info("%s : Slimport is connected so usb_online\n",__func__);
			power_supply_set_online(chip->usb_psy, true);
		}
		else{
			if(chip->ac_present)
				power_supply_set_present(&(chip->ac_psy), BQ24262_CHG_SRC_NONE);
			if(chip->ac_online)
				power_supply_set_online(&(chip->ac_psy), BQ24262_CHG_SRC_NONE);
		}
	}

	/* When facebattery mode and  charging set 2, charging current set 900mA
	* set : echo 1 1 100 40 4100 80 2 > /sys/class/power_supply/battery/pseudo_batt
	*/
	/*                                */
	if((chip->usb_online && bq24262_is_charger_present(chip))
		&& ((pseudo_batt_info_new.mode == 1)
		&& (pseudo_batt_info_new.charging == 2))){
		ret_limit = bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_900mA);
		ret_ibat = bq24262_set_ibat_max(chip, INPUT_CURRENT_LIMIT_900mA);
		chip->set_chg_step = INPUT_CURRENT_LIMIT_900mA;
	}
	/*                              */

	pr_debug("%s\n",__func__);
	power_supply_changed(chip->usb_psy);
	power_supply_changed(&chip->ac_psy);
	power_supply_changed(&chip->batt_psy);

}


static void bq24262_batt_external_power_changed(struct power_supply *psy)
{
	struct bq24262_chip *chip = container_of(psy,
					struct bq24262_chip, batt_psy);

	pr_debug("%s \n",__func__);

	if (chargerlogo_state) {
		wake_lock_timeout(&chip->uevent_wake_lock, HZ*10);
	}
	else {
		wake_lock_timeout(&chip->uevent_wake_lock, HZ*2);
	}

//	power_supply_changed(chip->usb_psy);
//	power_supply_changed(&chip->ac_psy);
	power_supply_changed(&chip->batt_psy);
}

static int bq24262_power_get_property_usb(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
//	int current_max;
	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 0;
		val->intval = the_chip->ac_present;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = the_chip->usb_online;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		if (the_chip->host_mode)
			val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		else
			val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq24262_power_set_property_usb(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		the_chip->ac_present = val->intval;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		the_chip->usb_online = val->intval;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		if (val->intval == POWER_SUPPLY_SCOPE_SYSTEM){
//			return bq24262_is_otg_mode(the_chip);
//			if(!bq24262_is_otg_mode(the_chip)){
				bq24262_enable_charging(the_chip,false);
				bq24262_enable_otg(the_chip, true);
#ifdef REGISGER_DUMP
				bq24262_get_register(the_chip);
#endif
//			}
		return bq24262_is_otg_mode(the_chip);
		}
		if (val->intval == POWER_SUPPLY_SCOPE_DEVICE){
//			return bq24262_is_otg_mode(the_chip);
//			if(bq24262_is_otg_mode(the_chip)){
				bq24262_enable_otg(the_chip, false);
				bq24262_enable_charging(the_chip,true);
#ifdef REGISGER_DUMP
				bq24262_get_register(the_chip);
#endif
//			}
		return bq24262_is_otg_mode(the_chip);
		}
		else
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_TYPE:
	case POWER_SUPPLY_PROP_HEALTH:
	default:
		return -EINVAL;
	}
	pr_debug("%s\n",__func__);
	power_supply_changed(the_chip->usb_psy);
	return 0;
}

static int bq24262_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = the_chip->set_chg_current_ma;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = the_chip->ac_present;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = the_chip->ac_online;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq24262_get_prop_charge_type(the_chip);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq24262_power_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		the_chip->ac_present = val->intval;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		the_chip->ac_online = val->intval;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		the_chip->set_chg_current_ma = val->intval;
		bq24262_set_ibat_max(the_chip, val->intval / 1000);
		break;
	default:
		return -EINVAL;
	}
	pr_debug("%s\n",__func__);
	power_supply_changed(&the_chip->ac_psy);
	return 0;
}

static int bq24262_power_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

#define LOW_SOC_HEARTBEAT_MS	 20000
#define MIN_LOW_SOC_HEARTBEAT_MS 3000
#define UPDATE_TIME_MS 60000

static void update_heartbeat(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bq24262_chip *chip = container_of(dwork,
				struct bq24262_chip, update_heartbeat_work);

	power_supply_changed(&chip->batt_psy);

	if (bq24262_get_prop_batt_capacity(chip) <= 20){
		if (bq24262_get_prop_batt_capacity(chip) <= 1){
			printk("%s : SOC update per 1sec\n",__func__);
			schedule_delayed_work(&chip->update_heartbeat_work,
				      round_jiffies_relative(msecs_to_jiffies
							     (MIN_LOW_SOC_HEARTBEAT_MS)));
			}

		else {
			printk("%s : SOC update per 20sec\n",__func__);
			schedule_delayed_work(&chip->update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (LOW_SOC_HEARTBEAT_MS)));
			}
	}

	else{
		printk("%s : SOC update per 60sec\n",__func__);
		schedule_delayed_work(&chip->update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (UPDATE_TIME_MS)));
	}
}


static ssize_t at_chg_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int r;
	bool b_chg_ok = false;
	int chg_type;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	chg_type = bq24262_get_prop_charge_type(the_chip);
	if (chg_type != POWER_SUPPLY_CHARGE_TYPE_NONE) {
		b_chg_ok = true;
		r = snprintf(buf, 3, "%d\n", b_chg_ok);
		pr_debug("[Diag] true ! buf = %s, charging=1\n", buf);
	} else {
		b_chg_ok = false;
		r = snprintf(buf, 3, "%d\n", b_chg_ok);
		pr_debug("[Diag] false ! buf = %s, charging=0\n", buf);
	}

	return r;
}

static ssize_t at_chg_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;

	lge_power_test_flag = 1;

	if (!count) {
		pr_err("[Diag] count 0 error\n");
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (strncmp(buf, "0", 1) == 0) {
		/* stop charging */
		pr_debug("[Diag] stop charging start\n");
		ret = bq24262_enable_charging(the_chip, false);

	} else if (strncmp(buf, "1", 1) == 0) {
		/* start charging */
		pr_debug("[Diag] start charging start\n");
		ret = bq24262_enable_charging(the_chip, true);
	}

	if (ret)
		return -EINVAL;

	return 1;
}

static ssize_t at_chg_complete_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int guage_level = 0;
	int r = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	guage_level = bq24262_get_prop_batt_capacity(the_chip);

	if (guage_level == 100) {
		r = snprintf(buf, 3, "%d\n", 0);
		pr_debug("[Diag] buf = %s, gauge==100\n", buf);
	} else {
		r = snprintf(buf, 3, "%d\n", 1);
		pr_debug("[Diag] buf = %s, gauge<=100\n", buf);
	}

	return r;
}

static ssize_t at_chg_complete_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;

	if (!count) {
		pr_err("[Diag] count 0 error\n");
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (strncmp(buf, "0", 1) == 0) {
		/* charging not complete */
		pr_debug("[Diag] charging not complete start\n");
		ret = bq24262_enable_charging(the_chip, true);
	} else if (strncmp(buf, "1", 1) == 0) {
		/* charging complete */
		pr_debug("[Diag] charging complete start\n");
		ret = bq24262_enable_charging(the_chip, false);
	}

	if (ret)
		return -EINVAL;

	return 1;
}

static ssize_t at_pmic_reset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r = 0;
	bool pm_reset = true;

	msleep(3000); /* for waiting return values of testmode */

	machine_restart(NULL);

	r = snprintf(buf, 3, "%d\n", pm_reset);

	return r;
}
static ssize_t at_otg_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int otg_mode;
	int r = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	otg_mode = bq24262_is_otg_mode(the_chip);
	if(otg_mode) {
		otg_mode = 1;
		r = snprintf(buf, 3, "%d\n", otg_mode);
		pr_debug("[Diag] true ! buf = %s, OTG Enabled\n", buf);
	}
	else {
		otg_mode = 0;
		r = snprintf(buf, 3, "%d\n", otg_mode);
		pr_debug("[Diag] false ! buf = %s, OTG Disabled\n", buf);
	}
	return r;
}

static ssize_t at_otg_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;

	if (!count) {
		pr_err("[Diag] count 0 error\n");
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (strncmp(buf, "0", 1) == 0) {
		pr_debug("[Diag] OTG Disable start\n");
		if(bq24262_is_otg_mode(the_chip))
			ret = bq24262_enable_otg(the_chip, false);

	} else if (strncmp(buf, "1", 1) == 0) {
		pr_debug("[Diag] OTG Enable start\n");
		if(!bq24262_is_otg_mode(the_chip))
			ret = bq24262_enable_otg(the_chip, true);
	}

	if(ret)
		return -EINVAL;
	return 1;
}

static ssize_t at_current_limit_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int r = 0;
	bool current_limit = true;

	//	printk(KERN_INFO "Current limit is successfully called\n");

	bq24262_set_input_i_limit(the_chip, INPUT_CURRENT_LIMIT_1500mA);

	r = sprintf(buf, "%d\n", current_limit);

	return r;
}

static ssize_t at_vcoin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int r;
	struct pm8xxx_adc_chan_result result = { 0 };

	r = pm8xxx_adc_read(CHANNEL_VCOIN, &result);
	r = sprintf(buf, "%lld\n", result.physical);

	return r;
}


//DEVICE_ATTR(at_charge1, 0644, at_chg_status_show, at_chg_status_store);
DEVICE_ATTR(at_charge, 0644, at_chg_status_show, at_chg_status_store);
DEVICE_ATTR(at_current1, 0644, at_current_limit_show, NULL);
DEVICE_ATTR(at_chcomp1, 0644, at_chg_complete_show, at_chg_complete_store);
DEVICE_ATTR(at_pmrst1, 0640, at_pmic_reset_show, NULL);
DEVICE_ATTR(at_otg, 0644, at_otg_status_show, at_otg_status_store);
DEVICE_ATTR(at_vcoin, 0644, at_vcoin_show, NULL);


int pseudo_batt_set_new(struct pseudo_batt_info_type *info)
{
	struct bq24262_chip *chip = the_chip;
	pr_err("pseudo_batt_set_new\n");
	pseudo_batt_info_new.mode = info->mode;
	pseudo_batt_info_new.id = info->id;
	pseudo_batt_info_new.therm = info->therm;
	pseudo_batt_info_new.temp = info->temp;
	pseudo_batt_info_new.volt = info->volt;
	pseudo_batt_info_new.capacity = info->capacity;
	pseudo_batt_info_new.charging = info->charging;

	pr_debug("%s\n",__func__);
	power_supply_changed(&chip->batt_psy);

	return 0;
}
EXPORT_SYMBOL(pseudo_batt_set_new);


static int bq24262_chg_is_battery_too_hot_or_too_cold(void *data,int batt_temp, int batt_level)
{
	int batt_temp_level;
	int rtnValue = 0;

	struct bq24262_chip *chip = data;

    pseudo_ui_charging = 0;

	if (batt_temp > chip->temp_level_1)
		batt_temp_level = CHG_BATT_TEMP_LEVEL_1;/*  Over 55  */
	else if (batt_temp >= chip->temp_level_2)
		batt_temp_level = CHG_BATT_TEMP_LEVEL_2;/*  45~55   */
	else if (batt_temp >= chip->temp_level_3)
		batt_temp_level = CHG_BATT_TEMP_LEVEL_3;/*  42~45   */
	else if (batt_temp > chip->temp_level_4)
		batt_temp_level = CHG_BATT_TEMP_LEVEL_4;/*  M4~41   */
	else if (batt_temp >= chip->temp_level_5)
		batt_temp_level = CHG_BATT_TEMP_LEVEL_5;/*  M10~M5   */
	else
		batt_temp_level = CHG_BATT_TEMP_LEVEL_6;/*  Under M10   */

	switch(chg_batt_temp_state)
	{
	case CHG_BATT_NORMAL_STATE:
		if (batt_temp_level == CHG_BATT_TEMP_LEVEL_1 ||
			batt_temp_level == CHG_BATT_TEMP_LEVEL_6 ||
			(batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000)) {

			chg_batt_temp_state = CHG_BATT_STOP_CHARGING_STATE;
			/*
                                  
                                                                            
    */
/* CONFIG_PM_S submit ATnT temp scenario kwangjae1.lee */
			/*                                                               */
			if (batt_temp_level == CHG_BATT_TEMP_LEVEL_1)
			{
				pseudo_ui_charging = 0;
			} else if(batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000) {
				dcreasing_charging = 1;
				/* we must show charging image although charging is stopped. */
				pseudo_ui_charging = 1;
			}
/* CONFIG_PM_E submit ATnT temp scenario kwangjae1.lee */
			pr_debug("BATT TEMP OUT OF SPEC (STATE: %d) (thm: %d) (volt: %d)!.\n",
				 CHG_BATT_NORMAL_STATE, batt_temp, batt_level);

			rtnValue = 1;
		}
		else if (batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level <= 4000) {
			chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;
			dcreasing_charging = 1;

			pr_debug("BATT TEMP 46 ~ 55 (STATE: %d) (thm: %d) (volt: %d)!.\n",
				 CHG_BATT_NORMAL_STATE, batt_temp, batt_level);

			rtnValue = 0;

			if (the_chip)
				bq24262_set_ibat_max(chip,INPUT_CURRENT_LIMIT_500mA);
			}

		else {
			chg_batt_temp_state = CHG_BATT_NORMAL_STATE;

			pr_debug("BATT TEMP NORMAL (STATE: %d) (thm: %d) (volt: %d)!.\n",
				 CHG_BATT_NORMAL_STATE, batt_temp, batt_level);

			rtnValue = 0;
		}
		break;

	case CHG_BATT_DC_CURRENT_STATE:
		if (batt_temp_level == CHG_BATT_TEMP_LEVEL_1 ||
			batt_temp_level == CHG_BATT_TEMP_LEVEL_6 ||
			(batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000)) {

			chg_batt_temp_state = CHG_BATT_STOP_CHARGING_STATE;
			dcreasing_charging = 1;
			/*
                                  
                                                                            
    */
			if (batt_temp_level == CHG_BATT_TEMP_LEVEL_1)
			{
				pseudo_ui_charging = 0;
			} else if(batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000) {
				/* we must show charging image although charging is stopped. */
				pseudo_ui_charging = 1;
			}
			pr_debug("BATT TEMP OUT OF SPEC (STATE: %d) (thm: %d) (volt: %d)!.\n",
				 CHG_BATT_DC_CURRENT_STATE, batt_temp, batt_level);

			rtnValue = 1;

			}else if ((batt_temp_level == CHG_BATT_TEMP_LEVEL_2 ||
					batt_temp_level == CHG_BATT_TEMP_LEVEL_3) && batt_level <= 4000) {

			chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;

			pr_debug("BATT TEMP 46 ~ 55 (STATE: %d) (thm: %d) (volt: %d)!.\n",
				 CHG_BATT_DC_CURRENT_STATE, batt_temp, batt_level);

            if (the_chip)
                bq24262_set_ibat_max(chip,INPUT_CURRENT_LIMIT_500mA);

			rtnValue = 0;

		} else if (batt_temp_level == CHG_BATT_TEMP_LEVEL_4) {

			chg_batt_temp_state = CHG_BATT_NORMAL_STATE;

			pr_debug("BATT TEMP NORMAL (STATE: %d) (thm: %d) (volt: %d)!.\n",
				CHG_BATT_DC_CURRENT_STATE, batt_temp, batt_level);

			rtnValue = 0;

			if (the_chip){
                bq24262_set_ibat_max(chip,chip->chg_current_ma);

				pr_err("%s: BATT TEMP NORMAL last_usb_chg_current : %d)!.\n",
					__func__, last_usb_chg_current);
			}
		} else {
			chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;

			pr_debug("BATT TEMP UNREAL (STATE: %d) (thm: %d) (volt: %d)!.\n",
				 CHG_BATT_DC_CURRENT_STATE, batt_temp, batt_level);

			rtnValue = 0;
		}
		break;

	case CHG_BATT_STOP_CHARGING_STATE:
		if (batt_temp_level == CHG_BATT_TEMP_LEVEL_4)
		{
			chg_batt_temp_state = CHG_BATT_NORMAL_STATE;

			pr_err("BATT TEMP NORMAL (STATE: %d) (thm: %d) (volt: %d)!.\n",
				 CHG_BATT_STOP_CHARGING_STATE, batt_temp, batt_level);

			rtnValue = 0;

			if (the_chip){
				pr_err("BATT TEMP NORMAL last_usb_chg_current : %d)!.\n",
					 last_usb_chg_current);
			}
		}
		else {
			chg_batt_temp_state = CHG_BATT_STOP_CHARGING_STATE;
			/*                                                               */
			if (batt_temp_level == CHG_BATT_TEMP_LEVEL_1)
			{
				dcreasing_charging = 0;
				pseudo_ui_charging = 0;
			} else if(batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000) {
				 /* we must show charging image although charging is stopped. */
				pseudo_ui_charging = 1;
			}
			pr_debug("BATT TEMP OUT OF SPEC (STATE: %d) (thm: %d) (volt: %d)!.\n",
				CHG_BATT_STOP_CHARGING_STATE, batt_temp,batt_level);

			rtnValue = 1;
		}
		break;

	case DISCHG_BATT_NORMAL_STATE:

			if (batt_temp_level == CHG_BATT_TEMP_LEVEL_1 ||
				batt_temp_level == CHG_BATT_TEMP_LEVEL_6 ||
				(batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000)) {

				chg_batt_temp_state = CHG_BATT_STOP_CHARGING_STATE;

				/*                                                               */
				if (batt_temp_level == CHG_BATT_TEMP_LEVEL_1)
				{
					pseudo_ui_charging = 0;
				} else if(batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000) {
					dcreasing_charging = 1;
				/* we must show charging image although charging is stopped. */
					pseudo_ui_charging = 1;
				}

				pr_debug("BATT TEMP OUT OF SPEC (STATE: %d) (thm: %d) (volt: %d)!.\n",
				 CHG_BATT_NORMAL_STATE, batt_temp, batt_level);

				rtnValue = 1;
				} else if (batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level <= 4000) {
				chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;
				dcreasing_charging = 1;

				pr_debug("BATT TEMP 46 ~ 55 (STATE: %d) (thm: %d) (volt: %d)!.\n",
						 CHG_BATT_NORMAL_STATE, batt_temp, batt_level);

				rtnValue = 0;

		       		if (the_chip)
					bq24262_set_ibat_max(chip,INPUT_CURRENT_LIMIT_500mA);
				} else {
				chg_batt_temp_state = CHG_BATT_NORMAL_STATE;

				pr_debug("BATT TEMP NORMAL (STATE: %d) (thm: %d) (volt: %d)!.\n",
						 CHG_BATT_NORMAL_STATE, batt_temp, batt_level);

				rtnValue = 0;
				}
				break;

	}
		/* To contorl End of charging state, Control regulation voltage */
		if(batt_level > 4300){
			if(reg_vol_set_count == 0){
				bq24262_set_vbat_max(chip, 4300);
				reg_vol_set_count++;
				pr_info("%s= Set the Regulation volage to 4300V\n", __func__);
				queue_delayed_work(system_nrt_wq, &chip->battemp_work, msecs_to_jiffies(2000));
			}
			else{
				bq24262_set_vbat_max(chip, 4280);
				pr_info("%s= Set the Regulation volage to 4280V\n", __func__);
			}
		}

		if(bq24262_get_dpm_state(chip) && (dpm_retry_count>0)){
			pr_info("%s : Try again DPM detect work\n",__func__);
			queue_delayed_work(system_nrt_wq, &chip->dpm_detect_work,msecs_to_jiffies(100));
			dpm_retry_count=0;
		}

	return rtnValue;
}


static void bq24262_monitor_batt_temp(struct work_struct *work)
{
	struct bq24262_chip *chip =
		container_of(work, struct bq24262_chip, battemp_work.work);
	int rc, batt_temp;
	int batt_volt;
	int bat_voltage;
	int stop_charging = 0;
	static unsigned int prev_chg_batt_temp_state = 0;

	printk(">>bq24262_monitor_batt_temp\n");

	if (is_factory_cable()){
		batt_temp = 250;
	}else{
		rc = bq24262_get_prop_batt_temp(chip, &batt_temp);
	}

	batt_volt = __max17048_get_voltage();

	if(pseudo_batt_info_new.mode){
		batt_temp = pseudo_batt_info_new.temp * 10;
		batt_volt = (pseudo_batt_info_new.volt == 0) ? batt_volt : pseudo_batt_info_new.volt;
	}

	bat_voltage = bq24262_get_prop_batt_temp_adc(chip);

	pr_debug(" psudo=%d, factory=%d temp=%d, volt=%d, temp_ADC=%d\n",
		pseudo_batt_info_new.mode,is_factory_cable(), batt_temp, batt_volt, bat_voltage);

	stop_charging = bq24262_chg_is_battery_too_hot_or_too_cold(chip, batt_temp, batt_volt);
	pr_debug("stop_charging = %d last = %d\n",stop_charging,last_stop_charging);

	if(wake_lock_active(&chip->monitor_batt_temp_wake_lock) && !bq24262_is_charger_plugin()) {
		printk(KERN_INFO "[PM] monitor_batt_temp: Release wake lock , no charger\n");
		wake_unlock(&chip->monitor_batt_temp_wake_lock);
	}

	if (stop_charging == 1 && last_stop_charging == 0) {
		/*                                                                           */
		if(wake_lock_active(&chip->eoc_wake_lock)) {
			printk(KERN_INFO "[PM] monitor_batt_temp: Charging stop & wake lock by Temperature Scenario\n");
			wake_lock(&chip->monitor_batt_temp_wake_lock);
		}
		if(bq24262_chg_status[chip->chg_status] == bq24262_chg_status[3]){
			pr_info("%s : (exception)Chg_status : %s\n",__func__, bq24262_chg_status[chip->chg_status]);
			bq24262_enable_charging(chip, true);
			last_stop_charging = 0;
		}
		else{
			pr_info("%s : (normal)Chg_status : %s\n",__func__, bq24262_chg_status[chip->chg_status]);
			bq24262_enable_charging(chip, false);
			last_stop_charging = stop_charging;
//		pseudo_ui_charging = 1;
		}
	}
	else if (stop_charging == 0 && last_stop_charging == 1) {
		/*                                                                          */
		bq24262_set_input_i_limit(chip, chip->set_chg_step);
		bq24262_set_ibat_max(chip, chip->chg_current_ma);
                bq24262_enable_charging(chip, true);
		last_stop_charging = stop_charging;
//		pseudo_ui_charging = 0;

		if(wake_lock_active(&chip->monitor_batt_temp_wake_lock)) {
			printk(KERN_INFO "[PM] monitor_batt_temp: Release wake lock by Temperature Scenario\n");
			wake_unlock(&chip->monitor_batt_temp_wake_lock);
	}
}
	/* DO NOT REPORT BATTERY UEVENT PERIODICALLY HERE !!!
	    It is cause of heavy load for Framework - Periodic Report is doing in Heartbeat workqueue */

	if (prev_chg_batt_temp_state != chg_batt_temp_state) {
		prev_chg_batt_temp_state = chg_batt_temp_state;
		power_supply_changed(&chip->batt_psy);
	}

	if ((!last_stop_charging) && (chg_batt_temp_state != CHG_BATT_DC_CURRENT_STATE)) {

		if ((thermal_mitigation == 0)&&(thermal_mitigation != pre_mitigation)) {
			bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_1500mA);
			printk("== thermal_mitigation = %d, pre_mitigation = %d ilimit 1500 \n", thermal_mitigation, pre_mitigation);
			pre_mitigation = thermal_mitigation;
		} else if ((thermal_mitigation == 1)&&(thermal_mitigation != pre_mitigation)){
			bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_900mA);
			printk("== thermal_mitigation = %d, pre_mitigation = %d ilimit 900\n", thermal_mitigation, pre_mitigation);
			pre_mitigation = thermal_mitigation;
		}else if ((thermal_mitigation == 2)&&(thermal_mitigation != pre_mitigation)) {
			bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_500mA);
			printk("== thermal_mitigation = %d, pre_mitigation = %d ilimit 500\n", thermal_mitigation, pre_mitigation);
			pre_mitigation = thermal_mitigation;
		}else if ((thermal_mitigation == 3)&&(thermal_mitigation != pre_mitigation)) {
			bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_150mA);
			printk("== thermal_mitigation = %d, pre_mitigation = %d ilimit 150\n", thermal_mitigation, pre_mitigation);
			pre_mitigation = thermal_mitigation;
		}else if ((thermal_mitigation == 4)&&(thermal_mitigation != pre_mitigation)) {
			bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_100mA);
			printk("== thermal_mitigation = %d, pre_mitigation = %d ilimit 100\n", thermal_mitigation, pre_mitigation);
			pre_mitigation = thermal_mitigation;
		}

	}

#ifdef REGISGER_DUMP
	bq24262_get_register(chip);
#endif
//	power_supply_changed(&chip->batt_psy);

    pr_info("temp = %d status = %d health = %d ac = %d usb = %d volt = %d\n",
    batt_temp, bq24262_get_prop_batt_status(chip), bq24262_get_prop_batt_health(chip),
    chip->ac_online, chip->usb_online,__max17048_get_voltage());

    pr_info(" chg [%s] falut [%s] \n",
		bq24262_chg_status[chip->chg_status],fault_str[chip->fault_state]);

    pr_info("[PA THERM] result [%d], raw [%d] \n", bq24262_get_prop_pa0_therm(), bq24262_get_prop_pa0_therm_adc());

	schedule_delayed_work(&chip->battemp_work,
		msecs_to_jiffies(MONITOR_BATTEMP_POLLING_PERIOD));
}

static int bq24262_create_debugfs_entries(struct bq24262_chip *chip)
{
	int i;

	chip->dent = debugfs_create_dir(BQ24262_NAME, NULL);
	if (IS_ERR(chip->dent)) {
		pr_err("bq24262 driver couldn't create debugfs dir\n");
		return -EFAULT;
	}

	for (i = 0 ; i < ARRAY_SIZE(bq24262_debug_regs) ; i++) {
		char *name = bq24262_debug_regs[i].name;
		u32 reg = bq24262_debug_regs[i].reg;
		struct dentry *file;

		file = debugfs_create_file(name, 0644, chip->dent,
					(void *) reg, &reg_fops);
		if (IS_ERR(file)) {
			pr_err("debugfs_create_file %s failed.\n", name);
			return -EFAULT;
		}
	}

	return 0;
}


#define OTG_ENABLE_SHIFT  6
static int bq24262_enable_otg(struct bq24262_chip *chip, bool enable)
{
	int ret;
	u8 val = ((u8)enable << OTG_ENABLE_SHIFT);

	pr_debug("otg enable = %d\n", enable);

	ret = bq24262_masked_write(chip->client, R00_STATUS_CONTROL_REG,
					BOOST_MODE_MASK, val);
	if (ret) {
		pr_err("failed to set CHG_CONFIG rc=%d\n", ret);
		return ret;
	}

	return 0;
}

struct input_ma_limit_entry {
	int  icl_ma;
	u8  value;
};

static struct input_ma_limit_entry icl_ma_table[] = {
	{100, 0x00},
	{150, 0x01},
	{500, 0x02},
	{900, 0x03},
	{1500, 0x04},
	{2500, 0x06},
};

static void bq24262_set_clear_reg(struct bq24262_chip *chip)
{
	chg_batt_temp_state = CHG_BATT_NORMAL_STATE;
	bq24262_enable_charging(chip,true);
	bq24262_set_vbat_max(chip, chip->regulation_mV);
	wake_lock_timeout(&chip->uevent_wake_lock, HZ*1);
	bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_500mA);
	bq24262_set_ibat_max(chip, INPUT_CURRENT_LIMIT_500mA);
	chip->set_chg_step = INPUT_CURRENT_LIMIT_500mA;

	reg_vol_set_count=0;
	dpm_count=0;
	dpm_retry_count=0;
	last_stop_charging = 0;
	chg_state = VZW_COMPATIBLE_CHG;
	cancel_delayed_work_sync(&chip->dpm_detect_work);
	pr_debug("%s : Cable removed!! all registers are changed\n",__func__);
	if(bq24262_get_hz_mode(chip))
		bq24262_set_hz_mode(chip, false);
	if(wake_lock_active(&chip->eoc_wake_lock)){
		wake_unlock(&chip->eoc_wake_lock);
	}
	return;
}

static int bq24262_set_input_i_limit(struct bq24262_chip *chip, int ma)
{
	int i;
	u8 temp;

	if (ma < INPUT_CURRENT_LIMIT_100mA
			|| ma > INPUT_CURRENT_LIMIT_2500mA) {
		pr_err("bad mA=%d asked to set\n", ma);
		return -EINVAL;
	}

	for (i = ARRAY_SIZE(icl_ma_table) - 1; i >= 0; i--) {
		if (icl_ma_table[i].icl_ma == ma)
			break;
	}

	if (i < 0) {
		pr_err("can't find %d in icl_ma_table. Use min.\n", ma);
		i = 0;
	}

	temp = icl_ma_table[i].value;
	temp = temp << 4;

	return bq24262_masked_write(chip->client, R01_CONTROL_REG,
			IINLIM_MASK, temp);
}

#define BIT_SHIFT 4
static int bq24262_get_input_i_limit(struct bq24262_chip *chip)
{
    int i,ret;
    u8 sys_control;

    ret = bq24262_read_reg(chip->client, R01_CONTROL_REG, &sys_control);
    if (ret) {
        pr_err("failed to read R01_CONTROL_REG rc=%d\n", ret);
        return false;
    }
    if(!bq24262_is_charger_plugin()){
	return 0;
	}

    sys_control &= IINLIM_MASK;
    sys_control = sys_control >> BIT_SHIFT;
    sys_control &=~EN_STAT;

    for (i = ARRAY_SIZE(icl_ma_table) - 1; i >= 0; i--) {
        if (icl_ma_table[i].value == sys_control)
            break;
    }

    return icl_ma_table[i].icl_ma;
}

#define EN_CHG_TERM_BIT 2
static int bq24262_enable_charger_current_termination(struct bq24262_chip *chip, u8 bit_set)
{
	bit_set = bit_set << EN_CHG_TERM_BIT;
	return bq24262_masked_write(chip->client, R01_CONTROL_REG,
			EN_CHG_TERM_MASK, bit_set);
}

#define EN_CHG_INT_BIT 3
static int bq24262_enable_interrupt(struct bq24262_chip *chip, u8 int_set)
{
	int_set = int_set << EN_CHG_INT_BIT;
	return bq24262_masked_write(chip->client, R01_CONTROL_REG,
			EN_STAT, int_set);
}

#define CHG_ENABLE_SHIFT  1
static int bq24262_enable_charging(struct bq24262_chip *chip, bool enable)
{
	int ret;
	u8 val = (u8)(!enable << CHG_ENABLE_SHIFT);

	pr_debug("enable=%d\n", enable);

	ret = bq24262_masked_write(chip->client, R01_CONTROL_REG,
						CHG_CONFIG_MASK, val);
	if (ret){
		pr_err("failed to set CHG_CONFIG ret=%d\n", ret);
		return ret;
	}

	return 0;
}

#if 0
#define REG_CLEAR_SHIFT  7
static int set_register_to_default(struct bq24262_chip *chip, bool enable)
{
	int ret;
	u8 val = (u8)(!enable << REG_CLEAR_SHIFT);

	pr_debug("reg_clear=%d\n", enable);

	ret = bq24262_masked_write(chip->client, R01_CONTROL_REG,
						ALL_REG_CLEAR, val);
	if (ret){
		pr_err("failed to set ALL_REG_CLEAR ret=%d\n", ret);
		return ret;
	}

	return 0;
}
#endif

static int bq24262_set_hz_mode(struct bq24262_chip *chip, bool enable)
{
    int ret;
    u8 val = (u8)(enable);

    pr_debug("enable=%d\n", enable);

    ret = bq24262_masked_write(chip->client, R01_CONTROL_REG,
    		HZ_MODE, val);
    if (ret) {
       pr_err("failed to set CHG_CONFIG ret=%d\n", ret);
        return ret;
    }

    return 0;
}

#define HZ_MOCE_GET 1
static int bq24262_get_hz_mode(struct bq24262_chip *chip)
{
    int ret,value;
    u8 val;

    ret = bq24262_read_reg(chip->client, R01_CONTROL_REG, &val);
    if (ret) {
       pr_err("failed to set CHG_CONFIG ret=%d\n", ret);
        return ret;
    }
	value = (int)(val & HZ_MOCE_GET);
	pr_info("hz mode = %d\n",value);
    return value;
}


#define VBAT_MAX_MV  4440
#define VBAT_MIN_MV  3500
#define VBAT_STEP_MV  10

static int bq24262_set_vbat_max(struct bq24262_chip *chip, int mv)
{
	u8 reg_val = 0;
	int set_vbat = 0;

	if (mv < VBAT_MIN_MV || mv > VBAT_MAX_MV) {
		pr_err("bad mv=%d asked to set\n", mv);
		return -EINVAL;
	}

	reg_val = (mv - VBAT_MIN_MV)/VBAT_STEP_MV;
	set_vbat = reg_val * VBAT_STEP_MV + VBAT_MIN_MV;
	reg_val = reg_val << 1;

	pr_debug("req_vbat = %d set_vbat = %d reg_val = 0x%02x\n",
				mv, set_vbat, reg_val);

	return bq24262_masked_write(chip->client, R02_CONTROL_BAT_VOL_REG,
			VBREG_MASK, reg_val);
}


#define IBAT_MAX_MA  3000
#define IBAT_MIN_MA  500
#define IBAT_STEP_MA  100
static int bq24262_set_ibat_max(struct bq24262_chip *chip, int ma)
{
	u8 reg_val = 0;
	int set_ibat = 0;

	if (ma < IBAT_MIN_MA || ma > IBAT_MAX_MA) {
		pr_err("bad mA=%d asked to set\n", ma);
		return -EINVAL;
	}

	reg_val = (ma - IBAT_MIN_MA)/IBAT_STEP_MA;
	set_ibat = reg_val * IBAT_STEP_MA + IBAT_MIN_MA;
	reg_val = reg_val << 3;
	chip->set_chg_current_ma = set_ibat;
	pr_debug("req_ibat = %d set_ibat = %d reg_val = 0x%02x\n",
				ma, set_ibat, reg_val);

	return bq24262_masked_write(chip->client, R04_BAT_TERM_FAST_CHARGE_CUR_REG,
			ICHG_MASK, reg_val);
}

#define ITERM_MIN_MA  50
#define ITERM_MAX_MA  400
#define ITERM_STEP_MA  50
static int bq24262_set_term_current(struct bq24262_chip *chip, int ma)
{
	u8 reg_val = 0;
	int set_ma = 0;

	if (ma < ITERM_MIN_MA || ma > ITERM_MAX_MA) {
		pr_err("bad mv=%d asked to set\n", ma);
		return -EINVAL;
	}

	reg_val = (ma - ITERM_MIN_MA)/ITERM_STEP_MA;
	set_ma = reg_val * ITERM_STEP_MA + ITERM_MIN_MA;

	pr_debug("req_i = %d set_i = %d reg_val = 0x%02x\n",
				ma, set_ma, reg_val);

	return bq24262_masked_write(chip->client, R04_BAT_TERM_FAST_CHARGE_CUR_REG,
			ITERM_MASK, reg_val);
}


#define VIN_LIMIT_MIN_MV	4200
#define VIN_LIMIT_MAX_MV	4788
#define VIN_LIMIT_STEP_MV	84
static int bq24262_set_input_vin_limit(struct bq24262_chip *chip, int mv)
{
	u8 reg_val = 0;
	int set_vin = 0;

	if (mv < VIN_LIMIT_MIN_MV || mv > VIN_LIMIT_MAX_MV) {
		pr_err("bad mV=%d asked to set\n", mv);
		return -EINVAL;
	}

	reg_val = (mv - VIN_LIMIT_MIN_MV)/VIN_LIMIT_STEP_MV;
	set_vin = reg_val * VIN_LIMIT_STEP_MV + VIN_LIMIT_MIN_MV;

	pr_info("%s = req_vin = %d set_vin = %d reg_val = 0x%02x\n", __func__,
				mv, set_vin, reg_val);

	return bq24262_masked_write(chip->client, R05_VINDPM_VOL_DPPM_STAT_REG,
			VINDPM_MASK, reg_val);
}


static int bq24262_safetime_setting(struct bq24262_chip *chip, u8 tmr_n_ts_set)
{
	return bq24262_masked_write(chip->client, R06_SAFETY_TMR_NTC_MON_REG, tmr_n_ts_set, tmr_n_ts_set);
}


static bool bq24262_is_otg_mode(struct bq24262_chip *chip)
{
	u8 temp = 0;
	int ret;

	ret = bq24262_read_reg(chip->client, R00_STATUS_CONTROL_REG, &temp);
	if (ret) {
		pr_err("failed to read R00_STATUS_CONTROL_REG rc=%d\n", ret);
		return false;
	}
	pr_debug("%s = %x\n",__func__,temp);
	return ((temp & BOOST_MODE_MASK) == BOOST_MODE_MASK) ? true : false;
}

static bool bq24262_is_charger_present(struct bq24262_chip *chip)
{
	int ret = 0;
	u8 sys_status, fault_status;
	bool power_ok;

	ret = bq24262_read_reg(chip->client, R00_STATUS_CONTROL_REG, &sys_status);
	if (ret) {
		pr_err("failed to read R00_STATUS_CONTROL_REG ret=%d\n", ret);
		return false;
	}

	fault_status = (sys_status & (CHRG_STAT_MASK |CHG_FAULT_MASK));

	pr_debug("%s = %x\n",__func__,fault_status);

	if(fault_status == 0x32){
		power_ok = false;
		pr_debug("DC is disconnect\n");
	}else if(fault_status == 0x20){
	/* Low Supply connected */
		power_ok = true;
		pr_debug("DC is missing\n");
	}else if( (fault_status == BQ_FAULT_NORMAL)
		|| (fault_status == BQ_FAULT_LOW_SUPPLY)){
		power_ok = false;
		pr_debug("DC is missing\n");
	} else {
		power_ok = true;
		pr_debug("DC is present.\n");
	}

	return power_ok;
}


static int bq24262_get_prop_charge_type(struct bq24262_chip *chip)
{
	int ret = 0;
	u8 sys_status = 0;
	enum bq24262_chg_status status;
	int chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	ret = bq24262_read_reg(chip->client, R00_STATUS_CONTROL_REG, &sys_status);
	if (ret) {
		pr_err("fail to read R00_STATUS_CONTROL_REG. ret=%d\n", ret);
		chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		goto exception_handling;
	}

	sys_status = (sys_status & CHRG_STAT_MASK) >> 4;
	if(sys_status==0x00 || sys_status==0x02){
		chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		status = BQ_CHG_STATUS_NONE;
	}
	else if (sys_status == 0x01) {
		chg_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		status = BQ_CHG_STATUS_FAST_CHARGE;
	}
	else {	/* Fault */
		if(slimport_is_connected()){
			pr_info("%s : Slimport is connected so Charging enable\n",__func__);
			chg_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
			status = BQ_CHG_STATUS_FAST_CHARGE;
		}
		else{
			chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
			status = BQ_CHG_STATUS_EXCEPTION;
		}
	}
	pr_debug("bq-chg-status (%d=%s).\n", status, bq24262_chg_status[status]);

	if (chip->chg_status != status) {
		if( (status == BQ_CHG_STATUS_NONE) ||(status == BQ_CHG_STATUS_EXCEPTION)) {
			pr_info("Charging stopped.\n");
			if(status==BQ_CHG_STATUS_EXCEPTION)
				wake_unlock(&chip->chg_wake_lock);
			if(wake_lock_active(&chip->eoc_wake_lock))
				wake_unlock(&chip->eoc_wake_lock);
//			if(wake_lock_active(&chip->monitor_batt_temp_wake_lock))
//				wake_unlock(&chip->monitor_batt_temp_wake_lock);
		} else {
//			chip->eoc_check_count = 0;
			pr_info("Charging started.\n");
			wake_lock(&chip->chg_wake_lock);
		}
		chip->chg_status = status;
	}

#ifdef REGISGER_DUMP
	bq24262_get_register(chip);
#endif

	return chg_type;

exception_handling:
	chip->chg_status = BQ_CHG_STATUS_EXCEPTION;
	if (wake_lock_active(&chip->chg_wake_lock)) {
		pr_err("exception_handling : unlock chg_wake_lock.\n");
		wake_unlock(&chip->chg_wake_lock);
	}
	return chg_type;

}

#if 0//leekj
#define VAVG_VOLT_SAMPLES    5
#define PLUS_REG_VOLT_LEVEL  20
#define VAVG_LOW_EOC		4235
#define VAVG_MIDDLE_EOC		4255

static int bq24262_eoc_volt_check(struct bq24262_chip *chip)
{
	int ret,i;
	static int vavg_samples[VAVG_VOLT_SAMPLES];
	int vavg_milivolt = 0, plus_reg_voltage;

	msleep(30000);
	for(i = 0; i < VAVG_VOLT_SAMPLES; i++){
		vavg_samples[i] = __max17048_get_voltage();
		vavg_milivolt += vavg_samples[i];
		mdelay(10);
	}

	vavg_milivolt /= VAVG_VOLT_SAMPLES;

	for(i = 0; i < VAVG_VOLT_SAMPLES; i++)
		pr_info("table %d (%d)mV avg = %d mV\n",i,vavg_samples[i],vavg_milivolt);

	if((VAVG_LOW_EOC < vavg_milivolt) && (vavg_milivolt < VAVG_MIDDLE_EOC))
		plus_reg_voltage = 1;
	else if(VAVG_LOW_EOC > vavg_milivolt )
		plus_reg_voltage = 2;
	else
		plus_reg_voltage = 0;

	/* regulation voltage :: 4.30V */
	ret = bq24262_set_vbat_max(chip, (chip->regulation_mV + (PLUS_REG_VOLT_LEVEL * plus_reg_voltage)));
	if (ret)
		pr_err("failed to set vbat max\n");

	if(plus_reg_voltage){
		 bq24262_enable_charging(chip,false);
		 mdelay(5);
		 bq24262_enable_charging(chip,true);
	}else{
		if(wake_lock_active(&chip->eoc_wake_lock))
			wake_unlock(&chip->eoc_wake_lock);
	}

	return ret;
}
#endif

#define ORIG_CAPACITY_PULL		94
static void bq24262_eoc_soc_check(struct bq24262_chip *chip)
{

	int orig_capacity;
	//extern void msm_otg_unlock(void);
	//extern void usb_bus_active_unlock(void);

	orig_capacity = bq24262_get_prop_batt_orig_capacity(chip);
	if(orig_capacity < ORIG_CAPACITY_PULL){
		pr_info("%s : Not End of charging state recharging again\n", __func__);
		mdelay(5);
		bq24262_enable_charging(chip,false);
		mdelay(5);
		bq24262_enable_charging(chip,true);
	}else{
		pr_info("%s : End of charging_state = Chargerlogo = %d \n",__func__, chargerlogo_state);
		if(chargerlogo_state){
			pr_info("%s : End of charging in Chargerlogo_state -> release wake_lock \n",__func__);
			if(wake_lock_active(&chip->eoc_wake_lock))
					wake_unlock(&chip->eoc_wake_lock);
			if(wake_lock_active(&chip->chg_wake_lock))
					wake_unlock(&chip->chg_wake_lock);
		}
//		else
//		if(wake_lock_active(&chip->eoc_wake_lock))
//					wake_unlock(&chip->eoc_wake_lock);
	}
	return;
}


//#define EOC_FULL_COUNT	3
static void bq24262_irq_worker(struct work_struct *work)
{
	struct bq24262_chip *chip =
		container_of(work, struct bq24262_chip, irq_work.work);
	u8 reg_val = 0;
	int ret = 0, cable_present = 0;

	ret = bq24262_read_reg(chip->client, R00_STATUS_CONTROL_REG, &reg_val);
	if (ret) {
		pr_err("failed to read R00_STATUS_CONTROL_REG. val=%d\n", ret);
		return;
	}
	pr_info("R00_STATUS_CONTROL_REG: 0x%2x\n",reg_val);

	bq24262_get_state(chip, reg_val);

	pr_info("BQcharger State [%s]\n",chg_state_str[chip->chg_status]);

	if(chip->chg_cable_status == BQ_CHG_STATUS_PRE_CHARGE)
        bq24262_get_register(chip);

	/* If register 00 was 0x20, The charging state done. It is mean EOC */
	if(chip->chg_cable_status == BQ_CHG_STATUS_FAST_CHARGE
		&& chip->fault_state == BQ_FAULT_NORMAL){
			pr_err("Battery End Of Charging\n");

#if 0
			chip->eoc_check_count++;
			if(chip->eoc_check_count > EOC_FULL_COUNT){
				if(wake_lock_active(&chip->eoc_wake_lock))
					wake_unlock(&chip->eoc_wake_lock);
				goto IRQ_PASS;
			}
			ret = bq24262_eoc_volt_check(chip);
			if (ret) {
				pr_err("failed to set vbat max\n");
			}
#endif
			reg_vol_set_count=0;
			bq24262_eoc_soc_check(chip);
			goto IRQ_PASS;

	}else if(chg_batt_temp_state != CHG_BATT_STOP_CHARGING_STATE){
		 bq24262_enable_charging(chip,true);
		if(slimport_is_connected()){
			pr_info("%s : Slimport is connected so usb_online\n",__func__);
			power_supply_set_online(chip->usb_psy, true);
		}
	}

	//cable_present = bq24262_is_charger_present(chip);
	if(chip->chg_cable_status == BQ_CHG_STATUS_EXCEPTION
		&& chip->fault_state == BQ_FAULT_LOW_SUPPLY){
		cable_present = false;
		if(chargerlogo_state){
			if(!chip->ac_present){
				power_supply_set_online(chip->usb_psy, chip->ac_present);
				power_supply_set_online(&(chip->ac_psy), chip->ac_present);
			}
		}
		bq24262_set_clear_reg(chip);
//		chip-> = 0;

	}else{
		pr_debug("%s : Cable present -> True \n", __func__);
		cable_present = true;
	}

	/* stop charging from setting battery of temp, go to pass. */
	if(chg_batt_temp_state == CHG_BATT_STOP_CHARGING_STATE
        && last_stop_charging == 1 && cable_present){
		goto IRQ_PASS;
        }

 //	cable_present = bq24262_is_charger_present(chip);
	if (chip->ac_present ^ cable_present) {
		wake_lock_timeout(&chip->uevent_wake_lock, HZ*2);
		bq24262_notify_usb_of_the_plugin_event(cable_present);
		chip->ac_present = cable_present;
	}

IRQ_PASS:
    pr_info("%s : IRQ_PASS \n",__func__);

	if(!chip->ac_present){
		power_supply_set_online(chip->usb_psy, chip->ac_present);
		power_supply_set_online(&(chip->ac_psy), chip->ac_present);
	}
	return;
}

#if 0
static int bq24262_is_eoc_state(struct bq24262_chip * chip)
{
	int ret;
	u8	r01_status,r05_status;

	ret = bq24262_read_reg(chip->client, R01_CONTROL_REG, &r01_status);
	if (ret) {
		pr_err("failed to read R00_STATUS_CONTROL_REG. val=%d\n", ret);
		return ret;
	}

	ret = bq24262_read_reg(chip->client, R05_VINDPM_VOL_DPPM_STAT_REG , &r05_status);
	if (ret) {
		pr_err("failed to read R00_STATUS_CONTROL_REG. val=%d\n", ret);
		return ret;
	}
	r01_status &= HZ_MODE;
	r05_status &= CD_STATUS;

	pr_info(" Reg 0x%x val = 0x%x, Reg 0x%x val = 0x%x\n",R01_CONTROL_REG, r01_status,
		R05_VINDPM_VOL_DPPM_STAT_REG, r05_status);

	if((r01_status & HZ_MODE) && (r05_status & CD_STATUS))
		return true;
	else
		return false;
}
#endif

/* To detect DPM mode, Check the Register05 */
//#define DPM_FULL_COUNT 4
static void bq24262_dpm_detect_work(struct work_struct *work)
{
	struct bq24262_chip *chip =
		container_of(work, struct bq24262_chip, dpm_detect_work.work);

	int ret_05 =0, delay =0;
	int dpm_val = bq24262_get_input_i_limit(chip);
	u8 status =0;
#if 0
	int dpm_check [DPM_FULL_COUNT];
	int i=0;
#endif
	int dpm_enable=0;
	ret_05 = bq24262_read_reg(chip->client, R05_VINDPM_VOL_DPPM_STAT_REG , &status);
	pr_info("Reg05 0x%x val = 0x%x\n", R05_VINDPM_VOL_DPPM_STAT_REG, status);
	dpm_enable=bq24262_get_dpm_state(chip);
	pr_info("%s : DPM is %d\n", __func__, dpm_enable);

		switch(dpm_count){
			case DPM_CHECK_START:
					bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_900mA);
					bq24262_set_ibat_max(chip, INPUT_CURRENT_LIMIT_900mA);
					chip->set_chg_step = INPUT_CURRENT_LIMIT_900mA;
					pr_info("%s : Start ->DPM is not active\n", __func__);
					pr_info("%s : Start -> Set current 500 to 900\n", __func__);
					dpm_count = DPM_CHECK_FIRST;
					delay = 500;

			break;

			case DPM_CHECK_FIRST:
					if(bq24262_get_dpm_state(chip)){
						bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_500mA);
						bq24262_set_ibat_max(chip, INPUT_CURRENT_LIMIT_500mA);
						chip->set_chg_step = INPUT_CURRENT_LIMIT_500mA;
						pr_info("%s : First ->DPM is active >> 900mA : active \n", __func__);
						pr_info("%s : First ->Set current %d to 500\n", __func__, dpm_val);
						dpm_count = DPM_CHECK_DONE;
						delay = 0;
					}
					else{
						bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_1500mA);
						bq24262_set_ibat_max(chip, INPUT_CURRENT_LIMIT_1500mA);
						chip->set_chg_step = INPUT_CURRENT_LIMIT_1500mA;
						pr_info("%s : First ->DPM is not active >> 900mA : non -active \n", __func__);
						pr_info("%s : First -> Set current %d to 1500\n", __func__, dpm_val);
						dpm_count = DPM_CHECK_SECOND;
						delay = 500;
					}

			break;

			case DPM_CHECK_SECOND:
					if(bq24262_get_dpm_state(chip)){
						bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_900mA);
						bq24262_set_ibat_max(chip, INPUT_CURRENT_LIMIT_900mA);
						chip->set_chg_step = INPUT_CURRENT_LIMIT_900mA;
						pr_info("%s : Second->DPM is active>>1500mA : active \n", __func__);
						pr_info("%s : Second ->Set current %d to 900\n", __func__, dpm_val);
						dpm_count = DPM_CHECK_DONE;
						delay = 0;
					}
					else{
						pr_info("%s : Second->DPM is not active>> 1500mA : non-active\n", __func__);
						pr_info("%s : Second-> Set current %d to 1500\n", __func__, dpm_val);
						dpm_count = DPM_CHECK_DONE;
						delay = 0;
					}
			break;

			case DPM_CHECK_DONE:
					if(dpm_enable == true && dpm_val == INPUT_CURRENT_LIMIT_500mA){
						pr_info("%s : DPM is active Enable to slow charging\n", __func__);
						vzw_fast_chg_ma = 600;
						set_vzw_charging_state();
						return;
					}
					else if(dpm_enable == true && (dpm_val == INPUT_CURRENT_LIMIT_1500mA)){
						bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_900mA);
						bq24262_set_ibat_max(chip, INPUT_CURRENT_LIMIT_900mA);
						chip->set_chg_step = INPUT_CURRENT_LIMIT_900mA;
						pr_info("Final step : DPM is active in 1500mA, set 1500 to 900\n");
						dpm_retry_count++;
						dpm_count=0;
						return;
					}
					else{
						dpm_retry_count++;
						dpm_count=0;
						pr_info("DPM_detect_work done: Not slow charging\n");
						return;
					}
			break;

			default:
				pr_info("DPM_detect_work: DPM detection done\n");
			return;
			}
			queue_delayed_work(system_nrt_wq, &chip->dpm_detect_work, msecs_to_jiffies(delay));

}

static int bq24262_get_dpm_state(struct bq24262_chip * chip)
{
	int ret;
	u8 status;

	ret = bq24262_read_reg(chip->client, R05_VINDPM_VOL_DPPM_STAT_REG , &status);
	if (ret) {
		pr_err("failed to read R05_VINDPM_VOL_DPPM_STAT_REG. val=%d\n", ret);
		return ret;
	}
	status &= DPM_STATUS;
	status=status >>4;

	pr_debug("Reg05 0x%x val = 0x%x\n", R05_VINDPM_VOL_DPPM_STAT_REG, status);

	if(status == 0x04){
		return true;
	}
	else
		return false;
}


int bq24262_is_charger_plugin(void)
{
	int cable_present = 0;
	pr_debug("%s \n",__func__);

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	cable_present = bq24262_is_charger_present(the_chip);
	//the_chip->ac_present = cable_present;
	pr_info("%s %d \n",__func__,cable_present);
//	usb_present = bq24262_is_charger_present(the_chip);
	//return the_chip->ac_present;
	return cable_present;
}

static int bq24262_get_prop_batt_present(struct bq24262_chip *chip)
{
#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
	int batt_present = 0;
	int rc = 0;
	struct qpnp_vadc_result results;

#ifdef CONFIG_MACH_MSM8974_Z_KR
	if(lge_get_board_revno() == HW_REV_D)
		return 1;
#endif
	rc = qpnp_vadc_read(LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_err("Unable to read batt term rc=%d\n", rc);
		return 0;
	}

	/* We can check battery presence by reading REG00[FAULT_x]*/
	if(results.physical <= -300) {
		pr_err("Battery Removed!!!\n");
		batt_present = 0;
	}
	else {
		batt_present = 1;
	}

	return batt_present;
#else
#ifdef CONFIG_MACH_APQ8064_ALTEV

	pr_debug("ALTE was embeded battery.\n");
	return 1;

#else
	pr_err("CONFIG_SENSORS_QPNP_ADC_VOLTAGE is not defined.\n");
	return batt_present;
#endif
#endif
}

#ifdef CONFIG_MACH_APQ8064_ALTEV
#define VZW_TA_DETECTING_CURRENT 900
#define VZW_CHG_RATED_CURRENT 850
#define VZW_CHG_DPM_CURRENT 600
#define VZW_CHG_MIN_CURRENT 500
void set_vzw_charging_state(void)
{
	if((!slimport_is_connected()) && (!is_factory_cable())){
		// incompatible charger, impossible to charge
		if (vzw_fast_chg_ma >0 && vzw_fast_chg_ma < VZW_CHG_MIN_CURRENT) {
			chg_state = VZW_INCOMPATIBLE_CHG;
			pr_info("%s : Incompatible=VZW_CHG_STATE ======  %d\n",__func__, chg_state);
			bq24262_set_hz_mode(the_chip, true);
		}
		// under current charging, DPM mode is active
		else if(vzw_fast_chg_ma==VZW_CHG_DPM_CURRENT){
			chg_state = VZW_UNDER_CURRENT_CHG;
			pr_info("%s : DPM is active=VZW_CHG_STATE ======  %d\n",__func__, chg_state);
			dpm_count =0 ;
		}
		// Stop charging animation on USB enumeration
		else if(vzw_fast_chg_ma==VZW_TA_DETECTING_CURRENT){
			chg_state = VZW_TA_DETECTING;
			pr_info("%s : Detecting=VZW_CHG_STATE ======  %d\n",__func__, chg_state);
		}
		// compatible charger, possible to charge in rated current
		else{
			chg_state = VZW_COMPATIBLE_CHG;
			pr_info("%s : Compatible=VZW_CHG_STATE ======  %d\n",__func__, chg_state);
		}

		power_supply_changed(&the_chip->batt_psy);
	}
	else{
		chg_state=VZW_COMPATIBLE_CHG;
		pr_info("%s=compatible=VZW_CHG_STATE(%d)(slimport and Charger)\n",__func__, chg_state);
		power_supply_changed(&the_chip->batt_psy);
		}
		return;
}
EXPORT_SYMBOL(set_vzw_charging_state);
#endif

#ifdef CONFIG_THERMAL_QPNP_ADC_TM
static void bq24262_vbat_work(struct work_struct *work)
{
	struct bq24262_chip *chip =
		container_of(work, struct bq24262_chip, vbat_work.work);
	int step_current_ma;
	pr_debug("%s \n",__func__);

	if (chip->vbat_noti_stat == ADC_TM_HIGH_STATE) {
		step_current_ma = chip->step_dwn_currnet_ma;
		chip->adc_param.state_request = ADC_TM_LOW_THR_ENABLE;
	} else {
		step_current_ma = chip->chg_current_ma;
		chip->adc_param.state_request = ADC_TM_HIGH_THR_ENABLE;
	}

	if (bq24262_is_charger_present(chip)) {
		pr_debug("change chg current step to %d\n", step_current_ma);
		bq24262_set_ibat_max(chip, step_current_ma);
		qpnp_adc_tm_channel_measure(&chip->adc_param);
	}
	wake_unlock(&chip->chg_wake_lock);
}

static void bq24262_vbat_notification(enum qpnp_tm_state state, void *ctx)
{
	struct bq24262_chip *chip = ctx;

	wake_lock(&chip->chg_wake_lock);
	chip->vbat_noti_stat = state;
	schedule_delayed_work(&chip->vbat_work, msecs_to_jiffies(100));
}

static int bq24262_step_down_detect_init(struct bq24262_chip *chip)
{
	int ret;

	ret = qpnp_adc_tm_is_ready();
	if (ret) {
		pr_err("qpnp_adc is not ready");
		return ret;
	}

	chip->adc_param.high_thr = chip->step_dwn_thr_mv * 1000;
	chip->adc_param.low_thr = (chip->step_dwn_thr_mv - 100) * 1000;
	chip->adc_param.timer_interval = ADC_MEAS1_INTERVAL_2S;
	chip->adc_param.state_request = ADC_TM_HIGH_THR_ENABLE;
	chip->adc_param.btm_ctx = chip;
	chip->adc_param.threshold_notification = bq24262_vbat_notification;
	chip->adc_param.channel = VBAT_SNS;

	ret = qpnp_adc_tm_channel_measure(&chip->adc_param);
	if (ret)
		pr_err("request ADC error %d\n", ret);

	return ret;
}
//#else
#if 0
static void bq24262_vbat_work(struct work_struct *work)
{
	pr_warn("vbat notification is not suppored!\n");
}
#endif
//static void bq24262_vbat_notification(int state, void *ctx)
//{
//	pr_warn("vbat notification is not suppored!\n");
//}
#if 0
static int bq24262_step_down_detect_init(struct bq24262_chip *chip)
{
	pr_warn("vbat notification is not suppored!\n");
	return 0;
}
#endif
#endif

static void bq24262_remove_debugfs_entries(struct bq24262_chip *chip)
{
	if (chip->dent)
		debugfs_remove_recursive(chip->dent);
}

static int __devinit bq24262_init_batt_psy(struct bq24262_chip *chip)
{
	int ret;

	chip->batt_psy.name = "battery";
	chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.properties = bq24262_batt_power_props;
	chip->batt_psy.num_properties =
					ARRAY_SIZE(bq24262_batt_power_props);
	chip->batt_psy.get_property = bq24262_batt_power_get_property;
	chip->batt_psy.set_property = bq24262_batt_power_set_property;
	chip->batt_psy.property_is_writeable =
					bq24262_batt_power_property_is_writeable;
	chip->batt_psy.external_power_changed =
					bq24262_batt_external_power_changed;

	ret = power_supply_register(&chip->client->dev,
				&chip->batt_psy);
	if (ret) {
		pr_err("failed to register power_supply. ret=%d.\n", ret);
		return ret;
	}

	return 0;
}

static int __devinit bq24262_init_usb_psy(struct bq24262_chip *chip)
{
	int ret = 0;

//	chip->usb_psy->name = "bq24262-usb";
	chip->usb_psy->name = "usb";
	chip->usb_psy->type = POWER_SUPPLY_TYPE_USB;
	chip->usb_psy->supplied_to = bq24262_power_supplied_to;
	chip->usb_psy->num_supplicants = ARRAY_SIZE(bq24262_power_supplied_to);
	chip->usb_psy->properties = bq24262_power_props_usb;
	chip->usb_psy->num_properties = ARRAY_SIZE(bq24262_power_props_usb);
	chip->usb_psy->get_property = bq24262_power_get_property_usb;
	chip->usb_psy->set_property = bq24262_power_set_property_usb;
	ret = power_supply_register(&chip->client->dev,
				chip->usb_psy);
	if (ret) {
		pr_err("failed to register power_supply. ret=%d.\n", ret);
		return ret;
	}

	return 0;
}

static int __devinit bq24262_init_ac_psy(struct bq24262_chip *chip)
{
	int ret = 0;

//	chip->ac_psy.name = "bq24262-ac";
	chip->ac_psy.name = "pm8921-dc";
	chip->ac_psy.type = POWER_SUPPLY_TYPE_MAINS;
	chip->ac_psy.supplied_to = bq24262_power_supplied_to;
	chip->ac_psy.num_supplicants = ARRAY_SIZE(bq24262_power_supplied_to);
	chip->ac_psy.properties = bq24262_power_props;
	chip->ac_psy.num_properties = ARRAY_SIZE(bq24262_power_props);
	chip->ac_psy.get_property = bq24262_power_get_property;
	chip->ac_psy.set_property = bq24262_power_set_property;
	chip->ac_psy.property_is_writeable =
				bq24262_power_property_is_writeable;
	ret = power_supply_register(&chip->client->dev,
				&chip->ac_psy);
	if (ret) {
		pr_err("failed to register power_supply. ret=%d.\n", ret);
		return ret;
	}

	return 0;
}


#define EN_CHG_TERM_ENABLE	0x1

static int bq24262_hw_init(struct bq24262_chip *chip)
{
	int ret = 0;

	ret = bq24262_enable_otg(chip,false);
	if (ret) {
		pr_err("failed to set input voltage limit\n");
		return ret;
	}

	if(is_factory_cable_56k() || is_factory_cable_910k())
		ret = bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_1500mA);
	else if(is_factory_cable_130k())
		ret = bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_500mA);
	else
		ret = bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_500mA);

	if (ret) {
		pr_err("failed to set input current limit\n");
		return ret;
	}

	/* regulation voltage :: 4.32V */
	ret = bq24262_set_vbat_max(chip, chip->regulation_mV);
	if (ret) {
		pr_err("failed to set vbat max\n");
		return ret;
	}

	ret = bq24262_set_ibat_max(chip, chip->chg_current_ma);
	if (ret) {
		pr_err("failed to set charging current\n");
		return ret;
	}

	ret = bq24262_set_term_current(chip, chip->term_current_ma);
	if (ret) {
		pr_err("failed to set charge termination current\n");
		return ret;
	}

	/* Enable Charge Current Termination*/
	ret = bq24262_enable_charger_current_termination(chip, EN_CHG_TERM_ENABLE);
	if (ret) {
		pr_err("failed to enable chg termination\n");
		return ret;
	}

	/* Enable Interrupt*/
	ret = bq24262_enable_interrupt(chip, true);
	if (ret) {
		pr_err("failed to enable interrupt\n");
		return ret;
	}

	/* VINDPM */
	ret = bq24262_set_input_vin_limit(chip, 4620);
	if (ret) {
		pr_err("failed to set input voltage limit\n");
		return ret;
	}

	ret = bq24262_safetime_setting(chip,(TMR_BIT|(!TS_EN)));

//	if(is_factory_cable_130k())
//		bq24262_enable_charging(chip,false);

#ifdef REGISGER_DUMP
	bq24262_get_register(chip);
#endif

	return 0;
}

static void cable_init_termination(struct bq24262_chip *chip)
{
	chip->ac_present = bq24262_is_charger_present(chip);
	pr_debug("%s cable present = %d\n",__func__,chip->ac_present);
	bq24262_notify_usb_of_the_plugin_event(chip->ac_present);
	bq24262_enable_charging(chip, true);
}


static irqreturn_t bq24262_irq(int irq, void *dev_id)
{
	struct bq24262_chip *chip = dev_id;
	pr_debug("%s\n",__func__);
	schedule_delayed_work(&chip->irq_work, msecs_to_jiffies(0));

	return IRQ_HANDLED;
}

static int bq24262_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	const struct bq24262_platform_data *pdata;
//	struct device_node *dev_node = client->dev.of_node;
	struct bq24262_chip *chip;
	int ret = 0;

	unsigned int *p_cable_type = (unsigned int *)
		(smem_get_entry(SMEM_ID_VENDOR1, &cable_smem_size));

	if (p_cable_type)
		cable_type = *p_cable_type;
	else
		cable_type = 0;

	pr_debug("cable_type is = %d\n", cable_type);

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("i2c func fail.\n");
		return -EIO;
	}

	chip = kzalloc(sizeof(struct bq24262_chip), GFP_KERNEL);
	if (!chip) {
		pr_err("failed to alloc memory\n");
		return -ENOMEM;
	}

	chip->client = client;

	chip->usb_psy = kzalloc(sizeof(struct power_supply), GFP_KERNEL);
	if (!chip) {
		pr_err("failed to alloc memory\n");
		return -ENOMEM;
	}

		pdata = client->dev.platform_data;
		if (pdata == NULL) {
			pr_err("no platform data.\n");
			return -EINVAL;
		}

		chip->int_gpio = pdata->int_gpio;
		chip->chg_current_ma = pdata->chg_current_ma;
		chip->term_current_ma = pdata->term_current_ma;
		//chip->step_dwn_thr_mv = pdata->step_dwn_thr_mv;
		//chip->step_dwn_currnet_ma = pdata->step_dwn_currnet_ma;
//	}

	chip->set_chg_current_ma = chip->chg_current_ma;
	chip->temp_level_1 = pdata->temp_level_1;
	chip->temp_level_2 = pdata->temp_level_2;
	chip->temp_level_3 = pdata->temp_level_3;
	chip->temp_level_4 = pdata->temp_level_4;
	chip->temp_level_5 = pdata->temp_level_5;
	chip->thermal_mitigation = pdata->thermal_mitigation;
	chip->thermal_levels = pdata->thermal_levels;
	chip->regulation_mV = pdata->regulation_mV;

	last_stop_charging = 0;
	chg_batt_temp_state = CHG_BATT_NORMAL_STATE;
	pseudo_ui_charging = 0;
	last_usb_chg_current = 0;
	pre_mitigation = 0;
	soc_limit = 0;
	dcreasing_charging = 0;

	register_reboot_notifier(&set_default_notifier);

	ret =  gpio_request_one(chip->int_gpio, GPIOF_DIR_IN,
			"bq24262_int");
	if (ret) {
		pr_err("failed to request int_gpio ret = %d\n", ret);
		goto error;
	}

	chip->irq = gpio_to_irq(chip->int_gpio);
	pr_debug("int_gpio irq#=%d.\n", chip->irq);

	i2c_set_clientdata(client, chip);

	ret = bq24262_hw_init(chip);
	if (ret) {
		pr_err("bq24262_hwinit failed.ret=%d\n",ret);
		goto err_hw_init;
	}

#if 0 //                 
		/* Initialize wake lock for deliver Uevent before suspend */
		/* Move it to before IRQ request because very very rarely it can be called by IRQ before initialized */
		wake_lock_init(&chip->deliver_uevent_wake_lock,WAKE_LOCK_SUSPEND, "deliver_uevent");
#endif

	wake_lock_init(&chip->chg_wake_lock,
		        WAKE_LOCK_SUSPEND, BQ24262_NAME);
	wake_lock_init(&chip->uevent_wake_lock,
			WAKE_LOCK_SUSPEND, "bq24262_chg_uevent");

#if 0
	wake_lock_init(&chip->uevent_wake_lock,
		       WAKE_LOCK_SUSPEND, "bq24262_chg_uevent");
 /*                                    */
	wake_lock_init(&chip->lcs_wake_lock,
			WAKE_LOCK_SUSPEND, "LGE charging scenario");
#endif

	ret = bq24262_init_batt_psy(chip);
	if (ret) {
		pr_err("bq24262_init_batt_psy failed ret=%d\n", ret);
		goto err_init_batt_psy;
	}

	ret = bq24262_init_ac_psy(chip);
	if (ret) {
		pr_err("bq24262_init_ac_psy failed ret=%d\n", ret);
		goto err_init_ac_psy;
	}

	ret = bq24262_init_usb_psy(chip);
	if (ret) {
		pr_err("bq24262_init_usb_psy failed ret=%d\n", ret);
		goto err_init_ac_psy;
	}

	INIT_DELAYED_WORK(&chip->update_heartbeat_work, update_heartbeat);
	schedule_delayed_work(&chip->update_heartbeat_work,
							round_jiffies_relative(msecs_to_jiffies
							(UPDATE_TIME_MS)));

	the_chip = chip;

	cable_init_termination(chip);

	if (is_factory_cable()) {
		if (is_factory_cable_130k())
			pr_debug("Factory cable detected(130k)\n");
		 else
			pr_debug("Factory cable detected(not 130k)\n");
		bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_1500mA);
		bq24262_set_ibat_max(chip, INPUT_CURRENT_LIMIT_500mA);
	}

	ret = bq24262_create_debugfs_entries(chip);
	if (ret) {
		pr_err("bq24262_create_debugfs_entries failed ret=%d\n", ret);
		goto err_debugfs;
	}

//	INIT_DELAYED_WORK(&chip->vbat_work, bq24262_vbat_work);


	INIT_DELAYED_WORK(&chip->irq_work, bq24262_irq_worker);
	INIT_DELAYED_WORK(&chip->dpm_detect_work, bq24262_dpm_detect_work);

#if 1 /*                                    */
	wake_lock_init(&chip->monitor_batt_temp_wake_lock,
					WAKE_LOCK_SUSPEND,"bq24262_monitor_batt_temp");
	wake_lock_init(&chip->eoc_wake_lock, WAKE_LOCK_SUSPEND, "bq24262_irq_worker");

	INIT_DELAYED_WORK(&chip->battemp_work, bq24262_monitor_batt_temp);
#endif

	ret = request_irq(chip->irq, bq24262_irq,
			(IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING|IRQF_ONESHOT),
			"bq24262_irq", chip);
	if (ret) {
		pr_err("request_irq %d failed\n", chip->irq);
		goto err_req_irq;
	}
	enable_irq_wake(chip->irq);

	ret = device_create_file(&client->dev, &dev_attr_at_charge);
	if (ret < 0) {
		pr_err("%s:File dev_attr_at_charge creation failed: %d\n",
				__func__, ret);
		ret = -ENODEV;
		goto err_at_charge;
	}
	ret = device_create_file(&client->dev, &dev_attr_at_current1);
	if (ret < 0) {
		pr_err("%s:File dev_attr_at_current creation failed: %d\n",
				__func__, ret);		ret = -ENODEV;
		goto err_at_current;
	}
	ret = device_create_file(&client->dev, &dev_attr_at_vcoin);
	if (ret < 0) {
		pr_err("%s:File dev_attr_at_current creation failed: %d\n",
				__func__, ret);		ret = -ENODEV;
		goto err_at_vcoin;
	}

	ret = device_create_file(&client->dev, &dev_attr_at_chcomp1);
	if (ret < 0) {
		pr_err("%s:File dev_attr_at_chcomp creation failed: %d\n",
				__func__, ret);
		ret = -ENODEV;
		goto err_at_chcomp;
	}

	ret = device_create_file(&client->dev, &dev_attr_at_pmrst1);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_at_pmrst;
	}

	ret = device_create_file(&client->dev, &dev_attr_at_otg);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_at_otg;
	}

#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
	last_batt_temp = DEFAULT_TEMP;
	last_batt_current = DEFAULT_CURRENT;
#endif

#if 1 /*                                    */
	schedule_delayed_work(&chip->battemp_work,
		5*HZ);
#endif
	schedule_delayed_work(&chip->irq_work, msecs_to_jiffies(2000));

	return 0;
err_at_otg:
	device_remove_file(&client->dev, &dev_attr_at_pmrst1);
err_at_pmrst:
	device_remove_file(&client->dev, &dev_attr_at_chcomp1);
err_at_chcomp:
	device_remove_file(&client->dev, &dev_attr_at_charge);
err_at_charge:
err_at_current:
	device_remove_file(&client->dev, &dev_attr_at_current1);
err_at_vcoin:
	device_remove_file(&client->dev, &dev_attr_at_vcoin);
err_req_irq:
	bq24262_remove_debugfs_entries(chip);
err_debugfs:
	power_supply_unregister(&chip->ac_psy);
err_init_ac_psy:
	power_supply_unregister(&chip->batt_psy);
err_init_batt_psy:
	wake_lock_destroy(&chip->chg_wake_lock);
	wake_lock_destroy(&chip->uevent_wake_lock);

#if 0 /*                                    */
	wake_lock_destroy(&chip->lcs_wake_lock);
#endif
err_hw_init:
	if (chip->int_gpio)
		gpio_free(chip->int_gpio);
error:
	kfree(chip);
	pr_debug("fail to probe\n");
	return ret;

}

static int bq24262_remove(struct i2c_client *client)
{
	struct bq24262_chip *chip = i2c_get_clientdata(client);

	bq24262_remove_debugfs_entries(chip);

	device_remove_file(&client->dev, &dev_attr_at_charge);
	device_remove_file(&client->dev, &dev_attr_at_current1);
	device_remove_file(&client->dev, &dev_attr_at_vcoin);
	device_remove_file(&client->dev, &dev_attr_at_chcomp1);
	device_remove_file(&client->dev, &dev_attr_at_pmrst1);
	device_remove_file(&client->dev, &dev_attr_at_otg);

	wake_lock_destroy(&chip->eoc_wake_lock);
	wake_lock_destroy(&chip->chg_wake_lock);
	wake_lock_destroy(&chip->uevent_wake_lock);
	wake_lock_destroy(&chip->monitor_batt_temp_wake_lock);
#if 0 /*                                    */
	wake_lock_destroy(&chip->lcs_wake_lock);
#endif

	power_supply_unregister(&chip->ac_psy);
	power_supply_unregister(&chip->batt_psy);

	if (chip->irq)
		free_irq(chip->irq, chip);
	if (chip->int_gpio)
		gpio_free(chip->int_gpio);

	kfree(chip);
	return 0;
}
static int bq24262_charger_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct bq24262_chip *chip = i2c_get_clientdata(client);

    mb();
    flush_work(&chip->irq_work.work);

    if ( chip && device_may_wakeup(&client->dev)) {
		enable_irq_wake(chip->irq);
    }
	cancel_delayed_work_sync(&chip->update_heartbeat_work);
//	cancel_delayed_work_sync(&chip->battemp_work);


    return 0;
}

static int bq24262_charger_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq24262_chip *chip = i2c_get_clientdata(client);

	if ( chip && device_may_wakeup(&client->dev)) {
		disable_irq_wake(chip->irq);
	}
	printk("%s : resume sucess\n",__func__);

	bq24262_enable_charging(chip,true);
	chg_batt_temp_state = CHG_BATT_NORMAL_STATE;
	schedule_delayed_work(&chip->update_heartbeat_work, 0);
	schedule_delayed_work(&chip->irq_work, msecs_to_jiffies(150));
	if(delayed_work_pending(&chip->battemp_work))
		cancel_delayed_work(&chip->battemp_work);
	schedule_delayed_work(&chip->battemp_work, msecs_to_jiffies(400));
	return 0;
}

static const struct dev_pm_ops bq24262_charger_pm_ops = {
    .suspend	= bq24262_charger_suspend,
    .resume		= bq24262_charger_resume,
};

static const struct i2c_device_id bq24262_id[] = {
	{BQ24262_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq24262_id);

static struct i2c_driver bq24262_driver = {
	.driver	= {
			.name	= BQ24262_NAME,
			.owner	= THIS_MODULE,
			.pm	= &bq24262_charger_pm_ops,
	},
	.probe		= bq24262_probe,
	.remove		= bq24262_remove,
	.id_table	= bq24262_id,
};

static int __init bq24262_init(void)
{
	int result;
	result =i2c_add_driver(&bq24262_driver);
	pr_debug("bq24262_init result %d\n", result);

	return result;
}

//module_init(bq24262_init);
late_initcall(bq24262_init);

static void __exit bq24262_exit(void)
{
	pr_debug("bq24262_exit\n");
	return i2c_del_driver(&bq24262_driver);
}
module_exit(bq24262_exit);
//module_i2c_driver(bq24262_driver);


MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("BQ24262 external charger");
MODULE_LICENSE("GPL");

MODULE_ALIAS("platform:" BQ24262_NAME);
