/*
 * Copyright (C) 2012, Kyungtae Oh <kyungtae.oh@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 *
 */

#ifndef __LINUX_POWER_BQ51051B_CHARGER_H__
#define __LINUX_POWER_BQ51051B_CHARGER_H__

#define BQ51051B_WLC_DEV_NAME "bq51051b_wlc"

#define WLC_DEG

#ifdef WLC_DEG
#define WLC_DBG_INFO(fmt, args...) \
	pr_info("wlc: %s: " fmt, __func__, ##args)
#define WLC_DBG_ERR(fmt, args...) \
	pr_err("wlc: %s: " fmt, __func__, ##args)
#define WLC_DBG(fmt, args...) \
	pr_debug("wlc: %s: " fmt, __func__, ##args)
#else
#define WLC_DBG_INFO(fmt, args...)  do { } while(0)
#define WLC_DBG_ERR(fmt, args...)  do { } while(0)
#define WLC_DBG(fmt, arges...)      do { } while(0)
#endif

#define PM8921_GPIO_WLC_ACTIVE   17
#define PM8921_GPIO_WLC_STATE 26
#define PM8921_GPIO_WLC_TS_CTRL 7
#define GPIO_WLC_ACTIVE        PM8921_GPIO_PM_TO_SYS(PM8921_GPIO_WLC_ACTIVE)
#define GPIO_WLC_STATE         PM8921_GPIO_PM_TO_SYS(PM8921_GPIO_WLC_STATE)
#define GPIO_WLC_TS_CTRL       PM8921_GPIO_PM_TO_SYS(PM8921_GPIO_WLC_TS_CTRL)

#ifdef CONFIG_WIRELESS_CHARGER_TEMP_SCENARIO
enum {
	DISCHARGING_TEMP,
	DISCHARGING_CLEAR_TEMP,
};
#endif

struct bq51051b_wlc_platform_data {
	unsigned int chg_state_gpio;
	unsigned int active_n_gpio;
	unsigned int wc_ts_ctrl_gpio;
#ifdef CONFIG_WIRELESS_CHARGER_TEMP_SCENARIO
	unsigned int discharging_temp;
	unsigned int discharging_clear_temp;
#endif
	unsigned int wireless_charging;
	int (*wlc_is_plugged)(void);
};

void wireless_check_batt_temp(int batt_temp);

#endif
