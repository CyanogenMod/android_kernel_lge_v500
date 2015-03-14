/*
 *  Copyright (C) 2009 LG Electronics
 *  Dajin Kim <dajin.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX17048_FUELGAUGE_H_
#define __MAX17048_FUELGAUGE_H_

typedef enum {
	MAX17403_UNKNOWN,
	MAX17048_RESET,
	MAX17048_QUICKSTART,
	MAX17048_WORKING,
	MAX17048_STATE_MAX
} max17048_status;

struct max17048g_platform_data {
	unsigned int test_data;
	unsigned int (*test_func)(void);
};
/*                                                                           */
enum {
	DISABLE_MAX17048_WORK,
	ENABLE_MAX17048_WORK,
};
/*                                                                        */

/*                                                                                          */
struct max17048_ocv_to_soc_data {
	int voltage;
	int soc;
};
/*                                   */

/*                                                                                           */
struct max17048_platform_data {
	u8			starting_rcomp;
	int			temp_co_hot;
	int			temp_co_cold;
	/*                                                                                          */
	struct max17048_ocv_to_soc_data		*soc_cal_data;
	/*                                   */
};
#ifdef CONFIG_BATTERY_MAX17048
extern int __max17048_get_orig_capacity(void);
#endif
/*                                   */
extern int __max17048_get_capacity(void);
extern int __max17048_get_voltage(void);
extern int max17048_do_calibrate(void);
extern int max17048_set_rcomp_by_temperature(void);
extern int max17048_set_alert_level(int alert_level);

#endif
