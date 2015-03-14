/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2012, LGE Inc.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/gpio_event.h>

#include <mach/vreg.h>
#include <mach/rpc_server_handset.h>
#include <mach/board.h>

/* keypad */
#include <linux/mfd/pm8xxx/pm8921.h>

/* i2c */
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4
#include <linux/input/touch_synaptics_rmi4_i2c.h>
#include <linux/input/lge_touch_core_ds5.h>
#endif
#include <mach/board_lge.h>

#if defined(CONFIG_RMI4_I2C)
#include <linux/input/synaptics_dsx_g2.h>
#include <linux/input/synaptics_dsx_i2c.h>
#endif


/* TOUCH GPIOS */
#define SYNAPTICS_TS_I2C_SDA                 	8
#define SYNAPTICS_TS_I2C_SCL                 	9
#define SYNAPTICS_TS_I2C_INT_GPIO            	6
#define TOUCH_RESET                             33
#define TOUCH_POWER_EN                          62

/* TOUCH I2C SETTING */
#define APQ8064_GSBI3_QUP_I2C_BUS_ID            3

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4
static struct touch_power_module touch_pwr = {
	.use_regulator	= 0,
	.vdd			= "8921_l15",
	.vdd_voltage	= 3300000,
	.vio			= "8921_l22",
	.vio_voltage	= 1800000,
};

static struct touch_device_caps touch_caps = {
	.button_support 			= 1,
	.y_button_boundary			= 0,
	.number_of_button 			= 2,
	.button_name 				= {KEY_BACK,KEY_MENU},
	.button_margin				= 0,	
	.is_width_supported 		= 1,
	.is_pressure_supported 		= 1,
	.is_id_supported			= 1,
	.max_width 					= 15,
	.max_pressure 				= 0xFF,
	.max_id						= 10,
	.lcd_x						= 1080,
	.lcd_y						= 1920,
	.x_max						= 2159, //5M 1100 before E006
	.y_max						= 3839, //5M 1900 before E006
};

static struct touch_operation_role touch_role = {
	.operation_mode 		= INTERRUPT_MODE,
	.key_type				= TOUCH_HARD_KEY,
	.report_mode			= CONTINUOUS_REPORT_MODE,
	.delta_pos_threshold 	= 1,
	.orientation 			= 0,
	.report_period			= 10000000,
	.booting_delay 			= 80,
	.reset_delay			= 5,
	.suspend_pwr			= POWER_OFF,
	.resume_pwr				= POWER_ON,
	.jitter_filter_enable	= 0,
	.jitter_curr_ratio		= 30,
	.accuracy_filter_enable = 1,
	.irqflags 				= IRQF_TRIGGER_FALLING,
	.ghost_detection_enable = 1,
};

static struct touch_platform_data omega_ts_data = {
	.int_pin	= SYNAPTICS_TS_I2C_INT_GPIO,
	.reset_pin	= TOUCH_RESET,
	.maker		= "Synaptics",
	.fw_version	= "DEFAULT",
	.caps		= &touch_caps,
	.role		= &touch_role,
	.pwr		= &touch_pwr,
};

static struct i2c_board_info synaptics_ts_info[] = {
	[0] = {
		I2C_BOARD_INFO(LGE_TOUCH_NAME, 0x20),
		.platform_data = &omega_ts_data,
		.irq = MSM_GPIO_TO_INT(SYNAPTICS_TS_I2C_INT_GPIO),
	},
	
};
#endif

#if defined(CONFIG_RMI4_I2C)
static struct synaptics_dsx_cap_button_map rmi_button = {
	.nbuttons = 2,
	.map = {KEY_BACK,KEY_MENU},
};

static struct synaptics_dsx_platform_data rmi_platformdata = {
	.x_flip = 0,
	.y_flip = 0,
	.regulator_en = 0,
	.reset_gpio = 33,
	.irq_gpio = 6,
	.irq_flags = IRQF_TRIGGER_FALLING,
	.panel_x = 1080,
	.panel_y = 1920,
	.cap_button_map = &rmi_button,
};

static struct i2c_board_info synaptics_ds4_rmi_info[] = {
     [0] = {
         I2C_BOARD_INFO("synaptics_dsx_i2c", 0x20),
	      .platform_data = &rmi_platformdata,
     },
};
#endif


void __init apq8064_init_input(void)
{
	printk(KERN_INFO "[Touch D] %s: reg synaptics driver \n",__func__);

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4)
	i2c_register_board_info(APQ8064_GSBI3_QUP_I2C_BUS_ID,
			&synaptics_ts_info[0], 1);	
#endif

// Wireless Debugging Porting
#if defined(CONFIG_RMI4_I2C) 
	printk(KERN_INFO "[Touch D] wireless debugging driver. rmi4. \n");
	i2c_register_board_info(APQ8064_GSBI3_QUP_I2C_BUS_ID,
			&synaptics_ds4_rmi_info[0], 1);
#endif

}
