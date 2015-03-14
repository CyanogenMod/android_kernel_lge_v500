/*
 * Copyright (C) 2013 LGE, Inc.
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

#ifndef __I2C_BL_H__
#define __I2C_BL_H__

void i2c_bl_lcd_backlight_set_level_export(int level);
void i2c_bl_lcd_backlight_set_level_scale(int percentage, unsigned long duration);

struct i2c_bl_cmd {
	unsigned char addr;
	unsigned char value;
	unsigned char mask;
	char *description;
};

struct i2c_bl_platform_data {
	void (*platform_init)(void);
	int gpio;
	unsigned short i2c_addr;
	int min_brightness;
	int max_brightness;
	int default_brightness;
	int factory_brightness;
    int factory_mode;

	struct i2c_bl_cmd *init_cmds;
	int init_cmds_size;

	struct i2c_bl_cmd *set_brightness_cmds;
	int set_brightness_cmds_size;

	struct i2c_bl_cmd *get_brightness_cmds;
	int get_brightness_cmds_size;

	struct i2c_bl_cmd *deinit_cmds;
	int deinit_cmds_size;

	struct i2c_bl_cmd *dump_regs;
	int dump_regs_size;

	char *blmap;
	int blmap_size;
};

#endif
