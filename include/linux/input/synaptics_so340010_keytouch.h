#ifndef __SO340010_KEYTOUCH_H
#define __SO340010_KEYTOUCH_H

#define SYNAPTICS_KEYTOUCH_NAME "so340010"
#define	SYNAPTICS_KEYTOUCH_I2C_SLAVE_ADDR		0x2c
#define	VENDOR_LGE			0x1004

#define	KEY_TOUCH_GPIO_I2C1_SCL					9
#define	KEY_TOUCH_GPIO_I2C1_SDA					8
#define	KEY_TOUCH_GPIO_I2C2_SCL					85
#define	KEY_TOUCH_GPIO_I2C2_SDA					84
#define	KEY_TOUCH_GPIO_INT						83

struct key_touch_platform_data {
	int (*tk_power)(int on, bool log_on);
	int irq;
	int scl;
	int sda;
    int ldo;
	unsigned char *keycode;
	int keycodemax;
	int gpio_int;
    bool tk_power_flag;
};
#endif