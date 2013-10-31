/*
 * drivers/input/touchscreen/Synaptics_so340010_keytouch.c - Touch keypad driver
 *
 * Copyright (C) 2013 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <mach/vreg.h>
#include <mach/board_lge.h>
#include <linux/random.h>
#include <linux/jiffies.h>
#include <linux/kfifo.h>
#include <linux/input/lge_touch_core.h>
#include <linux/input/synaptics_so340010_keytouch.h>

struct TSKEY {
	unsigned int keycode;
	unsigned char sensitivity;
};

struct so340010_device {
	struct i2c_client	*client;					/* i2c client for adapter */
	struct input_dev		*input_dev;				/* input device for android */

	struct work_struct	work_init;				/* work for init and ESD reset */
	struct work_struct	work_getkey;			/* work for get touch key data*/
	struct delayed_work	work_report;			/* work for send event to framework */
	struct delayed_work	work_lock;				/* Key lock during TSP active */

	struct kfifo			key_fifo;				/* FIFO for key data */
	spinlock_t			key_fifo_lock;			/* lock for FIFO */

	struct TSKEY		key_home;				/* Home key */
	struct TSKEY		key_menu;				/* Menu key */
	struct TSKEY		key_back;				/* Back key */

	bool					touchkey_lock;			/* lock for TSP */
	bool					touchkey_btn_press;	/*whether key pressed or not */

	#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	earlysuspend;
	#endif

	int (*tk_power)(int on, bool log_on);			/* power on/off function pointer */

	int					irq;						/* IRQ number */
	int					gpio_attn;				/* ATTN GPIO number */
	bool					initialized;				/* Whether init or not */
	int					init_retry_cnt;			/* Re-init count in error case */
};

static struct workqueue_struct *so340010_wq = NULL;
static struct so340010_device *pDev = NULL;

#define ATTN_HIGH					1
#define ATTN_LOW					0

#define KEYCODE_SIZE				sizeof(unsigned short)
#define KFIFO_SIZE					(8 * KEYCODE_SIZE)
#define WQ_DELAY(x)				msecs_to_jiffies(x)
#define ERROR_RETRY_COUNT		10

#define TSKEY_REG_DUMMY			0x00
#define TSKEY_REG_INTERFACE		0x00
#define TSKEY_REG_GENERAL		0x01
#define TSKEY_REG_BTN_ENABLE	0x04
#define TSKEY_REG_SEN1			0x10
#define TSKEY_REG_SEN2			0x11
#define TSKEY_REG_BTN_STATE1	0x01
#define TSKEY_REG_BTN_STATE2	0x09
#define TSKEY_VAL_INTERFACE		0x07
#define TSKEY_VAL_GENERAL		0x30
#define TSKEY_VAL_BTN_ENABLE	0x0E
#define TSKEY_VAL_S1_HOME		0x200
#define TSKEY_VAL_S2_MENU		0x400
#define TSKEY_VAL_S3_BACK		0x800
#define TSKEY_VAL_RELEASE		0x000
#define TSKEY_VAL_ALL				(TSKEY_VAL_S1_HOME | TSKEY_VAL_S2_MENU | TSKEY_VAL_S3_BACK)
#define TSKEY_VAL_CANCEL			0xFF
#define TSKEY_BTN_PRESS			1
#define TSKEY_BTN_RELEASE		0
#define TSKEY_POWER_ON			1
#define TSKEY_POWER_OFF			0

struct reg_key_sensitivity_table {
	unsigned char home;
	unsigned char menu;
	unsigned char back;
};

struct reg_code_table {
	u8 val1;
	u8 val2;
	u8 val3;
	u8 val4;
};

static struct reg_code_table initial_code_table[] = {
	{TSKEY_REG_DUMMY,	TSKEY_REG_INTERFACE,		TSKEY_REG_DUMMY,	TSKEY_VAL_INTERFACE},
	{TSKEY_REG_DUMMY,	TSKEY_REG_GENERAL,		TSKEY_REG_DUMMY,	TSKEY_VAL_GENERAL},
	{TSKEY_REG_DUMMY,	TSKEY_REG_BTN_ENABLE,	TSKEY_REG_DUMMY,	TSKEY_VAL_BTN_ENABLE},
	{TSKEY_REG_DUMMY,	TSKEY_REG_SEN1,			TSKEY_REG_DUMMY,	TSKEY_REG_DUMMY},
	{TSKEY_REG_DUMMY,	TSKEY_REG_SEN2,			TSKEY_REG_DUMMY,	TSKEY_REG_DUMMY},
};

static int so340010_i2c_write(u8 reg_h, u8 reg_l, u8 val_h, u8 val_l)
{
	u8 buf[4] = {0};
	struct i2c_msg msg = {
		pDev->client->addr, 0, sizeof(buf), buf
	};

	buf[0] = reg_h;
	buf[1] = reg_l;
	buf[2] = val_h;
	buf[3] = val_l;

	return i2c_transfer(pDev->client->adapter, &msg, 1);
}

static int so340010_i2c_read(u8 reg_h, u8 reg_l, u16 *ret)
{
	u8 buf[2] = {0};
	struct i2c_msg msg[2] = {
		{ pDev->client->addr, 0, sizeof(buf), buf },
		{ pDev->client->addr, I2C_M_RD, sizeof(buf), (__u8 *)ret }
	};

	buf[0] = reg_h;
	buf[1] = reg_l;

	return i2c_transfer(pDev->client->adapter, msg, 2);
}

static void so340010_i2c_power_onoff(bool onoff)
{
	pDev->initialized = false;
	pDev->init_retry_cnt = 0;
	
	if (likely(pDev->tk_power))
		pDev->tk_power(onoff, true);
}

void so340010_keytouch_cancel(void)
{
	u16 btn_state = TSKEY_VAL_CANCEL;
	
	pDev->touchkey_lock = true;

	if (pDev->touchkey_btn_press) {
		spin_lock(&pDev->key_fifo_lock);

		cancel_delayed_work_sync(&pDev->work_lock);
		cancel_delayed_work_sync(&pDev->work_report);

		kfifo_reset(&pDev->key_fifo);
		kfifo_in(&pDev->key_fifo, &btn_state, KEYCODE_SIZE);

		spin_unlock(&pDev->key_fifo_lock);
		
		schedule_delayed_work(&pDev->work_report, WQ_DELAY(0));
	}

}

void so340010_keytouch_lock_free(void)
{
	if (pDev->touchkey_lock)
		schedule_delayed_work(&pDev->work_lock, WQ_DELAY(200));
}

static int so340010_i2c_initialize(void)
{
	struct reg_key_sensitivity_table key_sensitivity[] = {
						/*Home		Menu		Back */
	/* HW_REV_A */	{ 0xFF,		0xFF,		0xFF },
	/* HW_REV_B */	{ 0xC8,		0xC8,		0xB4 },
	/* HW_REV_C */	{ 0xE3,		0xE3,		0xE3 },
	};

	int ret = 0;
	u16 data = 0;
	int i = 0;
	int index = 0;

	if (pDev->init_retry_cnt >= ERROR_RETRY_COUNT)
		return -1;

	if (pDev->key_home.sensitivity == 0 && pDev->key_menu.sensitivity == 0 && pDev->key_back.sensitivity == 0) {
		switch(lge_get_board_revno()) {
			case HW_REV_A :
				index = 0;
				break;
			case HW_REV_B :
				index = 1;
				break;
			case HW_REV_C :
			default :
				index = 2;
				break;
		}

		pDev->key_home.sensitivity = key_sensitivity[index].home;
		pDev->key_menu.sensitivity = key_sensitivity[index].menu;
		pDev->key_back.sensitivity = key_sensitivity[index].back;
	}

	initial_code_table[3].val3 = pDev->key_home.sensitivity;
	initial_code_table[4].val3 = pDev->key_menu.sensitivity;
	initial_code_table[4].val4 = pDev->key_back.sensitivity;

	for (i = 0; i < ARRAY_SIZE(initial_code_table); i++) {
		ret = so340010_i2c_write(
				initial_code_table[i].val1, initial_code_table[i].val2,
				initial_code_table[i].val3, initial_code_table[i].val4);
		if (unlikely(ret < 0)) {
			TOUCH_ERR_MSG("write initial_code_table failed (I2C NAK) \n");
			goto Exit;
		}
	}

	/*DUMMY read*/
	ret = so340010_i2c_read(TSKEY_REG_BTN_STATE1, TSKEY_REG_BTN_STATE2, &data);
	if (unlikely(ret < 0))
		goto Exit;

	pDev->initialized = true;
	pDev->init_retry_cnt = 0;
	
	TOUCH_INFO_MSG("%s: initialized \n", __func__);

	return 0;

Exit :
	pDev->initialized = false;
	pDev->init_retry_cnt++;

	TOUCH_INFO_MSG("%s: Failed \n", __func__);
	
	return -1;
}

static int so340010_i2c_uninit(void)
{
	if (likely(pDev->irq))
		disable_irq_nosync(pDev->irq);

	so340010_i2c_power_onoff(TSKEY_POWER_OFF);

	return 0;
}

static void so340010_i2c_enqueue_keycode(u16 btn_state)
{
	unsigned long delay = 0;

	if (pDev->touchkey_lock == false) {
		kfifo_in_locked(&pDev->key_fifo, &btn_state, KEYCODE_SIZE, &pDev->key_fifo_lock);

		if (btn_state == TSKEY_VAL_RELEASE)
			delay = WQ_DELAY(30);

		schedule_delayed_work(&pDev->work_report, delay);
	} else {
		TOUCH_INFO_MSG("KEY is Locked \n");
	}

	if (likely(pDev->irq))
		enable_irq(pDev->irq);
}

static void so340010_i2c_read_keycode(void)
{
	int retry_cnt = 0;
	int ret = 0;
	u16 btn_state = 0;

RETRY :
	// Read IC first in order to check proper IRQ.
	ret = so340010_i2c_read(TSKEY_REG_BTN_STATE1, TSKEY_REG_BTN_STATE2, &btn_state);
	if (ret >= 0) {
		so340010_i2c_enqueue_keycode(btn_state);
		return;
	}

	//  I2C error means something wrong. Check again.
	if (likely(pDev->gpio_attn)) {
		if (gpio_get_value(pDev->gpio_attn) == ATTN_LOW) {
			TOUCH_INFO_MSG("ATTN is Low. Check Reset %d/%d \n", ++retry_cnt, ERROR_RETRY_COUNT);
			mdelay(10);
			if (retry_cnt >= ERROR_RETRY_COUNT) {
				TOUCH_ERR_MSG("Start Re-Initialize");
				pDev->initialized = false;
				queue_work(so340010_wq, &pDev->work_init);
				return;
			} else {
				goto RETRY;
			}
		}
	}

	if (likely(pDev->irq))
		enable_irq(pDev->irq);
}

static void so340010_i2c_work_init(struct work_struct *work)
{
	int retry_cnt = 0;
	int ret = 0;

	if (unlikely(pDev->initialized == false)) {
		if (likely(pDev->gpio_attn)) {
ATTN_CHECK :
			if (gpio_get_value(pDev->gpio_attn) == ATTN_HIGH) { // ATTN still high
				mdelay(10);
				if (retry_cnt++ < ERROR_RETRY_COUNT) {
					TOUCH_INFO_MSG("ATTN is High. Waiting... : %d/%d \n", retry_cnt, ERROR_RETRY_COUNT);
					goto ATTN_CHECK;
				}
			}
		}
		
		ret = so340010_i2c_initialize();
		if (ret < 0) {
			if (retry_cnt++ < ERROR_RETRY_COUNT)
				goto ATTN_CHECK;
			else {
				TOUCH_ERR_MSG("IC failed. Power down.\n");
				so340010_i2c_uninit();
				return;
			}
		}
	}

	if (likely(pDev->irq))
		enable_irq(pDev->irq);
}

static void so340010_i2c_work_getkey(struct work_struct *work)
{
	if (unlikely(pDev->initialized == false)) {
		queue_work(so340010_wq, &pDev->work_init);
		return;
	}

	so340010_i2c_read_keycode();
}

static void so340010_i2c_work_report(struct work_struct *work)
{
	unsigned int fifo_num = 0;
	unsigned int fifo_cnt = 0;
	unsigned int keycode = 0;
	static unsigned int old_keycode = 0;
	unsigned short btn_state = 0;
	int readbyte = 0;
	int i;

	spin_lock(&pDev->key_fifo_lock);

	fifo_num = kfifo_len(&pDev->key_fifo);
	if (fifo_num == 0) {
		goto Exit;
	}

	fifo_cnt = fifo_num / KEYCODE_SIZE;

	for (i = 0; i < fifo_cnt; i++) {
		readbyte = kfifo_out(&pDev->key_fifo, &btn_state, KEYCODE_SIZE);
		if (unlikely(readbyte != KEYCODE_SIZE)) {
			TOUCH_ERR_MSG("Wrong FIFO size \n");
			goto Exit;
		}

		if (btn_state == TSKEY_VAL_CANCEL) {
			input_report_key(pDev->input_dev, old_keycode,		TSKEY_VAL_CANCEL);
			TOUCH_INFO_MSG("KEY[%2X] is canceled\n", old_keycode);
			input_sync(pDev->input_dev);
			pDev->touchkey_btn_press = false;
			goto Exit;
		}
		
		switch (btn_state) {
			case TSKEY_VAL_S1_HOME :
				keycode = pDev->key_home.keycode;
				break;
			case TSKEY_VAL_S2_MENU :
				keycode = pDev->key_menu.keycode;
				break;
			case TSKEY_VAL_S3_BACK :
				keycode = pDev->key_back.keycode;
				break;
		}

		switch (btn_state) {
			case TSKEY_VAL_S1_HOME :
			case TSKEY_VAL_S2_MENU :
			case TSKEY_VAL_S3_BACK :
				input_report_key(pDev->input_dev, keycode,	TSKEY_BTN_PRESS);
				input_report_key(pDev->input_dev, BTN_TOUCH,	TSKEY_BTN_PRESS);
				old_keycode = keycode;
				TOUCH_INFO_MSG("KEY[%2X] is pressed\n", keycode);
				pDev->touchkey_btn_press = true;
				break;
			case TSKEY_VAL_RELEASE :
				if (pDev->touchkey_btn_press) {
					input_report_key(pDev->input_dev, pDev->key_home.keycode,	TSKEY_BTN_RELEASE);
					input_report_key(pDev->input_dev, pDev->key_back.keycode,		TSKEY_BTN_RELEASE);
					input_report_key(pDev->input_dev, pDev->key_menu.keycode,	TSKEY_BTN_RELEASE);
					input_report_key(pDev->input_dev, BTN_TOUCH,						TSKEY_BTN_RELEASE);
					TOUCH_INFO_MSG("KEY[%2X] is released\n", old_keycode);
					pDev->touchkey_btn_press = false;
				}
				break;
		}

		input_sync(pDev->input_dev);
		
	}

Exit :
	spin_unlock(&pDev->key_fifo_lock);

}

static void so340010_i2c_work_lock(struct work_struct *work)
{	
	pDev->touchkey_lock = false;
}

static irqreturn_t so340010_i2c_irq_handler(int irq, void *handle)
{
	if (likely(pDev->irq))
		disable_irq_nosync(pDev->irq);

	queue_work(so340010_wq, &pDev->work_getkey);

	return IRQ_HANDLED;
}

static int so340010_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	so340010_i2c_uninit();

	cancel_work_sync(&pDev->work_init);
	cancel_work_sync(&pDev->work_getkey);
	cancel_delayed_work_sync(&pDev->work_lock);
	cancel_delayed_work_sync(&pDev->work_report);

	pDev->touchkey_lock = false;
	pDev->touchkey_btn_press = false;
	kfifo_reset(&pDev->key_fifo);

	return 0;
}

static int so340010_i2c_resume(struct i2c_client *client)
{
	so340010_i2c_power_onoff(TSKEY_POWER_ON);

	if (likely(pDev->irq))
		enable_irq(pDev->irq);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void so340010_early_suspend(struct early_suspend *h)
{
	so340010_i2c_suspend(pDev->client, PMSG_SUSPEND);
}

static void so340010_late_resume(struct early_suspend *h)
{
	so340010_i2c_resume(pDev->client);
}
#endif

static ssize_t so340010_set_sen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "HOME[%4d] MENU[%4d] BACK[%4d]\n",
			pDev->key_home.sensitivity, pDev->key_menu.sensitivity, pDev->key_back.sensitivity);
}

static ssize_t so340010_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	u16 a=0, b=0, c=0, d=0;

	ret = so340010_i2c_read(TSKEY_REG_DUMMY, TSKEY_REG_INTERFACE, &a);
	if (ret < 0)
		printk("[TK] i2c read fail a\n");
	
	ret = so340010_i2c_read(TSKEY_REG_DUMMY, TSKEY_REG_GENERAL, &b);
	if (ret < 0)
		printk("[TK] i2c read fail b\n");

	ret = so340010_i2c_read(TSKEY_REG_DUMMY, TSKEY_REG_BTN_ENABLE, &c);
	if (ret < 0)
		printk("[TK] i2c read fail c\n");

	ret = so340010_i2c_read(TSKEY_REG_BTN_STATE1, TSKEY_REG_BTN_STATE2, &d);
	if (ret < 0)
		printk("[TK] i2c read fail d\n");

	return sprintf(buf, "0x0000[%x] 0x0001[%x] 0x0004[%x] 0x0109[%x]\n", a, b, c, d);
}

static ssize_t so340010_set_sen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int cmd_no = 0, config_value = 0;
	unsigned long after = 0;
	int ret = 0;

	ret = strict_strtoul(buf, 10, &after);
	if (ret)
		return -EINVAL;

	TOUCH_INFO_MSG("[SO340010] %s\n", __func__);
	cmd_no = (int)(after / 1000);
	config_value = (int)(after % 1000);

	if (cmd_no == 0) {
		pDev->key_home.sensitivity = config_value;
		TOUCH_INFO_MSG("[SO340010] Set Home Key Sen[%d]\n", pDev->key_home.sensitivity);
		so340010_i2c_write(TSKEY_REG_DUMMY, TSKEY_REG_SEN1, pDev->key_home.sensitivity, 0);
	} else if (cmd_no == 1) {
		pDev->key_menu.sensitivity = config_value;
		TOUCH_INFO_MSG("[SO340010] Set Menu Key Sen[%d]\n", pDev->key_menu.sensitivity);
		so340010_i2c_write(TSKEY_REG_DUMMY, TSKEY_REG_SEN2, pDev->key_back.sensitivity, pDev->key_menu.sensitivity);
	} else if (cmd_no == 2) {
		pDev->key_back.sensitivity = config_value;
		TOUCH_INFO_MSG("[SO340010] Set Back Key Sen[%d]\n", pDev->key_back.sensitivity);
		so340010_i2c_write(TSKEY_REG_DUMMY, TSKEY_REG_SEN2, pDev->key_back.sensitivity, pDev->key_menu.sensitivity);
	} else {
		TOUCH_ERR_MSG("[%s] unknown CMD\n", __func__);
	}

	return size;
}

static int so340010_get_random(int min, int max)
{
	int rand_buf = 0;
	int value = 0;

	get_random_bytes((int*)&rand_buf, sizeof(int));
	if(min > max) {
		value = min;
		min = max;
		max = value;
	}

	max -= min;

	value = (rand_buf%max) + min;
	if(value < 0)
		value *= -1;
		
	return value;

}

static ssize_t so340010_reset_test(struct device *dev, struct device_attribute *attr, char *buf)
{
	int delay = so340010_get_random(1, 300);

	TOUCH_INFO_MSG("power will down during %d ms \n", delay);

	if (likely(pDev->tk_power))
		pDev->tk_power(TSKEY_POWER_OFF, true);
	
	mdelay(delay);

	if (likely(pDev->tk_power))
		pDev->tk_power(TSKEY_POWER_ON, true);

	return 4;
}

static ssize_t so340010_reset_test1(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0;
	int cnt = 0;

	cnt = so340010_get_random(3, 50);

	TOUCH_INFO_MSG("power will down and up %d times \n", cnt);

	for (i = 0; i < cnt; i++) {
		if (likely(pDev->tk_power))
			pDev->tk_power(TSKEY_POWER_OFF, true);
		
		mdelay(so340010_get_random(1, 100));

		if (likely(pDev->tk_power))
			pDev->tk_power(TSKEY_POWER_ON, true);

		mdelay(so340010_get_random(1, 100));
	}
	return 4;
}

static struct device_attribute dev_attr_device_test[] = {
	__ATTR(set_sen, S_IRUGO | S_IWUSR | S_IXOTH, so340010_set_sen_show, so340010_set_sen_store),
	__ATTR(test, S_IRUGO | S_IWUSR | S_IXOTH, so340010_test_show, NULL),
	__ATTR(reset, S_IRUGO | S_IWUSR | S_IXOTH, so340010_reset_test, NULL),
	__ATTR(reset1, S_IRUGO | S_IWUSR | S_IXOTH, so340010_reset_test1, NULL),
};

static int so340010_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct key_touch_platform_data *pData = NULL;
	unsigned char keycode = KEY_UNKNOWN;
	int i = 0;
	int ret = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	if (unlikely(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))) {
		TOUCH_ERR_MSG("it is not support I2C_FUNC_I2C. \n");
		return -ENODEV;
	}

	pDev = kzalloc(sizeof(struct so340010_device), GFP_KERNEL);
	if (unlikely(pDev == NULL)) {
		TOUCH_ERR_MSG("failed to allocation \n");
		return -ENOMEM;
	}

	pDev->initialized = false;
	pDev->client = client;
	pData = client->dev.platform_data;

	if (likely(pData->tk_power))
		pDev->tk_power = pData->tk_power;

	pDev->key_home.keycode = pData->keycode[2];
	pDev->key_back.keycode = pData->keycode[4];
	pDev->key_menu.keycode = pData->keycode[6];

	pDev->gpio_attn = pData->irq;
	pDev->irq = MSM_GPIO_TO_INT(pData->irq);
	i2c_set_clientdata(pDev->client, pDev);

	pDev->input_dev = input_allocate_device();
	if (unlikely(pDev->input_dev == NULL)) {
		TOUCH_ERR_MSG("failed to input allocation \n");
		goto Exit;
	}

	pDev->input_dev->name = "touch_keypad";
	pDev->input_dev->phys = "touch_keypad/i2c";
	pDev->input_dev->id.vendor = VENDOR_LGE;
	pDev->input_dev->evbit[0] = BIT_MASK(EV_KEY);
	pDev->input_dev->keycode = pData->keycode;
	pDev->input_dev->keycodesize = sizeof(unsigned short);
	pDev->input_dev->keycodemax = pData->keycodemax;

	for (i = 0; i < pData->keycodemax; i++) {
		keycode = pData->keycode[2 * i];
		set_bit(keycode, pDev->input_dev->keybit);
	}

	ret = input_register_device(pDev->input_dev);
	if (unlikely(ret < 0)) {
		TOUCH_ERR_MSG("failed to register input %s \n", pDev->input_dev->name);
		goto Exit;
	}

	INIT_WORK(&pDev->work_init, so340010_i2c_work_init);
	INIT_WORK(&pDev->work_getkey, so340010_i2c_work_getkey);
	INIT_DELAYED_WORK(&pDev->work_lock, so340010_i2c_work_lock);
	INIT_DELAYED_WORK(&pDev->work_report, so340010_i2c_work_report);

	spin_lock_init(&pDev->key_fifo_lock);
	if (kfifo_alloc(&pDev->key_fifo, KFIFO_SIZE, GFP_KERNEL)) {
		TOUCH_ERR_MSG("failed to kfifo allocation \n");
		goto Exit;
	}

	#ifdef CONFIG_HAS_EARLYSUSPEND
	pDev->earlysuspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;
	pDev->earlysuspend.suspend = so340010_early_suspend;
	pDev->earlysuspend.resume = so340010_late_resume;
	register_early_suspend(&pDev->earlysuspend);
	#endif

	if (likely(pDev->gpio_attn)) {
		ret = gpio_request(pDev->gpio_attn, "touch_key_attn");
		if (unlikely(ret < 0)) {
			TOUCH_ERR_MSG("gpio_attn_request %d failed\n", pDev->gpio_attn);
		} else {
			gpio_direction_input(pDev->gpio_attn);
		}
	}

	for (i = 0; i < ARRAY_SIZE(dev_attr_device_test); i++) {
		ret = device_create_file(&client->dev, &dev_attr_device_test[i]);
		if (ret)
			TOUCH_ERR_MSG("%s: device_create_file fail\n", __func__);
	}

	so340010_i2c_power_onoff(TSKEY_POWER_ON);

	if(likely(pDev->irq)) {
		if (unlikely(request_irq(pDev->irq, so340010_i2c_irq_handler, IRQF_TRIGGER_LOW, client->name, pDev)))
			TOUCH_ERR_MSG("request_irq(%d) failed \n", pDev->irq);
	}

Exit :
	return ret;
}

static int so340010_i2c_remove(struct i2c_client *client)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(dev_attr_device_test); i++)
		device_remove_file(&client->dev, &dev_attr_device_test[i]);
	
	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&pDev->earlysuspend);
	#endif

	if (likely(pDev->irq))
		free_irq(pDev->irq, pDev);

	if (likely(pDev->gpio_attn))
		gpio_free(pDev->gpio_attn);

	input_unregister_device(pDev->input_dev);
	input_free_device(pDev->input_dev);
	kfifo_free(&pDev->key_fifo);
	kfree(pDev);

	so340010_i2c_power_onoff(TSKEY_POWER_OFF);
	
	return 0;
}

static const struct i2c_device_id so340010_i2c_ids[] = {
		{SYNAPTICS_KEYTOUCH_NAME, 0 },
		{ },
};

static struct i2c_driver so340010_i2c_driver = {
	.probe		= so340010_i2c_probe,
	.remove		= so340010_i2c_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= so340010_i2c_suspend,
	.resume		= so340010_i2c_resume,
#endif
	.id_table	= so340010_i2c_ids,
	.driver = {
		.name	= SYNAPTICS_KEYTOUCH_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init so340010_init(void)
{
	int ret = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	so340010_wq = create_singlethread_workqueue("so340010_wq");
	if (unlikely(!so340010_wq)) {
		TOUCH_ERR_MSG("failed to create singlethread so340010_wq \n");
		return -ENOMEM;
	}
	
	ret = i2c_add_driver(&so340010_i2c_driver);
	if (unlikely(ret < 0)) {
		TOUCH_ERR_MSG("failed to i2c_add_driver \n");
		destroy_workqueue(so340010_wq);
		return ret;
	}

	return 0;
}

static void __exit so340010_exit(void)
{
	TOUCH_INFO_MSG("%s \n", __func__);
	
	i2c_del_driver(&so340010_i2c_driver);

	if (likely(so340010_wq))
		destroy_workqueue(so340010_wq);
	
	return;
}

module_init(so340010_init);
module_exit(so340010_exit);

MODULE_DESCRIPTION("Synaptics so340010 touch key driver");
MODULE_LICENSE("GPL");

