/*
 * Copyright (C) 2011-2012 LGE, Inc.
 *
 * Author: Fred Cho <fred.cho@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/cradle.h>
#include <linux/gpio.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#if defined(CONFIG_MACH_APQ8064_AWIFI) || defined(CONFIG_MACH_APQ8064_ALTEV)
#define POUCH_DETECT_DELAY 100
#endif
static int pre_set_flag;
struct pm8xxx_cradle {
	struct switch_dev sdev;
#if defined(CONFIG_MACH_APQ8064_AWIFI) || defined(CONFIG_MACH_APQ8064_ALTEV)
	struct delayed_work pouch_work;
#else
	struct work_struct work;
#endif
	struct device *dev;
	struct wake_lock wake_lock;
	const struct pm8xxx_cradle_platform_data *pdata;
	int carkit;
	int pouch;
	spinlock_t lock;
	int state;
};

static struct workqueue_struct *cradle_wq;
static struct pm8xxx_cradle *cradle;

#if (defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)) && defined(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4)
static int is_smart_cover_closed = 0; /* check status of smart cover to resize knock-on area */

int cradle_smart_cover_status(void)
{
	return is_smart_cover_closed;/* check status of smart cover to resize knock-on area */
}
#endif
static void boot_cradle_det_func(void)
{
	int state;

	if (cradle->pdata->pouch_detect_pin)
		cradle->pouch = !gpio_get_value(cradle->pdata->pouch_detect_pin);

	pr_info("%s : boot pouch === > %d \n", __func__ , cradle->pouch);

	if (cradle->pouch == 1)
		state = CRADLE_SMARTCOVER;
	else
		state = CRADLE_SMARTCOVER_NO_DEV;

	pr_info("%s : [Cradle] boot cradle value is %d\n", __func__ , state);
	cradle->state = state;
	switch_set_state(&cradle->sdev, cradle->state);
#if (defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)) && defined(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4)
	is_smart_cover_closed = cradle->pouch; /* check status of smart cover to resize knock-on area */
#endif
}

#if defined(CONFIG_MACH_APQ8064_AWIFI) || defined(CONFIG_MACH_APQ8064_ALTEV)
static void pm8xxx_cradle_work_func(struct work_struct *work)
{
	int state=0;
	unsigned long flags;
spin_lock_irqsave(&cradle->lock, flags);
	if (cradle->pdata->pouch_detect_pin)
		cradle->pouch = !gpio_get_value_cansleep(cradle->pdata->pouch_detect_pin);

	pr_info("%s : pouch === > %d \n", __func__ , cradle->pouch);

	if (cradle->pouch == 1)
		state = CRADLE_SMARTCOVER;
	else
		state = CRADLE_SMARTCOVER_NO_DEV;

    if (cradle->state != state) {
		cradle->state = state;
		spin_unlock_irqrestore(&cradle->lock, flags);
		wake_lock_timeout(&cradle->wake_lock, msecs_to_jiffies(3000));
		switch_set_state(&cradle->sdev, cradle->state);
		printk("%s : [Cradle] pouch value is %d\n", __func__ , state);
	}
	else {
		spin_unlock_irqrestore(&cradle->lock, flags);
		printk("%s : [Cradle] pouch value is %d (no change)\n", __func__ , state);
	}
}
#else
static void pm8xxx_cradle_work_func(struct work_struct *work)
{
	int state;
	unsigned long flags;

	if (cradle->pdata->pouch_detect_pin)
		cradle->pouch = !gpio_get_value_cansleep(cradle->pdata->pouch_detect_pin);

	pr_info("%s : pouch === > %d \n", __func__ , cradle->pouch);

	spin_lock_irqsave(&cradle->lock, flags);

	if (cradle->pouch == 1)
		state = CRADLE_SMARTCOVER;
	else
		state = CRADLE_SMARTCOVER_NO_DEV;

	pr_info("%s : [Cradle] cradle value is %d\n", __func__ , state);
	cradle->state = state;
	spin_unlock_irqrestore(&cradle->lock, flags);

	wake_lock_timeout(&cradle->wake_lock, msecs_to_jiffies(3000));
	switch_set_state(&cradle->sdev, cradle->state);
#if (defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)) && defined(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4)
	is_smart_cover_closed = cradle->pouch; /* check status of smart cover to resize knock-on area */
#endif
}
#endif

void cradle_set_deskdock(int state)
{
	unsigned long flags;

	if (&cradle->sdev) {
		spin_lock_irqsave(&cradle->lock, flags);
		cradle->state = state;
		spin_unlock_irqrestore(&cradle->lock, flags);
		switch_set_state(&cradle->sdev, cradle->state);
	} else {
		pre_set_flag = state;
	}
}

int cradle_get_deskdock(void)
{
	if (!cradle)
		return pre_set_flag;

	return cradle->state;
}

static irqreturn_t pm8xxx_pouch_irq_handler(int irq, void *handle)
{
	struct pm8xxx_cradle *cradle_handle = handle;
	printk("pouch irq!!!!\n");
#if defined(CONFIG_MACH_APQ8064_AWIFI) || defined(CONFIG_MACH_APQ8064_ALTEV)
	wake_lock_timeout(&cradle->wake_lock, msecs_to_jiffies(POUCH_DETECT_DELAY*5));
	queue_delayed_work(cradle_wq, &cradle_handle->pouch_work, msecs_to_jiffies(POUCH_DETECT_DELAY));
#else
	queue_work(cradle_wq, &cradle_handle->work);
#endif
	return IRQ_HANDLED;
}

static ssize_t
cradle_pouch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;
	struct pm8xxx_cradle *cradle = dev_get_drvdata(dev);
	len = snprintf(buf, PAGE_SIZE, "pouch : %d\n", cradle->pouch);

	return len;

}

static ssize_t
cradle_sensing_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;
	struct pm8xxx_cradle *cradle = dev_get_drvdata(dev);
	len = snprintf(buf, PAGE_SIZE, "sensing : %d\n", cradle->state);

	return len;

}

static struct device_attribute cradle_device_attrs[] = {
	__ATTR(pouch, S_IRUGO | S_IWUSR, cradle_pouch_show, NULL),
	__ATTR(sensing,  S_IRUGO | S_IWUSR, cradle_sensing_show, NULL),
};

static ssize_t cradle_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case 0:
		return sprintf(buf, "UNDOCKED\n");
	case 2:
		return sprintf(buf, "CARKIT\n");
	default:
		break;
	}

	return -EINVAL;
}

static int __devinit pm8xxx_cradle_probe(struct platform_device *pdev)
{
	int ret, i;

	unsigned int hall_gpio_irq;

	const struct pm8xxx_cradle_platform_data *pdata =
		pdev->dev.platform_data;
	if (!pdata)
		return -EINVAL;

	cradle = kzalloc(sizeof(*cradle), GFP_KERNEL);
	if (!cradle)
		return -ENOMEM;

	cradle->pdata    = pdata;

	cradle->sdev.name = "smartcover";
	cradle->sdev.print_name = cradle_print_name;
	cradle->pouch = cradle->carkit = 0;

	spin_lock_init(&cradle->lock);

	ret = switch_dev_register(&cradle->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	if (pre_set_flag) {
		cradle_set_deskdock(pre_set_flag);
		cradle->state = pre_set_flag;
	}
	wake_lock_init(&cradle->wake_lock, WAKE_LOCK_SUSPEND, "hall_ic_wakeups");

#if defined(CONFIG_MACH_APQ8064_AWIFI) || defined(CONFIG_MACH_APQ8064_ALTEV)
	INIT_DELAYED_WORK(&cradle->pouch_work, pm8xxx_cradle_work_func);
#else
	INIT_WORK(&cradle->work, pm8xxx_cradle_work_func);
#endif
	printk("%s : init cradle\n", __func__);


	/* initialize irq of gpio_hall */
	hall_gpio_irq = gpio_to_irq(cradle->pdata->pouch_detect_pin);
	pr_info("%s : hall_gpio_irq = [%d]\n", __func__,hall_gpio_irq);
	if (hall_gpio_irq < 0) {
		printk("Failed : GPIO TO IRQ \n");
		ret = hall_gpio_irq;
		goto err_hall_gpio_irq;
	}


	pr_info("%s : pdata->irq_flags = [%d]\n", __func__,(int)pdata->irq_flags);

	ret = request_irq(hall_gpio_irq, pm8xxx_pouch_irq_handler, pdata->irq_flags, HALL_IC_DEV_NAME, cradle);
	if (ret > 0) {
		pr_err("%s: Can't allocate irq %d, ret %d\n", __func__, hall_gpio_irq, ret);
		goto err_request_irq;
	}

	printk("%s : enable_irq_wake \n", __func__);


	if (enable_irq_wake(hall_gpio_irq) == 0)
		pr_info("%s :enable_irq_wake Enable\n", __func__);
	else
		pr_info("%s :enable_irq_wake failed\n", __func__);

	pr_info("%s :boot_cradle_det_func START\n", __func__);
	boot_cradle_det_func();

	for (i = 0; i < ARRAY_SIZE(cradle_device_attrs); i++) {
		ret = device_create_file(&pdev->dev, &cradle_device_attrs[i]);
		if (ret)
			goto err_device_create_file;
	}

	platform_set_drvdata(pdev, cradle);
	return 0;


err_device_create_file:

err_request_irq:

err_hall_gpio_irq:
	if (hall_gpio_irq)
		free_irq(hall_gpio_irq, 0);

	switch_dev_unregister(&cradle->sdev);

err_switch_dev_register:
	kfree(cradle);
	return ret;
}

static int __devexit pm8xxx_cradle_remove(struct platform_device *pdev)
{
	struct pm8xxx_cradle *cradle = platform_get_drvdata(pdev);
#if defined(CONFIG_MACH_APQ8064_AWIFI) || defined(CONFIG_MACH_APQ8064_ALTEV)
	cancel_delayed_work_sync(&cradle->pouch_work);
#else
	cancel_work_sync(&cradle->work);
#endif
	switch_dev_unregister(&cradle->sdev);
	platform_set_drvdata(pdev, NULL);
	kfree(cradle);

	return 0;
}

static int pm8xxx_cradle_suspend(struct device *dev)
{
	return 0;
}

static int pm8xxx_cradle_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops pm8xxx_cradle_pm_ops = {
	.suspend = pm8xxx_cradle_suspend,
	.resume = pm8xxx_cradle_resume,
};

static struct platform_driver pm8xxx_cradle_driver = {
	.probe		= pm8xxx_cradle_probe,
	.remove		= __devexit_p(pm8xxx_cradle_remove),
	.driver		= {
		.name		= HALL_IC_DEV_NAME,
		.owner		= THIS_MODULE,
#ifdef CONFIG_PM
		.pm		= &pm8xxx_cradle_pm_ops,
#endif
	},
};

static int __init pm8xxx_cradle_init(void)
{
	cradle_wq = create_singlethread_workqueue("cradle_wq");
	pr_err("%s: cradle init \n", __func__);
	if (!cradle_wq)
		return -ENOMEM;

	return platform_driver_register(&pm8xxx_cradle_driver);
}
module_init(pm8xxx_cradle_init);

static void __exit pm8xxx_cradle_exit(void)
{
	if (cradle_wq)
		destroy_workqueue(cradle_wq);
	platform_driver_unregister(&pm8xxx_cradle_driver);
}
module_exit(pm8xxx_cradle_exit);

MODULE_ALIAS("platform:" HALL_IC_DEV_NAME);
MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("hall ic driver");
MODULE_LICENSE("GPL");
