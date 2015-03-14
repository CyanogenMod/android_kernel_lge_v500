/*
UEI_IRRC_DRIVER_FOR_MSM9860
*/

#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/sysdev.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/gpio.h>
#include <mach/gpiomux.h>
//#include <linux/regulator/gpio-regulator.h>
#include <linux/regulator/consumer.h>
#include <mach/board_lge.h>
#include <linux/lge_uei_irrc.h>

static int uei_irrc_probe(struct platform_device *pdev)
{
	struct uei_irrc_pdata_type *pdata = pdev->dev.platform_data;

	if(!pdata)
		return -EINVAL;

	pr_info("%s\n", __FUNCTION__);
#if defined(CONFIG_MACH_APQ8064_GVKT)
    gpio_set_value_cansleep(VREG_3P0_IRDA_EN,1);
#endif
	gpio_set_value(pdata->en_gpio,1);

	return 0;
}

static int uei_irrc_remove(struct platform_device *pdev)
{

	struct uei_irrc_pdata_type *pdata = platform_get_drvdata(pdev);

	pr_info("[UEI IrRC] remove (err:%d)\n", 104);

	pdata = NULL;

	return 0;
}

static int uei_irrc_suspend(struct platform_device *pdev, pm_message_t state)
{
#if defined(CONFIG_MACH_APQ8064_GVKT)
		struct uei_irrc_pdata_type *pdata = pdev->dev.platform_data;
		if(!pdata)
			   return -EINVAL;
#endif

	pr_info("%s\n", __FUNCTION__);

#if defined(CONFIG_MACH_APQ8064_GVKT)
		gpio_set_value_cansleep(VREG_3P0_IRDA_EN,0);
		gpio_set_value(pdata->en_gpio,0);
#endif
	return 0;
}

static int uei_irrc_resume(struct platform_device *pdev)
{
#if defined(CONFIG_MACH_APQ8064_GVKT)
		struct uei_irrc_pdata_type *pdata = pdev->dev.platform_data;
		if(!pdata)
			   return -EINVAL;
#endif

	pr_info("%s\n", __FUNCTION__);

#if defined(CONFIG_MACH_APQ8064_GVKT)
		gpio_set_value_cansleep(VREG_3P0_IRDA_EN,1);
		gpio_set_value(pdata->en_gpio,1);
#endif
	return 0;
}

static struct platform_driver uei_irrc_driver = {
	.probe			= uei_irrc_probe,
	.remove			= uei_irrc_remove,
	.suspend		= uei_irrc_suspend,
	.resume			= uei_irrc_resume,
	.driver			= {
		.name	= UEI_IRRC_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init uei_irrc_init(void)
{
	int ret;
	pr_info("%s\n", __FUNCTION__);

	ret = platform_driver_register(&uei_irrc_driver);

	if(ret)
		pr_info("%s : init fail\n" ,__FUNCTION__);

	return ret;
}

static void __exit uei_irrc_exit(void)
{
	pr_info("%s\n", __FUNCTION__);

	platform_driver_unregister(&uei_irrc_driver);
}

module_init(uei_irrc_init);
module_exit(uei_irrc_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("UEI IrRC Driver");
MODULE_LICENSE("GPL");
