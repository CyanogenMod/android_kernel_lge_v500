/*********************************************************************
 *
 *	lge_irtty-sir.h:	please refer to irtty-sir.h
 *
 ********************************************************************/

#ifndef LGE_IRTTYSIR_H
#define LGE_IRTTYSIR_H

#include <net/irda/irda.h>
#include <net/irda/irda_device.h>		// chipio_t
#include <linux/gpio.h>

#include "../../../../arch/arm/mach-msm/lge/L05E/board-L05E.h"

#define IRTTY_IOC_MAGIC 'e'
#define IRTTY_IOCTDONGLE  _IO(IRTTY_IOC_MAGIC, 1)
#define IRTTY_IOCGET     _IOR(IRTTY_IOC_MAGIC, 2, struct irtty_info)
#define IRTTY_IOC_MAXNR   2

enum{
  GPIO_LOW_VALUE = 0,
  GPIO_HIGH_VALUE,
};

#define GPIO_IRDA_SW_EN_ENABLE	GPIO_HIGH_VALUE
#define GPIO_IRDA_SW_EN_DISABLE	GPIO_LOW_VALUE

#define GPIO_IRDA_PWDN_ENABLE		GPIO_LOW_VALUE
#define GPIO_IRDA_PWDN_DISABLE	GPIO_HIGH_VALUE

#define GPIO_IRDA_PWDN       	    PM8921_GPIO_PM_TO_SYS(20)
#define GPIO_IRDA_SW_EN       	    PM8921_GPIO_PM_TO_SYS(25)

#if defined(CONFIG_LGE_IRDA_DCM)
struct sirtty_cb {
	magic_t magic;

	struct sir_dev *dev;
	struct tty_struct  *tty;

	chipio_t io;               /* IrDA controller information */
};
#endif
#endif
