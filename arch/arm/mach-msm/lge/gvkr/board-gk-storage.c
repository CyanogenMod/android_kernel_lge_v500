/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
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

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/bootmem.h>
#include <asm/mach-types.h>
#include <asm/mach/mmc.h>
#include <mach/msm_bus_board.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <mach/socinfo.h>
#include "devices.h"
#include "board-gk.h"
#include "board-storage-common-a.h"

#include <mach/board_lge.h>
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/pm_qos.h>
#endif /* CONFIG_MMC_MSM_SDC4_SUPPORT */

/* APQ8064 has 4 SDCC controllers */
enum sdcc_controllers {
	SDCC1,
	SDCC2,
	SDCC3,
	SDCC4,
	MAX_SDCC_CONTROLLER
};

/* All SDCC controllers require VDD/VCC voltage */
static struct msm_mmc_reg_data mmc_vdd_reg_data[MAX_SDCC_CONTROLLER] = {
	/* SDCC1 : eMMC card connected */
	[SDCC1] = {
		.name = "sdc_vdd",
		.high_vol_level = 2950000,
		.low_vol_level = 2950000,
		.always_on = 1,
		.lpm_sup = 1,
		.lpm_uA = 9000,
		.hpm_uA = 200000, /* 200mA */
	},
	/* SDCC3 : External card slot connected */
	[SDCC3] = {
		.name = "sdc_vdd",
		.high_vol_level = 2950000,
		.low_vol_level = 2950000,
#ifdef CONFIG_LGE_SD_LIFETIME_STRENGTHEN
        .always_on = 1,
#endif
		.hpm_uA = 800000, /* 800mA */
	}
};

/* SDCC controllers may require voting for VDD IO voltage */
static struct msm_mmc_reg_data mmc_vdd_io_reg_data[MAX_SDCC_CONTROLLER] = {
	/* SDCC1 : eMMC card connected */
	[SDCC1] = {
		.name = "sdc_vdd_io",
		.always_on = 1,
		.high_vol_level = 1800000,
		.low_vol_level = 1800000,
		.hpm_uA = 200000, /* 200mA */
	},
	/* SDCC3 : External card slot connected */
	[SDCC3] = {
		.name = "sdc_vdd_io",
		.high_vol_level = 2950000,
		.low_vol_level = 1850000,
		.always_on = 1,
		.lpm_sup = 1,
		/* Max. Active current required is 16 mA */
		.hpm_uA = 16000,
		/*
		 * Sleep current required is ~300 uA. But min. vote can be
		 * in terms of mA (min. 1 mA). So let's vote for 2 mA
		 * during sleep.
		 */
		.lpm_uA = 2000,
	}
};

static struct msm_mmc_slot_reg_data mmc_slot_vreg_data[MAX_SDCC_CONTROLLER] = {
	/* SDCC1 : eMMC card connected */
	[SDCC1] = {
		.vdd_data = &mmc_vdd_reg_data[SDCC1],
		.vdd_io_data = &mmc_vdd_io_reg_data[SDCC1],
	},
	/* SDCC3 : External card slot connected */
	[SDCC3] = {
		.vdd_data = &mmc_vdd_reg_data[SDCC3],
		.vdd_io_data = &mmc_vdd_io_reg_data[SDCC3],
	}
};

/* SDC1 pad data */
static struct msm_mmc_pad_drv sdc1_pad_drv_on_cfg[] = {
	{TLMM_HDRV_SDC1_CLK, GPIO_CFG_16MA},
	{TLMM_HDRV_SDC1_CMD, GPIO_CFG_10MA},
	{TLMM_HDRV_SDC1_DATA, GPIO_CFG_10MA}
};

static struct msm_mmc_pad_drv sdc1_pad_drv_off_cfg[] = {
	{TLMM_HDRV_SDC1_CLK, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC1_CMD, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC1_DATA, GPIO_CFG_2MA}
};

static struct msm_mmc_pad_pull sdc1_pad_pull_on_cfg[] = {
	{TLMM_PULL_SDC1_CLK, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC1_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC1_DATA, GPIO_CFG_PULL_UP}
};

static struct msm_mmc_pad_pull sdc1_pad_pull_off_cfg[] = {
	{TLMM_PULL_SDC1_CLK, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC1_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC1_DATA, GPIO_CFG_PULL_UP}
};

/* SDC3 pad data */
static struct msm_mmc_pad_drv sdc3_pad_drv_on_cfg[] = {
#if defined(CONFIG_MACH_APQ8064_GKATT) || defined(CONFIG_MACH_APQ8064_GKGLOBAL)
	{TLMM_HDRV_SDC3_CLK, GPIO_CFG_12MA},
	{TLMM_HDRV_SDC3_CMD, GPIO_CFG_12MA},
	{TLMM_HDRV_SDC3_DATA, GPIO_CFG_12MA}
#else
	{TLMM_HDRV_SDC3_CLK, GPIO_CFG_12MA},
	{TLMM_HDRV_SDC3_CMD, GPIO_CFG_12MA},
	{TLMM_HDRV_SDC3_DATA, GPIO_CFG_16MA}
#endif
};

static struct msm_mmc_pad_drv sdc3_pad_drv_off_cfg[] = {
	{TLMM_HDRV_SDC3_CLK, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC3_CMD, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC3_DATA, GPIO_CFG_2MA}
};

static struct msm_mmc_pad_pull sdc3_pad_pull_on_cfg[] = {
	{TLMM_PULL_SDC3_CLK, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC3_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC3_DATA, GPIO_CFG_PULL_UP}
};

static struct msm_mmc_pad_pull sdc3_pad_pull_off_cfg[] = {
	{TLMM_PULL_SDC3_CLK, GPIO_CFG_NO_PULL},
#ifdef CONFIG_LGE_SD_LIFETIME_STRENGTHEN
	{TLMM_PULL_SDC3_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC3_DATA, GPIO_CFG_PULL_UP}
#else
	{TLMM_PULL_SDC3_CMD, GPIO_CFG_PULL_DOWN},
	{TLMM_PULL_SDC3_DATA, GPIO_CFG_PULL_DOWN}
#endif
};

static struct msm_mmc_pad_pull_data mmc_pad_pull_data[MAX_SDCC_CONTROLLER] = {
	[SDCC1] = {
		.on = sdc1_pad_pull_on_cfg,
		.off = sdc1_pad_pull_off_cfg,
		.size = ARRAY_SIZE(sdc1_pad_pull_on_cfg)
	},
	[SDCC3] = {
		.on = sdc3_pad_pull_on_cfg,
		.off = sdc3_pad_pull_off_cfg,
		.size = ARRAY_SIZE(sdc3_pad_pull_on_cfg)
	},
};

static struct msm_mmc_pad_drv_data mmc_pad_drv_data[MAX_SDCC_CONTROLLER] = {
	[SDCC1] = {
		.on = sdc1_pad_drv_on_cfg,
		.off = sdc1_pad_drv_off_cfg,
		.size = ARRAY_SIZE(sdc1_pad_drv_on_cfg)
	},
	[SDCC3] = {
		.on = sdc3_pad_drv_on_cfg,
		.off = sdc3_pad_drv_off_cfg,
		.size = ARRAY_SIZE(sdc3_pad_drv_on_cfg)
	},
};

static struct msm_mmc_pad_data mmc_pad_data[MAX_SDCC_CONTROLLER] = {
	[SDCC1] = {
		.pull = &mmc_pad_pull_data[SDCC1],
		.drv = &mmc_pad_drv_data[SDCC1]
	},
	[SDCC3] = {
		.pull = &mmc_pad_pull_data[SDCC3],
		.drv = &mmc_pad_drv_data[SDCC3]
	},
};

static struct msm_mmc_gpio sdc2_gpio[] = {
	{59, "sdc2_clk"},
	{57, "sdc2_cmd"},
	{62, "sdc2_dat_0"},
	{61, "sdc2_dat_1"},
	{60, "sdc2_dat_2"},
	{58, "sdc2_dat_3"},
};

static struct msm_mmc_gpio sdc4_gpio[] = {
	{68, "sdc4_clk"},
	{67, "sdc4_cmd"},
	{66, "sdc4_dat_0"},
	{65, "sdc4_dat_1"},
	{64, "sdc4_dat_2"},
	{63, "sdc4_dat_3"},
};

static struct msm_mmc_gpio_data mmc_gpio_data[MAX_SDCC_CONTROLLER] = {
	[SDCC2] = {
		.gpio = sdc2_gpio,
		.size = ARRAY_SIZE(sdc2_gpio),
	},
	[SDCC4] = {
		.gpio = sdc4_gpio,
		.size = ARRAY_SIZE(sdc4_gpio),
	}
};

static struct msm_mmc_pin_data mmc_slot_pin_data[MAX_SDCC_CONTROLLER] = {
	[SDCC1] = {
		.pad_data = &mmc_pad_data[SDCC1],
	},
	[SDCC2] = {
		.is_gpio = 1,
		.gpio_data = &mmc_gpio_data[SDCC2],
	},
	[SDCC3] = {
		.pad_data = &mmc_pad_data[SDCC3],
	},
	[SDCC4] = {
		.is_gpio = 1,
		.gpio_data = &mmc_gpio_data[SDCC4],
	},
};

#define MSM_MPM_PIN_SDC1_DAT1	17
#define MSM_MPM_PIN_SDC3_DAT1	21

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static unsigned int sdc1_sup_clk_rates[] = {
	400000, 24000000, 48000000, 96000000
};

static unsigned int sdc1_sup_clk_rates_all[] = {
	400000, 24000000, 48000000, 96000000, 192000000
};

static struct mmc_platform_data sdc1_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.sup_clk_table	= sdc1_sup_clk_rates,
	.sup_clk_cnt	= ARRAY_SIZE(sdc1_sup_clk_rates),
	.nonremovable	= 1,
	.pin_data	= &mmc_slot_pin_data[SDCC1],
	.vreg_data	= &mmc_slot_vreg_data[SDCC1],
	.uhs_caps	= MMC_CAP_1_8V_DDR | MMC_CAP_UHS_DDR50,
	.uhs_caps2	= MMC_CAP2_HS200_1_8V_SDR,
	.packed_write	= MMC_CAP2_PACKED_WR | MMC_CAP2_PACKED_WR_CONTROL,
	.mpm_sdiowakeup_int = MSM_MPM_PIN_SDC1_DAT1,
	.msm_bus_voting_data = &sps_to_ddr_bus_voting_data,
};
static struct mmc_platform_data *apq8064_sdc1_pdata = &sdc1_data;
#else
static struct mmc_platform_data *apq8064_sdc1_pdata;
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static unsigned int sdc2_sup_clk_rates[] = {
	400000, 24000000, 48000000
};

static struct mmc_platform_data sdc2_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.sup_clk_table	= sdc2_sup_clk_rates,
	.sup_clk_cnt	= ARRAY_SIZE(sdc2_sup_clk_rates),
	.pin_data	= &mmc_slot_pin_data[SDCC2],
	.sdiowakeup_irq = MSM_GPIO_TO_INT(61),
	.msm_bus_voting_data = &sps_to_ddr_bus_voting_data,
};
static struct mmc_platform_data *apq8064_sdc2_pdata = &sdc2_data;
#else
static struct mmc_platform_data *apq8064_sdc2_pdata;
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static unsigned int sdc3_sup_clk_rates[] = {
	400000, 24000000, 48000000, 96000000, 192000000
};

static struct mmc_platform_data sdc3_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.sup_clk_table	= sdc3_sup_clk_rates,
	.sup_clk_cnt	= ARRAY_SIZE(sdc3_sup_clk_rates),
	.pin_data	= &mmc_slot_pin_data[SDCC3],
	.vreg_data	= &mmc_slot_vreg_data[SDCC3],
	.wpswitch_gpio	= PM8921_GPIO_PM_TO_SYS(17),
	.is_wpswitch_active_low = true,
	.status_gpio	= 26,
	.status_irq	= MSM_GPIO_TO_INT(26),
	.irq_flags	= IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.is_status_gpio_active_low = 1,
	.xpc_cap	= 1,
	.uhs_caps	= (MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 |
			MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_DDR50 |
			MMC_CAP_UHS_SDR104 | MMC_CAP_MAX_CURRENT_800),
	.mpm_sdiowakeup_int = MSM_MPM_PIN_SDC3_DAT1,
	.msm_bus_voting_data = &sps_to_ddr_bus_voting_data,
};
static struct mmc_platform_data *apq8064_sdc3_pdata = &sdc3_data;
#else
static struct mmc_platform_data *apq8064_sdc3_pdata;
#endif


#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static unsigned int sdc4_sup_clk_rates[] = {
	//400000, 24000000
	400000, 24000000, 48000000
};

static unsigned int g_wifi_detect;
static void *sdc4_dev;
void (*sdc4_status_cb)(int card_present, void *dev);
#if defined(CONFIG_LGE_BCM433X_PATCH) && !defined(CONFIG_BCMDHD_MODULE)
extern void
msmsdcc_set_mmc_enable(int card_present, void *dev_id);
#endif

int sdc4_status_register(void (*cb)(int card_present, void *dev), void *dev)
{
	if(sdc4_status_cb) {
		return -EINVAL;
	}
	sdc4_status_cb = cb;
	sdc4_dev = dev;
	return 0;
}

unsigned int sdc4_status(struct device *dev)
{
	return g_wifi_detect;
}

static struct mmc_platform_data sdc4_data = {
	.ocr_mask       = MMC_VDD_165_195 | MMC_VDD_27_28 | MMC_VDD_28_29,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.sup_clk_table	= sdc4_sup_clk_rates,
	.sup_clk_cnt	= ARRAY_SIZE(sdc4_sup_clk_rates),
	.pin_data	= &mmc_slot_pin_data[SDCC4],
#ifndef CONFIG_BCMDHD_MODULE
	.nonremovable   =  1,
#endif
	//.sdiowakeup_irq = MSM_GPIO_TO_INT(65), // inband test
	//.msm_bus_voting_data = &sps_to_ddr_bus_voting_data,
	.status         = sdc4_status,
	.register_status_notify = sdc4_status_register,
};
static struct mmc_platform_data *apq8064_sdc4_pdata = &sdc4_data;

// For broadcom
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM

#define WLAN_STATIC_SCAN_BUF0		5
#define WLAN_STATIC_SCAN_BUF1		6
#define PREALLOC_WLAN_SEC_NUM		12
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER	24

#define WLAN_SECTION_SKBUFF_IDX         4
#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)
#define WLAN_SECTION_SIZE_4	0 /* Index 4 is static socket buffer */
#define WLAN_SECTION_SIZE_5	(65536)
#define WLAN_SECTION_SIZE_6	(65536)
#define WLAN_SECTION_SIZE_7	(16 * 1024)
#define WLAN_SECTION_SIZE_8	(64 * 1024) // 23032
#ifdef CONFIG_BCMDHD_SDIO
#define WLAN_SECTION_SIZE_9		0
#define WLAN_SECTION_SIZE_10		0
#else
#define WLAN_SECTION_SIZE_9		(18 * 1024) // 16338
#define WLAN_SECTION_SIZE_10		(32 * 1024)
#endif
#ifdef CONFIG_BCMDHD_SDIO
#define WLAN_SECTION_SIZE_11            (73760)       /* sizeof(WLFC_HANGER_SIZE(3072)) */
#else
#define WLAN_SECTION_SIZE_11            0
#endif

#define DHD_SKB_HDRSIZE			336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

#define WLAN_SKB_BUF_NUM	17

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wlan_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1) },
	{NULL, (WLAN_SECTION_SIZE_2) },
	{NULL, (WLAN_SECTION_SIZE_3) },
	{NULL, (WLAN_SECTION_SIZE_4) },
	{NULL, (WLAN_SECTION_SIZE_5) },
	{NULL, (WLAN_SECTION_SIZE_6) },
	{NULL, (WLAN_SECTION_SIZE_7) },
	{NULL, (WLAN_SECTION_SIZE_8) },
	{NULL, (WLAN_SECTION_SIZE_9) },
	{NULL, (WLAN_SECTION_SIZE_10) },
    {NULL, (WLAN_SECTION_SIZE_11) }
};

void *wlan_static_scan_buf0;
void *wlan_static_scan_buf1;
static void *brcm_wlan_mem_prealloc(int section, unsigned long size)
{
	if (section == WLAN_SECTION_SKBUFF_IDX)
		return wlan_static_skb;
	if (section == WLAN_STATIC_SCAN_BUF0)
		return wlan_static_scan_buf0;
	if (section == WLAN_STATIC_SCAN_BUF1)
		return wlan_static_scan_buf1;
	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;

	if (wlan_mem_array[section].size < size)
		return NULL;

	return wlan_mem_array[section].mem_ptr;
}

static int brcm_init_wlan_mem(void)
{
	int i;
	int j;

	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;

	for (i = 0 ; i < PREALLOC_WLAN_SEC_NUM ; i++) {
		wlan_mem_array[i].mem_ptr =
				kmalloc(wlan_mem_array[i].size, GFP_KERNEL);

		if (!wlan_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}
	wlan_static_scan_buf0 = kmalloc (65536, GFP_KERNEL);
	if(!wlan_static_scan_buf0)
		goto err_mem_alloc;
	wlan_static_scan_buf1 = kmalloc (65536, GFP_KERNEL);
	if(!wlan_static_scan_buf1)
		goto err_mem_alloc;

	printk("%s: WIFI MEM Allocated\n", __FUNCTION__);
	return 0;

 err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		kfree(wlan_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

 err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

#define WLAN_POWER    PM8921_GPIO_PM_TO_SYS(CONFIG_BCMDHD_GPIO_WL_RESET) // PMIC gpio 29
#define WLAN_HOSTWAKE CONFIG_BCMDHD_GPIO_WL_HOSTWAKEUP
static unsigned wlan_wakes_msm[] = {
	GPIO_CFG(WLAN_HOSTWAKE, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA) };

/* for wifi power supply */
/*
static unsigned wifi_config_power_on[] = {
	GPIO_CFG(WLAN_POWER, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA) };
*/

int bcm_wifi_set_power(int enable)
{
	int ret = 0;
	if (enable)
	{
#if defined(CONFIG_LGE_BCM433X_PATCH) && !defined(CONFIG_BCMDHD_MODULE)
		msmsdcc_set_mmc_enable(enable,sdc4_dev);
#endif
		ret = gpio_direction_output(WLAN_POWER, 1); 
		if (ret) 
		{
			printk(KERN_ERR "%s: WL_REG_ON  failed to pull up (%d)\n",
					__func__, ret);
			return -EIO;
		}

		// WLAN chip to reset
		mdelay(150); //for booting time save
		printk("J:%s: applied delay. 150ms\n",__func__);
		printk(KERN_ERR "%s: wifi power successed to pull up\n",__func__);

	}
	else{
		ret = gpio_direction_output(WLAN_POWER, 0); 
		if (ret) 
		{
			printk(KERN_ERR "%s:  WL_REG_ON  failed to pull down (%d)\n",
					__func__, ret);
			return -EIO;
		}

		// WLAN chip down 
		// mdelay(100);//for booring time save
#if defined(CONFIG_LGE_BCM433X_PATCH) && !defined(CONFIG_BCMDHD_MODULE)
		msmsdcc_set_mmc_enable(enable,sdc4_dev);
#endif
		printk(KERN_ERR "%s: wifi power successed to pull down\n",__func__);
	}

	return ret;
}

#define LGE_BCM_WIFI_DMA_QOS_CONTROL
#ifdef LGE_BCM_WIFI_DMA_QOS_CONTROL
static int wifi_dma_state; /* 0 : INATIVE, 1:INIT, 2:IDLE, 3:ACTIVE */
static struct pm_qos_request wifi_dma_qos;
static struct delayed_work req_dma_work;
static uint32_t packet_transfer_cnt;

static void bcm_wifi_req_dma_work(struct work_struct *work)
{
	switch (wifi_dma_state) {
		case 2: /* IDLE State */
			if (packet_transfer_cnt < 100) {
				/* IDLE -> INIT */
				wifi_dma_state = 1;
				/* printk(KERN_ERR "%s: schedule work : %d : (IDLE -> INIT)\n", __func__, packet_transfer_cnt); */
			} else {
				/* IDLE -> ACTIVE */
				wifi_dma_state = 3;
				pm_qos_update_request(&wifi_dma_qos, 7);
				schedule_delayed_work(&req_dma_work, msecs_to_jiffies(50));
				/* printk(KERN_ERR "%s: schedule work : %d : (IDLE -> ACTIVE)\n", __func__, packet_transfer_cnt); */
			}
			break;

		case 3: /* ACTIVE State */
			if (packet_transfer_cnt < 10) {
				/* ACTIVE -> IDLE */
				wifi_dma_state = 2;
				pm_qos_update_request(&wifi_dma_qos, PM_QOS_DEFAULT_VALUE);
				schedule_delayed_work(&req_dma_work, msecs_to_jiffies(1000));
				/* printk(KERN_ERR "%s: schedule work : %d : (ACTIVE -> IDLE)\n", __func__, packet_transfer_cnt); */
			} else {
				/* Keep ACTIVE */
				schedule_delayed_work(&req_dma_work, msecs_to_jiffies(50));
				/* printk(KERN_ERR "%s: schedule work : %d :  (ACTIVE -> ACTIVE)\n", __func__, packet_transfer_cnt); */
			}
			break;

		default:
			break;
	}

	packet_transfer_cnt = 0;
}

void bcm_wifi_req_dma_qos(int vote)
{
	if (vote)
		packet_transfer_cnt++;

	/* INIT -> IDLE */
	if (wifi_dma_state == 1 && vote) {
		wifi_dma_state = 2; /* IDLE */
		schedule_delayed_work(&req_dma_work, msecs_to_jiffies(1000));
		/* printk(KERN_ERR "%s: schedule work (INIT -> IDLE)\n", __func__); */
	}
}
#endif

int __init bcm_wifi_init_gpio_mem(void)
{
	int rc=0;
/*
	if (gpio_tlmm_config(wifi_config_power_on[0], GPIO_CFG_ENABLE))
		printk(KERN_ERR "%s: Failed to configure WLAN_POWER\n", __func__);
*/

#ifdef LGE_BCM_WIFI_DMA_QOS_CONTROL
	INIT_DELAYED_WORK(&req_dma_work, bcm_wifi_req_dma_work);
	pm_qos_add_request(&wifi_dma_qos, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	wifi_dma_state = 1; /* INIT */
	printk("%s: wifi_dma_qos is added\n", __func__);
#endif

	if (gpio_request(WLAN_POWER, "WL_REG_ON"))		
		printk("Failed to request gpio %d for WL_REG_ON\n", WLAN_POWER);	

	if (gpio_direction_output(WLAN_POWER, 0)) 
		printk(KERN_ERR "%s: WL_REG_ON  failed to pull down \n", __func__);
	//gpio_free(WLAN_POWER);

	if (gpio_request(WLAN_HOSTWAKE, "wlan_wakes_msm"))		
		printk("Failed to request gpio %d for wlan_wakes_msm\n", WLAN_HOSTWAKE);			

	rc = gpio_tlmm_config(wlan_wakes_msm[0], GPIO_CFG_ENABLE);	
	if (rc)		
		printk(KERN_ERR "%s: Failed to configure wlan_wakes_msm = %d\n",__func__, rc);

	//gpio_free(WLAN_HOSTWAKE); 

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	brcm_init_wlan_mem();
#endif

	printk("bcm_wifi_init_gpio_mem successfully \n");

	return 0;
}

static int bcm_wifi_reset(int on)
{
	return 0;
}

static int bcm_wifi_carddetect(int val)
{
	g_wifi_detect = val;
	if(sdc4_status_cb)
		sdc4_status_cb(val, sdc4_dev);
	else
		printk("%s:There is no callback for notify\n", __FUNCTION__);
	return 0;
}

static int bcm_wifi_get_mac_addr(unsigned char* buf)
{
	uint rand_mac;
	static unsigned char mymac[6] = {0,};
	const unsigned char nullmac[6] = {0,};
	pr_debug("%s: %p\n", __func__, buf);

	printk("[%s] Entering...in Board-l1-mmc.c\n", __func__  );

	if( buf == NULL ) return -EAGAIN;

	if( memcmp( mymac, nullmac, 6 ) != 0 )
	{
		/* Mac displayed from UI are never updated..
		   So, mac obtained on initial time is used */
		memcpy( buf, mymac, 6 );
		return 0;
	}

	srandom32((uint)jiffies);
	rand_mac = random32();
	buf[0] = 0x00;
	buf[1] = 0x90;
	buf[2] = 0x4c;
	buf[3] = (unsigned char)rand_mac;
	buf[4] = (unsigned char)(rand_mac >> 8);
	buf[5] = (unsigned char)(rand_mac >> 16);

	memcpy( mymac, buf, 6 );

	printk("[%s] Exiting. MyMac :  %x : %x : %x : %x : %x : %x \n",__func__ , buf[0], buf[1], buf[2], buf[3], buf[4], buf[5] );

	return 0;
}

#define COUNTRY_BUF_SZ	4
struct cntry_locales_custom {
	char iso_abbrev[COUNTRY_BUF_SZ];
	char custom_locale[COUNTRY_BUF_SZ];
	int custom_locale_rev;
};

/* Customized Locale table */
const struct cntry_locales_custom bcm_wifi_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement BCM4334 series */
           {"",       "GB",     0},
           {"AD",    "GB",     0},
           {"AE",    "BR",     0},
           {"AF",    "GB",     0},
           {"AG",    "BR",     0},
           {"AI",     "US",     100},
           {"AL",    "GB",     0},
           {"AM",   "KW",      1},
           {"AN",   "BR",     0},
           {"AO",   "GB",      0},
           {"AR",    "AR",     1},
           {"AS",    "US",     100},
           {"AT",    "GB",     0},
           {"AU",    "AR",    1},
           {"AW",   "BR",     0},
           {"AZ",    "GB",     0},
           {"BA",    "GB",     0},
           {"BB",    "CN",     0},
           {"BD",    "QA",    0},
           {"BE",    "GB",     0},
           {"BF",    "GB",    0},
           {"BG",    "GB",     0},
           {"BH",    "CN",     0},
           {"BI",     "GB",      0},
           {"BJ",     "GB",      0},
           {"BM",   "US",     100},
           {"BN",    "CN",     0},
           {"BO",    "NG",      0},
           {"BR",    "BR",     0},
           {"BS",    "US",     100},
           {"BT",    "GB",      0},
           {"BW",   "GB",     0},
           {"BY",    "GB",     0},
           {"BZ",    "BR",      0},
           {"CA",    "US",     100},
           {"CD",    "GB",      0},
           {"CF",    "GB",      0},
           {"CG",    "GB",      0},
           {"CH",    "GB",     0},
           {"CI",     "GB",      0},
           {"CK",    "BR",     0},
           {"CL",    "CN",     0},
           {"CM",   "GB",      0},
           {"CN",    "CN",    0},
           {"CO",    "BR",     0},
           {"CR",    "BR",     0},
           {"CU",    "BR",     0},
           {"CV",    "GB",     0},
           {"CX",    "AR",    	1},
           {"CY",    "GB",     0},
           {"CZ",    "GB",     0},
           {"DE",    "GB",     0},
           {"DJ",    "IL",      10},
           {"DK",    "GB",     0},
           {"DM",   "BR",     0},
           {"DO",   "BR",     0},
           {"DZ",    "GB",    0},
           {"EC",    "BR",     0},
           {"EE",    "GB",     0},
           {"EG",    "CN",     0},
           {"ER",    "IL",      10},
           {"ES",    "GB",     0},
           {"ET",    "GB",     0},
           {"FI",     "GB",     0},
           {"FJ",     "AR",      1},
           {"FK",     "GB",      0},
           {"FM",   "US",     100},
           {"FO",    "GB",     0},
           {"FR",    "GB",     0},
           {"GA",    "GB",      0},
           {"GB",    "GB",     0},
           {"GD",    "BR",     0},
           {"GE",    "GB",     0},
           {"GF",    "GB",     0},
           {"GH",    "GB",     0},
           {"GI",     "GB",     0},
           {"GL",   "GB",      0},
           {"GM",   "GB",      0},
           {"GN",   "GB",      10},
           {"GP",    "GB",     0},
           {"GQ",   "GB",      0},
           {"GR",    "GB",     0},
           {"GT",    "BR",     0},
           {"GU",    "US",     100},
           {"GW",   "GB",      0},
           {"GY",    "QA",    0},
           {"HK",    "BR",     0},
           {"HN",   "QA",    0},
           {"HR",    "GB",     0},
           {"HT",    "QA",     0},
           {"HU",    "GB",     0},
           {"ID",     "ID",    1},
           {"IE",     "GB",     0},
           {"IL",     "KW",      1},
           {"IM",    "GB",     0},
           {"IN",    "CN",     0},
           {"IQ",    "GB",      0},
           {"IR",     "IL",      10},
           {"IS",     "GB",     0},
           {"IT",     "GB",     0},
           {"JE",     "GB",     0},
           {"JM",    "QA",     0},
           {"JO",    "JO",     0},
           {"JP",     "JP",      5},
           {"KE",    "SA",     0},
           {"KG",    "GB",      0},
           {"KH",    "BR",     0},
           {"KI",     "AR",    1},
           {"KM",   "GB",      0},
           {"KP",    "IL",      10},
           {"KR",    "KR",     24},
           {"KW",   "KW",    1},
           {"KY",    "US",     100},
           {"KZ",    "GB",     0},
           {"LA",    "BR",     0},
           {"LB",    "BR",     0},
           {"LC",    "BR",     0},
           {"LI",     "GB",     0},
           {"LK",    "BR",     0},
           {"LR",    "BR",     0},
           {"LS",     "GB",     0},
           {"LT",     "GB",     0},
           {"LU",    "GB",     0},
           {"LV",     "GB",     0},
           {"LY",     "GB",      0},
           {"MA",   "KW",    1},
           {"MC",   "GB",     0},
           {"MD",   "GB",     0},
           {"ME",   "GB",     0},
           {"MF",   "BR",     0},
           {"MG",   "GB",      0},
           {"MG",   "GB",      0},
           {"MK",   "GB",     0},
           {"ML",    "GB",      0},
           {"MM",  "GB",      0},
           {"MN",   "BR",      0},
           {"MO",   "BR",    0},
           {"MP",   "US",     100},
           {"MQ",   "GB",     0},
           {"MR",   "GB",     0},
           {"MS",   "GB",     0},
           {"MT",   "GB",     0},
           {"MU",   "GB",     0},
           {"MD",   "GB",     0},
           {"ME",   "GB",     0},
           {"MF",   "BR",     0},
           {"MG",   "GB",      0},
           {"MH",   "BR",     0},
           {"MK",   "GB",     0},
           {"ML",    "GB",      0},
           {"MM",  "GB",      0},
           {"MN",   "BR",      0},
           {"MO",   "BR",    0},
           {"MP",   "US",     100},
           {"MQ",   "GB",     0},
           {"MR",   "GB",     0},
           {"MS",   "GB",     0},
           {"MT",   "GB",     0},
           {"MU",   "GB",     0},
           {"MV",   "CN",     0},
           {"MW",  "BR",    0},
           {"MX",   "AR",     1},
           {"MY",   "CN",     0},
           {"MZ",   "BR",     0},
           {"NA",   "BR",     0},
           {"NC",    "GB",      0},
           {"NE",    "GB",     0},
           {"NF",    "BR",     0},
           {"NG",   "NG",    0},
           {"NI",    "BR",     0},
           {"NL",    "GB",     0},
           {"NO",   "GB",     0},
           {"NP",    "NP",     0},
           {"NR",    "GB",      0},
           {"NU",   "BR",     0},
           {"NZ",    "BR",     0},
           {"OM",   "GB",     0},
           {"PA",    "CN",     0},
           {"PE",    "BR",     0},
           {"PF",    "GB",     0},
           {"PG",    "AR",     1},
           {"PH",    "BR",     0},
           {"PK",    "QA",    0},
           {"PL",     "GB",     0},
           {"PM",   "GB",     0},
           {"PN",    "GB",     0},
           {"PR",    "US",     100},
           {"PS",    "BR",     0},
           {"PT",    "GB",     0},
           {"PW",   "BR",     0},
           {"PY",    "BR",     0},
           {"QA",   "QA",    0},
           {"RE",    "GB",     0},
           {"RKS",   "GB",     0},
           {"RO",    "GB",     0},
           {"RS",    "GB",     0},
           {"RU",    "AR",     1},
           {"RW",   "GB",    0},
           {"SA",    "SA",     0},
           {"SB",    "IL",      10},
           {"SC",    "BR",      10},
           {"SD",    "GB",     0},
           {"SE",    "GB",     0},
           {"SG",    "BR",     0},
           {"SI",     "GB",     0},
           {"SK",    "GB",     0},
           {"SKN",  "BR",   0},
           {"SL",     "GB",      0},
           {"SM",   "GB",     0},
           {"SN",    "KW",     1},
           {"SO",    "IL",      10},
           {"SR",    "GB",      0},
           {"SS",    "GB",     0},
           {"ST",    "GB",      0},
           {"SV",    "BR",     0},
           {"SY",    "BR",     0},
           {"SZ",    "GB",      0},
           {"TC",    "GB",     0},
           {"TD",    "GB",      0},
           {"TF",    "GB",     0},
           {"TG",    "GB",      0},
           {"TH",    "BR",     0},
           {"TJ",     "GB",      0},
           {"TL",     "BR",     0},
           {"TM",   "GB",      0},
           {"TN",    "KW",    1},
           {"TO",    "IL",      10},
           {"TR",    "GB",     0},
           {"TT",    "BR",     0},
           {"TV",    "IL",      10},
           {"TW",   "TW",    2},
           {"TZ",    "GB",    0},
           {"UA",    "CN",     0},
           {"UG",    "SA",     0},
           {"UM",    "US",     100},
           {"US",    "US",     100},
           {"UY",    "CN",     0},
           {"UZ",    "KW",      1},
           {"VA",    "GB",     0},
           {"VC",    "BR",     0},
           {"VE",    "CN",     0},
           {"VG",    "BR",     0},
           {"VI",     "US",     100},
           {"VN",    "BR",     0},
           {"VU",    "BR",      0},
           {"WS",   "SA",     0},
           {"YE",    "BR",      0},
           {"YT",    "GB",     0},
           {"ZA",    "GB",     0},
           {"ZM",   "BR",     0},
           {"ZW",   "GB",     0},
};

static void *bcm_wifi_get_country_code(char *ccode)
{
	int size, i;
	static struct cntry_locales_custom country_code;

	size = ARRAY_SIZE(bcm_wifi_translate_custom_table);

	if ((size == 0) || (ccode == NULL))
		return NULL;

	for (i = 0; i < size; i++) {
		if (strcmp(ccode, bcm_wifi_translate_custom_table[i].iso_abbrev) == 0)
			return (void *)&bcm_wifi_translate_custom_table[i];
	}

	memset(&country_code, 0, sizeof(struct cntry_locales_custom));
	strlcpy(country_code.custom_locale, ccode, COUNTRY_BUF_SZ);

	return (void *)&country_code;
}

static struct wifi_platform_data bcm_wifi_control = {
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	.mem_prealloc	= brcm_wlan_mem_prealloc,
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */
	.set_power	= bcm_wifi_set_power,
	.set_reset      = bcm_wifi_reset,
	.set_carddetect = bcm_wifi_carddetect,
	.get_mac_addr   = bcm_wifi_get_mac_addr, // Get custom MAC address
	.get_country_code = bcm_wifi_get_country_code,
};

static struct resource wifi_resource[] = {
	[0] = {
		.name = "bcmdhd_wlan_irq",
		.start = MSM_GPIO_TO_INT(WLAN_HOSTWAKE),
		.end   = MSM_GPIO_TO_INT(WLAN_HOSTWAKE),
#ifdef CONFIG_BCMDHD_HW_OOB
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE, // for HW_OOB
#else
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_SHAREABLE, //joon for SW_OOB
#endif
		//.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE | IORESOURCE_IRQ_SHAREABLE, //joon for SW_OOB
	},
};

static struct platform_device bcm_wifi_device = {
	.name           = "bcmdhd_wlan",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(wifi_resource),
	.resource       = wifi_resource,
	.dev            = {
		.platform_data = &bcm_wifi_control,
	},
};
#else
static struct mmc_platform_data *apq8064_sdc4_pdata;
#endif

void __init apq8064_init_mmc(void)
{
	hw_rev_type lge_bd_rev  = HW_REV_EVB1;

	if (apq8064_sdc1_pdata) {
		/* 8064 v2 supports upto 200MHz clock on SDC1 slot */
		if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) >= 2) {
			apq8064_sdc1_pdata->sup_clk_table =
					sdc1_sup_clk_rates_all;
			apq8064_sdc1_pdata->sup_clk_cnt	=
					ARRAY_SIZE(sdc1_sup_clk_rates_all);
		}
		apq8064_add_sdcc(1, apq8064_sdc1_pdata);
	}

	msm_add_uio();

	if (apq8064_sdc2_pdata)
		apq8064_add_sdcc(2, apq8064_sdc2_pdata);

	if (apq8064_sdc3_pdata) {
		if (!machine_is_apq8064_cdp()) {
			apq8064_sdc3_pdata->wpswitch_gpio = 0;
                        //apq8064_sdc3_pdata->wpswitch_polarity = 0; kinam119.kim
		}
		if (machine_is_mpq8064_cdp() || machine_is_mpq8064_hrd() ||
			machine_is_mpq8064_dtv()) {
			int rc;
			struct pm_gpio sd_card_det_init_cfg = {
				.direction      = PM_GPIO_DIR_IN,
				.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
				.pull           = PM_GPIO_PULL_UP_30,
				.vin_sel        = PM_GPIO_VIN_S4,
				.out_strength   = PM_GPIO_STRENGTH_NO,
				.function       = PM_GPIO_FUNC_NORMAL,
			};

			apq8064_sdc3_pdata->status_gpio =
				PM8921_GPIO_PM_TO_SYS(31);
			apq8064_sdc3_pdata->status_irq =
				PM8921_GPIO_IRQ(PM8921_IRQ_BASE, 31);
			rc = pm8xxx_gpio_config(apq8064_sdc3_pdata->status_gpio,
					&sd_card_det_init_cfg);
			if (rc) {
				pr_info("%s: SD_CARD_DET GPIO%d config "
					"failed(%d)\n", __func__,
					apq8064_sdc3_pdata->status_gpio, rc);
				apq8064_sdc3_pdata->status_gpio = 0;
				apq8064_sdc3_pdata->status_irq = 0;
			}
		}
		if (machine_is_apq8064_cdp()) {
			int i;

			for (i = 0;
			     i < apq8064_sdc3_pdata->pin_data->pad_data->\
				 drv->size;
			     i++)
				apq8064_sdc3_pdata->pin_data->pad_data->\
					drv->on[i].val = GPIO_CFG_10MA;
		}
		lge_bd_rev = lge_get_board_revno();
		if(lge_bd_rev == HW_REV_F)
			apq8064_sdc3_pdata->hw_rev_sd_low =true;
		else
			apq8064_sdc3_pdata->hw_rev_sd_low =false;
		apq8064_add_sdcc(3, apq8064_sdc3_pdata);
	}

	if (apq8064_sdc4_pdata) {
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
		bcm_wifi_init_gpio_mem();
		platform_device_register(&bcm_wifi_device);
#endif /* CONFIG_MMC_MSM_SDC4_SUPPORT */
		apq8064_add_sdcc(4, apq8064_sdc4_pdata);
	}

}
