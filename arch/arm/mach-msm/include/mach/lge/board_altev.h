/* arch/arm/mach-msm/include/mach/board_gku.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2012, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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

#ifndef __ASM_ARCH_MSM_BOARD_ALTEV_H
#define __ASM_ARCH_MSM_BOARD_ALTEV_H

#ifdef CONFIG_LGE_PM
#define ADC_CHANGE_REV	HW_REV_EVB1
#define IBAT_CURRENT		825

/* Ref resistance value = 665K */
#define ADC_NO_INIT_CABLE_MIN   0
#define ADC_CABLE_56K_MIN       50000
#define ADC_CABLE_130K_MIN      190000
#define ADC_CABLE_270K_MIN      475000		/* louis.kang@lgepartner.com 2013-09-09 check 270K registance for speaker dock */
#define ADC_CABLE_330K_MIN      550000
#define ADC_CABLE_910K_MIN      950000
#define ADC_CABLE_OPEN_MIN      1650000

#define ADC_NO_INIT_CABLE_MAX   0
#define ADC_CABLE_56K_MAX       185000
#define ADC_CABLE_130K_MAX      340000
#define ADC_CABLE_270K_MAX      550000		/* louis.kang@lgepartner.com 2013-09-09 check 270K registance for speaker dock */
#define ADC_CABLE_330K_MAX      725000
#define ADC_CABLE_910K_MAX      1100000
#define ADC_CABLE_OPEN_MAX      1800000

/* Ref resistance value = 665K */
#define ADC_NO_INIT_CABLE   0
#define ADC_CABLE_MHL_1K    30000
#define ADC_CABLE_U_28P7K   60000
#define ADC_CABLE_28P7K     100000
#define ADC_CABLE_56K       185000
#define ADC_CABLE_100K      265000
#define ADC_CABLE_130K      340000
#define ADC_CABLE_180K      400000
#define ADC_CABLE_200K      431000
#define ADC_CABLE_220K      485000
#define ADC_CABLE_270K      560000
#define ADC_CABLE_330K      735000
#define ADC_CABLE_620K      955000
#define ADC_CABLE_910K      1140000
#define ADC_CABLE_NONE      1800000



#define C_NO_INIT_TA_MA     0
#define C_MHL_1K_TA_MA      500
#define C_U_28P7K_TA_MA     500
#define C_28P7K_TA_MA       500
#define C_56K_TA_MA         1500 /* it will be changed in future */
#define C_100K_TA_MA        500
#define C_130K_TA_MA        1500
#define C_180K_TA_MA        700
#define C_200K_TA_MA        700
#define C_220K_TA_MA        900
#define C_270K_TA_MA        800
#define C_330K_TA_MA        500
#define C_620K_TA_MA        500
#define C_910K_TA_MA        1500//[ORG]500
#ifdef CONFIG_MACH_APQ8064_ALTEV
#define C_OPEN_TA_MA        1500 // Adaptive current from 500mA to 900mA
#else
#define C_OPEN_TA_MA        500 // Adaptive current from 500mA to 900mA
#endif
#define C_NONE_TA_MA        1500 //900mA for open cable

#define C_NO_INIT_USB_MA    0
#define C_MHL_1K_USB_MA     500
#define C_U_28P7K_USB_MA    500
#define C_28P7K_USB_MA      500
#define C_56K_USB_MA        1500 /* it will be changed in future */
#define C_100K_USB_MA       500
#define C_130K_USB_MA       1500
#define C_180K_USB_MA       500
#define C_200K_USB_MA       500
#define C_220K_USB_MA       500
#define C_270K_USB_MA       500
#define C_330K_USB_MA       500
#define C_620K_USB_MA       500
#define C_910K_USB_MA       1500//[ORG]500
#define C_OPEN_USB_MA       500
#define C_NONE_USB_MA       500
#endif
	
#endif // __ASM_ARCH_MSM_BOARD_ALTEV_H
