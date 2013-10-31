/* arch/arm/mach-msm/include/mach/board_gvarcmcc.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2012, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2012, LGE.
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

#ifndef __ASM_ARCH_MSM_BOARD_GVARCMCC_H
#define __ASM_ARCH_MSM_BOARD_GVARCMCC_H

#ifdef CONFIG_LGE_PM
#define ADC_CHANGE_REV	HW_REV_EVB1
#define IBAT_CURRENT		825
#ifdef CONFIG_MACH_APQ8064_GVAR_CMCC
/* Ref resistance value = 665K */
#define ADC_NO_INIT_CABLE   0
#define ADC_CABLE_MHL_1K    5000
#define ADC_CABLE_U_28P7K   5500/* This value is obsolete */
#define ADC_CABLE_28P7K     6000/* min value of 56K is so low because of factory cable issue */
#define ADC_CABLE_56K       185000
#define ADC_CABLE_100K      240000
#define ADC_CABLE_130K      377000
#define ADC_CABLE_180K      400000
#define ADC_CABLE_200K		432000
#define ADC_CABLE_220K      483000
#define ADC_CABLE_270K      555000
#define ADC_CABLE_330K      672000
#define ADC_CABLE_620K      945000
#define ADC_CABLE_910K      1118725
#define ADC_CABLE_OPEN      1800000

#define C_NO_INIT_TA_MA     0
#define C_MHL_1K_TA_MA      500
#define C_U_28P7K_TA_MA     500
#define C_28P7K_TA_MA       500
#define C_56K_TA_MA         1500 /* it will be changed in future */
#define C_100K_TA_MA        500
#define C_130K_TA_MA        1500
#define C_180K_TA_MA        500
#define C_200K_TA_MA        500
#define C_220K_TA_MA        500
#define C_270K_TA_MA        500
#define C_330K_TA_MA        500
#define C_620K_TA_MA        500
#define C_910K_TA_MA        1500//[ORG]500
#define C_OPEN_TA_MA        500 // Adaptive current from 500mA to 900mA

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


#else
/* Ref resistance value = 665K */
#define ADC_NO_INIT_CABLE_MIN   0
#define ADC_CABLE_56K_MIN       81000
#define ADC_CABLE_130K_MIN      266000
#define ADC_CABLE_910K_MIN      956000
#define ADC_CABLE_OPEN_MIN      1150000

#define ADC_NO_INIT_CABLE_MAX   0
#define ADC_CABLE_56K_MAX       185000
#define ADC_CABLE_130K_MAX      340000
#define ADC_CABLE_910K_MAX      1140000
#define ADC_CABLE_OPEN_MAX      1800000

#define C_NO_INIT_TA_MA     0
#define C_56K_TA_MA         1500 /* it will be changed in future */
#define C_130K_TA_MA        1500
#define C_910K_TA_MA        1500//[ORG]500
#define C_OPEN_TA_MA        500 // Adaptive current from 500mA to 900mA

#define C_NO_INIT_USB_MA    0
#define C_56K_USB_MA        1500 /* it will be changed in future */
#define C_130K_USB_MA       1500
#define C_910K_USB_MA       1500//[ORG]500
#define C_OPEN_USB_MA       500
#endif
#endif
	
#endif // __ASM_ARCH_MSM_BOARD_GVARCMCC_H
