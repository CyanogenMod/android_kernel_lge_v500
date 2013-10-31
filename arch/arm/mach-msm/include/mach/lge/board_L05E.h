/* arch/arm/mach-msm/include/mach/board_mako.h
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

#ifndef __ASM_ARCH_MSM_BOARD_L05E_H
#define __ASM_ARCH_MSM_BOARD_L05E_H

#ifdef CONFIG_LGE_PM
#define ADC_CHANGE_REV	HW_REV_EVB1
#define IBAT_CURRENT		825
#ifdef CONFIG_MACH_APQ8064_L05E
/* Ref resistance value = 200K */
#define ADC_NO_INIT_CABLE   0
#define ADC_CABLE_MHL_1K	 50000
#define ADC_CABLE_U_28P7K	 90000/* This value is obsolete */
#define ADC_CABLE_28P7K 	100000/* min value of 56K is so low because of factory cable issue */
#define ADC_CABLE_56K		490000
#define ADC_CABLE_100K		650000
#define ADC_CABLE_130K		800000
#define ADC_CABLE_180K		875000
#define ADC_CABLE_200K		920000
#define ADC_CABLE_220K		988000
#define ADC_CABLE_270K		1077000
#define ADC_CABLE_330K		1294000
#define ADC_CABLE_620K		1350000
#define ADC_CABLE_910K		1550000
#define ADC_CABLE_OPEN		1800000


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
/* Ref resistance value = 200K */
#define ADC_NO_INIT_CABLE_MIN   0
#define ADC_CABLE_56K_MIN       100000
#define ADC_CABLE_130K_MIN      500000
#define ADC_CABLE_910K_MIN      1350000
#define ADC_CABLE_OPEN_MIN      1650000

#define ADC_NO_INIT_CABLE_MAX   0
#define ADC_CABLE_56K_MAX       490000
#define ADC_CABLE_130K_MAX      800000
#define ADC_CABLE_910K_MAX      1550000
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

#endif // __ASM_ARCH_MSM_BOARD_MAKO_H
