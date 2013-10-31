/* include/linux/platform_data/hds_fsa8008.h
 *
 * Copyright (C) 2012 LG Electronics Inc.
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

#ifndef __HDS_FSA8008_H__
#define __HDS_FSA8008_H__

struct fsa8008_platform_data {
	const char *switch_name;  /* switch device name */
	const char *keypad_name;  /* keypad device name */

	unsigned int key_code;    /* key code for hook */

	unsigned int gpio_detect; /* DET : to detect jack inserted or not */
	unsigned int gpio_detect_can_wakeup;
	unsigned int gpio_mic_en; /* EN : to enable mic */
	unsigned int gpio_mic_bias_en; /* EN : to enable mic bias */
	unsigned int gpio_jpole;  /* JPOLE : 3pole or 4pole */
	unsigned int gpio_key;    /* S/E button */

	/* latency for pole (3 or 4)detection (in ms) */
	unsigned int latency_for_detection;

	/* callback function which is initialized while probing */
	void (*set_headset_mic_bias)(int enable);
	void (*set_uart_console)(int enable);

	//2013-04-17 Ilda_jung(ilda.jung@lge.com) [AWIFI/AUDIO BSP] Enable earjack power(LDO) [START]
	#if defined(CONFIG_MACH_APQ8064_AWIFI)
	unsigned int gpio_power_en;
	unsigned int gpio_hph_en;
	#endif
	//2013-04-17 Ilda_jung(ilda.jung@lge.com) [AWIFI/AUDIO BSP] Enable earjack power(LDO) [END]
};

#endif /* __HDS_FSA8008_H__ */
