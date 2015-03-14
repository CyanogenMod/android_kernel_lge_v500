/* include/linux/platform_data/hds_max1462x.h
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

#ifndef __HDS_MAX1462X_H__
#define __HDS_MAX1462X_H__

struct max1462x_platform_data {
    /* mandotary */
    const char *switch_name;    /* switch device name */
    const char *keypad_name;    /* keypad device name */

    unsigned int key_code;      /* key code for hook[KEY_MEDIA(226)], volume up[KEY_VOLUMEUP(115)], volume down[KEY_VOLUMEDOWN(114)] */

    unsigned int gpio_mode;     /* MODE : high, low, high-z */
    unsigned int gpio_det;      /* DET : to detect jack inserted or not */
    unsigned int gpio_swd;      /* SWD : to detect 3 pole or 4 pole | to detect among hook, volum up or down key */

    unsigned int latency_for_detection; /* latency for DETIN Debounce Time (in ms) */
    unsigned int latency_for_key;       /* latency for SEND/END Debounce Time (in ms) */

    /* optional */
    unsigned int adc_mpp_num;   /* PMIC adc mpp number to read adc level on MIC */
    unsigned int adc_channel;   /* PMIC adc channel to read adc level on MIC */

    unsigned int external_ldo_mic_bias;         /* GPIO for an external LDO control */
    void (*set_headset_mic_bias)(int enable);   /* callback function for an external LDO control */
};

#endif /* __HDS_MAX1462X_H__ */
