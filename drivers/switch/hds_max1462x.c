/*
 *  lge/com_device/input/max1462x.c
 *
 *  LGE 3.5 PI Headset detection driver using max1462x.
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * Copyright (C) 2009 ~ 2010 LGE, Inc.
 * Author: Lee SungYoung <lsy@lge.com>
 *
 * Copyright (C) 2010 LGE, Inc.
 * Author: Kim Eun Hye <ehgrace.kim@lge.com>
 *
 * Copyright (C) 2011 LGE, Inc.
 * Author: Yoon Gi Souk <gisouk.yoon@lge.com>
 *
 * Copyright (C) 2012 LGE, Inc.
 * Author: Park Gyu Hwa <gyuhwa.park@lge.com>
 *
 * Copyright (C) 2013 LGE, Inc.
 * Author: GooYeon Jung <gooyeon.jung@lge.com>
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

/* Interface is following;
 * source file is android/frameworks/base/services/java/com/android/server/HeadsetObserver.java
 * HEADSET_UEVENT_MATCH = "DEVPATH=/sys/devices/virtual/switch/h2w"
 * HEADSET_STATE_PATH = /sys/class/switch/h2w/state
 * HEADSET_NAME_PATH = /sys/class/switch/h2w/name
 */

#ifdef CONFIG_SWITCH_MAX1462X
#define CONFIG_MAX1462X_USE_LOCAL_WORK_QUEUE
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <linux/platform_data/hds_max1462x.h>
#include <linux/jiffies.h>
#include <linux/mfd/pm8xxx/pm8921.h>


#undef  LGE_HSD_DEBUG_PRINT /*TODO*/
#define LGE_HSD_DEBUG_PRINT /*TODO*/
#undef  LGE_HSD_ERROR_PRINT
#define LGE_HSD_ERROR_PRINT

#define HOOK_MIN    0
#define HOOK_MAX    150000
#define VUP_MIN     150000
#define VUP_MAX     400000
#define VDOWN_MIN   400000
#define VDOWN_MAX   600000

#if defined(LGE_HSD_DEBUG_PRINT)
#define HSD_DBG(fmt, args...) printk(KERN_INFO "HSD_max1462x[%-18s:%5d] " fmt, __func__, __LINE__, ## args)
#else
#define HSD_DBG(fmt, args...) do {} while (0)
#endif

#if defined(LGE_HSD_ERROR_PRINT)
#define HSD_ERR(fmt, args...) printk(KERN_ERR "HSD_max1462x[%-18s:%5d] " fmt, __func__, __LINE__, ## args)
#else
#define HSD_ERR(fmt, args...) do { } while (0)
#endif

#ifdef CONFIG_MAX1462X_USE_LOCAL_WORK_QUEUE
static struct workqueue_struct *local_max1462x_workqueue;
#endif

static struct wake_lock ear_hook_wake_lock;

#ifdef CONFIG_LGE_BROADCAST_ONESEG //for 325 BATMAN, L05E MMBI
static int ear_state = 0;
extern void isdbt_hw_antenna_switch(int ear_state);
#endif

struct ear_3button_info_table {
    unsigned int ADC_HEADSET_BUTTON;
    int PERMISS_REANGE_MAX;
    int PERMISS_REANGE_MIN;
    int PRESS_OR_NOT;
};

/* This table is only for J1 */
static struct ear_3button_info_table max1462x_ear_3button_type_data[]={
    {KEY_MEDIA, HOOK_MAX, HOOK_MIN, 0},
    {KEY_VOLUMEUP, VUP_MAX, VUP_MIN, 0},
    {KEY_VOLUMEDOWN, VDOWN_MAX, VDOWN_MIN, 0}
};

struct hsd_info {
    /* function devices provided by this driver */
    struct switch_dev sdev;
    struct input_dev *input;

    /* mutex */
    struct mutex mutex_lock;

    /* h/w configuration : initilized by platform data */
    unsigned int key_code;                      /* KEY_MEDIA, KEY_VOLUMEUP or KEY_VOLUMEDOWN */

    unsigned int gpio_mode;                     /* MODE : high, low, high-z */
    unsigned int gpio_swd;                      /* SWD : to detect 3pole or 4pole | to detect among hook, volum up or down key */
    unsigned int gpio_det;                      /* DET : to detect jack inserted or not */

    unsigned int latency_for_detection;         /* latency for DETIN Debounce Time (in jiffies) */
    unsigned int latency_for_key;               /* latency for SEND/END Debounce Time (in jiffies) */

    unsigned int adc_mpp_num;                   /* PMIC adc mpp number to read adc level on MIC */
    unsigned int adc_channel;                   /* PMIC adc channel to read adc level on MIC */

    unsigned int external_ldo_mic_bias;         /* GPIO for an external LDO control */
    void (*set_headset_mic_bias)(int enable);   /* callback function which is initialized while probing */

    /* internal variables */
    unsigned int irq_detect;                    /* irq number of gpio_det */
    unsigned int irq_key;                       /* irq number of gpio_swd */

    /* internal states */
    atomic_t irq_key_enabled;
    atomic_t is_3_pole_or_not;
    atomic_t btn_state;

    /* work for detect_work */
    struct delayed_work work;
    struct delayed_work work_for_key_pressed;
    struct delayed_work work_for_key_released;
};

/* status for sdev */
enum {
    HEADSET_NO_DEVICE   = 0,
    HEADSET_WITH_MIC    = (1 << 0),
    HEADSET_NO_MIC      = (1 << 1),
};

/* status for btn_state */
enum {
    HEADSET_BTN_INIT = -1,
    HEADSET_BTN_RELEASED = 0,
    HEADSET_BTN_PRESSED = 1,
};

/* status for is_3_pole_or_not */
enum {
    HEADSET_POLE_INIT = -1,
    HEADSET_POLE_3 = 0,
    HEADSET_POLE_4 = 1,
};

enum {
    FALSE = 0,
    TRUE = 1,
};

static ssize_t lge_hsd_print_name(struct switch_dev *sdev, char *buf)
{
    switch (switch_get_state(sdev)) {

        case HEADSET_NO_DEVICE:
            return sprintf(buf, "No Headset Device");

        case HEADSET_WITH_MIC:
            return sprintf(buf, "Headset w/ MIC");

        case HEADSET_NO_MIC:
            return sprintf(buf, "Headset w/o MIC");
    }

    return -EINVAL;
}

static ssize_t lge_hsd_print_state(struct switch_dev *sdev, char *buf)
{
    return sprintf(buf, "%d\n", switch_get_state(sdev));
}

static void button_pressed(struct work_struct *work)
{
    struct delayed_work *dwork = container_of(work, struct delayed_work, work);
    struct hsd_info *hi = container_of(dwork, struct hsd_info, work_for_key_pressed);
    struct pm8xxx_adc_chan_result result;
    struct ear_3button_info_table *table;

    int acc_read_value=0, i=0, rc=0, table_size=ARRAY_SIZE(max1462x_ear_3button_type_data);

    HSD_DBG("button_pressed begin \n");

    if( gpio_cansleep(hi->gpio_det) ) {
        if( gpio_get_value_cansleep(hi->gpio_det) ) {
            HSD_ERR("button_pressed but ear jack is plugged out already! just ignore the event.\n");
            return;
        }
    } else {
        if( gpio_get_value(hi->gpio_det) ) {
            HSD_ERR("button_pressed but ear jack is plugged out already! just ignore the event.\n");
            return;
        }
    }

	for (i = 0; i < table_size; i++) {
        rc = pm8xxx_adc_mpp_config_read(hi->adc_mpp_num, hi->adc_channel, &result);

        if (rc < 0) {
            if (rc == -ETIMEDOUT) {
                HSD_ERR("button_pressed : adc read timeout[try count:%d]\n", i+1);
            } else {
                HSD_ERR("button_pressed : adc read error - rc:%d[try count:%d]\n", rc, i+1);
            }
        } else {
            acc_read_value = (int)result.physical;
            HSD_DBG("======= acc_read_value - %d [try count:%d] =======\n", (int)result.physical, i+1);

            if( acc_read_value > VDOWN_MAX ) {
                HSD_DBG("********** read again acc_read_value [try count:%d] **********\n", i+1);
                continue;
            }

            // if success, exit from loop
            break;
        }
	}

    for (i = 0; i < table_size; i++) {
        table = &max1462x_ear_3button_type_data[i];
        // [AUDIO_BSP] 20130110, junday.lee, include min value '=' for 1 button earjack (ADC value= 0)
        if ((acc_read_value <= table->PERMISS_REANGE_MAX)&&(acc_read_value >= table->PERMISS_REANGE_MIN)) {

            atomic_set(&hi->btn_state, HEADSET_BTN_PRESSED);

            switch(table->ADC_HEADSET_BUTTON){
                case KEY_MEDIA:
                    input_report_key(hi->input, KEY_MEDIA, 1);
                    HSD_DBG("********** KEY_MEDIA **********\n");
                    break;

                case KEY_VOLUMEUP:
                    input_report_key(hi->input, KEY_VOLUMEUP, 1);
                    HSD_DBG("********** KEY_VOLUMEUP **********\n");
                    break;

                case KEY_VOLUMEDOWN:
                    input_report_key(hi->input, KEY_VOLUMEDOWN, 1);
                    HSD_DBG("********** KEY_VOLUMEDOWN **********\n");
                    break;
            }

            table->PRESS_OR_NOT = 1;
            input_sync(hi->input);
            break;
        }
    }

    HSD_DBG("button_pressed end \n");
}

static void button_released(struct work_struct *work)
{
    struct delayed_work *dwork = container_of(work, struct delayed_work, work);
    struct hsd_info *hi = container_of(dwork, struct hsd_info, work_for_key_released);
    struct ear_3button_info_table *table;
    int table_size = ARRAY_SIZE(max1462x_ear_3button_type_data);
    int i;

    if( gpio_cansleep(hi->gpio_det) ) {
        if( gpio_get_value_cansleep(hi->gpio_det) ) {
        //if( gpio_get_value_cansleep(hi->gpio_det) && !(atomic_read(&hi->btn_state) == HEADSET_BTN_PRESSED) ) {
            HSD_ERR("button_released but ear jack is plugged out already! just ignore the event.\n");
            return;
        }
    } else {
        if( gpio_get_value(hi->gpio_det) ) {
        //if( gpio_get_value(hi->gpio_det) && !(atomic_read(&hi->btn_state) == HEADSET_BTN_PRESSED) ) {
            HSD_ERR("button_released but ear jack is plugged out already! just ignore the event.\n");
            return;
        }
    }

    HSD_DBG("======= button_released =======");

    for (i = 0; i < table_size; i++) {
        table = &max1462x_ear_3button_type_data[i];
        if (table->PRESS_OR_NOT) {
            atomic_set(&hi->btn_state, HEADSET_BTN_RELEASED);
            switch(table->ADC_HEADSET_BUTTON){
                case  KEY_MEDIA :
                input_report_key(hi->input, KEY_MEDIA, 0);
                break;
            case KEY_VOLUMEUP :
                input_report_key(hi->input, KEY_VOLUMEUP, 0);
                break;
            case KEY_VOLUMEDOWN :
                input_report_key(hi->input, KEY_VOLUMEDOWN, 0);
                break;
            }

            table->PRESS_OR_NOT = 0;
            input_sync(hi->input);
            break;
        }
    }
}

static void insert_headset(struct hsd_info *hi)
{
    int earjack_type;

    HSD_DBG("insert_headset");

    // set irq to wake up in sleep mode
    irq_set_irq_wake(hi->irq_key, 1);

    // set high to an external LDO for mic bias
    if (hi->set_headset_mic_bias) {
        hi->set_headset_mic_bias(TRUE);
    } else if( hi->external_ldo_mic_bias > 0 ) {
        gpio_cansleep(hi->external_ldo_mic_bias) ?
            gpio_set_value_cansleep(hi->external_ldo_mic_bias, 1) : gpio_set_value(hi->external_ldo_mic_bias, 1);
    }

    // set mode pin high
    gpio_direction_output(hi->gpio_mode, 1);
    msleep(25);

    /* check if 3-pole or 4-pole
    * 1. read gpio_swd
    * 2. check if 3-pole or 4-pole
    * 3-1. NOT regiter irq with gpio_swd if 3-pole. complete.
    * 3-2. regiter irq with gpio_swd if 4-pole
    * 4. read MPP1 and decide a pressed key when interrupt occurs
    */

    /* 1. read gpio_swd */
    if( gpio_cansleep(hi->gpio_swd) )
        earjack_type = gpio_get_value_cansleep(hi->gpio_swd);
    else
        earjack_type = gpio_get_value(hi->gpio_swd);

    /* 2. check if 3-pole or 4-pole */
    if ( earjack_type == HEADSET_POLE_4 ) {// high
        HSD_DBG("======= 4 polarity earjack =======");

        atomic_set(&hi->is_3_pole_or_not, HEADSET_POLE_4);

        mutex_lock(&hi->mutex_lock);
        switch_set_state(&hi->sdev, HEADSET_WITH_MIC);
        mutex_unlock(&hi->mutex_lock);

        if (!atomic_read(&hi->irq_key_enabled)) {
            unsigned long irq_flags;
            HSD_DBG("irq_key_enabled = enable");

            local_irq_save(irq_flags);
            enable_irq(hi->irq_key);
            local_irq_restore(irq_flags);

            atomic_set(&hi->irq_key_enabled, TRUE);
        }

        // set mode pin high
#if 1
        //gpio_direction_output(hi->gpio_mode, 1);
#else
        gpio_tlmm_config(           // GPIOMUX_FUNC_GPIO
            GPIO_CFG(hi->gpio_mode, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, 0xf),
                GPIO_CFG_ENABLE);
#endif

        input_report_switch(hi->input, SW_HEADPHONE_INSERT, 1);
        input_report_switch(hi->input, SW_MICROPHONE_INSERT, 1);
        input_sync(hi->input);
    } else {// low
        HSD_DBG("********** 3 polarity earjack **********");

        atomic_set(&hi->is_3_pole_or_not, HEADSET_POLE_3);

        mutex_lock(&hi->mutex_lock);
        switch_set_state(&hi->sdev, HEADSET_NO_MIC);
        mutex_unlock(&hi->mutex_lock);

        irq_set_irq_wake(hi->irq_key, 0);

        // set low to an external LDO for mic bias
        if (hi->set_headset_mic_bias) {
            hi->set_headset_mic_bias(FALSE);
        } else if( hi->external_ldo_mic_bias > 0 ) {
            gpio_cansleep(hi->external_ldo_mic_bias) ?
                gpio_set_value_cansleep(hi->external_ldo_mic_bias, 0) : gpio_set_value(hi->external_ldo_mic_bias, 0);
        }
        // set mode pin high-z
#if 1
        gpio_direction_input(hi->gpio_mode);
#else
        gpio_tlmm_config(           // GPIOMUX_FUNC_GPIO
            GPIO_CFG(hi->gpio_mode, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
              GPIO_CFG_ENABLE);
#endif

        input_report_switch(hi->input, SW_HEADPHONE_INSERT, 1);
        input_sync(hi->input);
    }
}

static void remove_headset(struct hsd_info *hi)
{
    int has_mic = switch_get_state(&hi->sdev);

    HSD_DBG("remove_headset");

    atomic_set(&hi->is_3_pole_or_not, HEADSET_POLE_INIT);
    mutex_lock(&hi->mutex_lock);
    switch_set_state(&hi->sdev, HEADSET_NO_DEVICE);
    mutex_unlock(&hi->mutex_lock);

    if (atomic_read(&hi->irq_key_enabled)) {
        unsigned long irq_flags;

        local_irq_save(irq_flags);
        disable_irq(hi->irq_key);
        local_irq_restore(irq_flags);
        atomic_set(&hi->irq_key_enabled, FALSE);
    }

    if( atomic_read(&hi->btn_state) == HEADSET_BTN_PRESSED )
#ifdef	CONFIG_MAX1462X_USE_LOCAL_WORK_QUEUE
        queue_delayed_work(local_max1462x_workqueue, &(hi->work_for_key_released), hi->latency_for_key );
#else
        schedule_delayed_work(&(hi->work_for_key_released), hi->latency_for_key );
#endif

    input_report_switch(hi->input, SW_HEADPHONE_INSERT, 0);

    if (has_mic == HEADSET_WITH_MIC) {
        irq_set_irq_wake(hi->irq_key, 0);
        input_report_switch(hi->input, SW_MICROPHONE_INSERT, 0);

        // set low to an external LDO for mic bias
        if (hi->set_headset_mic_bias) {
            hi->set_headset_mic_bias(FALSE);
        } else if( hi->external_ldo_mic_bias > 0 ) {
            gpio_cansleep(hi->external_ldo_mic_bias) ?
                gpio_set_value_cansleep(hi->external_ldo_mic_bias, 0) : gpio_set_value(hi->external_ldo_mic_bias, 0);
        }
    }

    input_sync(hi->input);
}

static void detect_work(struct work_struct *work)
{
    int state = 0;
    struct delayed_work *dwork = container_of(work, struct delayed_work, work);
	struct hsd_info *hi = container_of(dwork, struct hsd_info, work);

    HSD_DBG("detect_work");

    if( gpio_cansleep(hi->gpio_det) )
        state = gpio_get_value_cansleep(hi->gpio_det);
    else
        state = gpio_get_value(hi->gpio_det);

    if( state == 1 ) {  // gpio_det high - jack out
        if( switch_get_state(&hi->sdev) != HEADSET_NO_DEVICE ) {
            HSD_DBG("======= LGE headset removing =======");
            remove_headset(hi);
#ifdef CONFIG_LGE_BROADCAST_ONESEG
            isdbt_hw_antenna_switch(0);
            ear_state = 0;
#endif
        } else {
            HSD_DBG("err_invalid_state state = %d\n", state);
        }
    } else {    // gpio_det low - jack in
        if( switch_get_state(&hi->sdev) == HEADSET_NO_DEVICE ) {
            HSD_DBG("********** LGE headset inserting **********");
            insert_headset(hi);

#ifdef CONFIG_LGE_BROADCAST_ONESEG
            isdbt_hw_antenna_switch(1);
            ear_state = 1;
#endif
        } else {
            HSD_DBG("err_invalid_state state = %d\n", state);
        }
    }
}

#ifdef CONFIG_LGE_BROADCAST_ONESEG //for 325 BATMAN, L05E MMBI
int check_ear_state(void)
{
    return ear_state;
}
EXPORT_SYMBOL(check_ear_state);
#endif

static irqreturn_t earjack_det_irq_handler(int irq, void *dev_id)
{
    struct hsd_info *hi = (struct hsd_info *) dev_id;

    wake_lock_timeout(&ear_hook_wake_lock, 2 * HZ);

    HSD_DBG("earjack_det_irq_handler");

#ifdef CONFIG_MAX1462X_USE_LOCAL_WORK_QUEUE
    queue_delayed_work(local_max1462x_workqueue, &(hi->work), hi->latency_for_detection);

#else
    schedule_delayed_work(&(hi->work), hi->latency_for_detection);
#endif
    return IRQ_HANDLED;
}

static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
    struct hsd_info *hi = (struct hsd_info *) dev_id;

    int value;

    wake_lock_timeout(&ear_hook_wake_lock, 2 * HZ);

    HSD_DBG("button_irq_handler");

    // low:pressed, high:released
    if( gpio_cansleep(hi->gpio_swd) )
        value = gpio_get_value_cansleep(hi->gpio_swd);
    else
        value = gpio_get_value(hi->gpio_swd);

    HSD_DBG("======= hi->gpio_swd : %d =======", value);

    if (value)
        queue_delayed_work(local_max1462x_workqueue, &(hi->work_for_key_released), hi->latency_for_key );
    else
        queue_delayed_work(local_max1462x_workqueue, &(hi->work_for_key_pressed), hi->latency_for_key );

    return IRQ_HANDLED;
}

static int lge_hsd_probe(struct platform_device *pdev)
{
    int ret = 0;
    struct max1462x_platform_data *pdata = pdev->dev.platform_data;

    struct hsd_info *hi;

    HSD_DBG("lge_hsd_probe");

    hi = kzalloc(sizeof(struct hsd_info), GFP_KERNEL);

    if (NULL == hi) {
        HSD_ERR("Failed to allloate headset per device info\n");
        return -ENOMEM;
    }

    platform_set_drvdata(pdev, hi);

    // initialize internal variables
    atomic_set(&hi->btn_state, HEADSET_BTN_INIT);
    atomic_set(&hi->is_3_pole_or_not, HEADSET_POLE_INIT);

    // set key code
    hi->key_code = pdata->key_code; // KEY_RESERVED(0), KEY_MEDIA(226), KEY_VOLUMEUP(115) or KEY_VOLUMEDOWN(114)

    // set GPIO number for each pin
    hi->gpio_mode = pdata->gpio_mode;
    hi->gpio_det = pdata->gpio_det;
    hi->gpio_swd = pdata->gpio_swd;

    // set delayed time for latency
    hi->latency_for_detection = msecs_to_jiffies(pdata->latency_for_detection);
    HSD_DBG("jiffies of hi->latency_for_detection : %u \n", hi->latency_for_detection);

    hi->latency_for_key = msecs_to_jiffies(pdata->latency_for_key);
    HSD_DBG("jiffies of hi->latency_for_key : %u \n", hi->latency_for_key);

    hi->adc_mpp_num = pdata->adc_mpp_num;
    HSD_DBG("hi->adc_mpp_num : %u \n", hi->adc_mpp_num);

    hi->adc_channel = pdata->adc_channel;
    HSD_DBG("hi->adc_channel : %u \n", hi->adc_channel);

    // set gpio for an external LDO control
    if( pdata->external_ldo_mic_bias > 0 ) {
        hi->external_ldo_mic_bias = pdata->external_ldo_mic_bias;
        HSD_DBG("control an external LDO(GPIO# %u) for MIC BIAS", hi->external_ldo_mic_bias);
    } else {
        hi->external_ldo_mic_bias = 0;
        HSD_DBG("don't control an external LDO for MIC");
    }

    // set callback function for an external LDO control
    if( pdata->set_headset_mic_bias!= NULL ) {
        hi->set_headset_mic_bias = pdata->set_headset_mic_bias;
        HSD_DBG("set a func pointer of an external LDO for MIC");
    } else {
        pdata->set_headset_mic_bias = NULL;
        HSD_DBG("don't set a func pointer of an external LDO for MIC");
    }

    mutex_init(&hi->mutex_lock);

    INIT_DELAYED_WORK(&hi->work, detect_work);
    INIT_DELAYED_WORK(&hi->work_for_key_pressed, button_pressed);
    INIT_DELAYED_WORK(&hi->work_for_key_released, button_released);


    // initialize gpio_mode
    // set gpio_mode high as a default, and set mode among high,low,high-Z after deciding 3 or 4 polarity
    ret = gpio_request(hi->gpio_mode, "gpio_mode");
    if (ret < 0) {
        HSD_ERR("Failed to configure gpio%u (gpio_mode) gpio_request\n", hi->gpio_mode);
        goto error_01;
    }

    ret = gpio_direction_output(hi->gpio_mode, 1);
    if (ret < 0) {
        HSD_ERR("Failed to configure gpio%d (gpio_mode) gpio_direction_input\n", hi->gpio_mode);
        goto error_01;
    }

    // initialize gpio_det
    ret = gpio_request(hi->gpio_det, "gpio_det");
    if (ret < 0) {
        HSD_ERR("Failed to configure gpio%u (gpio_det) gpio_request\n", hi->gpio_det);
        goto error_02;
    }

    ret = gpio_direction_input(hi->gpio_det);
    if (ret < 0) {
        HSD_ERR("Failed to configure gpio%u (gpio_det) gpio_direction_input\n", hi->gpio_det);
        goto error_02;
    }

    // initialize gpio_swd
    ret = gpio_request(hi->gpio_swd, "gpio_swd");
    if (ret < 0) {
        HSD_ERR("Failed to configure gpio%u (gpio_swd) gpio_request\n", hi->gpio_swd);
        goto error_03;
    }

    ret = gpio_direction_input(hi->gpio_swd);
    if (ret < 0) {
        HSD_ERR("Failed to configure gpio%u (gpio_swd) gpio_direction_input\n", hi->gpio_swd);
        goto error_03;
    }

    // initialize external_ldo_mic_bias
    if( hi->external_ldo_mic_bias > 0 ) {
        ret = gpio_request(hi->external_ldo_mic_bias, "external_ldo_mic_bias");
        if (ret < 0) {
            HSD_ERR("Failed to configure gpio%u (external_ldo_mic_bias) gpio_request\n", hi->external_ldo_mic_bias);
            goto error_04;
        }

        ret = gpio_direction_output(hi->external_ldo_mic_bias, 0);
        if (ret < 0) {
            HSD_ERR("Failed to configure gpio%u (external_ldo_mic_bias) gpio_direction_input\n", hi->external_ldo_mic_bias);
            goto error_04;
        }

        HSD_DBG("hi->external_ldo_mic_bias value = %d \n",
            gpio_cansleep(hi->external_ldo_mic_bias) ?
                gpio_get_value_cansleep(hi->external_ldo_mic_bias) : gpio_get_value(hi->external_ldo_mic_bias));
    }

    /* initialize irq of detection */
    hi->irq_detect = gpio_to_irq(hi->gpio_det);

    HSD_DBG("hi->irq_detect = %d\n", hi->irq_detect);

    if (hi->irq_detect < 0) {
        HSD_ERR("Failed to get interrupt number\n");
        ret = hi->irq_detect;
        goto error_05;
    }

    ret = request_threaded_irq(hi->irq_detect, NULL, earjack_det_irq_handler,
        IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, pdev->name, hi);

    if (ret) {
        HSD_ERR("failed to request button irq");
        goto error_05;
    }

    ret = irq_set_irq_wake(hi->irq_detect, 1);
    if (ret < 0) {
        HSD_ERR("Failed to set irq_detect interrupt wake\n");
        goto error_05;
    }

    /* initialize irq of gpio_key */
    hi->irq_key = gpio_to_irq(hi->gpio_swd);

    HSD_DBG("hi->irq_key = %d\n", hi->irq_key);

    if (hi->irq_key < 0) {
        HSD_ERR("Failed to get interrupt number\n");
        ret = hi->irq_key;
        goto error_06;
    }

    ret = request_threaded_irq(hi->irq_key, NULL, button_irq_handler,
        IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, pdev->name, hi);

    if (ret) {
        HSD_ERR("failed to request button irq");
        goto error_06;
    }

    disable_irq(hi->irq_key);

#if 0
    ret = irq_set_irq_wake(hi->irq_key, 1);
    if (ret < 0) {
        HSD_ERR("Failed to set irq_key interrupt wake\n");
        goto error_06;
    }
#endif

    /* initialize switch device */
    hi->sdev.name = pdata->switch_name;
    hi->sdev.print_state = lge_hsd_print_state;
    hi->sdev.print_name = lge_hsd_print_name;

    ret = switch_dev_register(&hi->sdev);
    if (ret < 0) {
        HSD_ERR("Failed to register switch device\n");
        goto error_06;
    }

    /* initialize input device */
    hi->input = input_allocate_device();
    if (!hi->input) {
        HSD_ERR("Failed to allocate input device\n");
        ret = -ENOMEM;
        goto error_07;
    }

    hi->input->name = pdata->keypad_name;

    hi->input->id.vendor    = 0x0001;
    hi->input->id.product   = 1;
    hi->input->id.version   = 1;

    // [START] // headset tx noise
    {
        struct pm8xxx_adc_chan_result result;
        int acc_read_value = 0;
        int i, rc;
        int count = 3;

        for (i = 0; i < count; i++)
        {
            rc = pm8xxx_adc_mpp_config_read(hi->adc_mpp_num, hi->adc_channel, &result);

            if (rc < 0)
            {
                if (rc == -ETIMEDOUT) {
                    HSD_ERR("[DEBUG]adc read timeout \n");
                } else {
                    HSD_ERR("[DEBUG]adc read error - %d\n", rc);
                }
            }
            else
            {
                acc_read_value = (int)result.physical;
                HSD_DBG("%s: acc_read_value - %d\n", __func__, (int)result.physical);
                break;
            }
        }
    }
    // [END]

    /*input_set_capability(hi->input, EV_SW, SW_HEADPHONE_INSERT);*/
    set_bit(EV_SYN, hi->input->evbit);
    set_bit(EV_KEY, hi->input->evbit);
    set_bit(EV_SW, hi->input->evbit);
    set_bit(hi->key_code, hi->input->keybit);
    set_bit(SW_HEADPHONE_INSERT, hi->input->swbit);
    set_bit(SW_MICROPHONE_INSERT, hi->input->swbit);
    input_set_capability(hi->input, EV_KEY, KEY_MEDIA);
    input_set_capability(hi->input, EV_KEY, KEY_VOLUMEUP);
    input_set_capability(hi->input, EV_KEY, KEY_VOLUMEDOWN);

    ret = input_register_device(hi->input);
    if (ret) {
        HSD_ERR("Failed to register input device\n");
        goto error_08;
    }

     // to detect in initialization with eacjack insertion
     if( gpio_cansleep(hi->gpio_det) ) {
        if( !gpio_get_value_cansleep(hi->gpio_det) ) {
#ifdef CONFIG_MAX1462X_USE_LOCAL_WORK_QUEUE
            queue_delayed_work(local_max1462x_workqueue, &(hi->work), 0);
#else
            schedule_delayed_work(&(hi->work), 0);
#endif
        }
     } else {
        if( !gpio_get_value_cansleep(hi->gpio_det) ) {
#ifdef CONFIG_MAX1462X_USE_LOCAL_WORK_QUEUE
            queue_delayed_work(local_max1462x_workqueue, &(hi->work), 0);
#else
            schedule_delayed_work(&(hi->work), 0);
#endif
        }
    }

    return ret;

error_08:
    input_free_device(hi->input);
error_07:
    switch_dev_unregister(&hi->sdev);
error_06:
    free_irq(hi->irq_key, 0);
error_05:
    free_irq(hi->irq_detect, 0);
error_04:
    if( hi->external_ldo_mic_bias > 0 )
        gpio_free(hi->external_ldo_mic_bias);
error_03:
    gpio_free(hi->gpio_swd);
error_02:
    gpio_free(hi->gpio_det);
error_01:
    mutex_destroy(&hi->mutex_lock);
    kfree(hi);
    return ret;
}

static int lge_hsd_remove(struct platform_device *pdev)
{
    struct hsd_info *hi = (struct hsd_info *)platform_get_drvdata(pdev);

    HSD_DBG("lge_hsd_remove");

    if (switch_get_state(&hi->sdev))
        remove_headset(hi);

    input_unregister_device(hi->input);
    switch_dev_unregister(&hi->sdev);

    free_irq(hi->irq_key, 0);
    free_irq(hi->irq_detect, 0);
    gpio_free(hi->gpio_det);
    gpio_free(hi->gpio_swd);
    gpio_free(hi->gpio_mode);

    if( hi->external_ldo_mic_bias > 0 )
        gpio_free(hi->external_ldo_mic_bias);

    mutex_destroy(&hi->mutex_lock);

    kfree(hi);

    return 0;
}

// do not set a pin, mode, low when going to sleep mode in call
#if defined(SET_MODE_PIN_LOW_IN_SLEEP)
int lge_hsd_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct hsd_info *hi = (struct hsd_info *)platform_get_drvdata(pdev);

    if( atomic_read(&hi->is_3_pole_or_not) == HEADSET_POLE_4 ) {
        // set mode low
#if 1
        gpio_direction_output(hi->gpio_mode, 0);
        //gpio_cansleep(hi->gpio_mode) ? gpio_set_value_cansleep(hi->gpio_mode, 0) : gpio_set_value(hi->gpio_mode, 0);
        msleep(16); // MODE 1 -> 0 Detection Time in Spec is 16ms.
#else
        gpio_tlmm_config(           // GPIOMUX_FUNC_GPIO
            GPIO_CFG(hi->gpio_mode, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
                GPIO_CFG_ENABLE);
#endif
        HSD_DBG("lge_hsd_suspend - 4 polarity, mode set by low\n");
    }

    return 0;
}

int lge_hsd_resume(struct platform_device *pdev)
{
    struct hsd_info *hi = (struct hsd_info *)platform_get_drvdata(pdev);

    if( atomic_read(&hi->is_3_pole_or_not) == HEADSET_POLE_4 ) {
        // set mode high
#if 1
        gpio_direction_output(hi->gpio_mode, 1);
        msleep(16); // MODE 0 -> 1 Detection Time in Spec is 4ms, but 4ms is not enough to set a value to a regiseter.
        //gpio_cansleep(hi->gpio_mode) ? gpio_set_value_cansleep(hi->gpio_mode, 1) : gpio_set_value(hi->gpio_mode, 1);
#else
        gpio_tlmm_config(           // GPIOMUX_FUNC_GPIO
            GPIO_CFG(hi->gpio_mode, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
                GPIO_CFG_ENABLE);
#endif
        HSD_DBG("lge_hsd_resume - 4 polarity, mode set by high\n");
    }

    return 0;
}
#endif  //SET_MODE_PIN_LOW_IN_SLEEP



static struct platform_driver lge_hsd_driver = {
    .probe          = lge_hsd_probe,
    .remove         = lge_hsd_remove,
#if defined(SET_MODE_PIN_LOW_IN_SLEEP)   // do not set mode low when going to sleep mode in call
    .suspend        = lge_hsd_suspend,
    .resume         = lge_hsd_resume,
#endif  //SET_MODE_PIN_LOW_IN_SLEEP
    .driver         = {
        .name           = "max1462x",
        .owner          = THIS_MODULE,
    },
};

static int __init lge_hsd_init(void)
{
    int ret;

    HSD_DBG("lge_hsd_init");

#ifdef CONFIG_MAX1462X_USE_LOCAL_WORK_QUEUE
    local_max1462x_workqueue = create_workqueue("max1462x");
    if(!local_max1462x_workqueue)
        return -ENOMEM;
#endif
    HSD_DBG("lge_hsd_init : wake_lock_init");
    wake_lock_init(&ear_hook_wake_lock, WAKE_LOCK_SUSPEND, "ear_hook");

    ret = platform_driver_register(&lge_hsd_driver);
    if (ret) {
        HSD_ERR("Fail to register platform driver\n");
    }

    return ret;
}

static void __exit lge_hsd_exit(void)
{
    HSD_DBG("lge_hsd_exit");

#ifdef CONFIG_MAX1462X_USE_LOCAL_WORK_QUEUE
    if(local_max1462x_workqueue)
        destroy_workqueue(local_max1462x_workqueue);
    local_max1462x_workqueue = NULL;
#endif

    platform_driver_unregister(&lge_hsd_driver);

    HSD_DBG("lge_hsd_exit : wake_lock_destroy");
    wake_lock_destroy(&ear_hook_wake_lock);
}

/* to make init after pmic8058-othc module */
/* module_init(lge_hsd_init); */
late_initcall_sync(lge_hsd_init);
module_exit(lge_hsd_exit);

MODULE_AUTHOR("GooYeon Jung <gooyeon.jung@lge.com>");
MODULE_DESCRIPTION("LGE Headset detection driver (max1462x)");
MODULE_LICENSE("GPL");
#endif
