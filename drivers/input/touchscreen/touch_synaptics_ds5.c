/* Touch_synaptics.c
 *
 * Copyright (C) 2013 LGE.
 *
 * Author: yehan.ahn@lge.com, hyesung.shin@lge.com
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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/async.h>

#include <linux/input/lge_touch_core_ds5.h>
#include <linux/input/touch_synaptics_ds5.h>

#if defined(CONFIG_MACH_MSM8974_VU3_KR)
#include "SynaImage_for_VU3_revA.h"
#include "SynaImage_for_VU3_revB.h"
#elif defined(CONFIG_LGE_Z_TOUCHSCREEN)
#include "SynaImage_for_Z_revA.h"
#include "SynaImage_for_Z_revB.h"
#elif defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
#include "SynaImage_for_5M.h"
#else
#include "SynaImage_for_G2_LGIT_revA.h"
#include "SynaImage_for_G2_LGIT_revB.h"
#include "SynaImage_for_G2_TPK.h"
#endif
/* RMI4 spec from 511-000405-01 Rev.D
 * Function	Purpose									See page
 * $01		RMI Device Control						45
 * $1A		0-D capacitive button sensors			61
 * $05		Image Reporting							68
 * $07		Image Reporting							75
 * $08		BIST									82
 * $09		BIST									87
 * $11		2-D TouchPad sensors					93
 * $19		0-D capacitive button sensors			141
 * $30		GPIO/LEDs								148
 * $31		LEDs									162
 * $34		Flash Memory Management					163
 * $36		Auxiliary ADC							174
 * $54		Test Reporting							176
 */
#define RMI_DEVICE_CONTROL				0x01
#define TOUCHPAD_SENSORS				0x12
#if defined(CONFIG_MACH_MSM8974_VU3_KR) || defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
#define CAPACITIVE_BUTTON_SENSORS		0x1A
#define BUTTON_THRESHOLD_CONTROL
#endif
#define FLASH_MEMORY_MANAGEMENT			0x34
#define ANALOG_CONTROL					0x54
#define SENSOR_CONTROL						0x55


/* Register Map & Register bit mask
 * - Please check "One time" this map before using this device driver
 */
/* RMI_DEVICE_CONTROL */
#define MANUFACTURER_ID_REG				(ts->common_fc.dsc.query_base)			/* Manufacturer ID */
#define FW_REVISION_REG					(ts->common_fc.dsc.query_base+3)		/* FW revision */
#define PRODUCT_ID_REG					(ts->common_fc.dsc.query_base+11)		/* Product ID */

#define DEVICE_COMMAND_REG				(ts->common_fc.dsc.command_base)

#define DEVICE_CONTROL_REG 				(ts->common_fc.dsc.control_base)		/* Device Control */
#define DEVICE_CONTROL_NORMAL_OP		0x00	/* sleep mode : go to doze mode after 500 ms */
#define DEVICE_CONTROL_SLEEP 			0x01	/* sleep mode : go to sleep */
#define DEVICE_CONTROL_SPECIFIC			0x02	/* sleep mode : go to doze mode after 5 sec */
#define DEVICE_CONTROL_NOSLEEP			0x04
#define DEVICE_CONTROL_CONFIGURED		0x80
#ifdef CUST_G2_TOUCH
#define DEVICE_CHARGER_CONNECTED		0x20
#endif

#define INTERRUPT_ENABLE_REG			(ts->common_fc.dsc.control_base+1)		/* Interrupt Enable 0 */

#define DEVICE_STATUS_REG				(ts->common_fc.dsc.data_base)			/* Device Status */
#define DEVICE_FAILURE_MASK				0x03
#define DEVICE_CRC_ERROR_MASK			0x04
#define DEVICE_STATUS_FLASH_PROG		0x40
#define DEVICE_STATUS_UNCONFIGURED		0x80

#define INTERRUPT_STATUS_REG			(ts->common_fc.dsc.data_base+1)		/* Interrupt Status */
#define INTERRUPT_MASK_FLASH			0x01
#define INTERRUPT_MASK_ABS0				0x04
#define INTERRUPT_MASK_BUTTON			0x10

/* TOUCHPAD_SENSORS */
#define FINGER_COMMAND_REG				(ts->finger_fc.dsc.command_base)
#define MOTION_SUPPRESSION				(ts->finger_fc.dsc.control_base+6)

#define OBJECT_TYPE_AND_STATUS_REG				(ts->finger_fc.dsc.data_base)			/* Finger State */
#define OBJECT_ATTENTION_REG			(ts->finger_fc.dsc.data_base+2)
#define FINGER_DATA_REG_START			(ts->finger_fc.dsc.data_base)		/* Finger Data Register */
#define REG_OBJECT_TYPE_AND_STATUS	0
#define REG_X_LSB						1
#define REG_X_MSB					2
#define REG_Y_LSB						3
#define REG_Y_MSB					4
#define REG_Z							5
#define REG_WX						6
#define REG_WY							7
#ifdef RESOLUTION_FROM_FW
#define MAXIMUM_XY_COORDINATE_REG		(ts->finger_fc.dsc.control_base)
#endif
/* ANALOG_CONTROL */
#define ANALOG_COMMAND_REG				(ts->analog_fc.dsc.command_base)

#define ANALOG_CONTROL_REG				(ts->analog_fc.dsc.control_base)

#if defined(CONFIG_MACH_MSM8974_VU3_KR) || defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
/* CAPACITIVE_BUTTON_SENSORS */
#define BUTTON_COMMAND_REG				(ts->button_fc.dsc.command_base)
#define BUTTON_DATA_REG					(ts->button_fc.dsc.data_base)			/* Button Data */
#endif

/* FLASH_MEMORY_MANAGEMENT */
#define FLASH_CONFIG_ID_REG				(ts->flash_fc.dsc.control_base)		/* Flash Control */
#define FLASH_CONTROL_REG				(ts->flash_fc.dsc.data_base+2)
#ifdef CUST_G2_TOUCH
#define FLASH_STATUS_REG				(ts->flash_fc.dsc.data_base+3)
#define FLASH_STATUS_MASK				0xFF
#else
#define FLASH_STATUS_MASK				0xF0
#endif

/* Page number */
#define COMMON_PAGE						(ts->common_fc.function_page)
#define FINGER_PAGE						(ts->finger_fc.function_page)
#if defined(CONFIG_MACH_MSM8974_VU3_KR) || defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
#define BUTTON_PAGE						(ts->button_fc.function_page)
#endif
#define ANALOG_PAGE						(ts->analog_fc.function_page)
#define FLASH_PAGE						(ts->flash_fc.function_page)
#define SENSOR_PAGE					(ts->sensor_fc.function_page)
#define DEFAULT_PAGE					0x00
#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
#define LPWG_CTRL_PAGE					0x04
#define MULTITAP_COUNT_REG				0x31
#define MAX_INTERTAP_TIME_REG			0x33
#define MULTITAP_PRESS_RELEASE_DISTANCE	0x34
#define INTERTAP_DISTANCE_REG			0x35
#endif

/* Get user-finger-data from register.
 */
#define TS_SNTS_GET_X_POSITION(_msb_reg, _lsb_reg) \
		(((u16)((_msb_reg <<8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define TS_SNTS_GET_Y_POSITION(_msb_reg, _lsb_reg) \
		(((u16)((_msb_reg <<8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define TS_SNTS_GET_WIDTH_MAJOR(_width_x, _width_y) \
			((_width_x - _width_y) > 0) ? _width_x : _width_y
#define TS_SNTS_GET_WIDTH_MINOR(_width_x, _width_y) \
			((_width_x - _width_y) > 0) ? _width_y : _width_x

#define TS_SNTS_GET_ORIENTATION(_width_y, _width_x) \
		((_width_y - _width_x) > 0) ? 0 : 1
#define TS_SNTS_GET_PRESSURE(_pressure) \
		_pressure
/* Get user-finger-data from S7020 register.
 */
#define TS_SNTS_GET_X_POSITION_S7020(_high_reg, _low_reg) \
		( ((u16)((_high_reg << 4) & 0x0FF0) | (u16)(_low_reg&0x0F)))
#define TS_SNTS_GET_Y_POSITION_S7020(_high_reg, _low_reg) \
		( ((u16)((_high_reg << 4) & 0x0FF0) | (u16)((_low_reg >> 4) & 0x0F)))
#define TS_SNTS_GET_WIDTH_MAJOR_S7020(_width) \
		((((_width & 0xF0) >> 4) - (_width & 0x0F)) > 0) ? (_width & 0xF0) >> 4 : _width & 0x0F
#define TS_SNTS_GET_WIDTH_MINOR_S7020(_width) \
		((((_width & 0xF0) >> 4) - (_width & 0x0F)) > 0) ? _width & 0x0F : (_width & 0xF0) >> 4
#define TS_SNTS_GET_ORIENTATION_S7020(_width) \
		((((_width & 0xF0) >> 4) - (_width & 0x0F)) > 0) ? 0 : 1

/* GET_BIT_MASK & GET_INDEX_FROM_MASK
 *
 * For easily checking the user input.
 * Usually, User use only one or two fingers.
 * However, we should always check all finger-status-register
 * because we can't know the total number of fingers.
 * These Macro will prevent it.
 */
#define GET_BIT_MASK(_finger_reg)	\
		((_finger_reg[0][0] & 0x0F)?1:0)	| ((_finger_reg[1][0] & 0x0F)?1:0)<<1 |	\
		((_finger_reg[2][0] & 0x0F)?1:0)<<2 | ((_finger_reg[3][0] & 0x0F)?1:0)<<3 | \
		((_finger_reg[4][0] & 0x0F)?1:0)<<4 | ((_finger_reg[5][0] & 0x0F)?1:0)<<5 |	\
		((_finger_reg[6][0] & 0x0F)?1:0)<<6	| ((_finger_reg[7][0] & 0x0F)?1:0)<<7 |	\
		((_finger_reg[8][0] & 0x0F)?1:0)<<8 | ((_finger_reg[9][0] & 0x0F)?1:0)<<9
/* S7020 */
#define GET_BIT_MASK_S7020(_finger_status_reg)	\
		(_finger_status_reg[2] & 0x04)<<7 | (_finger_status_reg[2] & 0x01)<<8 |	\
		(_finger_status_reg[1] & 0x40)<<1 | (_finger_status_reg[1] & 0x10)<<2 | \
		(_finger_status_reg[1] & 0x04)<<3 | (_finger_status_reg[1] & 0x01)<<4 |	\
		(_finger_status_reg[0] & 0x40)>>3 | (_finger_status_reg[0] & 0x10)>>2 | \
		(_finger_status_reg[0] & 0x04)>>1 | (_finger_status_reg[0] & 0x01)


#define GET_INDEX_FROM_MASK(_index, _bit_mask, _max_finger)	\
		for(; !((_bit_mask>>_index)&0x01) && _index <= _max_finger; _index++);	\
		if (_index <= _max_finger) _bit_mask &= ~(_bit_mask & (1<<(_index)));

#ifdef CUST_G2_TOUCH
extern int ts_charger_plug;
#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
extern void send_uevent_lpwg(int type);
#endif
#define OLD_S3404A_BOOT_ID	1245782
#define S3404A_BOOT_ID 	1328275
#define S3404B_BOOT_ID 	1380018
#define PALM_DETECT_SIZE 28
#endif
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
bool s7020 = false;	/*board is gk or not*/
extern void synaptics_change_fw_on_init(struct i2c_client* client); //OMEGA_TOUCH_THERMAL_CHECK
#ifdef CUST_G2_TOUCH_WAKEUP_GESTURE
extern int quick_cover_status;
#endif
#endif

#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
#define MS_TO_NS(x)	(x * 1E6L)

struct workqueue_struct*	touch_multi_tap_wq;

static enum hrtimer_restart touch_multi_tap_timer_handler(struct hrtimer *multi_tap_timer)
{
	struct synaptics_ts_data *ts = container_of(multi_tap_timer, struct synaptics_ts_data, multi_tap_timer);
	queue_work(touch_multi_tap_wq, &ts->multi_tap_work);

	return HRTIMER_NORESTART;
}

static u8 custom_gesture_status = 0;
static u8 multi_tap_fail_try = 0;
static void touch_multi_tap_work(struct work_struct *multi_tap_work)
{
	struct synaptics_ts_data *ts = container_of(multi_tap_work, struct synaptics_ts_data, multi_tap_work);
	unsigned char *r_mem = NULL;
	r_mem = kzalloc(sizeof(char) * (3), GFP_KERNEL);
	if (touch_i2c_read(ts->client, REPORT_WAKEUP_GESTURE_ONLY_REG, (3), r_mem) < 0) {
		TOUCH_ERR_MSG("%d bytes read fail!", (3));
	} else {
		*(r_mem+2) = 0x2;
		if (touch_i2c_write(ts->client, REPORT_WAKEUP_GESTURE_ONLY_REG, (3), r_mem) < 0)
			TOUCH_ERR_MSG("REPORT_WAKEUP_GESTURE_ONLY_REG write fail");
	}
	if (r_mem != NULL)
		kfree(r_mem);
	if(report_lpwg){
		send_uevent_lpwg(LPWG_PASSWORD);
	} else {
		TOUCH_INFO_MSG("[%s] Don't send uevent. Report LPWG is not set!", __func__);
	}
}
#endif

/* wrapper function for i2c communication - except defalut page
 * if you have to select page for reading or writing, then using this wrapper function */
int synaptics_ts_page_data_read(struct i2c_client *client, u8 page, u8 reg, int size, u8 *data)
{
	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, page) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_read(client, reg, size, data) < 0)) {
		TOUCH_ERR_MSG("[%dP:%d]register read fail\n", page, reg);
		return -EIO;
	}

	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	return 0;
}

int synaptics_ts_page_data_write(struct i2c_client *client, u8 page, u8 reg, int size, u8 *data)
{
	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, page) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_write(client, reg, size, data) < 0)) {
		TOUCH_ERR_MSG("[%dP:%d]register read fail\n", page, reg);
		return -EIO;
	}

	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	return 0;
}

int synaptics_ts_page_data_write_byte(struct i2c_client *client, u8 page, u8 reg, u8 data)
{
	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, page) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_write_byte(client, reg, data) < 0)) {
		TOUCH_ERR_MSG("[%dP:%d]register write fail\n", page, reg);
		return -EIO;
	}

	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	return 0;
}

#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
static u8 lpwg_data[MAX_POINT_SIZE_FOR_LPWG*4];
#endif
int synaptics_ts_get_data(struct i2c_client *client, struct touch_data* data)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);

	u16 touch_finger_bit_mask=0;
#ifdef CUST_G2_TOUCH
	u8 palm_detected = 0;
	int z_sum = 0;
	u8 udata[2] = {0,0};
	u8 cns = 0;
	u16 im = 0;
	u16 cid_im = 0;
	u16 feq_scan_im = 0;
#endif
	u8 finger_index = 0;
	u8 index = 0;
#if defined(CONFIG_MACH_MSM8974_VU3_KR) || defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
	u8 cnt;
#endif
	data->total_num = 0;

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");
	
		if (unlikely(touch_i2c_read(client, DEVICE_STATUS_REG,
			sizeof(ts->ts_data.interrupt_status_reg),
			&ts->ts_data.device_status_reg) < 0)) {
				TOUCH_ERR_MSG("DEVICE_STATUS_REG read fail\n");
				goto err_synaptics_getdata;
		}


	/* ESD damage check */
		if ((ts->ts_data.device_status_reg & DEVICE_FAILURE_MASK) == DEVICE_FAILURE_MASK) {
			TOUCH_ERR_MSG("ESD damage occured. Reset Touch IC\n");
			goto err_synaptics_device_damage;
		}
		
		/* Internal reset check */
		if (((ts->ts_data.device_status_reg & DEVICE_STATUS_UNCONFIGURED) >> 7) == 1) {
			TOUCH_ERR_MSG("Touch IC resetted internally. Reconfigure register setting\n");
			goto err_synaptics_device_damage;
		}


		if (unlikely(touch_i2c_read(client, INTERRUPT_STATUS_REG,
			sizeof(ts->ts_data.interrupt_status_reg),
			&ts->ts_data.interrupt_status_reg) < 0)) {
		TOUCH_ERR_MSG("INTERRUPT_STATUS_REG read fail\n");
		goto err_synaptics_getdata;
		}

	if (unlikely(touch_debug_mask & DEBUG_GET_DATA))
		TOUCH_INFO_MSG("Interrupt_status : 0x%x\n", ts->ts_data.interrupt_status_reg);

#ifdef CUST_G2_TOUCH
//do nothing
#else
	/* IC bug Exception handling - Interrupt status reg is 0 when interrupt occur */
	if (ts->ts_data.interrupt_status_reg == 0) {
		TOUCH_ERR_MSG("Interrupt_status reg is 0. Something is wrong in IC\n");
		goto err_synaptics_device_damage;
	}
#endif

	/* Because of ESD damage... */
	if (unlikely(ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_FLASH)) {
		TOUCH_ERR_MSG("Impossible Interrupt\n");
		goto err_synaptics_device_damage;
	}

#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
	if (ts->double_tap_enable)  {
//		send_uevent_lpwg(client, LPWG_DOUBLE_TAP);
		TOUCH_INFO_MSG("[synaptics_ts_get_data] ts->double_tap_enable");
		goto ignore_interrupt;
	} else if (ts->password_enable) {
#if defined(KNOCKON_MASK)
		TOUCH_INFO_MSG("[synaptics_ts_get_data] ts->password_enable ts->password_tap_count=X");
#else
		TOUCH_INFO_MSG("[synaptics_ts_get_data] ts->password_enable ts->password_tap_count=%d", ts->password_tap_count);
#endif
		if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, 4) < 0)) {
			TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
			return -EIO;
		}
		if (unlikely(touch_i2c_read(client, 0x0, 1, lpwg_data) < 0)) {
			TOUCH_ERR_MSG("MultipleTap Custom Gesture Status read fail\n");
		//	goto err_synaptics_getdata;
		}
		TOUCH_INFO_MSG("custom gesture status %d \n", lpwg_data[0]);

		custom_gesture_status = lpwg_data[0];

		if (unlikely(touch_i2c_read(client, 0x1, ts->password_tap_count*4, lpwg_data) < 0)) {
			TOUCH_ERR_MSG("MultipleTap Custom Data read fail\n");
		//	goto err_synaptics_getdata;
		}
		{
			int i = 0;
			if (custom_gesture_status) {
#if defined(KNOCKON_MASK)
				TOUCH_INFO_MSG("[%s] Get lpwg data !!!!!\n", __func__);
#endif
				for(i=0;i<ts->password_tap_count;i++) {
#if !defined(KNOCKON_MASK)
					TOUCH_INFO_MSG("lpwg data %d: 0:0x%-4x 1:0x%-4x 2:0x%-4x 3:0x%-4x	(x %d 	y %d)\n",  i,
							lpwg_data[4*i], lpwg_data[4*i+1],lpwg_data[4*i+2],lpwg_data[4*i+3],
							lpwg_data[4*i+1]<<8 | lpwg_data[4*i], lpwg_data[4*i+3]<<8 | lpwg_data[4*i+2]);
#endif
				}
				report_lpwg = true;
			}
		}
		if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0) < 0)) {
			TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
			return -EIO;
		}
		{
			unsigned char *r_mem = NULL;
			r_mem = kzalloc(sizeof(char) * (3), GFP_KERNEL);

			if (touch_i2c_read(ts->client, REPORT_WAKEUP_GESTURE_ONLY_REG, (3), r_mem) < 0) {
				TOUCH_ERR_MSG("%d bytes read fail!", (3));
			} else {
				if(report_lpwg){
					*(r_mem+2) = 0;
					if (touch_i2c_write(ts->client, REPORT_WAKEUP_GESTURE_ONLY_REG, (3), r_mem) < 0){
						TOUCH_ERR_MSG("REPORT_WAKEUP_GESTURE_ONLY_REG write fail");
					}
				}
			}

			if (r_mem != NULL)
				kfree(r_mem);

			hrtimer_try_to_cancel(&ts->multi_tap_timer);
			if (!hrtimer_callback_running(&ts->multi_tap_timer))
				hrtimer_start(&ts->multi_tap_timer, ktime_set(0, MS_TO_NS(200)), HRTIMER_MODE_REL);
		}
		return 0;
	}
#endif

#ifdef CUST_G2_TOUCH
	if ( ts->ts_data.interrupt_status_reg == 0 ) {
		TOUCH_ERR_MSG("Ignore interrupt. interrupt status reg = 0x%x\n", ts->ts_data.interrupt_status_reg);
		goto ignore_interrupt;
	}
#endif

	/* Finger */
	if (likely(ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_ABS0)) {
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
		if(s7020){
			if (unlikely(touch_i2c_read(client, FINGER_DATA_REG_START,
				sizeof(ts->ts_data.finger.finger_status_reg),
				ts->ts_data.finger.finger_status_reg) < 0)) {
					TOUCH_ERR_MSG("FINGER_STATE_REG read fail\n");
					goto err_synaptics_getdata;
			}

			touch_finger_bit_mask = GET_BIT_MASK_S7020(ts->ts_data.finger.finger_status_reg);

			if (unlikely(touch_debug_mask & DEBUG_GET_DATA)) {
			TOUCH_INFO_MSG("Finger_status : 0x%x, 0x%x, 0x%x\n", ts->ts_data.finger.finger_status_reg[0],
					ts->ts_data.finger.finger_status_reg[1], ts->ts_data.finger.finger_status_reg[2]);
			TOUCH_INFO_MSG("Touch_bit_mask: 0x%x\n", touch_finger_bit_mask);
			}	

			while(touch_finger_bit_mask) {
			GET_INDEX_FROM_MASK(finger_index, touch_finger_bit_mask, MAX_NUM_OF_FINGERS);
			if (unlikely(touch_i2c_read(ts->client,
					(FINGER_DATA_REG_START+3) + (5 * finger_index),5,ts->ts_data.finger.finger_reg[finger_index]) < 0)) {
				TOUCH_ERR_MSG("FINGER_DATA_REG read fail\n");
				goto err_synaptics_getdata;
			}
			
			data->curr_data[finger_index].id = finger_index;
			data->curr_data[finger_index].x_position =
					TS_SNTS_GET_X_POSITION_S7020(ts->ts_data.finger.finger_reg[finger_index][0], ts->ts_data.finger.finger_reg[finger_index][2]);
			data->curr_data[finger_index].y_position =
					TS_SNTS_GET_Y_POSITION_S7020(ts->ts_data.finger.finger_reg[finger_index][1], ts->ts_data.finger.finger_reg[finger_index][2]);
			data->curr_data[finger_index].width_major = TS_SNTS_GET_WIDTH_MAJOR_S7020(ts->ts_data.finger.finger_reg[finger_index][3]);
			data->curr_data[finger_index].width_minor = TS_SNTS_GET_WIDTH_MINOR_S7020(ts->ts_data.finger.finger_reg[finger_index][3]);
			data->curr_data[finger_index].width_orientation = TS_SNTS_GET_ORIENTATION_S7020(ts->ts_data.finger.finger_reg[finger_index][3]);
			data->curr_data[finger_index].pressure = TS_SNTS_GET_PRESSURE(ts->ts_data.finger.finger_reg[finger_index][4]);
			data->curr_data[finger_index].status = FINGER_PRESSED;

			if (unlikely(touch_debug_mask & DEBUG_GET_DATA))
				TOUCH_INFO_MSG("<%d> pos(%4d,%4d) w_m[%2d] w_n[%2d] w_o[%2d] p[%2d]\n",
								finger_index, data->curr_data[finger_index].x_position, data->curr_data[finger_index].y_position,
								data->curr_data[finger_index].width_major, data->curr_data[finger_index].width_minor,
								data->curr_data[finger_index].width_orientation, data->curr_data[finger_index].pressure);

			index++;
		}
		}else{
#endif
			if (unlikely(touch_i2c_read(ts->client,
					FINGER_DATA_REG_START,
					(NUM_OF_EACH_FINGER_DATA_REG * MAX_NUM_OF_FINGERS),
					ts->ts_data.finger.finger_reg[finger_index]) < 0)) {
				TOUCH_ERR_MSG("FINGER_DATA_REG read fail\n");
				goto err_synaptics_getdata;
			}
			touch_finger_bit_mask = GET_BIT_MASK(ts->ts_data.finger.finger_reg);
			
			if (unlikely(touch_debug_mask & DEBUG_GET_DATA)) {
				TOUCH_INFO_MSG("Object Type & Status : %x, %x, %x, %x, %x, %x, %x, %x, %x, %x\n",
					    ts->ts_data.finger.finger_reg[0][0], ts->ts_data.finger.finger_reg[1][0],
						ts->ts_data.finger.finger_reg[2][0], ts->ts_data.finger.finger_reg[3][0],
						ts->ts_data.finger.finger_reg[4][0], ts->ts_data.finger.finger_reg[5][0],
						ts->ts_data.finger.finger_reg[6][0], ts->ts_data.finger.finger_reg[7][0],
						ts->ts_data.finger.finger_reg[8][0], ts->ts_data.finger.finger_reg[9][0]);
				TOUCH_INFO_MSG("Touch_bit_mask: 0x%x\n", touch_finger_bit_mask);
			}
	
		while (touch_finger_bit_mask) {
			GET_INDEX_FROM_MASK(finger_index, touch_finger_bit_mask, MAX_NUM_OF_FINGERS)

			data->curr_data[finger_index].id = finger_index;
#ifdef CUST_G2_TOUCH
			data->curr_data[finger_index].object =
				ts->ts_data.finger.finger_reg[finger_index][REG_OBJECT_TYPE_AND_STATUS];
			if(data->curr_data[finger_index].width_major >= PALM_DETECT_SIZE)
				data->curr_data[finger_index].object = PALM;
			if(data->curr_data[finger_index].object == PALM) palm_detected = 1;
#endif
				data->curr_data[finger_index].x_position =
					TS_SNTS_GET_X_POSITION(ts->ts_data.finger.finger_reg[finger_index][REG_X_MSB],
										   ts->ts_data.finger.finger_reg[finger_index][REG_X_LSB]);
				data->curr_data[finger_index].y_position =
					TS_SNTS_GET_Y_POSITION(ts->ts_data.finger.finger_reg[finger_index][REG_Y_MSB],
										   ts->ts_data.finger.finger_reg[finger_index][REG_Y_LSB]);
				data->curr_data[finger_index].width_major = TS_SNTS_GET_WIDTH_MAJOR(ts->ts_data.finger.finger_reg[finger_index][REG_WX],ts->ts_data.finger.finger_reg[finger_index][REG_WY]);
				data->curr_data[finger_index].width_minor = TS_SNTS_GET_WIDTH_MINOR(ts->ts_data.finger.finger_reg[finger_index][REG_WX],ts->ts_data.finger.finger_reg[finger_index][REG_WY]);
				data->curr_data[finger_index].width_orientation = TS_SNTS_GET_ORIENTATION(ts->ts_data.finger.finger_reg[finger_index][REG_WY],
													ts->ts_data.finger.finger_reg[finger_index][REG_WX]);
			
			data->curr_data[finger_index].pressure = TS_SNTS_GET_PRESSURE(ts->ts_data.finger.finger_reg[finger_index][REG_Z]);
			data->curr_data[finger_index].status = FINGER_PRESSED;
#ifdef CUST_G2_TOUCH
			z_sum += data->curr_data[finger_index].pressure;
#endif
			if (unlikely(touch_debug_mask & DEBUG_GET_DATA))
				TOUCH_INFO_MSG("<%d> pos(%4d,%4d) w_m[%2d] w_n[%2d] w_o[%2d] p[%2d]\n",
								finger_index, data->curr_data[finger_index].x_position, data->curr_data[finger_index].y_position,
								data->curr_data[finger_index].width_major, data->curr_data[finger_index].width_minor,
								data->curr_data[finger_index].width_orientation, data->curr_data[finger_index].pressure);

			index++;

			}
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
		}
#endif
		data->total_num = index;

#ifdef CUST_G2_TOUCH
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
		if(!s7020){
			/*S7020 doesn't support Palm Rejection just in this case.*/
#endif
			if(z_sum > 500 && palm_detected){
				if(data->large_palm_status == NO_PALM || data->large_palm_status == PALM_RELEASED) {
					data->large_palm_status = PALM_PRESSED;
				} else {
					data->large_palm_status = PALM_MOVE;
				}
			} else {
				if(data->large_palm_status == PALM_PRESSED || data->large_palm_status == PALM_MOVE) {
					data->large_palm_status = PALM_RELEASED;
				} else {
					data->large_palm_status = NO_PALM;
				}
			}
			data->palm = palm_detected;
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
		}
	if(!(s7020)){
		/*S7020 doesn't support TA solutions.*/
#endif
		if( (ts_charger_plug == 1 && (data->prev_total_num != data->total_num)) ||
			(touch_debug_mask & DEBUG_NOISE) ) {
			
			if (unlikely(synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x08, 1, &cns) < 0)) {
				TOUCH_ERR_MSG("Current Noise State REG read fail\n");
				goto err_synaptics_getdata;
			}

			if (unlikely(synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x04, 1, &udata[0]) < 0)) {
				TOUCH_ERR_MSG("Interference Metric REG read fail\n");
				goto err_synaptics_getdata;
			}

			if (unlikely(synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x05, 1, &udata[1]) < 0)) {
				TOUCH_ERR_MSG("Interference Metric REG read fail\n");
				goto err_synaptics_getdata;
			}
			im = (udata[1]<<8)|udata[0];

			if (unlikely(synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x09, 2, &udata[0]) < 0)) {
				TOUCH_ERR_MSG("CID IM REG read fail\n");
				goto err_synaptics_getdata;
			}
			cid_im = (udata[1]<<8)|udata[0];

			if (unlikely(synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x0a, 2, &udata[0]) < 0)) {
				TOUCH_ERR_MSG("Freq Scan IM REG read fail\n");
				goto err_synaptics_getdata;
			}
			feq_scan_im = (udata[1]<<8)|udata[0];
		}
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
	}
#endif
#endif

		if (unlikely(touch_debug_mask & DEBUG_GET_DATA))
			TOUCH_INFO_MSG("Total_num: %d\n", data->total_num);
	}

#if defined(CONFIG_MACH_MSM8974_VU3_KR) || defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
	 /* Button */
	if (unlikely(ts->button_fc.dsc.id != 0)) {
		if (likely(ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_BUTTON)) {
			if (unlikely(synaptics_ts_page_data_read(client, BUTTON_PAGE, BUTTON_DATA_REG,
					sizeof(ts->ts_data.button_data_reg), &ts->ts_data.button_data_reg) < 0)) {
				TOUCH_ERR_MSG("BUTTON_DATA_REG read fail\n");
				goto err_synaptics_getdata;
			}

			if (unlikely(touch_debug_mask & DEBUG_BUTTON))
				TOUCH_DEBUG_MSG("Button register: 0x%x\n", ts->ts_data.button_data_reg);

			if (ts->ts_data.button_data_reg) {
				/* pressed - find first one */
				for (cnt = 0; cnt < ts->pdata->caps->number_of_button; cnt++)
				{
					if ((ts->ts_data.button_data_reg >> cnt) & 0x1) {
						ts->ts_data.button.key_code = ts->pdata->caps->button_name[cnt];
						data->curr_button.key_code = ts->ts_data.button.key_code;
						data->curr_button.state = 1;
						break;
					}
				}
			}else {
				/* release */
				data->curr_button.key_code = ts->ts_data.button.key_code;
				data->curr_button.state = 0;
			}
		}
	}
#endif
	return 0;

err_synaptics_device_damage:
err_synaptics_getdata:
	return -EIO;
#ifdef CUST_G2_TOUCH
ignore_interrupt:
	return -IGNORE_INTERRUPT;
#endif
}

static int read_page_description_table(struct i2c_client *client)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);

	struct function_descriptor buffer;
	unsigned short u_address = 0;
	unsigned short page_num = 0;

	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");
	memset(&buffer, 0x0, sizeof(struct function_descriptor));
	memset(&ts->common_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->finger_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->sensor_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->analog_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->flash_fc, 0x0, sizeof(struct ts_ic_function));

#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
	if(s7020){
		for (page_num = 0; page_num < 4; page_num++) {
			if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, page_num) < 0)) {
				TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
				return -EIO;
			}

			
			for (u_address = DESCRIPTION_TABLE_START; u_address > 10; u_address -= sizeof(struct function_descriptor)) {
						if (unlikely(touch_i2c_read(client, u_address, sizeof(buffer), (unsigned char *)&buffer) < 0)) {
							TOUCH_ERR_MSG("RMI4 Function Descriptor read fail\n");
							return -EIO;
						}

						if (buffer.id == 0)
							break;
						
						switch (buffer.id) {
						case RMI_DEVICE_CONTROL:
							ts->common_fc.dsc = buffer;
							ts->common_fc.function_page = page_num;
							break;
						case 0x11: /*TOUCHPAD_SENSORS*/
							ts->finger_fc.dsc = buffer;
							ts->finger_fc.function_page = page_num;
							break;
#if defined(CONFIG_MACH_MSM8974_VU3_KR) || defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
						case CAPACITIVE_BUTTON_SENSORS:
							ts->button_fc.dsc = buffer;
							ts->button_fc.function_page = page_num;
							break;
#endif
						case ANALOG_CONTROL:
							ts->analog_fc.dsc = buffer;
							ts->analog_fc.function_page = page_num;
							break;
						case FLASH_MEMORY_MANAGEMENT:
							ts->flash_fc.dsc = buffer;
							ts->flash_fc.function_page = page_num;
						default:
							break;
						}
					}
				}
	}else{
#endif
		for (page_num = 0; page_num < PAGE_MAX_NUM; page_num++) {
			if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, page_num) < 0)) {
				TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
				return -EIO;
			}

			
			for (u_address = DESCRIPTION_TABLE_START; u_address > 10; u_address -= sizeof(struct function_descriptor)) {
						if (unlikely(touch_i2c_read(client, u_address, sizeof(buffer), (unsigned char *)&buffer) < 0)) {
							TOUCH_ERR_MSG("RMI4 Function Descriptor read fail\n");
							return -EIO;
						}
			
						if (buffer.id == 0)
							break;
			
						switch (buffer.id) {
						case RMI_DEVICE_CONTROL:
							ts->common_fc.dsc = buffer;
							ts->common_fc.function_page = page_num;
							break;
						case TOUCHPAD_SENSORS:
							ts->finger_fc.dsc = buffer;
							ts->finger_fc.function_page = page_num;
							break;
						case SENSOR_CONTROL:
							ts->sensor_fc.dsc = buffer;
							ts->sensor_fc.function_page = page_num;
							break;
#if defined(CONFIG_MACH_MSM8974_VU3_KR) || defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
						case CAPACITIVE_BUTTON_SENSORS:
							ts->button_fc.dsc = buffer;
							ts->button_fc.function_page = page_num;
							break;
#endif
						case ANALOG_CONTROL:
							ts->analog_fc.dsc = buffer;
							ts->analog_fc.function_page = page_num;
							break;
						case FLASH_MEMORY_MANAGEMENT:
							ts->flash_fc.dsc = buffer;
							ts->flash_fc.function_page = page_num;
						default:
							break;
						}
					}
				}
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
		}
#endif	
		
	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x00) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
	if (touch_debug_mask & DEBUG_BASE_INFO)
		TOUCH_INFO_MSG("common[%dP:0x%02x] finger[%dP:0x%02x] sensor[%dP:0x%02x] analog[%dP:0x%02x] flash[%dP:0x%02x] button[%dP:0x%02x]\n",
				ts->common_fc.function_page, ts->common_fc.dsc.id,
				ts->finger_fc.function_page, ts->finger_fc.dsc.id,
				ts->sensor_fc.function_page, ts->sensor_fc.dsc.id,
				ts->analog_fc.function_page, ts->analog_fc.dsc.id,
				ts->flash_fc.function_page, ts->flash_fc.dsc.id,
				ts->button_fc.function_page, ts->button_fc.dsc.id);
#else
	if (touch_debug_mask & DEBUG_BASE_INFO)
		TOUCH_INFO_MSG("common[%dP:0x%02x] finger[%dP:0x%02x] sensor[%dP:0x%02x] analog[%dP:0x%02x] flash[%dP:0x%02x]\n",
				ts->common_fc.function_page, ts->common_fc.dsc.id,
				ts->finger_fc.function_page, ts->finger_fc.dsc.id,
				ts->sensor_fc.function_page, ts->sensor_fc.dsc.id,
				ts->analog_fc.function_page, ts->analog_fc.dsc.id,
				ts->flash_fc.function_page, ts->flash_fc.dsc.id);
#endif

	if (ts->common_fc.dsc.id == 0 || ts->finger_fc.dsc.id == 0
			|| ts->analog_fc.dsc.id == 0 || ts->flash_fc.dsc.id == 0){
		TOUCH_ERR_MSG("common/finger/analog/flash are not initiailized\n");
		return -EPERM;
	}

	return 0;
}

#ifdef CUST_G2_TOUCH
int synaptics_ts_power(struct i2c_client *client, int power_ctrl);
int get_bootloader_fw_ver(struct synaptics_ts_data* ts, struct touch_fw_info* fw_info)
{
	u8 buf[3] = {0};
	u16 bootloader_id = 0;

	if(unlikely(touch_i2c_read(ts->client, ts->flash_fc.dsc.query_base, 2, buf) < 0)) {
		TOUCH_ERR_MSG("Enter Bootloader Mode Step 1 Fail\n");
		return -EIO;
	} else {
		bootloader_id = buf[0] | (buf[1] << 8);

		buf[0] = bootloader_id % 0x100;
		buf[1] = bootloader_id / 0x100;

		if(unlikely(touch_i2c_write(ts->client, ts->flash_fc.dsc.data_base + 1, 2, &buf[0]) < 0)) {
			TOUCH_ERR_MSG("Enter Bootloader Mode Step 2 Fail\n");
			return -EIO;
		}
		if(unlikely(touch_i2c_write_byte(ts->client, ts->flash_fc.dsc.data_base + 2, 0x0F) < 0)) {
			TOUCH_ERR_MSG("Enter Bootloader Mode Step 3 Fail\n");
			return -EIO;
		}

		msleep(1000);

		read_page_description_table(ts->client);

		if(unlikely(touch_i2c_read(ts->client, ts->common_fc.dsc.query_base + 18, 3, buf) < 0)) {
			TOUCH_ERR_MSG("Read Bootloader Firmware Version Fail\n");
			return -EIO;
		}

		synaptics_ts_power(ts->client, POWER_OFF);
		synaptics_ts_power(ts->client, POWER_ON);
		msleep(ts->pdata->role->booting_delay);

		return (buf[2] << 16 | buf[1] << 8 | buf[0]);
	}
}

int get_ic_chip_rev(struct synaptics_ts_data* ts, struct touch_fw_info* fw_info)
{
	read_page_description_table(ts->client);

	if (unlikely(touch_i2c_read(ts->client, FW_REVISION_REG,
			sizeof(ts->fw_info.fw_rev), &ts->fw_info.fw_rev) < 0)) {
		TOUCH_ERR_MSG("FW_REVISION_REG read fail\n");
		return -1;
	}
	TOUCH_INFO_MSG("FW_REVISION_REG = %d\n", ts->fw_info.fw_rev);
	if(ts->fw_info.fw_rev == 1) {
		return TOUCH_CHIP_REV_B;
	}
#if defined(CONFIG_MACH_MSM8974_G2_KR)
	if(lge_get_board_revno() >= HW_REV_E) {
		return TOUCH_CHIP_REV_B;
	}
#elif defined(CONFIG_MACH_MSM8974_G2_VZW) || defined(CONFIG_MACH_MSM8974_G2_ATT)
	if(lge_get_board_revno() >= HW_REV_D) {
		return TOUCH_CHIP_REV_B;
	}
#elif defined(CONFIG_MACH_MSM8974_G2_DCM) || defined(CONFIG_MACH_MSM8974_G2_SPR) || defined(CONFIG_MACH_MSM8974_G2_TMO_US) || defined(CONFIG_MACH_MSM8974_G2_OPEN_COM) || defined(CONFIG_MACH_MSM8974_G2_VDF_COM) || defined(CONFIG_MACH_MSM8974_G2_CA)
	if(lge_get_board_revno() >= HW_REV_C) {
		return TOUCH_CHIP_REV_B;
	}
#elif defined(CONFIG_MACH_MSM8974_VU3_KR)
	return TOUCH_CHIP_REV_B;
#elif defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
	return TOUCH_CHIP_REV_B;
#endif
	fw_info->fw_setting.bootloader_fw_ver = get_bootloader_fw_ver(ts, fw_info);
	TOUCH_INFO_MSG("Bootloader Firmware Version = %d\n", fw_info->fw_setting.bootloader_fw_ver);

	switch(fw_info->fw_setting.bootloader_fw_ver)
	{
		case -EIO:
			return TOUCH_CHIP_UNKNOWN_REV;
		case OLD_S3404A_BOOT_ID:
		case S3404A_BOOT_ID:
			return TOUCH_CHIP_REV_A;
		case S3404B_BOOT_ID:
			return TOUCH_CHIP_REV_B;
		default:
			return TOUCH_CHIP_REV_B;
	}
}

int set_fw_info(struct synaptics_ts_data* ts, struct touch_fw_info* fw_info)
{
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
	if(s7020)
		return 0;
#endif
	fw_info->fw_setting.ic_chip_rev = get_ic_chip_rev(ts, fw_info);

	memset(&SynaFirmware,0x00,sizeof(SynaFirmware));

#if defined(CONFIG_LGE_Z_TOUCHSCREEN)

	switch(fw_info->fw_setting.ic_chip_rev) {
		case TOUCH_CHIP_REV_A:
			memcpy(&SynaFirmware,&SynaFirmware_a,sizeof(SynaFirmware));
			break;
		case TOUCH_CHIP_REV_B:
			memcpy(&SynaFirmware,&SynaFirmware_b,sizeof(SynaFirmware));
			break;
		default:
			memcpy(&SynaFirmware,&SynaFirmware_b,sizeof(SynaFirmware));
			break;
	}
#elif defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
	memcpy(&SynaFirmware,&SynaFirmware_PLG259,sizeof(SynaFirmware));
#else
	if(fw_info->fw_setting.curr_touch_vendor == TOUCH_VENDOR_LGIT) {
		switch(fw_info->fw_setting.ic_chip_rev) {
			case TOUCH_CHIP_REV_A:
				memcpy(&SynaFirmware,&SynaFirmware_a,sizeof(SynaFirmware));
				break;
			case TOUCH_CHIP_REV_B:
				memcpy(&SynaFirmware,&SynaFirmware_b,sizeof(SynaFirmware));
				break;
			default:
				memcpy(&SynaFirmware,&SynaFirmware_b,sizeof(SynaFirmware));
				break;
		}
#if defined(A1_only)
	} else if(fw_info->fw_setting.curr_touch_vendor == TOUCH_VENDOR_TPK) {
		memcpy(&SynaFirmware,&SynaFirmware_TPK,sizeof(SynaFirmware));
#endif
	} else {
		TOUCH_ERR_MSG("No Match\n");
		return -1;
	}
#endif

	return 0;
}
#endif

int get_ic_info(struct synaptics_ts_data* ts, struct touch_fw_info* fw_info)
{
#if defined(ARRAYED_TOUCH_FW_BIN)
	int cnt;
#endif

	u8 device_status = 0;
#ifdef CUST_G2_TOUCH
	u8 flash_status = 0;
#else
	u8 flash_control = 0;
#endif

	if(unlikely(set_fw_info(ts, fw_info) < 0)) {
		return -1;
	}

	read_page_description_table(ts->client);

	memset(&ts->fw_info, 0, sizeof(struct synaptics_ts_fw_info));

	if (unlikely(touch_i2c_read(ts->client, FW_REVISION_REG,
			sizeof(ts->fw_info.fw_rev), &ts->fw_info.fw_rev) < 0)) {
		TOUCH_ERR_MSG("FW_REVISION_REG read fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_read(ts->client, MANUFACTURER_ID_REG,
			sizeof(ts->fw_info.manufacturer_id), &ts->fw_info.manufacturer_id) < 0)) {
		TOUCH_ERR_MSG("MANUFACTURER_ID_REG read fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_read(ts->client, PRODUCT_ID_REG,
			sizeof(ts->fw_info.product_id) - 1, ts->fw_info.product_id) < 0)) {
		TOUCH_ERR_MSG("PRODUCT_ID_REG read fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_read(ts->client, FLASH_CONFIG_ID_REG,
			sizeof(ts->fw_info.config_id) - 1, ts->fw_info.config_id) < 0)) {
		TOUCH_ERR_MSG("FLASH_CONFIG_ID_REG read fail\n");
		return -EIO;
	}

	snprintf(fw_info->ic_fw_identifier, sizeof(fw_info->ic_fw_identifier),
			"%s - %d", ts->fw_info.product_id, ts->fw_info.manufacturer_id);
	snprintf(fw_info->ic_fw_version, sizeof(fw_info->ic_fw_version),
			"%s", ts->fw_info.config_id);
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
	if(s7020)
		return 0;
#endif	
#if defined(ARRAYED_TOUCH_FW_BIN)
	for (cnt = 0; cnt < sizeof(SynaFirmware)/sizeof(SynaFirmware[0]); cnt++) {
		strncpy(ts->fw_info.fw_image_product_id, &SynaFirmware[cnt][16], 10);
		if (!(strncmp(ts->fw_info.product_id , ts->fw_info.fw_image_product_id, 10)))
			break;
	}
	strncpy(ts->fw_info.image_config_id, &SynaFirmware[cnt][0xef00],4);
	ts->fw_info.fw_start = (unsigned char *)&SynaFirmware[cnt][0];
	ts->fw_info.fw_size = sizeof(SynaFirmware[0]);
#else
	strncpy(ts->fw_info.fw_image_product_id, &SynaFirmware[16], 10);
	strncpy(ts->fw_info.image_config_id, &SynaFirmware[0xef00], 4);
#ifdef CUST_G2_TOUCH
	strncpy(fw_info->syna_img_fw_version, &SynaFirmware[0xef00], 4);
	strncpy(fw_info->syna_img_fw_product_id, &SynaFirmware[0x0040], 6);
#endif
	ts->fw_info.fw_start = (unsigned char *)&SynaFirmware[0];
	ts->fw_info.fw_size = sizeof(SynaFirmware);
#endif

	ts->fw_info.fw_image_rev = ts->fw_info.fw_start[31];

#ifdef CUST_G2_TOUCH
	if (unlikely(touch_i2c_read(ts->client, FLASH_STATUS_REG, sizeof(flash_status), &flash_status) < 0)) {
		TOUCH_ERR_MSG("FLASH_STATUS_REG read fail\n");
		return -EIO;
	}
#else
	if (unlikely(touch_i2c_read(ts->client, FLASH_CONTROL_REG, sizeof(flash_control), &flash_control) < 0)) {
		TOUCH_ERR_MSG("FLASH_CONTROL_REG read fail\n");
		return -EIO;
	}
#endif

	if (unlikely(touch_i2c_read(ts->client, DEVICE_STATUS_REG, sizeof(device_status), &device_status) < 0)) {
		TOUCH_ERR_MSG("DEVICE_STATUS_REG read fail\n");
		return -EIO;
	}

	/* Firmware has a problem, so we should firmware-upgrade */
	if (device_status & DEVICE_STATUS_FLASH_PROG
			|| (device_status & DEVICE_CRC_ERROR_MASK) != 0
#ifdef CUST_G2_TOUCH
			|| (flash_status & FLASH_STATUS_MASK) != 0) {
#else
			|| (flash_control & FLASH_STATUS_MASK) != 0) {
#endif
		TOUCH_ERR_MSG("Firmware has a unknown-problem, so it needs firmware-upgrade.\n");
#ifdef CUST_G2_TOUCH
		TOUCH_ERR_MSG("FLASH_STATUS[%x] DEVICE_STATUS_REG[%x]\n", (u32)flash_status, (u32)device_status);
#else
		TOUCH_ERR_MSG("FLASH_CONTROL[%x] DEVICE_STATUS_REG[%x]\n", (u32)flash_control, (u32)device_status);
#endif
		TOUCH_ERR_MSG("FW-upgrade Force Rework.\n");

		/* firmware version info change by force for rework */
		ts->fw_info.fw_rev = 0;
		snprintf(ts->fw_info.config_id, sizeof(ts->fw_info.config_id), "ERR");
#ifdef CUST_G2_TOUCH
		fw_info->fw_force_rework = true;
#endif
	}

	return 0;
}

#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
/* Old Version Touch IC resolution is different from the latest SW */
int get_ic_fw_version(struct synaptics_ts_data* ts)
{
	//read_page_description_table(ts->client);

	if (unlikely(touch_i2c_read(ts->client, FLASH_CONFIG_ID_REG,
					sizeof(ts->fw_info.config_id) - 1, ts->fw_info.config_id) < 0)) {
		TOUCH_ERR_MSG("FLASH_CONFIG_ID_REG read fail\n");
		return -EIO;
	}

	return (int)simple_strtoul(&ts->fw_info.config_id[1], NULL, 10);
}
#endif

#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
static int lpwg_tap_control(struct synaptics_ts_data *ts, int on);
#endif
int synaptics_ts_init(struct i2c_client* client, struct touch_fw_info* fw_info)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);

	u8 buf = 0;
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
	u8 xy_coordinate[4] = {0x6F, 0x08, 0xFF, 0x0E};//2159*3839 = 86F * EFF
#endif

	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
	if(fw_info->S7020){
		s7020 = true;
		TOUCH_INFO_MSG("(2)GK BOARD. S7020 IC.\n");
	}
#endif	
	if (!ts->is_probed) {
		if (unlikely(get_ic_info(ts, fw_info) < 0))
			return -EIO;
	}
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
	if(s7020){
		/*no sleep mode enable*/
		TOUCH_INFO_MSG("Enable no sleep Mode\n");
		if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
				DEVICE_CONTROL_NOSLEEP | DEVICE_CONTROL_CONFIGURED) < 0)) {
			TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail\n");
			return -EIO;
		}
	}else{
		/*doze mode enable & TA Connected select*/
		if(ts_charger_plug==0){
			if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
					DEVICE_CONTROL_NORMAL_OP | DEVICE_CONTROL_CONFIGURED) < 0)) {
				TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail(1)\n");
				return -EIO;
			}
		}else if(ts_charger_plug==1){
			if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
					DEVICE_CONTROL_NORMAL_OP | DEVICE_CONTROL_CONFIGURED | DEVICE_CHARGER_CONNECTED) < 0)) {
				TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail(1)\n");
				return -EIO;
			}
		}

		/*doze holdoff setting*/
		if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG +4,
						1 ) < 0)) {	// 1 = 100ms
					TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail(2)\n");
					return -EIO;
		}

		/*doze interval setting*/
		if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG +2,
						1 ) < 0)) {	// 1 = 10ms
					TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail(3)\n");
					return -EIO;
		}

		/*doze wakeup threshold setting*/
		if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG +3,
						10 ) < 0)) {	// more than 10
					TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail(4)\n");
					return -EIO;
		}
	}
#else
	if(ts->pdata->role->ghost_detection_enable) {
		if(ts_charger_plug==0){
			if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
					DEVICE_CONTROL_NOSLEEP | DEVICE_CONTROL_CONFIGURED) < 0)) {
				TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail\n");
				return -EIO;
			}
		} else if(ts_charger_plug==1){
			if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
					DEVICE_CONTROL_NOSLEEP | DEVICE_CONTROL_CONFIGURED | DEVICE_CHARGER_CONNECTED) < 0)) {
				TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail\n");
				return -EIO;
			}
		}
	} else {
		if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
				DEVICE_CONTROL_NOSLEEP | DEVICE_CONTROL_CONFIGURED) < 0)) {
			TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail\n");
			return -EIO;
		}
	}
#endif

	if (unlikely(touch_i2c_read(client, INTERRUPT_ENABLE_REG,
			1, &buf) < 0)) {
		TOUCH_ERR_MSG("INTERRUPT_ENABLE_REG read fail\n");
		return -EIO;
	}
	if (unlikely(touch_i2c_write_byte(client, INTERRUPT_ENABLE_REG,
			buf | INTERRUPT_MASK_ABS0) < 0)) {
		TOUCH_ERR_MSG("INTERRUPT_ENABLE_REG write fail\n");
		return -EIO;
	}


	if (unlikely(touch_i2c_read(client, INTERRUPT_STATUS_REG, 1, &buf) < 0)) {
		TOUCH_ERR_MSG("INTERRUPT_STATUS_REG read fail\n");
		return -EIO;	// it is critical problem because interrupt will not occur.
	}
/*
	if (unlikely(touch_i2c_read(client, OBJECT_TYPE_AND_STATUS_REG, sizeof(ts->ts_data.finger.finger_reg),
			ts->ts_data.finger.finger_reg) < 0)) {
		TOUCH_ERR_MSG("FINGER_STATE_REG read fail\n");
		return -EIO;	// it is critical problem because interrupt will not occur on some FW.
	}
*/
#ifdef BUTTON_THRESHOLD_CONTROL
	if(!s7020){
		if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x02) < 0)) {
			TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
			return -EIO;
		}
		if (unlikely(touch_i2c_read(client, 0x0c, 1, &buf) < 0)) {
			TOUCH_ERR_MSG("back Button Threshold read fail\n");
		}

		if (unlikely(touch_i2c_read(client, 0x0d, 1, &buf) < 0)) {
			TOUCH_ERR_MSG("menu Button Threshold read fail\n");
		}
		//TOUCH_INFO_MSG("fw) Button sensitivity: 0x%x\n", buf); //if menu and back button sensitivity is same.(E019: 0x75)

		if (unlikely(touch_i2c_write_byte(client, 0x0c, 0x72) < 0)) {
			TOUCH_ERR_MSG("back Button Threshold read fail\n");
		}
		if (unlikely(touch_i2c_write_byte(client, 0x0d, 0x72) < 0)) {
			TOUCH_ERR_MSG("menu Button Threshold read fail\n");
		}

		if (unlikely(touch_i2c_read(client, 0x0c, 1, &buf) < 0)) {
			TOUCH_ERR_MSG("back Button Threshold read fail\n");
		}

		if (unlikely(touch_i2c_read(client, 0x0d, 1, &buf) < 0)) {
			TOUCH_ERR_MSG("menu Button Threshold read fail\n");
		}
		//TOUCH_INFO_MSG("force_setting) Button sensitivity : 0x%x\n", buf); //0x72

		if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x00) < 0)) {
			TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
			return -EIO;
		}
	}
#endif

#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
	/* Old Version Touch IC resolution is different from the latest SW */
#if defined(CONFIG_LGE_SECURITY_KNOCK_ON)
	TOUCH_INFO_MSG("Password enable : %d\n", ts->password_enable);
	if(ts->password_enable)
#else
	if (get_ic_fw_version(ts) < 6)
#endif
	{
		TOUCH_INFO_MSG("Change Resolution to 2159*3839\n");
		if (touch_i2c_write(client, MAXIMUM_XY_COORDINATE_REG, 4, xy_coordinate) < 0){
			TOUCH_ERR_MSG("MAXIMUM XY COORDINATE write fail!!\n");
		}
	}
#endif
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR) //OMEGA_TOUCH_THERMAL_CHECK
	synaptics_change_fw_on_init(client);
#endif
	ts->is_probed = 1;
#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
	DO_SAFE(lpwg_tap_control(ts, 0), error);
	ts->double_tap_enable = 0;
	ts->password_enable = 0;

	//TOUCH_DEBUG_MSG("synaptics_ts_suspend lpwg_mode : 0x%x, 0x%x, 0x%x\n", ts->lpwg_mode, ts->double_tap_enable, ts->password_enable );

#endif
	return 0;
#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
error:
	return -EIO;
#endif
}

int synaptics_ts_power(struct i2c_client *client, int power_ctrl)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);

	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	switch (power_ctrl) {
	case POWER_OFF:
#ifdef CUST_G2_TOUCH
		if ((ts->pdata->int_pin > 0) && (ts->pdata->reset_pin > 0))  {
			gpio_tlmm_config(GPIO_CFG(ts->pdata->int_pin, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
			gpio_set_value(ts->pdata->reset_pin, 0);
		}
#endif

#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
		cancel_work_sync(&ts->multi_tap_work);
		hrtimer_cancel(&ts->multi_tap_timer);
#endif
		if (ts->pdata->pwr->use_regulator) {
			regulator_disable(ts->regulator_vio);
			regulator_disable(ts->regulator_vdd);
		}
		else
			ts->pdata->pwr->power(client, 0);
		break;
	case POWER_ON:
		if (ts->pdata->reset_pin > 0) {
			gpio_set_value(ts->pdata->reset_pin, 1);
		}

		if (ts->pdata->pwr->use_regulator) {
			regulator_enable(ts->regulator_vdd);
			regulator_enable(ts->regulator_vio);
		}
		else
			ts->pdata->pwr->power(client, 1);

		if (ts->pdata->reset_pin > 0) {
			gpio_set_value(ts->pdata->reset_pin, 0);
			msleep(ts->pdata->role->reset_delay);
			gpio_set_value(ts->pdata->reset_pin, 1);
		}
		break;
	case POWER_SLEEP:
#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
		cancel_work_sync(&ts->multi_tap_work);
		hrtimer_cancel(&ts->multi_tap_timer);
#endif
		if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
				DEVICE_CONTROL_SLEEP | DEVICE_CONTROL_CONFIGURED) < 0)) {
			TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail\n");
			return -EIO;
		}
		break;
	case POWER_WAKE:
		if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
				DEVICE_CONTROL_NORMAL_OP | DEVICE_CONTROL_CONFIGURED) < 0)) {
			TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail\n");
			return -EIO;
		}
		break;
	default:
		return -EIO;
		break;
	}

	return 0;
}

//argument changed as the client doesn't have valid pdata any more...
//pdata'll be given from DTS.
int synaptics_ts_probe(struct lge_touch_data* lge_touch_ts)//int synaptics_ts_probe(struct i2c_client* client)
{
	struct synaptics_ts_data *ts;
	int ret = 0;
	struct i2c_client* client =	lge_touch_ts->client;

	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	ts = kzalloc(sizeof(struct synaptics_ts_data), GFP_KERNEL);
	if (!ts) {
		TOUCH_ERR_MSG("Can not allocate memory\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	set_touch_handle(client, ts);

	ts->client = client;
	ts->pdata = lge_touch_ts->pdata;

	if (ts->pdata->pwr->use_regulator) {
		ts->regulator_vdd = regulator_get(&client->dev, ts->pdata->pwr->vdd);
		if (IS_ERR(ts->regulator_vdd)) {
			TOUCH_ERR_MSG("FAIL: regulator_get_vdd - %s\n", ts->pdata->pwr->vdd);
			ret = -EPERM;
			goto err_get_vdd_failed;
	}

		ts->regulator_vio = regulator_get(&client->dev, ts->pdata->pwr->vio);
		if (IS_ERR(ts->regulator_vio)) {
			TOUCH_ERR_MSG("FAIL: regulator_get_vio - %s\n", ts->pdata->pwr->vio);
			ret = -EPERM;
			goto err_get_vio_failed;
		}

		if (ts->pdata->pwr->vdd_voltage > 0) {
			ret = regulator_set_voltage(ts->regulator_vdd, ts->pdata->pwr->vdd_voltage, ts->pdata->pwr->vdd_voltage);
			if (ret < 0)
				TOUCH_ERR_MSG("FAIL: VDD voltage setting - (%duV)\n", ts->pdata->pwr->vdd_voltage);
	}
		/* // G2 : vio is not adjustable regulator
		if (ts->pdata->pwr->vio_voltage > 0) {
			ret = regulator_set_voltage(ts->regulator_vio, ts->pdata->pwr->vio_voltage, ts->pdata->pwr->vio_voltage);
			if (ret < 0)
				TOUCH_ERR_MSG("FAIL: VIO voltage setting - (%duV)\n",ts->pdata->pwr->vio_voltage);
		}
		*/
	}

#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
	touch_multi_tap_wq = create_singlethread_workqueue("touch_multi_tap_wq");
	if(touch_multi_tap_wq) {
		ts->lge_touch_ts = lge_touch_ts;
		INIT_WORK(&ts->multi_tap_work, touch_multi_tap_work);
		hrtimer_init(&ts->multi_tap_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->multi_tap_timer.function = touch_multi_tap_timer_handler;
	}
#endif
	return ret;

err_get_vio_failed:
	if (ts->pdata->pwr->use_regulator) {
		regulator_put(ts->regulator_vio);
	}
err_get_vdd_failed:
	if (ts->pdata->pwr->use_regulator) {
		regulator_put(ts->regulator_vdd);
	}
err_alloc_data_failed:
	kfree(ts);
	return ret;
}

#ifdef RESOLUTION_FROM_FW
int synaptics_ts_resolution(struct i2c_client* client) {
	struct synaptics_ts_data* ts =
			(struct synaptics_ts_data*)get_touch_handle(client);

	u8 resolution[4] = {0};

	if(ts->pdata->role->key_type == TOUCH_HARD_KEY) {
		if (unlikely(touch_i2c_read(ts->client, MAXIMUM_XY_COORDINATE_REG, sizeof(resolution), resolution) < 0)) {
			TOUCH_ERR_MSG("MAXIMUM_XY_COORDINATE_REG read fail\n");
			return -EIO;	// it is critical problem because interrupt will not occur.
		}
		ts->pdata->caps->x_max = (int)(resolution[1] << 8 | resolution[0]);
		ts->pdata->caps->y_max = (int)(resolution[3] << 8 | resolution[2]);
	}
	return 0;

}
#endif

void synaptics_ts_remove(struct i2c_client* client)
{
	struct synaptics_ts_data* ts =
			(struct synaptics_ts_data*)get_touch_handle(client);

	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	if (ts->pdata->pwr->use_regulator) {
		regulator_put(ts->regulator_vio);
		regulator_put(ts->regulator_vdd);
	}

	kfree(ts);
}

int synaptics_ts_fw_upgrade(struct i2c_client* client, struct touch_fw_info* fw_info)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data*)get_touch_handle(client);
	int ret = 0;

	ts->is_probed = 0;

#if defined(A1_only)
	if(fw_info->fw_setting.prev_touch_vendor != fw_info->fw_setting.curr_touch_vendor) {
		TOUCH_INFO_MSG("Panel changed prev_touch_vendor=%d curr_touch_vendor=%d\n", fw_info->fw_setting.prev_touch_vendor, fw_info->fw_setting.curr_touch_vendor);
		if(unlikely(set_fw_info(ts, fw_info) < 0)) {
			return -1;
		}
	} else {
		TOUCH_INFO_MSG("Panel Not changed\n");
	}
#endif
	ret = FirmwareUpgrade(ts, fw_info->fw_upgrade.fw_path);

	if (ts->fw_info.fw_reflash_twice){
		TOUCH_INFO_MSG("Touch IC Bootloader is old, So Reflash again....\n");
		ret = FirmwareUpgrade(ts, fw_info->fw_upgrade.fw_path);
		}
	/* update IC info */
	if (ret >= 0)
		get_ic_info(ts, fw_info);

	return ret;
	}

#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
err_t synaptics_ts_lpwg(struct i2c_client* client, u32 code, u32 value, struct point *data);
#endif
int synaptics_ts_ic_ctrl(struct i2c_client *client, u8 code, u16 value)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);
	u8 buf = 0;
	u8 buf2[2] = {0};
#if defined(CONFIG_LGE_SECURITY_KNOCK_ON)
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
	u8 xy_coordinate[4] = {0x38, 0x04, 0x80, 0x07};//1080*1920 = 438 * 780
#endif
#endif

	switch (code)
	{
	case IC_CTRL_BASELINE:
		switch (value)
		{
		case BASELINE_OPEN:
			break;
		case BASELINE_FIX:
			break;
		case BASELINE_REBASE:
			/* rebase base line */
			if (likely(ts->finger_fc.dsc.id != 0)) {
				if (unlikely(synaptics_ts_page_data_write_byte(client, ANALOG_PAGE, ANALOG_COMMAND_REG, 0x02) < 0)) {
					TOUCH_ERR_MSG("finger baseline reset command write fail\n");
					return -EIO;
				}
			}
			break;
		default:
			break;
		}
		break;
	case IC_CTRL_READ:
		if (touch_i2c_read(client, value, 1, &buf) < 0) {
			TOUCH_ERR_MSG("IC register read fail\n");
			return -EIO;
		}
		break;
	case IC_CTRL_WRITE:
		if (touch_i2c_write_byte(client, ((value & 0xFF00) >> 8), (value & 0xFF)) < 0) {
			TOUCH_ERR_MSG("IC register write fail\n");
			return -EIO;
		}
		break;
	case IC_CTRL_RESET_CMD:
		if (unlikely(touch_i2c_write_byte(client, DEVICE_COMMAND_REG, 0x1) < 0)) {
			TOUCH_ERR_MSG("IC Reset command write fail\n");
			return -EIO;
		}
		break;

#ifdef CUST_G2_TOUCH
	case IC_CTRL_REPORT_MODE:

		switch (value)
		{
			case 0:   // continuous mode
			    buf2[0] = buf2[1] = 0;
			    if (unlikely(touch_i2c_write(client, MOTION_SUPPRESSION, 2, buf2) < 0)) {
				TOUCH_ERR_MSG("MOTION_SUPPRESSION write fail\n");
				return -EIO;
			    }
			    break;

			case 1:  // reduced mode
			    buf2[0] = buf2[1] = 2;
			    if (unlikely(touch_i2c_write(client, MOTION_SUPPRESSION, 2, buf2) < 0)) {
				TOUCH_ERR_MSG("MOTION_SUPPRESSION write fail\n");
				return -EIO;
			    }
			    break;

			default:
				break;
		}
		break;
#endif
#ifdef CUST_G2_TOUCH_WAKEUP_GESTURE
	case IC_CTRL_DOUBLE_TAP_WAKEUP_MODE:
				switch (value)
				{
					unsigned char *r_mem = NULL;

					case 0: /* touch double-tap disable */
					{
					TOUCH_INFO_MSG("ic_ctrl: IC_CTRL_DOUBLE_TAP_WAKEUP_MODE = 0\n");
						r_mem = kzalloc(sizeof(char) * (6), GFP_KERNEL);

						if (touch_i2c_read(ts->client, REPORT_WAKEUP_GESTURE_ONLY_REG,(3), r_mem) < 0) {
							TOUCH_ERR_MSG("%d bytes read fail!", (3));
						} else {
							if(*(r_mem+2) !=0x0) {
								*(r_mem+2) = 0;
								if(touch_i2c_write(ts->client, REPORT_WAKEUP_GESTURE_ONLY_REG,(3), r_mem) < 0)
									TOUCH_ERR_MSG("REPORT_WAKEUP_GESTURE_ONLY_REG write fail");
							}
						}
						if(touch_i2c_write_byte(ts->client, DOZE_INTERVAL_REG, 1) < 0) {
							TOUCH_ERR_MSG("DOZE_INTERVAL_REG write fail");
							if(r_mem != NULL) kfree(r_mem);
							return 0;
						}
						if(touch_i2c_write_byte(ts->client, 0x10, 10) < 0) {	//DOZE_WAKEUP_TRESHOLD_REG=0x10
							TOUCH_ERR_MSG("DOZE_WAKEUP_TRESHOLD_REG write fail");
							if(r_mem != NULL) kfree(r_mem);
							return 0;
						}
						if(r_mem != NULL) kfree(r_mem);
					}
					break;

					case 1: /* touch double-tap enable */
					{
					TOUCH_INFO_MSG("ic_ctrl: IC_CTRL_DOUBLE_TAP_WAKEUP_MODE = 1\n");

					r_mem = kzalloc(sizeof(char) * (10), GFP_KERNEL);
						*(r_mem+0) = 0x1;
						*(r_mem+1) = 0x3C;	//0x14 wake up report rate 40(0x28) -> 60(0x3C)
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR) /* Film+Palm+KnockOn */
						*(r_mem+2) = 0xA;
						*(r_mem+3) = 0xA; //max active duration 3sec(0x6) -> 5sec(0xA)
#else
						*(r_mem+2) = 0x3;
						*(r_mem+3) = 0x6;
#endif
						*(r_mem+4) = 0x2;
						*(r_mem+5) = 0x2;

						if(touch_i2c_write(ts->client, WAKEUP_GESTURE_ENABEL_REG,(6), r_mem) < 0) {
							TOUCH_ERR_MSG("WAKEUP_GESTURE_ENABEL_REG write fail");
							if(r_mem != NULL) kfree(r_mem);
							return -EIO;
						}

#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR) /* Film+Palm+KnockOn */
						if (touch_i2c_read(ts->client, MINIMUM_PEAK_AMPLITUDE_REG,(3), r_mem) < 0) {
							TOUCH_ERR_MSG("%d bytes read fail!", (3));
						} else {
							*(r_mem+2) = 0x50;
							if(touch_i2c_write(ts->client, MINIMUM_PEAK_AMPLITUDE_REG,(3), r_mem) < 0) {
								TOUCH_ERR_MSG("Peak_merge_threshold write fail");
								if(r_mem != NULL) kfree(r_mem);
								return 0;
							}
						}
#endif

						if(touch_i2c_write_byte(ts->client, DOZE_INTERVAL_REG, 5) < 0) {
							TOUCH_ERR_MSG("DOZE_INTERVAL_REG write fail");
							if(r_mem != NULL) kfree(r_mem);
							return -EIO;
						}
						
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR) /* Film+Palm+KnockOn */
						if(touch_i2c_write_byte(ts->client, 0x10, 20) < 0) {
							TOUCH_ERR_MSG("DOZE_WAKEUP_TRESHOLD_REG write fail");
							if(r_mem != NULL) kfree(r_mem);
							return -EIO;
						}
#else
						if(touch_i2c_write_byte(ts->client, 0x10, 30) < 0) {
							TOUCH_ERR_MSG("DOZE_WAKEUP_TRESHOLD_REG write fail");
							if(r_mem != NULL) kfree(r_mem);
							return -EIO;
						}
#endif

						if (touch_i2c_read(ts->client, REPORT_WAKEUP_GESTURE_ONLY_REG,(3), r_mem) < 0) {
							TOUCH_ERR_MSG("%d bytes read fail!", (3));
						} else {
							*(r_mem+2) = 0x2;
							if(touch_i2c_write(ts->client, REPORT_WAKEUP_GESTURE_ONLY_REG,(3), r_mem) < 0)
								TOUCH_ERR_MSG("REPORT_WAKEUP_GESTURE_ONLY_REG write fail");
						}
						//set double-tap area (omega resolution : 2159*3839 / size : 6.8cm(x-axis))
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
						if (quick_cover_status) {
							/* if smart cover is closed, resize knock-on area */
							*(r_mem+0) = 0x58;	/* Double-Tap X0 LSB */
							*(r_mem+1) = 0x02;	/* Double-Tap X0 MSB 600=0x258 */
							*(r_mem+2) = 0x64;	/* Double-Tap Y0 LSB */
							*(r_mem+3) = 0x0;	/* Double-Tap Y0 MSB 100=0x64 */
							*(r_mem+4) = 0x40;	/* Double-Tap X1 LSB */
							*(r_mem+5) = 0x06;	/* Double-Tap X1 MSB 1600=0x640 */
							*(r_mem+6) = 0x78;	/* Double-Tap Y1 LSB */
							*(r_mem+7) = 0x05;	/* Double-Tap Y1 MSB 1400=0x578 */
						} else {
#endif
							*(r_mem+0) = 0x2C;	/* Double-Tap X0 LSB */
							*(r_mem+1) = 0x01;	/* Double-Tap X0 MSB 300=0x12C */
							*(r_mem+2) = 0x2C;	/* Double-Tap Y0 LSB */
							*(r_mem+3) = 0x01;	/* Double-Tap Y0 MSB 300=0x12C */
							*(r_mem+4) = 0x43;	/* Double-Tap X1 LSB */
							*(r_mem+5) = 0x07;	/* Double-Tap X1 MSB 2159-300=1859=0x743 */
							*(r_mem+6) = 0xD3;	/* Double-Tap Y1 LSB */
							*(r_mem+7) = 0x0D;	/* Double-Tap Y1 MSB 3839-300=3539=0xDD3 */
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
						}
#endif
						*(r_mem+8) = 60;	/* Maximum Tap Time */
						*(r_mem+9) = 50;	/* Maximum Tap Distance */

					if (touch_i2c_write(ts->client, DOUBLE_TAP_AREA_REG, (10), r_mem) < 0)
							TOUCH_ERR_MSG("DOUBLE_TAP_AREA_REG write fail");
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
					if (touch_i2c_read(ts->client, SMALL_FINGER_AMPLITUDE_THRESHOLD_REG,(6), r_mem) < 0) {
						TOUCH_ERR_MSG("%d bytes read fail!", (6));
					} else {
						*(r_mem+0) = 0x32;	//finger amplitude threshold 0x32(50) -> normal mode (0x4D)
						if(touch_i2c_write(ts->client, SMALL_FINGER_AMPLITUDE_THRESHOLD_REG,(6), r_mem) < 0) {
							TOUCH_ERR_MSG("SMALL_FINGER_AMPLITUDE_THRESHOLD_REG write fail");
							if(r_mem != NULL) kfree(r_mem);
								return 0;
						} else {
							TOUCH_INFO_MSG("Small Finger Amplitude Threshold  = 0x%02x\n", *(r_mem+0));
						}
					}
#endif
#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
						if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, LPWG_CTRL_PAGE) < 0)) {
							TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
							return -EIO;
						}
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
						*(r_mem+0) = 0x64;	//50(5mm) -> 100 (10mm)
						if (touch_i2c_write(client, MULTITAP_PRESS_RELEASE_DISTANCE, 1, r_mem) < 0) {
							TOUCH_ERR_MSG("MultiTap Press Release Distance reg(0x04P:0x34) write fail\n");
							return -EIO;
						} else {
							TOUCH_INFO_MSG("MultiTap Press Release Distance = 0x%02x\n", *(r_mem+0));
						}
#endif
						if (touch_i2c_read(client, INTERTAP_DISTANCE_REG, 1, r_mem) < 0) {
							TOUCH_ERR_MSG("INTERTAP_DISTANCE_REG read fail!\n");
							return -EIO;
						} else {
							*(r_mem+0) = 0x0A;
							if(touch_i2c_write(client, INTERTAP_DISTANCE_REG, 1, r_mem) < 0) {
								TOUCH_ERR_MSG("Double TAP Maximum InterTap Distance reg(0x04P:0x4B) write fail\n");
								return -EIO;
							} else {
								TOUCH_INFO_MSG("Double TAP Maximum InterTap Distance = 0x%02x\n", *(r_mem+0));
							}
						}
						if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE) < 0)) {
							TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
							return -EIO;
						}
	/*
						if (touch_i2c_read(client, MOTION_SUPPRESSION,(3), buf3) < 0) {
							TOUCH_ERR_MSG("%d bytes read fail!", (3));
						} else {
							buf3[2]=buf3[2]|0x2;
							if(touch_i2c_write(client, MOTION_SUPPRESSION,(3), buf3) < 0)
								TOUCH_ERR_MSG("MOTION_SUPPRESSION write fail\n");
						}
	*/
						DO_SAFE(synaptics_ts_page_data_read(client, LPWG_CTRL_PAGE, MULTITAP_COUNT_REG, 1, &buf), error);
						TOUCH_INFO_MSG("double TAP COUNT %d\n", 2);  // for double tap mode
						buf = (buf & 0x07) | (2 << 3);
						TOUCH_INFO_MSG("MultiTap LPWG Control Reg value 0x%02X\n", buf);
						DO_SAFE(synaptics_ts_page_data_write_byte(client, LPWG_CTRL_PAGE, MULTITAP_COUNT_REG, buf), error);
						DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE), error);
						ts->double_tap_enable = 1;
#endif
						if(r_mem != NULL) kfree(r_mem);
					}
					break;
#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
					case 2: /* touch password tap enable */
					{
#if defined(KNOCKON_MASK)
						TOUCH_INFO_MSG("ic_ctrl: IC_CTRL_DOUBLE_TAP_WAKEUP_MODE = 2\n");
#else
						TOUCH_INFO_MSG("ic_ctrl: IC_CTRL_DOUBLE_TAP_WAKEUP_MODE = 2, password_tap_count=%d\n", ts->password_tap_count);
#endif
						r_mem = kzalloc(sizeof(char) * (10), GFP_KERNEL);
						*(r_mem+0) = 0x1;
						*(r_mem+1) = 0x3C;	//0x14 wake up report rate 40(0x28) -> 60(0x3C)
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR) /* Film+Palm+KnockOn */
						*(r_mem+2) = 0xA;
						*(r_mem+3) = 0xA; //max active duration 3sec(0x6) -> 5sec(0xA)
#else
						*(r_mem+2) = 0x3;
						*(r_mem+3) = 0x6;
#endif
						*(r_mem+4) = 0x2;
						*(r_mem+5) = 0x2;

						if(touch_i2c_write(ts->client, WAKEUP_GESTURE_ENABEL_REG,(6), r_mem) < 0) {
							TOUCH_ERR_MSG("WAKEUP_GESTURE_ENABEL_REG write fail");
							if(r_mem != NULL) kfree(r_mem);
							return -EIO;
						}

#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR) /* Film+Palm+KnockOn */
						if (touch_i2c_read(ts->client, MINIMUM_PEAK_AMPLITUDE_REG,(3), r_mem) < 0) {
							TOUCH_ERR_MSG("%d bytes read fail!", (3));
						} else {
							*(r_mem+2) = 0x50;
							if(touch_i2c_write(ts->client, MINIMUM_PEAK_AMPLITUDE_REG,(3), r_mem) < 0) {
								TOUCH_ERR_MSG("Peak_merge_threshold write fail");
								if(r_mem != NULL) kfree(r_mem);
									return 0;
								}
						}
#endif

						if (touch_i2c_write_byte(ts->client, DOZE_INTERVAL_REG, 5) < 0) {
							TOUCH_ERR_MSG("DOZE_INTERVAL_REG write fail");
						if (r_mem != NULL)
							kfree(r_mem);
							return -EIO;
						}
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR) /* Film+Palm+KnockOn */
						if(touch_i2c_write_byte(ts->client, 0x10, 20) < 0) {
							TOUCH_ERR_MSG("DOZE_WAKEUP_TRESHOLD_REG write fail");
							if(r_mem != NULL) kfree(r_mem);
							return -EIO;
						}
#else
						if(touch_i2c_write_byte(ts->client, 0x10, 30) < 0) {
							TOUCH_ERR_MSG("DOZE_WAKEUP_TRESHOLD_REG write fail");
							if(r_mem != NULL) kfree(r_mem);
							return -EIO;
						}
#endif

						if (touch_i2c_read(ts->client, REPORT_WAKEUP_GESTURE_ONLY_REG, (3), r_mem) < 0) {
							TOUCH_ERR_MSG("%d bytes read fail!", (3));
						} else {
							*(r_mem+2) = 0x2;
							if (touch_i2c_write(ts->client, REPORT_WAKEUP_GESTURE_ONLY_REG, (3), r_mem) < 0)
								TOUCH_ERR_MSG("REPORT_WAKEUP_GESTURE_ONLY_REG write fail");
						}

#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)	/* resolution 1080 * 1920 */
						/* Knock code Resolution : 1080 * 1920 */
						TOUCH_INFO_MSG("Change Resolution to 1080*1920\n");
						if (touch_i2c_write(client, MAXIMUM_XY_COORDINATE_REG, 4, xy_coordinate) < 0){
							TOUCH_ERR_MSG("MAXIMUM XY COORDINATE write fail!!\n");
						}

						if (touch_i2c_read(ts->client, SMALL_FINGER_AMPLITUDE_THRESHOLD_REG,(6), r_mem) < 0) {
							TOUCH_ERR_MSG("%d bytes read fail!", (6));
						} else {
							*(r_mem+0) = 0x32;	//finger amplitude threshold 0x32(50) -> normal mode (0x4D)
							if(touch_i2c_write(ts->client, SMALL_FINGER_AMPLITUDE_THRESHOLD_REG,(6), r_mem) < 0) {
								TOUCH_ERR_MSG("SMALL_FINGER_AMPLITUDE_THRESHOLD_REG write fail");
								if(r_mem != NULL) kfree(r_mem);
									return 0;
							} else {
								TOUCH_INFO_MSG("Small Finger Amplitude Threshold  = 0x%02x\n", *(r_mem+0));
							}
						}
						*(r_mem+0) = 0x32;	/* Double-Tap X0 LSB */
						*(r_mem+1) = 0x0;	/* Double-Tap X0 MSB 50=0x32 */
						*(r_mem+2) = 0x32;	/* Double-Tap Y0 LSB */
						*(r_mem+3) = 0x0;	/* Double-Tap Y0 MSB 50=0x32 */
						*(r_mem+4) = 0x06;	/* Double-Tap X1 LSB */
						*(r_mem+5) = 0x04;	/* Double-Tap X1 MSB 1080-50=1030=0x406 */
						*(r_mem+6) = 0x4E;	/* Double-Tap Y1 LSB */
						*(r_mem+7) = 0x07;	/* Double-Tap Y1 MSB 1920-50=1870=0x74E */
						*(r_mem+8) = 60;	/* maximum tap time */
						*(r_mem+9) = 50;	/* maximum tap distance */
#endif
						if (touch_i2c_write(ts->client, DOUBLE_TAP_AREA_REG, (10), r_mem) < 0)
							TOUCH_ERR_MSG("DOUBLE_TAP_AREA_REG write fail");
						if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, LPWG_CTRL_PAGE) < 0)) {
							TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
							return -EIO;
						}
						if (touch_i2c_read(client, MAX_INTERTAP_TIME_REG, 1, r_mem) < 0) {
							TOUCH_ERR_MSG("MAX_INTERTAP_TIME_REG read fail!\n");
							return -EIO;
						} else {
							*(r_mem+0) = 0x46;
							if (touch_i2c_write(client, MAX_INTERTAP_TIME_REG, 1, r_mem) < 0) {
								TOUCH_ERR_MSG("MultiTap Maximum InterTap Time reg(0x04P:0x33) write fail\n");
								return -EIO;
							} else {
								TOUCH_INFO_MSG("MultiTap Maximum InterTap Time = 0x%02x\n", *(r_mem+0));
							}
						}
#if defined(CONFIG_MACH_APQ8064_OMEGAR_KR) || defined(CONFIG_MACH_APQ8064_OMEGA_KR)
						*(r_mem+0) = 0x64;	//50(5mm) -> 100 (10mm)
						if (touch_i2c_write(client, MULTITAP_PRESS_RELEASE_DISTANCE, 1, r_mem) < 0) {
							TOUCH_ERR_MSG("MultiTap Press Release Distance reg(0x04P:0x34) write fail\n");
							return -EIO;
						} else {
							TOUCH_INFO_MSG("MultiTap Press Release Distance = 0x%02x\n", *(r_mem+0));
						}
#endif
						if (touch_i2c_read(client, INTERTAP_DISTANCE_REG, 1, r_mem) < 0) {
							TOUCH_ERR_MSG("INTERTAP_DISTANCE_REG read fail!\n");
							return -EIO;
						} else {
							*(r_mem+0) = 0xFF;
							if (touch_i2c_write(client, INTERTAP_DISTANCE_REG, 1, r_mem) < 0) {
								TOUCH_ERR_MSG("Multi TAP Maximum InterTap Distance reg(0x04P:0x35) write fail\n");
								return -EIO;
							} else {
								TOUCH_INFO_MSG("Multi TAP Maximum InterTap Distance = 0x%02x\n", *(r_mem+0));
							}
						}
						if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE) < 0)) {
							TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
							return -EIO;
						}
						if (r_mem != NULL)
							kfree(r_mem);
						synaptics_ts_lpwg(ts->client, LPWG_TAP_COUNT, ts->password_tap_count, NULL);
						ts->password_enable = 1;
					}
					break;

#endif
				}
				break;
#endif
	default:
		break;
}

	return buf;
#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
error:
	return -EIO;
#endif
}

#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
static int lpwg_tap_control(struct synaptics_ts_data *ts, int on)
{
	if(on){
		if(ts->lpwg_mode == LPWG_DOUBLE_TAP)
			synaptics_ts_ic_ctrl(ts->client, IC_CTRL_DOUBLE_TAP_WAKEUP_MODE, 1);
		else if(ts->lpwg_mode == LPWG_PASSWORD)
			synaptics_ts_ic_ctrl(ts->client, IC_CTRL_DOUBLE_TAP_WAKEUP_MODE, 2);
	} else {
		synaptics_ts_ic_ctrl(ts->client, IC_CTRL_DOUBLE_TAP_WAKEUP_MODE, 0);
	}

	return 0;
}

err_t synaptics_ts_suspend(struct i2c_client* client)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data*)get_touch_handle(client);
	multi_tap_fail_try = 0;

	switch (ts->lpwg_mode) {
		case LPWG_DOUBLE_TAP:
//		        DO_SAFE(lpwg_tap_control(ts, 1), error);
		        ts->double_tap_enable = 1;
			break;
		case LPWG_PASSWORD:
//		        DO_SAFE(lpwg_tap_control(ts, 1), error);
		        ts->password_enable = 1;
			break;
		default:
			break;
	}
	TOUCH_DEBUG_MSG("synaptics_ts_suspend lpwg_mode : 0x%x, 0x%x, 0x%x\n", ts->lpwg_mode, ts->double_tap_enable, ts->password_enable );

    return NO_ERROR;
//error:
    //return ERROR;
}

err_t synaptics_ts_resume(struct i2c_client* client)
{
//  struct synaptics_ts_data *ts = (struct synaptics_ts_data*)get_touch_handle(client);

    return NO_ERROR;
}

err_t synaptics_ts_lpwg(struct i2c_client* client, u32 code, u32 value, struct point *data)
{
    struct synaptics_ts_data *ts = (struct synaptics_ts_data*)get_touch_handle(client);
    u8 buf = 0;
	int i = 0;

    switch (code) {
    case LPWG_READ:
		if (ts->password_enable) {
			if( custom_gesture_status == 0) {
				data[0].x = 1;
				data[0].y = 1;
				data[1].x = -1;
				data[1].y = -1;
				TOUCH_DEBUG_MSG("TAP Position x:%d, y:%d\n", data[i].x, data[i].y);
				break;
			}
#if defined(KNOCKON_MASK)
			TOUCH_DEBUG_MSG("Read TAP Position\n");
#endif

			for(i=0;i< ts->password_tap_count;i++) {
				data[i].x = lpwg_data[4*i+1]<<8 | lpwg_data[4*i];
				data[i].y = lpwg_data[4*i+3]<<8 | lpwg_data[4*i+2];
#if !defined(KNOCKON_MASK)
				TOUCH_DEBUG_MSG("TAP Position x:%d, y:%d\n", data[i].x, data[i].y);
#endif
				// '-1' should be assinged to the last data.
				// Each data should be converted to LCD-resolution.
			}
			data[i].x = -1;
			data[i].y = -1;
		}
        break;
    case LPWG_ENABLE:
        ts->lpwg_mode = value;
		TOUCH_DEBUG_MSG("synaptics_ts_lpwg lpwg_mode : 0x%x, 0x%x, 0x%x", ts->lpwg_mode, ts->double_tap_enable, ts->password_enable );
        // The 'lpwg_mode' is changed to 'value' but it is applied in suspend-state.

        if (ts->double_tap_enable || ts->password_enable) {
            // IC should sleep when proximity sensor's value is 'NEAR'.
            if (value) {
                DO_SAFE(touch_i2c_read(client, DEVICE_CONTROL_REG, 1, &buf), error);
                buf = (buf & 0xFC) | DEVICE_CONTROL_NORMAL_OP;
                DO_SAFE(touch_i2c_write_byte(client, DEVICE_CONTROL_REG, buf), error);
                lpwg_tap_control(ts, 1);
            } else {
                DO_SAFE(touch_i2c_read(client, DEVICE_CONTROL_REG, 1, &buf), error);
                buf = (buf & 0xFC) | DEVICE_CONTROL_SLEEP;
                DO_SAFE(touch_i2c_write_byte(client, DEVICE_CONTROL_REG, buf), error);
            }
        }
        break;
    case LPWG_LCD_X:
    case LPWG_LCD_Y:
        // If touch-resolution is not same with LCD-resolution,
        // position-data should be converted to LCD-resolution.
        break;
    case LPWG_ACTIVE_AREA_X1:
    case LPWG_ACTIVE_AREA_X2:
    case LPWG_ACTIVE_AREA_Y1:
    case LPWG_ACTIVE_AREA_Y2:
        // Quick Cover Area
        break;
    case LPWG_TAP_COUNT:
		// Tap Count Control
		if (value) {
			DO_SAFE(synaptics_ts_page_data_read(client, LPWG_CTRL_PAGE, MULTITAP_COUNT_REG, 1, &buf), error);
#if !defined(KNOCKON_MASK)
			TOUCH_DEBUG_MSG("TAP COUNT %d \n", value);
#endif
			buf = (buf & 0x07) | (value << 3);
			TOUCH_DEBUG_MSG("MultiTap LPWG Control Reg value 0x%02X \n", buf);
			DO_SAFE(synaptics_ts_page_data_write_byte(client, LPWG_CTRL_PAGE, MULTITAP_COUNT_REG, buf), error);
			DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE), error);
		}
		ts->password_tap_count = value;
        break;
    case LPWG_REPLY:
        // Do something, if you need.
        TOUCH_INFO_MSG("[%s] LPWG_REPLY ???????????????\n", __func__);
		wait_reply = 0;
		if(value==0)
			multi_tap_fail_try++;
		else
			multi_tap_fail_try = 0;
		break;
    default:
        break;
    }
    return NO_ERROR;
error:
    return ERROR;
}
#endif

struct touch_device_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
#ifdef RESOLUTION_FROM_FW
	.resolution = synaptics_ts_resolution,
#endif
	.remove		= synaptics_ts_remove,
	.init  	= synaptics_ts_init,
	.data  	= synaptics_ts_get_data,
	.power 	= synaptics_ts_power,
	.fw_upgrade = synaptics_ts_fw_upgrade,
	.ic_ctrl	= synaptics_ts_ic_ctrl,
#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
	.suspend    = synaptics_ts_suspend,
	.resume     = synaptics_ts_resume,
	.lpwg       = synaptics_ts_lpwg,
#endif
};

static void async_touch_init(void *data, async_cookie_t cookie)
{
	touch_driver_register(&synaptics_ts_driver);
	return;
}


static int __devinit touch_init(void)
{
	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");
	
	async_schedule(async_touch_init, NULL);

	return 0;
}

static void __exit touch_exit(void)
{
	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");
#ifdef CONFIG_LGE_SECURITY_KNOCK_ON
	if (touch_multi_tap_wq)
		destroy_workqueue(touch_multi_tap_wq);
#endif
	touch_driver_unregister();
}

module_init(touch_init);
module_exit(touch_exit);

MODULE_AUTHOR("yehan.ahn@lge.com, hyesung.shin@lge.com");
MODULE_DESCRIPTION("LGE Touch Driver");
MODULE_LICENSE("GPL");

