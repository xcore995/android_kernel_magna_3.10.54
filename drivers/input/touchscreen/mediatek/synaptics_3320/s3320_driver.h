/*
 * Copyright (C) 2013 LG Electironics, Inc.
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



/****************************************************************************
* Macros
****************************************************************************/
#define TPD_TAG                  "[S3320] "
#define TPD_FUN(f)               printk(KERN_ERR TPD_TAG"%s\n", __FUNCTION__)
#define TPD_ERR(fmt, args...)    printk(KERN_ERR TPD_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define TPD_LOG(fmt, args...)    printk(KERN_ERR TPD_TAG fmt, ##args)

#define TS_SNTS_GET_X_POSITION(_msb_reg, _lsb_reg)	(((u16)((_msb_reg << 8) & 0xFF00) | (u16)((_lsb_reg) & 0xFF)))
#define TS_SNTS_GET_Y_POSITION(_msb_reg, _lsb_reg)	(((u16)((_msb_reg << 8) & 0xFF00) | (u16)((_lsb_reg) & 0xFF)))
#define TS_SNTS_GET_WIDTH_MAJOR(_width_x, _width_y)		((_width_x - _width_y) > 0) ? _width_x : _width_y
#define TS_SNTS_GET_WIDTH_MINOR(_width_x, _width_y)		((_width_x - _width_y) > 0) ? _width_y : _width_x
#define TS_SNTS_GET_ORIENTATION(_width_y, _width_x)		((_width_y - _width_x) > 0) ? 0 : 1
#define TS_SNTS_GET_PRESSURE(_pressure)		_pressure
#define MS_TO_NS(x)		(x * 1E6L)

#define DO_IF(do_work, goto_error) 								\
	do {												\
		if(do_work){										\
			TPD_ERR("[Touch E] Action Failed");	\
			goto goto_error;				\
		}							\
	} while (0)

#define DO_SAFE(do_work, goto_error) 				\
		DO_IF(unlikely((do_work) < 0), goto_error)


/****************************************************************************
* Type Definitions
****************************************************************************/
typedef struct {
	u8 query_base;
	u8 command_base;
	u8 control_base;
	u8 data_base;
	u8 interrupt_count;
	u8 function_exist;
} function_descriptor;

#define MAX_NUM_OF_FINGER				10
#define NUM_OF_EACH_FINGER_DATA_REG		8

typedef struct {
	unsigned char finger_data[MAX_NUM_OF_FINGER][NUM_OF_EACH_FINGER_DATA_REG];
} touch_sensor_data;

typedef struct {
	u16 pos_x[MAX_NUM_OF_FINGER];
	u16 pos_y[MAX_NUM_OF_FINGER];
	u8 pressure[MAX_NUM_OF_FINGER];
	u8 width_major[MAX_NUM_OF_FINGER];
	u8 width_minor[MAX_NUM_OF_FINGER];
	u8 orientation[MAX_NUM_OF_FINGER];
} touch_finger_info;

enum {
	LPWG_NONE = 0,
	LPWG_DOUBLE_TAP = (1U << 0),
	LPWG_PASSWORD = (1U << 1),
	LPWG_SIGNATURE = (1U << 2),
};

enum {
	LPWG_READ = 1,
	LPWG_ENABLE,
	LPWG_LCD_X,
	LPWG_LCD_Y,
	LPWG_ACTIVE_AREA_X1,
	LPWG_ACTIVE_AREA_X2,
	LPWG_ACTIVE_AREA_Y1,
	LPWG_ACTIVE_AREA_Y2,
	LPWG_TAP_COUNT,
	LPWG_LENGTH_BETWEEN_TAP,
	LPWG_DOUBLE_TAP_CHECK,
	LPWG_REPLY,
	LPWG_UPDATE_ALL,
};

struct point {
    int x;
    int y;
};

struct st_i2c_msgs {
	struct i2c_msg *msg;
	int count;
};

struct foo_obj {
	struct kobject kobj;
	int interrupt;
};

struct synaptics_ts_f12_info {
	bool ctrl_reg_is_present[32];
	bool data_reg_is_present[16];
	u8 ctrl_reg_addr[32];
	u8 data_reg_addr[16];
};

struct lpwg_control {
	u8 lpwg_mode;
	u8 screen;
	u8 sensor;
	u8 qcover;
	u8 double_tap_enable;
	u8 password_enable;
	u8 is_suspend;
};

struct synaptics_ts_f12_query_5 {
	union {
		struct {
			unsigned char size_of_query_6;

			struct {
				unsigned char ctrl_00_is_present:1;
				unsigned char ctrl_01_is_present:1;
				unsigned char ctrl_02_is_present:1;
				unsigned char ctrl_03_is_present:1;
				unsigned char ctrl_04_is_present:1;
				unsigned char ctrl_05_is_present:1;
				unsigned char ctrl_06_is_present:1;
				unsigned char ctrl_07_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl_08_is_present:1;
				unsigned char ctrl_09_is_present:1;
				unsigned char ctrl_10_is_present:1;
				unsigned char ctrl_11_is_present:1;
				unsigned char ctrl_12_is_present:1;
				unsigned char ctrl_13_is_present:1;
				unsigned char ctrl_14_is_present:1;
				unsigned char ctrl_15_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl_16_is_present:1;
				unsigned char ctrl_17_is_present:1;
				unsigned char ctrl_18_is_present:1;
				unsigned char ctrl_19_is_present:1;
				unsigned char ctrl_20_is_present:1;
				unsigned char ctrl_21_is_present:1;
				unsigned char ctrl_22_is_present:1;
				unsigned char ctrl_23_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl_24_is_present:1;
				unsigned char ctrl_25_is_present:1;
				unsigned char ctrl_26_is_present:1;
				unsigned char ctrl_27_is_present:1;
				unsigned char ctrl_28_is_present:1;
				unsigned char ctrl_29_is_present:1;
				unsigned char ctrl_30_is_present:1;
				unsigned char ctrl_31_is_present:1;
			} __packed;
		};

		unsigned char data[5];
	};
};

struct synaptics_ts_f12_query_8
{
	union {
		struct {
			unsigned char size_of_query_9;

			struct {
				unsigned char data_00_is_present:1;
				unsigned char data_01_is_present:1;
				unsigned char data_02_is_present:1;
				unsigned char data_03_is_present:1;
				unsigned char data_04_is_present:1;
				unsigned char data_05_is_present:1;
				unsigned char data_06_is_present:1;
				unsigned char data_07_is_present:1;
			} __packed;
			struct {
				unsigned char data_08_is_present:1;
				unsigned char data_09_is_present:1;
				unsigned char data_10_is_present:1;
				unsigned char data_11_is_present:1;
				unsigned char data_12_is_present:1;
				unsigned char data_13_is_present:1;
				unsigned char data_14_is_present:1;
				unsigned char data_15_is_present:1;
			} __packed;
		};

		unsigned char data[3];
	};
};
