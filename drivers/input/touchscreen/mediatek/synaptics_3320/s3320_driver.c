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
* Include Files
****************************************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/input/mt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>
#include <linux/proc_fs.h>
#include <linux/jiffies.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>
#include <linux/dma-mapping.h>

#include <mach/wd_api.h>
#include <mach/eint.h>
#include <mach/mt_wdt.h>
#include <mach/mt_gpt.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include <asm/uaccess.h>
#include <cust_eint.h>

#include "tpd.h"
#include "s3320_driver.h"

#define LGE_USE_SYNAPTICS_FW_UPGRADE
#if defined(LGE_USE_SYNAPTICS_FW_UPGRADE)
#include <linux/workqueue.h>
#if defined(TARGET_MT6582_Y90)
#include "s3320_fw.h"
#elif defined(TARGET_MT6732_C90)
#include "s3320_fw.h"
#elif defined(TARGET_MT6582_Y70)
#include "s3320_fw_y70.h"
#else
#include "s3320_fw.h"
#endif
#endif

#include "RefCode_F54.h"

//#include <mach/board_lge.h>
#include <linux/file.h>     //for file access
#include <linux/syscalls.h> //for file access
#include <linux/uaccess.h>  //for file access


/****************************************************************************
* Constants / Definitions
****************************************************************************/
#define LGE_TOUCH_NAME						"lge_touch"
#define	TPD_DEV_NAME						"S3320"
#define TPD_I2C_ADDRESS						0x20

#define BUFFER_SIZE							128

#define MAX_POINT_SIZE_FOR_LPWG				12

#define TOUCH_PRESSED						1
#define TOUCH_RELEASED						0
#define BUTTON_CANCLED						0xFF

#define PAGE_MAX_NUM						5
#define PAGE_SELECT_REG						0xFF
#define DESCRIPTION_TABLE_START				0xE9

#define RMI_DEVICE_CONTROL					0x01
#define TOUCHPAD_SENSORS					0x12
#define CAPACITIVE_BUTTON_SENSORS			0x1A
#define FLASH_MEMORY_MANAGEMENT				0x34
#define MULTI_TAP_GESTURE					0x51

/* Function $01 (RMI_DEVICE_CONTROL) */
#define PRODUCT_ID_REG						(device_control.query_base+11)

#define DEVICE_CONTROL_REG					(device_control.control_base)
#define DEVICE_CONTROL_NORMAL_OP			0x00	// sleep mode : go to doze mode after 500 ms
#define DEVICE_CONTROL_SLEEP 				0x01	// sleep mode : go to sleep
#define DEVICE_CONTROL_SLEEP_NO_RECAL		0x02	// sleep mode : go to sleep. no-recalibration
#define DEVICE_CONTROL_NOSLEEP				0x04
#define DEVICE_CHARGER_CONNECTED			0x20
#define DEVICE_CONTROL_CONFIGURED			0x80

#define INTERRUPT_ENABLE_REG				(device_control.control_base+1)
#define INTERRUPT_MASK_ABS0					0x04
#define BUTTON_MASK							0x10
#define INTERRUPT_MASK_LPWG					0x40

#define DEVICE_STATUS_REG					(device_control.data_base)
#define DEVICE_FAILURE_MASK					0x03
#define FW_CRC_FAILURE_MASK					0x04
#define FLASH_PROG_MASK						0x40
#define UNCONFIGURED_MASK					0x80

#define INTERRUPT_STATUS_REG				(device_control.data_base+1)

/* Function $12 (TOUCHPAD_SENSORS) */
#define FINGER_DATA_REG_START				(finger.data_base)
#define REG_OBJECT_TYPE_AND_STATUS			0
#define F12_NO_OBJECT_STATUS				(0x00)
#define F12_FINGER_STATUS					(0x01)
#define F12_STYLUS_STATUS					(0x02)
#define F12_PALM_STATUS						(0x03)
#define F12_HOVERING_FINGER_STATUS			(0x05)
#define F12_GLOVED_FINGER_STATUS			(0x06)
#define REG_X_LSB							1
#define REG_X_MSB							2
#define REG_Y_LSB							3
#define REG_Y_MSB							4
#define REG_Z								5
#define REG_WX								6
#define REG_WY								7

/* Function $1A (CAPACITIVE_BUTTON_SENSORS) */
#define BUTTON_DATA_REG						(button.data_base)

/* Function $34 (FLASH_MEMORY_MANAGEMENT) */
#define FLASH_CONFIG_ID_REG					(flash_memory.control_base)
#define FLASH_CONTROL_REG					(flash_memory.data_base+2)
#define FLASH_STATUS_REG					(flash_memory.data_base+3)
#define FLASH_STATUS_MASK					0xFF

/* Function $51 (MULTI_TAP_GESTURE) */
#define LPWG_PAGE							0x04

#define LPWG_STATUS_REG						0x00
#define LPWG_DATA_REG						0x01
#define LPWG_TAPCOUNT_REG					0x31
#define LPWG_MIN_INTERTAP_REG				0x32
#define LPWG_MAX_INTERTAP_REG				0x33
#define LPWG_TOUCH_SLOP_REG					0x34
#define LPWG_TAP_DISTANCE_REG				0x35
#define LPWG_INTERRUPT_DELAY_REG			0x37

#define LPWG_TAPCOUNT_REG2					0x38
#define LPWG_MIN_INTERTAP_REG2				0x39
#define LPWG_MAX_INTERTAP_REG2				0x3A
#define LPWG_TOUCH_SLOP_REG2				0x3B
#define LPWG_TAP_DISTANCE_REG2				0x3C
#define LPWG_INTERRUPT_DELAY_REG2			0x3E
#define WAKEUP_GESTURE_ENABLE_REG			0x20	/* f12_info.ctrl_reg_addr[27] */
#define MISC_HOST_CONTROL_REG				0x3F
#define THERMAL_HIGH_FINGER_AMPLITUDE		0x60	/* finger_amplitude(0x80) = 0.5 */

/* LPWG Control Value */
#define REPORT_MODE_CTRL					1
#define TCI_ENABLE_CTRL						2
#define TAP_COUNT_CTRL						3
#define MIN_INTERTAP_CTRL					4
#define MAX_INTERTAP_CTRL					5
#define TOUCH_SLOP_CTRL						6
#define TAP_DISTANCE_CTRL					7
#define INTERRUPT_DELAY_CTRL				8

#define TCI_ENABLE_CTRL2					22
#define TAP_COUNT_CTRL2						23
#define MIN_INTERTAP_CTRL2					24
#define MAX_INTERTAP_CTRL2					25
#define TOUCH_SLOP_CTRL2					26
#define TAP_DISTANCE_CTRL2					27
#define INTERRUPT_DELAY_CTRL2   			28

#define I2C_DELAY							50
#define UEVENT_DELAY						200
#define KNOCKON_DELAY						68	/* 700ms */

//#define TPD_HAVE_BUTTON

#ifdef TPD_HAVE_BUTTON
#ifdef LGE_USE_DOME_KEY
#define TPD_KEY_COUNT	2
static int tpd_keys_local[TPD_KEY_COUNT] = {KEY_BACK , KEY_MENU};
#else
#define TPD_KEY_COUNT	3
static int tpd_keys_local[TPD_KEY_COUNT] = {KEY_BACK, KEY_MENU, KEY_HOMEPAGE};
#endif
#endif


/****************************************************************************
* Variables
****************************************************************************/
static function_descriptor device_control;
static function_descriptor finger;
static function_descriptor button;
static function_descriptor flash_memory;
static function_descriptor multi_tap;

u8 device_control_page;
u8 finger_page;
u8 button_page;
u8 flash_memory_page;
u8 multi_tap_page;

char finger_state[MAX_NUM_OF_FINGER];

/* knock on/code */
char knock_on_type;
u8 double_tap_check = 0;
u8 lpwg_tap_count = 0;
static u8 lpwg_data_num = 0;
struct lpwg_control	lpwg_ctrl;
static u8 gesture_property[MAX_POINT_SIZE_FOR_LPWG*4] = {0};
static struct point lpwg_data[MAX_POINT_SIZE_FOR_LPWG];
static char *lpwg_uevent[2][2] = {
	{ "TOUCH_GESTURE_WAKEUP=WAKEUP", NULL },
	{ "TOUCH_GESTURE_WAKEUP=PASSWORD", NULL }
};
struct wake_lock knock_code_lock;
struct delayed_work work_timer;
struct delayed_work	lpwg_update_timer;
struct workqueue_struct* multi_tap_wq;
static struct foo_obj *foo_obj;
static struct synaptics_ts_f12_info f12_info;

/* FW update */
#if defined(LGE_USE_SYNAPTICS_FW_UPGRADE)
char fw_path[256];
unsigned char *fw_start;
unsigned long fw_size;
bool fw_force_update;
bool download_status;		/* avoid power off during F/W upgrade */
struct wake_lock fw_suspend_lock;
struct device *touch_fw_dev;
struct class *touch_class;
struct work_struct work_fw_upgrade;
struct workqueue_struct* touch_wq;
#endif

int f54_window_crack_check_mode = 0;
int f54_window_crack = 0;

bool key_lock_status;
//extern g_qem_check;

static u8* I2CDMABuf_va = NULL;
static u32 I2CDMABuf_pa = NULL;
struct st_i2c_msgs i2c_msgs;

extern struct tpd_device *tpd;
struct i2c_client *tpd_i2c_client = NULL;
struct i2c_client *ds4_i2c_client;
static int tpd_flag = 0;

static DEFINE_MUTEX(i2c_access);
static DEFINE_MUTEX(notify_access);
static DECLARE_WAIT_QUEUE_HEAD(tpd_waiter);


/****************************************************************************
* Extern Function Prototypes
****************************************************************************/
extern int FirmwareUpgrade ( struct i2c_client *client, const char* fw_path, unsigned long fw_size, unsigned char* fw_start );
extern int mtk_wdt_enable ( enum wk_wdt_en en );
extern void arch_reset(char mode, const char *cmd);


/****************************************************************************
* Local Function Prototypes
****************************************************************************/
static void touch_eint_interrupt_handler ( void );
void synaptics_initialize ( struct i2c_client *client );


/****************************************************************************
* Platform(AP) dependent functions
****************************************************************************/
static void synaptics_setup_eint ( void )
{
	TPD_FUN ();

	/* Configure GPIO settings for external interrupt pin  */
	mt_set_gpio_mode ( GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT );
	mt_set_gpio_dir ( GPIO_CTP_EINT_PIN, GPIO_DIR_IN );
	mt_set_gpio_pull_enable ( GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE );
	mt_set_gpio_pull_select ( GPIO_CTP_EINT_PIN, GPIO_PULL_UP );

	msleep ( 50 );

	/* Configure external interrupt settings for external interrupt pin */
	mt_eint_registration ( CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, touch_eint_interrupt_handler, 1 );

	/* unmask external interrupt */
	mt_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
}

void synaptics_power ( unsigned int on )
{
	TPD_FUN ();

	if ( on )
	{
#if defined(TARGET_MT6582_Y90)
		hwPowerOn ( MT6323_POWER_LDO_VGP2, VOL_3000, "TP" );
#elif defined(TARGET_MT6582_Y70)
		hwPowerOn ( MT6323_POWER_LDO_VGP1, VOL_3000, "TP" );
#elif defined(TARGET_MT6582_P1S3G)
		hwPowerOn ( MT6323_POWER_LDO_VGP1, VOL_3000, "TP" );
#endif
		msleep ( 100 );
	}
	else
	{
#if defined(TARGET_MT6582_Y90)
		hwPowerDown ( MT6323_POWER_LDO_VGP2, "TP" );
#elif defined(TARGET_MT6582_Y70)
		hwPowerDown ( MT6323_POWER_LDO_VGP1, "TP" );
#elif defined(TARGET_MT6582_P1S3G)
		hwPowerDown ( MT6323_POWER_LDO_VGP1, "TP" );
#endif
		msleep ( 10 );
	}
}


/****************************************************************************
* Synaptics I2C  Read / Write Funtions
****************************************************************************/
int i2c_dma_write ( struct i2c_client *client, const uint8_t *buf, int len )
{
	int i = 0;
	for ( i = 0 ; i < len ; i++ )
	{
		I2CDMABuf_va[i] = buf[i];
	}

	if ( len < 8 )
	{
		client->addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		return i2c_master_send(client, buf, len);
	}
	else
	{
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		return i2c_master_send(client, I2CDMABuf_pa, len);
	}
}

int i2c_dma_read ( struct i2c_client *client, uint8_t *buf, int len )
{
	int i = 0, ret = 0;
	if ( len < 8 )
	{
		client->addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		return i2c_master_recv(client, buf, len);
	}
	else
	{
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		ret = i2c_master_recv(client, I2CDMABuf_pa, len);
		if ( ret < 0 )
		{
			return ret;
		}

		for ( i = 0 ; i < len ; i++ )
		{
			buf[i] = I2CDMABuf_va[i];
		}
	}

	return ret;
}

static int i2c_msg_transfer ( struct i2c_client *client, struct i2c_msg *msgs, int count )
{
	int i = 0, ret = 0;
	for ( i = 0 ; i < count ; i++ )
	{
		if ( msgs[i].flags & I2C_M_RD )
		{
			ret = i2c_dma_read(client, msgs[i].buf, msgs[i].len);
		}
		else
		{
			ret = i2c_dma_write(client, msgs[i].buf, msgs[i].len);
		}

		if ( ret < 0 )
		{
			return ret;
		}
	}

	return 0;
}

int synaptics_ts_read_f54 ( struct i2c_client *client, u8 reg, int num, u8 *buf )
{
	int message_count = ( ( num - 1 ) / BUFFER_SIZE ) + 2;
	int message_rest_count = num % BUFFER_SIZE;
	int i, data_len;

	if ( i2c_msgs.msg == NULL || i2c_msgs.count < message_count )
	{
		if ( i2c_msgs.msg != NULL )
			kfree(i2c_msgs.msg);

		i2c_msgs.msg = (struct i2c_msg*)kcalloc(message_count, sizeof(struct i2c_msg), GFP_KERNEL);
		i2c_msgs.count = message_count;
		//dev_dbg(&client->dev, "%s: Update message count %d(%d)\n",  __func__, i2c_msgs.count, message_count);
	}

	i2c_msgs.msg[0].addr = client->addr;
	i2c_msgs.msg[0].flags = 0;
	i2c_msgs.msg[0].len = 1;
	i2c_msgs.msg[0].buf = &reg;

	if ( !message_rest_count )
		message_rest_count = BUFFER_SIZE;
	for ( i = 0 ; i < message_count - 1 ; i++ )
	{
		if ( i == message_count - 1 )
			data_len = message_rest_count;
		else
			data_len = BUFFER_SIZE;

		i2c_msgs.msg[i + 1].addr = client->addr;
		i2c_msgs.msg[i + 1].flags = I2C_M_RD;
		i2c_msgs.msg[i + 1].len = data_len;
		i2c_msgs.msg[i + 1].buf = buf + BUFFER_SIZE * i;
	}

#if 1
	if ( i2c_msg_transfer(client, i2c_msgs.msg, message_count) < 0 )
	{
#else
	if (i2c_transfer(client->adapter, i2c_msgs.msg, message_count) < 0) {
#endif
		if ( printk_ratelimit() )
			TPD_ERR ( "transfer error\n" );
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL ( synaptics_ts_read_f54 );

int synaptics_ts_read ( struct i2c_client *client, u8 reg, int size, u8 *buf )
{
	u8 retry = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = size,
			.buf = buf,
		},
	};

	while ( i2c_msg_transfer(client, msgs, 2) < 0 )
	{
		retry++;

		TPD_ERR ( "I2C 0x%X read retry\n", reg );

		if ( retry == 3 )
		{
			return -1;
		}
	}

	return 0;
}
EXPORT_SYMBOL ( synaptics_ts_read );

int synaptics_ts_write ( struct i2c_client *client, u8 reg, int size, u8 *buf )
{
	u8 retry = 0;

	unsigned char send_buf[size + 1];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags,
			.len = size+1,
			.buf = send_buf,
		},
	};

	send_buf[0] = (unsigned char) reg;
	memcpy(&send_buf[1], buf, size);

	while ( i2c_msg_transfer(client, msgs, 1) < 0 )
	{
		retry++;

		TPD_ERR ( "I2C 0x%X write retry\n", reg );

		if ( retry == 3 )
		{
			return -1;
		}
	}

	return 0;
}
EXPORT_SYMBOL ( synaptics_ts_write );

int synaptics_page_data_read ( struct i2c_client *client, u8 page, u8 reg, int size, u8 *data )
{
	int ret = 0;

	ret = synaptics_ts_write ( client, PAGE_SELECT_REG, 1, &page );
	if ( ret < 0 )
	{
		TPD_ERR ( "PAGE_SELECT_REG write fail\n" );
		return ret;
	}

	ret = synaptics_ts_read ( client, reg, size, data );
	if ( ret < 0 )
	{
		TPD_ERR ( "synaptics_page_data_read fail\n" );
		return ret;
	}

	ret = synaptics_ts_write ( client, PAGE_SELECT_REG, 1, &device_control_page );
	if ( ret < 0 )
	{
		TPD_ERR ( "PAGE_SELECT_REG write fail\n" );
		return ret;
	}

	return 0;
}

int synaptics_page_data_write ( struct i2c_client *client, u8 page, u8 reg, int size, u8 *data )
{
	int ret = 0;

	ret = synaptics_ts_write ( client, PAGE_SELECT_REG, 1, &page );
	if ( ret < 0 )
	{
		TPD_ERR ( "PAGE_SELECT_REG write fail\n" );
		return ret;
	}

	ret = synaptics_ts_write ( client, reg, size, data );
	if ( ret < 0 )
	{
		TPD_ERR ( "synaptics_page_data_write fail\n" );
		return ret;
	}

	ret = synaptics_ts_write ( client, PAGE_SELECT_REG, 1, &device_control_page );
	if ( ret < 0 )
	{
		TPD_ERR ( "PAGE_SELECT_REG write fail\n" );
		return ret;
	}

	return 0;
}


/****************************************************************************
* Touch malfunction Prevention Function
****************************************************************************/
static void synaptics_release_all_finger ( void )
{
	TPD_FUN ();
	u8 finger_count = 0;

	for ( finger_count = 0 ; finger_count < MAX_NUM_OF_FINGER ; finger_count++ )
	{
		input_mt_slot ( tpd->dev, finger_count );
		input_mt_report_slot_state ( tpd->dev, MT_TOOL_FINGER, false );
	}

	input_sync ( tpd->dev );
}

static void synaptics_ic_reset ( void )
{
	TPD_FUN ();

	mt_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );

	msleep ( 20 );
	mt_set_gpio_out ( GPIO_CTP_RST_PIN, GPIO_OUT_ZERO );
	msleep ( 10 );
	mt_set_gpio_out ( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
	msleep ( 100 );

	mt_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
}

static int synaptics_ic_status_check ( void )
{
	int ret;
	u8 device_status = 0;
	u8 temp = 0;

	/* read device status */
	ret = synaptics_ts_read ( tpd_i2c_client, DEVICE_STATUS_REG, 1, (u8 *) &device_status );
	if ( ret < 0 )
	{
		TPD_ERR ( "DEVICE_STATUS_REG read fail\n" );
		return -1;
	}

	/* ESD damage check */
	if ( ( device_status & DEVICE_FAILURE_MASK ) == DEVICE_FAILURE_MASK )
	{
		TPD_ERR ( "ESD damage occured. Reset Touch IC\n" );
		synaptics_ic_reset ();
		synaptics_initialize ( tpd_i2c_client );
		return -1;
	}

	/* Palm check */
	//TBD

	return 0;
}

void touch_keylock_enable ( int key_lock )
{
	TPD_FUN ();

	if ( !key_lock )
	{
		mt_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
		key_lock_status = 0;
	}
	else
	{
		mt_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );
		key_lock_status = 1;
	}
}
EXPORT_SYMBOL ( touch_keylock_enable );


/****************************************************************************
* Synaptics Knock_On Funtions
****************************************************************************/
static int sleep_control ( struct i2c_client *client, int mode, int recal )
{
	int ret = 0;
	u8 curr = 0;
	u8 next = 0;

	ret = synaptics_ts_read ( client, DEVICE_CONTROL_REG, 1, &curr );
	if ( ret < 0 )
	{
		TPD_ERR ( "DEVICE_CONTROL_REG read fail\n" );
		return -1;
	}

	next = ( curr & 0xFC ) | ( mode ? DEVICE_CONTROL_NORMAL_OP : DEVICE_CONTROL_SLEEP );

	TPD_LOG ( "sleep_control: curr = [%x] next[%x]\n", curr, next );

	ret = synaptics_ts_write ( client, DEVICE_CONTROL_REG, 1, &next );
	if ( ret < 0 )
	{
		TPD_ERR ( "DEVICE_CONTROL_REG write fail\n" );
		return -1;
	}

	return 0;
}

static int tci_control ( struct i2c_client *client, int type, u8 value )
{
	u8 buffer[3] = { 0 };
	u8 temp = 0;

	switch ( type )
	{
		case REPORT_MODE_CTRL:
			temp = device_control_page;
			DO_SAFE ( synaptics_ts_write ( client, PAGE_SELECT_REG, 1, &temp ), error );
			DO_SAFE ( synaptics_ts_read ( client, INTERRUPT_ENABLE_REG, 1, buffer ), error );
			temp = value ? buffer[0] & ~INTERRUPT_MASK_ABS0 : buffer[0] | INTERRUPT_MASK_ABS0;
			DO_SAFE ( synaptics_ts_write ( client, INTERRUPT_ENABLE_REG, 1, &temp ), error );

			if ( value )
			{
				buffer[0] = 0x29;
				buffer[1] = 0x29;
				DO_SAFE ( synaptics_ts_read ( client, f12_info.ctrl_reg_addr[15], 2, buffer ), error );
			}
			DO_SAFE ( synaptics_ts_read ( client, f12_info.ctrl_reg_addr[20], 3, buffer ), error );
			buffer[2] = ( buffer[2] & 0xfc ) | ( value ? 0x2 : 0x0 );
			DO_SAFE ( synaptics_ts_write ( client, f12_info.ctrl_reg_addr[20], 3, buffer ), error );

			TPD_LOG ( "report mode: %d", value );
			break;

		case TCI_ENABLE_CTRL:
			temp = LPWG_PAGE;
			DO_SAFE ( synaptics_ts_write ( client, PAGE_SELECT_REG, 1, &temp ), error );
			DO_SAFE ( synaptics_ts_read ( client, LPWG_TAPCOUNT_REG, 1, buffer ), error );
			buffer[0] = ( buffer[0] & 0xfe ) | ( value & 0x1 );
			DO_SAFE ( synaptics_ts_write ( client, LPWG_TAPCOUNT_REG, 1, buffer ), error );
			break;

		case TCI_ENABLE_CTRL2:
			DO_SAFE ( synaptics_ts_read ( client, LPWG_TAPCOUNT_REG2, 1, buffer ), error );
			buffer[0] = ( buffer[0] & 0xfe ) | ( value & 0x1 );
			DO_SAFE ( synaptics_ts_write ( client, LPWG_TAPCOUNT_REG2, 1, buffer ), error );
			break;

		case TAP_COUNT_CTRL:
			DO_SAFE ( synaptics_ts_read ( client, LPWG_TAPCOUNT_REG, 1, buffer ), error );
			buffer[0] = ( ( value << 3 ) & 0xf8 ) | ( buffer[0] & 0x7 );
			DO_SAFE ( synaptics_ts_write ( client, LPWG_TAPCOUNT_REG, 1, buffer ), error );
			break;

		case TAP_COUNT_CTRL2:
			DO_SAFE ( synaptics_ts_read ( client, LPWG_TAPCOUNT_REG2, 1, buffer ), error );
			buffer[0] = ( ( value << 3 ) & 0xf8 ) | ( buffer[0] & 0x7 );
			DO_SAFE ( synaptics_ts_write ( client, LPWG_TAPCOUNT_REG2, 1, buffer ), error );
			break;

		case MIN_INTERTAP_CTRL:
			DO_SAFE ( synaptics_ts_write ( client, LPWG_MIN_INTERTAP_REG, 1, &value ), error );
			break;

		case MIN_INTERTAP_CTRL2:
			DO_SAFE ( synaptics_ts_write ( client, LPWG_MIN_INTERTAP_REG2, 1, &value ), error );
			break;

		case MAX_INTERTAP_CTRL:
			DO_SAFE ( synaptics_ts_write ( client, LPWG_MAX_INTERTAP_REG, 1, &value ), error );
			break;

		case MAX_INTERTAP_CTRL2:
			DO_SAFE ( synaptics_ts_write ( client, LPWG_MAX_INTERTAP_REG2, 1, &value ), error );
			break;

		case TOUCH_SLOP_CTRL:
			DO_SAFE ( synaptics_ts_write ( client, LPWG_TOUCH_SLOP_REG, 1, &value ), error );
			break;

		case TOUCH_SLOP_CTRL2:
			DO_SAFE ( synaptics_ts_write ( client, LPWG_TOUCH_SLOP_REG2, 1, &value ), error );
			break;

		case TAP_DISTANCE_CTRL:
			DO_SAFE ( synaptics_ts_write ( client, LPWG_TAP_DISTANCE_REG, 1, &value ), error );
			break;

		case TAP_DISTANCE_CTRL2:
			DO_SAFE ( synaptics_ts_write ( client, LPWG_TAP_DISTANCE_REG2, 1, &value ), error );
			break;

		case INTERRUPT_DELAY_CTRL:
			temp = value ? ( ( KNOCKON_DELAY << 1 ) | 0x1 ) : 0;
			DO_SAFE ( synaptics_ts_write ( client, LPWG_INTERRUPT_DELAY_REG, 1, &temp ), error );
			break;

		case INTERRUPT_DELAY_CTRL2:
			temp = value ? ( ( KNOCKON_DELAY << 1 ) | 0x1 ) : 0;
			DO_SAFE ( synaptics_ts_write ( client, LPWG_INTERRUPT_DELAY_REG2, 1, &temp ), error );
			break;

		default:
			break;
	}

	return 0;

error:
	TPD_ERR ( "I2C error in tci_control\n" );
	return -1;
}

static int lpwg_control ( int mode )
{
	lpwg_ctrl.double_tap_enable = ( mode & ( LPWG_DOUBLE_TAP | LPWG_PASSWORD ) ) ? 1 : 0;
	lpwg_ctrl.password_enable = ( mode & LPWG_PASSWORD ) ? 1 : 0;

	switch ( mode )
	{
		case LPWG_DOUBLE_TAP:	// Only Double-Tap
			tci_control ( tpd_i2c_client, TCI_ENABLE_CTRL, 1 );			// tci enable
			tci_control ( tpd_i2c_client, TAP_COUNT_CTRL, 2 );			// tap count = 2
			tci_control ( tpd_i2c_client, MIN_INTERTAP_CTRL, 0 );		// min inter_tap = 0
			tci_control ( tpd_i2c_client, MAX_INTERTAP_CTRL, 70 );	 	// max inter_tap = 700ms
			tci_control ( tpd_i2c_client, TOUCH_SLOP_CTRL, 100 );		// touch_slop = 10mm
			tci_control ( tpd_i2c_client, TAP_DISTANCE_CTRL, 10 );	 	// tap distance = 10mm
			tci_control ( tpd_i2c_client, INTERRUPT_DELAY_CTRL, 0 );	// interrupt delay = 0ms
			tci_control ( tpd_i2c_client, TCI_ENABLE_CTRL2, 0 );		// tci-2 disable
			tci_control ( tpd_i2c_client, REPORT_MODE_CTRL, 1 );		// wakeup_gesture_only
			break;

		case LPWG_PASSWORD:		// Double-Tap and Multi-Tap
			tci_control ( tpd_i2c_client, TCI_ENABLE_CTRL, 1 );			// tci-1 enable
			tci_control ( tpd_i2c_client, TAP_COUNT_CTRL, 2 );			// tap count = 2
			tci_control ( tpd_i2c_client, MIN_INTERTAP_CTRL, 0 );		// min inter_tap = 0
			tci_control ( tpd_i2c_client, MAX_INTERTAP_CTRL, 70 );		// max inter_tap = 700ms
			tci_control ( tpd_i2c_client, TOUCH_SLOP_CTRL, 100 );		// touch_slop = 10mm
			tci_control ( tpd_i2c_client, TAP_DISTANCE_CTRL, 7 );		// tap distance = 7mm
			tci_control ( tpd_i2c_client, INTERRUPT_DELAY_CTRL, double_tap_check );	// interrupt delay = 0ms

			tci_control ( tpd_i2c_client, TCI_ENABLE_CTRL2, 1 );		// tci-2 enable
			tci_control ( tpd_i2c_client, TAP_COUNT_CTRL2, lpwg_tap_count ); // tap count = "user_setting"
			tci_control ( tpd_i2c_client, MIN_INTERTAP_CTRL2, 0 );		// min inter_tap = 0
			tci_control ( tpd_i2c_client, MAX_INTERTAP_CTRL2, 70 );		// max inter_tap = 700ms
			tci_control ( tpd_i2c_client, TOUCH_SLOP_CTRL2, 100 );		// touch_slop = 10mm
			tci_control ( tpd_i2c_client, TAP_DISTANCE_CTRL2, 255 );	// tap distance = MAX
			tci_control ( tpd_i2c_client, INTERRUPT_DELAY_CTRL2, 0 );	// interrupt delay = 0ms

			tci_control ( tpd_i2c_client, REPORT_MODE_CTRL, 1 );		// wakeup_gesture_only
			break;

		default:
			tci_control ( tpd_i2c_client, TCI_ENABLE_CTRL, 0 );			// tci-1 disable
			tci_control ( tpd_i2c_client, TCI_ENABLE_CTRL2, 0 );		// tci-2 disable
			tci_control ( tpd_i2c_client, REPORT_MODE_CTRL, 0 );		// normal
			sleep_control ( tpd_i2c_client, 1, 0 );
			break;
	}

	TPD_LOG ( "lpwg_control: lpwg_mode[%d]\n", mode );

	return 0;
}

static int lpwg_update_all ( struct work_struct *lpwg_update_timer )
{
	TPD_FUN ();
	int ret = 0;
	int sleep_status = 0;
	int lpwg_status = 0;
	bool req_lpwg_param = false;

	if ( lpwg_ctrl.screen )		// on(1)
	{
		sleep_status = 1;
		lpwg_status  = 0;
	}
	else if ( !lpwg_ctrl.screen && lpwg_ctrl.qcover )	// off(0), closed(1)
	{
		sleep_status = 1;
		lpwg_status = 1;
	}
	else if ( !lpwg_ctrl.screen && !lpwg_ctrl.qcover && lpwg_ctrl.sensor )	// off(0),   open(0),  far(1)
	{
		sleep_status = 1;
		lpwg_status = lpwg_ctrl.lpwg_mode;
	}
	else if ( !lpwg_ctrl.screen && !lpwg_ctrl.qcover && !lpwg_ctrl.sensor )	// off(0),   open(0), near(0)
	{
		sleep_status = 0;
		req_lpwg_param = true;
	}

	ret = sleep_control ( tpd_i2c_client, sleep_status, 0 );
	if ( ret != 0 )
	{
		TPD_ERR ( "sleep_control is failed\n" );
		return -1;
	}

	if ( req_lpwg_param == false )
	{
		lpwg_control ( lpwg_status );
	}

	return 0;
}

static int synaptics_knock_lpwg ( struct i2c_client *client, u32 code, u32 value, struct point *data )
{
	int i = 0, ret = 0;
	u8 buffer[50] = { 0 };

	switch ( code )
	{
		case LPWG_READ:
			if ( lpwg_ctrl.password_enable )
			{
				if ( lpwg_data_num == 0 )
				{
					data[0].x = 1;
					data[0].y = 1;
					data[1].x = -1;
					data[1].y = -1;
					break;
				}

				for ( i = 0 ; i < lpwg_tap_count ; i++ )
				{
					data[i].x = TS_SNTS_GET_X_POSITION(gesture_property[4*i+1],gesture_property[4*i]);
					data[i].y = TS_SNTS_GET_Y_POSITION(gesture_property[4*i+3],gesture_property[4*i+2]);
					TPD_LOG ( "TAP Position x[%3d], y[%3d]\n", data[i].x, data[i].y );
					// '-1' should be assinged to the last data.
					// Each data should be converted to LCD-resolution.
				}
				data[i].x = -1;
				data[i].y = -1;
			}
			break;

		case LPWG_ENABLE:
			if ( !lpwg_ctrl.is_suspend )
			{
				lpwg_ctrl.lpwg_mode = value;
			}
			break;

		case LPWG_LCD_X:
		case LPWG_LCD_Y:
			// If touch-resolution is not same with LCD-resolution,
			// position-data should be converted to LCD-resolution.
			break;

		case LPWG_ACTIVE_AREA_X1:
			for ( i = 0 ; i < 2 ; i++ )
			{
				synaptics_ts_read ( client, f12_info.ctrl_reg_addr[18], i+1, buffer );
				if ( i == 0 )
					buffer[i] = value;
				else
					buffer[i] = value >> 8;
				synaptics_ts_write ( client, f12_info.ctrl_reg_addr[18], i+1, buffer );
			}
			break;

		case LPWG_ACTIVE_AREA_X2:
			for ( i = 4 ; i < 6 ; i++ )
			{
				synaptics_ts_read ( client, f12_info.ctrl_reg_addr[18], i+1, buffer );
				if ( i == 4 )
					buffer[i] = value;
				else
					buffer[i] = value >> 8;
				synaptics_ts_write ( client, f12_info.ctrl_reg_addr[18], i+1, buffer );
			}
			break;

		case LPWG_ACTIVE_AREA_Y1:
			for ( i = 2 ; i < 4 ; i++ )
			{
				synaptics_ts_read ( client, f12_info.ctrl_reg_addr[18], i+1, buffer );
				if ( i == 2 )
					buffer[i] = value;
				else
					buffer[i] = value >> 8;
				synaptics_ts_write ( client, f12_info.ctrl_reg_addr[18], i+1, buffer );
			}
			sleep_control ( tpd_i2c_client, 1, 0 );
			break;

		case LPWG_ACTIVE_AREA_Y2:
			// Quick Cover Area
			for ( i = 6 ; i < 8 ; i++ )
			{
				synaptics_ts_read ( client, f12_info.ctrl_reg_addr[18], i+1, buffer );
				if ( i == 6 )
					buffer[i] = value;
				else
					buffer[i] = value >> 8;
				synaptics_ts_write ( client, f12_info.ctrl_reg_addr[18], i+1, buffer );
			}
			break;

		case LPWG_TAP_COUNT:
			lpwg_tap_count = value;
			if ( lpwg_ctrl.password_enable )
			{
				tci_control ( client, TAP_COUNT_CTRL, lpwg_tap_count );
			}
			break;

		case LPWG_LENGTH_BETWEEN_TAP:
			if ( lpwg_ctrl.double_tap_enable || lpwg_ctrl.password_enable )
			{
				tci_control ( client, TAP_DISTANCE_CTRL, value );
			}
			break;

		case LPWG_DOUBLE_TAP_CHECK:
			double_tap_check = value;
			if ( lpwg_ctrl.password_enable )
			{
				tci_control ( client, INTERRUPT_DELAY_CTRL, value );
			}
			break;

		case LPWG_REPLY:
			if ( lpwg_ctrl.is_suspend )
			{
				TPD_LOG ( "LPWG_REPLY: already screen on\n" );
				break;
			}
			lpwg_ctrl.screen = value;
			queue_delayed_work ( touch_wq, &lpwg_update_timer, 0 );
			break;

		case LPWG_UPDATE_ALL:
		{
			int *v = (int *) value;
			int mode = *(v + 0);
			int screen = *(v + 1);
			int sensor = *(v + 2);
			int qcover = *(v + 3);

			lpwg_ctrl.lpwg_mode = mode;
			lpwg_ctrl.screen = screen;
			lpwg_ctrl.sensor = sensor;
			lpwg_ctrl.qcover = qcover;

			TPD_LOG ( "mode[%d], screen[%d], sensor[%d], qcover[%d]\n", lpwg_ctrl.lpwg_mode, lpwg_ctrl.screen, lpwg_ctrl.sensor, lpwg_ctrl.qcover );

			if ( lpwg_ctrl.is_suspend )
			{
				if ( cancel_delayed_work_sync ( &lpwg_update_timer ) )
				{
					TPD_LOG ( "pending queue work\n" );
				}
				queue_delayed_work ( touch_wq, &lpwg_update_timer, I2C_DELAY );
			}
			break;
		}

		default:
			break;
	}

	return 0;
}

static void lpwg_timer_func ( struct work_struct *work_timer )
{
	int ret = 0;

	ret = sleep_control ( tpd_i2c_client, 0, 1 );
	if ( ret != 0 )
	{
		TPD_ERR ( "sleep_control is failed\n" );
	}

	/* uevent report */
	kobject_uevent_env ( &foo_obj->kobj, KOBJ_CHANGE, lpwg_uevent[LPWG_PASSWORD-1] );
}

static int get_tci_data ( struct i2c_client *client, int count )
{
	int ret = 0;

	lpwg_data_num = count;
	TPD_LOG ( "LPWG tap count = %d\n", count );

	if ( !count )
	{
		return 0;
	}

	ret = synaptics_page_data_read ( client, LPWG_PAGE, LPWG_DATA_REG, 4*count, &gesture_property );
	if ( ret < 0 )
	{
		TPD_ERR ( "LPWG_DATA_REG read fail\n" );
		return -1;
	}

	return 0;
}

static int synaptics_knock_check ( void )
{
	TPD_FUN ();
	int ret = 0;
	u8 gesture_status = 0;

	ret = synaptics_page_data_read ( tpd_i2c_client, LPWG_PAGE, LPWG_STATUS_REG, 1, &gesture_status );
	if ( ret < 0 )
	{
		TPD_ERR ( "LPWG_STATUS_REG read fail\n" );
		return -1;
	}
	TPD_LOG ( "LPWG_STATUS_REG = 0x%x\n", gesture_status );

	if ( gesture_status & 0x1 )  /* Double-Tap */
	{
		if ( lpwg_ctrl.double_tap_enable )
		{
			TPD_LOG ( "Knock On occured!!\n" );
			get_tci_data ( tpd_i2c_client, 2 );

			/* uevent report */
			kobject_uevent_env ( &foo_obj->kobj, KOBJ_CHANGE, lpwg_uevent[LPWG_DOUBLE_TAP-1] );
	   	}
	}
	else if ( gesture_status & 0x2 )  /* Multi-Tap */
	{
		if ( lpwg_ctrl.password_enable )
		{
			TPD_LOG ( "Knock Code occured!!\n" );
			wake_lock ( &knock_code_lock );
			get_tci_data ( tpd_i2c_client, lpwg_tap_count );
			tci_control ( tpd_i2c_client, REPORT_MODE_CTRL, 0 );

			queue_delayed_work ( multi_tap_wq, &work_timer, msecs_to_jiffies(UEVENT_DELAY-I2C_DELAY) );
		}
	}
	else
	{
	   	TPD_ERR ( "Ignore interrupt [gesture_status=0x%02x, double_tap_enable=%d, password_enable=%d]\n", gesture_status, lpwg_ctrl.double_tap_enable, lpwg_ctrl.password_enable );
		goto error;
	}

	return 0;

error :
	return -1;
}


/****************************************************************************
* Touch Interrupt Service Routines
****************************************************************************/
static void touch_eint_interrupt_handler ( void )
{
	mt_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );
	tpd_flag = 1;
	wake_up_interruptible ( &tpd_waiter );
}

static int synaptics_touch_event_handler ( void *unused )
{
	int ret = 0;
	u8 int_status;
	u8 int_enable;
	u8 finger_count, button_count;
	u8 reg_num, finger_order;
	touch_sensor_data sensor_data;
	touch_finger_info finger_info;

	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };

	sched_setscheduler ( current, SCHED_RR, &param );

	do
	{
		set_current_state ( TASK_INTERRUPTIBLE );
		wait_event_interruptible ( tpd_waiter, tpd_flag != 0 );

		tpd_flag = 0;
		set_current_state ( TASK_RUNNING );
		mutex_lock ( &i2c_access );

		memset ( &sensor_data, 0x0, sizeof(touch_sensor_data) );
		memset ( &finger_info, 0x0, sizeof(touch_finger_info) );

		ret = synaptics_ic_status_check ();
		if ( ret != 0 )
		{
			goto exit_work;
		}

		/* read interrupt information */
		ret = synaptics_ts_read ( tpd_i2c_client, INTERRUPT_STATUS_REG, 1, (u8 *) &int_status );
		if ( ret < 0 )
		{
			TPD_ERR ( "INTERRUPT_STATUS_REG read fail\n" );
			goto exit_work;
		}

		/* Processing touch event */
		if ( int_status & INTERRUPT_MASK_LPWG )	//LPWG Check
		{
			ret = synaptics_knock_check ();
			if ( ret != 0 )
			{
				TPD_LOG ( "Touch Knock function fail\n" );
			}

			goto exit_work;
		}
		else if ( int_status & INTERRUPT_MASK_ABS0 )	//Finger Check
		{
			ret = synaptics_ts_read ( tpd_i2c_client, FINGER_DATA_REG_START, sizeof(sensor_data), (u8 *) &sensor_data.finger_data[0] );
			if (ret < 0)
			{
				TPD_ERR ( "FINGER_DATA_REG_START read fail\n" );
				goto exit_work;
			}

			if ( lpwg_ctrl.password_enable )
			{
				if ( wake_lock_active ( &knock_code_lock ) )
				{
					if ( cancel_delayed_work ( &work_timer ) )
					{
						TPD_LOG ( "Code input more than saved Knock Code\n" );
						lpwg_data_num = 0;
						queue_delayed_work ( multi_tap_wq, &work_timer, msecs_to_jiffies(UEVENT_DELAY) );
						goto exit_work;
					}
				}
			}

			for ( finger_count = 0 ; finger_count < MAX_NUM_OF_FINGER ; finger_count++ )
			{
				if ( sensor_data.finger_data[finger_count][0] == F12_FINGER_STATUS || sensor_data.finger_data[finger_count][0] == F12_STYLUS_STATUS )
				{
					/* Parsing touch data */
					finger_info.pos_x[finger_count] = TS_SNTS_GET_X_POSITION(sensor_data.finger_data[finger_count][REG_X_MSB], sensor_data.finger_data[finger_count][REG_X_LSB]);
					finger_info.pos_y[finger_count] = TS_SNTS_GET_Y_POSITION(sensor_data.finger_data[finger_count][REG_Y_MSB], sensor_data.finger_data[finger_count][REG_Y_LSB]);
					finger_info.width_major[finger_count] = TS_SNTS_GET_WIDTH_MAJOR(sensor_data.finger_data[finger_count][REG_WX], sensor_data.finger_data[finger_count][REG_WY]);
					finger_info.width_minor[finger_count] = TS_SNTS_GET_WIDTH_MINOR(sensor_data.finger_data[finger_count][REG_WX], sensor_data.finger_data[finger_count][REG_WY]);
					finger_info.orientation[finger_count] = TS_SNTS_GET_ORIENTATION(sensor_data.finger_data[finger_count][REG_WY], sensor_data.finger_data[finger_count][REG_WX]);
					finger_info.pressure[finger_count] = TS_SNTS_GET_PRESSURE(sensor_data.finger_data[finger_count][REG_Z]);

					/* Reporting touch press data */
					input_mt_slot ( tpd->dev, finger_count );
					input_mt_report_slot_state ( tpd->dev, MT_TOOL_FINGER, true );
					input_report_abs ( tpd->dev, ABS_MT_POSITION_X, finger_info.pos_x[finger_count] );
					input_report_abs ( tpd->dev, ABS_MT_POSITION_Y, finger_info.pos_y[finger_count] );
					input_report_abs ( tpd->dev, ABS_MT_PRESSURE, finger_info.pressure[finger_count] );
					input_report_abs ( tpd->dev, ABS_MT_WIDTH_MAJOR, finger_info.width_major[finger_count] );
					input_report_abs ( tpd->dev, ABS_MT_WIDTH_MINOR, finger_info.width_minor[finger_count] );
					input_report_abs ( tpd->dev, ABS_MT_ORIENTATION, finger_info.orientation[finger_count] );

					finger_state[finger_count] = TOUCH_PRESSED;
				}
				else if ( finger_state[finger_count] == TOUCH_PRESSED )
				{
					/* Reporting touch release data */
					input_mt_slot ( tpd->dev, finger_count );
					input_mt_report_slot_state ( tpd->dev, MT_TOOL_FINGER, false );

					finger_state[finger_count] = TOUCH_RELEASED;
				}
			}

			input_sync ( tpd->dev );
		}

exit_work:
		mutex_unlock ( &i2c_access );
		mt_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
	} while ( !kthread_should_stop () );

	return 0;
}


/****************************************************************************
* SYSFS function for Touch
****************************************************************************/
static ssize_t show_knock_on_type ( struct device *dev, struct device_attribute *attr, char *buf )
{
	int ret = 0;

	ret = sprintf ( buf, "%d\n", knock_on_type );

	return ret;
}
static DEVICE_ATTR ( knock_on_type, 0664, show_knock_on_type, NULL );

static ssize_t show_lpwg_data ( struct device *dev, struct device_attribute *attr, char *buf )
{
	TPD_FUN ();
	int i = 0, ret = 0;

	memset ( lpwg_data, 0, sizeof(struct point)*MAX_POINT_SIZE_FOR_LPWG );

	synaptics_knock_lpwg ( tpd_i2c_client, LPWG_READ, 0, lpwg_data );
	for ( i = 0 ; i < MAX_POINT_SIZE_FOR_LPWG ; i++ )
	{
		if ( lpwg_data[i].x == -1 && lpwg_data[i].y == -1 )
		{
			break;
		}
		ret += sprintf ( buf+ret, "%d %d\n", lpwg_data[i].x, lpwg_data[i].y );
	}

	return ret;
}

static ssize_t store_lpwg_data ( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	TPD_FUN ();
	int reply = 0;

	sscanf ( buf, "%d", &reply );
	TPD_LOG ( "LPWG RESULT = %d ", reply );

	synaptics_knock_lpwg ( tpd_i2c_client, LPWG_REPLY, reply, NULL );

	wake_unlock ( &knock_code_lock );

	return size;
}
static DEVICE_ATTR ( lpwg_data, 0664, show_lpwg_data, store_lpwg_data );

static ssize_t store_lpwg_notify ( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	int type = 0;
	int value[4] = { 0 };
	int ret = 0;

	sscanf ( buf, "%d %d %d %d %d", &type, &value[0], &value[1], &value[2], &value[3] );

	TPD_LOG ( "LPWG: type[%d] value[%d/%d/%d/%d]\n", type, value[0], value[1], value[2], value[3] );

	if ( type == 6 || type ==7 )
	{
		return size;
	}

	mutex_lock ( &notify_access );

	switch ( type )
	{
		case 1:
			TPD_LOG ( "LPWG_ENABLE : %s", value[0] ? "Enable\n" : "Disable\n" );
			synaptics_knock_lpwg ( tpd_i2c_client, LPWG_ENABLE, value[0], NULL );
			break;

		case 2:
			synaptics_knock_lpwg ( tpd_i2c_client, LPWG_LCD_X, value[0], NULL );
			synaptics_knock_lpwg ( tpd_i2c_client, LPWG_LCD_Y, value[1], NULL );
			break;

		case 3:
			synaptics_knock_lpwg ( tpd_i2c_client, LPWG_ACTIVE_AREA_X1, value[0], NULL );
			synaptics_knock_lpwg ( tpd_i2c_client, LPWG_ACTIVE_AREA_X2, value[1], NULL );
			synaptics_knock_lpwg ( tpd_i2c_client, LPWG_ACTIVE_AREA_Y1, value[2], NULL );
			synaptics_knock_lpwg ( tpd_i2c_client, LPWG_ACTIVE_AREA_Y2, value[3], NULL );
			break;

		case 4:
			synaptics_knock_lpwg ( tpd_i2c_client, LPWG_TAP_COUNT, value[0], NULL );
			break;

		case 5:
			synaptics_knock_lpwg ( tpd_i2c_client, LPWG_LENGTH_BETWEEN_TAP, value[0], NULL );
			break;

		case 6:
			break;

		case 7:
			break;

		case 8:
			synaptics_knock_lpwg ( tpd_i2c_client, LPWG_DOUBLE_TAP_CHECK, value[0], NULL );
			break;

		case 9:
			synaptics_knock_lpwg ( tpd_i2c_client, LPWG_UPDATE_ALL, (int) &value[0], NULL );
			break;

		default:
			break;
	}

	mutex_unlock ( &notify_access );

	return size;
}
static DEVICE_ATTR ( lpwg_notify, 0664, NULL, store_lpwg_notify );

static void write_log ( char *data )
{
	int fd;
	char *fname = "/mnt/sdcard/touch_self_test.txt";

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0644);

	if (fd >= 0)
	{
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}

	set_fs(old_fs);
}

static ssize_t show_sd ( struct device *dev, struct device_attribute *attr, char *buf )
{
	TPD_FUN ();
	int ret = 0;
	int full_raw_cap = 0;
	int trx_to_trx = 0;
	int trx_to_gnd = 0;
	int high_resistance = 0;
	int noise_delta = 0;
	char fw_config_id[5] = {0};
	char fw_product_id[11] = {0};

	if ( !lpwg_ctrl.is_suspend )
	{
		ret += sprintf ( buf+ret, "\n\n" );
		write_log ( buf );
		msleep ( 10 );

		mt_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );

		ret = synaptics_ts_read ( tpd_i2c_client, PRODUCT_ID_REG, 10, &fw_product_id[0] );
		if ( ret < 0 )
		{
			TPD_ERR ( "PRODUCT_ID_REG read fail\n" );
		}
		ret = synaptics_ts_read ( tpd_i2c_client, FLASH_CONFIG_ID_REG, 4, &fw_config_id[0] );
		if ( ret < 0 )
		{
			TPD_ERR ( "FLASH_CONFIG_ID_REG read fail\n" );
		}

		ret += sprintf ( buf+ret, "==Touch IC information==\n" );
		ret += sprintf ( buf+ret, "PRODUCT ID : %s\n", fw_product_id );
		ret += sprintf ( buf+ret, "CONFIG ID  : %02x%02x%02x%02x\n", fw_config_id[0], fw_config_id[1], fw_config_id[2], fw_config_id[3] );

		SCAN_PDT ();

		full_raw_cap = F54Test ( 'a', 0, buf );
		msleep(30);
		noise_delta = F54Test ( 'x', 0, buf );
		trx_to_trx = F54Test ( 'f', 0, buf );
		trx_to_gnd = F54Test ( 'e', 0, buf );
		high_resistance = F54Test ( 'g', 0, buf );

		write_log ( buf );
		msleep ( 10 );
		synaptics_initialize ( tpd_i2c_client );
		msleep ( 30 );

		ret += sprintf ( buf+ret, "========RESULT=======\n" );
		ret += sprintf ( buf+ret, "Channel Status : %s", (trx_to_trx && trx_to_gnd && high_resistance) ? "Pass\n" : "Fail\n" );
		ret += sprintf ( buf+ret, "Raw Data : %s", (full_raw_cap && noise_delta) ? "Pass\n" : "Fail\n" );

		mt_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
	}
	else
	{
		ret += sprintf ( buf+ret, "state=[suspend]. we cannot use I2C, now. Test Result: Fail\n" );
	}

	return ret;
}
static DEVICE_ATTR ( sd, 0664, show_sd, NULL );

static ssize_t show_reset ( struct device *dev, struct device_attribute *attr, char *buf )
{
	int ret = 0;

	TPD_LOG ( "Touch Reset!!\n" );
	synaptics_ic_reset ();
	synaptics_initialize ( tpd_i2c_client );

	return ret;
}
static DEVICE_ATTR ( reset, 0664, show_reset, NULL );

static ssize_t synaptics_store_write ( struct device *dev, struct device_attribute *attr, char *buf, size_t size )
{
	u8 temp = 0;
	u8 page = 0;
	u8 reg = 0;
	u8 value = 0;
	int ret;

	//sscanf ( buf, "%d %x %x", &page, &reg, &value );
	TPD_LOG ( "(write) page=%d, reg=0x%x, value=0x%x\n", page, reg, value );

	ret = synaptics_page_data_write ( tpd_i2c_client, page, reg, 1, &value );
	if ( ret < 0 )
	{
		TPD_ERR ( "REGISTER write fail\n" );
	}

	return size;
}
static DEVICE_ATTR ( write, 0664, NULL, synaptics_store_write );

static ssize_t synaptics_store_read ( struct device *dev, struct device_attribute *attr, char *buf, size_t size )
{
	u8 temp = 0;
	u8 page = 0;
	u8 reg = 0;
	int ret;

	//sscanf ( buf, "%d %x", &page, &reg );

	ret = synaptics_page_data_read ( tpd_i2c_client, page, reg, 1, &temp );
	if ( ret < 0 )
	{
		TPD_ERR ( "REGISTER read fail\n" );
	}

	TPD_LOG ( "(read) page=%d, reg=0x%x, value=0x%x\n", page, reg, temp );

	return size;
}
static DEVICE_ATTR ( read, 0664, NULL, synaptics_store_read );

static struct attribute *lge_touch_attrs[] = {
	&dev_attr_knock_on_type.attr,
	&dev_attr_lpwg_data.attr,
	&dev_attr_lpwg_notify.attr,
	&dev_attr_sd.attr,
	&dev_attr_reset.attr,
	&dev_attr_write.attr,
	&dev_attr_read.attr,
	NULL,
};

static struct attribute_group lge_touch_group = {
	.name = LGE_TOUCH_NAME,
	.attrs = lge_touch_attrs,
};

#if defined(LGE_USE_SYNAPTICS_FW_UPGRADE)
static ssize_t synaptics_store_firmware ( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	int ret;
	char path[256] = { "/mnt/sdcard/fw.img" };

	fw_force_update = 1;
	memcpy ( fw_path, path, sizeof(fw_path) );

	queue_work ( touch_wq, &work_fw_upgrade );

	return size;
}
static DEVICE_ATTR ( firmware, 0664, NULL, synaptics_store_firmware );
#endif

static ssize_t synaptics_show_version ( struct device *dev, struct device_attribute *attr, char *buf )
{
	u8 fw_release = 0;
	u8 fw_ver = 0;
	char fw_config_id[5] = {0};
	int ret;

	ret = synaptics_ts_read ( tpd_i2c_client, FLASH_CONFIG_ID_REG, 4, &fw_config_id[0] );
	if ( ret < 0 )
	{
		TPD_ERR ( "FLASH_CONFIG_ID_REG read fail\n" );
	}

	fw_release = fw_config_id[3] & 0x80;
	fw_ver = fw_config_id[3] & 0x7F;

	return sprintf ( buf, "%d.%d\n", fw_release, fw_ver );
}
static DEVICE_ATTR ( version, 0664, synaptics_show_version, NULL );


/****************************************************************************
* Synaptics_Touch_IC Initialize Function
****************************************************************************/
void get_f12_info ( struct i2c_client *client )
{
	int retval;
	struct synaptics_ts_f12_query_5 query_5;
	struct synaptics_ts_f12_query_8 query_8;
	int i;
	u8 offset;

	/* ctrl_reg_info setting */
	retval = synaptics_ts_read ( client, finger.query_base+5, sizeof(query_5.data), query_5.data );
	if ( retval < 0 )
	{
		TPD_ERR ( "Failed to read from F12_2D_QUERY_05_Control_Presence register\n" );
		return;
	}

	f12_info.ctrl_reg_is_present[0] = query_5.ctrl_00_is_present;
	f12_info.ctrl_reg_is_present[1] = query_5.ctrl_01_is_present;
	f12_info.ctrl_reg_is_present[2] = query_5.ctrl_02_is_present;
	f12_info.ctrl_reg_is_present[3] = query_5.ctrl_03_is_present;
	f12_info.ctrl_reg_is_present[4] = query_5.ctrl_04_is_present;
	f12_info.ctrl_reg_is_present[5] = query_5.ctrl_05_is_present;
	f12_info.ctrl_reg_is_present[6] = query_5.ctrl_06_is_present;
	f12_info.ctrl_reg_is_present[7] = query_5.ctrl_07_is_present;
	f12_info.ctrl_reg_is_present[8] = query_5.ctrl_08_is_present;
	f12_info.ctrl_reg_is_present[9] = query_5.ctrl_09_is_present;
	f12_info.ctrl_reg_is_present[10] = query_5.ctrl_10_is_present;
	f12_info.ctrl_reg_is_present[11] = query_5.ctrl_11_is_present;
	f12_info.ctrl_reg_is_present[12] = query_5.ctrl_12_is_present;
	f12_info.ctrl_reg_is_present[13] = query_5.ctrl_13_is_present;
	f12_info.ctrl_reg_is_present[14] = query_5.ctrl_14_is_present;
	f12_info.ctrl_reg_is_present[15] = query_5.ctrl_15_is_present;
	f12_info.ctrl_reg_is_present[16] = query_5.ctrl_16_is_present;
	f12_info.ctrl_reg_is_present[17] = query_5.ctrl_17_is_present;
	f12_info.ctrl_reg_is_present[18] = query_5.ctrl_18_is_present;
	f12_info.ctrl_reg_is_present[19] = query_5.ctrl_19_is_present;
	f12_info.ctrl_reg_is_present[20] = query_5.ctrl_20_is_present;
	f12_info.ctrl_reg_is_present[21] = query_5.ctrl_21_is_present;
	f12_info.ctrl_reg_is_present[22] = query_5.ctrl_22_is_present;
	f12_info.ctrl_reg_is_present[23] = query_5.ctrl_23_is_present;
	f12_info.ctrl_reg_is_present[24] = query_5.ctrl_24_is_present;
	f12_info.ctrl_reg_is_present[25] = query_5.ctrl_25_is_present;
	f12_info.ctrl_reg_is_present[26] = query_5.ctrl_26_is_present;
	f12_info.ctrl_reg_is_present[27] = query_5.ctrl_27_is_present;
	f12_info.ctrl_reg_is_present[28] = query_5.ctrl_28_is_present;
	f12_info.ctrl_reg_is_present[29] = query_5.ctrl_29_is_present;
	f12_info.ctrl_reg_is_present[30] = query_5.ctrl_30_is_present;
	f12_info.ctrl_reg_is_present[31] = query_5.ctrl_31_is_present;

	offset = 0;

	for ( i = 0 ; i < 32 ; i++ )
	{
		f12_info.ctrl_reg_addr[i] = finger.control_base + offset;

		if ( f12_info.ctrl_reg_is_present[i] )
		{
			offset++;
		}
	}

	/* data_reg_info setting */
	retval = synaptics_ts_read ( client, (finger.query_base+8), sizeof(query_8.data), query_8.data );
	if ( retval < 0 )
	{
		TPD_ERR ( "Failed to read from F12_2D_QUERY_08_Data_Presence register\n" );
		return;
	}

	f12_info.data_reg_is_present[0] = query_8.data_00_is_present;
	f12_info.data_reg_is_present[1] = query_8.data_01_is_present;
	f12_info.data_reg_is_present[2] = query_8.data_02_is_present;
	f12_info.data_reg_is_present[3] = query_8.data_03_is_present;
	f12_info.data_reg_is_present[4] = query_8.data_04_is_present;
	f12_info.data_reg_is_present[5] = query_8.data_05_is_present;
	f12_info.data_reg_is_present[6] = query_8.data_06_is_present;
	f12_info.data_reg_is_present[7] = query_8.data_07_is_present;
	f12_info.data_reg_is_present[8] = query_8.data_08_is_present;
	f12_info.data_reg_is_present[9] = query_8.data_09_is_present;
	f12_info.data_reg_is_present[10] = query_8.data_10_is_present;
	f12_info.data_reg_is_present[11] = query_8.data_11_is_present;
	f12_info.data_reg_is_present[12] = query_8.data_12_is_present;
	f12_info.data_reg_is_present[13] = query_8.data_13_is_present;
	f12_info.data_reg_is_present[14] = query_8.data_14_is_present;
	f12_info.data_reg_is_present[15] = query_8.data_15_is_present;

	offset = 0;

	for ( i = 0 ; i < 16 ; i++ )
	{
		f12_info.data_reg_addr[i] = finger.data_base + offset;

		if ( f12_info.data_reg_is_present[i] )
		{
			offset++;
		}
	}

	/* print info */
	for ( i = 0 ; i < 32 ; i++ )
	{
		if ( f12_info.ctrl_reg_is_present[i] )
		{
			TPD_LOG ( "f12_info.ctrl_reg_addr[%d]=0x%02X\n", i, f12_info.ctrl_reg_addr[i] );
		}
	}

	for ( i = 0 ; i < 16 ; i++ )
	{
		if ( f12_info.data_reg_is_present[i] )
		{
			TPD_LOG ( "f12_info.data_reg_addr[%d]=0x%02X\n", i, f12_info.data_reg_addr[i] );
		}
	}

	return;
}

static void read_page_description_table ( struct i2c_client *client )
{
	TPD_FUN ();
	int ret = 0;
	function_descriptor buffer;
	u8 page_num;
	u16 u_address;

	memset ( &buffer, 0x0, sizeof(function_descriptor) );

	for ( page_num = 0 ; page_num < PAGE_MAX_NUM ; page_num++ )
	{
		ret = synaptics_ts_write ( client, PAGE_SELECT_REG, 1, &page_num );
		if ( ret < 0 )
		{
			TPD_ERR ( "PAGE_SELECT_REG write fail (page_num=%d)\n", page_num );
		}

		for ( u_address = DESCRIPTION_TABLE_START ; u_address > 10 ; u_address -= sizeof(function_descriptor) )
		{
			ret = synaptics_ts_read ( client, u_address, sizeof(buffer), (u8 *) &buffer );
			if ( ret < 0 )
			{
				TPD_ERR ( "function_descriptor read fail\n" );
				return;
			}

			TPD_LOG ( "page_num = %d, function=%x\n", page_num, buffer.function_exist );
			if ( buffer.function_exist == 0 )
				break;

			switch ( buffer.function_exist )
			{
				case RMI_DEVICE_CONTROL:
					device_control = buffer;
					device_control_page = page_num;
					break;
				case TOUCHPAD_SENSORS:
					finger = buffer;
					finger_page = page_num;
					break;
				case CAPACITIVE_BUTTON_SENSORS:
					button = buffer;
					button_page = page_num;
					break;
				case FLASH_MEMORY_MANAGEMENT:
					flash_memory = buffer;
					flash_memory_page = page_num;
					break;
				case MULTI_TAP_GESTURE:
					multi_tap = buffer;
					multi_tap_page = page_num;
					break;
			}
		}
	}

	ret = synaptics_ts_write ( client, PAGE_SELECT_REG, 1, &device_control_page );
	if ( ret < 0 )
	{
		TPD_ERR ( "PAGE_SELECT_REG write fail\n" );
	}

	get_f12_info ( client );
}

#if defined(LGE_USE_SYNAPTICS_FW_UPGRADE)
static int synaptics_firmware_check ( struct i2c_client *client )
{
	TPD_FUN ();
	int ret;
	u8 device_status = 0;
	u8 flash_status = 0;
	u8 fw_ver, image_ver;
	u8 fw_release, image_release;
	char fw_config_id[5] = {0};
	char fw_product_id[11] = {0};
	char image_config_id[5] = {0};
	char image_product_id[11] = {0};

	if ( fw_force_update == 1 )
	{
		TPD_LOG ( "Firmware force update by fw img\n" );
		return -1;
	}

	/* read Firmware information in Download Image */
	fw_start = (unsigned char *) &SynaFirmware[0];
	fw_size = sizeof(SynaFirmware);
	strncpy ( image_product_id, &SynaFirmware[0x0040], 6 );
	strncpy ( image_config_id, &SynaFirmware[0x16d00], 4 );

	/* read Firmware information in Touch IC */
	ret = synaptics_ts_read ( client, PRODUCT_ID_REG, 10, &fw_product_id[0] );
	if ( ret < 0 )
	{
		TPD_ERR ( "PRODUCT_ID_REG read fail\n" );
		goto err_firmware_check;
	}
	ret = synaptics_ts_read ( client, FLASH_CONFIG_ID_REG, 4, &fw_config_id[0] );
	if ( ret < 0 )
	{
		TPD_ERR ( "FLASH_CONFIG_ID_REG read fail\n" );
		goto err_firmware_check;
	}

	/* parsing Firmware Version information */
	fw_release = fw_config_id[3] & 0x80;
	image_release = image_config_id[3] & 0x80;
	fw_ver = fw_config_id[3] & 0x7F;
	image_ver = image_config_id[3] & 0x7F;

	TPD_LOG ( "<Firmware in download image>\n" );
	TPD_LOG ( "PRODUCT ID : %s\n", image_product_id );
	TPD_LOG ( "CONFIG ID  : %02x%02x%02x%02x\n", image_config_id[0], image_config_id[1], image_config_id[2], image_config_id[3] );
	TPD_LOG ( "FW Version : %d.%d\n", image_release, image_ver );
	TPD_LOG ( "<Firmware in touch IC>\n" );
	TPD_LOG ( "PRODUCT ID : %s\n", fw_product_id );
	TPD_LOG ( "CONFIG ID  : %02x%02x%02x%02x\n", fw_config_id[0], fw_config_id[1], fw_config_id[2], fw_config_id[3] );
	TPD_LOG ( "FW Version : %d.%d\n\n", fw_release, fw_ver );

	ret = synaptics_ts_read ( client, DEVICE_STATUS_REG, 1, &device_status );
	if ( ret < 0 )
	{
		TPD_ERR ( "DEVICE_STATUS_REG read fail\n" );
		goto err_firmware_check;
	}
	ret = synaptics_ts_read ( client, FLASH_STATUS_REG, 1, &flash_status );
	if ( ret < 0 )
	{
		TPD_ERR ( "FLASH_STATUS_REG read fail\n" );
		goto err_firmware_check;
	}

	if ( ( device_status & FLASH_PROG_MASK ) || ( device_status & FW_CRC_FAILURE_MASK ) != 0 || ( flash_status & FLASH_STATUS_MASK ) != 0)
	{
		TPD_LOG ( "Firmware has a problem. [device_status=%x, flash_status=%x]\nso it needs Firmware update.\n", device_status, flash_status );
		TPD_LOG ( "Firmware Update [%d.%d ver ==> %d.%d ver]\n", fw_release, fw_ver, image_release, image_ver );
		return -1;
	}
	else
	{
		if ( ( !strcmp ( fw_product_id, image_product_id ) ) && ( ( fw_release != image_release ) || ( fw_ver < image_ver ) ) )
		{
			TPD_LOG ( "Firmware Update [%d.%d ver ==> %d.%d ver]\n", fw_release, fw_ver, image_release, image_ver );
			return -1;
		}
		else
		{
			TPD_LOG ( "No need to update Firmware [%d.%d ver]\n", fw_release, fw_ver );
			return 0;
		}
	}

err_firmware_check:
	TPD_ERR ( "Firmware check error for I2C fail\n" );
	return 0;
}

static int synaptics_firmware_update ( struct work_struct *work_fw_upgrade )
{
	TPD_FUN ();
	int ret = 0;

	ret	= synaptics_firmware_check ( tpd_i2c_client );

	ret = 0;//Temp
	if ( ret != 0 )
	{
		if ( !download_status )
		{
			download_status = 1;
			wake_lock ( &fw_suspend_lock );

			mt_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );

			mtk_wdt_enable ( WK_WDT_DIS );
			TPD_LOG ( "Watchdog disable\n" );

			ret = FirmwareUpgrade ( tpd_i2c_client, fw_path, fw_size, fw_start );
			if ( ret < 0 )
			{
				TPD_ERR ( "Firmware update Fail!!!\n" );
			}
			else
			{
				TPD_ERR ( "Firmware upgrade Complete\n" );
			}

			mtk_wdt_enable ( WK_WDT_EN );
			TPD_LOG ( "Watchdog enable\n" );

			memset ( fw_path, 0x00, sizeof(fw_path) );
			fw_force_update = 0;

			synaptics_ic_reset ();

			wake_unlock ( &fw_suspend_lock );
			download_status = 0;

			arch_reset(0,NULL);
		}
		else
		{
			TPD_ERR ( "Firmware Upgrade process is aready working on\n" );
		}

		read_page_description_table ( tpd_i2c_client );
	}

	synaptics_initialize ( tpd_i2c_client );
	mt_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
}
#endif

void synaptics_initialize( struct i2c_client *client )
{
	TPD_FUN ();
	int ret = 0;
	u8 temp = 0;

	temp = DEVICE_CONTROL_NORMAL_OP | DEVICE_CONTROL_CONFIGURED;
	ret = synaptics_ts_write ( client, DEVICE_CONTROL_REG, 1, &temp );
	if (ret < 0)
	{
		TPD_ERR ( "DEVICE_CONTROL_REG write fail\n" );
	}

	temp = 0x7F;
	ret = synaptics_ts_write ( client, INTERRUPT_ENABLE_REG, 1, &temp );
	if (ret < 0)
	{
		TPD_ERR ( "INTERRUPT_ENABLE_REG write fail\n" );
	}

	ret = synaptics_ts_read ( client, INTERRUPT_STATUS_REG, 1, &temp ); // It always should be done last.==> Interrupt Pin Clear.
	if ( ret < 0 )
	{
		TPD_ERR ( "INTERRUPT_STATUS_REG read fail\n" );
	}
	TPD_LOG ( "INTERRUPT_STATUS_REG value = %d\n", temp );

	return 0;
}

static int synaptics_workqueue_init ( void )
{
	TPD_FUN ();

#ifdef LGE_USE_SYNAPTICS_FW_UPGRADE
	touch_wq = create_singlethread_workqueue ( "touch_wq" );
	if ( touch_wq )
	{
		INIT_WORK ( &work_fw_upgrade, synaptics_firmware_update );
		INIT_DELAYED_WORK ( &lpwg_update_timer, lpwg_update_all );

		wake_lock_init ( &fw_suspend_lock, WAKE_LOCK_SUSPEND, "fw_wakelock" );
	}
	else
	{
		goto err_workqueue_init;
	}
#endif

	multi_tap_wq = create_singlethread_workqueue ( "multi_tap_wq" );
	if ( multi_tap_wq )
	{
		INIT_DELAYED_WORK ( &work_timer, lpwg_timer_func );

		wake_lock_init ( &knock_code_lock, WAKE_LOCK_SUSPEND, "knock_code" );
	}
	else
	{
		goto err_workqueue_init;
	}

	return 0;

err_workqueue_init:
	TPD_ERR ( "create_singlethread_workqueue failed\n" );
	return -1;
}

void synaptics_init_sysfs ( void )
{
	TPD_FUN ();
	int err;

	touch_class = class_create ( THIS_MODULE, "touch" );

#if defined(LGE_USE_SYNAPTICS_FW_UPGRADE)
	touch_fw_dev = device_create ( touch_class, NULL, 0, NULL, "firmware" );
	err = device_create_file ( touch_fw_dev, &dev_attr_firmware );
	if ( err )
	{
		TPD_ERR ( "Touchscreen : [firmware] touch device_create_file Fail\n" );
		device_remove_file ( touch_fw_dev, &dev_attr_firmware );
	}

	err = device_create_file ( touch_fw_dev, &dev_attr_version );
	if ( err )
	{
		TPD_ERR ( "Touchscreen : [version] touch device_create_file Fail\n" );
		device_remove_file ( touch_fw_dev, &dev_attr_version );
	}
#endif

	sysfs_create_group ( tpd->dev->dev.kobj.parent, &lge_touch_group );
}
EXPORT_SYMBOL ( synaptics_init_sysfs );

#define to_foo_obj(x) container_of(x, struct foo_obj, kobj)
struct foo_attribute {
	struct attribute attr;
	ssize_t (*show)(struct foo_obj *foo, struct foo_attribute *attr, char *buf);
	ssize_t (*store)(struct foo_obj *foo, struct foo_attribute *attr, const char *buf, size_t count);
};
#define to_foo_attr(x) container_of(x, struct foo_attribute, attr)
static ssize_t foo_attr_show(struct kobject *kobj,
			     struct attribute *attr,
			     char *buf){
		struct foo_attribute *attribute;
	struct foo_obj *foo;

	attribute = to_foo_attr(attr);
	foo = to_foo_obj(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(foo, attribute, buf);
}
static ssize_t foo_attr_store(struct kobject *kobj,
			      struct attribute *attr,
			      const char *buf, size_t len)
{
	struct foo_attribute *attribute;
	struct foo_obj *foo;

	attribute = to_foo_attr(attr);
	foo = to_foo_obj(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(foo, attribute, buf, len);
}
static const struct sysfs_ops foo_sysfs_ops = {
	.show = foo_attr_show,
	.store = foo_attr_store,
};
static void foo_release(struct kobject *kobj)
{
	struct foo_obj *foo;

	foo = to_foo_obj(kobj);
	kfree(foo);
}

static ssize_t foo_show(struct foo_obj *foo_obj, struct foo_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", foo_obj->interrupt);
}
static ssize_t foo_store(struct foo_obj *foo_obj, struct foo_attribute *attr,
			 const char *buf, size_t count)
{
	sscanf(buf, "%du", &foo_obj->interrupt);
	return count;
}
static struct foo_attribute foo_attribute =__ATTR(interrupt, 0664, foo_show, foo_store);
static struct attribute *foo_default_attrs[] = {
	&foo_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};
static struct kobj_type foo_ktype = {
	.sysfs_ops = &foo_sysfs_ops,
	.release = foo_release,
	.default_attrs = foo_default_attrs,
};
static struct kset *example_kset;

static struct foo_obj *create_foo_obj(const char *name){
	struct foo_obj *foo;
	int retval;
	foo = kzalloc(sizeof(*foo), GFP_KERNEL);
	if (!foo)
		return NULL;
	foo->kobj.kset = example_kset;
	retval = kobject_init_and_add(&foo->kobj, &foo_ktype, NULL, "%s", name);
	if (retval) {
		kobject_put(&foo->kobj);
		return NULL;
	}
	kobject_uevent(&foo->kobj, KOBJ_ADD);
	return foo;
}


/****************************************************************************
* I2C BUS Related Functions
****************************************************************************/
static int synaptics_i2c_probe ( struct i2c_client *client, const struct i2c_device_id *id )
{
	TPD_FUN ();
	int ret = 0;
	struct foo_obj *foo;
	struct task_struct *thread = NULL;

	/* i2c_check_functionality */
	if ( !i2c_check_functionality(client->adapter, I2C_FUNC_I2C) )
	{
		TPD_ERR ( "i2c functionality check error\n" );
		goto err_probing;
	}

	tpd_i2c_client = client;
	ds4_i2c_client = client;

	/* Turn on the power for Touch */
	//synaptics_power ( 0 );
	//synaptics_power ( 1 );

	mt_set_gpio_mode ( GPIO_CTP_RST_PIN, GPIO_MODE_00 );
	mt_set_gpio_dir ( GPIO_CTP_RST_PIN, GPIO_DIR_OUT );
	mt_set_gpio_out ( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
	synaptics_ic_reset ();

	/* Initialize work queue  */
	ret = synaptics_workqueue_init ();
	if ( ret != 0 )
	{
		TPD_ERR ( "synaptics_workqueue_init failed\n" );
		goto err_probing;
	}

	thread = kthread_run ( synaptics_touch_event_handler, 0, TPD_DEVICE );
	if ( IS_ERR ( thread ) )
	{
		ret = PTR_ERR ( thread );
		TPD_ERR ( "failed to create kernel thread: %d\n", ret );
		goto err_probing;
	}

	/* Configure external ( GPIO ) interrupt */
	synaptics_setup_eint ();

	/* Find register map */
	read_page_description_table ( client );

	/* Initialize for Knock function  */
	knock_on_type = 1;
	example_kset = kset_create_and_add ( "lge", NULL, kernel_kobj );
	foo_obj = create_foo_obj ( LGE_TOUCH_NAME );

	/* Touch Firmware Update */
#if defined(LGE_USE_SYNAPTICS_FW_UPGRADE)
	queue_work ( touch_wq, &work_fw_upgrade );
#else
	synaptics_initialize ( client );
#endif

	tpd_load_status = 1;
	return 0;

err_probing:
	return ret;
}

static int synaptics_i2c_remove(struct i2c_client *client)
{
	TPD_FUN ();
	return 0;
}

static int synaptics_i2c_detect ( struct i2c_client *client, struct i2c_board_info *info )
{
	TPD_FUN ();
	strcpy ( info->type, "mtk-tpd" );
	return 0;
}

static const struct i2c_device_id tpd_i2c_id[] = { { TPD_DEV_NAME, 0 },	{} };

static struct i2c_driver tpd_i2c_driver = {
	.driver.name = "mtk-tpd",
	.probe = synaptics_i2c_probe,
	.remove = synaptics_i2c_remove,
	.detect = synaptics_i2c_detect,
	.id_table = tpd_i2c_id,
};


/****************************************************************************
* Linux Device Driver Related Functions
****************************************************************************/
static int synaptics_local_init ( void )
{
	TPD_FUN ();

	if ( i2c_add_driver ( &tpd_i2c_driver ) != 0 )
	{
		TPD_ERR ( "i2c_add_driver failed\n" );
		return -1;
	}

	if ( tpd_load_status == 0 )
	{
		TPD_ERR ( "touch driver probing failed\n" );
		i2c_del_driver ( &tpd_i2c_driver );
		return -1;
	}

	tpd_type_cap = 1;

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_suspend ( struct early_suspend *h )
{
	TPD_FUN ();

	lpwg_ctrl.is_suspend = 1;

	if ( cancel_delayed_work_sync ( &lpwg_update_timer ) )
	{
		TPD_LOG ( "pending queue work\n" );
	}

	synaptics_release_all_finger ();

	//if ( g_qem_check == '1' )
	//{
		//TPD_LOG ( "Touch power off for MFTS\n" );
		//synaptics_power ( 0 );
		//return;
	//}

	queue_delayed_work ( touch_wq, &lpwg_update_timer, I2C_DELAY );
}

static void synaptics_resume ( struct early_suspend *h )
{
	TPD_FUN ();

	if ( cancel_delayed_work_sync ( &lpwg_update_timer ) )
	{
		TPD_LOG ( "pending queue work\n" );
	}

	//if ( g_qem_check == '1' )
	//{
		//TPD_LOG("Touch power on for MFTS\n");
		//synaptics_power ( 1 );
		//synaptics_ic_reset ();
		//synaptics_release_all_finger ();
		//lpwg_ctrl.is_suspend = 0;
		//return;
	//}

	synaptics_ic_reset ();
	synaptics_initialize ( tpd_i2c_client );

	if ( key_lock_status == 0 )
	{
		mt_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
	}
	else
	{
		mt_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );
	}

	lpwg_ctrl.is_suspend = 0;
}
#endif

static struct i2c_board_info __initdata i2c_tpd = {	I2C_BOARD_INFO ( TPD_DEV_NAME, TPD_I2C_ADDRESS ) };

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = TPD_DEV_NAME,
	.tpd_local_init = synaptics_local_init,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend = synaptics_suspend,
	.resume = synaptics_resume,
#endif
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

static int __init synaptics_driver_init ( void )
{
	TPD_FUN ();

	I2CDMABuf_va = (u8 *) dma_alloc_coherent ( NULL, 4096, &I2CDMABuf_pa, GFP_KERNEL );
	if ( !I2CDMABuf_va )
	{
		TPD_ERR ( "Allocate Touch DMA I2C Buffer failed!\n" );
		return -ENOMEM;
	}

	i2c_register_board_info ( 0, &i2c_tpd, 1 );
	if ( tpd_driver_add ( &tpd_device_driver ) < 0 )
	{
		TPD_ERR ( "tpd_driver_add failed\n" );
	}

	return 0;
}

static void __exit synaptics_driver_exit ( void )
{
	TPD_FUN ();

	tpd_driver_remove ( &tpd_device_driver );

	if ( I2CDMABuf_va )
	{
		dma_free_coherent ( NULL, 4096, I2CDMABuf_va, I2CDMABuf_pa );
		I2CDMABuf_va = NULL;
		I2CDMABuf_pa = 0;
	}
}


module_init ( synaptics_driver_init );
module_exit ( synaptics_driver_exit );

MODULE_DESCRIPTION ( "Synaptics 3320 Touchscreen Driver for MTK platform" );
MODULE_AUTHOR ( "Junmo Kang <junmo.kang@lge.com>" );
MODULE_LICENSE ( "GPL" );

/* End Of File */
