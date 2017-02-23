/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2009
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

/*****************************************************************************
*                E X T E R N A L      R E F E R E N C E S
******************************************************************************
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include "yusu_android_speaker.h"
#include "yusu_amp_reg.h"
#include <cust_gpio_usage.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/xlog.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <mach/mt_gpio.h>
//tpa2028 start
//#include <mach/board_lge.h>
#include "tpa2028d.h"
//tpa2028 end

/*****************************************************************************
*                          DEBUG INFO
*****************************************************************************/

static bool eamp_log_on = true;

#define EAMP_PRINTK(fmt, arg...) \
    do { \
        if (eamp_log_on) xlog_printk(ANDROID_LOG_INFO,"EAMP", \
                    "[EAMP]: %s() "fmt"\n", __func__,##arg); \
    }while (0)

#define ON_OFF_STR(val) (val) ? "On" : "Off"
#define IN_MODE_STR(val) (val) ? "Diffential Input" : "Single Input"
#define SPK_RCV_SEL_STR(val) (val) ? "Receiver" : "Speaker"
#define REG_VALUE(name, data)   (((data) & name ## _MASK) >> name ## _SHIFT)

#define SOUND_I2C_CHANNEL  2

#define TPA2028_EAMP_I2C_DEVNAME "TPA2028D"
#define TPA2028D_ADDRESS (0xB0)

#define AGC_COMPRESIION_RATE        0
#define AGC_OUTPUT_LIMITER_DISABLE  1
//                                                                                     
#define AGC_FIXED_GAIN_AUDIO        0x12 // 18dB
#define AGC_FIXED_GAIN_VOICE        0x12 // 18dB
//                                                                                                         
#define AGC_FIXED_GAIN_COMBO_RING   0x1C // 28dB
#define DEVICE_OUT_EARPIECER 0
#define DEVICE_OUT_HEADSETR 2
#define DEVICE_OUT_SPEAKERR 4
#define DEVICE_OUT_SPEAKER_HEADSET_R 6
char agc_fixed_gain = 0;
static u32 set_device = 0;
//                                                                                                       
//                                                                                   


static struct i2c_client *new_client = NULL;
/*                                               */
char proc_str[128];
int register_setting_mode = 0;
/*                                               */

static const struct i2c_device_id tpa2028_eamp_i2c_id[] = {
    {TPA2028_EAMP_I2C_DEVNAME,0},
    {}
};

static struct i2c_board_info __initdata tpa2028_eamp_dev = {
    I2C_BOARD_INFO(TPA2028_EAMP_I2C_DEVNAME,(TPA2028D_ADDRESS>>1))
};


//extern hw_rev_type lge_get_board_revno(void);

static int tpa2028d_powerdown(void);
static void set_amp_gain(int amp_state);

static int tpa2028d_poweron(void);

static int tpa2028_eamp_i2c_probe(struct i2c_client *client,
                                const struct i2c_device_id *id);
static int tpa2028_eamp_i2c_remove(struct i2c_client *client);

int gsk_on;
int ghp_on;
int gep_on;
//                                                                                      
static u32 set_mode = 0;
//                                                                                    

struct i2c_driver tpa2028_eamp_i2c_driver = {
    .probe = tpa2028_eamp_i2c_probe,
    .remove = tpa2028_eamp_i2c_remove,
    //.detect = eamp_i2c_detect,
    .driver = {
        .name =TPA2028_EAMP_I2C_DEVNAME,
    },
    .id_table = tpa2028_eamp_i2c_id,
    //.address_data = &addr_data,
};


ssize_t static eamp_read_byte(u8 addr, u8 *returnData)
{
    char     cmd_buf[1]={0x00};
    char     readData = 0;
    int     ret=0;

    if(!new_client)
    {
        EAMP_PRINTK("I2C client not initialized!!");
        return -1;
    }

    cmd_buf[0] = addr;
    ret = i2c_master_send(new_client, &cmd_buf[0], 1);
    if (ret < 0) {
        EAMP_PRINTK("read sends command error!!");
        return -1;
    }
    ret = i2c_master_recv(new_client, &readData, 1);
    if (ret < 0) {
        EAMP_PRINTK("reads recv data error!!");
        return -1;
    }
    *returnData = readData;
    EAMP_PRINTK("addr 0x%x data 0x%x",addr, readData);
    return 0;
}

static u8 I2CRead(u8 addr)
{
    u8 regvalue;
    eamp_read_byte(addr,&regvalue);
    return regvalue;
}

static ssize_t    eamp_write_byte(u8 addr, u8 writeData)
{
    char    write_data[2] = {0};
    int    ret=0;

    if(!new_client)
    {
        EAMP_PRINTK("I2C client not initialized!!");
        return -1;
    }

    write_data[0] = addr;          // ex. 0x01
    write_data[1] = writeData;
    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0) {
        EAMP_PRINTK("write sends command error!!");
        return -1;
    }
    EAMP_PRINTK("addr 0x%x data 0x%x",addr,writeData);
    return 0;
}


static ssize_t I2CWrite(u8 addr, u8 writeData)
{
    char    write_data[2] = {0};
    int    ret=0;

    if(!new_client)
    {
        EAMP_PRINTK("I2C client not initialized!!");
        return -1;
    }

    write_data[0] = addr;          // ex. 0x01
    write_data[1] = writeData;
    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0) {
        EAMP_PRINTK("write sends command error!! ret=%d", ret);
        EAMP_PRINTK("addr 0x%x data 0x%x",addr,writeData);
        return -1;
    }
    EAMP_PRINTK("addr 0x%x data 0x%x",addr,writeData);
    return 0;
}

static ssize_t eamp_openEarpiece()
{
    EAMP_PRINTK("");
    gep_on = true;
    return 0;
}

static ssize_t eamp_closeEarpiece()
{
    EAMP_PRINTK("");
    gep_on = false;
    return 0;
}

static ssize_t eamp_openheadPhone()
{
	mt_set_gpio_out(GPIO_HP_AMP_EN, GPIO_OUT_ONE);
	EAMP_PRINTK("eamp_openheadphone : set GPIO_HP_AMP_EN to %d", mt_get_gpio_out(GPIO_HP_AMP_EN));
	ghp_on = true;
    return 0;
}

static ssize_t eamp_closeheadPhone()
{
	mt_set_gpio_out(GPIO_HP_AMP_EN, GPIO_OUT_ZERO);
	EAMP_PRINTK("eamp_closeheadphone : set GPIO_HP_AMP_EN to %d", mt_get_gpio_out(GPIO_HP_AMP_EN));	
	ghp_on = false;
  	return 0;
}

static ssize_t eamp_openspeaker()
{
    EAMP_PRINTK("");
	set_amp_gain(SPK_ON);
	gsk_on = true;
    return 0;
}

static ssize_t eamp_closespeaker()
{
    EAMP_PRINTK("");
    set_amp_gain(SPK_OFF);
    gsk_on = false;
    return 0;
}

static ssize_t eamp_changeGainVolume(unsigned long int param)
{
    EAMP_PRINTK("param(0x%x)",param);
    return 0;
}

static ssize_t eamp_getGainVolume(void)
{
    EAMP_PRINTK("");
    return 0;
}

static ssize_t eamp_resetRegister()
{
	set_amp_gain(SPK_OFF);
	eamp_closeheadPhone();
	return 0;
}

static ssize_t eamp_suspend()
{
    EAMP_PRINTK("");
    eamp_resetRegister();
    return 0;
}

static ssize_t eamp_resume()
{
    EAMP_PRINTK("");
    if(gsk_on)
    {
        eamp_openspeaker();
    }
    if(ghp_on)
    {
        eamp_openheadPhone();
    }
    if(gep_on)
    {
        eamp_openEarpiece();
    }
    return 0;
}

static ssize_t eamp_getRegister(unsigned int regName)
{
    EAMP_PRINTK("Regname=%u",regName);
    if(regName >7)
        return -1;
    return I2CRead(regName);
}

static ssize_t eamp_setRegister(unsigned long int param)
{
    AMP_Control * p = (AMP_Control*)param;

    EAMP_PRINTK("");

    if(p->param1 >7)
        return -1;

    return I2CWrite(p->param1,p->param2);
}

static ssize_t eamp_setMode(unsigned long int param)
{
    EAMP_PRINTK("mode(%u)",param);
//                                                                                     
    set_mode = param;
//                                                                                   
    return 0;
}

//                                                                                                         
static ssize_t eamp_setDevice(unsigned long int param)
{
    EAMP_PRINTK("set Device (%u)", param);
    set_device = param;
	if (set_mode == 2) {
		agc_fixed_gain = AGC_FIXED_GAIN_VOICE;
		I2CWrite(AGC_FIXED_GAIN_CONTROL, register_setting_mode ? register_setting_mode:agc_fixed_gain);
		EAMP_PRINTK("eamp_setDevice : set agc gain VOICE = 0x%x\n", agc_fixed_gain);
	} else if (set_device == DEVICE_OUT_SPEAKER_HEADSET_R) {
		agc_fixed_gain = AGC_FIXED_GAIN_COMBO_RING;
		I2CWrite(AGC_FIXED_GAIN_CONTROL, register_setting_mode ? register_setting_mode:agc_fixed_gain);
		EAMP_PRINTK("eamp_setDevice : set agc gain COMBO_PATH = 0x%x\n", agc_fixed_gain);
	} else {
		agc_fixed_gain = AGC_FIXED_GAIN_AUDIO;
		I2CWrite(AGC_FIXED_GAIN_CONTROL, register_setting_mode ? register_setting_mode:agc_fixed_gain);
		EAMP_PRINTK("eamp_setDevice : set agc gain AUDIO = 0x%x\n", agc_fixed_gain);
	}
    return 0;
}
//                                                                                                       

static int eamp_command( unsigned int  type, unsigned long args,unsigned int count)
{
    EAMP_PRINTK("type(%u) test22",type);
    switch(type)
    {
        case EAMP_SPEAKER_CLOSE:
        {
            eamp_closespeaker();
            break;
        }
        case EAMP_SPEAKER_OPEN:
        {
            eamp_openspeaker();
            break;
        }
        case EAMP_HEADPHONE_CLOSE:
        {
            eamp_closeheadPhone();
            break;
        }
        case EAMP_HEADPHONE_OPEN:
        {
            eamp_openheadPhone();
            break;
        }
        case EAMP_EARPIECE_OPEN:
        {
            eamp_openEarpiece();
            break;
        }
        case EAMP_EARPIECE_CLOSE:
        {
            eamp_closeEarpiece();
            break;
        }
        case EAMP_GETREGISTER_VALUE:
        {
            return eamp_getRegister(args);
            break;
        }
        case EAMP_GETAMP_GAIN:
        {
            return eamp_getGainVolume();
            break;
        }
        case EAMP_SETAMP_GAIN:
        {
            eamp_changeGainVolume(args);
            break;
        }
        case EAMP_SETREGISTER_VALUE:
        {
            eamp_setRegister(args);
            break;
        }
//                                                                                     
        case EAMP_SETMODE:
        {
            eamp_setMode(args);
            break;
        }
//                                                                                   
//                                                                                                         
        case EAMP_SETDEVICE:
		{
			eamp_setDevice(args);
			break;
		}
//                                                                                                       
        default:
        {
        	EAMP_PRINTK("Not support command=%d", type);
        	return 0;
        }
    }
    return 0;
}

int Audio_eamp_command_tpa2028d(unsigned int type, unsigned long args, unsigned int count)
{
    return eamp_command(type,args,count);
}

#if 0
static int eamp_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
    strcpy(info->type, EAMP_I2C_DEVNAME);
    return 0;
}
#endif

static int tpa2028_eamp_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    int ret = 0;
    EAMP_PRINTK("tpa2028_eamp_i2c_probe success");

    new_client = client;
    ret = eamp_resetRegister();
    EAMP_PRINTK("client=%x !!",client);

    if(ret < 0) {
        new_client = NULL;
        EAMP_PRINTK("[ERROR] maybe, chip is died by .. !!",client);
        return -1;
    }else {
    	return 0;
	}
}

static int tpa2028_eamp_i2c_remove(struct i2c_client *client)
{
    EAMP_PRINTK("");
    new_client = NULL;
    i2c_unregister_device(client);
    i2c_del_driver(&tpa2028_eamp_i2c_driver);
    return 0;
}

static void eamp_poweron(void)
{
    return;
}

static void eamp_powerdown(void)
{
    EAMP_PRINTK("");
    eamp_closeheadPhone();
    eamp_closespeaker();
    return;
}


static int eamp_init()
{
	int result = 0;
	result = mt_set_gpio_mode(GPIO_HP_AMP_EN, GPIO_HP_AMP_EN_M_GPIO);
   	EAMP_PRINTK("GPIO_HP_AMP_EN GPIO Status : mt_set_gpio_mode %d\n", result);
   	result = mt_set_gpio_pull_enable(GPIO_HP_AMP_EN, GPIO_PULL_DISABLE);
   	EAMP_PRINTK("GPIO_HP_AMP_EN GPIO Status : mt_set_gpio_pull_enable %d\n", result);
   	result = mt_set_gpio_dir(GPIO_HP_AMP_EN, GPIO_DIR_OUT);
   	EAMP_PRINTK("GPIO_HP_AMP_EN GPIO Status : mt_set_gpio_dir %d\n", result);
   	result = mt_set_gpio_out(GPIO_HP_AMP_EN, GPIO_OUT_ONE);
   	EAMP_PRINTK("GPIO_HP_AMP_EN GPIO Status : mt_set_gpio_out %d, %d\n", result, mt_get_gpio_out(GPIO_HP_AMP_EN));
   	result = mt_set_gpio_mode(GPIO_SPK_AMP_EN, GPIO_SPK_AMP_EN_M_GPIO);
   	EAMP_PRINTK("GPIO_SPK_AMP_EN GPIO Status : mt_set_gpio_mode %d\n", result);
   	result = mt_set_gpio_pull_enable(GPIO_SPK_AMP_EN, GPIO_PULL_DISABLE);
   	EAMP_PRINTK("GPIO_SPK_AMP_EN GPIO Status : mt_set_gpio_pull_enable %d\n", result);
   	result = mt_set_gpio_dir(GPIO_SPK_AMP_EN, GPIO_DIR_OUT);
   	EAMP_PRINTK("GPIO_SPK_AMP_EN GPIO Status : mt_set_gpio_dir %d\n", result);
   	result = mt_set_gpio_out(GPIO_SPK_AMP_EN, GPIO_OUT_ONE);
   	EAMP_PRINTK("GPIO_SPK_AMP_EN GPIO Status : mt_set_gpio_out %d, %d\n", result, mt_get_gpio_out(GPIO_SPK_AMP_EN));
   	eamp_poweron();
   	eamp_powerdown();
   	return 0;
}

static int eamp_deinit()
{
    EAMP_PRINTK("");
    eamp_powerdown();
    return 0;
}

static int eamp_register()
{
    EAMP_PRINTK("");
    i2c_register_board_info(SOUND_I2C_CHANNEL,&tpa2028_eamp_dev,1);
    if (i2c_add_driver(&tpa2028_eamp_i2c_driver)){
        EAMP_PRINTK("fail to add device into i2c");
        return -1;
    }
    return 0;
}


bool Speaker_Register_tpa2028d(void)
{
    EAMP_PRINTK("");
    eamp_register();
    return true;
}

static int amp_enable(int on_state)
{
	int err = 0;

	switch (on_state) {
	case 0:
		err = mt_set_gpio_out(GPIO_SPK_AMP_EN, GPIO_OUT_ZERO);
		EAMP_PRINTK("AMP_EN is set to %d\n", mt_get_gpio_out(GPIO_SPK_AMP_EN));
		break;
	case 1:
		err = mt_set_gpio_out(GPIO_SPK_AMP_EN, GPIO_OUT_ONE);
		EAMP_PRINTK("AMP_EN is set to %d\n", mt_get_gpio_out(GPIO_SPK_AMP_EN));
		break;
	case 2:
		EAMP_PRINTK("amp enable bypass(%d)\n", on_state);
		err = 0;
		break;

	default:
		pr_err("amp enable fail\n");
		err = 1;
		break;
	}
	return err;
}


static int tpa2028d_poweron(void)
{
	int fail = 0;
	char agc_compression_rate = AGC_COMPRESIION_RATE;
	char agc_output_limiter_disable = AGC_OUTPUT_LIMITER_DISABLE;

	agc_fixed_gain = AGC_FIXED_GAIN_AUDIO;
	agc_output_limiter_disable = (agc_output_limiter_disable<<7);

	fail |= I2CWrite(IC_CONTROL, 0xE3); /*Tuen On*/
	fail |= I2CWrite(AGC_ATTACK_CONTROL, 0x05); /*Tuen On*/
	fail |= I2CWrite(AGC_RELEASE_CONTROL, 0x0B); /*Tuen On*/
	fail |= I2CWrite(AGC_HOLD_TIME_CONTROL, 0x00); /*Tuen On*/
/*                                               */
	fail |= I2CWrite(AGC_FIXED_GAIN_CONTROL, register_setting_mode ? register_setting_mode:agc_fixed_gain); /*Tuen On*/
/*                                               */
	fail |= I2CWrite(AGC1_CONTROL, 0x3A|agc_output_limiter_disable); /*Tuen On*/
	fail |= I2CWrite(AGC2_CONTROL, 0xC0|agc_compression_rate); /*Tuen On*/
	fail |= I2CWrite(IC_CONTROL, 0xC3); /*Tuen On*/

	return fail;
}

static int tpa2028d_powerdown(void)
{
	int fail = 0;
	EAMP_PRINTK("");
	return fail;
}

static void set_amp_gain(int amp_state)
{
	int fail = 0;
	EAMP_PRINTK("amp_state=%d",amp_state);
	switch (amp_state) {
	case SPK_ON:
		msleep(5);
		fail = amp_enable(1);
		/*need 10 msec for chip ready*/
		msleep(100);
		fail = tpa2028d_poweron();
		break;
	case SPK_OFF:
		amp_enable(2);
		fail = tpa2028d_powerdown();
		fail = amp_enable(0);
		break;
	default:
		printk("Amp_state [%d] does not support \n", amp_state);
	}
}

/*                                               */
static int eamp_write_procmem(struct file *file, const char __user *buffer,
	unsigned long count, void *data)
{
	char *register_data;
	int addr;
	int value;
	int fail;

	register_data = (char*)data;
	copy_from_user(register_data, buffer, count);
	register_data[count] = '\0';
	if(register_data[count - 1] == '\n')
			register_data[count - 1] = '\0';

	sscanf(register_data, "%d %d", &addr, &value);
	EAMP_PRINTK("S register_data=%s %d %d\n", register_data, addr, value);
	if(addr >= IC_CONTROL && addr <= AGC2_CONTROL){
		if(AGC_FIXED_GAIN_CONTROL == addr){
			if(value >= -28 && value <= 30)
				register_setting_mode = value;
			else{
				EAMP_PRINTK("S Wrong value\n");
				return count;
			}

		}
		fail |= I2CWrite(addr, value);
		EAMP_PRINTK("S register Write Done Gain=%d\n", I2CRead(addr));
	}
	else
		EAMP_PRINTK("S Wrong addr or value\n");

	return count;
}
/*                                               */

static int eamp_read_procmem(char *buf, char **start, off_t offset,
        int count , int *eof, void *data)
{
    int len = 0;
    u8 val;
    EAMP_PRINTK("S\n");

    len += sprintf(buf + len , "SPK Amp Enable GPIO (%d) = %s\n", mt_get_gpio_out(GPIO_SPK_AMP_EN), ON_OFF_STR(mt_get_gpio_out(GPIO_SPK_AMP_EN)));
    len += sprintf(buf + len , "HPH Amp Enable GPIO (%d) = %s\n", mt_get_gpio_out(GPIO_HP_AMP_EN), ON_OFF_STR(mt_get_gpio_out(GPIO_HP_AMP_EN)));
    
    if (gep_on == true) {
    	len += sprintf(buf + len , "gep_on is %d, RCV ON\n", gep_on);
    } else {
    	len += sprintf(buf + len , "gep_on is %d, RCV OFF\n", gep_on);
    }

    eamp_read_byte(IC_CONTROL, &val);
    len += sprintf(buf + len , "IC_CONTROL = 0x%x\n", val);
    
    eamp_read_byte(AGC_ATTACK_CONTROL, &val);
    len += sprintf(buf + len , "AGC_ATTACK_CONTROL = 0x%x\n", val);
    
    eamp_read_byte(AGC_RELEASE_CONTROL, &val);
    len += sprintf(buf + len , "AGC_RELEASE_CONTROL = 0x%x\n", val);
    
    eamp_read_byte(AGC_HOLD_TIME_CONTROL, &val);
    len += sprintf(buf + len , "AGC_HOLD_TIME_CONTROL = 0x%x\n", val);
    
    eamp_read_byte(AGC_FIXED_GAIN_CONTROL, &val);
    len += sprintf(buf + len , "AGC_FIXED_GAIN_CONTROL = 0x%x\n", val);
    
    eamp_read_byte(AGC1_CONTROL, &val);
    len += sprintf(buf + len , "AGC1_CONTROL = 0x%x\n", val);
    
    eamp_read_byte(AGC2_CONTROL, &val);
    len += sprintf(buf + len , "AGC2_CONTROL = 0x%x\n", val);

    return len;
}

static ssize_t eamp_getCtrlPointNum()
{
    EAMP_PRINTK("");
    return 0;
}

static ssize_t eamp_getCtrPointBits(unsigned long int param)
{
    EAMP_PRINTK("CtrPointBits(%u)",param);
    return 0;
}

static ssize_t eamp_getCtrlPointTable(unsigned long int param)
{
    EAMP_PRINTK("CtrlPointTable(0x%x)",param);
    return 0;
}

//input mode and volume control Register
static ssize_t eamp_clear_input_gain()
{
    EAMP_PRINTK("");
    return 0;
}

// set input gain on channel 1 and 2
static ssize_t eamp_set_input_gain( u8 inGain)
{
    EAMP_PRINTK("inGain(0x%x)",inGain);
    return 0;
}

//  set input mode on channel 1 and 2.  0 for single-end inputs, 1 for differential inputs.
static ssize_t eamp_set_input_mode( bool in1se, bool in2se)
{
    return 0;
}

static ssize_t eamp_Release_attackTime_speed(u8 ATK_time, u8 REL_time)
{
    return 0;
}

static ssize_t eamp_set_speakerLimiter_level(u8 limitlev )
{
    return 0;
}

// speaker limiter enable. 1 enable, 0 disable.
static ssize_t eamp_speakerLimiter_enable(bool enable )
{
    return 0;
}

// control for speaker channel.
static ssize_t eamp_set_speakerOut(u8  speakerout )
{
    return 0;
}

//Headphone mux and limiter
// set headphone limiter level
static ssize_t eamp_set_headPhoneLimiter_level(u8 limitlev )
{
    return 0;
}

// enable /disable headphone limiter
static ssize_t eamp_headPhoneLimiter_enable(bool enable )
{
    return 0;
}

// control for headphone channel
static ssize_t eamp_set_headPhoneOut(u8  headphoneout )
{
    return 0;
}

//speaker volume
// openspeaker
static ssize_t eamp_set_speaker_Open(bool enable)
{
    EAMP_PRINTK("enable=%d",enable);
    return 0;
}

//set  speaker volume
static ssize_t eamp_set_speaker_vol(u8    vol)
{
    EAMP_PRINTK("vol=0x%x",vol);
    return 0;
}

//Headphone left channel volume
//enable headphone left  channel
static ssize_t eamp_set_headPhoneL_open( bool  enable )
{
    EAMP_PRINTK("enable=%d",enable);
    return 0;
}

//set  headphone  volume
static ssize_t eamp_set_headPhone_vol(u8 HP_vol)
{
    EAMP_PRINTK("vol=0x%x",HP_vol);
    return 0;
}

//set  headphone left  volume
static ssize_t eamp_set_headPhone_lvol(u8 HPL_Vol)
{
    EAMP_PRINTK("vol=0x%x",HPL_Vol);
    return 0;
}

//Headphone right channel volume  register
//enable headphone Right  channel
static ssize_t eamp_set_headPhoneR_open( bool  enable )
{
    EAMP_PRINTK("enable=%d",enable);
    return 0;
}

//set  headphone right volume
static ssize_t eamp_set_headPhone_rvol(u8 HPR_Vol)
{
    EAMP_PRINTK("vol=0x%x",HPR_Vol);
    return 0;
}

bool Speaker_Init_tpa2028d(void)
{
    EAMP_PRINTK("");
    eamp_init();

/*                                               */
	//create_proc_read_write_entry("audio_ext_amp", 0644, NULL,
	//	eamp_write_procmem, eamp_read_procmem, proc_str);
/*                                               */

    return true;
}


