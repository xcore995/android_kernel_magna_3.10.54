#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <mach/mt6516_pll.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "hi707yuv_Sensor.h"
#include "hi707yuv_Camera_Sensor_para.h"
#include "hi707yuv_CameraCustomized.h"

#define HI707YUV_DEBUG
#ifdef HI707YUV_DEBUG
#define SENSORDB printk
#else

#define SENSORDB(x,...)
#endif
#define HI707_TEST_PATTERN_CHECKSUM (0x786a9657)//(0x7ba87eae)

#if 0
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
static int sensor_id_fail = 0;
#define HI707_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para ,1,HI707_WRITE_ID)
#define HI707_write_cmos_sensor_2(addr, para, bytes) iWriteReg((u16) addr , (u32) para ,bytes,HI707_WRITE_ID)
kal_uint16 HI707_read_cmos_sensor(kal_uint32 addr)
{
kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,HI707_WRITE_ID);
    return get_byte;
}

#endif
typedef enum {
  HI707_60HZ,
  HI707_50HZ,
  HI707_HZ_MAX_NUM,
} HI707AntibandingType;
static DEFINE_SPINLOCK(hi707_yuv_drv_lock);
static MSDK_SCENARIO_ID_ENUM CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_ZSD;
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId);


UINT8 HI707SetBrightness_value = 0x50;
UINT8 HI707_set_param_exposure_value = 0x48;

//                                                                   
UINT16 ExposureTime_High;
UINT16 ExposureTime_Mid;
UINT16 ExposureTime_Low;
UINT16 ExposureTime = 0;
UINT16 ISO_RegVal = 0;
//                                                                   

//for burst mode

#define USE_I2C_BURST_WRITE
#ifdef USE_I2C_BURST_WRITE
#define I2C_BUFFER_LEN 254 //MAX data to send by MT6572 i2c dma mode is 255 bytes
#define BLOCK_I2C_DATA_WRITE iBurstWriteReg
#else
#define I2C_BUFFER_LEN 8   // MT6572 i2s bus master fifo length is 8 bytes
#define BLOCK_I2C_DATA_WRITE iWriteRegI2C
#endif

// {addr, data} pair in para
// len is the total length of addr+data
// Using I2C multiple/burst write if the addr increase by 1
static kal_uint16 HI707_table_write_cmos_sensor(kal_uint8* para, kal_uint32 len)
{
   kal_uint8 puSendCmd[I2C_BUFFER_LEN]={0,};
   kal_uint32 tosend=0 , IDX=0;
   kal_uint8 addr, addr_next, data;
   while(IDX < len)
   {
       addr = para[IDX];
       if (tosend == 0) // new (addr, data) to send
       {
           puSendCmd[tosend++] = (kal_uint8)addr;
           data = para[IDX+1];
		   puSendCmd[tosend++] = (kal_uint8)data;
           addr_next =  addr+1;
           IDX += 2;
       }
       else if (addr == addr_next) // to multiple write the data to the incremental address
       {
           data = para[IDX+1];
		   puSendCmd[tosend++] = (kal_uint8)data;
           addr_next =  addr+1;
           IDX += 2;
       }
       else // to send out the data if the address not incremental.
       {
           BLOCK_I2C_DATA_WRITE(puSendCmd , tosend, HI707_WRITE_ID);
           tosend = 0;
       }

       // to send out the data if the sen buffer is full or last data.
       if ((tosend >= (I2C_BUFFER_LEN-8)) || (IDX == len))
       {
           BLOCK_I2C_DATA_WRITE(puSendCmd , tosend, HI707_WRITE_ID);
           tosend = 0;
       }
   }
   return 0;
}
//


kal_uint16 HI707_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
    char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
    iWriteRegI2C(puSendCmd , 2,HI707_WRITE_ID);
    return 0;
}
kal_uint16 HI707_read_cmos_sensor(kal_uint8 addr)
{
    kal_uint16 get_byte=0;
    char puSendCmd = { (char)(addr & 0xFF) };
    iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte,1,HI707_WRITE_ID);
    return get_byte;
}


/*******************************************************************************
* // Adapter for Winmo typedef
********************************************************************************/
#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT


/*******************************************************************************
* follow is define by jun
********************************************************************************/
MSDK_SENSOR_CONFIG_STRUCT HI707SensorConfigData;

static struct HI707_sensor_STRUCT HI707_sensor;
static kal_uint32 HI707_zoom_factor = 0;
static int sensor_id_fail = 0;

static void HI707_Initial_Setting(void)
{
	kal_uint32 len;

	////////////////////////////////////////////
	/////////////////////////////// Hi-707 Setting
	////////////////////////////////////////////

	/////////////////////////////////I2C_ID = 0x60
	////////////////////////////I2C_BYTE  = 0x11
 printk("%s [tmpdbg]", __func__);
	len = sizeof(HI707_Init_Reg[HI707_sensor.banding])/sizeof(HI707_Init_Reg[HI707_sensor.banding][0]);
	HI707_table_write_cmos_sensor(HI707_Init_Reg[HI707_sensor.banding],len);

}

//                                                                            
extern int soc_antibanding;
//                                                                            

static void HI707_Init_Parameter(void)
{
    spin_lock(&hi707_yuv_drv_lock);
    HI707_sensor.first_init = KAL_TRUE;
    HI707_sensor.Sensor_mode= SENSOR_MODE_PREVIEW;
    HI707_sensor.night_mode = KAL_FALSE;
    HI707_sensor.MPEG4_Video_mode = KAL_FALSE;

    HI707_sensor.cp_pclk = HI707_sensor.pv_pclk;

    HI707_sensor.pv_dummy_pixels = 0;
    HI707_sensor.pv_dummy_lines = 0;
    HI707_sensor.cp_dummy_pixels = 0;
    HI707_sensor.cp_dummy_lines = 0;

    HI707_sensor.wb = 0;
    HI707_sensor.exposure = 0;
    HI707_sensor.effect = 0;
//                                                                            
    HI707_sensor.banding = soc_antibanding;
//                                                                            
    HI707_sensor.video_fps = 30;

    HI707_sensor.pv_line_length = 640;
    HI707_sensor.pv_frame_height = 480;
    HI707_sensor.cp_line_length = 640;
    HI707_sensor.cp_frame_height = 480;
    spin_unlock(&hi707_yuv_drv_lock);
}

static kal_uint8 HI707_power_on(void)
{
    kal_uint8 HI707_sensor_id = 0;
    spin_lock(&hi707_yuv_drv_lock);
    HI707_sensor.pv_pclk = 13000000;
    spin_unlock(&hi707_yuv_drv_lock);
    //Software Reset
    HI707_write_cmos_sensor(0x01,0xf1);
    HI707_write_cmos_sensor(0x01,0xf3);
    HI707_write_cmos_sensor(0x01,0xf1);

    /* Read Sensor ID  */
    HI707_sensor_id = HI707_read_cmos_sensor(0x04);
    SENSORDB("[HI707YUV]:read Sensor ID:%x\n",HI707_sensor_id);
    //HI707_sensor_id = HI707_SENSOR_ID;
    return HI707_sensor_id;
}


/*************************************************************************
* FUNCTION
*    HI707Open
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HI707Open(void)
{
    spin_lock(&hi707_yuv_drv_lock);
    sensor_id_fail = 0;
    spin_unlock(&hi707_yuv_drv_lock);
    SENSORDB("[Enter]: [tmpdbg] HI707 Open func:");

    if (HI707_power_on() != HI707_SENSOR_ID)
    {
        SENSORDB("[HI707]Error:read sensor ID fail\n");
        spin_lock(&hi707_yuv_drv_lock);
        sensor_id_fail = 1;
        spin_unlock(&hi707_yuv_drv_lock);
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    /* Apply sensor initail setting*/
    HI707_Init_Parameter();
    HI707_Initial_Setting();
    SENSORDB("[HI707]:antibanding = %d \n", HI707_sensor.banding); // 0 => 60hz , 1 => 50hz 

    return ERROR_NONE;
}    /* HI707Open() */

/*************************************************************************
* FUNCTION
*    HI707_GetSensorID
*
* DESCRIPTION
*    This function get the sensor ID
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 HI707_GetSensorID(kal_uint32 *sensorID)
{
    SENSORDB("[Enter]:HI707 Open func ");
    *sensorID = HI707_power_on() ;

    if (*sensorID != HI707_SENSOR_ID)
    {
        SENSORDB("[HI707]Error:read sensor ID fail\n");
        spin_lock(&hi707_yuv_drv_lock);
        sensor_id_fail = 1;
        spin_unlock(&hi707_yuv_drv_lock);
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    return ERROR_NONE;
}   /* HI707Open  */


/*************************************************************************
* FUNCTION
*    HI707Close
*
* DESCRIPTION
*    This function is to turn off sensor module power.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HI707Close(void)
{

    return ERROR_NONE;
}    /* HI707Close() */



static void HI707_Set_Mirror_Flip(kal_uint8 image_mirror)
{
    /********************************************************
    * Page Mode 0: Reg 0x0011 bit[1:0] = [Y Flip : X Flip]
    * 0: Off; 1: On.
    *********************************************************/
    kal_uint8 temp_data;
    SENSORDB("[Enter]:HI707 set Mirror_flip func:image_mirror=%d\n",image_mirror);
    HI707_write_cmos_sensor(0x03,0x00);     //Page 0
    temp_data = (HI707_read_cmos_sensor(0x11) & 0xfc);
    //HI707_sensor.mirror = (HI707_read_cmos_sensor(0x11) & 0xfc);
    switch (image_mirror)
    {
    case IMAGE_NORMAL:
        //HI707_sensor.mirror |= 0x00;
        temp_data |= 0x00;
        break;
    case IMAGE_H_MIRROR:
        //HI707_sensor.mirror |= 0x01;
        temp_data |= 0x01;
        break;
    case IMAGE_V_MIRROR:
        //HI707_sensor.mirror |= 0x02;
        temp_data |= 0x02;
        break;
    case IMAGE_HV_MIRROR:
        //HI707_sensor.mirror |= 0x03;
        temp_data |= 0x03;
        break;
    default:
        //HI707_sensor.mirror |= 0x00;
        temp_data |= 0x00;
    }
    spin_lock(&hi707_yuv_drv_lock);
    HI707_sensor.mirror = temp_data;
    spin_unlock(&hi707_yuv_drv_lock);
    HI707_write_cmos_sensor(0x11, HI707_sensor.mirror);
    SENSORDB("[Exit]:HI707 set Mirror_flip func\n");
}

#if 0
static void HI707_set_dummy(kal_uint16 dummy_pixels,kal_uint16 dummy_lines)
{
    HI707_write_cmos_sensor(0x03, 0x00);                        //Page 0
    HI707_write_cmos_sensor(0x40,((dummy_pixels & 0x0F00))>>8);       //HBLANK
    HI707_write_cmos_sensor(0x41,(dummy_pixels & 0xFF));
    HI707_write_cmos_sensor(0x42,((dummy_lines & 0xFF00)>>8));       //VBLANK ( Vsync Type 1)
    HI707_write_cmos_sensor(0x43,(dummy_lines & 0xFF));
}
#endif

// 640 * 480


static void HI707_Cal_Min_Frame_Rate(kal_uint16 min_framerate)
{
    kal_uint32 HI707_expmax = 0;
    kal_uint32 HI707_expbanding = 0;
    kal_uint32 temp_data;


    SENSORDB("[HI707] HI707_Cal_Min_Frame_Rate:min_fps=%d\n",min_framerate);

    //No Fixed Framerate
    HI707_write_cmos_sensor(0x01, HI707_read_cmos_sensor(0x01)|0x01);   //Sleep: For Write Reg
    HI707_write_cmos_sensor(0x03, 0x00);
    HI707_write_cmos_sensor(0x11, HI707_read_cmos_sensor(0x11)&0xfb);

    HI707_write_cmos_sensor(0x03, 0x20);
    HI707_write_cmos_sensor(0x10, HI707_read_cmos_sensor(0x10)&0x7f);   //Close AE

    HI707_write_cmos_sensor(0x11, 0x04);
    HI707_write_cmos_sensor(0x18, HI707_read_cmos_sensor(0x18)|0x08);   //Reset AE
    HI707_write_cmos_sensor(0x2a, 0xf0);
    HI707_write_cmos_sensor(0x2b, 0x34);

    HI707_write_cmos_sensor(0x03, 0x00);
    temp_data = ((HI707_read_cmos_sensor(0x40)<<8)|HI707_read_cmos_sensor(0x41));
    spin_lock(&hi707_yuv_drv_lock);
    HI707_sensor.pv_dummy_pixels = temp_data;
    HI707_sensor.pv_line_length = HI707_VGA_DEFAULT_PIXEL_NUMS+ HI707_sensor.pv_dummy_pixels ;
    spin_unlock(&hi707_yuv_drv_lock);

    if(HI707_sensor.banding == HI707_50HZ)
    {
        HI707_expbanding = (HI707_sensor.pv_pclk/HI707_sensor.pv_line_length/100)*HI707_sensor.pv_line_length/8;
        HI707_expmax = HI707_expbanding*100*10/min_framerate ;
	    HI707_write_cmos_sensor(0x03, 0x20);
	    HI707_write_cmos_sensor(0xa0, (HI707_expmax>>16)&0xff);
	    HI707_write_cmos_sensor(0xa1, (HI707_expmax>>8)&0xff);
	    HI707_write_cmos_sensor(0xa2, (HI707_expmax>>0)&0xff);
    }
    else if(HI707_sensor.banding == HI707_60HZ)
    {
        HI707_expbanding = (HI707_sensor.pv_pclk/HI707_sensor.pv_line_length/120)*HI707_sensor.pv_line_length/8;
        HI707_expmax = HI707_expbanding*120*10/min_framerate ;
	    HI707_write_cmos_sensor(0x03, 0x20);
	    HI707_write_cmos_sensor(0x88, (HI707_expmax>>16)&0xff);
	    HI707_write_cmos_sensor(0x89, (HI707_expmax>>8)&0xff);
	    HI707_write_cmos_sensor(0x8a, (HI707_expmax>>0)&0xff);
    }
    else//default 5oHZ
    {
        //SENSORDB("[HI707][Error] Wrong Banding Setting!!!...");
        HI707_expbanding = (HI707_sensor.pv_pclk/HI707_sensor.pv_line_length/100)*HI707_sensor.pv_line_length/8;
        HI707_expmax = HI707_expbanding*100*10/min_framerate ;
	    HI707_write_cmos_sensor(0x03, 0x20);
	    HI707_write_cmos_sensor(0xa0, (HI707_expmax>>16)&0xff);
	    HI707_write_cmos_sensor(0xa1, (HI707_expmax>>8)&0xff);
	    HI707_write_cmos_sensor(0xa2, (HI707_expmax>>0)&0xff);

	}


    HI707_write_cmos_sensor(0x01, HI707_read_cmos_sensor(0x01)&0xfe);   //Exit Sleep: For Write Reg

    HI707_write_cmos_sensor(0x03, 0x20);
    HI707_write_cmos_sensor(0x10, HI707_read_cmos_sensor(0x10)|0x80);   //Open AE
    HI707_write_cmos_sensor(0x18, HI707_read_cmos_sensor(0x18)&0xf7);   //Reset AE
}


static void HI707_Fix_Video_Frame_Rate(kal_uint16 fix_framerate)
{
    kal_uint32 HI707_expfix;
    kal_uint32 HI707_expfix_temp;
    kal_uint32 HI707_expmax = 0;
    kal_uint32 HI707_expbanding = 0;
    kal_uint32 temp_data1,temp_data2;

    SENSORDB("[Enter]HI707 Fix_video_frame_rate func: fix_fps=%d\n",fix_framerate);

    spin_lock(&hi707_yuv_drv_lock);
    HI707_sensor.video_current_frame_rate = fix_framerate;
    spin_unlock(&hi707_yuv_drv_lock);
    // Fixed Framerate
    HI707_write_cmos_sensor(0x01, HI707_read_cmos_sensor(0x01)|0x01);   //Sleep: For Write Reg

    HI707_write_cmos_sensor(0x03, 0x00);
    HI707_write_cmos_sensor(0x11, HI707_read_cmos_sensor(0x11)|0x04);

    HI707_write_cmos_sensor(0x03, 0x20);
    HI707_write_cmos_sensor(0x10, HI707_read_cmos_sensor(0x10)&0x7f);   //Close AE

    HI707_write_cmos_sensor(0x11, 0x00);
    HI707_write_cmos_sensor(0x18, HI707_read_cmos_sensor(0x18)|0x08);   //Reset AE
    HI707_write_cmos_sensor(0x2a, 0x00);
    HI707_write_cmos_sensor(0x2b, 0x35);

    HI707_write_cmos_sensor(0x03, 0x00);
    temp_data1 = ((HI707_read_cmos_sensor(0x40)<<8)|HI707_read_cmos_sensor(0x41));
    temp_data2 = ((HI707_read_cmos_sensor(0x42)<<8)|HI707_read_cmos_sensor(0x43));
    spin_lock(&hi707_yuv_drv_lock);
    HI707_sensor.pv_dummy_pixels = temp_data1;
    HI707_sensor.pv_line_length = HI707_VGA_DEFAULT_PIXEL_NUMS + HI707_sensor.pv_dummy_pixels ;
    HI707_sensor.pv_dummy_lines = temp_data2;
    spin_unlock(&hi707_yuv_drv_lock);

    HI707_expfix_temp = ((HI707_sensor.pv_pclk*10/fix_framerate)-(HI707_sensor.pv_line_length*HI707_sensor.pv_dummy_lines))/8;
    HI707_expfix = ((HI707_expfix_temp*8/HI707_sensor.pv_line_length)*HI707_sensor.pv_line_length)/8;

    HI707_write_cmos_sensor(0x03, 0x20);
    //HI707_write_cmos_sensor(0x83, (HI707_expfix>>16)&0xff);
    //HI707_write_cmos_sensor(0x84, (HI707_expfix>>8)&0xff);
    //HI707_write_cmos_sensor(0x85, (HI707_expfix>>0)&0xff);
    HI707_write_cmos_sensor(0x91, (HI707_expfix>>16)&0xff);
    HI707_write_cmos_sensor(0x92, (HI707_expfix>>8)&0xff);
    HI707_write_cmos_sensor(0x93, (HI707_expfix>>0)&0xff);

    if(HI707_sensor.banding == HI707_50HZ)
    {
        HI707_expbanding = ((HI707_read_cmos_sensor(0x8b)<<8)|HI707_read_cmos_sensor(0x8c));

		//HI707_expmax = ((HI707_expfix_temp-HI707_expbanding)/HI707_expbanding)*HI707_expbanding;
		 HI707_expbanding = (HI707_sensor.pv_pclk/HI707_sensor.pv_line_length/100)*HI707_sensor.pv_line_length/8;
              HI707_expmax = HI707_expbanding*100*10/fix_framerate ;

	    HI707_write_cmos_sensor(0x03, 0x20);
	    HI707_write_cmos_sensor(0xa0, (HI707_expmax>>16)&0xff);
	    HI707_write_cmos_sensor(0xa1, (HI707_expmax>>8)&0xff);
	    HI707_write_cmos_sensor(0xa2, (HI707_expmax>>0)&0xff);

		HI707_write_cmos_sensor(0x01, HI707_read_cmos_sensor(0x01)&0xfe);	//Exit Sleep: For Write Reg

		HI707_write_cmos_sensor(0x03, 0x20);
		HI707_write_cmos_sensor(0x10, HI707_read_cmos_sensor(0x10)|0x80);	//Open AE
		HI707_write_cmos_sensor(0x18, HI707_read_cmos_sensor(0x18)&0xf7);	//Reset AE
    }
    else if(HI707_sensor.banding == HI707_60HZ)
    {
        HI707_expbanding = ((HI707_read_cmos_sensor(0x8d)<<8)|HI707_read_cmos_sensor(0x8e));

		//HI707_expmax = ((HI707_expfix_temp-HI707_expbanding)/HI707_expbanding)*HI707_expbanding;
			 HI707_expbanding = (HI707_sensor.pv_pclk/HI707_sensor.pv_line_length/100)*HI707_sensor.pv_line_length/8;
              HI707_expmax = HI707_expbanding*120*10/fix_framerate ;
		HI707_write_cmos_sensor(0x03, 0x20);
		HI707_write_cmos_sensor(0x88, (HI707_expmax>>16)&0xff);
		HI707_write_cmos_sensor(0x89, (HI707_expmax>>8)&0xff);
		HI707_write_cmos_sensor(0x8a, (HI707_expmax>>0)&0xff);

		HI707_write_cmos_sensor(0x01, HI707_read_cmos_sensor(0x01)&0xfe);	//Exit Sleep: For Write Reg

		HI707_write_cmos_sensor(0x03, 0x20);
		HI707_write_cmos_sensor(0x10, HI707_read_cmos_sensor(0x10)|0x80);	//Open AE
		HI707_write_cmos_sensor(0x18, HI707_read_cmos_sensor(0x18)&0xf7);	//Reset AE
    }
    else//default 50HZ
    {
        HI707_expbanding = ((HI707_read_cmos_sensor(0x8b)<<8)|HI707_read_cmos_sensor(0x8c));

		//HI707_expmax = ((HI707_expfix_temp-HI707_expbanding)/HI707_expbanding)*HI707_expbanding;
			 HI707_expbanding = (HI707_sensor.pv_pclk/HI707_sensor.pv_line_length/100)*HI707_sensor.pv_line_length/8;
              HI707_expmax = HI707_expbanding*100*10/fix_framerate ;
	    HI707_write_cmos_sensor(0x03, 0x20);
	    HI707_write_cmos_sensor(0xa0, (HI707_expmax>>16)&0xff);
	    HI707_write_cmos_sensor(0xa1, (HI707_expmax>>8)&0xff);
	    HI707_write_cmos_sensor(0xa2, (HI707_expmax>>0)&0xff);

		HI707_write_cmos_sensor(0x01, HI707_read_cmos_sensor(0x01)&0xfe);	//Exit Sleep: For Write Reg

		HI707_write_cmos_sensor(0x03, 0x20);
		HI707_write_cmos_sensor(0x10, HI707_read_cmos_sensor(0x10)|0x80);	//Open AE
		HI707_write_cmos_sensor(0x18, HI707_read_cmos_sensor(0x18)&0xf7);	//Reset AE
    }
}

#if 0
// 320 * 240
static void HI707_Set_QVGA_mode(void)
{
    HI707_write_cmos_sensor(0x01, HI707_read_cmos_sensor(0x01)|0x01);   //Sleep: For Write Reg

    HI707_write_cmos_sensor(0x03, 0x00);
    HI707_write_cmos_sensor(0x10, 0x01);        //QVGA Size: 0x10 -> 0x01

    HI707_write_cmos_sensor(0x20, 0x00);
    HI707_write_cmos_sensor(0x21, 0x02);

    HI707_write_cmos_sensor(0x40, 0x01);        //HBLANK:  0x0158 = 344
    HI707_write_cmos_sensor(0x41, 0x58);
    HI707_write_cmos_sensor(0x42, 0x00);        //VBLANK:  0x14 = 20
    HI707_write_cmos_sensor(0x43, 0x14);

    HI707_write_cmos_sensor(0x03, 0x11);        //QVGA Fixframerate
    HI707_write_cmos_sensor(0x10, 0x21);

    HI707_write_cmos_sensor(0x03, 0x20);
    HI707_write_cmos_sensor(0x10, HI707_read_cmos_sensor(0x10)&0x7f);   //Close AE
    HI707_write_cmos_sensor(0x18, HI707_read_cmos_sensor(0x18)|0x08);   //Reset AE

    HI707_write_cmos_sensor(0x83, 0x00);
    HI707_write_cmos_sensor(0x84, 0xaf);
    HI707_write_cmos_sensor(0x85, 0xc8);
    HI707_write_cmos_sensor(0x86, 0x00);
    HI707_write_cmos_sensor(0x87, 0xfa);

    HI707_write_cmos_sensor(0x8b, 0x3a);
    HI707_write_cmos_sensor(0x8c, 0x98);
    HI707_write_cmos_sensor(0x8d, 0x30);
    HI707_write_cmos_sensor(0x8e, 0xd4);

    HI707_write_cmos_sensor(0x9c, 0x0b);
    HI707_write_cmos_sensor(0x9d, 0x3b);
    HI707_write_cmos_sensor(0x9e, 0x00);
    HI707_write_cmos_sensor(0x9f, 0xfa);

    HI707_write_cmos_sensor(0x01, HI707_read_cmos_sensor(0x01)&0xfe);   //Exit Sleep: For Write Reg

    HI707_write_cmos_sensor(0x03, 0x20);
    HI707_write_cmos_sensor(0x10, HI707_read_cmos_sensor(0x10)|0x80);   //Open AE
    HI707_write_cmos_sensor(0x18, HI707_read_cmos_sensor(0x18)&0xf7);   //Reset AE

}
#endif
void HI707_night_mode(kal_bool enable)
{
    kal_uint32 len=0;
    kal_bool prv_night_mode; //                                                                  

    SENSORDB("HHL[Enter]HI707 night mode func:enable = %d\n",enable);
    SENSORDB("HI707_sensor.video_mode = %d\n",HI707_sensor.MPEG4_Video_mode);
    SENSORDB("HI707_sensor.night_mode = %d\n",HI707_sensor.night_mode);
    spin_lock(&hi707_yuv_drv_lock);
    prv_night_mode = HI707_sensor.night_mode; //                                                                  
    HI707_sensor.night_mode = enable;
    spin_unlock(&hi707_yuv_drv_lock);

    if(HI707_sensor.MPEG4_Video_mode == KAL_TRUE)
        return;

    if(enable)
    {
        #ifdef MTK_ORIGINAL_CODE
        HI707_Cal_Min_Frame_Rate(HI707_MIN_FRAMERATE_5);
        #else
        len = sizeof(HI707_Night_On_Reg[HI707_sensor.banding])/sizeof(HI707_Night_On_Reg[HI707_sensor.banding][0]);
        HI707_table_write_cmos_sensor(HI707_Night_On_Reg[HI707_sensor.banding],len);
        #endif
    }
    else if (prv_night_mode) //                                                                  
    {
        #ifdef MTK_ORIGINAL_CODE
        HI707_Cal_Min_Frame_Rate(HI707_MIN_FRAMERATE_10);
        #else
        len = sizeof(HI707_Night_Off_Reg[HI707_sensor.banding])/sizeof(HI707_Night_Off_Reg[HI707_sensor.banding][0]);
        HI707_table_write_cmos_sensor(HI707_Night_Off_Reg[HI707_sensor.banding],len);
        #endif
    }
}

/*************************************************************************
* FUNCTION
*    HI707Preview
*
* DESCRIPTION
*    This function start the sensor preview.
*
* PARAMETERS
*    *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static UINT32 HI707Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint32 len=0;
    spin_lock(&hi707_yuv_drv_lock);
    sensor_config_data->SensorImageMirror = IMAGE_HV_MIRROR;
    spin_unlock(&hi707_yuv_drv_lock);

    SENSORDB("HHL[Enter]:HI707 preview func:");
    SENSORDB("HI707_sensor.video_mode = %d\n",HI707_sensor.MPEG4_Video_mode);

    if(sensor_config_data->SensorOperationMode== ACDK_SENSOR_OPERATION_MODE_VIDEO)
    {
        spin_lock(&hi707_yuv_drv_lock);
        HI707_sensor.Sensor_mode =SENSOR_MODE_VIDEO;
        spin_unlock(&hi707_yuv_drv_lock);
    }
    else
    {
        spin_lock(&hi707_yuv_drv_lock);
        HI707_sensor.Sensor_mode =SENSOR_MODE_PREVIEW;
        spin_unlock(&hi707_yuv_drv_lock);
    }
    //spin_unlock(&hi707_yuv_drv_lock);
	if(CurrentScenarioId == MSDK_SCENARIO_ID_VIDEO_PREVIEW)
	{ //15-30fps
		if(HI707_sensor.video_fps == 15)
		{ //MMS Size. Fixed 15fps
			len = sizeof(HI707_Video_Reg_15fps[HI707_sensor.banding])/sizeof(HI707_Video_Reg_15fps[HI707_sensor.banding][0]);
			HI707_table_write_cmos_sensor(HI707_Video_Reg_15fps[HI707_sensor.banding],len);
		}
#if 0
		else if(HI707_sensor.video_fps == 30)
		{ //VGA Size. 15-30fps
			len = sizeof(HI707_Video_Reg_30fps[HI707_sensor.banding])/sizeof(HI707_Video_Reg_30fps[HI707_sensor.banding][0]);
			HI707_table_write_cmos_sensor(HI707_Video_Reg_30fps[HI707_sensor.banding],len);
		}
#endif
		else
		{ //VGA Size. 10-30fps
	        len = sizeof(HI707_Preview_Reg[HI707_sensor.banding])/sizeof(HI707_Preview_Reg[HI707_sensor.banding][0]);
	        HI707_table_write_cmos_sensor(HI707_Preview_Reg[HI707_sensor.banding],len);
		}
	}
    else
    { //10-30fps
        len = sizeof(HI707_Preview_Reg[HI707_sensor.banding])/sizeof(HI707_Preview_Reg[HI707_sensor.banding][0]);
        HI707_table_write_cmos_sensor(HI707_Preview_Reg[HI707_sensor.banding],len);
    }
    SENSORDB("[Exit]:HI707 preview func\n");

    return TRUE;
}    /* HI707_Preview */


UINT32 HI707Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    SENSORDB("HHL[HI707][Enter]HI707_capture_func\n");
    spin_lock(&hi707_yuv_drv_lock);
    HI707_sensor.Sensor_mode =SENSOR_MODE_CAPTURE;
    spin_unlock(&hi707_yuv_drv_lock);
    return ERROR_NONE;
}    /* HM3451Capture() */


UINT32 HI707GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    SENSORDB("[Enter]:HI707 get Resolution func\n");

    pSensorResolution->SensorFullWidth=HI707_IMAGE_SENSOR_FULL_WIDTH ;
    pSensorResolution->SensorFullHeight=HI707_IMAGE_SENSOR_FULL_HEIGHT ;
    pSensorResolution->SensorPreviewWidth=HI707_IMAGE_SENSOR_PV_WIDTH ;
    pSensorResolution->SensorPreviewHeight=HI707_IMAGE_SENSOR_PV_HEIGHT ;
    pSensorResolution->SensorVideoWidth=HI707_IMAGE_SENSOR_PV_WIDTH ;
    pSensorResolution->SensorVideoHeight=HI707_IMAGE_SENSOR_PV_HEIGHT ;
    pSensorResolution->Sensor3DFullWidth=HI707_IMAGE_SENSOR_FULL_WIDTH ;
    pSensorResolution->Sensor3DFullHeight=HI707_IMAGE_SENSOR_FULL_HEIGHT ;
    pSensorResolution->Sensor3DPreviewWidth=HI707_IMAGE_SENSOR_PV_WIDTH ;
    pSensorResolution->Sensor3DPreviewHeight=HI707_IMAGE_SENSOR_PV_HEIGHT ;
    pSensorResolution->Sensor3DVideoWidth=HI707_IMAGE_SENSOR_PV_WIDTH ;
    pSensorResolution->Sensor3DVideoHeight=HI707_IMAGE_SENSOR_PV_HEIGHT ;

    SENSORDB("[Exit]:HI707 get Resolution func\n");
    return ERROR_NONE;
}    /* HI707GetResolution() */

UINT32 HI707GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                      MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                      MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

    switch(ScenarioId)
        {

            case MSDK_SCENARIO_ID_CAMERA_ZSD:
                 pSensorInfo->SensorPreviewResolutionX=HI707_IMAGE_SENSOR_PV_WIDTH;
                 pSensorInfo->SensorPreviewResolutionY=HI707_IMAGE_SENSOR_PV_HEIGHT;
                 pSensorInfo->SensorFullResolutionX=HI707_IMAGE_SENSOR_FULL_WIDTH;
                 pSensorInfo->SensorFullResolutionY=HI707_IMAGE_SENSOR_FULL_HEIGHT;
                 pSensorInfo->SensorCameraPreviewFrameRate=15;
                 break;

            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                 pSensorInfo->SensorPreviewResolutionX=HI707_IMAGE_SENSOR_PV_WIDTH;
                 pSensorInfo->SensorPreviewResolutionY=HI707_IMAGE_SENSOR_PV_HEIGHT;
                 pSensorInfo->SensorFullResolutionX=HI707_IMAGE_SENSOR_FULL_WIDTH;
                 pSensorInfo->SensorFullResolutionY=HI707_IMAGE_SENSOR_FULL_HEIGHT;
                 pSensorInfo->SensorCameraPreviewFrameRate=30;
                 break;
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                 pSensorInfo->SensorPreviewResolutionX=HI707_IMAGE_SENSOR_PV_WIDTH;
                 pSensorInfo->SensorPreviewResolutionY=HI707_IMAGE_SENSOR_PV_HEIGHT;
                 pSensorInfo->SensorFullResolutionX=HI707_IMAGE_SENSOR_FULL_WIDTH;
                 pSensorInfo->SensorFullResolutionY=HI707_IMAGE_SENSOR_FULL_HEIGHT;
                 pSensorInfo->SensorCameraPreviewFrameRate=30;
                break;
            case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
            case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
            case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added
                 pSensorInfo->SensorPreviewResolutionX=HI707_IMAGE_SENSOR_PV_WIDTH;
                 pSensorInfo->SensorPreviewResolutionY=HI707_IMAGE_SENSOR_PV_HEIGHT;
                 pSensorInfo->SensorFullResolutionX=HI707_IMAGE_SENSOR_FULL_WIDTH;
                 pSensorInfo->SensorFullResolutionY=HI707_IMAGE_SENSOR_FULL_HEIGHT;
                 pSensorInfo->SensorCameraPreviewFrameRate=30;
                break;
            default:

                 pSensorInfo->SensorPreviewResolutionX=HI707_IMAGE_SENSOR_PV_WIDTH;
                 pSensorInfo->SensorPreviewResolutionY=HI707_IMAGE_SENSOR_PV_HEIGHT;
                 pSensorInfo->SensorFullResolutionX=HI707_IMAGE_SENSOR_FULL_WIDTH;
                 pSensorInfo->SensorFullResolutionY=HI707_IMAGE_SENSOR_FULL_HEIGHT;
                 pSensorInfo->SensorCameraPreviewFrameRate=30;
                 break;

            }



    SENSORDB("[Enter]:HI707 getInfo func:ScenarioId = %d\n",ScenarioId);

  //  pSensorInfo->SensorPreviewResolutionX=HI707_IMAGE_SENSOR_PV_WIDTH;
  //  pSensorInfo->SensorPreviewResolutionY=HI707_IMAGE_SENSOR_PV_HEIGHT;
 //   pSensorInfo->SensorFullResolutionX=HI707_IMAGE_SENSOR_FULL_WIDTH;
 //   pSensorInfo->SensorFullResolutionY=HI707_IMAGE_SENSOR_FULL_HEIGHT;

    pSensorInfo->SensorCameraPreviewFrameRate=30;
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=30;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;//low is to reset
    pSensorInfo->SensorResetDelayCount=4;  //4ms
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV; //SENSOR_OUTPUT_FORMAT_YVYU;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;


    pSensorInfo->CaptureDelayFrame = 4;
    pSensorInfo->PreviewDelayFrame = 1;//                                                                                                           
    pSensorInfo->VideoDelayFrame = 0;
    pSensorInfo->SensorMasterClockSwitch = 0;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;

    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW:
    case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=    3;
        pSensorInfo->SensorClockRisingCount= 0;
        pSensorInfo->SensorClockFallingCount= 2;
        pSensorInfo->SensorPixelClockCount= 3;
        pSensorInfo->SensorDataLatchCount= 2;
        pSensorInfo->SensorGrabStartX = 4;
        pSensorInfo->SensorGrabStartY = 2;
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=    3;
        pSensorInfo->SensorClockRisingCount= 0;
        pSensorInfo->SensorClockFallingCount= 2;
        pSensorInfo->SensorPixelClockCount= 3;
        pSensorInfo->SensorDataLatchCount= 2;
        pSensorInfo->SensorGrabStartX = 4;
        pSensorInfo->SensorGrabStartY = 2;//4;
        break;
    default:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=3;
        pSensorInfo->SensorDataLatchCount=2;
        pSensorInfo->SensorGrabStartX = 4;
        pSensorInfo->SensorGrabStartY = 2;//4;
        break;
    }
    //    HI707_PixelClockDivider=pSensorInfo->SensorPixelClockCount;


    pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;
    pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 4;
    pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;


    memcpy(pSensorConfigData, &HI707SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    SENSORDB("[Exit]:HI707 getInfo func\n");
    return ERROR_NONE;
}    /* HI707GetInfo() */


UINT32 HI707Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                      MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    SENSORDB("HHL [Enter]:HI707 Control func:ScenarioId = %d\n",ScenarioId);
	CurrentScenarioId = ScenarioId;

    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    //case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW:
    //case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        HI707Preview(pImageWindow, pSensorConfigData);
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
    //case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE:
        HI707Capture(pImageWindow, pSensorConfigData);
        break;
     //   case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
     //   case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
    //    case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added
    //    HI707Preview(pImageWindow, pSensorConfigData);
     //   break;
    //    case MSDK_SCENARIO_ID_CAMERA_ZSD:
    //    HI707Capture(pImageWindow, pSensorConfigData);
    //    break;
    default:
         HI707Preview(pImageWindow, pSensorConfigData);
        break;
    }

    SENSORDB("[Exit]:HI707 Control func\n");
    return TRUE;
}    /* HI707Control() */

void HI707_set_scene_mode(UINT16 para)
{


     SENSORDB("[Hi704_Debug]enter HI707_set_scene_mode function:\n ");
    SENSORDB("[Hi704_Debug] HI707_set_scene_mode=%d",para);
    spin_lock(&hi707_yuv_drv_lock);
    HI707_sensor.SceneMode = para;
    spin_unlock(&hi707_yuv_drv_lock);

        switch (para)
        {

        case SCENE_MODE_NIGHTSCENE:

            HI707_night_mode(KAL_TRUE);
            break;

        case SCENE_MODE_HDR:
            SENSORDB("[Hi704_Debug]enter HI707_set_scene_mode function HDR:\n ");
            break;
        default :
            HI707_night_mode(KAL_FALSE);
            break;
        }
      return;
}


/*************************************************************************
* FUNCTION
*    HI707_set_param_wb
*
* DESCRIPTION
*    wb setting.
*
* PARAMETERS
*    none
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL HI707_set_param_wb(UINT16 para)
{
    //This sensor need more time to balance AWB,
    //we suggest higher fps or drop some frame to avoid garbage color when preview initial
    SENSORDB("[Enter]HI707 set_param_wb func:para = %d\n",para);

    if(HI707_sensor.wb == para) return KAL_TRUE;

    spin_lock(&hi707_yuv_drv_lock);
    HI707_sensor.wb = para;
    spin_unlock(&hi707_yuv_drv_lock);

    switch (para)
    {
    case AWB_MODE_AUTO:
        {
        HI707_write_cmos_sensor(0x03, 0x22);
        HI707_write_cmos_sensor(0x10, 0x7b);
        HI707_write_cmos_sensor(0x11, 0x2e);
        HI707_write_cmos_sensor(0x80, 0x40); //3d},
        HI707_write_cmos_sensor(0x81, 0x20);
        HI707_write_cmos_sensor(0x82, 0x38); //40},
        HI707_write_cmos_sensor(0x83, 0x59); //58}, //RMAX
        HI707_write_cmos_sensor(0x84, 0x20); //RMIN
        HI707_write_cmos_sensor(0x85, 0x53); //BMAX
        HI707_write_cmos_sensor(0x86, 0x24); //BMIN
        HI707_write_cmos_sensor(0x87, 0x49); //48}, //RMAXB
        HI707_write_cmos_sensor(0x88, 0x3c); //RMINB
        HI707_write_cmos_sensor(0x89, 0x3e); //BMAXB
        HI707_write_cmos_sensor(0x8a, 0x34); //BMINB
        HI707_write_cmos_sensor(0x8d, 0x24); //11,}, //IN/OUT slop R
        HI707_write_cmos_sensor(0x8e, 0x61); //11,}, //IN/OUT slop B
        HI707_write_cmos_sensor(0x10, 0xfb);
        }
        break;
    case AWB_MODE_CLOUDY_DAYLIGHT:
        {
        HI707_write_cmos_sensor(0x03, 0x22);
        HI707_write_cmos_sensor(0x10, 0x7b);
        HI707_write_cmos_sensor(0x11, 0x26);
        //Cloudy
        HI707_write_cmos_sensor(0x80, 0x60);
        HI707_write_cmos_sensor(0x81, 0x20);
        HI707_write_cmos_sensor(0x82, 0x20);
        HI707_write_cmos_sensor(0x83, 0x70);
        HI707_write_cmos_sensor(0x84, 0x51);
        HI707_write_cmos_sensor(0x85, 0x2A);
        HI707_write_cmos_sensor(0x86, 0x20);
        HI707_write_cmos_sensor(0x87, 0x60); //RMAX
        HI707_write_cmos_sensor(0x88, 0x20); //RMIN
        HI707_write_cmos_sensor(0x89, 0x60); //BMAX
        HI707_write_cmos_sensor(0x8a, 0x20); //BMIN
        HI707_write_cmos_sensor(0x8d, 0x00); //IN/OUT slop R
        HI707_write_cmos_sensor(0x8e, 0x00); //IN/OUT slop B
        HI707_write_cmos_sensor(0x10, 0xfb);
        }
        break;
    case AWB_MODE_DAYLIGHT:
        {
        HI707_write_cmos_sensor(0x03, 0x22);
        HI707_write_cmos_sensor(0x10, 0x7b);
        HI707_write_cmos_sensor(0x11, 0x26);
        //D50
        HI707_write_cmos_sensor(0x80, 0x42);
        HI707_write_cmos_sensor(0x81, 0x20);
        HI707_write_cmos_sensor(0x82, 0x3d);
        HI707_write_cmos_sensor(0x83, 0x49);
        HI707_write_cmos_sensor(0x84, 0x3A);
        HI707_write_cmos_sensor(0x85, 0x47);
        HI707_write_cmos_sensor(0x86, 0x33);
        HI707_write_cmos_sensor(0x87, 0x60); //RMAX
        HI707_write_cmos_sensor(0x88, 0x20); //RMIN
        HI707_write_cmos_sensor(0x89, 0x60); //BMAX
        HI707_write_cmos_sensor(0x8a, 0x20); //BMIN
        HI707_write_cmos_sensor(0x8d, 0x00); //IN/OUT slop R
        HI707_write_cmos_sensor(0x8e, 0x00); //IN/OUT slop B
        HI707_write_cmos_sensor(0x10, 0xfb);
        }
        break;
    case AWB_MODE_INCANDESCENT:
        {
        HI707_write_cmos_sensor(0x03, 0x22);
        HI707_write_cmos_sensor(0x10, 0x7b);
        HI707_write_cmos_sensor(0x11, 0x26);
        //INCA
        HI707_write_cmos_sensor(0x80, 0x20);
        HI707_write_cmos_sensor(0x81, 0x20);
        HI707_write_cmos_sensor(0x82, 0x60);
        HI707_write_cmos_sensor(0x83, 0x2F);
        HI707_write_cmos_sensor(0x84, 0x11);
        HI707_write_cmos_sensor(0x85, 0x67);
        HI707_write_cmos_sensor(0x86, 0x58);
        HI707_write_cmos_sensor(0x87, 0x60); //RMAX
        HI707_write_cmos_sensor(0x88, 0x20); //RMIN
        HI707_write_cmos_sensor(0x89, 0x60); //BMAX
        HI707_write_cmos_sensor(0x8a, 0x20); //BMIN
        HI707_write_cmos_sensor(0x8d, 0x00); //IN/OUT slop R
        HI707_write_cmos_sensor(0x8e, 0x00); //IN/OUT slop B
        HI707_write_cmos_sensor(0x10, 0xfb);
        }
        break;
    case AWB_MODE_FLUORESCENT:
        {
        HI707_write_cmos_sensor(0x03, 0x22);
        HI707_write_cmos_sensor(0x10, 0x7b);
        HI707_write_cmos_sensor(0x11, 0x26);
        //TL84
        HI707_write_cmos_sensor(0x80, 0x2d);
        HI707_write_cmos_sensor(0x81, 0x20);
        HI707_write_cmos_sensor(0x82, 0x50);
        HI707_write_cmos_sensor(0x83, 0x37);
        HI707_write_cmos_sensor(0x84, 0x23);
        HI707_write_cmos_sensor(0x85, 0x55);
        HI707_write_cmos_sensor(0x86, 0x4B);
        HI707_write_cmos_sensor(0x87, 0x60); //RMAX
        HI707_write_cmos_sensor(0x88, 0x20); //RMIN
        HI707_write_cmos_sensor(0x89, 0x60); //BMAX
        HI707_write_cmos_sensor(0x8a, 0x20); //BMIN
        HI707_write_cmos_sensor(0x8d, 0x00); //IN/OUT slop R
        HI707_write_cmos_sensor(0x8e, 0x00); //IN/OUT slop B
        HI707_write_cmos_sensor(0x10, 0xfb);
        }
        break;
    case AWB_MODE_OFF:
        {
        SENSORDB("HI707 AWB OFF");
        HI707_write_cmos_sensor(0x03, 0x22);
        HI707_write_cmos_sensor(0x10, 0xe2);
        }
        break;
    default:
        return FALSE;
    }

    return TRUE;
} /* HI707_set_param_wb */

/*************************************************************************
* FUNCTION
*	HI707_set_param_effect
*
* DESCRIPTION
*	effect setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL HI707_set_param_effect(UINT16 para)
{
   SENSORDB("[Enter]HI707 set_param_effect func:para = %d\n",para);

    if(HI707_sensor.effect == para) return KAL_TRUE;

    spin_lock(&hi707_yuv_drv_lock);
    HI707_sensor.effect = para;
    spin_unlock(&hi707_yuv_drv_lock);

    switch (para)
    {
    case MEFFECT_OFF:
        {
        HI707_write_cmos_sensor(0x03, 0x10);
        HI707_write_cmos_sensor(0x11, 0x43);
        HI707_write_cmos_sensor(0x12, 0x30);
        HI707_write_cmos_sensor(0x44, 0x80);
        HI707_write_cmos_sensor(0x45, 0x80);
        }
        break;
    case MEFFECT_SEPIA:
        {
        HI707_write_cmos_sensor(0x03, 0x10);
        HI707_write_cmos_sensor(0x11, 0x03);
        HI707_write_cmos_sensor(0x12, 0x33);
        HI707_write_cmos_sensor(0x44, 0x70);
        HI707_write_cmos_sensor(0x45, 0x98);
        }
        break;
    case MEFFECT_NEGATIVE:
        {
        HI707_write_cmos_sensor(0x03, 0x10);
        HI707_write_cmos_sensor(0x11, 0x03);
        HI707_write_cmos_sensor(0x12, 0x38);
        HI707_write_cmos_sensor(0x44, 0x80);
        HI707_write_cmos_sensor(0x45, 0x80);
        }
        break;
    case MEFFECT_MONO:
        {
        HI707_write_cmos_sensor(0x03, 0x10);
        HI707_write_cmos_sensor(0x11, 0x03);
        HI707_write_cmos_sensor(0x12, 0x33);
        HI707_write_cmos_sensor(0x44, 0x80);
        HI707_write_cmos_sensor(0x45, 0x80);
        }
        break;
    default:
        return KAL_FALSE;
    }

    return KAL_TRUE;
} /* HI707_set_param_effect */

/*************************************************************************
* FUNCTION
*	HI707_set_param_banding
*
* DESCRIPTION
*	banding setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL HI707_set_param_banding(UINT16 para)
{
#if 0
    SENSORDB("[Enter]HI707 set_param_banding func:para = %d\n",para);

    if(HI707_sensor.banding == para) return KAL_TRUE;

    spin_lock(&hi707_yuv_drv_lock);
    HI707_sensor.banding = para;
    spin_unlock(&hi707_yuv_drv_lock);

    switch (para)
    {
    case AE_FLICKER_MODE_50HZ:
        {
        HI707_write_cmos_sensor(0x03,0x20);
        HI707_write_cmos_sensor(0x10,0x9c);
        }
        break;
    case AE_FLICKER_MODE_60HZ:
        {
        HI707_write_cmos_sensor(0x03,0x20);
        HI707_write_cmos_sensor(0x10,0x8c);
        }
        break;
    default:
        return KAL_FALSE;
    }
#endif
    return KAL_TRUE;
} /* HI707_set_param_banding */




/*************************************************************************
* FUNCTION
*	HI707_set_param_exposure
*
* DESCRIPTION
*    exposure setting.
*
* PARAMETERS
*    none
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL HI707_set_param_exposure(UINT16 para)
{
    SENSORDB("[Enter]HI707 set_param_exposure func:para = %d\n",para);

    if(HI707_sensor.exposure == para)
    return KAL_TRUE;
    spin_lock(&hi707_yuv_drv_lock);
    HI707_sensor.exposure = para;
    spin_unlock(&hi707_yuv_drv_lock);
    if (SCENE_MODE_HDR == HI707_sensor.SceneMode && SENSOR_MODE_CAPTURE == HI707_sensor.Sensor_mode)
    {

               switch (para)
                    {
                      case AE_EV_COMP_n10:
                      case AE_EV_COMP_n20:
                          /* EV -2 */
                          SENSORDB("[Hi704_Debug]HDR AE_EV_COMP_n20 Para:%d;\n",para);
                          HI707_write_cmos_sensor(0x03,0x10);
                          HI707_write_cmos_sensor(0x12,HI707_read_cmos_sensor(0x12)|0x10);
                          HI707_write_cmos_sensor(0x40,0xe0);
                    HI707_write_cmos_sensor(0x03,0x20);
                    HI707_write_cmos_sensor(0x70,0x20);

                          break;
                      case AE_EV_COMP_10:
                      case AE_EV_COMP_20:               /* EV +2 */
                          SENSORDB("[Hi704_Debug]HDR AE_EV_COMP_20 Para:%d;\n",para);
                          HI707_write_cmos_sensor(0x03,0x10);
                          HI707_write_cmos_sensor(0x12,HI707_read_cmos_sensor(0x12)|0x10);
                          HI707_write_cmos_sensor(0x40,0x50);
                    HI707_write_cmos_sensor(0x03,0x20);
                    HI707_write_cmos_sensor(0x70,0x70);
                          break;
                      case AE_EV_COMP_00:              /* EV +2 */
                            SENSORDB("[Hi704_Debug]HDR AE_EV_COMP_00 Para:%d;\n",para);
                            HI707_write_cmos_sensor(0x03,0x10);
                            HI707_write_cmos_sensor(0x12,HI707_read_cmos_sensor(0x12)|0x10);
                            //HI707_write_cmos_sensor(0x40,0x80);
                            HI707_write_cmos_sensor(0x40,HI707SetBrightness_value);

                    HI707_write_cmos_sensor(0x03,0x20);
                    HI707_write_cmos_sensor(0x70,HI707_set_param_exposure_value);
                          break;
                      default:
                          return KAL_FALSE;
                    }
               return TRUE;
    }
//                                                                
    else{
		#if 1
			HI707_write_cmos_sensor(0x03,0x10);
			HI707_write_cmos_sensor(0x12,HI707_read_cmos_sensor(0x12)|0x10);
			switch (para)
			    {
			    case AE_EV_COMP_30: //+3 EV
					HI707_write_cmos_sensor(0x40,0x50);
					break;
			    case AE_EV_COMP_20: //+2 EV
					HI707_write_cmos_sensor(0x40,0x20);
					break;
			    case AE_EV_COMP_10:  //+1 EV
					HI707_write_cmos_sensor(0x40,0x10);
					break;
			    case AE_EV_COMP_00:  //0 EV
					HI707_write_cmos_sensor(0x40,0x80);
					break;
			    case AE_EV_COMP_n10: // -1 EV
					HI707_write_cmos_sensor(0x40,0x90);
					break;
			    case AE_EV_COMP_n20:// -2 EV
					HI707_write_cmos_sensor(0x40,0xA0);
					break;
			    case AE_EV_COMP_n30:// -3 EV
					HI707_write_cmos_sensor(0x40,0xD0);
					break;
			    default:
					return FALSE;
			    }
		//                                                                
		#else
                switch (para)
                {
                case AE_EV_COMP_20: //+2 EV
                    HI707_write_cmos_sensor(0x03,0x20);
                    HI707_write_cmos_sensor(0x70,0x70);
                    HI707_set_param_exposure_value = 0x70;
                    break;
                case AE_EV_COMP_10:  //+1 EV
                    HI707_write_cmos_sensor(0x03,0x20);
                    HI707_write_cmos_sensor(0x70,0x60);
                    HI707_set_param_exposure_value = 0x60;
                    break;
                case AE_EV_COMP_00:  //+2 EV
                    HI707_write_cmos_sensor(0x03,0x20);
                    HI707_write_cmos_sensor(0x70,0x48);
                    HI707_set_param_exposure_value = 0x48;
                    break;
                 case AE_EV_COMP_n10: // -1 EV
                     HI707_write_cmos_sensor(0x03,0x20);
                     HI707_write_cmos_sensor(0x70,0x30);
                     HI707_set_param_exposure_value = 0x30;
                    break;
                case AE_EV_COMP_n20:// -2 EV
                    HI707_write_cmos_sensor(0x03,0x20);
                    HI707_write_cmos_sensor(0x70,0x20);
                    HI707_set_param_exposure_value = 0x20;
                    break;
                 default:
                    return FALSE;
                }
		#endif
            }
    return TRUE;
} /* HI707_set_param_exposure */

void HI707_set_AE_mode(UINT32 iPara)
{
    UINT8 temp_AE_reg = 0;
    SENSORDB("HI707_set_AE_mode = %d E \n",iPara);
    HI707_write_cmos_sensor(0x03,0x20);
    temp_AE_reg = HI707_read_cmos_sensor(0x10);

    if (AE_MODE_OFF == iPara)
    {
        // turn off AEC/AGC
        HI707_write_cmos_sensor(0x10,temp_AE_reg &~ 0x80);
    }
    else
    {
        HI707_write_cmos_sensor(0x10,temp_AE_reg | 0x80);
    }
}
UINT32 HI707SetTestPatternMode(kal_bool bEnable)
{
    //    HI257MIPISENSORDB("[OV5645MIPI_OV5645SetTestPatternMode]test pattern bEnable:=%d\n",bEnable);
    if(bEnable)
    {

        HI707_write_cmos_sensor(0x03,0x00);
        HI707_write_cmos_sensor(0x50,0x05);
        //run_test_potten=1;
    }
    else
    {
        HI707_write_cmos_sensor(0x03,0x00);
        HI707_write_cmos_sensor(0x50,0x00);
        //run_test_potten=0;
    }
    return ERROR_NONE;
}


void HI707SetBrightness(UINT16 para)
{
/*
 HI707_write_cmos_sensor(0x03,0x10);
 HI707_write_cmos_sensor(0x12,HI707_read_cmos_sensor(0x12)|0x10);
    switch (para)
    {
        case ISP_BRIGHT_LOW:
            HI707_write_cmos_sensor(0x40,0xd0);
            HI707SetBrightness_value = 0xd0;
             break;
        case ISP_BRIGHT_HIGH:
            HI707_write_cmos_sensor(0x40,0x50);
            HI707SetBrightness_value = 0x50;
             break;
        case ISP_BRIGHT_MIDDLE:
            HI707_write_cmos_sensor(0x40,0x80);
            HI707SetBrightness_value = 0x80;
            break;
        default:
             break;
    }
*/
    return;
}



void HI707SetContrast(UINT16 para)
{
/*
	HI707_write_cmos_sensor(0x03,0x10);
	HI707_write_cmos_sensor(0x11,HI707_read_cmos_sensor(0x11)|0x40);

    switch (para)
    {
        case ISP_CONTRAST_LOW:
			HI707_write_cmos_sensor(0x48,0x60);
             break;
        case ISP_CONTRAST_HIGH:
			HI707_write_cmos_sensor(0x48,0xa0);
             break;
        case ISP_CONTRAST_MIDDLE:
			HI707_write_cmos_sensor(0x48,0x80);
        default:
             break;
    }
    //spin_unlock(&s5k4ecgx_mipi_rw_lock);
*/
    return;
}



void HI707SetSetIso(UINT16 para)
{
    //spin_lock(&s5k4ecgx_mipi_rw_lock);
    switch (para)
    {
        case AE_ISO_100:
             //ISO 100

             break;
        case AE_ISO_200:
             //ISO 200

             break;
        case AE_ISO_400:
             //ISO 400

             break;
        default:
        case AE_ISO_AUTO:
             //ISO Auto

             break;
    }
    //spin_unlock(&s5k4ecgx_mipi_rw_lock);
}


void HI707SetSaturation(UINT16 para)
{
/*
    switch (para)
    {
        case ISP_SAT_HIGH:
			HI707_write_cmos_sensor(0x03,0x10);
			HI707_write_cmos_sensor(0x62,0xa0);
			HI707_write_cmos_sensor(0x63,0xa0);
             break;
        case ISP_SAT_LOW:
			HI707_write_cmos_sensor(0x03,0x10);
			HI707_write_cmos_sensor(0x62,0x60);
			HI707_write_cmos_sensor(0x63,0x60);
             break;
        case ISP_SAT_MIDDLE:
			HI707_write_cmos_sensor(0x03,0x10);
			HI707_write_cmos_sensor(0x62,0x80);
			HI707_write_cmos_sensor(0x63,0x80);
        default:
             break;
    }
*/
}

UINT32 HI707YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
    SENSORDB("[Enter]HI707YUVSensorSetting func:cmd = %d\n",iCmd);

    switch (iCmd)
    {
    case FID_SCENE_MODE:        //auto mode or night mode
        HI707_set_scene_mode(iPara);
         break;
    case FID_AWB_MODE:
        HI707_set_param_wb(iPara);
        break;
    case FID_COLOR_EFFECT:
        HI707_set_param_effect(iPara);
        break;
    case FID_AE_EV:
        HI707_set_param_exposure(iPara);
        break;
    case FID_AE_FLICKER:
        HI707_set_param_banding(iPara);
        break;
    case FID_ZOOM_FACTOR:
        spin_lock(&hi707_yuv_drv_lock);
        HI707_zoom_factor = iPara;
        spin_unlock(&hi707_yuv_drv_lock);
        break;
    case FID_AE_SCENE_MODE:
        HI707_set_AE_mode(iPara);
        break;
        case FID_ISP_CONTRAST:
            SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_ISP_CONTRAST:%d\n",iPara);
            HI707SetContrast(iPara);
            break;
        case FID_ISP_BRIGHT:
            SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_ISP_BRIGHT:%d\n",iPara);
            HI707SetBrightness(iPara);
            break;
        case FID_ISP_SAT:
            SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_ISP_SAT:%d\n",iPara);
            HI707SetSaturation(iPara);
            break;
        case FID_AE_ISO:
            SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_AE_ISO:%d\n",iPara);
            HI707SetSetIso(iPara);
            break;
    default:
        break;
    }
    return TRUE;
}   /* HI707YUVSensorSetting */

UINT32 HI707YUVSetVideoMode(UINT16 u2FrameRate)
{
	kal_uint32 len=0;
    spin_lock(&hi707_yuv_drv_lock);
    HI707_sensor.MPEG4_Video_mode = KAL_TRUE;
    spin_unlock(&hi707_yuv_drv_lock);
    SENSORDB("[Enter]HI707 Set Video Mode:FrameRate= %d\n",u2FrameRate);
    SENSORDB("HI707_sensor.video_mode = %d\n",HI707_sensor.MPEG4_Video_mode);
	if(CurrentScenarioId == MSDK_SCENARIO_ID_VIDEO_PREVIEW)
	{ //15-30fps
		if(u2FrameRate == 15)
		{ //MMS Size. Fixed 15fps
			len = sizeof(HI707_Video_Reg_15fps[HI707_sensor.banding])/sizeof(HI707_Video_Reg_15fps[HI707_sensor.banding][0]);
			HI707_table_write_cmos_sensor(HI707_Video_Reg_15fps[HI707_sensor.banding],len);
		}
#if 0	//video frame rate = 15~30
		else if(u2FrameRate == 30)
		{ //VGA Size. 15-30fps
			len = sizeof(HI707_Video_Reg_30fps[HI707_sensor.banding])/sizeof(HI707_Video_Reg_30fps[HI707_sensor.banding][0]);
			HI707_table_write_cmos_sensor(HI707_Video_Reg_30fps[HI707_sensor.banding],len);
		}
#endif
		else
		{ //VGA Size. 10-30fps
			len = sizeof(HI707_Preview_Reg[HI707_sensor.banding])/sizeof(HI707_Preview_Reg[HI707_sensor.banding][0]);
			HI707_table_write_cmos_sensor(HI707_Preview_Reg[HI707_sensor.banding],len);
		}
	}

#ifdef MTK_ORIGINAL_CODE
    if(u2FrameRate >= 30)
		u2FrameRate = 30;
	else if(u2FrameRate>0)
		u2FrameRate = 15;
	else
    {
        SENSORDB("Wrong Frame Rate");

		return FALSE;
    }


    spin_lock(&hi707_yuv_drv_lock);
    HI707_sensor.fix_framerate = u2FrameRate * 10;
    spin_unlock(&hi707_yuv_drv_lock);

        HI707_Fix_Video_Frame_Rate(HI707_sensor.fix_framerate);
#endif


    return TRUE;
}

void HI707GetAFMaxNumFocusAreas(UINT32 *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 0;
    SENSORDB("HI707GetAFMaxNumFocusAreas *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);
}

void HI707GetAEMaxNumMeteringAreas(UINT32 *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 1;//                                                              
    SENSORDB("HI707GetAEMaxNumMeteringAreas *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);
}
//                                                               
#define AEC_ROI_DX (192)
#define AEC_ROI_DY (192)
void HI707SetAEWindow(UINT32 zone_addr)
{
	INT16 x0, y0, x1, y1, width, height, i, j, coordinate_x, coordinate_y;
	INT16 x_start, x_end, y_start, y_end;
	UINT32* ptr = (UINT32*)zone_addr;

	x0 = *ptr;
	y0 = *(ptr + 1);
	x1 = *(ptr + 2);
	y1 = *(ptr + 3);
	width = *(ptr + 4);
	height = *(ptr + 5);
	SENSORDB("[HI707SetAEWindow] AE_Set_Window 3AWin: (%d,%d)~(%d,%d) w=%d h=%d\n",	x0, y0, x1, y1, width, height);

	coordinate_x = ((x0 + x1)/2)*(HI707_IMAGE_SENSOR_VGA_WIDTH/width);
	coordinate_y = ((y0 + y1)/2)*(HI707_IMAGE_SENSOR_VGA_HEIGHT/height);
	SENSORDB("[HI707SetAEWindow] AE_Set_Window 3AWin: cor_x=%d, cor_y=%d\n", coordinate_x, coordinate_y);

	/* Set page */
	HI707_write_cmos_sensor(0x03, 0x20);

	/* AE weight */
	HI707_write_cmos_sensor(0x60, 0x70);
	HI707_write_cmos_sensor(0x61, 0x00);
	HI707_write_cmos_sensor(0x62, 0x70);
	HI707_write_cmos_sensor(0x63, 0x00);

	if ((x0 == x1) && (y0 == y1))
	{
		/* AE window */
		HI707_write_cmos_sensor(0x68, 0x41);
		HI707_write_cmos_sensor(0x69, 0x81);
		HI707_write_cmos_sensor(0x6A, 0x38);
		HI707_write_cmos_sensor(0x6B, 0xb8);
	}
	else
	{
#ifdef CONFIG_HI707_ROT_180
		coordinate_x = HI707_IMAGE_SENSOR_VGA_WIDTH - coordinate_x;
		coordinate_y = HI707_IMAGE_SENSOR_VGA_HEIGHT - coordinate_y;
#endif

		x_start = ((coordinate_x - (AEC_ROI_DX/2) > 0)? coordinate_x - (AEC_ROI_DX/2) : 0)/4;
		x_end = ((coordinate_x + (AEC_ROI_DX/2) < HI707_IMAGE_SENSOR_VGA_WIDTH)? coordinate_x + (AEC_ROI_DX/2) : HI707_IMAGE_SENSOR_VGA_WIDTH)/4;

		y_start = ((coordinate_y - (AEC_ROI_DY/2) > 0)? coordinate_y - (AEC_ROI_DY/2) : 0)/2;
		y_end = ((coordinate_y + (AEC_ROI_DY/2) < HI707_IMAGE_SENSOR_VGA_HEIGHT)? coordinate_y + (AEC_ROI_DY/2) : HI707_IMAGE_SENSOR_VGA_HEIGHT)/2;

		SENSORDB("[HI707SetAEWindow] AE_Set_Window 3AWin: (%d,%d)~(%d,%d)\n", x_start, y_start, x_end, y_end);

		/* AE window */
		HI707_write_cmos_sensor(0x68, x_start&0xFF);
		HI707_write_cmos_sensor(0x69, x_end&0xFF);
		HI707_write_cmos_sensor(0x6A, y_start&0xFF);
		HI707_write_cmos_sensor(0x6B, y_end&0xFF);
	}
}
//                                                               
void HI707_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{

    SENSORDB(" HI707_3ACtrl is %d\n",action);

    switch (action)
    {
    case SENSOR_3A_AE_LOCK:
        HI707_set_AE_mode(KAL_FALSE);
        break;
    case SENSOR_3A_AE_UNLOCK:
        HI707_set_AE_mode(KAL_TRUE);
        break;

    case SENSOR_3A_AWB_LOCK:
        HI707_set_param_wb(AWB_MODE_OFF);
        break;

    case SENSOR_3A_AWB_UNLOCK:
        HI707_set_param_wb(HI707_sensor.wb);
        break;
    default:
        break;
    }
    return;
}

void HI707GetExifInfo(UINT32 exifAddr)
{
    UINT16 isoSpeed = 0;
	ExposureTime_High = HI707_read_cmos_sensor(0x2080);
    ExposureTime_Mid = HI707_read_cmos_sensor(0x2081);
    ExposureTime_Low = HI707_read_cmos_sensor(0x2082);
	ExposureTime_Low = HI707_read_cmos_sensor(0x2082);
	ISO_RegVal = HI707_read_cmos_sensor(0xB0);
	if(ISO_RegVal <= 0x28)
		ISO_RegVal = 0x28;
	if(ISO_RegVal > 0)
		isoSpeed = (ISO_RegVal / 32);
    ExposureTime = ((ExposureTime_High<<16) + (ExposureTime_Mid<<8) + (ExposureTime_Low));
	SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
	pExifInfo->FNumber = 30;
	pExifInfo->FocalLength = 129;
    pExifInfo->AEISOSpeed = isoSpeed;
    pExifInfo->AWBMode = HI707_sensor.wb;
    pExifInfo->CapExposureTime = (ExposureTime*4);//                                                                                
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = isoSpeed;
}
void HI707GetDelayInfo(UINT32 delayAddr)
{
    SENSOR_DELAY_INFO_STRUCT* pDelayInfo = (SENSOR_DELAY_INFO_STRUCT*)delayAddr;
    pDelayInfo->InitDelay = 3;
    pDelayInfo->EffectDelay = 5;
    pDelayInfo->AwbDelay = 5;
}
void HI707GetAEAWBLock(UINT32 *pAElockRet32,UINT32 *pAWBlockRet32)
{
    *pAElockRet32 = 1;
    *pAWBlockRet32 = 1;
    SENSORDB("HI707GetAEAWBLock,AE=%d ,AWB=%d\n,",*pAElockRet32,*pAWBlockRet32);
}

UINT32 HI707FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                             UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    //UINT16 u2Temp = 0;
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    SENSORDB("HHL [Enter]:HI707 Feature Control func:FeatureId = %d\n",FeatureId);

    switch (FeatureId)
    {
    case SENSOR_FEATURE_GET_RESOLUTION:
        *pFeatureReturnPara16++=HI707_IMAGE_SENSOR_FULL_WIDTH;
        *pFeatureReturnPara16=HI707_IMAGE_SENSOR_FULL_HEIGHT;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PERIOD:
        *pFeatureReturnPara16++=HI707_IMAGE_SENSOR_PV_WIDTH;//+HI707_sensor.pv_dummy_pixels;
        *pFeatureReturnPara16=HI707_IMAGE_SENSOR_PV_HEIGHT;//+HI707_sensor.pv_dummy_lines;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        //*pFeatureReturnPara32 = HI707_sensor_pclk/10;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_ESHUTTER:

        break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
        HI707_night_mode((BOOL) *pFeatureData16);
        break;
    case SENSOR_FEATURE_SET_GAIN:
        break;
    case SENSOR_FEATURE_SET_FLASHLIGHT:
        break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
        break;
    case SENSOR_FEATURE_SET_REGISTER:
        HI707_write_cmos_sensor(0x3, pSensorRegData->RegAddr>>8);
        HI707_write_cmos_sensor(pSensorRegData->RegAddr&0xff, pSensorRegData->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        HI707_write_cmos_sensor(0x3, pSensorRegData->RegAddr>>8);
        pSensorRegData->RegData = HI707_read_cmos_sensor(pSensorRegData->RegAddr&0xff);
        break;
    case SENSOR_FEATURE_GET_CONFIG_PARA:
        memcpy(pSensorConfigData, &HI707SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
        *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
        break;
    case SENSOR_FEATURE_SET_CCT_REGISTER:
    case SENSOR_FEATURE_GET_CCT_REGISTER:
    case SENSOR_FEATURE_SET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
    case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
    case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
    case SENSOR_FEATURE_GET_GROUP_INFO:
    case SENSOR_FEATURE_GET_ITEM_INFO:
    case SENSOR_FEATURE_SET_ITEM_INFO:
    case SENSOR_FEATURE_GET_ENG_INFO:
        break;
    case SENSOR_FEATURE_GET_GROUP_COUNT:
        // *pFeatureReturnPara32++=0;
        //*pFeatureParaLen=4;
        break;

    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
        // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
        // if EEPROM does not exist in camera module.
        *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_YUV_CMD:
        HI707YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
        break;
    case SENSOR_FEATURE_SET_VIDEO_MODE:
        HI707YUVSetVideoMode(*pFeatureData16);
        break;
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
        HI707_GetSensorID(pFeatureData32);
        break;
    case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
        HI707GetAFMaxNumFocusAreas(pFeatureReturnPara32);
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
        HI707GetAEMaxNumMeteringAreas(pFeatureReturnPara32);
        *pFeatureParaLen=4;
        break;
	//                                                               
    case SENSOR_FEATURE_SET_AE_WINDOW:
        SENSORDB("SENSOR_FEATURE_SET_AE_WINDOW\n");
        HI707SetAEWindow(*pFeatureReturnPara32);
        break;
	//                                                               
    case SENSOR_FEATURE_GET_EXIF_INFO:
        SENSORDB("SENSOR_FEATURE_GET_EXIF_INFO\n");
        SENSORDB("EXIF addr = 0x%x\n",*pFeatureData32);
        HI707GetExifInfo(*pFeatureData32);
        break;
    case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
        HI707GetAEAWBLock((*pFeatureData32),*(pFeatureData32+1));
        break;
    case SENSOR_FEATURE_GET_DELAY_INFO:
        SENSORDB("SENSOR_FEATURE_GET_DELAY_INFO\n");
        HI707GetDelayInfo(*pFeatureData32);
        break;
    case SENSOR_FEATURE_SET_YUV_3A_CMD:
        HI707_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*pFeatureData32);
        break;
    case SENSOR_FEATURE_SET_TEST_PATTERN:
        HI707SetTestPatternMode((BOOL)*pFeatureData16);
        break;
    case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
        *pFeatureReturnPara32=HI707_TEST_PATTERN_CHECKSUM;
        *pFeatureParaLen=4;
        break;

		//                                                                           
         case SENSOR_FEATURE_GET_SENSOR_VIEWANGLE:
             {
                 UINT32 *pHorFOV = (UINT32*)pFeatureReturnPara32[0];
                 UINT32 *pVerFOV = (UINT32*)pFeatureReturnPara32[1];

                 SENSORDB("SENSOR_FEATURE_GET_SENSOR_VIEWANGLE\n");
                 *pHorFOV = (0x37)<<16 | (0x37); // 16:9 = 55, 4:3 = 55 // (59)<<16 | (55)
                 *pVerFOV = (0x31)<<16 | (0x31); // 16:9 = 49, 4:3 = 49 // (49)<<16 | (49)
                 *pFeatureParaLen = 8;
                 break;
             }
         //                                                                           
    default:
        break;
    }
    return ERROR_NONE;
}    /* HI707FeatureControl() */


SENSOR_FUNCTION_STRUCT    SensorFuncHI707=
{
    HI707Open,
    HI707GetInfo,
    HI707GetResolution,
    HI707FeatureControl,
    HI707Control,
    HI707Close
};

UINT32 HI707_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncHI707;

    return ERROR_NONE;
}    /* SensorInit() */
