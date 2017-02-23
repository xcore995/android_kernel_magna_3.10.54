#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/err.h>

#include "Tcc353xDriver/PAL/tcpal_os.h"
#include "tcc353x_hal.h"

#include "../broadcast_dmb_typedef.h"
#include "../broadcast_dmb_drv_ifdef.h"
#include "broadcast_tcc353x.h"
#include <mach/board_lge.h> 
#if defined(CONFIG_ARCH_MT6582)  || defined(CONFIG_ARCH_MT6572)  // MediaTek platform
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <cust_eint_md1.h>
#include <cust_gpio_usage.h>
#include <cust_eint.h>
#include <linux/rtpm_prio.h>
#include <mach/mt_pm_ldo.h>    // CORE POWER LDO
#include <mach/mt_spi.h>
#endif //defined(CONFIG_ARCH_MT6582) // MediaTek platform

TcpalSemaphore_t Tcc353xDrvSem;

extern void tcc353x_isr_kthread_start (void);
extern void tcc353x_isr_kthread_stop (void);

extern I32S Tcc353xTccspiOpen(I32S _moduleIndex);
extern void Tcc353xTccspiInit(void);
extern I32S Tcc353xTccspiRead(I32S _moduleIndex, 
	I32S _chipAddress, I08U _registerAddr, I08U * _outData, I32S _size);
extern I32S Tcc353xTccspiClose(I32S _moduleIndex);

extern int g_lge_dtv_flag; 

#define _REG_CHIP_ID_ (0x0C)
#define _DEVICE_ADDR_ (0xA8>>1)

static Device_drv device_tcc353x = {
	&broadcast_tcc353x_drv_if_power_on,
	&broadcast_tcc353x_drv_if_power_off,
	&broadcast_tcc353x_drv_if_open,
	&broadcast_tcc353x_drv_if_close,
	&broadcast_tcc353x_drv_if_set_channel,
	&broadcast_tcc353x_drv_if_resync,
	&broadcast_tcc353x_drv_if_detect_sync,
	&broadcast_tcc353x_drv_if_get_sig_info,
	&broadcast_tcc353x_drv_if_get_ch_info,
	&broadcast_tcc353x_drv_if_get_dmb_data,
	&broadcast_tcc353x_drv_if_reset_ch,
	&broadcast_tcc353x_drv_if_user_stop,
	&broadcast_tcc353x_drv_if_select_antenna,
	&broadcast_tcc353x_drv_if_read_control,
	&broadcast_tcc353x_drv_if_get_mode,
};

/*#define _NOT_USE_WAKE_LOCK_*/

struct broadcast_tcc353x_ctrl_data
{
	int			pwr_state;
	struct wake_lock	wake_lock;
	struct spi_device	*spi_dev;
#if !defined (_TCSPI_ONLY_)
	struct i2c_client	*pclient;
#endif
};

static struct broadcast_tcc353x_ctrl_data  IsdbCtrlInfo;

#if !defined (_TCSPI_ONLY_)
struct i2c_client*	TCC_GET_I2C_DRIVER(void)
{
	return IsdbCtrlInfo.pclient;
}
#endif

struct spi_device *TCC_GET_SPI_DRIVER(void)
{
	return IsdbCtrlInfo.spi_dev;
}

int tcc353x_power_on(void)
{
	TcpalPrintStatus((I08S *)"0807 16th TCC3536 [%s]\n", __func__);

	if(IsdbCtrlInfo.pwr_state != 1)
	{
#ifndef _NOT_USE_WAKE_LOCK_
		wake_lock(&IsdbCtrlInfo.wake_lock);
#endif
		TchalPowerOnDevice();
	}
	else
	{
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][debug_info] aready on!! \n");
	}
	IsdbCtrlInfo.pwr_state = 1;
	return OK;
}

int tcc353x_is_power_on()
{
	return (int)IsdbCtrlInfo.pwr_state;
}

int tcc353x_power_off(void)
{
	if(IsdbCtrlInfo.pwr_state == 0)
	{
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][debug_info] Isdb_tcc353x_power is immediately off\n");
		return OK;
	}
	else
	{
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] Isdb_tcc353x_power_off\n");
		TchalPowerDownDevice();
	}

#ifndef _NOT_USE_WAKE_LOCK_
	wake_unlock(&IsdbCtrlInfo.wake_lock);
#endif
	IsdbCtrlInfo.pwr_state = 0;

	return OK;
}

static int broadcast_dmb_sample_check_chip_id(void)
{
	int rc = ERROR;
	I08U chip_id = 0;

	TchalPowerOnDevice();
	Tcc353xTccspiOpen(0);
	
	Tcc353xTccspiRead(0, _DEVICE_ADDR_, _REG_CHIP_ID_, &chip_id, 1);
	Tcc353xTccspiClose(0);
	TchalPowerDownDevice();

	if(chip_id==0x33) {
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] tcc3536 chipID matched! chip_id=0x%02x\n",chip_id);
		rc = OK;
	} else {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] tcc3536 chipID missmatched! chip_id=0x%02x\n",chip_id);
		rc = ERROR;
	}
	
	return rc;
}

#if defined (_TCSPI_ONLY_)
static int broadcast_Isdb_spi_probe(struct spi_device *spi_dev)
{
	int rc = 0;
#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6572)   // MediaTek platform
    struct mt_chip_conf *mtk_spi_config = NULL;

    mtk_spi_config = ( struct mt_chip_conf * ) spi_dev->controller_data;
    if(mtk_spi_config)
    {
        mtk_spi_config->setuptime = 3;
        mtk_spi_config->holdtime = 3;
        mtk_spi_config->high_time = 2;
        mtk_spi_config->low_time = 1;
        mtk_spi_config->cs_idletime = 2;
        mtk_spi_config->ulthgh_thrsh = 0;
        mtk_spi_config->cpol = 0;
        mtk_spi_config->cpha = 0;
        mtk_spi_config->rx_mlsb = 1;
        mtk_spi_config->tx_mlsb = 1;
        mtk_spi_config->tx_endian = 0;
        mtk_spi_config->rx_endian = 0;
        mtk_spi_config->com_mod = DMA_TRANSFER;
        mtk_spi_config->pause = 0;
        mtk_spi_config->finish_intr = 1;
        mtk_spi_config->deassert = 0;
        mtk_spi_config->ulthigh = 0;
        mtk_spi_config->tckdly = 0;
    }
    TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] tcc3536 tckdly=0, %s\n",__func__);
#endif
    TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] tcc3536 %s\n",__func__);
	spi_dev->mode = SPI_MODE_0;
	spi_dev->bits_per_word = 8;
	//spi_dev->max_speed_hz = 34*1000*1000;	/* 34 MHz	*/
	rc = spi_setup(spi_dev);
	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] tcc3536 spi_setup complete\n");

	IsdbCtrlInfo.spi_dev = spi_dev;
	IsdbCtrlInfo.pwr_state = 0;
	TcpalPrintStatus((I08S *)"[dtv][tcc3536][debug_info] tcc3536 probe spi=%p\n", spi_dev);

	TcpalCreateSemaphore(&Tcc353xDrvSem,
			(I08S *) "Tcc353xDriverControlSemaphore", 1);

	TchalInit();
	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] tcc3536 tchal init comp\n");

	TcpalIrqDisable();
	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] tcc3536 irq disable comp\n");

	tcc353x_isr_kthread_start ();
	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] tcc3536 kthread start comp\n");

	TcpalRegisterIrqHandler();
	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] tcc3536 register irq handler comp\n");

	TcpalSemaphoreLock(&Tcc353xDrvSem);
	if(broadcast_dmb_sample_check_chip_id() != OK) {
		rc = ERROR;
		TcpalSemaphoreUnLock(&Tcc353xDrvSem);
		TcpalDeleteSemaphore(&Tcc353xDrvSem);
		return rc;
	}
	TcpalSemaphoreUnLock(&Tcc353xDrvSem);

#ifndef _NOT_USE_WAKE_LOCK_
	wake_lock_init(&IsdbCtrlInfo.wake_lock, WAKE_LOCK_SUSPEND,
					dev_name(&spi_dev->dev));	
#endif

	rc = broadcast_dmb_drv_start(&device_tcc353x);	
	if (rc) 
	{
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] failed to load\n");
		return rc;
	}

	TcpalPrintStatus((I08S *)"[dtv][tcc3536][debug_info] broadcast_add_driver rc=%d\n", rc);
	return rc;
}

static int broadcast_Isdb_spi_remove(struct spi_device *spi)
{
	int rc = 0;

	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] tcc3536 %s\n",__func__);

	tcc353x_isr_kthread_stop ();
	TcpalUnRegisterIrqHandler();
	
#ifndef _NOT_USE_WAKE_LOCK_
	wake_lock_destroy(&IsdbCtrlInfo.wake_lock);
#endif
	memset((unsigned char*)&IsdbCtrlInfo, 0x0, sizeof(struct broadcast_tcc353x_ctrl_data));
	TcpalDeleteSemaphore(&Tcc353xDrvSem);
	return rc;
}

static int broadcast_Isdb_spi_suspend(struct spi_device *spi, pm_message_t mesg)
{
	int rc = 0;

	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok][%s]\n", __func__);
	return rc;
}

static int broadcast_Isdb_spi_resume(struct spi_device *spi)
{
	int rc = 0;

	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok][%s]\n", __func__);
	return rc;
}

static struct of_device_id broadcast_spi_table[] = {
	{
		//.compatible = "raontech, isdbt-mtv222",
		.compatible = "dtv_spi",
	},
	{}
};

static struct spi_driver broadcast_Isdb_driver = {
	.driver = {
		//.name		= DRIVER_NAME,
		.name = "dtv_spi",
		.of_match_table = broadcast_spi_table,
		.bus	= &spi_bus_type,
		.owner = THIS_MODULE,
	},

	.probe = broadcast_Isdb_spi_probe,
	.suspend = broadcast_Isdb_spi_suspend,
	.resume	= broadcast_Isdb_spi_resume,
	.remove = __exit_p(broadcast_Isdb_spi_remove),
};

#else
static int broadcast_Isdb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	int addr;

	TcpalPrintLog("[dtv][tcc3536][ok] broadcast_Isdb_i2c_probe client=0x%X\n", (unsigned int)client);
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TcpalPrintErr("[dtv][tcc3536][error] need I2C_FUNC_I2C\n");
		return -ENODEV;
	}
	/* taew00k.kang added for Device Tree Structure 2013-06-04 [start] */
	addr = client->addr; //Slave Addr
	pr_err("[dtv][tcc3536][debug_info] i2c Slaveaddr=0x%x\n", addr);
	/* taew00k.kang added for Device Tree Structure 2013-06-04 [end] */

	TcpalCreateSemaphore(&Tcc353xDrvSem,
			(I08S *) "Tcc353xDriverControlSemaphore", 1);

	IsdbCtrlInfo.pclient = client;
	//i2c_set_clientdata(client, (void*)&IsdbCtrlInfo.pclient);

	TchalInit();
#ifndef _NOT_USE_WAKE_LOCK_
	wake_lock_init(&IsdbCtrlInfo.wake_lock, WAKE_LOCK_SUSPEND,
					dev_name(&client->dev));	
#endif

	return rc;
}

static int broadcast_Isdb_i2c_remove(struct i2c_client* client)
{
	int rc = 0;

	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok][%s]\n", __func__);
#ifndef _NOT_USE_WAKE_LOCK_
	wake_lock_destroy(&IsdbCtrlInfo.wake_lock);
#endif
	memset((unsigned char*)&IsdbCtrlInfo, 0x0, sizeof(struct broadcast_tcc353x_ctrl_data));
	TcpalDeleteSemaphore(&Tcc353xDrvSem);
	return rc;
}

static int broadcast_Isdb_i2c_suspend(struct i2c_client* client, pm_message_t mesg)
{
	int rc = 0;

	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok][%s]\n", __func__);
	return rc;
}

static int broadcast_Isdb_i2c_resume(struct i2c_client* client)
{
	int rc = 0;

	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok][%s]\n", __func__);
	return rc;
}

static const struct i2c_device_id isdbt_tcc353x_id[] = {
/* taew00k.kang added for Device Tree Structure 2013-06-04 [start] */
	{"tcc353x_tcspi",	0},
/* taew00k.kang added for Device Tree Structure 2013-06-04 [end] */
	{},
};

MODULE_DEVICE_TABLE(i2c, isdbt_tcc353x_id);


/* taew00k.kang added for Device Tree Structure 2013-06-04 [start] */
static struct of_device_id tcc353x_tcspi_table[] = {
	{
		.compatible = "telechips,tcc353x-tcspi", //Compatible node must match dts
	},
	{ },
};
/* taew00k.kang added for Device Tree Structure 2013-06-04 [end] */

static struct i2c_driver broadcast_Isdb_driver = {
	.driver = {
		.name = "tcc353x_tcspi",
		.owner = THIS_MODULE,
		.of_match_table = tcc353x_tcspi_table,
	},
	.probe = broadcast_Isdb_i2c_probe,
	.remove	= __devexit_p(broadcast_Isdb_i2c_remove),
	.id_table = isdbt_tcc353x_id,
	.suspend = broadcast_Isdb_i2c_suspend,
	.resume  = broadcast_Isdb_i2c_resume,
};
#endif

int __init broadcast_tcc353x_dmb_drv_init(void)
{
	int rc = ERROR;

#if defined (CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6572)  // MediaTek platform
#if defined (TARGET_MT6582_Y90)
// Y90 model "EVB" can not get g_lge_dtv value. so if Y90 model "EVB", always set DTV func no matter g_lge_dtv value is.
    if(!g_lge_dtv_flag && lge_get_board_revno() >= HW_REV_A)
#else //defined (TARGET_MT6582_Y90)
    if(!g_lge_dtv_flag)
#endif //defined (TARGET_MT6582_Y90)
    {
        TcpalPrintStatus((I08S *) "[dtv] it's not DTV Model, lge_get_board_revno()=%d, g_lge_dtv_flag=%d\n", lge_get_board_revno(), g_lge_dtv_flag);
        return rc;
    }
    else
    {
        TcpalPrintStatus((I08S *) "[dtv] it's DTV Model\n");
    }
    TcpalPrintStatus((I08S *) "[dtv][tcc3536] it's TCC3536 Model, Rev=%d, g_lge_dtv_flag=%d\n",lge_get_board_revno(),g_lge_dtv_flag);
#endif //defined (CONFIG_ARCH_MT6582)  || defined(CONFIG_ARCH_MT6572)  // MediaTek platform
    if(broadcast_dmb_drv_check_module_init() != OK) 
    {
        rc = ERROR;
        return rc;
    }
#if defined (_TCSPI_ONLY_)
	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok][%s add spi driver]\n", __func__);
	rc =  spi_register_driver(&broadcast_Isdb_driver);
#else
	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok][%s add i2c driver]\n", __func__);
	rc = i2c_add_driver(&broadcast_Isdb_driver);
#endif	

	return rc;
}

static void __exit broadcast_tcc353x_dmb_drv_exit(void)
{
#if defined (_TCSPI_ONLY_)
#else
	i2c_del_driver(&broadcast_Isdb_driver);
#endif
}

module_init(broadcast_tcc353x_dmb_drv_init);
module_exit(broadcast_tcc353x_dmb_drv_exit);
MODULE_DESCRIPTION("broadcast_dmb_drv_init");
MODULE_LICENSE("TELECHIPS");
