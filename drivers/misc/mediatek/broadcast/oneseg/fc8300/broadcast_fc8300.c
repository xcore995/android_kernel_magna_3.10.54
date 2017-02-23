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

#include "broadcast_dmb_typedef.h"
#include "broadcast_dmb_drv_ifdef.h"
#include "broadcast_fc8300.h"
#include "fci_types.h"
#include "fci_oal.h"
#include "bbm.h"
#include "fc8300_drv_api.h"
#include "fc8300_isr.h"
#include "fci_ringbuffer.h"

#if defined(CONFIG_ARCH_MT6582)    // MediaTek platform
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <cust_eint_md1.h>
#include <cust_gpio_usage.h>
#include <cust_eint.h>
#include <linux/rtpm_prio.h>
#include <mach/mt_pm_ldo.h>    // FC8150 CORE  POWER LDO
#endif //defined(CONFIG_ARCH_MT6582) // MediaTek platform

#include <mach/board_lge.h>
//TcpalSemaphore_t fc8300DrvSem;

extern int g_lge_dtv_flag;
//static struct msm_xo_voter *xo_handle_tcc;
struct ISDBT_INIT_INFO_T *hInit;

#define RING_BUFFER_SIZE    (188 * 320 * 20)
static struct task_struct *isdbt_kthread = NULL;
struct fci_ringbuffer        RingBuffer;
static wait_queue_head_t isdbt_isr_wait;
u8 irq_error_cnt;
static u8 isdbt_isr_sig=0;
static u8 isdbt_isr_start=0;
u32 totalTS=0;
u32 totalErrTS=0;

enum ISDBT_MODE{
    ISDBT_POWERON       = 0,
    ISDBT_POWEROFF        = 1,
    ISDBT_DATAREAD        = 2
};
enum ISDBT_MODE driver_mode = ISDBT_POWEROFF;

u8 static_ringbuffer[RING_BUFFER_SIZE];
static DEFINE_MUTEX(ringbuffer_lock);

/*#define _NOT_USE_WAKE_LOCK_*/

#define MTK_FTRACE_TEST

struct broadcast_fc8300_ctrl_data
{
    int            pwr_state;
    struct wake_lock    wake_lock;
    struct spi_device    *spi_dev;
    struct i2c_client    *pclient;
    boolean probe_check_flag;
};

static struct broadcast_fc8300_ctrl_data  IsdbCtrlInfo;

struct i2c_client*    FCI_GET_I2C_DRIVER(void)
{
    return IsdbCtrlInfo.pclient;
}

struct spi_device*    FCI_GET_SPI_DRIVER(void)
{
    return IsdbCtrlInfo.spi_dev;
}

static Device_drv device_fc8300 = {
    &broadcast_fc8300_drv_if_power_on,
    &broadcast_fc8300_drv_if_power_off,
    &broadcast_fc8300_drv_if_open,
    &broadcast_fc8300_drv_if_close,
    &broadcast_fc8300_drv_if_set_channel,
    &broadcast_fc8300_drv_if_resync,
    &broadcast_fc8300_drv_if_detect_sync,
    &broadcast_fc8300_drv_if_get_sig_info,
    &broadcast_fc8300_drv_if_get_ch_info,
    &broadcast_fc8300_drv_if_get_dmb_data,
    &broadcast_fc8300_drv_if_reset_ch,
    &broadcast_fc8300_drv_if_user_stop,
    &broadcast_fc8300_drv_if_select_antenna,
    &broadcast_fc8300_drv_if_isr,
    &broadcast_fc8300_drv_if_read_control,
    &broadcast_fc8300_drv_if_get_mode,
};

unsigned int overrun_count = 0;
unsigned int isr_flag = 0;

int fc8300_power_on(void)
{
    int i =0;

    print_log(NULL, "[FC8300]power on test overrun_count %d!! \n", overrun_count);

    overrun_count = 0;

    if(IsdbCtrlInfo.pwr_state != 1)
    {

#ifndef _NOT_USE_WAKE_LOCK_
        wake_lock(&IsdbCtrlInfo.wake_lock);
#endif
        while(driver_mode == ISDBT_DATAREAD)
        {
            msWait(100);
            print_log(NULL, "[FC8300]ISDBT_DATARREAD mode i=(%d)\n", i);

            if(i++>5)
                break;
        }

        tunerbb_drv_hw_init();

        driver_mode = ISDBT_POWERON;
#if 0
        rc = msm_xo_mode_vote(xo_handle_tcc, MSM_XO_MODE_ON);
        if(rc < 0) {
            pr_err("Configuring MSM_XO_MODE_ON failed (%d)\n", rc);
            msm_xo_put(xo_handle_tcc);
            return FALSE;
        }
#endif
    }
    else
    {
        print_log(NULL, "[FC8300]aready on!! \n");
    }

    IsdbCtrlInfo.pwr_state = 1;



    return OK;
}

int fc8300_is_power_on()
{
    return (int)IsdbCtrlInfo.pwr_state;
}

int fc8300_power_off(void)
{
    driver_mode = ISDBT_POWEROFF;
    if(IsdbCtrlInfo.pwr_state == 0)
    {
        print_log(NULL, "Isdb_tcc3530_power is immediately off\n");
        return OK;
    }
    else
    {
    #if 0
        if(xo_handle_tcc != NULL) {
            msm_xo_mode_vote(xo_handle_tcc, MSM_XO_MODE_OFF);
        }
    #endif
        print_log(NULL, "Isdb_tcc3530_power_off\n");
        tunerbb_drv_hw_deinit();
    }

#ifndef _NOT_USE_WAKE_LOCK_
    wake_unlock(&IsdbCtrlInfo.wake_lock);
#endif
    IsdbCtrlInfo.pwr_state = 0;

    return OK;
}

unsigned int fc8300_get_ts(void *buf, unsigned int size)
{
    s32 avail;
    ssize_t len, total_len = 0;

    if ( fci_ringbuffer_empty(&RingBuffer) )
    {
        //print_log(hInit, "return fci_ringbuffer_empty EWOULDBLOCK\n");
        return 0;
    }

    //mutex_lock(&ringbuffer_lock);
    avail = fci_ringbuffer_avail(&RingBuffer);

    if (size >= avail)
        len = avail;
    else
        len = size - (size % 188);

    total_len = fci_ringbuffer_read_user(&RingBuffer, buf, len);
    //mutex_unlock(&ringbuffer_lock);

    return total_len;
}

void isdbt_irq(void)
{
    //print_log(NULL, "[FC8300] isdbt_irq MODE %d  ERR : %d, SIGCNT : %d\n", driver_mode, irq_error_cnt, isdbt_isr_sig);
    //bbm_byte_write(NULL, DIV_MASTER, 0x26, 0x00);

    if((driver_mode == ISDBT_POWEROFF)||(isdbt_isr_start)) {
        print_log(0, "fc8300 isdbt_irq : abnormal Interrupt occurred fc8300 power off state.cnt : %d\n", irq_error_cnt);
        irq_error_cnt++;
        isdbt_isr_start = 0;
    }
    else {
        isdbt_isr_sig++;
        wake_up(&isdbt_isr_wait);
    }

    mt_eint_unmask(CUST_EINT_DTV_NUM);
}

int data_callback(u32 hDevice, u8 bufid, u8 *data, int len)
{
    int i;
    unsigned long ts_error_count = 0;
    totalTS +=(len/188);

    for(i=0;i<len;i+=188)
    {
        if((data[i+1]&0x80)||data[i]!=0x47) {
            ts_error_count++;
        }
    }

    if(ts_error_count > 0)
    {
        totalErrTS += ts_error_count;
        print_log(NULL, "[FC8300] data_callback totalErrTS : %d, len : %d\n", totalErrTS, len);
    }
    //mutex_lock(&ringbuffer_lock);
    if(fci_ringbuffer_free(&RingBuffer) < len )
        FCI_RINGBUFFER_SKIP(&RingBuffer, len);

    fci_ringbuffer_write(&RingBuffer, data, len);
    //wake_up_interruptible(&(RingBuffer.queue));

    //mutex_unlock(&ringbuffer_lock);

    return 0;
}

static int isdbt_thread(void *hDevice)
{
    struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

    #ifdef MTK_FTRACE_TEST
    unsigned long previous_time = 0;
    unsigned int isdbt_ftrace_mode = 0;
    #endif

    struct sched_param param = { .sched_priority = RTPM_PRIO_DTV };
    sched_setscheduler ( current, SCHED_FIFO, &param );

    set_user_nice(current, -20);

    print_log(hInit, "isdbt_kthread enter\n");

    bbm_com_ts_callback_register((u32)hInit, data_callback);

    init_waitqueue_head(&isdbt_isr_wait);

    while(1)
    {
        wait_event_interruptible(isdbt_isr_wait, isdbt_isr_sig || kthread_should_stop());
        #if 0
        if (irq_error_cnt >= 1){
            print_log(0, "fc8300 isdbt_irq : abnormal Interrupt occurred fc8300 power off state.cnt : %d\n", irq_error_cnt);
            irq_error_cnt = 0;
        }
        #endif
        //print_log(NULL, "[FC8300] isdbt_thread mode %d sigcnt : %d\n", driver_mode, isdbt_isr_sig);

        //isdbt_thread_check_cnt++;
        if(driver_mode == ISDBT_POWERON)
        {
            driver_mode = ISDBT_DATAREAD;
            bbm_com_isr(NULL);
            driver_mode = ISDBT_POWERON;
        }

        if(isdbt_isr_sig > 0)
        {
            #ifdef MTK_FTRACE_TEST
            extern void tracing_off(void);
            isdbt_isr_sig--;
            if(isdbt_isr_sig > 0) {
                if(isdbt_ftrace_mode == 0)
                {
                    if(isdbt_isr_sig > 1)
                    {
                        tracing_off();
                        isdbt_ftrace_mode = 1;
                    }
                    else if(isdbt_isr_sig)
                    {
                        if((previous_time) && ((unsigned long)jiffies - previous_time) < 300) //3sec)
                        {
                            tracing_off();
                            isdbt_ftrace_mode = 1;
                        }
                        previous_time = (unsigned long)jiffies;
                    }
                }
                isdbt_isr_sig = 0;
            }
            #else
            isdbt_isr_sig--;
            if(isdbt_isr_sig > 0) {
                isdbt_isr_sig = 0;
            }
            #endif
        }

         if (kthread_should_stop())
            break;
    }

    bbm_com_ts_callback_deregister();

    print_log(NULL, "isdbt_kthread exit\n");

    return 0;
}

void fci_irq_disable(void)
{
    //mt_eint_mask(CUST_EINT_DTV_NUM);
    isdbt_isr_sig = 0;
    print_log(NULL, "[fc8300]fci_irq_disable %d\n", IsdbCtrlInfo.spi_dev->irq);
}

void fci_irq_enable(void)
{
    //enable_irq(IsdbCtrlInfo.spi_dev->irq);
    isdbt_isr_sig = 0;
    isdbt_isr_start = 1;
    mt_eint_unmask(CUST_EINT_DTV_NUM);
    print_log(NULL, "[fc8300]fci_irq_enable %d\n", IsdbCtrlInfo.spi_dev->irq);
}

void broadcast_fci_ringbuffer_flush(void)
{
    fci_ringbuffer_flush(&RingBuffer);
}

static int broadcast_dmb_fc8300_check_chip_id(void)
{
    int rc = ERROR;

    rc = fc8300_power_on();
    rc = bbm_com_hostif_select(NULL, BBM_SPI);
    rc |= bbm_com_probe(NULL, DIV_BROADCAST);

    print_log(NULL, "broadcast_dmb_fc8300_check_chip_id ret(%d)\n", rc);

    fc8300_power_off();

    return rc;
}

static int broadcast_spi_suspend(struct spi_device *spi, pm_message_t mesg)
{
    print_log(NULL, "Suspend\n");
    return 0;
}

static int broadcast_spi_resume(struct spi_device *spi)
{
    print_log(NULL, "Resume\n");
    return 0;
}

static int broadcast_spi_remove(struct spi_device *spi)
{
    int ret;
    print_log(NULL, "broadcast_spi_remove \n");

    #if defined(CONFIG_ARCH_MT6582)    // MediaTek platform
    // gpio
    ret = mt_set_gpio_mode(GPIO_FULL_SEG_INT, GPIO_FULL_SEG_INT_M_GPIO);
    print_log(0,"[1seg][MTK][isdbt_exit] GPIO_FULL_SEG_INT mt_set_gpio_mode = %d!!!\n", ret);
    ret = mt_set_gpio_dir(GPIO_FULL_SEG_INT, GPIO_DIR_IN);
    print_log(0,"[1seg][MTK][isdbt_exit] GPIO_FULL_SEG_INT mt_set_gpio_dir = %d!!!\n", ret);

    // mask
    //mt_eint_mask(CUST_EINT_DTV_NUM);
#else //defined(CONFIG_ARCH_MT6582) // MediaTek platform
        free_irq(GPIO_ISDBT_IRQ, NULL);
#endif //defined(CONFIG_ARCH_MT6582) // MediaTek platform

     if(isdbt_kthread)
    {
        kthread_stop(isdbt_kthread);
        isdbt_kthread = NULL;
    }
     bbm_com_hostif_deselect(NULL);

    wake_lock_destroy(&IsdbCtrlInfo.wake_lock);

    return 0;
}

static int broadcast_spi_probe(struct spi_device *spi)
{
    int rc;
    int ret;

    spi->mode             = SPI_MODE_0;
    spi->bits_per_word    = 8;
    //spi->max_speed_hz     = (25000*1000);

    rc = spi_setup(spi);
    if (rc) {
        print_log(NULL, "Spi setup error(%d)\n", rc);
        return rc;
    }

    IsdbCtrlInfo.spi_dev = spi;

#ifndef _NOT_USE_WAKE_LOCK_
    wake_lock_init(&IsdbCtrlInfo.wake_lock, WAKE_LOCK_SUSPEND,
                    dev_name(&spi->dev));
#endif
    fci_ringbuffer_init(&RingBuffer, &static_ringbuffer[0], RING_BUFFER_SIZE);

    tunerbb_drv_hw_setting();

    if(broadcast_dmb_fc8300_check_chip_id() != OK) {
        rc = ERROR;
    }

     if (!isdbt_kthread)
    {
        print_log(NULL, "kthread run\n");
        isdbt_kthread = kthread_run(isdbt_thread, NULL, "isdbt_thread");
    }
 #if defined(CONFIG_ARCH_MT6582)    // MediaTek platform
    // configuration for detect
    ret = (int)mt_eint_set_sens(CUST_EINT_DTV_NUM, CUST_EINT_EDGE_SENSITIVE);
    print_log(0,"[1seg][MTK] GPIO_1SEG_INT mt_eint_set_sens = %d!!!\n", ret);
    //mt_eint_set_hw_debounce(CUST_EINT_DTV_NUM, CUST_EINT_DTV_DEBOUNCE_CN);
    //print_log(0,"[1seg][MTK] GPIO_1SEG_INT mt_eint_set_hw_debounce !!!\n");

    //gpio
    ret = mt_set_gpio_mode(GPIO_FULL_SEG_INT, GPIO_FULL_SEG_INT_M_EINT);
    print_log(0,"[1seg][MTK] GPIO_FULL_SEG_INT mt_set_gpio_mode = %d!!!\n", ret);
    ret = mt_set_gpio_dir(GPIO_FULL_SEG_INT, GPIO_DIR_IN);
    print_log(0,"[1seg][MTK] GPIO_FULL_SEG_INT mt_set_gpio_dir = %d!!!\n", ret);

    ret = mt_set_gpio_pull_enable(GPIO_FULL_SEG_INT, GPIO_PULL_ENABLE);
    print_log(0, "[1seg][MTK] mt_set_gpio_pull_enable(GPIO_FULL_SEG_INT, GPIO_PULL_ENABLE)(%d)\n", ret);

    ret = mt_set_gpio_pull_select(GPIO_FULL_SEG_INT,GPIO_PULL_DOWN);
    print_log(0, "[1seg][MTK] mt_set_gpio_pull_select(GPIO_FULL_SEG_INT,GPIO_PULL_DOWN)(%d)\n", ret);


    // irq handler register
    mt_eint_registration(CUST_EINT_DTV_NUM, EINTF_TRIGGER_FALLING/*CUST_EINT_DTV_POLARITY*/, isdbt_irq, 0);
    print_log(0,"[1seg][MTK] GPIO_1SEG_INT mt_eint_registration!!!\n");

    // disable irq
    //mt_eint_mask(CUST_EINT_DTV_NUM);
#else //defined(CONFIG_ARCH_MT6582) // MediaTek platform
    res = request_irq(gpio_to_irq(GPIO_ISDBT_IRQ), isdbt_irq, IRQF_DISABLED | IRQF_TRIGGER_FALLING, FC8150_NAME, NULL);
    if(res)
        print_log(hInit, "[1seg] dmb rquest irq fail : %d\n", res);
#endif //defined(CONFIG_ARCH_MT6582) // MediaTek platform

    print_log(NULL, "request_irq = %d\n", rc);

    ret = broadcast_dmb_drv_start(&device_fc8300);
    if (ret) {
        print_log(NULL, "Failed to load (%d)\n", ret);
        rc = ERROR;
    }

    if (rc < 0)
        goto free_irq;

    fci_irq_disable(); /* Must disabled */

    print_log(NULL, "End.\n");

    return 0;

free_irq:
    broadcast_spi_remove(IsdbCtrlInfo.spi_dev);

    return rc;
}

struct spi_device_id spi_dtv_id_table = {"dtv_spi", 0};

static struct spi_driver broadcast_spi_driver = {
	.driver = {
		.name = "dtv_spi",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = broadcast_spi_probe,
	.remove= __exit_p(broadcast_spi_remove),
	.id_table = &spi_dtv_id_table,
};
/*
static struct of_device_id broadcast_spi_table[] = {
    {
        //.compatible = "raontech, isdbt-mtv222",
        .compatible = "dtv_spi",
    },
    {}
};

static struct spi_driver broadcast_spi_driver = {
    .driver = {
        //.name        = DRIVER_NAME,
        .name = "dtv_spi",
        .of_match_table = broadcast_spi_table,
        .bus    = &spi_bus_type,
        .owner = THIS_MODULE,
    },

    .probe = broadcast_spi_probe,
    .suspend = broadcast_spi_suspend,
    .resume    = broadcast_spi_resume,
    .remove    = __exit_p(broadcast_spi_remove),
};*/

static int __init broadcast_dmb_fc8300_drv_init(void)
{
    int ret = 0;

    if(!g_lge_dtv_flag)
    {
        print_log(0, "it's not DTV Model");
        return ERROR;
    }

    // Todo(add revision check rountine)

    if(broadcast_dmb_drv_check_module_init() != OK) {
        ret = ERROR;
        return ret;
    }

    ret = spi_register_driver(&broadcast_spi_driver);

    return ret;
}

static void __exit broadcast_dmb_fc8300_drv_exit(void)
{
    spi_unregister_driver(&broadcast_spi_driver);
}

#if 0
static int broadcast_Isdb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc = 0;

    int addr;

    print_log(NULL, "broadcast_Isdb_i2c_probe client:0x%X\n", (unsigned int)client);
    if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        print_log(NULL, "need I2C_FUNC_I2C\n");
        return -ENODEV;
    }
    /* taew00k.kang added for Device Tree Structure 2013-06-04 [start] */
    addr = client->addr; //Slave Addr
    pr_err("[1seg] i2c Slaveaddr [%x] \n", addr);

    IsdbCtrlInfo.pclient = client;
    //i2c_set_clientdata(client, (void*)&IsdbCtrlInfo.pclient);

    tunerbb_drv_hw_setting();
#ifndef _NOT_USE_WAKE_LOCK_
    wake_lock_init(&IsdbCtrlInfo.wake_lock, WAKE_LOCK_SUSPEND,
                    dev_name(&client->dev));
#endif

    return rc;
}

static int broadcast_Isdb_i2c_remove(struct i2c_client* client)
{
    int rc = 0;

    print_log(NULL, "[%s]\n", __func__);
#ifndef _NOT_USE_WAKE_LOCK_
    wake_lock_destroy(&IsdbCtrlInfo.wake_lock);
#endif
    memset((unsigned char*)&IsdbCtrlInfo, 0x0, sizeof(struct broadcast_fc8300_ctrl_data));
    //TcpalDeleteSemaphore(&fc8300DrvSem);
    return rc;
}

static int broadcast_Isdb_i2c_suspend(struct i2c_client* client, pm_message_t mesg)
{
    int rc = 0;
    print_log(NULL, "[%s]\n", __func__);
    return rc;
}

static int broadcast_Isdb_i2c_resume(struct i2c_client* client)
{
    int rc = 0;
    print_log(NULL, "[%s]\n", __func__);
    return rc;
}

static const struct i2c_device_id isdbt_fc8300_id[] = {
/* taew00k.kang added for Device Tree Structure 2013-06-04 [start] */
    {"tcc3535_i2c",    0},
/* taew00k.kang added for Device Tree Structure 2013-06-04 [end] */
    {},
};

MODULE_DEVICE_TABLE(i2c, isdbt_fc8300_id);


/* taew00k.kang added for Device Tree Structure 2013-06-04 [start] */
static struct of_device_id tcc3535_i2c_table[] = {
{ .compatible = "telechips,tcc3535-i2c",}, //Compatible node must match dts
{ },
};
/* taew00k.kang added for Device Tree Structure 2013-06-04 [end] */

static struct i2c_driver broadcast_Isdb_driver = {
    .driver = {
        .name = "tcc3535_i2c",
        .owner = THIS_MODULE,
        .of_match_table = tcc3535_i2c_table,
    },
    .probe = broadcast_Isdb_i2c_probe,
    .remove    = __devexit_p(broadcast_Isdb_i2c_remove),
    .id_table = isdbt_fc8300_id,
    .suspend = broadcast_Isdb_i2c_suspend,
    .resume  = broadcast_Isdb_i2c_resume,
};

int __init broadcast_dmb_drv_init(void)
{
    int rc;
    print_log(NULL, "[%s]\n", __func__);
    rc = broadcast_dmb_drv_start();
    if (rc)
    {
        print_log(NULL, "failed to load\n");
        return rc;
    }
    print_log(NULL, "[%s add i2c driver]\n", __func__);
    rc = i2c_add_driver(&broadcast_Isdb_driver);
    print_log(NULL, "broadcast_add_driver rc = (%d)\n", rc);
    return rc;
}

static void __exit broadcast_dmb_drv_exit(void)
{
    i2c_del_driver(&broadcast_Isdb_driver);
}
#endif
module_init(broadcast_dmb_fc8300_drv_init);
module_exit(broadcast_dmb_fc8300_drv_exit);
MODULE_DESCRIPTION("broadcast_dmb_drv_init");
MODULE_LICENSE("FCI");
