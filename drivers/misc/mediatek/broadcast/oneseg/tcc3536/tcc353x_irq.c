
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include "Tcc353xDriver/PAL/tcpal_os.h"
#include "tcc353x_hal.h"
#include <linux/time.h>
#include <linux/ktime.h>

/*#define INT_TIME_CHECK*/
I32U Tcc353xInterruptProcess(void);
void Tcc353xInterruptGetStream(I32U _fifoSize);

struct spi_device *TCC_GET_SPI_DRIVER(void);

typedef struct _TcbdIrqData_t
{
	TcpalSemaphore_t lock;
	int tcbd_irq;
	int isIrqEnable;
} TcbdIrqData_t;

static TcbdIrqData_t TcbdIrqData;
extern TcpalSemaphore_t Tcc353xDrvSem;

#if defined (CONFIG_ARCH_MT6582)
/*-----------------------------------------------------------------------------
MT6582 :
	In this case we use kthead instead of threaded irq.
	threaded irq is not supported 
-----------------------------------------------------------------------------*/
static wait_queue_head_t Tcc353x_isr_wait;
static I32S Isr_counter = 0;
static struct task_struct *Tcc353x_isr_kthread = NULL;
static int TcbdIrqThreadfn(void *hDevice);

void tcc353x_isr_kthread_start (void)
{
	if (!Tcc353x_isr_kthread)
	{
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] kthread run\n");
		Tcc353x_isr_kthread = kthread_run(TcbdIrqThreadfn, NULL, "isdbt_thread");
	}
}

void tcc353x_isr_kthread_stop (void)
{
	if(Tcc353x_isr_kthread)
	{
		kthread_stop(Tcc353x_isr_kthread);
		Tcc353x_isr_kthread = NULL;
	}
}

void tcc353x_irq_handler(void)
{
	Isr_counter ++;
	wake_up_interruptible(&Tcc353x_isr_wait);
	TchalIrqUnmask();
}

static int TcbdIrqThreadfn(void *hDevice)
{
	struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)hDevice;
	struct sched_param param = { .sched_priority = RTPM_PRIO_DTV }; 
	sched_setscheduler ( current, SCHED_FIFO, &param );
	set_user_nice(current, -20);

	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok] isdbt_kthread inited\n");

	init_waitqueue_head(&Tcc353x_isr_wait);

	while(1)
	{
		I32U fifoSize;
		int old_irq_enable = 0;
#ifdef INT_TIME_CHECK
		ktime_t st, et, delta_t;
		long delta;

		st = ktime_get();
#endif
		wait_event_interruptible(Tcc353x_isr_wait, Isr_counter||kthread_should_stop());
		old_irq_enable = TcbdIrqData.isIrqEnable;

		if(Isr_counter>0)
			Isr_counter --;

		TcpalSemaphoreLock(&Tcc353xDrvSem);
		if (TcbdIrqData.isIrqEnable == 0) {
			TcpalPrintErr((I08S *)"[dtv][tcc3536][error] TcbdIrqThreadfn_irqDisabled. old_irq_enable=%d\n",
				old_irq_enable);
			TcpalSemaphoreUnLock(&Tcc353xDrvSem);
				continue;
		}

		fifoSize = Tcc353xInterruptProcess();
		if(fifoSize)
			Tcc353xInterruptGetStream(fifoSize);
		TcpalSemaphoreUnLock(&Tcc353xDrvSem);

#ifdef INT_TIME_CHECK
		et = ktime_get();
		delta_t = ktime_sub(et, st);
		delta = (ktime_to_ns(delta_t) >> 10);
		if(delta>(50*1000))
			TcpalPrintStatus((I08S *)"[dtv][tcc3536][debug_info] delta %ld [%ld]\n", delta, fifoSize);
#endif

		if (kthread_should_stop())
			break;
	}
	TcpalPrintStatus((I08S *) "[dtv][tcc3536][debug_info] isdbt_kthread exit\n");

	return 0;
}

#else
/*-----------------------------------------------------------------------------
	using threaded irq case
-----------------------------------------------------------------------------*/
static irqreturn_t TcbdIrqThreadfn(int _irq, void* _param)
{
	I32U fifoSize;
	int old_irq_enable = 0;
#ifdef INT_TIME_CHECK
	ktime_t st, et, delta_t;
	long delta;

	st = ktime_get();
#endif

	old_irq_enable = TcbdIrqData.isIrqEnable;

	TcpalSemaphoreLock(&Tcc353xDrvSem);
	if (TcbdIrqData.isIrqEnable == 0) {
		TcpalPrintErr((I08S *)"[dtv][tcc3536][error] TcbdIrqThreadfn_irqDisabled. old_irq_enable=%d\n",
			old_irq_enable);
		TcpalSemaphoreUnLock(&Tcc353xDrvSem);
			return IRQ_HANDLED;
	}

	fifoSize = Tcc353xInterruptProcess();
	if(fifoSize)
		Tcc353xInterruptGetStream(fifoSize);

	TcpalSemaphoreUnLock(&Tcc353xDrvSem);
	
#ifdef INT_TIME_CHECK
	et = ktime_get();
	delta_t = ktime_sub(et, st);
	delta = (ktime_to_ns(delta_t) >> 10);
	if(delta>(50*1000))
		TcpalPrintStatus((I08S *)"[dtv][tcc3536][debug_info] delta %ld [%ld]\n", delta, fifoSize);
#endif
	return IRQ_HANDLED;	
}
#endif

int TcpalRegisterIrqHandler(void)
{
	struct spi_device *spi = TCC_GET_SPI_DRIVER();

	TcbdIrqData.isIrqEnable = 0;
	TcbdIrqData.tcbd_irq = spi->irq;
#if defined (CONFIG_ARCH_MT6582)
	TchalIrqSetup();
	return 0;
#else
	return request_threaded_irq(TcbdIrqData.tcbd_irq, NULL, TcbdIrqThreadfn,
				IRQF_DISABLED | IRQF_TRIGGER_FALLING,  "tc353x_stream", &TcbdIrqData);
#endif
}

int TcpalUnRegisterIrqHandler(void)
{
#if defined (CONFIG_ARCH_MT6582)
	return 0;
#else
	disable_irq(TcbdIrqData.tcbd_irq);
	free_irq(TcbdIrqData.tcbd_irq, NULL);
	return 0;
#endif
}

int TcpalIrqEnable(void)
{
	TcbdIrqData.isIrqEnable = 1;
#if defined (CONFIG_ARCH_MT6582)
	Isr_counter = 0;
	TchalIrqUnmask();
#else
	enable_irq(TcbdIrqData.tcbd_irq);
#endif
	return 0;
}

int TcpalIrqDisable(void)
{
#if defined (CONFIG_ARCH_MT6582)
	TchalIrqMask();
	Isr_counter = 0;
#else
	disable_irq_nosync(TcbdIrqData.tcbd_irq);
#endif
	TcbdIrqData.isIrqEnable = 0;
	return 0;
}

