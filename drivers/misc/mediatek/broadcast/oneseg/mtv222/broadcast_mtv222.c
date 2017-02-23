#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/uaccess.h>

#include "../broadcast_dmb_typedef.h"
#include "../broadcast_dmb_drv_ifdef.h"

#include "raontv.h"
#include "raontv_internal.h"
#include "broadcast_mtv222.h"

#include <mach/board_lge.h>

#if defined(CONFIG_ARCH_MT6582)  || defined(CONFIG_ARCH_MT6572)  // MediaTek platform
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <cust_eint_md1.h>
#include <cust_gpio_usage.h>
#include <cust_eint.h>
#include <linux/rtpm_prio.h>
#include <mach/mt_pm_ldo.h>    // FC8150 CORE  POWER LDO
#include <mach/mt_spi.h>
#endif //defined(CONFIG_ARCH_MT6582) // MediaTek platform

#if defined(CONFIG_MACH_MSM8X10_W6DS_TIM_BR) || defined(CONFIG_MACH_MSM8X10_W6DS_GLOBAL_SCA)
extern u8 module_init_flag;
#endif

#if defined(TARGET_MT6582_Y50) || defined(TARGET_MT6582_Y70) || defined(TARGET_MT6582_P1S3G)
/*
EXPORT_SYMBOL(g_lge_dtv_flag) in kernel/init/main.c : Support For SBP
*/
extern int g_lge_dtv_flag;
#endif

#define DRIVER_NAME "isdbt"

/* TS Buffer Descriptor information. */
struct TSB_DESC {
	/* Flag of operation enabled or not. */
	volatile int op_enabled; /* 0: disabled, 1: enabled */

	/* TSP buffer index which updated when read operation. */
	volatile int read_idx;

	/* TSP buffer index which update when write operation. */
	volatile int write_idx;

	/* The pointer to the base address of TSP buffer segments. */
	unsigned long seg_base[ISDBT_NUM_TSP_CHNUKS];
};

/* TS Buffer control block. */
struct TSB_CB {
	struct mutex lock;

	/* Index of available tsp segment to be write. */
	int avail_seg_idx;

	/* Index of available tsp buffer to be write. */
	int avail_write_tspb_idx;

	/* Index of tsp segment to be enqueued. */
	int enqueue_seg_idx;

	/* Number of buffering TSPs per segment. */
	int num_tsp_per_seg;

	/* Number of allocated  total buffering tsps in the TSB. */
	int num_total_tsp;

	/* Number of allocated segments. */
	int num_total_seg;

	/* To debug */
	int max_enqueued_tsp_cnt;

	/* TSB descriptor informations. */
	struct TSB_DESC tsbd;
};

static struct TSB_CB *tsb_cb = NULL;
struct MTV222_CB *mtv222_cb = NULL;

static boolean mtv_probe_check_flag = FALSE;

static Device_drv device_mtv = {
	&broadcast_mtv_drv_if_power_on,
	&broadcast_mtv_drv_if_power_off,
	&broadcast_mtv_drv_if_open,
	&broadcast_mtv_drv_if_close,
	&broadcast_mtv_drv_if_set_channel,
	&broadcast_mtv_drv_if_resync,
	&broadcast_mtv_drv_if_detect_sync,
	&broadcast_mtv_drv_if_get_sig_info,
	&broadcast_mtv_drv_if_get_ch_info,
	&broadcast_mtv_drv_if_get_dmb_data,
	&broadcast_mtv_drv_if_reset_ch,
	&broadcast_mtv_drv_if_user_stop,
	&broadcast_mtv_drv_if_select_antenna,
	&broadcast_mtv_drv_if_read_control,
	&broadcast_mtv_drv_if_get_mode
};

/* F7 MSM8960 GPIOs */
#define MTV222_DMB_EN			100
#define MTV222_DMB_INT_N		82
#define MTV222_DMB_RESET_N		92

void configure_gpios(void)
{
	int rc = OK;

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6572)    // MediaTek platform

	//PWR Enable

	printk("[1seg][MTK] GPIO_ISDBT_PWR_EN Port request!!!\n");

	rc = mt_set_gpio_mode(GPIO_1SEG_EN, GPIO_1SEG_EN_M_GPIO);
	printk("[1seg][MTK] mt_set_gpio_mode(GPIO_1SEG_EN, GPIO_1SEG_EN_M_GPIO) : (%d)\n", rc);
	rc = mt_set_gpio_pull_enable(GPIO_1SEG_EN, GPIO_PULL_ENABLE);
	printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO_1SEG_EN, GPIO_PULL_ENABLE) : (%d)\n", rc);
	rc = mt_set_gpio_pull_select(GPIO_1SEG_EN,GPIO_PULL_DOWN);
	printk("[1seg][MTK] mt_set_gpio_pull_select(GPIO_1SEG_EN,GPIO_PULL_DOWN) : (%d)\n", rc);

	udelay(50);

	rc = mt_set_gpio_dir(GPIO_1SEG_EN, GPIO_DIR_OUT);
	printk("[1seg][MTK] mt_set_gpio_dir(GPIO_1SEG_EN, GPIO_DIR_OUT) : (%d)\n", rc);
	rc = mt_set_gpio_out(GPIO_1SEG_EN, GPIO_OUT_ZERO);
	printk("[1seg][MTK] mt_set_gpio_out(GPIO_1SEG_EN, GPIO_OUT_ZERO) : (%d)\n", rc);

	// Interrupt  - Setting
	printk("[1seg][MTK] ISDBT_IRQ_INT Port request!!!\n");

	rc = mt_set_gpio_mode(GPIO_1SEG_INT, GPIO_1SEG_INT_M_GPIO);
	printk("[1seg][MTK] mt_set_gpio_mode(GPIO_1SEG_INT, GPIO_1SEG_INT_M_GPIO) : (%d)\n", rc);

	rc = mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_ENABLE);
	printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_ENABLE)(%d)\n", rc);

	rc = mt_set_gpio_pull_select(GPIO_1SEG_INT,GPIO_PULL_DOWN);
	printk("[1seg][MTK] mt_set_gpio_pull_select(GPIO_1SEG_INT,GPIO_PULL_DOWN)(%d)\n", rc);

	rc = mt_set_gpio_dir(GPIO_1SEG_INT, GPIO_DIR_IN);
	printk("[1seg][MTK] mt_set_gpio_dir(GPIO_1SEG_INT, GPIO_DIR_IN) : (%d)\n", rc);


	//SPI MISO
	rc = mt_set_gpio_pull_enable(GPIO82|0x80000000, GPIO_PULL_ENABLE);
	printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO82|0x80000000, GPIO_PULL_ENABLE) : (%d)\n", rc);
	rc = mt_set_gpio_pull_select(GPIO82|0x80000000,GPIO_PULL_DOWN);
	printk("[1seg][MTK] mt_set_gpio_pull_select(GPIO82|0x80000000,GPIO_PULL_DOWN) : (%d)\n", rc);

	//SPI MOSI
	rc = mt_set_gpio_pull_enable(GPIO83|0x80000000, GPIO_PULL_ENABLE);
	printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO83|0x80000000, GPIO_PULL_ENABLE) : (%d)\n", rc);
	rc = mt_set_gpio_pull_select(GPIO83|0x80000000,GPIO_PULL_DOWN);
	printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO83|0x80000000, GPIO_PULL_ENABLE) : (%d)\n", rc);

	//SPI CLK
	rc = mt_set_gpio_pull_enable(GPIO81|0x80000000, GPIO_PULL_ENABLE);
	printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO81|0x80000000, GPIO_PULL_ENABLE) : (%d)\n", rc);
	rc = mt_set_gpio_pull_select(GPIO81|0x80000000,GPIO_PULL_DOWN);
	printk("[1seg][MTK] mt_set_gpio_pull_select(GPIO81|0x80000000,GPIO_PULL_DOWN) : (%d)\n", rc);

	//SPI CS
	rc = mt_set_gpio_pull_enable(GPIO80|0x80000000, GPIO_PULL_ENABLE);
	printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO80|0x80000000, GPIO_PULL_ENABLE) : (%d)\n", rc);
	rc = mt_set_gpio_pull_select(GPIO80|0x80000000,GPIO_PULL_DOWN);
	printk("[1seg][MTK] mt_set_gpio_pull_select(GPIO80|0x80000000,GPIO_PULL_DOWN) : (%d)\n", rc);

	rc = hwPowerOn(MT6323_POWER_LDO_VGP3, VOL_1800, "1V8_MTK_DTV");	// IO power On
	printk("[1seg][MTK] hwPowerOn MT6323_POWER_LDO_VGP3(%d)\n", rc);
	msleep(10);
#else //CONFIG_ARCH_MT6582

	rc = gpio_request(MTV222_DMB_EN, "DMB_EN");
	if (rc < 0) {
		printk("%s:Failed GPIO DMB_EN request!!!\n",__func__);
	}
	gpio_direction_output(MTV222_DMB_EN, 0);  /* output and low */

	rc = gpio_request(MTV222_DMB_RESET_N, "DMB_RESET_N");
	if (rc < 0) {
		printk("%s:Failed GPIO DMB_RESET_N request!!!\n",__func__);
	}
	gpio_direction_output(MTV222_DMB_RESET_N, 0);  /* output and low */

	rc = gpio_request(MTV222_DMB_INT_N, "DMB_INT_N");
	if (rc < 0) {
		printk("%s:Failed GPIO DMB_INT_N request!!!\n",__func__);
	}
	gpio_direction_input(MTV222_DMB_INT_N);

#endif
}

void mtv222_drv_hw_init(void)
{
#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6572)
        int ret;
        // MediaTek platform
        printk("[1seg][MTK] isdbt_hw_init\n");

        ret = mt_set_gpio_out(GPIO_1SEG_EN, GPIO_OUT_ONE);
        printk("[1seg][MTK] mt_set_gpio_out(GPIO_1SEG_EN, GPIO_OUT_ONE)(%d)\n", ret);
        msleep(10);

        //Interrupt
        ret = mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_DISABLE);
        printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_DISABLE)(%d)\n", ret);

        //SPI MISO
        ret = mt_set_gpio_pull_enable(GPIO82|0x80000000, GPIO_PULL_DISABLE);
        printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO82|0x80000000, GPIO_PULL_DISABLE)(%d)\n", ret);

        //SPI MOSI
        ret = mt_set_gpio_pull_enable(GPIO83|0x80000000, GPIO_PULL_DISABLE);
        printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO83|0x80000000, GPIO_PULL_DISABLE)(%d)\n", ret);

        //SPI CLK
        ret = mt_set_gpio_pull_enable(GPIO81|0x80000000, GPIO_PULL_DISABLE);
        printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO81|0x80000000, GPIO_PULL_DISABLE)(%d)\n", ret);

        //SPI CS
        ret = mt_set_gpio_pull_enable(GPIO80|0x80000000, GPIO_PULL_DISABLE);
        printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO80|0x80000000, GPIO_PULL_DISABLE)(%d)\n", ret);
        ret = mt_set_gpio_mode(GPIO80|0x80000000, GPIO_MODE_01);	 //GPIO_MATV_I2S_WS_PIN_M_SPI_CS   GPIO_MODE_01
        printk("[1seg][MTK] mt_set_gpio_mode(GPIO80|0x80000000, GPIO_MODE_01)(%d)\n", ret);
        ret = mt_set_gpio_dir(GPIO80|0x80000000, GPIO_DIR_IN);
        printk("[1seg][MTK] mt_set_gpio_dir(GPIO80|0x80000000, GPIO_DIR_IN)(%d)\n", ret);

#else //CONFIG_ARCH_MT6582
        gpio_set_value(GPIO_ISDBT_RST, 0);
        gpio_set_value(GPIO_ISDBT_PWR_EN, 1);
        msleep(3);
        gpio_set_value(GPIO_ISDBT_RST, 1);
        msleep(2);
#endif //CONFIG_ARCH_MT6582
        return;
}

void mtv222_drv_hw_deinit(void)
{
#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6572)   // MediaTek platform
       int ret = 0;
	mt_eint_mask(CUST_EINT_DTV_NUM);

	printk("[1seg][MTK] isdbt_hw_deinit\n");

	// pin setting
	// 1SEG EN
	ret = mt_set_gpio_pull_enable(GPIO_1SEG_EN, GPIO_PULL_ENABLE);
	printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO_1SEG_EN, GPIO_PULL_ENABLE)(%d)\n", ret);

	ret = mt_set_gpio_pull_select(GPIO_1SEG_EN,GPIO_PULL_DOWN);
	printk("[1seg][MTK] mt_set_gpio_pull_select(GPIO_1SEG_EN,GPIO_PULL_DOWN)(%d)\n", ret);

	ret = mt_set_gpio_dir(GPIO_1SEG_EN, GPIO_DIR_OUT);
	printk("[1seg][MTK] mt_set_gpio_dir(GPIO_1SEG_EN, GPIO_DIR_OUT)(%d)\n", ret);

	ret = mt_set_gpio_out(GPIO_1SEG_EN, GPIO_OUT_ZERO);
	printk("[1seg][MTK] mt_set_gpio_out(GPIO_1SEG_EN, GPIO_OUT_ZERO)(%d)\n", ret);

	// Interrupt
	ret = mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_ENABLE);
	printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_ENABLE)(%d)\n", ret);

	ret = mt_set_gpio_pull_select(GPIO_1SEG_INT,GPIO_PULL_DOWN);
	printk("[1seg][MTK] mt_set_gpio_pull_select(GPIO_1SEG_INT,GPIO_PULL_DOWN)(%d)\n", ret);

	//SPI MISO
	ret = mt_set_gpio_pull_enable(GPIO82|0x80000000, GPIO_PULL_ENABLE);
	printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO82|0x80000000, GPIO_PULL_ENABLE)(%d)\n", ret);

	ret = mt_set_gpio_pull_select(GPIO82|0x80000000,GPIO_PULL_DOWN);
	printk("[1seg][MTK] mt_set_gpio_pull_select(GPIO82|0x80000000,GPIO_PULL_DOWN)(%d)\n", ret);


	//SPI MOSI
	ret = mt_set_gpio_pull_enable(GPIO83|0x80000000, GPIO_PULL_ENABLE);
	printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO83|0x80000000, GPIO_PULL_ENABLE)(%d)\n", ret);

	ret = mt_set_gpio_pull_select(GPIO83|0x80000000,GPIO_PULL_DOWN);
	printk("[1seg][MTK] mt_set_gpio_pull_select(GPIO83|0x80000000,GPIO_PULL_DOWN)(%d)\n", ret);

	//SPI CLK
	ret = mt_set_gpio_pull_enable(GPIO81|0x80000000, GPIO_PULL_ENABLE);
	printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO81|0x80000000, GPIO_PULL_ENABLE)(%d)\n", ret);
	
	ret = mt_set_gpio_pull_select(GPIO81|0x80000000,GPIO_PULL_DOWN);
	printk("[1seg][MTK] mt_set_gpio_pull_select(GPIO81|0x80000000,GPIO_PULL_DOWN)(%d)\n", ret);

	//SPI CS
	ret = mt_set_gpio_mode(GPIO80|0x80000000, GPIO_MODE_00); //GPIO_MATV_I2S_WS_PIN_M_GPIO
	printk("[1seg][MTK] mt_set_gpio_mode(GPIO80|0x80000000, GPIO_MODE_00)(%d)\n", ret);

	ret = mt_set_gpio_pull_enable(GPIO80|0x80000000, GPIO_PULL_ENABLE);
	printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO80|0x80000000, GPIO_PULL_ENABLE)(%d)\n", ret);

	ret = mt_set_gpio_pull_select(GPIO80|0x80000000,GPIO_PULL_DOWN);
	printk("[1seg][MTK] mt_set_gpio_pull_select(GPIO80|0x80000000,GPIO_PULL_DOWN)(%d)\n", ret);

	ret = mt_set_gpio_dir(GPIO80|0x80000000, GPIO_DIR_OUT);
	printk("[1seg][MTK] mt_set_gpio_dir(GPIO80|0x80000000, GPIO_DIR_OUT)(%d)\n", ret);

	ret = mt_set_gpio_out(GPIO80|0x80000000, GPIO_OUT_ZERO);
	printk("[1seg][MTK] mt_set_gpio_out(GPIO80|0x80000000, GPIO_OUT_ZERO)(%d)\n", ret);

#else //CONFIG_ARCH_MT6582
	gpio_set_value(GPIO_ISDBT_PWR_EN, 0);
#endif //CONFIG_ARCH_MT6582
}

int mtv222_device_power_on(void)
{
#ifdef FEATURE_DMB_USE_XO
	int rc;
#endif

#ifdef FEATURE_DMB_USE_XO
	if (mtv222_cb->clk != NULL) {
		rc = clk_prepare_enable(mtv222_cb->clk);
		if (rc) {
			dev_err(&mtv222_cb->spi_ptr->dev, "could not enable clock\n");
			return rc;
		}
	}
#endif
	mtv222_drv_hw_init();

	mtv222_cb->power_state = true;

	return 0;
}

int mtv222_device_power_off(void)
{
#ifdef FEATURE_DMB_USE_XO
	if (mtv222_cb->clk != NULL)
		clk_disable_unprepare(mtv222_cb->clk);
#endif

       mtv222_drv_hw_deinit();

	mtv222_cb->power_state = false;

	return 0;
}

bool mtv222_device_power_state(void)
{
	return mtv222_cb->power_state;
}

void mtv222_disable_irq(void)
{
#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6572)   // MediaTek platform
    mt_eint_mask(CUST_EINT_DTV_NUM);
#else
	disable_irq(mtv222_cb->spi_ptr->irq);
#endif
}

void mtv222_enable_irq(void)
{
#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6572)   // MediaTek platform
    mt_eint_unmask(CUST_EINT_DTV_NUM);
#else
	enable_irq(mtv222_cb->spi_ptr->irq);
#endif
}

/* Reset tsp buffer informations */
static inline void __tsb_reset_info(void)
{
	tsb_cb->tsbd.read_idx = 0;
	tsb_cb->tsbd.write_idx = 0;

	tsb_cb->avail_seg_idx = 0;
	tsb_cb->avail_write_tspb_idx = 0;
	tsb_cb->enqueue_seg_idx = 0;

	tsb_cb->max_enqueued_tsp_cnt = 0;
}

static inline int __tsb_get_enqueued_count(void)
{
	int readi, writei;
	int num_tsp = 0;

	readi = tsb_cb->tsbd.read_idx;
	writei = tsb_cb->tsbd.write_idx;

	if (writei > readi)
		num_tsp = writei - readi;
	else if (writei < readi)
		num_tsp = tsb_cb->num_total_tsp - (readi - writei);
	else
		num_tsp = 0;

	/* Update the max enqueued tsp count */
	tsb_cb->max_enqueued_tsp_cnt
			= max(tsb_cb->max_enqueued_tsp_cnt, num_tsp);

	return num_tsp;
}

int mtv222_tsb_read(void *buf, unsigned int size)
{
	uint8 *seg_ptr, *tspb_ptr;
	int has_num_tsp, req_num_tsp, num_copy_tsp;
	int read_tspi, seg_idx, tsp_idx;
	int num_copied_tsp, copied_bytes;
	char __user *ts_buf = (char __user *)buf;
	int total_copied_bytes = 0;

	mutex_lock(&tsb_cb->lock);
	if (!tsb_cb->tsbd.op_enabled)
		goto release_read_mutex;

	has_num_tsp = __tsb_get_enqueued_count();
	mutex_unlock(&tsb_cb->lock);

	if ((has_num_tsp > 0) && (size >= TS_PACKET_SIZE)) {
		req_num_tsp = size / TS_PACKET_SIZE;
		num_copy_tsp = min(has_num_tsp, req_num_tsp);

		do {
			mutex_lock(&tsb_cb->lock);

			/* Check if the TSB was disabled by applicaton? */
			if (!tsb_cb->tsbd.op_enabled) {
				goto release_read_mutex;
			}

			/* Get the read index from shared variable. */
			read_tspi = tsb_cb->tsbd.read_idx;

			/* Get the pointer of buffer from a segment. */
			seg_idx = read_tspi / tsb_cb->num_tsp_per_seg;
			tsp_idx = read_tspi % tsb_cb->num_tsp_per_seg; /* for MIN() */

			seg_ptr = (uint8 *)tsb_cb->tsbd.seg_base[seg_idx];
			tspb_ptr = &seg_ptr[tsp_idx * TS_PACKET_SIZE];

			/* Copying of TSP must be within the same segment  */
			num_copied_tsp = min(tsb_cb->num_tsp_per_seg - tsp_idx,
								num_copy_tsp);

			copied_bytes = num_copied_tsp * TS_PACKET_SIZE;

			if (copy_to_user(ts_buf, tspb_ptr, copied_bytes)) {
				MTVERR("Copy error\n");
				goto release_read_mutex;
			}

			/* Get the next index by avoiding of modular operation. */
			read_tspi += num_copied_tsp;
			if (read_tspi >= tsb_cb->num_total_tsp)
				read_tspi -= tsb_cb->num_total_tsp;

#if 0
			MTVMSG("seg_idx(%u), tsp_idx(%u), num_copied_tsp(%u), next_tspi(%u)\n",
					seg_idx, tsp_idx, num_copied_tsp, read_tspi);
#endif

			/* Update the read index from local variable. */
			tsb_cb->tsbd.read_idx = read_tspi;

			mutex_unlock(&tsb_cb->lock);

			total_copied_bytes += copied_bytes;
			ts_buf += copied_bytes;
			num_copy_tsp -= num_copied_tsp;
		} while (num_copy_tsp);
	}

	return total_copied_bytes;

release_read_mutex:
	mutex_unlock(&tsb_cb->lock);

	return 0;
}

/* Use when the channel was stopped. */
int mtv222_tsb_disable(void)
{
	/* NOTE: Do not use __tsb_reset_info() and lock.
	Application maybe read data in continue. */
	tsb_cb->tsbd.op_enabled = 0;

	return 0;
}

/* Enbale TS Buffer
Call when the channel was started */
int mtv222_tsb_enable(void)
{
	mutex_lock(&tsb_cb->lock);

	__tsb_reset_info();

	tsb_cb->tsbd.op_enabled = 1;

	mutex_unlock(&tsb_cb->lock);

	return 0;
}

int mtv222_tsb_enqueue(unsigned char *ts_chunk)
{
	int ret = 0;
#ifdef DEBUG_TSP_BUF
	int readi, writei, num_euqueued_tsp, num_euqueued_seg;
#endif

	mutex_lock(&tsb_cb->lock);

	if (tsb_cb->tsbd.op_enabled) {
		/* Check if the specified tspb is the allocated tspb? */
		if (ts_chunk == (uint8 *)tsb_cb->tsbd.seg_base[tsb_cb->enqueue_seg_idx]) {
			/* Set the next index of write-tsp. */
			tsb_cb->tsbd.write_idx = tsb_cb->avail_write_tspb_idx;

			/* Set the next index of enqueuing segment. */
			tsb_cb->enqueue_seg_idx = tsb_cb->avail_seg_idx;

#ifdef DEBUG_TSP_BUF
			readi = tsb_cb->tsbd.read_idx;
			writei = tsb_cb->tsbd.write_idx;

			if (writei > readi)
				num_euqueued_tsp = writei - readi;
			else if (writei < readi)
				num_euqueued_tsp = tsb_cb->num_total_tsp - (readi - writei);
			else
				num_euqueued_tsp = 0;

			mtv222_cb->max_enqueued_tsp_cnt
				= MAX(mtv222_cb->max_enqueued_tsp_cnt, num_euqueued_tsp);

			num_euqueued_seg = num_euqueued_tsp / ISDBT_1SEG_SPI_INTR_PKT_CNT;
			mtv222_cb->max_enqueued_seg_cnt
				= MAX(mtv222_cb->max_enqueued_seg_cnt, num_euqueued_seg);
#endif
		} else {
			MTVERR("Invalid the enqueuing chunk address!\n");
			ret = -1;
		}
	} else {
		//MTVMSG("TSB memory was disabled\n");
		ret = -2;
	}

	mutex_unlock(&tsb_cb->lock);

	return ret;
}

/*
Get a TS chunk buffer to be write.
*/
unsigned char *mtv222_tsb_get(void)
{
	int readi;
	int nwi; /* Next index of tsp buffer to be write. */
	unsigned char *tspb = NULL;
	int num_tsp_per_seg = ISDBT_1SEG_SPI_INTR_PKT_CNT;
#ifdef DEBUG_TSP_BUF
	int num_used_segment; /* Should NOT zero. */
	int write_seg_idx, read_seg_idx;
#endif

	mutex_lock(&tsb_cb->lock);

	if (tsb_cb->tsbd.op_enabled) {
		readi = tsb_cb->tsbd.read_idx;

		/* Get the next avaliable index of segment to be write in the next time. */
		nwi = tsb_cb->avail_write_tspb_idx + num_tsp_per_seg;
		if (nwi >= tsb_cb->num_total_tsp)
			nwi = 0;

		if ((readi < nwi) || (readi >= (nwi + num_tsp_per_seg))) {
			tspb = (uint8 *)tsb_cb->tsbd.seg_base[tsb_cb->avail_seg_idx];

			/* Update the writting index of tsp buffer. */
			tsb_cb->avail_write_tspb_idx = nwi;

			/* Update the avaliable index of segment to be write in the next time. */
			if (++tsb_cb->avail_seg_idx >= tsb_cb->num_total_seg)
				tsb_cb->avail_seg_idx = 0;

#ifdef DEBUG_TSP_BUF
			write_seg_idx = tsb_cb->avail_seg_idx;
			read_seg_idx = readi / num_tsp_per_seg;

			if (write_seg_idx > read_seg_idx)
				num_used_segment = write_seg_idx - read_seg_idx;
			else if (write_seg_idx < read_seg_idx)
				num_used_segment
					= tsb_cb->num_total_seg - (read_seg_idx - write_seg_idx);
			else
				num_used_segment = 0;

			//MTVMSG("wseg_idx(%d), rseg_idx(%d), num_used_segment(%d)\n",
			//			write_seg_idx, read_seg_idx, num_used_segment);

			mtv222_cb->max_alloc_seg_cnt
				= MAX(mtv222_cb->max_alloc_seg_cnt, num_used_segment);
#endif
		} else
			MTVERR("Full tsp buffer.\n");
	} else {
		//MTVMSG("Tsp pool was disabled.\n");
	}

	mutex_unlock(&tsb_cb->lock);

	return tspb;
}

void mtv222_tsb_destory_pool(void)
{
	int i;

	/* NOTE: Do not use __tsb_reset_info() and lock.
	Application maybe read data in continue. */
	tsb_cb->tsbd.op_enabled = 0;

	mutex_lock(&tsb_cb->lock);

	if (tsb_cb->tsbd.seg_base[0]) {
		for (i = 0; i < tsb_cb->num_total_seg; i++) {
			if (tsb_cb->tsbd.seg_base[i]) {
				kfree((void *)tsb_cb->tsbd.seg_base[i]);
				tsb_cb->tsbd.seg_base[i] = (unsigned long)NULL;
			}
		}
	}

	mutex_unlock(&tsb_cb->lock);
}

/*
tsp_chunk_size: Interrupt size.
*/
int mtv222_tsb_create_pool(unsigned int num_tsp_chunk, unsigned int tsp_chunk_size)
{
	int ret;
	unsigned int i;

	if (tsp_chunk_size > (32 * PAGE_SIZE)) { /* for Linux kmalloc() */
		MTVERR("Too big size of tsp chunk\n");
		return -2;
	}

	mutex_lock(&tsb_cb->lock);

	if (tsb_cb->tsbd.seg_base[0] == (unsigned long)NULL) {
		for (i = 0; i < num_tsp_chunk; i++) {
			tsb_cb->tsbd.seg_base[i]
				= (unsigned long)kmalloc(tsp_chunk_size, GFP_KERNEL|GFP_DMA);
			if (tsb_cb->tsbd.seg_base[i] == (unsigned long)NULL) {
				MTVERR("SEG[%u] allocation error\n", i);
				ret = -ENOMEM;
				goto free_tspb;
			}
		}

		tsb_cb->num_total_seg = num_tsp_chunk;
		tsb_cb->num_tsp_per_seg = tsp_chunk_size / TS_PACKET_SIZE;
		tsb_cb->num_total_tsp = tsb_cb->num_tsp_per_seg * num_tsp_chunk;
	} else
		MTVMSG("TSB pool was already created\n");

	mutex_unlock(&tsb_cb->lock);

	return 0;

free_tspb:
	mutex_unlock(&tsb_cb->lock);

	mtv222_tsb_destory_pool();

	return ret;
}

void mtv222_tsb_exit(void)
{
	if (tsb_cb) {
		mtv222_tsb_destory_pool();

		kfree(tsb_cb);
		tsb_cb = NULL;
	}
}

int mtv222_tsb_init(void)
{
	int ret;

	if (tsb_cb == NULL) {
		tsb_cb = (struct TSB_CB *)kzalloc(sizeof(struct TSB_CB), GFP_KERNEL);
		if (tsb_cb == NULL) {
			MTVERR("TSB CB allocation error\n");
			return -ENOMEM;
		}

		mutex_init(&tsb_cb->lock);

		ret = mtv222_tsb_create_pool(ISDBT_NUM_TSP_CHNUKS,
									ISDBT_1SEG_SPI_INTR_SIZE);
		if (ret < 0) {
			kfree(tsb_cb);
			tsb_cb = NULL;
			return ret;
		}

	} else
		MTVERR("TSB CB was already allocated!\n");

	return 0;
}

unsigned char mtv222_spi_read(unsigned char page, unsigned char reg)
{
	int ret;
	static u8 out_buf[4] __attribute__((aligned(8)));
	static u8 in_buf[4] __attribute__((aligned(8)));
	struct spi_message msg;
	struct spi_transfer msg_xfer = {
		.tx_buf = out_buf,
		.rx_buf = in_buf,
		.len = 4,
		.cs_change = 0,
		.delay_usecs = 0
	};

	out_buf[0] = 0x90 | page;
	out_buf[1] = reg;
	out_buf[2] = 1; /* Read size */

	spi_message_init(&msg);
	spi_message_add_tail(&msg_xfer, &msg);

	ret = spi_sync(mtv222_cb->spi_ptr, &msg);
	if (ret) {
		MTVERR("error: %d\n", ret);
		return 0xFF;
	}

	return in_buf[MTV222_SPI_CMD_SIZE];
}

void mtv222_spi_read_burst(unsigned char page, unsigned char reg,
						unsigned char *buf, int size)
{
	int ret;
	static u8 out_buf[MTV222_SPI_CMD_SIZE+1] __attribute__((aligned(8)));
	struct spi_message msg;
	struct spi_transfer xfer0 = {
		.tx_buf = out_buf,
		.rx_buf = buf,
		.len = MTV222_SPI_CMD_SIZE,
		.cs_change = 0,
		.delay_usecs = 0,
	};

	struct spi_transfer xfer1 = {
		.tx_buf = buf,
		.rx_buf = buf,
		.len = size,
		.cs_change = 0,
		.delay_usecs = 0,
	};

	out_buf[0] = 0xA0; /* Memory read */
	out_buf[1] = 0x00;
	out_buf[2] = 188; /* Fix */

	spi_message_init(&msg);
	spi_message_add_tail(&xfer0, &msg);
	spi_message_add_tail(&xfer1, &msg);

	ret = spi_sync(mtv222_cb->spi_ptr, &msg);
	if (ret)
		MTVERR("error: %d\n", ret);
}

void mtv222_spi_write(unsigned char page, unsigned char reg, unsigned char val)
{
	int ret;
	static u8 out_buf[4] __attribute__((aligned(8)));
	static u8 in_buf[4] __attribute__((aligned(8)));
	struct spi_message msg;
	struct spi_transfer msg_xfer = {
		.tx_buf = out_buf,
		.rx_buf = in_buf,
		.len = 4,
		.cs_change = 0,
		.delay_usecs = 0
	};

	out_buf[0] = 0x80 | page;
	out_buf[1] = reg;
	out_buf[2] = 1; /* size */
	out_buf[3] = val;

	spi_message_init(&msg);
	spi_message_add_tail(&msg_xfer, &msg);

	ret = spi_sync(mtv222_cb->spi_ptr, &msg);
	if (ret)
		MTVERR("error: %d\n", ret);
}


static inline void mtv222_verify_tsp(unsigned char *ts_buf, int size)
{
	do {
#if 0 // to debug
		MTVMSG("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", ts_buf[0], ts_buf[1], ts_buf[2], ts_buf[3], ts_buf[4]);

#endif

		if ((ts_buf[0] != 0x47) || (ts_buf[1] & 0x80))
			mtv222_cb->err_tsp_cnt++;

		ts_buf += 188;
		size -= 188;
	} while (size);
}

void mtv222_init_tsp_statistics(void)
{
	mtv222_cb->total_tsp_cnt = 0;
	mtv222_cb->err_tsp_cnt = 0;
}

static void mtv222_spi_isr_handler(struct MTV222_CB *mtv222_cb_ptr)
{
	U8 istatus;
	unsigned char *ts_buf;
	int err_code;

	RTV_GUARD_LOCK;

	rtv_UpdateAdj();

	RTV_REG_MAP_SEL(SPI_CTRL_PAGE);

	istatus = RTV_REG_GET(0x10);
	//MTVMSG("$$$$$$$$ istatus(0x%02X)\n", istatus);

#if 0 // interrupt test
	RTV_REG_SET(0x2A, 1);
	RTV_REG_SET(0x2A, 0);
	RTV_GUARD_FREE;
	return;
#endif

	if (istatus & (U8)(~SPI_INTR_BITS)) {
		RTV_REG_SET(0x2A, 1);
		RTV_REG_SET(0x2A, 0);
		RTV_GUARD_FREE;
		MTVMSG("Interface error (0x%02X)\n", istatus);
		return;
	}

	if (istatus & SPI_UNDERFLOW_INTR) {
		RTV_REG_SET(0x2A, 1);
		RTV_REG_SET(0x2A, 0);
		RTV_GUARD_FREE;
		MTVMSG("UDF: 0x%02X\n", istatus);
		return;
	}

	if (istatus & (SPI_THRESHOLD_INTR|SPI_OVERFLOW_INTR)) {
		mtv222_cb->total_tsp_cnt += ISDBT_1SEG_SPI_INTR_PKT_CNT;

		ts_buf = mtv222_tsb_get();
		if (ts_buf) {
			RTV_REG_MAP_SEL(SPI_MEM_PAGE);
			RTV_REG_BURST_GET(0x10, ts_buf, ISDBT_1SEG_SPI_INTR_SIZE);

#ifdef _MTV_KERNEL_FILE_DUMP_ENABLE
			mtv_ts_dump_kfile_write(ts_buf, ISDBT_1SEG_SPI_INTR_SIZE);
#endif

			err_code = mtv222_tsb_enqueue(ts_buf);
			if (err_code == 0)
				mtv222_verify_tsp(ts_buf, ISDBT_1SEG_SPI_INTR_SIZE);
			else
				mtv222_cb->err_tsp_cnt += ISDBT_1SEG_SPI_INTR_PKT_CNT;

			if (istatus & SPI_OVERFLOW_INTR) /* To debug */
				MTVMSG("OVF: 0x%02X\n", istatus);
		} else {
			RTV_REG_MAP_SEL(SPI_CTRL_PAGE);
			RTV_REG_SET(0x2A, 1); /* SRAM init */
			RTV_REG_SET(0x2A, 0);

		#ifdef DEBUG_TSP_BUF
			mtv222_cb->alloc_tspb_err_cnt++;
		#endif

			mtv222_cb->err_tsp_cnt += ISDBT_1SEG_SPI_INTR_PKT_CNT;
		}
	}
	else
		MTVERR("No data interrupt (0x%02X)\n", istatus);

	RTV_GUARD_FREE;
}

#ifdef FEATURE_DMB_USE_WORKQUEUE
static void broacast_dmb_spi_work(struct work_struct *dmb_work)
{
	struct MTV222_CB *mtv222_cb_ptr
		= container_of(dmb_work, struct MTV222_CB, spi_work);

	if (mtv222_cb_ptr->tsout_enabled)
		mtv222_spi_isr_handler(mtv222_cb_ptr);
	else
		MTVMSG("Channel was not started\n");
}
#endif

#ifndef FEATURE_DMB_USE_WORKQUEUE
#define SCHED_FIFO_USE

static int mtv222_isr_thread(void *data)
{
#ifdef SCHED_FIFO_USE
	struct sched_param param = { .sched_priority = RTPM_PRIO_DTV };
	sched_setscheduler(current, SCHED_FIFO, &param);
#else
	set_user_nice(current, -20);
#endif

	MTVMSG("Start...\n");

	while (!kthread_should_stop()) {
		wait_event_interruptible(mtv222_cb->isr_wq,
			kthread_should_stop() || (atomic_read(&mtv222_cb->isr_cnt) > 0));

		atomic_dec(&mtv222_cb->isr_cnt);

		if(atomic_read(&mtv222_cb->isr_cnt) < 0)
		{
			atomic_set(&mtv222_cb->isr_cnt, 0);
		}

		if (kthread_should_stop())
			break;

		mtv222_spi_isr_handler(mtv222_cb);
	}

	MTVMSG("Exit.\n");

	return 0;
}

static int mtv222_create_isr_thread(void)
{
	int ret = 0;

	if (mtv222_cb->isr_thread_cb == NULL) {
		atomic_set(&mtv222_cb->isr_cnt, 0); /* Reset */
		init_waitqueue_head(&mtv222_cb->isr_wq);

		mtv222_cb->isr_thread_cb
			= kthread_run(mtv222_isr_thread, NULL, "isdbt_spi_isr");
		if (IS_ERR(mtv222_cb->isr_thread_cb)) {
			WARN(1, KERN_ERR "Create ISR thread error\n");
			ret = PTR_ERR(mtv222_cb->isr_thread_cb);
			mtv222_cb->isr_thread_cb = NULL;
		}
	}

	return ret;
}

static void mtv222_destory_isr_thread(void)
{
	if (mtv222_cb->isr_thread_cb) {
		kthread_stop(mtv222_cb->isr_thread_cb);
		mtv222_cb->isr_thread_cb = NULL;
	}
}
#endif /* #ifndef FEATURE_DMB_USE_WORKQUEUE */

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6572)    // MediaTek platform
void broadcast_spi_irq(void)
#else
static irqreturn_t broadcast_spi_irq(int irq, void *handle)
#endif
{
    if (mtv222_cb->tsout_enabled) {
#ifdef FEATURE_DMB_USE_WORKQUEUE
        queue_work(mtv222_cb->spi_wq, &mtv222_cb->spi_work);
#else
        if (mtv222_cb->isr_thread_cb) {
            if (mtv222_cb->tsout_enabled) {
                atomic_inc(&mtv222_cb->isr_cnt);
                wake_up_interruptible(&mtv222_cb->isr_wq);
                }
            }
#endif
        } else
            MTVMSG("Channel was not started\n");

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6572)    // MediaTek platform
    mt_eint_unmask(CUST_EINT_DTV_NUM);
    return ;
#else
    return IRQ_HANDLED;
#endif
}

static int broadcast_dmb_mtv_check_chip_id(void)
{
    u8 chip_id;
    int rc = ERROR;

    rc = mtv222_device_power_on();

    // To read chip id.
    chip_id = rtvISDBT_GetDeviceID();
    if (chip_id == RAONTV_CHIP_ID)
    {
	// OK
        MTVMSG("[1seg] MTV222 Chip ID(0x%x) Read OK \n", chip_id);
        rc = OK;
    }
    else
    {
        MTVERR("Invalid chip ID(%02X)\n", chip_id);
        rc = ERROR;
    }

    mtv222_device_power_off();

    return rc;
}

/*
static int broadcast_spi_suspend(struct spi_device *spi, pm_message_t mesg)
{
	MTVMSG("Suspend\n");
	return 0;
}

static int broadcast_spi_resume(struct spi_device *spi)
{
	MTVMSG("Resume\n");
	return 0;
}
*/

static int broadcast_spi_remove(struct spi_device *spi)
{
#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6572)   // MediaTek platform
       int ret;
#endif
	MTVMSG("broadcast_spi_remove \n");

#ifdef FEATURE_DMB_USE_WORKQUEUE
	if (mtv222_cb->spi_wq) {
		flush_workqueue(mtv222_cb->spi_wq);
		destroy_workqueue(mtv222_cb->spi_wq);
	}

#else
	mtv222_destory_isr_thread();
#endif

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6572)   // MediaTek platform
    // gpio
    ret = mt_set_gpio_mode(GPIO_1SEG_INT, GPIO_1SEG_INT_M_GPIO);
    printk("[1seg][MTK][isdbt_exit] GPIO_1SEG_INT mt_set_gpio_mode = %d!!!\n", ret);
    ret = mt_set_gpio_dir(GPIO_1SEG_INT, GPIO_DIR_IN);
    printk("[1seg][MTK][isdbt_exit] GPIO_1SEG_INT mt_set_gpio_dir = %d!!!\n", ret);
#else //defined(CONFIG_ARCH_MT6582) // MediaTek platform
	free_irq(spi->irq, mtv222_cb);
#endif

	wake_lock_destroy(&mtv222_cb->wake_lock);

	mtv222_tsb_exit();

	if (mtv222_cb) {
		kfree(mtv222_cb);
		mtv222_cb = NULL;
	}

	return 0;
}

static int broadcast_spi_probe(struct spi_device *spi)
{
	int rc;
       int ret;

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6572)   // MediaTek platform
       struct mt_chip_conf *mtk_spi_config = NULL;

    mtk_spi_config = ( struct mt_chip_conf * ) spi->controller_data;
    if(mtk_spi_config)
    {
        mtk_spi_config->setuptime = 5;
        mtk_spi_config->holdtime = 5;
        mtk_spi_config->high_time = 10;
        mtk_spi_config->low_time = 10;
        mtk_spi_config->cs_idletime = 6;
        mtk_spi_config->ulthgh_thrsh = 0;
        mtk_spi_config->cpol = 0;
        mtk_spi_config->cpha = 0;
        mtk_spi_config->rx_mlsb = 1;
        mtk_spi_config->tx_mlsb = 1;
        mtk_spi_config->tx_endian = 0;
        mtk_spi_config->rx_endian = 0;
        mtk_spi_config->com_mod = DMA_TRANSFER,
        mtk_spi_config->pause = 0;
        mtk_spi_config->finish_intr = 1;
        mtk_spi_config->deassert = 0;
        mtk_spi_config->ulthigh = 0;
        mtk_spi_config->tckdly = 0;
    }
#endif

	spi->mode 			= SPI_MODE_0;
	spi->bits_per_word	= 8;
	//spi->max_speed_hz 	= (10800*1000);

	MTVMSG("[1seg] broadcast_spi_probe Entered...\n");

	rc = spi_setup(spi);
	if (rc) {
		MTVERR("Spi setup error(%d)\n", rc);
		return rc;
	}

	mtv222_cb = kzalloc(sizeof(struct MTV222_CB), GFP_KERNEL);
	if (!mtv222_cb) {
		MTVERR("mtv222_cb allocation error\n");
		return -ENOMEM;
	}

	rc = mtv222_tsb_init();
	if (rc != 0)
		goto free_mtv_cb;

	mtv222_cb->spi_ptr = spi;

	mutex_init(&mtv222_cb->ioctl_lock);
	atomic_set(&mtv222_cb->open_flag, 0);
	atomic_set(&mtv222_cb->read_flag, 0);
	wake_lock_init(&mtv222_cb->wake_lock, WAKE_LOCK_SUSPEND, dev_name(&spi->dev));

	configure_gpios();

   if(broadcast_dmb_mtv_check_chip_id() != OK) {
        rc = ERROR;
   }

#if defined(FEATURE_DMB_USE_WORKQUEUE)
	INIT_WORK(&mtv222_cb->spi_work, broacast_dmb_spi_work);

	mtv222_cb->spi_wq = create_singlethread_workqueue("dmb_spi_wq");
	if (mtv222_cb->spi_wq == NULL){
		MTVERR("Failed to setup dmb spi workqueue \n");
		rc = -ENOMEM;
		goto free_mtv_cb;
	}
#else
	rc = mtv222_create_isr_thread();
	if (rc != 0)
		goto free_mtv_cb;
#endif

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6572)   // MediaTek platform
	// configuration for detect

	ret = (int)mt_eint_set_sens(CUST_EINT_DTV_NUM, CUST_EINT_EDGE_SENSITIVE);
	printk("[1seg][MTK] GPIO_1SEG_INT mt_eint_set_sens = %d!!!\n", ret);

	//gpio
	ret = mt_set_gpio_mode(GPIO_1SEG_INT, GPIO_1SEG_INT_M_EINT);
	printk("[1seg][MTK] GPIO_1SEG_INT mt_set_gpio_mode = %d!!!\n", ret);
	ret = mt_set_gpio_dir(GPIO_1SEG_INT, GPIO_DIR_IN);
	printk("[1seg][MTK] GPIO_1SEG_INT mt_set_gpio_dir = %d!!!\n", ret);

	ret = mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_DISABLE);
	printk("[1seg][MTK] mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_DISABLE)(%d)\n", ret);

	// irq handler register
	mt_eint_registration(CUST_EINT_DTV_NUM, EINTF_TRIGGER_FALLING/*CUST_EINT_DTV_POLARITY*/, broadcast_spi_irq, 0);
	printk("[1seg][MTK] GPIO_1SEG_INT mt_eint_registration!!!\n");

        mt_eint_mask(CUST_EINT_DTV_NUM);

	ret = mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_ENABLE);
	printk("[1seg][MTK] GPIO_1SEG_INT mt_set_gpio_pull_enable = %d!!!\n", ret);

        //SPI CS
        mt_set_gpio_mode(GPIO80|0x80000000, GPIO_MODE_00);  //GPIO_MATV_I2S_WS_PIN_M_GPIO  GPIO_MODE_00
        mt_set_gpio_pull_enable(GPIO80|0x80000000, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(GPIO80|0x80000000, GPIO_PULL_DOWN);
        mt_set_gpio_dir(GPIO80|0x80000000, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO80|0x80000000, GPIO_OUT_ZERO);

#else //defined(CONFIG_ARCH_MT6582) // MediaTek platform
	rc = request_irq(spi->irq, broadcast_spi_irq,
					IRQF_DISABLED | IRQF_TRIGGER_FALLING,
					spi->dev.driver->name, mtv222_cb);
#endif

	mtv_probe_check_flag = TRUE;

	ret = broadcast_dmb_drv_start(&device_mtv);
		if (ret) {
			MTVERR("Failed to load (%d)\n", ret);
			rc = ERROR;
		}

	if (rc < 0)
		goto free_mtv_cb;

       mtv222_disable_irq(); /* Must disabled */

	MTVMSG("[1seg] broadcast_spi_probe End.\n");

	return 0;

free_mtv_cb:
	broadcast_spi_remove(mtv222_cb->spi_ptr);

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
	.remove= broadcast_spi_remove,
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
*/
/*
static struct spi_driver broadcast_spi_driver = {
	.driver = {
		//.name		= DRIVER_NAME,
		.name = "dtv_spi",
		.of_match_table = broadcast_spi_table,
		.bus	= &spi_bus_type,
		.owner = THIS_MODULE,
	},

	.probe = broadcast_spi_probe,
	.suspend = broadcast_spi_suspend,
	.resume	= broadcast_spi_resume,
	.remove	= broadcast_spi_remove,
};
*/
static int __init broadcast_mtv_dmb_drv_init(void)
{
    int rc = ERROR;

    printk("%s is called\n", __func__);

#if defined(TARGET_MT6582_Y50) || defined(TARGET_MT6582_Y70) || defined(TARGET_MT6582_P1S3G)
    if(!g_lge_dtv_flag)
    {
        printk("[MTV222]This Model Does not Support DTV\n");
        return ERROR;
    }
#endif

    if(broadcast_dmb_drv_check_module_init() != OK) {
        rc = ERROR;
        return rc;
    }

    rc = spi_register_driver(&broadcast_spi_driver);
	
	//rc = broadcast_dmb_drv_start(&device_mtv);
	
    if (rc < 0)
    {
        printk("SPI driver register failed\n");
    }

return rc;

}

static void __exit broadcast_mtv_dmb_drv_exit(void)
{
	spi_unregister_driver(&broadcast_spi_driver);
}

module_init(broadcast_mtv_dmb_drv_init);
module_exit(broadcast_mtv_dmb_drv_exit);
MODULE_DESCRIPTION("MTV222 oneseg device driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("RAONTECH");
