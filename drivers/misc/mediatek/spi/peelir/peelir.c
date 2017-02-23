/*
* Copyright (C) 2014  Peel Technologies Inc
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <asm/delay.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/gpio.h>

#include <linux/spi/spi.h>
#include "peelir.h"

#include <asm/uaccess.h>


#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <cust_eint_md1.h>
#include <cust_gpio_usage.h>
#include <cust_eint.h>
#ifndef GPIO_IRRC_SPI_CS
#define GPIO_IRRC_SPI_CS (GPIO80 | 0x80000000)
#endif

#ifndef GPIO_IRRC_SPI_CLK
#define GPIO_IRRC_SPI_CLK (GPIO81 | 0x80000000)
#endif

#ifndef GPIO_IRRC_SPI_MISO
#define GPIO_IRRC_SPI_MISO (GPIO82 | 0x80000000)
#endif

#ifndef GPIO_IRRC_SPI_MOSI
#define GPIO_IRRC_SPI_MOSI (GPIO83 | 0x80000000)
#endif
#define peelir_MAJOR			152	/* assigned */
#define N_SPI_MINORS			100	/* ... up to 256 */
extern int spi_register_driver(struct spi_driver *sdrv);
static DECLARE_BITMAP(minors, N_SPI_MINORS);

#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)

#define TRANS_BUFF	40960
#define NUM_BUFF	14
#define MAX_BUFF	NUM_BUFF*TRANS_BUFF
#define LR_EN		73

u8 t_buff[MAX_BUFF], rx_snd_buff[MAX_BUFF];
static int peelir_major;
struct peelir_data {
	dev_t			devt;
	struct spi_device	*spi;
	struct list_head	device_entry;
	struct mutex		buf_lock;
	spinlock_t		spi_lock;
	unsigned		users;
	u8			*buffer;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 300 * 1024;  /* Default buffer size */
static int mode = 0, bpw = 8, spi_clk_freq = 960000 ;
static int prev_tx_status; /* Status of previous transaction */
static unsigned int field;
static void *p_kbuf;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "Max buffer size to hold SPI message");
static void peelir_complete(void *arg)
{
	complete(arg);
}


static void peel_led_power ( unsigned int on )
{
	static unsigned int led_power_on = 0xFF;

	if ( led_power_on != on )
	{
		if ( on )
		{

			if ( !hwPowerOn ( MT6323_POWER_LDO_VGP3, VOL_1800, "PEEL_LED" ) )
				{
					//APS_ERR ( "failed to power on ( MT65XX_POWER_LDO_VGP3 )\n" );
					goto EXIT_ERR;
				}
			//APS_LOG("turned on the power ( PEEL_VGP3 )");
		}
		else
		{
			if ( !hwPowerDown ( MT6323_POWER_LDO_VGP3, "PEEL_LED" ) )
				{
					//APS_ERR ( "failed to power down ( MT65XX_POWER_LDO_VGP3 )\n" );
					goto EXIT_ERR;
				}
			//APS_LOG("turned off the power ( PEEL_VGP3 )\n");
		}
		led_power_on = on;
	}
		return;

	EXIT_ERR:
		return;
}

static ssize_t
peelir_sync(struct peelir_data *peelir, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = peelir_complete;
	message->context = &done;

	spin_lock_irq(&peelir->spi_lock);
	if (peelir->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(peelir->spi, message);

	spin_unlock_irq(&peelir->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static int peelir_read_message(struct peelir_data *peelir,
		struct spi_ioc_transfer *u_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		total;
	u8			*buf;
	char *tx_data;
	int status = -EFAULT;
	int i;

	spi_message_init(&msg);
	k_xfers = kcalloc(1, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Accept Pattern from the user and generate the SPI message */

	tx_data = (char *) kmalloc (MAX_BUFF,GFP_KERNEL);

	memset(tx_data,0, MAX_BUFF); // Transmit buffer
	memset(t_buff,0xff,MAX_BUFF);	//Receive Buffer

	u_xfers->tx_buf = (unsigned long )tx_data;

	/* Transmit 40K buffer filled with zero for 3 second to keep the SPI clk active for receiving IR input */

	buf = peelir->buffer;
	total = 0;
	k_tmp = k_xfers;
	u_tmp = u_xfers;
	k_tmp->len = u_tmp->len;
	total += k_tmp->len;
	if (total > bufsiz) {
		status = -EMSGSIZE;
		goto done;
	}

	if (u_tmp->rx_buf) {
		k_tmp->rx_buf = buf;
		if (!access_ok(VERIFY_WRITE, (u8 __user *)
					(uintptr_t) u_tmp->rx_buf,
					u_tmp->len))
			printk("\nERROR::::: In rcv");
	}
	if (u_tmp->tx_buf) {
		buf = tx_data;
		k_tmp->tx_buf = buf;
	}
	buf += k_tmp->len;

	k_tmp->cs_change = 0;
	k_tmp->bits_per_word = u_tmp->bits_per_word;
//	k_tmp->bits_per_word = 8;
	k_tmp->speed_hz = u_tmp->speed_hz;

	spi_message_add_tail(k_tmp, &msg);

	/* Receiving IR input for 3 seconds */

	printk("\n Waiting for IR data.... ");
	printk("\n Press the Key\n");

	status = peelir_sync(peelir, &msg);
	buf = peelir->buffer;
	for (i=0; i < MAX_BUFF; i++)
		t_buff[i] = buf[i];

	u_tmp = u_xfers;
	u_tmp->len = MAX_BUFF;
	buf = peelir->buffer;

	/* copy any rx data to user space */
	if (u_tmp->rx_buf) {
		printk("\nCopying data to user space");
		if (__copy_to_user((u8 __user *)
			(uintptr_t) u_tmp->rx_buf, k_tmp->rx_buf, k_tmp->len))
		{
			printk("\nCopy to user space failed !!!");
			status = -EFAULT;
			goto done;
		}
	}

	status = total;

	kfree(tx_data);

	done:
	kfree(k_xfers);

	return 0;
}


static int peelir_write_message(struct peelir_data *peelir,
		struct spi_ioc_transfer *u_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		total;
	u8 *buf;
	int status = -EFAULT;


	spi_message_init(&msg);
	k_xfers = kcalloc(1, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;


	buf = peelir->buffer;
	total = 0;

	k_tmp = k_xfers;
	u_tmp = u_xfers;
	k_tmp->len = u_tmp->len;

	total += k_tmp->len;
	if (total > bufsiz) {
		status = -EMSGSIZE;
		goto done;
	}

	if (u_tmp->rx_buf) {
		k_tmp->rx_buf = buf;
		if (!access_ok(VERIFY_WRITE, (u8 __user *)
					(uintptr_t) u_tmp->rx_buf,
					u_tmp->len))
			goto done;
	}
	if (u_tmp->tx_buf) {
		k_tmp->tx_buf = buf;
		if (copy_from_user(buf, (const u8 __user *)
					(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
			goto done;
	}
	buf += k_tmp->len;
	k_tmp->cs_change = 0;
	k_tmp->bits_per_word = u_tmp->bits_per_word;
	//k_tmp->bits_per_word = 32;
	k_tmp->speed_hz = u_tmp->speed_hz;
	peelir->spi->bits_per_word = k_tmp->bits_per_word;
	printk("%s: Bits per word = %d\n", __func__, k_tmp->bits_per_word);
	peelir->spi->mode = 0;
	spi_message_add_tail(k_tmp, &msg);

	status = peelir_sync(peelir, &msg);
	if (status < 0)
		goto done;


	status = total;


done:
	kfree(k_xfers);
	return status;
}

static long
peelir_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			retval = 0;
	struct peelir_data	*peelir;
	struct spi_device	*spi;
	struct spi_ioc_transfer	*ioc;

	peelir = filp->private_data;

	spin_lock_irq(&peelir->spi_lock);

	spi = spi_dev_get(peelir->spi);

	spin_unlock_irq(&peelir->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	mutex_lock(&peelir->buf_lock);

	spi->mode = mode;
	spi->bits_per_word = bpw;
	spi->max_speed_hz = spi_clk_freq;

	retval = spi_setup(spi);
	if(retval < 0)
		dev_dbg(&spi->dev, "Error configuring SPI\n");
	printk("%s: arg= %lu \n",__func__,arg);
	switch (cmd) {
	/* read ioctls */
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;

	case SPI_IOC_WR_MSG:
		printk("%s:WRITE_INVOKED with arg= %lu \n",__func__,arg);
		gpio_set_value(LR_EN, 0);	// LR Enable low for Tx
		/* copy into scratch area */
		ioc = kmalloc( sizeof(struct spi_ioc_transfer), GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}
		if (__copy_from_user(ioc, (void __user *)arg, sizeof(struct spi_ioc_transfer))) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}

		printk("%s: peelir_write_message \n",__func__);
		retval = peelir_write_message(peelir, ioc);
		if(retval > 0)
			prev_tx_status = 1;
		else
			prev_tx_status = 0;

		kfree(ioc);
		gpio_set_value(LR_EN, 0);	// LR Enable default state

		break;

	case SPI_IOC_RD_MSG:
		gpio_set_value(LR_EN, 1);	// LR Enable high for Rx
		/* copy into scratch area */
                ioc = kmalloc( sizeof(struct spi_ioc_transfer), GFP_KERNEL);
                if (!ioc) {
                        retval = -ENOMEM;
                        break;
                }
                if (__copy_from_user(ioc, (void __user *)arg, sizeof(struct spi_ioc_transfer))) {
                        kfree(ioc);
                        retval = -EFAULT;
                        break;
                }

		retval = peelir_read_message(peelir, ioc);
		gpio_set_value(LR_EN, 0);	// LR Enable default state
	}



	mutex_unlock(&peelir->buf_lock);
	spi_dev_put(spi);

	return retval;
}

static int peelir_open(struct inode *inode, struct file *filp)
{
	struct peelir_data	*peelir;
	int			status = -ENXIO;

    peel_led_power(1); 
	mutex_lock(&device_list_lock);
	printk("%s: invoked\n",__func__);
	list_for_each_entry(peelir, &device_list, device_entry) {
		if (peelir->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (!peelir->buffer) {
			peelir->buffer = p_kbuf;
			if (!peelir->buffer) {
				printk(KERN_ERR"%s: no buffer\n",__func__);
				dev_dbg(&peelir->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			peelir->users++;
			filp->private_data = peelir;
			nonseekable_open(inode, filp);
		}
	} else
		pr_debug("peelir: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
	return status;
}

static int peelir_release(struct inode *inode, struct file *filp)
{
	struct peelir_data	*peelir;
	int			status = 0;

	mutex_lock(&device_list_lock);
	peelir = filp->private_data;
	filp->private_data = NULL;

	peelir->users--;

	mutex_unlock(&device_list_lock);
    peel_led_power(0); 

	return status;
}

/*
 * sysfs layer
 */

static ssize_t ir_tx_status(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
        return sprintf( buf, "%d\n", prev_tx_status );
}

static ssize_t field_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
        return sprintf( buf, "%x\n", field );
}

static ssize_t field_store(struct device *dev,
                                 struct device_attribute *attr, const char *buf, size_t count)
{
        sscanf(buf, "%x", &field);
        return count;
}

static DEVICE_ATTR(txstat, S_IRUGO, ir_tx_status, NULL);
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field_show, field_store);

static struct attribute *peel_attributes[] = {
        &dev_attr_txstat.attr,
        &dev_attr_field.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = peel_attributes,
};

static const struct file_operations peelir_fops = {
	.owner =	THIS_MODULE,
	.unlocked_ioctl = peelir_ioctl,
	.open =		peelir_open,
	.release =	peelir_release,
};

static struct class *peelir_class;

/*-------------------------------------------------------------------------*/

static int __init peelir_probe(struct spi_device *spi)
{
	struct peelir_data	*peelir;
	int			status;
	unsigned long		minor;

	/* Allocate driver data */
	peelir = kzalloc(sizeof(*peelir), GFP_KERNEL);
	printk("peelir_probe 11111111\n");
	if (!peelir)
		return -ENOMEM;
	printk("peelir_probe \n");
	/* Initialize the driver data */
	peelir->spi = spi;
	spin_lock_init(&peelir->spi_lock);
	mutex_init(&peelir->buf_lock);

	INIT_LIST_HEAD(&peelir->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
    mt_set_gpio_mode(GPIO_IRRC_SPI_MISO, GPIO_MODE_01);
    mt_set_gpio_pull_enable(GPIO_IRRC_SPI_MISO, GPIO_PULL_ENABLE);

    mt_set_gpio_mode(GPIO_IRRC_SPI_MOSI, GPIO_MODE_01);
    mt_set_gpio_pull_enable(GPIO_IRRC_SPI_MOSI, GPIO_PULL_ENABLE);

    mt_set_gpio_mode(GPIO_IRRC_SPI_CLK, GPIO_MODE_01);
    mt_set_gpio_pull_enable(GPIO_IRRC_SPI_CLK, GPIO_PULL_ENABLE);

    mt_set_gpio_mode(GPIO_IRRC_SPI_CS, GPIO_MODE_01);
    mt_set_gpio_pull_enable(GPIO_IRRC_SPI_CS, GPIO_PULL_ENABLE);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		peelir->devt = MKDEV(peelir_major, minor);
		dev = device_create(peelir_class, &spi->dev, peelir->devt,
				    peelir, "peel_ir");
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&peelir->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status == 0)
		spi_set_drvdata(spi, peelir);
	else
		kfree(peelir);

	if (gpio_is_valid(LR_EN)) {
                /* configure LR enable gpio */
                status= gpio_request(LR_EN, "lr_enable");
                if (status) {
                        printk("unable to request gpio [73]: %d\n",status);
                }
                status = gpio_direction_output(LR_EN, 0);
                if (status) {
                        printk("unable to set direction for gpio [73]: %d\n", status);
                }
                gpio_set_value(LR_EN, 0);
        }
       else
               printk("gpio 73 is not valid \n");

	/* sysfs entry */
	status = sysfs_create_group(&spi->dev.kobj, &attr_group);
	if( status )
		dev_dbg(&spi->dev, " Error creating sysfs entry " );

	return status;
}

static int __exit peelir_remove(struct spi_device *spi)
{
	struct peelir_data	*peelir = spi_get_drvdata(spi);

	sysfs_remove_group(&spi->dev.kobj, &attr_group);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&peelir->spi_lock);
	peelir->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&peelir->spi_lock);

	/* prevent opening a new instance of the device
	   during the removal of the device
	 */
	mutex_lock(&device_list_lock);
	list_del(&peelir->device_entry);
	device_destroy(peelir_class, peelir->devt);
	clear_bit(MINOR(peelir->devt), minors);
	if (peelir->users == 0)
		kfree(peelir);
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_driver peelir_spi_driver = {
	.driver = {
		.name =		"peelir",
		.owner =	THIS_MODULE,
	},
	.probe =	peelir_probe,
	.remove =	peelir_remove,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/

static int __init peelir_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	p_kbuf = kzalloc(bufsiz, GFP_KERNEL);
	if(IS_ERR_OR_NULL(p_kbuf))
		return -ENOMEM;
	peelir_major = register_chrdev(0, "peel_ir", &peelir_fops);
	if (peelir_major < 0)
		return peelir_major;

	peelir_class = class_create(THIS_MODULE, "peel_ir");
	if (IS_ERR(peelir_class)) {
		unregister_chrdev(peelir_major, peelir_spi_driver.driver.name);
		return PTR_ERR(peelir_class);
	}
	printk("peelir_init 22222\n");
	status = spi_register_driver(&peelir_spi_driver);
	if (status < 0) {
		class_destroy(peelir_class);
		unregister_chrdev(peelir_major, peelir_spi_driver.driver.name);
	}
	return status;
}
module_init(peelir_init);

static void __exit peelir_exit(void)
{
	spi_unregister_driver(&peelir_spi_driver);
	class_destroy(peelir_class);
	unregister_chrdev(peelir_major, peelir_spi_driver.driver.name);
}
module_exit(peelir_exit);

MODULE_DESCRIPTION("Peel IR SPI driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("PEEL_IR");

