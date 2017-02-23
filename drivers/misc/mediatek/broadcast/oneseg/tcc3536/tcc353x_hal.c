#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include "Tcc353xDriver/PAL/tcpal_os.h"
#include "tcc353x_hal.h"

#if defined(CONFIG_ARCH_MT6582)
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <cust_eint_md1.h>
#include <cust_gpio_usage.h>
#include <cust_eint.h>
#include <linux/rtpm_prio.h>  
#include <mach/mt_pm_ldo.h>
#endif

#if defined(CONFIG_ARCH_MT6582)
#else
#define ISDB_EN			76	/* GPIO 85 */
#define ISDB_RESET_N	75	/* GPIO 75 */
#define ISDB_INT_N		77	/* GPIO 77 */
#endif

void TchalInit(void)
{
#if defined(CONFIG_ARCH_MT6582)
	I32S ret = 0;
		
	/* DTV LDO EN */
	//ret = mt_set_gpio_mode(GPIO_DTV_LDO_EN, GPIO_DTV_LDO_EN_M_GPIO);
	//ret = mt_set_gpio_pull_enable(GPIO_DTV_LDO_EN, GPIO_PULL_ENABLE);
	//ret = mt_set_gpio_pull_select(GPIO_DTV_LDO_EN,GPIO_PULL_DOWN);

	/* PWR Enable */
	//ret = mt_set_gpio_mode(GPIO_1SEG_EN, GPIO_1SEG_EN_M_GPIO);
	//ret = mt_set_gpio_pull_enable(GPIO_1SEG_EN, GPIO_PULL_ENABLE);
	//ret = mt_set_gpio_pull_select(GPIO_1SEG_EN,GPIO_PULL_DOWN);

	udelay(50);

	//ret = mt_set_gpio_dir(GPIO_DTV_LDO_EN, GPIO_DIR_OUT);
	//ret = mt_set_gpio_out(GPIO_DTV_LDO_EN, GPIO_OUT_ZERO);

	//ret = mt_set_gpio_dir(GPIO_1SEG_EN, GPIO_DIR_OUT);
	//ret = mt_set_gpio_out(GPIO_1SEG_EN, GPIO_OUT_ZERO);

	/* interrupt */
	ret = mt_set_gpio_mode(GPIO_1SEG_INT, GPIO_1SEG_INT_M_GPIO);
	ret = mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_ENABLE);
	ret = mt_set_gpio_pull_select(GPIO_1SEG_INT,GPIO_PULL_DOWN);
	ret = mt_set_gpio_dir(GPIO_1SEG_INT, GPIO_DIR_IN);	

	/* Reset */
	udelay(50);
	ret = mt_set_gpio_mode(GPIO_1SEG_RESET_N,  GPIO_1SEG_RESET_N_M_GPIO);
	ret = mt_set_gpio_pull_enable(GPIO_1SEG_RESET_N, GPIO_PULL_ENABLE);
	ret = mt_set_gpio_pull_select(GPIO_1SEG_RESET_N,GPIO_PULL_DOWN);
	ret = mt_set_gpio_dir(GPIO_1SEG_RESET_N, GPIO_DIR_OUT);
	ret = mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ZERO);

	/* SPI Setting */
	/* SPI MISO */
	ret = mt_set_gpio_pull_enable(GPIO82|0x80000000, GPIO_PULL_ENABLE);
	ret = mt_set_gpio_pull_select(GPIO82|0x80000000,GPIO_PULL_DOWN);

	/* SPI MOSI */
	ret = mt_set_gpio_pull_enable(GPIO83|0x80000000, GPIO_PULL_ENABLE);
	ret = mt_set_gpio_pull_select(GPIO83|0x80000000,GPIO_PULL_DOWN);

	/* SPI CLK */
	ret = mt_set_gpio_pull_enable(GPIO81|0x80000000, GPIO_PULL_ENABLE);
	ret = mt_set_gpio_pull_select(GPIO81|0x80000000,GPIO_PULL_DOWN);

	/* SPI CS */
	ret = mt_set_gpio_pull_enable(GPIO80|0x80000000, GPIO_PULL_ENABLE);
	ret = mt_set_gpio_pull_select(GPIO80|0x80000000,GPIO_PULL_DOWN);
#else
	gpio_request(ISDB_RESET_N, "ISDB_RESET");
	gpio_request(ISDB_EN, "ISDB_EN");
	gpio_request(ISDB_INT_N, "ISDB_INT");
	gpio_direction_output(ISDB_RESET_N, false); 	/* output low */
	gpio_direction_output(ISDB_EN, false); 		/* output low */
	gpio_direction_input(ISDB_INT_N); 		/* input */
#endif
	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok][%s:%d]\n", __func__, __LINE__);
}

void TchalResetDevice(void)
{
#if defined(CONFIG_ARCH_MT6582)
	int ret;

	ret = mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ONE);
	TcpalmSleep(5);
	ret = mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ZERO);
	TcpalmSleep(5);
	ret = mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ONE);    // current leackage
	TcpalmSleep(5);
#else
	gpio_set_value(ISDB_RESET_N, 1);		/* high ISDB_RESET_N */
	TcpalmSleep(5);
	gpio_set_value(ISDB_RESET_N, 0);		/* low ISDB_RESET_N */
	TcpalmSleep(5);
	gpio_set_value(ISDB_RESET_N, 1);		/* high ISDB_RESET_N */
	TcpalmSleep(5);
#endif
	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok][%s:%d]\n", __func__, __LINE__);
}

void TchalPowerOnDevice(void)
{
	int ret;

#if defined(CONFIG_ARCH_MT6582) 
	/* interrupt */
	ret = mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_ENABLE);
	ret = mt_set_gpio_pull_select(GPIO_1SEG_INT,GPIO_PULL_UP);

	/* SPI MISO */
	ret = mt_set_gpio_pull_enable(GPIO82|0x80000000, GPIO_PULL_DISABLE);

	/* SPI MOSI */
	ret = mt_set_gpio_pull_enable(GPIO83|0x80000000, GPIO_PULL_DISABLE);

	/* SPI CLK */
	ret = mt_set_gpio_pull_enable(GPIO81|0x80000000, GPIO_PULL_DISABLE);

	/* SPI CS */
	ret = mt_set_gpio_pull_enable(GPIO80|0x80000000, GPIO_PULL_DISABLE);
	ret = mt_set_gpio_mode(GPIO80|0x80000000, GPIO_MODE_01);	 //GPIO_MATV_I2S_WS_PIN_M_SPI_CS   GPIO_MODE_01
	//ret = mt_set_gpio_dir(GPIO80|0x80000000, GPIO_DIR_IN); // fix 20140807

	/* 20140806 for stability pwr sequence -------------------------- */
	//ret = mt_set_gpio_out(GPIO_1SEG_EN, GPIO_OUT_ZERO);
	ret = mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ZERO);
	/*----------------------------------------------------------------*/

	/* PWR */
	ret = hwPowerOn(MT6323_POWER_LDO_VGP3, VOL_1800, "1V8_MTK_DTV");	// IO power On
	TcpalmSleep(2);
	//ret = mt_set_gpio_out(GPIO_DTV_LDO_EN, GPIO_OUT_ONE);
	//ret = mt_set_gpio_out(GPIO_1SEG_EN, GPIO_OUT_ONE);
	TcpalmSleep(10);

	TchalResetDevice();

#else
	gpio_direction_output(ISDB_EN, false); 		/* output low */
	gpio_direction_output(ISDB_RESET_N, false); 	/* output low */
	gpio_set_value(ISDB_EN, 1);			/* high ISDB_EN */
	TcpalmSleep(10);
	TchalResetDevice();
	TchalIrqSetup();
#endif    
	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok][%s:%d]\n", __func__, __LINE__);
}

void TchalPowerDownDevice(void)
{
#if defined(CONFIG_ARCH_MT6582)
        int ret;

	mt_eint_mask(CUST_EINT_DTV_NUM);

	/* Reset */
	ret = mt_set_gpio_pull_enable(GPIO_1SEG_RESET_N, GPIO_PULL_ENABLE);
	ret = mt_set_gpio_pull_select(GPIO_1SEG_RESET_N,GPIO_PULL_DOWN);
	ret = mt_set_gpio_dir(GPIO_1SEG_RESET_N, GPIO_DIR_OUT);
	ret = mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ZERO);

	/* LDO EN */
	//ret = mt_set_gpio_pull_enable(GPIO_1SEG_EN, GPIO_PULL_ENABLE);
	//ret = mt_set_gpio_pull_select(GPIO_1SEG_EN,GPIO_PULL_DOWN);
	//ret = mt_set_gpio_dir(GPIO_1SEG_EN, GPIO_DIR_OUT);
	//ret = mt_set_gpio_out(GPIO_1SEG_EN, GPIO_OUT_ZERO);

	/* DTV LDO EN */
	//ret = mt_set_gpio_pull_enable(GPIO_DTV_LDO_EN, GPIO_PULL_ENABLE);
	//ret = mt_set_gpio_pull_select(GPIO_DTV_LDO_EN,GPIO_PULL_DOWN);
	//ret = mt_set_gpio_dir(GPIO_DTV_LDO_EN, GPIO_DIR_OUT);
	//ret = mt_set_gpio_out(GPIO_DTV_LDO_EN, GPIO_OUT_ZERO);

	/* interrupt */
	ret = mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_ENABLE);
	ret = mt_set_gpio_pull_select(GPIO_1SEG_INT,GPIO_PULL_DOWN);

	/* SPI MISO */
	ret = mt_set_gpio_pull_enable(GPIO82|0x80000000, GPIO_PULL_ENABLE);
	ret = mt_set_gpio_pull_select(GPIO82|0x80000000,GPIO_PULL_DOWN);

	/* SPI MOSI */
	ret = mt_set_gpio_pull_enable(GPIO83|0x80000000, GPIO_PULL_ENABLE);
	ret = mt_set_gpio_pull_select(GPIO83|0x80000000,GPIO_PULL_DOWN);

	/* SPI CLK */
	ret = mt_set_gpio_pull_enable(GPIO81|0x80000000, GPIO_PULL_ENABLE);
	ret = mt_set_gpio_pull_select(GPIO81|0x80000000,GPIO_PULL_DOWN);

	/* SPI CS */
	ret = mt_set_gpio_mode(GPIO80|0x80000000, GPIO_MODE_00); //GPIO_MATV_I2S_WS_PIN_M_GPIO
	ret = mt_set_gpio_pull_enable(GPIO80|0x80000000, GPIO_PULL_ENABLE);
	ret = mt_set_gpio_pull_select(GPIO80|0x80000000,GPIO_PULL_DOWN);
	ret = mt_set_gpio_dir(GPIO80|0x80000000, GPIO_DIR_OUT);
	ret = mt_set_gpio_out(GPIO80|0x80000000, GPIO_OUT_ZERO);

	/* IO power Off */
	ret = hwPowerDown(MT6323_POWER_LDO_VGP3, "1V8_MTK_DTV");
#else
	gpio_set_value(ISDB_RESET_N, 0);		/* low ISDB_RESET_N */
	TcpalmSleep(5);
	gpio_set_value(ISDB_EN, 0);			/* low ISDB_EN */
#endif
	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok][%s:%d]\n", __func__, __LINE__);
}

void TchalIrqMask(void)
{
	mt_eint_mask(CUST_EINT_DTV_NUM);
}

void TchalIrqUnmask(void)
{
	mt_eint_unmask(CUST_EINT_DTV_NUM);
}

extern void tcc353x_irq_handler(void);

void TchalIrqSetup(void)
{
#ifdef CONFIG_ARCH_MT6582
	I32S ret;

	/* interrupt configure */
	ret = (int)mt_eint_set_sens(CUST_EINT_DTV_NUM, CUST_EINT_EDGE_SENSITIVE);
	ret = mt_set_gpio_mode(GPIO_1SEG_INT, GPIO_1SEG_INT_M_EINT);
	ret = mt_set_gpio_dir(GPIO_1SEG_INT, GPIO_DIR_IN);
	ret = mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_ENABLE);
	ret = mt_set_gpio_pull_select(GPIO_1SEG_INT,GPIO_PULL_DOWN);

	/* register irq handler */
	mt_eint_registration(CUST_EINT_DTV_NUM, EINTF_TRIGGER_FALLING/*CUST_EINT_DTV_POLARITY*/, tcc353x_irq_handler, 0);
#else
	gpio_direction_input(ISDB_INT_N);		/* input mode */
#endif
	TcpalPrintStatus((I08S *)"[dtv][tcc3536][ok][%s:%d]\n", __func__, __LINE__);
}

