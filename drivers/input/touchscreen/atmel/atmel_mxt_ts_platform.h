/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_PLATFORM_H
#define __LINUX_ATMEL_MXT_TS_PLATFORM_H

#include <linux/types.h>

#define CONFIG_MXT_PROBE_ALTERNATIVE_CHIP 8
//#define CONFIG_MXT_POWER_CONTROL_SUPPORT_AT_PROBE
//#define CONFIG_MXT_T38_SKIP_LEN_AT_UPDATING 2
#define CONFIG_MXT_VENDOR_ID_BY_T19
//#define CONFIG_MXT_FORCE_RESET_AT_POWERUP

//#define CONFIG_MXT_SELFCAP_TUNE

//#define CONFIG_MXT_PLATFORM_MTK
#define CONFIG_DUMMY_PARSE_DTS
#define CONFIG_MXT_PLATFORM_QUALCOMM
#if defined(CONFIG_MXT_PLATFORM_MTK)
#	define CONFIG_MXT_IRQ_WORKQUEUE
#	define CONFIG_MXT_EXTERNAL_TRIGGER_IRQ_WORKQUEUE
#	define CONFIG_MXT_EXTERNAL_MODULE
#	define CONFIG_MXT_I2C_DMA
#else
#	define CONFIG_FB_PM
#	define CONFIG_MXT_IRQ_NESTED
#endif

#if defined(CONFIG_MXT_PLATFORM_MTK)
#include <mach/mt_pm_ldo.h>
#include <cust_eint.h>
#include "cust_gpio_usage.h"
#include <mach/mt_gpio.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_typedefs.h>
#include <mach/eint.h>
#include <mach/mt_pm_ldo.h>
#include "tpd.h"
#endif

#if defined(CONFIG_MXT_PLATFORM_QUALCOMM)
#include <linux/io.h>
//#include <mach/gpio.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#endif

#if defined(CONFIG_MXT_EXTERNAL_MODULE)
#if !defined(TPD_DEVICE)
#include "atmel_mxt_ts_mtk_dummy.h"
#endif
#endif


#if defined(CONFIG_MXT_I2C_DMA)
#include <linux/dma-mapping.h>
static int __mxt_read(struct i2c_client *client,
			void *val, u16 len);
static int __mxt_write(struct i2c_client *client,
			const void *val, u16 len);
#endif

#if defined(CONFIG_MXT_EXTERNAL_TRIGGER_IRQ_WORKQUEUE) || defined(CONFIG_MXT_REPORT_VIRTUAL_KEY_SLOT_NUM)
static struct mxt_data *mxt_g_data;
#endif

#if defined(CONFIG_MXT_PLATFORM_MTK)
static void board_pulse_irq_thread(void);
#endif

/*** ZTEMT start ***/
extern int mxt_rst_number;
extern int mxt_int_number;
#define MXT_RST_PORT    mxt_rst_number //gpio_to_irq(mxt_rst_number)
#define MXT_INT_PORT    mxt_int_number //gpio_to_irq(mxt_int_number)

//#define ATMEL_PINCTRL_STATE_SLEEP "atmel_pin_suspend"
//#define ATMEL_PINCTRL_STATE_DEFAULT "atmel_pin_active"
/*ZTEMT end*/

static inline void board_gpio_init(const struct mxt_platform_data *pdata)
{

	// if gpio init in board, or use regulator , skip this function

	/* set irq pin input/pull high */
	/*	 power off vdd/avdd   */
	/*		msleep(100);		*/
	/* 	  set reset output 0 	*/
	/*	 power up vdd/avdd	*/
	/*		msleep(50);			*/
	/*	   set reset output 1 	*/
	/*		msleep(200);		*/

/*#if defined(CONFIG_MXT_PLATFORM_MTK)
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(10);
	hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_2800, "TP");
	hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_1800, "TP");
	msleep(50);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#if 0
	msleep(50);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(50);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	//printk("%s:reset-gpio:%d\n",__func__, mt_get_gpio_out(GPIO_CTP_RST_PIN));
#endif
	msleep(200);
#endif
*/
#if defined(CONFIG_MXT_PLATFORM_QUALCOMM)
    /*** ZTEMT start ***/
#if 1
	int rc = 0;

    if (!pdata) {
		printk("ATMEL ERROR: NULL Pointer detected!\n");
		WARN_ON(1);
		//return -EINVAL;
	}
	/* reset, irq gpio info */
   // pdata->gpio_reset = MXT_RST_PORT;
   // pdata->gpio_irq = MXT_INT_PORT;

   // printk("------pdata->gpio_reset---------: %d  MXT_RST_PORT= %ud \n",(pdata->gpio_reset),mxt_rst_number);
   // printk("------pdata->gpio_irq-----------: %d  MXT_INT_PORT= %ud \n",(pdata->gpio_irq), mxt_int_number);
    printk("--------mxt_rst_number------------: %ud\n",mxt_rst_number);
    printk("--------mxt_int_number------------: %ud\n",mxt_int_number);

	if(!gpio_is_valid(mxt_rst_number))
		printk("ATMEL ERROR: gpio_is_valid(MXT_RST_PORT)!\n");//return -ENODEV;
        rc = gpio_request(mxt_rst_number, "MXT_RST_PORT");
    if (rc < 0) 
    	{
        printk("Failed to request GPIO:%ud, ERRNO:%d", (s32)mxt_rst_number, rc);
        rc = -ENODEV;
    	}
   	 else
   	 {
            gpio_direction_output(mxt_rst_number, 0);
            msleep(50);
            gpio_direction_output(mxt_rst_number, 1);
            msleep(50);
		
   	 }

	if(!gpio_is_valid(mxt_int_number))
		printk("ATMEL ERROR: gpio_is_valid(MXT_INT_PORT)!\n");//return -ENODEV;

	rc = gpio_request(mxt_int_number, "MXT_INT_PORT");
	if (rc < 0) {
			printk("Failed request ATMEL_I2C_IRQ_GPIO.\n");
			//return rc;
		}
	gpio_direction_input(mxt_int_number);

    printk("%s: INIT ATMEL RST gpio=%ud and IRQ gpio=%ud r=%d\n",__func__, mxt_rst_number, mxt_int_number, rc);
	
#else
    /*ZTEMT end*/

	// if gpio init in board, null this
	//don't use the read_chg in mxt_wait_for_chg, because the msg can be readout by interrupt thread
	MXT_GPIO_REQUEST(MXT_INT_PORT, "MXT_INT_IRQ");
	MXT_GPIO_REQUEST(MXT_RST_PORT, "MXT_RST_PORT");
	
	// set irq pin input/pull high
	MXT_GPIO_AS_INPUT(MXT_INT_PORT);
	// set reset output 0
	MXT_GPIO_OUTPUT(MXT_RST_PORT, 0);
	msleep(10);
	// set reset output 1
	MXT_GPIO_OUTPUT(MXT_RST_PORT, 1);
	msleep(200);
#endif
    return;

#endif
}
static inline void board_gpio_deinit(const struct mxt_platform_data *pdata)
{
#if defined(CONFIG_MXT_PLATFORM_QUALCOMM)
#if 1
/*** ZTEMT start ***/
    gpio_free(mxt_int_number);
    gpio_free(mxt_rst_number);
/*ZTEMT end*/
#else
	MXT_GPIO_FREE(MXT_INT_PORT);
	MXT_GPIO_FREE(MXT_RST_PORT);
#endif
#endif   
}
static void board_init_irq(const struct mxt_platform_data *pdata)
{
//Here should math irqflags in interface file
/*  
	1 IRQF_TRIGGER_FALLING: 
		<a>should set auto_unmask bit.
		<b>.irqflags = IRQF_TRIGGER_FALLING
	
	2 IRQF_TRIGGER_LOW: 
		<a>shouldn't set auto_unmask bit
		<b>.irqflags = IRQF_TRIGGER_LOW
*/
#if defined(CONFIG_MXT_PLATFORM_MTK)
	#if defined(CONFIG_USE_FALLING_IRQ) 
		//.irqflags = IRQF_TRIGGER_FALLING
		//CUST_EINTF_TRIGGER_FALLING
		//mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
		mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
		mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINTF_TRIGGER_FALLING, board_pulse_irq_thread, 1); 
		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	#else
		//.irqflags = IRQF_TRIGGER_LOW
		//CUST_EINTF_TRIGGER_LOW_LEVEL
	#if defined(CONFIG_6575)
		mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_LEVEL_SENSITIVE);
		mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
		mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, board_pulse_irq_thread, /*1*/0); 
		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	#endif
	#if defined(CONFIG_6592)
		mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
		mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, /*CUST_EINTF_TRIGGER_FALLING*/EINTF_TRIGGER_LOW, board_pulse_irq_thread, 0); 
		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	#endif
#endif
#endif

#if defined(CONFIG_MXT_PLATFORM_MTK_EXAMPLE_2)
	//.irqflags = IRQF_TRIGGER_FALLING
	//CUST_EINTF_TRIGGER_FALLING
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	msleep(50);

	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINTF_TRIGGER_FALLING, board_pulse_irq_thread, 1);
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
}

static inline void board_enable_irq(const struct mxt_platform_data *pdata, unsigned int irq)
{
#if defined(CONFIG_MXT_PLATFORM_MTK)
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#else
	enable_irq(irq);
#endif
}

static inline void board_enable_irq_wake(const struct mxt_platform_data *pdata, unsigned int irq)
{
#if defined(CONFIG_MXT_PLATFORM_MTK)
#else
	enable_irq_wake(irq);
#endif
}

static inline void board_disable_irq_wake(const struct mxt_platform_data *pdata, unsigned int irq)
{
#if defined(CONFIG_MXT_PLATFORM_MTK)
#else
	disable_irq_wake(irq);
#endif
}

static inline void board_disable_irq(const struct mxt_platform_data *pdata, unsigned int irq)
{
#if defined(CONFIG_MXT_PLATFORM_MTK)
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#else
	disable_irq(irq);
#endif
}

static inline void board_free_irq(const struct mxt_platform_data *pdata, unsigned int irq, void *dev_id)
{	
#if defined(CONFIG_MXT_PLATFORM_MTK)
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#else
	free_irq(irq,dev_id);
#endif
}

#endif /* __LINUX_ATMEL_MXT_TS_PLATFORM_H */
