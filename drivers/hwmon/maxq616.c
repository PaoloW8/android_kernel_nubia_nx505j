/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
*
* File Name          : maxq616.c
* Authors            : Zhu Bing
* Version            : V.1.0.0
* Date               : 07/024/2013
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
********************************************************************************
Version History.
 
Revision 1-0-0 07/024/2013
 first revision

*******************************************************************************/
 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/i2c/maxq616.h>


#define LOG_TAG "IR_REMOTE"
#define DEBUG_ON //DEBUG SWITCH

#define SENSOR_LOG_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/')+1) : __FILE__

#ifdef  CONFIG_FEATURE_ZTEMT_SENSORS_LOG_ON
#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR   "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
    #ifdef  DEBUG_ON
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO  "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
                                              
#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
    #else
#define SENSOR_LOG_INFO(fmt, args...)
#define SENSOR_LOG_DEBUG(fmt, args...)
    #endif

#else
#define SENSOR_LOG_ERROR(fmt, args...)
#define SENSOR_LOG_INFO(fmt, args...)
#define SENSOR_LOG_DEBUG(fmt, args...)
#endif


static int maxq616_power_init(struct maxq616_chip *chip)
{
	int rc;

    chip->power = regulator_get(&(chip->client->dev), "vdd-chip");

	if (IS_ERR(chip->power))
    {
		rc = PTR_ERR(chip->power);
		SENSOR_LOG_ERROR("Regulator get failed chip->power rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(chip->power) > 0)
    {
		rc = regulator_set_voltage(chip->power, 1800000, 1800000);
		if (rc)
        {
			SENSOR_LOG_ERROR("Regulator set chip->power failed rc=%d\n", rc);
			goto error_set_voltage;
		}
	}
    
    rc = regulator_set_optimum_mode(chip->power, 600000);
    if (rc < 0)
    {
        SENSOR_LOG_ERROR("Regulator chip->power set_opt failed rc=%d\n", rc);
        goto error_set_optimum;
    }
    
    SENSOR_LOG_INFO("success\n");
    return 0;

error_set_optimum:
    regulator_set_voltage(chip->power, 0, 1800000);
    regulator_put(chip->power);

error_set_voltage:
	regulator_put(chip->power);
    SENSOR_LOG_INFO("failed\n");
	return rc;
}



static int maxq616_power_on(struct maxq616_chip *chip, bool enable)
{
	int rc;
    if (enable == chip->power_on)
    {
        SENSOR_LOG_INFO("double %s power, retern here\n",enable? "enable" : "disable");
        return 0;
    }
    else
    {
        SENSOR_LOG_INFO("%s power\n",enable? "enable" : "disable");
    }

    if (enable)
    {
        rc = regulator_enable(chip->power);
        if (rc)
        {
            SENSOR_LOG_ERROR("Regulator chip->power enable failed rc=%d\n", rc);
            goto err_power_enable_failed;
        }
        chip->power_on = true;
    }
    else
    {
        rc = regulator_disable(chip->power);
        if (rc)
        {
            SENSOR_LOG_ERROR("Regulator chip->power enable failed rc=%d\n", rc);
            goto err_power_disable_failed;
        }
        chip->power_on = false;
    }
    
    return 0;

err_power_enable_failed:
err_power_disable_failed:
    return rc;

}

static void maxq616_chip_data_init(struct maxq616_chip *chip)
{
    chip->power_on = false;
}
 
static int __devinit maxq616_probe(struct i2c_client *client,
                  const struct i2c_device_id *id)
{
    static struct maxq616_chip *chip;
    int ret;

    SENSOR_LOG_INFO("prob start\n");

    chip = kzalloc(sizeof(struct maxq616_chip), GFP_KERNEL);
    if (!chip) {
        ret = -ENOMEM;
        goto malloc_chip_failed;
    }

    maxq616_chip_data_init(chip);

    chip->client = client;
    i2c_set_clientdata(client, chip);

    maxq616_power_init(chip);

    maxq616_power_on(chip, true);

    SENSOR_LOG_INFO("prob success\n");

    return 0;

malloc_chip_failed:
    SENSOR_LOG_INFO("prob failed\n");
    return ret;
}

 
 /**
  * maxq616_remove() - remove device
  * @client: I2C client device
  */
 static int __devexit maxq616_remove(struct i2c_client *client)
 {
     struct maxq616_chip *chip = i2c_get_clientdata(client);
     kfree(chip);
     chip = NULL;
     SENSOR_LOG_INFO("maxq616_remove\n");
     return 0;
 }


//resume
static int maxq616_resume(struct device *dev)
{
	struct maxq616_chip *chip = dev_get_drvdata(dev);
    if (0)
    {
        SENSOR_LOG_INFO("enter\n");
        maxq616_power_on(chip, true);
        SENSOR_LOG_INFO("eixt\n");
    }
    return 0 ;
}

//suspend
static int maxq616_suspend(struct device *dev)
{
	struct maxq616_chip *chip = dev_get_drvdata(dev);
    if (0)
    {
        SENSOR_LOG_INFO("enter\n");
        maxq616_power_on(chip, false);
        SENSOR_LOG_INFO("eixt\n");
    }
    return 0 ;
}

static const struct dev_pm_ops maxq616_pm_ops = {
    .suspend = maxq616_suspend,
    .resume  = maxq616_resume,
};


static const struct i2c_device_id maxq616_idtable_id[] = {
     { "uie,maxq616", 0 },
     { },
 };
 
static struct of_device_id of_maxq616_idtable[] = {
     { .compatible = "uei,maxq616",},
     {}
};
 
 MODULE_DEVICE_TABLE(i2c, maxq616_idtable);
 
 static struct i2c_driver maxq616_driver = {
     .driver = {
         .name = "maxq616",
         .of_match_table = of_maxq616_idtable,
         .pm = &maxq616_pm_ops,
     },
     .id_table = maxq616_idtable_id,
     .probe = maxq616_probe,
     .remove = __devexit_p(maxq616_remove),
 };



static int __init maxq616_init(void)
{
        SENSOR_LOG_INFO("driver: init\n");
        return i2c_add_driver(&maxq616_driver);
}
 
static void __exit maxq616_exit(void)
{
        SENSOR_LOG_INFO("driver: exit\n");
        i2c_del_driver(&maxq616_driver);
}

module_init(maxq616_init);
module_exit(maxq616_exit);
 
MODULE_DESCRIPTION("UEI maxq616 driver");
MODULE_AUTHOR("ZhuBing, ZTEMT");
MODULE_LICENSE("GPL");
