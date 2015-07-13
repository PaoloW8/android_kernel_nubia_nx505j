/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
*
* File Name          : hts221.c
* Authors            : Motion MEMS
*                      Morris Chen (morris.chen@st.com)
* Version            : V.1.0.0
* Date               : 06/03/2013
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
 
Revision 1-0-0 06/03/2013
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
#include <linux/i2c/sensor_common.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/miscdevice.h>
#include <linux/of_gpio.h>


#define LOG_TAG "SENSOR_COMMON"
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

static const struct i2c_device_id sensor_common_idtable_id[] = {
    { "zte,sensor_common", 0 },
    { },
};
 
static struct of_device_id of_sensor_common_idtable[] = {
    { .compatible = "zte,sensor_common",},
    {}
};
 
 MODULE_DEVICE_TABLE(i2c, sensor_common_idtable);
 
static struct i2c_driver sensor_common_driver = {
    .driver = {
        .name = "sensor_common",
        .of_match_table = of_sensor_common_idtable,
        .pm = NULL,
    },
    .id_table = sensor_common_idtable_id,
    .probe = sensor_common_probe,
    .remove = __devexit_p(sensor_common_remove),
};


struct class  *sensor_common_class;

static dev_t const sensor_compass_dev               =   MKDEV(MISC_MAJOR, 210);
static dev_t const sensor_temp_humidity_dev         =   MKDEV(MISC_MAJOR, 211);

int sensor_common_read_file_int(char *file_path)
{
    struct file *file_p;
    int vfs_read_retval = 0;
    mm_segment_t old_fs; 
    char read_buf[32];
    //unsigned short read_value;
    int read_value;

    if (NULL==file_path)
    {
        SENSOR_LOG_ERROR("file_path is NULL\n");
        goto error;
    }

    memset(read_buf, 0, 32);

    file_p = filp_open(file_path, O_RDONLY, 0);
    if (IS_ERR(file_p))
    {
        SENSOR_LOG_ERROR("[open file <%s>failed]\n",file_path);
        goto error;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    
    vfs_read_retval = vfs_read(file_p, (char*)read_buf, 16, &file_p->f_pos);
    if (vfs_read_retval < 0)
    {
        SENSOR_LOG_ERROR("[read file <%s>failed]\n",file_path);
        goto error;
    }

    set_fs(old_fs);
    filp_close(file_p, NULL);

    read_value = simple_strtol(read_buf, NULL, 10);

    return read_value;

error:
    return 0;
}

int sensor_common_read_file_char(char *file_path, char *save_buf)
{
    struct file *file_p;
    int vfs_read_retval = 0;
    mm_segment_t old_fs; 
    char read_buf[32];

    if (NULL==file_path)
    {
        SENSOR_LOG_ERROR("file_path is NULL\n");
        goto error;
    }

    memset(read_buf, 0, 32);

    file_p = filp_open(file_path, O_RDONLY, 0);
    if (IS_ERR(file_p))
    {
        SENSOR_LOG_ERROR("[open file <%s>failed]\n",file_path);
        goto error;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    
    vfs_read_retval = vfs_read(file_p, (char*)read_buf, 16, &file_p->f_pos);
    if (vfs_read_retval < 0)
    {
        SENSOR_LOG_ERROR("[read file <%s>failed]\n",file_path);
        goto error;
    }

    set_fs(old_fs);
    filp_close(file_p, NULL);

    SENSOR_LOG_ERROR("read_buf is %s\n",read_buf);

    return sprintf(save_buf, "%s",read_buf);


error:
    return 0;
}



static ssize_t compass_int_pin_get(struct device *dev,struct device_attribute *attr, char *buf)
{    
	struct sensor_common_data *chip = dev_get_drvdata(dev);
    return sprintf(buf, "gpio is %d\n",gpio_get_value(chip->compass_irq_gpio));
}
 

static ssize_t compass_int_pin_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    bool value;
	struct sensor_common_data *chip = dev_get_drvdata(dev);

    if (strtobool(buf, &value))
        return -EINVAL;

    if (value)
    { 
        SENSOR_LOG_INFO("set to be 1\n");
        gpio_set_value(chip->compass_irq_gpio, 1);
    }
    else
    {
        SENSOR_LOG_INFO("set to be 0\n");
        gpio_set_value(chip->compass_irq_gpio, 0);
    }

    return size;
}



static struct device_attribute attrs_sensor_compass[] = {
	__ATTR(int_pin,                  0640,   compass_int_pin_get,          compass_int_pin_set),
};



static int sensor_compass_create_sysfs_interfaces(struct device *dev)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(attrs_sensor_compass); i++)
     if (device_create_file(dev, attrs_sensor_compass + i))
         goto error;
    return 0;

error:
    for ( ; i >= 0; i--)
    device_remove_file(dev, attrs_sensor_compass + i);
    SENSOR_LOG_ERROR("Unable to create interface\n");
    return -1;
}




static ssize_t attr_raw_temp_get(struct device *dev,struct device_attribute *attr, char *buf)
{
    static int raw_temp;

    raw_temp = sensor_common_read_file_int(PATH_RAW_TEMP);
    SENSOR_LOG_INFO("raw_temp is %d\n",raw_temp);
    return sprintf(buf, "%d\n", raw_temp);
}

static ssize_t attr_raw_humidity_get(struct device *dev,struct device_attribute *attr, char *buf)
{
    static int raw_humidity;

    raw_humidity = sensor_common_read_file_int(PATH_RAW_HUMIDITY);
    SENSOR_LOG_INFO("raw_humidity is %d\n",raw_humidity);
    return sprintf(buf, "%d\n", raw_humidity);
}


static ssize_t attr_timestamp_get(struct device *dev,struct device_attribute *attr, char *buf)
{
    static struct timespec time;
    time = current_kernel_time();
    
    return sprintf(buf, "%ld.%ld\n",time.tv_sec,time.tv_nsec);
}


static ssize_t attr_temp_cpu_0_get(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", sensor_common_read_file_int(PATH_TEMP_CPU_0));
}

static ssize_t attr_temp_cpu_1_get(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", sensor_common_read_file_int(PATH_TEMP_CPU_1));
}

static ssize_t attr_temp_cpu_2_get(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", sensor_common_read_file_int(PATH_TEMP_CPU_2));
}

static ssize_t attr_temp_cpu_3_get(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", sensor_common_read_file_int(PATH_TEMP_CPU_3));
}

static ssize_t attr_temp_rf(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n",sensor_common_read_file_int(PATH_TEMP_RF));
}

static ssize_t attr_temp_battery(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n",sensor_common_read_file_int(PATH_TEMP_BATTERY));
}

static ssize_t attr_temp_charging(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n",sensor_common_read_file_int(PATH_TEMP_CHARGING));
}

static ssize_t attr_temp_lcd(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n",sensor_common_read_file_int(PATH_TEMP_LCD));
}


static ssize_t attr_lcd_brightness_get(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", sensor_common_read_file_int(PATH_LCD_BRIGHTNESS)*100 / 255);
}

static ssize_t attr_battery_level(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n",sensor_common_read_file_int(PATH_BATTERY_LEVEL));
}

static ssize_t attr_battery_state(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n",sensor_common_read_file_int(PATH_BATTERY_STATE));
}

static ssize_t attr_charge_current(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n",sensor_common_read_file_int(PATH_CHARGE_CURRENT));
}


/*
 "Unknown", "Charging", "Discharging", "Not charging", "Full"
     0          1             2              3           4
 */
static ssize_t attr_charge_state(struct device *dev,struct device_attribute *attr, char *buf)
{

    char *charge_status[5] = {"Unknown", "Charging", "Discharging", "Not charging", "Full"};
    char save_buf[32];
    int charge_state = 0;
    int i = 0;

    sensor_common_read_file_char(PATH_CHARGE_STATE, save_buf);

    for (i=0; i<5; i++)
    {
        SENSOR_LOG_ERROR("charge_status[%d] is %s\n",i,charge_status[i]);
        if (0 == strncmp(save_buf, charge_status[i], strlen(charge_status[i])))
        {
            charge_state = i;
            break;
        }
    }
   
    return sprintf(buf, "%d\n",charge_state);
}

static ssize_t attr_rf_state(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "1\n");
}

static struct device_attribute attrs_sensor_temp_humidity[] = {
	__ATTR(timestamp,                  0644,        attr_timestamp_get,                  NULL),
    __ATTR(raw_temp,                   0644,        attr_raw_temp_get,                   NULL),
    __ATTR(raw_humidity,               0644,        attr_raw_humidity_get,               NULL),
    __ATTR(temp_cpu_0,                 0644,        attr_temp_cpu_0_get,                 NULL),
    __ATTR(temp_cpu_1,                 0644,        attr_temp_cpu_1_get,                 NULL),
    __ATTR(temp_cpu_2,                 0644,        attr_temp_cpu_2_get,                 NULL),
    __ATTR(temp_cpu_3,                 0644,        attr_temp_cpu_3_get,                 NULL),
    __ATTR(temp_rf,                    0644,        attr_temp_rf,                        NULL),
    __ATTR(temp_battery,               0644,        attr_temp_battery,                   NULL),
    __ATTR(temp_charging,              0644,        attr_temp_charging,                  NULL),
    __ATTR(temp_lcd,                   0644,        attr_temp_lcd,                       NULL),
    __ATTR(lcd_brightness,             0644,        attr_lcd_brightness_get,             NULL),
    __ATTR(battery_level,              0644,        attr_battery_level,                  NULL),
    __ATTR(battery_state,              0644,        attr_battery_state,                  NULL),
    __ATTR(charge_current,             0644,        attr_charge_current,                 NULL),
    __ATTR(charge_state,               0644,        attr_charge_state,                   NULL),
    __ATTR(rf_state,                   0644,        attr_rf_state,                       NULL),

};

static int sensor_temp_humidity_create_sysfs_interfaces(struct device *dev)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(attrs_sensor_temp_humidity); i++)
     if (device_create_file(dev, attrs_sensor_temp_humidity + i))
         goto error;
    return 0;

error:
    for ( ; i >= 0; i--)
    device_remove_file(dev, attrs_sensor_temp_humidity + i);
    SENSOR_LOG_ERROR("Unable to create interface\n");
    return -1;
}


static int sensor_compass_int_pin_init(int pin_num)
{
    int ret = 0;

    ret = gpio_request(pin_num, "compass_int");
    if (ret)    
    {
        SENSOR_LOG_INFO("gpio %d is busy and then to free it\n",pin_num);
        
        gpio_free(pin_num);
        ret = gpio_request(pin_num, "compass_int");
        if (ret) 
        {
            SENSOR_LOG_INFO("gpio %d is busy and then to free it\n",pin_num);
            return ret;
        }
    }
    else
    {
        SENSOR_LOG_INFO("gpio %d get success\n",pin_num);
    }

    ret = gpio_tlmm_config(GPIO_CFG(pin_num, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if (ret < 0)
    {
        SENSOR_LOG_ERROR("gpio_tlmm_config failed ret = %d",ret);
    }
    
    return ret;
}

static int sensor_common_parse_dt(struct sensor_common_data *chip)
{
	struct device_node *np = chip->client->dev.of_node;
	chip->compass_irq_gpio = of_get_named_gpio(np, "compass,irq-gpio", 0);
    SENSOR_LOG_INFO("compass_irq_gpio is %d\n",chip->compass_irq_gpio);
    return 0;
}


static int __devinit sensor_common_probe(struct i2c_client *client,
                  const struct i2c_device_id *id)
{

    int ret;

    struct sensor_common_data *chip_data;

    SENSOR_LOG_INFO("prob start\n");

	chip_data = kzalloc(sizeof(struct sensor_common_data), GFP_KERNEL);
    chip_data->client = client;

    i2c_set_clientdata(client, chip_data);

    sensor_common_parse_dt(chip_data);

    sensor_common_class = class_create(THIS_MODULE, "sensor");

    chip_data->sensor_compass_dev = device_create(sensor_common_class, NULL, sensor_compass_dev, &sensor_common_driver ,"compass");
    if (IS_ERR(chip_data->sensor_compass_dev)) 
    {
        ret = PTR_ERR(chip_data->sensor_compass_dev);
        goto create_sensor_compass_failed;
    }
    dev_set_drvdata(chip_data->sensor_compass_dev, chip_data);


    chip_data->sensor_temp_humidity_dev = device_create(sensor_common_class, NULL, sensor_temp_humidity_dev, &sensor_common_driver ,"temp_humidity");
    if (IS_ERR(chip_data->sensor_temp_humidity_dev)) 
    {
        ret = PTR_ERR(chip_data->sensor_temp_humidity_dev);
        goto create_sensor_sensor_temp_humidity_failed;
    }
    dev_set_drvdata(chip_data->sensor_temp_humidity_dev, chip_data);

    sensor_compass_create_sysfs_interfaces(chip_data->sensor_compass_dev);
    sensor_temp_humidity_create_sysfs_interfaces(chip_data->sensor_temp_humidity_dev);


    sensor_compass_int_pin_init(chip_data->compass_irq_gpio);


    SENSOR_LOG_INFO("prob success\n");

    return 0;

create_sensor_sensor_temp_humidity_failed:
    chip_data->sensor_compass_dev = NULL;
    class_destroy(sensor_common_class); 


create_sensor_compass_failed:
    chip_data->sensor_compass_dev = NULL;
    class_destroy(sensor_common_class); 

    return ret;
}

 
 /**
  * sensor_common_remove() - remove device
  * @client: I2C client device
  */
 static int __devexit sensor_common_remove(struct i2c_client *client)
 {
     struct sensor_common_data *chip_data = i2c_get_clientdata(client);
 
      SENSOR_LOG_INFO("sensor_common_remove\n");
     //hwmon_device_unregister(chip_data->hwmon_dev);
     //sysfs_remove_group(&client->dev.kobj, &sensor_common_attr_group);
    
     kfree(chip_data);
     return 0;
 }

static int __init sensor_common_init(void)
{
        SENSOR_LOG_INFO("driver: init\n");
        return i2c_add_driver(&sensor_common_driver);
}
 
static void __exit sensor_common_exit(void)
{
        SENSOR_LOG_INFO("driver: exit\n");
        i2c_del_driver(&sensor_common_driver);
}

module_init(sensor_common_init);
module_exit(sensor_common_exit);
 
MODULE_DESCRIPTION("sensor commom driver");
MODULE_AUTHOR("ZhuBing, ZTEMT");
MODULE_LICENSE("GPL");
