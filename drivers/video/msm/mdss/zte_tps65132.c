/*
 * tps65312 driver
 *
 * Copyright (C) 2014 ZTEMT
 *
 * Create by mayu.
 * Modify by luochangyang 2014.06.01
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/device.h>

#include "zte_tps65132.h"

#define TPS65132_DRIVER_NAME	"tps65132"
//#define NXP_TPS65132_DEBUG

#if defined(CONFIG_ZTEMT_MIPI_1080P_R63311_SHARP_IPS_6P4)
#define POSITIVE_VOLTAGE 5600
#define NEGATIVE_VOLTAGE 5600
#elif defined(CONFIG_ZTEMT_MIPI_1080P_R63311_SHARP_IPS_5P5)
#define POSITIVE_VOLTAGE 5600
#define NEGATIVE_VOLTAGE 5400
#elif defined(CONFIG_ZTEMT_MIPI_2K_R63419_SHARP_IPS_5P5)
#define POSITIVE_VOLTAGE 5800
#define NEGATIVE_VOLTAGE 5800
#else
#define POSITIVE_VOLTAGE 5000
#define NEGATIVE_VOLTAGE 5000
#endif

enum tps65132_irq {
	TPS65132_NONE,
	TPS65132_INT,
};

struct tps65132_data	{
	wait_queue_head_t	read_wq;
	struct mutex read_mutex;
	struct i2c_client	*client;
	struct miscdevice	miscdev;
	bool irq_enabled;
	spinlock_t irq_enabled_lock;
	enum tps65132_irq read_irq;
	
	int updata_gpio;
	int ven_gpio;
	int irq_gpio;
	int (*request_resources) (void);
	void (*free_resources) (void);
	void (*enable) (int fw);
	int (*test) (void);
	void (*disable) (void);
	int (*irq_status) (void);
};

static struct tps65132_data * tps65132_dev;

static int tps65132_write_reg_val(unsigned char reg, unsigned char val)
{
	int i = 0;
	int err = 0;

	while (i < 3)
	{
#ifdef NXP_TPS65132_DEBUG
		printk("%s: 0x%x:0x%x \n", __FUNCTION__,reg, val);
#endif
		err = i2c_smbus_write_byte_data(tps65132_dev->client, reg, val);
		if(err < 0){
			printk(KERN_ERR"%s, err=%d\n", __FUNCTION__, err);
			i++;
		} else {
			break;
		}
	}

	return err;
}

static unsigned char tps65132_read_reg( unsigned char reg)
{
	return i2c_smbus_read_byte_data(tps65132_dev->client, reg);
}

void tps65132_set_output_voltage(int positive_voltage, int negative_voltage)
{
	int pos_val=0;
	int neg_val=0;
	int regff_val=0;
	
	pos_val= ( positive_voltage - 4000)/100;
	neg_val= ( negative_voltage - 4000)/100;	

#ifdef NXP_TPS65132_DEBUG
	printk("%s:positive_voltage=%d,negative_voltage=%d, ",__func__,positive_voltage,negative_voltage);
	printk("pos_val=0x%x , neg_val= 0x%x ~~~~~~~~~\n",pos_val,neg_val);
#endif

	tps65132_write_reg_val(0x00,pos_val);
	tps65132_write_reg_val(0x01,neg_val);
	regff_val=tps65132_read_reg(0xff)	;
#ifdef NXP_TPS65132_DEBUG
	printk("regff_val=0x%x~~~~~~~~~\n",regff_val);	
#endif
	regff_val |=(1<<8);
#ifdef NXP_TPS65132_DEBUG
	printk("regff_val=0x%x~~~~~~~~~\n",regff_val);
#endif
	tps65132_write_reg_val(0xff,regff_val);	

}

void  tps65132_show_reg(void )
{
	int val0,val1,regff_val;
	val0=tps65132_read_reg(0x00);
	val1=tps65132_read_reg(0x01);
	regff_val=tps65132_read_reg(0xff);
#ifdef NXP_TPS65132_DEBUG
	printk("%s:reg00=0x%x, reg01=0x%x ,regff_val= 0x%x~~~~~~~~~~~~\n",__func__,val0,val1,regff_val);
#endif	
}

void tps65132_set_output_avdd(void)
{
    pr_info("%s: +%d -%d\n", __func__, POSITIVE_VOLTAGE, NEGATIVE_VOLTAGE);
	tps65132_set_output_voltage(POSITIVE_VOLTAGE, NEGATIVE_VOLTAGE);
}

static __devinit int tps65132_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int irq_gpio = -1;
	int updata_gpio = -1;
	int ven_gpio = -1;
	int ret;
    struct regulator *vcc_i2c;
  
    dev_info(&client->dev, "%s : Start init.\n", __func__);

#ifdef NXP_TPS65132_DEBUG
	printk("[tps] %s:line%d +\n",__FUNCTION__,__LINE__);
#endif
	//I2C POWER
	vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
	if (IS_ERR(vcc_i2c))
	{
		ret = PTR_ERR(vcc_i2c);
		dev_err(&client->dev, "Regulator get failed rc=%d\n", ret);
	}
	ret = regulator_enable(vcc_i2c);

	if (ret)
		dev_err(&client->dev, "Regulator vcc_i2c enable failed rc=%d\n", ret);

	if (tps65132_dev != NULL) {
		dev_warn(&client->dev, "only one tps65132 supported.\n");
		return -EBUSY;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}
	
	tps65132_dev = kzalloc(sizeof(struct tps65132_data), GFP_KERNEL);
	if (tps65132_dev == NULL) {
		dev_err(&client->dev, "failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}
	
	tps65132_dev->client = client;
	tps65132_dev->irq_gpio = irq_gpio;
	tps65132_dev->updata_gpio = updata_gpio;
	tps65132_dev->ven_gpio = ven_gpio;
	
	i2c_set_clientdata(client, tps65132_dev);
	
	tps65132_set_output_avdd();

#ifdef NXP_TPS65132_DEBUG
   printk("[tps] %s:line%d -\n",__FUNCTION__,__LINE__);
#endif

	return 0;

err_exit:
	return ret;
	
}

static __devexit int tps65132_remove(struct i2c_client *client)
{
	struct tps65132_data *dev = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s\n", __func__);

	if (dev->disable)
		dev->disable();

	mutex_destroy(&dev->read_mutex);
	kfree(dev);

	tps65132_dev = NULL;
	
	return 0;
}

static struct of_device_id tps_match_table[] = {
	{ .compatible = "tps,tps65132_i2c_adapter",},
	{ },
};

static const struct i2c_device_id tps65132_id_table[] = {
	{ TPS65132_DRIVER_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tps65132_id_table);

static struct i2c_driver tps65132_driver = {
	.id_table	= tps65132_id_table,
	.probe		= tps65132_probe,
	.remove		= tps65132_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= 	TPS65132_DRIVER_NAME,
		.of_match_table = tps_match_table,
	},
};

module_i2c_driver(tps65132_driver);

MODULE_DESCRIPTION("I2C_TEST_TPS");
MODULE_LICENSE("GPL");

