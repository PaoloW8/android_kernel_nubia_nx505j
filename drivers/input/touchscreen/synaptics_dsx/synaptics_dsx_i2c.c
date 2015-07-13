/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/platform_device.h>

/*** ZTEMT Added by luochangyang, 2013/12/23 ***/
#ifdef CONFIG_OF
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#endif
/***ZTEMT END***/

#include "synaptics_dsx.h"
#include "synaptics_dsx_core.h"

#define SYN_I2C_RETRY_TIMES 2   //10  Modify by luochangyang, 2013/12/19

static int synaptics_rmi4_i2c_set_page(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr)
{
	int retval;
	unsigned char retry;
	unsigned char buf[PAGE_SELECT_LEN];
	unsigned char page;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);

	page = ((addr >> 8) & MASK_8BIT);
	if (page != rmi4_data->current_page) {
		buf[0] = MASK_8BIT;
		buf[1] = page;
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			retval = i2c_master_send(i2c, buf, PAGE_SELECT_LEN);
			if (retval != PAGE_SELECT_LEN) {
				dev_err(rmi4_data->pdev->dev.parent,
						"%s: I2C retry %d\n",
						__func__, retry + 1);
				msleep(20);
			} else {
				rmi4_data->current_page = page;
				break;
			}
		}
	} else {
		retval = PAGE_SELECT_LEN;
	}

	return retval;
}

static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};

	buf = addr & MASK_8BIT;

	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	retval = synaptics_rmi4_i2c_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN) {
		retval = -EIO;
		goto exit;
	}

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(i2c->adapter, msg, 2) == 2) {
			retval = length;
			break;
		}
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C read over retry limit\n",
				__func__);
		retval = -EIO;
	}

exit:
	mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);

	return retval;
}

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	retval = synaptics_rmi4_i2c_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN) {
		retval = -EIO;
		goto exit;
	}

	buf[0] = addr & MASK_8BIT;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(i2c->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C write over retry limit\n",
				__func__);
		retval = -EIO;
	}

exit:
	mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);

	return retval;
}

static struct synaptics_dsx_bus_access bus_access = {
	.type = BUS_I2C,
	.read = synaptics_rmi4_i2c_read,
	.write = synaptics_rmi4_i2c_write,
};

static struct synaptics_dsx_hw_interface hw_if;

static struct platform_device *synaptics_dsx_i2c_device;

static void synaptics_rmi4_i2c_dev_release(struct device *dev)
{
	kfree(synaptics_dsx_i2c_device);

	return;
}

/*** ZTEMT Added by luochangyang, 2013/11/28 ***/
#ifdef CONFIG_OF
static int synaptics_gpio_setup(int gpio, bool configure, int dir, int state)
{
	int retval = 0;
	unsigned char buf[16];

	if (configure) {
		snprintf(buf, PAGE_SIZE, "dsx_gpio_%u\n", gpio);

		retval = gpio_request(gpio, buf);
		if (retval) {
			pr_err("%s: Failed to get gpio %d (code: %d)",
					__func__, gpio, retval);
			return retval;
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);
		if (retval) {
			pr_err("%s: Failed to set gpio %d direction",
					__func__, gpio);
			return retval;
		}
	} else {
		gpio_free(gpio);
	}

	return retval;
}

#if defined CONFIG_ZTEMT_TOUCHSCREEN_SYNAPTICS_S3508
static unsigned char TM_f1a_button_codes[] = {KEY_MENU, KEY_HOME, KEY_BACK};
#elif defined CONFIG_ZTEMT_TOUCHSCREEN_SYNAPTICS_S3208_5keys
static unsigned char TM_f1a_button_codes[] = {KEY_MENU, KEY_MENU, KEY_HOME, KEY_BACK, KEY_BACK};
#elif defined CONFIG_ZTEMT_TOUCHSCREEN_SYNAPTICS_S3208_3keys
static unsigned char TM_f1a_button_codes[] = {KEY_MENU, KEY_HOME, KEY_BACK};
#endif

static struct synaptics_dsx_cap_button_map cap_button_map = {
	.nbuttons = ARRAY_SIZE(TM_f1a_button_codes),
	.map = TM_f1a_button_codes,
};

static int synaptics_parse_dt(struct device *dev, 
        struct synaptics_dsx_board_data *pdata)
{
    int rc;
    int temp_int;
    u32 temp_val;
	struct device_node *np = dev->of_node;

	/* irq gpio info */
	pdata->irq_gpio = of_get_named_gpio(np, "synaptics,irq-gpio", 0);
    pdata->irq_flags = IRQF_TRIGGER_FALLING;

    /* reset gpio info */
	pdata->reset_gpio = of_get_named_gpio(np, "synaptics,reset-gpio", 0);
    
    rc = of_property_read_u32(np, "synaptics,reset_on_state", &temp_int);
    if (rc) {
        dev_err(dev, "Unable to read synaptics,reset_on_state\n");
        return rc;
    } else
        pdata->reset_on_state = temp_int;
    rc = of_property_read_u32(np, "synaptics,reset_delay_ms", &temp_val);
    if (rc) {
        dev_err(dev, "Unable to read synaptics,reset_delay_ms\n");
        return rc;
    } else
        pdata->reset_delay_ms = temp_val;
    rc = of_property_read_u32(np, "synaptics,reset_active_ms", &temp_val);
    if (rc) {
        dev_err(dev, "Unable to read synaptics,reset_active_ms\n");
        return rc;
    } else
        pdata->reset_active_ms = temp_val;

    if (of_property_read_bool(np, "synaptics,power_gpio") == 1) {
        rc = of_property_read_u32(np, "synaptics,power_gpio", &temp_int);
        if (rc) {
            dev_err(dev, "Unable to read synaptics,power_gpio\n");
            return rc;
        } else
            pdata->power_gpio = temp_int;
    } else {
        pdata->power_gpio = -1;
    }
    
    rc = of_property_read_u32(np, "synaptics,power_on_state", &temp_val);
    if (rc) {
        dev_err(dev, "Unable to read synaptics,power_on_state\n");
        return rc;
    } else
        pdata->power_on_state = temp_val;
    rc = of_property_read_u32(np, "synaptics,power_delay_ms", &temp_val);
    if (rc) {
        dev_err(dev, "Unable to read synaptics,power_delay_ms\n");
        return rc;
    } else
        pdata->power_delay_ms = temp_val;

    pdata->gpio_config = synaptics_gpio_setup;
    pdata->cap_button_map = &cap_button_map;
	
	return rc;
}
#else
static int synaptics_parse_dt(struct device *dev, 
        struct synaptics_dsx_board_data *pdata;)
{
	return -ENODEV;
}
#endif
/***ZTEMT END***/

static int synaptics_rmi4_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int retval;
    struct synaptics_dsx_board_data *platform_data; //Added by luochangyang, 2013/12/19

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data commands not supported by host\n",
				__func__);
		return -EIO;
	}

	synaptics_dsx_i2c_device = kzalloc(
			sizeof(struct platform_device),
			GFP_KERNEL);
	if (!synaptics_dsx_i2c_device) {
		dev_err(&client->dev,
				"%s: Failed to allocate memory for synaptics_dsx_i2c_device\n",
				__func__);
		return -ENOMEM;
	}

    /*** ZTEMT Added by luochangyang, 2013/12/19 ***/
    if (client->dev.of_node) {
        platform_data = devm_kzalloc(&client->dev,
            sizeof(struct synaptics_dsx_board_data), GFP_KERNEL);
        if (!platform_data) {
            dev_err(&client->dev, "Failed to allocate memory\n");
            return -ENOMEM;
        }

        retval = synaptics_parse_dt(&client->dev, platform_data);
        if (retval)
            return retval;

        hw_if.board_data = platform_data;
    } else
        hw_if.board_data = client->dev.platform_data;
    /***ZTEMT END***/
	
	hw_if.bus_access = &bus_access;

	synaptics_dsx_i2c_device->name = PLATFORM_DRIVER_NAME;
	synaptics_dsx_i2c_device->id = 0;
	synaptics_dsx_i2c_device->num_resources = 0;
	synaptics_dsx_i2c_device->dev.parent = &client->dev;
	synaptics_dsx_i2c_device->dev.platform_data = &hw_if;
	synaptics_dsx_i2c_device->dev.release = synaptics_rmi4_i2c_dev_release;

	retval = platform_device_register(synaptics_dsx_i2c_device);
	if (retval) {
		dev_err(&client->dev,
				"%s: Failed to register platform device\n",
				__func__);
		return -ENODEV;
	}

	return 0;
}

static int synaptics_rmi4_i2c_remove(struct i2c_client *client)
{
	platform_device_unregister(synaptics_dsx_i2c_device);

	return 0;
}

/*** ZTEMT Added by luochangyang, 2013/12/19 ***/
static struct of_device_id synaptics_rmi4_i2c_of_match[] = {
	{ .compatible = "synaptics_dsx_i2c,rmi4", },
	{ }
};
MODULE_DEVICE_TABLE(of, synaptics_rmi4_i2c_of_match);
/***ZTEMT END***/

static const struct i2c_device_id synaptics_rmi4_id_table[] = {
	{I2C_DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_id_table);

static struct i2c_driver synaptics_rmi4_i2c_driver = {
	.driver = {
		.name = I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = synaptics_rmi4_i2c_of_match,
	},
	.probe = synaptics_rmi4_i2c_probe,
	.remove = __devexit_p(synaptics_rmi4_i2c_remove),
	.id_table = synaptics_rmi4_id_table,
};

int synaptics_rmi4_bus_init(void)
{
	return i2c_add_driver(&synaptics_rmi4_i2c_driver);
}
EXPORT_SYMBOL(synaptics_rmi4_bus_init);

void synaptics_rmi4_bus_exit(void)
{
	i2c_del_driver(&synaptics_rmi4_i2c_driver);

	return;
}
EXPORT_SYMBOL(synaptics_rmi4_bus_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX I2C Bus Support Module");
MODULE_LICENSE("GPL v2");
