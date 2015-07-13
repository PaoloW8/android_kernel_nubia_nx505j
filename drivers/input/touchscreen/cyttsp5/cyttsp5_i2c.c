/*
 * cyttsp5_i2c.c
 * Cypress TrueTouch(TM) Standard Product V5 I2C Module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2012-2013 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include "cyttsp5_bus.h"
#include "cyttsp5_core.h"
#include "cyttsp5_i2c.h"

#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
/*** ZTEMT Added by luochangyang, 2013/09/03  ***/
#ifdef CONFIG_OF
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#endif
/***ZTEMT END***/

#include "cyttsp5_devtree.h"

#define CY_I2C_DATA_SIZE  (3 * 256)

struct cyttsp5_i2c {
	struct i2c_client *client;
	u8 wr_buf[CY_I2C_DATA_SIZE];
	char const *id;
	struct mutex lock;
};

static int cyttsp5_i2c_read_default_(struct cyttsp5_adapter *adap,
	void *buf, int size)
{
	struct cyttsp5_i2c *ts = dev_get_drvdata(adap->dev);
	struct i2c_client *client = ts->client;
	int rc;

	if (!buf || !size)
		return -EINVAL;

	rc = i2c_master_recv(client, buf, size);

	return (rc < 0) ? rc : rc != size ? -EIO : 0;
}

static int cyttsp5_i2c_read_default(struct cyttsp5_adapter *adap,
	void *buf, int size)
{
	struct cyttsp5_i2c *ts = dev_get_drvdata(adap->dev);
	int rc;

	pm_runtime_get_noresume(adap->dev);
	mutex_lock(&ts->lock);
	rc = cyttsp5_i2c_read_default_(adap, buf, size);
	mutex_unlock(&ts->lock);
	pm_runtime_put_noidle(adap->dev);

	return rc;
}

static int cyttsp5_i2c_read_default_nosize_(struct cyttsp5_adapter *adap,
	u8 *buf, u32 max)
{
	struct cyttsp5_i2c *ts = dev_get_drvdata(adap->dev);
	struct i2c_client *client = ts->client;
	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;
	u32 size;

	if (!buf)
		return -EINVAL;

	msgs[0].addr = ts->client->addr;
	msgs[0].flags = (client->flags & I2C_M_TEN) | I2C_M_RD;
	msgs[0].len = 2;
	msgs[0].buf = buf;
	rc = i2c_transfer(client->adapter, msgs, msg_count);
	if (rc < 0 || rc != msg_count)
		return (rc < 0) ? rc : -EIO;

	size = get_unaligned_le16(&buf[0]);
	if (!size)
		return 0;

	if (size > max)
		return -EINVAL;

	rc = i2c_master_recv(client, buf, size);

	return (rc < 0) ? rc : rc != (int)size ? -EIO : 0;
}

static int cyttsp5_i2c_read_default_nosize(struct cyttsp5_adapter *adap,
	void *buf, int max)
{
	struct cyttsp5_i2c *ts = dev_get_drvdata(adap->dev);
	int rc;

	pm_runtime_get_noresume(adap->dev);
	mutex_lock(&ts->lock);

	rc = cyttsp5_i2c_read_default_nosize_(adap, buf, max);

	mutex_unlock(&ts->lock);
	pm_runtime_put_noidle(adap->dev);

	return rc;
}

static int cyttsp5_i2c_write_read_specific_(struct cyttsp5_adapter *adap,
		u8 write_len, u8 *write_buf, u8 *read_buf)
{
	struct cyttsp5_i2c *ts = dev_get_drvdata(adap->dev);
	struct i2c_client *client = ts->client;
	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;

	if (!write_buf || !write_len)
		return -EINVAL;

	msgs[0].addr = ts->client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len = write_len;
	msgs[0].buf = write_buf;
	rc = i2c_transfer(client->adapter, msgs, msg_count);

	if (rc < 0 || rc != msg_count)
		return (rc < 0) ? rc : -EIO;
	else
		rc = 0;

	if (read_buf)
		rc = cyttsp5_i2c_read_default_nosize_(adap, read_buf, 512);

	return rc;
}

static int cyttsp5_i2c_write_read_specific(struct cyttsp5_adapter *adap,
		u8 write_len, u8 *write_buf, u8 *read_buf)
{
	struct cyttsp5_i2c *ts = dev_get_drvdata(adap->dev);
	int rc;

	pm_runtime_get_noresume(adap->dev);
	mutex_lock(&ts->lock);

	rc = cyttsp5_i2c_write_read_specific_(adap, write_len, write_buf,
			read_buf);

	mutex_unlock(&ts->lock);
	pm_runtime_put_noidle(adap->dev);

	return rc;
}

/*** ZTEMT Added by luochangyang, 2013/09/03 ***/
#ifdef CONFIG_OF
static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int cyttsp_power_on(struct device *dev)
{
	int rc;
    static struct regulator *vcc_ana;
    static struct regulator *vcc_i2c;

	vcc_ana = regulator_get(dev, "vdd_ana");
	if (IS_ERR(vcc_ana))
    {
		rc = PTR_ERR(vcc_ana);
		dev_err(dev, "Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(vcc_ana) > 0)
    {
		rc = regulator_set_voltage(vcc_ana, 2850000, 2850000);
		if (rc)
        {
			dev_err(dev, "Regulator set ana vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}
    
    rc = reg_set_optimum_mode_check(vcc_ana, 15000);
    if (rc < 0)
    {
        dev_err(dev, "Regulator vcc_ana set_opt failed rc=%d\n", rc);
        return rc;
    }
    
    rc = regulator_enable(vcc_ana);
    if (rc)
    {
        dev_err(dev, "Regulator vcc_ana enable failed rc=%d\n", rc);
        goto error_reg_en_vcc_ana;
    }
    
	vcc_i2c = regulator_get(dev, "vcc_i2c");
	if (IS_ERR(vcc_i2c))
    {
		rc = PTR_ERR(vcc_i2c);
		dev_err(dev, "Regulator get failed rc=%d\n", rc);
		goto error_reg_opt_vcc_dig;
	}
/*    
	if (regulator_count_voltages(vcc_i2c) > 0)
    {
 		rc = regulator_set_voltage(vcc_i2c, 1800000, 1800000);
		if (rc)
        {
			dev_err(dev, "Regulator set i2c vtg failed rc=%d\n", rc);
			goto error_set_vtg_i2c;
		}
	}
    
    rc = reg_set_optimum_mode_check(vcc_i2c, 10000);
    if (rc < 0)
    {
        dev_err(dev, "Regulator vcc_i2c set_opt failed rc=%d\n", rc);
        goto error_set_vtg_i2c;
    }
*/
    rc = regulator_enable(vcc_i2c);
    if (rc)
    {
        dev_err(dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
        goto error_reg_en_vcc_i2c;
    }

    msleep(100);
    
    return 0;

error_reg_en_vcc_i2c:
    reg_set_optimum_mode_check(vcc_i2c, 0);
//error_set_vtg_i2c:
//    regulator_put(vcc_i2c);
error_reg_opt_vcc_dig:
    regulator_disable(vcc_ana);
error_reg_en_vcc_ana:
    reg_set_optimum_mode_check(vcc_ana, 0);
error_set_vtg_vcc_ana:
	regulator_put(vcc_ana);
	return rc;
}

static int cyttsp_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
    int rst_gpio = 0;
    int irq_gpio = 0;

	/* reset, irq gpio info */
	rst_gpio = of_get_named_gpio(np, "cy,reset-gpio", 0);
	irq_gpio = of_get_named_gpio(np, "cy,irq-gpio", 0);
    
    return 0;
}
#endif
/***ZTEMT END***/

static struct cyttsp5_ops ops = {
	.read_default = cyttsp5_i2c_read_default,
	.read_default_nosize = cyttsp5_i2c_read_default_nosize,
	.write_read_specific = cyttsp5_i2c_write_read_specific,
};

static struct of_device_id cyttsp5_i2c_of_match[] = {
	{ .compatible = "cy,cyttsp5_i2c_adapter", },
	{ }
};
MODULE_DEVICE_TABLE(of, cyttsp5_i2c_of_match);

static int cyttsp5_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *i2c_id)
{
	struct cyttsp5_i2c *ts_i2c;
	struct device *dev = &client->dev;
//	const struct of_device_id *match;
	char const *adap_id;
	int rc;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "%s: fail check I2C functionality\n", __func__);
		rc = -EIO;
		goto error_alloc_data_failed;
	}

	ts_i2c = kzalloc(sizeof(struct cyttsp5_i2c), GFP_KERNEL);
	if (ts_i2c == NULL) {
		dev_err(dev, "%s: Error, kzalloc.\n", __func__);
		rc = -ENOMEM;
		goto error_alloc_data_failed;
	}
    
/*** ZTEMT Modify by luochangyang, 2013/09/03 ***/
#if 1
#ifdef CONFIG_OF
    if (client->dev.of_node) {
        rc = cyttsp_parse_dt(&client->dev);
        if (rc)
            return rc;
    } else
        dev_err(dev, "%s: no client->dev.of_node.\n", __func__);
    
    cyttsp_power_on(&client->dev);

    adap_id = dev_get_platdata(dev);
#endif
#else
    match = of_match_device(of_match_ptr(cyttsp5_i2c_of_match), dev);
    if (match) {
        rc = of_property_read_string(dev->of_node, "cy,adapter_id",
                &adap_id);
        if (rc) {
            dev_err(dev, "%s: OF error rc=%d\n", __func__, rc);
            goto error_free_data;
        }
        cyttsp5_devtree_register_devices(dev);
    } else {
        adap_id = dev_get_platdata(dev);
    }
#endif
/***ZTEMT END***/

	mutex_init(&ts_i2c->lock);
	ts_i2c->client = client;
	ts_i2c->id = (adap_id) ? adap_id : CYTTSP5_I2C_NAME;
	client->dev.bus = &i2c_bus_type;
	i2c_set_clientdata(client, ts_i2c);
	dev_set_drvdata(&client->dev, ts_i2c);

	dev_dbg(dev, "%s: add adap='%s' (CYTTSP5_I2C_NAME=%s)\n", __func__,
		ts_i2c->id, CYTTSP5_I2C_NAME);

	pm_runtime_enable(&client->dev);

	rc = cyttsp5_add_adapter(ts_i2c->id, &ops, dev);
	if (rc) {
		dev_err(dev, "%s: Error on probe %s\n", __func__,
			CYTTSP5_I2C_NAME);
		goto add_adapter_err;
	}

	return 0;

add_adapter_err:
	pm_runtime_disable(&client->dev);
	dev_set_drvdata(&client->dev, NULL);
	i2c_set_clientdata(client, NULL);
//error_free_data:
	kfree(ts_i2c);
error_alloc_data_failed:
	return rc;
}

/* registered in driver struct */
static int cyttsp5_i2c_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct cyttsp5_i2c *ts_i2c = dev_get_drvdata(dev);

	cyttsp5_del_adapter(ts_i2c->id);
	pm_runtime_disable(&client->dev);
	dev_set_drvdata(&client->dev, NULL);
	i2c_set_clientdata(client, NULL);
	kfree(ts_i2c);
	return 0;
}

static const struct i2c_device_id cyttsp5_i2c_id[] = {
	{ CYTTSP5_I2C_NAME, 0, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cyttsp5_i2c_id);

static struct i2c_driver cyttsp5_i2c_driver = {
	.driver = {
		.name = CYTTSP5_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = cyttsp5_i2c_of_match,
	},
	.probe = cyttsp5_i2c_probe,
	.remove = __devexit_p(cyttsp5_i2c_remove),
	.id_table = cyttsp5_i2c_id,
};

static int __init cyttsp5_i2c_init(void)
{
	int rc = i2c_add_driver(&cyttsp5_i2c_driver);

	pr_info("%s: Cypress TTSP v5 I2C Driver (Built %s) rc=%d\n",
		 __func__, CY_DRIVER_DATE, rc);
	return rc;
}
module_init(cyttsp5_i2c_init);

static void __exit cyttsp5_i2c_exit(void)
{
	i2c_del_driver(&cyttsp5_i2c_driver);
}
module_exit(cyttsp5_i2c_exit);

MODULE_ALIAS(CYTTSP5_I2C_NAME);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product I2C driver");
MODULE_AUTHOR("Cypress Semiconductor <ttdrivers@cypress.com>");
