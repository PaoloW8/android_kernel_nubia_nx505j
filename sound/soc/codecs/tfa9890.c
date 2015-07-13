#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

int reset_gpio = -1;
int intr_gpio = -1;

static ssize_t tfa9890_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = -1;
    if(reset_gpio > 0)
        ret = gpio_get_value(reset_gpio);
    pr_info("tfa reset_gpio status ...  %d\n",ret);
    return snprintf(buf, PAGE_SIZE, "%d\n",ret) ;
}

static ssize_t tfa9890_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    bool value;
    if (strtobool(buf, &value))
        return -EINVAL;
    if(reset_gpio >0)
        gpio_set_value(reset_gpio,((value > 0) ? 1: 0));
    pr_debug("Reset the tfa9890.... %d\n",value);
    return size;

}

static struct device_attribute attrs_tfa9890_device[] = {
    __ATTR(reset_gpio,0640,tfa9890_enable_show,tfa9890_enable_store),
};
static int __devinit tfa9890_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
    struct device_node *np = i2c->dev.of_node;
    reset_gpio =  of_get_named_gpio(np, "tfa9890-reset-gpio", 0);
    intr_gpio =  of_get_named_gpio(np, "tfa9890-intr-gpio", 0);
    if(reset_gpio > 0)
    {
        gpio_set_value(reset_gpio,0);
        printk("reset_gpio ==== is %d\n",reset_gpio);
        printk("%s... getvalue %d\n",__func__,gpio_get_value(reset_gpio));
    }
    device_create_file(&i2c->dev, attrs_tfa9890_device);
	return 0;
}

static int __devexit tfa9890_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id tfa9890_i2c_id[] = {
	{ "nxp,tfa9890", 0},
	{ },
};

static const struct of_device_id of_tfa_device_idtable[] = {
    { .compatible = "nxp,tfa9890",0},
	{ },
};

/* corgi i2c codec control layer */
static struct i2c_driver tfa9890_i2c_driver = {
	.driver = {
		.name = "tfa9890",
        .of_match_table = of_tfa_device_idtable,
		.owner = THIS_MODULE,
	},
	.probe = tfa9890_i2c_probe,
	.remove = __devexit_p(tfa9890_i2c_remove),
	.id_table = tfa9890_i2c_id,
};


static int __init tfa9890_modinit(void)
{
	int ret = 0;

    pr_info("%s enter \n",__func__);
	ret = i2c_add_driver(&tfa9890_i2c_driver);
    return ret;
    
}
module_init(tfa9890_modinit);

static void __exit tfa9890_exit(void)
{
	i2c_del_driver(&tfa9890_i2c_driver);
}
module_exit(tfa9890_exit);

MODULE_DESCRIPTION("NXP tfa9890 I2C driver");
MODULE_AUTHOR("wuzehui");
MODULE_LICENSE("GPL");
