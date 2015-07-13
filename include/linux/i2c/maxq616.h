#ifndef __MAXQ616_H
#define __MAXQ616_H

#include <linux/types.h>
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
#include <linux/wakelock.h>

#ifdef CONFIG_BOARD_ZTEMT_NX504J
#define HALL_DEVICE_INT_S    67
#else
#define HALL_DEVICE_INT_S    62
#endif
#define HALL_DEVICE_INT_N    68

#define MAGNETIC_DEVICE_NEAR   1  //Near
#define MAGNETIC_DEVICE_FAR    2  //Far

struct akm8789_irq {
    unsigned int irq_num;
    unsigned int irq_pin;
    bool enabled;
};

struct akm8789_wake_lock{
    struct wake_lock lock;
    bool   locked;
    char   *name;
};


struct maxq616_chip {
	struct i2c_client *client;
    struct regulator  *power;
    bool  power_on;
};






#endif /* __MAXQ616_H */
