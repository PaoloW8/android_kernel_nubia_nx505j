
#ifndef __SENSOR_COMMON_H_
#define __SENSOR_COMMON_H_


struct sensor_common_data {
    struct i2c_client *client;
    struct device *sensor_compass_dev;
    struct device *sensor_temp_humidity_dev;

    int compass_irq_gpio;

};

#define COMPASS_INT_PIN     64

#define PATH_RAW_TEMP           "/sys/class/temperature/temperature/temperature_data"
#define PATH_RAW_HUMIDITY       "/sys/class/humidity/humidity/humidity_data"
#define PATH_TEMP_CPU_0         "/sys/class/thermal/thermal_zone5/temp"
#define PATH_TEMP_CPU_1         "/sys/class/thermal/thermal_zone6/temp"
#define PATH_TEMP_CPU_2         "/sys/class/thermal/thermal_zone7/temp"
#define PATH_TEMP_CPU_3         "/sys/class/thermal/thermal_zone8/temp"
#define PATH_TEMP_RF            "/sys/class/thermal/thermal_zone13/temp"
#define PATH_TEMP_BATTERY       "/sys/class/power_supply/battery/temp"
#define PATH_TEMP_CHARGING      "/sys/class/thermal/thermal_zone2/temp"
#define PATH_TEMP_LCD           "/sys/class/thermal/thermal_zone3/temp"
#define PATH_LCD_BRIGHTNESS     "/sys/class/leds/lcd-backlight/brightness"
#define PATH_BATTERY_LEVEL      "/sys/class/power_supply/battery/capacity"
#define PATH_BATTERY_STATE      "/sys/class/power_supply/battery/voltage_now"
#define PATH_CHARGE_CURRENT     "/sys/class/power_supply/battery/current_now"
#define PATH_CHARGE_STATE       "/sys/class/power_supply/battery/status"
#define PATH_RF_STATE           "/sys/class/"


static int __devinit sensor_common_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit sensor_common_remove(struct i2c_client *client);
#endif
