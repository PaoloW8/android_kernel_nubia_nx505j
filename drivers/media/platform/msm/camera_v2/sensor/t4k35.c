/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#define T4K35_SENSOR_NAME "t4k35"
DEFINE_MSM_MUTEX(t4k35_mut);
extern void t4k35_otp_init_setting(struct msm_sensor_ctrl_t *s_ctrl);

static struct msm_sensor_ctrl_t t4k35_s_ctrl;

static struct msm_sensor_power_setting t4k35_power_setting[] = {

	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
#if 1 
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
#endif
#if 1
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},
#endif
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info t4k35_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id t4k35_i2c_id[] = {
	{T4K35_SENSOR_NAME, (kernel_ulong_t)&t4k35_s_ctrl},
	{ }
};

static int32_t msm_t4k35_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &t4k35_s_ctrl);
}

static struct i2c_driver t4k35_i2c_driver = {
	.id_table = t4k35_i2c_id,
	.probe  = msm_t4k35_i2c_probe,
	.driver = {
		.name = T4K35_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client t4k35_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id t4k35_dt_match[] = {
	{.compatible = "qcom,t4k35", .data = &t4k35_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, t4k35_dt_match);

static struct platform_driver t4k35_platform_driver = {
	.driver = {
		.name = "qcom,t4k35",
		.owner = THIS_MODULE,
		.of_match_table = t4k35_dt_match,
	},
};

static int32_t t4k35_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(t4k35_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init t4k35_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&t4k35_platform_driver,
		t4k35_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&t4k35_i2c_driver);
}

static void __exit t4k35_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (t4k35_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&t4k35_s_ctrl);
		platform_driver_unregister(&t4k35_platform_driver);
	} else
		i2c_del_driver(&t4k35_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t t4k35_s_ctrl = {
	.sensor_i2c_client = &t4k35_sensor_i2c_client,
	.power_setting_array.power_setting = t4k35_power_setting,
	.power_setting_array.size = ARRAY_SIZE(t4k35_power_setting),
	.zte_read_otp = t4k35_otp_init_setting,
	.msm_sensor_mutex = &t4k35_mut,
	.sensor_v4l2_subdev_info = t4k35_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(t4k35_subdev_info),
};

module_init(t4k35_init_module);
module_exit(t4k35_exit_module);
MODULE_DESCRIPTION("t4k35");
MODULE_LICENSE("GPL v2");
