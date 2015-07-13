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
#include <mach/gpiomux.h>
#include "msm_sensor.h"
#include "msm_sd.h"
#include "camera.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include <mach/rpm-regulator.h>
#include <mach/rpm-regulator-smd.h>
#include <linux/regulator/consumer.h>
#include "../../../../../../video/msm/mdss/mdss_fb.h"
	 extern struct msm_fb_data_type *zte_camera_mfd;

#define IMX135_SENSOR_NAME "imx135_gbao_lc898122"
DEFINE_MSM_MUTEX(imx135_mut);
#ifdef CONFIG_OIS_DEBUG
#define CDBG_OIS(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG_OIS(fmt, args...) do { } while (0)
#endif


extern void read_ois_byte_data_lc898122(unsigned short reg_addr, unsigned char *read_data_8);
extern void read_ois_word_data_lc898122(unsigned short reg_addr, uint16_t *read_data_16);

extern void	SetH1cMod_lc898122( unsigned char	UcSetNum );
extern void RegReadA_lc898122(unsigned short reg_addr, unsigned char *read_data_8);
extern void RegWriteA_lc898122(unsigned short reg_addr, unsigned char write_data_8);
extern void RamReadA_lc898122(unsigned short ram_addr, void *read_data_16);
extern void RamWriteA_lc898122(unsigned short ram_addr, unsigned short write_data_16);
extern void RamRead32A_lc898122(unsigned short ram_addr, void *read_data_32);
extern void RamWrite32A_lc898122(unsigned short ram_addr, unsigned long write_data_32);

extern unsigned char RtnCen_lc898122(unsigned char	UcCmdPar);
extern void SetPanTiltMode_lc898122(unsigned char UcPnTmod);

extern void OisEna_lc898122(void);
extern void	IniSet_lc898122( void );
extern void	SrvCon_lc898122( unsigned char	UcDirSel, unsigned char	UcSwcCon );
extern void	S2cPro_lc898122( unsigned char uc_mode );
extern void msm_ois_init_cci_lc898122(void);
extern void msm_ois_release_cci_lc898122(void);

extern void RamAccFixMod_lc898122( unsigned char UcAccMod );
extern void IniSetAf_lc898122( void );
extern void	SetH1cMod_lc898122( unsigned char	UcSetNum );
                   
unsigned char read_otp_ready_flag_lc898122=0;

/* 16bits RAM */
unsigned short  hall_offset_x_lc898122=0; 			   
unsigned short  hall_offset_y_lc898122=0; 
unsigned short  hall_bias_x_lc898122=0; 
unsigned short  hall_bias_y_lc898122=0; 
unsigned short  hall_ad_offset_x_lc898122=0; 
unsigned short  hall_ad_offset_y_lc898122=0; 
unsigned short  loop_gain_x_lc898122=0; 
unsigned short  loop_gain_y_lc898122=0; 

/* 8bits Register */
unsigned char gyro_offset_x_msb_lc898122=0;
unsigned char gyro_offset_x_lsb_lc898122=0;			   
unsigned char gyro_offset_y_msb_lc898122=0;
unsigned char gyro_offset_y_lsb_lc898122=0;

/* 32bits RAM */ 
unsigned long  gyro_gain_x_lc898122=0;
unsigned char  gyro_gain_x_31_24_lc898122=0;
unsigned char  gyro_gain_x_23_16_lc898122=0;
unsigned char  gyro_gain_x_15_8_lc898122=0;
unsigned char  gyro_gain_x_7_0_lc898122=0;
unsigned long  gyro_gain_y_lc898122=0;
unsigned char  gyro_gain_y_31_24_lc898122=0;
unsigned char  gyro_gain_y_23_16_lc898122=0;
unsigned char  gyro_gain_y_15_8_lc898122=0;
unsigned char  gyro_gain_y_7_0_lc898122=0;

/* 8bits Register */
unsigned char osc_value_lc898122=1;

#define OTP_PAGE_ADDR_LC898122			0x3B02
#define	OTP_READ_MODE_ADDR_LC898122		0x3B00
#define	OTP_READ_READY_ADDR_LC898122		0x3B01
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---Start*/
#define AF_START_CURRENT_LC898122        0x20
#define AF_START_INFINITY_LC898122       0x22
#define AF_START_MACRO_LC898122          0x24
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---End*/


#define HALL_OFFSET_X_ADDR_LC898122		0x30
#define HALL_OFFSET_Y_ADDR_LC898122		0x32
#define HALL_BIAS_X_ADDR_LC898122		0x34
#define HALL_BIAS_Y_ADDR_LC898122		0x36
#define HALL_AD_OFFSET_X_ADDR_LC898122	0x38
#define HALL_AD_OFFSET_Y_ADDR_LC898122	0x3A
#define LOOP_GAIN_X_ADDR_LC898122		0x3C
#define LOOP_GAIN_Y_ADDR_LC898122		0x3E

#define	GYRO_OFFSET_X_MSB_ADDR_LC898122	0x44
#define	GYRO_OFFSET_X_LSB_ADDR_LC898122	0x45
#define	GYRO_OFFSET_Y_MSB_ADDR_LC898122	0x46
#define	GYRO_OFFSET_Y_LSB_ADDR_LC898122	0x47

#define GYRO_GAIN_X_31_24_ADDR_LC898122	0x49
#define GYRO_GAIN_X_23_16_ADDR_LC898122	0x4A
#define GYRO_GAIN_X_15_8_ADDR_LC898122	0x4B
#define GYRO_GAIN_X_7_0_ADDR_LC898122	0x4C
#define GYRO_GAIN_Y_31_24_ADDR_LC898122	0x4D
#define GYRO_GAIN_Y_23_16_ADDR_LC898122	0x4E
#define GYRO_GAIN_Y_15_8_ADDR_LC898122	0x4F
#define GYRO_GAIN_Y_7_0_ADDR_LC898122	0x50
#define OSC_VALUE_ADDR_LC898122			0x48

extern unsigned short af_start_value_lc898122;
extern unsigned short af_infinity_value_lc898122;
extern unsigned short af_macro_value_lc898122 ;
extern unsigned short af_start_value_lc898122_sharp;
extern unsigned short af_infinity_value_lc898122_sharp;
extern unsigned short af_macro_value_lc898122_sharp ;

static void imx135_ois_otp_lc898122(struct work_struct *work)
{
	struct msm_sensor_ctrl_t *s_ctrl = container_of(to_delayed_work(work),
					struct msm_sensor_ctrl_t, zte_otp_worker);
	//unsigned char temp_test;
	static int ois_otp_lc898122_flag = 0;

	mutex_lock(&s_ctrl->zte_otp_mutex);
	msm_ois_init_cci_lc898122();
	
	IniSetAf_lc898122();
	IniSet_lc898122();

	if (ois_otp_lc898122_flag == 0)
	{
		read_ois_word_data_lc898122(HALL_OFFSET_X_ADDR_LC898122,(uint16_t *)&hall_offset_x_lc898122);
	       CDBG_OIS("hall_offset_x = 0x%x\n",hall_offset_x_lc898122);
		read_ois_word_data_lc898122(HALL_OFFSET_Y_ADDR_LC898122,(uint16_t *)&hall_offset_y_lc898122);
	       CDBG_OIS("hall_offset_y = 0x%x\n",hall_offset_y_lc898122);
		read_ois_word_data_lc898122(HALL_BIAS_X_ADDR_LC898122,(uint16_t *)&hall_bias_x_lc898122);
	       CDBG_OIS("hall_bias_x = 0x%x\n",hall_bias_x_lc898122);
		read_ois_word_data_lc898122(HALL_BIAS_Y_ADDR_LC898122,(uint16_t *)&hall_bias_y_lc898122);
	       CDBG_OIS("hall_bias_y = 0x%x\n",hall_bias_y_lc898122);
		read_ois_word_data_lc898122(HALL_AD_OFFSET_X_ADDR_LC898122,(uint16_t *)&hall_ad_offset_x_lc898122);
	       CDBG_OIS("hall_ad_offset_x = 0x%x\n",hall_ad_offset_x_lc898122);
		read_ois_word_data_lc898122(HALL_AD_OFFSET_Y_ADDR_LC898122,(uint16_t *)&hall_ad_offset_y_lc898122);
	       CDBG_OIS("hall_ad_offset_y = 0x%x\n",hall_ad_offset_y_lc898122);
		read_ois_word_data_lc898122(LOOP_GAIN_X_ADDR_LC898122,(uint16_t *)&loop_gain_x_lc898122);
	       CDBG_OIS("loop_gain_x = 0x%x\n",loop_gain_x_lc898122);
		read_ois_word_data_lc898122(LOOP_GAIN_Y_ADDR_LC898122,(uint16_t *)&loop_gain_y_lc898122);
		CDBG_OIS("loop_gain_y = 0x%x\n",loop_gain_y_lc898122);

		
	       read_ois_byte_data_lc898122(GYRO_OFFSET_X_MSB_ADDR_LC898122,(unsigned char *)&gyro_offset_x_msb_lc898122);
	       CDBG_OIS("gyro_offset_x_msb = 0x%x\n",gyro_offset_x_msb_lc898122);
		read_ois_byte_data_lc898122(GYRO_OFFSET_X_LSB_ADDR_LC898122,(unsigned char *)&gyro_offset_x_lsb_lc898122);
	       CDBG_OIS("gyro_offset_x_lsb = 0x%x\n",gyro_offset_x_lsb_lc898122);
		read_ois_byte_data_lc898122(GYRO_OFFSET_Y_MSB_ADDR_LC898122,(unsigned char *)&gyro_offset_y_msb_lc898122);
	       CDBG_OIS("gyro_offset_y_msb = 0x%x\n",gyro_offset_y_msb_lc898122);
		read_ois_byte_data_lc898122(GYRO_OFFSET_Y_LSB_ADDR_LC898122,(unsigned char *)&gyro_offset_y_lsb_lc898122);
	       CDBG_OIS("gyro_offset_y_lsb = 0x%x\n",gyro_offset_y_lsb_lc898122);
		read_ois_byte_data_lc898122(OSC_VALUE_ADDR_LC898122,(unsigned char *)&osc_value_lc898122);
	       CDBG_OIS("osc_value = 0x%x\n",osc_value_lc898122);
		read_ois_byte_data_lc898122(GYRO_GAIN_X_31_24_ADDR_LC898122,(unsigned char *)&gyro_gain_x_31_24_lc898122);
	       CDBG_OIS("gyro_gain_x_31_24 = 0x%x\n",gyro_gain_x_31_24_lc898122);
		read_ois_byte_data_lc898122(GYRO_GAIN_X_23_16_ADDR_LC898122,(unsigned char *)&gyro_gain_x_23_16_lc898122);
	       CDBG_OIS("gyro_gain_x_23_16 = 0x%x\n",gyro_gain_x_23_16_lc898122);
		read_ois_byte_data_lc898122(GYRO_GAIN_X_15_8_ADDR_LC898122,(unsigned char *)&gyro_gain_x_15_8_lc898122);
	       CDBG_OIS("gyro_gain_x_15_8 = 0x%x\n",gyro_gain_x_15_8_lc898122);
		read_ois_byte_data_lc898122(GYRO_GAIN_X_7_0_ADDR_LC898122,(unsigned char *)&gyro_gain_x_7_0_lc898122);
	       CDBG_OIS("gyro_gain_x_7_0 = 0x%x\n",gyro_gain_x_7_0_lc898122);

		gyro_gain_x_lc898122 = (gyro_gain_x_31_24_lc898122 <<24) | (gyro_gain_x_23_16_lc898122 <<16)|(gyro_gain_x_15_8_lc898122<<8)|gyro_gain_x_7_0_lc898122;

		
		read_ois_byte_data_lc898122(GYRO_GAIN_Y_31_24_ADDR_LC898122,(unsigned char *)&gyro_gain_y_31_24_lc898122);
	       CDBG_OIS("gyro_gain_y_31_24 = 0x%x\n",gyro_gain_y_31_24_lc898122);
		read_ois_byte_data_lc898122(GYRO_GAIN_Y_23_16_ADDR_LC898122,(unsigned char *)&gyro_gain_y_23_16_lc898122);
	       CDBG_OIS("gyro_gain_y_23_16 = 0x%x\n",gyro_gain_y_23_16_lc898122);
		read_ois_byte_data_lc898122(GYRO_GAIN_Y_15_8_ADDR_LC898122,(unsigned char *)&gyro_gain_y_15_8_lc898122);
	       CDBG_OIS("gyro_gain_y_15_8 = 0x%x\n",gyro_gain_y_15_8_lc898122);
		read_ois_byte_data_lc898122(GYRO_GAIN_Y_7_0_ADDR_LC898122,(unsigned char *)&gyro_gain_y_7_0_lc898122);
		CDBG_OIS("gyro_gain_y_7_0 = 0x%x\n",gyro_gain_y_7_0_lc898122);
			
		gyro_gain_y_lc898122 = (gyro_gain_y_31_24_lc898122 <<24) | (gyro_gain_y_23_16_lc898122 <<16)|(gyro_gain_y_15_8_lc898122<<8)|gyro_gain_y_7_0_lc898122;

		ois_otp_lc898122_flag = 1;
		
	}
	
	RamAccFixMod_lc898122(0x01);
	
	RamWriteA_lc898122(0x1479 ,hall_offset_x_lc898122 );
	RamWriteA_lc898122(0x14F9 ,hall_offset_y_lc898122);
	RamWriteA_lc898122(0x147A ,hall_bias_x_lc898122 );
	RamWriteA_lc898122(0x14FA ,hall_bias_y_lc898122 );
	RamWriteA_lc898122(0x1450 ,hall_ad_offset_x_lc898122 );
	RamWriteA_lc898122(0x14D0 ,hall_ad_offset_y_lc898122 );
	RamWriteA_lc898122(0x10D3 ,loop_gain_x_lc898122 );
	RamWriteA_lc898122(0x11D3 ,loop_gain_y_lc898122 );

	RamAccFixMod_lc898122(0x00);
	
	RegWriteA_lc898122(0x02A0,gyro_offset_x_msb_lc898122);
	RegWriteA_lc898122(0x02A1,gyro_offset_x_lsb_lc898122);
	RegWriteA_lc898122(0x02A2,gyro_offset_y_msb_lc898122);
	RegWriteA_lc898122(0x02A3,gyro_offset_y_lsb_lc898122);
	
	RamWrite32A_lc898122(0x1020,gyro_gain_x_lc898122);
	RamWrite32A_lc898122(0x1120,gyro_gain_y_lc898122);
	
	RegWriteA_lc898122(0x0257,osc_value_lc898122);

       RamWriteA_lc898122(0x0304 ,af_infinity_value_lc898122 );
	   
	RtnCen_lc898122(0x00);
	SetPanTiltMode_lc898122(1);
	OisEna_lc898122();
	RegWriteA_lc898122(0x257,0x2c);
	//RegReadA_lc898122(0x257, &temp_test);
	//printk("ssss temp_test=%x\n", temp_test);
	
	mutex_unlock(&s_ctrl->zte_otp_mutex);
}

static struct msm_sensor_ctrl_t imx135_s_ctrl;

static struct msm_sensor_power_setting imx135_power_setting[] = {

#if defined(CONFIG_ZTE_CAMERA_Z7)
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VAF,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},

	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VAF,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
#endif

	{
		.seq_type = SENSOR_VREG,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = 1,
		.config_val = 0,
		.delay = 0,
	},
#if 0
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 0,
	},
#endif
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
	{
 		.seq_type = SENSOR_GPIO,
 		.seq_val = SENSOR_GPIO_STANDBY,
 		.config_val = GPIO_OUT_HIGH,
 		.delay = 5,
 	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
 		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
 		.delay = 5,
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

static struct v4l2_subdev_info imx135_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id imx135_i2c_id[] = {
	{IMX135_SENSOR_NAME, (kernel_ulong_t)&imx135_s_ctrl},
	{ }
};

static int32_t msm_imx135_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &imx135_s_ctrl);
}

static struct i2c_driver imx135_i2c_driver = {
	.id_table = imx135_i2c_id,
	.probe  = msm_imx135_i2c_probe,
	.driver = {
		.name = IMX135_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx135_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id imx135_dt_match[] = {
	{.compatible = "qcom,imx135_gbao_lc898122", .data = &imx135_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, imx135_dt_match);

static struct platform_driver imx135_platform_driver = {
	.driver = {
		.name = "qcom,imx135_gbao_lc898122",
		.owner = THIS_MODULE,
		.of_match_table = imx135_dt_match,
	},
};

static int32_t imx135_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(imx135_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init imx135_gbao_lc898122_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	
	rc = platform_driver_probe(&imx135_platform_driver,
		imx135_platform_probe);
	
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&imx135_i2c_driver);
}

static void __exit imx135_gbao_lc898122_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (imx135_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx135_s_ctrl);
		platform_driver_unregister(&imx135_platform_driver);
	} else
		i2c_del_driver(&imx135_i2c_driver);
	return;
}

static int zte_adaptive_imx135_gbao_lc898122(struct msm_sensor_ctrl_t* s_ctrl)
{
	enum msm_camera_i2c_reg_addr_type temp_i2c_addr_type; 
	int rc = 0;
	uint16_t ois_version_flag_lc898122 =0; 
	s_ctrl->sensor_i2c_client->cci_client->sid = 0x48 >> 1;
	temp_i2c_addr_type = s_ctrl->sensor_i2c_client->addr_type;
	s_ctrl->sensor_i2c_client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
						 s_ctrl->sensor_i2c_client,0x027E,
				&ois_version_flag_lc898122, MSM_CAMERA_I2C_BYTE_DATA);
	
	printk("ois_version_flag_lc898122 = 0x%x\n",ois_version_flag_lc898122);
	
	if (rc < 0) {
			pr_err("%s failed\n", __func__);
	}
	if ( ois_version_flag_lc898122 != 0x93) {
		rc = -1;
		pr_err("%s	ois_version_flag_lc898122 0x93 cci_init failed,rc =%d\n", __func__,rc);
	}
	s_ctrl->sensor_i2c_client->cci_client->sid = s_ctrl->sensordata->slave_info->sensor_slave_addr >> 1;
	s_ctrl->sensor_i2c_client->addr_type = temp_i2c_addr_type;
	return rc;
}

static void zte_read_otp_imx135_gbao_lc898122(struct msm_sensor_ctrl_t* s_ctrl)
{
	enum msm_camera_i2c_reg_addr_type temp_i2c_data_type; 
	static int ois_init_flag_up_lc898122=0;
	int rc;
    if (ois_init_flag_up_lc898122==0) {
	   	ois_init_flag_up_lc898122=1;
		/*ZTEMT: Add for Read AF OTP  ---Start*/
	    s_ctrl->sensor_i2c_client->cci_client->sid = 0xA0 >> 1;
		temp_i2c_data_type = s_ctrl->sensor_i2c_client->addr_type;
		s_ctrl->sensor_i2c_client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		                     s_ctrl->sensor_i2c_client,AF_START_CURRENT_LC898122,
					(uint16_t *)&af_start_value_lc898122, MSM_CAMERA_I2C_WORD_DATA);
		
	    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		                     s_ctrl->sensor_i2c_client,AF_START_INFINITY_LC898122,
					(uint16_t *)&af_infinity_value_lc898122, MSM_CAMERA_I2C_WORD_DATA);
			
	    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		                     s_ctrl->sensor_i2c_client,AF_START_MACRO_LC898122,
					(uint16_t *)&af_macro_value_lc898122, MSM_CAMERA_I2C_WORD_DATA);
				   
		s_ctrl->sensor_i2c_client->cci_client->sid = s_ctrl->sensordata->slave_info->sensor_slave_addr >> 1;

		s_ctrl->sensor_i2c_client->addr_type = temp_i2c_data_type;
		/*ZTEMT: Add for Read AF OTP  ---End*/
	}

}
static unsigned long otp_duration = HZ/1000;
static void zte_workquene_schedule_imx135_gbao_lc898122(struct msm_sensor_ctrl_t* s_ctrl)
{
	static int once = 1;
	if (once == 1) {
		once = 0;
		return;
	}
	schedule_delayed_work(&s_ctrl->zte_otp_worker,
			otp_duration);
}
static void zte_power_down(struct msm_sensor_ctrl_t* s_ctrl)
{
	static int ois_init_flag_down=0;

	if (ois_init_flag_down==0)
		ois_init_flag_down=1;
	else {
		mutex_lock(&s_ctrl->zte_otp_mutex);
		RtnCen_lc898122(0x00);
		SrvCon_lc898122(0x00,0);  
		SrvCon_lc898122(0x01,0);  
		RegWriteA_lc898122(0x0304,0x00);
		RegWriteA_lc898122(0x0305,0x00);
		msm_ois_release_cci_lc898122();
		mutex_unlock(&s_ctrl->zte_otp_mutex);	
	}
}
static void zte_workquene_cancel_imx135_gbao_lc898122(struct msm_sensor_ctrl_t* s_ctrl)
{
	static int once = 1;
	if (once == 1) {
		once = 0;
		return;
	}
	cancel_delayed_work_sync(&s_ctrl->zte_otp_worker);
}

static void zte_workquene_init_imx135_gbao_lc898122(struct msm_sensor_ctrl_t* s_ctrl)
{	     
	INIT_DELAYED_WORK(&s_ctrl->zte_otp_worker, imx135_ois_otp_lc898122);
	mutex_init(&s_ctrl->zte_otp_mutex);
}
static void zte_control_ois_imx135_gbao_lc898122(struct msm_sensor_ctrl_t* s_ctrl, int enable)
{
	if (enable) {
		mutex_lock(&s_ctrl->zte_otp_mutex);
		OisEna_lc898122();		
		mutex_unlock(&s_ctrl->zte_otp_mutex);
	} else {
		mutex_lock(&s_ctrl->zte_otp_mutex);
		RtnCen_lc898122(0x00);		
		mutex_unlock(&s_ctrl->zte_otp_mutex);
	}
}

static struct msm_sensor_ctrl_t imx135_s_ctrl = {
	.sensor_i2c_client = &imx135_sensor_i2c_client,
	.power_setting_array.power_setting = imx135_power_setting,
	.power_setting_array.size = ARRAY_SIZE(imx135_power_setting),
	//added by congshan start
	.zte_workquene_init = zte_workquene_init_imx135_gbao_lc898122,
	.zte_adaptive_sensor = zte_adaptive_imx135_gbao_lc898122,
	.zte_read_otp = zte_read_otp_imx135_gbao_lc898122,
	.zte_workquene_schedule = zte_workquene_schedule_imx135_gbao_lc898122,
	.zte_power_down = zte_power_down,
	.zte_workquene_cancel = zte_workquene_cancel_imx135_gbao_lc898122,
	.zte_control_ois = zte_control_ois_imx135_gbao_lc898122,
	//added by congshan end
	.msm_sensor_mutex = &imx135_mut,
	.sensor_v4l2_subdev_info = imx135_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx135_subdev_info),
};

module_init(imx135_gbao_lc898122_init_module);
module_exit(imx135_gbao_lc898122_exit_module);
MODULE_DESCRIPTION("imx135");
MODULE_LICENSE("GPL v2");
