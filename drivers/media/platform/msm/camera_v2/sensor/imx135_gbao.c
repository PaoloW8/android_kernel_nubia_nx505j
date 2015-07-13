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


#define IMX135_SENSOR_NAME "imx135_gbao"
DEFINE_MSM_MUTEX(imx135_mut);
//#define CONFIG_OIS_DEBUG
#define CONFIG_OIS_DEBUG

#ifdef CONFIG_OIS_DEBUG
#define CDBG_OIS(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG_OIS(fmt, args...) do { } while (0)
#endif

extern void	SetH1cMod( unsigned char	UcSetNum );
extern void RegReadA(unsigned short reg_addr, unsigned char *read_data_8);
extern void RegWriteA(unsigned short reg_addr, unsigned char write_data_8);
extern void RamReadA(unsigned short ram_addr, void *read_data_16);
extern void RamWriteA(unsigned short ram_addr, unsigned short write_data_16);
extern void RamRead32A(unsigned short ram_addr, void *read_data_32);
extern void RamWrite32A(unsigned short ram_addr, unsigned long write_data_32);

extern unsigned char RtnCen(unsigned char	UcCmdPar);
extern void SetPanTiltMode(unsigned char UcPnTmod);

extern void OisEna(void);
extern void	IniSet( void );
extern void	SrvCon( unsigned char	UcDirSel, unsigned char	UcSwcCon );
extern void	S2cPro( unsigned char uc_mode );
extern void msm_ois_init_cci(void);
extern void msm_ois_release_cci(void);
                   
uint16_t read_otp_ready_flag=0;

/* 16bits RAM */
unsigned short  hall_offset_x=0; 			   
unsigned short  hall_offset_y=0; 
unsigned short  hall_bias_x=0; 
unsigned short  hall_bias_y=0; 
unsigned short  hall_ad_offset_x=0; 
unsigned short  hall_ad_offset_y=0; 
unsigned short  loop_gain_x=0; 
unsigned short  loop_gain_y=0; 

/* 8bits Register */
uint16_t gyro_offset_x_msb=0;
uint16_t gyro_offset_x_lsb=0;			   
uint16_t gyro_offset_y_msb=0;
uint16_t gyro_offset_y_lsb=0;

/* 32bits RAM */ 
unsigned long  gyro_gain_x=0;
uint16_t  gyro_gain_x_31_24=0;
uint16_t  gyro_gain_x_23_16=0;
uint16_t  gyro_gain_x_15_8=0;
unsigned char  gyro_gain_x_7_0=0;
unsigned long  gyro_gain_y=0;
uint16_t  gyro_gain_y_31_24=0;
uint16_t  gyro_gain_y_23_16=0;
uint16_t  gyro_gain_y_15_8=0;
uint16_t  gyro_gain_y_7_0=0;

/* 8bits Register */
uint16_t osc_value=1;
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---Start*/
unsigned short af_start_value = 0;
unsigned short af_infinity_value = 0;
unsigned short af_macro_value = 0;
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---End*/

#define OTP_PAGE_ADDR			0x3B02
#define	OTP_READ_MODE_ADDR		0x3B00
#define	OTP_READ_READY_ADDR		0x3B01
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---Start*/
#define AF_START_CURRENT        0x3B04
#define AF_START_INFINITY       0x3B06
#define AF_START_MACRO          0X3B08
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---End*/

#define HALL_OFFSET_X_ADDR		0x3B14
#define HALL_OFFSET_Y_ADDR		0x3B16
#define HALL_BIAS_X_ADDR		0x3B18
#define HALL_BIAS_Y_ADDR		0x3B1A
#define HALL_AD_OFFSET_X_ADDR	0x3B1C
#define HALL_AD_OFFSET_Y_ADDR	0x3B1E
#define LOOP_GAIN_X_ADDR		0x3B20
#define LOOP_GAIN_Y_ADDR		0x3B22

#define	GYRO_OFFSET_X_MSB_ADDR	0x3B28
#define	GYRO_OFFSET_X_LSB_ADDR	0x3B29
#define	GYRO_OFFSET_Y_MSB_ADDR	0x3B2A
#define	GYRO_OFFSET_Y_LSB_ADDR	0x3B2B

#define GYRO_GAIN_X_31_24_ADDR	0x3B34
#define GYRO_GAIN_X_23_16_ADDR	0x3B35
#define GYRO_GAIN_X_15_8_ADDR	0x3B36
#define GYRO_GAIN_X_7_0_ADDR	0x3B37
#define GYRO_GAIN_Y_31_24_ADDR	0x3B38
#define GYRO_GAIN_Y_23_16_ADDR	0x3B39
#define GYRO_GAIN_Y_15_8_ADDR	0x3B3A
#define GYRO_GAIN_Y_7_0_ADDR	0x3B3B

#define OSC_VALUE_ADDR			0x3B2C

//add code for ois version D,just need for old module
#if 0
unsigned short  read_loop_gain_x=0;
unsigned short  read_loop_gain_y=0;
unsigned long  read_gyro_gain_x =0;
unsigned long  read_loop_gain_x_multipy =0;
unsigned long  read_loop_gain_y_multipy =0;

unsigned short  temp_loop_gain_x=0;
unsigned short  temp_loop_gain_y=0;

unsigned short UsRltVal1=0;
unsigned short UsRltVal2=0;
#endif
//end code for ois version D,just need for old module
static void imx135_ois_otp(struct work_struct *work)
{
	struct msm_sensor_ctrl_t *s_ctrl = container_of(to_delayed_work(work),
					struct msm_sensor_ctrl_t, zte_otp_worker);
	mutex_lock(&s_ctrl->zte_otp_mutex);
	msm_ois_init_cci();
	//printk("sss e\n");
	IniSet();
	//printk("sss x\n");
	
	RamWriteA(0x1114 ,hall_offset_x );
	RamWriteA(0x1116 ,hall_offset_y);
	RamWriteA(0x1115 ,hall_bias_x );
	RamWriteA(0x1117 ,hall_bias_y );
	RamWriteA(0x1102 ,hall_ad_offset_x );
	RamWriteA(0x1105 ,hall_ad_offset_y );
	RamWriteA(0x132A ,loop_gain_x );
	RamWriteA(0x136A ,loop_gain_y );
	RegWriteA(0x03A0,gyro_offset_x_msb);
	RegWriteA(0x03A1,gyro_offset_x_lsb);
	RegWriteA(0x03A2,gyro_offset_y_msb);
	RegWriteA(0x03A3,gyro_offset_y_lsb);
	RamWrite32A(0x1828,gyro_gain_x);
	RamWrite32A(0x1928,gyro_gain_y);
	RegWriteA(0x0264,osc_value);
	//add code for ois version D,just need for old module
#if 0
	RamReadA(0x132A ,&read_loop_gain_x );
	RamReadA(0x136A ,&read_loop_gain_y );
	read_loop_gain_x_multipy = read_loop_gain_x;
	read_loop_gain_x_multipy = read_loop_gain_x_multipy *56 / 100;   
	read_loop_gain_y_multipy = read_loop_gain_y;
	read_loop_gain_y_multipy = read_loop_gain_y_multipy  *56 / 100;
	temp_loop_gain_x= read_loop_gain_x_multipy & 0xffff;
	temp_loop_gain_y= read_loop_gain_y_multipy & 0xffff;
	RamWriteA(0x132A ,temp_loop_gain_x );
	RamWriteA(0x136A ,temp_loop_gain_y );
	RegWriteA(0x011A,0x01);
	RamReadA(0x1828, &UsRltVal1);
	RamReadA(0x1928, &UsRltVal2);
	RegWriteA(0x011A,0x00);
	if (UsRltVal1 > 0x5998 || UsRltVal1 <0x3332)
		RamWrite32A(0x1828 ,0x3f0ccccd);
	if (UsRltVal2 > 0xcccd || UsRltVal2 <0xa667)
		RamWrite32A(0x1928 ,0xbf0ccccd);
#endif
 //end code for ois version D,just need for old module
	RtnCen(0x00);
	SetPanTiltMode(1);
	OisEna();
	//SetH1cMod(0xff);
	mutex_unlock(&s_ctrl->zte_otp_mutex);
}

static struct msm_sensor_ctrl_t imx135_s_ctrl;

static struct msm_sensor_power_setting imx135_power_setting[] = {
#ifdef CONFIG_IMX135_Z5S
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,    // This VREG is DRV_AVDD, it can enbale the IMX135's AVDD
		.config_val = 0,
		.delay = 0,
	},
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
 		.delay = 5,
	},
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

#else
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
#endif
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
#ifdef CONFIG_IMX135_Z5S
	{.compatible = "qcom,imx135_z5s", .data = &imx135_s_ctrl},
#else
	{.compatible = "qcom,imx135_gbao", .data = &imx135_s_ctrl},
#endif
	{}
};

MODULE_DEVICE_TABLE(of, imx135_dt_match);

static struct platform_driver imx135_platform_driver = {
	.driver = {
		.name = "qcom,imx135_gbao",
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

static int __init imx135_gbao_init_module(void)
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

static void __exit imx135_gbao_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (imx135_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx135_s_ctrl);
		platform_driver_unregister(&imx135_platform_driver);
	} else
		i2c_del_driver(&imx135_i2c_driver);
	return;
}
static int RegRead8byte_adaptive(uint16_t reg_addr, struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t data[8];
	int32_t rc=0;
	enum msm_camera_i2c_reg_addr_type addr_type;
	addr_type = s_ctrl->sensor_i2c_client->addr_type;
	memset(data, 0x00, 8);
	s_ctrl->sensor_i2c_client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;

	rc =  s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq
	 (
		s_ctrl->sensor_i2c_client,
		reg_addr, &data[0],
		8);
	if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	}
	s_ctrl->sensor_i2c_client->addr_type = addr_type;
	//CDBG("sss %x %x %x %x\n", data[0],data[1],data[2],data[3]);
	return rc;
}

static int zte_adaptive_imx135_gbao(struct msm_sensor_ctrl_t* s_ctrl)
{	
	int rc;
	s_ctrl->sensor_i2c_client->cci_client->sid = 0x1c >> 1;
	rc = RegRead8byte_adaptive(0x02, s_ctrl);
	if (rc < 0){
		pr_err("%s rc=%d\n", __func__, rc);
	}
	s_ctrl->sensor_i2c_client->cci_client->sid = s_ctrl->sensordata->slave_info->sensor_slave_addr >> 1;
	return rc;
}

static void zte_read_otp_imx135_gbao(struct msm_sensor_ctrl_t* s_ctrl)
{
	static int ois_init_flag_up=0;
	int page_number = 14;
	int count = 0;
	int rc;
	if (ois_init_flag_up==0) {
	   	ois_init_flag_up=1;
		/*ZTEMT: Jinghongliang Add for Read AF OTP  ---Start*/
		do{
			CDBG_OIS("<ZTEMT_CAM>%s, read OTP page number is %d\n",__func__,page_number);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
					s_ctrl->sensor_i2c_client,OTP_PAGE_ADDR,
					page_number, MSM_CAMERA_I2C_BYTE_DATA);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
					s_ctrl->sensor_i2c_client,OTP_READ_MODE_ADDR,
					0x01, MSM_CAMERA_I2C_BYTE_DATA);
			mdelay(10);
			for(count = 0;count < 10;count++){
				/* read the OTP ready flag */
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,OTP_READ_READY_ADDR,
					(uint16_t *)&read_otp_ready_flag, MSM_CAMERA_I2C_BYTE_DATA);

				if((read_otp_ready_flag & 0x01) == 0x01){
				/* check the correct page */
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				     s_ctrl->sensor_i2c_client,AF_START_CURRENT,
				     (uint16_t *)&af_start_value, MSM_CAMERA_I2C_WORD_DATA);
				CDBG_OIS("af_start_value = 0x%x,count = %d\n",af_start_value,count);
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				     s_ctrl->sensor_i2c_client,AF_START_INFINITY,
				     (uint16_t *)&af_infinity_value, MSM_CAMERA_I2C_WORD_DATA);
				CDBG_OIS("af_infinity_value = 0x%x,count = %d\n",af_infinity_value,count);
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				     s_ctrl->sensor_i2c_client,AF_START_MACRO,
				     (uint16_t *)&af_macro_value, MSM_CAMERA_I2C_WORD_DATA);
				CDBG_OIS("af_macro_value = 0x%x,count = %d\n",af_macro_value,count);
				break;
				}
				mdelay(10);
			}	
			if ((af_start_value > 0) && (af_infinity_value > 0) && (af_macro_value > 0))
				break;
			page_number = page_number -1;
		  }while(page_number > 11);
		page_number = 14;
		read_otp_ready_flag = 0;
		do{
			CDBG_OIS("<ZTEMT_CAM>%s, page number is %d\n",__func__,page_number);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
						s_ctrl->sensor_i2c_client,OTP_PAGE_ADDR,
						page_number, MSM_CAMERA_I2C_BYTE_DATA);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
						s_ctrl->sensor_i2c_client,OTP_READ_MODE_ADDR,
						0x01, MSM_CAMERA_I2C_BYTE_DATA);
			mdelay(10);
			for(count = 0;count < 10;count++){
				/* read the OTP ready flag */
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
						s_ctrl->sensor_i2c_client,OTP_READ_READY_ADDR,
						(uint16_t *)&read_otp_ready_flag, MSM_CAMERA_I2C_BYTE_DATA);
				
				if((read_otp_ready_flag & 0x01) == 0x01){
				/* check the correct page */
					rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					     s_ctrl->sensor_i2c_client,OSC_VALUE_ADDR,
					     (uint16_t *)&osc_value, MSM_CAMERA_I2C_BYTE_DATA);
					CDBG_OIS("osc_value = 0x%x,count = %d\n",osc_value,count);
					break;
				}
				mdelay(10);
			}
			page_number = page_number -1;
		}while(osc_value == 0x00 && (page_number > 11));
		if ((read_otp_ready_flag & 0x01) == 0x01){
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,HALL_OFFSET_X_ADDR,
					(uint16_t *)&hall_offset_x, MSM_CAMERA_I2C_WORD_DATA);
			CDBG_OIS("hall_offset_x = 0x%x\n",hall_offset_x);
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,HALL_OFFSET_Y_ADDR,
					(uint16_t *)&hall_offset_y, MSM_CAMERA_I2C_WORD_DATA);
			CDBG_OIS("hall_offset_y = 0x%x\n",hall_offset_y);
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,HALL_BIAS_X_ADDR,
					(uint16_t *)&hall_bias_x, MSM_CAMERA_I2C_WORD_DATA);
			CDBG_OIS("hall_bias_x = 0x%x\n",hall_bias_x);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,HALL_BIAS_Y_ADDR,
					(uint16_t *)&hall_bias_y, MSM_CAMERA_I2C_WORD_DATA);
			CDBG_OIS("hall_bias_y = 0x%x\n",hall_bias_y);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,HALL_AD_OFFSET_X_ADDR,
					(uint16_t *)&hall_ad_offset_x, MSM_CAMERA_I2C_WORD_DATA);
			CDBG_OIS("hall_ad_offset_x = 0x%x\n",hall_ad_offset_x);
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,HALL_AD_OFFSET_Y_ADDR,
					(uint16_t *)&hall_ad_offset_y, MSM_CAMERA_I2C_WORD_DATA);
			CDBG_OIS("hall_ad_offset_y = 0x%x\n",hall_ad_offset_y);
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,LOOP_GAIN_X_ADDR,
					(uint16_t *)&loop_gain_x, MSM_CAMERA_I2C_WORD_DATA);
			CDBG_OIS("loop_gain_x = 0x%x\n",loop_gain_x);
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,LOOP_GAIN_Y_ADDR,
					(uint16_t *)&loop_gain_y, MSM_CAMERA_I2C_WORD_DATA);
			CDBG_OIS("loop_gain_y = 0x%x\n",loop_gain_y);
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,GYRO_OFFSET_X_MSB_ADDR,
					(uint16_t *)&gyro_offset_x_msb, MSM_CAMERA_I2C_BYTE_DATA);
			CDBG_OIS("gyro_offset_x_msb = 0x%x\n",gyro_offset_x_msb);
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,GYRO_OFFSET_X_LSB_ADDR,
					(uint16_t *)&gyro_offset_x_lsb, MSM_CAMERA_I2C_BYTE_DATA);
			CDBG_OIS("gyro_offset_x_lsb = 0x%x\n",gyro_offset_x_lsb);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,GYRO_OFFSET_Y_MSB_ADDR,
					(uint16_t *)&gyro_offset_y_msb, MSM_CAMERA_I2C_BYTE_DATA);
			CDBG_OIS("gyro_offset_y_msb = 0x%x\n",gyro_offset_y_msb);
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,GYRO_OFFSET_Y_LSB_ADDR,
					(uint16_t *)&gyro_offset_y_lsb, MSM_CAMERA_I2C_BYTE_DATA);
			CDBG_OIS("gyro_offset_y_lsb = 0x%x\n",gyro_offset_y_lsb);
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,OSC_VALUE_ADDR,
					(uint16_t *)&osc_value, MSM_CAMERA_I2C_BYTE_DATA);
			CDBG_OIS("osc_value = 0x%x\n",osc_value);

		}else{
			printk("OIS OTP Read OSC_VALUE Failed!\n");
		}

		page_number = 14;
		read_otp_ready_flag = 0;
		do{
			CDBG_OIS("<ZTEMT_CAM>%s, page_Gyro_gain number is %d\n",__func__,page_number);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
						s_ctrl->sensor_i2c_client,OTP_PAGE_ADDR,
						page_number, MSM_CAMERA_I2C_BYTE_DATA);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
						s_ctrl->sensor_i2c_client,OTP_READ_MODE_ADDR,
						0x01, MSM_CAMERA_I2C_BYTE_DATA);
			mdelay(10);
			for(count = 0;count < 10;count++){
				/* read the OTP ready flag */
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
						s_ctrl->sensor_i2c_client,OTP_READ_READY_ADDR,
						(uint16_t *)&read_otp_ready_flag, MSM_CAMERA_I2C_BYTE_DATA);
				
				if((read_otp_ready_flag & 0x01) == 0x01){
					/* check the correct page */
					rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
							s_ctrl->sensor_i2c_client,GYRO_GAIN_X_31_24_ADDR,
							(uint16_t *)&gyro_gain_x_31_24, MSM_CAMERA_I2C_BYTE_DATA);
					CDBG_OIS("gyro_gain_x_31_24 = 0x%x\n",gyro_gain_x_31_24);
					break;
				}
			mdelay(10);
			}
			page_number = page_number -1;
		}while(gyro_gain_x_31_24 == 0x00 && (page_number > 11));

		if ((read_otp_ready_flag & 0x01) == 0x01){
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,GYRO_GAIN_X_31_24_ADDR,
					(uint16_t *)&gyro_gain_x_31_24, MSM_CAMERA_I2C_BYTE_DATA);
			CDBG_OIS("gyro_gain_x_31_24 = 0x%x\n",gyro_gain_x_31_24);
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,GYRO_GAIN_X_23_16_ADDR,
					(uint16_t *)&gyro_gain_x_23_16, MSM_CAMERA_I2C_BYTE_DATA);
			CDBG_OIS("gyro_gain_x_23_16 = 0x%x\n",gyro_gain_x_23_16);
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,GYRO_GAIN_X_15_8_ADDR,
					(uint16_t *)&gyro_gain_x_15_8, MSM_CAMERA_I2C_BYTE_DATA);
			CDBG_OIS("gyro_gain_x_15_8 = 0x%x\n",gyro_gain_x_15_8);
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,GYRO_GAIN_X_7_0_ADDR,
					(uint16_t *)&gyro_gain_x_7_0, MSM_CAMERA_I2C_BYTE_DATA);
			CDBG_OIS("gyro_gain_x_7_0 = 0x%x\n",gyro_gain_x_7_0);
			
			gyro_gain_x = (gyro_gain_x_31_24 <<24) | (gyro_gain_x_23_16 <<16)|(gyro_gain_x_15_8<<8)|gyro_gain_x_7_0;

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,GYRO_GAIN_Y_31_24_ADDR,
					(uint16_t *)&gyro_gain_y_31_24, MSM_CAMERA_I2C_BYTE_DATA);
			CDBG_OIS("gyro_gain_y_31_24 = 0x%x\n",gyro_gain_y_31_24);
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,GYRO_GAIN_Y_23_16_ADDR,
					(uint16_t *)&gyro_gain_y_23_16, MSM_CAMERA_I2C_BYTE_DATA);
			CDBG_OIS("gyro_gain_y_23_16 = 0x%x\n",gyro_gain_y_23_16);
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,GYRO_GAIN_Y_15_8_ADDR,
					(uint16_t *)&gyro_gain_y_15_8, MSM_CAMERA_I2C_BYTE_DATA);
			CDBG_OIS("gyro_gain_y_15_8 = 0x%x\n",gyro_gain_y_15_8);
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,GYRO_GAIN_Y_7_0_ADDR,
					(uint16_t *)&gyro_gain_y_7_0, MSM_CAMERA_I2C_BYTE_DATA);
			CDBG_OIS("gyro_gain_y_7_0 = 0x%x\n",gyro_gain_y_7_0);
			
			gyro_gain_y = (gyro_gain_y_31_24 <<24) | (gyro_gain_y_23_16 <<16)|(gyro_gain_y_15_8<<8)|gyro_gain_y_7_0;
		}else{
			printk("OIS OTP Read GYRO GAIN Failed!\n");
		}
	/*ZTEMT: Jinghongliang Add for Read AF OTP  ---End*/
	} 
	      
}
static unsigned long otp_duration = HZ/1000;
static void zte_workquene_schedule_imx135_gbao(struct msm_sensor_ctrl_t* s_ctrl)
{
	static int once = 1;
	
	printk("%s once=%d %d\n", __func__, once, __LINE__);
	if (once == 1) {
		once = 0;
		
		printk("%s once=%d %d\n", __func__, once, __LINE__);
		return;
	}
	printk("%s once=%d %d\n", __func__, once, __LINE__);
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
		RtnCen(0x00);
		SrvCon(0x00,0);  
		SrvCon(0x01,0);  
		msm_ois_release_cci();		
		mutex_unlock(&s_ctrl->zte_otp_mutex);
	}
	printk("%s %d\n", __func__, ois_init_flag_down);
}
static void zte_workquene_cancel_imx135_gbao(struct msm_sensor_ctrl_t* s_ctrl)
{
	static int once = 1;
	if (once == 1) {
		once = 0;
		return;
	}
	cancel_delayed_work_sync(&s_ctrl->zte_otp_worker);
}

static void zte_workquene_init_imx135_gbao(struct msm_sensor_ctrl_t* s_ctrl)
{	     
	INIT_DELAYED_WORK(&s_ctrl->zte_otp_worker, imx135_ois_otp);
	mutex_init(&s_ctrl->zte_otp_mutex);
}
static void zte_control_ois_imx135_gbao(struct msm_sensor_ctrl_t* s_ctrl, int enable)
{
	if (enable) {
		mutex_lock(&s_ctrl->zte_otp_mutex);
		OisEna();		
		SetH1cMod(0xff);
		mutex_unlock(&s_ctrl->zte_otp_mutex);
	} else {
		mutex_lock(&s_ctrl->zte_otp_mutex);
		RtnCen(0x00);
		mutex_unlock(&s_ctrl->zte_otp_mutex);
	}
}

static struct msm_sensor_ctrl_t imx135_s_ctrl = {
	.sensor_i2c_client = &imx135_sensor_i2c_client,
	.power_setting_array.power_setting = imx135_power_setting,
	.power_setting_array.size = ARRAY_SIZE(imx135_power_setting),
	//added by congshan start
	.zte_workquene_init = zte_workquene_init_imx135_gbao,
	.zte_adaptive_sensor = zte_adaptive_imx135_gbao,
	.zte_read_otp = zte_read_otp_imx135_gbao,
	.zte_workquene_schedule = zte_workquene_schedule_imx135_gbao,
	.zte_power_down = zte_power_down,
	.zte_workquene_cancel = zte_workquene_cancel_imx135_gbao,
	.zte_control_ois = zte_control_ois_imx135_gbao,
	//added by congshan end
	.msm_sensor_mutex = &imx135_mut,
	.sensor_v4l2_subdev_info = imx135_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx135_subdev_info),
};

module_init(imx135_gbao_init_module);
module_exit(imx135_gbao_exit_module);
MODULE_DESCRIPTION("imx135");
MODULE_LICENSE("GPL v2");
