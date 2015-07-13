/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_sd.h"
#include "msm_ois_lc898122.h"
#include "msm_cci.h"

/*#define MSM_OIS_DEBUG*/
#undef CDBG
#ifdef MSM_OIS_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

static struct msm_ois_ctrl_t msm_ois_lc898122_t;

static struct msm_ois_ctrl_t msm_af_lc898122_t;

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
};

	
void msm_af_reg_write_lc898122(unsigned short reg_addr, unsigned char write_data_8)
{
        int32_t rc=0;
	 msm_af_lc898122_t.i2c_client.cci_client->sid = 0x24;
	 msm_af_lc898122_t.i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	 msm_af_lc898122_t.i2c_data_type = MSM_OIS_BYTE_DATA;
	 
	 rc =  msm_af_lc898122_t.i2c_client.i2c_func_tbl->i2c_write
	 (
		&msm_af_lc898122_t.i2c_client,
		reg_addr, write_data_8,
		msm_af_lc898122_t.i2c_data_type);

	 if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	 }

};

EXPORT_SYMBOL(msm_af_reg_write_lc898122);


void read_ois_byte_data_lc898122(unsigned short reg_addr, unsigned char *read_data_8)
{
        int32_t rc=0;
        uint16_t temp_data;
        msm_ois_lc898122_t.i2c_client.cci_client->sid = 0xA0>>1;
 	 msm_ois_lc898122_t.i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	 msm_ois_lc898122_t.i2c_data_type = MSM_OIS_BYTE_DATA;
	 rc =  msm_ois_lc898122_t.i2c_client.i2c_func_tbl->i2c_read
	 (
		&msm_ois_lc898122_t.i2c_client,
		reg_addr, &temp_data,
		msm_ois_lc898122_t.i2c_data_type);

	 if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	 }

	 *read_data_8 = (unsigned char)temp_data;
		
};

void read_ois_word_data_lc898122(unsigned short reg_addr, uint16_t *read_data_16)
{
        int32_t rc=0;
        uint16_t temp_data;
        msm_ois_lc898122_t.i2c_client.cci_client->sid = 0xA0>>1;
 	 msm_ois_lc898122_t.i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	 msm_ois_lc898122_t.i2c_data_type = MSM_OIS_WORD_DATA;
	 rc =  msm_ois_lc898122_t.i2c_client.i2c_func_tbl->i2c_read
	 (
		&msm_ois_lc898122_t.i2c_client,
		reg_addr, &temp_data,
		msm_ois_lc898122_t.i2c_data_type);

	 if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	 }
        
	 *read_data_16 = temp_data;
		
};


void imx214_gbao_read_ois_byte_data_lc898122(unsigned short reg_addr, unsigned char *read_data_8)
{
        int32_t rc=0;
        uint16_t temp_data;
        msm_ois_lc898122_t.i2c_client.cci_client->sid = 0xA8>>1;
 	 msm_ois_lc898122_t.i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	 msm_ois_lc898122_t.i2c_data_type = MSM_OIS_BYTE_DATA;
	 rc =  msm_ois_lc898122_t.i2c_client.i2c_func_tbl->i2c_read
	 (
		&msm_ois_lc898122_t.i2c_client,
		reg_addr, &temp_data,
		msm_ois_lc898122_t.i2c_data_type);

	 if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	 }

	 *read_data_8 = (unsigned char)temp_data;
		
};


void RegReadA_lc898122(unsigned short reg_addr, unsigned char *read_data_8)
{
        int32_t rc=0;
        uint16_t temp_data;
        msm_ois_lc898122_t.i2c_client.cci_client->sid = 0x24;
 	 msm_ois_lc898122_t.i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	 msm_ois_lc898122_t.i2c_data_type = MSM_OIS_BYTE_DATA;
	 rc =  msm_ois_lc898122_t.i2c_client.i2c_func_tbl->i2c_read
	 (
		&msm_ois_lc898122_t.i2c_client,
		reg_addr, &temp_data,
		msm_ois_lc898122_t.i2c_data_type);

	 if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	 }
         
	 *read_data_8 = (unsigned char)temp_data;
		
};
	
void RegWriteA_lc898122(unsigned short reg_addr, unsigned char write_data_8)
{
        int32_t rc=0;
	 msm_ois_lc898122_t.i2c_client.cci_client->sid = 0x24;
	 msm_ois_lc898122_t.i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	 msm_ois_lc898122_t.i2c_data_type = MSM_OIS_BYTE_DATA;
	 
	 rc =  msm_ois_lc898122_t.i2c_client.i2c_func_tbl->i2c_write
	 (
		&msm_ois_lc898122_t.i2c_client,
		reg_addr, write_data_8,
		msm_ois_lc898122_t.i2c_data_type);

	 if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	 }

};


void RamReadA_lc898122(unsigned short ram_addr, void *read_data_16)
{
        int32_t rc=0;
        unsigned short *temp_data_16=(unsigned short *)read_data_16;
	 msm_ois_lc898122_t.i2c_client.cci_client->sid = 0x24;
	 msm_ois_lc898122_t.i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	 msm_ois_lc898122_t.i2c_data_type = MSM_OIS_WORD_DATA;
	 rc =  msm_ois_lc898122_t.i2c_client.i2c_func_tbl->i2c_read
	 (
		&msm_ois_lc898122_t.i2c_client,
		ram_addr, temp_data_16,
		msm_ois_lc898122_t.i2c_data_type);

	 if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	 }

};
void RamWriteA_lc898122(unsigned short ram_addr, unsigned short write_data_16)
{
        int32_t rc=0;
	 msm_ois_lc898122_t.i2c_client.cci_client->sid = 0x24;
	 msm_ois_lc898122_t.i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	 msm_ois_lc898122_t.i2c_data_type = MSM_OIS_WORD_DATA;
	 rc =  msm_ois_lc898122_t.i2c_client.i2c_func_tbl->i2c_write
	 (
		&msm_ois_lc898122_t.i2c_client,
		ram_addr, write_data_16,
		msm_ois_lc898122_t.i2c_data_type);

	 if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	 }
};

void RamRead32A_lc898122(unsigned short ram_addr, void *read_data_32)
{
        uint8_t data[4];
        unsigned long *temp_read_data_32=(unsigned long *)read_data_32;
        int32_t rc=0;

	 msm_ois_lc898122_t.i2c_client.cci_client->sid = 0x24;
	 msm_ois_lc898122_t.i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	 msm_ois_lc898122_t.i2c_data_type = MSM_OIS_BYTE_DATA;
	 rc =  msm_ois_lc898122_t.i2c_client.i2c_func_tbl->i2c_read_seq
	 (
		&msm_ois_lc898122_t.i2c_client,
		ram_addr, &data[0],
		4);

	 if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	 }
        *temp_read_data_32=  ((data[0]<<24)|(data[1]<<16)|(data[2]<<8)|data[3]);
};


void RamWrite32A_lc898122(unsigned short ram_addr, unsigned long write_data_32)
{
        uint8_t data[4];
        int32_t rc=0;

        data[0] = (write_data_32 >> 24) &0xFF;
        data[1] = (write_data_32 >> 16) &0xFF;

        data[2] = (write_data_32 >> 8) &0xFF;
        data[3] = (write_data_32) &0xFF;
	 msm_ois_lc898122_t.i2c_client.cci_client->sid = 0x24;
	 msm_ois_lc898122_t.i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	 msm_ois_lc898122_t.i2c_data_type = MSM_OIS_BYTE_DATA;
	 rc =  msm_ois_lc898122_t.i2c_client.i2c_func_tbl->i2c_write_seq
	 (
		&msm_ois_lc898122_t.i2c_client,
		ram_addr, &data[0],
		4);

	 if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	 }
      
};
EXPORT_SYMBOL(read_ois_byte_data_lc898122);
EXPORT_SYMBOL(read_ois_word_data_lc898122);
EXPORT_SYMBOL(RegReadA_lc898122);
EXPORT_SYMBOL(RegWriteA_lc898122);
EXPORT_SYMBOL(RamReadA_lc898122);
EXPORT_SYMBOL(RamWriteA_lc898122);
EXPORT_SYMBOL(RamRead32A_lc898122);
EXPORT_SYMBOL(RamWrite32A_lc898122);
EXPORT_SYMBOL(imx214_gbao_read_ois_byte_data_lc898122);


void msm_af_init_cci_lc898122(void)
{
       int rc;
	rc = msm_af_lc898122_t.i2c_client.i2c_func_tbl->i2c_util(
		&msm_af_lc898122_t.i2c_client, MSM_CCI_INIT);

	if (rc < 0)
		pr_err("cci_init failed\n");
	
}

void msm_af_release_cci_lc898122(void)
{
       int rc;
	rc = msm_af_lc898122_t.i2c_client.i2c_func_tbl->i2c_util(
		&msm_af_lc898122_t.i2c_client, MSM_CCI_RELEASE);

	if (rc < 0)
		pr_err("cci_init failed\n");
	
}
EXPORT_SYMBOL(msm_af_init_cci_lc898122);
EXPORT_SYMBOL(msm_af_release_cci_lc898122);


void msm_ois_init_cci_lc898122(void)
{
       int rc;
	rc = msm_ois_lc898122_t.i2c_client.i2c_func_tbl->i2c_util(
		&msm_ois_lc898122_t.i2c_client, MSM_CCI_INIT);

	if (rc < 0)
		pr_err("cci_init failed\n");
	
}

void msm_ois_release_cci_lc898122(void)
{
       int rc;
	rc = msm_ois_lc898122_t.i2c_client.i2c_func_tbl->i2c_util(
		&msm_ois_lc898122_t.i2c_client, MSM_CCI_RELEASE);

	if (rc < 0)
		pr_err("cci_init failed\n");
	
}

static int32_t msm_ois_i2c_init_lc898122(void)
{
	int32_t rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_camera_cci_client *af_cci_client = NULL;
	CDBG("Enter\n");

       msm_ois_lc898122_t.cci_master = 0;
	msm_ois_lc898122_t.cam_name = 0;

	msm_ois_lc898122_t.i2c_client.i2c_func_tbl = &msm_sensor_cci_func_tbl;
	msm_ois_lc898122_t.i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!msm_ois_lc898122_t.i2c_client.cci_client) {
		pr_err("failed no memory\n");
		return -ENOMEM;
	}

	cci_client = msm_ois_lc898122_t.i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();

	msm_ois_lc898122_t.i2c_client.cci_client->sid = 0x24;
	msm_ois_lc898122_t.i2c_client.cci_client->retries = 3;
	msm_ois_lc898122_t.i2c_client.cci_client->id_map = 0;
	msm_ois_lc898122_t.i2c_client.cci_client->cci_i2c_master = 0;

	msm_af_lc898122_t.cci_master = 0;
	msm_af_lc898122_t.cam_name = 0;

	msm_af_lc898122_t.i2c_client.i2c_func_tbl = &msm_sensor_cci_func_tbl;
	msm_af_lc898122_t.i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!msm_af_lc898122_t.i2c_client.cci_client) {
		pr_err("failed no memory\n");
		return -ENOMEM;
	}

	af_cci_client = msm_af_lc898122_t.i2c_client.cci_client;
	af_cci_client->cci_subdev = msm_cci_get_subdev();

	msm_af_lc898122_t.i2c_client.cci_client->sid = 0x24;
	msm_af_lc898122_t.i2c_client.cci_client->retries = 3;
	msm_af_lc898122_t.i2c_client.cci_client->id_map = 0;
	msm_af_lc898122_t.i2c_client.cci_client->cci_i2c_master = 0;
	
	CDBG("Exit\n");

	return rc;
}

static int __init msm_ois_init_module_lc898122(void)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	rc = msm_ois_i2c_init_lc898122();
	if (!rc)
		return rc;
	CDBG("%s:%d rc %d\n", __func__, __LINE__, rc);
	return rc;
}

module_init(msm_ois_init_module_lc898122);
MODULE_DESCRIPTION("MSM OIS");
MODULE_LICENSE("GPL v2");
