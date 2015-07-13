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
#include "ztemt_maf.h"
#include "msm_cci.h"

/*#define ZTEMT_MAF_DEBUG*/
#undef CDBG
#ifdef ZTEMT_MAF_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

static struct ztemt_maf_ctrl_t ztemt_maf_t;

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

void ZtemtMoveFocus(unsigned short reg_addr, unsigned char write_data_8)
{
        int32_t rc=0;
	 ztemt_maf_t.i2c_client.addr_type = ZTEMT_MAF_BYTE_DATA;
	 ztemt_maf_t.i2c_data_type = ZTEMT_MAF_BYTE_DATA;
	 
	 rc =  ztemt_maf_t.i2c_client.i2c_func_tbl->i2c_write
	 (
		&ztemt_maf_t.i2c_client,
		reg_addr, write_data_8,
		ztemt_maf_t.i2c_data_type);
	 if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	 }

};

EXPORT_SYMBOL(ZtemtMoveFocus);

void ztemt_maf_init_cci(void)
{
       int rc;
	rc = ztemt_maf_t.i2c_client.i2c_func_tbl->i2c_util(
		&ztemt_maf_t.i2c_client, MSM_CCI_INIT);

	if (rc < 0)
		pr_err("cci_init failed\n");
	
}

void ztemt_maf_release_cci(void)
{
       int rc;
	rc = ztemt_maf_t.i2c_client.i2c_func_tbl->i2c_util(
		&ztemt_maf_t.i2c_client, MSM_CCI_RELEASE);

	if (rc < 0)
		pr_err("cci_init failed\n");
	
}

static int32_t ztemt_maf_i2c_init(void)
{
	int32_t rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	CDBG("Enter\n");

       ztemt_maf_t.cci_master = 0;
	ztemt_maf_t.cam_name = 0;

	ztemt_maf_t.i2c_client.i2c_func_tbl = &msm_sensor_cci_func_tbl;
	ztemt_maf_t.i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!ztemt_maf_t.i2c_client.cci_client) {
		pr_err("failed no memory\n");
		return -ENOMEM;
	}

	cci_client = ztemt_maf_t.i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();

	#ifdef CONFIG_IMX214 
	ztemt_maf_t.i2c_client.cci_client->sid = 0x18 >> 1;
    #else
	ztemt_maf_t.i2c_client.cci_client->sid = 0x18 >> 1; //jinghongliang: This changes follow other VCM IC   	
	#endif
	ztemt_maf_t.i2c_client.cci_client->retries = 3;
	ztemt_maf_t.i2c_client.cci_client->id_map = 0;
	ztemt_maf_t.i2c_client.cci_client->cci_i2c_master = 0;

	CDBG("Exit\n");

	return rc;
}

static int __init ztemt_maf_init_module(void)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	rc = ztemt_maf_i2c_init();
	if (!rc)
		return rc;
	CDBG("%s:%d rc %d\n", __func__, __LINE__, rc);
	return rc;
}

module_init(ztemt_maf_init_module);
MODULE_DESCRIPTION("MSM ZTEMT_MAF");
MODULE_LICENSE("GPL v2");
