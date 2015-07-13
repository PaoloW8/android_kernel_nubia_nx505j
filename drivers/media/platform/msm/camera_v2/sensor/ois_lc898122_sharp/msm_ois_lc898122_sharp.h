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
#ifndef MSM_OIS_H
#define MSM_OIS_H

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <mach/camera2.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_camera.h>
#include "msm_camera_i2c.h"

struct msm_ois_ctrl_t;

enum msm_ois_data_type {
	MSM_OIS_BYTE_DATA = 1,
	MSM_OIS_WORD_DATA,
};

struct msm_ois_ctrl_t {
	struct msm_camera_i2c_client i2c_client;
	enum af_camera_name cam_name;
	enum msm_ois_data_type i2c_data_type;
	enum cci_i2c_master_t cci_master;
};

 void RamReadA_lc898122_sharp(unsigned short ram_addr, void *read_data_16);
 void RamWriteA_lc898122_sharp(unsigned short ram_addr, unsigned short write_data_16);
 void RamRead32A_lc898122_sharp(unsigned short ram_addr, void *read_data_32);
 void RamWrite32A_lc898122_sharp(unsigned short ram_addr, unsigned long write_data_32);
 void RegReadA_lc898122_sharp(unsigned short reg_addr, unsigned char *read_data_8);
 void RegWriteA_lc898122_sharp(unsigned short reg_addr, unsigned char write_data_8);

#endif
