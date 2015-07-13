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
#ifndef ZTEMT_MAF_H
#define ZTEMT_MAF_H

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <mach/camera2.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_camera.h>
#include "msm_camera_i2c.h"

struct ztemt_maf_ctrl_t;

enum ztemt_maf_data_type {
	ZTEMT_MAF_BYTE_DATA = 1,
	ZTEMT_MAF_WORD_DATA,
};

struct ztemt_maf_ctrl_t {
	struct msm_camera_i2c_client i2c_client;
	enum af_camera_name cam_name;
	enum ztemt_maf_data_type i2c_data_type;
	enum cci_i2c_master_t cci_master;
};

void ZtemtMoveFocus(unsigned short reg_addr, unsigned char write_data_8);

#endif
