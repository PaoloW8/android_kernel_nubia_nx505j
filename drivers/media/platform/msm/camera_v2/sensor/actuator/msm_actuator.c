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
#include "msm_actuator.h"
#include "msm_cci.h"

DEFINE_MSM_MUTEX(msm_actuator_mutex);

/*#define MSM_ACUTUATOR_DEBUG*/
#undef CDBG
#ifdef MSM_ACUTUATOR_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

#if defined(CONFIG_ZTE_CAMERA_NX505J) || defined(CONFIG_ZTE_CAMERA_NX507J) || defined(CONFIG_ZTE_CAMERA_NX504J)
extern unsigned short af_infinity_value_imx214;
extern unsigned short af_macro_value_imx214;
#define IMX214_MIN_INITIAL_CODE 30
#define IMX214_AF_OFFSET 68
#endif

#if defined(CONFIG_IMX214_OIS_SHARP) || defined(CONFIG_IMX135_GBAO_LC898122)
unsigned short af_start_value_lc898122 = 0;
unsigned short af_infinity_value_lc898122 = 0;
unsigned short af_macro_value_lc898122 = 0;
unsigned short af_start_value_lc898122_sharp = 0;
unsigned short af_infinity_value_lc898122_sharp = 0;
unsigned short af_macro_value_lc898122_sharp = 0;
#endif

/*ZTEMT: Jinghongliang Add for Read AF OTP  ---Start*/
#if defined(CONFIG_IMX135_GBAO) || defined(CONFIG_IMX135_Z5S)
extern unsigned short af_start_value;
extern unsigned short af_infinity_value;
extern unsigned short af_macro_value;
#endif
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---End*/

#ifdef CONFIG_IMX214_APP
void RegRead8byte(uint16_t reg_addr, struct msm_actuator_ctrl_t *a_ctrl);
int32_t Regwrite8byte(uint16_t reg_addr, unsigned long write_data1_32, \
	unsigned long write_data2_32, struct msm_actuator_ctrl_t *a_ctrl);
extern int ofei_720p_120fps_flag;
int32_t OfeiRegwrite8byte(uint16_t reg_addr, unsigned long write_data1_32
	, unsigned long write_data2_32);

void ofei_swtich_to_preview_mode(void)
{
    OfeiRegwrite8byte(0x20, 0x08010101, 0x90050000);//480k PWM frequence
}

void ofei_swtich_to_snapshot_mode(void)
{
    OfeiRegwrite8byte(0x20, 0x08010101, 0x54030000);//564.7k PWM frequence
}


void ofei_swtich_to_720p_120fps_mode(void)
{
    OfeiRegwrite8byte(0x20, 0x08010101, 0x54030000);//564.7kk PWM frequence
}

void ofei_swtich_to_720p_120fps_mode_new(void)
{
    OfeiRegwrite8byte(0x20, 0x08010101, 0x54030000);//564.7k PWM frequence
}
void ofei_swtich_to_slow_shutter_mode(void)
{
    OfeiRegwrite8byte(0x20, 0x08010101, 0x90050000);//480k PWM frequence
}

#endif
static struct msm_actuator msm_vcm_actuator_table;
static struct msm_actuator msm_piezo_actuator_table;

static struct i2c_driver msm_actuator_i2c_driver;
static struct msm_actuator *actuators[] = {
	&msm_vcm_actuator_table,
	&msm_piezo_actuator_table,
};

static int32_t msm_actuator_piezo_set_default_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t rc = 0;
	struct msm_camera_i2c_reg_setting reg_setting;
	CDBG("Enter\n");

	if (a_ctrl->curr_step_pos != 0) {
		a_ctrl->i2c_tbl_index = 0;
		a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
			a_ctrl->initial_code, 0, 0);
		a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
			a_ctrl->initial_code, 0, 0);
		reg_setting.reg_setting = a_ctrl->i2c_reg_tbl;
		reg_setting.data_type = a_ctrl->i2c_data_type;
		reg_setting.size = a_ctrl->i2c_tbl_index;
		rc = a_ctrl->i2c_client.i2c_func_tbl->
			i2c_write_table_w_microdelay(
			&a_ctrl->i2c_client, &reg_setting);
		if (rc < 0) {
			pr_err("%s: i2c write error:%d\n",
				__func__, rc);
			return rc;
		}
		a_ctrl->i2c_tbl_index = 0;
		a_ctrl->curr_step_pos = 0;
	}
	CDBG("Exit\n");
	return rc;
}
static void msm_actuator_parse_i2c_params(struct msm_actuator_ctrl_t *a_ctrl,
	int16_t next_lens_position, uint32_t hw_params, uint16_t delay)
{
	struct msm_actuator_reg_params_t *write_arr = a_ctrl->reg_tbl;
	uint32_t hw_dword = hw_params;
	uint16_t i2c_byte1 = 0, i2c_byte2 = 0;
	uint16_t value = 0;
	uint32_t size = a_ctrl->reg_tbl_size, i = 0;
	struct msm_camera_i2c_reg_array *i2c_tbl = a_ctrl->i2c_reg_tbl;
	CDBG("Enter\n");
	for (i = 0; i < size; i++) {
		/* check that the index into i2c_tbl cannot grow larger that
		the allocated size of i2c_tbl */
		if ((a_ctrl->total_steps + 1) < (a_ctrl->i2c_tbl_index)) {
			break;
		}
		if (write_arr[i].reg_write_type == MSM_ACTUATOR_WRITE_DAC) {
			value = (next_lens_position <<
				write_arr[i].data_shift) |
				((hw_dword & write_arr[i].hw_mask) >>
				write_arr[i].hw_shift);

			if (write_arr[i].reg_addr != 0xFFFF) {
				i2c_byte1 = write_arr[i].reg_addr;
				i2c_byte2 = value;
				if (size != (i+1)) {
#if defined(CONFIG_IMX135_GBAO) || defined(CONFIG_IMX135_Z5S)
					if (a_ctrl->i2c_client.cci_client->sid == 0x1c >> 1) {
						i2c_byte2 = (value & 0x0300) >> 8;
					} else {
						i2c_byte2 = value & 0xFF;
					}						
#else
					i2c_byte2 = value & 0xFF;
#endif
					CDBG("byte1:0x%x, byte2:0x%x\n",
						i2c_byte1, i2c_byte2);
					i2c_tbl[a_ctrl->i2c_tbl_index].
						reg_addr = i2c_byte1;
					i2c_tbl[a_ctrl->i2c_tbl_index].
						reg_data = i2c_byte2;
					i2c_tbl[a_ctrl->i2c_tbl_index].
						delay = 0;
					a_ctrl->i2c_tbl_index++;
					i++;
					i2c_byte1 = write_arr[i].reg_addr;
#if defined(CONFIG_IMX135_GBAO) || defined(CONFIG_IMX135_Z5S)
					if (a_ctrl->i2c_client.cci_client->sid == 0x1c >> 1) {
						i2c_byte2 = value & 0xFF;
					} else {
						i2c_byte2 = (value & 0xFF00) >> 8;
					}
#else
					i2c_byte2 = (value & 0xFF00) >> 8;
#endif
				}
			} else {
#ifdef CONFIG_IMX135_GBAO_LC898122
			if (a_ctrl->i2c_client.cci_client->sid == 0x48 >> 1) 
			{
                            i2c_byte1 = ((value & 0x0700) >> 8)|0x04;
				i2c_byte2 = value & 0xFF;
				//printk("af i2c_byte1 = 0x%x\n",i2c_byte1);
			       //printk("af i2c_byte2 = 0x%x\n",i2c_byte2);
			}else{
				i2c_byte1 = (value & 0xFF00) >> 8;
				i2c_byte2 = value & 0xFF;
			}
#elif defined(CONFIG_IMX214_LC898122) || defined(CONFIG_IMX214_OIS_SHARP)
                     if (a_ctrl->i2c_client.cci_client->sid == 0x48 >> 1) 
			{
                            i2c_byte1 = ((value & 0x0700) >> 8)|0x04;
				i2c_byte2 = value & 0xFF;
				//printk("  af i2c_byte1 = 0x%x\n",i2c_byte1);
			       //printk("  af i2c_byte2 = 0x%x\n",i2c_byte2);
			}else{
				i2c_byte1 = (value & 0xFF00) >> 8;
				i2c_byte2 = value & 0xFF;
			}
#else
                            i2c_byte1 = (value & 0xFF00) >> 8;
				i2c_byte2 = value & 0xFF;
#endif
			}
		} else {
			i2c_byte1 = write_arr[i].reg_addr;
			i2c_byte2 = (hw_dword & write_arr[i].hw_mask) >>
				write_arr[i].hw_shift;
		}
		CDBG("i2c_byte1:0x%x, i2c_byte2:0x%x\n", i2c_byte1, i2c_byte2);
		i2c_tbl[a_ctrl->i2c_tbl_index].reg_addr = i2c_byte1;
		i2c_tbl[a_ctrl->i2c_tbl_index].reg_data = i2c_byte2;
		i2c_tbl[a_ctrl->i2c_tbl_index].delay = delay;
		a_ctrl->i2c_tbl_index++;
	}
	CDBG("Exit\n");
}

static int32_t msm_actuator_init_focus(struct msm_actuator_ctrl_t *a_ctrl,
	uint16_t size, enum msm_actuator_data_type type,
	struct reg_settings_t *settings)
{
	int32_t rc = -EFAULT;
	int32_t i = 0;
	CDBG("Enter\n");
#if defined(CONFIG_IMX214_APP) || defined(CONFIG_IMX135_GBAO_LC898122)|| defined(CONFIG_IMX214_OIS_SHARP)|| defined(CONFIG_IMX214_LC898122)
	
#ifdef CONFIG_IMX214_APP
	i = 0;
	if ((a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) && \
		(a_ctrl->i2c_client.cci_client->sid == 0x32 >> 1)) {
		//RegRead8byte(0x91, a_ctrl);
		printk("k_debug enter msm_actuator_init_focus CONFIG_IMX214_APP ofei_flag=%x\n ",ofei_720p_120fps_flag);
		//rc = Regwrite8byte(0x20, 0x08010103, 0x20060000, a_ctrl);
		//rc = Regwrite8byte(0x20, 0x08010101, 0x90050000, a_ctrl);
		if (ofei_720p_120fps_flag == 0)
			ofei_swtich_to_preview_mode();
	    if (ofei_720p_120fps_flag == 1)
			ofei_swtich_to_720p_120fps_mode();
		if (ofei_720p_120fps_flag == 0xa0)
			ofei_swtich_to_snapshot_mode();
        if (ofei_720p_120fps_flag == 0xa4)
			ofei_swtich_to_720p_120fps_mode_new();
		if (ofei_720p_120fps_flag == 0xa6)
			ofei_swtich_to_slow_shutter_mode();
		msleep(100);
		rc = Regwrite8byte(0x15, 0x00640000, 0x00000000, a_ctrl);
		//rc = Regwrite8byte(0x01, 0x03000000, 0x00000000, a_ctrl);
		rc = Regwrite8byte(0x27, 0x30000000, 0x00000000, a_ctrl);
	}
#endif
#ifdef CONFIG_IMX135_GBAO_LC898122
	i = 0;
	if ((a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) && \
		(a_ctrl->i2c_client.cci_client->sid == 0x48 >> 1)) {
		 rc = 0;
               //printk(" goto init gbao lc898122 focus\n");
	}
#endif
#ifdef CONFIG_IMX214_OIS_SHARP
	i = 0;
	if ((a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) && \
		(a_ctrl->i2c_client.cci_client->sid == 0x48 >> 1)) {
		 rc = 0;
               //printk(" goto init gbao lc898122 focus\n");
	}
#endif
#ifdef CONFIG_IMX214_LC898122
	i = 0;
	if ((a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) && \
		(a_ctrl->i2c_client.cci_client->sid == 0x48 >> 1)) {
		 rc = 0;
               //printk(" goto init gbao lc898122 focus\n");
	}
#endif

	if ((a_ctrl->i2c_client.cci_client->sid != 0x32 >> 1) && (a_ctrl->i2c_client.cci_client->sid != 0x48 >> 1)) {
		for (i = 0; i < size; i++) {
			switch (type) {
			case MSM_ACTUATOR_BYTE_DATA:
				rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&a_ctrl->i2c_client,
					settings[i].reg_addr,
					settings[i].reg_data, MSM_CAMERA_I2C_BYTE_DATA);
				break;
			case MSM_ACTUATOR_WORD_DATA:
				rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&a_ctrl->i2c_client,
					settings[i].reg_addr,
					settings[i].reg_data, MSM_CAMERA_I2C_WORD_DATA);
				break;
			default:
				pr_err("Unsupport data type: %d\n", type);
				break;
			}
			if (rc < 0)
				break;
		}
	}
#else
	for (i = 0; i < size; i++) {
		switch (type) {
		case MSM_ACTUATOR_BYTE_DATA:
			rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&a_ctrl->i2c_client,
				settings[i].reg_addr,
				settings[i].reg_data, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_ACTUATOR_WORD_DATA:
			rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&a_ctrl->i2c_client,
				settings[i].reg_addr,
				settings[i].reg_data, MSM_CAMERA_I2C_WORD_DATA);
			break;
		default:
			pr_err("Unsupport data type: %d\n", type);
			break;
		}
		if (rc < 0)
			break;
	}
#endif
	a_ctrl->curr_step_pos = 0;
	CDBG("Exit\n");
	return rc;
}

static void msm_actuator_write_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	uint16_t curr_lens_pos,
	struct damping_params_t *damping_params,
	int8_t sign_direction,
	int16_t code_boundary)
{
	int16_t next_lens_pos = 0;
	uint16_t damping_code_step = 0;
	uint16_t wait_time = 0;
	CDBG("Enter\n");

	damping_code_step = damping_params->damping_step;
	wait_time = damping_params->damping_delay;

	/* Write code based on damping_code_step in a loop */
	for (next_lens_pos =
		curr_lens_pos + (sign_direction * damping_code_step);
		(sign_direction * next_lens_pos) <=
			(sign_direction * code_boundary);
		next_lens_pos =
			(next_lens_pos +
				(sign_direction * damping_code_step))) {
		a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
			next_lens_pos, damping_params->hw_params, wait_time);
		curr_lens_pos = next_lens_pos;
	}

	if (curr_lens_pos != code_boundary) {
		//pr_err("%s[jun] positon=%d,\n",__func__,code_boundary);
		a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
			code_boundary, damping_params->hw_params, wait_time);
	}
	CDBG("Exit\n");
}

static int32_t msm_actuator_piezo_move_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t dest_step_position = move_params->dest_step_pos;
	struct damping_params_t ringing_params_kernel;
	int32_t rc = 0;
	int32_t num_steps = move_params->num_steps;
	struct msm_camera_i2c_reg_setting reg_setting;
	CDBG("Enter\n");

	if (copy_from_user(&ringing_params_kernel,
		&(move_params->ringing_params[0]),
		sizeof(struct damping_params_t))) {
		pr_err("copy_from_user failed\n");
		return -EFAULT;
	}

	if (num_steps == 0)
		return rc;

	a_ctrl->i2c_tbl_index = 0;
	a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
		(num_steps *
		a_ctrl->region_params[0].code_per_step),
		ringing_params_kernel.hw_params, 0);

	reg_setting.reg_setting = a_ctrl->i2c_reg_tbl;
	reg_setting.data_type = a_ctrl->i2c_data_type;
	reg_setting.size = a_ctrl->i2c_tbl_index;
	rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_table_w_microdelay(
		&a_ctrl->i2c_client, &reg_setting);
	if (rc < 0) {
		pr_err("i2c write error:%d\n", rc);
		return rc;
	}
	a_ctrl->i2c_tbl_index = 0;
	a_ctrl->curr_step_pos = dest_step_position;
	CDBG("Exit\n");
	return rc;
}
#if defined(CONFIG_IMX135_GBAO_LC898122)
extern void msm_af_reg_write_lc898122(unsigned short reg_addr, unsigned char write_data_8);
extern void msm_ois_init_cci_lc898122(void);
extern void msm_ois_release_cci_lc898122(void);
#endif

#if defined(CONFIG_IMX214_LC898122)||defined(CONFIG_IMX214_OIS_SHARP)
extern void msm_af_reg_write_lc898122(unsigned short reg_addr, unsigned char write_data_8);
#endif

#if defined(CONFIG_IMX214_OIS_SHARP)
//extern void msm_af_reg_write_lc898122_sharp(unsigned short reg_addr, unsigned char write_data_8);
//extern void	SetTregAf_lc898122_sharp( unsigned short UsTregAf );
extern void msm_ois_init_cci_lc898122_sharp(void);
extern void msm_ois_release_cci_lc898122_sharp(void);
#endif

static int32_t msm_actuator_move_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t rc = 0;
	struct damping_params_t ringing_params_kernel;
	int8_t sign_dir = move_params->sign_dir;
	uint16_t step_boundary = 0;
	uint16_t target_step_pos = 0;
	uint16_t target_lens_pos = 0;
	int16_t dest_step_pos = move_params->dest_step_pos;
	uint16_t curr_lens_pos = 0;
	int dir = move_params->dir;
	int32_t num_steps = move_params->num_steps;
	struct msm_camera_i2c_reg_setting reg_setting;
	
	curr_lens_pos = a_ctrl->step_position_table[a_ctrl->curr_step_pos];
	move_params->curr_lens_pos = curr_lens_pos;

	if (copy_from_user(&ringing_params_kernel,
		&(move_params->ringing_params[a_ctrl->curr_region_index]),
		sizeof(struct damping_params_t))) {
		pr_err("copy_from_user failed\n");
		return -EFAULT;
	}


	CDBG("called, dir %d, num_steps %d\n", dir, num_steps);

	if (dest_step_pos == a_ctrl->curr_step_pos)
		return rc;
	
	if ((sign_dir > MSM_ACTUATOR_MOVE_SIGNED_NEAR) ||
		(sign_dir < MSM_ACTUATOR_MOVE_SIGNED_FAR)) {
		pr_err("%s:%d Invalid sign_dir = %d\n",
		__func__, __LINE__, sign_dir);
		return -EFAULT;
	}
	if ((dir > MOVE_FAR) || (dir < MOVE_NEAR)) {
		pr_err("%s:%d Invalid direction = %d\n",
		__func__, __LINE__, dir);
		return -EFAULT;
	}
	if (dest_step_pos > a_ctrl->total_steps) {
		pr_err("Step pos greater than total steps = %d\n",
		dest_step_pos);
		return -EFAULT;
	}
	curr_lens_pos = a_ctrl->step_position_table[a_ctrl->curr_step_pos];
//comment by congshan start
	//a_ctrl->i2c_tbl_index = 0;
//comment by congshan end¢˜
	CDBG("curr_step_pos =%d dest_step_pos =%d curr_lens_pos=%d\n",
		a_ctrl->curr_step_pos, dest_step_pos, curr_lens_pos);

	while (a_ctrl->curr_step_pos != dest_step_pos) {
		//added  by congshan start
		a_ctrl->i2c_tbl_index = 0;
		//added by congshan end
		step_boundary =
			a_ctrl->region_params[a_ctrl->curr_region_index].
			step_bound[dir];
		if ((dest_step_pos * sign_dir) <=
			(step_boundary * sign_dir)) {

			target_step_pos = dest_step_pos;
			target_lens_pos =
				a_ctrl->step_position_table[target_step_pos];
			a_ctrl->func_tbl->actuator_write_focus(a_ctrl,
					curr_lens_pos,
					&ringing_params_kernel,
					sign_dir,
					target_lens_pos);
			curr_lens_pos = target_lens_pos;

		} else {
			target_step_pos = step_boundary;
			target_lens_pos =
				a_ctrl->step_position_table[target_step_pos];
			a_ctrl->func_tbl->actuator_write_focus(a_ctrl,
					curr_lens_pos,
					&ringing_params_kernel,
					sign_dir,
					target_lens_pos);
			curr_lens_pos = target_lens_pos;

			a_ctrl->curr_region_index += sign_dir;
		}
		a_ctrl->curr_step_pos = target_step_pos;
	}

	move_params->curr_lens_pos = curr_lens_pos;
	reg_setting.reg_setting = a_ctrl->i2c_reg_tbl;
	reg_setting.data_type = a_ctrl->i2c_data_type;
	reg_setting.size = a_ctrl->i2c_tbl_index;
#if defined(CONFIG_IMX135_GBAO_LC898122) || defined(CONFIG_IMX214_APP)|| defined(CONFIG_IMX214_OIS_SHARP)|| defined(CONFIG_IMX214_LC898122)

#if defined(CONFIG_IMX214_LC898122)|| defined(CONFIG_IMX214_OIS_SHARP)|| defined(CONFIG_IMX135_GBAO_LC898122)
	if (a_ctrl->i2c_client.cci_client->sid == 0x48 >> 1) {	
		msm_af_reg_write_lc898122(0x0304,a_ctrl->i2c_reg_tbl->reg_addr);
		msm_af_reg_write_lc898122(0x0305,a_ctrl->i2c_reg_tbl->reg_data);
		//printk(" reg_addr =0x%x,reg_data =0x%x\n",a_ctrl->i2c_reg_tbl->reg_addr,a_ctrl->i2c_reg_tbl->reg_data);
		
	}
#endif


#ifdef CONFIG_IMX214_APP
	if (a_ctrl->i2c_client.cci_client->sid == 0x32 >> 1) {
		rc = a_ctrl->i2c_client.i2c_func_tbl->z7_i2c_write_seq_microdelay(
			&a_ctrl->i2c_client,
			a_ctrl->i2c_reg_tbl, a_ctrl->i2c_tbl_index,
			a_ctrl->i2c_data_type);
	}
#endif
	if ((a_ctrl->i2c_client.cci_client->sid != 0x48 >> 1) && (a_ctrl->i2c_client.cci_client->sid != 0x32 >> 1)) {
		rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_table_w_microdelay(
			&a_ctrl->i2c_client, &reg_setting);

		if (rc < 0) {
			pr_err("i2c write error:%d\n", rc);
			return rc;
		}
	}
#else
	rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_table_w_microdelay(
		&a_ctrl->i2c_client, &reg_setting);
	if (rc < 0) {
		pr_err("i2c write error:%d\n", rc);
		return rc;
	}
#endif
	a_ctrl->i2c_tbl_index = 0;
	CDBG("Exit\n");

	return rc;
}

static int32_t msm_actuator_init_step_table(struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_info_t *set_info)
{
	int16_t code_per_step = 0;
	int16_t cur_code = 0;
	uint16_t step_index = 0, region_index = 0;
	uint16_t step_boundary = 0;
	uint32_t max_code_size = 1;
	uint16_t data_size = set_info->actuator_params.data_size;
	//int16_t i;
	CDBG("Enter\n");

	for (; data_size > 0; data_size--)
		max_code_size *= 2;

	kfree(a_ctrl->step_position_table);
	a_ctrl->step_position_table = NULL;

	if (set_info->af_tuning_params.total_steps
		>  MAX_ACTUATOR_AF_TOTAL_STEPS) {
		pr_err("%s: Max actuator totalsteps exceeded = %d\n",
		__func__, set_info->af_tuning_params.total_steps);
		return -EFAULT;
	}
	/* Fill step position table */
	a_ctrl->step_position_table =
		kmalloc(sizeof(uint16_t) *
		(set_info->af_tuning_params.total_steps + 1), GFP_KERNEL);

	if (a_ctrl->step_position_table == NULL)
		return -ENOMEM;
#if defined(CONFIG_IMX135_GBAO) || defined(CONFIG_IMX135_GBAO_LC898122) || defined(CONFIG_IMX135_Z5S)|| defined(CONFIG_IMX214_OIS_SHARP) 

	/*ZTEMT: Jinghongliang Add for Read AF OTP	---Start*/
#if defined(CONFIG_IMX135_GBAO)  || defined(CONFIG_IMX135_Z5S)
	printk("af_start_current = %d\n",af_start_value);
	printk("af_infinity_current = %d\n",af_infinity_value);
	printk("af_macro_current = %d\n",af_macro_value);
	//printk("actuator i2c addr = 0x%x\n",set_info->actuator_params.i2c_addr);
	if((set_info->actuator_params.i2c_addr == 0x1c) && (af_start_value > 0) && (af_infinity_value > af_start_value) \
		&& (af_macro_value > af_infinity_value)){
		cur_code = af_start_value*2/3;
		//cur_code = af_start_value - 100 > 0 ? af_start_value - 100 : 30;
		a_ctrl->step_position_table[step_index++] = cur_code;
		for (region_index = 0;region_index < a_ctrl->region_size;region_index++) {
			step_boundary =a_ctrl->region_params[region_index].step_bound[MOVE_NEAR];
			if(region_index == 0){
				code_per_step = (af_infinity_value - af_start_value)/step_boundary;
				code_per_step = 1;
				//printk("code_per_step_1 = %d\n",code_per_step);
				}
			if(region_index == 1){
				#if 0
				code_per_step = (af_macro_value - af_infinity_value)*10;
				code_per_step = code_per_step/40;
				//printk("code_per_step_211 = %d\n",code_per_step);
				code_per_step = (code_per_step + 5)/10;
				#endif
				code_per_step = 1;
				//printk("code_per_step_222 = %d\n",code_per_step);
				}
			for (; step_index <= step_boundary;step_index++) {
				cur_code += code_per_step;
				if (cur_code < max_code_size){
					a_ctrl->step_position_table[step_index] =
						cur_code;
					}else {
					for (; step_index <
						set_info->af_tuning_params.total_steps;
						step_index++)
						a_ctrl->
							step_position_table[
							step_index] =
							max_code_size;
				}
			}
		}
	}
	if ((set_info->actuator_params.i2c_addr == 0x1c)&& ((af_start_value == 0) || (af_infinity_value == 0) || (af_macro_value == 0))) {
		cur_code = set_info->af_tuning_params.initial_code;
		a_ctrl->step_position_table[step_index++] = cur_code;
		for (region_index = 0;
			region_index < a_ctrl->region_size;
			region_index++) {
			code_per_step =
				a_ctrl->region_params[region_index].code_per_step;
			step_boundary =
				a_ctrl->region_params[region_index].
				step_bound[MOVE_NEAR];
			for (; step_index <= step_boundary;
				step_index++) {
				cur_code += code_per_step;
				if (cur_code < max_code_size)
					a_ctrl->step_position_table[step_index] =
						cur_code;
				else {
					for (; step_index <
						set_info->af_tuning_params.total_steps;
						step_index++)
						a_ctrl->
							step_position_table[
							step_index] =
							max_code_size;
				}
			}
		}
	}
#endif
#ifdef CONFIG_IMX135_GBAO_LC898122
	printk("af_start_value_lc898122 = %d\n",af_start_value_lc898122);
	printk("af_infinity_value_lc898122 = %d\n",af_infinity_value_lc898122);
	printk("af_macro_value_lc898122 = %d\n",af_macro_value_lc898122);

	if((set_info->actuator_params.i2c_addr == 0x48) && af_infinity_value_lc898122 > 0 \
		&& af_start_value_lc898122 > 0 && af_macro_value_lc898122 > 0){
		cur_code = af_start_value_lc898122*2/3;
		if (cur_code >= af_infinity_value_lc898122)
				cur_code = af_infinity_value_lc898122*39/50;
		a_ctrl->step_position_table[step_index++] = cur_code;
		for (region_index = 0;region_index < a_ctrl->region_size;region_index++) {
			step_boundary =a_ctrl->region_params[region_index].step_bound[MOVE_NEAR];
			if(region_index == 0){
				code_per_step = (af_infinity_value_lc898122 - cur_code)/step_boundary;
				
				code_per_step = 1;
				//printk(" code_per_step_1 = %d\n",code_per_step);
				}
			if(region_index == 1){
				code_per_step = (af_macro_value_lc898122 - af_infinity_value_lc898122)*10;
				code_per_step = code_per_step/40;
				//printk(" code_per_step_211 = %d\n",code_per_step);
				code_per_step = (code_per_step + 5)/10;
				
				code_per_step = 1;
				//printk(" code_per_step_222 = %d\n",code_per_step);
				}
			for (; step_index <= step_boundary;step_index++) {
				cur_code += code_per_step;
				if (cur_code < max_code_size){
					a_ctrl->step_position_table[step_index] =
						cur_code;
					}else {
					for (; step_index <
						set_info->af_tuning_params.total_steps;
						step_index++)
						a_ctrl->
							step_position_table[
							step_index] =
							max_code_size;
				}
			}
		}
	}
	if ((set_info->actuator_params.i2c_addr == 0x48)&&((af_infinity_value_lc898122 == 0) || (af_macro_value_lc898122 == 0) || (af_start_value_lc898122 == 0)) \
		    && ((af_infinity_value_lc898122_sharp == 0) || (af_macro_value_lc898122_sharp == 0)|| (af_start_value_lc898122_sharp == 0))) {
		cur_code = set_info->af_tuning_params.initial_code;
		a_ctrl->step_position_table[step_index++] = cur_code;
		for (region_index = 0;
			region_index < a_ctrl->region_size;
			region_index++) {
			code_per_step =
				a_ctrl->region_params[region_index].code_per_step;
			step_boundary =
				a_ctrl->region_params[region_index].
				step_bound[MOVE_NEAR];
			for (; step_index <= step_boundary;
				step_index++) {
				cur_code += code_per_step;
				if (cur_code < max_code_size)
					a_ctrl->step_position_table[step_index] =
						cur_code;
				else {
					for (; step_index <
						set_info->af_tuning_params.total_steps;
						step_index++)
						a_ctrl->
							step_position_table[
							step_index] =
							max_code_size;
				}
			}
		}
	}
#endif
#ifdef CONFIG_IMX214_OIS_SHARP
	printk(" af_start_value_lc898122_sharp = %d\n",af_start_value_lc898122_sharp);
	printk(" af_infinity_value_lc898122_sharp = %d\n",af_infinity_value_lc898122_sharp);
	printk(" af_macro_value_lc898122_sharp = %d\n",af_macro_value_lc898122_sharp);

	if((set_info->actuator_params.i2c_addr == 0x48) && af_infinity_value_lc898122_sharp > 0 \
		&& af_start_value_lc898122_sharp > 0 && af_macro_value_lc898122_sharp > 0){

		int16_t total_code = 0;
		int16_t adjust_code = 0;
		int16_t offset_code = 0;
		cur_code = af_start_value_lc898122_sharp - 40;
		a_ctrl->step_position_table[step_index++] = cur_code;
		for (region_index = 0;region_index < a_ctrl->region_size;region_index++) {
			step_boundary =a_ctrl->region_params[region_index].step_bound[MOVE_NEAR];
			if(region_index == 0){
				code_per_step = (af_infinity_value_lc898122_sharp - cur_code)/step_boundary;
				//printk(" code_per_step_1 = %d\n",code_per_step);
				}
			if(region_index == 1){
				code_per_step = (af_macro_value_lc898122_sharp - af_infinity_value_lc898122_sharp)*10;
				code_per_step = code_per_step/40;
				code_per_step = (code_per_step + 5)/10;
				//ZTEMT:jixd af opt code optimization begin
				total_code = (af_macro_value_lc898122_sharp - af_infinity_value_lc898122_sharp)*
					(a_ctrl->region_params[1].step_bound[MOVE_NEAR]-a_ctrl->region_params[0].step_bound[MOVE_NEAR])/40;
				adjust_code = code_per_step*(a_ctrl->region_params[1].step_bound[MOVE_NEAR]-a_ctrl->region_params[0].step_bound[MOVE_NEAR]);
				offset_code = total_code - adjust_code;
				printk(" jxdadd_0102,code_per_step_222 = %d,%d,%d,%d\n",code_per_step,total_code,adjust_code,offset_code);
				//ZTEMT:jixd af opt code optimization end

			}
			for (; step_index <= step_boundary;step_index++) {

				//ZTEMT:jixd af opt code optimization begin
				if((region_index == 1)&&(offset_code > 0)
					 &&(step_index > a_ctrl->region_params[1].step_bound[MOVE_NEAR] - offset_code))
			    {
					cur_code = cur_code + code_per_step + 1;
			    }
				else if((region_index == 1)&&(offset_code < 0) 
					&&(step_index <= a_ctrl->region_params[0].step_bound[MOVE_NEAR] - offset_code))
				{
					cur_code = cur_code + code_per_step - 1;
				}
				else
				{
					cur_code += code_per_step;
				}
				printk(" jxdadd_0102_2,index = %d,code=%d\n",step_index,cur_code);
				//ZTEMT:jixd af opt code optimization end


				if (cur_code < max_code_size){
					a_ctrl->step_position_table[step_index] =
						cur_code;
			    }
			    else {
					for (; step_index <
						set_info->af_tuning_params.total_steps;
						step_index++)
						a_ctrl->
							step_position_table[
							step_index] =
							max_code_size;
				}
			}
		}
	}
	if ((set_info->actuator_params.i2c_addr == 0x48)&&((af_infinity_value_lc898122_sharp == 0) || (af_macro_value_lc898122_sharp == 0)|| (af_start_value_lc898122_sharp == 0)) \
		 && ((af_infinity_value_lc898122 == 0) || (af_macro_value_lc898122 == 0) || (af_start_value_lc898122 == 0))) {
		cur_code = set_info->af_tuning_params.initial_code;
		a_ctrl->step_position_table[step_index++] = cur_code;
		for (region_index = 0;
			region_index < a_ctrl->region_size;
			region_index++) {
			code_per_step =
				a_ctrl->region_params[region_index].code_per_step;
			step_boundary =
				a_ctrl->region_params[region_index].
				step_bound[MOVE_NEAR];
			for (; step_index <= step_boundary;
				step_index++) {
				cur_code += code_per_step;
				if (cur_code < max_code_size)
					a_ctrl->step_position_table[step_index] =
						cur_code;
				else {
					for (; step_index <
						set_info->af_tuning_params.total_steps;
						step_index++)
						a_ctrl->
							step_position_table[
							step_index] =
							max_code_size;
				}
			}
		}
	}
#endif

	if ((set_info->actuator_params.i2c_addr != 0x1c) && (set_info->actuator_params.i2c_addr != 0x48)) {
		cur_code = set_info->af_tuning_params.initial_code;
		a_ctrl->step_position_table[step_index++] = cur_code;
		for (region_index = 0;
			region_index < a_ctrl->region_size;
			region_index++) {
			code_per_step =
				a_ctrl->region_params[region_index].code_per_step;
			step_boundary =
				a_ctrl->region_params[region_index].
				step_bound[MOVE_NEAR];
			for (; step_index <= step_boundary;
				step_index++) {
				cur_code += code_per_step;
				if (cur_code < max_code_size)
					a_ctrl->step_position_table[step_index] =
						cur_code;
				else {
					for (; step_index <
						set_info->af_tuning_params.total_steps;
						step_index++)
						a_ctrl->
							step_position_table[
							step_index] =
							max_code_size;
				}
			}
		}
	}
#else
	cur_code = set_info->af_tuning_params.initial_code;
    #if defined(CONFIG_ZTE_CAMERA_NX505J) || defined(CONFIG_ZTE_CAMERA_NX507J)|| defined(CONFIG_ZTE_CAMERA_NX504J)
		if(af_infinity_value_imx214 > 99 && af_infinity_value_imx214 < 401){
		cur_code = af_infinity_value_imx214 - IMX214_AF_OFFSET;
		if(cur_code < IMX214_MIN_INITIAL_CODE)
			cur_code = IMX214_MIN_INITIAL_CODE;
		}
    #endif
	a_ctrl->step_position_table[step_index++] = cur_code;
	for (region_index = 0;
		region_index < a_ctrl->region_size;
		region_index++) {
		code_per_step =
			a_ctrl->region_params[region_index].code_per_step;
		step_boundary =
			a_ctrl->region_params[region_index].
			step_bound[MOVE_NEAR];
		if (step_boundary >
			set_info->af_tuning_params.total_steps) {
			pr_err("invalid step_boundary = %d, max_val = %d",
				step_boundary,
				set_info->af_tuning_params.total_steps);
			kfree(a_ctrl->step_position_table);
			a_ctrl->step_position_table = NULL;
			return -EINVAL;
		}
		for (; step_index <= step_boundary;
			step_index++) {
			cur_code += code_per_step;
			if (cur_code < max_code_size)
				a_ctrl->step_position_table[step_index] =
					cur_code;
			else {
				for (; step_index <
					set_info->af_tuning_params.total_steps;
					step_index++)
					a_ctrl->
						step_position_table[
						step_index] =
						max_code_size;
			}
		}
	}
#endif

	//for(i=0;i<set_info->af_tuning_params.total_steps;i++)
	// printk("%s:jun] table[%d]=%d\n",__func__, i,a_ctrl->step_position_table[i]);

	/*ZTEMT: Jinghongliang Add for Read AF OTP	---end*/
	CDBG("Exit\n");
	return 0;
}

static int32_t msm_actuator_set_default_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t rc = 0;
	//int32_t step_boundary= a_ctrl->region_params[0].step_bound[MOVE_NEAR];
	CDBG("Enter\n");
	
	#if 0  //jun add for avoid click sound when exit from camera
	pr_err("%s[jun]Enter, step_boundary=%d,\n",__func__,step_boundary);
	if (a_ctrl->curr_step_pos != 0) {
		move_params->dest_step_pos = step_boundary;
		rc = a_ctrl->func_tbl->actuator_move_focus(a_ctrl, move_params);
		//mdelay(10);
		move_params->dest_step_pos = 0;
		rc = a_ctrl->func_tbl->actuator_move_focus(a_ctrl, move_params);
		}
	pr_err("%s[jun]Exit\n",__func__);
	#else

	if (a_ctrl->curr_step_pos != 0)
	    rc = a_ctrl->func_tbl->actuator_move_focus(a_ctrl, move_params);

	//ZTEMT:jixd for lc898122 begin
	#ifdef CONFIG_IMX214_OIS_SHARP
	else
	{
	    msm_af_reg_write_lc898122(0x0304,0x07&(a_ctrl->step_position_table[0]>>8));
	    msm_af_reg_write_lc898122(0x0305,a_ctrl->step_position_table[0]&0xff);
	}
	#endif /*CONFIG_IMX214_OIS_SHARP*/
	//ZTEMT:jixd for lc898122 end
	#endif
	
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_actuator_power_down(struct msm_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	if (a_ctrl->vcm_enable) {
		rc = gpio_direction_output(a_ctrl->vcm_pwd, 0);
		if (!rc)
			gpio_free(a_ctrl->vcm_pwd);
	}

	kfree(a_ctrl->step_position_table);
	a_ctrl->step_position_table = NULL;
	kfree(a_ctrl->i2c_reg_tbl);
	a_ctrl->i2c_reg_tbl = NULL;
	a_ctrl->i2c_tbl_index = 0;
	#ifdef CONFIG_IMX135_GBAO_LC898122
	if (a_ctrl->i2c_client.cci_client->sid == 0x48 >> 1) {	
		msm_ois_release_cci_lc898122();
	}
	#endif
	
	#ifdef CONFIG_IMX214_OIS_SHARP
	if (a_ctrl->i2c_client.cci_client->sid == 0x48 >> 1) {	
		msm_ois_release_cci_lc898122_sharp();
	}
	#endif

	CDBG("Exit\n");
	return rc;
}

static int32_t msm_actuator_set_position(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_position_t *set_pos)
{
	int32_t rc = 0;
	int32_t index;
	uint16_t next_lens_position;
	uint16_t delay;
	#if defined(CONFIG_ZTE_CAMERA_NX505J) || defined(CONFIG_ZTE_CAMERA_NX507J) || defined(CONFIG_ZTE_CAMERA_NX504J)
    uint32_t hw_params = 0xF400;
    uint16_t value=0;
       #elif defined(CONFIG_ZTE_CAMERA_NX506J)
        unsigned short sharp_af_manual_step = 0;
        uint16_t value=0;
        uint32_t hw_params = 0;
    #else
    uint16_t value=0;
    uint32_t hw_params = 0;
    #endif
	struct msm_camera_i2c_reg_setting reg_setting;
	CDBG("%s Enter %d\n", __func__, __LINE__);
	if (set_pos->number_of_steps  == 0)
		return rc;

	a_ctrl->i2c_tbl_index = 0;
	for (index = 0; index < set_pos->number_of_steps; index++) {
		next_lens_position = set_pos->pos[index];
		delay = set_pos->delay[index];
		
        value = set_pos->pos[index];
        if(value < 0 || value > 79){     /* if over total steps*/
            pr_err("%s Failed I2C write Line %d\n", __func__, __LINE__);
                return rc;
         }
 
        value = 79 - value;

#if defined(CONFIG_ZTE_CAMERA_NX505J) || defined(CONFIG_ZTE_CAMERA_NX507J)|| defined(CONFIG_ZTE_CAMERA_NX504J)
        if(value == 79){
		   next_lens_position = 900;
           a_ctrl->curr_step_pos = a_ctrl->region_params[a_ctrl->region_size - 1].step_bound[MOVE_NEAR] - 3;
        }else{
            next_lens_position = a_ctrl->step_position_table[0] + 7*value;
			a_ctrl->curr_step_pos = next_lens_position - a_ctrl->step_position_table[0];
        }
#elif defined(CONFIG_ZTE_CAMERA_NX506J)
       if(value == 79){
        next_lens_position = a_ctrl->step_position_table[a_ctrl->total_steps-1];
        a_ctrl->curr_step_pos = a_ctrl->region_params[a_ctrl->region_size - 1].step_bound[MOVE_NEAR] - 3;
	 }else{
        sharp_af_manual_step = (af_macro_value_lc898122_sharp - (af_start_value_lc898122_sharp-30))/70;
        next_lens_position = (af_start_value_lc898122_sharp-30) + sharp_af_manual_step*value;
	 a_ctrl->curr_step_pos = next_lens_position - (af_start_value_lc898122_sharp-30);
	 }
#else
        if(value == 79){
           next_lens_position = a_ctrl->step_position_table[0] + a_ctrl->region_params[a_ctrl->region_size - 1].step_bound[MOVE_NEAR] - 3;
           //pr_err("%s jun boundy = %d,\n", __func__, a_ctrl->region_params[a_ctrl->region_size - 1].step_bound[MOVE_NEAR]);
        }else{
            next_lens_position = a_ctrl->step_position_table[0] + 7*value;
        }
                        
        a_ctrl->curr_step_pos = next_lens_position - a_ctrl->step_position_table[0];
#endif

        pr_err("%s jun next_lens_position = %d, cur_step_pos=%d\n", __func__, next_lens_position,a_ctrl->curr_step_pos);
      //add end                       
		a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
		next_lens_position, hw_params, delay);

		reg_setting.reg_setting = a_ctrl->i2c_reg_tbl;
		reg_setting.size = a_ctrl->i2c_tbl_index;
		reg_setting.data_type = a_ctrl->i2c_data_type;
#if defined(CONFIG_IMX135_GBAO_LC898122)|| defined(CONFIG_IMX214_OIS_SHARP)
		if (a_ctrl->i2c_client.cci_client->sid == 0x48 >> 1) {	
			msm_af_reg_write_lc898122(0x0304,a_ctrl->i2c_reg_tbl->reg_addr);
			msm_af_reg_write_lc898122(0x0305,a_ctrl->i2c_reg_tbl->reg_data);
			//printk(" reg_addr =0x%x,reg_data =0x%x\n",a_ctrl->i2c_reg_tbl->reg_addr,a_ctrl->i2c_reg_tbl->reg_data);
			
		}
		if (a_ctrl->i2c_client.cci_client->sid != 0x48 >> 1) {	
			rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_table_w_microdelay(
			&a_ctrl->i2c_client, &reg_setting);
			if (rc < 0) {
				pr_err("%s Failed I2C write Line %d\n", __func__, __LINE__);
				return rc;
			}
		}
#else
		rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_table_w_microdelay(
			&a_ctrl->i2c_client, &reg_setting);
		if (rc < 0) {
			pr_err("%s Failed I2C write Line %d\n", __func__, __LINE__);
			return rc;
		}
#endif
		a_ctrl->i2c_tbl_index = 0;
	}
	CDBG("%s exit %d\n", __func__, __LINE__);
	return rc;
}

static int32_t msm_actuator_init(struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_info_t *set_info) {
	struct reg_settings_t *init_settings = NULL;
	int32_t rc = -EFAULT;
	uint16_t i = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	CDBG("Enter\n");

	for (i = 0; i < ARRAY_SIZE(actuators); i++) {
		if (set_info->actuator_params.act_type ==
			actuators[i]->act_type) {
			a_ctrl->func_tbl = &actuators[i]->func_tbl;
			rc = 0;
		}
	}

	if (rc < 0) {
		pr_err("Actuator function table not found\n");
		return rc;
	}
	if (set_info->af_tuning_params.total_steps
		>  MAX_ACTUATOR_AF_TOTAL_STEPS) {
		pr_err("%s: Max actuator totalsteps exceeded = %d\n",
		__func__, set_info->af_tuning_params.total_steps);
		return -EFAULT;
	}
	if (set_info->af_tuning_params.region_size
		> MAX_ACTUATOR_REGION) {
		pr_err("MAX_ACTUATOR_REGION is exceeded.\n");
		return -EFAULT;
	}

	a_ctrl->region_size = set_info->af_tuning_params.region_size;
	a_ctrl->pwd_step = set_info->af_tuning_params.pwd_step;
	a_ctrl->total_steps = set_info->af_tuning_params.total_steps;

	if (copy_from_user(&a_ctrl->region_params,
		(void *)set_info->af_tuning_params.region_params,
		a_ctrl->region_size * sizeof(struct region_params_t)))
		return -EFAULT;

	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		cci_client = a_ctrl->i2c_client.cci_client;
		cci_client->sid =
			set_info->actuator_params.i2c_addr >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->cci_i2c_master = a_ctrl->cci_master;
	} else {
		a_ctrl->i2c_client.client->addr =
			set_info->actuator_params.i2c_addr;
	}

	a_ctrl->i2c_data_type = set_info->actuator_params.i2c_data_type;
	a_ctrl->i2c_client.addr_type = set_info->actuator_params.i2c_addr_type;
	if (set_info->actuator_params.reg_tbl_size <=
		MAX_ACTUATOR_REG_TBL_SIZE) {
		a_ctrl->reg_tbl_size = set_info->actuator_params.reg_tbl_size;
	} else {
		a_ctrl->reg_tbl_size = 0;
		pr_err("MAX_ACTUATOR_REG_TBL_SIZE is exceeded.\n");
		return -EFAULT;
	}

	kfree(a_ctrl->i2c_reg_tbl);

	a_ctrl->i2c_reg_tbl =
		kmalloc(sizeof(struct msm_camera_i2c_reg_array) *
		(set_info->af_tuning_params.total_steps + 1), GFP_KERNEL);
	if (!a_ctrl->i2c_reg_tbl) {
		pr_err("kmalloc fail\n");
		return -ENOMEM;
	}

	if (copy_from_user(&a_ctrl->reg_tbl,
		(void *)set_info->actuator_params.reg_tbl_params,
		a_ctrl->reg_tbl_size *
		sizeof(struct msm_actuator_reg_params_t))) {
		kfree(a_ctrl->i2c_reg_tbl);
		return -EFAULT;
	}

	if (set_info->actuator_params.init_setting_size &&
		set_info->actuator_params.init_setting_size
		<= MAX_ACTUATOR_REG_TBL_SIZE) {
		if (a_ctrl->func_tbl->actuator_init_focus) {
			init_settings = kmalloc(sizeof(struct reg_settings_t) *
				(set_info->actuator_params.init_setting_size),
				GFP_KERNEL);
			if (init_settings == NULL) {
				kfree(a_ctrl->i2c_reg_tbl);
				pr_err("Error allocating memory for init_settings\n");
				return -EFAULT;
			}
			if (copy_from_user(init_settings,
				(void *)set_info->actuator_params.init_settings,
				set_info->actuator_params.init_setting_size *
				sizeof(struct reg_settings_t))) {
				kfree(init_settings);
				kfree(a_ctrl->i2c_reg_tbl);
				pr_err("Error copying init_settings\n");
				return -EFAULT;
			}
			rc = a_ctrl->func_tbl->actuator_init_focus(a_ctrl,
				set_info->actuator_params.init_setting_size,
				a_ctrl->i2c_data_type,
				init_settings);
			kfree(init_settings);
			if (rc < 0) {
				kfree(a_ctrl->i2c_reg_tbl);
				pr_err("Error actuator_init_focus\n");
				return -EFAULT;
			}
		}
	}

	a_ctrl->initial_code = set_info->af_tuning_params.initial_code;
	if (a_ctrl->func_tbl->actuator_init_step_table)
		rc = a_ctrl->func_tbl->
			actuator_init_step_table(a_ctrl, set_info);

	a_ctrl->curr_step_pos = 0;
	a_ctrl->curr_region_index = 0;
	CDBG("Exit\n");

	return rc;
}

static int32_t msm_actuator_config(struct msm_actuator_ctrl_t *a_ctrl,
	void __user *argp)
{
	struct msm_actuator_cfg_data *cdata =
		(struct msm_actuator_cfg_data *)argp;
	int32_t rc = 0;
	mutex_lock(a_ctrl->actuator_mutex);
	CDBG("Enter\n");
	CDBG("%s type %d\n", __func__, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_ACTUATOR_INFO:
		cdata->is_af_supported = 1;
		cdata->cfg.cam_name = a_ctrl->cam_name;
		break;

	case CFG_SET_ACTUATOR_INFO:
		rc = msm_actuator_init(a_ctrl, &cdata->cfg.set_info);
		if (rc < 0)
			pr_err("init table failed %d\n", rc);
		break;

	case CFG_SET_DEFAULT_FOCUS:
		rc = a_ctrl->func_tbl->actuator_set_default_focus(a_ctrl,
			&cdata->cfg.move);
		if (rc < 0)
			pr_err("move focus failed %d\n", rc);
		break;

	case CFG_MOVE_FOCUS:
		rc = a_ctrl->func_tbl->actuator_move_focus(a_ctrl,
			&cdata->cfg.move);
		if (rc < 0)
			pr_err("move focus failed %d\n", rc);
		break;

	case CFG_SET_POSITION:
		rc = a_ctrl->func_tbl->actuator_set_position(a_ctrl,
			&cdata->cfg.setpos);
		if (rc < 0)
			pr_err("actuator_set_position failed %d\n", rc);
		break;
	default:
		break;
	}
	mutex_unlock(a_ctrl->actuator_mutex);
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_actuator_get_subdev_id(struct msm_actuator_ctrl_t *a_ctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	CDBG("Enter\n");
	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		*subdev_id = a_ctrl->pdev->id;
	else
		*subdev_id = a_ctrl->subdev_id;

	CDBG("subdev_id %d\n", *subdev_id);
	CDBG("Exit\n");
	return 0;
}

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
#ifdef CONFIG_IMX214_APP
	.z7_i2c_write_seq_microdelay = 
		z7_msm_camera_cci_i2c_write_seq_microdelay,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
#endif
};

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
};

static int msm_actuator_open(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_actuator_ctrl_t *a_ctrl =  v4l2_get_subdevdata(sd);
	pr_err("Enter\n");
	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_util(
			&a_ctrl->i2c_client, MSM_CCI_INIT);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}
	pr_err("Exit\n");
	return rc;
}

static int msm_actuator_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_actuator_ctrl_t *a_ctrl =  v4l2_get_subdevdata(sd);
	pr_err("Enter\n");
	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_util(
			&a_ctrl->i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}
	kfree(a_ctrl->i2c_reg_tbl);
	a_ctrl->i2c_reg_tbl = NULL;

	pr_err("Exit\n");
	return rc;
}

static const struct v4l2_subdev_internal_ops msm_actuator_internal_ops = {
	.open = msm_actuator_open,
	.close = msm_actuator_close,
};

static long msm_actuator_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	struct msm_actuator_ctrl_t *a_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;
	CDBG("Enter\n");
	CDBG("%s:%d a_ctrl %p argp %p\n", __func__, __LINE__, a_ctrl, argp);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_actuator_get_subdev_id(a_ctrl, argp);
	case VIDIOC_MSM_ACTUATOR_CFG:
		return msm_actuator_config(a_ctrl, argp);
	case MSM_SD_SHUTDOWN:
		msm_actuator_close(sd, NULL);
		return 0;
	default:
		return -ENOIOCTLCMD;
	}
}

static int32_t msm_actuator_power_up(struct msm_actuator_ctrl_t *a_ctrl)
{
	int rc = 0;
	CDBG("%s called\n", __func__);

	CDBG("vcm info: %x %x\n", a_ctrl->vcm_pwd,
		a_ctrl->vcm_enable);
	if (a_ctrl->vcm_enable) {
		rc = gpio_request(a_ctrl->vcm_pwd, "msm_actuator");
		if (!rc) {
			CDBG("Enable VCM PWD\n");
			gpio_direction_output(a_ctrl->vcm_pwd, 1);
		}
	}
	#ifdef CONFIG_IMX135_GBAO_LC898122
	
	if (a_ctrl->i2c_client.cci_client->sid == 0x48 >> 1) {	
		msm_ois_init_cci_lc898122();
	}
	#endif
	
	#ifdef CONFIG_IMX214_OIS_SHARP
	if (a_ctrl->i2c_client.cci_client->sid == 0x48 >> 1) {	
		msm_ois_init_cci_lc898122_sharp();
	}
	#endif

	CDBG("Exit\n");
	return rc;
}

static int32_t msm_actuator_power(struct v4l2_subdev *sd, int on)
{
	int rc = 0;
	struct msm_actuator_ctrl_t *a_ctrl = v4l2_get_subdevdata(sd);
	CDBG("Enter\n");
	mutex_lock(a_ctrl->actuator_mutex);
	if (on)
		rc = msm_actuator_power_up(a_ctrl);
	else
		rc = msm_actuator_power_down(a_ctrl);
	mutex_unlock(a_ctrl->actuator_mutex);
	CDBG("Exit\n");
	return rc;
}

static struct v4l2_subdev_core_ops msm_actuator_subdev_core_ops = {
	.ioctl = msm_actuator_subdev_ioctl,
	.s_power = msm_actuator_power,
};

static struct v4l2_subdev_ops msm_actuator_subdev_ops = {
	.core = &msm_actuator_subdev_core_ops,
};

static const struct i2c_device_id msm_actuator_i2c_id[] = {
	{"qcom,actuator", (kernel_ulong_t)NULL},
	{ }
};
#ifdef CONFIG_IMX214_APP
void RegRead8byte(uint16_t reg_addr, struct msm_actuator_ctrl_t *a_ctrl)
{
	uint8_t data[8];
	int32_t rc=0;
	memset(data, 0x00, 8);
	rc =  a_ctrl->i2c_client.i2c_func_tbl->i2c_write_seq
	(
	   &a_ctrl->i2c_client,
	   reg_addr,
	   data,
	   8);
	if (rc < 0) {
	   pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	}
	rc =  a_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq
	 (
		&a_ctrl->i2c_client,
		reg_addr, &data[0],
		8);
	if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	}
}

int32_t Regwrite8byte(uint16_t reg_addr, unsigned long write_data1_32

	, unsigned long write_data2_32, struct msm_actuator_ctrl_t *a_ctrl)
{
	uint8_t data[8];
	int32_t rc=0;
	memset(data, 0x00, 8);
	data[0] = (write_data1_32 >> 24) &0xFF;
	data[1] = (write_data1_32 >> 16) &0xFF;

	data[2] = (write_data1_32 >> 8) &0xFF;
	data[3] = (write_data1_32) &0xFF;
	
	data[4] = (write_data2_32 >> 24) &0xFF;
	data[5] = (write_data2_32 >> 16) &0xFF;

	data[6] = (write_data2_32 >> 8) &0xFF;
	data[7] = (write_data2_32) &0xFF;
	rc =  a_ctrl->i2c_client.i2c_func_tbl->i2c_write_seq
	(
	   &a_ctrl->i2c_client,
	   reg_addr,
	   data,
	   8);
	if (rc < 0) {
	   pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	}
	return rc;
}

EXPORT_SYMBOL(RegRead8byte);
EXPORT_SYMBOL(Regwrite8byte);

static struct msm_actuator_ctrl_t *zte_msm_actuator_t;
static ssize_t show_model(struct device *cd,
            struct device_attribute *attr, char *buf)
{
 //struct media_device *mdev = to_media_device(to_media_devnode(cd));
 //int a = 1;
 //printk("sss test\n");
   return 0;//sprintf(buf, "%d", (int)sizeof(int), a);
 }
 static ssize_t store_model(struct device *class_dev,
                  struct device_attribute *attr,
                 const char *buf, size_t count)
 {
 	//ssize_t ret = 0;
 	int val;
    unsigned long init = 0x20000000;
    static unsigned long init_add = 0;
    unsigned long temp;
    sscanf(buf, "%d", &val);
	init_add= init_add +1;
	temp = init | (init_add << 16);
    pr_err("sss %s init_add=%x,state=%d size=%d temp=%lx\n", __func__, (int)init_add, (int)val, (int)count, temp);
    Regwrite8byte(0x20, 0x08010103, temp, zte_msm_actuator_t); 
    return count;
 }
 static DEVICE_ATTR(model, 0644/*S_IRUGO*/, show_model, store_model);

#ifdef CONFIG_IMX214_APP
struct msm_actuator_ctrl_t msm_actuator_t_ofei_ois ;

enum msm_ofei_ois_data_type {
	MSM_OIS_BYTE_DATA = 1,
	MSM_OIS_WORD_DATA,
};

struct msm_ofei_ois_ctrl_t {
	struct msm_camera_i2c_client i2c_client;
	enum af_camera_name cam_name;
	enum msm_ofei_ois_data_type i2c_data_type;
	enum cci_i2c_master_t cci_master;
};

void msm_ofei_init_cci(void)
{
       int rc;
       rc = msm_actuator_t_ofei_ois.i2c_client.i2c_func_tbl->i2c_util(
               &msm_actuator_t_ofei_ois.i2c_client, MSM_CCI_INIT);

       if (rc < 0)
               printk("ztemt_ofei cci_init failed\n");

}

void msm_ofei_release_cci(void)
{
       int rc;
       rc = msm_actuator_t_ofei_ois.i2c_client.i2c_func_tbl->i2c_util(
               &msm_actuator_t_ofei_ois.i2c_client, MSM_CCI_RELEASE);

       if (rc < 0)
               printk("ztemt_ofei cci_init failed\n");

}

int32_t OfeiRegwrite8byte(uint16_t reg_addr, unsigned long write_data1_32
	, unsigned long write_data2_32)
{


	uint8_t data[8];
	int32_t rc=0;
		  
	memset(data, 0x00, 8);
	data[0] = (write_data1_32 >> 24) &0xFF;
	data[1] = (write_data1_32 >> 16) &0xFF;

	data[2] = (write_data1_32 >> 8) &0xFF;
	data[3] = (write_data1_32) &0xFF;
	
	data[4] = (write_data2_32 >> 24) &0xFF;
	data[5] = (write_data2_32 >> 16) &0xFF;

	data[6] = (write_data2_32 >> 8) &0xFF;
	data[7] = (write_data2_32) &0xFF;
	
       msm_ofei_init_cci();

	rc =  msm_actuator_t_ofei_ois.i2c_client.i2c_func_tbl->i2c_write_seq
	(
	   &msm_actuator_t_ofei_ois.i2c_client,
	   reg_addr,
	   data,
	   8);

       msm_ofei_release_cci();

	if (rc < 0) {
	   pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	}
	return rc;
}
void ofei_shut_down_af_mode(void)
{
       int rc = 0;
	   
	//debug for click sound
       rc = OfeiRegwrite8byte(0x01, 0x00000000, 0x00000000);   //shut down ois 
	msleep(100);
	rc = OfeiRegwrite8byte(0x15, 0x00640000, 0x00000000);
	msleep(50);
	rc = OfeiRegwrite8byte(0x01, 0xFF000000, 0x00000000);//shut down pwn
	msleep(200);
	
	printk("ztemt_ofei enter msm_actuator_power_down  rc = %d\n",rc);
	
}

EXPORT_SYMBOL(ofei_shut_down_af_mode);
EXPORT_SYMBOL(ofei_swtich_to_preview_mode);
#endif

#endif
static int32_t msm_actuator_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_actuator_ctrl_t *act_ctrl_t = NULL;
	CDBG("Enter\n");

	if (client == NULL) {
		pr_err("msm_actuator_i2c_probe: client is null\n");
		rc = -EINVAL;
		goto probe_failure;
	}

	act_ctrl_t = kzalloc(sizeof(struct msm_actuator_ctrl_t),
		GFP_KERNEL);
	if (!act_ctrl_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	CDBG("client = %x\n", (unsigned int) client);

	rc = of_property_read_u32(client->dev.of_node, "cell-index",
		&act_ctrl_t->subdev_id);
	CDBG("cell-index %d, rc %d\n", act_ctrl_t->subdev_id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	act_ctrl_t->i2c_driver = &msm_actuator_i2c_driver;
	act_ctrl_t->i2c_client.client = client;
	act_ctrl_t->curr_step_pos = 0,
	act_ctrl_t->curr_region_index = 0,
	/* Set device type as I2C */
	act_ctrl_t->act_device_type = MSM_CAMERA_I2C_DEVICE;
	act_ctrl_t->i2c_client.i2c_func_tbl = &msm_sensor_qup_func_tbl;
	act_ctrl_t->act_v4l2_subdev_ops = &msm_actuator_subdev_ops;
	act_ctrl_t->actuator_mutex = &msm_actuator_mutex;

	act_ctrl_t->cam_name = act_ctrl_t->subdev_id;
	CDBG("act_ctrl_t->cam_name: %d", act_ctrl_t->cam_name);
	/* Assign name for sub device */
	snprintf(act_ctrl_t->msm_sd.sd.name, sizeof(act_ctrl_t->msm_sd.sd.name),
		"%s", act_ctrl_t->i2c_driver->driver.name);

	/* Initialize sub device */
	v4l2_i2c_subdev_init(&act_ctrl_t->msm_sd.sd,
		act_ctrl_t->i2c_client.client,
		act_ctrl_t->act_v4l2_subdev_ops);
	v4l2_set_subdevdata(&act_ctrl_t->msm_sd.sd, act_ctrl_t);
	act_ctrl_t->msm_sd.sd.internal_ops = &msm_actuator_internal_ops;
	act_ctrl_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&act_ctrl_t->msm_sd.sd.entity, 0, NULL, 0);
	act_ctrl_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	act_ctrl_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_ACTUATOR;
	act_ctrl_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&act_ctrl_t->msm_sd);
	pr_info("msm_actuator_i2c_probe: succeeded\n");
	CDBG("Exit\n");

probe_failure:
	return rc;
}

static int32_t msm_actuator_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_actuator_ctrl_t *msm_actuator_t = NULL;

	#ifdef CONFIG_IMX214_APP
       struct msm_camera_cci_client *cci_client_ofei_ois = NULL;
       #endif

	CDBG("Enter\n");

	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	msm_actuator_t = kzalloc(sizeof(struct msm_actuator_ctrl_t),
		GFP_KERNEL);
	if (!msm_actuator_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	rc = of_property_read_u32((&pdev->dev)->of_node, "cell-index",
		&pdev->id);
	CDBG("cell-index %d, rc %d\n", pdev->id, rc);
	if (rc < 0) {
		kfree(msm_actuator_t);
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	rc = of_property_read_u32((&pdev->dev)->of_node, "qcom,cci-master",
		&msm_actuator_t->cci_master);
	CDBG("qcom,cci-master %d, rc %d\n", msm_actuator_t->cci_master, rc);
	if (rc < 0) {
		kfree(msm_actuator_t);
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	msm_actuator_t->act_v4l2_subdev_ops = &msm_actuator_subdev_ops;
	msm_actuator_t->actuator_mutex = &msm_actuator_mutex;
	msm_actuator_t->cam_name = pdev->id;

	/* Set platform device handle */
	msm_actuator_t->pdev = pdev;
	/* Set device type as platform device */
	msm_actuator_t->act_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	msm_actuator_t->i2c_client.i2c_func_tbl = &msm_sensor_cci_func_tbl;
	msm_actuator_t->i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!msm_actuator_t->i2c_client.cci_client) {
		kfree(msm_actuator_t);
		pr_err("failed no memory\n");
		return -ENOMEM;
	}

	cci_client = msm_actuator_t->i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = MASTER_MAX;


       #ifdef CONFIG_IMX214_APP
       msm_actuator_t_ofei_ois.cci_master = 0;
	msm_actuator_t_ofei_ois.cam_name = 0;

	msm_actuator_t_ofei_ois.i2c_client.i2c_func_tbl = &msm_sensor_cci_func_tbl;
	msm_actuator_t_ofei_ois.i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!msm_actuator_t_ofei_ois.i2c_client.cci_client) {
		pr_err("failed no memory\n");
		return -ENOMEM;
	}

	cci_client_ofei_ois = msm_actuator_t_ofei_ois.i2c_client.cci_client;
	cci_client_ofei_ois->cci_subdev = msm_cci_get_subdev();

	msm_actuator_t_ofei_ois.i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
       msm_actuator_t_ofei_ois.i2c_data_type = MSM_ACTUATOR_BYTE_DATA;

	msm_actuator_t_ofei_ois.i2c_client.cci_client->sid = 0x32 >> 1;
	msm_actuator_t_ofei_ois.i2c_client.cci_client->retries = 3;
	msm_actuator_t_ofei_ois.i2c_client.cci_client->id_map = 0;
	msm_actuator_t_ofei_ois.i2c_client.cci_client->cci_i2c_master = 0;
	#endif


	v4l2_subdev_init(&msm_actuator_t->msm_sd.sd,
		msm_actuator_t->act_v4l2_subdev_ops);
	v4l2_set_subdevdata(&msm_actuator_t->msm_sd.sd, msm_actuator_t);
	msm_actuator_t->msm_sd.sd.internal_ops = &msm_actuator_internal_ops;
	msm_actuator_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(msm_actuator_t->msm_sd.sd.name,
		ARRAY_SIZE(msm_actuator_t->msm_sd.sd.name), "msm_actuator");
	media_entity_init(&msm_actuator_t->msm_sd.sd.entity, 0, NULL, 0);
	msm_actuator_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	msm_actuator_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_ACTUATOR;
	msm_actuator_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&msm_actuator_t->msm_sd);
	#ifdef CONFIG_IMX214_APP
	if (!strncmp(dev_name(&pdev->dev),"32.qcom,actuator",sizeof(dev_name(&pdev->dev)))){
		zte_msm_actuator_t = msm_actuator_t;
		rc = device_create_file(&pdev->dev, &dev_attr_model);
    	if (rc < 0) {
      		printk("sss create error\n");
   		}
	}
	#endif
	CDBG("Exit\n");
	return rc;
}

static const struct of_device_id msm_actuator_i2c_dt_match[] = {
	{.compatible = "qcom,actuator"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_actuator_i2c_dt_match);

static struct i2c_driver msm_actuator_i2c_driver = {
	.id_table = msm_actuator_i2c_id,
	.probe  = msm_actuator_i2c_probe,
	.remove = __exit_p(msm_actuator_i2c_remove),
	.driver = {
		.name = "qcom,actuator",
		.owner = THIS_MODULE,
		.of_match_table = msm_actuator_i2c_dt_match,
	},
};

static const struct of_device_id msm_actuator_dt_match[] = {
	{.compatible = "qcom,actuator", .data = NULL},
	{}
};

MODULE_DEVICE_TABLE(of, msm_actuator_dt_match);

static struct platform_driver msm_actuator_platform_driver = {
	.driver = {
		.name = "qcom,actuator",
		.owner = THIS_MODULE,
		.of_match_table = msm_actuator_dt_match,
	},
};

static int __init msm_actuator_init_module(void)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	rc = platform_driver_probe(&msm_actuator_platform_driver,
		msm_actuator_platform_probe);
	if (!rc)
		return rc;
	CDBG("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&msm_actuator_i2c_driver);
}

static struct msm_actuator msm_vcm_actuator_table = {
	.act_type = ACTUATOR_VCM,
	.func_tbl = {
		.actuator_init_step_table = msm_actuator_init_step_table,
		.actuator_move_focus = msm_actuator_move_focus,
		.actuator_write_focus = msm_actuator_write_focus,
		.actuator_set_default_focus = msm_actuator_set_default_focus,
		.actuator_init_focus = msm_actuator_init_focus,
		.actuator_parse_i2c_params = msm_actuator_parse_i2c_params,
		.actuator_set_position = msm_actuator_set_position,
	},
};

static struct msm_actuator msm_piezo_actuator_table = {
	.act_type = ACTUATOR_PIEZO,
	.func_tbl = {
		.actuator_init_step_table = NULL,
		.actuator_move_focus = msm_actuator_piezo_move_focus,
		.actuator_write_focus = NULL,
		.actuator_set_default_focus =
			msm_actuator_piezo_set_default_focus,
		.actuator_init_focus = msm_actuator_init_focus,
		.actuator_parse_i2c_params = msm_actuator_parse_i2c_params,
	},
};

module_init(msm_actuator_init_module);
MODULE_DESCRIPTION("MSM ACTUATOR");
MODULE_LICENSE("GPL v2");
