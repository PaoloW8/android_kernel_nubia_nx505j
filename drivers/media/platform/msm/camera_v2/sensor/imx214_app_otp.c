#include "msm_sensor.h"
#include "msm_sd.h"
#include "camera.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#undef CDBG
#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

struct imx214_app_otp_struct
{
	int AWB_Year;
	int AWB_Monty;
	int AWB_Day;
	int AWB_MID;
	int AWB_VendorID;
	int AWB_Current_RGr;
	int AWB_Current_BGr;
	int AWB_Current_GbGr;
	int AWB_Page_Flag;
};


#define AWB_TYPICAL_RGr 0x023c   //golden sample R/Gr ratio
#define AWB_TYPICAL_BGr 0x0271   //golden sample B/Gr ratio
#define AWB_TYPICAL_GbGr 0x0400      //golden sample Gb/Gr ratio

int imx214_app_check_otp_wb(int Index, struct msm_sensor_ctrl_t *s_ctrl);
int imx214_app_update_wb_register_from_otp(struct msm_sensor_ctrl_t *s_ctrl);
int imx214_app_update_wb_gain(int R_Gain,int Gr_Gain,int B_Gain,int Gb_Gain, struct msm_sensor_ctrl_t *s_ctrl);
int imx214_app_read_otp_wb(int otp_wb_page,struct imx214_app_otp_struct *current_otp_wb, struct msm_sensor_ctrl_t *s_ctrl);
int imx214_app_read_test(struct msm_sensor_ctrl_t *s_ctrl);

int imx214_app_write_sensor(uint32_t addr, uint16_t data, struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
			addr,
			data,
			MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: write imx214 otp failed\n", __func__);
		return rc;
	}
	CDBG("%s addr=%x, data=%x\n", __func__, addr, data);
	return rc;
}

int imx214_app_read_sensor(uint32_t addr, struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			addr,
			&chipid, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: read imx214 otp failed\n", __func__);
		return rc;
	}
	chipid = chipid >> 8;
	CDBG("%s: read imx214 otp addr: %x value %x:\n", __func__, addr,
		chipid);
	return chipid;
}

int imx214_app_update_wb_register_from_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct imx214_app_otp_struct current_otp_wb;
	//int wb_ration_RGr=0,wb_ration_BGr=0,wb_ration_GbGr=0;
	int AWB_RGB_Gain[3]={0,0,0}; //test R/Gr B/Gr Gb/Gr 大少
	int R_Gain=0,B_Gain=0,Gr_Gain=0,Gb_Gain=0;
    int otp_wb_page =12;
	int i;
	int temp;
	int minIndex=0;
	int min=AWB_RGB_Gain[0];
    for(i=0;i<3;i++)
    {
    		otp_wb_page=otp_wb_page-1;
    		temp =imx214_app_check_otp_wb(otp_wb_page, s_ctrl);
    		if(temp ==2)
    			{
    				break;
    			}
    }
		
		if(i<3)
		{
				imx214_app_read_otp_wb(otp_wb_page,&current_otp_wb, s_ctrl);
		}
		else
			{
				current_otp_wb.AWB_Year=2013;
			  current_otp_wb.AWB_Monty=10;
			  current_otp_wb.AWB_Day=1;
				current_otp_wb.AWB_MID=7;
				current_otp_wb.AWB_VendorID=0;
				current_otp_wb.AWB_Current_RGr=AWB_TYPICAL_RGr;
				current_otp_wb.AWB_Current_BGr=AWB_TYPICAL_BGr;
				current_otp_wb.AWB_Current_GbGr=AWB_TYPICAL_GbGr;
			}
			
		AWB_RGB_Gain[0]=AWB_TYPICAL_RGr*1024/current_otp_wb.AWB_Current_RGr;
		AWB_RGB_Gain[1]=AWB_TYPICAL_BGr*1024/current_otp_wb.AWB_Current_BGr;
		AWB_RGB_Gain[2]=AWB_TYPICAL_GbGr*1024/current_otp_wb.AWB_Current_GbGr;
		
		for(i=0;i<3;i++)
		{
			if(min<AWB_RGB_Gain[i])
				{
					min=AWB_RGB_Gain[i];
					minIndex =i;
				}
		}
		
		if(min<1024)
		{
			switch(minIndex)
			{
				case 0://R gain 最少
					R_Gain =0x0100;	
					Gr_Gain =R_Gain*current_otp_wb.AWB_Current_RGr/AWB_TYPICAL_RGr;
					B_Gain =Gr_Gain*AWB_TYPICAL_BGr/current_otp_wb.AWB_Current_BGr;
					Gb_Gain =Gr_Gain*AWB_TYPICAL_GbGr/current_otp_wb.AWB_Current_GbGr;
					break;
				case 1://B gain 最少
					B_Gain =0x0100;	
					Gr_Gain =B_Gain*current_otp_wb.AWB_Current_BGr/AWB_TYPICAL_BGr;
					R_Gain =Gr_Gain*AWB_TYPICAL_RGr/current_otp_wb.AWB_Current_RGr;
					Gb_Gain =Gr_Gain*AWB_TYPICAL_GbGr/current_otp_wb.AWB_Current_GbGr;
					
					break;
				default ://Gb gain 最少
					Gb_Gain =0x0100;	
					Gr_Gain =Gb_Gain*current_otp_wb.AWB_Current_GbGr/AWB_TYPICAL_GbGr;
					R_Gain =Gr_Gain*AWB_TYPICAL_RGr/current_otp_wb.AWB_Current_RGr;
					B_Gain =Gr_Gain*AWB_TYPICAL_BGr/current_otp_wb.AWB_Current_BGr;
					
					break;
			}
		}else
			{//Gr gain 最少
				Gr_Gain =0x0100;	
				Gb_Gain =Gr_Gain*AWB_TYPICAL_GbGr/current_otp_wb.AWB_Current_GbGr;
				R_Gain =Gr_Gain*AWB_TYPICAL_RGr/current_otp_wb.AWB_Current_RGr;
				B_Gain =Gr_Gain*AWB_TYPICAL_BGr/current_otp_wb.AWB_Current_BGr;
			}
		
		imx214_app_update_wb_gain(R_Gain,Gr_Gain,B_Gain,Gb_Gain, s_ctrl);

		return 0 ;//Success	
}

int imx214_app_update_wb_gain(int R_Gain,int Gr_Gain,int B_Gain,int Gb_Gain, struct msm_sensor_ctrl_t *s_ctrl)
{
	//Gr_gain[15:8]0X020E Gr_gain[7:0]0X020F
	//R_gain[15:8]0X0210 R_gain[7:0]0X0211
	//B_gain[15:8]0X0212 B_gain[7:0]0X0213
	//Gb_gain[15:8]0X0214 Gb_gain[7:0]0X0215
	//Gr_gain
	imx214_app_write_sensor(0X020E,(0xff&(Gr_Gain>>8)), s_ctrl);
	imx214_app_write_sensor(0X020F,(0xff&Gr_Gain), s_ctrl);
	//R_gain
	imx214_app_write_sensor(0X0210,(0xff&(R_Gain>>8)), s_ctrl);
	imx214_app_write_sensor(0X0211,(0xff&R_Gain), s_ctrl);
	//B_gain
	imx214_app_write_sensor(0X0212,(0xff&(B_Gain>>8)), s_ctrl);
	imx214_app_write_sensor(0X0213,(0xff&B_Gain), s_ctrl);
	//Gb_gain
	imx214_app_write_sensor(0X0214,(0xff&(Gb_Gain>>8)), s_ctrl);
	imx214_app_write_sensor(0X0215,(0xff&Gb_Gain), s_ctrl);
	return 0;
}
int imx214_app_read_test(struct msm_sensor_ctrl_t *s_ctrl)
{
	imx214_app_read_sensor(0X020E,s_ctrl);
	imx214_app_read_sensor(0X020F,s_ctrl);
	imx214_app_read_sensor(0X0210,s_ctrl);
	imx214_app_read_sensor(0X0211,s_ctrl);
	imx214_app_read_sensor(0X0212,s_ctrl);
	imx214_app_read_sensor(0X0213,s_ctrl);
	imx214_app_read_sensor(0X0214,s_ctrl);
	imx214_app_read_sensor(0X0215,s_ctrl);
	return 0;
}


int imx214_app_read_otp_wb(int otp_wb_page,struct imx214_app_otp_struct *current_otp_wb, struct msm_sensor_ctrl_t *s_ctrl)

{
	int temp_value=0;
	int i=0,delayCode=10;
	int high_byte=0,low_byte=0;
	imx214_app_write_sensor(0x3b02,otp_wb_page, s_ctrl);
	
	imx214_app_write_sensor(0x3b00,0x01, s_ctrl);
	
	for(i=0;i<delayCode;i++)
	{
		temp_value=imx214_app_read_sensor(0x3b01, s_ctrl);
		if(temp_value==1)
			{
				break;
			}
		msleep(1);//delay 1ms
	}
	
	if(i==delayCode)
	{
			return 0;//read page fail
	}
	
	temp_value=0;
	
	current_otp_wb->AWB_Year=imx214_app_read_sensor(0x3b04, s_ctrl);
	current_otp_wb->AWB_Monty=imx214_app_read_sensor(0x3b05, s_ctrl);
	current_otp_wb->AWB_Day=imx214_app_read_sensor(0x3b06, s_ctrl);
	current_otp_wb->AWB_MID=imx214_app_read_sensor(0x3b07, s_ctrl);
	current_otp_wb->AWB_VendorID=imx214_app_read_sensor(0x3b08, s_ctrl);
	
	high_byte=imx214_app_read_sensor(0x3b09, s_ctrl);
	low_byte=imx214_app_read_sensor(0x3b0a, s_ctrl);
	current_otp_wb->AWB_Current_RGr=(high_byte<<8)|low_byte;
	
	high_byte=imx214_app_read_sensor(0x3b0b, s_ctrl);
	low_byte=imx214_app_read_sensor(0x3b0c, s_ctrl);
	current_otp_wb->AWB_Current_BGr=(high_byte<<8)|low_byte;
	
	high_byte=imx214_app_read_sensor(0x3b0d, s_ctrl);
	low_byte=imx214_app_read_sensor(0x3b0e, s_ctrl);
	current_otp_wb->AWB_Current_GbGr=(high_byte<<8)|low_byte;

	return 2;
}


int imx214_app_check_otp_wb(int Index, struct msm_sensor_ctrl_t *s_ctrl)
{
	int i=0,delayCode=10;
	int temp_value=0;
	imx214_app_write_sensor(0x0A02,Index,s_ctrl);
	
	imx214_app_write_sensor(0x0A00,0x01,s_ctrl);
	
	for(i=0;i<delayCode;i++)
	{
		temp_value=imx214_app_read_sensor(0x0A01,s_ctrl);
		if(temp_value==1)
		{
			break;
		}
		msleep(1);//delay 1ms
	}
	
	temp_value=imx214_app_read_sensor(0x3b0f,s_ctrl);
	if(temp_value==0xff)
	{
			return 2;
	}
	
	if(temp_value==0x00)
	{
			return 0;
	}
	else
		{
			return 1;
		}
}

