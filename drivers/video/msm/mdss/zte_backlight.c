#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/err.h>

#include "zte_backlight.h"
#include "zte_disp_enhance.h"
#include "zte_backlight_data.h"
#include "mdss_dsi.h"



static ssize_t pwm_setting_store(struct kobject *kobj, struct kobj_attribute *attr,
		    const char *buf, size_t size);
static ssize_t zte_backlight_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf);
static ssize_t zte_backlight_store(struct kobject *kobj, struct kobj_attribute *attr,
		    const char *buf, size_t size);


extern struct kobject *enhance__kobj;

static struct mdss_dsi_ctrl_pdata *zte_mdss_dsi_ctrl = NULL;

static int zte_backlight_enable = 1;

static struct kobj_attribute pwm_attribute = __ATTR(pwm_setting,0664,NULL,pwm_setting_store);
static struct kobj_attribute zte_backlight_attribute = __ATTR(zte_backlight_EnSetting,0664,zte_backlight_show,zte_backlight_store);
static struct attribute *cali_attrs[] = {
				&pwm_attribute.attr,
				&zte_backlight_attribute.attr,
				NULL,
				};
static struct attribute_group cali_attrs_groups = {
				.attrs = cali_attrs,
				};



static bool IsPwmTableExit(void)
{
	if((sizeof(brightness2pwm_normal)/sizeof(struct BRIGHTNESS_PWM_TYPE) <2) ||\
		(sizeof(bgvalue2brightness)/sizeof(struct BGVALUE_BRIGHTNESS_TYPE) < 2))
	{
		printk("any of the two pwm tables are null \n");
		return false;
	}

	return true;
}


int ZteBgvalue2pwm(int bgvalue)
{
	int brightness = bgvalue2brightness[0].brightness;
	int pwm = 0;
	//int cnt = 0;
	int i;
	const struct BRIGHTNESS_PWM_TYPE *p_brightness2pwm = NULL;
	int table_len = 0;

	if(!IsPwmTableExit())
	{
		printk("%s:%s pwm table is not exit\n",__FILE__,__func__);
		return -1;
	}

	if(0 == bgvalue)
	{
		return 0;
	}
//use bgvalue to get true brightness
	table_len = sizeof(bgvalue2brightness)/sizeof(struct BGVALUE_BRIGHTNESS_TYPE);
	for(i=0;i<table_len;i++)
	{
		if(bgvalue == bgvalue2brightness[i].bgvalue)
		{
			brightness = bgvalue2brightness[i].brightness;
			break;
		}
		else if(bgvalue <  bgvalue2brightness[i].bgvalue)
		{
			if(0 == i)
			{
				brightness = bgvalue2brightness[i].brightness;
				break;
			}
			brightness = bgvalue2brightness[i - 1].brightness + \
					(bgvalue2brightness[i].brightness - bgvalue2brightness[i-1].brightness)*(bgvalue-bgvalue2brightness[i-1].bgvalue)\
					/(bgvalue2brightness[i].bgvalue -bgvalue2brightness[i-1].bgvalue );
			break;
		}
	}
	if(i >= table_len)
	{
		brightness = bgvalue2brightness[table_len - 1].brightness;
	}

	ZTE_BACKLIGHT_DEBUG("%s %s: i: %d, brightness: %d, table_len: %d, bgvalue:%d\n",__FILE__,__func__,i,brightness,table_len,bgvalue);
	
	//use colortmp to choose brightness2pwm table
	
	p_brightness2pwm = brightness2pwm_normal;
	table_len = sizeof(brightness2pwm_normal)/sizeof(struct BRIGHTNESS_PWM_TYPE);

	ZTE_BACKLIGHT_DEBUG("zte_enhance.en_colortmp not enable : brightness2pwm_normal\n");



//use true brightness to get pwm value
	for (i = 0; i < table_len; i++)
	{
		if(brightness == p_brightness2pwm[i].brightness)
		{
			pwm = p_brightness2pwm[i].pwm;
			break;
		}
		else if(brightness < p_brightness2pwm[i].brightness)
		{
			if(0 == i)
			{
				pwm = p_brightness2pwm[i].pwm;
				break;
			}
			pwm = p_brightness2pwm[i - 1].pwm + \
				(brightness - p_brightness2pwm[i-1].brightness)*(p_brightness2pwm[i].pwm - p_brightness2pwm[i-1].pwm)/\
				  (p_brightness2pwm[i].brightness - p_brightness2pwm[i-1].brightness);
			break;
		}
	}
	if(i >= table_len)
	{
		pwm = p_brightness2pwm[table_len - 1].pwm;
	}

	if(pwm > ZTE_DEFALUT_MAX_BLLEVEL)
	{
		pwm = ZTE_DEFALUT_MAX_BLLEVEL;
	}

	ZTE_BACKLIGHT_DEBUG("%s %s: i: %d, brightness: %d, table_len: %d, pwm:%d\n",__FILE__,__func__,i,brightness,table_len,pwm);


	return pwm;
}

void ztePwmBri_SetPoint(struct mdss_dsi_ctrl_pdata * ctrl)
{
	zte_mdss_dsi_ctrl = ctrl;
}


bool IsBgvalueTranEnable(void)
{

	ZTE_BACKLIGHT_DEBUG("zte_backlight_enable:%d\n",zte_backlight_enable);
	if(zte_backlight_enable)
	{
		if(IsPwmTableExit())
		{
			return true;
		}
	}

	return false;
}

static ssize_t zte_backlight_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n",zte_backlight_enable);
}


static ssize_t zte_backlight_store(struct kobject *kobj, struct kobj_attribute *attr,
		    const char *buf, size_t size)
{
	int setvalue;

	sscanf(buf,"%d",&setvalue);

	ZTE_BACKLIGHT_DEBUG("zte_backlight_enable before set:%d,setvalue:%d,size:%d\n",zte_backlight_enable,setvalue,size);

	if(setvalue > 1 || setvalue < 0)
	{
		return -EBUSY;
	}

	zte_backlight_enable = setvalue;
	ZTE_BACKLIGHT_DEBUG("zte_backlight_enable after set:%d\n",zte_backlight_enable);

	return size;
}

static ssize_t pwm_setting_store(struct kobject *kobj, struct kobj_attribute *attr,
		    const char *buf, size_t size)
{
	int pwm_value;
	struct mdss_panel_data *pdata;


	sscanf(buf, "%d", &pwm_value);
	
	printk("%s:%s pwm_value = %d, size = %d\n",__FILE__,__func__,pwm_value,(int)size);

	if(zte_mdss_dsi_ctrl)
	{
		pdata = &zte_mdss_dsi_ctrl->panel_data;
		if(pdata && (pdata->set_backlight))
		{
			pdata->set_backlight(pdata,pwm_value);
		}
		else
		{
			printk("%s:%s zte_mdss_dsi_ctrl->panel_data or ->set_backlight is null\n",__FILE__,__func__);
			return -EBUSY;
		}
	}
	else
	{
		printk("%s:%s zte_mdss_dsi_ctrl is NULL.\n",__FILE__,__func__);
		return -EBUSY;
	}
	return size;
}

static int __init pwm_brightness_cali_init(void)
{
	int ret;

	if(!enhance__kobj)
	{
		printk("%s enhance_kobj is null now\n",__func__);
		return -ENOMEM;
	}

	ret = sysfs_create_group(enhance__kobj,&cali_attrs_groups);


	return ret;
}

static void __exit pwm_brightness_cali_exit(void)
{

}

 module_init(pwm_brightness_cali_init);
 module_exit(pwm_brightness_cali_exit);

