#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include "mdss_mdp.h"

#include "zte_disp_enhance.h"

#define ZTE_SHARP_ENHANCE_CMD_COUNT 1
#define ZTE_DISP_ENHANCE_DEBUG		0
#define ZTEMT_LCD_CORLORTMP_DEBUG 	0


/*mdp adjust colortmp mayu add*/
#if defined(CONFIG_ZTEMT_MIPI_1080P_R63311_SHARP_IPS_5P0) || \
	defined(CONFIG_ZTEMT_MIPI_1080P_R63311_SHARP_IPS_5P0_NX507J)
#include "zte_disp_enhance_NX503A.h"
#endif

#if defined(CONFIG_ZTEMT_MIPI_1080P_R63311_SHARP_IPS_5P5)
#include "zte_disp_enhance_NX504J.h"
#endif

#if defined(CONFIG_ZTEMT_MIPI_1080P_R63417_SHARP_IPS_5P5)
#include "zte_disp_enhance_NX505J.h"
#endif

#if defined(CONFIG_ZTEMT_MIPI_2K_R63419_SHARP_IPS_5P5)
#include "zte_disp_enhance_NX506J.h"
#endif

#if defined(CONFIG_ZTEMT_MIPI_1080P_R63311_SHARP_IPS_6P4)
#include "zte_disp_enhance_NX601J.h"
#endif

#if ZTEMT_LCD_CORLORTMP_DEBUG

struct mdp_pcc_cfg_data zte_pcc_cfg_debug = {
	.block = 0x10,
	.ops = 0x5,
	{
	  .c = 0,
	  .r = 0x8000,
	  .g = 0,
	  .b = 0,
	  .rr = 0,
	  .gg = 0,
	  .bb = 0,
	  .rg = 0,
	  .gb = 0,
	  .rb = 0,
	  .rgb_0 = 0,
	  .rgb_1 = 0
	},
	{
	  .c = 0,
	  .r = 0,
	  .g = 0x8000,
	  .b = 0,
	  .rr = 0,
	  .gg = 0,
	  .bb = 0,
	  .rg = 0,
	  .gb = 0,
	  .rb = 0,
	  .rgb_0 = 0,
	  .rgb_1 = 0
	},
	{
	  .c = 0,
	  .r = 0,
	  .g = 0,
	  .b = 0x8000,
	  .rr = 0,
	  .gg = 0,
	  .bb = 0,
	  .rg = 0,
	  .gb = 0,
	  .rb = 0,
	  .rgb_0 = 0,
	  .rgb_1 = 0
	},
};

#endif


static struct zte_enhance_type zte_enhance_val = {
	.en_saturation =1,
	.saturation = INTENSITY_01,
	.colortmp =  INTENSITY_00,
#if defined (CONFIG_ZTEMT_MIPI_1080P_R63417_SHARP_IPS_5P5) || \
	defined (CONFIG_ZTEMT_MIPI_2K_R63419_SHARP_IPS_5P5) || \
	defined (CONFIG_ZTEMT_MIPI_1080P_R63311_SHARP_IPS_5P0_NX507J) || \
	defined (CONFIG_ZTEMT_MIPI_1080P_R63311_SHARP_IPS_6P4)
	.en_colortmp = 1,
#else
	.en_colortmp = 0,
#endif
};

static struct mdss_dsi_ctrl_pdata *zte_mdss_dsi_ctrl = NULL;

struct zte_enhance_type zte_get_lcd_enhance_param(void)
{
	return zte_enhance_val;
}

void zte_send_cmd(struct dsi_cmd_desc *cmds,int cmdcount)
{
	struct dcs_cmd_req cmdreq;

	if((!zte_mdss_dsi_ctrl) || (cmdcount < 1))	{
		pr_err("lcd:faild:%s zte_mdss_dsi_ctrl is null\n",__func__);
		return;
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = cmds;
	cmdreq.cmds_cnt = cmdcount;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(zte_mdss_dsi_ctrl, &cmdreq);
}

void zte_mipi_saturation(void)
{
	unsigned int value;
	value =zte_enhance_val.saturation;

	if(!zte_enhance_val.en_saturation || ((NULL == display_std_cmd.payload) || \
		(NULL == display_soft_cmd.payload) ||(NULL == display_glow_cmd.payload)) ||\
		(NULL == zte_mdss_dsi_ctrl))
		return;

#if ZTE_DISP_ENHANCE_DEBUG
	printk("lcd:%s value=%d\n", __func__, value);
#endif

	switch (value) {
	case INTENSITY_00:
		zte_send_cmd(&display_soft_cmd,ZTE_SHARP_ENHANCE_CMD_COUNT);
		break;
	case INTENSITY_01:
		zte_send_cmd(&display_std_cmd,ZTE_SHARP_ENHANCE_CMD_COUNT);
		break;
	case INTENSITY_02:
		zte_send_cmd(&display_glow_cmd,ZTE_SHARP_ENHANCE_CMD_COUNT);
		break;
	default:
		zte_send_cmd(&display_std_cmd,ZTE_SHARP_ENHANCE_CMD_COUNT);
		break;
	}
}

static ssize_t saturation_show(struct kobject *kobj, 
	struct kobj_attribute *attr, char *buf)
{	
	return snprintf(buf, PAGE_SIZE, "%d\n",	zte_enhance_val.en_saturation);
}

static ssize_t saturation_store(struct kobject *kobj, struct kobj_attribute *attr,
							const char *buf, size_t size)
{
	int val;

	if(!zte_enhance_val.en_saturation)
		return size;

	sscanf(buf, "%d", &val);

#if ZTE_DISP_ENHANCE_DEBUG
	printk("lcd:%s state=%d size=%d\n", __func__, (int)val, (int)size);
#endif

	zte_enhance_val.saturation =val;

	zte_mipi_saturation();
	return size;
}


void zte_mipi_colortmp(void)
{
	unsigned int value;
	value =zte_enhance_val.colortmp;

	if(!zte_enhance_val.en_colortmp || (NULL == zte_mdss_dsi_ctrl))
		return ;
	
#if ZTE_DISP_ENHANCE_DEBUG
	printk("lcd:%s value=%d\n", __func__, value);
#endif

	switch (value) {
	case INTENSITY_00:
		zte_mdss_pcc_config(&zte_pcc_cfg_warm);
		break;
	case INTENSITY_01:
		zte_mdss_pcc_config(&zte_pcc_cfg_normal);
		break;
	case INTENSITY_02:
		zte_mdss_pcc_config(&zte_pcc_cfg_cool);
		break;
	default:
#if defined(CONFIG_ZTEMT_MIPI_2K_R63419_SHARP_IPS_5P5) || \
	defined(CONFIG_ZTEMT_MIPI_1080P_R63311_SHARP_IPS_6P4)
		zte_mdss_pcc_config(&zte_pcc_cfg_warm);
#else
		zte_mdss_pcc_config(&zte_pcc_cfg_normal);
#endif
		break;
	}
}

static ssize_t colortmp_show(struct kobject *kobj, 
	struct kobj_attribute *attr, char *buf)
{	
	return snprintf(buf, PAGE_SIZE, "%d\n",	zte_enhance_val.colortmp);
}

static ssize_t colortmp_store(struct kobject *kobj, struct kobj_attribute *attr,
    const char *buf, size_t size)
{
	int val;

	if(!zte_enhance_val.en_colortmp)
	     return size;

	sscanf(buf, "%d", &val);

#if ZTE_DISP_ENHANCE_DEBUG
	printk("lcd:%s state=%d size=%d\n", __func__, (int)val, (int)size);
#endif

	zte_enhance_val.colortmp = val;
	
	zte_mipi_colortmp();
	return size;
}

#if ZTEMT_LCD_CORLORTMP_DEBUG

static ssize_t colortmp_debug_show(struct kobject *kobj, 
	struct kobj_attribute *attr, char *buf)
{	
	return snprintf(buf, PAGE_SIZE, "r = 0x%x, g = 0x%x, b = 0x%x\n",
		zte_pcc_cfg_debug.r.r, zte_pcc_cfg_debug.g.g, zte_pcc_cfg_debug.b.b);
}

static ssize_t colortmp_debug_store(struct kobject *kobj, struct kobj_attribute *attr,
    const char *buf, size_t size)
{
	int val;

	if(!zte_enhance_val.en_colortmp)
	     return size;

	sscanf(buf, "%d", &val);

#if ZTE_DISP_ENHANCE_DEBUG
	printk("lcd:%s state=%d size=%d\n", __func__, (int)val, (int)size);
#endif

	if (val == 1) {
		zte_mdss_pcc_config(&zte_pcc_cfg_debug);
	}

	return size;
}

static ssize_t colortmp_r_store(struct kobject *kobj, struct kobj_attribute *attr,
    const char *buf, size_t size)
{
	uint32_t val;

	sscanf(buf, "%x", &val);

	zte_pcc_cfg_debug.r.r = val;
	
	return size;
}

static ssize_t colortmp_g_store(struct kobject *kobj, struct kobj_attribute *attr,
    const char *buf, size_t size)
{
	uint32_t val;

	sscanf(buf, "%x", &val);

	zte_pcc_cfg_debug.g.g = val;
	
	return size;
}

static ssize_t colortmp_b_store(struct kobject *kobj, struct kobj_attribute *attr,
    const char *buf, size_t size)
{
	uint32_t val;

	sscanf(buf, "%x", &val);

	zte_pcc_cfg_debug.b.b = val;
	
	return size;
}

#endif

void zte_set_ctrl_point(struct mdss_dsi_ctrl_pdata * ctrl)
{
#if ZTE_DISP_ENHANCE_DEBUG
	printk("lcd:%s \n", __func__);
#endif

	zte_mdss_dsi_ctrl = ctrl;
}

void zte_boot_begin_enhance(struct mdss_dsi_ctrl_pdata *ctrl)
{
#if ZTE_DISP_ENHANCE_DEBUG
	printk("lcd:%s \n", __func__);
#endif
	zte_set_ctrl_point(ctrl);

	zte_mipi_saturation();
	zte_mipi_colortmp();
}

static struct kobj_attribute attrs[] = {
	__ATTR(saturation, 0664, saturation_show, saturation_store),
	__ATTR(colortmp, 0664, colortmp_show, colortmp_store),
#if ZTEMT_LCD_CORLORTMP_DEBUG
	__ATTR(colortmp_r, 0664, NULL, colortmp_r_store),
	__ATTR(colortmp_g, 0664, NULL, colortmp_g_store),
	__ATTR(colortmp_b, 0664, NULL, colortmp_b_store),
	__ATTR(colortmp_debug, 0664, colortmp_debug_show, colortmp_debug_store),
#endif
};

struct kobject *enhance__kobj;

static int __init enhance_init(void)
{
	int retval;
	int attr_count = 0;

	enhance__kobj = kobject_create_and_add("lcd_enhance", kernel_kobj);

	if (!enhance__kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		
		if(!zte_enhance_val.en_colortmp && (attr_count == 1))
			continue;
	
		retval = sysfs_create_file(enhance__kobj, &attrs[attr_count].attr);
		if (retval < 0) {
			pr_err("%s: Failed to create sysfs attributes\n", __func__);
			goto err_sys;
		}
	}
	
	pr_info("lcd: %s Done.\n",__func__);

	return retval;
	
err_sys:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(enhance__kobj, &attrs[attr_count].attr);
	}
	
	kobject_put(enhance__kobj);
	
	pr_info("lcd: %s init ERR.\n",__func__);

	return retval;
}

static void __exit enhance_exit(void)
{
	int attr_count = 0;
	
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(enhance__kobj, &attrs[attr_count].attr);
	}
	
	kobject_put(enhance__kobj);
	zte_mdss_dsi_ctrl = NULL;
}

module_init(enhance_init);
module_exit(enhance_exit);

