/*********************************************************************

  Copyright (C), 2001-2013, ZTE. Co., Ltd.

 *********************************************************************
  File Name     : zte-ctp.c
  Version        : Initial Draft
  Author         : luochangyang
  Created       : 2013/09/03
  Last Modified :
  Description   : This is touchscreen driver board file
  Function List :
  History       :
  1.Date        : 2013/09/03
    Author      : luochangyang
    Modification: Created file

**********************************************************************/

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/bootmem.h>
#include <linux/ion.h>
#include <asm/mach-types.h>
#include <mach/msm_memtypes.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <mach/ion.h>
#include <linux/regulator/consumer.h>
#include <mach/msm_bus_board.h>
#include <mach/socinfo.h>
#include <mach/irqs.h>

#include <linux/i2c.h>


#ifdef CONFIG_TOUCHSCREEN_CYPRESS_TMA568
/* cyttsp */
#include "cyttsp5_bus.h"
#include "cyttsp5_core.h"
#include "cyttsp5_btn.h"
#include "cyttsp5_mt.h"
#include "cyttsp5_proximity.h"
#include "cyttsp5_platform.h"


#define CYTTSP5_USE_I2C
/* #define CYTTSP5_USE_SPI */

#ifdef CYTTSP5_USE_I2C
#define CYTTSP5_I2C_NAME "cyttsp5_i2c_adapter"
#define CYTTSP5_I2C_TCH_ADR 0x24
#define CYTTSP5_LDR_TCH_ADR 0x24
#define CYTTSP5_I2C_IRQ_GPIO 61 /* J6.9, C19, GPMC_AD14/GPIO_38 */
#define CYTTSP5_I2C_RST_GPIO 60 /* J6.10, D18, GPMC_AD13/GPIO_37 */
#endif

#ifndef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT

#define CYTTSP5_HID_DESC_REGISTER 1

/*NX601J*/
#if defined CONFIG_CYTTSP5_6P4_INCH_TP
#define CY_VKEYS_X 1080
#define CY_VKEYS_Y 1920
#define CY_MAXX 1080
#define CY_MAXY 1920

#define CYTTSP5_GW_FIRMEARE_VERSION 	0x0000
#define CYTTSP5_TPK_FIRMEARE_VERSION	0x05D1

/*NX504J*/
#elif defined CONFIG_CYTTSP5_5P5_INCH_TP
#define CY_VKEYS_X 1080
#define CY_VKEYS_Y 1920
#define CY_MAXX 1080
#define CY_MAXY 1920

#define CYTTSP5_GW_FIRMEARE_VERSION 	0x0705
#define CYTTSP5_TPK_FIRMEARE_VERSION	0x0000

/*NX506J*/
#elif defined CONFIG_CYTTSP5_5P5_INCH_TP_2K

#define CY_VKEYS_X 1440
#define CY_VKEYS_Y 2560
#define CY_MAXX 1440
#define CY_MAXY 2560

#define CYTTSP5_GW_FIRMEARE_VERSION 	0x0908
#define CYTTSP5_TPK_FIRMEARE_VERSION	0x0A02

#else
#define CY_VKEYS_X 1080
#define CY_VKEYS_Y 1920
#define CY_MAXX 1080
#define CY_MAXY 1920

#define CYTTSP5_GW_FIRMEARE_VERSION 	0x0000
#define CYTTSP5_TPK_FIRMEARE_VERSION	0x0000

#endif

#define CY_MINX 0
#define CY_MINY 0

#define CY_ABS_MIN_X CY_MINX
#define CY_ABS_MIN_Y CY_MINY
#define CY_ABS_MAX_X CY_MAXX
#define CY_ABS_MAX_Y CY_MAXY
#define CY_ABS_MIN_P 0
#define CY_ABS_MAX_P 255
#define CY_ABS_MIN_W 0
#define CY_ABS_MAX_W 255
#define CY_PROXIMITY_MIN_VAL	0
#define CY_PROXIMITY_MAX_VAL	1

#define CY_ABS_MIN_T 0

#define CY_ABS_MAX_T 15

#define CY_IGNORE_VALUE 0xFFFF

#define CYTTSP5_VIRTUAL_KEYS

/* Button to keycode conversion */
static u16 cyttsp5_btn_keys[] = {
	/* use this table to map buttons to keycodes (see input.h) */
	KEY_BACK,		/* 158 */
	KEY_HOME,		/* 102 */
	KEY_MENU,		/* 139 */
	KEY_SEARCH,		/* 217 */
	KEY_VOLUMEDOWN,		/* 114 */
	KEY_VOLUMEUP,		/* 115 */
	KEY_CAMERA,		/* 212 */
	KEY_POWER		/* 116 */
};

static struct touch_settings cyttsp5_sett_btn_keys = {
	.data = (uint8_t *)&cyttsp5_btn_keys[0],
	.size = ARRAY_SIZE(cyttsp5_btn_keys),
	.tag = 0,
};


/*** ZTEMT Added by luochangyang, 2013/09/12 ***/
static int cyttsp_check_version(struct cyttsp5_sysinfo *si, struct device *dev)
{
    u16 fw_ver_ic;
    u16 fw_ver_dr;
    
	if (!si) {
		dev_info(dev, "%s NULL Pointer detected!\n",__func__);
		WARN_ON(1);
		return -EINVAL;
	}
	
	memcpy(si->cy_fw_file_name, "cyttsp5_fw.bin", CY_FW_FILE_NAME_LEN);

    if (si->fw_ver_ic == 0x0584 || si->fw_ver_ic == 0x05A2
        || (si->fw_ver_ic & 0xFFF0) == 0x05C0 || si->fw_ver_ic == 0x0000) {
        si->fw_ver_dr = CYTTSP5_TPK_FIRMEARE_VERSION;

        return 1;
    }

    switch (si->fw_ver_ic & 0xFFF0) {
        case 0x05D0:
            si->fw_ver_dr = CYTTSP5_TPK_FIRMEARE_VERSION;
            break;
        case 0x0700:
            si->fw_ver_dr = CYTTSP5_GW_FIRMEARE_VERSION;
            break;
        case 0x0900:
            si->fw_ver_dr = CYTTSP5_GW_FIRMEARE_VERSION;
            break;
		case 0x0A00:
            si->fw_ver_dr = CYTTSP5_TPK_FIRMEARE_VERSION;
			memcpy(si->cy_fw_file_name, "cyttsp5_fw_1.bin", CY_FW_FILE_NAME_LEN);
            break;
        default:
            si->fw_ver_dr = 0x0000;
            dev_info(dev, "%s: FW didn't match, will NOT upgarde!\n", __func__);
            return -1;
    }

	/*For NX506J GW TP write FW 0x0701 */
	if (si->fw_ver_ic == 0x0701 && (si->fw_ver_dr & 0xFFF0) == 0x0900) {
        return 1;
    }

	if ((si->fw_ver_ic & 0xFFF0) == (si->fw_ver_dr & 0xFFF0)) {
		fw_ver_ic = si->fw_ver_ic & 0x000F; 
		fw_ver_dr = si->fw_ver_dr & 0x000F;

		if (fw_ver_ic  == fw_ver_dr) {/*equal*/
	        return 0;
		} else if (fw_ver_dr > fw_ver_ic) {
			return 1;
		} else {
			return -1;
		}
	} else {
		dev_info(dev, "%s: FW didn't match, will NOT upgarde!\n", __func__);
		return -1;
	}
}
/***ZTEMT END***/

static struct cyttsp5_core_platform_data _cyttsp5_core_platform_data = {
	.irq_gpio = CYTTSP5_I2C_IRQ_GPIO,
	.rst_gpio = CYTTSP5_I2C_RST_GPIO,
	.hid_desc_register = CYTTSP5_HID_DESC_REGISTER,
	.xres = cyttsp5_xres,
	.init = cyttsp5_init,
	.power = cyttsp5_power,
	.irq_stat = cyttsp5_irq_stat,
	.check_version = cyttsp_check_version,
	.sett = {
		NULL,	/* Reserved */
		NULL,	/* Command Registers */
		NULL,	/* Touch Report */
		NULL,	/* Cypress Data Record */
		NULL,	/* Test Record */
		NULL,	/* Panel Configuration Record */
		NULL,	/* &cyttsp5_sett_param_regs, */
		NULL,	/* &cyttsp5_sett_param_size, */
		NULL,	/* Reserved */
		NULL,	/* Reserved */
		NULL,	/* Operational Configuration Record */
		NULL, /* &cyttsp5_sett_ddata, *//* Design Data Record */
		NULL, /* &cyttsp5_sett_mdata, *//* Manufacturing Data Record */
		NULL,	/* Config and Test Registers */
		&cyttsp5_sett_btn_keys,	/* button-to-keycode table */
	},
	.loader_pdata = &_cyttsp5_loader_platform_data,
	.flags = CY_CORE_FLAG_WAKE_ON_GESTURE,
};

static struct cyttsp5_core_info cyttsp5_core_info __initdata = {
	.name = CYTTSP5_CORE_NAME,
	.id = "main_ttsp_core",
	.adap_id = CYTTSP5_I2C_NAME,
	.platform_data = &_cyttsp5_core_platform_data,
};

static const uint16_t cyttsp5_abs[] = {
	ABS_MT_POSITION_X, CY_ABS_MIN_X, CY_ABS_MAX_X, 0, 0,
	ABS_MT_POSITION_Y, CY_ABS_MIN_Y, CY_ABS_MAX_Y, 0, 0,
	ABS_MT_PRESSURE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
	CY_IGNORE_VALUE, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
	ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
	ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0,
	ABS_MT_TOUCH_MINOR, 0, 255, 0, 0,
	ABS_MT_ORIENTATION, -128, 127, 0, 0,
//	ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0,
};

struct touch_framework cyttsp5_framework = {
	.abs = (uint16_t *)&cyttsp5_abs[0],
	.size = ARRAY_SIZE(cyttsp5_abs),
	.enable_vkeys = 0,
};

static struct cyttsp5_mt_platform_data _cyttsp5_mt_platform_data = {
	.frmwrk = &cyttsp5_framework,
	.flags = CY_MT_FLAG_VKEYS,   //CY_MT_FLAG_INV_X | CY_MT_FLAG_INV_Y,
	.inp_dev_name = CYTTSP5_MT_NAME,
	.vkeys_x = CY_VKEYS_X,
	.vkeys_y = CY_VKEYS_Y,
};

static struct cyttsp5_device_info cyttsp5_mt_info __initdata = {
	.name = CYTTSP5_MT_NAME,
	.core_id = "main_ttsp_core",
	.platform_data = &_cyttsp5_mt_platform_data,
};

static struct cyttsp5_btn_platform_data _cyttsp5_btn_platform_data = {
	.inp_dev_name = CYTTSP5_BTN_NAME,
};

static struct cyttsp5_device_info cyttsp5_btn_info __initdata = {
	.name = CYTTSP5_BTN_NAME,
	.core_id = "main_ttsp_core",
	.platform_data = &_cyttsp5_btn_platform_data,
};

static const uint16_t cyttsp5_prox_abs[] = {
	ABS_DISTANCE, CY_PROXIMITY_MIN_VAL, CY_PROXIMITY_MAX_VAL, 0, 0,
};

struct touch_framework cyttsp5_prox_framework = {
	.abs = (uint16_t *)&cyttsp5_prox_abs[0],
	.size = ARRAY_SIZE(cyttsp5_prox_abs),
};

static struct cyttsp5_proximity_platform_data
		_cyttsp5_proximity_platform_data = {
	.frmwrk = &cyttsp5_prox_framework,
	.inp_dev_name = CYTTSP5_PROXIMITY_NAME,
};

struct cyttsp5_device_info cyttsp5_proximity_info __initdata = {
	.name = CYTTSP5_PROXIMITY_NAME,
	.core_id = "main_ttsp_core",
	.platform_data = &_cyttsp5_proximity_platform_data,
};

#ifdef CYTTSP5_VIRTUAL_KEYS
static ssize_t cyttsp5_virtualkeys_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
/*** ZTEMT Modify by luochangyang, 2013/09/11 ***/
	return sprintf(buf,
        __stringify(EV_KEY)":"__stringify(KEY_MENU)":270:2035:180:110" ":"
        __stringify(EV_KEY)":"__stringify(KEY_HOME)":540:2035:120:110" ":"
        __stringify(EV_KEY)":"__stringify(KEY_BACK)":810:2035:180:110" "\n"
//		":" __stringify(EV_KEY) ":"
//		__stringify(KEY_SEARCH) ":1360:630:160:180"
/***ZTEMT END***/
    );
}

static struct kobj_attribute cyttsp5_virtualkeys_attr = {
	.attr = {
		.name = "virtualkeys.cyttsp5_mt",
		.mode = S_IRUGO,
	},
	.show = &cyttsp5_virtualkeys_show,
};

static struct attribute *cyttsp5_properties_attrs[] = {
	&cyttsp5_virtualkeys_attr.attr,
	NULL
};

static struct attribute_group cyttsp5_properties_attr_group = {
	.attrs = cyttsp5_properties_attrs,
};
#endif
#endif /* !CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT */

static int __init cyttsp5_i2c_device_init(void)
{
	int ret = 0;
    
#ifndef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
#ifdef CYTTSP5_VIRTUAL_KEYS
    struct kobject *properties_kobj;
#endif

    pr_debug("%s: Start...\n", __func__);

    /* Register core and devices */
    cyttsp5_register_core_device(&cyttsp5_core_info);
    cyttsp5_register_device(&cyttsp5_mt_info);
    cyttsp5_register_device(&cyttsp5_btn_info);
	cyttsp5_register_device(&cyttsp5_proximity_info);

#ifdef CYTTSP5_VIRTUAL_KEYS
    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                &cyttsp5_properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("%s: failed to create board_properties\n", __func__);
#endif
#endif

    return ret;
}

#ifndef CONFIG_OF
static struct i2c_board_info cypress_tma568_ts_i2c_info[] __initdata= {
	{
		I2C_BOARD_INFO(CYTTSP5_I2C_NAME, CYTTSP5_I2C_TCH_ADR),
		.irq = MSM_GPIO_TO_INT(CYTTSP5_I2C_IRQ_GPIO),
		.platform_data = CYTTSP5_I2C_NAME,
	},
};
#endif
#endif

static int  __init zte_init_ctp(void)
{
	int ret = 0;

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_TMA568
	ret = cyttsp5_i2c_device_init();

#ifndef CONFIG_OF
	ret = i2c_register_board_info(2, cypress_tma568_ts_i2c_info, 
								  ARRAY_SIZE(cypress_tma568_ts_i2c_info));
#endif
#endif
	return ret;
}

arch_initcall(zte_init_ctp);

