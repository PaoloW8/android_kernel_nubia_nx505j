/*
 * BQ24192 charger driver
 *
 * Copyright (C) 2013 ZTEMT
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/qpnp/qpnp-adc.h>
#include <bq24192_charger.h>
#ifdef CONFIG_ZTEMT_BATTERY_MAX17050
#include <max17050_battery.h>
#endif

#ifdef CONFIG_SLIMPORT_FAST_CHARGE
#include "../drivers/video/msm/mdss/slimport_anx7808/slimport.h"
#endif
//#include <../usb/dwc3/dwc3_otg.h>

#ifdef CONFIG_ZTEMT_HW_VERSION_NX601J
#include <ztemt_hw_version.h>
#endif

#define DRIVER_VERSION			"1.0.0"

//REG 01
#define BQ_MIN_SYS_V_MASK		0x0E
#define BQ_BOOST_I_BIT		BIT(0)
//REG 02
#define BQ_20PCT_BIT		BIT(0)
//REG 03

#define BQ_PRECHG_I_MASK	0xFO
//REG 04
#define BQ_FAST_THRESH_BIT		BIT(1)
#define BQ_RECHG_THRESH_BIT		BIT(0)
//REG 05
#define BQ_CHG_TERM_BIT		BIT(7)
#define BQ_TERM_INDT_BIT		BIT(6)
#define BQ_SAFE_TIME_BIT		BIT(3)
#define BQ_FAST_TIME_MASK		0X06
#define BQ_LOW_PER_BIT		BIT(0)
//REG 06
#define BQ_THERM_THRESH_MASK		0x03
//REG 07
#define BQ_DPDM_EN_BIT		BIT(7)
#define BQ_TMR2_EN_BIT		BIT(6)
#define BQ_BATFET_BIT		BIT(5)
#define BQ_VSET_BIT		BIT(4)
#define BQ_INT1_BIT		BIT(1)
#define BQ_INT2_BIT		BIT(0)
//REG 08
#define BQ_VBUS_STAT_MASK		0xC0

#define BQ_DPM_STAT_BIT		BIT(3)
#define BQ_PG_STAT_BIT		BIT(2)
#define BQ_THERM_STAT_BIT		BIT(1)
#define BQ_VSYS_STAT_BIT		BIT(0)
//REG 09
#define BQ_WDOG_FAULT_BIT		BIT(7)
#define BQ_BOOST_FAULT_BIT		BIT(6)
#define BQ_CHG_FAULT_MASK		0x30
#define BQ_BAT_FAULT_BIT		BIT(3)
#define BQ_NTC_FAULT_MASK		0x07
//REG 0A
#define BQ_PN_DEV_MASK		0x1C
#define BQ_TS_PROFILE_BIT	BIT(2)
#define BQ_DEV_MASK		0x03

#define PM_INFO 1
#define PM_DEBUG 4
//log level < bqlog_level will show
int bqlog_level = 3;  
module_param(bqlog_level, int, 0644);

#define BQLOG_INFO(fmt, args...) \
		if (PM_INFO < bqlog_level) \
			printk(KERN_WARNING "_%s: "  fmt,__func__, ##args)
	
#define BQLOG_DEBUG(fmt, args...) \
		if (PM_DEBUG < bqlog_level) \
			printk(KERN_WARNING "_%s: "  fmt,__func__, ##args)
			
struct bq24192_chg_chip {
	struct device			*i2c_dev;
	struct i2c_client *i2c;
	struct power_supply	   *batt_psy;
	struct hrtimer wdog_timer;
	struct delayed_work		chg_work;
	struct delayed_work		hvdcp_work;
	struct device_node *dev_node;
	struct wake_lock wlock;
	struct qpnp_vadc_chip	*vadc_dev;
	enum bq_chg_type chg_type;
	int chg_en_gpio;
	int otg_gpio;
	int psel_gpio;
	int irq_gpio;
	
	//batt status info
	int batt_status; 
	int batt_temp;
	int chg_status;
	int batt_i;
	int batt_vol;
	int batt_soc;
	
	int ibatmax_ma;
	int vusb_min;
	int iusb_init_ma;
	int iterm_ma;
	int vbatt_max;
	int compen_mohm;
	int compen_mv;
	u32 hvdcp_delay_time;
	int usb_in;
	bool in_work;
	bool temp_abnormal;
	bool soc_chg_done;
	bool in_rechging;
	bool is_hvdcp_chg;
	bool is_chg_full;
};

static struct bq24192_chg_chip   *bq_chip;
static int usbin_current = INPUT_1500_MA;
module_param(usbin_current, int, 0644);

static int ibat_current = 1200;
module_param(ibat_current, int, 0644);
static int charger_online = 0;

extern int qpnp_get_battery_temp(void);
extern int qpnp_dcdc_enable(int enable);
static int bq24192_get_temp_status(const struct batt_status_map *pts,
		uint32_t tablesize, int input, int *batt_status);


#ifdef CONFIG_ZTEMT_NX506J_CHARGE
static const struct batt_status_map batt_temp_map[] = {
	{-300,  -50, BATT_STATUS_COLD,    0}, 
	{ -50,  100, BATT_STATUS_COOL1, 800}, 
	{ 100,  430, BATT_STATUS_GOOD, 1984}, 
	{ 430,  500, BATT_STATUS_WARM,  800}, 
	{ 500,  800, BATT_STATUS_HOT,     0}, 
};
#else
static const struct batt_status_map batt_temp_map[] = {
	{CHG_TEMP_MIN,  CHG_TEMP_COLD, BATT_STATUS_COLD,     0}, 
	{CHG_TEMP_COLD, CHG_TEMP_COOL, BATT_STATUS_COOL1,  850}, 
	{CHG_TEMP_COOL, CHG_TEMP_GOOD, BATT_STATUS_COOL2, 2000},  
	{CHG_TEMP_GOOD, CHG_TEMP_WARM, BATT_STATUS_GOOD,  2800}, 
	{CHG_TEMP_WARM, CHG_TEMP_HOT,  BATT_STATUS_WARM,  2000}, 
	{CHG_TEMP_HOT,  CHG_TEMP_MAX,  BATT_STATUS_HOT,      0}, 
};
#endif

/******************************************************** 
 *					 I2C I/O function 				              *
 *********************************************************/

static int bq24192_i2c_readb(
		struct i2c_client *i2c,
		unsigned char  reg,
		unsigned char* buf)
{
	struct i2c_msg msgs[2];

	//write message: this is the sub address that we are reading from
	msgs[0].addr = i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg;

	//read message: now we read into the buffer
	msgs[1].addr = i2c->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

	if (i2c_transfer(i2c->adapter, msgs, 2) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.\n", __func__);
		return -4;
	}
	pr_debug("return  buf[0]=0x%x!\n",buf[0]);

	return 0;
}

//write bq24192 i2c function
static int bq24192_i2c_writeb(
		struct i2c_client *i2c,
		unsigned char reg, 
		unsigned char buf)
{
	struct i2c_msg msgs;
	char bufwr[2];

	bufwr[0] = reg;
	bufwr[1] = buf;

	//write message: this is the sub address that we are reading from
	msgs.addr = i2c->addr;
	msgs.flags = 0;
	msgs.len = 2;
	msgs.buf = bufwr;

	if (i2c_transfer(i2c->adapter, &msgs, 1) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.\n", __func__);
		return -4;
	}

	return 0;
}
/****************************************************************/

static int  bq24192_masked_write(struct bq24192_chg_chip *chip, u8 reg,
							u8 mask, u8 val)
{
	int rc;
	u8 buf;

	rc = bq24192_i2c_readb(chip->i2c,reg,&buf);
	if (rc) {
		pr_err("bq24192_i2c_readb failed: reg=0x%x, rc=%d\n", reg, rc);
		return rc;
	}
	
	buf &= ~mask;
	buf |= val & mask;

	rc = bq24192_i2c_writeb(chip->i2c,reg,buf);
	if (rc) {
		pr_err("bq24192_i2c_writeb failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	
	return 0;
}

static void bq24192_chg_gpio_enable(struct bq24192_chg_chip *chip,int enable)
{
    #ifdef CONFIG_ZTEMT_HW_VERSION_NX601J
    if(ztemt_get_hw_id() == NX601J_HW_A)
		gpio_set_value(chip->chg_en_gpio, !enable);
	else
		gpio_set_value(chip->chg_en_gpio, enable);
	#else
	    gpio_set_value(chip->chg_en_gpio, enable);
	#endif
}

static int ztemt_poweroffchg = 0;
static int  ztemt_power_off_chg(char *param)
{
	if(!strcmp(param,"charger"))
		ztemt_poweroffchg = 1;
	
	printk("%s:param=%s ztemt_poweroffchg=%d\n",__func__,param,ztemt_poweroffchg);
    return 0;
}
early_param("androidboot.mode", ztemt_power_off_chg);

//Reg 0x0 ---------------------------------------------------------------
static int  bq24192_hiz_mode_enable(struct bq24192_chg_chip *chip,int enable)
{
	int rc;
	u8 temp;

	temp = enable;
	temp = temp << BQ24192_HIZ_EN_SHIFT;
	
	rc = bq24192_masked_write(chip,BQ24192_REG_INPUT_LIMIT,BQ_HIZ_BIT,temp);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

static int  bq24192_is_hiz_mode(struct bq24192_chg_chip *chip)
{
	int rc;
	u8  buf;

	rc = bq24192_i2c_readb(chip->i2c,BQ24192_REG_INPUT_LIMIT,&buf);
    if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}

	buf &= BQ_HIZ_BIT;
	buf = buf >> BQ24192_HIZ_EN_SHIFT;
		
	return buf;
}

static int	bq24192_set_chg_iusb(struct bq24192_chg_chip *chip, enum input_current iusb)
{
	int rc;
	u8 temp;
	
	temp = iusb;
	rc = bq24192_masked_write(chip,BQ24192_REG_INPUT_LIMIT,BQ_LIMIT_I_MASK,temp);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

static int  bq24192_set_chg_voltage(struct bq24192_chg_chip *chip, int input_vol)
{
	u8 temp;

	if (input_vol < BQ24192_INPUTVOL_MIN_MV || input_vol > BQ24192_INPUTVOL_MAX_MV) {
		pr_err("bad current = %dmA asked to set\n", input_vol);
		return -EINVAL;
	}

	temp = (input_vol - BQ24192_INPUTVOL_MIN_MV)/BQ24192_INPUTVOL_STEP_MV;
	temp = temp << BQ24192_INPUTVOL_SHIFT;
	
	BQLOG_INFO("set iterm=0x%x\n",temp);
	return bq24192_masked_write(chip, BQ24192_REG_INPUT_LIMIT,BQ_INPUTVOL_MASK,temp);
}

//Reg 0x1 ---------------------------------------------------------------
int  bq24192_charge_config(enum charge_config chg_config)
{
	int rc;
	u8 temp;

    if (!bq_chip) {
		pr_err("%s:called before init\n",__func__);
		return -1;
	}
		
	temp = chg_config;
	temp = temp << BQ24192_CHG_EN_SHIFT;
	
	rc = bq24192_masked_write(bq_chip,BQ24192_REG_POWER_CONFIG,BQ_CHG_CONFIG_MASK,temp);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

#define BQ_RESET_BIT		BIT(7)
static int  bq24192_reset_regs(struct bq24192_chg_chip *chip)
{
    BQLOG_INFO("reset_regs\n");
	return bq24192_masked_write(chip, BQ24192_REG_POWER_CONFIG, BQ_RESET_BIT,BIT(7));
}

//Reg 0x2 ---------------------------------------------------------------
static int  bq24192_set_chg_ibatt(struct bq24192_chg_chip *chip, int ichg)
{
	u8 temp;

	if (ichg < BQ24192_CHG_I_MIN_MA || ichg > BQ24192_CHG_I_MAX_MA) {
		pr_err("bad current = %dmA asked to set\n", ichg);
		return -EINVAL;
	}

	temp = (ichg - BQ24192_CHG_I_MIN_MA)/BQ24192_CHG_I_STEP_MA;
	temp = temp << BQ24192_CHG_I_SHIFT;
	
	BQLOG_INFO("set ichg=0x%x\n",temp);
	return bq24192_masked_write(chip, BQ24192_REG_CURRENT_CNTL,BQ_FAST_I_MASK,temp);
}

//Reg 0x3 ---------------------------------------------------------------
static int  bq24192_set_chg_iterm(struct bq24192_chg_chip *chip, int iterm)
{
	u8 temp;

	if (iterm < BQ24192_ITERM_MIN_MA || iterm > BQ24192_ITERM_MAX_MA) {
		pr_err("bad current = %dmA asked to set\n", iterm);
		return -EINVAL;
	}

	temp = (iterm - BQ24192_ITERM_MIN_MA)/BQ24192_ITERM_STEP_MA;
	
	BQLOG_INFO("set iterm=0x%x\n",temp);
	return bq24192_masked_write(chip, BQ24192_REG_PRE_TERM_CHG,BQ_TERM_I_MASK,temp);
}

//Reg 0x4 ---------------------------------------------------------------
static int  bq24192_set_vbattmax(struct bq24192_chg_chip *chip, int vbatt)
{
	u8 temp;

	if (vbatt < BQ24192_CHG_MIN_VBATT || vbatt > BQ24192_CHG_MAX_VBATT) {
		pr_err("bad vbattmax = %dmV asked to set\n", vbatt);
		return -EINVAL;
	}

	temp = (vbatt - BQ24192_CHG_MIN_VBATT)/BQ24192_CHG_VBATT_STEP_MV;
	temp = temp << BQ24192_CHG_VBATT_SHIFT;
	
	BQLOG_DEBUG("set vbattmax=0x%x\n",temp);
	return bq24192_masked_write(chip, BQ24192_REG_VOL_CNTL,BQ_CHG_MV_MASK,temp);
}

//Reg 0x5 ---------------------------------------------------------------
static int  bq24192_set_wdog(struct bq24192_chg_chip *chip,enum wdog_time wtime)
{
	u8 temp;

	temp = wtime;
	temp = temp << BQ24192_WDOG_TIME_SHIFT;
	
	return bq24192_masked_write(chip, BQ24192_REG_TIME_CNTL,BQ_WTCHDOG_MASK,temp);
}

//Reg 0x6 ---------------------------------------------------------------
static int	bq24192_set_compen_resistor(struct bq24192_chg_chip *chip, enum compen_resistor comp_r)
{
	int rc;
	u8 temp;
	
	temp = comp_r << BQ24192_COMP_R_SHIFT;
	rc = bq24192_masked_write(chip,BQ24192_REG_IR_THERM_CNTL,BQ_IR_COMP_MASK,temp);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

static int	bq24192_set_compen_vol(struct bq24192_chg_chip *chip, int  comp_mv)
{
	int rc;
	u8 temp;

	if (comp_mv < BQ24192_COMP_MIN_MV || comp_mv > BQ24192_COMP_MAX_MV) {
		pr_err("bad current = %dmA asked to set\n", comp_mv);
		return -EINVAL;
	}

	temp = (comp_mv - BQ24192_COMP_MIN_MV)/BQ24192_COMP_STEP_MV;
	temp = temp << BQ24192_COMP_V_SHIFT;

	rc = bq24192_masked_write(chip,BQ24192_REG_IR_THERM_CNTL,BQ_IR_VOL_MASK,temp);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

//Reg 0xA ---------------------------------------------------------------
static int  bq24192_get_dev_info(struct bq24192_chg_chip *chip)
{
	int rc;
	u8 buf;

	rc = bq24192_i2c_readb(chip->i2c,BQ24192_REG_STATUS,&buf);
	if (rc) {
		pr_err("%s i2c read failed: rc=%d\n", __func__, rc);
		return rc;
	}
	
	buf &= BQ_PN_DEV_MASK;
	buf = buf >> BQ24192_DEV_INFO_SHIFT;
	return (int)buf;
}

static void bq24192_dump_regs(struct bq24192_chg_chip *chip)
{
	u8 buf;
	int i;

    if (PM_DEBUG > bqlog_level)
		return;
	for(i=0;i<11;i++){
		bq24192_i2c_readb(chip->i2c,i,&buf);
		printk("reg[%d] buf=0x%x\n",i,buf);
	}
}

static void bq24192_update_power_supply(struct bq24192_chg_chip *chip)
{
	if (chip->batt_psy == NULL || chip->batt_psy < 0)
		chip->batt_psy = power_supply_get_by_name("battery");

	if (chip->batt_psy > 0)
		power_supply_changed(chip->batt_psy);
}

static int bq24192_get_batt_param(struct bq24192_chg_chip *chip)
{
    union power_supply_propval ret = {0,};
	
	if (chip->batt_psy == NULL || chip->batt_psy < 0)
		chip->batt_psy = power_supply_get_by_name("battery");

	if(!chip->batt_psy)
		return -1;

	chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_TEMP,&ret);
	chip->batt_temp = ret.intval;
	
    chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_CURRENT_NOW,&ret);
	chip->batt_i = ret.intval/1000;
	
	chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_VOLTAGE_NOW,&ret);
	chip->batt_vol = ret.intval/1000;

	chip->chg_status = bq24192_get_chg_status();

    #ifdef CONFIG_ZTEMT_BATTERY_MAX17050
	chip->batt_soc = max17050_get_batt_soc();
	#else
	chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_CAPACITY,&ret);
	chip->batt_soc = ret.intval;
    #endif
	
	BQLOG_DEBUG("batt_temp=%d chg_status=%d  batt_i=%d  batt_vol=%d  batt_soc=%d\n",
		chip->batt_temp,chip->chg_status,chip->batt_i,chip->batt_vol,chip->batt_soc);

	return 0;
}
//battery  power supply property
int bq_prop_batt_status(void)
{
    int batt_st = bq24192_get_chg_status();

	if(batt_st == BQ_FAST_CHGING || batt_st == BQ_PRE_CHGING)
		return POWER_SUPPLY_STATUS_CHARGING;
	else if(batt_st == BQ_NOT_CHGING)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	else if(batt_st == BQ_CHGING_DONE)
		return POWER_SUPPLY_STATUS_FULL;
	else
		return POWER_SUPPLY_STATUS_UNKNOWN;
}

int bq_prop_charging_type(void)
{
    int batt_st = bq24192_get_chg_status();

	if(batt_st == BQ_FAST_CHGING)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if(batt_st == BQ_PRE_CHGING)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

int bq_prop_batt_health(void)
{
    int batt_status;
	batt_status = bq24192_get_batt_stauts();

	if(batt_status == BATT_STATUS_HOT)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if(batt_status == BATT_STATUS_COLD)
		return POWER_SUPPLY_HEALTH_COLD;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

//Module Interface---------------------------------------------------------------------- 
int bq24192_is_charger_online(void)
{
	if (!bq_chip) {
	    pr_err("%s:called before init\n",__func__);
		return 0;
	}

    return bq_chip->usb_in;
}

int bq24192_get_batt_stauts(void)
{
	int batt_temp;
	int batt_status;

	if (!bq_chip) {
	    pr_err("%s:called before init\n",__func__);
		return BATT_STATUS_GOOD;
	}

	if(!bq_chip->in_work){
		batt_temp = qpnp_get_battery_temp();
		bq24192_get_temp_status(batt_temp_map, ARRAY_SIZE(batt_temp_map), batt_temp, &batt_status);
		return batt_status;
	}

    return bq_chip->batt_status;
}

void bq24192_set_otg_stauts(int usb_in,int host_mode)
{
    if (!bq_chip) {
		pr_err("%s:called before init\n",__func__);
		return;
	}

	if(usb_in && !host_mode)
	    bq24192_chg_gpio_enable(bq_chip,1);
	else
		bq24192_chg_gpio_enable(bq_chip,0);
}

int  bq24192_input_current(enum input_current input_i)
{
    int rc;
	
	usbin_current = input_i;
    if (!bq_chip) {
		pr_err("%s:called before init\n",__func__);
		return -1;
	}
	
	BQLOG_INFO("input_i=0x%x\n",input_i);

	rc = bq24192_set_chg_iusb(bq_chip,input_i);
    if (rc) {
		pr_err("%s  failed: rc=%d\n", __func__,rc);
		return rc;
	}
	return 0;
}

static void bq24192_hvdcp_worker(struct work_struct *work)
{
	struct delayed_work *hwork = to_delayed_work(work);
	struct bq24192_chg_chip *chip = container_of(hwork,	struct bq24192_chg_chip,hvdcp_work);

	BQLOG_INFO("%s: chg_type=%d usb_in=%d\n",__func__,chip->chg_type,chip->usb_in);

    if(!chip->usb_in)
		return;
	
	if(!chip->temp_abnormal)
	    qpnp_dcdc_enable(0);

	if(chip->chg_type == BQ_DCP_CHARGER || chip->chg_type == BQ_CDP_CHARGER 
		  || chip->chg_type == BQ_PROPRIETARY_CHARGER)
		bq24192_input_current(INPUT_1500_MA);
}

int  bq24192_notify_charger(enum bq_chg_type chg_type)
{	
	BQLOG_INFO("chg->chg_type=%d\n",chg_type);
	
	if (!bq_chip) {
		pr_err("%s:called before init\n",__func__);
		return -1;
	}
	
	bq_chip->chg_type = chg_type;
	bq24192_input_current(INPUT_500_MA);
	
	if(chg_type && bq_chip->usb_in){
		BQLOG_INFO("schedule_delayed_work: bq_chip->usb_in=%d\n",bq_chip->usb_in);
    	schedule_delayed_work(&bq_chip->hvdcp_work,
		               round_jiffies_relative(msecs_to_jiffies(bq_chip->hvdcp_delay_time)));
	}
	return 0;
}
EXPORT_SYMBOL_GPL(bq24192_notify_charger);

int  bq24192_get_chg_status(void)
{
	int rc;
	u8  buf;

	if (!bq_chip) {
		pr_err("%s:called before init\n",__func__);
		return -1;
	}

	rc = bq24192_i2c_readb(bq_chip->i2c,BQ24192_REG_SYS_STATUS,&buf);
    if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}

	buf &= BQ_CHG_STAT_MASK;
	buf = buf >> BQ24192_CHG_STAT_SHIFT;
	
	return buf;
}

#define BATT_TEMTP_DELTA    20
#define TEMP_INIT    -500
static int bq24192_get_temp_status(const struct batt_status_map *pts,
		uint32_t tablesize, int input, int *batt_status)
{
	static int current_index = 0;
	static int init_status = 1;

	if ( pts == NULL || batt_status == NULL)
		return BATT_STATUS_UNKNOW;
		
	if(init_status){
		while (current_index < tablesize) {
			if ( (pts[current_index].low_temp <= input) && (input <= pts[current_index].high_temp) ) 
				break;
			else 
				current_index++;
		}
		init_status = 0;
		BQLOG_DEBUG("-First-input=%d  current_index=%d \n",input,current_index);
	}else{
		if(input < (pts[current_index].low_temp - BATT_TEMTP_DELTA))
			current_index--;
		else if(input > pts[current_index].high_temp)
			current_index++;
	}

    if(current_index < 0)
		*batt_status = BATT_STATUS_COLD;
	else if(current_index >= tablesize)
		*batt_status = BATT_STATUS_HOT;
	else
		*batt_status = pts[current_index].batt_st;

	BQLOG_DEBUG("input=%d  batt_status=%d \n",input,*batt_status);
	
	return current_index;

}

static void bq24192_chg_temp_cntl(struct bq24192_chg_chip *chip,int batt_temp)
{
    int battery_status;
	int batt_current = 0;
	int state_index;

	state_index = bq24192_get_temp_status(batt_temp_map,
		                                     ARRAY_SIZE(batt_temp_map),
		                                     batt_temp,
		                                     &battery_status);
	if(battery_status != BATT_STATUS_UNKNOW)
	    batt_current = batt_temp_map[state_index].batt_current;
		
	if(battery_status != chip->batt_status && chip->usb_in){
		BQLOG_INFO("last chip->batt_status=%d new_battery_status=%d\n",chip->batt_status,battery_status);
		if(batt_current > 0){
		    bq24192_hiz_mode_enable(chip,0);
			bq24192_set_chg_ibatt(chip,batt_current);
			chip->temp_abnormal = 0;
		    BQLOG_INFO("batt_temp=%d batt_status=%d batt_current=%d start charging...\n",batt_temp,battery_status,batt_current);
		}else{
			bq24192_hiz_mode_enable(chip,1);
			qpnp_dcdc_enable(1);
			chip->temp_abnormal = 1;
			BQLOG_INFO("batt_temp=%d out of rangge,stop charging!\n",batt_temp);
		}
		bq24192_update_power_supply(chip);
	}
	
	chip->batt_status = battery_status;

}

static void bq24192_recharging_cntl(struct bq24192_chg_chip *chip)
{
	if(chip->batt_soc == 100 && chip->batt_vol>4300 && chip->usb_in)
		chip->soc_chg_done = 1;
	else if(chip->batt_soc <= 96 || !chip->usb_in)
		chip->soc_chg_done = 0;
	
    if((chip->soc_chg_done || chip->chg_status == BQ_CHGING_DONE) ){
        if(chip->batt_vol<4300 || chip->batt_soc<=99){
			bq24192_hiz_mode_enable(chip,1);
			mdelay(3);
			bq24192_hiz_mode_enable(chip,0);
			chip->soc_chg_done = 0;
			BQLOG_INFO("batt_vol=%d  restart charging...\n",chip->batt_vol);
        }
    }
}

static void bq24192_hiz_mode_check(struct bq24192_chg_chip *chip)
{	
	if( bq24192_is_hiz_mode(chip) && chip->usb_in && !chip->temp_abnormal && chip->chg_status != BQ_CHGING_DONE){
		BQLOG_INFO("disable hiz_mode iusb=%d\n",usbin_current);
		bq24192_hiz_mode_enable(chip,0);
		bq24192_set_chg_iusb(chip,usbin_current);
	}
}

static void bq24192_chg_status_reset(struct bq24192_chg_chip *chip)
{	
	chip->temp_abnormal = 0;
	chip->soc_chg_done = 0;
	chip->is_chg_full = 0;
	chip->batt_status = BATT_STATUS_UNKNOW;
	chip->chg_type = BQ_INVALID_CHARGER;
}

#define START_CHG_MS	1000
#define CHG_PERIOD_MS	10000
#define MS_UNIT  1000000ULL
#define BQ_WDOG_RST_BIT		BIT(6)
static void bq24192_chg_worker(struct work_struct *work)
{
    int ret;
	struct delayed_work *cwork = to_delayed_work(work);
	struct bq24192_chg_chip *chip = container_of(cwork,	struct bq24192_chg_chip,chg_work);

	if(chip->in_work == 0){
        bq24192_set_wdog(chip,WDOG_TIMER_80S);
		chip->in_work = 1;
	}
	
	//kick the watchdog
    bq24192_masked_write(chip,BQ24192_REG_POWER_CONFIG,BQ_WDOG_RST_BIT,BIT(6));

	if(!chip->usb_in)
		goto out_work;

	ret = bq24192_get_batt_param(chip);
	if(ret < 0)
		goto next;
	
	//battery temperature check
	bq24192_chg_temp_cntl(chip,chip->batt_temp);

	//recharging status check
    if(!chip->temp_abnormal)
	    bq24192_recharging_cntl(chip);

    //pm dcdc status check
	if( chip->chg_status == BQ_CHGING_DONE || chip->chg_status == BQ_NOT_CHGING  || chip->batt_i<0 )
		qpnp_dcdc_enable(1);

	//charge full check
	if(chip->chg_status == BQ_CHGING_DONE){
		if(!chip->is_chg_full){
			BQLOG_INFO("charging is full.\n");
			bq24192_update_power_supply(chip);
			chip->is_chg_full = 1;
		}
	}else
	    chip->is_chg_full = 0;

    //to avoid  reseting vbatmax because of watchdog timeout
	if(chip->batt_vol > 4100)
		bq24192_set_vbattmax(chip,chip->vbatt_max);

	//hiz mode check
	bq24192_hiz_mode_check(chip);

	#ifdef CONFIG_SLIMPORT_FAST_CHARGE
	if(sp_get_ds_charge_type() == FAST_CHARGING && usbin_current != INPUT_1500_MA){
        bq24192_input_current(INPUT_1500_MA);
	}
	#endif

    bq24192_dump_regs(chip);
	BQLOG_INFO("usb_in=%d  hvdcp=%d status=%d\n",chip->usb_in,qpnp_is_hvdcp_charger(),chip->chg_status);
	
next:
	schedule_delayed_work(&chip->chg_work,
		round_jiffies_relative(msecs_to_jiffies(CHG_PERIOD_MS)));
	return;

out_work:
	bq24192_set_wdog(chip,WDOG_DISABLE);
	bq24192_input_current(INPUT_100_MA);
	#ifdef CONFIG_SLIMPORT_FAST_CHARGE
	sp_set_ds_charge_type(NO_CHARGING);
    #endif
	bq24192_chg_status_reset(chip);
	chip->in_work = 0;
	wake_unlock(&chip->wlock);

}

int  bq24192_set_chg_status(int usb_in)
{
	BQLOG_INFO("usb_in=%d\n",usb_in);
		
	if (!bq_chip) {
		pr_err("%s:called before init\n",__func__);
		charger_online = usb_in;
		return -1;
	}  

    bq_chip->usb_in = usb_in;  
	
	if(usb_in){
		bq24192_chg_gpio_enable(bq_chip,1);
		wake_lock(&bq_chip->wlock);
		schedule_delayed_work(&bq_chip->chg_work,
			   round_jiffies_relative(msecs_to_jiffies(START_CHG_MS)));
	}else
	    bq24192_chg_gpio_enable(bq_chip,0);
	
 	return 0;
}
EXPORT_SYMBOL_GPL(bq24192_set_chg_status);


static int debug_reg;
module_param(debug_reg, int, 0644);

static int bq24192_debug;
static int bq24192_debug_mode(const char *val, struct kernel_param *kp)
{
	int ret;
    int i;
    u8 buf;
	
	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	
	printk("__%s: bq24192_debug=%d!\n",__func__,bq24192_debug);

	if(bq24192_debug < 0){
		buf = abs(bq24192_debug);
        bq24192_i2c_writeb(bq_chip->i2c,debug_reg,buf);
		return 0;
	}
		
	switch(bq24192_debug){
    case 0:
		bq24192_charge_config(0);
		break;
	case 1:
		bq24192_charge_config(1);
		break;
	case 2:
		bq24192_set_chg_ibatt(bq_chip,ibat_current);
		break;
	case 3:
		bq24192_get_dev_info(bq_chip);
		break;
	case 4:
		for(i=0;i<11;i++){
			bq24192_i2c_readb(bq_chip->i2c,i,&buf);
            printk("---reg[%d] buf=0x%x\n",i,buf);
		}
		break;
    case 5:
		bq24192_reset_regs(bq_chip);
		break;
	case 6:
		bq24192_input_current(usbin_current);
		break;
	case 7:
		bq24192_hiz_mode_enable(bq_chip,0);
		break;
	case 8:
		bq24192_hiz_mode_enable(bq_chip,1);
		break;
	case 9:
		bq24192_i2c_writeb(bq_chip->i2c,debug_reg,buf);
		break;
	case 10:
		gpio_set_value(bq_chip->chg_en_gpio, 0);
		break;
	case 11:
		gpio_set_value(bq_chip->chg_en_gpio, 1);
		break;
	default:
		break;
	};
	
	return 0;
}
module_param_call(bq24192_debug, bq24192_debug_mode, param_get_uint,
					&bq24192_debug, 0644);

static int
bq24192_charger_read_dt_props(struct bq24192_chg_chip *chip)
{
    int rc;
	
    chip->chg_en_gpio = of_get_named_gpio(chip->dev_node, "bq24192-enable-gpio", 0);
	chip->otg_gpio = of_get_named_gpio(chip->dev_node, "bq24192-otg-gpio", 0);
	chip->psel_gpio = of_get_named_gpio(chip->dev_node, "bq24192-psel-gpio", 0);
	chip->irq_gpio = of_get_named_gpio(chip->dev_node, "bq24192-irq-gpio", 0);

	rc = of_property_read_u32(chip->dev_node, "bq-ibatmax-ma", &chip->ibatmax_ma);
	if (rc) {
		pr_err( "Unable to parse 'bq-ibatmax-ma'\n");
		return rc;
	}

	rc = of_property_read_u32(chip->dev_node, "bq-vusb-min", &chip->vusb_min);
	if (rc) {
		pr_err( "Unable to parse 'bq-vusb-min'\n");
		return rc;
	}

	rc = of_property_read_u32(chip->dev_node, "bq-iterm-ma", &chip->iterm_ma);
	if (rc) {
		pr_err( "Unable to parse 'bq-iterm-ma'\n");
		return rc;
	}

	rc = of_property_read_u32(chip->dev_node, "bq-initusb-ma", &chip->iusb_init_ma);
	if (rc) {
		pr_err( "Unable to parse 'bq-initusb-ma'\n");
		return rc;
	}

	rc = of_property_read_u32(chip->dev_node, "bq-vbatmax-mv", &chip->vbatt_max);
	if (rc) {
		pr_err( "Unable to parse 'bq-vbatmax-mv'\n");
		return rc;
	}
	
	rc = of_property_read_u32(chip->dev_node, "bq-compen-mohm", &chip->compen_mohm);
	if (rc) {
		pr_err( "Unable to parse 'bq-compen-mohm'\n");
		return rc;
	}

	rc = of_property_read_u32(chip->dev_node, "bq-compen-mv", &chip->compen_mv);
	if (rc) {
		pr_err( "Unable to parse 'bq-compen-mv'\n");
		return rc;
	}

	return 0;
}
#if 0
static irqreturn_t bq24192_chg_status_handler(int irq, void *_chip)
{

	return IRQ_HANDLED;
}
#endif

static int bq24192_chg_hw_init(struct bq24192_chg_chip *chip)
{
    int ret;

    ret = bq24192_set_wdog(chip,WDOG_DISABLE);
	if (ret) {
		pr_err("failed disable hiz mode rc=%d\n", ret);
	}
	//chg_en gpio
	ret = gpio_request(chip->chg_en_gpio, "bq24192_chg_en");
	if (ret) {
		pr_err("%s: fail gpio_request(%d)=%d\n", __func__,chip->chg_en_gpio, ret);
	}
	gpio_tlmm_config(GPIO_CFG(chip->chg_en_gpio, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	bq24192_chg_gpio_enable(chip,0);

    //irq  gpio
    #if 0
    ret = request_irq(chip->irq_gpio, bq24192_chg_status_handler, 
			IRQF_TRIGGER_FALLING , "bq24192_chg_status", chip);
	if (ret) {
		pr_err("Unable to allocate bq24192_chg_status interrupt!\n");
	}
	enable_irq_wake(chip->irq_gpio);
	#else
	ret = gpio_request(chip->irq_gpio, "bq24192_chg_int");
	if (ret) {
		pr_err("%s: fail gpio_request(%d)=%d\n", __func__,chip->irq_gpio, ret);
	}
	gpio_tlmm_config(GPIO_CFG(chip->irq_gpio, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	#endif
	//otg  gpio
	ret = gpio_request(chip->otg_gpio, "bq24192_otg");
	if (ret) {
		pr_err("%s: fail gpio_request(%d)=%d\n", __func__,chip->otg_gpio, ret);
	}
	gpio_tlmm_config(GPIO_CFG(chip->otg_gpio, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
    //psel  gpio
	ret = gpio_request(chip->psel_gpio, "bq24192_psel");
	if (ret) {
		pr_err("%s: fail gpio_request(%d)=%d\n", __func__,chip->psel_gpio, ret);
	}
	gpio_tlmm_config(GPIO_CFG(chip->psel_gpio, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);

	ret = bq24192_hiz_mode_enable(chip,0);
	if (ret) {
		pr_err("failed disable hiz mode rc=%d\n", ret);
		return ret;
	}

    ret = bq24192_set_chg_voltage(chip,chip->vusb_min);
	if (ret) {
		pr_err("failed set charging voltage rc=%d\n", ret);
		return ret;
	}	
	
	ret = bq24192_set_chg_ibatt(chip,chip->ibatmax_ma);
	if (ret) {
		pr_err("failed set charging ibatt rc=%d\n", ret);
		return ret;
	}

	ret = bq24192_set_chg_iusb(chip,chip->iusb_init_ma);
	if (ret) {
		pr_err("failed set charging iusb rc=%d\n", ret);
		return ret;
	}
	
    ret = bq24192_set_chg_iterm(chip,chip->iterm_ma);
	if (ret) {
		pr_err("failed set charging iterm rc=%d\n", ret);
		return ret;
	}

	ret = bq24192_set_vbattmax(chip,chip->vbatt_max);
	if (ret) {
		pr_err("failed set charging vbattmax rc=%d\n", ret);
		return ret;
	}

	ret = bq24192_set_compen_resistor(chip,chip->compen_mohm);
	if (ret) {
		pr_err("failed set compen_mohm rc=%d\n", ret);
		return ret;
	}

	ret = bq24192_set_compen_vol(chip,chip->compen_mv);
	if (ret) {
		pr_err("failed set compen_mv rc=%d\n", ret);
		return ret;
	}

	
	return 0;
}

static int  bq24192_charger_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	struct bq24192_chg_chip *chip;
	int ret;

	chip = kzalloc(sizeof(struct bq24192_chg_chip),	GFP_KERNEL);
	if (!chip) {
		pr_err("Cannot allocate bq24192_chg_chip\n");
		return -ENOMEM;
	}
	
    BQLOG_INFO("enter driver probe:\n");
	
	chip->i2c = client;
	chip->dev_node = client->dev.of_node;
	chip->in_work = 0;
	chip->temp_abnormal = 0;
	chip->in_rechging = 0;
	chip->batt_status = BATT_STATUS_UNKNOW;
	if(ztemt_poweroffchg)
		chip->hvdcp_delay_time = 60*1000;
	else
		chip->hvdcp_delay_time = 5*1000;
	
	chip->vadc_dev = qpnp_get_vadc(&chip->i2c->dev, "bq24192");
	if (IS_ERR(chip->vadc_dev)) {
		ret = PTR_ERR(chip->vadc_dev);
		if (ret != -EPROBE_DEFER)
		    pr_err("vadc property missing\n");
	}
	bq_chip = chip;
	
	bq24192_charger_read_dt_props(chip);

	ret = bq24192_chg_hw_init(chip);
    if (ret) {
		pr_err("failed to intit bq24192 hw %d\n", ret);
	}
	wake_lock_init(&chip->wlock, WAKE_LOCK_SUSPEND, "bq24192_charger");
	
	INIT_DELAYED_WORK(&chip->chg_work, bq24192_chg_worker);
	INIT_DELAYED_WORK(&chip->hvdcp_work, bq24192_hvdcp_worker);

	ret = bq24192_charge_config(CHG_ENABLE);
	if (ret) 
		pr_err("failed enable charging rc=%d\n", ret);

	if(charger_online){
		wake_lock(&bq_chip->wlock);
		schedule_delayed_work(&bq_chip->chg_work,
			   round_jiffies_relative(msecs_to_jiffies(START_CHG_MS)));
	}

    return 0;
}

static int bq24192_charger_remove(struct i2c_client *client)
{	
	kfree(bq_chip);
	bq_chip = NULL;

	return 0;
}

static int bq24192_suspend(struct i2c_client *cl, pm_message_t mesg)
{
	BQLOG_DEBUG(" suspend:\n");
	if(bq_chip)
		bq24192_set_wdog(bq_chip,WDOG_DISABLE);
	return 0;
};

static int bq24192_resume(struct i2c_client *cl)
{
	BQLOG_DEBUG(" resume:\n");

	return 0;
};

static struct of_device_id bq_24192_match_table[] = {
	{ .compatible = "ti,bq24192",},
	{}
};

static const struct i2c_device_id bq24192_id[] = {
	{ "bq24192", 1 },
	{},
};

static struct i2c_driver bq24192_charger_driver = {
	.driver = {
		.name = "bq24192",
		.of_match_table = bq_24192_match_table,
	},
	.id_table 	= bq24192_id,
	.probe 		= bq24192_charger_probe,
	.remove 	= bq24192_charger_remove,

	.suspend	= bq24192_suspend,
	.resume 	= bq24192_resume,
};

static int __init bq24192_charger_init(void)
{
	printk( "%s:enter...\n", __func__);

	return i2c_add_driver(&bq24192_charger_driver);
}

static void __exit bq24192_charger_exit(void)
{
	printk( "%s:bq24192 is exiting\n", __func__);

	i2c_del_driver(&bq24192_charger_driver);
}

module_init(bq24192_charger_init);


module_exit(bq24192_charger_exit);

MODULE_AUTHOR("ztemt-swang<wang.shuai12@zte.com.cn>");
MODULE_DESCRIPTION("bq24192 charger driver");
MODULE_LICENSE("GPL");
