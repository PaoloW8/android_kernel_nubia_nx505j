#ifndef __LINUX_BQ24194_CHARGER_H__
#define __LINUX_BQ24194_CHARGER_H__

//regs
#define BQ24192_REG_INPUT_LIMIT		0x00
#define BQ24192_REG_POWER_CONFIG		0x01
#define BQ24192_REG_CURRENT_CNTL		0x02
#define BQ24192_REG_PRE_TERM_CHG		0x03
#define BQ24192_REG_VOL_CNTL		0x04
#define BQ24192_REG_TIME_CNTL		0x05
#define BQ24192_REG_IR_THERM_CNTL		0x06
#define BQ24192_REG_MISC_CNTL		0x07
#define BQ24192_REG_SYS_STATUS		0x08
#define BQ24192_REG_FAULT		0x09
#define BQ24192_REG_STATUS		0x0A


//REG 0x0
#define BQ_HIZ_BIT		BIT(7)
#define BQ24192_HIZ_EN_SHIFT		7

#define BQ24192_INPUTVOL_MIN_MV	  3880
#define BQ24192_INPUTVOL_MAX_MV	  5080
#define BQ24192_INPUTVOL_STEP_MV    80
#define BQ24192_INPUTVOL_SHIFT		 3
#define BQ_INPUTVOL_MASK		  0x78

#define BQ_LIMIT_I_MASK		0x07

//REG 0x1
#define BQ_CHG_CONFIG_MASK		0x30
#define BQ24192_CHG_EN_SHIFT   4

//REG 0x2
#define BQ24192_CHG_I_MIN_MA	  512
#define BQ24192_CHG_I_MAX_MA	  4544
#define BQ24192_CHG_I_STEP_MA	   64
#define BQ_FAST_I_MASK		0xFC
#define BQ24192_CHG_I_SHIFT	       2

//REG 0x3
#define BQ24192_ITERM_MIN_MA	  128
#define BQ24192_ITERM_MAX_MA	  2048
#define BQ24192_ITERM_STEP_MA	   128
#define BQ_TERM_I_MASK		0x0F

//REG 0x4
#define BQ24192_CHG_MIN_VBATT	  3504
#define BQ24192_CHG_MAX_VBATT	  4400
#define BQ24192_CHG_VBATT_STEP_MV	   16
#define BQ_CHG_MV_MASK		0xFC
#define BQ24192_CHG_VBATT_SHIFT	       2

//REG 0x5
#define BQ_WTCHDOG_MASK		0x30
#define BQ24192_WDOG_TIME_SHIFT	       4

//REG 0x6
#define BQ_IR_COMP_MASK		0xE0
#define BQ24192_COMP_R_SHIFT    5

#define BQ24192_COMP_MIN_MV	  0
#define BQ24192_COMP_MAX_MV	  112
#define BQ24192_COMP_STEP_MV   16
#define BQ24192_COMP_V_SHIFT    2
#define BQ_IR_VOL_MASK		0x1C

//REG 0x8
#define BQ_CHG_STAT_MASK		0x30
#define BQ24192_CHG_STAT_SHIFT	       4

//REG 0xA
#define BQ24192_DEV_INFO_SHIFT   3

//0.1 degree   eg:470 = 47C
#define CHG_TEMP_MAX    800
#define CHG_TEMP_HOT    500
#define CHG_TEMP_WARM   450
#define CHG_TEMP_GOOD   230
#define CHG_TEMP_COOL   100
#define CHG_TEMP_COLD   -50
#define CHG_TEMP_MIN   -300

enum input_current {
	INPUT_100_MA,
	INPUT_150_MA,
	INPUT_500_MA,
	INPUT_900_MA,
	INPUT_1200_MA,
	INPUT_1500_MA,
	INPUT_2000_MA,
	INPUT_3000_MA,
};

enum wdog_time {
	WDOG_DISABLE,
	WDOG_TIMER_40S,
	WDOG_TIMER_80S,
	WDOG_TIMER_1600S,
};

enum charge_config {
	CHG_DISABLE,
	CHG_ENABLE,
	CHG_OTG,
};

enum bq_chg_status{
	BQ_NOT_CHGING,
	BQ_PRE_CHGING,
	BQ_FAST_CHGING,
	BQ_CHGING_DONE,
};

enum bq_batt_status{
	BATT_STATUS_COLD = 0,
	BATT_STATUS_COOL1,
	BATT_STATUS_COOL2,
	BATT_STATUS_GOOD,
	BATT_STATUS_WARM,
	BATT_STATUS_HOT,
	BATT_STATUS_UNKNOW,
};

enum compen_resistor {
	COMP_0_MOHM,
	COMP_10_MOHM,
	COMP_20_MOHM,
	COMP_30_MOHM,
	COMP_40_MOHM,
	COMP_50_MOHM,
	COMP_60_MOHM,
	COMP_70_MOHM,
};

enum bq_chg_type {
	BQ_INVALID_CHARGER = 0,
	BQ_SDP_CHARGER,
	BQ_DCP_CHARGER,
	BQ_CDP_CHARGER,
	BQ_PROPRIETARY_CHARGER,
	BQ_FLOATED_CHARGER,
};

struct batt_status_map {
	int low_temp;
	int high_temp;
	enum bq_batt_status batt_st;
	int batt_current;
};

int  bq24192_get_chg_status(void);

int  bq24192_set_chg_status(int usb_in);

int  bq24192_charge_config(enum charge_config);

int  bq24192_input_current(enum input_current input_i);

int  ztemt_get_hw_id(void);

int qpnp_is_hvdcp_charger(void);

void bq24192_set_otg_stauts(int usb_in,int host_mode);

int  bq24192_notify_charger(enum bq_chg_type chg_type);

int bq24192_get_batt_stauts(void);

int bq24192_is_charger_online(void);

int bq_prop_batt_status(void);

int bq_prop_charging_type(void);

int bq_prop_batt_health(void);

#endif
