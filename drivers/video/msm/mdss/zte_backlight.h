#ifndef H_ZTE_BACKLIGHT
#define H_ZTE_BACKLIGHT

#include "mdss_dsi.h"

//#define ZTE_BACKLIGHT_DEBUG_ENABLE

#ifdef ZTE_BACKLIGHT_DEBUG_ENABLE
#define ZTE_BACKLIGHT_DEBUG  printk
#else
#define ZTE_BACKLIGHT_DEBUG(fmt,...)
#endif

struct BGVALUE_BRIGHTNESS_TYPE{
	int bgvalue;
	int brightness;
};

struct BRIGHTNESS_PWM_TYPE{
	int brightness;
	int pwm;
};

void ztePwmBri_SetPoint(struct mdss_dsi_ctrl_pdata * ctrl);

bool IsBgvalueTranEnable(void);

int ZteBgvalue2pwm(int bgvalue);
//#endif
#endif
