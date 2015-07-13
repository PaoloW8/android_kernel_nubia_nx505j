#ifndef _ZTE_DISP_ENHANCE_
#define _ZTE_DISP_ENHANCE_

#include "mdss_dsi.h"

#define ZTE_DEFALUT_MAX_BLLEVEL 4095

enum {
	INTENSITY_00 = 24,
	INTENSITY_01,
	INTENSITY_02
};
struct zte_enhance_type{
  int en_saturation;
  int en_colortmp;
  unsigned int saturation;
  unsigned int colortmp;
};

struct zte_enhance_type zte_get_lcd_enhance_param(void);

void zte_set_ctrl_point(struct mdss_dsi_ctrl_pdata * ctrl);
void zte_mipi_saturation(void);
void zte_mipi_colortmp(void);
void zte_boot_begin_enhance(struct mdss_dsi_ctrl_pdata *ctrl);
#endif 

