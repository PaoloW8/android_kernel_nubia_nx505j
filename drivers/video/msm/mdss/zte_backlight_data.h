#ifndef H_ZTE_BACKLIGHT_DATA
#define H_ZTE_BACKLIGHT_DATA

#include "zte_backlight.h"

//this table maped 0-255 bright set from use space to the true brightness show in machine, 
#if defined(CONFIG_ZTEMT_MIPI_1080P_R63311_SHARP_IPS_6P4)
const struct BGVALUE_BRIGHTNESS_TYPE bgvalue2brightness[] = \
{
	//{0,4},
	//{1,4},
	//{2,4},
	//{4,8},
	{0,20},
	{4,20},
	{5,20},
	//{8,16},
	{10,20},
	{25,50},
	{45,90},
	{58,115},
	{65,130},
	{110,220},
	{160,320},
	{190,380},//modify for using 380 for the bigest brightness
	//{255,380},
	{210,420},
	{223,446},
	//{240,480},
	{255,446}
};

const struct BRIGHTNESS_PWM_TYPE brightness2pwm_normal[] = \
{
	{4,51},
	{8,87},
	{16,158},
	{50,452},
	{90,800},
	{115,1020},
	{130,1153},
	{220,1963},
	{320,2885},
	{380,3452},
	{420,3840},
	{446,4095}
};
#elif defined(CONFIG_ZTEMT_MIPI_1080P_R63311_SHARP_IPS_5P5)
const struct BGVALUE_BRIGHTNESS_TYPE bgvalue2brightness[] = \
{
	{0,4},
	{1,4},
	{2,4},
	{4,8},
	//{0,10},
	//{4,10},
	//{5,10},
	{8,16},
	{25,50},
	{45,90},
	{58,115},
	{65,130},
	{110,220},
	{160,320},
	{190,380},//modify for using 380 for the bigest brightness
	//{255,380},
	{210,420},
	{223,446},
	//{240,480},
	{255,446}
};

const struct BRIGHTNESS_PWM_TYPE brightness2pwm_normal[] = \
{
	{2,20},
	{4,40},
	{8,74},
	{16,140},
	{50,411},
	{90,726},
	{115,925},
	{130,1045},
	{220,1772},
	{320,2608},
	{380,3129},
	{420,3486},
	{450,3763},
	{487,4095}
};
#elif defined(CONFIG_ZTEMT_MIPI_1080P_R63417_SHARP_IPS_5P5)
const struct BGVALUE_BRIGHTNESS_TYPE bgvalue2brightness[] = \
{
    {0,20},
    {1,20},
    {2,20},
    {4,20},
    {8,20},
    {25,50},
    {45,90},
    {58,115},
    {65,130},
    {110,220},
    {160,320},
    {190,380},//modify for using 380 for the bigest brightness
    //{255,380},
    {210,420},
    {223,446},
    //{240,480},
    {255,446}
};

const struct BRIGHTNESS_PWM_TYPE brightness2pwm_normal[] = \
{
    {4,32},
    {10,72},
    {16,110},
    {20,134},
    {50,313},
    {90,548},
    {115,697},
    {130,785},
    {220,1330},
    {320,1950},
    {380,2320},
    {420,2585},
    {446,2754}
};
#elif defined(CONFIG_ZTEMT_MIPI_2K_R63419_SHARP_IPS_5P5)
#if 0
const struct BGVALUE_BRIGHTNESS_TYPE bgvalue2brightness[] = \
{
	{0,4},
	{1,4},
	{2,4},
	{4,8},
	{8,16},
	{25,50},
	{45,90},
	{58,115},
	{65,130},
	{110,220},
	{160,320},
	{190,380},//modify for using 380 for the bigest brightness
	{210,420},
	{223,446},
	{255,446}
};

const struct BRIGHTNESS_PWM_TYPE brightness2pwm_normal[] = \
{
	{2,20},
	{4,40},
	{8,74},
	{16,140},
	{50,411},
	{90,726},
	{115,925},
	{130,1045},
	{220,1772},
	{320,2608},
	{380,3129},
	{420,3486},
	{450,3763},
	{487,4095}
};
#endif
#endif

#endif
