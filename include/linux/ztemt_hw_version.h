#ifndef __ZTEMT_HW_VERSION_H__
#define __ZTEMT_HW_VERSION_H__

#ifdef CONFIG_ZTEMT_HW_VERSION_NX601J
typedef enum
{
	NX601J_HW_A,
	NX601J_HW_B,
	NX601J_HW_C,
	NX601J_HW_D,
	NX601J_HW_E,
	NX601J_HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#elif defined CONFIG_ZTEMT_HW_VERSION_NX504J
typedef enum
{
	NX504J_HW_A,
	NX504J_HW_B,
	NX504J_HW_C,
	NX504J_HW_D,
	NX504J_HW_E,
	NX504J_HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#elif defined CONFIG_ZTEMT_HW_VERSION_NX505J
typedef enum
{
	NX505J_HW_A,
	NX505J_HW_B,
	NX505J_HW_C,
	NX505J_HW_D,
	NX505J_HW_E,
	NX505J_HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#elif defined CONFIG_ZTEMT_HW_VERSION_NX506J
typedef enum
{
	NX506J_HW_A,
	NX506J_HW_B,
	NX506J_HW_C,
	NX506J_HW_D,
	NX506J_HW_E,
	NX506J_HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#elif defined CONFIG_ZTEMT_HW_VERSION_NX507J
typedef enum
{
	NX507J_HW_A,
	NX507J_HW_B,
	NX507J_HW_C,
	NX507J_HW_D,
	NX507J_HW_E,
	NX507J_HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#elif defined CONFIG_ZTEMT_HW_VERSION_NX514J
typedef enum
{
	NX514J_HW_A,
	NX514J_HW_B,
	NX514J_HW_C,
	NX514J_HW_D,
	NX514J_HW_E,
	NX514J_HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#else
typedef enum
{
	HW_A,
	HW_B,
	HW_C,
	HW_D,
	HW_E,
	HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#endif

#ifdef CONFIG_ZTEMT_HW_VERSION_NX507J
struct hardware_id_map_st {
	int low_mv;
	int high_mv;
	int low_mv_2;
	int high_mv_2;
	hw_version_type hw_type;
	char hw_ver[20];
	char hw_sc[20];
};
#else
struct hardware_id_map_st {
	int low_mv;
	int high_mv;
	hw_version_type hw_type;
	char hw_ver[20];
};
#endif

#endif
