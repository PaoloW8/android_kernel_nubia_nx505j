//********************************************************************************
//
//		<< LC898109 Evaluation Soft>>
//		Program Name	: Ois.h
// 		Explanation		: LC898109 Global Declaration & ProtType Declaration
//		Design			: Y.Yamada
//		History			: First edition						2009.07.30 Y.Tashita
//********************************************************************************
#include "msm_ois.h"
#define	FW_VER			0x000D

#ifdef	OISINI
	#define	OISINI__
#else
	#define	OISINI__		extern
#endif







#ifdef	OISCMD
	#define	OISCMD__
#else
	#define	OISCMD__		extern
#endif


// Define According To Usage

/****************************** Define説明 ******************************/
/*	USE_EXE2PROM		外部EEPROM使用									*/
/*		I2CE2PROM		(I2C EEPROM使用) *SPI or I2C共存できない		*/
/*		SPIE2PROM		(SPI EEPROM使用) *SPI or I2C共存できない		*/
/*	CIRC_MOVE			円動作制御使用									*/
/*	STANDBY_MODE		Standby制御使用(未確認)							*/
/*	GAIN_CONT			:Gain control機能使用							*/
/*						:三脚Mode使用									*/
/*	LOW_NOISE			Driver 低Noise Mode/従来PWM						*/
/*						電流変化を直線補正する処理必須(Fc設定必要)		*/
/*	HALLADJ_HW			Hall Calibration LSI機能使用					*/
/************************************************************************/

//#define		USE_EXE2PROM		// Use Eeprom
#ifdef	USE_EXE2PROM
 //#define		I2CE2PROM			// Ex I2CE2PROM I/F
 #define		SPIE2PROM			// Ex SPIE2PROM I/F
#endif

/**/
#define		CIRC_MOVE			// enable Circular movement
#define		STANDBY_MODE		// STANDBY Mode

#define		GAIN_CONT			// Gain Control Mode

	/* (0.5X^3+0.4453261X)*(0.5X^3+0.4453261X) */
#define	LOWCURRENT			// Test Low current mode confirm (750Hz/Break/MODE&ENA dis)

//#define		HALLADJ_HW			// H/W Hall adjustment 

#define		TESTPNTL				/* Test for PAN/TILT	*/
#define		NEUTRAL_CENTER
//#define		MODULE_CALIBRATION		/* calibration code for Module maker */
#define		H1COEF_CHANGER			/* H1 coef lvl chage */

// Command Status
#define		EXE_END		0x02		// Execute End (Adjust OK)
#define		EXE_HXADJ	0x06		// Adjust NG : X Hall NG (Gain or Offset)
#define		EXE_HYADJ	0x0A		// Adjust NG : Y Hall NG (Gain or Offset)
#define		EXE_LXADJ	0x12		// Adjust NG : X Loop NG (Gain)
#define		EXE_LYADJ	0x22		// Adjust NG : Y Loop NG (Gain)
#define		EXE_GXADJ	0x42		// Adjust NG : X Gyro NG (offset)
#define		EXE_GYADJ	0x82		// Adjust NG : Y Gyro NG (offset)
#define		EXE_OCADJ	0x402		// Adjust NG : OSC Clock NG
#define		EXE_ERR		0x99		// Execute Error End

// Common Define
#define	SUCCESS			0x00		// Success
#define	FAILURE			0x01		// Failure

#ifndef ON
 #define	ON				0x01		// ON
 #define	OFF				0x00		// OFF
#endif
 #define	SPC				0x02		// Special

#define	X_DIR			0x00		// X Direction
#define	Y_DIR			0x01		// Y Direction
#define	X2_DIR			0x10		// X Direction
#define	Y2_DIR			0x11		// Y Direction

#define	NOP_TIME		0.00004166F

#ifdef STANDBY_MODE
 // Standby mode
 #define		STB1_ON		0x00		// Standby1 ON
 #define		STB1_OFF	0x01		// Standby1 OFF
 #define		STB2_ON		0x02		// Standby2 ON
 #define		STB2_OFF	0x03		// Standby2 OFF
 #define		STB3_ON		0x04		// Standby3 ON
 #define		STB3_OFF	0x05		// Standby3 OFF
 #define		STB4_ON		0x06		// Standby4 ON			/* for Digital Gyro Read */
 #define		STB4_OFF	0x07		// Standby4 OFF
 #define		STB5_LOP	0x08		// Standby5 OFF<->ON
OISINI__	unsigned char	UcStbySt ;	/* Standby State */
 #define		STBYST_OFF	0x00
 #define		STBYST_ON	0x01
#endif


// Adjust Parameter
 #define		DAHLXO_INI		0x4DDF
 #define		DAHLXB_INI		0xCE01
 #define		DAHLYO_INI		0xD94B
 #define		DAHLYB_INI		0xC001
 #define		LXGAIN_INI		0x4513
 #define		LYGAIN_INI		0x46F4
 #define		ADHXOFF_INI		0x0084
 #define		ADHYOFF_INI		0x002C
 #define		BAIS_CUR		0x11
 #define		AMP_GAIN		0x44

 #define		OSC_INI			0x91		/* VDD=2.8V */

// Digital Gyro offset Initial value 
 #define		DGYRO_OFST_XH	0x00
 #define		DGYRO_OFST_XL	0x32
 #define		DGYRO_OFST_YH	0x00
 #define		DGYRO_OFST_YL	0x91

 #define		OPTCEN_X		0x0000
 #define		OPTCEN_Y		0x0000


 #define		GYROX_INI		0x45
 #define		GYROY_INI		0x43
 
 #define		PZGXP_INI		0x8001
 #define		PZGYP_INI		0x8001

 #define		GXGAIN_INI		0x3F04910A
 #define		GYGAIN_INI		0xBF233548

#define		LXGAIN_LOP		0x3000
#define		LYGAIN_LOP		0x3000

#define		GYRO_LMT4L		0x40400000		/* 3.0 */
#define		GYRO_LMT4H		0x40400000		/* 3.0 */
 
#define		GYRA12_HGH		0x40000000		/* 2.00F */
#define		GYRA12_MID		0x3F19999A		/* 0.6F */
#define		GYRA34_HGH		0x3F000000		/* 0.5F */
#define		GYRA34_MID		0x3DCCCCCD		/* 0.1F */

#define		GYRB12_HGH		0x3E4CCCCD		/* 0.20F */
#define		GYRB12_MID		0x3CA3D70A		/* 0.02F */
#define		GYRB34_HGH		0x3CA3D70A		/* 0.02F */
#define		GYRB34_MID		0x3C23D70A		/* 0.001F */


#ifdef I2CE2PROM									/* 0x0000-0x01FF	*/
 #define	E2POFST				0x0000				/* 0x0000-			*/
#endif
#ifdef SPIE2PROM									/* 0x0000-0x1FFF	*/
 #define	E2POFST				0x1000				/* 0x1000-0x1FFF	*/
#endif
// EEPROM Address Define	0000_01FF
/****************** page ******************/
// X Axis Hall Data
#define		CENTER_HALL_AD_X	(0x0000 + E2POFST)		// X Center Hall A/D
#define		MAX_HALL_BEFORE_X	(0x0002 + E2POFST)		// X Max Hall Adjust Value Before
#define		MAX_HALL_AFTER_X	(0x0004 + E2POFST)		// X Max Hall Adjust Value After
#define		MIN_HALL_BEFORE_X	(0x0006 + E2POFST)		// X Min Hall Adjust Value Before
#define		MIN_HALL_AFTER_X	(0x0008 + E2POFST)		// X Min Hall Adjust Value After
#define		HALL_BIAS_X			(0x000A + E2POFST)		// X Hall Bias
#define		HALL_OFFSET_X		(0x000C + E2POFST)		// X Hall Offset
#define		HALL_AD_OFFSET_X	(0x000E + E2POFST)		// X Hall A/D Offset
/****************** page ******************/
// Y Axis Hall Data
#define		CENTER_HALL_AD_Y	(0x0010 + E2POFST)		// Y Center Hall A/D
#define		MAX_HALL_BEFORE_Y	(0x0012 + E2POFST)		// Y Max Hall Adjust Value Before
#define		MAX_HALL_AFTER_Y	(0x0014 + E2POFST)		// Y Max Hall Adjust Value After
#define		MIN_HALL_BEFORE_Y	(0x0016 + E2POFST)		// Y Min Hall Adjust Value Before
#define		MIN_HALL_AFTER_Y	(0x0018 + E2POFST)		// Y Min Hall Adjust Value After
#define		HALL_BIAS_Y			(0x001A + E2POFST)		// Y Hall Bias
#define		HALL_OFFSET_Y		(0x001C + E2POFST)		// Y Hall Offset
#define		HALL_AD_OFFSET_Y	(0x001E + E2POFST)		// Y Hall A/D Offset
/****************** page ******************/
// Loop Gain Status
#define		LOOP_GAIN_STATUS_X	(0x0020 + E2POFST)
#define		LOOP_GAIN_STATUS_Y	(0x0022 + E2POFST)
// Optical Center Adjust Result
#define		OPT_CENTER_X		(0x0024 + E2POFST)		// Optical Center Position X
#define		OPT_CENTER_Y		(0x0026 + E2POFST)		// Optical Center Position Y
// Loop Gain Adjust Result
#define		LOOP_GAIN_X			(0x0028 + E2POFST)
#define		LOOP_GAIN_Y			(0x002A + E2POFST)
// Gyro A/D Offset
#define		GYRO_AD_OFFSET_X	(0x002C + E2POFST)
#define		GYRO_AD_OFFSET_Y	(0x002E + E2POFST)
/****************** page ******************/
// Adjustment Completion Flag
#define		ADJ_COMP_FLAG		(0x0030 + E2POFST)
// Gyro GAIN 
#define		GYRO_GAIN_X			(0x0032 + E2POFST)
#define		GYRO_GAIN_Y			(0x0036 + E2POFST)
// OSC VALUE
#define		OSC_CLK_VAL			(0x003A + E2POFST)		// 2Byte Setting



/****************** page ******************/


/* Optical Center & Gyro Gain for Mode */
 #define	VAL_SET				0x00		// Setting mode
 #define	VAL_FIX				0x01		// Fix Set value
 #define	VAL_SPC				0x02		// Special mode


struct STHALREG {
	unsigned short	UsRegAdd ;
	unsigned char	UcRegDat ;
} ;													// Hall Register Data Table

struct STHALFIL {
	unsigned short	UsRamAdd ;
	unsigned short	UsRamDat ;
} ;													// Hall Filter Coefficient Table

struct STGYRFIL {
	unsigned short	UsRamAdd ;
	unsigned long	UlRamDat ;
} ;													// Gyro Filter Coefficient Table

struct STCMDTBL
{
	unsigned short Cmd ;
	unsigned int UiCmdStf ;
	void ( *UcCmdPtr )( void ) ;
} ;

/*** caution [little-endian] ***/

// Word Data Union
union	WRDVAL{
	unsigned short	UsWrdVal ;
	unsigned char	UcWrkVal[ 2 ] ;
	struct {
		unsigned char	UcLowVal ;
		unsigned char	UcHigVal ;
	} StWrdVal ;
} ;

typedef union WRDVAL	UnWrdVal ;

union	DWDVAL {
	unsigned long	UlDwdVal ;
	unsigned short	UsDwdVal[ 2 ] ;
	struct {
		unsigned short	UsLowVal ;
		unsigned short	UsHigVal ;
	} StDwdVal ;
	struct {
		unsigned char	UcRamVa0 ;
		unsigned char	UcRamVa1 ;
		unsigned char	UcRamVa2 ;
		unsigned char	UcRamVa3 ;
	} StCdwVal ;
} ;

typedef union DWDVAL	UnDwdVal;

// Float Data Union
union	FLTVAL {
	float			SfFltVal ;
	unsigned long	UlLngVal ;
	unsigned short	UsDwdVal[ 2 ] ;
	struct {
		unsigned short	UsLowVal ;
		unsigned short	UsHigVal ;
	} StFltVal ;
} ;

typedef union FLTVAL	UnFltVal ;


typedef struct STADJPAR {
	struct {
		unsigned char	UcAdjPhs ;				// Hall Adjust Phase

		unsigned short	UsHlxCna ;				// Hall Center Value after Hall Adjust
		unsigned short	UsHlxMax ;				// Hall Max Value
		unsigned short	UsHlxMxa ;				// Hall Max Value after Hall Adjust
		unsigned short	UsHlxMin ;				// Hall Min Value
		unsigned short	UsHlxMna ;				// Hall Min Value after Hall Adjust
		unsigned short	UsHlxGan ;				// Hall Gain Value
		unsigned short	UsHlxOff ;				// Hall Offset Value
		unsigned short	UsAdxOff ;				// Hall A/D Offset Value
		unsigned short	UsHlxCen ;				// Hall Center Value

		unsigned short	UsHlyCna ;				// Hall Center Value after Hall Adjust
		unsigned short	UsHlyMax ;				// Hall Max Value
		unsigned short	UsHlyMxa ;				// Hall Max Value after Hall Adjust
		unsigned short	UsHlyMin ;				// Hall Min Value
		unsigned short	UsHlyMna ;				// Hall Min Value after Hall Adjust
		unsigned short	UsHlyGan ;				// Hall Gain Value
		unsigned short	UsHlyOff ;				// Hall Offset Value
		unsigned short	UsAdyOff ;				// Hall A/D Offset Value
		unsigned short	UsHlyCen ;				// Hall Center Value
	} StHalAdj ;

	struct {
		unsigned short	UsLxgVal ;				// Loop Gain X
		unsigned short	UsLygVal ;				// Loop Gain Y
		unsigned short	UsLxgSts ;				// Loop Gain X Status
		unsigned short	UsLygSts ;				// Loop Gain Y Status
	} StLopGan ;

	struct {
		unsigned short	UsGxoVal ;				// Gyro A/D Offset X
		unsigned short	UsGyoVal ;				// Gyro A/D Offset Y
		unsigned short	UsGxoSts ;				// Gyro Offset X Status
		unsigned short	UsGyoSts ;				// Gyro Offset Y Status
	} StGvcOff ;
	
	unsigned char		UcOscVal ;				// OSC value

} stAdjPar ;

OISCMD__	stAdjPar	StAdjPar ;				// Execute Command Parameter

typedef struct	STLBGCON {
	unsigned short	UsLgxVal ;					// Lxbg Value
	unsigned short	UsLgyVal ;					// Lybg Value
	unsigned char	UcLcnFlg ;					// Lbg Control Flag
} stLbgCon ;

OISCMD__	stLbgCon	StLbgCon ;				// For Zoom Value Control
OISCMD__	unsigned char	UcOscAdjFlg ;		// For Measure trigger
  #define	MEASSTR		0x01
  #define	MEASCNT		0x08
  #define	MEASFIX		0x80
#ifdef H1COEF_CHANGER
 OISCMD__	unsigned char	UcH1LvlMod ;		// H1 level coef mode
#endif

OISINI__	unsigned short	UsCntXof ;				/* OPTICAL Center Xvalue */
OISINI__	unsigned short	UsCntYof ;				/* OPTICAL Center Yvalue */


OISINI__	unsigned char	UcPwmMod ;				/* PWM MODE */
#define		PWMMOD_CVL	0x00		// CVL PWM MODE
#define		PWMMOD_PWM	0x01		// PWM MODE

#define		INIT_PWMMODE	PWMMOD_CVL		// initial PWM mode



// Prottype Declation
OISINI__ void	IniSet( void ) ;													// Initial Top Function
//			#define		FS_SEL		0		/* ±262LSB/°/s  */
//			#define		FS_SEL		1		/* ±131LSB/°/s  */
//			#define		FS_SEL		2		/* ±65.5LSB/°/s  */
			#define		FS_SEL		3		/* ±32.8LSB/°/s  */

OISINI__ void	ClrGyr( unsigned char, unsigned char ); 							   // Clear Gyro RAM
	#define CLR_GYR_PRM_RAM 	0x01
	#define CLR_GYR_DLY_RAM 	0x02
	#define CLR_GYR_ALL_RAM 	0x03
OISINI__ void	BsyWit( unsigned short, unsigned char ) ;				// Busy Wait Function
OISINI__ void	WitTim( unsigned short ) ;											// Wait
OISINI__ void	MemClr( unsigned char *, unsigned short ) ;							// Memory Clear Function
OISINI__ void	GyOutSignal( void ) ;									// Slect Gyro Output signal Function
OISINI__ void	Bsy2Wit( void ) ;										// Busy2 wait Function
#ifdef STANDBY_MODE
OISINI__ void	AccWit( unsigned char ) ;								// Acc Wait Function
OISINI__ void	SelectGySleep( unsigned char ) ;						// Select Gyro Mode Function
#endif
#ifdef	GAIN_CONT
OISINI__ void	AutoGainControlSw( unsigned char ) ;							// Auto Gain Control Sw
#endif
OISINI__ void	DrvSw( unsigned char UcDrvSw ) ;						// Driver Mode setting function
OISINI__ void	IniPtMovMod( unsigned char ) ;							// Pan/Tilt parameter setting by mode function

OISCMD__ void			SrvCon( unsigned char, unsigned char ) ;					// Servo ON/OFF
#ifdef		MODULE_CALIBRATION		/* calibration code for Module maker */
 OISCMD__ unsigned short	TneRun( void ) ;											// Hall System Auto Adjustment Function
#endif
OISCMD__ unsigned char	RtnCen( unsigned char ) ;									// Return to Center Function
OISCMD__ void			OisEna( void ) ;											// OIS Enable Function
OISCMD__ void			OisEnaLin( void ) ;											// OISLIN Enable Function
OISCMD__ void			TimPro( void ) ;											// Timer Interrupt Process Function
OISCMD__ void			S2cPro( unsigned char ) ;									// S2 Command Process Function
	#define		DIFIL_S2		0x3F7FFD00
OISCMD__ void			SetSinWavePara( unsigned char , unsigned char ) ;			// Sin wave Test Function
	#define		SINEWAVE	0
	#define		XHALWAVE	1
	#define		YHALWAVE	2
	#define		CIRCWAVE	255
#ifdef		MODULE_CALIBRATION		/* calibration code for Module maker */
 OISCMD__ unsigned char	TneGvc( void ) ;											// Gyro VC Offset Adjust
 #ifdef	NEUTRAL_CENTER											// Gyro VC Offset Adjust
  OISCMD__ unsigned char	TneHvc( void ) ;											// Hall VC Offset Adjust
 #endif	//NEUTRAL_CENTER
#endif

#ifdef USE_EXE2PROM
 OISCMD__ void			E2pRed( unsigned short, unsigned char, unsigned char * ) ;	// E2P ROM Data Read
 OISCMD__ void			E2pWrt( unsigned short, unsigned char, unsigned char * ) ;	// E2P ROM Data Write
#endif
OISCMD__ void			SetZsp( unsigned char ) ;									// Set Zoom Step parameter Function
OISCMD__ void			OptCen( unsigned char, unsigned short, unsigned short ) ;	// Set Optical Center adjusted value Function
#ifdef		MODULE_CALIBRATION		/* calibration code for Module maker */
 OISCMD__ unsigned char	LopGan( unsigned char ) ;									// Loop Gain Adjust
#endif

#ifdef STANDBY_MODE
 OISCMD__ void			SetStandby( unsigned char ) ;								/* Standby control	*/
#endif
#ifdef		MODULE_CALIBRATION		/* calibration code for Module maker */
 OISCMD__ unsigned short	OscAdj( void ) ;											/* OSC clock adjustment */

 #ifdef	HALLADJ_HW
  OISCMD__ unsigned char	LoopGainAdj(   unsigned char );
  OISCMD__ unsigned char	BiasOffsetAdj( unsigned char , unsigned char );
 #endif
#endif

OISCMD__ void			GyrGan( unsigned char , unsigned long , unsigned long ) ;	/* Set Gyro Gain Function */
OISCMD__ void			SetPanTiltMode( unsigned char ) ;							/* Pan_Tilt control Function */
#ifdef		MODULE_CALIBRATION		/* calibration code for Module maker */
 #ifndef	HALLADJ_HW
 OISCMD__ unsigned long	TnePtp( unsigned char, unsigned char ) ;					// Get Hall Peak to Peak Values
 OISCMD__ unsigned char	TneCen( unsigned char, UnDwdVal ) ;							// Tuning Hall Center
 #define		PTP_BEFORE		0
 #define		PTP_AFTER		1
 #endif
#endif
#ifdef GAIN_CONT
 unsigned char	TriSts( void ) ;													// Read Status of Tripod mode Function
#endif
OISCMD__  unsigned char	DrvPwmSw( unsigned char ) ;											// Select Driver mode Function
	#define		Mlnp		0					// Linear PWM
	#define		Mpwm		1					// PWM
OISCMD__ void			SetGcf( unsigned char ) ;									// Set DI filter coefficient Function
#ifdef H1COEF_CHANGER
 OISCMD__ void			SetH1cMod( unsigned char ) ;								// Set H1C coefficient Level chang Function
 #define		S2MODE		0x40
 #define		ACTMODE		0x80
 #define		MOVMODE		0xFF
#endif
unsigned short	RdFwVr( void ) ;													// Read Fw Version Function
