//********************************************************************************
//
//		<< LC898122 Evaluation Soft >>
//		Program Name	: OisIni.c
//		Design			: Y.Yamada
//		History			: LC898122 						2013.01.09 Y.Shigeoka
//********************************************************************************
//**************************
//	Include Header File		
//**************************
#define		OISINI

//#include	"Main.h"
//#include	"Cmd.h"
#include	"Ois_lc898122.h"
#include	"OisFil_lc898122.h"
#include	"OisDef_lc898122.h"

//**************************
//	Local Function Prottype	
//**************************
void	IniClk_lc898122( void ) ;		// Clock Setting
void	IniIop_lc898122( void ) ;		// I/O Port Initial Setting
void	IniMon_lc898122( void ) ;		// Monitor & Other Initial Setting
void	IniSrv_lc898122( void ) ;		// Servo Register Initial Setting
void	IniGyr_lc898122( void ) ;		// Gyro Filter Register Initial Setting
void	IniFil_lc898122( void ) ;		// Gyro Filter Initial Parameter Setting
void	IniAdj_lc898122( void ) ;		// Adjust Fix Value Setting
void	IniCmd_lc898122( void ) ;		// Command Execute Process Initial
void	IniDgy_lc898122( void ) ;		// Digital Gyro Initial Setting
void	IniAf_lc898122( void ) ;			// Open AF Initial Setting
void	IniPtAve_lc898122( void ) ;		// Average setting


//********************************************************************************
// Function Name 	: IniSet
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Initial Setting Function
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniSet_lc898122( void )
{
	// Command Execute Process Initial
	IniCmd_lc898122() ;
	// Clock Setting
	IniClk_lc898122() ;
	// I/O Port Initial Setting
	IniIop_lc898122() ;
	// DigitalGyro Initial Setting
	IniDgy_lc898122() ;
	// Monitor & Other Initial Setting
	IniMon_lc898122() ;
	// Servo Initial Setting
	IniSrv_lc898122() ;
	// Gyro Filter Initial Setting
	IniGyr_lc898122() ;
	// Gyro Filter Initial Setting
	IniFil_lc898122() ;
	// Adjust Fix Value Setting
	IniAdj_lc898122() ;

}

//********************************************************************************
// Function Name 	: IniSetAf
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Initial AF Setting Function
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
void	IniSetAf_lc898122( void )
{
	// Command Execute Process Initial
	IniCmd_lc898122() ;
	// Clock Setting
	IniClk_lc898122() ;
	// AF Initial Setting
	IniAf_lc898122() ;

}



//********************************************************************************
// Function Name 	: IniClk
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Clock Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniClk_lc898122( void )
{
	ChkCvr_lc898122() ;								/* Read Cver */
	
	/*OSC Enables*/
	UcOscAdjFlg_lc898122	= 0 ;					// Osc adj flag 
	
#ifdef	DEF_SET
	/*OSC ENABLE*/
	RegWriteA_lc898122( OSCSTOP,		0x00 ) ;			// 0x0256
	RegWriteA_lc898122( OSCSET,		0x90 ) ;			// 0x0257	OSC ini
	RegWriteA_lc898122( OSCCNTEN,	0x00 ) ;			// 0x0258	OSC Cnt disable
#endif
	/*Clock Enables*/
	RegWriteA_lc898122( CLKON,		0x1F ) ;			// 0x020B

#ifdef	USE_EXTCLK_ALL
	RegWriteA_lc898122( CLKSEL,		0x07 ) ;			// 0x020C	All
#else
 #ifdef	USE_EXTCLK_PWM
	RegWriteA_lc898122( CLKSEL,		0x01 ) ;			// 0x020C	only PWM
 #else
  #ifdef	DEF_SET
	RegWriteA_lc898122( CLKSEL,		0x00 ) ;			// 0x020C	
  #endif
 #endif
#endif
	
#ifdef	USE_EXTCLK_ALL	// 24MHz
	RegWriteA_lc898122( PWMDIV,		0x00 ) ;			// 0x0210	24MHz/1
	RegWriteA_lc898122( SRVDIV,		0x00 ) ;			// 0x0211	24MHz/1
	RegWriteA_lc898122( GIFDIV,		0x02 ) ;			// 0x0212	24MHz/2 = 12MHz
	RegWriteA_lc898122( AFPWMDIV,	0x00 ) ;			// 0x0213	24MHz/1 = 24MHz
	RegWriteA_lc898122( OPAFDIV,		0x02 ) ;			// 0x0214	24MHz/2 = 12MHz
#else
 #ifdef	DEF_SET
	RegWriteA_lc898122( PWMDIV,		0x00 ) ;			// 0x0210	48MHz/1
	RegWriteA_lc898122( SRVDIV,		0x00 ) ;			// 0x0211	48MHz/1
	RegWriteA_lc898122( GIFDIV,		0x03 ) ;			// 0x0212	48MHz/3 = 16MHz
  #ifdef	AF_PWMMODE
	RegWriteA_lc898122( AFPWMDIV,	0x00 ) ;			// 0x0213	48MHz/1
  #else
	RegWriteA_lc898122( AFPWMDIV,	0x02 ) ;			// 0x0213	48MHz/2 = 24MHz
  #endif
	RegWriteA_lc898122( OPAFDIV,		0x04 ) ;			// 0x0214	48MHz/4 = 12MHz
 #endif
#endif
}



//********************************************************************************
// Function Name 	: IniIop
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: I/O Port Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniIop_lc898122( void )
{
#ifdef	DEF_SET
	/*set IOP direction*/
	RegWriteA_lc898122( P0LEV,		0x00 ) ;	// 0x0220	[ - 	| - 	| WLEV5 | WLEV4 ][ WLEV3 | WLEV2 | WLEV1 | WLEV0 ]
	RegWriteA_lc898122( P0DIR,		0x00 ) ;	// 0x0221	[ - 	| - 	| DIR5	| DIR4	][ DIR3  | DIR2  | DIR1  | DIR0  ]
	/*set pull up/down*/
	RegWriteA_lc898122( P0PON,		0x0F ) ;	// 0x0222	[ -    | -	  | PON5 | PON4 ][ PON3  | PON2 | PON1 | PON0 ]
	RegWriteA_lc898122( P0PUD,		0x0F ) ;	// 0x0223	[ -    | -	  | PUD5 | PUD4 ][ PUD3  | PUD2 | PUD1 | PUD0 ]
#endif
	/*select IOP signal*/
#ifdef	USE_3WIRE_DGYRO
	RegWriteA_lc898122( IOP1SEL,		0x02 ); 	// 0x0231	IOP1 : IOP1
#else
	RegWriteA_lc898122( IOP1SEL,		0x00 ); 	// 0x0231	IOP1 : DGDATAIN (ATT:0236h[0]=1)
#endif
#ifdef	DEF_SET
	RegWriteA_lc898122( IOP0SEL,		0x02 ); 	// 0x0230	IOP0 : IOP0
	RegWriteA_lc898122( IOP2SEL,		0x02 ); 	// 0x0232	IOP2 : IOP2
	RegWriteA_lc898122( IOP3SEL,		0x00 ); 	// 0x0233	IOP3 : DGDATAOUT
	RegWriteA_lc898122( IOP4SEL,		0x00 ); 	// 0x0234	IOP4 : DGSCLK
	RegWriteA_lc898122( IOP5SEL,		0x00 ); 	// 0x0235	IOP5 : DGSSB
	RegWriteA_lc898122( DGINSEL,		0x00 ); 	// 0x0236	DGDATAIN 0:IOP1 1:IOP2
	RegWriteA_lc898122( I2CSEL,		0x00 );		// 0x0248	I2C noise reduction ON
	RegWriteA_lc898122( DLMODE,		0x00 );		// 0x0249	Download OFF
#endif
	
}

//********************************************************************************
// Function Name 	: IniDgy
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Digital Gyro Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniDgy_lc898122( void )
{
 #ifdef USE_INVENSENSE
	unsigned char	UcGrini ;
 #endif
	
	/*************/
	/*For ST gyro*/
	/*************/
	
	/*Set SPI Type*/
 #ifdef USE_3WIRE_DGYRO
	RegWriteA_lc898122( SPIM 	, 0x00 );							// 0x028F 	[ - | - | - | - ][ - | - | - | DGSPI4 ]
 #else
	RegWriteA_lc898122( SPIM 	, 0x01 );							// 0x028F 	[ - | - | - | - ][ - | - | - | DGSPI4 ]
 #endif
															//				DGSPI4	0: 3-wire SPI, 1: 4-wire SPI

	/*Set to Command Mode*/
	RegWriteA_lc898122( GRSEL	, 0x01 );							// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

	/*Digital Gyro Read settings*/
	RegWriteA_lc898122( GRINI	, 0x80 );							// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]

 #ifdef USE_INVENSENSE

	RegReadA_lc898122( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
	RegWriteA_lc898122( GRINI, ( UcGrini | SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
	
	RegWriteA_lc898122( GRADR0,	0x6A ) ;					// 0x0283	Set I2C_DIS
	RegWriteA_lc898122( GSETDT,	0x10 ) ;					// 0x028A	Set Write Data
	RegWriteA_lc898122( GRACC,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	AccWit_lc898122( 0x10 ) ;								/* Digital Gyro busy wait 				*/

	RegWriteA_lc898122( GRADR0,	0x1B ) ;					// 0x0283	Set GYRO_CONFIG
	RegWriteA_lc898122( GSETDT,	( FS_SEL << 3) ) ;			// 0x028A	Set Write Data
	RegWriteA_lc898122( GRACC,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	AccWit_lc898122( 0x10 ) ;								/* Digital Gyro busy wait 				*/

	RegReadA_lc898122( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
	RegWriteA_lc898122( GRINI, ( UcGrini & ~SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]

 #endif
	
	RegWriteA_lc898122( RDSEL,	0x7C ) ;				// 0x028B	RDSEL(Data1 and 2 for continuos mode)
	
	GyOutSignal_lc898122() ;
	

}


//********************************************************************************
// Function Name 	: IniMon
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Monitor & Other Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniMon_lc898122( void )
{
	RegWriteA_lc898122( PWMMONA, 0x00 ) ;				// 0x0030	0:off
	
	RegWriteA_lc898122( MONSELA, 0x5C ) ;				// 0x0270	DLYMON1
	RegWriteA_lc898122( MONSELB, 0x5D ) ;				// 0x0271	DLYMON2
	RegWriteA_lc898122( MONSELC, 0x00 ) ;				// 0x0272	
	RegWriteA_lc898122( MONSELD, 0x00 ) ;				// 0x0273	

	// Monitor Circuit
	RegWriteA_lc898122( WC_PINMON1,	0x00 ) ;			// 0x01C0		Filter Monitor
	RegWriteA_lc898122( WC_PINMON2,	0x00 ) ;			// 0x01C1		
	RegWriteA_lc898122( WC_PINMON3,	0x00 ) ;			// 0x01C2		
	RegWriteA_lc898122( WC_PINMON4,	0x00 ) ;			// 0x01C3		
	/* Delay Monitor */
	RegWriteA_lc898122( WC_DLYMON11,	0x04 ) ;			// 0x01C5		DlyMonAdd1[10:8]
	RegWriteA_lc898122( WC_DLYMON10,	0x40 ) ;			// 0x01C4		DlyMonAdd1[ 7:0]
	RegWriteA_lc898122( WC_DLYMON21,	0x04 ) ;			// 0x01C7		DlyMonAdd2[10:8]
	RegWriteA_lc898122( WC_DLYMON20,	0xC0 ) ;			// 0x01C6		DlyMonAdd2[ 7:0]
	RegWriteA_lc898122( WC_DLYMON31,	0x00 ) ;			// 0x01C9		DlyMonAdd3[10:8]
	RegWriteA_lc898122( WC_DLYMON30,	0x00 ) ;			// 0x01C8		DlyMonAdd3[ 7:0]
	RegWriteA_lc898122( WC_DLYMON41,	0x00 ) ;			// 0x01CB		DlyMonAdd4[10:8]
	RegWriteA_lc898122( WC_DLYMON40,	0x00 ) ;			// 0x01CA		DlyMonAdd4[ 7:0]

/* Monitor */
	RegWriteA_lc898122( PWMMONA, 0x80 ) ;				// 0x0030	1:on 
//	RegWriteA_lc898122( IOP0SEL,		0x01 ); 			// 0x0230	IOP0 : MONA
/**/


}

//********************************************************************************
// Function Name 	: IniSrv
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Servo Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniSrv_lc898122( void )
{
	unsigned char	UcStbb0 ;

	UcPwmMod_lc898122 = INIT_PWMMODE ;					// Driver output mode

	RegWriteA_lc898122( WC_EQON,		0x00 ) ;				// 0x0101		Filter Calcu
	RegWriteA_lc898122( WC_RAMINITON,0x00 ) ;				// 0x0102		
	ClrGyr_lc898122( 0x0000 , CLR_ALL_RAM );					// All Clear
	
	RegWriteA_lc898122( WH_EQSWX,	0x02 ) ;				// 0x0170		[ - | - | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
	RegWriteA_lc898122( WH_EQSWY,	0x02 ) ;				// 0x0171		[ - | - | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
	
	RamAccFixMod_lc898122( OFF ) ;							// 32bit Float mode
	
	/* Monitor Gain */
	RamWrite32A_lc898122( dm1g, 0x3F800000 ) ;			// 0x109A
	RamWrite32A_lc898122( dm2g, 0x3F800000 ) ;			// 0x109B
	RamWrite32A_lc898122( dm3g, 0x3F800000 ) ;			// 0x119A
	RamWrite32A_lc898122( dm4g, 0x3F800000 ) ;			// 0x119B
	
	/* Hall output limitter */
	RamWrite32A_lc898122( sxlmta1,   0x3F800000 ) ;			// 0x10E6		Hall X output Limit
	RamWrite32A_lc898122( sylmta1,   0x3F800000 ) ;			// 0x11E6		Hall Y output Limit
	
	/* Emargency Stop */
	RegWriteA_lc898122( WH_EMGSTPON,	0x00 ) ;				// 0x0178		Emargency Stop OFF
	RegWriteA_lc898122( WH_EMGSTPTMR,0xFF ) ;				// 0x017A		255*(16/23.4375kHz)=174ms
	
	RamWrite32A_lc898122( sxemglev,   0x3F800000 ) ;			// 0x10EC		Hall X Emargency threshold
	RamWrite32A_lc898122( syemglev,   0x3F800000 ) ;			// 0x11EC		Hall Y Emargency threshold
	
	/* Hall Servo smoothing */
	RegWriteA_lc898122( WH_SMTSRVON,	0x00 ) ;				// 0x017C		Smooth Servo OFF
#ifdef	USE_EXTCLK_ALL	// 24MHz
	RegWriteA_lc898122( WH_SMTSRVSMP,0x03 ) ;				// 0x017D		2.7ms=2^03/11.718kHz
	RegWriteA_lc898122( WH_SMTTMR,	0x00 ) ;				// 0x017E		1.3ms=(0+1)*16/11.718kHz
#else
	RegWriteA_lc898122( WH_SMTSRVSMP,0x06 ) ;				// 0x017D		2.7ms=2^06/23.4375kHz
	RegWriteA_lc898122( WH_SMTTMR,	0x01 ) ;				// 0x017E		1.3ms=(1+1)*16/23.4375kHz
#endif
	
	RamWrite32A_lc898122( sxsmtav,   0xBC800000 ) ;			// 0x10ED		1/64 X smoothing ave coefficient
	RamWrite32A_lc898122( sysmtav,   0xBC800000 ) ;			// 0x11ED		1/64 Y smoothing ave coefficient
	RamWrite32A_lc898122( sxsmtstp,  0x3AE90466 ) ;			// 0x10EE		0.001778 X smoothing offset
	RamWrite32A_lc898122( sysmtstp,  0x3AE90466 ) ;			// 0x11EE		0.001778 Y smoothing offset
	
	/* High-dimensional correction  */
	RegWriteA_lc898122( WH_HOFCON,	0x11 ) ;				// 0x0174		OUT 3x3
	
	/* Front */
	RamWrite32A_lc898122( sxiexp3,   A3_IEXP3 ) ;			// 0x10BA		
	RamWrite32A_lc898122( sxiexp2,   0x00000000 ) ;			// 0x10BB		
	RamWrite32A_lc898122( sxiexp1,   A1_IEXP1 ) ;			// 0x10BC		
	RamWrite32A_lc898122( sxiexp0,   0x00000000 ) ;			// 0x10BD		
	RamWrite32A_lc898122( sxiexp,    0x3F800000 ) ;			// 0x10BE		

	RamWrite32A_lc898122( syiexp3,   A3_IEXP3 ) ;			// 0x11BA		
	RamWrite32A_lc898122( syiexp2,   0x00000000 ) ;			// 0x11BB		
	RamWrite32A_lc898122( syiexp1,   A1_IEXP1 ) ;			// 0x11BC		
	RamWrite32A_lc898122( syiexp0,   0x00000000 ) ;			// 0x11BD		
	RamWrite32A_lc898122( syiexp,    0x3F800000 ) ;			// 0x11BE		

	/* Back */
	RamWrite32A_lc898122( sxoexp3,   A3_IEXP3 ) ;			// 0x10FA		
	RamWrite32A_lc898122( sxoexp2,   0x00000000 ) ;			// 0x10FB		
	RamWrite32A_lc898122( sxoexp1,   A1_IEXP1 ) ;			// 0x10FC		
	RamWrite32A_lc898122( sxoexp0,   0x00000000 ) ;			// 0x10FD		
	RamWrite32A_lc898122( sxoexp,    0x3F800000 ) ;			// 0x10FE		

	RamWrite32A_lc898122( syoexp3,   A3_IEXP3 ) ;			// 0x11FA		
	RamWrite32A_lc898122( syoexp2,   0x00000000 ) ;			// 0x11FB		
	RamWrite32A_lc898122( syoexp1,   A1_IEXP1 ) ;			// 0x11FC		
	RamWrite32A_lc898122( syoexp0,   0x00000000 ) ;			// 0x11FD		
	RamWrite32A_lc898122( syoexp,    0x3F800000 ) ;			// 0x11FE		
	
	/* Sine wave */
#ifdef	DEF_SET
	RegWriteA_lc898122( WC_SINON,	0x00 ) ;				// 0x0180		Sin Wave off
	RegWriteA_lc898122( WC_SINFRQ0,	0x00 ) ;				// 0x0181		
	RegWriteA_lc898122( WC_SINFRQ1,	0x60 ) ;				// 0x0182		
	RegWriteA_lc898122( WC_SINPHSX,	0x00 ) ;				// 0x0183		
	RegWriteA_lc898122( WC_SINPHSY,	0x20 ) ;				// 0x0184		
	
	/* AD over sampling */
	RegWriteA_lc898122( WC_ADMODE,	0x06 ) ;				// 0x0188		AD Over Sampling
	
	/* Measure mode */
	RegWriteA_lc898122( WC_MESMODE,		0x00 ) ;				// 0x0190		Measurement Mode
	RegWriteA_lc898122( WC_MESSINMODE,	0x00 ) ;				// 0x0191		
	RegWriteA_lc898122( WC_MESLOOP0,		0x08 ) ;				// 0x0192		
	RegWriteA_lc898122( WC_MESLOOP1,		0x02 ) ;				// 0x0193		
	RegWriteA_lc898122( WC_MES1ADD0,		0x00 ) ;				// 0x0194		
	RegWriteA_lc898122( WC_MES1ADD1,		0x00 ) ;				// 0x0195		
	RegWriteA_lc898122( WC_MES2ADD0,		0x00 ) ;				// 0x0196		
	RegWriteA_lc898122( WC_MES2ADD1,		0x00 ) ;				// 0x0197		
	RegWriteA_lc898122( WC_MESABS,		0x00 ) ;				// 0x0198		
	RegWriteA_lc898122( WC_MESWAIT,		0x00 ) ;				// 0x0199		
	
	/* auto measure */
	RegWriteA_lc898122( WC_AMJMODE,		0x00 ) ;				// 0x01A0		Automatic measurement mode
	
	RegWriteA_lc898122( WC_AMJLOOP0,		0x08 ) ;				// 0x01A2		Self-Aadjustment
	RegWriteA_lc898122( WC_AMJLOOP1,		0x02 ) ;				// 0x01A3		
	RegWriteA_lc898122( WC_AMJIDL0,		0x02 ) ;				// 0x01A4		
	RegWriteA_lc898122( WC_AMJIDL1,		0x00 ) ;				// 0x01A5		
	RegWriteA_lc898122( WC_AMJ1ADD0,		0x00 ) ;				// 0x01A6		
	RegWriteA_lc898122( WC_AMJ1ADD1,		0x00 ) ;				// 0x01A7		
	RegWriteA_lc898122( WC_AMJ2ADD0,		0x00 ) ;				// 0x01A8		
	RegWriteA_lc898122( WC_AMJ2ADD1,		0x00 ) ;				// 0x01A9		
	
	/* Data Pass */
	RegWriteA_lc898122( WC_DPI1ADD0,		0x00 ) ;				// 0x01B0		Data Pass
	RegWriteA_lc898122( WC_DPI1ADD1,		0x00 ) ;				// 0x01B1		
	RegWriteA_lc898122( WC_DPI2ADD0,		0x00 ) ;				// 0x01B2		
	RegWriteA_lc898122( WC_DPI2ADD1,		0x00 ) ;				// 0x01B3		
	RegWriteA_lc898122( WC_DPI3ADD0,		0x00 ) ;				// 0x01B4		
	RegWriteA_lc898122( WC_DPI3ADD1,		0x00 ) ;				// 0x01B5		
	RegWriteA_lc898122( WC_DPI4ADD0,		0x00 ) ;				// 0x01B6		
	RegWriteA_lc898122( WC_DPI4ADD1,		0x00 ) ;				// 0x01B7		
	RegWriteA_lc898122( WC_DPO1ADD0,		0x00 ) ;				// 0x01B8		Data Pass
	RegWriteA_lc898122( WC_DPO1ADD1,		0x00 ) ;				// 0x01B9		
	RegWriteA_lc898122( WC_DPO2ADD0,		0x00 ) ;				// 0x01BA		
	RegWriteA_lc898122( WC_DPO2ADD1,		0x00 ) ;				// 0x01BB		
	RegWriteA_lc898122( WC_DPO3ADD0,		0x00 ) ;				// 0x01BC		
	RegWriteA_lc898122( WC_DPO3ADD1,		0x00 ) ;				// 0x01BD		
	RegWriteA_lc898122( WC_DPO4ADD0,		0x00 ) ;				// 0x01BE		
	RegWriteA_lc898122( WC_DPO4ADD1,		0x00 ) ;				// 0x01BF		
	RegWriteA_lc898122( WC_DPON,			0x00 ) ;				// 0x0105		Data pass OFF
	
	/* Interrupt Flag */
	RegWriteA_lc898122( WC_INTMSK,	0xFF ) ;				// 0x01CE		All Mask
	
#endif
	
	/* Ram Access */
	RamAccFixMod_lc898122( OFF ) ;							// 32bit float mode

	// PWM Signal Generate
	DrvSw_lc898122( OFF ) ;									/* 0x0070	Drvier Block Ena=0 */
	RegWriteA_lc898122( DRVFC2	, 0x90 );					// 0x0002	Slope 3, Dead Time = 30 ns
	RegWriteA_lc898122( DRVSELX	, 0xFF );					// 0x0003	PWM X drv max current  DRVSELX[7:0]
	RegWriteA_lc898122( DRVSELY	, 0xFF );					// 0x0004	PWM Y drv max current  DRVSELY[7:0]

#ifdef	PWM_BREAK
 #ifdef	PWM_CAREER_TEST
	RegWriteA_lc898122( PWMFC,		0x7C ) ;				// 0x0011	VREF, PWMFRQ=7:PWMCLK(EXCLK)/PWMPERIODX[5:2]=18MHz/4=4.5MHz, MODE0B, 11-bit Accuracy
 #else		//PWM_CAREER_TEST
	if( UcCvrCod == CVER122 ) {
		RegWriteA_lc898122( PWMFC,   0x2D ) ;					// 0x0011	VREF, PWMCLK/256, MODE0B, 12Bit Accuracy
	} else {
		RegWriteA_lc898122( PWMFC,   0x3D ) ;					// 0x0011	VREF, PWMCLK/128, MODE0B, 12Bit Accuracy
	}
 #endif	//PWM_CAREER_TEST
#else
	RegWriteA_lc898122( PWMFC,   0x21 ) ;					// 0x0011	VREF, PWMCLK/256, MODE1, 12Bit Accuracy
#endif

#ifdef	USE_VH_SYNC
	RegWriteA_lc898122( STROBEFC,	0x80 ) ;				// 0x001C	外部入力Strobe信号の有効
	RegWriteA_lc898122( STROBEDLYX,	0x00 ) ;				// 0x001D	Delay
	RegWriteA_lc898122( STROBEDLYY,	0x00 ) ;				// 0x001E	Delay
#endif	//USE_VH_SYNC

	RegWriteA_lc898122( PWMA,    0x00 ) ;					// 0x0010	PWM X/Y standby
	RegWriteA_lc898122( PWMDLYX,  0x04 ) ;					// 0x0012	X Phase Delay Setting
	RegWriteA_lc898122( PWMDLYY,  0x04 ) ;					// 0x0013	Y Phase Delay Setting
	
#ifdef	DEF_SET
	RegWriteA_lc898122( DRVCH1SEL,	0x00 ) ;				// 0x0005	OUT1/OUT2	X axis
	RegWriteA_lc898122( DRVCH2SEL,	0x00 ) ;				// 0x0006	OUT3/OUT4	Y axis
	
	RegWriteA_lc898122( PWMDLYTIMX,	0x00 ) ;				// 0x0014		PWM Timing
	RegWriteA_lc898122( PWMDLYTIMY,	0x00 ) ;				// 0x0015		PWM Timing
#endif
	
	if( UcCvrCod == CVER122 ) {
#ifdef	PWM_CAREER_TEST
		RegWriteA_lc898122( PWMPERIODY,	0xD0 ) ;				// 0x001A	11010000h --> PWMPERIODX[5:2] = 0100h = 4
		RegWriteA_lc898122( PWMPERIODY2,	0xD0 ) ;				// 0x001B	11010000h --> PWMPERIODY[5:2] = 0100h = 4
#else		//PWM_CAREER_TEST
		RegWriteA_lc898122( PWMPERIODY,	0x00 ) ;				// 0x001A		PWM Carrier Freq
		RegWriteA_lc898122( PWMPERIODY2,	0x00 ) ;				// 0x001B		PWM Carrier Freq
#endif
	} else {
#ifdef	PWM_CAREER_TEST
		RegWriteA_lc898122( PWMPERIODX,	0xF2 ) ;				// 0x0018		PWM Carrier Freq
		RegWriteA_lc898122( PWMPERIODX2,	0x00 ) ;				// 0x0019		PWM Carrier Freq
		RegWriteA_lc898122( PWMPERIODY,	0xF2 ) ;				// 0x001A		PWM Carrier Freq
		RegWriteA_lc898122( PWMPERIODY2,	0x00 ) ;				// 0x001B		PWM Carrier Freq
#else		//PWM_CAREER_TEST
		RegWriteA_lc898122( PWMPERIODX,	0x00 ) ;				// 0x0018		PWM Carrier Freq
		RegWriteA_lc898122( PWMPERIODX2,	0x00 ) ;				// 0x0019		PWM Carrier Freq
		RegWriteA_lc898122( PWMPERIODY,	0x00 ) ;				// 0x001A		PWM Carrier Freq
		RegWriteA_lc898122( PWMPERIODY2,	0x00 ) ;				// 0x001B		PWM Carrier Freq
#endif
	}
	
	/* Linear PWM circuit setting */
	RegWriteA_lc898122( CVA		, 0xC0 );			// 0x0020	Linear PWM mode enable

	if( UcCvrCod == CVER122 ) {
		RegWriteA_lc898122( CVFC 	, 0x22 );			// 0x0021	
	}

	RegWriteA_lc898122( CVFC2 	, 0x80 );			// 0x0022
	if( UcCvrCod == CVER122 ) {
		RegWriteA_lc898122( CVSMTHX	, 0x00 );			// 0x0023	smooth off
		RegWriteA_lc898122( CVSMTHY	, 0x00 );			// 0x0024	smooth off
	}

	RegReadA_lc898122( STBB0 	, &UcStbb0 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	UcStbb0 &= 0x80 ;
	RegWriteA_lc898122( STBB0, UcStbb0 ) ;			// 0x0250	OIS standby
	
}



//********************************************************************************
// Function Name 	: IniGyr
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Gyro Filter Setting Initialize Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
#ifdef GAIN_CONT
  #define	TRI_LEVEL		0x3A031280		/* 0.0005 */
  #define	TIMELOW			0x50			/* */
  #define	TIMEHGH			0x05			/* */
 #ifdef	USE_EXTCLK_ALL	// 24MHz
  #define	TIMEBSE			0x2F			/* 4.0ms */
 #else
  #define	TIMEBSE			0x5D			/* 3.96ms */
 #endif
  #define	MONADR			GXXFZ
  #define	GANADR			gxadj
  #define	XMINGAIN		0x00000000
  #define	XMAXGAIN		0x3F800000
  #define	YMINGAIN		0x00000000
  #define	YMAXGAIN		0x3F800000
  #define	XSTEPUP			0x38D1B717		/* 0.0001	 */
  #define	XSTEPDN			0xBD4CCCCD		/* -0.05 	 */
  #define	YSTEPUP			0x38D1B717		/* 0.0001	 */
  #define	YSTEPDN			0xBD4CCCCD		/* -0.05 	 */
#endif


void	IniGyr_lc898122( void )
{
	
	
	/*Gyro Filter Setting*/
	RegWriteA_lc898122( WG_EQSW	, 0x03 );		// 0x0110		[ - | Sw6 | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
	
	/*Gyro Filter Down Sampling*/
	
	RegWriteA_lc898122( WG_SHTON	, 0x10 );		// 0x0107		[ - | - | - | CmSht2PanOff ][ - | - | CmShtOpe(1:0) ]
										//				CmShtOpe[1:0] 00: シャッターOFF, 01: シャッターON, 1x:外部制御
										
#ifdef	DEF_SET
	RegWriteA_lc898122( WG_SHTDLYTMR , 0x00 );	// 0x0117	 	Shutter Delay
	RegWriteA_lc898122( WG_GADSMP, 	  0x00 );	// 0x011C		Sampling timing
	RegWriteA_lc898122( WG_HCHR, 	  0x00 );	// 0x011B		H-filter limitter control not USE
	RegWriteA_lc898122( WG_LMT3MOD , 0x00 );		// 0x0118 	[ - | - | - | - ][ - | - | - | CmLmt3Mod ]
										//				CmLmt3Mod	0: 通常リミッター動作, 1: 円の半径リミッター動作
	RegWriteA_lc898122( WG_VREFADD , 0x12 );		// 0x0119	 	センター戻しを行う遅延RAMのアドレス下位6ビット　(default 0x12 = GXH1Z2/GYH1Z2)
#endif
	RegWriteA_lc898122( WG_SHTMOD , 0x06 );		// 0x0116	 	Shutter Hold mode

	// Limiter
	RamWrite32A_lc898122( gxlmt1H, GYRLMT1H ) ;			// 0x1028
	RamWrite32A_lc898122( gylmt1H, GYRLMT1H ) ;			// 0x1128

	RamWrite32A_lc898122( gxlmt3HS0, GYRLMT3_S1 ) ;		// 0x1029
	RamWrite32A_lc898122( gylmt3HS0, GYRLMT3_S1 ) ;		// 0x1129
	
	RamWrite32A_lc898122( gxlmt3HS1, GYRLMT3_S2 ) ;		// 0x102A
	RamWrite32A_lc898122( gylmt3HS1, GYRLMT3_S2 ) ;		// 0x112A

	RamWrite32A_lc898122( gylmt4HS0, GYRLMT4_S1 ) ;		//0x112B	Y軸Limiter4 High閾値0
	RamWrite32A_lc898122( gxlmt4HS0, GYRLMT4_S1 ) ;		//0x102B	X軸Limiter4 High閾値0
	
	RamWrite32A_lc898122( gxlmt4HS1, GYRLMT4_S2 ) ;		//0x102C	X軸Limiter4 High閾値1
	RamWrite32A_lc898122( gylmt4HS1, GYRLMT4_S2 ) ;		//0x112C	Y軸Limiter4 High閾値1

	
	/* Pan/Tilt parameter */
	RegWriteA_lc898122( WG_PANADDA, 		0x12 );		// 0x0130	GXH1Z2/GYH1Z2 Select
	RegWriteA_lc898122( WG_PANADDB, 		0x09 );		// 0x0131	GXIZ/GYIZ Select
	
	 //Threshold
	RamWrite32A_lc898122( SttxHis, 	0x00000000 );			// 0x1226
	RamWrite32A_lc898122( SttxaL, 	0x00000000 );			// 0x109D
	RamWrite32A_lc898122( SttxbL, 	0x00000000 );			// 0x109E
	RamWrite32A_lc898122( Sttx12aM, 	GYRA12_MID );	// 0x104F
	RamWrite32A_lc898122( Sttx12aH, 	GYRA12_HGH );	// 0x105F
	RamWrite32A_lc898122( Sttx12bM, 	GYRB12_MID );	// 0x106F
	RamWrite32A_lc898122( Sttx12bH, 	GYRB12_HGH );	// 0x107F
	RamWrite32A_lc898122( Sttx34aM, 	GYRA34_MID );	// 0x108F
	RamWrite32A_lc898122( Sttx34aH, 	GYRA34_HGH );	// 0x109F
	RamWrite32A_lc898122( Sttx34bM, 	GYRB34_MID );	// 0x10AF
	RamWrite32A_lc898122( Sttx34bH, 	GYRB34_HGH );	// 0x10BF
	RamWrite32A_lc898122( SttyaL, 	0x00000000 );			// 0x119D
	RamWrite32A_lc898122( SttybL, 	0x00000000 );			// 0x119E
	RamWrite32A_lc898122( Stty12aM, 	GYRA12_MID );	// 0x114F
	RamWrite32A_lc898122( Stty12aH, 	GYRA12_HGH );	// 0x115F
	RamWrite32A_lc898122( Stty12bM, 	GYRB12_MID );	// 0x116F
	RamWrite32A_lc898122( Stty12bH, 	GYRB12_HGH );	// 0x117F
	RamWrite32A_lc898122( Stty34aM, 	GYRA34_MID );	// 0x118F
	RamWrite32A_lc898122( Stty34aH, 	GYRA34_HGH );	// 0x119F
	RamWrite32A_lc898122( Stty34bM, 	GYRB34_MID );	// 0x11AF
	RamWrite32A_lc898122( Stty34bH, 	GYRB34_HGH );	// 0x11BF
	
	// Pan level
	RegWriteA_lc898122( WG_PANLEVABS, 		0x00 );		// 0x0133
	
	// Average parameter are set IniAdj

	// Phase Transition Setting
	// State 2 -> 1
	RegWriteA_lc898122( WG_PANSTT21JUG0, 	0x00 );		// 0x0140
	RegWriteA_lc898122( WG_PANSTT21JUG1, 	0x00 );		// 0x0141
	// State 3 -> 1
	RegWriteA_lc898122( WG_PANSTT31JUG0, 	0x00 );		// 0x0142
	RegWriteA_lc898122( WG_PANSTT31JUG1, 	0x00 );		// 0x0143
	// State 4 -> 1
	RegWriteA_lc898122( WG_PANSTT41JUG0, 	0x01 );		// 0x0144
	RegWriteA_lc898122( WG_PANSTT41JUG1, 	0x00 );		// 0x0145
	// State 1 -> 2
	RegWriteA_lc898122( WG_PANSTT12JUG0, 	0x00 );		// 0x0146
	RegWriteA_lc898122( WG_PANSTT12JUG1, 	0x07 );		// 0x0147
	// State 1 -> 3
	RegWriteA_lc898122( WG_PANSTT13JUG0, 	0x00 );		// 0x0148
	RegWriteA_lc898122( WG_PANSTT13JUG1, 	0x00 );		// 0x0149
	// State 2 -> 3
	RegWriteA_lc898122( WG_PANSTT23JUG0, 	0x11 );		// 0x014A
	RegWriteA_lc898122( WG_PANSTT23JUG1, 	0x00 );		// 0x014B
	// State 4 -> 3
	RegWriteA_lc898122( WG_PANSTT43JUG0, 	0x00 );		// 0x014C
	RegWriteA_lc898122( WG_PANSTT43JUG1, 	0x00 );		// 0x014D
	// State 3 -> 4
	RegWriteA_lc898122( WG_PANSTT34JUG0, 	0x01 );		// 0x014E
	RegWriteA_lc898122( WG_PANSTT34JUG1, 	0x00 );		// 0x014F
	// State 2 -> 4
	RegWriteA_lc898122( WG_PANSTT24JUG0, 	0x00 );		// 0x0150
	RegWriteA_lc898122( WG_PANSTT24JUG1, 	0x00 );		// 0x0151
	// State 4 -> 2
	RegWriteA_lc898122( WG_PANSTT42JUG0, 	0x44 );		// 0x0152
	RegWriteA_lc898122( WG_PANSTT42JUG1, 	0x04 );		// 0x0153

	// State Timer
	RegWriteA_lc898122( WG_PANSTT1LEVTMR, 	0x00 );		// 0x015B
	RegWriteA_lc898122( WG_PANSTT2LEVTMR, 	0x00 );		// 0x015C
	RegWriteA_lc898122( WG_PANSTT3LEVTMR, 	0x00 );		// 0x015D
	RegWriteA_lc898122( WG_PANSTT4LEVTMR, 	0x03 );		// 0x015E
	
	// Control filter
	RegWriteA_lc898122( WG_PANTRSON0, 		0x11 );		// 0x0132	USE I12/iSTP/Gain-Filter
	
	// State Setting
	IniPtMovMod_lc898122( OFF ) ;							// Pan/Tilt setting (Still)
	
	// Hold
	RegWriteA_lc898122( WG_PANSTTSETILHLD,	0x00 );		// 0x015F
	
	
	// State2,4 Step Time Setting
	RegWriteA_lc898122( WG_PANSTT2TMR0,	0x01 );		// 0x013C
	RegWriteA_lc898122( WG_PANSTT2TMR1,	0x00 );		// 0x013D	
	RegWriteA_lc898122( WG_PANSTT4TMR0,	0x02 );		// 0x013E
	RegWriteA_lc898122( WG_PANSTT4TMR1,	0x07 );		// 0x013F	
	
	RegWriteA_lc898122( WG_PANSTTXXXTH,	0x00 );		// 0x015A

#ifdef GAIN_CONT
	RamWrite32A_lc898122( gxlevlow, TRI_LEVEL );					// 0x10AE	Low Th
	RamWrite32A_lc898122( gylevlow, TRI_LEVEL );					// 0x11AE	Low Th
	RamWrite32A_lc898122( gxadjmin, XMINGAIN );					// 0x1094	Low gain
	RamWrite32A_lc898122( gxadjmax, XMAXGAIN );					// 0x1095	Hgh gain
	RamWrite32A_lc898122( gxadjdn, XSTEPDN );					// 0x1096	-step
	RamWrite32A_lc898122( gxadjup, XSTEPUP );					// 0x1097	+step
	RamWrite32A_lc898122( gyadjmin, YMINGAIN );					// 0x1194	Low gain
	RamWrite32A_lc898122( gyadjmax, YMAXGAIN );					// 0x1195	Hgh gain
	RamWrite32A_lc898122( gyadjdn, YSTEPDN );					// 0x1196	-step
	RamWrite32A_lc898122( gyadjup, YSTEPUP );					// 0x1197	+step
	
	RegWriteA_lc898122( WG_LEVADD, (unsigned char)MONADR );		// 0x0120	Input signal
	RegWriteA_lc898122( WG_LEVTMR, 		TIMEBSE );				// 0x0123	Base Time
	RegWriteA_lc898122( WG_LEVTMRLOW, 	TIMELOW );				// 0x0121	X Low Time
	RegWriteA_lc898122( WG_LEVTMRHGH, 	TIMEHGH );				// 0x0122	X Hgh Time
	RegWriteA_lc898122( WG_ADJGANADD, (unsigned char)GANADR );		// 0x0128	control address
	RegWriteA_lc898122( WG_ADJGANGO, 		0x00 );					// 0x0108	manual off

	/* exe function */
//	AutoGainControlSw_lc898122( OFF ) ;							/* Auto Gain Control Mode OFF */
	AutoGainControlSw_lc898122( ON ) ;							/* Auto Gain Control Mode ON  */
#endif
	
}


//********************************************************************************
// Function Name 	: IniFil
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Gyro Filter Initial Parameter Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniFil_lc898122( void )
{
 	unsigned short	UsAryId ;

	// Filter Registor Parameter Setting
	UsAryId	= 0 ;
	while( CsFilReg[ UsAryId ].UsRegAdd != 0xFFFF )
	{
		RegWriteA_lc898122( CsFilReg[ UsAryId ].UsRegAdd, CsFilReg[ UsAryId ].UcRegDat ) ;
		UsAryId++ ;
	}

	// Filter Ram Parameter Setting
	UsAryId	= 0 ;
	while( CsFilRam[ UsAryId ].UsRamAdd != 0xFFFF )
	{
		RamWrite32A_lc898122( CsFilRam[ UsAryId ].UsRamAdd, CsFilRam[ UsAryId ].UlRamDat ) ;
		UsAryId++ ;
	}
	
}



//********************************************************************************
// Function Name 	: IniAdj
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Adjust Value Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniAdj_lc898122( void )
{
	RegWriteA_lc898122( WC_RAMACCXY, 0x00 ) ;			// 0x018D	Filter copy off

	IniPtAve_lc898122( ) ;								// Average setting
	
	/* OIS */
	RegWriteA_lc898122( CMSDAC0, BIAS_CUR_OIS ) ;		// 0x0251	Hall Dac電流
	RegWriteA_lc898122( OPGSEL0, AMP_GAIN_X ) ;			// 0x0253	Hall amp Gain X
	RegWriteA_lc898122( OPGSEL1, AMP_GAIN_Y ) ;			// 0x0254	Hall amp Gain Y
	/* AF */
	RegWriteA_lc898122( CMSDAC1, BIAS_CUR_AF ) ;			// 0x0252	Hall Dac電流
	RegWriteA_lc898122( OPGSEL2, AMP_GAIN_AF ) ;			// 0x0255	Hall amp Gain AF

	RegWriteA_lc898122( OSCSET, OSC_INI ) ;				// 0x0257	OSC ini
	
	/* adjusted value */
	RegWriteA_lc898122( IZAH,	DGYRO_OFST_XH ) ;	// 0x02A0		Set Offset High byte
	RegWriteA_lc898122( IZAL,	DGYRO_OFST_XL ) ;	// 0x02A1		Set Offset Low byte
	RegWriteA_lc898122( IZBH,	DGYRO_OFST_YH ) ;	// 0x02A2		Set Offset High byte
	RegWriteA_lc898122( IZBL,	DGYRO_OFST_YL ) ;	// 0x02A3		Set Offset Low byte
	
	/* Ram Access */
	RamAccFixMod_lc898122( ON ) ;							// 16bit Fix mode
	
	/* OIS adjusted parameter */
	RamWriteA_lc898122( DAXHLO,		DAHLXO_INI ) ;		// 0x1479
	RamWriteA_lc898122( DAXHLB,		DAHLXB_INI ) ;		// 0x147A
	RamWriteA_lc898122( DAYHLO,		DAHLYO_INI ) ;		// 0x14F9
	RamWriteA_lc898122( DAYHLB,		DAHLYB_INI ) ;		// 0x14FA
	RamWriteA_lc898122( OFF0Z,		HXOFF0Z_INI ) ;		// 0x1450
	RamWriteA_lc898122( OFF1Z,		HYOFF1Z_INI ) ;		// 0x14D0
	RamWriteA_lc898122( sxg,			SXGAIN_INI ) ;		// 0x10D3
	RamWriteA_lc898122( syg,			SYGAIN_INI ) ;		// 0x11D3
//	UsCntXof_lc898122 = OPTCEN_X ;						/* Clear Optical center X value */
//	UsCntYof_lc898122 = OPTCEN_Y ;						/* Clear Optical center Y value */
//	RamWriteA_lc898122( SXOFFZ1,		UsCntXof_lc898122 ) ;		// 0x1461
//	RamWriteA_lc898122( SYOFFZ1,		UsCntYof_lc898122 ) ;		// 0x14E1

	/* AF adjusted parameter */
	RamWriteA_lc898122( DAZHLO,		DAHLZO_INI ) ;		// 0x1529
	RamWriteA_lc898122( DAZHLB,		DAHLZB_INI ) ;		// 0x152A

	/* Ram Access */
	RamAccFixMod_lc898122( OFF ) ;							// 32bit Float mode
	
	RamWrite32A_lc898122( gxzoom, GXGAIN_INI ) ;		// 0x1020 Gyro X axis Gain adjusted value
	RamWrite32A_lc898122( gyzoom, GYGAIN_INI ) ;		// 0x1120 Gyro Y axis Gain adjusted value

	RamWrite32A_lc898122( sxq, SXQ_INI ) ;			// 0x10E5	X axis output direction initial value
	RamWrite32A_lc898122( syq, SYQ_INI ) ;			// 0x11E5	Y axis output direction initial value
	
	if( GXHY_GYHX ){			/* GX -> HY , GY -> HX */
		RamWrite32A_lc898122( sxgx, 0x00000000 ) ;			// 0x10B8
		RamWrite32A_lc898122( sxgy, 0x3F800000 ) ;			// 0x10B9
		
		RamWrite32A_lc898122( sygy, 0x00000000 ) ;			// 0x11B8
		RamWrite32A_lc898122( sygx, 0x3F800000 ) ;			// 0x11B9
	}
	
	SetZsp_lc898122(0) ;								// Zoom coefficient Initial Setting
	
	RegWriteA_lc898122( PWMA 	, 0xC0 );			// 0x0010		PWM enable

	RegWriteA_lc898122( STBB0 	, 0xDF );			// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	RegWriteA_lc898122( WC_EQSW	, 0x02 ) ;			// 0x01E0
	RegWriteA_lc898122( WC_MESLOOP1	, 0x02 ) ;		// 0x0193
	RegWriteA_lc898122( WC_MESLOOP0	, 0x00 ) ;		// 0x0192
	RegWriteA_lc898122( WC_AMJLOOP1	, 0x02 ) ;		// 0x01A3
	RegWriteA_lc898122( WC_AMJLOOP0	, 0x00 ) ;		// 0x01A2
	
	
	SetPanTiltMode_lc898122( OFF ) ;					/* Pan/Tilt OFF */
	
	SetGcf_lc898122( 0 ) ;							/* DI initial value */
#ifdef H1COEF_CHANGER
	SetH1cMod_lc898122( ACTMODE ) ;					/* Lvl Change Active mode */
#endif
	
	DrvSw_lc898122( ON ) ;							/* 0x0001		Driver Mode setting */
	
	RegWriteA_lc898122( WC_EQON, 0x01 ) ;			// 0x0101	Filter ON
}



//********************************************************************************
// Function Name 	: IniCmd
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Command Execute Process Initial
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniCmd_lc898122( void )
{

	MemClr_lc898122( ( unsigned char * )&StAdjPar_lc898122, sizeof( stAdjPar ) ) ;	// Adjust Parameter Clear
	
}


//********************************************************************************
// Function Name 	: BsyWit
// Retun Value		: NON
// Argment Value	: Trigger Register Address, Trigger Register Data
// Explanation		: Busy Wait Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
void	BsyWit_lc898122( unsigned short	UsTrgAdr, unsigned char	UcTrgDat )
{
	unsigned char	UcFlgVal ;

	RegWriteA_lc898122( UsTrgAdr, UcTrgDat ) ;	// Trigger Register Setting

	UcFlgVal	= 1 ;

	while( UcFlgVal ) {

		RegReadA_lc898122( UsTrgAdr, &UcFlgVal ) ;
		UcFlgVal	&= 	( UcTrgDat & 0x0F ) ;
	} ;

}


//********************************************************************************
// Function Name 	: MemClr
// Retun Value		: void
// Argment Value	: Clear Target Pointer, Clear Byte Number
// Explanation		: Memory Clear Function
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	MemClr_lc898122( unsigned char	*NcTgtPtr, unsigned short	UsClrSiz )
{
	unsigned short	UsClrIdx ;

	for ( UsClrIdx = 0 ; UsClrIdx < UsClrSiz ; UsClrIdx++ )
	{
		*NcTgtPtr	= 0 ;
		NcTgtPtr++ ;
	}
}



//********************************************************************************
// Function Name 	: WitTim
// Retun Value		: NON
// Argment Value	: Wait Time(ms)
// Explanation		: Timer Wait Function
// History			: First edition 						2009.07.31 Y.Tashita
//********************************************************************************
void	WitTim_lc898122( unsigned short	UsWitTim )
{
	unsigned long	UlLopIdx, UlWitCyc ;

	UlWitCyc	= ( unsigned long )( ( float )UsWitTim / NOP_TIME / ( float )12 ) ;

	for( UlLopIdx = 0 ; UlLopIdx < UlWitCyc ; UlLopIdx++ )
	{
		;
	}
}

//********************************************************************************
// Function Name 	: GyOutSignal
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Select Gyro Signal Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	GyOutSignal_lc898122( void )
{

	RegWriteA_lc898122( GRADR0,	GYROX_INI ) ;			// 0x0283	Set Gyro XOUT H~L
	RegWriteA_lc898122( GRADR1,	GYROY_INI ) ;			// 0x0284	Set Gyro YOUT H~L
	
	/*Start OIS Reading*/
	RegWriteA_lc898122( GRSEL	, 0x02 );			// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

}

//********************************************************************************
// Function Name 	: GyOutSignalCont
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Select Gyro Continuosl Function
// History			: First edition 						2013.06.06 Y.Shigeoka
//********************************************************************************
void	GyOutSignalCont_lc898122( void )
{

	/*Start OIS Reading*/
	RegWriteA_lc898122( GRSEL	, 0x04 );			// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

}

#ifdef STANDBY_MODE
//********************************************************************************
// Function Name 	: AccWit
// Retun Value		: NON
// Argment Value	: Trigger Register Data
// Explanation		: Acc Wait Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	AccWit_lc898122( unsigned char UcTrgDat )
{
	unsigned char	UcFlgVal ;

	UcFlgVal	= 1 ;

	while( UcFlgVal ) {
		RegReadA_lc898122( GRACC, &UcFlgVal ) ;			// 0x0282
		UcFlgVal	&= UcTrgDat ;
	} ;

}

//********************************************************************************
// Function Name 	: SelectGySleep
// Retun Value		: NON
// Argment Value	: mode	
// Explanation		: Select Gyro mode Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	SelectGySleep_lc898122( unsigned char UcSelMode )
{
 #ifdef USE_INVENSENSE
	unsigned char	UcRamIni ;
	unsigned char	UcGrini ;

	if(UcSelMode == ON)
	{
		RegWriteA_lc898122( WC_EQON, 0x00 ) ;		// 0x0101	Equalizer OFF
		RegWriteA_lc898122( GRSEL,	0x01 ) ;		/* 0x0280	Set Command Mode			*/

		RegReadA_lc898122( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
		RegWriteA_lc898122( GRINI, ( UcGrini | SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
		
		RegWriteA_lc898122( GRADR0,	0x6B ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_lc898122( GRACC,	0x01 ) ;		/* 0x0282	Set Read Trigger ON				*/
		AccWit_lc898122( 0x01 ) ;					/* Digital Gyro busy wait 				*/
		RegReadA_lc898122( GRDAT0H, &UcRamIni ) ;	/* 0x0290 */
		
		UcRamIni |= 0x40 ;					/* Set Sleep bit */
  #ifdef GYROSTBY
		UcRamIni &= ~0x01 ;					/* Clear PLL bit(internal oscillator */
  #endif
		
		RegWriteA_lc898122( GRADR0,	0x6B ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_lc898122( GSETDT,	UcRamIni ) ;	/* 0x028A	Set Write Data(Sleep ON)	*/
		RegWriteA_lc898122( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_lc898122( 0x10 ) ;					/* Digital Gyro busy wait 				*/

  #ifdef GYROSTBY
		RegWriteA_lc898122( GRADR0,	0x6C ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_lc898122( GSETDT,	0x07 ) ;		/* 0x028A	Set Write Data(STBY ON)	*/
		RegWriteA_lc898122( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_lc898122( 0x10 ) ;					/* Digital Gyro busy wait 				*/
  #endif
	}
	else
	{
  #ifdef GYROSTBY
		RegWriteA_lc898122( GRADR0,	0x6C ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_lc898122( GSETDT,	0x00 ) ;		/* 0x028A	Set Write Data(STBY OFF)	*/
		RegWriteA_lc898122( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_lc898122( 0x10 ) ;					/* Digital Gyro busy wait 				*/
  #endif
		RegWriteA_lc898122( GRADR0,	0x6B ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_lc898122( GRACC,	0x01 ) ;		/* 0x0282	Set Read Trigger ON				*/
		AccWit_lc898122( 0x01 ) ;					/* Digital Gyro busy wait 				*/
		RegReadA_lc898122( GRDAT0H, &UcRamIni ) ;	/* 0x0290 */
		
		UcRamIni &= ~0x40 ;					/* Clear Sleep bit */
  #ifdef GYROSTBY
		UcRamIni |=  0x01 ;					/* Set PLL bit */
  #endif
		
		RegWriteA_lc898122( GSETDT,	UcRamIni ) ;	// 0x028A	Set Write Data(Sleep OFF)
		RegWriteA_lc898122( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_lc898122( 0x10 ) ;					/* Digital Gyro busy wait 				*/
		
		RegReadA_lc898122( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
		RegWriteA_lc898122( GRINI, ( UcGrini & ~SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
		
		GyOutSignal_lc898122( ) ;					/* Select Gyro output signal 			*/
		
		WitTim_lc898122( 50 ) ;						// 50ms wait
		
		RegWriteA_lc898122( WC_EQON, 0x01 ) ;		// 0x0101	GYRO Equalizer ON

		ClrGyr_lc898122( 0x007F , CLR_FRAM1 );		// Gyro Delay RAM Clear
	}
 #else									/* Panasonic */
	
//	unsigned char	UcRamIni ;


	if(UcSelMode == ON)
	{
		RegWriteA_lc898122( WC_EQON, 0x00 ) ;		// 0x0101	GYRO Equalizer OFF
		RegWriteA_lc898122( GRSEL,	0x01 ) ;		/* 0x0280	Set Command Mode			*/
		RegWriteA_lc898122( GRADR0,	0x4C ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_lc898122( GSETDT,	0x02 ) ;		/* 0x028A	Set Write Data(Sleep ON)	*/
		RegWriteA_lc898122( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_lc898122( 0x10 ) ;					/* Digital Gyro busy wait 				*/
	}
	else
	{
		RegWriteA_lc898122( GRADR0,	0x4C ) ;		// 0x0283	Set Write Command
		RegWriteA_lc898122( GSETDT,	0x00 ) ;		// 0x028A	Set Write Data(Sleep OFF)
		RegWriteA_lc898122( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_lc898122( 0x10 ) ;					/* Digital Gyro busy wait 				*/
		GyOutSignal_lc898122( ) ;					/* Select Gyro output signal 			*/
		
		WitTim_lc898122( 50 ) ;						// 50ms wait
		
		RegWriteA_lc898122( WC_EQON, 0x01 ) ;		// 0x0101	GYRO Equalizer ON
		ClrGyr_lc898122( 0x007F , CLR_FRAM1 );		// Gyro Delay RAM Clear
	}
 #endif
}
#endif

#ifdef	GAIN_CONT
//********************************************************************************
// Function Name 	: AutoGainControlSw
// Retun Value		: NON
// Argment Value	: 0 :OFF  1:ON
// Explanation		: Select Gyro Signal Function
// History			: First edition 						2010.11.30 Y.Shigeoka
//********************************************************************************
void	AutoGainControlSw_lc898122( unsigned char UcModeSw )
{

	if( UcModeSw == OFF )
	{
		RegWriteA_lc898122( WG_ADJGANGXATO, 	0xA0 );					// 0x0129	X exe off
		RegWriteA_lc898122( WG_ADJGANGYATO, 	0xA0 );					// 0x012A	Y exe off
		RamWrite32A_lc898122( GANADR			 , XMAXGAIN ) ;			// Gain Through
		RamWrite32A_lc898122( GANADR | 0x0100 , YMAXGAIN ) ;			// Gain Through
	}
	else
	{
		RegWriteA_lc898122( WG_ADJGANGXATO, 	0xA3 );					// 0x0129	X exe on
		RegWriteA_lc898122( WG_ADJGANGYATO, 	0xA3 );					// 0x012A	Y exe on
	}

}
#endif


//********************************************************************************
// Function Name 	: ClrGyr
// Retun Value		: NON
// Argment Value	: UsClrFil - Select filter to clear.  If 0x0000, clears entire filter
//					  UcClrMod - 0x01: FRAM0 Clear, 0x02: FRAM1, 0x03: All RAM Clear
// Explanation		: Gyro RAM clear function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
void	ClrGyr_lc898122( unsigned short UsClrFil , unsigned char UcClrMod )
{
	unsigned char	UcRamClr;

	/*Select Filter to clear*/
	RegWriteA_lc898122( WC_RAMDLYMOD1,	(unsigned char)(UsClrFil >> 8) ) ;		// 0x018F		FRAM Initialize Hbyte
	RegWriteA_lc898122( WC_RAMDLYMOD0,	(unsigned char)UsClrFil ) ;				// 0x018E		FRAM Initialize Lbyte

	/*Enable Clear*/
	RegWriteA_lc898122( WC_RAMINITON	, UcClrMod ) ;	// 0x0102	[ - | - | - | - ][ - | - | 遅延Clr | 係数Clr ]
	
	/*Check RAM Clear complete*/
	do{
		RegReadA_lc898122( WC_RAMINITON, &UcRamClr );
		UcRamClr &= UcClrMod;
	}while( UcRamClr != 0x00 );
}


//********************************************************************************
// Function Name 	: DrvSw
// Retun Value		: NON
// Argment Value	: 0:OFF  1:ON
// Explanation		: Driver Mode setting function
// History			: First edition 						2012.04.25 Y.Shigeoka
//********************************************************************************
void	DrvSw_lc898122( unsigned char UcDrvSw )
{
	if( UcDrvSw == ON )
	{
		if( UcPwmMod_lc898122 == PWMMOD_CVL ) {
			RegWriteA_lc898122( DRVFC	, 0xF0 );			// 0x0001	Drv.MODE=1,Drv.BLK=1,MODE2,LCEN
		} else {
#ifdef	PWM_BREAK
			RegWriteA_lc898122( DRVFC	, 0x00 );			// 0x0001	Drv.MODE=0,Drv.BLK=0,MODE0B
#else
			RegWriteA_lc898122( DRVFC	, 0xC0 );			// 0x0001	Drv.MODE=1,Drv.BLK=1,MODE1
#endif
		}
	}
	else
	{
		if( UcPwmMod_lc898122 == PWMMOD_CVL ) {
			RegWriteA_lc898122( DRVFC	, 0x30 );				// 0x0001	Drvier Block Ena=0
		} else {
#ifdef	PWM_BREAK
			RegWriteA_lc898122( DRVFC	, 0x00 );				// 0x0001	Drv.MODE=0,Drv.BLK=0,MODE0B
#else
			RegWriteA_lc898122( DRVFC	, 0x00 );				// 0x0001	Drvier Block Ena=0
#endif
		}
	}
}

//********************************************************************************
// Function Name 	: AfDrvSw
// Retun Value		: NON
// Argment Value	: 0:OFF  1:ON
// Explanation		: AF Driver Mode setting function
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
void	AfDrvSw_lc898122( unsigned char UcDrvSw )
{
	if( UcDrvSw == ON )
	{
#ifdef	AF_PWMMODE
		RegWriteA_lc898122( DRVFCAF	, 0x00 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
#else
		RegWriteA_lc898122( DRVFCAF	, 0x20 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-2
#endif
		RegWriteA_lc898122( CCAAF,   0x80 ) ;				// 0x00A0	[7]=0:OFF 1:ON
	}
	else
	{
		RegWriteA_lc898122( CCAAF,   0x00 ) ;				// 0x00A0	[7]=0:OFF 1:ON
	}
}

//********************************************************************************
// Function Name 	: RamAccFixMod
// Retun Value		: NON
// Argment Value	: 0:OFF  1:ON
// Explanation		: Ram Access Fix Mode setting function
// History			: First edition 						2013.05.21 Y.Shigeoka
//********************************************************************************
void	RamAccFixMod_lc898122( unsigned char UcAccMod )
{
	switch ( UcAccMod ) {
		case OFF :
			RegWriteA_lc898122( WC_RAMACCMOD,	0x00 ) ;		// 0x018C		GRAM Access Float32bit
			break ;
		case ON :
			RegWriteA_lc898122( WC_RAMACCMOD,	0x31 ) ;		// 0x018C		GRAM Access Fix32bit
			break ;
	}
}
	

//********************************************************************************
// Function Name 	: IniAf
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Open AF Initial Setting
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
void	IniAf_lc898122( void )
{
	unsigned char	UcStbb0 ;
	
	AfDrvSw_lc898122( OFF ) ;								/* AF Drvier Block Ena=0 */
#ifdef	AF_PWMMODE
	RegWriteA_lc898122( DRVFCAF	, 0x00 );					// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
#else
	RegWriteA_lc898122( DRVFCAF	, 0x20 );					// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-2
#endif
	RegWriteA_lc898122( DRVFC3AF	, 0x00 );					// 0x0083	DGAINDAF	Gain 0
	RegWriteA_lc898122( DRVFC4AF	, 0x80 );					// 0x0084	DOFSTDAF
	RegWriteA_lc898122( PWMAAF,    0x00 ) ;					// 0x0090	AF PWM standby
	RegWriteA_lc898122( AFFC,   0x80 ) ;						// 0x0088	OpenAF/-/-
#ifdef	AF_PWMMODE
	RegWriteA_lc898122( DRVFC2AF,    0x82 ) ;				// 0x0082	AF slope3
	RegWriteA_lc898122( DRVCH3SEL,   0x02 ) ;				// 0x0085	AF only IN1 control
	RegWriteA_lc898122( PWMFCAF,     0x89 ) ;				// 0x0091	AF GND , Carrier , MODE1 
	RegWriteA_lc898122( PWMPERIODAF, 0xA0 ) ;				// 0x0099	AF none-synchronism
#else
	RegWriteA_lc898122( DRVFC2AF,    0x00 ) ;				// 0x0082	AF slope0
	RegWriteA_lc898122( DRVCH3SEL,   0x00 ) ;				// 0x0085	AF H bridge control
	RegWriteA_lc898122( PWMFCAF,     0x01 ) ;				// 0x0091	AF VREF , Carrier , MODE1
	RegWriteA_lc898122( PWMPERIODAF, 0x20 ) ;				// 0x0099	AF none-synchronism
#endif
	RegWriteA_lc898122( CCFCAF,   0x40 ) ;					// 0x00A1	GND/-
	
	RegReadA_lc898122( STBB0 	, &UcStbb0 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	UcStbb0 &= 0x7F ;
	RegWriteA_lc898122( STBB0, UcStbb0 ) ;			// 0x0250	OIS standby
	RegWriteA_lc898122( STBB1, 0x00 ) ;				// 0x0264	All standby
	
	/* AF Initial setting */
	RegWriteA_lc898122( FSTMODE,		FSTMODE_AF ) ;		// 0x0302
	RamWriteA_lc898122( RWEXD1_L,	RWEXD1_L_AF ) ;		// 0x0396 - 0x0397 (Register continuos write)
	RamWriteA_lc898122( RWEXD2_L,	RWEXD2_L_AF ) ;		// 0x0398 - 0x0399 (Register continuos write)
	RamWriteA_lc898122( RWEXD3_L,	RWEXD3_L_AF ) ;		// 0x039A - 0x039B (Register continuos write)
	RegWriteA_lc898122( FSTCTIME,	FSTCTIME_AF ) ;		// 0x0303 	
	RamWriteA_lc898122( TCODEH,		0x0000 ) ;			// 0x0304 - 0x0305 (Register continuos write)
	
#ifdef	AF_PWMMODE
	RegWriteA_lc898122( PWMAAF,    0x80 ) ;			// 0x0090	AF PWM enable
#endif

	UcStbb0 |= 0x80 ;
	RegWriteA_lc898122( STBB0, UcStbb0 ) ;			// 0x0250	
	RegWriteA_lc898122( STBB1	, 0x05 ) ;			// 0x0264	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]

	AfDrvSw_lc898122( ON ) ;								/* AF Drvier Block Ena=1 */
}



//********************************************************************************
// Function Name 	: IniPtAve
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Pan/Tilt Average parameter setting function
// History			: First edition 						2013.09.26 Y.Shigeoka
//********************************************************************************
void	IniPtAve_lc898122( void )
{
	RegWriteA_lc898122( WG_PANSTT1DWNSMP0, 0x00 );		// 0x0134
	RegWriteA_lc898122( WG_PANSTT1DWNSMP1, 0x00 );		// 0x0135
	RegWriteA_lc898122( WG_PANSTT2DWNSMP0, 0x90 );		// 0x0136 400
	RegWriteA_lc898122( WG_PANSTT2DWNSMP1, 0x01 );		// 0x0137
	RegWriteA_lc898122( WG_PANSTT3DWNSMP0, 0x64 );		// 0x0138 100
	RegWriteA_lc898122( WG_PANSTT3DWNSMP1, 0x00 );		// 0x0139
	RegWriteA_lc898122( WG_PANSTT4DWNSMP0, 0x00 );		// 0x013A
	RegWriteA_lc898122( WG_PANSTT4DWNSMP1, 0x00 );		// 0x013B

	RamWrite32A_lc898122( st1mean, 0x3f800000 );		// 0x1235
	RamWrite32A_lc898122( st2mean, 0x3B23D700 );		// 0x1236	1/400
	RamWrite32A_lc898122( st3mean, 0x3C23D700 );		// 0x1237	1/100
	RamWrite32A_lc898122( st4mean, 0x3f800000 );		// 0x1238
			
}
	
//********************************************************************************
// Function Name 	: IniPtMovMod
// Retun Value		: NON
// Argment Value	: OFF:Still  ON:Movie
// Explanation		: Pan/Tilt parameter setting by mode function
// History			: First edition 						2013.09.26 Y.Shigeoka
//********************************************************************************
void	IniPtMovMod_lc898122( unsigned char UcPtMod )
{
	switch ( UcPtMod ) {
		case OFF :
			RegWriteA_lc898122( WG_PANSTTSETGYRO, 	0x00 );		// 0x0154
			RegWriteA_lc898122( WG_PANSTTSETGAIN, 	0x54 );		// 0x0155
			RegWriteA_lc898122( WG_PANSTTSETISTP, 	0x14 );		// 0x0156
			RegWriteA_lc898122( WG_PANSTTSETIFTR,	0x94 );		// 0x0157
			RegWriteA_lc898122( WG_PANSTTSETLFTR,	0x00 );		// 0x0158

			break ;
		case ON :
			RegWriteA_lc898122( WG_PANSTTSETGYRO, 	0x00 );		// 0x0154
			RegWriteA_lc898122( WG_PANSTTSETGAIN, 	0x00 );		// 0x0155
			RegWriteA_lc898122( WG_PANSTTSETISTP, 	0x14 );		// 0x0156
			RegWriteA_lc898122( WG_PANSTTSETIFTR,	0x94 );		// 0x0157
			RegWriteA_lc898122( WG_PANSTTSETLFTR,	0x00 );		// 0x0158
			break ;
	}
}

//********************************************************************************
// Function Name 	: ChkCvr
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Check Cver function
// History			: First edition 						2013.10.03 Y.Shigeoka
//********************************************************************************
void	ChkCvr_lc898122( void )
{
	RegReadA_lc898122( CVER ,	&UcCvrCod );		// 0x027E
	RegWriteA_lc898122( MDLREG ,	MDL_VER );			// 0x00FF	Model
	RegWriteA_lc898122( VRREG ,	FW_VER );			// 0x02D0	Version
}

