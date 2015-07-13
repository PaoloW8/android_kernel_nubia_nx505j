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
#include	"Ois_lc898122_sharp.h"
#include	"OisFil_lc898122_sharp.h"
#include	"OisDef_lc898122_sharp.h"

//**************************
//	Local Function Prottype	
//**************************
void	IniClk_lc898122_sharp( void ) ;		// Clock Setting
void	IniIop_lc898122_sharp( void ) ;		// I/O Port Initial Setting
void	IniMon_lc898122_sharp( void ) ;		// Monitor & Other Initial Setting
void	IniSrv_lc898122_sharp( void ) ;		// Servo Register Initial Setting
void	IniGyr_lc898122_sharp( void ) ;		// Gyro Filter Register Initial Setting
void	IniFil_lc898122_sharp( void ) ;		// Gyro Filter Initial Parameter Setting
void	IniAdj_lc898122_sharp( void ) ;		// Adjust Fix Value Setting
void	IniCmd_lc898122_sharp( void ) ;		// Command Execute Process Initial
void	IniDgy_lc898122_sharp( void ) ;		// Digital Gyro Initial Setting
void	IniAf_lc898122_sharp( void ) ;			// Open AF Initial Setting
void	IniPtAve_lc898122_sharp( void ) ;		// Average setting
#ifdef	AF_MID_MOUNT
void	SetTregAf_lc898122_sharp( unsigned short );
#endif	//AF_MID_MOUNT


//********************************************************************************
// Function Name 	: IniSet_lc898122_sharp
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Initial Setting Function
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniSet_lc898122_sharp( void )
{
	// Command Execute Process Initial
	IniCmd_lc898122_sharp() ;
	// Clock Setting
	IniClk_lc898122_sharp() ;
	// I/O Port Initial Setting
	IniIop_lc898122_sharp() ;
	// DigitalGyro Initial Setting
	IniDgy_lc898122_sharp() ;
	// Monitor & Other Initial Setting
	IniMon_lc898122_sharp() ;
	// Servo Initial Setting
	IniSrv_lc898122_sharp() ;
	// Gyro Filter Initial Setting
	IniGyr_lc898122_sharp() ;
	// Gyro Filter Initial Setting
	IniFil_lc898122_sharp() ;
	// Adjust Fix Value Setting
	IniAdj_lc898122_sharp() ;

}

//********************************************************************************
// Function Name 	: IniSetAf_lc898122_sharp
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Initial AF Setting Function
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
void	IniSetAf_lc898122_sharp( void )
{
	// Command Execute Process Initial
	IniCmd_lc898122_sharp() ;
	// Clock Setting
	IniClk_lc898122_sharp() ;
	// AF Initial Setting
	IniAf_lc898122_sharp() ;

}



//********************************************************************************
// Function Name 	: IniClk_lc898122_sharp
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Clock Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniClk_lc898122_sharp( void )
{
	ChkCvr_lc898122_sharp() ;									/* Read Cver */
	
	/*OSC Enables*/
	UcOscAdjFlg_lc898122_sharp_lc898122_sharp	= 0 ;							// Osc adj flag 
	
#ifdef	DEF_SET
	/*OSC ENABLE*/
	RegWriteA_lc898122_sharp( OSCSTOP,		0x00 ) ;			// 0x0256
	RegWriteA_lc898122_sharp( OSCSET,		0x90 ) ;			// 0x0257	OSC ini
	RegWriteA_lc898122_sharp( OSCCNTEN,	0x00 ) ;			// 0x0258	OSC Cnt disable
#endif
	/*Clock Enables*/
	RegWriteA_lc898122_sharp( CLKON,		0x1F ) ;			// 0x020B

#ifdef	USE_EXTCLK_ALL
	RegWriteA_lc898122_sharp( CLKSEL,		0x07 ) ;			// 0x020C	All
#else
 #ifdef	USE_EXTCLK_PWM
	RegWriteA_lc898122_sharp( CLKSEL,		0x01 ) ;			// 0x020C	only PWM
 #else
  #ifdef	DEF_SET
	RegWriteA_lc898122_sharp( CLKSEL,		0x00 ) ;			// 0x020C	
  #endif
 #endif
#endif
	
#ifdef	USE_EXTCLK_ALL	// 24MHz
	RegWriteA_lc898122_sharp( PWMDIV,		0x00 ) ;			// 0x0210	24MHz/1
	RegWriteA_lc898122_sharp( SRVDIV,		0x00 ) ;			// 0x0211	24MHz/1
	RegWriteA_lc898122_sharp( GIFDIV,		0x02 ) ;			// 0x0212	24MHz/2 = 12MHz
	RegWriteA_lc898122_sharp( AFPWMDIV,	0x00 ) ;			// 0x0213	24MHz/1 = 24MHz
	RegWriteA_lc898122_sharp( OPAFDIV,		0x02 ) ;			// 0x0214	24MHz/2 = 12MHz
#else
 #ifdef	DEF_SET
	RegWriteA_lc898122_sharp( PWMDIV,		0x00 ) ;			// 0x0210	48MHz/1
	RegWriteA_lc898122_sharp( SRVDIV,		0x00 ) ;			// 0x0211	48MHz/1
	RegWriteA_lc898122_sharp( GIFDIV,		0x03 ) ;			// 0x0212	48MHz/3 = 16MHz
  #ifdef	AF_PWMMODE
	RegWriteA_lc898122_sharp( AFPWMDIV,	0x00 ) ;			// 0x0213	48MHz/1
  #else
	RegWriteA_lc898122_sharp( AFPWMDIV,	0x02 ) ;			// 0x0213	48MHz/2 = 24MHz
  #endif
	RegWriteA_lc898122_sharp( OPAFDIV,		0x04 ) ;			// 0x0214	48MHz/6 = 8MHz
 #endif
#endif
}



//********************************************************************************
// Function Name 	: IniIop_lc898122_sharp
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: I/O Port Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniIop_lc898122_sharp( void )
{
#ifdef	DEF_SET
	/*set IOP direction*/
	RegWriteA_lc898122_sharp( P0LEV,		0x00 ) ;	// 0x0220	[ - 	| - 	| WLEV5 | WLEV4 ][ WLEV3 | WLEV2 | WLEV1 | WLEV0 ]
	RegWriteA_lc898122_sharp( P0DIR,		0x00 ) ;	// 0x0221	[ - 	| - 	| DIR5	| DIR4	][ DIR3  | DIR2  | DIR1  | DIR0  ]
	/*set pull up/down*/
	RegWriteA_lc898122_sharp( P0PON,		0x0F ) ;	// 0x0222	[ -    | -	  | PON5 | PON4 ][ PON3  | PON2 | PON1 | PON0 ]
	RegWriteA_lc898122_sharp( P0PUD,		0x0F ) ;	// 0x0223	[ -    | -	  | PUD5 | PUD4 ][ PUD3  | PUD2 | PUD1 | PUD0 ]
#endif
	/*select IOP signal*/
#ifdef	USE_3WIRE_DGYRO
	RegWriteA_lc898122_sharp( IOP1SEL,		0x02 ); 	// 0x0231	IOP1 : IOP1
#else
	RegWriteA_lc898122_sharp( IOP1SEL,		0x00 ); 	// 0x0231	IOP1 : DGDATAIN (ATT:0236h[0]=1)
#endif
#ifdef	DEF_SET
	RegWriteA_lc898122_sharp( IOP0SEL,		0x02 ); 	// 0x0230	IOP0 : IOP0
	RegWriteA_lc898122_sharp( IOP2SEL,		0x02 ); 	// 0x0232	IOP2 : IOP2
	RegWriteA_lc898122_sharp( IOP3SEL,		0x00 ); 	// 0x0233	IOP3 : DGDATAOUT
	RegWriteA_lc898122_sharp( IOP4SEL,		0x00 ); 	// 0x0234	IOP4 : DGSCLK
	RegWriteA_lc898122_sharp( IOP5SEL,		0x00 ); 	// 0x0235	IOP5 : DGSSB
	RegWriteA_lc898122_sharp( DGINSEL,		0x00 ); 	// 0x0236	DGDATAIN 0:IOP1 1:IOP2
	RegWriteA_lc898122_sharp( I2CSEL,		0x00 );		// 0x0248	I2C noise reduction ON
	RegWriteA_lc898122_sharp( DLMODE,		0x00 );		// 0x0249	Download OFF
#endif
	
}

//********************************************************************************
// Function Name 	: IniDgy_lc898122_sharp
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Digital Gyro Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniDgy_lc898122_sharp( void )
{
 #ifdef USE_INVENSENSE
	unsigned char	UcGrini ;
 #endif

	/*************/
	/*For ST gyro*/
	/*************/
	
	/*Set SPI Type*/
 #ifdef USE_3WIRE_DGYRO
	RegWriteA_lc898122_sharp( SPIM 	, 0x00 );					// 0x028F 	[ - | - | - | - ][ - | - | - | DGSPI4 ]
 #else
	RegWriteA_lc898122_sharp( SPIM 	, 0x01 );					// 0x028F 	[ - | - | - | - ][ - | - | - | DGSPI4 ]
 #endif												// 		DGSPI4	0: 3-wire SPI, 1: 4-wire SPI
	
	/*Set to Command Mode*/
	RegWriteA_lc898122_sharp( GRSEL	, 0x01 );					// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

	/*Digital Gyro Read settings*/
	RegWriteA_lc898122_sharp( GRINI	, 0x80 );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]


 #ifdef USE_INVENSENSE
	RegReadA_lc898122_sharp( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
	RegWriteA_lc898122_sharp( GRINI, ( UcGrini | SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
	
	RegWriteA_lc898122_sharp( GRADR0,	0x6A ) ;					// 0x0283	Set I2C_DIS
	RegWriteA_lc898122_sharp( GSETDT,	0x10 ) ;					// 0x028A	Set Write Data
	RegWriteA_lc898122_sharp( GRACC,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	AccWit_lc898122_sharp( 0x10 ) ;								/* Digital Gyro busy wait 				*/

	RegWriteA_lc898122_sharp( GRADR0,	0x1B ) ;					// 0x0283	Set GYRO_CONFIG
	RegWriteA_lc898122_sharp( GSETDT,	( FS_SEL << 3) ) ;			// 0x028A	Set Write Data
	RegWriteA_lc898122_sharp( GRACC,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	AccWit_lc898122_sharp( 0x10 ) ;								/* Digital Gyro busy wait 				*/

	RegReadA_lc898122_sharp( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
	RegWriteA_lc898122_sharp( GRINI, ( UcGrini & ~SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
 #endif // USE_INVENSENSE

	
	RegWriteA_lc898122_sharp( RDSEL,	0x7C ) ;					// 0x028B	RDSEL(Data1 and 2 for continuos mode)
	
	GyOutSignal_lc898122_sharp() ;
}


//********************************************************************************
// Function Name 	: IniMon_lc898122_sharp
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Monitor & Other Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniMon_lc898122_sharp( void )
{
	RegWriteA_lc898122_sharp( PWMMONA, 0x00 ) ;				// 0x0030	0:off
	
	RegWriteA_lc898122_sharp( MONSELA, 0x5C ) ;				// 0x0270	DLYMON1
	RegWriteA_lc898122_sharp( MONSELB, 0x5D ) ;				// 0x0271	DLYMON2
	RegWriteA_lc898122_sharp( MONSELC, 0x00 ) ;				// 0x0272	
	RegWriteA_lc898122_sharp( MONSELD, 0x00 ) ;				// 0x0273	

	// Monitor Circuit
	RegWriteA_lc898122_sharp( WC_PINMON1,	0x00 ) ;			// 0x01C0	Filter Monitor
	RegWriteA_lc898122_sharp( WC_PINMON2,	0x00 ) ;			// 0x01C1	
	RegWriteA_lc898122_sharp( WC_PINMON3,	0x00 ) ;			// 0x01C2	
	RegWriteA_lc898122_sharp( WC_PINMON4,	0x00 ) ;			// 0x01C3	
	/* Delay Monitor */
	RegWriteA_lc898122_sharp( WC_DLYMON11,	0x04 ) ;			// 0x01C5	DlyMonAdd1[10:8]
	RegWriteA_lc898122_sharp( WC_DLYMON10,	0x40 ) ;			// 0x01C4	DlyMonAdd1[ 7:0]
	RegWriteA_lc898122_sharp( WC_DLYMON21,	0x04 ) ;			// 0x01C7	DlyMonAdd2[10:8]
	RegWriteA_lc898122_sharp( WC_DLYMON20,	0xC0 ) ;			// 0x01C6	DlyMonAdd2[ 7:0]
	RegWriteA_lc898122_sharp( WC_DLYMON31,	0x00 ) ;			// 0x01C9	DlyMonAdd3[10:8]
	RegWriteA_lc898122_sharp( WC_DLYMON30,	0x00 ) ;			// 0x01C8	DlyMonAdd3[ 7:0]
	RegWriteA_lc898122_sharp( WC_DLYMON41,	0x00 ) ;			// 0x01CB	DlyMonAdd4[10:8]
	RegWriteA_lc898122_sharp( WC_DLYMON40,	0x00 ) ;			// 0x01CA	DlyMonAdd4[ 7:0]

/* Monitor */
	RegWriteA_lc898122_sharp( PWMMONA, 0x80 ) ;				// 0x0030	1:on 
//	RegWriteA_lc898122_sharp( IOP0SEL,		0x01 ); 			// 0x0230	IOP0 : MONA
/**/


}

//********************************************************************************
// Function Name 	: IniSrv_lc898122_sharp
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Servo Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniSrv_lc898122_sharp( void )
{
	unsigned char	UcStbb0 ;

	UcPwmMod_lc898122_sharp = INIT_PWMMODE ;						// Driver output mode

	RegWriteA_lc898122_sharp( WC_EQON,		0x00 ) ;				// 0x0101		Filter Calcu
	RegWriteA_lc898122_sharp( WC_RAMINITON,0x00 ) ;				// 0x0102		
	ClrGyr_lc898122_sharp( 0x0000 , CLR_ALL_RAM );					// All Clear
	
	RegWriteA_lc898122_sharp( WH_EQSWX,	0x02 ) ;				// 0x0170		[ - | - | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
	RegWriteA_lc898122_sharp( WH_EQSWY,	0x02 ) ;				// 0x0171		[ - | - | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
	
	RamAccFixMod_lc898122_sharp( OFF ) ;							// 32bit Float mode
	
	/* Monitor Gain */
	RamWrite32A_lc898122_sharp( dm1g, 0x3F800000 ) ;				// 0x109A
	RamWrite32A_lc898122_sharp( dm2g, 0x3F800000 ) ;				// 0x109B
	RamWrite32A_lc898122_sharp( dm3g, 0x3F800000 ) ;				// 0x119A
	RamWrite32A_lc898122_sharp( dm4g, 0x3F800000 ) ;				// 0x119B
	
	/* Hall output limitter */
	RamWrite32A_lc898122_sharp( sxlmta1,   0x3F800000 ) ;			// 0x10E6		Hall X output Limit
	RamWrite32A_lc898122_sharp( sylmta1,   0x3F800000 ) ;			// 0x11E6		Hall Y output Limit
	
	/* Emargency Stop */
	RegWriteA_lc898122_sharp( WH_EMGSTPON,	0x00 ) ;				// 0x0178		Emargency Stop OFF
	RegWriteA_lc898122_sharp( WH_EMGSTPTMR,0xFF ) ;				// 0x017A		255*(16/23.4375kHz)=174ms
	
	RamWrite32A_lc898122_sharp( sxemglev,   0x3F800000 ) ;			// 0x10EC		Hall X Emargency threshold
	RamWrite32A_lc898122_sharp( syemglev,   0x3F800000 ) ;			// 0x11EC		Hall Y Emargency threshold
	
	/* Hall Servo smoothing */
	RegWriteA_lc898122_sharp( WH_SMTSRVON,	0x00 ) ;				// 0x017C		Smooth Servo OFF
#ifdef	USE_EXTCLK_ALL	// 24MHz
	RegWriteA_lc898122_sharp( WH_SMTSRVSMP,0x03 ) ;				// 0x017D		2.7ms=2^03/11.718kHz
	RegWriteA_lc898122_sharp( WH_SMTTMR,	0x00 ) ;				// 0x017E		1.3ms=(0+1)*16/11.718kHz
#else
	RegWriteA_lc898122_sharp( WH_SMTSRVSMP,0x06 ) ;				// 0x017D		2.7ms=2^06/23.4375kHz
	RegWriteA_lc898122_sharp( WH_SMTTMR,	0x01 ) ;				// 0x017E		1.3ms=(1+1)*16/23.4375kHz
#endif
	
	RamWrite32A_lc898122_sharp( sxsmtav,   0xBC800000 ) ;			// 0x10ED		1/64 X smoothing ave coefficient
	RamWrite32A_lc898122_sharp( sysmtav,   0xBC800000 ) ;			// 0x11ED		1/64 Y smoothing ave coefficient
	RamWrite32A_lc898122_sharp( sxsmtstp,  0x3AE90466 ) ;			// 0x10EE		0.001778 X smoothing offset
	RamWrite32A_lc898122_sharp( sysmtstp,  0x3AE90466 ) ;			// 0x11EE		0.001778 Y smoothing offset
	
	/* High-dimensional correction  */
	RegWriteA_lc898122_sharp( WH_HOFCON,	0x11 ) ;				// 0x0174		OUT 3x3
	
#ifdef	ACTREG_6P5OHM
	/* (0.3750114X^3+0.5937681X)*(0.3750114X^3+0.5937681X) 6.5ohm*/
	/* Front */
	RamWrite32A_lc898122_sharp( sxiexp3,   0x3EC0017F ) ;			// 0x10BA		
	RamWrite32A_lc898122_sharp( sxiexp2,   0x00000000 ) ;			// 0x10BB		
	RamWrite32A_lc898122_sharp( sxiexp1,   0x3F180130 ) ;			// 0x10BC		
	RamWrite32A_lc898122_sharp( sxiexp0,   0x00000000 ) ;			// 0x10BD		
	RamWrite32A_lc898122_sharp( sxiexp,    0x3F800000 ) ;			// 0x10BE		

	RamWrite32A_lc898122_sharp( syiexp3,   0x3EC0017F ) ;			// 0x11BA		
	RamWrite32A_lc898122_sharp( syiexp2,   0x00000000 ) ;			// 0x11BB		
	RamWrite32A_lc898122_sharp( syiexp1,   0x3F180130 ) ;			// 0x11BC		
	RamWrite32A_lc898122_sharp( syiexp0,   0x00000000 ) ;			// 0x11BD		
	RamWrite32A_lc898122_sharp( syiexp,    0x3F800000 ) ;			// 0x11BE		

	/* Back */
	RamWrite32A_lc898122_sharp( sxoexp3,   0x3EC0017F ) ;			// 0x10FA		
	RamWrite32A_lc898122_sharp( sxoexp2,   0x00000000 ) ;			// 0x10FB		
	RamWrite32A_lc898122_sharp( sxoexp1,   0x3F180130 ) ;			// 0x10FC		
	RamWrite32A_lc898122_sharp( sxoexp0,   0x00000000 ) ;			// 0x10FD		
	RamWrite32A_lc898122_sharp( sxoexp,    0x3F800000 ) ;			// 0x10FE		

	RamWrite32A_lc898122_sharp( syoexp3,   0x3EC0017F ) ;			// 0x11FA		
	RamWrite32A_lc898122_sharp( syoexp2,   0x00000000 ) ;			// 0x11FB		
	RamWrite32A_lc898122_sharp( syoexp1,   0x3F180130 ) ;			// 0x11FC		
	RamWrite32A_lc898122_sharp( syoexp0,   0x00000000 ) ;			// 0x11FD		
	RamWrite32A_lc898122_sharp( syoexp,    0x3F800000 ) ;			// 0x11FE		
#else
	/* (0.4531388X^3+0.4531388X)*(0.4531388X^3+0.4531388X) 15ohm*/
	/* Front */
	RamWrite32A_lc898122_sharp( sxiexp3,   0x3EE801CF ) ;			// 0x10BA		
	RamWrite32A_lc898122_sharp( sxiexp2,   0x00000000 ) ;			// 0x10BB		
	RamWrite32A_lc898122_sharp( sxiexp1,   0x3EE801CF ) ;			// 0x10BC		
	RamWrite32A_lc898122_sharp( sxiexp0,   0x00000000 ) ;			// 0x10BD		
	RamWrite32A_lc898122_sharp( sxiexp,    0x3F800000 ) ;			// 0x10BE		

	RamWrite32A_lc898122_sharp( syiexp3,   0x3EE801CF ) ;			// 0x11BA		
	RamWrite32A_lc898122_sharp( syiexp2,   0x00000000 ) ;			// 0x11BB		
	RamWrite32A_lc898122_sharp( syiexp1,   0x3EE801CF ) ;			// 0x11BC		
	RamWrite32A_lc898122_sharp( syiexp0,   0x00000000 ) ;			// 0x11BD		
	RamWrite32A_lc898122_sharp( syiexp,    0x3F800000 ) ;			// 0x11BE		

	/* Back */
	RamWrite32A_lc898122_sharp( sxoexp3,   0x3EE801CF ) ;			// 0x10FA		
	RamWrite32A_lc898122_sharp( sxoexp2,   0x00000000 ) ;			// 0x10FB		
	RamWrite32A_lc898122_sharp( sxoexp1,   0x3EE801CF ) ;			// 0x10FC		
	RamWrite32A_lc898122_sharp( sxoexp0,   0x00000000 ) ;			// 0x10FD		
	RamWrite32A_lc898122_sharp( sxoexp,    0x3F800000 ) ;			// 0x10FE		

	RamWrite32A_lc898122_sharp( syoexp3,   0x3EE801CF ) ;			// 0x11FA		
	RamWrite32A_lc898122_sharp( syoexp2,   0x00000000 ) ;			// 0x11FB		
	RamWrite32A_lc898122_sharp( syoexp1,   0x3EE801CF ) ;			// 0x11FC		
	RamWrite32A_lc898122_sharp( syoexp0,   0x00000000 ) ;			// 0x11FD		
	RamWrite32A_lc898122_sharp( syoexp,    0x3F800000 ) ;			// 0x11FE		
#endif
	
	/* Sine wave */
#ifdef	DEF_SET
	RegWriteA_lc898122_sharp( WC_SINON,	0x00 ) ;				// 0x0180		Sin Wave off
	RegWriteA_lc898122_sharp( WC_SINFRQ0,	0x00 ) ;				// 0x0181		
	RegWriteA_lc898122_sharp( WC_SINFRQ1,	0x60 ) ;				// 0x0182		
	RegWriteA_lc898122_sharp( WC_SINPHSX,	0x00 ) ;				// 0x0183		
	RegWriteA_lc898122_sharp( WC_SINPHSY,	0x20 ) ;				// 0x0184		
	
	/* AD over sampling */
	RegWriteA_lc898122_sharp( WC_ADMODE,	0x06 ) ;				// 0x0188		AD Over Sampling
	
	/* Measure mode */
	RegWriteA_lc898122_sharp( WC_MESMODE,		0x00 ) ;			// 0x0190		Measurement Mode
	RegWriteA_lc898122_sharp( WC_MESSINMODE,	0x00 ) ;			// 0x0191		
	RegWriteA_lc898122_sharp( WC_MESLOOP0,		0x08 ) ;			// 0x0192		
	RegWriteA_lc898122_sharp( WC_MESLOOP1,		0x02 ) ;			// 0x0193		
	RegWriteA_lc898122_sharp( WC_MES1ADD0,		0x00 ) ;			// 0x0194		
	RegWriteA_lc898122_sharp( WC_MES1ADD1,		0x00 ) ;			// 0x0195		
	RegWriteA_lc898122_sharp( WC_MES2ADD0,		0x00 ) ;			// 0x0196		
	RegWriteA_lc898122_sharp( WC_MES2ADD1,		0x00 ) ;			// 0x0197		
	RegWriteA_lc898122_sharp( WC_MESABS,		0x00 ) ;			// 0x0198		
	RegWriteA_lc898122_sharp( WC_MESWAIT,		0x00 ) ;			// 0x0199		
	
	/* auto measure */
	RegWriteA_lc898122_sharp( WC_AMJMODE,		0x00 ) ;			// 0x01A0		Automatic measurement mode
	
	RegWriteA_lc898122_sharp( WC_AMJLOOP0,		0x08 ) ;			// 0x01A2		Self-Aadjustment
	RegWriteA_lc898122_sharp( WC_AMJLOOP1,		0x02 ) ;			// 0x01A3		
	RegWriteA_lc898122_sharp( WC_AMJIDL0,		0x02 ) ;			// 0x01A4		
	RegWriteA_lc898122_sharp( WC_AMJIDL1,		0x00 ) ;			// 0x01A5		
	RegWriteA_lc898122_sharp( WC_AMJ1ADD0,		0x00 ) ;			// 0x01A6		
	RegWriteA_lc898122_sharp( WC_AMJ1ADD1,		0x00 ) ;			// 0x01A7		
	RegWriteA_lc898122_sharp( WC_AMJ2ADD0,		0x00 ) ;			// 0x01A8		
	RegWriteA_lc898122_sharp( WC_AMJ2ADD1,		0x00 ) ;			// 0x01A9		
	
	/* Data Pass */
	RegWriteA_lc898122_sharp( WC_DPI1ADD0,		0x00 ) ;			// 0x01B0		Data Pass
	RegWriteA_lc898122_sharp( WC_DPI1ADD1,		0x00 ) ;			// 0x01B1		
	RegWriteA_lc898122_sharp( WC_DPI2ADD0,		0x00 ) ;			// 0x01B2		
	RegWriteA_lc898122_sharp( WC_DPI2ADD1,		0x00 ) ;			// 0x01B3		
	RegWriteA_lc898122_sharp( WC_DPI3ADD0,		0x00 ) ;			// 0x01B4		
	RegWriteA_lc898122_sharp( WC_DPI3ADD1,		0x00 ) ;			// 0x01B5		
	RegWriteA_lc898122_sharp( WC_DPI4ADD0,		0x00 ) ;			// 0x01B6		
	RegWriteA_lc898122_sharp( WC_DPI4ADD1,		0x00 ) ;			// 0x01B7		
	RegWriteA_lc898122_sharp( WC_DPO1ADD0,		0x00 ) ;			// 0x01B8		Data Pass
	RegWriteA_lc898122_sharp( WC_DPO1ADD1,		0x00 ) ;			// 0x01B9		
	RegWriteA_lc898122_sharp( WC_DPO2ADD0,		0x00 ) ;			// 0x01BA		
	RegWriteA_lc898122_sharp( WC_DPO2ADD1,		0x00 ) ;			// 0x01BB		
	RegWriteA_lc898122_sharp( WC_DPO3ADD0,		0x00 ) ;			// 0x01BC		
	RegWriteA_lc898122_sharp( WC_DPO3ADD1,		0x00 ) ;			// 0x01BD		
	RegWriteA_lc898122_sharp( WC_DPO4ADD0,		0x00 ) ;			// 0x01BE		
	RegWriteA_lc898122_sharp( WC_DPO4ADD1,		0x00 ) ;			// 0x01BF		
	RegWriteA_lc898122_sharp( WC_DPON,			0x00 ) ;			// 0x0105		Data pass OFF
	
	/* Interrupt Flag */
	RegWriteA_lc898122_sharp( WC_INTMSK,	0xFF ) ;				// 0x01CE		All Mask
	
#endif
	
	/* Ram Access */
	RamAccFixMod_lc898122_sharp( OFF ) ;							// 32bit float mode

	// PWM Signal Generate
	DrvSw_lc898122_sharp( OFF ) ;									/* 0x0070	Drvier Block Ena=0 */
	RegWriteA_lc898122_sharp( DRVFC2	, 0x90 );					// 0x0002	Slope 3, Dead Time = 30 ns
	RegWriteA_lc898122_sharp( DRVSELX	, 0xFF );					// 0x0003	PWM X drv max current  DRVSELX[7:0]
	RegWriteA_lc898122_sharp( DRVSELY	, 0xFF );					// 0x0004	PWM Y drv max current  DRVSELY[7:0]

#ifdef	PWM_BREAK
 #ifdef	PWM_CAREER_TEST
	RegWriteA_lc898122_sharp( PWMFC,		0x7C ) ;				// 0x0011	VREF, PWMFRQ=7:PWMCLK(EXCLK)/PWMPERIODX[5:2]=18MHz/4=4.5MHz, MODE0B, 11-bit Accuracy
 #else		//PWM_CAREER_TEST
	if( UcCvrCod_lc898122_sharp == CVER122 ) {
		RegWriteA_lc898122_sharp( PWMFC,   0x2D ) ;				// 0x0011	VREF, PWMCLK/256, MODE0B, 12Bit Accuracy
	} else {
		RegWriteA_lc898122_sharp( PWMFC,   0x3D ) ;				// 0x0011	VREF, PWMCLK/128, MODE0B, 12Bit Accuracy
	}
 #endif	//PWM_CAREER_TEST
#else
	RegWriteA_lc898122_sharp( PWMFC,   0x21 ) ;					// 0x0011	VREF, PWMCLK/256, MODE1, 12Bit Accuracy
#endif

#ifdef	USE_VH_SYNC
	RegWriteA_lc898122_sharp( STROBEFC,	0x80 ) ;				// 0x001C	外部入力Strobe信号の有効
	RegWriteA_lc898122_sharp( STROBEDLYX,	0x00 ) ;				// 0x001D	Delay
	RegWriteA_lc898122_sharp( STROBEDLYY,	0x00 ) ;				// 0x001E	Delay
#endif	//USE_VH_SYNC

	RegWriteA_lc898122_sharp( PWMA,    0x00 ) ;					// 0x0010	PWM X/Y standby
	RegWriteA_lc898122_sharp( PWMDLYX,  0x04 ) ;					// 0x0012	X Phase Delay Setting
	RegWriteA_lc898122_sharp( PWMDLYY,  0x04 ) ;					// 0x0013	Y Phase Delay Setting
	
#ifdef	DEF_SET
	RegWriteA_lc898122_sharp( DRVCH1SEL,	0x00 ) ;				// 0x0005	OUT1/OUT2	X axis
	RegWriteA_lc898122_sharp( DRVCH2SEL,	0x00 ) ;				// 0x0006	OUT3/OUT4	Y axis
	
	RegWriteA_lc898122_sharp( PWMDLYTIMX,	0x00 ) ;				// 0x0014		PWM Timing
	RegWriteA_lc898122_sharp( PWMDLYTIMY,	0x00 ) ;				// 0x0015		PWM Timing
#endif
	
	if( UcCvrCod_lc898122_sharp == CVER122 ) {
#ifdef	PWM_CAREER_TEST
		RegWriteA_lc898122_sharp( PWMPERIODY,	0xD0 ) ;			// 0x001A	11010000h --> PWMPERIODX[5:2] = 0100h = 4
		RegWriteA_lc898122_sharp( PWMPERIODY2,	0xD0 ) ;			// 0x001B	11010000h --> PWMPERIODY[5:2] = 0100h = 4
#else		//PWM_CAREER_TEST
		RegWriteA_lc898122_sharp( PWMPERIODY,	0x00 ) ;			// 0x001A		PWM Carrier Freq
		RegWriteA_lc898122_sharp( PWMPERIODY2,	0x00 ) ;			// 0x001B		PWM Carrier Freq
#endif
	} else {
#ifdef	PWM_CAREER_TEST
		RegWriteA_lc898122_sharp( PWMPERIODX,	0xF2 ) ;			// 0x0018		PWM Carrier Freq
		RegWriteA_lc898122_sharp( PWMPERIODX2,	0x00 ) ;			// 0x0019		PWM Carrier Freq
		RegWriteA_lc898122_sharp( PWMPERIODY,	0xF2 ) ;			// 0x001A		PWM Carrier Freq
		RegWriteA_lc898122_sharp( PWMPERIODY2,	0x00 ) ;			// 0x001B		PWM Carrier Freq
#else		//PWM_CAREER_TEST
	#ifdef	USE_EXTCLK_PWM
		RegWriteA_lc898122_sharp( PWMPERIODX,	0x84 ) ;			// 0x0018		PWM Carrier Freq
		RegWriteA_lc898122_sharp( PWMPERIODX2,	0x00 ) ;			// 0x0019		PWM Carrier Freq
		RegWriteA_lc898122_sharp( PWMPERIODY,	0x84 ) ;			// 0x001A		PWM Carrier Freq
		RegWriteA_lc898122_sharp( PWMPERIODY2,	0x00 ) ;			// 0x001B		PWM Carrier Freq
	#else	//USE_EXTCLK_PWM
		RegWriteA_lc898122_sharp( PWMPERIODX,	0x00 ) ;			// 0x0018		PWM Carrier Freq
		RegWriteA_lc898122_sharp( PWMPERIODX2,	0x00 ) ;			// 0x0019		PWM Carrier Freq
		RegWriteA_lc898122_sharp( PWMPERIODY,	0x00 ) ;			// 0x001A		PWM Carrier Freq
		RegWriteA_lc898122_sharp( PWMPERIODY2,	0x00 ) ;			// 0x001B		PWM Carrier Freq
	#endif	//USE_EXTCLK_PWM
#endif
	}
	
	/* Linear PWM circuit setting */
	RegWriteA_lc898122_sharp( CVA		, 0xC0 );					// 0x0020	Linear PWM mode enable

	if( UcCvrCod_lc898122_sharp == CVER122 ) {
		RegWriteA_lc898122_sharp( CVFC 	, 0x22 );				// 0x0021	
	}

	RegWriteA_lc898122_sharp( CVFC2 	, 0x80 );					// 0x0022
	if( UcCvrCod_lc898122_sharp == CVER122 ) {
		RegWriteA_lc898122_sharp( CVSMTHX	, 0x00 );				// 0x0023	smooth off
		RegWriteA_lc898122_sharp( CVSMTHY	, 0x00 );				// 0x0024	smooth off
	}

	RegReadA_lc898122_sharp( STBB0 	, &UcStbb0 );				// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	UcStbb0 &= 0x80 ;
	RegWriteA_lc898122_sharp( STBB0, UcStbb0 ) ;					// 0x0250	OIS standby
	
}



//********************************************************************************
// Function Name 	: IniGyr_lc898122_sharp
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Gyro Filter Setting Initialize Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
#ifdef GAIN_CONT
  #define	TRI_LEVEL		0x3A03126F		/* 0.0005 */
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


void	IniGyr_lc898122_sharp( void )
{
#ifdef	NEW_PTST
	UnFltVal		UnGyrLmt ;
#endif
	
	/*Gyro Filter Setting*/
	RegWriteA_lc898122_sharp( WG_EQSW	, 0x03 );						// 0x0110		[ - | Sw6 | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
	
	/*Gyro Filter Down Sampling*/
	
	RegWriteA_lc898122_sharp( WG_SHTON	, 0x10 );						// 0x0107		[ - | - | - | CmSht2PanOff ][ - | - | CmShtOpe(1:0) ]
														//				CmShtOpe[1:0] 00: シャッターOFF, 01: シャッターON, 1x:外部制御
										
#ifdef	DEF_SET
	RegWriteA_lc898122_sharp( WG_SHTDLYTMR , 0x00 );					// 0x0117	 	Shutter Delay
	RegWriteA_lc898122_sharp( WG_GADSMP, 	  0x00 );					// 0x011C		Sampling timing
	RegWriteA_lc898122_sharp( WG_HCHR, 	  0x00 );					// 0x011B		H-filter limitter control not USE
	RegWriteA_lc898122_sharp( WG_LMT3MOD , 0x00 );						// 0x0118 	[ - | - | - | - ][ - | - | - | CmLmt3Mod ]
														//				CmLmt3Mod	0: 通常リミッター動作, 1: 円の半径リミッター動作
	RegWriteA_lc898122_sharp( WG_VREFADD , 0x12 );						// 0x0119	 	センター戻しを行う遅延RAMのアドレス下位6ビット　(default 0x12 = GXH1Z2/GYH1Z2)
#endif
	RegWriteA_lc898122_sharp( WG_SHTMOD , 0x06 );						// 0x0116	 	Shutter Hold mode

	// Limiter
	RamWrite32A_lc898122_sharp( gxlmt1H, GYRLMT1H ) ;					// 0x1028
	RamWrite32A_lc898122_sharp( gylmt1H, GYRLMT1H ) ;					// 0x1128

	RamWrite32A_lc898122_sharp( gxlmt3HS0, GYRLMT3_S1 ) ;				// 0x1029
	RamWrite32A_lc898122_sharp( gylmt3HS0, GYRLMT3_S1 ) ;				// 0x1129
	
	RamWrite32A_lc898122_sharp( gxlmt3HS1, GYRLMT3_S2 ) ;				// 0x102A
	RamWrite32A_lc898122_sharp( gylmt3HS1, GYRLMT3_S2 ) ;				// 0x112A

	RamWrite32A_lc898122_sharp( gylmt4HS0, GYRLMT4_S1 ) ;				// 0x112B	Y軸Limiter4 High閾値0
	RamWrite32A_lc898122_sharp( gxlmt4HS0, GYRLMT4_S1 ) ;				// 0x102B	X軸Limiter4 High閾値0
	
	RamWrite32A_lc898122_sharp( gxlmt4HS1, GYRLMT4_S2 ) ;				// 0x102C	X軸Limiter4 High閾値1
	RamWrite32A_lc898122_sharp( gylmt4HS1, GYRLMT4_S2 ) ;				// 0x112C	Y軸Limiter4 High閾値1

	
	/* Pan/Tilt parameter */
	RegWriteA_lc898122_sharp( WG_PANADDA, 		0x12 );					// 0x0130	GXH1Z2/GYH1Z2 Select
	RegWriteA_lc898122_sharp( WG_PANADDB, 		0x09 );					// 0x0131	GXIZ/GYIZ Select
	
	 //Threshold
	RamWrite32A_lc898122_sharp( SttxHis, 	0x00000000 );				// 0x1226
	RamWrite32A_lc898122_sharp( SttxaL, 	0x00000000 );				// 0x109D
	RamWrite32A_lc898122_sharp( SttxbL, 	0x00000000 );				// 0x109E
	RamWrite32A_lc898122_sharp( Sttx12aM, 	GYRA12_MID );				// 0x104F
	RamWrite32A_lc898122_sharp( Sttx12aH, 	GYRA12_HGH );				// 0x105F
	RamWrite32A_lc898122_sharp( Sttx12bM, 	GYRB12_MID );				// 0x106F
	RamWrite32A_lc898122_sharp( Sttx12bH, 	GYRB12_HGH );				// 0x107F
	RamWrite32A_lc898122_sharp( Sttx34aM, 	GYRA34_MID );				// 0x108F
	RamWrite32A_lc898122_sharp( Sttx34aH, 	GYRA34_HGH );				// 0x109F
	RamWrite32A_lc898122_sharp( Sttx34bM, 	GYRB34_MID );				// 0x10AF
	RamWrite32A_lc898122_sharp( Sttx34bH, 	GYRB34_HGH );				// 0x10BF
	RamWrite32A_lc898122_sharp( SttyaL, 	0x00000000 );				// 0x119D
	RamWrite32A_lc898122_sharp( SttybL, 	0x00000000 );				// 0x119E
	RamWrite32A_lc898122_sharp( Stty12aM, 	GYRA12_MID );				// 0x114F
	RamWrite32A_lc898122_sharp( Stty12aH, 	GYRA12_HGH );				// 0x115F
	RamWrite32A_lc898122_sharp( Stty12bM, 	GYRB12_MID );				// 0x116F
	RamWrite32A_lc898122_sharp( Stty12bH, 	GYRB12_HGH );				// 0x117F
	RamWrite32A_lc898122_sharp( Stty34aM, 	GYRA34_MID );				// 0x118F
	RamWrite32A_lc898122_sharp( Stty34aH, 	GYRA34_HGH );				// 0x119F
	RamWrite32A_lc898122_sharp( Stty34bM, 	GYRB34_MID );				// 0x11AF
	RamWrite32A_lc898122_sharp( Stty34bH, 	GYRB34_HGH );				// 0x11BF
	
	// Pan level
	RegWriteA_lc898122_sharp( WG_PANLEVABS, 		0x00 );				// 0x0133
	
	// Average parameter are set IniAdj_lc898122_sharp

	// Phase Transition Setting
	// State 2 -> 1
	RegWriteA_lc898122_sharp( WG_PANSTT21JUG0, 	0x00 );				// 0x0140
	RegWriteA_lc898122_sharp( WG_PANSTT21JUG1, 	0x00 );				// 0x0141
	// State 3 -> 1
	RegWriteA_lc898122_sharp( WG_PANSTT31JUG0, 	0x00 );				// 0x0142
	RegWriteA_lc898122_sharp( WG_PANSTT31JUG1, 	0x00 );				// 0x0143
	// State 4 -> 1
	RegWriteA_lc898122_sharp( WG_PANSTT41JUG0, 	0x01 );				// 0x0144
	RegWriteA_lc898122_sharp( WG_PANSTT41JUG1, 	0x00 );				// 0x0145
	// State 1 -> 2
	RegWriteA_lc898122_sharp( WG_PANSTT12JUG0, 	0x00 );				// 0x0146
	RegWriteA_lc898122_sharp( WG_PANSTT12JUG1, 	0x07 );				// 0x0147
	// State 1 -> 3
	RegWriteA_lc898122_sharp( WG_PANSTT13JUG0, 	0x00 );				// 0x0148
	RegWriteA_lc898122_sharp( WG_PANSTT13JUG1, 	0x00 );				// 0x0149
	// State 2 -> 3
	RegWriteA_lc898122_sharp( WG_PANSTT23JUG0, 	0x11 );				// 0x014A
	RegWriteA_lc898122_sharp( WG_PANSTT23JUG1, 	0x00 );				// 0x014B
	// State 4 -> 3
	RegWriteA_lc898122_sharp( WG_PANSTT43JUG0, 	0x00 );				// 0x014C
	RegWriteA_lc898122_sharp( WG_PANSTT43JUG1, 	0x00 );				// 0x014D
	// State 3 -> 4
	RegWriteA_lc898122_sharp( WG_PANSTT34JUG0, 	0x01 );				// 0x014E
	RegWriteA_lc898122_sharp( WG_PANSTT34JUG1, 	0x00 );				// 0x014F
	// State 2 -> 4
	RegWriteA_lc898122_sharp( WG_PANSTT24JUG0, 	0x00 );				// 0x0150
	RegWriteA_lc898122_sharp( WG_PANSTT24JUG1, 	0x00 );				// 0x0151
	// State 4 -> 2
	RegWriteA_lc898122_sharp( WG_PANSTT42JUG0, 	0x44 );		        	// 0x0152
	RegWriteA_lc898122_sharp( WG_PANSTT42JUG1, 	0x04 );		 		// 0x0153

	// State Timer
	RegWriteA_lc898122_sharp( WG_PANSTT1LEVTMR, 	0x00 );				// 0x015B
	RegWriteA_lc898122_sharp( WG_PANSTT2LEVTMR, 	0x00 );				// 0x015C
	RegWriteA_lc898122_sharp( WG_PANSTT3LEVTMR, 	0x00 );				// 0x015D
	RegWriteA_lc898122_sharp( WG_PANSTT4LEVTMR, 	0x03 );				// 0x015E
	
	// Control filter
	RegWriteA_lc898122_sharp( WG_PANTRSON0, 		0x11 );				// 0x0132	USE I12/iSTP/Gain-Filter
	
	// State Setting
	IniPtMovMod_lc898122_sharp( OFF ) ;								// Pan/Tilt setting (Still)
	
	// Hold
	RegWriteA_lc898122_sharp( WG_PANSTTSETILHLD,	0x00 );				// 0x015F
	
	
	// State2,4 Step Time Setting
	RegWriteA_lc898122_sharp( WG_PANSTT2TMR0,	0x01 );					// 0x013C
	RegWriteA_lc898122_sharp( WG_PANSTT2TMR1,	0x00 );					// 0x013D	
	RegWriteA_lc898122_sharp( WG_PANSTT4TMR0,	0x02 );					// 0x013E
	RegWriteA_lc898122_sharp( WG_PANSTT4TMR1,	0x07 );					// 0x013F	
	
	RegWriteA_lc898122_sharp( WG_PANSTTXXXTH,	0x00 );					// 0x015A

#ifdef	NEW_PTST
	UnGyrLmt.SfFltVal	= 0.003F ;						// St4 Limiter　1/S↓
	RamWrite32A_lc898122_sharp( npxlev8, UnGyrLmt.UlLngVal ) ;			// 0x109B
	RamWrite32A_lc898122_sharp( npylev8, UnGyrLmt.UlLngVal ) ;			// 0x119B


	// Fast
//	UnGyrLmt.SfFltVal	= 0.0076F ;						// St1
	UnGyrLmt.SfFltVal	= 0.02F ;						// St1
	RamWrite32A_lc898122_sharp( npxlev1, UnGyrLmt.UlLngVal ) ;			// 0x100F
	RamWrite32A_lc898122_sharp( npylev1, UnGyrLmt.UlLngVal ) ;			// 0x110F
	RamWrite32A_lc898122_sharp( npxlev1_i, UnGyrLmt.UlLngVal ) ;		// 0x10CF
	RamWrite32A_lc898122_sharp( npylev1_i, UnGyrLmt.UlLngVal ) ;		// 0x11CF
	UnGyrLmt.SfFltVal	= 0.0005F ;					// St2 Limiter
	RamWrite32A_lc898122_sharp( npxlev2, UnGyrLmt.UlLngVal ) ;			// 0x101F
	RamWrite32A_lc898122_sharp( npylev2, UnGyrLmt.UlLngVal ) ;			// 0x111F
	RamWrite32A_lc898122_sharp( npxlev2_i, UnGyrLmt.UlLngVal ) ;		// 0x10DF
	RamWrite32A_lc898122_sharp( npylev2_i, UnGyrLmt.UlLngVal ) ;		// 0x11DF

	//Slow
	UnGyrLmt.SfFltVal	= 0.0005F ;  					// St3元
//	UnGyrLmt.SfFltVal	= 0.0050F ;  					// St3元
	RamWrite32A_lc898122_sharp( npxlev3, UnGyrLmt.UlLngVal ) ;			// 0x102F
	RamWrite32A_lc898122_sharp( npylev3, UnGyrLmt.UlLngVal ) ;			// 0x112F
//	UnGyrLmt.SfFltVal	= 0.001F ;  					// St3
	UnGyrLmt.SfFltVal	= 0.0025F ;  					// St3元
	RamWrite32A_lc898122_sharp( npxlev3_i, UnGyrLmt.UlLngVal ) ;		// 0x10EF
	RamWrite32A_lc898122_sharp( npylev3_i, UnGyrLmt.UlLngVal ) ;		// 0x11EF
//	UnGyrLmt.SfFltVal	= 0.0009F ;						// St4 Limiter
	UnGyrLmt.SfFltVal	= 0.0005f ;						// St4 Limiter
	RamWrite32A_lc898122_sharp( npxlev4, UnGyrLmt.UlLngVal ) ;			// 0x103F
	RamWrite32A_lc898122_sharp( npylev4, UnGyrLmt.UlLngVal ) ;			// 0x113F
//	UnGyrLmt.SfFltVal	= 0.0009F ;						// St4 Limiter(initial)
	UnGyrLmt.SfFltVal	= 0.0005F ;						// St4 Limiter
	RamWrite32A_lc898122_sharp( npxlev4_i, UnGyrLmt.UlLngVal ) ;		// 0x10FF
	RamWrite32A_lc898122_sharp( npylev4_i, UnGyrLmt.UlLngVal ) ;		// 0x11FF


//	RegWriteA_lc898122_sharp( WG_VREFADD , 0x12 );						// 0x0119

	// Pan/Tilt NEW　PanTilt　Setteing
	RegWriteA_lc898122_sharp( WG_NPANST12BTMR, 0x0F ) ;				// 0x0167 
	RegWriteA_lc898122_sharp( WG_NPANST3RTMR, 0x0A ) ;					// 0x0166 
	RegWriteA_lc898122_sharp( WG_NPANST12TMRX, 0x00 ) ;				// 0x0168 682u:682μ
	RegWriteA_lc898122_sharp( WG_NPANST12TMRY, 0x00 ) ;				// 0x0169 682u:682μ
	RegWriteA_lc898122_sharp( WG_NPANST3TMRX, 0x08 ) ;					// 0x016A 98ms
	RegWriteA_lc898122_sharp( WG_NPANST3TMRY, 0x08 ) ;					// 0x016B 98ms
	RegWriteA_lc898122_sharp( WG_NPANST4TMRX, 0x02 ) ;					// 0x016C 21ms
	RegWriteA_lc898122_sharp( WG_NPANST4TMRY, 0x02 ) ;					// 0x016D 21ms

	RegWriteA_lc898122_sharp( WG_NPANSTFRC, 0x00 ) ;					// 0x010B State posituon ON
	RegWriteA_lc898122_sharp( WG_NPANFUN, 0x01 ) ;						// 0x016E Gain Cut,decrease
	RegWriteA_lc898122_sharp( WG_NPANINITMR, 0x02 ) ;					// 0x016F 21ms '評価時に検討現在は同じ値
	RegWriteA_lc898122_sharp( WG_NPANSTOFF, 0xA0 ) ;					// 0x010E State5,7 OFF

	RegWriteA_lc898122_sharp( WG_NPANTST0 ,0x08 );						// 0x0164 Option Setting

	UnGyrLmt.UlLngVal = 0x3951B717 ;					// St1 		decrease
	RamWrite32A_lc898122_sharp( gxistp_1u, UnGyrLmt.UlLngVal ) ;		// 0x1085
	RamWrite32A_lc898122_sharp( gyistp_1u, UnGyrLmt.UlLngVal ) ;		// 0x1185
	UnGyrLmt.UlLngVal = 0x3F7FFE80 ;
	RamWrite32A_lc898122_sharp( gxistp_2d, UnGyrLmt.UlLngVal );		// 0x1087	ST1,ST3　Ccof Val
	RamWrite32A_lc898122_sharp( gyistp_2d, UnGyrLmt.UlLngVal );		// 0x1187	ST1,ST3　Ccof Val
	UnGyrLmt.UlLngVal = 0x3F7FFE80 ;
	RamWrite32A_lc898122_sharp( gyistp_2u, UnGyrLmt.UlLngVal );		// 0x1188	ST2,ST4　Ccof Val
	RamWrite32A_lc898122_sharp( gxistp_2u, UnGyrLmt.UlLngVal );		// 0x1088	ST2,ST4　Ccof Val
	
#endif

#ifdef GAIN_CONT
	RamWrite32A_lc898122_sharp( gxlevlow, TRI_LEVEL );					// 0x10AE	Low Th
	RamWrite32A_lc898122_sharp( gylevlow, TRI_LEVEL );					// 0x11AE	Low Th
	RamWrite32A_lc898122_sharp( gxadjmin, XMINGAIN );					// 0x1094	Low gain
	RamWrite32A_lc898122_sharp( gxadjmax, XMAXGAIN );					// 0x1095	Hgh gain
	RamWrite32A_lc898122_sharp( gxadjdn, XSTEPDN );					// 0x1096	-step
	RamWrite32A_lc898122_sharp( gxadjup, XSTEPUP );					// 0x1097	+step
	RamWrite32A_lc898122_sharp( gyadjmin, YMINGAIN );					// 0x1194	Low gain
	RamWrite32A_lc898122_sharp( gyadjmax, YMAXGAIN );					// 0x1195	Hgh gain
	RamWrite32A_lc898122_sharp( gyadjdn, YSTEPDN );					// 0x1196	-step
	RamWrite32A_lc898122_sharp( gyadjup, YSTEPUP );					// 0x1197	+step
	
	RegWriteA_lc898122_sharp( WG_LEVADD, (unsigned char)MONADR );		// 0x0120	Input signal
	RegWriteA_lc898122_sharp( WG_LEVTMR, 		TIMEBSE );				// 0x0123	Base Time
	RegWriteA_lc898122_sharp( WG_LEVTMRLOW, 	TIMELOW );				// 0x0121	X Low Time
	RegWriteA_lc898122_sharp( WG_LEVTMRHGH, 	TIMEHGH );				// 0x0122	X Hgh Time
	RegWriteA_lc898122_sharp( WG_ADJGANADD, (unsigned char)GANADR );	// 0x0128	control address
	RegWriteA_lc898122_sharp( WG_ADJGANGO, 		0x00 );				// 0x0108	manual off

	/* exe function */
	//	AutoGainControlSw( OFF ) ;							/* Auto Gain Control Mode OFF */
	AutoGainControlSw_lc898122_sharp( ON ) ;
#endif
	
}


//********************************************************************************
// Function Name 	: IniFil_lc898122_sharp
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Gyro Filter Initial Parameter Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniFil_lc898122_sharp( void )
{
 	unsigned short	UsAryId ;

	// Filter Registor Parameter Setting
	UsAryId	= 0 ;
	while( CsFilReg_lc898122_sharp[ UsAryId ].UsRegAdd != 0xFFFF )
	{
		RegWriteA_lc898122_sharp( CsFilReg_lc898122_sharp[ UsAryId ].UsRegAdd, CsFilReg_lc898122_sharp[ UsAryId ].UcRegDat ) ;
		UsAryId++ ;
	}

	// Filter Ram Parameter Setting
	UsAryId	= 0 ;
	while( CsFilRam_lc898122_sharp[ UsAryId ].UsRamAdd != 0xFFFF )
	{
		RamWrite32A_lc898122_sharp( CsFilRam_lc898122_sharp[ UsAryId ].UsRamAdd, CsFilRam_lc898122_sharp[ UsAryId ].UlRamDat ) ;
		UsAryId++ ;
	}
	
}



//********************************************************************************
// Function Name 	: IniAdj_lc898122_sharp
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Adjust Value Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniAdj_lc898122_sharp( void )
{
	RegWriteA_lc898122_sharp( WC_RAMACCXY, 0x00 ) ;			// 0x018D	Filter copy off

	IniPtAve_lc898122_sharp( ) ;								// Average setting
	
	/* OIS */
	RegWriteA_lc898122_sharp( CMSDAC0, BIAS_CUR_OIS ) ;		// 0x0251	Hall Dac電流
	RegWriteA_lc898122_sharp( OPGSEL0, AMP_GAIN_X ) ;			// 0x0253	Hall amp Gain X
	RegWriteA_lc898122_sharp( OPGSEL1, AMP_GAIN_Y ) ;			// 0x0254	Hall amp Gain Y
	/* AF */
	RegWriteA_lc898122_sharp( CMSDAC1, BIAS_CUR_AF ) ;			// 0x0252	Hall Dac電流
	RegWriteA_lc898122_sharp( OPGSEL2, AMP_GAIN_AF ) ;			// 0x0255	Hall amp Gain AF

	RegWriteA_lc898122_sharp( OSCSET, OSC_INI ) ;				// 0x0257	OSC ini
	
	/* adjusted value */
	RegWriteA_lc898122_sharp( IZAH,	DGYRO_OFST_XH ) ;		// 0x02A0		Set Offset High byte
	RegWriteA_lc898122_sharp( IZAL,	DGYRO_OFST_XL ) ;		// 0x02A1		Set Offset Low byte
	RegWriteA_lc898122_sharp( IZBH,	DGYRO_OFST_YH ) ;		// 0x02A2		Set Offset High byte
	RegWriteA_lc898122_sharp( IZBL,	DGYRO_OFST_YL ) ;		// 0x02A3		Set Offset Low byte
	
	/* Ram Access */
	RamAccFixMod_lc898122_sharp( ON ) ;						// 16bit Fix mode
	
	/* OIS adjusted parameter */
	RamWriteA_lc898122_sharp( DAXHLO,		DAHLXO_INI ) ;		// 0x1479
	RamWriteA_lc898122_sharp( DAXHLB,		DAHLXB_INI ) ;		// 0x147A
	RamWriteA_lc898122_sharp( DAYHLO,		DAHLYO_INI ) ;		// 0x14F9
	RamWriteA_lc898122_sharp( DAYHLB,		DAHLYB_INI ) ;		// 0x14FA
	RamWriteA_lc898122_sharp( OFF0Z,		HXOFF0Z_INI ) ;		// 0x1450
	RamWriteA_lc898122_sharp( OFF1Z,		HYOFF1Z_INI ) ;		// 0x14D0
	RamWriteA_lc898122_sharp( sxg,			SXGAIN_INI ) ;		// 0x10D3
	RamWriteA_lc898122_sharp( syg,			SYGAIN_INI ) ;		// 0x11D3
//	UsCntXof_lc898122_sharp = OPTCEN_X ;						/* Clear Optical center X value */
//	UsCntYof_lc898122_sharp = OPTCEN_Y ;						/* Clear Optical center Y value */
//	RamWriteA_lc898122_sharp( SXOFFZ1,		UsCntXof_lc898122_sharp ) ;		// 0x1461
//	RamWriteA_lc898122_sharp( SYOFFZ1,		UsCntYof_lc898122_sharp ) ;		// 0x14E1

	/* AF adjusted parameter */
	RamWriteA_lc898122_sharp( DAZHLO,		DAHLZO_INI ) ;		// 0x1529
	RamWriteA_lc898122_sharp( DAZHLB,		DAHLZB_INI ) ;		// 0x152A

	/* Ram Access */
	RamAccFixMod_lc898122_sharp( OFF ) ;						// 32bit Float mode
	
	RamWrite32A_lc898122_sharp( gxzoom, GXGAIN_INI ) ;			// 0x1020 Gyro X axis Gain adjusted value
	RamWrite32A_lc898122_sharp( gyzoom, GYGAIN_INI ) ;			// 0x1120 Gyro Y axis Gain adjusted value

	RamWrite32A_lc898122_sharp( sxq, SXQ_INI ) ;				// 0x10E5	X axis output direction initial value
	RamWrite32A_lc898122_sharp( syq, SYQ_INI ) ;				// 0x11E5	Y axis output direction initial value
	
	if( GXHY_GYHX ){			/* GX -> HY , GY -> HX */
		RamWrite32A_lc898122_sharp( sxgx, 0x00000000 ) ;		// 0x10B8
		RamWrite32A_lc898122_sharp( sxgy, 0x3F800000 ) ;		// 0x10B9
		
		RamWrite32A_lc898122_sharp( sygy, 0x00000000 ) ;		// 0x11B8
		RamWrite32A_lc898122_sharp( sygx, 0x3F800000 ) ;		// 0x11B9
	}
	
	SetZsp_lc898122_sharp(0) ;									// Zoom coefficient Initial Setting
	
	RegWriteA_lc898122_sharp( PWMA 	, 0xC0 );				// 0x0010		PWM enable

	RegWriteA_lc898122_sharp( STBB0 	, 0xDF );				// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	RegWriteA_lc898122_sharp( WC_EQSW	, 0x02 ) ;				// 0x01E0
	RegWriteA_lc898122_sharp( WC_MESLOOP1	, 0x02 ) ;			// 0x0193
	RegWriteA_lc898122_sharp( WC_MESLOOP0	, 0x00 ) ;			// 0x0192
	RegWriteA_lc898122_sharp( WC_AMJLOOP1	, 0x02 ) ;			// 0x01A3
	RegWriteA_lc898122_sharp( WC_AMJLOOP0	, 0x00 ) ;			// 0x01A2
	
	
	SetPanTiltMode_lc898122_sharp( OFF ) ;					/* Pan/Tilt OFF */
	
	SetGcf_lc898122_sharp( 0 ) ;							/* DI initial value */
#ifdef H1COEF_CHANGER
	SetH1cMod_lc898122_sharp( ACTMODE ) ;					/* Lvl Change Active mode */
#endif
	
	DrvSw_lc898122_sharp( ON ) ;							/* 0x0001		Driver Mode setting */
	
	RegWriteA_lc898122_sharp( WC_EQON, 0x01 ) ;				// 0x0101	Filter ON
}



//********************************************************************************
// Function Name 	: IniCmd_lc898122_sharp
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Command Execute Process Initial
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniCmd_lc898122_sharp( void )
{

	MemClr_lc898122_sharp( ( unsigned char * )&StAdjPar_lc898122_sharp, sizeof( stAdjPar ) ) ;	// Adjust Parameter Clear
	
}


//********************************************************************************
// Function Name 	: BsyWit_lc898122_sharp
// Retun Value		: NON
// Argment Value	: Trigger Register Address, Trigger Register Data
// Explanation		: Busy Wait Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
void	BsyWit_lc898122_sharp( unsigned short	UsTrgAdr, unsigned char	UcTrgDat )
{
	unsigned char	UcFlgVal ;

	RegWriteA_lc898122_sharp( UsTrgAdr, UcTrgDat ) ;	// Trigger Register Setting

	UcFlgVal	= 1 ;

	while( UcFlgVal ) {

		RegReadA_lc898122_sharp( UsTrgAdr, &UcFlgVal ) ;
		UcFlgVal	&= 	( UcTrgDat & 0x0F ) ;
	} ;

}


//********************************************************************************
// Function Name 	: MemClr_lc898122_sharp
// Retun Value		: void
// Argment Value	: Clear Target Pointer, Clear Byte Number
// Explanation		: Memory Clear Function
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	MemClr_lc898122_sharp( unsigned char	*NcTgtPtr, unsigned short	UsClrSiz )
{
	unsigned short	UsClrIdx ;

	for ( UsClrIdx = 0 ; UsClrIdx < UsClrSiz ; UsClrIdx++ )
	{
		*NcTgtPtr	= 0 ;
		NcTgtPtr++ ;
	}
}



//********************************************************************************
// Function Name 	: WitTim_lc898122_sharp
// Retun Value		: NON
// Argment Value	: Wait Time(ms)
// Explanation		: Timer Wait Function
// History			: First edition 						2009.07.31 Y.Tashita
//********************************************************************************
void	WitTim_lc898122_sharp( unsigned short	UsWitTim_lc898122_sharp )
{
	unsigned long	UlLopIdx, UlWitCyc ;

	UlWitCyc	= ( unsigned long )( ( float )UsWitTim_lc898122_sharp / NOP_TIME / ( float )12 ) ;

	for( UlLopIdx = 0 ; UlLopIdx < UlWitCyc ; UlLopIdx++ )
	{
		;
	}
}

//********************************************************************************
// Function Name 	: GyOutSignal_lc898122_sharp
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Select Gyro Signal Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	GyOutSignal_lc898122_sharp( void )
{

	RegWriteA_lc898122_sharp( GRADR0,	GYROX_INI ) ;			// 0x0283	Set Gyro XOUT H~L
	RegWriteA_lc898122_sharp( GRADR1,	GYROY_INI ) ;			// 0x0284	Set Gyro YOUT H~L
	
	/*Start OIS Reading*/
	RegWriteA_lc898122_sharp( GRSEL	, 0x02 );				// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

}

//********************************************************************************
// Function Name 	: GyOutSignalCont_lc898122_sharp
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Select Gyro Continuosl Function
// History			: First edition 						2013.06.06 Y.Shigeoka
//********************************************************************************
void	GyOutSignalCont_lc898122_sharp( void )
{

	/*Start OIS Reading*/
	RegWriteA_lc898122_sharp( GRSEL	, 0x04 );				// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

}

#ifdef STANDBY_MODE
//********************************************************************************
// Function Name 	: AccWit_lc898122_sharp
// Retun Value		: NON
// Argment Value	: Trigger Register Data
// Explanation		: Acc Wait Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	AccWit_lc898122_sharp( unsigned char UcTrgDat )
{
	unsigned char	UcFlgVal ;

	UcFlgVal	= 1 ;

	while( UcFlgVal ) {
		RegReadA_lc898122_sharp( GRACC, &UcFlgVal ) ;			// 0x0282
		UcFlgVal	&= UcTrgDat ;
	} ;
}

//********************************************************************************
// Function Name 	: SelectGySleep_lc898122_sharp
// Retun Value		: NON
// Argment Value	: mode	
// Explanation		: Select Gyro mode Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	SelectGySleep_lc898122_sharp( unsigned char UcSelMode )
{
 #ifdef USE_INVENSENSE
	unsigned char	UcRamIni ;
	unsigned char	UcGrini ;

	if(UcSelMode == ON)
	{
		RegWriteA_lc898122_sharp( WC_EQON, 0x00 ) ;		// 0x0101	Equalizer OFF
		RegWriteA_lc898122_sharp( GRSEL,	0x01 ) ;		/* 0x0280	Set Command Mode			*/

		RegReadA_lc898122_sharp( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
		RegWriteA_lc898122_sharp( GRINI, ( UcGrini | SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
		
		RegWriteA_lc898122_sharp( GRADR0,	0x6B ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_lc898122_sharp( GRACC,	0x01 ) ;		/* 0x0282	Set Read Trigger ON				*/
		AccWit_lc898122_sharp( 0x01 ) ;					/* Digital Gyro busy wait 				*/
		RegReadA_lc898122_sharp( GRDAT0H, &UcRamIni ) ;	/* 0x0290 */
		
		UcRamIni |= 0x40 ;					/* Set Sleep bit */
  #ifdef GYROSTBY
		UcRamIni &= ~0x01 ;					/* Clear PLL bit(internal oscillator */
  #endif
		
		RegWriteA_lc898122_sharp( GRADR0,	0x6B ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_lc898122_sharp( GSETDT,	UcRamIni ) ;	/* 0x028A	Set Write Data(Sleep ON)	*/
		RegWriteA_lc898122_sharp( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_lc898122_sharp( 0x10 ) ;					/* Digital Gyro busy wait 				*/

  #ifdef GYROSTBY
		RegWriteA_lc898122_sharp( GRADR0,	0x6C ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_lc898122_sharp( GSETDT,	0x07 ) ;		/* 0x028A	Set Write Data(STBY ON)	*/
		RegWriteA_lc898122_sharp( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_lc898122_sharp( 0x10 ) ;					/* Digital Gyro busy wait 				*/
  #endif
	}
	else
	{
  #ifdef GYROSTBY
		RegWriteA_lc898122_sharp( GRADR0,	0x6C ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_lc898122_sharp( GSETDT,	0x00 ) ;		/* 0x028A	Set Write Data(STBY OFF)	*/
		RegWriteA_lc898122_sharp( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_lc898122_sharp( 0x10 ) ;					/* Digital Gyro busy wait 				*/
  #endif
		RegWriteA_lc898122_sharp( GRADR0,	0x6B ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_lc898122_sharp( GRACC,	0x01 ) ;		/* 0x0282	Set Read Trigger ON				*/
		AccWit_lc898122_sharp( 0x01 ) ;					/* Digital Gyro busy wait 				*/
		RegReadA_lc898122_sharp( GRDAT0H, &UcRamIni ) ;	/* 0x0290 */
		
		UcRamIni &= ~0x40 ;					/* Clear Sleep bit */
  #ifdef GYROSTBY
		UcRamIni |=  0x01 ;					/* Set PLL bit */
  #endif
		
		RegWriteA_lc898122_sharp( GSETDT,	UcRamIni ) ;	// 0x028A	Set Write Data(Sleep OFF)
		RegWriteA_lc898122_sharp( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_lc898122_sharp( 0x10 ) ;					/* Digital Gyro busy wait 				*/
		
		RegReadA_lc898122_sharp( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
		RegWriteA_lc898122_sharp( GRINI, ( UcGrini & ~SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
		
		GyOutSignal_lc898122_sharp( ) ;					/* Select Gyro output signal 			*/
		
		WitTim_lc898122_sharp( 50 ) ;						// 50ms wait
		
		RegWriteA_lc898122_sharp( WC_EQON, 0x01 ) ;		// 0x0101	GYRO Equalizer ON

		ClrGyr_lc898122_sharp( 0x007F , CLR_FRAM1 );		// Gyro Delay RAM Clear
	}
 #else									/* Panasonic */
	
//	unsigned char	UcRamIni ;


	if(UcSelMode == ON)
	{
		RegWriteA_lc898122_sharp( WC_EQON, 0x00 ) ;		// 0x0101	GYRO Equalizer OFF
		RegWriteA_lc898122_sharp( GRSEL,	0x01 ) ;		/* 0x0280	Set Command Mode			*/
		RegWriteA_lc898122_sharp( GRADR0,	0x4C ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_lc898122_sharp( GSETDT,	0x02 ) ;		/* 0x028A	Set Write Data(Sleep ON)	*/
		RegWriteA_lc898122_sharp( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_lc898122_sharp( 0x10 ) ;					/* Digital Gyro busy wait 				*/
	}
	else
	{
		RegWriteA_lc898122_sharp( GRADR0,	0x4C ) ;		// 0x0283	Set Write Command
		RegWriteA_lc898122_sharp( GSETDT,	0x00 ) ;		// 0x028A	Set Write Data(Sleep OFF)
		RegWriteA_lc898122_sharp( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_lc898122_sharp( 0x10 ) ;					/* Digital Gyro busy wait 				*/
		GyOutSignal_lc898122_sharp( ) ;					/* Select Gyro output signal 			*/
		
		WitTim_lc898122_sharp( 50 ) ;						// 50ms wait
		
		RegWriteA_lc898122_sharp( WC_EQON, 0x01 ) ;		// 0x0101	GYRO Equalizer ON
		ClrGyr_lc898122_sharp( 0x007F , CLR_FRAM1 );		// Gyro Delay RAM Clear
	}
 #endif
}
#endif

#ifdef	GAIN_CONT
//********************************************************************************
// Function Name 	: AutoGainControlSw_lc898122_sharp
// Retun Value		: NON
// Argment Value	: 0 :OFF  1:ON
// Explanation		: Select Gyro Signal Function
// History			: First edition 						2010.11.30 Y.Shigeoka
//********************************************************************************
void	AutoGainControlSw_lc898122_sharp( unsigned char UcModeSw )
{

	if( UcModeSw == OFF )
	{
		RegWriteA_lc898122_sharp( WG_ADJGANGXATO, 	0xA0 );					// 0x0129	X exe off
		RegWriteA_lc898122_sharp( WG_ADJGANGYATO, 	0xA0 );					// 0x012A	Y exe off
		RamWrite32A_lc898122_sharp( GANADR			 , XMAXGAIN ) ;			// Gain Through
		RamWrite32A_lc898122_sharp( GANADR | 0x0100 , YMAXGAIN ) ;			// Gain Through
	}
	else
	{
		RegWriteA_lc898122_sharp( WG_ADJGANGXATO, 	0xA3 );					// 0x0129	X exe on
		RegWriteA_lc898122_sharp( WG_ADJGANGYATO, 	0xA3 );					// 0x012A	Y exe on
	}

}
#endif


//********************************************************************************
// Function Name 	: ClrGyr_lc898122_sharp
// Retun Value		: NON
// Argment Value	: UsClrFil - Select filter to clear.  If 0x0000, clears entire filter
//					  UcClrMod - 0x01: FRAM0 Clear, 0x02: FRAM1, 0x03: All RAM Clear
// Explanation		: Gyro RAM clear function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
void	ClrGyr_lc898122_sharp( unsigned short UsClrFil , unsigned char UcClrMod )
{
	unsigned char	UcRamClr;

	/*Select Filter to clear*/
	RegWriteA_lc898122_sharp( WC_RAMDLYMOD1,	(unsigned char)(UsClrFil >> 8) ) ;		// 0x018F		FRAM Initialize Hbyte
	RegWriteA_lc898122_sharp( WC_RAMDLYMOD0,	(unsigned char)UsClrFil ) ;				// 0x018E		FRAM Initialize Lbyte

	/*Enable Clear*/
	RegWriteA_lc898122_sharp( WC_RAMINITON	, UcClrMod ) ;	// 0x0102	[ - | - | - | - ][ - | - | 遅延Clr | 係数Clr ]
	
	/*Check RAM Clear complete*/
	do{
		RegReadA_lc898122_sharp( WC_RAMINITON, &UcRamClr );
		UcRamClr &= UcClrMod;
	}while( UcRamClr != 0x00 );
}


//********************************************************************************
// Function Name 	: DrvSw_lc898122_sharp
// Retun Value		: NON
// Argment Value	: 0:OFF  1:ON
// Explanation		: Driver Mode setting function
// History			: First edition 						2012.04.25 Y.Shigeoka
//********************************************************************************
void	DrvSw_lc898122_sharp( unsigned char UcDrvSw_lc898122_sharp )
{
	if( UcDrvSw_lc898122_sharp == ON )
	{
		if( UcPwmMod_lc898122_sharp == PWMMOD_CVL ) {
			RegWriteA_lc898122_sharp( DRVFC	, 0xF0 );			// 0x0001	Drv.MODE=1,Drv.BLK=1,MODE2,LCEN
		} else {
#ifdef	PWM_BREAK
			RegWriteA_lc898122_sharp( DRVFC	, 0x00 );			// 0x0001	Drv.MODE=0,Drv.BLK=0,MODE0B
#else
			RegWriteA_lc898122_sharp( DRVFC	, 0xC0 );			// 0x0001	Drv.MODE=1,Drv.BLK=1,MODE1
#endif
		}
	}
	else
	{
		if( UcPwmMod_lc898122_sharp == PWMMOD_CVL ) {
			RegWriteA_lc898122_sharp( DRVFC	, 0x30 );				// 0x0001	Drvier Block Ena=0
		} else {
#ifdef	PWM_BREAK
			RegWriteA_lc898122_sharp( DRVFC	, 0x00 );				// 0x0001	Drv.MODE=0,Drv.BLK=0,MODE0B
#else
			RegWriteA_lc898122_sharp( DRVFC	, 0x00 );				// 0x0001	Drvier Block Ena=0
#endif
		}
	}
}

//********************************************************************************
// Function Name 	: AfDrvSw_lc898122_sharp
// Retun Value		: NON
// Argment Value	: 0:OFF  1:ON
// Explanation		: AF Driver Mode setting function
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
void	AfDrvSw_lc898122_sharp( unsigned char UcDrvSw_lc898122_sharp )
{
	if( UcDrvSw_lc898122_sharp == ON )
	{
	#ifdef	AF_MID_MOUNT
		RegWriteA_lc898122_sharp( DRVFCAF,		0x10 ) ;			// DRVFCAF(0x0081)
	#else	//AF_MID_MOUNT
	  #ifdef	AF_PWMMODE
		RegWriteA_lc898122_sharp( DRVFCAF	, 0x00 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
	  #else
		RegWriteA_lc898122_sharp( DRVFCAF	, 0x20 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-2
	  #endif
	#endif	//AF_MID_MOUNT
		RegWriteA_lc898122_sharp( CCAAF,   0x80 ) ;				// 0x00A0	[7]=0:OFF 1:ON
	}
	else
	{
		RegWriteA_lc898122_sharp( CCAAF,   0x00 ) ;				// 0x00A0	[7]=0:OFF 1:ON
	}
}

//********************************************************************************
// Function Name 	: RamAccFixMod_lc898122_sharp
// Retun Value		: NON
// Argment Value	: 0:OFF  1:ON
// Explanation		: Ram Access Fix Mode setting function
// History			: First edition 						2013.05.21 Y.Shigeoka
//********************************************************************************
void	RamAccFixMod_lc898122_sharp( unsigned char UcAccMod )
{
	switch ( UcAccMod ) {
		case OFF :
			RegWriteA_lc898122_sharp( WC_RAMACCMOD,	0x00 ) ;		// 0x018C		GRAM Access Float32bit
			break ;
		case ON :
			RegWriteA_lc898122_sharp( WC_RAMACCMOD,	0x31 ) ;		// 0x018C		GRAM Access Fix32bit
			break ;
	}
}
	

//********************************************************************************
// Function Name 	: IniAf_lc898122_sharp
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Open AF Initial Setting
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
void	IniAf_lc898122_sharp( void )
{
	unsigned char	UcStbb0 ;
	
	AfDrvSw_lc898122_sharp( OFF ) ;								/* AF Drvier Block Ena=0 */

#ifdef	AF_MID_MOUNT
	//RegWriteA( TCODEH,		0x04 ) ;			// TCODEH(0304h)
	//RamWriteA( TREG_H,		0x0000 ) ;			// TREG_H(0x0380) - TREG_L(0x0381)
	RamWriteA_lc898122_sharp( TCODEH,		0x0200 ) ;			// 0x0304 - 0x0305 (Register continuos write)
	RegWriteA_lc898122_sharp( DRVFCAF,		0x10 ) ;			// DRVFCAF(0x0081)
	RegWriteA_lc898122_sharp( DRVFC3AF,	0x40 ) ;			// DRVFC3AF(0x0083)
	RegWriteA_lc898122_sharp( DRVFC4AF,	0x80 ) ;			// DRVFC4AF(0x0084)
	RegWriteA_lc898122_sharp( AFFC,		0x90 ) ;			// AFFC(0x0088)
	//RegWriteA_lc898122_sharp( CCAAF,		0x80 ) ;			// CCAAF(0x00A0)
#else	//AF_MID_MOUNT
  #ifdef	AF_PWMMODE
	RegWriteA_lc898122_sharp( DRVFCAF	, 0x00 );					// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
  #else
	RegWriteA_lc898122_sharp( DRVFCAF	, 0x20 );					// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-2
  #endif
	RegWriteA_lc898122_sharp( DRVFC3AF	, 0x00 );					// 0x0083	DGAINDAF	Gain 0
	RegWriteA_lc898122_sharp( DRVFC4AF	, 0x80 );					// 0x0084	DOFSTDAF
	RegWriteA_lc898122_sharp( PWMAAF,    0x00 ) ;					// 0x0090	AF PWM standby
	RegWriteA_lc898122_sharp( AFFC,   0x80 ) ;						// 0x0088	OpenAF/-/-
#endif	//AF_MID_MOUNT

#ifdef	AF_PWMMODE
	RegWriteA_lc898122_sharp( DRVFC2AF,    0x82 ) ;				// 0x0082	AF slope3
	RegWriteA_lc898122_sharp( DRVCH3SEL,   0x02 ) ;				// 0x0085	AF only IN1 control
	RegWriteA_lc898122_sharp( PWMFCAF,     0x89 ) ;				// 0x0091	AF GND , Carrier , MODE1 
	RegWriteA_lc898122_sharp( PWMPERIODAF, 0xA0 ) ;				// 0x0099	AF none-synchronism
#else
	RegWriteA_lc898122_sharp( DRVFC2AF,    0x00 ) ;				// 0x0082	AF slope0
	RegWriteA_lc898122_sharp( DRVCH3SEL,   0x00 ) ;				// 0x0085	AF H bridge control
	RegWriteA_lc898122_sharp( PWMFCAF,     0x01 ) ;				// 0x0091	AF VREF , Carrier , MODE1
	RegWriteA_lc898122_sharp( PWMPERIODAF, 0x20 ) ;				// 0x0099	AF none-synchronism
#endif
	
#ifdef	AF_MID_MOUNT
	RegWriteA_lc898122_sharp( CCFCAF,	 0x08 ) ;					// CCFCAF(0x00A1)
#else	//AF_MID_MOUNT
	RegWriteA_lc898122_sharp( CCFCAF,   0x40 ) ;					// 0x00A1	GND/-
#endif	//AF_MID_MOUNT
	
	RegReadA_lc898122_sharp( STBB0 	, &UcStbb0 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	UcStbb0 &= 0x7F ;
	RegWriteA_lc898122_sharp( STBB0, UcStbb0 ) ;			// 0x0250	OIS standby
	RegWriteA_lc898122_sharp( STBB1, 0x00 ) ;				// 0x0264	All standby
	
	/* AF Initial setting */
	RegWriteA_lc898122_sharp( FSTMODE,		FSTMODE_AF ) ;		// 0x0302
	RamWriteA_lc898122_sharp( RWEXD1_L,	RWEXD1_L_AF ) ;		// 0x0396 - 0x0397 (Register continuos write)
	RamWriteA_lc898122_sharp( RWEXD2_L,	RWEXD2_L_AF ) ;		// 0x0398 - 0x0399 (Register continuos write)
	RamWriteA_lc898122_sharp( RWEXD3_L,	RWEXD3_L_AF ) ;		// 0x039A - 0x039B (Register continuos write)
	RegWriteA_lc898122_sharp( FSTCTIME,	FSTCTIME_AF ) ;		// 0x0303 	
#ifdef	AF_MID_MOUNT
	//SetTregAf( 0x0400 );						// 
	RamWriteA_lc898122_sharp( TCODEH,		0x0600 ) ;			// 0x0304 - 0x0305 (Register continuos write) fast mode on
#else	//AF_MID_MOUNT
	RamWriteA_lc898122_sharp( TCODEH,		0x0000 ) ;			// 0x0304 - 0x0305 (Register continuos write)
#endif	//AF_MID_MOUNT
	
#ifdef	AF_PWMMODE
	RegWriteA_lc898122_sharp( PWMAAF,    0x80 ) ;			// 0x0090	AF PWM enable
#endif

	UcStbb0 |= 0x80 ;
	RegWriteA_lc898122_sharp( STBB0, UcStbb0 ) ;			// 0x0250	
	RegWriteA_lc898122_sharp( STBB1	, 0x05 ) ;			// 0x0264	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]

	AfDrvSw_lc898122_sharp( ON ) ;								/* AF Drvier Block Ena=1 */
}

#ifdef	AF_MID_MOUNT
//********************************************************************************
// Function Name 	: SetTregAf_lc898122_sharp
// Retun Value		: 
// Argment Value	: Min:000h Max:7FFh
// Explanation		: 
// History			: First edition 						2014.06.19 T.Tokoro
//********************************************************************************
void	SetTregAf_lc898122_sharp( unsigned short UsTregAf )
{
	RamWriteA_lc898122_sharp( TREG_H,	(UsTregAf << 5) ) ;		// TREG_H(0x0380) - TREG_L(0x0381)
												// TREG[15:5] 11bit
												// AF_D[10:0]=TREG[15:5](0380h/0381h)。
												// Min:000h Max:7FFh
}
#endif	//AF_MID_MOUNT

//********************************************************************************
// Function Name 	: IniPtAve_lc898122_sharp
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Pan/Tilt Average parameter setting function
// History			: First edition 						2013.09.26 Y.Shigeoka
//********************************************************************************
void	IniPtAve_lc898122_sharp( void )
{
	RegWriteA_lc898122_sharp( WG_PANSTT1DWNSMP0, 0x00 );		// 0x0134
	RegWriteA_lc898122_sharp( WG_PANSTT1DWNSMP1, 0x00 );		// 0x0135
	RegWriteA_lc898122_sharp( WG_PANSTT2DWNSMP0, 0x90 );		// 0x0136 400
	RegWriteA_lc898122_sharp( WG_PANSTT2DWNSMP1, 0x01 );		// 0x0137
	RegWriteA_lc898122_sharp( WG_PANSTT3DWNSMP0, 0x64 );		// 0x0138 100
	RegWriteA_lc898122_sharp( WG_PANSTT3DWNSMP1, 0x00 );		// 0x0139
	RegWriteA_lc898122_sharp( WG_PANSTT4DWNSMP0, 0x00 );		// 0x013A
	RegWriteA_lc898122_sharp( WG_PANSTT4DWNSMP1, 0x00 );		// 0x013B

	RamWrite32A_lc898122_sharp( st1mean, 0x3f800000 );		// 0x1235
	RamWrite32A_lc898122_sharp( st2mean, 0x3B23D700 );		// 0x1236	1/400
	RamWrite32A_lc898122_sharp( st3mean, 0x3C23D700 );		// 0x1237	1/100
	RamWrite32A_lc898122_sharp( st4mean, 0x3f800000 );		// 0x1238
			
}
	
//********************************************************************************
// Function Name 	: IniPtMovMod_lc898122_sharp
// Retun Value		: NON
// Argment Value	: OFF:Still  ON:Movie
// Explanation		: Pan/Tilt parameter setting by mode function
// History			: First edition 						2013.09.26 Y.Shigeoka
//********************************************************************************
void	IniPtMovMod_lc898122_sharp( unsigned char UcPtMod )
{
	switch ( UcPtMod ) {
		case OFF :
			RegWriteA_lc898122_sharp( WG_PANSTTSETGYRO, 	0x00 );		// 0x0154
			RegWriteA_lc898122_sharp( WG_PANSTTSETGAIN, 	0x54 );		// 0x0155
			RegWriteA_lc898122_sharp( WG_PANSTTSETISTP, 	0x14 );		// 0x0156
			RegWriteA_lc898122_sharp( WG_PANSTTSETIFTR,	0x94 );		// 0x0157
			RegWriteA_lc898122_sharp( WG_PANSTTSETLFTR,	0x00 );		// 0x0158

			break ;
		case ON :
			RegWriteA_lc898122_sharp( WG_PANSTTSETGYRO, 	0x00 );		// 0x0154
			RegWriteA_lc898122_sharp( WG_PANSTTSETGAIN, 	0x00 );		// 0x0155
			RegWriteA_lc898122_sharp( WG_PANSTTSETISTP, 	0x14 );		// 0x0156
			RegWriteA_lc898122_sharp( WG_PANSTTSETIFTR,	0x94 );		// 0x0157
			RegWriteA_lc898122_sharp( WG_PANSTTSETLFTR,	0x00 );		// 0x0158
			break ;
	}
}

//********************************************************************************
// Function Name 	: ChkCvr_lc898122_sharp
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Check Cver function
// History			: First edition 						2013.10.03 Y.Shigeoka
//********************************************************************************
void	ChkCvr_lc898122_sharp( void )
{
	RegReadA_lc898122_sharp( CVER ,	&UcCvrCod_lc898122_sharp );		// 0x027E
	RegWriteA_lc898122_sharp( VRREG ,	(unsigned char)FW_VER );		// 0x2D0	Version
}

