/*==================================================================================
This document is the property of Nanjing Kangni Mechanical & Electrical Co., Ltd..
It may not be reproduced or disclosed to third party without prior to authorisation
 * header file name: tmr.h      
 * heafer file description: define the date type and initalization used by the timer module 
 * service condition: use in timer module
 * version information: Wan Lei, V0100-0000, 20180104
===================================================================================*/

/*==================================================================================
     avoid redefinition
===================================================================================*/
#ifndef __TMR_H
#define	__TMR_H


/*==================================================================================
     constant definition
===================================================================================*/

//------------------------set timer counter-----------------------------------------
#define DFLT_100MS_TIMR_CNTR_NUM      17
#define DFLT_10MS_TIMR_CNTR_NUM      20
#define DFLT_1MS_TIMR_CNTR_NUM      5

/*==================================================================================
     100ms timer counter definition
===================================================================================*/
#define TIMER_1_100MS     iT100msTimer[0]
#define TIMER_2_100MS     iT100msTimer[1]
#define TIMER_3_100MS     iT100msTimer[2]
#define TIMER_4_100MS     iT100msTimer[3]
#define TIMER_5_100MS     iT100msTimer[4]
#define TIMER_6_100MS     iT100msTimer[5]
#define TIMER_7_100MS      iT100msTimer[6]
#define TIMER_8_100MS      iT100msTimer[7]
#define TIMER_MC1_OPNCCT_100MS        iT100msTimer[8]
#define TIMER_MC2_OPNCCT_100MS        iT100msTimer[9]
#define TIMER_MC1_NOLAOD_100MS        iT100msTimer[10]
#define TIMER_MC2_NOLAOD_100MS        iT100msTimer[11]
#define TIMER_MC1_HALLFAULT_100MS     iT100msTimer[12]
#define TIMER_CLSOBSTCNS_WAIT_T100MS     iT100msTimer[13]
#define TIMER_CLSOPN_OVERTIME_100MS           iT100msTimer[14]
#define TIMER_AUTORUN_100MS           iT100msTimer[15]
#define TIMER_LEDDELAY_100MS           iT100msTimer[16]
/*==================================================================================
     10ms timer counter definition
===================================================================================*/
#define TIMER_CLS_OBSTCNS_T10MS   iT10msTimer[0]
#define TIMER_INPUT01_FLTR_T10MS   iT10msTimer[1]
#define TIMER_INPUT02_FLTR_T10MS   iT10msTimer[2]
#define TIMER_INPUT03_FLTR_T10MS   iT10msTimer[3]
#define TIMER_INPUT04_FLTR_T10MS   iT10msTimer[4]
#define TIMER_INPUT05_FLTR_T10MS   iT10msTimer[5]
#define TIMER_INPUT06_FLTR_T10MS   iT10msTimer[6]

#define TIMER_CMD_DELAY_T10MS      iT10msTimer[7]
#define TIMER_OPN_OBSTC_T10MS      iT10msTimer[8]
#define TIMER_CLS_OBSTC_T10MS      iT10msTimer[9]

#define TIMER_CLS_OBSTC_OPNDLY_T10MS   iT10msTimer[10]
#define TIMER_CLS_OBSTC_OPNDLYNS_T10MS        iT10msTimer[11]
#define TIMER_OPNCLS_SWITCH_T10MS  iT10msTimer[12]
#define TIMER_SHOW_LED_T10MS       iT10msTimer[13]

#define TIMER_UPDATE_HALL_T10MS    iT10msTimer[14]

#define TIMER_MA_OBSTC_T10MS    iT10msTimer[15]
#define TIMER_MB_OBSTC_T10MS    iT10msTimer[16]
#define TIMER_MC_OBSTC_T10MS    iT10msTimer[17]
#define TIMER_MD_OBSTC_T10MS    iT10msTimer[18]
#define TIMER_ME_OBSTC_T10MS    iT10msTimer[19]

/*==================================================================================
     10ms timer counter definition
===================================================================================*/
#define TIMER_SPEED_LOOP_T1MS       iT1msTimer[0]

/*==================================================================================
     variable definition
===================================================================================*/
typedef struct
{
    U8 uM100msTimrCntrNum;
    U8 uM10msTimrCntrNum;
		U8 uM1msTimrCntrNum;
		U8 uM100msCntr;
		U8 uM10msCntr;
	
		S16 iT100msTimer[DFLT_100MS_TIMR_CNTR_NUM];
		S16 iT10msTimer[DFLT_10MS_TIMR_CNTR_NUM];
		S16 iT1msTimer[DFLT_1MS_TIMR_CNTR_NUM];
}TMRGEN;

/*==================================================================================
     variable initialization definition
===================================================================================*/
#define TIMR_DEFAULTS  {0,0,0,0,0,{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},\
                           {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},\
                           {-1,-1,-1,-1,-1}}
	
/*==================================================================================
     declare definition
===================================================================================*/
void Update100msTimer(TMRGEN *tmr);
void Update10msTimer(TMRGEN *tmr);
void Update1msTimer(TMRGEN *tmr);
void CnfgrTimer6(void);
void CnfgrTimer2(void);
void CnfgrTimer4(void);
	
#endif

/*==================================================================================
     the end of file
===================================================================================*/
	
	
	
	
	


