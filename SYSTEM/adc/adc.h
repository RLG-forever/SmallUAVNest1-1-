/*==================================================================================
This document is the property of Nanjing Kangni Mechanical & Electrical Co., Ltd.. 
It may not be reproduced or disclosed to third party without prior to authorisation.
 * header file name: adc.h      
 * heafer file description: define the date type and initalization used by the module 
 * service condition: use in adc module
 * version information: Wan Lei, V0100-0000, 20180104
===================================================================================*/

/*==================================================================================
     avoid redefinition
===================================================================================*/
#ifndef __ADC_H
#define	__ADC_H
/*==================================================================================
     variable definition
===================================================================================*/
typedef struct
{
	U8 uFAdcInit;
	U8 uMAdcCnt;
	U8 uMAdcCntNum;
    U16 wMAdcSum[5];
	U16 wMAdcCrrnt;
	U16 wMAdcCrrntFltr[5];
    U16 wMAdcMaxCrrnt;
	
    U16 wPAdcTbl[5][16];
 	U16 wMotorAdcCrrnt[5];
}ADCGEN;

/*==================================================================================
     variable initialization definition
===================================================================================*/
#define ADC_DEFAULTS  {0,0,0,{0},0,{0},0, {0},{0}}

/*==================================================================================
     declare definition
===================================================================================*/

// extern PI ACR;
// extern PI ASR;
void CnfgrADC1(void);
void CnfgrADC2(void);
void RawCrrntFilter(ADCGEN *adc);
//void ADCInit(MCGEN *mc, ADCGEN *adc);
void GetRefCrrnt(ADCGEN *adc);
//void UpdateCurrent(MCGEN *mc, ADCGEN *adc);
void GetMotorADCVal(ADCGEN *adc);


#endif
/******************* (C) COPYRIGHT 2017 Electric Control Team *****END OF FILE************/
