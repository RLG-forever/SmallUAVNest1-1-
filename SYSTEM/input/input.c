/*==================================================================================
This document is the property of Nanjing Kangni Mechanical & Electrical Co.,Ltd.. 
It may not be reproduced or disclosed to third party without prior to authorisation.
Module name: io.c
Description: input port module,used to read input signal.
Function List: 
(  ID            Procedure name     Version)
 [sunit-0101]    ReadInputSignal     V0100
 [sunit-0102]    RawSgnlFilter       V0100
 [sunit-0103]    InputPortFilter     V0100
 [sunit-0104]    UpdateInputSignal   V0100
 [sunit-0105]    CnfgrInputPort      V0100
Target: STM32F103VET6
Status: TESTED
History: Wan Lei, V0100-0000, 20180104
===================================================================================*/

/*==================================================================================
   list of header files
===================================================================================*/
#include "project.h"
/*==================================================================================
Procedure description: read input signal
Parameter description£ºread the inputport (00-06) and outputport feedback (00-03)
use in: timer 6 interrupt
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180104
History: none
===================================================================================*/
void ReadInputSignal(IPCGEN *ipc)
{

    ipc->uIInputPort01 = GPIO_ReadInputDataBit(IPORT01, IPORT01_PIN);
    ipc->uIInputPort02 = GPIO_ReadInputDataBit(IPORT02, IPORT02_PIN);
    ipc->uIInputPort03 = GPIO_ReadInputDataBit(IPORT03, IPORT03_PIN);
    ipc->uIInputPort04 = GPIO_ReadInputDataBit(IPORT04, IPORT04_PIN);
    ipc->uIInputPort05 = GPIO_ReadInputDataBit(IPORT05, IPORT05_PIN);
    ipc->uIInputPort06 = GPIO_ReadInputDataBit(IPORT06, IPORT06_PIN);
	
	#ifdef TEST
        ipc->uITest = GPIO_ReadInputDataBit(TEST, TEST_PIN);
    #endif
	

}

/*==================================================================================
Procedure description: signal filtering rule
Parameter description£ºwhen the signal change and maintain the specified time, give 
                       the filtering signal
use in: InputPortFilter
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180104
History: none
===================================================================================*/
void RawSgnlFilter(U8 *uMSgnlPre, U8 uMSgnlNow, U8 *uMSgnlFilter, S16 *iMTimer,\
                   U8 uMClockSetUp, U8 uMClockSetDn)
{
    if((*uMSgnlPre) != uMSgnlNow)
    {
        if(uMSgnlNow == 1)
        {
            *iMTimer = uMClockSetUp;
        }	
        else
        {
			*iMTimer = uMClockSetDn;
        }		
        *uMSgnlPre = uMSgnlNow;
    }
    else
    {
        if(*iMTimer == 0)
        {
			      *uMSgnlFilter = uMSgnlNow & 0x0001;  //only 1 bit, error control
			      *iMTimer = -1;
        }
    }
}

/*==================================================================================
Procedure description: input signal filtering
Parameter description£ºfilter the input signal, use 10ms base timer
use in: main.c
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180104
History: none
===================================================================================*/
void InputPortFilter(IPCGEN *ipc, TMRGEN *tmr)
{
	
    RawSgnlFilter(&ipc->uMIPreInputPort01, ipc->uIInputPort01, &ipc->uFInputPort01Fltr,\
        &tmr->TIMER_INPUT01_FLTR_T10MS, DFLT_II01_FLT_TIME_UP, DFLT_II01_FLT_TIME_DN);

//    RawSgnlFilter(&ipc->uMIPreInputPort02, ipc->uIInputPort02, &ipc->uFInputPort02Fltr,\
//        &tmr->TIMER_INPUT02_FLTR_T10MS, DFLT_II02_FLT_TIME_UP, DFLT_II02_FLT_TIME_DN);

//    RawSgnlFilter(&ipc->uMIPreInputPort03, ipc->uIInputPort03, &ipc->uFInputPort03Fltr,\
//        &tmr->TIMER_INPUT03_FLTR_T10MS, DFLT_II03_FLT_TIME_UP, DFLT_II03_FLT_TIME_DN);

//    RawSgnlFilter(&ipc->uMIPreInputPort04, ipc->uIInputPort04, &ipc->uFInputPort04Fltr,\
//        &tmr->TIMER_INPUT04_FLTR_T10MS, DFLT_II04_FLT_TIME_UP, DFLT_II04_FLT_TIME_DN);

//    RawSgnlFilter(&ipc->uMIPreInputPort05, ipc->uIInputPort05, &ipc->uFInputPort05Fltr,\
//        &tmr->TIMER_INPUT05_FLTR_T10MS, DFLT_II05_FLT_TIME_UP, DFLT_II05_FLT_TIME_DN);
//	
//    RawSgnlFilter(&ipc->uMIPreInputPort06, ipc->uIInputPort06, &ipc->uFInputPort06Fltr,\
//        &tmr->TIMER_INPUT06_FLTR_T10MS, DFLT_II06_FLT_TIME_UP, DFLT_II06_FLT_TIME_DN);
}

/*==================================================================================
Procedure description: updata input signal
Parameter description£ºupdata the switch and driver command
use in: main.c
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180104
History: none
===================================================================================*/
// void UpdateInputSignal(DOORLGEN *drl, IPCGEN *ipc)
//{
////    drl->uFLSwitch = LEFT_SWITCH_LOGIC_FLAG;
////    drl->uFESwitch = EMERGENCY_SWITCH_LOGIC_FLAG;
////    drl->uFOpnFltr = OPN_LINE_LOGIC_FLAG;
////    drl->uFClsFltr = CLS_LINE_LOGIC_FLAG;
////    drl->uFZeroSpdFltr = 1;//ZERO_SPD_LINE_LOGIC_FLAG;//ZERO_SPD_LINE_LOGIC_FLAG;
//	
//	
//	  drl->uFLOpenSwitch = LEFT_SWITCH_LOGIC_FLAG;  //I1
//    drl->uFLCloseSwitch = OPN_LINE_LOGIC_FLAG;  //I3
//		drl->uFROpenSwitch = EMERGENCY_SWITCH_LOGIC_FLAG;  //I2
//    drl->uFRCloseSwitch = CLS_LINE_LOGIC_FLAG; //I4
////    drl->uFOpnFltr = OPN_LINE_LOGIC_FLAG;
////    drl->uFClsFltr = CLS_LINE_LOGIC_FLAG;
//    drl->uFZeroSpdFltr = 1;//ZERO_SPD_LINE_LOGIC_FLAG;//ZERO_SPD_LINE_LOGIC_FLAG;
//	
//	
//	
//	//use to automatic program
//	if(drl->uFDoorInit == 1)
//	{
//        if((drl->uFOpnFltr == 1) && (drl->uFClsFltr == 1))
//        {
//            drl->uFAuto = 1;
//        }
//        else
//        {
//            drl->uFAuto = 0;
//        }
//	}

//	
//// 	  drl->uFZeroSpdFltr = ZERO_SPD_LINE_LOGIC_FLAG;
//}

/*==================================================================================
Procedure description: configure input port
Parameter description£ºnone
use in: main.c
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180104
History: none
===================================================================================*/
void CnfgrInputPort(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | 
                           RCC_AHB1Periph_GPIOC, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); 

    GPIO_InitStructure.GPIO_Pin = IPORT01_PIN;	 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;   
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(IPORT01, &GPIO_InitStructure);

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;	 
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    
//    GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	
//    GPIO_InitStructure.GPIO_Pin = IPORT02_PIN;	 
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    
//    GPIO_Init(IPORT02, &GPIO_InitStructure);

//    GPIO_InitStructure.GPIO_Pin = IPORT03_PIN;	 
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    
//    GPIO_Init(IPORT03, &GPIO_InitStructure);

//    GPIO_InitStructure.GPIO_Pin = IPORT04_PIN;	 
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    
//    GPIO_Init(IPORT04, &GPIO_InitStructure);


		

		
//		PWR_BackupAccessCmd(ENABLE );
//	  RCC_LSEConfig( RCC_LSE_OFF );
//    BKP_TamperPinCmd(DISABLE);
//		

//		PWR_BackupAccessCmd(DISABLE);




}

void OutputPortInit(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE);

		GPIO_InitStructure.GPIO_Pin = LED1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                           
		GPIO_Init(LED_PORT, &GPIO_InitStructure);
		GPIO_ResetBits(LED_PORT, LED1);
}

/*==================================================================================
     the end of file
===================================================================================*/

