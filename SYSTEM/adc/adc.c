/*==================================================================================
This document is the property of Nanjing Kangni Mechanical & Electrical Co.,Ltd.. 
It may not be reproduced or disclosed to third party without prior to authorisation.
Module name: adc.c
Description: adc module,used to configure adc parameters. get the motor current.
Function List: 
( ID             Procedure name     Version)
 [sunit-0201]    CnfgrADC1           V0100
 [sunit-0202]    CnfgrADC2           V0100
 [sunit-0203]    RawCrrntFilter      V0100
 [sunit-0204]    ADCInit             V0100
 [sunit-0205]    GetRefCrrnt         V0100
 [sunit-0206]    UpdateCurrent       V0100
Target: STM32F103VET6
Status: TESTED
History: Wan Lei, V0100-0000, 20180104
===================================================================================*/

/*==================================================================================
   list of header files
===================================================================================*/
#include "project.h"


#define ADC1_DR_Address    ((u32)0x40012400+0x4c)//((u32)0x4001244C)
uint16_t ADC_ConvertedValue[5];
/*==================================================================================
Procedure description: configure adc1 function
Parameter description：1. use GPIOC4 to be the left motor current sample channel  
                       2. use timer 1 CCR4 to be the interrupt trigger
use in: main.c
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180104
History: none
===================================================================================*/
void CnfgrADC1(void)
{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    DMA_InitTypeDef DMA_InitStructure;
//    ADC_InitTypeDef ADC_InitStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;
//	
//    RCC_APB2PeriphClockCmd(ADC_MTR1_RCC | RCC_APB2Periph_ADC1, ENABLE );	
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//    GPIO_Init(ADC_MTR1, &GPIO_InitStructure);

// 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//	
// 	DMA_DeInit(DMA1_Channel1);
// 	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	 //ADC地址
// 	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;//内存地址
// 	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
// 	DMA_InitStructure.DMA_BufferSize = 5;
// 	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址固定
// 	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址固定
// 	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//半字
// 	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
// 	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//循环传输
// 	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
// 	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
// 	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
//	
////		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1 ;
////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
////	NVIC_Init(&NVIC_InitStructure);	
////		//DMA_Init(DMA1_Channel1, &DMA_InitStructure);
////	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
//	
//	//ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular); 
//	//* Enable DMA channel1 *//
// 	DMA_Cmd(DMA1_Channel1, ENABLE);
//	
//    ADC_DeInit(ADC1);
//    /* ADC1 configuration ------------------------------------------------------*/
//    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	   /* 独立模式 */
//    ADC_InitStructure.ADC_ScanConvMode = ENABLE;			   /* 连续多通道模式 */
//    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	   /* 连续转换 */
//    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;   //ADC_ExternalTrigInjecConv_T1_TRGO;  /* 转换不受外界决定 */
//    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;		       /* 右对齐 */
//    ADC_InitStructure.ADC_NbrOfChannel = 5;					   /* 扫描通道数 */
//    ADC_Init(ADC1, &ADC_InitStructure);
//    ADC_DMACmd(ADC1,ENABLE);
//    RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
//	/*配置ADC1的通道11为55.	5个采样周期，序列为1 */ 
// 	  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
// 	  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
//   	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5);
// 	  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5);
// 	  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_239Cycles5);


//   
//    ADC_Cmd(ADC1, ENABLE);                 	/* Enable ADC1 */  
//    /* Enable ADC1 reset calibaration register */   
//    ADC_ResetCalibration(ADC1);

//    /* Check the end of ADC1 reset calibration register */
//    while(ADC_GetResetCalibrationStatus(ADC1));
//    /* Start ADC1 calibaration */
//    ADC_StartCalibration(ADC1);
//    /* Check the end of ADC1 calibration */
//    while(ADC_GetCalibrationStatus(ADC1));                      
////    ADC_SoftwareStartConvCmd(ADC1,ENABLE);    /* 使能转换开始 */
////    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
////    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
////    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
////    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
////    NVIC_Init(&NVIC_InitStructure);
////	
////    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
////    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
//    ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}


/*==================================================================================
Procedure description: the rule of filtering the current
Parameter description：average the value of the first 15 adc values
use in: ADC1/2 interrupt
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180104
History: none
===================================================================================*/
void RawCrrntFilter(ADCGEN *adc)
{
	if(adc->uFAdcInit == 1)
	{
        adc->uFAdcInit = 0;
	}
    adc->wMAdcSum[0] = 0;
	  adc->wMAdcSum[1] = 0;
	  adc->wMAdcSum[2] = 0;
	  adc->wMAdcSum[3] = 0;
	  adc->wMAdcSum[4] = 0;
    adc->wPAdcTbl[0][adc->uMAdcCntNum] = adc->wMotorAdcCrrnt[0];
	  adc->wPAdcTbl[1][adc->uMAdcCntNum] = adc->wMotorAdcCrrnt[1];
	  adc->wPAdcTbl[2][adc->uMAdcCntNum] = adc->wMotorAdcCrrnt[2];
	  adc->wPAdcTbl[3][adc->uMAdcCntNum] = adc->wMotorAdcCrrnt[3];
	  adc->wPAdcTbl[4][adc->uMAdcCntNum] = adc->wMotorAdcCrrnt[4];
	  adc->uMAdcCntNum++;
    if(adc->uMAdcCntNum == 16)
    {
        adc->uMAdcCntNum = 0;
    }
    for(adc->uMAdcCnt = 0; adc->uMAdcCnt < 16; adc->uMAdcCnt++)
    {
        adc->wMAdcSum[0] = adc->wMAdcSum[0] + adc->wPAdcTbl[0][adc->uMAdcCnt];
			  adc->wMAdcSum[1] = adc->wMAdcSum[1] + adc->wPAdcTbl[1][adc->uMAdcCnt];
			  adc->wMAdcSum[2] = adc->wMAdcSum[2] + adc->wPAdcTbl[2][adc->uMAdcCnt];
			  adc->wMAdcSum[3] = adc->wMAdcSum[3] + adc->wPAdcTbl[3][adc->uMAdcCnt];
			  adc->wMAdcSum[4] = adc->wMAdcSum[4] + adc->wPAdcTbl[4][adc->uMAdcCnt];
    }
    adc->wMAdcCrrntFltr[0] = adc->wMAdcSum[0] / 16;
		adc->wMAdcCrrntFltr[1] = adc->wMAdcSum[1] / 16;
		adc->wMAdcCrrntFltr[2] = adc->wMAdcSum[2] / 16;
		adc->wMAdcCrrntFltr[3] = adc->wMAdcSum[3] / 16;
		adc->wMAdcCrrntFltr[4] = adc->wMAdcSum[4] / 16;
//    adc->wMAdcSum[3] = adc->wMAdcSum[3]+ADC_ConvertedValue[3];
//		adc->wMAdcSum[4] = adc->wMAdcSum[4]+ADC_ConvertedValue[4];
//		adc->wMAdcSum[2] = adc->wMAdcSum[2]+ADC_ConvertedValue[2];
//		adc->wMAdcSum[1] = adc->wMAdcSum[1]+ADC_ConvertedValue[1];
//		adc->wMAdcSum[0] = adc->wMAdcSum[0]+ADC_ConvertedValue[0];
//    adc->uMAdcCnt++;
//		if(adc->uMAdcCnt == 8)
//		{
//		    adc->wMAdcCrrntFltr[3] = adc->wMAdcSum[3] /8;
//			  adc->wMAdcSum[3] = 0;
//			
//		    adc->wMAdcCrrntFltr[4] = adc->wMAdcSum[4] /8;
//			  adc->wMAdcSum[4] = 0;
//			
//		    adc->wMAdcCrrntFltr[2] = adc->wMAdcSum[2] /8;
//			  adc->wMAdcSum[2] = 0;
//			
//		    adc->wMAdcCrrntFltr[1] = adc->wMAdcSum[1] /8;
//			  adc->wMAdcSum[1] = 0;
//			
//		    adc->wMAdcCrrntFltr[0] = adc->wMAdcSum[0] /8;
//			  adc->wMAdcSum[0] = 0;
//			
//		    adc->uMAdcCnt = 0;
//		}
}


void GetMotorADCVal(ADCGEN *adc)
{
    adc->wMotorAdcCrrnt[0] = ADC_ConvertedValue[0]; 
	  adc->wMotorAdcCrrnt[1] = ADC_ConvertedValue[1];
    adc->wMotorAdcCrrnt[2] = ADC_ConvertedValue[2]; 
    adc->wMotorAdcCrrnt[3] = ADC_ConvertedValue[3]; 
    adc->wMotorAdcCrrnt[4] = ADC_ConvertedValue[4]; 
    RawCrrntFilter(adc);

}



void DMA1_Channel1_IRQHandler()
{
//	if(DMA_GetITStatus(DMA1_IT_TC1) == SET)
//	{
//    //GetMotorADCVal(&adc1);
//		DMA_ClearITPendingBit(DMA1_IT_TC1);
//	}
}


/*==================================================================================
Procedure description: initializing the parameters about the current filtering
Parameter description：
use in: main.c
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 201803016
History: none
===================================================================================*/
//void ADCInit(MCGEN *mc, ADCGEN *adc)
//{
//  if((adc->uFAdcInit == 0) && (mc->uFStop == 1))
//	{
//    for(adc->uMAdcCnt = 0; adc->uMAdcCnt < 16; adc->uMAdcCnt++)
//    {
//		adc->wPAdcTbl[adc->uMAdcCnt] = 0;
//    }
//		adc->uFAdcInit = 1;
//	}
//}

/*==================================================================================
Procedure description: get the reference current
Parameter description：get the maximum current value when close the door
use in: ADC1/2 interrupt
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180104
History: none
===================================================================================*/
void GetRefCrrnt(ADCGEN *adc)
{
//    if(adc->wMAdcCrrntFltr > adc->wMAdcMaxCrrnt)
//    {
//        adc->wMAdcMaxCrrnt = adc->wMAdcCrrntFltr;
//    }
}

/*==================================================================================
Procedure description: update the motor current
Parameter description: put the filtering current in the motor current 
use in: timer 7 interrupt
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180104
History: none
===================================================================================*/
//void UpdateCurrent(MCGEN *mc, ADCGEN *adc)
//{
////	if(mc->uFStop == 0)
////	{
////        mc->wRMtrCurrent = adc->wMAdcCrrntFltr;
////	}
////	else
////	{
////        mc->wRMtrCurrent = 0;
////	}
////    mc->wRRefCurrent = adc->wMAdcMaxCrrnt;
//}

/*==================================================================================
     the end of file
===================================================================================*/
