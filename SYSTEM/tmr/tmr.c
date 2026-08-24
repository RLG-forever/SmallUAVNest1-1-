/*==================================================================================
This document is the property of Nanjing Kangni Mechanical & Electrical Co.,Ltd.. 
It may not be reproduced or disclosed to third party without prior to authorisation.
Module name: tmr.c
Description: timer module,used to configure timer 6 parameters. has 1ms/10ms/100ms.
Function List: 
(ID              Procedure name,    Version)
 [sunit-0401]    Update100msTimer    V0100
 [sunit-0402]    Update10msTimer     V0100
 [sunit-0403]    Update1msTimer      V0100
 [sunit-0404]    CnfgrTimer6         V0100
 [sunit-0405]    CnfgrTimer7         V0100
Target: STM32F103VET6
Status: TESTED
History: Wan Lei, V0100-0000, 20180104
===================================================================================*/

/*==================================================================================
   list of header files
===================================================================================*/
#include "project.h"

/*==================================================================================
Procedure description: 100ms timer
Parameter description: none
use in: timer 6 interrupt
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180104
History: none
===================================================================================*/
void Update100msTimer(TMRGEN *tmr)
{
    tmr->uM100msCntr++;
	  
    if(tmr->uM100msCntr >= 100)                //update every 100ms
    {
        tmr->uM100msCntr = 0;                  //clear timer counter
			  
        for(tmr->uM100msTimrCntrNum=0; tmr->uM100msTimrCntrNum<DFLT_100MS_TIMR_CNTR_NUM;\
		    tmr->uM100msTimrCntrNum++)
        {
            if(tmr->iT100msTimer[tmr->uM100msTimrCntrNum] > 0)
            {
				tmr->iT100msTimer[tmr->uM100msTimrCntrNum]--;
            }
        }
    }
}
					
/*==================================================================================
Procedure description: 10ms timer
Parameter description: none
use in: timer 6 interrupt
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180104
History: none
===================================================================================*/
void Update10msTimer(TMRGEN *tmr)
{
    tmr->uM10msCntr++;
	  
    if(tmr->uM10msCntr >= 10)                //update every 10ms
    {
        tmr->uM10msCntr = 0;                  //clear timer counter
			  
        for(tmr->uM10msTimrCntrNum=0; tmr->uM10msTimrCntrNum<DFLT_10MS_TIMR_CNTR_NUM;\
            tmr->uM10msTimrCntrNum++)
        {
            if(tmr->iT10msTimer[tmr->uM10msTimrCntrNum] > 0)
            {
                tmr->iT10msTimer[tmr->uM10msTimrCntrNum]--;
            }
        }
    }
}
		 
/*==================================================================================
Procedure description: 1ms timer
Parameter description: none
use in: timer 6 interrupt
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180104
History: none
===================================================================================*/
void Update1msTimer(TMRGEN *tmr)
{
    for(tmr->uM1msTimrCntrNum=0; tmr->uM1msTimrCntrNum<DFLT_1MS_TIMR_CNTR_NUM;\
        tmr->uM1msTimrCntrNum++)
    {
        if(tmr->iT1msTimer[tmr->uM1msTimrCntrNum] > 0)
        {
            tmr->iT1msTimer[tmr->uM1msTimrCntrNum]--;
        }
    }
}

/*==================================================================================
Procedure description: configure timer 6
Parameter description: 1ms interrupt
use in: main.c
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180104
History: none
===================================================================================*/
void CnfgrTimer2(void)                //999,71
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能
    TIM_TimeBaseStructure.TIM_Period = 999; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
    TIM_TimeBaseStructure.TIM_Prescaler = 83; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //TIM_CKD_DIV1设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位;
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //使能指定的TIM4中断,允许更新中断
    TIM_Cmd(TIM2, ENABLE);  //使能TIMx	

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM4中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
}

/*==================================================================================
Procedure description: configure timer 7
Parameter description：the frequency is 1KHz
use in: timer 7 interrupt
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180104
History: none
===================================================================================*/
void CnfgrTimer4()                //999,71
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能
    TIM_TimeBaseStructure.TIM_Period = 99; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
    TIM_TimeBaseStructure.TIM_Prescaler = 71; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //TIM_CKD_DIV1设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位;
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断
    TIM_Cmd(TIM4, ENABLE);  //使能TIMx	

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM7中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级2级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
}

/*==================================================================================
     the end of file
===================================================================================*/




