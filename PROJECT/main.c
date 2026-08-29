/*==================================================================================
This document is the property of Nanjing Kangni Mechanical & Electrical Co., Ltd. It may
not be reproduced or disclosed to third party without prior to authorisation
---------------The home of this file-----------------
File Name: main.c
File Description:
File Originator: This file created by...
Function List:
                   void TIM6_IRQHandler(void)
                   void TIM7_IRQHandler(void)
                   void ADC1_2_IRQHandler(void)
                   void ADC3_IRQHandler(void)

History: V0100-0000

-version information: Wan Lei, V0100-0000, 201803015
===================================================================================*/

/*==================================================================================
   list of header files
===================================================================================*/
#define PROJECT_DEBUG 1U

#include "project.h"
#include "debug_log.h"

const uint8_t g_project_debug_enabled = PROJECT_DEBUG;

//static IPCGEN       ipc = IPC_DEFAULTS;
 TMRGEN       tmr = TIMR_DEFAULTS;
//ADCGEN              adc1 = ADC_DEFAULTS;
//static ADCGEN       adc2 = ADC_DEFAULTS;
//DOORLGEN     drl = DOORL_DEFAULTS;
//static DGNSGEN      dgns = DGNS_DEFAULTS;
//MCGEN               lmc = MC1_DEFAULTS;
//MCGEN               rmc = MC2_DEFAULTS;
//PIGEN ASR1,ACR1;
//ERRORGEN            err = ERROR_DEFAULTS;
LinterMotor         lim = LINTER_DEFAULTS;
//static  CCGEN       cc = CC_DEFAULTS;
int testI,testJ=0;
u8 ErrorState[]={0xFE,0x01};
//extern u8 RS485_RX_BUFF[30];
u8 NRF2401_RX_BUFF[4];
extern uint16_t ADC_ConvertedValue[5];
//#define ALARM_POLL_INTERVAL_MS  1000   // 1秒轮询一次

void SysTickInit(void)
{
	SysTick_Config(SystemCoreClock / 1000);	//Set SysTick Timer for 1ms interrupts  
}

u8 a =8;
u8 b =0;

/**
 * @brief 堵转后的非阻塞恢复短任务。
 *
 * 首次触发时停止当前序列，随后依次推进 Battery_22、Battery_15 和
 * LeaveCenter1；每个动作返回 PENDING 时立即让出主循环。
 */
static void StallRecovery_Task(void)
{
    static uint8_t recovery_step = 0U;
    uint8_t result;

    if (!MotorMonitor_StallTriggered() && recovery_step == 0U) {
        return;
    }
    if (recovery_step == 0U) {
        LOG_WARN("RECOVERY", "stall detected; stopping current sequence\r\n");
        Sequence_Stop();
        recovery_step = 1U;
    }

    if (recovery_step == 1U) {
        result = Battery_22();
    } else if (recovery_step == 2U) {
        result = Battery_15();
    } else {
        result = LeaveCenter1();
    }

    if (result == MODBUS_RESULT_PENDING || ModbusMaster_IsBusy() || ModbusBatch_IsBusy()) {
        return;
    }
    if (result == MODBUS_RESULT_OK) {
        LOG_INFO("RECOVERY", "step %u completed\r\n",
                 (unsigned int)recovery_step);
    } else {
        LOG_ERROR("RECOVERY", "step %u failed: result=%u\r\n",
                  (unsigned int)recovery_step, (unsigned int)result);
    }
    recovery_step++;
    if (recovery_step > 3U) {
        recovery_step = 0U;
        MotorMonitor_ClearStallTrigger();
        LOG_INFO("RECOVERY", "stall recovery finished\r\n");
    }
}
/*==================================================================================
Procedure description: main program entry
Parameter description：none
use in: none
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180305
History: none
===================================================================================*/
		
int main(void)
{
	uint8_t motor1_up_active = 1U;
	uint8_t motor1_up_result = MODBUS_RESULT_PENDING;

//	SysTickInit();
	SystemInit();
	Tick_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
//	Delayinit(168);    //初始化延时函数
	uart4_init(115200);//串口调试信息输出printf
	ModbusPort_InitMaster(MODBUS_PORT_DEFAULT_BAUDRATE);
	ModbusPort_InitSlave(MODBUS_PORT_DEFAULT_BAUDRATE);
	Relay_Init();
	OutputPortInit();//led灯测试
	ModbusPort_InitMasterFrameTimer(MODBUS_PORT_DEFAULT_BAUDRATE);
	ModbusPort_InitSlaveFrameTimer(MODBUS_PORT_DEFAULT_BAUDRATE);
	StatusRegs_Init();
	ModbusSlave_Init(GATEWAY_SERVICE_MODBUS_ADDRESS);
	GatewayService_Init();
	ModbusMaster_Init();
	Sequence_Init();
	bsp_InitHardTimer();
	W25QXX_Init();          // 初始化SPI Flash
	SwapState_Init();   // 从Flash读取空仓号
	LOG_INFO("MAIN", "initialization completed; entering main loop\r\n");
  __enable_irq();  /* 开启全局中断 */
	delay_ms(1000);
//	Motor_Reset(MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, 8);
	// 复位前确保主站状态空闲
	
	//GPIO_SetBits(LED_PORT, LED1);
	
//		while(W25QXX_ReadID()!=W25Q128)								//检测不到W25Q128
//	{
//		printf("W25Q128 Check Failed!");
//		delay_ms(500);
//		//LCD_ShowString(30,150,200,16,16,"Please Check!      ");
//		///delay_ms(500);
//		//LED0=!LED0;		//DS0闪烁
//	}
//	printf("W25Q128 Check OK");
	
//	W25QXX_Write(&a,0,1);
//	delay_ms(500);
//	
//	
//	W25QXX_Read(&b,0,1);	
//	delay_ms(500);
//	
//	printf("读取电机电流失败，错误码%d\n", b);
//	a=9;
//		W25QXX_Write(&a,0,1);
//	delay_ms(500);
//	
//	
//	W25QXX_Read(&b,0,1);	
//	delay_ms(500);
	

//	LeaveCenter();
// LeaveCenter2();
//Battery_2();
//Battery_6();
//Center_1();
//Center_2();
//Battery_21();
//Battery_4();
//Battery_10();
//OpenAC();
//CloseAC();
//Battery_21();
//FlyOpen();
//Battery_5();
//Battery_7();
//OpenDr();

//printf("\r\n============= MCU RESET DETECTED =============\r\n");
		while(1)
		{		
			ModbusMaster_Process();
			ModbusSlave_Process();
			if (motor1_up_active != 0U) {
				motor1_up_result = Motor1Up1();
				if (motor1_up_result == MODBUS_RESULT_OK) {
					motor1_up_active = 0U;
					LOG_INFO("MAIN", "Motor1Up1 completed\r\n");
				} else if (motor1_up_result != MODBUS_RESULT_PENDING &&
						   motor1_up_result != MODBUS_RESULT_BUSY) {
					motor1_up_active = 0U;
					LOG_ERROR("MAIN", "Motor1Up1 failed: result=%u\r\n",
							  (unsigned int)motor1_up_result);
				}
			}
			// Sequence_Process();   	// 处理序列（一键起飞/降落完成）
			// GatewayService_Process();
			// SwapState_TrySave();   // 延迟保存（Flash 写入过程仍为同步执行）
			// StallRecovery_Task();
			/* 后台轮询优先级最低，避免抢在控制命令之前占用主站总线。 */
			if (motor1_up_active == 0U) {
				MasterPolling_Task();
			}
		}
}

/*==================================================================================
Procedure description: timer 6 interrupt
Parameter description：read input port and timer count
use in: none
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180105
History: none
===================================================================================*/
//void TIM2_IRQHandler(void)   //TIM4中断
//{
//    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM6更新中断发生与否
//    {
//        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIM6更新中断标志 
//        //the input module read the signal every 1ms
////        ReadInputSignal(&ipc);
//        //the timer module updata
//        Update100msTimer(&tmr);
//        Update10msTimer(&tmr);
//        Update1msTimer(&tmr);
////			  testI++;
////			  if(testI == 3000)
////				{
////				    //testI = 0;
////					  drl.uFDoorstate = 0xAA;
////				}
////				else if(testI == 6000)
////				{
////				    drl.uFDoorstate = 0xBB;
////				}
////				else if(testI == 9000)
////				{
////				    drl.uFDoorstate = 0xCC;
////				}				
////				else if(testI == 12000)
////				{
////				    drl.uFDoorstate = 0xDD;
////				}
////				else if(testI == 15000)
////				{
////					  testI = 0;
////				    drl.uFDoorstate = 0x88;
////				}
////				else{}
//    }
//}

/*==================================================================================
Procedure description: timer 7 interrupt
Parameter description：read motor current and calculate the pwm
use in: none
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180105
History: none
===================================================================================*/
//void TIM4_IRQHandler(void)
//{
//    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //检查TIM7更新中断发生与否
//    {
//        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //清除TIM7更新中断标志 
//        UpdateCurrent(&lmc, &adc1);
//        CalcPWM(&lmc, &rmc, &tmr);
//    }	
//}

/*==================================================================================
Procedure description: adc1/2 interrupt
Parameter description：read adc current and maximum current
use in: none
Transfer procedure: none
Status: TESTED
Originator: Wan Lei, V0100-0000, 20180105
History: none
===================================================================================*/



/*==================================================================================
     the end of file
===================================================================================*/
