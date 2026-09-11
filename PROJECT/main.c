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
#include "exec_steps.h"

const uint8_t g_project_debug_enabled = PROJECT_DEBUG;

static const MotorMoveAbsPosParams stall_recovery_motor2_home =
    EXEC_ABS_POS(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS);
static const MotorMoveAbsPosParams stall_recovery_motor1_home =
    EXEC_ABS_POS(MOTOR1_SLAVE_ADDR, MOTOR_HOME_POS);
static const ExecMoveAbsPosParams EXEC_MOTOR2_HOME = {
    &stall_recovery_motor2_home, 1U
};
static const ExecMoveAbsPosParams EXEC_MOTOR1_HOME = {
    &stall_recovery_motor1_home, 1U
};

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
 * 首次触发时停止当前序列，随后依次推进电机2回零、电机1回零和
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
        result = ExecSteps_MoveToAbsPos(&EXEC_MOTOR2_HOME);
    } else if (recovery_step == 2U) {
        result = ExecSteps_MoveToAbsPos(&EXEC_MOTOR1_HOME);
    } else {
        result = LeaveCenter();
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

/**
 * @brief 启动上电后的全回原点流程，并初始化对应的命令状态。
 * @note  调用前必须完成Modbus主站、状态寄存器、Flash及电机位置存储初始化。
 * @return 1表示全回原点状态机已启动，0表示启动失败。
 */
static uint8_t StartInitialFullHoming(void)
{
    uint8_t saved_position_valid;
    uint8_t home_start_result;
    int32_t saved_motor7_position = 0L;
    int32_t saved_motor8_position = 0L;

    /* 电机7、8需要使用掉电前保存的位置完成回零前的安全预移动。 */
    saved_position_valid = MotorPositionStore_Get(
        &saved_motor7_position, &saved_motor8_position);
    home_start_result = MotorControl_FullHomeStart(
        saved_position_valid, saved_motor7_position,
        saved_motor8_position, MOTOR_HOME_SPEED_NORMAL);

    if (home_start_result == MODBUS_RESULT_OK) {
        StatusRegs_Update(REG_COMMAND_CODE,
                          GATEWAY_SERVICE_HOME_COMMAND_REG);
        StatusRegs_Update(REG_COMMAND_STATE, COMMAND_STATE_EXECUTING);
        StatusRegs_Update(REG_COMMAND_STEP,
                          MotorControl_FullHomeGetCurrentSlave());
        return 1U;
    }

    /* 启动失败时不上报执行中，保持正常业务锁定并记录失败原因。 */
    StatusRegs_Update(REG_COMMAND_STATE, COMMAND_STATE_FAILED);
    StatusRegs_Update(REG_FAULT_CODE, home_start_result);
    LOG_ERROR("MAIN", "failed to start initial homing: result=%u\r\n",
              (unsigned int)home_start_result);
    return 0U;
}

/**
 * @brief 监控全回原点状态，并统一管理业务使能及上电回原点的状态上报。
 * @param normal_operations_enabled 正常业务是否允许执行。
 * @param startup_full_homing_pending 是否仍在等待上电全回原点完成。
 * @note  必须在MotorControl_FullHomeProcess()之后调用，才能获取本轮最新状态。
 */
static void FullHomeState_Task(uint8_t *normal_operations_enabled,
                               uint8_t *startup_full_homing_pending)
{
    static MotorFullHomeState last_full_home_state =
        MOTOR_FULL_HOME_STATE_IDLE;
    MotorFullHomeState full_home_state;
    uint16_t fault_code;

    full_home_state = MotorControl_FullHomeGetState();

    /* 上电回原点期间持续上报当前正在处理的电机地址。 */
    if (*startup_full_homing_pending && MotorControl_FullHomeIsBusy()) {
        StatusRegs_Update(REG_COMMAND_STEP,
                          MotorControl_FullHomeGetCurrentSlave());
    }

    /* 同一状态无需重复处理，避免重复打印日志和反复写状态寄存器。 */
    if (full_home_state == last_full_home_state) {
        return;
    }

    if (MotorControl_FullHomeIsBusy()) {
        /* 任意来源的全回原点执行期间，都禁止新业务命令和后台任务。 */
        *normal_operations_enabled = 0U;
        GatewayService_SetControlEnabled(0U);
    } else if (full_home_state == MOTOR_FULL_HOME_STATE_SUCCESS) {
        *normal_operations_enabled = 1U;
        GatewayService_SetControlEnabled(1U);

        /* 只有上电自动回原点负责更新启动命令的完成状态。 */
        if (*startup_full_homing_pending) {
            *startup_full_homing_pending = 0U;
            StatusRegs_Update(REG_COMMAND_STATE, COMMAND_STATE_SUCCESS);
            StatusRegs_Update(REG_COMMAND_STEP,
                              MotorControl_FullHomeGetCurrentSlave());
            StatusRegs_Update(REG_FAULT_CODE, 0U);
        }
        LOG_INFO("MAIN",
                 "full homing completed; normal operations enabled\r\n");
    } else if (full_home_state == MOTOR_FULL_HOME_STATE_FAILED) {
        fault_code =
            ((uint16_t)MotorControl_FullHomeGetFailedSlave() << 8) |
            MotorControl_FullHomeGetLastError();

        *normal_operations_enabled = 0U;
        GatewayService_SetControlEnabled(0U);

        /* 上电自动回原点失败时，将失败电机和错误码反馈给上位机。 */
        if (*startup_full_homing_pending) {
            *startup_full_homing_pending = 0U;
            StatusRegs_Update(REG_COMMAND_STATE, COMMAND_STATE_FAILED);
            StatusRegs_Update(REG_COMMAND_STEP,
                              MotorControl_FullHomeGetFailedSlave());
            StatusRegs_Update(REG_FAULT_CODE, fault_code);
        }
        LOG_ERROR("MAIN",
                  "full homing failed; operations locked: slave=0x%02X, result=%u\r\n",
                  (unsigned int)MotorControl_FullHomeGetFailedSlave(),
                  (unsigned int)MotorControl_FullHomeGetLastError());
    }

    last_full_home_state = full_home_state;
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
	uint8_t normal_operations_enabled = 0U;
	uint8_t startup_full_homing_pending = 0U;

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
	MotorPositionStore_Init();
	LOG_INFO("MAIN", "initialization completed\r\n");
  __enable_irq();  /* 开启全局中断 */
	delay_ms(1000);

	/* 上电初始化完成后启动全回原点，成功前不开放正常业务。 */
	startup_full_homing_pending = StartInitialFullHoming();

//	Motor_Reset(MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, 8);
	// 复位前确保主站状态空闲

//printf("\r\n============= MCU RESET DETECTED =============\r\n");
		while(1)
		{		
			ModbusMaster_Process();
			ModbusSlave_Process();
			MotorControl_FullHomeProcess();

			FullHomeState_Task(&normal_operations_enabled,
							   &startup_full_homing_pending);

			GatewayService_Process();
			/* 自动告警恢复中的序列需要在全回原点失败后完成收尾。 */
			Sequence_Process();
			if (normal_operations_enabled) {
				SwapState_TrySave();   // 延迟保存（Flash 写入过程仍为同步执行）
				StallRecovery_Task();
				/* 后台轮询优先级最低，避免抢在控制命令之前占用主站总线。 */
				//MasterPolling_Task();
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
