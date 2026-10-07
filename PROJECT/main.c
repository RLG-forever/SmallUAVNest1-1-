#define PROJECT_DEBUG 1U

#include "project.h"
#include "debug_log.h"
#include "startup_door.h"
#include "full_home_service.h"

const uint8_t g_project_debug_enabled = PROJECT_DEBUG;

/* 0=关闭；0x38=上电回零后执行一次起飞步骤；0x40=执行一次降落步骤。 */
#ifndef STARTUP_DEBUG_COMMAND
#define STARTUP_DEBUG_COMMAND             0x40U
#endif

/* 0 表示使用已保存的空仓号；1～3 表示调试降落流程时指定空仓号。 */
#ifndef STARTUP_DEBUG_EMPTY_BAY
#define STARTUP_DEBUG_EMPTY_BAY        3U
#endif

#if STARTUP_DEBUG_EMPTY_BAY > 3U
#error STARTUP_DEBUG_EMPTY_BAY must be 0U, 1U, 2U or 3U
#endif

/* 调试模式下执行 0x40 命令（上电自动或网关触发）时，在指定仓号存取电池。 */
#ifndef STARTUP_DEBUG_SAME_BAY
#define STARTUP_DEBUG_SAME_BAY          1U
#endif

#if STARTUP_DEBUG_SAME_BAY > 1U
#error STARTUP_DEBUG_SAME_BAY must be 0U or 1U
#endif

#if STARTUP_DEBUG_COMMAND != 0x00U && \
    STARTUP_DEBUG_COMMAND != 0x38U && \
    STARTUP_DEBUG_COMMAND != 0x40U
#error STARTUP_DEBUG_COMMAND must be 0x00U, 0x38U or 0x40U
#endif

TMRGEN       tmr = TIMR_DEFAULTS;
LinterMotor  lim = LINTER_DEFAULTS;

void SysTickInit(void)
{
	SysTick_Config(SystemCoreClock / 1000);	//Set SysTick Timer for 1ms interrupts  
}

int main(void)
{
	uint8_t startup_door_sequence_pending = 1U;
	uint8_t startup_result;
#if STARTUP_DEBUG_COMMAND != 0x00U
    uint8_t debug_command_started = 0U;
#endif

	SystemInit();
	Tick_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
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
#if PROJECT_DEBUG != 0U && STARTUP_DEBUG_EMPTY_BAY != 0U
    if (SwapState_GetEmptyBay() != STARTUP_DEBUG_EMPTY_BAY) {
        SwapState_SetEmptyBay((uint8_t)STARTUP_DEBUG_EMPTY_BAY);
    }
#endif
    Sequence_SetLandingSameBayDebugEnabled(
        (uint8_t)(PROJECT_DEBUG != 0U && STARTUP_DEBUG_SAME_BAY != 0U));
    StatusRegs_Update(REG_RESERVED2, SwapState_GetEmptyBay());
    LOG_INFO("MAIN", "startup empty bay=%u, debug override=%u\r\n",
             (unsigned int)SwapState_GetEmptyBay(),
             (unsigned int)STARTUP_DEBUG_EMPTY_BAY);
	delay_ms(1000);

    while(1)
    {		
        ModbusMaster_Process();
        ModbusSlave_Process();
        if (startup_door_sequence_pending) {
            startup_result = StartupDoorSequence_Task();
            if (startup_result != MODBUS_RESULT_PENDING &&
                startup_result != MODBUS_RESULT_BUSY) {
                startup_door_sequence_pending = 0U;
                if (startup_result == MODBUS_RESULT_OK) {
                    /* 舱门启动流程完成或被配置跳过后，再启动整机回零。 */
                    FullHomeService_StartInitial();
                }
            }
        }
        MotorControl_FullHomeProcess();

        FullHomeService_Process();

#if STARTUP_DEBUG_COMMAND != 0x00U
        if (FullHomeService_IsNormalOperationsEnabled() && !debug_command_started &&
            !Sequence_IsBusy() && !MotorControl_IsBusy()) {
            SequenceStartResult debug_start_result;
#if STARTUP_DEBUG_COMMAND == 0x38U
            debug_start_result = Sequence_Start(SEQ_ID_TAKEOFF);
#else
            debug_start_result = Sequence_Start(SEQ_ID_LANDING);
#endif
            if (debug_start_result == SEQ_START_OK) {
                debug_command_started = 1U;
                LOG_INFO("MAIN", "startup debug command 0x%02X started, same_bay=%u\r\n",
                         (unsigned int)STARTUP_DEBUG_COMMAND,
                         (unsigned int)(PROJECT_DEBUG != 0U &&
                                        STARTUP_DEBUG_COMMAND == 0x40U &&
                                        STARTUP_DEBUG_SAME_BAY != 0U));
            }
        }
#endif

        GatewayService_Process();
        /* 自动告警恢复中的序列需要在全回原点失败后完成收尾。 */
        Sequence_Process(); 
        /*
            * GatewayService_Process() 可能在本轮启动全回原点，执行后台
            * Modbus 任务前必须重新检查实时状态。否则轮询可能在回原点启动后
            * 提交事务，随后因正常业务被禁用而无法取走完成结果，造成 Modbus
            * 主站被该轮询事务持续占用。
            */
        if (FullHomeService_IsNormalOperationsEnabled() &&
            !MotorControl_FullHomeIsBusy()) {
            /* 后台轮询优先级最低，避免抢在控制命令之前占用主站总线。 */
            MasterPolling_Task();
        }
    }
}
