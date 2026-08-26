#ifndef SEQUENCE_H
#define SEQUENCE_H

#include <stdint.h>
#include "status_service.h"

#define UAV_CONTROLLER_SLAVE          0x14U
#define UAV_POWER_CTRL_REG            0x0001U
#define UAV_POWER_ON_VALUE            0x0001U
#define UAV_POWER_MODE2_VALUE         0x0002U

#define CHARGER_SLAVE                 0x13U
#define CHARGER_POWER_CTRL_REG        0x0000U
#define CHARGER_POWER_ON_VALUE        0x0001U
#define CHARGER_POWER_OFF_VALUE       0x0000U

#define AIR_CONDITIONER_SLAVE         0x0DU
#define AIR_POWER_CTRL_REG            0x002FU
#define AIR_POWER_ON_VALUE            0x0001U
#define AIR_POWER_OFF_VALUE           0x0000U
#define AIR_COOLING_STOP_TEMP_REG     0x0000U
#define AIR_HEATING_STOP_TEMP_REG     0x0002U


// 居中目标位置（示例值，需根据实际测量）
#define MOTOR5_CENTER_POS   0x00000000
#define MOTOR6_CENTER_POS   0x00000000
#define MOTOR7_CENTER_POS   0x00000000
#define MOTOR8_CENTER_POS   0x00000000
#define MOTOR9_CENTER_POS   0x00000000
#define MOTOR10_CENTER_POS  0x00000000
#define MOTOR11_CENTER_POS  0x00000000
#define MOTOR12_CENTER_POS  0x00000000

// 释放目标位置（示例）
#define MOTOR5_RELEASE_POS  0x00010000
#define MOTOR6_RELEASE_POS  0x00010000
#define MOTOR7_RELEASE_POS  0x00010000
#define MOTOR8_RELEASE_POS  0x00010000
#define MOTOR9_RELEASE_POS  0x00010000
#define MOTOR10_RELEASE_POS 0x00010000
#define MOTOR11_RELEASE_POS 0x00010000
#define MOTOR12_RELEASE_POS 0x00010000

// 序列ID
typedef enum {
    SEQ_ID_TAKEOFF = 0,   // 一键起飞
    SEQ_ID_LANDING,       // 降落完成
		SEQ_ID_CLOSECENTER,   // 居中杆居中
		SEQ_ID_LEAVECENTER,   // 居中杆释放
		SEQ_ID_LOADBATTERY,   // 装电池
		SEQ_ID_DOWNBATTERY,   // 下电池
		SEQ_ID_OPENUP,    		// 开门上升
		SEQ_ID_CLOSEDOWN,    	// 下降关门
		SEQ_ID_OPENFLY,    	  // 飞机开机
	  SEQ_ID_CLOSEFLY,    	// 飞机关机
		SEQ_ID_OPENDR,         // 打开舱门
	  SEQ_ID_CLOSEDR,       // 关闭舱门
		SEQ_ID_CANCEL,    		// 取消
		SEQ_ID_OPENDR1,         // 打开舱门
} SeqId;

/* Sequence_Start 的明确返回结果，调用者不应再通过全局状态猜测。 */
typedef enum {
    SEQ_START_OK = 0,          /* 已成功创建并启动序列 */
    SEQ_START_BUSY,            /* 已有序列正在运行 */
    SEQ_START_INVALID_ID,      /* 序列 ID 不受支持 */
    SEQ_START_MASTER_BUSY      /* 主站事务或电机批处理仍被占用 */
} SequenceStartResult;


/* 步骤函数可返回 ModbusResult；需要显式重试时返回 STEP_RESULT_RETRY。 */
typedef uint8_t (*StepFunc)(void);

/* 独立于 ModbusResult，避免将通信错误误解释为动作重试。 */
#define STEP_RESULT_RETRY 0x80U

// 步骤定义
typedef struct {
    StepFunc func;      // 执行该步骤的函数
    uint32_t wait_ms;   // 执行后等待时间（毫秒）
        StatusServiceField update_field; // 完成后更新的业务状态字段
    uint16_t update_value;   // 更新值
} StepDef;

// 初始化序列模块
void Sequence_Init(void);

// 启动指定序列（由从站回调调用）
SequenceStartResult Sequence_Start(SeqId id);

// 查询是否有序列正在执行
uint8_t Sequence_IsBusy(void);

// 序列处理函数（需在主循环中周期性调用）
void Sequence_Process(void);

void Sequence_Pause(void);   // 暂停当前序列
void Sequence_Resume(void);  // 恢复执行
void BatterySequence_TimerTick(void);      // 定时器中断调用
void Sequence_ForceStop(void);

uint8_t LeaveCenter(void);
uint8_t LeaveCenter1(void);
uint8_t LeaveCenter2(void);
uint8_t HoistDown(void);
void Sequence_Cancel(void);


uint8_t Center_1(void);
uint8_t Center_2(void);


uint8_t Battery_1(void);
uint8_t Battery_2(void);
uint8_t Battery_3(void);
uint8_t Battery_4(void);
uint8_t Battery_5(void);
uint8_t Battery_6(void);
uint8_t Battery_7(void);
uint8_t Battery_8(void);
uint8_t Battery_9(void);
uint8_t Battery_10(void);
uint8_t Battery_11(void);
uint8_t Battery_12(void);
uint8_t Battery_13(void);
uint8_t Battery_14(void);
uint8_t Battery_15(void);
uint8_t Battery_16(void);
uint8_t Battery_17(void);
uint8_t Battery_18(void);
uint8_t Battery_19(void);
uint8_t Battery_20(void);
uint8_t Battery_21(void);
uint8_t Battery_22(void);
uint8_t Battery_23(void);
uint8_t Battery_24(void);
uint8_t Battery_25(void);
uint8_t Battery_26(void);
uint8_t Battery_27(void);
uint8_t Battery_28(void);
uint8_t Battery_29(void);

uint8_t OpenAC(void);
uint8_t CloseAC(void);
uint8_t RelayCtrl(void);

uint8_t OpenDr(void);
uint8_t CloseDr(void);
uint8_t StopDr(void);
uint8_t CheckAndCloseDoor(void);

uint8_t RemoveBattery(void);

uint8_t Sequence_GetCurrentId(void);
uint8_t Sequence_GetCurrentStep(void);
const uint8_t* GetMotorListForCurrentStep(void);

#endif
