#ifndef SEQUENCE_STEPS_H
#define SEQUENCE_STEPS_H

#include <stdint.h>
#include "status_regs.h"

typedef uint8_t (*StepFunc)(void);
typedef uint8_t (*StepContextFunc)(const void *context);

#define STEP_RESULT_RETRY 0x80U
#define STEP_RESULT_WAIT  0x81U

typedef struct {
    StepFunc run;
    uint32_t post_delay_ms;
    StatusRegAddr completion_reg;
    uint16_t completion_value;
    StepContextFunc run_with_context;
    const void *context;
    uint32_t wait_timeout_ms;
    uint8_t no_restart_from_here; // 降落从本步骤起，失败后回原点并结束
} StepDef;


/* 电机设备地址、寄存器、命令值和动作预置参数。 */
// 1. 电机标识（区分不同电机）
#define MOTOR_ID_1       2       // 电机1的唯一标识
#define MOTOR_ID_2       3       // 电机2的唯一标识
#define MOTOR_CLAMP_ID   1       // 夹紧电机的唯一标识
#define MOTOR_ID_4       4       // 电机4的唯一标识
// 每个电机独立的Modbus从站地址
#define MOTOR1_SLAVE_ADDR  0x03   // 电机1从站地址
#define MOTOR2_SLAVE_ADDR  0x02   // 电机2从站地址
#define MOTOR_CLAMP_SLAVE_ADDR 0x01 // 夹紧电机从站地址

#define MOTOR4_SLAVE_ADDR  0x04   // 舱门从站地址

#define MOTOR5_SLAVE_ADDR  0x05   // 电机5从站地址
#define MOTOR6_SLAVE_ADDR  0x06   // 电机6从站地址
#define MOTOR7_SLAVE_ADDR  0x07   // 电机7从站地址
#define MOTOR8_SLAVE_ADDR  0x08   // 电机8从站地址
#define MOTOR9_SLAVE_ADDR  0x09   // 电机9从站地址
#define MOTOR10_SLAVE_ADDR 0x0a   // 电机10从站地址
#define MOTOR11_SLAVE_ADDR 0x0b   // 电机11从站地址
#define MOTOR12_SLAVE_ADDR 0x0c   // 电机12从站地址

#define MOTOR14_SLAVE_ADDR 0x14   // 电机14从站地址(控制遥控器的舵机)

#define MOTOR1_RETURN_POS 10000L    //升降电机1的停住位置
#define MOTOR2_RETURN_POS 100000L   //升降电机2的停住位置

/* 降落流程：检查旧电池是否成功推入电池仓。 */
#ifndef LANDING_BAY_PUSH_RETRY_BACK_POS
#define LANDING_BAY_PUSH_RETRY_BACK_POS       320000L
#endif
#define LANDING_BAY_PUSH_CURRENT_DELTA_LIMIT 10L
#define LANDING_BAY_PUSH_CURRENT_WAIT_MS     10000UL
#define LANDING_BAY_PUSH_MAX_REPUSHES        2U
/* 电机2推电池后电流增长不足，使用独立的业务故障码。 */
#define LANDING_BAY_PUSH_CURRENT_FAULT_CODE  0x02F1U

/* 根据编译目标选择设备标定参数。 */
#include "config/device_profile.h"

/* 控制电流后台轮询和降落时电池仓推进前后的电流检查，默认关闭。 */
#ifndef LANDING_BAY_PUSH_CURRENT_CHECK_ENABLE
#define LANDING_BAY_PUSH_CURRENT_CHECK_ENABLE 1U
#endif
#if LANDING_BAY_PUSH_CURRENT_CHECK_ENABLE != 0U && \
    LANDING_BAY_PUSH_CURRENT_CHECK_ENABLE != 1U
#error "LANDING_BAY_PUSH_CURRENT_CHECK_ENABLE must be 0 or 1"
#endif

/* 步进电机控制寄存器*/
#define MOTOR1_CTRL_REG1      0x00d0  // 电机1绝对位置控制寄存器（D0低字，D1高字）
#define MOTOR2_CTRL_REG1      0x00d0  // 电机2绝对位置控制寄存器（D0低字，D1高字）

/* 夹紧电机控制寄存器*/
#define MOTOR_CLAMP_CTRL_REG1 0x0036  // 夹紧电机控制寄存器地址（松开）
#define MOTOR_CLAMP_CTRL_REG2 0x0037  // 夹紧电机控制寄存器地址（夹紧）
#define MOTOR_CLAMP_CTRL_REG3 0x0047  // 夹紧电机控制寄存器地址（保存）

/* 舱门控制参数 */
#define MOTOR4_CTRL_REG1       0x0046  // 电机4控制寄存器地址
#define MOTOR4_CMD_OPEN                  0x0002  // 开舱
#define MOTOR4_CMD_CLOSE                 0x0001  // 关舱
#define MOTOR4_CMD_STOP                  0x0005  // 停止
#define MOTOR4_HOME_CTRL_REG             0x0047U // 十进制71，回零指令
#define MOTOR4_HOME_TO_NEGATIVE_LIMIT    0x0002U
#define MOTOR4_STATUS_REG                0x004BU // 十进制75，驱动器状态
#define MOTOR4_HOME_COMPLETE_MASK        0x0004U // bit2，回零完成
#define MOTOR4_MOVE_COMPLETE_MASK        0x0080U // bit7，运动完成

/* 夹紧电机命令*/
#define MOTOR_CLAMP_CMD_RELEASE      0x0001         //松开
#define MOTOR_CLAMP_CMD_CLAMP        0x0001         //夹紧

uint8_t Motor1Up1(void);
uint8_t Motor2Forward(void);
uint8_t Motor3Clamp(void);
uint8_t Motor2Back(void);
uint8_t Motor1Down1(void);
uint8_t Motor3Lossen(void);
uint8_t Motor1Up3(void);
uint8_t FlyForward(void);
uint8_t FlyBack(void);
uint8_t RelayCtrl(void);

uint8_t Center_1(void);
uint8_t Center_2(void);
uint8_t LeaveCenter(void);

uint8_t OpenDr(void);
uint8_t CloseDr(void);
uint8_t StopDr(void);
uint8_t CheckAndCloseDoor(void);
uint8_t WaitForUavPowerOn(void);
uint8_t OpenAC(void);
uint8_t CloseAC(void);
uint8_t CloseACForTakeoff(void);
void LandingBayPush_Reset(void);
uint8_t LandingBayPush_HasCurrentFault(void);

/* 只读序列步骤表。 */
extern const StepDef opendr_steps[];
extern const StepDef closedr_steps[];
extern const StepDef takeoff_steps[];
extern const StepDef landing_steps[];
extern const StepDef closecenter_steps[];
extern const StepDef leavecenter_steps[];
extern const StepDef loadbattery_steps[];
extern const StepDef downbattery_steps[];
extern const StepDef recovery_with_battery_steps[];
extern const StepDef recovery_without_battery_steps[];

extern const uint8_t OPENDR_STEP_COUNT;
extern const uint8_t CLOSEDR_STEP_COUNT;
#define TAKEOFF_UAV_POWER_RETRY_START_STEP_INDEX 6U
#define TAKEOFF_UAV_POWER_CHECK_STEP_INDEX      11U
#define TAKEOFF_UAV_POWER_MAX_RETRIES            1U
#define TAKEOFF_UAV_POWER_RETRY_POSITION_OFFSET 2000L
#define OPENFLY_STEP_OFFSET             0U
/* 飞机开关机不执行末尾的居中释放、关舱门和开空调。 */
#define OPENFLY_TRAILING_EXCLUDED_STEPS 3U
#define OPENFLY_STEP_COUNT                                              \
    ((uint8_t)(TAKEOFF_STEP_COUNT - OPENFLY_TRAILING_EXCLUDED_STEPS))
extern const uint8_t TAKEOFF_STEP_COUNT;
/* 不关舱门时，同时排除关门后的开空调步骤。 */
#define TAKEOFF_NO_CLOSE_STEP_COUNT ((uint8_t)(TAKEOFF_STEP_COUNT - 2U))
extern const uint8_t LANDING_STEP_COUNT;
extern const uint8_t CLOSECENTER_STEP_COUNT;
extern const uint8_t LEAVECENTER_STEP_COUNT;
extern const uint8_t LOADBATTERY_STEP_COUNT;
extern const uint8_t DOWNBATTERY_STEP_COUNT;
extern const uint8_t RECOVERY_WITH_BATTERY_COUNT;
extern const uint8_t RECOVERY_WITHOUT_BATTERY_COUNT;

#endif
