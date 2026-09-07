#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

/*
 * 03 功能码读取当前位置，两类驱动器均连续读取 2 个寄存器：
 * - 绝对位置命令写入 0x00D0~0x00D1：0x00D0 为低 16 位，0x00D1 为高 16 位；
 * - 当前位置读取 0x0004~0x0005：0x0004 为高 16 位，0x0005 为低 16 位；
 * - 其他电机：十进制 1000 为高 16 位，1001 为低 16 位。
 */
#define MOTOR_CONTROL_ABSOLUTE_COMMAND_REG        0x00D0U
#define MOTOR_CONTROL_RELATIVE_COMMAND_REG        0x00CEU
#define MOTOR_CONTROL_DRIVER_POSITION_REG         0x0004U
#define MOTOR_CONTROL_POSITION_REG                1000U
#define MOTOR_CONTROL_POSITION_REG_COUNT          2U

/* 0x00A3 的低 4 位为当前告警代码，0 表示当前无告警。 */
#define MOTOR_CONTROL_ALARM_STATUS_REG             0x00A3U
#define MOTOR_CONTROL_ALARM_CLEAR_REG              0x00A4U
#define MOTOR_CONTROL_ALARM_STATUS_MASK            0x000FU
#define MOTOR_CONTROL_RESULT_ALARM_BASE            0xA0U
#define MOTOR_CONTROL_RESULT_ALARM(code) \
    ((uint8_t)(MOTOR_CONTROL_RESULT_ALARM_BASE | \
               ((uint8_t)(code) & (uint8_t)MOTOR_CONTROL_ALARM_STATUS_MASK)))
#define MOTOR_CONTROL_IS_ALARM_RESULT(result) \
    ((((uint8_t)(result) & 0xF0U) == MOTOR_CONTROL_RESULT_ALARM_BASE) ? 1U : 0U)

/* 到位检测参数；位置和容差的单位均为电机脉冲数。 */
#define MOTOR_CONTROL_POSITION_TOLERANCE          1000L
#define MOTOR_CONTROL_POSITION_POLL_INTERVAL_MS   1000U
#define MOTOR_CONTROL_MOVE_TIMEOUT_MS             30000U

/* 回原点协议：向 0x00C9 写速度命令，读取 0x0006~0x0007 的运行状态。 */
#define MOTOR_HOME_COMMAND_REG                    0x00C9U
#define MOTOR_HOME_STATUS_REG                     0x0006U
#define MOTOR_HOME_STATUS_REG_COUNT               2U
#define MOTOR_HOME_COMPLETE_MASK                  0x8000U

/* 回零前配对电机调平参数，位置单位均为脉冲。 */
#define MOTOR_HOME_LEVEL_PAIR_COUNT                4U
#define MOTOR_HOME_LEVEL_ERROR_THRESHOLD           5000L
#define MOTOR_HOME_LEVEL_COMPLETE_TOLERANCE         100L
#define MOTOR_HOME_LEVEL_STABLE_SAMPLE_COUNT          2U
#define MOTOR_HOME_LEVEL_POLL_INTERVAL_MS           500U
#define MOTOR_HOME_LEVEL_TIMEOUT_MS               30000U

#define MOTOR_HOME_SPEED_SLOW                     0xCFD0U
#define MOTOR_HOME_SPEED_NORMAL                   0xCFD0U
#define MOTOR_HOME_SPEED_FAST                     0xFFD0U
#define MOTOR_HOME_POLL_INTERVAL_MS               500U
#define MOTOR_HOME_TIMEOUT_MS                     60000U

/* 开机回零前，电机7/8的安全预移动参数。 */
#define MOTOR_PREHOME_POSITION_LIMIT              275000L
#define MOTOR_PREHOME_STABLE_TOLERANCE            100L
#define MOTOR_PREHOME_STABLE_SAMPLE_COUNT         3U
#define MOTOR_PREHOME_POLL_INTERVAL_MS            500U
#define MOTOR_PREHOME_TIMEOUT_MS                  30000U

typedef enum {
    MOTOR_HOME_STATE_IDLE = 0,
    MOTOR_HOME_STATE_CHECK_ALARM,
    MOTOR_HOME_STATE_CLEAR_ALARM,
    MOTOR_HOME_STATE_VERIFY_CLEAR,
    MOTOR_HOME_STATE_READ_PAIR_FIRST,
    MOTOR_HOME_STATE_READ_PAIR_SECOND,
    MOTOR_HOME_STATE_LEVEL_PAIR,
    MOTOR_HOME_STATE_WAIT_LEVEL,
    MOTOR_HOME_STATE_CHECK_LEVEL_ALARM,
    MOTOR_HOME_STATE_SEND_COMMAND,
    MOTOR_HOME_STATE_WAIT_POLL,
    MOTOR_HOME_STATE_READ_STATUS,
    MOTOR_HOME_STATE_SUCCESS,
    MOTOR_HOME_STATE_FAILED
} MotorHomeState;

typedef enum {
    MOTOR_PREHOME_STATE_IDLE = 0,
    MOTOR_PREHOME_STATE_CHECK_ALARM,
    MOTOR_PREHOME_STATE_CLEAR_ALARM,
    MOTOR_PREHOME_STATE_VERIFY_CLEAR,
    MOTOR_PREHOME_STATE_READ_INITIAL_MOTOR7,
    MOTOR_PREHOME_STATE_READ_INITIAL_MOTOR8,
    MOTOR_PREHOME_STATE_WRITE_MOTOR7,
    MOTOR_PREHOME_STATE_WRITE_MOTOR8,
    MOTOR_PREHOME_STATE_WAIT_POLL,
    MOTOR_PREHOME_STATE_READ_STABLE_MOTOR7,
    MOTOR_PREHOME_STATE_READ_STABLE_MOTOR8,
    MOTOR_PREHOME_STATE_READY,
    MOTOR_PREHOME_STATE_FAILED
} MotorPreHomeState;

typedef enum {
    MOTOR_FULL_HOME_STATE_IDLE = 0,
    MOTOR_FULL_HOME_STATE_INITIAL_HOMING,
    MOTOR_FULL_HOME_STATE_CHECK_MOTOR2_HOME,
    MOTOR_FULL_HOME_STATE_CHECK_MOTOR1_HOME,
    MOTOR_FULL_HOME_STATE_PREHOME,
    MOTOR_FULL_HOME_STATE_OTHER_HOMING,
    MOTOR_FULL_HOME_STATE_FINAL_HOMING,
    MOTOR_FULL_HOME_STATE_SUCCESS,
    MOTOR_FULL_HOME_STATE_FAILED
} MotorFullHomeState;

typedef struct {
    uint8_t slave_addr;
    uint16_t register_address;
    uint16_t value_low_word;
    uint16_t value_high_word;
    /* 当前位置起始寄存器；非 0 时优先使用，否则根据控制寄存器自动选择。 */
    uint16_t position_register_address;
} MotorControlParams;

typedef struct {
    uint8_t slave_addr;
    uint16_t register_address;
    int32_t absolute_pos;
    uint16_t position_register_address;
} MotorMoveAbsPosParams;

uint8_t Motor_Single_Control(uint8_t slave_addr, uint8_t motor_num, uint16_t motor_cmd);
uint8_t Motor_Control(uint8_t motor_id, uint8_t reg_num, uint16_t motor_cmd);
uint8_t Motor_Batch_Control(uint8_t slave_addr, uint16_t start_reg,
                            uint16_t reg_num, const uint16_t *motor_cmds);

uint8_t MotorControl_IsBusy(void);
void MotorControl_Cancel(void);

/*
 * 非阻塞回原点服务；Start 只创建任务，Process 在主循环中持续推进。
 * Motor1 的回原点命令仅在 Motor2 已成功回原点后才允许下发。
 */
uint8_t MotorControl_HomeStart(uint16_t speed_command);
uint8_t MotorControl_HomeStartSingle(uint8_t slave_addr,
                                     uint16_t speed_command);
void MotorControl_HomeProcess(void);
uint8_t MotorControl_HomeIsBusy(void);
MotorHomeState MotorControl_HomeGetState(void);
uint8_t MotorControl_HomeGetCurrentSlave(void);
uint8_t MotorControl_HomeGetFailedSlave(void);
uint8_t MotorControl_HomeGetLastError(void);
uint16_t MotorControl_HomeGetStatusWord(void);
uint16_t MotorControl_HomeGetCompletedMask(void);
uint8_t MotorControl_PreHomeStart(uint8_t saved_position_valid,
                                  int32_t motor7_saved_position,
                                  int32_t motor8_saved_position);
void MotorControl_PreHomeProcess(void);
uint8_t MotorControl_PreHomeIsBusy(void);
MotorPreHomeState MotorControl_PreHomeGetState(void);
uint8_t MotorControl_PreHomeGetCurrentSlave(void);
uint8_t MotorControl_PreHomeGetFailedSlave(void);
uint8_t MotorControl_PreHomeGetLastError(void);
uint8_t MotorControl_FullHomeStart(uint8_t saved_position_valid,
                                   int32_t motor7_saved_position,
                                   int32_t motor8_saved_position,
                                   uint16_t speed_command);
void MotorControl_FullHomeProcess(void);
uint8_t MotorControl_FullHomeIsBusy(void);
MotorFullHomeState MotorControl_FullHomeGetState(void);
uint8_t MotorControl_FullHomeGetCurrentSlave(void);
uint8_t MotorControl_FullHomeGetFailedSlave(void);
uint8_t MotorControl_FullHomeGetLastError(void);
typedef enum {
    MOTOR_CONTROL_TARGET_LIFT_UP = 0,
    MOTOR_CONTROL_TARGET_LIFT_DOWN
} MotorControlTarget;

uint8_t MotorControl_WriteTarget(MotorControlTarget target, uint16_t value);
/*
 * 异步批量绝对位置：直接向 0x00D0~0x00D1 下发目标位置，最后轮询当前位置。
 * 调用者须以相同参数重复调用，直至返回 OK 或错误。轮询间隔不阻塞主循环，
 * 等待期间返回 PENDING；批次中所有电机到位后才返回 OK。
 */
uint8_t MotorControl_BatchMove(const MotorControlParams *motors, uint8_t count,
                               uint8_t *results);
uint8_t MotorControl_BatchMoveCapture(const MotorControlParams *motors,
                                      uint8_t count, uint8_t *results,
                                      int32_t *positions);
/* 使用有符号绝对位置；调用方式、到位检测和返回值与 BatchMove 相同。 */
uint8_t MotorControl_MoveToAbsPos(const MotorMoveAbsPosParams *motors,
                                 uint8_t count, uint8_t *results);
uint8_t MotorControl_MoveToAbsPosCapture(const MotorMoveAbsPosParams *motors,
                                        uint8_t count, uint8_t *results,
                                        int32_t *positions);
/*
 * 异步批量下发绝对位置，但不读取和校验当前位置。所有写事务收到正确应答后
 * 即返回 OK；该结果只表示命令下发成功，不表示电机已经运动到位。
 */
uint8_t MotorControl_BatchMoveNoPositionCheck(
    const MotorControlParams *motors, uint8_t count, uint8_t *results);
uint8_t ModbusBatch_IsBusy(void);
void ModbusBatch_Cancel(void);

uint8_t MotorMonitor_StallTriggered(void);
void MotorMonitor_ClearStallTrigger(void);

#endif
