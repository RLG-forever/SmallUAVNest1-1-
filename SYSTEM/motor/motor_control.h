#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

/*
 * 03 功能码读取当前位置，两类驱动器均连续读取 2 个寄存器：
 * - 位置命令写入 0x00CE：0x0004 为低 16 位，0x0005 为高 16 位；
 * - 其他电机：十进制 1000 为高 16 位，1001 为低 16 位。
 */
#define MOTOR_CONTROL_CE_COMMAND_REG              0x00CEU
#define MOTOR_CONTROL_CE_POSITION_REG             0x0004U
#define MOTOR_CONTROL_POSITION_REG                1000U
#define MOTOR_CONTROL_POSITION_REG_COUNT          2U

/* 到位检测参数；位置和容差的单位均为电机脉冲数。 */
#define MOTOR_CONTROL_POSITION_TOLERANCE          1000L
#define MOTOR_CONTROL_POSITION_POLL_INTERVAL_MS   1000U
#define MOTOR_CONTROL_MOVE_TIMEOUT_MS             30000U

typedef struct {
    uint8_t slave_addr;
    uint16_t register_address;
    uint16_t value_low_word;
    uint16_t value_high_word;
    /* 当前位置起始寄存器；非 0 时优先使用，否则根据控制寄存器自动选择。 */
    uint16_t position_register_address;
} MotorControlParams;

uint8_t Motor_Single_Control(uint8_t slave_addr, uint8_t motor_num, uint16_t motor_cmd);
uint8_t Motor_Control(uint8_t motor_id, uint8_t reg_num, uint16_t motor_cmd);
uint8_t Motor_Batch_Control(uint8_t slave_addr, uint16_t start_reg,
                            uint16_t reg_num, const uint16_t *motor_cmds);

uint8_t MotorControl_IsBusy(void);
void MotorControl_Cancel(void);
typedef enum {
    MOTOR_CONTROL_TARGET_LIFT_UP = 0,
    MOTOR_CONTROL_TARGET_LIFT_DOWN
} MotorControlTarget;

uint8_t MotorControl_WriteTarget(MotorControlTarget target, uint16_t value);
/*
 * 异步批量相对位移：先读取起始位置，再依次下发位移，最后轮询当前位置。
 * 调用者须以相同参数重复调用，直至返回 OK 或错误。轮询间隔不阻塞主循环，
 * 等待期间返回 PENDING；批次中所有电机到位后才返回 OK。
 */
uint8_t MotorControl_BatchMove(const MotorControlParams *motors, uint8_t count,
                               uint8_t *results);
/*
 * 异步批量下发位移，但不读取和校验当前位置。所有写事务收到正确应答后
 * 即返回 OK；该结果只表示命令下发成功，不表示电机已经运动到位。
 */
uint8_t MotorControl_BatchMoveNoPositionCheck(
    const MotorControlParams *motors, uint8_t count, uint8_t *results);
uint8_t ModbusBatch_IsBusy(void);
void ModbusBatch_Cancel(void);

uint8_t MotorMonitor_StallTriggered(void);
void MotorMonitor_ClearStallTrigger(void);

#endif
