#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

typedef struct {
    uint8_t slave_addr;
    uint16_t register_address;
    uint16_t value_low_word;
    uint16_t value_high_word;
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
uint8_t MotorControl_BatchMove(const MotorControlParams *motors, uint8_t count,
                               uint8_t *results);
uint8_t ModbusBatch_IsBusy(void);
void ModbusBatch_Cancel(void);

uint8_t MotorMonitor_StallTriggered(void);
void MotorMonitor_ClearStallTrigger(void);

#endif
