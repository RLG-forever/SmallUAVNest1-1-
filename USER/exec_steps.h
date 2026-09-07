#ifndef EXEC_STEPS_H
#define EXEC_STEPS_H

#include <stddef.h>
#include <stdint.h>

#include "motor_control.h"

typedef struct {
    const MotorMoveAbsPosParams *motors;
    uint8_t count;
} ExecMoveAbsPosParams;

uint8_t ExecSteps_MoveToAbsPos(const void *context);

#define EXEC_ABS_POS(slave, position)                                      \
    {(slave), MOTOR_CONTROL_ABSOLUTE_COMMAND_REG, (position), 0U}

#define EXEC_ARRAY_COUNT(array)                                           \
    ((uint8_t)(sizeof(array) / sizeof((array)[0])))

/* 多电机：传入 MotorMoveAbsPosParams 数组，数量由宏自动计算。 */
#define EXEC_MOVE_ABS_ARRAY_STEP(motors, delay_ms, completion_reg,         \
                                 completion_value)                        \
    {NULL, (delay_ms), (completion_reg), (completion_value),               \
     ExecSteps_MoveToAbsPos,                                              \
     &(const ExecMoveAbsPosParams){(motors), EXEC_ARRAY_COUNT(motors)}}

/* 单电机：直接传入从站地址和绝对位置。 */
#define EXEC_MOVE_ABS_STEP(slave, position, delay_ms, completion_reg,      \
                           completion_value)                              \
    {NULL, (delay_ms), (completion_reg), (completion_value),               \
     ExecSteps_MoveToAbsPos,                                              \
     &(const ExecMoveAbsPosParams){                                       \
         (const MotorMoveAbsPosParams[]){                                 \
             EXEC_ABS_POS((slave), (position))},                          \
         1U}}

#endif /* EXEC_STEPS_H */
