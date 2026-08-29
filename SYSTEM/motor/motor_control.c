#include "motor_control.h"
#include "sequence_steps.h"
#include "modbus_common.h"
#include "modbus_master.h"
#include "tick.h"
#include "debug_log.h"

#include <stddef.h>
#include <string.h>

#define MODBUS_BATCH_MAX_MOTORS 8U

typedef enum {
    MOTOR_BATCH_PHASE_IDLE = 0,
    MOTOR_BATCH_PHASE_READ_START,
    MOTOR_BATCH_PHASE_WRITE,
    MOTOR_BATCH_PHASE_WAIT_POLL,
    MOTOR_BATCH_PHASE_VERIFY
} MotorBatchPhase;

/* 物理总线为串行总线，因此多电机命令按事务依次执行。 */
typedef struct {
    uint8_t active;
    uint8_t check_position;
    MotorBatchPhase phase;
    uint8_t count;
    uint8_t index;
    uint8_t failures;
    uint8_t first_failure;
    uint16_t position_words[MOTOR_CONTROL_POSITION_REG_COUNT];
    uint32_t next_poll_tick;
    uint32_t verify_deadline;
    int32_t target_positions[MODBUS_BATCH_MAX_MOTORS];
    uint8_t results[MODBUS_BATCH_MAX_MOTORS];
    MotorControlParams motors[MODBUS_BATCH_MAX_MOTORS];
} MotorBatchContext;

static MotorBatchContext motor_batch;

static uint16_t MotorControl_GetPositionRegister(
    const MotorControlParams *motor)
{
    if (motor->position_register_address != 0U) {
        return motor->position_register_address;
    }
    if (motor->register_address == MOTOR_CONTROL_CE_COMMAND_REG) {
        return MOTOR_CONTROL_CE_POSITION_REG;
    }
    return MOTOR_CONTROL_POSITION_REG;
}

static uint8_t MotorControl_IsTimeReached(uint32_t now, uint32_t deadline)
{
    return (uint8_t)((int32_t)(now - deadline) >= 0);
}

static int32_t MotorControl_CombinePosition(uint16_t high_word,
                                            uint16_t low_word)
{
    uint32_t raw_position;

    raw_position = ((uint32_t)high_word << 16) | (uint32_t)low_word;
    return (int32_t)raw_position;
}

static int32_t MotorControl_CommandDelta(const MotorControlParams *motor)
{
    return MotorControl_CombinePosition(motor->value_high_word,
                                        motor->value_low_word);
}

static uint8_t MotorControl_PositionReached(int32_t current,
                                            int32_t target)
{
    int64_t difference = (int64_t)current - (int64_t)target;

    if (difference < 0) {
        difference = -difference;
    }
    return (uint8_t)(difference <= MOTOR_CONTROL_POSITION_TOLERANCE);
}

static uint8_t MotorControl_RequestMatches(const MotorControlParams *motors,
                                           uint8_t count,
                                           uint8_t check_position)
{
    uint8_t index;

    if (count != motor_batch.count ||
        check_position != motor_batch.check_position) {
        return 0U;
    }
    for (index = 0U; index < count; index++) {
        if (motor_batch.motors[index].slave_addr != motors[index].slave_addr ||
            motor_batch.motors[index].register_address !=
                motors[index].register_address ||
            motor_batch.motors[index].value_low_word !=
                motors[index].value_low_word ||
            motor_batch.motors[index].value_high_word !=
                motors[index].value_high_word ||
            MotorControl_GetPositionRegister(&motor_batch.motors[index]) !=
                MotorControl_GetPositionRegister(&motors[index])) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t MotorControl_FinishBatch(uint8_t result, uint8_t *results)
{
    LOG_INFO("MOTOR", "batch finished: count=%u, result=%u\r\n",
                      (unsigned int)motor_batch.count,
                      (unsigned int)result);
    if (results != NULL) {
        memcpy(results, motor_batch.results,
               motor_batch.count * sizeof(motor_batch.results[0]));
    }
    memset(&motor_batch, 0, sizeof(motor_batch));
    return result;
}

static uint8_t MotorControl_ReadCurrentPosition(int32_t *position)
{
    uint8_t result;
    uint16_t position_register;

    position_register = MotorControl_GetPositionRegister(
        &motor_batch.motors[motor_batch.index]);
    result = ModbusMaster_03_ReadHoldReg(
        MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
        motor_batch.motors[motor_batch.index].slave_addr,
        position_register,
        MOTOR_CONTROL_POSITION_REG_COUNT,
        motor_batch.position_words);
    if (result == MODBUS_RESULT_OK) {
        if (position_register == MOTOR_CONTROL_CE_POSITION_REG) {
            /* This driver returns 0x0004 low word, then 0x0005 high word. */
            *position = MotorControl_CombinePosition(
                motor_batch.position_words[1],
                motor_batch.position_words[0]);
        } else {
            *position = MotorControl_CombinePosition(
                motor_batch.position_words[0],
                motor_batch.position_words[1]);
        }
    }
    return result;
}

/* 堵转状态由电机服务模块统一持有，避免头文件静态变量产生多个副本。 */
static volatile uint8_t stall_triggered = 0U;

uint8_t MotorMonitor_StallTriggered(void)
{
    return stall_triggered;
}

void MotorMonitor_ClearStallTrigger(void)
{
    stall_triggered = 0U;
}

uint8_t Motor_Single_Control(uint8_t slave_addr, uint8_t motor_num, uint16_t motor_cmd)
{
    uint8_t result;

    if (motor_num != 1U) {
        LOG_ERROR("MOTOR", "single write rejected: slave=0x%02X, motor_num=%u\r\n",
                          (unsigned int)slave_addr,
                          (unsigned int)motor_num);
        return MODBUS_RESULT_PARAM;
    }
    result = ModbusMaster_06_WriteSingleReg(
        MODBUS_MASTER_CLIENT_MOTOR_CONTROL, slave_addr, MOTOR1_CTRL_REG1,
        motor_cmd);
    if (result != MODBUS_RESULT_PENDING && result != MODBUS_RESULT_BUSY) {
        LOG_INFO("MOTOR",
            "single write: slave=0x%02X, reg=0x%04X, value=0x%04X, result=%u\r\n",
            (unsigned int)slave_addr, (unsigned int)MOTOR1_CTRL_REG1,
            (unsigned int)motor_cmd, (unsigned int)result);
    }
    return result;
}

uint8_t Motor_Control(uint8_t motor_id, uint8_t reg_num, uint16_t motor_cmd)
{
    uint8_t slave_addr;
    uint8_t result;
    uint16_t reg_addr;

    switch (motor_id) {
        case MOTOR_ID_1:
            if (reg_num != 1U) return MODBUS_RESULT_PARAM;
            slave_addr = MOTOR1_SLAVE_ADDR;
            reg_addr = MOTOR1_CTRL_REG1;
            break;
        case MOTOR_ID_2:
            if (reg_num != 1U) return MODBUS_RESULT_PARAM;
            slave_addr = MOTOR2_SLAVE_ADDR;
            reg_addr = MOTOR2_CTRL_REG1;
            break;
        case MOTOR_CLAMP_ID:
            slave_addr = MOTOR_CLAMP_SLAVE_ADDR;
            if (reg_num == 1U) reg_addr = MOTOR_CLAMP_CTRL_REG1;
            else if (reg_num == 2U) reg_addr = MOTOR_CLAMP_CTRL_REG2;
            else if (reg_num == 3U) reg_addr = MOTOR_CLAMP_CTRL_REG3;
            else return MODBUS_RESULT_PARAM;
            break;
        case MOTOR_ID_4:
            if (reg_num != 1U) return MODBUS_RESULT_PARAM;
            slave_addr = MOTOR4_SLAVE_ADDR;
            reg_addr = MOTOR4_CTRL_REG1;
            break;
        default:
            LOG_ERROR("MOTOR", "control rejected: motor_id=%u, reg_num=%u\r\n",
                              (unsigned int)motor_id,
                              (unsigned int)reg_num);
            return MODBUS_RESULT_PARAM;
    }
    result = ModbusMaster_06_WriteSingleReg(
        MODBUS_MASTER_CLIENT_MOTOR_CONTROL, slave_addr, reg_addr, motor_cmd);
    if (result != MODBUS_RESULT_PENDING && result != MODBUS_RESULT_BUSY) {
        LOG_INFO("MOTOR",
            "control: motor_id=%u, slave=0x%02X, reg=0x%04X, value=0x%04X, result=%u\r\n",
            (unsigned int)motor_id, (unsigned int)slave_addr,
            (unsigned int)reg_addr, (unsigned int)motor_cmd,
            (unsigned int)result);
    }
    return result;
}

uint8_t Motor_Batch_Control(uint8_t slave_addr, uint16_t start_reg,
                            uint16_t reg_num, const uint16_t *motor_cmds)
{
    uint8_t result;

    result = ModbusMaster_10_WriteMultiReg(
        MODBUS_MASTER_CLIENT_MOTOR_CONTROL, slave_addr, start_reg, reg_num,
        motor_cmds);
    if (result != MODBUS_RESULT_PENDING && result != MODBUS_RESULT_BUSY) {
        LOG_INFO("MOTOR",
            "multi write: slave=0x%02X, start_reg=0x%04X, count=%u, result=%u\r\n",
            (unsigned int)slave_addr, (unsigned int)start_reg,
            (unsigned int)reg_num, (unsigned int)result);
    }
    return result;
}

uint8_t MotorControl_IsBusy(void)
{
    return (uint8_t)(ModbusMaster_IsBusy() || motor_batch.active);
}

void MotorControl_Cancel(void)
{
    LOG_WARN("MOTOR", "control canceled\r\n");
    ModbusBatch_Cancel();
    ModbusMaster_Cancel();
}

uint8_t MotorControl_WriteTarget(MotorControlTarget target, uint16_t value)
{
    switch (target) {
        case MOTOR_CONTROL_TARGET_LIFT_UP:
            return ModbusMaster_06_WriteSingleReg(
                MODBUS_MASTER_CLIENT_GATEWAY, 0x12U, 0x0005U, value);
        case MOTOR_CONTROL_TARGET_LIFT_DOWN:
            return ModbusMaster_06_WriteSingleReg(
                MODBUS_MASTER_CLIENT_GATEWAY, 0x12U, 0x0006U, value);
        default:
            return MODBUS_RESULT_PARAM;
    }
}

static uint8_t MotorControl_BatchMoveInternal(
    const MotorControlParams *motors, uint8_t count, uint8_t *results,
    uint8_t check_position)
{
    uint16_t commands[2];
    uint8_t result;
    int32_t current_position;
    uint32_t now;

    if (motors == NULL || count == 0U || count > MODBUS_BATCH_MAX_MOTORS) {
        LOG_ERROR("MOTOR", "batch rejected: motors=%p, count=%u\r\n",
                          (const void *)motors, (unsigned int)count);
        return MODBUS_RESULT_PARAM;
    }
    if (!motor_batch.active) {
        memcpy(motor_batch.motors, motors,
               count * sizeof(MotorControlParams));
        memset(motor_batch.results, MODBUS_RESULT_PENDING,
               count * sizeof(motor_batch.results[0]));
        motor_batch.active = 1U;
        motor_batch.check_position = check_position;
        motor_batch.phase = check_position ? MOTOR_BATCH_PHASE_READ_START
                                           : MOTOR_BATCH_PHASE_WRITE;
        motor_batch.count = count;
        motor_batch.index = 0U;
        motor_batch.failures = 0U;
        motor_batch.first_failure = MODBUS_RESULT_OK;
        LOG_INFO("MOTOR", "batch started: count=%u, position_check=%u\r\n",
                          (unsigned int)count,
                          (unsigned int)check_position);
    } else if (!MotorControl_RequestMatches(motors, count, check_position)) {
        LOG_WARN("MOTOR", "batch request rejected: another batch is active\r\n");
        return MODBUS_RESULT_BUSY;
    }

    switch (motor_batch.phase) {
        case MOTOR_BATCH_PHASE_READ_START:
            result = MotorControl_ReadCurrentPosition(&current_position);
            if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
                return result;
            }
            if (result != MODBUS_RESULT_OK) {
                motor_batch.results[motor_batch.index] = result;
                LOG_ERROR("MOTOR",
                    "initial position read failed: slave=0x%02X, reg=%u, result=%u\r\n",
                    (unsigned int)motor_batch.motors[motor_batch.index].slave_addr,
                    (unsigned int)MotorControl_GetPositionRegister(
                        &motor_batch.motors[motor_batch.index]),
                    (unsigned int)result);
                return MotorControl_FinishBatch(result, results);
            }

            /* 0x00CE 下发相对行程，因此目标为起始位置加本次位移。 */
            motor_batch.target_positions[motor_batch.index] =
                (int32_t)((uint32_t)current_position +
                          (uint32_t)MotorControl_CommandDelta(
                              &motor_batch.motors[motor_batch.index]));
            LOG_DEBUG("MOTOR",
                "target calculated: slave=0x%02X, position_reg=%u, current=%ld, delta=%ld, target=%ld\r\n",
                (unsigned int)motor_batch.motors[motor_batch.index].slave_addr,
                (unsigned int)MotorControl_GetPositionRegister(
                    &motor_batch.motors[motor_batch.index]),
                (long)current_position,
                (long)MotorControl_CommandDelta(
                    &motor_batch.motors[motor_batch.index]),
                (long)motor_batch.target_positions[motor_batch.index]);
            motor_batch.index++;
            if (motor_batch.index >= motor_batch.count) {
                motor_batch.index = 0U;
                motor_batch.phase = MOTOR_BATCH_PHASE_WRITE;
            }
            return MODBUS_RESULT_PENDING;

        case MOTOR_BATCH_PHASE_WRITE:
            commands[0] = motor_batch.motors[motor_batch.index].value_low_word;
            commands[1] = motor_batch.motors[motor_batch.index].value_high_word;
            result = ModbusMaster_10_WriteMultiReg(
                MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
                motor_batch.motors[motor_batch.index].slave_addr,
                motor_batch.motors[motor_batch.index].register_address,
                2U, commands);
            if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
                return result;
            }
            motor_batch.results[motor_batch.index] = result;
            LOG_INFO("MOTOR",
                "move command: slave=0x%02X, reg=0x%04X, low=0x%04X, high=0x%04X, result=%u\r\n",
                (unsigned int)motor_batch.motors[motor_batch.index].slave_addr,
                (unsigned int)motor_batch.motors[motor_batch.index].register_address,
                (unsigned int)commands[0], (unsigned int)commands[1],
                (unsigned int)result);
            if (result != MODBUS_RESULT_OK) {
                if (motor_batch.check_position) {
                    return MotorControl_FinishBatch(result, results);
                }
                motor_batch.failures++;
                if (motor_batch.first_failure == MODBUS_RESULT_OK) {
                    motor_batch.first_failure = result;
                }
            }
            motor_batch.index++;
            if (motor_batch.index < motor_batch.count) {
                return MODBUS_RESULT_PENDING;
            }

            if (!motor_batch.check_position) {
                result = motor_batch.failures == 0U
                         ? MODBUS_RESULT_OK
                         : motor_batch.first_failure;
                return MotorControl_FinishBatch(result, results);
            }

            now = GetTick();
            motor_batch.index = 0U;
            motor_batch.next_poll_tick =
                now + MOTOR_CONTROL_POSITION_POLL_INTERVAL_MS;
            motor_batch.verify_deadline = now + MOTOR_CONTROL_MOVE_TIMEOUT_MS;
            motor_batch.phase = MOTOR_BATCH_PHASE_WAIT_POLL;
            return MODBUS_RESULT_PENDING;

        case MOTOR_BATCH_PHASE_WAIT_POLL:
            now = GetTick();
            if (MotorControl_IsTimeReached(now, motor_batch.verify_deadline)) {
                motor_batch.results[motor_batch.index] = MODBUS_RESULT_TIMEOUT;
                LOG_ERROR("MOTOR",
                    "position timeout: slave=0x%02X, target=%ld\r\n",
                    (unsigned int)motor_batch.motors[motor_batch.index].slave_addr,
                    (long)motor_batch.target_positions[motor_batch.index]);
                return MotorControl_FinishBatch(MODBUS_RESULT_TIMEOUT, results);
            }
            if (!MotorControl_IsTimeReached(now, motor_batch.next_poll_tick)) {
                return MODBUS_RESULT_PENDING;
            }
            motor_batch.phase = MOTOR_BATCH_PHASE_VERIFY;
            return MODBUS_RESULT_PENDING;

        case MOTOR_BATCH_PHASE_VERIFY:
            result = MotorControl_ReadCurrentPosition(&current_position);
            if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
                return result;
            }
            if (result != MODBUS_RESULT_OK) {
                motor_batch.results[motor_batch.index] = result;
                LOG_ERROR("MOTOR",
                    "position verify failed: slave=0x%02X, reg=%u, result=%u\r\n",
                    (unsigned int)motor_batch.motors[motor_batch.index].slave_addr,
                    (unsigned int)MotorControl_GetPositionRegister(
                        &motor_batch.motors[motor_batch.index]),
                    (unsigned int)result);
                return MotorControl_FinishBatch(result, results);
            }

            if (MotorControl_PositionReached(
                    current_position,
                    motor_batch.target_positions[motor_batch.index])) {
                motor_batch.results[motor_batch.index] = MODBUS_RESULT_OK;
                LOG_INFO("MOTOR",
                    "position reached: slave=0x%02X, current=%ld, target=%ld\r\n",
                    (unsigned int)motor_batch.motors[motor_batch.index].slave_addr,
                    (long)current_position,
                    (long)motor_batch.target_positions[motor_batch.index]);
                motor_batch.index++;
                if (motor_batch.index >= motor_batch.count) {
                    return MotorControl_FinishBatch(MODBUS_RESULT_OK, results);
                }
                /* 其他电机同时运动，直接检查批次中的下一台。 */
                return MODBUS_RESULT_PENDING;
            }

            LOG_DEBUG("MOTOR",
                "position pending: slave=0x%02X, current=%ld, target=%ld\r\n",
                (unsigned int)motor_batch.motors[motor_batch.index].slave_addr,
                (long)current_position,
                (long)motor_batch.target_positions[motor_batch.index]);

            now = GetTick();
            if (MotorControl_IsTimeReached(now, motor_batch.verify_deadline)) {
                motor_batch.results[motor_batch.index] = MODBUS_RESULT_TIMEOUT;
                LOG_ERROR("MOTOR",
                    "position timeout: slave=0x%02X, current=%ld, target=%ld\r\n",
                    (unsigned int)motor_batch.motors[motor_batch.index].slave_addr,
                    (long)current_position,
                    (long)motor_batch.target_positions[motor_batch.index]);
                return MotorControl_FinishBatch(MODBUS_RESULT_TIMEOUT, results);
            }
            motor_batch.next_poll_tick =
                now + MOTOR_CONTROL_POSITION_POLL_INTERVAL_MS;
            motor_batch.phase = MOTOR_BATCH_PHASE_WAIT_POLL;
            return MODBUS_RESULT_PENDING;

        default:
            return MotorControl_FinishBatch(MODBUS_RESULT_PARAM, results);
    }
}

uint8_t MotorControl_BatchMove(const MotorControlParams *motors, uint8_t count,
                               uint8_t *results)
{
    return MotorControl_BatchMoveInternal(motors, count, results, 1U);
}

uint8_t MotorControl_BatchMoveNoPositionCheck(
    const MotorControlParams *motors, uint8_t count, uint8_t *results)
{
    return MotorControl_BatchMoveInternal(motors, count, results, 0U);
}

uint8_t ModbusBatch_IsBusy(void)
{
    return motor_batch.active;
}

void ModbusBatch_Cancel(void)
{
    memset(&motor_batch, 0, sizeof(motor_batch));
}
