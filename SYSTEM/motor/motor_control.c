#include "motor_control.h"
#include "sequence_steps.h"
#include "modbus_common.h"
#include "modbus_master.h"
#include "tick.h"
#include "debug_log.h"

#include <stddef.h>
#include <string.h>

#define MODBUS_BATCH_MAX_MOTORS 8U
#define MOTOR_HOME_MAX_MOTORS   10U

typedef enum {
    MOTOR_BATCH_PHASE_IDLE = 0,
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

typedef struct {
    MotorHomeState state;
    uint8_t motor_count;
    uint8_t addresses[MOTOR_HOME_MAX_MOTORS];
    uint8_t command_index;
    uint8_t status_index;
    uint8_t current_slave;
    uint8_t failed_slave;
    uint8_t last_error;
    uint8_t motor2_gate_active;
    uint16_t speed_command;
    uint16_t status_words[MOTOR_HOME_STATUS_REG_COUNT];
    uint16_t last_status_word;
    uint16_t completed_mask;
    uint32_t next_poll_tick;
    uint32_t timeout_deadline;
} MotorHomeContext;

static MotorBatchContext motor_batch;
static MotorHomeContext motor_home;
static uint8_t motor2_home_completed;

/*
 * 夹紧电机和舱门电机使用不同协议，不参与本批次回原点。
 * Motor2 必须排在 Motor1 前面；状态机还会等待 Motor2 完成后才释放 Motor1。
 */
static const uint8_t motor_home_addresses[MOTOR_HOME_MAX_MOTORS] = {
    MOTOR2_SLAVE_ADDR,
    MOTOR1_SLAVE_ADDR,
    MOTOR5_SLAVE_ADDR,
    MOTOR6_SLAVE_ADDR,
    MOTOR7_SLAVE_ADDR,
    MOTOR8_SLAVE_ADDR,
    MOTOR9_SLAVE_ADDR,
    MOTOR10_SLAVE_ADDR,
    MOTOR11_SLAVE_ADDR,
    MOTOR12_SLAVE_ADDR
};

static uint16_t MotorControl_GetPositionRegister(
    const MotorControlParams *motor)
{
    if (motor->position_register_address != 0U) {
        return motor->position_register_address;
    }
    if (motor->register_address == MOTOR_CONTROL_ABSOLUTE_COMMAND_REG) {
        return MOTOR_CONTROL_DRIVER_POSITION_REG;
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

static int32_t MotorControl_CommandPosition(const MotorControlParams *motor)
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
        if (position_register == MOTOR_CONTROL_DRIVER_POSITION_REG) {
            /* This driver returns 0x0004 high word, then 0x0005 low word. */
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
uint8_t MotorControl_HomeIsBusy(void)
{
    return (motor_home.state == MOTOR_HOME_STATE_SEND_COMMAND ||
            motor_home.state == MOTOR_HOME_STATE_WAIT_POLL ||
            motor_home.state == MOTOR_HOME_STATE_READ_STATUS) ? 1U : 0U;
}

static void MotorControl_HomeFail(uint8_t slave_addr, uint8_t error)
{
    if (slave_addr == MOTOR2_SLAVE_ADDR) {
        motor2_home_completed = 0U;
    }
    motor_home.failed_slave = slave_addr;
    motor_home.current_slave = slave_addr;
    motor_home.last_error = error;
    motor_home.state = MOTOR_HOME_STATE_FAILED;
    LOG_ERROR("MOTOR",
              "homing failed: slave=0x%02X, result=%u, status=0x%04X\r\n",
              (unsigned int)slave_addr, (unsigned int)error,
              (unsigned int)motor_home.last_status_word);
}

static uint16_t MotorControl_HomeAllMask(void)
{
    return (uint16_t)((1UL << motor_home.motor_count) - 1UL);
}

static void MotorControl_HomeSelectFirstIncomplete(void)
{
    motor_home.status_index = 0U;
    while (motor_home.status_index < motor_home.motor_count &&
           (motor_home.completed_mask &
            (uint16_t)(1UL << motor_home.status_index)) != 0U) {
        motor_home.status_index++;
    }
}

static uint8_t MotorControl_HomeStartInternal(const uint8_t *addresses,
                                              uint8_t count,
                                              uint16_t speed_command)
{
    uint8_t index;
    uint8_t includes_motor1 = 0U;
    uint8_t includes_motor2 = 0U;

    if (MotorControl_HomeIsBusy()) {
        return MODBUS_RESULT_BUSY;
    }
    if (motor_batch.active || ModbusMaster_IsBusy()) {
        return MODBUS_RESULT_BUSY;
    }
    if (speed_command != MOTOR_HOME_SPEED_SLOW &&
        speed_command != MOTOR_HOME_SPEED_NORMAL &&
        speed_command != MOTOR_HOME_SPEED_FAST) {
        return MODBUS_RESULT_PARAM;
    }
    if (addresses == NULL || count == 0U || count > MOTOR_HOME_MAX_MOTORS) {
        return MODBUS_RESULT_PARAM;
    }

    for (index = 0U; index < count; index++) {
        if (addresses[index] == MOTOR1_SLAVE_ADDR) {
            includes_motor1 = 1U;
        } else if (addresses[index] == MOTOR2_SLAVE_ADDR) {
            includes_motor2 = 1U;
        }
    }
    if (includes_motor1 && !includes_motor2 && !motor2_home_completed) {
        LOG_WARN("MOTOR",
                 "Motor1 homing blocked until Motor2 homing completes\r\n");
        return MODBUS_RESULT_BUSY;
    }
    if (includes_motor2) {
        motor2_home_completed = 0U;
    }

    memset(&motor_home, 0, sizeof(motor_home));
    motor_home.state = MOTOR_HOME_STATE_SEND_COMMAND;
    motor_home.motor_count = count;
    memcpy(motor_home.addresses, addresses, count);
    motor_home.speed_command = speed_command;
    motor_home.current_slave = motor_home.addresses[0];
    LOG_INFO("MOTOR", "homing started: count=%u, speed_cmd=0x%04X\r\n",
             (unsigned int)motor_home.motor_count,
             (unsigned int)speed_command);
    return MODBUS_RESULT_OK;
}

uint8_t MotorControl_HomeStart(uint16_t speed_command)
{
    return MotorControl_HomeStartInternal(
        motor_home_addresses,
        (uint8_t)(sizeof(motor_home_addresses) /
                  sizeof(motor_home_addresses[0])),
        speed_command);
}

uint8_t MotorControl_HomeStartSingle(uint8_t slave_addr,
                                     uint16_t speed_command)
{
    uint8_t index;

    for (index = 0U;
         index < (uint8_t)(sizeof(motor_home_addresses) /
                           sizeof(motor_home_addresses[0]));
         index++) {
        if (motor_home_addresses[index] == slave_addr) {
            return MotorControl_HomeStartInternal(&slave_addr, 1U,
                                                  speed_command);
        }
    }
    return MODBUS_RESULT_PARAM;
}

void MotorControl_HomeProcess(void)
{
    uint8_t result;
    uint8_t slave_addr;
    uint32_t now;

    if (!MotorControl_HomeIsBusy()) {
        return;
    }

    if (motor_home.state == MOTOR_HOME_STATE_SEND_COMMAND) {
        slave_addr = motor_home.addresses[motor_home.command_index];
        motor_home.current_slave = slave_addr;
        result = ModbusMaster_10_WriteMultiReg(
            MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
            slave_addr,
            MOTOR_HOME_COMMAND_REG,
            1U,
            &motor_home.speed_command);
        if (result == MODBUS_RESULT_PENDING ||
            result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_HomeFail(slave_addr, result);
            return;
        }

        LOG_INFO("MOTOR", "homing command accepted: slave=0x%02X\r\n",
                 (unsigned int)slave_addr);
        motor_home.command_index++;
        if (slave_addr == MOTOR2_SLAVE_ADDR &&
            motor_home.command_index < motor_home.motor_count &&
            motor_home.addresses[motor_home.command_index] ==
                MOTOR1_SLAVE_ADDR) {
            now = GetTick();
            motor_home.motor2_gate_active = 1U;
            motor_home.status_index =
                (uint8_t)(motor_home.command_index - 1U);
            motor_home.next_poll_tick =
                now + MOTOR_HOME_POLL_INTERVAL_MS;
            motor_home.timeout_deadline = now + MOTOR_HOME_TIMEOUT_MS;
            motor_home.state = MOTOR_HOME_STATE_WAIT_POLL;
            LOG_INFO("MOTOR",
                     "waiting for Motor2 homing before starting Motor1\r\n");
        } else if (motor_home.command_index >= motor_home.motor_count) {
            now = GetTick();
            MotorControl_HomeSelectFirstIncomplete();
            motor_home.next_poll_tick = now + MOTOR_HOME_POLL_INTERVAL_MS;
            motor_home.timeout_deadline = now + MOTOR_HOME_TIMEOUT_MS;
            motor_home.state = MOTOR_HOME_STATE_WAIT_POLL;
        }
        return;
    }

    now = GetTick();
    if (MotorControl_IsTimeReached(now, motor_home.timeout_deadline)) {
        slave_addr = motor_home.addresses[motor_home.status_index];
        /* 若全局超时发生在一次状态读取途中，必须释放主站事务。 */
        ModbusMaster_Cancel();
        MotorControl_HomeFail(slave_addr, MODBUS_RESULT_TIMEOUT);
        return;
    }

    if (motor_home.state == MOTOR_HOME_STATE_WAIT_POLL) {
        if (!MotorControl_IsTimeReached(now, motor_home.next_poll_tick)) {
            return;
        }
        motor_home.state = MOTOR_HOME_STATE_READ_STATUS;
        return;
    }

    if (motor_home.state == MOTOR_HOME_STATE_READ_STATUS) {
        slave_addr = motor_home.addresses[motor_home.status_index];
        motor_home.current_slave = slave_addr;
        motor_home.last_status_word = 0U;
        result = ModbusMaster_03_ReadHoldReg(
            MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
            slave_addr,
            MOTOR_HOME_STATUS_REG,
            MOTOR_HOME_STATUS_REG_COUNT,
            motor_home.status_words);
        if (result == MODBUS_RESULT_PENDING ||
            result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_HomeFail(slave_addr, result);
            return;
        }

        /* 文档约定只判断状态低字的 bit15，其他位不参与完成判定。 */
        motor_home.last_status_word = motor_home.status_words[0];
        if ((motor_home.last_status_word & MOTOR_HOME_COMPLETE_MASK) != 0U) {
            motor_home.completed_mask |=
                (uint16_t)(1UL << motor_home.status_index);
            if (slave_addr == MOTOR2_SLAVE_ADDR) {
                motor2_home_completed = 1U;
            }
            LOG_INFO("MOTOR",
                     "homing completed: slave=0x%02X, status=0x%04X\r\n",
                     (unsigned int)slave_addr,
                     (unsigned int)motor_home.last_status_word);
        } else {
            LOG_DEBUG("MOTOR",
                      "homing pending: slave=0x%02X, status=0x%04X\r\n",
                      (unsigned int)slave_addr,
                      (unsigned int)motor_home.last_status_word);
        }

        if (motor_home.motor2_gate_active) {
            if ((motor_home.completed_mask &
                 (uint16_t)(1UL << motor_home.status_index)) != 0U) {
                motor_home.motor2_gate_active = 0U;
                motor_home.current_slave =
                    motor_home.addresses[motor_home.command_index];
                motor_home.state = MOTOR_HOME_STATE_SEND_COMMAND;
                LOG_INFO("MOTOR",
                         "Motor2 homed; Motor1 homing is now enabled\r\n");
            } else {
                motor_home.next_poll_tick =
                    GetTick() + MOTOR_HOME_POLL_INTERVAL_MS;
                motor_home.state = MOTOR_HOME_STATE_WAIT_POLL;
            }
            return;
        }

        motor_home.status_index++;
        while (motor_home.status_index < motor_home.motor_count &&
               (motor_home.completed_mask &
                (uint16_t)(1UL << motor_home.status_index)) != 0U) {
            motor_home.status_index++;
        }

        if (motor_home.completed_mask == MotorControl_HomeAllMask()) {
            motor_home.state = MOTOR_HOME_STATE_SUCCESS;
            LOG_INFO("MOTOR", "all motors homed: mask=0x%04X\r\n",
                     (unsigned int)motor_home.completed_mask);
            return;
        }

        if (motor_home.status_index >= motor_home.motor_count) {
            MotorControl_HomeSelectFirstIncomplete();
            motor_home.next_poll_tick =
                GetTick() + MOTOR_HOME_POLL_INTERVAL_MS;
            motor_home.state = MOTOR_HOME_STATE_WAIT_POLL;
        }
    }
}

MotorHomeState MotorControl_HomeGetState(void)
{
    return motor_home.state;
}

uint8_t MotorControl_HomeGetCurrentSlave(void)
{
    return motor_home.current_slave;
}

uint8_t MotorControl_HomeGetFailedSlave(void)
{
    return motor_home.failed_slave;
}

uint8_t MotorControl_HomeGetLastError(void)
{
    return motor_home.last_error;
}

uint16_t MotorControl_HomeGetStatusWord(void)
{
    return motor_home.last_status_word;
}

uint16_t MotorControl_HomeGetCompletedMask(void)
{
    return motor_home.completed_mask;
}

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
    uint16_t position_words[2];

    if (MotorControl_HomeIsBusy()) {
        return MODBUS_RESULT_BUSY;
    }
    if (motor_num != 1U) {
        LOG_ERROR("MOTOR", "single write rejected: slave=0x%02X, motor_num=%u\r\n",
                          (unsigned int)slave_addr,
                          (unsigned int)motor_num);
        return MODBUS_RESULT_PARAM;
    }
    position_words[0] = motor_cmd;
    position_words[1] = 0U;
    result = ModbusMaster_10_WriteMultiReg(
        MODBUS_MASTER_CLIENT_MOTOR_CONTROL, slave_addr, MOTOR1_CTRL_REG1,
        2U, position_words);
    if (result != MODBUS_RESULT_PENDING && result != MODBUS_RESULT_BUSY) {
        LOG_INFO("MOTOR",
            "single absolute write: slave=0x%02X, reg=0x%04X, value=0x%04X, result=%u\r\n",
            (unsigned int)slave_addr, (unsigned int)MOTOR1_CTRL_REG1,
            (unsigned int)motor_cmd, (unsigned int)result);
    }
    return result;
}

uint8_t Motor_Control(uint8_t motor_id, uint8_t reg_num, uint16_t motor_cmd)
{
    uint8_t slave_addr;
    uint8_t result;
    uint8_t write_absolute_position = 0U;
    uint16_t reg_addr;
    uint16_t position_words[2];

    if (MotorControl_HomeIsBusy()) {
        return MODBUS_RESULT_BUSY;
    }
    switch (motor_id) {
        case MOTOR_ID_1:
            if (reg_num != 1U) return MODBUS_RESULT_PARAM;
            slave_addr = MOTOR1_SLAVE_ADDR;
            reg_addr = MOTOR1_CTRL_REG1;
            write_absolute_position = 1U;
            break;
        case MOTOR_ID_2:
            if (reg_num != 1U) return MODBUS_RESULT_PARAM;
            slave_addr = MOTOR2_SLAVE_ADDR;
            reg_addr = MOTOR2_CTRL_REG1;
            write_absolute_position = 1U;
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
    if (write_absolute_position) {
        position_words[0] = motor_cmd;
        position_words[1] = 0U;
        result = ModbusMaster_10_WriteMultiReg(
            MODBUS_MASTER_CLIENT_MOTOR_CONTROL, slave_addr, reg_addr,
            2U, position_words);
    } else {
        result = ModbusMaster_06_WriteSingleReg(
            MODBUS_MASTER_CLIENT_MOTOR_CONTROL, slave_addr, reg_addr,
            motor_cmd);
    }
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

    if (MotorControl_HomeIsBusy()) {
        return MODBUS_RESULT_BUSY;
    }
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
    return (uint8_t)(ModbusMaster_IsBusy() || motor_batch.active ||
                     MotorControl_HomeIsBusy());
}

void MotorControl_Cancel(void)
{
    if (MotorControl_HomeIsBusy()) {
        LOG_WARN("MOTOR", "cancel rejected while homing is active\r\n");
        return;
    }
    LOG_WARN("MOTOR", "control canceled\r\n");
    ModbusBatch_Cancel();
    ModbusMaster_Cancel();
}

uint8_t MotorControl_WriteTarget(MotorControlTarget target, uint16_t value)
{
    if (MotorControl_HomeIsBusy()) {
        return MODBUS_RESULT_BUSY;
    }
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
    uint8_t index;
    int32_t current_position;
    uint32_t now;

    if (MotorControl_HomeIsBusy()) {
        return MODBUS_RESULT_BUSY;
    }
    if (motors == NULL || count == 0U || count > MODBUS_BATCH_MAX_MOTORS) {
        LOG_ERROR("MOTOR", "batch rejected: motors=%p, count=%u\r\n",
                          (const void *)motors, (unsigned int)count);
        return MODBUS_RESULT_PARAM;
    }
    for (index = 0U; index < count; index++) {
        if (motors[index].register_address !=
            MOTOR_CONTROL_ABSOLUTE_COMMAND_REG) {
            LOG_ERROR("MOTOR",
                "absolute batch rejected: slave=0x%02X, reg=0x%04X\r\n",
                (unsigned int)motors[index].slave_addr,
                (unsigned int)motors[index].register_address);
            return MODBUS_RESULT_PARAM;
        }
    }
    if (!motor_batch.active) {
        memcpy(motor_batch.motors, motors,
               count * sizeof(MotorControlParams));
        memset(motor_batch.results, MODBUS_RESULT_PENDING,
               count * sizeof(motor_batch.results[0]));
        motor_batch.active = 1U;
        motor_batch.check_position = check_position;
        motor_batch.phase = MOTOR_BATCH_PHASE_WRITE;
        motor_batch.count = count;
        motor_batch.index = 0U;
        motor_batch.failures = 0U;
        motor_batch.first_failure = MODBUS_RESULT_OK;
        for (index = 0U; index < count; index++) {
            motor_batch.target_positions[index] =
                MotorControl_CommandPosition(&motor_batch.motors[index]);
            LOG_DEBUG("MOTOR",
                "absolute target prepared: slave=0x%02X, target=%ld\r\n",
                (unsigned int)motor_batch.motors[index].slave_addr,
                (long)motor_batch.target_positions[index]);
        }
        LOG_INFO("MOTOR", "batch started: count=%u, position_check=%u\r\n",
                          (unsigned int)count,
                          (unsigned int)check_position);
    } else if (!MotorControl_RequestMatches(motors, count, check_position)) {
        LOG_WARN("MOTOR", "batch request rejected: another batch is active\r\n");
        return MODBUS_RESULT_BUSY;
    }

    switch (motor_batch.phase) {
        case MOTOR_BATCH_PHASE_WRITE:
            /* 0x00D0 为低字，0x00D1 为高字。 */
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
                "absolute move: slave=0x%02X, reg=0x%04X, low=0x%04X, high=0x%04X, result=%u\r\n",
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
