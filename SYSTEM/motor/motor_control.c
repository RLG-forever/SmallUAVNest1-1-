#include "motor_control.h"
#include "sequence_steps.h"
#include "modbus_common.h"
#include "modbus_master.h"
#include "tick.h"
#include "debug_log.h"
#include "battery_swap.h"

#include <stddef.h>
#include <string.h>

#define MODBUS_BATCH_MAX_MOTORS 8U
#define MOTOR_HOME_MAX_MOTORS   10U

typedef enum {
    MOTOR_BATCH_PHASE_IDLE = 0,
    MOTOR_BATCH_PHASE_WRITE,
    MOTOR_BATCH_PHASE_WAIT_POLL,
    MOTOR_BATCH_PHASE_VERIFY_POSITION,
    MOTOR_BATCH_PHASE_VERIFY_ALARM
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
    uint8_t reached_mask;
    uint16_t position_words[MOTOR_CONTROL_POSITION_REG_COUNT];
    uint16_t alarm_status_word;
    uint32_t next_poll_tick;
    uint32_t verify_deadline;
    int32_t current_position;
    int32_t target_positions[MODBUS_BATCH_MAX_MOTORS];
    int32_t reached_positions[MODBUS_BATCH_MAX_MOTORS];
    uint8_t results[MODBUS_BATCH_MAX_MOTORS];
    MotorControlParams motors[MODBUS_BATCH_MAX_MOTORS];
} MotorBatchContext;

typedef struct {
    MotorHomeState state;
    uint8_t motor_count;
    uint8_t addresses[MOTOR_HOME_MAX_MOTORS];
    uint8_t precheck_count;
    uint8_t precheck_addresses[MOTOR_HOME_MAX_MOTORS];
    uint8_t precheck_index;
    uint8_t pair_index;
    uint8_t command_index;
    uint8_t status_index;
    uint8_t current_slave;
    uint8_t failed_slave;
    uint8_t last_error;
    uint8_t motor2_gate_active;
    uint8_t level_in_progress;
    uint8_t level_stable_count;
    uint16_t saved_position_valid_mask;
    uint16_t driver_baseline_mask;
    uint16_t speed_command;
    uint16_t alarm_status_word;
    uint16_t position_words[MOTOR_CONTROL_POSITION_REG_COUNT];
    uint16_t relative_position_words[2];
    uint16_t status_words[MOTOR_HOME_STATUS_REG_COUNT];
    uint16_t last_status_word;
    uint16_t completed_mask;
    int32_t pair_first_position;
    int32_t pair_second_position;
    int32_t relative_adjustment;
    int32_t level_target;
    int32_t saved_positions[MOTOR_POSITION_STORE_COUNT];
    int32_t driver_baselines[MOTOR_POSITION_STORE_COUNT];
    uint32_t next_poll_tick;
    uint32_t timeout_deadline;
} MotorHomeContext;

typedef struct {
    MotorPreHomeState state;
    uint8_t saved_position_valid;
    uint8_t move_mask;
    uint8_t stable_sample_count;
    uint8_t alarm_index;
    uint8_t current_slave;
    uint8_t failed_slave;
    uint8_t last_error;
    uint16_t alarm_status_word;
    int32_t saved_positions[2];
    int32_t initial_positions[2];
    int32_t last_positions[2];
    int32_t sampled_positions[2];
    uint32_t relative_distances[2];
    uint16_t position_words[MOTOR_CONTROL_POSITION_REG_COUNT];
    uint32_t next_poll_tick;
    uint32_t timeout_deadline;
} MotorPreHomeContext;

typedef struct {
    MotorFullHomeState state;
    uint8_t saved_position_valid;
    uint8_t current_slave;
    uint8_t failed_slave;
    uint8_t last_error;
    uint8_t motor2_was_homed;
    int32_t saved_positions[2];
    uint16_t speed_command;
    uint16_t status_words[MOTOR_HOME_STATUS_REG_COUNT];
} MotorFullHomeContext;

static MotorBatchContext motor_batch;
static MotorHomeContext motor_home;
static MotorPreHomeContext motor_prehome;
static MotorFullHomeContext motor_full_home;
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

/* 开机回零第一阶段：这些电机始终先完成回零。 */
static const uint8_t motor_home_before_prehome_addresses[] = {
    MOTOR5_SLAVE_ADDR,
    MOTOR6_SLAVE_ADDR,
    MOTOR9_SLAVE_ADDR,
    MOTOR10_SLAVE_ADDR,
    MOTOR11_SLAVE_ADDR,
    MOTOR12_SLAVE_ADDR
};

/* 电机7/8回退稳定后，先等待其他电机完成回零。 */
static const uint8_t motor_home_after_prehome_addresses[] = {
    MOTOR2_SLAVE_ADDR,
    MOTOR1_SLAVE_ADDR
};

/* 所有其他电机完成后，最后单独回零电机7/8。 */
static const uint8_t motor_home_motor78_addresses[] = {
    MOTOR7_SLAVE_ADDR,
    MOTOR8_SLAVE_ADDR
};

static const uint8_t motor_home_level_pairs[MOTOR_HOME_LEVEL_PAIR_COUNT][2] = {
    {MOTOR5_SLAVE_ADDR, MOTOR6_SLAVE_ADDR},
    {MOTOR7_SLAVE_ADDR, MOTOR8_SLAVE_ADDR},
    {MOTOR9_SLAVE_ADDR, MOTOR10_SLAVE_ADDR},
    {MOTOR11_SLAVE_ADDR, MOTOR12_SLAVE_ADDR}
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

static int64_t MotorControl_AbsI64(int64_t value)
{
    return value < 0 ? -value : value;
}

static uint8_t MotorControl_HomePrecheckContains(uint8_t slave_addr)
{
    uint8_t index;

    for (index = 0U; index < motor_home.precheck_count; index++) {
        if (motor_home.precheck_addresses[index] == slave_addr) {
            return 1U;
        }
    }
    return 0U;
}

static uint8_t MotorControl_HomeReadPosition(uint8_t slave_addr,
                                             int32_t *position)
{
    uint8_t result;
    uint8_t saved_index;
    uint16_t baseline_bit;
    uint16_t saved_pair_mask;
    int32_t driver_position;
    int64_t restored_position;

    result = ModbusMaster_03_ReadHoldReg(
        MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
        slave_addr,
        MOTOR_CONTROL_DRIVER_POSITION_REG,
        MOTOR_CONTROL_POSITION_REG_COUNT,
        motor_home.position_words);
    if (result == MODBUS_RESULT_OK) {
        /* 驱动器返回低字在前、高字在后。 */
        driver_position = MotorControl_CombinePosition(
            motor_home.position_words[1], motor_home.position_words[0]);
        if (slave_addr >= MOTOR_POSITION_STORE_FIRST_SLAVE &&
            slave_addr < MOTOR_POSITION_STORE_FIRST_SLAVE +
                         MOTOR_POSITION_STORE_COUNT) {
            saved_index = (uint8_t)(
                slave_addr - MOTOR_POSITION_STORE_FIRST_SLAVE);
            baseline_bit = (uint16_t)(1U << saved_index);
        } else {
            saved_index = 0U;
            baseline_bit = 0U;
        }
        saved_pair_mask = baseline_bit == 0U
                          ? 0U
                          : (uint16_t)(3U << (saved_index & 0xFEU));
        if (baseline_bit != 0U &&
            (motor_home.saved_position_valid_mask & saved_pair_mask) ==
                saved_pair_mask) {
            if ((motor_home.driver_baseline_mask & baseline_bit) == 0U) {
                motor_home.driver_baselines[saved_index] =
                    driver_position;
                motor_home.driver_baseline_mask |= baseline_bit;
            }
            restored_position =
                (int64_t)motor_home.saved_positions[saved_index] +
                ((int64_t)driver_position -
                 (int64_t)motor_home.driver_baselines[saved_index]);
            if (restored_position > (int64_t)INT32_MAX ||
                restored_position < (int64_t)INT32_MIN) {
                return MODBUS_RESULT_PARAM;
            }
            *position = (int32_t)restored_position;
        } else {
            *position = driver_position;
        }
    }
    return result;
}

static uint8_t MotorControl_PreHomePositionStable(int32_t current,
                                                  int32_t previous)
{
    int64_t difference = (int64_t)current - (int64_t)previous;

    if (difference < 0) {
        difference = -difference;
    }
    return (uint8_t)(difference <= MOTOR_PREHOME_STABLE_TOLERANCE);
}

static uint8_t MotorControl_PreHomeReadPosition(uint8_t slave_addr,
                                                int32_t *position)
{
    uint8_t result;

    result = ModbusMaster_03_ReadHoldReg(
        MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
        slave_addr,
        MOTOR_CONTROL_DRIVER_POSITION_REG,
        MOTOR_CONTROL_POSITION_REG_COUNT,
        motor_prehome.position_words);
    if (result == MODBUS_RESULT_OK) {
        /* 驱动器返回低字在前、高字在后。 */
        *position = MotorControl_CombinePosition(
            motor_prehome.position_words[1],
            motor_prehome.position_words[0]);
    }
    return result;
}

static uint8_t MotorControl_PreHomeWriteRelative(uint8_t index)
{
    uint16_t commands[2];
    uint32_t distance = motor_prehome.relative_distances[index];
    uint8_t slave_addr = index == 0U
                         ? MOTOR7_SLAVE_ADDR
                         : MOTOR8_SLAVE_ADDR;

    commands[0] = (uint16_t)(distance & 0xFFFFUL);
    commands[1] = (uint16_t)((distance >> 16) & 0xFFFFUL);
    return ModbusMaster_10_WriteMultiReg(
        MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
        slave_addr,
        MOTOR_CONTROL_RELATIVE_COMMAND_REG,
        2U,
        commands);
}

static void MotorControl_PreHomeFail(uint8_t slave_addr, uint8_t error)
{
    motor_prehome.failed_slave = slave_addr;
    motor_prehome.current_slave = slave_addr;
    motor_prehome.last_error = error;
    motor_prehome.state = MOTOR_PREHOME_STATE_FAILED;
    LOG_ERROR("PREHOME", "failed: slave=0x%02X, result=%u\r\n",
              (unsigned int)slave_addr, (unsigned int)error);
}

static void MotorControl_PreHomeSetReady(void)
{
    const uint8_t slave_addrs[2] = {
        MOTOR7_SLAVE_ADDR, MOTOR8_SLAVE_ADDR
    };
    int32_t positions[2];
    int64_t restored_position;
    uint8_t index;

    for (index = 0U; index < 2U; index++) {
        if (motor_prehome.saved_position_valid) {
            restored_position =
                (int64_t)motor_prehome.saved_positions[index] +
                (int64_t)motor_prehome.relative_distances[index];
            if (restored_position > (int64_t)INT32_MAX ||
                restored_position < (int64_t)INT32_MIN) {
                MotorControl_PreHomeFail(slave_addrs[index],
                                         MODBUS_RESULT_PARAM);
                return;
            }
            positions[index] = (int32_t)restored_position;
        } else {
            positions[index] = motor_prehome.initial_positions[index];
        }
    }
    if (!MotorPositionStore_UpdateBatch(slave_addrs, positions, 2U)) {
        MotorControl_PreHomeFail(motor_prehome.current_slave,
                                 MODBUS_RESULT_ECHO);
        return;
    }

    motor_prehome.state = MOTOR_PREHOME_STATE_READY;
    motor_prehome.current_slave = 0U;
    LOG_INFO("PREHOME", "ready for normal homing\r\n");
}

static void MotorControl_PreHomeBeginStableCheck(void)
{
    uint32_t now = GetTick();

    motor_prehome.last_positions[0] = motor_prehome.initial_positions[0];
    motor_prehome.last_positions[1] = motor_prehome.initial_positions[1];
    motor_prehome.stable_sample_count = 0U;
    motor_prehome.next_poll_tick = now + MOTOR_PREHOME_POLL_INTERVAL_MS;
    motor_prehome.timeout_deadline = now + MOTOR_PREHOME_TIMEOUT_MS;
    motor_prehome.state = MOTOR_PREHOME_STATE_WAIT_POLL;
}

static void MotorControl_PreHomeAdvanceAlarmCheck(void)
{
    if (motor_prehome.alarm_index == 0U) {
        motor_prehome.alarm_index = 1U;
        motor_prehome.current_slave = MOTOR8_SLAVE_ADDR;
        motor_prehome.state = MOTOR_PREHOME_STATE_CHECK_ALARM;
    } else {
        motor_prehome.current_slave = MOTOR7_SLAVE_ADDR;
        motor_prehome.state = MOTOR_PREHOME_STATE_READ_INITIAL_MOTOR7;
    }
}

uint8_t MotorControl_PreHomeIsBusy(void)
{
    return (uint8_t)(
        motor_prehome.state != MOTOR_PREHOME_STATE_IDLE &&
        motor_prehome.state != MOTOR_PREHOME_STATE_READY &&
        motor_prehome.state != MOTOR_PREHOME_STATE_FAILED);
}

MotorPreHomeState MotorControl_PreHomeGetState(void)
{
    return motor_prehome.state;
}

uint8_t MotorControl_PreHomeGetCurrentSlave(void)
{
    return motor_prehome.current_slave;
}

uint8_t MotorControl_PreHomeGetFailedSlave(void)
{
    return motor_prehome.failed_slave;
}

uint8_t MotorControl_PreHomeGetLastError(void)
{
    return motor_prehome.last_error;
}

uint8_t MotorControl_PreHomeStart(uint8_t saved_position_valid,
                                  int32_t motor7_saved_position,
                                  int32_t motor8_saved_position)
{
    if (MotorControl_PreHomeIsBusy() || MotorControl_HomeIsBusy() ||
        motor_batch.active || ModbusMaster_IsBusy()) {
        return MODBUS_RESULT_BUSY;
    }

    memset(&motor_prehome, 0, sizeof(motor_prehome));
    motor_prehome.saved_position_valid = saved_position_valid ? 1U : 0U;
    motor_prehome.saved_positions[0] = motor7_saved_position;
    motor_prehome.saved_positions[1] = motor8_saved_position;
    motor_prehome.current_slave = MOTOR7_SLAVE_ADDR;
    motor_prehome.state = MOTOR_PREHOME_STATE_CHECK_ALARM;
    LOG_INFO("PREHOME", "started: saved_valid=%u, motor7=%ld, motor8=%ld\r\n",
             (unsigned int)motor_prehome.saved_position_valid,
             (long)motor7_saved_position, (long)motor8_saved_position);
    return MODBUS_RESULT_OK;
}

void MotorControl_PreHomeProcess(void)
{
    uint8_t result;
    uint8_t index;
    uint8_t stable;
    uint8_t alarm_status;
    uint8_t slave_addr;
    uint32_t now;

    if (!MotorControl_PreHomeIsBusy()) {
        return;
    }

    if (motor_prehome.state == MOTOR_PREHOME_STATE_CHECK_ALARM) {
        slave_addr = motor_prehome.alarm_index == 0U
                     ? MOTOR7_SLAVE_ADDR : MOTOR8_SLAVE_ADDR;
        motor_prehome.current_slave = slave_addr;
        result = ModbusMaster_03_ReadHoldReg(
            MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
            slave_addr,
            MOTOR_CONTROL_ALARM_STATUS_REG,
            1U,
            &motor_prehome.alarm_status_word);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_PreHomeFail(slave_addr, result);
            return;
        }
        alarm_status = (uint8_t)(motor_prehome.alarm_status_word &
                                 MOTOR_CONTROL_ALARM_STATUS_MASK);
        if (alarm_status == 0U) {
            MotorControl_PreHomeAdvanceAlarmCheck();
        } else {
            LOG_WARN("PREHOME",
                     "alarm found before pre-move: slave=0x%02X, alarm=%u\r\n",
                     (unsigned int)slave_addr,
                     (unsigned int)alarm_status);
            motor_prehome.state = MOTOR_PREHOME_STATE_CLEAR_ALARM;
        }
        return;
    }

    if (motor_prehome.state == MOTOR_PREHOME_STATE_CLEAR_ALARM) {
        slave_addr = motor_prehome.alarm_index == 0U
                     ? MOTOR7_SLAVE_ADDR : MOTOR8_SLAVE_ADDR;
        result = ModbusMaster_06_WriteSingleReg(
            MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
            slave_addr,
            MOTOR_CONTROL_ALARM_CLEAR_REG,
            0U);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_PreHomeFail(slave_addr, result);
            return;
        }
        motor_prehome.state = MOTOR_PREHOME_STATE_VERIFY_CLEAR;
        return;
    }

    if (motor_prehome.state == MOTOR_PREHOME_STATE_VERIFY_CLEAR) {
        slave_addr = motor_prehome.alarm_index == 0U
                     ? MOTOR7_SLAVE_ADDR : MOTOR8_SLAVE_ADDR;
        result = ModbusMaster_03_ReadHoldReg(
            MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
            slave_addr,
            MOTOR_CONTROL_ALARM_STATUS_REG,
            1U,
            &motor_prehome.alarm_status_word);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_PreHomeFail(slave_addr, result);
            return;
        }
        alarm_status = (uint8_t)(motor_prehome.alarm_status_word &
                                 MOTOR_CONTROL_ALARM_STATUS_MASK);
        if (alarm_status != 0U) {
            MotorControl_PreHomeFail(
                slave_addr, MOTOR_CONTROL_RESULT_ALARM(alarm_status));
            return;
        }
        LOG_INFO("PREHOME", "alarm cleared: slave=0x%02X\r\n",
                 (unsigned int)slave_addr);
        MotorControl_PreHomeAdvanceAlarmCheck();
        return;
    }

    if (motor_prehome.state == MOTOR_PREHOME_STATE_READ_INITIAL_MOTOR7) {
        motor_prehome.current_slave = MOTOR7_SLAVE_ADDR;
        result = MotorControl_PreHomeReadPosition(
            MOTOR7_SLAVE_ADDR, &motor_prehome.initial_positions[0]);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_PreHomeFail(MOTOR7_SLAVE_ADDR, result);
            return;
        }
        motor_prehome.current_slave = MOTOR8_SLAVE_ADDR;
        motor_prehome.state = MOTOR_PREHOME_STATE_READ_INITIAL_MOTOR8;
        return;
    }

    if (motor_prehome.state == MOTOR_PREHOME_STATE_READ_INITIAL_MOTOR8) {
        motor_prehome.current_slave = MOTOR8_SLAVE_ADDR;
        result = MotorControl_PreHomeReadPosition(
            MOTOR8_SLAVE_ADDR, &motor_prehome.initial_positions[1]);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_PreHomeFail(MOTOR8_SLAVE_ADDR, result);
            return;
        }

        LOG_INFO("PREHOME", "current: motor7=%ld, motor8=%ld\r\n",
                 (long)motor_prehome.initial_positions[0],
                 (long)motor_prehome.initial_positions[1]);
        if (!motor_prehome.saved_position_valid) {
            if (MotorControl_PositionReached(
                    motor_prehome.initial_positions[0], 0L) &&
                MotorControl_PositionReached(
                    motor_prehome.initial_positions[1], 0L)) {
                MotorControl_PreHomeSetReady();
                return;
            }
            LOG_WARN("PREHOME",
                     "positions are not zero but no saved position is available\r\n");
            MotorControl_PreHomeSetReady();
            return;
        }

        for (index = 0U; index < 2U; index++) {
            if (motor_prehome.saved_positions[index] >= 0L &&
                motor_prehome.saved_positions[index] <
                    MOTOR_PREHOME_POSITION_LIMIT) {
                motor_prehome.relative_distances[index] =
                    (uint32_t)(MOTOR_PREHOME_POSITION_LIMIT -
                               motor_prehome.saved_positions[index]);
                if (motor_prehome.relative_distances[index] != 0UL) {
                    motor_prehome.move_mask |= (uint8_t)(1U << index);
                }
            }
        }

        if (motor_prehome.move_mask == 0U) {
            LOG_INFO("PREHOME", "no relative pre-move is required\r\n");
            MotorControl_PreHomeSetReady();
            return;
        }
        LOG_INFO("PREHOME", "relative distances: motor7=%lu, motor8=%lu\r\n",
                 (unsigned long)motor_prehome.relative_distances[0],
                 (unsigned long)motor_prehome.relative_distances[1]);
        if ((motor_prehome.move_mask & 0x01U) != 0U) {
            motor_prehome.current_slave = MOTOR7_SLAVE_ADDR;
            motor_prehome.state = MOTOR_PREHOME_STATE_WRITE_MOTOR7;
        } else {
            motor_prehome.current_slave = MOTOR8_SLAVE_ADDR;
            motor_prehome.state = MOTOR_PREHOME_STATE_WRITE_MOTOR8;
        }
        return;
    }

    if (motor_prehome.state == MOTOR_PREHOME_STATE_WRITE_MOTOR7) {
        result = MotorControl_PreHomeWriteRelative(0U);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_PreHomeFail(MOTOR7_SLAVE_ADDR, result);
            return;
        }
        LOG_INFO("PREHOME", "relative move accepted: slave=0x%02X, distance=%lu\r\n",
                 (unsigned int)MOTOR7_SLAVE_ADDR,
                 (unsigned long)motor_prehome.relative_distances[0]);
        if ((motor_prehome.move_mask & 0x02U) != 0U) {
            motor_prehome.current_slave = MOTOR8_SLAVE_ADDR;
            motor_prehome.state = MOTOR_PREHOME_STATE_WRITE_MOTOR8;
        } else {
            MotorControl_PreHomeBeginStableCheck();
        }
        return;
    }

    if (motor_prehome.state == MOTOR_PREHOME_STATE_WRITE_MOTOR8) {
        result = MotorControl_PreHomeWriteRelative(1U);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_PreHomeFail(MOTOR8_SLAVE_ADDR, result);
            return;
        }
        LOG_INFO("PREHOME", "relative move accepted: slave=0x%02X, distance=%lu\r\n",
                 (unsigned int)MOTOR8_SLAVE_ADDR,
                 (unsigned long)motor_prehome.relative_distances[1]);
        MotorControl_PreHomeBeginStableCheck();
        return;
    }

    now = GetTick();
    if (MotorControl_IsTimeReached(now, motor_prehome.timeout_deadline)) {
        ModbusMaster_Cancel();
        MotorControl_PreHomeFail(motor_prehome.current_slave,
                                 MODBUS_RESULT_TIMEOUT);
        return;
    }

    if (motor_prehome.state == MOTOR_PREHOME_STATE_WAIT_POLL) {
        if (!MotorControl_IsTimeReached(now, motor_prehome.next_poll_tick)) {
            return;
        }
        motor_prehome.current_slave = MOTOR7_SLAVE_ADDR;
        motor_prehome.state = MOTOR_PREHOME_STATE_READ_STABLE_MOTOR7;
        return;
    }

    if (motor_prehome.state == MOTOR_PREHOME_STATE_READ_STABLE_MOTOR7) {
        result = MotorControl_PreHomeReadPosition(
            MOTOR7_SLAVE_ADDR, &motor_prehome.sampled_positions[0]);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_PreHomeFail(MOTOR7_SLAVE_ADDR, result);
            return;
        }
        motor_prehome.current_slave = MOTOR8_SLAVE_ADDR;
        motor_prehome.state = MOTOR_PREHOME_STATE_READ_STABLE_MOTOR8;
        return;
    }

    if (motor_prehome.state == MOTOR_PREHOME_STATE_READ_STABLE_MOTOR8) {
        result = MotorControl_PreHomeReadPosition(
            MOTOR8_SLAVE_ADDR, &motor_prehome.sampled_positions[1]);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_PreHomeFail(MOTOR8_SLAVE_ADDR, result);
            return;
        }

        stable = (uint8_t)(
            MotorControl_PreHomePositionStable(
                motor_prehome.sampled_positions[0],
                motor_prehome.last_positions[0]) &&
            MotorControl_PreHomePositionStable(
                motor_prehome.sampled_positions[1],
                motor_prehome.last_positions[1]));
        motor_prehome.last_positions[0] = motor_prehome.sampled_positions[0];
        motor_prehome.last_positions[1] = motor_prehome.sampled_positions[1];
        if (stable) {
            motor_prehome.stable_sample_count++;
        } else {
            motor_prehome.stable_sample_count = 0U;
        }
        LOG_DEBUG("PREHOME",
                  "sample: motor7=%ld, motor8=%ld, stable=%u/%u\r\n",
                  (long)motor_prehome.sampled_positions[0],
                  (long)motor_prehome.sampled_positions[1],
                  (unsigned int)motor_prehome.stable_sample_count,
                  (unsigned int)MOTOR_PREHOME_STABLE_SAMPLE_COUNT);
        if (motor_prehome.stable_sample_count >=
            MOTOR_PREHOME_STABLE_SAMPLE_COUNT) {
            MotorControl_PreHomeSetReady();
        } else {
            motor_prehome.next_poll_tick =
                GetTick() + MOTOR_PREHOME_POLL_INTERVAL_MS;
            motor_prehome.state = MOTOR_PREHOME_STATE_WAIT_POLL;
        }
    }
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

static uint8_t MotorControl_FinishBatch(uint8_t result, uint8_t *results,
                                        int32_t *positions)
{
    uint8_t index;
    uint8_t saved_count = 0U;
    uint8_t saved_addresses[MODBUS_BATCH_MAX_MOTORS];
    int32_t saved_positions[MODBUS_BATCH_MAX_MOTORS];

    if (motor_batch.check_position) {
        for (index = 0U; index < motor_batch.count; index++) {
            if ((motor_batch.reached_mask &
                 (uint8_t)(1U << index)) != 0U) {
                saved_addresses[saved_count] =
                    motor_batch.motors[index].slave_addr;
                saved_positions[saved_count] =
                    motor_batch.reached_positions[index];
                saved_count++;
            }
        }
        if (saved_count > 0U &&
            !MotorPositionStore_UpdateBatch(saved_addresses,
                                            saved_positions,
                                            saved_count) &&
            result == MODBUS_RESULT_OK) {
            result = MODBUS_RESULT_ECHO;
        }
    }

    LOG_INFO("MOTOR_TRACE",
             "batch finished: count=%u, result=0x%02X, phase=%u, index=%u, slave=0x%02X, reached_mask=0x%02X, failures=%u\r\n",
             (unsigned int)motor_batch.count,
             (unsigned int)result,
             (unsigned int)motor_batch.phase,
             (unsigned int)motor_batch.index,
             (unsigned int)(motor_batch.index < motor_batch.count
                 ? motor_batch.motors[motor_batch.index].slave_addr : 0U),
             (unsigned int)motor_batch.reached_mask,
             (unsigned int)motor_batch.failures);
    if (results != NULL) {
        memcpy(results, motor_batch.results,
               motor_batch.count * sizeof(motor_batch.results[0]));
    }
    if (positions != NULL) {
        memcpy(positions, motor_batch.reached_positions,
               motor_batch.count * sizeof(motor_batch.reached_positions[0]));
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
    return (motor_home.state != MOTOR_HOME_STATE_IDLE &&
            motor_home.state != MOTOR_HOME_STATE_SUCCESS &&
            motor_home.state != MOTOR_HOME_STATE_FAILED) ? 1U : 0U;
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

static void MotorControl_HomeBeginCommandPhase(void)
{
    motor_home.command_index = 0U;
    motor_home.current_slave = motor_home.addresses[0];
    motor_home.state = MOTOR_HOME_STATE_SEND_COMMAND;
    LOG_INFO("MOTOR", "homing precheck completed\r\n");
}

static void MotorControl_HomeSelectNextPair(void)
{
    while (motor_home.pair_index < MOTOR_HOME_LEVEL_PAIR_COUNT &&
           (!MotorControl_HomePrecheckContains(
                motor_home_level_pairs[motor_home.pair_index][0]) ||
            !MotorControl_HomePrecheckContains(
                motor_home_level_pairs[motor_home.pair_index][1]))) {
        motor_home.pair_index++;
    }

    if (motor_home.pair_index >= MOTOR_HOME_LEVEL_PAIR_COUNT) {
        MotorControl_HomeBeginCommandPhase();
        return;
    }

    motor_home.level_in_progress = 0U;
    motor_home.level_stable_count = 0U;
    motor_home.current_slave =
        motor_home_level_pairs[motor_home.pair_index][0];
    motor_home.state = MOTOR_HOME_STATE_READ_PAIR_FIRST;
}

static uint8_t MotorControl_HomeSaveCurrentPair(void)
{
    uint8_t slave_addrs[2];
    int32_t positions[2];

    slave_addrs[0] = motor_home_level_pairs[motor_home.pair_index][0];
    slave_addrs[1] = motor_home_level_pairs[motor_home.pair_index][1];
    positions[0] = motor_home.pair_first_position;
    positions[1] = motor_home.pair_second_position;
    return MotorPositionStore_UpdateBatch(slave_addrs, positions, 2U);
}

static void MotorControl_HomeAdvancePrecheck(void)
{
    motor_home.precheck_index++;
    if (motor_home.precheck_index < motor_home.precheck_count) {
        motor_home.current_slave =
            motor_home.precheck_addresses[motor_home.precheck_index];
        motor_home.state = MOTOR_HOME_STATE_CHECK_ALARM;
        return;
    }

    motor_home.pair_index = 0U;
    MotorControl_HomeSelectNextPair();
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
                                              uint16_t speed_command,
                                              const uint8_t *precheck_addresses,
                                              uint8_t precheck_count)
{
    uint8_t index;
    uint8_t includes_motor1 = 0U;
    uint8_t includes_motor2 = 0U;

    if (MotorControl_HomeIsBusy() || MotorControl_PreHomeIsBusy()) {
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
    if (precheck_count > MOTOR_HOME_MAX_MOTORS ||
        (precheck_count > 0U && precheck_addresses == NULL)) {
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
    motor_home.motor_count = count;
    memcpy(motor_home.addresses, addresses, count);
    motor_home.precheck_count = precheck_count;
    if (precheck_count > 0U) {
        memcpy(motor_home.precheck_addresses, precheck_addresses,
               precheck_count);
    }
    if (MotorPositionStore_GetAll(
            motor_home.saved_positions,
            &motor_home.saved_position_valid_mask)) {
        LOG_INFO("MOTOR",
                 "homing loaded Flash positions: valid_mask=0x%02X\r\n",
                 (unsigned int)motor_home.saved_position_valid_mask);
    }
    motor_home.speed_command = speed_command;
    if (precheck_count > 0U) {
        motor_home.current_slave = motor_home.precheck_addresses[0];
        motor_home.state = MOTOR_HOME_STATE_CHECK_ALARM;
    } else {
        MotorControl_HomeBeginCommandPhase();
    }
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
        speed_command,
        motor_home_addresses,
        (uint8_t)(sizeof(motor_home_addresses) /
                  sizeof(motor_home_addresses[0])));
}

static uint8_t MotorControl_HomeStartBeforePreHome(uint16_t speed_command)
{
    return MotorControl_HomeStartInternal(
        motor_home_before_prehome_addresses,
        (uint8_t)(sizeof(motor_home_before_prehome_addresses) /
                  sizeof(motor_home_before_prehome_addresses[0])),
        speed_command,
        motor_home_before_prehome_addresses,
        (uint8_t)(sizeof(motor_home_before_prehome_addresses) /
                  sizeof(motor_home_before_prehome_addresses[0])));
}

static uint8_t MotorControl_HomeStartAfterPreHome(uint16_t speed_command)
{
    return MotorControl_HomeStartInternal(
        motor_home_after_prehome_addresses,
        (uint8_t)(sizeof(motor_home_after_prehome_addresses) /
                  sizeof(motor_home_after_prehome_addresses[0])),
        speed_command,
        motor_home_after_prehome_addresses,
        (uint8_t)(sizeof(motor_home_after_prehome_addresses) /
                  sizeof(motor_home_after_prehome_addresses[0])));
}

static uint8_t MotorControl_HomeStartMotor78(uint16_t speed_command)
{
    return MotorControl_HomeStartInternal(
        motor_home_motor78_addresses,
        (uint8_t)(sizeof(motor_home_motor78_addresses) /
                  sizeof(motor_home_motor78_addresses[0])),
        speed_command,
        motor_home_motor78_addresses,
        (uint8_t)(sizeof(motor_home_motor78_addresses) /
                  sizeof(motor_home_motor78_addresses[0])));
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
                                                  speed_command,
                                                  &slave_addr, 1U);
        }
    }
    return MODBUS_RESULT_PARAM;
}

void MotorControl_HomeProcess(void)
{
    uint8_t result;
    uint8_t slave_addr;
    uint8_t alarm_status;
    uint8_t pair_first_slave;
    uint8_t pair_second_slave;
    uint32_t now;
    uint32_t raw_adjustment;
    int32_t zero_position = 0L;
    int64_t position_difference;

    if (!MotorControl_HomeIsBusy()) {
        return;
    }

    if (motor_home.state == MOTOR_HOME_STATE_CHECK_ALARM) {
        slave_addr = motor_home.precheck_addresses[
            motor_home.precheck_index];
        motor_home.current_slave = slave_addr;
        result = ModbusMaster_03_ReadHoldReg(
            MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
            slave_addr,
            MOTOR_CONTROL_ALARM_STATUS_REG,
            1U,
            &motor_home.alarm_status_word);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_HomeFail(slave_addr, result);
            return;
        }

        alarm_status = (uint8_t)(motor_home.alarm_status_word &
                                 MOTOR_CONTROL_ALARM_STATUS_MASK);
        if (alarm_status == 0U) {
            MotorControl_HomeAdvancePrecheck();
        } else {
            LOG_WARN("MOTOR",
                     "alarm found before homing: slave=0x%02X, alarm=%u\r\n",
                     (unsigned int)slave_addr,
                     (unsigned int)alarm_status);
            motor_home.state = MOTOR_HOME_STATE_CLEAR_ALARM;
        }
        return;
    }

    if (motor_home.state == MOTOR_HOME_STATE_CLEAR_ALARM) {
        slave_addr = motor_home.precheck_addresses[
            motor_home.precheck_index];
        result = ModbusMaster_06_WriteSingleReg(
            MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
            slave_addr,
            MOTOR_CONTROL_ALARM_CLEAR_REG,
            0U);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_HomeFail(slave_addr, result);
            return;
        }
        motor_home.state = MOTOR_HOME_STATE_VERIFY_CLEAR;
        return;
    }

    if (motor_home.state == MOTOR_HOME_STATE_VERIFY_CLEAR) {
        slave_addr = motor_home.precheck_addresses[
            motor_home.precheck_index];
        result = ModbusMaster_03_ReadHoldReg(
            MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
            slave_addr,
            MOTOR_CONTROL_ALARM_STATUS_REG,
            1U,
            &motor_home.alarm_status_word);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_HomeFail(slave_addr, result);
            return;
        }

        alarm_status = (uint8_t)(motor_home.alarm_status_word &
                                 MOTOR_CONTROL_ALARM_STATUS_MASK);
        if (alarm_status != 0U) {
            MotorControl_HomeFail(
                slave_addr, MOTOR_CONTROL_RESULT_ALARM(alarm_status));
            return;
        }
        LOG_INFO("MOTOR", "alarm cleared: slave=0x%02X\r\n",
                 (unsigned int)slave_addr);
        MotorControl_HomeAdvancePrecheck();
        return;
    }

    if (motor_home.state == MOTOR_HOME_STATE_READ_PAIR_FIRST) {
        pair_first_slave =
            motor_home_level_pairs[motor_home.pair_index][0];
        motor_home.current_slave = pair_first_slave;
        result = MotorControl_HomeReadPosition(
            pair_first_slave, &motor_home.pair_first_position);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_HomeFail(pair_first_slave, result);
            return;
        }
        motor_home.current_slave =
            motor_home_level_pairs[motor_home.pair_index][1];
        motor_home.state = MOTOR_HOME_STATE_READ_PAIR_SECOND;
        return;
    }

    if (motor_home.state == MOTOR_HOME_STATE_READ_PAIR_SECOND) {
        pair_first_slave =
            motor_home_level_pairs[motor_home.pair_index][0];
        pair_second_slave =
            motor_home_level_pairs[motor_home.pair_index][1];
        motor_home.current_slave = pair_second_slave;
        result = MotorControl_HomeReadPosition(
            pair_second_slave, &motor_home.pair_second_position);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_HomeFail(pair_second_slave, result);
            return;
        }

        position_difference = (int64_t)motor_home.pair_first_position -
                              (int64_t)motor_home.pair_second_position;
        LOG_INFO("MOTOR",
                 "home level check: pair=0x%02X/0x%02X, first=%ld, second=%ld, diff=%ld\r\n",
                 (unsigned int)pair_first_slave,
                 (unsigned int)pair_second_slave,
                 (long)motor_home.pair_first_position,
                 (long)motor_home.pair_second_position,
                 (long)position_difference);

        if (motor_home.level_in_progress) {
            if (MotorControl_AbsI64(position_difference) <=
                    MOTOR_HOME_LEVEL_COMPLETE_TOLERANCE &&
                MotorControl_AbsI64(
                    (int64_t)motor_home.pair_second_position -
                    (int64_t)motor_home.level_target) <=
                    MOTOR_HOME_LEVEL_COMPLETE_TOLERANCE) {
                motor_home.level_stable_count++;
            } else {
                motor_home.level_stable_count = 0U;
            }

            if (motor_home.level_stable_count >=
                MOTOR_HOME_LEVEL_STABLE_SAMPLE_COUNT) {
                if (!MotorControl_HomeSaveCurrentPair()) {
                    MotorControl_HomeFail(pair_second_slave,
                                          MODBUS_RESULT_ECHO);
                    return;
                }
                LOG_INFO("MOTOR",
                         "home leveling completed: pair=0x%02X/0x%02X, diff=%ld\r\n",
                         (unsigned int)pair_first_slave,
                         (unsigned int)pair_second_slave,
                         (long)position_difference);
                motor_home.level_in_progress = 0U;
                motor_home.pair_index++;
                MotorControl_HomeSelectNextPair();
                return;
            }

            now = GetTick();
            if (MotorControl_IsTimeReached(now,
                                           motor_home.timeout_deadline)) {
                MotorControl_HomeFail(pair_second_slave,
                                      MODBUS_RESULT_TIMEOUT);
            } else {
                motor_home.next_poll_tick =
                    now + MOTOR_HOME_LEVEL_POLL_INTERVAL_MS;
                motor_home.state = MOTOR_HOME_STATE_WAIT_LEVEL;
            }
            return;
        }

        if (MotorControl_AbsI64(position_difference) <=
            MOTOR_HOME_LEVEL_ERROR_THRESHOLD) {
            if (!MotorControl_HomeSaveCurrentPair()) {
                MotorControl_HomeFail(pair_second_slave,
                                      MODBUS_RESULT_ECHO);
                return;
            }
            motor_home.pair_index++;
            MotorControl_HomeSelectNextPair();
            return;
        }

        if (position_difference > (int64_t)INT32_MAX ||
            position_difference < (int64_t)INT32_MIN) {
            MotorControl_HomeFail(pair_second_slave, MODBUS_RESULT_PARAM);
            return;
        }

        /* 以每组第一台电机为基准，让第二台相对移动 first-second 个脉冲。 */
        motor_home.relative_adjustment = (int32_t)position_difference;
        motor_home.level_target = motor_home.pair_first_position;
        raw_adjustment = (uint32_t)motor_home.relative_adjustment;
        motor_home.relative_position_words[0] =
            (uint16_t)(raw_adjustment & 0xFFFFUL);
        motor_home.relative_position_words[1] =
            (uint16_t)(raw_adjustment >> 16);
        motor_home.state = MOTOR_HOME_STATE_LEVEL_PAIR;
        return;
    }

    if (motor_home.state == MOTOR_HOME_STATE_LEVEL_PAIR) {
        pair_first_slave =
            motor_home_level_pairs[motor_home.pair_index][0];
        pair_second_slave =
            motor_home_level_pairs[motor_home.pair_index][1];
        motor_home.current_slave = pair_second_slave;
        result = ModbusMaster_10_WriteMultiReg(
            MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
            pair_second_slave,
            MOTOR_CONTROL_RELATIVE_COMMAND_REG,
            2U,
            motor_home.relative_position_words);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_HomeFail(pair_second_slave, result);
            return;
        }

        now = GetTick();
        motor_home.level_in_progress = 1U;
        motor_home.level_stable_count = 0U;
        motor_home.next_poll_tick = now + MOTOR_HOME_LEVEL_POLL_INTERVAL_MS;
        motor_home.timeout_deadline = now + MOTOR_HOME_LEVEL_TIMEOUT_MS;
        motor_home.state = MOTOR_HOME_STATE_WAIT_LEVEL;
        LOG_WARN("MOTOR",
                 "home leveling started: reference=0x%02X, adjusted=0x%02X, relative=%ld\r\n",
                 (unsigned int)pair_first_slave,
                 (unsigned int)pair_second_slave,
                 (long)motor_home.relative_adjustment);
        return;
    }

    if (motor_home.state == MOTOR_HOME_STATE_WAIT_LEVEL) {
        pair_second_slave =
            motor_home_level_pairs[motor_home.pair_index][1];
        now = GetTick();
        if (MotorControl_IsTimeReached(now, motor_home.timeout_deadline)) {
            ModbusMaster_Cancel();
            MotorControl_HomeFail(pair_second_slave,
                                  MODBUS_RESULT_TIMEOUT);
            return;
        }
        if (!MotorControl_IsTimeReached(now, motor_home.next_poll_tick)) {
            return;
        }
        motor_home.current_slave = pair_second_slave;
        motor_home.state = MOTOR_HOME_STATE_CHECK_LEVEL_ALARM;
        return;
    }

    if (motor_home.state == MOTOR_HOME_STATE_CHECK_LEVEL_ALARM) {
        pair_second_slave =
            motor_home_level_pairs[motor_home.pair_index][1];
        result = ModbusMaster_03_ReadHoldReg(
            MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
            pair_second_slave,
            MOTOR_CONTROL_ALARM_STATUS_REG,
            1U,
            &motor_home.alarm_status_word);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_HomeFail(pair_second_slave, result);
            return;
        }
        alarm_status = (uint8_t)(motor_home.alarm_status_word &
                                 MOTOR_CONTROL_ALARM_STATUS_MASK);
        if (alarm_status != 0U) {
            MotorControl_HomeFail(
                pair_second_slave,
                MOTOR_CONTROL_RESULT_ALARM(alarm_status));
            return;
        }
        motor_home.state = MOTOR_HOME_STATE_READ_PAIR_FIRST;
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
            if (!MotorPositionStore_UpdateBatch(&slave_addr,
                                                &zero_position, 1U)) {
                MotorControl_HomeFail(slave_addr, MODBUS_RESULT_ECHO);
                return;
            }
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

static void MotorControl_FullHomeFail(uint8_t slave_addr, uint8_t error)
{
    motor_full_home.failed_slave = slave_addr;
    motor_full_home.current_slave = slave_addr;
    motor_full_home.last_error = error;
    motor_full_home.state = MOTOR_FULL_HOME_STATE_FAILED;
    LOG_ERROR("MOTOR", "full homing failed: slave=0x%02X, result=%u\r\n",
              (unsigned int)slave_addr, (unsigned int)error);
}

uint8_t MotorControl_FullHomeIsBusy(void)
{
    return (uint8_t)(
        motor_full_home.state == MOTOR_FULL_HOME_STATE_INITIAL_HOMING ||
        motor_full_home.state == MOTOR_FULL_HOME_STATE_CHECK_MOTOR2_HOME ||
        motor_full_home.state == MOTOR_FULL_HOME_STATE_CHECK_MOTOR1_HOME ||
        motor_full_home.state == MOTOR_FULL_HOME_STATE_PREHOME ||
        motor_full_home.state == MOTOR_FULL_HOME_STATE_OTHER_HOMING ||
        motor_full_home.state == MOTOR_FULL_HOME_STATE_FINAL_HOMING);
}

MotorFullHomeState MotorControl_FullHomeGetState(void)
{
    return motor_full_home.state;
}

uint8_t MotorControl_FullHomeGetCurrentSlave(void)
{
    return motor_full_home.current_slave;
}

uint8_t MotorControl_FullHomeGetFailedSlave(void)
{
    return motor_full_home.failed_slave;
}

uint8_t MotorControl_FullHomeGetLastError(void)
{
    return motor_full_home.last_error;
}

uint8_t MotorControl_FullHomeStart(uint8_t saved_position_valid,
                                   int32_t motor7_saved_position,
                                   int32_t motor8_saved_position,
                                   uint16_t speed_command)
{
    uint8_t result;

    if (MotorControl_FullHomeIsBusy() || MotorControl_HomeIsBusy() ||
        MotorControl_PreHomeIsBusy() || motor_batch.active ||
        ModbusMaster_IsBusy()) {
        return MODBUS_RESULT_BUSY;
    }

    memset(&motor_full_home, 0, sizeof(motor_full_home));
    motor_full_home.saved_position_valid =
        saved_position_valid ? 1U : 0U;
    motor_full_home.saved_positions[0] = motor7_saved_position;
    motor_full_home.saved_positions[1] = motor8_saved_position;
    motor_full_home.speed_command = speed_command;

    result = MotorControl_HomeStartBeforePreHome(speed_command);
    if (result != MODBUS_RESULT_OK) {
        return result;
    }
    motor_full_home.current_slave = MotorControl_HomeGetCurrentSlave();
    motor_full_home.state = MOTOR_FULL_HOME_STATE_INITIAL_HOMING;
    LOG_INFO("MOTOR", "full homing started\r\n");
    return MODBUS_RESULT_OK;
}

static uint8_t MotorControl_FullHomeReadHomeStatus(uint8_t slave_addr,
                                                   uint8_t *homed)
{
    uint8_t result;

    result = ModbusMaster_03_ReadHoldReg(
        MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
        slave_addr,
        MOTOR_HOME_STATUS_REG,
        MOTOR_HOME_STATUS_REG_COUNT,
        motor_full_home.status_words);
    if (result == MODBUS_RESULT_OK) {
        *homed = (uint8_t)(
            (motor_full_home.status_words[0] &
             MOTOR_HOME_COMPLETE_MASK) != 0U);
    }
    return result;
}

void MotorControl_FullHomeProcess(void)
{
    uint8_t result;
    MotorHomeState home_state;
    MotorPreHomeState prehome_state;

    if (!MotorControl_FullHomeIsBusy()) {
        return;
    }

    if (motor_full_home.state ==
        MOTOR_FULL_HOME_STATE_INITIAL_HOMING) {
        MotorControl_HomeProcess();
        motor_full_home.current_slave =
            MotorControl_HomeGetCurrentSlave();
        if (MotorControl_HomeIsBusy()) {
            return;
        }

        home_state = MotorControl_HomeGetState();
        if (home_state == MOTOR_HOME_STATE_FAILED) {
            MotorControl_FullHomeFail(
                MotorControl_HomeGetFailedSlave(),
                MotorControl_HomeGetLastError());
            return;
        }
        if (home_state != MOTOR_HOME_STATE_SUCCESS) {
            MotorControl_FullHomeFail(0U, MODBUS_RESULT_PARAM);
            return;
        }

        motor_full_home.current_slave = MOTOR2_SLAVE_ADDR;
        motor_full_home.state =
            MOTOR_FULL_HOME_STATE_CHECK_MOTOR2_HOME;
        LOG_INFO("MOTOR",
                 "initial motors homed; checking motor1/2 home status\r\n");
        return;
    }

    if (motor_full_home.state ==
        MOTOR_FULL_HOME_STATE_CHECK_MOTOR2_HOME) {
        motor_full_home.current_slave = MOTOR2_SLAVE_ADDR;
        result = MotorControl_FullHomeReadHomeStatus(
            MOTOR2_SLAVE_ADDR, &motor_full_home.motor2_was_homed);
        if (result == MODBUS_RESULT_PENDING ||
            result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_FullHomeFail(MOTOR2_SLAVE_ADDR, result);
            return;
        }
        motor_full_home.current_slave = MOTOR1_SLAVE_ADDR;
        motor_full_home.state =
            MOTOR_FULL_HOME_STATE_CHECK_MOTOR1_HOME;
        return;
    }

    if (motor_full_home.state ==
        MOTOR_FULL_HOME_STATE_CHECK_MOTOR1_HOME) {
        uint8_t motor1_was_homed;

        motor_full_home.current_slave = MOTOR1_SLAVE_ADDR;
        result = MotorControl_FullHomeReadHomeStatus(
            MOTOR1_SLAVE_ADDR, &motor1_was_homed);
        if (result == MODBUS_RESULT_PENDING ||
            result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_FullHomeFail(MOTOR1_SLAVE_ADDR, result);
            return;
        }

        if (motor_full_home.motor2_was_homed && motor1_was_homed) {
            result = MotorControl_HomeStartMotor78(
                motor_full_home.speed_command);
            if (result == MODBUS_RESULT_BUSY) {
                return;
            }
            if (result != MODBUS_RESULT_OK) {
                MotorControl_FullHomeFail(0U, result);
                return;
            }
            motor_full_home.current_slave =
                MotorControl_HomeGetCurrentSlave();
            motor_full_home.state =
                MOTOR_FULL_HOME_STATE_FINAL_HOMING;
            LOG_INFO(
                "MOTOR",
                "motor1/2 already homed; motor7/8 pre-home skipped\r\n");
            return;
        }

        result = MotorControl_PreHomeStart(
            motor_full_home.saved_position_valid,
            motor_full_home.saved_positions[0],
            motor_full_home.saved_positions[1]);
        if (result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_FullHomeFail(0U, result);
            return;
        }
        motor_full_home.current_slave =
            MotorControl_PreHomeGetCurrentSlave();
        motor_full_home.state = MOTOR_FULL_HOME_STATE_PREHOME;
        LOG_INFO("MOTOR",
                 "initial motors homed; motor7/8 pre-home started\r\n");
        return;
    }

    if (motor_full_home.state == MOTOR_FULL_HOME_STATE_PREHOME) {
        MotorControl_PreHomeProcess();
        motor_full_home.current_slave =
            MotorControl_PreHomeGetCurrentSlave();
        if (MotorControl_PreHomeIsBusy()) {
            return;
        }

        prehome_state = MotorControl_PreHomeGetState();
        if (prehome_state == MOTOR_PREHOME_STATE_FAILED) {
            MotorControl_FullHomeFail(
                MotorControl_PreHomeGetFailedSlave(),
                MotorControl_PreHomeGetLastError());
            return;
        }
        if (prehome_state != MOTOR_PREHOME_STATE_READY) {
            MotorControl_FullHomeFail(0U, MODBUS_RESULT_PARAM);
            return;
        }

        result = MotorControl_HomeStartAfterPreHome(
            motor_full_home.speed_command);
        if (result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_FullHomeFail(0U, result);
            return;
        }
        motor_full_home.current_slave =
            MotorControl_HomeGetCurrentSlave();
        motor_full_home.state = MOTOR_FULL_HOME_STATE_OTHER_HOMING;
        LOG_INFO("MOTOR",
                 "motor7/8 pre-home completed; other motors homing started\r\n");
        return;
    }

    if (motor_full_home.state == MOTOR_FULL_HOME_STATE_OTHER_HOMING) {
        MotorControl_HomeProcess();
        motor_full_home.current_slave = MotorControl_HomeGetCurrentSlave();
        if (MotorControl_HomeIsBusy()) {
            return;
        }

        home_state = MotorControl_HomeGetState();
        if (home_state == MOTOR_HOME_STATE_FAILED) {
            MotorControl_FullHomeFail(
                MotorControl_HomeGetFailedSlave(),
                MotorControl_HomeGetLastError());
            return;
        }
        if (home_state != MOTOR_HOME_STATE_SUCCESS) {
            MotorControl_FullHomeFail(0U, MODBUS_RESULT_PARAM);
            return;
        }

        result = MotorControl_HomeStartMotor78(
            motor_full_home.speed_command);
        if (result == MODBUS_RESULT_BUSY) {
            return;
        }
        if (result != MODBUS_RESULT_OK) {
            MotorControl_FullHomeFail(0U, result);
            return;
        }
        motor_full_home.current_slave =
            MotorControl_HomeGetCurrentSlave();
        motor_full_home.state = MOTOR_FULL_HOME_STATE_FINAL_HOMING;
        LOG_INFO("MOTOR",
                 "other motors homed; motor7/8 final homing started\r\n");
        return;
    }

    MotorControl_HomeProcess();
    motor_full_home.current_slave = MotorControl_HomeGetCurrentSlave();
    if (MotorControl_HomeIsBusy()) {
        return;
    }

    home_state = MotorControl_HomeGetState();
    if (home_state == MOTOR_HOME_STATE_SUCCESS) {
        motor_full_home.state = MOTOR_FULL_HOME_STATE_SUCCESS;
        LOG_INFO("MOTOR", "full homing completed\r\n");
    } else if (home_state == MOTOR_HOME_STATE_FAILED) {
        MotorControl_FullHomeFail(MotorControl_HomeGetFailedSlave(),
                                  MotorControl_HomeGetLastError());
    } else {
        MotorControl_FullHomeFail(0U, MODBUS_RESULT_PARAM);
    }
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
                     MotorControl_HomeIsBusy() ||
                     MotorControl_PreHomeIsBusy() ||
                     MotorControl_FullHomeIsBusy());
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
    memset(&motor_prehome, 0, sizeof(motor_prehome));
    memset(&motor_full_home, 0, sizeof(motor_full_home));
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
    uint8_t check_position, int32_t *positions)
{
    uint16_t commands[2];
    uint8_t result;
    uint8_t alarm_status;
    uint8_t index;
    uint32_t now;

    if (MotorControl_HomeIsBusy() || MotorControl_PreHomeIsBusy()) {
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
                    return MotorControl_FinishBatch(result, results,
                                                    positions);
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
                return MotorControl_FinishBatch(result, results, positions);
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
                return MotorControl_FinishBatch(MODBUS_RESULT_TIMEOUT, results,
                                                positions);
            }
            if (!MotorControl_IsTimeReached(now, motor_batch.next_poll_tick)) {
                return MODBUS_RESULT_PENDING;
            }
            motor_batch.phase = MOTOR_BATCH_PHASE_VERIFY_POSITION;
            return MODBUS_RESULT_PENDING;

        case MOTOR_BATCH_PHASE_VERIFY_POSITION:
            result = MotorControl_ReadCurrentPosition(
                &motor_batch.current_position);
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
                return MotorControl_FinishBatch(result, results, positions);
            }

            /* 每次读取位置后都必须检查当前告警，再判断本次移动是否完成。 */
            motor_batch.phase = MOTOR_BATCH_PHASE_VERIFY_ALARM;
            return MODBUS_RESULT_PENDING;

        case MOTOR_BATCH_PHASE_VERIFY_ALARM:
            result = ModbusMaster_03_ReadHoldReg(
                MODBUS_MASTER_CLIENT_MOTOR_CONTROL,
                motor_batch.motors[motor_batch.index].slave_addr,
                MOTOR_CONTROL_ALARM_STATUS_REG,
                1U,
                &motor_batch.alarm_status_word);
            if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
                return result;
            }
            if (result != MODBUS_RESULT_OK) {
                motor_batch.results[motor_batch.index] = result;
                LOG_ERROR("MOTOR",
                    "alarm status read failed: slave=0x%02X, result=%u\r\n",
                    (unsigned int)motor_batch.motors[motor_batch.index].slave_addr,
                    (unsigned int)result);
                return MotorControl_FinishBatch(result, results, positions);
            }

            alarm_status = (uint8_t)(motor_batch.alarm_status_word &
                                     MOTOR_CONTROL_ALARM_STATUS_MASK);
            if (alarm_status != 0U) {
                result = MOTOR_CONTROL_RESULT_ALARM(alarm_status);
                motor_batch.results[motor_batch.index] = result;
                LOG_ERROR("MOTOR",
                    "move aborted by alarm: slave=0x%02X, alarm=%u, raw=0x%04X\r\n",
                    (unsigned int)motor_batch.motors[motor_batch.index].slave_addr,
                    (unsigned int)alarm_status,
                    (unsigned int)motor_batch.alarm_status_word);
                return MotorControl_FinishBatch(result, results, positions);
            }

            if (MotorControl_PositionReached(
                    motor_batch.current_position,
                    motor_batch.target_positions[motor_batch.index])) {
                motor_batch.results[motor_batch.index] = MODBUS_RESULT_OK;
                motor_batch.reached_positions[motor_batch.index] =
                    motor_batch.current_position;
                motor_batch.reached_mask |=
                    (uint8_t)(1U << motor_batch.index);
                LOG_INFO("MOTOR",
                    "position reached: slave=0x%02X, current=%ld, target=%ld\r\n",
                    (unsigned int)motor_batch.motors[motor_batch.index].slave_addr,
                    (long)motor_batch.current_position,
                    (long)motor_batch.target_positions[motor_batch.index]);
                motor_batch.index++;
                if (motor_batch.index >= motor_batch.count) {
                    return MotorControl_FinishBatch(MODBUS_RESULT_OK, results,
                                                    positions);
                }
                /* 其他电机同时运动，直接检查批次中的下一台。 */
                motor_batch.phase = MOTOR_BATCH_PHASE_VERIFY_POSITION;
                return MODBUS_RESULT_PENDING;
            }

            LOG_DEBUG("MOTOR",
                "position pending: slave=0x%02X, current=%ld, target=%ld\r\n",
                (unsigned int)motor_batch.motors[motor_batch.index].slave_addr,
                (long)motor_batch.current_position,
                (long)motor_batch.target_positions[motor_batch.index]);

            now = GetTick();
            if (MotorControl_IsTimeReached(now, motor_batch.verify_deadline)) {
                motor_batch.results[motor_batch.index] = MODBUS_RESULT_TIMEOUT;
                LOG_ERROR("MOTOR",
                    "position timeout: slave=0x%02X, current=%ld, target=%ld\r\n",
                    (unsigned int)motor_batch.motors[motor_batch.index].slave_addr,
                    (long)motor_batch.current_position,
                    (long)motor_batch.target_positions[motor_batch.index]);
                return MotorControl_FinishBatch(MODBUS_RESULT_TIMEOUT, results,
                                                positions);
            }
            motor_batch.next_poll_tick =
                now + MOTOR_CONTROL_POSITION_POLL_INTERVAL_MS;
            motor_batch.phase = MOTOR_BATCH_PHASE_WAIT_POLL;
            return MODBUS_RESULT_PENDING;

        default:
            return MotorControl_FinishBatch(MODBUS_RESULT_PARAM, results,
                                            positions);
    }
}

uint8_t MotorControl_BatchMove(const MotorControlParams *motors, uint8_t count,
                               uint8_t *results)
{
    return MotorControl_BatchMoveInternal(motors, count, results, 1U, NULL);
}

uint8_t MotorControl_BatchMoveCapture(const MotorControlParams *motors,
                                      uint8_t count, uint8_t *results,
                                      int32_t *positions)
{
    if (positions == NULL) {
        return MODBUS_RESULT_PARAM;
    }
    return MotorControl_BatchMoveInternal(motors, count, results, 1U,
                                          positions);
}

static uint8_t MotorControl_MoveToAbsPosInternal(
    const MotorMoveAbsPosParams *motors, uint8_t count, uint8_t *results,
    int32_t *positions)
{
    MotorControlParams converted[MODBUS_BATCH_MAX_MOTORS];
    uint32_t raw_position;
    uint8_t index;

    if (motors == NULL || count == 0U || count > MODBUS_BATCH_MAX_MOTORS) {
        return MotorControl_BatchMove(NULL, count, results);
    }

    for (index = 0U; index < count; index++) {
        /* 先转为无符号数，保留负位置的补码并使用逻辑右移拆分高低字。 */
        raw_position = (uint32_t)motors[index].absolute_pos;
        converted[index].slave_addr = motors[index].slave_addr;
        converted[index].register_address = motors[index].register_address;
        converted[index].value_low_word = (uint16_t)(raw_position & 0xFFFFU);
        converted[index].value_high_word = (uint16_t)(raw_position >> 16);
        converted[index].position_register_address =
            motors[index].position_register_address;
    }

    return MotorControl_BatchMoveInternal(converted, count, results, 1U,
                                          positions);
}

uint8_t MotorControl_MoveToAbsPos(const MotorMoveAbsPosParams *motors,
                                 uint8_t count, uint8_t *results)
{
    return MotorControl_MoveToAbsPosInternal(motors, count, results, NULL);
}

uint8_t MotorControl_MoveToAbsPosCapture(const MotorMoveAbsPosParams *motors,
                                        uint8_t count, uint8_t *results,
                                        int32_t *positions)
{
    if (positions == NULL) {
        return MODBUS_RESULT_PARAM;
    }
    return MotorControl_MoveToAbsPosInternal(motors, count, results,
                                             positions);
}

uint8_t MotorControl_BatchMoveNoPositionCheck(
    const MotorControlParams *motors, uint8_t count, uint8_t *results)
{
    return MotorControl_BatchMoveInternal(motors, count, results, 0U, NULL);
}

uint8_t ModbusBatch_IsBusy(void)
{
    return motor_batch.active;
}

void ModbusBatch_Cancel(void)
{
    memset(&motor_batch, 0, sizeof(motor_batch));
}
