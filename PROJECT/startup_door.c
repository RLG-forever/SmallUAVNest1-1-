#include "startup_door.h"
#include "debug_log.h"
#include "gateway_service.h"
#include "modbus_common.h"
#include "modbus_master.h"
#include "sequence_steps.h"
#include "status_regs.h"
#include "tick.h"

/* 开门命令为反向运动，因此默认使用负限位作为舱门回零基准。 */
#define STARTUP_DOOR_HOME_COMMAND          MOTOR4_HOME_TO_NEGATIVE_LIMIT
#define STARTUP_DOOR_POLL_INTERVAL_MS      250U
#define STARTUP_DOOR_HOME_TIMEOUT_MS     60000U
#define STARTUP_DOOR_CLOSE_TIMEOUT_MS    25000U
#define STARTUP_AC_MAX_ATTEMPTS              3U
#define STARTUP_AC_RETRY_DELAY_MS          1000U

typedef enum {
    STARTUP_DOOR_STATE_INIT = 0,
    STARTUP_DOOR_STATE_SEND_HOME,
    STARTUP_DOOR_STATE_WAIT_HOME_POLL,
    STARTUP_DOOR_STATE_READ_HOME_STATUS,
    STARTUP_DOOR_STATE_SEND_CLOSE,
    STARTUP_DOOR_STATE_WAIT_CLOSE_POLL,
    STARTUP_DOOR_STATE_READ_CLOSE_STATUS,
    STARTUP_DOOR_STATE_OPEN_AC,
    STARTUP_DOOR_STATE_SUCCESS,
    STARTUP_DOOR_STATE_FAILED
} StartupDoorState;

typedef struct {
    StartupDoorState state;
    uint8_t close_running_seen;
    uint8_t ac_failed_attempts;
    uint8_t last_error;
    uint16_t status_word;
    uint32_t next_action_tick;
    uint32_t timeout_deadline;
} StartupDoorContext;

static StartupDoorContext startup_door;

/* 功能：记录上电舱门流程的失败状态、错误码和失败步骤。 */
static uint8_t StartupDoorSequence_Fail(uint8_t error)
{
    StartupDoorState failed_state = startup_door.state;

    startup_door.last_error = error;
    startup_door.state = STARTUP_DOOR_STATE_FAILED;
    StatusRegs_Update(REG_COMMAND_STATE, COMMAND_STATE_FAILED);
    StatusRegs_Update(REG_COMMAND_STEP, MOTOR4_SLAVE_ADDR);
    StatusRegs_Update(REG_FAULT_CODE,
                      ((uint16_t)MOTOR4_SLAVE_ADDR << 8) | error);
    LOG_ERROR("MAIN",
              "startup door sequence failed: state=%u, result=%u\r\n",
              (unsigned int)failed_state,
              (unsigned int)error);
    return error;
}

/**
 * @brief 功能：非阻塞推进上电舱门初始化流程。
 * @note  依次执行舱门回零、关门到位确认和空调开启。调试开关关闭时整段跳过。
 */
uint8_t StartupDoorSequence_Task(void)
{
    uint8_t result;
    uint32_t now = GetTick();

    if (startup_door.state == STARTUP_DOOR_STATE_INIT) {
        if (STARTUP_DOOR_SEQUENCE_ENABLE == 0U) {
            startup_door.state = STARTUP_DOOR_STATE_SUCCESS;
            LOG_WARN("MAIN",
                     "startup door sequence skipped by configuration\r\n");
            return MODBUS_RESULT_OK;
        }

        StatusRegs_Update(REG_COMMAND_CODE,
                          GATEWAY_SERVICE_HOME_COMMAND_REG);
        StatusRegs_Update(REG_COMMAND_STATE, COMMAND_STATE_EXECUTING);
        StatusRegs_Update(REG_COMMAND_STEP, MOTOR4_SLAVE_ADDR);
        startup_door.timeout_deadline =
            now + STARTUP_DOOR_HOME_TIMEOUT_MS;
        startup_door.state = STARTUP_DOOR_STATE_SEND_HOME;
        LOG_INFO("MAIN", "startup door homing started\r\n");
        return MODBUS_RESULT_PENDING;
    }

    if (startup_door.state == STARTUP_DOOR_STATE_SEND_HOME) {
        result = ModbusMaster_06_WriteSingleReg(
            MODBUS_MASTER_CLIENT_MOTOR_CONTROL, MOTOR4_SLAVE_ADDR,
            MOTOR4_HOME_CTRL_REG, STARTUP_DOOR_HOME_COMMAND);
        if (result == MODBUS_RESULT_PENDING ||
            result == MODBUS_RESULT_BUSY) {
            return MODBUS_RESULT_PENDING;
        }
        if (result != MODBUS_RESULT_OK) {
            return StartupDoorSequence_Fail(result);
        }
        startup_door.next_action_tick =
            now + STARTUP_DOOR_POLL_INTERVAL_MS;
        startup_door.state = STARTUP_DOOR_STATE_WAIT_HOME_POLL;
        return MODBUS_RESULT_PENDING;
    }

    if (startup_door.state == STARTUP_DOOR_STATE_WAIT_HOME_POLL) {
        if ((int32_t)(now - startup_door.timeout_deadline) >= 0) {
            return StartupDoorSequence_Fail(MODBUS_RESULT_TIMEOUT);
        }
        if ((int32_t)(now - startup_door.next_action_tick) < 0) {
            return MODBUS_RESULT_PENDING;
        }
        startup_door.state = STARTUP_DOOR_STATE_READ_HOME_STATUS;
    }

    if (startup_door.state == STARTUP_DOOR_STATE_READ_HOME_STATUS) {
        result = ModbusMaster_03_ReadHoldReg(
            MODBUS_MASTER_CLIENT_MOTOR_CONTROL, MOTOR4_SLAVE_ADDR,
            MOTOR4_STATUS_REG, 1U, &startup_door.status_word);
        if (result == MODBUS_RESULT_PENDING ||
            result == MODBUS_RESULT_BUSY) {
            return MODBUS_RESULT_PENDING;
        }
        if (result != MODBUS_RESULT_OK) {
            return StartupDoorSequence_Fail(result);
        }
        if ((startup_door.status_word &
             MOTOR4_HOME_COMPLETE_MASK) != 0U) {
            startup_door.state = STARTUP_DOOR_STATE_SEND_CLOSE;
            LOG_INFO("MAIN", "startup door homing completed: status=0x%04X\r\n",
                     (unsigned int)startup_door.status_word);
            return MODBUS_RESULT_PENDING;
        }
        startup_door.next_action_tick =
            now + STARTUP_DOOR_POLL_INTERVAL_MS;
        startup_door.state = STARTUP_DOOR_STATE_WAIT_HOME_POLL;
        return MODBUS_RESULT_PENDING;
    }

    if (startup_door.state == STARTUP_DOOR_STATE_SEND_CLOSE) {
        result = CloseDr();
        if (result == MODBUS_RESULT_PENDING ||
            result == MODBUS_RESULT_BUSY) {
            return MODBUS_RESULT_PENDING;
        }
        if (result != MODBUS_RESULT_OK) {
            return StartupDoorSequence_Fail(result);
        }
        startup_door.close_running_seen = 0U;
        startup_door.next_action_tick =
            now + STARTUP_DOOR_POLL_INTERVAL_MS;
        startup_door.timeout_deadline =
            now + STARTUP_DOOR_CLOSE_TIMEOUT_MS;
        startup_door.state = STARTUP_DOOR_STATE_WAIT_CLOSE_POLL;
        StatusRegs_Update(REG_DOOR_STATE, 3U);
        LOG_INFO("MAIN", "startup door close command accepted\r\n");
        return MODBUS_RESULT_PENDING;
    }

    if (startup_door.state == STARTUP_DOOR_STATE_WAIT_CLOSE_POLL) {
        if ((int32_t)(now - startup_door.timeout_deadline) >= 0) {
            return StartupDoorSequence_Fail(MODBUS_RESULT_TIMEOUT);
        }
        if ((int32_t)(now - startup_door.next_action_tick) < 0) {
            return MODBUS_RESULT_PENDING;
        }
        startup_door.state = STARTUP_DOOR_STATE_READ_CLOSE_STATUS;
    }

    if (startup_door.state == STARTUP_DOOR_STATE_READ_CLOSE_STATUS) {
        result = ModbusMaster_03_ReadHoldReg(
            MODBUS_MASTER_CLIENT_MOTOR_CONTROL, MOTOR4_SLAVE_ADDR,
            MOTOR4_STATUS_REG, 1U, &startup_door.status_word);
        if (result == MODBUS_RESULT_PENDING ||
            result == MODBUS_RESULT_BUSY) {
            return MODBUS_RESULT_PENDING;
        }
        if (result != MODBUS_RESULT_OK) {
            return StartupDoorSequence_Fail(result);
        }
        if ((startup_door.status_word &
             MOTOR4_MOVE_COMPLETE_MASK) == 0U) {
            startup_door.close_running_seen = 1U;
        } else if (startup_door.close_running_seen) {
            startup_door.state = STARTUP_DOOR_STATE_OPEN_AC;
            startup_door.next_action_tick = 0U;
            StatusRegs_Update(REG_DOOR_STATE, 4U);
            LOG_INFO("MAIN", "startup door closed: status=0x%04X\r\n",
                     (unsigned int)startup_door.status_word);
            return MODBUS_RESULT_PENDING;
        }
        startup_door.next_action_tick =
            now + STARTUP_DOOR_POLL_INTERVAL_MS;
        startup_door.state = STARTUP_DOOR_STATE_WAIT_CLOSE_POLL;
        return MODBUS_RESULT_PENDING;
    }

    if (startup_door.state == STARTUP_DOOR_STATE_OPEN_AC) {
        if (startup_door.next_action_tick != 0U &&
            (int32_t)(now - startup_door.next_action_tick) < 0) {
            return MODBUS_RESULT_PENDING;
        }
        result = OpenAC();
        if (result == MODBUS_RESULT_PENDING ||
            result == MODBUS_RESULT_BUSY) {
            return MODBUS_RESULT_PENDING;
        }
        if (result == MODBUS_RESULT_OK) {
            startup_door.state = STARTUP_DOOR_STATE_SUCCESS;
            LOG_INFO("MAIN", "startup door sequence completed\r\n");
            return MODBUS_RESULT_OK;
        }

        startup_door.ac_failed_attempts++;
        if (startup_door.ac_failed_attempts >=
            STARTUP_AC_MAX_ATTEMPTS) {
            /* 保持原有策略：空调失败只记录，不阻止机械机构继续回零。 */
            startup_door.state = STARTUP_DOOR_STATE_SUCCESS;
            LOG_ERROR("MAIN",
                      "startup air conditioner open failed after %u attempts\r\n",
                      (unsigned int)startup_door.ac_failed_attempts);
            return MODBUS_RESULT_OK;
        }
        startup_door.next_action_tick =
            now + STARTUP_AC_RETRY_DELAY_MS;
        LOG_WARN("MAIN",
                 "startup air conditioner open retry scheduled: %u/%u\r\n",
                 (unsigned int)startup_door.ac_failed_attempts,
                 (unsigned int)STARTUP_AC_MAX_ATTEMPTS);
        return MODBUS_RESULT_PENDING;
    }

    if (startup_door.state == STARTUP_DOOR_STATE_SUCCESS) {
        return MODBUS_RESULT_OK;
    }
    if (startup_door.state == STARTUP_DOOR_STATE_FAILED) {
        return startup_door.last_error;
    }
    return StartupDoorSequence_Fail(MODBUS_RESULT_PARAM);
}

