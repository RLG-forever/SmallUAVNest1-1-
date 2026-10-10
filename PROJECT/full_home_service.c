#include "full_home_service.h"
#include "battery_swap.h"
#include "debug_log.h"
#include "gateway_service.h"
#include "modbus_common.h"
#include "motor_control.h"
#include "status_regs.h"

static uint8_t normal_operations_enabled;
static uint8_t startup_full_homing_pending;
static MotorFullHomeState last_full_home_state =
    MOTOR_FULL_HOME_STATE_IDLE;

/* 成功或无需重置时继续启动；写Flash失败时阻止自动回零。保留moving标志。 */
static uint8_t FullHomeService_ResetStartupPositions(void)
{
#if STARTUP_RESET_MOTOR78_POSITION == 0U
    int32_t saved_motor7_position;
    int32_t saved_motor8_position;
    int64_t difference;

    if (!MotorPositionStore_WasMotionInterrupted()) {
        return 1U;
    }
    if (!MotorPositionStore_Get(&saved_motor7_position,
                                &saved_motor8_position)) {
        LOG_WARN("MAIN", "startup motor7/8 position check skipped: saved positions unavailable\r\n");
        return 1U;
    }

    difference = (int64_t)saved_motor7_position -
                 (int64_t)saved_motor8_position;
    if (difference < 0) {
        difference = -difference;
    }
    LOG_INFO("MAIN",
             "startup motor7/8 interrupted movement check: motor7=%ld, motor8=%ld, difference=%lu, limit=%lu\r\n",
             (long)saved_motor7_position, (long)saved_motor8_position,
             (unsigned long)difference,
             (unsigned long)STARTUP_MOTOR78_POSITION_DIFF_LIMIT);
    if (difference <= (int64_t)STARTUP_MOTOR78_POSITION_DIFF_LIMIT) {
        return 1U;
    }
    LOG_WARN("MAIN", "interrupted motor7/8 movement difference exceeds limit; resetting both saved positions\r\n");
#else
    LOG_WARN("MAIN", "startup motor7/8 position reset forced by configuration\r\n");
#endif

    if (!MotorPositionStore_Save(0L, 0L)) {
        return 0U;
    }
    LOG_WARN("MAIN", "startup motor7/8 saved positions reset to zero\r\n");
    return 1U;
}

/**
 * @brief 启动上电后的全回原点流程，并初始化对应的命令状态。
 * @note  调用前必须完成Modbus主站、状态寄存器、Flash及电机位置存储初始化。
 * @return 1表示全回原点状态机已启动，0表示启动失败。
 */
uint8_t FullHomeService_StartInitial(void)
{
    uint8_t saved_position_valid;
    uint8_t home_start_result;
    int32_t saved_motor7_position = 0L;
    int32_t saved_motor8_position = 0L;

    if (!FullHomeService_ResetStartupPositions()) {
        startup_full_homing_pending = 0U;
        StatusRegs_Update(REG_COMMAND_STATE, COMMAND_STATE_FAILED);
        StatusRegs_Update(REG_FAULT_CODE, MODBUS_RESULT_ECHO);
        LOG_ERROR("MAIN", "motor7/8 position reset failed; automatic startup homing blocked\r\n");
        return 0U;
    }

    /* 电机7、8需要使用掉电前保存的位置完成回零前的安全预移动。 */
    saved_position_valid = MotorPositionStore_Get(
        &saved_motor7_position, &saved_motor8_position);
    home_start_result = MotorControl_FullHomeStart(
        saved_position_valid, saved_motor7_position,
        saved_motor8_position, MOTOR_HOME_SPEED_NORMAL);

    if (home_start_result == MODBUS_RESULT_OK) {
        StatusRegs_Update(REG_COMMAND_CODE,
                          GATEWAY_SERVICE_HOME_COMMAND_REG);
        StatusRegs_Update(REG_COMMAND_STATE, COMMAND_STATE_EXECUTING);
        StatusRegs_Update(REG_COMMAND_STEP,
                          MotorControl_FullHomeGetCurrentSlave());
        startup_full_homing_pending = 1U;
        return 1U;
    }

    /* 启动失败时不上报执行中，保持正常业务锁定并记录失败原因。 */
    StatusRegs_Update(REG_COMMAND_STATE, COMMAND_STATE_FAILED);
    StatusRegs_Update(REG_FAULT_CODE, home_start_result);
    LOG_ERROR("MAIN", "failed to start initial homing: result=%u\r\n",
              (unsigned int)home_start_result);
    startup_full_homing_pending = 0U;
    return 0U;
}

/**
 * @brief 监控全回原点状态，并统一管理业务使能及上电回原点的状态上报。
 * @note  必须在MotorControl_FullHomeProcess()之后调用，才能获取本轮最新状态。
 */
void FullHomeService_Process(void)
{
    MotorFullHomeState full_home_state;
    uint16_t fault_code;

    full_home_state = MotorControl_FullHomeGetState();

    /* 上电回原点期间持续上报当前正在处理的电机地址。 */
    if (startup_full_homing_pending && MotorControl_FullHomeIsBusy()) {
        StatusRegs_Update(REG_COMMAND_STEP,
                          MotorControl_FullHomeGetCurrentSlave());
    }

    /* 同一状态无需重复处理，避免重复打印日志和反复写状态寄存器。 */
    if (full_home_state == last_full_home_state) {
        return;
    }

    if (MotorControl_FullHomeIsBusy()) {
        /* 任意来源的全回原点执行期间，都禁止新业务命令和后台任务。 */
        normal_operations_enabled = 0U;
        GatewayService_SetControlEnabled(0U);
    } else if (full_home_state == MOTOR_FULL_HOME_STATE_SUCCESS) {
        if (MotorControl_FullHomeGetStage() == MOTOR_FULL_HOME_STAGE_ALL) {
            normal_operations_enabled = 1U;
            GatewayService_SetControlEnabled(1U);
        }

        /* 只有上电自动回原点负责更新启动命令的完成状态。 */
        if (startup_full_homing_pending) {
            startup_full_homing_pending = 0U;
            StatusRegs_Update(REG_COMMAND_STATE, COMMAND_STATE_SUCCESS);
            StatusRegs_Update(REG_COMMAND_STEP,
                              MotorControl_FullHomeGetCurrentSlave());
            StatusRegs_Update(REG_FAULT_CODE, 0U);
        }
        if (MotorControl_FullHomeGetStage() == MOTOR_FULL_HOME_STAGE_ALL) {
            LOG_INFO("MAIN",
                     "full homing completed; normal operations enabled\r\n");
        } else {
            LOG_INFO("MAIN", "homing stage %u completed; operations locked\r\n",
                     (unsigned int)MotorControl_FullHomeGetStage());
        }
    } else if (full_home_state == MOTOR_FULL_HOME_STATE_FAILED) {
        fault_code =
            ((uint16_t)MotorControl_FullHomeGetFailedSlave() << 8) |
            MotorControl_FullHomeGetLastError();

        normal_operations_enabled = 0U;
        GatewayService_SetControlEnabled(0U);

        /* 上电自动回原点失败时，将失败电机和错误码反馈给上位机。 */
        if (startup_full_homing_pending) {
            startup_full_homing_pending = 0U;
            StatusRegs_Update(REG_COMMAND_STATE, COMMAND_STATE_FAILED);
            StatusRegs_Update(REG_COMMAND_STEP,
                              MotorControl_FullHomeGetFailedSlave());
            StatusRegs_Update(REG_FAULT_CODE, fault_code);
        }
        LOG_ERROR("MAIN",
                  "homing failed; operations locked: slave=0x%02X, result=%u\r\n",
                  (unsigned int)MotorControl_FullHomeGetFailedSlave(),
                  (unsigned int)MotorControl_FullHomeGetLastError());
    }

    last_full_home_state = full_home_state;
}

uint8_t FullHomeService_IsNormalOperationsEnabled(void)
{
    return normal_operations_enabled;
}
