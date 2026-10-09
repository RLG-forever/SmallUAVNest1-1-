#include "sequence_steps.h"
#include "slave_ctrl.h"
#include "exec_steps.h"
#include "sequence.h"

#include "motor_control.h"
#include "modbus_common.h"
#include "modbus_master.h"
#include "status_regs.h"
#include "battery_swap.h"
#include "relay.h"
#include "tick.h"
#include "debug_log.h"

#include <stddef.h>
#include <string.h>

#define UAV_DEPARTURE_WAIT_TIMEOUT_MS (120UL * 1000UL)
#define UAV_POWER_ON_WAIT_TIMEOUT_MS  (30UL * 1000UL)

/* 需要同步运动的电机组；单电机步骤直接在 EXEC_MOVE_ABS_STEP 中写参数。 */
static const MotorMoveAbsPosParams plane_transfer_in_motors[] = {
    EXEC_ABS_POS(MOTOR5_SLAVE_ADDR, UAV_FORWARD_MOTOR56_POS),
    EXEC_ABS_POS(MOTOR6_SLAVE_ADDR, UAV_FORWARD_MOTOR56_POS),

    EXEC_ABS_POS(MOTOR7_SLAVE_ADDR, UAV_FORWARD_MOTOR78_POS),
    EXEC_ABS_POS(MOTOR8_SLAVE_ADDR, UAV_FORWARD_MOTOR78_POS)
};

static const MotorMoveAbsPosParams plane_transfer_out_motors[] = {
    EXEC_ABS_POS(MOTOR5_SLAVE_ADDR, UAV_BACKWARD_MOTOR56_POS),
    EXEC_ABS_POS(MOTOR6_SLAVE_ADDR, UAV_BACKWARD_MOTOR56_POS),
    EXEC_ABS_POS(MOTOR7_SLAVE_ADDR, UAV_BACKWARD_MOTOR78_POS),
    EXEC_ABS_POS(MOTOR8_SLAVE_ADDR, UAV_BACKWARD_MOTOR78_POS)
};

static const MotorMoveAbsPosParams fly_forward_motors[] = {
    EXEC_ABS_POS(MOTOR5_SLAVE_ADDR, PLANE_OFFSET_POS),
    EXEC_ABS_POS(MOTOR6_SLAVE_ADDR, PLANE_OFFSET_POS),
    EXEC_ABS_POS(MOTOR7_SLAVE_ADDR, PLANE_BASE_POS),
    EXEC_ABS_POS(MOTOR8_SLAVE_ADDR, PLANE_BASE_POS)
};

static const MotorMoveAbsPosParams fly_back_motors[] = {
    EXEC_ABS_POS(MOTOR5_SLAVE_ADDR, PLANE_BASE_POS),
    EXEC_ABS_POS(MOTOR6_SLAVE_ADDR, PLANE_BASE_POS),
    EXEC_ABS_POS(MOTOR7_SLAVE_ADDR, PLANE_OFFSET_POS),
    EXEC_ABS_POS(MOTOR8_SLAVE_ADDR, PLANE_OFFSET_POS)
};

static const MotorMoveAbsPosParams center_1_motors[] = {
    EXEC_ABS_POS(MOTOR10_SLAVE_ADDR, CENTER_SIDE_A_POS),
    EXEC_ABS_POS(MOTOR9_SLAVE_ADDR, CENTER_SIDE_A_POS),

    EXEC_ABS_POS(MOTOR11_SLAVE_ADDR, CENTER_SIDE_B_POS),
    EXEC_ABS_POS(MOTOR12_SLAVE_ADDR, CENTER_SIDE_B_POS)
};

static const MotorMoveAbsPosParams center_2_motors[] = {
    EXEC_ABS_POS(MOTOR5_SLAVE_ADDR, CENTER_FRONT_POS),
    EXEC_ABS_POS(MOTOR6_SLAVE_ADDR, CENTER_FRONT_POS),
    
    EXEC_ABS_POS(MOTOR7_SLAVE_ADDR, CENTER_REAR_POS),
    EXEC_ABS_POS(MOTOR8_SLAVE_ADDR, CENTER_REAR_POS)
};

static const MotorMoveAbsPosParams leave_center_motors[] = {
    EXEC_ABS_POS(MOTOR5_SLAVE_ADDR, MOTOR_HOME_POS),
    EXEC_ABS_POS(MOTOR6_SLAVE_ADDR, MOTOR_HOME_POS),
    EXEC_ABS_POS(MOTOR7_SLAVE_ADDR, MOTOR_HOME_POS),
    EXEC_ABS_POS(MOTOR8_SLAVE_ADDR, MOTOR_HOME_POS),
    EXEC_ABS_POS(MOTOR10_SLAVE_ADDR, MOTOR_HOME_POS),
    EXEC_ABS_POS(MOTOR12_SLAVE_ADDR, MOTOR_HOME_POS),
    EXEC_ABS_POS(MOTOR9_SLAVE_ADDR, MOTOR_HOME_POS),
    EXEC_ABS_POS(MOTOR11_SLAVE_ADDR, MOTOR_HOME_POS)
};

static uint8_t MoveOneToAbsPos(uint8_t slave_addr, int32_t position)
{
    const MotorMoveAbsPosParams motor =
        EXEC_ABS_POS(slave_addr, position);

    return MotorControl_MoveToAbsPos(&motor, 1U, NULL);
}

/* 第二轮开机尝试时整体前移，近端和远端仍保持原有相对距离。 */
static int32_t GetTakeoffMotor2NearPosition(void)
{
    int32_t position = MOTOR2_FLY_NEAR_POS;

    if (Sequence_GetUavPowerRetryCount() > 0U) {
        position += TAKEOFF_UAV_POWER_RETRY_POSITION_OFFSET;
    }
    return position;
}

static uint8_t MoveTakeoffMotor2Near(void)
{
    return MoveOneToAbsPos(MOTOR2_SLAVE_ADDR,
                           GetTakeoffMotor2NearPosition());
}

static uint8_t MoveTakeoffMotor2Far(void)
{
    return MoveOneToAbsPos(
        MOTOR2_SLAVE_ADDR,
        GetTakeoffMotor2NearPosition() + MOTOR2_FLY_FAR_RELA_POS);
}

static uint8_t MoveArrayToAbsPos(const MotorMoveAbsPosParams *motors,
                                 uint8_t count)
{
    return MotorControl_MoveToAbsPos(motors, count, NULL);
}

static uint8_t MovePlaneTransferIn(void)
{
    int32_t positions[EXEC_ARRAY_COUNT(plane_transfer_in_motors)];
    uint8_t result = MotorControl_MoveToAbsPosCapture(
        plane_transfer_in_motors,
        EXEC_ARRAY_COUNT(plane_transfer_in_motors), NULL, positions);

    if (result == MODBUS_RESULT_OK &&
        !MotorPositionStore_Save(positions[2], positions[3])) {
        return MODBUS_RESULT_ECHO;
    }
    return result;
}

enum {
    BATTERY_BAY_CURRENT = 0U,
    BATTERY_BAY_TARGET,
    BATTERY_BAY_OFFSET_COUNT
};

static const int32_t battery_bay_positions[3] = {
    BATTERY_LOAD_BAY1_HEIGHT,
    BATTERY_LOAD_BAY2_HEIGHT,
    BATTERY_LOAD_BAY3_HEIGHT
};

static const int32_t motor1_load_battery_positions[3] = {
    MOTOR1_LOAD_BATTERY_BAY1_POS,
    MOTOR1_LOAD_BATTERY_BAY2_POS,
    MOTOR1_LOAD_BATTERY_BAY3_POS
};

static const int32_t motor2_bay_work_positions[3] = {
    BATTERY_BAY1_REALEASE_POS,
    BATTERY_BAY2_REALEASE_POS,
    BATTERY_BAY3_REALEASE_POS
};

static const int32_t motor2_bay_release_positions[3] = {
    BATTERY_BAY1_PUSH_POS,
    BATTERY_BAY2_PUSH_POS,
    BATTERY_BAY3_PUSH_POS
};


static const int32_t battery_get_bay_positions[3] = {
    BATTERY_GET_BAY1_HEIGHT,
    BATTERY_GET_BAY2_HEIGHT,
    BATTERY_GET_BAY3_HEIGHT
};

static const uint8_t battery_bay_current = BATTERY_BAY_CURRENT;
static const uint8_t battery_bay_target = BATTERY_BAY_TARGET;

static uint8_t MoveMotor1ToSelectedBay(const void *context)
{
    const uint8_t *bay_offset = (const uint8_t *)context;
    uint8_t bay = Sequence_GetSelectedBay();
    uint8_t position_index;

    if (bay_offset == NULL || *bay_offset >= BATTERY_BAY_OFFSET_COUNT ||
        bay < 1U || bay > 3U) {
        return MODBUS_RESULT_PARAM;
    }

    position_index = (uint8_t)(((bay - 1U) + *bay_offset) % 3U);
    return MoveOneToAbsPos(MOTOR1_SLAVE_ADDR,
                           battery_bay_positions[position_index]);
}

static uint8_t MoveMotor1GetBayPos(const void *context)
{
    const uint8_t *bay_offset = (const uint8_t *)context;
    uint8_t bay = Sequence_GetSelectedBay();
    uint8_t position_index;

    if (bay_offset == NULL || *bay_offset >= BATTERY_BAY_OFFSET_COUNT) {
        return MODBUS_RESULT_PARAM;
    }
    if (*bay_offset == BATTERY_BAY_TARGET) {
        bay = Sequence_GetTargetEmptyBay();
    }
    if (bay < 1U || bay > 3U) {
        return MODBUS_RESULT_PARAM;
    }

    position_index = (uint8_t)(bay - 1U);
    return MoveOneToAbsPos(MOTOR1_SLAVE_ADDR,
                           battery_get_bay_positions[position_index]);
}

/* 装飞机电池：按流程锁定的新电池来源仓选择电机1的高度。 */
static uint8_t MoveMotor1ToLoadBatteryPos(void)
{
    const uint8_t bay = Sequence_GetTargetEmptyBay();

    if (bay < 1U || bay > 3U) {
        return MODBUS_RESULT_PARAM;
    }

    return MoveOneToAbsPos(MOTOR1_SLAVE_ADDR,
                           motor1_load_battery_positions[bay - 1U]);
}

/* 把电池装入电池仓：按仓号选择电机2的标定位置。 */
static uint8_t MoveMotor2ToBayWorkPos(const void *context)
{
    const uint8_t *bay_offset = (const uint8_t *)context;
    const uint8_t bay = Sequence_GetSelectedBay();

    if (bay_offset == NULL || *bay_offset != BATTERY_BAY_CURRENT) {
        return MODBUS_RESULT_PARAM;
    }
    if (bay < 1U || bay > 3U) {
        return MODBUS_RESULT_PARAM;
    }

    return MoveOneToAbsPos(MOTOR2_SLAVE_ADDR,
                           motor2_bay_work_positions[bay - 1U]);
}

/* 把电池推入电池仓：按当前仓号选择电机2的标定位置。 */
static uint8_t MoveMotor2ToBayReleasePos(const void *context)
{
    const uint8_t *bay_offset = (const uint8_t *)context;
    const uint8_t bay = Sequence_GetSelectedBay();

    if (bay_offset == NULL || *bay_offset != BATTERY_BAY_CURRENT ||
        bay < 1U || bay > 3U) {
        return MODBUS_RESULT_PARAM;
    }

    return MoveOneToAbsPos(MOTOR2_SLAVE_ADDR,
                           motor2_bay_release_positions[bay - 1U]);
}

typedef enum {
    LANDING_BAY_PUSH_READ_BEFORE = 0,
    LANDING_BAY_PUSH_MOVE,
    LANDING_BAY_PUSH_WAIT_CURRENT,
    LANDING_BAY_PUSH_READ_AFTER,
    LANDING_BAY_PUSH_MOVE_BACK
} LandingBayPushPhase;

static struct {
    LandingBayPushPhase phase;
    uint8_t repush_count;
    uint8_t monitor_enabled;
    uint16_t read_value;
    uint16_t before_value;
    uint32_t current_deadline;
} landing_bay_push;
static uint8_t landing_bay_push_current_fault;

void LandingBayPush_Reset(void)
{
    memset(&landing_bay_push, 0, sizeof(landing_bay_push));
    landing_bay_push_current_fault = 0U;
}

uint8_t LandingBayPush_HasCurrentFault(void)
{
    return landing_bay_push_current_fault;
}

static void LandingBayPush_ResetAction(void)
{
    memset(&landing_bay_push, 0, sizeof(landing_bay_push));
}

/* 在同一步骤中非阻塞地完成电流读取、推进及重推。 */
static uint8_t BatteryPushCheckAndRetry(void)
{
    uint8_t result;
    int32_t increase;

    if (LANDING_BAY_PUSH_CURRENT_CHECK_ENABLE == 0U) {
        return MoveMotor2ToBayReleasePos(&battery_bay_current);
    }

    switch (landing_bay_push.phase) {
    case LANDING_BAY_PUSH_READ_BEFORE:
        result = ModbusMaster_03_ReadHoldReg(
            MODBUS_MASTER_CLIENT_SEQUENCE, CHARGING_CURRENT_SLAVE,
            CHARGING_CURRENT_VALUE_REG, 1U, &landing_bay_push.read_value);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return MODBUS_RESULT_PENDING;
        }
        landing_bay_push.monitor_enabled = result == MODBUS_RESULT_OK;
        if (landing_bay_push.monitor_enabled) {
            StatusRegs_Update(REG_CHARGING_CURRENT,
                              landing_bay_push.read_value);
            landing_bay_push.before_value = landing_bay_push.read_value;
            LOG_INFO("STEP", "bay push current before: bay=%u, retry=%u, value=%u\r\n",
                     (unsigned int)Sequence_GetSelectedBay(),
                     (unsigned int)landing_bay_push.repush_count,
                     (unsigned int)landing_bay_push.before_value);
        } else {
            LOG_ERROR("STEP", "bay push current pre-read failed: result=%u; homing required\r\n",
                     (unsigned int)result);
            return result;
        }
        landing_bay_push.phase = LANDING_BAY_PUSH_MOVE;
        return MODBUS_RESULT_PENDING;

    case LANDING_BAY_PUSH_MOVE:
        result = MoveMotor2ToBayReleasePos(&battery_bay_current);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return MODBUS_RESULT_PENDING;
        }
        if (result != MODBUS_RESULT_OK) {
            return result;
        }
        if (!landing_bay_push.monitor_enabled) {
            LandingBayPush_ResetAction();
            return MODBUS_RESULT_OK;
        }
        landing_bay_push.current_deadline =
            GetTick() + LANDING_BAY_PUSH_CURRENT_WAIT_MS;
        landing_bay_push.phase = LANDING_BAY_PUSH_WAIT_CURRENT;
        return MODBUS_RESULT_PENDING;

    case LANDING_BAY_PUSH_WAIT_CURRENT:
        if ((int32_t)(GetTick() - landing_bay_push.current_deadline) < 0) {
            return MODBUS_RESULT_PENDING;
        }
        landing_bay_push.phase = LANDING_BAY_PUSH_READ_AFTER;
        return MODBUS_RESULT_PENDING;

    case LANDING_BAY_PUSH_READ_AFTER:
        result = ModbusMaster_03_ReadHoldReg(
            MODBUS_MASTER_CLIENT_SEQUENCE, CHARGING_CURRENT_SLAVE,
            CHARGING_CURRENT_VALUE_REG, 1U, &landing_bay_push.read_value);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return MODBUS_RESULT_PENDING;
        }
        if (result != MODBUS_RESULT_OK) {
            LOG_ERROR("STEP", "bay push current post-read failed: result=%u; homing required\r\n",
                     (unsigned int)result);
            LandingBayPush_ResetAction();
            return result;
        }
        StatusRegs_Update(REG_CHARGING_CURRENT,
                          landing_bay_push.read_value);
        increase = (int32_t)landing_bay_push.read_value -
                   (int32_t)landing_bay_push.before_value;
        LOG_INFO("STEP", "bay push current after: bay=%u, retry=%u, before=%u, after=%u, increase=%ld\r\n",
                 (unsigned int)Sequence_GetSelectedBay(),
                 (unsigned int)landing_bay_push.repush_count,
                 (unsigned int)landing_bay_push.before_value,
                 (unsigned int)landing_bay_push.read_value,
                 (long)increase);
        if (increase > LANDING_BAY_PUSH_CURRENT_DELTA_LIMIT) {
            LandingBayPush_ResetAction();
            return MODBUS_RESULT_OK;
        }
        if (landing_bay_push.repush_count < LANDING_BAY_PUSH_MAX_REPUSHES) {
            landing_bay_push.repush_count++;
            landing_bay_push.phase = LANDING_BAY_PUSH_MOVE_BACK;
            LOG_WARN("STEP", "bay push current increase too small; retry=%u/%u, back_pos=%ld\r\n",
                     (unsigned int)landing_bay_push.repush_count,
                     (unsigned int)LANDING_BAY_PUSH_MAX_REPUSHES,
                     (long)LANDING_BAY_PUSH_RETRY_BACK_POS);
            return MODBUS_RESULT_PENDING;
        }
        landing_bay_push_current_fault = 1U;
        LOG_ERROR("STEP", "bay push current increase failed after %u repushes: bay=%u, increase=%ld, fault=0x%04X; homing required\r\n",
                  (unsigned int)landing_bay_push.repush_count,
                  (unsigned int)Sequence_GetSelectedBay(),
                  (long)increase,
                  (unsigned int)LANDING_BAY_PUSH_CURRENT_FAULT_CODE);
        LandingBayPush_ResetAction();
        return MODBUS_RESULT_OK;

    case LANDING_BAY_PUSH_MOVE_BACK:
        result = MoveOneToAbsPos(MOTOR2_SLAVE_ADDR,
                                 LANDING_BAY_PUSH_RETRY_BACK_POS);
        if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
            return MODBUS_RESULT_PENDING;
        }
        if (result != MODBUS_RESULT_OK) {
            return result;
        }
        landing_bay_push.phase = LANDING_BAY_PUSH_READ_BEFORE;
        return MODBUS_RESULT_PENDING;

    default:
        return MODBUS_RESULT_PARAM;
    }
}

// 电机1移动到回收流程标定位置
uint8_t Motor1Up1(void)
{
    return MoveOneToAbsPos(MOTOR1_SLAVE_ADDR, MOTOR1_RECOVERY_POS);
}
// 电机2移动到回收流程前进标定位置
uint8_t Motor2Forward(void)
{
    return MoveOneToAbsPos(MOTOR2_SLAVE_ADDR, MOTOR2_RECOVERY_FORWARD_POS);
}

uint8_t Motor3Clamp(void)
{
    return Motor_Control(MOTOR_CLAMP_ID, 2U, MOTOR_CLAMP_CMD_CLAMP);
}

//电机2回零
uint8_t Motor2Back(void)
{
    return MoveOneToAbsPos(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS);
}

//电机1回到原点位置0
uint8_t Motor1Down1(void)
{
    return MoveOneToAbsPos(MOTOR1_SLAVE_ADDR, MOTOR_HOME_POS);
}

uint8_t Motor3Lossen(void)
{
    return Motor_Control(MOTOR_CLAMP_ID, 1U, MOTOR_CLAMP_CMD_RELEASE);
}

uint8_t Motor1Up3(void)
{
    return MoveOneToAbsPos(MOTOR1_SLAVE_ADDR, MOTOR1_TRANSFER_LIFT_POS);
}

uint8_t FlyForward(void)
{
    return MoveArrayToAbsPos(fly_forward_motors,
                             EXEC_ARRAY_COUNT(fly_forward_motors));
}

uint8_t FlyBack(void)
{
    return MoveArrayToAbsPos(fly_back_motors,
                             EXEC_ARRAY_COUNT(fly_back_motors));
}

uint8_t RelayCtrl(void)
{
		static uint8_t active = 0U;
		static uint32_t stop_at = 0U;

		if (!active) {
				Relay_Forward();
				stop_at = GetTick() + 3000U;
				active = 1U;
				return MODBUS_RESULT_PENDING;
		}
		if ((int32_t)(GetTick() - stop_at) < 0) {
				return MODBUS_RESULT_PENDING;
		}
		Relay_Stop();
		active = 0U;
		return MODBUS_RESULT_OK;
}

uint8_t Center_1(void)
{
    return MoveArrayToAbsPos(center_1_motors,
                             EXEC_ARRAY_COUNT(center_1_motors));
}

uint8_t Center_2(void)
{
    return MoveArrayToAbsPos(center_2_motors,
                             EXEC_ARRAY_COUNT(center_2_motors));
}

uint8_t LeaveCenter(void)
{
    return MoveArrayToAbsPos(leave_center_motors,
                             EXEC_ARRAY_COUNT(leave_center_motors));
}

uint8_t CheckAndCloseDoor(void)
{
    uint16_t uav_status = StatusRegs_GetLive(REG_RC_UAV_STATE);
    uint8_t ret;
    if (uav_status != 3U) {
        return STEP_RESULT_WAIT;
    }

    LOG_INFO("STEP", "UAV absent; closing door\r\n");
    ret = CloseDr();
    if (ret != MODBUS_RESULT_OK && ret != MODBUS_RESULT_PENDING &&
        ret != MODBUS_RESULT_BUSY) {
        LOG_ERROR("STEP", "close door failed: result=%u\r\n",
                  (unsigned int)ret);
    }
    return ret;
}

uint8_t WaitForUavPowerOn(void)
{
    uint16_t uav_status = StatusRegs_GetLive(REG_RC_UAV_STATE);

    if (Sequence_GetCurrentId() != SEQ_ID_TAKEOFF &&
        Sequence_GetCurrentId() != SEQ_ID_TAKEOFF_NO_CLOSE) {
        return MODBUS_RESULT_OK;
    }
    if (uav_status == 2U || uav_status == 3U) {
        LOG_INFO("STEP", "UAV power-state check passed: status=%u\r\n",
                 (unsigned int)uav_status);
        return MODBUS_RESULT_OK;
    }
    return STEP_RESULT_WAIT;
}

// uint8_t StopDr(void)
// {
//     //return Motor_Control(MOTOR_ID_4, 1U, MOTOR4_CMD_STOP);
//     return 0;
// }

// uint8_t OpenDr(void)
// {
//     //return Motor_Control(MOTOR_ID_4, 1U, MOTOR4_CMD_OPEN);
//     return 0;
// }

// uint8_t CloseDr(void)
// {
//     //return Motor_Control(MOTOR_ID_4, 1U, MOTOR4_CMD_CLOSE);
//     return 0;
// }

uint8_t StopDr(void)
{
    return Motor_Control(MOTOR_ID_4, 1U, MOTOR4_CMD_STOP);
}

uint8_t OpenDr(void)
{
    return Motor_Control(MOTOR_ID_4, 1U, MOTOR4_CMD_OPEN);
}

uint8_t CloseDr(void)
{
    uint8_t result = Motor_Control(MOTOR_ID_4, 1U, MOTOR4_CMD_CLOSE);

    if (result == MODBUS_RESULT_OK) {
        StatusRegs_Update(REG_DOOR_STATE, 3U);
    }
    return result;
}

uint8_t OpenAC(void)
{
	//打开空调
        uint8_t ctrl_ret3 = ModbusMaster_06_WriteSingleReg(
            MODBUS_MASTER_CLIENT_GATEWAY, AIR_CONDITIONER_SLAVE,
            AIR_POWER_CTRL_REG, AIR_POWER_ON_VALUE);
    if (ctrl_ret3 == MODBUS_RESULT_OK) {
        LOG_INFO("STEP", "air conditioner opened\r\n");
    } else if (ctrl_ret3 != MODBUS_RESULT_PENDING &&
               ctrl_ret3 != MODBUS_RESULT_BUSY) {
        LOG_ERROR("STEP", "open air conditioner failed: result=%u\r\n",
                  (unsigned int)ctrl_ret3);
    }
    return ctrl_ret3;
}

/* 起飞收尾或降落时开空调失败只记录告警，不阻止序列继续执行。 */
static uint8_t OpenACForSequence(void)
{
    uint8_t result = ModbusMaster_06_WriteSingleReg(
        MODBUS_MASTER_CLIENT_SEQUENCE, AIR_CONDITIONER_SLAVE,
        AIR_POWER_CTRL_REG, AIR_POWER_ON_VALUE);

    if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
        return MODBUS_RESULT_PENDING;
    }
    if (result != MODBUS_RESULT_OK) {
        LOG_WARN("STEP",
                 "open air conditioner failed: result=%u; continuing sequence\r\n",
                 (unsigned int)result);
        return MODBUS_RESULT_OK;
    }

    LOG_INFO("STEP", "air conditioner opened during sequence\r\n");
    return MODBUS_RESULT_OK;
}

uint8_t CloseAC(void)
{
	//关闭空调
        uint8_t ctrl_ret3 = ModbusMaster_06_WriteSingleReg(
            MODBUS_MASTER_CLIENT_GATEWAY, AIR_CONDITIONER_SLAVE,
            AIR_POWER_CTRL_REG, AIR_POWER_OFF_VALUE);
    if (ctrl_ret3 == MODBUS_RESULT_OK) {
        LOG_INFO("STEP", "air conditioner closed\r\n");
    } else if (ctrl_ret3 != MODBUS_RESULT_PENDING &&
               ctrl_ret3 != MODBUS_RESULT_BUSY) {
        LOG_ERROR("STEP", "close air conditioner failed: result=%u\r\n",
                  (unsigned int)ctrl_ret3);
    }
    return ctrl_ret3;
}

/* 起飞准备时关闭空调失败只记录告警，不阻止后续开机步骤。 */
uint8_t CloseACForTakeoff(void)
{
    uint8_t result = ModbusMaster_06_WriteSingleReg(
        MODBUS_MASTER_CLIENT_GATEWAY, AIR_CONDITIONER_SLAVE,
        AIR_POWER_CTRL_REG, AIR_POWER_OFF_VALUE);

    if (result == MODBUS_RESULT_PENDING ||
        result == MODBUS_RESULT_BUSY) {
        return MODBUS_RESULT_PENDING;
    }
    if (result != MODBUS_RESULT_OK) {
        LOG_WARN("STEP",
                 "close air conditioner failed during takeoff: result=%u; continuing sequence\r\n",
                 (unsigned int)result);
        return MODBUS_RESULT_OK;
    }

    LOG_INFO("STEP", "air conditioner closed during takeoff\r\n");
    return MODBUS_RESULT_OK;
}

/* Sequence step tables. */
// 打开舱门步骤表
const StepDef opendr_steps[] =
{
    {OpenDr, 16250, REG_DOOR_STATE, 2},      //1.打开舱门
    //{OpenDr, 500, REG_DOOR_STATE, 2},      //1.打开舱门
    //{StopDr, 1000, REG_DOOR_STATE, 2},       //2.停止
    {CloseAC, 0, STATUS_REG_NONE, 0},   // 关闭空调

};
const uint8_t OPENDR_STEP_COUNT = (uint8_t)(sizeof(opendr_steps) / sizeof(opendr_steps[0]));


// 关闭舱门步骤表
const StepDef closedr_steps[] =
{
    {CloseDr, 16250, REG_DOOR_STATE, 4},      //1.关闭舱门
	{OpenAC, 0, STATUS_REG_NONE, 0},   // 打开空调

};
const uint8_t CLOSEDR_STEP_COUNT = (uint8_t)(sizeof(closedr_steps) / sizeof(closedr_steps[0]));

// 一键起飞完整步骤表；中间步骤同时供飞机开机、关机流程复用
const StepDef takeoff_steps[] =
{
    {Center_1, 0, STATUS_REG_NONE, 0}, // 0x01 左右居中
    {Center_2, 0, REG_CENTER_ROD_STATE, 4}, // 0x02 前后居中
    /* 电机1移动到起降抬升标定位置。 */
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, MOTOR1_FLY_LIFT_POS, 0, STATUS_REG_NONE, 0), // 0x03 电机1上升
    //飞机前进
    {MovePlaneTransferIn, 0, STATUS_REG_NONE, 0}, // 0x04 飞机前进

    {OpenDr, 20250, REG_DOOR_STATE, 2}, // 0x05 飞机前进完成后打开舱门
    {CloseACForTakeoff, 500, STATUS_REG_NONE, 0}, // 0x06 关闭空调，失败后继续开机

    // 电机2按当前开机尝试次数移动到靠近位置
    {MoveTakeoffMotor2Near, 0, STATUS_REG_NONE, 0}, // 0x07 电机2前进
    // 电机2按当前开机尝试次数移动到远端位置
    {MoveTakeoffMotor2Far, 0, STATUS_REG_NONE, 0}, // 0x08 电机2前进
    // 电机2返回本轮对应的靠近位置
    {MoveTakeoffMotor2Near, 0, STATUS_REG_NONE, 0}, // 0x09 电机2后退
    // 电机2再次移动到本轮对应的远端位置
    {MoveTakeoffMotor2Far, 0, STATUS_REG_NONE, 0}, // 0x0A 电机2前进
    //移动至原点位置0
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_RETURN_POS, 0, STATUS_REG_NONE, 0), // 0x0B 电机2后退

    {WaitForUavPowerOn, 0, STATUS_REG_NONE, 0, NULL, NULL,  UAV_POWER_ON_WAIT_TIMEOUT_MS}, // 0x0C 等待无人机开机
    //待修改
    {Center_2, 0, STATUS_REG_NONE, 0}, // 0x0D 飞机后退
    //电机回零位
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, MOTOR1_RETURN_POS, 0, STATUS_REG_NONE, 0), // 0x0E 电机1下降
    //飞机前进
    EXEC_MOVE_ABS_ARRAY_STEP(plane_transfer_out_motors, 0, STATUS_REG_NONE, 0), // 0x0F 飞机前进

    {LeaveCenter, 0, REG_CENTER_ROD_STATE, 2}, // 0x10 居中杆释放

    {CheckAndCloseDoor, 16250, REG_DOOR_STATE, 4, NULL, NULL, UAV_DEPARTURE_WAIT_TIMEOUT_MS}, // 0x11 等待无人机离巢后关闭舱门

    {OpenACForSequence, 500, STATUS_REG_NONE, 0} // 0x12
};
const uint8_t TAKEOFF_STEP_COUNT = (uint8_t)(sizeof(takeoff_steps) / sizeof(takeoff_steps[0]));

// 降落完成步骤表（取电换电都完成）
const StepDef landing_steps[] =
{
    {Center_1, 0, STATUS_REG_NONE, 0}, // 0x01
    //飞机前进
    {MovePlaneTransferIn, 0, STATUS_REG_NONE, 0}, // 0x02
    {CloseDr, 16250, REG_DOOR_STATE, 4}, // 0x03
    //{CloseDr, 0, REG_DOOR_STATE, 4},
    {OpenACForSequence, 500, STATUS_REG_NONE, 0}, // 0x04
    {Center_2, 0, REG_CENTER_ROD_STATE, 4}, // 0x05
    //上升到取电池高度
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, MOTOR1_TRANSFER_LIFT_POS, 0, STATUS_REG_NONE, 0), // 0x06
    //飞机前进
    {MovePlaneTransferIn, 0, STATUS_REG_NONE, 0}, // 0x07
    //横移电机移动到取飞机电机位置
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_AIRCRAFT_WORK_POS, 0, STATUS_REG_NONE, 0), // 0x08

    {Motor3Clamp, 1000, STATUS_REG_NONE, 0}, // 0x09

    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0), // 0x0A
    {Center_2, 0, STATUS_REG_NONE, 0}, // 0x0B

    /* 根据序列启动时锁定的空仓号选择旧电池存放位置。 */
    {NULL, 0, STATUS_REG_NONE, 0, MoveMotor1ToSelectedBay,   &battery_bay_current}, // 0x0C
    //横移电机移动到电池仓装电池位置
    {NULL, 0, STATUS_REG_NONE, 0, MoveMotor2ToBayWorkPos,  &battery_bay_current}, // 0x0D
    //松开夹爪
    {Motor3Lossen, 1000, STATUS_REG_NONE, 0}, // 0x0E
    //横移电机移动到电池仓推进电池位置
    {BatteryPushCheckAndRetry, 0, STATUS_REG_NONE, 0, NULL, NULL, 0U, 1U}, // 0x0F

    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_RETURN_POS, 0, STATUS_REG_NONE, 0), // 0x10

    /* 根据本次序列锁定的目标仓号选择取电池位置。 */
    {NULL, 0, STATUS_REG_NONE, 0, MoveMotor1GetBayPos,  &battery_bay_target}, // 0x11
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_BAY_LOAD_POS, 0, STATUS_REG_NONE, 0), // 0x12
    {Motor3Clamp, 1000, STATUS_REG_NONE, 0}, // 0x13

    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0), // 0x14
    /* 新电池已从下一仓取出，该仓现在为空。 */
    {UpdateEmptyBay, 500, STATUS_REG_NONE, 0}, // 0x15
    //按新电池来源仓上升到对应的装飞机电池高度
    {MoveMotor1ToLoadBatteryPos, 0, STATUS_REG_NONE, 0}, // 0x16
    {MovePlaneTransferIn, 0, STATUS_REG_NONE, 0}, // 0x17

    //横移电机装飞机电池释放位置
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_AIRCRAFT_RELEASE_POS, 0, STATUS_REG_NONE, 0), // 0x18

    {Motor3Lossen, 1000, STATUS_REG_NONE, 0}, // 0x19

    //横移电机推进去电池位置
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_AIRCRAFT_CLEAR_POS, 2000, STATUS_REG_NONE, 0), // 0x1A

    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_RETURN_POS, 0, STATUS_REG_NONE, 0), // 0x1B
    {Center_2, 0, STATUS_REG_NONE, 0}, // 0x1C
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, MOTOR1_RETURN_POS, 0, STATUS_REG_NONE, 0), // 0x1D
    EXEC_MOVE_ABS_ARRAY_STEP(plane_transfer_out_motors, 0, REG_SWAP_MECH_STATE, 4) // 0x1E

    //{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},
};
const uint8_t LANDING_STEP_COUNT =
    (uint8_t)(sizeof(landing_steps) / sizeof(landing_steps[0]));

// 居中步骤
const StepDef closecenter_steps[] =
{
	{Center_1, 0, STATUS_REG_NONE, 0},
    {Center_2, 0, REG_CENTER_ROD_STATE, 4},
};
const uint8_t CLOSECENTER_STEP_COUNT = (uint8_t)(sizeof(closecenter_steps) / sizeof(closecenter_steps[0]));

// 释放步骤
const StepDef leavecenter_steps[] =
{
	{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},    // 到位后完成

};
const uint8_t LEAVECENTER_STEP_COUNT = (uint8_t)(sizeof(leavecenter_steps) / sizeof(leavecenter_steps[0]));

// 装电池公共步骤（按序列启动时锁定的空仓号取电池）
const StepDef loadbattery_steps[] =
{
    {Center_1, 0, STATUS_REG_NONE, 0},
    {Center_2, 0, REG_CENTER_ROD_STATE, 4},

    /* 根据序列启动时锁定的空仓号选择电池仓位置。 */
    {NULL, 0, STATUS_REG_NONE, 0, MoveMotor1GetBayPos, &battery_bay_current},
    //横移电机移动到电池仓取/装电池
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_BAY_LOAD_POS, 0, STATUS_REG_NONE, 0),
    
    {Motor3Clamp, 1000, STATUS_REG_NONE, 0},
    
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),
    //上下移电机取/换电池高度位置
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, MOTOR1_TRANSFER_LIFT_POS, 0, STATUS_REG_NONE, 0),
    {MovePlaneTransferIn, 0, STATUS_REG_NONE, 0},
    //横移电机装飞机电池位置
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_AIRCRAFT_WORK_POS, 0, STATUS_REG_NONE, 0),
    {Motor3Lossen, 1000, STATUS_REG_NONE, 0},
    //横移电机装飞机电池时推电池位置
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_AIRCRAFT_CLEAR_POS, 0, STATUS_REG_NONE, 0),
    
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),
    {Center_2, 0, STATUS_REG_NONE, 0},
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),
    EXEC_MOVE_ABS_ARRAY_STEP(plane_transfer_out_motors, 0, REG_SWAP_MECH_STATE, 4),
    {LeaveCenter, 0, REG_CENTER_ROD_STATE, 2}
};
const uint8_t LOADBATTERY_STEP_COUNT =
    (uint8_t)(sizeof(loadbattery_steps) / sizeof(loadbattery_steps[0]));


// 下电池公共步骤（按序列启动时锁定的空仓号存放电池）
const StepDef downbattery_steps[] =
{
    {Center_1, 0, STATUS_REG_NONE, 0},
    {Center_2, 0, REG_CENTER_ROD_STATE, 4},
    //上下移电机取/换电池高度位置
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, MOTOR1_TRANSFER_LIFT_POS, 0, STATUS_REG_NONE, 0),
    {MovePlaneTransferIn, 0, STATUS_REG_NONE, 0},
    //横移电机取飞机电池位置
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_AIRCRAFT_WORK_POS, 0, STATUS_REG_NONE, 0),
    {Motor3Clamp, 1000, STATUS_REG_NONE, 0},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),
    {Center_2, 0, STATUS_REG_NONE, 0},

    /* 根据序列启动时锁定的空仓号选择旧电池存放位置。 */
    {NULL, 0, STATUS_REG_NONE, 0, MoveMotor1ToSelectedBay,
     &battery_bay_current},
    {NULL, 0, STATUS_REG_NONE, 0, MoveMotor2ToBayWorkPos,
     &battery_bay_current},
    {Motor3Lossen, 1000, STATUS_REG_NONE, 0},
    {NULL, 0, STATUS_REG_NONE, 0, MoveMotor2ToBayReleasePos,
     &battery_bay_current},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),
    EXEC_MOVE_ABS_ARRAY_STEP(plane_transfer_out_motors, 0, REG_SWAP_MECH_STATE, 2),
    {LeaveCenter, 0, REG_CENTER_ROD_STATE, 2}
};
const uint8_t DOWNBATTERY_STEP_COUNT =
    (uint8_t)(sizeof(downbattery_steps) / sizeof(downbattery_steps[0]));

// 全局变量记录当前是否有旧电池（由主站轮询或状态寄存器更新）
extern uint8_t g_has_old_battery;   // 1:有电池, 0:无电池

// 步骤表1：有电池时，执行取下电池并放入空电池仓
const StepDef recovery_with_battery_steps[] =
{
    {Motor1Up1, 0, STATUS_REG_NONE, 0},             	//3.电机1上升1
		{FlyForward, 0, STATUS_REG_NONE, 0},						  //4.飞机前进
    {Motor2Forward, 10000, STATUS_REG_NONE, 0},          //5.电机2前进
		{Motor3Clamp, 500, STATUS_REG_NONE, 0},					    //6.电机3夹紧
    {FlyBack, 0, STATUS_REG_NONE, 0},					      //7.飞机后退
		{Motor2Back, 10000, STATUS_REG_NONE, 0}, 					  //8.电机2后退
    {Motor1Down1, 0, STATUS_REG_NONE, 0},						//9.电机1下降1
		{Motor2Forward, 10000, STATUS_REG_NONE, 0},					//10.电机2前进
		{Motor3Lossen, 500, STATUS_REG_NONE, 0},							//11.电机3松开
		{Motor2Back, 10000, STATUS_REG_NONE, 0},							//10.电机2后退
};
const uint8_t RECOVERY_WITH_BATTERY_COUNT = (uint8_t)(sizeof(recovery_with_battery_steps)/sizeof(recovery_with_battery_steps[0]));

// 步骤表2：无电池时，只需复位电机到初始状态（无需取电池）
const StepDef recovery_without_battery_steps[] =
{
    {Motor2Back, 5000, STATUS_REG_NONE, 0},              // 电机2后退（确保在初始位）
    {Motor1Down1, 0, STATUS_REG_NONE, 0},             // 电机1下降
	{Motor3Lossen, 500, STATUS_REG_NONE, 0},							// 电机3松开
    {LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},            // 居中杆释放
};
const uint8_t RECOVERY_WITHOUT_BATTERY_COUNT = (uint8_t)(sizeof(recovery_without_battery_steps)/sizeof(recovery_without_battery_steps[0]));
