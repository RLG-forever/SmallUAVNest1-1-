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
    uint16_t uav_status = StatusRegs_GetLive(REG_RESERVED4);
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
    uint16_t uav_status = StatusRegs_GetLive(REG_RESERVED4);

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

/* 起飞收尾时关闭空调失败只记录告警，不阻止后续关舱门步骤。 */
uint8_t CloseACForTakeoff(void)
{
    uint8_t result = ModbusMaster_06_WriteSingleReg(
        MODBUS_MASTER_CLIENT_GATEWAY, AIR_CONDITIONER_SLAVE,
        AIR_POWER_CTRL_REG, AIR_POWER_OFF_VALUE);

    if (result == MODBUS_RESULT_PENDING ||
        result == MODBUS_RESULT_BUSY) {
        return result;
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
	{CloseACForTakeoff, 0, STATUS_REG_NONE, 0},   // 打开空调

};
const uint8_t CLOSEDR_STEP_COUNT = (uint8_t)(sizeof(closedr_steps) / sizeof(closedr_steps[0]));

// 一键起飞完整步骤表；中间步骤同时供飞机开机、关机流程复用
const StepDef takeoff_steps[] =
{
    {Center_1, 0, STATUS_REG_NONE, 0},        //1.左右居中
    {Center_2, 0, REG_CENTER_ROD_STATE, 4},     	  //2.前后居中
    /* 电机1移动到起降抬升标定位置。 */
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, MOTOR1_FLY_LIFT_POS, 0, STATUS_REG_NONE, 0),      //3.电机1上升
    //飞机前进
    {MovePlaneTransferIn, 0, STATUS_REG_NONE, 0},       //4.飞机前进

    {OpenDr, 20250, REG_DOOR_STATE, 2},      // 飞机前进完成后打开舱门

    // 电机2按当前开机尝试次数移动到靠近位置
    {MoveTakeoffMotor2Near, 0, STATUS_REG_NONE, 0},      //5.电机2前进
    // 电机2按当前开机尝试次数移动到远端位置
    {MoveTakeoffMotor2Far, 0, STATUS_REG_NONE, 0},       //27.电机2前进
    // 电机2返回本轮对应的靠近位置
    {MoveTakeoffMotor2Near, 0, STATUS_REG_NONE, 0},      //28.电机2后退
    // 电机2再次移动到本轮对应的远端位置
    {MoveTakeoffMotor2Far, 0, STATUS_REG_NONE, 0},       //29.电机2前进
    //移动至原点位置0
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_RETURN_POS, 0, STATUS_REG_NONE, 0),     //30.电机2后退

    {WaitForUavPowerOn, 0, STATUS_REG_NONE, 0, NULL, NULL,  UAV_POWER_ON_WAIT_TIMEOUT_MS},   //等待无人机开机
    //待修改
    {Center_2, 0, STATUS_REG_NONE, 0},       //31.飞机后退
    //电机回零位
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, MOTOR1_RETURN_POS, 0, STATUS_REG_NONE, 0),      //32.电机1下降
    //飞机前进
    EXEC_MOVE_ABS_ARRAY_STEP(plane_transfer_out_motors, 0, STATUS_REG_NONE, 0),     //33.飞机前进

    {LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},   	  //34.居中杆释放

    {CloseACForTakeoff, 500, STATUS_REG_NONE, 0}, // 关闭空调，失败后继续关舱门

    {CheckAndCloseDoor, 16250, REG_DOOR_STATE, 4, NULL, NULL, UAV_DEPARTURE_WAIT_TIMEOUT_MS},                  // 等待无人机离巢后关闭舱门
};
const uint8_t TAKEOFF_STEP_COUNT = (uint8_t)(sizeof(takeoff_steps) / sizeof(takeoff_steps[0]));

// 降落完成步骤表（取电换电都完成）
const StepDef landing_steps[] =
{
    {Center_1, 0, STATUS_REG_NONE, 0},
    {Center_2, 0, REG_CENTER_ROD_STATE, 4},
    //上升到取电池高度
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, MOTOR1_TRANSFER_LIFT_POS, 0, STATUS_REG_NONE, 0),
    //飞机前进
    {MovePlaneTransferIn, 0, STATUS_REG_NONE, 0},
    //横移电机移动到取飞机电机位置
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_AIRCRAFT_WORK_POS, 0, STATUS_REG_NONE, 0),
    
    {Motor3Clamp, 1000, STATUS_REG_NONE, 0},

    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),
    {Center_2, 0, STATUS_REG_NONE, 0},

    /* 根据序列启动时锁定的空仓号选择旧电池存放位置。 */
    {NULL, 0, STATUS_REG_NONE, 0, MoveMotor1ToSelectedBay,   &battery_bay_current},
    //横移电机移动到电池仓装电池位置
    {NULL, 0, STATUS_REG_NONE, 0, MoveMotor2ToBayWorkPos,
     &battery_bay_current},
    //松开夹爪
    {Motor3Lossen, 1000, STATUS_REG_NONE, 0},
    //横移电机移动到电池仓推进电池位置
    {NULL, 0, STATUS_REG_NONE, 0, MoveMotor2ToBayReleasePos,
     &battery_bay_current},
    
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_RETURN_POS, 0, STATUS_REG_NONE, 0),

    /* 根据本次序列锁定的目标仓号选择取电池位置。 */
    {NULL, 0, STATUS_REG_NONE, 0, MoveMotor1GetBayPos,  &battery_bay_target},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_BAY_LOAD_POS, 0, STATUS_REG_NONE, 0),
    {Motor3Clamp, 1000, STATUS_REG_NONE, 0},

    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),
    /* 新电池已从下一仓取出，该仓现在为空。 */
    {UpdateEmptyBay, 500, STATUS_REG_NONE, 0},
    //上升到装电池高度
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, MOTOR1_LOAD_BATTERY_POS, 0, STATUS_REG_NONE, 0),
    {MovePlaneTransferIn, 0, STATUS_REG_NONE, 0},

    //横移电机装飞机电池释放位置
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_AIRCRAFT_RELEASE_POS, 0, STATUS_REG_NONE, 0),

    {Motor3Lossen, 1000, STATUS_REG_NONE, 0},

    //横移电机推进去电池位置
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_AIRCRAFT_CLEAR_POS, 2000, STATUS_REG_NONE, 0),

    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR2_RETURN_POS, 0, STATUS_REG_NONE, 0),
    {Center_2, 0, STATUS_REG_NONE, 0},
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, MOTOR1_RETURN_POS, 0, STATUS_REG_NONE, 0),
    EXEC_MOVE_ABS_ARRAY_STEP(plane_transfer_out_motors, 0, REG_SWAP_MECH_STATE, 4),
    
    //{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},

    {CloseDr, 16250, REG_DOOR_STATE, 4},

    {OpenAC, 500, STATUS_REG_NONE, 0},
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
