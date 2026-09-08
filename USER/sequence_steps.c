#include "sequence_steps.h"
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

/* 需要同步运动的电机组；单电机步骤直接在 EXEC_MOVE_ABS_STEP 中写参数。 */
static const MotorMoveAbsPosParams plane_transfer_in_motors[] = {
    EXEC_ABS_POS(MOTOR5_SLAVE_ADDR, CALIB_PLANE_TRANSFER_IN_MOTOR56_POS),
    EXEC_ABS_POS(MOTOR6_SLAVE_ADDR, CALIB_PLANE_TRANSFER_IN_MOTOR56_POS),

    EXEC_ABS_POS(MOTOR7_SLAVE_ADDR, CALIB_PLANE_TRANSFER_IN_MOTOR78_POS),
    EXEC_ABS_POS(MOTOR8_SLAVE_ADDR, CALIB_PLANE_TRANSFER_IN_MOTOR78_POS)
};

static const MotorMoveAbsPosParams plane_transfer_out_motors[] = {
    EXEC_ABS_POS(MOTOR5_SLAVE_ADDR, CALIB_PLANE_TRANSFER_OUT_MOTOR56_POS),
    EXEC_ABS_POS(MOTOR6_SLAVE_ADDR, CALIB_PLANE_TRANSFER_OUT_MOTOR56_POS),
    EXEC_ABS_POS(MOTOR7_SLAVE_ADDR, CALIB_PLANE_TRANSFER_OUT_MOTOR78_POS),
    EXEC_ABS_POS(MOTOR8_SLAVE_ADDR, CALIB_PLANE_TRANSFER_OUT_MOTOR78_POS)
};

static const MotorMoveAbsPosParams fly_forward_motors[] = {
    EXEC_ABS_POS(MOTOR5_SLAVE_ADDR, CALIB_PLANE_OFFSET_POS),
    EXEC_ABS_POS(MOTOR6_SLAVE_ADDR, CALIB_PLANE_OFFSET_POS),
    EXEC_ABS_POS(MOTOR7_SLAVE_ADDR, CALIB_PLANE_BASE_POS),
    EXEC_ABS_POS(MOTOR8_SLAVE_ADDR, CALIB_PLANE_BASE_POS)
};

static const MotorMoveAbsPosParams fly_back_motors[] = {
    EXEC_ABS_POS(MOTOR5_SLAVE_ADDR, CALIB_PLANE_BASE_POS),
    EXEC_ABS_POS(MOTOR6_SLAVE_ADDR, CALIB_PLANE_BASE_POS),
    EXEC_ABS_POS(MOTOR7_SLAVE_ADDR, CALIB_PLANE_OFFSET_POS),
    EXEC_ABS_POS(MOTOR8_SLAVE_ADDR, CALIB_PLANE_OFFSET_POS)
};

static const MotorMoveAbsPosParams center_1_motors[] = {
    EXEC_ABS_POS(MOTOR10_SLAVE_ADDR, CALIB_CENTER_SIDE_A_POS),
    EXEC_ABS_POS(MOTOR9_SLAVE_ADDR, CALIB_CENTER_SIDE_A_POS),

    EXEC_ABS_POS(MOTOR11_SLAVE_ADDR, CALIB_CENTER_SIDE_B_POS),
    EXEC_ABS_POS(MOTOR12_SLAVE_ADDR, CALIB_CENTER_SIDE_B_POS)
};

static const MotorMoveAbsPosParams center_2_motors[] = {
    EXEC_ABS_POS(MOTOR5_SLAVE_ADDR, CALIB_CENTER_FRONT_POS),
    EXEC_ABS_POS(MOTOR6_SLAVE_ADDR, CALIB_CENTER_FRONT_POS),
    EXEC_ABS_POS(MOTOR7_SLAVE_ADDR, CALIB_CENTER_REAR_POS),
    EXEC_ABS_POS(MOTOR8_SLAVE_ADDR, CALIB_CENTER_REAR_POS)
};

static const MotorMoveAbsPosParams leave_center_motors[] = {
    EXEC_ABS_POS(MOTOR5_SLAVE_ADDR, MOTOR_HOME_POS),
    EXEC_ABS_POS(MOTOR6_SLAVE_ADDR, MOTOR_HOME_POS),
    EXEC_ABS_POS(MOTOR7_SLAVE_ADDR, MOTOR_HOME_POS),
    EXEC_ABS_POS(MOTOR8_SLAVE_ADDR, MOTOR_HOME_POS),
    EXEC_ABS_POS(MOTOR10_SLAVE_ADDR, MOTOR_HOME_POS),
    EXEC_ABS_POS(MOTOR12_SLAVE_ADDR, MOTOR_HOME_POS),
    EXEC_ABS_POS(MOTOR9_SLAVE_ADDR, CALIB_CENTER_RELEASE_POS),
    EXEC_ABS_POS(MOTOR11_SLAVE_ADDR, CALIB_CENTER_RELEASE_POS)
};

static uint8_t MoveOneToAbsPos(uint8_t slave_addr, int32_t position)
{
    const MotorMoveAbsPosParams motor =
        EXEC_ABS_POS(slave_addr, position);

    return MotorControl_MoveToAbsPos(&motor, 1U, NULL);
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
    BATTERY_BAY_NEXT,
    BATTERY_BAY_OFFSET_COUNT
};

static const int32_t battery_bay_positions[3] = {
    CALIB_BATTERY_BAY1_POS,
    CALIB_BATTERY_BAY2_POS,
    CALIB_BATTERY_BAY3_POS
};

static const uint8_t battery_bay_current = BATTERY_BAY_CURRENT;
static const uint8_t battery_bay_next = BATTERY_BAY_NEXT;

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

// 电机1移动到回收流程标定位置
uint8_t Motor1Up1(void)
{
    return MoveOneToAbsPos(MOTOR1_SLAVE_ADDR, CALIB_MOTOR1_RECOVERY_POS);
}
// 电机2移动到回收流程前进标定位置
uint8_t Motor2Forward(void)
{
    return MoveOneToAbsPos(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_RECOVERY_FORWARD_POS);
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

uint8_t Motor1Up2(void)
{
    return MoveOneToAbsPos(MOTOR1_SLAVE_ADDR, CALIB_MOTOR1_AUX_POS);
}

uint8_t Motor1Up3(void)
{
    return MoveOneToAbsPos(MOTOR1_SLAVE_ADDR, CALIB_MOTOR1_TRANSFER_LIFT_POS);
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

uint8_t CloseDr(void)
{
    return Motor_Control(MOTOR_ID_4, 1U, MOTOR4_CMD_CLOSE);
}

uint8_t CheckAndCloseDoor(void)
{
    uint16_t uav_status = StatusRegs_Get(REG_RESERVED4);
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

uint8_t StopDr(void)
{
    // return Motor_Control(MOTOR_ID_4, 1U, MOTOR4_CMD_STOP);
    return 0;
}

uint8_t OpenDr(void)
{
    //return Motor_Control(MOTOR_ID_4, 1U, MOTOR4_CMD_OPEN);
    return 0;
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

/* Sequence step tables. */
const StepDef opendr1_steps[] =
{
//      {OpenDr, 15250, STATUS_REG_NONE, 0},      //1.打开舱门
			{StopDr, 1000, REG_DOOR_STATE, 2},       //2.停止
			{CloseAC, 0, STATUS_REG_NONE, 0},   // 关闭空调

};
const uint8_t OPENDR1_STEP_COUNT = (uint8_t)(sizeof(opendr1_steps) / sizeof(opendr1_steps[0]));
// 打开舱门步骤表
const StepDef opendr_steps[] =
{
      {OpenDr, 15250, REG_DOOR_STATE, 2},      //1.打开舱门
//			{StopDr, 1000, REG_DOOR_STATE, 2},       //2.停止
			{CloseAC, 0, STATUS_REG_NONE, 0},   // 关闭空调

};
const uint8_t OPENDR_STEP_COUNT = (uint8_t)(sizeof(opendr_steps) / sizeof(opendr_steps[0]));


// 关闭舱门步骤表
const StepDef closedr_steps[] =
{
    {CloseDr, 16250, REG_DOOR_STATE, 2},      //1.关闭舱门
//			{StopDr, 1000, REG_DOOR_STATE, 4},       //2.停止
	{OpenAC, 0, STATUS_REG_NONE, 0},   // 打开空调

};
const uint8_t CLOSEDR_STEP_COUNT = (uint8_t)(sizeof(closedr_steps) / sizeof(closedr_steps[0]));

// 一键起飞完整步骤表；中间步骤同时供飞机开机、关机流程复用
const StepDef takeoff_steps[] =
{
        {Center_1, 0, STATUS_REG_NONE, 0},        //1.左右居中
        {Center_2, 0, REG_CENTER_ROD_STATE, 4},     	  //2.前后居中
        /* 电机1移动到起降抬升标定位置。 */
        EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, CALIB_MOTOR1_FLY_LIFT_POS, 0, STATUS_REG_NONE, 0),      //3.电机1上升
        //飞机前进
        {MovePlaneTransferIn, 0, STATUS_REG_NONE, 0},       //4.飞机前进
        // 电机2移动到无人机电池靠近标定位置
        EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_FLY_NEAR_POS, 0, STATUS_REG_NONE, 0),      //5.电机2前进
        // 电机2移动到无人机电池远端标定位置
        EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_FLY_NEAR_POS + MOTOR2_FLY_FAR_RELA_POS, 0, STATUS_REG_NONE, 0),      //27.电机2前进
        // 电机2移动到无人机电池靠近标定位置
        EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_FLY_NEAR_POS, 0, STATUS_REG_NONE, 0),      //28.电机2后退
        // 电机2移动到无人机电池远端标定位置
        EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_FLY_NEAR_POS + MOTOR2_FLY_FAR_RELA_POS, 0, STATUS_REG_NONE, 0),      //29.电机2前进
        //移动至原点位置0
        EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),     //30.电机2后退
        //待修改
        {Center_2, 0, STATUS_REG_NONE, 0},       //31.飞机后退
        //电机回零位
        EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),      //32.电机1下降
        //飞机前进
        EXEC_MOVE_ABS_ARRAY_STEP(plane_transfer_out_motors, 0, STATUS_REG_NONE, 0),     //33.飞机前进

        {LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},   	  //34.居中杆释放

        {OpenDr, 15250, REG_DOOR_STATE, 2},     	  //1.打开舱门
        {CloseAC, 500, STATUS_REG_NONE, 0},            // 关闭空调

        {CheckAndCloseDoor, 16250, STATUS_REG_NONE, 0, NULL, NULL,
         UAV_DEPARTURE_WAIT_TIMEOUT_MS},                  // 等待无人机离巢后关闭舱门
};
const uint8_t TAKEOFF_STEP_COUNT = (uint8_t)(sizeof(takeoff_steps) / sizeof(takeoff_steps[0]));

// 降落完成步骤表（取电换电都完成）
const StepDef landing_steps[] =
{
    {Center_1, 0, STATUS_REG_NONE, 0},
    {Center_2, 0, REG_CENTER_ROD_STATE, 4},
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, CALIB_MOTOR1_TRANSFER_LIFT_POS, 0, STATUS_REG_NONE, 0),
    {MovePlaneTransferIn, 0, STATUS_REG_NONE, 0},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_AIRCRAFT_WORK_POS, 0, STATUS_REG_NONE, 0),
    
    {Motor3Clamp, 1000, STATUS_REG_NONE, 0},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),
    {Center_2, 0, STATUS_REG_NONE, 0},

    /* 根据序列启动时锁定的空仓号选择旧电池存放位置。 */
    {NULL, 0, STATUS_REG_NONE, 0, MoveMotor1ToSelectedBay,
     &battery_bay_current},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_BAY_WORK_POS, 0, STATUS_REG_NONE, 0),
    {Motor3Lossen, 1000, STATUS_REG_NONE, 0},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_BAY_RELEASE_POS, 0, STATUS_REG_NONE, 0),
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),

    /* 根据同一个空仓号选择新电池取出位置。 */
    {NULL, 0, STATUS_REG_NONE, 0, MoveMotor1ToSelectedBay,
     &battery_bay_next},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_BAY_WORK_POS, 0, STATUS_REG_NONE, 0),
    {Motor3Clamp, 1000, STATUS_REG_NONE, 0},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, CALIB_MOTOR1_TRANSFER_LIFT_POS, 0, STATUS_REG_NONE, 0),
    {MovePlaneTransferIn, 0, STATUS_REG_NONE, 0},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_AIRCRAFT_RELEASE_POS - 2000L, 0, STATUS_REG_NONE, 0),
    {Motor3Lossen, 1000, STATUS_REG_NONE, 0},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_AIRCRAFT_CLEAR_POS, 0, STATUS_REG_NONE, 0),
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),
    {Center_2, 0, STATUS_REG_NONE, 0},
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),
    EXEC_MOVE_ABS_ARRAY_STEP(plane_transfer_out_motors, 0, REG_SWAP_MECH_STATE, 4),
    {LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},
    //{CloseDr, 16250, REG_DOOR_STATE, 4},
    {OpenAC, 500, STATUS_REG_NONE, 0},
    {UpdateEmptyBay, 500, STATUS_REG_NONE, 0}
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
    {NULL, 0, STATUS_REG_NONE, 0, MoveMotor1ToSelectedBay,
     &battery_bay_current},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_BAY_WORK_POS, 0, STATUS_REG_NONE, 0),
    {Motor3Clamp, 1000, STATUS_REG_NONE, 0},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, CALIB_MOTOR1_TRANSFER_LIFT_POS, 0, STATUS_REG_NONE, 0),
    {MovePlaneTransferIn, 0, STATUS_REG_NONE, 0},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_AIRCRAFT_WORK_POS, 0, STATUS_REG_NONE, 0),
    {Motor3Lossen, 1000, STATUS_REG_NONE, 0},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_AIRCRAFT_CLEAR_POS, 0, STATUS_REG_NONE, 0),
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_LOAD_RETRACT_POS, 0, STATUS_REG_NONE, 0),
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
    EXEC_MOVE_ABS_STEP(MOTOR1_SLAVE_ADDR, CALIB_MOTOR1_TRANSFER_LIFT_POS, 0, STATUS_REG_NONE, 0),
    {MovePlaneTransferIn, 0, STATUS_REG_NONE, 0},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_AIRCRAFT_WORK_POS, 0, STATUS_REG_NONE, 0),
    {Motor3Clamp, 1000, STATUS_REG_NONE, 0},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, MOTOR_HOME_POS, 0, STATUS_REG_NONE, 0),
    {Center_2, 0, STATUS_REG_NONE, 0},

    /* 根据序列启动时锁定的空仓号选择旧电池存放位置。 */
    {NULL, 0, STATUS_REG_NONE, 0, MoveMotor1ToSelectedBay,
     &battery_bay_current},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_BAY_WORK_POS, 0, STATUS_REG_NONE, 0),
    {Motor3Lossen, 1000, STATUS_REG_NONE, 0},
    EXEC_MOVE_ABS_STEP(MOTOR2_SLAVE_ADDR, CALIB_MOTOR2_BAY_RELEASE_POS, 0, STATUS_REG_NONE, 0),
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

/* Motor lists associated with sequence steps. */
typedef struct {
    uint8_t seq_id;          // 序列ID
    uint8_t step_index;      // 步骤索引（从0开始）
    const uint8_t *motor_addrs; // 电机地址列表，以 0 结尾
} StepMotorMap;

// 定义各步骤的电机地址列表
static const uint8_t motors_center1[]   = {0x0A, 0x0C, 0x09, 0x0B, 0x00}; // Center_1
static const uint8_t motors_center2[]   = {0x05, 0x06, 0x07, 0x08, 0x00}; // Center_2
static const uint8_t motors_leave[]     = {0x05,0x06,0x07,0x08,0x0A,0x0C,0x09,0x0B,0x00}; // LeaveCenter / LeaveCenter1
static const uint8_t motors_battery1[]  = {0x03, 0x00}; // Battery_1 / 等电机1单动
static const uint8_t motors_battery2[]  = {0x02, 0x00}; // Battery_3 / 等电机2单动
static const uint8_t motors_battery3[]  = {0x00}; // Battery_4 / 等电机3单动
static const uint8_t motors_battery4[]  = {0x05,0x06,0x07,0x08,0x00}; // Battery_2,6,21 等
static const uint8_t motors_door[]      = {0x00}; // OpenDr, CloseDr, StopDr
static const uint8_t motors_none[]      = {0x00};       // 无电机

static const StepMotorMap step_motor_map[] = {
    // ========== 序列0：一键起飞 (TAKEOFF) ==========
    {0, 0, motors_center1},
    {0, 1, motors_center2},
    {0, 2, motors_battery1},   // motor 1 lift
    {0, 3, motors_battery4},   // plane transfer in
    {0, 4, motors_battery2},   // motor 2 near
    {0, 5, motors_battery2},   // motor 2 far
    {0, 6, motors_battery2},   // motor 2 near
    {0, 7, motors_battery2},   // motor 2 far
    {0, 8, motors_battery2},   // motor 2 home
    {0, 9, motors_center2},    // center return
    {0,10, motors_battery1},   // motor 1 home
    {0,11, motors_battery4},   // plane transfer out
    {0,12, motors_leave},      // leave center
    {0,13, motors_door},       // open door
    {0,14, motors_none},       // close air conditioner
    {0,15, motors_door},       // wait for UAV, then close door

    // ========== 序列1：降落 (LANDING) ==========
    {1, 0, motors_center1},
    {1, 1, motors_center2},
    {1, 2, motors_battery1},   // Battery_1
    {1, 3, motors_battery4},   // Battery_2
    {1, 4, motors_battery2},   // Battery_3
    {1, 5, motors_battery3},   // Battery_4
    {1, 6, motors_battery2},   // Battery_5
    {1, 7, motors_battery4},   // Battery_6
    {1, 8, motors_battery1},   // Battery_7
    {1, 9, motors_battery1},   // Battery_8 / 23 / 14
    {1,10, motors_battery2},   // Battery_9
    {1,11, motors_battery3},   // Battery_10
    {1,12, motors_battery2},   // Battery_11
    {1,13, motors_battery2},   // Battery_12
    {1,14, motors_battery1},   // Battery_13 / 24 / 15
    {1,15, motors_battery4},   // Battery_21
    {1,16, motors_leave},      // LeaveCenter

    // ========== 序列2：居中 (CLOSECENTER) ==========
    {2, 0, motors_center1},
    {2, 1, motors_center2},

    // ========== 序列3：释放 (LEAVECENTER) ==========
    {3, 0, motors_leave},      // LeaveCenter1

    // ========== 序列4：装电池 (LOADBATTERY) ==========
    {4, 0, motors_center1},
    {4, 1, motors_center2},
    {4, 2, motors_battery1},   // Battery_8 / 23 / 14
    {4, 3, motors_battery2},   // Battery_9
    {4, 4, motors_battery3},   // Battery_4
    {4, 5, motors_battery2},   // Battery_22
    {4, 6, motors_battery1},   // Battery_13 / 24 / 15
    {4, 7, motors_battery1},   // Battery_1
    {4, 8, motors_battery4},   // Battery_2
    {4, 9, motors_battery2},   // Battery_3
    {4,10, motors_battery3},   // Battery_10
    {4,11, motors_battery2},   // Battery_16
    {4,12, motors_battery2},   // Battery_17
    {4,13, motors_battery2},   // Battery_5
    {4,14, motors_battery4},   // Battery_6
    {4,15, motors_battery1},   // Battery_7
    {4,16, motors_battery4},   // Battery_21
    {4,17, motors_leave},      // LeaveCenter

    // ========== 序列5：下电池 (DOWNBATTERY) ==========
    {5, 0, motors_center1},
    {5, 1, motors_center2},
    {5, 2, motors_battery1},   // Battery_1
    {5, 3, motors_battery4},   // Battery_2
    {5, 4, motors_battery2},   // Battery_3
    {5, 5, motors_battery3},   // Battery_4
    {5, 6, motors_battery2},   // Battery_5
    {5, 7, motors_battery4},   // Battery_6
    {5, 8, motors_battery1},   // Battery_7
    {5, 9, motors_battery1},   // Battery_8 / 23 / 14
    {5,10, motors_battery2},   // Battery_9
    {5,11, motors_battery3},   // Battery_10
    {5,12, motors_battery2},   // Battery_11
    {5,13, motors_battery2},   // Battery_12
    {5,14, motors_battery1},   // Battery_13 / 24 / 15
    {5,15, motors_battery4},   // Battery_21
    {5,16, motors_leave},      // LeaveCenter

    // ========== 序列8：飞机开机 (OPENFLY) ==========
    {8, 0, motors_center1},
    {8, 1, motors_center2},
    {8, 2, motors_battery1},   // Battery_1
    {8, 3, motors_battery4},   // Battery_2
    {8, 4, motors_battery2},   // Battery_3
    {8, 5, motors_battery2},   // Battery_18
    {8, 6, motors_battery2},   // Battery_19
    {8, 7, motors_battery2},   // Battery_18
    {8, 8, motors_battery2},   // Battery_20
    {8, 9, motors_battery4},   // Battery_6
    {8,10, motors_battery1},   // Battery_7
    {8,11, motors_battery4},   // Battery_21
    {8,12, motors_leave},      // LeaveCenter

    // ========== 序列9：飞机关机 (CLOSEFLY) ==========
    {9, 0, motors_center1},
    {9, 1, motors_center2},
    {9, 2, motors_battery1},   // Battery_1
    {9, 3, motors_battery4},   // Battery_2
    {9, 4, motors_battery2},   // Battery_3
    {9, 5, motors_battery2},   // Battery_18
    {9, 6, motors_battery2},   // Battery_19
    {9, 7, motors_battery2},   // Battery_18
    {9, 8, motors_battery2},   // Battery_20
    {9, 9, motors_battery4},   // Battery_6
    {9,10, motors_battery1},   // Battery_7
    {9,11, motors_battery4},   // Battery_21
    {9,12, motors_leave},      // LeaveCenter

    // ========== 序列10：打开舱门 (OPENDR) ==========
    {10, 0, motors_door},      // OpenDr
    {10, 1, motors_door},      // StopDr

    // ========== 序列11：关闭舱门 (CLOSEDR) ==========
    {11, 0, motors_door},      // CloseDr
    {11, 1, motors_door},      // StopDr
};

#define MAP_SIZE (sizeof(step_motor_map)/sizeof(step_motor_map[0]))

const uint8_t *SequenceSteps_GetMotorList(uint8_t sequence_id,
                                          uint8_t step_index)
{
    uint16_t index;

    for (index = 0U; index < (uint16_t)MAP_SIZE; index++) {
        if (step_motor_map[index].seq_id == sequence_id &&
            step_motor_map[index].step_index == step_index) {
            return step_motor_map[index].motor_addrs;
        }
    }
    return NULL;
}
