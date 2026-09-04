#include "sequence_steps.h"

#include "motor_control.h"
#include "modbus_common.h"
#include "modbus_master.h"
#include "status_regs.h"
#include "battery_swap.h"
#include "relay.h"
#include "tick.h"
#include "debug_log.h"

#include <stddef.h>

//MOTOR1_SLAVE_ADDR运动到位置315000
uint8_t Motor1Up1(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR1_MOVE_POS_315000_LOW_WORD, MOTOR1_MOVE_POS_315000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//电机2移动到136000位置
uint8_t Motor2Forward(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1,
         MOTOR2_MOVE_POS_136000_LOW_WORD,
         MOTOR2_MOVE_POS_136000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}

uint8_t Motor3Clamp(void)
{
    return Motor_Control(MOTOR_CLAMP_ID, 2U, MOTOR_CLAMP_CMD_CLAMP);
}

//电机2回零
uint8_t Motor2Back(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1,
         MOTOR_HOME_POSITION_LOW_WORD,
         MOTOR_HOME_POSITION_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}

//电机1回到原点位置0
uint8_t Motor1Down1(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1,
         MOTOR_HOME_POSITION_LOW_WORD,
         MOTOR_HOME_POSITION_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}

uint8_t Motor3Lossen(void)
{
    return Motor_Control(MOTOR_CLAMP_ID, 1U, MOTOR_CLAMP_CMD_RELEASE);
}

uint8_t Motor1Up2(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR1_MOVE_POS_9000_LOW_WORD, MOTOR1_MOVE_POS_9000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}

uint8_t Motor1Up3(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR1_MOVE_POS_330000_LOW_WORD, MOTOR1_MOVE_POS_330000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}


uint8_t FlyForward(void)
{
    MotorControlParams motors[4] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_MOVE_POS_LOW_WORD, MOTOR6_MOVE_POS_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_MOVE_POS_LOW_WORD, MOTOR6_MOVE_POS_HIGH_WORD},
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_TRAVEL_LOW_WORD, MOTOR5_TRAVEL_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_TRAVEL_LOW_WORD, MOTOR5_TRAVEL_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 4U, NULL);
}

uint8_t FlyBack(void)
{
    MotorControlParams motors[4] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_TRAVEL_LOW_WORD, MOTOR5_TRAVEL_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_TRAVEL_LOW_WORD, MOTOR5_TRAVEL_HIGH_WORD},
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_MOVE_POS_LOW_WORD, MOTOR6_MOVE_POS_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_MOVE_POS_LOW_WORD, MOTOR6_MOVE_POS_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 4U, NULL);
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
    MotorControlParams motors[4] = {
        {MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_MOVE_POS_LOW_WORD, MOTOR10_MOVE_POS_HIGH_WORD},        
        {MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_TRAVEL_LOW_WORD, MOTOR9_TRAVEL_HIGH_WORD},

        {MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR11_TRAVEL_LOW_WORD, MOTOR11_TRAVEL_HIGH_WORD},
        {MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_MOVE_POS_LOW_WORD, MOTOR12_MOVE_POS_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 4U, NULL);
}

uint8_t Center_2(void)
{
    MotorControlParams motors[4] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR14_MOVE_POS_LOW_WORD, MOTOR14_MOVE_POS_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR14_MOVE_POS_LOW_WORD, MOTOR14_MOVE_POS_HIGH_WORD},
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR8_MOVE_POS_LOW_WORD, MOTOR8_MOVE_POS_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR8_MOVE_POS_LOW_WORD, MOTOR8_MOVE_POS_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 4U, NULL);
}

uint8_t LeaveCenter(void)
{
    MotorControlParams motors[8] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR_HOME_POSITION_LOW_WORD, MOTOR_HOME_POSITION_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR_HOME_POSITION_LOW_WORD, MOTOR_HOME_POSITION_HIGH_WORD},
        
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR_HOME_POSITION_LOW_WORD, MOTOR_HOME_POSITION_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR_HOME_POSITION_LOW_WORD, MOTOR_HOME_POSITION_HIGH_WORD},
        
        {MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR_HOME_POSITION_LOW_WORD, MOTOR_HOME_POSITION_HIGH_WORD},        
        {MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR_HOME_POSITION_LOW_WORD, MOTOR_HOME_POSITION_HIGH_WORD},
        
        {MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_LEAVE_CENTER_POS_LOW_WORD, MOTOR9_LEAVE_CENTER_POS_HIGH_WORD},
        {MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_LEAVE_CENTER_POS_LOW_WORD, MOTOR12_LEAVE_CENTER_POS_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 8U, NULL);
}

//调试用
uint8_t LeaveCenter1(void)
{
    MotorControlParams motors[8] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_TRAVEL_LOW_WORD, MOTOR13_TRAVEL_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_TRAVEL_LOW_WORD, MOTOR13_TRAVEL_HIGH_WORD},
        
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR7_TRAVEL_LOW_WORD, MOTOR17_TRAVEL_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR7_TRAVEL_LOW_WORD, MOTOR17_TRAVEL_HIGH_WORD},

        {MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_TRAVEL_LOW_WORD, MOTOR9_TRAVEL_HIGH_WORD},
        {MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR11_TRAVEL_LOW_WORD, MOTOR11_TRAVEL_HIGH_WORD},
        
        {MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_MOVE_POS_LOW_WORD, MOTOR10_MOVE_POS_HIGH_WORD},
        {MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_MOVE_POS_LOW_WORD, MOTOR12_MOVE_POS_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 8U, NULL);
}

//电机3运动到330000
uint8_t Battery_1(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR1_MOVE_POS_330000_LOW_WORD, MOTOR1_MOVE_POS_330000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}

//飞机前进
//MOTOR5_SLAVE_ADDR和MOTOR6_SLAVE_ADDR移动到位置183000
//MOTOR7_SLAVE_ADDR和MOTOR8_SLAVE_ADDR移动到位置174000
uint8_t Battery_2(void)
{
    int32_t positions[4];
    uint8_t result;
    MotorControlParams motors[4] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR56_MOVE_POS_183000_LOW_WORD, MOTOR56_MOVE_POS_183000_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR56_MOVE_POS_183000_LOW_WORD, MOTOR56_MOVE_POS_183000_HIGH_WORD},
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR78_MOVE_POS_174000_LOW_WORD, MOTOR78_MOVE_POS_174000_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR78_MOVE_POS_174000_LOW_WORD, MOTOR78_MOVE_POS_174000_HIGH_WORD}
    };
    result = MotorControl_BatchMoveCapture(motors, 4U, NULL, positions);
    if (result == MODBUS_RESULT_OK &&
        !MotorPositionStore_Save(positions[2], positions[3])) {
        return MODBUS_RESULT_ECHO;
    }
    return result;
}
//电机2运动到315000
uint8_t Battery_3(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR2_MOVE_POS_315000_LOW_WORD, MOTOR2_MOVE_POS_315000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//电机夹紧
uint8_t Battery_4(void)
{
    return Motor_Control(MOTOR_CLAMP_ID, 2U, MOTOR_CLAMP_CMD_CLAMP);
}
//回到零位
uint8_t Battery_5(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_HOME_POSITION_LOW_WORD, MOTOR_HOME_POSITION_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//再次回到夹紧位置
uint8_t Battery_6(void)
{
    /*MotorControlParams motors[4] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_TRAVEL_LOW_WORD, MOTOR5_TRAVEL_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_TRAVEL_LOW_WORD, MOTOR5_TRAVEL_HIGH_WORD},
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_MOVE_POS_LOW_WORD, MOTOR6_MOVE_POS_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_MOVE_POS_LOW_WORD, MOTOR6_MOVE_POS_HIGH_WORD}
    };    
    return MotorControl_BatchMove(motors, 4U, NULL);*/
    return Center_2();
}
//电机回零位
uint8_t Battery_7(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_HOME_POSITION_LOW_WORD, MOTOR_HOME_POSITION_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//运动到位置136000
uint8_t Battery_8(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR1_MOVE_POS_136000_LOW_WORD, MOTOR1_MOVE_POS_136000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//电机2运动到位置361000
uint8_t Battery_9(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR2_MOVE_POS_361000_LOW_WORD, MOTOR2_MOVE_POS_361000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//夹爪松开
uint8_t Battery_10(void)
{
    return Motor_Control(MOTOR_CLAMP_ID, 1U, MOTOR_CLAMP_CMD_RELEASE);
}
//移动至位置370000
uint8_t Battery_11(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR2_MOVE_POS_370000_LOW_WORD, MOTOR2_MOVE_POS_370000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}

//MOTOR2_SLAVE_ADDR回到零点
uint8_t Battery_12(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_HOME_POSITION_LOW_WORD, MOTOR_HOME_POSITION_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//MOTOR1_SLAVE_ADDR回到零点
uint8_t Battery_13(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_HOME_POSITION_LOW_WORD, MOTOR_HOME_POSITION_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//MOTOR1_SLAVE_ADDR上升到位置2000
uint8_t Battery_14(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR1_MOVE_POS_2000_LOW_WORD, MOTOR1_MOVE_POS_2000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//电机3回零
uint8_t Battery_15(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, 
            MOTOR_HOME_POSITION_LOW_WORD, 
            MOTOR_HOME_POSITION_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//电机2移动至位置314000
uint8_t Battery_16(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR2_MOVE_POS_314000_LOW_WORD, MOTOR2_MOVE_POS_314000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//电机2移动至位置299000
uint8_t Battery_17(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR2_MOVE_POS_299000_LOW_WORD, MOTOR2_MOVE_POS_299000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//电机2移动到位置303000
uint8_t Battery_18(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, OPEN_UAV_BATTERY_POS_LOW_WORD, OPEN_UAV_BATTERY_POS_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//电机2移动到位置293000
uint8_t Battery_19(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, NEAR_UAV_BATTERY_POS_LOW_WORD, NEAR_UAV_BATTERY_POS_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//MOTOR2_SLAVE_ADDR回零位
uint8_t Battery_20(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_HOME_POSITION_LOW_WORD, MOTOR_HOME_POSITION_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//飞机前进
//MOTOR5_SLAVE_ADDR和MOTOR6_SLAVE_ADDR移动到位置207000
//MOTOR7_SLAVE_ADDR和MOTOR8_SLAVE_ADDR移动到位置150000
uint8_t Battery_21(void)
{
    MotorControlParams motors[4] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR56_MOVE_POS_207000_LOW_WORD, MOTOR56_MOVE_POS_207000_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR56_MOVE_POS_207000_LOW_WORD, MOTOR56_MOVE_POS_207000_HIGH_WORD},
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR78_MOVE_POS_150000_LOW_WORD, MOTOR78_MOVE_POS_150000_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR78_MOVE_POS_150000_LOW_WORD, MOTOR78_MOVE_POS_150000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 4U, NULL);
}
//电机2回零
uint8_t Battery_22(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_HOME_POSITION_LOW_WORD, MOTOR_HOME_POSITION_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//移动到位置69000
uint8_t Battery_23(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR1_MOVE_POS_69000_LOW_WORD, MOTOR1_MOVE_POS_69000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//MOTOR1_SLAVE_ADDR回零
uint8_t Battery_24(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_HOME_POSITION_LOW_WORD, MOTOR_HOME_POSITION_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}

//移动至原点位置0
uint8_t Battery_25(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_HOME_POSITION_LOW_WORD, MOTOR_HOME_POSITION_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//移动至290000
uint8_t Battery_26(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR2_MOVE_POS_290000_LOW_WORD, MOTOR2_MOVE_POS_290000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
/* 电机3移动到绝对位置329000。 */
uint8_t Battery_27(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR1_MOVE_POS_329000_LOW_WORD, MOTOR1_MOVE_POS_329000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}
//电机2移动到位置293000
uint8_t Battery_28(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR2_MOVE_POS_293000_LOW_WORD, MOTOR2_MOVE_POS_293000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}

//电机3移动到位置293000
uint8_t Battery_29(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR1_MOVE_POS_293000_LOW_WORD, MOTOR1_MOVE_POS_293000_HIGH_WORD}
    };
    return MotorControl_BatchMove(motors, 1U, NULL);
}

uint8_t CloseDr(void)
{
    return Motor_Control(MOTOR_ID_4, 1U, MOTOR4_CMD_CLOSE);
}

uint8_t CheckAndCloseDoor(void)
{
    uint16_t uav_status = StatusRegs_Get(REG_RESERVED4);
    LOG_DEBUG("STEP", "door check: uav_status=%u\r\n",
              (unsigned int)uav_status);
    if (uav_status == 3) {
        LOG_INFO("STEP", "UAV absent; closing door\r\n");
        uint8_t ret = CloseDr();
        if (ret != MODBUS_RESULT_OK && ret != MODBUS_RESULT_PENDING &&
            ret != MODBUS_RESULT_BUSY) {
            LOG_ERROR("STEP", "close door failed: result=%u\r\n",
                      (unsigned int)ret);
        }
        return ret;
    } else {
        LOG_DEBUG("STEP", "door close skipped: uav_status=%u\r\n",
                  (unsigned int)uav_status);
    }
    return MODBUS_RESULT_OK;
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

// 飞机开机步骤表
const StepDef openfly_steps[] =
{
      {Center_1, 0, STATUS_REG_NONE, 0},        //1.左右居中
			{Center_2, 0, REG_CENTER_ROD_STATE, 4},       //2.前后居中
            //电机3移动到位置329000
			/* 电机3移动到绝对位置329000。 */
			{Battery_27, 0, STATUS_REG_NONE, 0},      //3.电机1上升  
            //飞机前进,4电机共同运动          
			{Battery_2, 0, STATUS_REG_NONE, 0},       //4.飞机前进
            //电机2移动到位置293000
			{Battery_28, 0, STATUS_REG_NONE, 0},      //5.电机2前进

            //电机2正向相对移动10000，移动到位置303000
			{Battery_18, 0, STATUS_REG_NONE, 0},      //27.电机2前进
            //电机2负向相对移动10000，移动到位置293000
			//电机2移动到位置293000
			{Battery_19, 0, STATUS_REG_NONE, 0},      //28.电机2后退            
			//电机2移动到位置303000
			{Battery_18, 0, STATUS_REG_NONE, 0},      //29.电机2前进
            //电机2移动至原点位置
			{Battery_25, 0, STATUS_REG_NONE, 0},     //30.电机2后退
            //飞机后退
			//待修改
			{Battery_6, 0, STATUS_REG_NONE, 0},       //31.飞机后退
            //电机3回零位
			//电机回零位
			{Battery_7, 0, STATUS_REG_NONE, 0},      //32.电机1下降
			//飞机前进
			{Battery_21, 0, STATUS_REG_NONE, 0},     //33.飞机前进
			{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},    //34.居中杆释放

};
const uint8_t OPENFLY_STEP_COUNT = (uint8_t)(sizeof(openfly_steps) / sizeof(openfly_steps[0]));

// 飞机关机步骤表
const StepDef closefly_steps[] =
{

            {Center_1, 0, STATUS_REG_NONE, 0},        //1.左右居中
			{Center_2, 0, REG_CENTER_ROD_STATE, 4},       //2.前后居中
			//电机1移动到绝对位置329000
            /* 电机3移动到绝对位置329000。 */
            {Battery_27, 0, STATUS_REG_NONE, 0},      //3.电机1上升
			//飞机前进
            {Battery_2, 0, STATUS_REG_NONE, 0},       //4.飞机前进
			//电机2移动到位置293000
            {Battery_28, 0, STATUS_REG_NONE, 0},      //5.电机2前进
			//电机2移动到位置303000
            {Battery_18, 0, STATUS_REG_NONE, 0},      //27.电机2前进
			//电机2移动到位置293000
            {Battery_19, 0, STATUS_REG_NONE, 0},      //28.电机2后退
			//电机2移动到位置303000
            {Battery_18, 0, STATUS_REG_NONE, 0},      //29.电机2前进
			//电机2回零
            //移动至原点位置0
            {Battery_25, 0, STATUS_REG_NONE, 0},     //30.电机2后退			
            //待修改
            {Battery_6, 0, STATUS_REG_NONE, 0},       //31.飞机后退
			//电机3回零
            {Battery_7, 0, STATUS_REG_NONE, 0},      //32.电机1下降
			//飞机前进
			{Battery_21, 0, STATUS_REG_NONE, 0},     //33.飞机前进
			{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},    //34.居中杆释放

};
const uint8_t CLOSEFLY_STEP_COUNT = (uint8_t)(sizeof(closefly_steps) / sizeof(closefly_steps[0]));

// 一键起飞步骤表(只将飞机开机)
const StepDef takeoff_steps_1[] =
{
	    {OpenDr, 15250, REG_DOOR_STATE, 2},     	  //1.打开舱门
//			{StopDr, 1000, REG_DOOR_STATE, 2},      		  //2.停止
			{CloseAC, 500, STATUS_REG_NONE, 0},            // 关闭空调
            {Center_1, 0, STATUS_REG_NONE, 0},        //1.左右居中
			{Center_2, 0, REG_CENTER_ROD_STATE, 4},     	  //2.前后居中
			/* 电机3移动到绝对位置329000。 */
			{Battery_27, 0, STATUS_REG_NONE, 0},      //3.电机1上升
			//飞机前进
			{Battery_2, 0, STATUS_REG_NONE, 0},       //4.飞机前进
			//电机2移动到位置293000
			{Battery_28, 0, STATUS_REG_NONE, 0},      //5.电机2前进
			//电机2移动到位置303000
			{Battery_18, 0, STATUS_REG_NONE, 0},      //27.电机2前进
			//电机2移动到位置293000
			{Battery_19, 0, STATUS_REG_NONE, 0},      //28.电机2后退
			//电机2移动到位置303000
			{Battery_18, 0, STATUS_REG_NONE, 0},      //29.电机2前进
			//移动至原点位置0
			{Battery_25, 0, STATUS_REG_NONE, 0},     //30.电机2后退
			//待修改
			{Battery_6, 0, STATUS_REG_NONE, 0},       //31.飞机后退
			//电机回零位
			{Battery_7, 0, STATUS_REG_NONE, 0},      //32.电机1下降
			//飞机前进
			{Battery_21, 0, STATUS_REG_NONE, 0},     //33.飞机前进
			{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},   	  //34.居中杆释放
			{CloseAC, 16000, STATUS_REG_NONE, 0},            // 关闭空调
//      {CheckAndCloseDoor, 16250, STATUS_REG_NONE, 0},     	  //1.关闭舱门
			{StopDr, 1000, REG_DOOR_STATE, 4},      		  //2.停止

};
const uint8_t TAKEOFF_STEPS_1_COUNT = (uint8_t)(sizeof(takeoff_steps_1) / sizeof(takeoff_steps_1[0]));


// 一键起飞步骤表(只将飞机开机)
const StepDef takeoff_steps_2[] =
{
	    {OpenDr, 15250, REG_DOOR_STATE, 2},     	  //1.打开舱门
//			{StopDr, 1000, REG_DOOR_STATE, 2},      		  //2.停止
			{CloseAC, 500, STATUS_REG_NONE, 0},            // 关闭空调
            {Center_1, 0, STATUS_REG_NONE, 0},        //1.左右居中
			{Center_2, 0, REG_CENTER_ROD_STATE, 4},     	  //2.前后居中
			/* 电机3移动到绝对位置329000。 */
			{Battery_27, 0, STATUS_REG_NONE, 0},      //3.电机1上升
			//飞机前进
			{Battery_2, 0, STATUS_REG_NONE, 0},       //4.飞机前进
			//电机2移动到位置293000
			{Battery_28, 0, STATUS_REG_NONE, 0},      //5.电机2前进
			//电机2移动到位置303000
			{Battery_18, 0, STATUS_REG_NONE, 0},      //27.电机2前进
			//电机2移动到位置293000
			{Battery_19, 0, STATUS_REG_NONE, 0},      //28.电机2后退
			//电机2移动到位置303000
			{Battery_18, 0, STATUS_REG_NONE, 0},      //29.电机2前进
			//电机2移动至原点位置0
			{Battery_25, 0, STATUS_REG_NONE, 0},     //30.电机2后退
			//待修改
			{Battery_6, 0, STATUS_REG_NONE, 0},       //31.飞机后退
			//电机回零位
			{Battery_7, 0, STATUS_REG_NONE, 0},      //32.电机1下降
			//飞机前进
			{Battery_21, 0, STATUS_REG_NONE, 0},     //33.飞机前进
			{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},   	  //34.居中杆释放
			{CloseAC, 16000, STATUS_REG_NONE, 0},            // 关闭空调
//      {CheckAndCloseDoor, 16250, STATUS_REG_NONE, 0},     	  //1.关闭舱门
			{StopDr, 1000, REG_DOOR_STATE, 4},      		  //2.停止
};
const uint8_t TAKEOFF_STEPS_2_COUNT = (uint8_t)(sizeof(takeoff_steps_2) / sizeof(takeoff_steps_2[0]));


// 一键起飞步骤表(只将飞机开机)
const StepDef takeoff_steps_3[] =
{
	    {OpenDr, 15250, REG_DOOR_STATE, 2},     	  //1.打开舱门
//			{StopDr, 1000, REG_DOOR_STATE, 2},      		  //2.停止
			{CloseAC, 500, STATUS_REG_NONE, 0},            // 关闭空调
			{StopDr, 1000, REG_DOOR_STATE, 2},      		  //2.停止
      {Center_1, 0, STATUS_REG_NONE, 0},        //1.左右居中
			{Center_2, 0, REG_CENTER_ROD_STATE, 4},     	  //2.前后居中
			/* 电机3移动到绝对位置329000。 */
			{Battery_27, 0, STATUS_REG_NONE, 0},      //3.电机1上升
			//飞机前进
			{Battery_2, 0, STATUS_REG_NONE, 0},       //4.飞机前进
			//电机2移动到位置293000
			{Battery_28, 0, STATUS_REG_NONE, 0},      //5.电机2前进
			//电机2移动到位置303000
			{Battery_18, 0, STATUS_REG_NONE, 0},      //27.电机2前进
			//电机2移动到位置293000
			{Battery_19, 0, STATUS_REG_NONE, 0},      //28.电机2后退
			//电机2移动到位置303000
			{Battery_18, 0, STATUS_REG_NONE, 0},      //29.电机2前进
			//电机2移动至原点位置0
			{Battery_25, 0, STATUS_REG_NONE, 0},     //30.电机2后退
			//待修改
			{Battery_6, 0, STATUS_REG_NONE, 0},       //31.飞机后退
			//电机3回零位
			{Battery_7, 0, STATUS_REG_NONE, 0},      //32.电机1下降
			//飞机前进
			{Battery_21, 0, STATUS_REG_NONE, 0},     //33.飞机前进
			{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},   	  //34.居中杆释放
			{CloseAC, 18000, STATUS_REG_NONE, 0},            // 关闭空调
//      {CheckAndCloseDoor, 16250, STATUS_REG_NONE, 0},     	  //1.关闭舱门
			{StopDr, 1000, REG_DOOR_STATE, 4},      		  //2.停止

};
const uint8_t TAKEOFF_STEPS_3_COUNT = (uint8_t)(sizeof(takeoff_steps_3) / sizeof(takeoff_steps_3[0]));

// 降落完成步骤表（取电换电都完成）
const StepDef landing_steps_1[] =
{
			//下电池步骤（1号空仓，将飞机电池放入1号仓）（取电装电都完成）
            {Center_1, 0, STATUS_REG_NONE, 0},        //1.左右居中
			{Center_2, 0, REG_CENTER_ROD_STATE, 4},      		//2.前后居中
			//电机3运动到330000
            {Battery_1, 0, STATUS_REG_NONE, 0},      //3.电机1上升
			//飞机前进
			{Battery_2, 0, STATUS_REG_NONE, 0},       //4.飞机前进
			//电机2运动到315000
            {Battery_3, 0, STATUS_REG_NONE, 0},      //5.电机2前进            
            //电机夹紧
            {Battery_4, 1000, STATUS_REG_NONE, 0},       //6.电机3夹紧
			//电机2回到零位
            //回到零位
            {Battery_5, 0, STATUS_REG_NONE, 0},      //7.电机2后退
			//待修改
			{Battery_6, 0, STATUS_REG_NONE, 0},       //8.飞机后退
            //电机3回到0位，可省略
			//{Battery_7, 0, STATUS_REG_NONE, 0},      //9.电机1下降
            //电机3运动到位置136000
			//运动到位置136000
			{Battery_8, 0, STATUS_REG_NONE, 0},      //10.电机1上升
            //电机2运动到361000
			//电机2运动到位置361000
			{Battery_9, 0, STATUS_REG_NONE, 0},      //11.电机2前进
			//夹爪松开
			{Battery_10, 1000, STATUS_REG_NONE, 0},       //12.电机3松开
            //电机2前移9000，移动至370000
            //移动至位置370000
            {Battery_11, 0, STATUS_REG_NONE, 0},      //13.电机2前进
            //电机2回到零点
			//MOTOR2_SLAVE_ADDR回到零点
			{Battery_12, 0, STATUS_REG_NONE, 0},     //14.电机2后退
            //电机3下移136000，可省略
			//MOTOR1_SLAVE_ADDR回到零点
			//{Battery_13, 0, REG_SWAP_MECH_STATE, 2},     //15.电机1下降
                                                        //装电池步骤(从2号仓取电池)
            //电机3移动到位置69000
			//移动到位置69000
			{Battery_23, 0, STATUS_REG_NONE, 0},      //16.电机1上升
            //电机2运动到位置361000
			{Battery_9, 0, STATUS_REG_NONE, 0},      //17.电机2前进
			//电机夹紧
			{Battery_4, 1000, STATUS_REG_NONE, 0},       //18.电机3夹紧
			//电机2运动到363000,电机回零，可省略
            //电机2回零
            {Battery_22, 0, STATUS_REG_NONE, 0},     //19.电机2后退
			//电机3回零，可省略
            //{Battery_24, 0, STATUS_REG_NONE, 0},      //20.电机1下降
            //电机3运动到330000
			{Battery_1, 0, STATUS_REG_NONE, 0},      //21.电机1上升
            //飞机前进
            {Battery_2, 0, STATUS_REG_NONE, 0},       //22.飞机前进
			//电机2移动至290000
            //移动至290000
            {Battery_26, 0, STATUS_REG_NONE, 0},      //23.电机2前进
			//夹爪松开
			{Battery_10, 1000, STATUS_REG_NONE, 0},      //24.电机3松开
			//电机2前进24000，移动至位置314000
            //电机2移动至位置314000
            {Battery_16, 0, STATUS_REG_NONE, 0},      //25.电机2前进
			//电机2后退308000，回零位
            //MOTOR2_SLAVE_ADDR回零位
            {Battery_20, 0, STATUS_REG_NONE, 0},     //7.电机2后退
			//待修改
			{Battery_6, 0, STATUS_REG_NONE, 0},       //31.飞机后退
            //电机下降
			//电机回零位
			{Battery_7, 0, STATUS_REG_NONE, 0},      //32.电机1下降
			//飞机前进
			{Battery_21, 0, REG_SWAP_MECH_STATE, 4},     	//33.飞机前进
			{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},   		//34.居中杆释放
      {CloseDr, 16250, REG_DOOR_STATE, 4},     	  //1.关闭舱门
//			{StopDr, 1000, REG_DOOR_STATE, 4},        		//2.停止
			{OpenAC, 500, STATUS_REG_NONE, 0},             // 打开空调
			{UpdateEmptyBay, 500, STATUS_REG_NONE, 0},     // 最后一步更新空仓号

};
const uint8_t LANDING_STEPS_1_COUNT = (uint8_t)(sizeof(landing_steps_1) / sizeof(landing_steps_1[0]));

// 降落完成步骤表（2号空仓，将飞机电池放入2号仓）（取电装电都完成）
const StepDef landing_steps_2[] =
{
			//下电池步骤(放入2号仓)
      {Center_1, 0, STATUS_REG_NONE, 0},        //1.左右居中
			{Center_2, 0, REG_CENTER_ROD_STATE, 4},        //2.前后居中
			//电机3运动到330000
            {Battery_1, 0, STATUS_REG_NONE, 0},      //3.电机1上升
			//飞机前进
			{Battery_2, 0, STATUS_REG_NONE, 0},       //4.飞机前进
			//电机2运动到315000
			{Battery_3, 0, STATUS_REG_NONE, 0},      //5.电机2前进
			//电机夹紧
			{Battery_4, 1000, STATUS_REG_NONE, 0},        //6.电机3夹紧
			//回到零位
			{Battery_5, 0, STATUS_REG_NONE, 0},      //7.电机2后退
			//待修改
			{Battery_6, 0, STATUS_REG_NONE, 0},       //8.飞机后退
			//电机回零位
			//{Battery_7, 0, STATUS_REG_NONE, 0},      //9.电机1下降
			//移动到位置69000
			{Battery_23, 0, STATUS_REG_NONE, 0},       //10.电机1上升
			//电机2运动到位置361000
			{Battery_9, 0, STATUS_REG_NONE, 0},      //11.电机2前进
			//夹爪松开
			{Battery_10, 1000, STATUS_REG_NONE, 0},       //12.电机3松开
			//移动至位置370000
			{Battery_11, 0, STATUS_REG_NONE, 0},      //13.电机2前进
			//MOTOR2_SLAVE_ADDR回到零点
			{Battery_12, 0, STATUS_REG_NONE, 0},     //14.电机2后退
			//MOTOR1_SLAVE_ADDR回零
			//{Battery_24, 0, REG_SWAP_MECH_STATE, 2},      //15.电机1下降
			//装电池步骤(从3号仓取电池)
		  //MOTOR1_SLAVE_ADDR上升到位置2000
		  {Battery_14, 0, STATUS_REG_NONE, 0},       //16.电机1上升
			//电机2运动到位置361000
			{Battery_9, 0, STATUS_REG_NONE, 0},       //17.电机2前进
			//电机夹紧
			{Battery_4, 1000, STATUS_REG_NONE, 0},         //18.电机3夹紧
			//电机2回零
			{Battery_22, 0, STATUS_REG_NONE, 0},       //19.电机2后退
			//电机3回零
			//{Battery_15, 0, STATUS_REG_NONE, 0},       //20.电机1下降
			//电机3运动到330000
			{Battery_1, 0, STATUS_REG_NONE, 0},       //21.电机1上升
      //飞机前进
      {Battery_2, 0, STATUS_REG_NONE, 0},        //22.飞机前进
			//移动至290000
			{Battery_26, 0, STATUS_REG_NONE, 0},       //23.电机2前进
			//夹爪松开
			{Battery_10, 1000, STATUS_REG_NONE, 0},        //24.电机3松开
			//电机2移动至位置314000
			{Battery_16, 0, STATUS_REG_NONE, 0},       //25.电机2前进
			//MOTOR2_SLAVE_ADDR回零位
			{Battery_20, 0, STATUS_REG_NONE, 0},      //7.电机2后退
			//待修改
			{Battery_6, 0, STATUS_REG_NONE, 0},        //31.飞机后退
			//电机回零位
			{Battery_7, 0, STATUS_REG_NONE, 0},        //32.电机1下降
			//飞机前进
			{Battery_21, 0, REG_SWAP_MECH_STATE, 4},      //33.飞机前进

			{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},     //34.居中杆释放
      {CloseDr, 16250, REG_DOOR_STATE, 4},     	  //1.关闭舱门
//			{StopDr, 1000, REG_DOOR_STATE, 4},        		//2.停止
			{OpenAC, 500, STATUS_REG_NONE, 0},           // 打开空调
			{UpdateEmptyBay, 500, STATUS_REG_NONE, 0},   // 最后一步更新空仓号

};
const uint8_t LANDING_STEPS_2_COUNT = (uint8_t)(sizeof(landing_steps_2) / sizeof(landing_steps_2[0]));

// 降落完成步骤表（3号空仓，将飞机电池放入3号仓）（取电装电都完成）
const StepDef landing_steps_3[] =
{
			//下电池步骤(放入3号仓)
      {Center_1, 0, STATUS_REG_NONE, 0},        //1.左右居中
			{Center_2, 0, REG_CENTER_ROD_STATE, 4},       //2.前后居中
			//电机3运动到330000
			{Battery_1, 0, STATUS_REG_NONE, 0},      //3.电机1上升
			//飞机前进
			{Battery_2, 0, STATUS_REG_NONE, 0},       //4.飞机前进
			//电机2运动到315000
			{Battery_3, 0, STATUS_REG_NONE, 0},      //5.电机2前进
			//电机夹紧
			{Battery_4, 1000, STATUS_REG_NONE, 0},        //6.电机3夹紧
			//回到零位
			{Battery_5, 0, STATUS_REG_NONE, 0},      //7.电机2后退
			//待修改
			{Battery_6, 0, STATUS_REG_NONE, 0},       //8.飞机后退
			//电机回零位
			//{Battery_7, 0, STATUS_REG_NONE, 0},      //9.电机1下降
			//MOTOR1_SLAVE_ADDR上升到位置2000
			{Battery_14, 0, STATUS_REG_NONE, 0},       //10.电机1上升
			//电机2运动到位置361000
			{Battery_9, 0, STATUS_REG_NONE, 0},      //11.电机2前进
			//夹爪松开
			{Battery_10, 1000, STATUS_REG_NONE, 0},       //12.电机3松开
			//移动至位置370000
			{Battery_11, 0, STATUS_REG_NONE, 0},      //13.电机2前进
			//MOTOR2_SLAVE_ADDR回到零点
			{Battery_12, 0, STATUS_REG_NONE, 0},     //14.电机2后退
			//电机3回零
			//{Battery_15, 0, REG_SWAP_MECH_STATE, 2},      //15.电机1下降
			//装电池步骤(从1号仓取电池)
			//运动到位置136000
			{Battery_8, 0, STATUS_REG_NONE, 0},       //16.电机1上升
			//电机2运动到位置361000
			{Battery_9, 0, STATUS_REG_NONE, 0},       //17.电机2前进
			//电机夹紧
			{Battery_4, 1000, STATUS_REG_NONE, 0},         //18.电机3夹紧
			//电机2回零
			{Battery_22, 0, STATUS_REG_NONE, 0},       //19.电机2后退
			//MOTOR1_SLAVE_ADDR回到零点
			//{Battery_13, 0, STATUS_REG_NONE, 0},       //20.电机1下降
			//电机3运动到330000
			{Battery_1, 0, STATUS_REG_NONE, 0},       //21.电机1上升
      //飞机前进
      {Battery_2, 0, STATUS_REG_NONE, 0},        //22.飞机前进
			//移动至290000
			{Battery_26, 0, STATUS_REG_NONE, 0},       //23.电机2前进
			//夹爪松开
			{Battery_10, 1000, STATUS_REG_NONE, 0},        //24.电机3松开
			//电机2移动至位置314000
			{Battery_16, 0, STATUS_REG_NONE, 0},       //25.电机2前进
			//MOTOR2_SLAVE_ADDR回零位
			{Battery_20, 0, STATUS_REG_NONE, 0},      //7.电机2后退
			//待修改
			{Battery_6, 0, STATUS_REG_NONE, 0},        //31.飞机后退
			//电机回零位
			{Battery_7, 0, STATUS_REG_NONE, 0},        //32.电机1下降

			//飞机前进
			{Battery_21, 0, REG_SWAP_MECH_STATE, 4},      //33.飞机前进
			{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},     //34.居中杆释放
            {CloseDr, 16250, REG_DOOR_STATE, 4},     	  //1.关闭舱门
//			{StopDr, 1000, REG_DOOR_STATE, 4},        		//2.停止
			{OpenAC, 500, STATUS_REG_NONE, 0},             // 打开空调
			{UpdateEmptyBay, 500, STATUS_REG_NONE, 0},   // 最后一步更新空仓号

};
const uint8_t LANDING_STEPS_3_COUNT = (uint8_t)(sizeof(landing_steps_3) / sizeof(landing_steps_3[0]));

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

// 装电池步骤（取1号仓电池）
const StepDef loadbattery_steps_1[] =
{
		{Center_1, 0, STATUS_REG_NONE, 0},        //1.左右居中
		{Center_2, 0, REG_CENTER_ROD_STATE, 4},       //2.前后居中
        //电机3移动到136000
		//运动到位置136000
		{Battery_8, 0, STATUS_REG_NONE, 0},       //16.电机1上升
        //电机2移动到361000
		//电机2运动到位置361000
		{Battery_9, 0, STATUS_REG_NONE, 0},       //17.电机2前进		
        //电机夹紧
        {Battery_4, 1000, STATUS_REG_NONE, 0},         //18.电机3夹紧
        //电机2回到限位
		//电机2回零
		{Battery_22, 0, STATUS_REG_NONE, 0},       //19.电机2后退
        //电机3下降到136000，此步骤可省略
		//MOTOR1_SLAVE_ADDR回到零点
		//{Battery_13, 0, STATUS_REG_NONE, 0},       //20.电机1下降
        //电机3移动到330000
		//电机3运动到330000
		{Battery_1, 0, STATUS_REG_NONE, 0},       //21.电机1上升        
		//飞机前进
		{Battery_2, 0, STATUS_REG_NONE, 0},        //22.飞机前进
        //电机2移动到315000
		//电机2运动到315000
		{Battery_3, 0, STATUS_REG_NONE, 0},       //23.电机2前进
		//夹爪松开
		{Battery_10, 1000, STATUS_REG_NONE, 0},        //24.电机3松开
        //电机2移动到24000
		//电机2移动至位置314000
		{Battery_16, 0, STATUS_REG_NONE, 0},       //25.电机2前进
        //电机2后退15000脉冲
		//电机2移动至位置299000
		{Battery_17, 0, STATUS_REG_NONE, 0},        //26.电机2后退
        //电机2回原点，需修改
		//回到零位
		{Battery_5, 0, STATUS_REG_NONE, 0},      //7.电机2后退

		//待修改
		{Battery_6, 0, STATUS_REG_NONE, 0},        //31.飞机后退
        //电机3回原点，需修改
		//电机回零位
		{Battery_7, 0, STATUS_REG_NONE, 0},        //32.电机1下降
		//飞机前进
		{Battery_21, 0, REG_SWAP_MECH_STATE, 4},      //33.飞机前进
		{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},     //34.居中杆释放
};
const uint8_t LOADBATTERY_STEPS_1_COUNT = (uint8_t)(sizeof(loadbattery_steps_1) / sizeof(loadbattery_steps_1[0]));



// 装电池步骤（取2号仓电池）
const StepDef loadbattery_steps_2[] =
{
		{Center_1, 0, STATUS_REG_NONE, 0},        //1.左右居中
		{Center_2, 0, REG_CENTER_ROD_STATE, 4},       //2.前后居中
		//电机2移动到位置69000
		{Battery_23, 0, STATUS_REG_NONE, 0},       //16.电机1上升
		//电机2运动到位置361000
		{Battery_9, 0, STATUS_REG_NONE, 0},       //17.电机2前进
		//电机夹紧
		{Battery_4, 1000, STATUS_REG_NONE, 0},         //18.电机3夹紧
		//电机2回零
		{Battery_22, 0, STATUS_REG_NONE, 0},       //19.电机2后退
		//MOTOR1_SLAVE_ADDR回零
		//{Battery_24, 0, STATUS_REG_NONE, 0},       //20.电机1下降
		//电机3运动到330000
		{Battery_1, 0, STATUS_REG_NONE, 0},       //21.电机1上升
		//飞机前进
		{Battery_2, 0, STATUS_REG_NONE, 0},        //22.飞机前进
		//电机2运动到315000
		{Battery_3, 0, STATUS_REG_NONE, 0},       //23.电机2前进
		//夹爪松开
		{Battery_10, 1000, STATUS_REG_NONE, 0},        //24.电机3松开
		//电机2移动至位置314000
		{Battery_16, 0, STATUS_REG_NONE, 0},       //25.电机2前进
		//电机2移动至位置299000
		{Battery_17, 0, STATUS_REG_NONE, 0},        //26.电机2后退
		//回到零位
		{Battery_5, 0, STATUS_REG_NONE, 0},      //7.电机2后退
		//待修改
		{Battery_6, 0, STATUS_REG_NONE, 0},        //31.飞机后退
		//电机回零位
		{Battery_7, 0, STATUS_REG_NONE, 0},        //32.电机1下降
		//飞机前进
		{Battery_21, 0, REG_SWAP_MECH_STATE, 4},      //33.飞机前进
		{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},     //34.居中杆释放
};
const uint8_t LOADBATTERY_STEPS_2_COUNT = (uint8_t)(sizeof(loadbattery_steps_2) / sizeof(loadbattery_steps_2[0]));


// 装电池步骤（取3号仓电池）
const StepDef loadbattery_steps_3[] =
{
      {Center_1, 0, STATUS_REG_NONE, 0},        //1.左右居中
			{Center_2, 0, REG_CENTER_ROD_STATE, 4},       //2.前后居中
            //电机3上升到位置2000
            //MOTOR1_SLAVE_ADDR上升到位置2000
            {Battery_14, 0, STATUS_REG_NONE, 0},       //16.电机1上升
            //电机2运动到位置361000
			{Battery_9, 0, STATUS_REG_NONE, 0},       //17.电机2前进
			//电机夹紧
			{Battery_4, 1000, STATUS_REG_NONE, 0},         //18.电机3夹紧
            //电机2回零
			{Battery_22, 0, STATUS_REG_NONE, 0},       //19.电机2后退
            //电机3回零
			//{Battery_15, 0, STATUS_REG_NONE, 0},       //20.电机1下降
            //电机3运动到330000
			{Battery_1, 0, STATUS_REG_NONE, 0},       //21.电机1上升
            //飞机前进
            {Battery_2, 0, STATUS_REG_NONE, 0},        //22.飞机前进
            //电机2运动到315000
			{Battery_3, 0, STATUS_REG_NONE, 0},       //23.电机2前进
			//夹爪松开
			{Battery_10, 1000, STATUS_REG_NONE, 0},        //24.电机3松开
            //电机2移动至位置314000
			{Battery_16, 0, STATUS_REG_NONE, 0},       //25.电机2前进
            //电机2后退15000，到位置299000
			//电机2移动至位置299000
			{Battery_17, 0, STATUS_REG_NONE, 0},        //26.电机2后退
			//回到零位
			{Battery_5, 0, STATUS_REG_NONE, 0},      //7.电机2后退
			//待修改
			{Battery_6, 0, STATUS_REG_NONE, 0},        //31.飞机后退
			//电机回零位
			{Battery_7, 0, STATUS_REG_NONE, 0},        //32.电机1下降
			//飞机前进
			{Battery_21, 0, REG_SWAP_MECH_STATE, 4},      //33.飞机前进
			{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},     //34.居中杆释放
};
const uint8_t LOADBATTERY_STEPS_3_COUNT = (uint8_t)(sizeof(loadbattery_steps_3) / sizeof(loadbattery_steps_3[0]));


// 下电池步骤(放入1号仓)
const StepDef downbattery_steps_1[] =
{
            {Center_1, 0, STATUS_REG_NONE, 0},        //1.左右居中
			{Center_2, 0, REG_CENTER_ROD_STATE, 4},       //2.前后居中
			//电机3运动到330000
			{Battery_1, 0, STATUS_REG_NONE, 0},      //3.电机1上升
			//飞机前进
			{Battery_2, 0, STATUS_REG_NONE, 0},       //4.飞机前进
			//电机2运动到315000
			{Battery_3, 0, STATUS_REG_NONE, 0},      //5.电机2前进
			//电机夹紧
			{Battery_4, 1000, STATUS_REG_NONE, 0},        //6.电机3夹紧
			//回到零位
			{Battery_5, 0, STATUS_REG_NONE, 0},      //7.电机2后退
			//待修改
			{Battery_6, 0, STATUS_REG_NONE, 0},       //8.飞机后退

			//电机回零位
			//{Battery_7, 0, STATUS_REG_NONE, 0},      //9.电机1下降
			//运动到位置136000
			{Battery_8, 0, STATUS_REG_NONE, 0},       //10.电机1上升
			//电机2运动到位置361000
			{Battery_9, 0, STATUS_REG_NONE, 0},      //11.电机2前进
			//夹爪松开
			{Battery_10, 1000, STATUS_REG_NONE, 0},       //12.电机3松开
			//移动至位置370000
			{Battery_11, 0, STATUS_REG_NONE, 0},      //13.电机2前进
			//MOTOR2_SLAVE_ADDR回到零点
			{Battery_12, 0, STATUS_REG_NONE, 0},     //14.电机2后退
			//MOTOR1_SLAVE_ADDR回到零点
			{Battery_13, 0, STATUS_REG_NONE, 0},      //15.电机1下降
			//飞机前进
			{Battery_21, 0, REG_SWAP_MECH_STATE, 2},      //33.飞机前进

			{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},     //34.居中杆释放
};
const uint8_t DOWNBATTERY_STEPS_1_COUNT = (uint8_t)(sizeof(downbattery_steps_1) / sizeof(downbattery_steps_1[0]));

// 下电池步骤(放入2号仓)
const StepDef downbattery_steps_2[] =
{
      {Center_1, 0, STATUS_REG_NONE, 0},        //1.左右居中
			{Center_2, 0, REG_CENTER_ROD_STATE, 4},       //2.前后居中
			//电机3运动到330000
			{Battery_1, 0, STATUS_REG_NONE, 0},      //3.电机1上升
			//飞机前进
			{Battery_2, 0, STATUS_REG_NONE, 0},       //4.飞机前进
			//电机2运动到315000
			{Battery_3, 0, STATUS_REG_NONE, 0},      //5.电机2前进
			//电机夹紧
			{Battery_4, 1000, STATUS_REG_NONE, 0},        //6.电机3夹紧
			//回到零位
			{Battery_5, 0, STATUS_REG_NONE, 0},      //7.电机2后退
			//待修改
			{Battery_6, 0, STATUS_REG_NONE, 0},       //8.飞机后退
			//电机回零位
			//{Battery_7, 0, STATUS_REG_NONE, 0},      //9.电机1下降
			//移动到位置69000
			{Battery_23, 0, STATUS_REG_NONE, 0},       //10.电机1上升
			//电机2运动到位置361000
			{Battery_9, 0, STATUS_REG_NONE, 0},      //11.电机2前进
			//夹爪松开
			{Battery_10, 1000, STATUS_REG_NONE, 0},       //12.电机3松开
			//移动至位置370000
			{Battery_11, 0, STATUS_REG_NONE, 0},      //13.电机2前进
			//MOTOR2_SLAVE_ADDR回到零点
			{Battery_12, 0, STATUS_REG_NONE, 0},     //14.电机2后退
			//MOTOR1_SLAVE_ADDR回零
			{Battery_24, 0, STATUS_REG_NONE, 0},      //15.电机1下降
			//飞机前进
			{Battery_21, 0, REG_SWAP_MECH_STATE, 2},      //33.飞机前进
			{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},     //34.居中杆释放
};
const uint8_t DOWNBATTERY_STEPS_2_COUNT = (uint8_t)(sizeof(downbattery_steps_2) / sizeof(downbattery_steps_2[0]));

// 下电池步骤(放入3号仓)
const StepDef downbattery_steps_3[] =
{
      {Center_1, 0, STATUS_REG_NONE, 0},        //1.左右居中
			{Center_2, 0, REG_CENTER_ROD_STATE, 4},       //2.前后居中
			//电机3运动到330000
			{Battery_1, 0, STATUS_REG_NONE, 0},      //3.电机1上升
			//飞机前进
			{Battery_2, 0, STATUS_REG_NONE, 0},       //4.飞机前进
			//电机2运动到315000
			{Battery_3, 0, STATUS_REG_NONE, 0},      //5.电机2前进
			//电机夹紧
			{Battery_4, 1000, STATUS_REG_NONE, 0},        //6.电机3夹紧
			//回到零位
			{Battery_5, 0, STATUS_REG_NONE, 0},      //7.电机2后退
			//待修改
			{Battery_6, 0, STATUS_REG_NONE, 0},       //8.飞机后退
			//电机回零位
			//{Battery_7, 0, STATUS_REG_NONE, 0},      //9.电机1下降
			//MOTOR1_SLAVE_ADDR上升到位置2000
			{Battery_14, 0, STATUS_REG_NONE, 0},       //10.电机1上升
			//电机2运动到位置361000
			{Battery_9, 0, STATUS_REG_NONE, 0},      //11.电机2前进
			//夹爪松开
			{Battery_10, 1000, STATUS_REG_NONE, 0},       //12.电机3松开
			//移动至位置370000
			{Battery_11, 0, STATUS_REG_NONE, 0},      //13.电机2前进
			//MOTOR2_SLAVE_ADDR回到零点
			{Battery_12, 0, STATUS_REG_NONE, 0},     //14.电机2后退
			//电机3回零
			{Battery_15, 0, STATUS_REG_NONE, 0},      //15.电机1下降
			//飞机前进
			{Battery_21, 0, REG_SWAP_MECH_STATE, 2},      //33.飞机前进
			{LeaveCenter, 0, REG_CENTER_ROD_STATE, 2},     //34.居中杆释放
};
const uint8_t DOWNBATTERY_STEPS_3_COUNT = (uint8_t)(sizeof(downbattery_steps_3) / sizeof(downbattery_steps_3[0]));

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
    {0, 2, motors_battery1},   // Battery_8 / 23 / 14
    {0, 3, motors_battery2},   // Battery_9
    {0, 4, motors_battery3},   // Battery_4
    {0, 5, motors_battery2},   // Battery_22
    {0, 6, motors_battery1},   // Battery_13 / 24 / 15
    {0, 7, motors_battery1},   // Battery_1
    {0, 8, motors_battery4},   // Battery_2
    {0, 9, motors_battery2},   // Battery_3
    {0,10, motors_battery3},   // Battery_10
    {0,11, motors_battery2},   // Battery_16
    {0,12, motors_battery2},   // Battery_17
    {0,13, motors_battery2},   // Battery_5
    {0,14, motors_battery4},   // Battery_6
    {0,15, motors_battery1},   // Battery_7
    {0,16, motors_battery4},   // Battery_21
    {0,17, motors_leave},      // LeaveCenter
    {0,18, motors_none},       // UpdateEmptyBay

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
