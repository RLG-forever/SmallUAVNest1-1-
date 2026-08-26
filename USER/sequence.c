#include "sequence.h"
#include "motor_service.h"
#include "modbus_master.h"
#include "motor_config.h"
#include "modbus_common.h"
#include "status_service.h"
#include "battery_swap.h"
#include "relay.h"
#include "tick.h"
#include "bsp_timer.h"
#include <stdio.h>
#include <string.h>

volatile uint8_t relay_stop = 0;
// 外部主站写函数

// 序列控制变量
static volatile uint8_t seq_active = 0;       // 是否有序列正在执行
static volatile uint8_t seq_paused = 0;       // 1=暂停，0=运行
static uint8_t current_seq_id;                // 当前序列ID
static uint8_t current_step;                  // 当前步骤索引（0开始）
static uint8_t action_in_progress = 0;        // 是否有电机动作正在执行

// 步骤定义
typedef struct {
    uint8_t motor_addr;   // 电机从站地址
    uint16_t reg_addr;    // 寄存器地址
    uint16_t reg_data;    // 写入值
    uint16_t check_reg;   // 用于检查完成的状态寄存器地址
    uint16_t expected_val;// 期望完成时的值
} SequenceStep;

// 当前运行的序列
struct {
    SeqId id;
    const StepDef *steps;
    uint8_t step_count;
    uint8_t current_index;
    uint32_t wait_until;
    uint8_t busy;          // 是否正在执行
    uint8_t error;         // 错误标志
		uint8_t paused;        // 暂停标志
		StatusServiceField pending_update_field;
    uint16_t pending_update_value;
		uint8_t retry_count;   // 新增：当前序列的重试次数
}seq_runner;

// 步骤电机列表映射
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

volatile uint8_t g_step_timer_expired = 0;   // 定时器超时标志

// 定时器回调函数（在中断上下文中执行）
static void StepTimerCallback(void)
{
    g_step_timer_expired = 1;
}

static void Sequence_Finish(uint8_t error)
{
    bsp_StopHardTimer(1U);
    g_step_timer_expired = 0U;
    seq_runner.wait_until = 0U;
    seq_runner.pending_update_field = STATUS_FIELD_NONE;
    seq_runner.busy = 0U;
    seq_runner.error = error;
    seq_runner.paused = 0U;
    StatusService_ReleaseSnapshot();
}

// 暂停序列
void Sequence_Pause(void) {
    if (seq_runner.busy && !seq_runner.paused) {
        seq_runner.paused = 1;
        printf("序列已暂停\r\n");
    }
}

// 恢复序列
void Sequence_Resume(void) {
    if (seq_runner.busy && seq_runner.paused)
		{
        seq_runner.paused = 0;
        printf("序列已恢复\r\n");
        // 若当前没有等待计时器（即处于步骤边界），下一次 Sequence_Process 会立即执行下一步
        // 若正处于等待期间，等待结束后也会自动继续
    }
}

// ---------- 飞机降落步骤函数定义 ----------

//1.打开舱门
static uint8_t OpenDoor(void)
{
//	  uint8_t ret;
//    ret = Motor_Control(MOTOR1_SLAVE_ADDR, 8, MOTOR4_SPEED_VALUE);
//    if (ret != 0) return ret;
//    master_state = MASTER_IDLE;
//    timeout_cnt = 0;
    return MODBUS_RESULT_PARAM;
}

//1.关闭舱门
static uint8_t CloseDoor(void)
{
//	  uint8_t ret;
//    ret = Motor_Control(MOTOR1_SLAVE_ADDR, 8, MOTOR4_SPEED_VALUE);
//    if (ret != 0) return ret;
//    master_state = MASTER_IDLE;
//    timeout_cnt = 0;
    return MODBUS_RESULT_PARAM;
}




//2.电机1上升1
static uint8_t Motor1Up1(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_PRESET_PULSE_02, MOTOR_PRESET_PULSE_01}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

//3.飞机前进

//4.电机2前进
static uint8_t Motor2Forward(void)
{
    uint16_t commands[5] = {
        MOTOR2_DIRECTION_FORWARD,
        MOTOR2_SPEED_VALUE,
        MOTOR_PRESET_PULSE_11,
        MOTOR_PRESET_PULSE_12,
        MOTOR2_POSITION_MODE_RELATIVE
    };

    return Motor_Batch_Control(MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG5,
                               5U, commands);
}

//5.电机3夹住电池
static uint8_t Motor3Clamp(void)
{
    return Motor_Control(MOTOR_ID_3, 2U, MOTOR3_CMD_CLAMP);
}

//6.飞机后退

//7.电机2后退
static uint8_t Motor2Back(void)
{
    uint16_t commands[5] = {
        MOTOR2_DIRECTION_REVERSE,
        MOTOR2_SPEED_VALUE,
        MOTOR_PRESET_PULSE_11,
        MOTOR_PRESET_PULSE_12,
        MOTOR2_POSITION_MODE_RELATIVE
    };

    return Motor_Batch_Control(MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG5,
                               5U, commands);
}

//8.电机1下降
static uint8_t Motor1Down1(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_PRESET_PULSE_04, MOTOR_PRESET_PULSE_03}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

//9.电机2前进

//10.电机3松开电池
static uint8_t Motor3Lossen(void)
{
    return Motor_Control(MOTOR_ID_3, 1U, MOTOR3_CMD_RELEASE);
}

//11.电机2后退

//12.电机1上升2
static uint8_t Motor1Up2(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_PRESET_PULSE_06, MOTOR_PRESET_PULSE_05}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

//13.电机2前进
//14.电机3夹住电池
//15.电机2后退

//16.电机1上升3
static uint8_t Motor1Up3(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_PRESET_PULSE_08, MOTOR_PRESET_PULSE_07}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}
//17.飞机前进
//18.电机2前进
//19.电机3松开电池
//20.飞机后退
//21.电机2后退
//22.电机1下降


//居中1
static uint8_t CloseCenter1(void)
{
    MotorControlParams motors[4] = {
        {MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_TRAVEL_LOW_WORD, MOTOR10_TRAVEL_HIGH_WORD},
        {MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_TRAVEL_LOW_WORD, MOTOR10_TRAVEL_HIGH_WORD},
        {MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_TRAVEL_LOW_WORD, MOTOR9_TRAVEL_HIGH_WORD},
        {MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_TRAVEL_LOW_WORD, MOTOR9_TRAVEL_HIGH_WORD}
    };
    return MotorService_BatchMove(motors, 4U, NULL);
}


//居中2
static uint8_t CloseCenter2(void)
{
    MotorControlParams motors[2] = {
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR8_TRAVEL_LOW_WORD, MOTOR8_TRAVEL_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR8_TRAVEL_LOW_WORD, MOTOR8_TRAVEL_HIGH_WORD}
    };
    return MotorService_BatchMove(motors, 2U, NULL);
}


//飞机前进
static uint8_t FlyForward(void)
{
    MotorControlParams motors[4] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_TRAVEL_LOW_WORD, MOTOR6_TRAVEL_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_TRAVEL_LOW_WORD, MOTOR6_TRAVEL_HIGH_WORD},
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_TRAVEL_LOW_WORD, MOTOR5_TRAVEL_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_TRAVEL_LOW_WORD, MOTOR5_TRAVEL_HIGH_WORD}
    };
    return MotorService_BatchMove(motors, 4U, NULL);
}

//飞机后退
uint8_t FlyBack(void)
{
    MotorControlParams motors[4] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_TRAVEL_LOW_WORD, MOTOR5_TRAVEL_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_TRAVEL_LOW_WORD, MOTOR5_TRAVEL_HIGH_WORD},
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_TRAVEL_LOW_WORD, MOTOR6_TRAVEL_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_TRAVEL_LOW_WORD, MOTOR6_TRAVEL_HIGH_WORD}
    };
    return MotorService_BatchMove(motors, 4U, NULL);
}


/**
 * @brief 继电器正向吸合 3 秒后停止的非阻塞步骤。
 * @return PENDING：3 秒尚未到；OK：动作完成。
 */
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


static const StepDef opendr1_steps[] =
{
//      {OpenDr, 15250, STATUS_FIELD_NONE, 0},      //1.打开舱门
			{StopDr, 1000, STATUS_FIELD_DOOR, 2},       //2.停止
			{CloseAC, 0, STATUS_FIELD_NONE, 0},   // 关闭空调

};
#define OPENDR1_STEP_COUNT (sizeof(opendr1_steps) / sizeof(opendr1_steps[0]))
// 打开舱门步骤表
static const StepDef opendr_steps[] =
{
      {OpenDr, 15250, STATUS_FIELD_DOOR, 2},      //1.打开舱门
//			{StopDr, 1000, STATUS_FIELD_DOOR, 2},       //2.停止
			{CloseAC, 0, STATUS_FIELD_NONE, 0},   // 关闭空调

};
#define OPENDR_STEP_COUNT (sizeof(opendr_steps) / sizeof(opendr_steps[0]))


// 关闭舱门步骤表
static const StepDef closedr_steps[] =
{
    {CloseDr, 16250, STATUS_FIELD_DOOR, 2},      //1.关闭舱门
//			{StopDr, 1000, STATUS_FIELD_DOOR, 4},       //2.停止
	{OpenAC, 0, STATUS_FIELD_NONE, 0},   // 打开空调

};
#define CLOSEDR_STEP_COUNT (sizeof(closedr_steps) / sizeof(closedr_steps[0]))

// 飞机开机步骤表
static const StepDef openfly_steps[] =
{
      {Center_1, 5000, STATUS_FIELD_NONE, 0},        //1.左右居中
			{Center_2, 13000, STATUS_FIELD_CENTER_ROD, 4},       //2.前后居中
			{Battery_27, 16000, STATUS_FIELD_NONE, 0},      //3.电机1上升
			{Battery_2, 5000, STATUS_FIELD_NONE, 0},       //4.飞机前进
			{Battery_28, 16000, STATUS_FIELD_NONE, 0},      //5.电机2前进
			{Battery_18, 800, STATUS_FIELD_NONE, 0},      //27.电机2前进
			{Battery_19, 1000, STATUS_FIELD_NONE, 0},      //28.电机2后退
			{Battery_18, 2000, STATUS_FIELD_NONE, 0},      //29.电机2前进
			{Battery_25, 16000, STATUS_FIELD_NONE, 0},     //30.电机2后退
			{Battery_6, 5000, STATUS_FIELD_NONE, 0},       //31.飞机后退
			{Battery_7, 16000, STATUS_FIELD_NONE, 0},      //32.电机1下降
			{Battery_21, 10000, STATUS_FIELD_NONE, 0},     //33.飞机前进
			{LeaveCenter, 10000, STATUS_FIELD_CENTER_ROD, 2},    //34.居中杆释放

};
#define OPENFLY_STEP_COUNT (sizeof(openfly_steps) / sizeof(openfly_steps[0]))

// 飞机关机步骤表
static const StepDef closefly_steps[] =
{

      {Center_1, 5000, STATUS_FIELD_NONE, 0},        //1.左右居中
			{Center_2, 13000, STATUS_FIELD_CENTER_ROD, 4},       //2.前后居中
			{Battery_27, 16000, STATUS_FIELD_NONE, 0},      //3.电机1上升
			{Battery_2, 5000, STATUS_FIELD_NONE, 0},       //4.飞机前进
			{Battery_28, 16000, STATUS_FIELD_NONE, 0},      //5.电机2前进
			{Battery_18, 1000, STATUS_FIELD_NONE, 0},      //27.电机2前进
			{Battery_19, 1000, STATUS_FIELD_NONE, 0},      //28.电机2后退
			{Battery_18, 2000, STATUS_FIELD_NONE, 0},      //29.电机2前进
			{Battery_25, 16000, STATUS_FIELD_NONE, 0},     //30.电机2后退
			{Battery_6, 5000, STATUS_FIELD_NONE, 0},       //31.飞机后退
			{Battery_7, 16000, STATUS_FIELD_NONE, 0},      //32.电机1下降
			{Battery_21, 10000, STATUS_FIELD_NONE, 0},     //33.飞机前进
			{LeaveCenter, 10000, STATUS_FIELD_CENTER_ROD, 2},    //34.居中杆释放

};
#define CLOSEFLY_STEP_COUNT (sizeof(closefly_steps) / sizeof(closefly_steps[0]))

// 一键起飞步骤表(只将飞机开机)
static const StepDef takeoff_steps_1[] =
{
	    {OpenDr, 15250, STATUS_FIELD_DOOR, 2},     	  //1.打开舱门
//			{StopDr, 1000, STATUS_FIELD_DOOR, 2},      		  //2.停止
			{CloseAC, 500, STATUS_FIELD_NONE, 0},            // 关闭空调
      {Center_1, 5000, STATUS_FIELD_NONE, 0},        //1.左右居中
			{Center_2, 13000, STATUS_FIELD_CENTER_ROD, 4},     	  //2.前后居中
			{Battery_27, 16000, STATUS_FIELD_NONE, 0},      //3.电机1上升
			{Battery_2, 5000, STATUS_FIELD_NONE, 0},       //4.飞机前进
			{Battery_28, 16000, STATUS_FIELD_NONE, 0},      //5.电机2前进
			{Battery_18, 1000, STATUS_FIELD_NONE, 0},      //27.电机2前进
			{Battery_19, 1000, STATUS_FIELD_NONE, 0},      //28.电机2后退
			{Battery_18, 2000, STATUS_FIELD_NONE, 0},      //29.电机2前进
			{Battery_25, 16000, STATUS_FIELD_NONE, 0},     //30.电机2后退
			{Battery_6, 5000, STATUS_FIELD_NONE, 0},       //31.飞机后退
			{Battery_7, 16000, STATUS_FIELD_NONE, 0},      //32.电机1下降
			{Battery_21, 10000, STATUS_FIELD_NONE, 0},     //33.飞机前进
			{LeaveCenter, 10000, STATUS_FIELD_CENTER_ROD, 2},   	  //34.居中杆释放
			{CloseAC, 16000, STATUS_FIELD_NONE, 0},            // 关闭空调
//      {CheckAndCloseDoor, 16250, STATUS_FIELD_NONE, 0},     	  //1.关闭舱门
			{StopDr, 1000, STATUS_FIELD_DOOR, 4},      		  //2.停止

};
#define TAKEOFF_STEPS_1_COUNT  (sizeof(takeoff_steps_1) / sizeof(takeoff_steps_1[0]))


// 一键起飞步骤表(只将飞机开机)
static const StepDef takeoff_steps_2[] =
{
	    {OpenDr, 15250, STATUS_FIELD_DOOR, 2},     	  //1.打开舱门
//			{StopDr, 1000, STATUS_FIELD_DOOR, 2},      		  //2.停止
			{CloseAC, 500, STATUS_FIELD_NONE, 0},            // 关闭空调
      {Center_1, 5000, STATUS_FIELD_NONE, 0},        //1.左右居中
			{Center_2, 13000, STATUS_FIELD_CENTER_ROD, 4},     	  //2.前后居中
			{Battery_27, 16000, STATUS_FIELD_NONE, 0},      //3.电机1上升
			{Battery_2, 5000, STATUS_FIELD_NONE, 0},       //4.飞机前进
			{Battery_28, 16000, STATUS_FIELD_NONE, 0},      //5.电机2前进
			{Battery_18, 1000, STATUS_FIELD_NONE, 0},      //27.电机2前进
			{Battery_19, 1000, STATUS_FIELD_NONE, 0},      //28.电机2后退
			{Battery_18, 2000, STATUS_FIELD_NONE, 0},      //29.电机2前进
			{Battery_25, 16000, STATUS_FIELD_NONE, 0},     //30.电机2后退
			{Battery_6, 5000, STATUS_FIELD_NONE, 0},       //31.飞机后退
			{Battery_7, 16000, STATUS_FIELD_NONE, 0},      //32.电机1下降
			{Battery_21, 10000, STATUS_FIELD_NONE, 0},     //33.飞机前进
			{LeaveCenter, 10000, STATUS_FIELD_CENTER_ROD, 2},   	  //34.居中杆释放
			{CloseAC, 16000, STATUS_FIELD_NONE, 0},            // 关闭空调
//      {CheckAndCloseDoor, 16250, STATUS_FIELD_NONE, 0},     	  //1.关闭舱门
			{StopDr, 1000, STATUS_FIELD_DOOR, 4},      		  //2.停止

};
#define TAKEOFF_STEPS_2_COUNT (sizeof(takeoff_steps_2) / sizeof(takeoff_steps_2[0]))


// 一键起飞步骤表(只将飞机开机)
static const StepDef takeoff_steps_3[] =
{
	    {OpenDr, 15250, STATUS_FIELD_DOOR, 2},     	  //1.打开舱门
//			{StopDr, 1000, STATUS_FIELD_DOOR, 2},      		  //2.停止
			{CloseAC, 500, STATUS_FIELD_NONE, 0},            // 关闭空调
			{StopDr, 1000, STATUS_FIELD_DOOR, 2},      		  //2.停止
      {Center_1, 5000, STATUS_FIELD_NONE, 0},        //1.左右居中
			{Center_2, 13000, STATUS_FIELD_CENTER_ROD, 4},     	  //2.前后居中
			{Battery_27, 16000, STATUS_FIELD_NONE, 0},      //3.电机1上升
			{Battery_2, 5000, STATUS_FIELD_NONE, 0},       //4.飞机前进
			{Battery_28, 16000, STATUS_FIELD_NONE, 0},      //5.电机2前进
			{Battery_18, 1000, STATUS_FIELD_NONE, 0},      //27.电机2前进
			{Battery_19, 1000, STATUS_FIELD_NONE, 0},      //28.电机2后退
			{Battery_18, 2000, STATUS_FIELD_NONE, 0},      //29.电机2前进
			{Battery_25, 16000, STATUS_FIELD_NONE, 0},     //30.电机2后退
			{Battery_6, 5000, STATUS_FIELD_NONE, 0},       //31.飞机后退
			{Battery_7, 16000, STATUS_FIELD_NONE, 0},      //32.电机1下降
			{Battery_21, 10000, STATUS_FIELD_NONE, 0},     //33.飞机前进
			{LeaveCenter, 10000, STATUS_FIELD_CENTER_ROD, 2},   	  //34.居中杆释放
			{CloseAC, 18000, STATUS_FIELD_NONE, 0},            // 关闭空调
//      {CheckAndCloseDoor, 16250, STATUS_FIELD_NONE, 0},     	  //1.关闭舱门
			{StopDr, 1000, STATUS_FIELD_DOOR, 4},      		  //2.停止

};
#define TAKEOFF_STEPS_3_COUNT  (sizeof(takeoff_steps_3) / sizeof(takeoff_steps_3[0]))

// 降落完成步骤表（取电换电都完成）
static const StepDef landing_steps_1[] =
{
			//下电池步骤（1号空仓，将飞机电池放入1号仓）（取电装电都完成）
      {Center_1, 5000, STATUS_FIELD_NONE, 0},        //1.左右居中
			{Center_2, 13000, STATUS_FIELD_CENTER_ROD, 4},      		//2.前后居中
			{Battery_1, 16000, STATUS_FIELD_NONE, 0},      //3.电机1上升
			{Battery_2, 5000, STATUS_FIELD_NONE, 0},       //4.飞机前进
			{Battery_3, 18000, STATUS_FIELD_NONE, 0},      //5.电机2前进
			{Battery_4, 1000, STATUS_FIELD_NONE, 0},       //6.电机3夹紧
			{Battery_5, 15000, STATUS_FIELD_NONE, 0},      //7.电机2后退
			{Battery_6, 5000, STATUS_FIELD_NONE, 0},       //8.飞机后退
			{Battery_7, 18000, STATUS_FIELD_NONE, 0},      //9.电机1下降
			{Battery_8, 10000, STATUS_FIELD_NONE, 0},      //10.电机1上升
			{Battery_9, 19000, STATUS_FIELD_NONE, 0},      //11.电机2前进
			{Battery_10, 1000, STATUS_FIELD_NONE, 0},       //12.电机3松开
			{Battery_11, 2000, STATUS_FIELD_NONE, 0},      //13.电机2前进
			{Battery_12, 18000, STATUS_FIELD_NONE, 0},     //14.电机2后退
			{Battery_13, 10000, STATUS_FIELD_SWAP_MECHANISM, 2},     //15.电机1下降
			//装电池步骤(从2号仓取电池)
			{Battery_23, 8000, STATUS_FIELD_NONE, 0},      //16.电机1上升
			{Battery_9, 18000, STATUS_FIELD_NONE, 0},      //17.电机2前进
			{Battery_4, 1000, STATUS_FIELD_NONE, 0},       //18.电机3夹紧
			{Battery_22, 18000, STATUS_FIELD_NONE, 0},     //19.电机2后退
			{Battery_24, 8000, STATUS_FIELD_NONE, 0},      //20.电机1下降
			{Battery_1, 16000, STATUS_FIELD_NONE, 0},      //21.电机1上升
      {Battery_2, 8000, STATUS_FIELD_NONE, 0},       //22.飞机前进
			{Battery_26, 18000, STATUS_FIELD_NONE, 0},      //23.电机2前进
			{Battery_10, 1000, STATUS_FIELD_NONE, 0},      //24.电机3松开
			{Battery_16, 2000, STATUS_FIELD_NONE, 0},      //25.电机2前进
			{Battery_20, 15000, STATUS_FIELD_NONE, 0},     //7.电机2后退
			{Battery_6, 5000, STATUS_FIELD_NONE, 0},       //31.飞机后退
			{Battery_7, 16000, STATUS_FIELD_NONE, 0},      //32.电机1下降
			{Battery_21, 10000, STATUS_FIELD_SWAP_MECHANISM, 4},     	//33.飞机前进

			{LeaveCenter, 15000, STATUS_FIELD_CENTER_ROD, 2},   		//34.居中杆释放
      {CloseDr, 16250, STATUS_FIELD_DOOR, 4},     	  //1.关闭舱门
//			{StopDr, 1000, STATUS_FIELD_DOOR, 4},        		//2.停止
			{OpenAC, 500, STATUS_FIELD_NONE, 0},             // 打开空调
			{UpdateEmptyBay, 500, STATUS_FIELD_NONE, 0},     // 最后一步更新空仓号

};
#define LANDING_STEPS_1_COUNT (sizeof(landing_steps_1) / sizeof(landing_steps_1[0]))

// 降落完成步骤表（2号空仓，将飞机电池放入2号仓）（取电装电都完成）
static const StepDef landing_steps_2[] =
{
			//下电池步骤(放入2号仓)
      {Center_1, 5000, STATUS_FIELD_NONE, 0},        //1.左右居中
			{Center_2, 13000, STATUS_FIELD_CENTER_ROD, 4},        //2.前后居中
			{Battery_1, 16000, STATUS_FIELD_NONE, 0},      //3.电机1上升
			{Battery_2, 5000, STATUS_FIELD_NONE, 0},       //4.飞机前进
			{Battery_3, 18000, STATUS_FIELD_NONE, 0},      //5.电机2前进
			{Battery_4, 1000, STATUS_FIELD_NONE, 0},        //6.电机3夹紧
			{Battery_5, 15000, STATUS_FIELD_NONE, 0},      //7.电机2后退
			{Battery_6, 5000, STATUS_FIELD_NONE, 0},       //8.飞机后退
			{Battery_7, 18000, STATUS_FIELD_NONE, 0},      //9.电机1下降
			{Battery_23, 8000, STATUS_FIELD_NONE, 0},       //10.电机1上升
			{Battery_9, 19000, STATUS_FIELD_NONE, 0},      //11.电机2前进
			{Battery_10, 1000, STATUS_FIELD_NONE, 0},       //12.电机3松开
			{Battery_11, 2000, STATUS_FIELD_NONE, 0},      //13.电机2前进
			{Battery_12, 18000, STATUS_FIELD_NONE, 0},     //14.电机2后退
			{Battery_24, 8000, STATUS_FIELD_SWAP_MECHANISM, 2},      //15.电机1下降
			//装电池步骤(从3号仓取电池)
		  {Battery_14, 1000, STATUS_FIELD_NONE, 0},       //16.电机1上升
			{Battery_9, 18000, STATUS_FIELD_NONE, 0},       //17.电机2前进
			{Battery_4, 1000, STATUS_FIELD_NONE, 0},         //18.电机3夹紧
			{Battery_22, 18000, STATUS_FIELD_NONE, 0},       //19.电机2后退
			{Battery_15, 1000, STATUS_FIELD_NONE, 0},       //20.电机1下降
			{Battery_1, 16000, STATUS_FIELD_NONE, 0},       //21.电机1上升
      {Battery_2, 8000, STATUS_FIELD_NONE, 0},        //22.飞机前进
			{Battery_26, 18000, STATUS_FIELD_NONE, 0},       //23.电机2前进
			{Battery_10, 1000, STATUS_FIELD_NONE, 0},        //24.电机3松开
			{Battery_16, 2000, STATUS_FIELD_NONE, 0},       //25.电机2前进
			{Battery_20, 15000, STATUS_FIELD_NONE, 0},      //7.电机2后退
			{Battery_6, 5000, STATUS_FIELD_NONE, 0},        //31.飞机后退
			{Battery_7, 16000, STATUS_FIELD_NONE, 0},        //32.电机1下降
			{Battery_21, 10000, STATUS_FIELD_SWAP_MECHANISM, 4},      //33.飞机前进

			{LeaveCenter, 15000, STATUS_FIELD_CENTER_ROD, 2},     //34.居中杆释放
      {CloseDr, 16250, STATUS_FIELD_DOOR, 4},     	  //1.关闭舱门
//			{StopDr, 1000, STATUS_FIELD_DOOR, 4},        		//2.停止
			{OpenAC, 500, STATUS_FIELD_NONE, 0},           // 打开空调
			{UpdateEmptyBay, 500, STATUS_FIELD_NONE, 0},   // 最后一步更新空仓号

};
#define LANDING_STEPS_2_COUNT (sizeof(landing_steps_2) / sizeof(landing_steps_2[0]))

// 降落完成步骤表（3号空仓，将飞机电池放入3号仓）（取电装电都完成）
static const StepDef landing_steps_3[] =
{
			//下电池步骤(放入3号仓)
      {Center_1, 5000, STATUS_FIELD_NONE, 0},        //1.左右居中
			{Center_2, 13000, STATUS_FIELD_CENTER_ROD, 4},       //2.前后居中
			{Battery_1, 16000, STATUS_FIELD_NONE, 0},      //3.电机1上升
			{Battery_2, 5000, STATUS_FIELD_NONE, 0},       //4.飞机前进
			{Battery_3, 18000, STATUS_FIELD_NONE, 0},      //5.电机2前进
			{Battery_4, 1000, STATUS_FIELD_NONE, 0},        //6.电机3夹紧
			{Battery_5, 15000, STATUS_FIELD_NONE, 0},      //7.电机2后退
			{Battery_6, 5000, STATUS_FIELD_NONE, 0},       //8.飞机后退
			{Battery_7, 18000, STATUS_FIELD_NONE, 0},      //9.电机1下降
			{Battery_14, 1000, STATUS_FIELD_NONE, 0},       //10.电机1上升
			{Battery_9, 19000, STATUS_FIELD_NONE, 0},      //11.电机2前进
			{Battery_10, 1000, STATUS_FIELD_NONE, 0},       //12.电机3松开
			{Battery_11, 2000, STATUS_FIELD_NONE, 0},      //13.电机2前进
			{Battery_12, 18000, STATUS_FIELD_NONE, 0},     //14.电机2后退
			{Battery_15, 1000, STATUS_FIELD_SWAP_MECHANISM, 2},      //15.电机1下降
			//装电池步骤(从1号仓取电池)
			{Battery_8, 10000, STATUS_FIELD_NONE, 0},       //16.电机1上升
			{Battery_9, 18000, STATUS_FIELD_NONE, 0},       //17.电机2前进
			{Battery_4, 1000, STATUS_FIELD_NONE, 0},         //18.电机3夹紧
			{Battery_22, 18000, STATUS_FIELD_NONE, 0},       //19.电机2后退
			{Battery_13, 10000, STATUS_FIELD_NONE, 0},       //20.电机1下降
			{Battery_1, 16000, STATUS_FIELD_NONE, 0},       //21.电机1上升
      {Battery_2, 8000, STATUS_FIELD_NONE, 0},        //22.飞机前进
			{Battery_26, 18000, STATUS_FIELD_NONE, 0},       //23.电机2前进
			{Battery_10, 1000, STATUS_FIELD_NONE, 0},        //24.电机3松开
			{Battery_16, 2000, STATUS_FIELD_NONE, 0},       //25.电机2前进
			{Battery_20, 15000, STATUS_FIELD_NONE, 0},      //7.电机2后退
			{Battery_6, 5000, STATUS_FIELD_NONE, 0},        //31.飞机后退
			{Battery_7, 16000, STATUS_FIELD_NONE, 0},        //32.电机1下降

			{Battery_21, 10000, STATUS_FIELD_SWAP_MECHANISM, 4},      //33.飞机前进
			{LeaveCenter, 15000, STATUS_FIELD_CENTER_ROD, 2},     //34.居中杆释放
      {CloseDr, 16250, STATUS_FIELD_DOOR, 4},     	  //1.关闭舱门
//			{StopDr, 1000, STATUS_FIELD_DOOR, 4},        		//2.停止
			{OpenAC, 500, STATUS_FIELD_NONE, 0},             // 打开空调
			{UpdateEmptyBay, 500, STATUS_FIELD_NONE, 0},   // 最后一步更新空仓号

};
#define LANDING_STEPS_3_COUNT (sizeof(landing_steps_3) / sizeof(landing_steps_3[0]))

// 居中步骤
static const StepDef closecenter_steps[] =
{
		{Center_1, 5000, STATUS_FIELD_NONE, 0},
    {Center_2, 13000, STATUS_FIELD_CENTER_ROD, 4},
};
#define CLOSECENTER_STEP_COUNT (sizeof(closecenter_steps) / sizeof(closecenter_steps[0]))

// 释放步骤
static const StepDef leavecenter_steps[] =
{
		{LeaveCenter1, 15000, STATUS_FIELD_CENTER_ROD, 2},    // 等待15秒

};
#define LEAVECENTER_STEP_COUNT (sizeof(leavecenter_steps) / sizeof(leavecenter_steps[0]))

// 装电池步骤（取1号仓电池）
static const StepDef loadbattery_steps_1[] =
{
		{Center_1, 5000, STATUS_FIELD_NONE, 0},        //1.左右居中
		{Center_2, 13000, STATUS_FIELD_CENTER_ROD, 4},       //2.前后居中
		{Battery_8, 10000, STATUS_FIELD_NONE, 0},       //16.电机1上升
		{Battery_9, 18000, STATUS_FIELD_NONE, 0},       //17.电机2前进
		{Battery_4, 1000, STATUS_FIELD_NONE, 0},         //18.电机3夹紧
		{Battery_22, 18000, STATUS_FIELD_NONE, 0},       //19.电机2后退
		{Battery_13, 10000, STATUS_FIELD_NONE, 0},       //20.电机1下降
		{Battery_1, 16000, STATUS_FIELD_NONE, 0},       //21.电机1上升
		{Battery_2, 8000, STATUS_FIELD_NONE, 0},        //22.飞机前进
		{Battery_3, 18000, STATUS_FIELD_NONE, 0},       //23.电机2前进
		{Battery_10, 1000, STATUS_FIELD_NONE, 0},        //24.电机3松开
		{Battery_16, 2000, STATUS_FIELD_NONE, 0},       //25.电机2前进
		{Battery_17, 1000, STATUS_FIELD_NONE, 0},        //26.电机2后退
		{Battery_5, 15000, STATUS_FIELD_NONE, 0},      //7.电机2后退
		{Battery_6, 5000, STATUS_FIELD_NONE, 0},        //31.飞机后退
		{Battery_7, 16000, STATUS_FIELD_NONE, 0},        //32.电机1下降
		{Battery_21, 10000, STATUS_FIELD_SWAP_MECHANISM, 4},      //33.飞机前进
		{LeaveCenter, 10000, STATUS_FIELD_CENTER_ROD, 2},     //34.居中杆释放
};
#define LOADBATTERY_STEPS_1_COUNT (sizeof(loadbattery_steps_1) / sizeof(loadbattery_steps_1[0]))



// 装电池步骤（取2号仓电池）
static const StepDef loadbattery_steps_2[] =
{
		{Center_1, 5000, STATUS_FIELD_NONE, 0},        //1.左右居中
		{Center_2, 13000, STATUS_FIELD_CENTER_ROD, 4},       //2.前后居中
		{Battery_23, 8000, STATUS_FIELD_NONE, 0},       //16.电机1上升
		{Battery_9, 18000, STATUS_FIELD_NONE, 0},       //17.电机2前进
		{Battery_4, 1000, STATUS_FIELD_NONE, 0},         //18.电机3夹紧
		{Battery_22, 18000, STATUS_FIELD_NONE, 0},       //19.电机2后退
		{Battery_24, 8000, STATUS_FIELD_NONE, 0},       //20.电机1下降
		{Battery_1, 16000, STATUS_FIELD_NONE, 0},       //21.电机1上升
		{Battery_2, 8000, STATUS_FIELD_NONE, 0},        //22.飞机前进
		{Battery_3, 18000, STATUS_FIELD_NONE, 0},       //23.电机2前进
		{Battery_10, 1000, STATUS_FIELD_NONE, 0},        //24.电机3松开
		{Battery_16, 2000, STATUS_FIELD_NONE, 0},       //25.电机2前进
		{Battery_17, 1000, STATUS_FIELD_NONE, 0},        //26.电机2后退
		{Battery_5, 15000, STATUS_FIELD_NONE, 0},      //7.电机2后退
		{Battery_6, 5000, STATUS_FIELD_NONE, 0},        //31.飞机后退
		{Battery_7, 16000, STATUS_FIELD_NONE, 0},        //32.电机1下降
		{Battery_21, 10000, STATUS_FIELD_SWAP_MECHANISM, 4},      //33.飞机前进
		{LeaveCenter, 10000, STATUS_FIELD_CENTER_ROD, 2},     //34.居中杆释放
};
#define LOADBATTERY_STEPS_2_COUNT (sizeof(loadbattery_steps_2) / sizeof(loadbattery_steps_2[0]))


// 装电池步骤（取3号仓电池）
static const StepDef loadbattery_steps_3[] =
{
      {Center_1, 5000, STATUS_FIELD_NONE, 0},        //1.左右居中
			{Center_2, 13000, STATUS_FIELD_CENTER_ROD, 4},       //2.前后居中
      {Battery_14, 2000, STATUS_FIELD_NONE, 0},       //16.电机1上升
			{Battery_9, 18000, STATUS_FIELD_NONE, 0},       //17.电机2前进
			{Battery_4, 1000, STATUS_FIELD_NONE, 0},         //18.电机3夹紧
			{Battery_22, 18000, STATUS_FIELD_NONE, 0},       //19.电机2后退
			{Battery_15, 2000, STATUS_FIELD_NONE, 0},       //20.电机1下降
			{Battery_1, 16000, STATUS_FIELD_NONE, 0},       //21.电机1上升
      {Battery_2, 8000, STATUS_FIELD_NONE, 0},        //22.飞机前进
			{Battery_3, 18000, STATUS_FIELD_NONE, 0},       //23.电机2前进
			{Battery_10, 1000, STATUS_FIELD_NONE, 0},        //24.电机3松开
			{Battery_16, 2000, STATUS_FIELD_NONE, 0},       //25.电机2前进
			{Battery_17, 1000, STATUS_FIELD_NONE, 0},        //26.电机2后退
			{Battery_5, 15000, STATUS_FIELD_NONE, 0},      //7.电机2后退
			{Battery_6, 5000, STATUS_FIELD_NONE, 0},        //31.飞机后退
			{Battery_7, 16000, STATUS_FIELD_NONE, 0},        //32.电机1下降
			{Battery_21, 10000, STATUS_FIELD_SWAP_MECHANISM, 4},      //33.飞机前进
			{LeaveCenter, 10000, STATUS_FIELD_CENTER_ROD, 2},     //34.居中杆释放
};
#define LOADBATTERY_STEPS_3_COUNT (sizeof(loadbattery_steps_3) / sizeof(loadbattery_steps_3[0]))


// 下电池步骤(放入1号仓)
static const StepDef downbattery_steps_1[] =
{
      {Center_1, 5000, STATUS_FIELD_NONE, 0},        //1.左右居中
			{Center_2, 13000, STATUS_FIELD_CENTER_ROD, 4},       //2.前后居中
			{Battery_1, 16000, STATUS_FIELD_NONE, 0},      //3.电机1上升
			{Battery_2, 5000, STATUS_FIELD_NONE, 0},       //4.飞机前进
			{Battery_3, 18000, STATUS_FIELD_NONE, 0},      //5.电机2前进
			{Battery_4, 1000, STATUS_FIELD_NONE, 0},        //6.电机3夹紧
			{Battery_5, 15000, STATUS_FIELD_NONE, 0},      //7.电机2后退
			{Battery_6, 5000, STATUS_FIELD_NONE, 0},       //8.飞机后退
			{Battery_7, 18000, STATUS_FIELD_NONE, 0},      //9.电机1下降
			{Battery_8, 10000, STATUS_FIELD_NONE, 0},       //10.电机1上升
			{Battery_9, 19000, STATUS_FIELD_NONE, 0},      //11.电机2前进
			{Battery_10, 1000, STATUS_FIELD_NONE, 0},       //12.电机3松开
			{Battery_11, 2000, STATUS_FIELD_NONE, 0},      //13.电机2前进
			{Battery_12, 18000, STATUS_FIELD_NONE, 0},     //14.电机2后退
			{Battery_13, 10000, STATUS_FIELD_NONE, 0},      //15.电机1下降
			{Battery_21, 10000, STATUS_FIELD_SWAP_MECHANISM, 2},      //33.飞机前进
			{LeaveCenter, 10000, STATUS_FIELD_CENTER_ROD, 2},     //34.居中杆释放
};
#define DOWNBATTERY_STEPS_1_COUNT (sizeof(downbattery_steps_1) / sizeof(downbattery_steps_1[0]))

// 下电池步骤(放入2号仓)
static const StepDef downbattery_steps_2[] =
{
      {Center_1, 5000, STATUS_FIELD_NONE, 0},        //1.左右居中
			{Center_2, 13000, STATUS_FIELD_CENTER_ROD, 4},       //2.前后居中
			{Battery_1, 16000, STATUS_FIELD_NONE, 0},      //3.电机1上升
			{Battery_2, 5000, STATUS_FIELD_NONE, 0},       //4.飞机前进
			{Battery_3, 18000, STATUS_FIELD_NONE, 0},      //5.电机2前进
			{Battery_4, 1000, STATUS_FIELD_NONE, 0},        //6.电机3夹紧
			{Battery_5, 15000, STATUS_FIELD_NONE, 0},      //7.电机2后退
			{Battery_6, 5000, STATUS_FIELD_NONE, 0},       //8.飞机后退
			{Battery_7, 18000, STATUS_FIELD_NONE, 0},      //9.电机1下降
			{Battery_23, 8000, STATUS_FIELD_NONE, 0},       //10.电机1上升
			{Battery_9, 19000, STATUS_FIELD_NONE, 0},      //11.电机2前进
			{Battery_10, 1000, STATUS_FIELD_NONE, 0},       //12.电机3松开
			{Battery_11, 2000, STATUS_FIELD_NONE, 0},      //13.电机2前进
			{Battery_12, 18000, STATUS_FIELD_NONE, 0},     //14.电机2后退
			{Battery_24, 8000, STATUS_FIELD_NONE, 0},      //15.电机1下降
			{Battery_21, 10000, STATUS_FIELD_SWAP_MECHANISM, 2},      //33.飞机前进
			{LeaveCenter, 10000, STATUS_FIELD_CENTER_ROD, 2},     //34.居中杆释放
};
#define DOWNBATTERY_STEPS_2_COUNT (sizeof(downbattery_steps_2) / sizeof(downbattery_steps_2[0]))

// 下电池步骤(放入3号仓)
static const StepDef downbattery_steps_3[] =
{
      {Center_1, 5000, STATUS_FIELD_NONE, 0},        //1.左右居中
			{Center_2, 13000, STATUS_FIELD_CENTER_ROD, 4},       //2.前后居中
			{Battery_1, 16000, STATUS_FIELD_NONE, 0},      //3.电机1上升
			{Battery_2, 5000, STATUS_FIELD_NONE, 0},       //4.飞机前进
			{Battery_3, 18000, STATUS_FIELD_NONE, 0},      //5.电机2前进
			{Battery_4, 1000, STATUS_FIELD_NONE, 0},        //6.电机3夹紧
			{Battery_5, 15000, STATUS_FIELD_NONE, 0},      //7.电机2后退
			{Battery_6, 5000, STATUS_FIELD_NONE, 0},       //8.飞机后退
			{Battery_7, 18000, STATUS_FIELD_NONE, 0},      //9.电机1下降
			{Battery_14, 2000, STATUS_FIELD_NONE, 0},       //10.电机1上升
			{Battery_9, 19000, STATUS_FIELD_NONE, 0},      //11.电机2前进
			{Battery_10, 1000, STATUS_FIELD_NONE, 0},       //12.电机3松开
			{Battery_11, 2000, STATUS_FIELD_NONE, 0},      //13.电机2前进
			{Battery_12, 18000, STATUS_FIELD_NONE, 0},     //14.电机2后退
			{Battery_15, 2000, STATUS_FIELD_NONE, 0},      //15.电机1下降
			{Battery_21, 10000, STATUS_FIELD_SWAP_MECHANISM, 2},      //33.飞机前进
			{LeaveCenter, 10000, STATUS_FIELD_CENTER_ROD, 2},     //34.居中杆释放
};
#define DOWNBATTERY_STEPS_3_COUNT (sizeof(downbattery_steps_3) / sizeof(downbattery_steps_3[0]))

// 全局变量记录当前是否有旧电池（由主站轮询或状态寄存器更新）
extern uint8_t g_has_old_battery;   // 1:有电池, 0:无电池

// 步骤表1：有电池时，执行取下电池并放入空电池仓
static const StepDef cancel_with_battery_steps[] =
{
    {Motor1Up1, 5000, STATUS_FIELD_NONE, 0},             	//3.电机1上升1
		{FlyForward, 5000, STATUS_FIELD_NONE, 0},						  //4.飞机前进
    {Motor2Forward, 10000, STATUS_FIELD_NONE, 0},          //5.电机2前进
		{Motor3Clamp, 500, STATUS_FIELD_NONE, 0},					    //6.电机3夹紧
    {FlyBack, 12000, STATUS_FIELD_NONE, 0},					      //7.飞机后退
		{Motor2Back, 10000, STATUS_FIELD_NONE, 0}, 					  //8.电机2后退
    {Motor1Down1, 13000, STATUS_FIELD_NONE, 0},						//9.电机1下降1
		{Motor2Forward, 10000, STATUS_FIELD_NONE, 0},					//10.电机2前进
		{Motor3Lossen, 500, STATUS_FIELD_NONE, 0},							//11.电机3松开
		{Motor2Back, 10000, STATUS_FIELD_NONE, 0},							//10.电机2后退
};
#define CANCEL_WITH_BATTERY_COUNT (sizeof(cancel_with_battery_steps)/sizeof(cancel_with_battery_steps[0]))

// 步骤表2：无电池时，只需复位电机到初始状态（无需取电池）
static const StepDef cancel_without_battery_steps[] =
{
    {Motor2Back, 5000, STATUS_FIELD_NONE, 0},              // 电机2后退（确保在初始位）
    {Motor1Down1, 1000, STATUS_FIELD_NONE, 0},             // 电机1下降
		{Motor3Lossen, 500, STATUS_FIELD_NONE, 0},							// 电机3松开
    {LeaveCenter, 15000, STATUS_FIELD_CENTER_ROD, 2},            // 居中杆释放
};
#define CANCEL_WITHOUT_BATTERY_COUNT (sizeof(cancel_without_battery_steps)/sizeof(cancel_without_battery_steps[0]))



void Sequence_Init(void)
{
    memset(&seq_runner, 0, sizeof(seq_runner));
}

uint8_t Sequence_IsBusy(void)
{
		return seq_runner.busy;
}

/**
 * @brief 校验运行条件并启动指定序列。
 * @return 明确区分启动成功、序列忙、ID 无效和主站事务忙。
 */
SequenceStartResult Sequence_Start(SeqId id)
{
    if (seq_runner.busy) {
        printf("序列已在执行中，忽略新请求\r\n");
        return SEQ_START_BUSY;
    }

    if (MotorService_IsBusy()) {
        return SEQ_START_MASTER_BUSY;
    }

		// 获取当前空仓号（1~3）
    uint8_t empty_bay = SwapState_GetEmptyBay();
    if (empty_bay < 1 || empty_bay > 3) {
        printf("无效的空仓号: %d，使用默认1号仓\n", empty_bay);
        empty_bay = 1;
    }
    uint8_t index = empty_bay - 1; // 数组索引

    switch (id) {
				case SEQ_ID_OPENDR1:
						seq_runner.steps = opendr1_steps;
						seq_runner.step_count = OPENDR1_STEP_COUNT;
						printf("打开舱门序列\r\n");
				break;
				case SEQ_ID_OPENDR:
						seq_runner.steps = opendr_steps;
						seq_runner.step_count = OPENDR_STEP_COUNT;
						printf("打开舱门序列\r\n");
				break;
				case SEQ_ID_CLOSEDR:
						seq_runner.steps = closedr_steps;
						seq_runner.step_count = CLOSEDR_STEP_COUNT;
						printf("关闭舱门序列\r\n");
				break;
			  case SEQ_ID_CLOSECENTER:
						seq_runner.steps = closecenter_steps;
						seq_runner.step_count = CLOSECENTER_STEP_COUNT;
						printf("居中杆居中序列\r\n");
				break;
				case SEQ_ID_LEAVECENTER:
						seq_runner.steps = leavecenter_steps;
						seq_runner.step_count = LEAVECENTER_STEP_COUNT;
						printf("居中杆释放序列\r\n");
				break;
				case SEQ_ID_LOADBATTERY:
				{
						const StepDef *array[] = {loadbattery_steps_1, loadbattery_steps_2, loadbattery_steps_3};
            const uint8_t counts[] = {LOADBATTERY_STEPS_1_COUNT, LOADBATTERY_STEPS_2_COUNT, LOADBATTERY_STEPS_3_COUNT};
            seq_runner.steps = array[index];
            seq_runner.step_count = counts[index];
            printf("启动装电池序列，空仓=%d\n", empty_bay);
            break;
				}
				case SEQ_ID_DOWNBATTERY:
				{
					  const StepDef *array[] = {downbattery_steps_1, downbattery_steps_2, downbattery_steps_3};
            const uint8_t counts[] = {DOWNBATTERY_STEPS_1_COUNT, DOWNBATTERY_STEPS_2_COUNT, DOWNBATTERY_STEPS_3_COUNT};
            seq_runner.steps = array[index];
            seq_runner.step_count = counts[index];
            printf("启动下电池序列，空仓=%d\n", empty_bay);
            break;
				}
        case SEQ_ID_TAKEOFF:
				{
						const StepDef *array[] = {takeoff_steps_1, takeoff_steps_2, takeoff_steps_3};
            const uint8_t counts[] = {TAKEOFF_STEPS_1_COUNT, TAKEOFF_STEPS_2_COUNT, TAKEOFF_STEPS_3_COUNT};
            seq_runner.steps = array[index];
            seq_runner.step_count = counts[index];
            printf("启动起飞序列，空仓=%d\n", empty_bay);
            break;
				}
        case SEQ_ID_LANDING:
				{
            const StepDef *array[] = {landing_steps_1, landing_steps_2, landing_steps_3};
            const uint8_t counts[] = {LANDING_STEPS_1_COUNT, LANDING_STEPS_2_COUNT, LANDING_STEPS_3_COUNT};
            seq_runner.steps = array[index];
            seq_runner.step_count = counts[index];
            printf("启动降落序列，空仓=%d\n", empty_bay);
            break;
				}
				case SEQ_ID_OPENFLY:
            seq_runner.steps = openfly_steps;
            seq_runner.step_count = OPENFLY_STEP_COUNT;
            printf("飞机开机完成序列\r\n");
            break;
				case SEQ_ID_CLOSEFLY:
            seq_runner.steps = closefly_steps;
            seq_runner.step_count = CLOSEFLY_STEP_COUNT;
            printf("飞机关机完成序列\r\n");
            break;

//        case SEQ_ID_CANCEL:
//            seq_runner.steps = landing_steps;
//            seq_runner.step_count = LANDING_STEP_COUNT;
//            printf("取消序列\r\n");
//            break;
//				case SEQ_ID_OPENUP:
//						seq_runner.steps = OpenUp_steps;
//						seq_runner.step_count = OPENUP_STEP_COUNT;
//						printf("开门及/平台上升序列\r\n");
//				break;
//				case SEQ_ID_CLOSEDOWN:
//						seq_runner.steps = downclose_steps;
//						seq_runner.step_count = CLOSEDOWN_STEP_COUNT;
//						printf("关门及/平台下降序列\r\n");
//				break;
        default:
            return SEQ_START_INVALID_ID;
    }
		// ========== 拍摄状态快照 ==========
    bsp_StopHardTimer(1U);
    g_step_timer_expired = 0U;
    StatusService_TakeSnapshot();

    seq_runner.id = id;
    seq_runner.current_index = 0;
    seq_runner.wait_until = 0;
    seq_runner.busy = 1;
    seq_runner.error = 0;
		seq_runner.paused = 0;
		seq_runner.pending_update_field = STATUS_FIELD_NONE;
    seq_runner.retry_count = 0;   // 重置重试计数
    return SEQ_START_OK;
}


// 执行当前步骤（发送指令）

/**
 * @brief 推进当前序列步骤。
 * @note  步骤或底层 Modbus 批处理未完成时直接返回，下一轮主循环继续推进。
 */
static void ExecuteCurrentStep(void)
{
    const StepDef *step = &seq_runner.steps[seq_runner.current_index];

    uint8_t ret = step->func();

    if (ret == MODBUS_RESULT_PENDING || MotorService_IsBusy()) {
        return;
    }

    if (ret == MODBUS_RESULT_OK) {
        // 正常成功
        if (step->wait_ms > 0) {
            bsp_StartHardTimer(1, step->wait_ms * 1000, StepTimerCallback);
            seq_runner.wait_until = 1;
            if (step->update_field != STATUS_FIELD_NONE) {
				seq_runner.pending_update_field = step->update_field;
                seq_runner.pending_update_value = step->update_value;
            } else {
				seq_runner.pending_update_field = STATUS_FIELD_NONE;
            }
            printf("步骤 %d 成功，启动硬件定时器 %d ms\r\n", seq_runner.current_index, step->wait_ms);
        } else {
            if (step->update_field != STATUS_FIELD_NONE) {
                StatusService_Update(step->update_field, step->update_value);
            }
            seq_runner.current_index++;
            if (seq_runner.current_index >= seq_runner.step_count) {
                printf("序列执行完成\r\n");
                Sequence_Finish(0U);
            }
        }
    } else if (ret == STEP_RESULT_RETRY) {
				// 堵转触发重试
        seq_runner.retry_count++;
        printf("步骤 %d 触发重试，当前重试次数 %d/3\r\n", seq_runner.current_index, seq_runner.retry_count);
        if (seq_runner.retry_count < 3) {
            // 重启整个序列
            uint8_t seq_id = seq_runner.id;
            uint8_t retry_count = seq_runner.retry_count;

            Sequence_Finish(0U);
            if (Sequence_Start((SeqId)seq_id) != SEQ_START_OK) {
                Sequence_Finish(1U);
            } else {
                seq_runner.retry_count = retry_count;
            }
        } else {
            // 超过重试次数，更新故障码为0xFF，终止序列
            StatusService_Update(STATUS_FIELD_FAULT, 0x00FFU);
            printf("重试次数超过3次，更新故障码0xFF，序列终止\r\n");
            Sequence_Finish(1U);
        }
    } else {
        printf("步骤 %d 执行失败，错误码 %d\r\n", seq_runner.current_index, ret);
        Sequence_Finish(1U);
    }
}

void Sequence_Process(void)
{
	  if (!seq_runner.busy || seq_runner.error) {
//        printf("Sequence_Process: busy=%d, error=%d\n", seq_runner.busy, seq_runner.error);
        return;
    }

    if (seq_runner.wait_until > 0) {
//			printf("wait_until=%d, g_step_timer_expired=%d\n", seq_runner.wait_until, g_step_timer_expired);
        // 暂停时，不处理任何超时，保留状态
        if (seq_runner.paused) {
            return;
        }

        // 正常等待超时处理
        if (g_step_timer_expired) {
            g_step_timer_expired = 0;
            seq_runner.wait_until = 0;

            // 执行挂起的寄存器更新
            if (seq_runner.pending_update_field != STATUS_FIELD_NONE) {
                StatusService_Update(seq_runner.pending_update_field,
                                     seq_runner.pending_update_value);
                seq_runner.pending_update_field = STATUS_FIELD_NONE;
            }

            // 进入下一步
            seq_runner.current_index++;
            if (seq_runner.current_index >= seq_runner.step_count) {
                printf("序列执行完毕\r\n");
                Sequence_Finish(0U);
            } else {
                printf("等待结束，进入步骤 %d\r\n", seq_runner.current_index);
            }
        }
        return;
    }

    // 若暂停，则不允许执行新步骤
    if (seq_runner.paused) return;


    if (seq_runner.current_index < seq_runner.step_count) {
        ExecuteCurrentStep();
    }
}

// 判断无人机内是否有旧电池（根据电池充电状态）
static uint8_t HasOldBatteryInUAV(void)
{
    // 读取三块电池的充电状态（0=充电中，1=未充电）
    uint16_t bat1_charge = StatusService_GetBatteryChargeState(1U);
    uint16_t bat2_charge = StatusService_GetBatteryChargeState(2U);
    uint16_t bat3_charge = StatusService_GetBatteryChargeState(3U);

    uint8_t not_charging_count = 0;
    if (bat1_charge == 1) not_charging_count++;
    if (bat2_charge == 1) not_charging_count++;
    if (bat3_charge == 1) not_charging_count++;

    // 只有当恰好一块电池不在充电时，才认为无人机内有旧电池
    return (not_charging_count == 1) ? 1 : 0;
}

// 取消序列启动函数
void Sequence_Cancel(void)
{
    if (seq_runner.busy) {
        /* 仅终止软件序列和未完成的通讯；物理急停需由专用安全接口完成。 */
        Sequence_ForceStop();
        return;
    }

    // 根据电池充电状态判断是否有旧电池
    if (HasOldBatteryInUAV()) {
        // 有电池：执行换电序列（取下旧电池，放入空电池仓）
        seq_runner.steps = cancel_with_battery_steps;
        seq_runner.step_count = CANCEL_WITH_BATTERY_COUNT;
        printf("启动取消序列（有电池）\r\n");
    } else {
        // 无电池：直接复位到换电完成状态
        seq_runner.steps = cancel_without_battery_steps;
        seq_runner.step_count = CANCEL_WITHOUT_BATTERY_COUNT;
        printf("启动取消序列（无电池）\r\n");
    }

    seq_runner.id = SEQ_ID_CANCEL;
    seq_runner.current_index = 0;
    seq_runner.wait_until = 0;
    seq_runner.busy = 1;
    seq_runner.error = 0;
	seq_runner.paused = 0;
	seq_runner.pending_update_field = STATUS_FIELD_NONE;
	seq_runner.retry_count = 0U;

    // 拍摄状态快照（可选）
    bsp_StopHardTimer(1U);
    g_step_timer_expired = 0U;
    StatusService_TakeSnapshot();
}



//////////////////////////////////////居中升降主体函数//////////////////////////////////////////////////
uint8_t Center_1(void)
{
    MotorControlParams motors[4] = {
        {MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_TRAVEL_LOW_WORD, MOTOR10_TRAVEL_HIGH_WORD},
        {MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_TRAVEL_LOW_WORD, MOTOR12_TRAVEL_HIGH_WORD},
        {MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_TRAVEL_LOW_WORD, MOTOR9_TRAVEL_HIGH_WORD},
        {MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR11_TRAVEL_LOW_WORD, MOTOR11_TRAVEL_HIGH_WORD}
    };
    return MotorService_BatchMove(motors, 4U, NULL);
}

//uint8_t Center_1(void)
//{
//    MotorControlParams motors[4] = {
//        {MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_TRAVEL_LOW_WORD, MOTOR10_TRAVEL_HIGH_WORD},
//        {MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_TRAVEL_LOW_WORD, MOTOR12_TRAVEL_HIGH_WORD},
//        {MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_TRAVEL_LOW_WORD, MOTOR9_TRAVEL_HIGH_WORD},
//        {MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR11_TRAVEL_LOW_WORD, MOTOR11_TRAVEL_HIGH_WORD}
//    };

//    uint8_t addrs[] = {MOTOR10_SLAVE_ADDR, MOTOR12_SLAVE_ADDR, MOTOR9_SLAVE_ADDR, MOTOR11_SLAVE_ADDR};
//		int32_t target_pos[] = {MOTOR9_CENTER_POS, MOTOR10_CENTER_POS, MOTOR11_CENTER_POS, MOTOR12_CENTER_POS};
//    uint8_t ret = WaitWithPositionCheck(addrs, 4, 5000, 1000, 3, target_pos);
//    if (ret == 1) {
//        // 堵转
//        StatusRegs_Update(REG_FAULT_CODE, 0x00E1);
//        printf("Center_1 检测到电机堵转，执行释放并准备重试\n");
//        delay_ms(2000);
//        LeaveCenter1();
//        delay_ms(15000);
//        return 3;   // 重试
//    } else if (ret == 2) {
//        // 到位失败
//        StatusRegs_Update(REG_FAULT_CODE, 0x00E2);
//        printf("Center_1 电机未到位，上报故障\n");
//        master_state = MASTER_IDLE;
//        timeout_cnt = 0;
//        return 1;   // 终止序列
//    }

//    master_state = MASTER_IDLE;
//    timeout_cnt = 0;
//    return 0;
//}

uint8_t Center_2(void)
{
    MotorControlParams motors[4] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR14_TRAVEL_LOW_WORD, MOTOR14_TRAVEL_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR14_TRAVEL_LOW_WORD, MOTOR14_TRAVEL_HIGH_WORD},
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR8_TRAVEL_LOW_WORD, MOTOR8_TRAVEL_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR8_TRAVEL_LOW_WORD, MOTOR8_TRAVEL_HIGH_WORD}
    };
    return MotorService_BatchMove(motors, 4U, NULL);
}
//uint8_t Center_2(void)
//{
//    MotorControlParams motors[4] = {
//        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR14_TRAVEL_LOW_WORD, MOTOR14_TRAVEL_HIGH_WORD},
//        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR14_TRAVEL_LOW_WORD, MOTOR14_TRAVEL_HIGH_WORD},
//        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR8_TRAVEL_LOW_WORD, MOTOR8_TRAVEL_HIGH_WORD},
//        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR8_TRAVEL_LOW_WORD, MOTOR8_TRAVEL_HIGH_WORD}
//    };

//    uint8_t addrs[] = {MOTOR5_SLAVE_ADDR, MOTOR6_SLAVE_ADDR, MOTOR7_SLAVE_ADDR, MOTOR8_SLAVE_ADDR};
//    int32_t target_pos[] = {MOTOR5_CENTER_POS, MOTOR6_CENTER_POS, MOTOR7_CENTER_POS, MOTOR8_CENTER_POS};
//    uint8_t ret = WaitWithPositionCheck(addrs, 4, 13000, 1000, 3, target_pos);
//    if (ret == 1) {
//        // 堵转
//        StatusRegs_Update(REG_FAULT_CODE, 0x00E1);
//        printf("Center_2 检测到电机堵转，执行释放并准备重试\n");
//        delay_ms(2000);
//        LeaveCenter1();
//        delay_ms(15000);
//        return 3;   // 重试
//    } else if (ret == 2) {
//        // 到位失败
//        StatusRegs_Update(REG_FAULT_CODE, 0x00E2);
//        printf("Center_2 电机未到位，上报故障\n");
//				delay_ms(2000);
//        LeaveCenter1();
//        delay_ms(15000);
//        return 3;   // 终止序列
//    }

//    master_state = MASTER_IDLE;
//    timeout_cnt = 0;
//    return 0;
//}

/**
 * @brief 快速释放居中杆（不进行位置监测，仅发送指令）
 * @return 0 成功
 */
uint8_t LeaveCenter(void)
{
    MotorControlParams motors[8] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_TRAVEL_LOW_WORD, MOTOR13_TRAVEL_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_TRAVEL_LOW_WORD, MOTOR13_TRAVEL_HIGH_WORD},
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR7_TRAVEL_LOW_WORD, MOTOR7_TRAVEL_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR7_TRAVEL_LOW_WORD, MOTOR7_TRAVEL_HIGH_WORD},
        {MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_TRAVEL_LOW_WORD, MOTOR9_TRAVEL_HIGH_WORD},
        {MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR11_TRAVEL_LOW_WORD, MOTOR11_TRAVEL_HIGH_WORD},
        {MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_TRAVEL_LOW_WORD, MOTOR10_TRAVEL_HIGH_WORD},
        {MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_TRAVEL_LOW_WORD, MOTOR12_TRAVEL_HIGH_WORD}
    };
    return MotorService_BatchMove(motors, 8U, NULL);
}

//uint8_t LeaveCenter(void)
//{
//    MotorControlParams motors[8] = {
//        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_TRAVEL_LOW_WORD, MOTOR13_TRAVEL_HIGH_WORD},
//        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_TRAVEL_LOW_WORD, MOTOR13_TRAVEL_HIGH_WORD},
//        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR17_TRAVEL_LOW_WORD, MOTOR17_TRAVEL_HIGH_WORD},
//        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR17_TRAVEL_LOW_WORD, MOTOR17_TRAVEL_HIGH_WORD},
//        {MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_TRAVEL_LOW_WORD, MOTOR9_TRAVEL_HIGH_WORD},
//        {MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR11_TRAVEL_LOW_WORD, MOTOR11_TRAVEL_HIGH_WORD},
//        {MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_TRAVEL_LOW_WORD, MOTOR10_TRAVEL_HIGH_WORD},
//        {MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_TRAVEL_LOW_WORD, MOTOR12_TRAVEL_HIGH_WORD}
//    };

//    uint8_t addrs[] = {MOTOR5_SLAVE_ADDR, MOTOR6_SLAVE_ADDR, MOTOR7_SLAVE_ADDR, MOTOR8_SLAVE_ADDR,
//                       MOTOR10_SLAVE_ADDR, MOTOR12_SLAVE_ADDR, MOTOR9_SLAVE_ADDR, MOTOR11_SLAVE_ADDR};
//    int32_t target_pos[] = {MOTOR5_RELEASE_POS, MOTOR6_RELEASE_POS, MOTOR7_RELEASE_POS, MOTOR8_RELEASE_POS,
//                            MOTOR10_RELEASE_POS, MOTOR12_RELEASE_POS, MOTOR9_RELEASE_POS, MOTOR11_RELEASE_POS};
//    uint8_t ret = WaitWithPositionCheck(addrs, 8, 10000, 1000, 3, target_pos);
//    if (ret == 2) {
//        StatusRegs_Update(REG_FAULT_CODE, 0x00E3); // 释放不到位故障码
//        printf("释放未到位，上报故障\n");
//        delay_ms(2000);
//        LeaveCenter1();
//        delay_ms(15000);
//        master_state = MASTER_IDLE;
//        timeout_cnt = 0;
//        return 1;
//    }
//    master_state = MASTER_IDLE;
//    timeout_cnt = 0;
//    return 0;
//}

//释放1
uint8_t LeaveCenter1(void)
{
    MotorControlParams motors[8] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_TRAVEL_LOW_WORD, MOTOR13_TRAVEL_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_TRAVEL_LOW_WORD, MOTOR13_TRAVEL_HIGH_WORD},
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR7_TRAVEL_LOW_WORD, MOTOR17_TRAVEL_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR7_TRAVEL_LOW_WORD, MOTOR17_TRAVEL_HIGH_WORD},
        {MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_TRAVEL_LOW_WORD, MOTOR9_TRAVEL_HIGH_WORD},
        {MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR11_TRAVEL_LOW_WORD, MOTOR11_TRAVEL_HIGH_WORD},
        {MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_TRAVEL_LOW_WORD, MOTOR10_TRAVEL_HIGH_WORD},
        {MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_TRAVEL_LOW_WORD, MOTOR12_TRAVEL_HIGH_WORD}
    };
    return MotorService_BatchMove(motors, 8U, NULL);
}

uint8_t LeaveCenter2(void)
{
    MotorControlParams motors[8] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_TRAVEL_LOW_WORD, MOTOR13_TRAVEL_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_TRAVEL_LOW_WORD, MOTOR13_TRAVEL_HIGH_WORD},
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR17_TRAVEL_LOW_WORD, MOTOR17_TRAVEL_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR17_TRAVEL_LOW_WORD, MOTOR17_TRAVEL_HIGH_WORD},
        {MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_TRAVEL_LOW_WORD, MOTOR9_TRAVEL_HIGH_WORD},
        {MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR11_TRAVEL_LOW_WORD, MOTOR11_TRAVEL_HIGH_WORD},
        {MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_TRAVEL_LOW_WORD, MOTOR10_TRAVEL_HIGH_WORD},
        {MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_TRAVEL_LOW_WORD, MOTOR12_TRAVEL_HIGH_WORD}
    };
    return MotorService_BatchMove(motors, 8U, NULL);
}

//uint8_t LeaveCenter1(void)
//{
//    MotorControlParams motors[8] = {
//        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_TRAVEL_LOW_WORD, MOTOR13_TRAVEL_HIGH_WORD},
//        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_TRAVEL_LOW_WORD, MOTOR13_TRAVEL_HIGH_WORD},
//        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR17_TRAVEL_LOW_WORD, MOTOR17_TRAVEL_HIGH_WORD},
//        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR17_TRAVEL_LOW_WORD, MOTOR17_TRAVEL_HIGH_WORD},
//        {MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_TRAVEL_LOW_WORD, MOTOR9_TRAVEL_HIGH_WORD},
//        {MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR11_TRAVEL_LOW_WORD, MOTOR11_TRAVEL_HIGH_WORD},
//        {MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_TRAVEL_LOW_WORD, MOTOR10_TRAVEL_HIGH_WORD},
//        {MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_TRAVEL_LOW_WORD, MOTOR12_TRAVEL_HIGH_WORD}
//    };

//    uint8_t addrs[] = {MOTOR5_SLAVE_ADDR, MOTOR6_SLAVE_ADDR, MOTOR7_SLAVE_ADDR, MOTOR8_SLAVE_ADDR,
//                       MOTOR10_SLAVE_ADDR, MOTOR12_SLAVE_ADDR, MOTOR9_SLAVE_ADDR, MOTOR11_SLAVE_ADDR};
//    uint8_t ret = WaitWithPositionCheck(addrs, 8, 15000, 1000, 3);
//    if (ret != 0) {
//        StatusRegs_Update(REG_FAULT_CODE, 0x00E1);
//        printf("LeaveCenter1 检测到电机堵转，执行释放并准备重试\n");
//        delay_ms(2000);
//        LeaveCenter();
//        delay_ms(15000);
//        return 3;
//    }

//    master_state = MASTER_IDLE;
//    timeout_cnt = 0;
//    return 0;
//}

uint8_t Battery_1(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_PRESET_PULSE_07, MOTOR_PRESET_PULSE_08}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_2(void)
{
    MotorControlParams motors[4] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_TRAVEL_LOW_WORD, MOTOR6_TRAVEL_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_TRAVEL_LOW_WORD, MOTOR6_TRAVEL_HIGH_WORD},
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_TRAVEL_LOW_WORD, MOTOR5_TRAVEL_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_TRAVEL_LOW_WORD, MOTOR5_TRAVEL_HIGH_WORD}
    };
    return MotorService_BatchMove(motors, 4U, NULL);
}

uint8_t Battery_3(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_PRESET_PULSE_01, MOTOR_PRESET_PULSE_02}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

//uint8_t Battery_4(void)
//{
//  	//电机3夹紧电池
//		uint8_t motor_num3 = 2;   // 寄存器地址（夹紧）
//		uint16_t motor_cmd3 = MOTOR3_CMD_CLAMP; // 电机指令：
//		uint8_t ctrl_ret3 = Motor_Control(MOTOR3_SLAVE_ADDR, motor_num3, motor_cmd3);
//		if(ctrl_ret3 == 0)
//    {
//        printf("从机3夹紧指令发送成功！\r\n");
//    }
//    else
//    {
//        printf("从机3夹紧指令发送失败，错误码：%d\r\n", ctrl_ret3);
//    }
//		master_state = MASTER_IDLE;
//		timeout_cnt = 0; // 同时重置超时计数器
//    __disable_irq();
//    Master_RX_CNT = 0;
//    __enable_irq();
//    // 返回0表示成功，非0表示失败
//    return (ctrl_ret3 == 0) ? 0 : 1;
//}

//uint8_t Battery_4(void)
//{
//    // 直接发送06指令，不等待响应
//printf("Battery_4 前：master_state=%d, RX_CNT=%d\n", master_state, Master_RX_CNT);
//	delay_ms(5);
//    uint8_t ret = ModbusMaster_06_WriteSingleReg(MOTOR3_SLAVE_ADDR, MOTOR3_CTRL_REG2, MOTOR3_CMD_CLAMP);
//	delay_ms(10);
//    // 由于电机已动作，强制认为成功
//    master_state = MASTER_IDLE;
//    timeout_cnt = 0;
//    __disable_irq();
//    Master_RX_CNT = 0;
//    __enable_irq();
//    printf("电机3夹紧指令已发送（忽略响应）\r\n");
//    return 0;
//}


/** 电机 3 夹紧步骤；直接返回统一非阻塞 0x06 事务结果。 */
uint8_t Battery_4(void)
{
    return Motor_Control(MOTOR_ID_3, 2U, MOTOR3_CMD_CLAMP);
}

uint8_t Battery_5(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_PRESET_PULSE_19, MOTOR_PRESET_PULSE_20}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}


uint8_t Battery_6(void)
{
    MotorControlParams motors[4] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_TRAVEL_LOW_WORD, MOTOR5_TRAVEL_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_TRAVEL_LOW_WORD, MOTOR5_TRAVEL_HIGH_WORD},
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_TRAVEL_LOW_WORD, MOTOR6_TRAVEL_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_TRAVEL_LOW_WORD, MOTOR6_TRAVEL_HIGH_WORD}
    };
    return MotorService_BatchMove(motors, 4U, NULL);
}

uint8_t Battery_7(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_PRESET_PULSE_09, MOTOR_PRESET_PULSE_10}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_8(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_PRESET_PULSE_11, MOTOR_PRESET_PULSE_12}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_9(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_PRESET_PULSE_27, MOTOR_PRESET_PULSE_28}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

//uint8_t Battery_10(void)
//{
//  	//电机3松开电池
//		uint8_t motor_num3 = 1;   // 寄存器地址（松开）
//		uint16_t motor_cmd3 = MOTOR3_CMD_RELEASE; // 电机指令：
//		uint8_t ctrl_ret3 = Motor_Control(MOTOR3_SLAVE_ADDR, motor_num3, motor_cmd3);
//		if(ctrl_ret3 == 0)
//    {
//        printf("从机3松开指令发送成功！\r\n");
//    }
//    else
//    {
//        printf("从机3松开指令发送失败，错误码：%d\r\n", ctrl_ret3);
//    }
//		master_state = MASTER_IDLE;
//		timeout_cnt = 0; // 同时重置超时计数器
//		return 0;
//}

//uint8_t Battery_10(void)
//{
//    // 直接发送06指令，不等待响应
//	printf("Battery_10 开始执行\n");
//    uint8_t ret = ModbusMaster_06_WriteSingleReg(MOTOR3_SLAVE_ADDR, MOTOR3_CTRL_REG1, MOTOR3_CMD_RELEASE);
//    // 由于电机已动作，强制认为成功
//    master_state = MASTER_IDLE;
//    timeout_cnt = 0;
//    __disable_irq();
//    Master_RX_CNT = 0;
//    __enable_irq();
//    printf("电机3松开指令已发送（忽略响应）\r\n");
//    return 0;
//}


/** 电机 3 控制步骤；向 CTRL_REG1 写入 MOTOR3_CMD_CLAMP，并返回统一非阻塞事务结果。 */
uint8_t Battery_10(void)
{
    return Motor_Control(MOTOR_ID_3, 1U, MOTOR3_CMD_RELEASE);
}

uint8_t Battery_11(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_PRESET_PULSE_05, MOTOR_PRESET_PULSE_06}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}


uint8_t Battery_12(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_PRESET_PULSE_03, MOTOR_PRESET_PULSE_04}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_13(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_PRESET_PULSE_13, MOTOR_PRESET_PULSE_14}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_14(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_PRESET_PULSE_15, MOTOR_PRESET_PULSE_16}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_15(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_PRESET_PULSE_17, MOTOR_PRESET_PULSE_18}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_16(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_PRESET_PULSE_21, MOTOR_PRESET_PULSE_22}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_17(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_PRESET_PULSE_23, MOTOR_PRESET_PULSE_24}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_18(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_PRESET_PULSE_31, MOTOR_PRESET_PULSE_32}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_19(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_PRESET_PULSE_33, MOTOR_PRESET_PULSE_34}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}


uint8_t Battery_20(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_PRESET_PULSE_25, MOTOR_PRESET_PULSE_26}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_21(void)
{
    MotorControlParams motors[4] = {
        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR16_TRAVEL_LOW_WORD, MOTOR16_TRAVEL_HIGH_WORD},
        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR16_TRAVEL_LOW_WORD, MOTOR16_TRAVEL_HIGH_WORD},
        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR15_TRAVEL_LOW_WORD, MOTOR15_TRAVEL_HIGH_WORD},
        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR15_TRAVEL_LOW_WORD, MOTOR15_TRAVEL_HIGH_WORD}
    };
    return MotorService_BatchMove(motors, 4U, NULL);
}

uint8_t Battery_22(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_PRESET_PULSE_29, MOTOR_PRESET_PULSE_30}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_23(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_PRESET_PULSE_35, MOTOR_PRESET_PULSE_36}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_24(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_PRESET_PULSE_37, MOTOR_PRESET_PULSE_38}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_25(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_PRESET_PULSE_39, MOTOR_PRESET_PULSE_40}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_26(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_PRESET_PULSE_41, MOTOR_PRESET_PULSE_42}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}


uint8_t Battery_27(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_PRESET_PULSE_43, MOTOR_PRESET_PULSE_44}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_28(void)
{
    MotorControlParams motors[1] = {
        {MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, MOTOR_PRESET_PULSE_45, MOTOR_PRESET_PULSE_46}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t Battery_29(void)
{
    MotorControlParams motors[1] = {
        {MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, MOTOR_PRESET_PULSE_47, MOTOR_PRESET_PULSE_48}
    };
    return MotorService_BatchMove(motors, 1U, NULL);
}

uint8_t CloseDr(void)
{
    return Motor_Control(MOTOR_ID_4, 1U, MOTOR4_CMD_CLOSE);
}
//uint8_t CloseDr(void)
//{
//    // 读取舱门当前状态（0x11寄存器）
//    uint16_t door_state = StatusRegs_Get(REG_DOOR_STATE);
//
//    // 如果舱门已经打开到位（值为4），则不再执行打开指令
////    if (door_state == 4) {
////        printf("舱门已关闭到位，无需重复执行打开指令\n");
////        master_state = MASTER_IDLE;
////        timeout_cnt = 0;
////        return 0;
////    }

//    // 执行舱门关闭指令
//    uint8_t motor_num4 = 1;
//    uint16_t motor_cmd4 = MOTOR4_CMD_CLOSE;
//    uint8_t ctrl_ret3 = Motor_Control(MOTOR4_SLAVE_ADDR, motor_num4, motor_cmd4);
//    if (ctrl_ret3 == 0) {
//        printf("舱门关闭指令发送成功！\r\n");
//    } else {
//        printf("舱门关闭指令发送失败，错误码：%d\r\n", ctrl_ret3);
//    }
//
//    master_state = MASTER_IDLE;
//    timeout_cnt = 0;
//    return 0;
//}

uint8_t CheckAndCloseDoor(void)
{
    uint16_t uav_status = StatusService_GetUavStatus();
		printf("CheckAndCloseDoor: uav_status=%d\n", uav_status);
    if (uav_status == 3) {
        printf("检测到无人机不在机巢（0x60=3），执行关闭舱门\n");
        uint8_t ret = CloseDr();
        if (ret != 0) {
            printf("关闭舱门执行失败，错误码：%d\n", ret);
        }
        return ret;
    } else {
        printf("0x60=%d，无需关闭舱门\n", uav_status);
    }
    return MODBUS_RESULT_OK;
}

uint8_t StopDr(void)
{
    return Motor_Control(MOTOR_ID_4, 1U, MOTOR4_CMD_STOP);
}


uint8_t OpenDr(void)
{
    return Motor_Control(MOTOR_ID_4, 1U, MOTOR4_CMD_OPEN);
}

//uint8_t OpenDr(void)
//{
//    // 读取舱门当前状态（0x11寄存器）
//    uint16_t door_state = StatusRegs_Get(REG_DOOR_STATE);
//
//    // 如果舱门已经打开到位（值为2），则不再执行打开指令
////    if (door_state == 2) {
////        printf("舱门已打开到位，无需重复执行打开指令\n");
////        master_state = MASTER_IDLE;
////        timeout_cnt = 0;
////        return 0;
////    }

//    // 执行舱门打开指令
//    uint8_t motor_num4 = 1;
//    uint16_t motor_cmd4 = MOTOR4_CMD_OPEN;
//    uint8_t ctrl_ret3 = Motor_Control(MOTOR4_SLAVE_ADDR, motor_num4, motor_cmd4);
//    if (ctrl_ret3 == 0) {
//        printf("舱门打开指令发送成功！\r\n");
//    } else {
//        printf("舱门打开指令发送失败，错误码：%d\r\n", ctrl_ret3);
//    }
//
//    master_state = MASTER_IDLE;
//    timeout_cnt = 0;
//    return 0;
//}


uint8_t OpenAC(void)
{
	//打开空调
        uint8_t ctrl_ret3 = ModbusMaster_06_WriteSingleReg(
            MODBUS_MASTER_CLIENT_GATEWAY, AIR_CONDITIONER_SLAVE,
            AIR_POWER_CTRL_REG, AIR_POWER_ON_VALUE);
		if(ctrl_ret3 == 0)
    {
        printf("空调打开指令发送成功！\r\n");
    }
    else
    {
        printf("空调打开指令发送失败，错误码：%d\r\n", ctrl_ret3);
    }
		return ctrl_ret3;
}

uint8_t CloseAC(void)
{
	//关闭空调
        uint8_t ctrl_ret3 = ModbusMaster_06_WriteSingleReg(
            MODBUS_MASTER_CLIENT_GATEWAY, AIR_CONDITIONER_SLAVE,
            AIR_POWER_CTRL_REG, AIR_POWER_OFF_VALUE);
		if(ctrl_ret3 == 0)
    {
        printf("空调关闭指令发送成功！\r\n");
    }
    else
    {
        printf("空调关闭指令发送失败，错误码：%d\r\n", ctrl_ret3);
    }
		return ctrl_ret3;
}

uint8_t Sequence_GetCurrentId(void)
{
    if (!seq_runner.busy) return 0;
    return seq_runner.id;
}

uint8_t Sequence_GetCurrentStep(void)
{
    if (!seq_runner.busy) return 0xFF;   // 序列未运行返回0xFF
    return seq_runner.current_index;
}

const uint8_t* GetMotorListForCurrentStep(void)
{
    uint8_t seq = Sequence_GetCurrentId();
    uint8_t step = Sequence_GetCurrentStep();
    for (uint8_t i = 0; i < MAP_SIZE; i++) {
        if (step_motor_map[i].seq_id == seq && step_motor_map[i].step_index == step) {
            return step_motor_map[i].motor_addrs;
        }
    }
    return NULL; // 返回空指针表示未找到
}

/** 强制停止序列，并同时取消其可能占用的批处理及主站事务。 */
void Sequence_ForceStop(void) {
        MotorService_Cancel();
        Sequence_Finish(1U);
}
