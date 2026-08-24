#include "project.h"
#include <stdio.h>
#include <string.h>


volatile uint8_t relay_stop = 0;
// 外部主站写函数
extern uint8_t Modbus_06_WriteSingleReg(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_data);

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
		uint16_t pending_update_addr;
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
//    ret = Motor_Control(MOTOR1_SLAVE_ADDR, 8, MOTOR4_speed);
//    if (ret != 0) return ret;
//    master_state = MASTER_IDLE;
//    timeout_cnt = 0;
	
}

//1.关闭舱门
static uint8_t CloseDoor(void)
{
//	  uint8_t ret;
//    ret = Motor_Control(MOTOR1_SLAVE_ADDR, 8, MOTOR4_speed);
//    if (ret != 0) return ret;
//    master_state = MASTER_IDLE;
//    timeout_cnt = 0;
	
}




//2.电机1上升1 
static uint8_t Motor1Up1(void)
{
		uint8_t Reg_num = 2;   // 寄存器数量
		uint16_t slave2_cmds[2] = {Pulse_num2, Pulse_num1};//寄存器指令
		if (master_state != MASTER_IDLE) 
		{
				master_state = MASTER_IDLE;
				timeout_cnt = 0;
    }
		uint8_t ret = Motor_Batch_Control(MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, Reg_num, slave2_cmds);// 指令发送返回值 
		printf("ret= %d\r\n", ret);
		if (ret != 0) return ret;
    master_state = MASTER_IDLE;
    timeout_cnt = 0;
		
		return 0;
}

//3.飞机前进

//4.电机2前进
static uint8_t Motor2Forward(void)
{
		uint8_t Reg_num = 5;   // 寄存器数量
		uint16_t slave2_cmds[5] = {Direction,Speed,Pulse_num11, Pulse_num12,Position1};//寄存器指令
		if (master_state != MASTER_IDLE) 
		{
				master_state = MASTER_IDLE;
				timeout_cnt = 0;
    }
		uint8_t ret = Motor_Batch_Control(MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG5, Reg_num, slave2_cmds);// 指令发送返回值 
		if (ret != 0) return ret;
    master_state = MASTER_IDLE;
    timeout_cnt = 0;
		
		return 0;
}

//5.电机3夹住电池
static uint8_t Motor3Clamp(void)
{
    uint8_t ret;
    // 设置初始速度 
    ret = Motor_Control(MOTOR3_SLAVE_ADDR, 2, Clamp);
    if (ret != 0) return ret;
    master_state = MASTER_IDLE;
    timeout_cnt = 0;

    return 0;
}

//6.飞机后退

//7.电机2后退
static uint8_t Motor2Back(void)
{
		uint8_t Reg_num = 5;   // 寄存器数量
		uint16_t slave2_cmds[5] = {Direction_back,Speed,Pulse_num11, Pulse_num12,Position1};//寄存器指令
		if (master_state != MASTER_IDLE) 
		{
				master_state = MASTER_IDLE;
				timeout_cnt = 0;
    }
		uint8_t ret = Motor_Batch_Control(MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG5, Reg_num, slave2_cmds);// 指令发送返回值 
		if (ret != 0) return ret;
    master_state = MASTER_IDLE;
    timeout_cnt = 0;
		
		return 0;
}

//8.电机1下降 
static uint8_t Motor1Down1(void)
{
		uint8_t Reg_num = 2;   // 寄存器数量
		uint16_t slave2_cmds[2] = {Pulse_num4, Pulse_num3};//寄存器指令
		if (master_state != MASTER_IDLE) 
		{
				master_state = MASTER_IDLE;
				timeout_cnt = 0;
    }
		uint8_t ret = Motor_Batch_Control(MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, Reg_num, slave2_cmds);// 指令发送返回值 
		printf("ret= %d\r\n", ret);
		if (ret != 0) return ret;
    master_state = MASTER_IDLE;
    timeout_cnt = 0;
		
		return 0;
}

//9.电机2前进

//10.电机3松开电池
static uint8_t Motor3Lossen(void)
{
    uint8_t ret;
    ret = Motor_Control(MOTOR3_SLAVE_ADDR, 1, Lossen);
    if (ret != 0) return ret;
    master_state = MASTER_IDLE;
    timeout_cnt = 0;

    return 0;
}

//11.电机2后退

//12.电机1上升2 
static uint8_t Motor1Up2(void)
{
		uint8_t Reg_num = 2;   // 寄存器数量
		uint16_t slave2_cmds[2] = {Pulse_num6, Pulse_num5};//寄存器指令
		if (master_state != MASTER_IDLE) 
		{
				master_state = MASTER_IDLE;
				timeout_cnt = 0;
    }
		uint8_t ret = Motor_Batch_Control(MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, Reg_num, slave2_cmds);// 指令发送返回值 
		printf("ret= %d\r\n", ret);
		if (ret != 0) return ret;
    master_state = MASTER_IDLE;
    timeout_cnt = 0;
		
		return 0;
}

//13.电机2前进
//14.电机3夹住电池
//15.电机2后退

//16.电机1上升3 
static uint8_t Motor1Up3(void)
{
		uint8_t Reg_num = 2;   // 寄存器数量
		uint16_t slave2_cmds[2] = {Pulse_num8, Pulse_num7};//寄存器指令
		if (master_state != MASTER_IDLE) 
		{
				master_state = MASTER_IDLE;
				timeout_cnt = 0;
    }
		uint8_t ret = Motor_Batch_Control(MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, Reg_num, slave2_cmds);// 指令发送返回值 
		printf("ret= %d\r\n", ret);
		if (ret != 0) return ret;
    master_state = MASTER_IDLE;
    timeout_cnt = 0;
		
		return 0;
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
		//需要修改具体参数  
		MotorControlParams motors[4] = 
		{
			{MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_length_l, MOTOR10_length_h},
			{MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_length_l, MOTOR10_length_h},
			{MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_length_l, MOTOR9_length_h},
			{MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_length_l, MOTOR9_length_h}
    };
    // 同步发送所有电机指令（不等待响应）
    Sync_Motors_Control(motors, 4);
		
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
    return 0; 
}


//居中2
static uint8_t CloseCenter2(void)
{
		//需要修改具体参数  
		MotorControlParams motors[2] = 
		{
			{MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR8_length_l, MOTOR8_length_h},
			{MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR8_length_l, MOTOR8_length_h}
    };
    uint8_t results[2];
    // 同步发送所有电机指令（不等待响应）
    Sync_Motors_Control(motors, 2);
//		StatusRegs_Update(0x15, 4); //将居中状态写入寄存器
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
    return 0; 
}


//飞机前进
static uint8_t FlyForward(void)
{
		//需要修改具体参数  
		MotorControlParams motors[4] = 
		{
			{MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_length_l, MOTOR6_length_h},
			{MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_length_l, MOTOR6_length_h},
      {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_length_l, MOTOR5_length_h},
			{MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_length_l, MOTOR5_length_h}
    };
    uint8_t results[4];
    // 同步发送所有电机指令（不等待响应）
    Sync_Motors_Control(motors, 4);
//		StatusRegs_Update(0x15, 4); //将居中状态写入寄存器
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
    return 0; 
}

//飞机后退
uint8_t FlyBack(void)
{
		//需要修改具体参数  
		MotorControlParams motors[4] = 
		{
			{MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_length_l, MOTOR5_length_h},
			{MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_length_l, MOTOR5_length_h},
			{MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_length_l, MOTOR6_length_h},
			{MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_length_l, MOTOR6_length_h}
    };
    uint8_t results[4];
    uint8_t ret = Control_Motors_Complete(motors, 4, results);
    if (ret == 0) {
        printf("纵向归中控制成功\r\n");
    } else {
        printf("纵向归中控制有 %d 个失败\r\n", ret);
    }
//		StatusRegs_Update(0x15, 2);
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
    return 0; 
}


// 控制器继电器关启
uint8_t RelayCtrl(void)
{
		Relay_Forward();
		delay_ms(3000);   // 正转3秒
		Relay_Stop();
		return 0;
}


static const StepDef opendr1_steps[] = 
{
//      {OpenDr, 15250, 0xFFFF, 0},      //1.打开舱门      
			{StopDr, 1000, 0x11, 2},       //2.停止
			{CloseAC, 0, 0xFFFF, 0},   // 关闭空调

};
#define OPENDR1_STEP_COUNT (sizeof(opendr1_steps) / sizeof(opendr1_steps[0]))
// 打开舱门步骤表
static const StepDef opendr_steps[] = 
{
      {OpenDr, 15250, 0x11, 2},      //1.打开舱门      
//			{StopDr, 1000, 0x11, 2},       //2.停止
			{CloseAC, 0, 0xFFFF, 0},   // 关闭空调

};
#define OPENDR_STEP_COUNT (sizeof(opendr_steps) / sizeof(opendr_steps[0]))


// 关闭舱门步骤表
static const StepDef closedr_steps[] = 
{
      {CloseDr, 16250, 0x11, 2},      //1.关闭舱门      
//			{StopDr, 1000, 0x11, 4},       //2.停止
			{OpenAC, 0, 0xFFFF, 0},   // 打开空调

};
#define CLOSEDR_STEP_COUNT (sizeof(closedr_steps) / sizeof(closedr_steps[0]))

// 飞机开机步骤表
static const StepDef openfly_steps[] = 
{
      {Center_1, 5000, 0xFFFF, 0},        //1.左右居中      
			{Center_2, 13000, 0x15, 4},       //2.前后居中
			{Battery_27, 16000, 0xFFFF, 0},      //3.电机1上升
			{Battery_2, 5000, 0xFFFF, 0},       //4.飞机前进
			{Battery_28, 16000, 0xFFFF, 0},      //5.电机2前进
			{Battery_18, 800, 0xFFFF, 0},      //27.电机2前进
			{Battery_19, 1000, 0xFFFF, 0},      //28.电机2后退
			{Battery_18, 2000, 0xFFFF, 0},      //29.电机2前进
			{Battery_25, 16000, 0xFFFF, 0},     //30.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},       //31.飞机后退
			{Battery_7, 16000, 0xFFFF, 0},      //32.电机1下降
			{Battery_21, 10000, 0xFFFF, 0},     //33.飞机前进
			{LeaveCenter, 10000, 0x15, 2},    //34.居中杆释放

};
#define OPENFLY_STEP_COUNT (sizeof(openfly_steps) / sizeof(openfly_steps[0]))
	
// 飞机关机步骤表
static const StepDef closefly_steps[] = 
{

      {Center_1, 5000, 0xFFFF, 0},        //1.左右居中      
			{Center_2, 13000, 0x15, 4},       //2.前后居中
			{Battery_27, 16000, 0xFFFF, 0},      //3.电机1上升
			{Battery_2, 5000, 0xFFFF, 0},       //4.飞机前进
			{Battery_28, 16000, 0xFFFF, 0},      //5.电机2前进
			{Battery_18, 1000, 0xFFFF, 0},      //27.电机2前进
			{Battery_19, 1000, 0xFFFF, 0},      //28.电机2后退
			{Battery_18, 2000, 0xFFFF, 0},      //29.电机2前进
			{Battery_25, 16000, 0xFFFF, 0},     //30.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},       //31.飞机后退
			{Battery_7, 16000, 0xFFFF, 0},      //32.电机1下降
			{Battery_21, 10000, 0xFFFF, 0},     //33.飞机前进
			{LeaveCenter, 10000, 0x15, 2},    //34.居中杆释放
			
};
#define CLOSEFLY_STEP_COUNT (sizeof(closefly_steps) / sizeof(closefly_steps[0]))

// 一键起飞步骤表(只将飞机开机)
static const StepDef takeoff_steps_1[] = 
{	
	    {OpenDr, 15250, 0x11, 2},     	  //1.打开舱门      
//			{StopDr, 1000, 0x11, 2},      		  //2.停止
			{CloseAC, 500, 0xFFFF, 0},            // 关闭空调
      {Center_1, 5000, 0xFFFF, 0},        //1.左右居中      
			{Center_2, 13000, 0x15, 4},     	  //2.前后居中
			{Battery_27, 16000, 0xFFFF, 0},      //3.电机1上升
			{Battery_2, 5000, 0xFFFF, 0},       //4.飞机前进
			{Battery_28, 16000, 0xFFFF, 0},      //5.电机2前进
			{Battery_18, 1000, 0xFFFF, 0},      //27.电机2前进
			{Battery_19, 1000, 0xFFFF, 0},      //28.电机2后退
			{Battery_18, 2000, 0xFFFF, 0},      //29.电机2前进
			{Battery_25, 16000, 0xFFFF, 0},     //30.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},       //31.飞机后退
			{Battery_7, 16000, 0xFFFF, 0},      //32.电机1下降
			{Battery_21, 10000, 0xFFFF, 0},     //33.飞机前进
			{LeaveCenter, 10000, 0x15, 2},   	  //34.居中杆释放
			{CloseAC, 16000, 0xFFFF, 0},            // 关闭空调
//      {CheckAndCloseDoor, 16250, 0xFFFF, 0},     	  //1.关闭舱门      
			{StopDr, 1000, 0x11, 4},      		  //2.停止
      
};
#define TAKEOFF_STEPS_1_COUNT  (sizeof(takeoff_steps_1) / sizeof(takeoff_steps_1[0]))


// 一键起飞步骤表(只将飞机开机)
static const StepDef takeoff_steps_2[] = 
{
	    {OpenDr, 15250, 0x11, 2},     	  //1.打开舱门  
//			{StopDr, 1000, 0x11, 2},      		  //2.停止	
			{CloseAC, 500, 0xFFFF, 0},            // 关闭空调
      {Center_1, 5000, 0xFFFF, 0},        //1.左右居中      
			{Center_2, 13000, 0x15, 4},     	  //2.前后居中
			{Battery_27, 16000, 0xFFFF, 0},      //3.电机1上升
			{Battery_2, 5000, 0xFFFF, 0},       //4.飞机前进
			{Battery_28, 16000, 0xFFFF, 0},      //5.电机2前进
			{Battery_18, 1000, 0xFFFF, 0},      //27.电机2前进
			{Battery_19, 1000, 0xFFFF, 0},      //28.电机2后退
			{Battery_18, 2000, 0xFFFF, 0},      //29.电机2前进
			{Battery_25, 16000, 0xFFFF, 0},     //30.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},       //31.飞机后退
			{Battery_7, 16000, 0xFFFF, 0},      //32.电机1下降
			{Battery_21, 10000, 0xFFFF, 0},     //33.飞机前进
			{LeaveCenter, 10000, 0x15, 2},   	  //34.居中杆释放
			{CloseAC, 16000, 0xFFFF, 0},            // 关闭空调
//      {CheckAndCloseDoor, 16250, 0xFFFF, 0},     	  //1.关闭舱门      
			{StopDr, 1000, 0x11, 4},      		  //2.停止
      
};
#define TAKEOFF_STEPS_2_COUNT (sizeof(takeoff_steps_2) / sizeof(takeoff_steps_2[0]))


// 一键起飞步骤表(只将飞机开机)
static const StepDef takeoff_steps_3[] = 
{	
	    {OpenDr, 15250, 0x11, 2},     	  //1.打开舱门
//			{StopDr, 1000, 0x11, 2},      		  //2.停止			
			{CloseAC, 500, 0xFFFF, 0},            // 关闭空调			
			{StopDr, 1000, 0x11, 2},      		  //2.停止
      {Center_1, 5000, 0xFFFF, 0},        //1.左右居中      
			{Center_2, 13000, 0x15, 4},     	  //2.前后居中
			{Battery_27, 16000, 0xFFFF, 0},      //3.电机1上升
			{Battery_2, 5000, 0xFFFF, 0},       //4.飞机前进
			{Battery_28, 16000, 0xFFFF, 0},      //5.电机2前进
			{Battery_18, 1000, 0xFFFF, 0},      //27.电机2前进
			{Battery_19, 1000, 0xFFFF, 0},      //28.电机2后退
			{Battery_18, 2000, 0xFFFF, 0},      //29.电机2前进
			{Battery_25, 16000, 0xFFFF, 0},     //30.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},       //31.飞机后退
			{Battery_7, 16000, 0xFFFF, 0},      //32.电机1下降
			{Battery_21, 10000, 0xFFFF, 0},     //33.飞机前进
			{LeaveCenter, 10000, 0x15, 2},   	  //34.居中杆释放
			{CloseAC, 18000, 0xFFFF, 0},            // 关闭空调
//      {CheckAndCloseDoor, 16250, 0xFFFF, 0},     	  //1.关闭舱门      
			{StopDr, 1000, 0x11, 4},      		  //2.停止
      
};
#define TAKEOFF_STEPS_3_COUNT  (sizeof(takeoff_steps_3) / sizeof(takeoff_steps_3[0]))
	
// 降落完成步骤表（取电换电都完成）
static const StepDef landing_steps_1[] = 
{
			//下电池步骤（1号空仓，将飞机电池放入1号仓）（取电装电都完成）
      {Center_1, 5000, 0xFFFF, 0},        //1.左右居中      
			{Center_2, 13000, 0x15, 4},      		//2.前后居中
			{Battery_1, 16000, 0xFFFF, 0},      //3.电机1上升
			{Battery_2, 5000, 0xFFFF, 0},       //4.飞机前进
			{Battery_3, 18000, 0xFFFF, 0},      //5.电机2前进
			{Battery_4, 1000, 0xFFFF, 0},       //6.电机3夹紧
			{Battery_5, 15000, 0xFFFF, 0},      //7.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},       //8.飞机后退
			{Battery_7, 18000, 0xFFFF, 0},      //9.电机1下降			
			{Battery_8, 10000, 0xFFFF, 0},      //10.电机1上升
			{Battery_9, 19000, 0xFFFF, 0},      //11.电机2前进
			{Battery_10, 1000, 0xFFFF, 0},       //12.电机3松开
			{Battery_11, 2000, 0xFFFF, 0},      //13.电机2前进
			{Battery_12, 18000, 0xFFFF, 0},     //14.电机2后退
			{Battery_13, 10000, 0x14, 2},     //15.电机1下降
			//装电池步骤(从2号仓取电池)
			{Battery_23, 8000, 0xFFFF, 0},      //16.电机1上升
			{Battery_9, 18000, 0xFFFF, 0},      //17.电机2前进
			{Battery_4, 1000, 0xFFFF, 0},       //18.电机3夹紧
			{Battery_22, 18000, 0xFFFF, 0},     //19.电机2后退
			{Battery_24, 8000, 0xFFFF, 0},      //20.电机1下降	
			{Battery_1, 16000, 0xFFFF, 0},      //21.电机1上升
      {Battery_2, 8000, 0xFFFF, 0},       //22.飞机前进      
			{Battery_26, 18000, 0xFFFF, 0},      //23.电机2前进
			{Battery_10, 1000, 0xFFFF, 0},      //24.电机3松开
			{Battery_16, 2000, 0xFFFF, 0},      //25.电机2前进
			{Battery_20, 15000, 0xFFFF, 0},     //7.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},       //31.飞机后退
			{Battery_7, 16000, 0xFFFF, 0},      //32.电机1下降
			{Battery_21, 10000, 0x14, 4},     	//33.飞机前进
			
			{LeaveCenter, 15000, 0x15, 2},   		//34.居中杆释放	
      {CloseDr, 16250, 0x11, 4},     	  //1.关闭舱门      
//			{StopDr, 1000, 0x11, 4},        		//2.停止
			{OpenAC, 500, 0xFFFF, 0},             // 打开空调
			{UpdateEmptyBay, 500, 0xFFFF, 0},     // 最后一步更新空仓号
		
};
#define LANDING_STEPS_1_COUNT (sizeof(landing_steps_1) / sizeof(landing_steps_1[0]))
	
// 降落完成步骤表（2号空仓，将飞机电池放入2号仓）（取电装电都完成）
static const StepDef landing_steps_2[] = 
{
			//下电池步骤(放入2号仓)
      {Center_1, 5000, 0xFFFF, 0},        //1.左右居中      
			{Center_2, 13000, 0x15, 4},        //2.前后居中
			{Battery_1, 16000, 0xFFFF, 0},      //3.电机1上升
			{Battery_2, 5000, 0xFFFF, 0},       //4.飞机前进
			{Battery_3, 18000, 0xFFFF, 0},      //5.电机2前进
			{Battery_4, 1000, 0xFFFF, 0},        //6.电机3夹紧
			{Battery_5, 15000, 0xFFFF, 0},      //7.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},       //8.飞机后退
			{Battery_7, 18000, 0xFFFF, 0},      //9.电机1下降			
			{Battery_23, 8000, 0xFFFF, 0},       //10.电机1上升
			{Battery_9, 19000, 0xFFFF, 0},      //11.电机2前进
			{Battery_10, 1000, 0xFFFF, 0},       //12.电机3松开
			{Battery_11, 2000, 0xFFFF, 0},      //13.电机2前进
			{Battery_12, 18000, 0xFFFF, 0},     //14.电机2后退
			{Battery_24, 8000, 0x14, 2},      //15.电机1下降
			//装电池步骤(从3号仓取电池)
		  {Battery_14, 1000, 0xFFFF, 0},       //16.电机1上升
			{Battery_9, 18000, 0xFFFF, 0},       //17.电机2前进
			{Battery_4, 1000, 0xFFFF, 0},         //18.电机3夹紧
			{Battery_22, 18000, 0xFFFF, 0},       //19.电机2后退
			{Battery_15, 1000, 0xFFFF, 0},       //20.电机1下降	
			{Battery_1, 16000, 0xFFFF, 0},       //21.电机1上升
      {Battery_2, 8000, 0xFFFF, 0},        //22.飞机前进      
			{Battery_26, 18000, 0xFFFF, 0},       //23.电机2前进
			{Battery_10, 1000, 0xFFFF, 0},        //24.电机3松开
			{Battery_16, 2000, 0xFFFF, 0},       //25.电机2前进
			{Battery_20, 15000, 0xFFFF, 0},      //7.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},        //31.飞机后退
			{Battery_7, 16000, 0xFFFF, 0},        //32.电机1下降
			{Battery_21, 10000, 0x14, 4},      //33.飞机前进
			
			{LeaveCenter, 15000, 0x15, 2},     //34.居中杆释放	
      {CloseDr, 16250, 0x11, 4},     	  //1.关闭舱门      
//			{StopDr, 1000, 0x11, 4},        		//2.停止
			{OpenAC, 500, 0xFFFF, 0},           // 打开空调
			{UpdateEmptyBay, 500, 0xFFFF, 0},   // 最后一步更新空仓号
		
};
#define LANDING_STEPS_2_COUNT (sizeof(landing_steps_2) / sizeof(landing_steps_2[0]))
	
// 降落完成步骤表（3号空仓，将飞机电池放入3号仓）（取电装电都完成）
static const StepDef landing_steps_3[] = 
{
			//下电池步骤(放入3号仓)
      {Center_1, 5000, 0xFFFF, 0},        //1.左右居中      
			{Center_2, 13000, 0x15, 4},       //2.前后居中
			{Battery_1, 16000, 0xFFFF, 0},      //3.电机1上升
			{Battery_2, 5000, 0xFFFF, 0},       //4.飞机前进
			{Battery_3, 18000, 0xFFFF, 0},      //5.电机2前进
			{Battery_4, 1000, 0xFFFF, 0},        //6.电机3夹紧
			{Battery_5, 15000, 0xFFFF, 0},      //7.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},       //8.飞机后退
			{Battery_7, 18000, 0xFFFF, 0},      //9.电机1下降			
			{Battery_14, 1000, 0xFFFF, 0},       //10.电机1上升
			{Battery_9, 19000, 0xFFFF, 0},      //11.电机2前进
			{Battery_10, 1000, 0xFFFF, 0},       //12.电机3松开
			{Battery_11, 2000, 0xFFFF, 0},      //13.电机2前进
			{Battery_12, 18000, 0xFFFF, 0},     //14.电机2后退
			{Battery_15, 1000, 0x14, 2},      //15.电机1下降
			//装电池步骤(从1号仓取电池)
			{Battery_8, 10000, 0xFFFF, 0},       //16.电机1上升
			{Battery_9, 18000, 0xFFFF, 0},       //17.电机2前进
			{Battery_4, 1000, 0xFFFF, 0},         //18.电机3夹紧
			{Battery_22, 18000, 0xFFFF, 0},       //19.电机2后退
			{Battery_13, 10000, 0xFFFF, 0},       //20.电机1下降	
			{Battery_1, 16000, 0xFFFF, 0},       //21.电机1上升
      {Battery_2, 8000, 0xFFFF, 0},        //22.飞机前进      
			{Battery_26, 18000, 0xFFFF, 0},       //23.电机2前进
			{Battery_10, 1000, 0xFFFF, 0},        //24.电机3松开
			{Battery_16, 2000, 0xFFFF, 0},       //25.电机2前进
			{Battery_20, 15000, 0xFFFF, 0},      //7.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},        //31.飞机后退
			{Battery_7, 16000, 0xFFFF, 0},        //32.电机1下降
			
			{Battery_21, 10000, 0x14, 4},      //33.飞机前进
			{LeaveCenter, 15000, 0x15, 2},     //34.居中杆释放	
      {CloseDr, 16250, 0x11, 4},     	  //1.关闭舱门      
//			{StopDr, 1000, 0x11, 4},        		//2.停止
			{OpenAC, 500, 0xFFFF, 0},             // 打开空调
			{UpdateEmptyBay, 500, 0xFFFF, 0},   // 最后一步更新空仓号
		
};
#define LANDING_STEPS_3_COUNT (sizeof(landing_steps_3) / sizeof(landing_steps_3[0]))

// 居中步骤
static const StepDef closecenter_steps[] = 
{
		{Center_1, 5000, 0xFFFF, 0},    
    {Center_2, 13000, 0x15, 4},  	
};
#define CLOSECENTER_STEP_COUNT (sizeof(closecenter_steps) / sizeof(closecenter_steps[0]))
                                              
// 释放步骤
static const StepDef leavecenter_steps[] = 
{
		{LeaveCenter1, 15000, 0x0015, 2},    // 等待15秒 
		
};
#define LEAVECENTER_STEP_COUNT (sizeof(leavecenter_steps) / sizeof(leavecenter_steps[0]))

// 装电池步骤（取1号仓电池）
static const StepDef loadbattery_steps_1[] = 
{ 
      {Center_1, 5000, 0xFFFF, 0},        //1.左右居中      
			{Center_2, 13000, 0x15, 4},       //2.前后居中
      {Battery_8, 10000, 0xFFFF, 0},       //16.电机1上升
			{Battery_9, 18000, 0xFFFF, 0},       //17.电机2前进
			{Battery_4, 1000, 0xFFFF, 0},         //18.电机3夹紧
			{Battery_22, 18000, 0xFFFF, 0},       //19.电机2后退
			{Battery_13, 10000, 0xFFFF, 0},       //20.电机1下降	
			{Battery_1, 16000, 0xFFFF, 0},       //21.电机1上升
      {Battery_2, 8000, 0xFFFF, 0},        //22.飞机前进      
			{Battery_3, 18000, 0xFFFF, 0},       //23.电机2前进
			{Battery_10, 1000, 0xFFFF, 0},        //24.电机3松开
			{Battery_16, 2000, 0xFFFF, 0},       //25.电机2前进
			{Battery_17, 1000, 0xFFFF, 0},        //26.电机2后退
			{Battery_5, 15000, 0xFFFF, 0},      //7.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},        //31.飞机后退
			{Battery_7, 16000, 0xFFFF, 0},        //32.电机1下降
			{Battery_21, 10000, 0x14, 4},      //33.飞机前进
			{LeaveCenter, 10000, 0x15, 2},     //34.居中杆释放
};
#define LOADBATTERY_STEPS_1_COUNT (sizeof(loadbattery_steps_1) / sizeof(loadbattery_steps_1[0]))



// 装电池步骤（取2号仓电池）
static const StepDef loadbattery_steps_2[] = 
{ 
      {Center_1, 5000, 0xFFFF, 0},        //1.左右居中      
			{Center_2, 13000, 0x15, 4},       //2.前后居中
      {Battery_23, 8000, 0xFFFF, 0},       //16.电机1上升
			{Battery_9, 18000, 0xFFFF, 0},       //17.电机2前进
			{Battery_4, 1000, 0xFFFF, 0},         //18.电机3夹紧
			{Battery_22, 18000, 0xFFFF, 0},       //19.电机2后退
			{Battery_24, 8000, 0xFFFF, 0},       //20.电机1下降	
			{Battery_1, 16000, 0xFFFF, 0},       //21.电机1上升
      {Battery_2, 8000, 0xFFFF, 0},        //22.飞机前进      
			{Battery_3, 18000, 0xFFFF, 0},       //23.电机2前进
			{Battery_10, 1000, 0xFFFF, 0},        //24.电机3松开
			{Battery_16, 2000, 0xFFFF, 0},       //25.电机2前进
			{Battery_17, 1000, 0xFFFF, 0},        //26.电机2后退
			{Battery_5, 15000, 0xFFFF, 0},      //7.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},        //31.飞机后退
			{Battery_7, 16000, 0xFFFF, 0},        //32.电机1下降
			{Battery_21, 10000, 0x14, 4},      //33.飞机前进
			{LeaveCenter, 10000, 0x15, 2},     //34.居中杆释放
};
#define LOADBATTERY_STEPS_2_COUNT (sizeof(loadbattery_steps_2) / sizeof(loadbattery_steps_2[0]))


// 装电池步骤（取3号仓电池）
static const StepDef loadbattery_steps_3[] = 
{ 
      {Center_1, 5000, 0xFFFF, 0},        //1.左右居中      
			{Center_2, 13000, 0x15, 4},       //2.前后居中
      {Battery_14, 2000, 0xFFFF, 0},       //16.电机1上升
			{Battery_9, 18000, 0xFFFF, 0},       //17.电机2前进
			{Battery_4, 1000, 0xFFFF, 0},         //18.电机3夹紧
			{Battery_22, 18000, 0xFFFF, 0},       //19.电机2后退
			{Battery_15, 2000, 0xFFFF, 0},       //20.电机1下降	
			{Battery_1, 16000, 0xFFFF, 0},       //21.电机1上升
      {Battery_2, 8000, 0xFFFF, 0},        //22.飞机前进      
			{Battery_3, 18000, 0xFFFF, 0},       //23.电机2前进
			{Battery_10, 1000, 0xFFFF, 0},        //24.电机3松开
			{Battery_16, 2000, 0xFFFF, 0},       //25.电机2前进
			{Battery_17, 1000, 0xFFFF, 0},        //26.电机2后退
			{Battery_5, 15000, 0xFFFF, 0},      //7.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},        //31.飞机后退
			{Battery_7, 16000, 0xFFFF, 0},        //32.电机1下降
			{Battery_21, 10000, 0x14, 4},      //33.飞机前进
			{LeaveCenter, 10000, 0x15, 2},     //34.居中杆释放
};
#define LOADBATTERY_STEPS_3_COUNT (sizeof(loadbattery_steps_3) / sizeof(loadbattery_steps_3[0]))


// 下电池步骤(放入1号仓)
static const StepDef downbattery_steps_1[] = 
{
      {Center_1, 5000, 0xFFFF, 0},        //1.左右居中      
			{Center_2, 13000, 0x15, 4},       //2.前后居中
			{Battery_1, 16000, 0xFFFF, 0},      //3.电机1上升
			{Battery_2, 5000, 0xFFFF, 0},       //4.飞机前进
			{Battery_3, 18000, 0xFFFF, 0},      //5.电机2前进
			{Battery_4, 1000, 0xFFFF, 0},        //6.电机3夹紧
			{Battery_5, 15000, 0xFFFF, 0},      //7.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},       //8.飞机后退
			{Battery_7, 18000, 0xFFFF, 0},      //9.电机1下降			
			{Battery_8, 10000, 0xFFFF, 0},       //10.电机1上升
			{Battery_9, 19000, 0xFFFF, 0},      //11.电机2前进
			{Battery_10, 1000, 0xFFFF, 0},       //12.电机3松开
			{Battery_11, 2000, 0xFFFF, 0},      //13.电机2前进
			{Battery_12, 18000, 0xFFFF, 0},     //14.电机2后退
			{Battery_13, 10000, 0xFFFF, 0},      //15.电机1下降
			{Battery_21, 10000, 0x14, 2},      //33.飞机前进
			{LeaveCenter, 10000, 0x15, 2},     //34.居中杆释放			
};
#define DOWNBATTERY_STEPS_1_COUNT (sizeof(downbattery_steps_1) / sizeof(downbattery_steps_1[0]))
	
// 下电池步骤(放入2号仓)
static const StepDef downbattery_steps_2[] = 
{
      {Center_1, 5000, 0xFFFF, 0},        //1.左右居中      
			{Center_2, 13000, 0x15, 4},       //2.前后居中
			{Battery_1, 16000, 0xFFFF, 0},      //3.电机1上升
			{Battery_2, 5000, 0xFFFF, 0},       //4.飞机前进
			{Battery_3, 18000, 0xFFFF, 0},      //5.电机2前进
			{Battery_4, 1000, 0xFFFF, 0},        //6.电机3夹紧
			{Battery_5, 15000, 0xFFFF, 0},      //7.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},       //8.飞机后退
			{Battery_7, 18000, 0xFFFF, 0},      //9.电机1下降			
			{Battery_23, 8000, 0xFFFF, 0},       //10.电机1上升
			{Battery_9, 19000, 0xFFFF, 0},      //11.电机2前进
			{Battery_10, 1000, 0xFFFF, 0},       //12.电机3松开
			{Battery_11, 2000, 0xFFFF, 0},      //13.电机2前进
			{Battery_12, 18000, 0xFFFF, 0},     //14.电机2后退
			{Battery_24, 8000, 0xFFFF, 0},      //15.电机1下降
			{Battery_21, 10000, 0x14, 2},      //33.飞机前进
			{LeaveCenter, 10000, 0x15, 2},     //34.居中杆释放			
};
#define DOWNBATTERY_STEPS_2_COUNT (sizeof(downbattery_steps_2) / sizeof(downbattery_steps_2[0]))

// 下电池步骤(放入3号仓)
static const StepDef downbattery_steps_3[] = 
{
      {Center_1, 5000, 0xFFFF, 0},        //1.左右居中      
			{Center_2, 13000, 0x15, 4},       //2.前后居中
			{Battery_1, 16000, 0xFFFF, 0},      //3.电机1上升
			{Battery_2, 5000, 0xFFFF, 0},       //4.飞机前进
			{Battery_3, 18000, 0xFFFF, 0},      //5.电机2前进
			{Battery_4, 1000, 0xFFFF, 0},        //6.电机3夹紧
			{Battery_5, 15000, 0xFFFF, 0},      //7.电机2后退
			{Battery_6, 5000, 0xFFFF, 0},       //8.飞机后退
			{Battery_7, 18000, 0xFFFF, 0},      //9.电机1下降			
			{Battery_14, 2000, 0xFFFF, 0},       //10.电机1上升
			{Battery_9, 19000, 0xFFFF, 0},      //11.电机2前进
			{Battery_10, 1000, 0xFFFF, 0},       //12.电机3松开
			{Battery_11, 2000, 0xFFFF, 0},      //13.电机2前进
			{Battery_12, 18000, 0xFFFF, 0},     //14.电机2后退
			{Battery_15, 2000, 0xFFFF, 0},      //15.电机1下降
			{Battery_21, 10000, 0x14, 2},      //33.飞机前进
			{LeaveCenter, 10000, 0x15, 2},     //34.居中杆释放			
};
#define DOWNBATTERY_STEPS_3_COUNT (sizeof(downbattery_steps_3) / sizeof(downbattery_steps_3[0]))
	
// 全局变量记录当前是否有旧电池（由主站轮询或状态寄存器更新）
extern uint8_t g_has_old_battery;   // 1:有电池, 0:无电池

// 步骤表1：有电池时，执行取下电池并放入空电池仓
static const StepDef cancel_with_battery_steps[] = 
{
    {Motor1Up1, 5000, 0xFFFF, 0},             	//3.电机1上升1
		{FlyForward, 5000, 0xFFFF, 0},						  //4.飞机前进
    {Motor2Forward, 10000, 0xFFFF, 0},          //5.电机2前进		
		{Motor3Clamp, 500, 0xFFFF, 0},					    //6.电机3夹紧
    {FlyBack, 12000, 0xFFFF, 0},					      //7.飞机后退		
		{Motor2Back, 10000, 0xFFFF, 0}, 					  //8.电机2后退
    {Motor1Down1, 13000, 0xFFFF, 0},						//9.电机1下降1		
		{Motor2Forward, 10000, 0xFFFF, 0},					//10.电机2前进
		{Motor3Lossen, 500, 0xFFFF, 0},							//11.电机3松开
		{Motor2Back, 10000, 0xFFFF, 0},							//10.电机2后退 
};
#define CANCEL_WITH_BATTERY_COUNT (sizeof(cancel_with_battery_steps)/sizeof(cancel_with_battery_steps[0]))

// 步骤表2：无电池时，只需复位电机到初始状态（无需取电池）
static const StepDef cancel_without_battery_steps[] = 
{
    {Motor2Back, 5000, 0xFFFF, 0},              // 电机2后退（确保在初始位）
    {Motor1Down1, 1000, 0xFFFF, 0},             // 电机1下降
		{Motor3Lossen, 500, 0xFFFF, 0},							// 电机3松开
    {LeaveCenter, 15000, 0x0015, 2},            // 居中杆释放
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

void Sequence_Start(SeqId id)
{
    if (seq_runner.busy) {
        printf("序列已在执行中，忽略新请求\r\n");
        return;
    }
		
		    // 强制将主站状态重置为IDLE
    if (master_state != MASTER_IDLE) {
        printf("序列启动前重置主站状态，原状态: %d\r\n", master_state);
        master_state = MASTER_IDLE;
        timeout_cnt = 0;
        memset(Master_RX_BUFF, 0, sizeof(Master_RX_BUFF));
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
            return;
    }
		// ========== 拍摄状态快照 ==========
    StatusRegs_TakeSnapshot();
		
    seq_runner.id = id;
    seq_runner.current_index = 0;
    seq_runner.wait_until = 0;
    seq_runner.busy = 1;
    seq_runner.error = 0;
		seq_runner.paused = 0; 
		seq_runner.pending_update_addr = 0xFFFF;
    seq_runner.retry_count = 0;   // 重置重试计数
}


// 执行当前步骤（发送指令）
//static void ExecuteCurrentStep(void)
//{
//		const StepDef *step = &seq_runner.steps[seq_runner.current_index];
//    
//    if (MasterPolling_IsBusy()) {
//			printf("ExecuteCurrentStep: 总线忙，跳过步骤 %d\n", seq_runner.current_index);
//        return;
//    }
//    
//    MasterBusy_Acquire();
//    uint8_t ret = step->func();
//    MasterBusy_Release();
//    
//    if (ret == 0) {
//        if (step->wait_ms > 0) {
//            // 启动硬件定时器，超时时间 = wait_ms 毫秒
//            bsp_StartHardTimer(1, step->wait_ms * 1000, StepTimerCallback);
//					  //bsp_StartHardTimer(1, step->wait_ms , StepTimerCallback);
//            seq_runner.wait_until = 1;   // 标记为等待状态（非零即可）
//						// 保存挂起的更新信息（如果地址不是0xFFFF）
//            if (step->update_addr != 0xFFFF) 
//						{
//                seq_runner.pending_update_addr = step->update_addr;
//                seq_runner.pending_update_value = step->update_value;
//            }
//						else 
//						{
//                seq_runner.pending_update_addr = 0xFFFF;
//            }
//            printf("步骤 %d 成功，启动硬件定时器 %d ms\r\n", seq_runner.current_index, step->wait_ms);
//        } 
//				else 
//				{
//            // 无等待，立即进入下一步
//						if (step->update_addr != 0xFFFF) 
//						{
//                StatusRegs_Update(step->update_addr, step->update_value);
//            }
//            seq_runner.current_index++;
//            if (seq_runner.current_index >= seq_runner.step_count) {
//                seq_runner.busy = 0;
//                printf("序列执行完成\r\n");
//								// 释放快照，避免长期占用
//								StatusRegs_ReleaseSnapshot();
//            }
//        }
//    } 
//		else if (ret == 1) 
//		{
//        printf("步骤 %d 需要继续执行\r\n", seq_runner.current_index);
//    } 
//		else 
//		{
//        printf("步骤 %d 执行失败，错误码 %d\r\n", seq_runner.current_index, ret);
//        seq_runner.error = 1;
//        seq_runner.busy = 0;
//    }
//}

static void ExecuteCurrentStep(void)
{
    const StepDef *step = &seq_runner.steps[seq_runner.current_index];

    if (MasterPolling_IsBusy()) {
        printf("ExecuteCurrentStep: 总线忙，跳过步骤 %d\n", seq_runner.current_index);
        return;
    }

    MasterBusy_Acquire();
    uint8_t ret = step->func();
    MasterBusy_Release();

    if (ret == 0) {
        // 正常成功
        if (step->wait_ms > 0) {
            bsp_StartHardTimer(1, step->wait_ms * 1000, StepTimerCallback);
            seq_runner.wait_until = 1;
            if (step->update_addr != 0xFFFF) {
                seq_runner.pending_update_addr = step->update_addr;
                seq_runner.pending_update_value = step->update_value;
            } else {
                seq_runner.pending_update_addr = 0xFFFF;
            }
            printf("步骤 %d 成功，启动硬件定时器 %d ms\r\n", seq_runner.current_index, step->wait_ms);
        } else {
            if (step->update_addr != 0xFFFF) {
                StatusRegs_Update(step->update_addr, step->update_value);
            }
            seq_runner.current_index++;
            if (seq_runner.current_index >= seq_runner.step_count) {
                seq_runner.busy = 0;
                printf("序列执行完成\r\n");
                StatusRegs_ReleaseSnapshot();
            }
        }
    } else if (ret == 1) {
        printf("步骤 %d 需要继续执行\r\n", seq_runner.current_index);
    } else if (ret == 3) {
				// 堵转触发重试
        seq_runner.retry_count++;
        printf("步骤 %d 触发重试，当前重试次数 %d/3\r\n", seq_runner.current_index, seq_runner.retry_count);
        if (seq_runner.retry_count < 3) {
            // 重启整个序列
            uint8_t seq_id = seq_runner.id;
            seq_runner.busy = 0;
            Sequence_Start(seq_id);
        } else {
            // 超过重试次数，更新故障码为0xFF，终止序列
            StatusRegs_Update(REG_FAULT_CODE, 0x00FF);
            printf("重试次数超过3次，更新故障码0xFF，序列终止\r\n");
            seq_runner.error = 1;
            seq_runner.busy = 0;
        }
    } else {
        printf("步骤 %d 执行失败，错误码 %d\r\n", seq_runner.current_index, ret);
        seq_runner.error = 1;
        seq_runner.busy = 0;
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
            if (seq_runner.pending_update_addr != 0xFFFF) {
                StatusRegs_Update(seq_runner.pending_update_addr, seq_runner.pending_update_value);
                seq_runner.pending_update_addr = 0xFFFF;
            }

            // 进入下一步
            seq_runner.current_index++;
            if (seq_runner.current_index >= seq_runner.step_count) {
                printf("序列执行完毕\r\n");
                seq_runner.busy = 0;
                seq_runner.paused = 0;
                StatusRegs_ReleaseSnapshot();
            } else {
                printf("等待结束，进入步骤 %d\r\n", seq_runner.current_index);
            }
        }
        return;
    }

    // 若暂停，则不允许执行新步骤
    if (seq_runner.paused) return;
		

    if (seq_runner.current_index < seq_runner.step_count) {
				printf("Sequence_Process: 执行步骤 %d\n", seq_runner.current_index);
        ExecuteCurrentStep();
    }
}

// 判断无人机内是否有旧电池（根据电池充电状态）
static uint8_t HasOldBatteryInUAV(void)
{
    // 读取三块电池的充电状态（0=充电中，1=未充电）
    uint16_t bat1_charge = StatusRegs_Get(REG_BAT1_CHARGE_STATE);
    uint16_t bat2_charge = StatusRegs_Get(REG_BAT2_CHARGE_STATE);
    uint16_t bat3_charge = StatusRegs_Get(REG_BAT3_CHARGE_STATE);

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
        printf("序列已在执行中，忽略取消请求\r\n");
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
    seq_runner.pending_update_addr = 0xFFFF;
    
    // 拍摄状态快照（可选）
    StatusRegs_TakeSnapshot();
}



//////////////////////////////////////居中升降主体函数//////////////////////////////////////////////////
uint8_t Center_1(void)
{
		MotorControlParams motors[4] = 
		{
			{MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_length_l, MOTOR10_length_h},
			{MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_length_l, MOTOR12_length_h},
			{MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_length_l, MOTOR9_length_h},
			{MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR11_length_l, MOTOR11_length_h}
    };
    // 同步发送所有电机指令（不等待响应）
    Sync_Motors_Control(motors, 4);
		
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

//uint8_t Center_1(void)
//{
//    MotorControlParams motors[4] = {
//        {MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_length_l, MOTOR10_length_h},
//        {MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_length_l, MOTOR12_length_h},
//        {MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_length_l, MOTOR9_length_h},
//        {MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR11_length_l, MOTOR11_length_h}
//    };
//    Sync_Motors_Control(motors, 4);

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
		MotorControlParams motors[4] = 
		{
			{MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR14_length_l, MOTOR14_length_h},
			{MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR14_length_l, MOTOR14_length_h},
      {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR8_length_l, MOTOR8_length_h},
			{MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR8_length_l, MOTOR8_length_h}
    };
    uint8_t results[4];
    Sync_Motors_Control(motors, 4);
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}
//uint8_t Center_2(void)
//{
//    MotorControlParams motors[4] = {
//        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR14_length_l, MOTOR14_length_h},
//        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR14_length_l, MOTOR14_length_h},
//        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR8_length_l, MOTOR8_length_h},
//        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR8_length_l, MOTOR8_length_h}
//    };
//    Sync_Motors_Control(motors, 4);

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
		//需要修改具体参数  
		MotorControlParams motors[8] = 
		{
			{MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_length_l, MOTOR13_length_h},
			{MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_length_l, MOTOR13_length_h},
      {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR7_length_l, MOTOR7_length_h},
			{MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR7_length_l, MOTOR7_length_h},
			{MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_length_l, MOTOR9_length_h},
			{MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR11_length_l, MOTOR11_length_h},
			{MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_length_l, MOTOR10_length_h},
			{MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_length_l, MOTOR12_length_h}
    };
    uint8_t results[8];
    uint8_t ret = Control_Motors_Complete(motors, 8, results);
    if (ret == 0) {
        printf("纵向归中控制成功\r\n");
    } else {
        printf("纵向归中控制有 %d 个失败\r\n", ret);
    }

		master_state = MASTER_IDLE;
    timeout_cnt = 0;
    return 0; 
}

//uint8_t LeaveCenter(void)
//{
//    MotorControlParams motors[8] = {
//        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_length_l, MOTOR13_length_h},
//        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_length_l, MOTOR13_length_h},
//        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR17_length_l, MOTOR17_length_h},
//        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR17_length_l, MOTOR17_length_h},
//        {MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_length_l, MOTOR9_length_h},
//        {MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR11_length_l, MOTOR11_length_h},
//        {MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_length_l, MOTOR10_length_h},
//        {MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_length_l, MOTOR12_length_h}
//    };
//    Sync_Motors_Control(motors, 8);

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
		//需要修改具体参数  
		MotorControlParams motors[8] = 
		{
			{MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_length_l, MOTOR13_length_h},
			{MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_length_l, MOTOR13_length_h},
      {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR7_length_l, MOTOR17_length_h},
			{MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR7_length_l, MOTOR17_length_h},
			{MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_length_l, MOTOR9_length_h},
			{MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR11_length_l, MOTOR11_length_h},
			{MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_length_l, MOTOR10_length_h},
			{MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_length_l, MOTOR12_length_h}
    };
    uint8_t results[8];
    uint8_t ret = Control_Motors_Complete(motors, 8, results);
    if (ret == 0) {
        printf("纵向归中控制成功\r\n");
    } else {
        printf("纵向归中控制有 %d 个失败\r\n", ret);
    }
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
    return 0; 
}

uint8_t LeaveCenter2(void)
{
		//需要修改具体参数  
		MotorControlParams motors[8] = 
		{
			{MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_length_l, MOTOR13_length_h},
			{MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_length_l, MOTOR13_length_h},
      {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR17_length_l, MOTOR17_length_h},
			{MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR17_length_l, MOTOR17_length_h},
			{MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_length_l, MOTOR9_length_h},
			{MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR11_length_l, MOTOR11_length_h},
			{MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_length_l, MOTOR10_length_h},
			{MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_length_l, MOTOR12_length_h}
    };
    uint8_t results[8];
    uint8_t ret = Control_Motors_Complete(motors, 8, results);
    if (ret == 0) {
        printf("纵向归中控制成功\r\n");
    } else {
        printf("纵向归中控制有 %d 个失败\r\n", ret);
    }
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
    return 0; 
}

//uint8_t LeaveCenter1(void)
//{
//    MotorControlParams motors[8] = {
//        {MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_length_l, MOTOR13_length_h},
//        {MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR13_length_l, MOTOR13_length_h},
//        {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR17_length_l, MOTOR17_length_h},
//        {MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR17_length_l, MOTOR17_length_h},
//        {MOTOR10_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR9_length_l, MOTOR9_length_h},
//        {MOTOR12_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR11_length_l, MOTOR11_length_h},
//        {MOTOR9_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR10_length_l, MOTOR10_length_h},
//        {MOTOR11_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR12_length_l, MOTOR12_length_h}
//    };
//    Sync_Motors_Control(motors, 8);

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
	  // 电机1上升
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, Pulse_num7, Pulse_num8},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_2(void)
{
	  // 飞机前进
		MotorControlParams motors[4] = 
		{
			{MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_length_l, MOTOR6_length_h},
			{MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_length_l, MOTOR6_length_h},
      {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_length_l, MOTOR5_length_h},
			{MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_length_l, MOTOR5_length_h}
    };
    uint8_t results[4];
    // 同步发送所有电机指令（不等待响应）
    Sync_Motors_Control(motors, 4);
//		StatusRegs_Update(0x15, 4); //将居中状态写入寄存器
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_3(void)
{
	
		// 电机2前进
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, Pulse_num1, Pulse_num2},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

//uint8_t Battery_4(void)
//{
//  	//电机3夹紧电池
//		uint8_t motor_num3 = 2;   // 寄存器地址（夹紧）
//		uint16_t motor_cmd3 = Clamp; // 电机指令：
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
//    uint8_t ret = Modbus_06_WriteSingleReg(MOTOR3_SLAVE_ADDR, MOTOR3_CTRL_REG2, Clamp);
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

uint8_t Battery_4(void)
{
    printf("Battery_4 开始执行\n");
    // 1. 强制恢复主站状态，清空接收缓冲区
    __disable_irq();
    Master_RX_CNT = 0;
    memset(Master_RX_BUFF, 0, sizeof(Master_RX_BUFF));
    __enable_irq();
    master_state = MASTER_IDLE;
    timeout_cnt = 0;
    delay_ms(5); // 确保总线稳定

    // 2. 构造06帧并直接发送（不调用Modbus_06_WriteSingleReg）
    uint8_t tx_buff[8];
    tx_buff[0] = MOTOR3_SLAVE_ADDR;
    tx_buff[1] = 0x06;
    tx_buff[2] = (MOTOR3_CTRL_REG2 >> 8) & 0xFF;
    tx_buff[3] = MOTOR3_CTRL_REG2 & 0xFF;
    tx_buff[4] = (Clamp >> 8) & 0xFF;
    tx_buff[5] = Clamp & 0xFF;
    uint16_t crc = Modbus_CRC16(tx_buff, 6);
    tx_buff[6] = crc & 0xFF;
    tx_buff[7] = (crc >> 8) & 0xFF;

    RS485_MasterSendData(tx_buff, 8); // 发送
    // 3. 立即恢复状态，不等待响应
    master_state = MASTER_IDLE;
    timeout_cnt = 0;
    __disable_irq();
    Master_RX_CNT = 0;
    __enable_irq();
    printf("电机3夹紧指令已发送（忽略响应）\r\n");
    return 0;
}

uint8_t Battery_5(void)
{
	  //电机2后退
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, Pulse_num19, Pulse_num20},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}


uint8_t Battery_6(void)
{
	  //飞机后退
		MotorControlParams motors[4] = 
		{
			{MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_length_l, MOTOR5_length_h},
			{MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR5_length_l, MOTOR5_length_h},
			{MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_length_l, MOTOR6_length_h},
			{MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR6_length_l, MOTOR6_length_h}
    };
    uint8_t results[4];
    uint8_t ret = Control_Motors_Complete(motors, 4, results);
    if (ret == 0) {
        printf("纵向归中控制成功\r\n");
    } else {
        printf("纵向归中控制有 %d 个失败\r\n", ret);
    }
//		StatusRegs_Update(0x15, 2);
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_7(void)
{
  	//电机1下降
		uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, Pulse_num9, Pulse_num10},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_8(void)
{
  	//电机1上升
		uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, Pulse_num11, Pulse_num12},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_9(void)
{
	  //电机2前进
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, Pulse_num27, Pulse_num28},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

//uint8_t Battery_10(void)
//{
//  	//电机3松开电池
//		uint8_t motor_num3 = 1;   // 寄存器地址（松开）
//		uint16_t motor_cmd3 = Lossen; // 电机指令：
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
//    uint8_t ret = Modbus_06_WriteSingleReg(MOTOR3_SLAVE_ADDR, MOTOR3_CTRL_REG1, Lossen);
//    // 由于电机已动作，强制认为成功
//    master_state = MASTER_IDLE;
//    timeout_cnt = 0;
//    __disable_irq();
//    Master_RX_CNT = 0;
//    __enable_irq();
//    printf("电机3松开指令已发送（忽略响应）\r\n");
//    return 0;
//}

uint8_t Battery_10(void)
{
    printf("Battery_10 开始执行\n");
    // 1. 强制恢复主站状态，清空接收缓冲区
    __disable_irq();
    Master_RX_CNT = 0;
    memset(Master_RX_BUFF, 0, sizeof(Master_RX_BUFF));
    __enable_irq();
    master_state = MASTER_IDLE;
    timeout_cnt = 0;
    delay_ms(5); // 确保总线稳定

    // 2. 构造06帧并直接发送（不调用Modbus_06_WriteSingleReg）
    uint8_t tx_buff[8];
    tx_buff[0] = MOTOR3_SLAVE_ADDR;
    tx_buff[1] = 0x06;
    tx_buff[2] = (MOTOR3_CTRL_REG1 >> 8) & 0xFF;
    tx_buff[3] = MOTOR3_CTRL_REG1 & 0xFF;
    tx_buff[4] = (Clamp >> 8) & 0xFF;
    tx_buff[5] = Clamp & 0xFF;
    uint16_t crc = Modbus_CRC16(tx_buff, 6);
    tx_buff[6] = crc & 0xFF;
    tx_buff[7] = (crc >> 8) & 0xFF;

    RS485_MasterSendData(tx_buff, 8); // 发送
    // 3. 立即恢复状态，不等待响应
    master_state = MASTER_IDLE;
    timeout_cnt = 0;
    __disable_irq();
    Master_RX_CNT = 0;
    __enable_irq();
    printf("电机3松开指令已发送（忽略响应）\r\n");
    return 0;
}

uint8_t Battery_11(void)
{
	  //电机2前进
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, Pulse_num5, Pulse_num6},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}


uint8_t Battery_12(void)
{
	  //电机2后退
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, Pulse_num3, Pulse_num4},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_13(void)
{
	  //电机1下降
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, Pulse_num13, Pulse_num14},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_14(void)
{
	  //电机1上升
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, Pulse_num15, Pulse_num16},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_15(void)
{
	  //电机1下降
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, Pulse_num17, Pulse_num18},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_16(void)
{
	  //电机2前进
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, Pulse_num21, Pulse_num22},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_17(void)
{
	  //电机2后退
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, Pulse_num23, Pulse_num24},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_18(void)
{
	  //电机2前进
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, Pulse_num31, Pulse_num32},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_19(void)
{
	  //电机2后退
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, Pulse_num33, Pulse_num34},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}


uint8_t Battery_20(void)
{
	  //电机2后退
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, Pulse_num25, Pulse_num26},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_21(void)
{
	  //飞机前进
		MotorControlParams motors[4] = 
		{
			{MOTOR5_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR16_length_l, MOTOR16_length_h},
			{MOTOR6_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR16_length_l, MOTOR16_length_h},
      {MOTOR7_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR15_length_l, MOTOR15_length_h},
			{MOTOR8_SLAVE_ADDR, MOTOR5_CTRL_REG1, MOTOR15_length_l, MOTOR15_length_h}
    };
    uint8_t results[4];
    // 同步发送所有电机指令（不等待响应）
    Sync_Motors_Control(motors, 4);
//		StatusRegs_Update(0x15, 4); //将居中状态写入寄存器
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_22(void)
{
	  //电机2后退
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, Pulse_num29, Pulse_num30},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_23(void)
{
	  //2号仓电机1上升
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, Pulse_num35, Pulse_num36},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_24(void)
{
	  //2号仓电机1下降
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, Pulse_num37, Pulse_num38},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_25(void)
{
	  //电机2后退
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, Pulse_num39, Pulse_num40},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_26(void)
{
	
		// 电机2前进
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, Pulse_num41, Pulse_num42},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}


uint8_t Battery_27(void)
{
	  // 电机1上升
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, Pulse_num43, Pulse_num44},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t Battery_28(void)
{
		// 电机2前进
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG1, Pulse_num45, Pulse_num46},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}
	
uint8_t Battery_29(void)
{
	  // 电机1上升
  	uint8_t ctrl_ret4;  // 指令发送返回值 
		uint16_t motor_cmd4 = MOTOR4_speed; // 速度500（0x01f4）
	  MotorControlParams motors[1] = 
		{
			{MOTOR1_SLAVE_ADDR, MOTOR1_CTRL_REG1, Pulse_num47, Pulse_num48},
    };
    uint8_t results[1];
		uint8_t ret = Control_Motors_Complete(motors, 1, results);
		if (ret == 0) {
				printf("所有电机控制成功\r\n");
		} else {
				printf("有 %d 个电机控制失败\r\n", ret);
		}
		master_state = MASTER_IDLE;
    timeout_cnt = 0;
		return 0;
}

uint8_t CloseDr(void)
{
  	//4.舱体关闭
		uint8_t motor_num4 = 1;   // 寄存器地址（夹紧）
		uint16_t motor_cmd4 = CLOSEC; // 电机指令：
		uint8_t ctrl_ret3 = Motor_Control(MOTOR4_SLAVE_ADDR, motor_num4, motor_cmd4);
		if(ctrl_ret3 == 0)
    {
        printf("舱体关闭指令发送成功！\r\n");
    }
    else
    {
        printf("舱体关闭指令发送失败，错误码：%d\r\n", ctrl_ret3);
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
		return 0;
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
//    uint16_t motor_cmd4 = CLOSEC;
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
    uint16_t uav_status = StatusRegs_Get(REG_RESERVED4);
		printf("CheckAndCloseDoor: uav_status=%d\n", uav_status);
    if (uav_status == 3) {
        printf("检测到无人机不在机巢（0x60=3），执行关闭舱门\n");
        uint8_t ret = CloseDr();
        if (ret != 0) {
            printf("关闭舱门执行失败，错误码：%d\n", ret);
        }
    } else {
        printf("0x60=%d，无需关闭舱门\n", uav_status);
    }
    master_state = MASTER_IDLE;
    timeout_cnt = 0;
    return 0;
}

uint8_t StopDr(void)
{
  	//4.舱体停止
		uint8_t motor_num4 = 1;   // 寄存器地址（夹紧）
		uint16_t motor_cmd4 = STOPC; // 电机指令：
		uint8_t ctrl_ret3 = Motor_Control(MOTOR4_SLAVE_ADDR, motor_num4, motor_cmd4);
		if(ctrl_ret3 == 0)
    {
        printf("舱体停止指令发送成功！\r\n");
    }
    else
    {
        printf("舱体停止指令发送失败，错误码：%d\r\n", ctrl_ret3);
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
		return 0;
}


uint8_t OpenDr(void)
{
  	//4.舱体打开
		uint8_t motor_num4 = 1;   // 寄存器地址
		uint16_t motor_cmd4 = OPENC; // 电机指令：
		uint8_t ctrl_ret3 = Motor_Control(MOTOR4_SLAVE_ADDR, motor_num4, motor_cmd4);
		if(ctrl_ret3 == 0)
    {
        printf("舱体打开指令发送成功！\r\n");
    }
    else
    {
        printf("舱体打开指令发送失败，错误码：%d\r\n", ctrl_ret3);
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
		return 0;
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
//    uint16_t motor_cmd4 = OPENC;
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
		uint8_t motor_num13 = 1;   // 寄存器地址
		uint16_t motor_cmd13 = OPENAC; // 电机指令：
		uint8_t ctrl_ret3 = Motor_Control(MOTOR13_SLAVE_ADDR, motor_num13, motor_cmd13);
		if(ctrl_ret3 == 0)
    {
        printf("空调打开指令发送成功！\r\n");
    }
    else
    {
        printf("空调打开指令发送失败，错误码：%d\r\n", ctrl_ret3);
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
		return 0;
}

uint8_t CloseAC(void)
{
  	//关闭空调
		uint8_t motor_num13 = 1;   // 寄存器地址
		uint16_t motor_cmd13 = CLOSEAC; // 电机指令：
		uint8_t ctrl_ret3 = Motor_Control(MOTOR13_SLAVE_ADDR, motor_num13, motor_cmd13);
		if(ctrl_ret3 == 0)
    {
        printf("空调关闭指令发送成功！\r\n");
    }
    else
    {
        printf("空调关闭指令发送失败，错误码：%d\r\n", ctrl_ret3);
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
		return 0;
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

void Sequence_ForceStop(void) {
        seq_runner.busy = 0;       
        seq_runner.wait_until = 0; 
        seq_runner.error = 1;      
        seq_runner.paused = 0;     
        g_step_timer_expired = 0;  // 清除可能残余的定时器标志
}