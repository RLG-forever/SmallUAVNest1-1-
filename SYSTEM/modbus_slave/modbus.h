#ifndef _MODBUS_H
#define _MODBUS_H

#define RS485_TX_EN PAout(4)
#include "stm32f4xx.h" 
#include <string.h>
#include <stdio.h>
#include <stdint.h> 


#define ALARM_POLL_INTERVAL_MS  1000   // 1秒轮询一次
// -------------------------- 硬件相关宏定义 --------------------------
#define USARTx               USART1        // 选用USART1作为RS485通信口
#define USART_BAUDRATE       9600         // Modbus-RTU波特率
#define RS485_RE_GPIO        GPIOC         // EN485=GPIOC
#define RS485_RE_PIN         GPIO_Pin_0    // EN485_PIN=GPIOA_Pin_8
#define RS485_DE_GPIO        GPIOC         // 复用RE引脚（硬件上RE/DE通常连在一起）
#define RS485_DE_PIN         GPIO_Pin_0

// USART引脚（匹配定义：USART_TX=GPIOB_Pin_10, USART_RX=GPIOB_Pin_11）
#define USART_TX_GPIO        GPIOA
//#define USART_TX_PIN         GPIO_Pin_9
#define USART_RX_GPIO        GPIOA
//#define USART_RX_PIN         GPIO_Pin_10

// 继电器控制引脚定义
#define RELAY_FORWARD_PIN    GPIO_Pin_2
#define RELAY_BACKWARD_PIN   GPIO_Pin_3
#define RELAY_GPIO_PORT      GPIOE
// 为了方便管理，定义全部引脚掩码
#define RELAY_ALL_PINS       (RELAY_FORWARD_PIN | RELAY_BACKWARD_PIN)

// -------------------------- Modbus主站配置 --------------------------
#define MOTOR_COUNT      3       // 控制3个电机
#define Reg_Number     Reg_num   //寄存器数量
// 1. 电机标识（区分不同电机）
#define MOTOR_ID_1       2       // 电机1的唯一标识
#define MOTOR_ID_2       3       // 电机2的唯一标识
#define MOTOR_ID_3       1       // 电机3的唯一标识
#define MOTOR_ID_4       4       // 电机4的唯一标识
#define MOTOR_ID_13      13      // 电机13的唯一标识
#define MOTOR_TOTAL      5       // 电机总数（仅用于合法性校验）
// 每个电机独立的Modbus从站地址
#define MOTOR1_SLAVE_ADDR  0x03   // 电机1从站地址
#define MOTOR2_SLAVE_ADDR  0x02   // 电机2从站地址
#define MOTOR3_SLAVE_ADDR  0x01   // 电机3从站地址
#define MOTOR4_SLAVE_ADDR  0x04   // 舱门从站地址
#define MOTOR5_SLAVE_ADDR  0x05   // 电机5从站地址
#define MOTOR6_SLAVE_ADDR  0x06   // 电机6从站地址
#define MOTOR7_SLAVE_ADDR  0x07   // 电机7从站地址
#define MOTOR8_SLAVE_ADDR  0x08   // 电机8从站地址
#define MOTOR9_SLAVE_ADDR  0x09   // 电机9从站地址
#define MOTOR10_SLAVE_ADDR 0x0a   // 电机10从站地址
#define MOTOR11_SLAVE_ADDR 0x0b   // 电机11从站地址
#define MOTOR12_SLAVE_ADDR 0x0c   // 电机12从站地址
#define MOTOR13_SLAVE_ADDR 0x0d   // 空调从站地址
#define MOTOR16_SLAVE_ADDR 0x0e   // 雨量地址
#define MOTOR17_SLAVE_ADDR 0x0f   // 风速地址
#define MOTOR14_SLAVE_ADDR 0x14   // 电机14从站地址(控制遥控器的舵机)
#define MOTOR15_SLAVE_ADDR 0x13   // 电机15从站地址(电池充电空开)
#define TIMEOUT_MS           500           // 通信超时时间(ms)
// 06功能码帧长度（固定8字节）
#define MODBUS_06_FRAME_LEN 8
// ======================== 步进电机控制寄存器映射 ========================
// 06/10功能码 - 写寄存器（控制指令）16进制
//电机1
#define MOTOR1_CTRL_REG1      0x00ce  // 电机1控制寄存器

//电机2
#define MOTOR2_CTRL_REG1      0x00ce  // 电机2控制寄存器
#define MOTOR2_CTRL_REG5      0x00fd  // 电机5控制寄存器（位置模式控制）

//电机3
#define MOTOR3_CTRL_REG1      0x0036  // 电机1控制寄存器地址（松开）
#define MOTOR3_CTRL_REG2      0x0037  // 电机2控制寄存器地址（夹紧）
#define MOTOR3_CTRL_REG3      0x0047  // 电机3控制寄存器地址（保存）

//舱门
#define MOTOR4_CTRL_REG1       0x0046  // 电机4控制寄存器地址
#define OPENC                  0x0002  // 开舱
#define CLOSEC                 0x0001  // 关舱
#define STOPC                  0x0005  // 停止

//电机5-12
#define MOTOR5_CTRL_REG1      0x00ce  // 电机5-12控制寄存器地址（行程设置）

//空调
#define MOTOR13_CTRL_REG1       0x002f  // 空调控制寄存器地址
#define OPENAC                  0x0001  // 打开空调
#define CLOSEAC                 0x0000  // 关闭空调

// 03功能码 - 读寄存器（状态读取）
//电机1
#define MOTOR1_STATUS_REG1    0x0046 // 电机1状态寄存器
#define MOTOR1_STATUS_REG2    0x0047 // 电机2状态寄存器
#define MOTOR1_STATUS_REG3    0x0048 // 电机3状态寄存器
#define MOTOR1_STATUS_REG4    0x0045 // 电机4状态寄存器
#define MOTOR1_STATUS_REG5    0x004b // 电机5状态寄存器
#define MOTOR1_STATUS_REG6    0x003c // 电机6状态寄存器


// 电机状态值定义（从站返回）
#define MOTOR1_STATUS_IDLE    0x0000        // 空闲
#define MOTOR1_STATUS_RUNNING 0x0001        // 运行中
#define MOTOR1_STATUS_ERROR   0x0002        // 故障
#define MOTOR1_STATUS_ZERO    0x0003        // 已回零

//电机1
// 位置控制模式
// 位置控制模式
#define Pulse_num7            0xF6F0        //行程低位，远离电机-332000  ef20
#define Pulse_num8            0xfffa        //行程高位
#define Pulse_num9            0x10e0        //程低位，靠近电机332000
#define Pulse_num10           0x0006        //行程高位   5
#define Pulse_num11           0xecc0        //程低位，远离电机-136000
#define Pulse_num12           0xfffd        //行程高位
#define Pulse_num13           0x1340        //程低位，靠近电机136000
#define Pulse_num14           0x0002        //行程高位
#define Pulse_num15           0xf830        //程低位，远离电机-2000
#define Pulse_num16           0xffff        //行程高位
#define Pulse_num17           0x07d0        //程低位，靠近电机2000
#define Pulse_num18           0x0000        //行程高位
#define Pulse_num35           0xf278        //程低位，远离电机-69000
#define Pulse_num36           0xfffe        //行程高位
#define Pulse_num37           0x0d88        //程低位，靠近电机69000
#define Pulse_num38           0x0001        //行程高位
#define Pulse_num43             0xFAD8        //电机2端低位脉冲数-329000
#define Pulse_num44             0xfffa        //电机2端高位脉冲数






//电机2
//速度模式控制
#define  Direction             0x0000        //方向（高位：00向前，01向后；低位：00表示加速度档位为0)
#define  Direction_back        0x0100
#define  Speed                 0x01f4        //速度（两字节）
#define  Multi_Flag            0x0000        //多机同步标志（高位单字节0/1,低字节为00）
//立即停止
#define  Motor_Stop2           0x9800        //
//触发回零
#define  Motor2_Back_ZERO      0x0300        //多圈限位回零
//使能信号控制
#define  EN_State              0xab01        //使能状态（00不使能/01使能）
#define  Multi_Flag            0x0000        //多机同步标志（高位单字节0/1,低字节为00）
//位置模式
#define Position1              0x0000        //相对模式
#define Position2              0x0001        //绝对模式
#define Pulse_num1             0x3188        //电机2端低位脉冲数-315000    8778
#define Pulse_num2             0xfffb        //电机2端高位脉冲数
#define Pulse_num3             0xa168        //电机2端低位脉冲数369000
#define Pulse_num4             0x0006        //电机2端高位脉冲数	  5
#define Pulse_num5             0xDCD8       //电机2端低位脉冲数-9000     eb89 
#define Pulse_num6             0xffff        //电机2端高位脉冲数
#define Pulse_num19             0x7888        //电机2端低位脉冲数293000
#define Pulse_num20             0x0005        //电机2端高位脉冲数   5
#define Pulse_num21             0xA240        //电机2端低位脉冲数-15000    c568
#define Pulse_num22             0xffff        //电机2端高位脉冲数
#define Pulse_num23             0x3a98        //电机2端低位脉冲数15000
#define Pulse_num24             0x0000        //电机2端高位脉冲数
#define Pulse_num25             0xb320        //电机2端低位脉冲数308000
#define Pulse_num26             0x0005        //电机2端高位脉冲数   4
#define Pulse_num27             0x7DD8        //电机2端低位脉冲数-363000   7608
#define Pulse_num28             0xfffa        //电机2端高位脉冲数
#define Pulse_num29             0x89f8        //电机2端低位脉冲数363000
#define Pulse_num30             0x0006        //电机2端高位脉冲数   5
#define Pulse_num31             0xd8f0        //电机2端低位脉冲数-10000
#define Pulse_num32             0xffff        //电机2端高位脉冲数
#define Pulse_num33             0x2710        //电机2端低位脉冲数10000
#define Pulse_num34             0x0000        //电机2端高位脉冲数
#define Pulse_num39             0xd260        //电机2端低位脉冲数316000
#define Pulse_num40             0x0005        //电机2端高位脉冲数   4
#define Pulse_num41             0x9330        //电机2端低位脉冲数-290000
#define Pulse_num42             0xfffb        //电机2端高位脉冲数
#define Pulse_num45             0x8778        //电机2端低位脉冲数-295000  7FA8
#define Pulse_num46             0xfffb        //电机2端高位脉冲数
#define Pulse_num47             0x8778        //电机2端低位脉冲数-290000
#define Pulse_num48             0xfffb        //电机2端高位脉冲数


//电机3
#define Lossen                 0x0001         //松开
#define Clamp                  0x0001         //夹紧
#define Save                   0x0001         //保存配置

//归中电机4~8公用寄存器地址
#define speed                  0x009a         //运行速度设置
#define length                 0x00c8         //固定行程运动
#define MOTOR4_RUN             0x0001         //正转连续运动
#define MOTOR4_deRUN           0x0257         //反转运动
#define MOTOR4_stop            0x0256         //停止运动
#define MOTOR4_speed           0x01f4         //运动速度
//#define MOTOR5_length_l        0xbf20         //行程低位，靠近电机180000  
//#define MOTOR5_length_h        0x0002         //行程高位，靠近电机
#define MOTOR5_length_l        0xaf80         //行程低位，靠近电机176000  
#define MOTOR5_length_h        0x0002         //行程高位，靠近电机
//#define MOTOR6_length_l        0x40e0         //行程低位，远离电机-180000
//#define MOTOR6_length_h        0xfffd         //行程高位，远离电机
#define MOTOR6_length_l        0x4F00         //行程低位，远离电机-176000
#define MOTOR6_length_h        0xfffd         //行程高位，远离电机
#define MOTOR7_length_l        0x5d78         //行程低位，靠近电机155000
#define MOTOR7_length_h        0x0002         //行程高位，靠近电机
#define MOTOR8_length_l        0xa8d0        //行程低位，远离电机-350000  
#define MOTOR8_length_h        0xfffa         //行程高位，远离电机
#define MOTOR9_length_l        0x5f90         //行程低位，靠近电机90000
#define MOTOR9_length_h        0x0001         //行程高位，靠近电机
#define MOTOR10_length_l        0xa070        //行程低位，远离电机-90000
#define MOTOR10_length_h        0xfffe         //行程高位，远离电机
#define MOTOR11_length_l        0x82b8         //行程低位，靠近电机99000
#define MOTOR11_length_h        0x0001         //行程高位，靠近电机
#define MOTOR12_length_l        0x7d48        //行程低位，远离电机-99000
#define MOTOR12_length_h        0xfffe         //行程高位，远离电机
#define MOTOR13_length_l        0x3450         //行程低位，靠近电机210000
#define MOTOR13_length_h        0x0003         //行程高位，靠近电机
#define MOTOR14_length_l        0xe4a8        //行程低位，远离电机-7000
#define MOTOR14_length_h        0xffff         //行程高位，远离电机
#define MOTOR15_length_l        0x0d40         //行程低位，靠近电机200000
#define MOTOR15_length_h        0x0003         //行程高位，靠近电机
#define MOTOR16_length_l        0xf2c0        //行程低位，远离电机-200000
#define MOTOR16_length_h        0xfffc         //行程高位，远离电机
#define MOTOR17_length_l        0x5f00         //行程低位，靠近电机352000
#define MOTOR17_length_h        0x0005         //行程高位，靠近电机

//电机12
#define MOTOR12_UP        0x0033  // 上升
#define MOTOR12_DOWN      0x0044  // 下降
#define MOTOR12_STOP      0x0055  // 停止

// 电机ID范围 
#define ALARM_QUERY_REG     0xA3    // 报警状态查询寄存器地址
#define ALARM_CLEAR_REG     0xA4    // 报警解除寄存器地址

// 寄存器地址
#define REG_CURRENT           0x001A   // 实时电流（单位mA）
#define REG_ALARM_STATUS      0x00A3   // 报警状态
#define REG_CLEAR_ALARM       0x00A4   // 清除报警
#define REG_ENABLE            0x00D4   // 使能/脱机控制

// 堵转电流阈值（单位mA，需根据电机实际参数标定）
#define STALL_CURRENT_THRESHOLD_MA  2000   // 例如2A

// 正常工作电流上限（参考值，用于平滑判断，可省略）
#define NORMAL_CURRENT_MAX_MA       1500
#define ALARM_POLL_COUNT_THRESHOLD   200

static const uint8_t motor_slave_addr[] = {
		0x02,
		0x03,
    0x05,
    0x06,
    0x07,
    0x08,
		0x09,
    0x0A,
    0x0B,
	  0x0C,
};
#define MOTOR_ID_START  0
#define MOTOR_ID_END    (sizeof(motor_slave_addr)/sizeof(motor_slave_addr[0]) - 1)

// -------------------------- 全局变量 --------------------------

typedef enum {
    MASTER_IDLE,        // 空闲
    MASTER_SENDING,     // 发送中
    MASTER_WAIT_RESP,   // 等待响应
    MASTER_RESP_OK,     // 响应正常
    MASTER_RESP_ERR     // 响应错误
} ModbusMasterState;



typedef struct {
    uint8_t slave_address;      // 从站地址
    uint16_t contrl_reg;       	// 控制寄存器起始地址
    uint16_t len_l;        			 	// 行程值低位
		uint16_t len_h;         		// 行程值高位
} MotorControlParams;

// 电池更换流程状态枚举
typedef enum {
    SWAP_STATE_IDLE = 0,
    SWAP_STATE_STEP1,          // 步骤1：设置电机1参数
    SWAP_STATE_WAIT1,          // 等待1秒
    SWAP_STATE_STEP2,          // 步骤2：纵向归中
    SWAP_STATE_WAIT2,          // （无等待，直接到下一步）
    SWAP_STATE_STEP3,          // 步骤3：横向移动到边端（未实现）
    SWAP_STATE_WAIT3,
    SWAP_STATE_STEP4,          // 步骤4：电机1上升
    SWAP_STATE_WAIT4,          // 等待11秒
    SWAP_STATE_STEP5,          // 步骤5：电机2前进
    SWAP_STATE_WAIT5,          // 等待7秒
    SWAP_STATE_STEP6,          // 步骤6：电机3夹紧
    SWAP_STATE_WAIT6,          // 等待0.5秒
    SWAP_STATE_STEP7,          // 步骤7：电机2后退
    SWAP_STATE_WAIT7,          // 等待7秒
    SWAP_STATE_STEP8,          // 步骤8：电机1下降
    SWAP_STATE_WAIT8,          // 等待11秒
    SWAP_STATE_STEP9,          // 步骤9：电机2前进至电池仓
    SWAP_STATE_WAIT9,          // 等待6秒
    SWAP_STATE_STEP10,         // 步骤10：电机3松开
    SWAP_STATE_WAIT10,         // 等待0.3秒
    SWAP_STATE_STEP11,         // 步骤11：电机2进0.5万脉冲
    SWAP_STATE_WAIT11,         // 等待0.3秒
    SWAP_STATE_STEP12,         // 步骤12：电机2退0.5万脉冲
    SWAP_STATE_WAIT12,         // 等待0.3秒
    SWAP_STATE_STEP13,         // 步骤13：电机3夹紧
    SWAP_STATE_WAIT13,         // 等待0.5秒
    SWAP_STATE_STEP14,         // 步骤14：电机2后退至原点
    SWAP_STATE_WAIT14,         // 等待6秒
    SWAP_STATE_STEP15,         // 步骤15：电机1上升
    SWAP_STATE_WAIT15,         // 等待11秒
    SWAP_STATE_STEP16,         // 步骤16：电机2前进至无人机
    SWAP_STATE_WAIT16,         // 等待7秒
    SWAP_STATE_STEP17,         // 步骤17：电机3松开
    SWAP_STATE_WAIT17,         // 等待0.1秒
    SWAP_STATE_STEP18,         // 步骤18：电机2进1.1万脉冲
    SWAP_STATE_WAIT18,         // 等待0.5秒
    SWAP_STATE_STEP19,         // 步骤19：电机2退1.1万脉冲
    SWAP_STATE_WAIT19,         // 等待0.2秒
    SWAP_STATE_STEP20,         // 步骤20：电机2后退至原点
    SWAP_STATE_WAIT20,         // 等待7秒
    SWAP_STATE_STEP21,         // 步骤21：电机1下降
    SWAP_STATE_WAIT21,         // 等待11秒
    SWAP_STATE_STEP22,         // 步骤22：横向归中（预留）
    SWAP_STATE_WAIT22,         // 等待1秒（循环间隔）
    SWAP_STATE_FINISH,      	 // 流程结束
    SWAP_STATE_ERROR           // 错误状态
} SwapState;


// 继电器状态枚举
typedef enum {
    RELAY_STOP = 0,
    RELAY_FORWARD,
    RELAY_BACKWARD
} RelayState;


static SwapState current_state = SWAP_STATE_IDLE;
static uint32_t wait_until = 0;
static uint16_t cycle_cnt = 1;      // 当前循环次数
static const uint16_t total_cycles = 100; // 总循环次数

//extern u8 RS485_TX_BUFF[500];  // 发送缓冲区
//extern u8 RS485_RX_BUFF[500];  // 接收缓冲区
// 主站接收缓冲区
extern volatile uint16_t Master_RX_CNT;
extern volatile uint8_t Master_FrameFlag;
extern uint8_t Master_RX_BUFF[2048];
extern volatile uint32_t Master_LastRxTime;   // 最后接收时间

extern uint16_t RX_LEN;            // 接收数据长度
extern ModbusMasterState master_state; // 主站状态
extern uint16_t timeout_cnt;       // 超时计数器

// 外部函数声明（实际的复位函数）
extern uint8_t Battery_22(void);    // 地址0x02复位
extern uint8_t Battery_15(void);    // 地址0x03复位
extern uint8_t LeaveCenter1(void);  // 地址0x05~0x0C复位（集体动作）

// ========================== 静态变量 ==========================
static uint8_t stall_poll_active = 0;      // 是否正在轮询中
static uint8_t stall_motor_index = 0;      // 当前检测的电机索引（0~MOTOR_COUNT-1）
static uint8_t stall_poll_counter = 0;     // 计数器，仅当非轮询时累加
static uint8_t stall_triggered = 0;        // 堵转触发标志（防止重复调用复位）
// 轮询启动计数阈值（主循环每调用一次该任务，计数器+1，达到阈值启动一轮检测）
#define STALL_POLL_THRESHOLD  200   // 若主循环 delay_ms(1000)，则约 3 秒一轮
// 电机从站地址列表（与报警轮询共用，但此处单独列出以保持独立性）
#define MOTORCOUNT  10
static const uint8_t stall_motor_addr_list[MOTORCOUNT] = {
    0x02, 0x03, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C
};

void RS485_Init(unsigned long bound);
void CnfgrTimer3(void);
//void RS485_Service(void);
uint8_t Modbus_03_ReadHoldReg(uint8_t slave_addr, uint16_t start_reg, uint16_t reg_num, uint16_t *read_buff);
//uint8_t Modbus_04_ReadInReg(uint16_t reg_addr, uint16_t reg_num, uint16_t *data_buf);
uint8_t Modbus_06_WriteSingleReg(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_data);
uint8_t Modbus_10_WriteMultiReg(uint8_t slave_addr, uint16_t start_reg, uint16_t reg_num, uint16_t *write_buff);
//void RS485_SendData(uint8_t *pData, uint16_t len);  // RS485数据发送
//void RS485_SendData(unsigned char *buff, unsigned char len);
void RS485_MasterSendData(uint8_t *buff, uint16_t len);//主站数据发送
void RS485_SlaveSendData(uint8_t *buff, uint16_t len);//从站数据发送
uint16_t Modbus_CRC16(uint8_t *pData, uint16_t len); // CRC16校验

// 通信超时检测（需在定时器中断中调用，1ms触发一次）
void Modbus_Timeout_Check(void);
void SysTick_Init(void);
void Timer3Init(void);
void Timer4Init(void);

// 步进电机控制封装
uint8_t Motor_Single_Control(uint8_t slave_addr, uint8_t motor_num, uint16_t motor_cmd); // 单电机控制（06功能码）
uint8_t Motor_Control(uint8_t motor_id, uint8_t reg_num, uint16_t motor_cmd);// 单电机控制（06功能码）
uint8_t Motor_Batch_Control(uint8_t slave_addr, uint16_t start_reg, uint16_t reg_num, uint16_t *motor_cmds); // 批量电机控制（10功能码）
uint8_t Motor_Read_Status(uint8_t motor_num, uint16_t *motor_status); // 读取单电机状态（03功能码）
uint8_t Motor_Batch_Read_Status(uint16_t start_reg, uint16_t motor_num, uint16_t *status_buff); // 批量读取状态（03功能码）


void Quick_Motors_Control(MotorControlParams *motors, uint8_t count);
uint8_t Modbus_Send_NonBlocking(uint8_t slave_addr, uint16_t start_reg, uint16_t reg_num, uint16_t *data);
uint8_t Modbus_Check_Response(uint8_t slave_addr, uint16_t start_reg, uint16_t *result, uint16_t timeout_ms);
uint8_t Modbus_Check_Response_FromBuffer(uint8_t slave_addr, uint16_t start_reg, uint16_t *result);
uint8_t Modbus_Read_Status(uint8_t slave_addr, uint16_t status_reg, uint16_t *status_value, uint16_t timeout_ms);
uint8_t Build_Modbus_Frame(uint8_t slave_addr, uint8_t func_code, uint16_t start_reg, 
                          uint8_t reg_num, uint16_t *data, uint8_t *frame);
uint8_t Control_Motors_Complete(MotorControlParams *motors, uint8_t count, uint8_t *results);
void battery_swap(void);
void BatterySwap_Process(void);
void Sync_Motors_Control(MotorControlParams *motors, uint8_t count);
void PollAndClearMotorAlarms_NonBlocking(void);//报警轮询
uint8_t Motor_Reset(uint8_t slave_addr, uint16_t reg_addr, uint16_t reset_value);

//继电器相关函数
void Relay_Init(void);
void Relay_Control(RelayState state);
void Relay_Forward(void);
void Relay_Backward(void);
void Relay_Stop(void);
void RELAY(void);

uint8_t Motor_CheckAndRecoverStall(uint8_t slave_addr, uint16_t current_threshold_ma);
void MotorStallMonitorTask(void);
void AlarmPoll_Init(void);

uint8_t ReadMotorPosition(uint8_t slave_addr, int32_t *pos); //读取电机实时位置
void StopMotors(uint8_t *addrs, uint8_t count); //停止多个电机
uint8_t WaitWithPositionCheck(uint8_t *addrs, uint8_t count, uint32_t timeout_ms, uint32_t check_interval_ms, uint8_t stall_threshold, int32_t *target_pos); //带位置监测的等待函数
#endif
