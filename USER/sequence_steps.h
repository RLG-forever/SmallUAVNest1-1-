#ifndef SEQUENCE_STEPS_H
#define SEQUENCE_STEPS_H

#include <stdint.h>
#include "status_regs.h"

typedef uint8_t (*StepFunc)(void);

#define STEP_RESULT_RETRY 0x80U

typedef struct {
    StepFunc run;
    uint32_t post_delay_ms;
    StatusRegAddr completion_reg;
    uint16_t completion_value;
} StepDef;


/* 电机设备地址、寄存器、命令值和动作预置参数。 */
// 1. 电机标识（区分不同电机）
#define MOTOR_ID_1       2       // 电机1的唯一标识
#define MOTOR_ID_2       3       // 电机2的唯一标识
#define MOTOR_ID_3       1       // 电机3的唯一标识
#define MOTOR_ID_4       4       // 电机4的唯一标识
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
#define MOTOR16_SLAVE_ADDR 0x0e   // 雨量地址
#define MOTOR17_SLAVE_ADDR 0x0f   // 风速地址
#define MOTOR14_SLAVE_ADDR 0x14   // 电机14从站地址(控制遥控器的舵机)
#define MOTOR15_SLAVE_ADDR 0x13   // 电机15从站地址(电池充电空开)
// 06功能码帧长度（固定8字节）
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
#define MOTOR4_CMD_OPEN                  0x0002  // 开舱
#define MOTOR4_CMD_CLOSE                 0x0001  // 关舱
#define MOTOR4_CMD_STOP                  0x0005  // 停止

//电机5-12
#define MOTOR5_CTRL_REG1      0x00ce  // 电机5-12控制寄存器地址（行程设置）

//空调

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
#define MOTOR_PRESET_PULSE_07            0xF6F0        //行程低位，远离电机-332000  ef20
#define MOTOR_PRESET_PULSE_08            0xfffa        //行程高位
#define MOTOR_PRESET_PULSE_09            0x10e0        //程低位，靠近电机332000
#define MOTOR_PRESET_PULSE_10           0x0006        //行程高位   5
#define MOTOR_PRESET_PULSE_11           0xecc0        //程低位，远离电机-136000
#define MOTOR_PRESET_PULSE_12           0xfffd        //行程高位
#define MOTOR_PRESET_PULSE_13           0x1340        //程低位，靠近电机136000
#define MOTOR_PRESET_PULSE_14           0x0002        //行程高位
#define MOTOR_PRESET_PULSE_15           0xf830        //程低位，远离电机-2000
#define MOTOR_PRESET_PULSE_16           0xffff        //行程高位
#define MOTOR_PRESET_PULSE_17           0x07d0        //程低位，靠近电机2000
#define MOTOR_PRESET_PULSE_18           0x0000        //行程高位
#define MOTOR_PRESET_PULSE_35           0xf278        //程低位，远离电机-69000
#define MOTOR_PRESET_PULSE_36           0xfffe        //行程高位
#define MOTOR_PRESET_PULSE_37           0x0d88        //程低位，靠近电机69000
#define MOTOR_PRESET_PULSE_38           0x0001        //行程高位
#define MOTOR_PRESET_PULSE_43             0xFAD8        //电机2端低位脉冲数-329000
#define MOTOR_PRESET_PULSE_44             0xfffa        //电机2端高位脉冲数






//电机2
//速度模式控制
#define  MOTOR2_DIRECTION_FORWARD             0x0000        //方向（高位：00向前，01向后；低位：00表示加速度档位为0)
#define  MOTOR2_DIRECTION_REVERSE        0x0100
#define  MOTOR2_SPEED_VALUE                 0x01f4        //速度（两字节）
#define  MOTOR2_SYNC_FLAG            0x0000        //多机同步标志（高位单字节0/1,低字节为00）
//立即停止
#define  MOTOR2_CMD_STOP           0x9800        //
//触发回零
#define  MOTOR2_CMD_RETURN_TO_ZERO      0x0300        //多圈限位回零
//使能信号控制
#define  MOTOR2_ENABLE_STATE              0xab01        //使能状态（00不使能/01使能）
//位置模式
#define MOTOR2_POSITION_MODE_RELATIVE              0x0000        //相对模式
#define MOTOR2_POSITION_MODE_ABSOLUTE              0x0001        //绝对模式
#define MOTOR_PRESET_PULSE_01             0x3188        //电机2端低位脉冲数-315000    8778
#define MOTOR_PRESET_PULSE_02             0xfffb        //电机2端高位脉冲数
#define MOTOR_PRESET_PULSE_03             0xa168        //电机2端低位脉冲数369000
#define MOTOR_PRESET_PULSE_04             0x0006        //电机2端高位脉冲数	  5
#define MOTOR_PRESET_PULSE_05             0xDCD8       //电机2端低位脉冲数-9000     eb89
#define MOTOR_PRESET_PULSE_06             0xffff        //电机2端高位脉冲数
#define MOTOR_PRESET_PULSE_19             0x7888        //电机2端低位脉冲数293000
#define MOTOR_PRESET_PULSE_20             0x0005        //电机2端高位脉冲数   5
#define MOTOR_PRESET_PULSE_21             0xA240        //电机2端低位脉冲数-15000    c568
#define MOTOR_PRESET_PULSE_22             0xffff        //电机2端高位脉冲数
#define MOTOR_PRESET_PULSE_23             0x3a98        //电机2端低位脉冲数15000
#define MOTOR_PRESET_PULSE_24             0x0000        //电机2端高位脉冲数
#define MOTOR_PRESET_PULSE_25             0xb320        //电机2端低位脉冲数308000
#define MOTOR_PRESET_PULSE_26             0x0005        //电机2端高位脉冲数   4
#define MOTOR_PRESET_PULSE_27             0x7DD8        //电机2端低位脉冲数-363000   7608
#define MOTOR_PRESET_PULSE_28             0xfffa        //电机2端高位脉冲数
#define MOTOR_PRESET_PULSE_29             0x89f8        //电机2端低位脉冲数363000
#define MOTOR_PRESET_PULSE_30             0x0006        //电机2端高位脉冲数   5
#define MOTOR_PRESET_PULSE_31             0xd8f0        //电机2端低位脉冲数-10000
#define MOTOR_PRESET_PULSE_32             0xffff        //电机2端高位脉冲数
#define MOTOR_PRESET_PULSE_33             0x2710        //电机2端低位脉冲数10000
#define MOTOR_PRESET_PULSE_34             0x0000        //电机2端高位脉冲数
#define MOTOR_PRESET_PULSE_39             0xd260        //电机2端低位脉冲数316000
#define MOTOR_PRESET_PULSE_40             0x0005        //电机2端高位脉冲数   4
#define MOTOR_PRESET_PULSE_41             0x9330        //电机2端低位脉冲数-290000
#define MOTOR_PRESET_PULSE_42             0xfffb        //电机2端高位脉冲数
#define MOTOR_PRESET_PULSE_45             0x8778        //电机2端低位脉冲数-295000  7FA8
#define MOTOR_PRESET_PULSE_46             0xfffb        //电机2端高位脉冲数
#define MOTOR_PRESET_PULSE_47             0x8778        //电机2端低位脉冲数-290000
#define MOTOR_PRESET_PULSE_48             0xfffb        //电机2端高位脉冲数


//电机3
#define MOTOR3_CMD_RELEASE                 0x0001         //松开
#define MOTOR3_CMD_CLAMP                  0x0001         //夹紧
#define MOTOR3_CMD_SAVE_CONFIG                   0x0001         //保存配置

//归中电机4~8公用寄存器地址
#define MOTOR4_12_REG_SPEED                  0x009a         //运行速度设置
#define MOTOR4_12_REG_TRAVEL                 0x00c8         //固定行程运动
#define MOTOR4_CMD_RUN_FORWARD             0x0001         //正转连续运动
#define MOTOR4_CMD_RUN_REVERSE           0x0257         //反转运动
#define MOTOR4_CMD_STOP_MOTION            0x0256         //停止运动
#define MOTOR4_SPEED_VALUE           0x01f4         //运动速度
//#define MOTOR5_TRAVEL_LOW_WORD        0xbf20         //行程低位，靠近电机180000
//#define MOTOR5_TRAVEL_HIGH_WORD        0x0002         //行程高位，靠近电机
#define MOTOR5_TRAVEL_LOW_WORD        0xaf80         //行程低位，靠近电机176000
#define MOTOR5_TRAVEL_HIGH_WORD        0x0002         //行程高位，靠近电机
//#define MOTOR6_TRAVEL_LOW_WORD        0x40e0         //行程低位，远离电机-180000
//#define MOTOR6_TRAVEL_HIGH_WORD        0xfffd         //行程高位，远离电机
#define MOTOR6_TRAVEL_LOW_WORD        0x4F00         //行程低位，远离电机-176000
#define MOTOR6_TRAVEL_HIGH_WORD        0xfffd         //行程高位，远离电机
#define MOTOR7_TRAVEL_LOW_WORD        0x5d78         //行程低位，靠近电机155000
#define MOTOR7_TRAVEL_HIGH_WORD        0x0002         //行程高位，靠近电机
#define MOTOR8_TRAVEL_LOW_WORD        0xa8d0        //行程低位，远离电机-350000
#define MOTOR8_TRAVEL_HIGH_WORD        0xfffa         //行程高位，远离电机
#define MOTOR9_TRAVEL_LOW_WORD        0x5f90         //行程低位，靠近电机90000
#define MOTOR9_TRAVEL_HIGH_WORD        0x0001         //行程高位，靠近电机
#define MOTOR10_TRAVEL_LOW_WORD        0xa070        //行程低位，远离电机-90000
#define MOTOR10_TRAVEL_HIGH_WORD        0xfffe         //行程高位，远离电机
#define MOTOR11_TRAVEL_LOW_WORD        0x82b8         //行程低位，靠近电机99000
#define MOTOR11_TRAVEL_HIGH_WORD        0x0001         //行程高位，靠近电机
#define MOTOR12_TRAVEL_LOW_WORD        0x7d48        //行程低位，远离电机-99000
#define MOTOR12_TRAVEL_HIGH_WORD        0xfffe         //行程高位，远离电机
#define MOTOR13_TRAVEL_LOW_WORD        0x3450         //行程低位，靠近电机210000
#define MOTOR13_TRAVEL_HIGH_WORD        0x0003         //行程高位，靠近电机
#define MOTOR14_TRAVEL_LOW_WORD        0xe4a8        //行程低位，远离电机-7000
#define MOTOR14_TRAVEL_HIGH_WORD        0xffff         //行程高位，远离电机
#define MOTOR15_TRAVEL_LOW_WORD        0x0d40         //行程低位，靠近电机200000
#define MOTOR15_TRAVEL_HIGH_WORD        0x0003         //行程高位，靠近电机
#define MOTOR16_TRAVEL_LOW_WORD        0xf2c0        //行程低位，远离电机-200000
#define MOTOR16_TRAVEL_HIGH_WORD        0xfffc         //行程高位，远离电机
#define MOTOR17_TRAVEL_LOW_WORD        0x5f00         //行程低位，靠近电机352000
#define MOTOR17_TRAVEL_HIGH_WORD        0x0005         //行程高位，靠近电机

//电机12
#define MOTOR12_UP        0x0033  // 上升
#define MOTOR12_DOWN      0x0044  // 下降
#define MOTOR12_STOP      0x0055  // 停止



/* 非电机设备的 Modbus 地址和控制参数。 */
#define UAV_CONTROLLER_SLAVE          0x14U
#define UAV_POWER_CTRL_REG            0x0001U
#define UAV_POWER_ON_VALUE            0x0001U
#define UAV_POWER_MODE2_VALUE         0x0002U

#define CHARGER_SLAVE                 0x13U
#define CHARGER_POWER_CTRL_REG        0x0000U
#define CHARGER_POWER_ON_VALUE        0x0001U
#define CHARGER_POWER_OFF_VALUE       0x0000U

#define AIR_CONDITIONER_SLAVE         0x0DU
#define AIR_POWER_CTRL_REG            0x002FU
#define AIR_POWER_ON_VALUE            0x0001U
#define AIR_POWER_OFF_VALUE           0x0000U
#define AIR_COOLING_STOP_TEMP_REG     0x0000U
#define AIR_HEATING_STOP_TEMP_REG     0x0002U

uint8_t Motor1Up1(void);
uint8_t Motor2Forward(void);
uint8_t Motor3Clamp(void);
uint8_t Motor2Back(void);
uint8_t Motor1Down1(void);
uint8_t Motor3Lossen(void);
uint8_t Motor1Up2(void);
uint8_t Motor1Up3(void);
uint8_t CloseCenter1(void);
uint8_t CloseCenter2(void);
uint8_t FlyForward(void);
uint8_t FlyBack(void);
uint8_t RelayCtrl(void);

uint8_t Center_1(void);
uint8_t Center_2(void);
uint8_t LeaveCenter(void);
uint8_t LeaveCenter1(void);
uint8_t LeaveCenter2(void);

uint8_t Battery_1(void);
uint8_t Battery_2(void);
uint8_t Battery_3(void);
uint8_t Battery_4(void);
uint8_t Battery_5(void);
uint8_t Battery_6(void);
uint8_t Battery_7(void);
uint8_t Battery_8(void);
uint8_t Battery_9(void);
uint8_t Battery_10(void);
uint8_t Battery_11(void);
uint8_t Battery_12(void);
uint8_t Battery_13(void);
uint8_t Battery_14(void);
uint8_t Battery_15(void);
uint8_t Battery_16(void);
uint8_t Battery_17(void);
uint8_t Battery_18(void);
uint8_t Battery_19(void);
uint8_t Battery_20(void);
uint8_t Battery_21(void);
uint8_t Battery_22(void);
uint8_t Battery_23(void);
uint8_t Battery_24(void);
uint8_t Battery_25(void);
uint8_t Battery_26(void);
uint8_t Battery_27(void);
uint8_t Battery_28(void);
uint8_t Battery_29(void);

uint8_t OpenDr(void);
uint8_t CloseDr(void);
uint8_t StopDr(void);
uint8_t CheckAndCloseDoor(void);
uint8_t OpenAC(void);
uint8_t CloseAC(void);

/* Read-only sequence step tables. */
extern const StepDef opendr1_steps[];
extern const StepDef opendr_steps[];
extern const StepDef closedr_steps[];
extern const StepDef openfly_steps[];
extern const StepDef closefly_steps[];
extern const StepDef takeoff_steps_1[];
extern const StepDef takeoff_steps_2[];
extern const StepDef takeoff_steps_3[];
extern const StepDef landing_steps_1[];
extern const StepDef landing_steps_2[];
extern const StepDef landing_steps_3[];
extern const StepDef closecenter_steps[];
extern const StepDef leavecenter_steps[];
extern const StepDef loadbattery_steps_1[];
extern const StepDef loadbattery_steps_2[];
extern const StepDef loadbattery_steps_3[];
extern const StepDef downbattery_steps_1[];
extern const StepDef downbattery_steps_2[];
extern const StepDef downbattery_steps_3[];
extern const StepDef recovery_with_battery_steps[];
extern const StepDef recovery_without_battery_steps[];

extern const uint8_t OPENDR1_STEP_COUNT;
extern const uint8_t OPENDR_STEP_COUNT;
extern const uint8_t CLOSEDR_STEP_COUNT;
extern const uint8_t OPENFLY_STEP_COUNT;
extern const uint8_t CLOSEFLY_STEP_COUNT;
extern const uint8_t TAKEOFF_STEPS_1_COUNT;
extern const uint8_t TAKEOFF_STEPS_2_COUNT;
extern const uint8_t TAKEOFF_STEPS_3_COUNT;
extern const uint8_t LANDING_STEPS_1_COUNT;
extern const uint8_t LANDING_STEPS_2_COUNT;
extern const uint8_t LANDING_STEPS_3_COUNT;
extern const uint8_t CLOSECENTER_STEP_COUNT;
extern const uint8_t LEAVECENTER_STEP_COUNT;
extern const uint8_t LOADBATTERY_STEPS_1_COUNT;
extern const uint8_t LOADBATTERY_STEPS_2_COUNT;
extern const uint8_t LOADBATTERY_STEPS_3_COUNT;
extern const uint8_t DOWNBATTERY_STEPS_1_COUNT;
extern const uint8_t DOWNBATTERY_STEPS_2_COUNT;
extern const uint8_t DOWNBATTERY_STEPS_3_COUNT;
extern const uint8_t RECOVERY_WITH_BATTERY_COUNT;
extern const uint8_t RECOVERY_WITHOUT_BATTERY_COUNT;

const uint8_t *SequenceSteps_GetMotorList(uint8_t sequence_id,
                                          uint8_t step_index);

#endif
