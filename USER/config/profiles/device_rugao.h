#ifndef DEVICE_RUGAO_PROFILE_H
#define DEVICE_RUGAO_PROFILE_H

#define DEVICE_PROFILE_NAME                         "RuGao"

// 电机通用原点位置
#define MOTOR_HOME_POS                           0L
// 上下移电机 回收流程标定位置
#define MOTOR1_RECOVERY_POS                315000L
//上下移电机 开飞机电池 高度位置
#define MOTOR1_FLY_LIFT_POS                333000L
//上下移电机 取飞机电池 高度位置
#define MOTOR1_TRANSFER_LIFT_POS           333500L
//上下移电机 装飞机电池 高度位置，按新电池来源仓分别标定
#define MOTOR1_LOAD_BATTERY_BAY1_POS       333000L
#define MOTOR1_LOAD_BATTERY_BAY2_POS       333000L
#define MOTOR1_LOAD_BATTERY_BAY3_POS       333000L

// 横移电机 回收流程前进位置
#define MOTOR2_RECOVERY_FORWARD_POS        136000L

//横移电机 靠近飞机电池开关 位置
#define MOTOR2_FLY_NEAR_POS                292000L
//横移电机 打开飞机电池开关 前进的相对位置
#define MOTOR2_FLY_FAR_RELA_POS              8000L

//横移电机 取飞机电池 位置
#define MOTOR2_AIRCRAFT_WORK_POS           296000L
//横移电机装 飞机电池 释放位置
#define MOTOR2_AIRCRAFT_RELEASE_POS        297000L
//横移电机装 飞机电池 时推电池位置
#define MOTOR2_AIRCRAFT_CLEAR_POS          320000L

//横移电机移动到 电池仓  推进去电池位置
#define MOTOR2_BAY_RELEASE_POS             363000L
//横移电机移动到 电池仓  取电池位置
#define MOTOR2_BAY_LOAD_POS                358000L

//电池仓 释放电池 横向位置
#define BATTERY_BAY1_REALEASE_POS          337000L
#define BATTERY_BAY2_REALEASE_POS          336000L
#define BATTERY_BAY3_REALEASE_POS          336000L

//电池仓 推电池 横向位置
#define BATTERY_BAY1_PUSH_POS              363000L
#define BATTERY_BAY2_PUSH_POS              363000L
#define BATTERY_BAY3_PUSH_POS              367000L

//电池仓 装电池 高度位置
#define BATTERY_LOAD_BAY1_HEIGHT           134500L
#define BATTERY_LOAD_BAY2_HEIGHT            67000L
#define BATTERY_LOAD_BAY3_HEIGHT             1800L

//电池仓 取电池 高度位置
#define BATTERY_GET_BAY1_HEIGHT            135000L
#define BATTERY_GET_BAY2_HEIGHT             70000L
#define BATTERY_GET_BAY3_HEIGHT              3000L

//让飞机前进时电机5和6移动到的位置
#define UAV_FORWARD_MOTOR56_POS            177000L
//让飞机前进时电机7和8移动到的位置
#define UAV_FORWARD_MOTOR78_POS            180000L

// 让飞机退出时电机5和6移动到的位置
#define UAV_BACKWARD_MOTOR56_POS           207000L
// 让飞机退出时电机7和8移动到的位置
#define UAV_BACKWARD_MOTOR78_POS           150000L

// 飞机传送机构基础位置
#define PLANE_BASE_POS                     176000L
// 飞机传送机构偏移位置
#define PLANE_OFFSET_POS                   176384L

//电机5和6的center2夹紧位置
#define CENTER_FRONT_POS                    98000L
//电机7和8的center2夹紧位置
#define CENTER_REAR_POS                    257000L

//电机9和10的center1夹紧位置
#define CENTER_SIDE_A_POS                  92000L
//电机11和12的center1夹紧位置
#define CENTER_SIDE_B_POS                  99000L

#endif
