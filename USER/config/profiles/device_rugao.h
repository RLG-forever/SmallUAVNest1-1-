#ifndef DEVICE_RUGAO_PROFILE_H
#define DEVICE_RUGAO_PROFILE_H

#define DEVICE_PROFILE_NAME                         "RuGao"

// 电机通用原点位置
#define MOTOR_HOME_POS                           0L
// 上下移电机 回收流程标定位置
#define CALIB_MOTOR1_RECOVERY_POS                315000L
// 上下移电机 辅助标定位置
#define CALIB_MOTOR1_AUX_POS                     9000L

//上下移电机 开飞机电池 高度位置
#define CALIB_MOTOR1_FLY_LIFT_POS                334000L
//上下移电机 取飞机电池 高度位置
#define CALIB_MOTOR1_TRANSFER_LIFT_POS           334500L
//上下移电机 装飞机电池 高度位置
#define CALIB_MOTOR1_LOAD_BATTERY_POS            333000L

// 横移电机 回收流程前进位置
#define CALIB_MOTOR2_RECOVERY_FORWARD_POS        136000L

//横移电机 靠近飞机电池开关 位置
#define CALIB_MOTOR2_FLY_NEAR_POS                289000L
//横移电机 打开飞机电池开关 前进的相对位置
#define MOTOR2_FLY_FAR_RELA_POS                    8000L

//横移电机 取飞机电池 位置
#define CALIB_MOTOR2_AIRCRAFT_WORK_POS           287000L

//横移电机移动到 电池仓  装电池位置
#define CALIB_MOTOR2_BAY_WORK_POS                356000L
//横移电机移动到 电池仓  推进去电池位置
#define CALIB_MOTOR2_BAY_RELEASE_POS             367000L
//横移电机移动到 电池仓  取电池位置
#define CALIB_MOTOR2_BAY_LOAD_POS                360000L

//电池1号仓位置
#define CALIB_BATTERY_BAY1_POS                   136000L
//电池2号仓位置
#define CALIB_BATTERY_BAY2_POS                   68000L
//电池3号仓位置
#define CALIB_BATTERY_BAY3_POS                   2500L

//横移电机装 飞机电池 释放位置
#define CALIB_MOTOR2_AIRCRAFT_RELEASE_POS        295000L
//横移电机装 飞机电池 时推电池位置
#define CALIB_MOTOR2_AIRCRAFT_CLEAR_POS          302000L

//让飞机前进时电机5和6移动到的位置
#define CALIB_PLANE_TRANSFER_IN_MOTOR56_POS      181000L
//让飞机前进时电机7和8移动到的位置
#define CALIB_PLANE_TRANSFER_IN_MOTOR78_POS      176000L

// 让飞机退出时电机5和6移动到的位置
#define CALIB_PLANE_TRANSFER_OUT_MOTOR56_POS     207000L
// 让飞机退出时电机7和8移动到的位置
#define CALIB_PLANE_TRANSFER_OUT_MOTOR78_POS     150000L

// 飞机传送机构基础位置
#define CALIB_PLANE_BASE_POS                     176000L
// 飞机传送机构偏移位置
#define CALIB_PLANE_OFFSET_POS                   176384L

//电机5和6的center2夹紧位置
#define CALIB_CENTER_FRONT_POS                   7000L
//电机7和8的center2夹紧位置
#define CALIB_CENTER_REAR_POS                    347000L

//电机9和10的center1夹紧位置
#define CALIB_CENTER_SIDE_A_POS                  91500L
//电机11和12的center1夹紧位置
#define CALIB_CENTER_SIDE_B_POS                  97500L

#endif
