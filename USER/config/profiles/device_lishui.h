#ifndef DEVICE_LISHUI_PROFILE_H
#define DEVICE_LISHUI_PROFILE_H

#define DEVICE_PROFILE_NAME                         "LiShui"

// 电机通用原点位置
#define MOTOR_HOME_POS                              0L
// 上下移电机 回收流程标定位置
#define CALIB_MOTOR1_RECOVERY_POS                   315000L
// 上下移电机 辅助标定位置
#define CALIB_MOTOR1_AUX_POS                        9000L

// 上下移电机 开飞机电池高度位置
#define CALIB_MOTOR1_FLY_LIFT_POS                  333500L
// 上下移电机 取飞机电池高度位置
#define CALIB_MOTOR1_TRANSFER_LIFT_POS             332000L
// 上下移电机 装飞机电池高度位置
#define CALIB_MOTOR1_LOAD_BATTERY_POS              332000L

// 横移电机 回收流程前进位置
#define CALIB_MOTOR2_RECOVERY_FORWARD_POS           136000L

// 横移电机 靠近飞机电池开关位置
#define CALIB_MOTOR2_FLY_NEAR_POS                  293000L
// 横移电机 打开飞机电池开关的相对前进位置
#define MOTOR2_FLY_FAR_RELA_POS                       8000L

// 横移电机 取飞机电池位置
#define CALIB_MOTOR2_AIRCRAFT_WORK_POS             304000L
// 横移电机 电池仓装电池位置
#define CALIB_MOTOR2_BAY_WORK_POS                  355000L
// 横移电机 电池仓推进电池位置
#define CALIB_MOTOR2_BAY_RELEASE_POS               371000L
// 横移电机 电池仓取电池位置
#define CALIB_MOTOR2_BAY_LOAD_POS                  361000L

// 电池1号仓位置
#define CALIB_BATTERY_BAY1_POS                     137000L
// 电池2号仓位置
#define CALIB_BATTERY_BAY2_POS                      71000L
// 电池3号仓位置
#define CALIB_BATTERY_BAY3_POS                       2000L

// 电池1号仓取电池位置
#define CALIB_BATTERY_GET_BAY1_POS                  137000L
// 电池2号仓取电池位置
#define CALIB_BATTERY_GET_BAY2_POS                   71000L
// 电池3号仓取电池位置
#define CALIB_BATTERY_GET_BAY3_POS                    2000L

// 横移电机 装飞机电池释放位置
#define CALIB_MOTOR2_AIRCRAFT_RELEASE_POS          288000L
// 横移电机 装飞机电池时推电池位置
#define CALIB_MOTOR2_AIRCRAFT_CLEAR_POS            314000L

// 让飞机进入时电机5和6移动到的位置
#define CALIB_PLANE_TRANSFER_IN_MOTOR56_POS        185000L
// 让飞机进入时电机7和8移动到的位置
#define CALIB_PLANE_TRANSFER_IN_MOTOR78_POS        170000L

// 让飞机退出时电机5和6移动到的位置
#define CALIB_PLANE_TRANSFER_OUT_MOTOR56_POS       207000L
// 让飞机退出时电机7和8移动到的位置
#define CALIB_PLANE_TRANSFER_OUT_MOTOR78_POS       149000L
// 飞机传送机构基础位置
#define CALIB_PLANE_BASE_POS                       176000L
// 飞机传送机构偏移位置
#define CALIB_PLANE_OFFSET_POS                     176384L

// 电机5和6的center2夹紧位置
#define CALIB_CENTER_FRONT_POS                       7000L
// 电机7和8的center2夹紧位置
#define CALIB_CENTER_REAR_POS                      347000L
// 电机9和10的center1夹紧位置
#define CALIB_CENTER_SIDE_A_POS                     89000L
// 电机11和12的center1夹紧位置
#define CALIB_CENTER_SIDE_B_POS                    101000L

#endif
