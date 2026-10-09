#ifndef DEVICE_PROFILE_H
#define DEVICE_PROFILE_H

/* Select exactly one device profile in the build configuration. */
#if (defined(DEVICE_JURONG) + defined(DEVICE_LISHUI) + \
     defined(DEVICE_RUGAO)) != 1
#error "Define exactly one device profile"
#endif

#if defined(DEVICE_JURONG)
#include "profiles/device_jurong.h"
#elif defined(DEVICE_LISHUI)
#include "profiles/device_lishui.h"
#elif defined(DEVICE_RUGAO)
#include "profiles/device_rugao.h"
#endif

#ifdef DEVICE_PROFILE_INCOMPLETE
#error "The selected device profile has incomplete calibration values"
#else
/* Every profile must implement this calibration contract. */
#ifndef MOTOR_HOME_POS
#error "Missing MOTOR_HOME_POS"
#endif
#ifndef MOTOR1_RECOVERY_POS
#error "Missing MOTOR1_RECOVERY_POS"
#endif
#ifndef MOTOR1_FLY_LIFT_POS
#error "Missing MOTOR1_FLY_LIFT_POS"
#endif
#ifndef MOTOR1_TRANSFER_LIFT_POS
#error "Missing MOTOR1_TRANSFER_LIFT_POS"
#endif
#ifndef MOTOR1_LOAD_BATTERY_BAY1_POS
#error "Missing MOTOR1_LOAD_BATTERY_BAY1_POS"
#endif
#ifndef MOTOR1_LOAD_BATTERY_BAY2_POS
#error "Missing MOTOR1_LOAD_BATTERY_BAY2_POS"
#endif
#ifndef MOTOR1_LOAD_BATTERY_BAY3_POS
#error "Missing MOTOR1_LOAD_BATTERY_BAY3_POS"
#endif
#ifndef MOTOR2_RECOVERY_FORWARD_POS
#error "Missing MOTOR2_RECOVERY_FORWARD_POS"
#endif
#ifndef MOTOR2_FLY_NEAR_POS
#error "Missing MOTOR2_FLY_NEAR_POS"
#endif
#ifndef MOTOR2_FLY_FAR_RELA_POS
#error "Missing MOTOR2_FLY_FAR_RELA_POS"
#endif
#ifndef MOTOR2_AIRCRAFT_WORK_POS
#error "Missing MOTOR2_AIRCRAFT_WORK_POS"
#endif
#ifndef MOTOR2_BAY_RELEASE_POS
#error "Missing MOTOR2_BAY_RELEASE_POS"
#endif
#ifndef MOTOR2_BAY_LOAD_POS
#error "Missing MOTOR2_BAY_LOAD_POS"
#endif
#ifndef BATTERY_BAY1_PUSH_POS
#error "Missing BATTERY_BAY1_PUSH_POS"
#endif
#ifndef BATTERY_BAY2_PUSH_POS
#error "Missing BATTERY_BAY2_PUSH_POS"
#endif
#ifndef BATTERY_BAY3_PUSH_POS
#error "Missing BATTERY_BAY3_PUSH_POS"
#endif
#ifndef BATTERY_BAY1_REALEASE_POS
#error "Missing BATTERY_BAY1_REALEASE_POS"
#endif
#ifndef BATTERY_BAY2_REALEASE_POS
#error "Missing BATTERY_BAY2_REALEASE_POS"
#endif
#ifndef BATTERY_BAY3_REALEASE_POS
#error "Missing BATTERY_BAY3_REALEASE_POS"
#endif
#ifndef BATTERY_LOAD_BAY1_HEIGHT
#error "Missing BATTERY_LOAD_BAY1_HEIGHT"
#endif
#ifndef BATTERY_LOAD_BAY2_HEIGHT
#error "Missing BATTERY_LOAD_BAY2_HEIGHT"
#endif
#ifndef BATTERY_LOAD_BAY3_HEIGHT
#error "Missing BATTERY_LOAD_BAY3_HEIGHT"
#endif
#ifndef BATTERY_GET_BAY1_HEIGHT
#error "Missing BATTERY_GET_BAY1_HEIGHT"
#endif
#ifndef BATTERY_GET_BAY2_HEIGHT
#error "Missing BATTERY_GET_BAY2_HEIGHT"
#endif
#ifndef BATTERY_GET_BAY3_HEIGHT
#error "Missing BATTERY_GET_BAY3_HEIGHT"
#endif
#ifndef MOTOR2_AIRCRAFT_RELEASE_POS
#error "Missing MOTOR2_AIRCRAFT_RELEASE_POS"
#endif
#ifndef MOTOR2_AIRCRAFT_CLEAR_POS
#error "Missing MOTOR2_AIRCRAFT_CLEAR_POS"
#endif
#ifndef UAV_FORWARD_MOTOR56_POS
#error "Missing UAV_FORWARD_MOTOR56_POS"
#endif
#ifndef UAV_FORWARD_MOTOR78_POS
#error "Missing UAV_FORWARD_MOTOR78_POS"
#endif
#ifndef UAV_BACKWARD_MOTOR56_POS
#error "Missing UAV_BACKWARD_MOTOR56_POS"
#endif
#ifndef UAV_BACKWARD_MOTOR78_POS
#error "Missing UAV_BACKWARD_MOTOR78_POS"
#endif
#ifndef PLANE_BASE_POS
#error "Missing PLANE_BASE_POS"
#endif
#ifndef PLANE_OFFSET_POS
#error "Missing PLANE_OFFSET_POS"
#endif
#ifndef CENTER_FRONT_POS
#error "Missing CENTER_FRONT_POS"
#endif
#ifndef CENTER_REAR_POS
#error "Missing CENTER_REAR_POS"
#endif
#ifndef CENTER_SIDE_A_POS
#error "Missing CENTER_SIDE_A_POS"
#endif
#ifndef CENTER_SIDE_B_POS
#error "Missing CENTER_SIDE_B_POS"
#endif
#endif

#endif
