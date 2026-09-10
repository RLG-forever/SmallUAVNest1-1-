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
#ifndef CALIB_MOTOR1_FLY_LIFT_POS
#error "Missing CALIB_MOTOR1_FLY_LIFT_POS"
#endif
#ifndef CALIB_MOTOR1_TRANSFER_LIFT_POS
#error "Missing CALIB_MOTOR1_TRANSFER_LIFT_POS"
#endif
#ifndef CALIB_MOTOR1_LOAD_BATTERY_POS
#error "Missing CALIB_MOTOR1_LOAD_BATTERY_POS"
#endif
#ifndef CALIB_MOTOR2_FLY_NEAR_POS
#error "Missing CALIB_MOTOR2_FLY_NEAR_POS"
#endif
#ifndef CALIB_MOTOR2_AIRCRAFT_WORK_POS
#error "Missing CALIB_MOTOR2_AIRCRAFT_WORK_POS"
#endif
#ifndef CALIB_MOTOR2_BAY_WORK_POS
#error "Missing CALIB_MOTOR2_BAY_WORK_POS"
#endif
#ifndef CALIB_MOTOR2_BAY_RELEASE_POS
#error "Missing CALIB_MOTOR2_BAY_RELEASE_POS"
#endif
#ifndef CALIB_MOTOR2_BAY_LOAD_POS
#error "Missing CALIB_MOTOR2_BAY_LOAD_POS"
#endif
#ifndef CALIB_BATTERY_BAY1_POS
#error "Missing CALIB_BATTERY_BAY1_POS"
#endif
#ifndef CALIB_BATTERY_BAY2_POS
#error "Missing CALIB_BATTERY_BAY2_POS"
#endif
#ifndef CALIB_BATTERY_BAY3_POS
#error "Missing CALIB_BATTERY_BAY3_POS"
#endif
#ifndef CALIB_MOTOR2_AIRCRAFT_RELEASE_POS
#error "Missing CALIB_MOTOR2_AIRCRAFT_RELEASE_POS"
#endif
#ifndef CALIB_MOTOR2_AIRCRAFT_CLEAR_POS
#error "Missing CALIB_MOTOR2_AIRCRAFT_CLEAR_POS"
#endif
#ifndef CALIB_PLANE_TRANSFER_IN_MOTOR56_POS
#error "Missing CALIB_PLANE_TRANSFER_IN_MOTOR56_POS"
#endif
#ifndef CALIB_PLANE_TRANSFER_IN_MOTOR78_POS
#error "Missing CALIB_PLANE_TRANSFER_IN_MOTOR78_POS"
#endif
#ifndef CALIB_CENTER_REAR_POS
#error "Missing CALIB_CENTER_REAR_POS"
#endif
#ifndef CALIB_CENTER_SIDE_A_POS
#error "Missing CALIB_CENTER_SIDE_A_POS"
#endif
#ifndef CALIB_CENTER_SIDE_B_POS
#error "Missing CALIB_CENTER_SIDE_B_POS"
#endif
#endif

#endif
