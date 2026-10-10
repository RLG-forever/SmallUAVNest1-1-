#ifndef FULL_HOME_SERVICE_H
#define FULL_HOME_SERVICE_H

#include <stdint.h>

/* 1=每次启动强制重置电机7、8的Flash历史位置；0=只按中断运动差值判断。 */
#ifndef STARTUP_RESET_MOTOR78_POSITION
#define STARTUP_RESET_MOTOR78_POSITION  0U
#endif

#if STARTUP_RESET_MOTOR78_POSITION != 0U && STARTUP_RESET_MOTOR78_POSITION != 1U
#error STARTUP_RESET_MOTOR78_POSITION must be 0U or 1U
#endif

/* 上次运动未正常结束且两台有效历史位置差值超过此值时，一起重置为0。 */
#ifndef STARTUP_MOTOR78_POSITION_DIFF_LIMIT
#define STARTUP_MOTOR78_POSITION_DIFF_LIMIT  10000L
#endif

#if STARTUP_MOTOR78_POSITION_DIFF_LIMIT < 0
#error STARTUP_MOTOR78_POSITION_DIFF_LIMIT must be nonnegative
#endif

/* 舱门启动流程完成后调用；返回 1 表示已启动，0 表示启动失败。 */
uint8_t FullHomeService_StartInitial(void);

/* 必须在 MotorControl_FullHomeProcess() 之后调用。 */
void FullHomeService_Process(void);

uint8_t FullHomeService_IsNormalOperationsEnabled(void);

#endif
