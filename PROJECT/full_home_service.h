#ifndef FULL_HOME_SERVICE_H
#define FULL_HOME_SERVICE_H

#include <stdint.h>

/* 舱门启动流程完成后调用；返回 1 表示已启动，0 表示启动失败。 */
uint8_t FullHomeService_StartInitial(void);

/* 必须在 MotorControl_FullHomeProcess() 之后调用。 */
void FullHomeService_Process(void);

uint8_t FullHomeService_IsNormalOperationsEnabled(void);

#endif
