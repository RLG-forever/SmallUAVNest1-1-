#ifndef GATEWAY_TASKS_H
#define GATEWAY_TASKS_H

#include <stdint.h>

/* 预约延时执行的遥控器电源命令。 */
void GatewayTasks_ScheduleRemotePowerOff(uint32_t delay_ms);
void GatewayTasks_CancelRemotePowerOff(void);
void GatewayTasks_Process(void);

#endif
