#ifndef GATEWAY_SERVICE_H
#define GATEWAY_SERVICE_H

#include <stdint.h>

/* 本机作为上游 Modbus 从站时使用的地址。 */
#define GATEWAY_SERVICE_MODBUS_ADDRESS 0x22U

/* 应用网关层：将从站寄存器请求映射到业务命令、动作序列和下游事务。 */
void GatewayService_Init(void);
void GatewayService_ScheduleRemotePowerOff(uint32_t delay_ms);
void GatewayService_CancelRemotePowerOff(void);
void GatewayService_Process(void);

#endif
