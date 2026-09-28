#ifndef GATEWAY_SERVICE_H
#define GATEWAY_SERVICE_H

#include <stdint.h>

/* 本机作为上游 Modbus 从站时使用的地址。 */
#define GATEWAY_SERVICE_MODBUS_ADDRESS 0x22U
/* 0x46 写入值：0=完整回零；1=5/6/9/10/11/12回零；2=7/8回退；
 * 3=横移电机回零；4=上下移电机回零；5=7/8最终回零。 */
#define GATEWAY_SERVICE_HOME_COMMAND_REG 0x0046U
#define GATEWAY_SERVICE_HOME_ALL_VALUE   0x0000U

/* 应用网关层：将从站寄存器请求映射到业务命令、动作序列和下游事务。 */
void GatewayService_Init(void);
void GatewayService_SetControlEnabled(uint8_t enabled);
void GatewayService_ScheduleRemotePowerOff(uint32_t delay_ms);
void GatewayService_CancelRemotePowerOff(void);
void GatewayService_Process(void);

#endif
