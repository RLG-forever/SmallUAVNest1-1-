#ifndef GATEWAY_MODBUS_H
#define GATEWAY_MODBUS_H

#include <stdint.h>

/* 本机作为上游 Modbus 从站时使用的地址。 */
#define GATEWAY_MODBUS_SLAVE_ADDRESS 0x22U

/* 应用网关层：将从站寄存器请求映射到项目命令、动作序列和下游事务。 */
void GatewayModbus_Init(void);

#endif
