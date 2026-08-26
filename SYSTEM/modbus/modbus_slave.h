#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <stdint.h>

/*
 * 从站协议层向业务层提供的只读请求。
 * 业务层不接触接收缓冲区、CRC、接收计数或帧结束标志。
 */
typedef struct {
    uint8_t address;
    uint8_t function;
    uint16_t start_register;
    uint16_t register_count;
    uint16_t value;
    const uint8_t *write_data;
    uint8_t byte_count;
} ModbusSlaveRequest;

/* 返回 0 表示请求已经处理完成，返回非 0 表示需要在主循环中继续处理。 */
typedef uint8_t (*ModbusSlaveRequestHandler)(const ModbusSlaveRequest *request);

typedef struct {
    ModbusSlaveRequestHandler handle03;
    ModbusSlaveRequestHandler handle06;
    ModbusSlaveRequestHandler handle10;
} ModbusSlaveHandlers;

/* 初始化从站协议状态并注册端口回调，slave_address 的有效范围为 1～247。 */
void ModbusSlave_Init(uint8_t slave_address);
void ModbusSlave_RegisterHandlers(const ModbusSlaveHandlers *handlers);

/*
 * 启动异步应答发送：返回 0 表示端口已接收，非 0 表示参数错误或发送端口忙。
 * payload 不包含 CRC，协议层会自动追加 CRC16。
 */
uint8_t ModbusSlave_SendFrame(const uint8_t *payload, uint16_t payload_length);
uint8_t ModbusSlave_SendException(const ModbusSlaveRequest *request,
                                  uint8_t exception_code);
uint8_t ModbusSlave_SendWriteAck(const ModbusSlaveRequest *request);

/* 在主循环中周期调用，完成从站帧校验、解析、业务分派和应答。 */
void ModbusSlave_Process(void);

#endif
