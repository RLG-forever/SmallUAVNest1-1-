#ifndef MODBUS_PORT_H
#define MODBUS_PORT_H

#include <stdint.h>

/* 板级 Modbus 默认波特率；主从端口共用该配置。 */
#define MODBUS_PORT_DEFAULT_BAUDRATE 9600U

/* RS485/USART 硬件传输层：不解析 Modbus 帧，也不包含具体业务。 */
/* 所有回调均在中断上下文执行，必须短小且不可阻塞。 */
typedef void (*ModbusPortRxByteCallback)(uint8_t byte);
typedef void (*ModbusPortFrameEndCallback)(void);
typedef void (*ModbusPortTxDoneCallback)(void);

/* 必须在开启全局中断前调用：主站使用USART1/TIM4，从站使用USART2/TIM3。 */
void ModbusPort_InitMaster(uint32_t baudrate);
void ModbusPort_InitSlave(uint32_t baudrate);
void ModbusPort_InitMasterFrameTimer(uint32_t baudrate);
void ModbusPort_InitSlaveFrameTimer(uint32_t baudrate);
uint32_t ModbusPort_FrameT35Us(uint32_t baudrate);

void ModbusPort_SetMasterCallbacks(ModbusPortRxByteCallback rx_callback,
                                   ModbusPortFrameEndCallback frame_end_callback,
                                   ModbusPortTxDoneCallback tx_done_callback);
void ModbusPort_SetSlaveCallbacks(ModbusPortRxByteCallback rx_callback,
                                  ModbusPortFrameEndCallback frame_end_callback);
/* 启动中断发送：返回0表示已接收，返回1表示端口忙或参数无效。 */
uint8_t ModbusPort_MasterSend(const uint8_t *data, uint16_t len);
uint8_t ModbusPort_SlaveSend(const uint8_t *data, uint16_t len);
void ModbusPort_MasterCancel(void);
void ModbusPort_MasterRestartFrameTimer(void);
void ModbusPort_MasterStopFrameTimer(void);
void ModbusPort_SlaveRestartFrameTimer(void);
void ModbusPort_SlaveStopFrameTimer(void);

#endif
