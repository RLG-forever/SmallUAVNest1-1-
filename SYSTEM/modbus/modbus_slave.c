#include "modbus_slave.h"

#include "modbus_common.h"
#include "modbus_port.h"
#include "stm32f4xx.h"

#include <stddef.h>
#include <string.h>

/* 接收状态只属于从站协议层，不向业务层暴露。 */
static uint8_t slave_rx_buffer[MODBUS_RTU_MAX_ADU_LENGTH];
static volatile uint16_t slave_rx_count;
static volatile uint8_t slave_frame_ready;
static uint8_t slave_address;
static ModbusSlaveHandlers slave_handlers;

static void ModbusSlave_OnFrameEnd(void)
{
    if (slave_rx_count > 0U) {
        slave_frame_ready = 1U;
    }
}

/* USART 中断只收集字节；帧解析和业务处理全部留在主循环。 */
static void ModbusSlave_OnRxByte(uint8_t byte)
{
    if (slave_frame_ready) {
        return;
    }
    if (slave_rx_count < sizeof(slave_rx_buffer)) {
        slave_rx_buffer[slave_rx_count++] = byte;
    } else {
        /* 溢出后丢弃当前帧，等待下一次帧间隔重新同步。 */
        slave_rx_count = 0U;
    }
    ModbusPort_SlaveRestartFrameTimer();
}

/* 重置中断与主循环共享的接收状态，并保持调用前的中断屏蔽状态。 */
static void ModbusSlave_ResetFrame(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    slave_rx_count = 0U;
    slave_frame_ready = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

void ModbusSlave_Init(uint8_t address)
{
    slave_address = (address >= 1U && address <= 247U) ? address : 1U;
    memset(slave_rx_buffer, 0, sizeof(slave_rx_buffer));
    memset(&slave_handlers, 0, sizeof(slave_handlers));
    slave_rx_count = 0U;
    slave_frame_ready = 0U;
    ModbusPort_SetSlaveCallbacks(ModbusSlave_OnRxByte, ModbusSlave_OnFrameEnd);
}

void ModbusSlave_RegisterHandlers(const ModbusSlaveHandlers *handlers)
{
    if (handlers == NULL) {
        memset(&slave_handlers, 0, sizeof(slave_handlers));
    } else {
        slave_handlers = *handlers;
    }
}

uint8_t ModbusSlave_SendFrame(const uint8_t *payload, uint16_t payload_length)
{
    uint8_t tx_buffer[MODBUS_RTU_MAX_ADU_LENGTH];
    uint16_t crc;

    if (payload == NULL || payload_length == 0U ||
        payload_length + 2U > sizeof(tx_buffer)) {
        return 1U;
    }

    memcpy(tx_buffer, payload, payload_length);
    crc = Modbus_CRC16(tx_buffer, payload_length);
    tx_buffer[payload_length] = (uint8_t)crc;
    tx_buffer[payload_length + 1U] = (uint8_t)(crc >> 8);
    return ModbusPort_SlaveSend(tx_buffer, payload_length + 2U);
}

uint8_t ModbusSlave_SendException(const ModbusSlaveRequest *request,
                                  uint8_t exception_code)
{
    uint8_t response[3];

    if (request == NULL) {
        return 1U;
    }
    response[0] = request->address;
    response[1] = (uint8_t)(request->function | 0x80U);
    response[2] = exception_code;
    return ModbusSlave_SendFrame(response, sizeof(response));
}

uint8_t ModbusSlave_SendWriteAck(const ModbusSlaveRequest *request)
{
    uint8_t response[6];

    if (request == NULL ||
        (request->function != 0x06U && request->function != 0x10U)) {
        return 1U;
    }

    response[0] = request->address;
    response[1] = request->function;
    Modbus_PutU16BE(&response[2], request->start_register);
    if (request->function == 0x06U) {
        Modbus_PutU16BE(&response[4], request->value);
    } else {
        Modbus_PutU16BE(&response[4], request->register_count);
    }
    return ModbusSlave_SendFrame(response, sizeof(response));
}

static uint8_t ModbusSlave_ParseRequest(ModbusSlaveRequest *request,
                                        uint16_t frame_length)
{
    uint16_t register_count;
    uint8_t byte_count;

    memset(request, 0, sizeof(*request));
    request->address = slave_rx_buffer[0];
    request->function = slave_rx_buffer[1];

    if (request->function == 0x03U || request->function == 0x06U) {
        if (frame_length != 8U) {
            return 0x03U;
        }
        request->start_register = Modbus_GetU16BE(&slave_rx_buffer[2]);
        if (request->function == 0x03U) {
            request->register_count = Modbus_GetU16BE(&slave_rx_buffer[4]);
        } else {
            request->register_count = 1U;
            request->value = Modbus_GetU16BE(&slave_rx_buffer[4]);
        }
        return 0U;
    }

    if (request->function == 0x10U) {
        if (frame_length < 9U) {
            return 0x03U;
        }
        register_count = Modbus_GetU16BE(&slave_rx_buffer[4]);
        byte_count = slave_rx_buffer[6];
        if (register_count == 0U || register_count > 123U ||
            byte_count != (uint8_t)(register_count * 2U) ||
            frame_length != (uint16_t)(9U + byte_count)) {
            return 0x03U;
        }
        request->start_register = Modbus_GetU16BE(&slave_rx_buffer[2]);
        request->register_count = register_count;
        request->write_data = &slave_rx_buffer[7];
        request->byte_count = byte_count;
    }
    return 0U;
}

void ModbusSlave_Process(void)
{
    ModbusSlaveRequest request;
    ModbusSlaveRequestHandler handler = NULL;
    uint16_t frame_length;
    uint8_t exception_code;
    uint8_t pending = 0U;

    if (!slave_frame_ready) {
        return;
    }

    frame_length = slave_rx_count;
    if (frame_length < 4U ||
        Modbus_CRC16(slave_rx_buffer, frame_length) != 0U) {
        ModbusSlave_ResetFrame();
        return;
    }
    if (slave_rx_buffer[0] != slave_address) {
        ModbusSlave_ResetFrame();
        return;
    }

    exception_code = ModbusSlave_ParseRequest(&request, frame_length);
    if (exception_code != 0U) {
        if (ModbusSlave_SendException(&request, exception_code) != 0U) {
            return;
        }
        ModbusSlave_ResetFrame();
        return;
    }

    if (request.function == 0x03U) {
        handler = slave_handlers.handle03;
    } else if (request.function == 0x06U) {
        handler = slave_handlers.handle06;
    } else if (request.function == 0x10U) {
        handler = slave_handlers.handle10;
    }

    if (handler != NULL) {
        pending = handler(&request);
    } else if (ModbusSlave_SendException(&request, 0x01U) != 0U) {
        pending = 1U;
    }

    if (!pending) {
        ModbusSlave_ResetFrame();
    }
}
