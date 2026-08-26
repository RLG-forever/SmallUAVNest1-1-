#include "modbus_common.h"
#include <stddef.h>

/* CRC与字节序工具不依赖硬件，也不保存任何通信状态。 */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    if (data == NULL && len != 0U) {
        return 0U;
    }

    while (len-- > 0U) {
        crc ^= *data++;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x0001U) != 0U
                ? (uint16_t)((crc >> 1) ^ 0xA001U)
                : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

uint16_t Modbus_GetU16BE(const uint8_t *data)
{
    if (data == NULL) {
        return 0U;
    }

    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

void Modbus_PutU16BE(uint8_t *data, uint16_t value)
{
    if (data == NULL) {
        return;
    }

    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}
