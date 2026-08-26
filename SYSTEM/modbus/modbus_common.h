#ifndef MODBUS_COMMON_H
#define MODBUS_COMMON_H

#include <stdint.h>

/* Modbus RTU 地址域、PDU 和 CRC 合计最多 256 字节。 */
#define MODBUS_RTU_MAX_ADU_LENGTH 256U

/* 主从站共用的纯协议工具：不得依赖USART、定时器、GPIO或业务代码。 */
typedef enum {
    MODBUS_RESULT_OK = 0U,        /* 操作完成且结果正确。 */
    MODBUS_RESULT_PENDING = 1U,   /* 异步操作尚未完成。 */
    MODBUS_RESULT_TIMEOUT = 2U,   /* 等待应答超时。 */
    MODBUS_RESULT_LENGTH = 3U,    /* 报文长度不符合协议要求。 */
    MODBUS_RESULT_HEADER = 4U,    /* 从站地址或功能码不匹配。 */
    MODBUS_RESULT_ECHO = 5U,      /* 写操作应答内容与请求不一致。 */
    MODBUS_RESULT_CRC = 6U,       /* CRC 校验失败。 */
    MODBUS_RESULT_EXCEPTION = 7U, /* 收到 Modbus 异常应答。 */
    MODBUS_RESULT_PARAM = 8U,     /* 调用参数非法。 */
    MODBUS_RESULT_BUSY = 9U       /* 通信通道正忙。 */
} ModbusResult;

/* 返回Modbus RTU使用的CRC值，发送时先发送低字节。 */
/*
 * data 为 NULL 且 len 非零时返回 0U，避免解引用空指针。
 * data 为 NULL 且 len 为零时按空报文计算，返回初始 CRC 值 0xFFFFU。
 */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len);
/* 按Modbus载荷的大端字节序读写一个16位寄存器。 */
/* data 必须指向至少两个可读字节；传入 NULL 时返回 0U。 */
uint16_t Modbus_GetU16BE(const uint8_t *data);
/* data 必须指向至少两个可写字节；传入 NULL 时不执行写入。 */
void Modbus_PutU16BE(uint8_t *data, uint16_t value);

#endif
