#include "modbus_master.h"
#include "modbus_port.h"
#include "modbus_common.h"
#include "stm32f4xx.h"
#include "tick.h"
#include <string.h>

#define MODBUS_MASTER_MAX_RETRIES  3U
#define MODBUS_MASTER_RETRY_GAP_MS 5U
#define MODBUS_MASTER_SEND_TIMEOUT_MS 100U
#define MODBUS_MASTER_TIMEOUT_MS   500U

/* 主站内部状态不对外暴露，调用方通过 ModbusMaster_IsBusy() 查询。 */
typedef enum {
    MASTER_IDLE,
    MASTER_SENDING,
    MASTER_WAIT_RESP,
    MASTER_RESP_OK,
    MASTER_RESP_ERR
} ModbusMasterState;

#define MASTER_RETRY_WAIT MASTER_WAIT_RESP

/* 一个RTU主站同一时刻只能有一笔在途事务。03/06/10接口返回PENDING后，
 * 调用方必须以相同参数重复调用，直到取得最终结果。 */
typedef enum {
    MASTER_TXN_FREE = 0,
    MASTER_TXN_SENDING,    /* 请求帧正在发送，等待发送完成。 */
    MASTER_TXN_WAITING,    /* 请求已发送，等待从站响应。 */
    MASTER_TXN_RETRY_WAIT, /* 本次尝试失败，等待重试间隔结束。 */
    MASTER_TXN_RESULT      /* 事务已结束，等待调用方读取结果。 */
} ModbusTxnPhase;

typedef struct {
    ModbusTxnPhase phase;
    ModbusMasterClient client;
    uint8_t slave;
    uint8_t function;
    uint16_t start_reg;
    uint16_t reg_num;
    uint16_t write_data[123];
    uint16_t read_data[125];
    uint8_t frame[256];
    uint16_t frame_len;
    uint8_t attempts;
    uint8_t result;
    uint32_t deadline;
} ModbusMasterTransaction;

static ModbusMasterState master_state = MASTER_IDLE;
static volatile uint16_t Master_RX_CNT;   /* 当前接收帧已接收的字节数。 */
static volatile uint8_t Master_FrameFlag; /* 帧间隔结束后置 1，表示一帧数据接收完成。 */
static uint8_t Master_RX_BUFF[MODBUS_RTU_MAX_ADU_LENGTH];

static ModbusMasterTransaction master_transaction;
static volatile uint8_t master_tx_done;

static void ModbusMaster_OnRxByte(uint8_t data)
{
    if (master_state == MASTER_WAIT_RESP && Master_FrameFlag == 0U &&
        Master_RX_CNT < sizeof(Master_RX_BUFF)) {
        Master_RX_BUFF[Master_RX_CNT++] = data;
    }
}

static void ModbusMaster_OnFrameEnd(void)
{
    if (Master_RX_CNT > 0U && Master_FrameFlag == 0U) {
        Master_FrameFlag = 1U;
    }
}

static void ModbusMaster_OnTxDone(void)
{
    master_tx_done = 1U;
    if (master_state == MASTER_SENDING) {
        master_state = MASTER_WAIT_RESP;
    }
}

static void ModbusMaster_ResetRx(void)
{
    uint32_t primask;

    ModbusPort_MasterStopFrameTimer();
    primask = __get_PRIMASK();
    __disable_irq();
    Master_RX_CNT = 0U;
    Master_FrameFlag = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

/* 检查调用参数是否与当前事务完全一致，用于继续查询同一笔事务的状态或结果。 */
static uint8_t ModbusMaster_RequestMatches(ModbusMasterClient client,
                                           uint8_t slave, uint8_t function,
                                           uint16_t start_reg, uint16_t reg_num,
                                           const uint16_t *write_data)
{
    /* 先比较事务的基本参数，避免不同请求共用当前事务。 */
    if (master_transaction.client != client ||
        master_transaction.slave != slave || master_transaction.function != function ||
        master_transaction.start_reg != start_reg || master_transaction.reg_num != reg_num) {
        return 0U;
    }
    /* 06 号功能码只有一个写寄存器值，需要单独校验。 */
    if (function == 0x06U && master_transaction.write_data[0] != write_data[0]) {
        return 0U;
    }
    /* 10 号功能码写入多个寄存器，逐字节比较完整的写入数据。 */
    if (function == 0x10U &&
        memcmp(master_transaction.write_data, write_data,
               reg_num * sizeof(uint16_t)) != 0) {
        return 0U;
    }
    return 1U;
}

/* 按当前事务参数组装 Modbus RTU 请求帧，并在帧尾附加 CRC16。 */
static void ModbusMaster_BuildFrame(void)
{
    uint16_t crc;

    /* RTU 帧固定以从站地址、功能码和起始寄存器开始。 */
    master_transaction.frame[0] = master_transaction.slave;
    master_transaction.frame[1] = master_transaction.function;
    Modbus_PutU16BE(&master_transaction.frame[2], master_transaction.start_reg);

    if (master_transaction.function == 0x06U) {
        /* 06：写单个寄存器，数据区只有一个寄存器值。 */
        Modbus_PutU16BE(&master_transaction.frame[4], master_transaction.write_data[0]);
        master_transaction.frame_len = 8U;
    } else {
        /* 03/10：数据区先放寄存器数量。 */
        Modbus_PutU16BE(&master_transaction.frame[4], master_transaction.reg_num);
        if (master_transaction.function == 0x10U) {
            /* 10：追加字节数和待写入的寄存器数据。 */
            master_transaction.frame[6] = (uint8_t)(master_transaction.reg_num * 2U);
            for (uint16_t i = 0U; i < master_transaction.reg_num; i++) {
                Modbus_PutU16BE(&master_transaction.frame[7U + i * 2U],
                                master_transaction.write_data[i]);
            }
            master_transaction.frame_len = (uint16_t)(9U + master_transaction.reg_num * 2U);
        } else {
            master_transaction.frame_len = 8U;
        }
    }

    /* Modbus RTU 的 CRC 低字节在前，高字节在后。 */
    crc = Modbus_CRC16(master_transaction.frame,
                       (uint16_t)(master_transaction.frame_len - 2U));
    master_transaction.frame[master_transaction.frame_len - 2U] = (uint8_t)crc;
    master_transaction.frame[master_transaction.frame_len - 1U] = (uint8_t)(crc >> 8);
}

static void ModbusMaster_RetryOrFinish(uint8_t result);

static void ModbusMaster_StartAttempt(void)
{
    ModbusMaster_ResetRx();
    master_transaction.attempts++;
    master_transaction.phase = MASTER_TXN_SENDING;
    master_transaction.deadline = GetTick() + MODBUS_MASTER_SEND_TIMEOUT_MS;
    master_state = MASTER_SENDING;
    master_tx_done = 0U;
    if (ModbusPort_MasterSend(master_transaction.frame,
                              master_transaction.frame_len) != 0U) {
        ModbusMaster_RetryOrFinish(MODBUS_RESULT_BUSY);
    }
}

static void ModbusMaster_SetResult(uint8_t result)
{
    master_transaction.result = result;
    master_transaction.phase = MASTER_TXN_RESULT;
    master_state = result == MODBUS_RESULT_OK ? MASTER_RESP_OK : MASTER_RESP_ERR;
    ModbusMaster_ResetRx();
}

static void ModbusMaster_RetryOrFinish(uint8_t result)
{
    if (master_transaction.attempts < MODBUS_MASTER_MAX_RETRIES) {
        master_transaction.phase = MASTER_TXN_RETRY_WAIT;
        master_transaction.deadline = GetTick() + MODBUS_MASTER_RETRY_GAP_MS;
        master_state = MASTER_RETRY_WAIT;
        ModbusMaster_ResetRx();
    } else {
        ModbusMaster_SetResult(result);
    }
}

/* 校验当前接收缓冲区中的 Modbus RTU 响应，并解析读寄存器数据。 */
static uint8_t ModbusMaster_ValidateResponse(void)
{
    uint16_t len = Master_RX_CNT;

    /* 最短的正常响应为 5 字节，过短的帧无法完成基本校验。 */
    if (len < 5U) {
        return MODBUS_RESULT_LENGTH;
    }
    /* 异常响应固定为：从站地址、功能码|0x80、异常码、CRC。 */
    if (len == 5U && Master_RX_BUFF[0] == master_transaction.slave &&
        Master_RX_BUFF[1] == (uint8_t)(master_transaction.function | 0x80U) &&
        Modbus_CRC16(Master_RX_BUFF, len) == 0U) {
        return MODBUS_RESULT_EXCEPTION;
    }
    /* CRC 校验覆盖整帧，正确帧的计算结果应为 0。 */
    if (Modbus_CRC16(Master_RX_BUFF, len) != 0U) return MODBUS_RESULT_CRC;
    /* 校验响应是否来自目标从站，并且对应当前请求的功能码。 */
    if (Master_RX_BUFF[0] != master_transaction.slave ||
        Master_RX_BUFF[1] != master_transaction.function) return MODBUS_RESULT_HEADER;

    if (master_transaction.function == 0x03U) {
        /* 03：校验数据字节数和总长度后，解析大端序寄存器数据。 */
        if (len != (uint16_t)(5U + master_transaction.reg_num * 2U) ||
            Master_RX_BUFF[2] != (uint8_t)(master_transaction.reg_num * 2U)) {
            return MODBUS_RESULT_LENGTH;
        }
        for (uint16_t i = 0U; i < master_transaction.reg_num; i++) {
            master_transaction.read_data[i] =
                Modbus_GetU16BE(&Master_RX_BUFF[3U + i * 2U]);
        }
        return MODBUS_RESULT_OK;
    }

    /* 06/10 正常响应应原样回显起始地址和寄存器数量或写入值。 */
    if (len != 8U) return MODBUS_RESULT_LENGTH;
    if (Modbus_GetU16BE(&Master_RX_BUFF[2]) != master_transaction.start_reg) {
        return MODBUS_RESULT_ECHO;
    }
    if (master_transaction.function == 0x06U) {
        /* 06：回显单个寄存器的写入值。 */
        if (Modbus_GetU16BE(&Master_RX_BUFF[4]) != master_transaction.write_data[0]) {
            return MODBUS_RESULT_ECHO;
        }
    } else if (Modbus_GetU16BE(&Master_RX_BUFF[4]) != master_transaction.reg_num) {
        /* 10：回显本次写入的寄存器数量。 */
        return MODBUS_RESULT_ECHO;
    }
    return MODBUS_RESULT_OK;
}

/* 在端口和帧间隔定时器初始化后，注册主站端口回调。 */
void ModbusMaster_Init(void)
{
    memset(&master_transaction, 0, sizeof(master_transaction));
    master_tx_done = 0U;
    master_state = MASTER_IDLE;
    ModbusPort_SetMasterCallbacks(ModbusMaster_OnRxByte,
                                  ModbusMaster_OnFrameEnd,
                                  ModbusMaster_OnTxDone);
    ModbusMaster_ResetRx();
}

/* 必须在主循环中高频调用，用于推进超时与重试状态机。 */
void ModbusMaster_Process(void)
{
    uint8_t result;
    uint32_t now = GetTick();

    switch (master_transaction.phase) {
        case MASTER_TXN_SENDING:
            if (master_tx_done) {
                master_tx_done = 0U;
                master_transaction.phase = MASTER_TXN_WAITING;
                master_transaction.deadline = now + MODBUS_MASTER_TIMEOUT_MS;
                master_state = MASTER_WAIT_RESP;
            } else if ((int32_t)(now - master_transaction.deadline) >= 0) {
                ModbusPort_MasterCancel();
                ModbusMaster_RetryOrFinish(MODBUS_RESULT_TIMEOUT);
            }
            break;
        case MASTER_TXN_WAITING:
            if (Master_FrameFlag) {
                result = ModbusMaster_ValidateResponse();
                if (result == MODBUS_RESULT_OK || result == MODBUS_RESULT_EXCEPTION) {
                    ModbusMaster_SetResult(result);
                } else {
                    ModbusMaster_RetryOrFinish(result);
                }
            } else if ((int32_t)(now - master_transaction.deadline) >= 0) {
                ModbusMaster_RetryOrFinish(MODBUS_RESULT_TIMEOUT);
            }
            break;
        case MASTER_TXN_RETRY_WAIT:
            if ((int32_t)(now - master_transaction.deadline) >= 0) {
                ModbusMaster_StartAttempt();
            }
            break;
        default:
            break;
    }
}

uint8_t ModbusMaster_IsBusy(void)
{
    return master_transaction.phase != MASTER_TXN_FREE;
}

void ModbusMaster_Cancel(void)
{
    ModbusPort_MasterCancel();
    master_tx_done = 0U;
    memset(&master_transaction, 0, sizeof(master_transaction));
    master_state = MASTER_IDLE;
    ModbusMaster_ResetRx();
}

/* 提交新事务，或读取与当前参数匹配的已完成事务结果。 */
static uint8_t ModbusMaster_Submit(ModbusMasterClient client,
                                  uint8_t slave, uint8_t function,
                                  uint16_t start_reg, uint16_t reg_num,
                                  const uint16_t *write_data, uint16_t *read_data)
{
    uint8_t result;

    if (master_transaction.phase != MASTER_TXN_FREE) {
        if (!ModbusMaster_RequestMatches(client, slave, function,
                                         start_reg, reg_num, write_data)) {
            return MODBUS_RESULT_BUSY;
        }
        if (master_transaction.phase != MASTER_TXN_RESULT) {
            return MODBUS_RESULT_PENDING;
        }
        result = master_transaction.result;
        if (result == MODBUS_RESULT_OK && function == 0x03U) {
            memcpy(read_data, master_transaction.read_data, reg_num * sizeof(uint16_t));
        }
        memset(&master_transaction, 0, sizeof(master_transaction));
        master_state = MASTER_IDLE;
        return result;
    }

    master_transaction.client = client;
    master_transaction.slave = slave;
    master_transaction.function = function;
    master_transaction.start_reg = start_reg;
    master_transaction.reg_num = reg_num;
    if (function == 0x06U) {
        master_transaction.write_data[0] = write_data[0];
    } else if (function == 0x10U) {
        memcpy(master_transaction.write_data, write_data, reg_num * sizeof(uint16_t));
    }
    ModbusMaster_BuildFrame();
    ModbusMaster_StartAttempt();
    return MODBUS_RESULT_PENDING;
}

uint8_t ModbusMaster_03_ReadHoldReg(ModbusMasterClient client,
                                   uint8_t slave_addr, uint16_t start_reg,
                                   uint16_t reg_num, uint16_t *read_buff)
{
    if (read_buff == NULL || slave_addr == 0U || slave_addr > 247U ||
        reg_num == 0U || reg_num > 125U) return MODBUS_RESULT_PARAM;
    return ModbusMaster_Submit(client, slave_addr, 0x03U,
                              start_reg, reg_num, NULL, read_buff);
}

uint8_t ModbusMaster_06_WriteSingleReg(ModbusMasterClient client,
                                      uint8_t slave_addr, uint16_t reg_addr,
                                      uint16_t reg_data)
{
    if (slave_addr == 0U || slave_addr > 247U) return MODBUS_RESULT_PARAM;
    return ModbusMaster_Submit(client, slave_addr, 0x06U,
                              reg_addr, 1U, &reg_data, NULL);
}

uint8_t ModbusMaster_10_WriteMultiReg(ModbusMasterClient client,
                                     uint8_t slave_addr, uint16_t start_reg,
                                     uint16_t reg_num,
                                     const uint16_t *write_buff)
{
    if (write_buff == NULL || slave_addr == 0U || slave_addr > 247U ||
        reg_num == 0U || reg_num > 123U) return MODBUS_RESULT_PARAM;
    return ModbusMaster_Submit(client, slave_addr, 0x10U,
                              start_reg, reg_num, write_buff, NULL);
}
