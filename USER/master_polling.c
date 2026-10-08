#include "master_polling.h"

#include "modbus_common.h"
#include "modbus_master.h"
#include "sequence.h"
#include "slave_ctrl.h"
#include "status_regs.h"
#include "tick.h"
#include "debug_log.h"

/* 一轮轮询结束后，等待10秒再开始下一轮。 */
#define MASTER_POLLING_INTERVAL_MS 10000U

// 轮询块定义：每个块对应一次Modbus读操作
typedef struct {
    uint8_t slave_addr;      // 从站地址（实际传感器的地址）
    uint16_t start_reg;      // 起始寄存器地址（在传感器中的地址）
    uint8_t reg_count;       // 寄存器数量
    uint16_t status_start;   // 对应状态缓存的起始索引
    uint8_t function;        // 读取功能码：02离散输入，03保持寄存器
} PollBlock;

// 轮询块数组（根据实际硬件修改从站地址和寄存器对应关系）
static const PollBlock poll_blocks[] = 
{
    {RAINFALL_SLAVE, RAINFALL_VALUE_REG, 1, REG_RAINFALL, 0x03U},    // 降雨量
    {WIND_SPEED_SLAVE, WIND_SPEED_VALUE_REG, 1, REG_WIND_SPEED, 0x03U}, // 风速
    {CABIN_TEMP_SLAVE, CABIN_TEMP_VALUE_REG, 1, REG_CABIN_TEMP, 0x03U}, // 机舱温度
    {AIR_CONDITIONER_SLAVE, AIR_COOLING_STATE_REG,
     AIR_HEATING_DEHUMIDIFY_STATE_REG - AIR_COOLING_STATE_REG + 1U,
     REG_AC_STATE, 0x02U}, // 四个BOOL状态合并为空调运行状态
#if LANDING_BAY_PUSH_CURRENT_CHECK_ENABLE == 1U
    {CHARGING_CURRENT_SLAVE, CHARGING_CURRENT_VALUE_REG,
     1U, REG_CHARGING_CURRENT, 0x03U},
#endif
};
#define POLL_BLOCK_COUNT (sizeof(poll_blocks) / sizeof(poll_blocks[0]))

/* 雨量传感器累计值维护：每十分钟清除一次传感器的历史数据。 */
#define RAIN_HISTORY_CLEAR_REG               0x0006U
#define RAIN_HISTORY_CLEAR_VALUE             0x0000U
#define RAIN_HISTORY_CLEAR_INTERVAL_MS       (10UL * 60UL * 1000UL)
#define RAIN_HISTORY_CLEAR_RETRY_INTERVAL_MS 5000U

// 静态变量
static uint8_t current_block = 0;          // 当前处理的块索引
static uint32_t last_poll_time = 0;        // 上一轮结束时间戳
static uint8_t polling_active = 0;         // 是否正在轮询
/* 读缓冲区必须跨主循环保留，因为读取请求会经历多次非阻塞调用。 */
static uint16_t poll_read_buf[8];
static uint32_t rain_history_clear_deadline;
static uint8_t rain_history_clear_initialized;
static uint8_t rain_history_clear_pending;

static void MasterPolling_UpdateAirConditionerState(const uint16_t *states)
{
    uint16_t state = 3U; // 四项都停止：待机

    /* 同时出现制冷和加热标志时，按规则顺序优先制冷。 */
    if (states[0] != 0U ||
        states[AIR_COOLING_DEHUMIDIFY_STATE_REG - AIR_COOLING_STATE_REG] != 0U) {
        state = 1U;
    } else if (states[AIR_HEATING_STATE_REG - AIR_COOLING_STATE_REG] != 0U ||
               states[AIR_HEATING_DEHUMIDIFY_STATE_REG - AIR_COOLING_STATE_REG] != 0U) {
        state = 2U;
    }
    StatusRegs_Update(REG_AC_STATE, state);
    LOG_DEBUG("POLL", "air conditioner state updated: state=%u\r\n",
              (unsigned int)state);
}


/**
 * @brief 分时读取环境传感器、电流和空调状态的非阻塞轮询任务。
 * @note  每次调用只推进当前块；轮询间隔由MASTER_POLLING_INTERVAL_MS配置。
 */
void MasterPolling_Task(void)
{
    uint32_t now = GetTick();
    const PollBlock *block;
    uint8_t ret;

    if (Sequence_IsBusy()) {
        return;
    }

    /* 首次进入轮询任务后开始计时，避免上电立即清除雨量历史数据。 */
    if (!rain_history_clear_initialized) {
        rain_history_clear_initialized = 1U;
        rain_history_clear_deadline =
            now + RAIN_HISTORY_CLEAR_INTERVAL_MS;
    }

    if (!rain_history_clear_pending &&
        (int32_t)(now - rain_history_clear_deadline) >= 0) {
        rain_history_clear_pending = 1U;
        LOG_INFO("POLL",
                 "rain history clear due: slave=0x%02X, reg=0x%04X\r\n",
                 (unsigned int)RAINFALL_SLAVE,
                 (unsigned int)RAIN_HISTORY_CLEAR_REG);
    }

    /*
     * 已有读请求时先让该请求结束；清零请求在两轮轮询之间发起，
     * 因而不会与当前轮询请求争用同一笔主站事务。
     */
    if (rain_history_clear_pending && !polling_active) {
        ret = ModbusMaster_06_WriteSingleReg(
            MODBUS_MASTER_CLIENT_POLLING,
            RAINFALL_SLAVE,
            RAIN_HISTORY_CLEAR_REG,
            RAIN_HISTORY_CLEAR_VALUE);
        if (ret == MODBUS_RESULT_PENDING || ret == MODBUS_RESULT_BUSY) {
            return;
        }
        if (ret == MODBUS_RESULT_OK) {
            rain_history_clear_pending = 0U;
            rain_history_clear_deadline =
                now + RAIN_HISTORY_CLEAR_INTERVAL_MS;
            LOG_INFO("POLL",
                     "rain history cleared; next clear in %lu seconds\r\n",
                     (unsigned long)(RAIN_HISTORY_CLEAR_INTERVAL_MS /
                                     1000UL));
        } else {
            rain_history_clear_pending = 0U;
            rain_history_clear_deadline =
                now + RAIN_HISTORY_CLEAR_RETRY_INTERVAL_MS;
            LOG_WARN("POLL",
                     "rain history clear failed: result=%u; retry in %lu seconds\r\n",
                     (unsigned int)ret,
                     (unsigned long)(RAIN_HISTORY_CLEAR_RETRY_INTERVAL_MS /
                                     1000UL));
        }
        return;
    }

    if (!polling_active) {
        if ((now - last_poll_time) < MASTER_POLLING_INTERVAL_MS) {
            return;
        }
        polling_active = 1U;
        current_block = 0U;
    }

    block = &poll_blocks[current_block];
    if (block->function == 0x02U) {
        ret = ModbusMaster_02_ReadDiscreteInputs(
            MODBUS_MASTER_CLIENT_POLLING, block->slave_addr, block->start_reg,
            block->reg_count, poll_read_buf);
    } else {
        ret = ModbusMaster_03_ReadHoldReg(
            MODBUS_MASTER_CLIENT_POLLING, block->slave_addr, block->start_reg,
            block->reg_count, poll_read_buf);
    }
    if (ret == MODBUS_RESULT_PENDING || ret == MODBUS_RESULT_BUSY) {
        return;
    }
    if (ret == MODBUS_RESULT_OK) {
        if (block->status_start == REG_AC_STATE) {
            MasterPolling_UpdateAirConditionerState(poll_read_buf);
        } else {
            StatusRegs_UpdateBatch(block->status_start, poll_read_buf, block->reg_count);
        }
        LOG_DEBUG("POLL", "sensor updated: slave=0x%02X, status_reg=%u, value=%u\r\n",
                  (unsigned int)block->slave_addr,
                  (unsigned int)block->status_start,
                  (unsigned int)poll_read_buf[0]);
    } else {
        LOG_WARN("POLL", "sensor read failed: slave=0x%02X, reg=0x%04X, result=%u\r\n",
                 (unsigned int)block->slave_addr,
                 (unsigned int)block->start_reg,
                 (unsigned int)ret);
    }

    current_block++;
    if (current_block >= POLL_BLOCK_COUNT) {
        polling_active = 0U;
        last_poll_time = now;
    }
}
