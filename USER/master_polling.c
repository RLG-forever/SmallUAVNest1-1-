#include "master_polling.h"

#include "modbus_common.h"
#include "modbus_master.h"
#include "sequence.h"
#include "status_regs.h"
#include "tick.h"

// 轮询块定义：每个块对应一次Modbus读操作
typedef struct {
    uint8_t slave_addr;      // 从站地址（实际传感器的地址）
    uint16_t start_reg;      // 起始寄存器地址（在传感器中的地址）
    uint8_t reg_count;       // 寄存器数量
    uint16_t status_start;   // 对应状态缓存的起始索引
} PollBlock;

// 轮询块数组（根据实际硬件修改从站地址和寄存器对应关系）
static const PollBlock poll_blocks[] = 
{
    {0x0e, 0x00, 1, REG_RAINFALL},          //  降雨量    
    {0x0f, 0x00, 1, REG_WIND_SPEED},        //  风速 
		{0x16, 0x00, 1, REG_CABIN_TEMP},        //  舱		
};
#define POLL_BLOCK_COUNT (sizeof(poll_blocks) / sizeof(poll_blocks[0]))

// 静态变量
static uint8_t current_block = 0;          // 当前处理的块索引
static uint32_t last_poll_time = 0;        // 上一轮结束时间戳
static uint8_t polling_active = 0;         // 是否正在轮询
/* 读缓冲区必须跨主循环保留，因为 0x03 请求会经历多次非阻塞调用。 */
static uint16_t poll_read_buf[8];


/**
 * @brief 分时读取环境传感器的非阻塞轮询任务。
 * @note  每次调用只推进当前块；一轮结束后等待 5 秒再开始下一轮。
 */
void MasterPolling_Task(void)
{
    uint32_t now = GetTick();
    const PollBlock *block;
    uint8_t ret;

    if (Sequence_IsBusy()) {
        return;
    }
    if (!polling_active) {
        if ((now - last_poll_time) < 5000U) {
            return;
        }
        polling_active = 1U;
        current_block = 0U;
    }

    block = &poll_blocks[current_block];
    ret = ModbusMaster_03_ReadHoldReg(MODBUS_MASTER_CLIENT_POLLING,
                                block->slave_addr, block->start_reg,
                                block->reg_count, poll_read_buf);
    if (ret == MODBUS_RESULT_PENDING || ret == MODBUS_RESULT_BUSY) {
        return;
    }
    if (ret == MODBUS_RESULT_OK) {
        StatusRegs_UpdateBatch(block->status_start, poll_read_buf, block->reg_count);
    }

    current_block++;
    if (current_block >= POLL_BLOCK_COUNT) {
        polling_active = 0U;
        last_poll_time = now;
    }
}
