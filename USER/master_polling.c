#include "project.h"

#include <string.h>

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
static volatile uint8_t master_busy = 0;   // 主站总线忙标志（用于从站等待）



// 轮询计数器，用于分时读取不同传感器（避免总线冲突）
static uint8_t poll_step = 0;

void MasterPolling_Init(void)
{
    // 初始化串口等（若需要）
}

void MasterBusy_Acquire(void)
{
    ENTER_CRITICAL();
    master_busy = 1;
    EXIT_CRITICAL();
}

void MasterBusy_Release(void)
{
    ENTER_CRITICAL();
    master_busy = 0;
    EXIT_CRITICAL();
}

uint8_t MasterPolling_IsBusy(void)
{
    return master_busy;   // 原子操作，无需临界区
}

void MasterPolling_Task(void)
{
    uint32_t now = GetTick();

		if (Sequence_IsBusy()) return;  // 序列执行时暂停主站轮询
	
    // 如果不在轮询中，且距离上次结束超过5秒，则开始新的一轮
    if (!polling_active && (now - last_poll_time >= 5000)) {
        polling_active = 1;
        current_block = 0;
    }

    if (polling_active) {
        const PollBlock *block = &poll_blocks[current_block];
        uint16_t read_buf[8];   // 足够容纳最大块长度（目前最大5）
        uint8_t ret;

        // 设置总线忙标志
        master_busy = 1;

        ret = Modbus_03_ReadHoldReg(block->slave_addr, block->start_reg, block->reg_count, read_buf);

        // 清除总线忙标志
        master_busy = 0;

        if (ret == 0) {
            // 读取成功，更新缓存
            StatusRegs_UpdateBatch(block->status_start, read_buf, block->reg_count);
        } else {
            // 读取失败，可添加日志（这里忽略）
        }

        current_block++;
        if (current_block >= POLL_BLOCK_COUNT) {
            polling_active = 0;          // 本轮结束
            last_poll_time = now;         // 记录结束时间
        }
    }
}