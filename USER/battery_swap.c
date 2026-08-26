#include "battery_swap.h"
#include "w25qxx.h"      // 包含SPI Flash读写函数
#include "project.h"
#include <stdint.h>   // 提供uint8_t/uint16_t等固定宽度整数类型
#include <string.h>   // 提供memset函数声明

#ifndef FLASH_SIZE
#define FLASH_SIZE  (128UL * 1024UL * 1024UL)   // 128Mbit = 16MByte
#endif
/* 当前状态缓存 */
static Swapstate g_swap_state;
static uint8_t need_save = 0;   // 保存标志

/* 初始化：读取Flash，若无效则设为默认值（1号仓空） */
void SwapState_Init(void)
{
		W25QXX_Read((u8*)&g_swap_state, SWAP_STATE_ADDR, sizeof(g_swap_state));
	
    // 检查魔数和范围
    if (g_swap_state.empty_bay < 1 || g_swap_state.empty_bay > 3) {
        // 无效，设为默认值（初始空仓为1）
        g_swap_state.empty_bay = 1;
//        g_swap_state.magic = 0x5A;
//        SwapState_Save();   // 保存到Flash
        printf("Swap state invalid, set default empty_bay=1\n");
    } else {
        printf("Swap state loaded from Flash: empty_bay=%d\n", g_swap_state.empty_bay);
    }
}

/* 保存状态到Flash（自动擦除所在扇区） */
void SwapState_Save(void)
{
    printf("开始写 Flash 地址 0x%X\n", SWAP_STATE_ADDR);
    W25QXX_Write((u8*)&g_swap_state, SWAP_STATE_ADDR, sizeof(g_swap_state));
    printf("Swap state saved: empty_bay=%d\n", g_swap_state.empty_bay);
}

/* 获取当前空仓号 */
uint8_t SwapState_GetEmptyBay(void)
{
    return g_swap_state.empty_bay;
}

/* 设置空仓号并保存 */
void SwapState_SetEmptyBay(uint8_t bay)
{
    if (bay < 1 || bay > 3) return;
    g_swap_state.empty_bay = bay;
    SwapState_Save();
}

/* 执行换电操作（由外部触发） */
void BatterySwap_Perform(void)
{
    uint8_t empty_bay = g_swap_state.empty_bay;
    uint8_t next_bay = (empty_bay % 3) + 1;  // 下一个仓号（循环）

    printf("=== Battery Swap Start ===\n");
    printf("Current empty bay: %d, next bay to take: %d\n", empty_bay, next_bay);

    // 1. 从无人机取下电池（假设无人机当前电池来自上一个仓）
    //    这里调用底层电机控制函数，例如：TakeBatteryFromUAV();
    // 2. 将取下的电池放入空仓（empty_bay）
    //    例如：PutBatteryToBay(empty_bay);
    // 3. 从下一个仓（next_bay）取出电池装入无人机
    //    例如：TakeBatteryFromBay(next_bay);
    //    例如：LoadBatteryToUAV();

    // 模拟操作（实际应调用真实电机控制）
    printf("Step 1: Take battery from UAV\n");
    printf("Step 2: Put battery to bay %d\n", empty_bay);
    printf("Step 3: Take battery from bay %d and load to UAV\n", next_bay);

    // 更新空仓号：新的空仓就是刚刚取出的仓（next_bay），因为该仓电池已被取出
    g_swap_state.empty_bay = next_bay;
    SwapState_Save();

    printf("=== Swap completed, new empty bay: %d ===\n", next_bay);
}

// 用于序列步骤表，更新空仓号并保存到Flash
uint8_t UpdateEmptyBay(void)
{
    uint8_t current = SwapState_GetEmptyBay();
    uint8_t next = (current % 3) + 1;
    g_swap_state.empty_bay = next;
    need_save = 1;
		// 同步更新状态寄存器 0x19，供网关查询
    StatusRegs_Update(REG_RESERVED2, next);
    printf("换电完成，空仓号更新为 %d (待保存)\n", next);
    return 0;
}

// 主循环中调用，尝试保存
void SwapState_TrySave(void)
{
		static uint32_t last_print = 0;
    if (GetTick() - last_print > 5000) {
        last_print = GetTick();
//        printf("TrySave: need_save=%d, master_state=%d, seq_busy=%d\r\n", 
//               need_save, master_state, Sequence_IsBusy());
    }
    if (need_save && !ModbusMaster_IsBusy() && !Sequence_IsBusy()) {
        printf("开始保存 Flash...\n");
        SwapState_Save();
        need_save = 0;
        printf("Flash 保存完成\n");
    }
}
