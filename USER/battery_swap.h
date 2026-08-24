#ifndef __BATTERY_SWAP_H
#define __BATTERY_SWAP_H

#include <stdint.h>

/* 电池仓数量 */
#define BAY_COUNT           3

/* Flash存储地址（使用FLASH末尾区域） */
#define FLASH_SIZE      (16UL * 1024UL * 1024UL)   // 16M 字节
/* 存储地址直接使用宏计算 */
#define SWAP_STATE_ADDR (FLASH_SIZE - 512)

/* 状态结构体 */
typedef struct {
    uint8_t empty_bay;      // 当前空仓编号 (1~3)
//    uint8_t magic;          // 校验字节，固定为0x5A
} Swapstate;

/* 函数声明 */
void SwapState_Init(void);              // 初始化，读取Flash或设置默认
void SwapState_Save(void);              // 保存当前状态到Flash
uint8_t SwapState_GetEmptyBay(void);    // 获取当前空仓号
void SwapState_SetEmptyBay(uint8_t bay); // 设置空仓号并保存
void BatterySwap_Perform(void);         // 执行一次换电操作（由外部触发）
uint8_t UpdateEmptyBay(void);
void SwapState_TrySave(void);

#endif