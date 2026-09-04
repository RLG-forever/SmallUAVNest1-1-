#include "battery_swap.h"
#include "w25qxx.h"      // 包含SPI Flash读写函数
#include "project.h"
#include "debug_log.h"
#include <stdint.h>   // 提供uint8_t/uint16_t等固定宽度整数类型
#include <string.h>   // 提供memset函数声明

#ifndef FLASH_SIZE
#define FLASH_SIZE  (128UL * 1024UL * 1024UL)   // 128Mbit = 16MByte
#endif
/* 当前状态缓存 */
static Swapstate g_swap_state;
static uint8_t need_save = 0;   // 保存标志
static MotorPositionState g_motor_position_state;
static uint8_t motor_position_state_valid;

static uint32_t MotorPositionStore_Checksum(
    const MotorPositionState *state)
{
    uint32_t version_word;

    version_word = ((uint32_t)state->version << 16) |
                   (uint32_t)state->reserved;
    return state->magic ^ version_word ^
           (uint32_t)state->motor7_position ^
           (uint32_t)state->motor8_position ^ 0xA55A5AA5UL;
}

static uint8_t MotorPositionStore_IsValid(
    const MotorPositionState *state)
{
    if (state->magic != MOTOR_POSITION_STATE_MAGIC ||
        state->version != MOTOR_POSITION_STATE_VERSION ||
        state->checksum != MotorPositionStore_Checksum(state)) {
        return 0U;
    }
    if (state->motor7_position < 0L ||
        state->motor7_position > 1000000L ||
        state->motor8_position < 0L ||
        state->motor8_position > 1000000L) {
        return 0U;
    }
    return 1U;
}

void MotorPositionStore_Init(void)
{
    W25QXX_Read((u8 *)&g_motor_position_state,
                MOTOR_POSITION_STATE_ADDR,
                sizeof(g_motor_position_state));
    motor_position_state_valid =
        MotorPositionStore_IsValid(&g_motor_position_state);
    if (motor_position_state_valid) {
        LOG_INFO("MOTOR_POS",
                 "loaded: motor7=%ld, motor8=%ld\r\n",
                 (long)g_motor_position_state.motor7_position,
                 (long)g_motor_position_state.motor8_position);
    } else {
        memset(&g_motor_position_state, 0,
               sizeof(g_motor_position_state));
        LOG_WARN("MOTOR_POS", "no valid saved position\r\n");
    }
}

uint8_t MotorPositionStore_Save(int32_t motor7_position,
                                int32_t motor8_position)
{
    MotorPositionState verify_state;

    memset(&g_motor_position_state, 0, sizeof(g_motor_position_state));
    g_motor_position_state.magic = MOTOR_POSITION_STATE_MAGIC;
    g_motor_position_state.version = MOTOR_POSITION_STATE_VERSION;
    g_motor_position_state.motor7_position = motor7_position;
    g_motor_position_state.motor8_position = motor8_position;
    g_motor_position_state.checksum =
        MotorPositionStore_Checksum(&g_motor_position_state);

    W25QXX_Write((u8 *)&g_motor_position_state,
                 MOTOR_POSITION_STATE_ADDR,
                 sizeof(g_motor_position_state));
    W25QXX_Read((u8 *)&verify_state,
                MOTOR_POSITION_STATE_ADDR,
                sizeof(verify_state));
    if (memcmp(&verify_state, &g_motor_position_state,
               sizeof(verify_state)) != 0) {
        motor_position_state_valid = 0U;
        LOG_ERROR("MOTOR_POS", "save verification failed\r\n");
        return 0U;
    }

    motor_position_state_valid = 1U;
    LOG_INFO("MOTOR_POS", "saved: motor7=%ld, motor8=%ld\r\n",
             (long)motor7_position, (long)motor8_position);
    return 1U;
}

uint8_t MotorPositionStore_Get(int32_t *motor7_position,
                               int32_t *motor8_position)
{
    if (!motor_position_state_valid || motor7_position == NULL ||
        motor8_position == NULL) {
        return 0U;
    }
    *motor7_position = g_motor_position_state.motor7_position;
    *motor8_position = g_motor_position_state.motor8_position;
    return 1U;
}

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
        LOG_WARN("SWAP", "invalid saved state; default empty_bay=1\r\n");
    } else {
        LOG_INFO("SWAP", "state loaded: empty_bay=%u\r\n",
                 (unsigned int)g_swap_state.empty_bay);
    }
}

/* 保存状态到Flash（自动擦除所在扇区） */
void SwapState_Save(void)
{
    LOG_INFO("SWAP", "saving state: flash_addr=0x%lX\r\n",
             (unsigned long)SWAP_STATE_ADDR);
    W25QXX_Write((u8*)&g_swap_state, SWAP_STATE_ADDR, sizeof(g_swap_state));
    LOG_INFO("SWAP", "state saved: empty_bay=%u\r\n",
             (unsigned int)g_swap_state.empty_bay);
}

/* 获取当前空仓号 */
uint8_t SwapState_GetEmptyBay(void)
{
    return g_swap_state.empty_bay;
}

/* 设置空仓号并保存 */
void SwapState_SetEmptyBay(uint8_t bay)
{
    if (bay < 1 || bay > 3) {
        LOG_ERROR("SWAP", "invalid empty bay=%u\r\n", (unsigned int)bay);
        return;
    }
    g_swap_state.empty_bay = bay;
    SwapState_Save();
}

/* 执行换电操作（由外部触发） */
void BatterySwap_Perform(void)
{
    uint8_t empty_bay = g_swap_state.empty_bay;
    uint8_t next_bay = (empty_bay % 3) + 1;  // 下一个仓号（循环）

    LOG_INFO("SWAP", "manual swap started: empty_bay=%u, next_bay=%u\r\n",
             (unsigned int)empty_bay, (unsigned int)next_bay);

    // 1. 从无人机取下电池（假设无人机当前电池来自上一个仓）
    //    这里调用底层电机控制函数，例如：TakeBatteryFromUAV();
    // 2. 将取下的电池放入空仓（empty_bay）
    //    例如：PutBatteryToBay(empty_bay);
    // 3. 从下一个仓（next_bay）取出电池装入无人机
    //    例如：TakeBatteryFromBay(next_bay);
    //    例如：LoadBatteryToUAV();

    // 模拟操作（实际应调用真实电机控制）
    LOG_DEBUG("SWAP", "manual swap simulated actions started\r\n");

    // 更新空仓号：新的空仓就是刚刚取出的仓（next_bay），因为该仓电池已被取出
    g_swap_state.empty_bay = next_bay;
    SwapState_Save();

    LOG_INFO("SWAP", "manual swap completed: empty_bay=%u\r\n",
             (unsigned int)next_bay);
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
    LOG_INFO("SWAP", "empty bay updated: value=%u, save pending\r\n",
             (unsigned int)next);
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
        LOG_INFO("SWAP", "deferred flash save started\r\n");
        SwapState_Save();
        need_save = 0;
        LOG_INFO("SWAP", "deferred flash save completed\r\n");
    }
}
