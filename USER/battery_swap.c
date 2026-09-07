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

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    int32_t motor7_position;
    int32_t motor8_position;
    uint32_t checksum;
} MotorPositionStateV1;

static uint32_t MotorPositionStore_Checksum(
    const MotorPositionState *state)
{
    uint32_t checksum;
    uint8_t index;

    checksum = state->magic ^
               (((uint32_t)state->version << 16) |
                (uint32_t)state->valid_mask) ^
               0xA55A5AA5UL;
    for (index = 0U; index < MOTOR_POSITION_STORE_COUNT; index++) {
        checksum ^= (uint32_t)state->motor_positions[index];
    }
    return checksum;
}

static uint8_t MotorPositionStore_IsValid(
    const MotorPositionState *state)
{
    if (state->magic != MOTOR_POSITION_STATE_MAGIC ||
        state->version != MOTOR_POSITION_STATE_VERSION ||
        (state->valid_mask &
         (uint16_t)(~MOTOR_POSITION_STORE_VALID_MASK)) != 0U ||
        state->checksum != MotorPositionStore_Checksum(state)) {
        return 0U;
    }
    return 1U;
}

static uint8_t MotorPositionStore_IsLegacyValid(
    const MotorPositionStateV1 *state)
{
    uint32_t version_word;
    uint32_t checksum;

    version_word = ((uint32_t)state->version << 16) |
                   (uint32_t)state->reserved;
    checksum = state->magic ^ version_word ^
               (uint32_t)state->motor7_position ^
               (uint32_t)state->motor8_position ^ 0xA55A5AA5UL;
    return (state->magic == MOTOR_POSITION_STATE_MAGIC &&
            state->version == 1U && state->checksum == checksum) ? 1U : 0U;
}

static uint8_t MotorPositionStore_Write(void)
{
    MotorPositionState verify_state;

    g_motor_position_state.magic = MOTOR_POSITION_STATE_MAGIC;
    g_motor_position_state.version = MOTOR_POSITION_STATE_VERSION;
    g_motor_position_state.checksum =
        MotorPositionStore_Checksum(&g_motor_position_state);

    W25QXX_Write((u8 *)&g_motor_position_state,
                 MOTOR_POSITION_STATE_ADDR,
                 sizeof(g_motor_position_state));
    W25QXX_Read((u8 *)&verify_state,
                MOTOR_POSITION_STATE_ADDR,
                sizeof(verify_state));
    if (memcmp(&verify_state, &g_motor_position_state,
               sizeof(verify_state)) != 0 ||
        !MotorPositionStore_IsValid(&verify_state)) {
        motor_position_state_valid = 0U;
        LOG_ERROR("MOTOR_POS", "save verification failed\r\n");
        return 0U;
    }

    motor_position_state_valid = 1U;
    return 1U;
}

void MotorPositionStore_Init(void)
{
    MotorPositionStateV1 legacy_state;

    W25QXX_Read((u8 *)&g_motor_position_state,
                MOTOR_POSITION_STATE_ADDR,
                sizeof(g_motor_position_state));
    motor_position_state_valid =
        MotorPositionStore_IsValid(&g_motor_position_state);
    if (motor_position_state_valid) {
        LOG_INFO("MOTOR_POS",
                 "loaded motor positions: valid_mask=0x%02X\r\n",
                 (unsigned int)g_motor_position_state.valid_mask);
        return;
    }

    W25QXX_Read((u8 *)&legacy_state, MOTOR_POSITION_STATE_ADDR,
                sizeof(legacy_state));
    if (MotorPositionStore_IsLegacyValid(&legacy_state)) {
        memset(&g_motor_position_state, 0,
               sizeof(g_motor_position_state));
        g_motor_position_state.valid_mask =
            (uint16_t)((1U << (7U -
                              MOTOR_POSITION_STORE_FIRST_SLAVE)) |
                       (1U << (8U -
                              MOTOR_POSITION_STORE_FIRST_SLAVE)));
        g_motor_position_state.motor_positions[
            7U - MOTOR_POSITION_STORE_FIRST_SLAVE] =
            legacy_state.motor7_position;
        g_motor_position_state.motor_positions[
            8U - MOTOR_POSITION_STORE_FIRST_SLAVE] =
            legacy_state.motor8_position;
        if (MotorPositionStore_Write()) {
            LOG_INFO("MOTOR_POS",
                     "legacy motor7/8 positions migrated to version %u\r\n",
                     (unsigned int)MOTOR_POSITION_STATE_VERSION);
        }
        return;
    }

    memset(&g_motor_position_state, 0,
           sizeof(g_motor_position_state));
    LOG_WARN("MOTOR_POS", "no valid saved position\r\n");
}

uint8_t MotorPositionStore_Save(int32_t motor7_position,
                                int32_t motor8_position)
{
    const uint8_t slave_addrs[2] = {
        7U, 8U
    };
    const int32_t positions[2] = {
        motor7_position, motor8_position
    };

    return MotorPositionStore_UpdateBatch(slave_addrs, positions, 2U);
}

uint8_t MotorPositionStore_Get(int32_t *motor7_position,
                               int32_t *motor8_position)
{
    uint16_t required_mask;

    required_mask =
        (uint16_t)((1U << (7U -
                          MOTOR_POSITION_STORE_FIRST_SLAVE)) |
                   (1U << (8U -
                          MOTOR_POSITION_STORE_FIRST_SLAVE)));
    if (!motor_position_state_valid || motor7_position == NULL ||
        motor8_position == NULL ||
        (g_motor_position_state.valid_mask & required_mask) !=
            required_mask) {
        return 0U;
    }
    *motor7_position = g_motor_position_state.motor_positions[
        7U - MOTOR_POSITION_STORE_FIRST_SLAVE];
    *motor8_position = g_motor_position_state.motor_positions[
        8U - MOTOR_POSITION_STORE_FIRST_SLAVE];
    return 1U;
}

uint8_t MotorPositionStore_GetAll(
    int32_t positions[MOTOR_POSITION_STORE_COUNT], uint16_t *valid_mask)
{
    if (!motor_position_state_valid || positions == NULL ||
        valid_mask == NULL) {
        return 0U;
    }
    memcpy(positions, g_motor_position_state.motor_positions,
           sizeof(g_motor_position_state.motor_positions));
    *valid_mask = g_motor_position_state.valid_mask;
    return 1U;
}

uint8_t MotorPositionStore_UpdateBatch(const uint8_t *slave_addrs,
                                       const int32_t *positions,
                                       uint8_t count)
{
    uint8_t index;
    uint8_t position_index;
    uint8_t changed = 0U;

    if (slave_addrs == NULL || positions == NULL || count == 0U) {
        return 0U;
    }
    if (!motor_position_state_valid) {
        memset(&g_motor_position_state, 0,
               sizeof(g_motor_position_state));
    }

    for (index = 0U; index < count; index++) {
        if (slave_addrs[index] < MOTOR_POSITION_STORE_FIRST_SLAVE ||
            slave_addrs[index] >=
                MOTOR_POSITION_STORE_FIRST_SLAVE +
                MOTOR_POSITION_STORE_COUNT) {
            continue;
        }
        position_index = (uint8_t)(
            slave_addrs[index] - MOTOR_POSITION_STORE_FIRST_SLAVE);
        if ((g_motor_position_state.valid_mask &
             (uint16_t)(1U << position_index)) == 0U ||
            g_motor_position_state.motor_positions[position_index] !=
                positions[index]) {
            g_motor_position_state.motor_positions[position_index] =
                positions[index];
            g_motor_position_state.valid_mask |=
                (uint16_t)(1U << position_index);
            changed = 1U;
        }
    }

    if (!changed) {
        return 1U;
    }
    if (!MotorPositionStore_Write()) {
        return 0U;
    }
    LOG_INFO("MOTOR_POS", "positions updated: valid_mask=0x%02X\r\n",
             (unsigned int)g_motor_position_state.valid_mask);
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
