#include "battery_swap.h"
#include "w25qxx.h"      // 包含SPI Flash读写函数
#include "project.h"
#include "debug_log.h"
#include <stdint.h>   // 提供uint8_t/uint16_t等固定宽度整数类型
#include <stddef.h>
#include <string.h>   // 提供memset函数声明

#ifndef FLASH_SIZE
#define FLASH_SIZE  (128UL * 1024UL * 1024UL)   // 128Mbit = 16MByte
#endif
/* 当前状态缓存 */
static Swapstate g_swap_state;
static uint8_t need_save = 0;   // 保存标志
static MotorPositionState g_motor_position_state;
static uint8_t motor_position_state_valid;

#define MOTOR_POSITION_JOURNAL_MAGIC       0x4D504A32UL
#define MOTOR_POSITION_JOURNAL_COMMIT      0x434F4D54UL
#define MOTOR_POSITION_JOURNAL_FLAG_MOVING 0x00000001UL
#define MOTOR_POSITION_JOURNAL_RECORD_SIZE 64UL
#define MOTOR_POSITION_JOURNAL_RECORD_COUNT \
    (MOTOR_POSITION_JOURNAL_SIZE / MOTOR_POSITION_JOURNAL_RECORD_SIZE)

typedef struct {
    uint32_t magic;
    uint32_t sequence;
    MotorPositionState state;
    uint32_t checksum;
    uint32_t flags;
    uint32_t commit;
} MotorPositionJournalRecord;

typedef char MotorPositionJournalRecordSizeCheck[
    sizeof(MotorPositionJournalRecord) == MOTOR_POSITION_JOURNAL_RECORD_SIZE
        ? 1 : -1];

static uint32_t motor_position_journal_sequence;
static uint32_t motor_position_journal_latest_addr;
static uint32_t motor_position_journal_next_addr;
static uint8_t motor_position_journal_has_record;
static uint8_t motor_position_motion_active;

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

static uint32_t MotorPositionJournal_Checksum(
    const MotorPositionJournalRecord *record)
{
    uint32_t checksum;
    uint8_t index;

    checksum = record->magic ^ record->sequence ^
               record->state.magic ^
               (((uint32_t)record->state.version << 16) |
               (uint32_t)record->state.valid_mask) ^
               record->state.checksum ^ record->flags ^ 0xC33CA55AUL;
    for (index = 0U; index < MOTOR_POSITION_STORE_COUNT; index++) {
        checksum ^= (uint32_t)record->state.motor_positions[index];
        checksum = (checksum << 5) | (checksum >> 27);
    }
    return checksum;
}

static uint8_t MotorPositionJournal_IsValid(
    const MotorPositionJournalRecord *record)
{
    return (record->magic == MOTOR_POSITION_JOURNAL_MAGIC &&
            record->commit == MOTOR_POSITION_JOURNAL_COMMIT &&
            MotorPositionStore_IsValid(&record->state) &&
            record->checksum == MotorPositionJournal_Checksum(record))
               ? 1U : 0U;
}

static uint8_t MotorPositionJournal_IsNewer(uint32_t candidate,
                                            uint32_t reference)
{
    return ((int32_t)(candidate - reference) > 0) ? 1U : 0U;
}

static uint32_t MotorPositionJournal_NextAddress(uint32_t address)
{
    address += MOTOR_POSITION_JOURNAL_RECORD_SIZE;
    if (address >= MOTOR_POSITION_JOURNAL_ADDR +
                   MOTOR_POSITION_JOURNAL_SIZE) {
        address = MOTOR_POSITION_JOURNAL_ADDR;
    }
    return address;
}

static uint8_t MotorPositionJournal_RecordIsErased(uint32_t address)
{
    uint8_t bytes[MOTOR_POSITION_JOURNAL_RECORD_SIZE];
    uint16_t index;

    W25QXX_Read(bytes, address, (uint16_t)sizeof(bytes));
    for (index = 0U; index < (uint16_t)sizeof(bytes); index++) {
        if (bytes[index] != 0xFFU) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t MotorPositionJournal_SectorIsErased(uint32_t sector_addr)
{
    uint32_t address;

    for (address = sector_addr;
         address < sector_addr + MOTOR_POSITION_JOURNAL_SECTOR_SIZE;
         address += MOTOR_POSITION_JOURNAL_RECORD_SIZE) {
        if (!MotorPositionJournal_RecordIsErased(address)) {
            return 0U;
        }
    }
    return 1U;
}

static void MotorPositionJournal_FindNextErased(void)
{
    uint32_t address = motor_position_journal_next_addr;
    uint32_t count;

    for (count = 0UL; count < MOTOR_POSITION_JOURNAL_RECORD_COUNT; count++) {
        if (MotorPositionJournal_RecordIsErased(address)) {
            motor_position_journal_next_addr = address;
            return;
        }
        address = MotorPositionJournal_NextAddress(address);
    }
    motor_position_journal_next_addr =
        MotorPositionJournal_NextAddress(
            motor_position_journal_latest_addr);
}

static void MotorPositionJournal_Scan(void)
{
    MotorPositionJournalRecord record;
    uint32_t address;

    motor_position_journal_has_record = 0U;
    motor_position_journal_sequence = 0UL;
    motor_position_journal_latest_addr = MOTOR_POSITION_JOURNAL_ADDR;
    motor_position_journal_next_addr = MOTOR_POSITION_JOURNAL_ADDR;

    for (address = MOTOR_POSITION_JOURNAL_ADDR;
         address < MOTOR_POSITION_JOURNAL_ADDR +
                   MOTOR_POSITION_JOURNAL_SIZE;
         address += MOTOR_POSITION_JOURNAL_RECORD_SIZE) {
        W25QXX_Read((u8 *)&record, address, sizeof(record));
        if (!MotorPositionJournal_IsValid(&record)) {
            continue;
        }
        if (!motor_position_journal_has_record ||
            MotorPositionJournal_IsNewer(
                record.sequence, motor_position_journal_sequence)) {
            motor_position_journal_has_record = 1U;
            motor_position_journal_sequence = record.sequence;
            motor_position_journal_latest_addr = address;
            g_motor_position_state = record.state;
            motor_position_motion_active =
                (record.flags &
                 MOTOR_POSITION_JOURNAL_FLAG_MOVING) != 0UL
                    ? 1U : 0U;
        }
    }

    if (motor_position_journal_has_record) {
        motor_position_journal_next_addr =
            MotorPositionJournal_NextAddress(
                motor_position_journal_latest_addr);
    }
    MotorPositionJournal_FindNextErased();
}

uint8_t MotorPositionStore_PrepareJournal(void)
{
    uint32_t current_sector;
    uint32_t next_sector;
    uint32_t sector_index;

    if (!motor_position_journal_has_record) {
        if (MotorPositionJournal_RecordIsErased(
                motor_position_journal_next_addr)) {
            return 1U;
        }
        W25QXX_Erase_Sector(
            MOTOR_POSITION_JOURNAL_ADDR /
            MOTOR_POSITION_JOURNAL_SECTOR_SIZE);
        return MotorPositionJournal_SectorIsErased(
            MOTOR_POSITION_JOURNAL_ADDR);
    }

    current_sector =
        (motor_position_journal_latest_addr -
         MOTOR_POSITION_JOURNAL_ADDR) /
        MOTOR_POSITION_JOURNAL_SECTOR_SIZE;
    sector_index = (current_sector + 1UL) %
                   MOTOR_POSITION_JOURNAL_SECTOR_COUNT;
    next_sector = MOTOR_POSITION_JOURNAL_ADDR +
                  sector_index * MOTOR_POSITION_JOURNAL_SECTOR_SIZE;
    if (MotorPositionJournal_SectorIsErased(next_sector)) {
        return 1U;
    }

    LOG_INFO("MOTOR_POS",
             "preparing journal sector: index=%lu\r\n",
             (unsigned long)sector_index);
    W25QXX_Erase_Sector(
        next_sector / MOTOR_POSITION_JOURNAL_SECTOR_SIZE);
    if (!MotorPositionJournal_SectorIsErased(next_sector)) {
        LOG_ERROR("MOTOR_POS",
                  "journal sector erase verification failed: index=%lu\r\n",
                  (unsigned long)sector_index);
        return 0U;
    }
    return 1U;
}

static uint8_t MotorPositionStore_Write(void)
{
    MotorPositionJournalRecord record;
    MotorPositionJournalRecord verify_record;
    uint32_t commit = MOTOR_POSITION_JOURNAL_COMMIT;
    uint32_t write_addr;

    g_motor_position_state.magic = MOTOR_POSITION_STATE_MAGIC;
    g_motor_position_state.version = MOTOR_POSITION_STATE_VERSION;
    g_motor_position_state.checksum =
        MotorPositionStore_Checksum(&g_motor_position_state);

    MotorPositionJournal_FindNextErased();
    write_addr = motor_position_journal_next_addr;
    if (!MotorPositionJournal_RecordIsErased(write_addr)) {
        LOG_ERROR("MOTOR_POS", "journal has no erased record\r\n");
        return 0U;
    }

    memset(&record, 0xFF, sizeof(record));
    record.magic = MOTOR_POSITION_JOURNAL_MAGIC;
    record.sequence = motor_position_journal_sequence + 1UL;
    record.state = g_motor_position_state;
    record.flags = motor_position_motion_active
                       ? MOTOR_POSITION_JOURNAL_FLAG_MOVING
                       : 0UL;
    record.checksum = MotorPositionJournal_Checksum(&record);

    /* 先写数据，最后单独写commit；掉电时旧记录仍然有效。 */
    W25QXX_Write_NoCheck((u8 *)&record, write_addr,
                         (uint16_t)offsetof(
                             MotorPositionJournalRecord, commit));
    W25QXX_Write_NoCheck((u8 *)&commit,
                         write_addr + (uint32_t)offsetof(
                             MotorPositionJournalRecord, commit),
                         (uint16_t)sizeof(commit));
    W25QXX_Read((u8 *)&verify_record, write_addr,
                sizeof(verify_record));
    if (!MotorPositionJournal_IsValid(&verify_record) ||
        memcmp(&verify_record.state, &g_motor_position_state,
               sizeof(g_motor_position_state)) != 0) {
        LOG_ERROR("MOTOR_POS", "save verification failed\r\n");
        motor_position_journal_next_addr =
            MotorPositionJournal_NextAddress(write_addr);
        return 0U;
    }

    motor_position_journal_has_record = 1U;
    motor_position_journal_sequence = record.sequence;
    motor_position_journal_latest_addr = write_addr;
    motor_position_journal_next_addr =
        MotorPositionJournal_NextAddress(write_addr);
    motor_position_state_valid = 1U;
    return 1U;
}

void MotorPositionStore_Init(void)
{
    MotorPositionStateV1 legacy_state;

    MotorPositionJournal_Scan();
    if (motor_position_journal_has_record &&
        MotorPositionStore_IsValid(&g_motor_position_state)) {
        motor_position_state_valid = 1U;
        LOG_INFO("MOTOR_POS",
                 "loaded journal positions: sequence=%lu, valid_mask=0x%02X, moving=%u\r\n",
                 (unsigned long)motor_position_journal_sequence,
                 (unsigned int)g_motor_position_state.valid_mask,
                 (unsigned int)motor_position_motion_active);
        if (motor_position_motion_active) {
            LOG_WARN("MOTOR_POS",
                     "previous motor7/8 movement ended unexpectedly\r\n");
        }
        MotorPositionStore_PrepareJournal();
        return;
    }

    /* 升级时从原固定地址读取版本2数据，再迁移到日志。 */
    W25QXX_Read((u8 *)&g_motor_position_state,
                MOTOR_POSITION_STATE_ADDR,
                sizeof(g_motor_position_state));
    motor_position_state_valid =
        MotorPositionStore_IsValid(&g_motor_position_state);
    if (motor_position_state_valid) {
        motor_position_motion_active = 0U;
        LOG_INFO("MOTOR_POS",
                 "loaded motor positions: valid_mask=0x%02X\r\n",
                 (unsigned int)g_motor_position_state.valid_mask);
        if (!MotorPositionStore_Write()) {
            LOG_ERROR("MOTOR_POS", "version 2 journal migration failed\r\n");
        }
        MotorPositionStore_PrepareJournal();
        return;
    }

    W25QXX_Read((u8 *)&legacy_state, MOTOR_POSITION_STATE_ADDR,
                sizeof(legacy_state));
    if (MotorPositionStore_IsLegacyValid(&legacy_state)) {
        motor_position_motion_active = 0U;
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
        MotorPositionStore_PrepareJournal();
        return;
    }

    memset(&g_motor_position_state, 0,
           sizeof(g_motor_position_state));
    motor_position_motion_active = 0U;
    LOG_WARN("MOTOR_POS", "no valid saved position\r\n");
    MotorPositionStore_PrepareJournal();
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
    MotorPositionState previous_state = g_motor_position_state;
    uint8_t previous_valid = motor_position_state_valid;
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
        g_motor_position_state = previous_state;
        motor_position_state_valid = previous_valid;
        return 0U;
    }
    LOG_INFO("MOTOR_POS", "positions updated: valid_mask=0x%02X\r\n",
             (unsigned int)g_motor_position_state.valid_mask);
    return 1U;
}

uint8_t MotorPositionStore_BeginMotion(void)
{
    if (motor_position_motion_active) {
        return 1U;
    }
    motor_position_motion_active = 1U;
    if (!MotorPositionStore_Write()) {
        motor_position_motion_active = 0U;
        return 0U;
    }
    LOG_INFO("MOTOR_POS", "motor7/8 movement checkpoint started\r\n");
    return 1U;
}

uint8_t MotorPositionStore_EndMotion(void)
{
    if (!motor_position_motion_active) {
        return 1U;
    }
    motor_position_motion_active = 0U;
    if (!MotorPositionStore_Write()) {
        motor_position_motion_active = 1U;
        return 0U;
    }
    LOG_INFO("MOTOR_POS", "motor7/8 movement checkpoint completed\r\n");
    return 1U;
}

uint8_t MotorPositionStore_WasMotionInterrupted(void)
{
    return motor_position_motion_active;
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

/*
 * 用于落地序列步骤表：新电池从下一仓取出且电机2退回原点后，
 * 将该仓提交为本次序列启动时锁定的空仓，并立即保存到 Flash。
 */
uint8_t UpdateEmptyBay(void)
{
    uint8_t target = Sequence_GetTargetEmptyBay();

    if (target < 1U || target > BAY_COUNT) {
        LOG_ERROR("SWAP", "invalid target empty bay=%u, sequence_id=%u\r\n",
                  (unsigned int)target,
                  (unsigned int)Sequence_GetCurrentId());
        return MODBUS_RESULT_PARAM;
    }

    /* 自动恢复重跑到本步骤时，目标值不变，因此不会再次循环增加机位。 */
    if (g_swap_state.empty_bay == target) {
        StatusRegs_Update(REG_RESERVED2, target);
        need_save = 0U;
        LOG_INFO("SWAP", "empty bay already committed: value=%u\r\n",
                 (unsigned int)target);
        return MODBUS_RESULT_OK;
    }

    g_swap_state.empty_bay = target;
    StatusRegs_Update(REG_RESERVED2, target);
    SwapState_Save();
    need_save = 0U;
    LOG_INFO("SWAP", "empty bay committed and saved: value=%u\r\n",
             (unsigned int)target);
    return MODBUS_RESULT_OK;
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
