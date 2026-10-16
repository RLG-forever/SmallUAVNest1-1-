#include "status_regs.h"

#include "stm32f4xx.h"

#include <stddef.h>
#include <string.h>

/* 状态存储属于本模块，业务代码只能通过公开接口访问。 */
static uint16_t status_regs[STATUS_REG_COUNT];
static uint16_t status_cache[STATUS_REG_COUNT];
static uint16_t status_snapshot[STATUS_REG_COUNT];
static volatile uint8_t snapshot_active;

/* 功能：保存进入临界区前的中断状态，避免原本关闭中断时被错误开启。 */
static uint32_t StatusRegs_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/* 功能：按进入临界区前保存的中断状态恢复全局中断。 */
static void StatusRegs_ExitCritical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/* 功能：清空实时寄存器和快照，并设置上电默认舱门状态。 */
void StatusRegs_Init(void)
{
    uint32_t primask = StatusRegs_EnterCritical();

    memset(status_regs, 0, sizeof(status_regs));
    memset(status_cache, 0, sizeof(status_cache));
    memset(status_snapshot, 0, sizeof(status_snapshot));
    /* 上电默认按舱门关到位处理，使起飞清理流程能尝试开门。 */
    status_regs[REG_DOOR_STATE] = 4U;
    status_cache[REG_DOOR_STATE] = 4U;
    snapshot_active = 0U;
    StatusRegs_ExitCritical(primask);
}

/* 功能：在临界区中更新一个实时状态寄存器和对应的Modbus响应缓存。 */
void StatusRegs_Update(StatusRegAddr addr, uint16_t value)
{
    uint32_t primask;

    if ((uint16_t)addr >= STATUS_REG_COUNT) {
        return;
    }
    primask = StatusRegs_EnterCritical();
    status_regs[addr] = value;
    status_cache[addr] = value;
    StatusRegs_ExitCritical(primask);
}

/* 功能：批量更新连续的状态寄存器，同时刷新Modbus响应缓存。 */
void StatusRegs_UpdateBatch(uint16_t start_addr, const uint16_t *values,
                            uint8_t count)
{
    uint32_t primask;
    uint8_t index;

    if (values == NULL || count == 0U || start_addr >= STATUS_REG_COUNT ||
        count > (uint8_t)(STATUS_REG_COUNT - start_addr)) {
        return;
    }
    primask = StatusRegs_EnterCritical();
    for (index = 0U; index < count; index++) {
        status_regs[start_addr + index] = values[index];
        status_cache[start_addr + index] = values[index];
    }
    StatusRegs_ExitCritical(primask);
}

/* 功能：读取状态寄存器；序列快照有效时优先返回快照值。 */
uint16_t StatusRegs_Get(StatusRegAddr addr)
{
    uint32_t primask;
    uint16_t value = 0U;

    if ((uint16_t)addr >= STATUS_REG_COUNT) {
        return 0U;
    }
    primask = StatusRegs_EnterCritical();
    value = snapshot_active ? status_snapshot[addr] : status_regs[addr];
    StatusRegs_ExitCritical(primask);
    return value;
}

/* 功能：忽略序列快照，直接读取当前实时状态寄存器。 */
uint16_t StatusRegs_GetLive(StatusRegAddr addr)
{
    uint32_t primask;
    uint16_t value = 0U;

    if ((uint16_t)addr >= STATUS_REG_COUNT) {
        return 0U;
    }
    primask = StatusRegs_EnterCritical();
    value = status_regs[addr];
    StatusRegs_ExitCritical(primask);
    return value;
}

/* 功能：冻结当前实时状态，供正在执行的序列读取稳定的过程状态。 */
void StatusRegs_TakeSnapshot(void)
{
    uint32_t primask = StatusRegs_EnterCritical();

    if (!snapshot_active) {
        memcpy(status_snapshot, status_regs, sizeof(status_snapshot));
        snapshot_active = 1U;
    }
    StatusRegs_ExitCritical(primask);
}

/* 功能：释放序列状态快照，后续读取恢复为实时值。 */
void StatusRegs_ReleaseSnapshot(void)
{
    uint32_t primask = StatusRegs_EnterCritical();

    snapshot_active = 0U;
    StatusRegs_ExitCritical(primask);
}

/* 功能：查询当前是否处于序列状态快照模式。 */
uint8_t StatusRegs_IsSnapshotActive(void)
{
    uint32_t primask = StatusRegs_EnterCritical();
    uint8_t active = snapshot_active;

    StatusRegs_ExitCritical(primask);
    return active;
}

/* 功能：把连续状态寄存器按Modbus大端格式写入03响应数据区。 */
void StatusRegs_GetBatch(uint16_t start_addr, uint8_t count, uint8_t *response)
{
    uint32_t primask;
    uint8_t index;

    if (response == NULL || count == 0U || start_addr >= STATUS_REG_COUNT ||
        count > (uint8_t)(STATUS_REG_COUNT - start_addr)) {
        return;
    }
    primask = StatusRegs_EnterCritical();
    for (index = 0U; index < count; index++) {
        uint16_t value = status_cache[start_addr + index];

        response[index * 2U] = (uint8_t)(value >> 8);
        response[index * 2U + 1U] = (uint8_t)value;
    }
    StatusRegs_ExitCritical(primask);
}

/*
 * 功能：根据三块电池的充电状态判断无人机内电池编号。
 * 返回：恰好一块电池处于未充电状态时返回其1~3编号，否则返回0。
 */
uint8_t GetBatteryInUAV(void)
{
    uint16_t bat1_charging = StatusRegs_Get(REG_BAT1_CHARGE_STATE);
    uint16_t bat2_charging = StatusRegs_Get(REG_BAT2_CHARGE_STATE);
    uint16_t bat3_charging = StatusRegs_Get(REG_BAT3_CHARGE_STATE);
    uint8_t not_charging_count = 0U;
    uint8_t battery_id = 0U;

    if (bat1_charging == 1U) {
        not_charging_count++;
        battery_id = 1U;
    }
    if (bat2_charging == 1U) {
        not_charging_count++;
        battery_id = 2U;
    }
    if (bat3_charging == 1U) {
        not_charging_count++;
        battery_id = 3U;
    }
    return not_charging_count == 1U ? battery_id : 0U;
}
