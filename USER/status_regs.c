#include "status_regs.h"
#include "status_service.h"

#include "stm32f4xx.h"

#include <stddef.h>
#include <string.h>

/* 状态存储属于本模块，业务代码只能通过公开接口访问。 */
static uint16_t status_regs[STATUS_REG_COUNT];
static uint16_t status_cache[STATUS_REG_COUNT];
static uint16_t status_snapshot[STATUS_REG_COUNT];
static volatile uint8_t snapshot_active;

/* 保存进入临界区前的中断状态，避免在中断原本关闭时被错误地重新开启。 */
static uint32_t StatusRegs_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void StatusRegs_ExitCritical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

void StatusRegs_Init(void)
{
    uint32_t primask = StatusRegs_EnterCritical();

    memset(status_regs, 0, sizeof(status_regs));
    memset(status_cache, 0, sizeof(status_cache));
    memset(status_snapshot, 0, sizeof(status_snapshot));
    snapshot_active = 0U;
    StatusRegs_ExitCritical(primask);
}

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

void StatusRegs_TakeSnapshot(void)
{
    uint32_t primask = StatusRegs_EnterCritical();

    if (!snapshot_active) {
        memcpy(status_snapshot, status_regs, sizeof(status_snapshot));
        snapshot_active = 1U;
    }
    StatusRegs_ExitCritical(primask);
}

void StatusRegs_ReleaseSnapshot(void)
{
    uint32_t primask = StatusRegs_EnterCritical();

    snapshot_active = 0U;
    StatusRegs_ExitCritical(primask);
}

uint8_t StatusRegs_IsSnapshotActive(void)
{
    uint32_t primask = StatusRegs_EnterCritical();
    uint8_t active = snapshot_active;

    StatusRegs_ExitCritical(primask);
    return active;
}

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

uint16_t StatusService_GetBatteryChargeState(uint8_t battery_index)
{
    switch (battery_index) {
        case 1U: return StatusRegs_Get(REG_BAT1_CHARGE_STATE);
        case 2U: return StatusRegs_Get(REG_BAT2_CHARGE_STATE);
        case 3U: return StatusRegs_Get(REG_BAT3_CHARGE_STATE);
        default: return 0U;
    }
}

uint16_t StatusService_GetUavStatus(void)
{
    return StatusRegs_Get(REG_RESERVED4);
}

void StatusService_SetUavStatus(uint16_t value)
{
    StatusRegs_Update(REG_RESERVED4, value);
}

void StatusService_Update(StatusServiceField field, uint16_t value)
{
    switch (field) {
        case STATUS_FIELD_DOOR:
            StatusRegs_Update(REG_DOOR_STATE, value);
            break;
        case STATUS_FIELD_CENTER_ROD:
            StatusRegs_Update(REG_CENTER_ROD_STATE, value);
            break;
        case STATUS_FIELD_SWAP_MECHANISM:
            StatusRegs_Update(REG_SWAP_MECH_STATE, value);
            break;
        case STATUS_FIELD_FAULT:
            StatusRegs_Update(REG_FAULT_CODE, value);
            break;
        case STATUS_FIELD_NONE:
        default:
            break;
    }
}

void StatusService_TakeSnapshot(void)
{
    StatusRegs_TakeSnapshot();
}

void StatusService_ReleaseSnapshot(void)
{
    StatusRegs_ReleaseSnapshot();
}

void StatusService_GetExternalBatch(uint16_t start_addr, uint8_t count,
                                    uint8_t *response)
{
    StatusRegs_GetBatch(start_addr, count, response);
}
