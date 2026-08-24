#include "project.h"
#include <string.h>

// 定义全局数组
uint16_t status_regs[STATUS_REG_COUNT];
uint16_t status_cache[STATUS_REG_COUNT];     // 缓存值（供03读取）
static volatile uint8_t cache_valid = 0;            // 缓存是否有效
// 定义快照数组和标志
static uint16_t status_snapshot[STATUS_REG_COUNT];
static volatile uint8_t snapshot_active = 0;


void StatusRegs_Init(void)
{
    ENTER_CRITICAL();
    memset(status_regs, 0, sizeof(status_regs));
    EXIT_CRITICAL();
}

void StatusRegs_Update(StatusRegAddr addr, uint16_t value)
{
    if (addr < STATUS_REG_COUNT) {
        ENTER_CRITICAL();
        status_regs[addr] = value;
				status_cache[addr] = value;   // 同步更新缓存
        cache_valid = 1;
        EXIT_CRITICAL();
    }
}

void StatusRegs_UpdateBatch(uint16_t start_addr, const uint16_t *values, uint8_t count)
{
    if (start_addr + count <= STATUS_REG_COUNT) {
        ENTER_CRITICAL();
        for (uint8_t i = 0; i < count; i++) {
            status_regs[start_addr + i] = values[i];
						status_cache[start_addr + i] = values[i];
        }
				cache_valid = 1;
        EXIT_CRITICAL();
    }
}

uint16_t StatusRegs_Get(StatusRegAddr addr)
{
    uint16_t val = 0;
    if (addr < STATUS_REG_COUNT) {
        ENTER_CRITICAL();
        if (snapshot_active) {
            val = status_snapshot[addr];   // 快照模式：返回序列开始时的值
        } else {
            val = status_regs[addr];        // 正常模式：返回实时值
        }
        EXIT_CRITICAL();
    }
    return val;
}

// 拍摄当前状态快照（复制真实值到快照数组）
void StatusRegs_TakeSnapshot(void)
{
    if (!snapshot_active) {
        ENTER_CRITICAL();
        memcpy(status_snapshot, status_regs, sizeof(status_snapshot));
        snapshot_active = 1;
        EXIT_CRITICAL();
    }
}

// 退出快照模式，恢复正常读取
void StatusRegs_ReleaseSnapshot(void)
{
    if (snapshot_active) {
        ENTER_CRITICAL();
        snapshot_active = 0;
        EXIT_CRITICAL();
    }
}

// 查询是否处于快照模式
uint8_t StatusRegs_IsSnapshotActive(void)
{
    uint8_t active;
    ENTER_CRITICAL();
    active = snapshot_active;
    EXIT_CRITICAL();
    return active;
}

// 批量读取缓存值到字节数组（用于03响应）
void StatusRegs_GetBatch(uint16_t start_addr, uint8_t count, uint8_t *resp)
{
    if (start_addr + count > STATUS_REG_COUNT) return;
    ENTER_CRITICAL();
    for (uint8_t i = 0; i < count; i++) {
        uint16_t val = status_cache[start_addr + i];
        resp[i*2]   = (val >> 8) & 0xFF;
        resp[i*2+1] = val & 0xFF;
    }
    EXIT_CRITICAL();
}

/**
 * @brief 判断哪块电池在无人机内（即不在充电的电池）
 * @return 电池编号：1,2,3；0表示无电池在无人机内
 */
uint8_t GetBatteryInUAV(void)
{
    // 读取三块电池的充电状态（0=充电中，1=未充电）
    uint16_t bat1_charging = StatusRegs_Get(REG_BAT1_CHARGE_STATE);
    uint16_t bat2_charging = StatusRegs_Get(REG_BAT2_CHARGE_STATE);
    uint16_t bat3_charging = StatusRegs_Get(REG_BAT3_CHARGE_STATE);

    // 统计不在充电的电池数量
    uint8_t not_charging_count = 0;
    uint8_t battery_id = 0;

    if (bat1_charging == 1) {
        not_charging_count++;
        battery_id = 1;
    }
    if (bat2_charging == 1) {
        not_charging_count++;
        battery_id = 2;
    }
    if (bat3_charging == 1) {
        not_charging_count++;
        battery_id = 3;
    }

    // 如果只有一块电池不在充电，则它就在无人机里；否则（0或>1）认为无电池或异常，返回0
    if (not_charging_count == 1) {
        return battery_id;
    } else {
        return 0;   // 无电池在无人机内
    }
}