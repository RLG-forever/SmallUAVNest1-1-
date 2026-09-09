#ifndef STATUS_REGS_H
#define STATUS_REGS_H

#include <stdint.h>

// 寄存器总数（0x00 ~ 0x1C 共29个）
#define STATUS_REG_COUNT  29U

// 寄存器地址枚举
typedef enum {
    REG_RAINFALL = 0x00,      // 降雨量（mm）
    REG_WIND_SPEED,           // 风速（m/s）
    REG_AMBIENT_TEMP,         // 环境温度（0.1℃）
    REG_CABIN_TEMP,           // 舱内温度（0.1℃）
    REG_CABIN_HUMIDITY,       // 舱内湿度（%）
    REG_BAT1_SN,              // 电池1序号
    REG_BAT1_CHARGE_STATE,    // 电池1充电状态
    REG_BAT1_POWER,           // 电池1电量（%）
    REG_BAT1_TEMP,            // 电池1温度（0.1℃）
    REG_BAT2_SN,              // 电池2序号
    REG_BAT2_CHARGE_STATE,    // 电池2充电状态
    REG_BAT2_POWER,           // 电池2电量
    REG_BAT2_TEMP,            // 电池2温度
    REG_BAT3_SN,              // 电池3序号
    REG_BAT3_CHARGE_STATE,    // 电池3充电状态
    REG_BAT3_POWER,           // 电池3电量
    REG_BAT3_TEMP,            // 电池3温度
    REG_DOOR_STATE,           // 舱门状态（1打开中2开到位3关门中4关到位）
    REG_LIFT_STATE,           // 升降杆状态（1上升中2升到位3下降中4降到位）
    REG_AC_STATE,             // 空调状态（1制冷2制热3待机）
    REG_SWAP_MECH_STATE,      // 换电机构状态（预留）
    REG_CENTER_ROD_STATE,     // 居中杆状态（1收紧2释放）
    REG_AIRCRAFT_PRESENT,     // 飞机是否在机巢（1是2否）
    REG_FAULT_CODE,           // 故障码
    REG_RESERVED1,            // 备用
    REG_RESERVED2,            // 备用
		REG_RESERVED3,						// 备用
		REG_RESERVED4,						// 备用  存储0x60的值
    REG_COMMAND_STEP,         // 当前步骤或最后失败步骤
} StatusRegAddr;

/* 复用现有备用寄存器，保持原有寄存器地址不变。 */
#define REG_COMMAND_STATE REG_RESERVED1
#define REG_COMMAND_CODE  REG_RESERVED3

typedef enum {
    COMMAND_STATE_IDLE = 0U,
    COMMAND_STATE_EXECUTING,
    COMMAND_STATE_SUCCESS,
    COMMAND_STATE_FAILED,
    COMMAND_STATE_PAUSED,
    COMMAND_STATE_CANCELLED
} CommandState;

#define STATUS_REG_NONE ((StatusRegAddr)0xFFU)

// 初始化状态寄存器
void StatusRegs_Init(void);
// 更新某个寄存器的值（带临界区保护）
void StatusRegs_Update(StatusRegAddr addr, uint16_t value);
// 批量更新连续寄存器
void StatusRegs_UpdateBatch(uint16_t start_addr, const uint16_t *values, uint8_t count);
// 获取寄存器值（供从站读取）
uint16_t StatusRegs_Get(StatusRegAddr addr);
// 获取寄存器实时值（忽略序列快照，用于0x60等运行中动态状态）
uint16_t StatusRegs_GetLive(StatusRegAddr addr);
void StatusRegs_TakeSnapshot(void);
void StatusRegs_ReleaseSnapshot(void);
uint8_t StatusRegs_IsSnapshotActive(void);
// 批量读取寄存器值（供03响应快速使用）
void StatusRegs_GetBatch(uint16_t start_addr, uint8_t count, uint8_t *resp);

uint8_t GetBatteryInUAV(void);

#endif
