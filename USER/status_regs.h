#ifndef STATUS_REGS_H
#define STATUS_REGS_H

#include <stdint.h>

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
    REG_FAULT_CODE,           // 故障码 0x17
    REG_COMMAND_STATE,        // 命令状态 0x18
    REG_COMMAND_STEP = 0x19, // 当前步骤或最后执行步骤 0x19
    REG_COMMAND_CODE = 0x1A, // 当前或最后执行的命令码 0x1A
    REG_EMPTY_BAY = 0x1B,       // 当前空仓号 0x1B
    REG_RC_UAV_STATE = 0x1C,    // 存储0x60写入的遥控器和无人机状态
    REG_CHARGING_CURRENT = 0x1D, //当前充电电流大小
    REG_ADDR_COUNT             // 必须保持在枚举末尾，自动取最后一个寄存器地址加1
} StatusRegAddr;

// 寄存器总数，随地址枚举自动计算
#define STATUS_REG_COUNT (REG_ADDR_COUNT + 0U)

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
