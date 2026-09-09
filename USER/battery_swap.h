#ifndef __BATTERY_SWAP_H
#define __BATTERY_SWAP_H

#include <stdint.h>

/* 电池仓数量 */
#define BAY_COUNT           3

/* Flash存储地址（使用FLASH末尾区域） */
#define FLASH_SIZE      (16UL * 1024UL * 1024UL)   // 16M 字节
/* 存储地址直接使用宏计算 */
#define SWAP_STATE_ADDR (FLASH_SIZE - 512)

/*
 * 电机位置旧格式保留在倒数第二个4KB扇区，用于升级兼容。
 * 新格式在其前面使用16个4KB扇区作为追加式循环日志，避免每次保存都擦除同一扇区。
 */
#define MOTOR_POSITION_STATE_ADDR (FLASH_SIZE - 8192UL)
#define MOTOR_POSITION_JOURNAL_SECTOR_SIZE 4096UL
#define MOTOR_POSITION_JOURNAL_SECTOR_COUNT 16UL
#define MOTOR_POSITION_JOURNAL_SIZE \
    (MOTOR_POSITION_JOURNAL_SECTOR_SIZE * MOTOR_POSITION_JOURNAL_SECTOR_COUNT)
#define MOTOR_POSITION_JOURNAL_ADDR \
    (MOTOR_POSITION_STATE_ADDR - MOTOR_POSITION_JOURNAL_SIZE)
#define MOTOR_POSITION_STATE_MAGIC 0x4D503738UL
#define MOTOR_POSITION_STATE_VERSION 2U
#define MOTOR_POSITION_STORE_FIRST_SLAVE 5U
#define MOTOR_POSITION_STORE_COUNT 8U
#define MOTOR_POSITION_STORE_VALID_MASK 0x00FFU

/* 状态结构体 */
typedef struct {
    uint8_t empty_bay;      // 当前空仓编号 (1~3)
//    uint8_t magic;          // 校验字节，固定为0x5A
} Swapstate;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t valid_mask;
    int32_t motor_positions[MOTOR_POSITION_STORE_COUNT];
    uint32_t checksum;
} MotorPositionState;

/* 函数声明 */
void SwapState_Init(void);              // 初始化，读取Flash或设置默认
void SwapState_Save(void);              // 保存当前状态到Flash
uint8_t SwapState_GetEmptyBay(void);    // 获取当前空仓号
void SwapState_SetEmptyBay(uint8_t bay); // 设置空仓号并保存
void BatterySwap_Perform(void);         // 执行一次换电操作（由外部触发）
/* 按当前落地序列锁定的目标机位提交状态，并立即保存到 Flash。 */
uint8_t UpdateEmptyBay(void);
void SwapState_TrySave(void);
void MotorPositionStore_Init(void);
uint8_t MotorPositionStore_Save(int32_t motor7_position,
                                int32_t motor8_position);
uint8_t MotorPositionStore_Get(int32_t *motor7_position,
                               int32_t *motor8_position);
uint8_t MotorPositionStore_GetAll(
    int32_t positions[MOTOR_POSITION_STORE_COUNT], uint16_t *valid_mask);
uint8_t MotorPositionStore_UpdateBatch(const uint8_t *slave_addrs,
                                       const int32_t *positions,
                                       uint8_t count);
/* 在电机启动前预先擦好下一个循环日志扇区，运动中只做小块页编程。 */
uint8_t MotorPositionStore_PrepareJournal(void);
uint8_t MotorPositionStore_BeginMotion(void);
uint8_t MotorPositionStore_EndMotion(void);
uint8_t MotorPositionStore_WasMotionInterrupted(void);

#endif
