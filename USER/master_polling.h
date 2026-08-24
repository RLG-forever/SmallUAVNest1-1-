#ifndef MASTER_POLLING_H
#define MASTER_POLLING_H

#include <stdint.h>

void MasterPolling_Init(void);   // 初始化主站轮询
void MasterPolling_Task(void);   // 主站轮询任务，周期性调用，例如每100ms
// 获取主站总线忙状态（用于从站等待）
uint8_t MasterPolling_IsBusy(void);
// 占用主站总线（从站写命令前调用）
void MasterBusy_Acquire(void);
// 释放主站总线（从站写命令后调用）
void MasterBusy_Release(void);

#endif