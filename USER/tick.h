#ifndef TICK_H
#define TICK_H

#include <stdint.h>

// 初始化系统滴答（例如配置SysTick每1ms中断一次）
void Tick_Init(void);
// 获取当前毫秒计数
uint32_t GetTick(void);
// 滴答中断服务函数中调用此函数（每1ms一次）
void Tick_Increment(void);  // 供中断调用
#endif