#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include "stm32f4xx.h"
/* 选择硬件定时器：使用 TIM2（32位定时器，可支持较长定时）*/
#define USE_TIM2
//#define USE_TIM3
//#define USE_TIM4
//#define USE_TIM5

/* 软件定时器数量（如果您还需要软件定时器）*/
#define TMR_COUNT    10

/* 定时器模式 */
#define TMR_ONCE_MODE   0   // 单次模式
#define TMR_AUTO_MODE   1   // 自动重装模式

/* 软件定时器结构体（如果需要）*/
typedef struct {
    volatile uint8_t  Mode;      // 模式
    volatile uint8_t  Flag;      // 超时标志
    volatile uint32_t Count;     // 当前计数值
    volatile uint32_t PreLoad;   // 重装载值
} SOFT_TMR;

/* 硬件定时器初始化 */
void bsp_InitHardTimer(void);

/* 启动一个硬件定时器（单次，微秒级） 
   _CC       : 捕获比较通道（1~4），通常使用1
   _uiTimeOut: 超时时间，单位微秒（最大约65ms for 16位定时器，32位定时器可更大）
   _pCallBack: 超时回调函数
*/
void bsp_StartHardTimer(uint8_t _CC, uint32_t _uiTimeOut, void (*_pCallBack)(void));
/* 取消指定比较通道的单次硬件定时器。 */
void bsp_StopHardTimer(uint8_t _CC);

/* 系统滴答初始化（1ms中断，用于软件定时器和GetTick）*/
void bsp_InitSysTick(void);

/* 软件定时器相关函数（如果需要）*/
void bsp_StartTimer(uint8_t _id, uint32_t _period);
uint8_t bsp_CheckTimer(uint8_t _id);
void bsp_StopTimer(uint8_t _id);
void bsp_StartAutoTimer(uint8_t _id, uint32_t _period);
void TIM5_Init(void);

#endif
