#include "project.h"

static volatile uint32_t sys_tick = 0;

void Tick_Increment(void)
{
    sys_tick++;
}

void Tick_Init(void)
{
		sys_tick = 0;
		// 配置 SysTick：时钟源为内核时钟（通常168MHz），重载值 = 时钟频率 / 1000 - 1
    // 使能中断，使能定时器
    if (SysTick_Config(SystemCoreClock / 1000)) {
        // 配置失败处理
        while (1);
    }
}

uint32_t GetTick(void)
{
    uint32_t tick;
    ENTER_CRITICAL();
    tick = sys_tick;
    EXIT_CRITICAL();
    return tick;
}

