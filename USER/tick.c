#include "tick.h"

#include "stm32f4xx.h"

static volatile uint32_t sys_tick = 0;

/* 功能：把系统毫秒计数加一；只由SysTick中断调用。 */
void Tick_Increment(void)
{
    sys_tick++;
}

/* 功能：清零毫秒计数并配置SysTick每1毫秒产生一次中断。 */
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

/* 功能：在关闭中断的保护区内读取一致的32位毫秒计数。 */
uint32_t GetTick(void)
{
    uint32_t primask;
    uint32_t tick;

    primask = __get_PRIMASK();
    __disable_irq();
    tick = sys_tick;
    if (primask == 0U) {
        __enable_irq();
    }
    return tick;
}

