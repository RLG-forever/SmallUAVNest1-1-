#include "bsp_timer.h"

/* 选择 TIM2 作为硬件定时器 */
#ifdef USE_TIM2
    #define TIM_HARD        TIM2
    #define TIM_HARD_IRQn   TIM2_IRQn
    #define TIM_HARD_RCC    RCC_APB1Periph_TIM2
#endif

/* 软件定时器数组（如果需要）*/
static SOFT_TMR s_tTmr[TMR_COUNT];

/* 硬件定时器回调函数指针 */
static void (*s_TIM_CallBack1)(void);
static void (*s_TIM_CallBack2)(void);
static void (*s_TIM_CallBack3)(void);
static void (*s_TIM_CallBack4)(void);

/* 系统运行时间（毫秒），用于 GetTick */
volatile int32_t g_iRunTime = 0;

/*
*********************************************************************************************************
*    函数名: bsp_InitSysTick
*    功能说明: 配置 SysTick 为 1ms 中断，用于软件定时器和系统时间
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitSysTick(void)
{
    /* 系统时钟频率 SystemCoreClock，例如 168MHz */
//    if (SysTick_Config(SystemCoreClock / 1000)) {
//        /* 配置失败，死循环 */
//        while(1);
//    }
}

/*
*********************************************************************************************************
*    函数名: TIM5_IRQHandler
*    功能说明: TIM5_IRQHandler 中断服务程序
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/

void TIM5_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 使能 TIM5 时钟（APB1 总线）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    
    // 计算预分频和周期：系统时钟 168MHz，APB1 定时器时钟 = 84MHz
    // 预分频 83 → 84MHz/84 = 1MHz，计数周期 1us
    // 要得到 1ms 中断，需要计数值 1000
    TIM_TimeBaseStructure.TIM_Prescaler = 83;          // 84-1? 实际上 84-1=83，计数频率 = 84MHz/84 = 1MHz
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;       // 1000 个计数 = 1ms
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    
    // 清除中断标志并使能更新中断
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    
    // 设置中断优先级（例如 1，低于串口中断）
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 启动 TIM5
    TIM_Cmd(TIM5, ENABLE);
}

void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
    {
        // 清除中断标志
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
				uint8_t i;
        // 停止定时器，等待下一帧接收开始
				g_iRunTime++;
				for (i = 0; i < TMR_COUNT; i++) {
        if (s_tTmr[i].Count > 0) {
            if (--s_tTmr[i].Count == 0) {
                s_tTmr[i].Flag = 1;
                if (s_tTmr[i].Mode == TMR_AUTO_MODE) {
                    s_tTmr[i].Count = s_tTmr[i].PreLoad;
                }
            }
        }
    }
    }
}



/*
*********************************************************************************************************
*    函数名: bsp_InitHardTimer
*    功能说明: 初始化硬件定时器（TIM2），配置为1us计数周期，自由运行
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitHardTimer(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    uint32_t uiTIMxCLK;
    
    /* 使能 TIM2 时钟 */
    RCC_APB1PeriphClockCmd(TIM_HARD_RCC, ENABLE);
    
    /* APB1 定时器时钟 = SystemCoreClock / 2 = 84MHz */
    uiTIMxCLK = SystemCoreClock / 2;
    /* 预分频器：84MHz / 84 = 1MHz，计数周期 1us */
    uint16_t usPrescaler = (uiTIMxCLK / 1000000) - 1;
    
    /* 定时器周期：16位定时器最大 0xFFFF，32位定时器可设更大，这里设为最大值 */
    uint32_t usPeriod = 0xFFFFFFFF;   // TIM2 是32位，可以很大
    #if defined (USE_TIM3) || defined (USE_TIM4)
        usPeriod = 0xFFFF;            // 16位定时器
    #endif
    
    TIM_TimeBaseStructure.TIM_Period = usPeriod;
    TIM_TimeBaseStructure.TIM_Prescaler = usPrescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM_HARD, &TIM_TimeBaseStructure);
    
    /* 使能定时器计数器 */
    TIM_Cmd(TIM_HARD, ENABLE);
    
    /* 配置中断优先级（抢占优先级 2，子优先级 0，可根据需要调整）*/
    NVIC_InitStructure.NVIC_IRQChannel = TIM_HARD_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//void bsp_InitHardTimer(void)
//{
//    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;
//    
//    // 使能 TIM4 时钟（APB1 总线）
//    RCC_APB1PeriphClockCmd(TIM_HARD_RCC, ENABLE);
//    
//    // 定时器配置：假设系统时钟 168MHz，APB1 定时器时钟 84MHz，分频 83 → 1MHz 计数频率
//    // 超时时间：3.5 字符时间 @ 9600bps ≈ 3.6ms，设置 Period = 3600 - 1
//    TIM_TimeBaseInitStructure.TIM_Period = 9999;   // 自动重装载值
//    TIM_TimeBaseInitStructure.TIM_Prescaler = 83;      // 预分频器
//    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//    TIM_TimeBaseInit(TIM_HARD, &TIM_TimeBaseInitStructure);
//    
//	
//	 TIM_Cmd(TIM_HARD, ENABLE);
//    // 清除中断标志
//    TIM_ClearITPendingBit(TIM_HARD, TIM_IT_Update);
//    TIM_ITConfig(TIM_HARD, TIM_IT_Update, ENABLE);   // 使能更新中断
//    
//    // 中断优先级配置：抢占优先级低于 UART1，高于主循环（例如抢占 1，子优先级 1）
//    NVIC_InitStructure.NVIC_IRQChannel = TIM_HARD_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//    
//    // 注意：定时器默认不启动，等待收到第一个字节后由中断启动
//   
////		TIM_Cmd(TIM4, ENABLE);
//}
/*
*********************************************************************************************************
*    函数名: bsp_StartHardTimer
*    功能说明: 启动一个硬件定时器（单次模式）
*    形    参: _CC         : 比较通道（1~4），通常用1
*              _uiTimeOut  : 超时时间（微秒），不能超过定时器最大周期
*              _pCallBack  : 超时回调函数
*    返 回 值: 无
*********************************************************************************************************
*/
void bsp_StartHardTimer(uint8_t _CC, uint32_t _uiTimeOut, void (*_pCallBack)(void))
{
    uint32_t cnt_now = TIM_GetCounter(TIM_HARD);
    uint32_t cnt_tar = cnt_now + _uiTimeOut;
    
    /* 补偿函数调用时间（可选）*/
    if (_uiTimeOut > 5) cnt_tar -= 5;
    
    if (_CC == 1) {
        s_TIM_CallBack1 = _pCallBack;
        TIM_SetCompare1(TIM_HARD, cnt_tar);
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC1);
        TIM_ITConfig(TIM_HARD, TIM_IT_CC1, ENABLE);
    } else if (_CC == 2) {
        s_TIM_CallBack2 = _pCallBack;
        TIM_SetCompare2(TIM_HARD, cnt_tar);
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC2);
        TIM_ITConfig(TIM_HARD, TIM_IT_CC2, ENABLE);
    } else if (_CC == 3) {
        s_TIM_CallBack3 = _pCallBack;
        TIM_SetCompare3(TIM_HARD, cnt_tar);
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC3);
        TIM_ITConfig(TIM_HARD, TIM_IT_CC3, ENABLE);
    } else if (_CC == 4) {
        s_TIM_CallBack4 = _pCallBack;
        TIM_SetCompare4(TIM_HARD, cnt_tar);
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC4);
        TIM_ITConfig(TIM_HARD, TIM_IT_CC4, ENABLE);
    }
}

/* 取消单次比较中断，防止已终止的业务流程收到过期定时器回调。 */
void bsp_StopHardTimer(uint8_t _CC)
{
    if (_CC == 1U) {
        TIM_ITConfig(TIM_HARD, TIM_IT_CC1, DISABLE);
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC1);
        s_TIM_CallBack1 = 0;
    } else if (_CC == 2U) {
        TIM_ITConfig(TIM_HARD, TIM_IT_CC2, DISABLE);
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC2);
        s_TIM_CallBack2 = 0;
    } else if (_CC == 3U) {
        TIM_ITConfig(TIM_HARD, TIM_IT_CC3, DISABLE);
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC3);
        s_TIM_CallBack3 = 0;
    } else if (_CC == 4U) {
        TIM_ITConfig(TIM_HARD, TIM_IT_CC4, DISABLE);
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC4);
        s_TIM_CallBack4 = 0;
    }
}

/*
*********************************************************************************************************
*    函数名: TIM2_IRQHandler
*    功能说明: TIM2 中断服务程序（需要在 stm32f4xx_it.c 中调用本函数，或者直接在此实现）
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM_HARD, TIM_IT_CC1)) {
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC1);
        TIM_ITConfig(TIM_HARD, TIM_IT_CC1, DISABLE);
        if (s_TIM_CallBack1) s_TIM_CallBack1();
    }
    if (TIM_GetITStatus(TIM_HARD, TIM_IT_CC2)) {
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC2);
        TIM_ITConfig(TIM_HARD, TIM_IT_CC2, DISABLE);
        if (s_TIM_CallBack2) s_TIM_CallBack2();
    }
    if (TIM_GetITStatus(TIM_HARD, TIM_IT_CC3)) {
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC3);
        TIM_ITConfig(TIM_HARD, TIM_IT_CC3, DISABLE);
        if (s_TIM_CallBack3) s_TIM_CallBack3();
    }
    if (TIM_GetITStatus(TIM_HARD, TIM_IT_CC4)) {
        TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC4);
        TIM_ITConfig(TIM_HARD, TIM_IT_CC4, DISABLE);
        if (s_TIM_CallBack4) s_TIM_CallBack4();
    }
}

/*
*********************************************************************************************************
*    以下为软件定时器函数（如果需要长延时，推荐使用软件定时器）
*********************************************************************************************************
*/
void bsp_StartTimer(uint8_t _id, uint32_t _period)
{
    if (_id >= TMR_COUNT) return;
    __disable_irq();
    s_tTmr[_id].Count = _period;
    s_tTmr[_id].PreLoad = _period;
    s_tTmr[_id].Flag = 0;
    s_tTmr[_id].Mode = TMR_ONCE_MODE;
    __enable_irq();
}

uint8_t bsp_CheckTimer(uint8_t _id)
{
    if (_id >= TMR_COUNT) return 0;
    if (s_tTmr[_id].Flag) {
        s_tTmr[_id].Flag = 0;
        return 1;
    }
    return 0;
}

void bsp_StopTimer(uint8_t _id)
{
    if (_id >= TMR_COUNT) return;
    __disable_irq();
    s_tTmr[_id].Count = 0;
    s_tTmr[_id].Flag = 0;
    s_tTmr[_id].Mode = TMR_ONCE_MODE;
    __enable_irq();
}

void bsp_StartAutoTimer(uint8_t _id, uint32_t _period)
{
    if (_id >= TMR_COUNT) return;
    __disable_irq();
    s_tTmr[_id].Count = _period;
    s_tTmr[_id].PreLoad = _period;
    s_tTmr[_id].Flag = 0;
    s_tTmr[_id].Mode = TMR_AUTO_MODE;
    __enable_irq();
}
