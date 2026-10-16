#include "relay.h"
#include "delay.h"
#include "stm32f4xx.h"

/* 继电器硬件引脚仅由本驱动使用。 */
#define RELAY_FORWARD_PIN GPIO_Pin_2
#define RELAY_BACKWARD_PIN GPIO_Pin_3
#define RELAY_GPIO_PORT GPIOE
#define RELAY_ALL_PINS (RELAY_FORWARD_PIN | RELAY_BACKWARD_PIN)

/* 功能：初始化继电器方向控制引脚，并默认将正反转输出全部关闭。 */
void Relay_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    gpio.GPIO_Pin = RELAY_ALL_PINS;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(RELAY_GPIO_PORT, &gpio);
    GPIO_ResetBits(RELAY_GPIO_PORT, RELAY_ALL_PINS);
}

/* 功能：先关闭两路继电器，再根据目标状态只接通正向或反向一路。 */
void Relay_Control(RelayState state)
{
    GPIO_ResetBits(RELAY_GPIO_PORT, RELAY_ALL_PINS);
    if (state == RELAY_FORWARD) {
        GPIO_SetBits(RELAY_GPIO_PORT, RELAY_FORWARD_PIN);
    } else if (state == RELAY_BACKWARD) {
        GPIO_SetBits(RELAY_GPIO_PORT, RELAY_BACKWARD_PIN);
    }
}

/* 功能：控制继电器进入正向输出状态。 */
void Relay_Forward(void)  { Relay_Control(RELAY_FORWARD); }
/* 功能：控制继电器进入反向输出状态。 */
void Relay_Backward(void) { Relay_Control(RELAY_BACKWARD); }
/* 功能：关闭继电器正反向输出。 */
void Relay_Stop(void)     { Relay_Control(RELAY_STOP); }

/* 功能：兼容旧接口，正向运行5秒后自动停止。 */
void RELAY(void)
{
    Relay_Forward();
    delay_ms(5000U);
    Relay_Stop();
}
