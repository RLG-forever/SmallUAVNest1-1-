#include "relay.h"
#include "delay.h"
#include "stm32f4xx.h"

/* 继电器硬件引脚仅由本驱动使用。 */
#define RELAY_FORWARD_PIN GPIO_Pin_2
#define RELAY_BACKWARD_PIN GPIO_Pin_3
#define RELAY_GPIO_PORT GPIOE
#define RELAY_ALL_PINS (RELAY_FORWARD_PIN | RELAY_BACKWARD_PIN)

/* 继电器方向由本地GPIO直接控制，本驱动不等待Modbus通信。 */
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

void Relay_Control(RelayState state)
{
    GPIO_ResetBits(RELAY_GPIO_PORT, RELAY_ALL_PINS);
    if (state == RELAY_FORWARD) {
        GPIO_SetBits(RELAY_GPIO_PORT, RELAY_FORWARD_PIN);
    } else if (state == RELAY_BACKWARD) {
        GPIO_SetBits(RELAY_GPIO_PORT, RELAY_BACKWARD_PIN);
    }
}

void Relay_Forward(void)  { Relay_Control(RELAY_FORWARD); }
void Relay_Backward(void) { Relay_Control(RELAY_BACKWARD); }
void Relay_Stop(void)     { Relay_Control(RELAY_STOP); }

void RELAY(void)
{
    Relay_Forward();
    delay_ms(5000U);
    Relay_Stop();
}
