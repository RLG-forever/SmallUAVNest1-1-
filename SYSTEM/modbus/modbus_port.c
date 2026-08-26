#include "stm32f4xx.h"
#include "modbus_port.h"
#include <string.h>

#define MODBUS_PORT_TX_BUFFER_SIZE 256U
#define MODBUS_BITS_PER_CHAR       10U
#define MODBUS_MASTER_DIR_GPIO     GPIOC
#define MODBUS_MASTER_DIR_PIN      GPIO_Pin_0
#define MODBUS_SLAVE_DIR_GPIO      GPIOC
#define MODBUS_SLAVE_DIR_PIN       GPIO_Pin_1

/* 本文件是Modbus USART/TIM中断的唯一归属处；协议层只能通过回调与其交互。 */
/* USART 发送上下文，主站和从站各维护一份，用于保存当前待发送帧及发送进度。 */
typedef struct {
    uint8_t data[MODBUS_PORT_TX_BUFFER_SIZE]; /* 待发送的数据缓冲区。 */
    volatile uint16_t length;                 /* 当前待发送帧的总字节数。 */
    volatile uint16_t position;               /* 下一个待发送字节在缓冲区中的位置。 */
    volatile uint8_t active;                  /* 发送进行中标志，1 表示当前上下文正在发送。 */
} ModbusPortTxContext;

static ModbusPortTxContext master_tx;
static ModbusPortTxContext slave_tx;
static ModbusPortRxByteCallback master_rx_callback;
static ModbusPortFrameEndCallback master_frame_end_callback;
static ModbusPortTxDoneCallback master_tx_done_callback;
static ModbusPortRxByteCallback slave_rx_callback;
static ModbusPortFrameEndCallback slave_frame_end_callback;

/* 总线静默达到3.5个字符时间时，判定一帧RTU报文结束。 */
uint32_t ModbusPort_FrameT35Us(uint32_t baudrate)
{
    uint32_t timeout_us;

    if (baudrate == 0U) {
        baudrate = 9600U;
    }
    /* Modbus RTU 规定波特率高于 19200 时，帧间隔固定使用 1.75 ms。 */
    if (baudrate > 19200U) {
        return 1750U;
    }
    timeout_us = (35UL * MODBUS_BITS_PER_CHAR * 100000UL + baudrate - 1UL) / baudrate;
    if (timeout_us == 0U) {
        timeout_us = 1U;
    } else if (timeout_us > 65536U) {
        timeout_us = 65536U;
    }
    return timeout_us;
}

void ModbusPort_SetMasterCallbacks(ModbusPortRxByteCallback rx_callback,
                                   ModbusPortFrameEndCallback frame_end_callback,
                                   ModbusPortTxDoneCallback tx_done_callback)
{
    master_rx_callback = rx_callback;
    master_frame_end_callback = frame_end_callback;
    master_tx_done_callback = tx_done_callback;
}

void ModbusPort_SetSlaveCallbacks(ModbusPortRxByteCallback rx_callback,
                                  ModbusPortFrameEndCallback frame_end_callback)
{
    slave_rx_callback = rx_callback;
    slave_frame_end_callback = frame_end_callback;
}

void ModbusPort_InitMaster(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate = baudrate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);
    USART_ClearFlag(USART1, USART_FLAG_TC);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    gpio.GPIO_Pin = MODBUS_MASTER_DIR_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(MODBUS_MASTER_DIR_GPIO, &gpio);
    GPIO_ResetBits(MODBUS_MASTER_DIR_GPIO, MODBUS_MASTER_DIR_PIN);

    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void ModbusPort_InitSlave(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate = baudrate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &usart);
    USART_Cmd(USART2, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    gpio.GPIO_Pin = MODBUS_SLAVE_DIR_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(MODBUS_SLAVE_DIR_GPIO, &gpio);
    GPIO_ResetBits(MODBUS_SLAVE_DIR_GPIO, MODBUS_SLAVE_DIR_PIN);

    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void ModbusPort_InitSlaveFrameTimer(uint32_t baudrate)
{
    TIM_TimeBaseInitTypeDef timer;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    timer.TIM_Period = ModbusPort_FrameT35Us(baudrate) - 1U;
    timer.TIM_Prescaler = 83U;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &timer);
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, DISABLE);

    nvic.NVIC_IRQChannel = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void ModbusPort_InitMasterFrameTimer(uint32_t baudrate)
{
    TIM_TimeBaseInitTypeDef timer;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    timer.TIM_Period = ModbusPort_FrameT35Us(baudrate) - 1U;
    timer.TIM_Prescaler = 83U;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &timer);
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, DISABLE);

    nvic.NVIC_IRQChannel = TIM4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

/* 发送前置位DE；仅在TC确认最后一个停止位已离开发送移位寄存器后才释放DE。 */
uint8_t ModbusPort_MasterSend(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U || len > MODBUS_PORT_TX_BUFFER_SIZE || master_tx.active) {
        return 1U;
    }
    memcpy(master_tx.data, data, len);
    master_tx.length = len;
    master_tx.position = 0U;
    master_tx.active = 1U;
    GPIO_SetBits(MODBUS_MASTER_DIR_GPIO, MODBUS_MASTER_DIR_PIN);
    USART_ClearFlag(USART1, USART_FLAG_TC);
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    return 0U;
}

uint8_t ModbusPort_SlaveSend(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U || len > MODBUS_PORT_TX_BUFFER_SIZE || slave_tx.active) {
        return 1U;
    }
    memcpy(slave_tx.data, data, len);
    slave_tx.length = len;
    slave_tx.position = 0U;
    slave_tx.active = 1U;
    GPIO_SetBits(MODBUS_SLAVE_DIR_GPIO, MODBUS_SLAVE_DIR_PIN);
    USART_ClearFlag(USART2, USART_FLAG_TC);
    USART_ITConfig(USART2, USART_IT_TC, DISABLE);
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    return 0U;
}

void ModbusPort_MasterCancel(void)
{
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
    GPIO_ResetBits(MODBUS_MASTER_DIR_GPIO, MODBUS_MASTER_DIR_PIN);
    master_tx.active = 0U;
    master_tx.length = 0U;
    master_tx.position = 0U;
    ModbusPort_MasterStopFrameTimer();
}

void ModbusPort_MasterRestartFrameTimer(void)
{
    TIM_Cmd(TIM4, DISABLE);
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    TIM_SetCounter(TIM4, 0U);
    TIM_Cmd(TIM4, ENABLE);
}

void ModbusPort_MasterStopFrameTimer(void)
{
    TIM_Cmd(TIM4, DISABLE);
    TIM_SetCounter(TIM4, 0U);
}

void ModbusPort_SlaveRestartFrameTimer(void)
{
    TIM_Cmd(TIM3, DISABLE);
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_SetCounter(TIM3, 0U);
    TIM_Cmd(TIM3, ENABLE);
}

void ModbusPort_SlaveStopFrameTimer(void)
{
    TIM_Cmd(TIM3, DISABLE);
    TIM_SetCounter(TIM3, 0U);
}

/* 中断只负责搬运字节和发布事件；帧解析在主从站任务上下文完成，业务代码不在中断中运行。 */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
        if (master_tx.position < master_tx.length) {
            USART_SendData(USART1, master_tx.data[master_tx.position++]);
        }
        if (master_tx.position >= master_tx.length) {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
            USART_ITConfig(USART1, USART_IT_TC, ENABLE);
        }
    }
    if (USART_GetITStatus(USART1, USART_IT_TC) != RESET) {
        USART_ClearITPendingBit(USART1, USART_IT_TC);
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
        GPIO_ResetBits(MODBUS_MASTER_DIR_GPIO, MODBUS_MASTER_DIR_PIN);
        master_tx.active = 0U;
        if (master_tx_done_callback != NULL) {
            master_tx_done_callback();
        }
    }
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET) {
        volatile uint16_t status = USART1->SR;
        status = USART1->DR;
        (void)status;
        ModbusPort_MasterStopFrameTimer();
    }
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t data = (uint8_t)USART_ReceiveData(USART1);
        if (master_rx_callback != NULL) {
            master_rx_callback(data);
        }
        ModbusPort_MasterRestartFrameTimer();
    }
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
        if (slave_tx.position < slave_tx.length) {
            USART_SendData(USART2, slave_tx.data[slave_tx.position++]);
        }
        if (slave_tx.position >= slave_tx.length) {
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
            USART_ITConfig(USART2, USART_IT_TC, ENABLE);
        }
    }
    if (USART_GetITStatus(USART2, USART_IT_TC) != RESET) {
        USART_ClearITPendingBit(USART2, USART_IT_TC);
        USART_ITConfig(USART2, USART_IT_TC, DISABLE);
        GPIO_ResetBits(MODBUS_SLAVE_DIR_GPIO, MODBUS_SLAVE_DIR_PIN);
        slave_tx.active = 0U;
    }
    if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET) {
        volatile uint16_t status = USART2->SR;
        status = USART2->DR;
        (void)status;
        ModbusPort_SlaveStopFrameTimer();
    }
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t data = (uint8_t)USART_ReceiveData(USART2);
        if (slave_rx_callback != NULL) {
            slave_rx_callback(data);
        }
    }
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        TIM_Cmd(TIM3, DISABLE);
        if (slave_frame_end_callback != NULL) {
            slave_frame_end_callback();
        }
    }
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        TIM_Cmd(TIM4, DISABLE);
        if (master_frame_end_callback != NULL) {
            master_frame_end_callback();
        }
    }
}
