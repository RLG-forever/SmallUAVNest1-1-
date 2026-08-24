#include "project.h"
#include <stdint.h>   // 提供uint8_t/uint16_t等固定宽度整数类型
#include <string.h>   // 提供memset函数声明


// 全局变量定义
u32 RS485_Baudrate = 9600; //通讯波特率
u8 RS485_Parity = 0; //0无校验；1奇校验；2偶校验
u8 SLAVE_ADDR = 0x22;//从机地址
u16 RS485_Frame_Distance = 10;//数据帧最小间隔（ms),超过此时间则认为是下一帧
u8 ParaSaveFlag, IDSaveFlag;

//u8 StaID[30];
//u16 BackSend[20];
u8 RS485_RX_BUFF[2048];//接收缓冲区2048字节
volatile  u16 RS485_RX_CNT = 0; //接收计数器
u8 RS485_TX_BUFF[2048];//发送缓冲区
u16 RS485_TX_CNT = 0; //发送计数器
uint16_t RX_LEN = 0;
ModbusMasterState master_state = MASTER_IDLE;
uint16_t timeout_cnt = 0;          // 超时计数器
volatile  u16 RS485_FrameFlag = 0;
volatile uint16_t Master_RX_CNT = 0;
volatile uint16_t Master_RX_LEN = 0;
volatile uint8_t Master_FrameFlag = 0;
uint8_t Master_RX_BUFF[2048];
volatile uint32_t Master_LastRxTime = 0;

// 用于同步的标志
static volatile uint8_t g_06_wait_done = 0;
static volatile uint8_t g_06_success = 0;
// 报警轮询状态机变量
uint32_t last_alarm_poll_time = 0;
uint8_t alarm_poll_active = 0;
uint8_t alarm_motor_index = 0;
volatile uint8_t alarm_substep = 0;  // 0:读报警, 1:清除报警

// 定时器回调函数（超时时调用）
static void Modbus06_TimeoutCallback(void)
{
    g_06_wait_done = 1;
    g_06_success = 0;   // 超时失败
}


/* 外部硬件定时器控制函数（需要在您的代码中实现） */
extern void StartFrameTimeout(void);    // 启动帧超时定时器（单次，3.5字符时间）
extern void StopFrameTimeout(void);     // 停止帧超时定时器

/* 发送数据（通过 RS485 串口） */
extern void RS485_SlaveSendData(uint8_t *buf, uint16_t len);

/* CRC16 计算（Modbus） */
extern uint16_t Modbus_CRC16(uint8_t *buf, uint16_t len);

/* 字节序转换 */
static uint16_t BEBufToUint16(uint8_t *buf)
{
    return (buf[0] << 8) | buf[1];
}

extern u8 uFprint;

void CnfgrTimer3(void);


//CRC校验 自己后面添加的
//void Modbus_10_WriteMultiReg(void);
//void Modbus_03_Solve(void);
const u8 auchCRCHi[] =
{
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;


const u8 auchCRCLo[] =
{
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
} ;


u16 CRC_Compute(u8 *puchMsg, u16 usDataLen)
{
    u8 uchCRCHi = 0xFF ;
    u8 uchCRCLo = 0xFF ;
    u32 uIndex ;

    while(usDataLen--)
    {
        uIndex = uchCRCHi ^ *puchMsg++ ;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
        uchCRCLo = auchCRCLo[uIndex] ;
    }

    return ((uchCRCHi << 8)  | (uchCRCLo)) ;
}


void RS485_Init(unsigned long bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
//		TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;

	    // 1. 使能时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟

		//串口1对应引脚复用映射
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
			//串口2对应引脚复用映射
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2
		
		//USART1端口配置
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10
		
		
		//USART2端口配置
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;//GPIOA2与GPIOA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		//USART1 初始化设置
		USART_InitStructure.USART_BaudRate = bound;//波特率设置
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
		USART_Init(USART1, &USART_InitStructure); //初始化串口1
		USART_Cmd(USART1, ENABLE);  //使能串口1
		USART_ClearFlag(USART1, USART_FLAG_TC);
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
		
    // USART2 初始化设置
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
		// 使能 USART2 并开启接收中断
    USART_Cmd(USART2, ENABLE);
    USART_ClearFlag(USART2, USART_FLAG_TC);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
		//Usart1 NVIC 配置
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级1
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级1
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
		
		// USART2 中断优先级配置（从站中断）
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;   // 从站优先级可略低于主站
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		// 主站方向引脚（PC0）
		RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE);
		GPIO_InitStructure.GPIO_Pin = USART_RE_MASTER_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;                           
		GPIO_Init(USART_MASTER_RE, &GPIO_InitStructure);
		GPIO_ResetBits(USART_MASTER_RE, USART_RE_MASTER_PIN);
		
	  // 从站方向引脚（PC1）
    GPIO_InitStructure.GPIO_Pin = USART_RE_SLAVE_PIN;
    GPIO_Init(USART_SLAVE_RE, &GPIO_InitStructure);
    GPIO_ResetBits(USART_SLAVE_RE, USART_RE_SLAVE_PIN);   // 初始为接收模式
}

//继电器IO口初始化
void Relay_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 1. 使能GPIOE时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    // 2. 配置PE2, PE3为推挽输出
    GPIO_InitStructure.GPIO_Pin   = RELAY_ALL_PINS;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;      // 输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // 继电器切换速度要求不高，50MHz足够
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;   // 无上下拉（或根据硬件设计选择下拉）
    GPIO_Init(RELAY_GPIO_PORT, &GPIO_InitStructure);

    // 3. 初始状态：两个继电器均断开（输出低电平，假设高电平吸合）
    //    如果你的继电器模块是低电平触发，这里改为 GPIO_SetBits
    GPIO_ResetBits(RELAY_GPIO_PORT, RELAY_ALL_PINS);
}


//继电器控制函数
void Relay_Control(RelayState state)
{
    // 先全部断开，形成硬件互锁
    GPIO_ResetBits(RELAY_GPIO_PORT, RELAY_ALL_PINS);

    // 根据指令吸合对应继电器
    switch(state)
    {
        case RELAY_FORWARD:
            GPIO_SetBits(RELAY_GPIO_PORT, RELAY_FORWARD_PIN);
            break;

        case RELAY_BACKWARD:
            GPIO_SetBits(RELAY_GPIO_PORT, RELAY_BACKWARD_PIN);
            break;

        case RELAY_STOP:
        default:
            // 保持全部断开
            break;
    }
}

// 以下为简化调用接口
void Relay_Forward(void)  { Relay_Control(RELAY_FORWARD); }
void Relay_Backward(void) { Relay_Control(RELAY_BACKWARD); }
void Relay_Stop(void)     { Relay_Control(RELAY_STOP); }


void RELAY(void) 
{
     Relay_Forward();
		 delay_ms(5000);
		 Relay_Stop();
}

//使能串口
//定时器7初始化
void CnfgrTimer3(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //TIM7时钟使能

    //TIM7初始化设置
    TIM_TimeBaseStructure.TIM_Period = RS485_Frame_Distance*999; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = 83; //设置用来作为TIMx时钟频率除数的预分频值 设置计数频率为10kHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //TIM7 允许更新中断

    //TIM7中断分组配置
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级2级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

}

/**
 * @brief 主站（USART1）发送数据
 * @param buff 数据缓冲区
 * @param len  数据长度（字节）
 */

void RS485_MasterSendData(uint8_t *buff, uint16_t len)
{
    // 1. 切换为发送模式
    GPIO_SetBits(USART_MASTER_RE, USART_RE_MASTER_PIN);
    
    // 2. 极短延时，等待 RS485 芯片物理电平切换稳定（非常关键）
    for(volatile int i=0; i<100; i++); 

    // 3. 【关键】：发送前强制清除 TC 标志位，防止被历史空闲状态干扰
    USART_ClearFlag(USART1, USART_FLAG_TC);

    // 4. 发送数据
    while (len--)
    {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *buff++);
    }

    // 5. 必须等待所有数据真正从移位寄存器打出到总线上
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    // 6. 切换回接收模式
    GPIO_ResetBits(USART_MASTER_RE, USART_RE_MASTER_PIN);
}
//void RS485_MasterSendData(uint8_t *buff, uint16_t len)
//{
//    // 切换为发送模式（拉高方向引脚）
//    GPIO_SetBits(USART_MASTER_RE, USART_RE_MASTER_PIN);

//    // 等待上一次发送完成（TC标志）
//    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

//    // 发送数据
//    while (len--)
//    {
//        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
//        USART_SendData(USART1, *buff++);
//    }

//    // 等待所有数据发送完成
//    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
////		delay_ms(1); 

//    // 切换回接收模式（拉低方向引脚）
//    GPIO_ResetBits(USART_MASTER_RE, USART_RE_MASTER_PIN);
//}


//void RS485_MasterSendData(uint8_t *buff, uint16_t len)
//{
//    printf("进入RS485发送\r\n");

//    GPIO_SetBits(USART_MASTER_RE, USART_RE_MASTER_PIN);


//    printf("等待TC前\r\n");

//    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);


//    printf("开始发送\r\n");


//    while (len--)
//    {
//        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

//        USART_SendData(USART1, *buff++);
//    }


//    printf("等待发送完成\r\n");


//    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);


//    printf("切换接收\r\n");


//    GPIO_ResetBits(USART_MASTER_RE, USART_RE_MASTER_PIN);
//}

/**
 * @brief 从站（USART2）发送数据
 * @param buff 数据缓冲区
 * @param len  数据长度（字节）
 */
void RS485_SlaveSendData(uint8_t *buff, uint16_t len)
{
    // 切换为发送模式
    GPIO_SetBits(USART_SLAVE_RE, USART_RE_SLAVE_PIN);

    // 等待上一次发送完成
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

    // 发送数据
    while (len--)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, *buff++);
    }

    // 等待所有数据发送完成
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

    // 切换回接收模式
    GPIO_ResetBits(USART_SLAVE_RE, USART_RE_SLAVE_PIN);
}

///////////////////////////////////////////////////////////////////////////////////////
//void USART1_IRQHandler(void)                	//串口1中断服务程序
//{
//		// 处理溢出错误（ORE）
//    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET) 
//		{
//        USART_ReceiveData(USART1);               // 清除错误标志
//        Master_RX_CNT = 0;                       // 复位主站接收状态
//        Master_FrameFlag = 0;
//        TIM_Cmd(TIM4, DISABLE);                  // 停止主站帧超时定时器
//    }

//    // 接收数据
//    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
//		{
//        uint8_t data = USART_ReceiveData(USART1);
////				printf("UART1 RX: %02X\r\n", data);   // 打印每个字节
//        
//        // 仅当处于主站等待响应模式时，才处理接收数据
//        if (master_state == MASTER_WAIT_RESP) 
//				{
//            // 如果帧未就绪且缓冲区未满，存储数据
//            if (Master_RX_CNT < sizeof(Master_RX_BUFF) && Master_FrameFlag == 0) 
//						{
//                Master_RX_BUFF[Master_RX_CNT++] = data;
//                Master_LastRxTime = GetTick();   // 更新软件超时
//                
//                // 复位帧超时定时器
//                TIM_SetCounter(TIM4, 0);
//                TIM_Cmd(TIM4, ENABLE);            // 开始计时
//            } 
//						else 
//						{
//                // 缓冲区溢出或帧已就绪，复位接收状态
//                Master_RX_CNT = 0;               	
//                TIM_Cmd(TIM4, DISABLE);
//            }
//            
//            // 更新主站超时计数器
//            timeout_cnt = 0;
//        }
//        // 如果不在主站等待响应模式，则丢弃数据（不存储）
//    }
//}
// 修改后的 USART1_IRQHandler
void USART1_IRQHandler(void)
{
    // 处理溢出错误（ORE）
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET) 
    {
        USART_ReceiveData(USART1);               
        // ORE发生时不要清零 RX_CNT，尽量抢救已收到的数据
        TIM_Cmd(TIM4, DISABLE);                  
    }

    // 接收数据
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
    {
        uint8_t data = USART_ReceiveData(USART1);
        
        if (master_state == MASTER_WAIT_RESP) 
        {
            // 只要帧没收完，且缓冲区没满，就存入
            if (Master_RX_CNT < sizeof(Master_RX_BUFF) && Master_FrameFlag == 0) 
            {
                Master_RX_BUFF[Master_RX_CNT++] = data;
                Master_LastRxTime = GetTick();   
                
                TIM_SetCounter(TIM4, 0);
                TIM_Cmd(TIM4, ENABLE);            
            } 
            else 
            {
                // 【修改这里】：如果是帧结束后的多余噪点，关定时器即可，不作任何处理。
                // 绝对不能执行 Master_RX_CNT = 0; 否则会清空刚收到的完整报文！
                TIM_Cmd(TIM4, DISABLE);
            }
            timeout_cnt = 0;
        }
    }
}
//串口2中断服务程序
void USART2_IRQHandler(void)
{
//	// 处理溢出错误（ORE）
//    if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)			
//		{
//        USART_ReceiveData(USART2);
////        RS485_RX_CNT = 0;
////        RS485_FrameFlag = 0;
//        TIM_Cmd(TIM3, DISABLE);
//    }

//    // 接收数据
//    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
//		{
//        uint8_t data = USART_ReceiveData(USART2);
//				MODS_ReciveNew(data);
//    }
	// 1. 更加绝对健壮的 ORE 溢出错误处理
    if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)			
    {
        // STM32 清除 ORE 标志的标准强制动作：先读 SR，再读 DR
        // 使用 volatile 防止高阶编译器优化掉这几行“看似无用”的读取操作
        volatile uint16_t temp = USART2->SR;
        temp = USART2->DR;
        (void)temp; // 消除“变量未使用”的编译警告

        TIM_Cmd(TIM3, DISABLE); // 保留你原有的：关闭Modbus帧超时定时器
        
        // 【建议新增】：如果是在等待电机响应期间发生溢出，可以加个标志位排查
        // 注意：千万不要在中断里调用 printf！
    }

    // 2. 正常接收数据处理
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
    {
        uint8_t data = USART_ReceiveData(USART2);
        
        // 【关键逻辑安全确认】：
        // 如果你的 MODS_ReciveNew(data) 内部已经处理了 Master_RX_BUFF 的写入，可以直接用下面这行：
        MODS_ReciveNew(data);
		}
}


// TIM4 中断服务函数（主站帧超时检测）
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        // 清除中断标志
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        // 停止定时器，等待下一帧接收开始
        TIM_Cmd(TIM4, DISABLE);
        
        // ========== 1. 标记帧接收完成 ==========
        // 如果主站接收缓冲区有数据且帧未处理，则标记一帧接收完成
//        if (Master_RX_CNT > 0 && Master_FrameFlag == 0)
//        {
//            Master_FrameFlag = 1;   // 通知主循环响应已完整接收
//        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////
//用定时器3判断接收空闲时间，当空闲时间大于指定时间，认为一帧结束
//定时器3中断服务程序
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
		{
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        TIM_Cmd(TIM3, DISABLE);          // 停止定时器
				if (g_tModS.RxCount > 0) 
				{            
						g_mods_timeout = 1;         // 通知从站一帧接收完成
				}            
    }
}


//从站定时器
void Timer3Init(void)                //999,71
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = RS485_Frame_Distance*999; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=83;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	
	// 清除中断标志
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//主站定时器
void Timer4Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 使能 TIM4 时钟（APB1 总线）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    // 定时器配置：假设系统时钟 168MHz，APB1 定时器时钟 84MHz，分频 83 → 1MHz 计数频率
    // 超时时间：3.5 字符时间 @ 9600bps ≈ 3.6ms，设置 Period = 3600 - 1
    TIM_TimeBaseInitStructure.TIM_Period = 9999;   // 自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 83;      // 预分频器
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
    
    // 清除中断标志
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);   // 使能更新中断
    
    // 中断优先级配置：抢占优先级低于 UART1，高于主循环（例如抢占 1，子优先级 1）
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 注意：定时器默认不启动，等待收到第一个字节后由中断启动
    TIM_Cmd(TIM4, DISABLE);
//		TIM_Cmd(TIM4, ENABLE);
}
// ======================== Modbus CRC16校验计算 ========================
uint16_t Modbus_CRC16(uint8_t *pData, uint16_t len)
{
    uint16_t crc = 0xFFFF;
		while (len--) {
						crc ^= *pData++;
						for (uint8_t i = 0; i < 8; i++) {
								if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
								else crc >>= 1;
						}
				}
				return crc;  // 返回低字节在前（即返回的值低字节先发送）
}



//Modbus功能码03处理程序///////////////////////////////////////////////////////////////////////////////////////已验证程序OK
//读保持寄存器
uint8_t Modbus_03_ReadHoldReg(uint8_t slave_addr, uint16_t start_reg, uint16_t reg_num, uint16_t *read_buff)
{
		// 1. 参数检查
		if (master_state != MASTER_IDLE || reg_num == 0 || reg_num > 125) 
		{
				printf("03参数错误：master_state=%d, reg_num=%d\n", master_state, reg_num);
				return 1;
    }

    printf("03请求：地址0x%02X, 寄存器0x%04X, 数量%d\n", slave_addr, start_reg, reg_num);

    // 2. 构建 03 请求帧（与之前相同）
    uint8_t frame[8];
    frame[0] = slave_addr;
    frame[1] = 0x03;
    frame[2] = (start_reg >> 8) & 0xFF;
    frame[3] = start_reg & 0xFF;
    frame[4] = (reg_num >> 8) & 0xFF;
    frame[5] = reg_num & 0xFF;
    uint16_t crc = Modbus_CRC16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;
		
		// 打印发送帧
    printf("03 TX: ");
    for (int i=0; i<8; i++) printf("%02X ", frame[i]);
    printf("\r\n");

    // 3. 清空接收缓冲区（避免旧数据干扰）
    __disable_irq();
    Master_RX_CNT = 0;
    __enable_irq();
    memset(Master_RX_BUFF, 0, sizeof(Master_RX_BUFF));

    // 4. 发送请求
    master_state = MASTER_WAIT_RESP;
    RS485_MasterSendData(frame, 8);

    // 5. 非阻塞轮询等待响应（超时 50ms）
    uint32_t start = GetTick();
    uint32_t timeout_ms = 50;
    uint16_t expected_len = 3 + reg_num * 2 + 2;  // 地址+功能+字节数+数据+CRC = 5 + reg_num*2
    uint16_t last_len = 0;

    while ((GetTick() - start) < timeout_ms) {
        __disable_irq();
        uint16_t len = Master_RX_CNT;
        __enable_irq();

				// 每次长度变化时打印缓冲区内容
        if (len != last_len) {
            printf("03 RX_len=%d: ", len);
            for (uint16_t i = 0; i < len; i++) printf("%02X ", Master_RX_BUFF[i]);
            printf("\r\n");
            last_len = len;
        }
			
        // 检查是否收到完整响应（至少包含地址、功能码、字节数、CRC最小长度）
        if (len >= 5) {
            // 先校验地址和功能码
            if (Master_RX_BUFF[0] == slave_addr && Master_RX_BUFF[1] == 0x03) {
                uint8_t byte_count = Master_RX_BUFF[2];
                // 字节数必须等于 reg_num*2
                if (byte_count == reg_num * 2 && len >= (3 + byte_count + 2)) {
                    // 校验 CRC（注意CRC低字节在前）
                    uint16_t recv_crc = (Master_RX_BUFF[3 + byte_count + 1] << 8) | Master_RX_BUFF[3 + byte_count];
                    uint16_t calc_crc = Modbus_CRC16(Master_RX_BUFF, 3 + byte_count); // CRC计算不包含CRC本身
                    if (recv_crc == calc_crc) {
                        // 成功：提取寄存器数据
                        for (uint16_t i = 0; i < reg_num; i++) {
                            read_buff[i] = (Master_RX_BUFF[3 + i * 2] << 8) | Master_RX_BUFF[4 + i * 2];
                        }
                        // 移除已处理帧（防止粘包）
                        __disable_irq();
                        if (Master_RX_CNT >= (3 + byte_count + 2)) {
                            memmove(Master_RX_BUFF, Master_RX_BUFF + (3 + byte_count + 2), Master_RX_CNT - (3 + byte_count + 2));
                            Master_RX_CNT -= (3 + byte_count + 2);
                        } else {
                            Master_RX_CNT = 0;
                        }
                        __enable_irq();
                        master_state = MASTER_IDLE;
                        printf("03指令成功\r\n");
                        return 0;
                    }
                }
            }
            // 不匹配：丢弃首字节，继续拼装下一帧
            __disable_irq();
            memmove(Master_RX_BUFF, Master_RX_BUFF + 1, Master_RX_CNT - 1);
            Master_RX_CNT--;
            __enable_irq();
        }
        delay_ms(1); // 避免忙等
    }

    // 6. 超时处理：检查是否收到部分有效响应（容错）
    if (Master_RX_CNT >= 4) {
        if (Master_RX_BUFF[0] == slave_addr && Master_RX_BUFF[1] == 0x03) {
            uint8_t byte_count = Master_RX_BUFF[2];
            if (byte_count == reg_num * 2 && Master_RX_CNT >= (3 + byte_count)) {
                // 虽缺少CRC，但可视为成功（极端情况）
                for (uint16_t i = 0; i < reg_num; i++) {
                    read_buff[i] = (Master_RX_BUFF[3 + i * 2] << 8) | Master_RX_BUFF[4 + i * 2];
                }
                printf("03指令部分响应（长度%d），视为成功\r\n", Master_RX_CNT);
                __disable_irq();
                Master_RX_CNT = 0;
                __enable_irq();
                master_state = MASTER_IDLE;
                return 0;
            }
        }
    }
    // 超时后打印最终缓冲区内容
    printf("03指令超时，最后缓冲区长度=%d, 内容: ", Master_RX_CNT);
    for (uint16_t i = 0; i < Master_RX_CNT; i++) printf("%02X ", Master_RX_BUFF[i]);
    printf("\r\n");
    master_state = MASTER_IDLE;
    return 2; // 超时错误
}

	


//Modbus功能码06处理程序   //////////////////////////////////////////////////////////////////////
//写单个保持寄存器
// 06功能码：写单个寄存器 - 控制电机启停/设置脉冲数/方向等单参数
//uint8_t Modbus_06_WriteSingleReg(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_data)
//{
////		printf("Modbus_06: reg_data = 0x%04X\n", reg_data);  
//		// 1. 强制重置主站状态（如果处于异常）
//    if (master_state != MASTER_IDLE) {
//        printf("Modbus_06: 强制重置状态，原状态=%d\r\n", master_state);
//        master_state = MASTER_IDLE;
//        timeout_cnt = 0;
//        __disable_irq();
//        Master_RX_CNT = 0;
//        __enable_irq();
//        memset(Master_RX_BUFF, 0, sizeof(Master_RX_BUFF));
//    }

//    // 2. 清空接收缓冲区（确保无残留）
//    __disable_irq();
//    Master_RX_CNT = 0;
//    __enable_irq();
//    memset(Master_RX_BUFF, 0, sizeof(Master_RX_BUFF));

//    // 3. 构建06帧
//    uint8_t frame[8];
//    frame[0] = slave_addr;
//    frame[1] = 0x06;
//    frame[2] = (reg_addr >> 8) & 0xFF;
//    frame[3] = reg_addr & 0xFF;
//    frame[4] = (reg_data >> 8) & 0xFF;
//    frame[5] = reg_data & 0xFF;
//    uint16_t crc = Modbus_CRC16(frame, 6);
//    frame[6] = crc & 0xFF;
//    frame[7] = (crc >> 8) & 0xFF;

//    // 4. 发送指令
//    master_state = MASTER_WAIT_RESP;
//    RS485_MasterSendData(frame, 8);

//    // 5. 软件超时等待响应（50ms）
//    uint32_t start = GetTick();
//    uint32_t timeout_ms = 50;
//    uint16_t last_len = 0;

//    while ((GetTick() - start) < timeout_ms) 
//		{
//				
//				MODS_Poll();   // 处理从站帧（非阻塞，若没有完整帧则快速返回）
//			
//        __disable_irq();
//        uint16_t len = Master_RX_CNT;
//        __enable_irq();

//        if (len != last_len) {
////            printf("RX_len=%d: ", len);
//            for (uint16_t i = 0; i < len; i++) printf("%02X ", Master_RX_BUFF[i]);
//            printf("\r\n");
//            last_len = len;
//        }

//        if (len >= 8) {
//            if (Master_RX_BUFF[0] == slave_addr && Master_RX_BUFF[1] == 0x06 &&
//                ((Master_RX_BUFF[2] << 8) | Master_RX_BUFF[3]) == reg_addr) {
//                uint16_t recv_crc = (Master_RX_BUFF[7] << 8) | Master_RX_BUFF[6];
//                uint16_t calc_crc = Modbus_CRC16(Master_RX_BUFF, 6);
//                if (recv_crc == calc_crc) {
//                    // 成功：移除该帧
//                    __disable_irq();
//                    if (Master_RX_CNT >= 8) {
//                        memmove(Master_RX_BUFF, Master_RX_BUFF + 8, Master_RX_CNT - 8);
//                        Master_RX_CNT -= 8;
//                    } else {
//                        Master_RX_CNT = 0;
//                    }
//                    __enable_irq();
//                    master_state = MASTER_IDLE;
//                    printf("06指令成功\r\n");
//                    return 0;
//                }
//            }
//            // 不匹配，丢弃首字节，继续等待
//            __disable_irq();
//            memmove(Master_RX_BUFF, Master_RX_BUFF + 1, Master_RX_CNT - 1);
//            Master_RX_CNT--;
//            __enable_irq();
//        }
//        else if (len == 7) {
//            // 容忍丢失首字节的情况
//            if (Master_RX_BUFF[0] == 0x06 &&
//                ((Master_RX_BUFF[1] << 8) | Master_RX_BUFF[2]) == reg_addr) {
//                uint8_t full_buf[8];
//                full_buf[0] = slave_addr;
//                memcpy(&full_buf[1], Master_RX_BUFF, 7);
//                uint16_t recv_crc = (full_buf[7] << 8) | full_buf[6];
//                uint16_t calc_crc = Modbus_CRC16(full_buf, 6);
//                if (recv_crc == calc_crc) {
//                    printf("检测到丢失地址的7字节响应，视为成功\r\n");
//                    __disable_irq();
//                    Master_RX_CNT = 0;
//                    __enable_irq();
//                    master_state = MASTER_IDLE;
//                    return 0;
//                }
//            }
//        }
////        delay_ms(1);
//    }

//    // 超时处理：检查是否收到部分有效数据
//    if (Master_RX_CNT >= 4) {
//        if (Master_RX_BUFF[0] == slave_addr && Master_RX_BUFF[1] == 0x06 &&
//            ((Master_RX_BUFF[2] << 8) | Master_RX_BUFF[3]) == reg_addr) {
//            printf("06指令部分响应（长度%d），视为成功\r\n", Master_RX_CNT);
//            __disable_irq();
//            Master_RX_CNT = 0;
//            __enable_irq();
//            master_state = MASTER_IDLE;
//            return 0;
//        }
//    }

//    printf("06指令超时，最后长度=%d\r\n", Master_RX_CNT);
//    master_state = MASTER_IDLE;
//    return 2;
//}

uint8_t Modbus_06_WriteSingleReg(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_data)
{
	printf("Modbus_06 entry: state=%d, addr=0x%04X\n", master_state, reg_addr);
    // 1. 状态检查
    if (master_state != MASTER_IDLE || reg_addr == 0) {
        return 1; // 状态冲突或参数错误
    }

    // 2. 构建06功能码帧（使用局部缓冲区）
    uint8_t local_tx_buff[8];
    local_tx_buff[0] = slave_addr;
    local_tx_buff[1] = 0x06;
    local_tx_buff[2] = (reg_addr >> 8) & 0xFF;
    local_tx_buff[3] = reg_addr & 0xFF;
    local_tx_buff[4] = (reg_data >> 8) & 0xFF;
    local_tx_buff[5] = reg_data & 0xFF;
    uint16_t crc = Modbus_CRC16(local_tx_buff, 6);
    local_tx_buff[6] = crc & 0xFF;
    local_tx_buff[7] = (crc >> 8) & 0xFF;

    // 复制到全局发送缓冲区
    memcpy(RS485_TX_BUFF, local_tx_buff, 8);

    // 3. 重置接收缓冲区和状态
    memset(Master_RX_BUFF, 0, sizeof(Master_RX_BUFF));
    Master_RX_CNT = 0;
    master_state = MASTER_SENDING;
    timeout_cnt = 0;
		
//		printf("06 TX:");
//    for(uint8_t i=0;i<8;i++)
//    {
//        printf("%02X ",local_tx_buff[i]);
//    }
//    printf("\r\n");

    // 4. 发送帧数据
    RS485_MasterSendData(RS485_TX_BUFF, 8);
    master_state = MASTER_WAIT_RESP;

    // 5. 等待响应（双重超时机制）
    uint32_t local_timeout = 0;
    const uint32_t LOCAL_TIMEOUT_MAX = 50;
    uint16_t last_rx_len = 0;
    while (master_state == MASTER_WAIT_RESP) {
        local_timeout++;
        if (timeout_cnt > TIMEOUT_MS || local_timeout > LOCAL_TIMEOUT_MAX) {
					
//            MODS_Poll();   // 处理从站帧（非阻塞）
					
            // 超时前检查是否有数据接收（宽松校验）
            if (Master_RX_CNT > 0) {
                if (Master_RX_BUFF[0] == slave_addr && Master_RX_BUFF[1] == 0x06) {
                    uint16_t resp_reg = (Master_RX_BUFF[2] << 8) | Master_RX_BUFF[3];
                    printf("宽松校验: resp_reg=0x%04X, reg_addr=0x%04X\n", resp_reg, reg_addr);
                    if (resp_reg == reg_addr) {
                        // 移除已处理的8字节帧
                        __disable_irq();
                        if (Master_RX_CNT >= 8) {
                            memmove(Master_RX_BUFF, Master_RX_BUFF + 8, Master_RX_CNT - 8);
                            Master_RX_CNT -= 8;
                        } else if (Master_RX_CNT == 7) {
                            // 容忍丢失首字节
                            if (Master_RX_BUFF[0] == 0x06 &&
                                ((Master_RX_BUFF[1] << 8) | Master_RX_BUFF[2]) == reg_addr) {
                                uint8_t full_buf[8];
                                full_buf[0] = slave_addr;
                                memcpy(&full_buf[1], Master_RX_BUFF, 7);
                                uint16_t recv_crc = (full_buf[7] << 8) | full_buf[6];
                                uint16_t calc_crc = Modbus_CRC16(full_buf, 6);
                                if (recv_crc == calc_crc) {
                                    printf("检测到丢失地址的7字节响应，视为成功\r\n");
                                    __disable_irq();
                                    Master_RX_CNT = 0;
                                    __enable_irq();
                                    master_state = MASTER_IDLE;
                                    return 0;
                                }
                            }
                        } else {
                            Master_RX_CNT = 0;
                        }
                        __enable_irq();
                        master_state = MASTER_IDLE;
                        timeout_cnt = 0;
                        return 0; // 成功
                    }
                }
            }
            printf("06超时退出：local_timeout=%d, Master_RX_CNT=%d\r\n", local_timeout, Master_RX_CNT);
            master_state = MASTER_IDLE;
            timeout_cnt = 0;
						delay_ms(10);
            return 2; // 超时错误
        }

        if (Master_RX_CNT != last_rx_len) {
            printf("RX len=%d: ", Master_RX_CNT);
            for (uint16_t i = 0; i < Master_RX_CNT; i++) printf("%02X ", Master_RX_BUFF[i]);
            printf("\r\n");
            last_rx_len = Master_RX_CNT;
        }

        if (local_timeout > 500000) local_timeout = 0;
        delay_ms(10);
    }

    // 6. 响应校验（master_state == MASTER_RESP_OK）
    if (master_state == MASTER_RESP_OK) {
        printf("06 strict: len=%d, data=", Master_RX_CNT);
        for (uint16_t i = 0; i < Master_RX_CNT; i++) printf("%02X ", Master_RX_BUFF[i]);
        printf("\r\n");

        if (Master_RX_CNT != 8) return 3; // 响应长度错误
        if (Master_RX_BUFF[0] != slave_addr || Master_RX_BUFF[1] != 0x06) return 4;

        uint16_t resp_reg_addr = (Master_RX_BUFF[2] << 8) | Master_RX_BUFF[3];
        uint16_t resp_data = (Master_RX_BUFF[4] << 8) | Master_RX_BUFF[5];
        printf("resp_reg: 0x%04X, reg_addr: 0x%04X, resp_data: 0x%04X, reg_data: 0x%04X\r\n",
               resp_reg_addr, reg_addr, resp_data, reg_data);
        if (resp_reg_addr != reg_addr || resp_data != reg_data) return 5;

        uint16_t resp_crc = (Master_RX_BUFF[7] << 8) | Master_RX_BUFF[6];
        uint16_t calc_crc = Modbus_CRC16(Master_RX_BUFF, 6);
        if (resp_crc != calc_crc) return 6;

        master_state = MASTER_IDLE;
        return 0; // 成功
    } else {
        master_state = MASTER_IDLE;
        return 7; // 响应异常
    }
}

// Modbus功能码06处理程序
// 写单个保持寄存器
// 返回:
// 0:成功
// 1:参数错误
// 2:超时
//uint8_t Modbus_06_WriteSingleReg(uint8_t slave_addr, 
//                                 uint16_t reg_addr, 
//                                 uint16_t reg_data)
//{
//    uint8_t frame[8];

//    /*
//     * 1. 检查主站状态
//     * 不允许强制清状态
//     */
//    if (master_state != MASTER_IDLE)
//    {
//        printf("Modbus_06: 主站忙，状态=%d\r\n", master_state);
//        return 2;
//    }


//    /*
//     * 2. 清空接收缓存
//     */
//    __disable_irq();
//    Master_RX_CNT = 0;
//    __enable_irq();

//    memset(Master_RX_BUFF, 0, sizeof(Master_RX_BUFF));


//    /*
//     * 3. 构造06功能码帧
//     *
//     * 格式:
//     * 地址
//     * 06
//     * 寄存器地址
//     * 数据
//     * CRC
//     */
//    frame[0] = slave_addr;
//    frame[1] = 0x06;

//    frame[2] = (reg_addr >> 8) & 0xFF;
//    frame[3] = reg_addr & 0xFF;

//    frame[4] = (reg_data >> 8) & 0xFF;
//    frame[5] = reg_data & 0xFF;


//    uint16_t crc = Modbus_CRC16(frame, 6);

//    frame[6] = crc & 0xFF;
//    frame[7] = (crc >> 8) & 0xFF;


//    printf("06 TX:");
//    for(uint8_t i=0;i<8;i++)
//    {
//        printf("%02X ",frame[i]);
//    }
//    printf("\r\n");


//    /*
//     * 4. 发送数据
//     */
//    master_state = MASTER_WAIT_RESP;

//    RS485_MasterSendData(frame, 8);


//    /*
//     * 5. 等待响应
//     *
//     * 注意：
//     * 这里不再调用 MODS_Poll()
//     *
//     * 接收由主循环统一处理
//     */
//    uint32_t start = GetTick();

//    uint32_t timeout_ms = 100;


//    while((GetTick() - start) < timeout_ms)
//    {

//        __disable_irq();
//        uint16_t len = Master_RX_CNT;
//        __enable_irq();


//        /*
//         * 收到完整06响应
//         *
//         * 格式:
//         * 地址 06 寄存器 数据 CRC
//         */
//        if(len >= 8)
//        {

//            printf("06 RX:");
//            for(uint16_t i=0;i<len;i++)
//            {
//                printf("%02X ",Master_RX_BUFF[i]);
//            }
//            printf("\r\n");


//            if(Master_RX_BUFF[0] == slave_addr &&
//               Master_RX_BUFF[1] == 0x06 &&
//               ((Master_RX_BUFF[2]<<8)|Master_RX_BUFF[3]) == reg_addr)
//            {

//                uint16_t recv_crc =
//                    (Master_RX_BUFF[7]<<8) |
//                    Master_RX_BUFF[6];


//                uint16_t calc_crc =
//                    Modbus_CRC16(Master_RX_BUFF,6);


//                if(recv_crc == calc_crc)
//                {

//                    __disable_irq();

//                    Master_RX_CNT = 0;

//                    __enable_irq();


//                    master_state = MASTER_IDLE;


//                    printf("06指令成功\r\n");

//                    return 0;
//                }
//            }


//            /*
//             * 非目标帧，丢弃
//             */
//            __disable_irq();

//            if(Master_RX_CNT > 0)
//            {
//                memmove(
//                    Master_RX_BUFF,
//                    Master_RX_BUFF+1,
//                    Master_RX_CNT-1
//                );

//                Master_RX_CNT--;
//            }

//            __enable_irq();

//        }

//    }


//    /*
//     * 6. 超时处理
//     */

//    printf("06指令超时，RX长度=%d\r\n",
//           Master_RX_CNT);


//    master_state = MASTER_IDLE;


//    __disable_irq();
//    Master_RX_CNT = 0;
//    __enable_irq();


//    return 2;
//}	

//Modbus功能码10处理程序 
//写多个保持寄存器
uint8_t Modbus_10_WriteMultiReg(uint8_t slave_addr, uint16_t start_reg, uint16_t reg_num, uint16_t *write_buff)
{
  					// 清空缓冲区
			memset(Master_RX_BUFF, 0, sizeof(Master_RX_BUFF));
			__disable_irq();
			Master_RX_CNT = 0;
			__enable_irq();  
	if(master_state != MASTER_IDLE || reg_num == 0 || reg_num > 123) return 1; // 参数错误
		// 使用局部缓冲区，避免全局缓冲区被污染
    uint8_t local_tx_buff[256];
    // 10功能码帧格式：从站地址(1) + 功能码10(1) + 起始寄存器(2) + 寄存器数(2) + 字节数(1) + 寄存器数据(N*2) + CRC(2)
    uint8_t frame_len = 7 + reg_num * 2 +2;
    local_tx_buff[0] = slave_addr;                          // 从站地址
    local_tx_buff[1] = 0x10;                                // 10功能码（0x10=16进制）
    local_tx_buff[2] = (start_reg >> 8) & 0xFF;             // 起始寄存器高字节
    local_tx_buff[3] = start_reg & 0xFF;                    // 起始寄存器低字节
    local_tx_buff[4] = (reg_num >> 8) & 0xFF;               // 寄存器数量高字节
    local_tx_buff[5] = reg_num & 0xFF;                      // 寄存器数量低字节
    local_tx_buff[6] = reg_num * 2;                         // 字节数（寄存器数*2）

    // 填充写入数据（高字节在前）
    for(uint16_t i=0; i<reg_num; i++)
    {
        local_tx_buff[7 + i*2] = (write_buff[i] >> 8) & 0xFF;
        local_tx_buff[8 + i*2] = write_buff[i] & 0xFF;
//				printf("循环i=%d：填充位置[%d]=%02X, [%d]=%02X\n", 
//           i, 7+i*2, RS485_TX_BUFF[7+i*2], 8+i*2, RS485_TX_BUFF[8+i*2]);
//			printf("\r\n");
    }

    // 计算CRC
    uint16_t crc = Modbus_CRC16(local_tx_buff, frame_len - 2);
    local_tx_buff[frame_len - 2] = crc & 0xFF;
    local_tx_buff[frame_len - 1] = (crc >> 8) & 0xFF;
		// 复制到全局发送缓冲区
    memcpy(RS485_TX_BUFF, local_tx_buff, frame_len);

    // 重置接收缓冲区和状态
    memset(Master_RX_BUFF, 0, sizeof(Master_RX_BUFF));
    Master_RX_CNT = 0;
    master_state = MASTER_SENDING;
    timeout_cnt = 0;
		
		printf("发送10指令: ");
		for (int i=0; i<frame_len; i++) printf("%02X ", local_tx_buff[i]);
		printf("\r\n");

    // 发送帧数据
    RS485_MasterSendData(RS485_TX_BUFF, frame_len);
    master_state = MASTER_WAIT_RESP;
    // 等待响应或超时
    uint32_t local_timeout = 0; // 本地超时计数器，避免依赖全局TIM3
		const uint32_t LOCAL_TIMEOUT_MAX = 50; 
		uint16_t last_rx_len = 0; 
		while(master_state == MASTER_WAIT_RESP)
		{
				local_timeout++;
				// 双重超时判断：全局timeout_cnt超时 OR 本地计数兜底超时（
				if(timeout_cnt > TIMEOUT_MS || local_timeout > LOCAL_TIMEOUT_MAX) 
				{
//						 MODS_Poll();   // 处理从站帧（非阻塞，若没有完整帧则快速返回）
						// 超时前先检查是否有数据接收（有数据=响应已到，宽松校验）
						if(Master_RX_CNT > 0)
						{
								if (Master_RX_BUFF[0] == slave_addr && Master_RX_BUFF[1] == 0x10) 
								{
									uint16_t resp_reg = (Master_RX_BUFF[2]<<8) | Master_RX_BUFF[3];
									printf("宽松校验: resp_reg=0x%04X, start_reg=0x%04X\n", resp_reg, start_reg);
									if (resp_reg == start_reg)  
									{
										// 移除已处理的8字节帧
										__disable_irq();
										if (Master_RX_CNT >= 8) 
										{
												memmove(Master_RX_BUFF, Master_RX_BUFF + 8, Master_RX_CNT - 8);
												Master_RX_CNT -= 8;
										}
										else if (Master_RX_CNT == 7) 
										{
												// 容忍丢失首字节的情况
												if (Master_RX_BUFF[0] == 0x10 &&
														((Master_RX_BUFF[1] << 8) | Master_RX_BUFF[2]) == start_reg) {
														uint8_t full_buf[8];
														full_buf[0] = slave_addr;
														memcpy(&full_buf[1], Master_RX_BUFF, 7);
														uint16_t recv_crc = (full_buf[7] << 8) | full_buf[6];
														uint16_t calc_crc = Modbus_CRC16(full_buf, 6);
														if (recv_crc == calc_crc) {
																printf("检测到丢失地址的7字节响应，视为成功\r\n");
																__disable_irq();
																Master_RX_CNT = 0;
																__enable_irq();
																master_state = MASTER_IDLE;
																return 0;
														}
												}
										}										
										else 
										{
												Master_RX_CNT = 0; // 安全起见，清空
										}
										__enable_irq();	
									
										master_state = MASTER_IDLE;
										timeout_cnt = 0;
										return 0; // 成功
									}
								}
						}
						// 无数据/校验失败，返回超时
						printf("超时退出：local_timeout=%d, Master_RX_CNT=%d\r\n", local_timeout, Master_RX_CNT);
						fflush(stdout); // 确保打印输出
						
						master_state = MASTER_IDLE;
						timeout_cnt = 0;
						return 2; // 超时错误
				}
				
				if (Master_RX_CNT != last_rx_len) {
				printf("RX len=%d: ", Master_RX_CNT);
				for (uint16_t i = 0; i < Master_RX_CNT; i++) printf("%02X ", Master_RX_BUFF[i]);
				printf("\r\n");
				last_rx_len = Master_RX_CNT;
				}
				
				// 本地计数溢出重置
				if(local_timeout > 500000) local_timeout = 0;
				// 补充短延时，降低CPU占用（必加）
//				delay_ms(10);
				delay_us(500);
		}

    // 校验响应（10功能码响应为原请求的起始寄存器+数量）
    if(master_state == MASTER_RESP_OK)
    {
        printf("10 func strict: len=%d, data=", Master_RX_CNT);
				for (uint16_t i=0; i<Master_RX_CNT; i++) printf("%02X ", Master_RX_BUFF[i]);
				printf("\r\n");  
			
			// 10功能码响应格式：从站地址(1) + 功能码10(1) + 起始寄存器(2) + 寄存器数(2) + CRC(2)
        if(Master_RX_CNT != 8) return 3; // 响应长度错误
        if(Master_RX_BUFF[0] != slave_addr || Master_RX_BUFF[1] != 0x10) return 4;

        // 校验起始寄存器和数量
        uint16_t resp_start_reg = (Master_RX_BUFF[2] << 8) | Master_RX_BUFF[3];
        uint16_t resp_reg_num = (Master_RX_BUFF[4] << 8) | Master_RX_BUFF[5];
				printf("resp_start_reg: %d, start_reg: %d\r\n", resp_start_reg, start_reg);
        if(resp_start_reg != start_reg || resp_reg_num != reg_num) return 5;

        // 校验CRC
        uint16_t resp_crc = (Master_RX_BUFF[7] << 8) | Master_RX_BUFF[6];
        uint16_t calc_crc = Modbus_CRC16(Master_RX_BUFF, 6);
        if(resp_crc != calc_crc) return 6;

        master_state = MASTER_IDLE;
        return 0; // 成功
    }
    else
    {
        master_state = MASTER_IDLE;
        return 7; // 响应异常
    }
}

/**
 * @brief 轮询电机2、5~12的报警状态，如有报警则存入0x17并清除报警
 * @return 0:全部成功; 非0:某个电机操作失败（可记录错误码）
 */
//void PollAndClearMotorAlarms_NonBlocking(void)
//{
//    static uint8_t poll_counter = 0;   // 计数器，仅当不在轮询时累加
////		printf("poll_cnt=%d, state=%d, busy=%d\r\n", poll_counter, master_state, Sequence_IsBusy());
//    // ---------- 1. 启动轮询（基于计数） ----------
//    if (!alarm_poll_active) {
//        poll_counter++;
//        if (poll_counter >= ALARM_POLL_COUNT_THRESHOLD &&
//        master_state == MASTER_IDLE && !Sequence_IsBusy()) {
//            poll_counter = 0;                       // 复位计数
//            alarm_poll_active = 1;                  // 进入轮询状态
//            alarm_motor_index = MOTOR_ID_START;     // 从第一个电机开始
//            alarm_substep = 0;                      // 初始步骤：读报警
//            printf("启动报警轮询（计数触发）\n");
//        } else {
//            return;  // 未达到阈值，本次不执行
//        }
//    }

//    // ---------- 2. 轮询执行（一次函数调用处理一个电机的一个步骤） ----------
//    if (alarm_motor_index > MOTOR_ID_END) {
//        // 所有电机处理完毕，结束轮询
//        alarm_poll_active = 0;
//        // 注意：不在此处复位计数器，下次进入时将从0开始累加
//        return;
//    }

//    uint8_t slave = motor_slave_addr[alarm_motor_index];
//    uint16_t alarm_value;
//    uint8_t ret;

//    MasterBusy_Acquire();   // 获取总线锁，防止与其他主站通信冲突

//    if (alarm_substep == 0) {
//        // 步骤0：读取报警值
//        ret = Modbus_03_ReadHoldReg(slave, ALARM_QUERY_REG, 1, &alarm_value);
//        printf("电机0x%02X 报警值: 0x%04X\r\n", slave, alarm_value);
//        if (ret != 0) {
//            printf("电机0x%02X 读报警失败\r\n", slave);
//            alarm_motor_index++;               // 失败则跳过该电机
//            MasterBusy_Release();
//            return;
//        }
//        if ((alarm_value & 0x000F) != 0) 
//				{    
//            alarm_substep = 1;                 // 有报警，进入清除步骤
//        }
//				else
//				{
//            alarm_motor_index++;               // 无报警，继续下一个电机
//        }
//    } else if (alarm_substep == 1) {
//        // 步骤1：清除报警（并重新使能）
//        ret = Modbus_06_WriteSingleReg(slave, ALARM_CLEAR_REG, 0x0000);
//        if (ret == 0) {
//            printf("电机0x%02X 清除报警成功\n", slave);
//            Modbus_06_WriteSingleReg(slave, REG_ENABLE, 0x0000);  // 重新使能
//        } else {
//            printf("电机0x%02X 清除报警失败\n", slave);
//        }
//        alarm_motor_index++;      // 无论成功与否，进入下一个电机
//        alarm_substep = 0;        // 复位步骤
//    }

//    MasterBusy_Release();   // 释放总线锁
//}
/**
 * @brief 轮询电机2、5~12的报警状态，如有报警则存入0x17并清除报警（基于 GetTick 非阻塞实现）
 * @return 0:全部成功; 非0:某个电机操作失败（可记录错误码）
 */
void PollAndClearMotorAlarms_NonBlocking(void)
{
    static uint32_t last_poll_tick = 0;   // 记录上一次启动轮询的系统时间戳
    const uint32_t poll_interval_ms = 1000; // 轮询时间间隔，单位毫秒（可根据需要调整，例如 1000ms = 1秒）

    // ---------- 1. 检查是否到达触发时间启动轮询 ----------
    if (!alarm_poll_active) {
        // 使用无符号减法，天然支持 sys_tick 溢出回绕
        if ((GetTick() - last_poll_tick) >= poll_interval_ms) {
            // 满足时间间隔，检查系统是否空闲
            if (master_state == MASTER_IDLE && !Sequence_IsBusy()) {
                last_poll_tick = GetTick();             // 更新上次触发时间
                alarm_poll_active = 1;                  // 进入轮询状态
                alarm_motor_index = MOTOR_ID_START;     // 从第一个电机开始
                alarm_substep = 0;                      // 初始步骤：读报警
                printf("启动报警轮询（时间戳触发）\n");
            }
        }
        return;  // 未达到时间间隔或系统忙，本次不执行
    }

    // ---------- 2. 轮询执行（一次函数调用处理一个电机的一个步骤） ----------
    if (alarm_motor_index > MOTOR_ID_END) {
        // 所有电机处理完毕，结束轮询
        alarm_poll_active = 0;
        return;
    }

    uint8_t slave = motor_slave_addr[alarm_motor_index];
    uint16_t alarm_value;
    uint8_t ret;

    MasterBusy_Acquire();   // 获取总线锁，防止与其他主站通信冲突

    if (alarm_substep == 0) {
        // 步骤0：读取报警值
        ret = Modbus_03_ReadHoldReg(slave, ALARM_QUERY_REG, 1, &alarm_value);
        printf("电机0x%02X 报警值: 0x%04X\r\n", slave, alarm_value);
        if (ret != 0) {
            printf("电机0x%02X 读报警失败\r\n", slave);
            alarm_motor_index++;               // 失败则跳过该电机
            MasterBusy_Release();
            return;
        }
        if ((alarm_value & 0x000F) != 0) {    
            alarm_substep = 1;                 // 有报警，进入清除步骤
        } else {
            alarm_motor_index++;               // 无报警，继续下一个电机
        }
    } else if (alarm_substep == 1) {
        // 步骤1：清除报警（并重新使能）
        ret = Modbus_06_WriteSingleReg(slave, ALARM_CLEAR_REG, 0x0000);
        if (ret == 0) {
            printf("电机0x%02X 清除报警成功\n", slave);
            Modbus_06_WriteSingleReg(slave, REG_ENABLE, 0x0000);  // 重新使能
        } else {
            printf("电机0x%02X 清除报警失败\n", slave);
        }
        alarm_motor_index++;      // 无论成功与否，进入下一个电机
        alarm_substep = 0;        // 复位步骤
    }

    MasterBusy_Release();   // 释放总线锁
}
// ======================== 步进电机控制封装函数 ========================
// 单电机控制（06功能码）
uint8_t Motor_Single_Control(uint8_t slave_addr, uint8_t motor_num, uint16_t motor_cmd)
{
    uint16_t reg_addr;
    // 校验电机编号
    if(motor_num < 1 || motor_num >6) return 1;
    // 匹配电机寄存器地址
    switch(motor_num)
    {
        case 1: reg_addr = MOTOR1_CTRL_REG1; break;
        default: return 1;
    }
		if(reg_addr > 0x0FFF) // 假设从站最大支持0x0FFF地址
    {
        return 6; // 寄存器地址越界
    }
    // 调用06功能码
    return Modbus_06_WriteSingleReg(slave_addr, reg_addr, motor_cmd);
}

uint8_t Motor_Control(uint8_t motor_id, uint8_t reg_num, uint16_t motor_cmd)
{
    uint8_t slave_addr;  // 电机对应的从站地址
    uint16_t reg_addr;   // 寄存器ID对应的实际地址

    // 1. 电机ID合法性校验
    if(motor_id < MOTOR_ID_3 || motor_id > MOTOR_ID_13)
    {
        return 1; // 电机ID错误
    }

    // 2. 核心映射：电机ID + 寄存器ID → 从站地址 + 实际寄存器地址
    switch(motor_id)
    {
        case MOTOR_ID_1: // 电机1
            slave_addr = MOTOR1_SLAVE_ADDR; // 电机1固定从站地址0x01
            // 电机1的寄存器ID映射到实际地址
            switch(reg_num)
            {
								case 1: reg_addr = MOTOR1_CTRL_REG1; break;
								default: return 1;
            }
            break;

        case MOTOR_ID_2: // 电机2
            slave_addr = MOTOR2_SLAVE_ADDR; // 电机2固定从站地址0x02
            // 电机2的寄存器ID映射到实际地址
            switch(reg_num)
            {
								case 1: reg_addr = MOTOR2_CTRL_REG1; break;
								default: return 1;
            }
            break;
						
				case MOTOR_ID_3: // 电机3
            slave_addr = MOTOR3_SLAVE_ADDR; // 电机3固定从站地址0x03
            // 电机2的寄存器ID映射到实际地址
            switch(reg_num)
            {
								case 1: reg_addr = MOTOR3_CTRL_REG1; break;
								case 2: reg_addr = MOTOR3_CTRL_REG2; break;
								case 3: reg_addr = MOTOR3_CTRL_REG3; break;
								default: return 1;
            }
            break;
						 
				case MOTOR_ID_4: // 电机4
            slave_addr = MOTOR4_SLAVE_ADDR; // 电机4固定从站地址0x04
            // 电机1的寄存器ID映射到实际地址
						switch(reg_num)
            {
								case 1: reg_addr = MOTOR4_CTRL_REG1; break;
								default: return 1;
            }
            break;
				
				case MOTOR_ID_13: // 电机13
            slave_addr = MOTOR13_SLAVE_ADDR; // 电机4固定从站地址0x0d
            // 电机1的寄存器ID映射到实际地址
						switch(reg_num)
            {
								case 1: reg_addr = MOTOR13_CTRL_REG1; break;
								default: return 1;
            }
            break;

        default:
            return 1; // 电机ID异常
    }

    // 3. 调用Modbus写寄存器
    return Modbus_06_WriteSingleReg(slave_addr, reg_addr, motor_cmd);
}

// 批量电机控制（10功能码）
uint8_t Motor_Batch_Control(uint8_t slave_addr, uint16_t start_reg, uint16_t reg_num, uint16_t *motor_cmds)
{
    return Modbus_10_WriteMultiReg(slave_addr, start_reg, reg_num, motor_cmds);
}

// 读取单电机状态（03功能码）
uint8_t Motor_Read_Status(uint8_t motor_num, uint16_t *motor_status)
{
//    uint16_t reg_addr;
//    if(motor_num < 1 || motor_num > 5) return 1;
//    switch(motor_num)
//    {
//        case 1: reg_addr = MOTOR1_STATUS_REG; break;
//        case 2: reg_addr = MOTOR2_STATUS_REG; break;
//        case 3: reg_addr = MOTOR3_STATUS_REG; break;
//        case 4: reg_addr = MOTOR4_STATUS_REG; break;
//        case 5: reg_addr = MOTOR5_STATUS_REG; break;
//        default: return 1;
//    }
//    return Modbus_03_ReadHoldReg(MODBUS_SLAVE_ADDR, reg_addr, 1, motor_status);
}

// 批量读取电机状态（03功能码）
uint8_t Motor_Batch_Read_Status(uint16_t start_reg, uint16_t motor_num, uint16_t *status_buff)
{
//    if(start_reg < MOTOR1_STATUS_REG || start_reg > MOTOR5_STATUS_REG || motor_num == 0 ||
//       (start_reg + motor_num - 1) > MOTOR5_STATUS_REG)
//    {
//        return 1; // 地址越界
//    }
//    return Modbus_03_ReadHoldReg(MODBUS_SLAVE_ADDR, start_reg, motor_num, status_buff);
}

// ======================== 超时检测函数 ========================
void Modbus_Timeout_Check(void)
{
    if(master_state == MASTER_WAIT_RESP)
    {
        // 仅在TIM3未触发时，递增超时计数（避免重复计数）
        if((TIM3->CR1 & TIM_CR1_CEN) != RESET) 
        {
            timeout_cnt++;
            // 超时阈值判断
            if(timeout_cnt > TIMEOUT_MS)
            {
                master_state = MASTER_RESP_ERR;
                TIM_Cmd(TIM3, DISABLE); // 停止TIM3
                RS485_RX_CNT = 0; // 清空计数
            }
        }
    }
}


// 全局变量：1ms对应的SysTick计数值（需提前初始化）
u32 fac_ms = 0;  

// 初始化SysTick，必须在main函数开头调用！
void SysTick_Init(void)
{
    // 配置SysTick时钟源为AHB（72MHz），关闭SysTick
    SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk; // 0=AHB/8(9MHz)，1=AHB(72MHz)
    // 若用AHB/8，注释上面，打开下面：
    // SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; 
    
    // 计算1ms对应的计数值
    if(SysTick->CTRL & SysTick_CTRL_CLKSOURCE_Msk)
    {
        fac_ms = SystemCoreClock / 1000; // AHB=72MHz → fac_ms=72000
    }
    else
    {
        fac_ms = SystemCoreClock / 8 / 1000; // AHB/8=9MHz → fac_ms=9000
    }
    
    // 关闭SysTick，清空计数器
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL = 0x00;
}


/**
 * @brief 快速控制电机4和5，实现接近同时执行
 * @param motor4_length 电机4的行程值
 * @param motor5_length 电机5的行程值
 * 
 * 功能：构建两条Modbus指令，快速连续发送给电机4和5
 * 发送间隔：约200-500微秒
 * 注意：此函数不等待响应，如需响应请后续调用状态查询函数
 */
void Quick_Motors_Control(MotorControlParams *motors, uint8_t count)
{
    // 禁用从站接收，避免干扰
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);

    // 统一清空缓冲区（仅一次）
    __disable_irq();
    Master_RX_CNT = 0;
    Master_FrameFlag = 0;
    __enable_irq();
    master_state = MASTER_WAIT_RESP;   // 设置一次

    for (uint8_t i = 0; i < count; i++) {
        uint16_t cmds[2] = {motors[i].len_l, motors[i].len_h};
        uint8_t frame[32];
        uint8_t len = Build_Modbus_Frame(motors[i].slave_address, 0x10,
                                         motors[i].contrl_reg, 2, cmds, frame);
        if (len == 0) {
            printf("构建电机%d帧失败\n", i+1);
            continue;
        }
        printf("Send to 0x%02X: ", motors[i].slave_address);
        for (uint8_t j = 0; j < len; j++) printf("%02X ", frame[j]);
        printf("\r\n");

        RS485_MasterSendData(frame, len);
        // 极短延时，避免总线冲突，不影响累积响应
				delay_ms(20); 
    }
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    // 等待所有电机返回响应（必要）
     

    master_state = MASTER_IDLE;
    
}
/**
 * @brief 非阻塞Modbus发送函数
 * @param slave_addr 从站地址
 * @param start_reg 起始寄存器地址
 * @param reg_num 寄存器数量
 * @param data 要写入的数据数组
 * @return 0:成功, 1:参数错误, 2:主站忙
 * 
 * 功能：构建Modbus 0x10功能码帧并发送，不等待响应
 * 注意：此函数不会阻塞，调用后立即返回
 */
uint8_t Modbus_Send_NonBlocking(uint8_t slave_addr, uint16_t start_reg, uint16_t reg_num, uint16_t *data)
{
    uint8_t frame[256];
    uint8_t frame_len = 0;
    
    // 参数检查
    if(reg_num == 0 || reg_num > 123) {
        printf("错误：寄存器数量无效，reg_num=%d\r\n", reg_num);
        return 1;
    }
    
    if(master_state != MASTER_IDLE) {
        printf("警告：主站忙，状态=%d\r\n", master_state);
        return 2;
    }
    
    // 构建Modbus帧
    frame_len = Build_Modbus_Frame(slave_addr, 0x10, start_reg, reg_num, data, frame);
    if(frame_len == 0) {
        printf("错误：构建Modbus帧失败\r\n");
        return 3;
    }
    
    // 更新主站状态
    master_state = MASTER_SENDING;
    timeout_cnt = 0;
    
    // 发送数据
    RS485_MasterSendData(frame, frame_len);
    
    // 切换状态到等待响应
    master_state = MASTER_WAIT_RESP;
    
    printf("已发送指令: 地址=0x%02X, 寄存器=0x%04X, 长度=%d字节\r\n", 
           slave_addr, start_reg, frame_len);
    
    return 0;
}

/**
 * @brief 构建Modbus RTU帧
 * @param slave_addr 从站地址
 * @param func_code 功能码
 * @param start_reg 起始寄存器地址
 * @param reg_num 寄存器数量
 * @param data 数据数组
 * @param frame 输出帧缓冲区
 * @return 帧长度
 * 
 * 功能：根据参数构建完整的Modbus RTU帧
 * 支持功能码0x10（写多个寄存器）和0x03（读保持寄存器）
 */
uint8_t Build_Modbus_Frame(uint8_t slave_addr, uint8_t func_code, uint16_t start_reg, 
                          uint8_t reg_num, uint16_t *data, uint8_t *frame)
{
    uint8_t idx = 0;
		uint8_t local_tx_buff[256];
    
    // 帧头
    local_tx_buff[idx++] = slave_addr;      // 从站地址
    local_tx_buff[idx++] = func_code;       // 功能码
    
    // 寄存器地址
    local_tx_buff[idx++] = (start_reg >> 8) & 0xFF;  // 高字节
    local_tx_buff[idx++] = start_reg & 0xFF;         // 低字节
    
    if(func_code == 0x10) {  // 写多个寄存器
        // 寄存器数量
        local_tx_buff[idx++] = 0x00;              // 寄存器数量高字节（通常为0）
        local_tx_buff[idx++] = reg_num;           // 寄存器数量低字节
        
        // 字节数
        local_tx_buff[idx++] = reg_num * 2;       // 字节数 = 寄存器数量 * 2
        
        // 数据
        for(uint8_t i = 0; i < reg_num; i++) {
            local_tx_buff[idx++] = (data[i] >> 8) & 0xFF;  // 数据高字节
            local_tx_buff[idx++] = data[i] & 0xFF;         // 数据低字节
        }
    } else if(func_code == 0x03) {  // 读保持寄存器
        // 寄存器数量
        local_tx_buff[idx++] = 0x00;              // 寄存器数量高字节
        local_tx_buff[idx++] = reg_num;           // 寄存器数量低字节
    } else {
       // printf("错误：不支持的功能码 0x%02X\r\n", func_code);
        return 0;
    }
    
    // 计算CRC
    uint16_t crc = Modbus_CRC16(local_tx_buff, idx);
    local_tx_buff[idx++] = crc & 0xFF;        // CRC低字节在前
    local_tx_buff[idx++] = (crc >> 8) & 0xFF; // CRC高字节在后
		memcpy(frame, local_tx_buff, idx);
    
    return idx;  // 返回帧长度
}

/**
 * @brief 检查Modbus响应
 * @param slave_addr 从站地址
 * @param start_reg 起始寄存器地址（用于校验）
 * @param result 输出结果（可选）
 * @param timeout_ms 超时时间（毫秒）
 * @return 0:成功, 1:参数错误, 2:超时, 3:响应格式错误, 4:CRC错误
 * 
 * 功能：等待并检查指定从站的Modbus响应
 */
uint8_t Modbus_Check_Response(uint8_t slave_addr, uint16_t start_reg, uint16_t *result, uint16_t timeout_ms)
{
    uint32_t start_tick = GetTick();  // 记录起始时间

    if (slave_addr == 0) {
        printf("错误：从站地址不能为0\r\n");
        return 1;
    }

//    printf("等待从站0x%02X响应，超时=%dms...\r\n", slave_addr, timeout_ms);

    while ((GetTick() - start_tick) < timeout_ms) {  // 超时判断
        if (Master_RX_CNT  > 0) {
            // 至少需要 8 字节完整帧
            if (Master_RX_CNT  < 8) {
                delay_ms(1);  // 等待更多数据
                continue;
            }

            // 检查从站地址
            if (Master_RX_BUFF[0] != slave_addr) {
                memmove(Master_RX_BUFF, Master_RX_BUFF + 1, Master_RX_CNT  - 1);
                Master_RX_CNT --;
                continue;
            }

            // 检查功能码（写多个寄存器应为 0x10）
            if (Master_RX_BUFF[1] != 0x10) {
                printf("功能码错误: 期望0x10, 收到0x%02X\r\n", Master_RX_BUFF[1]);
                memmove(Master_RX_BUFF, Master_RX_BUFF + 1, Master_RX_CNT  - 1);
                Master_RX_CNT --;
                continue;
            }

            // 检查起始寄存器
            uint16_t resp_start_reg = (Master_RX_BUFF[2] << 8) | Master_RX_BUFF[3];
            if (resp_start_reg != start_reg) {
                printf("起始寄存器不匹配: 期望0x%04X, 收到0x%04X\r\n", start_reg, resp_start_reg);
                memmove(Master_RX_BUFF, Master_RX_BUFF + 1, Master_RX_CNT  - 1);
                Master_RX_CNT --;
                continue;
            }

            // 检查 CRC
            uint16_t resp_crc = (Master_RX_BUFF[7] << 8) | Master_RX_BUFF[6];
            uint16_t calc_crc = Modbus_CRC16(Master_RX_BUFF, 6);
            if (resp_crc != calc_crc) {
                printf("CRC校验失败: 收到0x%04X, 计算0x%04X\r\n", resp_crc, calc_crc);
                memmove(Master_RX_BUFF, Master_RX_BUFF + 1, Master_RX_CNT  - 1);
                Master_RX_CNT --;
                continue;
            }

            // 成功，移除已处理的帧（8字节）
            memmove(Master_RX_BUFF, Master_RX_BUFF + 8, Master_RX_CNT  - 8);
            Master_RX_CNT  -= 8;
            printf("从站0x%02X响应成功\r\n", slave_addr);
            if (result != NULL) *result = 0x0001;
            return 0;
        }
        // 无数据，短暂延时避免空转
        delay_ms(1);
    }

    return 2;
}

/**
 * @brief 批量控制电机4和5并检查状态
 * @return 0:全部成功, 1:电机4失败电机5成功, 2:电机4成功电机5失败, 3:全部失败
 * 
 * 功能：完整的电机控制流程，包括发送指令、检查响应、读取状态
 */
//uint8_t Control_Motors_Complete(MotorControlParams *motors, uint8_t count, uint8_t *results)
//{
//    // 1. 快速发送所有电机指令
//    Quick_Motors_Control(motors, count);
//		printf("Master RX buffer len=%d, content: ", Master_RX_CNT);
//		for (uint16_t i = 0; i < Master_RX_CNT; i++) {
//    printf("%02X ", Master_RX_BUFF[i]);
//		}
//		printf("\r\n");
//    
////		delay_ms(500);
//    
//    // 3. 检查每个电机的响应
//    uint8_t success_count = 0;
//    for (uint8_t i = 0; i < count; i++) {
//        //printf("检查电机%d（地址0x%02X）响应...\r\n", i+1, motors[i].slave_address);
//        uint8_t ret = Modbus_Check_Response_FromBuffer(motors[i].slave_address, motors[i].contrl_reg, NULL);
//        if (results != NULL) {
//            results[i] = ret;
//        }
//        if (ret == 0) {
//             printf("电机0x%02X 响应成功\r\n", motors[i].slave_address);
//            success_count++;
//        } else {
//            printf("电机0x%02X 响应未找到\r\n", motors[i].slave_address);
//        }
//    }
//		
//		 //清理缓冲区
//		__disable_irq();
//		Master_RX_CNT = 0;
//		__enable_irq();

//    // 恢复主站状态
//    master_state = MASTER_IDLE;
//    // 返回结果：可根据需要定义，例如返回失败个数或掩码
//    if (success_count == count) {
//        return 0; // 全部成功
//    } else {
//        // 可自定义返回失败电机的掩码，这里简单返回失败个数
//        return count - success_count;
//    }
//}
uint8_t Control_Motors_Complete(MotorControlParams *motors, uint8_t count, uint8_t *results)
{
    uint8_t success_count = 0;

    for (uint8_t i = 0; i < count; i++) {
        uint16_t cmds[2] = {motors[i].len_l, motors[i].len_h};
        uint8_t frame[32];
        uint8_t len = Build_Modbus_Frame(motors[i].slave_address, 0x10, motors[i].contrl_reg, 2, cmds, frame);
        
        if (len == 0) continue;

        // 1. 发送前，彻底清空上一轮的接收状态
        __disable_irq();
        Master_RX_CNT = 0;
        Master_FrameFlag = 0; // 必须清零，否则中断里存不进新数据
        __enable_irq();
        
        master_state = MASTER_WAIT_RESP;

        // 2. 发送指令
        RS485_MasterSendData(frame, len);

        // 3. 闭环等待当前电机的响应 (等待定时器触发帧结束标志，最大超时约30ms)
        uint8_t timeout = 0;
        while (Master_FrameFlag == 0 && timeout < 30) { 
            delay_ms(1);
            timeout++;
        }

        // 4. 检查响应
        uint8_t ret = Modbus_Check_Response_FromBuffer(motors[i].slave_address, motors[i].contrl_reg, NULL);
        if (results != NULL) results[i] = ret;
				
//				 if (ret == 0) {
//            printf("电机0x%02X 响应成功\r\n", motors[i].slave_address);
//            success_count++;
//        } else {
//            printf("电机0x%02X 响应未找到\r\n", motors[i].slave_address);
//        }
        
        if (ret == 0) {
            success_count++;
        }

        // 5. 字节间安全延时，让总线电平彻底恢复平静后再进行下一台电机的通讯
        delay_ms(5);
    }

    master_state = MASTER_IDLE;
    return count - success_count;
}
/**
 * @brief 同步控制多个电机（批量发送指令，指令间延时极短，实现同步启动）
 * @param motors 电机控制参数数组
 * @param count  电机数量
 */
void Sync_Motors_Control(MotorControlParams *motors, uint8_t count)
{
    
    // 清空主站接收缓冲区（不等待响应，所以可以清空）
    __disable_irq();
    Master_RX_CNT = 0;
    Master_FrameFlag = 0;
    __enable_irq();
    
    // 设置主站状态为等待响应
    master_state = MASTER_WAIT_RESP;
    
    // 批量发送所有指令，指令间仅保持极短延时（保证总线稳定）
    for (uint8_t i = 0; i < count; i++) {
        uint16_t cmds[2] = {motors[i].len_l, motors[i].len_h};
        uint8_t frame[256];
        uint8_t len = Build_Modbus_Frame(motors[i].slave_address, 0x10, motors[i].contrl_reg, 2, cmds, frame);
        if (len == 0) {
            printf("错误：构建电机%d帧失败，地址=0x%02X\r\n", i+1, motors[i].slave_address);
            continue;
        }
        
        printf("Sync Send to 0x%02X: ", motors[i].slave_address);
        for (uint8_t j = 0; j < len; j++) printf("%02X ", frame[j]);
        printf("\r\n");
				
				// 禁用从站接收，避免总线冲突
//				USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
        
        // 发送指令
        RS485_MasterSendData(frame, len);
				
				    // 重新启用从站接收
//				USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
        
        // 极短延时（100us），确保总线空闲，同时保持同步性
        delay_ms(50);
    }
    
    // 发送完成后，恢复主站状态为空闲（不等待响应）
    master_state = MASTER_IDLE;

    printf("所有电机指令已同步发出\r\n");
}

uint8_t Modbus_Check_Response_FromBuffer(uint8_t slave_addr, uint16_t start_reg, uint16_t *result)
{
    uint16_t idx = 0;
    while (idx + 8 <= Master_RX_CNT) {
        uint8_t *buf = Master_RX_BUFF + idx;

        // 检查地址
        if (buf[0] != slave_addr) {
            idx++;
            continue;
        }
        // 检查功能码（写多个寄存器响应应为 0x10）
        if (buf[1] != 0x10) {
            idx++;
            continue;
        }
        // 检查起始寄存器
        uint16_t resp_start = (buf[2] << 8) | buf[3];
        if (resp_start != start_reg) {
            idx++;
            continue;
        }
        // 检查 CRC
        uint16_t recv_crc = (buf[7] << 8) | buf[6];
        uint16_t calc_crc = Modbus_CRC16(buf, 6);
        if (recv_crc != calc_crc) {
            idx++;
            continue;
        }

        // 找到匹配帧，从缓冲区中移除该帧（8字节）
				if (idx + 8 < Master_RX_CNT) {
						memmove(Master_RX_BUFF, Master_RX_BUFF + idx + 8, Master_RX_CNT - idx - 8);
				}
				Master_RX_CNT -= (idx + 8);

        if (result) *result = 1;
        return 0;   // 成功
    }
    return 2;   // 未找到
}

/**
 * @brief 上电复位指定电机
 * @param slave_addr 电机从站地址
 * @param reg_addr   复位寄存器地址（例如0x50）
 * @param reset_value 复位值（例如8）
 * @return 0:成功, 其他:错误码
 */
uint8_t Motor_Reset(uint8_t slave_addr, uint16_t reg_addr, uint16_t reset_value)
{
    uint8_t ret;
    printf("正在复位电机 地址0x%02X...\r\n", slave_addr);
    // 调用06功能码写单个寄存器
    ret = Modbus_06_WriteSingleReg(slave_addr, reg_addr, reset_value);
    if (ret == 0) {
        printf("电机 地址0x%02X 复位成功\r\n", slave_addr);
    } else {
        printf("电机 地址0x%02X 复位失败，错误码 %d\r\n", slave_addr, ret);
    }
    return ret;
}


/**
 * @brief 检测指定电机是否堵转，若堵转则尝试恢复
 * @param slave_addr 电机从站地址
 * @param current_threshold_ma 堵转电流阈值（mA）
 * @return 1 表示发生堵转并执行了恢复，0 表示正常
 */
uint8_t Motor_CheckAndRecoverStall(uint8_t slave_addr, uint16_t current_threshold_ma)
{
    uint16_t current_ma = 0;
    uint8_t ret;

    // 1. 读取实时电流
    ret = Modbus_03_ReadHoldReg(slave_addr, REG_CURRENT, 1, &current_ma);
    if (ret != 0) {
        printf("读取电机0x%02X电流失败，错误码%d\n", slave_addr, ret);
        return 0;
    }

    // 2. 判断是否超过堵转阈值
    if (current_ma < current_threshold_ma) {
        return 0;   // 正常
    }

    // 3. 堵转发生，执行恢复流程
    printf("电机0x%02X检测到堵转，电流=%d mA（阈值=%d mA），开始恢复\n", 
           slave_addr, current_ma, current_threshold_ma);

    // 3.1 清除报警（写0x00A4任意值，例如0x0000）
    ret = Modbus_06_WriteSingleReg(slave_addr, REG_CLEAR_ALARM, 0x0000);
    if (ret != 0) {
        printf("清除报警失败，错误码%d\n", ret);
    } else {
        printf("报警已清除\n");
    }

    // 3.2 重新使能电机（写0x00D4 = 0x0000 使能）
    //    注意：如果电机因故障处于脱机状态，可先写1脱机再写0使能，
    //    但通常清除报警后直接使能即可。
    ret = Modbus_06_WriteSingleReg(slave_addr, REG_ENABLE, 0x0000);
    if (ret != 0) {
        printf("重新使能电机失败，错误码%d\n", ret);
    } else {
        printf("电机已重新使能\n");
    }

    // 3.3 可选：等待驱动器稳定
    delay_ms(100);

    // 4. 再次读取电流确认是否恢复正常（可选）
    ret = Modbus_03_ReadHoldReg(slave_addr, REG_CURRENT, 1, &current_ma);
    if (ret == 0 && current_ma < current_threshold_ma) {
        printf("电机0x%02X恢复成功，当前电流=%d mA\n", slave_addr, current_ma);
    } else {
        printf("电机0x%02X可能未完全恢复，电流=%d mA\n", slave_addr, current_ma);
    }

    return 1;
}

// ========================== 辅助函数 ==========================
/**
 * @brief 根据从站地址获取对应的复位函数指针
 * @param slave_addr 从站地址
 * @return 函数指针，若未匹配则返回 NULL
 */
static uint8_t (*GetResetFunction(uint8_t slave_addr))(void)
{
    switch (slave_addr) {
        case 0x02: return Battery_22;
        case 0x03: return Battery_15;
        case 0x05:
        case 0x06:
        case 0x07:
        case 0x08:
        case 0x09:
        case 0x0A:
        case 0x0B:
        case 0x0C: return LeaveCenter1;
        default:   return NULL;
    }
}

// ========================== 主任务函数 ==========================
/**
 * @brief 非阻塞堵转监控任务（计数轮询方式）
 * @note  在主循环中周期性调用，每次调用处理一个电机。
 *        达到计数阈值后启动一轮轮询，检测到堵转则按顺序执行所有复位函数。
 */

volatile uint8_t stall_slave_addr = 0;
//void MotorStallMonitorTask(void)
//{
//    // ---------- 1. 启动轮询（基于计数） ----------
//    if (!stall_poll_active) {
//        stall_poll_counter++;
//        if (stall_poll_counter >= STALL_POLL_THRESHOLD &&
//        master_state == MASTER_IDLE && !Sequence_IsBusy()) {
//            stall_poll_counter = 0;                // 复位计数
//            stall_poll_active = 1;                 // 进入轮询状态
//            stall_motor_index = 0;                 // 从第一个电机开始
//            // 注意：不在此处清除 stall_triggered，由外部控制
//            // printf("启动堵转轮询\n");
//        } else {
//            return;   // 未达到阈值，本次不执行
//        }
//    }

//    // 如果已经触发过堵转且未恢复，跳过检测（避免重复执行复位）
//    if (stall_triggered) {
//        // 若需自动恢复，可在此添加条件清除 stall_triggered，但建议由外部调用 ClearStallTrigger()
//        return;
//    }

//    // ---------- 2. 轮询执行：检测当前索引对应的电机 ----------
//    if (stall_motor_index >= MOTORCOUNT) {
//        // 本轮所有电机检测完毕，且未触发堵转，结束轮询
//        stall_poll_active = 0;
//        // printf("本轮堵转检测结束，未发现堵转\n");
//        return;
//    }

//    uint8_t slave = stall_motor_addr_list[stall_motor_index];
//    uint16_t current_ma = 0;
//    uint8_t ret;

//    // 读取实时电流
//		MasterBusy_Acquire();
//    ret = Modbus_03_ReadHoldReg(slave, REG_CURRENT, 1, &current_ma);
//		MasterBusy_Release();
//		printf("ret=%d, current=%d mA\n", ret, current_ma);
//    if (ret != 0) {
//        printf("读取电机0x%02X电流失败，错误码%d\r\n", slave, ret);
//        // 读取失败则跳过该电机，继续下一个
//        stall_motor_index++;
//        return;
//    }
//		
//		if (current_ma >= STALL_CURRENT_THRESHOLD_MA) {
//    // 记录堵转电机和当前状态
//    stall_triggered = 1;
//    stall_slave_addr = slave;
//    stall_poll_active = 0;  // 结束本轮轮询
//    // 不在这里执行 Battery_22 等函数，而是返回
//    return;
//		}

//    // 当前电机正常，移动到下一个
//    stall_motor_index++;
//}



void MotorStallMonitorTask(void)
{
    // ---------- 计数控制：每200次主循环执行一次检测 ----------
    static uint16_t stall_poll_counter = 0;
    stall_poll_counter++;
    if (stall_poll_counter < STALL_POLL_THRESHOLD) {  // STALL_POLL_THRESHOLD = 200
        return;  // 未达到计数阈值，本次不执行
    }
    stall_poll_counter = 0;  // 复位计数  
  
		// 仅在序列执行时检测
    if (!Sequence_IsBusy()) {
        return;
    }

    // 如果已经触发过堵转且未恢复，跳过检测
    if (stall_triggered) {
        return;
    }

    uint8_t current_seq = Sequence_GetCurrentId();
    uint8_t current_step = Sequence_GetCurrentStep();

    // 获取当前步骤的电机列表
    const uint8_t *motor_list = GetMotorListForCurrentStep();
    if (motor_list == NULL) {
        return;   // 该步骤无需检测
    }

    // 遍历该步骤的所有电机
    while (*motor_list != 0) {
        uint8_t slave = *motor_list++;
        uint16_t current_ma = 0;
        uint8_t ret;

        // 读取电流（总线互斥）
        MasterBusy_Acquire();
        ret = Modbus_03_ReadHoldReg(slave, REG_CURRENT, 1, &current_ma);
        MasterBusy_Release();

        printf("序列%d 步骤%d 电机0x%02X 电流=%d mA\n", current_seq, current_step, slave, current_ma);
        if (ret != 0) {
            printf("读取电机0x%02X电流失败，错误码%d\n", slave, ret);
            continue;   // 跳过该电机
        }

        // ---------- 特殊处理：序列8/9（开机/关机），电机0x02电流异常 ----------
        if ((current_seq == 8 || current_seq == 9) && slave == 0x02) {
            if (current_ma < 100 || current_ma > 1000) {
                printf("特殊序列0x%02X 步骤%d：电机0x02电流异常 %d mA，发送急停\n", current_seq, current_step, current_ma);
                uint8_t stop_ret = Modbus_06_WriteSingleReg(0x02, 0x00C8, 0x0100); // 急停
                if (stop_ret != 0) {
                    printf("急停指令发送失败，错误码 %d\n", stop_ret);
                } else {
                    printf("急停指令发送成功\n");
                }
                // 仅在步骤7（索引7）时阻塞延时2秒
                if (current_step == 7) {
                    printf("步骤7：保持2秒\n");
                    delay_ms(2000);
                    printf("保持2秒结束\n");
                }
                // 继续检测下一个电机
                continue;
            }
        }

        // ---------- 通用堵转检测 ----------
        if (current_ma >= STALL_CURRENT_THRESHOLD_MA) {
            stall_triggered = 1;
            stall_slave_addr = slave;
            printf("电机0x%02X堵转，电流=%d mA\n", slave, current_ma);
            // 发现堵转，停止本轮检测（由主循环处理复位）
            return;
        }
    }
}

void AlarmPoll_Init(void)
{
    last_alarm_poll_time = GetTick() - ALARM_POLL_INTERVAL_MS - 100; // 确保立即触发
}

/**
 * @brief 读取电机当前位置（32位）
 * @param slave_addr 从站地址
 * @param pos 输出位置指针
 * @return 0成功，非0失败
 */
uint8_t ReadMotorPosition(uint8_t slave_addr, int32_t *pos)
{
    uint16_t regs[2];
    uint8_t ret = Modbus_03_ReadHoldReg(slave_addr, 0x0004, 2, regs);
    if (ret == 0) {
        *pos = (int32_t)((regs[1] << 16) | regs[0]);  // 低16位在前，高16位在后
    }
    return ret;
}

/**
 * @brief 发送停止指令给多个电机
 * @param addrs 从站地址数组
 * @param count 电机数量
 */
void StopMotors(uint8_t *addrs, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        Modbus_06_WriteSingleReg(addrs[i], 0x00C8, 0x0100);
    }
}

/**
 * @brief 等待指定时间，期间持续监测电机位置，若连续多次不变则判定堵转
 * @param addrs     电机从站地址列表
 * @param count     电机数量
 * @param timeout_ms  等待总时间（毫秒）
 * @param check_interval_ms  检查间隔（毫秒）
 * @param stall_threshold  连续位置不变次数阈值（默认3次）
 * @return 0成功，1检测到堵转
 */
uint8_t WaitWithPositionCheck(uint8_t *addrs, uint8_t count, 
                              uint32_t timeout_ms, 
                              uint32_t check_interval_ms,
                              uint8_t stall_threshold,
                              int32_t *target_pos)
{
     uint32_t start = GetTick();
    int32_t last_pos[8] = {0};
    uint8_t same_cnt[8] = {0};
    uint8_t first_read = 1;

    while ((GetTick() - start) < timeout_ms) {
        // 读取所有电机位置
        int32_t curr_pos[8];
        uint8_t read_fail = 0;
        for (uint8_t i = 0; i < count; i++) {
            if (ReadMotorPosition(addrs[i], &curr_pos[i]) != 0) {
                printf("读取电机0x%02X位置失败\n", addrs[i]);
                read_fail = 1;
                break;
            }
        }
        if (read_fail) {
            delay_ms(check_interval_ms);
            continue;
        }

        // 堵转检测
        if (first_read) {
            for (uint8_t i = 0; i < count; i++) {
                last_pos[i] = curr_pos[i];
                same_cnt[i] = 0;
            }
            first_read = 0;
        } else {
            uint8_t stall_detected = 0;
            for (uint8_t i = 0; i < count; i++) {
                if (curr_pos[i] == last_pos[i]) {
                    same_cnt[i]++;
                    if (same_cnt[i] >= stall_threshold) {
                        printf("电机0x%02X位置停滞（连续%d次不变），判定堵转！\n", 
                               addrs[i], stall_threshold);
                        stall_detected = 1;
                    }
                } else {
                    same_cnt[i] = 0;
                }
                last_pos[i] = curr_pos[i];
            }
            if (stall_detected) {
                StopMotors(addrs, count);
                return 1;   // 堵转
            }
        }

        if ((GetTick() - start) < timeout_ms) {
            delay_ms(check_interval_ms);
        }
    }

    // 等待结束，检查到位情况
		if (target_pos != NULL) 
		{
						int32_t curr_pos[8];
						uint8_t all_in_pos = 1;
						for (uint8_t i = 0; i < count; i++) {
								if (ReadMotorPosition(addrs[i], &curr_pos[i]) != 0) {
										printf("读取电机0x%02X位置失败\n", addrs[i]);
										return 2;
								}
								int32_t diff = curr_pos[i] - target_pos[i];
								if (diff < 0) diff = -diff;
								if (diff > 200) {
										printf("电机0x%02X未到位，当前位置=%ld, 目标=%ld, 偏差=%ld\n", 
													 addrs[i], curr_pos[i], target_pos[i], curr_pos[i] - target_pos[i]);
										all_in_pos = 0;
								}
						}
						if (!all_in_pos) {
								return 2; // 到位失败
						}
		}

    return 0;   // 成功
}
