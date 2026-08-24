/*==================================================================================
This document is the property of Nanjing Kangni Mechanical & Electrical Co.,Ltd.. 
It may not be reproduced or disclosed to third party without prior to authorisation.
Module name: usart.c
Description: serial port module, used to communication.
Function List: 
		(Procedure name,    Version)
		fputc               V0100
		CnfgrUsart          V0100
		USART1_IRQHandler   V0100
Target: STM32F103VET6
Status: TESTED
History: Wan Lei, V0100-0000, 20180104
===================================================================================*/

/*==================================================================================
   list of header files
===================================================================================*/
#include "project.h"

u8 usart_rx_temporary[40]; //数据保存暂存器,最多能够缓存40个字节
u8 usartrxbuf_pagebuf = 0;	 //最上面接受缓存的页码（5）缓存
u8 usart_rd_len = 0;       //有用信息的数据长度
u8 usart_rd_lentemp = 0;	 //用来记录已读取的数据长度
u8 usart_rx_enableflag = 0; //接收状态标记
u8 usart_rx_lenrightflag = 0; //数据长度校验位正确标志
u8 usart_rx_successflag = 0; //成功接收到数据信息
u8 usart_rd_lencount = 0;     //接收到的数据
uint8_t aTxBuffer[10];
//u8 RS485_FrameFlag = 0; //帧结束标记
//u16 RS485_Frame_Distance = 500;//数据帧最小间隔（ms),超过此时间则认为是下一帧

//u8 RS485_RX_BUFF[30];//接收缓冲区2048字节
//u16 RS485_RX_CNT = 0; //接收计数器

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
// 使用weak属性声明，避免与标准库冲突
__attribute__((weak)) struct __FILE 
{ 
    int handle; 
}; 

__attribute__((weak)) FILE __stdout;    
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
    x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	UART4->DR = (u8) ch;      	
	while((UART4->SR&0X40)==0);//循环发送,直到发送完毕   

	return ch;
}
#endif 
/*使用microLib的方法*/ 
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
unsigned char USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
unsigned short USART_RX_STA=0;       //接收状态标记	   
void CnfgrUsart(unsigned long bound)   //usart_init
{
    GPIO_InitTypeDef GPIO_InitStructure;
	  USART_InitTypeDef USART_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // USART1使用PA9/PA10
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  // USART1挂在APB2总线上
	
	  USART_DeInit(USART1);
	  //USART1_TX   PB.6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PA.10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//复用推挽输出
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // 推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);
   
	  //USART1_RX	  PB.7
    GPIO_InitStructure.GPIO_Pin = USART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//浮空输入
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);  
		
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART1);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART1);

    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1 ;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
	  //USART 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	  //USART_InitStructure.USART_Mode = USART_Mode_Rx ;	//收发模式
    USART_Init(USART1, &USART_InitStructure); //初始化串口
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
    USART_Cmd(USART1, ENABLE);                    //使能串口 
}

/*==================================================================================================
- 函数名称:uart_init
- 功能描述:串口4初始化函数
- 运行位置:-
- 调用函数:-
- 版本信息:V0100-0000,LiuXiao,20211013
==================================================================================================*/
void uart4_init(u32 bound)
{
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能USART4时钟
 
	//串口4对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOA2复用为USART2
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOA3复用为USART2
	
	//USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOA2与GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口2
	
  USART_Cmd(UART4, ENABLE);  //使能串口2 
	
	USART_ClearFlag(UART4, USART_FLAG_TC);
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =4;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}


// void USART3_IRQHandler(void)                	//串口1中断服务程序
// 	{
// 	u8 Res;
// #ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
// 	OSIntEnter();    
// #endif
// 	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
// 		{
// 		Res =USART_ReceiveData(USART3);//(USART1->DR);	//读取接收到的数据
// 		
// 		if((USART_RX_STA&0x8000)==0)//接收未完成
// 			{
// 			if(USART_RX_STA&0x4000)//接收到了0x0d
// 				{
// 				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
// 				else USART_RX_STA|=0x8000;	//接收完成了 
// 				}
// 			else //还没收到0X0D
// 				{	
// 				if(Res==0x0d)USART_RX_STA|=0x4000;
// 				else
// 					{
// 					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
// 					USART_RX_STA++;
// 					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
// 					}		 
// 				}
// 			}   		 
//      } 
// #ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
// 	OSIntExit();  											 
// #endif
// } 

//void CnfgrTimer3(void)
//{
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;

//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //TIM7时钟使能

//    //TIM7初始化设置
//    TIM_TimeBaseStructure.TIM_Period = RS485_Frame_Distance * 10; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
//    TIM_TimeBaseStructure.TIM_Prescaler = 7200; //设置用来作为TIMx时钟频率除数的预分频值 设置计数频率为10kHz
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
//    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
//    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //TIM7 允许更新中断

//    //TIM7中断分组配置
//    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM7中断
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //先占优先级2级
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级3级
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
//    NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

//}


void USART3_IRQHandler(void)                	//串口1中断服务程序
{
		u8 i, res, check_temp;

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  		//接收中断，每接收一个字节（8位二进制数据），中断一次,每次中断（即每接收一个数据）都执行一次下列程序
    {
        res = USART_ReceiveData(USART3);												//读取接收到的数据
//        testI = res;
//			  //USART_SendData(USART3, res);
//			  
//			
//        usart_rd_lencount++;

//        if((res == 0xee) && (usart_rx_enableflag == 0))							//当接受到包头(0xee)数据并且还没有成功接收完数据信息
//            usart_rx_enableflag = 1;											//说明这是包头，启动接收数据标志，进入数据接收阶段

//        if(usart_rx_enableflag == 0)
//        {
//            usart_rd_lencount = 0;
//        }

//        if((usart_rx_enableflag == 1) && (usart_rd_lencount > 1))  													//到接受数据标志置位时，接受数据
//        {
//            //res=USART_ReceiveData(USART3);										//读取串口标志
//            if(usart_rd_lentemp == 0)  													//包头后第一个数据为需要传输的数据的长度
//            {
//                usart_rd_len = res;															//读取数据的长度

//                usart_rd_lentemp++;

//                if(usart_rd_len >= 40)
//                {
//                    usart_rx_lenrightflag = 0;										//数据长度校验清零
//                    usart_rx_successflag = 0;											//数据接收成功标志清零
//                    usart_rx_enableflag = 0;											//数据接收完成，数据接收启动标志清零
//                    usart_rd_len = 0;															//数据长度清零
//                    usart_rd_lentemp = 0;													//数据长度暂存器清零
//                    usart_rd_lencount = 0;
//                }
//            }

//            else if(usart_rd_lentemp == usart_rd_len + 1)  			//当读取到第usart_rd_lentemp+1个数据时，校验是否是长度信息的反码
//            {
//                //check_temp=~usart_rd_len;											//取数据长度校验位的反码
//                //if(res==check_temp)														//当数据长度校验正确时
//                if(res == usart_rd_len)
//                {
//                    usart_rx_lenrightflag = 1;										//数据长度校验标志置一

//                    usart_rd_lentemp++;
//                }
//                else
//                {
//                    //当数据长度校验错误时
//                    usart_rx_lenrightflag = 0;										//数据长度校验清零
//                    usart_rx_successflag = 0;											//数据接收成功标志清零
//                    usart_rx_enableflag = 0;											//当数据长度校验错误时，数据接收启动标志清零
//                    usart_rd_len = 0;															//数据长度清零
//                    usart_rd_lentemp = 0;   											//数据长度暂存器清零
//                    usart_rd_lencount = 0;
//                }
//            }
//            else if(usart_rd_lentemp == usart_rd_len + 2)  				//当读取到第usart_rd_lentemp+2个数据时，校验包尾是否正确
//            {
//                if((res == 0xef) && (usart_rx_lenrightflag == 1))  	//如果包尾数据与长度校验都正确
//                {
//                    usart_rx_lenrightflag = 0;										//数据长度校验清零
//                    usart_rx_successflag = 1;											//数据接收成功标志置一
//                    usart_rx_enableflag = 0;											//数据接收完成，数据接收启动标志清零
//                    usart_rd_len = 0;															//数据长度清零
//                    usart_rd_lentemp = 0; 												//数据长度暂存器清零
//                    usart_rd_lencount = 0;

//                    usart_rd_lentemp++;
//									
//									 
//                }
//                else
//                {
//                    //当包尾数据校验错误时
//                    usart_rx_lenrightflag = 0;										//数据长度校验清零
//                    usart_rx_successflag = 0;											//数据接收成功标志清零
//                    usart_rx_enableflag = 0;											//数据接收完成，数据接收启动标志清零
//                    usart_rd_len = 0;															//数据长度清零
//                    usart_rd_lentemp = 0;													//数据长度暂存器清零
//                    usart_rd_lencount = 0;
//                }
//            }
//            else
//            {

//                usart_rx_temporary[usart_rd_lentemp - 1] = res;	//当usart_rd_lentemp为数据段时，将数据存到串口数据接收寄存器中
//                usart_rd_lentemp++;
//            }

//            //usart_rd_lentemp++;																	//每次记录数据，数据长度暂存器自加
//            if(usart_rx_successflag == 1)  												//如果成功接收到信息数据，将缓存usart_rx_temporary[]内的数据传递给usart_rx_buf[][]
//            {
//                usart_rd_lentemp = 0;
//                usart_rx_successflag = 0;

//                switch(usart_rx_temporary[0])
//                {
//                case 1:
//                {
//									if(lim.uFMotorARREnable == 1)
//									{							
//									 //LED_EX(LED1);
//                   MOTOR1_FR();
//									 lim.uFMotorASt = 1;
//									}
//									if(lim.uFMotorCRREnable == 1)
//									{		
//									 MOTOR3_FR();
//									 lim.uFMotorCSt = 1;
//									}
//									 lim.uFMotorAEnable = 1;
//									 lim.uFMotorCEnable = 1;
//				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
//                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
//                   TIM_Cmd(TIM3, ENABLE); //开始计时
//									

//                };

//                break;

//                case 2:
//                {
//       
//									if(lim.uFMotorAEnable == 1)
//									{	
//										
//										MOTOR1_RR();
//										lim.uFMotorASt = 2;
//									}
//									
//									if(lim.uFMotorCEnable == 1)
//									{	
//										MOTOR3_RR();
//										lim.uFMotorCSt = 2;
//										
//									}
//                   
//									 
//									 lim.uFMotorARREnable = 1;
//									 lim.uFMotorCRREnable = 1;
//									 
//				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
//                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
//                   TIM_Cmd(TIM3, ENABLE); //开始计时
//                };

//                break;

//                case 3:
//                {
//						      if(lim.uFMotorBEnable == 1)
//									{		
//                   MOTOR2_FR();
//									 lim.uFMotorBSt = 2;
//									}
//						      if(lim.uFMotorDEnable == 1)
//									{										
//									 MOTOR4_FR();
//									 lim.uFMotorDSt = 2;
//									}
//									
//									 lim.uFMotorBRREnable = 1;
//									 lim.uFMotorDRREnable = 1;
//				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
//                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
//                   TIM_Cmd(TIM3, ENABLE); //开始计时
//									

//                };

//                break;

//                case 4:
//                {
//									
//									if(lim.uFMotorBRREnable == 1)
//									{	
//										
//										MOTOR2_RR();
//										lim.uFMotorBSt = 1;
//									}
//									
//									if(lim.uFMotorDRREnable == 1)
//									{	
//										MOTOR4_RR();
//										lim.uFMotorDSt = 1;
//										
//									}									
//									
////                   MOTOR2_RR();
////									 MOTOR4_RR();
//									 lim.uFMotorBEnable = 1;
//									 lim.uFMotorDEnable = 1;
////									 lim.uFMotorBSt = 1;
////									 lim.uFMotorDSt = 1;
//				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
//                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
//                   TIM_Cmd(TIM3, ENABLE); //开始计时    

//                };

//                break;

//                case 5:
//                {
//									if(lim.uFMotorEEnable == 1)
//									{
//                   MOTOR5_FR();
//									 //MOTOR4_FR();
//										lim.uFMotorERREnable = 1;
//									 lim.uFMotorESt = 2;
//				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
//                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
//                   TIM_Cmd(TIM3, ENABLE); //开始计时
//									}
//                };

//                break;

//                case 6:
//                {
//									if(lim.uFMotorERREnable == 1)
//									{
//                    MOTOR5_RR();
//									  lim.uFMotorEEnable = 1;
//									 //MOTOR4_FR();
//									 lim.uFMotorESt = 1;
//				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
//                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
//                   TIM_Cmd(TIM3, ENABLE); //开始计时
//										}
//                };

//                break;

//                case 7:
//                {
//                   GPIO_SetBits(OPORT06, OPORT06_PIN);
//									 GPIO_SetBits(OPORT04, OPORT04_PIN);
//									 lim.uFMotorO6St = 1;
//				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
//                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
//                   TIM_Cmd(TIM3, ENABLE); //开始计时									
//                };

//                break;

//                case 8:
//                {
//                   GPIO_SetBits(OPORT07, OPORT07_PIN);
//									 GPIO_SetBits(OPORT04, OPORT04_PIN);
//									 lim.uFMotorO7St = 1;
//				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
//                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
//                   TIM_Cmd(TIM3, ENABLE); //开始计时		
//                };

//                break;

//                case 9:
//                {

//                };

//                break;
//                case 0x20:
//                {


//                };

//                break;

//                case 0x21:
//                {

//                };

//                break;
//                default:
//                {

//                }

//                }

//            }
//        }

    }
}




//void TIM3_IRQHandler(void)
//{
//    if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
//    {
//        TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除中断标志
//        TIM_Cmd(TIM3, DISABLE); //停止定时器
//        //GPIO_ResetBits(USART_RE, USART_RE_PIN); //发送状态

//        RS485_FrameFlag = 1; //置位帧结束标记
////        RS485_Service();
////        GPIO_SetBits(USART_RE, USART_RE_PIN); //默认接收状态
//        RS485_RX_CNT=0;
//			  if(lim.uFMotorASt == 1|| lim.uFMotorASt == 2)
//			  {
//			      MOTOR1_STOP();
//					  //MOTOR3_STOP();
//			 		  lim.uFMotorASt = 0;
//					GPIO_ResetBits(OPORT01, OPORT01_PIN);
//			  }
//				
//			  if(lim.uFMotorCSt == 1|| lim.uFMotorCSt == 2)
//			  {
//			      //MOTOR1_STOP();
//					  MOTOR3_STOP();
//			 		  lim.uFMotorCSt = 0;
//					GPIO_ResetBits(OPORT01, OPORT01_PIN);
//			  }
//				
//			  if(lim.uFMotorBSt == 1 || lim.uFMotorBSt == 2)
//			  {
//			      MOTOR2_STOP();
//					 //MOTOR4_STOP();
//			 		  lim.uFMotorBSt = 0;
//					GPIO_ResetBits(OPORT02, OPORT02_PIN);
//			  }
//				
//			  if(lim.uFMotorDSt == 1 || lim.uFMotorDSt == 2)
//			  {
//			      //MOTOR2_STOP();
//					  MOTOR4_STOP();
//			 		  lim.uFMotorDSt = 0;
//					GPIO_ResetBits(OPORT02, OPORT02_PIN);
//			  }				
//				
//			  if(lim.uFMotorESt == 1 || lim.uFMotorESt == 2)
//			  {
//			      MOTOR5_STOP();
//					  lim.uFMotorESt = 0;
//					  GPIO_ResetBits(OPORT03, OPORT03_PIN);
//			  }
//				
//			  if(lim.uFMotorO6St == 1)
//			  {
//			      GPIO_SetBits(OPORT06, OPORT06_PIN);
//				  	//GPIO_ResetBits(OPORT04, OPORT04_PIN);
//					  lim.uFMotorO6St = 0;
//			  }
//				
//			  if(lim.uFMotorO7St == 1)
//			  {
//			      GPIO_SetBits(OPORT07, OPORT07_PIN);
//				  	//GPIO_ResetBits(OPORT04, OPORT04_PIN);
//					  lim.uFMotorO7St = 0;
//			  }
//			  
////						   for(testI = 0;testI<10;testI++)
////			printf("\r\n%d :%d ",testI,RS485_RX_BUFF[testI]);
//    }
//}
U8 TX_CheckSum(U8 *buf, U8 len) //buf为数组，len为数组长度
{ 
    U8 i, ret = 0;
 
    for(i=0; i<len; i++)
    {
        ret += *(buf++);
    }
    return ret;
}

u8 NRF24L01_RxProcess(u8 *rxbuf)
{


    if(rxbuf[3] == TX_CheckSum(rxbuf,3) && rxbuf[0] == 0xbb && rxbuf[2] == 0xcc)
    {
        switch(rxbuf[1])
        {
            case 0x11:
            {
						    if(lim.uFMotorARREnable == 1)
								{							
									 //LED_EX(LED1);
                   MOTOR1_FR();
									 lim.uFMotorASt = 1;
								}
								if(lim.uFMotorCRREnable == 1)
								{		
									 MOTOR3_FR();
									 lim.uFMotorCSt = 1;
								}
								lim.uFMotorAEnable = 1;
								lim.uFMotorCEnable = 1;
				        TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
                TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
                TIM_Cmd(TIM3, ENABLE); //开始计时	
                memset(rxbuf, 0, 4);								
              };
                break;

              case 0x22:
              {
       
									if(lim.uFMotorAEnable == 1)
									{	
										
										MOTOR1_RR();
										lim.uFMotorASt = 2;
									}
									
									if(lim.uFMotorCEnable == 1)
									{	
										MOTOR3_RR();
										lim.uFMotorCSt = 2;
										
									}
                   
									 lim.uFMotorARREnable = 1;
									 lim.uFMotorCRREnable = 1;
									 
				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
                   TIM_Cmd(TIM3, ENABLE); //开始计时
									 memset(rxbuf, 0, 4);
                };

                break;

                case 0x33:
                {
						      if(lim.uFMotorBEnable == 1)
									{		
                   MOTOR2_FR();
									 lim.uFMotorBSt = 2;
									}
						      if(lim.uFMotorDEnable == 1)
									{										
									 MOTOR4_FR();
									 lim.uFMotorDSt = 2;
									}
									
									 lim.uFMotorBRREnable = 1;
									 lim.uFMotorDRREnable = 1;
				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
                   TIM_Cmd(TIM3, ENABLE); //开始计时
									 memset(rxbuf, 0, 4);
									

                };

                break;

                case 0x44:
                {
									
									if(lim.uFMotorBRREnable == 1)
									{	
										
										MOTOR2_RR();
										lim.uFMotorBSt = 1;
									}
									
									if(lim.uFMotorDRREnable == 1)
									{	
										MOTOR4_RR();
										lim.uFMotorDSt = 1;
										
									}									
									
//                   MOTOR2_RR();
//									 MOTOR4_RR();
									 lim.uFMotorBEnable = 1;
									 lim.uFMotorDEnable = 1;
//									 lim.uFMotorBSt = 1;
//									 lim.uFMotorDSt = 1;
				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
                   TIM_Cmd(TIM3, ENABLE); //开始计时  
                   memset(rxbuf, 0, 4);									

                };

                break;

                case 0x55:
                {
									if(lim.uFMotorEEnable == 1)
									{
                   MOTOR5_FR();
									 //MOTOR4_FR();
										lim.uFMotorERREnable = 1;
									 lim.uFMotorESt = 2;
				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
                   TIM_Cmd(TIM3, ENABLE); //开始计时
									}
									memset(rxbuf, 0, 4);
                };

                break;

                case 0x66:
                {
									if(lim.uFMotorERREnable == 1)
									{
                    MOTOR5_RR();
									  lim.uFMotorEEnable = 1;
									 //MOTOR4_FR();
									 lim.uFMotorESt = 1;
				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
                   TIM_Cmd(TIM3, ENABLE); //开始计时
										}
									memset(rxbuf, 0, 4);
                };

                break;

                case 0x77:
                {
                   GPIO_ResetBits(OPORT06, OPORT06_PIN);
									 GPIO_SetBits(OPORT04, OPORT04_PIN);
									 lim.uFMotorO6St = 1;
									tmr.TIMER_LEDDELAY_100MS = -2;
				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
                   TIM_Cmd(TIM3, ENABLE); //开始计时	
                   memset(rxbuf, 0, 4);									
                };

                break;

                case 0x88:
                {
                   GPIO_ResetBits(OPORT07, OPORT07_PIN);
									 GPIO_SetBits(OPORT04, OPORT04_PIN);
									 lim.uFMotorO7St = 1;
									tmr.TIMER_LEDDELAY_100MS = -2;
				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
                   TIM_Cmd(TIM3, ENABLE); //开始计时	
                   memset(rxbuf, 0, 4);									
                };

                break;

                case 0x99:
                {

                };

                break;
                case 0xaa:
                {


                };

                break;

                case 0xbb:
                {

                };

                break;
                default:
                {

                }

                }
    }

}

void Fengmingqi(void)
{
	if(lim.uFMotorO7St == 0 || lim.uFMotorO6St == 0)
	{
					  if(tmr.TIMER_LEDDELAY_100MS == -2)
						{
						     tmr.TIMER_LEDDELAY_100MS = 50;
						}
						if(tmr.TIMER_LEDDELAY_100MS == 0)
						{
						
								 GPIO_ResetBits(OPORT04, OPORT04_PIN);
					       //lim.uFMotorO7St = 0;
							   //tmr.TIMER_LEDDELAY_100MS = -1;
						}
	}


}

void WirelessCommProcess(u8 *rxbuf)
{
	
		u8 ucRecvCCDataLenth = 0;
	  u8 ucCheckSumValue = 0;
		
//	   ucRecvCCDataLenth = CC1101_Rx_Packet(rxbuf);
	   printf("\r\n%d,%d,%d",rxbuf[0],rxbuf[1],ucRecvCCDataLenth);//lmc.iMHallcnt wRMtrCurrent
		if(4 == ucRecvCCDataLenth && rxbuf[3] == TX_CheckSum(rxbuf,3) && rxbuf[0] == 0xbb && rxbuf[2] == 0xcc)
	  {
        switch(rxbuf[1])
        {
            case 0x11:
            {
						    if(lim.uFMotorARREnable == 1)
								{							
									 //LED_EX(LED1);
                   MOTOR1_FR();
									 lim.uFMotorASt = 1;
								}
								if(lim.uFMotorCRREnable == 1)
								{		
									 MOTOR3_FR();
									 lim.uFMotorCSt = 1;
								}
								lim.uFMotorAEnable = 1;
								lim.uFMotorCEnable = 1;
				        TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
                TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
                TIM_Cmd(TIM3, ENABLE); //开始计时	
                memset(rxbuf, 0, 4);
//                CC1101_Set_Mode(RX_MODE);								
              };
                break;

              case 0x22:
              {
       
									if(lim.uFMotorAEnable == 1)
									{	
										
										MOTOR1_RR();
										lim.uFMotorASt = 2;
									}
									
									if(lim.uFMotorCEnable == 1)
									{	
										MOTOR3_RR();
										lim.uFMotorCSt = 2;
										
									}
                   
									 lim.uFMotorARREnable = 1;
									 lim.uFMotorCRREnable = 1;
									 
				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
                   TIM_Cmd(TIM3, ENABLE); //开始计时
									 memset(rxbuf, 0, 4);
                };

                break;

                case 0x33:
                {
						      if(lim.uFMotorBEnable == 1)
									{		
                   MOTOR2_FR();
									 lim.uFMotorBSt = 2;
									}
						      if(lim.uFMotorDEnable == 1)
									{										
									 MOTOR4_FR();
									 lim.uFMotorDSt = 2;
									}
									
									 lim.uFMotorBRREnable = 1;
									 lim.uFMotorDRREnable = 1;
				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
                   TIM_Cmd(TIM3, ENABLE); //开始计时
									 memset(rxbuf, 0, 4);
									

                };

                break;

                case 0x44:
                {
									
									if(lim.uFMotorBRREnable == 1)
									{	
										
										MOTOR2_RR();
										lim.uFMotorBSt = 1;
									}
									
									if(lim.uFMotorDRREnable == 1)
									{	
										MOTOR4_RR();
										lim.uFMotorDSt = 1;
										
									}									
									
//                   MOTOR2_RR();
//									 MOTOR4_RR();
									 lim.uFMotorBEnable = 1;
									 lim.uFMotorDEnable = 1;
//									 lim.uFMotorBSt = 1;
//									 lim.uFMotorDSt = 1;
				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
                   TIM_Cmd(TIM3, ENABLE); //开始计时  
                   memset(rxbuf, 0, 4);									

                };

                break;

                case 0x55:
                {
									if(lim.uFMotorEEnable == 1)
									{
                   MOTOR5_FR();
									 //MOTOR4_FR();
										lim.uFMotorERREnable = 1;
									 lim.uFMotorESt = 2;
				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
                   TIM_Cmd(TIM3, ENABLE); //开始计时
									}
									memset(rxbuf, 0, 4);
                };

                break;

                case 0x66:
                {
									if(lim.uFMotorERREnable == 1)
									{
                    MOTOR5_RR();
									  lim.uFMotorEEnable = 1;
									 //MOTOR4_FR();
									 lim.uFMotorESt = 1;
				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
                   TIM_Cmd(TIM3, ENABLE); //开始计时
										}
									memset(rxbuf, 0, 4);
                };

                break;

                case 0x77:
                {
                   GPIO_ResetBits(OPORT06, OPORT06_PIN);
									 GPIO_SetBits(OPORT04, OPORT04_PIN);
									 lim.uFMotorO6St = 1;
									tmr.TIMER_LEDDELAY_100MS = -2;
				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
                   TIM_Cmd(TIM3, ENABLE); //开始计时	
                   memset(rxbuf, 0, 4);									
                };

                break;

                case 0x88:
                {
                   GPIO_ResetBits(OPORT07, OPORT07_PIN);
									 GPIO_SetBits(OPORT04, OPORT04_PIN);
									 lim.uFMotorO7St = 1;
									tmr.TIMER_LEDDELAY_100MS = -2;
				           TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
                   TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
                   TIM_Cmd(TIM3, ENABLE); //开始计时	
                   memset(rxbuf, 0, 4);									
                };

                break;

                case 0x99:
                {

                };

                break;
                case 0xaa:
                {


                };

                break;

                case 0xbb:
                {

                };

                break;
                default:
                {

                }

                }			
			
	  }

}
//void USART3_IRQHandler(void)                	//串口1中断服务程序
//{
//    u8 res;
//    u8 err;

//    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//    {
//        if(USART_GetFlagStatus(USART3, USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE))
//        {
//            err = 1; //检测到噪音、帧错误或校验错误
//        }
//        else err = 0;

////			          LED0=0;
//        res = USART_ReceiveData(USART3); //读接收到的字节，同时相关标志自动清除

//        if((RS485_RX_CNT < 2047) && (err == 0))
//        {
//            RS485_RX_BUFF[RS485_RX_CNT] = res;
//            RS485_RX_CNT++;

//            TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除定时器溢出中断
//            TIM_SetCounter(TIM3, 0); //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
//            TIM_Cmd(TIM3, ENABLE); //开始计时
//        }

//    }
//}
#endif	
/*==================================================================================
     the end of file
===================================================================================*/
