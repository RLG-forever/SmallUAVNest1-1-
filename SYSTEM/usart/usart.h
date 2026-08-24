/*==================================================================================
This document is the property of Nanjing Kangni Mechanical & Electrical Co., Ltd..
It may not be reproduced or disclosed to third party without prior to authorisation.
 * header file name: usart.h      
 * heafer file description: define the date type and initalization used by the timer module
 * service condition: use in searial module
 * version information: Wan Lei, V0100-0000, 20180104
===================================================================================*/

/*==================================================================================
     avoid redefinition
===================================================================================*/
#ifndef __USART_H
#define __USART_H
#define USART_REC_LEN  			1  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收  	
extern unsigned char  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern unsigned short USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void CnfgrUsart(unsigned long bound);
// void USART3_IRQHandler(void); 
extern  unsigned char i;
extern	unsigned char j;


typedef struct
{
     uint8_t uFMotorASt;
	   uint8_t uFMotorBSt;
	   uint8_t uFMotorCSt;
	   uint8_t uFMotorDSt;
	   uint8_t uFMotorESt;
	   uint8_t uFMotorO6St;
	   uint8_t uFMotorO7St;
	   uint8_t uFMotorEEnable;
	   uint8_t uFMotorDEnable;
	   uint8_t uFMotorCEnable;
	   uint8_t uFMotorBEnable;
	   uint8_t uFMotorAEnable;
	
	   uint8_t uFMotorERREnable;
	   uint8_t uFMotorDRREnable;
	   uint8_t uFMotorCRREnable;
	   uint8_t uFMotorBRREnable;
	   uint8_t uFMotorARREnable;	
	
	
}LinterMotor;

#define LINTER_DEFAULTS  {0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1};

u8 NRF24L01_RxProcess(u8 *rxbuf);
void uart4_init(u32 bound);

#endif
/*==================================================================================
     the end of file
===================================================================================*/
