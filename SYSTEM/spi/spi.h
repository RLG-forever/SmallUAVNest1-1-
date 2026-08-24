#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//SPI 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
 	    													  
void SPI1_Init(void);			 //初始化SPI1口
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI1速度   
//u8 SPI1_ReadWriteByte(u8 TxData);//SPI1总线读写一个字节
uint8_t drv_spi_read_write_byte( uint8_t TxByte );

//#define SPI_NSS_GPIO_PORT			GPIOA
//#define SPI_NSS_GPIO_CLK			RCC_AHB1Periph_GPIOA
//#define SPI_NSS_GPIO_PIN			GPIO_Pin_4		 


//#define spi_set_nss_high( )			SPI_NSS_GPIO_PORT->ODR |= SPI_NSS_GPIO_PIN								//????
//#define spi_set_nss_low( )			SPI_NSS_GPIO_PORT->ODR &= (uint32_t)( ~((uint32_t)SPI_NSS_GPIO_PIN ))	//????
#endif

