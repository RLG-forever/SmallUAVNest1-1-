/*==================================================================================
- 模块名称: [scom-] LED控制模块
- 模块描述: 1. led点灯处理模块
- 版本信息: V0100-0000,Li.Yuanyuan,20210819
- 函数列表: 
		(Procedure name,    Version)

===================================================================================*/

/*==================================================================================
   list of header files
===================================================================================*/
#include "led.h" 

/*=================================================================================
- 函数名称:LEDInit [sunit-]
- 功能描述:LED IO初始化函数
- 运行位置:
- 调用函数:
- 返回值:
- 版本信息:V0100-0000,Li.Yuanyuan,20210819
===================================================================================*/
void LEDInit(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
  /* GPIOA9初始化设置*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA,&GPIO_InitStructure);//初始化
  GPIO_ResetBits(GPIOA,GPIO_Pin_8);//GPIOA8设置高，灯亮
}   

/*=================================================================================
- 函数名称:LED_ON [sunit-]
- 功能描述:LED点灯函数
- 运行位置:
- 调用函数:
- 返回值:
- 版本信息:V0100-0000,Li.Yuanyuan,20210819
===================================================================================*/
void LED_ON(uint16_t GPIO_Pin)
{
    GPIO_SetBits(GPIOA,GPIO_Pin_8);
}

/*=================================================================================
- 函数名称:LED_OFF [sunit-]
- 功能描述:LED灭灯函数
- 运行位置:
- 调用函数:
- 返回值:
- 版本信息:V0100-0000,Li.Yuanyuan,20210819
===================================================================================*/
void LED_OFF(uint16_t GPIO_Pin)
{
    GPIO_ResetBits(GPIOA,GPIO_Pin_8);
}




