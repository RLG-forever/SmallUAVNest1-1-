/*==================================================================================
- 模块名称: [scom-] CAN通讯模块
- 模块描述: 1. CAN通讯接口接收数据处理、发送数据处理; 
- 版本信息: V0100-0000,Li.Yuanyuan,20210705
- 函数列表: 
		(Procedure name,    Version)
		ReadInputSignal     V0100
		RawSgnlFilter       V0100
		InputPortFilter     V0100
		UpdateInputSignal   V0100
		CnfgrInputPort      V0100
===================================================================================*/

/*==================================================================================
   list of header files
===================================================================================*/
#include "io.h" 
#include "project_cfg.h" 
/*==================================================================================
- 函数名称:ReadInputSignal [sunit-]
- 功能描述:输入口读取函数
- 运行位置:
- 调用函数:
- 返回值:
- 版本信息:V0100-0000,Li.Yuanyuan,20210819
===================================================================================*/
void ReadInputSignal(IOGEN *io)
{   
    io->uIInputPort01 = GPIO_ReadInputDataBit(IPORT01, IPORT01_PIN);
    io->uIInputPort02 = GPIO_ReadInputDataBit(IPORT02, IPORT02_PIN);
    io->uIInputPort03 = GPIO_ReadInputDataBit(IPORT03, IPORT03_PIN);
    io->uIInputPort04 = GPIO_ReadInputDataBit(IPORT04, IPORT04_PIN);
    io->uIInputPort05 = GPIO_ReadInputDataBit(IPORT05, IPORT05_PIN);
    io->uIInputPort06 = GPIO_ReadInputDataBit(IPORT06, IPORT06_PIN);	
}

/*==================================================================================
- 函数名称:RawSgnlFilter [sunit-]
- 功能描述:滤波子函数
- 运行位置:
- 调用函数:
- 返回值:
- 版本信息:V0100-0000,Li.Yuanyuan,20210819
===================================================================================*/
void RawSgnlFilter(u8 *uMSgnlPre, u8 uMSgnlNow, u8 *uMSgnlFilter, s16 *iMTimer,\
                   u8 uMClockSetUp, u8 uMClockSetDn)
{
    if((*uMSgnlPre) != uMSgnlNow)
    {
        if(uMSgnlNow == 1)
        {
            *iMTimer = uMClockSetUp;
        }	
        else
        {
			      *iMTimer = uMClockSetDn;
        }		
            *uMSgnlPre = uMSgnlNow;
    }
    else
    {
        if(*iMTimer == 0)
        {
		      	*uMSgnlFilter = uMSgnlNow & 0x0001;  //only 1 bit, error control
			      *iMTimer = -1;
        }
    }
}

/*==================================================================================
- 函数名称:InputPortFilter [sunit-]
- 功能描述:输入口滤波函数
- 运行位置:
- 调用函数:
- 返回值:
- 版本信息:V0100-0000,Li.Yuanyuan,20210819
===================================================================================*/
void InputPortFilter(IOGEN *io, TMRGEN *tmr)
{	
    RawSgnlFilter(&io->uMIPreInputPort01, io->uIInputPort01, &io->uFInputPort01Fltr,\
        &tmr->TIMER_INPUT01_FLTR_T10MS, DFLT_II01_FLT_TIME_UP, DFLT_II01_FLT_TIME_DN);

    RawSgnlFilter(&io->uMIPreInputPort02, io->uIInputPort02, &io->uFInputPort02Fltr,\
        &tmr->TIMER_INPUT02_FLTR_T10MS, DFLT_II02_FLT_TIME_UP, DFLT_II02_FLT_TIME_DN);

    RawSgnlFilter(&io->uMIPreInputPort03, io->uIInputPort03, &io->uFInputPort03Fltr,\
        &tmr->TIMER_INPUT03_FLTR_T10MS, DFLT_II03_FLT_TIME_UP, DFLT_II03_FLT_TIME_DN);

    RawSgnlFilter(&io->uMIPreInputPort04, io->uIInputPort04, &io->uFInputPort04Fltr,\
        &tmr->TIMER_INPUT04_FLTR_T10MS, DFLT_II04_FLT_TIME_UP, DFLT_II04_FLT_TIME_DN);

    RawSgnlFilter(&io->uMIPreInputPort05, io->uIInputPort05, &io->uFInputPort05Fltr,\
        &tmr->TIMER_INPUT05_FLTR_T10MS, DFLT_II05_FLT_TIME_UP, DFLT_II05_FLT_TIME_DN);
	
		RawSgnlFilter(&io->uMIPreInputPort06, io->uIInputPort06, &io->uFInputPort06Fltr,\
			  &tmr->TIMER_INPUT06_FLTR_T10MS, DFLT_II06_FLT_TIME_UP, DFLT_II06_FLT_TIME_DN);
}

/*==================================================================================
- 函数名称:UpdateInputSignal [sunit-]
- 功能描述:信号更新函数
- 运行位置:
- 调用函数:
- 返回值:
- 版本信息:V0100-0000,Li.Yuanyuan,20210819
===================================================================================*/
void UpdateInputSignal(IOGEN *io)
{
    io->uFOpnLineFltr = OPN_LINE_LOGIC_FLAG;       // 岗亭开门硬线信号
    io->uFClsLineFltr = CLS_LINE_LOGIC_FLAG;				// 岗亭关门硬线信号
}

/*==================================================================================
- 函数名称:CnfgrInputPort [sunit-]
- 功能描述:输入口配置函数
- 运行位置:
- 调用函数:
- 返回值:
- 版本信息:V0100-0000,Li.Yuanyuan,20210819
===================================================================================*/
void InputPortInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = IPORT01_PIN;	 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 
    GPIO_Init(IPORT01, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = IPORT02_PIN;	 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	  
    GPIO_Init(IPORT02, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = IPORT03_PIN;	 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	  
    GPIO_Init(IPORT03, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = IPORT04_PIN;	 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 
    GPIO_Init(IPORT04, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = IPORT05_PIN;	 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	    
    GPIO_Init(IPORT05, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = IPORT06_PIN;	 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
    GPIO_Init(IPORT06, &GPIO_InitStructure);
}

/*==================================================================================
- 函数名称:CnfgrOutputPort [sunit-]
- 功能描述:输出口配置函数
- 运行位置:
- 调用函数:
- 返回值:
- 版本信息: V0100-0000,Li.Yuanyuan,20210819
           V0200-0000,Liu Xiao,20211015
===================================================================================*/
void OutputPortInit(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOE, ENABLE);

		GPIO_InitStructure.GPIO_Pin = OPORT01_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                           
		GPIO_Init(OPORT01, &GPIO_InitStructure);
		GPIO_ResetBits(OPORT01, OPORT01_PIN);
	
		GPIO_InitStructure.GPIO_Pin = OPORT02_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                           
		GPIO_Init(OPORT02, &GPIO_InitStructure);
		GPIO_ResetBits(OPORT02, OPORT02_PIN);
	
		//GPIOE14初始化设置
		GPIO_InitStructure.GPIO_Pin = OPORT03_PIN ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//??????
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//????
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//??
		GPIO_Init(OPORT03, &GPIO_InitStructure);//???
		GPIO_SetBits(OPORT03, OPORT03_PIN);//???GPIO?????

		//GPIOE15初始化设置
		GPIO_InitStructure.GPIO_Pin = OPORT04_PIN ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//??????
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//????
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//??
		GPIO_Init(OPORT04, &GPIO_InitStructure);//???	
		GPIO_SetBits(OPORT04, OPORT04_PIN);//???GPIO?????
}

/*=================================================================================
- 函数名称:Output_ON [sunit-]
- 功能描述:输出口点亮函数
- 运行位置:
- 调用函数:
- 返回值:
- 版本信息:V0100-0000,Li.Yuanyuan,20210819
===================================================================================*/
void Output_ON(uint16_t GPIO_Pin)
{
    GPIO_SetBits(GPIOA,GPIO_Pin);
}

/*=================================================================================
- 函数名称:Output_OFF [sunit-]
- 功能描述:输出口点灭函数
- 运行位置:
- 调用函数:
- 返回值:
- 版本信息:V0100-0000,Li.Yuanyuan,20210819
===================================================================================*/
void Output_OFF(uint16_t GPIO_Pin)
{
    GPIO_ResetBits(GPIOA,GPIO_Pin);
}

/*==================================================================================
- 函数名称:CtrlOutputPort [sunit-]
- 功能描述:输出口控制函数
- 运行位置:
- 调用函数:
- 返回值:
- 版本信息:V0100-0000,Li.Yuanyuan,20210819
===================================================================================*/
void CtrlOutputPort(CANGEN *can, DGNSGEN *dgns)
{
	u8  i=0;
		
	//输入口信号驱动输出口
//	if(1U == io->uFOpnLineFltr)
//	{
//		Output_ON(OPORT01_PIN);
//	}
//	else
//	{
//		Output_OFF(OPORT01_PIN);
//	}
//	
//	if(1U == io->uFClsLineFltr)
//	{
//		Output_ON(OPORT02_PIN);
//	}
//	else
//	{
//		Output_OFF(OPORT02_PIN);
//	}
	
	//有一个电机CAN通讯掉线，就驱动输出口输出
	for(i=0;i<12;i++)
	{
		if(1 == dgns->uFCanCommFail[i])
		{
			if(1 == can->ucSimuOpnLin)
			{
				Output_ON(OPORT01_PIN);
			}
			else
			{
				Output_OFF(OPORT01_PIN);
			}
			if(1U == can->ucSimuClsLin)
			{
				Output_ON(OPORT02_PIN);
			}
			else
			{
				Output_OFF(OPORT02_PIN);
			}
			
			break;
		}
	}
}
/*==================================================================================
     the end of file
===================================================================================*/

