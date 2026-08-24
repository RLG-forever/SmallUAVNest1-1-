/*==================================================================================
This document is the property of Nanjing Kangni Mechanical & Electrical Co., Ltd. it 
may not be reproduced or disclosed to third party without prior to authorisation.
- 文件名称: io.h
- 文件描述: 定义输入输出口模块变量及函数
- 使用情况: 主要io控制模块使用
- 版本信息: V0100-0000, Li.Yuanyuan, 20210819
===================================================================================*/

/*==================================================================================
  文件定义
===================================================================================*/
#ifndef __IO_H
#define	__IO_H

/*===================================================================================
  引用头文件列表
===================================================================================*/
#include "sys.h"	 	 
//#include "timer.h"
//#include "door.h"
//#include "diagnose.h"
//#include "can.h"

/*==================================================================================
  常量定义
===================================================================================*/
//
#define POSITIVE_II01_FLTR    (io->uFInputPort01Fltr)        //input port 01 positive logic
#define NEGATIVE_II01_FLTR    (!(io->uFInputPort01Fltr))     //input port 01 negative logic
#define POSITIVE_II02_FLTR    (io->uFInputPort02Fltr)        //input port 00 positive logic
#define NEGATIVE_II02_FLTR    (!(io->uFInputPort02Fltr))     //input port 02 negative logic
#define POSITIVE_II03_FLTR    (io->uFInputPort03Fltr)        //input port 02 positive logic
#define NEGATIVE_II03_FLTR    (!(io->uFInputPort03Fltr))     //input port 02 negative logic
#define POSITIVE_II04_FLTR    (io->uFInputPort04Fltr)        //input port 03 positive logic
#define NEGATIVE_II04_FLTR    (!(io->uFInputPort04Fltr))     //input port 03 negative logic
#define POSITIVE_II05_FLTR    (io->uFInputPort05Fltr)        //input port 04 positive logic
#define NEGATIVE_II05_FLTR    (!(io->uFInputPort05Fltr))     //input port 04 negative logic
#define POSITIVE_II06_FLTR    (io->uFInputPort06Fltr)        //input port 06 positive logic
#define NEGATIVE_II06_FLTR    (!(io->uFInputPort06Fltr))     //input port 06 negative logic



/*==================================================================================
  变量定义
===================================================================================*/
typedef struct
{
	u8 uIInputPort01;
	u8 uIInputPort02;
	u8 uIInputPort03;
	u8 uIInputPort04;
	u8 uIInputPort05;
	u8 uIInputPort06;
	
	u8 uFOutputFdbck01;
	u8 uFOutputFdbck02;

	u8 uFInputPort01Fltr;
	u8 uFInputPort02Fltr;
	u8 uFInputPort03Fltr;
	u8 uFInputPort04Fltr;
	u8 uFInputPort05Fltr;		
	u8 uFInputPort06Fltr;
	
	u8 uMIPreInputPort01;
	u8 uMIPreInputPort02;
	u8 uMIPreInputPort03;
	u8 uMIPreInputPort04;
	u8 uMIPreInputPort05;
	u8 uMIPreInputPort06;
	
	u8 uFOpnLineFltr;     // 岗亭硬线开门信号
	u8 uFClsLineFltr;     // 岗亭硬线关门信号
}IOGEN;

/*==================================================================================
   变量初始化值
===================================================================================*/
#define IO_DEFAULTS  {0,0,0,0,0, 0,\
                      0,0,\
                      0,0,0,0,0, 0,\
                      0,0,0,0,0, 0,\
                      0,0}
/*==================================================================================
   函数定义
===================================================================================*/
void ReadInputSignal(IOGEN *io);
void InputPortInit(void);
void OutputPortInit(void);
void RawSgnlFilter(u8 *uMSgnlPre, u8 uMSgnlNow, u8 *uMSgnlFilter, s16 *iMTimer,\
                   u8 uMClockSetUp, u8 uMClockSetDn);
void InputPortFilter(IOGEN *io, TMRGEN *tmr);
void UpdateInputSignal(IOGEN *io);
void CtrlOutputPort(CANGEN *can, DGNSGEN *dgns);
void Output_ON(uint16_t GPIO_Pin);
void Output_OFF(uint16_t GPIO_Pin);
#endif
/*==================================================================================
     the end of file
===================================================================================*/

