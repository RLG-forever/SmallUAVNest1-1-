/*==================================================================================
This document is the property of Nanjing Kangni Mechanical & Electrical Co., Ltd. It may
not be reproduced or disclosed to third party without prior to authorisation
 * header file name: project.h      
 * heafer file description: define the date type and initalization used by the timer module
 * service condition: main.c
 * version information: Wan Lei, V0100-0000, 20180104
===================================================================================*/

/*==================================================================================
     avoid redefinition
===================================================================================*/
#ifndef __PROJECT_H
#define	__PROJECT_H
#include <stdio.h>	
#include "debug_log.h"

#include "stm32f4xx.h" 
#include "STM103REG.h"
#include "project_cfg.h"
#include "delay.h"      //No.0
#include "input.h"      //No.1
#include "adc.h"        //No.2
#include "usart.h"      //No.3
#include "tmr.h"        //No.4
#include "sys.h"
#include "string.h"
#include "modbus_master.h"
#include "modbus_common.h"
#include "motor_control.h"
#include "modbus_port.h"
#include "relay.h"
#include "status_regs.h"
#include "master_polling.h"
#include "modbus_slave.h"
#include "gateway_service.h"
#include "tick.h"
#include "sequence.h"
#include "bsp_timer.h"
#include "battery_swap.h"
#include "w25qxx.h" 
#include "spi.h"


typedef unsigned char INT8U;
typedef unsigned short INT16U;
typedef unsigned int INT32U;

/*Macro function*/
#define MAX(a, b)                ((a) < (b) ? (b) : (a))
#define MIN(a, b)                ((a) > (b) ? (b) : (a))

extern TMRGEN   tmr ;
//extern  MCGEN        lmc;
//extern  MCGEN        rmc;
//extern  HALLGEN      lmh;
extern  ADCGEN       adc1;
//extern  DOORLGEN     drl; 
//extern  ERRORGEN     err;
extern  LinterMotor         lim;
extern int testI,testJ;
/*==================================================================================
     declare definition
===================================================================================*/
void InputPortFilter(IPCGEN *io, TMRGEN *tmr);
//void UpdateInputSignal(DOORLGEN *drl, IPCGEN *io);
//void DoorLogicCntrlUnit(DOORLGEN *drl, TMRGEN *tmr, MCGEN *mc1, MCGEN *mc2);
//void ShowLed(DOORLGEN *drl, TMRGEN *tmr, ERRORGEN *err);
//void CalcPWM(MCGEN *mc1, MCGEN *mc2, TMRGEN *tmr);
//void DiagnoseCntrlUnit(DGNSGEN *dgns, MCGEN *mc1, MCGEN *mc2, TMRGEN *tmr, DOORLGEN *drl);
//void OutputPortCntrlUnit(DOORLGEN *drl, TMRGEN *tmr, ERRORGEN *err);


#endif
/*==================================================================================
     the end of file
===================================================================================*/
