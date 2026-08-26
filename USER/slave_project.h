/*==================================================================================
 * Slave Station Program for Drone Battery Swap System
 * Based on STM32F407VE
 * Communication: Modbus RTU (RS485)
 *==================================================================================*/

#ifndef __SLAVE_PROJECT_H
#define __SLAVE_PROJECT_H

#include <stdio.h>
#include "stm32f4xx.h"
#include "STM103REG.h"
#include "slave_project_cfg.h"
#include "delay.h"
#include "input.h"
#include "adc.h"
#include "usart.h"
#include "tmr.h"
#include "sys.h"
#include "modbus_master.h"

typedef unsigned char INT8U;
typedef unsigned short INT16U;
typedef unsigned int INT32U;

#define MAX(a, b) ((a) < (b) ? (b) : (a))
#define MIN(a, b) ((a) > (b) ? (b) : (a))

// Slave station address (configurable via DIP switch)
extern U8 slave_address;

// Status registers
typedef struct {
    U16 motor_status;      // Motor running status
    U16 input_status;      // Input port status
    U16 fault_code;        // Fault code
    U16 battery_voltage;   // Battery voltage (ADC)
    U16 motor_current;     // Motor current
    U16 position;          // Current position
} SLAVE_STATUS;

extern SLAVE_STATUS slave_status;
extern TMRGEN tmr;
extern ADCGEN adc1;

// Function declarations
void SlaveInit(void);
void SlaveMainLoop(void);
void ProcessModbusCommand(void);
void UpdateSlaveStatus(void);

#endif
