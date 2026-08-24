/*==================================================================================
 * Slave Station Configuration
 *==================================================================================*/

#ifndef __SLAVE_PROJECT_CFG_H
#define __SLAVE_PROJECT_CFG_H

// Slave address configuration (read from DIP switch CODE1-CODE5)
#define SLAVE_ADDR_MIN    1
#define SLAVE_ADDR_MAX    31

// Modbus configuration
#define MODBUS_BAUDRATE   9600
#define MODBUS_TIMEOUT    100  // ms

// Motor control ports (simplified - single motor)
#define MOTOR_FWD_PORT    GPIOC
#define MOTOR_FWD_PIN     GPIO_Pin_14
#define MOTOR_REV_PORT    GPIOC
#define MOTOR_REV_PIN     GPIO_Pin_15

// Input ports
#define LIMIT_OPEN_PORT   GPIOC
#define LIMIT_OPEN_PIN    GPIO_Pin_8
#define LIMIT_CLOSE_PORT  GPIOC
#define LIMIT_CLOSE_PIN   GPIO_Pin_7

// Status LED
#define LED_PORT          GPIOC
#define LED_PIN           GPIO_Pin_5

// Motor control macros
#define MOTOR_FORWARD()   GPIO_SetBits(MOTOR_FWD_PORT, MOTOR_FWD_PIN); GPIO_ResetBits(MOTOR_REV_PORT, MOTOR_REV_PIN)
#define MOTOR_REVERSE()   GPIO_ResetBits(MOTOR_FWD_PORT, MOTOR_FWD_PIN); GPIO_SetBits(MOTOR_REV_PORT, MOTOR_REV_PIN)
#define MOTOR_STOP()      GPIO_ResetBits(MOTOR_FWD_PORT, MOTOR_FWD_PIN); GPIO_ResetBits(MOTOR_REV_PORT, MOTOR_REV_PIN)

// Modbus register addresses
#define REG_MOTOR_CMD     0x0000  // Motor command: 0=stop, 1=forward, 2=reverse
#define REG_MOTOR_STATUS  0x0001  // Motor status
#define REG_INPUT_STATUS  0x0002  // Input status
#define REG_FAULT_CODE    0x0003  // Fault code
#define REG_VOLTAGE       0x0004  // Battery voltage
#define REG_CURRENT       0x0005  // Motor current

#endif
