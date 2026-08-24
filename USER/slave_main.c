/*==================================================================================
 * Slave Station Main Program
 *==================================================================================*/

#include "slave_project.h"

U8 slave_address = 1;
SLAVE_STATUS slave_status = {0};
TMRGEN tmr;
ADCGEN adc1;

// Read slave address from DIP switches
void ReadSlaveAddress(void) {
    U8 addr = 0;
    if(GPIO_ReadInputDataBit(CODE1, CODE1_PIN)) addr |= 0x01;
    if(GPIO_ReadInputDataBit(CODE2, CODE2_PIN)) addr |= 0x02;
    if(GPIO_ReadInputDataBit(CODE3, CODE3_PIN)) addr |= 0x04;
    if(GPIO_ReadInputDataBit(CODE4, CODE4_PIN)) addr |= 0x08;
    if(GPIO_ReadInputDataBit(CODE5, CODE5_PIN)) addr |= 0x10;

    slave_address = (addr == 0) ? 1 : addr;
}

// Initialize slave station
void SlaveInit(void) {
    // Read address from DIP switches
    ReadSlaveAddress();

    // Initialize Modbus as slave
    ModbusInit(slave_address, MODBUS_BAUDRATE);

    // Initialize GPIO
    GPIO_SetBits(LED_PORT, LED_PIN);

    // Clear status
    slave_status.motor_status = 0;
    slave_status.fault_code = 0;
}

// Process Modbus commands from master
void ProcessModbusCommand(void) {
    U16 cmd;

    if(ModbusReadHoldingReg(REG_MOTOR_CMD, &cmd)) {
        switch(cmd) {
            case 0: MOTOR_STOP(); break;
            case 1: MOTOR_FORWARD(); break;
            case 2: MOTOR_REVERSE(); break;
        }
        slave_status.motor_status = cmd;
    }
}

// Update slave status
void UpdateSlaveStatus(void) {
    // Read inputs
    slave_status.input_status = 0;
    if(GPIO_ReadInputDataBit(LIMIT_OPEN_PORT, LIMIT_OPEN_PIN))
        slave_status.input_status |= 0x01;
    if(GPIO_ReadInputDataBit(LIMIT_CLOSE_PORT, LIMIT_CLOSE_PIN))
        slave_status.input_status |= 0x02;

    // Read ADC
    slave_status.battery_voltage = adc1.adc_value;

    // Write to Modbus registers
    ModbusWriteHoldingReg(REG_MOTOR_STATUS, slave_status.motor_status);
    ModbusWriteHoldingReg(REG_INPUT_STATUS, slave_status.input_status);
    ModbusWriteHoldingReg(REG_FAULT_CODE, slave_status.fault_code);
    ModbusWriteHoldingReg(REG_VOLTAGE, slave_status.battery_voltage);
}

// Main loop
void SlaveMainLoop(void) {
    while(1) {
        ProcessModbusCommand();
        UpdateSlaveStatus();

        // LED blink
        if(tmr.t100ms_flag) {
            GPIO_ToggleBits(LED_PORT, LED_PIN);
        }
    }
}

int main(void) {
    SlaveInit();
    SlaveMainLoop();
}
