#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

/* 主站对外接口
 * modbus_master.c，硬件访问已迁移到modbus_port.c */
#include <stdint.h>

/* 标识事务所有者，防止不同模块发送相同报文时误取对方的完成结果。 */
typedef enum {
    MODBUS_MASTER_CLIENT_SEQUENCE = 0,
    MODBUS_MASTER_CLIENT_MOTOR_SERVICE,
    MODBUS_MASTER_CLIENT_GATEWAY,
    MODBUS_MASTER_CLIENT_GATEWAY_TASK,
    MODBUS_MASTER_CLIENT_POLLING
} ModbusMasterClient;

uint8_t ModbusMaster_03_ReadHoldReg(ModbusMasterClient client,
                                    uint8_t slave_addr, uint16_t start_reg,
                                    uint16_t reg_num, uint16_t *read_buff);
uint8_t ModbusMaster_06_WriteSingleReg(ModbusMasterClient client,
                                       uint8_t slave_addr, uint16_t reg_addr,
                                       uint16_t reg_data);
uint8_t ModbusMaster_10_WriteMultiReg(ModbusMasterClient client,
                                      uint8_t slave_addr, uint16_t start_reg,
                                      uint16_t reg_num,
                                      const uint16_t *write_buff);
/* 统一主站事务状态机；Task 必须在主循环中周期调用。 */
void ModbusMaster_Init(void);
/* 在主循环中周期调用，推进主站发送、接收、超时和重试状态机。 */
void ModbusMaster_Process(void);
uint8_t ModbusMaster_IsBusy(void);
void ModbusMaster_Cancel(void);

#endif
