#include "motor_service.h"
#include "motor_config.h"
#include "modbus_common.h"
#include "modbus_master.h"

#include <stddef.h>
#include <string.h>

#define MODBUS_BATCH_MAX_MOTORS 8U

/* 物理总线为串行总线，因此多电机命令按事务依次执行，不能同时发送。 */
typedef struct {
    uint8_t active;
    uint8_t count;
    uint8_t index;
    uint8_t failures;
    uint8_t first_failure;
    MotorControlParams motors[MODBUS_BATCH_MAX_MOTORS];
} MotorBatchContext;

static MotorBatchContext motor_batch;

/* 堵转状态由电机服务模块统一持有，避免头文件静态变量产生多个副本。 */
static volatile uint8_t stall_triggered = 0U;

uint8_t MotorMonitor_StallTriggered(void)
{
    return stall_triggered;
}

void MotorMonitor_ClearStallTrigger(void)
{
    stall_triggered = 0U;
}

uint8_t Motor_Single_Control(uint8_t slave_addr, uint8_t motor_num, uint16_t motor_cmd)
{
    if (motor_num != 1U) {
        return MODBUS_RESULT_PARAM;
    }
    return ModbusMaster_06_WriteSingleReg(MODBUS_MASTER_CLIENT_MOTOR_SERVICE,
                                          slave_addr, MOTOR1_CTRL_REG1,
                                          motor_cmd);
}

uint8_t Motor_Control(uint8_t motor_id, uint8_t reg_num, uint16_t motor_cmd)
{
    uint8_t slave_addr;
    uint16_t reg_addr;

    switch (motor_id) {
        case MOTOR_ID_1:
            if (reg_num != 1U) return MODBUS_RESULT_PARAM;
            slave_addr = MOTOR1_SLAVE_ADDR;
            reg_addr = MOTOR1_CTRL_REG1;
            break;
        case MOTOR_ID_2:
            if (reg_num != 1U) return MODBUS_RESULT_PARAM;
            slave_addr = MOTOR2_SLAVE_ADDR;
            reg_addr = MOTOR2_CTRL_REG1;
            break;
        case MOTOR_ID_3:
            slave_addr = MOTOR3_SLAVE_ADDR;
            if (reg_num == 1U) reg_addr = MOTOR3_CTRL_REG1;
            else if (reg_num == 2U) reg_addr = MOTOR3_CTRL_REG2;
            else if (reg_num == 3U) reg_addr = MOTOR3_CTRL_REG3;
            else return MODBUS_RESULT_PARAM;
            break;
        case MOTOR_ID_4:
            if (reg_num != 1U) return MODBUS_RESULT_PARAM;
            slave_addr = MOTOR4_SLAVE_ADDR;
            reg_addr = MOTOR4_CTRL_REG1;
            break;
        default:
            return MODBUS_RESULT_PARAM;
    }
    return ModbusMaster_06_WriteSingleReg(MODBUS_MASTER_CLIENT_MOTOR_SERVICE,
                                          slave_addr, reg_addr, motor_cmd);
}

uint8_t Motor_Batch_Control(uint8_t slave_addr, uint16_t start_reg,
                            uint16_t reg_num, const uint16_t *motor_cmds)
{
    return ModbusMaster_10_WriteMultiReg(MODBUS_MASTER_CLIENT_MOTOR_SERVICE,
                                         slave_addr, start_reg, reg_num,
                                         motor_cmds);
}

uint8_t Motor_Read_Status(uint8_t motor_num, uint16_t *motor_status)
{
    (void)motor_num;
    (void)motor_status;
    return MODBUS_RESULT_PARAM;
}

uint8_t Motor_Batch_Read_Status(uint16_t start_reg, uint16_t motor_num,
                                uint16_t *status_buff)
{
    (void)start_reg;
    (void)motor_num;
    (void)status_buff;
    return MODBUS_RESULT_PARAM;
}

uint8_t MotorService_IsBusy(void)
{
    return (uint8_t)(ModbusMaster_IsBusy() || motor_batch.active);
}

void MotorService_Cancel(void)
{
    ModbusBatch_Cancel();
    ModbusMaster_Cancel();
}

uint8_t MotorService_WriteTarget(MotorServiceTarget target, uint16_t value)
{
    switch (target) {
        case MOTOR_SERVICE_TARGET_LIFT_UP:
            return ModbusMaster_06_WriteSingleReg(
                MODBUS_MASTER_CLIENT_GATEWAY, 0x12U, 0x0005U, value);
        case MOTOR_SERVICE_TARGET_LIFT_DOWN:
            return ModbusMaster_06_WriteSingleReg(
                MODBUS_MASTER_CLIENT_GATEWAY, 0x12U, 0x0006U, value);
        default:
            return MODBUS_RESULT_PARAM;
    }
}

uint8_t MotorService_BatchMove(const MotorControlParams *motors, uint8_t count,
                               uint8_t *results)
{
    uint16_t commands[2];
    uint8_t result;

    if (motors == NULL || count == 0U || count > MODBUS_BATCH_MAX_MOTORS) {
        return MODBUS_RESULT_PARAM;
    }
    if (!motor_batch.active) {
        memcpy(motor_batch.motors, motors, count * sizeof(MotorControlParams));
        motor_batch.active = 1U;
        motor_batch.count = count;
        motor_batch.index = 0U;
        motor_batch.failures = 0U;
        motor_batch.first_failure = MODBUS_RESULT_OK;
    }

    commands[0] = motor_batch.motors[motor_batch.index].value_low_word;
    commands[1] = motor_batch.motors[motor_batch.index].value_high_word;
    result = ModbusMaster_10_WriteMultiReg(
        MODBUS_MASTER_CLIENT_MOTOR_SERVICE,
        motor_batch.motors[motor_batch.index].slave_addr,
        motor_batch.motors[motor_batch.index].register_address,
        2U, commands);

    if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
        return result;
    }
    if (results != NULL) {
        results[motor_batch.index] = result;
    }
    if (result != MODBUS_RESULT_OK) {
        motor_batch.failures++;
        if (motor_batch.first_failure == MODBUS_RESULT_OK) {
            motor_batch.first_failure = result;
        }
    }
    motor_batch.index++;
    if (motor_batch.index < motor_batch.count) {
        return MODBUS_RESULT_PENDING;
    }

    result = motor_batch.failures == 0U ? MODBUS_RESULT_OK : motor_batch.first_failure;
    memset(&motor_batch, 0, sizeof(motor_batch));
    return result;
}

uint8_t ModbusBatch_IsBusy(void)
{
    return motor_batch.active;
}

void ModbusBatch_Cancel(void)
{
    memset(&motor_batch, 0, sizeof(motor_batch));
}
