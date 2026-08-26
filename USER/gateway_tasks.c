#include "gateway_tasks.h"

#include "modbus_common.h"
#include "modbus_master.h"
#include "motor_config.h"
#include "tick.h"

static uint32_t remote_off_deadline;
static uint8_t remote_off_pending;
static uint8_t remote_off_command_active;

void GatewayTasks_ScheduleRemotePowerOff(uint32_t delay_ms)
{
    remote_off_deadline = GetTick() + delay_ms;
    remote_off_pending = 1U;
}

void GatewayTasks_CancelRemotePowerOff(void)
{
    remote_off_pending = 0U;
    remote_off_command_active = 0U;
}

/* 到期后使用同一组参数重复查询异步主站事务，直至取得最终结果。 */
void GatewayTasks_Process(void)
{
    uint8_t result;

    if (!remote_off_command_active) {
        if (!remote_off_pending ||
            (int32_t)(GetTick() - remote_off_deadline) < 0) {
            return;
        }
        remote_off_command_active = 1U;
    }

    result = ModbusMaster_06_WriteSingleReg(MODBUS_MASTER_CLIENT_GATEWAY_TASK,
                                            MOTOR14_SLAVE_ADDR,
                                            0x0001U, 0x0001U);
    if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
        return;
    }
    remote_off_command_active = 0U;
    remote_off_pending = 0U;
}
