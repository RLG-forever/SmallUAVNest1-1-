#include "gateway_modbus.h"

#include "gateway_commands.h"
#include "gateway_tasks.h"
#include "modbus_common.h"
#include "modbus_master.h"
#include "modbus_slave.h"
#include "motor_service.h"
#include "sequence.h"
#include "status_service.h"
#include "tick.h"

#include <stddef.h>

#define GATEWAY_DUPLICATE_WINDOW_MS       5000U
#define GATEWAY_REMOTE_OFF_DELAY_MS       (30UL * 60UL * 1000UL)
static uint8_t wait_open_fly;
static uint8_t takeoff_power_ready;
static uint16_t last_write_register = 0xFFFFU;
static uint16_t last_write_value = 0xFFFFU;
static uint32_t last_write_time;

static uint8_t GatewayModbus_StartSequence(SeqId id)
{
    SequenceStartResult result = Sequence_Start(id);

    if (result == SEQ_START_OK) {
        return MODBUS_RESULT_OK;
    }
    if (result == SEQ_START_MASTER_BUSY) {
        return MODBUS_RESULT_BUSY;
    }
    return MODBUS_RESULT_PARAM;
}

static uint8_t GatewayModbus_IsRecentDuplicate(uint16_t register_address,
                                               uint16_t value)
{
    return register_address == last_write_register &&
           value == last_write_value &&
           (uint32_t)(GetTick() - last_write_time) <
               GATEWAY_DUPLICATE_WINDOW_MS;
}

static void GatewayModbus_RecordCompletedWrite(uint16_t register_address,
                                               uint16_t value)
{
    last_write_register = register_address;
    last_write_value = value;
    last_write_time = GetTick();
}

/* 执行单个网关寄存器命令，不负责编码从站应答。 */
static uint8_t GatewayModbus_ExecuteWrite(uint16_t register_address,
                                         uint16_t value)
{
    GatewayCommandTarget target;
    uint8_t result = MODBUS_RESULT_PARAM;

    if (GatewayModbus_IsRecentDuplicate(register_address, value)) {
        return MODBUS_RESULT_OK;
    }

    switch (register_address) {
        case GATEWAY_CMD_OPEN_DOOR:
            result = GatewayModbus_StartSequence(SEQ_ID_OPENDR);
            break;
        case GATEWAY_CMD_CLOSE_DOOR:
            result = GatewayModbus_StartSequence(SEQ_ID_CLOSEDR);
            break;
        case GATEWAY_CMD_CLOSE_CENTER:
            result = GatewayModbus_StartSequence(SEQ_ID_CLOSECENTER);
            break;
        case GATEWAY_CMD_LEAVE_CENTER:
            result = GatewayModbus_StartSequence(SEQ_ID_LEAVECENTER);
            break;
        case GATEWAY_CMD_LOAD_BATTERY:
            result = GatewayModbus_StartSequence(SEQ_ID_LOADBATTERY);
            break;
        case GATEWAY_CMD_UNLOAD_BATTERY:
            result = GatewayModbus_StartSequence(SEQ_ID_DOWNBATTERY);
            break;
        case GATEWAY_CMD_PAUSE:
            Sequence_Pause();
            result = MODBUS_RESULT_OK;
            break;
        case GATEWAY_CMD_RESUME:
            Sequence_Resume();
            result = MODBUS_RESULT_OK;
            break;
        case GATEWAY_CMD_CANCEL:
            Sequence_Cancel();
            result = MODBUS_RESULT_OK;
            break;
        case GATEWAY_CMD_OPEN_DOOR_ALT:
        case GATEWAY_CMD_OPEN_UP:
            result = GatewayModbus_StartSequence(SEQ_ID_OPENDR1);
            break;
        case GATEWAY_CMD_CLOSE_DOWN:
            result = GatewayModbus_StartSequence(SEQ_ID_CLOSEDOWN);
            break;
        case GATEWAY_CMD_OPEN_FLY:
            result = GatewayModbus_StartSequence(SEQ_ID_OPENFLY);
            break;
        case GATEWAY_CMD_CLOSE_FLY:
            result = GatewayModbus_StartSequence(SEQ_ID_CLOSEFLY);
            break;

        case GATEWAY_CMD_TAKEOFF:
        {
            uint16_t uav_status = StatusService_GetUavStatus();

            if (uav_status == 0U && !takeoff_power_ready) {
                result = ModbusMaster_06_WriteSingleReg(
                    MODBUS_MASTER_CLIENT_GATEWAY,
                    UAV_CONTROLLER_SLAVE,
                    UAV_POWER_CTRL_REG,
                    UAV_POWER_ON_VALUE);
                if (result != MODBUS_RESULT_OK) {
                    break;
                }
                takeoff_power_ready = 1U;
            }
            if (uav_status == 1U) {
                wait_open_fly = 1U;
                result = MODBUS_RESULT_OK;
            } else {
                result = GatewayModbus_StartSequence(SEQ_ID_TAKEOFF);
                if (result == MODBUS_RESULT_OK) {
                    takeoff_power_ready = 0U;
                }
            }
            break;
        }

        case GATEWAY_CMD_LANDING:
            result = GatewayModbus_StartSequence(SEQ_ID_LANDING);
            if (result == MODBUS_RESULT_OK) {
                GatewayTasks_ScheduleRemotePowerOff(
                    GATEWAY_REMOTE_OFF_DELAY_MS);
            }
            break;

        case GATEWAY_CMD_CHARGER_ON:
            result = ModbusMaster_06_WriteSingleReg(
                MODBUS_MASTER_CLIENT_GATEWAY,
                CHARGER_SLAVE,
                CHARGER_POWER_CTRL_REG,
                CHARGER_POWER_ON_VALUE);
            break;
        case GATEWAY_CMD_CHARGER_OFF:
            result = ModbusMaster_06_WriteSingleReg(
                MODBUS_MASTER_CLIENT_GATEWAY,
                CHARGER_SLAVE,
                CHARGER_POWER_CTRL_REG,
                CHARGER_POWER_OFF_VALUE);
            break;
        case GATEWAY_CMD_UAV_POWER_ON:
            result = ModbusMaster_06_WriteSingleReg(
                MODBUS_MASTER_CLIENT_GATEWAY,
                UAV_CONTROLLER_SLAVE,
                UAV_POWER_CTRL_REG,
                UAV_POWER_ON_VALUE);
            break;
        case GATEWAY_CMD_UAV_POWER_MODE2:
            result = ModbusMaster_06_WriteSingleReg(
                MODBUS_MASTER_CLIENT_GATEWAY,
                UAV_CONTROLLER_SLAVE,
                UAV_POWER_CTRL_REG,
                UAV_POWER_MODE2_VALUE);
            break;
        case GATEWAY_CMD_COOLING_STOP_TEMP:
            result = ModbusMaster_06_WriteSingleReg(
                MODBUS_MASTER_CLIENT_GATEWAY,
                AIR_CONDITIONER_SLAVE,
                AIR_COOLING_STOP_TEMP_REG,
                value);
            break;
        case GATEWAY_CMD_HEATING_STOP_TEMP:
            result = ModbusMaster_06_WriteSingleReg(
                MODBUS_MASTER_CLIENT_GATEWAY,
                AIR_CONDITIONER_SLAVE,
                AIR_HEATING_STOP_TEMP_REG,
                value);
            break;

        case GATEWAY_CMD_UAV_STATUS:
            StatusService_SetUavStatus(value);
            if (value == 2U) {
                wait_open_fly = 0U;
                result = MODBUS_RESULT_OK;
            } else if (value == 1U && wait_open_fly && !Sequence_IsBusy()) {
                result = GatewayModbus_StartSequence(SEQ_ID_TAKEOFF);
                if (result == MODBUS_RESULT_OK) {
                    wait_open_fly = 0U;
                }
            } else {
                result = MODBUS_RESULT_OK;
            }
            break;

        default:
            if (GatewayCommands_FindTarget(register_address, &target)) {
                result = MotorService_WriteTarget(target.target, value);
            }
            break;
    }

    if (result == MODBUS_RESULT_OK) {
        GatewayModbus_RecordCompletedWrite(register_address, value);
    }
    return result;
}

static uint8_t GatewayModbus_Handle03(const ModbusSlaveRequest *request)
{
    uint16_t start_register = request->start_register;
    uint16_t register_count = request->register_count;
    uint8_t response[3U + STATUS_SERVICE_EXTERNAL_REGISTER_COUNT * 2U];

    if (register_count == 0U ||
        start_register >= STATUS_SERVICE_EXTERNAL_REGISTER_COUNT ||
        register_count > (uint16_t)(STATUS_SERVICE_EXTERNAL_REGISTER_COUNT -
                                    start_register)) {
        return ModbusSlave_SendException(request, 0x02U) != 0U;
    }

    response[0] = request->address;
    response[1] = 0x03U;
    response[2] = (uint8_t)(register_count * 2U);
    StatusService_GetExternalBatch(start_register, (uint8_t)register_count,
                                   &response[3]);
    return ModbusSlave_SendFrame(response,
                                 3U + register_count * 2U) != 0U;
}

static uint8_t GatewayModbus_Handle06(const ModbusSlaveRequest *request)
{
    uint8_t result = GatewayModbus_ExecuteWrite(request->start_register,
                                                request->value);

    if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
        return 1U;
    }
    if (result == MODBUS_RESULT_OK) {
        return ModbusSlave_SendWriteAck(request) != 0U;
    }
    return ModbusSlave_SendException(request, 0x02U) != 0U;
}

/* 每次主循环只执行 0x10 请求中的一个寄存器，避免阻塞其他任务。 */
static uint8_t GatewayModbus_Handle10(const ModbusSlaveRequest *request)
{
    static uint16_t index;
    static uint8_t active;
    uint16_t register_address;
    uint16_t value;
    uint8_t result;

    if (!active) {
        active = 1U;
        index = 0U;
    }

    register_address = (uint16_t)(request->start_register + index);
    value = Modbus_GetU16BE(&request->write_data[index * 2U]);
    result = GatewayModbus_ExecuteWrite(register_address, value);
    if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
        return 1U;
    }
    if (result != MODBUS_RESULT_OK) {
        active = 0U;
        index = 0U;
        return ModbusSlave_SendException(request, 0x02U) != 0U;
    }

    index++;
    if (index < request->register_count) {
        return 1U;
    }

    active = 0U;
    index = 0U;
    return ModbusSlave_SendWriteAck(request) != 0U;
}

void GatewayModbus_Init(void)
{
    const ModbusSlaveHandlers handlers = {
        GatewayModbus_Handle03,
        GatewayModbus_Handle06,
        GatewayModbus_Handle10
    };

    wait_open_fly = 0U;
    takeoff_power_ready = 0U;
    ModbusSlave_RegisterHandlers(&handlers);
}
