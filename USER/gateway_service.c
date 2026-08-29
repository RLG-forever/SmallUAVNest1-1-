#include "gateway_service.h"

#include "modbus_common.h"
#include "modbus_master.h"
#include "modbus_slave.h"
#include "motor_control.h"
#include "sequence.h"
#include "sequence_steps.h"
#include "status_regs.h"
#include "tick.h"
#include "debug_log.h"

#include <stddef.h>

#define GATEWAY_DUPLICATE_WINDOW_MS       5000U
#define GATEWAY_REMOTE_OFF_DELAY_MS       (30UL * 60UL * 1000UL)

typedef enum {
    GATEWAY_CMD_OPEN_DOOR = 0x0030U,
    GATEWAY_CMD_CLOSE_DOOR = 0x0031U,
    GATEWAY_CMD_CLOSE_CENTER = 0x0032U,
    GATEWAY_CMD_LEAVE_CENTER = 0x0033U,
    GATEWAY_CMD_LIFT_UP = 0x0034U,
    GATEWAY_CMD_LIFT_DOWN = 0x0035U,
    GATEWAY_CMD_LOAD_BATTERY = 0x0036U,
    GATEWAY_CMD_UNLOAD_BATTERY = 0x0037U,
    GATEWAY_CMD_TAKEOFF = 0x0038U,
    GATEWAY_CMD_OPEN_DOOR_ALT = 0x0039U, //与GATEWAY_CMD_OPEN_DOOR业务逻辑重复
    GATEWAY_CMD_LANDING = 0x0040U, 
    GATEWAY_CMD_PAUSE = 0x0041U,
    GATEWAY_CMD_RESUME = 0x0042U,
    GATEWAY_CMD_CANCEL = 0x0043U,
    GATEWAY_CMD_OPEN_UP = 0x0044U, //与GATEWAY_CMD_OPEN_DOOR业务逻辑重复
    GATEWAY_CMD_CLOSE_DOWN = 0x0045U,
    GATEWAY_CMD_CHARGER_ON = 0x0048U,
    GATEWAY_CMD_CHARGER_OFF = 0x0049U,
    GATEWAY_CMD_OPEN_FLY = 0x0050U,
    GATEWAY_CMD_CLOSE_FLY = 0x0051U,
    GATEWAY_CMD_UAV_POWER_ON = 0x0052U,
    GATEWAY_CMD_UAV_POWER_MODE2 = 0x0053U,
    GATEWAY_CMD_COOLING_STOP_TEMP = 0x0054U,
    GATEWAY_CMD_HEATING_STOP_TEMP = 0x0055U,
    GATEWAY_CMD_UAV_STATUS = 0x0060U
} GatewayCommandRegister;

typedef struct {
    MotorControlTarget target;
} GatewayCommandTarget;

typedef struct {
    uint16_t gateway_register;
    GatewayCommandTarget target;
} GatewayCommandMapEntry;

static const GatewayCommandMapEntry command_map[] = {
    {GATEWAY_CMD_LIFT_UP,   {MOTOR_CONTROL_TARGET_LIFT_UP}},
    {GATEWAY_CMD_LIFT_DOWN, {MOTOR_CONTROL_TARGET_LIFT_DOWN}}
};

static uint8_t wait_open_fly;
static uint8_t takeoff_power_ready;
static uint16_t last_write_register = 0xFFFFU;
static uint16_t last_write_value = 0xFFFFU;
static uint32_t last_write_time;
static uint32_t remote_off_deadline;
static uint8_t remote_off_pending;
static uint8_t remote_off_command_active;

static uint8_t GatewayService_FindTarget(uint16_t gateway_register,
                                         GatewayCommandTarget *target)
{
    uint16_t index;

    if (target == NULL) {
        return 0U;
    }
    for (index = 0U;
         index < (uint16_t)(sizeof(command_map) / sizeof(command_map[0]));
         index++) {
        if (command_map[index].gateway_register == gateway_register) {
            *target = command_map[index].target;
            return 1U;
        }
    }
    return 0U;
}

void GatewayService_ScheduleRemotePowerOff(uint32_t delay_ms)
{
    remote_off_deadline = GetTick() + delay_ms;
    remote_off_pending = 1U;
    LOG_INFO("GATEWAY", "remote power-off scheduled: delay=%lu ms\r\n",
             (unsigned long)delay_ms);
}

void GatewayService_CancelRemotePowerOff(void)
{
    if (remote_off_pending || remote_off_command_active) {
        LOG_INFO("GATEWAY", "remote power-off canceled\r\n");
    }
    remote_off_pending = 0U;
}

void GatewayService_Process(void)
{
    uint8_t result;

    if (remote_off_command_active) {
        result = ModbusMaster_06_WriteSingleReg(
            MODBUS_MASTER_CLIENT_GATEWAY_TASK,
            MOTOR14_SLAVE_ADDR, 0x0001U, 0x0001U);
        if (result == MODBUS_RESULT_PENDING) {
            return;
        }
        if (result == MODBUS_RESULT_BUSY) {
            remote_off_command_active = 0U;
            LOG_WARN("GATEWAY", "remote power-off lost bus ownership\r\n");
            return;
        }
        remote_off_command_active = 0U;
        remote_off_pending = 0U;
        if (result == MODBUS_RESULT_OK) {
            LOG_INFO("GATEWAY", "remote power-off command completed\r\n");
        } else {
            LOG_ERROR("GATEWAY", "remote power-off failed: result=%u\r\n",
                      (unsigned int)result);
        }
        return;
    }

    if (!remote_off_pending || Sequence_IsBusy() ||
        (int32_t)(GetTick() - remote_off_deadline) < 0) {
        return;
    }

    result = ModbusMaster_06_WriteSingleReg(MODBUS_MASTER_CLIENT_GATEWAY_TASK,
                                            MOTOR14_SLAVE_ADDR,
                                            0x0001U, 0x0001U);
    if (result == MODBUS_RESULT_PENDING) {
        remote_off_command_active = 1U;
        LOG_INFO("GATEWAY", "remote power-off command started\r\n");
        return;
    }
    if (result == MODBUS_RESULT_BUSY) {
        return;
    }
    remote_off_pending = 0U;
}

static uint8_t GatewayService_StartSequence(SeqId id)
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

/* 检测是否是在时间窗口内重复的指令 */
static uint8_t GatewayService_IsRecentDuplicate(uint16_t register_address,
                                                uint16_t value)
{
    return register_address == last_write_register &&
           value == last_write_value &&
           (uint32_t)(GetTick() - last_write_time) <
               GATEWAY_DUPLICATE_WINDOW_MS;
}

static void GatewayService_RecordCompletedWrite(uint16_t register_address,
                                                uint16_t value)
{
    last_write_register = register_address;
    last_write_value = value;
    last_write_time = GetTick();
}

/* 执行单个网关寄存器命令，不负责编码从站应答。 */
static uint8_t GatewayService_ExecuteWrite(uint16_t register_address,
                                          uint16_t value)
{
    GatewayCommandTarget target;
    uint8_t result = MODBUS_RESULT_PARAM;

    if (GatewayService_IsRecentDuplicate(register_address, value)) {
        LOG_DEBUG("GATEWAY", "duplicate command accepted: reg=0x%04X, value=0x%04X\r\n",
                  (unsigned int)register_address, (unsigned int)value);
        return MODBUS_RESULT_OK;
    }

    switch (register_address) {
        case GATEWAY_CMD_OPEN_DOOR:
            result = GatewayService_StartSequence(SEQ_ID_OPENDR);
            break;
        case GATEWAY_CMD_CLOSE_DOOR:
            result = GatewayService_StartSequence(SEQ_ID_CLOSEDR);
            break;
        case GATEWAY_CMD_CLOSE_CENTER:
            result = GatewayService_StartSequence(SEQ_ID_CLOSECENTER);
            break;
        case GATEWAY_CMD_LEAVE_CENTER:
            result = GatewayService_StartSequence(SEQ_ID_LEAVECENTER);
            break;
        case GATEWAY_CMD_LOAD_BATTERY:
            result = GatewayService_StartSequence(SEQ_ID_LOADBATTERY);
            break;
        case GATEWAY_CMD_UNLOAD_BATTERY:
            result = GatewayService_StartSequence(SEQ_ID_DOWNBATTERY);
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
            Sequence_Stop();
            result = MODBUS_RESULT_OK;
            break;
        case GATEWAY_CMD_OPEN_DOOR_ALT:
        case GATEWAY_CMD_OPEN_UP:
            result = GatewayService_StartSequence(SEQ_ID_OPENDR1);
            break;
        case GATEWAY_CMD_CLOSE_DOWN:
            result = GatewayService_StartSequence(SEQ_ID_CLOSEDOWN);
            break;
        case GATEWAY_CMD_OPEN_FLY:
            result = GatewayService_StartSequence(SEQ_ID_OPENFLY);
            break;
        case GATEWAY_CMD_CLOSE_FLY:
            result = GatewayService_StartSequence(SEQ_ID_CLOSEFLY);
            break;

        case GATEWAY_CMD_TAKEOFF:
        {
            uint16_t uav_status = StatusRegs_Get(REG_RESERVED4);

            GatewayService_CancelRemotePowerOff();
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
                result = GatewayService_StartSequence(SEQ_ID_TAKEOFF);
                if (result == MODBUS_RESULT_OK) {
                    takeoff_power_ready = 0U;
                }
            }
            break;
        }

        case GATEWAY_CMD_LANDING:
            result = GatewayService_StartSequence(SEQ_ID_LANDING);
            if (result == MODBUS_RESULT_OK) {
                GatewayService_ScheduleRemotePowerOff(
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
            StatusRegs_Update(REG_RESERVED4, value);
            if (value == 2U) {
                wait_open_fly = 0U;
                result = MODBUS_RESULT_OK;
            } else if (value == 1U && wait_open_fly && !Sequence_IsBusy()) {
                result = GatewayService_StartSequence(SEQ_ID_TAKEOFF);
                if (result == MODBUS_RESULT_OK) {
                    wait_open_fly = 0U;
                }
            } else {
                result = MODBUS_RESULT_OK;
            }
            break;

        default:
            if (GatewayService_FindTarget(register_address, &target)) {
                result = MotorControl_WriteTarget(target.target, value);
            }
            break;
    }

    if (result == MODBUS_RESULT_OK) {
        GatewayService_RecordCompletedWrite(register_address, value);
        LOG_INFO("GATEWAY", "command completed: reg=0x%04X, value=0x%04X\r\n",
                 (unsigned int)register_address, (unsigned int)value);
    } else if (result != MODBUS_RESULT_PENDING && result != MODBUS_RESULT_BUSY) {
        LOG_ERROR("GATEWAY", "command failed: reg=0x%04X, value=0x%04X, result=%u\r\n",
                  (unsigned int)register_address, (unsigned int)value,
                  (unsigned int)result);
    }
    return result;
}

static uint8_t GatewayService_Handle03(const ModbusSlaveRequest *request)
{
    uint16_t start_register = request->start_register;
    uint16_t register_count = request->register_count;
    uint8_t response[3U + STATUS_REG_COUNT * 2U];

    if (register_count == 0U ||
        start_register >= STATUS_REG_COUNT ||
        register_count > (uint16_t)(STATUS_REG_COUNT -
                                    start_register)) {
        LOG_ERROR("GATEWAY", "invalid read: start_reg=%u, count=%u\r\n",
                  (unsigned int)start_register,
                  (unsigned int)register_count);
        return ModbusSlave_SendException(request, 0x02U) != 0U;
    }

    response[0] = request->address;
    response[1] = 0x03U;
    response[2] = (uint8_t)(register_count * 2U);
    StatusRegs_GetBatch(start_register, (uint8_t)register_count, &response[3]);
    return ModbusSlave_SendFrame(response,
                                 3U + register_count * 2U) != 0U;
}

static uint8_t GatewayService_Handle06(const ModbusSlaveRequest *request)
{
    uint8_t result = GatewayService_ExecuteWrite(request->start_register,
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
static uint8_t GatewayService_Handle10(const ModbusSlaveRequest *request)
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
    result = GatewayService_ExecuteWrite(register_address, value);
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

void GatewayService_Init(void)
{
    const ModbusSlaveHandlers handlers = {
        GatewayService_Handle03,
        GatewayService_Handle06,
        GatewayService_Handle10
    };

    wait_open_fly = 0U;
    takeoff_power_ready = 0U;
    ModbusSlave_RegisterHandlers(&handlers);
    LOG_INFO("GATEWAY", "service initialized: slave=0x%02X\r\n",
             (unsigned int)GATEWAY_SERVICE_MODBUS_ADDRESS);
}
