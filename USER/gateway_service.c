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
#include <string.h>

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

/* 网关活动命令类型，用于决定后台采用哪种方式推进已接收的命令。 */
typedef enum {
    GATEWAY_ACTIVE_NONE = 0U,       // 当前没有需要后台推进的活动命令
    GATEWAY_ACTIVE_ASYNC_WRITE,     // 下游Modbus写操作未完成，需要重复调用直至得到最终结果
    GATEWAY_ACTIVE_SEQUENCE         // 动作序列已经启动，需要等待整个序列执行结束
} GatewayActiveType;

typedef struct {
    uint8_t active;
    GatewayActiveType type;
    uint16_t register_address;
    uint16_t value;
} GatewayActiveCommand;

static const GatewayCommandMapEntry command_map[] = {
    {GATEWAY_CMD_LIFT_UP,   {MOTOR_CONTROL_TARGET_LIFT_UP}},
    {GATEWAY_CMD_LIFT_DOWN, {MOTOR_CONTROL_TARGET_LIFT_DOWN}}
};

static uint8_t wait_open_fly;
static uint8_t takeoff_power_ready;
static GatewayActiveCommand active_command;
static uint16_t last_write_register = 0xFFFFU;
static uint16_t last_write_value = 0xFFFFU;
static uint32_t last_write_time;
static uint32_t remote_off_deadline;
static uint8_t remote_off_pending;
static uint8_t remote_off_command_active;

static uint8_t GatewayService_ExecuteWrite(uint16_t register_address,
                                           uint16_t value);
static void GatewayService_ProcessActiveCommand(void);

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

    GatewayService_ProcessActiveCommand();

    /* 已接收的下游写命令在完成前持续占用主站事务。 */
    if (active_command.active &&
        active_command.type == GATEWAY_ACTIVE_ASYNC_WRITE) {
        return;
    }

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
    if (result == SEQ_START_BUSY || result == SEQ_START_MASTER_BUSY) {
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

/**
 * @brief 判断收到的写命令是否与当前正在执行的命令完全相同。
 * @param register_address 命令寄存器地址。
 * @param value 命令写入值。
 * @return 相同返回1，否则返回0。
 */
static uint8_t GatewayService_IsSameActive(uint16_t register_address,
                                           uint16_t value)
{
    return active_command.active &&
           active_command.register_address == register_address &&
           active_command.value == value;
}

/**
 * @brief 判断指定寄存器是否为暂停、恢复或取消控制命令。
 * @param register_address 命令寄存器地址。
 * @return 控制命令返回1，否则返回0。
 */
static uint8_t GatewayService_IsControlCommand(uint16_t register_address)
{
    return register_address == GATEWAY_CMD_PAUSE ||
           register_address == GATEWAY_CMD_RESUME ||
           register_address == GATEWAY_CMD_CANCEL;
}

/**
 * @brief 统一更新可供上游主站读取的命令执行状态。
 * @param register_address 当前或最后执行的命令寄存器地址。
 * @param state 命令执行状态。
 * @param fault_code 故障码，无故障时为0。
 * @param step 当前步骤或最后失败步骤。
 */
static void GatewayService_SetCommandStatus(uint16_t register_address,
                                            CommandState state,
                                            uint16_t fault_code,
                                            uint8_t step)
{
    StatusRegs_Update(REG_COMMAND_CODE, register_address);
    StatusRegs_Update(REG_COMMAND_STATE, (uint16_t)state);
    StatusRegs_Update(REG_FAULT_CODE, fault_code);
    StatusRegs_Update(REG_COMMAND_STEP, step);
}

/**
 * @brief 登记一个已被接受、需要后台继续推进的活动命令。
 * @param register_address 命令寄存器地址。
 * @param value 命令写入值。
 * @param type 活动命令类型。
 */
static void GatewayService_BeginActive(uint16_t register_address,
                                       uint16_t value,
                                       GatewayActiveType type)
{
    active_command.active = 1U;
    active_command.type = type;
    active_command.register_address = register_address;
    active_command.value = value;
    GatewayService_SetCommandStatus(register_address,
                                    COMMAND_STATE_EXECUTING, 0U, 0U);
    LOG_INFO("GATEWAY", "command accepted: reg=0x%04X, value=0x%04X, type=%u\r\n",
             (unsigned int)register_address, (unsigned int)value,
             (unsigned int)type);
}

/**
 * @brief 结束当前活动命令并保存最终状态及短时间去重信息。
 * @param state 最终状态。
 * @param fault_code 最终故障码。
 * @param step 最终步骤或失败步骤。
 */
static void GatewayService_FinishActive(CommandState state,
                                        uint16_t fault_code,
                                        uint8_t step)
{
    uint16_t register_address = active_command.register_address;
    uint16_t value = active_command.value;

    GatewayService_SetCommandStatus(register_address, state,
                                    fault_code, step);
    GatewayService_RecordCompletedWrite(register_address, value);
    memset(&active_command, 0, sizeof(active_command));

    if (state == COMMAND_STATE_SUCCESS) {
        LOG_INFO("GATEWAY", "command finished: reg=0x%04X, value=0x%04X\r\n",
                 (unsigned int)register_address, (unsigned int)value);
    } else {
        LOG_ERROR("GATEWAY", "command ended: reg=0x%04X, state=%u, fault=%u, step=%u\r\n",
                  (unsigned int)register_address, (unsigned int)state,
                  (unsigned int)fault_code, (unsigned int)step);
    }
}

/* 执行单个网关寄存器命令，不负责编码从站应答。 */
static uint8_t GatewayService_ExecuteWrite(uint16_t register_address,
                                          uint16_t value)
{
    GatewayCommandTarget target;
    uint8_t result = MODBUS_RESULT_PARAM;

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

    if (result != MODBUS_RESULT_OK &&
        result != MODBUS_RESULT_PENDING && result != MODBUS_RESULT_BUSY) {
        LOG_ERROR("GATEWAY", "command failed: reg=0x%04X, value=0x%04X, result=%u\r\n",
                  (unsigned int)register_address, (unsigned int)value,
                  (unsigned int)result);
    }
    return result;
}

/**
 * @brief 在主循环中推进已接受的异步写命令或监视动作序列结果。
 * @note 命令完成后会更新状态寄存器并清除活动命令标志。
 */
static void GatewayService_ProcessActiveCommand(void)
{
    SequenceResult sequence_result;
    uint16_t fault_code;
    uint8_t result;
    uint8_t step;

    if (!active_command.active) {
        return;
    }

    if (active_command.type == GATEWAY_ACTIVE_SEQUENCE) {
        if (Sequence_IsBusy()) {
            StatusRegs_Update(REG_COMMAND_STEP, Sequence_GetCurrentStep());
            return;
        }

        sequence_result = Sequence_GetLastResult();
        step = Sequence_GetLastStep();
        if (sequence_result == SEQUENCE_RESULT_SUCCESS) {
            GatewayService_FinishActive(COMMAND_STATE_SUCCESS, 0U, step);
        } else if (sequence_result == SEQUENCE_RESULT_CANCELLED) {
            GatewayService_FinishActive(COMMAND_STATE_CANCELLED, 0U, step);
        } else {
            fault_code = Sequence_GetLastError();
            if (fault_code == 0U) {
                fault_code = 0x00FFU;
            }
            GatewayService_FinishActive(COMMAND_STATE_FAILED,
                                        fault_code, step);
        }
        return;
    }

    result = GatewayService_ExecuteWrite(active_command.register_address,
                                         active_command.value);
    if (result == MODBUS_RESULT_PENDING || result == MODBUS_RESULT_BUSY) {
        return;
    }
    if (result != MODBUS_RESULT_OK) {
        GatewayService_FinishActive(COMMAND_STATE_FAILED,
                                    result, 0U);
        return;
    }

    /* 部分命令需要先完成下游写操作，再启动动作序列。 */
    if (Sequence_IsBusy()) {
        active_command.type = GATEWAY_ACTIVE_SEQUENCE;
        StatusRegs_Update(REG_COMMAND_STEP, Sequence_GetCurrentStep());
        return;
    }

    GatewayService_FinishActive(COMMAND_STATE_SUCCESS, 0U, 0U);
}

/**
 * @brief 处理暂停、恢复和取消命令，这些命令允许在设备忙碌时执行。
 * @param request 上游主站的06写请求。
 * @return 返回0表示应答已发送，非0表示需要在后续主循环重试。
 */
static uint8_t GatewayService_HandleControlCommand(
    const ModbusSlaveRequest *request)
{
    uint8_t step = 0U;

    if (request->start_register == GATEWAY_CMD_PAUSE) {
        Sequence_Pause();
        if (active_command.active &&
            active_command.type == GATEWAY_ACTIVE_SEQUENCE) {
            step = Sequence_GetCurrentStep();
            StatusRegs_Update(REG_COMMAND_STATE, COMMAND_STATE_PAUSED);
            StatusRegs_Update(REG_COMMAND_STEP, step);
        }
    } else if (request->start_register == GATEWAY_CMD_RESUME) {
        Sequence_Resume();
        if (active_command.active &&
            active_command.type == GATEWAY_ACTIVE_SEQUENCE) {
            step = Sequence_GetCurrentStep();
            StatusRegs_Update(REG_COMMAND_STATE, COMMAND_STATE_EXECUTING);
            StatusRegs_Update(REG_COMMAND_STEP, step);
        }
    } else {
        if (active_command.active) {
            if (active_command.type == GATEWAY_ACTIVE_SEQUENCE) {
                Sequence_Stop();
                step = Sequence_GetLastStep();
            } else {
                ModbusMaster_Cancel();
            }
            GatewayService_FinishActive(COMMAND_STATE_CANCELLED, 0U, step);
        } else {
            Sequence_Stop();
        }
    }

    return ModbusSlave_SendWriteAck(request) != 0U;
}

/**
 * @brief 尝试接受并启动一个新的06写命令。
 * @param request 上游主站的06写请求。
 * @return 返回0表示应答已发送，非0表示需要在后续主循环重试。
 * @note 内部返回PENDING时立即正常应答，并由活动命令状态机继续执行。
 */
static uint8_t GatewayService_AcceptNewCommand(
    const ModbusSlaveRequest *request)
{
    uint8_t result = GatewayService_ExecuteWrite(request->start_register,
                                                 request->value);

    if (result == MODBUS_RESULT_BUSY) {
        return ModbusSlave_SendException(request, 0x06U) != 0U;
    }
    if (result != MODBUS_RESULT_OK && result != MODBUS_RESULT_PENDING) {
        uint8_t exception_code = result == MODBUS_RESULT_PARAM
                                 ? 0x02U : 0x04U;
        return ModbusSlave_SendException(request, exception_code) != 0U;
    }

    if (result == MODBUS_RESULT_PENDING) {
        GatewayService_BeginActive(request->start_register, request->value,
                                   GATEWAY_ACTIVE_ASYNC_WRITE);
    } else if (Sequence_IsBusy()) {
        GatewayService_BeginActive(request->start_register, request->value,
                                   GATEWAY_ACTIVE_SEQUENCE);
        StatusRegs_Update(REG_COMMAND_STEP, Sequence_GetCurrentStep());
    } else {
        GatewayService_SetCommandStatus(request->start_register,
                                        COMMAND_STATE_SUCCESS, 0U, 0U);
        GatewayService_RecordCompletedWrite(request->start_register,
                                            request->value);
    }

    return ModbusSlave_SendWriteAck(request) != 0U;
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
    if (GatewayService_IsSameActive(request->start_register,
                                    request->value)) {
        LOG_DEBUG("GATEWAY", "active duplicate acknowledged: reg=0x%04X, value=0x%04X\r\n",
                  (unsigned int)request->start_register,
                  (unsigned int)request->value);
        return ModbusSlave_SendWriteAck(request) != 0U;
    }

    if (GatewayService_IsControlCommand(request->start_register)) {
        return GatewayService_HandleControlCommand(request);
    }

    if (active_command.active) {
        return ModbusSlave_SendException(request, 0x06U) != 0U;
    }

    if (GatewayService_IsRecentDuplicate(request->start_register,
                                         request->value)) {
        LOG_DEBUG("GATEWAY", "recent duplicate acknowledged: reg=0x%04X, value=0x%04X\r\n",
                  (unsigned int)request->start_register,
                  (unsigned int)request->value);
        return ModbusSlave_SendWriteAck(request) != 0U;
    }

    return GatewayService_AcceptNewCommand(request);
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

    if (active_command.active) {
        active = 0U;
        index = 0U;
        if (request->register_count == 1U &&
            GatewayService_IsSameActive(register_address, value)) {
            return ModbusSlave_SendWriteAck(request) != 0U;
        }
        return ModbusSlave_SendException(request, 0x06U) != 0U;
    }

    result = GatewayService_ExecuteWrite(register_address, value);
    if (result == MODBUS_RESULT_PENDING) {
        return 1U;
    }
    if (result == MODBUS_RESULT_BUSY) {
        active = 0U;
        index = 0U;
        return ModbusSlave_SendException(request, 0x06U) != 0U;
    }
    if (result != MODBUS_RESULT_OK) {
        active = 0U;
        index = 0U;
        return ModbusSlave_SendException(request, 0x02U) != 0U;
    }

    GatewayService_RecordCompletedWrite(register_address, value);

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
    memset(&active_command, 0, sizeof(active_command));
    last_write_register = 0xFFFFU;
    last_write_value = 0xFFFFU;
    last_write_time = 0U;
    GatewayService_SetCommandStatus(0U, COMMAND_STATE_IDLE, 0U, 0U);
    ModbusSlave_RegisterHandlers(&handlers);
    LOG_INFO("GATEWAY", "service initialized: slave=0x%02X\r\n",
             (unsigned int)GATEWAY_SERVICE_MODBUS_ADDRESS);
}
