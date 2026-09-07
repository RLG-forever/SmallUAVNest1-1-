#include "sequence.h"
#include "motor_control.h"
#include "modbus_master.h"
#include "modbus_common.h"
#include "status_regs.h"
#include "battery_swap.h"
#include "relay.h"
#include "tick.h"
#include "debug_log.h"
#include <string.h>

#define SEQUENCE_STEP_RETRY_LIMIT 2U

// 外部主站写函数

// 序列控制变量
typedef enum {
    SEQUENCE_STATE_IDLE = 0,
    SEQUENCE_STATE_RUNNING,
    SEQUENCE_STATE_WAITING,
    SEQUENCE_STATE_PAUSED
} SequenceState;

// 当前运行的序列
struct {
    SeqId id;
    const StepDef *steps;
    uint8_t step_count;
    uint8_t current_index;
    uint8_t selected_bay;
    SequenceState state;
    SequenceState resume_state;
    SequenceResult last_result;
    uint32_t delay_deadline;
    uint8_t current_step_retry_count;
    uint8_t last_step;
    uint16_t last_error;
    uint8_t step_trace_started;
    uint8_t step_pending_logged;
    uint32_t step_start_tick;
} seq_runner;

static const char *Sequence_GetTraceStepName(SeqId id, uint8_t step_index)
{
    static const char *const takeoff_step_names[] = {
        "open-door", "close-ac", "center-1", "center-2",
        "motor1-fly-lift", "plane-transfer-in", "motor2-near-1",
        "motor2-far-1", "motor2-near-2", "motor2-far-2",
        "motor2-home", "center-2-return", "motor1-home",
        "plane-transfer-out", "leave-center", "close-ac-final",
        "stop-door"
    };
    uint8_t full_index = step_index;

    if (id == SEQ_ID_OPENFLY || id == SEQ_ID_CLOSEFLY) {
        full_index = (uint8_t)(step_index + OPENFLY_STEP_OFFSET);
    } else if (id != SEQ_ID_TAKEOFF) {
        return "generic-action";
    }

    if (full_index >= (uint8_t)(sizeof(takeoff_step_names) /
                                sizeof(takeoff_step_names[0]))) {
        return "invalid-step";
    }
    return takeoff_step_names[full_index];
}

static uint8_t Sequence_IsTimeReached(uint32_t now, uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

static void Sequence_Finish(SequenceResult result)
{
    LOG_INFO("SEQ_TRACE",
             "sequence finished: id=%u, step=%u/%u, name=%s, result=%u, error=0x%04X, elapsed=%lu ms\r\n",
             (unsigned int)seq_runner.id,
             (unsigned int)seq_runner.current_index,
             (unsigned int)seq_runner.step_count,
             Sequence_GetTraceStepName(seq_runner.id,
                                       seq_runner.current_index),
             (unsigned int)result,
             (unsigned int)seq_runner.last_error,
             (unsigned long)(seq_runner.step_trace_started
                 ? (GetTick() - seq_runner.step_start_tick) : 0U));
    seq_runner.last_step = seq_runner.current_index;
    seq_runner.id = SEQ_ID_INVALID;
    seq_runner.steps = NULL;
    seq_runner.step_count = 0U;
    seq_runner.current_index = 0U;
    seq_runner.selected_bay = 0U;
    seq_runner.state = SEQUENCE_STATE_IDLE;
    seq_runner.resume_state = SEQUENCE_STATE_IDLE;
    seq_runner.last_result = result;
    seq_runner.delay_deadline = 0U;
    seq_runner.current_step_retry_count = 0U;
    seq_runner.step_trace_started = 0U;
    seq_runner.step_pending_logged = 0U;
    seq_runner.step_start_tick = 0U;
    StatusRegs_ReleaseSnapshot();
}

static void Sequence_CompleteCurrentStep(void)
{
    const StepDef *step = &seq_runner.steps[seq_runner.current_index];

    LOG_INFO("SEQ_TRACE",
             "step completed: id=%u, step=%u/%u, name=%s, elapsed=%lu ms, completion_reg=0x%02X, completion_value=%u\r\n",
             (unsigned int)seq_runner.id,
             (unsigned int)seq_runner.current_index,
             (unsigned int)seq_runner.step_count,
             Sequence_GetTraceStepName(seq_runner.id,
                                       seq_runner.current_index),
             (unsigned long)(seq_runner.step_trace_started
                 ? (GetTick() - seq_runner.step_start_tick) : 0U),
             (unsigned int)step->completion_reg,
             (unsigned int)step->completion_value);

    if (step->completion_reg != STATUS_REG_NONE) {
        StatusRegs_Update(step->completion_reg, step->completion_value);
    }

    seq_runner.current_index++;
    seq_runner.current_step_retry_count = 0U;
    seq_runner.delay_deadline = 0U;
    seq_runner.step_trace_started = 0U;
    seq_runner.step_pending_logged = 0U;
    seq_runner.step_start_tick = 0U;

    if (seq_runner.current_index >= seq_runner.step_count) {
        Sequence_Finish(SEQUENCE_RESULT_SUCCESS);
    } else {
        seq_runner.state = SEQUENCE_STATE_RUNNING;
        LOG_DEBUG("SEQUENCE", "advance to step %u/%u\r\n",
                  (unsigned int)seq_runner.current_index,
                  (unsigned int)seq_runner.step_count);
    }
}

void Sequence_Pause(void)
{
    if (seq_runner.state != SEQUENCE_STATE_RUNNING &&
        seq_runner.state != SEQUENCE_STATE_WAITING) {
        return;
    }
    
    seq_runner.resume_state = seq_runner.state;
    seq_runner.state = SEQUENCE_STATE_PAUSED;
    LOG_INFO("SEQUENCE", "paused: id=%u, step=%u\r\n",
             (unsigned int)seq_runner.id,
             (unsigned int)seq_runner.current_index);
}

void Sequence_Resume(void)
{
    if (seq_runner.state != SEQUENCE_STATE_PAUSED) {
        return;
    }

    if (seq_runner.resume_state == SEQUENCE_STATE_WAITING) {
        seq_runner.state = SEQUENCE_STATE_WAITING;
    } else {
        seq_runner.state = SEQUENCE_STATE_RUNNING;
    }

    seq_runner.resume_state = SEQUENCE_STATE_IDLE;
    LOG_INFO("SEQUENCE", "resumed: id=%u, step=%u\r\n",
             (unsigned int)seq_runner.id,
             (unsigned int)seq_runner.current_index);
}

void Sequence_Init(void)
{
    memset(&seq_runner, 0, sizeof(seq_runner));
    seq_runner.id = SEQ_ID_INVALID;
    seq_runner.state = SEQUENCE_STATE_IDLE;
    seq_runner.resume_state = SEQUENCE_STATE_IDLE;
    seq_runner.last_result = SEQUENCE_RESULT_NONE;
    seq_runner.last_step = SEQUENCE_STEP_INVALID;
    seq_runner.last_error = 0U;
    LOG_INFO("SEQUENCE", "initialized\r\n");
}

uint8_t Sequence_IsBusy(void)
{
    return (seq_runner.state != SEQUENCE_STATE_IDLE) ? 1U : 0U;
}
SequenceStartResult Sequence_Start(SeqId id)
{
    if (Sequence_IsBusy()) {
        LOG_WARN("SEQUENCE", "start rejected: busy, requested_id=%u\r\n",
                 (unsigned int)id);
        return SEQ_START_BUSY;
    }

    if (MotorControl_IsBusy()) {
        LOG_WARN("SEQUENCE", "start rejected: motor busy, requested_id=%u\r\n",
                 (unsigned int)id);
        return SEQ_START_MASTER_BUSY;
    }

		// 获取当前空仓号（1~3）
    uint8_t empty_bay = SwapState_GetEmptyBay();
    if (empty_bay < 1 || empty_bay > 3) {
        LOG_WARN("SEQUENCE", "invalid empty bay %u; using bay 1\r\n",
                 (unsigned int)empty_bay);
        empty_bay = 1;
    }
    switch (id) {
				case SEQ_ID_OPENDR1:
						seq_runner.steps = opendr1_steps;
						seq_runner.step_count = OPENDR1_STEP_COUNT;
						LOG_INFO("SEQUENCE", "open-door sequence selected\r\n");
				break;
				case SEQ_ID_OPENDR:
						seq_runner.steps = opendr_steps;
						seq_runner.step_count = OPENDR_STEP_COUNT;
						LOG_INFO("SEQUENCE", "open-door sequence selected\r\n");
				break;
				case SEQ_ID_CLOSEDR:
						seq_runner.steps = closedr_steps;
						seq_runner.step_count = CLOSEDR_STEP_COUNT;
						LOG_INFO("SEQUENCE", "close-door sequence selected\r\n");
				break;
			  case SEQ_ID_CLOSECENTER:
						seq_runner.steps = closecenter_steps;
						seq_runner.step_count = CLOSECENTER_STEP_COUNT;
						LOG_INFO("SEQUENCE", "centering sequence selected\r\n");
				break;
				case SEQ_ID_LEAVECENTER:
						seq_runner.steps = leavecenter_steps;
						seq_runner.step_count = LEAVECENTER_STEP_COUNT;
						LOG_INFO("SEQUENCE", "release-center sequence selected\r\n");
				break;
				case SEQ_ID_LOADBATTERY:
                        seq_runner.steps = loadbattery_steps;
                        seq_runner.step_count = LOADBATTERY_STEP_COUNT;
                        LOG_INFO("SEQUENCE", "load-battery sequence selected: bay=%u\r\n",
                                 (unsigned int)empty_bay);
                break;
				case SEQ_ID_DOWNBATTERY:
                    seq_runner.steps = downbattery_steps;
                    seq_runner.step_count = DOWNBATTERY_STEP_COUNT;
                    LOG_INFO("SEQUENCE", "unload-battery sequence selected: bay=%u\r\n",
                             (unsigned int)empty_bay);
                break;
                case SEQ_ID_TAKEOFF:
                        seq_runner.steps = takeoff_steps;
                        seq_runner.step_count = TAKEOFF_STEP_COUNT;
                        LOG_INFO("SEQUENCE", "takeoff sequence selected: bay=%u\r\n",
                                 (unsigned int)empty_bay);
                break;
                case SEQ_ID_LANDING:
                    seq_runner.steps = landing_steps;
                    seq_runner.step_count = LANDING_STEP_COUNT;
                    LOG_INFO("SEQUENCE", "landing sequence selected: bay=%u\r\n",
                             (unsigned int)empty_bay);
                break;
				case SEQ_ID_OPENFLY:
                    seq_runner.steps = &takeoff_steps[OPENFLY_STEP_OFFSET];
                    seq_runner.step_count = OPENFLY_STEP_COUNT;
                    LOG_INFO("SEQUENCE", "open-fly sequence selected\r\n");
                break;
				case SEQ_ID_CLOSEFLY:
                    seq_runner.steps = &takeoff_steps[OPENFLY_STEP_OFFSET];
                    seq_runner.step_count = OPENFLY_STEP_COUNT;
                    LOG_INFO("SEQUENCE", "close-fly sequence selected\r\n");
                break;

//        case SEQ_ID_RECOVERY:
//            seq_runner.steps = landing_steps;
//            seq_runner.step_count = LANDING_STEP_COUNT;
//            printf("取消序列\r\n");
//            break;
//				case SEQ_ID_OPENUP:
//						seq_runner.steps = OpenUp_steps;
//						seq_runner.step_count = OPENUP_STEP_COUNT;
//						printf("开门及/平台上升序列\r\n");
//				break;
//				case SEQ_ID_CLOSEDOWN:
//						seq_runner.steps = downclose_steps;
//						seq_runner.step_count = CLOSEDOWN_STEP_COUNT;
//						printf("关门及/平台下降序列\r\n");
//				break;
        default:
            LOG_ERROR("SEQUENCE", "invalid sequence id=%u\r\n",
                      (unsigned int)id);
            return SEQ_START_INVALID_ID;
    }
		// ========== 拍摄状态快照 ==========
    StatusRegs_TakeSnapshot();

    seq_runner.id = id;
    seq_runner.selected_bay = empty_bay;
    seq_runner.current_index = 0U;
    seq_runner.state = SEQUENCE_STATE_RUNNING;
    seq_runner.resume_state = SEQUENCE_STATE_IDLE;
    seq_runner.last_result = SEQUENCE_RESULT_NONE;
    seq_runner.last_step = SEQUENCE_STEP_INVALID;
    seq_runner.last_error = 0U;
    seq_runner.delay_deadline = 0U;
    seq_runner.current_step_retry_count = 0U;   // 重置重试计数
    seq_runner.step_trace_started = 0U;
    seq_runner.step_pending_logged = 0U;
    seq_runner.step_start_tick = 0U;
    LOG_INFO("SEQUENCE", "started: id=%u, bay=%u, steps=%u\r\n",
             (unsigned int)id, (unsigned int)empty_bay,
             (unsigned int)seq_runner.step_count);
    return SEQ_START_OK;
}


// 执行当前步骤（发送指令）

/**
 * @brief 推进当前序列步骤。
 * @note  步骤或底层 Modbus 批处理未完成时直接返回，下一轮主循环继续推进。
 */
static void Sequence_ExecuteCurrentStep(void)
{
    const StepDef *step = &seq_runner.steps[seq_runner.current_index];
    uint8_t ret;

    if (!seq_runner.step_trace_started) {
        seq_runner.step_trace_started = 1U;
        seq_runner.step_pending_logged = 0U;
        seq_runner.step_start_tick = GetTick();
        LOG_INFO("SEQ_TRACE",
                 "step entered: id=%u, step=%u/%u, name=%s, delay=%lu ms\r\n",
                 (unsigned int)seq_runner.id,
                 (unsigned int)seq_runner.current_index,
                 (unsigned int)seq_runner.step_count,
                 Sequence_GetTraceStepName(seq_runner.id,
                                           seq_runner.current_index),
                 (unsigned long)step->post_delay_ms);
    }

    if (step->run_with_context != NULL) {
        ret = step->run_with_context(step->context);
    } else if (step->run != NULL) {
        ret = step->run();
    } else {
        ret = MODBUS_RESULT_PARAM;
    }

    if (ret == MODBUS_RESULT_PENDING || MotorControl_IsBusy()) {
        if (!seq_runner.step_pending_logged) {
            seq_runner.step_pending_logged = 1U;
            LOG_INFO("SEQ_TRACE",
                     "step pending: id=%u, step=%u, name=%s, action_result=%u, motor_busy=%u\r\n",
                     (unsigned int)seq_runner.id,
                     (unsigned int)seq_runner.current_index,
                     Sequence_GetTraceStepName(seq_runner.id,
                                               seq_runner.current_index),
                     (unsigned int)ret,
                     (unsigned int)MotorControl_IsBusy());
        }
        return;
    }

    if (ret == MODBUS_RESULT_OK) {
        if (step->post_delay_ms > 0U) {
            seq_runner.delay_deadline = GetTick() + step->post_delay_ms;
            seq_runner.state = SEQUENCE_STATE_WAITING;
            LOG_INFO("SEQ_TRACE",
                     "step action accepted: id=%u, step=%u, name=%s; waiting delay=%lu ms\r\n",
                     (unsigned int)seq_runner.id,
                     (unsigned int)seq_runner.current_index,
                     Sequence_GetTraceStepName(seq_runner.id,
                                               seq_runner.current_index),
                     (unsigned long)step->post_delay_ms);
        } else {
            Sequence_CompleteCurrentStep();
        }
        return;
    }

    if (ret == STEP_RESULT_RETRY) {
        if (seq_runner.current_step_retry_count < SEQUENCE_STEP_RETRY_LIMIT) {
            seq_runner.current_step_retry_count++;
            LOG_WARN("SEQUENCE", "retry step %u: %u/%u\r\n",
                     (unsigned int)seq_runner.current_index,
                     (unsigned int)seq_runner.current_step_retry_count,
                     (unsigned int)SEQUENCE_STEP_RETRY_LIMIT);
            return;
        }

        StatusRegs_Update(REG_FAULT_CODE, 0x00FFU);
        seq_runner.last_error = 0x00FFU;
        LOG_ERROR("SEQUENCE", "step %u retry limit reached\r\n",
                  (unsigned int)seq_runner.current_index);
        Sequence_Finish(SEQUENCE_RESULT_RETRY_EXHAUSTED);
        return;
    }

    LOG_ERROR("SEQ_TRACE",
              "step failed: id=%u, step=%u/%u, name=%s, result=%u, elapsed=%lu ms\r\n",
              (unsigned int)seq_runner.id,
              (unsigned int)seq_runner.current_index,
              (unsigned int)seq_runner.step_count,
              Sequence_GetTraceStepName(seq_runner.id,
                                        seq_runner.current_index),
              (unsigned int)ret,
              (unsigned long)(GetTick() - seq_runner.step_start_tick));
    seq_runner.last_error = ret;
    Sequence_Finish(SEQUENCE_RESULT_ACTION_FAILED);
}
void Sequence_Process(void)
{
    switch (seq_runner.state) {
    case SEQUENCE_STATE_IDLE:
    case SEQUENCE_STATE_PAUSED:
        return;

    case SEQUENCE_STATE_WAITING:
        if (Sequence_IsTimeReached(GetTick(), seq_runner.delay_deadline)) {
            Sequence_CompleteCurrentStep();
        }
        return;

    case SEQUENCE_STATE_RUNNING:
        if (seq_runner.current_index < seq_runner.step_count) {
            Sequence_ExecuteCurrentStep();
        } else {
            Sequence_Finish(SEQUENCE_RESULT_SUCCESS);
        }
        return;

    default:
        seq_runner.last_error = MODBUS_RESULT_PARAM;
        Sequence_Finish(SEQUENCE_RESULT_ACTION_FAILED);
        return;
    }
}
static uint8_t HasOldBatteryInUAV(void)
{
    // 读取三块电池的充电状态（0=充电中，1=未充电）
    uint16_t bat1_charge = StatusRegs_Get(REG_BAT1_CHARGE_STATE);
    uint16_t bat2_charge = StatusRegs_Get(REG_BAT2_CHARGE_STATE);
    uint16_t bat3_charge = StatusRegs_Get(REG_BAT3_CHARGE_STATE);

    uint8_t not_charging_count = 0;
    if (bat1_charge == 1) not_charging_count++;
    if (bat2_charge == 1) not_charging_count++;
    if (bat3_charge == 1) not_charging_count++;

    // 只有当恰好一块电池不在充电时，才认为无人机内有旧电池
    return (not_charging_count == 1) ? 1 : 0;
}

/* Start the explicit mechanical recovery sequence. */
SequenceStartResult Sequence_StartRecovery(void)
{
    if (Sequence_IsBusy()) {
        return SEQ_START_BUSY;
    }
    if (MotorControl_IsBusy()) {
        return SEQ_START_MASTER_BUSY;
    }

    if (HasOldBatteryInUAV()) {
        seq_runner.steps = recovery_with_battery_steps;
        seq_runner.step_count = RECOVERY_WITH_BATTERY_COUNT;
        LOG_INFO("SEQUENCE", "starting recovery with battery\r\n");
    } else {
        seq_runner.steps = recovery_without_battery_steps;
        seq_runner.step_count = RECOVERY_WITHOUT_BATTERY_COUNT;
        LOG_INFO("SEQUENCE", "starting recovery without battery\r\n");
    }

    StatusRegs_TakeSnapshot();
    seq_runner.id = SEQ_ID_RECOVERY;
    seq_runner.current_index = 0U;
    seq_runner.state = SEQUENCE_STATE_RUNNING;
    seq_runner.resume_state = SEQUENCE_STATE_IDLE;
    seq_runner.last_result = SEQUENCE_RESULT_NONE;
    seq_runner.last_step = SEQUENCE_STEP_INVALID;
    seq_runner.last_error = 0U;
    seq_runner.delay_deadline = 0U;
    seq_runner.current_step_retry_count = 0U;
    return SEQ_START_OK;
}

SeqId Sequence_GetCurrentId(void)
{
    if (!Sequence_IsBusy()) return SEQ_ID_INVALID;
    return seq_runner.id;
}

uint8_t Sequence_GetCurrentStep(void)
{
    if (!Sequence_IsBusy()) return SEQUENCE_STEP_INVALID;
    return seq_runner.current_index;
}

uint8_t Sequence_GetSelectedBay(void)
{
    return seq_runner.selected_bay;
}

const uint8_t *Sequence_GetCurrentStepMotors(void)
{
    return SequenceSteps_GetMotorList((uint8_t)Sequence_GetCurrentId(),
                                      Sequence_GetCurrentStep());
}

/**
 * @brief 获取最近一次动作序列的最终执行结果。
 * @return 序列执行结果；序列正在运行或尚未执行时返回NONE。
 */
SequenceResult Sequence_GetLastResult(void)
{
    return seq_runner.last_result;
}

/**
 * @brief 获取最近一次动作序列结束时的步骤编号。
 * @return 最后步骤编号；尚无有效结果时返回SEQUENCE_STEP_INVALID。
 */
uint8_t Sequence_GetLastStep(void)
{
    return seq_runner.last_step;
}

/**
 * @brief 获取最近一次动作序列的底层错误码。
 * @return 错误码；成功、取消或尚无错误时返回0。
 */
uint16_t Sequence_GetLastError(void)
{
    return seq_runner.last_error;
}

void Sequence_Stop(void)
{
    if (!Sequence_IsBusy()) {
        LOG_WARN("SEQ_TRACE", "stop requested while sequence is idle\r\n");
        return;
    }

    LOG_WARN("SEQ_TRACE",
             "external stop requested: id=%u, step=%u/%u, name=%s, state=%u, elapsed=%lu ms\r\n",
             (unsigned int)seq_runner.id,
             (unsigned int)seq_runner.current_index,
             (unsigned int)seq_runner.step_count,
             Sequence_GetTraceStepName(seq_runner.id,
                                       seq_runner.current_index),
             (unsigned int)seq_runner.state,
             (unsigned long)(seq_runner.step_trace_started
                 ? (GetTick() - seq_runner.step_start_tick) : 0U));
    MotorControl_Cancel();
    Sequence_Finish(SEQUENCE_RESULT_CANCELLED);
}
