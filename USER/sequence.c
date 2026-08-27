#include "sequence.h"
#include "motor_control.h"
#include "modbus_master.h"
#include "modbus_common.h"
#include "status_regs.h"
#include "battery_swap.h"
#include "relay.h"
#include "tick.h"
#include <stdio.h>
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

typedef enum {
    SEQUENCE_RESULT_NONE = 0,
    SEQUENCE_RESULT_SUCCESS,
    SEQUENCE_RESULT_CANCELLED,
    SEQUENCE_RESULT_ACTION_FAILED,
    SEQUENCE_RESULT_RETRY_EXHAUSTED
} SequenceResult;
// 当前运行的序列
struct {
    SeqId id;
    const StepDef *steps;
    uint8_t step_count;
    uint8_t current_index;
    SequenceState state;
    SequenceState resume_state;
    SequenceResult last_result;
    uint32_t delay_deadline;
    uint8_t current_step_retry_count;
} seq_runner;

static uint8_t Sequence_IsTimeReached(uint32_t now, uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

static void Sequence_Finish(SequenceResult result)
{
    seq_runner.id = SEQ_ID_INVALID;
    seq_runner.steps = NULL;
    seq_runner.step_count = 0U;
    seq_runner.current_index = 0U;
    seq_runner.state = SEQUENCE_STATE_IDLE;
    seq_runner.resume_state = SEQUENCE_STATE_IDLE;
    seq_runner.last_result = result;
    seq_runner.delay_deadline = 0U;
    seq_runner.current_step_retry_count = 0U;
    StatusRegs_ReleaseSnapshot();
}

static void Sequence_CompleteCurrentStep(void)
{
    const StepDef *step = &seq_runner.steps[seq_runner.current_index];

    if (step->completion_reg != STATUS_REG_NONE) {
        StatusRegs_Update(step->completion_reg, step->completion_value);
    }

    seq_runner.current_index++;
    seq_runner.current_step_retry_count = 0U;
    seq_runner.delay_deadline = 0U;

    if (seq_runner.current_index >= seq_runner.step_count) {
        printf("Sequence completed\r\n");
        Sequence_Finish(SEQUENCE_RESULT_SUCCESS);
    } else {
        seq_runner.state = SEQUENCE_STATE_RUNNING;
        printf("Sequence advances to step %d\r\n", seq_runner.current_index);
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
    printf("Sequence paused\r\n");
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
    printf("Sequence resumed\r\n");
}

void Sequence_Init(void)
{
    memset(&seq_runner, 0, sizeof(seq_runner));
    seq_runner.id = SEQ_ID_INVALID;
    seq_runner.state = SEQUENCE_STATE_IDLE;
    seq_runner.resume_state = SEQUENCE_STATE_IDLE;
    seq_runner.last_result = SEQUENCE_RESULT_NONE;
}

uint8_t Sequence_IsBusy(void)
{
    return (seq_runner.state != SEQUENCE_STATE_IDLE) ? 1U : 0U;
}
SequenceStartResult Sequence_Start(SeqId id)
{
    if (Sequence_IsBusy()) {
        printf("序列已在执行中，忽略新请求\r\n");
        return SEQ_START_BUSY;
    }

    if (MotorControl_IsBusy()) {
        return SEQ_START_MASTER_BUSY;
    }

		// 获取当前空仓号（1~3）
    uint8_t empty_bay = SwapState_GetEmptyBay();
    if (empty_bay < 1 || empty_bay > 3) {
        printf("Invalid empty bay %d; using bay 1\n", empty_bay);
        empty_bay = 1;
    }
    uint8_t index = empty_bay - 1; // 数组索引

    switch (id) {
				case SEQ_ID_OPENDR1:
						seq_runner.steps = opendr1_steps;
						seq_runner.step_count = OPENDR1_STEP_COUNT;
						printf("打开舱门序列\r\n");
				break;
				case SEQ_ID_OPENDR:
						seq_runner.steps = opendr_steps;
						seq_runner.step_count = OPENDR_STEP_COUNT;
						printf("打开舱门序列\r\n");
				break;
				case SEQ_ID_CLOSEDR:
						seq_runner.steps = closedr_steps;
						seq_runner.step_count = CLOSEDR_STEP_COUNT;
						printf("关闭舱门序列\r\n");
				break;
			  case SEQ_ID_CLOSECENTER:
						seq_runner.steps = closecenter_steps;
						seq_runner.step_count = CLOSECENTER_STEP_COUNT;
						printf("居中杆居中序列\r\n");
				break;
				case SEQ_ID_LEAVECENTER:
						seq_runner.steps = leavecenter_steps;
						seq_runner.step_count = LEAVECENTER_STEP_COUNT;
						printf("居中杆释放序列\r\n");
				break;
				case SEQ_ID_LOADBATTERY:
				{
						const StepDef *array[] = {loadbattery_steps_1, loadbattery_steps_2, loadbattery_steps_3};
                        const uint8_t counts[] = {LOADBATTERY_STEPS_1_COUNT, LOADBATTERY_STEPS_2_COUNT, LOADBATTERY_STEPS_3_COUNT};
                        seq_runner.steps = array[index];
                        seq_runner.step_count = counts[index];
                        printf("启动装电池序列，空仓=%d\n", empty_bay);
                        break;
				}
				case SEQ_ID_DOWNBATTERY:
				{
					  const StepDef *array[] = {downbattery_steps_1, downbattery_steps_2, downbattery_steps_3};
                    const uint8_t counts[] = {DOWNBATTERY_STEPS_1_COUNT, DOWNBATTERY_STEPS_2_COUNT, DOWNBATTERY_STEPS_3_COUNT};
                    seq_runner.steps = array[index];
                    seq_runner.step_count = counts[index];
                    printf("启动下电池序列，空仓=%d\n", empty_bay);
                break;
				}
                case SEQ_ID_TAKEOFF:
				{
						const StepDef *array[] = {takeoff_steps_1, takeoff_steps_2, takeoff_steps_3};
                        const uint8_t counts[] = {TAKEOFF_STEPS_1_COUNT, TAKEOFF_STEPS_2_COUNT, TAKEOFF_STEPS_3_COUNT};
                        seq_runner.steps = array[index];
                        seq_runner.step_count = counts[index];
                        printf("Starting takeoff sequence; bay=%d\n", empty_bay);
                break;
				}
                case SEQ_ID_LANDING:
				{
                    const StepDef *array[] = {landing_steps_1, landing_steps_2, landing_steps_3};
                    const uint8_t counts[] = {LANDING_STEPS_1_COUNT, LANDING_STEPS_2_COUNT, LANDING_STEPS_3_COUNT};
                    seq_runner.steps = array[index];
                    seq_runner.step_count = counts[index];
                    printf("Starting landing sequence; bay=%d\n", empty_bay);
                break;
				}
				case SEQ_ID_OPENFLY:
                    seq_runner.steps = openfly_steps;
                    seq_runner.step_count = OPENFLY_STEP_COUNT;
                    printf("飞机开机完成序列\r\n");
                break;
				case SEQ_ID_CLOSEFLY:
                    seq_runner.steps = closefly_steps;
                    seq_runner.step_count = CLOSEFLY_STEP_COUNT;
                    printf("飞机关机完成序列\r\n");
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
            return SEQ_START_INVALID_ID;
    }
		// ========== 拍摄状态快照 ==========
    StatusRegs_TakeSnapshot();

    seq_runner.id = id;
    seq_runner.current_index = 0U;
    seq_runner.state = SEQUENCE_STATE_RUNNING;
    seq_runner.resume_state = SEQUENCE_STATE_IDLE;
    seq_runner.last_result = SEQUENCE_RESULT_NONE;
    seq_runner.delay_deadline = 0U;
    seq_runner.current_step_retry_count = 0U;   // 重置重试计数
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
    uint8_t ret = step->run();

    if (ret == MODBUS_RESULT_PENDING || MotorControl_IsBusy()) {
        return;
    }

    if (ret == MODBUS_RESULT_OK) {
        if (step->post_delay_ms > 0U) {
            seq_runner.delay_deadline = GetTick() + step->post_delay_ms;
            seq_runner.state = SEQUENCE_STATE_WAITING;
            printf("Step %d completed; waiting %d ms\r\n",
                   seq_runner.current_index, step->post_delay_ms);
        } else {
            Sequence_CompleteCurrentStep();
        }
        return;
    }

    if (ret == STEP_RESULT_RETRY) {
        if (seq_runner.current_step_retry_count < SEQUENCE_STEP_RETRY_LIMIT) {
            seq_runner.current_step_retry_count++;
            printf("Retrying step %d: %d/%d\r\n",
                   seq_runner.current_index,
                   seq_runner.current_step_retry_count,
                   SEQUENCE_STEP_RETRY_LIMIT);
            return;
        }

        StatusRegs_Update(REG_FAULT_CODE, 0x00FFU);
        printf("Step %d retry limit reached\r\n",
               seq_runner.current_index);
        Sequence_Finish(SEQUENCE_RESULT_RETRY_EXHAUSTED);
        return;
    }

    printf("Step %d failed with result %d\r\n",
           seq_runner.current_index, ret);
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
        printf("Starting recovery sequence with battery\r\n");
    } else {
        seq_runner.steps = recovery_without_battery_steps;
        seq_runner.step_count = RECOVERY_WITHOUT_BATTERY_COUNT;
        printf("Starting recovery sequence without battery\r\n");
    }

    StatusRegs_TakeSnapshot();
    seq_runner.id = SEQ_ID_RECOVERY;
    seq_runner.current_index = 0U;
    seq_runner.state = SEQUENCE_STATE_RUNNING;
    seq_runner.resume_state = SEQUENCE_STATE_IDLE;
    seq_runner.last_result = SEQUENCE_RESULT_NONE;
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
    if (!Sequence_IsBusy()) return 0xFFU;   // 序列未运行返回0xFF
    return seq_runner.current_index;
}

const uint8_t *Sequence_GetCurrentStepMotors(void)
{
    return SequenceSteps_GetMotorList((uint8_t)Sequence_GetCurrentId(),
                                      Sequence_GetCurrentStep());
}
void Sequence_Stop(void)
{
    if (!Sequence_IsBusy()) {
        return;
    }

    MotorControl_Cancel();
    Sequence_Finish(SEQUENCE_RESULT_CANCELLED);
}
