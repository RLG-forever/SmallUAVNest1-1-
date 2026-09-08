#ifndef SEQUENCE_H
#define SEQUENCE_H

#include <stdint.h>
#include "sequence_steps.h"
#include "status_regs.h"

/* 表示当前没有有效的序列步骤。 */
#define SEQUENCE_STEP_INVALID 0xFFU


// 序列ID
typedef enum {
    SEQ_ID_TAKEOFF = 0,   // 一键起飞
    SEQ_ID_LANDING,       // 降落完成
		SEQ_ID_CLOSECENTER,   // 居中杆居中
		SEQ_ID_LEAVECENTER,   // 居中杆释放
		SEQ_ID_LOADBATTERY,   // 装电池
		SEQ_ID_DOWNBATTERY,   // 下电池
		SEQ_ID_OPENUP,    		// 开门上升
		SEQ_ID_CLOSEDOWN,    	// 下降关门
		SEQ_ID_OPENFLY,    	  // 飞机开机
	  SEQ_ID_CLOSEFLY,    	// 飞机关机
		SEQ_ID_OPENDR,         // 打开舱门
	  SEQ_ID_CLOSEDR,       // 关闭舱门
    SEQ_ID_RECOVERY,       /* explicit recovery sequence */
		SEQ_ID_OPENDR1,         // 打开舱门
    SEQ_ID_INVALID = 0xFF
} SeqId;

/* Sequence_Start 的明确返回结果，调用者不应再通过全局状态猜测。 */
typedef enum {
    SEQ_START_OK = 0,          /* 已成功创建并启动序列 */
    SEQ_START_BUSY,            /* 已有序列正在运行 */
    SEQ_START_INVALID_ID,      /* 序列 ID 不受支持 */
    SEQ_START_MASTER_BUSY      /* 主站事务或电机批处理仍被占用 */
} SequenceStartResult;

typedef enum {
    SEQUENCE_RESULT_NONE = 0,
    SEQUENCE_RESULT_SUCCESS,
    SEQUENCE_RESULT_CANCELLED,
    SEQUENCE_RESULT_ACTION_FAILED,
    SEQUENCE_RESULT_RETRY_EXHAUSTED
} SequenceResult;


// 初始化序列模块
void Sequence_Init(void);

// 启动指定序列（由从站回调调用）
SequenceStartResult Sequence_Start(SeqId id);

// 查询是否有序列正在执行
uint8_t Sequence_IsBusy(void);
/* 自动告警恢复（松开夹紧、回原点或重启序列）进行中时返回 1。 */
uint8_t Sequence_IsRecovering(void);

// 序列处理函数（需在主循环中周期性调用）
void Sequence_Process(void);

/*
 * 暂停只阻止状态机推进和新动作下发，不暂停已经开始的步骤等待计时。
 * 若等待在暂停期间到期，恢复后的下一次 Sequence_Process() 将立即推进。
 */
void Sequence_Pause(void);   // 暂停当前序列
void Sequence_Resume(void);  // 恢复执行
/* Stop the active sequence only; this never starts recovery steps. */
void Sequence_Stop(void);

/* Start the explicit mechanical recovery sequence while the runner is idle. */
SequenceStartResult Sequence_StartRecovery(void);

SeqId Sequence_GetCurrentId(void);
uint8_t Sequence_GetCurrentStep(void);
uint8_t Sequence_GetSelectedBay(void);
const uint8_t *Sequence_GetCurrentStepMotors(void);
SequenceResult Sequence_GetLastResult(void);
uint8_t Sequence_GetLastStep(void);
uint16_t Sequence_GetLastError(void);

#endif
