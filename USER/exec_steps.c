#include "exec_steps.h"

#include "modbus_common.h"

#include <stddef.h>

/* 功能：校验上下文后，把步骤表参数转交给异步绝对位置批处理。 */
uint8_t ExecSteps_MoveToAbsPos(const void *context)
{
    const ExecMoveAbsPosParams *params =
        (const ExecMoveAbsPosParams *)context;

    if (params == NULL || params->motors == NULL || params->count == 0U) {
        return MODBUS_RESULT_PARAM;
    }

    return MotorControl_MoveToAbsPos(params->motors, params->count, NULL);
}
