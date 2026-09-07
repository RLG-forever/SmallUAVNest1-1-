#include "exec_steps.h"

#include "modbus_common.h"

#include <stddef.h>

uint8_t ExecSteps_MoveToAbsPos(const void *context)
{
    const ExecMoveAbsPosParams *params =
        (const ExecMoveAbsPosParams *)context;

    if (params == NULL || params->motors == NULL || params->count == 0U) {
        return MODBUS_RESULT_PARAM;
    }

    return MotorControl_MoveToAbsPos(params->motors, params->count, NULL);
}
