#include "gateway_commands.h"

#include <stddef.h>

typedef struct {
    uint16_t gateway_register;
    GatewayCommandTarget target;
} GatewayCommandMapEntry;

/* 这里只保存无需状态机的一对一下游转发命令。 */
static const GatewayCommandMapEntry command_map[] = {
    {GATEWAY_CMD_LIFT_UP,         {MOTOR_SERVICE_TARGET_LIFT_UP}},
    {GATEWAY_CMD_LIFT_DOWN,       {MOTOR_SERVICE_TARGET_LIFT_DOWN}}
};

uint8_t GatewayCommands_FindTarget(uint16_t gateway_register,
                                   GatewayCommandTarget *target)
{
    uint16_t index;

    if (target == NULL) {
        return 0U;
    }
    for (index = 0U; index < (uint16_t)(sizeof(command_map) / sizeof(command_map[0]));
         index++) {
        if (command_map[index].gateway_register == gateway_register) {
            *target = command_map[index].target;
            return 1U;
        }
    }
    return 0U;
}
