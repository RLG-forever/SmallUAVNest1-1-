#ifndef STATUS_SERVICE_H
#define STATUS_SERVICE_H

#include <stdint.h>

#define STATUS_SERVICE_EXTERNAL_REGISTER_COUNT 28U

/* 业务层使用的状态字段；具体外部寄存器地址只保留在 status_regs.c。 */
typedef enum {
    STATUS_FIELD_NONE = 0,
    STATUS_FIELD_DOOR,
    STATUS_FIELD_CENTER_ROD,
    STATUS_FIELD_SWAP_MECHANISM,
    STATUS_FIELD_FAULT
} StatusServiceField;

uint16_t StatusService_GetBatteryChargeState(uint8_t battery_index);
uint16_t StatusService_GetUavStatus(void);
void StatusService_SetUavStatus(uint16_t value);
void StatusService_Update(StatusServiceField field, uint16_t value);
void StatusService_TakeSnapshot(void);
void StatusService_ReleaseSnapshot(void);
void StatusService_GetExternalBatch(uint16_t start_addr, uint8_t count,
                                    uint8_t *response);

#endif
