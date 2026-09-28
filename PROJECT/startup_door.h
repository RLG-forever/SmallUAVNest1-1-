#ifndef STARTUP_DOOR_H
#define STARTUP_DOOR_H

#include <stdint.h>

/* 设为 0U 时跳过上电舱门回零、关门到位检测和打开空调。 */
#ifndef STARTUP_DOOR_SEQUENCE_ENABLE
#define STARTUP_DOOR_SEQUENCE_ENABLE 0U
#endif

/* 非阻塞推进上电舱门流程，返回 MODBUS_RESULT_*。 */
uint8_t StartupDoorSequence_Task(void);

#endif
