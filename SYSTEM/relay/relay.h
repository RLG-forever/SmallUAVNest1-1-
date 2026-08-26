#ifndef RELAY_H
#define RELAY_H

/* 板级继电器驱动：仅管理继电器GPIO，不依赖Modbus。 */
typedef enum {
    RELAY_STOP = 0,
    RELAY_FORWARD,
    RELAY_BACKWARD
} RelayState;

/* 调用任何控制接口前，必须先完成一次GPIO初始化。 */
void Relay_Init(void);
void Relay_Control(RelayState state);
void Relay_Forward(void);
void Relay_Backward(void);
void Relay_Stop(void);
void RELAY(void);

#endif
