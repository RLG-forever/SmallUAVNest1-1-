#ifndef __RINGBUFFER_H
#define __RINGBUFFER_H
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "door.h"
#include "diagnose.h"


#define  RINGBUFF_LEN          (4096)     //定义最大接收字节数 4096
#define  RINGBUFF_OK           1     
#define  RINGBUFF_ERR          0   
typedef struct
{
    uint16_t Head;           
    uint16_t Tail;
    uint16_t Lenght;
    uint8_t  Ring_data[RINGBUFF_LEN];
}RingBuff_t;
RingBuff_t RxRingBuff_4G;//创建一个4G串口接收ringBuff缓冲区

#endif
