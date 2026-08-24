#include "ringbuffer.h"
#include "stdlib.h"
#include "string.h"
#include "usart.h"	
#include "led.h"
#include "can.h"
#include "diagnose.h"
#include "door.h"
#include "crc_library.h"
#include "project_cfg.h"

/**
* @brief  RingBuff_Init//环形存储器初始化，将环形缓冲区的头，尾和长度清零，表示没有任何数据存入。
* @param  void
* @return void
* @note   初始化环形缓冲区
*/
void RingBuff_Init(void)
{
  //初始化相关信息
  RxRingBuff_4G.Head = 0;
  RxRingBuff_4G.Tail = 0;
  RxRingBuff_4G.Lenght = 0;
}
/**
* @brief  Write_RingBuff
* @param  uint8_t data
* @return FLASE:环形缓冲区已满，写入失败;TRUE:写入成功
* @note   往环形缓冲区写入uint8_t类型的数据
*/
uint8_t Write_RingBuff(uint8_t data)
{
  if(RxRingBuff_4G.Lenght >= RINGBUFF_LEN) //判断缓冲区是否已满
  {
    return RINGBUFF_ERR;
  }
  RxRingBuff_4G.Ring_data[RxRingBuff_4G.Tail]=data;
  RxRingBuff_4G.Tail = (RxRingBuff_4G.Tail+1)%RINGBUFF_LEN;//防止越界非法访问
  RxRingBuff_4G.Lenght++;
  return RINGBUFF_OK;
}
/**
* @brief  Read_RingBuff
* @param  uint8_t *rData，用于保存读取的数据
* @return FLASE:环形缓冲区没有数据，读取失败;TRUE:读取成功
* @note   从环形缓冲区读取一个u8类型的数据
*/
uint8_t Read_RingBuff(uint8_t *rData)
{
  if(RxRingBuff_4G.Lenght == 0)//判断非空
  {
    return RINGBUFF_ERR;
  }
  *rData = RxRingBuff_4G.Ring_data[RxRingBuff_4G.Head];//先进先出FIFO，从缓冲区头出
  RxRingBuff_4G.Head = (RxRingBuff_4G.Head+1)%RINGBUFF_LEN;//防止越界非法访问
  RxRingBuff_4G.Lenght--;
  return RINGBUFF_OK;
}