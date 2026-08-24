/*==================================================================================
This document is the property of Nanjing Kangni Mechanical & Electrical Co., Ltd. It may
not be reproduced or disclosed to third party without prior to authorisation
 * header file name: STM103REG.h      
 * heafer file description: define the date type and initalization used by the timer module
 * service condition: peoject.h
 * version information: Wan Lei, V0100-0000, 20180104
===================================================================================*/

/*==================================================================================
     avoid redefinition
===================================================================================*/
#ifndef STM103REG
#define STM103REG

/**********************************************/
#define   U8    unsigned char        //无符号8位
#define   U16   unsigned short       //无符号16位
#define   U32   unsigned long        //无符号32位
#define   U64   unsigned long long   //无符号64位
#define   S8    signed char          //有符号8位
#define   S16   signed short         //有符号16位
#define   S32   signed long          //有符号32位
#define   S64   signed long	long     //有符号64位
/* MACRO DEFINE */
#define __UMEM8(addr)  (*((volatile unsigned char  *)addr))
#define __UMEM16(addr) (*((volatile unsigned short *)addr))
#define __UMEM32(addr) (*((volatile unsigned long  *)addr))
/*************************内存地址****************************/
#define opendst_addr        0X08060000  


// /********************变量定义********************/


extern U8 stalling_count1;		   //堵转计数器
extern U8 stalling_count2;		   //堵转计数器

#endif
/*==================================================================================
     the end of file
===================================================================================*/
