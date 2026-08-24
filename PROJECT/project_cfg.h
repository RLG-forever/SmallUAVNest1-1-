/*==================================================================================
This document is the property of Nanjing Kangni Mechanical & Electrical Co., Ltd. It may
not be reproduced or disclosed to third party without prior to authorisation
 * header file name: project_cfg.h      
 * heafer file description: define the constant date and initalization value used by the other module
 * service condition: 
 * version information: Li.Yuanyuan, V0100-0000, 20210729
===================================================================================*/

/*==================================================================================
		 文件名定义
===================================================================================*/
#ifndef __PROJECT_CFG_H
#define	__PROJECT_CFG_H

/*==================================================================================
     常量定义
===================================================================================*/
/* 软、硬件件版本*/   
#define DFLT_SOFT_TYPE                  'T'          // V:正式出厂程序,T:测试程序
#define DFLT_SOFT_VER                   0x0001U      // 软件版本号
#define DFLT_HARD_VER                   0x0001U      // 硬件版本号
#define PROJECT_ID                      0x0071U      // 项目号，暂时写一个，后面确定了再改

/* 上行、下行集中控制器使用定义*/
#define UPLINK_CONTROLLER			          0        // 1:使用上行集中控制器；0：未使用上行集中控制器
#define DOWNLINK_CONTROLLER			        1         // 1:使用下行集中控制器；0：未使用下行集中控制器

#define QNWLOCK	                            0        //集中控制器4G模块锁频点功能是否启用

//电机编号（上送服务器用电机ID）
#define ID_UPMTRL1                      0x00000001
#define ID_UPMTRL2                      0x00000002
#define ID_UPMTRL3                      0x00000003
#define ID_UPMTRL4                      0x00000004
#define ID_UPMTRL5                      0x00000005
#define ID_UPMTRL6                      0x00000006
#define ID_UPMTRL7                      0x00000007
#define ID_UPMTRL8                      0x00000008
#define ID_UPMTRL9                      0x00000009
#define ID_UPMTRL10                     0x0000000A
#define ID_UPMTRL11                     0x0000000B
#define ID_UPMTRL12                     0x0000000C
#define ID_UPMTRH1                      0x00000000
#define ID_UPMTRH2                      0x00000000
#define ID_UPMTRH3                      0x00000000
#define ID_UPMTRH4                      0x00000000
#define ID_UPMTRH5                      0x00000000
#define ID_UPMTRH6                      0x00000000
#define ID_UPMTRH7                      0x00000000
#define ID_UPMTRH8                      0x00000000
#define ID_UPMTRH9                      0x00000000
#define ID_UPMTRH10                      0x00000000
#define ID_UPMTRH11                      0x00000000
#define ID_UPMTRH12                      0x00000000

#define ID_DOWNMTRL1                     0x0000000D
#define ID_DOWNMTRL2                     0x0000000E
#define ID_DOWNMTRL3                     0x0000000F
#define ID_DOWNMTRL4                     0x00000010
#define ID_DOWNMTRL5                     0x00000011
#define ID_DOWNMTRL6                     0x00000012
#define ID_DOWNMTRL7                     0x00000013
#define ID_DOWNMTRL8                     0x00000014
#define ID_DOWNMTRL9                     0x00000015
#define ID_DOWNMTRL10                    0x00000016
#define ID_DOWNMTRL11                    0x00000017
#define ID_DOWNMTRL12                    0x00000018
#define ID_DOWNMTRH1                     0x00000000
#define ID_DOWNMTRH2                     0x00000000
#define ID_DOWNMTRH3                     0x00000000
#define ID_DOWNMTRH4                     0x00000000
#define ID_DOWNMTRH5                     0x00000000
#define ID_DOWNMTRH6                     0x00000000
#define ID_DOWNMTRH7                     0x00000000
#define ID_DOWNMTRH8                     0x00000000
#define ID_DOWNMTRH9                     0x00000000
#define ID_DOWNMTRH10                     0x00000000
#define ID_DOWNMTRH11                     0x00000000
#define ID_DOWNMTRH12                     0x00000000

/* 输入口*/
#define IPORT01                         GPIOD
#define IPORT01_PIN                     GPIO_Pin_7
#define IPORT02                         GPIOD
#define IPORT02_PIN                     GPIO_Pin_6
#define IPORT03                         GPIOD
#define IPORT03_PIN                     GPIO_Pin_5
#define IPORT04                         GPIOD
#define IPORT04_PIN                     GPIO_Pin_4
#define IPORT05                         GPIOD
#define IPORT05_PIN                     GPIO_Pin_3
#define IPORT06                         GPIOD
#define IPORT06_PIN                     GPIO_Pin_2

/* 输出口*/                           
#define OPORT01                         GPIOA
#define OPORT01_PIN                     GPIO_Pin_9
#define OPORT02                         GPIOA
#define OPORT02_PIN                     GPIO_Pin_10

#define OPORT03                         GPIOE
#define OPORT03_PIN                     GPIO_Pin_14
#define OPORT04                         GPIOE
#define OPORT04_PIN                     GPIO_Pin_15

/* 10ms时基定时时间*/
#define DFLT_300_MS_T10MS               30      
#define DFLT_200_MS_T10MS               20      
#define DFLT_100_MS_T10MS               10      
#define DFLT_50_MS_T10MS                5   

/* 输入口滤波时间*/
#define DFLT_II01_FLT_TIME_UP           DFLT_200_MS_T10MS      
#define DFLT_II01_FLT_TIME_DN           DFLT_200_MS_T10MS
#define DFLT_II02_FLT_TIME_UP           DFLT_200_MS_T10MS      
#define DFLT_II02_FLT_TIME_DN           DFLT_200_MS_T10MS
#define DFLT_II03_FLT_TIME_UP           DFLT_200_MS_T10MS           
#define DFLT_II03_FLT_TIME_DN           DFLT_200_MS_T10MS
#define DFLT_II04_FLT_TIME_UP           DFLT_200_MS_T10MS      
#define DFLT_II04_FLT_TIME_DN           DFLT_200_MS_T10MS
#define DFLT_II05_FLT_TIME_UP           DFLT_200_MS_T10MS       
#define DFLT_II05_FLT_TIME_DN           DFLT_200_MS_T10MS
#define DFLT_II06_FLT_TIME_UP           DFLT_200_MS_T10MS      
#define DFLT_II06_FLT_TIME_DN           DFLT_200_MS_T10MS

/* 根据项目需求选则，输入口与逻辑变量的对应关系*/
#define OPN_LINE_LOGIC_FLAG             NEGATIVE_II01_FLTR  // 输入口01，开门信号线
#define CLS_LINE_LOGIC_FLAG             NEGATIVE_II02_FLTR  // 输入口02，关门信号线
	
/* 门控参数初始值恒定义*/
#define DFLT_OPN_TIME_SET               30U         // 开门时间设定默认值3s,以100ms为单位
#define DFLT_CLS_TIME_SET               30U         // 关门时间设定默认值3s,以100ms为单位
#define DFLT_OPN_DLY_SET                0U          // 开门延时设定默认值0s
#define DFLT_CLS_DLY_SET                0U          // 关门延时设定默认值0s
#define DFLT_CLS_KEEP_FRC_TIME_SET      5U          // 防挤压堵转时间默认值500ms，以100ms为单位
#define DFLT_RECLS_DLY_PRMT_SET         3U          // 防挤压再关闭延时时间默认值300ms，以100ms为单位
#define DFLT_OPN_OBST_COUNT_SET         3U          // 开门防挤压次数设定默认值3次
#define DFLT_CLS_OBST_COUNT_SET         3U          // 关门防挤压次数设定默认值3次
#define DFLT_CLS_OBST_REOPN_DSTN_SET    100U        // 关门防挤压过程中再开门打开距离，单边100mm
#define DFLT_TOTAL_DSTN_DBL_SET         700U        // 双门开度默认值单边700mm

#endif

/*==================================================================================
     文件结束
===================================================================================*/
