/*==================================================================================
This document is the property of Nanjing Kangni Mechanical & Electrical Co., Ltd. It may
not be reproduced or disclosed to third party without prior to authorisation
 * header file name: project_cfg.h      
 * heafer file description: define the date type and initalization used by the timer module
 * service condition: peoject.h
 * version information: Wan Lei, V0100-0000, 20180404
===================================================================================*/

/*==================================================================================
     avoid redefinition
===================================================================================*/
#ifndef __PROJECT_CFG_H
#define	__PROJECT_CFG_H


/*==================================================================================
     port definition
===================================================================================*/
//input ports

#define IPORT01             GPIOC
#define IPORT01_PIN         GPIO_Pin_8
#define IPORT02             GPIOC
#define IPORT02_PIN         GPIO_Pin_7
#define IPORT03             GPIOC
#define IPORT03_PIN         GPIO_Pin_6
#define IPORT04             GPIOB
#define IPORT04_PIN         GPIO_Pin_15
#define IPORT05             GPIOC
#define IPORT05_PIN         GPIO_Pin_11
#define IPORT06             GPIOC
#define IPORT06_PIN         GPIO_Pin_12

#define IPORT00             GPIOA
#define IPORT00_PIN         GPIO_Pin_1

#define CODE1               GPIOB     
#define CODE1_PIN           GPIO_Pin_10
#define CODE2               GPIOB     
#define CODE2_PIN           GPIO_Pin_11 
#define CODE3               GPIOB     
#define CODE3_PIN           GPIO_Pin_12 
#define CODE4               GPIOB     
#define CODE4_PIN           GPIO_Pin_3 
#define CODE5               GPIOB     
#define CODE5_PIN           GPIO_Pin_4 

#define TEST                GPIOA
#define TEST_PIN            GPIO_Pin_11

#define USART_MASTER_RE              GPIOC
#define USART_RE_MASTER_PIN          GPIO_Pin_0
#define USART_SLAVE_RE               GPIOC
#define USART_RE_SLAVE_PIN           GPIO_Pin_1

//output ports
#define OPORT01             GPIOB
#define OPORT01_PIN         GPIO_Pin_5
#define OPORT02             GPIOB
#define OPORT02_PIN         GPIO_Pin_4
#define OPORT03             GPIOB
#define OPORT03_PIN         GPIO_Pin_3
#define OPORT04             GPIOD
#define OPORT04_PIN         GPIO_Pin_2
#define OPORT05             GPIOC
#define OPORT05_PIN         GPIO_Pin_12
#define OPORT06             GPIOC
#define OPORT06_PIN         GPIO_Pin_11
#define OPORT07             GPIOB
#define OPORT07_PIN         GPIO_Pin_1

#define OPORTA             GPIOC
#define OPORTA_PIN         GPIO_Pin_14
#define OPORTB             GPIOC
#define OPORTB_PIN         GPIO_Pin_15

#define OPORTC             GPIOB
#define OPORTC_PIN         GPIO_Pin_9
#define OPORTD             GPIOC
#define OPORTD_PIN         GPIO_Pin_0

#define OPORTE             GPIOB
#define OPORTE_PIN         GPIO_Pin_8
#define OPORTF             GPIOC
#define OPORTF_PIN         GPIO_Pin_1

#define OPORTH             GPIOB
#define OPORTH_PIN         GPIO_Pin_7
#define OPORTI             GPIOC
#define OPORTI_PIN         GPIO_Pin_2


#define OPORTJ             GPIOB
#define OPORTJ_PIN         GPIO_Pin_6
#define OPORTK             GPIOC
#define OPORTK_PIN         GPIO_Pin_3

#define PWMEN               GPIOA
#define PWMEN_PIN           GPIO_Pin_15

//#define EN485               GPIOA
//#define EN485_PIN           GPIO_Pin_8

#define USART_RE               GPIOC
#define USART_RE_PIN           GPIO_Pin_1
                           
#define LED_PORT            GPIOB   
#define LED1                GPIO_Pin_0   


//adc ports
#define ADC_MTR1_RCC        RCC_APB2Periph_GPIOA
#define ADC_MTR1            GPIOA
#define ADC_MTR1_PIN        GPIO_Pin_5



//hall ports
#define HALL_A              GPIOA
#define HALL_A_PIN          GPIO_Pin_6
#define HALL_B              GPIOA
#define HALL_B_PIN          GPIO_Pin_7
#define HALL_C              GPIOB
#define HALL_C_PIN          GPIO_Pin_0


//pwm ports
#define PWMAH               GPIOA
#define PWMAH_PIN           GPIO_Pin_8
#define PWMAL               GPIOB
#define PWMAL_PIN           GPIO_Pin_13
#define PWMBH               GPIOA
#define PWMBH_PIN           GPIO_Pin_9
#define PWMBL               GPIOB
#define PWMBL_PIN           GPIO_Pin_14
#define PWMCH               GPIOA
#define PWMCH_PIN           GPIO_Pin_10
#define PWMCL               GPIOB
#define PWMCL_PIN           GPIO_Pin_15



//usart ports
#define USART_TX            GPIOB
#define USART_TX_PIN        GPIO_Pin_10
#define USART_RX            GPIOB
#define USART_RX_PIN        GPIO_Pin_11



/*==================================================================================
     time definition
===================================================================================*/
//100ms base timer
#define DFLT_6000_MS_T100MS     420     
#define DFLT_2000_MS_T100MS     20    
#define DFLT_1000_MS_T100MS     10    
#define DFLT_800_MS_T100MS      8      
#define DFLT_500_MS_T100MS      5      
#define DFLT_300_MS_T100MS      3      

//10ms base timer
#define DFLT_300_MS_T10MS      35      
#define DFLT_200_MS_T10MS      20   
#define DFLT_100_MS_T10MS      5      
#define DFLT_50_MS_T10MS        5      

//1ms base timer
#define DFLT_20_MS_T1MS      20      

//input filtering time
#define DFLT_II00_FLT_TIME_UP            DFLT_50_MS_T10MS      
#define DFLT_II00_FLT_TIME_DN            DFLT_50_MS_T10MS
#define DFLT_II01_FLT_TIME_UP            DFLT_50_MS_T10MS      
#define DFLT_II01_FLT_TIME_DN            DFLT_50_MS_T10MS
#define DFLT_II02_FLT_TIME_UP            DFLT_50_MS_T10MS      
#define DFLT_II02_FLT_TIME_DN            DFLT_50_MS_T10MS
#define DFLT_II03_FLT_TIME_UP            DFLT_50_MS_T10MS           
#define DFLT_II03_FLT_TIME_DN            DFLT_50_MS_T10MS
#define DFLT_II04_FLT_TIME_UP            DFLT_50_MS_T10MS      
#define DFLT_II04_FLT_TIME_DN            DFLT_50_MS_T10MS
#define DFLT_II05_FLT_TIME_UP            DFLT_50_MS_T10MS       
#define DFLT_II05_FLT_TIME_DN            DFLT_50_MS_T10MS
#define DFLT_II06_FLT_TIME_UP            DFLT_200_MS_T10MS       
#define DFLT_II06_FLT_TIME_DN            DFLT_200_MS_T10MS
//o
#define DFLT_COMMAND_DELAY               DFLT_300_MS_T10MS     //driver conmand switch time 


#define DFLT_OPNCLS_SWITCH_TIME          DFLT_50_MS_T10MS      //switch between opening and closing
#define DFLT_OPN_OBSTC                   DFLT_100_MS_T10MS     //think time of open obstacle
#define DFLT_CLS_OBSTC                   30//DFLT_100_MS_T10MS     //think time if close obstacle  30
#define DFLT_CLS_OBSTC_OPNDLY_TIME       DFLT_50_MS_T10MS      //open delay of close obstacle 
#define DFLT_SHOW_LED_TIME               DFLT_200_MS_T10MS     //led frequency
#define DFLT_UPDATE_HALL_TIME            DFLT_200_MS_T10MS      //hall counter reset time
#define DFLT_ALLMOVE_DOOR                50//DFLT_300_MS_T10MS     //the moving delay bewteen two doors


#define DFLT_M1_START_SHEILD_TIME       15// DFLT_800_MS_T100MS   
#define DFLT_M2_START_SHEILD_TIME        15//DFLT_800_MS_T100MS
#define DFLT_MTR_OPNCCT_TIME             DFLT_800_MS_T100MS   
#define DFLT_MTR_NOLOAD_TIME             DFLT_2000_MS_T100MS   
#define DFLT_MTR_HALLFAULT_TIME          DFLT_1000_MS_T100MS   
#define DFLT_POWERUP_TIME                900   
#define DFLT_AUTORUN_TIME                25   


#define DFLT_TIMER_CLS_OVERTIME_TIME     DFLT_6000_MS_T100MS    //over time of clsing door
#define DFLT_TIMER_OPN_OVERTIME_TIME     DFLT_6000_MS_T100MS    //over time of opening door

#define DFLT_SPEED_LOOP_TIME              DFLT_20_MS_T1MS       //speed loop fequency


#define DFLT_CLSOBSTC_WAIT               5                     
#define DFLT_OPNOBSTC_WAIT               10

//the relationship between the input port and the logical variable
#define LEFT_SWITCH_LOGIC_FLAG           NEGATIVE_II01_FLTR         
#define EMERGENCY_SWITCH_LOGIC_FLAG      NEGATIVE_II02_FLTR         
#define OPN_LINE_LOGIC_FLAG              NEGATIVE_II03_FLTR         
#define CLS_LINE_LOGIC_FLAG              NEGATIVE_II04_FLTR
#define ZERO_SPD_LINE_LOGIC_FLAG         POSITIVE_II05_FLTR

#define DFLT_CLS_OBSTC_TIMES             3

//speed parameters
#define DFLT_INIT_SPEED                  1500
#define DFLT_OPEN_SPEED                  2100
#define DFLT_CLOSE_SPEED                 2100
#define DFLT_OPEN_END_SPEED              650     //the open speed at the end
#define DELT_CLOSE_END_SPEED             900//700     //the close speed at the end
#define DFLT_MAX_SPEED                   4000
#define DFLT_MIN_SPEED                   200
#define DFLT_ACC_SPEED                   100      //rpm/ms
#define DFLT_ADD_SPEED                   DFLT_ACC_SPEED*DFLT_SPEED_LOOP_TIME

//self-learning parameters
#define DFLT_CLS_SL_TIME                DFLT_1000_MS_T100MS
#define DFLT_CLS_OPN_TIME               DFLT_1000_MS_T100MS
#define DFLT_CLS_OBSTC_FORCE             50
#define DFLT_CLS_OBSTC_CRRNT_LIMIT       500
#define DFLT_OPN_OBSTC_FORCE             50
#define DFLT_OPN_OBSTC_CRRNT_LIMIT       600
#define DFLT_CLS_LOCK_FORCE              50
#define DFLT_CLS_LOCK_CRRNT_LIMIT        400
#define DFLT_CLS_LOCK_SPEED              50
#define DFLT_OPN_LOCK_FORCE              200
#define DFLT_OPN_LOCK_CRRNT_LIMIT        600
#define DFLT_OPN_LOCK_SPEED              500
// #define DFLT_CLS_OBSTC_CRRNT             1400
// #define DFLT_OPN_OBSTC_CRRNT             1400

#define DFLT_MIN_OPN_DISTANCE            1000
#define DFLT_MAX_OPN_DISTANCE            3000
#define DFLT_LEFT_OPN_DISTANCE           2100 //1317
#define DFLT_RIGHT_OPN_DISTANCE          1200

#define DFLT_LEFT_OPN_OBSTC_RNG          700
#define DFLT_RIGHT_OPN_OBSTC_RNG         700
#define DFLT_OPNED_OBSTC_RNG             50


/*==================================================================================
     diagnose parameter definition
===================================================================================*/
#define DFLT_FAULT_BIT_00         uFM1OpnCctFault
#define DFLT_FAULT_BIT_01         uFM2OpnCctFault
#define DFLT_FAULT_BIT_02         uFM1NoLoadFault
#define DFLT_FAULT_BIT_03         uFM2NoLoadFault
#define DFLT_FAULT_BIT_04         uFClsObstcFault
#define DFLT_FAULT_BIT_05         uFLSwitchFault
#define DFLT_FAULT_BIT_06         uFRSwitchFault
#define DFLT_FAULT_BIT_07         uFLHallFault
#define DFLT_FAULT_BIT_08         uFRHallFault
#define DFLT_FAULT_BIT_09         uFPowerUpEvent
// #define DFLT_FAULT_BIT_10         
// #define DFLT_FAULT_BIT_11         
// #define DFLT_FAULT_BIT_12         

#define MOTOR1_FR()   GPIO_ResetBits(OPORTB, OPORTB_PIN);	GPIO_SetBits(OPORTA, OPORTA_PIN);	
#define MOTOR1_RR()   GPIO_ResetBits(OPORTA, OPORTA_PIN);	GPIO_SetBits(OPORTB, OPORTB_PIN);	
#define MOTOR1_STOP() GPIO_ResetBits(OPORTB, OPORTB_PIN);	GPIO_ResetBits(OPORTA, OPORTA_PIN);

#define MOTOR2_FR()   GPIO_ResetBits(OPORTD, OPORTD_PIN);	GPIO_SetBits(OPORTC, OPORTC_PIN);	
#define MOTOR2_RR()   GPIO_ResetBits(OPORTC, OPORTC_PIN);	GPIO_SetBits(OPORTD, OPORTD_PIN);	
#define MOTOR2_STOP() GPIO_ResetBits(OPORTC, OPORTC_PIN);	GPIO_ResetBits(OPORTD, OPORTD_PIN);

#define MOTOR_CLAMP_FR()   GPIO_ResetBits(OPORTF, OPORTF_PIN);	GPIO_SetBits(OPORTE, OPORTE_PIN);
#define MOTOR_CLAMP_RR()   GPIO_ResetBits(OPORTE, OPORTE_PIN);	GPIO_SetBits(OPORTF, OPORTF_PIN);
#define MOTOR_CLAMP_STOP() GPIO_ResetBits(OPORTE, OPORTE_PIN);	GPIO_ResetBits(OPORTF, OPORTF_PIN);

#define MOTOR4_FR()   GPIO_ResetBits(OPORTI, OPORTI_PIN);	GPIO_SetBits(OPORTH, OPORTH_PIN);	
#define MOTOR4_RR()   GPIO_ResetBits(OPORTH, OPORTH_PIN);	GPIO_SetBits(OPORTI, OPORTI_PIN);	
#define MOTOR4_STOP() GPIO_ResetBits(OPORTH, OPORTH_PIN);	GPIO_ResetBits(OPORTI, OPORTI_PIN);

#define MOTOR5_FR()   GPIO_ResetBits(OPORTK, OPORTK_PIN);	GPIO_SetBits(OPORTJ, OPORTJ_PIN);	
#define MOTOR5_RR()   GPIO_ResetBits(OPORTJ, OPORTJ_PIN);	GPIO_SetBits(OPORTK, OPORTK_PIN);	
#define MOTOR5_STOP() GPIO_ResetBits(OPORTJ, OPORTJ_PIN);	GPIO_ResetBits(OPORTK, OPORTK_PIN);




#endif

/*==================================================================================
     the end of file
===================================================================================*/
