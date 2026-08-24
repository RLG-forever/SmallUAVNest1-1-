/*==================================================================================
This document is the property of Nanjing Kangni Mechanical & Electrical Co., Ltd..
It may not be reproduced or disclosed to third party without prior to authorisation
 * header file name: input.h      
 * heafer file description: define the date type and initalization used by the io module 
 * service condition: use in input port control module
 * version information: Wan Lei, V0100-0000, 20180104
===================================================================================*/

/*==================================================================================
     avoid redefinition
===================================================================================*/
#ifndef __INPUT_H
#define	__INPUT_H

/*==================================================================================
     constant definition
===================================================================================*/
//
#define POSITIVE_II00_FLTR    (ipc->uFInputPort00Fltr)     //input port 00 positive logic
#define NEGATIVE_II00_FLTR    (!(ipc->uFInputPort00Fltr))     //input port 00 negative logic
#define POSITIVE_II01_FLTR    (ipc->uFInputPort01Fltr)     //input port 01 positive logic
#define NEGATIVE_II01_FLTR    (!(ipc->uFInputPort01Fltr))     //input port 01 negative logic
#define POSITIVE_II02_FLTR    (ipc->uFInputPort02Fltr)     //input port 02 positive logic
#define NEGATIVE_II02_FLTR    (!(ipc->uFInputPort02Fltr))     //input port 02 negative logic
#define POSITIVE_II03_FLTR    (ipc->uFInputPort03Fltr)     //input port 03 positive logic
#define NEGATIVE_II03_FLTR    (!(ipc->uFInputPort03Fltr))    //input port 03 negative logic
#define POSITIVE_II04_FLTR    (ipc->uFInputPort04Fltr)     //input port 04 positive logic
#define NEGATIVE_II04_FLTR    (!(ipc->uFInputPort04Fltr))    //input port 04 negative logic
#define POSITIVE_II05_FLTR    (ipc->uFInputPort05Fltr)     //input port 05 positive logic
#define NEGATIVE_II05_FLTR    (!(ipc->uFInputPort05Fltr))     //input port 05 negative logic
#define POSITIVE_II06_FLTR    (ipc->uFInputPort06Fltr)     //input port 06 positive logic
#define NEGATIVE_II06_FLTR    (!(ipc->uFInputPort06Fltr))     //input port 06 negative logic



/*==================================================================================
     variable definition
===================================================================================*/
typedef struct
{
    U8 uIInputPort00;
	U8 uIInputPort01;
	U8 uIInputPort02;
	U8 uIInputPort03;
	U8 uIInputPort04;
	U8 uIInputPort05;
	U8 uIInputPort06;
	
	U8 uFOutputFdbck00;
	U8 uFOutputFdbck01;
	U8 uFOutputFdbck02;
	U8 uFOutputFdbck03;
	
    U8 uITest;

	U8 uFInputPort00Fltr;
	U8 uFInputPort01Fltr;
	U8 uFInputPort02Fltr;
	U8 uFInputPort03Fltr;
	U8 uFInputPort04Fltr;
	U8 uFInputPort05Fltr;
	U8 uFInputPort06Fltr;
	
	U8 uMIPreInputPort00;
	U8 uMIPreInputPort01;
	U8 uMIPreInputPort02;
	U8 uMIPreInputPort03;
	U8 uMIPreInputPort04;
	U8 uMIPreInputPort05;
	U8 uMIPreInputPort06;
}IPCGEN;

/*==================================================================================
     variable initialization definition
===================================================================================*/
#define IPC_DEFAULTS  {0,0,0,0,0,0,0, 0,0,0,0, 0, 0,0,0,0,0,0,0, 0,0,0,0,0,0,0}

/*==================================================================================
     declare definition
===================================================================================*/
void ReadInputSignal(IPCGEN *ipc);
void CnfgrInputPort(void);
void OutputPortInit(void);
#endif
/*==================================================================================
     the end of file
===================================================================================*/

