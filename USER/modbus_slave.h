#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <stdint.h>


/* 网关协议寄存器地址范围 */
#define STATUS_REG_START    0x0000   // 状态寄存器起始地址
#define STATUS_REG_END      0x0027   // 状态寄存器结束地址（共40个）
#define CMD_REG_START       0x0030   // 命令寄存器起始地址
#define CMD_REG_END         0x0048   // 命令寄存器结束地址

/* 特殊命令寄存器地址 */
#define CMD_TAKEOFF         0x0038
#define CMD_LANDING         0x0039
#define CMD_CLOSE_CENTER    0x0032
#define CMD_LEAVE_CENTER    0x0033
#define CMD_LOAD_BATTERY    0x0036
#define CMD_UNLOAD_BATTERY  0x0037
#define CMD_PAUSE           0x0041
#define CMD_RESUME          0x0042
#define CMD_OPEN_UP         0x0044
#define CMD_CLOSE_DOWN      0x0045

#ifndef SADDR485
#define SADDR485    0x22
#endif
#ifndef SBAUD485
#define SBAUD485    9600
#endif

/* 接收缓冲区大小 */
#define S_RX_BUF_SIZE   256
#define S_TX_BUF_SIZE   256
#define TMR_COUNT	4		/* 软件定时器的个数 （定时器ID范围 0 - 3) */
#define SLAVE_REG_P01		0x0301
#define SLAVE_REG_P02		0x0302

static uint32_t remote_off_time = 0;        // 预定执行时间（毫秒）
static uint8_t remote_off_pending = 0;      // 是否有待执行的任务

/* 从站全局变量 */
typedef struct {
    uint8_t RxBuf[S_RX_BUF_SIZE];
    uint8_t TxBuf[S_TX_BUF_SIZE];
    volatile uint16_t RxCount;
    uint8_t RspCode;        // 错误码
} MODS_T;

/* 定时器结构体，成员变量必须是 volatile, 否则C编译器优化时可能有问题 */
typedef enum
{
	TMR_ONCE_MODE = 0,		/* 一次工作模式 */
	TMR_AUTO_MODE = 1		/* 自动定时工作模式 */
}TMR_MODE_E;

typedef struct
{
	/* 03H 06H 读写保持寄存器 */
	uint16_t P01;
	uint16_t P02;

	/* 04H 读取模拟量寄存器 */
	uint16_t A01;

	/* 01H 05H 读写单个强制线圈 */
	uint16_t D01;
	uint16_t D02;
	uint16_t D03;
	uint16_t D04;

}VAR_T;

extern MODS_T g_tModS;
extern volatile uint8_t g_mods_timeout;  // 帧超时标志
uint16_t BEBufToUint16(uint8_t *_pBuf);
uint16_t LEBufToUint16(uint8_t *_pBuf);
void bsp_InitTimer(void);
void SendWithCRC(uint8_t *buf, uint8_t len);
void SendAckErr(uint8_t err_code);
void SendAckOk(void);

void MODS_Poll(void);
void MODS_ReciveNew(uint8_t _byte);
void Handle03(void);
void Handle06(void);
void Handle10(void);
void StartFrameTimeout(void);
void StopFrameTimeout(void);
void CheckRemoteOffTask(void);

#endif