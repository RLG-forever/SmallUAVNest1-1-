#include "project.h"
#include <string.h>

// 外部已有的Modbus主站写函数
extern uint8_t Modbus_06_WriteSingleReg(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_data);
extern uint16_t status_cache[STATUS_REG_COUNT]; 
// 从站参数
#define SLAVE_ADDR      0x22        // 本机地址
#define BUF_SIZE        256          // 接收缓冲区大小
#define USE_TIM2

// 引用 RS485 模块中定义的全局接收缓冲区及标志
extern uint8_t RS485_RX_BUFF[];        // RS485 接收缓冲区（在 rs485.c 或 main.c 中定义）
extern volatile u16 RS485_RX_CNT; // 当前接收字节数
extern volatile u16 RS485_FrameFlag; // 帧结束标志（由 TIM3 中断置位）
volatile uint8_t g_mods_timeout = 0;
VAR_T g_tVar;
MODS_T g_tModS;

///* 定时器结构体，成员变量必须是 volatile, 否则C编译器优化时可能有问题 */
//typedef struct
//{
//	volatile uint8_t Mode;		/* 计数器模式，1次性 */
//	volatile uint8_t Flag;		/* 定时到达标志  */
//	volatile uint32_t Count;	/* 计数器 */
//	volatile uint32_t PreLoad;	/* 计数器预装值 */
//}SOFT_TMR;
static SOFT_TMR s_tTmr[TMR_COUNT];
static uint8_t MODS_WriteRegValue(uint16_t reg_addr, uint16_t reg_value);

// 帧超时时间（毫秒），用于判断一帧结束
#define FRAME_TIMEOUT_MS   4

// 命令映射表：网关写入的寄存器地址 -> 实际电机地址和寄存器
typedef struct {
    uint16_t gateway_reg;   // 网关写入的寄存器地址（0x30~0x48）
    uint8_t motor_addr;     // 目标电机从站地址
    uint16_t motor_reg;     // 目标电机寄存器地址
} CommandMap;

// 根据实际控制协议填写映射
static const CommandMap cmd_map[] = {
    {0x30, 0x11, 0x01},   // 打开舱门
    {0x31, 0x11, 0x02},   // 关闭舱门
    {0x32, 0x15, 0x01},   // 居中杆居中
    {0x33, 0x15, 0x02},   // 居中杆释放
    {0x34, 0x12, 0x05},   // 升降上升
    {0x35, 0x12, 0x06},   // 升降下降
		{0x38, 0x12, 0x06},   // 一键起飞（换新电池、升降、开舱门、起飞后关舱门）
		{0x40, 0x12, 0x06},   // 飞机降落完成（开舱门、升降、取下旧电池）
		{0x45, 0x13, 0x06},   // 飞机降落完成（开舱门、升降、取下旧电池）
		{0x41, 0x12, 0x06},   // 升降下降
};
#define CMD_MAP_COUNT (sizeof(cmd_map) / sizeof(cmd_map[0]))

// 等待主站总线空闲，超时返回0（失败），成功返回1
static uint8_t WaitMasterIdle(uint32_t timeout_ms)
{
    uint32_t start = GetTick();
    while (MasterPolling_IsBusy()) {
        if (GetTick() - start >= timeout_ms) {
            return 0;   // 超时
        }
        // 短暂延时避免死循环
        for (volatile int i = 0; i < 1000; i++);
    }
    return 1;
}


/**
 * @brief 生成并发送响应帧
 * @param slave_addr  从站地址（响应中应返回相同）
 * @param func_code   功能码（0x03 或 0x06）
 * @param data        数据域指针（对于03，是寄存器数据；对于06，通常不需要额外数据）
 * @param data_len    数据长度（字节数）
 */
static void SendResponse(uint8_t slave_addr, uint8_t func_code, uint8_t *data, uint16_t data_len)
{
    uint8_t resp_buff[256];
    uint16_t len = 0;
		printf("SendResponse: addr=0x%02X, func=0x%02X, data_len=%d\r\n", slave_addr, func_code, data_len);
    resp_buff[len++] = slave_addr;
    resp_buff[len++] = func_code;

    // 03 响应需包含字节数；06 响应直接返回请求的寄存器地址+数据（共4字节）
    if (func_code == 0x03) {
        resp_buff[len++] = data_len;   // 字节数
        memcpy(&resp_buff[len], data, data_len);
        len += data_len;
    } else if (func_code == 0x06) {
        // 06 响应数据域：寄存器地址(2) + 写入值(2) = 4字节
        memcpy(&resp_buff[len], data, 4);
        len += 4;
    }

    // 计算 CRC
    uint16_t crc = Modbus_CRC16(resp_buff, len);
    resp_buff[len++] = crc & 0xFF;
    resp_buff[len++] = (crc >> 8) & 0xFF;

    // 发送
    RS485_SlaveSendData(resp_buff, len);
}



/* 发送带 CRC 的数据 */
void SendWithCRC(uint8_t *buf, uint8_t len)
{
    uint16_t crc = Modbus_CRC16(buf, len);
    uint8_t tx_buf[256];
    memcpy(tx_buf, buf, len);
    tx_buf[len] = crc & 0xFF;
    tx_buf[len+1] = (crc >> 8) & 0xFF;
    RS485_SlaveSendData(tx_buf, len + 2);
}

/* 发送错误应答 */
void SendAckErr(uint8_t err_code)
{
    uint8_t buf[3];
    buf[0] = g_tModS.RxBuf[0];
    buf[1] = g_tModS.RxBuf[1] | 0x80;
    buf[2] = err_code;
    SendWithCRC(buf, 3);
}

/* 发送正确应答（06/10 功能码用） */
void SendAckOk(void)
{
    SendWithCRC(g_tModS.RxBuf, 6);   // 前6字节原样返回
}


/* 帧接收完成，开始解析 */
void MODS_Poll(void)
{
    if (!g_mods_timeout) return;
    g_mods_timeout = 0;

    if (g_tModS.RxCount < 4) {
        g_tModS.RxCount = 0;
        return;
    }

    // CRC 校验
    uint16_t crc = Modbus_CRC16(g_tModS.RxBuf, g_tModS.RxCount);
    if (crc != 0) {
        g_tModS.RxCount = 0;
        return;
    }

    // 地址检查
    if (g_tModS.RxBuf[0] != SADDR485) {
        g_tModS.RxCount = 0;
        return;
    }

    uint8_t func = g_tModS.RxBuf[1];   // 关键：从正确的缓冲区读取功能码
    switch (func) {
        case 0x03: Handle03(); break;
        case 0x06: Handle06(); break;
        case 0x10: Handle10(); break;
        default:   SendAckErr(0x01); break;
    }

    g_tModS.RxCount = 0;   // 清空缓冲区，准备下一帧
}

/* 接收一个字节（在串口中断中调用） */
void MODS_ReciveNew(uint8_t _byte)
{
     // 停止并重启定时器（喂狗）
    TIM_Cmd(TIM3, DISABLE);
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_SetCounter(TIM3, 0);

    if (g_tModS.RxCount < sizeof(g_tModS.RxBuf)) {
        g_tModS.RxBuf[g_tModS.RxCount++] = _byte;
    } else {
        g_tModS.RxCount = 0;   // 溢出复位
    }
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		TIM_Cmd(TIM3, ENABLE);
}

/* 读取保持寄存器（网关读取机巢状态） */
//void Handle03(void)
//{
////		printf("Handle03 entered\r\n");  
//		uint16_t start_reg = BEBufToUint16(&g_tModS.RxBuf[2]);
//    uint16_t reg_count = BEBufToUint16(&g_tModS.RxBuf[4]);
//    if (start_reg + reg_count > 40) {
//        SendAckErr(0x02);
//        return;
//    }

//		// 准备响应缓冲区
//    uint8_t resp[256];
//    StatusRegs_GetBatch(start_reg, reg_count, resp);   // 批量获取，快速
//    uint8_t tx_buf[256];
//    tx_buf[0] = SADDR485;
//    tx_buf[1] = 0x03;
//    tx_buf[2] = reg_count * 2;
//    memcpy(&tx_buf[3], resp, reg_count * 2);
//    SendWithCRC(tx_buf, 3 + reg_count * 2);
//	
//}

//测试代码
void Handle03(void)
{
    uint16_t start_reg = BEBufToUint16(&g_tModS.RxBuf[2]);
    uint16_t reg_count = BEBufToUint16(&g_tModS.RxBuf[4]);
    if (start_reg + reg_count > 40) {
        SendAckErr(0x02);
        return;
    }

    // 准备响应缓冲区
    uint8_t resp[256];

    // 逐个寄存器填充，支持特定地址返回固定值
    for (uint16_t i = 0; i < reg_count; i++) {
        uint16_t reg_addr = start_reg + i;
        uint16_t val;

        // ---------- 临时测试：对舱门状态(0x11)和居中杆状态(0x15)强制返回2 ----------
        if (reg_addr == REG_DOOR_STATE || reg_addr == REG_CENTER_ROD_STATE) {
            val = 2;
        } else {
            // 从状态缓存读取
            ENTER_CRITICAL();
            val = status_cache[reg_addr];
            EXIT_CRITICAL();
        }

        resp[i*2]   = (val >> 8) & 0xFF;
        resp[i*2+1] = val & 0xFF;
    }

    // 组装响应帧
    uint8_t tx_buf[256];
    tx_buf[0] = SADDR485;
    tx_buf[1] = 0x03;
    tx_buf[2] = reg_count * 2;
    memcpy(&tx_buf[3], resp, reg_count * 2);
    SendWithCRC(tx_buf, 3 + reg_count * 2);
}


static uint8_t wait_openfly = 0;
/* 写单个保持寄存器（网关下发命令） */
void Handle06(void)
{
    uint16_t reg_addr = BEBufToUint16(&g_tModS.RxBuf[2]);
    uint16_t reg_data = BEBufToUint16(&g_tModS.RxBuf[4]);
    uint8_t ret = 1;   // 默认失败（1表示失败）

    // 去重：记录上次执行的命令，若相同则忽略
    static uint16_t last_reg_addr = 0xFFFF;
    static uint16_t last_reg_data = 0xFFFF;
		static uint32_t last_reg_time = 0;      // 上次执行时间（毫秒）
    uint8_t executed = 0;   // 标记当前命令是否实际执行了操作

//    // 若与上次命令完全相同，则忽略执行，直接返回成功（此处成功响应，不计入ret）
//    if (reg_addr == last_reg_addr && reg_data == last_reg_data) {
//        printf("忽略重复命令: addr=0x%04X, data=0x%04X\n", reg_addr, reg_data);
//        SendAckOk();
//        return;
//    }
	   // 检查是否与上次命令完全相同，且时间差 < 5秒
    if (reg_addr == last_reg_addr && reg_data == last_reg_data) 
		{
        uint32_t now = GetTick();
        if (now - last_reg_time < 5000) 
				{
            printf("忽略重复命令(5秒内): addr=0x%04X, data=0x%04X\n", reg_addr, reg_data);
            SendAckOk();
            return;
        }
        // 超过5秒，视为新命令，继续执行（不返回）
    }

    // 特殊命令处理
    switch (reg_addr) {
        case 0x30: Sequence_Start(SEQ_ID_OPENDR); executed = 1; ret = 0; break;
        case 0x31: Sequence_Start(SEQ_ID_CLOSEDR); executed = 1; ret = 0; break;
        case 0x32: Sequence_Start(SEQ_ID_CLOSECENTER); executed = 1; ret = 0; break;
        case 0x33: Sequence_Start(SEQ_ID_LEAVECENTER); executed = 1; ret = 0; break;
        case 0x36: Sequence_Start(SEQ_ID_LOADBATTERY); executed = 1; ret = 0; break;
        case 0x37: Sequence_Start(SEQ_ID_DOWNBATTERY); executed = 1; ret = 0; break;
//        case 0x38: Sequence_Start(SEQ_ID_TAKEOFF); executed = 1; ret = 0; break;
//        case 0x40: Sequence_Start(SEQ_ID_LANDING); executed = 1; ret = 0; break;
        case 0x41: Sequence_Pause(); executed = 1; ret = 0; break;
        case 0x42: Sequence_Resume(); executed = 1; ret = 0; break;
        case 0x43: Sequence_Cancel(); executed = 1; ret = 0; break;
        case 0x39: Sequence_Start(SEQ_ID_OPENDR1); executed = 1; ret = 0; break;
        case 0x44: Sequence_Start(SEQ_ID_OPENDR1); executed = 1; ret = 0; break;
        case 0x45: Sequence_Start(SEQ_ID_CLOSEDOWN); executed = 1; ret = 0; break;
        case 0x50: Sequence_Start(SEQ_ID_OPENFLY); executed = 1; ret = 0; break;
        case 0x51: Sequence_Start(SEQ_ID_CLOSEFLY); executed = 1; ret = 0; break;
			
				case 0x38:
				{
						uint16_t uav_status = StatusRegs_Get(REG_RESERVED4);
						if (uav_status == 0) {
								// 遥控器未开机，先发送开机指令
								if (WaitMasterIdle(100)) {
										MasterBusy_Acquire();
										ret = Modbus_06_WriteSingleReg(0x14, 0x0001, 0x0001);
										MasterBusy_Release();
										// 无论开机指令成功与否，都启动起飞序列（保持原有行为）
										Sequence_Start(SEQ_ID_TAKEOFF);
										executed = 1;
										ret = 0;
								} else {
										ret = 1;
										printf("遥控器开机指令失败：总线忙\n");
										executed = 0;
								}
						} else if (uav_status == 1) {
								// 值为1：不启动起飞序列，设置等待开机标志
								wait_openfly = 1;
								printf("设置等待无人机开机标志，不执行起飞序列\n");
								// 注意：此处不再调用 Sequence_Start(SEQ_ID_TAKEOFF)
								ret = 0;
								executed = 1;
						} else {
								// 值为2或3，直接启动起飞序列
								Sequence_Start(SEQ_ID_TAKEOFF);
								ret = 0;
								executed = 1;
						}
						break;
				}
			
				case 0x40:
				{
						// 先判断序列是否空闲，确保能启动
						if (!Sequence_IsBusy()) {
								Sequence_Start(SEQ_ID_LANDING);
								// 启动成功（此时busy应为1），设置半小时后遥控器关机
								remote_off_pending = 1;
								remote_off_time = GetTick() + 30UL * 60UL * 1000UL;
								printf("降落序列已启动，半小时后将自动发送遥控器关机指令\n");
						} else {
								printf("降落序列启动失败，已有序列在执行中\n");
						}
						executed = 1;
						ret = 0;
						break;
				}

        // 控制bms充电空开开机（0x48）
        case 0x48:
            if (WaitMasterIdle(100)) {
                MasterBusy_Acquire();
                ret = Modbus_06_WriteSingleReg(MOTOR15_SLAVE_ADDR, 0x00, 0x01);
                MasterBusy_Release();
                if (ret == 0) {
                    printf("电机0x13 开机指令发送成功\n");
										executed = 1;
                } else {
                    printf("电机0x13 开机指令发送失败，错误码: %d\n", ret);
										executed = 0;
                }
                
                // 注意：ret 已经是 Modbus_06_WriteSingleReg 的返回值（0成功，非0失败），无需再改
            } else {
                ret = 1;   // 总线忙，视为失败
                printf("电机0x13 开机指令失败：总线忙\n");
                // 总线忙未执行，不置 executed
            }
            break;

        // 控制bms充电空开关机（0x49）
        case 0x49:
            if (WaitMasterIdle(100)) {
                MasterBusy_Acquire();
                ret = Modbus_06_WriteSingleReg(MOTOR15_SLAVE_ADDR, 0x00, 0x00);
                MasterBusy_Release();
                if (ret == 0) {
                    printf("电机0x13 关机指令发送成功\n");
										executed = 1;
                } else {
                    printf("电机0x13 关机指令发送失败，错误码: %d\n", ret);
										executed = 0;	
                }
                
            } else {
                ret = 1;
                printf("电机0x13 关机指令失败：总线忙\n");
            }
            break;

        // 新增：0x52 和 0x53 控制舵机0x14的寄存器0x01
        case 0x52:   // 写1
            if (WaitMasterIdle(100)) {
                MasterBusy_Acquire();
                ret = Modbus_06_WriteSingleReg(0x14, 0x0001, 0x0001);
                MasterBusy_Release();
                if (ret == 0) {
                    printf("电机0x14 寄存器0x01 写1成功\n");
										executed = 1;
                } else {
                    printf("电机0x14 寄存器0x01 写1失败，错误码: %d\n", ret);
										executed = 0;	
                }
                
            } else {
                ret = 1;
                printf("电机0x14 写1失败：总线忙\n");
            }
            break;

        case 0x53:   // 写2
            if (WaitMasterIdle(100)) {
                MasterBusy_Acquire();
                ret = Modbus_06_WriteSingleReg(0x14, 0x0001, 0x0002);
                MasterBusy_Release();
                if (ret == 0) {
                    printf("电机0x14 寄存器0x01 写2成功\n");
										executed = 1;
                } else {
                    printf("电机0x14 寄存器0x01 写2失败，错误码: %d\n", ret);
										executed = 0;
                }
                
            } else {
                ret = 1;
                printf("电机0x14 写2失败：总线忙\n");
            }
            break;

        // 设置空调制冷停止温度（0x54）
        case 0x54:
            if (WaitMasterIdle(100)) {
                MasterBusy_Acquire();
                ret = Modbus_06_WriteSingleReg(MOTOR13_SLAVE_ADDR, 0x00, reg_data);
                MasterBusy_Release();
                if (ret == 0) {
                    printf("设置空调制冷停止温度成功: %.1f℃\n", reg_data / 10.0f);
										executed = 1;
                } else {
                    printf("设置空调制冷停止温度失败，错误码: %d\n", ret);
										executed = 0;
                }
                
            } else {
                ret = 1;
                printf("设置空调制冷停止温度失败：总线忙\n");
            }
            break;

        // 设置空调制热停止温度（0x55）
        case 0x55:
            if (WaitMasterIdle(100)) {
                MasterBusy_Acquire();
                ret = Modbus_06_WriteSingleReg(MOTOR13_SLAVE_ADDR, 0x02, reg_data);
                MasterBusy_Release();
                if (ret == 0) {
                    printf("设置空调制热停止温度成功: %.1f℃\n", reg_data / 10.0f);
										executed = 1;
                } else {
                    printf("设置空调制热停止温度失败，错误码: %d\n", ret);
										executed = 0;
                }
                
            } else {
                ret = 1;
                printf("设置空调制热停止温度失败：总线忙\n");
            }
            break;

        // 地址 0x60 处理（无人机状态监测，连续两次0则开机）
				case 0x60:
				{
						// 保存当前值到状态寄存器
						StatusRegs_Update(REG_RESERVED4, reg_data);
						printf("0x60更新寄存器为: %d\n", reg_data);

						// 如果值为2，则清除等待开机标志，并禁止后续1触发开机
						if (reg_data == 2) {
								wait_openfly = 0;
								printf("接收到0x60=2，清除等待开机标志，后续将不再执行无人机开机序列\n");
								ret = 0;
								executed = 1;
						}
						// 如果值为1且等待开机标志置位，则执行无人机开机序列
						else if (reg_data == 1 && wait_openfly) {
								if (!Sequence_IsBusy()) {
										printf("检测到0x60=1且等待开机标志，执行无人机开机序列\n");
										Sequence_Start(SEQ_ID_TAKEOFF);
										wait_openfly = 0;   // 清除标志，避免重复执行
										ret = 0;
										executed = 1;
								} else {
										// 序列忙，不清除标志，等待下次机会
										printf("序列忙，无人机开机序列稍后执行\n");
										ret = 0;
										executed = 1;   // 仍然返回成功，但实际未执行
								}
						} else {
								// 其他情况（包括值为0,3,或值为1但wait_openfly为0），仅返回成功
								ret = 0;
								executed = 1;
						}
						break;
				}

//        // 地址 0x61 处理（无人机状态监测，连续两次0则启动开机序列）
//        case 0x61:
//        {
//            static uint8_t last_zero = 0;
//            static uint8_t zero_count = 0;
//            if (reg_data == 0) {
//                if (last_zero == 1) {
//                    zero_count++;
//                } else {
//                    zero_count = 1;
//                }
//                if (zero_count >= 2) {
////                    printf("检测到连续两次无人机未开机（0x61=0），执行开机序列\n");
//                    if (WaitMasterIdle(100)) {
////                        MasterBusy_Acquire();
////                        Sequence_Start(SEQ_ID_OPENFLY);
////                        MasterBusy_Release();
//                        zero_count = 0;
//                        ret = 0;
//                        executed = 1;
//                    } else {
//                        ret = 1;
//                        printf("无人机开机序列失败：总线忙\n");
//                    }
//                }
//                last_zero = 1;
//            } else {
//                zero_count = 0;
//                last_zero = 0;
//            }
//						// 强制设置成功响应  测试用
//						    ret = 0;
//    executed = 1;
//            break;
//        }

        default:
            // 查找映射表
            ret = 1;   // 默认未找到
            for (uint8_t i = 0; i < CMD_MAP_COUNT; i++) {
                if (cmd_map[i].gateway_reg == reg_addr) {
                    if (WaitMasterIdle(100)) {
                        MasterBusy_Acquire();
                        ret = Modbus_06_WriteSingleReg(cmd_map[i].motor_addr, cmd_map[i].motor_reg, reg_data);
                        MasterBusy_Release();
                        executed = 1;
                        // ret 已经是 Modbus_06_WriteSingleReg 的返回值（0成功，非0失败）
                    } else {
                        ret = 1; // 总线忙，未执行
                        printf("映射命令失败：总线忙\n");
                    }
                    break;
                }
            }
            break;
    }

    // 若实际执行了操作（executed == 1），则更新上次命令记录，用于下次去重
    if (executed) {
        last_reg_addr = reg_addr;
        last_reg_data = reg_data;
    }

    // 响应网关：ret == 0 表示成功，否则失败
    if (ret == 0) {
        SendAckOk();
    } else {
        SendAckErr(0x02);   // 命令未找到或执行失败
    }
}

/* 写单个保持寄存器（网关下发命令） */
//void Handle06(void)
//{
//    uint16_t reg_addr = BEBufToUint16(&g_tModS.RxBuf[2]);
//    uint16_t reg_data = BEBufToUint16(&g_tModS.RxBuf[4]);
//    uint8_t ret = 0;   // 默认成功

//    // 去重：记录上次执行的命令，若相同则忽略
//    static uint16_t last_reg_addr = 0xFFFF;
//    static uint16_t last_reg_data = 0xFFFF;
//    uint8_t executed = 0;   // 标记当前命令是否实际执行了操作

//    // 若与上次命令完全相同，则忽略执行，直接返回成功
//    if (reg_addr == last_reg_addr && reg_data == last_reg_data) {
//        printf("忽略重复命令: addr=0x%04X, data=0x%04X\n", reg_addr, reg_data);
//        SendAckOk();
//        return;
//    }

//    // 特殊命令处理
//    switch (reg_addr) {
//        case 0x30: Sequence_Start(SEQ_ID_OPENDR); executed = 1; break;
//        case 0x31: Sequence_Start(SEQ_ID_CLOSEDR); executed = 1; break;
//        case 0x32: Sequence_Start(SEQ_ID_CLOSECENTER); executed = 1; break;
//        case 0x33: Sequence_Start(SEQ_ID_LEAVECENTER); executed = 1; break;
//        case 0x36: Sequence_Start(SEQ_ID_LOADBATTERY); executed = 1; break;
//        case 0x37: Sequence_Start(SEQ_ID_DOWNBATTERY); executed = 1; break;
//        case 0x38: Sequence_Start(SEQ_ID_TAKEOFF); executed = 1; break;
//        case 0x40: Sequence_Start(SEQ_ID_LANDING); executed = 1; break;
//        case 0x41: Sequence_Pause(); executed = 1; break;
//        case 0x42: Sequence_Resume(); executed = 1; break;
//        case 0x43: Sequence_Cancel(); executed = 1; break;
//        case 0x39: Sequence_Start(SEQ_ID_OPENDR); executed = 1; break;
//        case 0x44: Sequence_Start(SEQ_ID_OPENDR); executed = 1; break;
//        case 0x45: Sequence_Start(SEQ_ID_CLOSEDOWN); executed = 1; break;
//        case 0x50: Sequence_Start(SEQ_ID_OPENFLY); executed = 1; break;
//        case 0x51: Sequence_Start(SEQ_ID_CLOSEFLY); executed = 1; break;

//        // 控制bms充电空开开机（0x48）
//        case 0x48:
//            if (WaitMasterIdle(100)) {
//                MasterBusy_Acquire();
//                ret = Modbus_06_WriteSingleReg(MOTOR15_SLAVE_ADDR, 0x00, 0x01);
//                MasterBusy_Release();
//                if (ret == 0) {
//                    printf("电机0x13 开机指令发送成功\n");
//                } else {
//                    printf("电机0x13 开机指令发送失败，错误码: %d\n", ret);
//                }
//                executed = 1;   // 执行了写操作（无论结果）
//            } else {
//                ret = 1;
//                printf("电机0x13 开机指令失败：总线忙\n");
//                // 总线忙未执行，不置 executed
//            }
//            break;

//        // 控制bms充电空开关机（0x49）
//        case 0x49:
//            if (WaitMasterIdle(100)) {
//                MasterBusy_Acquire();
//                ret = Modbus_06_WriteSingleReg(MOTOR15_SLAVE_ADDR, 0x00, 0x00);
//                MasterBusy_Release();
//                if (ret == 0) {
//                    printf("电机0x13 关机指令发送成功\n");
//                } else {
//                    printf("电机0x13 关机指令发送失败，错误码: %d\n", ret);
//                }
//                executed = 1;
//            } else {
//                ret = 1;
//                printf("电机0x13 关机指令失败：总线忙\n");
//            }
//            break;
//				
//				// 新增：0x52 和 0x53 控制舵机0x14的寄存器0x01
//        case 0x52:   // 写1
//            if (WaitMasterIdle(100)) {
//                MasterBusy_Acquire();
//                ret = Modbus_06_WriteSingleReg(0x14, 0x0001, 0x0001);
//                MasterBusy_Release();
//                if (ret == 0) {
//                    printf("电机0x14 寄存器0x01 写1成功\n");
//                } else {
//                    printf("电机0x14 寄存器0x01 写1失败，错误码: %d\n", ret);
//                }
//                executed = 1;
//            } else {
//                ret = 1;
//                printf("电机0x14 写1失败：总线忙\n");
//            }
//            break;

//        case 0x53:   // 写2
//            if (WaitMasterIdle(100)) {
//                MasterBusy_Acquire();
//                ret = Modbus_06_WriteSingleReg(0x14, 0x0001, 0x0002);
//                MasterBusy_Release();
//                if (ret == 0) {
//                    printf("电机0x14 寄存器0x01 写2成功\n");
//                } else {
//                    printf("电机0x14 寄存器0x01 写2失败，错误码: %d\n", ret);
//                }
//                executed = 1;
//            } else {
//                ret = 1;
//                printf("电机0x14 写2失败：总线忙\n");
//            }
//            break;

//        // 设置空调制冷停止温度（0x54）
//        case 0x54:
//            if (WaitMasterIdle(100)) {
//                MasterBusy_Acquire();
//                ret = Modbus_06_WriteSingleReg(MOTOR13_SLAVE_ADDR, 0x00, reg_data);
//                MasterBusy_Release();
//                if (ret == 0) {
//                    printf("设置空调制冷停止温度成功: %.1f℃\n", reg_data / 10.0f);
//                } else {
//                    printf("设置空调制冷停止温度失败，错误码: %d\n", ret);
//                }
//                executed = 1;
//            } else {
//                ret = 1;
//                printf("设置空调制冷停止温度失败：总线忙\n");
//            }
//            break;

//        // 设置空调制热停止温度（0x55）
//        case 0x55:
//            if (WaitMasterIdle(100)) {
//                MasterBusy_Acquire();
//                ret = Modbus_06_WriteSingleReg(MOTOR13_SLAVE_ADDR, 0x02, reg_data);
//                MasterBusy_Release();
//                if (ret == 0) {
//                    printf("设置空调制热停止温度成功: %.1f℃\n", reg_data / 10.0f);
//                } else {
//                    printf("设置空调制热停止温度失败，错误码: %d\n", ret);
//                }
//                executed = 1;
//            } else {
//                ret = 1;
//                printf("设置空调制热停止温度失败：总线忙\n");
//            }
//            break;

//        // 地址 0x60 处理（无人机状态监测，连续两次0则开机）
//        case 0x60:
//        {
//            static uint8_t last_zero = 0;
//            static uint8_t zero_count = 0;
//            if (reg_data == 0) {
//                if (last_zero == 1) {
//                    zero_count++;
//                } else {
//                    zero_count = 1;
//                }
//                if (zero_count >= 2) {
//                    printf("检测到连续两次无人机未开机（0x60=0），执行开机指令\n");
//                    if (WaitMasterIdle(100)) {
////                        MasterBusy_Acquire();
////                        uint8_t result = Modbus_06_WriteSingleReg(MOTOR14_SLAVE_ADDR, 0x0001, 0x0000);
////                        MasterBusy_Release();
//                        zero_count = 0;
////                        executed = 1;   // 执行了写操作
//                    } else {
//                        printf("无人机开机指令失败：总线忙\n");
//                        // 总线忙未执行，不置 executed
//                    }
//                }
//                last_zero = 1;
//            } else {
//                zero_count = 0;
//                last_zero = 0;
//            }
//            break;
//        }

//        // 地址 0x61 处理（无人机状态监测，连续两次0则启动开机序列）
//        case 0x61:
//        {
//            static uint8_t last_zero = 0;
//            static uint8_t zero_count = 0;
//            if (reg_data == 0) {
//                if (last_zero == 1) {
//                    zero_count++;
//                } else {
//                    zero_count = 1;
//                }
//                if (zero_count >= 2) {
//                    printf("检测到连续两次无人机未开机（0x61=0），执行开机序列\n");
//                    if (WaitMasterIdle(100)) {
////                        MasterBusy_Acquire();
////                        Sequence_Start(SEQ_ID_OPENFLY);
////                        MasterBusy_Release();
//                        zero_count = 0;
////                        executed = 1;   // 执行了序列启动
//                    } else {
//                        printf("无人机开机序列失败：总线忙\n");
//                    }
//                }
//                last_zero = 1;
//            } else {
//                zero_count = 0;
//                last_zero = 0;
//            }
//            break;
//        }

//        default:
//            // 查找映射表
//            ret = 0;
//            for (uint8_t i = 0; i < CMD_MAP_COUNT; i++) {
//                if (cmd_map[i].gateway_reg == reg_addr) {
//                    if (WaitMasterIdle(100)) {
//                        MasterBusy_Acquire();
//                        ret = Modbus_06_WriteSingleReg(cmd_map[i].motor_addr, cmd_map[i].motor_reg, reg_data);
//                        MasterBusy_Release();
//                        executed = 1;   // 执行了写操作
//                    } else {
//                        ret = 1; // 总线忙，未执行
//                    }
//                    break;
//                }
//            }
//            break;
//    }

//    // 若实际执行了操作（executed == 1），则更新上次命令记录，用于下次去重
//    if (executed) {
//        last_reg_addr = reg_addr;
//        last_reg_data = reg_data;
//    }

//    // 响应网关
//    if (ret == 0) {
//        SendAckErr(0x02);   // 命令未找到或执行失败
//    } else {
//        SendAckOk();        // 执行成功或总线忙（视为成功）
//    }
//}

/* 10 功能码处理（写多个寄存器） */
void Handle10(void)
{
    uint16_t start_reg = BEBufToUint16(&g_tModS.RxBuf[2]);
    uint16_t reg_num = BEBufToUint16(&g_tModS.RxBuf[4]);
    uint8_t byte_num = g_tModS.RxBuf[6];
    if (byte_num != reg_num * 2) {
        SendAckErr(0x03);
        return;
    }
    uint8_t ok = 1;
    for (uint16_t i = 0; i < reg_num; i++) {
        uint16_t val = BEBufToUint16(&g_tModS.RxBuf[7 + i*2]);
        // 此处仅支持特殊命令，其他可忽略
        // 如果需要支持写多个状态寄存器，可扩展
        // 为简单，只检查第一个寄存器地址是否为命令范围
        if (start_reg + i >= 0x30 && start_reg + i <= 0x48) {
            // 调用写单个寄存器的处理
            uint8_t tmp_buf[8];
            tmp_buf[0] = g_tModS.RxBuf[0];
            tmp_buf[1] = 0x06;
            tmp_buf[2] = (start_reg + i) >> 8;
            tmp_buf[3] = (start_reg + i) & 0xFF;
            tmp_buf[4] = val >> 8;
            tmp_buf[5] = val & 0xFF;
            // 临时替换全局缓冲区，调用 Handle06
            uint8_t old_buf[8];
            memcpy(old_buf, g_tModS.RxBuf, 8);
            memcpy(g_tModS.RxBuf, tmp_buf, 8);
            Handle06();
            memcpy(g_tModS.RxBuf, old_buf, 8);
        } else {
            ok = 0;
            break;
        }
    }
    if (ok) {
        SendAckOk();
    } else {
        SendAckErr(0x02);
    }
}
void StartFrameTimeout(void)
{
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

void StopFrameTimeout(void)
{
    TIM_Cmd(TIM3, DISABLE);
    TIM_SetCounter(TIM3, 0);
}

uint16_t BEBufToUint16(uint8_t *_pBuf)
{
    return (((uint16_t)_pBuf[0] << 8) | _pBuf[1]);
}




static uint8_t MODS_WriteRegValue(uint16_t reg_addr, uint16_t reg_value)
{
	switch (reg_addr)							/* 判断寄存器地址 */
	{	
		case SLAVE_REG_P01:
			g_tVar.P01 = reg_value;				/* 将值写入保存寄存器 */
			break;
		
		case SLAVE_REG_P02:
			g_tVar.P02 = reg_value;				/* 将值写入保存寄存器 */
			break;
		
		default:
			return 0;		/* 参数异常，返回 0 */
	}

	return 1;		/* 读取成功 */
}

/**
 * @brief 检查半小时定时任务是否到期，若到期则执行遥控器开机
 * @note 需在主循环中周期性调用
 */
void CheckRemoteOffTask(void)
{
    if (remote_off_pending && GetTick() >= remote_off_time) {
        remote_off_pending = 0;   // 清除标志，防止重复执行
        printf("半小时定时到期，执行遥控器开机指令\n");
        MasterBusy_Acquire();
        uint8_t ret = Modbus_06_WriteSingleReg(0x14, 0x0001, 0x0001);
        MasterBusy_Release();
        if (ret == 0) {
            printf("遥控器开机指令发送成功（定时任务）\n");
        } else {
            printf("遥控器开机指令发送失败（定时任务），错误码：%d\n", ret);
        }
    }
}
