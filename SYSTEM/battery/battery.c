#include "project.h"
#include <stdio.h>
#include <string.h>


void RemoveBattery(void)
{
	  int battery_step = 0;  // 0:初始状态
		
		while(1)
   {
				switch(battery_step)
				{
						case 0:     // 1：设置电机1初始速度
								Battery1();
								battery_step = 1;               // 进入下一步
								tmr.TIMER_3_100MS = 5;          // 启动定时器3
								break;

						case 1:     // 2：等待定时器3超时，设置电机2初始速度
								if(tmr.TIMER_3_100MS == 0) {
										Battery2();
										battery_step = 2;           // 进入下一步
										tmr.TIMER_4_100MS = 5;      // 启动定时器4
								}
								break;

						case 2:     // 3：等待定时器4超时，电机2前进
								if(tmr.TIMER_4_100MS == 0) {
										Battery3();
										battery_step = 3;           // 进入下一步
										tmr.TIMER_3_100MS = 210;    // 启动定时器3
								}
								break;

						case 3:     // 4：等待定时器3超时，电机1下降设置
								if(tmr.TIMER_3_100MS == 0) {
										Battery4();
										battery_step = 4;           // 进入下一步
										tmr.TIMER_4_100MS = 5;      // 启动定时器4
								}
								break;

						case 4:     // 5：等待定时器4超时，电机1启动
								if(tmr.TIMER_4_100MS == 0) {
										Battery5();
										battery_step = 5;           // 进入下一步
										tmr.TIMER_3_100MS = 130;    // 启动定时器3
								}
								break;

						case 5:     // 6：等待定时器3超时，电机2前进
								if(tmr.TIMER_3_100MS == 0) {
										Battery6();
										battery_step = 6;           // 进入下一步
										tmr.TIMER_4_100MS = 20;     // 启动定时器4
								}
								break;

						case 6:     // 7：等待定时器4超时，电机1上升设置
								if(tmr.TIMER_4_100MS == 0) {
										Battery7();
										battery_step = 7;           // 进入下一步
										tmr.TIMER_3_100MS = 5;      // 启动定时器3
								}
								break;

						case 7:     // 8：等待定时器3超时，电机1启动
								if(tmr.TIMER_3_100MS == 0) {
										Battery8();
										battery_step = 8;           // 进入下一步
										tmr.TIMER_4_100MS = 130;    // 启动定时器4
								}
								break;

						case 8:     // 9：等待定时器4超时，电机3夹紧
								if(tmr.TIMER_4_100MS == 0) {
										Battery9();
										battery_step = 9;           // 进入下一步
										tmr.TIMER_3_100MS = 5;      // 启动定时器3
								}
								break;

						case 9:     // 10：等待定时器3超时，电机2后退
								if(tmr.TIMER_3_100MS == 0) {
										Battery10();
										battery_step = 10;          // 进入下一步
										tmr.TIMER_4_100MS = 240;    // 启动定时器4
								}
								break;

						case 10:    // 11：等待定时器4超时，电机3松开
								if(tmr.TIMER_4_100MS == 0) {
										Battery11();
										battery_step = 11;          // 进入下一步
										tmr.TIMER_3_100MS = 5;      // 启动定时器3
								}
								break;

						case 11:    // 12：等待定时器3超时，电机1下降设置
								if(tmr.TIMER_3_100MS == 0) {
										Battery12();
										battery_step = 12;          // 进入下一步
										tmr.TIMER_4_100MS = 5;      // 启动定时器4
								}
								break;

						case 12:    // 13：等待定时器4超时，电机1启动
								if(tmr.TIMER_4_100MS == 0) {
										Battery13();
										battery_step = 13;          // 进入下一步
										tmr.TIMER_3_100MS = 130;    // 启动定时器3
								}
								break;

						case 13:    // 14：等待定时器3超时，电机2后退
								if(tmr.TIMER_3_100MS == 0) {
										Battery14();
										battery_step = 14;          // 进入下一步
										tmr.TIMER_4_100MS = 20;     // 启动定时器4
								}
								break;

						case 14:    // 15：等待定时器4超时，电机1上升设置
								if(tmr.TIMER_4_100MS == 0) {
										Battery15();
										battery_step = 15;          // 进入下一步
										tmr.TIMER_3_100MS = 5;      // 启动定时器3
								}
								break;

						case 15:    // 16：等待定时器3超时，电机1启动
								if(tmr.TIMER_3_100MS == 0) {
										Battery16();
										battery_step = 16;          // 进入下一步
										tmr.TIMER_4_100MS = 100;    // 启动定时器4
								}
								break;

						case 16:    // 17：等待定时器4超时，电机2前进
								if(tmr.TIMER_4_100MS == 0) {
										Battery17();
										battery_step = 17;          // 进入下一步
										tmr.TIMER_3_100MS = 110;    // 启动定时器3
								}
								break;

						case 17:    // 18：等待定时器3超时，电机1下降设置
								if(tmr.TIMER_3_100MS == 0) {
										Battery18();
										battery_step = 18;          // 进入下一步
										tmr.TIMER_4_100MS = 5;      // 启动定时器4
								}
								break;

						case 18:    // 19：等待定时器4超时，电机1启动
								if(tmr.TIMER_4_100MS == 0) {
										Battery19();
										battery_step = 19;          // 进入下一步
										tmr.TIMER_3_100MS = 130;    // 启动定时器3
								}
								break;

						case 19:    // 20：等待定时器3超时，电机2前进
								if(tmr.TIMER_3_100MS == 0) {
										Battery20();
										battery_step = 20;          // 进入下一步
										tmr.TIMER_4_100MS = 20;     // 启动定时器4
								}
								break;

						case 20:    // 21：等待定时器4超时，电机1上升设置
								if(tmr.TIMER_4_100MS == 0) {
										Battery21();
										battery_step = 21;          // 进入下一步
										tmr.TIMER_3_100MS = 5;      // 启动定时器3
								}
								break;

						case 21:    // 22：等待定时器3超时，电机1启动
								if(tmr.TIMER_3_100MS == 0) {
										Battery22();
										battery_step = 22;          // 进入下一步
										tmr.TIMER_4_100MS = 130;    // 启动定时器4
								}
								break;

						case 22:    // 23：等待定时器4超时，电机3夹紧
								if(tmr.TIMER_4_100MS == 0) {
										Battery23();
										battery_step = 23;          // 进入下一步
										tmr.TIMER_3_100MS = 5;      // 启动定时器3
								}
								break;

						case 23:    // 24：等待定时器3超时，电机2前进
								if(tmr.TIMER_3_100MS == 0) {
										Battery24();
										battery_step = 24;          // 进入下一步
										tmr.TIMER_4_100MS = 120;    // 启动定时器4
								}
								break;

						case 24:    // 25：等待定时器4超时，电机3松开
								if(tmr.TIMER_4_100MS == 0) {
										Battery25();
										battery_step = 25;          // 进入下一步
										tmr.TIMER_3_100MS = 5;      // 启动定时器3
								}
								break;

						case 25:    // 26：等待定时器3超时，电机1下降设置
								if(tmr.TIMER_3_100MS == 0) {
										Battery26();
										battery_step = 26;          // 进入下一步
										tmr.TIMER_4_100MS = 5;      // 启动定时器4
								}
								break;

						case 26:    // 27：等待定时器4超时，电机1启动
								if(tmr.TIMER_4_100MS == 0) {
										Battery27();
										battery_step = 27;          // 进入下一步
										tmr.TIMER_3_100MS = 130;    // 启动定时器3
								}
								break;

						case 27:    // 28：等待定时器3超时，电机2后退
								if(tmr.TIMER_3_100MS == 0) {
										Battery28();
										battery_step = 28;          // 进入下一步
										tmr.TIMER_4_100MS = 10;     // 启动定时器4
								}
								break;

						case 28:    // 29：等待定时器4超时，电机1上升设置
								if(tmr.TIMER_4_100MS == 0) {
										Battery29();
										battery_step = 29;          // 进入下一步
										tmr.TIMER_3_100MS = 5;      // 启动定时器3
								}
								break;

						case 29:    // 30：等待定时器3超时，电机1启动
								if(tmr.TIMER_3_100MS == 0) {
										Battery30();
									  battery_step = 30;          // 进入下一步
									  tmr.TIMER_4_100MS = 130;    // 启动定时器4
								}
								break;
								
						case 30: // 31：等待定时器4超时，电机1启动
								if(tmr.TIMER_4_100MS == 0) {
									  battery_step = 31;          // 进入下一步
										break;                     // 退出循环
								}
								return;

						default:
								return;
				}

				// 添加适当延时避免CPU占用过高
				delay_ms(10);
   }
}	
	

void Battery1(void)
{
  	//电机1初始速度
		uint8_t motor_num1 = 3;   // 寄存器地址  
		uint16_t motor_cmd1 = MOTOR1_RUN; // 速度30000（0x7530）
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);//指令发送返回值
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}

void Battery2(void)
{
  	//电机2初始速度
		uint8_t motor_num1 = 4; //行程低位寄存器地址
		uint16_t motor_cmd1 = Speed; 
		uint8_t ctrl_ret1 = Motor_Control(MOTOR2_SLAVE_ADDR, motor_num1, motor_cmd1);
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}

void Battery3(void)
{
//2.电机2前进 
		uint8_t Reg_num = 2;   // 寄存器数量
		uint16_t slave2_cmds[Reg_num];//寄存器指令
		// 初始化数组（避免脏数据）
		memset(slave2_cmds, 0, sizeof(slave2_cmds));
		if(Reg_num >= 2) 
		{
				slave2_cmds[0] = Pulse_num1;            // 寄存器1：高位
				slave2_cmds[1] = Pulse_num2;            // 寄存器2：低位			
//				printf("赋值后：slave1_cmds[2] = 0x%04X\n", slave1_cmds[2]); // 确认赋值成功
		} else
		{
				// Reg_num不足时的容错处理（比如清空数组）
				memset(slave2_cmds, 0, sizeof(slave2_cmds));
		}
		if(master_state != MASTER_IDLE) 
		{
				master_state = MASTER_IDLE;
				timeout_cnt = 0;
				printf("强制重置主站状态为空闲\r\n");
				fflush(stdout);
		}
		uint8_t ctrl_ret2 = Motor_Batch_Control(MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG3, Reg_num, slave2_cmds);// 指令发送返回值 
		// 前进指令结果判断
    switch(ctrl_ret2)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}

void Battery4(void)
{
  	//3.电机1下降设置
		uint8_t motor_num1 = 2;  //圈数
		uint16_t motor_cmd1 = MOTOR1_deLengthRun; //下降方向和圈数值设置
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);// 指令发送返回值 
		// 下降指令结果判断
    switch(ctrl_ret1)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}

void Battery5(void)
{
  	//4.电机1启动
		uint8_t motor_num1 = 1;  //启停
		uint16_t motor_cmd1 = MOTOR1_deRUN; //启动
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);// 指令发送返回值 
		switch(ctrl_ret1)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}  	
		
void Battery6(void)
{
  			//5.电机2前进
		// 初始化数组（避免脏数据）
		uint8_t Reg_num = 2;   // 寄存器数量
	  uint16_t slave2_cmds[Reg_num];//寄存器指令
		// 初始化数组（避免脏数据）
		memset(slave2_cmds, 0, sizeof(slave2_cmds));
		if(Reg_num >= 2) 
		{
				slave2_cmds[0] = Pulse_num3;            // 寄存器1：高位
				slave2_cmds[1] = Pulse_num4;            // 寄存器2：低位			
//				printf("赋值后：slave1_cmds[2] = 0x%04X\n", slave1_cmds[2]); // 确认赋值成功
		} else
		{
				// Reg_num不足时的容错处理（比如清空数组）
				memset(slave2_cmds, 0, sizeof(slave2_cmds));
		}
		if(master_state != MASTER_IDLE) 
		{
				master_state = MASTER_IDLE;
				timeout_cnt = 0;
				printf("强制重置主站状态为空闲\r\n");
				fflush(stdout);
		}
		uint8_t ctrl_ret2 = Motor_Batch_Control(MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG3, Reg_num, slave2_cmds);
		// 前进指令结果判断
    switch(ctrl_ret2)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}		
		
void Battery7(void)
{
  	//6.电机1上升设置
		uint8_t motor_num1 = 2;  //圈数
		uint16_t motor_cmd1 = MOTOR1_LengthRun; //上升方向和圈数值设置
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);
		// 下降指令结果判断
    switch(ctrl_ret1)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
} 		
		
void Battery8(void)
{
  	//7.电机1启动
		uint8_t motor_num1 = 1;  //启停
		uint16_t motor_cmd1 = MOTOR1_deRUN; //启动
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);
		switch(ctrl_ret1)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
} 		
	 
void Battery9(void)
{
  	//8.电机3夹住电池
		uint8_t ctrl_ret3;  // 指令发送返回值
		uint8_t motor_num3 = 2;   // 寄存器地址（夹紧）
		uint16_t motor_cmd3 = Clamp; // 电机指令：
		ctrl_ret3 = Motor_Control(MOTOR3_SLAVE_ADDR, motor_num3, motor_cmd3);
		if(ctrl_ret3 == 0)
    {
        printf("从机3夹紧指令发送成功！\r\n");
    }
    else
    {
        printf("从机3夹紧指令发送失败，错误码：%d\r\n", ctrl_ret3);
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
} 		
		
void Battery10(void)
{
  	//9.电机2后退
		uint8_t Reg_num = 2;   // 寄存器数量
	  uint16_t slave2_cmds[Reg_num];//寄存器指令
		// 初始化数组（避免脏数据）
		memset(slave2_cmds, 0, sizeof(slave2_cmds));
		if(Reg_num >= 2) 
		{
				slave2_cmds[0] = Pulse_num9;            // 寄存器1：高位
				slave2_cmds[1] = Pulse_num10;            // 寄存器2：低位			
//				printf("赋值后：slave1_cmds[2] = 0x%04X\n", slave1_cmds[2]); // 确认赋值成功
		} else
		{
				// Reg_num不足时的容错处理（比如清空数组）
				memset(slave2_cmds, 0, sizeof(slave2_cmds));
		}
		if(master_state != MASTER_IDLE) 
		{
				master_state = MASTER_IDLE;
				timeout_cnt = 0;
				printf("强制重置主站状态为空闲\r\n");
				fflush(stdout);
		}
		uint8_t ctrl_ret2 = Motor_Batch_Control(MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG2, Reg_num, slave2_cmds);
		// 前进指令结果判断
    switch(ctrl_ret2)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
} 		
		
void Battery11(void)
{
  	//10.电机3松开电池
		uint8_t motor_num3 = 1;   // 寄存器地址（松开）
		uint16_t motor_cmd3 = Lossen; // 电机指令：
		uint8_t ctrl_ret3 = Motor_Control(MOTOR3_SLAVE_ADDR, motor_num3, motor_cmd3);
		if(ctrl_ret3 == 0)
    {
        printf("从机3松开指令发送成功！\r\n");
    }
    else
    {
        printf("从机3松开指令发送失败，错误码：%d\r\n", ctrl_ret3);
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
} 	
		
void Battery12(void)
{
  	//11.电机1下降设置
		uint8_t motor_num1 = 2;  //圈数
		uint16_t motor_cmd1 = MOTOR1_deLengthRun; //下降方向和圈数值设置
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);
		// 下降指令结果判断
    switch(ctrl_ret1)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
} 		
		
void Battery13(void)
{
  	//12.电机1启动
		uint8_t motor_num1 = 1;  //启停
		uint16_t motor_cmd1 = MOTOR1_deRUN; //启动
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);
		switch(ctrl_ret1)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器		
} 		
		
void Battery14(void)
{
  	//13.电机2后退
		uint8_t Reg_num = 2;   // 寄存器数量
	  uint16_t slave2_cmds[Reg_num];//寄存器指令
		// 初始化数组（避免脏数据）
		memset(slave2_cmds, 0, sizeof(slave2_cmds));
		if(Reg_num >= 2) 
		{
				slave2_cmds[0] = Pulse_num11;            // 寄存器1：高位
				slave2_cmds[1] = Pulse_num12;            // 寄存器2：低位			
//				printf("赋值后：slave1_cmds[2] = 0x%04X\n", slave1_cmds[2]); // 确认赋值成功
		} else
		{
				// Reg_num不足时的容错处理（比如清空数组）
				memset(slave2_cmds, 0, sizeof(slave2_cmds));
		}
		if(master_state != MASTER_IDLE) 
		{
				master_state = MASTER_IDLE;
				timeout_cnt = 0;
				printf("强制重置主站状态为空闲\r\n");
				fflush(stdout);
		}
		uint8_t ctrl_ret2 = Motor_Batch_Control(MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG2, Reg_num, slave2_cmds);
		// 前进指令结果判断
    switch(ctrl_ret2)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器	
} 		
		
				
void Battery15(void)
{
  	//14.电机1上升设置
		uint8_t motor_num1 = 2;  //圈数
		uint16_t motor_cmd1 = MOTOR1_LengthRun; //上升方向和圈数值设置
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);
		// 下降指令结果判断
    switch(ctrl_ret1)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}
		
void Battery16(void)
{
  	//15.电机1启动
		uint8_t motor_num1 = 1;  //启停
		uint16_t motor_cmd1 = MOTOR1_deRUN; //启动
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);
		switch(ctrl_ret1)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}	
		
		
void Battery17(void)
{
  	//16.电机2前进
		uint8_t Reg_num = 2;   // 寄存器数量
	  uint16_t slave2_cmds[Reg_num];//寄存器指令
		// 初始化数组（避免脏数据）
		memset(slave2_cmds, 0, sizeof(slave2_cmds));
		if(Reg_num >= 2) 
		{
				slave2_cmds[0] = Pulse_num5;            // 寄存器1：高位
				slave2_cmds[1] = Pulse_num6;            // 寄存器2：低位			
//				printf("赋值后：slave1_cmds[2] = 0x%04X\n", slave1_cmds[2]); // 确认赋值成功
		} else
		{
				// Reg_num不足时的容错处理（比如清空数组）
				memset(slave2_cmds, 0, sizeof(slave2_cmds));
		}
		if(master_state != MASTER_IDLE) 
		{
				master_state = MASTER_IDLE;
				timeout_cnt = 0;
				printf("强制重置主站状态为空闲\r\n");
				fflush(stdout);
		}
		uint8_t ctrl_ret2 = Motor_Batch_Control(MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG3, Reg_num, slave2_cmds);
		// 前进指令结果判断
    switch(ctrl_ret2)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}			
		
void Battery18(void)
{
  	//17.电机1下降设置
		uint8_t motor_num1 = 2;  //圈数
		uint16_t motor_cmd1 = MOTOR1_deLengthRun; //下降方向和圈数值设置
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);
		// 下降指令结果判断
    switch(ctrl_ret1)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}			
		
void Battery19(void)
{
  	//18.电机1启动
		uint8_t motor_num1 = 1;  //启停
		uint16_t motor_cmd1 = MOTOR1_deRUN; //启动
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);
		switch(ctrl_ret1)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}			
		
void Battery20(void)
{
  	//19.电机2前进
		uint8_t Reg_num = 2;   // 寄存器数量
	  uint16_t slave2_cmds[Reg_num];//寄存器指令
		// 初始化数组（避免脏数据）
		memset(slave2_cmds, 0, sizeof(slave2_cmds));
		if(Reg_num >= 2) 
		{
				slave2_cmds[0] = Pulse_num3;            // 寄存器1：高位
				slave2_cmds[1] = Pulse_num4;            // 寄存器2：低位			
//				printf("赋值后：slave1_cmds[2] = 0x%04X\n", slave1_cmds[2]); // 确认赋值成功
		} else
		{
				// Reg_num不足时的容错处理（比如清空数组）
				memset(slave2_cmds, 0, sizeof(slave2_cmds));
		}
		if(master_state != MASTER_IDLE) 
		{
				master_state = MASTER_IDLE;
				timeout_cnt = 0;
				printf("强制重置主站状态为空闲\r\n");
				fflush(stdout);
		}
		uint8_t ctrl_ret2 = Motor_Batch_Control(MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG3, Reg_num, slave2_cmds);
		// 前进指令结果判断
    switch(ctrl_ret2)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}		
	
void Battery21(void)
{
  	//20.电机1上升设置
		uint8_t motor_num1 = 2;  //圈数
		uint16_t motor_cmd1 = MOTOR1_LengthRun; //上升方向和圈数值设置
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);
		// 下降指令结果判断
    switch(ctrl_ret1)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}
		
void Battery22(void)
{
  	//21.电机1启动
		uint8_t motor_num1 = 1;  //启停
		uint16_t motor_cmd1 = MOTOR1_deRUN; //启动
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);
		switch(ctrl_ret1)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}			
		
void Battery23(void)
{
  	//22.电机3夹紧电池
		uint8_t motor_num3 = 2;   // 寄存器地址（夹紧）
		uint16_t motor_cmd3 = Clamp; // 电机指令：
		uint8_t ctrl_ret3 = Motor_Control(MOTOR3_SLAVE_ADDR, motor_num3, motor_cmd3);
		if(ctrl_ret3 == 0)
    {
        printf("从机3夹紧指令发送成功！\r\n");
    }
    else
    {
        printf("从机3夹紧指令发送失败，错误码：%d\r\n", ctrl_ret3);
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}			
		
void Battery24(void)
{
  	//23.电机2前进
		uint8_t Reg_num = 2;   // 寄存器数量
	  uint16_t slave2_cmds[Reg_num];//寄存器指令
		// 初始化数组（避免脏数据）
		memset(slave2_cmds, 0, sizeof(slave2_cmds));
		if(Reg_num >= 2) 
		{
				slave2_cmds[0] = Pulse_num7;            // 寄存器1：高位
				slave2_cmds[1] = Pulse_num8;            // 寄存器2：低位			
//				printf("赋值后：slave1_cmds[2] = 0x%04X\n", slave1_cmds[2]); // 确认赋值成功
		} else
		{
				// Reg_num不足时的容错处理（比如清空数组）
				memset(slave2_cmds, 0, sizeof(slave2_cmds));
		}
		if(master_state != MASTER_IDLE) 
		{
				master_state = MASTER_IDLE;
				timeout_cnt = 0;
				printf("强制重置主站状态为空闲\r\n");
				fflush(stdout);
		}
		uint8_t ctrl_ret2 = Motor_Batch_Control(MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG3, Reg_num, slave2_cmds);
		// 前进指令结果判断
    switch(ctrl_ret2)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}			
				
void Battery25(void)
{
  	//24.电机3松开电池
		uint8_t motor_num3 = 1;   // 寄存器地址（松开）
		uint16_t motor_cmd3 = Lossen; // 电机指令：
		uint8_t ctrl_ret3 = Motor_Control(MOTOR3_SLAVE_ADDR, motor_num3, motor_cmd3);
		if(ctrl_ret3 == 0)
    {
        printf("从机3松开指令发送成功！\r\n");
    }
    else
    {
        printf("从机3松开指令发送失败，错误码：%d\r\n", ctrl_ret3);
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}

void Battery26(void)
{
  	//25.电机1下降设置
		uint8_t motor_num1 = 2;  //圈数
		uint16_t motor_cmd1 = MOTOR1_deLengthRun; //下降方向和圈数值设置
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);
		// 下降指令结果判断
    switch(ctrl_ret1)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}
		
void Battery27(void)
{
  	//26.电机1启动
		uint8_t motor_num1 = 1;  //启停
		uint16_t motor_cmd1 = MOTOR1_deRUN; //启动
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);
		switch(ctrl_ret1)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}		
		
void Battery28(void)
{	
	  //27.电机2后退
		uint8_t Reg_num = 2;   // 寄存器数量
	  uint16_t slave2_cmds[Reg_num];//寄存器指令
		// 初始化数组（避免脏数据）
		memset(slave2_cmds, 0, sizeof(slave2_cmds));
		if(Reg_num >= 2) 
		{
				slave2_cmds[0] = Pulse_num13;            // 寄存器1：高位
				slave2_cmds[1] = Pulse_num14;            // 寄存器2：低位			
		} else
		{
				// Reg_num不足时的容错处理（比如清空数组）
				memset(slave2_cmds, 0, sizeof(slave2_cmds));
		}
		if(master_state != MASTER_IDLE) 
		{
				master_state = MASTER_IDLE;
				timeout_cnt = 0;
				printf("强制重置主站状态为空闲\r\n");
				fflush(stdout);
		}
		uint8_t ctrl_ret2 = Motor_Batch_Control(MOTOR2_SLAVE_ADDR, MOTOR2_CTRL_REG2, Reg_num, slave2_cmds);
		// 前进指令结果判断
    switch(ctrl_ret2)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}	 
		
void Battery29(void)
{
  	//28.电机1上升设置
		uint8_t motor_num1 = 2;  //圈数
		uint16_t motor_cmd1 = MOTOR1_LengthRun; //上升方向和圈数值设置
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);
		// 下降指令结果判断
    switch(ctrl_ret1)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}			
		
void Battery30(void)
{
  	//29.电机1启动
		uint8_t motor_num1 = 1;  //启停
		uint16_t motor_cmd1 = MOTOR1_deRUN; //启动
		uint8_t ctrl_ret1 = Motor_Control(MOTOR1_SLAVE_ADDR, motor_num1, motor_cmd1);
		switch(ctrl_ret1)
    {
        case 0:
            printf("上升指令发送成功，电机开始运行\r\n");
            break;
        case 1:
            printf("错误：电机编号非法或主站非空闲（上升指令未发送）\r\n");
            // 直接退出，避免执行停止指令
            while(1) { delay_ms(1000); }
        case 2:
            printf("警告：上升指令超时，但电机可能已运行\r\n");
            break;
        case 3:
            printf("错误：上升指令响应内容不匹配\r\n");
            break;
        default:
            printf("错误：上升指令发送异常\r\n");
            break;
    }
		master_state = MASTER_IDLE;
		timeout_cnt = 0; // 同时重置超时计数器
}	




	
