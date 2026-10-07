#ifndef SLAVE_CTRL_H
#define SLAVE_CTRL_H

/* 无人机遥控器打开及充电从站 */
#define UAV_CONTROLLER_SLAVE          0x14U
#define UAV_POWER_CTRL_REG            0x0001U
#define UAV_POWER_ON_VALUE            0x0001U
#define UAV_POWER_MODE2_VALUE         0x0002U

/* 电池充电器从站 */
#define CHARGER_SLAVE                 0x13U
#define CHARGER_POWER_CTRL_REG        0x0000U
#define CHARGER_POWER_ON_VALUE        0x0001U
#define CHARGER_POWER_OFF_VALUE       0x0000U

/* 空调从站 */
#define AIR_CONDITIONER_SLAVE         0x0DU
#define AIR_POWER_CTRL_REG            0x002FU
#define AIR_POWER_ON_VALUE            0x0001U
#define AIR_POWER_OFF_VALUE           0x0000U
#define AIR_COOLING_STOP_TEMP_REG     0x0000U
#define AIR_HEATING_STOP_TEMP_REG     0x0002U
/* 降雨量从站 */
#define RAINFALL_SLAVE                0x0EU
#define RAINFALL_VALUE_REG            0x0000U
/* 风速从站 */
#define WIND_SPEED_SLAVE              0x0FU
#define WIND_SPEED_VALUE_REG          0x0000U
/* 机舱温度从站 */
#define CABIN_TEMP_SLAVE              0x16U
#define CABIN_TEMP_VALUE_REG          0x0000U

/* 充电电流监测从站 */
#define CHARGING_CURRENT_SLAVE        0x20U
#define CHARGING_CURRENT_VALUE_REG    0x0EU

#endif
