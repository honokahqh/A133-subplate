#ifndef MODBUS_USER_H
#define MODBUS_USER_H
#include "Modbus_Core.h"
/*********************************************************************
 * @author      Honokahqh
 *
 * @brief       Modbus slaver
 *
 * @time:       2022.3.26
 */
/*Modbus slaver By Honokahqh*/
/***************************************************************/

#define MBS_IDAddr 0x7D
#define MBS_IDVersion 100
#define MBS_ID 0

#define MBS_RF433Addr 70
#define MBS_RF433Version 100
#define MBS_RF433 1

#define Dev_Version 101

/***************************************************************/
#define MBS_FUNCTION_01_ENABLE (1)
#define MBS_FUNCTION_02_ENABLE (0)
#define MBS_FUNCTION_03_ENABLE (1)
#define MBS_FUNCTION_04_ENABLE (0)
#define MBS_FUNCTION_05_ENABLE (1)
#define MBS_FUNCTION_06_ENABLE (1)
#define MBS_FUNCTION_0F_ENABLE (1)
#define MBS_FUNCTION_10_ENABLE (1)

#define ID_COIL_NUM (31)
#define ID_HOLDREG_NUM (12)

#define RF433_COIL_NUM (3)
#define RF433_HOLDREG_NUM (43)

#define USER_DISINPUT_NUM (0)
#define USER_INPUTREG_NUM (0)

#define MBS_PORT_RXBUFF_SIZE 64
#define MBS_PORT_TXBUFF_SIZE 64

#define COIL_ID 2
#define COIL_M1 4
#define REG_RFID 0
#define REG_M1 4
#define MBS_Addr 10
#define MBS_Ver 11

// coil:5000 0:正常 1:注册 5001:绑定reg 40100的呼叫铃 5002:注册模式有效 清除绑定
// reg:40000 按键索引,每个设备对应一个bit 40100:呼叫铃按键ID,每个设备对应一个寄存器
// reg:40200 注册模式下按键ID
// reg:45000 需要绑定的按键 地址索引 ID_H ID_L
// reg:46000 
#define Bell_MAX_NUM    32
#define RF433_COIL_STUDY 0
#define RF433_COIL_BIND 1
#define RF433_COIL_CLEAR 2
#define RF433_REG_WhichDown_H 0 // 40000
#define RF433_REG_WhichDown_L 1

#define RF433_REG_KeyCmd 2 // 40100

#define RF433_REG_Register_ID_H 34 // 40200
#define RF433_REG_Register_ID_L 35

#define RF433_REG_BIND_INDEX 36 // 45000
#define RF433_REG_BIND_ID_H 37  // 45001
#define RF433_REG_BIND_ID_L 38  // 45002

#define RF433_REG_BELL_ID_H 39 // 46000
#define RF433_REG_BELL_ID_L 40

#define RF433_REG_MBS_ADDR 41
#define RF433_REG_MBS_VER 42

void ModbusIDMapInit(void);
void ModbusRF433MapInit(void);
#endif /* MODBUS_USER_H */
