#ifndef __CMD_PROCESS_H__
#define __CMD_PROCESS_H__

#include "main.h"
#include "app.h"

void CmdSend(uint8_t MainCmd,uint8_t SubCmd, uint8_t *data, uint8_t len);
uint8_t CmdProcess(uint8_t *data, uint8_t len);

#define Index_Head    0
#define Index_MainCmd 1
#define Index_SubCmd  2
#define Index_Len     3
#define Index_Data    4

#define Cmd_IDCard  0x01
#define Cmd_M1Card  0x02
#define Cmd_RF433   0x03
#define Cmd_Temper  0x04
#define Cmd_Version 0x05
#define Cmd_OTA     0x06

#endif // !__CMD_PROCESS_H__
