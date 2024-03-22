#include "cmd_process.h"

void CmdSend(uint8_t MainCmd, uint8_t SubCmd, uint8_t *data, uint8_t len)
{
    uint8_t buff[256];
    uint16_t crc16;
    buff[0] = 0x88;
    buff[1] = MainCmd;
    buff[2] = SubCmd;
    buff[3] = len;
    memcpy(&buff[4], data, len);
    crc16 = mb_crc16(buff, len + 4);
    buff[len + 4] = crc16 & 0xff;
    buff[len + 5] = crc16 >> 8;
    uart_send_data(buff, len + 6);
}

extern uint32_t timeoutCount;
uint8_t CmdProcess(uint8_t *data, uint8_t len)
{
    if (mb_crc16(data, len) != 0)
        return 0;
    if (data[Index_Head] != 0x88)
        return 0;
    switch (data[Index_MainCmd])
    {
    case Cmd_RF433:
        if (data[Index_SubCmd] == 0x10)
        {
            uint8_t i;
            for (i = 0; i < 10; i++)
                if (bellParams[i].bit_num == 0)
                    break;
            CmdSend(Cmd_RF433, 0x11, (uint8_t *)bellParams, sizeof(BellParameters) * i);
        }
        if(data[Index_SubCmd] >= 0x12 && data[Index_SubCmd] < 0x12 + 10)
        {
            memcpy(&bellParams[data[Index_SubCmd] - 0x12], &data[Index_Data], sizeof(BellParameters) - 1);
            FlashSaveCallBellPara(data[Index_SubCmd] - 0x12);
        }
        break;
    case Cmd_Temper:
        if (data[Index_SubCmd] == 0x02)
        {
            timeoutCount = 0;
        }
        break;
    case Cmd_Version:
        if (data[Index_SubCmd] == 0x01)
        {
            uint8_t version = Dev_Version;
            CmdSend(Cmd_Version, 0x02, &version, 1);
        }
        break;
    default:
        break;
    }
}