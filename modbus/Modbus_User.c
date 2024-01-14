#include "Modbus_Core.h"

MBS_CoilValueTypes coilValue_ID[ID_COIL_NUM], coilValue_RF433[RF433_COIL_NUM];
MBS_CoilTypes mbsCoil_ID = {coilValue_ID, 0, 0, ID_COIL_NUM};
MBS_CoilTypes mbsCoil_RF433 = {coilValue_RF433, 0, 0, RF433_COIL_NUM};

MBS_HoldRegValueTypes HoldRegValue_ID[ID_HOLDREG_NUM], HoldRegValue_RF433[RF433_HOLDREG_NUM];
MBS_HoldRegTypes mbsHoldReg_ID = {HoldRegValue_ID, 0, 0, ID_HOLDREG_NUM};
MBS_HoldRegTypes mbsHoldReg_RF433 = {HoldRegValue_RF433, 0, 0, RF433_HOLDREG_NUM};

mbsNode_t mbsNode[MBS_NODE_NUM];

void MBS_MappingCoilInit(MBS_CoilValueTypes *coil, uint16 addr, uint8 Value)
{
    coil->coilAddr = addr;
    coil->pData = Value;
}

void MBS_MappingHoldRegInit(MBS_HoldRegValueTypes *reg, uint16 addr, uint16 Value)
{
    reg->regAddr = addr;
    reg->pData = Value;
}
/*
 */
void ModbusIDMapInit()
{
    uint8 i;
    mbsCoil_ID._startAddr = 10000; /* 起始地址 */
    mbsCoil_ID._endAddr = 10004;   /* 结束地址 */

    for (i = 0; i < 5; i++)
    {
        MBS_MappingCoilInit(&mbsCoil_ID._Value[i], i + 10000, 0);
    }

    mbsHoldReg_ID._startAddr = 50000; /* 起始地址 */
    mbsHoldReg_ID._endAddr = 59001;   /* 结束地址 */
    for (i = 0; i < 10; i++)
    {
        MBS_MappingHoldRegInit(&mbsHoldReg_ID._Value[i], i + 50200, 0);
    }
    for (i = 0; i < 2; i++)
    {
        MBS_MappingHoldRegInit(&mbsHoldReg_ID._Value[i + 10], i + 59000, 0);
    }
    mbsHoldReg_ID._Value[MBS_Addr].pData = MBS_IDAddr;
    mbsHoldReg_ID._Value[MBS_Ver].pData = MBS_IDVersion;

    mbsNode[0].mbsAddr = MBS_IDAddr;
    mbsNode[0].coil = &mbsCoil_ID;
    mbsNode[0].HoldReg = &mbsHoldReg_ID;

}

void ModbusRF433MapInit()
{
	uint8 i;
    mbsCoil_RF433._startAddr = 5000; /* 起始地址 */
    mbsCoil_RF433._endAddr = 5002;   /* 结束地址 */

    for (i = 0; i < 3; i++)
    {
        MBS_MappingCoilInit(&coilValue_RF433[i], i + 5000, 0);
    }

    mbsHoldReg_RF433._startAddr = 40000; /* 起始地址 */
    mbsHoldReg_RF433._endAddr = 59001;   /* 结束地址 */
    for (i = 0; i < 2; i++)
    {
        MBS_MappingHoldRegInit(&mbsHoldReg_RF433._Value[i], i + 40000, 0);
    }
    for (i = 0; i < 32; i++)
	{
		MBS_MappingHoldRegInit(&mbsHoldReg_RF433._Value[i + 2], 40100 + i, 0);
	}

    for (i = 0; i < 2; i++)
	{
		MBS_MappingHoldRegInit(&mbsHoldReg_RF433._Value[i + 34], 40200 + i, 0);
	}

	for (i = 0; i < 3; i++)
	{
		MBS_MappingHoldRegInit(&mbsHoldReg_RF433._Value[i + 36], 45000 + i, 0);
	}
    for ( i = 0; i < 2; i++)
	{
		MBS_MappingHoldRegInit(&mbsHoldReg_RF433._Value[i + 39], 46000 + i, 0);
	}
	for (i = 0; i < 2; i++)
	{
		MBS_MappingHoldRegInit(&mbsHoldReg_RF433._Value[i + 41], 59000 + i, 0);
	}
    mbsHoldReg_RF433._Value[RF433_REG_MBS_ADDR].pData = MBS_RF433Addr;
    mbsHoldReg_RF433._Value[RF433_REG_MBS_VER].pData = MBS_RF433Version;

    mbsNode[1].mbsAddr = MBS_RF433Addr;
    mbsNode[1].coil = &mbsCoil_RF433;
    mbsNode[1].HoldReg = &mbsHoldReg_RF433;
}
uint8 MBS_MemReadCoilState(MBS_CoilTypes *coilMap, uint16 coilAddr)
{
    uint16 i;
    for (i = 0; i < coilMap->_num; i++)
    {
        if (coilMap->_Value[i].pData == coilAddr)
            return (coilMap->_Value[i].pData) % 2;
    }
    return i;
}
uint16 MBS_MemWriteCoilsState(MBS_CoilTypes *coilMap, uint16 CoilAddr, uint16 Len, uint8 *data)
{
    uint16 i, j;

    for (i = 0; i < coilMap->_num; i++) // 查找符合地址
    {
        if (coilMap->_Value[i].coilAddr == CoilAddr)
            break;
    }
    if (i >= coilMap->_num)
        return i; /*读取失败，地址错误*/

    for (j = 0; j < Len; j++)
        coilMap->_Value[i + j].pData = (data[j / 8] >> (j % 8)) & 0x01;

    return 1; /* 读取成功 */
}
uint8 MBS_MemWriteCoilState(MBS_CoilTypes *coilMap, uint16 coilAddr, uint16 data)
{
    uint16 i;
    for (i = 0; i < coilMap->_num; i++)
    {
        if (coilMap->_Value[i].coilAddr == coilAddr)
        {
            if (data)
                coilMap->_Value[i].pData = 1;
            else
                coilMap->_Value[i].pData = 0;
            return 1;
        }
    }
    return i;
}

uint8 MBS_MemReadHoldRegValue(MBS_HoldRegTypes *HoldRegMap, uint16 regAddr, uint8 *Value, uint8 num)
{
    uint8 i, j;
    for (i = 0; i < HoldRegMap->_num; i++)
    {
        if (HoldRegMap->_Value[i].regAddr == regAddr) /* 保持寄存器首地址正确 */
        {
            if (HoldRegMap->_Value[i + num - 1].regAddr == (regAddr + num - 1)) /* 保持寄存器尾地址正确 */
            {
                for (j = 0; j < num; j++)
                {
                    *Value++ = (HoldRegMap->_Value[i + j].pData) >> 8;
                    *Value++ = (HoldRegMap->_Value[i + j].pData);
                }
                return 1; /* 读取成功 */
            }
        }
    }
    return 0;
}

uint8 MBS_MemWriteHoldRegValue(MBS_HoldRegTypes *HoldRegMap, uint16 regAddr, uint8 *Value, uint8 num)
{
    uint8 i, j;
    for (i = 0; i < HoldRegMap->_num; i++)
    {
        if (HoldRegMap->_Value[i].regAddr == regAddr) /* 保持寄存器首地址正确 */
        {
            if (HoldRegMap->_Value[i + num - 1].regAddr == (regAddr + num - 1)) /* 保持寄存器尾地址正确 */
            {
                for (j = 0; j < num; j++)
                {
                    HoldRegMap->_Value[i + j].pData = (*Value++) << 8;
                    HoldRegMap->_Value[i + j].pData |= *Value++;
                }
                return 1; /* 写入成功 */
            }
        }
    }
    return 0;
}
