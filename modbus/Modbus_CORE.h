#ifndef MODBUS_CORE_H
#define MODBUS_CORE_H
#include "Modbus_CRC.h"
#include "Modbus_USER.h"
#include "Modbus_hook.h"

/***************************************************************/
#define MBS_FRAME_SLAVER_ADDR (0)
#define MBS_FRAME_FUNCTION_CODE (1)
#define MBS_FRAME_START_ADD (2)
#define MBS_FRAME_OPT_NUM (4)
#define MBS_FRAME_WRITE_ONE_VALUE (4)
#define MBS_FRAME_BYTE_NUM (6)
/***************************************************************/
// #define	MBS_FALUT_ACK_ENABLE			        (1)
/***************************************************************/
#define MBS_FRAME_SLAVER_ADDR (0)
#define MBS_FRAME_FUNCTION_CODE (1)
#define MBS_FRAME_START_ADD (2)
#define MBS_FRAME_OPT_NUM (4)
#define MBS_FRAME_WRITE_ONE_VALUE (4)
#define MBS_FRAME_BYTE_NUM (6)
/***************************************************************/
typedef enum
{
  MBS_EX_NONE = 0x00,                 /*�޴���*/
  MBS_EX_ILLEGAL_FUNCTION = 0x01,     /*��֧�ֵĲ�������*/
  MBS_EX_ILLEGAL_DATA_ADDRESS = 0x02, /*�Ƿ��Ĳ�����ַ*/
  MBS_EX_ILLEGAL_DATA_VALUE = 0x03,   /*�Ƿ��Ĳ�����ֵ*/
  MBS_EX_SLAVE_DEVICE_FAILURE = 0x04, /*�豸����*/
  MBS_EX_ACKNOWLEDGE = 0x05,          /*ȷ��*/
  MBS_EX_SLAVE_BUSY = 0x06,           /*���豸æ*/
  MBS_EX_MEMORY_PARITY_ERROR = 0x08,  /*�洢��ż�Բ��?*/
  MBS_EX_GATEWAY_PATH_FAILED = 0x0A,  /*����������·��*/
  MBS_EX_GATEWAY_TGT_FAILED = 0x0B,   /*����Ŀ���豸��Ӧʧ��*/
} MBS_EX_STATE;
/***************************************************************/
typedef struct
{
  uint8 _rxBuff[MBS_PORT_RXBUFF_SIZE];
  uint8 _rxLen;
  uint8 _rxState;

  uint8 _txBuff[MBS_PORT_TXBUFF_SIZE];
  uint8 _txLen;
  uint8 _txState;

} MBS_PortFrameTypes;
extern MBS_PortFrameTypes MBS_Buf;

typedef struct
{
  uint16 coilAddr;
  uint16 pData;
} MBS_CoilValueTypes;

typedef struct
{
  MBS_CoilValueTypes *const _Value;
  uint16 _startAddr;
  uint16 _endAddr;
  uint16 const _num;
} MBS_CoilTypes;

typedef struct
{
  uint16 regAddr;
  uint16 pData;
} MBS_HoldRegValueTypes;

typedef struct
{
  MBS_HoldRegValueTypes *const _Value;
  uint16 _startAddr;
  uint16 _endAddr;
  uint16 const _num;
} MBS_HoldRegTypes;

#define MBS_NODE_NUM 2
typedef struct
{
  /* data */
  uint16 mbsAddr;
  MBS_CoilTypes *coil;
  MBS_HoldRegTypes *HoldReg;
} mbsNode_t;
extern mbsNode_t mbsNode[MBS_NODE_NUM];

#if MBS_FUNCTION_01_ENABLE
void MBS_Function01H(mbsNode_t mbsNode);
#endif
#if MBS_FUNCTION_03_ENABLE
void MBS_Function03H(mbsNode_t mbsNode);
#endif

#if MBS_FUNCTION_05_ENABLE
void MBS_Function05H(mbsNode_t mbsNode);
#endif
#if MBS_FUNCTION_06_ENABLE
void MBS_Function06H(mbsNode_t mbsNode);
#endif
#if MBS_FUNCTION_0F_ENABLE
void MBS_Function0FH(mbsNode_t mbsNode);
#endif
#if MBS_FUNCTION_10_ENABLE
void MBS_Function10H(mbsNode_t mbsNode);
#endif
/***************************************************************/
void MBS_CorePoll(void);
void MBS_CoreAnalyze(mbsNode_t mbsNode);
void MBS_PortSendAck(uint16 mbs_addr, MBS_EX_STATE ackCode);
void MBS_PortSendWithCRC(uint8 *buf, uint8 len);
void MBS_PhysicalSendBuff(uint8 *buf, uint8 len);
void MBS_DataSyn(void);

uint8 MBS_MemReadCoilState(MBS_CoilTypes *coilMap, uint16 coilAddr);
uint8 MBS_MemWriteCoilState(MBS_CoilTypes *coilMap, uint16 coilAddr, uint16 data);
uint16 MBS_MemWriteCoilsState(MBS_CoilTypes *coilMap, uint16 CoilAddr, uint16 Len, uint8 *data);
uint8 MBS_MemReadHoldRegValue(MBS_HoldRegTypes *HoldRegMap, uint16 regAddr, uint8 *Value, uint8 num);
uint8 MBS_MemWriteHoldRegValue(MBS_HoldRegTypes *HoldRegMap, uint16 regAddr, uint8 *Value, uint8 num);
#endif /* MODBUS_CORE_H */
