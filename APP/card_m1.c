#include "main.h"

#define RFCfgReg_Val 0x68
//********************************************//
// MF522寄存器定义
//********************************************//
// Page 0: Command and status
#define RFU00 0x00
#define CommandReg 0x01
#define ComIEnReg 0x02
#define DivIEnReg 0x03
#define ComIrqReg 0x04
#define DivIrqReg 0x05
#define ErrorReg 0x06
#define Status1Reg 0x07
#define Status2Reg 0x08
#define FIFODataReg 0x09
#define FIFOLevelReg 0x0A
#define WaterLevelReg 0x0B
#define ControlReg 0x0C
#define BitFramingReg 0x0D
#define CollReg 0x0E
#define ACDConfigReg 0x0F

// Page 1: Command
#define RFU10 0x10
#define ModeReg 0x11
#define TxModeReg 0x12
#define RxModeReg 0x13
#define TxControlReg 0x14
#define TxASKReg 0x15
#define TxSelReg 0x16
#define RxSelReg 0x17
#define RxThresholdReg 0x18
#define DemodReg 0x19
#define RFU1A 0x1A
#define RFU1B 0x1B
#define MfTxReg 0x1C
#define MfRxReg 0x1D
#define TypeBReg 0x1E
#define SerialSpeedReg 0x1F

// Page 2: Configuration
#define ACDConfigSelReg 0x20
#define CRCResultRegH 0x21
#define CRCResultRegL 0x22
#define RFU23 0x23
#define ModWidthReg 0x24
#define RFU25 0x25
#define RFCfgReg 0x26
#define GsNReg 0x27
#define CWGsPReg 0x28
#define ModGsPReg 0x29
#define TModeReg 0x2A
#define TPrescalerReg 0x2B
#define TReloadRegH 0x2C
#define TReloadRegL 0x2D
#define TCounterValueRegH 0x2E
#define TCounterValueRegL 0x2F

// Page 3: Test Register
#define RFU30 0x30
#define TestSel1Reg 0x31
#define TestSel2Reg 0x32
#define TestPinEnReg 0x33
#define TestPinValueReg 0x34
#define TestBusReg 0x35
#define AutoTestReg 0x36
#define VersionReg 0x37
#define AnalogTestReg 0x38
#define TestDAC1Reg 0x39
#define TestDAC2Reg 0x3A
#define TestADCReg 0x3B
#define RFU3C 0x3C
#define RFU3D 0x3D
#define RFU3E 0x3E
#define RFU3F 0x3F

/////////////////////////////////////////////////////////////////////
// MF522命令字
/////////////////////////////////////////////////////////////////////
#define PCD_IDLE 0x00       // 取消当前命令
#define PCD_AUTHENT 0x0E    // 验证密钥
#define PCD_RECEIVE 0x08    // 接收数据
#define PCD_TRANSMIT 0x04   // 发送数据
#define PCD_TRANSCEIVE 0x0C // 发送并接收数据
#define PCD_RESETPHASE 0x0F // 复位
#define PCD_CALCCRC 0x03    // CRC计算

/////////////////////////////////////////////////////////////////////
// 和MF522通讯时返回的错误代码
/////////////////////////////////////////////////////////////////////
#define MI_OK 0
#define MI_NOTAGERR 1
#define MI_ERR 2

/////////////////////////////////////////////////////////////////////
// MF522 FIFO长度定义
/////////////////////////////////////////////////////////////////////
#define DEF_FIFO_LENGTH 64 // FIFO size=64byte
#define MAXRLEN 18

/////////////////////////////////////////////////////////////////////
// Mifare_One卡片命令字
/////////////////////////////////////////////////////////////////////
#define PICC_REQIDL 0x26    // 寻天线区内未进入休眠状态
#define PICC_REQALL 0x52    // 寻天线区内全部卡
#define PICC_ANTICOLL1 0x93 // 防冲撞
#define PICC_ANTICOLL2 0x95 // 防冲撞
#define PICC_ANTICOLL3 0x97 // 防冲撞

#define PICC_AUTHENT1A 0x60 // 验证A密钥
#define PICC_AUTHENT1B 0x61 // 验证B密钥
#define PICC_READ 0x30      // 读块
#define PICC_WRITE 0xA0     // 写块
#define PICC_DECREMENT 0xC0 // 扣款
#define PICC_INCREMENT 0xC1 // 充值
#define PICC_RESTORE 0xC2   // 调块数据到缓冲区
#define PICC_TRANSFER 0xB0  // 保存缓冲区中数据
#define PICC_HALT 0x50      // 休眠
#define NSS_H LL_GPIO_SetOutputPin(Soft_SPI_CS_PORT, Soft_SPI_CS_PIN)
#define NSS_L LL_GPIO_ResetOutputPin(Soft_SPI_CS_PORT, Soft_SPI_CS_PIN)
#define SCK_H LL_GPIO_SetOutputPin(Soft_SPI_SCK_PORT, Soft_SPI_SCK_PIN)
#define SCK_L LL_GPIO_ResetOutputPin(Soft_SPI_SCK_PORT, Soft_SPI_SCK_PIN)
#define MOSI_H LL_GPIO_SetOutputPin(Soft_SPI_MOSI_PORT, Soft_SPI_MOSI_PIN)
#define MOSI_L LL_GPIO_ResetOutputPin(Soft_SPI_MOSI_PORT, Soft_SPI_MOSI_PIN)
#define READ_MISO LL_GPIO_IsInputPinSet(Soft_SPI_MISO_PORT, Soft_SPI_MISO_PIN)
#define M1_REST_H LL_GPIO_SetOutputPin(ID_RST_PORT, ID_RST_PIN)
#define M1_REST_L LL_GPIO_ResetOutputPin(ID_RST_PORT, ID_RST_PIN)

volatile int errcnt = 0;
unsigned char aaa = 0;
unsigned char SysM1_Val[5] = {0, 0, 0, 0, 0};

unsigned char SPI_RW_Data(uint8_t data)
{
    unsigned char readData = 0;
    unsigned char bitMask = 0x80;
    int i = 0;

    //    // NSS拉低，选中SPI设备
    //    GPIO_ResetBits(NSS_Port, NSS_PIN);

    // udelay(DELAY_CNT);
    // delay_temp(5);

    for (i = 0; i < 8; i++)
    {
        // Mode 0: CPOL = 0, CPHA = 0
        // CLK置低
        SCK_L;

        // 设置MOSI引脚
        if (data & bitMask)
        {
            MOSI_H;
        }
        else
        {
            MOSI_L;
        }

        // 延时约500ns以满足1MHz的SPI时钟要求
        // udelay(DELAY_CNT);
        // delay_temp(5);

        // CLK置高
        SCK_H;

        // 读取MISO引脚
        if (READ_MISO == 1)
        {
            readData |= bitMask;
        }

        // 延时约500ns以满足1MHz的SPI时钟要求
        // udelay(DELAY_CNT);
        // delay_temp(5);

        // 移位掩码
        bitMask >>= 1;
    }

    //    // NSS拉高，取消选中SPI设备
    //    GPIO_SetBits(NSS_Port, NSS_PIN);

    return readData;
}

void I_DP1322ES_IO_Write(unsigned char RegAddr, unsigned char value)
{
    NSS_H;
    RegAddr = (RegAddr & 0x3f) << 1; // code the first byte
    NSS_L;
    SPI_RW_Data(RegAddr); // write address first
    SPI_RW_Data(value);   // write value then
    NSS_H;
}

unsigned char I_DP1322ES_IO_Read(unsigned char RegAddr)
{
    unsigned char RegVal = 0;

    RegAddr = (RegAddr & 0x3f) << 1 | 0x80; // code the first byte

    NSS_L;
    SPI_RW_Data(RegAddr);    // write address first
    RegVal = SPI_RW_Data(0); // write value then
    NSS_H;
    return RegVal;
}

// DP1322ES_interfaces
void I_DP1322ES_ClearBitMask(unsigned char reg, unsigned char mask)
{
    char tmp = 0x00;
    tmp = I_DP1322ES_IO_Read(reg);
    I_DP1322ES_IO_Write(reg, tmp & ~mask); // clear bit mask
}

void I_DP1322ES_SetBitMask(unsigned char reg, unsigned char mask)
{
    char tmp = 0x00;
    tmp = I_DP1322ES_IO_Read(reg);
    I_DP1322ES_IO_Write(reg, tmp | mask); // set bit mask
}

/***********************************************
 * 函数名：CalulateCRC
 * 描述  ：用MF522计算CRC16函数
 * 输入  ：
 * 返回  : 无
 * 调用  ：外部调用
 **********************************************/
void CalulateCRC(unsigned char *pIndata, unsigned char len, unsigned char *pOutData)
{
    unsigned char i, n;
    I_DP1322ES_ClearBitMask(DivIrqReg, 0x04);
    I_DP1322ES_IO_Write(CommandReg, PCD_IDLE);
    I_DP1322ES_SetBitMask(FIFOLevelReg, 0x80);
    for (i = 0; i < len; i++)
    {
        I_DP1322ES_IO_Write(FIFODataReg, *(pIndata + i));
    }
    I_DP1322ES_IO_Write(CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do
    {
        n = I_DP1322ES_IO_Read(DivIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x04));
    pOutData[0] = I_DP1322ES_IO_Read(CRCResultRegL);
    pOutData[1] = I_DP1322ES_IO_Read(CRCResultRegH);
}

/***********************************************
 * 函数名：PcdAntennaOn
 * 描述  ：开启天线  每次启动或关闭天险发射之间应至少有1ms的间隔
 * 输入  ：无
 * 返回  : 无
 * 调用  ：外部调用
 **********************************************/
void PcdAntennaOn(void)
{
    unsigned char i;
    i = I_DP1322ES_IO_Read(TxControlReg);
    if (!(i & 0x03))
    {
        I_DP1322ES_SetBitMask(TxControlReg, 0x03); // bit1 bit0 置1
    }
}

/*===============================
 函数功能：读A卡初始化配置
 ================================*/
void PCD_DP1322ES_TypeA_Init(void)
{

    I_DP1322ES_ClearBitMask(Status2Reg, 0x08);
    // Reset baud rates
    I_DP1322ES_IO_Write(TxModeReg, 0x00);
    I_DP1322ES_IO_Write(RxModeReg, 0x00);
    // Reset ModWidthReg
    I_DP1322ES_IO_Write(ModWidthReg, 0x26);
    // RxGain:110,43dB by default;
    I_DP1322ES_IO_Write(RFCfgReg, RFCfgReg_Val);
    // When communicating with a PICC we need a timeout if something goes wrong.
    // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
    I_DP1322ES_IO_Write(TModeReg, 0x80);      // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    I_DP1322ES_IO_Write(TPrescalerReg, 0xa9); // TPreScaler = TModeReg[3..0]:TPrescalerReg
    I_DP1322ES_IO_Write(TReloadRegH, 0x03);   // Reload timer
    I_DP1322ES_IO_Write(TReloadRegL, 0xe8);   // Reload timer
    I_DP1322ES_IO_Write(TxASKReg, 0x40);      // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    I_DP1322ES_IO_Write(ModeReg, 0x3D);       // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
    I_DP1322ES_IO_Write(CommandReg, 0x00);    // Turn on the analog part of receiver
    // mdelay(10);
    PcdAntennaOn();
}

/**************************************************************************************************
 * 函数名：PcdComMF522
 * 描述  ：通过RC522和ISO14443卡通讯
 * 输入  ：Command[IN]:RC522命令字              pInData[IN]:通过RC522发送到卡片的数据    InLenByte[IN]:发送数据的字节长度
 *       ：pOutData[OUT]:接收到的卡片返回数据   pOutLenBit[OUT]:返回数据的位长度
 * 返回  : 无
 * 调用  ：外部调用
 *************************************************************************************************/
char PcdComMF522(unsigned char Command,
                 unsigned char *pInData,
                 unsigned char InLenByte,
                 unsigned char *pOutData,
                 unsigned int *pOutLenBit)
{
    char status = MI_ERR;
    unsigned char irqEn = 0x00;
    unsigned char waitFor = 0x00;
    unsigned char lastBits;
    unsigned char n;
    unsigned int i;
    switch (Command)
    {
    case PCD_AUTHENT:
        irqEn = 0x12;
        waitFor = 0x10;
        break;
    case PCD_TRANSCEIVE:
        irqEn = 0x77;
        waitFor = 0x30;
        break;
    default:
        break;
    }

    // I_DP1322ES_IO_Write(ComIEnReg,irqEn|0x80);
    I_DP1322ES_ClearBitMask(ComIrqReg, 0x80);
    I_DP1322ES_IO_Write(CommandReg, PCD_IDLE);
    I_DP1322ES_SetBitMask(FIFOLevelReg, 0x80);

    for (i = 0; i < InLenByte; i++)
    {
        I_DP1322ES_IO_Write(FIFODataReg, pInData[i]);
    }
    I_DP1322ES_IO_Write(CommandReg, Command);

    if (Command == PCD_TRANSCEIVE)
    {
        I_DP1322ES_SetBitMask(BitFramingReg, 0x80);
    }

    i = 600; // 根据时钟频率调整，操作M1卡最大等待时间25ms
    // i = 2000;
    // i = 20000;
    do
    {
        n = I_DP1322ES_IO_Read(ComIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitFor));
    I_DP1322ES_ClearBitMask(BitFramingReg, 0x80);

    if (i != 0)
    {
        aaa = I_DP1322ES_IO_Read(ErrorReg);

        if (!(I_DP1322ES_IO_Read(ErrorReg) & 0x1B))
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {
                status = MI_NOTAGERR;
            }
            if (Command == PCD_TRANSCEIVE)
            {
                n = I_DP1322ES_IO_Read(FIFOLevelReg);
                lastBits = I_DP1322ES_IO_Read(ControlReg) & 0x07;
                if (lastBits)
                {
                    *pOutLenBit = (n - 1) * 8 + lastBits;
                }
                else
                {
                    *pOutLenBit = n * 8;
                }
                if (n == 0)
                {
                    n = 1;
                }
                if (n > MAXRLEN)
                {
                    n = MAXRLEN;
                }
                for (i = 0; i < n; i++)
                {
                    pOutData[i] = I_DP1322ES_IO_Read(FIFODataReg);
                }
            }
        }
        else
        {
            status = MI_ERR;
        }
    }

    I_DP1322ES_SetBitMask(ControlReg, 0x80); // stop timer now
    I_DP1322ES_IO_Write(CommandReg, PCD_IDLE);
    return status;
}

/********************************************************************
 * 函数功能：寻卡
 * 参数说明: req_code[IN]:寻卡方式
 *                0x52 = 寻感应区内所有符合14443A标准的卡
 *                0x26 = 寻未进入休眠状态的卡
 *          pTagType[OUT]：卡片类型代码
 *                0x4400 = Mifare_UltraLight
 *                0x0400 = Mifare_One(S50)
 *                0x0200 = Mifare_One(S70)
 *                0x0800 = Mifare_Pro(X)
 *                0x4403 = Mifare_DESFire
 * 返回值 : 成功返回MI_OK
 ********************************************************************/
char PcdRequest(unsigned char req_code, unsigned char *pTagType)
{
    char status;
    unsigned int unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    I_DP1322ES_ClearBitMask(Status2Reg, 0x08);
    I_DP1322ES_IO_Write(BitFramingReg, 0x07);
    I_DP1322ES_SetBitMask(TxControlReg, 0x03);

    ucComMF522Buf[0] = req_code;

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf, &unLen);
    if ((status == MI_OK) && (unLen == 0x10))
    {
        *pTagType = ucComMF522Buf[0];
        *(pTagType + 1) = ucComMF522Buf[1];
    }
    else
    {
        status = MI_ERR;
    }

    return status;
}

void auto_reset(void)
{

    M1_REST_L;
    LL_mDelay(20);
    M1_REST_H;

    LL_mDelay(10);

    PCD_DP1322ES_TypeA_Init();
}

/////////////////////////////////////////////////////////////////////
// 功    能：防冲撞
// 参数说明: pSnr[OUT]:卡片序列号，4字节
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdAnticoll(unsigned char *pSnr, unsigned char anticollision_level)
{
    char status;
    unsigned char i, snr_check = 0;
    unsigned int unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    I_DP1322ES_ClearBitMask(Status2Reg, 0x08);
    I_DP1322ES_IO_Write(BitFramingReg, 0x00);
    I_DP1322ES_ClearBitMask(CollReg, 0x80);

    ucComMF522Buf[0] = anticollision_level;
    ucComMF522Buf[1] = 0x20;

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 2, ucComMF522Buf, &unLen);

    if (status == MI_OK)
    {
        for (i = 0; i < 4; i++)
        {
            *(pSnr + i) = ucComMF522Buf[i];
            snr_check ^= ucComMF522Buf[i];
        }
        if (snr_check != ucComMF522Buf[i])
        {
            status = MI_ERR;
        }
    }

    // tick_printf("orign : %02x %02x %02x %02x\r\n", *(pSnr),*(pSnr+1),*(pSnr+2),*(pSnr+3));

    I_DP1322ES_SetBitMask(CollReg, 0x80);
    return status;
}

char PcdSelect1(unsigned char *pSnr, unsigned char *sak)
{
    char status;
    unsigned char i;
    unsigned int unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i = 0; i < 4; i++)
    {
        ucComMF522Buf[i + 2] = *(pSnr + i);
        ucComMF522Buf[6] ^= *(pSnr + i);
    }
    CalulateCRC(ucComMF522Buf, 7, &ucComMF522Buf[7]);

    I_DP1322ES_ClearBitMask(Status2Reg, 0x08);

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf, &unLen);

    if ((status == MI_OK) && (unLen == 0x18))
    {
        *sak = ucComMF522Buf[0];
        status = MI_OK;
    }
    else
    {
        status = MI_ERR;
    }

    return status;
}

char PcdSelect2(unsigned char *pSnr, unsigned char *sak)
{
    char status;
    unsigned char i;
    unsigned int unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_ANTICOLL2;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i = 0; i < 4; i++)
    {
        ucComMF522Buf[i + 2] = *(pSnr + i);
        ucComMF522Buf[6] ^= *(pSnr + i);
    }
    CalulateCRC(ucComMF522Buf, 7, &ucComMF522Buf[7]);

    I_DP1322ES_ClearBitMask(Status2Reg, 0x08);

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf, &unLen);

    if ((status == MI_OK) && (unLen == 0x18))
    {
        *sak = ucComMF522Buf[0];
        status = MI_OK;
    }
    else
    {
        status = MI_ERR;
    }

    return status;
}

char PcdSelect3(unsigned char *pSnr, unsigned char *sak)
{
    char status;
    unsigned char i;
    unsigned int unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_ANTICOLL2;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i = 0; i < 4; i++)
    {
        ucComMF522Buf[i + 2] = *(pSnr + i);
        ucComMF522Buf[6] ^= *(pSnr + i);
    }
    CalulateCRC(ucComMF522Buf, 7, &ucComMF522Buf[7]);

    I_DP1322ES_ClearBitMask(Status2Reg, 0x08);

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf, &unLen);

    if ((status == MI_OK) && (unLen == 0x18))
    {
        *sak = ucComMF522Buf[0];
        status = MI_OK;
    }
    else
    {
        status = MI_ERR;
    }

    return status;
}

char PCD_DP1322ES_TypeA_GetUID(void)
{
    static unsigned char LastIdM1[5];
    static unsigned char Find_NewM1Card = 0;
    static unsigned char OverCntM1 = 0;
    unsigned char ATQA[2];
    unsigned char UID[12];
    unsigned char CardBufferFifo[16];
    unsigned char SAK = 0;
    unsigned char UID_complate1 = 0;
    unsigned char UID_complate2 = 0;
    // struct timeval tv_now;
    long tv_check = 0;
    int i, j, k, x1, x2;
    int xxx;
    unsigned int rawData = 0;
    // int timeout = 500000;//500ms
    int timeout = 200000; // 200ms

    if (PcdRequest(PICC_REQIDL, ATQA) != MI_OK) // 寻天线区内未进入休眠状态的卡，返回卡片类型 2字节
    {
        errcnt++;
        if (errcnt >= 8)
        {
            // clear auto reset for test
            // auto_reset();
            errcnt = 0;

            if (Find_NewM1Card == 1)
            {
                if (++OverCntM1 >= 1) /////////////超时处理，根据实际情况来更改
                {
                    OverCntM1 = 0;
                    LastIdM1[0] = 0;
                    LastIdM1[1] = 0;
                    LastIdM1[2] = 0;
                    LastIdM1[3] = 0;
                    LastIdM1[4] = 0;
                    Find_NewM1Card = 0;
                    SysM1_Val[0] = 0;
                    SysM1_Val[1] = 0;
                    SysM1_Val[2] = 0;
                    SysM1_Val[3] = 0;
                    SysM1_Val[4] = 0;
                }
            }
        }
        return 1;
    }
    else
    {
        tick_printf("Request3:ok  ATQA:%02x %02x\r\n", ATQA[0], ATQA[1]);
    }

    if (errcnt >= 8)
    {
        auto_reset();
        errcnt = 0;
        return 1;
    }
    // UID长度=4
    // Anticoll 冲突检测 level1
    if (PcdAnticoll(UID, PICC_ANTICOLL1) != MI_OK)
    {
        tick_printf("Anticoll1:fail\r\n");
        errcnt++;
        return 1;
    }
    else
    {
        if (PcdSelect1(UID, &SAK) != MI_OK)
        {
            tick_printf("Select1:fail\r\n");
            errcnt++;
            return 1;
        }
        else
        {
            tick_printf("Select1:ok  SAK1:%02x\r\n", SAK);
            if (SAK & 0x04)
            {
                UID_complate1 = 0;

                // UID长度=7
                if (UID_complate1 == 0)
                {
                    // Anticoll 冲突检测 level2
                    if (PcdAnticoll(UID + 4, PICC_ANTICOLL2) != MI_OK)
                    {
                        tick_printf("Anticoll2:fail\r\n");
                        errcnt++;
                        return 1;
                    }
                    else
                    {
                        if (PcdSelect2(UID + 4, &SAK) != MI_OK)
                        {
                            tick_printf("Select2:fail\r\n");
                            errcnt++;
                            return 1;
                        }
                        else
                        {
                            tick_printf("Select2:ok  SAK2:%02x\r\n", SAK);
                            if (SAK & 0x04)
                            {
                                UID_complate2 = 0;

                                // UID长度=10
                                if (UID_complate2 == 0)
                                {
                                    // Anticoll 冲突检测 level3
                                    if (PcdAnticoll(UID + 8, PICC_ANTICOLL3) != MI_OK)
                                    {
                                        tick_printf("Anticoll3:fail\r\n");
                                        errcnt++;
                                        return 1;
                                    }
                                    else
                                    {
                                        if (PcdSelect3(UID + 8, &SAK) != MI_OK)
                                        {
                                            tick_printf("Select3:fail\r\n");
                                            errcnt++;
                                            return 1;
                                        }
                                        else
                                        {
                                            tick_printf("Select3:ok  SAK3:%02x\r\n", SAK);
                                            if (SAK & 0x04)
                                            {
                                            }
                                            else
                                            {
                                                tick_printf("Anticoll3:ok  UID:%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
                                                            UID[1], UID[2], UID[3], UID[5], UID[6], UID[7], UID[8], UID[9], UID[10], UID[11]);
                                            }
                                        }
                                    }
                                }
                            }
                            else
                            {
                                UID_complate2 = 1;
                                tick_printf("Anticoll2:ok  UID:%02x %02x %02x %02x %02x %02x %02x\r\n",
                                            UID[1], UID[2], UID[3], UID[4], UID[5], UID[6], UID[7]);
                            }
                        }
                    }
                }
            }
            else
            {
                UID_complate1 = 1;
                errcnt = 0;
                rawData = UID[3] << 24 | UID[2] << 16 | UID[1] << 8 | UID[0];

                for (k = 0; k < 5; k++)
                {
                    x1 = rawData % 10;
                    rawData = rawData / 10;
                    x2 = rawData % 10;
                    rawData = rawData / 10;
                    CardBufferFifo[4 - k] = x2 * 16 + x1;
                }

                SysM1_Val[0] = CardBufferFifo[0];
                SysM1_Val[1] = CardBufferFifo[1];
                SysM1_Val[2] = CardBufferFifo[2];
                SysM1_Val[3] = CardBufferFifo[3];
                SysM1_Val[4] = CardBufferFifo[4];
                Find_NewM1Card = 1;
                OverCntM1 = 0;
                memcpy((uint8_t *)M1_Card_Info.ID, SysM1_Val, 5);
                M1_Card_Info.has_card = 1;
            }
        }
    }
    return 0;
}

void m1_init(void)
{
    MOSI_H;
    NSS_H;
    SCK_H;
    M1_REST_H;

    M1_REST_L;
    LL_mDelay(100);
    M1_REST_H;
    LL_mDelay(100);

    PCD_DP1322ES_TypeA_Init();
    LL_mDelay(100);
}
