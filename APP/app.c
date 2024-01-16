#include "app.h"

static void Check_M1(void);

static struct pt idcard_pt;
static struct pt rf433_pt;
static struct pt uart_pt;
static struct pt pt100ms;

static int Task1_idcard_process(struct pt *pt);
static int Task2_uart_process(struct pt *pt);
static int Task3_100ms_process(struct pt *pt);
static int Task4_rf433_process(struct pt *pt);

static void ymodem_data_process(void);
static void ymodem_timeout_process(void);

static const char *TAG = "app";
void System_Run()
{
    PT_INIT(&idcard_pt);
    PT_INIT(&uart_pt);
    PT_INIT(&pt100ms);
    PT_INIT(&rf433_pt);
    while (1)
    {
        Task1_idcard_process(&idcard_pt);
        Task2_uart_process(&uart_pt);
        Task3_100ms_process(&pt100ms);
        Task4_rf433_process(&rf433_pt);
    }
}

/**
 * Task1_infrared_process
 * @brief 红外数据处理
 * @author Honokahqh
 * @date 2023-12-16
 */
static int Task1_idcard_process(struct pt *pt)
{
    uint8_t temp_data[9];
    uint16_t crc16;
    PT_BEGIN(pt);
    while (1)
    {
        PT_WAIT_UNTIL(pt, ID_Card_Info.has_card);
        LOG_I(TAG, "Id card:%02x %02x %02x %02x %02x\r\n", ID_Card_Info.ID[0], ID_Card_Info.ID[1], ID_Card_Info.ID[2],
                    ID_Card_Info.ID[3], ID_Card_Info.ID[4]);
        temp_data[0] = 0x01;
        temp_data[1] = 5;
        temp_data[2] = ID_Card_Info.ID[0];
        temp_data[3] = ID_Card_Info.ID[1];
        temp_data[4] = ID_Card_Info.ID[2];
        temp_data[5] = ID_Card_Info.ID[3];
        temp_data[6] = ID_Card_Info.ID[4];
        crc16 = mb_crc16(temp_data, 7);
        temp_data[7] = crc16 >> 8;
        temp_data[8] = crc16;
        uart_send_data(temp_data, 9);
		memset((uint8_t *)&ID_Card_Info, 0, sizeof(ID_Card_Info_t));
    }
    PT_END(pt);
}

/**
 * Task2_uart_process
 * @brief 串口数据处理 OTA/MBS
 * @author Honokahqh
 * @date 2023-12-16
 */
static int Task2_uart_process(struct pt *pt)
{
    PT_BEGIN(pt);
    while (1)
    {
        PT_WAIT_UNTIL(pt, uart_state.has_data);
        ymodem_data_process();
        uart_state.has_data = 0;
        uart_state.rx_len = 0;
    }
    PT_END(pt);
}

/**
 * Task3_100ms_process
 * @brief 100ms周期处理,mbs通讯超时/喂狗/红外通讯超时
 * @author Honokahqh
 * @date 2023-12-16
 */
extern volatile uint16_t temperature;
static int Task3_100ms_process(struct pt *pt)
{
    static uint32_t cnt_100ms = 0;
    PT_BEGIN(pt);
    while (1)
    {
		if(cnt_100ms++ % 10 == 0 )
        {
            ymodem_timeout_process();
            LL_IWDG_ReloadCounter(IWDG);
        }
        Check_M1();
        cnt_100ms++;
        PT_TIMER_DELAY(pt, 100);
    }
    PT_END(pt);
}
static int Task4_rf433_process(struct pt *pt)
{
    uint16 crc16;
    uint8_t temp_data[7];
    PT_BEGIN(pt);
    while (1)
    {
        PT_WAIT_UNTIL(pt, RF433_Info.has_data);
        if(RF433_Info.type)
            LOG_I(TAG, "new bell ID:%02x %02x %02x\r\n", RF433_Info.ID[0], RF433_Info.ID[1], RF433_Info.ID[2]);
        else
            LOG_I(TAG, "bell ID:%02x %02x %02x\r\n", RF433_Info.ID[0], RF433_Info.ID[1], RF433_Info.ID[2]);
        temp_data[0] = 0x03;
        temp_data[1] = 3;
        temp_data[2] = RF433_Info.ID[0];
        temp_data[3] = RF433_Info.ID[1];
        temp_data[4] = RF433_Info.ID[2];
        crc16 = mb_crc16(temp_data, 5);
        temp_data[5] = crc16 >> 8;
        temp_data[6] = crc16;
//        uart_send_data(temp_data, 7);
        RF433_Info.has_data = 0;
    }
    PT_END(pt);
}

/**
 * ymodem_data_process
 * @brief ota协议 数据分析处理
 * @author Honokahqh
 * @date 2023-12-16
 */
void ymodem_data_process()
{
    static uint8_t IAP_Key[] = {0xff, MBS_Addr, 0x50, 0xA5, 0x5A, 0x38, 0x26, 0xFE};
    if (ymodem_session.state == YMODEM_STATE_IDLE)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            if (uart_state.rx_buff[i] != IAP_Key[i])
                return;
        }
        return;
    }
#if !Is_APP
    if (ymodem_session.state != YMODEM_STATE_IDLE)
    {
        uint8_t data[2];
        uint16_t len;
        int res = ymodem_packet_analysis(uart_state.rx_buff, uart_state.rx_len, data, &len);
        LL_mDelay(5);
        if (len > 0)
            uart_send_data(data, len);
        // if (res == 2) // 更新完成
        // {
        //     boot_to_app(APP_ADDR);
        // }
    }
#endif
}

/**
 * ymodem_timeout_process
 * @brief IAP通讯超时处理
 * @author Honokahqh
 * @date 2023-10-07
 */
static void ymodem_timeout_process()
{
#if !Is_APP
    if (ymodem_session.state != YMODEM_STATE_IDLE)
    { // 非lora模式下,OTA超时检测在此处而非Slaver_Period_1s
        uint8_t temp[2];
        ymodem_session.timeout++;
        if (ymodem_session.timeout > 3)
        {
            ymodem_session.error_count++;
            temp[0] = NAK;
            uart_send_data(temp, 1);
        }
        if (ymodem_session.error_count > 5)
        {
            memset(&ymodem_session, 0, sizeof(ymodem_session_t));
            temp[0] = CAN;
            temp[1] = CAN;
            uart_send_data(temp, 2);
            NVIC_SystemReset();
        }
    }
#endif
}

static void Check_M1()
{
    // 周期spi读卡
    PCD_DP1322ES_TypeA_GetUID();

    uint8_t temp_data[9];
    if (M1_Card_Info.has_card)
    {
        LOG_I(TAG, "M1 card:%02x %02x %02x %02x %02x\r\n", M1_Card_Info.ID[0], M1_Card_Info.ID[1], M1_Card_Info.ID[2],
                    M1_Card_Info.ID[3], M1_Card_Info.ID[4]);
        temp_data[0] = 0x02;
        temp_data[1] = 5;
        temp_data[2] = M1_Card_Info.ID[0];
        temp_data[3] = M1_Card_Info.ID[1];
        temp_data[4] = M1_Card_Info.ID[2];
        temp_data[5] = M1_Card_Info.ID[3];
        temp_data[6] = M1_Card_Info.ID[4];
        uint16_t crc16 = mb_crc16(temp_data, 7);
        temp_data[7] = crc16 >> 8;
        temp_data[8] = crc16;
        uart_send_data(temp_data, 9);
        M1_Card_Info.has_card = 0;
    }

    // static uint8_t new_card_flag;
    // if (M1_Card_Info.has_card)
    // {
    //     LOG_I(TAG, "M1 card:%02x %02x %02x %02x %02x\r\n", M1_Card_Info.ID[0], M1_Card_Info.ID[1], M1_Card_Info.ID[2],
    //                 M1_Card_Info.ID[3], M1_Card_Info.ID[4]);
    //     if (mbsNode[MBS_ID].HoldReg->_Value[REG_M1].pData != M1_Card_Info.ID[0] || mbsNode[MBS_ID].HoldReg->_Value[REG_M1 + 1].pData != M1_Card_Info.ID[1] || mbsNode[MBS_ID].HoldReg->_Value[REG_M1 + 2].pData != M1_Card_Info.ID[2] || mbsNode[MBS_ID].HoldReg->_Value[REG_M1 + 3].pData != M1_Card_Info.ID[3] || mbsNode[MBS_ID].HoldReg->_Value[REG_M1 + 4].pData != M1_Card_Info.ID[4] || new_card_flag == 0)
    //     {
    //         mbsNode[MBS_ID].HoldReg->_Value[REG_M1].pData = M1_Card_Info.ID[0];
    //         mbsNode[MBS_ID].HoldReg->_Value[REG_M1 + 1].pData = M1_Card_Info.ID[1];
    //         mbsNode[MBS_ID].HoldReg->_Value[REG_M1 + 2].pData = M1_Card_Info.ID[2];
    //         mbsNode[MBS_ID].HoldReg->_Value[REG_M1 + 3].pData = M1_Card_Info.ID[3];
    //         mbsNode[MBS_ID].HoldReg->_Value[REG_M1 + 4].pData = M1_Card_Info.ID[4];
    //         mbsNode[MBS_ID].coil->_Value[COIL_M1].pData = 1;
    //         new_card_flag = 1;
    //     }
    //     memset((uint8_t *)&M1_Card_Info, 0, sizeof(ID_Card_Info_t));
    // }
    // else
    // {
    //     new_card_flag = 0;
    // }
}
