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

uint32_t timeoutCount;
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
 * Task1_idcard_process
 * @brief id卡数据处理
 * @author Honokahqh
 * @date 2024-1-10
 */
static int Task1_idcard_process(struct pt *pt)
{
    static uint32_t sys_ms_last;
    PT_BEGIN(pt);
    while (1)
    {
        PT_WAIT_UNTIL(pt, ID_Card_Info.has_card);
        if (sys_ms > sys_ms_last + 200)
        {
            LOG_I(TAG, "Id card:%02x %02x %02x %02x %02x\r\n", ID_Card_Info.ID[0], ID_Card_Info.ID[1], ID_Card_Info.ID[2],
                  ID_Card_Info.ID[3], ID_Card_Info.ID[4]);
            CmdSend(Cmd_IDCard, 0x01, &ID_Card_Info.ID[1], 4);
        }
        memset(&ID_Card_Info, 0, sizeof(ID_Card_Info_t));
        sys_ms_last = sys_ms;
        ID_Card_Info.has_card = 0;
    }
    PT_END(pt);
}

/**
 * Task2_uart_process
 * @brief 串口数据处理
 * @author Honokahqh
 * @date 2024-1-10
 */
static int Task2_uart_process(struct pt *pt)
{
    PT_BEGIN(pt);
    while (1)
    {
        PT_WAIT_UNTIL(pt, uart_state.has_data);
        CmdProcess(uart_state.rx_buff, uart_state.rx_len);
        ymodem_data_process();
        uart_state.has_data = 0;
        uart_state.rx_len = 0;
    }
    PT_END(pt);
}

/**
 * Task3_100ms_process
 * @brief 100ms周期处理
 * @author Honokahqh
 * @date 2024-1-10
 */
extern volatile double temperature_average;
static int Task3_100ms_process(struct pt *pt)
{
    static uint32_t cnt_100ms = 0;
    PT_BEGIN(pt);
    while (1)
    {
        if (cnt_100ms % 10 == 0)
        {
            ymodem_timeout_process();
            LL_IWDG_ReloadCounter(IWDG);
        }
        if (cnt_100ms % 10 == 0)
        {
            ADC_start();
        }
        if (cnt_100ms % 100 == 0)
        {
            LOG_I(TAG, "temperature:%.2f\r\n", temperature_average);
            uint8_t adc_res[2];
            adc_res[0] = (uint16_t)temperature_average >> 8;
            adc_res[1] = (uint16_t)temperature_average;
            CmdSend(Cmd_Temper, 0x01, adc_res, 2);
        }
        if (timeoutCount > 6000)
        {
            NVIC_SystemReset();
        }
        Check_M1(); // 耗时20ms
        cnt_100ms++;
        timeoutCount++;
        PT_TIMER_DELAY(pt, 100);
    }
    PT_END(pt);
}

/**
 * Task4_rf433_process
 * @brief rf433接收数据处理
 * @author Honokahqh
 * @date 2024-1-10
 */
static int Task4_rf433_process(struct pt *pt)
{
    static uint32_t sys_ms_last;
    static uint8_t RF433_last[8], RF433_len_last;
    uint8_t i = 0;
    PT_BEGIN(pt);
    while (1)
    {
        PT_WAIT_UNTIL(pt, RF433_Info.has_data);
        LOG_I(TAG, "RF433_Info.type:%d ID:%02x %02x %02x %02x %02x %02x\r\n", RF433_Info.type, RF433_Info.ID[0], RF433_Info.ID[1], RF433_Info.ID[2],
              RF433_Info.ID[3], RF433_Info.ID[4], RF433_Info.ID[5]);
        for (i = 0; i < RF433_Info.data_len; i++)
        {
            if (RF433_Info.ID[i] != RF433_last[i])
                break;
        }
        if (i != RF433_Info.data_len || RF433_Info.data_len != RF433_len_last || sys_ms > sys_ms_last + 200)
        {
            if(RF433_Info.data_len == 6)
                CmdSend(Cmd_RF433, 0x02, RF433_Info.ID, RF433_Info.data_len);
            else
                CmdSend(Cmd_RF433, 0x01, RF433_Info.ID, RF433_Info.data_len);
        }
        memcpy(RF433_last, (uint8_t *)RF433_Info.ID, 6);
        RF433_len_last = RF433_Info.data_len;
        sys_ms_last = sys_ms;
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
        //        if (res == 2) // 更新完成
        //        {
        //            boot_to_app(APP_ADDR);
        //        }
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

/**
 * Check_M1
 * @brief 读M1卡 卡号
 * @author Honokahqh
 * @date 2024-1-10
 */
static void Check_M1()
{
    static uint8_t res_last, res = 0;
    uint8_t temp_data[11];
    // 周期spi读卡
    res = PCD_DP1322ES_TypeA_GetUID();
    if (res == 0xAA)
    {
        LOG_I(TAG, "new card:%02x %02x %02x %02x %02x\r\n", M1_Card_Info.ID[0], M1_Card_Info.ID[1], M1_Card_Info.ID[2],
              M1_Card_Info.ID[3], M1_Card_Info.ID[4]);
    }
    if (res != res_last)
    {
        res_last = res;
        if (res == 0xAA)
        {
            CmdSend(Cmd_M1Card, 0x01, M1_Card_Info.ID, 5);
        }
    }
}
