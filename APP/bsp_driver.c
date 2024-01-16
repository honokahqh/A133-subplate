#include "bsp_driver.h"

/**
 * gpio_init
 * @brief gpio_init
 * @author Honokahqh
 * @date 2023-12-16
 */
void gpio_init()
{
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

    // M1 io init
    LL_GPIO_SetPinMode(Soft_SPI_MISO_PORT, Soft_SPI_MISO_PIN, LL_GPIO_MODE_INPUT);

    LL_GPIO_SetPinMode(Soft_SPI_MOSI_PORT, Soft_SPI_MOSI_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(Soft_SPI_MOSI_PORT, Soft_SPI_MOSI_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(Soft_SPI_MOSI_PORT, Soft_SPI_MOSI_PIN, LL_GPIO_OUTPUT_PUSHPULL);

    LL_GPIO_SetPinMode(Soft_SPI_SCK_PORT, Soft_SPI_SCK_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(Soft_SPI_SCK_PORT, Soft_SPI_SCK_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(Soft_SPI_SCK_PORT, Soft_SPI_SCK_PIN, LL_GPIO_OUTPUT_PUSHPULL);

    LL_GPIO_SetPinMode(Soft_SPI_CS_PORT, Soft_SPI_CS_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(Soft_SPI_CS_PORT, Soft_SPI_CS_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(Soft_SPI_CS_PORT, Soft_SPI_CS_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetOutputPin(Soft_SPI_CS_PORT, Soft_SPI_CS_PIN);

    LL_GPIO_SetPinMode(Soft_SPI_RST_PORT, Soft_SPI_RST_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(Soft_SPI_RST_PORT, Soft_SPI_RST_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(Soft_SPI_RST_PORT, Soft_SPI_RST_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetOutputPin(Soft_SPI_RST_PORT, Soft_SPI_RST_PIN);

    // id io init
    // gpioA pin4 上升沿中断
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_UP);
    LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
    EXTI_InitStruct.Line = LL_EXTI_LINE_4;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
    LL_EXTI_Init(&EXTI_InitStruct);
    NVIC_SetPriority(EXTI4_15_IRQn, 0);
    NVIC_EnableIRQ(EXTI4_15_IRQn);

    LL_GPIO_SetPinMode(ID_RST_PORT, ID_RST_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(ID_RST_PORT, ID_RST_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(ID_RST_PORT, ID_RST_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetOutputPin(ID_RST_PORT, ID_RST_PIN);

    LL_GPIO_SetPinMode(ID_PWM_PORT, ID_PWM_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(ID_PWM_PORT, ID_PWM_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(ID_PWM_PORT, ID_PWM_PIN, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_0_7(ID_PWM_PORT, ID_PWM_PIN, 13);

    // call bell io init
    LL_GPIO_SetPinMode(CallBell_Enable_PORT, CallBell_Enable_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(CallBell_Enable_PORT, CallBell_Enable_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(CallBell_Enable_PORT, CallBell_Enable_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_ResetOutputPin(CallBell_Enable_PORT, CallBell_Enable_PIN);

    LL_GPIO_SetPinMode(CallBell_Data_PORT, CallBell_Data_PIN, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(CallBell_Data_PORT, CallBell_Data_PIN, LL_GPIO_PULL_UP);
    EXTI_InitStruct.Line = LL_EXTI_LINE_0;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
    LL_EXTI_Init(&EXTI_InitStruct);
    NVIC_SetPriority(EXTI0_1_IRQn, 0);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}

uart_state_t uart_state;

/**
 * uart2_init
 * @brief uart2初始化
 * @author Honokahqh
 * @date 2023-12-16
 */
void uart2_init()
{
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

    LL_GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF4_USART2;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
    GPIO_InitStruct.Alternate = LL_GPIO_AF9_USART2;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    NVIC_SetPriority(USART2_IRQn, 0);
    NVIC_EnableIRQ(USART2_IRQn);

    LL_USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.BaudRate = 921600;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART2, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(USART2);

    LL_USART_EnableIT_RXNE(USART2);

    LL_USART_Enable(USART2);
}

/**
 * uart_send_data
 * @brief uart 485发送数据
 * @author Honokahqh
 * @date 2023-12-16
 */
void uart_send_data(uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        USART2->DR = data[i];
        while (!(USART2->SR & 0x80))
            ;
    }
    while (!(USART2->SR & 0x40))
        ;
}

/**
 * USART2_IRQHandler
 * @brief 串口接收中断处理
 * @author Honokahqh
 * @date 2023-12-16
 */
void USART2_IRQHandler()
{
    if (USART2->SR & USART_SR_RXNE)
    {
        uart_state.rx_buff[uart_state.rx_len++] = USART2->DR;
        uart_state.IDLE = 1;
    }
}

/**
 * timer1_init
 * @brief 1ms定时器初始化
 * @author Honokahqh
 * @date 2023-12-16
 */
volatile uint32_t sys_ms, sys_50us_id;
volatile uint32_t sys_50us_433;
void timer1_init()
{
    LL_APB1_GRP2_EnableClock(RCC_APBENR2_TIM1EN);

    /*配置TIM1*/
    LL_TIM_InitTypeDef TIM1CountInit = {0};

    /***********************************************
    ** 输入时钟：    8000000
    ** 计数模式：    向上计数
    ** 时钟预分频：  8000
    ** 自动重装载值：500
    ** 重复计数值：  0
    ************************************************/
    TIM1CountInit.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM1CountInit.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM1CountInit.Prescaler = 0;
    TIM1CountInit.Autoreload = 192;
    TIM1CountInit.RepetitionCounter = 0;

    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;
    LL_TIM_OC_StructInit(&TIM_OC_InitStruct);
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.CompareValue = 96;

    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    /*初始化TIM1*/
    LL_TIM_Init(TIM1, &TIM1CountInit);
    TIM1->BDTR = 0x8000;  // 输出使能
    TIM1->CCMR1 = 0x0068; // PWM1模式 开启CCR1预装载

    /*使能TIM1计数器*/
    LL_TIM_EnableCounter(TIM1);
    TIM1->CCER = 1; // 启动pwm输出

    LL_APB1_GRP1_EnableClock(RCC_APBENR1_TIM3EN);

    TIM1CountInit.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM1CountInit.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM1CountInit.Prescaler = 40 - 1;
    TIM1CountInit.Autoreload = 29 - 1;
    TIM1CountInit.RepetitionCounter = 0;

    /*初始化TIM1*/
    LL_TIM_Init(TIM3, &TIM1CountInit);

    /*使能UPDATE中断*/
    LL_TIM_EnableIT_UPDATE(TIM3);

    /*使能TIM1计数器*/
    LL_TIM_EnableCounter(TIM3);

    /*开启UPDATE中断请求*/
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 0);
}
#define get_433_io_state() (GPIOA->IDR & CallBell_Data_PIN)

volatile RF433_t RF433; // 全局RF433结构体实例
volatile RF433_Info_t RF433_Info;

// 初始化不同类型的呼叫铃参数
const BellParameters bellParams[] = {
    // Start Min Max, Low Min Max, High Min Max
    {0xCA - 0x10, 0xCA + 0x10, 0x06 - 3, 0x06 + 4, 0x13 - 3, 0x13 + 4},   // 迅铃
    {0x145 - 0x10, 0x145 + 0x10, 0x0B - 3, 0x0B + 4, 0x20 - 3, 0x20 + 4}, // 索易
    // 可以在此添加更多的参数
};

void RT1587_decode(void)
{
    RF433.lowLevelCount++;
    if (RF433.NeedSample) // 延迟50us~100us进行采样
    {
        RF433.NeedSample--;
        if (!RF433.NeedSample)
        {
            if (!get_433_io_state())
            {
                RF433.lowLevelCount = 0; // 计时低电平时间
            }
            else
            {
                RF433.time[RF433.time_index++] = RF433.lowLevelCount;
                for (int i = 0; i < sizeof(bellParams) / sizeof(BellParameters); i++)
                {
                    if ((bellParams[i].startMin < RF433.lowLevelCount) && (RF433.lowLevelCount < bellParams[i].startMax))
                    {
                        memset((uint8_t *)&RF433, 0, sizeof(RF433_t));
                        RF433.type = (BellType)i; // 设置当前的呼叫铃类型
                        RF433.isBusy = 1;
                        return;
                    }
                }
                if (RF433.isBusy && (bellParams[RF433.type].lowMin) < RF433.lowLevelCount && RF433.lowLevelCount < (bellParams[RF433.type].lowMax))
                {
                    RF433.id[RF433.bit / 8] |= 1 << (7 - (RF433.bit % 8));
                    RF433.bit++;
                }
                else if (RF433.isBusy && (bellParams[RF433.type].highMin) < RF433.lowLevelCount && RF433.lowLevelCount < (bellParams[RF433.type].highMax))
                {
                    RF433.id[RF433.bit / 8] &= ~(1 << (7 - (RF433.bit % 8)));
                    RF433.bit++;
                }
                else if (RF433.isBusy)
                { // 干扰码
                    memset((uint8_t *)&RF433, 0, sizeof(RF433_t));
                    return;
                }
                if (RF433.bit == 24)
                {
                    RF433_Info.has_data = 1;
                    RF433_Info.type = RF433.type;
                    memcpy((uint8_t *)RF433_Info.ID, (uint8_t *)RF433.id, 3);
                    memset((uint8_t *)&RF433, 0, sizeof(RF433_t));
                }
            }
        }
    }
}
void EXTI0_1_IRQHandler()
{
    if (LL_EXTI_ReadFlag(LL_EXTI_LINE_0))
    { // 清除中断
        LL_EXTI_ClearFlag(LL_EXTI_LINE_0);
        RF433.NeedSample = 2;
    }
}

void TIM3_IRQHandler()
{
    LL_TIM_ClearFlag_UPDATE(TIM3);
    sys_50us_id++;
    RT1587_decode();
}

int checkCardData(uint8_t *data)
{
    // 行校验
    for (int row = 0; row < 10; row++)
    {
        int rowSum = 0;
        for (int bit = 0; bit < 4; bit++)
        {
            rowSum += (data[(row * 5 + bit + 9) / 8] >> ((row * 5 + bit + 9) % 8)) & 1;
        }
        rowSum += (data[(row * 5 + 4 + 9) / 8] >> ((row * 5 + 4 + 9) % 8)) & 1; // 加上Px
        if (rowSum % 2 != 0)
        {
            return 0; // 行校验失败
        }
    }

    // 列校验
    for (int col = 0; col < 4; col++)
    {
        int colSum = 0;
        for (int row = 0; row < 10; row++)
        {
            colSum += (data[(row * 5 + col + 9) / 8] >> ((row * 5 + col + 9) % 8)) & 1;
        }
        colSum += (data[(59 + col) / 8] >> ((59 + col) % 8)) & 1; // 加上PCx
        if (colSum % 2 != 0)
        {
            return 0; // 列校验失败
        }
    }

    //
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if ((data[((i * 5) + j + 9) / 8] & (1 << ((i * 5 + j + 9) % 8))))
                ID_Card_Info.ID[(i * 4 + j) / 8] |= 1 << ((i * 4 + j) % 8);
        }
    }
    // 所有校验通过
    return 1;
}
/**
 * EXTI4_15_IRQHandler
 * @brief 根据IO电平切换时间计算0 or 1
 * @author Honokahqh
 * @date 2023-12-16
 */
volatile ID_Card_t ID_Card;           // 缓存，读完完整卡号后清零
volatile ID_Card_Info_t ID_Card_Info; //  读卡号后,由APP进行校验
volatile ID_Card_Info_t M1_Card_Info; //  读卡号后,由APP进行校验
#define Half_Min 3                    // 150us
#define Half_Max 7                    // 350us
#define Full_Min 8                    // 400us
#define Full_Max 13                   // 650us
uint8_t get_id_io_state()
{
    if (GPIOA->IDR & ID_DATA_PIN)
        return 0;
    else
        return 1;
}
void id_card_decode()
{
    ID_Card.time[ID_Card.time_index++] = sys_50us_id;
    // get start frame
    if (ID_Card.state == ID_IDLE)
    {
        if (sys_50us_id >= Full_Min && sys_50us_id <= Full_Max && get_id_io_state()) // 必然是一个翻转信号且为1
        {
            ID_Card.state = ID_START;
            ID_Card.ID[0] = 1;
        }
    }
    else if (ID_Card.state == ID_START)
    {
        if (sys_50us_id >= Half_Min && sys_50us_id <= Half_Max) // 16个空跳
        {
            ID_Card.count++;
            if (ID_Card.count == 16)
            {
                ID_Card.state = ID_DATA;
                ID_Card.count = 0;
                ID_Card.ID[0] = 0xFF;
                ID_Card.ID[1] = 1;
                ID_Card.bit = 9;
            }
        }
        else
        {
            memset((uint8_t *)&ID_Card, 0, sizeof(ID_Card_t));
        }
    }
    else if (ID_Card.state == ID_DATA) // (4 + 1)*10 + 4 + 1
    {
        if (sys_50us_id >= Full_Min && sys_50us_id <= Full_Max)
        {
            ID_Card.value = get_id_io_state();
            ID_Card.ID[ID_Card.bit / 8] |= (ID_Card.value << (ID_Card.bit % 8));
            ID_Card.bit++;
            ID_Card.count = 0;
        }
        else if (sys_50us_id >= Half_Min && sys_50us_id <= Half_Max)
        {
            ID_Card.count++;
            if (ID_Card.count % 2 == 0)
            {
                ID_Card.value = get_id_io_state();
                ID_Card.ID[ID_Card.bit / 8] |= (ID_Card.value << (ID_Card.bit % 8));
                ID_Card.bit++;
            }
        }
        else
        {
            memset((uint8_t *)&ID_Card, 0, sizeof(ID_Card_t));
        }
    }
    if (ID_Card.bit == 64)
    {
        if (checkCardData((uint8_t *)ID_Card.ID)) // !check的时间超过了500us,导致下一次中断进不来
        {
            ID_Card_Info.has_card = 1;
        }
        memset((uint8_t *)&ID_Card, 0, sizeof(ID_Card_t));
    }
    sys_50us_id = 0;
}
void EXTI4_15_IRQHandler()
{
    if (LL_EXTI_ReadFlag(LL_EXTI_LINE_4))
    { // 清除中断
        LL_EXTI_ClearFlag(LL_EXTI_LINE_4);
        id_card_decode();
    }
}

/**
 * IWDG_Config
 * @brief 看门狗初始化
 * @author Honokahqh
 * @date 2023-12-16
 */
void IWDG_Config(void)
{
    /* 使能LSI */
    LL_RCC_LSI_Enable();
    while (LL_RCC_LSI_IsReady() == 0U)
    {
        ;
    }

    /* 使能IWDG */
    LL_IWDG_Enable(IWDG);
    /* 开启写权限 */
    LL_IWDG_EnableWriteAccess(IWDG);
    /* 设置IWDG分频 */
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_32); // T=1MS
    /* 设置喂狗事件*/
    LL_IWDG_SetReloadCounter(IWDG, 3000); // 1ms*1000=1s
    /* IWDG初始化*/
    while (LL_IWDG_IsReady(IWDG) == 0U)
    {
        ;
    }
    /*喂狗*/
    LL_IWDG_ReloadCounter(IWDG);
}
