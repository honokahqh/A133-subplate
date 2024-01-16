#include "main.h"

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static const char *TAG = "main";
/**
 * main
 * @brief
 * @author Honokahqh
 * @date 2023-12-16
 */
extern uint8_t wx_ok;
uint8_t data[128];
int main(void)
{
    /* 配置系统时钟 */
    SystemClock_Config();
//    IWDG_Config();
    gpio_init();
    uart2_init();
    timer1_init();
    m1_init();
    // ModbusIDMapInit();
    // ModbusRF433MapInit();
    LOG_I(TAG, "app run\r\n");
    System_Run();
}

/**
 * SystemClock_Config
 * @brief  system clock configration
 * @author Honokahqh
 * @date 2023-12-16
 */
static void SystemClock_Config(void)
{
    /* 使能HSI */
    LL_RCC_HSI_Enable();
    LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz);
    while (LL_RCC_HSI_IsReady() != 1)
    {
    }

    /* 设置 AHB 分频*/
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

    /* 配置HSISYS作为系统时钟源 */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
    {
    }

    /* 设置 APB1 分频*/
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_Init1msTick(24000000);

    /* 更新系统时钟全局变量SystemCoreClock(也可以通过调用SystemCoreClockUpdate函数更新) */
    LL_SetSystemCoreClock(24000000);

    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
}

/**
 * millis
 * @brief  获取系统运行时间，单位ms
 * @author Honokahqh
 * @date 2023-12-16
 */
unsigned int millis(void)
{
    return sys_ms;
}

/**
 * Error_Handler
 * @brief  This function is executed in case of error occurrence.
 * @author Honokahqh
 * @date 2023-12-16
 */
void Error_Handler(void)
{
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
/*******************************************************************************
**功能描述 ：输出产生断言错误的源文件名及行号
**输入参数 ：file：源文件名指针
**输入参数 ：line：发生断言错误的行号
**输出参数 ：
*******************************************************************************/
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Xinling *****END OF FILE****/
