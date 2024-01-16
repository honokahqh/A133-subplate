// #include <stdint.h>
#include "flash_driver.h"

static const char *TAG = "flash driver";

uint16_t offset1;
void flashDataSave(uint8_t type)
{
    // 单次写入至少32word
    switch (type)
    {
    default:
        break;
    }
}
void flashDataSyn()
{
}

static FlashStatus flash_unlock(void);
static FlashStatus flash_lock(void);
typedef void (*AppFunction)(void);

void JumpToApp()
{
    // 关闭所有中断
    __disable_irq();

    // 重新定位中断向量表到APP的起始地址
    SCB->VTOR = APP_ADDR;

    // 读取APP的堆栈指针（APP的第一个字）
    uint32_t appStack = *(volatile uint32_t *)APP_ADDR;

    // 读取APP的复位中断服务程序地址（APP的第二个字）
    uint32_t appEntry = *(volatile uint32_t *)(APP_ADDR + 4);

    // 设置堆栈指针
    __set_MSP(appStack);

    // 跳转到APP的入口点
    AppFunction app = (AppFunction)appEntry;
    app();
}

#define __RAM_FUNC __attribute__((section(".RamFunc")))

uint8_t FLASH_WaitForLastOperation(uint32_t Timeout)
{
    /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
       Even if the FLASH operation fails, the BUSY flag will be reset and an error
       flag will be set */
    uint32_t timeout = millis() + Timeout;
    /* Wait if any operation is ongoing */
    while (READ_BIT(FLASH->SR, FLASH_SR_BSY) != 0x00U)
    {
        if (millis() >= timeout)
        {
            return 0;
        }
    }

    /* Clear SR register */
    FLASH->SR = (1 << 0) | (1 << 4) | (1 << 15);

    return 1;
}

static __RAM_FUNC void FLASH_Program_Page(uint32_t Address, uint32_t *DataAddress)
{

    uint8_t index = 0;
    uint32_t dest = Address;
    uint32_t *src = DataAddress;
    uint32_t primask_bit;
    FLASH_WaitForLastOperation(100);
    SET_BIT(FLASH->CR, FLASH_CR_PG);
    /* Enter critical section: row programming should not be longer than 7 ms */
    primask_bit = __get_PRIMASK();
    __disable_irq();
    /* 32 words*/
    while (index < 32U)
    {
        *(uint32_t *)dest = *src;
        src += 1U;
        dest += 4U;
        index++;
        if (index == 31)
        {
            SET_BIT(FLASH->CR, FLASH_CR_PGSTRT);
        }
    }
    // while (FLASH->SR & FLASH_SR_BSY);
    FLASH_WaitForLastOperation(100);
    CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
    /* Exit critical section: restore previous priority mask */
    __set_PRIMASK(primask_bit);
}

uint32_t flash_write_buf[32];
void flash_program_bytes(uint32_t write_addr, uint8_t *data, uint32_t len)
{
    // Try to unlock the flash
    if (flash_unlock() != FLASH_SUCCESS)
        return;

    uint8_t *index = data;
    if (write_addr < APP_ADDR && write_addr + len > Flash_End_ADDR)
        return;

    while (len)
    {
        memset(flash_write_buf, 0, 128);
        if (len >= 128)
        {
            memcpy(flash_write_buf, index, 128);
            FLASH_Program_Page(write_addr, (uint32_t *)data);
            len -= 128;
            index += 128;
        }
        else if (len)
        {
            memcpy(flash_write_buf, index, len);
            FLASH_Program_Page(write_addr, (uint32_t *)data);
            len = 0;
        }
    }

    // Lock the flash after programming
    if (flash_lock() != FLASH_SUCCESS)
        return;
}

void flash_page_erase(uint32_t erase_addr)
{
    FLASH->PERTPE = 0x11940;
    FLASH->SMERTPE = 0x11940;
    FLASH->PRGTPE = 0x5DC0;
    FLASH->PRETPE = 0x12C0;
    while (FLASH->SR & FLASH_SR_BSY)
        ;
    if (flash_unlock() != FLASH_SUCCESS)
        return;
    SET_BIT(FLASH->CR, FLASH_CR_PER);
    SET_BIT(FLASH->CR, FLASH_CR_EOPIE);
    *(__IO uint32_t *)(erase_addr) = 0xFF;
    FLASH_WaitForLastOperation(100);
    if (READ_BIT(FLASH->SR, FLASH_SR_EOP) != 0x00U)
    {
        CLEAR_BIT(FLASH->SR, FLASH_SR_EOP);
    }
    CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
}

static FlashStatus flash_unlock(void)
{
    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0x00U)
    {
        /* Authorize the FLASH Registers access */
        WRITE_REG(FLASH->KEYR, FLASH_KEY1);
        WRITE_REG(FLASH->KEYR, FLASH_KEY2);

        if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0x00U)
        {
            LOG_I(TAG, "%s, flash unlock failed\r\n", __func__);
            return FLASH_ERROR_UNLOCK;
        }
    }
    return FLASH_SUCCESS;
}

static FlashStatus flash_lock(void)
{
    SET_BIT(FLASH->CR, FLASH_CR_LOCK);

    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) == 0x00u)
    {
        LOG_I(TAG, "%s, flash lock failed\r\n", __func__);
        return FLASH_ERROR_LOCK;
    }

    return FLASH_SUCCESS;
}
