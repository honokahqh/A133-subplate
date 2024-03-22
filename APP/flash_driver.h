#ifndef __FLASH_DRIVER_H
#define __FLASH_DRIVER_H

#include "main.h"

#define BOOT_ADDR 0X08000000
#define APP_ADDR 0X08004000

#define Flash_IAP_PAGE 0x0800F000
#define Flash_Data_PAGE 0x800E000
#define Flash_End_ADDR 0x08008000
typedef enum {
    FLASH_SUCCESS,
    FLASH_ERROR_UNLOCK,
    FLASH_ERROR_LOCK,
    FLASH_TIMEOUT
} FlashStatus;

void JumpToApp(void);
void flash_program_bytes(uint32_t write_addr, uint8_t *data, uint32_t len);
void flash_write(uint32_t Address, uint8_t *data, uint32_t len);
void flash_write_page(uint32_t Address, uint32_t *DataAddress);
void flash_page_erase(uint32_t erase_addr);
void FlashSaveCallBellPara(uint8_t index);
void FlashSynCallBellPara();
#endif //
    