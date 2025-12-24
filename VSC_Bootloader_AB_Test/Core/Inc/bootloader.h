#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#include "stm32g4xx_hal.h"

// #define APP_START_ADDRESS 0x08007000UL

// #define APP_START_ADDR   0x08007000UL
#define APP_START_ADDRESS 0x08028000UL

#define APP_START_ADDR   0x08028000UL
#define FLASH_PAGE_SIZE  0x800U        // 2 KB
#define FLASH_BASE_ADDR  0x08000000U

void Bootloader_Main(void);
void Bootloader_LPUART_Init(void);
HAL_StatusTypeDef Bootloader_WriteFlash(uint32_t address, uint8_t *data, uint32_t length);
void JumpToApplication(uint32_t appAddr);
void Flash_Erase_App(void);
uint32_t GetPage(uint32_t addr);

#endif