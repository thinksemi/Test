#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#include "stm32g4xx_hal.h"

#define APP_START_ADDRESS 0x08007000UL

void Bootloader_Main(void);
void Bootloader_LPUART_Init(void);
HAL_StatusTypeDef Bootloader_WriteFlash(uint32_t address, uint8_t *data, uint32_t length);
void JumpToApplication(void);

#endif