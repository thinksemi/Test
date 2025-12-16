#include "bootloader.h"
#include "stm32g4xx_hal.h"

extern UART_HandleTypeDef hlpuart1;

void Bootloader_LPUART_Init(void)
{
    __HAL_RCC_LPUART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hlpuart1.Instance          = LPUART1;
    hlpuart1.Init.BaudRate     = 9600;
    hlpuart1.Init.WordLength   = UART_WORDLENGTH_8B;
    hlpuart1.Init.StopBits     = UART_STOPBITS_1;
    hlpuart1.Init.Parity       = UART_PARITY_NONE;
    hlpuart1.Init.Mode         = UART_MODE_TX_RX;
    hlpuart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    hlpuart1.Init.ClockPrescaler  = UART_PRESCALER_DIV1;
    hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&hlpuart1);
}

HAL_StatusTypeDef Bootloader_WriteFlash(uint32_t address, uint8_t *data, uint32_t length)
{
    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < length; i += 8)
    {
        uint64_t word = *(uint64_t*)(data + i);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, word) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
    }
    HAL_FLASH_Lock();
    return HAL_OK;
}

void JumpToApplication(void)
{
    uint32_t appStack = *(volatile uint32_t*)APP_START_ADDRESS;
    uint32_t appReset = *(volatile uint32_t*)(APP_START_ADDRESS + 4);

    typedef void (*pFunction)(void);
    pFunction appEntry = (pFunction)appReset;

    __disable_irq();
    __set_MSP(appStack);
    appEntry();
}

void Bootloader_Main(void)
{
    uint8_t command;
    const char msg[] = "Bootloader started\r\n";

    //Bootloader_LPUART_Init();

    if (HAL_UART_Receive(&hlpuart1, &command, 1, 2000) == HAL_OK)
    {
        if (command == 'a')
        {
            uint8_t buffer[10];
            uint32_t address = APP_START_ADDRESS;
            HAL_UART_Transmit(&hlpuart1,
                  (uint8_t *)msg,
                  sizeof(msg) - 1,
                  HAL_MAX_DELAY);

            while (1)
            {
                HAL_UART_Receive(&hlpuart1, buffer, sizeof(buffer), HAL_MAX_DELAY);
                Bootloader_WriteFlash(address, buffer, sizeof(buffer));
                address += sizeof(buffer);
            }
        }
    }
    //JumpToApplication();
}
