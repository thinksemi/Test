/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bootloader.h"
#include "stm32g4xx_hal_uart.h"
#include <stdio.h>
#include <unistd.h>
#include "mbedtls/sha256.h"
#include "mbedtls/ecp.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/bignum.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//////////////secure boot header definition//////////////
typedef struct
{
    uint32_t magic;          // 0xDEADBEEF
    uint32_t fw_size;
    // uint32_t fw_version;
    uint8_t  hash[32];       // SHA-256
    uint8_t  signature[64];  // ECDSA
    uint8_t  reserved[24];   // Padding / future use
} fw_header_t;


//////////////END secure boot header definition//////////////

typedef struct {
    uint32_t magic;
    uint32_t active_app;  // 1 or 2
} boot_flag_t;

typedef void (*pFunction)(void);

int ecdsa_verify_signature(uint8_t *hash, uint8_t *signature);



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BOOTLOADER_ADDR   0x08000000
#define APP1_ADDR         0x08008000
#define APP2_ADDR         0x08028000
#define APP3_ADDR         0x08007000
#define BOOT_FLAG_ADDR    0x0807F800   // last page

#define APPT_ADDR         0x08028080  // application start after header

#define BOOT_MAGIC 0xB007B007

#define PUB_KEY_LEN 65
#define MAX_APP_SIZE 0x00020000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
__attribute__((section(".rodata"))) 
const uint8_t public_key[PUB_KEY_LEN] = {0x04,0Xe4,0Xd2,0X33,0Xc9,0X11,0X1d,0X4d,0X1d,0Xa6,0X5f,0X93,0X2f,0X9b,0X21,0X11,0Xc1,0X3a,0X51,0Xdb,0X81,0X96,0X6d,0X38,0Xc0,0Xfa,0Xcc,0X81,0X91,0Xa5,0X9a,0X50,0X03,0X41,0Xac,0X7b,0Xe9,0X22,0X21,0X1f,0X54,0X86,0X46,0X4f,0X93,0Xef,0Xa1,0X35,0X33,0X11,0X56,0X14,0X19,0Xb6,0Xd9,0X1f,0X27,0Xd8,0Xeb,0Xf2,0X16,0X84,0X03,0X72,0Xc8};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */
boot_flag_t* bootFlag = (boot_flag_t*)BOOT_FLAG_ADDR;
uint32_t app_address = 0X00000000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void SetActiveApp(uint32_t);
uint32_t GetActiveApp(void);
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit(&hlpuart1,
                                 (uint8_t *)"Ready from STM32\r\n",
                                 sizeof("Ready from STM32\r\n") - 1,
                                 HAL_MAX_DELAY);
  
                                 
  // JumpToApplication(APPT_ADDR);
  Bootloader_Main();
  // check_headers();
   SetActiveApp(2);
   app_address=GetActiveApp();
  //  JumpToApplication(APP2_ADDR);
    // JumpToApplication(APP3_ADDR);

   if(app_address==1)
   {
       JumpToApplication(APP1_ADDR); // correct code
   }
   else if (app_address==2)
   {
       JumpToApplication(APP2_ADDR); // correct code
   }
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //  Bootloader_Main();
  // check_headers();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      //  Bootloader_Main();
      
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint32_t GetActiveApp(void)
{
    if (bootFlag->magic != BOOT_MAGIC)
        return 1;  // default

    return bootFlag->active_app;
}
void SetActiveApp(uint32_t app)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Page      = FLASH_PAGE_NB - 1,
        .NbPages   = 1
    };
    uint32_t error;
    HAL_FLASHEx_Erase(&erase, &error);

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                      BOOT_FLAG_ADDR,
                      BOOT_MAGIC | ((uint64_t)app << 32));

    HAL_FLASH_Lock();
}

uint8_t IsAppValid(uint32_t appAddr)
{
    uint32_t sp = *(uint32_t*)appAddr;
    uint32_t reset = *(uint32_t*)(appAddr + 4);

    if ((sp & 0x2FFE0000) != 0x20000000)
        return 0;

    if (reset < appAddr || reset > (appAddr + 0x20000))
        return 0;

    return 1;
}


void JumpToApplication(uint32_t appAddr)
{
    uint32_t jumpAddress = *(uint32_t*)(appAddr + 4);
    pFunction Jump = (pFunction)jumpAddress;

    __disable_irq();

    // Deinit peripherals used by bootloader
    HAL_RCC_DeInit();
    HAL_DeInit();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    SCB->VTOR = appAddr;

    __set_MSP(*(uint32_t*)appAddr);

    __enable_irq();

    Jump();  // never returns
}

void check_headers (void)
{
  // fw_header_t *hdr = (fw_header_t *)0x08008000;
    fw_header_t *hdr = (fw_header_t *) 0x08028000;

    HAL_UART_Transmit(&hlpuart1,
                                 (uint8_t *)"Entered Check headers\r\n",
                                 sizeof("Entered Check headers\r\n") - 1,
                                 HAL_MAX_DELAY);


  /* Temporary buffer for computed hash */
  static unsigned char calc_hash[32];

  if (hdr->magic != 0xDEADBEEF) {
    Error_Handler();
  }

  if (hdr->fw_size > MAX_APP_SIZE) {
    Error_Handler();
  }

  /* Compute SHA-256 over the application image */
  if (mbedtls_sha256((const unsigned char *)0x08028080,
             hdr->fw_size,
             calc_hash,
             0) != 0) {
    Error_Handler();
  }

  if (memcmp(calc_hash, hdr->hash, 32) != 0) {
    Error_Handler();
  }

  /* Verify ECDSA signature (raw r||s 64 bytes) against the public key (uncompressed 0x04||X||Y)
     Uses curve secp256r1. */
  // {
  //   int ret = 0;
  //   mbedtls_ecp_group grp;
  //   mbedtls_ecp_point Q;
  //   mbedtls_mpi r, s;

  //   mbedtls_ecp_group_init(&grp);
  //   mbedtls_ecp_point_init(&Q);
  //   mbedtls_mpi_init(&r);
  //   mbedtls_mpi_init(&s);

  //   if ((ret = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP256R1)) != 0) {
  //     Error_Handler();
  //   }

  //   if ((ret = mbedtls_ecp_point_read_binary(&grp, &Q, public_key, PUB_KEY_LEN)) != 0) {
  //     Error_Handler();
  //   }

  //   if ((ret = mbedtls_mpi_read_binary(&r, hdr->signature, 32)) != 0) {
  //     Error_Handler();
  //   }
  //   if ((ret = mbedtls_mpi_read_binary(&s, hdr->signature + 32, 32)) != 0) {
  //     Error_Handler();
  //   }

  //   ret = mbedtls_ecdsa_verify(&grp, hdr->hash, 32, &Q, &r, &s);

  //   mbedtls_mpi_free(&r);
  //   mbedtls_mpi_free(&s);
  //   mbedtls_ecp_point_free(&Q);
  //   mbedtls_ecp_group_free(&grp);

  //   if (ret != 0) {
  //     Error_Handler();
  //   }
  // }

  if (ecdsa_verify_signature(hdr->hash, hdr->signature) != 0)
  {
      Error_Handler();   // Not signed by us
  }

  /* All checks passed — jump to application image */
  // JumpToApplication(APP2_ADDR + sizeof(fw_header_t));
  JumpToApplication(APPT_ADDR);

}

int ecdsa_verify_signature(uint8_t *hash, uint8_t *signature)
{
    int ret;
    mbedtls_ecp_group grp;
    mbedtls_ecp_point Q;
    mbedtls_mpi r, s;

    mbedtls_ecp_group_init(&grp);
    mbedtls_ecp_point_init(&Q);
    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);

    /* Load P-256 curve */
    ret = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP256R1);
    if (ret != 0) goto cleanup;

    /* Load public key (65 bytes uncompressed) */
    ret = mbedtls_ecp_point_read_binary(
        &grp,
        &Q,
        public_key,
        65
    );
    if (ret != 0) goto cleanup;

    /* r = signature[0..31] */
    ret = mbedtls_mpi_read_binary(&r, signature, 32);
    if (ret != 0) goto cleanup;

    /* s = signature[32..63] */
    ret = mbedtls_mpi_read_binary(&s, signature + 32, 32);
    if (ret != 0) goto cleanup;

    /* Verify */
    ret = mbedtls_ecdsa_verify(
        &grp,
        hash,
        32,
        &Q,
        &r,
        &s
    );

cleanup:
    mbedtls_ecp_group_free(&grp);
    mbedtls_ecp_point_free(&Q);
    mbedtls_mpi_free(&r);
    mbedtls_mpi_free(&s);

    return ret;   // 0 = VALID, non-zero = FAIL
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
