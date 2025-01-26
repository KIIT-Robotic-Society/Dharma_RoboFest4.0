/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <ILI9341.h>
#include "J3_IBUS_FLYSKY.h"

#include "008_Open_Sans_Bold.h"
#include "009_Open_Sans_Bold.h"
#include "010_Open_Sans_Bold.h"
#include "012_Open_Sans_Bold.h"
#include "014_Open_Sans_Bold.h"
#include "016_Open_Sans_Bold.h"
#include "018_Open_Sans_Bold.h"
#include "020_Open_Sans_Bold.h"
#include "022_Open_Sans_Bold.h"
#include "024_Open_Sans_Bold.h"
#include "026_Open_Sans_Bold.h"
#include "028_Open_Sans_Bold.h"
#include "036_Open_Sans_Bold.h"
#include "048_Open_Sans_Bold.h"
#include "072_Open_Sans_Bold.h"
#include "096_Open_Sans_Bold.h"
#include "112_Open_Sans_Bold.h"
#include "128_Open_Sans_Bold.h"


#define _Open_Sans_Bold_8      &Open_Sans_Bold_8
#define _Open_Sans_Bold_9      &Open_Sans_Bold_9
#define _Open_Sans_Bold_10     &Open_Sans_Bold_10
#define _Open_Sans_Bold_11     &Open_Sans_Bold_11
#define _Open_Sans_Bold_12      &Open_Sans_Bold_12
#define _Open_Sans_Bold_14      &Open_Sans_Bold_14
#define _Open_Sans_Bold_16      &Open_Sans_Bold_16
#define _Open_Sans_Bold_18      &Open_Sans_Bold_18
#define _Open_Sans_Bold_20      &Open_Sans_Bold_20
#define _Open_Sans_Bold_22      &Open_Sans_Bold_22
#define _Open_Sans_Bold_24      &Open_Sans_Bold_24
#define _Open_Sans_Bold_26      &Open_Sans_Bold_26
#define _Open_Sans_Bold_28      &Open_Sans_Bold_28
#define _Open_Sans_Bold_36      &Open_Sans_Bold_36
#define _Open_Sans_Bold_48      &Open_Sans_Bold_48
#define _Open_Sans_Bold_72      &Open_Sans_Bold_72
#define _Open_Sans_Bold_96      &Open_Sans_Bold_96
#define _Open_Sans_Bold_112      &Open_Sans_Bold_112
#define _Open_Sans_Bold_128      &Open_Sans_Bold_128

char buf[20];
char buf1[20];
char buf2[20];
char buf3[20];
char buf4[20];
char buf5[20];
TRxIBus *RxIBus;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init();
  ILI9341_FillScreen(WHITE);

  LCD_Font(10, 30, "CHANNEL1", _Open_Sans_Bold_36  , 1, PURPLE);
  LCD_Font(10, 70, "CHANNEL2", _Open_Sans_Bold_36  , 1, BLUE);
  LCD_Font(10, 110, "CHANNEL3", _Open_Sans_Bold_36  , 1, GREEN);
  LCD_Font(10, 150, "CHANNEL4", _Open_Sans_Bold_36  , 1, RED);
  LCD_Font(10, 190, "CHANNEL5", _Open_Sans_Bold_36  , 1, MAGENTA);
  LCD_Font(10, 230, "CHANNEL6", _Open_Sans_Bold_36  , 1, ORANGE);

  RxIBus = J3_IBUS_new(&huart1, 14);

  HAL_UART_Receive_DMA(&huart1, RxIBus->buffer, 64);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  HAL_UART_Transmit(&huart1, "CH1:", 4, 100);
	  LCD_Font(230, 30, buf, _Open_Sans_Bold_36  , 1, WHITE);
	  sprintf(buf, "%d", J3_IBUS_GetCh(RxIBus, 1));
	  HAL_UART_Transmit(&huart1, (char*)buf, sprintf(buf, "%d", J3_IBUS_GetCh(RxIBus, 1)), 100);
	  HAL_UART_Transmit(&huart1, "\r\n", 2, 100);
//	  ILI9341_printText(str, 0, 0, COLOR_WHITE, COLOR_BLACK, 1);
	  LCD_Font(230, 30, buf, _Open_Sans_Bold_36  , 1, PURPLE);

	  HAL_UART_Transmit(&huart1, "CH2:", 4, 100);
	  LCD_Font(230, 70, buf1, _Open_Sans_Bold_36  , 1, WHITE);
	  sprintf(buf1, "%d", J3_IBUS_GetCh(RxIBus, 2));
	  HAL_UART_Transmit(&huart1, (char*)buf1, sprintf(buf1, "%d", J3_IBUS_GetCh(RxIBus, 2)), 100);
	  HAL_UART_Transmit(&huart1, "\r\n", 2, 100);
//	  ILI9341_printText(buf1, 0, 20, COLOR_WHITE, COLOR_BLACK, 1);
	  LCD_Font(230, 70, buf1, _Open_Sans_Bold_36  , 1, BLUE);

	  HAL_UART_Transmit(&huart1, "CH3:", 4, 100);
	  LCD_Font(230, 110, buf2, _Open_Sans_Bold_36  , 1, WHITE);
	  sprintf(buf2, "%d", J3_IBUS_GetCh(RxIBus, 3));
	  HAL_UART_Transmit(&huart1, (char*)buf2, sprintf(buf2, "%d", J3_IBUS_GetCh(RxIBus, 3)), 100);
	  HAL_UART_Transmit(&huart1, "\r\n", 2, 100);
//	  ILI9341_printText(str, 0, 40, COLOR_WHITE, COLOR_BLACK, 1);
	  LCD_Font(230, 110, buf2, _Open_Sans_Bold_36  , 1, GREEN);

	  HAL_UART_Transmit(&huart1, "CH4:", 4, 100);
	  LCD_Font(230, 150, buf3, _Open_Sans_Bold_36  , 1, WHITE);
	  sprintf(buf3, "%d", J3_IBUS_GetCh(RxIBus, 4));
	  HAL_UART_Transmit(&huart1, (char*)buf3, sprintf(buf3, "%d", J3_IBUS_GetCh(RxIBus, 4)), 100);
	  HAL_UART_Transmit(&huart1, "\r\n", 2, 100);
//	  ILI9341_printText(str, 0, 60, COLOR_WHITE, COLOR_BLACK, 1);
	  LCD_Font(230, 150, buf3, _Open_Sans_Bold_36  , 1, RED);

	  HAL_UART_Transmit(&huart1, "CH4:", 4, 100);
	  LCD_Font(230, 190, buf4, _Open_Sans_Bold_36  , 1, WHITE);
	  sprintf(buf4, "%d", J3_IBUS_GetCh(RxIBus, 5));
	  HAL_UART_Transmit(&huart1, (char*)buf4, sprintf(buf4, "%d", J3_IBUS_GetCh(RxIBus, 5)), 100);
	  HAL_UART_Transmit(&huart1, "\r\n", 2, 100);
//	  ILI9341_printText(str, 0, 80, COLOR_WHITE, COLOR_BLACK, 1);
	  LCD_Font(230, 190, buf4, _Open_Sans_Bold_36  , 1, MAGENTA);

	  HAL_UART_Transmit(&huart1, "CH4:", 4, 100);
	  LCD_Font(230, 230, buf5, _Open_Sans_Bold_36  , 1, WHITE);
	  sprintf(buf5, "%d", J3_IBUS_GetCh(RxIBus, 6));
	  HAL_UART_Transmit(&huart1, (char*)buf5, sprintf(buf5, "%d", J3_IBUS_GetCh(RxIBus, 6)), 100);
	  HAL_UART_Transmit(&huart1, "\r\n", 2, 100);
//	  ILI9341_printText(str, 0, 100, COLOR_WHITE, COLOR_BLACK, 1);
	  LCD_Font(230, 230, buf5, _Open_Sans_Bold_36  , 1, ORANGE);






	  HAL_UART_Transmit(&huart1, RxIBus->buffer, 64, 100);
	  HAL_UART_Transmit(&huart1, "\r\n", 2, 100);
//	  ILI9341_printText(str, 0, 60, COLOR_WHITE, COLOR_BLACK, 1);


	  HAL_Delay(100);
    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS_Pin|RESET_Pin|DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_Pin RESET_Pin DC_Pin */
  GPIO_InitStruct.Pin = CS_Pin|RESET_Pin|DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

  J3_IBUS_ProcessBuffer(RxIBus);
  HAL_UART_Receive_DMA(&huart1, RxIBus->buffer, 64);
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

#ifdef  USE_FULL_ASSERT
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
