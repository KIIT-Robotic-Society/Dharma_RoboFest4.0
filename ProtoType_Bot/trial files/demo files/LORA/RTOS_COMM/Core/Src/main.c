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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "J3_IBUS_FLYSKY.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "LoRa.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
bool switch_temp=false;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId myTask01Handle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
/* USER CODE BEGIN PV */
TRxIBus *RxIBus;
LoRa LoRa_Receive;  //Receive Instance
LoRa LoRa_Transmit; //Transmit Instance
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

void StartTask01(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);

/* USER CODE BEGIN PFP */
bool extract_and_sum_data(void);
void channel_value(void);
void IBUS_data_print(void);
void LORA_data_print(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TEMP_BUFFER_SIZE 1024  // maximum size for temp_buffer
#define TARGET_SIZE 32
char temp_buffer[TEMP_BUFFER_SIZE];  // global buffer to store the formatted output
uint16_t temp_buffer_index = 0;  // index to track where we are in the temp_buffer
#define FRAME_SIZE 32
uint16_t expected_result = 0;
uint32_t result=0;

/*Global Channel Data*/
uint16_t channel_1= 0;
uint16_t channel_2= 0;
uint16_t channel_3= 0;
uint16_t channel_4= 0;
uint16_t channel_5= 0;
uint16_t channel_6= 0;
uint16_t channel_7= 0;
uint16_t channel_8= 0;
uint16_t channel_9= 0;
uint16_t channel_10= 0;

//LORA Global Variables
uint8_t read_data[128];
uint8_t send_data[128];
uint8_t packet_size = 0;

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
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    RxIBus = J3_IBUS_new(&huart1, 14);
    HAL_UART_Receive_DMA(&huart1, RxIBus->buffer, 64);

    //LORA Initialization
    /*LoRa_Receive = newLoRa();
    LoRa_Transmit = newLoRa();

    //LORA pins initialization
  	LoRa_Receive.hSPIx                 = &hspi3;
  	LoRa_Receive.CS_port               = NSS_Tx_GPIO_Port;
  	LoRa_Receive.CS_pin                = NSS_Tx_Pin;
  	LoRa_Receive.reset_port            = RST_Tx_GPIO_Port;
  	LoRa_Receive.reset_pin             = RST_Tx_Pin;
  	LoRa_Receive.DIO0_port			   =   DIO0_Tx_GPIO_Port;
  	LoRa_Receive.DIO0_pin			   = DIO0_Tx_Pin;

  	LoRa_Receive.frequency             = 450;					    // default = 433 MHz
  	LoRa_Receive.spredingFactor        = SF_7;					   // default = SF_7
  	LoRa_Receive.bandWidth			   = BW_125KHz;				  // default = BW_125KHz
  	LoRa_Receive.crcRate			   = CR_4_5;				 // default = CR_4_5
  	LoRa_Receive.power			       = POWER_20db;			// default = 20db
  	LoRa_Receive.overCurrentProtection = 120; 				   // default = 100 mA
  	LoRa_Receive.preamble			   = 10;	              // default = 8;


  	LoRa_Transmit.hSPIx                 = &hspi2;
  	LoRa_Transmit.CS_port               = NSS_Rx_GPIO_Port;
  	LoRa_Transmit.CS_pin                = NSS_Rx_Pin;
  	LoRa_Transmit.reset_port            = RST_Rx_GPIO_Port;
  	LoRa_Transmit.reset_pin             = RST_Rx_Pin;
  	LoRa_Transmit.DIO0_port			 	= DIO0_Rx_GPIO_Port;
  	LoRa_Transmit.DIO0_pin				= DIO0_Rx_Pin;

  	LoRa_Transmit.frequency             = 433;							 // default = 433 MHz
  	LoRa_Transmit.spredingFactor        = SF_7;							// default = SF_7
  	LoRa_Transmit.bandWidth			    = BW_125KHz;				   // default = BW_125KHz
  	LoRa_Transmit.crcRate				= CR_4_5;					  // default = CR_4_5
  	LoRa_Transmit.power			        = POWER_20db;				 // default = 20db
  	LoRa_Transmit.overCurrentProtection = 120; 					    // default = 100 mA
  	LoRa_Transmit.preamble				= 10;

  	LoRa_reset(&LoRa_Receive);
  	LoRa_reset(&LoRa_Transmit);
  	LoRa_init(&LoRa_Receive);
  	LoRa_init(&LoRa_Transmit);
  	*/

  	HAL_UART_Transmit(&huart2, (uint8_t*)"Communication Dharma STM32 RTOS !\n", strlen("Communication Dharma STM32 RTOS !\n"), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */

  if(switch_temp==true){
	  /* definition and creation of myTask02 */
	  osThreadDef(myTask02, StartTask02, osPriorityHigh, 0, 1024); //LORA Receive
	  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);


	  /* definition and creation of myTask04 */
	  osThreadDef(myTask04, StartTask04, osPriorityBelowNormal, 0, 1024); //Lora Transmit
	  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  }

  else {

	  /* definition and creation of myTask03 */
	  osThreadDef(myTask03, StartTask03, osPriorityHigh, 0, 1024); //IBUS
	  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

	  /* definition and creation of myTask01 */
	  osThreadDef(myTask01, StartTask01, osPriorityLow, 0, 1024); //Debugging IBUS
	  myTask01Handle = osThreadCreate(osThread(myTask01), NULL);
  }


  /* definition and creation of myTask05 */
  osThreadDef(myTask05, StartTask05, osPriorityNormal, 0, 128);// Simulating CAN
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RST_Tx_Pin|DIO0_Tx_Pin|NSS_Tx_Pin|RST_Rx_Pin
                          |LORA_RX_LED_Pin|CAN_LED_Pin|IBUS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIO0_Rx_Pin|NSS_Rx_Pin|LORA_TX_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_Tx_Pin DIO0_Tx_Pin NSS_Tx_Pin RST_Rx_Pin
                           LORA_RX_LED_Pin CAN_LED_Pin IBUS_LED_Pin */
  GPIO_InitStruct.Pin = RST_Tx_Pin|DIO0_Tx_Pin|NSS_Tx_Pin|RST_Rx_Pin
                          |LORA_RX_LED_Pin|CAN_LED_Pin|IBUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO0_Rx_Pin NSS_Rx_Pin LORA_TX_LED_Pin */
  GPIO_InitStruct.Pin = DIO0_Rx_Pin|NSS_Rx_Pin|LORA_TX_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_StartTask01 */

/* Debugging IBUS */
void StartTask01(void const * argument)
{
  /* USER CODE BEGIN StartTask01 */
  /* Infinite loop */
  for(;;)
  {
	  IBUS_data_print();

	//HAL_UART_Transmit(&huart2, (uint8_t*)"Task 1 !\n", strlen("Task 1 !\n"), HAL_MAX_DELAY);
	  osDelay(100);
  }
  /* USER CODE END StartTask01 */
}

/* USER CODE END Header_StartTask01 */

/* LORA Receiving */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	uint8_t packet_size = 0;
	LoRa_startReceiving(&LoRa_Receive);
	packet_size = LoRa_receive(&LoRa_Receive, read_data, 128);


	HAL_UART_Transmit(&huart2, (uint8_t*)read_data, packet_size,HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,(uint8_t*)"\n", 1,HAL_MAX_DELAY);

	//HAL_UART_Transmit(&huart2, (uint8_t*)"Task 2 !\n", strlen("Task 2 !\n"), HAL_MAX_DELAY);
    osDelay(130);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */

/* IBUS Data Receiving */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
	copy_buffer(RxIBus->buffer, 64); //parsing data from IBUS receiver

	//parsing data and check data via checksum
	if(extract_and_sum_data()==true){
	channel_value(); //channel data parsing
	 }
	J3_IBUS_RequestNewData(RxIBus); //getting new data via DMA

	//HAL_UART_Transmit(&huart2, (uint8_t*)"Task 3 !\n", strlen("Task 3 !\n"), HAL_MAX_DELAY);

    osDelay(5);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */

/* LORA Data Transmitting */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {

	//demo value
	    for (int i = 0; i <= 120; i++) {
	        send_data[i] = 'a' + (i % 26);
	    }

	 LoRa_transmit(&LoRa_Transmit, send_data, 128, 500);

	 //HAL_UART_Transmit(&huart2, (uint8_t*)"Task 4 !\n", strlen("Task 4 !\n"), HAL_MAX_DELAY);
	 osDelay(50);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */

/* Mode */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {

	HAL_UART_Transmit(&huart2, (uint8_t*)"CAN Simulate!\n", strlen("CAN Simulate!\n"), HAL_MAX_DELAY);
    osDelay(100);
  }
  /* USER CODE END StartTask05 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

bool extract_and_sum_data(void) {
	//local parsing variables

    uint32_t sum = 0;
    uint16_t num_values = 0;
    uint16_t num_values_x = 0;
    uint16_t last_two_values[2] = {0};


    for (uint16_t i = 0; i < temp_buffer_index - 3; i++) {
        if (temp_buffer[i] == '0' && temp_buffer[i + 1] == 'x') {

            char hex_value[3] = {};
            hex_value[0] = temp_buffer[i + 2];
            hex_value[1] = temp_buffer[i + 3];

            uint8_t value = (uint8_t)strtol(hex_value, NULL, 16);
            num_values_x++;
           if (num_values_x == 31) {
                last_two_values[0] = value;
            }
            if (num_values_x == 32) {
                last_two_values[1] = value;
            }

            i += 3;

            if (num_values_x >= 32) {
                break;
            }
        }
    }


    for (uint16_t i = 0; i < temp_buffer_index - 3; i++) {
        if (temp_buffer[i] == '0' && temp_buffer[i + 1] == 'x') {
            char hex_value[3] = {};
            hex_value[0] = temp_buffer[i + 2];
            hex_value[1] = temp_buffer[i + 3];

            uint8_t value = (uint8_t)strtol(hex_value, NULL, 16);
            sum += value;
            num_values++;

            i += 3;
            if (num_values >= 30) {
                break;
            }
        }
    }

    result = 0xFFFF - sum;


    expected_result = (last_two_values[1] << 8) | last_two_values[0];

    //optional, debugging statement

    /*char result_str[40];
    char result_str_2[40];

    snprintf(result_str, sizeof(result_str), "Result: 0x%04X", result);
    HAL_UART_Transmit(&huart2, (uint8_t*)result_str, strlen(result_str), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);

    snprintf(result_str_2, sizeof(result_str), "Expected: 0x%04X", expected_result);
    HAL_UART_Transmit(&huart2, (uint8_t*)result_str, strlen(result_str), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);  // Send newline after expected result*/

    if (result == expected_result) {
        //HAL_UART_Transmit(&huart2, "Match!\r\n", 8, HAL_MAX_DELAY);
    	return true;
    } else {
        //HAL_UART_Transmit(&huart2, "No Match!\r\n", 11, HAL_MAX_DELAY);
        //Error_Handler();
    	return false;
    }
}


void channel_value() {
    uint16_t num_values_x = 0;
    uint16_t channel_1_hex[2] = {0};
    uint16_t channel_2_hex[2] = {0};
    uint16_t channel_3_hex[2] = {0};
    uint16_t channel_4_hex[2] = {0};
    uint16_t channel_5_hex[2] = {0};
    uint16_t channel_6_hex[2] = {0};
    uint16_t channel_7_hex[2] = {0};
    uint16_t channel_8_hex[2] = {0};
    uint16_t channel_9_hex[2] = {0};
    uint16_t channel_10_hex[2] = {0};

    for (uint16_t i = 0; i < temp_buffer_index - 3; i++) {
        if (temp_buffer[i] == '0' && temp_buffer[i + 1] == 'x') {

            char hex_value[3] = {};
            hex_value[0] = temp_buffer[i + 2];
            hex_value[1] = temp_buffer[i + 3];

            uint8_t value = (uint8_t)strtol(hex_value, NULL, 16);
            num_values_x++;

            //channel 1
            if (num_values_x == 3) {
            	channel_1_hex[0] = value;
            }
            if (num_values_x == 4) {
            	channel_1_hex[1] = value;
            }
            //channel 2
            if (num_values_x == 5) {
            	channel_2_hex[0] = value;
            }
            if (num_values_x == 6) {
            	channel_2_hex[1] = value;
            }
            //channel 3
            if (num_values_x == 7) {
            	channel_3_hex[0] = value;
            }
            if (num_values_x == 8) {
            	channel_3_hex[1] = value;
            }
            //channel 4
            if (num_values_x == 9) {
            	channel_4_hex[0] = value;
            }
            if (num_values_x == 10) {
            	channel_4_hex[1] = value;
            }
            //channel 5
            if (num_values_x == 11) {
            	channel_5_hex[0] = value;
            }
            if (num_values_x == 12) {
            	channel_5_hex[1] = value;
            }
            //channel 6
            if (num_values_x == 13) {
            	channel_6_hex[0] = value;
            }
            if (num_values_x == 14) {
            	channel_6_hex[1] = value;
            }
            //channel 7
            if (num_values_x == 15) {
            	channel_7_hex[0] = value;
            }
            if (num_values_x == 16) {
            	channel_7_hex[1] = value;
            }
            //channel 8
            if (num_values_x == 17) {
            	channel_8_hex[0] = value;
            }
            if (num_values_x == 18) {
            	channel_8_hex[1] = value;
            }
            //channel 9
            if (num_values_x == 19) {
            	channel_9_hex[0] = value;
            }
            if (num_values_x == 20) {
            	channel_9_hex[1] = value;
            }
            //channel 10
            if (num_values_x == 21) {
            	channel_10_hex[0] = value;
            }
            if (num_values_x == 22) {
            	channel_10_hex[1] = value;
            }

            i += 3;

            if (num_values_x >= 25) {
                break;
            }
        }
    }

    // Combine the two last values to get the expected result in little-endian format
    channel_1 = (channel_1_hex[1] << 8) | channel_1_hex[0];
    channel_2 = (channel_2_hex[1] << 8) | channel_2_hex[0];
    channel_3 = (channel_3_hex[1] << 8) | channel_3_hex[0];
    channel_4 = (channel_4_hex[1] << 8) | channel_4_hex[0];
    channel_5 = (channel_5_hex[1] << 8) | channel_5_hex[0];
    channel_6 = (channel_6_hex[1] << 8) | channel_6_hex[0];
    channel_7 = (channel_7_hex[1] << 8) | channel_7_hex[0];
    channel_8 = (channel_8_hex[1] << 8) | channel_8_hex[0];
    channel_9 = (channel_9_hex[1] << 8) | channel_9_hex[0];
    channel_10 = (channel_10_hex[1] << 8) | channel_10_hex[0];

    //optional  debugging values

    // Format the expected result as a decimal number for UART transmission
    /*char result_str_1[40];
    char result_str_2[40];
    char result_str_3[40];
    char result_str_4[40];
    char result_str_5[40];
    char result_str_6[40];
    char result_str_7[40];
    char result_str_8[40];
    char result_str_9[40];
    char result_str_10[40];

    snprintf(result_str_1, sizeof(result_str_1), "Channel 1: %d", channel_1);
    snprintf(result_str_2, sizeof(result_str_2), "Channel 2: %d", channel_2);
    snprintf(result_str_3, sizeof(result_str_3), "Channel 3: %d", channel_3);
    snprintf(result_str_4, sizeof(result_str_4), "Channel 4: %d", channel_4);
    snprintf(result_str_5, sizeof(result_str_5), "Channel 5: %d", channel_5);
    snprintf(result_str_6, sizeof(result_str_6), "Channel 6: %d", channel_6);
    snprintf(result_str_7, sizeof(result_str_7), "Channel 7: %d", channel_7);
    snprintf(result_str_8, sizeof(result_str_8), "Channel 8: %d", channel_8);
    snprintf(result_str_9, sizeof(result_str_9), "Channel 9: %d", channel_9);
    snprintf(result_str_10, sizeof(result_str_10), "Channel 10: %d", channel_10);

    // Send the results via UART
    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_1, strlen(result_str_1), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);  // Send newline

    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_2, strlen(result_str_2), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);  // Send newline

    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_3, strlen(result_str_3), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);  // Send newline

    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_4, strlen(result_str_4), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);  // Send newline

    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_5, strlen(result_str_5), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);  // Send newline

    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_6, strlen(result_str_6), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);  // Send newline

    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_7, strlen(result_str_7), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);  // Send newline

    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_8, strlen(result_str_8), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);  // Send newline

    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_9, strlen(result_str_9), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);  // Send newline

    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_10, strlen(result_str_10), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);  // Send newline*/


}

void copy_buffer(uint8_t *buffer, uint16_t size) {
	char temp[5];  // Temporary buffer to format each byte
	    uint16_t start_index = 0;  // Index where the specific frame starts

	    // Clear the temp_buffer before storing new data
	    memset(temp_buffer, 0, TEMP_BUFFER_SIZE);
	    temp_buffer_index = 0;  // Reset the index to start fresh

	    // Loop through the buffer to find the specific frame, in this case from 0x20 (0x20 0x40 ... 0xF3)
	    for (uint16_t i = 0; i < size; i++) {
	        // Check if we are at the beginning of the specific frame we want to print
	        if (buffer[i] == 0x20 && i + TARGET_SIZE <= size) {  // Look for the start of the frame
	            start_index = i;
	            break;
	        }
	    }

	    // Loop through the specific part of the buffer and format it
	    for (uint16_t i = start_index; i < start_index + TARGET_SIZE; i++) {
	        sprintf(temp, "0x%02X", buffer[i]);

	        // Copy the formatted data (temp) into the global temp_buffer
	        uint16_t len = strlen(temp);
	        if (temp_buffer_index + len < TEMP_BUFFER_SIZE) {
	            memcpy(&temp_buffer[temp_buffer_index], temp, len);
	            temp_buffer_index += len;  // Update the index in the temp_buffer
	        } else {
	            // Handle buffer overflow (optional, you may reset or stop appending)
	            break;
	        }
	    }

	}


void IBUS_data_print(){

    // Format the expected result as a decimal number for UART transmission
    char result_str_1[40];
    char result_str_2[40];
    char result_str_3[40];
    char result_str_4[40];
    char result_str_5[40];
    char result_str_6[40];
    char result_str_7[40];
    char result_str_8[40];
    char result_str_9[40];
    char result_str_10[40];

    snprintf(result_str_1, sizeof(result_str_1), "Channel 1: %d", channel_1);
    snprintf(result_str_2, sizeof(result_str_2), "Channel 2: %d", channel_2);
    snprintf(result_str_3, sizeof(result_str_3), "Channel 3: %d", channel_3);
    snprintf(result_str_4, sizeof(result_str_4), "Channel 4: %d", channel_4);
    snprintf(result_str_5, sizeof(result_str_5), "Channel 5: %d", channel_5);
    snprintf(result_str_6, sizeof(result_str_6), "Channel 6: %d", channel_6);
    snprintf(result_str_7, sizeof(result_str_7), "Channel 7: %d", channel_7);
    snprintf(result_str_8, sizeof(result_str_8), "Channel 8: %d", channel_8);
    snprintf(result_str_9, sizeof(result_str_9), "Channel 9: %d", channel_9);
    snprintf(result_str_10, sizeof(result_str_10), "Channel 10: %d", channel_10);

    // Send the results via UART

    //channel 1
    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_1, strlen(result_str_1), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);

    //channel 2

    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_2, strlen(result_str_2), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);

    //channel 3
    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_3, strlen(result_str_3), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);

    //channel 4
    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_4, strlen(result_str_4), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);

    //channel 5
    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_5, strlen(result_str_5), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);

    //channel 6
    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_6, strlen(result_str_6), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);

    //channel 7
    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_7, strlen(result_str_7), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);

    //channel 8
    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_8, strlen(result_str_8), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);

    //channel 9
    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_9, strlen(result_str_9), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);

    //channel 10
    HAL_UART_Transmit(&huart2, (uint8_t*)result_str_10, strlen(result_str_10), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);


	 }


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
