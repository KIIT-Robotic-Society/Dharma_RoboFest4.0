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
#include "string.h"
#include "VL53L0X.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

osThreadId myTask01Handle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);

void StartTask01(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

/* USER CODE BEGIN PFP */
void reset_GPIO(void);
void set_TOF1(void);
void set_TOF2(void);
void set_TOF3(void);
void set_TOF4(void);
void set_TOF5(void);
void set_TOF6(void);
void set_TOF7(void);
void set_TOF8(void);
void setup_sensor(void);
/* USER CODE END PFP */
statInfo_t_VL53L0X extraStats1,
extraStats2,
extraStats3,
extraStats4,
extraStats5,
extraStats6,
extraStats7,
extraStats8;

uint16_t distance1;
uint16_t distance2;
uint16_t distance3;
uint16_t distance4;
uint16_t distance5;
uint16_t distance6;
uint16_t distance7;
uint16_t distance8;



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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /*reset_GPIO();
  set_TOF1();
  set_TOF2();
  set_TOF3();
  set_TOF4();
  set_TOF5();
  set_TOF6();
  set_TOF7();
  set_TOF8();
  setup_sensor();*/

  HAL_GPIO_WritePin(V1_GPIO_Port, V1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(V2_GPIO_Port, V2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(V3_GPIO_Port, V3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(V4_GPIO_Port, V4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(V5_GPIO_Port, V5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(V6_GPIO_Port, V6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(V7_GPIO_Port, V7_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(V8_GPIO_Port, V8_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);

  //sensor 1
   HAL_GPIO_WritePin(V1_GPIO_Port, V1_Pin, GPIO_PIN_SET);
   HAL_Delay(50);
   if(initVL53L0X(1, &hi2c1)){
 	  HAL_UART_Transmit(&huart2, "sensor 1 initialized/n", strlen("sensor 1 initialized"), HAL_MAX_DELAY);
   }
   else{
	   HAL_UART_Transmit(&huart2, "error", strlen("error"), HAL_MAX_DELAY);
   }

  setAddress_VL53L0X(0x30);
  extraStats1.address=0x30;

  //sensor 2
   HAL_GPIO_WritePin(V2_GPIO_Port, V2_Pin, GPIO_PIN_SET);
   HAL_Delay(50);

  if(initVL53L0X(1, &hi2c1)){
   HAL_UART_Transmit(&huart2, "sensor 2 initialized/n", strlen("sensor 2 initialized"), HAL_MAX_DELAY);
   }
  else{
 	   HAL_UART_Transmit(&huart2, "error", strlen("error"), HAL_MAX_DELAY);
    }
//
  setAddress_VL53L0X(0x32);
   extraStats2.address=0x32;

   //sensor 3
   HAL_GPIO_WritePin(V3_GPIO_Port, V3_Pin, GPIO_PIN_SET);
   HAL_Delay(50);

  if(initVL53L0X(1, &hi2c1)){
   HAL_UART_Transmit(&huart2, "sensor 3 initialized/n", strlen("sensor 3 initialized"), HAL_MAX_DELAY);
   }
  else{
 	   HAL_UART_Transmit(&huart2, "error", strlen("error"), HAL_MAX_DELAY);
    }
//
  setAddress_VL53L0X(0x34);
   extraStats3.address=0x34;

   //sensor 4
   HAL_GPIO_WritePin(V4_GPIO_Port, V4_Pin, GPIO_PIN_SET);
   HAL_Delay(50);

  if(initVL53L0X(1, &hi2c1)){
   HAL_UART_Transmit(&huart2, "sensor 4 initialized/n", strlen("sensor 4 initialized"), HAL_MAX_DELAY);
   }
  else{
 	   HAL_UART_Transmit(&huart2, "error", strlen("error"), HAL_MAX_DELAY);
    }
//
  setAddress_VL53L0X(0x36);
   extraStats4.address=0x36;


   //sensor 5
   HAL_GPIO_WritePin(V5_GPIO_Port, V5_Pin, GPIO_PIN_SET);
   HAL_Delay(50);
   if(initVL53L0X(1, &hi2c1)){
 	  HAL_UART_Transmit(&huart2, "sensor 5 initialized/n", strlen("sensor 5 initialized"), HAL_MAX_DELAY);
   }
   else{
	   HAL_UART_Transmit(&huart2, "error", strlen("error"), HAL_MAX_DELAY);
   }

  setAddress_VL53L0X(0x38);
  extraStats5.address=0x38;

  //sensor 6
   HAL_GPIO_WritePin(V6_GPIO_Port, V6_Pin, GPIO_PIN_SET);
   HAL_Delay(50);

  if(initVL53L0X(1, &hi2c1)){
   HAL_UART_Transmit(&huart2, "sensor 6 initialized/n", strlen("sensor 6 initialized"), HAL_MAX_DELAY);
   }
  else{
 	   HAL_UART_Transmit(&huart2, "error", strlen("error"), HAL_MAX_DELAY);
    }
//
  setAddress_VL53L0X(0x40);
   extraStats6.address=0x40;

   //sensor 7
   HAL_GPIO_WritePin(V7_GPIO_Port, V7_Pin, GPIO_PIN_SET);
   HAL_Delay(50);

  if(initVL53L0X(1, &hi2c1)){
   HAL_UART_Transmit(&huart2, "sensor 7 initialized/n", strlen("sensor 7 initialized"), HAL_MAX_DELAY);
   }
  else{
 	   HAL_UART_Transmit(&huart2, "error", strlen("error"), HAL_MAX_DELAY);
    }
//
  setAddress_VL53L0X(0x42);
   extraStats7.address=0x42;

   //sensor 8
   HAL_GPIO_WritePin(V8_GPIO_Port, V8_Pin, GPIO_PIN_SET);
   HAL_Delay(50);

  if(initVL53L0X(1, &hi2c1)){
   HAL_UART_Transmit(&huart2, "sensor 8 initialized/n", strlen("sensor 8 initialized"), HAL_MAX_DELAY);
   }
  else{
 	   HAL_UART_Transmit(&huart2, "error", strlen("error"), HAL_MAX_DELAY);
    }
//
  setAddress_VL53L0X(0x44);
   extraStats8.address=0x44;



   setSignalRateLimit_x(2000,&extraStats1);
   setSignalRateLimit_x(2000,&extraStats2);
   setSignalRateLimit_x(2000,&extraStats3);
   setSignalRateLimit_x(2000,&extraStats4);
   setSignalRateLimit_x(2000,&extraStats5);
   setSignalRateLimit_x(2000,&extraStats6);
   setSignalRateLimit_x(2000,&extraStats7);
   setSignalRateLimit_x(2000,&extraStats8);

   setVcselPulsePeriod_x(VcselPeriodPreRange, 10, &extraStats1);
  setVcselPulsePeriod_x(VcselPeriodPreRange, 10, &extraStats2);
  setVcselPulsePeriod_x(VcselPeriodPreRange, 10, &extraStats3);
  setVcselPulsePeriod_x(VcselPeriodPreRange, 10, &extraStats4);
  setVcselPulsePeriod_x(VcselPeriodPreRange, 10, &extraStats5);
    setVcselPulsePeriod_x(VcselPeriodPreRange, 10, &extraStats6);
    setVcselPulsePeriod_x(VcselPeriodPreRange, 10, &extraStats7);
    setVcselPulsePeriod_x(VcselPeriodPreRange, 10, &extraStats8);

   setVcselPulsePeriod_x(VcselPeriodFinalRange, 14, &extraStats1);
   setVcselPulsePeriod_x(VcselPeriodFinalRange, 14, &extraStats2);
   setVcselPulsePeriod_x(VcselPeriodFinalRange, 14, &extraStats3);
   setVcselPulsePeriod_x(VcselPeriodFinalRange, 14, &extraStats4);
   setVcselPulsePeriod_x(VcselPeriodFinalRange, 14, &extraStats5);
   setVcselPulsePeriod_x(VcselPeriodFinalRange, 14, &extraStats6);
   setVcselPulsePeriod_x(VcselPeriodFinalRange, 14, &extraStats7);
   setVcselPulsePeriod_x(VcselPeriodFinalRange, 14, &extraStats8);

   setMeasurementTimingBudget_x(300 * 1000UL, &extraStats1);
   setMeasurementTimingBudget_x(300 * 1000UL, &extraStats2);
   setMeasurementTimingBudget_x(300 * 1000UL, &extraStats2);
   setMeasurementTimingBudget_x(300 * 1000UL, &extraStats2);
   setMeasurementTimingBudget_x(300 * 1000UL, &extraStats5);
   setMeasurementTimingBudget_x(300 * 1000UL, &extraStats6);
   setMeasurementTimingBudget_x(300 * 1000UL, &extraStats7);
   setMeasurementTimingBudget_x(300 * 1000UL, &extraStats8);


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
  osThreadDef(myTask01, StartTask01, osPriorityAboveNormal, 0, 1024);
  myTask01Handle = osThreadCreate(osThread(myTask01), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityAboveNormal, 0, 1024);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityNormal, 0, 1024);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);


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
    /* USER CODE END WHILE */
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|V1_Pin|V4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, V8_Pin|V2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, V5_Pin|V6_Pin|V7_Pin|V3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin V1_Pin V4_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|V1_Pin|V4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : V8_Pin V2_Pin */
  GPIO_InitStruct.Pin = V8_Pin|V2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : V5_Pin V6_Pin V7_Pin V3_Pin */
  GPIO_InitStruct.Pin = V5_Pin|V6_Pin|V7_Pin|V3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void reset_GPIO(){

	  HAL_GPIO_WritePin(V1_GPIO_Port, V1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(V2_GPIO_Port, V2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(V3_GPIO_Port, V3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(V4_GPIO_Port, V4_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(V5_GPIO_Port, V5_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(V6_GPIO_Port, V6_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(V7_GPIO_Port, V7_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(V8_GPIO_Port, V8_Pin, GPIO_PIN_RESET);
	  HAL_Delay(10);

}


void set_TOF1(){

	   HAL_GPIO_WritePin(V1_GPIO_Port, V1_Pin, GPIO_PIN_SET);
	   HAL_Delay(50);
	   if(initVL53L0X(1, &hi2c1)){
		   HAL_UART_Transmit(&huart2, "sensor 1 initialized/n", strlen("sensor 1 initialized"), HAL_MAX_DELAY);
	   }
	   else{
		   HAL_UART_Transmit(&huart2, "error", strlen("error"), HAL_MAX_DELAY);
	   }
	  setAddress_VL53L0X(0x30);
	  extraStats1.address=0x30;

}


void set_TOF2(){

	   HAL_GPIO_WritePin(V2_GPIO_Port, V2_Pin, GPIO_PIN_SET);
	   HAL_Delay(50);

	  if(initVL53L0X(1, &hi2c1)){
		  HAL_UART_Transmit(&huart2, "sensor 2 initialized/n", strlen("sensor 2 initialized"), HAL_MAX_DELAY);
	   }
	  else{
		  HAL_UART_Transmit(&huart2, "error", strlen("error"), HAL_MAX_DELAY);
	  }
	  setAddress_VL53L0X(0x32);
	  extraStats2.address=0x32;

}


void set_TOF3(){

	   HAL_GPIO_WritePin(V3_GPIO_Port, V3_Pin, GPIO_PIN_SET);
	   HAL_Delay(50);

	  if(initVL53L0X(1, &hi2c1)){
		  HAL_UART_Transmit(&huart2, "sensor 3 initialized/n", strlen("sensor 3 initialized"), HAL_MAX_DELAY);
	   }
	  else{
	 	  HAL_UART_Transmit(&huart2, "error", strlen("error"), HAL_MAX_DELAY);
	    }
	  setAddress_VL53L0X(0x34);
	  extraStats3.address=0x34;

}


void set_TOF4(){
	   HAL_GPIO_WritePin(V4_GPIO_Port, V4_Pin, GPIO_PIN_SET);
	   HAL_Delay(50);

	  if(initVL53L0X(1, &hi2c1)){
		  HAL_UART_Transmit(&huart2, "sensor 4 initialized/n", strlen("sensor 4 initialized"), HAL_MAX_DELAY);
	   }
	  else{
		  HAL_UART_Transmit(&huart2, "error", strlen("error"), HAL_MAX_DELAY);
	    }
	  setAddress_VL53L0X(0x36);
	  extraStats4.address=0x36;
}


void set_TOF5(){
	   HAL_GPIO_WritePin(V5_GPIO_Port, V5_Pin, GPIO_PIN_SET);
	   HAL_Delay(50);
	   if(initVL53L0X(1, &hi2c1)){
		   HAL_UART_Transmit(&huart2, "sensor 5 initialized/n", strlen("sensor 5 initialized"), HAL_MAX_DELAY);
	   }
	   else{
		   HAL_UART_Transmit(&huart2, "error", strlen("error"), HAL_MAX_DELAY);
	   }

	  setAddress_VL53L0X(0x38);
	  extraStats5.address=0x38;
}


void set_TOF6(){
	   HAL_GPIO_WritePin(V6_GPIO_Port, V6_Pin, GPIO_PIN_SET);
	   HAL_Delay(50);

	  if(initVL53L0X(1, &hi2c1)){
		  HAL_UART_Transmit(&huart2, "sensor 6 initialized/n", strlen("sensor 6 initialized"), HAL_MAX_DELAY);
	   }
	  else{
		  HAL_UART_Transmit(&huart2, "error", strlen("error"), HAL_MAX_DELAY);
	    }
	  setAddress_VL53L0X(0x40);
	  extraStats6.address=0x40;
}


void set_TOF7(){
	   HAL_GPIO_WritePin(V7_GPIO_Port, V7_Pin, GPIO_PIN_SET);
	   HAL_Delay(50);

	  if(initVL53L0X(1, &hi2c1)){
		  HAL_UART_Transmit(&huart2, "sensor 7 initialized/n", strlen("sensor 7 initialized"), HAL_MAX_DELAY);
	   }
	  else{
		  HAL_UART_Transmit(&huart2, "error", strlen("error"), HAL_MAX_DELAY);
	    }
	  setAddress_VL53L0X(0x42);
	  extraStats7.address=0x42;
}


void set_TOF8(){
	   HAL_GPIO_WritePin(V8_GPIO_Port, V8_Pin, GPIO_PIN_SET);
	   HAL_Delay(50);

	  if(initVL53L0X(1, &hi2c1)){
		  HAL_UART_Transmit(&huart2, "sensor 8 initialized/n", strlen("sensor 8 initialized"), HAL_MAX_DELAY);
	   }
	  else{
		  HAL_UART_Transmit(&huart2, "error", strlen("error"), HAL_MAX_DELAY);
	    }
	  setAddress_VL53L0X(0x44);
	  extraStats8.address=0x44;
}

void setup_sensor(){

	   setSignalRateLimit_x(2000,&extraStats1);
	   setSignalRateLimit_x(2000,&extraStats2);
	   setSignalRateLimit_x(2000,&extraStats3);
	   setSignalRateLimit_x(2000,&extraStats4);
	   setSignalRateLimit_x(2000,&extraStats5);
	   setSignalRateLimit_x(2000,&extraStats6);
	   setSignalRateLimit_x(2000,&extraStats7);
	   setSignalRateLimit_x(2000,&extraStats8);

	   setVcselPulsePeriod_x(VcselPeriodPreRange, 10, &extraStats1);
	   setVcselPulsePeriod_x(VcselPeriodPreRange, 10, &extraStats2);
	   setVcselPulsePeriod_x(VcselPeriodPreRange, 10, &extraStats3);
	   setVcselPulsePeriod_x(VcselPeriodPreRange, 10, &extraStats4);
	   setVcselPulsePeriod_x(VcselPeriodPreRange, 10, &extraStats5);
	   setVcselPulsePeriod_x(VcselPeriodPreRange, 10, &extraStats6);
	   setVcselPulsePeriod_x(VcselPeriodPreRange, 10, &extraStats7);
	   setVcselPulsePeriod_x(VcselPeriodPreRange, 10, &extraStats8);

	   setVcselPulsePeriod_x(VcselPeriodFinalRange, 14, &extraStats1);
	   setVcselPulsePeriod_x(VcselPeriodFinalRange, 14, &extraStats2);
	   setVcselPulsePeriod_x(VcselPeriodFinalRange, 14, &extraStats3);
	   setVcselPulsePeriod_x(VcselPeriodFinalRange, 14, &extraStats4);
	   setVcselPulsePeriod_x(VcselPeriodFinalRange, 14, &extraStats5);
	   setVcselPulsePeriod_x(VcselPeriodFinalRange, 14, &extraStats6);
	   setVcselPulsePeriod_x(VcselPeriodFinalRange, 14, &extraStats7);
	   setVcselPulsePeriod_x(VcselPeriodFinalRange, 14, &extraStats8);

	   setMeasurementTimingBudget_x(300 * 1000UL, &extraStats1);
	   setMeasurementTimingBudget_x(300 * 1000UL, &extraStats2);
	   setMeasurementTimingBudget_x(300 * 1000UL, &extraStats2);
	   setMeasurementTimingBudget_x(300 * 1000UL, &extraStats2);
	   setMeasurementTimingBudget_x(300 * 1000UL, &extraStats5);
	   setMeasurementTimingBudget_x(300 * 1000UL, &extraStats6);
	   setMeasurementTimingBudget_x(300 * 1000UL, &extraStats7);
	   setMeasurementTimingBudget_x(300 * 1000UL, &extraStats8);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartTask01(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  distance1= readRangeSingleMillimeters(&extraStats1);
	char msgBuffer_1[20];
	sprintf(msgBuffer_1, "Distance1: %d\r\n", distance1);
	HAL_UART_Transmit(&huart2, (uint8_t*) msgBuffer_1, sizeof(msgBuffer_1), HAL_MAX_DELAY);	//HAL_UART_Transmit(&huart2, "TASK 1\n", strlen("TASK 1\n"), HAL_MAX_DELAY);
    osDelay(50);
  }
  /* USER CODE END StartTask02 */
}
/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  distance5= readRangeSingleMillimeters(&extraStats5);
	  char msgBuffer_2[20];
	  sprintf(msgBuffer_2, "Distance2: %d\r\n", distance5);
	  HAL_UART_Transmit(&huart2, (uint8_t*) msgBuffer_2, sizeof(msgBuffer_2), HAL_MAX_DELAY);
	//HAL_UART_Transmit(&huart2, "TASK 2\n", strlen("TASK 2\n"), HAL_MAX_DELAY);
    osDelay(30);
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
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10000);
  }
  /* USER CODE END StartTask03 */
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
