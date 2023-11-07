/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stm32l4s5i_iot01_tsensor.h"
#include "stm32l4s5i_iot01_magneto.h"
#include "stm32l4s5i_iot01_psensor.h"
#include "stm32l4s5i_iot01_accelero.h"

#include "stm32l4s5i_iot01_qspi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MEMORY_WRITE_ADDR	0x08000020UL

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

UART_HandleTypeDef huart1;

osThreadId Read_TaskHandle;
osThreadId UART_TransmitHandle;
osThreadId Button_TaskHandle;
/* USER CODE BEGIN PV */

/**************************** PART 1 AND PART 2 VARIABLES ******************/
uint8_t txbuffer[64];

int sensorNum = 0;

float temp;
int16_t magnet[3];
float pressure;
int16_t accel[3];

/**************************** PART 3 VARIABLES ****************************/
uint8_t *writeBuf = "6";


/**************************** PART 4 VARIABLES ****************************/


uint8_t tempBuf[11];
uint8_t tempBuf2[50];
uint8_t tempBuf3[200];
long tempWriteCnt = 0;
long tempReadCnt = 0;
int tempVals = 0;
float tempAvg = 0;
uint8_t readBufTemp[11];


uint8_t magnetBufX[5];
uint8_t magnetBufY[5];
uint8_t magnetBufZ[5];
uint8_t magnetBuf2[60];
uint8_t magnetBuf3[200];
long magnetWriteCntX = 4096;
long magnetWriteCntY = 5461;
long magnetWriteCntZ = 6826;
long magnetReadCntX = 4096;
long magnetReadCntY = 5461;
long magnetReadCntZ = 6826;
int magnetVals = 0;
float magnetAvgX = 0;
float magnetAvgY = 0;
float magnetAvgZ = 0;
uint8_t readBufMagnetX[5];
uint8_t readBufMagnetY[5];
uint8_t readBufMagnetZ[5];

uint8_t presBuf[1200];
uint8_t presCnt = 0;

uint8_t accelBuf[1600];
uint8_t accelCnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_OCTOSPI1_Init(void);
void StartRead_Task(void const * argument);
void StartUART_Transmit(void const * argument);
void StartButton_Task(void const * argument);

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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_OCTOSPI1_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize BSP Peripherals */
  if(BSP_TSENSOR_Init() != TSENSOR_OK){
	  Error_Handler();
  }
  if(BSP_MAGNETO_Init() != MAGNETO_OK){
	  Error_Handler();
  }
  if(BSP_PSENSOR_Init() != PSENSOR_OK){
	  Error_Handler();
  }
  if(BSP_ACCELERO_Init() != ACCELERO_OK){
	  Error_Handler();
  }

  /* Initialize QSPI */
  if(BSP_QSPI_Init() != QSPI_OK){
	  Error_Handler();
  }

  HAL_GPIO_WritePin(REDLED_GPIO_Port, REDLED_Pin, GPIO_PIN_SET);

  //Clear sector for temp values
  if (BSP_QSPI_Erase_Block(0) != QSPI_OK){
      Error_Handler();
  }

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
  /* definition and creation of Read_Task */
  osThreadDef(Read_Task, StartRead_Task, osPriorityNormal, 0, 512);
  Read_TaskHandle = osThreadCreate(osThread(Read_Task), NULL);

  /* definition and creation of UART_Transmit */
  osThreadDef(UART_Transmit, StartUART_Transmit, osPriorityNormal, 0, 512);
  UART_TransmitHandle = osThreadCreate(osThread(UART_Transmit), NULL);

  /* definition and creation of Button_Task */
  osThreadDef(Button_Task, StartButton_Task, osPriorityNormal, 0, 512);
  Button_TaskHandle = osThreadCreate(osThread(Button_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //HAL_GPIO_WritePin(REDLED_GPIO_Port, REDLED_Pin, GPIO_PIN_RESET);

//  if (BSP_QSPI_Erase_Block(0) != QSPI_OK){
//	  Error_Handler();
//  }
//
//  if (BSP_QSPI_Write(writeBuf, 0, strlen(writeBuf)) != QSPI_OK){
//	  Error_Handler();
//  }
//
//  if (BSP_QSPI_Read(readBuf, 0, strlen((char*)writeBuf)) != QSPI_OK){
//	  Error_Handler();
//  }
//  HAL_UART_Transmit(&huart1, readBuf, strlen((char*)readBuf), HAL_MAX_DELAY);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  // If 0, Temperature sensor is active
//	  if(sensorNum == 0){
//		  temp = BSP_TSENSOR_ReadTemp();
//		  sprintf((char*)txbuffer, "Temperature: %f\r\n", temp);
//		  HAL_UART_Transmit(&huart1, txbuffer, strlen((char*)txbuffer), HAL_MAX_DELAY);
//	  }
//	  // If 1, Magnetometer is active
//	  if(sensorNum == 1){
//		  BSP_MAGNETO_GetXYZ(magnet);
//		  sprintf((char*)txbuffer, "Magnetometer: %d, %d, %d\r\n", magnet[0], magnet[1], magnet[2]);
//		  HAL_UART_Transmit(&huart1, txbuffer, strlen((char*)txbuffer), HAL_MAX_DELAY);
//	  }
//	  // If 2, Pressure sensor is active
//	  if(sensorNum == 2){
//		  pressure = BSP_PSENSOR_ReadPressure();
//		  sprintf((char*)txbuffer, "Pressure: %f\r\n", pressure);
//		  HAL_UART_Transmit(&huart1, txbuffer, strlen((char*)txbuffer), HAL_MAX_DELAY);
//	  }
//	  // If 3, Accelerometer is active
//	  if(sensorNum == 3){
//		  BSP_ACCELERO_AccGetXYZ(accel);
//		  sprintf((char*)txbuffer, "Accelerometer: %d, %d, %d\r\n", accel[0], accel[1], accel[2]);
//		  HAL_UART_Transmit(&huart1, txbuffer, strlen((char*)txbuffer), HAL_MAX_DELAY);
//	  }
//	  HAL_Delay(100);

	  /************* PART 4 TESTING **********************/
//	  if(dataMode == 0){
//		  temp = BSP_TSENSOR_ReadTemp();
//		  sprintf((char*)tempBuf, "%f\r\n", temp);
//		  HAL_UART_Transmit(&huart1, tempBuf, strlen((char*)tempBuf), HAL_MAX_DELAY);
//	  }
	  //HAL_Delay(100);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 1;
  OSPIM_Cfg_Struct.NCSPort = 1;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(REDLED_GPIO_Port, REDLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : REDLED_Pin */
  GPIO_InitStruct.Pin = REDLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(REDLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTT_Pin MagnetSens_Pin */
  GPIO_InitStruct.Pin = BUTT_Pin|MagnetSens_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PresSens_Pin GyroSens_Pin TempSens_Pin */
  GPIO_InitStruct.Pin = PresSens_Pin|GyroSens_Pin|TempSens_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartRead_Task */
/**
  * @brief  Function implementing the Read_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRead_Task */
void StartRead_Task(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);

    // If 0, Temperature sensor is active
    if(sensorNum == 0){
    	temp = BSP_TSENSOR_ReadTemp();

    	sprintf((char*)tempBuf, "%f", temp);


    	if(tempWriteCnt >= 4081) {
    		tempWriteCnt = 0;
    	}

    	if (BSP_QSPI_Write(tempBuf, tempWriteCnt, 11) != QSPI_OK){
    		Error_Handler();
    	}

    	tempWriteCnt += 11;

    	if (tempVals < 371) tempVals++;
    }

    // If 1, Magnetometer is active
    if(sensorNum == 1){
    	BSP_MAGNETO_GetXYZ(magnet);

    	sprintf((char*)magnetBufX, "%d", magnet[0]);
    	sprintf((char*)magnetBufY, "%d", magnet[1]);
    	sprintf((char*)magnetBufZ, "%d", magnet[2]);

    	if(magnetWriteCntX >= 5456) {
    		magnetWriteCntX = 4096;
    	}
    	if(magnetWriteCntY >= 6821) {
    		magnetWriteCntY = 5461;
    	}
    	if(magnetWriteCntZ >= 8186) {
    		magnetWriteCntZ = 6826;
    	}

    	if (BSP_QSPI_Write(magnetBufX, magnetWriteCntX, 5) != QSPI_OK){
    		Error_Handler();
    	}
    	if (BSP_QSPI_Write(magnetBufY, magnetWriteCntY, 5) != QSPI_OK){
    		Error_Handler();
    	}
    	if (BSP_QSPI_Write(magnetBufZ, magnetWriteCntZ, 5) != QSPI_OK){
    		Error_Handler();
    	}

    	magnetWriteCntX += 5;
    	magnetWriteCntY += 5;
    	magnetWriteCntZ += 5;

    	if(magnetVals < 272) magnetVals++;
    }

    // If 2, Pressure sensor is active
    if(sensorNum == 2){
    	pressure = BSP_PSENSOR_ReadPressure();
    	sprintf((char*)txbuffer, "Pressure: %f\r\n", pressure);
    }

    // If 3, Accelerometer is active
    if(sensorNum == 3){
    	BSP_ACCELERO_AccGetXYZ(accel);
    	sprintf((char*)txbuffer, "Accelerometer: %d, %d, %d\r\n", accel[0], accel[1], accel[2]);
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUART_Transmit */
/**
* @brief Function implementing the UART_Transmit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART_Transmit */
void StartUART_Transmit(void const * argument)
{
  /* USER CODE BEGIN StartUART_Transmit */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
    tempAvg = 0;
    tempReadCnt = 0;

    magnetAvgX = 0;
    magnetAvgY = 0;
    magnetAvgZ = 0;

    magnetReadCntX = 4096;
    magnetReadCntX = 5461;
    magnetReadCntX = 6826;
    // If 0, Temperature sensor is active
    if(sensorNum == 0){
    	sprintf((char*)tempBuf2, "Temperature: %f\r\n", temp);
    	HAL_UART_Transmit(&huart1, tempBuf2, sizeof(tempBuf2), HAL_MAX_DELAY);
    }
    // If 1, Magnetometer is active
    if(sensorNum == 1){
    	sprintf((char*)magnetBuf2, "Magnetometer: %d, %d, %d\r\n", magnet[0], magnet[1], magnet[2]);
    	HAL_UART_Transmit(&huart1, magnetBuf2, sizeof(magnetBuf2), HAL_MAX_DELAY);
    }
    // If 2, Pressure sensor is active
    if(sensorNum == 2){
       	HAL_UART_Transmit(&huart1, txbuffer, strlen((char*)txbuffer), HAL_MAX_DELAY);
    }
    // If 3, Accelerometer is active
    if(sensorNum == 3){
       	HAL_UART_Transmit(&huart1, txbuffer, strlen((char*)txbuffer), HAL_MAX_DELAY);
    }
    // If 4, Information for sensors
    if(sensorNum == 4){

    	for(int i = 0; i < tempVals; i++){
    		if (BSP_QSPI_Read(readBufTemp, tempReadCnt, 11) != QSPI_OK){
    	    	Error_Handler();
    		}

    	    tempAvg += atof(readBufTemp);
    	    tempReadCnt+=11;
    	}

    	tempAvg = tempAvg/tempVals;

    	sprintf((char*)tempBuf3, "Number of Measurements: %d, Average temperature: %f\r\n", tempVals, tempAvg);
    	HAL_UART_Transmit(&huart1, tempBuf3, sizeof(tempBuf3), HAL_MAX_DELAY);

    	for(int i = 0; i < magnetVals; i++){
    		if (BSP_QSPI_Read(readBufMagnetX, magnetReadCntX, 5) != QSPI_OK){
    			Error_Handler();
    		}
    		if (BSP_QSPI_Read(readBufMagnetY, magnetReadCntY, 5) != QSPI_OK){
    			Error_Handler();
    		}
    		if (BSP_QSPI_Read(readBufMagnetZ, magnetReadCntZ, 5) != QSPI_OK){
    			Error_Handler();
    		}

    		magnetAvgX += atof(readBufMagnetX);
    		magnetAvgY += atof(readBufMagnetY);
    		magnetAvgZ += atof(readBufMagnetZ);

    		magnetReadCntX += 5;
    		magnetReadCntY += 5;
    		magnetReadCntZ += 5;
    	}

    	magnetAvgX = magnetAvgX/magnetVals;
    	magnetAvgY = magnetAvgY/magnetVals;
    	magnetAvgZ = magnetAvgZ/magnetVals;

    	sprintf((char*)magnetBuf3, "Number of Measurements: %d, Average magnetometer reading: %f, %f, %f\r\n", magnetVals, magnetAvgX, magnetAvgY, magnetAvgZ);
    	HAL_UART_Transmit(&huart1, magnetBuf3, sizeof(magnetBuf3), HAL_MAX_DELAY);
    }
  }
  /* USER CODE END StartUART_Transmit */
}

/* USER CODE BEGIN Header_StartButton_Task */
/**
* @brief Function implementing the Button_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButton_Task */
void StartButton_Task(void const * argument)
{
  /* USER CODE BEGIN StartButton_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
    if(HAL_GPIO_ReadPin(GPIOC, BUTT_Pin) == 0){
    	sensorNum++;
    	if(sensorNum == 5) sensorNum = 0;
    }
    HAL_Delay(150);
  }
  /* USER CODE END StartButton_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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

	HAL_GPIO_WritePin(REDLED_GPIO_Port, REDLED_Pin, GPIO_PIN_RESET);

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
