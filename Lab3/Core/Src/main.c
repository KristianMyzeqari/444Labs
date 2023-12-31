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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void play_Sounds();
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

float sound1[20];
float sound2[20];
float sound3[20];
float sound4[20];
float sound5[20];
float sound6[20];

int globalIndex;

#define AUDIO_REC 64000
#define WAIT_TIME 100

int32_t recBuf[AUDIO_REC];
uint32_t playBuf[AUDIO_REC];

uint8_t dmaRecBuffCplt = 0;
uint8_t isRecording = 0;
uint8_t hasPlayed = 0;

int32_t maxVal = -2147483648;
int32_t minVal = 2147483647;

//___________________________ PART 4 _____________________________
uint8_t counter = 0;
uint8_t playSong = 0;
uint8_t hasRecording = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DFSDM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/****
 * Interrupt function
 */
// PART 3 BUTTON PRESS INTERRUPT
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	if(hasPlayed == 1){
//		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
//	}
//	HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, recBuf, AUDIO_REC);
//	isRecording = 1;
//	hasPlayed = 1;
//}

// PART 1 TIMER INTERRUPT PLAYS THE NEXT SOUND
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
//	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0.7 * sound1[globalIndex]);
//
//	globalIndex++;
//	if (globalIndex == 20) globalIndex = 0;
//}

// PART 3 BUFFER IS FULL INTERRUPT
//void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter){
//	dmaRecBuffCplt = 1;
//	isRecording = 0;
//}

//______________________________________ PART 4 ______________________________________________

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(counter % 2 == 0){
		if(counter > 1) HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
		HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, recBuf, AUDIO_REC);
		isRecording = 1;
		hasRecording = 1;
	}

	else {
		playSong = 1;
	}
	counter++;
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter){
	dmaRecBuffCplt = 1;
	isRecording = 0;
}

void make_Sounds(){
	float curVal = 0.0;
	float trigVal;
	for(int i = 0; i < 20; i++){
		trigVal = 2047.5 * sin(curVal) + 2047.5;
	  	sound1[i] = 0.8*trigVal;
	  	curVal += 0.314159;
	}

	curVal = 0.0;
	trigVal = 0.0;
	for(int i = 0; i < 18; i++){
		trigVal = 2047.5 * sin(curVal) + 2047.5;
		sound2[i] = 0.8*trigVal;
		curVal += 0.349065;
	}

	curVal = 0.0;
	trigVal = 0.0;
	for(int i = 0; i < 16; i++){
		trigVal = 2047.5 * sin(curVal) + 2047.5;
		sound3[i] = 0.8*trigVal;
		curVal += 0.392698;
	}

	curVal = 0.0;
	trigVal = 0.0;
	for(int i = 0; i < 14; i++){
		trigVal = 2047.5 * sin(curVal) + 2047.5;
		sound4[i] = 0.8*trigVal;
		curVal += 0.448798;
	}

	curVal = 0.0;
	trigVal = 0.0;
	for(int i = 0; i < 12; i++){
		trigVal = 2047.5 * sin(curVal) + 2047.5;
		sound5[i] = 0.8*trigVal;
		curVal += 0.523598;
	}

	curVal = 0.0;
	trigVal = 0.0;
	for(int i = 0; i < 10; i++){
		trigVal = 2047.5 * sin(curVal) + 2047.5;
		sound6[i] = trigVal;
		curVal += 0.628318;
	}
}

void play_Sounds(){
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &sound1, 20, DAC_ALIGN_12B_R);
	HAL_Delay(1000);
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &sound2, 18, DAC_ALIGN_12B_R);
	HAL_Delay(1000);
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &sound3, 16, DAC_ALIGN_12B_R);
	HAL_Delay(1000);
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &sound4, 14, DAC_ALIGN_12B_R);
	HAL_Delay(1000);
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &sound5, 12, DAC_ALIGN_12B_R);
	HAL_Delay(1000);
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &sound6, 10, DAC_ALIGN_12B_R);
	HAL_Delay(1000);
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
}

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
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_DFSDM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  //HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &sound1, 20, DAC_ALIGN_12B_R);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //Populate array, we are using an array of 10 slots, where each slot is incremented by 0.6283 radians

  //PART 2 CODE
//  float curVal = 0.0;
//  float trigVal;
//  for(int i = 0; i < 20; i++){
//	  trigVal = 2047.5 * sin(curVal);
//	  array[i] = trigVal;
//	  curVal += 0.314159;
//  }
//  HAL_TIM_Base_Start_IT(&htim2);
//  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, array, 20, DAC_ALIGN_12B_R);



  //_________________________________ PART 4 ____________________________
  make_Sounds();
  //play_Sounds();


  float temp;
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(isRecording == 1){
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  HAL_Delay(WAIT_TIME);
	  }
	  else if(hasRecording == 1){
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  HAL_Delay(WAIT_TIME*7);
	  }
	  else{
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	  }

	  if((counter % 2 == 0) && (playSong == 1)) {
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	  		play_Sounds();
	  		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, playBuf, AUDIO_REC, DAC_ALIGN_12B_R);
	  		HAL_Delay(2560);
	  		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
	  		for(int i = 0; i < AUDIO_REC; i++){
	  			recBuf[i] = 0;
	  		}
	  		playSong = 0;
	  		hasRecording = 0;
	  }

	  if(dmaRecBuffCplt == 1){
		  for(int i = 0; i < AUDIO_REC; i++){

			  recBuf[i] = recBuf[i] >> 8;

			  if(recBuf[i] < minVal){
				  minVal = recBuf[i];
			  }
			  if(recBuf[i] > maxVal){
				  maxVal = recBuf[i];
			  }
		  }

		  if(minVal < 0) minVal = -1 * minVal;

		  temp = (float)((float)4095/((float)maxVal+(float)minVal));

		  for(int j = 0; j < AUDIO_REC; j++){
			  recBuf[j] = recBuf[j] + minVal;
			  playBuf[j] = temp * recBuf[j];


		  }

		  dmaRecBuffCplt = 0;
		  // PART 3 _________  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &playBuf, AUDIO_REC, DAC_ALIGN_12B_R);
	  }
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
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 100;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 48;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4800;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTT_Pin */
  GPIO_InitStruct.Pin = BUTT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
