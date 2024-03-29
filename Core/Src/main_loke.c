///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2022 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "cmsis_os.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include "oled.h"
//#include "motor.h"
//
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
//TIM_HandleTypeDef htim1;
//TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim8;
//
//UART_HandleTypeDef huart3;
//
///* Definitions for defaultTask */
//osThreadId_t defaultTaskHandle;
//const osThreadAttr_t defaultTask_attributes = {
//  .name = "defaultTask",
//  .stack_size = 128 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
///* Definitions for MotorTask */
//osThreadId_t MotorTaskHandle;
//const osThreadAttr_t MotorTask_attributes = {
//  .name = "MotorTask",
//  .stack_size = 128 * 4,
//  .priority = (osPriority_t) osPriorityLow,
//};
///* Definitions for Encoder */
//osThreadId_t EncoderHandle;
//const osThreadAttr_t Encoder_attributes = {
//  .name = "Encoder",
//  .stack_size = 128 * 4,
//  .priority = (osPriority_t) osPriorityLow,
//};
///* Definitions for Show */
//osThreadId_t ShowHandle;
//const osThreadAttr_t Show_attributes = {
//  .name = "Show",
//  .stack_size = 128 * 4,
//  .priority = (osPriority_t) osPriorityLow,
//};
///* USER CODE BEGIN PV */
//
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_TIM8_Init(void);
//static void MX_TIM2_Init(void);
//static void MX_USART3_UART_Init(void);
//static void MX_TIM1_Init(void);
//void StartDefaultTask(void *argument);
//void motors(void *argument);
//void encoder_task(void *argument);
//void show(void *argument);
//
///* USER CODE BEGIN PFP */
//
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//uint8_t aRxBuffer[20];
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* USER CODE BEGIN 1 */
//
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_TIM8_Init();
//  MX_TIM2_Init();
//  MX_USART3_UART_Init();
//  MX_TIM1_Init();
//  /* USER CODE BEGIN 2 */
//  OLED_Init();
//
//  HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer, 10);
//
//  HAL_UART_Transmit_IT(&huart3,(uint8_t *) aRxBuffer, 10);
//
//  /* USER CODE END 2 */
//
//  /* Init scheduler */
//  osKernelInitialize();
//
//  /* USER CODE BEGIN RTOS_MUTEX */
//  /* add mutexes, ... */
//  /* USER CODE END RTOS_MUTEX */
//
//  /* USER CODE BEGIN RTOS_SEMAPHORES */
//  /* add semaphores, ... */
//  /* USER CODE END RTOS_SEMAPHORES */
//
//  /* USER CODE BEGIN RTOS_TIMERS */
//  /* start timers, add new ones, ... */
//  /* USER CODE END RTOS_TIMERS */
//
//  /* USER CODE BEGIN RTOS_QUEUES */
//  /* add queues, ... */
//  /* USER CODE END RTOS_QUEUES */
//
//  /* Create the thread(s) */
//  /* creation of defaultTask */
//  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
//
//  /* creation of MotorTask */
//  MotorTaskHandle = osThreadNew(motors, NULL, &MotorTask_attributes);
//
//  /* creation of Encoder */
//  EncoderHandle = osThreadNew(encoder_task, NULL, &Encoder_attributes);
//
//  /* creation of Show */
//  ShowHandle = osThreadNew(show, NULL, &Show_attributes);
//
//  /* USER CODE BEGIN RTOS_THREADS */
//  /* add threads, ... */
//  /* USER CODE END RTOS_THREADS */
//
//  /* USER CODE BEGIN RTOS_EVENTS */
//  /* add events, ... */
//  /* USER CODE END RTOS_EVENTS */
//
//  /* Start scheduler */
//  osKernelStart();
//
//  /* We should never get here as control is now taken by the scheduler */
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//
//	  // if its blinking the os is broke
//	  HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
//	  HAL_Delay(2000);
//  }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief TIM1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM1_Init(void)
//{
//
//  /* USER CODE BEGIN TIM1_Init 0 */
//
//  /* USER CODE END TIM1_Init 0 */
//
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
//
//  /* USER CODE BEGIN TIM1_Init 1 */
//
//  /* USER CODE END TIM1_Init 1 */
//  htim1.Instance = TIM1;
//  htim1.Init.Prescaler = 160;
//  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim1.Init.Period = 1000;
//  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim1.Init.RepetitionCounter = 0;
//  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//  sBreakDeadTimeConfig.DeadTime = 0;
//  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM1_Init 2 */
//
//  /* USER CODE END TIM1_Init 2 */
//  HAL_TIM_MspPostInit(&htim1);
//
//}
//
///**
//  * @brief TIM2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM2_Init(void)
//{
//
//  /* USER CODE BEGIN TIM2_Init 0 */
//
//  /* USER CODE END TIM2_Init 0 */
//
//  TIM_Encoder_InitTypeDef sConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM2_Init 1 */
//
//  /* USER CODE END TIM2_Init 1 */
//  htim2.Instance = TIM2;
//  htim2.Init.Prescaler = 0;
//  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = 4294967295;
//  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
//  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
//  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
//  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
//  sConfig.IC1Filter = 10;
//  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
//  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
//  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
//  sConfig.IC2Filter = 10;
//  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM2_Init 2 */
//
//  /* USER CODE END TIM2_Init 2 */
//
//}
//
///**
//  * @brief TIM8 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM8_Init(void)
//{
//
//  /* USER CODE BEGIN TIM8_Init 0 */
//
//  /* USER CODE END TIM8_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
//
//  /* USER CODE BEGIN TIM8_Init 1 */
//
//  /* USER CODE END TIM8_Init 1 */
//  htim8.Instance = TIM8;
//  htim8.Init.Prescaler = 0;
//  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim8.Init.Period = 7199;
//  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim8.Init.RepetitionCounter = 0;
//  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//  sBreakDeadTimeConfig.DeadTime = 0;
//  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM8_Init 2 */
//
//  /* USER CODE END TIM8_Init 2 */
//
//}
//
///**
//  * @brief USART3 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART3_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART3_Init 0 */
//
//  /* USER CODE END USART3_Init 0 */
//
//  /* USER CODE BEGIN USART3_Init 1 */
//
//  /* USER CODE END USART3_Init 1 */
//  huart3.Instance = USART3;
//  huart3.Init.BaudRate = 115200;
//  huart3.Init.WordLength = UART_WORDLENGTH_8B;
//  huart3.Init.StopBits = UART_STOPBITS_1;
//  huart3.Init.Parity = UART_PARITY_NONE;
//  huart3.Init.Mode = UART_MODE_TX_RX;
//  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART3_Init 2 */
//
//  /* USER CODE END USART3_Init 2 */
//
//}
//
///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOE_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
//                          |LED_3_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOB, DIN1_Pin|DIN2_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
//                           LED_3_Pin */
//  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
//                          |LED_3_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : AIN2_Pin AIN1_Pin */
//  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : DIN1_Pin DIN2_Pin */
//  GPIO_InitStruct.Pin = DIN1_Pin|DIN2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//}
//
///* USER CODE BEGIN 4 */
//
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	// Prevent unused arguments(s) compilation warning
//	UNUSED(huart);
//
//	HAL_UART_Transmit(&huart3,(uint8_t *)aRxBuffer,10,0xFFFF);
//}
//
///* USER CODE END 4 */
//
///* USER CODE BEGIN Header_StartDefaultTask */
///**
//  * @brief  Function implementing the defaultTask thread.
//  * @param  argument: Not used
//  * @retval None
//  */
///* USER CODE END Header_StartDefaultTask */
//void StartDefaultTask(void *argument)
//{
//  /* USER CODE BEGIN 5 */
//  uint8_t ch = 'A';
//  /* Infinite loop */
//  for(;;)
//  {
//	HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1,0xFFFF);
//
//	if(ch<'Z')
//		ch++;
//	else
//		ch = 'A';
//	HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
//
//    osDelay(5000);
//  }
//  /* USER CODE END 5 */
//}
//
///* USER CODE BEGIN Header_motors */
///**
//* @brief Function implementing the MotorTask thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_motors */
//void motors(void *argument)
//{
//  /* USER CODE BEGIN motors */
//
//	// DC MOTOR
//
//	uint16_t pwmVal = 0;
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
//
//	/* Infinite loop */
//  for(;;)
//  {
//	//clockwise
//	while(pwmVal <4000)
//	{
//
//		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
//
//		HAL_GPIO_WritePin(GPIOD,DIN2_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOD,DIN1_Pin,GPIO_PIN_RESET);
//
//
//		pwmVal++;
//		//Modify the comparison value for the duty cycle
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal);
//
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,pwmVal);
//
//
//	    osDelay(10);
//
//	}
//
//	//anti clockwise
//	while(pwmVal >0)
//	{
//		// Anti clockwise
//		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
//		// Clockwise
//		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
//
//		// Anti clockwise
//		HAL_GPIO_WritePin(GPIOD,DIN1_Pin,GPIO_PIN_SET);
//		// Clockwise
//		HAL_GPIO_WritePin(GPIOD,DIN2_Pin,GPIO_PIN_RESET);
//
//
//
//		pwmVal--;
//		//Modify the comparison value for the duty cycle
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal);
//
//		//Modify the comparison value for the duty cycle
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,pwmVal);
//		osDelay(10);
//	}
//    osDelay(1);
//  }
//
//
//	for(;;){
////		motors_subroutine();
//
//	}
//	// SERVO MOTOR
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
//	for(;;){
//
//		htim1.Instance->CCR4 = 176 ; //Extreme right 176
//		osDelay(2000);
//		htim1.Instance->CCR4 = 148 ; // center 148
//		osDelay(2000);
//		htim1.Instance->CCR4 = 120 ; //Extreme left 120
//		osDelay(2000);
//		htim1.Instance->CCR4 = 148 ; // center 148
//		osDelay(2000);
//	}
//
//  /* USER CODE END motors */
//}
//
///* USER CODE BEGIN Header_encoder_task */
///**
//* @brief Function implementing the Encoder thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_encoder_task */
//void encoder_task(void *argument)
//{
//  /* USER CODE BEGIN encoder_task */
//	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
//
//	int cnt1, cnt2, diff;
//	uint32_t tick;
//
//	cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
//	tick = HAL_GetTick();
//	uint8_t hello[20];
//	uint16_t dir;
//
//  /* Infinite loop */
//  for(;;)
//  {
//	if(HAL_GetTick()-tick > 1000L){
//		cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
//		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
//
//			if(cnt2<cnt1)
//				diff = cnt1 - cnt2;
//			else
//				diff  = (65535 - cnt2) + cnt1;
//		}
//		else{
//			if(cnt2>cnt1)
//				diff = cnt2 - cnt1;
//			else
//				diff  = (65535 - cnt1) + cnt2;
//			}
//		sprintf(hello,"Speed:%5d\0",diff);
//		OLED_ShowString(10,20,hello);
//		dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
//		sprintf(hello,"Dir:%5d\0",dir);
//		OLED_ShowString(10,30,hello);
//		cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
//		tick = HAL_GetTick();
//	}
////	osDelay(1);
//  }
//  /* USER CODE END encoder_task */
//}
//
///* USER CODE BEGIN Header_show */
///**
//* @brief Function implementing the Show thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_show */
//void show(void *argument)
//{
//  /* USER CODE BEGIN show */
//	uint8_t hello[20] = "Hello World\0";
//
//  /* Infinite loop */
//  for(;;)
//  {
//	  sprintf(hello,"%s\0",aRxBuffer);
//	  OLED_ShowString(10,10,hello);
//	  OLED_Refresh_Gram();
//      osDelay(1000);
//  }
//  /* USER CODE END show */
//}
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
