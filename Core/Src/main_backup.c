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
//#include "math.h"
//#include "cmsis_os.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include <string.h>
//
//#include "oled.h"
//#include "motor.h"
//
//#include "ICM20948.h"
//
//#include "tm_stm32_ahrs_imu.h"
//
//#include "pid.h"
//
//#define SIZE 100
//
//
//// pid var//
//
//#define PID_KP  2.0f
//#define PID_KI  0.5f
//#define PID_KD  0.25f
//
//#define PID_TAU 0.02f
//
//#define PID_LIM_MIN -2.0f
//#define PID_LIM_MAX  2.0f
//
//#define PID_LIM_MIN_INT -5.0f
//#define PID_LIM_MAX_INT  5.0f
//
//////////////
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
//I2C_HandleTypeDef hi2c1;
//
//TIM_HandleTypeDef htim1;
//TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim4;
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
///* Definitions for Show */
//osThreadId_t ShowHandle;
//const osThreadAttr_t Show_attributes = {
//  .name = "Show",
//  .stack_size = 128 * 4,
//  .priority = (osPriority_t) osPriorityLow,
//};
///* Definitions for imu_task */
//osThreadId_t imu_taskHandle;
//const osThreadAttr_t imu_task_attributes = {
//  .name = "imu_task",
//  .stack_size = 128 * 4,
//  .priority = (osPriority_t) osPriorityLow,
//};
///* Definitions for pidTask */
//osThreadId_t pidTaskHandle;
//const osThreadAttr_t pidTask_attributes = {
//  .name = "pidTask",
//  .stack_size = 128 * 4,
//  .priority = (osPriority_t) osPriorityLow,
//};
///* Definitions for encoderTask */
//osThreadId_t encoderTaskHandle;
//const osThreadAttr_t encoderTask_attributes = {
//  .name = "encoderTask",
//  .stack_size = 128 * 4,
//  .priority = (osPriority_t) osPriorityLow,
//};
///* Definitions for steeringTask */
//osThreadId_t steeringTaskHandle;
//const osThreadAttr_t steeringTask_attributes = {
//  .name = "steeringTask",
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
//static void MX_TIM4_Init(void);
//static void MX_I2C1_Init(void);
//void StartDefaultTask(void *argument);
//void motors(void *argument);
//void show(void *argument);
//void Imu_Task(void *argument);
//void PidTask(void *argument);
//void EncoderTask(void *argument);
//void SteeringTask(void *argument);
//
///* USER CODE BEGIN PFP */
//void motor_A_init(void);
//void motor_B_init(void);
//
////void enQueue(move_order);
////move_order deQueue();
////void display();
//
//
//
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
//
//uint8_t aRxBuffer[20];
//uint8_t encoderA_val;
//uint8_t encoderC_val ;
//uint8_t encoderA_dir;
//uint8_t encoderC_dir;
//
//uint16_t motorA_pwm_val;
//uint16_t motorC_pwm_val;
//
//ICM20948 imu;
//
//uint32_t icmDelay;
//
//
//uint8_t batteryCounter, irCounter, ultraCounter, yawCounter;
//
//uint8_t uart_buf[20] = "\0";
//
//int cnt1=0, cnt2=0,diff=0;
//
//char current_wheel_dir = 'C';
//
//int update_encoder_start = 0;
//
//int motors_being_updated = 0;
//
//int center_val = 150;
//
//float current_global_yaw = 0;
//
//int mantain_heading = 0;
//float pid_control_val;
//
//int tick_dur;
//int imu_delay=0;
//
//float beta, inclination, prevYaw, yawVal[5], yawDiff, yawDir[4], yawThreshold;
//
//typedef struct{
//	   char  speed;
//	   char  dir;
//	   int   steps;
//}move_order;
//
//
//TM_AHRSIMU_t imu1;
//
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* USER CODE BEGIN 1 */
//	motorA_pwm_val =100;
//	motorC_pwm_val =100;
//
//	encoderA_val=0;
//	encoderC_val=0;
//	encoderA_dir=0;
//	encoderC_dir=0;
//
//	motorA_pwm_val=0;
//	motorC_pwm_val=0;
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
//  MX_TIM4_Init();
//  MX_I2C1_Init();
//  /* USER CODE BEGIN 2 */
//  OLED_Init();
//
//  HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer, 10);
//
//  HAL_UART_Transmit_IT(&huart3,(uint8_t *) aRxBuffer, 10);
//
//  uint8_t status = IMU_Initialise(&imu, &hi2c1, &huart3);
//
//
//    beta = 0.2;
//    inclination = 0;
//    prevYaw = -1;
//    yawDir[0] = 85; //North
//    yawDir[1] = 4; //East
//    yawDir[2] = 237; //South
//    yawDir[3] = 154; //West
//    yawThreshold = 5;
//
//    icmDelay = 7;
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
//  /* creation of Show */
//  ShowHandle = osThreadNew(show, NULL, &Show_attributes);
//
//  /* creation of imu_task */
//  imu_taskHandle = osThreadNew(Imu_Task, NULL, &imu_task_attributes);
//
//  /* creation of pidTask */
//  pidTaskHandle = osThreadNew(PidTask, NULL, &pidTask_attributes);
//
//  /* creation of encoderTask */
//  encoderTaskHandle = osThreadNew(EncoderTask, NULL, &encoderTask_attributes);
//
//  /* creation of steeringTask */
//  steeringTaskHandle = osThreadNew(SteeringTask, NULL, &steeringTask_attributes);
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
//  * @brief I2C1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_I2C1_Init(void)
//{
//
//  /* USER CODE BEGIN I2C1_Init 0 */
//
//  /* USER CODE END I2C1_Init 0 */
//
//  /* USER CODE BEGIN I2C1_Init 1 */
//
//  /* USER CODE END I2C1_Init 1 */
//  hi2c1.Instance = I2C1;
//  hi2c1.Init.ClockSpeed = 100000;
//  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
//  hi2c1.Init.OwnAddress1 = 0;
//  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//  hi2c1.Init.OwnAddress2 = 0;
//  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN I2C1_Init 2 */
//
//  /* USER CODE END I2C1_Init 2 */
//
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
//  * @brief TIM4 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM4_Init(void)
//{
//
//  /* USER CODE BEGIN TIM4_Init 0 */
//
//  /* USER CODE END TIM4_Init 0 */
//
//  TIM_Encoder_InitTypeDef sConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM4_Init 1 */
//
//  /* USER CODE END TIM4_Init 1 */
//  htim4.Instance = TIM4;
//  htim4.Init.Prescaler = 0;
//  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim4.Init.Period = 65535;
//  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
//  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
//  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
//  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
//  sConfig.IC1Filter = 10;
//  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
//  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
//  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
//  sConfig.IC2Filter = 10;
//  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM4_Init 2 */
//
//  /* USER CODE END TIM4_Init 2 */
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
//  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
//                          |LED_3_Pin|CIN1_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);
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
//  /*Configure GPIO pin : CIN2_Pin */
//  GPIO_InitStruct.Pin = CIN2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//  HAL_GPIO_Init(CIN2_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : CIN1_Pin */
//  GPIO_InitStruct.Pin = CIN1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//  HAL_GPIO_Init(CIN1_GPIO_Port, &GPIO_InitStruct);
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
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	// Prevent unused arguments(s) compilation warning
//	UNUSED(huart);
//
//	sprintf(uart_buf,aRxBuffer);
//
//	add_item_to_queue();
//
////	update_motor();
//
////	if(uart_buf[0]='['){
////	move_order temp_move_1;
////	char str_num[5] ="\0";
////	int duration =0;
////
////	temp_move_1.dir = uart_buf[1];
////	temp_move_1.speed = uart_buf[2];
////
////    for(int i =0;i<5;i++){
////        str_num[i] = uart_buf[4+i];
////        str_num[i+1] = '\0';
////    }
////
////    duration = atoi(str_num);
////
////
////	temp_move_1.steps = duration;
////
////	enQueue(temp_move_1);
////
////	}
//
//	aRxBuffer[0] = '/0';
//
//	HAL_UART_Transmit(&huart3,(uint8_t *) aRxBuffer,10, 0xFFFF);
//
//	HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer, 10);
//
//
//
//
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
////	HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1,0xFFFF);
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
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//int front = -1, rear = -1;
//move_order items[SIZE];
//
//
//void enQueue(move_order the_move_order) {
//  if (rear == SIZE - 1)
//    printf("\nQueue is Full!!");
//  else {
//    if (front == -1)
//      front = 0;
//    rear++;
//    items[rear] = the_move_order;
//    printf("\nInserted -> [%c, %d]", the_move_order.dir,the_move_order.steps);
//  }
//}
//
//move_order deQueue(void) {
//  int old_front;
//  if (front == -1){
//
//    printf("\nQueue is Empty!!");
//    move_order empty_move_order;
//    empty_move_order.dir = 'S';
//    empty_move_order.steps = 0;
//    empty_move_order.speed = 'S';
//
//
//    return empty_move_order;
//  }
//  else {
//    printf("\nDeleted : [%c, %d]", items[front].dir,items[front].steps);
//    old_front = front;
//
//    front++;
//    if (front > rear){
//      front = rear = -1;
//    }
//
//    return(items[old_front]);
//  }
//}
//
//int itemInQueue(void){
//  if (front == -1){
//	  return 0;
//  }
//  else{
//	  return 1;
//  }
//}
//
//// Function to print the queue
//void display() {
//  if (rear == -1)
//    printf("\nQueue is Empty!!!");
//  else {
//    int i;
//    printf("\nQueue elements are:\n");
//    for (i = front; i <= rear; i++){
//
//        printf("item %d : -> [%c, %d]",i, items[i].dir,items[i].steps);
//    }
//  }
//  printf("\n");
//}
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//void motor_A_init(void){
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//}
//
//void motor_C_init(void){
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
//}
//
//void steering_init(void){
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
//}
//
//
//void init_motors(void){
//	motor_A_init();
//	motor_C_init();
//	steering_init();
//}
//
//void set_A_clockwise(void){
//	HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
//}
//
//void set_A_anticlockwise(void){
//	HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
//}
//
//void set_C_anticlockwise(void){
//	HAL_GPIO_WritePin(GPIOC,CIN2_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOE,CIN1_Pin,GPIO_PIN_RESET);
//}
//
//void set_C_clockwise(void){
//	HAL_GPIO_WritePin(GPIOE,CIN1_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOC,CIN2_Pin,GPIO_PIN_RESET);
//}
//
//void set_A_pwm(int16_t speed){
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,speed);
//}
//
//void set_C_pwm(int16_t speed){
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,speed);
//}
//
//void move_forward(int16_t speed){
//	set_A_clockwise();
//	set_C_anticlockwise();
//
//	set_A_pwm(speed);
//	set_C_pwm(speed);
//}
//
//void move_backward(int16_t speed){
//	set_C_clockwise();
//	set_A_anticlockwise();
//
//	set_A_pwm(speed);
//	set_C_pwm(speed);
//}
//
//
//void move_tank_turn(void){
//	htim1.Instance->CCR4 = 190 ; //Extreme right 176
//	set_C_clockwise();
//	set_A_clockwise();
//
//	set_A_pwm(7199);
//	set_C_pwm(7199);
//}
//
//void stop_motor(void){
//
//	set_A_pwm(0);
//	set_C_pwm(0);
//}
//
//
//void steer_left(void){
//	mantain_heading = 0;
//
//	htim1.Instance->CCR4 = 95 ; //Extreme right 176
//
//	if(current_wheel_dir =='L'){
//		osDelay(95*10);
//	}
//	current_wheel_dir = 'L';
//
//}
//
//void steer_right(void){
//	mantain_heading = 0;
//
//	htim1.Instance->CCR4 = 220 ; //Extreme right 176
//
//	if(current_wheel_dir =='R'){
//		osDelay(225*10);
//	}
//	current_wheel_dir = 'R';
//}
//
//
//void steer_slightly_right(void){
//	mantain_heading = 0;
//
//	htim1.Instance->CCR4 = 190 ; //Extreme right 176
//	osDelay(100);
//
//}
//
//
//
//void steer_angle(int angle){
//
//	mantain_heading = 1;
//	center_val = center_val; // + ((int)pid_control_val)*10 ;
//	htim1.Instance->CCR4 = angle ; //Extreme right 176
//	osDelay(angle*10);
//}
//
//
//void steer_center(void){
//
//	mantain_heading = 0;
//	center_val = center_val; // + ((int)pid_control_val)*10 ;
//	htim1.Instance->CCR4 = center_val ; //Extreme right 176
//
//	if(current_wheel_dir =='C'){
//		osDelay(center_val*10+5);
//	}
//	current_wheel_dir = 'C';
//
//}
//
//
//void unit_forward(void){
//	steer_center();
//	move_forward(7199);
//	osDelay(110); // 111 with 1 delay on steering
//	stop_motor();
//}
//
//void unit_left(void){
//	steer_left();
//	move_forward(7199);
//	osDelay(210);
//	stop_motor();
//}
//
//void unit_reverse_left(void){
//	steer_left();
//	move_backward(7199);
//	osDelay(210);
//	stop_motor();
//}
//
//void unit_right(void){
//	steer_right();
//	move_forward(7199);
//	osDelay(210);
//	stop_motor();
//}
//
//void round_about(void){
//
//
//	for(;;){
//	htim1.Instance->CCR4 = 190 ; //Extreme right 176
//	move_forward(2000);
//
//	osDelay(10000);
//
//	stop_motor();
//
//	osDelay(5000);
//	}
//}
//
//
//
//void angle_turn(void){
//	int angle =90;
//	float angle_dis = 210/45;
//	float delay = 100+(angle_dis*angle);
//	steer_left();
//	move_forward(7199);
//	osDelay((int)1422);
//	stop_motor();
//}
//
//
//
//
//void unit_reverse_right(void){
//	steer_right();
//	move_backward(7199);
//	osDelay(210);
//	stop_motor();
//}
//
//
//void unit_reverse(void){
//	steer_center();
//	move_backward(7199);
//	osDelay(210);
//	stop_motor();
//}
//
//void spot_turn_right(void){
//	int i;
//	for(i=0;i<3;i++){
//		unit_right();
//		osDelay(500);
//		unit_reverse_left();
//		osDelay(500);
//	}
//	steer_center();
//}
//
//void spot_turn_left(void){
//	int i;
//	for(i=0;i<3;i++){
//		unit_left();
//		osDelay(500);
//		unit_reverse_right();
//		osDelay(500);
//	}
//	steer_center();
//}
//
//void add_item_to_queue(void){
//
//	move_order new_move;
//	char str_num[10];
//
//	char  speed;
//	char  direction;
//	int   duration;
//
//	 if(uart_buf[0] =='['){  //1){//itemInQueue()){ // [FC,00500]
////		prev_dir = direction;
//		speed = uart_buf[1];
//		direction = uart_buf[2];
//
//		for(int i=0;i<5;i++){
//			str_num[i] = uart_buf[4+i];
//			str_num[i+1] = '\0';
//			duration = atoi(str_num);
//		}
//
//		new_move.dir=direction;
//		new_move.speed=speed;
//		new_move.steps=duration;
//
//		enQueue(new_move);
//	 }
//	uart_buf[0]='\0';
//
//}
//
//int counter_1 = 0;
//
//int get_current_distance(void){
//	int distance;
//	distance = (int)(((diff*67)/4752));
//	return distance;
//}
//
//void update_global_yaw(void){
//	current_global_yaw = imu1.Yaw;
//}
//
//void stop_delay_angle(void){
//
//
//	float inital_yaw =0.0;
//	float current_yaw = 0.0;
//	float change_in_yaw = 0.0;
//
//	inital_yaw = current_global_yaw;
//	for(;;){
//		current_yaw = imu1.Yaw;
//		change_in_yaw = current_yaw - inital_yaw;
//		change_in_yaw = fabsf(change_in_yaw);
//		if(change_in_yaw>90){
//			return;
//		}
//		osDelay(20);
//	}
//	return;
//
//
//}
//
//void stop_delay(int units){
//
//	int stop_by_distance = 1;
//	int stop_by_time = 0;
//
//
//	int distance = 0;
//	int inital_distance=0;
//	int change_in_distance =0;
//
//
//	if(stop_by_time){
//		osDelay(units);
//		return;
//	}
//	if(stop_by_distance){
//		inital_distance = get_current_distance();
//		for(;;){
//			distance = get_current_distance();
//			change_in_distance = distance - inital_distance;
//			change_in_distance = abs(change_in_distance);
//			if(change_in_distance>=units){
//				return;
//			}
//			osDelay(20);
//		}
//		return;
//	}
//
//}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//void update_motor(void){
//
////	if(motors_being_updated){
////		return;
////	}
////	else{
////		motors_being_updated=1;
////	}
//
//	char prev_dir = 'F';
//	char direction = 'F';
//	char speed = 'C';
//
//
//
//	char hello1[20] = "";
//	int duration = 1;
//    char str_num[10];
//    move_order current_move;
//
//     if(itemInQueue()){
//    	 current_move = deQueue();
//     }
//     else{
//    	 return;
//     }
//
//	 direction = current_move.dir;
//	 speed = current_move.speed;
//	 duration = current_move.steps;
//
////	 sprintf(hello1,"%c, %c, %d",speed,direction,duration);
////	 OLED_ShowString(10,50,hello1);
//
//	 if(direction != 'C'){
//		 update_global_yaw();
//	 }
//
//	 switch (direction)
//	 {
//		case 'L':
//			steer_left();
//			break;
//		case 'R':
//			steer_right();
//			break;
//		case 'C':
//			steer_center();
//			break;
//		default:
//			;
////			steer_center();
//			break;
//	 }
//
//
//	 switch (speed)
//	 {
//		case 'F':
//			move_forward(2000);
//			break;
//		case 'B':
//			move_backward(2000);
//			break;
//		case 'S':
//			stop_motor();
//			break;
//		default:
//			;
////			stop_motor();
//			break;
//	 }
//
//	 if(direction == 'C'){
//		 stop_delay(duration);
//	 }
//	 else{
//		 stop_delay_angle();
//	 }
//
//	 stop_motor();
//	 osDelay(10);
//
////	 motors_being_updated = 0;
//}
//
//
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
//	init_motors();
//
//
//
//
////	move_order temp_move_1;
////
////	temp_move_1.dir = 'F';
////	temp_move_1.steps = 1;
////	enQueue(temp_move_1);
////
////	//	int k;
//////	for(k=0;k<1;k++){
//////		enQueue(temp_move_1);
//////	}
//
//	move_order current_move;
//
//	char selected_dir = 'S';
//	int remaining_steps = 0;
//
//	char prev_dir = 'F';
//	char direction = 'F';
//	char speed = 'C';
//
//
//	int i;
//
//
//	steer_center();
//	mantain_heading=0;
////
////	//	move_tank_turn();
////	//	round_about();
////
////	steer_right();
////	move_forward(2000);
////	stop_delay_angle();
////	stop_delay(10);
////
////	stop_motor();
//
//
//
//	/* Infinite loop */
//	for(;;)
//	{
//
//
//		update_motor();
//
//		osDelay(100);
//	}
//
//  /* USER CODE END motors */
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
//  int i = 0;
//
//  /* Infinite loop */
//  for(;;)
//  {
//	 i++;
//
//
////	uint8_t hello[20] = "Hello World\0";
//
////	sprintf(hello,"%d, %d",status,i);
//
//	sprintf(hello,"%d, %d",0,i);
//
//    OLED_ShowString(10,10,hello);
//
//    OLED_ShowString(10,30,uart_buf);
//////
////    sprintf(hello,"%s\0",aRxBuffer);
////    OLED_ShowString(10,30,hello);
////    sprintf(aRxBuffer,"\0");
//
////
//
//
////	sprintf(hello,"GyroX:%d\0",(int)(imu1.Roll));
////	OLED_ShowString(10,20,hello);
////	sprintf(hello,"GyroY:%d\0",(int)(imu1.Pitch));
////	OLED_ShowString(10,30,hello);
//
////	sprintf(hello,"imu:%4d %d\0",(int)(center_val),(int)pid_control_val);
////	OLED_ShowString(10,30,hello);
////
//    sprintf(hello,"GyroZ:%4d\0",(int)(imu1.Yaw));
//	OLED_ShowString(10,40,hello);
//
//
//    sprintf(hello,"imu delay %d %d\0",tick_dur,imu_delay);
//    OLED_ShowString(10,50,hello);
////    sprintf(hello,"%s\0",aRxBuffer);
////    OLED_ShowString(10,20,uart_buf);
//
//
////    if(hello[0] == '['){
////    move_order temp_move_1;
////
////	temp_move_1.dir = hello[1];
////	temp_move_1.steps = 1;
////	enQueue(temp_move_1);
////    }
//
//
////  char temp_steps_str[4];
////
////  move_order temp_move_order;
////  temp_move_order.dir = hello[1];
//
//
//    // [%c,%d]
////  int i;
////  for(i=0;hello[i]!='\0';i++){
////	  temp_steps_str[i] = hello[i+2];
////  }
////
////  temp_move_order.steps = hello[3];
//
//
//
////
////	sprintf(hello,"EncoderA:%5s\0",0);
////	OLED_ShowString(10,20,hello);
////	sprintf(hello,"EncoderA Dir:%5s\0",encoderC_val);
////	OLED_ShowString(10,30,hello);
////
////	sprintf(hello,"EncoderC:%5s\0",encoderC);
////	OLED_ShowString(10,40,hello);
////	sprintf(hello,"EncoderC Dir:%5s\0",encoderC);
////	OLED_ShowString(10,50,hello);
//
//
//	OLED_Refresh_Gram();
//    osDelay(500);
//
//  }
//  /* USER CODE END show */
//}
//
///* USER CODE BEGIN Header_Imu_Task */
///**
//* @brief Function implementing the imu_task thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_Imu_Task */
//void Imu_Task(void *argument)
//{
//  /* USER CODE BEGIN Imu_Task */
//	  int i = 0;
//
//	  int imu_delay=0;
//	  int tick,old_tick;
//	  int ticks_left=0;
//	  TM_AHRSIMU_Init(&imu1, 1000.0 / 40.0, beta, inclination);
//
//	  tick = HAL_GetTick();
///* Infinite loop */
//	for(;;)
//	{
//		  i++;
//		  old_tick = tick;
//		  tick = HAL_GetTick();
//		  tick_dur = tick-old_tick;
//		  imu_delay = 40 - tick_dur;
//		  if(imu_delay<0){
//			  imu_delay =0;
//		  }
//		  for(int i =0;i<17;i++){
//			  tick = HAL_GetTick();
//			  tick_dur = tick-old_tick;
//			  ticks_left = 40 - tick_dur;
//			  if(ticks_left<=0){
//				  break;
//			  }
//			  osDelay(2);
//		  }
//	//////////////////////////////////////////////////////////////////////////////////////
//		  IMU_AccelRead(&imu);
//		  IMU_GyroRead(&imu);
//
//
//		  TM_AHRSIMU_UpdateAHRS(&imu1, AHRSIMU_DEG2RAD(imu.gyro[0]), AHRSIMU_DEG2RAD(imu.gyro[1]), AHRSIMU_DEG2RAD(imu.gyro[2]), imu.acc[0], imu.acc[1], imu.acc[2], 0.0, 0.0, 0.0);
//
//
//	//////////////////////////////////////////////////////////////////////////////////////
//	}
//  /* USER CODE END Imu_Task */
//}
//
///* USER CODE BEGIN Header_PidTask */
///**
//* @brief Function implementing the pidTask thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_PidTask */
//void PidTask(void *argument)
//{
//  /* USER CODE BEGIN PidTask */
//
//    PIDController pid = { PID_KP, PID_KI, PID_KD,
//                          PID_TAU,
//                          PID_LIM_MIN, PID_LIM_MAX,
//						  PID_LIM_MIN_INT, PID_LIM_MAX_INT,
//                          100 };
//
//    PIDController_Init(&pid);
//
//
//   float setpoint = 0.0f;
//
//
//
//  /* Infinite loop */
//  for(;;)
//  {
//
////	  if(mantain_heading){
////
////	    setpoint = imu1.Yaw;
////		int center_val = 150;
////
////		/* Get measurement from system */
////		float measurement = imu1.Yaw;
////
////		/* Compute new control signal */
////		PIDController_Update(&pid, setpoint, measurement);
////
////		pid_control_val = pid.out;
////
////		steer_center();
////	  }
//	  osDelay(100);
//  }
//  /* USER CODE END PidTask */
//}
//
///* USER CODE BEGIN Header_EncoderTask */
///**
//* @brief Function implementing the encoderTask thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_EncoderTask */
//void EncoderTask(void *argument)
//{
//  /* USER CODE BEGIN EncoderTask */
//	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
//
//	uint32_t tick;
//
//	cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
//	tick = HAL_GetTick();
//	uint8_t hello[20];
//	uint16_t dir;
//	int distance = 0;
//
//  /* Infinite loop */
//  for(;;)
//  {
//	if(HAL_GetTick()-tick > 1000L){
////		if(update_encoder_start){
////			cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
////			update_encoder_start=0;
////		}
//		cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
//		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
//
//			if(cnt2<cnt1)
//				diff = cnt1 - cnt2;
//			else
//				diff  = (cnt2%265535) + cnt1;
//		}
//		else{
//			if(cnt2>cnt1)
//				diff = cnt2 - cnt1;
//			else
//				diff  = (cnt1%65535) + cnt2;
//			}
//		distance = (int)(((diff*67)/4752));
//
//		sprintf(hello,"Speed:%5d\0",distance);
//		OLED_ShowString(10,20,hello);
//		dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
//		sprintf(hello,"Dir:%5d\0",dir);
////		OLED_ShowString(10,30,hello);
//		//cnt1 = cnt2;//__HAL_TIM_GET_COUNTER(&htim2);
//		tick = HAL_GetTick();
//		osDelay(5);
//	}
//  }
//
//  /* USER CODE END EncoderTask */
//}
//
///* USER CODE BEGIN Header_SteeringTask */
///**
//* @brief Function implementing the steeringTask thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_SteeringTask */
//void SteeringTask(void *argument)
//{
//  /* USER CODE BEGIN SteeringTask */
//  /* Infinite loop */
//  float inital_yaw=0;
//  float current_yaw=0;
//  float diff_yaw =0;
//  int first_update = 1;
//  int current_angle = 150;
//
//  for(;;)
//  {
//	if(mantain_heading){
//		if(first_update){
//			current_angle = 150;
//			inital_yaw = imu1.Yaw;
//			first_update=0;
//		}
//
//		current_yaw = imu1.Yaw;
//		diff_yaw = current_yaw - inital_yaw;
//
//
//		if(diff_yaw<0){
//			if(current_angle>150){
//				current_angle=150;
//			}
//			current_angle=150+diff_yaw;
//		}
//
//		if(diff_yaw>0){
//			if(current_angle<150){
//				current_angle=150;
//			}
//			current_angle=150+diff_yaw;
//		}
//
//
//		steer_angle(current_angle);
//
//
//	}
//
//    osDelay(10);
//  }
//  /* USER CODE END SteeringTask */
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
