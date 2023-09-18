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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t step1 = 2000; //200 на оборот
uint16_t step2 = 8000; //40 оборотов
uint16_t timeAll = 40; //40 секунд //   берем мотор с більшоб кількістю кроків рауємо його час та розраховуємо швидкість імпульсів для мотора 2

int steppingyakkazavmaxim(float stepM1, float stepM2) {
	uint32_t psc_max=72; //72 мгц / 72 - 1 мегагерц = 1000 за секунду
	//числа 1, 2, 3, 4, 6, 8, 9, 12, 18, 24, 36 и 72 - Це можлива обрана максимальна швидкість для мотора з більшої кількістю кроків. Це дільник таймера
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim2);

	//  частота шим = входящая частота / период (arr)
	//  125 000 (125 килогерц)  = 16 000 000 / 128
	// (1/60)*1000 = частота 16 (герц);
float periodM1=60;
uint32_t psc=1800;

if(stepM1>stepM2){

	htim1.Instance->PSC = psc;
	htim1.Instance->ARR = periodM1;
	htim1.Instance->CCR3 = periodM1/2;

	float delimiter= stepM1/stepM2;
	float mnoj= ceil(periodM1*delimiter);

	htim2.Instance->PSC = psc;
	htim2.Instance->ARR = mnoj;
	htim2.Instance->CCR3 = mnoj/2;

} else if (stepM1<stepM2) {
//	uint8_t delimiter=stepM2/stepM1;
//	uint16_t impMore = (72000000/psc_max)/1000; 						//імпульсів кроків за секунду для мотора з більшої кількістю кроків  КРОКІВ НА СЕКУНДУ
//	uint16_t allSecMore = (stepM2/impMore)*1000;		 				//загальний час роботи мотора із більшої кількістю кроків  мілісекунд
//	uint16_t stepSecM1 =  (stepM1/allSecMore)*1000; 					//кроків на секунду на двигуна LESS  250
//	uint16_t PSCmLess= 72000000 / (stepSecM1 * 1000); 					//дільник для мотора LESS

	htim2.Instance->PSC = psc;
	htim2.Instance->ARR = periodM1;
	htim2.Instance->CCR3 = periodM1/2;

	float delimiter= stepM2/stepM1;
	float mnoj= ceil(periodM1*delimiter);

	htim1.Instance->PSC = psc;
	htim1.Instance->ARR = mnoj;
	htim1.Instance->CCR3 = mnoj/2;

}

	//Старт таймера та переривань
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);

	return 0;
}


int timetostep(int stepM1, int dir1, int stepM2, int dir2, int time, int pPulse) {
	double FREQstepM1 = 0, FREQstepM2 = 0, PSC_T1 = 0, PSC_T2 = 0;

	if (pPulse < 1 && pPulse > 100) {
		return 1;
	}

	HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, dir1);
	HAL_GPIO_WritePin(Dir2_GPIO_Port, Dir2_Pin, dir2);

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim2);

//	if((stepM1 % time) != 0 || (stepM2 % time) != 0  ) {
//		printf("Figovu steps suka");
//		return 1;
//	} else {

	FREQstepM1 = __builtin_ceil(stepM1 / time); // шагов в сек м1
	FREQstepM2 = __builtin_ceil(stepM2 / time); //шагов в сек м2

	PSC_T1 = 72000000 / (FREQstepM1 * 1000); //дільник частоти
	PSC_T2 = 72000000 / (FREQstepM2 * 1000);

	htim1.Instance->PSC = PSC_T1;
	htim1.Instance->ARR = 1000;  //ось тут треба глянуть щоб все було добре  на осцилографі
	htim1.Instance->CCR3 = pPulse*10; //довжина імпульсу

	htim2.Instance->PSC = PSC_T2;
	htim2.Instance->ARR = 1000;
	htim2.Instance->CCR3 = pPulse*10;

	//Старт таймера та переривань
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);

	return 0;
}


int cntImpulse1 = 0, cntImpulse2 = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) /*CallBack, вызванный при окончании периода Ш�?М*/
{
//	if (htim->Instance == TIM1)/*Проверяем от какого таймера пришёл CallBack тут надо проверить точность*/
//	{
//		++cntImpulse1;
//
//		HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
//		if (cntImpulse1 >= step1) {
//			HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, GPIO_PIN_SET);
//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
//			HAL_TIM_Base_Stop_IT(&htim1);
//		}
//
//	} else if (htim->Instance == TIM2) {
//
//		++cntImpulse2;
//
//		if (cntImpulse1 >= step1) {
//			HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, GPIO_PIN_SET);
//			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
//			HAL_TIM_Base_Stop_IT(&htim2);
//		}
//	}
}

//void Stepper_motor(uint8_t Direction, uint8_t Steps)/*Функция управления шаговым двигателем. В функции 2 аргумента:
// 1 -- определяет направление <i>, 2 -- определяет количество шагов.*/
//{
//
//	HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, GPIO_PIN_RESET);/*Разрешаем работу двигателя*/
//	switch (Direction) {/*Если переменная, определяющая направление вращения,*/
//	case 1:/*равна 1, то*/
//		HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, GPIO_PIN_SET);/*включаем пин, отвечающий за направление вращения.*/
//		break;
//	default:/*Если любое другое значение, то*/
//		HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, GPIO_PIN_RESET);/*отключаем пин, отвечающий за направление вращения.*/
//		break;
//	}
//	if (i < Steps)/*Если значение счётчика i меньше переменной, определяющей количество шагов,*/
//	{
//		/*то прибавим к i единицу*/
//		// pwm = 990;/*и приравняем переменную pwm к 500.*/
//	} else/*�?наче, если i будет равна переменной, определяющей количество шагов,*/
//	{
//		i = 0; /*приравниваем i к нулю,*/
//		/*Обнулим элемент массива, отвечающего за количество шагов,*/
//		pwm = 0;/*приравниваем pwm к нулю и*/
//		HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, GPIO_PIN_SET);/*запрещаем работу двигателя*/
//		HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
//	}
//}

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(En_GPIO_Port,En_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Dir2_GPIO_Port,Dir2_Pin, GPIO_PIN_SET);

 // timetostep(20000, 1, 5000, 0, 40, 50);

  steppingyakkazavmaxim(5000, 10000);
  /* USER CODE END 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, En_Pin|Dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Dir2_GPIO_Port, Dir2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : En_Pin Dir_Pin */
  GPIO_InitStruct.Pin = En_Pin|Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Dir2_Pin */
  GPIO_InitStruct.Pin = Dir2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Dir2_GPIO_Port, &GPIO_InitStruct);

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
	while (1) {
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
