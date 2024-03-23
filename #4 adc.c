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
#include "stdbool.h"

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
#define ARM_CM_DEMCR (*(uint32_t *)0xE000EDFC)
#define ARM_CM_DWT_CTRL (*(uint32_t *)0xE0001000)
#define ARM_CM_DWT_CYCCNT (*(uint32_t *)0xE0001004)
#define CPU_CLOCK_FREQ ((double)100000000)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t TimerCnt;
static uint8_t DisplayCOM = 0;
static uint8_t DisplayNumbers[4] = {-1, -1, -1, -1};
int ButtTime = 0;

#define BUFFER_LEN (3000)
uint32_t BufferT[BUFFER_LEN];
uint16_t BufferAD[BUFFER_LEN];
uint32_t idx = 0;
uint32_t max_idx = -1;

bool IS_FULL = false;
bool READ_DATA = false;

void set_number(){
	switch(DisplayCOM){
	case 0:
		HAL_GPIO_WritePin(GPIOC, COM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, COM2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM4_Pin, GPIO_PIN_SET);
		DisplayCOM = 1;
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOC, COM1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, COM3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM4_Pin, GPIO_PIN_SET);
		DisplayCOM = 2;
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOC, COM1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, COM4_Pin, GPIO_PIN_SET);
		DisplayCOM = 3;
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOC, COM1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM4_Pin, GPIO_PIN_RESET);
		DisplayCOM = 0;
		break;
	}

	switch(DisplayNumbers[DisplayCOM]){
	case 0:
		HAL_GPIO_WritePin(GPIOC, segA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segB_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segF_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segG_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segDP_Pin, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOC, segA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segB_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segD_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segF_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segG_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segDP_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOC, segA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segB_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segF_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segG_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segDP_Pin, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOC, segA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segB_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segF_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segG_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segDP_Pin, GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOC, segA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segB_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segD_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segF_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segG_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segDP_Pin, GPIO_PIN_RESET);
		break;
	case 5:
		HAL_GPIO_WritePin(GPIOC, segA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segF_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segG_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segDP_Pin, GPIO_PIN_RESET);
		break;
	case 6:
		HAL_GPIO_WritePin(GPIOC, segA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segF_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segG_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segDP_Pin, GPIO_PIN_RESET);
		break;
	case 7:
		HAL_GPIO_WritePin(GPIOC, segA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segB_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segD_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segF_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segG_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segDP_Pin, GPIO_PIN_RESET);
		break;
	case 8:
		HAL_GPIO_WritePin(GPIOC, segA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segB_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segF_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segG_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segDP_Pin, GPIO_PIN_RESET);
		break;
	case 9:
		HAL_GPIO_WritePin(GPIOC, segA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segB_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segF_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segG_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, segDP_Pin, GPIO_PIN_RESET);
		break;
	default:
		HAL_GPIO_WritePin(GPIOC, segA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segD_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segF_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segG_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, segDP_Pin, GPIO_PIN_RESET);
	}
}

void display(uint8_t tab[]){
	DisplayCOM = 0;
	for (int i=0; i < 4; i++){
		DisplayNumbers[i] = tab[i];
	}

}

void display_int(int num){
	uint8_t tab [4];
	for (int i=3; i>=0; i--){
		tab[i] = num % 10;
		num = num / 10;
	}
	display(tab);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if( htim == &htim1 ){
		set_number();
	}
	else if( htim == &htim4 ) {
		if (TimerCnt >= 100){
//			HAL_ADC_Start(&hadc1);
//			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//			int ADCRes = HAL_ADC_GetValue(&hadc1); // 0-4095 == 0-3.3V
//			ADCRes = (uint16_t)(3.3 * (double)ADCRes / 4.095); // [mV]
			if (READ_DATA){
				display_int(BufferAD[idx-1]);
			}
			else{
				uint8_t tab[4] = {-1, -1, -1, -1};
				(display(tab));
			}



//			BufferT[idx] = ARM_CM_DWT_CYCCNT;
//			BufferAD[idx] = ADCRes;
//
//			idx++;
//			if (idx == BUFFER_LEN-1){
//				IS_FULL = 1;
//				idx = 0;
//			}

			TimerCnt = 0;
		}
		else
			TimerCnt++;
	}
}

void button_click(uint8_t button_number){
	int tick = HAL_GetTick();
	if (tick-ButtTime > 200){
		ButtTime = tick;
		if (button_number == 0){
			if (DisplayNumbers[1] == 9){
				DisplayNumbers[0]++;
				DisplayNumbers[1] = 0;
			}
			else{
				DisplayNumbers[1]++;
			}
		}
		else {
			if (DisplayNumbers[3] == 9){
				DisplayNumbers[2]++;
				DisplayNumbers[3] = 0;
			}
			else{
				DisplayNumbers[3]++;
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == Button1_Pin){
		READ_DATA = true;
		idx = 0;
//		button_click(0);
//		//printf("button1\r\n");
	}
	if(GPIO_Pin == Button2_Pin){
		READ_DATA = false;
		max_idx = idx;
		IS_FULL = true;
//		button_click(1);
//		//printf("button2\r\n");
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (READ_DATA){
		int WYNIK_ADC = HAL_ADC_GetValue(&hadc1);
		int ADCRes = (uint16_t)(3.3 * (double)WYNIK_ADC / 4.095); // [mV]
		//display_int(ADCRes);

		BufferT[idx] = ARM_CM_DWT_CYCCNT;
		BufferAD[idx] = ADCRes;

		idx++;
		if (idx == BUFFER_LEN-1){
			IS_FULL = 1;
			max_idx = idx;
			READ_DATA = false;
		}
	}
}

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
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
  if (ARM_CM_DWT_CTRL != 0) { // See if DWT is available
	  ARM_CM_DEMCR |= 1 << 24; // Set bit 24
	  ARM_CM_DWT_CYCCNT = 0;
	  ARM_CM_DWT_CTRL |= 1 << 0; // Set bit 0
  }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  if (IS_FULL){
//		  for (int i = 0; i < BUFFER_LEN; i++){
//				printf("%li %i, \r\n", BufferT[i], BufferAD[i]);
//		  }
//		  IS_FULL = false;
//	  }
	  if (IS_FULL){
		  for (int i = 0; i < max_idx; i++){
				printf("%li %i, \r\n", BufferT[i], BufferAD[i]);
		  }
		  IS_FULL = false;
		  max_idx = -1;
	  }
//	  int Start = ARM_CM_DWT_CYCCNT;
//	  HAL_Delay(1000);
//	  int Stop = ARM_CM_DWT_CYCCNT;
//	  printf("HAL_Delay(1000): %d [us]\r\n", (unsigned)(1e6*((double)Stop-(double)Start)/CPU_CLOCK_FREQ) );
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
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, segG_Pin|segD_Pin|segE_Pin|segC_Pin
                          |segB_Pin|segF_Pin|segA_Pin|segDP_Pin
                          |COM3_Pin|COM2_Pin|COM1_Pin|COM4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : segG_Pin segD_Pin segE_Pin segC_Pin
                           segB_Pin segF_Pin segA_Pin segDP_Pin
                           COM3_Pin COM2_Pin COM1_Pin COM4_Pin */
  GPIO_InitStruct.Pin = segG_Pin|segD_Pin|segE_Pin|segC_Pin
                          |segB_Pin|segF_Pin|segA_Pin|segDP_Pin
                          |COM3_Pin|COM2_Pin|COM1_Pin|COM4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Button1_Pin Button2_Pin */
  GPIO_InitStruct.Pin = Button1_Pin|Button2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
