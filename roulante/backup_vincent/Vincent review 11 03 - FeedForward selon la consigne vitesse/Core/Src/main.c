/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/*  The following Table  describes the TIM1 Channels states (counterclockwise):
  -----------------------------------------------------------
  |Hall 1/2/3|  101  |  100  |  110  |  010  |  011  |  001  |
   ----------------------------------------------------------
  | aH-Ch1 |   0   |   0   |   0   |   1   |   1   |   0   |
   ----------------------------------------------------------
  | aL-Ch1N|   1   |   1   |   0   |   0   |   0   |   0   |
   ----------------------------------------------------------
  | bH-Ch2 |   1   |   0   |   0   |   0   |   0   |   1   |
   ----------------------------------------------------------
  | bL-Ch2N|   0   |   0   |   1   |   1   |   0   |   0   |
   ----------------------------------------------------------
  | cH-Ch3 |   0   |   1   |   1   |   0   |   0   |   0   |
   ----------------------------------------------------------
  | cL-Ch3N|   0   |   0   |   0   |   0   |   1   |   1   |
   ----------------------------------------------------------

   001 -> 011 -> 010 -> 110 -> 100 -> 101
*/

/*  The following Table  describes the TIM1 Channels states (clockwise):
  -----------------------------------------------------------
  |Hall 1/2/3|  101  |  100  |  110  |  010  |  011  |  001  |
   ----------------------------------------------------------
  | aH-Ch1 |   1   |   1   |   0   |   0   |   0   |   0   |
   ----------------------------------------------------------
  | aL-Ch1N|   0   |   0   |   0   |   1   |   1   |   0   |
   ----------------------------------------------------------
  | bH-Ch2 |   0   |   0   |   1   |   1   |   0   |   0   |
   ----------------------------------------------------------
  | bL-Ch2N|   1   |   0   |   0   |   0   |   0   |   1   |
   ----------------------------------------------------------
  | cH-Ch3 |   0   |   0   |   0   |   0   |   1   |   1   |
   ----------------------------------------------------------
  | cL-Ch3N|   0   |   1   |   1   |   0   |   0   |   0   |
   ----------------------------------------------------------

   101 -> 100 -> 110 -> 010 -> 011 -> 001
*/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "functions.h"
#include "pls.h"

//p number of poles pairs
// !!WARNING : IF CHANGED DEFINE, UPDATE FUNCTIONS.h FILE !!
#define P 8
#define FREQ_TIM7 10000
#define MAX_SPEED 20000
#define MIN_SPEED -20000
#define DC_MIN -85
#define DC_MAX 85


//hall sensors value
struct hall hall_L = {.h1=0,.h2=0,.h3=0,.h123=0,.prev_h123=0,.tickS=0,.tickP=0};
struct hall hall_R = {.h1=0,.h2=0,.h3=0,.h123=0,.prev_h123=0,.tickS=0,.tickP=0};

//enable bits for PWM : sense=1 -> motor clockwise // sense=0 -> motor anti clockwise
struct PWM PWM_L = {.aH=0,.aL=0,.bH=0,.bL=0,.cH=0,.cL=0,.sense=1};
struct PWM PWM_R = {.aH=0,.aL=0,.bH=0,.bL=0,.cH=0,.cL=0,.sense=0};

//speed control
struct dataSpeed dataSpeed_L = {.speed_ref=0,.speed=0,.error=0,.prev_error=0,.cmd=0,.cmdsat=0,.prev_cmd=0,.prev_cmdsat=0};
struct dataSpeed dataSpeed_R = {.speed_ref=0,.speed=0,.error=0,.prev_error=0,.cmd=0,.cmdsat=0,.prev_cmd=0,.prev_cmdsat=0};

//SYSTEM COMMAND
//int16_t speed_refs[3]={3500,6000,0}; 	//in ms : 15sec at Fe=1kHz acquition

//PID settings
struct PID Speed_P = {.Kp=0.008f,.w_i=0,.Te=0.01f,.a0=0,.a1=0};
//struct PID Speed_PI = {.Kp=0.008f,.w_i=7,.Te=0.01f,.a0=0.00828,.a1=0.00772};
//struct PID Speed_PI = {.Kp=0.012f,.w_i=7,.Te=0.01f,.a0=0.01242,.a1=0.01158};
//struct PID Speed_PI = {.Kp=0.02f,.w_i=7,.Te=0.01f,.a0=0.0207,.a1=0.0193};
//struct PID Speed_PI = {.Kp=0.015f,.w_i=10,.Te=0.01f,.a0=0.021,.a1=0.0145};
//struct PID Speed_PI = {.Kp=0.018f,.w_i=7,.Te=0.01f,.a0=0.0105,.a1=0.0095};
struct PID Speed_PI = {.Kp=0.018f,.w_i=7,.Te=0.01f,.a0=0.0189,.a1=0.0171};




uint8_t millis_cpt=0; 	//to generate 1ms time
uint8_t millis_div=0;  //counter to match with TIM freq, to adapt to 1000Hz freq
uint16_t millis=0;    	//1ms time step

const uint16_t N=14499; 		//sample number for system identification

int16_t cpt=0;

//enable for SVI

bool en_SVI=0;
bool en_MOVE=0;
bool en_STOP=0;
bool ended=0;
volatile uint16_t i=0; //read speed data

uint16_t samples=1000;
int16_t speedRef_L[1000]={0}; //10s at 100Hz
int16_t speedRef_R[1000]={0}; //10s at 100Hz
//volatile int16_t* speedRef_L;
//volatile int16_t* speedRef_R;

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
 ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
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

	//initialisation variables

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  //PWM START TIMER1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  //PWM START TIMER8
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);


  //START INTERRUPTS
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  TIM7->ARR = FREQ_TIM7/100; //set TIM7 interrupt frequency 100Hz

  //set interrupts priority
  //void HAL_NVIC_SetPriority (IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)

  //tim1 priority, tim7, tim6
  //HAL_NVIC_SetPriority (TIM1 IRQn, uint32_t PreemptPriority, uint32_t SubPriority);

  //generate speed command (STEP)
  //void generateStep(uint16_t t0, uint8_t value0, uint16_t t1, uint8_t value1, uint16_t samples,  uint16_t  data[])
  //generateStep(10, 1000, 5000, 2500, N, speed_data);
  //volatile int16_t speedRef_R[1000]={0}; //10s at 100Hz


  // serial protocol return parameters
  int16_t distance_robot=1000; //distance for the robot in mm
  int16_t angle_robot=90; //angle for the robot in degree
  uint16_t speed_max = 1000; //maximum speed reference possible (1000-1500 ticks/s)

  int16_t id = SRO; //give a distance SPO, give an angle SRO

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  switch(id) {
		  case SPO:
			  parseurSpeed(distance_robot,0,speed_max);
			  en_MOVE=1;
			  millis=0;
			  i=0;
			  id=0xff;
			  break;


		  case SRO:
			  parseurSpeed(0,angle_robot,speed_max);
			  en_MOVE=1;
			  millis=0;
			  i=0;
			  id=0xff;
			  break;

		  default:
			  break;
	  }

  }
}
  /* USER CODE END 3 */


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 8;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 28-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 2;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 20-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8400-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 28-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 100-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 2;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EN_B2_Pin|EN_B3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PUSHER_GPIO_Port, PUSHER_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : H1_L_Pin H2_L_Pin H3_L_Pin H1_R_Pin
                           H2_R_Pin H3_R_Pin */
  GPIO_InitStruct.Pin = H1_L_Pin|H2_L_Pin|H3_L_Pin|H1_R_Pin
                          |H2_R_Pin|H3_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BAT_INT_Pin */
  GPIO_InitStruct.Pin = BAT_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BAT_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_B2_Pin EN_B3_Pin */
  GPIO_InitStruct.Pin = EN_B2_Pin|EN_B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PUSHER_Pin */
  GPIO_InitStruct.Pin = PUSHER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PUSHER_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * General purpose functions
 */


/**
 * Interrupt function
 *
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	//check the timer version (generic function)
	if (htim == &htim6)
	/* 50kHz interrupt for hall reading and Moore state machine calculations
	*/

	{

		if(en_STOP)
		{
			//stop all
			dataSpeed_L.cmd = 0; dataSpeed_R.cmd = 0;
			PWM_L.aH=0; PWM_L.aL=0; PWM_L.bH=0; PWM_L.bL=0; PWM_L.cH=0; PWM_L.cL=0; PWM_L.sense=0;
			PWM_R.aH=0; PWM_R.aL=0; PWM_R.bH=0; PWM_R.bL=0; PWM_R.cH=0; PWM_R.cL=0; PWM_R.sense=1;
		}
		else
		{

			//time generator
			if (millis < N)//NOOOOKKK
			{
				if (millis_cpt >=50) //millis -> 1ms
				{
					millis++;	millis_cpt=0;
				}
				else
				{
					millis_cpt++;
				}
			}

			if(en_SVI || en_MOVE)
			{
				//read hall values
				readHall_L(&hall_L);	readHall_R(&hall_R);

				//calculate gate enable bits
				decodeHall(&hall_L,&PWM_L); decodeHall(&hall_R,&PWM_R);

				//change output values
				updateOutput_L(PWM_L); updateOutput_R(PWM_R);
			}
		}
	}



	if (htim == &htim7)
	{
	/* Update duty cycle if needed at 100Hz
	*/
		if(!en_STOP)
		{
			if(en_SVI || en_MOVE)
			{
				//COmmand send to motor
				if (millis<10000)
				{

					dataSpeed_L.speed_ref=speedRef_L[i];
					dataSpeed_R.speed_ref=speedRef_R[i];
					i++;


					//dataSpeed_L.speed_ref=5000;
					//dataSpeed_R.speed_ref=5000;
				}
				else
				{
					dataSpeed_L.speed_ref=0;
					dataSpeed_R.speed_ref=0;
				}
			}
		}

		else
		{
			//end command
			dataSpeed_L.speed_ref=0;
			dataSpeed_R.speed_ref=0;

		}

		//Calculate speed correction
		PIDS_PI(&Speed_PI, &dataSpeed_L,&dataSpeed_R,&hall_L, &hall_R);

		//update PWM level with min/max saturation
		writeCommand_L(&PWM_L,&dataSpeed_L);
		writeCommand_R(&PWM_R,&dataSpeed_R);

		//printf("ref L : %d | mes L : %d |  cmd L : %d\n",dataSpeed_L.speed_ref,dataSpeed_L.speed,dataSpeed_L.cmdsat);
		//printf("ref R : %d | mes R : %d |  cmd R : %d\n",dataSpeed_R.speed_ref,dataSpeed_R.speed,dataSpeed_R.cmdsat);
		//reset variables
		dataSpeed_L.prev_cmd=dataSpeed_L.cmd;
		dataSpeed_L.prev_error=dataSpeed_L.error;
		dataSpeed_R.prev_cmd=dataSpeed_R.cmd;
		dataSpeed_R.prev_error=dataSpeed_R.error;

		if(dataSpeed_L.speed_ref==0 && calculateSpeed(&hall_L)==0 & dataSpeed_R.speed_ref==0 && calculateSpeed(&hall_R)==0)
		{
			en_SVI=0;
			en_MOVE=0;
			en_STOP=0;
			ended=1;


			//reset all variables needed for STOP
			millis=0; millis_div=0; i=0;
			hall_L.h1=0;hall_L.h2=0;hall_L.h3=0;hall_L.tickS=0;
			hall_R.h1=0;hall_R.h2=0;hall_R.h3=0;hall_R.tickS=0;

			//reverse sense (FIXME : reverse speed)
			PWM_L.aH=0;PWM_L.aL=0;PWM_L.bH=0;PWM_L.bL=0;PWM_L.cH=0;PWM_L.cL=0;PWM_L.sense=0;
			PWM_R.aH=0;PWM_R.aL=0;PWM_R.bH=0;PWM_R.bL=0;PWM_R.cH=0;PWM_R.cL=0;PWM_R.sense=1;

			//speed control
			dataSpeed_L.speed_ref=0;dataSpeed_L.speed=0;dataSpeed_L.error=0;dataSpeed_L.prev_error=0;dataSpeed_L.cmd=0;dataSpeed_L.prev_cmd=0;
			dataSpeed_R.speed_ref=0;dataSpeed_R.speed=0;dataSpeed_R.error=0;dataSpeed_R.prev_error=0;dataSpeed_R.cmd=0;dataSpeed_R.prev_cmd=0;
		}





	}


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
