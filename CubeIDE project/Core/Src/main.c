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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_STEPS 4000					// steps of the encoder per revolution times four (due to quadrature read out)
#define PWM_STEPS 1400						// The PWM timer counts from 0 to PWM_STEPS-1 with a clock of 70 MHz (PWM_STEPS 1400 results in 50 kHz PWM frequency)
#define ADC_REF_VOLTAGE 3.3
#define ADC_STEPS 4096						// 12 bit resolution
#define SUPPLY_VOLTAGE_DIVIDER_RATIO 11.0	// the voltage on the adc channel monitoring the supply voltage is V_supply/SUPPLY_VOLTAGE_DIVIDER_RATIO
#define CURRENT_SENSOR_SENSITIVITY 0.3126	// sensitivity of the current sensor in V/A
#define TS 250								// period of the main loop in microseconds (sampling time of the current controller)
#define ALIGNMENT_TIME 3000					// for ALIGNMENT_TIME milliseconds, motor phase A is driven with approximately I_MAX in order to define the rotor zero position in which the d-axis and the a-axis are aligned
#define I_MAX 1.5		    				// maximal i_d, i_q currents
#define OVERCURRENT_LIMIT 1.9     			// i_a or i_b exceeding this current leads to a shutdown (over current protection is only active when the current controllers are running, not during the alignment phase)
#define MINIMAL_SUPPLY_VOLTAGE 8.0  		// no PWM output if the supply voltage is below MINIMAL_SUPPLY_VOLTAGE
#define PWM_STEPS_OFFSET 85					// due to the dead time of the H-bridge ic, a compare value of PWM_STEPS_OFFSET still produces zero output (PWM_STEPS_OFFSET is needed for compensating this effect)

// motor parameters
#define L 1.2E-3	// phase inductance
#define R 0.7		// phase resistance (taking into account H-bridge and wiring resistance)
#define NR 50		// number of rotor teeth (50 for a standard stepper motor with 1.8 degree step angle)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
volatile char triggered = 0;	/* This variable is set periodically in an
interrupt, whenever time Ts has passed. When set, the control algorithm in the main loop is executed. */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM16_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void set_output_a(float v_a, float v_supply);
void set_output_b(float v_b, float v_supply);

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

  // generate sin and cos tables (although periodic, generate them for all the NR electrical revolutions since there is enough memory available)
  float* sin_table = malloc(ENCODER_STEPS * sizeof(float));
  float* cos_table = malloc(ENCODER_STEPS * sizeof(float));
  for (int i = 0; i < ENCODER_STEPS; i++) {
	  float arg = i * 2 * NR * M_PI / ENCODER_STEPS;
	  sin_table[i] = (float)sin(arg);
	  cos_table[i] = (float)cos(arg);
  }

  // calculate the controller parameters
  // current controller C(z)=V*(z-exp(-R/L * Ts))/(z-1) results in closed-loop pole at 1-V/R*(1-exp(-R/L * Ts)), which must be within the unit circle
  const float E = exp(-R/L * TS * 1E-6);
  const float V = (6.0/I_MAX) < (R/(1-exp(-R/L * TS * 1E-6))) ? (6.0/I_MAX) : (R/(1-exp(-R/L * TS * 1E-6)));	/* controller gain such that for a step input i_d_ref (i_q_ref) with height I_MAX, the voltage v_d (v_q) does not exceed 6 V
 	 and such that the closed-loop pole is within the interval [0,1) */

  // calculate the conversion factors for converting the ADC readings to SI units (volts or amps)
  const float ADC_SUPPLY_VOLTAGE_CONVERSION_FACTOR = ADC_REF_VOLTAGE * SUPPLY_VOLTAGE_DIVIDER_RATIO / (float)ADC_STEPS;
  const float ADC_CURRENT_CONVERSION_FACTOR = ADC_REF_VOLTAGE / CURRENT_SENSOR_SENSITIVITY / (float)ADC_STEPS;
  const float ADC_REF_INPUTS_CONVERSION_FACTOR = 2 * I_MAX / (float)ADC_STEPS;

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
  MX_DMA_Init();
  MX_TIM16_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 0;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);		// phase B in_A
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);		// phase A in_A
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);		// phase A in_B
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);		// phase B in_B
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim16);

  uint16_t adc_buf[7];		// ordering: adc_buf[0]: supply voltage
  	  	  	  	  	  	  	//			 adc_buf[1]: current sensor phase B ref
  	  	  	  	  	  	  	//			 adc_buf[2]: current sensor phase B out
  	  	  	  	  	  	  	//			 adc_buf[3]: current sensor phase A ref
    	  	  	  	  	  	//			 adc_buf[4]: current sensor phase A out
  	  	  	  	  	  	    //			 adc_buf[5]: i_d ref
      	  	  	  	  	  	//			 adc_buf[6]: i_q ref

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float v_supply;
  float i_a, i_b;
  float v_a, v_b;
  float i_d,i_q;
  float v_d, v_q;
  float i_d_ref, i_q_ref;

  int cnt;
  float sin_value;
  float cos_value;

  float e_i_d, e_i_q;
  float e_i_d_prev = 0;
  float e_i_q_prev = 0;
  float v_d_prev = 0;
  float v_q_prev = 0;

  int limit_hit_flag = 0;

  HAL_Delay(100);
  // turn on green LED to indicate ready state and wait for button to be pressed before continuing
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);			// turn on green LED
  while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET);	// wait for button being pressed
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);			// turn off green LED

  /* Hold the current in phase A at approximately I_MAX for ALIGNMENT_TIME milliseconds. The position in which the rotor settles defines the
   * zero position in which the d-axis and the a-axis are aligned. */
  v_supply = (float)adc_buf[0] * ADC_SUPPLY_VOLTAGE_CONVERSION_FACTOR;
  //set_output_a(R * I_MAX, v_supply);
  set_output_a(R * I_MAX, v_supply);
  HAL_Delay(ALIGNMENT_TIME);
  TIM2->CNT = 0;
  set_output_a(0.0, 0.0);
  HAL_Delay(100);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);			// turn on green LED
  while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET);	// wait for button being pressed
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);			// turn off green LED

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (triggered) {
		triggered = 0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);		// allows for measuring the execution time of one cycle of the control algorithm via a oscilloscope

		// read the phase currents i_a and i_b
		i_a = (float)(adc_buf[4] - adc_buf[3]) * ADC_CURRENT_CONVERSION_FACTOR;
		i_b = (float)(adc_buf[2] - adc_buf[1]) * ADC_CURRENT_CONVERSION_FACTOR;

        // shut down if currents exceed the over current limit
        if (i_a > OVERCURRENT_LIMIT || i_b > OVERCURRENT_LIMIT || -i_a > OVERCURRENT_LIMIT || -i_b > OVERCURRENT_LIMIT) {
          set_output_a(0.0, 0.0);
          set_output_b(0.0, 0.0);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);		// turn on the red LED
          while(1);   // wait for reset
        }

		// calculate the currents i_d and i_q
		cnt = TIM2->CNT;	// read the encoder angle

		sin_value = sin_table[cnt];
		cos_value = cos_table[cnt];

		i_d = cos_value * i_a + sin_value * i_b;
		i_q = -sin_value * i_a + cos_value * i_b;

		// read the set points i_d_ref and i_q_ref for the currents i_d and i_q
		//i_d_ref = (float)(adc_buf[5] - (ADC_STEPS >> 1)) * ADC_REF_INPUTS_CONVERSION_FACTOR;
		i_d_ref = 0;
		i_q_ref = (float)(adc_buf[6] - (ADC_STEPS >> 1)) * ADC_REF_INPUTS_CONVERSION_FACTOR;

		// calculate v_d and v_q via the current controllers difference equations
		e_i_d = i_d_ref - i_d;
		e_i_q = i_q_ref - i_q;

		v_d = V * (e_i_d - E * e_i_d_prev) + v_d_prev;
		v_q = V * (e_i_q - E * e_i_q_prev) + v_q_prev;

	    e_i_d_prev = e_i_d;
	    e_i_q_prev = e_i_q;

		// calculate the phase voltages v_a and v_b
		v_a = cos_value * v_d - sin_value * v_q;
		v_b = sin_value * v_d + cos_value * v_q;

		// check whether the calculated voltages are within the supply voltage (i.e. whether they are realizable)
		v_supply = (float)adc_buf[0] * ADC_SUPPLY_VOLTAGE_CONVERSION_FACTOR;	// read the supply voltage
		if (v_a > v_supply) {
			limit_hit_flag = 1;
			v_a = v_supply;
		} else if (v_a < -v_supply) {
			limit_hit_flag = 1;
		    v_a = -v_supply;
		}

		if (v_b > v_supply) {
			limit_hit_flag = 1;
		    v_b = v_supply;
		} else if (v_b < -v_supply) {
			limit_hit_flag = 1;
		    v_b = -v_supply;
		}

		// apply the calculated phase voltages v_a and v_b to the motor phases
		set_output_a(v_a, v_supply);
		set_output_b(v_b, v_supply);

		if (limit_hit_flag) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);		// turn on the red LED
			// calculate the actually realized v_d and v_q voltages (anti windup)
			v_d = cos_value * v_a + sin_value * v_b;
			v_q = -sin_value * v_a + cos_value * v_b;
			limit_hit_flag = 0;
		} else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);	// turn off the red LED
		}

		v_d_prev = v_d;
		v_q_prev = v_q;

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);	// allows for measuring the execution time of one cycle of the control algorithm via a oscilloscope
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 35;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 7;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_7;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = PWM_STEPS-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = ENCODER_STEPS-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 70-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = TS-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD3_Pin PB4 */
  GPIO_InitStruct.Pin = LD3_Pin|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void set_output_a(float v_a, float v_supply) {
    if (v_supply < MINIMAL_SUPPLY_VOLTAGE) {   // no output when supply voltage is too low
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        return;
    }
	if (v_a > 0) {
		TIM1->CCR3 = 0;
		TIM1->CCR2 = (uint32_t)(PWM_STEPS * v_a/v_supply) + PWM_STEPS_OFFSET;
	} else {
		TIM1->CCR2 = 0;
		TIM1->CCR3 = (uint32_t)(-PWM_STEPS * v_a/v_supply) + PWM_STEPS_OFFSET;
	}
}

void set_output_b(float v_b, float v_supply) {
    if (v_supply < MINIMAL_SUPPLY_VOLTAGE) {   // no output when supply voltage is too low
        TIM1->CCR1 = 0;
        TIM1->CCR4 = 0;
        return;
    }
	if (v_b > 0) {
		TIM1->CCR4 = 0;
		TIM1->CCR1 = (uint32_t)(PWM_STEPS * v_b/v_supply) + PWM_STEPS_OFFSET;
	} else {
		TIM1->CCR1 = 0;
		TIM1->CCR4 = (uint32_t)(-PWM_STEPS * v_b/v_supply) + PWM_STEPS_OFFSET;
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim16) {
		triggered = 1;
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
