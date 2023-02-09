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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dfsdm.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define 		PLL_Kp 					1.0e-3f 								// PID parameters for our PLL tracking control
#define 		PLL_Ki 					1.0e-4f									// PoM: 4.0e-3f, 2.0e-2f
#define 		PLL_Kd 					0.0f
#define 		PLL_LIMIT 				500.0f
#define 		PLL_PERIOD 				7.8e-5f 								// 1 / (50 * 256) seconds
#define 		SINE_STEP_PERIOD 		6250 									// Ticks between incrementing our LO (Local Osc) index for 50Hz sine
//##############################################
#define			I_OUT_Kp				1.0e1f									// PID parameters for our current output controller
#define			I_OUT_Ki				1.0e0f
#define			I_OUT_Kd				0.0f
#define			I_OUT_Limit				975.0f
#define			I_OUT_PERIOD			50.0e-6f

//##############################################
#define 		SINE_STEPS           	256                         			// Number of steps to build our sine wave in
#define 		DUTY_LIMIT 				994 									// Our duty width can vary from -1000 to +1000
#define			INTEGRAL_SIZE			256
#define			RMS_INTEGRAL_SIZE		64

#define 		F_CONVERSION_K 			8.065e-3f								// These are calibration constants

#define 		RMS_LOWER_LIMIT 		0 										// These values are for the grid checks. We disconnect if our metrics are out of these ranges (SI Units)
#define 		RMS_UPPER_LIMIT 		62500
#define 		FREQ_DEVIATION_LIMIT 	0.5f
#define			V_BUS_MINIMUM			0.0f
#define 		V_BUS_MAXIMUM 			395.0f
#define 		I_OUTPUT_MAXIMUM 		3.5f

#define 		ENABLE_JOINING_GRID 	true
#define 		GRID_ACCEPTABLE 		1000
#define 		GRID_UNACCEPTABLE 		-1000
#define 		GRID_BAD_FAIL_RATE 		10 										// Some grid checks are allowed to fail for a certain amount of time (eg. Frequency) this parameter determines for how long
#define 		GRID_OK					0

#define 		CONSTRAIN(x,lower,upper) ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float Sin_LookupF[256] =
{
	0.00,6.28,12.56,18.83,25.09,31.34,37.56,43.77,49.94,56.09,62.20,68.28,74.31,80.30,86.24,92.13,
	97.97,103.74,109.45,115.10,120.68,126.18,131.61,136.96,142.23,147.41,152.50,157.50,162.40,167.21,171.92,176.52,
	181.02,185.41,189.68,193.85,197.89,201.82,205.62,209.30,212.86,216.28,219.58,222.74,225.77,228.67,231.42,234.04,
	236.51,238.85,241.04,243.08,244.98,246.73,248.33,249.78,251.08,252.23,253.23,254.07,254.77,255.31,255.69,255.92,

	256.00,255.92,255.69,255.31,254.77,254.07,253.23,252.23,251.08,249.78,248.33,246.73,244.98,243.08,241.04,238.85,
	236.51,234.04,231.42,228.67,225.77,222.74,219.58,216.28,212.86,209.30,205.62,201.82,197.89,193.85,189.68,185.41,
	181.02,176.52,171.92,167.21,162.40,157.50,152.50,147.41,142.23,136.96,131.61,126.18,120.68,115.10,109.45,103.74,
	97.97,92.13,86.24,80.30,74.31,68.28,62.20,56.09,49.94,43.77,37.56,31.34,25.09,18.83,12.56,6.28,

	0.00,-6.28,-12.56,-18.83,-25.09,-31.34,-37.56,-43.77,-49.94,-56.09,-62.20,-68.28,-74.31,-80.30,-86.24,-92.13,
	-97.97,-103.74,-109.45,-115.10,-120.68,-126.18,-131.61,-136.96,-142.23,-147.41,-152.50,-157.50,-162.40,-167.21,-171.92,-176.52,
	-181.02,-185.41,-189.68,-193.85,-197.89,-201.82,-205.62,-209.30,-212.86,-216.28,-219.58,-222.74,-225.77,-228.67,-231.42,-234.04,
	-236.51,-238.85,-241.04,-243.08,-244.98,-246.73,-248.33,-249.78,-251.08,-252.23,-253.23,-254.07,-254.77,-255.31,-255.69,-255.92,

	-256.00,-255.92,-255.69,-255.31,-254.77,-254.07,-253.23,-252.23,-251.08,-249.78,-248.33,-246.73,-244.98,-243.08,-241.04,-238.85,
	-236.51,-234.04,-231.42,-228.67,-225.77,-222.74,-219.58,-216.28,-212.86,-209.30,-205.62,-201.82,-197.89,-193.85,-189.68,-185.41,
	-181.02,-176.52,-171.92,-167.21,-162.40,-157.50,-152.50,-147.41,-142.23,-136.96,-131.61,-126.18,-120.68,-115.10,-109.45,-103.74,
	-97.97,-92.13,-86.24,-80.30,-74.31,-68.28,-62.20,-56.09,-49.94,-43.77,-37.56,-31.34,-25.09,-18.83,-12.56,-6.28
};

float Cos_LookupF[256] =
{
	256.00,255.92,255.69,255.31,254.77,254.07,253.23,252.23,251.08,249.78,248.33,246.73,244.98,243.08,241.04,238.85,
	236.51,234.04,231.42,228.67,225.77,222.74,219.58,216.28,212.86,209.30,205.62,201.82,197.89,193.85,189.68,185.41,
	181.02,176.52,171.92,167.21,162.40,157.50,152.50,147.41,142.23,136.96,131.61,126.18,120.68,115.10,109.45,103.74,
	97.97,92.13,86.24,80.30,74.31,68.28,62.20,56.09,49.94,43.77,37.56,31.34,25.09,18.83,12.56,6.28,

	0.00,-6.28,-12.56,-18.83,-25.09,-31.34,-37.56,-43.77,-49.94,-56.09,-62.20,-68.28,-74.31,-80.30,-86.24,-92.13,
	-97.97,-103.74,-109.45,-115.10,-120.68,-126.18,-131.61,-136.96,-142.23,-147.41,-152.50,-157.50,-162.40,-167.21,-171.92,-176.52,
	-181.02,-185.41,-189.68,-193.85,-197.89,-201.82,-205.62,-209.30,-212.86,-216.28,-219.58,-222.74,-225.77,-228.67,-231.42,-234.04,
	-236.51,-238.85,-241.04,-243.08,-244.98,-246.73,-248.33,-249.78,-251.08,-252.23,-253.23,-254.07,-254.77,-255.31,-255.69,-255.92,

	-256.00,-255.92,-255.69,-255.31,-254.77,-254.07,-253.23,-252.23,-251.08,-249.78,-248.33,-246.73,-244.98,-243.08,-241.04,-238.85,
	-236.51,-234.04,-231.42,-228.67,-225.77,-222.74,-219.58,-216.28,-212.86,-209.30,-205.62,-201.82,-197.89,-193.85,-189.68,-185.41,
	-181.02,-176.52,-171.92,-167.21,-162.40,-157.50,-152.50,-147.41,-142.23,-136.96,-131.61,-126.18,-120.68,-115.10,-109.45,-103.74,
	-97.97,-92.13,-86.24,-80.30,-74.31,-68.28,-62.20,-56.09,-49.94,-43.77,-37.56,-31.34,-25.09,-18.83,-12.56,-6.28,

	0.00,6.28,12.56,18.83,25.09,31.34,37.56,43.77,49.94,56.09,62.20,68.28,74.31,80.30,86.24,92.13,
	97.97,103.74,109.45,115.10,120.68,126.18,131.61,136.96,142.23,147.41,152.50,157.50,162.40,167.21,171.92,176.52,
	181.02,185.41,189.68,193.85,197.89,201.82,205.62,209.30,212.86,216.28,219.58,222.74,225.77,228.67,231.42,234.04,
	236.51,238.85,241.04,243.08,244.98,246.73,248.33,249.78,251.08,252.23,253.23,254.07,254.77,255.31,255.69,255.92
};

PIDControl 			PLL_PID, I_OUT_PID; 										// These structures are used to store the respective PID variables

// DMA buffers and flags
int32_t 			Vbus_DMA[1], Igrid_DMA[1], Vgrid_DMA[1], Icap_DMA[1];

// Voltage and Current values
float 				I_grid, V_grid, I_cap, V_Bus, Grid_RMS, Freq_Offset; 				// These are our SI unit values
float 				SD_Raw;
uint32_t			Error = 0;

volatile uint8_t 	_50_Index = 0;

volatile int16_t 	Grid_Good_Bad_Cnt = GRID_UNACCEPTABLE;
volatile uint8_t 	REQUEST_JOIN = false, Error_Code;
volatile float 		I_Output_Demand = 0;

uint8_t				HB_Enabled = false;
int32_t 			Mains_MS;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HB_Disable(void);
void HB_Enable(void);
float Integral(int32_t datum);
int32_t Integrate_Mains_MS(int32_t grid_voltage_sample);
void HAL_DFSDM_FilterAwdCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t Channel, uint32_t Threshold);
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void Grid_Checks(void);
void Controller(void);
void PLL(void);
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_DFSDM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // Initialise the PID controller for the PLL
  PIDInit(&PLL_PID, PLL_Kp, PLL_Ki, PLL_Kd, PLL_PERIOD, -PLL_LIMIT, PLL_LIMIT, AUTOMATIC, DIRECT, P_ON_E);
  PIDInit(&I_OUT_PID, I_OUT_Kp, I_OUT_Ki, I_OUT_Kd, I_OUT_PERIOD, -I_OUT_Limit, I_OUT_Limit, AUTOMATIC, DIRECT, P_ON_E);

  // Initialise our timers
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  HAL_GPIO_WritePin(GPIOA, Relay_Pin, GPIO_PIN_SET);

  // Start the DFSDMs
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, Vbus_DMA, 1);
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter1, Igrid_DMA, 1);
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter2, Vgrid_DMA, 1);
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter3, Icap_DMA, 1);

  // This should enable the interrupts by the analogue watchdog ----------- BUT IT DOESN'T!!!
  DFSDM_Filter_AwdParamTypeDef awdParamFilter0;
  awdParamFilter0.DataSource = DFSDM_FILTER_AWD_FILTER_DATA;
  awdParamFilter0.Channel = DFSDM_CHANNEL_0;
  awdParamFilter0.HighBreakSignal = DFSDM_NO_BREAK_SIGNAL;
  awdParamFilter0.HighThreshold = 250;
  awdParamFilter0.LowBreakSignal = DFSDM_NO_BREAK_SIGNAL;
  awdParamFilter0.LowThreshold = -250;
  //HAL_DFSDM_FilterAwdStart_IT(&hdfsdm1_filter0, &awdParamFilter0);

  DFSDM_Filter_AwdParamTypeDef awdParamFilter1;
  awdParamFilter1.DataSource = DFSDM_FILTER_AWD_FILTER_DATA;
  awdParamFilter1.Channel = DFSDM_CHANNEL_1;
  awdParamFilter1.HighBreakSignal = DFSDM_NO_BREAK_SIGNAL;
  awdParamFilter1.HighThreshold = 250;
  awdParamFilter1.LowBreakSignal = DFSDM_NO_BREAK_SIGNAL;
  awdParamFilter1.LowThreshold = -250;
  //HAL_DFSDM_FilterAwdStart_IT(&hdfsdm1_filter1, &awdParamFilter1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
}

/* USER CODE BEGIN 4 */
void HB_Disable() {
	// Disable the H-Bridge
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HB_Enabled = false;
}

void HB_Enable() {
	// Start our PWM driver which uses Timer1 to power our H-bridge. Both channels start with Duty = 0
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HB_Enabled = true;
}

float Integral(int32_t datum)
{
	// Keep a running summation of the last INTEGRAL_SIZE values passed into this function.
	static int32_t 		Buffer[INTEGRAL_SIZE];
	static uint16_t  	Index = 0;
	static int32_t  	Sum = 0;

	Sum += datum;

	Buffer[Index++] = datum;

	if(Index == INTEGRAL_SIZE)
		Index = 0;

	Sum -= Buffer[Index];
	return((float)Sum);
}

int32_t Integrate_Mains_MS(int32_t grid_voltage_sample)
{
	// Maintain an MS measurement of the grid voltage. Runs at 800Hz which is 16 samples per period
	static int32_t 		Buffer[RMS_INTEGRAL_SIZE];
	static int8_t  		Index = 0;
	static int32_t  	Sum = 0;

	int32_t Sample_sqrd = grid_voltage_sample * grid_voltage_sample;

	Sum += Sample_sqrd;

	Buffer[Index++] = Sample_sqrd;

	if(Index == RMS_INTEGRAL_SIZE)
		Index = 0;

	Sum -= Buffer[Index];

	int32_t Mean_Squared = Sum >> 6;			// Divide Sum by 64 (RMS_INTEGRAL_SIZE)
	return(Mean_Squared);						// return(sqrtf(RMS));
}

void HAL_DFSDM_FilterAwdCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t Channel, uint32_t Threshold)
{
	// Callback if Analogue watch dog triggers ---------- BUT CAN'T GET THIS TO RUN!!!!
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)	{
	if(hdfsdm_filter == &hdfsdm1_filter0)	{
		V_Bus = (float)Vbus_DMA[0] * 2.4414e-7 + 5.0e-4;
	}
	if(hdfsdm_filter == &hdfsdm1_filter1)	{
		I_grid = (float)Igrid_DMA[0] * -3.1738e-9 + 8.75e-6;
	}
	if(hdfsdm_filter == &hdfsdm1_filter2)	{
		V_grid = (float)Vgrid_DMA[0] * 2.4414e-7 + 5.0e-4;
	}
	if(hdfsdm_filter == &hdfsdm1_filter3)	{
		I_cap = (float)Icap_DMA[0] * 3.1738e-9 - 8.75e-6;
	}
}

void Grid_Checks()	{
	// Calculate some grid metrics for some following checks
	Mains_MS = Integrate_Mains_MS((int32_t)V_grid); 								// Update our mains RMS measurement
	float Freq_Offset = PLL_PID.output * F_CONVERSION_K; 							// Get the frequency difference between the grid and a 50Hz reference

	// These are high priority checks:
	if(V_Bus > V_BUS_MAXIMUM)	{													// If our DC Bus voltage is too high cut-out immediately
		HB_Disable();
		Grid_Good_Bad_Cnt = GRID_UNACCEPTABLE;
		Error_Code |= 0b00001;
	}

	if(I_grid > I_OUTPUT_MAXIMUM || I_grid < -I_OUTPUT_MAXIMUM)	{					// Likewise, if we detect excess current flowing, cut-out immediately
		HB_Disable();
		Grid_Good_Bad_Cnt = GRID_UNACCEPTABLE;
		Error_Code |= 0b00010;
	}

	// These are lower priority checks and have to be out of range for an amount of time
	if(Mains_MS > RMS_UPPER_LIMIT || Mains_MS < RMS_LOWER_LIMIT)	{				// If mains RMS voltage whacky
		Grid_Good_Bad_Cnt -= GRID_BAD_FAIL_RATE;
		Error_Code |= 0b00100;
	}

	//if(Freq_Offset > FREQ_DEVIATION_LIMIT || Freq_Offset < -FREQ_DEVIATION_LIMIT)	{
	//	Grid_Good_Bad_Cnt -= GRID_BAD_FAIL_RATE;									// If our mains frequency is out of tolerance
	//	Error_Code |= 0b01000;
	//}

	if(V_Bus < V_BUS_MINIMUM)	{													// If our DC Bus voltage is too low
		Grid_Good_Bad_Cnt -= GRID_BAD_FAIL_RATE;
		Error_Code |= 0b10000;
	}

	// Act on the above checks
	if(Grid_Good_Bad_Cnt < GRID_OK)	{												// If our metrics have been whacky for too long stop output
		if(HB_Enabled == true) {
			HB_Disable(); 															// This puts the H-bridge into a high impedance state
			Grid_Good_Bad_Cnt = GRID_UNACCEPTABLE;
		}
	}

	if(HB_Enabled == false) { 														// With our output idle, if the grid has normalised, restart
		if(Grid_Good_Bad_Cnt == GRID_ACCEPTABLE && ENABLE_JOINING_GRID == true) {
			REQUEST_JOIN = true;
			Error_Code = 0;
		}
	}

	// Constrain and decay any error counts
	Grid_Good_Bad_Cnt++;
	Grid_Good_Bad_Cnt = CONSTRAIN(Grid_Good_Bad_Cnt, GRID_UNACCEPTABLE, GRID_ACCEPTABLE);

	if(I_Output_Demand < 3.85f)
		I_Output_Demand += 0.0002f;
}

void Controller()	{
	if(HB_Enabled == false)	{
		if(REQUEST_JOIN == true)	{
			if(_50_Index == 0)	{
				REQUEST_JOIN = false;
				I_Output_Demand = 0;
				HB_Enable();
			}
		}
	}

	static int16_t Duty_Cycle;
	static int16_t Prev_Duty_Cycle;




	//I_OUT_PID.setpoint = Sin_LookupF[_50_Index] * 4.0e-3;
	//I_OUT_PID.input = I_grid;
	//PIDCompute(&I_OUT_PID);
	//int16_t PI_I_Term = (int16_t)I_OUT_PID.output;

	//Duty_Cycle = (int16_t)I_OUT_PID.output;
	Duty_Cycle = (int16_t)(Sin_LookupF[_50_Index] * I_Output_Demand);





	// Constrain and output the new duty cycle
	Duty_Cycle = CONSTRAIN(Duty_Cycle, -DUTY_LIMIT, DUTY_LIMIT);

	// Rather than do this it'd be better to force the outputs which i think is a thing
	if(Prev_Duty_Cycle < 0 && Duty_Cycle >= 0)	{
		while(htim1.Instance->CNT > 950) {}
	}
	else if(Prev_Duty_Cycle >= 0 && Duty_Cycle < 0)	{
		while(htim1.Instance->CNT > 950) {}
	}

	if(Duty_Cycle >= 0) {
		htim1.Instance->CCR1 = 0;
		htim1.Instance->CCR2 = Duty_Cycle;
	}
	else {
		htim1.Instance->CCR1 = 1000;
		htim1.Instance->CCR2 = 1000 + Duty_Cycle;
	}

	Prev_Duty_Cycle = Duty_Cycle;
}

void PLL()	{
	int32_t Signal_Multiple = (int32_t)(Cos_LookupF[_50_Index] * V_grid); 			// Multiply the LO Cosine value with our current phase voltage sample
	PLL_PID.input = Integral(Signal_Multiple);   									// Integrate this Multiple over the last 1 period (Ideally normalised to RMS)
	PIDCompute(&PLL_PID); 															// Plug result in a PI controller to maintain 0 phase shift

	TIM4->ARR = SINE_STEP_PERIOD;// - (int32_t)PLL_PID.output; 						// adjust LO frequency (step period) to synchronise to mains phase
	_50_Index += 1;  																// Increment our LO indices, since they're 8bits wide they will roll-over if they exceed 255
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2)	{
		Grid_Checks();																	// #### Runs every 1.25ms to perform our safety checks ####
	}

	if (htim == &htim3)	{
		Controller();																	// ### Runs every 100us to iterate the controller ###
	}

	if (htim == &htim4)	{
		PLL();																			// ### Runs 12.8 kHz and iterates the PLL ###
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
