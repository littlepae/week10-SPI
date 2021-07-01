/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
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
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char TxDataBuffer[32] = { 0 };
char RxDataBuffer[32] = { 0 };
int16_t Inputchar = 0;
int Waveform = 0;
float Wavefreq = 1;
float Wavehighvolt = 3.3;
float Wavelowvolt = 0.0;
int Waveduty = 50;
int slope = 0;
float VoltageOut = 0;
float VoltageIn = 0;
double Sineangle = 0;
uint16_t State = 0000;
uint16_t ADCin = 0;
uint64_t _micro = 0;
uint16_t dataOut = 0;
	uint8_t DACConfig = 0b0011;
char Menu_Start[] = "1: Sawtooth wave\r\n2: Sine wave\r\n3: Square wave\r\n";
char Menu_Sawtooth[] = "a: Freq\r\ns: Voltage\r\nd: Slope\r\nx: back\r\n";
char Menu_Sine[] = "a: Freq\r\ns: Voltage\r\nx: back\r\n";
char Menu_Square[] = "a: Freq\r\ns: Voltage\r\nd:Duty cyclex: back\r\n";
char Menu_Freq[] = "a:+0.1 Hz\r\ns:-0.1Hz\r\nx:back\r\n";
char Menu_Volt[] = "a:V High+0.1\r\ns:V high-0.1\r\nd:V Low+0.1\r\nf:V low-0.1\r\nx:back\r\n";
char Menu_Slope[] = "a:Slope Up\r\ns:Slope Down\r\nx:back\r\n";
char Menu_Duty[] = "a:Duty cycle +10%\r\ns:Duty cycle -10%\r\nx:back\r\n";
char Wave_Freq[20] = "";
char Wave_Volt[30] = "";
char Wave_Duty[30] = "";
char Wave_Slope[20] = "";
enum State_now
{
	State_Start = 0000,
	State_Menu = 0001,
	State_Sawtooth = 0010,
	State_Sine = 0011,
	State_Square = 0100,
	State_Freq = 0101,
	State_Volt = 0111,
	State_Slope = 0110,
	State_Duty = 1110,
	State_Sawtooth_Menu = 1010,
	State_Sine_Menu = 1000,
	State_Square_Menu = 1001,
	State_Freq_Menu = 1011,
	State_Volt_Menu = 1111,
	State_Slope_Menu = 1101,
	State_Duty_Menu = 1100,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int16_t UARTRecieveIT();
void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput);
uint64_t micros();
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADCin, 1);

	HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		HAL_UART_Receive_IT(&huart2, (uint8_t*)RxDataBuffer, 32);
		Inputchar = UARTRecieveIT();
		switch(State)
		{
			case State_Start: //Start state
				HAL_UART_Transmit(&huart2, (uint8_t*)Menu_Start, strlen(Menu_Start), 10);
				State = State_Menu;
				break;
			case State_Menu: //Choose menu
				if(Inputchar == '1')
				{
					State = State_Sawtooth;
				}
				else if(Inputchar == '2')
				{
					State = State_Sine;
				}
				else if(Inputchar == '3')
				{
					State = State_Square;
				}
				break;
			case State_Sawtooth: //Sawtooth menu
				HAL_UART_Transmit(&huart2, (uint8_t*)Menu_Sawtooth, strlen(Menu_Sawtooth), 10);
				State = State_Sawtooth_Menu;
				break;
			case State_Sawtooth_Menu: //Sawtooth control
				Waveform = 1;
				if(Inputchar == 'a')
				{
					State = State_Freq;
				}
				else if(Inputchar == 's')
				{
					State = State_Volt;
				}
				else if(Inputchar == 'd')
				{
					State = State_Slope;
				}
				else if(Inputchar == 'x')
				{
					State = State_Start;
				}
				break;
			case State_Sine: // Button menu
				HAL_UART_Transmit(&huart2, (uint8_t*)Menu_Sine, strlen(Menu_Sine), 10);
				State = State_Sine_Menu;
				break;
			case State_Sine_Menu: //Button status
				Waveform = 2;
				if(Inputchar == 'a')
				{
					State = State_Freq;
				}
				else if(Inputchar == 's')
				{
					State = State_Volt;
				}
				else if(Inputchar == 'x')
				{
					State = State_Start;
				}
				break;
			case State_Square:
				HAL_UART_Transmit(&huart2, (uint8_t*)Menu_Square, strlen(Menu_Square), 10);
				State = State_Square_Menu;
				break;
			case State_Square_Menu: //Button status
				Waveform = 3;
				if(Inputchar == 'a')
				{
					State = State_Freq;
				}
				else if(Inputchar == 's')
				{
					State = State_Volt;
				}
				else if(Inputchar == 'd')
				{
					State = State_Duty;
				}
				else if(Inputchar == 'x')
				{
				State = State_Start;
				}
				break;
			case State_Freq:
				HAL_UART_Transmit(&huart2, (uint8_t*)Menu_Freq, strlen(Menu_Freq), 10);
				State = State_Freq_Menu;
				break;
			case State_Freq_Menu:
				if(Inputchar == 'a')
				{
					Wavefreq += 0.1;
					if(Wavefreq > 10)
					{
						Wavefreq = 10;
					}
					sprintf(Wave_Freq,"Frequency: %f\r\n",Wavefreq);
					HAL_UART_Transmit(&huart2, (uint8_t*)Wave_Freq, strlen(Wave_Freq), 10);
					State = State_Freq;
				}
				else if(Inputchar == 's')
				{
					Wavefreq -= 0.1;
					if(Wavefreq <= 0)
					{
						Wavefreq = 0;
					}
					sprintf(Wave_Freq,"Frequency: %f\r\n",Wavefreq);
					HAL_UART_Transmit(&huart2, (uint8_t*)Wave_Freq, strlen(Wave_Freq), 10);
					State = State_Freq;
				}
				else if(Inputchar == 'x')
				{
					if(Waveform == 1)
					{
						State = State_Sawtooth;
					}
					else if(Waveform == 2)
					{
						State = State_Sine;
					}
					else if(Waveform == 3)
					{
						State = State_Square;
					}
				}
				break;
			case State_Volt:
				HAL_UART_Transmit(&huart2, (uint8_t*)Menu_Volt, strlen(Menu_Volt), 10);
				State = State_Volt_Menu;
				break;
			case State_Volt_Menu:
				if(Inputchar == 'a')
				{
					Wavehighvolt += 0.1;
					if(Wavehighvolt > 3.3)
					{
						Wavehighvolt = 3.3;
					}
					sprintf(Wave_Volt,"Hvolt:%fV | Lvolt:%fV\r\n",Wavehighvolt,Wavelowvolt);
					HAL_UART_Transmit(&huart2, (uint8_t*)Wave_Volt, strlen(Wave_Volt), 10);
					State = State_Volt;
				}
				else if(Inputchar == 's')
				{
					Wavehighvolt -= 0.1;
					if(Wavehighvolt <= 0)
					{
						Wavehighvolt = 0;
					}
					sprintf(Wave_Volt,"Hvolt:%fV | Lvolt:%fV\r\n",Wavehighvolt,Wavelowvolt);
					HAL_UART_Transmit(&huart2, (uint8_t*)Wave_Volt, strlen(Wave_Volt), 10);
					State = State_Volt;
				}
				else if(Inputchar == 'd')
				{
					Wavelowvolt += 0.1;
					if(Wavelowvolt > 3.3)
					{
						Wavelowvolt = 3.3;
					}
					sprintf(Wave_Volt,"Hvolt:%fV | Lvolt:%fV\r\n",Wavehighvolt,Wavelowvolt);
					HAL_UART_Transmit(&huart2, (uint8_t*)Wave_Volt, strlen(Wave_Volt), 10);
					State = State_Volt;
				}
				else if(Inputchar == 'f')
				{
					Wavelowvolt -= 0.1;
					if(Wavelowvolt <= 0)
					{
						Wavelowvolt = 0;
					}
					sprintf(Wave_Volt,"Hvolt:%fV | Lvolt:%fV\r\n",Wavehighvolt,Wavelowvolt);
					HAL_UART_Transmit(&huart2, (uint8_t*)Wave_Volt, strlen(Wave_Volt), 10);
					State = State_Volt;
				}
				else if(Inputchar == 'x')
				{
					if(Waveform == 1)
					{
						State = State_Sawtooth;
					}
					else if(Waveform == 2)
					{
						State = State_Sine;
					}
					else if(Waveform == 3)
					{
						State = State_Square;
					}
				}
				break;
			case State_Slope:
				HAL_UART_Transmit(&huart2, (uint8_t*)Menu_Slope, strlen(Menu_Slope), 10);
				State = State_Slope_Menu;
				break;
			case State_Slope_Menu:
				if(Inputchar == 'a')
				{
					slope = 0;
					sprintf(Wave_Slope,"Slope Up\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*)Wave_Slope, strlen(Wave_Slope), 10);
					State = State_Slope;
				}
				else if(Inputchar == 's')
				{
					slope = 1;
					sprintf(Wave_Slope,"Slope Down\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*)Wave_Slope, strlen(Wave_Slope), 10);
					State = State_Slope;
				}
				else if(Inputchar == 'x')
				{
					State = State_Sawtooth;
				}
				break;
			case State_Duty:
				HAL_UART_Transmit(&huart2, (uint8_t*)Menu_Duty, strlen(Menu_Duty), 10);
				State = State_Duty_Menu;
				break;
			case State_Duty_Menu:
				if(Inputchar == 'a')
				{
					Waveduty += 10;
					if(Waveduty > 100)
					{
						Waveduty = 100;
					}
					sprintf(Wave_Duty,"Duty Cycle:%d % \r\n",Waveduty);
					HAL_UART_Transmit(&huart2, (uint8_t*)Wave_Duty, strlen(Wave_Duty), 10);
					State = State_Duty;
				}
				else if(Inputchar == 's')
				{
					Waveduty -= 10;
					if(Waveduty <= 0)
					{
						Waveduty = 0;
					}
					sprintf(Wave_Duty,"Duty Cycle:%d % \r\n",Waveduty);
					HAL_UART_Transmit(&huart2, (uint8_t*)Wave_Duty, strlen(Wave_Duty), 10);
					State = State_Duty;
					}
				else if(Inputchar == 'x')
				{
					State = State_Square;
				}
				break;
		}

		static uint64_t timestamp = 0;
		static uint64_t timestamp_wave = 0;

		if(micros() - timestamp > 100)
		{
			if(Waveform == 1) //sawtooth
			{
				if(slope == 0) //up
				{
					if(micros() - timestamp_wave <= (1000000/Wavefreq))
					{
						VoltageOut = Wavelowvolt + ((Wavehighvolt - Wavelowvolt)*((micros()-timestamp_wave)/(1000000/Wavefreq)));
					}
					else if(micros() - timestamp_wave > (1000000/Wavefreq))
					{
						timestamp_wave = micros();
					}
				}
				else if(slope == 1) //down
				{
					if(micros() - timestamp_wave <= (1000000/Wavefreq))
					{
						VoltageOut = Wavehighvolt - ((Wavehighvolt - Wavelowvolt)*((micros()-timestamp_wave)/(1000000/Wavefreq)));
					}
					else if(micros() - timestamp_wave > (1000000/Wavefreq))
					{
						timestamp_wave = micros();
					}
				}
			}
			else if(Waveform == 2) //sine
			{
				if(micros() - timestamp_wave <= (1000000/Wavefreq))
				{
					Sineangle = ((micros() - timestamp_wave)/(1000000/Wavefreq))*2*3.14;
					VoltageOut = Wavelowvolt + ((sin(Sineangle)+1)*(Wavehighvolt-Wavelowvolt)/2);
				}
				else if(micros() - timestamp_wave > (1000000/Wavefreq))
				{
					timestamp_wave = micros();
				}
			}
			else if(Waveform == 3) //square
			{

				if(micros() - timestamp_wave <= (1000000/Wavefreq))
				{
					if(micros() - timestamp_wave <= (1000000/Wavefreq)*Waveduty/100)
					{
						VoltageOut = Wavehighvolt;
					}
					else if(micros() - timestamp_wave > (1000000/Wavefreq)*Waveduty/100)
					{
						VoltageOut = Wavelowvolt;
					}
				}
				else if(micros() - timestamp_wave > (1000000/Wavefreq))
				{
					timestamp_wave = micros();
				}
			}
			dataOut = (int)(VoltageOut*4096/3.3); //0-4096
			timestamp = micros();
			if (hspi3.State == HAL_SPI_STATE_READY && HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin)== GPIO_PIN_SET)
			{
				MCP4922SetOutput(DACConfig, dataOut);
			}
		}


		VoltageIn = ADCin * 3.3 / 4096; //0 - 3.3

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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim2.Init.Period = 4294967295;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SHDN_GPIO_Port, SHDN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LOAD_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LOAD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_SS_Pin */
  GPIO_InitStruct.Pin = SPI_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHDN_Pin */
  GPIO_InitStruct.Pin = SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SHDN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int16_t UARTRecieveIT()
{
	static uint32_t dataPos =0;
	int16_t data=-1;
	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)
	{
		data=RxDataBuffer[dataPos];
		dataPos= (dataPos+1)%huart2.RxXferSize;
	}
	return data;
}

void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput)
{
	uint32_t OutputPacket = (DACOutput & 0x0fff) | ((Config & 0xf) << 12);
	HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi3, &OutputPacket, 1);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi3)
	{
		HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		_micro += 4294967295;
	}
}

inline uint64_t micros()
{
	return htim2.Instance->CNT + _micro;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
