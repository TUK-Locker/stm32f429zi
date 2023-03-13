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

/*

Major : Mechatronics Engineering
Author: In-Hwan Jeong( 정인환 )
Reference : https://controllerstech.com/의 ESP8266_HAL.c, UartRingbuffer_multi.c

*/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ESP8266_HAL.h"
#include "UartRingbuffer_multi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LM_ENA_Yes 0
#define LM_ENA_No 1
#define Wheel_Right_Go 1
#define Wheel_Right_Back 0
#define Wheel_Left_Go 1
#define Wheel_Left_Back 0
#define Car_Go 1
#define Car_Back 0
#define Lockeroncall 1
#define Lockeron 2
#define Lockeroffcall 3
#define Lockeroff 4
#define Relay_ON 1
#define Relay_OFF 2
#define wifi_uart &huart2
#define pc_uart &huart3

#define LM_extra 20 //mm
#define LM_extra2 10//mm

#define duty 3000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
int i=0;
unsigned int LM_Guide_Step=0; // pwm on time
unsigned int Limit_Cnt=0;
unsigned int LM_Position=0; // (motor) 0mm ~ (max) 320mm
unsigned char Locker_Move_Flag=0;
unsigned char Locker_Flag[2][4]={0};
unsigned char Locker[2][4]={0};
unsigned char Locker_Floor=0;
unsigned char Elevator_Floor=0;
unsigned char Limit_Flag=0;
int Car_Time=1;
int Locker_Distance[2][4]={{2643,3786,2643,3786},{2643,3786,2643,3786}};
/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void LM_Guide_ENA(int ena);
void LM_Guide_DIR(int dir);
void LM_Guide_Move(int LM_Position_Goal,int Magnet);
void Wheel_DIR(int dir);
void TUK_Locker_Car_Move(int Car_Move_Delay,int Speed, int Car_DIR);
void TUK_Locker_Car_Move_Delay(int Car_Move_Delay,int Speed, int Car_DIR,int Car_Stop);
void Elevator(int floor);



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
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART3_UART_Init();
	MX_TIM6_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim6);

	ESP_Init("CIR_Wireless","cir123456!");
	HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, 0);


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */



	while (1)
	{
		Server_Start();
		if(Locker_Move_Flag >4)
		{
			Locker_Floor = 1;
		}
		else
		{
			Locker_Floor = 0;
		}



		if(Locker_Move_Flag>=1)
		{
			if( Locker_Move_Flag == 1 || Locker_Move_Flag == 2 || Locker_Move_Flag == 5  || Locker_Move_Flag == 6 )//
			{

				if(Locker_Flag[Locker_Floor][(Locker_Move_Flag-1)%4]==Lockeroncall)
				{
					if(Locker[Locker_Floor][(Locker_Move_Flag-1)%4]==0)
					{
						if(Locker_Floor==1 && Elevator_Floor==0)
						{
							Elevator(2);
							Elevator_Floor=1;
						}
						LM_Guide_Move(LM_extra,0);
						TUK_Locker_Car_Move_Delay(Locker_Distance[Locker_Floor][(Locker_Move_Flag-1)%4],duty,Car_Go,3800);
						LM_Guide_Move(320,Relay_ON);
						HAL_Delay(500);
						LM_Guide_Move(LM_extra,0);
						TUK_Locker_Car_Move_Delay(Locker_Distance[Locker_Floor][(Locker_Move_Flag-1)%4],duty,Car_Back,3800);

						Locker[Locker_Floor][(Locker_Move_Flag-1)%4]=1;
					}
					Locker_Flag[Locker_Floor][(Locker_Move_Flag-1)%4]=0;

					if(Elevator_Floor==1)
					{
						Elevator(1);
						Elevator_Floor=0;
					}
				}
				else if(Locker_Flag[Locker_Floor][(Locker_Move_Flag-1)%4]==Lockeron)
				{
					if(Locker[Locker_Floor][(Locker_Move_Flag-1)%4]==1)
					{
						if(Locker_Floor==1 && Elevator_Floor==0)
						{
							Elevator(2);
							Elevator_Floor=1;
						}
						TUK_Locker_Car_Move_Delay(Locker_Distance[Locker_Floor][(Locker_Move_Flag-1)%4],duty,Car_Go,3800);
						LM_Guide_Move(320-LM_extra2,Relay_OFF);
						LM_Guide_Move(LM_extra,0);
						TUK_Locker_Car_Move_Delay(Locker_Distance[Locker_Floor][(Locker_Move_Flag-1)%4],duty,Car_Back,3800);
						LM_Guide_Move(0,0);

						Locker[Locker_Floor][(Locker_Move_Flag-1)%4]=2;
					}
					Locker_Flag[Locker_Floor][(Locker_Move_Flag-1)%4]=0;


					if(Elevator_Floor==1)
					{
						Elevator(1);
						Elevator_Floor=0;
					}


				}
				else if(Locker_Flag[Locker_Floor][(Locker_Move_Flag-1)%4]==Lockeroffcall)
				{
					if(Locker[Locker_Floor][(Locker_Move_Flag-1)%4]==2)
					{
						if(Locker_Floor==1 && Elevator_Floor==0)
						{
							Elevator(2);
							Elevator_Floor=1;
						}
						LM_Guide_Move(LM_extra,0);
						TUK_Locker_Car_Move_Delay(Locker_Distance[Locker_Floor][(Locker_Move_Flag-1)%4],duty,Car_Go,3800);
						LM_Guide_Move(320,Relay_ON);
						HAL_Delay(500);
						LM_Guide_Move(LM_extra,0);
						TUK_Locker_Car_Move_Delay(Locker_Distance[Locker_Floor][(Locker_Move_Flag-1)%4],duty,Car_Back,3800);

						Locker[Locker_Floor][(Locker_Move_Flag-1)%4]=3;
					}
					Locker_Flag[Locker_Floor][(Locker_Move_Flag-1)%4]=0;

					if(Elevator_Floor==1)
					{
						Elevator(1);
						Elevator_Floor=0;
					}
				}
				else if(Locker_Flag[Locker_Floor][(Locker_Move_Flag-1)%4]==Lockeroff)
				{
					if(Locker[Locker_Floor][(Locker_Move_Flag-1)%4]==3)
					{
						if(Locker_Floor==1 && Elevator_Floor==0)
						{
							Elevator(2);
							Elevator_Floor=1;
						}
						TUK_Locker_Car_Move_Delay(Locker_Distance[Locker_Floor][(Locker_Move_Flag-1)%4],duty,Car_Go,3800);
						LM_Guide_Move(320-LM_extra2,Relay_OFF);
						LM_Guide_Move(LM_extra,0);
						TUK_Locker_Car_Move_Delay(Locker_Distance[Locker_Floor][(Locker_Move_Flag-1)%4],duty,Car_Back,3800);
						LM_Guide_Move(0,0);

						Locker[Locker_Floor][(Locker_Move_Flag-1)%4]=0;
					}
					Locker_Flag[Locker_Floor][(Locker_Move_Flag-1)%4]=0;

					if(Elevator_Floor==1)
					{
						Elevator(1);
						Elevator_Floor=0;
					}


				}



				Locker_Move_Flag=0;
			}
			else if( Locker_Move_Flag == 3 || Locker_Move_Flag == 4 || Locker_Move_Flag == 7  || Locker_Move_Flag == 8)//
			{

				if(Locker_Flag[Locker_Floor][(Locker_Move_Flag-1)%4]==Lockeroncall)
				{
					if(Locker[Locker_Floor][(Locker_Move_Flag-1)%4]==0)
					{
						if(Locker_Floor==1 && Elevator_Floor==0)
						{
							Elevator(2);
							Elevator_Floor=1;
						}
						LM_Guide_Move(LM_extra,0);
						TUK_Locker_Car_Move_Delay(Locker_Distance[Locker_Floor][(Locker_Move_Flag-1)%4],duty,Car_Go,3800);
						HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, 1);
						LM_Guide_Move(0,0);
						HAL_Delay(500);
						LM_Guide_Move(320-LM_extra,0);
						TUK_Locker_Car_Move_Delay(Locker_Distance[Locker_Floor][(Locker_Move_Flag-1)%4],duty,Car_Back,3800);

						Locker[Locker_Floor][(Locker_Move_Flag-1)%4]=1;
					}
					Locker_Flag[Locker_Floor][(Locker_Move_Flag-1)%4]=0;

					if(Elevator_Floor==1)
					{
						Elevator(1);
						Elevator_Floor=0;
					}
				}
				else if(Locker_Flag[Locker_Floor][(Locker_Move_Flag-1)%4]==Lockeron)
				{
					if(Locker[Locker_Floor][(Locker_Move_Flag-1)%4]==1)
					{
						if(Locker_Floor==1 && Elevator_Floor==0)
						{
							Elevator(2);
							Elevator_Floor=1;
						}
						TUK_Locker_Car_Move_Delay(Locker_Distance[Locker_Floor][(Locker_Move_Flag-1)%4],duty,Car_Go,3800);
						LM_Guide_Move(LM_extra2,Relay_OFF);
						LM_Guide_Move(LM_extra,0);
						TUK_Locker_Car_Move_Delay(Locker_Distance[Locker_Floor][(Locker_Move_Flag-1)%4],duty,Car_Back,3800);
						LM_Guide_Move(0,0);

						Locker[Locker_Floor][(Locker_Move_Flag-1)%4]=2;
					}
					Locker_Flag[Locker_Floor][(Locker_Move_Flag-1)%4]=0;

					if(Elevator_Floor==1)
					{
						Elevator(1);
						Elevator_Floor=0;
					}


				}
				else if(Locker_Flag[Locker_Floor][(Locker_Move_Flag-1)%4]==Lockeroffcall)
				{
					if(Locker[Locker_Floor][(Locker_Move_Flag-1)%4]==2)
					{
						if(Locker_Floor==1 && Elevator_Floor==0)
						{
							Elevator(2);
							Elevator_Floor=1;
						}
						LM_Guide_Move(LM_extra,0);
						TUK_Locker_Car_Move_Delay(Locker_Distance[Locker_Floor][(Locker_Move_Flag-1)%4],duty,Car_Go,3800);
						HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, 1);
						LM_Guide_Move(0,0);
						HAL_Delay(500);
						LM_Guide_Move(320-LM_extra,0);
						TUK_Locker_Car_Move_Delay(Locker_Distance[Locker_Floor][(Locker_Move_Flag-1)%4],duty,Car_Back,3800);

						Locker[Locker_Floor][(Locker_Move_Flag-1)%4]=3;
					}
					Locker_Flag[Locker_Floor][(Locker_Move_Flag-1)%4]=0;

					if(Elevator_Floor==1)
					{
						Elevator(1);
						Elevator_Floor=0;
					}
				}
				else if(Locker_Flag[Locker_Floor][(Locker_Move_Flag-1)%4]==Lockeroff)
				{
					if(Locker[Locker_Floor][(Locker_Move_Flag-1)%4]==3)
					{
						if(Locker_Floor==1 && Elevator_Floor==0)
						{
							Elevator(2);
							Elevator_Floor=1;
						}
						TUK_Locker_Car_Move_Delay(Locker_Distance[Locker_Floor][(Locker_Move_Flag-1)%4],duty,Car_Go,3800);
						LM_Guide_Move(LM_extra2,Relay_OFF);
						LM_Guide_Move(LM_extra,0);
						TUK_Locker_Car_Move_Delay(Locker_Distance[Locker_Floor][(Locker_Move_Flag-1)%4],duty,Car_Back,3800);
						LM_Guide_Move(0,0);

						Locker[Locker_Floor][(Locker_Move_Flag-1)%4]=0;
					}
					Locker_Flag[Locker_Floor][(Locker_Move_Flag-1)%4]=0;

					if(Elevator_Floor==1)
					{
						Elevator(1);
						Elevator_Floor=0;
					}
				}


				Locker_Move_Flag=0;
			}
		}













	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */


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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
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
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 84-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 10000-1;
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
	sConfigOC.Pulse = 5000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

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
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 84-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 250-1;
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
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 125-1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

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
	htim6.Init.Prescaler = 8400-1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 10-1;
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
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

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
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, Wheel_Right_DIR_Pin|Wheel_Left_DIR_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|LM_ENR_Pin|LM_DIR_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pins : Wheel_Right_DIR_Pin Wheel_Left_DIR_Pin */
	GPIO_InitStruct.Pin = Wheel_Right_DIR_Pin|Wheel_Left_DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : RELAY_Pin */
	GPIO_InitStruct.Pin = RELAY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(RELAY_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 LM_ENR_Pin LM_DIR_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_0|LM_ENR_Pin|LM_DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : BOOT1_Pin */
	GPIO_InitStruct.Pin = BOOT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : limit_Pin */
	GPIO_InitStruct.Pin = limit_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(limit_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==limit_Pin)
	{
		if(Limit_Cnt==0)
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_7);
	}
}
void LM_Guide_ENA(int ena)
{
	if(ena)
		HAL_GPIO_WritePin(LM_ENR_GPIO_Port, LM_ENR_Pin, LM_ENA_Yes);
	else if(ena==0)
		HAL_GPIO_WritePin(LM_ENR_GPIO_Port, LM_ENR_Pin, LM_ENA_No);
}

void LM_Guide_DIR(int dir)
{
	if(dir)
		HAL_GPIO_WritePin(LM_DIR_GPIO_Port, LM_DIR_Pin, GPIO_PIN_SET);
	else if(dir == 0)
		HAL_GPIO_WritePin(LM_DIR_GPIO_Port, LM_DIR_Pin, GPIO_PIN_RESET);
}

void LM_Guide_Move(int LM_Position_Goal,int Magnet)
{
	int dir;
	int distance_to_move;

	distance_to_move = LM_Position_Goal - LM_Position;
	if(LM_Position_Goal >= 320)
		LM_Position_Goal = 320;
	if(LM_Position_Goal <= 0)
		LM_Position_Goal = 0;


	LM_Position += distance_to_move;
	if(distance_to_move>0)
		dir=1;
	else
	{
		dir=0;
		distance_to_move *= -1;
	}


	LM_Guide_ENA(1);
	LM_Guide_DIR(dir);
	TIM3->CCR1 = 125;
	LM_Guide_Step = 8500 * distance_to_move / 320;


	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);



	if(Magnet==Relay_ON)
	{
		HAL_Delay( (LM_Guide_Step/4) - 500 );
		HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, 1);
		HAL_Delay(1500);
	}
	else if(Magnet==Relay_OFF)
	{
		HAL_Delay( (LM_Guide_Step/4) );
		HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, 0);
		HAL_Delay(2000);
	}
	else
	{
		HAL_Delay( (LM_Guide_Step/4) );
	}
}

void Wheel_DIR(int dir)
{
	if(dir)
	{
		HAL_GPIO_WritePin(Wheel_Right_DIR_GPIO_Port, Wheel_Right_DIR_Pin, Wheel_Right_Go);
		HAL_GPIO_WritePin(Wheel_Left_DIR_GPIO_Port, Wheel_Left_DIR_Pin, Wheel_Left_Go);
	}
	else if(dir==0)
	{
		HAL_GPIO_WritePin(Wheel_Right_DIR_GPIO_Port, Wheel_Right_DIR_Pin, Wheel_Right_Back);
		HAL_GPIO_WritePin(Wheel_Left_DIR_GPIO_Port, Wheel_Left_DIR_Pin, Wheel_Left_Back);
	}
}

void TUK_Locker_Car_Move(int Car_Move_Delay,int Speed, int Car_DIR)
{
	Wheel_DIR(Car_DIR);

	if(Speed<0)
		Speed=0;
	if(Speed>9999)
		Speed=9999;
	TIM2->CCR1 = Speed;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	Car_Time=Car_Move_Delay;
	HAL_TIM_Base_Start_IT(&htim6);

}
void TUK_Locker_Car_Move_Delay(int Car_Move_Delay,int Speed, int Car_DIR,int Car_Stop)
{
	Wheel_DIR(Car_DIR);

	if(Speed<0)
		Speed=0;
	if(Speed>9999)
		Speed=9999;
	TIM2->CCR1 = Speed;


	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	Limit_Cnt=1000;
	if( Locker_Move_Flag == 2 || Locker_Move_Flag == 4  || Locker_Move_Flag == 6  || Locker_Move_Flag == 8 )
	{
		Limit_Cnt=2500;

	}
	Car_Time=Car_Move_Delay;
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_Delay(Car_Stop);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance==TIM6)
	{
		if(Car_Time>0)
		{
			Car_Time--;
			if(Limit_Cnt>0)
				Limit_Cnt--;
		}
		else if(Car_Time<=0)
		{
			LM_Guide_ENA(0);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_TIM_Base_Stop_IT(&htim6);

		}

	}





}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)
	{
		if(LM_Guide_Step>0)
		{
			LM_Guide_Step--;
		}
		else if(LM_Guide_Step<=0)
		{
			HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
		}

	}
}


void Elevator(int floor)
{
	// 192.168.0.224 is TUK_Elevator's IP
	if(floor==1)
	{
		Uart_sendstring("AT+CIPSTART=1,\"TCP\",\"192.168.0.224\",80\r\n",wifi_uart);
		while (!(Wait_for("OK\r\n",wifi_uart)));
		Uart_sendstring("CIPSTART 192.168.0.224 OK\n\n", pc_uart);

		Uart_sendstring("AT+CIPSEND=1,5\r\n",wifi_uart);
		while (!(Wait_for(">", wifi_uart)));

		Uart_sendstring ("11111", wifi_uart);
		while (!(Wait_for("SEND OK", wifi_uart)));

		Uart_sendstring("AT+CIPCLOSE=1\r\n", wifi_uart);
		while (!(Wait_for("OK\r\n", wifi_uart)));
		Uart_sendstring("1\r\n", pc_uart);
		if((Wait_for("11111", wifi_uart)))
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_7);
			Uart_flush (wifi_uart);
		}
	}
	else if(floor==2)
	{
		Uart_sendstring("AT+CIPSTART=1,\"TCP\",\"192.168.0.224\",80\r\n",wifi_uart);
		while (!(Wait_for("OK\r\n",wifi_uart)));
		Uart_sendstring("CIPSTART 192.168.0.224 OK\n\n", pc_uart);


		Uart_sendstring("AT+CIPSEND=1,5\r\n",wifi_uart);
		while (!(Wait_for(">", wifi_uart)));

		Uart_sendstring ("22222", wifi_uart);
		while (!(Wait_for("SEND OK", wifi_uart)));

		Uart_sendstring("AT+CIPCLOSE=1\r\n", wifi_uart);
		while (!(Wait_for("OK\r\n", wifi_uart)));
		Uart_sendstring("2\r\n", pc_uart);
		if((Wait_for("22222", wifi_uart)))
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_7);
			Uart_flush (wifi_uart);
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

