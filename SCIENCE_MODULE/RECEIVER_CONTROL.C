/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
CAN_FilterTypeDef sFilterConfig;			//struct containing filter settings
CAN_RxHeaderTypeDef RxMessage;	 //struct for recieved data frame
CAN_TxHeaderTypeDef TxMessage;   // struct for transmitted dataframe
uint8_t rxData[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; //array for recieved data (8 bytes)
uint8_t txData[8] = { 10, 0, 0, 0, 0, 0, 0, 0 };
uint32_t usedmailbox;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Timer_Initialize(void);
void GPIO_Initialize(void);

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_CAN_Init();
	/* USER CODE BEGIN 2 */
	Timer_Initialize();
	GPIO_Initialize();

	//RX CODE

	TxMessage.IDE = CAN_ID_STD;				//standard identifier format (11bit)
	TxMessage.StdId = 0x211;								//identifier value
	TxMessage.RTR = CAN_RTR_DATA;//indicates frame mode (data frame or remote frame)
	TxMessage.DLC = 8;									//data length (8 bytes)
	TxMessage.TransmitGlobalTime = DISABLE;	//time of transmission is not transmitted along with the data

	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//filter bank consists of 2 32bit values (mask and ID)
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;//filter set to mask and ID mode
	sFilterConfig.FilterBank = 0;				//filter bank number 0 selected
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;//assign filter bank to FIFO 0
	sFilterConfig.FilterIdHigh = 0x446 << 5;//STD ID value is 7, here shifted by 5 because 11 bits starting from the left are for STD ID (FilterIdHigh is 16bit)
	sFilterConfig.FilterIdLow = 0;										//LSB
	sFilterConfig.FilterMaskIdHigh = 0x446 << 5;//0b111 shifted by 5 for the same reason, first 11 bits are for Identifier
	sFilterConfig.FilterMaskIdLow = 0;								//LSB
	sFilterConfig.FilterActivation = ENABLE;				//activate filter

	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);	//commits filter settings
	HAL_CAN_Start(&hcan);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		//HAL_CAN_AddTxMessage(&hcan, &TxMessage, txData, &usedmailbox);//send data
		//HAL_Delay(500);

		if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0))//checks if the number of messages in FIFO 0 is non zero
				{
			HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxMessage, rxData);//stores the data frame in RxMessage struct, stores data in rsData array
			int sm = rxData[0];
			switch (sm) {
			case 0: // **** Master shutdown***** kill all the processes which are running
				TIM4->CCR1 = 0;
				TIM4->CCR2 = 0;
				TIM4->CCR3 = 0;
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
				break;
			case 1:		// relay 1
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				//HAL_Delay(100);

				break;
			case 2:		//relay 2
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
				//HAL_Delay(1000);

				break;
			case 3:		// relay 3
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
				//HAL_Delay(1000);

				break;
			case 4:		// relay 4
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				//HAL_Delay(1000);
				break;
			case 5:		// relay 5
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
				//HAL_Delay(1000);

				break;
			case 6:		// relay 6
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
				//HAL_Delay(1000);

				break;
			case 7:		// relay 7
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
				//HAL_Delay(1000);
				break;
			case 8:		// relay 8
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
				//HAL_Delay(1000);
				break;
			case 9:		// water heater
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
				//HAL_Delay(1000);
				break;
			case 10:	//servo rotate
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_Delay(50);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_Delay(50);
				break;
			case 11: // servo direction toggle
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
				break;
			case 12: // auger down with rotation
				GPIOA->BSRR |= 1 << 4;
				GPIOA->BSRR |= 1 << 5;
				TIM4->CCR1 = 8000;
				TIM4->CCR2 = 8000;
				break;
			case 13: // auger up
				GPIOA->BRR |= 1 << 4;
				GPIOA->BRR |= 1 << 5;
				TIM4->CCR1 = 8000;
				TIM4->CCR2 = 0;
				break;
			case 14: // auger rotate for deposition
				GPIOA->BRR |= 1 << 4;
				GPIOA->BRR |= 1 << 5;
				TIM4->CCR1 = 0;
				TIM4->CCR2 = 8000;
				break;
			case 15: // sensor suite goes up
				GPIOA->BSRR |= 1 << 9;
				TIM4->CCR3 = 8000;
				break;
			case 16: // sensor suite goes down
				GPIOA->BRR |= 1 << 9;
				TIM4->CCR3 = 8000;
				break;
			default: // shut down all the timers which are working
				TIM4->CCR1 = 0;
				TIM4->CCR2 = 0;
				TIM4->CCR3 = 0;
				break;
			}

		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 8;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6
					| GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC14 PC15 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA1 PA2 PA3
	 PA6 PA7 PA8 PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
			| GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void GPIO_Initialize() {

	// CONFIGURE PORT C PIN 13 -> ONBOARD LED
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC->CRH |= GPIO_CRH_MODE13;
	GPIOC->CRH &= ~(GPIO_CRH_CNF13);

	//Enable Clocks:
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   //Enable Clock for Port A
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   //Enable Clock for Port B
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   //Enable Alternate Function

	//Setup PA4:
	GPIOA->CRL |= GPIO_CRL_MODE4;   //OUTPUT Mode 50Mhz
	GPIOA->CRL &= ~(GPIO_CRL_CNF4);   //Output Push-Pull

	//Setup PA5:
	GPIOA->CRL |= GPIO_CRL_MODE5;   //OUTPUT Mode 50Mhz
	GPIOA->CRL &= ~(GPIO_CRL_CNF5);   //Output Push-Pull

	//Setup PB5:
	GPIOB->CRL |= GPIO_CRL_MODE5;   //OUTPUT Mode 50Mhz
	//Enable AF Mode
	GPIOB->CRL |= GPIO_CRL_CNF5_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF5_0);

	//Setup PB6:
	GPIOB->CRL |= GPIO_CRL_MODE6;   //OUTPUT Mode 50Mhz
	//Enable AF Mode:
	GPIOB->CRL |= GPIO_CRL_CNF6_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF6_0);

	//Setup PB7:
	GPIOB->CRL |= GPIO_CRL_MODE7;   //OUTPUT Mode 50Mhz
	//Enable AF Mode:
	GPIOB->CRL |= GPIO_CRL_CNF7_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF7_0);

	//PA10 Setup: (UART Rx)
	GPIOA->CRH &= ~(GPIO_CRH_MODE10);   //INPUT Mode (00)
	GPIOA->CRH |= GPIO_CRH_CNF10;   //Input with pull-up/pull-down (10)
	GPIOA->CRH &= ~(GPIO_CRH_CNF10_0);

	//PB4 Setup:
	//Disable SWD & JTAG:
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_2;
	AFIO->MAPR &= ~(AFIO_MAPR_SWJ_CFG_1 | AFIO_MAPR_SWJ_CFG_0);
	GPIOB->CRL |= (GPIO_CRL_MODE4);   //OUTPUT Mode (11)
	GPIOB->CRL &= ~(GPIO_CRL_CNF4);   //Output Push-Pull (00)
	GPIOB->BRR = 1 << (4);   //Mandatory turn off
}

void Timer_Initialize() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;   //Enable Timer4
	TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E; //Enable Channel 1,2 and 4 as OUTPUT
	TIM4->CR1 |= TIM_CR1_ARPE;   //Enable Auto Re-Load Preload (ARPE)

	TIM4->CCMR1 |= TIM_CCMR1_OC1PE;   //Enable Preload for Channel 1
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;   //Enable Preload for Channel 2
	TIM4->CCMR2 |= TIM_CCMR2_OC3PE;   //Enable preload for channel 3

	//PWM Mode 1 for Channel 1:
	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M_0);
	//PWM Mode 1 for Channel 2:
	TIM4->CCMR1 |= (TIM_CCMR1_OC2M_2) | (TIM_CCMR1_OC2M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M_0);
	//PWM Mode 1 for Channel 3:
	TIM4->CCMR2 |= (TIM_CCMR2_OC3M_2) | (TIM_CCMR2_OC3M_1);
	TIM4->CCMR2 &= ~(TIM_CCMR2_OC3M_0);

	TIM4->PSC = 1;   //freq/1 = 8 Mhz
	TIM4->ARR = 8000;
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;
	TIM4->CCR3 = 0;

	TIM4->EGR |= TIM_EGR_UG;   //Update Registers
	TIM4->CR1 |= TIM_CR1_CEN;   //Start Counting
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
