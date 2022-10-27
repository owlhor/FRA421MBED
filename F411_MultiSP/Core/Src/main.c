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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define dynamix_WRKS

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
char TxDataBuffer[32] =
{ 0 };
char RxDataBuffer[32] =
{ 0 };
char UIBuffex[50] =
{ 0 };


uint8_t flag_spi2_read = 0;

typedef union _MCP3002_SET{
	struct MC
	{
		uint16_t bitread :12;
		uint16_t reserv :2; // LSB
		enum _REQFig{M_Diff_01,M_Diff_10,M_SE_CH0,M_SE_CH1} REQFig :2; // MSB
	}MCP3002_8;
	uint16_t U16;
}MCP3002_SET;

MCP3002_SET MCrq1;
MCP3002_SET MCrq2;
MCP3002_SET MCread1;
MCP3002_SET MCread2;


uint16_t A_bitread= 0;
float VADC_c = 0.0;
////========Dynamix===========================================
//// 0xFF	0xFF Packet ID	Length	Instruction	Param 1	…	Param N	CHKSUM
typedef union _dynamixel_buffer_1p0{
	struct dx{
		uint8_t header1;
		uint8_t header2;
		uint8_t DXL_ID;
		uint8_t d_length;
		enum {Ping = 0x01,read,write,reg_write,action,factory_reset,reboot = 0x08}Instruct;
		uint8_t param[10];
	}dymix;
	uint8_t U8[15];
} dynamix_bf10;
dynamix_bf10 dyna_01;

typedef union _U16Cvt{
	uint16_t U16;
	uint8_t U8[2];
}U16Cvt;

//U16Cvt A_bitreadx;

union {
	uint8_t U8[2];
	uint16_t U16;
} goalpos = {0};

uint8_t flag_dyna = 0;
uint16_t counter = 0;
uint8_t wdg_tig = 0;
uint32_t timestamp_one = 0;
uint32_t timestamp_wdg = 0;
uint32_t timestamp_dmx = 0;
uint64_t _micro;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void MCP3002_READ(uint16_t pTrX, uint16_t *pRcX);
uint64_t micros();
int16_t UARTRecieveIT();
void dynamix_enable_torque();
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM11_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim11);

  MCrq1.MCP3002_8.REQFig = M_SE_CH0;
  MCrq2.MCP3002_8.REQFig = M_SE_CH1;
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  dyna_01.dymix.header1 = 0xff;
  dyna_01.dymix.header2 = 0xff;
  dyna_01.dymix.DXL_ID = 0x03;
  dyna_01.dymix.Instruct = write;

  dynamix_enable_torque();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_UART_Receive_IT(&huart6,  (uint8_t*)RxDataBuffer, 32);

	  //// External Watchdog
	  if(HAL_GetTick()- timestamp_wdg >= 1400 ){ ////&& wdg_tig == 0
	  		  timestamp_wdg = HAL_GetTick();
	  		  // Toggle edge to watchdog xternal ic
	  		  HAL_GPIO_TogglePin(WDG_TG_GPIO_Port, WDG_TG_Pin);
	  	  }

	  ////========SPI MCP3002 ========================================
	  if(HAL_GetTick()- timestamp_one >= 500){
		  timestamp_one = HAL_GetTick();
		  flag_spi2_read = 1;
	  }

	  if (flag_spi2_read != 0 && hspi2.State == HAL_SPI_STATE_READY
							&& HAL_GPIO_ReadPin(SPI2_CS_GPIO_Port, SPI2_CS_Pin)
									== GPIO_PIN_SET)
		{

		  counter++;
		  //MCP3002_READ(MCrq1.U16, &MCread1.U16); //&A_bitread
		  if(counter%2==0){
			  MCP3002_READ(MCrq1.U16, &MCread1.U16); //&A_bitread
		  }
		  else{
			  MCP3002_READ(MCrq1.U16, &MCrq1.U16); //&A_bitread
		  }


		  //// Dout = ( 4096 x Vin )/ VCC
		  //// Dout x VCC / 4096 = Vin
		  //// << 1 to add 1 lost LSB
		  VADC_c = (MCread1.MCP3002_8.bitread << 1) * 0.00120; //// 1/4096 *5 = 5 * 0.000244140625

		  flag_spi2_read = 0;
		}

#ifdef dynamix_WRKS
	  //////========== Dynamixel =======
	  if(HAL_GetTick()- timestamp_dmx >= 500){
			  //// && flag_dyna != 0
		  	  timestamp_dmx = HAL_GetTick();
			  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

			  goalpos.U16 += 30;
			  //goalpos.U16 = 778;
			  if(goalpos.U16 >= 1020){goalpos.U16 -= 1000;}
			  //uint8_t chksum = 0;
			  uint16_t chksumii = 0;
			  uint8_t dg = 0;

			  static uint8_t ledss = 0;
			  ledss++; ledss%=2;

			  //// LED
			  dyna_01.dymix.param[dg] = 25; dg++; ////-- protoc[4+1]
			  dyna_01.dymix.param[dg] = ledss; dg++; ////-- protoc[4+2]

			  //// Goalpos
			  dyna_01.dymix.param[dg] = 30; dg++; ////-- protoc[4+1]
			  dyna_01.dymix.param[dg] = goalpos.U8[1]; dg++; ////-- protoc[4+2]
			  dyna_01.dymix.param[dg] = goalpos.U8[0]; dg++; ////-- protoc[4+3]


			  dyna_01.dymix.d_length = dg + 2;
 			  //// Checksum = ~( ID + Length + Instruction + Parameter1 + … Parameter N )
			  for(int i = 2; i <= dyna_01.dymix.d_length + 2; i++){
				  chksumii += dyna_01.U8[i];
			  }
			  dyna_01.dymix.param[dg] = ~(chksumii & 0xff) ; dg++;

			  HAL_UART_Transmit_IT(&huart6, &dyna_01.U8[0] , dyna_01.dymix.d_length + 4);
			  //HAL_UART_Receive_IT(&huart6,  (uint8_t*)RxDataBuffer, 32);
			  //flag_dyna--;
	  	  	  }
#endif

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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 57600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WDG_TG_GPIO_Port, WDG_TG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin SPI2_CS_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : WDG_TG_Pin */
  GPIO_InitStruct.Pin = WDG_TG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WDG_TG_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void dynamix_enable_torque(){
	//// enable torque
		uint8_t dg = 0;
		uint16_t chksumii = 0;
		U16Cvt movspeed, torque_lim;
		movspeed.U16 = 1000;
		torque_lim.U16 = 1023;
	//// enable torque
	  dyna_01.dymix.param[dg] = 24; dg++; ////-- protoc[4+1]
	  dyna_01.dymix.param[dg] = 1; dg++; ////-- protoc[4+2]

	  dyna_01.dymix.param[dg] = 26; dg++; ////-- protoc[4+1]
	  dyna_01.dymix.param[dg] = 1; dg++; ////-- protoc[4+2]

	  dyna_01.dymix.param[dg] = 27; dg++; ////-- protoc[4+1]
	  dyna_01.dymix.param[dg] = 1; dg++; ////-- protoc[4+2]

	  dyna_01.dymix.param[dg] = 28; dg++; ////-- protoc[4+1]
	  dyna_01.dymix.param[dg] = 32; dg++; ////-- protoc[4+2]

	  dyna_01.dymix.param[dg] = 29; dg++; ////-- protoc[4+1]
	  dyna_01.dymix.param[dg] = 32; dg++; ////-- protoc[4+2]

	  //// Moving speed
	  dyna_01.dymix.param[dg] = 32; dg++; ////-- protoc[4+1]
	  dyna_01.dymix.param[dg] = movspeed.U8[1]; dg++; ////-- protoc[4+2]
	  dyna_01.dymix.param[dg] = movspeed.U8[0]; dg++; ////-- protoc[4+2]

	  dyna_01.dymix.param[dg] = 34; dg++; ////-- protoc[4+1]
	  dyna_01.dymix.param[dg] = torque_lim.U8[1]; dg++; ////-- protoc[4+2]
	  dyna_01.dymix.param[dg] = torque_lim.U8[0]; dg++; ////-- protoc[4+2]


	  dyna_01.dymix.d_length = dg + 2;
	  //// Checksum = ~( ID + Length + Instruction + Parameter1 + … Parameter N )
	  //// start at ID[2], length[3], instr[4] , param[5,6,7]
	  for(int i = 2; i <= dyna_01.dymix.d_length + 2; i++){
		  chksumii += dyna_01.U8[i];
	  }
	  dyna_01.dymix.param[dg] = ~(chksumii & 0xff) ; dg++;

	  HAL_UART_Transmit_IT(&huart6, &dyna_01.U8[0] , dyna_01.dymix.d_length + 4);

}

int16_t UARTRecieveIT() // find last update position and return data in that position
{
	static uint32_t dataPos = 0;
	int16_t data=-1; // return -1 if no data
	// xfer size = 32, xfercount will down every time text typed in

	if(huart6.RxXferSize - huart6.RxXferCount!=dataPos) // update when datapos is changed
	{
		data=RxDataBuffer[dataPos];
		dataPos= (dataPos+1) % huart6.RxXferSize; // update and % 32(overflow)
	}
	return data;
}

void MCP3002_READ(uint16_t pTrX, uint16_t *pRcX){

	//uint16_t datain = pTrX; //0b1100000000000000
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	//HAL_SPI_Transmit_IT(&hspi2, &datain, 1);
	//HAL_SPI_TransmitReceive(&hspi2, pTrX, &pRcX, 1, 100);

	HAL_SPI_TransmitReceive_IT(&hspi2, &pTrX, pRcX, 1);

	//// Dout = ( 4096 x Vin )/ VCC
	//// Dout x VCC / 4096 = Vin

}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi2)
	{
		// set cs back to 1, finished
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		//wdg_tig++; // trickey stop watchdog tog
		flag_spi2_read = 1;
		}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim11)
	{_micro += 65535;}
}
inline uint64_t micros()
{
	return htim11.Instance->CNT + _micro;
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
