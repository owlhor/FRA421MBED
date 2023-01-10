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
  * -----------------------FRA421_MBED Final Exam ----------------------------
  * Author: owl_hor / 10 Jan 2023
  * - Using HC-35 Button Matrix Board / STM32H745 Nucleo
  * S[1..4] -> D[3..6] [PG6, PE14, PE11, PA8]
  * LED[1..8] -> D[7..14] [PG12, PG9, PD15, PD14, PB5, PA6, PA5, PB9]
  * - Design 1++ Person simple Game using these features one for at least
  * 	- RTC use
  * 	- Dual core
  * 	- Serial UART DMA, Interrupt Display
  * --------------------------------------------------------------------------
  * -o-o-o-o-o-o-o-o-o-o-o-o-  Whack a mole Game -o-o-o-o-o-o-o-o-o-o-o-o-o-o-
  *
  *  						  ??? How to play ???
  *
  * - Assume 1 LED as a mole -> There're 8 moles in this game
  * - To hit a mole, Using Red Button to hit a mole
  * 	-> BtnS1 hit moles 1,2
  * 	-> BtnS2 hit moles 3,4
  * 	-> BtnS3 hit moles 5,6
  * 	-> BtnS4 hit moles 7,8
  * - The moles in this game is special. You must """hit the moles together in the correct pair""". -> You'll get 1 point.
  * - If you hit a mole but not together in the correct pairs. -> the mole'll damage you -1 point.
  * 	ex. oo------ | btn 1 hit -> get  1 points // correct
  * 	ex. ----oo-- | btn 3 hit -> get  1 points // correct
  * 	ex. ----oo-- | btn 1 hit -> get  0 points // miss the moles
  * 	ex. -o-----o | btn 1 hit -> get -1 points // incomplete mole pair
  * 	ex. -oo----- | btn 2 hit -> get -1 points // incorrect pair
  *
  * - At first, You'll get "Score_start" points. If you reach "Score_WIN" points -> You Win.
  * 								 	 		 If you reach  0 points -> You Lose.
  * - There're 4 speed modes can be selected before the game starts: Easy | Hard | Cursed | Racknarock
  * -------------------------------------------------------------------------
  * - The scores & the game sequence'll be displayed via UART3 115200 8bit/stop 1 bit non-parity
  * - Using RTC to generate random sequence of the moles in mole_generator();
  * - Using External RTC(DS3231) with I2C2 to sync External->Internal RTC
  * - Using dual core: press blue button from CM7 send order to CM4 (for what ???!!!, so dumb)
  * 	(Actually, reciece All interrupt btns with CM4 and send flags to CM7 is one way can do. But for what? / I'm so lazy...)
  *
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DS_RTC.h"
#include "../../../Common/Src/SRAM4.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

#define Score_WIN 10
#define Score_start 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
uint8_t runner = 0;  //// using for mole generator
typedef union _runnerx_struct{
	struct _runs{
		uint8_t r7 : 1;
		uint8_t r6 : 1;
		uint8_t r5 : 1;
		uint8_t r4 : 1;
		uint8_t r3 : 1;
		uint8_t r2 : 1;
		uint8_t r1 : 1;
		uint8_t r0 : 1;
	}u1;
	uint8_t u8;
}runnerx_struct;

runnerx_struct rnx;  //// LED(mole) show buffer

uint8_t flag_btns = 0; 	//// flag which button is interrupt press
uint8_t chk; 			//// mole match check
int score = 0;			//// scores
uint8_t runo = 0;		//// LED sequence run buffer
enum{start,ready,playing,action,end} gameState = start;
/* Start: print start text
 * ready: wait for game run
 * playing: running the game
 * action: print the score back when red btns are pressed than back to playing
 * end: show result of the game WIN - LOSE
 * */

//// call RTC Times
RTC_TimeTypeDef NowTime;
RTC_DateTypeDef NowDate;

char txtUARTBF[100] = {0}; // UART Buffer

uint32_t timestamp_one[3] = {0};
uint16_t timetrig_one = 200;  // state speed
uint16_t timetrig_two = 1000; // mole generator speed
//uint32_t countert[3] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t randomrun_gen_RTC(uint8_t var);
void score_counter();
void mole_generator();
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

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  DS3231_Read(&hi2c2);
  EXIN_RTC_SYNC(&hi2c2,&hrtc);

  SRAM4->flag_blue_btn = 0;

  char temp[]="--------------------H745_ExamGame_CM4----------------------"
		  "\r\n Welcome to UART Port 115200 8 bit/stop1 none parity\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)temp, strlen(temp),30); // strlen = length of str -> config length of data

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  if(HAL_GetTick() - timestamp_one[1] >= timetrig_one){
			  timestamp_one[1] = HAL_GetTick();
			  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

			  switch(gameState){
			  case start:
				  sprintf(txtUARTBF, " - - - - Whack a Moles - - - - \r\n");
				  HAL_UART_Transmit(&huart3, (uint8_t*)txtUARTBF, strlen(txtUARTBF),30);
				  sprintf(txtUARTBF, " Press -Blue Button- to start:\r\n");
				  HAL_UART_Transmit(&huart3, (uint8_t*)txtUARTBF, strlen(txtUARTBF),30);

				  sprintf(txtUARTBF, "Select mode by red btn: Easy | Hard | Cursed | Racknarock (Default: Easy)\r\n");
				  HAL_UART_Transmit(&huart3, (uint8_t*)txtUARTBF, strlen(txtUARTBF),30);
				  gameState = ready;
				  break;

			  default:
			  case ready:
				  //// select speed mode
				  switch(flag_btns){
					  default:
						  break;
					  case 1:
						  timetrig_two = 1000;
						  sprintf(txtUARTBF, "Mode: Easy\r\n");
						  HAL_UART_Transmit(&huart3, (uint8_t*)txtUARTBF, strlen(txtUARTBF),10);
						  flag_btns = 0;
						  break;
					  case 2:
						  timetrig_two = 500;
						  sprintf(txtUARTBF, "Mode: Hard\r\n");
						  HAL_UART_Transmit(&huart3, (uint8_t*)txtUARTBF, strlen(txtUARTBF),10);
						  flag_btns = 0;
						  break;
					  case 3:
						  timetrig_two = 250;
						  sprintf(txtUARTBF, "Mode: Cursed\r\n");
						  HAL_UART_Transmit(&huart3, (uint8_t*)txtUARTBF, strlen(txtUARTBF),10);
						  flag_btns = 0;
						  break;
					  case 4:
						  timetrig_two = 100;
						  sprintf(txtUARTBF, "Mode: Racknarock\r\n");
						  HAL_UART_Transmit(&huart3, (uint8_t*)txtUARTBF, strlen(txtUARTBF),10);
						  flag_btns = 0;
						  break;
					  }

				  if(SRAM4->flag_blue_btn == 1){
					  SRAM4->flag_blue_btn = 0;
					  score = Score_start;

					  sprintf(txtUARTBF, " - Ready - \r\n");
					  HAL_UART_Transmit(&huart3, (uint8_t*)txtUARTBF, strlen(txtUARTBF),30);
					  HAL_Delay(1000);

					  sprintf(txtUARTBF, " - Set - \r\n");
					  HAL_UART_Transmit(&huart3, (uint8_t*)txtUARTBF, strlen(txtUARTBF),30);
					  HAL_Delay(1000);

					  sprintf(txtUARTBF, " - Go! - \r\nscore = %d \r\n",score);
					  HAL_UART_Transmit(&huart3, (uint8_t*)txtUARTBF, strlen(txtUARTBF),30);
					  HAL_Delay(1000);


					  gameState = playing;

				  }
				  break;

			  case playing:
				  //// mole generator
				  mole_generator();

				  //// use interrupt to go to action
				  if(flag_btns != 0){gameState = action;}


				  if (score >= Score_WIN){
					  gameState = end;
					  sprintf(txtUARTBF, "--- You Win!!! --- \r\n  Press -Blue Button- to home:\r\n");
					  HAL_UART_Transmit(&huart3, (uint8_t*)txtUARTBF, strlen(txtUARTBF),30);

					  timetrig_one = 100; // change state's speed for LED running
				  }

				  if (score <= 0){
					  gameState = end;
					  sprintf(txtUARTBF, "--- You Lose - TT TT TT TT \r\n  Press -Blue Button- to home:\r\n");
					  HAL_UART_Transmit(&huart3, (uint8_t*)txtUARTBF, strlen(txtUARTBF),30);
				  }
				  break;

			  case action: //// score process and print UART

				  switch(flag_btns){
				  default:
					  break;
				  case 1:
					  chk = rnx.u1.r0 + rnx.u1.r1;
					  score_counter();
					  rnx.u1.r0 = 0; rnx.u1.r1 = 0;
					  HAL_GPIO_WritePin(LED_D7_GPIO_Port, LED_D7_Pin, !(rnx.u1.r0));
					  HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, !(rnx.u1.r1));
					  break;
				  case 2:
					  chk = rnx.u1.r2 + rnx.u1.r3;
					  score_counter();
					  rnx.u1.r2 = 0; rnx.u1.r3 = 0;
					  HAL_GPIO_WritePin(LED_D9_GPIO_Port, LED_D9_Pin, !(rnx.u1.r2));
					  HAL_GPIO_WritePin(LED_D10_GPIO_Port, LED_D10_Pin, !(rnx.u1.r3));
					  break;
				  case 3:
					  chk = rnx.u1.r4 + rnx.u1.r5;
					  score_counter();
					  rnx.u1.r4 = 0; rnx.u1.r5 = 0;
					  HAL_GPIO_WritePin(LED_D11_GPIO_Port, LED_D11_Pin, !(rnx.u1.r4));
					  HAL_GPIO_WritePin(LED_D12_GPIO_Port, LED_D12_Pin, !(rnx.u1.r5));
				  	  break;
				  case 4:
					  chk = rnx.u1.r6 + rnx.u1.r7;
					  score_counter();
					  rnx.u1.r6 = 0; rnx.u1.r7 = 0;
					  HAL_GPIO_WritePin(LED_D13_GPIO_Port, LED_D13_Pin, !(rnx.u1.r6));
					  HAL_GPIO_WritePin(LED_D14_GPIO_Port, LED_D14_Pin, !(rnx.u1.r7));
				  	  break;
				  }

				 sprintf(txtUARTBF, "score = %d \r\n",score);
				 HAL_UART_Transmit(&huart3, (uint8_t*)txtUARTBF, strlen(txtUARTBF),30);
				 HAL_Delay(100);

				  flag_btns = 0;
				  gameState = playing;
				  break;

			  case end:

				  if (score >= Score_WIN){

				  	 runo++;
				  	 runo%=8;
				  	 rnx.u8 = (uint8_t)(0xff - (1 << runo));
				  }

				  if (score <= 0){
					  runo++;
					  runo%=2;
					  rnx.u8 = (runo>0)? 0xff:0x00;
				  }


				  if(SRAM4->flag_blue_btn == 1){

					  timetrig_one = 200; // change state's speed for LED running back
					  SRAM4->flag_blue_btn = 0;
					  gameState = start;
				  	}
				  break;
			  }


		  }

	  //// LED Moles writer
	  if(HAL_GetTick() - timestamp_one[2] >= 100){
		  timestamp_one[2] = HAL_GetTick();


		  HAL_GPIO_WritePin(LED_D7_GPIO_Port, LED_D7_Pin, !(rnx.u1.r0));
		  HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, !(rnx.u1.r1));
		  HAL_GPIO_WritePin(LED_D9_GPIO_Port, LED_D9_Pin, !(rnx.u1.r2));
		  HAL_GPIO_WritePin(LED_D10_GPIO_Port, LED_D10_Pin, !(rnx.u1.r3));

		  HAL_GPIO_WritePin(LED_D11_GPIO_Port, LED_D11_Pin, !(rnx.u1.r4));
		  HAL_GPIO_WritePin(LED_D12_GPIO_Port, LED_D12_Pin, !(rnx.u1.r5));
		  HAL_GPIO_WritePin(LED_D13_GPIO_Port, LED_D13_Pin, !(rnx.u1.r6));
		  HAL_GPIO_WritePin(LED_D14_GPIO_Port, LED_D14_Pin, !(rnx.u1.r7));

	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00B03FDB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_D13_Pin|LED_D12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LED_D11_Pin|LED_D14_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_D10_Pin|LED_D9_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED_D8_Pin|LED_D7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_D13_Pin LED_D12_Pin */
  GPIO_InitStruct.Pin = LED_D13_Pin|LED_D12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Btn_D5_Pin Btn_D4_Pin */
  GPIO_InitStruct.Pin = Btn_D5_Pin|Btn_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_D10_Pin LED_D9_Pin */
  GPIO_InitStruct.Pin = LED_D10_Pin|LED_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Btn_D3_Pin */
  GPIO_InitStruct.Pin = Btn_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Btn_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Btn_D6_Pin */
  GPIO_InitStruct.Pin = Btn_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Btn_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_D8_Pin LED_D7_Pin */
  GPIO_InitStruct.Pin = LED_D8_Pin|LED_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_D11_Pin LED_D14_Pin */
  GPIO_InitStruct.Pin = LED_D11_Pin|LED_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void score_counter(){
	if(chk == 2){score++;}
	else if(chk == 1){score--;}
	else{score = score;}
}

void mole_generator(){
	 if(HAL_GetTick() - timestamp_one[0] >= timetrig_two){
		  timestamp_one[0] = HAL_GetTick();
		  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

		 //// RTC Runner
		  HAL_RTC_GetTime(&hrtc, &NowTime, RTC_FORMAT_BCD);
		  HAL_RTC_GetDate(&hrtc, &NowDate, RTC_FORMAT_BCD);

		 ///// generate mole's random sequence
		  runner++;
		  runner %= 16;
		  runner = randomrun_gen_RTC(runner);

		  rnx.u8++;
		  rnx.u8 %= 16;
		  rnx.u8 =  (runner << 4) + rnx.u8 ;

//			  sprintf(txtUARTBF, "rx %d %d %d %d %d %d %d %d \r\n",
//					  rnx.u1.r0, rnx.u1.r1, rnx.u1.r2, rnx.u1.r3, rnx.u1.r4, rnx.u1.r5, rnx.u1.r6,rnx.u1.r7);
//			  HAL_UART_Transmit(&huart3, (uint8_t*)txtUARTBF, strlen(txtUARTBF),100);
//
//			  sprintf(txtUARTBF, "runner = %d \r\n",runner);
//			  HAL_UART_Transmit(&huart3, (uint8_t*)txtUARTBF, strlen(txtUARTBF),30);

			  }
}

//void mole_seq_1(){
//	static uint8_t runo = 0;
//	 runo++;
//	 runo%=8;
//	 rnx.u8 = !(1 << runo);
//}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	if(GPIO_Pin == GPIO_PIN_13){ // blue btn
//		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//		SRAM4->flag_blue_btn = 1;
//
//		}

	if(GPIO_Pin == GPIO_PIN_6){ // D3
		//// test interrupt
//				HAL_GPIO_WritePin(LED_D7_GPIO_Port, LED_D7_Pin, GPIO_PIN_RESET);
//				while(HAL_GPIO_ReadPin(Btn_D3_GPIO_Port,Btn_D3_Pin) == GPIO_PINh_RESET){
//						HAL_GPIO_TogglePin(LED_D7_GPIO_Port, LED_D7_Pin);            // While state loop
//						HAL_Delay(100);
//					}
//				HAL_GPIO_WritePin(LED_D7_GPIO_Port, LED_D7_Pin, GPIO_PIN_SET);
			flag_btns = 1;
			//gameState = action;


}


	if(GPIO_Pin == GPIO_PIN_14){ // D4
		//// test interrupt
//		HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, GPIO_PIN_RESET);
//		while(HAL_GPIO_ReadPin(Btn_D4_GPIO_Port,Btn_D4_Pin) == GPIO_PIN_RESET){
//				HAL_GPIO_TogglePin(LED_D8_GPIO_Port, LED_D8_Pin);            // While state loop
//				HAL_Delay(100);
//			}
//		HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, GPIO_PIN_SET);

		flag_btns = 2;
		//gameState = action;

		}

	if(GPIO_Pin == GPIO_PIN_11){ // D5

		//// test interrupt
//			HAL_GPIO_WritePin(LED_D9_GPIO_Port, LED_D9_Pin, GPIO_PIN_RESET);
//			while(HAL_GPIO_ReadPin(Btn_D5_GPIO_Port,Btn_D5_Pin) == GPIO_PIN_RESET){
//					HAL_GPIO_TogglePin(LED_D9_GPIO_Port, LED_D9_Pin);            // While state loop
//					HAL_Delay(100);
//				}
//			HAL_GPIO_WritePin(LED_D9_GPIO_Port, LED_D9_Pin, GPIO_PIN_SET);

		flag_btns = 3;
		//gameState = action;

			}

	if(GPIO_Pin == GPIO_PIN_8){ // D6
		//// test interrupt
//		HAL_GPIO_WritePin(LED_D10_GPIO_Port, LED_D10_Pin, GPIO_PIN_RESET);
//		while(HAL_GPIO_ReadPin(Btn_D6_GPIO_Port,Btn_D6_Pin) == GPIO_PIN_RESET){
//				HAL_GPIO_TogglePin(LED_D10_GPIO_Port, LED_D10_Pin);            // While state loop
//				HAL_Delay(100);
//			}
//		HAL_GPIO_WritePin(LED_D10_GPIO_Port, LED_D10_Pin, GPIO_PIN_SET);

		flag_btns = 4;
		//gameState = action;

		}

}

uint8_t randomrun_gen_RTC(uint8_t var){

	//// Generate cat cat random from RTC
	static uint8_t cg = 0;
	static uint8_t ci = 0;
	cg++;
	ci+=cg;
	uint8_t varx = (var + NowTime.Seconds + (NowTime.Minutes << 4) + (NowTime.Hours << 2) + cg + ci);
	return varx % 256;
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
