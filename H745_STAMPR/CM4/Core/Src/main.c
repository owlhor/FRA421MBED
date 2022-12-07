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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../../Common/Src/SRAM4.h"
#include "DS_RTC.h"
#include "MFRC522.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
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

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint64_t _micros = 0;
uint32_t timestamp_one = 0;
uint32_t timestamp_two = 0;

RTC_TimeTypeDef NowTime;
RTC_DateTypeDef NowDate;

////-------- MFRC522 --------------
uint8_t bufferMM[8] = {0};
uint8_t rc522_version;
u_char status_5221;

uint8_t stcnt[8] = {0};
u_char UID[5];
u_char status_522;
u_char cardstr[MAX_LEN + 1];
u_char card_data[17];
uint32_t delay_val = 1000; //ms
uint16_t result = 0;
uint8_t value = 0;

char str1[17]={'\0'};
char str2[17]={'\0'};
char str3[17]={'\0'};
char str4[17]={'\0'};
char tmp_str[65]={'\0'};
// a private key to scramble data writing/reading to/from RFID card:
u_char Mx1[7][5]={{0x12,0x45,0xF2,0xA8},{0xB2,0x6C,0x39,0x83},{0x55,0xE5,0xDA,0x18},
		  	  	  	{0x1F,0x09,0xCA,0x75},{0x99,0xA2,0x50,0xEC},{0x2C,0x88,0x7F,0x3D}};
u_char SectorKey[7];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_TIM17_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI4_Init(void);
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
  MX_TIM17_Init();
  MX_RTC_Init();
  MX_I2C2_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */

  //HAL_TIM_Base_Start_IT(&htim17);

  //// Start sync EXIN RTC
  EXIN_RTC_SYNC(&hi2c2,&hrtc);
  MFRC522_HardResetSet();
  MFRC522_Init();
  //// MFRC522 version 2.0 software version is: 92h
  rc522_version = Read_MFRC522(VersionReg);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //SRAM4->state1 = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
	  //chk2 = SRAM4->state1;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(HAL_GetTick() - timestamp_one >= 500){
		  timestamp_one = HAL_GetTick();
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		  DS3231_Read(&hi2c2);

		  if(HAL_HSEM_Take(1, 1) == HAL_OK){
		  //read RTC NEED TO READ BOTH IN OTHER
		  HAL_RTC_GetTime(&hrtc, &NowTime, RTC_FORMAT_BCD);
		  HAL_RTC_GetDate(&hrtc, &NowDate, RTC_FORMAT_BCD);
		  SRAM4->NowTimes = NowTime;
		  SRAM4->NowDates = NowDate;

		  HAL_HSEM_Release(1, 1);
		  	  }

	  }

	  if(HAL_GetTick() - timestamp_two >= 5000){

//		  //// test-------------------------
//		  uint8_t addr00[6] = {0x01,0x09,0x0A,0x0D,0x11,0x13};
//		  for(int i = 2;i < 5;i++){
//		  //// NSS pin trig using Hardware output NSS signal / setting at ioc
//		  //HAL_GPIO_WritePin(RC522_CS_GPIO_Port, RC522_CS_Pin, GPIO_PIN_RESET);
//		  HAL_SPI_Transmit(&hspi4, &addr00[i], 1, 100);
//		  HAL_SPI_Receive(&hspi4, &bufferMM[i], 1, 100);
//		  //HAL_GPIO_WritePin(RC522_CS_GPIO_Port, RC522_CS_Pin, GPIO_PIN_SET);
//		  }
//		  HAL_Delay(500);
//		  //// test-------------------------

		  for (int i = 0; i < 16; i++) {
			  cardstr[i] = 0;
		  }
		  status_522 = 0;
		  // Find cards
		  stcnt[0]++;
		  status_522 = MFRC522_Request(PICC_REQIDL, cardstr);
		  if(status_522 == MI_OK) {
			  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
			  //LED2_GPIO_Port -> BSRR = LED2_Pin;
			  result = 0;
			  result++;
			  sprintf(str1,"Card:%x,%x,%x", cardstr[0], cardstr[1], cardstr[2]);
			  //
			  // Anti-collision, return card serial number == 4 bytes
			  //DWT_Delay_ms(1);
			  HAL_Delay(1);

			  stcnt[1]++;
			  status_5221 = MFRC522_Anticoll(cardstr);
			  if(status_5221 == MI_OK) {

				  result++;
				  sprintf(str2,"UID:%x %x %x %x", cardstr[0], cardstr[1], cardstr[2], cardstr[3]);
				  UID[0] = cardstr[0];
				  UID[1] = cardstr[1];
				  UID[2] = cardstr[2];
				  UID[3] = cardstr[3];
				  UID[4] = cardstr[4];

				  stcnt[2]++;
				  //DWT_Delay_ms(1);
				  HAL_Delay(1);
				  status_522 = MFRC522_SelectTag(cardstr);
				  if (status_522 > 0){
					  result++;
					  //
					  SectorKey[0] = ((Mx1[0][0])^(UID[0])) + ((Mx1[0][1])^(UID[1])) + ((Mx1[0][2])^(UID[2])) + ((Mx1[0][3])^(UID[3]));// 0x11; //KeyA[0]
					  SectorKey[1] = ((Mx1[1][0])^(UID[0])) + ((Mx1[1][1])^(UID[1])) + ((Mx1[1][2])^(UID[2])) + ((Mx1[1][3])^(UID[3]));// 0x11; //KeyA[0]
					  SectorKey[2] = ((Mx1[2][0])^(UID[0])) + ((Mx1[2][1])^(UID[1])) + ((Mx1[2][2])^(UID[2])) + ((Mx1[2][3])^(UID[3]));// 0x11; //KeyA[0]
					  SectorKey[3] = ((Mx1[3][0])^(UID[0])) + ((Mx1[3][1])^(UID[1])) + ((Mx1[3][2])^(UID[2])) + ((Mx1[3][3])^(UID[3]));// 0x11; //KeyA[0]
					  SectorKey[4] = ((Mx1[4][0])^(UID[0])) + ((Mx1[4][1])^(UID[1])) + ((Mx1[4][2])^(UID[2])) + ((Mx1[4][3])^(UID[3]));// 0x11; //KeyA[0]
					  SectorKey[5] = ((Mx1[5][0])^(UID[0])) + ((Mx1[5][1])^(UID[1])) + ((Mx1[5][2])^(UID[2])) + ((Mx1[5][3])^(UID[3]));// 0x11; //KeyA[0]
					  //DWT_Delay_ms(1);
					  HAL_Delay(1);
					  status_522 = MFRC522_Auth(0x60, 3, SectorKey, cardstr);
					  if (status_522 == MI_OK){
						  result++;
						  sprintf(str3, "Auth. OK");
//						  if (HAL_GPIO_ReadPin(Key2_GPIO_Port, Key2_Pin) == 0){
//							  // Clean-Up the Card:
//							  card_data[0] = 0xFF;
//							  card_data[1] = 0xFF;
//							  card_data[2] = 0xFF;
//							  card_data[3] = 0xFF;
//							  card_data[4] = 0xFF;
//							  card_data[5] = 0xFF;
//							  card_data[6] = 0xFF; //Access_bits[6]
//							  card_data[7] = 0x07; //Access_bits[7]
//							  card_data[8] = 0x80; //Access_bits[8]
//							  card_data[9] = 0x88; //user_byte[9]
//							  card_data[10] = 0x88; //user_byte[10]
//							  card_data[11] = 0x88; //user_byte[11]
//							  card_data[12] = 0x88; //user_byte[12]
//							  card_data[13] = 0x88; //user_byte[13]
//							  card_data[14] = 0x88; //user_byte[14]
//							  card_data[15] = 0x88; //user_byte[15]
//							  DWT_Delay_ms(1);
//							  status_522 = MFRC522_Write(3, card_data);
//							  if(status_522 == MI_OK) {
//								  result++;
//								  sprintf(str3, "                ");
//								  sprintf(str4, "Card Cleared!");
//								  delay_val = 2000;
//							  }
//
//						  }
					  }
					  else{
						  for (int i = 0; i < 16; i++) {cardstr[i] = 0;}
						  status_522 = 0;
						  // Find cards
						  //DWT_Delay_ms(1);
						  HAL_Delay(1);
						  status_522 = MFRC522_Request(PICC_REQIDL, cardstr);
						  //DWT_Delay_ms(1);
						  HAL_Delay(1);
						  status_522 = MFRC522_Anticoll(cardstr);
						  //DWT_Delay_ms(1);
						  HAL_Delay(1);
						  status_522 = MFRC522_SelectTag(cardstr);
						  SectorKey[0] = 0xFF;
						  SectorKey[1] = 0xFF;
						  SectorKey[2] = 0xFF;
						  SectorKey[3] = 0xFF;
						  SectorKey[4] = 0xFF;
						  SectorKey[5] = 0xFF;
						  //DWT_Delay_ms(1);
						  HAL_Delay(1);
						  status_522 = MFRC522_Auth(0x60, 3, SectorKey, cardstr);
//						  if (status == MI_OK){
//							  if (HAL_GPIO_ReadPin(Key1_GPIO_Port, Key1_Pin) == 0){
//								  card_data[0] = ((Mx1[0][0])^(UID[0])) + ((Mx1[0][1])^(UID[1])) + ((Mx1[0][2])^(UID[2])) + ((Mx1[0][3])^(UID[3]));// 0x11; //KeyA[0]
//								  card_data[1] = ((Mx1[1][0])^(UID[0])) + ((Mx1[1][1])^(UID[1])) + ((Mx1[1][2])^(UID[2])) + ((Mx1[1][3])^(UID[3]));// 0x11; //KeyA[0]
//								  card_data[2] = ((Mx1[2][0])^(UID[0])) + ((Mx1[2][1])^(UID[1])) + ((Mx1[2][2])^(UID[2])) + ((Mx1[2][3])^(UID[3]));// 0x11; //KeyA[0]
//								  card_data[3] = ((Mx1[3][0])^(UID[0])) + ((Mx1[3][1])^(UID[1])) + ((Mx1[3][2])^(UID[2])) + ((Mx1[3][3])^(UID[3]));// 0x11; //KeyA[0]
//								  card_data[4] = ((Mx1[4][0])^(UID[0])) + ((Mx1[4][1])^(UID[1])) + ((Mx1[4][2])^(UID[2])) + ((Mx1[4][3])^(UID[3]));// 0x11; //KeyA[0]
//								  card_data[5] = ((Mx1[5][0])^(UID[0])) + ((Mx1[5][1])^(UID[1])) + ((Mx1[5][2])^(UID[2])) + ((Mx1[5][3])^(UID[3]));// 0x11; //KeyA[0]
//								  card_data[6] = 0xFF; //Access_bits[6]
//								  card_data[7] = 0x07; //Access_bits[7]
//								  card_data[8] = 0x80; //Access_bits[8]
//								  card_data[9] = 0x88; //user_byte[9]
//								  card_data[10] = 0x88; //user_byte[10]
//								  card_data[11] = 0x88; //user_byte[11]
//								  card_data[12] = 0x88; //user_byte[12]
//								  card_data[13] = 0x88; //user_byte[13]
//								  card_data[14] = 0x88; //user_byte[14]
//								  card_data[15] = 0x88; //user_byte[15]
//								  DWT_Delay_ms(1);
//								  status_522 = MFRC522_Write(3, card_data);
//								  if(status_522 == MI_OK) {
//									  result++;
//									  sprintf(str3, "Card Set!");
//									  delay_val = 2000;
//								  }
//							  }
//							  else{
//
//								  sprintf(str4, "New Card!");
//							  }
//						  }
//						  else if (status_522  != MI_OK){
//							  sprintf(str3, "Auth. Error");
//						  }
//					  }
					  //DWT_Delay_ms(1);
					  HAL_Delay(1);
					  MFRC522_StopCrypto1();
				  }
			  }
			  //DWT_Delay_ms(1);
			  HAL_Delay(1);
			  MFRC522_Halt();
//			  LED2_GPIO_Port -> BRR = LED2_Pin;

			  //DWT_Delay_ms(delay_val);
			  HAL_Delay(delay_val);
			  delay_val = 1000;
			  sprintf(str1, "                ");
			  sprintf(str2, "                ");
			  sprintf(str3, "                ");
			  sprintf(str4, "                ");
		  }
		  else{
			  sprintf(str1, "Waiting for Card");
//			  lcd_gotoxy(0,0);
//			  lcd_puts(str1);
//			  sprintf(str2, "                ");
//			  lcd_gotoxy(0,1);
//			  lcd_puts(str2);
//			  sprintf(str3, "                ");
//			  lcd_gotoxy(0,2);
//			  lcd_puts(str3);
//			  sprintf(str4, "                ");
//			  lcd_gotoxy(0,3);
//			  lcd_puts(str4);
		  }

	  }
	  } //// timestamp_two loop
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
  sTime.Minutes = 0x30;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_SET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_NOVEMBER;
  sDate.Date = 0x12;
  sDate.Year = 0x22;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the reference Clock input
  */
  if (HAL_RTCEx_SetRefClock(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 479;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(void)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RC522_Rst_GPIO_Port, RC522_Rst_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RC522_Rst_Pin LD2_Pin */
  GPIO_InitStruct.Pin = RC522_Rst_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint64_t micros()
{return _micros + htim17.Instance->CNT;}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
 if(htim == &htim17)
 {_micros += 65535;}
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
