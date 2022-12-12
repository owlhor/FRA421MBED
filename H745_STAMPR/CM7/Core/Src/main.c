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
#include "libjpeg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../../Common/Src/SRAM4.h"
//#include "../../Drivers/LCDrivers/lcd/ili9486/ili9486.h"
#include "LCDrivers/Fonts/fonts.h"
#include "LCDrivers/ili9486.h"
#include "testimg.h"
#include "personalINFO/persona_2.h"

/* USER CODE END Includes */

/* Private typedef ---------------------- -------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

#define k_tim_show_milli 5000
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

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

//// buffer for UART send
char txtdispBF[120] = {0};
//// RealTime Clock Handler
RTC_TimeTypeDef NowTim7;
RTC_DateTypeDef NowDat7;

uint32_t timestamp_one = 0;
uint32_t timestamp_grandis[2] = {0};

//// Grand State
static enum{st_lobby, st_search, st_show, st_waitend} GranDiSTATE = st_lobby;

// number of data in dataset which match the scanned card, -1 means no ones match
int8_t px_ID_match = -1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */
void ili_scr_1();
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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_DMA_Init();
  MX_LIBJPEG_Init();
  /* USER CODE BEGIN 2 */

  	SRAM4-> flag_UID = 0;

  	ili9486_Init();
    ili9486_DisplayOn();

    ili_scr_1();

    ili9486_FillRect(0, 0, 480, 35, cl_BLUE);
    ili9486_WriteStringNoBG(10, 10, " > STAMPR ----->>>-----", Font20, cl_WHITE);
    ili9486_WriteStringNoBG(400, 10, " OWL_HOR ", Font12, cl_WHITE);
    ili9486_DrawRGBImage(140, 120, 128, 128, (uint16_t*)test_img_128x128);
    ili9486_WriteStringNoBG(10, 40, " Scan the RFID Tag", Font20, cl_OLIVE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //// Time Clock Manager / Independent from GranDiState
	  if(HAL_GetTick() - timestamp_one >= 500){
	  		  timestamp_one = HAL_GetTick();
	  		  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	  		NowDat7 = SRAM4->NowDates;
	  		NowTim7 = SRAM4->NowTimes;

	  		sprintf(txtdispBF, "%02X:%02X:%02X", // use %02 to fill 0 front of 1 digit
	  				  NowTim7.Hours, NowTim7.Minutes, NowTim7.Seconds);
	  		ili9486_WriteString(365, 40, txtdispBF, Font20, cl_YELLOW, cl_BLACK);

	  		sprintf(txtdispBF, "%02x/%02x/%02x",
	  			  				  NowDat7.Date, NowDat7.Month, NowDat7.Year);
	  		ili9486_WriteString(365, 60, txtdispBF, Font20, cl_YELLOW, cl_BLACK);
	  	  }

	  //// State Manager
	  if(HAL_GetTick() - timestamp_grandis[0] >= 100){
		  timestamp_grandis[0] = HAL_GetTick();

		  switch (GranDiSTATE){
		  default:
		  case st_lobby:
			  //// ---- wait for scanned cards----------------------------------------------
			  //if(HAL_HSEM_Take(2, 2) == HAL_OK){
			  if ( SRAM4-> flag_UID == 1){

				  //GranDiSTATE = st_show;
				  GranDiSTATE = st_search;
				  SRAM4-> flag_UID = 0;
				  //timestamp_grandis[1] = HAL_GetTick();
			  }
			  	//  HAL_HSEM_Release(2, 2);
			  	//}
			  break;

		  case st_search:
			  //// ---------------------------------------------------------
			  /* search match ID before show everything in st_show
			   *
			   * if SD Card, call All ID in SD keep in buffer to search
			   * then call pic data if found
			   * 1 - call all ID in database
			   * 2 - search from ID 0 to last
			   * 3 - search from UID[0] if not match -> go to next ID
			   *   - if not found any -> return n/a status and report no data in base
			   *   - if found -> break and end the search/ return personna -> go to st_show
			   * */
#ifdef px_ID_search_datasss
			  px_ID_match = -1; //// -1 means not found preferred
			  //// Breadth-First-Search cat cat
			  	 for(int y = 0;y <= PERSONA_LEN; y++){
			  		 if (SRAM4->UUID[0] == pxs_persons[y].USID[0]){
			  			 if (SRAM4->UUID[1] == pxs_persons[y].USID[1]){
			  				if (SRAM4->UUID[2] == pxs_persons[y].USID[2]){
			  					if (SRAM4->UUID[3] == pxs_persons[y].USID[3]){
			  						px_ID_match = y; //// return match ID
			  						break; //// end search
			  					}// search layer 3
			  				}// search layer 2
			  			 }// search layer 1
			  		 }// search layer 0

			  	 }// for loop search
#endif
			  /// ---------------Finally-------------------------------
			  GranDiSTATE = st_show;
			  timestamp_grandis[1] = HAL_GetTick();

			  break;

		  case st_show:
			  //// ------------------------------------------------------------------------------

			  // ID Show----------------------------
			  sprintf(txtdispBF,"Scanned ID");
			  ili9486_WriteString(160, 100, txtdispBF, Font20, cl_ORANGE, cl_BLACK);

			  sprintf(txtdispBF,"UID: %02X %02X %02X %02X",
					 SRAM4->UUID[0],SRAM4->UUID[1],SRAM4->UUID[2],SRAM4->UUID[3]);
			  ili9486_WriteString(160, 125, txtdispBF, Font20, cl_YELLOW, cl_BLACK);
			  // ID Show-----------------------------

			  ////// single dataset dummy only, it works
			  ///// activate persona_1.c before uncomment these
//			  if (SRAM4->UUID[0] == p1_owl.USID[0] &&
//				  SRAM4->UUID[1] == p1_owl.USID[1] &&
//				  SRAM4->UUID[2] == p1_owl.USID[2] &&
//				  SRAM4->UUID[3] == p1_owl.USID[3]){
//
//				  ili9486_DrawRGBImage(20, 100, 128, 128, (uint16_t*)p1_owl.pic);
//				  ili9486_WriteString(160, 150, p1_owl.Name, Font20, cl_GREEN, cl_BLACK);
//				  ili9486_WriteString(160, 175, p1_owl.Surname, Font20, cl_GREEN, cl_BLACK);
//				  ili9486_WriteString(160, 200, p1_owl.welcom_txt, Font16, cl_ORANGE, cl_BLACK);
//			  }
#ifdef px_ID_search_datasss
			  //// ---- show pic
			  if(px_ID_match == -1){
				  ////ili9486_DrawRGBImage(20, 100, 128, 128, (uint16_t*)p1_owl.pic);
				  ili9486_WriteString(160, 160,"NO ID IN DATABASE", Font24, cl_RED, cl_BLACK);

			  }else{
				  ili9486_DrawRGBImage(20, 100,
						  pxs_persons[px_ID_match].picXs,
						  pxs_persons[px_ID_match].picYs,
						  (uint16_t*)pxs_persons[px_ID_match].pic);
				  ili9486_WriteString(160, 150, pxs_persons[px_ID_match].Name, Font20, cl_GREEN, cl_BLACK);
				  ili9486_WriteString(160, 175, pxs_persons[px_ID_match].Surname, Font20, cl_GREEN, cl_BLACK);
				  ili9486_WriteString(160, 200, pxs_persons[px_ID_match].welcom_txt, Font16, cl_CYAN, cl_BLACK);
			  }
#endif
			  GranDiSTATE = st_waitend;

			  break;

		  case st_waitend:
			  /* Using waitend to wait, if still in show -> CPU will write display continuously
			   * */
			  ////// Ending display and back to lobby------------------
			  if(HAL_GetTick() - timestamp_grandis[1] >= k_tim_show_milli){
				  GranDiSTATE = st_lobby;
				  // clear Display
				  ili9486_FillRect(20, 100, 450, 200, cl_BLACK);

				  //// clear UID if nothing left in queue
				  if (SRAM4->flag_UID == 0){
				  SRAM4->UUID[0] = 0;
				  SRAM4->UUID[1] = 0;
				  SRAM4->UUID[2] = 0;
				  SRAM4->UUID[3] = 0;
				  }
			  }
			  break;

 		  } // switch
	  }// GrandState

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
void MX_RTC_Init(void)
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
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ili_scr_1(){

	  ili9486_FillRect(0, 0, 480, 320, cl_BLACK);

	  ili9486_FillRect(0, 300, 80, 20, cl_RED); // Red
	  ili9486_FillRect(80, 300, 80, 20, cl_GREEN); // Green RGB565
	  ili9486_FillRect(160, 300, 80, 20, cl_BLUE); // Blue
	  ili9486_FillRect(240, 300,  80, 20, cl_CYAN); // C0x07FF
	  ili9486_FillRect(320, 300, 80, 20, cl_MAGENTA); // M 0xF81F
	  ili9486_FillRect(400, 300, 80, 20, cl_YELLOW); // Y0xFFE0
	  //ili9486_FillRect(390, 30, 70, 230, cl_BLACK); // K
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
