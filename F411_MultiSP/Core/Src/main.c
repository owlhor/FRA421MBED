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
#include "stdio.h"
#include "strings.h"
#include "string.h"
#include "MCP320X.h"
#include "DS_RTC.h"

//#include "personalINFO/persona_2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define dynamix_WRKS

//#define INA219_WRK
#define EXT_WWDG_TGGR
//#define MCP3002_WRK
//#define MCP3202_8_WRK
#define DS_RTC_WRK
//#define ILI9486_WRK
#define BMI270_wrk
#define ADC_DMA_wrk
#define ConsoUART_ANSI

#define INA219_ADDR 0b10000000
#define INA219_CRNT 0x04

#define BMI270_ADDR 0b1101000 << 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
//// ============= UART ================
char TxDataBuffer[32] =
{ 0 };
char RxDataBuffer[32] =
{ 0 };
char UIBuffex[50] =
{ 0 };

char ofcirc_ptrn_txt[] = "\r\n"
" ------------------ OFCIRC ------------------------\r\n"
"    ____   _______ ______       _________   ______ \r\n"
"   / __ \\ |______// ____/ |\\  |\\\\____   /  / ____/ \r\n"
"  | |  | | _____ | |      | | | |   /  /  | |      \r\n"
"  | |  | ||  __/ | |      | | | |  /  /   | |      \r\n"
"  | |__| || |    | |____  | | | |  \\  \\   | |____  \r\n"
"   \\____/ |__\\    \\_____\\  \\|  \\|   \\__\\   \\_____\\ \r\n"
" ===== TRACE TO THE FUTURE | BEYOND THE LIMIT =====   \r\n"
;

char ofcirc_ptrn3L_txt[] = "\r\n"
		" __________________________\r\n"
		"   __  ___  __     ___  __\r\n"
		"  /  \\ __  /    | |  / /  \r\n"
		"  \\__/ |   \\__  | |  \\ \\__\r\n"
		" __________________________\r\n"
;


uint8_t flag_spi2_read = 0;

typedef union _MCP3002_SET{
	struct MCP
	{
		uint16_t bitread :12;
		uint16_t reserv :2; // LSB
		enum _REQFigu{Mm_Diff_01,Mm_Diff_10,Mm_SE_CH0,Mm_SE_CH1} REQFigu :2; // MSB
	}MCP3002_8;
	uint16_t U16;
}MCP3002_SET;

MCP3002_SET MCrq1;
MCP3002_SET MCrq2;
MCP3002_SET MCread1;
MCP3002_SET MCread2;

MCP3202_SET mc2rq1;
MCP3202_SET mc2read1;

MCP3208_SET mc8rq1;
MCP3208_SET mc8read1;


union {
	uint8_t U8[4];
	uint16_t U16[2];
	uint32_t U32;
} MCPSHF = {0};

uint8_t D8x_MOSI[3];
uint8_t D8x_MISO[3];

uint16_t A_bitread = 0;
uint16_t AA_bitread = 0;
float VADC_c = 0.0;
float VADC_cv = 0.0;

////=======================================Dynamix======================================================
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

//////// External_RTC /////////////////////////////////////
DS3231_RG_Strc eRTC;
//// INA219 //////////////////////////////////////////////////////////////////////////////////////////
union {
	uint8_t U8[10];
	uint16_t U16[5];
} inabuf = {0};

uint8_t bmi_read[20];

////// ADC DMA ////////////////////
struct _ADCread{
	uint16_t Buf[3];
	uint32_t RawRead[300];
}adcr;


uint8_t flag_dyna = 0;
uint16_t counter = 0;
uint8_t wdg_tig = 0;
uint32_t timestamp_one[5] = {0};
//uint32_t timestamp_two = 0;
//uint32_t timestamp_wdg = 0;
//uint32_t timestamp_ds = 0;
//uint32_t timestamp_dsurt = 0;
uint64_t _micro;

struct _timestamp{
	uint32_t one;
	uint32_t two;

	uint32_t ds;
	uint32_t dsurt;

	uint32_t wdg;

	uint32_t ina;
	uint32_t bmi;
}
timestamp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void MCP3002_READ(uint16_t pTrX, uint16_t *pRcX);
void MCP3208_RrEAD(uint8_t *pTrX, uint8_t *pRcX);
//void MCP3208_RwEAD(uint16_t *pTrX, uint16_t *pRcX);
uint16_t MCP3208_READ_8_DataSPI_test(SPI_HandleTypeDef *hspi, MCP3208CHSelect M8_channel);
uint64_t micros();
int16_t UARTRecieveIT();
void dynamix_enable_torque();

void HUART_TX(char *chartex);
void ConsoleUARTPlay();
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
  MX_SPI2_Init();
  MX_TIM11_Init();
  MX_USART6_UART_Init();
  MX_SPI3_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim11);
  //HAL_TIM_Base_Start_IT(&htim3);

  MCrq1.MCP3002_8.REQFigu = Mm_Diff_01;
  MCrq2.MCP3002_8.REQFigu = Mm_SE_CH1;



  //mc8rq1.MCP3208_U.CHSlct = M8_CH0;
  mc8rq1.MCP3208_DI_16.CHSlct = M8_CH0;
  //HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  	HAL_UART_Transmit(&huart2, (uint8_t*)ofcirc_ptrn_txt, strlen(ofcirc_ptrn_txt), 100);

  	HAL_UART_Transmit(&huart2, (uint8_t*)ofcirc_ptrn3L_txt, strlen(ofcirc_ptrn_txt), 100);

    char temp[]="----------------- F411_MultiSP --------------------\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)temp, strlen(temp),10);

    ConsoleUARTPlay();
    HAL_Delay(1000);



#ifdef dynamix_WRKS
  dyna_01.dymix.header1 = 0xff;
  dyna_01.dymix.header2 = 0xff;
  dyna_01.dymix.DXL_ID = 0x03;
  dyna_01.dymix.Instruct = write;

  dynamix_enable_torque();
#endif

#ifdef ILI9486_WRK
  	 ili9486_Init();
     ili9486_DisplayOn();

     ili_scr_1();

     ili9486_FillRect(0, 0, 480, 35, cl_BLUE);
     ili9486_WriteStringNoBG(10, 10, " > PWMon ----->>>-----", Font20, cl_WHITE);
     ili9486_WriteStringNoBG(400, 10, " OWL_HOR ", Font12, cl_WHITE);

     ili9486_WriteStringNoBG(10, 40, " INA219 PWRMon Quick", Font20, cl_OLIVE);
#endif

#ifdef ADC_DMA_wrk
     HAL_ADC_Start_DMA(&hadc1, &adcr.RawRead[0], 300);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_UART_Receive_IT(&huart6,  (uint8_t*)RxDataBuffer, 32);

#ifdef ADC_DMA_wrk
     adcr.Buf[0] = adcr.RawRead[0];
     adcr.Buf[1] = adcr.RawRead[1];
     adcr.Buf[2] = adcr.RawRead[2];
#endif



#ifdef DS_RTC_WRK
	  if(HAL_GetTick() >= timestamp.ds ){ ////&& wdg_tig == 0
			  timestamp.ds = HAL_GetTick() + 2;
			  DS3231_Read_v(&hi2c1, &eRTC.D8[0]);
		  }

	  if(HAL_GetTick() >= timestamp.dsurt ){ ////&& wdg_tig == 0
	  		timestamp.dsurt = HAL_GetTick() + 1000 ;

#ifdef ConsoUART_ANSI
     HUART_TX("\x1b[2J \x1b[H"); //clear screen + start head
     HAL_UART_Transmit(&huart2, (uint8_t*)ofcirc_ptrn3L_txt, strlen(ofcirc_ptrn3L_txt), 200); // enough timeout

     sprintf(TxDataBuffer, "Timestamp dsurt : %lu \r\n ", timestamp.dsurt);
     HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer),10);
#endif


	  		sprintf(TxDataBuffer, "Time - %2X : %2X : %2X\r\n ", eRTC.DS3231RG.hour, eRTC.DS3231RG.min, eRTC.DS3231RG.sec);
	  		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer),10);

	  }
#endif

#ifdef EXT_WWDG_TGGR
	  //// External Watchdog
	  if(HAL_GetTick()- timestamp.wdg >= 1400 ){ ////&& wdg_tig == 0
	  		  timestamp.wdg = HAL_GetTick();
	  		  // Toggle edge to watchdog xternal ic
	  		  HAL_GPIO_TogglePin(WDG_TG_GPIO_Port, WDG_TG_Pin);
	  		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  	  }
#endif



#ifdef MCP3202_8_WRK
	  if(HAL_GetTick() >= timestamp_one[1]){
		  timestamp_one[1] += 500;
		  flag_spi2_read = 1;
	  }

	  if (flag_spi2_read != 0 && hspi3.State == HAL_SPI_STATE_READY && HAL_GPIO_ReadPin(M8_CS_GPIO_Port, M8_CS_Pin) == GPIO_PIN_SET)
		{
		  //MCP3202_READ(&hspi2, mc2rq1.U16, mc2read1.U16);
		  //VADC_cv = (mc2rq1.MCP3202_U.bitread << 1) * 0.00120;

		  //// Shitty bitshift to the correct position Fig 6-1, MCP3208, MICROCHIP
//		  MCPSHF.U8[0] = (0x0000 | mc8rq1.MCP3208_DI_16.CHSlct) >> 2;
//		  MCPSHF.U8[1] = (0x0000 | mc8rq1.MCP3208_DI_16.CHSlct) << 6;
		  //MCP3208_RrEAD(&MCPSHF.U8[0], &mc8read1.U8[0]);

		  //MCP3208_RrEAD(&mc8rq1.U8[0], &mc8read1.U8[0]);  // read work but bitshift uncorrect
		  //MCP3208_RwEAD(&mc8rq1.U16[0], &mc8read1.U16[0]);

		  //uint16_t bitredd = (((mc8read1.U8[1] << 8) + mc8read1.U8[2]) & 0x0FFF); //// for 8 clk
		  //A_bitread = (((mc8read1.U8[0] & 0x0F)<<8) + mc8read1.U8[3]); //// for 16 clk

		  AA_bitread = MCP3208_READ_8_DataSPI(&hspi3, M8_CH0);
		  VADC_cv =  MCP320x_ADCbit_to_Volt(AA_bitread); // 5 / 4096 * 0.00122

		  A_bitread = MCP3202_READ_8_DataSPI(&hspi2, M2_SE_CH0);
		  VADC_c = MCP320x_ADCbit_to_Volt(A_bitread);


		  //// UART Send
		  sprintf(TxDataBuffer, "VADC = %d %.3f _ %d %.3f \r\n ", AA_bitread, (float)VADC_cv, A_bitread, (float)VADC_c); //mc8read1.MCP3208_U.bitread
		  HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer),10);

		  flag_spi2_read = 0;
		}

#endif

#ifdef MCP3002_WRK
	  ////========SPI MCP3002 ========================================
	  if(HAL_GetTick()- timestamp_one[0] >= 500){
		  timestamp_one[0] = HAL_GetTick();
		  flag_spi2_read = 1;
	  }

	  if (flag_spi2_read != 0 && hspi2.State == HAL_SPI_STATE_READY
							&& HAL_GPIO_ReadPin(M2_CS_GPIO_Port, M2_CS_Pin)
									== GPIO_PIN_SET)
		{

		  counter++;
		  //MCP3002_READ(MCrq1.U16, &MCread1.U16); //&A_bitread
		  if(counter%2==0){
			  MCP3002_READ(MCrq1.U16, &MCread1.U16); //&A_bitread
		  }
		  else{
			  //MCP3002_READ(MCrq1.U16, &MCrq1.U16); //&A_bitread
		  }


		  //// Dout = ( 4096 x Vin )/ VCC
		  //// Dout x VCC / 4096 = Vin
		  //// << 1 to add 1 lost LSB
		  VADC_c = (MCread1.MCP3002_8.bitread << 1) * 0.00120; //// 1/4096 *5 = 5 * 0.000244140625

		  sprintf(TxDataBuffer, "VADC %x \r\n",  MCread1.MCP3002_8.bitread);
		  HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer),10);

		  flag_spi2_read = 0;
		}
#endif

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


#ifdef INA219_WRK
	  if (HAL_GetTick() - timestamp.ina >= 1000){
		  timestamp.ina = HAL_GetTick();
		  uint8_t memaddr = 0x00;

		  HAL_I2C_Mem_Read(&hi2c1, INA219_ADDR, memaddr, I2C_MEMADD_SIZE_8BIT, &inabuf.U8[0], 8, 100);
	  }
#endif

#ifdef BMI270_wrk
	  if (HAL_GetTick() - timestamp.bmi >= 1000){
	  		  timestamp.bmi = HAL_GetTick();

	  		  HAL_I2C_Mem_Read(&hi2c1, BMI270_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &bmi_read[0], 18, 100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
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
  HAL_GPIO_WritePin(GPIOA, CC_I2_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WDG_TG_GPIO_Port, WDG_TG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, M2_CS_Pin|M8_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CC_I2_Pin */
  GPIO_InitStruct.Pin = CC_I2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CC_I2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin M2_CS_Pin M8_CS_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|M2_CS_Pin|M8_CS_Pin;
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
	HAL_GPIO_WritePin(M2_CS_GPIO_Port, M2_CS_Pin, GPIO_PIN_RESET);
	//HAL_SPI_Transmit_IT(&hspi2, &datain, 1);
	//HAL_SPI_TransmitReceive(&hspi2, pTrX, &pRcX, 1, 100);

	//HAL_SPI_TransmitReceive_IT(&hspi2, &pTrX, pRcX, 1);

	//// Dout = ( 4096 x Vin )/ VCC
	//// Dout x VCC / 4096 = Vin

}

void MCP3208_RrEAD(uint8_t *pTrX, uint8_t *pRcX){

	//uint16_t datain = pTrX; //0b1100000000000000
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

	//HAL_SPI_Transmit_IT(&hspi2, &datain, 1);
	//HAL_SPI_TransmitReceive(&hspi2, pTrX, &pRcX, 1, 100);

	//HAL_SPI_TransmitReceive_IT(&hspi3, &pTrX, pRcX, 3);
	HAL_SPI_TransmitReceive_IT(&hspi3, pTrX, pRcX, 3);

	//// Dout = ( 4096 x Vin )/ VCC
	//// Dout x VCC / 4096 = Vin

}

//void MCP3208_RwEAD(uint16_t *pTrX, uint16_t *pRcX){
//
//	//uint16_t datain = pTrX; //0b1100000000000000
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
//
//	//HAL_SPI_Transmit_IT(&hspi2, &datain, 1);
//	//HAL_SPI_TransmitReceive(&hspi2, pTrX, &pRcX, 1, 100);
//
//	HAL_SPI_TransmitReceive_IT(&hspi3, pTrX, pRcX, 2);
//
//
//}

void ConsoleUARTPlay(){

	HUART_TX("\x1b[2J \x1b[H"); // clear screen  + Move Home

	HUART_TX(ofcirc_ptrn3L_txt);

	HUART_TX(" --- ANSI Escape Command --- \r\n");
	HUART_TX("\x1b[01m BOLD \r\n");
	HUART_TX("\x1b[00m");
	HUART_TX("\x1b[03m ITALIC \r\n");
	HUART_TX("\x1b[00m");
	HUART_TX("\x1b[04m UNDERLINE \r\n");
	HUART_TX("\x1b[00m");

	HUART_TX("\x1b[31m RED \r\n");
	HUART_TX("\x1b[32m GREEN \r\n");
	HUART_TX("\x1b[33m YELOW \r\n");
	HUART_TX("\x1b[34m BLUE \r\n");
	HUART_TX("\x1b[35m MAGENTA \r\n");
	HUART_TX("\x1b[36m CYAN \r\n");
	HUART_TX("\x1b[37m WHITE \r\n");
	HUART_TX("");


	HUART_TX("\x1b[00m RESET ANSI--- \r\n");

	HAL_Delay(1000);

	//// clear text in line + start at first pos
	HUART_TX("Starting"); HAL_Delay(600); HUART_TX("\x1b[2K \x1b[0G");
	HUART_TX("Starting ."); HAL_Delay(600); HUART_TX("\x1b[2K \x1b[0G");
	HUART_TX("Starting . ."); HAL_Delay(600); HUART_TX("\x1b[2K \x1b[0G");
	HUART_TX("Starting . . ."); HAL_Delay(600); HUART_TX("\r\n");

	HUART_TX("Ready ?"); HAL_Delay(1000); HUART_TX("\x1b[2K \x1b[0G");
	HUART_TX("Set ---"); HAL_Delay(1000); HUART_TX("\x1b[2K \x1b[0G");
	HUART_TX("GOOO !!!"); HAL_Delay(1000); HUART_TX("\x1b[2K \x1b[0G");

}

void HUART_TX(char *chartex){
	sprintf(TxDataBuffer, chartex);
	HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer),50);
}


uint16_t MCP3208_READ_8_DataSPI_test(SPI_HandleTypeDef *hspi, MCP3208CHSelect M8_channel){

	//// Shitty bitshift to the correct position Fig 6-1, MCP3208, MICROCHIP

	D8x_MOSI[0] = M8_channel >> 2;
	D8x_MOSI[1] = M8_channel << 6;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive_IT(hspi, &D8x_MOSI[0], &D8x_MISO[0], 3);

	return ((D8x_MISO[1] << 8) + D8x_MISO[2]) & 0x0FFF;

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi2)
	{
		// set cs back to 1, finished
		HAL_GPIO_WritePin(M2_CS_GPIO_Port, M2_CS_Pin, GPIO_PIN_SET);
	}

//	if (hspi == &hspi3)
//		{
//		// set cs back to 1, finished
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
//		}
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
	if (htim == &htim3){
		HAL_GPIO_TogglePin(CC_I2_GPIO_Port, CC_I2_Pin);
	}
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
