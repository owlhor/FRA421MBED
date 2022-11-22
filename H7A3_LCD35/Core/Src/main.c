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
#include "LCDrivers/Fonts/fonts.h"
#include "LCDrivers/ili9486.h"
#include "testimg.h"

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

TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint16_t baaa[5] = {0};
uint8_t bluecounter = 0;
uint8_t flag_blue = 0;

static uint8_t ff = 0;

////// for fontwrite test
union {
	uint8_t b8[4];
	uint32_t b32;
}buu32;
uint8_t rowbox;
uint16_t chpos;
uint32_t bfpos = 0;
uint32_t hop = 0;
uint32_t cliff = 0;

char txtbuf[20] = {0};
////// for fontwrite test


uint32_t timestamp_one = 0;
uint32_t timestamp_two = 0;

uint64_t _micros;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
uint64_t micros();
void ili_screen_1();
void ili_fonttest(uint16_t Xpo, uint16_t Ypo, char *chr, sFONT fonto, uint16_t RGB_Coder);
void ili_texttest(uint16_t Xpo, uint16_t Ypo,const char* strr,sFONT fonto, uint16_t RGB_Coder, uint16_t RGB_bg);
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
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  //HAL_TIM_Base_Start_IT(&htim17);
  ili9486_Init();
  ili9486_DisplayOn();

  BSP_LCD_Init();
  BSP_LCD_DisplayOn();

  baaa[0] = ili9486_GetLcdPixelWidth();
  baaa[1] = ili9486_GetLcdPixelHeight();
  baaa[2] = ili9486_ReadID();
  ili9486_FillRect(0, 0, 480, 320, 0x0000);
  //// force start testfont screen 3

  //flag_blue = 3;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(HAL_GetTick() - timestamp_one >= 3000){
		  timestamp_one = HAL_GetTick();
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  }


	  if(HAL_GetTick() - timestamp_two >= 20){
		  timestamp_two = HAL_GetTick();
		  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	  }

	  if(flag_blue == 4){
		  BSP_LCD_Clear(0xff00);
		  BSP_LCD_DrawRect(50, 50, 50, 50);
		  flag_blue = 0;
	  }

	  if(flag_blue == 3){
	  		  ili9486_FillRect(0, 0, 480, 320, 0x0000);
	  		  ili9486_DrawRGBImage(50, 100, 128, 128, (uint16_t*)image_data_ImageoftestN2);
	  		  ili9486_DrawRGBImage(300, 100, 128, 90, (uint16_t*)image_data_owlflood);


	  		  for(int i = 0; i < 40; i = i + 2){
	  			  for(int j = 0; j < 60; j = j + 4)
	  			ili9486_WritePixel(210 + i, 80 + j, cl_GREEN);
	  		  }

	  		  ili9486_WriteChar(20, 50, "E", Font8, cl_RED, cl_GREEN);
			  ili9486_WriteChar(50, 50, "E", Font12, cl_WHITE, cl_BLACK);
			  ili9486_WriteChar(80, 50, "E", Font16, cl_BLUE, cl_CYAN);
			  ili9486_WriteChar(110, 50, "E", Font20, cl_WHITE, cl_RED);
			  ili9486_WriteChar(140, 50, "E", Font24, cl_ORANGE, cl_BLACK);

			  //ili9486_FillRect(198, 30, 2, 30, cl_YELLOW);
			  ili9486_DrawVLine(cl_YELLOW, 200, 40, 24);
			  ili9486_DrawVLine(cl_YELLOW, 225, 40, 24);
			  //// Font24 @2664 E
			  ili_fonttest(200, 40, "A", Font24, cl_WHITE);
			  ili_fonttest(225, 40, "B", Font24, cl_WHITE);
			  ili_fonttest(250, 40, "C", Font24, cl_WHITE);
			  ili_fonttest(275, 40, "D", Font24, cl_WHITE);

			  ili_fonttest(300, 40, "A", Font8, cl_WHITE);
			  ili_fonttest(325, 40, "B", Font12, cl_WHITE);
			  ili_fonttest(350, 40, "C", Font16, cl_WHITE);
			  ili_fonttest(375, 40, "D", Font20, cl_WHITE);

			  ili9486_WriteString(20, 300, "KaleAR Terra", Font20, cl_WHITE, cl_BLACK);
			  ili_texttest(200, 200, "Helios Terra Renai Kaliber Barx Maxon 129035"
					  " __ --== + &&6.. [ ggg ]??? Rhivalia DIAR Barvarrian"
					  " vicar nexus iICCTVS \ / %%% $ *(!@#$%^&*)_{} "
					  , Font20, cl_GREEN, cl_BLACK);
			  char* aa = "A";
			  txtbuf[14] = *aa;

			  ili_fonttest(400, 30, aa, Font24, cl_WHITE); // A  <- output
			  ili_fonttest(420, 30, *aa, Font24, cl_WHITE); // N  <- output

	  		  flag_blue = 0; // comment this to forever loop
	  	  }

	  if(flag_blue == 1){
		  ili_screen_1();
		  flag_blue = 0;

	  }

	  if(flag_blue == 2){
		  //// Running Box ////
		  if(ff == 1){
			  ili9486_FillRect(0, 0, 480, 320, 0xF792); // screen
			  ff = 0;
		  }
		  int ratte = 1;
		  int sizo = 30;
		  int offs = 140;
		  static uint16_t xsh = 0;
		  ili9486_FillRect(xsh, offs, ratte ,sizo, 0xF792);
		  xsh += ratte;
		  ili9486_FillRect(xsh, offs, sizo, sizo, 0x0410); //// box
		  if(xsh >= 480){ // clear
			  ili9486_FillRect(xsh, offs, sizo, sizo, 0xF792);
			  xsh = 0;
		  }
		  //HAL_Delay(10);

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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /*AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 70;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim17.Init.Prescaler = 279;
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
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_N_Pin USB_FS_P_Pin */
  GPIO_InitStruct.Pin = USB_FS_N_Pin|USB_FS_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void ili_screen_1(){
	  ili9486_FillRect(0, 0, 480, 320, 0xffff);  // White BG

	  ili9486_FillRect(30, 30, 100, 100, 0xF8E0); // Red
	  ili9486_FillRect(150, 30, 100, 100, 0x07E0); // Green RGB565
	  ili9486_FillRect(270, 30, 100, 100, 0x001F); // Blue

	  ili9486_FillRect(30, 160,  100, 100, cl_CYAN); // C0x07FF
	  ili9486_FillRect(150, 160, 100, 100, cl_MAGENTA); // M 0xF81F
	  ili9486_FillRect(270, 160, 100, 100, cl_YELLOW); // Y0xFFE0
	  ili9486_FillRect(390, 30, 70, 230, cl_BLACK); // K
}

void ili_fonttest(uint16_t Xpo, uint16_t Ypo, char *chr,sFONT fonto, uint16_t RGB_Coder){

	rowbox = ceilf((float)(fonto.Width) / 8);
	//// choose MSB check pos for each font size
	cliff = 0x80 << (8 * (rowbox - 1));
	//rowbox = (fonto.Width / 8) + 1;

	for(int i = 0; i < fonto.Height; i++){
		//b = fonto.table[((chr - 31) * fonto.Height * rowbox)+ (i*rowbox)];
		chpos = (int)(*chr) - 32;
		bfpos = ((int)(*chr) - 32) * fonto.Height * rowbox;

		hop = 0;
		for(int k = 0; k < rowbox; k++){
			//// Works
			buu32.b8[k] = fonto.table[((int)(*chr - 32) * fonto.Height * rowbox) + (i * rowbox) + k];
			hop = (hop << 8) + buu32.b8[k];
			//HAL_Delay(30);
			//// how to insert in union
			//buu32.b8[rowbox - k] = fonto.table[((int)(*chr - 32) * fonto.Height * rowbox) + (i * rowbox) + k];
		}

		for(int j = 0; j < fonto.Width; j++){
			////Works
			if((hop << j) & cliff){ // buu32.b32

			//if((buu32.b32 << j) & cliff){ //
				ili9486_WritePixel(Xpo + j, Ypo + i, RGB_Coder);
			}

		}
	}
}

void ili_texttest(uint16_t Xpo, uint16_t Ypo,const char* strr,sFONT fonto, uint16_t RGB_Coder, uint16_t RGB_bg){

	uint16_t ili_heigh = ili9486_GetLcdPixelHeight();
	uint16_t ili_width = ili9486_GetLcdPixelWidth();

	while(*strr){

	//// Check screen overflow / new line
		if(Xpo + fonto.Width >= ili_width){
			Xpo = 0;
			Ypo += fonto.Height;

			if(Ypo + fonto.Height >= ili_heigh){
				break;
			}

			if(*strr == ' ') {
				// skip spaces in the beginning of the new line
				strr++;
				continue;
			}
		}
		//ST7735_WriteChar(x, y, *str, font, color, bgcolor);
		static int nummm = 0;
		txtbuf[nummm] = *strr;
		ili9486_WriteChar(Xpo, Ypo, strr, fonto, RGB_Coder, RGB_bg);
		Xpo += fonto.Width;
		strr++; nummm++;
	}
}

uint64_t micros()
{return _micros + htim17.Instance->CNT;}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		bluecounter++;
		bluecounter%=4;
		flag_blue = bluecounter;
		ff = 1;
	}

}

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
