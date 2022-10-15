/*
 * ST7735S_LCD.c
 *
 *  Created on: Oct 14, 2022
 *      Author: owl_hor
 *      Reference: AlphaP
 */
#include "ST7735S_LCD.h"

void LCD_CMD_SPI_set();
void LCD_DATA_SPI_set();

uint8_t Framememory[LCD_BUFFER_SIZE]={0};
uint8_t LCDSTARTUPSeq[]=
{
		//// Init Command in hex form
		0x01,						//SW Reset
		0x11,						//Sleep Out
		0x29,						//Display on
		0x36, 0b10001000,			//seting Scan Order // MY MX MV ML RGB MH _ _
		0x2a,0x00,0x00,0x00,127,	//Set Column Area [0,127] Pixel
		0x2b,0x00,0x00,0x00,127,	//Set Row Area    [0,127] Pixel
		0x2c						//Write Memory
		//// Then, DMA data to display
};

void LCD_init(LCDHandle *lcd){
	////reset lcd
	HAL_GPIO_WritePin(lcd->RstPort, lcd->RstPin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(lcd->RstPort, lcd->RstPin, GPIO_PIN_SET);
	/// SPI Pin Select
	HAL_GPIO_WritePin(lcd->CSPort, lcd->CSPin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(lcd->CSPort, lcd->CSPin, GPIO_PIN_RESET);


	//// Write Reset, Sleepout display on scan order
	//HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_RESET);
	LCD_CMD_SPI_set(lcd);
	HAL_SPI_Transmit(lcd->hspi, LCDSTARTUPSeq, 4, 100);

	//// Write param of Scan order
	//HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_SET);
	LCD_DATA_SPI_set(lcd);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[4], 1, 100);

	////CMD => Set C area
	//HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_RESET);
	LCD_CMD_SPI_set(lcd);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[5], 1, 100);

	////DATA => Set C area
	//HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_SET);
	LCD_DATA_SPI_set(lcd);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[6], 4, 100);

	////CMD => Set R area
	//HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_RESET);
	LCD_CMD_SPI_set(lcd);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[10], 1, 100);

	////DATA => Set R area
	//HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_SET);
	LCD_DATA_SPI_set(lcd);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[11], 4, 100);

	////CMD => Write to graphic memory
	//HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_RESET);
	LCD_CMD_SPI_set(lcd);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[15], 1, 100);

	////DATA => Set prepare image data.
	//HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_SET);
	LCD_DATA_SPI_set(lcd);

}

void LCD_flush(LCDHandle *lcd)
{
	//// Circular flush DMA
	HAL_SPI_Transmit_DMA(lcd->hspi, Framememory, LCD_BUFFER_SIZE);
}

uint8_t* LCDBufferAddr()
{
	return Framememory;
}

void LCD_CMD_SPI_set(LCDHandle *lcd){
	//Write D/CX, A0 pin
	// 0 => Control, Command address sent
	// 1 => Data sent
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_RESET);
}

void LCD_DATA_SPI_set(LCDHandle *lcd){
	//Write D/CX, A0 pin
	// 0 => Control, Command address sent
	// 1 => Data sent
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_SET);
}
