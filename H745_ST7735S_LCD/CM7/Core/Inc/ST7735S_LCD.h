/*
 * ST7735S_LCD.h
 *
 *  Created on: Oct 14, 2022
 *      Author: owl_hor
 *      Reference: AlphaP
 */

#ifndef INC_ST7735S_LCD_H_
#define INC_ST7735S_LCD_H_

#include "stm32h7xx_hal.h"

typedef struct
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *RstPort,*CSPort,*DCPort;
	uint32_t  RstPin;
	uint32_t  CSPin;
	uint32_t  DCPin;

}LCDHandle;
#define LCD_BUFFER_SIZE 3*128*128 // (R+G+B) x Column x Row
extern uint8_t Framememory[LCD_BUFFER_SIZE]; //// Edit Framememory to change picture

void LCD_init(LCDHandle *lcd);
void LCD_flush(LCDHandle *lcd);
uint8_t* LCDBufferAddr();


#endif /* INC_ST7735S_LCD_H_ */
